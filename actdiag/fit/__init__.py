from __future__ import annotations

import copy
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Iterator

import pandas as pd
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
)

from actdiag.config import load_run_config, load_yaml_mapping, StepTestProfile
from actdiag.fit.config import load_search_config
from actdiag.logging_io import make_output_dir
from actdiag.fit.evaluator import (
    EvaluationResult,
    evaluate_sample,
    interpolate_reference,
    load_reference,
)
from actdiag.fit.sampler import sample_parameters
from actdiag.fit.reporter import save_fit_results
from actdiag.signals import build_signal_series


# ---------------------------------------------------------------------------
# Top-level worker — must be module-level so ProcessPoolExecutor can pickle it
# ---------------------------------------------------------------------------

def _fit_sample_worker(args: dict[str, Any]) -> EvaluationResult:
    """Evaluate one parameter sample and return the result."""
    return evaluate_sample(
        args["sample"],
        args["run_config"],
        args["reference_interpolated"],
        args["objective_weights"],
        reference_metrics=args["reference_metrics"],
    )


# ---------------------------------------------------------------------------
# Sequential / parallel dispatch
# ---------------------------------------------------------------------------

def _iter_fit_results(
    all_args: list[dict[str, Any]], workers: int
) -> Iterator[EvaluationResult]:
    if workers <= 1:
        for args in all_args:
            yield _fit_sample_worker(args)
    else:
        with ProcessPoolExecutor(max_workers=workers) as executor:
            futures = [executor.submit(_fit_sample_worker, args) for args in all_args]
            for future in as_completed(futures):
                yield future.result()


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_fit(
    system_path: Path,
    scenario_path: Path,
    reference_path: Path | None,
    search_path: Path,
    output_dir: Path | None = None,
    workers: int = 1,
) -> int:
    output_dir = make_output_dir(Path.cwd(), "fits", output_dir)

    # 1. Load configs
    run_config = load_run_config(system_path, scenario_path)
    system_data = load_yaml_mapping(system_path)
    fit_config = load_search_config(search_path)

    # Generate signals to get the simulation time grid and desired trajectory
    signals = build_signal_series(run_config.test, run_config.simulation)

    # 2. Load and prepare reference
    if reference_path is not None:
        reference_raw = load_reference(reference_path)
        reference_interpolated = interpolate_reference(reference_raw, signals.time)
    else:
        reference_interpolated = pd.DataFrame({
            "time": signals.time,
            "q": signals.q_des,
            "dq": signals.dq_des,
            "tau_applied": signals.tau_des,
        })

    reference_interpolated["q_des"] = signals.q_des
    reference_interpolated["dq_des"] = signals.dq_des
    reference_interpolated["tau_des"] = signals.tau_des

    # 3. Sample parameters
    samples = sample_parameters(fit_config)

    # Pre-compute reference metrics if it's a step test
    reference_metrics = None
    if isinstance(run_config.test, StepTestProfile):
        from actdiag.simulate import compute_step_response_metrics
        reference_metrics = compute_step_response_metrics(reference_interpolated, run_config.test)

    # 4. Pre-build one args dict per sample
    all_sample_args: list[dict[str, Any]] = [
        {
            "sample": sample,
            "run_config": run_config,
            "reference_interpolated": reference_interpolated,
            "objective_weights": fit_config.objective,
            "reference_metrics": reference_metrics,
        }
        for sample in samples
    ]

    # 5. Evaluation loop with Rich progress bar
    effective_workers = workers if workers > 0 else os.cpu_count() or 1
    worker_label = f" · {effective_workers} workers" if effective_workers > 1 else ""
    base_desc = f"Fitting {len(samples)} samples{worker_label}"

    results: list[EvaluationResult] = []
    best_result: EvaluationResult | None = None

    with Progress(
        TextColumn("[bold cyan]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        TimeRemainingColumn(),
        refresh_per_second=4,
    ) as progress:
        task_id = progress.add_task(base_desc, total=len(samples))

        for result in _iter_fit_results(all_sample_args, effective_workers):
            results.append(result)

            if best_result is None or result.total_cost < best_result.total_cost:
                best_result = result

            desc = base_desc
            if best_result is not None and best_result.total_cost < 1e9:
                params_str = "  ".join(
                    f"{k.split('.')[-1]}={v:.3g}"
                    for k, v in best_result.parameters.items()
                )
                desc = f"{base_desc}  │  best cost={best_result.total_cost:.4g} ({params_str})"
            progress.update(task_id, advance=1, description=desc)

    if best_result is None or best_result.total_cost >= 1e9:
        print("Error: All simulations failed.")
        return 1

    # 5. Build best-fit system config by patching the original system YAML
    best_system_data = _apply_params_to_dict(system_data, best_result.parameters)

    # 6. Save results
    save_fit_results(
        output_dir,
        results,
        best_result,
        run_config.plots,
        reference_interpolated,
        best_system_data,
    )

    print(f"Fit complete. Best cost: {best_result.total_cost:.6f}")
    print(f"Results saved to: {output_dir}")
    print(f"Best parameters: {best_result.parameters}")

    return 0


def _apply_params_to_dict(
    system_data: dict[str, Any], parameters: dict[str, float]
) -> dict[str, Any]:
    """Return a deep copy of system_data with each parameter path applied."""
    data = copy.deepcopy(system_data)
    for path, value in parameters.items():
        parts = path.split(".")
        node: Any = data
        for part in parts[:-1]:
            node = node[part]
        node[parts[-1]] = value
    return data
