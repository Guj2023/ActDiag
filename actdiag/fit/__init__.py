from __future__ import annotations

import copy
import itertools
import os
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED
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
    _failure_result,
    evaluate_sample,
    interpolate_reference,
    load_reference,
)
from actdiag.fit.sampler import sample_parameters
from actdiag.fit.reporter import save_fit_results
from actdiag.signals import build_signal_series


# ---------------------------------------------------------------------------
# Per-process shared storage — populated once by the worker initializer so
# the large read-only data (run_config, reference DataFrame, …) is pickled
# only N_workers times instead of N_samples times.
# ---------------------------------------------------------------------------

_WORKER_SHARED: dict = {}


def _worker_initializer(
    run_config: Any,
    reference_interpolated: pd.DataFrame,
    objective_weights: Any,
    reference_metrics: Any,
) -> None:
    """Called once per worker process before any tasks are dispatched."""
    _WORKER_SHARED["run_config"] = run_config
    _WORKER_SHARED["reference_interpolated"] = reference_interpolated
    _WORKER_SHARED["objective_weights"] = objective_weights
    _WORKER_SHARED["reference_metrics"] = reference_metrics


def _fit_sample_worker(sample: dict[str, float]) -> EvaluationResult:
    """Evaluate one parameter sample using the per-process shared data.

    Never raises — exceptions outside evaluate_sample's own try/except (e.g.
    a missing _WORKER_SHARED key, a deepcopy failure, or a backend that cannot
    initialise inside a spawned subprocess) are caught here and returned as a
    failure result so the progress loop always receives a valid object.
    """
    try:
        return evaluate_sample(
            sample,
            _WORKER_SHARED["run_config"],
            _WORKER_SHARED["reference_interpolated"],
            _WORKER_SHARED["objective_weights"],
            reference_metrics=_WORKER_SHARED["reference_metrics"],
        )
    except Exception as exc:
        return _failure_result(sample, error=f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# Sequential / parallel dispatch with bounded in-flight futures
# ---------------------------------------------------------------------------

def _iter_fit_results(
    samples: list[dict[str, float]],
    workers: int,
    run_config: Any,
    reference_interpolated: pd.DataFrame,
    objective_weights: Any,
    reference_metrics: Any,
) -> Iterator[EvaluationResult]:
    if workers <= 1:
        for sample in samples:
            yield evaluate_sample(
                sample,
                run_config,
                reference_interpolated,
                objective_weights,
                reference_metrics=reference_metrics,
            )
        return

    # Parallel path: shared data sent once per worker via initializer;
    # only the tiny sample dict is pickled per task.
    # At most (workers * 2) futures are in-flight at any time to keep the
    # executor queue from ballooning with N_samples serialised payloads.
    #
    # We use the "forkserver" start context when available (Linux/macOS).
    # Unlike "spawn" it re-uses a single already-initialised server process,
    # which avoids per-worker SDK re-initialisation bugs (e.g. PhysX) while
    # still being safer than raw "fork" for multithreaded code.
    import multiprocessing as _mp
    import sys as _sys
    _mp_ctx = (
        _mp.get_context("forkserver")
        if _sys.platform != "win32"
        else _mp.get_context("spawn")
    )
    max_inflight = workers * 2
    with ProcessPoolExecutor(
        max_workers=workers,
        mp_context=_mp_ctx,
        initializer=_worker_initializer,
        initargs=(run_config, reference_interpolated, objective_weights, reference_metrics),
    ) as executor:
        args_iter = iter(samples)
        pending: set = set()

        # Prime the pump up to the in-flight cap
        for sample in itertools.islice(args_iter, max_inflight):
            pending.add(executor.submit(_fit_sample_worker, sample))

        while pending:
            done, pending = wait(pending, return_when=FIRST_COMPLETED)
            for future in done:
                yield future.result()
                # Immediately replace each finished task with the next one
                next_sample = next(args_iter, None)
                if next_sample is not None:
                    pending.add(executor.submit(_fit_sample_worker, next_sample))


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

    # 4a. Sequential probe — run one sample in the main process before
    #     spawning workers.  This catches config errors (wrong parameter path,
    #     bad backend, missing library…) immediately with a clear message
    #     instead of silently producing 100 % failures later.
    probe = evaluate_sample(
        samples[0],
        run_config,
        reference_interpolated,
        fit_config.objective,
        reference_metrics=reference_metrics,
    )
    if probe.total_cost >= 1e9:
        msg = probe.error or "unknown error — check system/scenario/search configs"
        print(f"error: probe simulation failed: {msg}", file=__import__('sys').stderr)
        return 1

    # 4b. Evaluation loop with Rich progress bar
    effective_workers = workers if workers > 0 else os.cpu_count() or 1
    worker_label = f" · {effective_workers} workers" if effective_workers > 1 else ""
    base_desc = f"Fitting {len(samples)} samples{worker_label}"

    # cost_records stores only scalar fields — no timeseries DataFrames.
    # best_result is the single EvaluationResult that keeps a timeseries.
    cost_records: list[dict[str, Any]] = []
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

        for result in _iter_fit_results(
            samples,
            effective_workers,
            run_config,
            reference_interpolated,
            fit_config.objective,
            reference_metrics,
        ):
            # Track running best — only best_result retains its timeseries
            if best_result is None or result.total_cost < best_result.total_cost:
                best_result = result

            # Accumulate a lightweight record (scalars only)
            cost_records.append({
                **result.parameters,
                "total_cost": result.total_cost,
                "mse_q": result.mse_q,
                "mse_dq": result.mse_dq,
                "mse_tau": result.mse_tau,
                "metric_error": result.metric_error,
                "error": result.error,
            })

            desc = base_desc
            if best_result is not None and best_result.total_cost < 1e9:
                params_str = "  ".join(
                    f"{k.split('.')[-1]}={v:.3g}"
                    for k, v in best_result.parameters.items()
                )
                if len(params_str) > 40:
                    params_str = params_str[:37] + "..."
                desc = f"{base_desc}  │  best cost={best_result.total_cost:.4g} ({params_str})"
            progress.update(task_id, advance=1, description=desc)

    if best_result is None or best_result.total_cost >= 1e9:
        # Surface the first error captured from the workers
        first_error = next(
            (r["error"] for r in cost_records if r.get("error")),
            None,
        )
        if first_error:
            print(
                f"error: all simulations failed in worker processes.\n"
                f"  First error: {first_error}\n"
                f"  The probe ran fine sequentially — this is likely a backend\n"
                f"  incompatibility with spawned subprocesses (e.g. PhysX).\n"
                f"  Try --workers 1 to run sequentially.",
                file=__import__('sys').stderr,
            )
        else:
            print("error: all simulations failed (no error details captured).",
                  file=__import__('sys').stderr)
        return 1

    # 5. Build best-fit system config by patching the original system YAML
    best_system_data = _apply_params_to_dict(system_data, best_result.parameters)

    # 6. Save results
    save_fit_results(
        output_dir,
        cost_records,
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
