from __future__ import annotations

import copy
from concurrent.futures import ProcessPoolExecutor, as_completed
from itertools import product
import math
import os
from pathlib import Path
import shutil
from typing import Any, Iterator

import numpy as np
import pandas as pd
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
)

from actdiag.config import (
    FrequencyResponseTestProfile,
    RunConfig,
    SweepMetricName,
    load_run_config_from_data,
    load_sweep_config,
    load_yaml_mapping,
    run_config_to_dict,
)
from actdiag.logging_io import (
    create_run_paths,
    frequency_slug,
    make_output_dir,
    RunPaths,
    save_frequency_response_summary,
    save_frequency_response_timeseries,
    save_input_config_data,
    save_resolved_config,
    save_step_metrics,
    save_timeseries,
)
from actdiag.plotting import (
    save_frequency_response_plots,
    save_plots,
    save_sweep_plots,
)
from actdiag.simulate import run_frequency_response_simulation, run_simulation


# ---------------------------------------------------------------------------
# Top-level worker — must be module-level so ProcessPoolExecutor can pickle it
# ---------------------------------------------------------------------------

def _sweep_case_worker(args: dict[str, Any]) -> dict[str, Any]:
    """Execute one sweep case and return a result dict."""
    case_id: str = args["case_id"]
    case_parameters: dict[str, float] = args["parameters"]
    case_system_data: dict = args["system_data"]
    case_scenario_data: dict = args["scenario_data"]
    case_dir = Path(args["case_dir"])
    output_dir = Path(args["output_dir"])
    metric_names: list[str] = args["metric_names"]

    metric_values = _default_metric_values(metric_names)
    error_message: str | None = None

    try:
        run_config = load_run_config_from_data(case_system_data, case_scenario_data)
        run_paths = create_run_paths(Path.cwd(), case_dir)
        save_input_config_data(run_paths, case_system_data, case_scenario_data)
        save_resolved_config(run_paths, run_config_to_dict(run_config))
        timeseries = _run_case(run_config, run_paths)
        metric_values = _compute_metrics(
            timeseries, metric_names, dt=run_config.simulation.dt
        )
    except Exception as exc:
        error_message = f"{type(exc).__name__}: {exc}"
        case_dir.mkdir(parents=True, exist_ok=True)
        (case_dir / "error.txt").write_text(error_message + "\n", encoding="utf-8")

    return {
        "case_id": case_id,
        "parameters": case_parameters,
        "metrics": metric_values,
        "error": error_message,
        "run_dir": str(case_dir.relative_to(output_dir)),
    }


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_sweep(
    system_path: Path,
    scenario_path: Path,
    sweep_path: Path,
    output_dir: Path | None = None,
    workers: int = 1,
) -> int:
    output_dir = make_output_dir(Path.cwd(), "sweeps", output_dir)

    system_data = load_yaml_mapping(system_path)
    scenario_data = load_yaml_mapping(scenario_path)
    sweep_config = load_sweep_config(sweep_path)

    load_run_config_from_data(copy.deepcopy(system_data), copy.deepcopy(scenario_data))

    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(
            f"output directory already exists and is not empty: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    _save_sweep_inputs(output_dir, system_path, scenario_path, sweep_path)

    parameter_names = list(sweep_config.parameters)
    metric_names = list(sweep_config.metrics)
    cases_dir = output_dir / "cases"
    plots_dir = output_dir / "plots"
    cases_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    parameter_values = [sweep_config.parameters[name].values for name in parameter_names]
    total_cases = math.prod(len(v) for v in parameter_values)

    # Pre-build one args dict per case
    all_case_args: list[dict[str, Any]] = []
    for combination in product(*parameter_values):
        case_parameters = {
            name: float(value) for name, value in zip(parameter_names, combination)
        }
        case_system_data = copy.deepcopy(system_data)
        for path, value in case_parameters.items():
            _apply_parameter_override(case_system_data, path, value)

        case_id = _case_slug(case_parameters)
        all_case_args.append({
            "case_id": case_id,
            "parameters": case_parameters,
            "system_data": case_system_data,
            "scenario_data": scenario_data,
            "case_dir": str(cases_dir / case_id),
            "output_dir": str(output_dir),
            "metric_names": metric_names,
        })

    # --- rich progress bar setup ---
    effective_workers = workers if workers > 0 else os.cpu_count() or 1
    worker_label = f" · {effective_workers} workers" if effective_workers > 1 else ""
    base_desc = f"Sweeping {total_cases} cases{worker_label}"

    rows: list[dict[str, object]] = []
    best_val: float | None = None
    best_params: dict[str, float] | None = None
    primary_metric = metric_names[0]

    with Progress(
        TextColumn("[bold cyan]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        TimeRemainingColumn(),
        refresh_per_second=4,
    ) as progress:
        task_id = progress.add_task(base_desc, total=total_cases)

        for result in _iter_results(all_case_args, effective_workers):
            row: dict[str, object] = {"case_id": result["case_id"]}
            row.update(result["parameters"])
            row.update(result["metrics"])
            row["run_dir"] = result["run_dir"]
            if result["error"] is not None:
                row["error"] = result["error"]
            rows.append(row)

            # Track running best for display (lower = better for float metrics)
            metric_val = result["metrics"].get(primary_metric)
            if isinstance(metric_val, float) and math.isfinite(metric_val):
                if best_val is None or metric_val < best_val:
                    best_val = metric_val
                    best_params = result["parameters"]

            desc = base_desc
            if best_val is not None and best_params is not None:
                params_str = "  ".join(
                    f"{k.split('.')[-1]}={v:.3g}" for k, v in best_params.items()
                )
                desc = f"{base_desc}  │  best {primary_metric}={best_val:.4g} ({params_str})"
            progress.update(task_id, advance=1, description=desc)

    summary_columns = ["case_id", *parameter_names, *metric_names, "run_dir"]
    if any("error" in row for row in rows):
        summary_columns.append("error")

    summary = pd.DataFrame(rows, columns=summary_columns)
    summary_path = output_dir / "summary.csv"
    summary.to_csv(summary_path, index=False)
    save_sweep_plots(summary, plots_dir, parameter_names, metric_names)

    print(f"Sweep complete: {output_dir}")
    print(f"Summary: {summary_path}")
    return 0


# ---------------------------------------------------------------------------
# Sequential / parallel dispatch
# ---------------------------------------------------------------------------

def _iter_results(
    all_args: list[dict[str, Any]], workers: int
) -> Iterator[dict[str, Any]]:
    if workers <= 1:
        for args in all_args:
            yield _sweep_case_worker(args)
    else:
        with ProcessPoolExecutor(max_workers=workers) as executor:
            futures = [executor.submit(_sweep_case_worker, args) for args in all_args]
            for future in as_completed(futures):
                yield future.result()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _save_sweep_inputs(
    output_dir: Path, system_path: Path, scenario_path: Path, sweep_path: Path
) -> None:
    config_dir = output_dir / "config"
    config_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(system_path, config_dir / "system.yaml")
    shutil.copy2(scenario_path, config_dir / "scenario.yaml")
    shutil.copy2(sweep_path, config_dir / "sweep.yaml")


def _apply_parameter_override(
    system_data: dict[str, object], path: str, value: float
) -> None:
    parts = path.split(".")
    target: object = system_data
    for part in parts[:-1]:
        if not isinstance(target, dict) or part not in target:
            raise KeyError(f"unknown parameter path '{path}'")
        target = target[part]
    if not isinstance(target, dict):
        raise KeyError(f"unknown parameter path '{path}'")
    leaf = parts[-1]
    if leaf not in target:
        raise KeyError(f"unknown parameter path '{path}'")
    target[leaf] = value


def _case_slug(case_parameters: dict[str, float]) -> str:
    return "__".join(
        f"{_path_slug(name)}_{_value_slug(value)}"
        for name, value in case_parameters.items()
    )


def _path_slug(path: str) -> str:
    return path.replace(".", "_")


def _value_slug(value: float) -> str:
    return (
        f"{value:g}"
        .replace("-", "neg_")
        .replace(".", "_")
        .replace("+", "")
    )


def _run_case(run_config: RunConfig, run_paths: RunPaths) -> pd.DataFrame:
    if isinstance(run_config.test, FrequencyResponseTestProfile):
        artifacts = run_frequency_response_simulation(run_config)
        save_frequency_response_summary(run_paths, artifacts.summary)
        if run_config.logging.save_csv:
            for frequency_hz, timeseries in artifacts.per_frequency_timeseries.items():
                save_frequency_response_timeseries(run_paths, frequency_hz, timeseries)

        combined: list[pd.DataFrame] = []
        for frequency_hz, timeseries in artifacts.per_frequency_timeseries.items():
            save_plots(
                timeseries,
                run_paths.figures_dir
                / "frequency_response"
                / frequency_slug(frequency_hz),
                run_config.plots,
            )
            combined.append(timeseries.assign(frequency_hz=frequency_hz))

        save_frequency_response_plots(
            artifacts.summary, run_paths.figures_dir, run_config.plots
        )
        return pd.concat(combined, ignore_index=True)

    artifacts = run_simulation(run_config)
    if run_config.logging.save_csv:
        save_timeseries(run_paths, artifacts.timeseries)
    save_plots(artifacts.timeseries, run_paths.figures_dir, run_config.plots)
    if artifacts.summary_metrics is not None:
        save_step_metrics(run_paths, artifacts.summary_metrics)
    return artifacts.timeseries


def _compute_metrics(
    timeseries: pd.DataFrame, metric_names: list[SweepMetricName], dt: float
) -> dict[str, float | bool]:
    position_error = pd.to_numeric(
        timeseries.get("position_error", pd.Series(dtype=float)), errors="coerce"
    )
    position_error_values = position_error.to_numpy(dtype=float)
    finite_position_error = position_error_values[np.isfinite(position_error_values)]
    stable = _is_stable(timeseries)

    results: dict[str, float | bool] = {}
    for metric in metric_names:
        if metric == "tracking_rmse":
            results[metric] = (
                math.nan if finite_position_error.size == 0
                else float(np.sqrt(np.mean(np.square(finite_position_error))))
            )
        elif metric == "max_abs_error":
            results[metric] = (
                math.nan if finite_position_error.size == 0
                else float(np.max(np.abs(finite_position_error)))
            )
        elif metric == "stable":
            results[metric] = stable
        elif metric == "jitter_metric":
            results[metric] = _compute_jitter_metric(finite_position_error, dt)

    return results


def _default_metric_values(
    metric_names: list[SweepMetricName],
) -> dict[str, float | bool]:
    return {m: (False if m == "stable" else math.nan) for m in metric_names}


def _is_stable(timeseries: pd.DataFrame) -> bool:
    if timeseries.empty:
        return False

    required_columns = ["q", "dq", "q_des", "dq_des"]
    if any(col not in timeseries for col in required_columns):
        return False

    q_values = timeseries["q"].to_numpy(dtype=float)
    dq_values = timeseries["dq"].to_numpy(dtype=float)
    q_des_values = timeseries["q_des"].to_numpy(dtype=float)
    dq_des_values = timeseries["dq_des"].to_numpy(dtype=float)

    if not (
        np.isfinite(q_values).all()
        and np.isfinite(dq_values).all()
        and np.isfinite(q_des_values).all()
        and np.isfinite(dq_des_values).all()
    ):
        return False

    q_scale = max(1.0, _finite_absmax(q_des_values))
    dq_scale = max(1.0, _finite_absmax(dq_des_values))

    if _finite_absmax(q_values) > max(10.0, 20.0 * q_scale):
        return False
    if _finite_absmax(dq_values) > max(50.0, 20.0 * dq_scale):
        return False
    if _finite_absmax(q_values - q_des_values) > max(5.0, 10.0 * q_scale):
        return False

    return True


def _finite_absmax(values: np.ndarray) -> float:
    finite_values = values[np.isfinite(values)]
    return 0.0 if finite_values.size == 0 else float(np.max(np.abs(finite_values)))


def _compute_jitter_metric(position_error: np.ndarray, dt: float) -> float:
    if position_error.size == 0:
        return math.nan
    if position_error.size < 5:
        return 0.0

    window = max(3, int(round(0.02 / dt)))
    if window % 2 == 0:
        window += 1
    if window >= position_error.size:
        window = (
            position_error.size - 1
            if position_error.size % 2 == 0
            else position_error.size
        )
    if window < 3:
        return 0.0

    kernel = np.ones(window, dtype=float) / float(window)
    smoothed = np.convolve(position_error, kernel, mode="same")
    return float(np.sqrt(np.mean(np.square(position_error - smoothed))))
