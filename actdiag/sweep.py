from __future__ import annotations

import copy
from itertools import product
import math
from pathlib import Path
import shutil

import numpy as np
import pandas as pd

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


def run_sweep(
    system_path: Path,
    scenario_path: Path,
    sweep_path: Path,
    output_dir: Path | None = None,
) -> int:
    output_dir = make_output_dir(Path.cwd(), "sweeps", output_dir)

    system_data = load_yaml_mapping(system_path)
    scenario_data = load_yaml_mapping(scenario_path)
    sweep_config = load_sweep_config(sweep_path)

    # Validate the base run config before enumerating cases.
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
    total_cases = math.prod(len(values) for values in parameter_values)
    print(f"Running sweep with {total_cases} cases...")

    rows: list[dict[str, object]] = []
    for combination in product(*parameter_values):
        case_system_data = copy.deepcopy(system_data)
        case_scenario_data = copy.deepcopy(scenario_data)
        case_parameters = {
            name: float(value) for name, value in zip(parameter_names, combination)
        }
        case_id = _case_slug(case_parameters)
        case_dir = cases_dir / case_id

        for path, value in case_parameters.items():
            _apply_parameter_override(case_system_data, path, value)

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
            error_message = str(exc)
            case_dir.mkdir(parents=True, exist_ok=True)
            with (case_dir / "error.txt").open("w", encoding="utf-8") as handle:
                handle.write(f"{type(exc).__name__}: {exc}\n")

        row: dict[str, object] = {"case_id": case_id}
        row.update(case_parameters)
        row.update(metric_values)
        row["run_dir"] = str(case_dir.relative_to(output_dir))
        if error_message is not None:
            row["error"] = error_message
        rows.append(row)

        if len(rows) % 10 == 0 or len(rows) == total_cases:
            index = len(rows)
            print(f"  Completed {index}/{total_cases} cases")

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
    parts = []
    for name, value in case_parameters.items():
        parts.append(f"{_path_slug(name)}_{_value_slug(value)}")
    return "__".join(parts)


def _path_slug(path: str) -> str:
    return path.replace(".", "_")


def _value_slug(value: float) -> str:
    text = f"{value:g}"
    return (
        text.replace("-", "neg_")
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
            if finite_position_error.size == 0:
                results[metric] = math.nan
            else:
                results[metric] = float(
                    np.sqrt(np.mean(np.square(finite_position_error)))
                )
        elif metric == "max_abs_error":
            if finite_position_error.size == 0:
                results[metric] = math.nan
            else:
                results[metric] = float(np.max(np.abs(finite_position_error)))
        elif metric == "stable":
            results[metric] = stable
        elif metric == "jitter_metric":
            results[metric] = _compute_jitter_metric(finite_position_error, dt)

    return results


def _default_metric_values(
    metric_names: list[SweepMetricName],
) -> dict[str, float | bool]:
    values: dict[str, float | bool] = {}
    for metric in metric_names:
        values[metric] = False if metric == "stable" else math.nan
    return values


def _is_stable(timeseries: pd.DataFrame) -> bool:
    if timeseries.empty:
        return False

    required_columns = ["q", "dq", "q_des", "dq_des"]
    if any(column not in timeseries for column in required_columns):
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
    position_error_scale = _finite_absmax(q_values - q_des_values)

    if _finite_absmax(q_values) > max(10.0, 20.0 * q_scale):
        return False
    if _finite_absmax(dq_values) > max(50.0, 20.0 * dq_scale):
        return False
    if position_error_scale > max(5.0, 10.0 * q_scale):
        return False

    return True


def _finite_absmax(values: np.ndarray) -> float:
    finite_values = values[np.isfinite(values)]
    if finite_values.size == 0:
        return 0.0
    return float(np.max(np.abs(finite_values)))


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
    high_frequency_component = position_error - smoothed
    return float(np.sqrt(np.mean(np.square(high_frequency_component))))
