from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from actdiag.config import RunConfig, StepTestProfile
from actdiag.simulate import compute_step_response_metrics, run_simulation


@dataclass
class EvaluationResult:
    total_cost: float
    mse_q: float
    mse_dq: float
    mse_tau: float
    metric_error: float
    parameters: dict[str, float]
    timeseries: pd.DataFrame
    metrics: dict[str, float | None] | None
    error: str | None = None


def load_reference(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if "time" not in df.columns:
        raise ValueError(f"Reference CSV {path} must contain 'time' column")
    return df


def interpolate_reference(
    reference: pd.DataFrame, simulation_times: np.ndarray
) -> pd.DataFrame:
    ref_time = reference["time"].to_numpy()
    results = {"time": simulation_times}

    for column in ["q", "dq", "tau_applied"]:
        if column in reference.columns:
            column_data = reference[column].to_numpy()
            results[column] = np.interp(simulation_times, ref_time, column_data)

    df = pd.DataFrame(results)
    # Ensure q column exists for evaluate_sample
    if "q" not in df.columns:
        df["q"] = 0.0
    return df


def set_nested_attr(obj: Any, path: str, value: Any) -> None:
    parts = path.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


def evaluate_sample(
    sample: dict[str, float],
    run_config: RunConfig,
    reference_interpolated: pd.DataFrame,
    objective_weights: Any,
    reference_metrics: dict[str, float | None] | None = None,
) -> EvaluationResult:
    config = copy.deepcopy(run_config)
    for param_name, value in sample.items():
        set_nested_attr(config, param_name, value)

    try:
        artifacts = run_simulation(config)
        timeseries = artifacts.timeseries

        # Check for NaN in simulation
        if timeseries["q"].isna().any() or timeseries["dq"].isna().any():
            return _failure_result(sample)

        # Trajectory Error
        # Use 0.0 as fallback for q reference if it contains NaNs (e.g. from a torque test)
        ref_q_values = reference_interpolated["q"].fillna(0.0).to_numpy()
        mse_q = float(
            np.mean((timeseries["q"] - ref_q_values) ** 2)
        )
        
        mse_dq = 0.0
        # If 'dq' is missing in reference, or is all NaN, use 0 as reference
        ref_dq = reference_interpolated.get("dq")
        if ref_dq is not None:
            ref_dq_values = ref_dq.fillna(0.0).to_numpy()
            mse_dq = float(np.mean((timeseries["dq"] - ref_dq_values) ** 2))
        else:
            mse_dq = float(np.mean(timeseries["dq"] ** 2))

        mse_tau = 0.0
        # If 'tau_applied' is missing in reference, or is all NaN, use 0 as reference
        ref_tau = reference_interpolated.get("tau_applied")
        if ref_tau is not None:
            ref_tau_values = ref_tau.fillna(0.0).to_numpy()
            mse_tau = float(np.mean((timeseries["tau_applied"] - ref_tau_values) ** 2))
        else:
            mse_tau = float(np.mean(timeseries["tau_applied"] ** 2))

        # Metric Error
        metric_error = 0.0
        sim_metrics = None
        if isinstance(config.test, StepTestProfile) and reference_metrics is not None:
            sim_metrics = compute_step_response_metrics(timeseries, config.test)
            
            for key in ["rise_time", "percent_overshoot", "settling_time"]:
                v_sim = sim_metrics.get(key)
                v_ref = reference_metrics.get(key)
                if v_sim is not None and v_ref is not None:
                    metric_error += abs(v_sim - v_ref)
                elif v_sim is not None or v_ref is not None:
                    # Penalty for missing metrics
                    metric_error += 10.0 

        total_cost = (
            objective_weights.q_weight * mse_q
            + objective_weights.dq_weight * mse_dq
            + objective_weights.tau_weight * mse_tau
            + objective_weights.metric_weight * metric_error
        )

        return EvaluationResult(
            total_cost=total_cost,
            mse_q=mse_q,
            mse_dq=mse_dq,
            mse_tau=mse_tau,
            metric_error=metric_error,
            parameters=sample,
            timeseries=timeseries,
            metrics=sim_metrics,
        )

    except Exception as exc:
        return _failure_result(sample, error=f"{type(exc).__name__}: {exc}")


def _failure_result(
    sample: dict[str, float], error: str | None = None
) -> EvaluationResult:
    return EvaluationResult(
        total_cost=1e9,
        mse_q=1e9,
        mse_dq=1e9,
        mse_tau=1e9,
        metric_error=1e9,
        parameters=sample,
        timeseries=pd.DataFrame(),
        metrics=None,
        error=error,
    )
