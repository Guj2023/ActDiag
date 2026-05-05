from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pandas as pd
import yaml

from actdiag.fit.evaluator import EvaluationResult
from actdiag.plotting import save_plots


def save_fit_results(
    output_dir: Path,
    results: list[EvaluationResult],
    best_result: EvaluationResult,
    plot_config: Any,
    reference_interpolated: pd.DataFrame,
    best_system_data: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. best_system.yaml — drop-in system config with fitted parameters applied
    best_system_path = output_dir / "best_system.yaml"
    with best_system_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(best_system_data, f, sort_keys=False)

    # 2. best_fit.yaml — raw parameter values for reference
    best_fit_path = output_dir / "best_fit.yaml"
    with best_fit_path.open("w", encoding="utf-8") as f:
        yaml.dump(best_result.parameters, f)

    # 3. top_candidates.csv
    # Sort results by cost
    sorted_results = sorted(results, key=lambda x: x.total_cost)
    top_data = []
    for res in sorted_results[:50]: # Save top 50
        row = {**res.parameters, "total_cost": res.total_cost}
        top_data.append(row)
    
    top_candidates_path = output_dir / "top_candidates.csv"
    pd.DataFrame(top_data).to_csv(top_candidates_path, index=False)

    # 4. objective_breakdown.json
    breakdown = {
        "total_cost": best_result.total_cost,
        "mse_q": best_result.mse_q,
        "mse_dq": best_result.mse_dq,
        "mse_tau": best_result.mse_tau,
        "metric_error": best_result.metric_error,
    }
    breakdown_path = output_dir / "objective_breakdown.json"
    with breakdown_path.open("w", encoding="utf-8") as f:
        json.dump(breakdown, f, indent=2)

    # 5. plots
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(exist_ok=True)
    _save_fit_plots(best_result.timeseries, reference_interpolated, plots_dir)


def _save_fit_plots(
    sim_ts: pd.DataFrame, ref_ts: pd.DataFrame, output_dir: Path
) -> None:
    import matplotlib.pyplot as plt

    # Position plot
    plt.figure(figsize=(10, 6))
    plt.plot(sim_ts["time"], sim_ts["q"], label="Simulation")
    plt.plot(ref_ts["time"], ref_ts["q"], "--", label="Reference")
    if "q_des" in sim_ts.columns:
        plt.plot(sim_ts["time"], sim_ts["q_des"], "k:", alpha=0.5, label="Desired")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position Fit")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "fit_q.png")
    plt.close()

    # Velocity plot
    plt.figure(figsize=(10, 6))
    plt.plot(sim_ts["time"], sim_ts["dq"], label="Simulation")
    if "dq" in ref_ts.columns:
        plt.plot(ref_ts["time"], ref_ts["dq"], "--", label="Reference")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Velocity Fit")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "fit_dq.png")
    plt.close()
