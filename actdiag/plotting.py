from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


def save_plots(timeseries: pd.DataFrame, figures_dir: Path) -> None:
    figures_dir.mkdir(parents=True, exist_ok=True)

    _save_line_plot(
        figures_dir / "position.png",
        timeseries["time"],
        (
            ("Measured position", timeseries["q"]),
            ("Desired position", timeseries["q_des"]),
        ),
        ylabel="Position [rad]",
        title="Position vs Time",
    )
    _save_line_plot(
        figures_dir / "velocity.png",
        timeseries["time"],
        (
            ("Measured velocity", timeseries["dq"]),
            ("Desired velocity", timeseries["dq_des"]),
        ),
        ylabel="Velocity [rad/s]",
        title="Velocity vs Time",
    )
    _save_line_plot(
        figures_dir / "torque.png",
        timeseries["time"],
        (
            ("Commanded torque", timeseries["tau_cmd"]),
            ("Applied torque", timeseries["tau_applied"]),
        ),
        ylabel="Torque [Nm]",
        title="Torque vs Time",
    )
    _save_line_plot(
        figures_dir / "error.png",
        timeseries["time"],
        (("Position error", timeseries["position_error"]),),
        ylabel="Position error [rad]",
        title="Position Error vs Time",
    )
    _save_phase_plot(figures_dir / "phase.png", timeseries)


def _save_line_plot(
    path: Path,
    time_values: pd.Series,
    series: tuple[tuple[str, pd.Series], ...],
    *,
    ylabel: str,
    title: str,
) -> None:
    figure, axis = plt.subplots(figsize=(9, 4.5))
    for label, values in series:
        axis.plot(time_values, values, label=label, linewidth=2)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    if len(series) > 1:
        axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _save_phase_plot(path: Path, timeseries: pd.DataFrame) -> None:
    figure, axis = plt.subplots(figsize=(5.5, 5.5))
    axis.plot(timeseries["q"], timeseries["dq"], linewidth=2)
    axis.set_xlabel("Position [rad]")
    axis.set_ylabel("Velocity [rad/s]")
    axis.set_title("Phase Plot")
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)

