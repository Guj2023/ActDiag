from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

from actdiag.config import PlotConfig


def save_plots(timeseries: pd.DataFrame, figures_dir: Path, plot_config: PlotConfig) -> None:
    figures_dir.mkdir(parents=True, exist_ok=True)

    if plot_config.position:
        _save_line_plot(
            figures_dir / "position.png",
            timeseries["time"],
            _defined_series(
                ("Measured position", timeseries["q"]),
                ("Desired position", timeseries["q_des"]),
            ),
            ylabel="Position [rad]",
            title="Position vs Time",
        )
    if plot_config.velocity:
        _save_line_plot(
            figures_dir / "velocity.png",
            timeseries["time"],
            _defined_series(
                ("Measured velocity", timeseries["dq"]),
                ("Desired velocity", timeseries["dq_des"]),
            ),
            ylabel="Velocity [rad/s]",
            title="Velocity vs Time",
        )
    if plot_config.torque:
        _save_line_plot(
            figures_dir / "torque.png",
            timeseries["time"],
            _defined_series(
                ("Desired torque", timeseries["tau_des"]),
                ("Commanded torque", timeseries["tau_cmd"]),
                ("Applied torque", timeseries["tau_applied"]),
            ),
            ylabel="Torque [Nm]",
            title="Torque vs Time",
        )
    if plot_config.error:
        _save_error_plot(figures_dir / "error.png", timeseries)
    if plot_config.phase:
        _save_phase_plot(figures_dir / "phase.png", timeseries)


def save_frequency_response_plots(
    summary: pd.DataFrame, figures_dir: Path, plot_config: PlotConfig
) -> None:
    if not plot_config.frequency_response:
        return

    figures_dir.mkdir(parents=True, exist_ok=True)
    _save_frequency_plot(
        figures_dir / "frequency_response_gain.png",
        summary,
        value_column="gain",
        ylabel="Gain [-]",
        title="Gain vs Frequency",
    )
    _save_frequency_plot(
        figures_dir / "frequency_response_phase.png",
        summary,
        value_column="phase_deg",
        ylabel="Phase [deg]",
        title="Phase vs Frequency",
    )


def _save_line_plot(
    path: Path,
    time_values: pd.Series,
    series: tuple[tuple[str, pd.Series], ...],
    *,
    ylabel: str,
    title: str,
) -> None:
    figure, axis = plt.subplots(figsize=(9, 4.5))
    plotted_series = 0
    for label, values in series:
        axis.plot(time_values, values, label=label, linewidth=2)
        plotted_series += 1
    if plotted_series == 0:
        axis.text(
            0.5,
            0.5,
            "No reference signal for this plot",
            ha="center",
            va="center",
            transform=axis.transAxes,
        )
    axis.set_xlabel("Time [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    if plotted_series > 1:
        axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _save_error_plot(path: Path, timeseries: pd.DataFrame) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axes[0].plot(timeseries["time"], timeseries["position_error"], linewidth=2)
    axes[0].set_ylabel("Pos. error [rad]")
    axes[0].set_title("Error vs Time")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(timeseries["time"], timeseries["velocity_error"], linewidth=2)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Vel. error [rad/s]")
    axes[1].grid(True, alpha=0.3)

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


def _save_frequency_plot(
    path: Path,
    summary: pd.DataFrame,
    *,
    value_column: str,
    ylabel: str,
    title: str,
) -> None:
    figure, axis = plt.subplots(figsize=(7, 4.5))
    axis.semilogx(
        summary["frequency_hz"],
        summary[value_column],
        marker="o",
        linewidth=2,
    )
    axis.set_xlabel("Frequency [Hz]")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, which="both", alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _defined_series(
    *series: tuple[str, pd.Series]
) -> tuple[tuple[str, pd.Series], ...]:
    return tuple((label, values) for label, values in series if values.notna().any())
