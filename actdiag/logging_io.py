from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import shutil
from typing import Any

import imageio.v2 as imageio
import pandas as pd
import yaml


@dataclass(frozen=True)
class RunPaths:
    run_dir: Path
    config_dir: Path
    data_dir: Path
    figures_dir: Path
    summary_dir: Path
    video_dir: Path


def create_run_paths(project_root: Path, output_dir: Path | None = None) -> RunPaths:
    if output_dir is None:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        run_dir = project_root / "runs" / timestamp
    else:
        run_dir = output_dir

    if run_dir.exists() and any(run_dir.iterdir()):
        raise FileExistsError(
            f"output directory already exists and is not empty: {run_dir}"
        )

    config_dir = run_dir / "config"
    data_dir = run_dir / "data"
    figures_dir = run_dir / "figures"
    summary_dir = run_dir / "summary"
    video_dir = run_dir / "video"

    for path in (config_dir, data_dir, figures_dir, summary_dir):
        path.mkdir(parents=True, exist_ok=True)

    return RunPaths(
        run_dir=run_dir,
        config_dir=config_dir,
        data_dir=data_dir,
        figures_dir=figures_dir,
        summary_dir=summary_dir,
        video_dir=video_dir,
    )


def save_input_configs(
    run_paths: RunPaths,
    system_path: Path,
    scenario_path: Path,
) -> None:
    shutil.copy2(system_path, run_paths.config_dir / "system.yaml")
    shutil.copy2(scenario_path, run_paths.config_dir / "scenario.yaml")


def save_resolved_config(run_paths: RunPaths, resolved_config: dict[str, Any]) -> None:
    output_path = run_paths.config_dir / "resolved.yaml"
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(resolved_config, handle, sort_keys=False)


def save_timeseries(run_paths: RunPaths, timeseries: pd.DataFrame) -> Path:
    output_path = run_paths.data_dir / "timeseries.csv"
    timeseries.to_csv(output_path, index=False)
    return output_path


def save_frequency_response_timeseries(
    run_paths: RunPaths, frequency_hz: float, timeseries: pd.DataFrame
) -> Path:
    output_dir = (
        run_paths.data_dir / "frequency_response" / frequency_slug(frequency_hz)
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "timeseries.csv"
    timeseries.to_csv(output_path, index=False)
    return output_path


def save_step_metrics(
    run_paths: RunPaths, metrics: dict[str, float | None]
) -> Path:
    output_path = run_paths.summary_dir / "step_metrics.json"
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)
    return output_path


def save_frequency_response_summary(
    run_paths: RunPaths, summary: pd.DataFrame
) -> Path:
    output_path = run_paths.summary_dir / "frequency_response.csv"
    summary.sort_values("frequency_hz").to_csv(output_path, index=False)
    return output_path


def save_video(run_paths: RunPaths, frames: list, fps: int) -> Path:
    run_paths.video_dir.mkdir(parents=True, exist_ok=True)
    output_path = run_paths.video_dir / "sim.mp4"
    imageio.mimsave(output_path, frames, fps=fps, macro_block_size=None)
    return output_path


def frequency_slug(frequency_hz: float) -> str:
    return f"{frequency_hz:.3f}_hz".replace(".", "_")
