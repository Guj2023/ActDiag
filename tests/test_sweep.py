from __future__ import annotations

import pandas as pd
import pytest
import yaml

from actdiag.config import load_sweep_config
from actdiag.sweep import run_sweep


def test_load_sweep_config_rejects_more_than_two_axes(tmp_path):
    sweep_path = tmp_path / "sweep.yaml"
    sweep_path.write_text(
        """
sweep:
  parameters:
    controller.kp:
      values: [1.0]
    controller.kd:
      values: [0.5]
    actuator.time_constant:
      values: [0.01]
  metrics:
    - tracking_rmse
""".strip()
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="1D and 2D"):
        load_sweep_config(sweep_path)


def test_run_sweep_1d_writes_summary_and_case_configs(tmp_path):
    system_path = tmp_path / "system.yaml"
    scenario_path = tmp_path / "scenario.yaml"
    sweep_path = tmp_path / "sweep.yaml"
    output_dir = tmp_path / "runs" / "sweep_1d"

    system_path.write_text(
        """
controller:
  type: pd_position
  kp: 4.0
  kd: 1.0

actuator:
  type: limited_torque
  torque_limit: 20.0
""".strip()
        + "\n",
        encoding="utf-8",
    )
    scenario_path.write_text(
        """
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1

test:
  type: step
  target: 0.2
  start_time: 0.0

simulation:
  duration: 0.1
  dt: 0.01

logging:
  save_csv: true

plots:
  position: false
  velocity: false
  torque: false
  error: false
  phase: false
  frequency_response: false
""".strip()
        + "\n",
        encoding="utf-8",
    )
    sweep_path.write_text(
        """
sweep:
  parameters:
    controller.kp:
      values: [2.0, 6.0]
  metrics:
    - tracking_rmse
    - max_abs_error
    - stable
""".strip()
        + "\n",
        encoding="utf-8",
    )

    assert run_sweep(system_path, scenario_path, sweep_path, output_dir) == 0

    summary = pd.read_csv(output_dir / "summary.csv")
    assert list(summary.columns) == [
        "case_id",
        "controller.kp",
        "tracking_rmse",
        "max_abs_error",
        "stable",
        "run_dir",
    ]
    assert len(summary) == 2
    assert set(summary["controller.kp"]) == {2.0, 6.0}
    assert (output_dir / "plots" / "line_tracking_rmse.png").exists()
    assert (output_dir / "plots" / "heatmap_tracking_rmse.png").exists()
    assert set(summary["run_dir"]) == {
        "cases/controller_kp_2",
        "cases/controller_kp_6",
    }

    case_system = yaml.safe_load(
        (
            output_dir / "cases" / "controller_kp_2" / "config" / "system.yaml"
        ).read_text(
            encoding="utf-8"
        )
    )
    assert case_system["controller"]["kp"] == 2.0


def test_run_sweep_2d_writes_heatmaps(tmp_path):
    system_path = tmp_path / "system.yaml"
    scenario_path = tmp_path / "scenario.yaml"
    sweep_path = tmp_path / "sweep.yaml"
    output_dir = tmp_path / "runs" / "sweep_2d"

    system_path.write_text(
        """
controller:
  type: pd_position
  kp: 4.0
  kd: 1.0

actuator:
  type: dynamic_torque
  torque_limit: 20.0
  time_constant: 0.01
""".strip()
        + "\n",
        encoding="utf-8",
    )
    scenario_path.write_text(
        """
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1

test:
  type: step
  target: 0.2
  start_time: 0.0

simulation:
  duration: 0.1
  dt: 0.01

logging:
  save_csv: false

plots:
  position: false
  velocity: false
  torque: false
  error: false
  phase: false
  frequency_response: false
""".strip()
        + "\n",
        encoding="utf-8",
    )
    sweep_path.write_text(
        """
sweep:
  parameters:
    controller.kp:
      values: [2.0, 6.0]
    actuator.time_constant:
      values: [0.005, 0.02]
  metrics:
    - tracking_rmse
    - jitter_metric
    - stable
""".strip()
        + "\n",
        encoding="utf-8",
    )

    assert run_sweep(system_path, scenario_path, sweep_path, output_dir) == 0

    summary = pd.read_csv(output_dir / "summary.csv")
    assert len(summary) == 4
    assert {
        "case_id",
        "controller.kp",
        "actuator.time_constant",
        "tracking_rmse",
        "jitter_metric",
        "stable",
        "run_dir",
    } == set(summary.columns)
    assert (output_dir / "plots" / "heatmap_tracking_rmse.png").exists()
    assert (output_dir / "plots" / "heatmap_jitter_metric.png").exists()
    assert (output_dir / "plots" / "heatmap_stable.png").exists()


def test_run_sweep_replaces_existing_output_dir(tmp_path):
    system_path = tmp_path / "system.yaml"
    scenario_path = tmp_path / "scenario.yaml"
    sweep_path = tmp_path / "sweep.yaml"
    output_dir = tmp_path / "runs" / "sweep_replace"

    system_path.write_text(
        """
controller:
  type: pd_position
  kp: 4.0
  kd: 1.0

actuator:
  type: limited_torque
  torque_limit: 20.0
""".strip()
        + "\n",
        encoding="utf-8",
    )
    scenario_path.write_text(
        """
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1

test:
  type: step
  target: 0.2
  start_time: 0.0

simulation:
  duration: 0.1
  dt: 0.01

logging:
  save_csv: false

plots:
  position: false
  velocity: false
  torque: false
  error: false
  phase: false
  frequency_response: false
""".strip()
        + "\n",
        encoding="utf-8",
    )
    sweep_path.write_text(
        """
sweep:
  parameters:
    controller.kp:
      values: [2.0]
  metrics:
    - tracking_rmse
""".strip()
        + "\n",
        encoding="utf-8",
    )

    output_dir.mkdir(parents=True)
    (output_dir / "stale.txt").write_text("old data\n", encoding="utf-8")

    assert run_sweep(system_path, scenario_path, sweep_path, output_dir) == 0
    assert not (output_dir / "stale.txt").exists()
    assert (output_dir / "summary.csv").exists()
