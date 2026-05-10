"""Tests for actdiag.simulate — simulation loop and step-response metrics."""
from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from actdiag.config import load_run_config_from_data
from actdiag.simulate import (
    compute_step_response_metrics,
    run_simulation,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_run_config(
    kp=50.0, kd=5.0, torque_limit=10.0,
    inertia=0.05, damping=0.5, gravity=False,
    target=1.0, duration=3.0, dt=0.01,
    backend="mujoco",
):
    return load_run_config_from_data(
        {
            "controller": {"type": "pd", "kp": kp, "kd": kd},
            "actuator": {"type": "ideal_torque", "torque_limit": torque_limit},
        },
        {
            "scene": {
                "type": "single_joint",
                "inertia": inertia,
                "damping": damping,
                "gravity": gravity,
            },
            "test": {"type": "step", "target": target},
            "simulation": {"duration": duration, "dt": dt, "backend": backend},
        },
    )


# ---------------------------------------------------------------------------
# run_simulation — basic contract
# ---------------------------------------------------------------------------

class TestRunSimulation:
    def test_returns_artifacts_with_timeseries(self):
        rc = _make_run_config()
        artifacts = run_simulation(rc)
        assert isinstance(artifacts.timeseries, pd.DataFrame)

    def test_timeseries_has_expected_columns(self):
        rc = _make_run_config()
        df = run_simulation(rc).timeseries
        for col in ("time", "q", "dq", "q_des", "dq_des", "tau_cmd", "tau_applied"):
            assert col in df.columns, f"missing column: {col}"

    def test_time_starts_at_zero(self):
        rc = _make_run_config(duration=1.0, dt=0.01)
        df = run_simulation(rc).timeseries
        assert df["time"].iloc[0] == pytest.approx(0.0)

    def test_time_length_matches_duration(self):
        rc = _make_run_config(duration=1.0, dt=0.01)
        df = run_simulation(rc).timeseries
        # 0, 0.01, ..., 1.00 → 101 rows
        assert len(df) == 101

    def test_pd_controller_converges_to_target(self):
        """High-gain PD on low-inertia system should settle near target."""
        rc = _make_run_config(kp=200.0, kd=10.0, damping=1.0, target=1.0, duration=3.0, dt=0.005)
        df = run_simulation(rc).timeseries
        final_q = df["q"].iloc[-1]
        assert final_q == pytest.approx(1.0, abs=0.05)

    def test_no_video_frames_by_default(self):
        rc = _make_run_config()
        artifacts = run_simulation(rc)
        assert artifacts.video_frames is None
        assert artifacts.video_fps is None

    def test_step_metrics_computed_for_step_test(self):
        rc = _make_run_config(kp=200.0, kd=10.0, damping=1.0)
        artifacts = run_simulation(rc)
        assert artifacts.summary_metrics is not None
        assert "steady_state_value" in artifacts.summary_metrics

    def test_torque_capped_by_actuator_limit(self):
        """Tau applied must never exceed the torque limit."""
        rc = _make_run_config(kp=1000.0, torque_limit=5.0)
        df = run_simulation(rc).timeseries
        assert df["tau_applied"].abs().max() <= 5.0 + 1e-9

    def test_q_starts_at_q0(self):
        """First recorded state should be the initial joint angle q0."""
        rc = load_run_config_from_data(
            {"controller": {"type": "pd", "kp": 10.0, "kd": 1.0},
             "actuator": {"type": "ideal_torque", "torque_limit": 5.0}},
            {"scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1, "q0": 0.5},
             "test": {"type": "step", "target": 1.0},
             "simulation": {"duration": 1.0, "dt": 0.01}},
        )
        df = run_simulation(rc).timeseries
        assert df["q"].iloc[0] == pytest.approx(0.5, abs=1e-6)

    def test_torque_test_with_none_controller(self):
        rc = load_run_config_from_data(
            {"controller": {"type": "none"},
             "actuator": {"type": "ideal_torque", "torque_limit": 5.0}},
            {"scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1},
             "test": {"type": "torque_step", "target_torque": 1.0},
             "simulation": {"duration": 1.0, "dt": 0.01}},
        )
        df = run_simulation(rc).timeseries
        # applied torque should be 1.0 for most of the trajectory
        late = df["tau_applied"].iloc[50:]
        assert late.mean() == pytest.approx(1.0, abs=0.01)

    def test_position_error_column(self):
        rc = _make_run_config(kp=100.0, kd=5.0, target=1.0, duration=2.0)
        df = run_simulation(rc).timeseries
        # position_error = q_des - q; should be large initially, small at end
        initial_error = df["position_error"].iloc[0]
        final_error = df["position_error"].iloc[-1]
        assert abs(initial_error) > abs(final_error)


# ---------------------------------------------------------------------------
# compute_step_response_metrics
# ---------------------------------------------------------------------------

class TestStepResponseMetrics:
    def _make_step_df(self, target=1.0, overshoot_frac=0.0, settle_at=2.0, dt=0.01):
        """Build a synthetic step-response timeseries."""
        t = np.arange(0.0, 3.0 + dt / 2, dt)
        q = np.where(
            t < 1.0,
            0.0,
            target * (1.0 + overshoot_frac * np.exp(-(t - 1.0) * 5.0)),
        )
        q_des = np.where(t >= 1.0, target, 0.0)
        return pd.DataFrame({"time": t, "q": q, "q_des": q_des})

    def test_steady_state_value_correct(self):
        rc_profile = load_run_config_from_data(
            {"controller": {"type": "pd", "kp": 200.0, "kd": 10.0},
             "actuator": {"type": "ideal_torque", "torque_limit": 20.0}},
            {"scene": {"type": "single_joint", "inertia": 0.05, "damping": 1.0},
             "test": {"type": "step", "target": 1.5},
             "simulation": {"duration": 4.0, "dt": 0.005}},
        ).test
        df = run_simulation(load_run_config_from_data(
            {"controller": {"type": "pd", "kp": 200.0, "kd": 10.0},
             "actuator": {"type": "ideal_torque", "torque_limit": 20.0}},
            {"scene": {"type": "single_joint", "inertia": 0.05, "damping": 1.0},
             "test": {"type": "step", "target": 1.5},
             "simulation": {"duration": 4.0, "dt": 0.005}},
        )).timeseries
        from actdiag.config import StepTestProfile
        metrics = compute_step_response_metrics(df, StepTestProfile(test_type="step", target=1.5))
        assert metrics["steady_state_value"] == pytest.approx(1.5, abs=0.05)

    def test_metrics_keys_present(self):
        from actdiag.config import StepTestProfile
        rc = _make_run_config(kp=200.0, kd=10.0, damping=1.0, target=1.0, duration=5.0, dt=0.005)
        df = run_simulation(rc).timeseries
        metrics = compute_step_response_metrics(
            df, StepTestProfile(test_type="step", target=1.0)
        )
        for key in ("steady_state_value", "steady_state_error", "peak_value",
                    "percent_overshoot", "rise_time", "settling_time"):
            assert key in metrics

    def test_zero_step_amplitude_returns_none_metrics(self):
        from actdiag.config import StepTestProfile
        t = np.linspace(0, 2, 201)
        df = pd.DataFrame({
            "time": t,
            "q": np.zeros_like(t),
            "q_des": np.zeros_like(t),
        })
        metrics = compute_step_response_metrics(
            df, StepTestProfile(test_type="step", target=0.0)
        )
        assert metrics["percent_overshoot"] is None
        assert metrics["rise_time"] is None
        assert metrics["settling_time"] is None
