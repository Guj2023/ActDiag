"""Tests for actdiag.actuator."""
from __future__ import annotations

import pytest

from actdiag.actuator import (
    ActuatorOutput,
    DynamicTorqueActuator,
    IdealActuator,
    build_actuator,
)
from actdiag.config import DynamicTorqueActuatorProfile, IdealActuatorProfile


# ---------------------------------------------------------------------------
# IdealActuator
# ---------------------------------------------------------------------------

class TestIdealActuator:
    def _make(self, limit=5.0):
        return IdealActuator(IdealActuatorProfile(type="ideal_actuator", torque_limit=limit))

    def test_within_limit_passthrough(self):
        act = self._make(5.0)
        out = act.apply(3.0)
        assert out.tau_applied == pytest.approx(3.0)
        assert out.is_saturated is False

    def test_positive_saturation(self):
        act = self._make(5.0)
        out = act.apply(10.0)
        assert out.tau_applied == pytest.approx(5.0)
        assert out.is_saturated is True

    def test_negative_saturation(self):
        act = self._make(5.0)
        out = act.apply(-8.0)
        assert out.tau_applied == pytest.approx(-5.0)
        assert out.is_saturated is True

    def test_zero_torque(self):
        act = self._make(5.0)
        out = act.apply(0.0)
        assert out.tau_applied == pytest.approx(0.0)
        assert out.is_saturated is False

    def test_exactly_at_limit_not_saturated(self):
        act = self._make(5.0)
        out = act.apply(5.0)
        assert out.tau_applied == pytest.approx(5.0)
        assert out.is_saturated is False


# ---------------------------------------------------------------------------
# DynamicTorqueActuator
# ---------------------------------------------------------------------------

class TestDynamicTorqueActuator:
    def _make(self, limit=5.0, tc=0.05, rate=None, deadzone=None, dt=0.01):
        profile = DynamicTorqueActuatorProfile(
            type="dynamic_torque",
            torque_limit=limit,
            time_constant=tc,
            torque_rate_limit=rate,
            deadzone=deadzone,
        )
        return DynamicTorqueActuator(profile, dt)

    def test_starts_at_zero(self):
        act = self._make()
        out = act.apply(5.0)
        # first step: alpha = dt/(tc+dt) = 0.01/0.06 ≈ 0.167; tau ≈ 0.833
        assert out.tau_applied < 5.0  # lags target

    def test_approaches_target_asymptotically(self):
        act = self._make(limit=5.0, tc=0.05, dt=0.01)
        for _ in range(200):
            out = act.apply(5.0)
        assert out.tau_applied == pytest.approx(5.0, abs=0.01)

    def test_saturation_applied_before_lag(self):
        act = self._make(limit=2.0, tc=0.05, dt=0.01)
        out = act.apply(100.0)
        assert out.tau_applied <= 2.0
        assert out.is_saturated is True

    def test_deadzone_suppresses_small_command(self):
        act = self._make(deadzone=0.5, dt=0.01)
        out = act.apply(0.4)  # inside deadzone
        assert out.tau_applied == pytest.approx(0.0, abs=0.01)

    def test_deadzone_passes_large_command(self):
        act = self._make(deadzone=0.5, tc=0.001, dt=0.01)
        for _ in range(100):
            out = act.apply(2.0)
        assert out.tau_applied > 0.5

    def test_torque_rate_limit(self):
        act = self._make(tc=0.001, rate=10.0, dt=0.01)
        # max delta per step = 10 * 0.01 = 0.1 N·m
        out = act.apply(5.0)
        assert out.tau_applied <= 0.11  # first step capped at ~0.1


# ---------------------------------------------------------------------------
# build_actuator dispatch
# ---------------------------------------------------------------------------

class TestBuildActuator:
    def test_ideal_profile(self):
        act = build_actuator(IdealActuatorProfile(type="ideal_actuator", torque_limit=5.0), 0.01)
        assert isinstance(act, IdealActuator)

    def test_dynamic_profile(self):
        act = build_actuator(
            DynamicTorqueActuatorProfile(type="dynamic_torque", torque_limit=5.0, time_constant=0.05),
            0.01,
        )
        assert isinstance(act, DynamicTorqueActuator)
