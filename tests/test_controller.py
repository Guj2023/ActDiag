"""Tests for actdiag.controller."""
from __future__ import annotations

import pytest

from actdiag.controller import (
    ControlOutput,
    ControlState,
    NoController,
    PDController,
    PIDController,
    build_controller,
)
from actdiag.config import (
    NoneControllerProfile,
    PDControllerProfile,
    PIDControllerProfile,
)


def _state(q=0.0, dq=0.0, q_des=0.0, dq_des=0.0, qdd_des=0.0, tau_des=0.0):
    return ControlState(
        time=0.0,
        q=q, dq=dq,
        q_des=q_des, dq_des=dq_des, qdd_des=qdd_des,
        tau_des=tau_des,
    )


# ---------------------------------------------------------------------------
# PDController
# ---------------------------------------------------------------------------

class TestPDController:
    def _make(self, kp=10.0, kd=1.0):
        return PDController(PDControllerProfile(type="pd", kp=kp, kd=kd))

    def test_zero_error_zero_output(self):
        ctrl = self._make()
        out = ctrl.compute(_state(q=1.0, dq=0.0, q_des=1.0, dq_des=0.0))
        assert out.tau_cmd == pytest.approx(0.0)

    def test_position_error_drives_torque(self):
        ctrl = self._make(kp=10.0, kd=0.0)
        out = ctrl.compute(_state(q=0.0, dq=0.0, q_des=1.0, dq_des=0.0))
        assert out.tau_cmd == pytest.approx(10.0)

    def test_velocity_error_drives_torque(self):
        ctrl = self._make(kp=0.0, kd=2.0)
        out = ctrl.compute(_state(q=0.0, dq=1.0, q_des=0.0, dq_des=3.0))
        assert out.tau_cmd == pytest.approx(4.0)

    def test_combined(self):
        ctrl = self._make(kp=5.0, kd=1.0)
        out = ctrl.compute(_state(q=0.5, dq=0.2, q_des=1.0, dq_des=0.5))
        # kp*(1.0-0.5) + kd*(0.5-0.2) = 2.5 + 0.3 = 2.8
        assert out.tau_cmd == pytest.approx(2.8)

    def test_integral_always_zero(self):
        ctrl = self._make()
        out = ctrl.compute(_state(q_des=1.0))
        assert out.integral_error == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# PIDController
# ---------------------------------------------------------------------------

class TestPIDController:
    def _make(self, kp=5.0, ki=1.0, kd=0.5, dt=0.01):
        return PIDController(PIDControllerProfile(type="pid", kp=kp, ki=ki, kd=kd), dt)

    def test_integral_accumulates(self):
        ctrl = self._make(kp=0.0, ki=1.0, kd=0.0, dt=0.1)
        out1 = ctrl.compute(_state(q=0.0, q_des=1.0))
        out2 = ctrl.compute(_state(q=0.0, q_des=1.0))
        # integral after step 1: 1.0 * 0.1 = 0.1 → tau = 0.1
        # integral after step 2: 0.2 → tau = 0.2
        assert out1.tau_cmd == pytest.approx(0.1)
        assert out2.tau_cmd == pytest.approx(0.2)
        assert out2.integral_error == pytest.approx(0.2)

    def test_zero_error_no_change_in_integral(self):
        ctrl = self._make(kp=0.0, ki=1.0, kd=0.0, dt=0.01)
        ctrl.compute(_state(q=0.0, q_des=1.0))  # add 0.01 to integral
        out = ctrl.compute(_state(q=1.0, q_des=1.0))  # zero error now
        assert out.integral_error == pytest.approx(0.01)  # unchanged

    def test_derivative_term(self):
        ctrl = self._make(kp=0.0, ki=0.0, kd=2.0, dt=0.01)
        out = ctrl.compute(_state(q=0.0, dq=0.5, q_des=0.0, dq_des=1.5))
        # kd * (dq_des - dq) = 2.0 * 1.0 = 2.0
        assert out.tau_cmd == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# NoController
# ---------------------------------------------------------------------------

class TestNoController:
    def test_passes_tau_des_through(self):
        ctrl = NoController(NoneControllerProfile(type="none"))
        out = ctrl.compute(_state(tau_des=3.5))
        assert out.tau_cmd == pytest.approx(3.5)

    def test_integral_is_zero(self):
        ctrl = NoController(NoneControllerProfile(type="none"))
        out = ctrl.compute(_state(tau_des=1.0))
        assert out.integral_error == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# build_controller dispatch
# ---------------------------------------------------------------------------

class TestBuildController:
    def test_pd(self):
        ctrl = build_controller(
            PDControllerProfile(type="pd", kp=5.0, kd=1.0), model=None, dt=0.01
        )
        assert isinstance(ctrl, PDController)

    def test_pid(self):
        ctrl = build_controller(
            PIDControllerProfile(type="pid", kp=5.0, ki=0.1, kd=1.0), model=None, dt=0.01
        )
        assert isinstance(ctrl, PIDController)

    def test_none(self):
        ctrl = build_controller(
            NoneControllerProfile(type="none"), model=None, dt=0.01
        )
        assert isinstance(ctrl, NoController)

    def test_inverse_dynamics_without_model_raises(self):
        from actdiag.config import InverseDynamicsControllerProfile
        with pytest.raises(ValueError, match="mujoco"):
            build_controller(
                InverseDynamicsControllerProfile(type="inverse_dynamics"),
                model=None,
                dt=0.01,
            )
