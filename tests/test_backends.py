"""Tests for actdiag.backends — MuJoCo backend physics and OpenModelica error handling."""
from __future__ import annotations

import math

import numpy as np
import pytest

from actdiag.backends import build_backend
from actdiag.backends.mujoco_backend import MuJoCoBackend
from actdiag.config import SingleJointConfig, SingleJointSceneProfile


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _profile(inertia=0.05, damping=0.1, gravity=False, q0=0.0, dq0=0.0):
    return SingleJointSceneProfile(
        scene_type="single_joint",
        joint=SingleJointConfig(
            inertia=inertia, damping=damping, gravity=gravity, q0=q0, dq0=dq0
        ),
    )


# ---------------------------------------------------------------------------
# MuJoCo backend
# ---------------------------------------------------------------------------

class TestMuJoCoBackend:
    def test_build_via_factory(self):
        backend = build_backend(_profile(), dt=0.002, backend_name="mujoco")
        assert isinstance(backend, MuJoCoBackend)

    def test_get_state_returns_initial_conditions(self):
        backend = MuJoCoBackend(_profile(q0=0.5, dq0=1.2), dt=0.002)
        q, dq = backend.get_state()
        assert q == pytest.approx(0.5, abs=1e-6)
        assert dq == pytest.approx(1.2, abs=1e-6)

    def test_zero_torque_step_changes_state(self):
        """A single step with zero torque should still move the joint (if dq0 ≠ 0)."""
        backend = MuJoCoBackend(_profile(q0=0.0, dq0=1.0, damping=0.0), dt=0.01)
        q0, dq0 = backend.get_state()
        backend.apply_torque_and_step(0.0)
        q1, dq1 = backend.get_state()
        assert q1 > q0

    def test_positive_torque_accelerates(self):
        """Positive torque from rest should produce positive velocity."""
        backend = MuJoCoBackend(_profile(q0=0.0, dq0=0.0, damping=0.0), dt=0.002)
        for _ in range(10):
            backend.apply_torque_and_step(1.0)
        _, dq = backend.get_state()
        assert dq > 0.0

    def test_damping_decays_velocity(self):
        """With only damping and initial velocity, speed should decay."""
        backend = MuJoCoBackend(_profile(q0=0.0, dq0=5.0, damping=0.5), dt=0.002)
        for _ in range(500):
            backend.apply_torque_and_step(0.0)
        _, dq = backend.get_state()
        assert abs(dq) < 0.5

    def test_dt_attribute(self):
        backend = MuJoCoBackend(_profile(), dt=0.005)
        assert backend.dt == pytest.approx(0.005)

    def test_gravity_with_gravity_enabled(self):
        """At q=0 with gravity, the torque should accelerate the joint."""
        backend_gravity = MuJoCoBackend(_profile(gravity=True, q0=0.0, dq0=0.0), dt=0.002)
        backend_nograv = MuJoCoBackend(_profile(gravity=False, q0=0.0, dq0=0.0), dt=0.002)
        for _ in range(50):
            backend_gravity.apply_torque_and_step(0.0)
            backend_nograv.apply_torque_and_step(0.0)
        q_grav, _ = backend_gravity.get_state()
        q_nograv, _ = backend_nograv.get_state()
        # gravity pulls the arm (q grows when gravity is enabled with q0=0)
        assert q_grav != pytest.approx(q_nograv, abs=1e-4)

    def test_energy_conservation_no_damping_no_gravity(self):
        """Undamped, no-gravity system: mechanical energy should be nearly constant."""
        I = 0.05
        backend = MuJoCoBackend(_profile(inertia=I, damping=0.0, gravity=False, q0=0.5, dq0=2.0), dt=0.002)
        # Kinetic energy only (no potential in zero-gravity, no spring)
        q0, dq0 = backend.get_state()
        ke0 = 0.5 * I * dq0**2
        for _ in range(1000):
            backend.apply_torque_and_step(0.0)
        _, dq1 = backend.get_state()
        ke1 = 0.5 * I * dq1**2
        # MuJoCo RK4 conserves energy very well — allow 1% drift over 2 s
        assert ke1 == pytest.approx(ke0, rel=0.01)

    def test_multiple_steps_advance_time(self):
        """Running N steps at dt should match N * dt simulation time."""
        dt = 0.002
        backend = MuJoCoBackend(_profile(), dt=dt)
        for _ in range(100):
            backend.apply_torque_and_step(0.0)
        # MuJoCo time should be close to 100 * dt = 0.2 s
        assert backend.data.time == pytest.approx(100 * dt, abs=1e-9)


# ---------------------------------------------------------------------------
# OpenModelica backend — error handling when omc is absent
# ---------------------------------------------------------------------------

class TestOpenModelicaBackendErrors:
    def test_missing_omc_raises_runtime_error(self):
        """When omc is not installed, the backend should raise RuntimeError with install instructions."""
        with pytest.raises(RuntimeError, match="omc"):
            build_backend(_profile(), dt=0.002, backend_name="openmodelica")

    def test_error_message_contains_install_hint(self):
        try:
            build_backend(_profile(), dt=0.002, backend_name="openmodelica")
        except RuntimeError as exc:
            msg = str(exc)
            assert "brew" in msg or "openmodelica.org" in msg


# ---------------------------------------------------------------------------
# build_backend dispatch
# ---------------------------------------------------------------------------

class TestBuildBackend:
    def test_mujoco_dispatch(self):
        b = build_backend(_profile(), dt=0.002, backend_name="mujoco")
        assert isinstance(b, MuJoCoBackend)

    def test_unknown_backend_raises(self):
        with pytest.raises(ValueError, match="unsupported"):
            build_backend(_profile(), dt=0.002, backend_name="nonexistent")

    def test_error_message_lists_valid_choices(self):
        try:
            build_backend(_profile(), dt=0.002, backend_name="bad")
        except ValueError as exc:
            msg = str(exc)
            assert "mujoco" in msg
            assert "openmodelica" in msg
