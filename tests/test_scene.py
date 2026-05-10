"""Tests for actdiag.scene — MuJoCo model generation."""
from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

from actdiag.config import SingleJointConfig, SingleJointSceneProfile
from actdiag.scene import build_single_joint_model, initialize_scene_state


def _profile(inertia=0.05, damping=0.1, gravity=False, q0=0.0, dq0=0.0):
    return SingleJointSceneProfile(
        scene_type="single_joint",
        joint=SingleJointConfig(
            inertia=inertia, damping=damping, gravity=gravity, q0=q0, dq0=dq0
        ),
    )


class TestBuildSingleJointModel:
    def test_returns_mujoco_model(self):
        model = build_single_joint_model(_profile(), dt=0.002)
        assert isinstance(model, mujoco.MjModel)

    def test_one_degree_of_freedom(self):
        model = build_single_joint_model(_profile(), dt=0.002)
        assert model.nq == 1
        assert model.nv == 1

    def test_timestep_set(self):
        model = build_single_joint_model(_profile(), dt=0.005)
        assert model.opt.timestep == pytest.approx(0.005)

    def test_gravity_disabled_when_false(self):
        model = build_single_joint_model(_profile(gravity=False), dt=0.002)
        np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, 0.0])

    def test_gravity_enabled_when_true(self):
        model = build_single_joint_model(_profile(gravity=True), dt=0.002)
        assert model.opt.gravity[2] == pytest.approx(-9.81)

    def test_damping_applied_to_joint(self):
        model = build_single_joint_model(_profile(damping=0.3), dt=0.002)
        # MuJoCo stores joint damping in dof_damping
        assert model.dof_damping[0] == pytest.approx(0.3)

    def test_inertia_affects_body_inertia(self):
        model_low = build_single_joint_model(_profile(inertia=0.02), dt=0.002)
        model_high = build_single_joint_model(_profile(inertia=0.10), dt=0.002)
        # Higher inertia → heavier/larger body
        assert model_high.body_mass[1] >= model_low.body_mass[1]


class TestInitializeSceneState:
    def test_initial_q0(self):
        profile = _profile(q0=0.7)
        model = build_single_joint_model(profile, dt=0.002)
        data = initialize_scene_state(model, profile)
        assert data.qpos[0] == pytest.approx(0.7)

    def test_initial_dq0(self):
        profile = _profile(dq0=1.2)
        model = build_single_joint_model(profile, dt=0.002)
        data = initialize_scene_state(model, profile)
        assert data.qvel[0] == pytest.approx(1.2)

    def test_zero_initial_conditions(self):
        profile = _profile()
        model = build_single_joint_model(profile, dt=0.002)
        data = initialize_scene_state(model, profile)
        assert data.qpos[0] == pytest.approx(0.0)
        assert data.qvel[0] == pytest.approx(0.0)

    def test_returns_mujoco_data(self):
        profile = _profile()
        model = build_single_joint_model(profile, dt=0.002)
        data = initialize_scene_state(model, profile)
        assert isinstance(data, mujoco.MjData)
