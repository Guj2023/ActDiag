"""Tests for actdiag.config — YAML parsing, validation, RunConfig assembly."""
from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from actdiag.config import (
    IdealActuatorProfile,
    DynamicTorqueActuatorProfile,
    PDControllerProfile,
    PIDControllerProfile,
    InverseDynamicsControllerProfile,
    NoneControllerProfile,
    SingleJointConfig,
    SingleJointSceneProfile,
    SimulationConfig,
    StepTestProfile,
    SineTestProfile,
    FrequencyResponseTestProfile,
    TorqueStepTestProfile,
    SweepConfig,
    SweepParameterValues,
    load_run_config_from_data,
    run_config_to_dict,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _system(controller=None, actuator=None):
    return {
        "controller": controller or {"type": "pd", "kp": 10.0, "kd": 1.0},
        "actuator": actuator or {"type": "ideal_torque", "torque_limit": 5.0},
    }


def _scenario(test=None, simulation=None, scene=None):
    return {
        "scene": scene or {"type": "single_joint", "inertia": 0.05, "damping": 0.1},
        "test": test or {"type": "step", "target": 1.0},
        "simulation": simulation or {"duration": 2.0, "dt": 0.01},
    }


def _run_config(**kwargs):
    s = dict(_scenario())
    s.update(kwargs.get("scenario_overrides", {}))
    sys = dict(_system())
    sys.update(kwargs.get("system_overrides", {}))
    return load_run_config_from_data(sys, s)


# ---------------------------------------------------------------------------
# Actuator profiles
# ---------------------------------------------------------------------------

class TestActuatorProfiles:
    def test_ideal_actuator_valid(self):
        p = IdealActuatorProfile(type="ideal_actuator", torque_limit=5.0)
        assert p.torque_limit == 5.0

    def test_ideal_actuator_zero_limit_rejected(self):
        with pytest.raises(ValidationError):
            IdealActuatorProfile(type="ideal_actuator", torque_limit=0.0)

    def test_dynamic_torque_valid(self):
        p = DynamicTorqueActuatorProfile(
            type="dynamic_torque",
            torque_limit=5.0,
            time_constant=0.05,
        )
        assert p.torque_rate_limit is None
        assert p.deadzone is None

    def test_dynamic_torque_all_fields(self):
        p = DynamicTorqueActuatorProfile(
            type="dynamic_torque",
            torque_limit=3.0,
            time_constant=0.02,
            torque_rate_limit=50.0,
            deadzone=0.1,
        )
        assert p.torque_rate_limit == 50.0
        assert p.deadzone == 0.1


# ---------------------------------------------------------------------------
# Controller profiles
# ---------------------------------------------------------------------------

class TestControllerProfiles:
    def test_pd_valid(self):
        p = PDControllerProfile(type="pd", kp=10.0, kd=1.0)
        assert p.kp == 10.0

    def test_pid_valid(self):
        p = PIDControllerProfile(type="pid", kp=5.0, ki=0.1, kd=0.5)
        assert p.ki == 0.1

    def test_pd_negative_kp_rejected(self):
        with pytest.raises(ValidationError):
            PDControllerProfile(type="pd", kp=-1.0, kd=0.0)

    def test_none_valid(self):
        p = NoneControllerProfile(type="none")
        assert p.type == "none"


# ---------------------------------------------------------------------------
# SingleJointConfig
# ---------------------------------------------------------------------------

class TestSingleJointConfig:
    def test_valid(self):
        c = SingleJointConfig(inertia=0.05, damping=0.1, gravity=True, q0=0.5, dq0=0.0)
        assert c.gravity is True

    def test_zero_damping_allowed(self):
        c = SingleJointConfig(inertia=0.05, damping=0.0)
        assert c.damping == 0.0

    def test_negative_inertia_rejected(self):
        with pytest.raises(ValidationError):
            SingleJointConfig(inertia=-0.05, damping=0.0)

    def test_nan_q0_rejected(self):
        with pytest.raises(ValidationError):
            SingleJointConfig(inertia=0.05, damping=0.0, q0=float("nan"))

    def test_inf_dq0_rejected(self):
        with pytest.raises(ValidationError):
            SingleJointConfig(inertia=0.05, damping=0.0, dq0=float("inf"))


# ---------------------------------------------------------------------------
# SimulationConfig
# ---------------------------------------------------------------------------

class TestSimulationConfig:
    def test_default_backend_is_mujoco(self):
        cfg = SimulationConfig(dt=0.002)
        assert cfg.backend == "mujoco"

    def test_physx_backend(self):
        cfg = SimulationConfig(dt=0.002, backend="physx")
        assert cfg.backend == "physx"

    def test_openmodelica_backend(self):
        cfg = SimulationConfig(dt=0.002, backend="openmodelica")
        assert cfg.backend == "openmodelica"

    def test_invalid_backend_rejected(self):
        with pytest.raises(ValidationError):
            SimulationConfig(dt=0.002, backend="genesis")

    def test_zero_dt_rejected(self):
        with pytest.raises(ValidationError):
            SimulationConfig(dt=0.0)


# ---------------------------------------------------------------------------
# load_run_config_from_data — type aliases and merging
# ---------------------------------------------------------------------------

class TestLoadRunConfig:
    def test_basic_step(self):
        rc = _run_config()
        assert rc.simulation.backend == "mujoco"
        assert rc.simulation.duration == pytest.approx(2.0)
        assert rc.simulation.dt == pytest.approx(0.01)

    def test_ideal_torque_alias(self):
        rc = _run_config(system_overrides={
            "actuator": {"type": "ideal_torque", "torque_limit": 3.0}
        })
        assert isinstance(rc.actuator, IdealActuatorProfile)

    def test_pd_position_alias(self):
        rc = _run_config(system_overrides={
            "controller": {"type": "pd_position", "kp": 5.0, "kd": 0.5}
        })
        assert isinstance(rc.controller, PDControllerProfile)

    def test_sine_test(self):
        rc = _run_config(scenario_overrides={
            "test": {"type": "sine", "amplitude": 0.5, "frequency": 1.0},
            "simulation": {"duration": 3.0, "dt": 0.01},
        })
        assert isinstance(rc.test, SineTestProfile)

    def test_frequency_response_duration_derived(self):
        """duration must not be set by user; it is derived from the test."""
        rc = load_run_config_from_data(
            _system(),
            {
                "scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1},
                "test": {
                    "type": "frequency_response",
                    "amplitude": 0.3,
                    "frequencies": [1.0, 2.0],
                    "cycles_per_frequency": 8,
                    "settle_cycles": 3,
                },
                "simulation": {"dt": 0.002},
            },
        )
        assert rc.simulation.duration is not None
        assert rc.simulation.duration > 0

    def test_none_controller_requires_torque_test(self):
        with pytest.raises(Exception):
            load_run_config_from_data(
                {"controller": {"type": "none"}, "actuator": {"type": "ideal_torque", "torque_limit": 5.0}},
                _scenario(),  # step test — position test, not torque
            )

    def test_pd_controller_requires_position_test(self):
        with pytest.raises(Exception):
            load_run_config_from_data(
                _system(),
                {
                    "scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1},
                    "test": {"type": "torque_step", "target_torque": 1.0},
                    "simulation": {"duration": 1.0, "dt": 0.01},
                },
            )

    def test_missing_duration_raises(self):
        with pytest.raises(Exception):
            load_run_config_from_data(
                _system(),
                {
                    "scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1},
                    "test": {"type": "step", "target": 1.0},
                    "simulation": {"dt": 0.01},  # no duration
                },
            )

    def test_extra_field_inside_model_rejected(self):
        """StrictModel — unknown fields *inside* a model dict are forbidden."""
        with pytest.raises(Exception):
            load_run_config_from_data(
                {
                    "controller": {"type": "pd", "kp": 10.0, "kd": 1.0, "unknown_gain": 9},
                    "actuator": {"type": "ideal_torque", "torque_limit": 5.0},
                },
                _scenario(),
            )

    def test_gravity_false_default(self):
        rc = _run_config()
        assert rc.scene.joint.gravity is False

    def test_gravity_true(self):
        rc = load_run_config_from_data(
            _system(),
            {
                "scene": {"type": "single_joint", "inertia": 0.05, "damping": 0.1, "gravity": True},
                "test": {"type": "step", "target": 1.0},
                "simulation": {"duration": 1.0, "dt": 0.01},
            },
        )
        assert rc.scene.joint.gravity is True


# ---------------------------------------------------------------------------
# run_config_to_dict round-trip
# ---------------------------------------------------------------------------

class TestRunConfigRoundTrip:
    def test_round_trip(self):
        rc = _run_config()
        d = run_config_to_dict(rc)
        assert "system" in d and "scenario" in d
        assert d["scenario"]["simulation"]["backend"] == "mujoco"
        assert d["scenario"]["simulation"]["dt"] == pytest.approx(0.01)

    def test_pd_alias_preserved(self):
        rc = _run_config()
        d = run_config_to_dict(rc)
        assert d["system"]["controller"]["type"] == "pd_position"


# ---------------------------------------------------------------------------
# SweepParameterValues
# ---------------------------------------------------------------------------

class TestSweepParameterValues:
    def test_explicit_values(self):
        p = SweepParameterValues(values=[1.0, 2.0, 3.0])
        assert p.values == [1.0, 2.0, 3.0]

    def test_range_step(self):
        p = SweepParameterValues(min=0.0, max=1.0, step=0.5)
        assert len(p.values) == 3
        assert p.values[0] == pytest.approx(0.0)
        assert p.values[-1] == pytest.approx(1.0)

    def test_range_num_linspace(self):
        p = SweepParameterValues(min=1.0, max=10.0, num=4)
        assert len(p.values) == 4
        assert p.values[0] == pytest.approx(1.0)
        assert p.values[-1] == pytest.approx(10.0)

    def test_range_num_logspace(self):
        p = SweepParameterValues(min=1.0, max=100.0, num=3, scale="log")
        assert len(p.values) == 3
        assert p.values[0] == pytest.approx(1.0)
        assert p.values[-1] == pytest.approx(100.0)
        assert p.values[1] == pytest.approx(10.0, rel=1e-6)

    def test_step_and_num_mutually_exclusive(self):
        with pytest.raises(Exception):
            SweepParameterValues(min=0.0, max=1.0, step=0.1, num=5)

    def test_empty_values_rejected(self):
        with pytest.raises(Exception):
            SweepParameterValues(values=[])


# ---------------------------------------------------------------------------
# SweepConfig
# ---------------------------------------------------------------------------

class TestSweepConfig:
    def test_1d_sweep_valid(self):
        cfg = SweepConfig(
            parameters={"controller.kp": SweepParameterValues(values=[1.0, 2.0])},
            metrics=["tracking_rmse"],
        )
        assert len(cfg.parameters) == 1

    def test_2d_sweep_valid(self):
        cfg = SweepConfig(
            parameters={
                "controller.kp": SweepParameterValues(values=[1.0, 2.0]),
                "controller.kd": SweepParameterValues(values=[0.1, 0.2]),
            },
            metrics=["tracking_rmse", "stable"],
        )
        assert len(cfg.parameters) == 2

    def test_3d_sweep_rejected(self):
        with pytest.raises(Exception):
            SweepConfig(
                parameters={
                    "controller.kp": SweepParameterValues(values=[1.0]),
                    "controller.kd": SweepParameterValues(values=[1.0]),
                    "actuator.torque_limit": SweepParameterValues(values=[1.0]),
                },
                metrics=["tracking_rmse"],
            )

    def test_non_controller_parameter_rejected(self):
        with pytest.raises(Exception):
            SweepConfig(
                parameters={"scene.inertia": SweepParameterValues(values=[0.05])},
                metrics=["tracking_rmse"],
            )

    def test_duplicate_metrics_rejected(self):
        with pytest.raises(Exception):
            SweepConfig(
                parameters={"controller.kp": SweepParameterValues(values=[1.0])},
                metrics=["tracking_rmse", "tracking_rmse"],
            )
