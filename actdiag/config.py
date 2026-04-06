from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    NonNegativeFloat,
    PositiveFloat,
    field_validator,
    model_validator,
)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class IdealActuatorProfile(StrictModel):
    type: Literal["ideal_actuator"]
    torque_limit: PositiveFloat


ActuatorProfile = IdealActuatorProfile


class PDControllerProfile(StrictModel):
    type: Literal["pd"]
    kp: NonNegativeFloat
    kd: NonNegativeFloat


class InverseDynamicsControllerProfile(StrictModel):
    type: Literal["inverse_dynamics"]
    kp: NonNegativeFloat = 25.0
    kd: NonNegativeFloat = 6.0


class NoneControllerProfile(StrictModel):
    type: Literal["none"]


ControllerProfile = (
    PDControllerProfile | InverseDynamicsControllerProfile | NoneControllerProfile
)


class SingleJointConfig(StrictModel):
    inertia: PositiveFloat
    damping: NonNegativeFloat
    gravity: bool = False
    q0: float = 0.0
    dq0: float = 0.0

    @field_validator("q0", "dq0")
    @classmethod
    def validate_finite_state(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class SingleJointSceneProfile(StrictModel):
    scene_type: Literal["single_joint"]
    joint: SingleJointConfig


class StepTestProfile(StrictModel):
    test_type: Literal["step"]
    target: float
    start_time: NonNegativeFloat = 0.0
    duration: PositiveFloat
    dt: PositiveFloat

    @field_validator("target")
    @classmethod
    def validate_finite_target(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class SineTestProfile(StrictModel):
    test_type: Literal["sine"]
    amplitude: float = Field(..., description="Peak position amplitude in radians.")
    frequency: PositiveFloat
    offset: float = 0.0
    duration: PositiveFloat
    dt: PositiveFloat

    @field_validator("amplitude", "offset")
    @classmethod
    def validate_finite_signal(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class TorqueStepTestProfile(StrictModel):
    test_type: Literal["torque_step"]
    target_torque: float
    start_time: NonNegativeFloat = 0.0
    duration: PositiveFloat
    dt: PositiveFloat

    @field_validator("target_torque")
    @classmethod
    def validate_finite_target_torque(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class TorqueSineTestProfile(StrictModel):
    test_type: Literal["torque_sine"]
    amplitude: float
    frequency: PositiveFloat
    offset: float = 0.0
    duration: PositiveFloat
    dt: PositiveFloat

    @field_validator("amplitude", "offset")
    @classmethod
    def validate_finite_torque_signal(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class LoggingConfig(StrictModel):
    save_csv: bool = True


class PlotConfig(StrictModel):
    position: bool = True
    velocity: bool = True
    torque: bool = True
    error: bool = True
    phase: bool = True


class OutputConfig(StrictModel):
    save_video: bool = False


PositionTestProfile = StepTestProfile | SineTestProfile
TorqueTestProfile = TorqueStepTestProfile | TorqueSineTestProfile
TestProfile = PositionTestProfile | TorqueTestProfile


class RunConfig(StrictModel):
    actuator: ActuatorProfile
    controller: ControllerProfile
    scene: SingleJointSceneProfile
    test: TestProfile
    logging: LoggingConfig = LoggingConfig()
    plots: PlotConfig = PlotConfig()
    output: OutputConfig = OutputConfig()

    @model_validator(mode="after")
    def validate_controller_test_pairing(self) -> "RunConfig":
        position_test = isinstance(self.test, (StepTestProfile, SineTestProfile))
        torque_test = isinstance(
            self.test, (TorqueStepTestProfile, TorqueSineTestProfile)
        )

        if isinstance(self.controller, NoneControllerProfile) and not torque_test:
            raise ValueError("controller type 'none' requires a torque trajectory test")
        if (
            isinstance(
                self.controller, (PDControllerProfile, InverseDynamicsControllerProfile)
            )
            and not position_test
        ):
            raise ValueError(
                f"controller type '{self.controller.type}' requires a position trajectory test"
            )
        return self


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None:
        raise ValueError(f"{path} is empty")
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a top-level mapping")
    return data


def _parse_actuator_profile(data: dict[str, Any]) -> ActuatorProfile:
    actuator_type = data.get("type")
    normalized_type = {
        "ideal_torque": "ideal_actuator",
        "ideal_actuator": "ideal_actuator",
    }.get(actuator_type)
    if normalized_type == "ideal_actuator":
        return IdealActuatorProfile.model_validate({**data, "type": normalized_type})
    raise ValueError("unsupported actuator type")


def _parse_controller_profile(data: dict[str, Any]) -> ControllerProfile:
    controller_type = data.get("type")
    normalized_type = {
        "pd_position": "pd",
        "pd": "pd",
        "inverse_dynamics": "inverse_dynamics",
        "none": "none",
    }.get(controller_type)
    if normalized_type == "pd":
        return PDControllerProfile.model_validate({**data, "type": normalized_type})
    if normalized_type == "inverse_dynamics":
        return InverseDynamicsControllerProfile.model_validate(
            {**data, "type": normalized_type}
        )
    if normalized_type == "none":
        return NoneControllerProfile.model_validate({**data, "type": normalized_type})
    raise ValueError("unsupported controller type")


def _parse_scene_profile(data: dict[str, Any]) -> SingleJointSceneProfile:
    if "scene_type" in data:
        scene_type = data.get("scene_type")
        scene_data = data
    else:
        scene_type = data.get("type")
        scene_data = {
            "scene_type": scene_type,
            "joint": {
                "inertia": data.get("inertia"),
                "damping": data.get("damping", 0.0),
                "gravity": data.get("gravity", False),
                "q0": data.get("q0", 0.0),
                "dq0": data.get("dq0", 0.0),
            },
        }

    if scene_type != "single_joint":
        raise ValueError("unsupported scene type")
    return SingleJointSceneProfile.model_validate(scene_data)


def _merge_simulation_settings(
    test_data: dict[str, Any], simulation_data: dict[str, Any]
) -> dict[str, Any]:
    merged = dict(test_data)
    for key in ("duration", "dt"):
        if key in merged and key in simulation_data and merged[key] != simulation_data[key]:
            raise ValueError(f"conflicting values for test.{key} and simulation.{key}")
        if key not in merged and key in simulation_data:
            merged[key] = simulation_data[key]
    return merged


def _parse_test_profile(data: dict[str, Any]) -> TestProfile:
    test_type = data.get("test_type")
    if test_type == "step":
        return StepTestProfile.model_validate(data)
    if test_type == "sine":
        return SineTestProfile.model_validate(data)
    if test_type == "torque_step":
        return TorqueStepTestProfile.model_validate(data)
    if test_type == "torque_sine":
        return TorqueSineTestProfile.model_validate(data)
    raise ValueError("unsupported test type")


def _parse_user_test_profile(
    test_data: dict[str, Any], simulation_data: dict[str, Any]
) -> TestProfile:
    merged_data = _merge_simulation_settings(test_data, simulation_data)
    normalized_type = merged_data.get("type", merged_data.get("test_type"))
    normalized_data = dict(merged_data)
    normalized_data.pop("type", None)
    normalized_data["test_type"] = normalized_type
    return _parse_test_profile(normalized_data)


def load_run_config(system_path: Path, scenario_path: Path) -> RunConfig:
    system_data = _load_yaml(system_path)
    scenario_data = _load_yaml(scenario_path)

    controller = _parse_controller_profile(system_data.get("controller", {}))
    actuator = _parse_actuator_profile(system_data.get("actuator", {}))
    scene = _parse_scene_profile(scenario_data.get("scene", {}))
    test = _parse_user_test_profile(
        scenario_data.get("test", {}), scenario_data.get("simulation", {})
    )
    logging = LoggingConfig.model_validate(scenario_data.get("logging", {}))
    plots = PlotConfig.model_validate(scenario_data.get("plots", {}))
    output = OutputConfig.model_validate(scenario_data.get("output", {}))

    return RunConfig(
        actuator=actuator,
        controller=controller,
        scene=scene,
        test=test,
        logging=logging,
        plots=plots,
        output=output,
    )


def _controller_to_user_dict(profile: ControllerProfile) -> dict[str, Any]:
    data = profile.model_dump(mode="python")
    if data["type"] == "pd":
        data["type"] = "pd_position"
    return data


def _actuator_to_user_dict(profile: ActuatorProfile) -> dict[str, Any]:
    data = profile.model_dump(mode="python")
    if data["type"] == "ideal_actuator":
        data["type"] = "ideal_torque"
    return data


def _scene_to_user_dict(profile: SingleJointSceneProfile) -> dict[str, Any]:
    return {
        "type": profile.scene_type,
        "inertia": profile.joint.inertia,
        "damping": profile.joint.damping,
        "gravity": profile.joint.gravity,
        "q0": profile.joint.q0,
        "dq0": profile.joint.dq0,
    }


def _test_to_user_dict(profile: TestProfile) -> tuple[dict[str, Any], dict[str, Any]]:
    data = profile.model_dump(mode="python")
    simulation = {
        "duration": data.pop("duration"),
        "dt": data.pop("dt"),
    }
    data["type"] = data.pop("test_type")
    return data, simulation


def run_config_to_dict(run_config: RunConfig) -> dict[str, Any]:
    test_data, simulation_data = _test_to_user_dict(run_config.test)
    return {
        "system": {
            "controller": _controller_to_user_dict(run_config.controller),
            "actuator": _actuator_to_user_dict(run_config.actuator),
        },
        "scenario": {
            "scene": _scene_to_user_dict(run_config.scene),
            "test": test_data,
            "simulation": simulation_data,
            "logging": run_config.logging.model_dump(mode="python"),
            "plots": run_config.plots.model_dump(mode="python"),
            "output": run_config.output.model_dump(mode="python"),
        },
    }
