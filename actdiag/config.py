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


PositionTestProfile = StepTestProfile | SineTestProfile
TorqueTestProfile = TorqueStepTestProfile | TorqueSineTestProfile
TestProfile = PositionTestProfile | TorqueTestProfile


class RunConfig(StrictModel):
    actuator: ActuatorProfile
    controller: ControllerProfile
    scene: SingleJointSceneProfile
    test: TestProfile

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
    if actuator_type == "ideal_actuator":
        return IdealActuatorProfile.model_validate(data)
    raise ValueError("unsupported actuator type")


def _parse_controller_profile(data: dict[str, Any]) -> ControllerProfile:
    controller_type = data.get("type")
    if controller_type == "pd":
        return PDControllerProfile.model_validate(data)
    if controller_type == "inverse_dynamics":
        return InverseDynamicsControllerProfile.model_validate(data)
    if controller_type == "none":
        return NoneControllerProfile.model_validate(data)
    raise ValueError("unsupported controller type")


def _parse_scene_profile(data: dict[str, Any]) -> SingleJointSceneProfile:
    scene_type = data.get("scene_type")
    if scene_type != "single_joint":
        raise ValueError("unsupported scene type")
    return SingleJointSceneProfile.model_validate(data)


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


def load_run_config(
    actuator_path: Path, controller_path: Path, scene_path: Path, test_path: Path
) -> RunConfig:
    actuator = _parse_actuator_profile(_load_yaml(actuator_path))
    controller = _parse_controller_profile(_load_yaml(controller_path))
    scene = _parse_scene_profile(_load_yaml(scene_path))
    test = _parse_test_profile(_load_yaml(test_path))
    return RunConfig(
        actuator=actuator,
        controller=controller,
        scene=scene,
        test=test,
    )


def run_config_to_dict(run_config: RunConfig) -> dict[str, Any]:
    return run_config.model_dump(mode="python")
