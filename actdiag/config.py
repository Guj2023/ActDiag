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


class DynamicTorqueActuatorProfile(StrictModel):
    type: Literal["dynamic_torque"]
    torque_limit: PositiveFloat
    time_constant: PositiveFloat
    torque_rate_limit: PositiveFloat | None = None
    deadzone: NonNegativeFloat | None = None


ActuatorProfile = IdealActuatorProfile | DynamicTorqueActuatorProfile


class PDControllerProfile(StrictModel):
    type: Literal["pd"]
    kp: NonNegativeFloat
    kd: NonNegativeFloat


class PIDControllerProfile(StrictModel):
    type: Literal["pid"]
    kp: NonNegativeFloat
    ki: NonNegativeFloat
    kd: NonNegativeFloat


class InverseDynamicsControllerProfile(StrictModel):
    type: Literal["inverse_dynamics"]
    kp: NonNegativeFloat = 25.0
    kd: NonNegativeFloat = 6.0


class NoneControllerProfile(StrictModel):
    type: Literal["none"]


ControllerProfile = (
    PDControllerProfile
    | PIDControllerProfile
    | InverseDynamicsControllerProfile
    | NoneControllerProfile
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

    @field_validator("amplitude", "offset")
    @classmethod
    def validate_finite_signal(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class ChirpTestProfile(StrictModel):
    test_type: Literal["chirp"]
    amplitude: PositiveFloat
    f0: PositiveFloat
    f1: PositiveFloat
    duration: PositiveFloat
    offset: float = 0.0
    sweep: Literal["linear", "log"] = "linear"

    @field_validator("offset")
    @classmethod
    def validate_finite_offset(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value

    @model_validator(mode="after")
    def validate_frequencies(self) -> "ChirpTestProfile":
        if self.f1 < self.f0:
            raise ValueError("f1 must be greater than or equal to f0")
        return self


class FrequencyResponseTestProfile(StrictModel):
    test_type: Literal["frequency_response"]
    amplitude: PositiveFloat
    frequencies: list[PositiveFloat] = Field(..., min_length=1)
    cycles_per_frequency: int = Field(default=8, ge=1)
    settle_cycles: int = Field(default=3, ge=0)
    offset: float = 0.0

    @field_validator("amplitude", "offset")
    @classmethod
    def validate_finite_signal(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value

    @field_validator("frequencies")
    @classmethod
    def validate_finite_frequencies(cls, values: list[float]) -> list[float]:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("frequencies must contain only finite values")
        return values

    @model_validator(mode="after")
    def validate_cycles(self) -> "FrequencyResponseTestProfile":
        if self.settle_cycles >= self.cycles_per_frequency:
            raise ValueError("settle_cycles must be smaller than cycles_per_frequency")
        return self


class TorqueStepTestProfile(StrictModel):
    test_type: Literal["torque_step"]
    target_torque: float
    start_time: NonNegativeFloat = 0.0

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

    @field_validator("amplitude", "offset")
    @classmethod
    def validate_finite_torque_signal(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class SimulationConfig(StrictModel):
    duration: PositiveFloat | None = None
    dt: PositiveFloat
    backend: Literal["mujoco", "physx", "openmodelica"] = "mujoco"


class LoggingConfig(StrictModel):
    save_csv: bool = True


class PlotConfig(StrictModel):
    position: bool = True
    velocity: bool = True
    torque: bool = True
    error: bool = True
    phase: bool = True
    frequency_response: bool = True


class OutputConfig(StrictModel):
    save_video: bool = False


PositionTestProfile = (
    StepTestProfile | SineTestProfile | ChirpTestProfile | FrequencyResponseTestProfile
)
TorqueTestProfile = TorqueStepTestProfile | TorqueSineTestProfile
TestProfile = PositionTestProfile | TorqueTestProfile
SweepMetricName = Literal[
    "tracking_rmse",
    "max_abs_error",
    "stable",
    "jitter_metric",
]


class SweepParameterValues(StrictModel):
    # --- explicit list ---
    values: list[float] | None = None
    # --- range spec (resolves to values at validation time) ---
    min: float | None = None
    max: float | None = None
    step: float | None = None   # arange-style: [min, min+step, …, max]
    num: int | None = None      # linspace/logspace: N evenly-spaced points
    scale: Literal["linear", "log"] = "linear"

    @model_validator(mode="after")
    def resolve_values(self) -> "SweepParameterValues":
        if self.values is not None:
            if not self.values:
                raise ValueError("values must not be empty")
            if not all(math.isfinite(v) for v in self.values):
                raise ValueError("values must contain only finite numbers")
            return self

        if self.min is None or self.max is None:
            raise ValueError(
                "specify 'values' or a range with 'min'+'max' and 'step'/'num'"
            )
        if not (math.isfinite(self.min) and math.isfinite(self.max)):
            raise ValueError("'min' and 'max' must be finite")
        if self.min >= self.max:
            raise ValueError("'min' must be less than 'max'")
        if self.step is not None and self.num is not None:
            raise ValueError("specify either 'step' or 'num', not both")
        if self.step is None and self.num is None:
            raise ValueError("specify 'step' or 'num' when using a range")

        import numpy as _np

        if self.step is not None:
            if self.step <= 0:
                raise ValueError("'step' must be positive")
            resolved = [
                float(v)
                for v in _np.arange(self.min, self.max + self.step * 0.5, self.step)
            ]
        else:
            if self.num < 2:
                raise ValueError("'num' must be at least 2")
            if self.scale == "log":
                if self.min <= 0:
                    raise ValueError("'min' must be positive for log scale")
                resolved = [
                    float(v)
                    for v in _np.logspace(
                        _np.log10(self.min), _np.log10(self.max), self.num
                    )
                ]
            else:
                resolved = [
                    float(v) for v in _np.linspace(self.min, self.max, self.num)
                ]

        if not resolved:
            raise ValueError("range spec produced no values")
        self.values = resolved
        return self


class SweepConfig(StrictModel):
    parameters: dict[str, SweepParameterValues]
    metrics: list[SweepMetricName] = Field(..., min_length=1)

    @model_validator(mode="after")
    def validate_structure(self) -> "SweepConfig":
        parameter_count = len(self.parameters)
        if parameter_count not in (1, 2):
            raise ValueError("only 1D and 2D sweeps are supported")

        for path in self.parameters:
            root = path.split(".", 1)[0]
            if root not in {"controller", "actuator"}:
                raise ValueError(
                    "sweep parameters must target controller.* or actuator.* fields"
                )

        if len(set(self.metrics)) != len(self.metrics):
            raise ValueError("metrics must not contain duplicates")

        return self


def derive_frequency_response_duration(profile: FrequencyResponseTestProfile) -> float:
    return sum(
        profile.cycles_per_frequency / frequency for frequency in profile.frequencies
    )


class RunConfig(StrictModel):
    actuator: ActuatorProfile
    controller: ControllerProfile
    scene: SingleJointSceneProfile
    test: TestProfile
    simulation: SimulationConfig
    logging: LoggingConfig = LoggingConfig()
    plots: PlotConfig = PlotConfig()
    output: OutputConfig = OutputConfig()

    @model_validator(mode="after")
    def validate_controller_test_pairing(self) -> "RunConfig":
        position_test = isinstance(
            self.test,
            (
                StepTestProfile,
                SineTestProfile,
                ChirpTestProfile,
                FrequencyResponseTestProfile,
            ),
        )
        torque_test = isinstance(
            self.test, (TorqueStepTestProfile, TorqueSineTestProfile)
        )

        if isinstance(self.controller, NoneControllerProfile) and not torque_test:
            raise ValueError("controller type 'none' requires a torque trajectory test")
        if (
            isinstance(
                self.controller,
                (
                    PDControllerProfile,
                    PIDControllerProfile,
                    InverseDynamicsControllerProfile,
                ),
            )
            and not position_test
        ):
            raise ValueError(
                f"controller type '{self.controller.type}' requires a position trajectory test"
            )

        if isinstance(self.test, FrequencyResponseTestProfile):
            self.simulation.duration = derive_frequency_response_duration(self.test)
        elif self.simulation.duration is None:
            raise ValueError("simulation.duration is required for this test type")

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
        "limited_torque": "ideal_actuator",  # alias — identical behaviour
        "dynamic_torque": "dynamic_torque",
    }.get(actuator_type)
    if normalized_type == "ideal_actuator":
        return IdealActuatorProfile.model_validate({**data, "type": normalized_type})
    if normalized_type == "dynamic_torque":
        return DynamicTorqueActuatorProfile.model_validate(
            {**data, "type": normalized_type}
        )
    raise ValueError("unsupported actuator type")


def _parse_controller_profile(data: dict[str, Any]) -> ControllerProfile:
    controller_type = data.get("type")
    normalized_type = {
        "pd_position": "pd",
        "pd": "pd",
        "pid_position": "pid",
        "pid": "pid",
        "inverse_dynamics": "inverse_dynamics",
        "none": "none",
    }.get(controller_type)
    if normalized_type == "pd":
        return PDControllerProfile.model_validate({**data, "type": normalized_type})
    if normalized_type == "pid":
        return PIDControllerProfile.model_validate({**data, "type": normalized_type})
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


def _normalize_test_and_simulation_data(
    test_data: dict[str, Any], simulation_data: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    normalized_test = dict(test_data)
    normalized_simulation = dict(simulation_data)

    for key in ("duration", "dt"):
        if key in normalized_test:
            if (
                key in normalized_simulation
                and normalized_simulation[key] != normalized_test[key]
            ):
                raise ValueError(
                    f"conflicting values for test.{key} and simulation.{key}"
                )
            normalized_simulation.setdefault(key, normalized_test.pop(key))

    return normalized_test, normalized_simulation


def _parse_test_profile(data: dict[str, Any]) -> TestProfile:
    test_type = data.get("test_type")
    if test_type == "step":
        return StepTestProfile.model_validate(data)
    if test_type == "sine":
        return SineTestProfile.model_validate(data)
    if test_type == "chirp":
        return ChirpTestProfile.model_validate(data)
    if test_type == "frequency_response":
        return FrequencyResponseTestProfile.model_validate(data)
    if test_type == "torque_step":
        return TorqueStepTestProfile.model_validate(data)
    if test_type == "torque_sine":
        return TorqueSineTestProfile.model_validate(data)
    raise ValueError("unsupported test type")


def _parse_user_test_profile(test_data: dict[str, Any]) -> TestProfile:
    normalized_type = test_data.get("type", test_data.get("test_type"))
    normalized_data = dict(test_data)
    normalized_data.pop("type", None)
    normalized_data["test_type"] = normalized_type
    return _parse_test_profile(normalized_data)


def load_run_config_from_data(
    system_data: dict[str, Any], scenario_data: dict[str, Any]
) -> RunConfig:
    test_data, simulation_data = _normalize_test_and_simulation_data(
        scenario_data.get("test", {}) or {},
        scenario_data.get("simulation", {}) or {},
    )

    # For chirp, we need duration in the test profile as well
    if test_data.get("type") == "chirp" or test_data.get("test_type") == "chirp":
        if "duration" in simulation_data and "duration" not in test_data:
            test_data["duration"] = simulation_data["duration"]

    controller = _parse_controller_profile(system_data.get("controller", {}) or {})
    actuator = _parse_actuator_profile(system_data.get("actuator", {}) or {})
    scene = _parse_scene_profile(scenario_data.get("scene", {}) or {})
    test = _parse_user_test_profile(test_data)
    simulation = SimulationConfig.model_validate(simulation_data)
    logging = LoggingConfig.model_validate(scenario_data.get("logging", {}) or {})
    plots = PlotConfig.model_validate(scenario_data.get("plots", {}) or {})
    output = OutputConfig.model_validate(scenario_data.get("output", {}) or {})

    return RunConfig(
        actuator=actuator,
        controller=controller,
        scene=scene,
        test=test,
        simulation=simulation,
        logging=logging,
        plots=plots,
        output=output,
    )


def load_yaml_mapping(path: Path) -> dict[str, Any]:
    return _load_yaml(path)


def load_run_config(system_path: Path, scenario_path: Path) -> RunConfig:
    return load_run_config_from_data(
        load_yaml_mapping(system_path),
        load_yaml_mapping(scenario_path),
    )


def load_sweep_config(path: Path) -> SweepConfig:
    data = load_yaml_mapping(path)
    if "sweep" not in data:
        raise ValueError(f"Missing 'sweep' section in {path}")
    return SweepConfig.model_validate(data["sweep"])


def _controller_to_user_dict(profile: ControllerProfile) -> dict[str, Any]:
    data = profile.model_dump(mode="python")
    if data["type"] == "pd":
        data["type"] = "pd_position"
    if data["type"] == "pid":
        data["type"] = "pid_position"
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


def _test_to_user_dict(profile: TestProfile) -> dict[str, Any]:
    data = profile.model_dump(mode="python")
    data["type"] = data.pop("test_type")
    return data


def run_config_to_dict(run_config: RunConfig) -> dict[str, Any]:
    return {
        "system": {
            "controller": _controller_to_user_dict(run_config.controller),
            "actuator": _actuator_to_user_dict(run_config.actuator),
        },
        "scenario": {
            "scene": _scene_to_user_dict(run_config.scene),
            "test": _test_to_user_dict(run_config.test),
            "simulation": run_config.simulation.model_dump(mode="python"),
            "logging": run_config.logging.model_dump(mode="python"),
            "plots": run_config.plots.model_dump(mode="python"),
            "output": run_config.output.model_dump(mode="python"),
        },
    }
