from __future__ import annotations

from dataclasses import dataclass

from actdiag.config import (
    ActuatorProfile,
    IdealActuatorProfile,
    LimitedTorqueActuatorProfile,
)


def _clip_torque(torque: float, torque_limit: float) -> float:
    return max(-torque_limit, min(torque, torque_limit))


@dataclass(frozen=True)
class ActuatorOutput:
    tau_applied: float
    is_saturated: bool


class IdealActuator:
    def __init__(self, profile: IdealActuatorProfile) -> None:
        self.profile = profile

    def apply(self, tau_cmd: float) -> ActuatorOutput:
        tau_applied = _clip_torque(tau_cmd, self.profile.torque_limit)
        return ActuatorOutput(
            tau_applied=tau_applied,
            is_saturated=abs(tau_applied - tau_cmd) > 1e-12,
        )


class LimitedTorqueActuator:
    def __init__(self, profile: LimitedTorqueActuatorProfile) -> None:
        self.profile = profile

    def apply(self, tau_cmd: float) -> ActuatorOutput:
        tau_applied = _clip_torque(tau_cmd, self.profile.torque_limit)
        return ActuatorOutput(
            tau_applied=tau_applied,
            is_saturated=abs(tau_applied - tau_cmd) > 1e-12,
        )


def build_actuator(
    profile: ActuatorProfile,
) -> IdealActuator | LimitedTorqueActuator:
    if isinstance(profile, IdealActuatorProfile):
        return IdealActuator(profile)
    if isinstance(profile, LimitedTorqueActuatorProfile):
        return LimitedTorqueActuator(profile)
    raise TypeError(f"unsupported actuator profile: {type(profile)!r}")
