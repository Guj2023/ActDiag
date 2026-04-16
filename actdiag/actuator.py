from __future__ import annotations

from dataclasses import dataclass

from actdiag.config import (
    ActuatorProfile,
    DynamicTorqueActuatorProfile,
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


class DynamicTorqueActuator:
    def __init__(self, profile: DynamicTorqueActuatorProfile, dt: float) -> None:
        self.profile = profile
        self.dt = dt
        self.tau_prev = 0.0

    def apply(self, tau_cmd: float) -> ActuatorOutput:
        # 1. Deadzone
        tau_cmd_effective = tau_cmd
        if self.profile.deadzone is not None:
            if abs(tau_cmd) < self.profile.deadzone:
                tau_cmd_effective = 0.0

        # 2. Torque limit (Saturation)
        tau_target = _clip_torque(tau_cmd_effective, self.profile.torque_limit)

        # 3. Torque rate limit
        if self.profile.torque_rate_limit is not None:
            max_delta = self.profile.torque_rate_limit * self.dt
            tau_target = self.tau_prev + _clip_torque(
                tau_target - self.tau_prev, max_delta
            )

        # 4. First-order lag
        # alpha = dt / (time_constant + dt)
        # tau_next = tau_prev + alpha * (tau_target - tau_prev)
        alpha = self.dt / (self.profile.time_constant + self.dt)
        tau_next = self.tau_prev + alpha * (tau_target - self.tau_prev)

        self.tau_prev = tau_next

        return ActuatorOutput(
            tau_applied=tau_next,
            is_saturated=abs(tau_target - tau_cmd_effective) > 1e-12,
        )


def build_actuator(
    profile: ActuatorProfile,
    dt: float,
) -> IdealActuator | LimitedTorqueActuator | DynamicTorqueActuator:
    if isinstance(profile, IdealActuatorProfile):
        return IdealActuator(profile)
    if isinstance(profile, LimitedTorqueActuatorProfile):
        return LimitedTorqueActuator(profile)
    if isinstance(profile, DynamicTorqueActuatorProfile):
        return DynamicTorqueActuator(profile, dt)
    raise TypeError(f"unsupported actuator profile: {type(profile)!r}")
