from __future__ import annotations

from actdiag.config import ActuatorProfile, IdealActuatorProfile


def _clip_torque(torque: float, torque_limit: float) -> float:
    return max(-torque_limit, min(torque, torque_limit))


class IdealActuator:
    def __init__(self, profile: IdealActuatorProfile) -> None:
        self.profile = profile

    def apply(self, tau_cmd: float) -> float:
        return _clip_torque(tau_cmd, self.profile.torque_limit)


def build_actuator(profile: ActuatorProfile) -> IdealActuator:
    if isinstance(profile, IdealActuatorProfile):
        return IdealActuator(profile)
    raise TypeError(f"unsupported actuator profile: {type(profile)!r}")
