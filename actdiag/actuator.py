from __future__ import annotations

from dataclasses import dataclass

import mujoco

from actdiag.config import ActuatorProfile, IdealTorqueActuatorProfile, PDActuatorProfile


@dataclass(frozen=True)
class ControlState:
    time: float
    q: float
    dq: float
    q_des: float
    dq_des: float
    qdd_des: float


def _clip_torque(torque: float, torque_limit: float) -> float:
    return max(-torque_limit, min(torque, torque_limit))


class PDActuator:
    def __init__(self, profile: PDActuatorProfile) -> None:
        self.profile = profile

    def compute(self, state: ControlState) -> tuple[float, float]:
        tau_cmd = (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )
        tau_applied = _clip_torque(tau_cmd, self.profile.torque_limit)
        return tau_cmd, tau_applied


class IdealTorqueActuator:
    def __init__(self, profile: IdealTorqueActuatorProfile, model: mujoco.MjModel) -> None:
        self.profile = profile
        self.model = model
        self.inverse_data = mujoco.MjData(model)

    def compute(self, state: ControlState) -> tuple[float, float]:
        self.inverse_data.qpos[0] = state.q_des
        self.inverse_data.qvel[0] = state.dq_des
        self.inverse_data.qacc[0] = state.qdd_des
        self.inverse_data.qfrc_applied[0] = 0.0
        mujoco.mj_inverse(self.model, self.inverse_data)

        feedforward = float(self.inverse_data.qfrc_inverse[0])
        feedback = (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )
        tau_cmd = feedforward + feedback
        tau_applied = _clip_torque(tau_cmd, self.profile.torque_limit)
        return tau_cmd, tau_applied


def build_actuator(profile: ActuatorProfile, model: mujoco.MjModel) -> PDActuator | IdealTorqueActuator:
    if isinstance(profile, PDActuatorProfile):
        return PDActuator(profile)
    if isinstance(profile, IdealTorqueActuatorProfile):
        return IdealTorqueActuator(profile, model)
    raise TypeError(f"unsupported actuator profile: {type(profile)!r}")

