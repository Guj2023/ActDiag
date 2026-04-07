from __future__ import annotations

from dataclasses import dataclass

import mujoco

from actdiag.config import (
    ControllerProfile,
    InverseDynamicsControllerProfile,
    NoneControllerProfile,
    PDControllerProfile,
    PIDControllerProfile,
)


@dataclass(frozen=True)
class ControlState:
    time: float
    q: float
    dq: float
    q_des: float
    dq_des: float
    qdd_des: float
    tau_des: float


@dataclass(frozen=True)
class ControlOutput:
    tau_cmd: float
    integral_error: float


class PDController:
    def __init__(self, profile: PDControllerProfile) -> None:
        self.profile = profile

    def compute(self, state: ControlState) -> ControlOutput:
        tau_cmd = (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )
        return ControlOutput(tau_cmd=tau_cmd, integral_error=0.0)


class PIDController:
    def __init__(self, profile: PIDControllerProfile, dt: float) -> None:
        self.profile = profile
        self.dt = dt
        self.integral_error = 0.0

    def compute(self, state: ControlState) -> ControlOutput:
        error = state.q_des - state.q
        self.integral_error += error * self.dt
        tau_cmd = (
            self.profile.kp * error
            + self.profile.ki * self.integral_error
            + self.profile.kd * (state.dq_des - state.dq)
        )
        return ControlOutput(tau_cmd=tau_cmd, integral_error=self.integral_error)


class InverseDynamicsController:
    def __init__(
        self, profile: InverseDynamicsControllerProfile, model: mujoco.MjModel
    ) -> None:
        self.profile = profile
        self.model = model
        self.inverse_data = mujoco.MjData(model)

    def compute(self, state: ControlState) -> ControlOutput:
        self.inverse_data.qpos[0] = state.q_des
        self.inverse_data.qvel[0] = state.dq_des
        self.inverse_data.qacc[0] = state.qdd_des
        self.inverse_data.qfrc_applied[0] = 0.0
        mujoco.mj_inverse(self.model, self.inverse_data)

        feedforward = float(self.inverse_data.qfrc_inverse[0])
        feedback = (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )
        return ControlOutput(tau_cmd=feedforward + feedback, integral_error=0.0)


class NoController:
    def __init__(self, profile: NoneControllerProfile) -> None:
        self.profile = profile

    def compute(self, state: ControlState) -> ControlOutput:
        return ControlOutput(tau_cmd=state.tau_des, integral_error=0.0)


def build_controller(
    profile: ControllerProfile, model: mujoco.MjModel, dt: float
) -> PDController | PIDController | InverseDynamicsController | NoController:
    if isinstance(profile, PDControllerProfile):
        return PDController(profile)
    if isinstance(profile, PIDControllerProfile):
        return PIDController(profile, dt)
    if isinstance(profile, InverseDynamicsControllerProfile):
        return InverseDynamicsController(profile, model)
    if isinstance(profile, NoneControllerProfile):
        return NoController(profile)
    raise TypeError(f"unsupported controller profile: {type(profile)!r}")
