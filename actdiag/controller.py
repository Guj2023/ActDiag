from __future__ import annotations

from dataclasses import dataclass

import mujoco

from actdiag.config import (
    ControllerProfile,
    InverseDynamicsControllerProfile,
    NoneControllerProfile,
    PDControllerProfile,
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


class PDController:
    def __init__(self, profile: PDControllerProfile) -> None:
        self.profile = profile

    def compute(self, state: ControlState) -> float:
        return (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )


class InverseDynamicsController:
    def __init__(
        self, profile: InverseDynamicsControllerProfile, model: mujoco.MjModel
    ) -> None:
        self.profile = profile
        self.model = model
        self.inverse_data = mujoco.MjData(model)

    def compute(self, state: ControlState) -> float:
        self.inverse_data.qpos[0] = state.q_des
        self.inverse_data.qvel[0] = state.dq_des
        self.inverse_data.qacc[0] = state.qdd_des
        self.inverse_data.qfrc_applied[0] = 0.0
        mujoco.mj_inverse(self.model, self.inverse_data)

        feedforward = float(self.inverse_data.qfrc_inverse[0])
        feedback = (self.profile.kp * (state.q_des - state.q)) + (
            self.profile.kd * (state.dq_des - state.dq)
        )
        return feedforward + feedback


class NoController:
    def __init__(self, profile: NoneControllerProfile) -> None:
        self.profile = profile

    def compute(self, state: ControlState) -> float:
        return state.tau_des


def build_controller(
    profile: ControllerProfile, model: mujoco.MjModel
) -> PDController | InverseDynamicsController | NoController:
    if isinstance(profile, PDControllerProfile):
        return PDController(profile)
    if isinstance(profile, InverseDynamicsControllerProfile):
        return InverseDynamicsController(profile, model)
    if isinstance(profile, NoneControllerProfile):
        return NoController(profile)
    raise TypeError(f"unsupported controller profile: {type(profile)!r}")
