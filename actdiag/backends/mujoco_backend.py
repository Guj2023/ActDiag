from __future__ import annotations

import mujoco

from actdiag.config import SingleJointSceneProfile
from actdiag.scene import build_single_joint_model, initialize_scene_state


class MuJoCoBackend:
    """MuJoCo physics backend — wraps a single-joint MjModel/MjData pair."""

    name = "mujoco"

    def __init__(self, scene: SingleJointSceneProfile, dt: float) -> None:
        self.dt = dt
        self._scene = scene
        self.model = build_single_joint_model(scene, dt)
        self.data = initialize_scene_state(self.model, scene)

    def get_state(self) -> tuple[float, float]:
        return float(self.data.qpos[0]), float(self.data.qvel[0])

    def apply_torque_and_step(self, torque: float) -> None:
        self.data.qfrc_applied[0] = torque
        mujoco.mj_step(self.model, self.data)
        self.data.qfrc_applied[0] = 0.0
