"""PhysX physics backend using pyphysx (PhysX 4 Python bindings).

Installation
------------
pyphysx must be built from source with the arm64 patches applied.
See the project README for step-by-step instructions.

Scene geometry
--------------
The single joint is modelled as a fixed anchor body connected to a freely
rotating dynamic link via a PhysX D6 joint with only the TWIST DOF free.
The TWIST axis (joint-local X) is oriented to align with world Y by rotating
the joint frames 90° around Z, so TWIST == Y-rotation.

    anchor ──[D6Joint, TWIST=FREE(Y), rest=LOCKED]── link (dynamic)

State is read back as the rotation angle of the link around the world Y-axis,
extracted from its quaternion, and the Y-component of its angular velocity.
"""
from __future__ import annotations

import math

import numpy as np

from actdiag.config import SingleJointSceneProfile

try:
    from pyphysx._pyphysx import (
        D6Axis,
        D6Joint,
        D6Motion,
        RigidDynamic,
        RigidStatic,
        Scene,
        SceneFlag,
    )
    _AVAILABLE = True
except ImportError:  # pragma: no cover
    _AVAILABLE = False


def _require_pyphysx() -> None:
    if not _AVAILABLE:
        raise ImportError(
            "The PhysX backend requires pyphysx built from source with arm64 patches.\n"
            "See the project README for installation instructions."
        )


class PhysXBackend:
    """PhysX single-joint backend (pyphysx D6Joint).

    Replicates the same single-revolute-joint scene as the MuJoCo backend so
    that simulation results from both backends can be compared directly.
    """

    name = "physx"

    def __init__(self, scene_profile: SingleJointSceneProfile, dt: float) -> None:
        _require_pyphysx()

        self.dt = dt
        self._scene_profile = scene_profile
        joint = scene_profile.joint

        # --- PhysX scene -------------------------------------------------
        # pyphysx hardcodes gravity to (0, 0, -9.81) in Scene; we disable it
        # per-actor when the joint config does not want gravity.
        self._px_scene = Scene(scene_flags=[SceneFlag.ENABLE_STABILIZATION])

        # --- Static anchor (world attachment point) ----------------------
        anchor = RigidStatic()
        self._px_scene.add_actor(anchor)

        # --- Dynamic link ------------------------------------------------
        # Mass = 1 kg; principal inertia along all axes = joint.inertia so that
        # single-axis rotation matches the MuJoCo model.
        link = RigidDynamic()
        link.set_mass(1.0)
        link.set_mass_space_inertia_tensor(
            np.array([joint.inertia, joint.inertia, joint.inertia], dtype=np.float32)
        )
        link.set_angular_damping(float(joint.damping))
        link.set_linear_damping(100.0)  # suppress translation drift
        if not joint.gravity:
            link.disable_gravity()
        self._px_scene.add_actor(link)
        self._link = link

        # --- D6 joint (revolute about Y) ---------------------------------
        # PhysX D6 TWIST == rotation around joint-local X.
        # To make TWIST align with world Y, rotate joint frames 90° around Z
        # so local X → world Y.  Pose format: [x, y, z, qw, qx, qy, qz].
        c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
        frame = np.array([0.0, 0.0, 0.0, c, 0.0, 0.0, s], dtype=np.float32)

        self._joint = D6Joint(anchor, link, frame, frame)
        self._joint.set_motion(D6Axis.TWIST, D6Motion.FREE)
        self._joint.set_motion(D6Axis.SWING1, D6Motion.LOCKED)
        self._joint.set_motion(D6Axis.SWING2, D6Motion.LOCKED)
        self._joint.set_motion(D6Axis.X, D6Motion.LOCKED)
        self._joint.set_motion(D6Axis.Y, D6Motion.LOCKED)
        self._joint.set_motion(D6Axis.Z, D6Motion.LOCKED)

        # --- Set initial conditions --------------------------------------
        self._set_state(float(joint.q0), float(joint.dq0))

    # ------------------------------------------------------------------
    # PhysicsBackend interface
    # ------------------------------------------------------------------

    def get_state(self) -> tuple[float, float]:
        _pos, q = self._link.get_global_pose()
        # q is a numpy-quaternion with .w .x .y .z; pure Y-rotation → angle = 2*atan2(y, w)
        angle = float(2.0 * math.atan2(float(q.y), float(q.w)))
        omega = self._link.get_angular_velocity()
        dq = float(omega[1])  # Y-component
        return angle, dq

    def apply_torque_and_step(self, torque: float) -> None:
        self._link.add_torque(np.array([0.0, float(torque), 0.0], dtype=np.float32))
        self._px_scene.simulate(self.dt)  # simulate() calls fetchResults internally

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_state(self, q0: float, dq0: float) -> None:
        """Set joint angle and velocity via body transform / angular velocity."""
        # Pure rotation around world Y by q0: (w, x, y, z) = (cos(q0/2), 0, sin(q0/2), 0)
        half = q0 / 2.0
        # Pose as [x, y, z, qw, qx, qy, qz]
        pose = np.array(
            [0.0, 0.0, 0.0, math.cos(half), 0.0, math.sin(half), 0.0],
            dtype=np.float32,
        )
        self._link.set_global_pose(pose)
        self._link.set_angular_velocity(np.array([0.0, dq0, 0.0], dtype=np.float32))
        self._link.set_linear_velocity(np.array([0.0, 0.0, 0.0], dtype=np.float32))
