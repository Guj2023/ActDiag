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
from actdiag.scene import joint_com_x

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

    # Standard gravity magnitude used by both backends [m/s²].
    _G: float = 9.81

    def __init__(self, scene_profile: SingleJointSceneProfile, dt: float) -> None:
        _require_pyphysx()

        self.dt = dt
        self._scene_profile = scene_profile
        joint = scene_profile.joint

        # --- Inertia geometry (must match scene.py / MuJoCo backend) -----
        # The link mass is fixed at 1 kg; com_x is derived from inertia the
        # same way build_single_joint_model() does it so that the gravity
        # torque τ = m·g·com_x·cos(q) is identical in both backends.
        self._mass: float = 1.0
        self._com_x: float = joint_com_x(joint.inertia, self._mass)
        self._gravity_enabled: bool = bool(joint.gravity)

        # --- PhysX scene -------------------------------------------------
        self._px_scene = Scene(scene_flags=[SceneFlag.ENABLE_STABILIZATION])

        # --- Static anchor (world attachment point) ----------------------
        anchor = RigidStatic()
        self._px_scene.add_actor(anchor)

        # --- Dynamic link ------------------------------------------------
        # Inertia consistency note:
        #   MuJoCo stores I_com_yy = inertia − m·com_x² and the engine adds
        #   back m·com_x² when computing equations of motion, so the effective
        #   Y-axis inertia about the pivot = joint.inertia.
        #   PhysX uses set_mass_space_inertia_tensor which is I about the COM.
        #   With COM at the pivot (no offset in the rigid body), the effective
        #   inertia about the pivot is also joint.inertia — the two are equal.
        #
        # Gravity note:
        #   We ALWAYS disable the built-in PhysX gravity on the body.  The
        #   reason: the body's COM is placed at the joint pivot, so gravity
        #   creates only a linear force that the D6 constraint absorbs with
        #   zero rotational effect.  The correct gravity *torque* is computed
        #   analytically and injected in apply_torque_and_step() instead.
        link = RigidDynamic()
        link.set_mass(self._mass)
        link.set_mass_space_inertia_tensor(
            np.array([joint.inertia, joint.inertia, joint.inertia], dtype=np.float32)
        )
        link.set_angular_damping(float(joint.damping))
        link.set_linear_damping(100.0)  # suppress translation drift
        link.disable_gravity()          # gravity torque is applied manually
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
        total = torque
        if self._gravity_enabled:
            # Gravity torque about the Y rotation axis due to COM offset:
            #   τ_y = m · g · com_x · cos(q)
            # Derivation: COM in world frame = (com_x·cos q, 0, −com_x·sin q);
            # gravity force = (0, 0, −m·g); cross-product Y component gives the
            # formula above.  Matches what MuJoCo computes from the geometry.
            q, _ = self.get_state()
            total += self._mass * self._G * self._com_x * math.cos(q)
        self._link.add_torque(np.array([0.0, float(total), 0.0], dtype=np.float32))
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
