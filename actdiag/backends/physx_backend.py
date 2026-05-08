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
        link.set_mass(_mass)
        link.set_mass(self._mass)
        link.set_mass_space_inertia_tensor(
            np.array([joint.inertia, joint.inertia, joint.inertia], dtype=np.float32)
        )
        # Angular damping: PhysX applies damping as ω *= (1 - d*dt), which
        # makes the effective torque d*I*ω — NOT d*ω as MuJoCo's joint damping.
        # With d=0.1 and I=0.05, the effective damping torque is only 0.005*ω
        # vs MuJoCo's 0.1*ω — 20× less.  We therefore disable PhysX angular
        # damping entirely and inject τ_damp = -b*dq analytically each step,
        # identical to how MuJoCo's joint.damping is handled.
        link.set_angular_damping(0.0)
        link.set_linear_damping(100.0)  # suppress translation drift only

        # PhysX places the rigid-body COM at the link origin (the pivot).
        # Gravity applied at the pivot produces zero torque about the hinge,
        # making the body act as if there were no gravity load.  We instead
        # always disable PhysX body gravity and inject the gravity torque
        # analytically — exactly as MuJoCo computes it from the COM offset.
        # Formula: τ_grav = m · g · com_x · cos(q)
        # com_x mirrors the offset used in scene.py / build_single_joint_model().
        link.disable_gravity()
        self._px_scene.add_actor(link)
        self._link = link

        # Store per-step injection parameters.
        self._gravity_enabled = bool(joint.gravity)
        self._damping = float(joint.damping)
        self._mass = _mass
        self._G = 9.81
        self._com_x = min(0.15, max(0.03, math.sqrt(joint.inertia * 0.4 / _mass)))

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
        q, dq = self.get_state()
        total = float(torque)
        # Inject joint damping analytically: τ = -b·dq  (matches MuJoCo joint.damping)
        total -= self._damping * dq
        # Inject gravity torque analytically: τ = m·g·com_x·cos(q)
        if self._gravity_enabled:
            total += self._mass * self._G * self._com_x * math.cos(q)
        self._link.add_torque(np.array([0.0, total, 0.0], dtype=np.float32))
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
