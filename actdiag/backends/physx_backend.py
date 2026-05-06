"""PhysX physics backend using pyphysx (PhysX 4 Python bindings).

Installation
------------
pyphysx is not on PyPI as a pre-built wheel — build it from source:

    # Prerequisites: CMake >= 3.14, a C++17 compiler, conan >= 1.x
    pip install conan==1.59
    pip install git+https://github.com/petrikvladimir/pyphysx.git

Or use the NVIDIA PhysX 5 Python wrapper (requires OpenUSD ≥ 25.11):

    pip install ovphysx

If you switch to ovphysx, the Scene / RigidDynamic / RevoluteJoint API calls
below need to be adapted to that library's namespace.

Scene geometry
--------------
The single joint is modelled as a fixed anchor body connected to a freely
rotating dynamic link via a PhysX revolute joint.  The joint axis is Y (same
as the MuJoCo hinge).  Gravity acts along −Z when enabled.

    anchor ──[RevoluteJoint, axis=Y]── link (dynamic, inertia = joint.inertia)

State is read back as the rotation angle of the link around the world Y-axis,
extracted from its quaternion, and the Y-component of its angular velocity.
"""
from __future__ import annotations

import math

import numpy as np

from actdiag.config import SingleJointSceneProfile

try:
    import pyphysx as px
    from pyphysx import RigidDynamic, RigidStatic, Transform
    _AVAILABLE = True
except ImportError:  # pragma: no cover
    _AVAILABLE = False


def _require_pyphysx() -> None:
    if not _AVAILABLE:
        raise ImportError(
            "The PhysX backend requires pyphysx.\n"
            "Build and install it with:\n"
            "    pip install conan==1.59\n"
            "    pip install git+https://github.com/petrikvladimir/pyphysx.git\n"
            "See actdiag/backends/physx_backend.py for alternative binding options."
        )


def _quat_to_y_angle(quat_xyzw: np.ndarray) -> float:
    """Extract rotation angle around world Y-axis from a unit quaternion [x,y,z,w]."""
    x, y, z, w = quat_xyzw
    # Rotation vector component along Y: atan2(2*(w*y + x*z), 1 - 2*(y*y + x*x))
    # Simplified for a pure Y-rotation: angle = 2*atan2(y, w)
    return float(2.0 * math.atan2(y, w))


class PhysXBackend:
    """PhysX single-joint backend (pyphysx).

    Replicates the same single-revolute-joint scene as the MuJoCo backend so
    that simulation results from both backends can be compared directly.
    """

    name = "physx"

    def __init__(self, scene: SingleJointSceneProfile, dt: float) -> None:
        _require_pyphysx()

        self.dt = dt
        self._scene_profile = scene
        joint = scene.joint

        # --- PhysX world -------------------------------------------------
        self._physics = px.Physics.create_physics()

        gravity = np.array([0.0, 0.0, -9.81]) if joint.gravity else np.zeros(3)
        self._px_scene = self._physics.create_scene(
            scene_flags=[px.SceneFlag.eENABLE_GYROSCOPIC_FORCES],
            gravity=gravity,
        )

        material = self._physics.create_material(
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.0,
        )

        # --- Static anchor (world attachment point) ----------------------
        anchor = RigidStatic.create_static()
        self._px_scene.add_actor(anchor)

        # --- Dynamic link ------------------------------------------------
        # pyphysx sets mass and inertia tensor separately.
        # We match the MuJoCo scene: mass=1 kg, principal inertia along Y
        # equals joint.inertia; the other axes are set equal to keep the
        # body well-conditioned (they don't affect single-axis rotation).
        link = RigidDynamic()
        link.set_mass(1.0)
        link.set_mass_space_inertia_tensor(
            np.array([joint.inertia, joint.inertia, joint.inertia])
        )
        link.set_angular_damping(joint.damping)
        link.set_linear_damping(0.0)
        self._px_scene.add_actor(link)
        self._link = link

        # --- Revolute joint (Y-axis) -------------------------------------
        # Frames: anchor frame = identity; link frame = identity.
        # pyphysx's RevoluteJoint locks all DOF except twist (X in PhysX's
        # joint-local convention), so we orient both frames so that the joint
        # X-axis aligns with the world Y-axis.
        #
        # Rotation of 90° around Z maps X → Y:
        rot_z90 = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
        anchor_frame = Transform(q=rot_z90)
        link_frame = Transform(q=rot_z90)

        self._joint = px.RevoluteJoint(
            self._physics,
            anchor, anchor_frame,
            link, link_frame,
        )
        # Disable any built-in drive so we apply torques manually
        self._joint.set_revolute_joint_flag(
            px.RevoluteJointFlag.eDRIVE_ENABLED, False
        )

        # --- Set initial conditions --------------------------------------
        self._set_state(joint.q0, joint.dq0)

    # ------------------------------------------------------------------
    # PhysicsBackend interface
    # ------------------------------------------------------------------

    def get_state(self) -> tuple[float, float]:
        pose = self._link.get_global_pose()
        # pose.q is the quaternion as [x, y, z, w]
        q = _quat_to_y_angle(np.array(pose.q))
        dq = float(np.array(self._link.get_angular_velocity())[1])  # Y-component
        return q, dq

    def apply_torque_and_step(self, torque: float) -> None:
        # Torque around world Y-axis
        self._link.add_torque(np.array([0.0, torque, 0.0]))
        self._px_scene.simulate(self.dt)
        self._px_scene.fetch_results(block=True)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_state(self, q0: float, dq0: float) -> None:
        """Set joint position and velocity via body transform / velocity."""
        # Pure rotation around Y by q0 → quaternion [x, y, z, w]
        half = q0 / 2.0
        quat_xyzw = np.array([0.0, math.sin(half), 0.0, math.cos(half)])
        self._link.set_global_pose(Transform(q=quat_xyzw))
        self._link.set_angular_velocity(np.array([0.0, dq0, 0.0]))
        self._link.set_linear_velocity(np.zeros(3))
