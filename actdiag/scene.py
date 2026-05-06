from __future__ import annotations

import math

import mujoco

from actdiag.config import SingleJointSceneProfile


def joint_com_x(inertia: float, mass: float = 1.0) -> float:
    """Return the COM offset along body-X [m] that corresponds to *inertia*.

    The single-joint scene models a rigid link whose rotational inertia about
    the pivot is *inertia* [kg·m²].  The COM is placed at this distance from
    the pivot so that the heuristic ``I_pivot ≈ mass * com_x² / 0.4`` holds
    (i.e. a uniform rod of the same inertia has its COM at this location).

    Both the MuJoCo and PhysX backends must use this function to ensure that
    gravity torques are consistent.
    """
    return min(0.15, max(0.03, math.sqrt(inertia * 0.4 / mass)))


def build_single_joint_model(
    scene_profile: SingleJointSceneProfile, dt: float
) -> mujoco.MjModel:
    joint = scene_profile.joint
    gravity = "0 0 -9.81" if joint.gravity else "0 0 0"

    mass = 1.0
    com_x = joint_com_x(joint.inertia, mass)
    iyy = max(joint.inertia - (mass * (com_x**2)), 1e-5)
    ixx = max(joint.inertia, 1e-5)
    izz = max(joint.inertia, 1e-5)

    # The real arm is a pendulum whose pivot is at the top; the link rotates about
    # the Y-axis, so the tip traces the XZ plane. With hinge axis=Y and link along
    # body-X, the tip world-z = body_z - 0.35*sin(q). To avoid floor contact for
    # any q in [0, pi/2] we need body_z > link_length + capsule_radius.
    link_length = 0.35
    capsule_radius = 0.03
    body_z = link_length + capsule_radius + 0.07  # 7 cm safety margin above floor

    xml = f"""
    <mujoco model="actdiag_single_joint">
      <compiler angle="radian" inertiafromgeom="false"/>
      <option timestep="{dt:.12g}" integrator="RK4" gravity="{gravity}"/>
      <visual>
        <global offwidth="960" offheight="540"/>
      </visual>
      <worldbody>
        <light name="key" pos="0 0 1.6" dir="0 0 -1" directional="true"/>
        <geom name="floor" type="plane" size="2 2 0.1" rgba="0.95 0.95 0.95 1"/>
        <body name="link" pos="0 0 {body_z:.4g}">
          <joint
            name="hinge"
            type="hinge"
            axis="0 1 0"
            damping="{joint.damping:.12g}"
            limited="false"
          />
          <inertial
            pos="{com_x:.12g} 0 0"
            mass="{mass:.12g}"
            diaginertia="{ixx:.12g} {iyy:.12g} {izz:.12g}"
          />
          <geom type="sphere" pos="0 0 0" size="0.04" rgba="0.15 0.15 0.15 1"/>
          <geom type="capsule" fromto="0 0 0 0.35 0 0" size="0.03" rgba="0.2 0.5 0.8 1"/>
        </body>
        <camera name="track" pos="1.1 -1.15 0.75" xyaxes="0.7 0.7 0 -0.3 0.3 0.9"/>
      </worldbody>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def initialize_scene_state(
    model: mujoco.MjModel, scene_profile: SingleJointSceneProfile
) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[0] = scene_profile.joint.q0
    data.qvel[0] = scene_profile.joint.dq0
    mujoco.mj_forward(model, data)
    return data
