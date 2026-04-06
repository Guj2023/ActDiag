from __future__ import annotations

import math

import mujoco

from actdiag.config import SingleJointSceneProfile


def build_single_joint_model(scene_profile: SingleJointSceneProfile, dt: float) -> mujoco.MjModel:
    joint = scene_profile.joint
    gravity = "0 0 -9.81" if joint.gravity else "0 0 0"

    mass = 1.0
    com_x = min(0.15, max(0.03, math.sqrt(joint.inertia * 0.4 / mass)))
    iyy = max(joint.inertia - (mass * (com_x**2)), 1e-5)
    ixx = max(joint.inertia, 1e-5)
    izz = max(joint.inertia, 1e-5)

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
        <body name="link" pos="0 0 0.2">
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


def initialize_scene_state(model: mujoco.MjModel, scene_profile: SingleJointSceneProfile) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[0] = scene_profile.joint.q0
    data.qvel[0] = scene_profile.joint.dq0
    mujoco.mj_forward(model, data)
    return data

