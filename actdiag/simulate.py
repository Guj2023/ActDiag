from __future__ import annotations

from dataclasses import dataclass

import mujoco
import pandas as pd

from actdiag.actuator import build_actuator
from actdiag.config import RunConfig
from actdiag.controller import ControlState, build_controller
from actdiag.scene import build_single_joint_model, initialize_scene_state
from actdiag.signals import build_signal_series


@dataclass
class SimulationArtifacts:
    timeseries: pd.DataFrame
    video_frames: list | None
    video_fps: int | None


class VideoRecorder:
    def __init__(self, model: mujoco.MjModel, fps: int) -> None:
        if fps <= 0:
            raise ValueError("video fps must be positive")
        try:
            self.renderer = mujoco.Renderer(model, width=960, height=540)
        except Exception as exc:  # pragma: no cover - depends on local GL setup
            raise RuntimeError(
                "unable to create an offscreen renderer for video export"
            ) from exc

        self.fps = fps
        self.frame_interval = 1.0 / fps
        self.next_capture_time = 0.0
        self.frames: list = []

    def maybe_capture(self, data: mujoco.MjData) -> None:
        if data.time + 1e-12 < self.next_capture_time:
            return
        self.renderer.update_scene(data, camera="track")
        self.frames.append(self.renderer.render().copy())
        self.next_capture_time += self.frame_interval

    def close(self) -> None:
        close = getattr(self.renderer, "close", None)
        if callable(close):
            close()


def run_simulation(
    run_config: RunConfig, save_video: bool = False, video_fps: int = 30
) -> SimulationArtifacts:
    signals = build_signal_series(run_config.test)
    model = build_single_joint_model(run_config.scene, run_config.test.dt)
    data = initialize_scene_state(model, run_config.scene)
    actuator = build_actuator(run_config.actuator)
    controller = build_controller(run_config.controller, model)
    recorder = VideoRecorder(model, video_fps) if save_video else None

    records: dict[str, list[float]] = {
        "time": [],
        "q": [],
        "dq": [],
        "q_des": [],
        "dq_des": [],
        "tau_des": [],
        "position_error": [],
        "velocity_error": [],
        "tau_cmd": [],
        "tau_applied": [],
    }

    final_index = len(signals.time) - 1
    for index, time_value in enumerate(signals.time):
        if recorder is not None:
            recorder.maybe_capture(data)

        q = float(data.qpos[0])
        dq = float(data.qvel[0])
        q_des = float(signals.q_des[index])
        dq_des = float(signals.dq_des[index])
        qdd_des = float(signals.qdd_des[index])
        tau_des = float(signals.tau_des[index])

        control_state = ControlState(
            time=float(time_value),
            q=q,
            dq=dq,
            q_des=q_des,
            dq_des=dq_des,
            qdd_des=qdd_des,
            tau_des=tau_des,
        )
        tau_cmd = controller.compute(control_state)
        tau_applied = actuator.apply(tau_cmd)

        records["time"].append(float(time_value))
        records["q"].append(q)
        records["dq"].append(dq)
        records["q_des"].append(q_des)
        records["dq_des"].append(dq_des)
        records["tau_des"].append(tau_des)
        position_error = q_des - q if not pd.isna(q_des) else float("nan")
        velocity_error = dq_des - dq if not pd.isna(dq_des) else float("nan")
        records["position_error"].append(position_error)
        records["velocity_error"].append(velocity_error)
        records["tau_cmd"].append(tau_cmd)
        records["tau_applied"].append(tau_applied)

        if index == final_index:
            continue

        data.qfrc_applied[0] = tau_applied
        mujoco.mj_step(model, data)
        data.qfrc_applied[0] = 0.0

    if recorder is not None:
        frames = recorder.frames
        recorder.close()
    else:
        frames = None

    return SimulationArtifacts(
        timeseries=pd.DataFrame(records),
        video_frames=frames,
        video_fps=video_fps if frames is not None else None,
    )
