from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import pandas as pd

from actdiag.actuator import build_actuator
from actdiag.backends import MuJoCoBackend, build_backend
from actdiag.config import FrequencyResponseTestProfile, RunConfig, StepTestProfile
from actdiag.controller import ControlState, build_controller
from actdiag.signals import (
    SignalSeries,
    build_frequency_response_signal,
    build_signal_series,
)


@dataclass
class SimulationArtifacts:
    timeseries: pd.DataFrame
    video_frames: list | None
    video_fps: int | None
    summary_metrics: dict[str, float | None] | None = None


@dataclass
class FrequencyResponseArtifacts:
    summary: pd.DataFrame
    per_frequency_timeseries: dict[float, pd.DataFrame]


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
    if isinstance(run_config.test, FrequencyResponseTestProfile):
        raise TypeError(
            "frequency_response requires run_frequency_response_simulation()"
        )

    signals = build_signal_series(run_config.test, run_config.simulation)
    artifacts = _simulate_signal_series(
        run_config, signals, save_video=save_video, video_fps=video_fps
    )

    if isinstance(run_config.test, StepTestProfile):
        artifacts.summary_metrics = compute_step_response_metrics(
            artifacts.timeseries, run_config.test
        )

    return artifacts


def run_frequency_response_simulation(
    run_config: RunConfig,
) -> FrequencyResponseArtifacts:
    if not isinstance(run_config.test, FrequencyResponseTestProfile):
        raise TypeError("run_config.test must be frequency_response")

    summaries: list[dict[str, float]] = []
    per_frequency_timeseries: dict[float, pd.DataFrame] = {}

    for frequency_hz in run_config.test.frequencies:
        signal = build_frequency_response_signal(
            run_config.test,
            dt=run_config.simulation.dt,
            frequency_hz=float(frequency_hz),
        )
        artifacts = _simulate_signal_series(run_config, signal)
        per_frequency_timeseries[float(frequency_hz)] = artifacts.timeseries
        summaries.append(
            _estimate_frequency_response(
                run_config.test, float(frequency_hz), artifacts.timeseries
            )
        )

    return FrequencyResponseArtifacts(
        summary=pd.DataFrame(summaries).sort_values("frequency_hz").reset_index(
            drop=True
        ),
        per_frequency_timeseries=per_frequency_timeseries,
    )


def _simulate_signal_series(
    run_config: RunConfig,
    signals: SignalSeries,
    *,
    save_video: bool = False,
    video_fps: int = 30,
) -> SimulationArtifacts:
    backend = build_backend(
        run_config.scene, run_config.simulation.dt, run_config.simulation.backend
    )
    actuator = build_actuator(run_config.actuator, run_config.simulation.dt)

    # InverseDynamicsController needs the MuJoCo model for mj_inverse — only
    # available with the MuJoCo backend.
    if isinstance(backend, MuJoCoBackend):
        model = backend.model
    else:
        model = None
    controller = build_controller(
        run_config.controller, model, run_config.simulation.dt
    )

    if save_video and not isinstance(backend, MuJoCoBackend):
        raise ValueError("video export is only supported with the mujoco backend")
    recorder = (
        VideoRecorder(backend.model, video_fps)
        if save_video and isinstance(backend, MuJoCoBackend)
        else None
    )

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
        "integral_error": [],
        "is_saturated": [],
    }

    final_index = len(signals.time) - 1
    for index, time_value in enumerate(signals.time):
        if recorder is not None and isinstance(backend, MuJoCoBackend):
            recorder.maybe_capture(backend.data)

        q, dq = backend.get_state()
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
        controller_output = controller.compute(control_state)
        actuator_output = actuator.apply(controller_output.tau_cmd)

        records["time"].append(float(time_value))
        records["q"].append(q)
        records["dq"].append(dq)
        records["q_des"].append(q_des)
        records["dq_des"].append(dq_des)
        records["tau_des"].append(tau_des)
        records["position_error"].append(
            q_des - q if not pd.isna(q_des) else float("nan")
        )
        records["velocity_error"].append(
            dq_des - dq if not pd.isna(dq_des) else float("nan")
        )
        records["tau_cmd"].append(controller_output.tau_cmd)
        records["tau_applied"].append(actuator_output.tau_applied)
        records["integral_error"].append(controller_output.integral_error)
        records["is_saturated"].append(actuator_output.is_saturated)

        if index == final_index:
            continue

        backend.apply_torque_and_step(actuator_output.tau_applied)

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


def compute_step_response_metrics(
    timeseries: pd.DataFrame, profile: StepTestProfile
) -> dict[str, float | None]:
    time_values = timeseries["time"].to_numpy()
    q_values = timeseries["q"].to_numpy()
    post_step_mask = time_values >= profile.start_time
    post_step_times = time_values[post_step_mask]
    post_step_q = q_values[post_step_mask]

    if post_step_q.size == 0:
        post_step_times = time_values
        post_step_q = q_values

    steady_window_samples = max(10, int(0.1 * post_step_q.size))
    steady_window_q = post_step_q[-steady_window_samples:]
    steady_state_value = float(np.mean(steady_window_q))
    steady_state_error = profile.target - steady_state_value
    peak_value = float(np.max(post_step_q))

    q_des_values = timeseries["q_des"].to_numpy()
    pre_step_des = q_des_values[time_values < profile.start_time]
    initial_target = (
        float(pre_step_des[-1]) if pre_step_des.size else float(q_des_values[0])
    )
    step_amplitude = profile.target - initial_target

    return {
        "steady_state_value": steady_state_value,
        "steady_state_error": steady_state_error,
        "peak_value": peak_value,
        "percent_overshoot": _compute_percent_overshoot(
            post_step_q, profile.target, step_amplitude
        ),
        "rise_time": _compute_rise_time(
            post_step_times, post_step_q, initial_target, profile.target
        ),
        "settling_time": _compute_settling_time(
            post_step_times,
            post_step_q,
            profile.start_time,
            profile.target,
            step_amplitude,
        ),
    }


def _estimate_frequency_response(
    profile: FrequencyResponseTestProfile,
    frequency_hz: float,
    timeseries: pd.DataFrame,
) -> dict[str, float]:
    period = 1.0 / frequency_hz
    steady_start_time = profile.settle_cycles * period
    steady_state = timeseries[timeseries["time"] >= steady_start_time]
    if len(steady_state) < 3:
        raise ValueError(
            f"not enough steady-state samples for frequency {frequency_hz:g} Hz"
        )

    time_values = steady_state["time"].to_numpy()
    q_values = steady_state["q"].to_numpy()
    omega = 2.0 * np.pi * frequency_hz
    # Fit q(t) ~= a*sin(wt) + b*cos(wt) + c over the settled window.
    design_matrix = np.column_stack(
        [
            np.sin(omega * time_values),
            np.cos(omega * time_values),
            np.ones_like(time_values),
        ]
    )
    coefficients, *_ = np.linalg.lstsq(design_matrix, q_values, rcond=None)
    sin_coeff, cos_coeff = float(coefficients[0]), float(coefficients[1])
    output_amplitude = float(np.hypot(sin_coeff, cos_coeff))
    phase_rad = float(np.arctan2(cos_coeff, sin_coeff))

    return {
        "frequency_hz": frequency_hz,
        "input_amplitude": float(profile.amplitude),
        "output_amplitude": output_amplitude,
        "gain": output_amplitude / float(profile.amplitude),
        "phase_rad": phase_rad,
        "phase_deg": float(np.degrees(phase_rad)),
    }


def _compute_percent_overshoot(
    post_step_q: np.ndarray, target: float, step_amplitude: float
) -> float | None:
    if post_step_q.size == 0 or abs(target) < 1e-12 or abs(step_amplitude) < 1e-12:
        return None

    if step_amplitude >= 0.0:
        extreme_value = float(np.max(post_step_q))
        overshoot = (extreme_value - target) / abs(target)
    else:
        extreme_value = float(np.min(post_step_q))
        overshoot = (target - extreme_value) / abs(target)

    return max(0.0, overshoot * 100.0)


def _compute_rise_time(
    time_values: np.ndarray,
    q_values: np.ndarray,
    initial_target: float,
    target: float,
) -> float | None:
    step_amplitude = target - initial_target
    if q_values.size == 0 or abs(step_amplitude) < 1e-12:
        return None

    threshold_10 = initial_target + (0.1 * step_amplitude)
    threshold_90 = initial_target + (0.9 * step_amplitude)
    rising = step_amplitude >= 0.0

    time_10 = _first_crossing_time(time_values, q_values, threshold_10, rising=rising)
    time_90 = _first_crossing_time(time_values, q_values, threshold_90, rising=rising)
    if time_10 is None or time_90 is None or time_90 < time_10:
        return None
    return time_90 - time_10


def _compute_settling_time(
    time_values: np.ndarray,
    q_values: np.ndarray,
    start_time: float,
    target: float,
    step_amplitude: float,
) -> float | None:
    if q_values.size == 0 or abs(step_amplitude) < 1e-12:
        return None

    tolerance = 0.05 * abs(step_amplitude)
    within_band = np.abs(q_values - target) <= tolerance
    stayed_within_band = np.logical_and.accumulate(within_band[::-1])[::-1]
    settled_indices = np.flatnonzero(stayed_within_band)
    if settled_indices.size == 0:
        return None
    return float(time_values[settled_indices[0]] - start_time)


def _first_crossing_time(
    time_values: np.ndarray,
    q_values: np.ndarray,
    threshold: float,
    *,
    rising: bool,
) -> float | None:
    if rising:
        indices = np.flatnonzero(q_values >= threshold)
    else:
        indices = np.flatnonzero(q_values <= threshold)
    if indices.size == 0:
        return None
    return float(time_values[indices[0]])
