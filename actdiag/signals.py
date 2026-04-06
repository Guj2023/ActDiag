from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from actdiag.config import SineTestProfile, StepTestProfile, TestProfile


@dataclass(frozen=True)
class SignalSeries:
    time: np.ndarray
    q_des: np.ndarray
    dq_des: np.ndarray
    qdd_des: np.ndarray


def build_signal_series(test_profile: TestProfile) -> SignalSeries:
    if isinstance(test_profile, StepTestProfile):
        return _build_step_signal(test_profile)
    if isinstance(test_profile, SineTestProfile):
        return _build_sine_signal(test_profile)
    raise TypeError(f"unsupported test profile: {type(test_profile)!r}")


def _build_step_signal(profile: StepTestProfile) -> SignalSeries:
    time = np.arange(0.0, profile.duration + (profile.dt * 0.5), profile.dt)
    q_des = np.where(time >= profile.start_time, profile.target, 0.0)
    dq_des = np.zeros_like(time)
    qdd_des = np.zeros_like(time)
    return SignalSeries(time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des)


def _build_sine_signal(profile: SineTestProfile) -> SignalSeries:
    time = np.arange(0.0, profile.duration + (profile.dt * 0.5), profile.dt)
    omega = 2.0 * np.pi * profile.frequency
    q_des = profile.offset + (profile.amplitude * np.sin(omega * time))
    dq_des = profile.amplitude * omega * np.cos(omega * time)
    qdd_des = -(profile.amplitude * (omega**2) * np.sin(omega * time))
    return SignalSeries(time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des)

