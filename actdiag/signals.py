from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from actdiag.config import (
    ChirpTestProfile,
    FrequencyResponseTestProfile,
    SimulationConfig,
    SineTestProfile,
    StepTestProfile,
    TestProfile,
    TorqueSineTestProfile,
    TorqueStepTestProfile,
)


@dataclass(frozen=True)
class SignalSeries:
    time: np.ndarray
    q_des: np.ndarray
    dq_des: np.ndarray
    qdd_des: np.ndarray
    tau_des: np.ndarray


def build_signal_series(
    test_profile: TestProfile, simulation: SimulationConfig
) -> SignalSeries:
    if isinstance(test_profile, StepTestProfile):
        return _build_step_signal(test_profile, simulation)
    if isinstance(test_profile, SineTestProfile):
        return _build_sine_signal(test_profile, simulation)
    if isinstance(test_profile, ChirpTestProfile):
        return _build_chirp_signal(test_profile, simulation)
    if isinstance(test_profile, TorqueStepTestProfile):
        return _build_torque_step_signal(test_profile, simulation)
    if isinstance(test_profile, TorqueSineTestProfile):
        return _build_torque_sine_signal(test_profile, simulation)
    if isinstance(test_profile, FrequencyResponseTestProfile):
        raise TypeError("frequency_response uses build_frequency_response_signal()")
    raise TypeError(f"unsupported test profile: {type(test_profile)!r}")


def build_frequency_response_signal(
    profile: FrequencyResponseTestProfile, *, dt: float, frequency_hz: float
) -> SignalSeries:
    duration = profile.cycles_per_frequency / frequency_hz
    time = _build_time_vector(duration, dt)
    omega = 2.0 * np.pi * frequency_hz
    q_des = profile.offset + (profile.amplitude * np.sin(omega * time))
    dq_des = profile.amplitude * omega * np.cos(omega * time)
    qdd_des = -(profile.amplitude * (omega**2) * np.sin(omega * time))
    tau_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_step_signal(
    profile: StepTestProfile, simulation: SimulationConfig
) -> SignalSeries:
    time = _build_time_vector(_require_duration(simulation), simulation.dt)
    q_des = np.where(time >= profile.start_time, profile.target, 0.0)
    dq_des = np.zeros_like(time)
    qdd_des = np.zeros_like(time)
    tau_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_sine_signal(
    profile: SineTestProfile, simulation: SimulationConfig
) -> SignalSeries:
    time = _build_time_vector(_require_duration(simulation), simulation.dt)
    omega = 2.0 * np.pi * profile.frequency
    q_des = profile.offset + (profile.amplitude * np.sin(omega * time))
    dq_des = profile.amplitude * omega * np.cos(omega * time)
    qdd_des = -(profile.amplitude * (omega**2) * np.sin(omega * time))
    tau_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_chirp_signal(
    profile: ChirpTestProfile, simulation: SimulationConfig
) -> SignalSeries:
    duration = _require_duration(simulation)
    time = _build_time_vector(duration, simulation.dt)
    f0, f1 = profile.f0, profile.f1

    if profile.sweep == "linear":
        # f(t) = f0 + (f1 - f0) * (t / duration)
        # phi(t) = 2pi * integral f(t) dt = 2pi * (f0*t + (f1 - f0)/(2*duration) * t^2)
        k = (f1 - f0) / duration
        phi = 2.0 * np.pi * (f0 * time + 0.5 * k * time**2)
        f_t = f0 + k * time
        # f_dot = (f1 - f0) / duration
        f_dot = k
    else:  # logarithmic
        if abs(f1 - f0) < 1e-12:
            # Special case: constant frequency
            omega = 2.0 * np.pi * f0
            phi = omega * time
            f_t = np.full_like(time, f0)
            f_dot = np.zeros_like(time)
        else:
            # f(t) = f0 * (f1 / f0)^(t / duration)
            # phi(t) = 2pi * integral f(t) dt
            # integral a^x dx = a^x / ln(a)
            # phi(t) = 2pi * f0 * integral (f1/f0)^(t/duration) dt
            # let u = t/duration, dt = duration * du
            # phi(t) = 2pi * f0 * duration * integral (f1/f0)^u du
            # phi(t) = 2pi * f0 * duration * [(f1/f0)^u / ln(f1/f0)]
            # phi(t) = 2pi * f0 * duration * [((f1/f0)^(t/duration) - 1) / ln(f1/f0)]
            beta = np.log(f1 / f0)
            phi = 2.0 * np.pi * f0 * (duration / beta) * (np.exp(beta * time / duration) - 1.0)
            f_t = f0 * np.exp(beta * time / duration)
            # f_dot = f0 * (f1/f0)^(t/duration) * ln(f1/f0) / duration
            f_dot = f_t * beta / duration

    omega_t = 2.0 * np.pi * f_t
    omega_dot = 2.0 * np.pi * f_dot

    q_des = profile.offset + profile.amplitude * np.sin(phi)
    # dq/dt = amplitude * cos(phi) * phi_dot
    dq_des = profile.amplitude * np.cos(phi) * omega_t
    # d2q/dt2 = amplitude * (-sin(phi) * phi_dot^2 + cos(phi) * phi_double_dot)
    qdd_des = profile.amplitude * (
        -np.sin(phi) * (omega_t**2) + np.cos(phi) * omega_dot
    )
    
    tau_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_torque_step_signal(
    profile: TorqueStepTestProfile, simulation: SimulationConfig
) -> SignalSeries:
    time = _build_time_vector(_require_duration(simulation), simulation.dt)
    tau_des = np.where(time >= profile.start_time, profile.target_torque, 0.0)
    q_des = np.full_like(time, np.nan)
    dq_des = np.full_like(time, np.nan)
    qdd_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_torque_sine_signal(
    profile: TorqueSineTestProfile, simulation: SimulationConfig
) -> SignalSeries:
    time = _build_time_vector(_require_duration(simulation), simulation.dt)
    omega = 2.0 * np.pi * profile.frequency
    tau_des = profile.offset + (profile.amplitude * np.sin(omega * time))
    q_des = np.full_like(time, np.nan)
    dq_des = np.full_like(time, np.nan)
    qdd_des = np.full_like(time, np.nan)
    return SignalSeries(
        time=time, q_des=q_des, dq_des=dq_des, qdd_des=qdd_des, tau_des=tau_des
    )


def _build_time_vector(duration: float, dt: float) -> np.ndarray:
    return np.arange(0.0, duration + (dt * 0.5), dt)


def _require_duration(simulation: SimulationConfig) -> float:
    if simulation.duration is None:
        raise ValueError("simulation.duration is required for this test type")
    return simulation.duration
