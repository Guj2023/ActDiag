"""Tests for actdiag.signals — signal generation for all test types."""
from __future__ import annotations

import math

import numpy as np
import pytest

from actdiag.config import (
    ChirpTestProfile,
    FrequencyResponseTestProfile,
    SimulationConfig,
    SineTestProfile,
    StepTestProfile,
    TorqueSineTestProfile,
    TorqueStepTestProfile,
)
from actdiag.signals import build_frequency_response_signal, build_signal_series


def _sim(duration=2.0, dt=0.01):
    return SimulationConfig(duration=duration, dt=dt)


# ---------------------------------------------------------------------------
# Step signal
# ---------------------------------------------------------------------------

class TestStepSignal:
    def test_shape(self):
        sig = build_signal_series(StepTestProfile(test_type="step", target=1.0), _sim())
        n = len(sig.time)
        assert sig.q_des.shape == (n,)
        assert sig.tau_des.shape == (n,)

    def test_time_starts_at_zero(self):
        sig = build_signal_series(StepTestProfile(test_type="step", target=1.0), _sim())
        assert sig.time[0] == pytest.approx(0.0)

    def test_time_length(self):
        sig = build_signal_series(StepTestProfile(test_type="step", target=1.0), _sim(duration=1.0, dt=0.1))
        assert len(sig.time) == 11  # 0.0, 0.1, ..., 1.0

    def test_step_before_start_time(self):
        sig = build_signal_series(
            StepTestProfile(test_type="step", target=1.5, start_time=0.5), _sim()
        )
        assert sig.q_des[0] == pytest.approx(0.0)

    def test_step_at_start_time(self):
        sig = build_signal_series(
            StepTestProfile(test_type="step", target=1.5, start_time=0.5), _sim()
        )
        post = sig.q_des[sig.time >= 0.5]
        assert np.all(post == pytest.approx(1.5))

    def test_tau_des_is_nan(self):
        sig = build_signal_series(StepTestProfile(test_type="step", target=1.0), _sim())
        assert np.all(np.isnan(sig.tau_des))

    def test_dq_des_is_zero(self):
        sig = build_signal_series(StepTestProfile(test_type="step", target=1.0), _sim())
        assert np.all(sig.dq_des == 0.0)


# ---------------------------------------------------------------------------
# Sine signal
# ---------------------------------------------------------------------------

class TestSineSignal:
    def test_amplitude_at_quarter_period(self):
        freq = 1.0
        sig = build_signal_series(
            SineTestProfile(test_type="sine", amplitude=0.5, frequency=freq),
            _sim(duration=1.0, dt=0.001),
        )
        # sin peaks at t = 0.25 s for 1 Hz
        idx = np.argmin(np.abs(sig.time - 0.25))
        assert sig.q_des[idx] == pytest.approx(0.5, abs=0.005)

    def test_velocity_is_derivative(self):
        sig = build_signal_series(
            SineTestProfile(test_type="sine", amplitude=0.3, frequency=2.0),
            _sim(duration=1.0, dt=0.0001),
        )
        # dq_des should match numerical derivative of q_des
        num_deriv = np.gradient(sig.q_des, sig.time)
        # compare in the middle (avoid endpoints)
        mid = slice(100, -100)
        np.testing.assert_allclose(sig.dq_des[mid], num_deriv[mid], atol=0.01)

    def test_offset_applied(self):
        sig = build_signal_series(
            SineTestProfile(test_type="sine", amplitude=0.2, frequency=1.0, offset=1.0),
            _sim(duration=0.5, dt=0.01),
        )
        assert sig.q_des[0] == pytest.approx(1.0, abs=0.01)  # sin(0) = 0, so offset only

    def test_tau_des_is_nan(self):
        sig = build_signal_series(
            SineTestProfile(test_type="sine", amplitude=0.5, frequency=1.0), _sim()
        )
        assert np.all(np.isnan(sig.tau_des))


# ---------------------------------------------------------------------------
# Chirp signal
# ---------------------------------------------------------------------------

class TestChirpSignal:
    def test_basic_shape(self):
        profile = ChirpTestProfile(
            test_type="chirp", amplitude=0.3, f0=0.5, f1=2.0, duration=4.0
        )
        sig = build_signal_series(profile, _sim(duration=4.0, dt=0.01))
        assert sig.q_des.shape == sig.time.shape

    def test_starts_at_zero_phase(self):
        profile = ChirpTestProfile(
            test_type="chirp", amplitude=0.5, f0=1.0, f1=5.0, duration=4.0
        )
        sig = build_signal_series(profile, _sim(duration=4.0, dt=0.01))
        assert sig.q_des[0] == pytest.approx(0.0, abs=0.001)

    def test_amplitude_bounded(self):
        profile = ChirpTestProfile(
            test_type="chirp", amplitude=0.4, f0=0.5, f1=3.0, duration=6.0
        )
        sig = build_signal_series(profile, _sim(duration=6.0, dt=0.01))
        assert np.max(np.abs(sig.q_des)) <= 0.41

    def test_log_sweep(self):
        profile = ChirpTestProfile(
            test_type="chirp", amplitude=0.3, f0=1.0, f1=10.0, duration=5.0, sweep="log"
        )
        sig = build_signal_series(profile, _sim(duration=5.0, dt=0.01))
        assert sig.q_des.shape == sig.time.shape


# ---------------------------------------------------------------------------
# Torque step signal
# ---------------------------------------------------------------------------

class TestTorqueStepSignal:
    def test_q_des_is_nan(self):
        sig = build_signal_series(
            TorqueStepTestProfile(test_type="torque_step", target_torque=2.0),
            _sim(),
        )
        assert np.all(np.isnan(sig.q_des))
        assert np.all(np.isnan(sig.dq_des))
        assert np.all(np.isnan(sig.qdd_des))

    def test_torque_value_after_start(self):
        sig = build_signal_series(
            TorqueStepTestProfile(test_type="torque_step", target_torque=1.5, start_time=0.5),
            _sim(),
        )
        post = sig.tau_des[sig.time >= 0.5]
        assert np.all(post == pytest.approx(1.5))

    def test_torque_zero_before_start(self):
        sig = build_signal_series(
            TorqueStepTestProfile(test_type="torque_step", target_torque=1.5, start_time=0.5),
            _sim(),
        )
        pre = sig.tau_des[sig.time < 0.5]
        assert np.all(pre == pytest.approx(0.0))


# ---------------------------------------------------------------------------
# Torque sine signal
# ---------------------------------------------------------------------------

class TestTorqueSineSignal:
    def test_q_des_is_nan(self):
        sig = build_signal_series(
            TorqueSineTestProfile(test_type="torque_sine", amplitude=1.0, frequency=2.0),
            _sim(),
        )
        assert np.all(np.isnan(sig.q_des))

    def test_tau_amplitude(self):
        sig = build_signal_series(
            TorqueSineTestProfile(test_type="torque_sine", amplitude=2.0, frequency=1.0),
            _sim(duration=1.0, dt=0.001),
        )
        assert np.max(np.abs(sig.tau_des)) == pytest.approx(2.0, abs=0.005)


# ---------------------------------------------------------------------------
# Frequency-response signal
# ---------------------------------------------------------------------------

class TestFrequencyResponseSignal:
    def test_duration_matches_cycles(self):
        profile = FrequencyResponseTestProfile(
            test_type="frequency_response",
            amplitude=0.3,
            frequencies=[2.0],
            cycles_per_frequency=8,
            settle_cycles=3,
        )
        sig = build_frequency_response_signal(profile, dt=0.001, frequency_hz=2.0)
        expected_duration = 8 / 2.0  # 4 s
        assert sig.time[-1] == pytest.approx(expected_duration, abs=0.002)

    def test_amplitude_correct(self):
        profile = FrequencyResponseTestProfile(
            test_type="frequency_response",
            amplitude=0.5,
            frequencies=[1.0],
            cycles_per_frequency=8,
            settle_cycles=3,
        )
        sig = build_frequency_response_signal(profile, dt=0.001, frequency_hz=1.0)
        assert np.max(np.abs(sig.q_des)) == pytest.approx(0.5, abs=0.005)

    def test_frequency_response_signal_raises_if_called_via_build_signal_series(self):
        profile = FrequencyResponseTestProfile(
            test_type="frequency_response",
            amplitude=0.3,
            frequencies=[1.0],
            cycles_per_frequency=8,
            settle_cycles=3,
        )
        with pytest.raises(TypeError):
            build_signal_series(profile, _sim())
