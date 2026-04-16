import pytest
from actdiag.actuator import DynamicTorqueActuator, build_actuator
from actdiag.config import DynamicTorqueActuatorProfile, IdealActuatorProfile, LimitedTorqueActuatorProfile
from pydantic import ValidationError

def test_dynamic_torque_config_validation():
    # Valid config
    profile = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=40.0,
        time_constant=0.01
    )
    assert profile.torque_limit == 40.0
    assert profile.time_constant == 0.01

    # Invalid torque_limit <= 0
    with pytest.raises(ValidationError):
        DynamicTorqueActuatorProfile(
            type="dynamic_torque",
            torque_limit=0.0,
            time_constant=0.01
        )

    # Invalid time_constant <= 0
    with pytest.raises(ValidationError):
        DynamicTorqueActuatorProfile(
            type="dynamic_torque",
            torque_limit=40.0,
            time_constant=0.0
        )

def test_dynamic_torque_dynamics():
    dt = 0.001
    time_constant = 0.01
    profile = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=40.0,
        time_constant=time_constant
    )
    actuator = DynamicTorqueActuator(profile, dt)

    # Constant command
    tau_cmd = 10.0
    
    # First step
    output = actuator.apply(tau_cmd)
    # alpha = 0.001 / (0.01 + 0.001) = 1/11
    # tau_next = 0 + (1/11) * (10 - 0) = 10/11 approx 0.909
    assert 0.9 < output.tau_applied < 1.0
    assert not output.is_saturated

    # Many steps, should approach 10.0
    for _ in range(100):
        output = actuator.apply(tau_cmd)
    
    assert abs(output.tau_applied - 10.0) < 1e-3

def test_dynamic_torque_saturation():
    dt = 0.001
    profile = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=10.0,
        time_constant=0.01
    )
    actuator = DynamicTorqueActuator(profile, dt)

    # Command exceeding limit
    tau_cmd = 20.0
    
    for _ in range(1000):
        output = actuator.apply(tau_cmd)
        assert output.is_saturated
        assert output.tau_applied <= 10.0 + 1e-12

    assert abs(output.tau_applied - 10.0) < 1e-3

def test_dynamic_torque_consistency_with_limited():
    dt = 0.001
    # Very small time constant
    profile_dynamic = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=40.0,
        time_constant=1e-9
    )
    actuator_dynamic = DynamicTorqueActuator(profile_dynamic, dt)

    profile_limited = LimitedTorqueActuatorProfile(
        type="limited_torque",
        torque_limit=40.0
    )
    actuator_limited = build_actuator(profile_limited, dt)

    tau_cmd = 15.0
    output_dynamic = actuator_dynamic.apply(tau_cmd)
    output_limited = actuator_limited.apply(tau_cmd)

    # With very small time constant, alpha is nearly 1
    # tau_next = 0 + approx 1 * (15 - 0) = 15
    assert abs(output_dynamic.tau_applied - output_limited.tau_applied) < 1e-3

def test_dynamic_torque_rate_limit():
    dt = 0.01
    rate_limit = 100.0 # 100 Nm/s
    profile = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=40.0,
        time_constant=1e-9, # Negligible lag to test rate limit
        torque_rate_limit=rate_limit
    )
    actuator = DynamicTorqueActuator(profile, dt)

    # Large jump in command
    tau_cmd = 10.0
    # max delta per step = 100 * 0.01 = 1.0
    output = actuator.apply(tau_cmd)
    assert abs(output.tau_applied - 1.0) < 1e-6

    output = actuator.apply(tau_cmd)
    assert abs(output.tau_applied - 2.0) < 1e-6

def test_dynamic_torque_deadzone():
    dt = 0.001
    profile = DynamicTorqueActuatorProfile(
        type="dynamic_torque",
        torque_limit=40.0,
        time_constant=1e-9,
        deadzone=2.0
    )
    actuator = DynamicTorqueActuator(profile, dt)

    # Command within deadzone
    output = actuator.apply(1.5)
    assert abs(output.tau_applied) < 1e-7

    # Command outside deadzone
    output = actuator.apply(3.0)
    assert abs(output.tau_applied - 3.0) < 1e-3
