"""pytest port of the key C++ PanTiltSim / ServoActuator cases, plus the
binding-specific contract: config is writable, state is read-only, axes are
live references that cannot be swapped out."""

import math

import pytest

import tinydynamics as td

DT = 0.01
# float32 arithmetic on the C++ side; compare loosely enough to absorb it.
REL = 1e-5


def close(a, b, rel=REL, abs_=1e-6):
    return math.isclose(a, b, rel_tol=rel, abs_tol=abs_)


# --- basic composition -------------------------------------------------------

def test_defaults_start_at_zero():
    sim = td.PanTiltSim()
    assert sim.pan_angle() == 0.0
    assert sim.tilt_angle() == 0.0
    assert sim.pan_velocity() == 0.0
    assert sim.tilt_velocity() == 0.0


def test_pan_moves_independently():
    sim = td.PanTiltSim()
    sim.set_pan_target(1.0)
    sim.update(DT)
    assert sim.pan_angle() > 0.0
    assert sim.tilt_angle() == 0.0
    assert sim.tilt_velocity() == 0.0


def test_tilt_moves_independently():
    sim = td.PanTiltSim()
    sim.set_tilt_target(1.0)
    sim.update(DT)
    assert sim.tilt_angle() > 0.0
    assert sim.pan_angle() == 0.0
    assert sim.pan_velocity() == 0.0


def test_both_axes_move():
    sim = td.PanTiltSim()
    sim.set_pan_target(1.0)
    sim.set_tilt_target(0.5)
    sim.update(DT)
    assert sim.pan_angle() > 0.0
    assert sim.tilt_angle() > 0.0


# --- clamping (delegated to ServoActuator) -----------------------------------

@pytest.mark.parametrize("axis", ["pan", "tilt"])
def test_target_clamps_to_travel(axis):
    sim = td.PanTiltSim()
    setter = getattr(sim, f"set_{axis}_target")
    servo = getattr(sim, axis)

    setter(-1.0)
    assert close(servo.target_angle, servo.min_angle)

    setter(10.0)
    assert close(servo.target_angle, servo.max_angle)

    setter(1.0)
    assert close(servo.target_angle, 1.0)


# --- rate limiting matches the C++ test exactly ------------------------------

def test_large_step_is_rate_limited():
    sim = td.PanTiltSim()
    sim.set_pan_target(math.pi)
    sim.set_tilt_target(math.pi)
    sim.update(DT)

    expected = sim.pan.max_angular_velocity * DT
    assert close(sim.pan_angle(), expected)
    assert close(sim.tilt_angle(), expected)
    assert close(sim.pan_velocity(), sim.pan.max_angular_velocity)


# --- calibration from Python: config is writable through the live reference --

def test_tuning_through_axis_reference_takes_effect():
    sim = td.PanTiltSim()
    sim.pan.max_angular_velocity = 1.0  # much slower than the MG90S default
    sim.set_pan_target(math.pi)
    sim.update(DT)

    # Step is now limited by the new speed, proving `sim.pan` is not a copy.
    assert close(sim.pan_angle(), 1.0 * DT)
    assert close(sim.pan.max_angular_velocity, 1.0)


def test_each_config_field_is_writable():
    servo = td.ServoActuator()
    servo.min_angle = -1.0
    servo.max_angle = 1.0
    servo.max_angular_velocity = 2.0
    servo.angular_deadband = 0.05
    assert close(servo.min_angle, -1.0)
    assert close(servo.max_angle, 1.0)
    assert close(servo.max_angular_velocity, 2.0)
    assert close(servo.angular_deadband, 0.05)

    # New limits are honored by clamping.
    servo.set_target_angle(5.0)
    assert close(servo.target_angle, 1.0)


# --- state is read-only: Python cannot bypass the model ----------------------

@pytest.mark.parametrize("field", ["target_angle", "current_angle", "angular_velocity"])
def test_state_fields_are_read_only(field):
    servo = td.ServoActuator()
    with pytest.raises(AttributeError):
        setattr(servo, field, 123.0)


@pytest.mark.parametrize("axis", ["pan", "tilt"])
def test_axes_cannot_be_replaced(axis):
    sim = td.PanTiltSim()
    with pytest.raises(AttributeError):
        setattr(sim, axis, td.ServoActuator())


# --- getters are views onto the servo state ----------------------------------

def test_getters_match_underlying_servos():
    sim = td.PanTiltSim()
    sim.set_pan_target(math.pi)
    sim.set_tilt_target(0.3)
    sim.update(DT)

    assert sim.pan_angle() == sim.pan.current_angle
    assert sim.tilt_angle() == sim.tilt.current_angle
    assert sim.pan_velocity() == sim.pan.angular_velocity
    assert sim.tilt_velocity() == sim.tilt.angular_velocity
    assert sim.pan_velocity() > 0.0
    assert sim.tilt_velocity() > 0.0


def test_servo_settles_and_stops():
    servo = td.ServoActuator()
    servo.set_target_angle(2.0)
    for _ in range(1000):
        servo.update(0.001)
    assert close(servo.current_angle, 2.0, abs_=servo.angular_deadband)
    assert servo.angular_velocity == 0.0
