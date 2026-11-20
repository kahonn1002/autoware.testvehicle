import math

import pytest

from autoware_carmaker_interface.platform_helpers import (
    ManualTransmission,
    rotate_vector,
    shift_velocity,
)


def test_yaw90_rotates_body_offset():
    half = math.pi / 4.0
    q = (0.0, 0.0, math.sin(half), math.cos(half))
    x, y, z = rotate_vector(q, (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1.0e-9)
    assert math.isclose(y, 1.0, abs_tol=1.0e-9)
    assert math.isclose(z, 0.0, abs_tol=1.0e-9)


def test_roll_and_pitch_rotations():
    half = math.pi / 4.0
    # Roll +90 deg about x: body-y maps to body-z.
    x, y, z = rotate_vector((math.sin(half), 0.0, 0.0, math.cos(half)), (0.0, 1.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1.0e-9)
    assert math.isclose(y, 0.0, abs_tol=1.0e-9)
    assert math.isclose(z, 1.0, abs_tol=1.0e-9)
    # Pitch +90 deg about y: body-x maps to -body-z.
    x, y, z = rotate_vector((0.0, math.sin(half), 0.0, math.cos(half)), (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1.0e-9)
    assert math.isclose(y, 0.0, abs_tol=1.0e-9)
    assert math.isclose(z, -1.0, abs_tol=1.0e-9)


def test_non_unit_quaternion_is_normalized():
    half = math.pi / 4.0
    unit = (0.0, 0.0, math.sin(half), math.cos(half))
    scaled = (0.0, 0.0, 2.0 * math.sin(half), 2.0 * math.cos(half))
    a = rotate_vector(unit, (1.0, 0.0, 0.0))
    b = rotate_vector(scaled, (1.0, 0.0, 0.0))
    for va, vb in zip(a, b):
        assert math.isclose(va, vb, abs_tol=1.0e-9)


def test_zero_quaternion_is_invalid():
    with pytest.raises(ValueError):
        rotate_vector((0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        rotate_vector((float("nan"), 0.0, 0.0, 1.0), (1.0, 0.0, 0.0))


def test_shift_velocity_cross_product():
    vx, vy, vz = shift_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert math.isclose(vx, 1.0, abs_tol=1.0e-9)
    assert math.isclose(vy, 1.0, abs_tol=1.0e-9)
    assert math.isclose(vz, 0.0, abs_tol=1.0e-9)
    # General omega x offset check.
    vx, vy, vz = shift_velocity((0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    assert math.isclose(vx, 2.0 * 6.0 - 3.0 * 5.0, abs_tol=1.0e-9)
    assert math.isclose(vy, 3.0 * 4.0 - 1.0 * 6.0, abs_tol=1.0e-9)
    assert math.isclose(vz, 1.0 * 5.0 - 2.0 * 4.0, abs_tol=1.0e-9)


def test_stationary_brake_stays_disconnected():
    t = ManualTransmission()
    cmd = t.step(0.0, 0.0, 1, 0.5, 0.5)
    assert (cmd.gear, cmd.clutch, cmd.gas) == (1, 1.0, 0.0)
    assert t.started is None


def test_progressive_clutch_release_caps_startup_gas():
    t = ManualTransmission(release_sec=1.5)
    first = t.step(10.0, 0.0, 1, 0.8, 0.0)
    assert first.gear == 1
    assert math.isclose(first.clutch, 1.0, abs_tol=1.0e-9)
    assert math.isclose(first.gas, 0.25, abs_tol=1.0e-9)
    mid = t.step(10.75, 0.0, 1, 0.8, 0.0)
    assert math.isclose(mid.clutch, 0.5, abs_tol=1.0e-9)
    assert math.isclose(mid.gas, 0.25, abs_tol=1.0e-9)
    done = t.step(11.5, 0.0, 1, 0.8, 0.0)
    assert math.isclose(done.clutch, 0.0, abs_tol=1.0e-9)


def test_forward_shift_cuts_throttle_then_resumes():
    t = ManualTransmission(shift_sec=0.6)
    t.step(0.0, 0.0, 1, 0.5, 0.0)
    # Speed 6 targets gear 2 while current gear is 1, opening a shift window.
    opening = t.step(1.0, 6.0, 1, 0.7, 0.0)
    assert opening.gear == 2
    assert math.isclose(opening.clutch, 1.0, abs_tol=1.0e-9)
    assert math.isclose(opening.gas, 0.0, abs_tol=1.0e-9)
    resumed = t.step(1.0 + 0.6, 6.0, 1, 0.7, 0.0)
    assert resumed.gear == 2
    assert math.isclose(resumed.clutch, 0.0, abs_tol=1.0e-9)
    assert math.isclose(resumed.gas, 0.7, abs_tol=1.0e-9)


def test_reverse_while_forward_moving_prohibited():
    t = ManualTransmission()
    t.step(0.0, 6.0, 1, 0.4, 0.0)
    before = t.gear
    cmd = t.step(1.0, 5.0, -1, 0.4, 0.0)
    assert cmd.gear == before
    assert math.isclose(cmd.clutch, 1.0, abs_tol=1.0e-9)
    assert math.isclose(cmd.gas, 0.0, abs_tol=1.0e-9)


def test_neutral_and_safe_stop_fail_closed():
    t = ManualTransmission()
    cmd = t.step(0.0, 6.0, 0, 0.6, 0.0)
    assert (cmd.gear, cmd.clutch, cmd.gas) == (0, 1.0, 0.0)
    cmd = t.step(1.0, 6.0, 1, 0.6, 0.0, safe_stop=True)
    assert (cmd.gear, cmd.clutch, cmd.gas) == (0, 1.0, 0.0)


def test_time_regression_resets_state():
    t = ManualTransmission()
    t.step(10.0, 10.0, 1, 0.5, 0.0)
    assert t.gear == 3
    cmd = t.step(5.0, 0.0, 1, 0.5, 0.0)
    assert cmd.gear == 1
    assert t.shifting is None
    assert t.last_time == 5.0


def test_nan_input_resets_and_fails_closed():
    t = ManualTransmission()
    t.step(10.0, 10.0, 1, 0.5, 0.0)
    cmd = t.step(float("nan"), 10.0, 1, 0.5, 0.0)
    assert (cmd.gear, cmd.clutch, cmd.gas) == (0, 1.0, 0.0)
    assert t.gear == 0
    assert t.last_time is None


def test_upshift_threshold_and_downshift_hysteresis():
    t = ManualTransmission()
    assert t.step(0.0, 5.1, 1, 0.4, 0.0).gear == 2
    # Gear 2 holds while speed stays within 1 m/s below the 5 m/s threshold.
    hold = ManualTransmission()
    hold.gear = 2
    hold.last_time = 0.0
    cmd = hold.step(1.0, 4.5, 1, 0.4, 0.0)
    assert cmd.gear == 2
    # Below the hysteresis band the downshift is allowed.
    drop = ManualTransmission()
    drop.gear = 2
    drop.last_time = 0.0
    cmd = drop.step(1.0, 3.5, 1, 0.4, 0.0)
    assert cmd.gear == 1
