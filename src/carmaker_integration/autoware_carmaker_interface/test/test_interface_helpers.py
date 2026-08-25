import math

from autoware_carmaker_interface.interface_node import (
    clamp,
    is_time_regression,
    quaternion_multiply,
    sensor_precedes_odometry_epoch,
    stamp_to_nanoseconds,
    transform_xy,
)
from builtin_interfaces.msg import Time


def test_clamp_rejects_non_finite_values():
    assert clamp(float("nan"), -1.0, 1.0) == 0.0
    assert clamp(2.0, -1.0, 1.0) == 1.0


def test_map_transform_rotates_then_translates():
    x, y = transform_xy(1.0, 0.0, math.pi / 2.0, 10.0, 20.0)
    assert math.isclose(x, 10.0, abs_tol=1.0e-9)
    assert math.isclose(y, 21.0, abs_tol=1.0e-9)


def test_yaw_quaternion_composition():
    half = math.pi / 4.0
    q = quaternion_multiply((0.0, 0.0, math.sin(half), math.cos(half)), (0.0, 0.0, 0.0, 1.0))
    assert math.isclose(q[2], math.sin(half))
    assert math.isclose(q[3], math.cos(half))


def test_stamp_to_nanoseconds():
    assert stamp_to_nanoseconds(Time(sec=3, nanosec=250)) == 3_000_000_250


def test_time_regression_detects_new_testrun():
    assert is_time_regression(30_000_000_000, 10_000_000, 0.05)
    assert not is_time_regression(None, 10_000_000, 0.05)
    assert not is_time_regression(1_000_000_000, 975_000_000, 0.05)


def test_sensor_waits_when_it_restarts_before_odometry():
    assert sensor_precedes_odometry_epoch(
        58_000_000_000,
        10_000_000,
        58_000_000_000,
        0.05,
    )
    assert not sensor_precedes_odometry_epoch(
        58_000_000_000,
        10_000_000,
        12_000_000,
        0.05,
    )
    assert not sensor_precedes_odometry_epoch(
        None,
        10_000_000,
        58_000_000_000,
        0.05,
    )
