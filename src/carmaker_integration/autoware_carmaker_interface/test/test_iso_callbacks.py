"""Callback-level tests: no ROS nodes or simulation are launched."""

import copy
import math
import time

import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from autoware_carmaker_interface.interface_node import (
    CarMakerInterface,
    stamp_to_nanoseconds,
)
from autoware_carmaker_interface.platform_helpers import (
    ManualTransmission,
    shift_acceleration,
)


class FakePub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class FakeTime:
    def __init__(self, sec, nanosec=0):
        self._ns = int(sec * 1_000_000_000 + nanosec)
        self._msg = Time(sec=int(sec), nanosec=int(nanosec))

    def to_msg(self):
        return Time(sec=self._msg.sec, nanosec=self._msg.nanosec)

    @property
    def nanoseconds(self):
        return self._ns


class FakeClock:
    def __init__(self, sec=200):
        self._t = FakeTime(sec)

    def now(self):
        return self._t


class FakeLogger:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class FakeBroadcaster:
    def __init__(self):
        self.msgs = []

    def sendTransform(self, msg):
        self.msgs.append(msg)


def make_node(**over):
    node = CarMakerInterface.__new__(CarMakerInterface)
    base = {
        "map_frame": "map", "base_frame": "base_link",
        "lidar_frame": "carmaker_lidar", "imu_frame": "base_link",
        "map_offset_x": 0.0, "map_offset_y": 0.0, "map_offset_z": 0.0,
        "map_yaw_offset": 0.0, "pose_covariance_xy": 0.0001,
        "pose_covariance_yaw": 0.0001, "command_timeout_sec": 0.5,
        "timeout_brake": 0.35, "steering_command_sign": 1.0,
        "steering_command_ratio": 1.0, "steering_feedback_sign": 1.0,
        "steering_feedback_ratio": 1.0, "max_steering_angle": 0.70,
        "max_steering_rate": 1.5, "selector_drive": 1,
        "selector_reverse": -1, "selector_neutral": 0, "selector_park": -9,
        "stale_sensor_timeout_sec": 0.5, "simulation_reset_tolerance_sec": 0.05,
        "use_manual_transmission": False, "publish_vehicle_control": True,
        "source_base_offset_x": 0.0,
        "source_base_offset_y": 0.0, "source_base_offset_z": 0.0,
        "last_source_odometry_stamp_ns": None,
        "last_source_pointcloud_stamp_ns": None,
        "last_source_imu_stamp_ns": None, "last_imu_omega": None,
        "source_epoch_offset_ns": None, "latest_localization_stamp": None,
        "last_odometry_wall": None, "last_pointcloud_wall": None,
        "latest_speed": 0.0, "odometry_count": 0, "pointcloud_count": 0,
        "pointcloud_waiting_for_localization_count": 0,
        "imu_waiting_for_localization_count": 0,
        "localization_initialized": True, "camera_counts": {},
        "camera_last_stamps": {}, "camera_active_gids": {},
        "camera_retired_gids": set(), "require_camera_epoch": False,
        "camera_expected_epoch": None,
        "camera_active_epochs": {}, "camera_retired_epochs": set(),
        "current_selector": 1,
        "current_gear_report": 3, "latest_steering_tire_angle": 0.0,
        "latest_actuation": (0.0, 0.0, 0.0), "command_hold_active": False,
        "safe_stop_sent": False, "command_active": False,
        "simulation_reset_count": 0, "route_clear_pending": False,
        "clear_route_on_simulation_reset": True,
        "next_route_clear_attempt_wall": 0.0,
        "transmission": ManualTransmission(),
    }
    base.update(over)
    for k, v in base.items():
        setattr(node, k, v)
    node.kinematic_pub = FakePub()
    node.pose_pub = FakePub()
    node.localization_state_pub = FakePub()
    node.initial_pose_pub = FakePub()
    node.pointcloud_pub = FakePub()
    node.velocity_pub = FakePub()
    node.steering_pub = FakePub()
    node.gear_pub = FakePub()
    node.control_mode_pub = FakePub()
    node.turn_indicators_pub = FakePub()
    node.hazard_lights_pub = FakePub()
    node.vehicle_control_pub = FakePub()
    node.tf_broadcaster = FakeBroadcaster()
    node.get_clock = lambda: FakeClock(200)  # noqa: E731
    node.get_logger = lambda: FakeLogger()  # noqa: E731
    return node


def _stamp(sec, ns=0):
    return Time(sec=int(sec), nanosec=int(ns))


def test_duplicate_pointclouds_dropped():
    node = make_node(latest_localization_stamp=_stamp(200),
                     last_source_odometry_stamp_ns=200_000_000_000)
    msg = PointCloud2()
    msg.header.stamp = _stamp(200)
    node._on_pointcloud(msg)
    node._on_pointcloud(copy.deepcopy(msg))
    assert node.pointcloud_count == 1
    assert len(node.pointcloud_pub.msgs) == 1


def test_camera_info_and_image_share_translated_stamp():
    offset = 5_000_000_000
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=100_000_000_000,
        source_epoch_offset_ns=offset,
        camera_counts={"front": 0}, camera_last_stamps={"front": None})
    img, info = Image(), CameraInfo()
    img.header.stamp = info.header.stamp = _stamp(100)
    img_pub, info_pub = FakePub(), FakePub()
    node._on_camera(img, "front", "VehSensor_0", img_pub, True)
    node._on_camera(info, "front", "VehSensor_0", info_pub, False)
    assert len(img_pub.msgs) == len(info_pub.msgs) == 1
    out_img, out_info = img_pub.msgs[0], info_pub.msgs[0]
    assert stamp_to_nanoseconds(out_img.header.stamp) == 100_000_000_000 + offset
    assert stamp_to_nanoseconds(out_info.header.stamp) == \
        stamp_to_nanoseconds(out_img.header.stamp)
    assert out_img.header.frame_id == out_info.header.frame_id == "VehSensor_0"
    # Stable offset: stamp deltas are preserved across frames.
    img2 = Image()
    img2.header.stamp = _stamp(100, 100_000_000)
    node._on_camera(img2, "front", "VehSensor_0", img_pub, True)
    assert stamp_to_nanoseconds(img_pub.msgs[1].header.stamp) - \
        stamp_to_nanoseconds(img_pub.msgs[0].header.stamp) == 100_000_000


def test_prior_epoch_images_rejected_and_duplicates_dropped():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=100_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        camera_counts={"front": 0}, camera_last_stamps={"front": None})
    pub = FakePub()
    old = Image()
    old.header.stamp = _stamp(90)  # 10 s before odometry epoch.
    node._on_camera(old, "front", "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0
    good = Image()
    good.header.stamp = _stamp(100)
    node._on_camera(good, "front", "VehSensor_0", pub, True)
    node._on_camera(copy.deepcopy(good), "front", "VehSensor_0", pub, True)
    assert len(pub.msgs) == 1 and node.camera_counts["front"] == 1


def test_manual_safe_stop_fails_closed():
    node = make_node(use_manual_transmission=True,
                     last_odometry_wall=None,  # stale -> safe stop.
                     current_selector=1)
    node._publish_control(0.6, 0.0, 0.1, 0.0)
    cmd = node.vehicle_control_pub.msgs[-1]
    assert (cmd.gear, cmd.clutch, cmd.gas) == (0, 1.0, 0.0)
    assert cmd.brake >= node.timeout_brake


def test_rear_axle_pose_shift_yaw90_and_twist():
    half = math.pi / 4.0
    node = make_node(source_base_offset_x=1.0,
                     source_base_offset_y=0.0, source_base_offset_z=0.0)
    msg = Odometry()
    msg.header.stamp = _stamp(50)
    msg.pose.pose.position.x, msg.pose.pose.position.y = 10.0, 20.0
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = \
        math.sin(half), math.cos(half)
    msg.twist.twist.linear.x = 1.0
    msg.twist.twist.angular.z = 1.0
    node._on_odometry(msg)
    out = node.kinematic_pub.msgs[0]
    # Body +x offset rotated by yaw90 lands on world +y (rear-axle shift).
    assert out.pose.pose.position.x == pytest.approx(10.0)
    assert out.pose.pose.position.y == pytest.approx(21.0)
    # Twist corrected by omega x offset: (1,0,0)+(0,0,1)x(1,0,0) = (1,1,0).
    assert out.twist.twist.linear.x == pytest.approx(1.0)
    assert out.twist.twist.linear.y == pytest.approx(1.0)


def test_shift_acceleration_centripetal_and_tangential():
    ax, ay, az = shift_acceleration((0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                                    (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert (ax, ay, az) == pytest.approx((-1.0, 0.0, 0.0))
    ax, ay, az = shift_acceleration((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    (0.0, 0.0, 2.0), (1.0, 0.0, 0.0))
    assert (ax, ay, az) == pytest.approx((0.0, 2.0, 0.0))


OLD_EPOCH = "VehSensor_0__epoch_AAA111"
NEW_EPOCH = "VehSensor_0__epoch_BBB222"


def _epoch_image(epoch, sec=10, ns=0):
    msg = Image()
    msg.header.stamp = _stamp(sec, ns)
    msg.header.frame_id = epoch
    return msg


def test_retired_frame_epoch_rejected_while_new_epoch_accepted():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=10_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="BBB222",
        camera_counts={"front": 0}, camera_last_stamps={"front": None},
        camera_active_epochs={}, camera_retired_epochs={OLD_EPOCH})
    pub = FakePub()
    # Retired epoch with a timestamp inside the new epoch is still dropped.
    node._on_camera(_epoch_image(OLD_EPOCH), "front", "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0
    # Same raw stamp from the replacement TestRun epoch is accepted and
    # stripped to the stable optical frame.
    node._on_camera(_epoch_image(NEW_EPOCH), "front", "VehSensor_0", pub, True)
    assert len(pub.msgs) == 1 and node.camera_counts["front"] == 1
    assert pub.msgs[0].header.frame_id == "VehSensor_0"
    assert node.camera_active_epochs["front"] == NEW_EPOCH


def test_missing_or_wrong_sensor_epoch_tag_rejected():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=10_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="BBB222",
        camera_counts={"front": 0}, camera_last_stamps={"front": None},
        camera_active_epochs={}, camera_retired_epochs=set())
    pub = FakePub()
    node._on_camera(_epoch_image("VehSensor_0"), "front", "VehSensor_0", pub, True)
    node._on_camera(_epoch_image("VehSensor_1__epoch_BBB222"),
                     "front", "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0


def test_message_only_camera_callback_strips_epoch_tag():
    import inspect

    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=10_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="BBB222",
        camera_counts={"front": 0}, camera_last_stamps={"front": None},
        camera_active_epochs={}, camera_retired_epochs=set())
    pub = FakePub()
    callback = node._camera_callback("front", "VehSensor_0", pub, True)
    assert len(inspect.signature(callback).parameters) == 1
    callback(_epoch_image(NEW_EPOCH))
    assert len(pub.msgs) == 1
    assert pub.msgs[0].header.frame_id == "VehSensor_0"
    assert node.camera_active_epochs["front"] == NEW_EPOCH


def test_simulation_reset_retires_frame_epochs_and_clears():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=58_000_000_000,
        camera_expected_epoch="AAA111",
        camera_counts={"front": 0}, camera_last_stamps={"front": 1},
        camera_active_epochs={"front": OLD_EPOCH},
        camera_retired_epochs=set())
    node._handle_simulation_reset(58_000_000_000, 10_000_000)
    assert node.camera_active_epochs == {}
    assert node.camera_expected_epoch is None
    assert OLD_EPOCH in node.camera_retired_epochs
    assert all(v is None for v in node.camera_last_stamps.values())
    # Re-arm the new epoch: new camera frame accepted but old epoch blocked.
    node.latest_localization_stamp = _stamp(200)
    node.last_source_odometry_stamp_ns = 10_000_000_000
    node.source_epoch_offset_ns = 1_000_000_000
    node.require_camera_epoch = True
    node.camera_expected_epoch = "BBB222"
    pub = FakePub()
    node._on_camera(_epoch_image(OLD_EPOCH), "front", "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0
    node._on_camera(_epoch_image(NEW_EPOCH), "front", "VehSensor_0", pub, True)
    assert len(pub.msgs) == 1 and node.camera_counts["front"] == 1


def test_direction_change_reverse_while_moving_forward_brakes():
    node = make_node(use_manual_transmission=True,
                     current_selector=-1,  # reverse request.
                     latest_speed=2.0,  # rolling forward.
                     last_odometry_wall=time.monotonic(),  # fresh odometry.
                     command_hold_active=False, safe_stop_sent=False)
    node._publish_control(0.6, 0.0, 0.0, 0.0)
    cmd = node.vehicle_control_pub.msgs[-1]
    assert (cmd.gear, cmd.clutch, cmd.gas) == (0, 1.0, 0.0)
    assert cmd.brake >= node.timeout_brake


def test_data_only_mode_publishes_no_vehicle_command():
    node = make_node(publish_vehicle_control=False, use_manual_transmission=True)
    node._publish_control(1.0, 0.0, 0.5, 0.3)
    node._publish_control(0.0, 0.5, 0.0, 0.0)
    assert node.vehicle_control_pub.msgs == []


def _odom_msg(epoch_suffix_or_none, sec):
    msg = Odometry()
    msg.header.stamp = _stamp(sec)
    if epoch_suffix_or_none is None:
        msg.header.frame_id = "map"
    else:
        msg.header.frame_id = "map__epoch_" + epoch_suffix_or_none
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_unseen_old_epoch_rejected_despite_fitting_timestamp():
    # CCC999 was never received (not retired) but must still be rejected:
    # only the odometry-bound BBB222 suffix is accepted.
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=10_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="BBB222",
        camera_counts={"front": 0, "rear": 0},
        camera_last_stamps={"front": None, "rear": None},
        camera_active_epochs={}, camera_retired_epochs=set())
    pub = FakePub()
    node._on_camera(_epoch_image("VehSensor_0__epoch_CCC999"), "front",
                    "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0
    assert node.camera_active_epochs == {}
    # The bound epoch is accepted and the tag is stripped.
    node._on_camera(_epoch_image(NEW_EPOCH), "front", "VehSensor_0", pub, True)
    assert len(pub.msgs) == 1
    assert pub.msgs[0].header.frame_id == "VehSensor_0"
    # The rear optical frame binds the same TestRun ID with its own prefix.
    rear_pub = FakePub()
    node._on_camera(_epoch_image("VehSensor_1__epoch_BBB222"), "rear",
                    "VehSensor_1", rear_pub, True)
    assert len(rear_pub.msgs) == 1
    assert rear_pub.msgs[0].header.frame_id == "VehSensor_1"


def test_missing_odometry_epoch_id_rejects_cameras():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=10_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="BBB222",
        camera_counts={"front": 0}, camera_last_stamps={"front": None},
        camera_active_epochs={}, camera_retired_epochs=set())
    # Odometry without a TestRun tag clears the expectation.
    node._on_odometry(_odom_msg(None, sec=11))
    assert node.camera_expected_epoch is None
    pub = FakePub()
    node._on_camera(_epoch_image(NEW_EPOCH), "front", "VehSensor_0", pub, True)
    assert pub.msgs == [] and node.camera_counts["front"] == 0
    node._on_camera(_epoch_image("VehSensor_0"), "front", "VehSensor_0", pub, True)
    assert pub.msgs == []


def test_epoch_change_without_time_regression_triggers_reset():
    node = make_node(
        latest_localization_stamp=_stamp(200),
        last_source_odometry_stamp_ns=20_000_000_000,
        source_epoch_offset_ns=1_000_000_000,
        require_camera_epoch=True,
        camera_expected_epoch="AAA111",
        camera_counts={"front": 0}, camera_last_stamps={"front": 20_000_000_000},
        camera_active_epochs={"front": OLD_EPOCH},
        camera_retired_epochs=set())
    resets_before = node.simulation_reset_count
    # Source time increases (20s -> 25s): only the TestRun ID change triggers reset.
    node._on_odometry(_odom_msg("BBB222", sec=25))
    assert node.simulation_reset_count == resets_before + 1
    assert node.camera_expected_epoch == "BBB222"
    assert node.camera_active_epochs == {}
    assert OLD_EPOCH in node.camera_retired_epochs
    # Old epoch rejected, new epoch accepted after the re-bind.
    node.latest_localization_stamp = _stamp(200)
    node.last_source_odometry_stamp_ns = 25_000_000_000
    node.source_epoch_offset_ns = 1_000_000_000
    pub = FakePub()
    node._on_camera(_epoch_image(OLD_EPOCH, sec=25), "front",
                    "VehSensor_0", pub, True)
    assert pub.msgs == []
    node._on_camera(_epoch_image(NEW_EPOCH, sec=25), "front",
                    "VehSensor_0", pub, True)
    assert len(pub.msgs) == 1
    assert pub.msgs[0].header.frame_id == "VehSensor_0"
