import copy
import math
import time
from typing import Iterable, Tuple

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.srv import ClearRoute
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped
from vehiclecontrol_msgs.msg import VehicleControl
from .platform_helpers import ManualTransmission, rotate_vector, shift_acceleration, shift_velocity

NANOSECONDS_PER_SECOND = 1_000_000_000


def clamp(value: float, lower: float, upper: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(lower, min(upper, value))


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def is_time_regression(
    previous_nanoseconds: int | None,
    current_nanoseconds: int,
    tolerance_sec: float,
) -> bool:
    if previous_nanoseconds is None:
        return False
    tolerance_nanoseconds = max(
        0, int(tolerance_sec * NANOSECONDS_PER_SECOND)
    )
    return current_nanoseconds + tolerance_nanoseconds < previous_nanoseconds


def sensor_precedes_odometry_epoch(
    previous_sensor_nanoseconds: int | None,
    current_sensor_nanoseconds: int,
    current_odometry_nanoseconds: int | None,
    tolerance_sec: float,
) -> bool:
    """Return true when a sensor has restarted before odometry has restarted."""
    return is_time_regression(
        previous_sensor_nanoseconds,
        current_sensor_nanoseconds,
        tolerance_sec,
    ) and is_time_regression(
        current_odometry_nanoseconds,
        current_sensor_nanoseconds,
        tolerance_sec,
    )


def quaternion_multiply(
    left: Iterable[float], right: Iterable[float]
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def transform_xy(
    x: float, y: float, yaw: float, offset_x: float, offset_y: float
) -> Tuple[float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * x - sine * y + offset_x,
        sine * x + cosine * y + offset_y,
    )


class CarMakerInterface(Node):
    def __init__(self) -> None:
        super().__init__("autoware_carmaker_interface")
        self._declare_parameters()
        self._load_parameters()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(depth=10)
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.kinematic_pub = self.create_publisher(
            Odometry, self.output_kinematic_state_topic, reliable_qos
        )
        self.localization_state_pub = self.create_publisher(
            LocalizationInitializationState,
            self.output_localization_state_topic,
            latched_qos,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            latched_qos,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_pose_topic, reliable_qos
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, self.output_pointcloud_topic, reliable_qos
        )
        # AEB's polling subscriber requests RELIABLE. A reliable publisher also
        # serves best-effort sensor consumers; keep the raw input best-effort.
        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, reliable_qos)
        self.acceleration_pub = self.create_publisher(
            AccelWithCovarianceStamped,
            self.output_acceleration_topic,
            reliable_qos,
        )
        self.velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", reliable_qos
        )
        self.steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", reliable_qos
        )
        self.gear_pub = self.create_publisher(
            GearReport, "/vehicle/status/gear_status", reliable_qos
        )
        self.control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", reliable_qos
        )
        self.turn_indicators_pub = self.create_publisher(
            TurnIndicatorsReport,
            "/vehicle/status/turn_indicators_status",
            reliable_qos,
        )
        self.hazard_lights_pub = self.create_publisher(
            HazardLightsReport,
            "/vehicle/status/hazard_lights_status",
            reliable_qos,
        )
        self.actuation_status_pub = self.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", reliable_qos
        )
        self.vehicle_control_pub = self.create_publisher(
            VehicleControl, self.vehicle_control_topic, reliable_qos
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", reliable_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.clear_route_client = self.create_client(
            ClearRoute, self.clear_route_service
        )

        self.create_subscription(
            Odometry, self.source_odometry_topic, self._on_odometry, reliable_qos
        )
        self.create_subscription(
            PointCloud2, self.source_pointcloud_topic, self._on_pointcloud, sensor_qos
        )
        self.create_subscription(Imu, self.source_imu_topic, self._on_imu, sensor_qos)
        self.create_subscription(
            Float64, self.source_steering_topic, self._on_steering, reliable_qos
        )
        self.create_subscription(
            ActuationCommandStamped,
            self.actuation_command_topic,
            self._on_actuation_command,
            reliable_qos,
        )
        self.create_subscription(
            GearCommand,
            self.gear_command_topic,
            self._on_gear_command,
            reliable_qos,
        )

        self.last_odometry_wall = None
        self.last_pointcloud_wall = None
        self.last_command_wall = None
        self.latest_speed = 0.0
        self.transmission = ManualTransmission()
        self.camera_counts = {}
        self.camera_last_stamps = {}
        self.camera_active_epochs = {}
        self.camera_retired_epochs = set()
        self.camera_expected_epoch = None
        if self.enable_cameras:
            for name, source, frame in (
                ("front", "VehSensor_0", "VehSensor_0"),
                ("rear", "VehSensor_1", "VehSensor_1"),
            ):
                self.camera_counts[name] = 0
                self.camera_last_stamps[name] = None
                image_pub = self.create_publisher(Image, f"/sensing/camera/{name}/image_raw", sensor_qos)
                info_pub = self.create_publisher(CameraInfo, f"/sensing/camera/{name}/camera_info", sensor_qos)
                self.create_subscription(Image, f"/{source}/image_raw",
                    self._camera_callback(name, frame, image_pub, True), sensor_qos)
                self.create_subscription(CameraInfo, f"/{source}/camera_info",
                    self._camera_callback(name, frame, info_pub, False), sensor_qos)
        self.command_active = False
        self.safe_stop_sent = False
        self.latest_steering_tire_angle = 0.0
        self.latest_actuation = (0.0, 0.0, 0.0)
        self.current_selector = self.selector_drive
        self.current_gear_report = GearReport.DRIVE
        self.odometry_count = 0
        self.pointcloud_count = 0
        self.command_count = 0
        self.localization_initialized = False
        self.latest_localization_stamp = None
        self.last_source_odometry_stamp_ns = None
        self.last_source_pointcloud_stamp_ns = None
        self.last_source_imu_stamp_ns = None
        self.last_imu_omega = None
        self.source_epoch_offset_ns = None
        self.simulation_reset_count = 0
        self.pointcloud_waiting_for_localization_count = 0
        self.imu_waiting_for_localization_count = 0
        self.route_clear_pending = False
        self.route_clear_future = None
        self.command_hold_active = False
        self.next_route_clear_attempt_wall = 0.0

        system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(0.1, self._watchdog, clock=system_clock)
        self.create_timer(1.0, self._publish_diagnostics, clock=system_clock)
        self.get_logger().info(
            "CarMaker interface ready: ground truth, LiDAR, vehicle status, and actuation"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "source_odometry_topic": "/carmaker/odometry",
            "source_pointcloud_topic": "/carmaker/lidar/front/points",
            "source_imu_topic": "/carmaker/imu",
            "source_steering_topic": "/carmaker/steering_angle",
            "output_kinematic_state_topic": "/localization/kinematic_state",
            "output_localization_state_topic": "/localization/initialization_state",
            "initial_pose_topic": "/initialpose3d",
            "output_pose_topic": "/localization/pose_estimator/pose_with_covariance",
            "output_pointcloud_topic": "/sensing/lidar/concatenated/pointcloud",
            "output_imu_topic": "/sensing/imu/carmaker/imu_raw",
            "output_acceleration_topic": "/localization/acceleration",
            "actuation_command_topic": "/control/command/actuation_cmd",
            "gear_command_topic": "/control/command/gear_cmd",
            "vehicle_control_topic": "/carmaker/VehicleControl",
            "clear_route_service": "/api/routing/clear_route",
            "map_frame": "map",
            "base_frame": "base_link",
            "lidar_frame": "carmaker_lidar",
            "imu_frame": "base_link",
            "map_offset_x": 0.0,
            "map_offset_y": 0.0,
            "map_offset_z": 0.0,
            "map_yaw_offset": 0.0,
            "pose_covariance_xy": 0.0001,
            "pose_covariance_yaw": 0.0001,
            "command_timeout_sec": 0.5,
            "timeout_brake": 0.35,
            "steering_command_sign": 1.0,
            "steering_command_ratio": 1.0,
            "steering_feedback_sign": 1.0,
            "steering_feedback_ratio": 1.0,
            "max_steering_angle": 0.70,
            "max_steering_rate": 1.5,
            "selector_drive": 1,
            "selector_reverse": -1,
            "selector_neutral": 0,
            "selector_park": -9,
            "stale_sensor_timeout_sec": 0.5,
            "simulation_reset_tolerance_sec": 0.05,
            "route_clear_retry_sec": 1.0,
            "clear_route_on_simulation_reset": True,
            "enable_cameras": False,
            "require_camera_epoch": False,
            "use_manual_transmission": False,
            "publish_vehicle_control": True,
            "source_base_offset_x": 0.0,
            "source_base_offset_y": 0.0,
            "source_base_offset_z": 0.0,
            "hardware_id": "FSAI_LTS_2026_CM15_Autoware",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        string_names = [
            "source_odometry_topic",
            "source_pointcloud_topic",
            "source_imu_topic",
            "source_steering_topic",
            "output_kinematic_state_topic",
            "output_localization_state_topic",
            "initial_pose_topic",
            "output_pose_topic",
            "output_pointcloud_topic",
            "output_imu_topic",
            "output_acceleration_topic",
            "actuation_command_topic",
            "gear_command_topic",
            "vehicle_control_topic",
            "clear_route_service",
            "map_frame",
            "base_frame",
            "lidar_frame",
            "imu_frame",
            "hardware_id",
        ]
        float_names = [
            "map_offset_x",
            "map_offset_y",
            "map_offset_z",
            "map_yaw_offset",
            "pose_covariance_xy",
            "pose_covariance_yaw",
            "command_timeout_sec",
            "timeout_brake",
            "steering_command_sign",
            "steering_command_ratio",
            "steering_feedback_sign",
            "steering_feedback_ratio",
            "max_steering_angle",
            "max_steering_rate",
            "stale_sensor_timeout_sec",
            "simulation_reset_tolerance_sec",
            "route_clear_retry_sec",
            "source_base_offset_x",
            "source_base_offset_y",
            "source_base_offset_z",
        ]
        for name in string_names:
            setattr(self, name, str(self.get_parameter(name).value))
        for name in float_names:
            setattr(self, name, float(self.get_parameter(name).value))
        integer_names = [
            "selector_drive",
            "selector_reverse",
            "selector_neutral",
            "selector_park",
        ]
        for name in integer_names:
            setattr(self, name, int(self.get_parameter(name).value))
        self.clear_route_on_simulation_reset = bool(
            self.get_parameter("clear_route_on_simulation_reset").value
        )
        self.enable_cameras = bool(self.get_parameter("enable_cameras").value)
        self.require_camera_epoch = bool(self.get_parameter("require_camera_epoch").value)
        self.use_manual_transmission = bool(self.get_parameter("use_manual_transmission").value)
        self.publish_vehicle_control = bool(self.get_parameter("publish_vehicle_control").value)
        if not self.publish_vehicle_control:
            self.get_logger().warning("Data-only run: CarMaker vehicle-command publication is disabled")

    def _on_odometry(self, msg: Odometry) -> None:
        source_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        source_epoch = None
        if self.require_camera_epoch and msg.header.frame_id.startswith("map__epoch_"):
            source_epoch = msg.header.frame_id[len("map__epoch_"):] or None
        epoch_changed = (self.require_camera_epoch and self.camera_expected_epoch is not None
                         and source_epoch is not None and source_epoch != self.camera_expected_epoch)
        if epoch_changed or is_time_regression(
            self.last_source_odometry_stamp_ns,
            source_stamp_ns,
            self.simulation_reset_tolerance_sec,
        ):
            self._handle_simulation_reset(
                self.last_source_odometry_stamp_ns, source_stamp_ns
            )
        self.camera_expected_epoch = source_epoch
        self.last_source_odometry_stamp_ns = source_stamp_ns
        self.last_odometry_wall = time.monotonic()
        self.odometry_count += 1
        output = copy.deepcopy(msg)
        offset = (self.source_base_offset_x, self.source_base_offset_y, self.source_base_offset_z)
        orientation = msg.pose.pose.orientation
        dx, dy, dz = rotate_vector((orientation.x, orientation.y, orientation.z, orientation.w), offset)
        output.pose.pose.position.x += dx
        output.pose.pose.position.y += dy
        output.pose.pose.position.z += dz
        velocity, omega = msg.twist.twist.linear, msg.twist.twist.angular
        vx, vy, vz = shift_velocity((velocity.x, velocity.y, velocity.z),
            (omega.x, omega.y, omega.z), offset)
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = vx, vy, vz
        self.latest_speed = vx
        output.header.stamp = self.get_clock().now().to_msg()
        if self.source_epoch_offset_ns is None:
            self.source_epoch_offset_ns = stamp_to_nanoseconds(output.header.stamp)-source_stamp_ns
        self.latest_localization_stamp = copy.deepcopy(output.header.stamp)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        x, y = transform_xy(
            output.pose.pose.position.x,
            output.pose.pose.position.y,
            self.map_yaw_offset,
            self.map_offset_x,
            self.map_offset_y,
        )
        output.pose.pose.position.x = x
        output.pose.pose.position.y = y
        output.pose.pose.position.z += self.map_offset_z
        yaw_rotation = (0.0, 0.0, math.sin(self.map_yaw_offset / 2.0), math.cos(self.map_yaw_offset / 2.0))
        source_q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        qx, qy, qz, qw = quaternion_multiply(yaw_rotation, source_q)
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.pose.covariance[0] = self.pose_covariance_xy
        output.pose.covariance[7] = self.pose_covariance_xy
        output.pose.covariance[35] = self.pose_covariance_yaw
        pose = PoseWithCovarianceStamped()
        pose.header = output.header
        pose.pose = output.pose
        self.kinematic_pub.publish(output)
        self.pose_pub.publish(pose)

        if not self.localization_initialized:
            # Localization is deliberately not launched: CarMaker is the
            # authoritative pose source. Publish both component state and the
            # standard initialized pose event so AD API and system readiness
            # monitors agree on the external ground-truth localization state.
            state = LocalizationInitializationState()
            state.stamp = output.header.stamp
            state.state = LocalizationInitializationState.INITIALIZED
            self.localization_state_pub.publish(state)
            self.initial_pose_pub.publish(pose)
            self.localization_initialized = True
            self.get_logger().info(
                "CarMaker ground-truth localization initialized and /initialpose3d published"
            )

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self._publish_vehicle_status(output)

    def _publish_vehicle_status(self, odometry: Odometry) -> None:
        velocity = VelocityReport()
        velocity.header.stamp = odometry.header.stamp
        velocity.header.frame_id = self.base_frame
        velocity.longitudinal_velocity = odometry.twist.twist.linear.x
        velocity.lateral_velocity = odometry.twist.twist.linear.y
        velocity.heading_rate = odometry.twist.twist.angular.z
        self.velocity_pub.publish(velocity)

        steering = SteeringReport()
        steering.stamp = odometry.header.stamp
        steering.steering_tire_angle = self.latest_steering_tire_angle
        self.steering_pub.publish(steering)

        gear = GearReport()
        gear.stamp = odometry.header.stamp
        gear.report = self.current_gear_report
        self.gear_pub.publish(gear)

        mode = ControlModeReport()
        mode.stamp = odometry.header.stamp
        mode.mode = ControlModeReport.AUTONOMOUS
        self.control_mode_pub.publish(mode)

        turn_indicators = TurnIndicatorsReport()
        turn_indicators.stamp = odometry.header.stamp
        turn_indicators.report = TurnIndicatorsReport.DISABLE
        self.turn_indicators_pub.publish(turn_indicators)

        hazard_lights = HazardLightsReport()
        hazard_lights.stamp = odometry.header.stamp
        hazard_lights.report = HazardLightsReport.DISABLE
        self.hazard_lights_pub.publish(hazard_lights)

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        source_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if source_stamp_ns == self.last_source_pointcloud_stamp_ns:
            return
        if sensor_precedes_odometry_epoch(
            self.last_source_pointcloud_stamp_ns,
            source_stamp_ns,
            self.last_source_odometry_stamp_ns,
            self.simulation_reset_tolerance_sec,
        ):
            # CarMaker sensor jobs can run before the first odometry job of a
            # restarted TestRun. Do not stamp the new scan with the previous
            # TestRun's pose.
            self.latest_localization_stamp = None
        self.last_source_pointcloud_stamp_ns = source_stamp_ns
        if self.latest_localization_stamp is None:
            self.pointcloud_waiting_for_localization_count += 1
            return

        self.last_pointcloud_wall = time.monotonic()
        self.pointcloud_count += 1
        output = copy.deepcopy(msg)
        output.header.stamp = copy.deepcopy(self.latest_localization_stamp)
        output.header.frame_id = self.lidar_frame
        self.pointcloud_pub.publish(output)

    def _camera_callback(self, name, frame, publisher, is_image):
        def callback(msg):
            # Humble's executor supports message-only callbacks. Epoch IDs
            # are carried by raw frame_id and matched against the odometry TestRun ID.
            self._on_camera(msg, name, frame, publisher, is_image)
        return callback

    def _on_camera(self, msg, name, frame, publisher, is_image):
        if self.latest_localization_stamp is None or self.last_source_odometry_stamp_ns is None:
            return
        if self.require_camera_epoch:
            epoch = msg.header.frame_id
            if (self.camera_expected_epoch is None
                    or epoch != frame+"__epoch_"+self.camera_expected_epoch
                    or epoch in self.camera_retired_epochs):
                return
            self.camera_active_epochs[name] = epoch
        source_stamp = stamp_to_nanoseconds(msg.header.stamp)
        # Images from the previous TestRun must never be placed in the new epoch.
        age = self.last_source_odometry_stamp_ns - source_stamp
        if age < -100_000_000 or age > 500_000_000:
            return
        if is_image:
            if source_stamp == self.camera_last_stamps[name]:
                return
            self.camera_last_stamps[name] = source_stamp
            self.camera_counts[name] += 1
        output = copy.deepcopy(msg)
        epoch_offset = self.source_epoch_offset_ns
        if epoch_offset is None:
            return
        stamp_ns = max(0, source_stamp + epoch_offset)
        output.header.stamp.sec = stamp_ns // NANOSECONDS_PER_SECOND
        output.header.stamp.nanosec = stamp_ns % NANOSECONDS_PER_SECOND
        output.header.frame_id = frame
        publisher.publish(output)

    def _on_imu(self, msg: Imu) -> None:
        source_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        previous_stamp = self.last_source_imu_stamp_ns
        if sensor_precedes_odometry_epoch(
            self.last_source_imu_stamp_ns,
            source_stamp_ns,
            self.last_source_odometry_stamp_ns,
            self.simulation_reset_tolerance_sec,
        ):
            self.latest_localization_stamp = None
        self.last_source_imu_stamp_ns = source_stamp_ns
        if self.latest_localization_stamp is None:
            self.imu_waiting_for_localization_count += 1
            return

        output = copy.deepcopy(msg)
        omega = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        dt = (source_stamp_ns-previous_stamp)/NANOSECONDS_PER_SECOND if previous_stamp is not None else 0.
        alpha = tuple((a-b)/dt for a,b in zip(omega,self.last_imu_omega)) if self.last_imu_omega is not None and 0.001 <= dt <= .1 else (0.,0.,0.)
        self.last_imu_omega = omega
        acceleration = msg.linear_acceleration
        ax, ay, az = shift_acceleration((acceleration.x, acceleration.y, acceleration.z), omega, alpha,
            (self.source_base_offset_x,self.source_base_offset_y,self.source_base_offset_z))
        output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = ax, ay, az
        output.header.stamp = copy.deepcopy(self.latest_localization_stamp)
        output.header.frame_id = self.imu_frame
        self.imu_pub.publish(output)

        acceleration = AccelWithCovarianceStamped()
        acceleration.header = output.header
        acceleration.header.frame_id = self.base_frame
        acceleration.accel.accel.linear = output.linear_acceleration
        acceleration.accel.covariance[0] = output.linear_acceleration_covariance[0]
        acceleration.accel.covariance[7] = output.linear_acceleration_covariance[4]
        acceleration.accel.covariance[14] = output.linear_acceleration_covariance[8]
        self.acceleration_pub.publish(acceleration)

    def _on_steering(self, msg: Float64) -> None:
        ratio = max(abs(self.steering_feedback_ratio), 1.0e-6)
        self.latest_steering_tire_angle = clamp(
            self.steering_feedback_sign * float(msg.data) / ratio,
            -abs(self.max_steering_angle),
            abs(self.max_steering_angle),
        )

    def _on_actuation_command(self, msg: ActuationCommandStamped) -> None:
        if self.command_hold_active:
            return
        self.last_command_wall = time.monotonic()
        self.command_active = True
        self.safe_stop_sent = False
        self.command_count += 1
        self._publish_control(
            clamp(float(msg.actuation.accel_cmd), 0.0, 1.0),
            clamp(float(msg.actuation.brake_cmd), 0.0, 1.0),
            clamp(
                float(msg.actuation.steer_cmd),
                -abs(self.max_steering_angle),
                abs(self.max_steering_angle),
            ),
            0.0,
        )
        self._publish_actuation_status(msg)

    def _on_gear_command(self, msg: GearCommand) -> None:
        command = int(msg.command)
        if GearCommand.DRIVE <= command <= GearCommand.DRIVE_18:
            self.current_selector = self.selector_drive
            self.current_gear_report = command
        elif GearCommand.REVERSE <= command <= GearCommand.REVERSE_2:
            self.current_selector = self.selector_reverse
            self.current_gear_report = command
        elif command == GearCommand.PARK:
            self.current_selector = self.selector_park
            self.current_gear_report = GearReport.PARK
        elif GearCommand.LOW <= command <= GearCommand.LOW_2:
            self.current_selector = self.selector_drive
            self.current_gear_report = command
        else:
            self.current_selector = self.selector_neutral
            self.current_gear_report = GearReport.NEUTRAL

    def _publish_control(
        self, gas: float, brake: float, tire_angle: float, tire_rate: float
    ) -> None:
        if not self.publish_vehicle_control:
            return
        command = VehicleControl()
        command.use_vc = True
        command.selector_ctrl = self.current_selector
        command.gas = clamp(gas, 0.0, 1.0)
        command.brake = clamp(brake, 0.0, 1.0)
        command.steer_ang = self.steering_command_sign * self.steering_command_ratio * clamp(
            tire_angle, -abs(self.max_steering_angle), abs(self.max_steering_angle)
        )
        command.steer_ang_vel = self.steering_command_sign * self.steering_command_ratio * clamp(
            tire_rate, -abs(self.max_steering_rate), abs(self.max_steering_rate)
        )
        command.steer_ang_acc = 0.0
        command.use_manual_transmission = self.use_manual_transmission
        if self.use_manual_transmission:
            odom_stale = self.last_odometry_wall is None or time.monotonic()-self.last_odometry_wall > self.stale_sensor_timeout_sec
            direction = 1 if self.current_selector == self.selector_drive else (-1 if self.current_selector == self.selector_reverse else 0)
            direction_change = direction*self.latest_speed < -0.2
            safe_stop = odom_stale or self.command_hold_active or self.safe_stop_sent or direction_change
            manual = self.transmission.step(self.get_clock().now().nanoseconds / NANOSECONDS_PER_SECOND,
                self.latest_speed, direction, command.gas, command.brake, safe_stop=safe_stop)
            command.gear, command.clutch, command.gas = manual.gear, manual.clutch, manual.gas
            if safe_stop:
                command.brake = max(command.brake, self.timeout_brake)
        self.latest_actuation = (command.gas, command.brake, command.steer_ang)
        self.vehicle_control_pub.publish(command)

    def _publish_actuation_status(self, command: ActuationCommandStamped) -> None:
        status = ActuationStatusStamped()
        status.header = command.header
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.base_frame
        status.status.accel_status = clamp(float(command.actuation.accel_cmd), 0.0, 1.0)
        status.status.brake_status = clamp(float(command.actuation.brake_cmd), 0.0, 1.0)
        status.status.steer_status = clamp(
            float(command.actuation.steer_cmd),
            -abs(self.max_steering_angle),
            abs(self.max_steering_angle),
        )
        self.actuation_status_pub.publish(status)

    def _handle_simulation_reset(
        self, previous_stamp_ns: int, current_stamp_ns: int
    ) -> None:
        self.simulation_reset_count += 1
        self.localization_initialized = False
        self.command_active = False
        self.last_command_wall = None
        self.safe_stop_sent = False
        self.last_pointcloud_wall = None
        self.latest_localization_stamp = None
        self.latest_steering_tire_angle = 0.0
        self.latest_speed = 0.0
        self.transmission.reset()
        self.last_imu_omega = None
        self.source_epoch_offset_ns = None
        self.camera_expected_epoch = None
        self.camera_retired_epochs.update(self.camera_active_epochs.values())
        self.camera_active_epochs.clear()
        self.camera_last_stamps = {name: None for name in self.camera_last_stamps}
        self.current_selector = self.selector_drive
        self.current_gear_report = GearReport.DRIVE
        self.command_hold_active = True
        self.safe_stop_sent = True
        self.route_clear_pending = self.clear_route_on_simulation_reset
        if not self.route_clear_pending:
            self.command_hold_active = False
        self.next_route_clear_attempt_wall = 0.0

        state = LocalizationInitializationState()
        state.stamp = self.get_clock().now().to_msg()
        state.state = LocalizationInitializationState.INITIALIZING
        self.localization_state_pub.publish(state)
        self._publish_control(0.0, self.timeout_brake, 0.0, 0.0)
        self.get_logger().warning(
            "Detected a new CarMaker TestRun "
            f"({previous_stamp_ns / NANOSECONDS_PER_SECOND:.3f}s -> "
            f"{current_stamp_ns / NANOSECONDS_PER_SECOND:.3f}s). "
            "Reinitializing localization and applying safe brake."
        )

    def _try_clear_route(self) -> None:
        if not self.route_clear_pending or self.route_clear_future is not None:
            return
        if time.monotonic() < self.next_route_clear_attempt_wall:
            return
        if not self.clear_route_client.service_is_ready():
            return
        self.next_route_clear_attempt_wall = (
            time.monotonic() + self.route_clear_retry_sec
        )
        self.route_clear_future = self.clear_route_client.call_async(
            ClearRoute.Request()
        )
        self.route_clear_future.add_done_callback(self._on_clear_route_done)

    def _on_clear_route_done(self, future) -> None:
        self.route_clear_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f"Failed to clear the previous TestRun route: {error}; retrying"
            )
            return
        self.route_clear_pending = False
        if response.status.success:
            self.command_hold_active = False
            self.get_logger().info("Cleared route from the previous CarMaker TestRun")
        else:
            self.route_clear_pending = True
            self.get_logger().warning(
                "Route clear was rejected; keeping safe brake and retrying: "
                f"{response.status.code}: {response.status.message}"
            )

    def _watchdog(self) -> None:
        self._try_clear_route()
        if self.command_hold_active:
            self._publish_control(0.0, self.timeout_brake, 0.0, 0.0)
            return
        if not self.command_active or self.last_command_wall is None:
            return
        age = time.monotonic() - self.last_command_wall
        if age <= self.command_timeout_sec or self.safe_stop_sent:
            return
        self.safe_stop_sent = True
        self._publish_control(0.0, self.timeout_brake, 0.0, 0.0)
        action = "applying safe brake" if self.publish_vehicle_control else "vehicle-command publication remains disabled"
        self.get_logger().error(
            f"Autoware actuation command stale for {age:.3f}s; {action}"
        )

    def _publish_diagnostics(self) -> None:
        now_wall = time.monotonic()
        odom_age = None if self.last_odometry_wall is None else now_wall - self.last_odometry_wall
        cloud_age = None if self.last_pointcloud_wall is None else now_wall - self.last_pointcloud_wall
        stale = (
            odom_age is None
            or cloud_age is None
            or odom_age > self.stale_sensor_timeout_sec
            or cloud_age > self.stale_sensor_timeout_sec
        )
        status = DiagnosticStatus()
        status.name = "carmaker/interface"
        status.hardware_id = self.hardware_id
        status.level = DiagnosticStatus.ERROR if stale else DiagnosticStatus.OK
        status.message = "waiting for or receiving stale CarMaker data" if stale else "connected"
        status.values = [
            KeyValue(key="odometry_count", value=str(self.odometry_count)),
            KeyValue(key="pointcloud_count", value=str(self.pointcloud_count)),
            KeyValue(key="command_count", value=str(self.command_count)),
            KeyValue(key="camera_counts", value=str(self.camera_counts)),
            KeyValue(key="manual_gear", value=str(self.transmission.gear)),
            KeyValue(key="command_safe_stop", value=str(self.safe_stop_sent)),
            KeyValue(
                key="simulation_reset_count",
                value=str(self.simulation_reset_count),
            ),
            KeyValue(
                key="route_clear_pending",
                value=str(self.route_clear_pending),
            ),
            KeyValue(
                key="command_hold_active",
                value=str(self.command_hold_active),
            ),
            KeyValue(
                key="last_source_odometry_stamp_ns",
                value=str(self.last_source_odometry_stamp_ns),
            ),
            KeyValue(
                key="last_source_pointcloud_stamp_ns",
                value=str(self.last_source_pointcloud_stamp_ns),
            ),
            KeyValue(
                key="pointcloud_waiting_for_localization_count",
                value=str(self.pointcloud_waiting_for_localization_count),
            ),
            KeyValue(
                key="imu_waiting_for_localization_count",
                value=str(self.imu_waiting_for_localization_count),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarMakerInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
