"""Exercise real DDS serialization, camera callbacks, manual controls and watchdog.

Uses synthetic sensor messages on an isolated ROS domain, without a simulator.
"""

import argparse
import copy
import json
import math
import time

import rclpy
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from vehiclecontrol_msgs.msg import VehicleControl

from autoware_carmaker_interface.interface_node import CarMakerInterface


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-only", action="store_true", help="Assert no vehicle commands are emitted")
    args = parser.parse_args()
    rclpy.init(args=["--ros-args", "-p", "enable_cameras:=true", "-p", "use_manual_transmission:=true",
        "-p", "require_camera_epoch:=true",
        "-p", "source_base_offset_x:=0.863", "-p", "steering_command_sign:=-1.0", "-p", "steering_command_ratio:=13.66",
        "-p", "output_imu_topic:=/sensing/imu/imu_data",
        "-p", "publish_vehicle_control:="+str(not args.data_only).lower()])
    bridge = CarMakerInterface()
    fixture = rclpy.create_node("iso_fixture")
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(fixture)
    commands, poses, cameras, imus, accels = [], [], [], [], []
    fixture.create_subscription(VehicleControl,"/carmaker/VehicleControl",commands.append,10)
    fixture.create_subscription(Odometry,"/localization/kinematic_state",poses.append,10)
    fixture.create_subscription(Image,"/sensing/camera/front/image_raw",cameras.append,qos_profile_sensor_data)
    # Match AEB's reliable subscriber, not just permissive best-effort probes.
    fixture.create_subscription(Imu,"/sensing/imu/imu_data",imus.append,10)
    fixture.create_subscription(AccelWithCovarianceStamped,"/localization/acceleration",accels.append,10)
    odom_pub = fixture.create_publisher(Odometry,"/carmaker/odometry",10)
    image_pub = fixture.create_publisher(Image,"/VehSensor_0/image_raw",qos_profile_sensor_data)
    info_pub = fixture.create_publisher(CameraInfo,"/VehSensor_0/camera_info",qos_profile_sensor_data)
    cloud_pub = fixture.create_publisher(PointCloud2,"/carmaker/lidar/front/points",qos_profile_sensor_data)
    imu_pub = fixture.create_publisher(Imu,"/carmaker/imu",qos_profile_sensor_data)
    actuation_pub = fixture.create_publisher(ActuationCommandStamped,"/control/command/actuation_cmd",10)
    try:
        start = time.monotonic()
        while time.monotonic()-start < 3.0:
            raw_time = time.monotonic()-start+1.0
            odom = Odometry()
            odom.header.stamp.sec = int(raw_time)
            odom.header.stamp.nanosec = int((raw_time-int(raw_time))*1e9)
            odom.header.frame_id = "map__epoch_fixture"
            odom.pose.pose.position.x = 450.0
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            cloud = PointCloud2()
            cloud.header = copy.deepcopy(odom.header)
            cloud_pub.publish(cloud)
            imu = Imu()
            # Distinct stamp on the same source epoch as odom (+5ms, no regression).
            imu_ns = odom.header.stamp.nanosec+5_000_000
            imu.header.stamp.sec = odom.header.stamp.sec+imu_ns//1_000_000_000
            imu.header.stamp.nanosec = imu_ns%1_000_000_000
            imu.header.frame_id = "carmaker_imu"
            imu.orientation.w = 1.0
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = 0.01, 0.02, 0.03
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = 0.5, -0.2, 0.1
            for cov in (imu.orientation_covariance, imu.angular_velocity_covariance,
                    imu.linear_acceleration_covariance):
                for i in range(9):
                    cov[i] = 0.01 if i in (0, 4, 8) else 0.0
            imu_pub.publish(imu)
            image = Image()
            image.header = copy.deepcopy(odom.header)
            image.header.frame_id = "VehSensor_0__epoch_fixture"
            image.height, image.width, image.encoding, image.step = 1, 1, "rgb8", 3
            image.data = bytes((0,0,0))
            image_pub.publish(image)
            info = CameraInfo()
            info.header = copy.deepcopy(odom.header)
            info.header.frame_id = "VehSensor_0__epoch_fixture"
            info_pub.publish(info)
            command = ActuationCommandStamped()
            command.actuation.accel_cmd = .2
            command.actuation.steer_cmd = .05
            actuation_pub.publish(command)
            until = time.monotonic()+.03
            while time.monotonic()<until:
                executor.spin_once(timeout_sec=.005)
        until = time.monotonic()+.8
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
        assert poses and abs(poses[-1].pose.pose.position.x-450.863)<1e-6
        assert cameras and cameras[-1].header.frame_id == "VehSensor_0"
        if args.data_only:
            assert not commands
        else:
            assert any(cmd.gear == 1 and cmd.clutch < .1 and cmd.gas > 0 for cmd in commands)
            assert any(abs(cmd.steer_ang+.683)<1e-6 for cmd in commands)
            assert commands[-1].gas == 0 and commands[-1].clutch == 1 and commands[-1].gear == 0
            assert commands[-1].brake >= .35
        assert imus and accels
        assert imus[-1].header.frame_id == "base_link"
        assert accels[-1].header.frame_id == "base_link"
        for msg in imus:
            vals = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                *msg.orientation_covariance, *msg.angular_velocity_covariance,
                *msg.linear_acceleration_covariance)
            assert all(math.isfinite(v) for v in vals)
        for msg in accels:
            vals = (msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z,
                *msg.accel.covariance)
            assert all(math.isfinite(v) for v in vals)
        result = {"passed":True,"poses":len(poses),"cameras":len(cameras),"commands":len(commands),
            "imus":len(imus),"accels":len(accels),
            "rear_axle_x":poses[-1].pose.pose.position.x,"data_only":args.data_only,
            "watchdog_brake":commands[-1].brake if commands else None}
        print(json.dumps(result,indent=2),flush=True)
    finally:
        executor.shutdown()
        fixture.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
