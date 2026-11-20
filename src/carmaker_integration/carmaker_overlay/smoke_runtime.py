#!/usr/bin/env python3
"""Bounded CarMaker commissioning. Actuation requires explicit --drive-check."""

import argparse
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from vehiclecontrol_msgs.msg import VehicleControl


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project",type=Path,required=True)
    parser.add_argument("--seconds",type=float,default=10.)
    parser.add_argument("--no-rsi",action="store_true",help="Verify vehicle/odometry without licensed GPU sensors")
    parser.add_argument("--drive-check",action="store_true",help="Bounded low-speed control test; requires --no-rsi")
    parser.add_argument("--watchdog-check",action="store_true",help="Stop sending controls before test end to verify the CM watchdog; requires --drive-check")
    parser.add_argument("--report",type=Path,default=Path("/tmp/iso-runtime-report.json"))
    args = parser.parse_args()
    project = args.project.resolve(strict=True)
    if project.name != "FSAI_LTS_2026_CM15_Autoware" or not 1 <= args.seconds <= 30:
        raise ValueError("Expected duplicated CM15 project and 1..30 second test")
    if args.drive_check and not args.no_rsi:
        raise ValueError("Commission controls with --no-rsi before the RSI run")
    if args.watchdog_check and (not args.drive_check or args.seconds < 8):
        raise ValueError("Watchdog check requires --drive-check and at least eight seconds")
    logs = Path(tempfile.mkdtemp(prefix="iso-carmaker-smoke-"))
    env = dict(os.environ,ROS_LOG_DIR=str(logs/"ros"))
    processes, streams = [], []
    report = {"logs":str(logs),"messages":{},"nonempty_sensor_messages":{},"camera_info":{},"positions":[]}
    def start(name,command):
        stream = (logs/(name+".log")).open("w")
        streams.append(stream)
        process = subprocess.Popen(command,cwd=project,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process
    rclpy.init()
    node = rclpy.create_node("iso_sensor_smoke")
    control_pub = node.create_publisher(VehicleControl,"/carmaker/VehicleControl",10)
    latest = None
    max_speed = 0.
    def receive(msg,topic):
        nonlocal latest, max_speed
        report["messages"][topic] = report["messages"].get(topic,0)+1
        if isinstance(msg,(Image,PointCloud2)) and msg.width > 0 and msg.height > 0 and msg.data:
            report["nonempty_sensor_messages"][topic] = report["nonempty_sensor_messages"].get(topic,0)+1
        if isinstance(msg,CameraInfo):
            report["camera_info"][topic] = {"width":msg.width,"height":msg.height,"k":list(msg.k),"frame":msg.header.frame_id}
        if isinstance(msg,Odometry) and len(report["positions"]) < 20:
            report["positions"].append([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x])
        if isinstance(msg,Odometry):
            latest = msg
            max_speed = max(max_speed,abs(msg.twist.twist.linear.x))
            report["raw_odometry_frame"] = msg.header.frame_id
    def subscriber(topic):
        def callback(msg):
            receive(msg,topic)
        return callback
    for cls,topic in ((Odometry,"/carmaker/odometry"),(PointCloud2,"/carmaker/lidar/front/points"),
        (Image,"/VehSensor_0/image_raw"),(CameraInfo,"/VehSensor_0/camera_info"),
        (Image,"/VehSensor_1/image_raw"),(CameraInfo,"/VehSensor_1/camera_info")):
        node.create_subscription(cls,topic,subscriber(topic),qos_profile_sensor_data)
    try:
        start("hellocm",["ros2","launch","hellocm","hellocm.launch.py","use_sim_time:=True","rviz2:=False"])
        deadline = time.monotonic()+8.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if any(name == "/hellocm/init" for name,_ in node.get_service_names_and_types()):
                break
        else:
            raise RuntimeError("hellocm init service unavailable")
        cm = start("carmaker",[str(project/"bin/CarMaker.linux64"),"-screen","-tstop",str(args.seconds),
            "-taccel","1","LaneChange_ISO_Autoware_NoRSI" if args.no_rsi else "LaneChange_ISO_Autoware"])
        deadline = time.monotonic()+args.seconds+25.
        while cm.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if args.drive_check and latest is not None:
                t = latest.header.stamp.sec+1e-9*latest.header.stamp.nanosec
                if args.watchdog_check and t >= args.seconds-3.:
                    continue
                command = VehicleControl()
                command.use_vc = True
                command.use_manual_transmission = True
                command.selector_ctrl = 1
                command.gear = 1
                command.clutch = max(0.,1.-max(0.,t-1.)/2.)
                speed = latest.twist.twist.linear.x
                command.gas = .15 if 1. < t < args.seconds-3. and speed < 1.4 else 0.
                command.brake = .6 if t >= args.seconds-3. or abs(speed)>1.8 else 0.
                if command.brake > 0.:
                    command.clutch = 1.
                control_pub.publish(command)
        report["carmaker_exit"] = cm.poll()
        report["no_rsi"] = args.no_rsi
        report["max_speed_mps"] = max_speed
        report["final_speed_mps"] = latest.twist.twist.linear.x if latest is not None else None
        report["watchdog_check"] = args.watchdog_check
        report["final_pose"] = [latest.pose.pose.position.x,latest.pose.pose.position.y] if latest is not None else None
        sensors_ok = all(report["nonempty_sensor_messages"].get(topic,0)>0 for topic in (
            "/carmaker/lidar/front/points", "/VehSensor_0/image_raw", "/VehSensor_1/image_raw"))
        sensors_ok = sensors_ok and all(topic in report["camera_info"] for topic in (
            "/VehSensor_0/camera_info", "/VehSensor_1/camera_info"))
        report["passed"] = report["messages"].get("/carmaker/odometry",0)>0 and (args.no_rsi or sensors_ok)
        if args.drive_check:
            report["passed"] = report["passed"] and .1 < max_speed < 2.0 and abs(latest.twist.twist.linear.x)<.1
    except Exception as error:
        report["error"] = str(error)
        report["passed"] = False
    finally:
        # Only process groups started by this harness are stopped.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=5.)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        cm_log = (logs/"carmaker.log").read_text() if (logs/"carmaker.log").exists() else ""
        report["watchdog_observed"] = "Autoware VC watchdog: gas=0 brake=0.5 gear=0 clutch=1" in cm_log
        if args.watchdog_check:
            report["passed"] = report["passed"] and report["watchdog_observed"]
        args.report.write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2),flush=True)
        print(cm_log[-5000:] or "No CarMaker log",flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
