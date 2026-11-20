#!/usr/bin/env python3
"""Bounded, isolated ISO timing test. Never enables autonomous mode or actuates.

The bridge's vehicle-command publication is disabled. Only a route is requested;
the freshly started copied TestRun retains its neutral/clutch-open/brake hold.
"""

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import struct
import subprocess
import tempfile
import time

import rclpy
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from install_overlay import prepare_scenario


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def summarize(values):
    if not values:
        return None
    ordered = sorted(values)
    return {"count": len(values), "min": ordered[0], "median": statistics.median(ordered),
            "p95": ordered[min(len(ordered)-1, int(.95*len(ordered)))], "max": ordered[-1]}


def cloud_sample_count(msg):
    """Count finite, nonzero sampled returns, not just advertised cloud width."""
    fields = {field.name: field for field in msg.fields}
    if not all(key in fields and fields[key].datatype in (7, 8) for key in ("x", "y", "z")):
        return 0
    total = msg.width * msg.height
    if not total or not msg.point_step:
        return 0
    data = memoryview(msg.data)
    valid = 0
    for index in range(0, total, max(1, total//64)):
        row, col = divmod(index, msg.width)
        base = row*msg.row_step + col*msg.point_step
        coordinates = []
        for key in ("x", "y", "z"):
            field = fields[key]
            fmt = (">" if msg.is_bigendian else "<") + ("f" if field.datatype == 7 else "d")
            if base + field.offset + struct.calcsize(fmt) > len(data):
                return valid
            coordinates.append(struct.unpack_from(fmt, data, base+field.offset)[0])
        if all(math.isfinite(value) for value in coordinates) and sum(v*v for v in coordinates) > .0001:
            valid += 1
    return valid


def evaluate_report(report):
    """Keep useful runtime evidence separate from a clean end-to-end pass."""
    topics = report.get("topics", {})
    final = topics.get("/planning/trajectory", {})
    raw = topics.get("/carmaker/lidar/front/points", {})
    safety = report.get("safety", {})
    steady = final.get("steady_age_sec")
    report["readiness_passed"] = bool(
        "error" not in report and report.get("command_inhibit_verified", False)
        and report.get("route", {}).get("success", False)
        and raw.get("valid_xyz_samples", 0) > 0
        and topics.get("/sensing/imu/imu_data", {}).get("messages", 0) > 0
        and final.get("messages", 0) >= 10
        and report.get("autonomous_available_observations", 0) > 0
        and steady is not None and -.1 <= steady["p95"] <= 1.
        and steady["min"] > -.25
        and safety.get("vehicle_commands_observed") == 0
        and safety.get("autonomous_mode_seen") is False
        and safety.get("max_speed_mps", float("inf")) <= .02)
    exits = report.get("children_exit", {})
    report["cleanup_passed"] = bool(
        not report.get("office_cleanup_timeout", False)
        and not report.get("cleanup_errors")
        and not report.get("shutdown_segfault", False)
        and all(exits.get(name) == 0 for name in ("carmaker", "autoware", "hellocm")))
    report["passed"] = report["readiness_passed"] and report["cleanup_passed"]


def gpu_child_identity(proc, parent_pid, project):
    """Identify only MovieNX children of this run's Office, before reparenting."""
    try:
        fields = (proc/"stat").read_text().rsplit(") ", 1)[1].split()
        if fields[0] == "Z" or int(fields[1]) != parent_pid or proc.stat().st_uid != os.getuid():
            return None
        if os.readlink(proc/"exe") != "/opt/ipg/movienx/linux64-15.0/bin/MovieNX.exe":
            return None
        args = (proc/"cmdline").read_bytes().decode().split("\0")
        required = {"-mode": "GPUSensor", "-projectdir": str(project)}
        if not all(args.count(key) == 1 and args.index(key)+1 < len(args)
                   and args[args.index(key)+1] == value for key, value in required.items()):
            return None
        return fields[19]  # Linux /proc stat field 22: process start time.
    except (FileNotFoundError, ProcessLookupError):
        return None


def track_gpu_children(parent_pid, project, handles):
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in handles:
            continue
        identity = gpu_child_identity(proc, parent_pid, project)
        if identity is None:
            continue
        try:
            fd = os.pidfd_open(int(proc.name))
        except ProcessLookupError:
            continue
        # Hold a stable kernel handle, not a PID that might later be reused.
        if gpu_child_identity(proc, parent_pid, project) != identity:
            os.close(fd)
            continue
        handles[int(proc.name)] = fd


def stop_gpu_children(handles, report):
    """Release only previously verified owned renderers; never adopt or kill all."""
    report["gpu_processes_owned"] = sorted(handles)
    remaining = dict(handles)
    try:
        for pid, fd in handles.items():
            try:
                signal.pidfd_send_signal(fd, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except OSError as error:
                report.setdefault("cleanup_errors", []).append(f"GPU {pid}: {error}")
        deadline = time.monotonic()+8.
        while remaining and time.monotonic() < deadline:
            exited, _, _ = select.select(list(remaining.values()), [], [], .1)
            remaining = {pid: fd for pid, fd in remaining.items() if fd not in exited}
        report["gpu_processes_remaining"] = sorted(remaining)
        if remaining:
            report.setdefault("cleanup_errors", []).append("Owned GPU processes did not exit after SIGTERM")
    finally:
        for fd in handles.values():
            os.close(fd)


class Probe:
    def __init__(self, node):
        self.node = node
        self.clock = None
        self.subscriptions = {}
        self.topics = {}
        self.ages = {}
        self.steady_ages = {}
        self.metrics = {}
        self.route_wall = None
        self.commands = 0
        self.max_speed = 0.
        self.autonomous_seen = False
        self.available_count = 0

    def discover(self):
        wanted = {"/clock", "/carmaker/odometry", "/localization/kinematic_state", "/carmaker/imu",
                  "/sensing/imu/imu_data", "/localization/acceleration", "/carmaker/VehicleControl",
                  "/carmaker/lidar/front/points", "/sensing/lidar/concatenated/pointcloud",
                  "/perception/obstacle_segmentation/pointcloud",
                  "/perception/object_recognition/detection/pointcloud_map_filtered/pointcloud",
                  "/perception/object_recognition/objects", "/perception/occupancy_grid_map/map",
                  "/planning/mission_planning/route", "/api/operation_mode/state",
                  "/system/operation_mode/state", "/planning/trajectory",
                  "/control/trajectory_follower/control_cmd", "/control/command/control_cmd",
                  "/control/command/actuation_cmd"}
        for topic, types in self.node.get_topic_names_and_types():
            if topic in self.subscriptions or len(types) != 1:
                continue
            kind = types[0]
            # Subscribe only to actual pipeline stages, not every large debug
            # trajectory/per-module metric: an overloaded probe biases its ages.
            path = (topic.startswith("/planning/scenario_planning/") and "/debug/" not in topic
                    and kind.endswith(("/Path", "/PathWithLaneId", "/Trajectory")))
            metric = (topic.startswith("/planning/")
                      and topic.endswith(("/debug/processing_time_ms", "/debug/total_time/processing_time_ms"))
                      and kind.endswith(("/Float64Stamped", "/Float32Stamped")))
            if topic not in wanted and not path and not metric:
                continue
            try:
                message_type = get_message(kind)
            except (ImportError, AttributeError, ModuleNotFoundError):
                continue
            # Keep only the newest sample, especially /clock. A five-deep clock
            # queue can make fresh IMU stamps appear spuriously in the future.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic.endswith("operation_mode/state"):
                publishers = self.node.get_publishers_info_by_topic(topic)
                if publishers:
                    qos = publishers[0].qos_profile
            self.subscriptions[topic] = self.node.create_subscription(message_type, topic, self.callback(topic), qos)

    def callback(self, topic):
        def receive_message(msg):
            self.receive(topic, msg)
        return receive_message

    def receive(self, topic, msg):
        item = self.topics.setdefault(topic, {"messages": 0})
        item["messages"] += 1
        if topic == "/clock":
            self.clock = stamp_seconds(msg.clock)
            return
        if topic == "/carmaker/VehicleControl":
            self.commands += 1
        if topic == "/carmaker/odometry":
            self.max_speed = max(self.max_speed, abs(msg.twist.twist.linear.x))
            item["position"] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if topic.endswith("operation_mode/state"):
            self.autonomous_seen |= msg.mode == msg.AUTONOMOUS
            self.available_count += int(msg.is_autonomous_mode_available)
            item["state"] = {"mode": msg.mode, "autonomous_available": msg.is_autonomous_mode_available,
                             "control_enabled": msg.is_autoware_control_enabled}
        if hasattr(msg, "fields") and hasattr(msg, "point_step"):
            item["last_points"] = msg.width*msg.height
            if item["messages"] % 10 == 1:
                item["valid_xyz_samples"] = item.get("valid_xyz_samples", 0) + cloud_sample_count(msg)
        if hasattr(msg, "objects"):
            item["last_objects"] = len(msg.objects)
        if hasattr(msg, "points"):
            item["last_points"] = len(msg.points)
            speeds = [abs(point.longitudinal_velocity_mps) for point in msg.points
                      if hasattr(point, "longitudinal_velocity_mps")]
            if speeds:
                item["last_max_planned_speed_mps"] = max(speeds)
        if hasattr(msg, "header") and self.clock is not None:
            age = self.clock-stamp_seconds(msg.header.stamp)
            self.ages.setdefault(topic, []).append(age)
            if self.route_wall is not None and time.monotonic()-self.route_wall > 10.:
                self.steady_ages.setdefault(topic, []).append(age)
        if isinstance(getattr(msg, "data", None), (float, int)):
            self.metrics.setdefault(topic, []).append(float(msg.data))

    def result(self):
        for topic, item in self.topics.items():
            item["age_sec"] = summarize(self.ages.get(topic, []))
            item["steady_age_sec"] = summarize(self.steady_ages.get(topic, []))
            if topic in self.metrics:
                item["metric"] = summarize(self.metrics[topic])
        return {"topics": self.topics, "safety": {"vehicle_commands_observed": self.commands,
                "max_speed_mps": self.max_speed, "autonomous_mode_seen": self.autonomous_seen},
                "autonomous_available_observations": self.available_count}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--domain-id", type=int, default=57)
    parser.add_argument("--camera-bridge", choices=("true", "false"), default="true")
    parser.add_argument("--office", action="store_true", help="Use owned CarMaker Office for GPU-sensor startup")
    parser.add_argument("--report", type=Path, default=Path("/tmp/iso-stationary-report.json"))
    args = parser.parse_args()
    if args.office and not (hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal")):
        raise RuntimeError("Office diagnostics require Linux pidfd support for owned GPU cleanup")
    project = args.project.resolve(strict=True)
    if project.name != "FSAI_LTS_2026_CM15_Autoware" or not (project/"src/CM_Main.cpp").is_file():
        raise ValueError("Only the duplicated CM15 project is allowed")
    if not 20 <= args.seconds <= 120 or not 1 <= args.domain_id <= 100:
        raise ValueError("Use 20..120 seconds and an isolated ROS domain in 1..100")
    expected = prepare_scenario((project/"Data/TestRun/LaneChange_ISO_Camera_RSI").read_text())
    if (project/"Data/TestRun/LaneChange_ISO_Autoware").read_text() != expected:
        raise ValueError("Copied TestRun differs from the verified neutral/clutch-open/brake hold")
    report_path = args.report.resolve()
    if not report_path.is_relative_to(Path("/tmp")) or args.report.is_symlink():
        raise ValueError("Diagnostic report must be an ordinary file under /tmp")
    if not report_path.parent.is_dir() or (report_path.exists() and not report_path.is_file()):
        raise ValueError("Diagnostic report must have an existing parent and cannot be a directory")
    logs = Path(tempfile.mkdtemp(prefix="iso-stationary-"))
    print(f"Stationary test logs: {logs}", flush=True)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOG_DIR"] = str(logs/"ros")
    rclpy.init()
    node = rclpy.create_node("iso_stationary_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    probe = Probe(node)
    children, streams, gpu_handles = [], [], {}
    report = {"logs": str(logs), "camera_bridge": args.camera_bridge, "domain_id": args.domain_id,
              "office": args.office}

    def start(name, command):
        stream = (logs/(name+".log")).open("w")
        streams.append(stream)
        child = subprocess.Popen(command, cwd=project, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, child))
        return child

    try:
        before = time.monotonic()+2.
        while time.monotonic() < before:
            executor.spin_once(timeout_sec=.05)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others:
            raise RuntimeError(f"ROS domain {args.domain_id} is not empty: {others}")
        start("hellocm", ["ros2", "launch", "hellocm", "hellocm.launch.py", "use_sim_time:=True", "rviz2:=False"])
        start("autoware", ["ros2", "launch", "autoware_carmaker_launch", "carmaker.launch.py",
                           "profile:=iso_lanechange_demo", "rviz:=false", "publish_vehicle_control:=false",
                           "camera_bridge:="+args.camera_bridge])
        route_client = node.create_client(SetRoutePoints, "/api/routing/set_route_points")
        guard_client = node.create_client(GetParameters, "/autoware_carmaker_interface/get_parameters")
        guard_future = None
        guard_verified = False
        started = time.monotonic()
        next_discovery = 0.
        cm = None
        route_future = None
        while time.monotonic()-started < args.seconds+(110. if args.office else 65.):
            executor.spin_once(timeout_sec=.01)
            now = time.monotonic()
            if now >= next_discovery:
                probe.discover()
                if cm is not None:
                    if args.office:
                        track_gpu_children(cm.pid, project, gpu_handles)
                    if "\nSIM_ABORT\t" in (logs/"carmaker.log").read_text(errors="replace"):
                        raise RuntimeError("CarMaker aborted the TestRun; see carmaker.log")
                next_discovery = now+1.
            if probe.commands or probe.max_speed > .02 or probe.autonomous_seen:
                raise RuntimeError("Stationary guard tripped; stopping only owned test processes")
            if any(child.poll() is not None for name, child in children if name != "carmaker"):
                raise RuntimeError("ROS launch exited before the timing test completed")
            if guard_future is None and guard_client.service_is_ready():
                guard_request = GetParameters.Request()
                guard_request.names = ["publish_vehicle_control"]
                guard_future = guard_client.call_async(guard_request)
            if guard_future is not None and guard_future.done() and not guard_verified:
                values = guard_future.result().values
                if len(values) != 1 or values[0].type != 1 or values[0].bool_value:
                    raise RuntimeError("Bridge command-publication inhibit was not verified")
                guard_verified = True
                report["command_inhibit_verified"] = True
            if cm is None and now-started > 25. and route_client.service_is_ready() and guard_verified:
                if args.office:
                    # Refuse to adopt/interfere with an existing simulation backend.
                    listing = subprocess.run(["ps", "-eo", "comm="], capture_output=True, text=True, check=True)
                    if any(name.strip().startswith(("CarMaker.", "MovieNX"))
                           for name in listing.stdout.splitlines()):
                        raise RuntimeError("CarMaker/MovieNX processes already exist; leave them untouched")
                    script = str(Path(__file__).resolve().with_name("stationary_office.tcl"))
                    if any(character in script for character in "{}\n\r"):
                        raise ValueError("Unexpected Tcl startup-script path")
                    os.environ.update(CM_STATIONARY_GUARD="verified", CM_STATIONARY_SECONDS=str(args.seconds),
                                      CM_STATIONARY_LOGS=str(logs))
                    command = ["/opt/ipg/bin/CM_Office-15.0", str(project), "-apphost", "localhost",
                               "-ext", str(project/"GUI/CMExt-CMRosIF.mod"), "-debuggpu", "-cmd", "source {"+script+"}"]
                else:
                    command = [str(project/"bin/CarMaker.linux64"), "-screen", "-tstop", str(args.seconds),
                               "-taccel", "1", "LaneChange_ISO_Autoware"]
                cm = start("carmaker", command)
                print("Stationary CarMaker test started (command publication disabled)", flush=True)
            if cm is not None and cm.poll() is not None:
                report["carmaker_exit"] = cm.returncode
                break
            if (route_future is None and probe.clock is not None and route_client.service_is_ready()
                    and probe.topics.get("/localization/kinematic_state", {}).get("messages", 0) > 20):
                request = SetRoutePoints.Request()
                request.header.frame_id = "map"
                request.header.stamp.sec = int(probe.clock)
                request.header.stamp.nanosec = int((probe.clock-int(probe.clock))*1e9)
                request.goal.position.x = 650.
                request.goal.orientation.w = 1.
                route_future = route_client.call_async(request)
            if route_future is not None and route_future.done() and "route" not in report:
                response = route_future.result()
                report["route"] = {"success": response.status.success, "code": response.status.code,
                                   "message": response.status.message}
                if response.status.success:
                    probe.route_wall = now
                print("Stationary route response: "+json.dumps(report["route"]), flush=True)
        else:
            report["error"] = "Bounded timing test reached its wall-clock deadline"
    except Exception as error:
        report["error"] = str(error)
    finally:
        if args.office:
            for name, child in children:
                if name == "carmaker" and child.poll() is None:
                    try:
                        track_gpu_children(child.pid, project, gpu_handles)
                    except (OSError, ValueError, IndexError) as error:
                        report.setdefault("cleanup_errors", []).append("GPU ownership discovery: "+str(error))
        # No broad kill: stop exactly the process groups created by this run.
        for name, child in reversed(children):
            if child.poll() is None:
                if name == "carmaker" and args.office:
                    (logs/"stop_requested").touch()
                    try:
                        child.wait(timeout=12.)
                    except subprocess.TimeoutExpired:
                        report["office_cleanup_timeout"] = True
                    if child.poll() is not None:
                        continue
                try:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=8.)
                    except subprocess.TimeoutExpired:
                        report.setdefault("cleanup_errors", []).append(name+": SIGINT timeout")
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=8.)
                except ProcessLookupError:
                    child.poll()
                except (OSError, subprocess.TimeoutExpired) as error:
                    report.setdefault("cleanup_errors", []).append(name+": "+str(error))
        if args.office:
            stop_gpu_children(gpu_handles, report)
        for stream in streams:
            stream.close()
        report.update(probe.result())
        report["children_exit"] = {name: child.returncode for name, child in children}
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        report["shutdown_segfault"] = "exit code -11" in (logs/"autoware.log").read_text() if (logs/"autoware.log").exists() else False
        evaluate_report(report)
        report_path.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({key: value for key, value in report.items() if key != "topics"}, indent=2), flush=True)
        print(f"Detailed timing report: {report_path}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
