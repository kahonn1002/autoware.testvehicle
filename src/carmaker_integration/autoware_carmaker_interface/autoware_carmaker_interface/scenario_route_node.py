"""Set Autoware's route from a converted CarMaker TestRun, optionally engaging.

`testrun_to_autoware` stores the goal and the waypoints that pin the TestRun's
route in `<map>/scenario.yaml`. The interface clears the route when a TestRun
restarts; the route is then set again from the same file.
"""

import math
from pathlib import Path

import rclpy
import yaml
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, SetRoutePoints
from geometry_msgs.msg import Pose
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def pose_from_dict(item) -> Pose:
    pose = Pose()
    pose.position.x = float(item["x"])
    pose.position.y = float(item["y"])
    pose.position.z = float(item.get("z", 0.0))
    yaw = float(item.get("yaw", 0.0))
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def load_route(path):
    """Return (goal, waypoints) from a converted scenario file."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    autoware = data.get("autoware") or {}
    if "goal" not in autoware:
        raise ValueError(f"{path} has no autoware.goal")
    return (pose_from_dict(autoware["goal"]),
            [pose_from_dict(item) for item in autoware.get("waypoints") or []])


def next_action(initialized, route_state, mode, auto_engage, engaged):
    """Decide the next AD API request: 'set_route', 'engage' or None."""
    if not initialized:
        return None
    if route_state == RouteState.UNSET:
        return "set_route"
    if (auto_engage and not engaged and route_state == RouteState.SET and mode is not None
            and mode.is_autonomous_mode_available and mode.mode != OperationModeState.AUTONOMOUS):
        return "engage"
    return None


class ScenarioRoute(Node):
    def __init__(self):
        super().__init__("carmaker_scenario_route")
        scenario_file = self.declare_parameter("scenario_file", "").value
        self.auto_engage = bool(self.declare_parameter("auto_engage", False).value)
        self.retry_period = float(self.declare_parameter("retry_period", 2.0).value)
        self.goal, self.waypoints = load_route(scenario_file)
        self.initialized = False
        self.route_state = RouteState.UNKNOWN
        self.mode = None
        self.engaged = False
        self.pending = None
        self.not_before = 0.0
        self.clock_steady = Clock(clock_type=ClockType.STEADY_TIME)

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LocalizationInitializationState,
                                 "/api/localization/initialization_state", self._on_localization, state_qos)
        self.create_subscription(RouteState, "/api/routing/state", self._on_route, state_qos)
        self.create_subscription(OperationModeState, "/api/operation_mode/state", self._on_mode, state_qos)
        self.set_route = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.change_mode = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        # Requests continue while simulation time is paused.
        self.create_timer(0.5, self._step, clock=self.clock_steady)
        self.get_logger().info(
            f"Route from {scenario_file}: goal and {len(self.waypoints)} waypoints, "
            f"auto_engage={self.auto_engage}")

    def _now(self):
        return self.clock_steady.now().nanoseconds * 1e-9

    def _on_localization(self, msg):
        self.initialized = msg.state == LocalizationInitializationState.INITIALIZED

    def _on_route(self, msg):
        if msg.state in (RouteState.UNSET, RouteState.ARRIVED):
            self.engaged = False
        self.route_state = msg.state

    def _on_mode(self, msg):
        self.mode = msg

    def _step(self):
        if self.pending is not None or self._now() < self.not_before:
            return
        action = next_action(self.initialized, self.route_state, self.mode, self.auto_engage, self.engaged)
        if action == "set_route" and self.set_route.service_is_ready():
            request = SetRoutePoints.Request()
            request.header.frame_id = "map"
            request.header.stamp = self.get_clock().now().to_msg()
            request.goal = self.goal
            request.waypoints = self.waypoints
            self._send(action, self.set_route.call_async(request))
        elif action == "engage" and self.change_mode.service_is_ready():
            self._send(action, self.change_mode.call_async(ChangeOperationMode.Request()))

    def _send(self, action, future):
        self.pending = action
        future.add_done_callback(lambda done: self._on_response(action, done))

    def _on_response(self, action, future):
        self.pending = None
        try:
            status = future.result().status
        except Exception as error:  # A failed service call is retried like a rejection.
            status, message = None, str(error)
        else:
            message = status.message
        if status is not None and status.success:
            if action == "set_route":
                # Until /api/routing/state confirms it; UNSET there sets it again.
                self.route_state = RouteState.SET
                self.get_logger().info("Route set from scenario")
            else:
                self.engaged = True
                self.get_logger().info("Autonomous mode requested")
            return
        self.not_before = self._now() + self.retry_period
        self.get_logger().warning(f"{action} rejected ({message}); retrying in {self.retry_period:.1f} s")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioRoute()
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
