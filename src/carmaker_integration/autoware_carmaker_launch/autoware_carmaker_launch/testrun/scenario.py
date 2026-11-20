"""Ego start, goal, traffic objects and vehicle geometry from a CarMaker TestRun."""

import math
import re
from dataclasses import dataclass
from pathlib import Path

from .infofile import InfoFile

CONE_MIN_OFFSET = 0.75
STATIC_HINTS = ("cone", "pylon", "guidepost", "streetfurniture", "barrier", "bollard", "sign")
DYNAMIC_HINTS = ("vehicles/", "people/", "animals/", "pedestrian", "bicycle", "motorcycle")


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float

    def as_dict(self):
        return {"x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4),
                "yaw": round(self.yaw, 6)}


@dataclass
class EgoStart:
    route: int
    s: float
    t: float
    yaw_offset: float


@dataclass
class TrafficObject:
    name: str
    template: str
    position_type: str
    route: int
    position: list
    yaw_offset: float
    velocity: float
    kind: str
    side: int
    pose: Pose = None


def route_object(testrun):
    """Ego route object id from `Vehicle.Routing` or the start position."""
    for key in ("Vehicle.Routing.ObjId", "Vehicle.StartPos.ObjId"):
        value = testrun.number(key)
        if value is not None:
            return int(value)
    raise ValueError("TestRun defines no ego route (Vehicle.Routing.ObjId)")


def ego_start(testrun, route):
    kind = testrun.get("Vehicle.StartPos.Type", "Route")
    if kind != "Route":
        raise ValueError(f"Vehicle.StartPos.Type {kind} is not supported; use a Route start")
    position = testrun.floats("Vehicle.StartPos", [0.0, 0.0])
    start_route = int(testrun.number("Vehicle.StartPos.ObjId", route))
    if start_route != route:
        raise ValueError(f"Ego starts on route {start_route} but is routed on {route}")
    yaw_offset = 0.0
    if testrun.get("Vehicle.StartPos.Orientation.Type", "Relative") == "Relative":
        yaw_offset = math.radians(testrun.number("Vehicle.StartPos.Orientation", 0.0))
    return EgoStart(route, position[0], position[1] if len(position) > 1 else 0.0, yaw_offset)


def classify_object(template, velocity):
    lowered = template.lower()
    if "cone" in lowered or "pylon" in lowered:
        return "cone"
    if velocity > 0.0 or any(hint in lowered for hint in DYNAMIC_HINTS):
        return "dynamic"
    if any(hint in lowered for hint in STATIC_HINTS):
        return "static"
    return "dynamic"


def cone_side(template, t):
    """Formula Student colours first (blue left, yellow right, orange markers), then t."""
    lowered = template.lower()
    if "blue" in lowered:
        return 1
    if "yellow" in lowered:
        return -1
    if "orange" in lowered or abs(t) < CONE_MIN_OFFSET:
        return 0
    return 1 if t > 0 else -1


def traffic_objects(testrun):
    objects = []
    for index in testrun.indices("Traffic"):
        prefix = f"Traffic.{index}."
        template = testrun.get(prefix + "Template.FName") or testrun.get(prefix + "Movie.Geometry") or ""
        position = testrun.floats(prefix + "StartPos", [0.0, 0.0])
        velocity = testrun.number(prefix + "Man.Start.Velocity", 0.0) or 0.0
        orientation = testrun.floats(prefix + "StartPos.Orientation", [0.0])
        kind = classify_object(template, velocity)
        side = cone_side(template, position[1] if len(position) > 1 else 0.0) if kind == "cone" else 0
        objects.append(TrafficObject(
            testrun.get(prefix + "Name", f"T{index}"), template,
            testrun.get(prefix + "StartPos.Type", "Route"),
            int(testrun.number(prefix + "StartPos.ObjId", -1)), position,
            math.radians(orientation[0]), velocity, kind, side))
    return objects


def route_pose(road, route, s, t=0.0):
    """Pose at route (s, t); wraps on loops and extrapolates past open route ends."""
    length = road.route_length(route)
    if road.route_loop(route) is not None:
        s %= length
    end = min(max(s, 0.0), length)
    point = road.route_point(route, end, t)
    beyond = s - end
    return Pose(point.x + beyond * math.cos(point.heading),
                point.y + beyond * math.sin(point.heading), point.z, point.heading)


def locate_traffic(road, objects, warnings):
    """Fill global poses of route-positioned traffic objects."""
    for item in objects:
        try:
            if item.position_type == "Route":
                pose = route_pose(road, item.route, item.position[0], item.position[1])
                pose.yaw += item.yaw_offset
                item.pose = pose
            elif item.position_type == "Global" and len(item.position) >= 2:
                z = item.position[2] if len(item.position) > 2 else 0.0
                item.pose = Pose(item.position[0], item.position[1], z, item.yaw_offset)
        except Exception as error:  # IPGRoad reports positions off the road as errors.
            warnings.append(f"Traffic object {item.name} could not be placed: {error}")
    return objects


def goal_station(length, start_s, closed, margin, chunk_length):
    """Goal on the ego route: route end, or one lap minus a lanelet for loops.

    On loops the goal stays a full lanelet behind the start; Autoware cannot
    route to a goal behind the start within the same lanelet.
    """
    if closed:
        return (start_s - margin - chunk_length) % length
    goal = length - margin
    if goal <= start_s + 1.0:
        raise ValueError(f"Route ends {length - start_s:.1f} m after the start; no room for a goal")
    return goal


def waypoint_stations(route_paths, pinned_paths, start_s, goal_s, length, closed, clearance=20.0):
    """Route s of one waypoint per pinned (link lane) path segment, in driving order.

    Without waypoints Autoware takes the shortest route; one point between every
    pair of junctions makes it follow the TestRun's route instead.
    """
    driven = (goal_s - start_s) % length if closed else goal_s - start_s
    stations, offset = [], 0.0
    for path, first, last in route_paths:
        used = abs(last - first)
        if path in pinned_paths and used >= 10.0:
            ahead = ((offset + used / 2.0) - start_s) % length if closed else offset + used / 2.0 - start_s
            if clearance <= ahead <= driven - clearance:
                stations.append((ahead, (start_s + ahead) % length if closed else start_s + ahead))
        offset += used
    return [station for _, station in sorted(stations)]


def vehicle_file(project, name, carmaker_home):
    for root in (Path(project), Path(carmaker_home)):
        candidate = root / "Data" / "Vehicle" / name
        if candidate.is_file():
            return candidate
    return None


def vehicle_geometry(info: InfoFile):
    """Autoware vehicle_info quantities derived from a CarMaker vehicle (Fr1) file."""
    wheels = {name: info.floats(f"Wheel.{name}.pos") or info.floats(f"WheelCarrier.{name}.pos")
              for name in ("fl", "fr", "rl", "rr")}
    if any(value is None for value in wheels.values()):
        return None
    front_x = (wheels["fl"][0] + wheels["fr"][0]) / 2.0
    rear_x = (wheels["rl"][0] + wheels["rr"][0]) / 2.0
    geometry = {"rear_axle_x": rear_x, "wheel_base": front_x - rear_x,
                "wheel_tread": abs(wheels["fl"][1] - wheels["fr"][1])}
    skin = info.floats("Vehicle.OuterSkin")
    if skin and len(skin) >= 6:
        geometry.update({
            "front_overhang": max(skin[0], skin[3]) - front_x,
            "rear_overhang": rear_x - min(skin[0], skin[3]),
            "vehicle_width": abs(skin[1] - skin[4]),
            "vehicle_height": max(skin[2], skin[5]),
        })
    tire = re.search(r"(\d{3})_(\d{2})R(\d{2})", info.get("Tire.0", "") or "")
    if tire:
        width, ratio, rim = (int(value) for value in tire.groups())
        geometry["wheel_width"] = width / 1000.0
        geometry["wheel_radius"] = (rim * 25.4 / 2.0 + width * ratio / 100.0) / 1000.0
    return {key: round(value, 4) for key, value in geometry.items()}
