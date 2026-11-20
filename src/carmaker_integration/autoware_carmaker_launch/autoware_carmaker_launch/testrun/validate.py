"""Check a generated map the way Autoware's Local projector loads it.

autoware_map_loader overwrites x/y from local_x/local_y and then re-aligns each
lanelet's bounds geometrically; Lanelet2's Python bindings do not expose
`geometry.align`, so the same rule is applied here before routing.
"""

import math

YAW_LIMIT = math.radians(45.0)


def _xy(line):
    return [(point.x, point.y) for point in line]


def signed_distance(points, target):
    """Distance from the closest polyline segment; positive on the left."""
    best = None
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        squared = dx * dx + dy * dy
        ratio = 0.0 if squared == 0 else max(0.0, min(1.0, ((target[0] - ax) * dx + (target[1] - ay) * dy) / squared))
        distance = math.hypot(target[0] - ax - ratio * dx, target[1] - ay - ratio * dy)
        if best is None or distance < best[0]:
            cross = dx * (target[1] - ay) - dy * (target[0] - ax)
            best = (distance, math.copysign(distance, cross))
    return best[1] if best else 0.0


def _middle(points):
    if len(points) > 2:
        return points[len(points) // 2]
    return ((points[0][0] + points[-1][0]) / 2.0, (points[0][1] + points[-1][1]) / 2.0)


def align(lanelet):
    """Port of lanelet::geometry::align as used by autoware_map_loader."""
    left, right = lanelet.leftBound, lanelet.rightBound
    if len(left) < 2 and len(right) < 2:
        return
    if not signed_distance(_xy(left), _middle(_xy(right))) < 0 and len(left) > 1:
        left = left.invert()
    if not signed_distance(_xy(right), _middle(_xy(left))) > 0 and len(right) > 1:
        right = right.invert()
    lanelet.leftBound, lanelet.rightBound = left, right


def _heading_at(lanelet, x, y):
    points = _xy(lanelet.centerline)
    best = min(zip(points, points[1:]), key=lambda segment: math.hypot(
        (segment[0][0] + segment[1][0]) / 2.0 - x, (segment[0][1] + segment[1][1]) / 2.0 - y))
    return math.atan2(best[1][1] - best[0][1], best[1][0] - best[0][0])


def find_lanelet(lanelet_map, geometry, pose):
    """Lanelet containing the pose and facing its yaw, like Autoware's route start."""
    import lanelet2
    point = lanelet2.core.BasicPoint2d(pose.x, pose.y)
    matches = []
    for lanelet in lanelet_map.laneletLayer:
        if not geometry.inside(lanelet, point):
            continue
        error = abs((_heading_at(lanelet, pose.x, pose.y) - pose.yaw + math.pi) % (2 * math.pi) - math.pi)
        if error < YAW_LIMIT:
            matches.append((error, lanelet))
    return min(matches, key=lambda item: item[0])[1] if matches else None


def validate_map(osm_path, start, goal, waypoints=()):
    try:
        import lanelet2
        from lanelet2 import geometry, routing, traffic_rules
    except ImportError as error:
        return {"skipped": f"lanelet2 Python bindings unavailable ({error}); source ROS first"}

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0.0, 0.0))
    lanelet_map, load_errors = lanelet2.io.loadRobust(str(osm_path), projector)
    for point in lanelet_map.pointLayer:
        point.x = float(point.attributes["local_x"])
        point.y = float(point.attributes["local_y"])
    for lanelet in lanelet_map.laneletLayer:
        align(lanelet)
    rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
    graph = routing.RoutingGraph(lanelet_map, rules)
    report = {"load_errors": [str(error) for error in load_errors][:20],
              "lanelets": len(lanelet_map.laneletLayer),
              "routing_graph_errors": [str(error) for error in graph.checkValidity()][:20]}

    checkpoints = [find_lanelet(lanelet_map, geometry, pose) for pose in (start, *waypoints, goal)]
    report["start_lanelet"] = checkpoints[0].id if checkpoints[0] else None
    report["goal_lanelet"] = checkpoints[-1].id if checkpoints[-1] else None
    report["unmatched_waypoints"] = sum(item is None for item in checkpoints[1:-1])
    found, lanelets, length = True, 0, 0.0
    for index, (first, second) in enumerate(zip(checkpoints, checkpoints[1:])):
        path = None if first is None or second is None else (
            [first] if first.id == second.id else graph.shortestPath(first, second))
        if path is None:
            found = False
            report["failed_leg"] = index
            break
        # Each later leg starts on the previous leg's final lanelet.
        legs = list(path)[1:] if index else list(path)
        lanelets += len(legs)
        length += sum(geometry.length2d(item) for item in legs)
    report["route_found"] = found
    if found:
        report["route_lanelets"] = lanelets
        report["route_length_m"] = round(length, 1)
    report["passed"] = not load_errors and found
    return report
