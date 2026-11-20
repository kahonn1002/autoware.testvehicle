"""Unit tests of the TestRun converter that need neither CarMaker nor a simulator."""

import math
from xml.etree import ElementTree as ET

import pytest

from autoware_carmaker_launch.generate_iso_map import course_point
from autoware_carmaker_launch.testrun import cli, profiles
from autoware_carmaker_launch.testrun.corridor import (ConeProfile, PylonProfile, build_corridor,
                                                       path_profile, self_overlap)
from autoware_carmaker_launch.testrun.infofile import InfoFile, parse_named_values
from autoware_carmaker_launch.testrun.lanelet_map import LaneletMap, chunk_bounds, lanelet_tags
from autoware_carmaker_launch.testrun.network import _border_tags, turn_direction
from autoware_carmaker_launch.testrun.road_api import RoutePoint
from autoware_carmaker_launch.testrun.scenario import (classify_object, cone_side, goal_station,
                                                       route_pose, vehicle_geometry,
                                                       waypoint_stations)
from autoware_carmaker_launch.testrun.validate import signed_distance, validate_map
from autoware_carmaker_launch.testrun.scenario import Pose

ISO_SECTIONS = [[500.0, 515.0, 0.0, 2.142], [545.0, 570.0, 3.5, 2.314], [595.0, 625.0, 0.0, 2.486]]


class StraightRoad:
    """Route along +x; t is +y and the driving path is the reference line."""

    def __init__(self, length=100.0, width=3.0, markers=()):
        self.length, self.width, self.markers = length, width, list(markers)

    def route_length(self, route):
        return self.length

    def route_loop(self, route):
        return None

    def route_point(self, route, s, t=0.0, use_path=True):
        return RoutePoint(s, t, 0.0, 0.0, s, t, self.width / 2.0, self.width / 2.0)

    def route_project(self, route, x, y, z=0.0, use_path=True):
        return self.route_point(route, x, y)

    def route_markers(self, route, kinds):
        return self.markers


class CircleRoad(StraightRoad):
    """Counter-clockwise circle(s) of radius r starting at (0, -r); t points to the centre."""

    def __init__(self, radius=20.0, laps=1, closed=True):
        super().__init__(2.0 * math.pi * radius * laps)
        self.radius, self.closed = radius, closed

    def route_loop(self, route):
        return (0.0, self.length) if self.closed else None

    def route_point(self, route, s, t=0.0, use_path=True):
        angle = -math.pi / 2.0 + s / self.radius
        distance = self.radius - t
        return RoutePoint(distance * math.cos(angle), distance * math.sin(angle), 0.0,
                          angle + math.pi / 2.0, s, t, 1.5, 1.5)


def _osm(path):
    root = ET.parse(path).getroot()
    nodes = {node.get("id") for node in root.iter("node")}
    ways = {way.get("id"): [nd.get("ref") for nd in way.iter("nd")] for way in root.iter("way")}
    relations = {relation.get("id"): ({member.get("role"): member.get("ref") for member in relation.iter("member")},
                                      {tag.get("k"): tag.get("v") for tag in relation.iter("tag")})
                 for relation in root.iter("relation")}
    return nodes, ways, relations


# InfoFile -----------------------------------------------------------------

def test_infofile_parses_values_blocks_and_named_values():
    info = InfoFile.parse(
        "#INFOFILE1.1 (UTF-8) - Do not remove this line!\n"
        "Vehicle = Examples/DemoCar\n"
        "Driver.Vel.CruisingSpeed = $Speed=90\n"
        "Description:\n\tline one\n\tline two\n"
        "Traffic.0.StartPos = 8 2.0\n"
        "Traffic.12.Name = T12\n"
        "Empty =\n")
    assert info.get("Vehicle") == "Examples/DemoCar"
    assert info.number("Driver.Vel.CruisingSpeed") == 90.0
    assert info.lines("Description") == ["line one", "line two"]
    assert info.floats("Traffic.0.StartPos") == [8.0, 2.0]
    assert info.indices("Traffic") == [0, 12]
    assert info.get("Empty") == ""
    assert info.get("Missing", "x") == "x"
    overridden = InfoFile.parse("Speed = $Speed=90\n", named_values=parse_named_values(["$Speed=40"]))
    assert overridden.number("Speed") == 40.0
    with pytest.raises(ValueError):
        InfoFile.parse("Speed = $Speed\n").get("Speed")
    with pytest.raises(ValueError):
        parse_named_values(["Speed"])


# Lanelet map --------------------------------------------------------------

def test_lanelet_map_merges_points_and_uses_unique_ids(tmp_path):
    lmap = LaneletMap()
    left = [lmap.add_point(x, 1.0) for x in (0, 5, 10)]
    right = [lmap.add_point(x, -1.0) for x in (0, 5, 10)]
    next_left = [lmap.add_point(x, 1.0) for x in (10, 15)]
    next_right = [lmap.add_point(x, -1.0) for x in (10, 15)]
    first = lmap.add_lanelet(lmap.add_way(left, type="road_border"), lmap.add_way(right, type="road_border"),
                             **lanelet_tags(20))
    second = lmap.add_lanelet(lmap.add_way(next_left), lmap.add_way(next_right), **lanelet_tags(20))
    lmap.merge(left[-1], next_left[0])
    lmap.merge(right[-1], next_right[0])
    degenerate = lmap.add_lanelet(lmap.add_way([left[0], left[0]]), lmap.add_way(right), **lanelet_tags(20))
    stats = lmap.write(tmp_path / "map.osm")
    nodes, ways, relations = _osm(tmp_path / "map.osm")
    assert stats["dropped_degenerate_lanelets"] == 1 and str(degenerate) not in relations
    assert len(nodes) == 8
    assert ways[relations[str(first)][0]["left"]][-1] == ways[relations[str(second)][0]["left"]][0]
    assert all(ref in nodes for refs in ways.values() for ref in refs)
    assert not set(nodes) & set(ways) and not set(ways) & set(relations)
    assert relations[str(first)][1]["speed_limit"] == "20"
    assert relations[str(first)][1]["subtype"] == "road"


def test_chunk_bounds_cover_stations_with_forced_edges():
    chunks = chunk_bounds(101, 100.0, 20.0, forced=[33])
    assert chunks[0][0] == 0 and chunks[-1][1] == 100
    assert all(a[1] == b[0] for a, b in zip(chunks, chunks[1:]))
    assert any(first == 33 for first, _ in chunks)
    assert all(last - first <= 20 for first, last in chunks)
    assert chunk_bounds(2, 0.5, 20.0) == [(0, 1)]


# Corridor -----------------------------------------------------------------

def test_open_corridor_is_a_connected_chain(tmp_path):
    lmap = LaneletMap()
    corridor = build_corridor(lmap, StraightRoad(), 1, path_profile, "path", 1.0, 20.0, lambda s: 15.0)
    assert len(corridor.lanelets) == 5 and not corridor.closed
    assert corridor.chunk_starts == pytest.approx([0, 20, 40, 60, 80])
    for first, second in zip(corridor.lanelets, corridor.lanelets[1:]):
        assert lmap.bounds(first)[0][-1] == lmap.bounds(second)[0][0]
    left, right = lmap.bounds(corridor.lanelets[0])
    assert left[0][1] == pytest.approx(1.5) and right[0][1] == pytest.approx(-1.5)


def test_closed_corridor_shares_its_first_points():
    lmap = LaneletMap()
    corridor = build_corridor(lmap, CircleRoad(), 1, path_profile, "path", 1.0, 20.0, lambda s: 15.0)
    assert corridor.closed and not corridor.warnings
    first_left = lmap.bounds(corridor.lanelets[0])[0][0]
    last_left = lmap.bounds(corridor.lanelets[-1])[0][-1]
    assert first_left == last_left


def test_route_revisiting_itself_is_reported():
    lmap = LaneletMap()
    corridor = build_corridor(lmap, CircleRoad(laps=2, closed=False), 1, path_profile, "path",
                              1.0, 20.0, lambda s: 15.0)
    assert any("overlap" in warning for warning in corridor.warnings)
    circle = CircleRoad()
    single = [(s, point.x, point.y, point.heading)
              for s in range(0, 120, 2) for point in [circle.route_point(1, s)]]
    assert self_overlap(single, circle.length, True, 40.0) is None


def test_pylon_profile_matches_iso_generator():
    gates = [(s, lateral, width) for first, last, lateral, width in ISO_SECTIONS for s in (first, last)]
    profile = PylonProfile(gates)
    for s in (100.0, 499.0, 507.0, 530.0, 557.0, 582.0, 610.0, 690.0):
        lateral, _, width = course_point(s, ISO_SECTIONS)
        assert profile.centre(s) == pytest.approx((lateral, width))
    left, right = profile(557.0, None)
    assert (left, right) == pytest.approx((3.5 + 2.314 / 2.0, 3.5 - 2.314 / 2.0))


def test_cone_profile_interpolates_and_wraps():
    cones = [(0.0, 2.0, 1), (10.0, 3.0, 1), (0.0, -2.0, -1), (10.0, -2.0, -1)]
    profile = ConeProfile(cones, 20.0, True, 0.1)
    assert profile(5.0, None) == pytest.approx((2.4, -1.9))
    assert profile(15.0, None) == pytest.approx((2.4, -1.9))
    with pytest.raises(ValueError):
        ConeProfile(cones[:1] + cones[2:], 20.0, False, 0.0)


# Scenario -----------------------------------------------------------------

def test_goal_station_open_and_loop():
    assert goal_station(700.0, 0.0, False, 10.0, 20.0) == pytest.approx(690.0)
    assert goal_station(729.0, 719.3, True, 10.0, 20.0) == pytest.approx(689.3)
    assert goal_station(100.0, 5.0, True, 10.0, 20.0) == pytest.approx(75.0)
    with pytest.raises(ValueError):
        goal_station(20.0, 15.0, False, 10.0, 20.0)


def test_waypoints_follow_driving_order_and_skip_unpinned_paths():
    paths = [(1, 0.0, 100.0), (2, 0.0, 20.0), (3, 0.0, 200.0), (4, 0.0, 100.0)]
    pinned = {1, 3, 4}
    assert waypoint_stations(paths, pinned, 0.0, 410.0, 420.0, False) == pytest.approx([50.0, 220.0, 370.0])
    # Loop starting in path 3: waypoints after the start come first, then wrap.
    stations = waypoint_stations(paths, pinned, 300.0, 270.0, 420.0, True)
    assert stations == pytest.approx([370.0, 50.0, 220.0])


def test_route_pose_extrapolates_open_route_ends():
    road = StraightRoad(50.0)
    assert route_pose(road, 1, -2.0, 1.0).x == pytest.approx(-2.0)
    assert route_pose(road, 1, 53.0).x == pytest.approx(53.0)
    assert route_pose(CircleRoad(), 1, -1.0).y == pytest.approx(route_pose(CircleRoad(), 1, CircleRoad().length - 1.0).y)


def test_formula_student_cone_colours_define_sides():
    assert cone_side("TrafficCone_Small_Blue", -2.0) == 1
    assert cone_side("TrafficCone_Small_Yellow", 2.0) == -1
    assert cone_side("TrafficCone_Large_Orange", 2.0) == 0
    assert cone_side("TrafficCone", 1.6) == 1 and cone_side("TrafficCone", 0.2) == 0
    assert classify_object("TrafficCone_Small_Blue", 0.0) == "cone"
    assert classify_object("4_StreetFurniture/GuidePost", 0.0) == "static"
    assert classify_object("1_Vehicles/Audi_TT_2015", 0.0) == "dynamic"


def test_vehicle_geometry_matches_demo_car_description():
    info = InfoFile.parse(
        "Wheel.fl.pos = 3.40 0.789 0.29\nWheel.fr.pos = 3.40 -0.789 0.29\n"
        "Wheel.rl.pos = 0.863 0.772 0.29\nWheel.rr.pos = 0.863 -0.772 0.29\n"
        "Vehicle.OuterSkin = 0.0 0.9 0.2 4.28 -0.9 1.49\nTire.0 = Examples/RT_195_65R15\n")
    geometry = vehicle_geometry(info)
    assert geometry["wheel_base"] == pytest.approx(2.537)
    assert geometry["wheel_tread"] == pytest.approx(1.578)
    assert geometry["front_overhang"] == pytest.approx(0.88)
    assert geometry["rear_overhang"] == pytest.approx(0.863)
    assert geometry["vehicle_height"] == pytest.approx(1.49)
    assert geometry["wheel_radius"] == pytest.approx(0.317, abs=0.002)
    assert vehicle_geometry(InfoFile.parse("Body.mass = 1\n")) is None


# Network helpers, profiles and CLI ----------------------------------------

def test_turn_directions_and_border_types():
    assert turn_direction(0.0, 0.1) == "straight"
    assert turn_direction(0.0, math.pi / 2) == "left"
    assert turn_direction(math.pi, math.pi / 2) == "right"
    lanes = [{"direction": 1}, {"direction": 1}, {"direction": -1}, {"direction": 0}]
    assert _border_tags(lanes, 0) == {"type": "road_border"}
    assert _border_tags(lanes, 1) == {"type": "line_thin", "subtype": "dashed"}
    assert _border_tags(lanes, 2) == {"type": "line_thin", "subtype": "solid"}
    assert _border_tags(lanes, 3) == {"type": "road_border"}


def test_profiles_protect_hand_written_files(tmp_path):
    assert profiles.default_name("/x/FS_autonomous_Sprint_LTS") == "fs_autonomous_sprint_lts"
    assert profiles.default_name("/x/2024 run") == "2024_run"
    available = {"fsai_sprint": {}, "iso_lanechange_demo": {}}
    assert profiles.select_base_profile("Examples/DemoCar", None, available) == "iso_lanechange_demo"
    with pytest.raises(ValueError, match="--base-profile"):
        profiles.select_base_profile("Unknown", None, available)
    hand = tmp_path / "fsai_sprint.yaml"
    hand.write_text("vehicle_model: a\n")
    with pytest.raises(cli.ConversionError, match="not generated"):
        cli.check_target(hand, True, "Profile")
    generated = tmp_path / "new.yaml"
    generated.write_text(f"generated_by: {profiles.GENERATOR}\n")
    with pytest.raises(cli.ConversionError, match="--force"):
        cli.check_target(generated, False, "Profile")
    cli.check_target(generated, True, "Profile")
    differences = profiles.compare_vehicle({"wheel_base": 1.53}, {"wheel_base": 1.6, "max_steer_angle": 0.7})
    assert differences == [{"parameter": "wheel_base", "carmaker": 1.53, "autoware": 1.6, "difference": 0.07}]


def test_speed_lookup_uses_active_limit_and_cap():
    speeds = [(50 / 3.6, 0.0), (30 / 3.6, 0.0), (30 / 3.6, 177.8)]
    assert cli.speed_lookup(speeds, 10.0, 40.0) == pytest.approx(30.0)
    assert cli.speed_lookup(speeds, 10.0, 20.0) == pytest.approx(20.0)
    assert cli.speed_lookup([], 0.0, 20.0) == pytest.approx(20.0)


def test_cli_rejects_bad_numbers():
    with pytest.raises(SystemExit):
        cli.parse_args(["x", "--spacing", "0"])
    with pytest.raises(SystemExit):
        cli.parse_args(["x", "--name", "Bad-Name"])


# Validation ---------------------------------------------------------------

def test_signed_distance_is_positive_on_the_left():
    line = [(0.0, 0.0), (10.0, 0.0)]
    assert signed_distance(line, (5.0, 2.0)) == pytest.approx(2.0)
    assert signed_distance(line, (5.0, -3.0)) == pytest.approx(-3.0)


def test_opposing_lanes_share_a_centre_line_like_autoware(tmp_path):
    pytest.importorskip("lanelet2")
    lmap = LaneletMap()
    centre = [lmap.add_point(x, 0.0) for x in range(0, 31, 10)]
    south = [lmap.add_point(x, -3.0) for x in range(0, 31, 10)]
    north = [lmap.add_point(x, 3.0) for x in range(0, 31, 10)]
    shared = lmap.add_way(centre, type="line_thin", subtype="solid")
    lmap.add_lanelet(shared, lmap.add_way(south, type="road_border"), **lanelet_tags(30))
    # Driving towards -x, the shared centre line is on the left of the north lane.
    lmap.add_lanelet(shared, lmap.add_way(north, type="road_border"), **lanelet_tags(30))
    lmap.write(tmp_path / "map.osm")
    eastbound = validate_map(tmp_path / "map.osm", Pose(2.0, -1.5, 0.0, 0.0), Pose(28.0, -1.5, 0.0, 0.0))
    westbound = validate_map(tmp_path / "map.osm", Pose(28.0, 1.5, 0.0, math.pi), Pose(2.0, 1.5, 0.0, math.pi))
    assert eastbound["start_lanelet"] is not None and westbound["start_lanelet"] is not None
    assert eastbound["start_lanelet"] != westbound["start_lanelet"]
    wrong_way = validate_map(tmp_path / "map.osm", Pose(2.0, 1.5, 0.0, 0.0), Pose(28.0, 1.5, 0.0, 0.0))
    assert wrong_way["start_lanelet"] is None and not wrong_way["passed"]
