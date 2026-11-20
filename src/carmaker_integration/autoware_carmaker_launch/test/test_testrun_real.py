"""Convert the real CarMaker 15 TestRuns and check them like Autoware loads them.

Needs CarMaker's IPGRoad wheel and license plus the Lanelet2 Python bindings;
skipped otherwise. Outputs go to a temporary workspace.
"""

import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from autoware_carmaker_launch.generate_iso_map import course_point
from autoware_carmaker_launch.testrun import cli
from autoware_carmaker_launch.testrun.road_api import find_carmaker_home, load_ipgroad

FSAI = Path("/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware")
ADAS = Path("/home/klee/project/Carmaker_proj/CarMaker_ROS2_ADAS_Autoware")
SOURCE = Path(__file__).resolve().parents[2]
ISO_SECTIONS = [[500.0, 515.0, 0.0, 2.142], [545.0, 570.0, 3.5, 2.314], [595.0, 625.0, 0.0, 2.486]]


@pytest.fixture(scope="module", autouse=True)
def carmaker():
    pytest.importorskip("lanelet2")
    if not FSAI.is_dir():
        pytest.skip("Requires the CarMaker 15 FSAI project")
    try:
        load_ipgroad(find_carmaker_home())
    except Exception as error:  # Missing installation or wheel.
        pytest.skip(f"IPGRoad API unavailable: {error}")


@pytest.fixture
def workspace(tmp_path):
    launch = tmp_path / "src" / "carmaker_integration" / "autoware_carmaker_launch"
    shutil.copytree(SOURCE / "autoware_carmaker_launch" / "config" / "profiles", launch / "config" / "profiles",
                    ignore=lambda folder, names: [name for name in names
                                                  if name not in ("fsai_sprint.yaml", "iso_lanechange_demo.yaml")])
    for vehicle in ("carmaker_fsai_vehicle_description", "carmaker_demo_vehicle_description"):
        target = tmp_path / "src" / "carmaker_integration" / vehicle / "config"
        target.mkdir(parents=True)
        shutil.copy(SOURCE / vehicle / "config" / "vehicle_info.param.yaml", target)
    shutil.copytree(SOURCE / "autoware_carmaker_interface" / "config",
                    tmp_path / "src" / "carmaker_integration" / "autoware_carmaker_interface" / "config")
    return tmp_path


def convert(workspace, testrun, project=FSAI, *extra):
    if not (project / "Data" / "TestRun" / testrun).is_file():
        pytest.skip(f"{testrun} not available")
    args = cli.parse_args([testrun, "--project", str(project), "--workspace", str(workspace), *extra])
    return cli.convert(args)["scenario"]


def lateral_bounds(osm, x):
    """Left/right y of a straight +x corridor at station x."""
    root = ET.parse(osm).getroot()
    nodes = {node.get("id"): tuple(float(node.find(f"tag[@k='{key}']").get("v")) for key in ("local_x", "local_y"))
             for node in root.iter("node")}
    ways = {way.get("id"): [nodes[nd.get("ref")] for nd in way.iter("nd")] for way in root.iter("way")}
    values = {}
    for relation in root.iter("relation"):
        for member in relation.iter("member"):
            for (x0, y0), (x1, y1) in zip(ways[member.get("ref")], ways[member.get("ref")][1:]):
                if min(x0, x1) <= x <= max(x0, x1) and x1 != x0:
                    values[member.get("role")] = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return values["left"], values["right"]


def test_iso_lane_change_becomes_pylon_corridor(workspace):
    scenario = convert(workspace, "LaneChange_ISO_Camera_RSI")
    assert scenario["conversion"]["mode"] == "corridor"
    assert scenario["conversion"]["boundary"] == "pylons"
    assert scenario["validation"]["passed"], scenario["validation"]
    osm = workspace / "maps" / "lanechange_iso_camera_rsi" / "lanelet2_map.osm"
    for x in (100.0, 507.0, 557.0, 610.0, 690.0):
        lateral, _, width = course_point(x, ISO_SECTIONS)
        assert lateral_bounds(osm, x) == pytest.approx((lateral + width / 2, lateral - width / 2), abs=0.01)
    assert scenario["autoware"]["goal"]["x"] == pytest.approx(690.0, abs=0.01)
    # The DemoCar interface moves CarMaker's Fr1 origin 0.863 m forward to base_link.
    assert scenario["autoware"]["initial_pose_fr1"]["x"] == pytest.approx(0.0, abs=0.01)
    assert scenario["autoware"]["initial_pose"]["x"] == pytest.approx(0.863, abs=0.01)


def test_sprint_loop_follows_the_cones(workspace):
    scenario = convert(workspace, "FS_autonomous_Sprint_LTS")
    conversion, validation = scenario["conversion"], scenario["validation"]
    assert (conversion["mode"], conversion["boundary"], conversion["route"]["closed"]) == ("corridor", "cones", True)
    assert validation["passed"] and validation["start_lanelet"] != validation["goal_lanelet"]
    # One lap less the gap between goal and start.
    assert validation["route_length_m"] == pytest.approx(validation["carmaker_route_m"], rel=0.05)
    assert scenario["traffic_objects"]["cones"] == 366
    assert not [warning for warning in scenario["warnings"] if "could not be placed" in warning]
    assert any(item["parameter"] == "wheel_base" for item in scenario["vehicle"]["differences"])


def test_skidpad_overlap_is_reported(workspace):
    scenario = convert(workspace, "FS_autonomous_SkidPad_LTS")
    assert scenario["conversion"]["boundary"] == "cones" and scenario["validation"]["passed"]
    assert any("overlap" in warning for warning in scenario["warnings"])


def test_junction_road_becomes_lane_network(workspace):
    scenario = convert(workspace, "test_road")
    assert scenario["conversion"]["mode"] == "network"
    assert scenario["validation"]["passed"] and scenario["autoware"]["waypoints"]
    assert scenario["validation"]["unmatched_waypoints"] == 0
    root = ET.parse(workspace / "maps" / "test_road" / "lanelet2_map.osm").getroot()
    turns = {tag.get("v") for tag in root.iter("tag") if tag.get("k") == "turn_direction"}
    assert turns == {"left", "right", "straight"} or turns == {"left", "right"}
    with pytest.raises(cli.ConversionError, match="--force"):
        convert(workspace, "test_road")
    assert convert(workspace, "test_road", FSAI, "--force")["validation"]["passed"]


def test_adas_network_follows_the_testrun_route(workspace):
    if not ADAS.is_dir():
        pytest.skip("Requires the CarMaker ADAS project")
    scenario = convert(workspace, "ADAS", ADAS, "--base-profile", "iso_lanechange_demo")
    validation = scenario["validation"]
    assert validation["passed"] and validation["unmatched_waypoints"] == 0
    assert validation["route_length_m"] == pytest.approx(validation["carmaker_route_m"], rel=0.05)
    assert not [warning for warning in scenario["warnings"] if "not connected" in warning
                or "without lanelets" in warning]


def test_unknown_vehicle_and_old_testrun_are_explained(workspace):
    if not ADAS.is_dir():
        pytest.skip("Requires the CarMaker ADAS project")
    with pytest.raises(ValueError, match="--base-profile"):
        convert(workspace, "ADAS", ADAS)
    with pytest.raises(cli.ConversionError, match="CarMaker 15"):
        convert(workspace, "PerformanceBenchmark", ADAS, "--base-profile", "iso_lanechange_demo")
