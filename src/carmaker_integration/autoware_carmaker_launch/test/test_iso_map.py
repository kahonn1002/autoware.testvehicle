"""ISO double-lane-change map tests against the real DemoCar TestRun."""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from autoware_carmaker_launch.generate_iso_map import (
    course_point,
    parse_iso_course,
    write_map,
)

TESTRUN = Path(
    "/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware"
    "/Data/TestRun/LaneChange_ISO_Camera_RSI"
)
EXPECTED_SECTIONS = [
    [500.0, 515.0, 0.0, 2.142],
    [545.0, 570.0, 3.5, 2.314],
    [595.0, 625.0, 0.0, 2.486],
]


def _read_original():
    if not TESTRUN.is_file():
        pytest.skip("Requires the original CarMaker ISO TestRun")
    before = TESTRUN.stat().st_mtime_ns
    text = TESTRUN.read_text(encoding="utf-8")
    assert TESTRUN.stat().st_mtime_ns == before
    return text


def test_parse_real_testrun_length_and_sections():
    length, sections, gates = parse_iso_course(_read_original())
    assert length == pytest.approx(700.0)
    assert len(sections) == 3
    for got, want in zip(sections, EXPECTED_SECTIONS):
        assert got[0] == pytest.approx(want[0])
        assert got[1] == pytest.approx(want[1])
        assert got[2] == pytest.approx(want[2])
        assert got[3] == pytest.approx(want[3])
    assert len(gates) == 11


def test_course_point_reaches_section_centers():
    for start, end, lateral, width in EXPECTED_SECTIONS:
        y, yaw, w = course_point((start + end) / 2.0, EXPECTED_SECTIONS)
        assert y == pytest.approx(lateral)
        assert yaw == pytest.approx(0.0)
        assert w == pytest.approx(width)


def test_transitions_are_smooth_and_forward():
    for (end, _, lat0, _), (start, _, lat1, _) in zip(
        EXPECTED_SECTIONS, EXPECTED_SECTIONS[1:]
    ):
        prev_yaw = 0.0
        prev_y = lat0
        n = 60
        for k in range(n + 1):
            s = end + (start - end) * k / n
            y, yaw, _ = course_point(s, EXPECTED_SECTIONS)
            assert math.isfinite(yaw) and abs(yaw) < 0.35
            # Yaw continuity: no step jumps between dense samples.
            assert abs(yaw - prev_yaw) < 0.05
            # Monotone lateral progress toward the next gate, no reversal.
            if lat1 >= lat0:
                assert y >= prev_y - 1e-9
            else:
                assert y <= prev_y + 1e-9
            prev_yaw, prev_y = yaw, y
        # Blend meets both gate centers with zero end heading.
        assert course_point(end, EXPECTED_SECTIONS)[1] == pytest.approx(0.0)
        assert course_point(start, EXPECTED_SECTIONS)[1] == pytest.approx(0.0)


def test_generated_osm_refs_valid_and_boundaries_shared(tmp_path):
    meta = write_map(tmp_path, TESTRUN)
    assert meta["speed_limit_kmh"] == pytest.approx(10.0)
    assert meta["goal"]["x"] == pytest.approx(650.0)
    root = ET.parse(tmp_path / "lanelet2_map.osm").getroot()
    nodes = {n.get("id") for n in root.findall("node")}
    assert nodes
    ways = {}
    for w in root.findall("way"):
        refs = [nd.get("ref") for nd in w.findall("nd")]
        assert len(refs) > 1
        assert all(r in nodes for r in refs)
        ways[w.get("id")] = refs
    # Adjacent chunks share exactly one boundary node per side.
    left_ids = sorted(int(k) for k in ways if int(k) % 2 == 0)
    right_ids = sorted(int(k) for k in ways if int(k) % 2 == 1)
    assert len(left_ids) == len(right_ids) and len(left_ids) > 1
    for a, b in zip(left_ids, left_ids[1:]):
        assert ways[str(a)][-1] == ways[str(b)][0]
    for a, b in zip(right_ids, right_ids[1:]):
        assert ways[str(a)][-1] == ways[str(b)][0]
    # Relations reference valid ways and carry the 10 km/h limit.
    relations = root.findall("relation")
    assert relations
    for r in relations:
        roles = {m.get("role"): m.get("ref") for m in r.findall("member")}
        assert roles["left"] in ways and roles["right"] in ways
        tags = {t.get("k"): t.get("v") for t in r.findall("tag")}
        assert tags["subtype"] == "road"
        assert float(tags["speed_limit"]) == pytest.approx(10.0)


def test_generated_extent_covers_700m(tmp_path):
    write_map(tmp_path, TESTRUN)
    root = ET.parse(tmp_path / "lanelet2_map.osm").getroot()
    xs = [float(n.find("./tag[@k='local_x']").get("v")) for n in root.findall("node")]
    assert min(xs) == pytest.approx(0.0, abs=1.0)
    assert max(xs) == pytest.approx(700.0, abs=1.0)
    # Strictly forward path: centerline x increases, no wrong-direction loop.
    for s in [0.0, 250.0, 507.0, 530.0, 557.0, 582.0, 610.0, 660.0, 700.0]:
        y, _, _ = course_point(s, EXPECTED_SECTIONS)
        assert -2.0 < y < 5.5
    info = json.loads((tmp_path / "course_metadata.json").read_text())
    assert info["length_m"] == pytest.approx(700.0)


def _with(key, val, text):
    import re

    return re.sub(r"^" + re.escape(key) + r"\s*=\s*[^\n]+",
                  f"{key} = {val}", text, flags=re.M)


@pytest.mark.parametrize("key,val", [
    ("Road.nLinks", "2"),
    ("Road.nJunctions", "1"),
    ("Road.Link.0.Seg.0.Type", "Curve"),
    ("Road.Link.0.Node0", "1 0 0"),
])
def test_parse_rejects_unsupported_roads(key, val):
    with pytest.raises(ValueError):
        parse_iso_course(_with(key, val, _read_original()))


def test_parse_rejects_bad_spacing_and_speed(tmp_path):
    for bad in (0.0, -0.5, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            write_map(tmp_path, TESTRUN, spacing=bad)
    with pytest.raises(ValueError):
        write_map(tmp_path, TESTRUN, speed_kmh=0.0)
