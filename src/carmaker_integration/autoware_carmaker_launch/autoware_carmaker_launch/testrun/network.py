"""Lane-level Lanelet2 network from IPGRoad links, lane sections and paths.

Lanes on links are built from CarMaker's lane borders sampled at shared
stations, so neighbouring lanes share boundary line strings and lane changes
are possible. Junction lanes and connector paths are sampled along their path
and joined to the lanes they connect through IPGRoad's path topology.
"""

import math
from dataclasses import dataclass, field

from .lanelet_map import chunk_bounds, lanelet_tags

# Road, Bus, HOV, Emergency, OnRamp, OffRamp and SlipLane lanes (tRoadLaneType).
DRIVABLE_LANE_TYPES = {0, 14, 15, 16, 17, 18, 19}
MIN_LANE_WIDTH = 0.5
FALLBACK_HALF_WIDTH = 1.75
MAX_CONNECT_GAP = 1.0
STRAIGHT_LIMIT = math.radians(30.0)


@dataclass
class PathLanelets:
    path: int
    lanelets: list
    chunk_offsets: list
    start: tuple
    end: tuple
    length: float
    exact: bool


@dataclass
class Network:
    paths: dict
    warnings: list = field(default_factory=list)

    def lanelets(self):
        return [lanelet for item in self.paths.values() for lanelet in item.lanelets]


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def is_lane_change(info):
    """Connector paths on a link move between neighbouring lanes mid-link.

    Lanelet2 expresses that through shared (dashed) lane boundaries, so these
    paths get no lanelet of their own.
    """
    return info.kind == "ConPath" and info.root_kind == "Link"


def turn_direction(start_heading, end_heading):
    change = _wrap(end_heading - start_heading)
    if abs(change) < STRAIGHT_LIMIT:
        return "straight"
    return "left" if change > 0 else "right"


def _border_tags(lanes, border):
    """Line type between lane `border - 1` (right) and lane `border` (left)."""
    right = lanes[border - 1]["direction"] if border > 0 else 0
    left = lanes[border]["direction"] if border < len(lanes) else 0
    if right and left:
        return {"type": "line_thin", "subtype": "dashed" if right == left else "solid"}
    return {"type": "road_border"}


def _build_section(lmap, road, network, link, section, spacing, chunk_length, speed_at):
    start, section_length = road.section_range(section)
    if section_length < 1e-3:
        return
    ordered = list(reversed(road.lanes(section, right=True))) + road.lanes(section, right=False)
    count = max(2, math.ceil(section_length / spacing)) + 1
    stations = [section_length * i / (count - 1) for i in range(count)]
    borders = [road.section_borders(section, s) for s in stations]
    if any(len(values) != len(ordered) + 1 for values in borders):
        network.warnings.append(f"Link {link} section {section}: lane borders do not match lanes")
        return

    lanes = []
    for index, lane in enumerate(ordered):
        path = road.lane_path(lane)
        widest = max(values[index + 1] - values[index] for values in borders)
        drivable = (road.lane_type(lane) in DRIVABLE_LANE_TYPES and path > 0
                    and widest >= MIN_LANE_WIDTH)
        direction = road.path_info(path).in_link_dir if drivable else 0
        lanes.append({"lane": lane, "path": path, "direction": direction})
    needed = sorted({border for index, lane in enumerate(lanes) if lane["direction"]
                     for border in (index, index + 1)})
    if not needed:
        return
    points = {border: [lmap.add_point(*road.link_point(link, start + s, values[border]))
                       for s, values in zip(stations, borders)] for border in needed}
    chunks = chunk_bounds(count, section_length, chunk_length)
    ways = {(chunk, border): lmap.add_way(points[border][first:last + 1],
                                          **_border_tags(lanes, border))
            for chunk, (first, last) in enumerate(chunks) for border in needed}

    for index, lane in enumerate(lanes):
        direction, path = lane["direction"], lane["path"]
        if not direction:
            continue
        forward = direction > 0
        left, right = (index + 1, index) if forward else (index, index + 1)
        order = range(len(chunks)) if forward else reversed(range(len(chunks)))
        lanelets, offsets = [], []
        for chunk in order:
            first, last = chunks[chunk]
            offset = stations[first] if forward else section_length - stations[last]
            tags = lanelet_tags(speed_at(path, offset), cm_path=path, cm_link=link)
            lanelets.append(lmap.add_lanelet(ways[(chunk, left)], ways[(chunk, right)], **tags))
            offsets.append(offset)
        head, tail = (0, count - 1) if forward else (count - 1, 0)
        network.paths[path] = PathLanelets(
            path, lanelets, offsets, (points[left][head], points[right][head]),
            (points[left][tail], points[right][tail]), section_length, True)


def _build_sampled_path(lmap, road, network, info, spacing, chunk_length, speed_at):
    length = info.length
    if length < 1e-3:
        return
    count = max(2, math.ceil(length / spacing)) + 1
    samples = [road.path_sample(info.path_id, length * i / (count - 1)) for i in range(count)]
    left_points, right_points = [], []
    for sample in samples:
        left, right = sample.left, sample.right
        if math.dist(left[:2], right[:2]) < MIN_LANE_WIDTH:
            nx, ny = -math.sin(sample.heading), math.cos(sample.heading)
            left = (sample.x + FALLBACK_HALF_WIDTH * nx, sample.y + FALLBACK_HALF_WIDTH * ny, sample.z)
            right = (sample.x - FALLBACK_HALF_WIDTH * nx, sample.y - FALLBACK_HALF_WIDTH * ny, sample.z)
        left_points.append(lmap.add_point(*left))
        right_points.append(lmap.add_point(*right))
    in_junction = info.root_kind == "Junction" or info.kind == "ConPath"
    turn = turn_direction(samples[0].heading, samples[-1].heading) if in_junction else None
    line = {"type": "virtual"} if in_junction else {"type": "line_thin", "subtype": "solid"}
    lanelets, offsets = [], []
    for first, last in chunk_bounds(count, length, chunk_length):
        offset = length * first / (count - 1)
        tags = lanelet_tags(speed_at(info.path_id, offset), turn_direction=turn,
                            cm_path=info.path_id)
        lanelets.append(lmap.add_lanelet(lmap.add_way(left_points[first:last + 1], **line),
                                         lmap.add_way(right_points[first:last + 1], **line),
                                         **tags))
        offsets.append(offset)
    network.paths[info.path_id] = PathLanelets(
        info.path_id, lanelets, offsets, (left_points[0], right_points[0]),
        (left_points[-1], right_points[-1]), length, False)


def _connect(lmap, road, network):
    """Join path ends to successor starts so Lanelet2 routing sees the topology."""
    mid_path_splits = 0
    for path, item in network.paths.items():
        for successor, offset in road.out_paths(path):
            target = network.paths.get(successor)
            if target is None:
                continue
            if offset < item.length - MAX_CONNECT_GAP:
                mid_path_splits += 1
                continue
            gap = max(lmap.distance(item.end[0], target.start[0]),
                      lmap.distance(item.end[1], target.start[1]))
            if gap > MAX_CONNECT_GAP:
                network.warnings.append(
                    f"Path {path} -> {successor} not connected: {gap:.2f} m boundary gap")
                continue
            for own, other in zip(item.end, target.start):
                if target.exact and not item.exact:
                    lmap.merge(other, own)
                else:
                    lmap.merge(own, other)
    if mid_path_splits:
        network.warnings.append(
            f"{mid_path_splits} path branches start mid-lane and were not connected")


def build_network(lmap, road, spacing, chunk_length, speed_at, include_paths=()):
    """Convert every drivable lane, junction lane and connector into lanelets."""
    network = Network({})
    for link in road.links():
        for section in road.lane_sections(link):
            _build_section(lmap, road, network, link, section, spacing, chunk_length, speed_at)
    include_paths = set(include_paths)
    for path in road.paths():
        if path in network.paths:
            continue
        info = road.path_info(path)
        if is_lane_change(info):
            continue
        junction_path = info.kind in ("LanePath", "ConPath") and info.root_kind != "Link"
        if junction_path or path in include_paths:
            _build_sampled_path(lmap, road, network, info, spacing, chunk_length, speed_at)
    _connect(lmap, road, network)
    return network
