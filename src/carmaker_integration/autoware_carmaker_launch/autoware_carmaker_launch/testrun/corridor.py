"""Lanelet corridor along one CarMaker route, bounded by cones, pylons or path width.

Formula Student tracks and ISO pylon courses are defined by objects rather than
by road lanes, so the drivable area follows the ego route between them.
"""

import bisect
import math
from dataclasses import dataclass, field

from .lanelet_map import chunk_bounds, lanelet_tags
from .road_api import MARKER_PYLON

MAX_OPEN_WIDTH = 6.0


@dataclass
class Corridor:
    lanelets: list
    chunk_starts: list
    length: float
    closed: bool
    boundary: str
    warnings: list = field(default_factory=list)


def path_profile(s, point):
    return point.width_left, -point.width_right


class ConeProfile:
    """Piecewise-linear left/right boundary through cone rows, inset by a margin."""

    def __init__(self, cones, length, closed, margin):
        self.length, self.closed = length, closed
        self.sides = []
        for side in (1, -1):
            rows = sorted((s, t - side * margin) for s, t, row in cones if row == side)
            if len(rows) < 2:
                raise ValueError("Cone boundary needs at least two cones on each side")
            self.sides.append(rows)

    def _value(self, rows, s):
        stations = [item[0] for item in rows]
        if self.closed:
            s %= self.length
            index = bisect.bisect_right(stations, s)
            before = rows[index - 1] if index else (rows[-1][0] - self.length, rows[-1][1])
            after = rows[index] if index < len(rows) else (rows[0][0] + self.length, rows[0][1])
        else:
            index = bisect.bisect_right(stations, s)
            if index == 0:
                return rows[0][1]
            if index == len(rows):
                return rows[-1][1]
            before, after = rows[index - 1], rows[index]
        span = after[0] - before[0]
        ratio = 0.0 if span <= 1e-9 else (s - before[0]) / span
        return before[1] + (after[1] - before[1]) * ratio

    def __call__(self, s, point):
        return self._value(self.sides[0], s), self._value(self.sides[1], s)


def pylon_sections(gates):
    """Merge consecutive (s, t, width) gates with equal offset and width."""
    sections = []
    for station, lateral, width in sorted(gates):
        if sections and math.isclose(lateral, sections[-1][2], abs_tol=0.05) \
                and math.isclose(width, sections[-1][3], abs_tol=0.01):
            sections[-1][1] = station
        else:
            sections.append([station, station, lateral, width])
    return sections


class PylonProfile:
    """ISO 3888-style gate corridor with smooth (zero end curvature) transitions."""

    def __init__(self, gates):
        if len(gates) < 2:
            raise ValueError("Pylon boundary needs at least two gates")
        self.sections = pylon_sections(gates)

    def centre(self, s):
        sections = self.sections
        for i, (start, end, lateral, width) in enumerate(sections):
            if start <= s <= end:
                return lateral, width
            if i + 1 < len(sections) and end < s < sections[i + 1][0]:
                next_start, _, next_lateral, next_width = sections[i + 1]
                t = (s - end) / (next_start - end)
                blend = t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
                # Open pavement between gates accommodates the body sweep.
                sweep = 2.0 * math.sin(math.pi * t) ** 2
                return (lateral + (next_lateral - lateral) * blend,
                        width * (1.0 - blend) + next_width * blend + sweep)
        if s < sections[0][0]:
            return sections[0][2], min(MAX_OPEN_WIDTH, sections[0][3] + (sections[0][0] - s) / 10.0)
        return sections[-1][2], min(MAX_OPEN_WIDTH, sections[-1][3] + (s - sections[-1][1]) / 10.0)

    def __call__(self, s, point):
        lateral, width = self.centre(s)
        return lateral + width / 2.0, lateral - width / 2.0


def route_gates(road, route):
    """Pylon gates on a route as (s, t, width), t relative to the ego driving path."""
    gates = []
    for marker in road.route_markers(route, [MARKER_PYLON]):
        projected = road.route_project(route, marker.x, marker.y, marker.z)
        gates.append((projected.s, projected.t, marker.width))
    return gates


def gate_pylons(road, route, gates):
    """Global (x, y, z) of the two pylons of every gate."""
    pylons = []
    for s, t, width in gates:
        for side in (1.0, -1.0):
            point = road.route_point(route, s, t + side * width / 2.0)
            pylons.append((point.x, point.y, point.z))
    return pylons


def self_overlap(centres, length, closed, separation):
    """First (earlier s, later s) where the route revisits a place in the same direction."""
    grid = {}
    for s, x, y, heading in centres:
        cell = (round(x / 2.0), round(y / 2.0))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other_s, ox, oy, other_heading in grid.get((cell[0] + dx, cell[1] + dy), ()):
                    gap = s - other_s
                    if closed:
                        gap = min(gap, length - gap)
                    turn = abs((heading - other_heading + math.pi) % (2.0 * math.pi) - math.pi)
                    if gap > separation and math.hypot(x - ox, y - oy) < 1.0 and turn < math.pi / 6:
                        return other_s, s
        grid.setdefault(cell, []).append((s, x, y, heading))
    return None


def build_corridor(lmap, road, route, profile, boundary, spacing, chunk_length, speed_at,
                   extra_tags=None):
    """Add one lanelet chain following `route` and return it in driving order."""
    length = road.route_length(route)
    if not math.isfinite(length) or length <= spacing:
        raise ValueError(f"Route {route} is too short ({length:.2f} m)")
    loop = road.route_loop(route)
    count = max(2, math.ceil(length / spacing)) + 1
    stations = [length * i / (count - 1) for i in range(count)]
    loop_index = None
    warnings = []
    if loop is not None:
        loop_start = loop[0] % length if loop[1] < length - 1e-3 else 0.0
        loop_index = min(range(count - 1), key=lambda i: abs(stations[i] - loop_start))

    left_points, right_points, centres = [], [], []
    for index, s in enumerate(stations):
        if loop_index is not None and index == count - 1:
            left_points.append(left_points[loop_index])
            right_points.append(right_points[loop_index])
            continue
        point = road.route_point(route, s)
        centres.append((s, point.x, point.y, point.heading))
        t_left, t_right = profile(s, point)
        if t_left - t_right < 0.5:
            warnings.append(f"Corridor narrower than 0.5 m at s={s:.1f} m")
            middle = (t_left + t_right) / 2.0
            t_left, t_right = middle + 0.25, middle - 0.25
        for t, target in ((t_left, left_points), (t_right, right_points)):
            boundary_point = road.route_point(route, s, t)
            target.append(lmap.add_point(boundary_point.x, boundary_point.y, boundary_point.z))

    overlap = self_overlap(centres, length, loop is not None, 2.0 * chunk_length)
    if overlap:
        warnings.append(
            f"Route drives over s={overlap[0]:.0f} m again at s={overlap[1]:.0f} m in the same "
            "direction; the lanelets overlap there and Autoware may match the wrong lap")
    line = ({"type": "road_border"} if boundary in ("cones", "pylons")
            else {"type": "line_thin", "subtype": "solid"})
    lanelets, starts = [], []
    forced = [loop_index] if loop_index else []
    for first, last in chunk_bounds(count, length, chunk_length, forced):
        left = lmap.add_way(left_points[first:last + 1], **line)
        right = lmap.add_way(right_points[first:last + 1], **line)
        tags = lanelet_tags(speed_at(stations[first]), **(extra_tags or {}))
        lanelets.append(lmap.add_lanelet(left, right, **tags))
        starts.append(stations[first])
    return Corridor(lanelets, starts, length, loop is not None, boundary, warnings)
