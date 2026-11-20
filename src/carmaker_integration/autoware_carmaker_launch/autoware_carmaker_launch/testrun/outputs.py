"""Point cloud, projector and YAML outputs of a converted TestRun."""

import hashlib
import math

import yaml

CONE_HEIGHT = 0.5
OBJECT_HEIGHT = 1.0


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _resample(points, count):
    if len(points) == count:
        return points
    return [points[round(i * (len(points) - 1) / (count - 1))] for i in range(count)]


def surface_points(lmap, lanelets, spacing=1.0):
    """Road surface samples across every lanelet (flat intensity 0)."""
    points = []
    for lanelet in lanelets:
        if lanelet not in lmap.lanelets:
            continue
        left, right = lmap.bounds(lanelet)
        count = max(len(left), len(right))
        for (lx, ly, lz), (rx, ry, rz) in zip(_resample(left, count), _resample(right, count)):
            steps = max(1, math.ceil(math.hypot(lx - rx, ly - ry) / spacing))
            for i in range(steps + 1):
                ratio = i / steps
                points.append((rx + (lx - rx) * ratio, ry + (ly - ry) * ratio,
                               rz + (lz - rz) * ratio, 0.0))
    return points


def column_points(x, y, z, height):
    """Vertical point column (intensity 200) marking a static object."""
    return [(x, y, z + step * 0.05, 200.0) for step in range(int(height / 0.05) + 1)]


def object_points(objects):
    """Columns at static traffic objects (cones, posts); moving traffic is omitted."""
    points = []
    for item in objects:
        if item.pose is not None and item.kind != "dynamic":
            height = CONE_HEIGHT if item.kind == "cone" else OBJECT_HEIGHT
            points.extend(column_points(item.pose.x, item.pose.y, item.pose.z, height))
    return points


def thin(points, voxel=0.25):
    """Keep one point per voxel so large networks stay small."""
    kept = {}
    for point in points:
        key = tuple(math.floor(value / voxel) for value in point[:3])
        kept.setdefault(key, point)
    return list(kept.values())


def write_pcd(path, points):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        stream.write("FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for point in points:
            stream.write(" ".join(f"{value:.5f}" for value in point) + "\n")


def write_projector(path):
    path.write_text("projector_type: Local\n", encoding="utf-8")


def write_yaml(path, data, header=None):
    text = yaml.safe_dump(data, sort_keys=False, default_flow_style=False)
    path.write_text((header or "") + text, encoding="utf-8")
