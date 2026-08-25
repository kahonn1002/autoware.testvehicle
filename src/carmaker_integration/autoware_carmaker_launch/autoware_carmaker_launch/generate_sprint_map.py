import argparse
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET


def _sample_line(start, end, spacing):
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(distance / spacing)))
    return [
        (
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
        )
        for index in range(1, count + 1)
    ]


def _sample_hermite(start, end, start_heading, end_heading, spacing):
    """Sample an IPGRoad PointList span with its endpoint tangents."""
    chord = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(chord / spacing)))
    start_tangent = (chord * math.cos(start_heading), chord * math.sin(start_heading))
    end_tangent = (chord * math.cos(end_heading), chord * math.sin(end_heading))
    samples = []
    for index in range(1, count + 1):
        t = index / count
        h00 = 2.0 * t**3 - 3.0 * t**2 + 1.0
        h10 = t**3 - 2.0 * t**2 + t
        h01 = -2.0 * t**3 + 3.0 * t**2
        h11 = t**3 - t**2
        x = (
            h00 * start[0]
            + h10 * start_tangent[0]
            + h01 * end[0]
            + h11 * end_tangent[0]
        )
        y = (
            h00 * start[1]
            + h10 * start_tangent[1]
            + h01 * end[1]
            + h11 * end_tangent[1]
        )
        dx = (
            (6.0 * t**2 - 6.0 * t) * start[0]
            + (3.0 * t**2 - 4.0 * t + 1.0) * start_tangent[0]
            + (-6.0 * t**2 + 6.0 * t) * end[0]
            + (3.0 * t**2 - 2.0 * t) * end_tangent[0]
        )
        dy = (
            (6.0 * t**2 - 6.0 * t) * start[1]
            + (3.0 * t**2 - 4.0 * t + 1.0) * start_tangent[1]
            + (-6.0 * t**2 + 6.0 * t) * end[1]
            + (3.0 * t**2 - 2.0 * t) * end_tangent[1]
        )
        samples.append((x, y, math.atan2(dy, dx)))
    return samples


def parse_rd5_centerline(path: Path, spacing: float, lane_center_offset: float):
    text = path.read_text(encoding="utf-8")
    node_match = re.search(r"Link\.0\.Node0 = ([^\n]+)", text)
    if not node_match:
        raise ValueError("Link.0.Node0 was not found in the IPGRoad file")
    node = [float(value) for value in node_match.group(1).split()[:3]]
    x, y, heading = node[0], node[1], math.radians(node[2])
    reference = [(x, y, heading)]

    point_block = re.search(
        r"Link\.0\.Seg\.0\.Curve\.Points:\n((?:\s+[^\n]+\n)+)", text
    )
    if point_block:
        explicit = []
        for line in point_block.group(1).strip().splitlines():
            fields = line.split()
            explicit.append(
                (float(fields[2]), float(fields[3]), math.radians(float(fields[8])))
            )
        x, y, heading = explicit[0]
        reference = [(x, y, heading)]
        for end_x, end_y, end_heading in explicit[1:]:
            start = (x, y)
            end = (end_x, end_y)
            reference.extend(
                _sample_hermite(start, end, heading, end_heading, spacing)
            )
            x, y, heading = end_x, end_y, end_heading

    segment_pattern = re.compile(
        r"Link\.0\.Seg\.(\d+)\.Type = (\w+)\n"
        r"Link\.0\.Seg\.\1\.Param = ([^\n]+)"
    )
    for index_text, kind, parameter_text in segment_pattern.findall(text):
        if int(index_text) == 0:
            continue
        values = [float(value) for value in parameter_text.split()]
        if kind == "Straight":
            length = values[0]
            count = max(1, int(math.ceil(length / spacing)))
            step = length / count
            for _ in range(count):
                x += step * math.cos(heading)
                y += step * math.sin(heading)
                reference.append((x, y, heading))
        elif kind in ("TurnLeft", "TurnRight"):
            radius = values[0]
            sign = 1.0 if kind == "TurnLeft" else -1.0
            total_angle = sign * math.radians(values[1])
            count = max(1, int(math.ceil(abs(radius * total_angle) / spacing)))
            center_x = x - sign * radius * math.sin(heading)
            center_y = y + sign * radius * math.cos(heading)
            start_phase = math.atan2(y - center_y, x - center_x)
            start_heading = heading
            for step_index in range(1, count + 1):
                delta = total_angle * step_index / count
                phase = start_phase + delta
                x = center_x + radius * math.cos(phase)
                y = center_y + radius * math.sin(phase)
                heading = start_heading + delta
                reference.append((x, y, heading))
        else:
            raise ValueError(f"Unsupported IPGRoad segment type: {kind}")

    centerline = []
    for x, y, heading in reference:
        centerline.append(
            (
                x + lane_center_offset * math.sin(heading),
                y - lane_center_offset * math.cos(heading),
                heading,
            )
        )
    return centerline


def write_lanelet_map(centerline, output: Path, lane_width: float, chunk_size: int):
    root = ET.Element("osm", version="0.6", generator="autoware_carmaker_launch")
    ET.SubElement(root, "MetaInfo", format_version="1", map_version="1")
    node_id = 1
    way_id = 100000
    relation_id = 200000
    half_width = lane_width / 2.0

    left_ids = []
    right_ids = []
    closed_loop = math.hypot(
        centerline[-1][0] - centerline[0][0],
        centerline[-1][1] - centerline[0][1],
    ) < 0.1
    for index, (x, y, heading) in enumerate(centerline):
        if closed_loop and index == len(centerline) - 1:
            left_ids.append(left_ids[0])
            right_ids.append(right_ids[0])
            continue
        left = (x - half_width * math.sin(heading), y + half_width * math.cos(heading))
        right = (x + half_width * math.sin(heading), y - half_width * math.cos(heading))
        for point, destination in ((left, left_ids), (right, right_ids)):
            current_id = node_id
            node_id += 1
            node = ET.SubElement(root, "node", id=str(current_id), lat="", lon="")
            ET.SubElement(node, "tag", k="local_x", v=f"{point[0]:.6f}")
            ET.SubElement(node, "tag", k="local_y", v=f"{point[1]:.6f}")
            ET.SubElement(node, "tag", k="ele", v="0.0")
            destination.append(current_id)

    start = 0
    while start < len(centerline) - 1:
        end = min(len(centerline) - 1, start + chunk_size)
        boundary_ways = []
        for ids, boundary_type in ((left_ids, "line_thin"), (right_ids, "road_border")):
            current_way = way_id
            way_id += 1
            way = ET.SubElement(root, "way", id=str(current_way))
            for identifier in ids[start : end + 1]:
                ET.SubElement(way, "nd", ref=str(identifier))
            ET.SubElement(way, "tag", k="type", v=boundary_type)
            ET.SubElement(way, "tag", k="subtype", v="solid")
            boundary_ways.append(current_way)
        relation = ET.SubElement(root, "relation", id=str(relation_id))
        relation_id += 1
        ET.SubElement(relation, "member", type="way", role="left", ref=str(boundary_ways[0]))
        ET.SubElement(relation, "member", type="way", role="right", ref=str(boundary_ways[1]))
        ET.SubElement(relation, "tag", k="type", v="lanelet")
        ET.SubElement(relation, "tag", k="subtype", v="road")
        ET.SubElement(relation, "tag", k="speed_limit", v="15")
        ET.SubElement(relation, "tag", k="location", v="urban")
        ET.SubElement(relation, "tag", k="one_way", v="yes")
        start = end

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="UTF-8", xml_declaration=True)


def write_terrain_pcd(obj_path: Path, output: Path):
    points = []
    with obj_path.open(encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                fields = line.split()
                if len(fields) >= 4:
                    points.append((float(fields[1]), float(fields[2]), float(fields[3])))
    if not points:
        raise ValueError(f"No vertices found in {obj_path}")
    with output.open("w", encoding="utf-8") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\nFIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            stream.write(f"{x:.5f} {y:.5f} {z:.5f} 0.0\n")


def main():
    parser = argparse.ArgumentParser(description="Generate the Autoware Sprint map from CM15 assets")
    parser.add_argument("--road", type=Path, required=True)
    parser.add_argument("--terrain", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--lane-width", type=float, default=4.0)
    parser.add_argument(
        "--lane-center-offset",
        type=float,
        default=0.0,
        help=(
            "Lateral offset from the IPGRoad reference line; "
            "the FSAI vehicle starts on 0.0"
        ),
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    centerline = parse_rd5_centerline(args.road, args.spacing, args.lane_center_offset)
    write_lanelet_map(centerline, args.output / "lanelet2_map.osm", args.lane_width, 20)
    write_terrain_pcd(args.terrain, args.output / "pointcloud_map.pcd")
    (args.output / "map_projector_info.yaml").write_text("projector_type: Local\n", encoding="utf-8")
    print(f"Generated {len(centerline)} centerline samples in {args.output}")


if __name__ == "__main__":
    main()
