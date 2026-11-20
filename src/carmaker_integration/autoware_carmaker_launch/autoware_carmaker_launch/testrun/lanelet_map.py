"""In-memory Lanelet2 map with point merging and Autoware Local-projector output."""

import math
from xml.etree import ElementTree as ET


class LaneletMap:
    """Points, line strings and lanelets sharing one ID space, as Lanelet2 expects."""

    def __init__(self):
        self.points = {}
        self.ways = {}
        self.lanelets = {}
        self._parent = {}
        self._next_id = 1

    def _allocate(self):
        identifier = self._next_id
        self._next_id += 1
        return identifier

    def add_point(self, x, y, z=0.0):
        identifier = self._allocate()
        self.points[identifier] = (float(x), float(y), float(z))
        self._parent[identifier] = identifier
        return identifier

    def find(self, point):
        root = point
        while self._parent[root] != root:
            root = self._parent[root]
        while self._parent[point] != root:
            self._parent[point], point = root, self._parent[point]
        return root

    def coordinates(self, point):
        return self.points[self.find(point)]

    def distance(self, first, second):
        a, b = self.coordinates(first), self.coordinates(second)
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def merge(self, keep, other):
        """Make `other` the same Lanelet2 point as `keep`; `keep` wins the position."""
        keep, other = self.find(keep), self.find(other)
        if keep != other:
            self._parent[other] = keep

    def add_way(self, points, **tags):
        identifier = self._allocate()
        self.ways[identifier] = {"points": list(points), "tags": tags}
        return identifier

    def add_lanelet(self, left, right, **tags):
        identifier = self._allocate()
        self.lanelets[identifier] = {"left": left, "right": right, "tags": tags}
        return identifier

    def way_points(self, way):
        resolved = []
        for point in self.ways[way]["points"]:
            root = self.find(point)
            if not resolved or resolved[-1] != root:
                resolved.append(root)
        return resolved

    def bounds(self, lanelet):
        """Return resolved (left, right) coordinate lists of a lanelet."""
        item = self.lanelets[lanelet]
        return ([self.points[p] for p in self.way_points(item["left"])],
                [self.points[p] for p in self.way_points(item["right"])])

    def write(self, path, generator="autoware_carmaker_launch"):
        """Write an Autoware Local-projector OSM file; drop degenerate lanelets."""
        dropped = [identifier for identifier, item in self.lanelets.items()
                   if len(self.way_points(item["left"])) < 2
                   or len(self.way_points(item["right"])) < 2]
        for identifier in dropped:
            del self.lanelets[identifier]
        used_ways = {item[role] for item in self.lanelets.values() for role in ("left", "right")}
        used_points = sorted({point for way in used_ways for point in self.way_points(way)})

        root = ET.Element("osm", version="0.6", generator=generator)
        ET.SubElement(root, "MetaInfo", format_version="1", map_version="1")
        for identifier in used_points:
            x, y, z = self.points[identifier]
            node = ET.SubElement(root, "node", id=str(identifier), lat="", lon="")
            for key, value in (("local_x", x), ("local_y", y), ("ele", z)):
                ET.SubElement(node, "tag", k=key, v=f"{value:.6f}")
        for identifier in sorted(used_ways):
            way = ET.SubElement(root, "way", id=str(identifier))
            for point in self.way_points(identifier):
                ET.SubElement(way, "nd", ref=str(point))
            for key, value in self.ways[identifier]["tags"].items():
                ET.SubElement(way, "tag", k=key, v=str(value))
        for identifier in sorted(self.lanelets):
            item = self.lanelets[identifier]
            relation = ET.SubElement(root, "relation", id=str(identifier))
            for role in ("left", "right"):
                ET.SubElement(relation, "member", type="way", role=role, ref=str(item[role]))
            tags = {"type": "lanelet", **item["tags"]}
            for key, value in tags.items():
                ET.SubElement(relation, "tag", k=key, v=str(value))
        ET.indent(root, space="  ")
        ET.ElementTree(root).write(path, encoding="UTF-8", xml_declaration=True)
        return {"points": len(used_points), "ways": len(used_ways),
                "lanelets": len(self.lanelets), "dropped_degenerate_lanelets": len(dropped)}


def lanelet_tags(speed_limit_kmh, **extra):
    """Tags Autoware requires on drivable road lanelets."""
    tags = {"subtype": "road", "location": "urban", "one_way": "yes",
            "speed_limit": f"{speed_limit_kmh:g}"}
    tags.update({key: value for key, value in extra.items() if value is not None})
    return tags


def chunk_bounds(count, length, chunk_length, forced=()):
    """Split `count` stations covering `length` metres into lanelet-sized index ranges.

    `forced` station indices always start a new lanelet, e.g. where a loop closes.
    """
    pieces = max(1, math.ceil(length / max(chunk_length, 1e-6) - 1e-9))
    pieces = min(pieces, max(1, count - 1))
    edges = {round(i * (count - 1) / pieces) for i in range(pieces + 1)}
    edges.update(index for index in forced if 0 < index < count - 1)
    edges = sorted(edges)
    return list(zip(edges, edges[1:]))
