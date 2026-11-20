"""Build a local Lanelet2 course from the embedded ISO TestRun pylon gates.

The road surface is flat in this TestRun. The PCD is sampled road geometry,
not a localization scan; use simulator ground-truth localization.
"""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET


def parse_iso_course(text):
    """Extract only supported straight, flat ISO geometry; reject other roads."""
    def value(key):
        match = re.search(r"^" + re.escape(key) + r"\s*=\s*([^\n]+)", text, re.M)
        if not match:
            raise ValueError(f"Missing {key}")
        return match.group(1).strip()
    if value("Road.nLinks") != "1" or value("Road.nJunctions") != "0":
        raise ValueError("ISO generator supports one straight road without junctions")
    if value("Road.Link.0.Seg.0.Type") != "Straight":
        raise ValueError("ISO road must be straight")
    origin = [float(x) for x in value("Road.Link.0.Node0").split()]
    if any(abs(x) > 1e-8 for x in origin):
        raise ValueError("ISO generator requires the original (0,0,0) road origin")
    length = float(value("Road.Link.0.Seg.0.Param").split()[0])
    gates = []
    pattern = r"^Road\.RL\.1\.Marker\.(\d+)\.Type\s*=\s*DrvPylon$"
    for index in re.findall(pattern, text, re.M):
        params = value(f"Road.RL.1.Marker.{index}.Param")
        variables = dict((name, float(number)) for name, number in
            re.findall(r"\$(\w+)=(-?[\d.]+)", params))
        fields = params.split()
        station = float(fields[0])
        lateral = variables.get("Lateral_Offset", float(fields[2]) if not fields[2].startswith("$") else 0.)
        widths = [number for name, number in variables.items() if name.startswith("Width_")]
        if len(widths) != 1 or not 0 < widths[0] < 20:
            raise ValueError("Each ISO pylon gate must specify one positive section width")
        gates.append((station, lateral, widths[0]))
    gates.sort()
    sections = []
    for station, lateral, width in gates:
        if sections and (lateral, width) == tuple(sections[-1][2:]):
            sections[-1][1] = station
        else:
            sections.append([station, station, lateral, width])
    if len(sections) != 3 or not sections[0][0] > 0 or sections[-1][1] > length:
        raise ValueError("Expected three ordered double-lane-change gate sections")
    return length, sections, gates


def course_point(station, sections):
    """Return y, heading, lane width; quintic transitions have zero end curvature."""
    for i, (start, end, lateral, width) in enumerate(sections):
        if start <= station <= end:
            return lateral, 0., width
        if i+1 < len(sections) and end < station < sections[i+1][0]:
            next_start, _, next_lateral, next_width = sections[i+1]
            span = next_start-end
            t = (station-end)/span
            blend = t*t*t*(10.-15.*t+6.*t*t)
            derivative = 30.*t*t*(1.-t)*(1.-t)/span
            # Open pavement between the narrow gates accommodates body sweep.
            corridor_width = width*(1.-blend)+next_width*blend + 2.*math.sin(math.pi*t)**2
            return lateral+(next_lateral-lateral)*blend, math.atan((next_lateral-lateral)*derivative), corridor_width
    if station < sections[0][0]:
        distance = sections[0][0]-station
        width = min(6., sections[0][3]+distance/10.)
        return sections[0][2], 0., width
    distance = station-sections[-1][1]
    return sections[-1][2], 0., min(6., sections[-1][3]+distance/10.)


def write_map(output, source, spacing=0.5, speed_kmh=10.):
    if not math.isfinite(spacing) or spacing <= 0 or not math.isfinite(speed_kmh) or speed_kmh <= 0:
        raise ValueError("Spacing and speed limit must be finite and positive")
    text = source.read_text(encoding="utf-8")
    length, sections, gates = parse_iso_course(text)
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element("osm", version="0.6", generator="autoware_iso_course")
    ET.SubElement(root, "MetaInfo", format_version="1", map_version="1")
    count = math.ceil(length/spacing)
    left, right, centerline = [], [], []
    for i in range(count+1):
        x = length*i/count
        y, heading, width = course_point(x, sections)
        centerline.append([x, y, heading, width])
        for side, ids in ((1., left), (-1., right)):
            node_id = 1+2*i+(0 if side > 0 else 1)
            px, py = x-side*width/2.*math.sin(heading), y+side*width/2.*math.cos(heading)
            node = ET.SubElement(root, "node", id=str(node_id), lat="", lon="")
            for k,v in (("local_x",px),("local_y",py),("ele",0.)):
                ET.SubElement(node,"tag", k=k,v=f"{v:.6f}")
            ids.append(node_id)
    for chunk, start in enumerate(range(0,count,20)):
        end = min(start+20,count)
        ways = []
        for side, ids in enumerate((left,right)):
            way_id = 100000+2*chunk+side
            way = ET.SubElement(root,"way",id=str(way_id))
            for node_id in ids[start:end+1]:
                ET.SubElement(way,"nd",ref=str(node_id))
            ET.SubElement(way,"tag",k="type",v="road_border")
            ways.append(way_id)
        relation = ET.SubElement(root,"relation",id=str(200000+chunk))
        for role,way_id in zip(("left","right"),ways):
            ET.SubElement(relation,"member",type="way",role=role,ref=str(way_id))
        for k,v in (("type","lanelet"),("subtype","road"),("location","urban"),
                    ("one_way","yes"),("speed_limit",str(speed_kmh))):
            ET.SubElement(relation,"tag",k=k,v=v)
    ET.indent(root,space="  ")
    ET.ElementTree(root).write(output/"lanelet2_map.osm",encoding="utf-8",xml_declaration=True)
    # Road surface and actual gate boundary positions, sampled at 0.5m.
    points = [(length*i/count,j*0.5,0.,0.) for i in range(count+1) for j in range(-20,21)]
    for x,y,width in gates:
        for side in (-1,1):
            for z in range(11):
                points.append((x,y+side*width/2.,z*0.05,200.))
    with (output/"pointcloud_map.pcd").open("w") as stream:
        stream.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        for point in points:
            stream.write(" ".join(f"{value:.5f}" for value in point)+"\n")
    (output/"map_projector_info.yaml").write_text("projector_type: Local\n")
    metadata = {"source":str(source),"sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
        "length_m":length,"gate_sections":sections,"speed_limit_kmh":speed_kmh,
        "localization":"CarMaker ground truth", "pcd":"sampled flat road and pylon gate geometry; not an NDT localization scan",
        "goal":{"x":650.,"y":sections[-1][2],"yaw":0.}}
    (output/"course_metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--testrun",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--spacing",type=float,default=.5)
    parser.add_argument("--speed-kmh",type=float,default=10.)
    args = parser.parse_args()
    metadata = write_map(args.output,args.testrun,args.spacing,args.speed_kmh)
    print(json.dumps(metadata,indent=2))


if __name__ == "__main__":
    main()
