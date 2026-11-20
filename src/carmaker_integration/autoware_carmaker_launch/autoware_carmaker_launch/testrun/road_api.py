"""Read-only access to IPGRoad networks through CarMaker's IPGRoad Python API.

The API is CarMaker's own road evaluator, so sampled geometry is the same
geometry CarMaker uses for ground truth. It is shipped as a wheel inside the
CarMaker installation and needs an IPGRoad license.
"""

import contextlib
import importlib
import math
import os
import re
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path

DEFAULT_IPG_ROOT = Path("/opt/ipg/carmaker")
# tRoadMarkerType values used here.
MARKER_SPEED, MARKER_PYLON = 0, 2


class RoadError(RuntimeError):
    pass


def find_carmaker_home(explicit=None):
    """Return the newest CarMaker installation (CM15 here) unless one is given."""
    if explicit:
        home = Path(explicit)
        if not (home / "Python" / "IPGRoad").is_dir():
            raise RoadError(f"{home} does not contain Python/IPGRoad")
        return home
    candidates = []
    if os.environ.get("IPGHOME"):
        candidates.extend(sorted(Path(os.environ["IPGHOME"], "carmaker").glob("linux64-*")))
    candidates.extend(sorted(DEFAULT_IPG_ROOT.glob("linux64-*")))
    candidates = [home for home in candidates if (home / "Python" / "IPGRoad").is_dir()]
    if not candidates:
        raise RoadError("No CarMaker installation with Python/IPGRoad found; pass --carmaker-home")
    return max(candidates, key=lambda home: tuple(int(part) for part in re.findall(r"\d+", home.name)))


def load_ipgroad(carmaker_home):
    """Import the `ipgroad` module, extracting CarMaker's wheel into a user cache."""
    with contextlib.suppress(ImportError):
        return importlib.import_module("ipgroad")
    wheels = sorted((Path(carmaker_home) / "Python" / "IPGRoad").glob("ipgroad-*.whl"))
    if not wheels:
        raise RoadError(f"No ipgroad wheel in {carmaker_home}/Python/IPGRoad")
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    target = cache_root / "autoware_carmaker" / wheels[-1].stem
    if not (target / "ipgroad" / "__init__.py").is_file():
        with zipfile.ZipFile(wheels[-1]) as archive:
            for member in archive.namelist():
                relative = member.split("/purelib/", 1)[-1] if "/purelib/" in member else member
                if not relative.startswith("ipgroad/") or member.endswith("/"):
                    continue
                destination = target / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(archive.read(member))
    sys.path.insert(0, str(target))
    return importlib.import_module("ipgroad")


@dataclass
class RoutePoint:
    x: float
    y: float
    z: float
    heading: float
    s: float
    t: float
    width_left: float
    width_right: float


@dataclass
class PathInfo:
    path_id: int
    kind: str
    root: int
    root_kind: str
    length: float
    in_link_dir: int


@dataclass
class Marker:
    kind: int
    s: float
    t: float
    x: float
    y: float
    z: float
    width: float = 0.0
    speed: float = 0.0


@dataclass
class PathSample:
    x: float
    y: float
    z: float
    heading: float
    left: tuple
    right: tuple


class IpgRoad:
    """Thin wrapper hiding the SWIG pointer conventions of the IPGRoad API."""

    def __init__(self, api, road_file):
        self.api = api
        self.road = api.RoadNew()
        self.road_file = Path(road_file)
        if api.RoadReadInfoFile(self.road, str(road_file)) != api.ROAD_Ok:
            raise RoadError(f"IPGRoad could not read {road_file}: {self.messages()}")
        self._evals = {}
        self._kinds = {}
        for name in dir(api):
            if name.startswith("ROT_"):
                self._kinds.setdefault(getattr(api, name), name[4:])

    def messages(self):
        api = self.api
        messages = api.tRoadMessageList()
        api.RoadGetMessageList(self.road, api.RMT_Error, messages)
        return "; ".join(api.tRoadMessageArray_getitem(messages.msg, i).msg
                         for i in range(messages.nMsgList)) or "no IPGRoad message"

    def _ids(self, function, *args):
        api = self.api
        holder = api.new_tObjIdPointerPointer()
        count = function(*args, holder)
        array = api.tObjIdPointerPointer_value(holder)
        return [api.tObjIdArray_getitem(array, i) for i in range(max(count, 0))]

    def _doubles(self, holder, count):
        array = self.api.doublePointerPointer_value(holder)
        return [self.api.doubleArray_getitem(array, i) for i in range(max(count, 0))]

    def _vector(self, array, size=3):
        return [self.api.doubleArray_getitem(array, i) for i in range(size)]

    def _eval(self, route=None, use_path=True):
        key = (route, use_path)
        if key not in self._evals:
            api = self.api
            handle = api.RoadNewRoadEval(self.road, api.ROAD_BUMP_NONE, api.ROAD_OT_ALL, None)
            if route is not None and api.RoadEvalSetRouteByObjId(handle, route, int(use_path)) != api.ROAD_Ok:
                raise RoadError(f"Route object {route} is invalid: {self.messages()}")
            self._evals[key] = handle
        return self._evals[key]

    def kind(self, object_id):
        return self._kinds.get(self.api.RoadGetObjectType(self.road, object_id), "Invalid")

    # Network topology -----------------------------------------------------
    def links(self):
        return self._ids(self.api.RoadGetLinkObjIds, self.road)

    def lane_sections(self, link):
        return self._ids(self.api.RoadGetLaneSectionObjIds, self.road, link)

    def section_range(self, section):
        return (self.api.RoadGetLSStart(self.road, section),
                self.api.RoadGetLSLength(self.road, section))

    def lanes(self, section, right):
        side = self.api.R_RIGHT if right else self.api.R_LEFT
        return self._ids(self.api.RoadGetLaneObjIds, self.road, section, side)

    def lane_type(self, lane):
        return int(self.api.RoadGetLaneType(self.road, lane))

    def lane_path(self, lane):
        return self.api.RoadLaneGetLanePathObjId(self.road, lane)

    def section_borders(self, section, s_relative):
        holder = self.api.new_doublePointerPointer()
        count = self.api.RoadLaneSectionGetLaneBorders(self.road, section, s_relative, holder)
        if count < 0:
            raise RoadError(f"No lane borders for section {section} at s={s_relative:.3f}")
        return self._doubles(holder, count)

    def link_point(self, link, s, t):
        api = self.api
        request = api.tRoadLinkIn()
        request.lObjId, request.useST = link, 1
        api.doubleArray_setitem(request.st, 0, s)
        api.doubleArray_setitem(request.st, 1, t)
        result = api.tRoadLinkOut()
        if api.RoadLinkGeoEval(self._eval(), "link", request, result) != api.ROAD_Ok:
            raise RoadError(f"Link {link} evaluation failed at s={s:.3f}, t={t:.3f}")
        return tuple(self._vector(result.xyz))

    def paths(self):
        return self._ids(self.api.RoadGetPathObjIds, self.road, self.api.ROAD_InvalidObjId,
                         self.api.ROT_Invalid)

    def path_info(self, path):
        info = self.api.RoadGetRoadPathByObjId(self.road, path)
        if info is None:
            raise RoadError(f"Path {path} does not exist")
        kind = self._kinds.get(info.pathType, str(info.pathType))
        return PathInfo(path, kind, info.rootObjId, self.kind(info.rootObjId), info.len,
                        1 if info.inRefDir >= 0 else -1)

    def out_paths(self, path):
        api = self.api
        offsets = api.new_doublePointerPointer()
        identifiers = self._ids(lambda road, pid, holder: api.RoadPathGetPossibleOutPaths(
            road, pid, holder, offsets), self.road, path)
        return list(zip(identifiers, self._doubles(offsets, len(identifiers))))

    def path_sample(self, path, s):
        api = self.api
        request = api.tRoadPathIn()
        request.actObjId, request.nextObjId = path, api.ROAD_InvalidObjId
        api.doubleArray_setitem(request.st, 0, s)
        api.doubleArray_setitem(request.st, 1, 0.0)
        result = api.tRoadPathOut()
        if api.RoadPathGeoEval(self._eval(), "path", api.RIT_ST, request, result) != api.ROAD_Ok:
            raise RoadError(f"Path {path} evaluation failed at s={s:.3f}")
        x, y, z = self._vector(result.xyz)
        sx, sy, _ = self._vector(result.suv)
        tx, ty, tz = self._vector(result.tuv)
        left, right = (max(value, 0.0) for value in self._vector(result.width, 2))
        return PathSample(x, y, z, math.atan2(sy, sx),
                          (x + left * tx, y + left * ty, z + left * tz),
                          (x - right * tx, y - right * ty, z - right * tz))

    def path_speeds(self, paths):
        """Speed limits (m/s, offset along the paths) from the first path's start."""
        api = self.api
        identifiers = api.new_tObjIdArray(len(paths) + 1)
        for index, path in enumerate(paths):
            api.tObjIdArray_setitem(identifiers, index, path)
        api.tObjIdArray_setitem(identifiers, len(paths), -1)
        request = api.tRoadPathSpeedIn()
        request.objId, request.sOnRef, request.range = identifiers, 0.0, 1.0e6
        result = api.new_tRoadPathSpeedOutArray(3)
        count = api.RoadPathGetSpeed(self.road, request, result)
        return [(api.tRoadPathSpeedOutArray_getitem(result, i).speed,
                 api.tRoadPathSpeedOutArray_getitem(result, i).sOnRefOff)
                for i in range(max(count, 0))]

    # Routes ---------------------------------------------------------------
    def routes(self):
        api = self.api
        routes = {}
        for index in range(api.RoadGetRouteCount(self.road)):
            route = api.RoadRouteIdGetRouteObjId(self.road, index)
            routes[route] = api.RoadRouteGetNameByObjId(self.road, route) or f"Route_{index}"
        return routes

    def route_length(self, route):
        return self.api.RoadRouteObjGetLength(self.road, route)

    def route_loop(self, route):
        """Return (loop start, loop length) for a closed route, else None."""
        start, length = self.api.new_doublePointer(), self.api.new_doublePointer()
        if self.api.RoadRouteObjClosed(self.road, route, start, length) <= 0:
            return None
        return self.api.doublePointer_value(start), self.api.doublePointer_value(length)

    def route_paths(self, route):
        api = self.api
        driving_path = api.RoadRouteGetDrvPathObjId(self.road, route)
        starts, ends = api.new_doublePointerPointer(), api.new_doublePointerPointer()
        identifiers = self._ids(lambda road, path, holder: api.RoadDrvPathGetPathList(
            road, path, holder, starts, ends), self.road, driving_path)
        return list(zip(identifiers, self._doubles(starts, len(identifiers)),
                        self._doubles(ends, len(identifiers))))

    def route_point(self, route, s, t=0.0, use_path=True):
        api = self.api
        request = api.tRoadRouteIn()
        api.doubleArray_setitem(request.st, 0, s)
        api.doubleArray_setitem(request.st, 1, t)
        return self._route_eval(route, use_path, api.RIT_ST, request, f"s={s:.3f}, t={t:.3f}")

    def route_project(self, route, x, y, z=0.0, use_path=True):
        """Return the route point closest to a global position (route s/t)."""
        api = self.api
        request = api.tRoadRouteIn()
        for index, value in enumerate((x, y, z)):
            api.doubleArray_setitem(request.xyz, index, value)
        return self._route_eval(route, use_path, api.RIT_XY_S, request, f"x={x:.3f}, y={y:.3f}")

    def _route_eval(self, route, use_path, input_type, request, where):
        api = self.api
        result = api.tRoadRouteOut()
        if api.RoadRouteEval(self._eval(route, use_path), "route", input_type, request,
                             result) != api.ROAD_Ok:
            raise RoadError(f"Route {route} evaluation failed at {where}: {self.messages()}")
        x, y, z = self._vector(result.xyz)
        sx, sy, _ = self._vector(result.suv)
        s, t = self._vector(result.st, 2)
        left, right = self._vector(result.width, 2)
        return RoutePoint(x, y, z, math.atan2(sy, sx), s, t, left, right)

    def route_markers(self, route, kinds):
        api = self.api
        mask = sum(1 << kind for kind in kinds)
        holder = api.new_tRoadMarkerOutPointerPointer()
        count = api.RoadGetAllMarkerByObjId(self.road, route, mask, holder)
        array = api.tRoadMarkerOutPointerPointer_value(holder)
        markers = []
        for index in range(max(count, 0)):
            item = api.tRoadMarkerOutArray_getitem(array, index)
            x, y, z = self._vector(item.xyz)
            marker = Marker(item.type, item.s, item.t, x, y, z)
            if item.type == MARKER_PYLON:
                marker.width = item.pylon.width
            elif item.type == MARKER_SPEED:
                marker.speed = item.speed.speed
            markers.append(marker)
        return markers


def open_road(road_file, carmaker_home):
    """Load an IPGRoad file (or a TestRun with an embedded road) from its project."""
    api = load_ipgroad(carmaker_home)
    road_file = Path(road_file).resolve()
    data_dir = next((parent for parent in road_file.parents if parent.name == "Data"), None)
    previous = Path.cwd()
    # Relative resources in a road file are resolved against the project root.
    os.chdir(data_dir.parent if data_dir else road_file.parent)
    try:
        return IpgRoad(api, road_file)
    finally:
        os.chdir(previous)
