"""Launch profiles for converted TestRuns.

A generated profile reuses the vehicle, sensor kit and interface settings of an
existing profile and points it at the converted map.
"""

import re
from pathlib import Path

import yaml

GENERATOR = "testrun_to_autoware"
NAME_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_]*$")
VEHICLE_PROFILES = {
    "FS-AI_IMechE_ADS-DV_2026": "fsai_sprint",
    "Examples/DemoCar": "iso_lanechange_demo",
    "Autoware/DemoCar_ISO": "iso_lanechange_demo",
    "Autoware/DemoCar_ISO_NoRSI": "iso_lanechange_demo",
}
COPIED_KEYS = ("vehicle_model", "sensor_model", "vehicle_launch_package", "interface_package",
               "interface_params", "converter_params", "camera_bridge")
COMPARED_KEYS = ("wheel_base", "wheel_tread", "front_overhang", "rear_overhang",
                 "vehicle_height", "wheel_radius", "wheel_width")


def default_name(testrun_path):
    name = re.sub(r"[^a-z0-9]+", "_", Path(testrun_path).name.lower()).strip("_")
    return name if NAME_PATTERN.match(name) else f"testrun_{name}"


def select_base_profile(vehicle, explicit, available):
    name = explicit or VEHICLE_PROFILES.get(vehicle)
    if name is None:
        raise ValueError(
            f"No Autoware vehicle profile is known for CarMaker vehicle '{vehicle}'. "
            f"Pass --base-profile with one of: {', '.join(sorted(available))}.")
    if name not in available:
        raise ValueError(f"Base profile '{name}' not found; expected one of: {', '.join(sorted(available))}")
    return name


def load_profiles(profile_dir):
    profiles = {}
    for path in sorted(Path(profile_dir).glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            profiles[path.stem] = data
    return profiles


def is_generated(path):
    if not path.exists():
        return False
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return isinstance(data, dict) and data.get("generated_by") == GENERATOR


def _ros_parameters(path):
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for value in data.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            return value["ros__parameters"]
    return None


def vehicle_info(source_root, vehicle_model):
    return _ros_parameters(
        Path(source_root) / f"{vehicle_model}_description" / "config" / "vehicle_info.param.yaml")


def interface_offset(source_root, base):
    """Offset the base profile's interface adds to CarMaker's Fr1 pose to get base_link."""
    params = _ros_parameters(Path(source_root) / base.get("interface_package", "") / "config"
                             / base.get("interface_params", "")) or {}
    return tuple(float(params.get(f"source_base_offset_{axis}", 0.0)) for axis in "xyz")


def compare_vehicle(carmaker, autoware, tolerance=0.05):
    """Differences between CarMaker-derived geometry and the Autoware vehicle_info."""
    if not carmaker or not autoware:
        return []
    differences = []
    for key in COMPARED_KEYS:
        if key in carmaker and key in autoware:
            delta = float(autoware[key]) - carmaker[key]
            if abs(delta) > tolerance:
                differences.append({"parameter": key, "carmaker": carmaker[key],
                                    "autoware": float(autoware[key]), "difference": round(delta, 4)})
    return differences


def generated_profile(base_name, base, map_path, testrun_path):
    profile = {"generated_by": GENERATOR, "source_testrun": str(testrun_path),
               "base_profile": base_name}
    profile.update({key: base[key] for key in COPIED_KEYS if key in base})
    profile.update({"map_path": map_path, "auto_route": True})
    return profile
