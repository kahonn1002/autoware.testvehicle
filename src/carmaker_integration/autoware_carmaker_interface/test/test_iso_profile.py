"""Regression: ISO profile must route IMU to the AEB-expected topic.

No ROS nodes or simulation are launched; this only parses YAML profiles.
"""

import re
from pathlib import Path

CONFIG = Path(__file__).resolve().parent.parent / "config"
ISO = CONFIG / "demo_iso.interface.param.yaml"
DEFAULT = CONFIG / "interface.param.yaml"
ISO_IMU_TOPIC = "/sensing/imu/imu_data"
DEFAULT_IMU_TOPIC = "/sensing/imu/carmaker/imu_raw"


def _imu_topic(path: Path) -> str:
    match = re.search(r"(?m)^\s*output_imu_topic:\s*(\S+)\s*$", path.read_text(encoding="utf-8"))
    assert match, f"output_imu_topic missing in {path.name}"
    return match.group(1)


def test_iso_profile_routes_imu_to_aeb_topic():
    assert _imu_topic(ISO) == ISO_IMU_TOPIC


def test_default_profile_unchanged():
    assert _imu_topic(DEFAULT) == DEFAULT_IMU_TOPIC
