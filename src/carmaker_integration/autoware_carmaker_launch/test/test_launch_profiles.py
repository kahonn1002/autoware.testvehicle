"""Profile discovery of carmaker.launch.py; parses YAML only, launches nothing."""

import importlib.util
from pathlib import Path

import pytest

LAUNCH_FILE = Path(__file__).resolve().parent.parent / "launch" / "carmaker.launch.py"


@pytest.fixture(scope="module")
def launch_module():
    spec = importlib.util.spec_from_file_location("carmaker_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_installed_and_generated_profiles_are_discovered(launch_module, tmp_path):
    profiles = tmp_path / "config" / "profiles"
    profiles.mkdir(parents=True)
    (profiles / "fsai_sprint.yaml").write_text("map_path: maps/fs_autonomous_sprint\n")
    (profiles / "fs_autonomous_sprint_lts.yaml").write_text(
        "generated_by: testrun_to_autoware\nmap_path: maps/fs_autonomous_sprint_lts\nauto_route: true\n")
    assert list(launch_module._profile_files(str(tmp_path))) == ["fs_autonomous_sprint_lts", "fsai_sprint"]
    assert launch_module._load_profile("fs_autonomous_sprint_lts", str(tmp_path))["auto_route"] is True
    with pytest.raises(RuntimeError, match="Expected one of: fs_autonomous_sprint_lts, fsai_sprint"):
        launch_module._load_profile("fsai_sprnt", str(tmp_path))


def test_repository_profiles_still_load(launch_module):
    share = LAUNCH_FILE.parent.parent
    for name in ("fsai_sprint", "iso_lanechange_demo"):
        profile = launch_module._load_profile(name, str(share))
        assert {"vehicle_model", "sensor_model", "map_path"} <= set(profile)
