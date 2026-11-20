"""Dry-run installer tests on synthetic /tmp projects only.

Builds a pristine synthetic project from the existing installed CarMaker
sources by reversing the overlay patch chain, then checks that
``install_overlay.py --dry-run`` accepts the original, accepts the fully
installed chain idempotently, and refuses a drifted/corrupt chain without
touching any target. The real project is only read, never written.
"""

import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import pytest

OVERLAY_DIR = Path(__file__).resolve().parents[2] / "carmaker_overlay"
INSTALLER = OVERLAY_DIR / "install_overlay.py"
BASE_PATCH = OVERLAY_DIR / "iso_ros.patch"
ADDITIONAL_PATCHES = [
    OVERLAY_DIR / "iso_camera_lifecycle.patch",
    OVERLAY_DIR / "iso_camera_epoch.patch",
    OVERLAY_DIR / "iso_control_watchdog.patch",
    OVERLAY_DIR / "iso_shared_epoch.patch",
    OVERLAY_DIR / "iso_watchdog_diagnostics.patch",
]
RELATIVE_SOURCES = [
    "ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
    "ros/ros2_ws/src/carmaker_rsds_client/src/carmaker_rsds_client_node.cpp",
    "ros/ros2_ws/src/carmaker_rsds_client/launch/carmaker_rsds_client.launch.py",
]
REAL_PROJECT = Path(
    "/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware"
)


def _require_inputs():
    if shutil.which("git") is None:
        pytest.skip("git is required for patch-chain tests")
    if not (REAL_PROJECT / "src/CM_Main.cpp").is_file():
        pytest.skip("Requires the existing installed CarMaker project")
    for relative in RELATIVE_SOURCES:
        if not (REAL_PROJECT / relative).is_file():
            pytest.skip(f"Missing installed source: {relative}")
    for name in ("Data/Vehicle/Examples/DemoCar",
                 "Data/TestRun/LaneChange_ISO_Camera_RSI"):
        if not (REAL_PROJECT / name).is_file():
            pytest.skip(f"Missing reference data: {name}")
    if not INSTALLER.is_file() or not BASE_PATCH.is_file():
        pytest.skip("Overlay installer sources missing")


def _run_git(args, cwd):
    return subprocess.run(
        ["git"] + args, cwd=cwd, capture_output=True, text=True)


def _make_pristine_original(root: Path) -> Path:
    """Copy installed THREE sources into root and reverse the patch chain."""
    _require_inputs()
    project = root / "FSAI_LTS_2026_CM15_Autoware"
    assert not project.exists(), "Synthetic project must use a fresh temporary directory"
    for relative in RELATIVE_SOURCES:
        dest = project / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REAL_PROJECT / relative, dest)
    # Reverse optional patches first (newest first), mirroring the installer.
    for patch in reversed(ADDITIONAL_PATCHES):
        check = _run_git(["apply", "--reverse", "--check", str(patch)], cwd=project)
        if check.returncode == 0:
            rev = _run_git(["apply", "--reverse", str(patch)], cwd=project)
            assert rev.returncode == 0, rev.stderr
    check = _run_git(["apply", "--reverse", "--check", str(BASE_PATCH)], cwd=project)
    assert check.returncode == 0, f"installed sources lack base patch: {check.stderr}"
    rev = _run_git(["apply", "--reverse", str(BASE_PATCH)], cwd=project)
    assert rev.returncode == 0, rev.stderr
    (project / "src").mkdir(parents=True, exist_ok=True)
    shutil.copy2(REAL_PROJECT / "src/CM_Main.cpp", project / "src/CM_Main.cpp")
    for name in ("Data/Vehicle/Examples/DemoCar",
                 "Data/TestRun/LaneChange_ISO_Camera_RSI"):
        dest = project / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REAL_PROJECT / name, dest)
    return project


def _hashes(project: Path):
    out = {}
    for relative in RELATIVE_SOURCES:
        data = (project / relative).read_bytes()
        out[relative] = hashlib.sha256(data).hexdigest()
    return out


def _dry_run(project: Path):
    return subprocess.run(
        ["/usr/bin/python3", str(INSTALLER), "--project", str(project), "--dry-run"],
        capture_output=True, text=True)


def _assert_no_backups(project: Path):
    leftovers = [p.name for p in project.iterdir()
                 if p.name.startswith("iso-overlay-")]
    assert leftovers == [], f"dry-run touched targets: {leftovers}"


def test_dry_run_accepts_pristine_original(tmp_path):
    project = _make_pristine_original(tmp_path / "orig")
    before = _hashes(project)
    proc = _dry_run(project)
    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["patch_already_applied"] is False
    assert _hashes(project) == before
    _assert_no_backups(project)


def test_dry_run_idempotent_on_installed_chain(tmp_path):
    project = _make_pristine_original(tmp_path / "inst")
    # Forward-apply the full chain to synthesize the installed state.
    assert _run_git(["apply", "--check", str(BASE_PATCH)], cwd=project).returncode == 0
    assert _run_git(["apply", str(BASE_PATCH)], cwd=project).returncode == 0
    for patch in ADDITIONAL_PATCHES:
        assert _run_git(["apply", "--check", str(patch)], cwd=project).returncode == 0
        assert _run_git(["apply", str(patch)], cwd=project).returncode == 0
    before = _hashes(project)
    proc = _dry_run(project)
    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["patch_already_applied"] is True
    assert _hashes(project) == before
    _assert_no_backups(project)


def test_dry_run_refuses_drifted_partial_chain(tmp_path):
    project = _make_pristine_original(tmp_path / "corr")
    assert _run_git(["apply", str(BASE_PATCH)], cwd=project).returncode == 0
    target = project / RELATIVE_SOURCES[0]
    text = target.read_text()
    assert "use_manual_transmission" in text
    target.write_text(text.replace("use_manual_transmission",
                                   "use_manual_transmission_DRIFTED"))
    others_before = {k: v for k, v in _hashes(project).items()
                     if k != RELATIVE_SOURCES[0]}
    proc = _dry_run(project)
    assert proc.returncode != 0
    assert "do not match patch" in proc.stderr
    # Failed dry-run must not create backups or heal/modify other targets.
    _assert_no_backups(project)
    after = _hashes(project)
    for key, digest in others_before.items():
        assert after[key] == digest
