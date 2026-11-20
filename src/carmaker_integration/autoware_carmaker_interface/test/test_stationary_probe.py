"""Tests for carmaker_overlay/stationary_readiness.py pure helpers + Probe.

No simulator/DDS/ROS launches; Probe exercised with a fake node.
"""

import importlib.util
import inspect
import math
import struct
import sys
import time
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

OVERLAY_DIR = Path(__file__).resolve().parents[2] / "carmaker_overlay"


def _load_readiness():
    path = str(OVERLAY_DIR)
    sys.path.insert(0, path)
    try:
        spec = importlib.util.spec_from_file_location(
            "stationary_readiness", OVERLAY_DIR / "stationary_readiness.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["stationary_readiness"] = mod
        spec.loader.exec_module(mod)
        return mod
    finally:
        try:
            sys.path.remove(path)
        except ValueError:
            pass


_sr = _load_readiness()
summarize = _sr.summarize
stamp_seconds = _sr.stamp_seconds
cloud_sample_count = _sr.cloud_sample_count
evaluate_report = _sr.evaluate_report
Probe = _sr.Probe


def _stamp(sec, ns=0):
    return Time(sec=int(sec), nanosec=int(ns))


def _cloud(points, datatype=7, bigendian=False, height=1, pad=0):
    fmt = ("f" if datatype == 7 else "d")
    size = struct.calcsize(fmt)
    step = 3 * size
    width = len(points)
    row_step = width * step + pad
    fields = [PointField(name=n, offset=i * size, datatype=datatype, count=1)
              for i, n in enumerate(("x", "y", "z"))]
    data = bytearray(row_step * height)
    for r in range(height):
        for c, pt in enumerate(points):
            base = r * row_step + c * step
            for i, v in enumerate(pt):
                struct.pack_into((">" if bigendian else "<") + fmt, data,
                                 base + i * size, v)
    msg = PointCloud2()
    msg.fields = fields
    msg.point_step = step
    msg.row_step = row_step
    msg.width = width
    msg.height = height
    msg.is_bigendian = bigendian
    msg.data = bytes(data)
    return msg


def _probe():
    return Probe(object())


# --- summarize / stamp_seconds ---

def test_summarize_empty_is_none():
    assert summarize([]) is None


def test_summarize_median_p95_bounds():
    vals = list(range(100))
    out = summarize(vals)
    assert out["count"] == 100 and out["min"] == 0 and out["max"] == 99
    assert out["median"] == 49.5
    assert out["p95"] == vals[min(99, int(.95 * 100))]
    single = summarize([3.0])
    assert single == {"count": 1, "min": 3.0, "median": 3.0, "p95": 3.0, "max": 3.0}


def test_stamp_seconds():
    assert stamp_seconds(_stamp(10, 500_000_000)) == 10.5


# --- cloud_sample_count ---

def test_cloud_zero_width_or_step_is_zero():
    msg = _cloud([(1.0, 2.0, 3.0)])
    msg.width = 0
    assert cloud_sample_count(msg) == 0
    msg2 = _cloud([(1.0, 2.0, 3.0)])
    msg2.point_step = 0
    assert cloud_sample_count(msg2) == 0


def test_cloud_all_zero_not_counted():
    assert cloud_sample_count(_cloud([(0.0, 0.0, 0.0)] * 4)) == 0


def test_cloud_nan_inf_not_counted():
    pts = [(math.nan, 0.0, 0.0), (math.inf, 1.0, 1.0), (0.0, 0.0, math.nan)]
    assert cloud_sample_count(_cloud(pts)) == 0


def test_cloud_truncated_data_returns_partial():
    msg = _cloud([(1.0, 2.0, 3.0)] * 4)
    msg.data = msg.data[:5]  # malformed/truncated
    assert cloud_sample_count(msg) == 0  # must not raise


def test_cloud_float32_le_nonzero():
    assert cloud_sample_count(_cloud([(1.0, 2.0, 3.0)] * 3)) == 3


def test_cloud_float64_be_nonzero():
    msg = _cloud([(1.0, -2.0, 0.5)] * 2, datatype=8, bigendian=True)
    assert cloud_sample_count(msg) == 2


def test_cloud_organized_row_padding():
    msg = _cloud([(1.0, 1.0, 1.0)] * 2, height=2, pad=8)
    # width=2,height=2 -> total 4 sampled points, all valid despite padding
    assert cloud_sample_count(msg) == 4


def test_cloud_missing_or_wrong_fields():
    msg = _cloud([(1.0, 2.0, 3.0)])
    msg.fields = [f for f in msg.fields if f.name != "z"]
    assert cloud_sample_count(msg) == 0
    msg2 = _cloud([(1.0, 2.0, 3.0)])
    msg2.fields[0].datatype = 2  # not float32/64
    assert cloud_sample_count(msg2) == 0


# --- Probe ---

def test_callback_signature_single_argument():
    cb = _probe().callback("/clock")
    assert len(inspect.signature(cb).parameters) == 1


def test_header_age_uses_latest_clock():
    p = _probe()
    p.receive("/clock", SimpleNamespace(clock=_stamp(100)))
    p.receive("/clock", SimpleNamespace(clock=_stamp(110)))
    p.receive("/topic", SimpleNamespace(header=SimpleNamespace(stamp=_stamp(109))))
    assert p.ages["/topic"] == [1.0]


def test_steady_ages_only_after_route_plus_10s():
    p = _probe()
    p.receive("/clock", SimpleNamespace(clock=_stamp(50)))
    msg = SimpleNamespace(header=SimpleNamespace(stamp=_stamp(49)))
    p.route_wall = None
    p.receive("/t", msg)
    assert "/t" not in p.steady_ages
    p.route_wall = time.monotonic()  # just now -> not steady
    p.receive("/t", msg)
    assert "/t" not in p.steady_ages
    p.route_wall = time.monotonic() - 11.0
    p.receive("/t", msg)
    assert len(p.steady_ages["/t"]) == 1


def _odom(vx):
    return SimpleNamespace(
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=vx))),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0))))


def test_counts_raw_odom_speed_and_any_vehicle_control():
    p = _probe()
    p.receive("/carmaker/odometry", _odom(1.5))
    p.receive("/carmaker/odometry", _odom(-2.5))
    assert p.max_speed == 2.5
    assert p.topics["/carmaker/odometry"]["position"] == [1.0, 2.0]
    p.receive("/carmaker/VehicleControl", SimpleNamespace(foo=1))
    p.receive("/carmaker/VehicleControl", SimpleNamespace(bar=2))
    assert p.commands == 2


def _mode_msg(mode, auto, avail, enabled):
    return SimpleNamespace(mode=mode, AUTONOMOUS=auto,
                           is_autonomous_mode_available=avail,
                           is_autoware_control_enabled=enabled)


def test_detects_autonomous_and_availability():
    p = _probe()
    p.receive("/api/operation_mode/state", _mode_msg(0, 1, False, False))
    assert not p.autonomous_seen and p.available_count == 0
    p.receive("/system/operation_mode/state", _mode_msg(1, 1, True, True))
    assert p.autonomous_seen and p.available_count == 1
    assert p.topics["/system/operation_mode/state"]["state"]["control_enabled"] is True


def test_result_stats_metric_fields():
    p = _probe()
    p.receive("/clock", SimpleNamespace(clock=_stamp(20)))
    p.receive("/planning/trajectory",
              SimpleNamespace(header=SimpleNamespace(stamp=_stamp(19))))
    p.receive("/metric", SimpleNamespace(data=2.0))
    p.receive("/metric", SimpleNamespace(data=4))
    out = p.result()
    assert out["topics"]["/planning/trajectory"]["age_sec"]["count"] == 1
    assert out["topics"]["/planning/trajectory"]["steady_age_sec"] is None
    assert out["topics"]["/metric"]["metric"]["median"] == 3.0
    assert out["safety"]["vehicle_commands_observed"] == 0
    assert out["autonomous_available_observations"] == 0


def test_no_input_does_not_fake_presence():
    p = _probe()
    p.receive("/perception/object_recognition/objects", SimpleNamespace(nothing=1))
    item = p.topics["/perception/object_recognition/objects"]
    assert "last_objects" not in item and "last_points" not in item
    p.receive("/perception/object_recognition/objects",
              SimpleNamespace(objects=[1, 2, 3]))
    assert p.topics["/perception/object_recognition/objects"]["last_objects"] == 3
    p.receive("/x", SimpleNamespace(points=[1, 2]))
    assert p.topics["/x"]["last_points"] == 2


def test_trajectory_reports_planned_speed_without_actuation():
    p = _probe()
    p.receive("/planning/trajectory", SimpleNamespace(points=[
        SimpleNamespace(longitudinal_velocity_mps=0.),
        SimpleNamespace(longitudinal_velocity_mps=2.78),
        SimpleNamespace(longitudinal_velocity_mps=-1.),
    ]))
    assert p.topics["/planning/trajectory"]["last_max_planned_speed_mps"] == 2.78
    assert p.commands == 0


def test_discovery_limits_load_and_keeps_newest_clock(monkeypatch):
    stage = "/planning/scenario_planning/lane_driving/trajectory"
    metric = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/total_time/processing_time_ms"
    advertised = [("/clock", ["example/Clock"]), (stage, ["example/Trajectory"]),
                  (metric, ["example/Float64Stamped"]),
                  ("/planning/scenario_planning/x/debug/trajectory", ["example/Trajectory"]),
                  ("/planning/path_candidate/x", ["example/Trajectory"]),
                  ("/perception/x/debug/processing_time_ms", ["example/Float64Stamped"])]
    created = {}
    node = SimpleNamespace(get_topic_names_and_types=lambda: advertised,
                           create_subscription=lambda kind, topic, callback, qos: created.setdefault(topic, qos))
    monkeypatch.setattr(_sr, "get_message", lambda kind: object)
    p = Probe(node)
    p.discover()
    assert set(created) == {"/clock", stage, metric}
    assert created["/clock"].depth == 1
    assert created["/clock"].reliability == _sr.ReliabilityPolicy.BEST_EFFORT
    p.discover()
    assert len(p.subscriptions) == 3


# --- evaluate_report ---

def _passing_report():
    return {
        "command_inhibit_verified": True,
        "route": {"success": True, "code": 0, "message": "ok"},
        "autonomous_available_observations": 3,
        "topics": {
            "/planning/trajectory": {
                "messages": 50,
                "steady_age_sec": {"count": 10, "min": 0.05, "median": 0.4,
                                   "p95": 0.5, "max": 0.6},
            },
            "/carmaker/lidar/front/points": {"messages": 20, "valid_xyz_samples": 8},
            "/sensing/imu/imu_data": {"messages": 30},
        },
        "safety": {"vehicle_commands_observed": 0,
                   "autonomous_mode_seen": False, "max_speed_mps": 0.0},
        "children_exit": {"carmaker": 0, "autoware": 0, "hellocm": 0},
    }


def _break_readiness(report, case):
    topics = report["topics"]
    safety = report["safety"]
    if case == "error-present":
        report["error"] = "boom"
    elif case == "inhibit-false":
        report["command_inhibit_verified"] = False
    elif case == "inhibit-missing":
        del report["command_inhibit_verified"]
    elif case == "route-false":
        report["route"] = {"success": False}
    elif case == "route-missing":
        del report["route"]
    elif case == "lidar-zero":
        topics["/carmaker/lidar/front/points"]["valid_xyz_samples"] = 0
    elif case == "lidar-missing":
        del topics["/carmaker/lidar/front/points"]
    elif case == "imu-zero":
        topics["/sensing/imu/imu_data"]["messages"] = 0
    elif case == "trajectory-few":
        topics["/planning/trajectory"]["messages"] = 9
    elif case == "no-availability":
        report["autonomous_available_observations"] = 0
    elif case == "steady-none":
        topics["/planning/trajectory"]["steady_age_sec"] = None
    elif case == "steady-stale":
        topics["/planning/trajectory"]["steady_age_sec"]["p95"] = 1.5
    elif case == "steady-future":
        topics["/planning/trajectory"]["steady_age_sec"]["p95"] = -0.2
    elif case == "steady-min-old":
        topics["/planning/trajectory"]["steady_age_sec"]["min"] = -0.3
    elif case == "commands-seen":
        safety["vehicle_commands_observed"] = 1
    elif case == "mode-seen":
        safety["autonomous_mode_seen"] = True
    elif case == "motion":
        safety["max_speed_mps"] = 0.05
    else:
        raise AssertionError(case)


_READINESS_CASES = ["error-present", "inhibit-false", "inhibit-missing",
                    "route-false", "route-missing", "lidar-zero", "lidar-missing",
                    "imu-zero", "trajectory-few", "no-availability", "steady-none",
                    "steady-stale", "steady-future", "steady-min-old",
                    "commands-seen", "mode-seen", "motion"]


def test_evaluate_report_passing_sets_both():
    report = _passing_report()
    evaluate_report(report)
    assert report["readiness_passed"] is True
    assert report["cleanup_passed"] is True
    assert report["passed"] is True


@pytest.mark.parametrize("case", _READINESS_CASES)
def test_evaluate_report_readiness_breakers_fail_overall(case):
    report = _passing_report()
    _break_readiness(report, case)
    evaluate_report(report)
    assert report["readiness_passed"] is False, case
    assert report["cleanup_passed"] is True, case
    assert report["passed"] is False, case


def test_evaluate_report_readiness_true_plus_shutdown_crash_fails_overall():
    report = _passing_report()
    report["shutdown_segfault"] = True
    evaluate_report(report)
    assert report["readiness_passed"] is True
    assert report["cleanup_passed"] is False
    assert report["passed"] is False


@pytest.mark.parametrize("field,value", [
    ("office_cleanup_timeout", True), ("shutdown_segfault", True),
    ("cleanup_errors", ["autoware: SIGINT timeout"]),
])
def test_evaluate_report_cleanup_flags_fail_overall(field, value):
    report = _passing_report()
    report[field] = value
    evaluate_report(report)
    assert report["readiness_passed"] is True
    assert report["cleanup_passed"] is False
    assert report["passed"] is False


@pytest.mark.parametrize("exit_patch",
                         [{"carmaker": 1}, {"autoware": 137}, {"hellocm": None}, {}])
def test_evaluate_report_bad_child_exit_fails_cleanup_only(exit_patch):
    report = _passing_report()
    report["children_exit"] = {"carmaker": 0, "autoware": 0, "hellocm": 0}
    report["children_exit"].update(exit_patch)
    if not exit_patch:
        del report["children_exit"]["hellocm"]
    evaluate_report(report)
    assert report["readiness_passed"] is True
    assert report["cleanup_passed"] is False
    assert report["passed"] is False


def test_evaluate_report_cleanup_true_alone_does_not_fake_runtime_pass():
    report = {"children_exit": {"carmaker": 0, "autoware": 0, "hellocm": 0}}
    evaluate_report(report)
    assert report["cleanup_passed"] is True
    assert report["readiness_passed"] is False
    assert report["passed"] is False


# --- GPU child lifecycle (safety: never touch unowned processes) ---

_GPU_EXE = "/opt/ipg/movienx/linux64-15.0/bin/MovieNX.exe"


def _make_gpu_proc(root, pid, *, ppid, project, state="S",
                   exe_target=_GPU_EXE, argv=None, starttime="777777"):
    import os as _os
    d = Path(root) / str(pid)
    d.mkdir(parents=True, exist_ok=True)
    # Tokens after ") ": state(0) ppid(1) ... starttime(19).
    toks = [state, str(ppid), "1", "1", "0", "0", "0", "0", "0", "0",
            "0", "0", "0", "0", "0", "0", "0", "1", "0", str(starttime),
            "0", "0"]
    assert len(toks) > 19
    (d / "stat").write_text(f"{pid} (MovieNX.exe) " + " ".join(toks))
    exe_link = d / "exe"
    try:
        if exe_link.is_symlink() or exe_link.exists():
            exe_link.unlink()
    except FileNotFoundError:
        pass
    # Dangling symlink to the literal expected path; never executed.
    _os.symlink(exe_target, exe_link)
    if argv is None:
        argv = ["MovieNX.exe", "-mode", "GPUSensor",
                "-projectdir", str(project)]
    (d / "cmdline").write_bytes("\0".join(argv).encode() + b"\0")
    return d


def _patch_proc_root(monkeypatch, fake_root):
    import pathlib as _pl
    _real_path = _pl.Path

    def _fake_path(*args, **kwargs):
        if len(args) == 1 and str(args[0]) == "/proc" and not kwargs:
            return _real_path(fake_root)
        return _real_path(*args, **kwargs)

    monkeypatch.setattr(_sr, "Path", _fake_path)


def test_gpu_identity_accepts_exact_child(tmp_path):
    project = tmp_path / "proj"
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project, starttime="424242")
    assert _sr.gpu_child_identity(proc, 777, project) == "424242"


def test_gpu_identity_rejects_wrong_parent(tmp_path):
    project = tmp_path / "proj"
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=999,
                          project=project)
    assert _sr.gpu_child_identity(proc, 777, project) is None


def test_gpu_identity_rejects_zombie(tmp_path):
    project = tmp_path / "proj"
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project, state="Z")
    assert _sr.gpu_child_identity(proc, 777, project) is None


def test_gpu_identity_rejects_wrong_exe(tmp_path):
    project = tmp_path / "proj"
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project, exe_target="/usr/bin/other")
    assert _sr.gpu_child_identity(proc, 777, project) is None


@pytest.mark.parametrize("argv", [
    ["MovieNX.exe", "-projectdir", "/p"],  # missing -mode
    ["MovieNX.exe", "-mode", "GLSensor", "-projectdir", "/p"],  # wrong mode
    ["MovieNX.exe", "-mode", "GPUSensor", "-mode", "GPUSensor",
     "-projectdir", "/p"],  # duplicate -mode
    ["MovieNX.exe", "-mode"],  # dangling key, no value
])
def test_gpu_identity_rejects_mode_problems(tmp_path, argv):
    project = Path("/p")
    fixed = [a.replace("/p", str(project)) for a in argv]
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project, argv=fixed)
    assert _sr.gpu_child_identity(proc, 777, project) is None


@pytest.mark.parametrize("argv", [
    ["MovieNX.exe", "-mode", "GPUSensor"],  # missing projectdir
    ["MovieNX.exe", "-mode", "GPUSensor", "-projectdir", "/other"],
    ["MovieNX.exe", "-mode", "GPUSensor", "-projectdir", "/p",
     "-projectdir", "/p"],  # duplicate projectdir flag
    ["MovieNX.exe", "-mode", "GPUSensor", "-projectdir"],  # no value
])
def test_gpu_identity_rejects_projectdir_problems(tmp_path, argv):
    project = Path("/p")
    fixed = [a.replace("/p", str(project)) for a in argv]
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project, argv=fixed)
    assert _sr.gpu_child_identity(proc, 777, project) is None


def test_gpu_identity_missing_proc_returns_none(tmp_path):
    project = tmp_path / "proj"
    missing = tmp_path / "proc" / "99999"
    assert _sr.gpu_child_identity(missing, 777, project) is None


def test_gpu_identity_rejects_foreign_uid(tmp_path, monkeypatch):
    import os as _os
    project = tmp_path / "proj"
    proc = _make_gpu_proc(tmp_path / "proc", 1234, ppid=777,
                          project=project)
    real_uid = _os.getuid()
    monkeypatch.setattr(_sr.os, "getuid", lambda: real_uid + 1)
    assert _sr.gpu_child_identity(proc, 777, project) is None


def test_track_gpu_children_adopts_only_owned(tmp_path, monkeypatch):
    import os as _os
    project = tmp_path / "proj"
    root = tmp_path / "proc"
    _make_gpu_proc(root, 1111, ppid=777, project=project)  # owned
    _make_gpu_proc(root, 2222, ppid=999, project=project)  # wrong parent
    _make_gpu_proc(root, 3333, ppid=777, project=project,
                   exe_target="/usr/bin/other")  # wrong exe
    _patch_proc_root(monkeypatch, root)
    opened = []
    monkeypatch.setattr(_sr.os, "pidfd_open",
                        lambda pid: opened.append(pid) or (1000 + pid),
                        raising=False)
    closed = []
    monkeypatch.setattr(_sr.os, "close", lambda fd: closed.append(fd))
    handles = {3333: 9999}  # already known: must be skipped, never reopened
    _sr.track_gpu_children(777, project, handles)
    assert handles == {3333: 9999, 1111: 2111}
    assert opened == [1111]
    assert closed == []


def test_track_gpu_children_race_discards_and_closes(tmp_path, monkeypatch):
    project = tmp_path / "proj"
    root = tmp_path / "proc"
    _make_gpu_proc(root, 1111, ppid=777, project=project,
                   starttime="111")
    _patch_proc_root(monkeypatch, root)
    real_identity = _sr.gpu_child_identity
    calls = []

    def _flipping(proc, ppid, proj):
        ident = real_identity(proc, ppid, proj)
        calls.append(ident)
        # Second recheck after pidfd_open sees a reused PID/starttime.
        if len(calls) == 2:
            return "different-starttime"
        return ident

    monkeypatch.setattr(_sr, "gpu_child_identity", _flipping)
    monkeypatch.setattr(_sr.os, "pidfd_open", lambda pid: 5555, raising=False)
    closed = []
    monkeypatch.setattr(_sr.os, "close", lambda fd: closed.append(fd))
    handles = {}
    _sr.track_gpu_children(777, project, handles)
    assert handles == {}
    assert closed == [5555]


def test_stop_gpu_children_signals_owned_and_closes(monkeypatch):
    import signal as _sig
    import select as _sel
    sent = []
    monkeypatch.setattr(_sr.signal, "pidfd_send_signal",
                        lambda fd, sig: sent.append((fd, sig)), raising=False)
    monkeypatch.setattr(_sr.select, "select",
                        lambda r, w, x, t=0: (list(r), [], []))
    monkeypatch.setattr(_sr.time, "monotonic", lambda: 100.0)
    closed = []
    monkeypatch.setattr(_sr.os, "close", lambda fd: closed.append(fd))
    for name in ("kill", "killpg"):
        monkeypatch.setattr(_sr.os, name,
                            lambda *a, **k: (_ for _ in ()).throw(
                                AssertionError("must not kill")),
                            raising=False)
    handles = {111: 901, 222: 902}
    report = {}
    _sr.stop_gpu_children(handles, report)
    assert sorted(s for _, s in sent) == [_sig.SIGTERM, _sig.SIGTERM]
    assert sorted(f for f, _ in sent) == [901, 902]
    assert report["gpu_processes_owned"] == [111, 222]
    assert report["gpu_processes_remaining"] == []
    assert "cleanup_errors" not in report
    assert sorted(closed) == [901, 902]


def test_stop_gpu_children_tolerates_exited_and_reports_timeout(monkeypatch):
    import signal as _sig
    calls = []

    def _fake_send(fd, sig):
        calls.append((fd, sig))
        if fd == 901:
            raise ProcessLookupError("already exited")

    monkeypatch.setattr(_sr.signal, "pidfd_send_signal", _fake_send,
                        raising=False)
    # Nothing ever becomes readable: owned GPU PIDs stay alive past deadline.
    monkeypatch.setattr(_sr.select, "select",
                        lambda r, w, x, t=0: ([], [], []))
    timeline = iter([100.0, 100.0, 200.0])

    def _fake_mono():
        try:
            return next(timeline)
        except StopIteration:
            return 200.0

    monkeypatch.setattr(_sr.time, "monotonic", _fake_mono)
    closed = []
    monkeypatch.setattr(_sr.os, "close", lambda fd: closed.append(fd))
    monkeypatch.setattr(_sr.os, "kill",
                        lambda *a, **k: (_ for _ in ()).throw(
                            AssertionError("no stronger kill")),
                        raising=False)
    monkeypatch.setattr(_sr.os, "killpg",
                        lambda *a, **k: (_ for _ in ()).throw(
                            AssertionError("no stronger kill")),
                        raising=False)
    report = {}
    _sr.stop_gpu_children({111: 901, 222: 902}, report)
    assert sorted(f for f, _ in calls) == [901, 902]
    assert all(s == _sig.SIGTERM for _, s in calls)
    assert report["gpu_processes_owned"] == [111, 222]
    assert report["gpu_processes_remaining"] == [111, 222]
    assert any("did not exit after SIGTERM" in e
               for e in report.get("cleanup_errors", []))
    # Already-exited pidfd must not itself be an error; only the timeout is.
    assert not any("GPU 111" in e for e in report.get("cleanup_errors", []))
    assert sorted(closed) == [901, 902]
