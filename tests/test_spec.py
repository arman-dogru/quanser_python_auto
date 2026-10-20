"""The committed model spec is fresh, and the facts the port relies on hold in it."""
from __future__ import annotations

import json
import re
import subprocess
import sys

import numpy as np
import pytest

from qdrone_autoland import protocol
from qdrone_autoland.config import DEFAULT

MS = "Mission_Server_QDrone_YOLO_Autonomous_Land"
CMD = "QDrone_YOLO_Direct_Stream_Commander"
TOP = "MISSION SERVER - LEADER/FOLLOWER with QBot 2e and QDrone"


@pytest.fixture(scope="module")
def ms(root):
    return json.loads((root / "docs" / "spec" / f"{MS}.json").read_text())


@pytest.fixture(scope="module")
def cmd(root):
    return json.loads((root / "docs" / "spec" / f"{CMD}.json").read_text())


def _src(spec, dst):
    return [ln["src"] for ln in spec["wiring"] if ln["dst"] == dst]


def test_spec_is_fresh(root):
    r = subprocess.run([sys.executable, str(root / "tools" / "mdl_spec.py"), "--check"], cwd=root,
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr


def test_phase10_trigger_path_wiring(ms):
    """Structural half of matlab/test_phase10_autoland_dry_run.m, on the parsed model."""
    arb = f"{TOP}/Autonomous LAND Trigger Arbiter"
    assert _src(ms, f"{arb}#in:2") == [f"{TOP}/Landing Readiness Gate#out:1"]
    assert _src(ms, f"{TOP}/Triggers (0,1) [4]#in:1") == [f"{arb}#out:1"]
    assert _src(ms, f"{arb}#in:1") == [f"{TOP}/JOYSTICK#out:1"]


def test_vision_and_commander_endpoints(ms, cmd):
    used = {e["uri"] for e in ms["endpoints"] if e.get("used")}
    assert f"tcpip://localhost:{protocol.VISION_PORT}" in used
    assert f"tcpip://localhost:{protocol.COMMAND_PORT}" in used
    text = json.dumps(cmd)
    assert f":{protocol.COMMAND_PORT}" in text
    assert f":{protocol.FRAME_PORT}" in text


def test_chart_constants_match_config(root):
    charts = root / "docs" / "spec" / MS / "charts"
    conf = (charts / "chart_65_YOLO_Confirmation_and_Hover_Hold.m").read_text()
    a = DEFAULT.auto
    assert f"vision(4)>={a.yolo_confidence:.6f}" in conf
    assert f"good<uint8({a.yolo_confirm_frames})" in conf
    assert f"bad<uint8({a.yolo_release_frames})" in conf
    assert f"clockTime-lastFresh>{a.yolo_stale_seconds:.6f}" in conf
    assert "vision(2)<=640" in conf and "vision(3)<=480" in conf
    grid = (charts / "chart_23_Grid_Search_Manager.m").read_text()
    for h in DEFAULT.model.stage_heights:
        assert f"{h:.2f}" in grid
    assert "0.45" in grid and "0.005" in grid


def test_config_mirrors_config_autonomous_m(root):
    src = (root / "matlab" / "config_autonomous.m").read_text()
    a = DEFAULT.auto
    def num(name):
        m = re.search(rf"cfg\.{name}\s*=\s*([-\d.]+)\s*;", src)
        assert m, name
        return float(m.group(1))
    assert num("searchHeight") == a.search_height
    assert num("arrivalTolerance") == a.arrival_tolerance
    assert num("dwellSamples") == a.dwell_samples
    assert num("yoloTcpPort") == a.yolo_tcp_port == protocol.VISION_PORT
    assert num("yoloConfidence") == a.yolo_confidence
    assert num("yoloStaleSeconds") == a.yolo_stale_seconds
    assert num("centerMaxStep") == a.center_max_step
    assert num("centerPixelTolerance") == a.center_pixel_tolerance
    assert num("visionCalibrationYaw") == a.vision_calibration_yaw
    np.testing.assert_allclose(a.vision_target_pixel, [252.845, 237.168], atol=5e-4)


@pytest.mark.octave
def test_waypoints_match_octave(root):
    r = subprocess.run(["octave-cli", "--no-gui", "--quiet", "--eval",
                        f"addpath('{root / 'matlab'}'); addpath('{root / 'tools' / 'octave' / 'shims'}', '-begin');"
                        "c=config_autonomous(); printf('%.17g,', c.waypoints');"],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    ref = np.array([float(v) for v in r.stdout.strip(",\n ").split(",")]).reshape(-1, 3)
    # linspace rounding differs by 1 ulp between numpy, Octave and MATLAB; the
    # model itself only ever sees mat2str(waypoints, 6) (see test_chart_waypoints).
    np.testing.assert_allclose(DEFAULT.auto.waypoints, ref, rtol=0, atol=1e-15)


def test_chart_waypoints_are_config_waypoints_at_6_digits(root):
    from qdrone_autoland.logic.grid_manager import GridSearchManager
    grid = (root / "docs" / "spec" / MS / "charts" / "chart_23_Grid_Search_Manager.m").read_text()
    m = re.search(r"wps=\[([^\]]+)\]", grid)
    assert m
    rows = [[float(v) for v in r.split()] for r in m.group(1).split(";")]
    np.testing.assert_array_equal(np.array(rows), GridSearchManager().wps)
