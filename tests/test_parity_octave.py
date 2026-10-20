"""Parity of the Python ports with the extracted MATLAB charts run in Octave.

Fixtures come from ``tools/make_parity_fixtures.py``. Comparison is exact
(bit-for-bit, NaN == NaN) unless stated otherwise.
"""
from __future__ import annotations

import hashlib

import pytest
from conftest import ROOT, load_fixture, same

from quanser_landing.blocks import MatlabIndexError, detect_stream_connection_loss, extract_object_data
from quanser_landing.controllers import LandingPadRecenterLogic, PoseCommandConfig, YOLOPoseCommandGenerator, legacy

POSE = load_fixture("pose_command")
POSE_DESCENT = load_fixture("pose_command_descent_enabled")
RECENTER = load_fixture("recenter")
EXTRACT = load_fixture("extract_object_data")
COMMLOSS = load_fixture("comm_loss")
LEGACY = load_fixture("legacy")


def _sha(rel: str) -> str:
    return hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()


@pytest.mark.parametrize("fx", [POSE, RECENTER, EXTRACT, COMMLOSS])
def test_fixture_source_is_current(fx):
    assert _sha(fx["source"]) == fx["source_sha256"]


def _run_pose(cases, config):
    mismatches = []
    for case in cases:
        gen = YOLOPoseCommandGenerator(config=config)
        for k, (inp, exp) in enumerate(zip(case["inputs"], case["expected"], strict=True)):
            out = gen.step(inp[0:4], inp[4], inp[5], inp[6])
            got = [*out.cmd_pose, out.forward_cmd, out.right_cmd, float(out.state)]
            if not all(same(g, e) for g, e in zip(got, exp, strict=True)):
                mismatches.append((case["name"], k, got, exp))
    return mismatches


def test_pose_command_parity_bit_exact():
    assert _run_pose(POSE["cases"], PoseCommandConfig()) == []


def test_pose_command_dormant_descent_branch_parity():
    assert _run_pose(POSE_DESCENT["cases"], PoseCommandConfig(enable_descent=True)) == []


def test_pose_fixture_covers_required_behaviour():
    names = " ".join(c["name"] for c in POSE["cases"])
    for needle in ("sign", "deadband", "smoothing", "loss decay", "ten updates", "yaw rotation", "random"):
        assert needle in names
    states = {e[6] for c in POSE["cases"] for e in c["expected"]}
    assert states == {0.0, 1.0, 2.0, 3.0}


def test_recenter_parity_bit_exact():
    mismatches = []
    for case in RECENTER["cases"]:
        ctl = LandingPadRecenterLogic()
        for k, (inp, exp) in enumerate(zip(case["inputs"], case["expected"], strict=True)):
            out = ctl.step(*inp)
            got = [out.forward_cmd, out.right_cmd, out.descend_cmd, float(out.state)]
            if not all(same(g, e) for g, e in zip(got, exp, strict=True)):
                mismatches.append((case["name"], k, got, exp))
    assert mismatches == []


def test_extract_object_data_parity():
    for row in EXTRACT["rows"]:
        data = EXTRACT["data"][row["data"]]
        if row["expected"] == "ERR":
            with pytest.raises(MatlabIndexError):
                extract_object_data(data, row["index"])
        else:
            o = extract_object_data(data, row["index"])
            got = [*o.position, *o.orientation, o.tracking]
            assert all(same(g, e) for g, e in zip(got, row["expected"], strict=True)), row


def test_comm_loss_parity():
    for row in COMMLOSS["rows"]:
        assert detect_stream_connection_loss(*row["inputs"]) == row["expected"][0], row


def test_legacy_config_effective_values():
    ours = legacy.legacy_config()
    assert set(ours) == set(LEGACY["config"])
    for k, v in LEGACY["config"].items():
        assert float(ours[k]) == v, k


@pytest.mark.parametrize("kind", ["waypoint", "align", "descend"])
def test_legacy_controllers_parity(kind):
    p = legacy.legacy_config()
    for row in LEGACY[kind]:
        i = row["inputs"]
        if kind == "waypoint":
            cmd, flag = legacy.waypoint(i[0:3], i[3:6], p)
        elif kind == "align":
            cmd, flag = legacy.align_to_pad((int(i[0]), int(i[1])), i[2:4], p)
        else:
            cmd, flag = legacy.descend(i[4:7], (int(i[0]), int(i[1])), i[2:4], p)
        got = [*cmd, float(flag)]
        assert all(same(g, e) for g, e in zip(got, row["expected"], strict=True)), (kind, row, got)


def test_legacy_grid_parity():
    p = legacy.legacy_config()
    for row in LEGACY["grid"]:
        i = row["inputs"]
        q = {**p, "xMin": i[0], "xMax": i[1], "yMin": i[2], "yMax": i[3],
             "gridSpacing": i[4], "searchHeight": i[5]}
        pts = legacy.generate_grid_waypoints(q)
        exp = row["expected"]
        assert len(pts) == int(exp[0]), row
        flat = [v for pt in pts for v in pt]
        assert all(same(g, e) for g, e in zip(flat, exp[1:], strict=True)), (row, flat)


def test_shared_chart_copies_are_identical():
    """One Python class covers every model that embeds the same chart."""
    ex = ROOT / "reference" / "extracted"
    groups = [
        ["QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/chart_51_YOLOPoseCommandGenerator.m",
         "QDrone_YOLO_Recenter_Integration_Test/chart_52_YOLOPoseCommandGenerator.m",
         "QDrone_YOLO_Pose_Command_Test/chart_13_YOLOPoseCommandGenerator.m"],
        ["QDrone_Recenter_Logic_Test/chart_12_LandingPadRecenterLogic.m",
         "QDrone_Recenter_CSV_Test/chart_12_LandingPadRecenterLogic.m"],
        ["QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/chart_13_fcn.m",
         "QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/chart_23_fcn.m",
         "QDrone_YOLO_Recenter_Integration_Test/chart_14_fcn.m",
         "QDrone_YOLO_Recenter_Integration_Test/chart_24_fcn.m"],
        ["QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/chart_33_fcn.m",
         "QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/chart_42_fcn.m",
         "QDrone_YOLO_Recenter_Integration_Test/chart_34_fcn.m",
         "QDrone_YOLO_Recenter_Integration_Test/chart_43_fcn.m"],
    ]
    for g in groups:
        assert len({(ex / p).read_bytes() for p in g}) == 1, g
