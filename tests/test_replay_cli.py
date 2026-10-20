from __future__ import annotations

import csv
import json
import math

from quanser_landing.cli import main
from quanser_landing.controllers import YOLOPoseCommandGenerator
from quanser_landing.replay import PacketRecord, PoseRecord, load_detections, load_poses, replay, summarize


def test_tick_replay_holds_last_packet_like_stream_server():
    poses = [PoseRecord(0.0, (0.0, 0.0, 1.25, 0.0))]
    dets = [PacketRecord(0.0, 1, 0, 0)]  # one centred packet, then silence
    rows = replay(poses, dets, mode="tick", t_end=0.5)
    assert len(rows) == 16
    assert rows[-1]["state"] == 3  # stale packet keeps counting up
    assert rows[-1]["detection_age_s"] > 0.45
    assert [r["detection_new"] for r in rows[:3]] == [1, 0, 0]


def test_tick_replay_default_before_first_packet():
    rows = replay([PoseRecord(0.0, (0, 0, 1, 0))], [PacketRecord(0.1, 1, 100, 0)], mode="tick")
    assert rows[0]["detected"] == 0.0 and math.isnan(rows[0]["detection_age_s"])


def test_event_replay_matches_direct_controller():
    poses = [PoseRecord(0.0, (0.1, 0.2, 1.0, 0.3)), PoseRecord(1.0, (0.2, 0.2, 1.0, 0.3))]
    dets = [PacketRecord(0.5, 1, 120, -80), PacketRecord(1.5, 0, 0, 0), PacketRecord(2.0, 1, -50, 50)]
    rows = replay(poses, dets, mode="event")
    g = YOLOPoseCommandGenerator()
    expected = [g.step((0.1, 0.2, 1.0, 0.3), 1, 120, -80), g.step((0.2, 0.2, 1.0, 0.3), 0, 0, 0),
                g.step((0.2, 0.2, 1.0, 0.3), 1, -50, 50)]
    for r, e in zip(rows, expected, strict=True):
        assert (r["cmd_x"], r["cmd_y"], r["state"]) == (e.cmd_pose[0], e.cmd_pose[1], int(e.state))


def test_recenter_replay():
    rows = replay([PoseRecord(0, (0, 0, 1, 0))], [PacketRecord(0, 1, 0, 0)], controller="recenter", t_end=0.4)
    assert rows[-1]["descend_cmd"] == 0.05 and math.isnan(rows[-1]["cmd_x"])


def test_cli_scenario_and_replay(tmp_path):
    assert main(["scenario", "--out-dir", str(tmp_path), "--duration", "3", "--stop-sending-at", "2"]) == 0
    poses = load_poses(tmp_path / "poses_seed0.csv")
    dets = load_detections(tmp_path / "detections_seed0.jsonl")
    assert len(poses) > 80 and len(dets) > 50
    out = tmp_path / "replay.csv"
    assert main(["replay", "--poses", str(tmp_path / "poses_seed0.csv"),
                 "--detections", str(tmp_path / "detections_seed0.jsonl"), "--out", str(out)]) == 0
    with open(out) as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == len(poses)
    summ = summarize(replay(poses, dets))
    assert summ["max_detection_age_s"] > 0.9  # sender stopped at 2 s of 3 s
    json.dumps(summ)


def test_detection_csv_log(tmp_path):
    p = tmp_path / "d.csv"
    p.write_text("timestamp_s,detected,error_x,error_y\n0.0,1,10,20\n0.1,0,0,0\n")
    assert [d.detected for d in load_detections(p)] == [1.0, 0.0]
