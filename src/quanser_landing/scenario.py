"""Synthetic pose/detection logs for exercising the replay harness.

**Synthetic, not recorded.** No flight or bench logs were supplied with the
source. This generator produces plausible inputs (a hovering drone drifting
over a pad, with detector dropouts and a detector that stops sending) so the
harness, CLI and plots can be run end to end. It is not a vehicle model and
says nothing about real closed-loop behaviour.

Camera model (for the synthetic data only): the pad's offset in the body
frame maps to pixel errors using the chart's "calibration-confirmed" signs,
``error_x = -k * forward_offset`` and ``error_y = -k * right_offset``, with
``k`` pixels per metre at the hover height. The simulated drone moves a
fraction of the way to the commanded pose each tick, i.e. it behaves as if
``use_yolo`` were 1. Stage 6 itself never sends that pose (``use_yolo = 0``).
"""
from __future__ import annotations

import csv
import json
import math
import random
from pathlib import Path

from .controllers import YOLOPoseCommandGenerator
from .detection.contract import BoundingBox, Detection

PERIOD = 0.033


def generate(out_dir: Path, seed: int = 0, duration_s: float = 20.0, pad=(0.35, -0.25),
             px_per_m: float = 600.0, follow_gain: float = 0.5, dropout_p: float = 0.04,
             stop_sending_at: float | None = 15.0) -> tuple[Path, Path]:
    rng = random.Random(seed)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    gen = YOLOPoseCommandGenerator()
    x, y, z, yaw = 0.0, 0.0, 1.25, 0.3
    poses, dets = [], []
    t = 0.0
    while t <= duration_s:
        # body-frame offset of the pad
        dxw, dyw = pad[0] - x, pad[1] - y
        fwd = math.cos(yaw) * dxw + math.sin(yaw) * dyw
        rgt = -math.sin(yaw) * dxw + math.cos(yaw) * dyw
        ex = -px_per_m * fwd + rng.gauss(0, 3)
        ey = -px_per_m * rgt + rng.gauss(0, 3)
        visible = abs(ex) < 320 and abs(ey) < 240 and rng.random() > dropout_p
        sending = stop_sending_at is None or t < stop_sending_at
        if sending:
            if visible:
                cx, cy = 320 + ex, 240 + ey
                det = Detection(timestamp_s=round(t, 6), image_width=640, image_height=480, detected=True,
                                class_id=0, class_name="landing_pad (synthetic)", confidence=0.9,
                                box=BoundingBox(cx - 40, cy - 40, cx + 40, cy + 40), source="synthetic")
            else:
                det = Detection(timestamp_s=round(t, 6), image_width=640, image_height=480,
                                detected=False, source="synthetic")
            dets.append(det)
        poses.append((round(t, 6), x + rng.gauss(0, 0.001), y + rng.gauss(0, 0.001), z, yaw))
        d, e1, e2 = dets[-1].controller_inputs() if dets else (0.0, 0.0, 0.0)
        cmd = gen.step((x, y, z, yaw), d, e1, e2).cmd_pose
        x += follow_gain * (cmd[0] - x) + rng.gauss(0, 0.0005)
        y += follow_gain * (cmd[1] - y) + rng.gauss(0, 0.0005)
        t += PERIOD
    pose_path = out_dir / f"poses_seed{seed}.csv"
    det_path = out_dir / f"detections_seed{seed}.jsonl"
    with open(pose_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_s", "x", "y", "z", "yaw"])
        w.writerows(poses)
    with open(det_path, "w") as fh:
        for d in dets:
            fh.write(json.dumps(json.loads(d.to_json())) + "\n")
    return pose_path, det_path
