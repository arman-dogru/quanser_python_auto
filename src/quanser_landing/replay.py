"""Replay recorded pose and detection logs through a controller.

Two timing modes:

``tick`` (default, model-faithful)
    The controller runs every 0.033 s, the rate at which Stage 6 samples
    both its UDP detection stream and (through a rate transition) the
    measured pose. Each tick uses the most recent detection *as the Stream
    Server would*: the last received sample is reused until a newer one
    arrives, and ``[0 0 0]`` is used before the first one. The pose is the
    most recent sample (zeros before the first, the rate transition's initial
    condition). This reproduces a property of the model worth knowing: a
    detector that stops sending leaves the controller acting on its last
    packet indefinitely (``detection_age_s`` shows how old it is).

``event``
    One controller step per detection record, with the most recent pose.

Log formats
    poses: CSV with header ``timestamp_s,x,y,z,yaw`` (measured pose after
    the model's measurement conditioning).
    detections: JSONL of :class:`~quanser_landing.detection.Detection`, or CSV
    with header ``timestamp_s,detected,error_x,error_y`` (packet values).
"""
from __future__ import annotations

import bisect
import csv
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from .controllers import LandingPadRecenterLogic, YOLOPoseCommandGenerator
from .detection.contract import Detection

STAGE6_PERIOD_S = 0.033


@dataclass(frozen=True)
class PacketRecord:
    timestamp_s: float
    detected: float
    error_x: float
    error_y: float


@dataclass(frozen=True)
class PoseRecord:
    timestamp_s: float
    pose: tuple[float, float, float, float]


def load_poses(path: Path) -> list[PoseRecord]:
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    out = [PoseRecord(float(r["timestamp_s"]), (float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])))
           for r in rows]
    return sorted(out, key=lambda p: p.timestamp_s)


def load_detections(path: Path, rescale: bool = False) -> list[PacketRecord]:
    path = Path(path)
    out: list[PacketRecord] = []
    if path.suffix == ".jsonl":
        with open(path) as fh:
            for line in fh:
                if line.strip():
                    det = Detection.from_dict(json.loads(line))
                    out.append(PacketRecord(det.timestamp_s, *det.controller_inputs(rescale=rescale)))
    else:
        with open(path, newline="") as fh:
            for r in csv.DictReader(fh):
                out.append(PacketRecord(float(r["timestamp_s"]), float(r["detected"]),
                                        float(r["error_x"]), float(r["error_y"])))
    return sorted(out, key=lambda d: d.timestamp_s)


def _latest(times: Sequence[float], t: float) -> int | None:
    i = bisect.bisect_right(times, t + 1e-9) - 1  # tolerate tick rounding
    return i if i >= 0 else None


COLUMNS = ["t", "detected", "error_x", "error_y", "detection_new", "detection_age_s",
           "x", "y", "z", "yaw", "cmd_x", "cmd_y", "cmd_z", "cmd_yaw",
           "forward_cmd", "right_cmd", "descend_cmd", "state"]


def replay(poses: list[PoseRecord], detections: list[PacketRecord], controller: str = "stage6",
           mode: str = "tick", period_s: float = STAGE6_PERIOD_S,
           t_start: float | None = None, t_end: float | None = None) -> list[dict]:
    if controller not in ("stage6", "recenter"):
        raise ValueError("controller must be 'stage6' or 'recenter'")
    gen = YOLOPoseCommandGenerator() if controller == "stage6" else LandingPadRecenterLogic()
    ptimes = [p.timestamp_s for p in poses]
    dtimes = [d.timestamp_s for d in detections]
    all_t = ptimes + dtimes
    if not all_t:
        return []
    if mode == "tick":
        t0 = min(all_t) if t_start is None else t_start
        t1 = max(all_t) if t_end is None else t_end
        n = int(math.floor((t1 - t0) / period_s + 1e-9)) + 1
        ticks = [t0 + k * period_s for k in range(n)]
    elif mode == "event":
        ticks = list(dtimes)
    else:
        raise ValueError("mode must be 'tick' or 'event'")

    rows = []
    last_det_idx = None
    for t in ticks:
        di = _latest(dtimes, t)
        if di is None:
            pkt, age = (0.0, 0.0, 0.0), math.nan
        else:
            d = detections[di]
            pkt, age = (d.detected, d.error_x, d.error_y), t - d.timestamp_s
        new = di is not None and di != last_det_idx
        last_det_idx = di
        pi = _latest(ptimes, t)
        pose = poses[pi].pose if pi is not None else (0.0, 0.0, 0.0, 0.0)
        row = {"t": t, "detected": pkt[0], "error_x": pkt[1], "error_y": pkt[2],
               "detection_new": int(new), "detection_age_s": age,
               "x": pose[0], "y": pose[1], "z": pose[2], "yaw": pose[3]}
        if controller == "stage6":
            out = gen.step(pose, *pkt)
            row.update(cmd_x=out.cmd_pose[0], cmd_y=out.cmd_pose[1], cmd_z=out.cmd_pose[2],
                       cmd_yaw=out.cmd_pose[3], forward_cmd=out.forward_cmd, right_cmd=out.right_cmd,
                       descend_cmd=0.0, state=int(out.state))
        else:
            out = gen.step(*pkt)
            row.update(cmd_x=math.nan, cmd_y=math.nan, cmd_z=math.nan, cmd_yaw=math.nan,
                       forward_cmd=out.forward_cmd, right_cmd=out.right_cmd,
                       descend_cmd=out.descend_cmd, state=int(out.state))
        rows.append(row)
    return rows


def write_rows(rows: list[dict], path: Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (repr(v) if isinstance(v, float) else v) for k, v in r.items()})


def summarize(rows: list[dict]) -> dict:
    if not rows:
        return {"ticks": 0}
    states = [r["state"] for r in rows]
    ages = [r["detection_age_s"] for r in rows if not math.isnan(r["detection_age_s"])]
    return {
        "ticks": len(rows),
        "state_counts": {s: states.count(s) for s in sorted(set(states))},
        "first_state3_t": next((r["t"] for r in rows if r["state"] == 3), None),
        "max_detection_age_s": max(ages) if ages else None,
        "max_abs_forward_cmd": max(abs(r["forward_cmd"]) for r in rows),
        "max_abs_right_cmd": max(abs(r["right_cmd"]) for r in rows),
    }
