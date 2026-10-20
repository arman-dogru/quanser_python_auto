"""``qlanding`` command-line interface.

Subcommands
    replay      run pose + detection logs through a controller (monitor-only)
    scenario    write synthetic pose/detection logs for trying the harness
    detect      YOLO on a folder of recorded frames -> detection JSONL
    udp-monitor listen on the Stage 6 detection port and print controller output
    udp-send    send a detection log to the Stage 6 detection port
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def _replay(a) -> int:
    from .replay import load_detections, load_poses, replay, summarize, write_rows

    rows = replay(load_poses(a.poses), load_detections(a.detections, rescale=a.rescale),
                  controller=a.controller, mode=a.mode, period_s=a.period)
    write_rows(rows, a.out)
    print(json.dumps({"out": str(a.out), **summarize(rows)}, indent=1, default=str))
    return 0


def _scenario(a) -> int:
    from .scenario import generate

    pose, det = generate(a.out_dir, seed=a.seed, duration_s=a.duration,
                         stop_sending_at=None if a.stop_sending_at < 0 else a.stop_sending_at)
    print(f"wrote {pose}\nwrote {det}\n(synthetic data, not a recording)")
    return 0


def _detect(a) -> int:
    from .detection.contract import write_jsonl
    from .detection.yolo import YoloPadDetector, frame_timestamps, list_frames

    frames = list_frames(a.frames)
    if not frames:
        print(f"no images in {a.frames}", file=sys.stderr)
        return 1
    ts = frame_timestamps(frames, fps=a.fps, timestamps_csv=a.timestamps)
    det = YoloPadDetector(Path(a.weights), classes=a.classes, conf_threshold=a.conf, imgsz=a.imgsz)
    n = write_jsonl(det.detect_frames(frames, ts), a.out)
    print(f"wrote {n} detections to {a.out} (classes={det.class_ids}, conf>={a.conf}, weights={a.weights})")
    return 0


def _udp_monitor(a) -> int:
    from .controllers import YOLOPoseCommandGenerator
    from .io.udp_detection import UdpDetectionReceiver

    rx = UdpDetectionReceiver(a.host, a.port)
    gen = YOLOPoseCommandGenerator()
    pose = tuple(a.pose)
    print(f"monitor-only: listening on {rx.address}, controller at {a.period}s; Ctrl-C to stop")
    t_end = time.monotonic() + a.seconds if a.seconds > 0 else float("inf")
    try:
        while time.monotonic() < t_end:
            s = rx.poll()
            out = gen.step(pose, *s.values)
            print(json.dumps({"new": s.new, "packet": s.values, "state": int(out.state),
                              "forward_cmd": out.forward_cmd, "right_cmd": out.right_cmd,
                              "cmd_pose": out.cmd_pose}))
            time.sleep(a.period)
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
    return 0


def _udp_send(a) -> int:
    from .io.udp_detection import UdpDetectionSender
    from .replay import load_detections

    recs = load_detections(a.detections, rescale=a.rescale)
    tx = UdpDetectionSender(a.host, a.port)
    t0, s0 = time.monotonic(), recs[0].timestamp_s if recs else 0.0
    for r in recs:
        if a.realtime:
            delay = (r.timestamp_s - s0) - (time.monotonic() - t0)
            if delay > 0:
                time.sleep(delay)
        tx.send(r.detected, r.error_x, r.error_y)
    tx.close()
    print(f"sent {len(recs)} packets to {a.host}:{a.port}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="qlanding", description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("replay", help="replay logs through a controller")
    r.add_argument("--poses", type=Path, required=True)
    r.add_argument("--detections", type=Path, required=True)
    r.add_argument("--out", type=Path, required=True)
    r.add_argument("--controller", choices=["stage6", "recenter"], default="stage6")
    r.add_argument("--mode", choices=["tick", "event"], default="tick")
    r.add_argument("--period", type=float, default=0.033)
    r.add_argument("--rescale", action="store_true", help="scale non-640x480 detections")
    r.set_defaults(func=_replay)

    s = sub.add_parser("scenario", help="write synthetic logs (not a recording)")
    s.add_argument("--out-dir", type=Path, default=Path("outputs/scenario"))
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--duration", type=float, default=20.0)
    s.add_argument("--stop-sending-at", type=float, default=15.0, help="-1: never stop")
    s.set_defaults(func=_scenario)

    d = sub.add_parser("detect", help="YOLO on recorded frames")
    d.add_argument("--frames", type=Path, required=True)
    d.add_argument("--weights", type=Path, required=True)
    d.add_argument("--classes", nargs="+", required=True, help="class names or ids to keep")
    d.add_argument("--conf", type=float, required=True, help="confidence threshold (not in the source)")
    d.add_argument("--fps", type=float)
    d.add_argument("--timestamps", type=Path, help="CSV frame,timestamp_s")
    d.add_argument("--imgsz", type=int, default=640)
    d.add_argument("--out", type=Path, required=True)
    d.set_defaults(func=_detect)

    m = sub.add_parser("udp-monitor", help="listen for Stage 6 detection packets (monitor-only)")
    m.add_argument("--host", default="127.0.0.1")
    m.add_argument("--port", type=int, default=5010)
    m.add_argument("--period", type=float, default=0.033)
    m.add_argument("--pose", type=float, nargs=4, default=[0.0, 0.0, 1.25, 0.0])
    m.add_argument("--seconds", type=float, default=0.0)
    m.set_defaults(func=_udp_monitor)

    u = sub.add_parser("udp-send", help="send a detection log as Stage 6 packets")
    u.add_argument("--detections", type=Path, required=True)
    u.add_argument("--host", default="127.0.0.1")
    u.add_argument("--port", type=int, default=5010)
    u.add_argument("--realtime", action="store_true")
    u.add_argument("--rescale", action="store_true")
    u.set_defaults(func=_udp_send)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
