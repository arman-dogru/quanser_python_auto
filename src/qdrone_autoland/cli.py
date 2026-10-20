"""``qautoland``: simulate, loop back, replay or serve the Phase 10 mission server."""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import sys
import time
from pathlib import Path

from .config import DEFAULT
from .safety import Mode

MODES = {m.value: m for m in Mode}


def cmd_sim(args) -> int:
    from . import sim
    names = list(sim.SCENARIOS) if args.scenario == "all" else [args.scenario]
    report = {}
    for name in names:
        r = sim.run(name)
        s = r.signals
        report[name] = {
            "events": {k: round(v, 3) for k, v in r.events.items()},
            "final_position": [round(float(v), 3) for v in r.vehicle.pos],
            "pad": list(r.scenario.pad_xy),
            "max_stage": int(max(s["stage"])),
            "abort_bits": int(s["abort_bits"][-1]),
            "gate_faults": dict(r.runtime.gate.faults),
            "vision_refused": dict(r.runtime.vision_guard.refused),
            "vision_resyncs": r.runtime.vision_guard.resyncs,
            "pose_refused": dict(r.runtime.pose_guard.refused),
            "final_triggers": [s[f"trig{i}"][-1] for i in range(1, 5)],
        }
        if not args.json:
            print(f"{name}: {json.dumps(report[name])}")
    if args.json:
        print(json.dumps(report, indent=2))
    return 0


def cmd_loopback(args) -> int:
    """Everything on 127.0.0.1: fake publisher -> mission server -> fake Commander."""
    import numpy as np

    from .blocks import R_RSF_TO_MF, JoystickSample
    from .io.loopback import FakeCommander, FakePublisher
    from .io.serve import MissionServer
    from .io.sockets import CommanderServer, VisionClient
    from .io.sources import ScriptedJoystick, StaticPose
    from .mission_server import OptiTrackSample

    pub = FakePublisher(0)
    cmdsrv = CommanderServer("127.0.0.1", 0)
    fake = FakeCommander(cmdsrv.port)
    fake.start()
    rsf = np.array([0.0, -0.9, 0.0 + DEFAULT.model.rigid_body_height_offset])
    pose = StaticPose(OptiTrackSample(tuple(R_RSF_TO_MF @ rsf), (0.0, 0.0, 0.0), 1.0))

    def joy(t):
        b = [0.0] * 32
        b[1] = 1.0 if t > 6 else 0.0
        return JoystickSample(buttons=tuple(b), x=-1.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=-1.0)

    server = MissionServer(DEFAULT, MODES[args.mode], VisionClient(port=pub.port), cmdsrv, pose,
                           ScriptedJoystick(joy))
    frame = 0
    t_end = time.monotonic() + args.duration
    next_pub = 0.0
    while time.monotonic() < t_end:
        server.run(duration=server.k * DEFAULT.model.step_size + 0.05)
        now = time.monotonic()
        if now >= next_pub:
            frame += 1
            pub.publish(1, 300.0, 200.0, 0.9, frame, time.time())
            next_pub = now + 0.2
    fake.stop()
    st = server.stats
    out = {"ticks": st.ticks, "sent": st.sent, "send_dropped": st.send_dropped, "commander_packets": len(fake.packets),
           "catch_up_ticks": st.catch_up_ticks, "max_lag_ms": round(st.max_lag_s * 1e3, 3),
           "vision_accepted": server.runtime.vision_guard.accepted, "stopped": st.stopped_reason,
           "loopback_ms": None}
    if fake.packets:
        last = fake.packets[-1]
        out["last_packet"] = list(last.as_vector())
    if server.last is not None and math.isfinite(server.last.outputs.loopback_time):
        out["loopback_ms"] = round(server.last.outputs.loopback_time * 1e3, 3)
    server.close()
    pub.close()
    print(json.dumps(out, indent=2))
    return 0 if st.stopped_reason is None else 1


def cmd_replay(args) -> int:
    """Replay a publisher session log (logs/yolo_direct_fresh_*.csv) through the
    vision guard and the YOLO confirmation block, hovering at the search height."""
    from .logic.yolo_confirmation import YoloConfirmation
    from .protocol import VISION
    from .safety import VisionGuard
    a, m = DEFAULT.auto, DEFAULT.model
    guard = VisionGuard(DEFAULT)
    conf = YoloConfirmation(a.yolo_confidence, a.yolo_confirm_frames, a.yolo_release_frames, a.yolo_stale_seconds,
                            m.image_width, m.image_height)
    rows = list(csv.DictReader(Path(args.log).open(newline="", encoding="utf-8")))
    if not rows:
        print("empty log")
        return 1
    t0 = float(rows[0]["timestamp"])
    confirmed_at = None
    released = 0
    prev = False
    for r in rows:
        ts = float(r["timestamp"])
        det = int(r["detected"])
        cx, cy, cf = (_num(r, k) if det else 0.0 for k in ("centerX", "centerY", "confidence"))
        rec = VISION.pack(float(det), cx, cy, cf, float(r["frame"]), ts)
        v, new = guard.step(ts, rec)
        o = conf.step((0, 0, a.search_height, 0), v, new, ts - t0, (1, 1, 0, 0))
        if o.confirmed_pad and confirmed_at is None:
            confirmed_at = ts - t0
        if prev and not o.confirmed_pad:
            released += 1
        prev = o.confirmed_pad
    print(json.dumps({"rows": len(rows), "accepted": guard.accepted, "refused": dict(guard.refused),
                      "first_confirmation_s": confirmed_at, "releases": released}, indent=2))
    return 0


def _num(row, key) -> float:
    v = row.get(key, "")
    return float(v) if v not in ("", "NaN") else 0.0


def cmd_serve(args) -> int:
    from .io.serve import MissionServer
    from .io.sockets import CommanderServer, VisionClient
    mode = MODES[args.mode]
    if mode is Mode.AUTOLAND and not args.i_understand_autoland:
        print("refusing --mode autoland without --i-understand-autoland", file=sys.stderr)
        return 2
    print("WARNING: no NatNet (OptiTrack) or game-controller reader is implemented yet; without them the "
          "Commander sees isTracking=0 and a disconnected joystick, so it will not fly.", file=sys.stderr)
    server = MissionServer(DEFAULT, mode, VisionClient(args.vision_host, args.vision_port),
                           CommanderServer(args.bind, args.commander_port))
    try:
        st = server.run(duration=args.duration)
    except KeyboardInterrupt:
        st = server.stats
    finally:
        server.close()
    print(json.dumps(st.__dict__, indent=2, default=str))
    return 0 if st.stopped_reason is None else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="qautoland", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("sim", help="closed-loop SIL scenarios (stand-in vehicle and camera)")
    from . import sim
    p.add_argument("scenario", nargs="?", default="all", choices=["all", *sim.SCENARIOS])
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_sim)
    p = sub.add_parser("loopback", help="sockets end to end on 127.0.0.1 with fake peers")
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--mode", choices=list(MODES), default="monitor")
    p.set_defaults(fn=cmd_loopback)
    p = sub.add_parser("replay", help="replay a yolo_direct_fresh session CSV through guard + confirmation")
    p.add_argument("log")
    p.set_defaults(fn=cmd_replay)
    p = sub.add_parser("serve", help="real-time mission server (replaces the Simulink model)")
    p.add_argument("--mode", choices=list(MODES), default="monitor",
                   help="dry-run: send nothing; monitor (default): Phase 9 behaviour, the operator alone lands; "
                        "autoland: faithful Phase 10, the arbiter may command LAND")
    p.add_argument("--i-understand-autoland", action="store_true")
    p.add_argument("--vision-host", default="127.0.0.1")
    p.add_argument("--vision-port", type=int, default=DEFAULT.auto.yolo_tcp_port)
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--commander-port", type=int, default=18001)
    p.add_argument("--duration", type=float)
    p.set_defaults(fn=cmd_serve)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
