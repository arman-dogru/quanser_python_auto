"""Closed-loop software-in-the-loop scenarios.

The *mission side* is the real runtime (guards -> core -> output gate). The
*world side* is a deliberately simple stand-in, NOT a QDrone model:

* vehicle: first-order tracking of the desired pose (time constant 0.6 s,
  0.5 m/s cap) while armed with Takeoff/Land high; constant-rate descent
  when Takeoff/Land drops in flight; motors cut on E-stop. This is a proxy
  for the Commander FSM (IDLE/ARMED/.../FLIGHT/AUTO_LAND/E-STOP), enough to
  close the loop, not to validate flight dynamics.
* OptiTrack: 100 Hz Motive-frame samples (inverse of the model's frame
  correction) with 0.5 mm noise, rigid body 0.07 m above the landing centre.
* camera + vision publisher: a pinhole consistent with the calibrated pixel
  map, 5 Hz (the Commander's 0.2 s camera rate), detection latency, misses
  and pixel noise. Observations are packed with the publisher's own
  ``struct.pack("<6d", ...)`` layout and delivered as raw TCP bytes, so the
  runtime's backlog drain and freshness checks are on the path.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .blocks import R_RSF_TO_MF, JoystickSample
from .config import DEFAULT, Config
from .mission_server import OptiTrackSample
from .protocol import VISION
from .runtime import MissionRuntime, RawInputs
from .safety import Mode

WALL0 = 1_760_000_000.0


@dataclass
class Scenario:
    name: str
    duration: float = 90.0
    pad_xy: tuple[float, float] = (0.35, -0.30)
    start_xy: tuple[float, float] = (-0.10, -0.85)
    start_yaw: float = 0.02
    mode: Mode = Mode.AUTOLAND
    arm_at: float = 6.0
    takeoff_at: float = 7.0
    land_at: float | None = None          # operator lowers Takeoff/Land
    estop_at: float | None = None
    joystick_lost_at: float | None = None
    vision_lost_at: float | None = None   # publisher stops sending
    vision_lost_when_stage: int | None = None
    tracking_lost_when_stage: int | None = None
    garbage_vision_at: float | None = None
    vision_latency: float = 0.06
    miss_probability: float = 0.02
    pixel_noise: float = 1.0
    seed: int = 1
    stop_when_landed: bool = True


SCENARIOS = {
    "nominal-autoland": Scenario("nominal-autoland"),
    "monitor-only": Scenario("monitor-only", duration=75.0, mode=Mode.MONITOR),
    "vision-loss-in-descent": Scenario("vision-loss-in-descent", duration=70.0, vision_lost_when_stage=3),
    "tracking-loss-in-centering": Scenario("tracking-loss-in-centering", duration=60.0, tracking_lost_when_stage=2),
    "estop-in-search": Scenario("estop-in-search", duration=30.0, estop_at=20.0),
    "joystick-disconnect": Scenario("joystick-disconnect", duration=30.0, joystick_lost_at=20.0),
    "malformed-vision": Scenario("malformed-vision", garbage_vision_at=10.0),
    "operator-manual-land": Scenario("operator-manual-land", duration=40.0, land_at=25.0),
}


@dataclass
class Vehicle:
    pos: np.ndarray
    yaw: float
    armed: bool = False
    flying: bool = False
    landing: bool = False
    killed: bool = False
    landed_after_flight: bool = False

    def step(self, dt: float, triggers, desired) -> None:
        arm, takeoff, estop = triggers[0] > 0.5, triggers[1] > 0.5, triggers[2] > 0.5
        if estop or self.killed:
            self.killed = True
            self.flying = False
            self.pos[2] = max(0.0, self.pos[2] - 2.0 * dt)
            return
        if not arm:
            self.flying = False
            self.pos[2] = max(0.0, self.pos[2] - 1.0 * dt)
            return
        if takeoff and not self.landing:
            self.flying = True
            target = np.array(desired[:3])
            v = (target - self.pos) / 0.6
            speed = np.linalg.norm(v)
            if speed > 0.5:
                v *= 0.5 / speed
            self.pos += v * dt
            self.yaw += (desired[3] - self.yaw) * dt / 0.6
        elif self.flying:
            self.landing = True
            self.pos[2] = max(0.0, self.pos[2] - 0.25 * dt)
            if self.pos[2] <= 0.0:
                self.flying = False
                self.landed_after_flight = True


@dataclass
class SimResult:
    scenario: Scenario
    t: np.ndarray
    signals: dict[str, np.ndarray]
    runtime: MissionRuntime
    vehicle: Vehicle
    events: dict[str, float] = field(default_factory=dict)
    block_io: dict[str, list] | None = None


def project_pad(cfg: Config, pad_xy, pos, yaw) -> tuple[float, float] | None:
    """Pixel of the pad centre, using the inverse of the mission's own map."""
    a = cfg.auto
    z = pos[2]
    if z < 0.2:
        return None
    cmap = a.map * (z / a.search_height)
    target = np.array(a.camera_optical_center_pixel) - np.linalg.solve(cmap, np.array(a.camera_offset_body))
    ang = yaw - a.vision_calibration_yaw
    c, s = math.cos(ang), math.sin(ang)
    rot_t = np.array([[c, s], [-s, c]])
    d = np.array(pad_xy) - pos[:2]
    px = target + np.linalg.solve(cmap, rot_t @ d)
    if not (0 <= px[0] <= cfg.model.image_width and 0 <= px[1] <= cfg.model.image_height):
        return None
    return float(px[0]), float(px[1])


SIGNALS = ("x", "y", "z", "dx", "dy", "dz", "stage", "abort", "ready", "fresh", "reason", "confirmed",
           "trig1", "trig2", "trig3", "trig4", "op_trig2", "latched", "vision_age", "tracking", "faults",
           "pix_ex", "pix_ey", "abort_bits")


def run(scenario: Scenario | str, cfg: Config = DEFAULT, record_block_io: bool = False) -> SimResult:
    sc = SCENARIOS[scenario] if isinstance(scenario, str) else scenario
    rng = np.random.default_rng(sc.seed)
    dt = cfg.model.step_size
    n = int(round(sc.duration / dt))
    rt = MissionRuntime(cfg, sc.mode)
    veh = Vehicle(np.array([sc.start_xy[0], sc.start_xy[1], 0.0]), sc.start_yaw)
    sig = {k: np.full(n, np.nan) for k in SIGNALS}
    pending: list[tuple[int, bytes]] = []   # (deliver_tick, bytes)
    frame_no = 0
    events: dict[str, float] = {}
    vision_dead = False
    tracking_dead = False
    io = {k: [] for k in ("grid", "confirm", "gate", "abort", "arbiter")} if record_block_io else None
    cam_period = int(round(0.2 / dt))
    opti_period = cfg.model.optitrack_decimation
    k_end = n
    for k in range(n):
        t = k * dt
        wall = WALL0 + t
        stage_now = sig["stage"][k - 1] if k else 0
        if sc.vision_lost_when_stage is not None and not vision_dead and stage_now >= sc.vision_lost_when_stage:
            vision_dead = True
            events["vision_lost"] = t
        if sc.vision_lost_at is not None and t >= sc.vision_lost_at and not vision_dead:
            vision_dead = True
            events["vision_lost"] = t
        if sc.tracking_lost_when_stage is not None and not tracking_dead and stage_now >= sc.tracking_lost_when_stage:
            tracking_dead = True
            events["tracking_lost"] = t
        # --- OptiTrack (100 Hz)
        opti = None
        if k % opti_period == 0 and not tracking_dead:
            rsf = veh.pos + np.array([0.0, 0.0, cfg.model.rigid_body_height_offset]) + rng.normal(0, 0.0005, 3)
            opti = OptiTrackSample(tuple(R_RSF_TO_MF @ rsf), tuple(R_RSF_TO_MF @ np.array([0.0, 0.0, veh.yaw])), 1.0)
        # --- joystick (100 Hz, "new" toggles per sample)
        joy = None
        if k % 10 == 0 and not (sc.joystick_lost_at is not None and t >= sc.joystick_lost_at):
            buttons = [0.0] * 32
            buttons[0] = 1.0 if (sc.estop_at is not None and t >= sc.estop_at) else 0.0
            buttons[1] = 1.0 if t >= sc.arm_at else 0.0
            up = t >= sc.takeoff_at and not (sc.land_at is not None and t >= sc.land_at)
            joy = JoystickSample(new=float((k // 10) % 2), buttons=tuple(buttons), x=-1.0, y=0.0, z=0.0,
                                 rx=0.0, ry=0.0, rz=1.0 if up else -1.0)
        # --- camera + publisher (5 Hz)
        if k % cam_period == 0 and not vision_dead and k > 0:
            frame_no += 1
            px = project_pad(cfg, sc.pad_xy, veh.pos, veh.yaw)
            if px is not None and rng.random() >= sc.miss_probability:
                u, v = px[0] + rng.normal(0, sc.pixel_noise), px[1] + rng.normal(0, sc.pixel_noise)
                rec = VISION.pack(1.0, u, v, float(rng.uniform(0.80, 0.97)), float(frame_no), wall)
            else:
                rec = VISION.pack(0.0, 0.0, 0.0, 0.0, float(frame_no), wall)
            pending.append((k + int(round(sc.vision_latency / dt)), rec))
        if sc.garbage_vision_at is not None and abs(t - sc.garbage_vision_at) < dt / 2:
            # One bad record per tick (a tick only ever uses the newest record),
            # then 3 stray bytes that break the 48-byte framing until resync.
            for j, bad in enumerate(((1.0, 300, 200, 0.9, 9e9, wall + 1), (1.0, 300, 200, 0.9, 9e9 + 1, wall - 5),
                                     (1.0, 300, 200, 7.0, 9e9 + 2, wall), (math.nan, 1, 1, 1, 9e9 + 3, wall),
                                     (2.0, 1, 1, 1, 9e9 + 4, wall))):
                pending.append((k + 1 + j, VISION.pack(*bad)))
            pending.append((k + 300, b"\x00\x01\x02"))
            events["garbage_injected"] = t
        data = b"".join(rec for due, rec in pending if due <= k)
        pending = [(due, rec) for due, rec in pending if due > k]
        raw = RawInputs(t, wall, opti, joy, data, 2.0, 0.0, wall - 0.004)
        rec = rt.tick(raw)
        if rt.vision_guard.resync_requested:
            # Stand-in for VisionClient.close()/reconnect: in-flight bytes are lost.
            rt.vision_guard.resync_requested = False
            pending = [(due, r) for due, r in pending if due > k + int(round(sc.vision_latency / dt))]
            events.setdefault("vision_resync", t)
        if rec is None:
            raise RuntimeError(f"runtime failed: {rt.failed}")
        out, dec = rec.outputs, rec.decision
        # In dry-run nothing is sent; the stand-in vehicle then follows what the
        # model would have sent, so the mission logic can still be exercised.
        pkt = dec.packet if dec.packet is not None else dec.candidate
        veh.step(dt, pkt.triggers, pkt.desired_pose)
        if io is not None:
            _record_block_io(io, rec)
        s = sig
        s["x"][k], s["y"][k], s["z"][k] = veh.pos
        s["dx"][k], s["dy"][k], s["dz"][k] = pkt.desired_pose[:3]
        s["stage"][k], s["abort"][k] = out.descent_stage, float(out.descent_abort)
        s["ready"][k], s["fresh"][k], s["reason"][k] = float(out.landing_ready), out.fresh_count, out.reason_code
        s["confirmed"][k] = float(out.confirmed_pad)
        s["trig1"][k], s["trig2"][k], s["trig3"][k], s["trig4"][k] = pkt.triggers
        s["op_trig2"][k] = out.operator_triggers[1]
        s["latched"][k] = float(out.autoland_latched)
        s["vision_age"][k] = out.vision_age
        s["tracking"][k] = pkt.is_tracking
        s["faults"][k] = len(dec.faults)
        s["pix_ex"][k], s["pix_ey"][k] = out.pixel_error
        s["abort_bits"][k] = out.abort_cause_bits
        if out.landing_ready and "landing_ready" not in events:
            events["landing_ready"] = t
        if veh.landed_after_flight and "landed" not in events:
            events["landed"] = t
            if sc.stop_when_landed:
                k_end = k + 1
                break
    sig = {key: val[:k_end] for key, val in sig.items()}
    return SimResult(sc, np.arange(k_end) * dt, sig, rt, veh, events, io)


def _record_block_io(io, rec) -> None:
    i, o = rec.inputs, rec.outputs
    trig = o.operator_triggers
    mp = o.measured_pose
    io["confirm"].append([*mp, *i.vision, float(i.new_vision), i.t, *trig])
    io["grid"].append([*mp, *o.home_pose, *trig, float(o.confirmed_pad), *o.hold_pose, o.pad_u, o.pad_v,
                       float(o.new_pad_sample), o.is_tracking])
    io["gate"].append([o.descent_stage, float(o.descent_abort), float(o.confirmed_pad), float(o.centered),
                       float(o.new_pad_sample), o.is_tracking, *mp, *o.desired_pose, i.t])
    io["abort"].append([o.descent_stage, float(o.descent_abort), float(o.confirmed_pad), o.is_tracking, *mp,
                        o.vision_age, i.t])
    io["arbiter"].append([*trig, float(o.landing_ready)])


__all__ = ["SCENARIOS", "Scenario", "SimResult", "Vehicle", "project_pad", "run"]
