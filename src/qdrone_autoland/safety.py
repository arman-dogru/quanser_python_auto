"""Safety layer between the I/O and :class:`MissionServerCore`.

Every component here sits on the runtime path (``runtime.MissionRuntime.tick``
calls all of them on every tick; there is no code path that reaches the
Commander socket without passing through :class:`OutputGate`). Rules:

* Malformed, missing or stale inputs are refused: they are never presented
  to the mission logic as fresh, and they never raise.
* Pose: a non-finite or stale OptiTrack sample becomes ``tracking = 0`` with
  the last finite pose held. Non-finite numbers never reach the wire.
* Vision: the TCP backlog is drained and only the newest complete 48-byte
  record is considered; it must be finite, well-formed and no older than
  ``yoloStaleSeconds`` by its own publish timestamp.
* Joystick: a missing or malformed sample is presented to the model as a
  disconnected controller (all axes -1), which drives the model's own
  joystick-issue path (arm and takeoff forced to 0 after 0.2 s).
* Output: E-stop, arm and joystick-issue triggers are copied from the
  operator bit for bit in every mode. Desired poses outside the
  config-derived bounds are refused (last accepted pose held, fault raised).
  The autonomous LAND request is applied only in ``AUTOLAND`` mode.
"""
from __future__ import annotations

import enum
import math
from collections import Counter
from dataclasses import dataclass, field

from .blocks import JoystickSample
from .config import DEFAULT, Config
from .mission_server import OptiTrackSample, TickOutputs
from .protocol import VISION, VISION_DEFAULT, CommandPacket, RecordBuffer


class Mode(enum.Enum):
    DRY_RUN = "dry-run"      # compute everything, send nothing
    MONITOR = "monitor"      # send to the Commander, operator Takeoff/Land passes through (Phase 9 behaviour)
    AUTOLAND = "autoland"    # faithful Phase 10: the arbiter may lower Takeoff/Land


def _finite(values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


class PoseGuard:
    def __init__(self, cfg: Config = DEFAULT) -> None:
        self.stale_s = cfg.safety.pose_stale_s
        self.reset()

    def reset(self) -> None:
        self.last_good: OptiTrackSample | None = None
        self.last_time: float | None = None
        self.refused = Counter()
        self.lost = False

    def step(self, now: float, sample) -> OptiTrackSample | None:
        """Return the sample for the core (None = hold the previous one)."""
        if sample is not None:
            ok = False
            try:
                pos, ori, trk = tuple(sample.position), tuple(sample.orientation), float(sample.tracking)
                ok = len(pos) == 3 and len(ori) == 3 and _finite((*pos, *ori, trk))
            except (AttributeError, TypeError, ValueError):
                ok = False
            if ok:
                good = OptiTrackSample(tuple(map(float, pos)), tuple(map(float, ori)), 1.0 if trk > 0.5 else 0.0)
                self.last_good, self.last_time, self.lost = good, now, False
                return good
            self.refused["malformed"] += 1
            return self._lost_sample()
        if self.last_time is None or now - self.last_time > self.stale_s:
            if not self.lost:
                self.refused["stale"] += 1
            return self._lost_sample()
        return None

    def _lost_sample(self) -> OptiTrackSample:
        self.lost = True
        if self.last_good is None:
            return OptiTrackSample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        return OptiTrackSample(self.last_good.position, self.last_good.orientation, 0.0)


class VisionGuard:
    def __init__(self, cfg: Config = DEFAULT) -> None:
        self.max_age = cfg.auto.yolo_stale_seconds
        self.future_tol = cfg.safety.vision_future_tolerance_s
        self.width, self.height = cfg.model.image_width, cfg.model.image_height
        self.reset()

    def reset(self) -> None:
        self.buffer = RecordBuffer(VISION.size)
        self.vector = VISION_DEFAULT
        self.refused = Counter()
        self.accepted = 0
        self.consecutive_malformed = 0
        self.resyncs = 0
        self.resync_requested = False

    def classify(self, v, now: float) -> str | None:
        """Return a refusal reason, or None when the record may be used."""
        if not _finite(v):
            return "non-finite"
        detected, cx, cy, conf, frame, ts = v
        if detected not in (0.0, 1.0):
            return "bad-detected-flag"
        if frame < 0 or frame != math.floor(frame):
            return "bad-frame-number"
        if detected == 1.0 and not (0.0 <= conf <= 1.0):
            return "bad-confidence"
        age = now - ts
        if age < -self.future_tol:
            return "from-future"
        if age > self.max_age:
            return "stale"
        return None

    STRUCTURAL = ("non-finite", "bad-detected-flag", "bad-frame-number", "bad-confidence")
    RESYNC_AFTER = 3

    def step(self, now: float, data: bytes) -> tuple[tuple[float, ...], bool]:
        newest = self.buffer.feed(data) if data else None
        if newest is None:
            return self.vector, False
        v = VISION.unpack(newest)
        reason = self.classify(v, now)
        if reason is not None:
            self.refused[reason] += 1
            if reason in self.STRUCTURAL:
                self.consecutive_malformed += 1
                if self.consecutive_malformed >= self.RESYNC_AFTER:
                    # Persistent garbage means the 48-byte framing is lost.
                    # Drop the partial buffer and ask the caller to reconnect.
                    self.buffer = RecordBuffer(VISION.size)
                    self.consecutive_malformed = 0
                    self.resyncs += 1
                    self.resync_requested = True
            return self.vector, False
        self.consecutive_malformed = 0
        self.vector = v
        self.accepted += 1
        return v, True


class JoystickGuard:
    """Re-presents the last good Host Game Controller sample between updates;
    after ``joystick_stale_s`` without one (or on a malformed sample) the model
    sees a disconnected controller."""

    def __init__(self, cfg: Config = DEFAULT) -> None:
        self.stale_s = cfg.safety.joystick_stale_s
        self.reset()

    def reset(self) -> None:
        self.last_time: float | None = None
        self.last_sample: JoystickSample | None = None
        self.last_new = 0.0
        self.refused = Counter()

    def step(self, now: float, sample: JoystickSample | None) -> JoystickSample:
        if sample is not None:
            try:
                values = (sample.new, *sample.buttons, *sample.sliders, sample.x, sample.y, sample.z,
                          sample.rx, sample.ry, sample.rz)
                ok = _finite(values) and len(sample.buttons) == 32
            except (AttributeError, TypeError):
                ok = False
            if ok:
                self.last_time, self.last_sample, self.last_new = now, sample, float(sample.new)
                return sample
            self.refused["malformed"] += 1
            self.last_time, self.last_sample = None, None
            return JoystickSample.disconnected(self.last_new)
        if self.last_sample is None or self.last_time is None or now - self.last_time > self.stale_s:
            if self.last_sample is not None:
                self.refused["stale"] += 1
                self.last_sample = None
            return JoystickSample.disconnected(self.last_new)
        return self.last_sample


@dataclass
class GateDecision:
    packet: CommandPacket | None       # None in DRY_RUN
    candidate: CommandPacket           # what the model would have sent
    faults: list[str] = field(default_factory=list)
    autoland_applied: bool = False


class OutputGate:
    def __init__(self, cfg: Config = DEFAULT, mode: Mode = Mode.DRY_RUN) -> None:
        s, m, a = cfg.safety, cfg.model, cfg.auto
        self.mode = mode
        self.z_lo, self.z_hi = s.z_limits(m)
        (self.x_lo, self.x_hi), (self.y_lo, self.y_hi) = s.xy_limits(m)
        self.max_dz = s.max_dz(m)
        self.center_step = a.center_max_step
        self.max_disp = a.center_max_displacement
        self.eps = s.geofence_margin
        self.reset()

    def reset(self) -> None:
        self.last_desired: tuple[float, ...] | None = None
        self.last_measured: tuple[float, ...] | None = None
        self.prev_centering = False
        self.center_origin: tuple[float, float] | None = None
        self.faults = Counter()

    def _desired_ok(self, d, out: TickOutputs) -> str | None:
        if len(d) != 4 or not _finite(d):
            return "desired-non-finite"
        x, y, z, _ = d
        e = self.eps
        if not (self.x_lo - e <= x <= self.x_hi + e and self.y_lo - e <= y <= self.y_hi + e):
            return "desired-outside-geofence"
        if not (self.z_lo - e <= z <= self.z_hi + e):
            return "desired-height-out-of-range"
        if out.centering_active and self.prev_centering and self.last_desired is not None:
            px, py, pz, _ = self.last_desired
            if math.hypot(x - px, y - py) > self.center_step + e:
                return "centering-step-too-large"
            if abs(z - pz) > self.max_dz + e:
                return "height-step-too-large"
            if self.center_origin is not None and math.hypot(x - self.center_origin[0],
                                                             y - self.center_origin[1]) > self.max_disp + e:
                return "centering-displacement-too-large"
        return None

    def step(self, out: TickOutputs) -> GateDecision:
        faults: list[str] = []
        op = tuple(float(v) for v in out.operator_triggers)
        arb = tuple(float(v) for v in out.arbiter_triggers)
        if not all(v in (0.0, 1.0) for v in op):
            faults.append("operator-trigger-not-boolean")
            op = tuple(1.0 if v > 0.5 else 0.0 for v in op)
        # Arm, E-stop and joystick issue always come straight from the operator path.
        triggers = [op[0], op[1], op[2], op[3]]
        autoland = False
        if arb[1] == 0.0 and op[1] == 1.0 and op[0] == 1.0:
            if self.mode is Mode.AUTOLAND:
                triggers[1] = 0.0
                autoland = True
        if (arb[0], arb[2], arb[3]) != (op[0], op[2], op[3]):
            faults.append("arbiter-altered-protected-trigger")
        # Desired pose
        if out.centering_active and not self.prev_centering:
            self.center_origin = (float(out.desired_pose[0]), float(out.desired_pose[1]))
        if not out.centering_active:
            self.center_origin = None
        reason = self._desired_ok(out.desired_pose, out)
        if reason is None:
            desired = tuple(float(v) for v in out.desired_pose)
            self.last_desired = desired
        else:
            faults.append(reason)
            desired = self.last_desired
        self.prev_centering = bool(out.centering_active)
        # Measured pose / tracking
        tracking = 1.0 if out.is_tracking > 0.5 else 0.0
        if _finite(out.measured_pose):
            measured = tuple(float(v) for v in out.measured_pose)
            self.last_measured = measured
        else:
            faults.append("measured-non-finite")
            tracking = 0.0
            measured = self.last_measured
        if measured is None:
            measured = (0.0, 0.0, 0.0, 0.0)
            tracking = 0.0
        if desired is None:
            # Nothing valid has ever been produced: hold position on the
            # measured pose and report no tracking so the Commander refuses
            # to arm/fly (its FSM errors on Tracking_Issue).
            desired = (measured[0], measured[1], min(max(measured[2], self.z_lo), self.z_hi), measured[3])
            tracking = 0.0
        ts = float(out.packet.timestamp) if math.isfinite(out.packet.timestamp) else 0.0
        packet = CommandPacket(tuple(triggers), tracking, measured, desired, ts)
        for f in faults:
            self.faults[f] += 1
        return GateDecision(None if self.mode is Mode.DRY_RUN else packet, out.packet, faults, autoland)
