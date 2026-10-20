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


class GateStatus(enum.Enum):
    OK = "ok"
    CENTRING_REFUSED = "centring-refused"      # refusing centring poses, holding where the vehicle is
    CENTRING_FAULT = "centring-fault"          # refusals lasted too long: latched hold, no autonomous LAND


@dataclass
class GateDecision:
    packet: CommandPacket | None       # None in DRY_RUN
    candidate: CommandPacket           # what the model would have sent
    faults: list[str] = field(default_factory=list)
    autoland_applied: bool = False
    status: GateStatus = GateStatus.OK


class OutputGate:
    """Last line of defence before the Commander socket (see docs/SAFETY.md).

    Centring rules. Until the gate has accepted a centring pose in the
    current centring phase (the entry), the model's pose may be at most 0.5 m
    from where the vehicle *is* (see ``_vehicle_xy``). After that, every
    accepted centring pose is at most 5 mm from the previously accepted one,
    whatever was refused in between, and never more than 0.5 m from the
    centring origin. A refused centring pose holds the last accepted centring
    pose, or during the entry a validated measured position captured once
    (see ``_hold_pose``). Refusals that go on for
    ``centring_refusal_escalate_s`` latch ``CENTRING_FAULT`` until centring
    ends.

    Autonomous LAND. A *new* LAND is only applied while the gate status is
    OK. A LAND already applied is never interrupted by the gate: trigger 2
    stays at the model's LAND state (raising it mid-landing is an E-stop to
    the Commander) while bad poses are still refused and held.
    """

    def __init__(self, cfg: Config = DEFAULT, mode: Mode = Mode.DRY_RUN) -> None:
        s, m, a = cfg.safety, cfg.model, cfg.auto
        self.mode = mode
        self.z_lo, self.z_hi = s.z_limits(m)
        (self.x_lo, self.x_hi), (self.y_lo, self.y_hi) = s.xy_limits(m)
        g = s.hold_area_margin_m
        (self.fx_lo, self.fx_hi), (self.fy_lo, self.fy_hi) = ((a.x_limits[0] - g, a.x_limits[1] + g),
                                                              (a.y_limits[0] - g, a.y_limits[1] + g))
        self.center_step = a.center_max_step
        self.max_disp = a.center_max_displacement
        self.eps = s.geofence_margin
        self.measured_jump = s.hold_measured_jump_m
        self.escalate_ticks = max(1, int(round(s.centring_refusal_escalate_s / m.step_size)))
        self.reset()

    def reset(self) -> None:
        self.last_desired: tuple[float, ...] | None = None
        self.last_measured: tuple[float, ...] | None = None
        self.prev_centering = False
        self.center_origin: tuple[float, float] | None = None
        self.centring_last: tuple[float, ...] | None = None   # last accepted centring pose, this phase
        self.refusal_hold: tuple[float, ...] | None = None    # entry-phase hold, captured once
        self.refusal_ticks = 0               # consecutive refused centring ticks
        self.status = GateStatus.OK
        self.land_active = False             # an autonomous LAND has been applied and is still requested
        self.faults = Counter()
        self.centring_refusals = 0           # refused centring ticks, total
        self.escalations = 0

    def _measured_xy(self, out: TickOutputs) -> tuple[float, float] | None:
        """The measured xy if it is plausible: tracked, finite, inside the
        mission's flight area grown by ``hold_area_margin_m`` (the vehicle
        overshoots edge waypoints), within ``hold_measured_jump_m`` of the previous
        finite sample (one teleported OptiTrack frame is not trusted). An
        entry hold is captured on the tick the centring origin is set from
        it, so it is inside the origin's 0.5 m disc by construction."""
        m = tuple(out.measured_pose[:2])
        if not (out.is_tracking > 0.5 and len(m) == 2 and _finite(m)):
            return None
        x, y = float(m[0]), float(m[1])
        e = self.eps
        if not (self.fx_lo - e <= x <= self.fx_hi + e and self.fy_lo - e <= y <= self.fy_hi + e):
            return None
        if self.last_measured is not None and math.hypot(
                x - self.last_measured[0], y - self.last_measured[1]) > self.measured_jump + e:
            return None
        return x, y

    def _vehicle_xy(self, out: TickOutputs) -> tuple[float, float] | None:
        """Where the vehicle is: the plausible measured xy, else the last
        accepted desired pose, else None."""
        xy = self._measured_xy(out)
        if xy is not None:
            return xy
        if self.last_desired is not None:
            return self.last_desired[0], self.last_desired[1]
        return None

    def _desired_ok(self, d, out: TickOutputs) -> str | None:
        if len(d) != 4 or not _finite(d):
            return "desired-non-finite"
        x, y, z, _ = d
        e = self.eps
        if not (self.x_lo - e <= x <= self.x_hi + e and self.y_lo - e <= y <= self.y_hi + e):
            return "desired-outside-geofence"
        if not (self.z_lo - e <= z <= self.z_hi + e):
            return "desired-height-out-of-range"
        if not out.centering_active:
            return None
        # No separate height-step limit: z is already confined to [0.55, 0.70] m
        # above, and the model's abort hold legitimately steps 0.70 -> 0.55 m.
        if self.centring_last is None:
            # Entry: the model starts from its hover-hold pose, captured from
            # the measured pose, so compare with where the vehicle is (not the
            # last waypoint sent). Refuses e.g. a NaN hold pose clamped to the
            # flight-area corner by MATLAB min/max.
            ref = self._vehicle_xy(out)
            if ref is not None and math.hypot(x - ref[0], y - ref[1]) > self.max_disp + e:
                return "centering-entry-jump"
        else:
            px, py = self.centring_last[0], self.centring_last[1]
            if math.hypot(x - px, y - py) > self.center_step + e:
                return "centering-step-too-large"
        if self.center_origin is not None and math.hypot(
                x - self.center_origin[0], y - self.center_origin[1]) > self.max_disp + e:
            return "centering-displacement-too-large"
        return None

    def _origin(self, out: TickOutputs, accepted: bool) -> tuple[float, float] | None:
        """Centring origin for the 0.5 m displacement limit: the model's first
        centring pose when the gate accepts it (the model's own origin). When
        it is refused (e.g. a NaN hold pose: the model commands the flight-area
        corner with a NaN yaw, and its own 0.5 m limit becomes a no-op because
        NaN comparisons are false), where the vehicle is (see _vehicle_xy).
        Never a non-finite value; None means the first accepted pose sets it."""
        if accepted:
            return float(out.desired_pose[0]), float(out.desired_pose[1])
        return self._vehicle_xy(out)

    def _hold_pose(self, out: TickOutputs) -> tuple[float, ...] | None:
        """What a refused centring pose is replaced by. After the first
        accepted centring pose: that last accepted centring pose (validated,
        within 5 mm of what was sent before). During the entry: a hold
        captured once when the refusals start, at the plausible measured xy
        (not the last grid waypoint, which can be far away), else the last
        accepted pose; cleared when a pose is accepted."""
        if self.centring_last is not None:
            return self.centring_last
        if self.refusal_hold is None:
            xy = self._measured_xy(out)
            if xy is None:
                return self.last_desired
            if self.last_desired is not None:
                z, yaw = self.last_desired[2], self.last_desired[3]
            else:
                mz, myaw = float(out.measured_pose[2]), float(out.measured_pose[3])
                z = min(max(mz, self.z_lo), self.z_hi) if math.isfinite(mz) else self.z_hi
                yaw = myaw if math.isfinite(myaw) else 0.0
            self.refusal_hold = (xy[0], xy[1], z, yaw)
        return self.refusal_hold

    def step(self, out: TickOutputs) -> GateDecision:
        faults: list[str] = []
        op = tuple(float(v) for v in out.operator_triggers)
        arb = tuple(float(v) for v in out.arbiter_triggers)
        if not all(v in (0.0, 1.0) for v in op):
            faults.append("operator-trigger-not-boolean")
            op = tuple(1.0 if v > 0.5 else 0.0 for v in op)
        if (arb[0], arb[2], arb[3]) != (op[0], op[2], op[3]):
            faults.append("arbiter-altered-protected-trigger")
        # Desired pose
        centring = bool(out.centering_active)
        reason = self._desired_ok(out.desired_pose, out)
        if centring and not self.prev_centering:
            self.center_origin = self._origin(out, accepted=reason is None)
        if not centring:
            self.center_origin = None
            self.centring_last = None
            self.refusal_hold = None
            self.refusal_ticks = 0
            self.status = GateStatus.OK
        elif self.center_origin is None and reason is None:
            # No finite origin at entry (nothing accepted yet, measured invalid):
            # the first accepted centring pose becomes the origin.
            self.center_origin = (float(out.desired_pose[0]), float(out.desired_pose[1]))
        if centring and self.status is GateStatus.CENTRING_FAULT:
            reason = reason or "centring-fault-hold"
        if reason is None:
            desired = tuple(float(v) for v in out.desired_pose)
            self.last_desired = desired
            if centring:
                self.centring_last = desired      # from now on the hold (refusal_hold is entry-only)
                self.refusal_ticks = 0
                self.status = GateStatus.OK
        else:
            faults.append(reason)
            desired = self.last_desired
            if centring:
                self.centring_refusals += 1
                self.refusal_ticks += 1
                hold = self._hold_pose(out)
                if hold is not None:
                    desired = hold
                if self.status is GateStatus.OK:
                    self.status = GateStatus.CENTRING_REFUSED
                if self.refusal_ticks >= self.escalate_ticks and self.status is not GateStatus.CENTRING_FAULT:
                    self.status = GateStatus.CENTRING_FAULT
                    self.escalations += 1
                    faults.append("centring-refusals-escalated")
        self.prev_centering = centring
        # Triggers. Arm, E-stop and joystick issue always come straight from
        # the operator path. Takeoff/Land is the operator's, except that in
        # AUTOLAND the arbiter may lower it while the operator has arm and
        # takeoff selected: a new LAND only while the gate status is OK; a
        # LAND already applied continues (never raised again by the gate).
        triggers = [op[0], op[1], op[2], op[3]]
        autoland = False
        if arb[1] == 0.0 and op[1] == 1.0 and op[0] == 1.0:
            if self.mode is Mode.AUTOLAND and (self.land_active or self.status is GateStatus.OK):
                triggers[1] = 0.0
                autoland = True
        self.land_active = autoland
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
        return GateDecision(None if self.mode is Mode.DRY_RUN else packet, out.packet, faults, autoland, self.status)
