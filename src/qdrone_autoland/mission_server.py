"""One tick of the Phase 10 mission server, wired exactly as the model.

:class:`MissionServerCore` is pure: every input (time, OptiTrack sample,
joystick sample, vision packet, stream status) is passed in, and the output
is the 14-double Commander packet plus every diagnostic the model displays.
The I/O, real-time loop and safety layer live in ``runtime.py``/``safety.py``.
Block execution order follows the data dependencies in the model:

Data Server -> Extract Vehicle Data -> Measurement Conditioning -> JOYSTICK
-> Calculate Home Pose -> YOLO Confirmation -> Grid Search Manager ->
Landing Readiness Gate -> Abort Diagnostic -> Autonomous LAND Arbiter.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .blocks import HomePose, Joystick, JoystickSample, MeasurementConditioning, MissionCommIssue, motive_to_rsf
from .config import DEFAULT, Config
from .logic import (
    AbortDiagnostic,
    AutolandArbiter,
    GridSearchManager,
    LandingReadinessGate,
    YoloConfirmation,
    extract_object_data,
)
from .protocol import VISION_DEFAULT, CommandPacket


@dataclass(frozen=True)
class OptiTrackSample:
    """OptiTrack Trackables outputs for the configured rigid body, Motive frame."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float]
    tracking: float


@dataclass
class TickInputs:
    t: float                                   # model time (s); drives the Clock blocks
    wall_time: float                           # QUARC "Time" (Wall clock) for the timestamp
    optitrack: OptiTrackSample | None          # None: no new sample this tick (held)
    joystick: JoystickSample
    vision: tuple[float, ...] = VISION_DEFAULT
    new_vision: bool = False
    stream_state: float = 0.0                  # Commander stream status
    stream_error: float = 0.0
    loopback_timestamp: float = 0.0


@dataclass
class TickOutputs:
    packet: CommandPacket                      # what the model sends to the Commander
    operator_triggers: tuple[float, float, float, float]
    arbiter_triggers: tuple[float, float, float, float]
    measured_pose: tuple[float, float, float, float]
    is_tracking: float
    home_pose: tuple[float, float, float, float]
    confirmed_pad: bool
    hold_pose: tuple[float, float, float, float]
    pad_u: float
    pad_v: float
    new_pad_sample: bool
    vision_age: float
    confidence: float
    desired_pose: tuple[float, float, float, float]
    waypoint_index: float
    grid_complete: bool
    visited_count: float
    centering_active: bool
    centered: bool
    pixel_error: tuple[float, float]
    descent_stage: float
    descent_abort: bool
    landing_ready: bool
    fresh_count: float
    reason_code: int
    abort_cause_bits: float
    abort_vision_age: float
    abort_time: float
    autoland_latched: bool
    joystick_issue: float
    comm_issue: float
    comm_issue_shown: float
    loopback_time: float


class MissionServerCore:
    def __init__(self, cfg: Config = DEFAULT) -> None:
        a, m = cfg.auto, cfg.model
        self.cfg = cfg
        self.rigid_body_index = 1                 # "Vehicle ID - QDrone" constant
        self.conditioning = MeasurementConditioning(cfg)
        self.joystick = Joystick(cfg)
        self.home = HomePose(cfg)
        self.confirm = YoloConfirmation(a.yolo_confidence, a.yolo_confirm_frames, a.yolo_release_frames,
                                        a.yolo_stale_seconds, m.image_width, m.image_height)
        self.grid = GridSearchManager(cfg)
        self.gate = LandingReadinessGate(m.gate_fresh_frames, m.gate_xy_tolerance, m.gate_height,
                                         m.gate_height_tolerance, m.gate_stale_seconds)
        self.abort = AbortDiagnostic(m.abort_diag_floor)
        self.arbiter = AutolandArbiter()
        self.comm = MissionCommIssue(cfg)
        self.reset()

    def reset(self) -> None:
        for block in (self.conditioning, self.joystick, self.home, self.confirm, self.grid, self.gate,
                      self.abort, self.arbiter, self.comm):
            block.reset()
        # Rate Transition3 (100 Hz -> 1 kHz) InitialCondition 0
        self.data7 = (0.0,) * 7

    def step(self, x: TickInputs) -> TickOutputs:
        if x.optitrack is not None:
            xyz, rot = motive_to_rsf(x.optitrack.position, x.optitrack.orientation)
            self.data7 = (*map(float, xyz), *map(float, rot), float(x.optitrack.tracking))
        pos, ori, trk = extract_object_data(self.data7, self.rigid_body_index)
        cond = self.conditioning.step((*pos, *ori, trk))
        mp = cond.measured_pose
        joy = self.joystick.step(x.t, x.joystick)
        trig = joy.triggers
        home = self.home.step(x.t, mp, trig[0])
        c = self.confirm.step(mp, x.vision, x.new_vision, x.t, trig)
        g = self.grid.step(mp, home, trig, c.confirmed_pad, c.hold_pose, c.pad_u, c.pad_v, c.new_pad_sample,
                           cond.is_tracking)
        gate = self.gate.step(g.descent_stage, g.descent_abort, c.confirmed_pad, g.centered, c.new_pad_sample,
                              cond.is_tracking, mp, g.desired_pose, x.t)
        ab = self.abort.step(g.descent_stage, g.descent_abort, c.confirmed_pad, cond.is_tracking, mp,
                             c.vision_age, x.t)
        arb = self.arbiter.step(trig, 1.0 if gate.landing_ready else 0.0)
        comm_issue, comm_shown = self.comm.step(x.t, x.stream_state, x.stream_error)
        packet = CommandPacket(arb.commander_triggers, cond.is_tracking, mp, g.desired_pose, x.wall_time)
        loop = x.wall_time - x.loopback_timestamp
        return TickOutputs(
            packet=packet, operator_triggers=trig, arbiter_triggers=arb.commander_triggers, measured_pose=mp,
            is_tracking=cond.is_tracking, home_pose=home, confirmed_pad=c.confirmed_pad, hold_pose=c.hold_pose,
            pad_u=c.pad_u, pad_v=c.pad_v, new_pad_sample=c.new_pad_sample, vision_age=c.vision_age,
            confidence=c.confidence,
            desired_pose=g.desired_pose, waypoint_index=g.waypoint_index, grid_complete=g.grid_complete,
            visited_count=g.visited_count, centering_active=g.centering_active, centered=g.centered,
            pixel_error=g.pixel_error, descent_stage=g.descent_stage, descent_abort=g.descent_abort,
            landing_ready=gate.landing_ready, fresh_count=gate.fresh_count, reason_code=gate.reason_code,
            abort_cause_bits=ab.cause_bits, abort_vision_age=ab.age_at_abort, abort_time=ab.time_at_abort,
            autoland_latched=arb.autoland_latched, joystick_issue=joy.joystick_issue, comm_issue=comm_issue,
            comm_issue_shown=comm_shown, loopback_time=loop if math.isfinite(loop) else math.nan,
        )
