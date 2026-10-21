"""Runtime path: guards -> mission core -> output gate.

:class:`MissionRuntime` is the only object that turns raw inputs into a
Commander packet. Both the real-time server (``serve``) and the offline
replay/simulation call :meth:`MissionRuntime.tick`, so the safety layer is
exercised by every test that runs a scenario.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

from .blocks import JoystickSample
from .config import DEFAULT, Config
from .mission_server import MissionServerCore, OptiTrackSample, TickInputs, TickOutputs
from .safety import GateDecision, JoystickGuard, Mode, OutputGate, PoseGuard, VisionGuard

log = logging.getLogger(__name__)


@dataclass
class RawInputs:
    t: float                                    # model clock (s since start)
    wall_time: float                            # wall clock (s), same base as the publisher timestamps
    optitrack: OptiTrackSample | None = None    # newest sample received since the last tick
    joystick: JoystickSample | None = None
    vision_bytes: bytes = b""                   # everything read from TCP 18901 since the last tick
    stream_state: float = 0.0
    stream_error: float = 0.0
    loopback_timestamp: float = 0.0


@dataclass
class TickRecord:
    raw: RawInputs
    inputs: TickInputs
    outputs: TickOutputs
    decision: GateDecision


@dataclass
class MissionRuntime:
    cfg: Config = DEFAULT
    mode: Mode = Mode.DRY_RUN
    core: MissionServerCore = field(init=False)
    pose_guard: PoseGuard = field(init=False)
    vision_guard: VisionGuard = field(init=False)
    joystick_guard: JoystickGuard = field(init=False)
    gate: OutputGate = field(init=False)
    failed: str | None = None

    def __post_init__(self) -> None:
        self.core = MissionServerCore(self.cfg)
        self.pose_guard = PoseGuard(self.cfg)
        self.vision_guard = VisionGuard(self.cfg)
        self.joystick_guard = JoystickGuard(self.cfg)
        self.gate = OutputGate(self.cfg, self.mode)

    def reset(self) -> None:
        for part in (self.core, self.pose_guard, self.vision_guard, self.joystick_guard, self.gate):
            part.reset()
        self.failed = None

    def tick(self, raw: RawInputs) -> TickRecord | None:
        """One model step. Returns None (and latches ``failed``) if anything
        raises: the caller must then stop streaming to the Commander, whose
        own comm-loss path takes over (see docs/SAFETY.md)."""
        if self.failed is not None:
            return None
        try:
            pose = self.pose_guard.step(raw.t, raw.optitrack)
            vision, new_vision = self.vision_guard.step(raw.wall_time, raw.vision_bytes)
            joy = self.joystick_guard.step(raw.t, raw.joystick)
            inputs = TickInputs(raw.t, raw.wall_time, pose, joy, vision, new_vision, raw.stream_state,
                                raw.stream_error, raw.loopback_timestamp)
            outputs = self.core.step(inputs)
            decision = self.gate.step(outputs)
            if decision.packet is not None and not decision.packet.is_finite():
                raise RuntimeError("non-finite packet escaped the output gate")
            return TickRecord(raw, inputs, outputs, decision)
        except Exception as exc:  # noqa: BLE001 - fail-stop on anything
            self.failed = f"{type(exc).__name__}: {exc}"
            log.exception("mission runtime failed; streaming to the Commander stops")
            return None


def finite_or(value: float, default: float) -> float:
    return value if math.isfinite(value) else default
