"""Port of ``phase9_landing_gate_code.txt`` (block "Landing Readiness Gate").

Reason codes: 0 ready, 1 stage, 2 abort, 3 vision, 4 tracking/pose,
5 alignment/height, 6 stale, 7 accumulating fresh observations.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ._matlab import all_finite, norm2

READY, STAGE, ABORT, VISION, TRACKING, ALIGNMENT, STALE, ACCUMULATING = range(8)


@dataclass
class GateOutput:
    landing_ready: bool
    fresh_count: float
    reason_code: int


class LandingReadinessGate:
    def __init__(self, fresh_frames: int = 10, xy_tolerance: float = 0.08, height: float = 0.55,
                 height_tolerance: float = 0.04, stale_seconds: float = 0.50, ready_stage: int = 5) -> None:
        self.fresh_frames = fresh_frames
        self.xy_tolerance = xy_tolerance
        self.height = height
        self.height_tolerance = height_tolerance
        self.stale_seconds = stale_seconds
        self.ready_stage = ready_stage
        self.reset()

    def reset(self) -> None:
        self.count = 0          # uint8 in MATLAB, saturates at fresh_frames
        self.last_fresh = -1.0

    def _clear(self) -> None:
        self.count = 0
        self.last_fresh = -1.0

    def step(self, descent_stage, descent_abort, confirmed_pad, centered, new_pad_sample, is_tracking,
             measured_pose, desired_pose, clock_time) -> GateOutput:
        ready = False
        reason = ACCUMULATING
        stale = self.last_fresh < 0 or clock_time - self.last_fresh > self.stale_seconds
        if descent_stage != self.ready_stage:
            self._clear()
            reason = STAGE
        elif descent_abort > 0.5:
            self._clear()
            reason = ABORT
        elif confirmed_pad <= 0.5:
            self._clear()
            reason = VISION
        elif (is_tracking <= 0.5 or not all_finite(measured_pose) or not all_finite(desired_pose)
              or not math.isfinite(clock_time)):
            self._clear()
            reason = TRACKING
        elif (centered <= 0.5
              or norm2(measured_pose[0] - desired_pose[0], measured_pose[1] - desired_pose[1]) > self.xy_tolerance
              or abs(measured_pose[2] - self.height) > self.height_tolerance):
            self._clear()
            reason = ALIGNMENT
        elif not new_pad_sample and stale:
            self._clear()
            reason = STALE
        else:
            if new_pad_sample:
                if stale:
                    self.count = 0
                self.last_fresh = float(clock_time)
                if self.count < self.fresh_frames:
                    self.count += 1
            ready = (self.count >= self.fresh_frames and self.last_fresh >= 0
                     and clock_time - self.last_fresh <= self.stale_seconds)
            if ready:
                reason = READY
        return GateOutput(ready, float(self.count), reason)
