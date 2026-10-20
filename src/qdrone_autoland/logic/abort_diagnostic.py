"""Port of ``phase9_abort_diagnostic_code.txt`` (block "Abort Diagnostic").
Latches every abort cause present at the first abort sample. Diagnostic only."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ._matlab import all_finite

PAD_LOST, TRACKING_LOST, INVALID_POSE, BELOW_FLOOR, UNKNOWN = 1, 2, 4, 8, 16


@dataclass
class AbortDiagnosticOutput:
    cause_bits: float
    age_at_abort: float
    time_at_abort: float


class AbortDiagnostic:
    def __init__(self, floor: float = 0.45) -> None:
        self.floor = floor
        self.reset()

    def reset(self) -> None:
        self.seen_abort = False
        self.saved_bits = 0.0
        self.saved_age = 0.0
        self.saved_time = 0.0

    def step(self, descent_stage, descent_abort, confirmed_pad, is_tracking, measured_pose,
             vision_age, clock_time) -> AbortDiagnosticOutput:
        if descent_stage == 0 and descent_abort <= 0.5:
            self.reset()
        elif descent_abort > 0.5 and not self.seen_abort:
            bits = 0
            if confirmed_pad <= 0.5:
                bits += PAD_LOST
            if is_tracking <= 0.5:
                bits += TRACKING_LOST
            if not all_finite(measured_pose):
                bits += INVALID_POSE
            z = float(measured_pose[2])
            if math.isfinite(z) and z < self.floor:
                bits += BELOW_FLOOR
            if bits == 0:
                bits = UNKNOWN
            self.saved_bits = float(bits)
            self.saved_age = float(vision_age)
            self.saved_time = float(clock_time)
            self.seen_abort = True
        return AbortDiagnosticOutput(self.saved_bits, self.saved_age, self.saved_time)
