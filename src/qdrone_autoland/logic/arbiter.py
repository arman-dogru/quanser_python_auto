"""Port of ``phase10_autoland_arbiter_code.txt`` (block "Autonomous LAND
Trigger Arbiter"). Only trigger 2 (Takeoff/Land) may be lowered; arm,
emergency stop and joystick-issue pass through bit-for-bit."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ArbiterOutput:
    commander_triggers: tuple[float, float, float, float]
    autoland_latched: bool


class AutolandArbiter:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.latched = False

    def step(self, operator_triggers, landing_ready: float) -> ArbiterOutput:
        trig = [float(v) for v in operator_triggers]
        if len(trig) != 4:
            raise ValueError("operator_triggers must have 4 elements")
        armed = trig[0] > 0.5
        takeoff_selected = trig[1] > 0.5
        if not armed or not takeoff_selected:
            self.latched = False
        elif landing_ready > 0.5:
            self.latched = True
        out = list(trig)
        if self.latched and armed and takeoff_selected:
            out[1] = 0.0
        return ArbiterOutput(tuple(out), self.latched)
