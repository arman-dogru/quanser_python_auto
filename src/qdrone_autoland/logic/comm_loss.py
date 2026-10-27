"""Port of the mission server's "Communication - QDrone/Detect Communication
loss/MATLAB Function" chart (raw flag before the 0.1 s QUARC Threshold)."""
from __future__ import annotations


class MissionCommLoss:
    def __init__(self, after_s: float = 1.0) -> None:
        self.after_s = after_s

    def reset(self) -> None:  # stateless; present for a uniform interface
        pass

    def step(self, time: float, state: float, error: float) -> float:
        loss = 0.0
        if time > self.after_s:
            if not (state == 2 and error >= 0):
                loss = 1.0
        return loss
