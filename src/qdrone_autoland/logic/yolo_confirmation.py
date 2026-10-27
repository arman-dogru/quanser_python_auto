"""Port of the "YOLO Confirmation and Hover Hold" chart as saved in the Phase 10
model (the Phase 9 abort-diagnostics builder appended the ``visionAge`` output).

Inputs per tick: measured pose [4], the 6-double vision packet
``[detected, centerX, centerY, confidence, frame, timestamp]``, the stream
client's "new data" flag, the mission clock and the operator triggers [4].
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class ConfirmationOutput:
    confirmed_pad: bool
    hold_pose: tuple[float, float, float, float]
    confidence: float
    pad_u: float
    pad_v: float
    new_pad_sample: bool
    vision_age: float


class YoloConfirmation:
    def __init__(self, confidence: float = 0.60, confirm_frames: int = 5, release_frames: int = 10,
                 stale_seconds: float = 0.50, width: float = 640.0, height: float = 480.0) -> None:
        self.min_confidence = confidence
        self.confirm_frames = confirm_frames
        self.release_frames = release_frames
        self.stale_seconds = stale_seconds
        self.width = width
        self.height = height
        self.reset()

    def reset(self) -> None:
        self.good = 0
        self.bad = 0
        self.active = False
        self.captured = (0.0, 0.0, 0.0, 0.0)
        self.last_frame = -1.0
        self.last_fresh = -1.0
        self.latest_u = 0.0
        self.latest_v = 0.0

    def step(self, measured_pose, vision, new_vision, clock_time, triggers) -> ConfirmationOutput:
        v = [float(x) for x in vision]
        if len(v) != 6:
            raise ValueError("vision packet must have 6 elements")
        takeoff = triggers[1] > 0.5
        new_pad_sample = False
        if not takeoff:
            self.good = 0
            self.bad = 0
            self.active = False
            self.last_fresh = float(clock_time)
        if takeoff and new_vision and v[4] != self.last_frame:
            self.last_frame = v[4]
            self.last_fresh = float(clock_time)
            valid = (v[0] > 0.5 and v[3] >= self.min_confidence and v[1] >= 0 and v[1] <= self.width
                     and v[2] >= 0 and v[2] <= self.height)
            if valid:
                self.latest_u = v[1]
                self.latest_v = v[2]
                new_pad_sample = True
                self.bad = 0
                if self.good < self.confirm_frames:
                    self.good += 1
                if not self.active and self.good >= self.confirm_frames:
                    self.active = True
                    self.captured = tuple(float(x) for x in measured_pose)
            else:
                self.good = 0
                if self.bad < self.release_frames:
                    self.bad += 1
                if self.active and self.bad >= self.release_frames:
                    self.active = False
        if self.active and (self.last_fresh < 0 or clock_time - self.last_fresh > self.stale_seconds):
            self.active = False
            self.good = 0
            self.bad = 0
        if self.last_fresh < 0:
            vision_age = math.inf
        else:
            vision_age = max(0.0, clock_time - self.last_fresh) if not math.isnan(clock_time - self.last_fresh) else 0.0
        return ConfirmationOutput(self.active, self.captured, v[3], self.latest_u, self.latest_v,
                                  new_pad_sample, vision_age)
