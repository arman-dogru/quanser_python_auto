"""Pose and joystick sources (interfaces).

The model reads OptiTrack through QUARC's NatNet-based "OptiTrack Trackables"
block and the FrSKY transmitter through "Host Game Controller". Neither has a
Python implementation in this port yet: these protocols are the seam where a
NatNet client (e.g. NaturalNetwork / the Motive NatNet SDK) and a game
controller reader (e.g. pygame / SDL) plug in. What *is* implemented are the
fail-safe sources used when nothing is attached, and in-process stand-ins for
tests and the loopback demo.

Default behaviour without real sources is deliberately inert: no pose means
``isTracking = 0`` (the Commander FSM refuses to fly on a tracking issue) and
no joystick means a disconnected controller (arm and takeoff forced to 0).
"""
from __future__ import annotations

from typing import Protocol

from ..blocks import JoystickSample
from ..mission_server import OptiTrackSample


class PoseSource(Protocol):
    def poll(self, now: float) -> OptiTrackSample | None:
        """Newest rigid-body sample since the last call (Motive frame), or None."""


class JoystickSource(Protocol):
    def poll(self, now: float) -> JoystickSample | None:
        """Newest Host Game Controller sample since the last call, or None."""


class NoPose:
    """No OptiTrack attached: always None (the safety layer reports tracking 0)."""

    def poll(self, now: float) -> OptiTrackSample | None:
        return None


class NoJoystick:
    """No transmitter attached: always None (presented as disconnected)."""

    def poll(self, now: float) -> JoystickSample | None:
        return None


class StaticPose:
    """Test/demo stand-in: a fixed Motive-frame sample at 100 Hz."""

    def __init__(self, sample: OptiTrackSample, period: float = 0.01) -> None:
        self.sample, self.period, self.next = sample, period, 0.0

    def poll(self, now: float) -> OptiTrackSample | None:
        if now + 1e-12 < self.next:
            return None
        self.next = now + self.period
        return self.sample


class ScriptedJoystick:
    """Test/demo stand-in: ``fn(now) -> JoystickSample`` at 100 Hz, ``new`` toggling."""

    def __init__(self, fn, period: float = 0.01) -> None:
        self.fn, self.period, self.next, self.count = fn, period, 0.0, 0

    def poll(self, now: float) -> JoystickSample | None:
        if now + 1e-12 < self.next:
            return None
        self.next = now + self.period
        self.count += 1
        s = self.fn(now)
        s.new = float(self.count % 2)
        return s
