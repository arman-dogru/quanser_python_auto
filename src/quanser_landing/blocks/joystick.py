"""Mission server ``JOYSTICK`` subsystem: the arming/abort interlock inputs.

Every constant and wire below is read from the Stage 6 model XML
(``.../JOYSTICK/FrSKYMapping`` and ``.../Initialization and Conditioning``);
block semantics are Simulink's/QUARC's documented ones. Not parity-checked
(no oracle for Simulink blocks).

Triggers sent to the QDrone (first 4 elements of its packet):
``[arm * ok, takeoff * ok, emergency_abort, joystick_issue]`` with
``ok = not joystick_issue``. Commands ``[throttle * ok, deadzone([roll,
pitch, yaw]) * ok]`` are only logged/used for the QBot 2e.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from .simulink import QuarcThreshold


def simulink_dead_zone(u: float, lower: float = -0.01, upper: float = 0.01) -> float:
    """Simulink Dead Zone: zero inside, shifted by the edge outside."""
    if u > upper:
        return u - upper
    if u < lower:
        return u - lower
    return 0.0


@dataclass(frozen=True)
class JoystickOutputs:
    triggers: tuple[float, float, float, float]  # arm, takeoff, abort, joystick_issue
    commands: tuple[float, float, float, float]  # throttle, roll, pitch, yaw
    joystick_issue: float


@dataclass
class JoystickInterlock:
    """FrSky mapping + issue detection + conditioning, one step per sample.

    ``buttons`` is the Host Game Controller button vector (index 1 = E-stop,
    index 2 = arm, MATLAB 1-based); axes are its ``x, y, z, Rx, Ry, Rz``.
    """

    estop_threshold: QuarcThreshold = field(default_factory=lambda: QuarcThreshold(">=", 0.5, 0.2))
    issue_threshold: QuarcThreshold = field(default_factory=lambda: QuarcThreshold(">=", 0.5, 0.2))

    def reset(self) -> None:
        self.estop_threshold.reset()
        self.issue_threshold.reset()

    def step(self, t: float, buttons: Sequence[float], axes: Sequence[float]) -> JoystickOutputs:
        x, y, z, rx, _ry, rz = (float(a) for a in axes)
        # Joystick issue: all six axes == -1 (disconnected) held for 0.2 s.
        all_neg = float(all(a == -1 for a in axes))
        issue, _ = self.issue_threshold.step(t, all_neg)
        ok = 0.0 if issue else 1.0
        abort, _ = self.estop_threshold.step(t, float(buttons[0]))
        arm = float(buttons[1])
        takeoff = float(rz >= -0.5)
        throttle = 0.5 * x + 0.5
        yaw = -rx
        roll, pitch = y, z
        triggers = (arm * ok, takeoff * ok, float(abort), float(issue))
        dz = [simulink_dead_zone(v) for v in (roll, pitch, yaw)]
        commands = (throttle * ok, dz[0] * ok, dz[1] * ok, dz[2] * ok)
        return JoystickOutputs(triggers, commands, float(issue))
