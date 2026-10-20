"""Earlier ``LandingPadRecenterLogic`` chart (recenter-only, pre-pose models).

Source: ``reference/extracted/QDrone_Recenter_Logic_Test/
chart_12_LandingPadRecenterLogic.m`` (identical bytes in the CSV test model).

This is deliberately a separate class from the Stage 6 generator because its
outputs mean different things:

* ``forward_cmd``/``right_cmd`` are ``0.20 * smoothed`` (a "small safe command
  range"), not normalized values and not pose offsets.
* Once centred for 10 consecutive updates it outputs state 3 with a positive
  ``descend_cmd = 0.05`` and forces the raw lateral requests to zero.

Deadband, smoothing, NaN and ``detected`` semantics are the same as in
:mod:`.pose_command`; see that module for the details.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ._matlab import clamp
from .pose_command import PadState


@dataclass(frozen=True)
class RecenterConfig:
    deadband_pixels: float = 40.0
    alpha: float = 0.25
    max_cmd: float = 0.20
    centered_required: int = 10
    descend_speed: float = 0.05
    image_center_x: float = 320.0
    image_center_y: float = 240.0
    loss_decay: float = 0.7


@dataclass(frozen=True)
class RecenterCommand:
    forward_cmd: float
    right_cmd: float
    descend_cmd: float
    state: PadState


@dataclass
class RecenterState:
    smooth_forward: float = 0.0
    smooth_right: float = 0.0
    centered_counter: float = 0.0


@dataclass
class LandingPadRecenterLogic:
    config: RecenterConfig = field(default_factory=RecenterConfig)
    state: RecenterState = field(default_factory=RecenterState)

    def reset(self) -> None:
        self.state = RecenterState()

    def step(self, detected: float, error_x: float, error_y: float) -> RecenterCommand:
        c, s = self.config, self.state
        detected = float(detected)
        error_x = float(error_x)
        error_y = float(error_y)

        raw_forward = 0.0
        raw_right = 0.0
        descend_cmd = 0.0
        state = PadState.NO_DETECTION

        if detected == 1:
            x_centered = abs(error_x) <= c.deadband_pixels
            y_centered = abs(error_y) <= c.deadband_pixels
            if x_centered and y_centered:
                s.centered_counter = s.centered_counter + 1
                state = PadState.CENTERED
            else:
                s.centered_counter = 0.0
                state = PadState.RECENTERING
                if not x_centered:
                    raw_forward = -error_x / c.image_center_x
                if not y_centered:
                    raw_right = -error_y / c.image_center_y
                raw_forward = clamp(raw_forward, -1.0, 1.0)
                raw_right = clamp(raw_right, -1.0, 1.0)
            if s.centered_counter >= c.centered_required:
                state = PadState.CENTERED_STABLE
                descend_cmd = c.descend_speed
                raw_forward = 0.0
                raw_right = 0.0
        else:
            s.centered_counter = 0.0
            state = PadState.NO_DETECTION
            raw_forward = 0.0
            raw_right = 0.0

        s.smooth_forward = c.alpha * raw_forward + (1 - c.alpha) * s.smooth_forward
        s.smooth_right = c.alpha * raw_right + (1 - c.alpha) * s.smooth_right

        if detected == 0:
            s.smooth_forward = c.loss_decay * s.smooth_forward
            s.smooth_right = c.loss_decay * s.smooth_right

        return RecenterCommand(
            forward_cmd=c.max_cmd * s.smooth_forward,
            right_cmd=c.max_cmd * s.smooth_right,
            descend_cmd=descend_cmd,
            state=state,
        )
