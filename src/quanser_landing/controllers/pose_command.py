"""Stage 6 ``YOLOPoseCommandGenerator`` as a pure Python class.

Source: ``reference/extracted/QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/
chart_51_YOLOPoseCommandGenerator.m``. The same script (identical bytes) is
embedded in the YOLO pose-command test and the YOLO recenter integration
models, so this class covers all three.

Semantics, kept exactly as in the MATLAB source:

* Pixel errors are pad centre minus image centre for a 640 x 480 image.
* An axis whose ``|error| <= 40`` px (inclusive) requests zero on that axis.
* ``raw_forward = -error_x / 320`` and ``raw_right = -error_y / 240``,
  each clamped to [-1, 1] with MATLAB ``min``/``max`` (a NaN error clamps to
  -1, an infinite one to +/-1).
* Exponential smoothing with ``alpha = 0.25``; when ``detected == 0`` the
  smoothed values are additionally multiplied by 0.7 after smoothing.
* ``detected`` is compared with ``== 1`` and ``== 0`` separately. Any other
  value (for example 2 or NaN) takes the no-detection branch (state 0,
  counter reset) but does *not* apply the 0.7 decay.
* State 0 no detection, 1 recentering, 2 centered, 3 centered for at least 10
  consecutive updates. The counter is a plain double and keeps growing.
* ``cmd_pose = [x + dx, y + dy, z, yaw]`` with body offsets of
  ``0.05 * smoothed`` rotated by yaw. ``forward_cmd``/``right_cmd`` are the
  smoothed normalized values, not velocities.
* The descent branch (``z - 0.02`` in state 3) is dormant because
  ``enable_descent = 0`` in the source. It is kept, off by default, so the
  dormant path can be tested; nothing in this package turns it on.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence

from ._matlab import clamp


class PadState(IntEnum):
    NO_DETECTION = 0
    RECENTERING = 1
    CENTERED = 2
    CENTERED_STABLE = 3


@dataclass(frozen=True)
class PoseCommandConfig:
    """Tunables, defaulting to the literal values in the Stage 6 chart."""

    deadband_pixels: float = 40.0
    alpha: float = 0.25
    max_norm_cmd: float = 1.0
    max_position_step: float = 0.05  # metres per update
    enable_descent: bool = False  # enable_descent = 0 in Stage 6
    descent_step: float = 0.02  # metres, dormant
    centered_required: int = 10
    image_center_x: float = 320.0  # 640 px wide
    image_center_y: float = 240.0  # 480 px high
    loss_decay: float = 0.7


@dataclass(frozen=True)
class PoseCommand:
    cmd_pose: tuple[float, float, float, float]
    forward_cmd: float
    right_cmd: float
    state: PadState


@dataclass
class PoseCommandState:
    smooth_forward: float = 0.0
    smooth_right: float = 0.0
    centered_counter: float = 0.0


@dataclass
class YOLOPoseCommandGenerator:
    """Stateful equivalent of the chart's ``persistent`` variables.

    Call :meth:`step` once per chart execution (the Stage 6 model runs it at
    the 0.033 s rate of its input rate transition). :meth:`reset` restores
    the ``isempty(smooth_forward)`` initialisation.
    """

    config: PoseCommandConfig = field(default_factory=PoseCommandConfig)
    state: PoseCommandState = field(default_factory=PoseCommandState)

    def reset(self) -> None:
        self.state = PoseCommandState()

    def step(self, measured_pose: Sequence[float], detected: float,
             error_x: float, error_y: float) -> PoseCommand:
        c, s = self.config, self.state
        if len(measured_pose) < 4:
            raise ValueError("measured_pose must be [x, y, z, yaw]")
        x, y, z, yaw = (float(v) for v in measured_pose[:4])
        detected = float(detected)
        error_x = float(error_x)
        error_y = float(error_y)

        raw_forward = 0.0
        raw_right = 0.0
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
                # Calibration-confirmed mapping in the source:
                # error_x > 0 -> move backward, error_y > 0 -> move left.
                if not x_centered:
                    raw_forward = -error_x / c.image_center_x
                if not y_centered:
                    raw_right = -error_y / c.image_center_y
                raw_forward = clamp(raw_forward, -c.max_norm_cmd, c.max_norm_cmd)
                raw_right = clamp(raw_right, -c.max_norm_cmd, c.max_norm_cmd)
            if s.centered_counter >= c.centered_required:
                state = PadState.CENTERED_STABLE
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

        forward_offset = c.max_position_step * s.smooth_forward
        right_offset = c.max_position_step * s.smooth_right

        cy, sy = math.cos(yaw), math.sin(yaw)
        dx_world = cy * forward_offset - sy * right_offset
        dy_world = sy * forward_offset + cy * right_offset

        desired_z = z
        if c.enable_descent and state == PadState.CENTERED_STABLE:
            desired_z = z - c.descent_step

        return PoseCommand(
            cmd_pose=(x + dx_world, y + dy_world, desired_z, yaw),
            forward_cmd=s.smooth_forward,
            right_cmd=s.smooth_right,
            state=state,
        )
