"""Ports of the legacy MATLAB prototype in ``reference/matlab/legacy``.

These predate the YOLO models and use a *different* image-to-motion mapping:
velocity commands ``vx = kp * error_y`` and ``vy = kp * error_x`` with no
sign inversion, versus Stage 6's ``forward = -error_x / 320``. They are kept
for reference and regression only; do not feed their outputs to the Stage 6
pose path.

Faithfully reproduced quirks (see ``docs/PORT_NOTES.md``):

* ``max(min(cmd, maxSpeed), -maxSpeed)`` uses MATLAB NaN-ignoring min/max,
  so a NaN pad centre (lost detection) yields ``+maxSpeed`` on both lateral
  axes instead of zero. The prototype never guards against this.
* ``config()`` assigns several fields twice; :func:`legacy_config` returns the
  effective (last-assigned) values.
* The grid uses the MATLAB colon operator; values are ``start + k * step``
  with the last element clamped to the limit, as Octave computes them.
  MATLAB's colon algorithm is documented differently and may differ from
  this in the last bit; it is only checked against Octave here.
"""
from __future__ import annotations

import math
from typing import Sequence

from ._matlab import mmax, mmin


def legacy_config() -> dict[str, float | int | bool]:
    """Effective parameter values of ``config.m`` after its overrides."""
    return {
        "dt": 0.05, "maxIterations": 3000,
        "xMin": -1.0, "xMax": 1.0, "yMin": -1.0, "yMax": 1.0,
        "searchHeight": 1.0, "gridSpacing": 0.30, "waypointTolerance": 0.08,
        "landingHeight": 0.10, "descentSpeed": 0.06,
        "kp_waypoint": 0.8, "kp_image": 0.001,
        "maxSpeed": 0.3,
        "centerTolerancePixels": 25,
        "maxDetectionImageSize": 640,
        "gaussianSigma": 1.0,
        "minCircleRadiusFraction": 0.08, "maxCircleRadiusFraction": 0.48,
        "circleSensitivity": 0.84, "circleEdgeThreshold": 0.12,
        "darkInsideThreshold": 0.35,  # 0.45 overridden
        "brightPadThreshold": 0.50, "minLocalContrast": 0.25,
        "minCircleDarkRatio": 0.06, "maxCircleDarkRatio": 0.65,
        "minCircleBrightRatio": 0.35,
        "brightPercentile": 70, "brightThresholdMin": 0.40,
        "minBrightArea": 500,  # 100 overridden
        "brightCloseRadius": 12,
        "minPadAreaFraction": 0.003,
        "maxPadAreaFraction": 0.85,  # 0.95 overridden
        "minPadWidth": 40, "minPadHeight": 40,  # 25 overridden
        "minAspectRatio": 0.35, "maxAspectRatio": 3.00,
        "minDarkInsideRatio": 0.01,  # 0.002 overridden
        "maxDarkInsideRatio": 0.60,  # 0.75 overridden
        "darkRatioWeight": 0.30,
        "darkThreshold": 0.35, "minDarkArea": 200, "closeRadius": 20, "minPadArea": 3000,
        "brightThreshold": 0.55, "minPadBrightArea": 3000,
        "useFallbackDetector": False,
    }


def _sat(values: Sequence[float], limit: float) -> list[float]:
    # MATLAB: max(min(command, maxSpeed), -maxSpeed)
    return [mmax(mmin(v, limit), -limit) for v in values]


def _norm(values: Sequence[float]) -> float:
    if any(math.isnan(v) for v in values):
        return math.nan
    return math.hypot(*values)


def align_to_pad(image_size: tuple[int, int], pad_center: Sequence[float],
                 params: dict) -> tuple[list[float], bool]:
    """``alignToPadController``; ``image_size`` is ``(width, height)``."""
    width, height = image_size
    ex = pad_center[0] - width / 2
    ey = pad_center[1] - height / 2
    cmd = _sat([params["kp_image"] * ey, params["kp_image"] * ex, 0.0], params["maxSpeed"])
    return cmd, bool(_norm((ex, ey)) < params["centerTolerancePixels"])


def descend(drone_pos: Sequence[float], image_size: tuple[int, int],
            pad_center: Sequence[float], params: dict) -> tuple[list[float], bool]:
    """``descendController``; returns ``(command, landed)``."""
    width, height = image_size
    ex = pad_center[0] - width / 2
    ey = pad_center[1] - height / 2
    cmd = _sat([params["kp_image"] * ey, params["kp_image"] * ex, -params["descentSpeed"]],
               params["maxSpeed"])
    return cmd, bool(drone_pos[2] <= params["landingHeight"])


def waypoint(drone_pos: Sequence[float], target: Sequence[float],
             params: dict) -> tuple[list[float], bool]:
    """``waypointController``; returns ``(command, waypoint_reached)``."""
    err = [t - p for t, p in zip(target, drone_pos)]
    cmd = _sat([params["kp_waypoint"] * e for e in err], params["maxSpeed"])
    return cmd, bool(_norm(err) < params["waypointTolerance"])


def _colon(start: float, step: float, stop: float) -> list[float]:
    if step == 0 or (step > 0 and start > stop) or (step < 0 and start < stop):
        return []
    n = int(math.floor((stop - start) / step * (1 + 2.220446049250313e-16)))
    vals = [start + k * step for k in range(n + 1)]
    # Octave clamps the final element to the limit when rounding overshoots.
    if (step > 0 and vals[-1] > stop) or (step < 0 and vals[-1] < stop):
        vals[-1] = stop
    return vals


def generate_grid_waypoints(params: dict) -> list[tuple[float, float, float]]:
    """``generateGridWaypoints``: boustrophedon grid at ``searchHeight``."""
    xs = _colon(params["xMin"], params["gridSpacing"], params["xMax"])
    ys = _colon(params["yMin"], params["gridSpacing"], params["yMax"])
    if xs[-1] < params["xMax"]:
        xs.append(params["xMax"])
    if ys[-1] < params["yMax"]:
        ys.append(params["yMax"])
    out = []
    for i, y in enumerate(ys, start=1):
        row = xs if i % 2 == 1 else list(reversed(xs))
        out.extend((x, y, params["searchHeight"]) for x in row)
    return out
