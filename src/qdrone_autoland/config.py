"""Mission configuration.

Two sources, kept separate on purpose:

* :func:`config_autonomous` mirrors ``matlab/config_autonomous.m`` field by
  field (the build script substitutes these into the Grid Search Manager).
* :class:`ModelConstants` holds numbers that the Phase 10 model hardcodes
  inside its MATLAB Function blocks or block dialogs rather than reading from
  the config (heights 0.70/0.60/0.55, the 0.45 m abort floor, 20 px, the
  10-frame counters, the 0.5 s freshness windows, ...). They are named here so
  the Python safety layer can derive its bounds from the same values, and
  ``tests/test_config.py`` checks every one against the text extracted from
  the model in ``docs/spec``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def generate_grid_waypoints(x_limits, y_limits, columns: int, rows: int, search_height: float) -> np.ndarray:
    """Port of ``matlab/generateGridWaypoints.m`` (serpentine, endpoint preserving)."""
    if not x_limits[0] < x_limits[1]:
        raise ValueError("xLimits must be [minimum maximum].")
    if not y_limits[0] < y_limits[1]:
        raise ValueError("yLimits must be [minimum maximum].")
    if int(columns) != columns or columns <= 0 or int(rows) != rows or rows <= 0:
        raise ValueError("columns and rows must be positive integers")
    x = np.linspace(x_limits[0], x_limits[1], int(columns))
    y = np.linspace(y_limits[0], y_limits[1], int(rows))
    out = np.zeros((int(columns) * int(rows), 3))
    k = 0
    for row in range(int(rows)):
        row_x = x if row % 2 == 0 else x[::-1]
        for cx in row_x:
            out[k] = (cx, y[row], search_height)
            k += 1
    return out


@dataclass(frozen=True)
class AutonomousConfig:
    """Field-for-field mirror of ``config_autonomous.m``."""

    qdrone_ip: str = "192.168.2.15"
    rigid_body_id: int = 1
    search_height: float = 0.70
    x_limits: tuple[float, float] = (-1.20, 1.20)
    y_limits: tuple[float, float] = (-1.00, 1.00)
    grid_columns: int = 5
    grid_rows: int = 4
    arrival_tolerance: float = 0.12
    dwell_samples: int = 100
    yolo_tcp_port: int = 18901
    yolo_confidence: float = 0.60
    yolo_confirm_frames: int = 5
    yolo_release_frames: int = 10
    yolo_stale_seconds: float = 0.50
    pixel_to_world_at_search_height: tuple[tuple[float, float], tuple[float, float]] = (
        (0.00163, 0.00019),
        (0.00007, -0.00166),
    )
    vision_calibration_yaw: float = 0.015
    camera_optical_center_pixel: tuple[float, float] = (320.0, 240.0)
    camera_offset_body: tuple[float, float] = (0.11, 0.0)
    center_max_step: float = 0.005
    center_max_displacement: float = 0.50
    center_pixel_tolerance: float = 20.0
    center_dwell_frames: int = 10

    @property
    def map(self) -> np.ndarray:
        return np.array(self.pixel_to_world_at_search_height, dtype=float)

    @property
    def vision_target_pixel(self) -> np.ndarray:
        """``cameraOpticalCenterPixel - map \\ cameraOffsetBody`` at search height."""
        return np.array(self.camera_optical_center_pixel) - np.linalg.solve(self.map, np.array(self.camera_offset_body))

    @property
    def waypoints(self) -> np.ndarray:
        return generate_grid_waypoints(self.x_limits, self.y_limits, self.grid_columns, self.grid_rows,
                                       self.search_height)


@dataclass(frozen=True)
class ModelConstants:
    """Values hardcoded in the Phase 10 mission-server model (see module doc)."""

    # Grid Search Manager (phase8_grid_manager_code.txt)
    stage_heights: tuple[float, float, float] = (0.70, 0.60, 0.55)   # descent ladder
    height_tolerance: float = 0.05            # |z - stage height| to advance
    abort_floor: float = 0.45                 # measured z below this latches abort
    hold_height_limits: tuple[float, float] = (0.55, 0.70)  # abort hold clamp
    map_height_limits: tuple[float, float] = (0.50, 0.70)   # map scaling clamp
    xy_settle_tolerance: float = 0.12         # centering xy error to advance stage
    # Landing Readiness Gate (phase9_landing_gate_code.txt)
    gate_fresh_frames: int = 10
    gate_xy_tolerance: float = 0.08
    gate_height: float = 0.55
    gate_height_tolerance: float = 0.04
    gate_stale_seconds: float = 0.50
    # Abort Diagnostic
    abort_diag_floor: float = 0.45
    # Measurement Conditioning
    rigid_body_height_offset: float = 0.07
    pose_saturation_lower: tuple[float, float, float] = (-2.0, -2.0, 0.0)
    pose_saturation_upper: tuple[float, float, float] = (2.0, 2.0, 5.0)
    # Calculate Home Pose
    home_height: float = 0.70
    home_enable_after_s: float = 5.0
    # YOLO Confirmation and Hover Hold: image bounds used to validate centres
    image_width: float = 640.0
    image_height: float = 480.0
    # JOYSTICK
    joystick_threshold_s: float = 0.2
    estop_threshold_s: float = 0.2
    takeoff_axis_threshold: float = -0.5
    # Communication - QDrone / Detect Communication loss
    comm_loss_after_s: float = 1.0
    comm_loss_threshold_s: float = 0.1
    # Solver
    step_size: float = 0.001                  # FixedStep of the active config set
    optitrack_decimation: int = 10            # OptiTrack Trackables sample time = 10 steps


@dataclass(frozen=True)
class SafetyConfig:
    """Python-side guards (not in the model). Bounds are derived from the
    mission/model values above so they cannot drift from what the logic flies."""

    pose_stale_s: float = 0.05          # 5 missed OptiTrack frames at 100 Hz
    joystick_stale_s: float = 0.20      # same window as the model's disconnect threshold
    vision_future_tolerance_s: float = 0.05
    geofence_margin: float = 1e-9       # numerical slack on every bound
    centring_refusal_escalate_s: float = 2.0   # refused centring poses for this long -> CENTRING_FAULT

    def z_limits(self, model: ModelConstants) -> tuple[float, float]:
        """Every desired height the logic can produce lies in the abort-hold clamp."""
        return model.hold_height_limits

    def xy_limits(self, model: ModelConstants) -> tuple[tuple[float, float], tuple[float, float]]:
        """The measured pose is saturated to this box, and home is a measured pose."""
        lo, hi = model.pose_saturation_lower, model.pose_saturation_upper
        return (lo[0], hi[0]), (lo[1], hi[1])


@dataclass(frozen=True)
class Config:
    auto: AutonomousConfig = field(default_factory=AutonomousConfig)
    model: ModelConstants = field(default_factory=ModelConstants)
    safety: SafetyConfig = field(default_factory=SafetyConfig)


DEFAULT = Config()
