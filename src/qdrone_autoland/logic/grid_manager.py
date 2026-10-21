"""Port of the Phase 10 "Grid Search Manager" chart (``phase8_grid_manager_code.txt``
with the config substituted by ``build_phase8_guarded_descent.m``).

Descent stage: 0 search; 1 centre at 0.70 m; 2 descend to 0.60 m;
3 centre at 0.60 m; 4 descend to 0.55 m; 5 low hover.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import DEFAULT, Config
from ._matlab import all_finite, clamp, norm2, norm3


@dataclass
class GridOutput:
    desired_pose: tuple[float, float, float, float]
    waypoint_index: float
    grid_complete: bool
    visited_count: float
    centering_active: bool
    centered: bool
    pixel_error: tuple[float, float]
    descent_stage: float
    descent_abort: bool


class GridSearchManager:
    def __init__(self, cfg: Config = DEFAULT) -> None:
        a, m = cfg.auto, cfg.model
        # The build script substitutes mat2str(cfg.waypoints, 6): six significant
        # digits, so the model flies y = -0.333333, not -1/3. Reproduce that.
        self.wps = np.array([[float(f"{v:.6g}") for v in row] for row in a.waypoints])
        self.n_wp = len(self.wps)
        self.base_map = a.map
        self.camera_offset = np.array(a.camera_offset_body, dtype=float)
        self.image_center = np.array(a.camera_optical_center_pixel, dtype=float)
        self.yaw0 = a.vision_calibration_yaw
        self.max_step = a.center_max_step
        self.max_disp = a.center_max_displacement
        self.x_lim = a.x_limits
        self.y_lim = a.y_limits
        self.px_tol = a.center_pixel_tolerance
        self.good_frames = a.center_dwell_frames
        self.arrival = a.arrival_tolerance
        self.dwell_samples = a.dwell_samples
        self.h0, self.h1, self.h2 = m.stage_heights
        self.h_tol = m.height_tolerance
        self.floor = m.abort_floor
        self.hold_lim = m.hold_height_limits
        self.map_lim = m.map_height_limits
        self.xy_tol = m.xy_settle_tolerance
        self.reset()

    def reset(self) -> None:
        self.stage = 0
        self.dwell = 0
        self.takeoff_was_low = True
        self.reverse_order = False
        self.complete = False
        self.visited = 0
        self.session_active = False
        self.center_mode = False
        self.center_pose = [0.0, 0.0, 0.0, 0.0]
        self.center_origin = [0.0, 0.0]
        self.center_good = 0
        self.descent_mode = 0
        self.abort_latched = False

    def _wp(self, stage: int) -> np.ndarray:
        idx = stage if not self.reverse_order else self.n_wp + 1 - stage
        return self.wps[idx - 1]

    def step(self, measured_pose, home_pose, triggers, confirmed_pad, hold_pose, pad_u, pad_v,
             new_pad_sample, is_tracking) -> GridOutput:
        mp = [float(v) for v in measured_pose]
        hp = [float(v) for v in home_pose]
        takeoff = triggers[1] > 0.5
        pixel_error = (0.0, 0.0)
        valid_pose = all_finite(mp) and all_finite(hp)
        if not takeoff:
            self.stage = 0
            self.dwell = 0
            self.takeoff_was_low = True
            self.reverse_order = False
            self.session_active = False
            self.center_mode = False
            self.center_good = 0
            self.descent_mode = 0
            self.abort_latched = False
        elif not self.session_active:
            self.complete = False
            self.visited = 0
            self.session_active = True
        if takeoff and confirmed_pad and is_tracking > 0.5 and valid_pose and not self.center_mode:
            self.center_mode = True
            self.center_pose = [float(v) for v in hold_pose]
            self.center_pose[2] = self.h0
            self.center_origin = [float(hold_pose[0]), float(hold_pose[1])]
            self.center_good = 0
            self.descent_mode = 1
        desired = [hp[0], hp[1], self.h0, hp[3]]
        if self.center_mode:
            if is_tracking <= 0.5 or not valid_pose or not confirmed_pad or mp[2] < self.floor:
                self.abort_latched = True
                self.center_good = 0
                if valid_pose and is_tracking > 0.5:
                    self.center_pose[2] = clamp(mp[2], *self.hold_lim)
            if not self.abort_latched:
                h = clamp(mp[2], *self.map_lim)
                cmap = self.base_map * (h / self.h0)
                target = self.image_center - np.linalg.solve(cmap, self.camera_offset)
                pixel_error = (pad_u - target[0], pad_v - target[1])
                if new_pad_sample:
                    if norm2(*pixel_error) <= self.px_tol:
                        if self.center_good < self.good_frames:
                            self.center_good += 1
                    else:
                        self.center_good = 0
                        bx = cmap[0, 0] * pixel_error[0] + cmap[0, 1] * pixel_error[1]
                        by = cmap[1, 0] * pixel_error[0] + cmap[1, 1] * pixel_error[1]
                        a = mp[3] - self.yaw0
                        c, s = math.cos(a), math.sin(a)
                        sx = c * bx + (-s) * by
                        sy = s * bx + c * by
                        n = norm2(sx, sy)
                        if n > self.max_step:
                            sx, sy = sx * (self.max_step / n), sy * (self.max_step / n)
                        cx = self.center_pose[0] + sx
                        cy = self.center_pose[1] + sy
                        ox, oy = cx - self.center_origin[0], cy - self.center_origin[1]
                        on = norm2(ox, oy)
                        if on > self.max_disp:
                            cx = self.center_origin[0] + ox * (self.max_disp / on)
                            cy = self.center_origin[1] + oy * (self.max_disp / on)
                        self.center_pose[0] = clamp(cx, *self.x_lim)
                        self.center_pose[1] = clamp(cy, *self.y_lim)
                    xy_error = norm2(mp[0] - self.center_pose[0], mp[1] - self.center_pose[1])
                    if self.center_good >= self.good_frames and xy_error <= self.xy_tol:
                        z = mp[2]
                        if self.descent_mode == 1 and abs(z - self.h0) <= self.h_tol:
                            self.descent_mode = 2
                            self.center_pose[2] = self.h1
                            self.center_good = 0
                        elif self.descent_mode == 2 and abs(z - self.h1) <= self.h_tol:
                            self.descent_mode = 3
                            self.center_good = 0
                        elif self.descent_mode == 3 and abs(z - self.h1) <= self.h_tol:
                            self.descent_mode = 4
                            self.center_pose[2] = self.h2
                            self.center_good = 0
                        elif self.descent_mode == 4 and abs(z - self.h2) <= self.h_tol:
                            self.descent_mode = 5
                            self.center_good = 0
            desired = list(self.center_pose)
        elif takeoff and self.takeoff_was_low:
            e_home = norm3(mp[0] - desired[0], mp[1] - desired[1], mp[2] - desired[2])
            if e_home <= self.arrival:
                self.dwell += 1
                if self.dwell >= self.dwell_samples:
                    f = self.wps[0]
                    last = self.wps[self.n_wp - 1]
                    d_first = (mp[0] - f[0]) * (mp[0] - f[0]) + (mp[1] - f[1]) * (mp[1] - f[1])
                    d_last = (mp[0] - last[0]) * (mp[0] - last[0]) + (mp[1] - last[1]) * (mp[1] - last[1])
                    self.reverse_order = d_last < d_first
                    self.stage = 1
                    self.dwell = 0
                    self.takeoff_was_low = False
            else:
                self.dwell = 0
        elif takeoff and self.stage >= 1:
            wp = self._wp(self.stage)
            desired[0], desired[1], desired[2] = float(wp[0]), float(wp[1]), float(wp[2])
            e = norm3(mp[0] - desired[0], mp[1] - desired[1], mp[2] - desired[2])
            if e <= self.arrival:
                self.dwell += 1
                if self.dwell >= self.dwell_samples:
                    self.visited = self.stage
                    if self.stage < self.n_wp:
                        self.stage += 1
                    else:
                        self.complete = True
                    self.dwell = 0
            else:
                self.dwell = 0
        waypoint_index = 0.0
        if self.stage >= 1:
            waypoint_index = float(self.stage if not self.reverse_order else self.n_wp + 1 - self.stage)
        centered = self.center_good >= self.good_frames and self.descent_mode == 5
        return GridOutput(tuple(desired), waypoint_index, self.complete, float(self.visited), self.center_mode,
                          centered, (float(pixel_error[0]), float(pixel_error[1])), float(self.descent_mode),
                          self.abort_latched)
