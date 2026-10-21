"""Simulink/QUARC block chains of the Phase 10 mission server that are not
MATLAB Function blocks: measurement conditioning, the Motive -> Research
Studio frame correction, home pose, joystick mapping and comm-loss timing.

These are re-implemented from the block parameters recorded in
``docs/spec/Mission_Server_QDrone_YOLO_Autonomous_Land.json``. Unlike the
MATLAB Function blocks there is no executable oracle for them here (the
QUARC library is closed and needs MATLAB); each class names the blocks it
reproduces, and every assumption about QUARC block semantics is listed in
docs/INTERFACES.md under "Unverified QUARC block semantics".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import DEFAULT, Config

TWO_PI = 2.0 * math.pi

# "Rotate Basis from RSF to MF" constant; the model multiplies by its transpose.
R_RSF_TO_MF = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_yzx_to_matrix(a1: float, a2: float, a3: float) -> np.ndarray:
    """"Euler Angles to Rotation Matrix (Relative Y-Z-X)": R = Ry(a1) Rz(a2) Rx(a3)."""
    return _ry(a1) @ _rz(a2) @ _rx(a3)


def matrix_to_euler_zyx(r: np.ndarray) -> tuple[float, float, float]:
    """"Euler Angles from Rotation Matrix (Relative Z-Y-X)": R = Rz(o1) Ry(o2) Rx(o3)."""
    o2 = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    o1 = math.atan2(r[1, 0], r[0, 0])
    o3 = math.atan2(r[2, 1], r[2, 2])
    return o1, o2, o3


def motive_to_rsf(xyz_mf, rot_mf):
    """"Convention Correction (Motive Frame to Research Studio Frame) - QDrone".

    Position: ``R' * xyz``. Attitude: ``R' * rot`` then "Prevent Gimbal Lock":
    demux -> YZX matrix from (rot2, rot3, rot1) -> ZYX angles (o1, o2, o3) ->
    mux (o3, o2, o1). Returns (xyz_rsf[3], rot_rsf[3]).
    """
    rt = R_RSF_TO_MF.T
    xyz = rt @ np.asarray(xyz_mf, dtype=float)
    rot = rt @ np.asarray(rot_mf, dtype=float)
    o1, o2, o3 = matrix_to_euler_zyx(euler_yzx_to_matrix(rot[1], rot[2], rot[0]))
    return xyz, np.array([o3, o2, o1])


class Unwrap:
    """QUARC "Inverse Modulus" (modulus 2*pi, unbounded): continuous angle."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.prev_in: float | None = None
        self.offset = 0.0

    def step(self, x: float) -> float:
        if not math.isfinite(x):
            return x
        if self.prev_in is not None:
            d = x - self.prev_in
            if d > math.pi:
                self.offset -= TWO_PI
            elif d < -math.pi:
                self.offset += TWO_PI
        self.prev_in = x
        return x + self.offset


@dataclass
class Conditioned:
    measured_pose: tuple[float, float, float, float]
    is_tracking: float
    corrected7: tuple[float, ...]


class MeasurementConditioning:
    """"Measurement Conditioning": bias [0 0 -0.07], saturation [-2 -2 0]..[2 2 5],
    yaw unwrap; isTracking passes through."""

    def __init__(self, cfg: Config = DEFAULT) -> None:
        m = cfg.model
        self.bias = np.array([0.0, 0.0, -m.rigid_body_height_offset])
        self.lo = np.array(m.pose_saturation_lower)
        self.hi = np.array(m.pose_saturation_upper)
        self.unwrap = Unwrap()

    def reset(self) -> None:
        self.unwrap.reset()

    def step(self, vehicle7) -> Conditioned:
        v = np.asarray(vehicle7, dtype=float)
        pos = np.minimum(np.maximum(v[0:3] + self.bias, self.lo), self.hi)
        # Simulink Saturation passes NaN through; np.maximum/minimum do too.
        yaw = self.unwrap.step(float(v[5]))
        measured = (float(pos[0]), float(pos[1]), float(pos[2]), yaw)
        corrected = (*map(float, pos), float(v[3]), float(v[4]), yaw, float(v[6]))
        return Conditioned(measured, float(v[6]), corrected)


class ThresholdTimer:
    """QUARC "Threshold" with a duration: true once ``x >= threshold`` has held
    continuously for at least ``duration`` seconds."""

    def __init__(self, threshold: float = 0.5, duration: float = 0.1) -> None:
        self.threshold = threshold
        self.duration = duration
        self.reset()

    def reset(self) -> None:
        self.since: float | None = None

    def step(self, t: float, x: float) -> bool:
        if x >= self.threshold:
            if self.since is None:
                self.since = t
            return t - self.since >= self.duration - 1e-12
        self.since = None
        return False


class HomePose:
    """"Calculate Home Pose": while ``Clock > 5`` and a one-shot that fires on
    the arm rising edge (pulse width 1/step samples = 1 s), the pose is passed
    through; otherwise the last captured value is held. Output z += 0.70."""

    def __init__(self, cfg: Config = DEFAULT) -> None:
        m = cfg.model
        self.enable_after = m.home_enable_after_s
        self.home_height = m.home_height
        self.pulse_samples = int(round(1.0 / m.step_size))
        self.reset()

    def reset(self) -> None:
        self.captured = (0.0, 0.0, 0.0, 0.0)
        self.prev_arm = 0.0
        self.remaining = 0

    def step(self, t: float, pose, arm: float):
        rising = arm > 0 and not self.prev_arm > 0
        self.prev_arm = arm
        if rising and self.remaining == 0:  # "Ignore extraneous pulse"
            self.remaining = self.pulse_samples
        pulse = self.remaining > 0
        if self.remaining > 0:
            self.remaining -= 1
        if t > self.enable_after and pulse:
            self.captured = tuple(float(v) for v in pose)
        x, y, z, yaw = self.captured
        return (x, y, z + self.home_height, yaw)


@dataclass
class JoystickSample:
    """One Host Game Controller sample (QUARC outputs new, buttons, sliders, axes)."""

    new: float = 0.0
    buttons: tuple[float, ...] = field(default_factory=lambda: (0.0,) * 32)
    sliders: tuple[float, ...] = (0.0, 0.0)
    x: float = -1.0
    y: float = -1.0
    z: float = -1.0
    rx: float = -1.0
    ry: float = -1.0
    rz: float = -1.0

    @classmethod
    def disconnected(cls, new: float = 0.0) -> JoystickSample:
        return cls(new=new)


@dataclass
class JoystickOutput:
    triggers: tuple[float, float, float, float]   # [arm, takeoff, e-stop, joystick issue]
    joystick_issue: float
    commands: tuple[float, float, float, float]   # [throttle, roll, pitch, yaw] (terminated in the model)


class Joystick:
    """"JOYSTICK" subsystem: FrSKY mapping, disconnect detection (all six axes
    at -1 for 0.2 s), pulse-check enable, and conditioning that zeroes arm and
    takeoff on a joystick issue while passing E-stop and the issue flag."""

    def __init__(self, cfg: Config = DEFAULT) -> None:
        m = cfg.model
        self.issue_timer = ThresholdTimer(0.5, m.joystick_threshold_s)
        self.estop_timer = ThresholdTimer(0.5, m.estop_threshold_s)
        self.takeoff_threshold = m.takeoff_axis_threshold
        self.reset()

    def reset(self) -> None:
        self.issue_timer.reset()
        self.estop_timer.reset()
        self.pulses = 0
        self.prev_new: float | None = None
        # FrSKYMapping outputs (held while disabled), initial 0
        self.map_out = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Initialization and Conditioning outputs (held while disabled), initial 0
        self.out = JoystickOutput((0.0, 0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.0, 0.0))

    def step(self, t: float, s: JoystickSample) -> JoystickOutput:
        axes = (s.x, s.y, s.z, s.rx, s.ry, s.rz)
        all_minus_one = all(a == -1.0 for a in axes)
        issue = 1.0 if self.issue_timer.step(t, 1.0 if all_minus_one else 0.0) else 0.0
        # The E-stop threshold runs inside FrSKYMapping, an enabled subsystem:
        # it only executes (and only advances its timer) while enabled.
        if not issue:
            buttons = list(s.buttons) + [0.0] * (32 - len(s.buttons))
            estop = 1.0 if self.estop_timer.step(t, buttons[0]) else 0.0
            arm = float(buttons[1])
            takeoff = 1.0 if s.rz >= self.takeoff_threshold else 0.0
            self.map_out = (estop, -s.rx, 0.5 * s.x + 0.5, s.y, s.z, arm, takeoff)
        # Pulse Check: count either edge of "new"; enable once >= 1.
        if self.prev_new is not None and (s.new > 0) != (self.prev_new > 0):
            self.pulses += 1
        self.prev_new = s.new
        if self.pulses >= 1:
            estop, yaw, throttle, roll, pitch, arm, takeoff = self.map_out
            ok = 0.0 if issue else 1.0
            self.out = JoystickOutput((ok * arm, ok * takeoff, float(estop), issue), issue,
                                      (ok * throttle, ok * _deadzone(roll), ok * _deadzone(pitch),
                                       ok * _deadzone(yaw)))
        return self.out


def _deadzone(v: float, lo: float = -0.01, hi: float = 0.01) -> float:
    if v > hi:
        return v - hi
    if v < lo:
        return v - lo
    return 0.0


class MissionCommIssue:
    """"Communication - QDrone/Detect Communication loss" (chart + 0.1 s
    threshold) followed by the mission-level "Triggered Subsystem" x Product:
    the warning is shown only after the issue has cleared at least once."""

    def __init__(self, cfg: Config = DEFAULT) -> None:
        from .logic.comm_loss import MissionCommLoss
        m = cfg.model
        self.chart = MissionCommLoss(m.comm_loss_after_s)
        self.timer = ThresholdTimer(0.5, m.comm_loss_threshold_s)
        self.reset()

    def reset(self) -> None:
        self.timer.reset()
        self.prev_issue = 0.0
        self.falling_edges = 0

    def step(self, t: float, state: float, error: float) -> tuple[float, float]:
        raw = self.chart.step(t, state, error)
        issue = 1.0 if self.timer.step(t, raw) else 0.0
        if self.prev_issue > 0 and issue == 0:
            self.falling_edges += 1
        self.prev_issue = issue
        shown = issue * (1.0 if self.falling_edges >= 1 else 0.0)
        return issue, shown
