from __future__ import annotations

import math

import numpy as np
import pytest

from qdrone_autoland.blocks import (
    R_RSF_TO_MF,
    HomePose,
    Joystick,
    JoystickSample,
    MeasurementConditioning,
    MissionCommIssue,
    ThresholdTimer,
    Unwrap,
    motive_to_rsf,
)


def test_convention_correction_position_and_yaw():
    rsf = np.array([0.3, -0.4, 0.7])
    xyz, rot = motive_to_rsf(R_RSF_TO_MF @ rsf, R_RSF_TO_MF @ np.array([0.0, 0.0, 0.25]))
    np.testing.assert_allclose(xyz, rsf, atol=1e-15)
    np.testing.assert_allclose(rot, [0.0, 0.0, 0.25], atol=1e-12)


def test_conditioning_bias_saturation_unwrap():
    c = MeasurementConditioning()
    o = c.step([0.1, 0.2, 0.77, 0, 0, 3.1, 1])
    assert o.measured_pose[:3] == pytest.approx((0.1, 0.2, 0.70))
    o = c.step([5, -5, -1, 0, 0, -3.1, 1])
    assert o.measured_pose[:3] == (2.0, -2.0, 0.0)
    assert o.measured_pose[3] == pytest.approx(-3.1 + 2 * math.pi)


def test_unwrap_passes_nan():
    u = Unwrap()
    assert math.isnan(u.step(math.nan))


def test_threshold_timer():
    t = ThresholdTimer(0.5, 0.2)
    assert not t.step(0.0, 1) and not t.step(0.199, 1) and t.step(0.2, 1) and not t.step(0.3, 0)


def test_home_pose_captures_on_arm_after_5s():
    h = HomePose()
    out = None
    for k in range(9000):                     # 1 kHz, as the model runs
        t = k * 0.001
        arm = 1.0 if 1.0 <= t < 2.0 or t >= 6.0 else 0.0
        pose = (0.3, -0.9, 0.0, 0.1) if t < 7.0 else (5, 5, 5, 5)
        out = h.step(t, pose, arm)
        if abs(t - 3.0) < 1e-9:
            assert out == (0.0, 0.0, 0.7, 0.0)            # armed before 5 s: nothing captured
    # captured during the 1 s one-shot after the arm edge at 6 s, then held
    assert out == pytest.approx((0.3, -0.9, 0.7, 0.1))


def _joy(t, estop=0.0, arm=1.0, rz=1.0):
    b = [0.0] * 32
    b[0], b[1] = estop, arm
    return JoystickSample(new=float(int(t * 100) % 2), buttons=tuple(b), x=-1.0, y=0, z=0, rx=0, ry=0, rz=rz)


def test_joystick_mapping_estop_and_disconnect():
    j = Joystick()
    t = 0.0
    for _ in range(20):
        o = j.step(t, _joy(t)); t += 0.01
    assert o.triggers == (1.0, 1.0, 0.0, 0.0)
    for _ in range(25):
        o = j.step(t, _joy(t, estop=1.0)); t += 0.01
    assert o.triggers[2] == 1.0                                  # E-stop after 0.2 s held
    for _ in range(25):
        o = j.step(t, JoystickSample.disconnected()); t += 0.01
    assert o.triggers[0] == 0 and o.triggers[1] == 0 and o.triggers[3] == 1.0
    # During the 0.2 s before the issue latches, FrSKYMapping still runs on the
    # disconnected sample (buttons read 0), so the E-stop bit drops to 0 and is
    # then held there; the Commander acts on the joystick-issue bit instead.
    assert o.triggers[2] == 0.0


def test_mission_comm_issue_shown_only_after_first_recovery():
    m = MissionCommIssue()
    t, shown_seen = 0.0, []
    for k in range(3000):
        state = 0.0 if 1500 <= k < 1800 or 2200 <= k < 2600 else 2.0
        issue, shown = m.step(t, state, 0.0)
        shown_seen.append((issue, shown)); t += 0.001
    first = [i for i, (iss, _) in enumerate(shown_seen) if iss][0]
    assert shown_seen[first][1] == 0                             # first loss: not displayed
    assert any(s for _, s in shown_seen[2200:2600])              # second loss: displayed
