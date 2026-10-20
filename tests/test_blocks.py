"""Simulink/QUARC block equivalents (documented semantics, no oracle)."""
from __future__ import annotations

import math

from quanser_landing.blocks.joystick import JoystickInterlock, simulink_dead_zone
from quanser_landing.blocks.simulink import (
    FallingEdgeLatch,
    InverseModulus,
    MeasurementConditioning,
    QuarcThreshold,
    StreamCommMonitor,
    motive_to_rsf_position,
    stage6_desired_pose,
)


def test_threshold_needs_hold_longer_than_duration():
    th = QuarcThreshold(">=", 0.5, 0.1)
    outs = [th.step(k * 0.001, 1.0)[0] for k in range(0, 102)]
    assert outs[100] == 0 and outs[101] == 1
    assert th.step(0.2, 0.0) == (0, 0.0)


def test_falling_edge_latch_reports_only_after_first_recovery():
    latch = FallingEdgeLatch()
    assert [latch.step(u) for u in (1, 1, 0, 1, 0)] == [0, 0, 1, 1, 1]


def test_comm_monitor_suppresses_initial_issue_until_recovery():
    mon = StreamCommMonitor()
    dt = 0.001
    out = []
    for k in range(3000):
        t = k * dt
        state = 0 if t < 1.5 or 2.0 <= t < 2.5 else 2
        out.append(mon.step(t, state, 0))
    raw = [o[0] for o in out]
    reported = [o[1] for o in out]
    assert max(raw[:1000]) == 0  # chart is silent for the first second
    assert max(raw[1200:1500]) == 1 and max(reported[:1500]) == 0  # not reported before a recovery
    assert max(reported[2150:2500]) == 1  # second loss is reported


def test_inverse_modulus_unwraps():
    u = InverseModulus()
    seq = [3.0, 3.1, -3.1, -3.0, 3.1]
    out = [u.step(v) for v in seq]
    assert out[2] == -3.1 + 2 * math.pi
    assert out[4] == 3.1


def test_measurement_conditioning_bias_and_saturation():
    mc = MeasurementConditioning()
    measured, tracking, corrected = mc.step([3.0, -0.5, 0.05, 0.1, 0.2, 0.3, 1.0])
    assert measured == (2.0, -0.5, 0.0, 0.3)  # x saturated, z - 0.07 saturated at 0
    assert tracking == 1.0 and corrected[3:5] == (0.1, 0.2)


def test_motive_to_rsf_position():
    assert motive_to_rsf_position((1.0, 2.0, 3.0)) == (-3.0, -1.0, 2.0)


def test_stage6_desired_pose_scalar_expansion_when_use_yolo_off():
    yolo = (1.0, 2.0, 1.25, 0.1)
    assert stage6_desired_pose(0, yolo, 1.0) == (1.0, 1.0, 1.0, 1.0)
    assert stage6_desired_pose(1, yolo, 1.0) == yolo


def test_dead_zone_is_shifted():
    assert simulink_dead_zone(0.005) == 0.0
    assert simulink_dead_zone(0.5) == 0.5 - 0.01
    assert simulink_dead_zone(-0.5) == -0.5 + 0.01


def test_joystick_interlock():
    j = JoystickInterlock()
    buttons = [0.0, 1.0] + [0.0] * 30
    out = j.step(0.0, buttons, [0.0, 0.2, -0.3, 0.4, 0.0, 0.0])
    assert out.triggers == (1.0, 1.0, 0.0, 0.0)
    assert out.commands[0] == 0.5 and out.commands[3] == -0.4 + 0.01
    # all axes at -1 for more than 0.2 s -> joystick issue zeroes arm/takeoff
    for k in range(1, 300):
        out = j.step(k * 0.001, buttons, [-1.0] * 6)
    assert out.joystick_issue == 1.0 and out.triggers[:2] == (0.0, 0.0)
