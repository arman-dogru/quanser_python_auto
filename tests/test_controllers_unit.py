"""Readable unit tests for the controller rules (the Octave parity tests are
the authority; these document the behaviour and pin the edge cases)."""
from __future__ import annotations

import math

import pytest

from quanser_landing.controllers import LandingPadRecenterLogic, PadState, PoseCommandConfig, YOLOPoseCommandGenerator

POSE = (0.0, 0.0, 1.25, 0.0)


def step_n(gen, n, *args):
    out = None
    for _ in range(n):
        out = gen.step(*args)
    return out


def test_signs_follow_calibration_mapping():
    g = YOLOPoseCommandGenerator()
    out = g.step(POSE, 1, 100, 0)  # pad right of centre in x -> move backward
    assert out.forward_cmd < 0 and out.right_cmd == 0 and out.state == PadState.RECENTERING
    g.reset()
    out = g.step(POSE, 1, 0, 100)  # pad below centre in y -> move left
    assert out.right_cmd < 0 and out.forward_cmd == 0


@pytest.mark.parametrize("err, inside", [(40.0, True), (-40.0, True), (math.nextafter(40.0, 99), False),
                                         (39.999, True), (41.0, False)])
def test_deadband_is_inclusive_40px(err, inside):
    g = YOLOPoseCommandGenerator()
    out = g.step(POSE, 1, err, 0.0)
    assert (out.forward_cmd == 0.0) is inside
    assert out.state == (PadState.CENTERED if inside else PadState.RECENTERING)


def test_axes_are_independent_inside_deadband():
    g = YOLOPoseCommandGenerator()
    out = g.step(POSE, 1, 10, 200)
    assert out.forward_cmd == 0.0 and out.right_cmd == 0.25 * (-200 / 240)


def test_smoothing_alpha_quarter_and_clamp():
    g = YOLOPoseCommandGenerator()
    a = g.step(POSE, 1, -640, 0)  # raw -(-640)/320 = 2 -> clamped to 1
    assert a.forward_cmd == 0.25
    b = g.step(POSE, 1, -640, 0)
    assert b.forward_cmd == 0.25 + 0.75 * 0.25


def test_detection_loss_decays_after_smoothing():
    g = YOLOPoseCommandGenerator()
    g.step(POSE, 1, -320, 0)  # smooth = 0.25
    out = g.step(POSE, 0, 999, 999)
    assert out.forward_cmd == 0.7 * (0.75 * 0.25)
    assert out.state == PadState.NO_DETECTION


def test_non_binary_detected_resets_without_decay():
    g = YOLOPoseCommandGenerator()
    g.step(POSE, 1, -320, 0)
    out = g.step(POSE, 2, 0, 0)
    assert out.state == PadState.NO_DETECTION and out.forward_cmd == 0.75 * 0.25


def test_ten_update_transition():
    g = YOLOPoseCommandGenerator()
    states = [g.step(POSE, 1, 0, 0).state for _ in range(11)]
    assert states[:9] == [PadState.CENTERED] * 9
    assert states[9:] == [PadState.CENTERED_STABLE] * 2
    assert g.step(POSE, 1, 41, 0).state == PadState.RECENTERING
    assert g.state.centered_counter == 0


def test_yaw_rotation_of_offsets():
    g = YOLOPoseCommandGenerator()
    out = g.step((1.0, 2.0, 1.0, math.pi / 2), 1, -320, 0)  # forward 0.25 -> +y in world
    assert out.cmd_pose[0] == pytest.approx(1.0, abs=1e-15)
    assert out.cmd_pose[1] == pytest.approx(2.0 + 0.05 * 0.25)
    assert out.cmd_pose[2:] == (1.0, math.pi / 2)


def test_descent_dormant_by_default():
    g = YOLOPoseCommandGenerator()
    out = step_n(g, 12, POSE, 1, 0, 0)
    assert out.state == PadState.CENTERED_STABLE and out.cmd_pose[2] == POSE[2]
    g2 = YOLOPoseCommandGenerator(PoseCommandConfig(enable_descent=True))
    assert step_n(g2, 12, POSE, 1, 0, 0).cmd_pose[2] == POSE[2] - 0.02


def test_nan_error_clamps_like_matlab():
    out = YOLOPoseCommandGenerator().step(POSE, 1, math.nan, 0)
    assert out.forward_cmd == 0.25 * -1.0


def test_recenter_semantics_differ_from_pose_command():
    r = LandingPadRecenterLogic()
    out = r.step(1, -320, 0)
    assert out.forward_cmd == 0.20 * 0.25 and out.descend_cmd == 0.0
    r.reset()
    out = step_n(r, 10, 1, 0, 0)
    assert out.state == PadState.CENTERED_STABLE and out.descend_cmd == 0.05


def test_reset_restores_initial_state():
    g = YOLOPoseCommandGenerator()
    step_n(g, 5, POSE, 1, 300, 300)
    g.reset()
    assert (g.state.smooth_forward, g.state.smooth_right, g.state.centered_counter) == (0.0, 0.0, 0.0)


def test_short_pose_rejected():
    with pytest.raises(ValueError):
        YOLOPoseCommandGenerator().step((0, 0, 1), 1, 0, 0)
