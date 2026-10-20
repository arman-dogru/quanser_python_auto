"""Safety layer: refusals never crash; protected triggers always reach the wire."""
from __future__ import annotations

import dataclasses
import math
import random

import pytest

from qdrone_autoland.blocks import JoystickSample
from qdrone_autoland.config import DEFAULT
from qdrone_autoland.mission_server import OptiTrackSample
from qdrone_autoland.protocol import VISION
from qdrone_autoland.runtime import MissionRuntime, RawInputs
from qdrone_autoland.safety import GateStatus, JoystickGuard, Mode, OutputGate, PoseGuard, VisionGuard

NOW = 1_760_000_000.0


# ------------------------------------------------------------------ pose
def test_pose_guard_refuses_nan_and_holds_last_good():
    g = PoseGuard()
    good = OptiTrackSample((0.1, 0.2, 0.3), (0, 0, 0), 1.0)
    assert g.step(0.0, good) == good
    out = g.step(0.01, OptiTrackSample((math.nan, 0, 0), (0, 0, 0), 1.0))
    assert out.position == good.position and out.tracking == 0.0 and g.refused["malformed"] == 1


def test_pose_guard_stale_and_garbage_objects():
    g = PoseGuard()
    g.step(0.0, OptiTrackSample((0, 0, 0), (0, 0, 0), 1.0))
    assert g.step(0.04, None) is None                       # within 50 ms: hold
    assert g.step(0.06, None).tracking == 0.0               # stale
    for junk in (object(), OptiTrackSample((1, 2), (0, 0, 0), 1), OptiTrackSample(("x", 0, 0), (0, 0, 0), 1)):
        assert g.step(0.07, junk).tracking == 0.0


def test_pose_guard_keeps_motive_tracking_flag():
    """Motive reports a lost rigid body by continuing to stream with tracking = 0."""
    g = PoseGuard()
    s = g.step(0.0, OptiTrackSample((0.1, 0.2, 0.3), (0, 0, 0), 0.0))
    assert s.tracking == 0.0 and s.position == (0.1, 0.2, 0.3)
    assert g.step(0.01, OptiTrackSample((0.1, 0.2, 0.3), (0, 0, 0), 1.0)).tracking == 1.0


def test_runtime_streams_tracking_zero_to_wire():
    """A finite pose with tracking = 0, streamed at 100 Hz, must reach the
    Commander as isTracking = 0 on every tick (not only when samples stop)."""
    rt = MissionRuntime(DEFAULT, Mode.MONITOR)
    for k in range(300):
        t = k * 0.001
        trk = 1.0 if k < 100 else 0.0
        opti = OptiTrackSample((0.0, 0.7, 0.0), (0, 0, 0), trk) if k % 10 == 0 else None
        rec = rt.tick(RawInputs(t, NOW + t, opti))
        if 20 <= k < 100:
            assert rec.decision.packet.is_tracking == 1.0, k
        if k >= 100:
            assert rec.decision.packet.is_tracking == 0.0, k


# ---------------------------------------------------------------- vision
@pytest.mark.parametrize("vec,reason", [
    ((math.nan, 1, 1, 1, 1, NOW), "non-finite"),
    ((0.5, 1, 1, 1, 1, NOW), "bad-detected-flag"),
    ((1, 1, 1, 0.9, -1, NOW), "bad-frame-number"),
    ((1, 1, 1, 0.9, 1.5, NOW), "bad-frame-number"),
    ((1, 1, 1, 1.5, 1, NOW), "bad-confidence"),
    ((1, 1, 1, 0.9, 1, NOW + 1), "from-future"),
    ((1, 1, 1, 0.9, 1, NOW - 0.51), "stale"),
])
def test_vision_guard_refusals(vec, reason):
    g = VisionGuard()
    v, new = g.step(NOW, VISION.pack(*vec))
    assert not new and g.refused[reason] == 1 and v == (0.0, 0.0, 0.0, 0.0, -1.0, 0.0)


@pytest.mark.parametrize("ahead,accepted", [(0.049, True), (0.051, False), (0.09, False)])
def test_vision_guard_future_tolerance_boundary(ahead, accepted):
    g = VisionGuard()
    assert DEFAULT.safety.vision_future_tolerance_s == 0.05
    _, new = g.step(NOW, VISION.pack(1, 300, 200, 0.9, 1, NOW + ahead))
    assert new is accepted and g.refused["from-future"] == (0 if accepted else 1)


def test_vision_guard_accepts_and_uses_newest_only():
    g = VisionGuard()
    data = VISION.pack(1, 10, 10, 0.9, 1, NOW) + VISION.pack(1, 20, 20, 0.9, 2, NOW)
    v, new = g.step(NOW, data)
    assert new and v[1] == 20 and g.buffer.dropped == 1
    v, new = g.step(NOW, b"")
    assert not new and v[1] == 20


def test_vision_guard_resyncs_after_framing_loss():
    g = VisionGuard()
    g.step(NOW, b"\x01\x02\x03")
    for k in range(3):
        g.step(NOW, VISION.pack(1, 300, 200, 0.9, 10 + k, NOW))
    assert g.resyncs == 1 and g.resync_requested
    v, new = g.step(NOW, VISION.pack(1, 300, 200, 0.9, 20, NOW))
    assert new and v[4] == 20


def test_vision_guard_valid_record_resets_malformed_count():
    g = VisionGuard()
    bad = VISION.pack(math.nan, 0, 0, 0, 1, NOW)
    for k in range(4):
        g.step(NOW, bad)
        g.step(NOW, bad)
        g.step(NOW, VISION.pack(1, 300, 200, 0.9, 10 + k, NOW))
    assert g.resyncs == 0


# -------------------------------------------------------------- joystick
def test_joystick_guard_stale_and_malformed():
    g = JoystickGuard()
    s = JoystickSample(new=1.0, buttons=(0.0,) * 32, x=0, y=0, z=0, rx=0, ry=0, rz=0)
    assert g.step(0.0, s) is s
    assert g.step(0.1, None) is s
    assert g.step(0.25, None).rz == -1.0 and g.refused["stale"] == 1
    bad = JoystickSample(new=1.0, buttons=(0.0,) * 3)
    assert g.step(0.3, bad).x == -1.0 and g.refused["malformed"] == 1
    nan = JoystickSample(new=1.0, buttons=(0.0,) * 32, x=0, y=0, z=0, rx=0, ry=0, rz=math.nan)
    assert g.step(0.31, nan).x == -1.0 and g.refused["malformed"] == 2


def test_joystick_guard_malformed_forgets_last_good():
    """After a malformed sample the stale last good one must not come back."""
    g = JoystickGuard()
    s = JoystickSample(new=1.0, buttons=(0.0,) * 32, x=0, y=0, z=0, rx=0, ry=0, rz=1)
    assert g.step(0.0, s) is s
    assert g.step(0.01, JoystickSample(new=0.0, buttons=(0.0,) * 3)).rz == -1.0
    for t in (0.02, 0.05, 0.1):
        out = g.step(t, None)
        assert out is not s and out.rz == -1.0 and out.x == -1.0


# ------------------------------------------------------------ output gate
@pytest.fixture
def outputs():
    rt = MissionRuntime(DEFAULT, Mode.MONITOR)
    rec = rt.tick(RawInputs(0.0, NOW, OptiTrackSample((0, 0, 0), (0, 0, 0), 1.0)))
    return dataclasses.replace(rec.outputs, measured_pose=(0.0, -0.9, 0.6, 0.0), desired_pose=(0.0, -0.9, 0.7, 0.0),
                               is_tracking=1.0)


@pytest.mark.parametrize("mode", list(Mode))
def test_protected_triggers_pass_through_in_every_mode(outputs, mode):
    gate = OutputGate(DEFAULT, mode)
    for op in [(a, b, c, d) for a in (0., 1.) for b in (0., 1.) for c in (0., 1.) for d in (0., 1.)]:
        for arb2 in (0.0, op[1]):
            o = dataclasses.replace(outputs, operator_triggers=op, arbiter_triggers=(op[0], arb2, op[2], op[3]))
            dec = gate.step(o)
            pkt = dec.packet if dec.packet is not None else None
            if mode is Mode.DRY_RUN:
                assert pkt is None
                continue
            assert (pkt.triggers[0], pkt.triggers[2], pkt.triggers[3]) == (op[0], op[2], op[3])
            lowered = mode is Mode.AUTOLAND and arb2 == 0.0 and op[1] == 1.0 and op[0] == 1.0
            assert pkt.triggers[1] == (0.0 if lowered else op[1])


def test_arbiter_cannot_alter_protected_triggers(outputs):
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    o = dataclasses.replace(outputs, operator_triggers=(1., 1., 1., 0.), arbiter_triggers=(0., 1., 0., 1.))
    dec = gate.step(o)
    assert dec.packet.triggers == (1., 1., 1., 0.) and "arbiter-altered-protected-trigger" in dec.faults


def test_monitor_never_lowers_takeoff(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, operator_triggers=(1., 1., 0., 0.), arbiter_triggers=(1., 0., 0., 0.))
    assert gate.step(o).packet.triggers[1] == 1.0


@pytest.mark.parametrize("desired,fault", [
    ((math.nan, 0, 0.6, 0), "desired-non-finite"),
    ((2.5, 0, 0.6, 0), "desired-outside-geofence"),
    ((0, -2.1, 0.6, 0), "desired-outside-geofence"),
    ((0, 0, 0.9, 0), "desired-height-out-of-range"),
    ((0, 0, 0.3, 0), "desired-height-out-of-range"),
])
def test_out_of_bounds_desired_is_refused_and_last_good_held(outputs, desired, fault):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(outputs)
    dec = gate.step(dataclasses.replace(outputs, desired_pose=desired))
    assert fault in dec.faults and dec.packet.desired_pose == outputs.desired_pose


def test_centering_step_limit(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, centering_active=True, desired_pose=(0.0, -0.5, 0.7, 0.0))
    gate.step(o)
    gate.step(dataclasses.replace(o, desired_pose=(0.004, -0.5, 0.7, 0.0)))
    dec = gate.step(dataclasses.replace(o, desired_pose=(0.03, -0.5, 0.7, 0.0)))
    assert "centering-step-too-large" in dec.faults
    dec = gate.step(dataclasses.replace(o, desired_pose=(0.004, -0.5, 0.5, 0.0)))
    assert "desired-height-out-of-range" in dec.faults


def test_centering_displacement_limit(outputs):
    """Walking in legal 5 mm steps still stops 0.5 m from the centring origin."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, centering_active=True, desired_pose=(0.0, -0.9, 0.7, 0.0))
    gate.step(dataclasses.replace(o, centering_active=False))
    assert not gate.step(o).faults
    x = 0.0
    faults = []
    for _ in range(150):
        x += 0.0049
        faults = gate.step(dataclasses.replace(o, desired_pose=(x, -0.9, 0.7, 0.0))).faults
        if faults:
            break
    assert faults == ["centering-displacement-too-large"] and 0.5 < x < 0.51
    assert gate.step(dataclasses.replace(o, desired_pose=(x, -0.9, 0.7, 0.0))).packet.desired_pose[0] < 0.5


def test_nan_hold_corner_jump_refused_and_limit_kept(outputs):
    """The model with a NaN hold pose commands (-1.2, -1.0, 0.7, NaN) and then
    walks, with its own 0.5 m limit disabled (NaN origin). The gate refuses the
    jump and the NaN yaw, and keeps the 0.5 m limit with a finite origin."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    hover = dataclasses.replace(outputs, desired_pose=(0.0, -0.9, 0.7, 0.0))
    gate.step(hover)
    c = dataclasses.replace(hover, centering_active=True)
    dec = gate.step(dataclasses.replace(c, desired_pose=(-1.2, -1.0, 0.7, math.nan)))
    assert "desired-non-finite" in dec.faults and dec.packet.desired_pose == (0.0, -0.9, 0.7, 0.0)
    assert gate.center_origin == (0.0, -0.9)
    # Same jump with a finite yaw: refused as an entry jump.
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(hover)
    dec = gate.step(dataclasses.replace(c, desired_pose=(-1.2, -1.0, 0.7, 0.0)))
    assert dec.faults == ["centering-entry-jump"] and dec.packet.desired_pose == (0.0, -0.9, 0.7, 0.0)


def test_centring_origin_when_entry_has_nothing_finite(outputs):
    """Entry with a refused pose, nothing accepted before and a non-finite
    measured pose: the first accepted centring pose becomes the origin, so
    the 0.5 m limit still holds."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    c = dataclasses.replace(outputs, centering_active=True, measured_pose=(math.nan, 0, 0.6, 0))
    gate.step(dataclasses.replace(c, desired_pose=(math.nan, -0.9, 0.7, 0.0)))
    assert gate.center_origin is None
    c = dataclasses.replace(c, measured_pose=(0.0, -0.9, 0.6, 0.0))
    assert not gate.step(dataclasses.replace(c, desired_pose=(0.0, -0.9, 0.7, 0.0))).faults
    assert gate.center_origin == (0.0, -0.9)
    x, faults = 0.0, []
    while not faults and x < 1.0:
        x += 0.0049
        faults = gate.step(dataclasses.replace(c, desired_pose=(x, -0.9, 0.7, 0.0))).faults
    assert faults == ["centering-displacement-too-large"] and x < 0.51


def _grid_then_centre(gate, outputs, waypoint, measured, entry):
    """Grid search towards ``waypoint`` while the vehicle is at ``measured``,
    then the model enters centring at ``entry`` (its hover-hold pose)."""
    o = dataclasses.replace(outputs, measured_pose=measured, is_tracking=1.0)
    assert not gate.step(dataclasses.replace(o, desired_pose=waypoint)).faults
    return gate.step(dataclasses.replace(o, centering_active=True, desired_pose=entry))


@pytest.mark.parametrize("waypoint,measured,entry", [
    ((-1.2, -1.0, 0.7, 0.0), (-0.098, -0.851, 0.7, 0.0), (-0.10, -0.852, 0.7, 0.0)),   # pad (0.1, -0.95)
    ((0.0, 0.333, 0.7, 0.0), (-0.578, 0.322, 0.7, 0.0), (-0.58, 0.321, 0.7, 0.0)),     # pad (-0.3, 0.407)
    ((0.6, 1.0, 0.7, 0.0), (1.142, 0.905, 0.7, 0.0), (1.144, 0.906, 0.7, 0.0)),        # pad (0.9, 0.95)
])
def test_centring_entry_is_measured_from_where_the_vehicle_is(outputs, waypoint, measured, entry):
    """Pad confirmed while the vehicle is still > 0.5 m short of its waypoint:
    the entry pose (the hover-hold pose, captured from the measured pose) is
    accepted, and centring proceeds."""
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    dec = _grid_then_centre(gate, outputs, waypoint, measured, entry)
    assert not dec.faults and dec.packet.desired_pose == entry and dec.status is GateStatus.OK
    nxt = (entry[0] + 0.004, entry[1], 0.7, 0.0)
    dec = gate.step(dataclasses.replace(outputs, measured_pose=measured, centering_active=True, desired_pose=nxt))
    assert not dec.faults


def test_centring_entry_falls_back_to_last_accepted_when_not_tracking(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(dataclasses.replace(outputs, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    lost = dataclasses.replace(outputs, measured_pose=(1.0, 0.5, 0.7, 0.0), is_tracking=0.0, centering_active=True)
    assert gate.step(dataclasses.replace(lost, desired_pose=(1.0, 0.5, 0.7, 0.0))).faults[0] == "centering-entry-jump"
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(dataclasses.replace(outputs, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    assert "centering-entry-jump" not in gate.step(dataclasses.replace(lost, desired_pose=(0.1, -0.8, 0.7, 0.0))).faults


def _centring_at(gate, outputs, here):
    """Accept a centring entry at ``here`` with the vehicle there."""
    o = dataclasses.replace(outputs, measured_pose=here, is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=here))
    c = dataclasses.replace(o, centering_active=True)
    assert not gate.step(dataclasses.replace(c, desired_pose=here)).faults
    return c


def test_step_limit_holds_across_refused_ticks(outputs):
    """Every accepted centring pose is within 5 mm of the previously accepted
    one: a refused tick does not turn the limit into a one-tick delay."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    here = (0.2, -0.5, 0.7, 0.0)
    c = _centring_at(gate, outputs, here)
    for k in range(5):                                                              # a 0.4 m jump, held
        dec = gate.step(dataclasses.replace(c, desired_pose=(0.6, -0.5, 0.7, 0.0)))
        assert dec.faults == ["centering-step-too-large"], k
        assert dec.packet.desired_pose == here and dec.status is GateStatus.CENTRING_REFUSED
    dec = gate.step(dataclasses.replace(c, desired_pose=(0.21, -0.5, 0.7, 0.0)))   # 10 mm from the last accepted
    assert dec.faults == ["centering-step-too-large"]
    dec = gate.step(dataclasses.replace(c, desired_pose=(0.204, -0.5, 0.7, 0.0)))  # 4 mm: accepted
    assert not dec.faults and dec.status is GateStatus.OK and dec.packet.desired_pose[0] == 0.204
    assert gate.centring_refusals == 6


def test_glitch_during_centring_does_not_lock_out(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    here = (0.2, -0.5, 0.7, 0.0)
    c = _centring_at(gate, outputs, here)
    dec = gate.step(dataclasses.replace(c, desired_pose=(math.nan, -0.5, 0.7, 0.0)))
    assert dec.faults == ["desired-non-finite"] and dec.packet.desired_pose == here
    dec = gate.step(dataclasses.replace(c, desired_pose=(0.203, -0.5, 0.7, 0.0)))
    assert not dec.faults and dec.status is GateStatus.OK


def test_refuse_accept_alternation_cannot_leave_the_origin_disc(outputs):
    """Refuse/accept alternation: the vehicle follows 0.2 m steps; nothing sent leaves
    the 0.5 m disc around the centring origin."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    c = _centring_at(gate, outputs, (0.0, 0.0, 0.7, 0.0))
    sent = []
    for k in range(1, 41):
        x = 0.2 * k
        dec = gate.step(dataclasses.replace(c, measured_pose=(x, 0.0, 0.7, 0.0), desired_pose=(x, 0.0, 0.7, 0.0)))
        sent.append(dec.packet.desired_pose[0])
    assert max(sent) <= 0.5 + 1e-9 and max(sent) == 0.0


def test_displacement_limit_applies_on_the_tick_after_a_refusal(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    c = _centring_at(gate, outputs, (0.0, 0.0, 0.7, 0.0))
    for k in range(1, 101):                                                         # walk out to 0.5 m
        assert not gate.step(dataclasses.replace(c, desired_pose=(0.005 * k, 0.0, 0.7, 0.0))).faults, k
    assert gate.step(dataclasses.replace(c, desired_pose=(math.nan, 0.0, 0.7, 0.0))).faults
    dec = gate.step(dataclasses.replace(c, desired_pose=(0.504, 0.0, 0.7, 0.0)))
    assert dec.faults == ["centering-displacement-too-large"] and dec.packet.desired_pose[0] == 0.5


def test_displacement_limit_applies_while_the_entry_is_refused(outputs):
    """Entry refused: origin = where the vehicle was. If the vehicle then
    drifts, a pose within 0.5 m of the vehicle but not of the origin is refused."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(0.0, 0.0, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(0.0, 0.0, 0.7, 0.0)))
    dec = gate.step(dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0)))
    assert dec.faults == ["centering-entry-jump"] and gate.center_origin == (0.0, 0.0)
    for k in range(1, 4):
        m = (0.1 * k, 0.0, 0.7, 0.0)
        d = (0.1 * k + 0.45, 0.0, 0.7, 0.0)                                         # 0.45 m from the vehicle
        dec = gate.step(dataclasses.replace(o, measured_pose=m, centering_active=True, desired_pose=d))
        assert dec.faults == ["centering-displacement-too-large"], k


def test_entry_hold_is_captured_once_and_not_dragged_by_the_measured_pose(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(0.0, 0.0, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(1.2, 1.0, 0.7, 0.0)))
    bad = dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0))
    assert gate.step(bad).packet.desired_pose[:2] == (0.0, 0.0)
    for k in range(1, 4):
        dec = gate.step(dataclasses.replace(bad, measured_pose=(0.05 * k, 0.0, 0.7, 0.0)))
        assert dec.packet.desired_pose[:2] == (0.0, 0.0), k


def test_entry_hold_is_recaptured_in_a_new_centring_phase(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(0.0, 0.0, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(0.0, 0.0, 0.7, 0.0)))
    bad = dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0))
    assert gate.step(bad).packet.desired_pose[:2] == (0.0, 0.0)
    for k in range(1, 6):                                                           # centring ends, vehicle moves
        m = (0.05 * k, 0.0, 0.7, 0.0)
        gate.step(dataclasses.replace(o, measured_pose=m, desired_pose=m))
    dec = gate.step(dataclasses.replace(bad, measured_pose=(0.25, 0.0, 0.7, 0.0)))
    assert dec.faults == ["centering-entry-jump"] and dec.packet.desired_pose[:2] == (0.25, 0.0)


def test_teleported_measured_sample_never_becomes_the_hold(outputs):
    """One OptiTrack frame at (1.95, 1.95) on the tick the
    refusals start, then the real pose again."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    H = (0.0, 0.0, 0.7, 0.0)
    c = _centring_at(gate, outputs, H)
    dec = gate.step(dataclasses.replace(c, measured_pose=(1.95, 1.95, 0.7, 0.0), desired_pose=(0.3, 0.0, 0.7, 0.0)))
    assert dec.faults and dec.packet.desired_pose == H
    for _ in range(5):
        dec = gate.step(dataclasses.replace(c, measured_pose=H, desired_pose=(0.9, 0.0, 0.7, 0.0)))
        assert dec.faults and dec.packet.desired_pose == H


def test_teleported_measured_sample_during_entry_is_not_the_hold(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(0.0, 0.0, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(0.0, 0.0, 0.7, 0.0)))
    bad = dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0))
    for m in [(0.6, 0.6, 0.7, 0.0), (1.95, 1.95, 0.7, 0.0), (0.0, 0.0, 0.7, 0.0)]:    # jump, outside the area, back
        dec = gate.step(dataclasses.replace(bad, measured_pose=m))
        assert dec.packet.desired_pose[:2] == (0.0, 0.0), m
    dec = gate.step(dataclasses.replace(bad, measured_pose=(0.01, 0.0, 0.7, 0.0)))
    assert dec.packet.desired_pose[:2] in ((0.0, 0.0), (0.01, 0.0))


def test_measured_pose_outside_the_flight_area_is_not_the_hold(outputs):
    """A tracked, continuous measured xy 0.3 m beyond the flight area's edge
    (more than the 0.25 m overshoot allowance) is not trusted as a hold."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(1.48, 0.0, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(1.2, 0.0, 0.7, 0.0)))
    dec = gate.step(dataclasses.replace(o, measured_pose=(1.5, 0.0, 0.7, 0.0), centering_active=True,
                                        desired_pose=(-1.2, -1.0, 0.7, 0.0)))
    assert dec.faults == ["centering-entry-jump"] and dec.packet.desired_pose[:2] == (1.2, 0.0)
    gate = OutputGate(DEFAULT, Mode.MONITOR)                                        # inside the allowance: trusted
    o = dataclasses.replace(o, measured_pose=(1.38, 0.0, 0.7, 0.0))
    gate.step(dataclasses.replace(o, desired_pose=(1.2, 0.0, 0.7, 0.0)))
    dec = gate.step(dataclasses.replace(o, measured_pose=(1.4, 0.0, 0.7, 0.0), centering_active=True,
                                        desired_pose=(-1.2, -1.0, 0.7, 0.0)))
    assert dec.packet.desired_pose[:2] == (1.4, 0.0)


def test_centring_origin_never_comes_from_an_untracked_pose(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(dataclasses.replace(outputs, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    lost = dataclasses.replace(outputs, measured_pose=(0.05, -0.85, 0.7, 0.0), is_tracking=0.0, centering_active=True,
                               desired_pose=(1.0, 0.5, 0.7, 0.0))
    assert gate.step(lost).faults[0] == "centering-entry-jump"
    assert gate.center_origin == (0.0, -0.9)


def _landing(gate, outputs):
    """AUTOLAND, operator armed with takeoff selected, centring; the arbiter
    lowers Takeoff/Land on the returned tick."""
    H = (0.0, 0.0, 0.7, 0.0)
    o = dataclasses.replace(outputs, measured_pose=H, desired_pose=H, operator_triggers=(1., 1., 0., 0.),
                            arbiter_triggers=(1., 1., 0., 0.))
    gate.step(o)
    gate.step(dataclasses.replace(o, centering_active=True))
    return dataclasses.replace(o, centering_active=True, arbiter_triggers=(1., 0., 0., 0.))


def test_fault_during_autonomous_land_never_raises_takeoff_land(outputs):
    """Raising trigger 2 mid-landing is an E-stop to the
    Commander. A LAND already applied continues; the bad pose is still held."""
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    land = _landing(gate, outputs)
    dec = gate.step(land)
    assert dec.packet.triggers[1] == 0.0 and dec.autoland_applied
    bad = dataclasses.replace(land, desired_pose=(math.nan, 0.0, 0.7, 0.0))
    for k in range(gate.escalate_ticks + 500):
        dec = gate.step(bad)
        assert dec.packet.triggers[1] == 0.0 and dec.autoland_applied, k
        assert dec.packet.desired_pose == (0.0, 0.0, 0.7, 0.0)
    assert dec.status is GateStatus.CENTRING_FAULT and gate.escalations == 1
    # The operator still ends it: dropping takeoff raises nothing new, disarm disarms.
    dec = gate.step(dataclasses.replace(bad, operator_triggers=(1., 0., 0., 0.), arbiter_triggers=(1., 0., 0., 0.)))
    assert dec.packet.triggers[1] == 0.0 and not dec.autoland_applied


def test_no_new_autonomous_land_while_centring_poses_are_refused(outputs):
    """LAND requested while the gate is refusing."""
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    land = _landing(gate, outputs)
    bad = dataclasses.replace(land, desired_pose=(math.nan, 0.0, 0.7, 0.0))
    gate.step(dataclasses.replace(bad, arbiter_triggers=(1., 1., 0., 0.)))
    dec = gate.step(bad)
    assert dec.status is GateStatus.CENTRING_REFUSED and not dec.autoland_applied and dec.packet.triggers[1] == 1.0
    dec = gate.step(land)                                                           # pose fine again: LAND starts
    assert dec.status is GateStatus.OK and dec.autoland_applied and dec.packet.triggers[1] == 0.0


def test_land_ends_when_the_arbiter_releases_it(outputs):
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    land = _landing(gate, outputs)
    assert gate.step(land).autoland_applied
    dec = gate.step(dataclasses.replace(land, arbiter_triggers=(1., 1., 0., 0.)))
    assert not dec.autoland_applied and dec.packet.triggers[1] == 1.0 and not gate.land_active
    bad = dataclasses.replace(land, desired_pose=(math.nan, 0.0, 0.7, 0.0))
    assert not gate.step(bad).autoland_applied                                      # a new LAND: refused now


def test_refused_centring_entry_holds_the_vehicle_not_the_stale_waypoint(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    here = (0.2, -0.5, 0.7, 0.0)
    o = dataclasses.replace(outputs, measured_pose=here, is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(1.2, 1.0, 0.7, 0.0)))               # waypoint still far away
    dec = gate.step(dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0)))
    assert dec.faults == ["centering-entry-jump"] and dec.packet.desired_pose[:2] == here[:2]


def test_intermittent_centring_refusals_do_not_escalate(outputs):
    """The 2 s escalation timer counts consecutive refusals; an accepted pose resets it."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    o = dataclasses.replace(outputs, measured_pose=(0.0, -0.9, 0.7, 0.0), is_tracking=1.0)
    gate.step(dataclasses.replace(o, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    bad = dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0))
    good = dataclasses.replace(o, centering_active=True, desired_pose=(0.0, -0.9, 0.7, 0.0))
    for _ in range(3):
        for _ in range(gate.escalate_ticks - 1):
            assert gate.step(bad).status is GateStatus.CENTRING_REFUSED
        assert gate.step(good).status is GateStatus.OK
    assert gate.escalations == 0 and gate.centring_refusals == 3 * (gate.escalate_ticks - 1)


def test_persistent_centring_refusals_escalate_to_a_latched_fault(outputs):
    gate = OutputGate(DEFAULT, Mode.AUTOLAND)
    n = int(round(DEFAULT.safety.centring_refusal_escalate_s / DEFAULT.model.step_size))
    assert n == 2000
    o = dataclasses.replace(outputs, measured_pose=(0.0, -0.9, 0.7, 0.0), is_tracking=1.0,
                            operator_triggers=(1.0, 1.0, 0.0, 0.0), arbiter_triggers=(1.0, 1.0, 0.0, 0.0))
    gate.step(dataclasses.replace(o, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    c = dataclasses.replace(o, centering_active=True, desired_pose=(-1.2, -1.0, 0.7, 0.0))   # NaN-hold corner
    for k in range(1, n):
        dec = gate.step(c)
        assert dec.status is GateStatus.CENTRING_REFUSED, k
    dec = gate.step(c)
    assert dec.status is GateStatus.CENTRING_FAULT and "centring-refusals-escalated" in dec.faults
    assert gate.escalations == 1 and gate.centring_refusals == n
    assert dec.packet.desired_pose[:2] == (0.0, -0.9)
    # Latched: an acceptable pose, or the arbiter asking for LAND, does not resume autonomy.
    land = dataclasses.replace(o, centering_active=True, desired_pose=(0.0, -0.9, 0.7, 0.0),
                               arbiter_triggers=(1.0, 0.0, 0.0, 0.0))
    dec = gate.step(land)
    assert dec.status is GateStatus.CENTRING_FAULT and dec.packet.triggers[1] == 1.0 and not dec.autoland_applied
    assert "centring-fault-hold" in dec.faults and gate.escalations == 1
    # Centring ends (operator lands / disarms): cleared.
    dec = gate.step(dataclasses.replace(o, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    assert dec.status is GateStatus.OK and not dec.faults


def test_entering_centring_from_hold_pose_is_not_a_fault(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(dataclasses.replace(outputs, desired_pose=(0.0, -0.9, 0.7, 0.0)))
    dec = gate.step(dataclasses.replace(outputs, centering_active=True, desired_pose=(0.01, -0.88, 0.7, 0.0)))
    assert not dec.faults


def test_abort_hold_height_step_is_accepted(outputs):
    """There is no separate height-step limit: the abort hold steps 0.70 -> 0.55 m
    in one tick while centring (seen in the E-stop scenario); z stays bounded by
    the [0.55, 0.70] m range check."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    c = dataclasses.replace(outputs, centering_active=True, desired_pose=(0.0, -0.9, 0.7, 0.0))
    gate.step(c)
    assert not gate.step(dataclasses.replace(c, desired_pose=(0.0, -0.9, 0.55, 0.0))).faults
    assert "desired-height-out-of-range" in gate.step(dataclasses.replace(c, desired_pose=(0.0, -0.9, 0.5, 0.0))).faults


def test_nothing_valid_yet_reports_no_tracking(outputs):
    """Until one desired pose has been accepted the gate holds on the measured
    pose and reports isTracking = 0, even with a good measured pose."""
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    dec = gate.step(dataclasses.replace(outputs, desired_pose=(math.nan, 0, 0.7, 0), is_tracking=1.0))
    assert dec.packet.is_tracking == 0.0 and dec.packet.desired_pose == (0.0, -0.9, 0.6, 0.0)
    dec = gate.step(outputs)
    assert dec.packet.is_tracking == 1.0


def test_nonfinite_measured_reports_no_tracking(outputs):
    gate = OutputGate(DEFAULT, Mode.MONITOR)
    gate.step(outputs)
    dec = gate.step(dataclasses.replace(outputs, measured_pose=(math.nan, 0, 0, 0)))
    assert dec.packet.is_tracking == 0.0 and dec.packet.is_finite()


# ---------------------------------------------------------------- runtime
def _random_raw(rng: random.Random, k: int) -> RawInputs:
    t = k * 0.001
    junk_f = [math.nan, math.inf, -math.inf, 1e308, -1e308, 0.0, 1.0, -1.0]
    def f():
        return rng.choice(junk_f) if rng.random() < 0.2 else rng.uniform(-3, 3)
    opti = None
    if rng.random() < 0.2:
        opti = rng.choice([OptiTrackSample((f(), f(), f()), (f(), f(), f()), f()),
                           OptiTrackSample((0.0, 0.7, 0.0), (0, 0, 0), 1.0), object(), None])
    joy = None
    if rng.random() < 0.2:
        joy = rng.choice([JoystickSample(new=f(), buttons=tuple(f() for _ in range(32)), x=f(), y=f(), z=f(),
                                         rx=f(), ry=f(), rz=f()),
                          JoystickSample(buttons=(1.0,)), None])
    vb = b""
    if rng.random() < 0.1:
        vb = rng.choice([bytes(rng.getrandbits(8) for _ in range(rng.randint(1, 120))),
                         VISION.pack(1, f(), f(), f(), float(k), NOW + t + rng.uniform(-1, 1))])
    return RawInputs(t, NOW + t, opti, joy, vb, rng.choice([0.0, 1.0, 2.0, f()]), f(), f())


@pytest.mark.parametrize("mode", list(Mode))
@pytest.mark.parametrize("seed", range(3))
def test_runtime_fuzz_never_crashes_never_emits_nonfinite(mode, seed):
    rng = random.Random(seed)
    rt = MissionRuntime(DEFAULT, mode)
    for k in range(4000):
        rec = rt.tick(_random_raw(rng, k))
        assert rec is not None, rt.failed
        if rec.decision.packet is not None:
            assert rec.decision.packet.is_finite()
            assert all(v in (0.0, 1.0) for v in rec.decision.packet.triggers)


def test_runtime_fail_stop(monkeypatch):
    rt = MissionRuntime(DEFAULT, Mode.MONITOR)
    assert rt.tick(RawInputs(0.0, NOW)) is not None
    monkeypatch.setattr(rt.core, "step", lambda *_: 1 / 0)
    assert rt.tick(RawInputs(0.001, NOW)) is None and "ZeroDivisionError" in rt.failed
    monkeypatch.undo()
    assert rt.tick(RawInputs(0.002, NOW)) is None         # latched until reset
    rt.reset()
    assert rt.tick(RawInputs(0.0, NOW)) is not None


@pytest.mark.parametrize("mode", [Mode.MONITOR, Mode.AUTOLAND])
def test_estop_reaches_wire_within_threshold(mode):
    rt = MissionRuntime(DEFAULT, mode)
    b = [0.0] * 32
    b[1] = 1.0
    t, k = 0.0, 0
    first = None
    while t < 2.0:
        if k % 10 == 0:
            b[0] = 1.0 if t >= 1.0 else 0.0
            joy = JoystickSample(new=float((k // 10) % 2), buttons=tuple(b), x=-1, y=0, z=0, rx=0, ry=0, rz=1)
        else:
            joy = None
        rec = rt.tick(RawInputs(t, NOW + t, OptiTrackSample((0, 0.7, 0), (0, 0, 0), 1.0) if k % 10 == 0 else None,
                                joy))
        if rec.decision.packet.triggers[2] == 1.0 and first is None:
            first = t
        k += 1
        t = k * 0.001
    assert first is not None and 1.0 + 0.2 - 1e-9 <= first <= 1.0 + 0.2 + 0.011
