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
from qdrone_autoland.safety import JoystickGuard, Mode, OutputGate, PoseGuard, VisionGuard

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
