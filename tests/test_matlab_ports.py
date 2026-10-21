"""pytest ports of the repository's MATLAB tests, assertion for assertion.

* matlab/test_phase9_landing_gate.m
* matlab/test_phase9_abort_diagnostics.m
* matlab/test_phase10_autoland_dry_run.m (behavioural half; the structural
  half is in test_spec.py)
"""
from __future__ import annotations

import math

from qdrone_autoland.logic.abort_diagnostic import AbortDiagnostic
from qdrone_autoland.logic.arbiter import AutolandArbiter
from qdrone_autoland.logic.landing_gate import LandingReadinessGate

MEASURED = (0.0, 0.0, 0.55, 0.0)


def test_phase9_landing_gate():
    g = LandingReadinessGate()
    desired = MEASURED
    for k in range(1, 13):
        o = g.step(4, 0, 1, 1, True, 1, MEASURED, desired, k * 0.20)
        assert not o.landing_ready and o.reason_code == 1, "Gate armed before stage 5."
    for k in range(1, 10):
        o = g.step(5, 0, 1, 1, True, 1, MEASURED, desired, 3 + k * 0.20)
        assert not o.landing_ready and o.fresh_count == k, "Gate counted or armed too early."
    o = g.step(5, 0, 1, 1, True, 1, MEASURED, desired, 5.0)
    assert o.landing_ready and o.fresh_count == 10 and o.reason_code == 0
    o = g.step(5, 0, 1, 1, False, 1, MEASURED, desired, 5.3)
    assert o.landing_ready and o.fresh_count == 10, "A brief gap unexpectedly cleared readiness."
    o = g.step(5, 0, 1, 1, False, 1, MEASURED, desired, 5.6)
    assert not o.landing_ready and o.fresh_count == 0 and o.reason_code == 6, "Stale vision did not disarm gate."
    o = g.step(5, 1, 1, 1, True, 1, MEASURED, desired, 5.8)
    assert not o.landing_ready and o.fresh_count == 0 and o.reason_code == 2
    o = g.step(5, 0, 1, 1, True, 0, MEASURED, desired, 6.0)
    assert not o.landing_ready and o.reason_code == 4
    o = g.step(5, 0, 1, 1, True, 1, (0.09, 0.0, 0.55, 0.0), desired, 6.2)
    assert not o.landing_ready and o.reason_code == 5
    o = g.step(5, 0, 1, 1, True, 1, (0.0, 0.0, 0.60, 0.0), desired, 6.4)
    assert not o.landing_ready and o.reason_code == 5


def test_phase9_abort_diagnostics():
    a = AbortDiagnostic()
    pose = (0.0, 0.0, 0.70, 0.0)
    o = a.step(1, 0, 1, 1, pose, 0.1, 10)
    assert o.cause_bits == 0 and o.age_at_abort == 0 and o.time_at_abort == 0
    o = a.step(1, 1, 0, 1, pose, 0.58, 10.58)
    assert o.cause_bits == 1 and abs(o.age_at_abort - 0.58) < 1e-9 and abs(o.time_at_abort - 10.58) < 1e-9
    o = a.step(1, 1, 1, 0, pose, 0.02, 10.60)
    assert o.cause_bits == 1 and abs(o.age_at_abort - 0.58) < 1e-9 and abs(o.time_at_abort - 10.58) < 1e-9
    assert a.step(0, 0, 0, 1, pose, 0, 11).cause_bits == 0
    assert a.step(1, 1, 1, 0, pose, 0.1, 12).cause_bits == 2
    a.step(0, 0, 0, 1, pose, 0, 13)
    bad = (math.nan, 0.0, 0.40, 0.0)
    assert a.step(1, 1, 0, 0, bad, 0.8, 14).cause_bits == 15


def test_phase10_autoland_dry_run_sequence():
    raw = [(0, 0, 0, 0), (1, 1, 0, 1), (1, 1, 0, 1), (1, 1, 0, 1), (1, 1, 0, 1), (1, 0, 0, 1), (1, 1, 0, 1),
           (0, 1, 0, 1), (1, 1, 1, 1), (1, 1, 1, 1)]
    ready = [0, 0, 1, 0, 0, 0, 0, 0, 1, 0]
    arb = AutolandArbiter()
    outs = [arb.step(r, q) for r, q in zip(raw, ready, strict=True)]
    cmd = [o.commander_triggers for o in outs]
    assert [(c[0], c[2], c[3]) for c in cmd] == [(r[0], r[2], r[3]) for r in raw]
    assert [c[1] for c in cmd] == [0, 1, 0, 0, 0, 0, 1, 1, 0, 0]
    assert [float(o.autoland_latched) for o in outs] == [0, 0, 1, 1, 1, 0, 0, 0, 1, 1]
