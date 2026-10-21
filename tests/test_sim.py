"""Closed-loop scenarios through the full runtime (guards -> core -> gate)."""
from __future__ import annotations

import functools

import numpy as np
import pytest

from qdrone_autoland import sim


@functools.cache
def run(name):
    return sim.run(name)


def test_nominal_autoland_lands_on_pad():
    r = run("nominal-autoland")
    s = r.signals
    assert set(np.unique(s["stage"])) == {0, 1, 2, 3, 4, 5}
    assert "landing_ready" in r.events and "landed" in r.events
    assert r.events["landed"] > r.events["landing_ready"]
    assert np.hypot(*(r.vehicle.pos[:2] - np.array(r.scenario.pad_xy))) < 0.05
    assert s["faults"].sum() == 0
    # LAND only after readiness, and only through the arbiter latch
    first_land = np.argmax((s["trig2"] == 0) & (s["trig1"] == 1) & (s["op_trig2"] == 1))
    assert s["latched"][first_land] == 1 and s["ready"][: first_land + 1].max() == 1


def test_model_packet_carries_arbiter_triggers():
    s = run("nominal-autoland").signals
    np.testing.assert_array_equal(s["model_trig2"], s["trig2"])     # AUTOLAND: wire == model
    assert s["model_trig2"].min() == 0
    m = run("monitor-only").signals
    assert m["model_trig2"].min() == 0 and m["trig2"][m["op_trig2"] == 1].min() == 1


def test_autoland_ends_with_floor_abort_bit():
    """Model behaviour worth knowing: descending below 0.45 m during LAND
    always latches descentAbort, so the abort diagnostic reports bit 8."""
    r = run("nominal-autoland")
    assert r.signals["abort_bits"][-1] == 8


def test_monitor_mode_never_lands_autonomously():
    r = run("monitor-only")
    s = r.signals
    assert s["latched"].max() == 1                  # the model wanted to land
    np.testing.assert_array_equal(s["trig2"], s["op_trig2"])   # Takeoff/Land is always the operator's
    assert "landed" not in r.events and abs(r.vehicle.pos[2] - 0.55) < 0.02


def test_vision_loss_in_descent_aborts_and_holds():
    r = run("vision-loss-in-descent")
    s = r.signals
    assert int(s["abort_bits"][-1]) & 1
    assert s["trig2"][-1] == 1 and "landed" not in r.events
    assert s["ready"][int(r.events["vision_lost"] / 0.001):].max() == 0


def test_tracking_loss_in_centering_aborts():
    r = run("tracking-loss-in-centering")
    s = r.signals
    assert int(s["abort_bits"][-1]) & 2
    assert s["tracking"][-1] == 0 and r.runtime.pose_guard.refused["stale"] >= 1
    assert "landed" not in r.events


def test_estop_reaches_wire():
    r = run("estop-in-search")
    s = r.signals
    i = int(round(20.0 / 0.001))
    assert s["trig3"][:i].max() == 0 and s["trig3"][i + 210:].min() == 1


def test_joystick_disconnect_disarms():
    r = run("joystick-disconnect")
    s = r.signals
    assert (s["trig1"][-1], s["trig2"][-1], s["trig4"][-1]) == (0, 0, 1)


def test_malformed_vision_is_refused_and_mission_continues():
    r = run("malformed-vision")
    g = r.runtime.vision_guard
    for reason in ("non-finite", "bad-detected-flag", "bad-confidence", "from-future", "stale"):
        assert g.refused[reason] >= 1, reason
    assert g.resyncs >= 2 and "landed" in r.events


def test_operator_manual_land():
    r = run("operator-manual-land")
    assert "landed" in r.events and r.signals["latched"].max() == 0


def test_dry_run_sends_nothing():
    from qdrone_autoland.runtime import MissionRuntime, RawInputs
    rt = MissionRuntime(mode=sim.Mode.DRY_RUN)
    for k in range(200):
        assert rt.tick(RawInputs(k * 0.001, sim.WALL0 + k * 0.001)).decision.packet is None


@pytest.mark.parametrize("name", list(sim.SCENARIOS))
def test_no_scenario_emits_gate_faults(name):
    assert run(name).signals["faults"].sum() == 0


# A subset of the 12x8 pad grid swept by tools/pad_sweep.py (full sweep: 94/96
# land, 0 gate faults).  It covers pads confirmed while the vehicle is still
# far short of its waypoint, which is where a centring-entry check anchored on
# the wrong pose stalls the mission, and pads confirmed at the area's edge.
PAD_SUBSET = [(0.1, -0.95), (-0.3, 0.407), (0.9, 0.95), (-1.1, 0.95), (1.1, -0.95), (0.5, -0.136), (-0.7, -0.679),
              # confirmed while the vehicle has overshot the flight area's edge (y = +-1.0 m) by a hair
              (-0.7, 0.95), (0.3, -0.679)]


@pytest.mark.parametrize("pad", PAD_SUBSET, ids=[f"{x:+.2f},{y:+.3f}" for x, y in PAD_SUBSET])
def test_autoland_lands_across_pad_positions(pad):
    r = sim.run_pad(pad)
    assert r["gate_faults"] == {} and r["escalations"] == 0
    assert r["landed"] and r["touchdown_error_m"] < 0.05


def test_pad_grid_matches_sweep_definition():
    pads = sim.pad_grid()
    assert len(pads) == 96 and pads[0] == (-1.1, -0.95) and pads[-1] == (1.1, 0.95)


def test_centring_fault_during_autonomous_land_never_raises_takeoff_land(monkeypatch):
    """Closed loop: the model's desired x is NaN for 2.5 s starting 50 ticks
    after the gate applied LAND. Trigger 2 must stay lowered (raising it
    mid-landing is an E-stop to the Commander) and the vehicle still lands."""
    import dataclasses as dc

    from qdrone_autoland import safety
    orig = safety.OutputGate.step
    state = {"k": 0, "land_k": None}

    def step(self, out):
        state["k"] += 1
        if state["land_k"] is None and self.land_active:
            state["land_k"] = state["k"]
        if state["land_k"] is not None and state["land_k"] + 50 < state["k"] < state["land_k"] + 2600:
            out = dc.replace(out, desired_pose=(float("nan"),) + tuple(out.desired_pose[1:]))
        return orig(self, out)
    monkeypatch.setattr(safety.OutputGate, "step", step)
    r = sim.run(dc.replace(sim.SCENARIOS["nominal-autoland"], duration=120.0))
    lk = state["land_k"]
    t2, gs = r.signals["trig2"], r.signals["gate_status"]
    assert lk is not None and "landed" in r.events
    assert max(gs[lk:]) == 2.0                                                      # it did escalate
    assert all(t2[k] == 0.0 for k in range(lk, len(t2))), "Takeoff/Land raised during LAND"
