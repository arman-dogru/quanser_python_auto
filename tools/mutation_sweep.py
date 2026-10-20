"""Mutation sweeps: break one rule at a time and require a test to fail.

Two sweeps, reported separately:

* ``logic``: the ported model logic (MATLAB Function blocks, block chains,
  wire formats, core wiring). Suites: ``parity`` (the Octave golden fixtures
  alone, to show how much the oracle constrains the port) and ``all``.
* ``safety``: the guards, the output gate, the runtime fail-stop and the
  socket/serve layer. Suite ``all``. Target: 0 survivors, or every survivor
  argued equivalent below.

Every mutant is one textual replacement that must match exactly once (the
sweep aborts otherwise, so a stale mutant cannot silently become a no-op).
The package is copied, mutated, and the tests run against the copy via
``QDRONE_AUTOLAND_SRC`` (tests/conftest.py refuses to run if the import does
not resolve to the copy).

Outcomes: ``killed`` (an assertion failed), ``killed-error`` (only errors:
exceptions, collection errors), ``timeout`` and ``survived``.

Usage::

    python tools/mutation_sweep.py --sweep logic --json docs/figures/data/mutation_logic.json
    python tools/mutation_sweep.py --sweep safety --json docs/figures/data/mutation_safety.json
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
P = "src/qdrone_autoland"
ARB, GATE, ABRT = f"{P}/logic/arbiter.py", f"{P}/logic/landing_gate.py", f"{P}/logic/abort_diagnostic.py"
CONF, GRID, EXT = f"{P}/logic/yolo_confirmation.py", f"{P}/logic/grid_manager.py", f"{P}/logic/extract_object.py"
COMM, ML, BLK = f"{P}/logic/comm_loss.py", f"{P}/logic/_matlab.py", f"{P}/blocks.py"
CORE, PROTO, CFG = f"{P}/mission_server.py", f"{P}/protocol.py", f"{P}/config.py"
SAFE, RT, SERVE, SOCK = f"{P}/safety.py", f"{P}/runtime.py", f"{P}/io/serve.py", f"{P}/io/sockets.py"

# (id, file, rule, old, new)
LOGIC = [
    # arbiter
    ("arb-armed-threshold", ARB, "armed is trig1 > 0.5", "armed = trig[0] > 0.5", "armed = trig[0] >= 0.5"),
    ("arb-reset-or", ARB, "latch resets on disarm OR takeoff low",
     "if not armed or not takeoff_selected:", "if not armed and not takeoff_selected:"),
    ("arb-ready-threshold", ARB, "ready > 0.5", "elif landing_ready > 0.5:", "elif landing_ready >= 0.5:"),
    ("arb-lower-index", ARB, "only trigger 2 lowered", "out[1] = 0.0", "out[2] = 0.0"),
    ("arb-no-reset", ARB, "latch clears", "            self.latched = False\n        elif", "            pass\n        elif"),
    # gate
    ("gate-stage", GATE, "ready only at stage 5", "ready_stage: int = 5", "ready_stage: int = 4"),
    ("gate-fresh-10", GATE, "10 fresh frames", "fresh_frames: int = 10", "fresh_frames: int = 9"),
    ("gate-xy-strict", GATE, "xy error > 0.08 rejects",
     "measured_pose[1] - desired_pose[1]) > self.xy_tolerance", "measured_pose[1] - desired_pose[1]) >= self.xy_tolerance"),
    ("gate-height-tol", GATE, "height tolerance 0.04", "height_tolerance: float = 0.04", "height_tolerance: float = 0.05"),
    ("gate-stale-strict", GATE, "stale is > 0.5 s",
     "stale = self.last_fresh < 0 or clock_time - self.last_fresh > self.stale_seconds",
     "stale = self.last_fresh < 0 or clock_time - self.last_fresh >= self.stale_seconds"),
    ("gate-abort-priority", GATE, "abort checked before vision",
     "        elif descent_abort > 0.5:\n            self._clear()\n            reason = ABORT\n",
     "        elif descent_abort > 1.5:\n            self._clear()\n            reason = ABORT\n"),
    ("gate-stale-restart", GATE, "count restarts after stale", "                if stale:\n                    self.count = 0\n",
     ""),
    ("gate-tracking-finite", GATE, "non-finite desired rejected", " or not all_finite(desired_pose)", ""),
    ("gate-ready-recheck", GATE, "ready rechecks freshness",
     "and clock_time - self.last_fresh <= self.stale_seconds)", "and True)"),
    # abort diagnostic
    ("abort-reset-cond", ABRT, "reset only at stage 0 without abort",
     "if descent_stage == 0 and descent_abort <= 0.5:", "if descent_stage == 0:"),
    ("abort-floor-strict", ABRT, "below floor is z < 0.45", "z < self.floor", "z <= self.floor"),
    ("abort-unknown", ABRT, "no cause -> 16", "bits = UNKNOWN", "bits = 0"),
    ("abort-latch-first", ABRT, "first abort latched", "elif descent_abort > 0.5 and not self.seen_abort:",
     "elif descent_abort > 0.5:"),
    ("abort-tracking-bit", ABRT, "tracking bit 2", "bits += TRACKING_LOST", "bits += INVALID_POSE"),
    # confirmation
    ("conf-new-frame", CONF, "repeated frame ignored", "if takeoff and new_vision and v[4] != self.last_frame:",
     "if takeoff and new_vision:"),
    ("conf-conf-geq", CONF, "confidence >= threshold", "v[3] >= self.min_confidence", "v[3] > self.min_confidence"),
    ("conf-bounds-u", CONF, "u <= width", "v[1] <= self.width", "v[1] < self.width"),
    ("conf-bounds-v0", CONF, "v >= 0", "v[2] >= 0 and", "v[2] > 0 and"),
    ("conf-release-10", CONF, "release after 10 bad", "if self.active and self.bad >= self.release_frames:",
     "if self.active and self.bad >= self.release_frames - 1:"),
    ("conf-stale", CONF, "stale > 0.5 s drops", "clock_time - self.last_fresh > self.stale_seconds",
     "clock_time - self.last_fresh > 2 * self.stale_seconds"),
    ("conf-bad-reset", CONF, "valid frame resets bad", "                self.bad = 0\n                if self.good",
     "                if self.good"),
    ("conf-takeoff-low-fresh", CONF, "takeoff low refreshes lastFresh",
     "            self.active = False\n            self.last_fresh = float(clock_time)\n",
     "            self.active = False\n"),
    ("conf-capture-pose", CONF, "hold pose captured at confirmation",
     "self.captured = tuple(float(x) for x in measured_pose)", "pass"),
    # grid manager
    ("grid-wp-rounding", GRID, "waypoints at mat2str 6 digits", 'float(f"{v:.6g}")', "float(v)"),
    ("grid-floor-strict", GRID, "abort when z < 0.45", "mp[2] < self.floor", "mp[2] <= self.floor"),
    ("grid-max-step", GRID, "0.005 m per frame", "if n > self.max_step:", "if n > 2 * self.max_step:"),
    ("grid-px-tol", GRID, "pixel tolerance inclusive", "norm2(*pixel_error) <= self.px_tol",
     "norm2(*pixel_error) < self.px_tol"),
    ("grid-yaw-sign", GRID, "rotation by yaw - yaw0", "a = mp[3] - self.yaw0", "a = mp[3] + self.yaw0"),
    ("grid-rot-sy", GRID, "rotation sy", "sy = s * bx + c * by", "sy = -s * bx + c * by"),
    ("grid-map-scale", GRID, "map scales with height", "cmap = self.base_map * (h / self.h0)", "cmap = self.base_map"),
    ("grid-offset-solve", GRID, "target = centre - map\\offset",
     "target = self.image_center - np.linalg.solve(cmap, self.camera_offset)", "target = self.image_center"),
    ("grid-disp-limit", GRID, "0.5 m displacement limit", "if on > self.max_disp:", "if on > 2 * self.max_disp:"),
    ("grid-stage2-height", GRID, "stage 2 targets 0.60", "self.center_pose[2] = self.h1", "self.center_pose[2] = self.h2"),
    ("grid-xy-settle", GRID, "xy settle 0.12", "xy_error <= self.xy_tol", "xy_error <= 2 * self.xy_tol"),
    ("grid-hold-clamp", GRID, "abort hold height clamp", "self.center_pose[2] = clamp(mp[2], *self.hold_lim)",
     "self.center_pose[2] = mp[2]"),
    ("grid-dwell", GRID, "dwell 100 samples at waypoint", "                if self.dwell >= self.dwell_samples:\n                    self.visited",
     "                if self.dwell >= self.dwell_samples - 1:\n                    self.visited"),
    ("grid-reverse", GRID, "reverse when last waypoint closer", "self.reverse_order = d_last < d_first",
     "self.reverse_order = d_last > d_first"),
    ("grid-centered-stage5", GRID, "centered needs stage 5",
     "centered = self.center_good >= self.good_frames and self.descent_mode == 5",
     "centered = self.center_good >= self.good_frames"),
    ("grid-tracking-enter", GRID, "centering needs tracking",
     "if takeoff and confirmed_pad and is_tracking > 0.5 and valid_pose and not self.center_mode:",
     "if takeoff and confirmed_pad and valid_pose and not self.center_mode:"),
    # extract / comm / matlab
    ("ext-range", EXT, "index > n -> zeros", "if index <= 0 or index > n:", "if index <= 0 or index >= n + 1:"),
    ("ext-tracking", EXT, "tracking at index + 6n", "tracking = at(index + n * 6)", "tracking = at(index + n * 5)"),
    ("comm-time", COMM, "silent first second", "if time > self.after_s:", "if time >= self.after_s:"),
    ("comm-error", COMM, "error >= 0 healthy", "state == 2 and error >= 0", "state == 2 and error > 0"),
    ("ml-max-nan", ML, "max ignores NaN", "    if math.isnan(a):\n        return b\n    if math.isnan(b):\n        return a\n    return a if a >= b else b",
     "    return a if a >= b else b"),
    # block chains
    ("blk-bias", BLK, "rigid body offset subtracted", "self.bias = np.array([0.0, 0.0, -m.rigid_body_height_offset])",
     "self.bias = np.array([0.0, 0.0, m.rigid_body_height_offset])"),
    ("blk-r-transpose", BLK, "xyz_RSF = R' * xyz_MF", "    rt = R_RSF_TO_MF.T\n", "    rt = R_RSF_TO_MF\n"),
    ("blk-euler-order", BLK, "YZX inputs (rot2, rot3, rot1)", "euler_yzx_to_matrix(rot[1], rot[2], rot[0])",
     "euler_yzx_to_matrix(rot[0], rot[1], rot[2])"),
    ("blk-unwrap", BLK, "yaw unwrapped", "        return x + self.offset", "        return x"),
    ("blk-threshold-dur", BLK, "threshold needs full duration", "return t - self.since >= self.duration - 1e-12",
     "return True"),
    ("blk-takeoff-axis", BLK, "takeoff when Rz >= -0.5", "takeoff = 1.0 if s.rz >= self.takeoff_threshold else 0.0",
     "takeoff = 1.0 if s.rz > 0.5 else 0.0"),
    ("blk-issue-zero-arm", BLK, "joystick issue zeroes arm", "self.out = JoystickOutput((ok * arm,",
     "self.out = JoystickOutput((arm,"),
    ("blk-estop-button", BLK, "E-stop is button 1", "estop = 1.0 if self.estop_timer.step(t, buttons[0]) else 0.0",
     "estop = 1.0 if self.estop_timer.step(t, buttons[2]) else 0.0"),
    ("blk-home-height", BLK, "home z + 0.70", "return (x, y, z + self.home_height, yaw)", "return (x, y, z, yaw)"),
    ("blk-home-after5", BLK, "home only after 5 s", "if t > self.enable_after and pulse:", "if pulse:"),
    ("blk-pulse-check", BLK, "conditioning waits for first edge", "if self.pulses >= 1:", "if True:"),
    # core wiring / protocol
    ("core-arbiter-ready", CORE, "arbiter fed by gate", "arb = self.arbiter.step(trig, 1.0 if gate.landing_ready else 0.0)",
     "arb = self.arbiter.step(trig, 0.0)"),
    ("core-packet-triggers", CORE, "packet triggers from arbiter",
     "packet = CommandPacket(arb.commander_triggers,", "packet = CommandPacket(trig,"),
    ("core-rate-hold", CORE, "rate transition holds", "        if x.optitrack is not None:\n",
     "        if x.optitrack is None:\n            self.data7 = (0.0,) * 7\n        else:\n"),
    ("proto-endian", PROTO, "little endian", 'VISION = struct.Struct("<6d")', 'VISION = struct.Struct(">6d")'),
    ("proto-frame-order", PROTO, "column-major frames", 'return gray.tobytes(order="F")', 'return gray.tobytes(order="C")'),
    ("proto-newest", PROTO, "newest record used", "newest = bytes(self.buf[end - self.size:end])",
     "newest = bytes(self.buf[0:self.size])"),
]

SAFETY = [
    ("pg-nan", SAFE, "non-finite pose refused", "ok = len(pos) == 3 and len(ori) == 3 and _finite((*pos, *ori, trk))",
     "ok = len(pos) == 3 and len(ori) == 3"),
    ("pg-stale", SAFE, "stale pose -> lost", "if self.last_time is None or now - self.last_time > self.stale_s:",
     "if self.last_time is None:"),
    ("pg-lost-tracking", SAFE, "lost sample has tracking 0",
     "return OptiTrackSample(self.last_good.position, self.last_good.orientation, 0.0)",
     "return OptiTrackSample(self.last_good.position, self.last_good.orientation, 1.0)"),
    ("pg-catch", SAFE, "garbage pose objects caught", "except (AttributeError, TypeError, ValueError):\n                ok = False",
     "except ValueError:\n                ok = False"),
    ("vg-finite", SAFE, "non-finite vision refused", '        if not _finite(v):\n            return "non-finite"\n', ""),
    ("vg-flag", SAFE, "detected must be 0/1", "if detected not in (0.0, 1.0):", "if detected not in (0.0, 1.0, 2.0):"),
    ("vg-frame", SAFE, "frame integer >= 0", "if frame < 0 or frame != math.floor(frame):", "if frame < -1:"),
    ("vg-conf", SAFE, "confidence in [0,1]", "if detected == 1.0 and not (0.0 <= conf <= 1.0):", "if False:"),
    ("vg-future", SAFE, "future timestamps refused", "if age < -self.future_tol:", "if age < -100 * self.future_tol:"),
    ("vg-stale", SAFE, "stale observation refused", "if age > self.max_age:", "if age > 10 * self.max_age:"),
    ("vg-resync", SAFE, "resync after 3 structural", "if self.consecutive_malformed >= self.RESYNC_AFTER:", "if False:"),
    ("vg-resync-flag", SAFE, "resync requests reconnect", "                    self.resync_requested = True\n", ""),
    ("vg-accept-reset", SAFE, "accepted record resets counter",
     "        self.consecutive_malformed = 0\n        self.vector = v\n", "        self.vector = v\n"),
    ("jg-stale", SAFE, "stale joystick -> disconnected",
     "if self.last_sample is None or self.last_time is None or now - self.last_time > self.stale_s:",
     "if self.last_sample is None or self.last_time is None:"),
    ("jg-buttons", SAFE, "32 buttons required", "ok = _finite(values) and len(sample.buttons) == 32", "ok = _finite(values)"),
    ("jg-finite", SAFE, "finite joystick required", "ok = _finite(values) and len(sample.buttons) == 32",
     "ok = len(sample.buttons) == 32"),
    ("og-estop", SAFE, "E-stop from operator", "triggers = [op[0], op[1], op[2], op[3]]",
     "triggers = [op[0], op[1], arb[2], op[3]]"),
    ("og-arm", SAFE, "arm from operator", "triggers = [op[0], op[1], op[2], op[3]]",
     "triggers = [arb[0], op[1], op[2], op[3]]"),
    ("og-issue", SAFE, "joystick issue from operator", "triggers = [op[0], op[1], op[2], op[3]]",
     "triggers = [op[0], op[1], op[2], arb[3]]"),
    ("og-mode", SAFE, "LAND only in AUTOLAND", "            if self.mode is Mode.AUTOLAND:\n", "            if True:\n"),
    ("og-autoland-arm", SAFE, "LAND only while armed", "if arb[1] == 0.0 and op[1] == 1.0 and op[0] == 1.0:",
     "if arb[1] == 0.0 and op[1] == 1.0:"),
    ("og-dryrun", SAFE, "dry-run sends nothing", "return GateDecision(None if self.mode is Mode.DRY_RUN else packet,",
     "return GateDecision(packet,"),
    ("og-geofence", SAFE, "geofence enforced",
     "if not (self.x_lo - e <= x <= self.x_hi + e and self.y_lo - e <= y <= self.y_hi + e):", "if False:"),
    ("og-height", SAFE, "height range enforced", "if not (self.z_lo - e <= z <= self.z_hi + e):", "if False:"),
    ("og-step", SAFE, "centering step limit", "if math.hypot(x - px, y - py) > self.center_step + e:", "if False:"),
    ("og-hold-last", SAFE, "fault holds last good pose", "            faults.append(reason)\n            desired = self.last_desired\n",
     "            faults.append(reason)\n            desired = tuple(float(v) for v in out.desired_pose)\n"),
    ("og-measured-nan", SAFE, "non-finite measured -> no tracking", '            faults.append("measured-non-finite")\n            tracking = 0.0\n',
     '            faults.append("measured-non-finite")\n'),
    ("og-bool", SAFE, "operator triggers coerced to 0/1", "op = tuple(1.0 if v > 0.5 else 0.0 for v in op)", "pass"),
    ("og-desired-finite", SAFE, "non-finite desired refused", "if len(d) != 4 or not _finite(d):", "if len(d) != 4:"),
    ("rt-failstop", RT, "exceptions fail-stop", "            self.failed = f\"{type(exc).__name__}: {exc}\"\n",
     "            raise\n"),
    ("rt-latched", RT, "failure latched", "        if self.failed is not None:\n            return None\n", ""),
    ("rt-guard-pose", RT, "pose guard on path", "pose = self.pose_guard.step(raw.t, raw.optitrack)", "pose = raw.optitrack"),
    ("rt-guard-joy", RT, "joystick guard on path", "joy = self.joystick_guard.step(raw.t, raw.joystick)",
     "joy = raw.joystick or JoystickSample()"),
    ("sv-disconnect", SERVE, "fail-stop disconnects Commander", "        self.commander.disconnect()\n", ""),
    ("sv-overrun", SERVE, "overrun fail-stops", "            if lag > max_lag:", "            if False:"),
    ("sv-resync", SERVE, "resync closes vision socket", "            self.vision.close()                     # reconnect",
     "            pass  # reconnect"),
    ("sv-dryrun", SERVE, "only gated packets sent", "        if rec.decision.packet is not None:",
     "        if rec.decision.candidate is not None:\n            rec.decision.packet = rec.decision.candidate\n        if True:"),
    ("so-partial", SOCK, "partial write completed before next packet", "            self.pending = rest\n            if self.pending:",
     "            self.pending = b\"\"\n            if self.pending:"),
    ("so-size", SOCK, "112-byte payload enforced", "        if len(payload) != COMMAND.size:\n", "        if False:\n"),
    ("so-drain", SOCK, "vision backlog drained", "            chunks.append(data)\n", "            chunks.append(data)\n            break\n"),
    # Independent review mutants (22, one no longer applies: the height-step check was removed as dead code).
    ("rv-estop-hold-2s", CFG, "E-stop hold 0.2 s", "estop_threshold_s: float = 0.2", "estop_threshold_s: float = 2.0"),
    ("rv-estop-never-out", BLK, "E-stop reaches the triggers", "(ok * arm, ok * takeoff, float(estop), issue)",
     "(ok * arm, ok * takeoff, 0.0, issue)"),
    ("rv-issue-5-axes", BLK, "issue needs all six axes at -1", "all(a == -1.0 for a in axes)",
     "all(a == -1.0 for a in axes[:5])"),
    ("rv-takeoff-not-gated", BLK, "takeoff masked by joystick issue", "(ok * arm, ok * takeoff,", "(ok * arm, takeoff,"),
    ("rv-issue-threshold-2s", CFG, "joystick issue after 0.2 s", "joystick_threshold_s: float = 0.2",
     "joystick_threshold_s: float = 2.0"),
    ("rv-jg-stale-10x", SAFE, "joystick stale window",
     "now - self.last_time > self.stale_s:\n            if self.last_sample",
     "now - self.last_time > 10 * self.stale_s:\n            if self.last_sample"),
    ("rv-jg-malformed-keeps-last", SAFE, "malformed joystick forgets last good",
     '            self.refused["malformed"] += 1\n            self.last_time, self.last_sample = None, None\n',
     '            self.refused["malformed"] += 1\n'),
    ("rv-pg-stale-10x", SAFE, "pose stale window", "if self.last_time is None or now - self.last_time > self.stale_s:",
     "if self.last_time is None or now - self.last_time > 10 * self.stale_s:"),
    ("rv-pg-tracking-forced", SAFE, "Motive tracking flag kept", "1.0 if trk > 0.5 else 0.0)", "1.0)"),
    ("rv-vg-age-ignored", SAFE, "detection age used", "age = now - ts", "age = 0.0"),
    ("rv-vg-stale-2x", SAFE, "vision stale 0.5 s", "if age > self.max_age:", "if age > 2 * self.max_age:"),
    ("rv-vg-future-2x", SAFE, "future tolerance 50 ms", "if age < -self.future_tol:", "if age < -2 * self.future_tol:"),
    ("rv-vg-frac-frame", SAFE, "fractional frame refused", "if frame < 0 or frame != math.floor(frame):",
     "if frame < 0:"),
    ("rv-og-step-10x", SAFE, "centring step 5 mm", "> self.center_step + e:", "> 10 * self.center_step + e:"),
    ("rv-og-disp-off", SAFE, "0.5 m displacement limit",
     "y - self.center_origin[1]) > self.max_disp + e:", "y - self.center_origin[1]) > 1e9:"),
    ("rv-og-none-tracking", SAFE, "nothing valid yet -> tracking 0",
     "self.z_lo), self.z_hi), measured[3])\n            tracking = 0.0\n", "self.z_lo), self.z_hi), measured[3])\n"),
    ("rv-og-meas-none-tracking", SAFE, "no measured pose yet -> tracking 0",
     "            measured = (0.0, 0.0, 0.0, 0.0)\n            tracking = 0.0\n", "            measured = (0.0, 0.0, 0.0, 0.0)\n"),
    ("rv-rt-nonfinite-check", RT, "runtime re-checks finiteness",
     "if decision.packet is not None and not decision.packet.is_finite():", "if False:"),
    ("rv-sv-maxlag-100x", SERVE, "overrun limit", "            if lag > max_lag:", "            if lag > 100 * max_lag:"),
    ("rv-so-state-always-2", SOCK, "state 0 without a client", "return 2.0 if self.client is not None else 0.0",
     "return 2.0"),
    ("rv-og-arbiter-fault-off", SAFE, "arbiter tampering flagged",
     "if (arb[0], arb[2], arb[3]) != (op[0], op[2], op[3]):", "if False:"),
    # Added with the review fixes
    ("og-entry-jump", SAFE, "jump on centring entry refused", '                return "centering-entry-jump"\n',
     "                pass\n"),
    ("og-origin-refused", SAFE, "refused entry pose is not the origin",
     "cands = [tuple(out.desired_pose[:2])] if accepted else []", "cands = [tuple(out.desired_pose[:2])]"),
    ("og-late-origin", SAFE, "first accepted centring pose becomes the origin",
     "        elif self.center_origin is None and reason is None:", "        elif False:"),
    ("sv-input-failstop", SERVE, "input exceptions fail-stop",
     '            self._fail(f"input: {type(exc).__name__}: {exc}")\n', "            raise\n"),
    ("sv-no-busy-spin", SERVE, "hybrid sleep", "sleep(-lag - spin_s if -lag > spin_s else 0)", "pass"),
]

# Survivors argued equivalent (or boundary-only). Each is kept in the sweep so
# the argument is re-checked whenever the code changes.
EQUIVALENT: dict[str, str] = {
    "arb-ready-threshold": "Equivalent in composition: landingReady is the gate's logical output and the core passes "
                           "1.0/0.0, so > 0.5 and >= 0.5 cannot differ. Only a unit-level input of exactly 0.5 "
                           "would tell them apart.",
    "gate-ready-recheck": "The ready expression is only reached when not (no new sample and stale): either a new "
                          "sample just set lastFresh = clockTime (finite, checked earlier) or the gate is not stale, "
                          "i.e. clockTime - lastFresh <= 0.5 already holds.",
    "grid-max-step": "A step is only taken when the pixel error exceeds 20 px; with this map the smallest "
                     "resulting step is sigma_min(map) * (0.50/0.70) * 20 px = 0.0227 m (sigma_min = 0.0015883), so n > 0.005 and "
                     "n > 0.010 always agree and the clamp result is the same (config-dependent equivalence).",
    "rv-takeoff-not-gated": "The FrSKY mapping already computes takeoff = 0 from rz = -1 before the issue latches, "
                            "and the mapping is held while the issue is set, so the held takeoff is already 0.",
    "rv-og-meas-none-tracking": "measured is None only if no finite measured pose was ever seen; that same tick "
                                "took the non-finite branch, which already set tracking = 0.",
    "rv-rt-nonfinite-check": "Defence in depth: every packet field is finite after the OutputGate (desired and "
                             "measured are finite-checked, triggers are 0/1, the timestamp is sanitised).",
    "ml-max-nan": "Every call site passes the possibly-NaN value as the first operand and a finite constant "
                  "second; for a NaN first operand `a >= b` is False and the fallback already returns b, "
                  "exactly what the NaN branch does.",
}

SUITES = {
    "logic": {
        "parity": ["tests/test_parity.py"],
        "all": ["tests/test_parity.py", "tests/test_matlab_ports.py", "tests/test_blocks.py", "tests/test_protocol.py",
                "tests/test_spec.py", "tests/test_sim.py", "tests/test_safety.py"],
    },
    "safety": {
        "all": ["tests/test_safety.py", "tests/test_blocks.py", "tests/test_sockets.py", "tests/test_sim.py"],
    },
}
DESELECT = ["--deselect", "tests/test_spec.py::test_spec_is_fresh", "-m", "not vision and not octave"]


def _classify(proc: subprocess.CompletedProcess) -> str:
    out = proc.stdout + proc.stderr
    if proc.returncode == 0:
        return "survived"
    failed = re.search(r"(\d+) failed", out)
    if failed and int(failed.group(1)) > 0:
        return "killed"
    return "killed-error"


def run_one(mutant, suites, timeout: float) -> dict:
    mid, rel, rule, old, new = mutant
    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / "src"
        shutil.copytree(ROOT / "src", dst, ignore=shutil.ignore_patterns("*.egg-info", "__pycache__"))
        target = Path(td) / rel
        text = target.read_text()
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"mutant {mid}: pattern matched {count} times in {rel}")
        target.write_text(text.replace(old, new))
        env = {**os.environ, "QDRONE_AUTOLAND_SRC": str(dst), "PYTHONDONTWRITEBYTECODE": "1"}
        res = {"id": mid, "file": rel.removeprefix(P + "/"), "rule": rule}
        for suite, tests in suites.items():
            try:
                proc = subprocess.run([sys.executable, "-m", "pytest", "-q", "-x", "-p", "no:cacheprovider",
                                       *DESELECT, *tests], capture_output=True, text=True, env=env, cwd=ROOT,
                                      timeout=timeout)
                res[suite] = _classify(proc)
                if "expected under" in proc.stdout + proc.stderr:
                    raise SystemExit(f"mutant {mid}: tests did not import the mutated copy")
            except subprocess.TimeoutExpired:
                res[suite] = "timeout"
        return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sweep", choices=["logic", "safety"], required=True)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--timeout", type=float, default=300.0)
    args = ap.parse_args(argv)
    mutants = LOGIC if args.sweep == "logic" else SAFETY
    suites = SUITES[args.sweep]
    for m in mutants:                       # validate all patterns up front
        n = (ROOT / m[1]).read_text().count(m[3])
        if n != 1:
            raise SystemExit(f"mutant {m[0]}: pattern matched {n} times in {m[1]}")
    base = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", *DESELECT,
                           *suites["all"]], capture_output=True, text=True, cwd=ROOT)
    if base.returncode != 0:
        print(base.stdout[-3000:])
        raise SystemExit("baseline tests fail; fix them before sweeping")
    sel = [m for m in mutants if not args.only or m[0] in args.only]
    t0 = time.time()
    with ThreadPoolExecutor(args.jobs) as ex:
        results = list(ex.map(lambda m: run_one(m, suites, args.timeout), sel))
    for r in results:
        cols = " ".join(f"{s}={r[s]:13s}" for s in suites)
        print(f"{r['id']:24s} {cols} {r['rule']}", flush=True)
    summary = {s: {k: sum(1 for r in results if r[s] == k) for k in ("killed", "killed-error", "timeout", "survived")}
               for s in suites}
    survivors = [r["id"] for r in results if r["all"] == "survived"]
    unexplained = [s for s in survivors if s not in EQUIVALENT]
    report = {"sweep": args.sweep, "mutants": len(results), "summary": summary, "survivors": survivors,
              "equivalent": {k: v for k, v in EQUIVALENT.items() if k in survivors},
              "seconds": round(time.time() - t0, 1), "results": results}
    print(json.dumps({k: v for k, v in report.items() if k != "results"}, indent=1))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=1) + "\n")
    return 0 if not unexplained else 1


if __name__ == "__main__":
    sys.exit(main())
