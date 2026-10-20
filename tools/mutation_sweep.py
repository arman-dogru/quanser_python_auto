"""Mutation sweep: break each controller rule once and check a test fails.

Every mutant is a single textual replacement that must match exactly once
in its file (the sweep aborts otherwise, so a stale mutant cannot silently
become a no-op). For each mutant the package is copied to a temporary
directory, mutated, and the test suites are run against the copy with
``PYTHONPATH`` pointing at it (``quanser_landing.__file__`` is checked).

Two suites are run per mutant:

* ``parity``: ``tests/test_parity_octave.py`` only, i.e. the Octave golden
  fixtures. This measures how much the oracle alone constrains the port.
* ``all``: parity plus the unit and block tests.

Outcomes are labelled separately: ``killed`` (a test assertion failed),
``killed-error`` (an exception or collection error rather than a failed
assertion; a run with both counts as ``killed``), ``survived`` and
``timeout``.

Usage::

    python tools/mutation_sweep.py [--json outputs/mutation_sweep.json]
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
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = "src/quanser_landing"
PC = f"{PKG}/controllers/pose_command.py"
RC = f"{PKG}/controllers/recenter.py"
ML = f"{PKG}/controllers/_matlab.py"
LG = f"{PKG}/controllers/legacy.py"
CH = f"{PKG}/blocks/charts.py"

# (id, file, rule, old, new)
#
# Deliberately absent, because they would be equivalent mutants:
# * zeroing raw_forward/raw_right inside the recenter state-3 branch. The
#   counter only reaches 10 on a centred step, where both raw values are
#   already 0, so removing those two source lines cannot change any output.
# * the NaN guards inside mmax/mmin for the second operand: every call site
#   passes a constant there, so only clamp-level semantics are observable.
MUTANTS = [
    ("pc-deadband-x-inclusive", PC, "deadband inclusive (x)",
     "x_centered = abs(error_x) <= c.deadband_pixels", "x_centered = abs(error_x) < c.deadband_pixels"),
    ("pc-deadband-y-inclusive", PC, "deadband inclusive (y)",
     "y_centered = abs(error_y) <= c.deadband_pixels", "y_centered = abs(error_y) < c.deadband_pixels"),
    ("pc-deadband-40", PC, "deadband 40 px", "deadband_pixels: float = 40.0", "deadband_pixels: float = 41.0"),
    ("pc-both-axes-centered", PC, "centred needs both axes",
     "if x_centered and y_centered:", "if x_centered or y_centered:"),
    ("pc-alpha", PC, "alpha 0.25", "alpha: float = 0.25", "alpha: float = 0.3"),
    ("pc-smoothing-memory", PC, "smoothing keeps (1-alpha) memory",
     "s.smooth_forward = c.alpha * raw_forward + (1 - c.alpha) * s.smooth_forward",
     "s.smooth_forward = c.alpha * raw_forward + (1 - c.alpha) * s.smooth_right"),
    ("pc-decay-value", PC, "loss decay 0.7", "loss_decay: float = 0.7", "loss_decay: float = 0.75"),
    ("pc-decay-only-detected-0", PC, "decay only when detected == 0",
     "        if detected == 0:\n            s.smooth_forward = c.loss_decay",
     "        if detected != 1:\n            s.smooth_forward = c.loss_decay"),
    ("pc-decay-after-smoothing", PC, "decay applied after smoothing",
     "        s.smooth_forward = c.alpha * raw_forward + (1 - c.alpha) * s.smooth_forward\n",
     "        s.smooth_forward = c.alpha * raw_forward + (1 - c.alpha) * s.smooth_forward * "
     "(c.loss_decay if detected == 0 else 1)\n"),
    ("pc-detected-equals-1", PC, "detection branch is detected == 1", "        if detected == 1:\n",
     "        if detected:\n"),
    ("pc-sign-forward", PC, "forward sign", "raw_forward = -error_x / c.image_center_x",
     "raw_forward = error_x / c.image_center_x"),
    ("pc-sign-right", PC, "right sign", "raw_right = -error_y / c.image_center_y",
     "raw_right = error_y / c.image_center_y"),
    ("pc-norm-axis", PC, "forward normalized by 320", "raw_forward = -error_x / c.image_center_x",
     "raw_forward = -error_x / c.image_center_y"),
    ("pc-clamp-forward", PC, "forward clamp",
     "raw_forward = clamp(raw_forward, -c.max_norm_cmd, c.max_norm_cmd)", "raw_forward = raw_forward"),
    ("pc-clamp-right", PC, "right clamp",
     "raw_right = clamp(raw_right, -c.max_norm_cmd, c.max_norm_cmd)", "raw_right = raw_right"),
    ("pc-counter-reset-offcentre", PC, "counter resets when off-centre",
     "                s.centered_counter = 0.0\n                state = PadState.RECENTERING",
     "                state = PadState.RECENTERING"),
    ("pc-counter-reset-loss", PC, "counter resets on no detection",
     "            s.centered_counter = 0.0\n            state = PadState.NO_DETECTION",
     "            state = PadState.NO_DETECTION"),
    ("pc-required-10", PC, "10 updates for state 3", "centered_required: int = 10", "centered_required: int = 9"),
    ("pc-required-geq", PC, "state 3 at counter >= 10",
     "if s.centered_counter >= c.centered_required:", "if s.centered_counter > c.centered_required:"),
    ("pc-rotation-dx", PC, "yaw rotation dx",
     "dx_world = cy * forward_offset - sy * right_offset", "dx_world = cy * forward_offset + sy * right_offset"),
    ("pc-rotation-dy", PC, "yaw rotation dy",
     "dy_world = sy * forward_offset + cy * right_offset", "dy_world = cy * forward_offset + sy * right_offset"),
    ("pc-step-0.05", PC, "0.05 m position step", "max_position_step: float = 0.05", "max_position_step: float = 0.5"),
    ("pc-descent-gated", PC, "descent needs enable_descent",
     "if c.enable_descent and state == PadState.CENTERED_STABLE:", "if state == PadState.CENTERED_STABLE:"),
    ("pc-descent-step", PC, "dormant descent step 0.02", "descent_step: float = 0.02", "descent_step: float = 0.03"),
    ("pc-yaw-passthrough", PC, "yaw passed through",
     "cmd_pose=(x + dx_world, y + dy_world, desired_z, yaw)", "cmd_pose=(x + dx_world, y + dy_world, desired_z, 0.0)"),
    ("pc-forward-output", PC, "forward_cmd is the smoothed value",
     "forward_cmd=s.smooth_forward,", "forward_cmd=forward_offset,"),
    ("ml-clamp-nan", ML, "clamp keeps MATLAB NaN semantics (NaN -> low)",
     "return mmin(mmax(value, low), high)", "return min(max(value, low), high)"),
    ("rc-max-cmd", RC, "recenter scales by 0.20", "max_cmd: float = 0.20", "max_cmd: float = 0.25"),
    ("rc-descend-speed", RC, "recenter descend 0.05", "descend_speed: float = 0.05", "descend_speed: float = 0.06"),
    ("rc-deadband", RC, "recenter deadband inclusive",
     "x_centered = abs(error_x) <= c.deadband_pixels", "x_centered = abs(error_x) < c.deadband_pixels"),
    ("rc-sign", RC, "recenter right sign", "raw_right = -error_y / c.image_center_y",
     "raw_right = error_y / c.image_center_y"),
    ("rc-decay", RC, "recenter loss decay", "loss_decay: float = 0.7", "loss_decay: float = 0.8"),
    ("rc-required", RC, "recenter 10 updates", "centered_required: int = 10", "centered_required: int = 11"),
    ("ch-out-of-range", CH, "index > n returns zeros", "if index <= 0 or index > n:", "if index <= 0 or index >= n:"),
    ("ch-tracking-index", CH, "tracking at index + 6n", "tracking = _at(data, index + n * 6)",
     "tracking = _at(data, index + n * 5)"),
    ("ch-orientation-offset", CH, "orientation block offset",
     "orientation = (_at(data, (index + n) * 3 - 2)", "orientation = (_at(data, (index + n) * 3 - 1)"),
    ("ch-noninteger-index", CH, "non-integer index errors",
     "one_based != int(one_based)", "False"),
    ("ch-loss-time", CH, "silent during first second", "    if time > 1:", "    if time >= 1:"),
    ("ch-loss-error", CH, "error >= 0 is healthy", "state == 2 and error >= 0", "state == 2 and error > 0"),
    ("ch-loss-state", CH, "only state 2 is connected", "state == 2 and error >= 0", "state >= 2 and error >= 0"),
    ("lg-swap-axes", LG, "legacy vx from error_y",
     "cmd = _sat([params[\"kp_image\"] * ey, params[\"kp_image\"] * ex, 0.0]",
     "cmd = _sat([params[\"kp_image\"] * ex, params[\"kp_image\"] * ey, 0.0]"),
    ("lg-sat-order", LG, "max(min(cmd, s), -s) with NaN",
     "return [mmax(mmin(v, limit), -limit) for v in values]",
     "return [mmin(mmax(v, -limit), limit) for v in values]"),
    ("lg-centered-strict", LG, "centered uses < tolerance",
     "_norm((ex, ey)) < params[\"centerTolerancePixels\"]", "_norm((ex, ey)) <= params[\"centerTolerancePixels\"]"),
    ("lg-landed", LG, "landed when z <= landingHeight",
     "drone_pos[2] <= params[\"landingHeight\"]", "drone_pos[2] < params[\"landingHeight\"]"),
    ("lg-descent-sign", LG, "descend vz negative", "-params[\"descentSpeed\"]],", "params[\"descentSpeed\"]],"),
    ("lg-grid-serpentine", LG, "odd rows forward", "row = xs if i % 2 == 1 else", "row = xs if i % 2 == 0 else"),
    ("lg-grid-clamp", LG, "colon clamps the last element",
     "    if (step > 0 and vals[-1] > stop) or (step < 0 and vals[-1] < stop):\n        vals[-1] = stop\n", ""),
    ("lg-config-override", LG, "config.m override (minPadWidth 40)",
     "\"minPadWidth\": 40,", "\"minPadWidth\": 25,"),
]

SUITES = {
    "parity": ["tests/test_parity_octave.py"],
    "all": ["tests/test_parity_octave.py", "tests/test_controllers_unit.py", "tests/test_blocks.py"],
}


def _classify(proc: subprocess.CompletedProcess) -> str:
    out = proc.stdout + proc.stderr
    failed = re.search(r"(\d+) failed", out)
    errors = re.search(r"(\d+) errors?", out)
    if proc.returncode == 0:
        return "survived"
    if failed and int(failed.group(1)) > 0:
        return "killed"
    if errors or proc.returncode in (2, 3, 4):
        return "killed-error"
    return "killed-error"


def run_one(mutant, timeout: float = 120.0) -> dict:
    mid, rel, rule, old, new = mutant
    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / "src"
        shutil.copytree(ROOT / "src", dst)
        target = Path(td) / rel
        text = target.read_text()
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"mutant {mid}: pattern matched {count} times in {rel}")
        target.write_text(text.replace(old, new))
        env = {**os.environ, "PYTHONPATH": str(dst), "PYTHONDONTWRITEBYTECODE": "1"}
        check = subprocess.run([sys.executable, "-c", "import quanser_landing,sys;print(quanser_landing.__file__)"],
                               capture_output=True, text=True, env=env, cwd=ROOT)
        if not check.stdout.strip().startswith(str(dst)):
            raise SystemExit(f"mutant {mid}: import resolved to {check.stdout.strip()}, not the mutated copy")
        res = {"id": mid, "file": rel, "rule": rule}
        for suite, tests in SUITES.items():
            try:
                proc = subprocess.run([sys.executable, "-m", "pytest", "-q", "-x", "-p", "no:cacheprovider", *tests],
                                      capture_output=True, text=True, env=env, cwd=ROOT, timeout=timeout)
                res[suite] = _classify(proc)
            except subprocess.TimeoutExpired:
                res[suite] = "timeout"
        return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--json", type=Path)
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args(argv)
    # Baseline must pass, otherwise every mutant would look killed.
    base = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", *SUITES["all"]],
                          capture_output=True, text=True, cwd=ROOT)
    if base.returncode != 0:
        print(base.stdout[-2000:])
        raise SystemExit("baseline tests fail; fix them before sweeping")
    mutants = [m for m in MUTANTS if not args.only or m[0] in args.only]
    t0 = time.time()
    results = []
    for m in mutants:
        r = run_one(m)
        results.append(r)
        print(f"{r['id']:28s} parity={r['parity']:13s} all={r['all']:13s} {r['rule']}", flush=True)
    summary = {}
    for suite in SUITES:
        summary[suite] = {k: sum(1 for r in results if r[suite] == k)
                          for k in ("killed", "killed-error", "survived", "timeout")}
    print(json.dumps({"mutants": len(results), "summary": summary, "seconds": round(time.time() - t0, 1)}))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"mutants": results, "summary": summary}, indent=1))
    return 0 if all(r["all"] != "survived" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
