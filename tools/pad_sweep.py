"""Run the nominal Phase 10 mission with the pad at every point of a grid.

Each run is ``qdrone_autoland.sim.run_pad``: the full runtime (guards, model
logic, output gate) in closed loop against the stand-in vehicle and camera,
120 s per pad, stopping at touchdown. ``tests/test_sim.py`` runs a subset of
the same grid; this script runs all of it.

Usage::

    python tools/pad_sweep.py                                   # 12 x 8, prints a summary
    python tools/pad_sweep.py --json docs/figures/data/pad_sweep.json
    python tools/pad_sweep.py --rev eec0a74 --json docs/figures/data/pad_sweep_eec0a74.json

``--rev`` runs the same grid against an earlier commit, checked out into a
temporary git worktree (it needs the full history: on a shallow clone, such as
the default ``actions/checkout``, it stops with a message) (commits before ``sim.run_pad`` existed are driven
through ``sim.run`` with the same scenario change). The figures use it to
compare the final output gate with an earlier one.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from qdrone_autoland import sim  # noqa: E402

# Runs inside the worktree of --rev: the same summary as sim.run_pad.
_REV_DRIVER = r"""
import dataclasses, json, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from qdrone_autoland import sim

def one(arg):
    pad, duration = arg
    sc = dataclasses.replace(sim.SCENARIOS["nominal-autoland"], name=f"pad({pad[0]:g},{pad[1]:g})",
                             pad_xy=tuple(pad), duration=duration)
    r = sim.run(sc)
    s, gate = r.signals, r.runtime.gate
    return {"pad": list(pad), "landed": "landed" in r.events, "landed_s": r.events.get("landed"),
            "confirmed": bool(np.nanmax(s["confirmed"]) > 0), "max_stage": int(np.nanmax(s["stage"])),
            "gate_faults": dict(gate.faults),
            "touchdown_error_m": round(float(np.hypot(*(r.vehicle.pos[:2] - np.array(pad)))), 4)}

if __name__ == "__main__":
    pads, duration, jobs = json.loads(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
    with ProcessPoolExecutor(jobs) as ex:
        print(json.dumps(list(ex.map(one, [(p, duration) for p in pads]))))
"""


def run_at_rev(rev: str, pads, duration: float, jobs: int) -> tuple[str, list[dict]]:
    shallow = subprocess.run(["git", "rev-parse", "--is-shallow-repository"], cwd=ROOT, capture_output=True,
                             text=True).stdout.strip()
    if shallow == "true":
        raise SystemExit("--rev needs the full git history; this clone is shallow (git fetch --unshallow)")
    if subprocess.run(["git", "cat-file", "-e", f"{rev}^{{commit}}"], cwd=ROOT, capture_output=True).returncode:
        raise SystemExit(f"--rev {rev}: no such commit in this clone")
    sha = subprocess.run(["git", "rev-parse", rev], cwd=ROOT, check=True, capture_output=True,
                         text=True).stdout.strip()
    with tempfile.TemporaryDirectory() as tmp:
        wt = Path(tmp) / "wt"
        subprocess.run(["git", "worktree", "add", "--detach", str(wt), sha], cwd=ROOT, check=True,
                       capture_output=True)
        try:
            (Path(tmp) / "driver.py").write_text(_REV_DRIVER)
            env = {**os.environ, "PYTHONPATH": str(wt / "src")}
            out = subprocess.run([sys.executable, str(Path(tmp) / "driver.py"), json.dumps(pads), str(duration),
                                  str(jobs)], cwd=tmp, env=env, check=True, capture_output=True, text=True).stdout
        finally:
            subprocess.run(["git", "worktree", "remove", "--force", str(wt)], cwd=ROOT, check=True)
    return sha, json.loads(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--nx", type=int, default=sim.PAD_GRID_X[2])
    ap.add_argument("--ny", type=int, default=sim.PAD_GRID_Y[2])
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--json", type=Path)
    ap.add_argument("--rev", help="run the grid against this earlier commit (temporary git worktree)")
    args = ap.parse_args(argv)
    pads = sim.pad_grid(args.nx, args.ny)
    t0 = time.time()
    rev_sha = None
    if args.rev:
        rev_sha, runs = run_at_rev(args.rev, pads, args.duration, args.jobs)
    else:
        with ProcessPoolExecutor(args.jobs) as ex:
            runs = list(ex.map(sim.run_pad, pads, [args.duration] * len(pads)))
    summary = {"pads": len(runs), "landed": sum(r["landed"] for r in runs),
               "runs_with_gate_faults": sum(bool(r["gate_faults"]) for r in runs),
               "never_confirmed": [r["pad"] for r in runs if not r["confirmed"]],
               "not_landed": [r["pad"] for r in runs if not r["landed"]],
               "max_touchdown_error_m": max((r["touchdown_error_m"] for r in runs if r["landed"]), default=None),
               "grid": {"x": list(sim.PAD_GRID_X[:2]) + [args.nx], "y": list(sim.PAD_GRID_Y[:2]) + [args.ny]},
               "duration_s": args.duration}
    if rev_sha:
        summary["commit"] = rev_sha
    print(json.dumps(summary, indent=1))
    print(f"{time.time() - t0:.0f} s", file=sys.stderr)
    if args.json:
        args.json.write_text(json.dumps({"summary": summary, "runs": runs}, indent=1) + "\n")
    return 0 if summary["runs_with_gate_faults"] == 0 or args.rev else 1


if __name__ == "__main__":
    sys.exit(main())
