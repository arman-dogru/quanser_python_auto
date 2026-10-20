"""Run the nominal Phase 10 mission with the pad at every point of a grid.

Each run is ``qdrone_autoland.sim.run_pad``: the full runtime (guards, model
logic, output gate) in closed loop against the stand-in vehicle and camera,
120 s per pad, stopping at touchdown. ``tests/test_sim.py`` runs a subset of
the same grid; this script runs all of it.

Usage::

    python tools/pad_sweep.py                                   # 12 x 8, prints a summary
    python tools/pad_sweep.py --json docs/figures/data/pad_sweep.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from qdrone_autoland import sim  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--nx", type=int, default=sim.PAD_GRID_X[2])
    ap.add_argument("--ny", type=int, default=sim.PAD_GRID_Y[2])
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)
    pads = sim.pad_grid(args.nx, args.ny)
    t0 = time.time()
    with ProcessPoolExecutor(args.jobs) as ex:
        runs = list(ex.map(sim.run_pad, pads, [args.duration] * len(pads)))
    summary = {"pads": len(runs), "landed": sum(r["landed"] for r in runs),
               "runs_with_gate_faults": sum(bool(r["gate_faults"]) for r in runs),
               "never_confirmed": [r["pad"] for r in runs if not r["confirmed"]],
               "not_landed": [r["pad"] for r in runs if not r["landed"]],
               "max_touchdown_error_m": max((r["touchdown_error_m"] for r in runs if r["landed"]), default=None),
               "grid": {"x": list(sim.PAD_GRID_X[:2]) + [args.nx], "y": list(sim.PAD_GRID_Y[:2]) + [args.ny]},
               "duration_s": args.duration}
    print(json.dumps(summary, indent=1))
    print(f"{time.time() - t0:.0f} s", file=sys.stderr)
    if args.json:
        args.json.write_text(json.dumps({"summary": summary, "runs": runs}, indent=1) + "\n")
    return 0 if summary["runs_with_gate_faults"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
