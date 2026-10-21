"""Measure the real-time loop: ``qautoland serve --mode dry-run`` for 5 s.

Each run is the CLI's own statistics (CPU time, wall time, tick-lag
percentiles), exactly as printed by ``qautoland serve``. ``--runs N`` times N
local runs; ``--ci RUN_ID ...`` reads the same statistics from the Windows CI
job's log (``gh run view --log``, needs the GitHub CLI), so the figure shows
both hosts from recorded output.

The local runs were made in the development container (Linux x86_64,
Python 3.12.15), not on the lab PC; ``--host-label`` records that.
``--keep-local`` keeps the local runs already in ``--json`` and refreshes
only the CI entries (each tagged with the commit it ran on).

Usage::

    python tools/serve_timing.py --runs 5 --host-label "dev container" \
        --ci 39493996791 39500677539 39510188039 --json docs/figures/data/serve_timing.json
    python tools/serve_timing.py --keep-local --ci ... --json docs/figures/data/serve_timing.json
"""
from __future__ import annotations

import argparse
import json
import platform
import re
import subprocess
import sys
from pathlib import Path

KEYS = ("ticks", "cpu_s", "wall_s", "lag_p50_s", "lag_p99_s", "lag_p999_s", "max_lag_s", "stopped_reason")


def local_run(duration: float) -> dict:
    out = subprocess.run([sys.executable, "-m", "qdrone_autoland.cli", "serve", "--mode", "dry-run", "--duration",
                          str(duration)], check=True, capture_output=True, text=True).stdout
    st = json.loads(out[out.index("{"):])
    return {k: st[k] for k in KEYS}


def ci_run(run_id: str, repo: str) -> dict:
    log = subprocess.run(["gh", "run", "view", run_id, "-R", repo, "--log"], check=True, capture_output=True,
                         text=True).stdout
    st = {}
    for line in log.splitlines():
        if "windows" not in line.lower():
            continue
        m = re.search(r'"(\w+)": (null|[-0-9.e]+|"[^"]*"),?\s*$', line)
        if m and m.group(1) in KEYS and m.group(1) not in st:
            st[m.group(1)] = json.loads(m.group(2))
    missing = [k for k in KEYS if k not in st]
    if missing:
        raise SystemExit(f"run {run_id}: no {missing} in the Windows job log")
    sha = subprocess.run(["gh", "run", "view", run_id, "-R", repo, "--json", "headSha", "-q", ".headSha"],
                         check=True, capture_output=True, text=True).stdout.strip()
    return {"run_id": run_id, "commit": sha, **st}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--ci", nargs="*", default=[])
    ap.add_argument("--repo", default="arman-dogru/quanser_python_auto")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--host-label", default="local", help="what the local machine is, e.g. 'dev container'")
    ap.add_argument("--keep-local", action="store_true", help="reuse the local runs already in --json")
    args = ap.parse_args(argv)
    if args.keep_local:
        local = json.loads(args.json.read_text())["local"]
        local.setdefault("label", args.host_label)
    else:
        local = {"label": args.host_label,
                 "host": f"{platform.system()} {platform.machine()}, Python {platform.python_version()}",
                 "runs": [local_run(args.duration) for _ in range(args.runs)]}
    data = {"duration_s": args.duration, "local": local, "windows_ci": [ci_run(r, args.repo) for r in args.ci]}
    print(json.dumps(data, indent=1))
    if args.json:
        args.json.write_text(json.dumps(data, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
