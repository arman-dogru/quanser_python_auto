"""Plots for the README, from real runs (outputs are gitignored).

* parity_sequence.png: Python vs Octave on Octave fixture sequences
  (one seeded random sequence, the loss-decay case and the 10-update case),
  with the absolute difference.
* parity_error.png: max |Python - Octave| per output over every fixture step.
* scenario_replay.png: the synthetic closed-loop replay, showing the hold of
  the last UDP packet after the sender stops.
* mutation_sweep.png: only if outputs/mutation_sweep.json exists.

Usage::

    python tools/make_plots.py [--out docs/plots]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from quanser_landing.controllers import PoseCommandConfig, YOLOPoseCommandGenerator  # noqa: E402
from quanser_landing.replay import load_detections, load_poses, replay  # noqa: E402
from quanser_landing.scenario import generate  # noqa: E402

OUTS = ["cmd_x", "cmd_y", "cmd_z", "cmd_yaw", "forward_cmd", "right_cmd", "state"]


def run_case(case, enable_descent=False):
    g = YOLOPoseCommandGenerator(PoseCommandConfig(enable_descent=enable_descent))
    rows = []
    for inp in case["inputs"]:
        o = g.step(inp[:4], *inp[4:])
        rows.append([*o.cmd_pose, o.forward_cmd, o.right_cmd, float(o.state)])
    return rows


def diff(a, b):
    if math.isnan(a) and math.isnan(b):
        return 0.0
    if a == b:
        return 0.0
    return abs(a - b)


def plot_parity(fx, out: Path):
    cases = {c["name"]: c for c in fx["cases"]}
    picks = ["random seed=1 seq=0", "loss decay after drive", "ten updates broken by offcentre at 10"]
    fig, axes = plt.subplots(3, len(picks), figsize=(15, 8.5), sharex="col")
    for j, name in enumerate(picks):
        c = cases[name]
        py = run_case(c)
        exp = c["expected"]
        k = range(len(py))
        ax = axes[0][j]
        ax.plot(k, [e[4] for e in exp], lw=3, alpha=0.35, label="Octave forward_cmd")
        ax.plot(k, [p[4] for p in py], "--", lw=1.2, label="Python forward_cmd")
        ax.plot(k, [e[5] for e in exp], lw=3, alpha=0.35, label="Octave right_cmd")
        ax.plot(k, [p[5] for p in py], ":", lw=1.2, label="Python right_cmd")
        ax.set_title(name)
        ax.legend(fontsize=7)
        ax = axes[1][j]
        ax.step(k, [e[6] for e in exp], where="post", lw=3, alpha=0.35, label="Octave state")
        ax.step(k, [p[6] for p in py], "--", where="post", lw=1.2, label="Python state")
        ax.plot(k, [i[4] for i in c["inputs"]], "k.", ms=3, label="detected input")
        ax.set_yticks([0, 1, 2, 3])
        ax.legend(fontsize=7)
        ax = axes[2][j]
        d = [max(diff(a, b) for a, b in zip(p, e, strict=True)) for p, e in zip(py, exp, strict=True)]
        ax.plot(k, d, "r-")
        ax.set_ylim(-1e-16, 1e-15)
        ax.set_ylabel("max |Python - Octave|")
        ax.set_xlabel("controller update (33 ms each)")
    fig.suptitle("YOLOPoseCommandGenerator: Python port vs Octave running the extracted Stage 6 chart")
    fig.tight_layout()
    fig.savefig(out / "parity_sequence.png", dpi=110)
    plt.close(fig)


def plot_parity_error(fixtures: dict, out: Path):
    labels, values, steps = [], [], 0
    for fname, fx in fixtures.items():
        worst = [0.0] * len(OUTS)
        for c in fx["cases"]:
            for p, e in zip(run_case(c, fname.endswith("descent_enabled")), c["expected"], strict=True):
                steps += 1
                for i, (a, b) in enumerate(zip(p, e, strict=True)):
                    worst[i] = max(worst[i], diff(a, b))
        for o, w in zip(OUTS, worst, strict=True):
            labels.append(f"{fname}\n{o}")
            values.append(w)
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.bar(range(len(values)), values, color="tab:green")
    ax.set_xticks(range(len(values)), labels, rotation=60, fontsize=7)
    ax.set_ylim(0, 1e-15)
    ax.set_ylabel("max |Python - Octave|")
    ax.set_title(f"Max absolute difference over {steps} fixture steps (all bars are exactly 0)")
    fig.tight_layout()
    fig.savefig(out / "parity_error.png", dpi=110)
    plt.close(fig)
    return steps, max(values)


def plot_scenario(out: Path):
    with tempfile.TemporaryDirectory() as td:
        generate(Path(td), seed=0)
        poses = load_poses(Path(td) / "poses_seed0.csv")
        dets = load_detections(Path(td) / "detections_seed0.jsonl")
    rows = replay(poses, dets)
    t = [r["t"] for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, [r["x"] for r in rows], label="measured x")
    axes[0].plot(t, [r["cmd_x"] for r in rows], "--", label="cmd x")
    axes[0].plot(t, [r["y"] for r in rows], label="measured y")
    axes[0].plot(t, [r["cmd_y"] for r in rows], "--", label="cmd y")
    axes[0].set_ylabel("m")
    axes[0].legend(fontsize=8, ncol=4)
    axes[1].plot(t, [r["error_x"] for r in rows], label="error_x held")
    axes[1].plot(t, [r["error_y"] for r in rows], label="error_y held")
    axes[1].axhspan(-40, 40, color="grey", alpha=0.15, label="deadband ±40 px")
    axes[1].set_ylabel("px")
    axes[1].legend(fontsize=8)
    axes[2].step(t, [r["state"] for r in rows], where="post")
    axes[2].set_yticks([0, 1, 2, 3])
    axes[2].set_ylabel("state")
    axes[3].plot(t, [r["detection_age_s"] for r in rows], color="tab:red")
    axes[3].set_ylabel("age of held packet (s)")
    axes[3].set_xlabel("time (s)")
    fig.suptitle("SYNTHETIC scenario (seed 0), 33 ms replay: after the sender stops at 15 s the "
                 "model keeps using the last packet")
    fig.tight_layout()
    fig.savefig(out / "scenario_replay.png", dpi=110)
    plt.close(fig)


def plot_mutation(out: Path):
    p = ROOT / "outputs" / "mutation_sweep.json"
    if not p.exists():
        return None
    data = json.loads(p.read_text())
    res = data["mutants"]
    fig, ax = plt.subplots(figsize=(12, 4))
    colors = {"killed": "tab:green", "killed-error": "tab:orange", "survived": "tab:red", "timeout": "grey"}
    ax.bar(range(len(res)), [1] * len(res), color=[colors[r["parity"]] for r in res])
    ax.set_xticks(range(len(res)), [r["id"] for r in res], rotation=80, fontsize=7)
    ax.set_yticks([])
    s = data["summary"]["parity"]
    ax.set_title(f"Mutation sweep, Octave parity tests only: {s['killed']} killed by a failed assertion, "
                 f"{s['killed-error']} by an error, {s['survived']} survived, {s['timeout']} timed out")
    fig.tight_layout()
    fig.savefig(out / "mutation_sweep.png", dpi=110)
    plt.close(fig)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "plots")
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    fx = {n: json.loads((ROOT / "tests" / "fixtures" / f"{n}.json").read_text())
          for n in ("pose_command", "pose_command_descent_enabled")}
    plot_parity(fx["pose_command"], args.out)
    print("parity_error:", plot_parity_error(fx, args.out))
    plot_scenario(args.out)
    print("mutation:", plot_mutation(args.out))
    print("wrote", sorted(p.name for p in args.out.glob("*.png")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
