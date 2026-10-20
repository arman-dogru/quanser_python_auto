"""Regenerate the README figures from real runs.

* parity.png: Python vs Octave max abs error per block over every committed
  fixture (plus ``outputs/scenario_parity`` when present: all eight SIL
  scenarios, regenerated with ``make_fixtures.py --scenarios``).
* nominal_mission.png: the nominal Phase 10 closed loop (top view + timeline).
* safety_scenarios.png: height and trigger bits for every fault scenario.
* mutation.png: both mutation sweeps (needs docs/figures/data/mutation_*.json).

Usage: ``python tools/make_figures.py`` (writes docs/figures/*.png and data/*.json).
"""
from __future__ import annotations

import gzip
import io
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from qdrone_autoland import parity, sim  # noqa: E402
from qdrone_autoland.config import DEFAULT  # noqa: E402

FIG = ROOT / "docs" / "figures"
DATA = FIG / "data"
plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "figure.dpi": 110, "savefig.dpi": 110})


def _load(p: Path) -> np.ndarray:
    if p.suffix == ".gz":
        return np.loadtxt(io.BytesIO(gzip.decompress(p.read_bytes())), delimiter=",", ndmin=2)
    return parity.load_csv(p)


def parity_figure() -> dict:
    roots = [("committed", ROOT / "tests" / "fixtures" / "parity"), ("scenarios", ROOT / "outputs" / "scenario_parity")]
    stats: dict = {}
    for label, root in roots:
        if not root.exists():
            continue
        for inp in sorted([*root.glob("*/*.in.csv"), *root.glob("*/*.in.csv.gz")]):
            block = inp.parent.name
            out = inp.with_name(inp.name.replace(".in.", ".out."))
            x, e = _load(inp), _load(out)
            err = parity.compare(e, parity.run_sequence(block, x))
            s = stats.setdefault(block, {"ticks": 0, "sequences": 0, "max_abs_error": 0.0, "scenario_ticks": 0})
            s["ticks"] += len(x)
            s["sequences"] += 1
            if label == "scenarios":
                s["scenario_ticks"] += len(x)
            s["max_abs_error"] = max(s["max_abs_error"], err)
    blocks = list(stats)
    fig, ax = plt.subplots(1, 2, figsize=(8, 2.6))
    ticks = [stats[b]["ticks"] for b in blocks]
    ax[0].bar(blocks, ticks, color="#4c72b0")
    ax[0].set_yscale("log")
    ax[0].set_title("Ticks compared against Octave (log)")
    for i, t in enumerate(ticks):
        ax[0].text(i, t, f"{t:,}", ha="center", va="bottom", fontsize=6)
    errs = [max(stats[b]["max_abs_error"], 1e-20) for b in blocks]
    ax[1].bar(blocks, errs, color=["#55a868" if e <= 1e-12 else "#c44e52" for e in errs])
    ax[1].axhline(1e-12, color="k", ls="--", lw=0.8)
    ax[1].text(len(blocks) - 0.5, 1.5e-12, "tolerance 1e-12", ha="right", fontsize=6)
    ax[1].set_yscale("log")
    ax[1].set_ylim(1e-21, 1e-9)
    ax[1].set_title("Max |Python - Octave| (1e-20 = exact)")
    for a in ax:
        a.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(FIG / "parity.png")
    plt.close(fig)
    return stats


def nominal_figure(r) -> None:
    s, t = r.signals, r.t
    wps = np.array([[float(f"{v:.6g}") for v in w] for w in DEFAULT.auto.waypoints])
    fig = plt.figure(figsize=(8, 3.2))
    a0 = fig.add_axes([0.06, 0.13, 0.30, 0.78])
    a0.plot(wps[:, 0], wps[:, 1], "o--", color="#bbbbbb", ms=3, lw=0.8, label="grid waypoints")
    a0.plot(s["x"], s["y"], color="#4c72b0", lw=1.2, label="vehicle (stand-in)")
    a0.plot(*r.scenario.pad_xy, marker="s", color="#c44e52", ms=8, ls="", label="pad")
    a0.plot(r.vehicle.pos[0], r.vehicle.pos[1], "kx", ms=7, label="touchdown")
    a0.set_aspect("equal")
    a0.set_xlim(-1.4, 1.4)
    a0.set_ylim(-1.2, 1.2)
    a0.set_title("Top view (m)")
    a0.legend(fontsize=6, loc="upper left")
    a1 = fig.add_axes([0.45, 0.42, 0.53, 0.49])
    a1.plot(t, s["z"], color="#4c72b0", label="height z")
    a1.plot(t, s["dz"], color="#dd8452", lw=0.8, ls="--", label="desired z")
    a1.set_ylabel("m")
    a1.set_ylim(-0.05, 0.8)
    ax2 = a1.twinx()
    ax2.step(t, s["stage"], color="#55a868", lw=0.8, where="post", label="descent stage")
    ax2.set_ylim(-0.3, 5.6)
    ax2.set_ylabel("stage")
    a1.set_title("Nominal Phase 10 mission (SIL, full runtime incl. safety layer)")
    h1, l1 = a1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    a1.legend(h1 + h2, l1 + l2, fontsize=6, loc="lower left")
    a1.tick_params(labelbottom=False)
    a2 = fig.add_axes([0.45, 0.13, 0.53, 0.25], sharex=a1)
    for i, (k, lab) in enumerate([("confirmed", "pad confirmed"), ("ready", "landing ready"),
                                  ("latched", "LAND latched"), ("trig2", "trigger 2 sent")]):
        a2.fill_between(t, i + 0.1, i + 0.1 + 0.8 * s[k], step="post", alpha=0.7)
        a2.text(t[0], i + 0.45, lab, fontsize=6, va="center")
    a2.set_yticks([])
    a2.set_xlabel("time (s)")
    fig.savefig(FIG / "nominal_mission.png")
    plt.close(fig)


def safety_figure(results) -> dict:
    names = [n for n in sim.SCENARIOS if n != "nominal-autoland"]
    fig, axes = plt.subplots(2, 4, figsize=(8, 3.6), sharey=True)
    summary = {}
    for ax, name in zip(axes.flat, names, strict=False):
        r = results[name]
        s, t = r.signals, r.t
        ax.plot(t, s["z"], color="#4c72b0", lw=1)
        ax.plot(t, 0.9 + 0.08 * s["trig2"], color="#dd8452", lw=0.8)
        ax.plot(t, 1.05 + 0.08 * s["trig3"], color="#c44e52", lw=0.8)
        ax.plot(t, 1.2 + 0.08 * s["trig4"], color="#8172b2", lw=0.8)
        for ev, c in (("vision_lost", "#937860"), ("tracking_lost", "#937860"), ("garbage_injected", "#937860")):
            if ev in r.events:
                ax.axvline(r.events[ev], color=c, ls=":", lw=0.8)
        ax.set_title(name, fontsize=7)
        bits = int(s["abort_bits"][-1])
        ax.text(0.02, 0.02, f"abort bits {bits}\nlanded {'yes' if 'landed' in r.events else 'no'}",
                transform=ax.transAxes, fontsize=6)
        summary[name] = {"events": r.events, "abort_bits": bits, "gate_faults": int(s["faults"].sum()),
                         "vision_refused": dict(r.runtime.vision_guard.refused),
                         "pose_refused": dict(r.runtime.pose_guard.refused)}
    for ax in list(axes.flat)[len(names):]:
        ax.set_visible(False)
    axes[0, 0].set_ylabel("z (m) / bits")
    axes[1, 0].set_ylabel("z (m) / bits")
    axes[0, 0].set_yticks([0, 0.5, 0.94, 1.09, 1.24], ["0", "0.5", "trig2", "E-stop", "joy issue"])
    for ax in axes[1][: len(names) - 4]:
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(FIG / "safety_scenarios.png")
    plt.close(fig)
    return summary


def mutation_figure() -> dict | None:
    rows = []
    out = {}
    for sweep in ("logic", "safety"):
        p = DATA / f"mutation_{sweep}.json"
        if not p.exists():
            continue
        d = json.loads(p.read_text())
        out[sweep] = d["summary"]
        for suite, counts in d["summary"].items():
            rows.append((f"{sweep}\n({suite})", counts))
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(6, 2.2))
    keys = [("killed", "#55a868"), ("killed-error", "#8172b2"), ("timeout", "#ccb974"), ("survived", "#c44e52")]
    left = np.zeros(len(rows))
    for k, c in keys:
        vals = np.array([r[1][k] for r in rows])
        ax.barh([r[0] for r in rows], vals, left=left, color=c, label=k)
        for i, v in enumerate(vals):
            if v:
                ax.text(left[i] + v / 2, i, str(v), ha="center", va="center", fontsize=7, color="white")
        left += vals
    ax.invert_yaxis()
    ax.legend(fontsize=6, ncol=4, loc="lower right", bbox_to_anchor=(1, 1.0))
    ax.set_xlabel("mutants")
    fig.tight_layout()
    fig.savefig(FIG / "mutation.png")
    plt.close(fig)
    return out


def main() -> int:
    FIG.mkdir(parents=True, exist_ok=True)
    DATA.mkdir(parents=True, exist_ok=True)
    results = {n: sim.run(n) for n in sim.SCENARIOS}
    stats = parity_figure()
    nominal_figure(results["nominal-autoland"])
    safety = safety_figure(results)
    mut = mutation_figure()
    nom = results["nominal-autoland"]
    (DATA / "parity.json").write_text(json.dumps(stats, indent=1) + "\n")
    (DATA / "scenarios.json").write_text(json.dumps({
        "nominal-autoland": {"events": nom.events, "touchdown": [round(float(v), 4) for v in nom.vehicle.pos[:2]],
                             "pad": list(nom.scenario.pad_xy), "abort_bits": int(nom.signals["abort_bits"][-1])},
        **safety}, indent=1, default=float) + "\n")
    print(json.dumps({"parity": stats, "mutation": mut}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
