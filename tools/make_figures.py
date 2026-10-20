"""Regenerate the README figures from real runs.

* parity.png: ticks compared against Octave per block, labelled with the
  max |Python - Octave|, over every committed fixture plus the eight SIL
  scenarios. The scenario fixtures need Octave
  (``python tools/make_fixtures.py --scenarios outputs/scenario_parity``);
  without them the scenario numbers recorded in ``data/parity.json`` are
  reused, so a fresh clone rebuilds every figure without Octave.
* nominal_mission.png: the nominal Phase 10 closed loop (top view + timeline).
* safety_scenarios.png: height and trigger bits for every fault scenario.
* mutation.png: both mutation sweeps (from the committed
  docs/figures/data/mutation_*.json written by tools/mutation_sweep.py).

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
plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "legend.fontsize": 8, "figure.dpi": 110,
                     "savefig.dpi": 110})


def _load(p: Path) -> np.ndarray:
    if p.suffix == ".gz":
        return np.loadtxt(io.BytesIO(gzip.decompress(p.read_bytes())), delimiter=",", ndmin=2)
    return parity.load_csv(p)


def _parity_stats(root: Path) -> dict:
    stats: dict = {}
    for inp in sorted([*root.glob("*/*.in.csv"), *root.glob("*/*.in.csv.gz")]):
        block = inp.parent.name
        x, e = _load(inp), _load(inp.with_name(inp.name.replace(".in.", ".out.")))
        err = parity.compare(e, parity.run_sequence(block, x))
        st = stats.setdefault(block, {"ticks": 0, "sequences": 0, "max_abs_error": 0.0})
        st["ticks"] += len(x)
        st["sequences"] += 1
        st["max_abs_error"] = max(st["max_abs_error"], err)
    return stats


def parity_figure() -> dict:
    committed = _parity_stats(ROOT / "tests" / "fixtures" / "parity")
    scen_dir = ROOT / "outputs" / "scenario_parity"
    if scen_dir.exists():
        scenarios = _parity_stats(scen_dir)
    else:
        prev = json.loads((DATA / "parity.json").read_text()) if (DATA / "parity.json").exists() else {}
        scenarios = prev.get("scenarios", {})
        print(f"scenario parity: reused from data/parity.json (no {scen_dir.relative_to(ROOT)})")
    blocks = list(dict.fromkeys([*committed, *scenarios]))
    fig, ax = plt.subplots(figsize=(8.5, 3.4))
    xs = np.arange(len(blocks))
    c_ticks = np.array([committed.get(b, {}).get("ticks", 0) for b in blocks])
    s_ticks = np.array([scenarios.get(b, {}).get("ticks", 0) for b in blocks])
    ax.bar(xs - 0.2, c_ticks, 0.4, color="#4c72b0", label="committed fixtures")
    ax.bar(xs + 0.2, s_ticks, 0.4, color="#55a868", label="8 closed-loop scenarios")
    ax.set_yscale("log")
    ax.set_ylim(10, 2e7)
    for i, b in enumerate(blocks):
        err = max(committed.get(b, {}).get("max_abs_error", 0.0), scenarios.get(b, {}).get("max_abs_error", 0.0))
        label = "max |Δ| = 0" if err == 0 else f"max |Δ| = {err:.1e}"
        ax.text(i, max(c_ticks[i], s_ticks[i]) * 1.4, label, ha="center", va="bottom", fontsize=8)
    ax.set_xticks(xs, blocks)
    ax.set_ylabel("ticks compared (log)")
    total = int(c_ticks.sum() + s_ticks.sum())
    ax.set_title(f"Python vs Octave running the model's chart code: {total:,} ticks, tolerance 1e-12")
    ax.legend(loc="upper left", ncol=2)
    fig.tight_layout()
    fig.savefig(FIG / "parity.png")
    plt.close(fig)
    return {"committed": committed, "scenarios": scenarios}


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
    a0.legend(fontsize=7, loc="upper left")
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
    a1.legend(h1 + h2, l1 + l2, fontsize=7, loc="lower left")
    a1.tick_params(labelbottom=False)
    a2 = fig.add_axes([0.45, 0.13, 0.53, 0.25], sharex=a1)
    for i, (k, lab) in enumerate([("confirmed", "pad confirmed"), ("ready", "landing ready"),
                                  ("latched", "LAND latched"), ("trig2", "trigger 2 sent")]):
        a2.fill_between(t, i + 0.1, i + 0.1 + 0.8 * s[k], step="post", alpha=0.7)
        a2.text(t[0], i + 0.45, lab, fontsize=7, va="center")
    a2.set_yticks([])
    a2.set_xlabel("time (s)")
    fig.savefig(FIG / "nominal_mission.png")
    plt.close(fig)


def safety_figure(results) -> dict:
    names = [n for n in sim.SCENARIOS if n != "nominal-autoland"]
    fig = plt.figure(figsize=(11, 6.2))
    outer = fig.add_gridspec(2, 4, hspace=0.55, wspace=0.22, left=0.1, right=0.99, top=0.88, bottom=0.08)
    lanes = [("trig2", "Takeoff/Land", "#dd8452"), ("trig3", "E-stop", "#c44e52"), ("trig4", "joy issue", "#8172b2")]
    summary = {}
    for idx, name in enumerate(names):
        r = results[name]
        s, t = r.signals, r.t
        cell = outer[idx // 4, idx % 4].subgridspec(2, 1, height_ratios=[2, 1], hspace=0.08)
        az = fig.add_subplot(cell[0])
        ab = fig.add_subplot(cell[1], sharex=az)
        az.plot(t, s["z"], color="#4c72b0", lw=1.2)
        az.set_ylim(-0.05, 0.85)
        for ev in ("vision_lost", "tracking_lost", "garbage_injected"):
            if ev in r.events:
                for a in (az, ab):
                    a.axvline(r.events[ev], color="#937860", ls=":", lw=1)
        for k, (key, _, c) in enumerate(lanes):
            ab.fill_between(t, k + 0.1, k + 0.1 + 0.8 * s[key], step="post", color=c, alpha=0.85)
        ab.set_ylim(0, 3)
        ab.set_yticks([0.5, 1.5, 2.5], [lab for _, lab, _ in lanes] if idx % 4 == 0 else [])
        az.tick_params(labelbottom=False)
        if idx % 4 == 0:
            az.set_ylabel("height z (m)")
        else:
            az.tick_params(labelleft=False)
        if idx >= 4 or idx + 4 >= len(names):
            ab.set_xlabel("time (s)")
        bits = int(s["abort_bits"][-1])
        az.set_title(f"{name}\nabort bits {bits}, landed {'yes' if 'landed' in r.events else 'no'}", fontsize=9)
        summary[name] = {"events": r.events, "abort_bits": bits, "gate_faults": int(s["faults"].sum()),
                         "vision_refused": dict(r.runtime.vision_guard.refused),
                         "pose_refused": dict(r.runtime.pose_guard.refused)}
    fig.suptitle("Fault scenarios: vehicle height (top) and the trigger bits sent to the Commander (bottom); "
                 "dotted line = injected fault", fontsize=10)
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
    fig, ax = plt.subplots(figsize=(7, 2.6))
    keys = [("killed", "#55a868"), ("killed-error", "#8172b2"), ("timeout", "#ccb974"), ("survived", "#c44e52")]
    left = np.zeros(len(rows))
    for k, c in keys:
        vals = np.array([r[1][k] for r in rows])
        ax.barh([r[0] for r in rows], vals, left=left, color=c, label=k)
        for i, v in enumerate(vals):
            if v:
                ax.text(left[i] + v / 2, i, str(v), ha="center", va="center", fontsize=9, color="white")
        left += vals
    ax.invert_yaxis()
    ax.legend(fontsize=8, ncol=4, loc="lower right", bbox_to_anchor=(1, 1.0))
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
