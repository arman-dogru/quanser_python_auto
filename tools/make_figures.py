"""Regenerate the README figures from real runs.

* parity.png: ticks compared against Octave per block, labelled with the
  max |Python - Octave|, over every committed fixture plus the eight SIL
  scenarios. The scenario fixtures need Octave
  (``python tools/make_fixtures.py --scenarios outputs/scenario_parity``);
  without them the scenario numbers recorded in ``data/parity.json`` are
  reused, so a fresh clone rebuilds every figure without Octave.
* landing_xy.png, landing_timeline.png: the nominal Phase 10 closed loop, top
  view (path, detections, camera footprint, touchdown) and a timeline of
  height, stage, latches and the bits sent.
* edge_pad.png: a pad the model never confirms, with the YOLO confirmation
  counter replayed from the block's recorded inputs.
* pad_sweep.png: data/pad_sweep.json (tools/pad_sweep.py) next to the same grid
  at an earlier output gate (data/pad_sweep_eec0a74.json, ``--rev eec0a74``).
* gate_faults.png: an injected centring fault before and during LAND.
* serve_timing.png: data/serve_timing.json (tools/serve_timing.py).
* safety_scenarios.png: height and trigger bits for every fault scenario.
* mutation.png: both mutation sweeps (from the committed
  docs/figures/data/mutation_*.json written by tools/mutation_sweep.py).

Measured numbers behind the new figures go to data/visuals.json.

Usage: ``python tools/make_figures.py`` (writes docs/figures/*.png and data/*.json).
"""
from __future__ import annotations

import dataclasses
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
from qdrone_autoland import parity, safety, sim  # noqa: E402
from qdrone_autoland.config import DEFAULT  # noqa: E402
from qdrone_autoland.logic.yolo_confirmation import YoloConfirmation  # noqa: E402

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


# ------------------------------------------------------------------ closed-loop visuals
A = DEFAULT.auto
EDGE_PAD = (-1.1, -0.407)          # never confirmed in the sweep (PYTHON_PORT.md)
COMPARE_REV = "eec0a74"           # earlier output gate, swept by tools/pad_sweep.py --rev


def _frames(r) -> np.ndarray:
    """Replay the recorded inputs of the YOLO confirmation block through a fresh
    copy of the ported block: one row per camera frame it consumed, with
    (t, vehicle x, vehicle y, valid detection, consecutive valid frames)."""
    blk = YoloConfirmation(A.yolo_confidence, A.yolo_confirm_frames, A.yolo_release_frames, A.yolo_stale_seconds,
                           DEFAULT.model.image_width, DEFAULT.model.image_height)
    rows = []
    for x in r.block_io["confirm"]:
        before = blk.last_frame
        o = blk.step(x[0:4], x[4:10], x[10] != 0, x[11], x[12:16])
        if blk.last_frame != before:
            rows.append((x[11], x[0], x[1], float(o.new_pad_sample), blk.good))
    return np.array(rows, dtype=float).reshape(-1, 5)


def _footprint(pos, yaw) -> np.ndarray:
    """Ground area seen by the camera: the inverse of sim.project_pad's pinhole."""
    cmap = A.map * (pos[2] / A.search_height)
    target = np.array(A.camera_optical_center_pixel) - np.linalg.solve(cmap, np.array(A.camera_offset_body))
    c, s = np.cos(yaw - A.vision_calibration_yaw), np.sin(yaw - A.vision_calibration_yaw)
    rot_t = np.array([[c, s], [-s, c]])
    w, h = DEFAULT.model.image_width, DEFAULT.model.image_height
    return np.array([pos[:2] + np.linalg.solve(rot_t, cmap @ (np.array(px) - target))
                     for px in [(0, 0), (w, 0), (w, h), (0, h)]])


def _top_view(ax, r, frames, title) -> dict:
    s, wps = r.signals, np.asarray(A.waypoints)
    lo, hi = (A.x_limits[0], A.y_limits[0]), (A.x_limits[1], A.y_limits[1])
    ax.add_patch(plt.Rectangle(lo, hi[0] - lo[0], hi[1] - lo[1], fill=False, ls="--", lw=1, ec="0.4",
                               label="flight area"))
    ax.plot(wps[:, 0], wps[:, 1], "s:", color="0.65", mfc="white", ms=4, lw=0.8, label="search waypoints")
    fly = s["z"] > 0.05
    ax.plot(s["x"][fly], s["y"][fly], color="#4c72b0", lw=1.1, label="vehicle path")
    det = frames[frames[:, 3] > 0]
    ax.plot(det[:, 1], det[:, 2], "o", ms=3.5, color="#55a868", mec="k", mew=0.3,
            label=f"vehicle at a valid detection ({len(det)})")
    out = {"detections": len(det), "max_consecutive": int(frames[:, 4].max()) if len(frames) else 0}
    if len(det):
        k = int(np.argmin(np.abs(r.t - det[0, 0])))
        fp = _footprint(np.array([s["x"][k], s["y"][k], s["z"][k]]), r.scenario.start_yaw)
        ax.add_patch(plt.Polygon(fp, alpha=0.15, color="#55a868", label="camera footprint, first detection"))
    ax.plot(*r.scenario.pad_xy, "*", ms=13, color="gold", mec="k", label=f"pad {r.scenario.pad_xy}")
    if "landed" in r.events:
        err = float(np.hypot(*(r.vehicle.pos[:2] - np.array(r.scenario.pad_xy))))
        ax.plot(*r.vehicle.pos[:2], "X", ms=9, color="#c44e52", mec="k",
                label=f"touchdown {r.events['landed']:.1f} s, {err * 1e3:.0f} mm off")
        out["touchdown_error_m"] = round(err, 4)
    else:
        ax.plot(s["x"][-1], s["y"][-1], "P", ms=9, color="#8172b2", mec="k",
                label=f"at {r.t[-1]:.0f} s: hovering, never landed")
    ax.set_aspect("equal")
    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.2, 1.2)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.25)
    ax.set_title(title)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7)
    return out


def landing_xy_figure(r) -> dict:
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    out = _top_view(ax, r, _frames(r), "Nominal autonomous landing, top view (SIL, stand-in vehicle)")
    fig.tight_layout()
    fig.savefig(FIG / "landing_xy.png")
    plt.close(fig)
    return out


def landing_timeline_figure(r) -> dict:
    s, t = r.signals, r.t
    ci = np.flatnonzero(s["confirmed"] > 0)
    ev = {"arm": r.scenario.arm_at, "takeoff": r.scenario.takeoff_at, "pad confirmed": float(t[ci[0]]),
          "landing ready": r.events["landing_ready"], "landed": r.events["landed"]}
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.5, 4.6), sharex=True, gridspec_kw={"height_ratios": [1.5, 1]})
    a1.plot(t, s["z"], color="#4c72b0", lw=1.4, label="vehicle z")
    a1.plot(t, s["dz"], color="#dd8452", lw=0.9, ls="--", label="desired z sent")
    for h in DEFAULT.model.stage_heights:
        a1.axhline(h, color="0.8", lw=0.7)
    for i, (name, te) in enumerate(ev.items()):
        for a in (a1, a2):
            a.axvline(te, color="0.45", ls=":", lw=0.9)
        a1.text(te, 0.84 - 0.09 * (i % 2), f" {name} {te:.1f} s", fontsize=7, va="top")
    a1.set_ylim(-0.03, 0.88)
    a1.set_ylabel("height (m)")
    a1.legend(loc="center left", fontsize=7)
    a1.set_title("Nominal autonomous landing: height, mission stage and what is sent (SIL)")
    a1.grid(alpha=0.25)
    lanes = [("stage", "descent stage / 5", s["stage"] / 5.0, "#8172b2"),
             ("confirmed", "pad confirmed", s["confirmed"], "#55a868"),
             ("ready", "landing ready", s["ready"], "#ccb974"),
             ("latched", "LAND latched (arbiter)", s["latched"], "#c44e52"),
             ("trig2", "Takeoff/Land bit sent", s["trig2"], "#dd8452"),
             ("gate", "gate status not ok", (s["gate_status"] > 0).astype(float), "#937860"),
             ("abort8", "abort bit 8 (floor)", ((s["abort_bits"].astype(int) & 8) > 0).astype(float), "0.3")]
    for i, (_, _lab, v, c) in enumerate(lanes):
        a2.fill_between(t, i + 0.9 - 0.8 * v, i + 0.9, step="post", color=c, alpha=0.8)
    a2.set_yticks([i + 0.5 for i in range(len(lanes))], [lab for _, lab, _, _ in lanes], fontsize=7)
    a2.set_ylim(0, len(lanes))
    a2.invert_yaxis()
    a2.set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(FIG / "landing_timeline.png")
    plt.close(fig)
    return {"events": {k: round(float(v), 3) for k, v in ev.items()},
            "gate_not_ok_ticks": int((s["gate_status"] > 0).sum()), "final_abort_bits": int(s["abort_bits"][-1])}


def edge_pad_figure() -> dict:
    sc = dataclasses.replace(sim.SCENARIOS["nominal-autoland"], name="edge-pad", pad_xy=EDGE_PAD, duration=120.0)
    r = sim.run(sc, record_block_io=True)
    fr = _frames(r)
    fig = plt.figure(figsize=(7.2, 6.4))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.2, 1])
    ax = fig.add_subplot(gs[0])
    out = _top_view(ax, r, fr, f"Pad {EDGE_PAD}: seen, never confirmed")
    det = fr[fr[:, 3] > 0]
    lo = fr[(fr[:, 0] > det[0, 0] - 2) & (fr[:, 0] < det[-1, 0] + 2)]
    a2 = fig.add_subplot(gs[1])
    a2.step(lo[:, 0], lo[:, 4], where="post", color="#4c72b0", lw=1.4, label="consecutive valid frames")
    a2.plot(lo[lo[:, 3] > 0, 0], lo[lo[:, 3] > 0, 4], "o", ms=4, color="#55a868", mec="k", mew=0.3,
            label="frame with the pad")
    a2.plot(lo[lo[:, 3] == 0, 0], lo[lo[:, 3] == 0, 4], "x", ms=4, color="#c44e52", label="frame without (reset)")
    a2.axhline(A.yolo_confirm_frames, color="k", ls="--", lw=0.9,
               label=f"needed: {A.yolo_confirm_frames} (yoloConfirmFrames)")
    a2.set_ylim(-0.3, A.yolo_confirm_frames + 1)
    a2.set_xlabel("time (s)")
    a2.set_ylabel("count")
    a2.grid(alpha=0.25)
    a2.set_title(f"YOLO confirmation counter, camera at 5 Hz: longest run {out['max_consecutive']}")
    a2.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG / "edge_pad.png")
    plt.close(fig)
    out.update({"pad": list(EDGE_PAD), "confirmed": bool(np.nanmax(r.signals["confirmed"]) > 0),
                "landed": "landed" in r.events, "gate_faults": int(r.signals["faults"].sum()),
                "final_xyz": [round(float(v), 3) for v in r.vehicle.pos]})
    return out


def pad_sweep_figure() -> dict | None:
    cur_p, old_p = DATA / "pad_sweep.json", DATA / f"pad_sweep_{COMPARE_REV}.json"
    if not cur_p.exists():
        return None
    panels = [("this branch", json.loads(cur_p.read_text()))]
    if old_p.exists():
        panels.append((f"{COMPARE_REV}, earlier output gate", json.loads(old_p.read_text())))
    fig, axs = plt.subplots(1, len(panels), figsize=(5.0 * len(panels), 4.2), squeeze=False)
    cmap = matplotlib.colormaps["viridis"]
    gx, gy = panels[0][1]["summary"]["grid"]["x"], panels[0][1]["summary"]["grid"]["y"]
    cw, ch = (gx[1] - gx[0]) / (gx[2] - 1), (gy[1] - gy[0]) / (gy[2] - 1)
    out = {}
    for ax, (label, d) in zip(axs[0], panels, strict=True):
        runs = d["runs"]
        for run in runs:
            x, y = run["pad"]
            if run["landed"]:
                e = run["touchdown_error_m"] * 1e3
                col, txt, tc = cmap(min(e / 25.0, 1.0)), f"{e:.0f}", ("w" if e < 15 else "k")
            elif run["gate_faults"]:
                col, txt, tc = "#dd8452", "stall", "k"
            else:
                col, txt, tc = "#c44e52", "n/c", "w"
            ax.add_patch(plt.Rectangle((x - cw / 2, y - ch / 2), cw, ch, color=col, ec="w", lw=0.6))
            ax.text(x, y, txt, ha="center", va="center", fontsize=6.5, color=tc)
        landed = sum(r["landed"] for r in runs)
        faulted = sum(bool(r["gate_faults"]) for r in runs)
        out[label] = {"landed": landed, "with_gate_faults": faulted, "pads": len(runs),
                      "commit": d["summary"].get("commit")}
        ax.set_title(f"{label}\n{landed}/{len(runs)} landed, {faulted} with gate faults", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(gx[0] - cw, gx[1] + cw)
        ax.set_ylim(gy[0] - ch, gy[1] + ch)
        ax.set_xlabel("pad x (m)")
        ax.set_ylabel("pad y (m)")
    sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=matplotlib.colors.Normalize(0, 25))
    fig.colorbar(sm, ax=axs[0].tolist(), shrink=0.85, label="touchdown error (mm)")
    fig.text(0.01, 0.01, "cell = pad position; number = touchdown error in mm; n/c = never confirmed; "
             "stall = output gate refused the centring entry", fontsize=7)
    fig.suptitle("12 x 8 pad sweep, nominal mission, 120 s per pad (SIL, stand-in vehicle and camera)", fontsize=10)
    fig.savefig(FIG / "pad_sweep.png")
    plt.close(fig)
    return out


def _inject_nan_x(start: str, after_ticks: int, length_ticks: int, duration: float):
    """The nominal mission with the model's desired x replaced by NaN for
    ``length_ticks`` ticks, ``after_ticks`` after centring starts
    (``start = "centring"``) or after the gate applied LAND (``"land"``),
    the injection used by tests/test_sim.py."""
    orig = safety.OutputGate.step
    st = {"k": 0, "k0": None, "window": None}

    def step(self, out):
        st["k"] += 1
        if st["k0"] is None and (self.land_active if start == "land" else bool(out.centering_active)):
            st["k0"] = st["k"]
        if st["k0"] is not None and st["k0"] + after_ticks < st["k"] <= st["k0"] + after_ticks + length_ticks:
            out = dataclasses.replace(out, desired_pose=(float("nan"),) + tuple(out.desired_pose[1:]))
        return orig(self, out)
    safety.OutputGate.step = step
    try:
        r = sim.run(dataclasses.replace(sim.SCENARIOS["nominal-autoland"], name=f"nan-x-{start}", duration=duration))
    finally:
        safety.OutputGate.step = orig
    dt = DEFAULT.model.step_size
    k0 = st["k0"] - 1                       # signal index of the tick the trigger fired
    return r, ((k0 + after_ticks + 1) * dt, (k0 + after_ticks + length_ticks) * dt)


def gate_faults_figure() -> dict:
    cases = [("centring", 1000, 2500, 90.0, "desired x NaN for 2.5 s, 1 s into centring"),
             ("land", 50, 2550, 120.0, "desired x NaN for 2.55 s, 50 ms after LAND applied")]
    fig, axs = plt.subplots(2, 2, figsize=(10, 4.6), sharex="col", gridspec_kw={"height_ratios": [1.4, 1]})
    out = {}
    for col, (start, after, length, dur, title) in enumerate(cases):
        r, (w0, w1) = _inject_nan_x(start, after, length, dur)
        s, t = r.signals, r.t
        az, ab = axs[0, col], axs[1, col]
        for a in (az, ab):
            a.axvspan(w0, w1, color="#c44e52", alpha=0.12, lw=0)
        az.plot(t, s["z"], color="#4c72b0", lw=1.3, label="vehicle z (m)")
        az.set_ylim(-0.05, 0.85)
        az.set_ylabel("z (m)" if col == 0 else "")
        ax_x = az.twinx()
        ax_x.plot(t, s["dx"], color="#dd8452", lw=0.9, label="desired x sent (m, right)")
        ax_x.set_ylim(-1.3, 1.3)
        az.set_title(title, fontsize=9)
        h1, l1 = az.get_legend_handles_labels()
        h2, l2 = ax_x.get_legend_handles_labels()
        az.legend(h1 + h2, l1 + l2, loc="lower left", fontsize=7)
        az.grid(alpha=0.25)
        lanes = [("gate refused", s["gate_status"] == 1.0, "#ccb974"),
                 ("gate fault", s["gate_status"] == 2.0, "#c44e52"),
                 ("LAND latched", s["latched"] > 0, "#8172b2"), ("Takeoff/Land sent", s["trig2"] > 0, "#dd8452")]
        for i, (_, v, c) in enumerate(lanes):
            ab.fill_between(t, i + 0.9 - 0.8 * v.astype(float), i + 0.9, step="post", color=c, alpha=0.85)
        ab.set_yticks([i + 0.5 for i in range(len(lanes))], [lab for lab, _, _ in lanes] if col == 0 else [],
                      fontsize=7)
        ab.set_ylim(0, len(lanes))
        ab.invert_yaxis()
        ab.set_xlabel("time (s)")
        lo, hi = max(0.0, w0 - 12.0), min(t[-1], w1 + 20.0)
        az.set_xlim(lo, hi)
        k_land = np.flatnonzero((s["latched"] > 0) & (s["trig2"] == 0))
        out[start] = {"nan_window_s": [round(w0, 3), round(w1, 3)], "landed": "landed" in r.events,
                      "landed_s": r.events.get("landed"), "max_gate_status": float(np.nanmax(s["gate_status"])),
                      "escalations": r.runtime.gate.escalations, "centring_refusals": r.runtime.gate.centring_refusals,
                      "first_land_applied_s": round(float(t[k_land[0]]), 3) if len(k_land) else None,
                      "trig2_raised_after_land": bool(len(k_land) and (s["trig2"][k_land[0]:] > 0).any())}
    fig.suptitle("Output gate under an injected centring fault (red band): refuse, hold, escalate after 2 s; "
                 "an applied LAND is never interrupted", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIG / "gate_faults.png")
    plt.close(fig)
    return out


def serve_timing_figure() -> dict | None:
    p = DATA / "serve_timing.json"
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    rows = [(f"{d['local']['host'].split(',')[0]} #{i + 1}", r) for i, r in enumerate(d["local"]["runs"])]
    rows += [(f"Windows CI {r['run_id']}", r) for r in d["windows_ci"]]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 0.4 * len(rows) + 1.3), sharey=True)
    ys = np.arange(len(rows))
    for k, c in [("lag_p50_s", "#55a868"), ("lag_p99_s", "#4c72b0"), ("lag_p999_s", "#8172b2"),
                                ("max_lag_s", "#c44e52")]:
        a1.plot([r[k] * 1e3 for _, r in rows], ys, "o", color=c, ms=5, label=k.replace("_s", "").replace("_", " "))
    a1.set_xscale("log")
    a1.axvline(1.0, color="0.5", ls=":", lw=0.9)
    a1.axvline(50.0, color="#c44e52", ls="--", lw=0.9)
    a1.text(50.0, -0.5, "fail-stop 50 ms ", ha="right", fontsize=7, color="#c44e52")
    a1.text(1.0, -0.5, "1 tick ", ha="right", fontsize=7, color="0.4")
    a1.set_xlabel("tick lag behind model time (ms, log)")
    a1.legend(fontsize=7, loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=4, frameon=False)
    a1.set_yticks(ys, [lab for lab, _ in rows], fontsize=7)
    a1.set_ylim(-0.8, len(rows) - 0.4)
    cpu = [100 * r["cpu_s"] / r["wall_s"] for _, r in rows]
    a2.barh(ys, cpu, color="#4c72b0")
    for y, v in zip(ys, cpu, strict=True):
        a2.text(v + 1, y, f"{v:.0f} %", va="center", fontsize=7)
    a2.set_xlim(0, 100)
    a2.set_xlabel("CPU, % of one core")
    fig.suptitle(f"qautoland serve --mode dry-run, {d['duration_s']:.0f} s per run, 1 kHz model time", fontsize=9,
                 y=0.995)
    fig.tight_layout()
    fig.savefig(FIG / "serve_timing.png")
    plt.close(fig)
    return {lab: {"cpu_pct": round(c, 1), "lag_p50_ms": round(r["lag_p50_s"] * 1e3, 4),
                  "lag_p99_ms": round(r["lag_p99_s"] * 1e3, 3), "max_lag_ms": round(r["max_lag_s"] * 1e3, 3)}
            for (lab, r), c in zip(rows, cpu, strict=True)}


def main() -> int:
    FIG.mkdir(parents=True, exist_ok=True)
    DATA.mkdir(parents=True, exist_ok=True)
    results = {n: sim.run(n, record_block_io=(n == "nominal-autoland")) for n in sim.SCENARIOS}
    stats = parity_figure()
    visuals = {"landing_xy": landing_xy_figure(results["nominal-autoland"]),
               "landing_timeline": landing_timeline_figure(results["nominal-autoland"]),
               "edge_pad": edge_pad_figure(), "pad_sweep": pad_sweep_figure(), "gate_faults": gate_faults_figure(),
               "serve_timing": serve_timing_figure()}
    safety = safety_figure(results)
    mut = mutation_figure()
    nom = results["nominal-autoland"]
    (DATA / "parity.json").write_text(json.dumps(stats, indent=1) + "\n")
    (DATA / "scenarios.json").write_text(json.dumps({
        "nominal-autoland": {"events": nom.events, "touchdown": [round(float(v), 4) for v in nom.vehicle.pos[:2]],
                             "pad": list(nom.scenario.pad_xy), "abort_bits": int(nom.signals["abort_bits"][-1])},
        **safety}, indent=1, default=float) + "\n")
    (DATA / "visuals.json").write_text(json.dumps(visuals, indent=1, default=float) + "\n")
    print(json.dumps({"parity": stats, "mutation": mut, "visuals": visuals}, indent=1, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
