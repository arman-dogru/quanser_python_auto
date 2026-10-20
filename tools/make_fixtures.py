"""Generate golden parity fixtures from the MATLAB code inside the Phase 10 model.

Inputs are produced here (deterministic edge-case sequences, seeded random
sequences, and optionally closed-loop SIL scenario traces); outputs come from
GNU Octave running the chart scripts that ``tools/mdl_spec.py`` extracted
verbatim from ``simulink/Mission_Server_QDrone_YOLO_Autonomous_Land.mdl``
(``tools/octave/parity_driver.m``). Python never computes an expected value.

Usage::

    python tools/make_fixtures.py             # (re)write tests/fixtures/parity
    python tools/make_fixtures.py --check     # regenerate in a temp dir, require byte-identical
    python tools/make_fixtures.py --scenarios outputs/scenario_parity
                                              # SIL scenario traces (large; not committed)
"""
from __future__ import annotations

import argparse
import filecmp
import gzip
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CHARTS = ROOT / "docs" / "spec" / "Mission_Server_QDrone_YOLO_Autonomous_Land" / "charts"
OUT = ROOT / "tests" / "fixtures" / "parity"
NAN = math.nan


def fmt(v: float) -> str:
    v = float(v)
    if math.isnan(v):
        return "NaN"
    if math.isinf(v):
        return "Inf" if v > 0 else "-Inf"
    return repr(v)


def write(path: Path, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as f:
        for r in rows:
            f.write(",".join(fmt(v) for v in r) + "\n")


# ----------------------------------------------------------- deterministic
def det_gate():
    m = [0, 0, 0.55, 0]
    seqs = {}
    # matlab/test_phase9_landing_gate.m, verbatim inputs
    rows = [[4, 0, 1, 1, 1, 1, *m, *m, k * 0.20] for k in range(1, 13)]
    rows += [[5, 0, 1, 1, 1, 1, *m, *m, 3 + k * 0.20] for k in range(1, 10)]
    rows += [[5, 0, 1, 1, 1, 1, *m, *m, 5.0], [5, 0, 1, 1, 0, 1, *m, *m, 5.3], [5, 0, 1, 1, 0, 1, *m, *m, 5.6],
             [5, 1, 1, 1, 1, 1, *m, *m, 5.8], [5, 0, 1, 1, 1, 0, *m, *m, 6.0],
             [5, 0, 1, 1, 1, 1, 0.09, 0, 0.55, 0, *m, 6.2], [5, 0, 1, 1, 1, 1, 0, 0, 0.60, 0, *m, 6.4]]
    seqs["original_test"] = rows
    # Boundaries: exactly 0.08 m, exactly 0.04 m, exactly 0.50 s.
    b = []
    for k in range(10):
        b.append([5, 0, 1, 1, 1, 1, 0.08, 0, 0.55, 0, 0, 0, 0.55, 0, 1 + 0.1 * k])
    b.append([5, 0, 1, 1, 0, 1, 0.08, 0, 0.59, 0, 0, 0, 0.55, 0, 1.9 + 0.5])
    b.append([5, 0, 1, 1, 0, 1, 0, 0, 0.55, 0, 0, 0, 0.55, 0, 1.9 + 0.5000001])
    b.append([5, 0, 1, 1, 1, 1, 0, 0, 0.51, 0, 0, 0, 0.55, 0, 3.0])
    b.append([5, 0, 1, 1, 1, 1, NAN, 0, 0.55, 0, 0, 0, 0.55, 0, 3.1])
    b.append([5, 0, 1, 1, 1, 1, 0, 0, 0.55, 0, 0, 0, 0.55, 0, NAN])
    b.append([5, 0, 1, 1, 1, 1, 0, 0, 0.55, 0, NAN, 0, 0.55, 0, 3.2])
    b.append([5, 0, 0, 1, 1, 1, 0, 0, 0.55, 0, 0, 0, 0.55, 0, 3.3])
    b.append([5, 0, 1, 0, 1, 1, 0, 0, 0.55, 0, 0, 0, 0.55, 0, 3.4])
    seqs["boundaries"] = b
    return seqs


def det_abort():
    p = [0, 0, 0.70, 0]
    bad = [NAN, 0, 0.40, 0]
    rows = [[1, 0, 1, 1, *p, 0.1, 10], [1, 1, 0, 1, *p, 0.58, 10.58], [1, 1, 1, 0, *p, 0.02, 10.60],
            [0, 0, 0, 1, *p, 0, 11], [1, 1, 1, 0, *p, 0.1, 12], [0, 0, 0, 1, *p, 0, 13],
            [1, 1, 0, 0, *bad, 0.8, 14],
            [0, 0, 1, 1, *p, 0, 15], [3, 1, 1, 1, *p, math.inf, 16],             # unknown cause -> 16
            [0, 1, 1, 1, *p, 0, 17],                                            # stage 0 with abort: no reset
            [0, 0, 1, 1, *p, 0, 18], [2, 1, 1, 1, 0, 0, 0.45, 0, 0.3, 19],       # exactly 0.45: not below
            [0, 0, 1, 1, *p, 0, 20], [2, 1, 1, 1, 0, 0, NAN, 0, 0.3, 21]]        # NaN z: bit 4 only
    return {"original_test_and_edges": rows}


def det_arbiter():
    raw = [[0, 0, 0, 0], [1, 1, 0, 1], [1, 1, 0, 1], [1, 1, 0, 1], [1, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 1],
           [0, 1, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1]]
    ready = [0, 0, 1, 0, 0, 0, 0, 0, 1, 0]
    seq = [[*r, q] for r, q in zip(raw, ready, strict=True)]
    extra = [[1, 1, 1, 0, 1], [1, 1, 1, 0, 0], [1, 1, 0, 0, 0], [0.5, 1, 0, 0, 1], [1, 0.5, 0, 0, 1],
             [1, 1, 0, 0, 0.5], [1, 1, 0, 0, 0.6], [0.6, 0.6, 0.7, 0.8, 0]]
    return {"original_test": seq, "thresholds": extra}


def det_confirm():
    t = 0.0
    rows = []
    hold = [0.1, -0.2, 0.7, 0.01]
    trig_off, trig_on = [1, 0, 0, 0], [1, 1, 0, 0]
    frame = 0
    def obs(det, u, v, conf, new=1, trig=trig_on, fr=None):
        nonlocal t, frame
        t += 0.2
        if fr is None:
            frame += 1
            fr = frame
        rows.append([*hold, det, u, v, conf, fr, 1e9 + t, new, t, *trig])
    obs(1, 300, 200, 0.9, trig=trig_off)
    for _ in range(4):
        obs(1, 300, 200, 0.9)
    obs(1, 300, 200, 0.9, fr=frame)        # repeated frame number: ignored
    obs(1, 300, 200, 0.9)                  # 5th distinct -> confirmed
    obs(1, 300, 200, 0.6)                  # exactly the confidence threshold
    obs(1, 300, 200, 0.5999999)            # just below
    obs(1, 0, 0, 0.9)
    obs(1, 640, 480, 0.9)
    obs(1, 640.0001, 480, 0.9)
    obs(1, NAN, 200, 0.9)
    for _ in range(10):
        obs(0, 0, 0, 0)                    # release after 10 bad frames
    for _ in range(6):
        obs(1, 320, 240, 0.95)
    obs(1, 320, 240, 0.95, new=0)          # not new: nothing happens
    t += 0.3
    rows.append([*hold, 1, 320, 240, 0.95, frame, 1e9, 0, t, *trig_on])
    t += 0.21
    rows.append([*hold, 1, 320, 240, 0.95, frame, 1e9, 0, t, *trig_on])    # > 0.5 s since fresh: drops
    obs(1, 320, 240, 0.95, trig=trig_off)
    obs(1, 320, 240, 0.95, fr=frame)       # same frame after re-takeoff: ignored
    return {"edges": rows}


def det_extract():
    rows = [[1, 2, 3, 4, 5, 6, 1, 1], [1, 2, 3, 4, 5, 6, 0, 1], [1, 2, 3, 4, 5, 6, 1, 0], [1, 2, 3, 4, 5, 6, 1, 2],
            [NAN, 2, 3, 4, 5, 6, 1, 1], [1, 2, 3, 4, 5, 6, 1, -1], [1, 2, 3, 4, 5, 6, 1, 1.5]]
    return {"edges": rows}


def det_grid():
    """Scripted centring/descent edge cases. The measured pose is shaped from
    the Python port's own desired pose (input shaping only; every expected
    value still comes from Octave)."""
    from qdrone_autoland.config import DEFAULT
    from qdrone_autoland.logic.grid_manager import GridSearchManager
    g = GridSearchManager(DEFAULT)
    tx, ty = (float(v) for v in DEFAULT.auto.vision_target_pixel)
    rows = []
    home = [0.0, -0.9, 0.7, 0.0]
    desired = [0.0, 0.0, 0.7, 0.01]

    def tick(mp, takeoff=1.0, confirmed=1.0, hold=(0.0, 0.0, 0.7, 0.01), pad=(tx, ty), new=1.0, trk=1.0):
        nonlocal desired
        r = [*mp, *home, 1.0, takeoff, 0.0, 0.0, confirmed, *hold, pad[0], pad[1], new, trk]
        rows.append(r)
        o = g.step(r[0:4], r[4:8], r[8:12], r[12] != 0, r[13:17], r[17], r[18], r[19] != 0, r[20])
        desired = list(o.desired_pose)
        return o

    hover = [0.0, 0.0, 0.70, 0.01]
    for _ in range(10):
        tick(hover, takeoff=0.0, confirmed=0.0)
    for _ in range(20):                       # confirmed but not tracking: centring must not start
        tick(hover, trk=0.0)
    for _ in range(150):                      # far pad: 5 mm steps, then the 0.5 m displacement limit
        tick(hover, pad=(400.0, 330.0))
    for _ in range(20):                       # on target but 0.18 m from the centring pose: no descent
        tick([desired[0] + 0.18, desired[1], 0.70, 0.01])
    for _ in range(20):                       # on target, aligned, wrong height: good frames but stage 1
        tick([desired[0], desired[1], 0.60, 0.01])
    for _ in range(120):                      # follow the ladder down to stage 5 and hold
        tick([desired[0], desired[1], desired[2], 0.01])
    for _ in range(5):                        # exactly at the 0.45 m floor: not below it
        tick([desired[0], desired[1], 0.45, 0.01])
    for _ in range(5):
        tick([desired[0], desired[1], 0.44, 0.01])
    for _ in range(5):
        tick(hover, takeoff=0.0, confirmed=0.0)
    for _ in range(5):                        # NaN hold pose: MATLAB min/max clamp NaN to the limit
        tick(hover, hold=(NAN, NAN, 0.7, 0.0), pad=(400.0, 330.0))
    return {"centring_edges": rows}


def det_mscomm():
    rows = [[t, s, e] for t in (0.5, 1.0, 1.0001, 5) for s in (0, 1, 2, 3) for e in (-1, 0, 1)]
    return {"grid": rows}


# ----------------------------------------------------------------- random
def rand_bool(rng, p=0.5):
    return 1.0 if rng.random() < p else 0.0


def rnd_gate(rng, n):
    rows, t = [], 0.0
    stage = 5
    for _ in range(n):
        t += rng.choice([0.001, 0.05, 0.2, 0.3, 0.6])
        if rng.random() < 0.05:
            stage = float(rng.integers(0, 6))
        m = [rng.normal(0, 0.05), rng.normal(0, 0.05), 0.55 + rng.normal(0, 0.03), rng.normal(0, 0.1)]
        d = [0.0, 0.0, 0.55, 0.0]
        if rng.random() < 0.01:
            m[int(rng.integers(0, 4))] = NAN
        rows.append([stage, rand_bool(rng, 0.03), rand_bool(rng, 0.95), rand_bool(rng, 0.9), rand_bool(rng, 0.6),
                     rand_bool(rng, 0.97), *m, *d, t])
    return rows


def rnd_abort(rng, n):
    rows, t = [], 0.0
    for _ in range(n):
        t += 0.01
        z = rng.uniform(0.3, 0.8)
        m = [rng.normal(0, 0.1), rng.normal(0, 0.1), z, 0.0]
        if rng.random() < 0.05:
            m[int(rng.integers(0, 4))] = NAN
        stage = float(rng.integers(0, 6)) if rng.random() < 0.1 else (0.0 if rng.random() < 0.05 else 3.0)
        rows.append([stage, rand_bool(rng, 0.3), rand_bool(rng, 0.8), rand_bool(rng, 0.9), *m,
                     rng.uniform(0, 1), t])
    return rows


def rnd_arbiter(rng, n):
    return [[rand_bool(rng, 0.9), rand_bool(rng, 0.85), rand_bool(rng, 0.05), rand_bool(rng, 0.05),
             rand_bool(rng, 0.1)] for _ in range(n)]


def rnd_confirm(rng, n):
    rows, t, frame = [], 0.0, 0
    takeoff = 1.0
    for _ in range(n):
        t += rng.choice([0.001, 0.05, 0.2, 0.2, 0.6])
        if rng.random() < 0.02:
            takeoff = 1.0 - takeoff
        new = rand_bool(rng, 0.7)
        if new and rng.random() < 0.9:
            frame += 1
        det = rand_bool(rng, 0.8)
        u, v = rng.uniform(-20, 660), rng.uniform(-20, 500)
        conf = rng.uniform(0.4, 1.0)
        if rng.random() < 0.02:
            u = NAN
        pose = [rng.normal(0, 0.5), rng.normal(0, 0.5), rng.uniform(0.5, 0.8), rng.normal(0, 0.2)]
        rows.append([*pose, det, u, v, conf, float(frame), 1e9 + t, new, t, 1.0, takeoff, 0.0, 0.0])
    return rows


def rnd_grid(rng, n, tracking_follow=True):
    """Random but *mission-shaped*: the measured pose chases the last desired
    pose from a Python-independent target schedule, so long dwell counters,
    waypoint advances, centring and the descent ladder are all reached."""
    from qdrone_autoland.config import DEFAULT  # only for waypoint coordinates
    wps = DEFAULT.auto.waypoints
    rows = []
    pos = np.array([0.0, -0.9, 0.0, 0.0])
    home = [0.0, -0.9, 0.7, 0.0]
    takeoff = 0.0
    confirmed = 0.0
    hold = [0.0, 0.0, 0.7, 0.0]
    target = np.array(home[:3])
    wp = 0
    phase_len = 0
    padu, padv = 300.0, 200.0
    for k in range(n):
        if k == 5:
            takeoff = 1.0
        if rng.random() < 0.0005:
            takeoff = 1.0 - takeoff
        phase_len += 1
        if phase_len > 130:                     # move to the next waypoint after a dwell
            phase_len = 0
            wp = (wp + 1) % len(wps)
            target = wps[wp].copy()
        if not confirmed and rng.random() < 0.002:
            confirmed = 1.0
            hold = [*pos[:3], pos[3]]
        elif confirmed and rng.random() < 0.0005:
            confirmed = 0.0
        if confirmed:
            stage_z = rng.choice([0.70, 0.60, 0.55])
            target = np.array([hold[0], hold[1], stage_z])
        pos[:3] += (target - pos[:3]) * 0.05 + rng.normal(0, 0.002, 3)
        pos[3] += rng.normal(0, 0.002)
        m = list(pos)
        if rng.random() < 0.002:
            m[int(rng.integers(0, 4))] = NAN
        newp = rand_bool(rng, 0.2)
        if newp:
            padu = float(rng.normal(253, 25))
            padv = float(rng.normal(237, 25))
        trk = 1.0 if rng.random() > 0.001 else 0.0
        rows.append([*m, *home, 1.0, takeoff, 0.0, 0.0, confirmed, *hold, padu, padv, newp, trk])
    return rows


def rnd_extract(rng, n):
    return [[*rng.normal(0, 1, 7), float(rng.integers(-1, 3))] for _ in range(n)]


def rnd_mscomm(rng, n):
    return [[rng.uniform(0, 3), float(rng.integers(0, 4)), float(rng.integers(-2, 2))] for _ in range(n)]


def build_inputs(out: Path) -> None:
    det = {"grid": det_grid(), "gate": det_gate(), "abort": det_abort(), "arbiter": det_arbiter(),
           "confirm": det_confirm(),
           "extract": det_extract(), "mscomm": det_mscomm()}
    for block, seqs in det.items():
        for name, rows in seqs.items():
            write(out / block / f"det_{name}.in.csv", rows)
    gens = {"gate": (rnd_gate, 3, 400), "abort": (rnd_abort, 3, 400),
            "arbiter": (rnd_arbiter, 3, 300),
            "confirm": (rnd_confirm, 3, 600), "grid": (rnd_grid, 2, 4000), "extract": (rnd_extract, 1, 200),
            "mscomm": (rnd_mscomm, 1, 200)}
    for block, (gen, count, n) in gens.items():
        for seed in range(count):
            rng = np.random.default_rng(1000 * (seed + 1) + len(block))
            rows = [[round(v, 6) if math.isfinite(v) else v for v in map(float, r)] for r in gen(rng, n)]
            write(out / block / f"rnd_seed{seed}.in.csv", rows)


def build_scenarios(out: Path, names=None) -> None:
    from qdrone_autoland import sim
    for name in names or sim.SCENARIOS:
        res = sim.run(name, record_block_io=True)
        for block, rows in res.block_io.items():
            write(out / block / f"scn_{name}.in.csv", rows)


COMMITTED_SCENARIO = "nominal-autoland"   # one full closed-loop mission, gzip-compressed


def build_committed_scenario(out: Path) -> None:
    """Write ``scn_<name>.in.csv.gz`` for the five chained blocks.

    The inputs come from the closed-loop SIL run (``qdrone_autoland.sim``), so
    every stage of the search/centre/descend/gate/LAND sequence is exercised;
    the outputs still come only from Octave. ``--check`` re-runs Octave on the
    committed inputs instead of regenerating them, so libm differences between
    machines cannot make the check flaky.
    """
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        build_scenarios(tmp, [COMMITTED_SCENARIO])
        for f in tmp.rglob("*.in.csv"):
            dst = out / f.relative_to(tmp)
            dst.parent.mkdir(parents=True, exist_ok=True)
            with gzip.GzipFile(dst.with_name(dst.name + ".gz"), "wb", mtime=0) as g:
                g.write(f.read_bytes())


def octave_on_gz(src_root: Path, out: Path) -> None:
    """Decompress committed scenario inputs, run Octave, gzip the outputs into ``out``."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for gz in src_root.rglob("*.in.csv.gz"):
            dst = tmp / gz.relative_to(src_root).with_suffix("")
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(gzip.decompress(gz.read_bytes()))
        if not any(tmp.rglob("*.csv")):
            return
        run_octave(tmp)
        for f in tmp.rglob("*.out.csv"):
            dst = out / f.relative_to(tmp)
            dst.parent.mkdir(parents=True, exist_ok=True)
            with gzip.GzipFile(dst.with_name(dst.name + ".gz"), "wb", mtime=0) as g:
                g.write(f.read_bytes())


def run_octave(out: Path) -> None:
    cmd = ["octave-cli", "--no-gui", "--quiet", "--eval",
           f"addpath('{ROOT / 'tools' / 'octave'}'); parity_driver('{CHARTS}', '{out}')"]
    subprocess.run(cmd, check=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--scenarios", type=Path, help="write SIL scenario fixtures to this directory")
    args = ap.parse_args(argv)
    sys.path.insert(0, str(ROOT / "src"))
    if args.scenarios:
        shutil.rmtree(args.scenarios, ignore_errors=True)
        build_scenarios(args.scenarios)
        run_octave(args.scenarios)
        return 0
    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            build_inputs(tmp)
            run_octave(tmp)
            for gz in OUT.rglob("*.in.csv.gz"):
                dst = tmp / gz.relative_to(OUT)
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(gz.read_bytes())
            octave_on_gz(OUT, tmp)
            cmp = filecmp.dircmp(tmp, OUT)
            bad = []
            def walk(c, prefix=""):
                bad.extend(prefix + f for f in c.left_only + c.right_only + c.diff_files)
                for name, sub in c.subdirs.items():
                    walk(sub, prefix + name + "/")
            walk(cmp)
            # dircmp compares shallowly by stat; confirm content explicitly
            last_bits = []
            for f in [*tmp.rglob("*.csv"), *tmp.rglob("*.csv.gz")]:
                g = OUT / f.relative_to(tmp)
                if g.exists() and _content(f) != _content(g):
                    if ".out." in f.name and _close(_content(f), _content(g)):
                        last_bits.append(str(f.relative_to(tmp)))
                    else:
                        bad.append(str(f.relative_to(tmp)))
            if bad:
                print("fixtures differ:", *sorted(set(bad)), sep="\n  ")
                return 1
            if last_bits:
                # A different Octave build/BLAS can round the 2x2 centring
                # products differently; anything beyond 1e-12 is a failure.
                print(f"fixtures regenerate within 1e-12 ({len(last_bits)} output files differ in the last bits:",
                      ", ".join(sorted(last_bits)) + ")")
                return 0
            print("fixtures regenerate byte-identically")
            return 0
    shutil.rmtree(OUT, ignore_errors=True)
    build_inputs(OUT)
    run_octave(OUT)
    build_committed_scenario(OUT)
    octave_on_gz(OUT, OUT)
    return 0


def _close(a: bytes, b: bytes, tol: float = 1e-12) -> bool:
    x = np.loadtxt(a.decode().splitlines(), delimiter=",", ndmin=2)
    y = np.loadtxt(b.decode().splitlines(), delimiter=",", ndmin=2)
    if x.shape != y.shape or (np.isnan(x) != np.isnan(y)).any():
        return False
    m = ~np.isnan(x)
    return bool(np.all(np.abs(x[m] - y[m]) <= tol) and np.array_equal(np.isinf(x), np.isinf(y)))


def _content(p: Path) -> bytes:
    data = p.read_bytes()
    return gzip.decompress(data) if p.suffix == ".gz" else data


if __name__ == "__main__":
    sys.exit(main())
