"""Generate golden parity fixtures by running the extracted MATLAB charts in
GNU Octave.

The extracted ``.m`` files under ``reference/extracted`` are executed
unmodified (persistent state included): Octave calls each file by its name,
and ``clear <name>`` resets its persistent variables at the start of every
sequence. The only modified copy is the ``enable_descent = 1`` variant of the
Stage 6 generator, made by replacing exactly one line in a temporary copy so
that the otherwise dormant descent branch is covered too.

Inputs are deterministic edge-case sequences plus seeded random sequences.
Everything is written to ``tests/fixtures/*.json``; the fixtures are
deterministic, so ``--check`` regenerates them and fails if anything differs.

Usage::

    python tools/make_parity_fixtures.py            # (re)write fixtures
    python tools/make_parity_fixtures.py --check    # verify committed ones
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXTRACTED = ROOT / "reference" / "extracted"
STAGE6 = EXTRACTED / "QDrone_YOLO_Stage6_RealYOLO_MonitorOnly"
RECENTER = EXTRACTED / "QDrone_Recenter_Logic_Test"
LEGACY = ROOT / "reference" / "matlab" / "legacy"
DRIVER = ROOT / "tools" / "octave" / "parity_driver.m"
OUT = ROOT / "tests" / "fixtures"
SEEDS = (1, 2, 3)
PI = math.pi

# Deadband edge values: exactly 40 is inside (inclusive), the next double
# above 40 is outside.
EDGE = [40.0, math.nextafter(40.0, math.inf), math.nextafter(40.0, 0.0), 39.0, 41.0]
EDGES = sorted({v for e in EDGE for v in (e, -e)} | {0.0})


def fmt(v: float) -> str:
    if math.isnan(v):
        return "NaN"
    if math.isinf(v):
        return "Inf" if v > 0 else "-Inf"
    return repr(float(v))


# --------------------------------------------------------------- cases
def pose_cases() -> list[dict]:
    """Each case: name + list of (x, y, z, yaw, detected, ex, ey)."""
    cases: list[dict] = []
    P = (0.3, -0.2, 1.25, 0.0)

    def add(name, steps):
        cases.append({"name": name, "steps": [list(map(float, s)) for s in steps]})

    for ex in (-400, -320, -100, -41, 41, 100, 320, 400):
        for ey in (-300, -240, -100, -41, 0, 41, 100, 240, 300):
            add(f"sign ex={ex} ey={ey}", [(*P, 1, ex, ey)] * 3)
    for ex in EDGES:
        for ey in EDGES:
            add(f"deadband ex={ex!r} ey={ey!r}", [(*P, 1, ex, ey)] * 2)
    add("smoothing converge forward", [(*P, 1, -200, 0)] * 40)
    add("smoothing converge right", [(*P, 1, 0, 180)] * 40)
    add("smoothing step then centre", [(*P, 1, 300, -200)] * 12 + [(*P, 1, 0, 0)] * 20)
    add("loss decay after drive", [(*P, 1, 250, -150)] * 15 + [(*P, 0, 250, -150)] * 25)
    add("loss decay keeps errors ignored", [(*P, 1, -90, 90)] * 5 + [(*P, 0, 1e6, -1e6)] * 10)
    add("ten updates exact", [(*P, 1, 10, -10)] * 9 + [(*P, 1, 10, -10)] * 3)
    add("ten updates broken by offcentre at 10", [(*P, 1, 0, 0)] * 9 + [(*P, 1, 50, 0)] + [(*P, 1, 0, 0)] * 11)
    add("ten updates broken by loss", [(*P, 1, 40, 40)] * 9 + [(*P, 0, 0, 0)] + [(*P, 1, 40, 40)] * 12)
    add("ten updates counter keeps growing", [(*P, 1, 0, 0)] * 60)
    add("detected=2 no decay", [(*P, 1, 200, 200)] * 5 + [(*P, 2, 200, 200)] * 5)
    add("detected=0.5", [(*P, 1, -200, 50)] * 5 + [(*P, 0.5, 0, 0)] * 5)
    add("detected=NaN", [(*P, 1, -200, 50)] * 5 + [(*P, math.nan, 0, 0)] * 5)
    add("detected=-1", [(*P, 1, 120, 120)] * 5 + [(*P, -1, 0, 0)] * 5)
    add("nan errors clamp", [(*P, 1, math.nan, 100)] * 3 + [(*P, 1, 100, math.nan)] * 3
        + [(*P, 1, math.nan, math.nan)] * 3)
    add("inf errors clamp", [(*P, 1, math.inf, -math.inf)] * 3 + [(*P, 1, -math.inf, math.inf)] * 3)
    add("huge errors clamp", [(*P, 1, 1e300, -1e300)] * 3)
    for yaw in (0.0, PI / 2, PI, -PI / 2, 0.3, -2.9, 2 * PI + 0.1, -7.0, 100.0):
        add(f"yaw rotation {yaw!r}", [(1.0, 2.0, 1.25, yaw, 1, -160, 120)] * 6
            + [(1.0, 2.0, 1.25, yaw, 0, 0, 0)] * 3)
    add("pose passthrough z/yaw", [(-1.5, 1.75, 0.0, 3.0, 1, 0, 0), (2.0, -2.0, 5.0, -3.0, 1, 0, 0)])
    add("nan yaw", [(0.0, 0.0, 1.0, math.nan, 1, 100, 100)] * 2)
    add("inf pose", [(math.inf, -math.inf, 1.0, 0.0, 1, 100, 100)] * 2)
    add("varying pose while centred", [(0.01 * k, -0.02 * k, 1 + 0.001 * k, 0.05 * k, 1, 5, -5) for k in range(15)])
    return cases


def random_steps(rng: random.Random, n: int, with_pose: bool) -> list[list[float]]:
    steps = []
    detected = 1.0
    ex = ey = 0.0
    for _ in range(n):
        # Markov detection with occasional odd values.
        r = rng.random()
        if r < 0.08:
            detected = 1.0 - detected if detected in (0.0, 1.0) else 1.0
        elif r < 0.10:
            detected = rng.choice([2.0, 0.5, -1.0, math.nan])
        elif detected not in (0.0, 1.0):
            detected = rng.choice([0.0, 1.0])
        mode = rng.random()
        if mode < 0.25:
            ex, ey = rng.uniform(-40, 40), rng.uniform(-40, 40)
        elif mode < 0.40:
            ex, ey = rng.choice(EDGES), rng.choice(EDGES)
        elif mode < 0.55:
            ex, ey = float(rng.randint(-320, 320)), float(rng.randint(-240, 240))
        elif mode < 0.60:
            ex, ey = rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)
        elif mode < 0.62:
            ex, ey = rng.choice([math.nan, math.inf, -math.inf, 1e9]), rng.uniform(-300, 300)
        elif mode < 0.80:
            ex, ey = ex + rng.gauss(0, 15), ey + rng.gauss(0, 15)  # drift
        else:
            ex, ey = rng.uniform(-330, 330), rng.uniform(-250, 250)
        if with_pose:
            pose = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0, 5), rng.uniform(-2 * PI, 2 * PI)]
            steps.append([*pose, detected, ex, ey])
        else:
            steps.append([detected, ex, ey])
    return steps


def random_cases(with_pose: bool, seeds=SEEDS, n_seq=12, length=250) -> list[dict]:
    out = []
    for seed in seeds:
        rng = random.Random(seed)
        for k in range(n_seq):
            out.append({"name": f"random seed={seed} seq={k}", "steps": random_steps(rng, length, with_pose)})
    return out


def recenter_cases() -> list[dict]:
    # Reuse the pose sequences without the pose columns.
    det = [{"name": c["name"], "steps": [s[4:] for s in c["steps"]]} for c in pose_cases()
           if not c["name"].startswith(("yaw", "pose", "nan yaw", "inf pose", "varying"))]
    det.append({"name": "descend request after ten", "steps": [[1, 20, 20]] * 14 + [[1, 60, 0]] * 3})
    det.append({"name": "lateral forced to zero in state 3",
                "steps": [[1, 300, 0]] * 8 + [[1, 0, 0]] * 15})
    return det


def extract_cases():
    data = [
        [float(i + 1) for i in range(14)],
        [float(i) * 0.5 - 3 for i in range(7)],
        [math.sin(i) for i in range(21)],
        [float(i) for i in range(10)],  # length not a multiple of 7
        [],
    ]
    rows = []
    for d_i, d in enumerate(data, start=1):
        n = len(d) / 7
        for idx in (-1, 0, 1, 2, 3, 4, n, n + 1, 0.5, 1.5, math.nan, math.inf):
            rows.append([1, d_i, idx])
    return data, rows


def commloss_rows():
    rows = []
    for t in (0.0, 0.5, 1.0, math.nextafter(1.0, 2.0), 1.001, 5.0, math.nan):
        for st in (0, 1, 2, 3, math.nan):
            for err in (-1, -1e-300, 0, 1, math.nan):
                rows.append([1, t, st, err])
    return rows


def legacy_rows(rng: random.Random):
    wp, al, de, gr = [], [], [], []
    for _ in range(200):
        p = [rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5), rng.uniform(0, 2)]
        t = [v + rng.choice([rng.gauss(0, 0.04), rng.uniform(-2, 2)]) for v in p]
        wp.append([1, *p, *t])
    wp += [[1, 0, 0, 1, 0, 0, 1], [1, 0, 0, 1, 0.08, 0, 1], [1, 0, 0, 1, math.nan, 0, 1]]
    for w, h in ((640, 480), (1280, 720), (641, 481)):
        for _ in range(60):
            c = [rng.uniform(-50, w + 50), rng.uniform(-50, h + 50)]
            al.append([1, w, h, *c])
            de.append([1, w, h, *c, 0.0, 0.0, rng.choice([0.05, 0.1, 0.10000001, 1.0])])
        al.append([1, w, h, w / 2 + 25, h / 2])
        al.append([1, w, h, math.nan, math.nan])
        de.append([1, w, h, math.nan, math.nan, 0, 0, 0.1])
    for spacing in (0.30, 0.25, 0.5, 0.7, 2.5):
        gr.append([1, -1.0, 1.0, -1.0, 1.0, spacing, 1.0])
    gr.append([1, -0.9, 1.2, -0.3, 0.6, 0.3, 0.8])
    gr.append([1, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0])
    return wp, al, de, gr


# --------------------------------------------------------------- octave
def run_octave(chart: str, fn: str, path_dirs: list[Path], rows: list[list[float]],
               data=None) -> list[list[float] | str]:
    with tempfile.TemporaryDirectory() as td:
        case = Path(td) / "case_file.m"
        lines = [f"CHART = '{chart}';", f"FN = '{fn}';"]
        for d in path_dirs:
            lines.append(f"addpath('{d.as_posix()}');")
        if data is not None:
            lines.append("DATA = {" + ", ".join("[" + " ".join(map(fmt, d)) + "]" for d in data) + "};")
        lines.append("ROWS = [")
        lines += [" ".join(map(fmt, r)) + ";" for r in rows]
        lines.append("];")
        lines.append(f"source('{DRIVER.as_posix()}');")
        case.write_text("\n".join(lines) + "\n")
        proc = subprocess.run(["octave-cli", "--no-gui", "--norc", "-q", str(case)],
                              capture_output=True, text=True, check=True, cwd=td)
    results: list[list[float] | str] = []
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts or not parts[0].isdigit():
            continue
        if len(parts) > 1 and parts[1] == "ERR":
            results.append("ERR")
        else:
            results.append([float(p) for p in parts[1:]])
    if len(results) != len(rows):
        raise RuntimeError(f"{chart}: expected {len(rows)} results, got {len(results)}\n{proc.stderr}")
    return results


def flatten(cases: list[dict]) -> list[list[float]]:
    rows = []
    for c in cases:
        for i, s in enumerate(c["steps"]):
            rows.append([1.0 if i == 0 else 0.0, *s])
    return rows


def regroup(cases, results):
    out, k = [], 0
    for c in cases:
        n = len(c["steps"])
        out.append({"name": c["name"], "inputs": c["steps"], "expected": results[k:k + n]})
        k += n
    return out


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def octave_version() -> str:
    return subprocess.run(["octave-cli", "--version"], capture_output=True, text=True).stdout.splitlines()[0]


def build() -> dict[str, dict]:
    fixtures: dict[str, dict] = {}
    meta = {"generator": "tools/make_parity_fixtures.py", "oracle": octave_version(), "seeds": list(SEEDS)}

    pose_src = STAGE6 / "chart_51_YOLOPoseCommandGenerator.m"
    pc = pose_cases() + random_cases(True)
    fixtures["pose_command"] = {**meta, "source": pose_src.relative_to(ROOT).as_posix(),
                                "source_sha256": sha(pose_src), "columns": {
                                    "inputs": ["x", "y", "z", "yaw", "detected", "error_x", "error_y"],
                                    "expected": ["cmd_x", "cmd_y", "cmd_z", "cmd_yaw", "forward_cmd",
                                                 "right_cmd", "state"]},
                                "cases": regroup(pc, run_octave("pose", pose_src.stem, [STAGE6], flatten(pc)))}

    # enable_descent = 1 variant, one textual substitution in a temp copy.
    with tempfile.TemporaryDirectory() as td:
        text = pose_src.read_text(encoding="utf-8")
        old = "enable_descent = 0;"
        assert text.count(old) == 1, "expected exactly one enable_descent assignment"
        variant = Path(td) / "pose_descent_variant.m"
        variant.write_text(text.replace(old, "enable_descent = 1;"), encoding="utf-8")
        dc = [c for c in pose_cases() if c["name"].startswith(("ten", "loss", "varying"))] + random_cases(
            True, seeds=(4,), n_seq=4, length=200)
        fixtures["pose_command_descent_enabled"] = {
            **meta, "source": pose_src.relative_to(ROOT).as_posix() + " with 'enable_descent = 1;'",
            "source_sha256": sha(pose_src),
            "cases": regroup(dc, run_octave("pose", variant.stem, [Path(td)], flatten(dc)))}

    rec_src = RECENTER / "chart_12_LandingPadRecenterLogic.m"
    rc = recenter_cases() + random_cases(False)
    fixtures["recenter"] = {**meta, "source": rec_src.relative_to(ROOT).as_posix(),
                            "source_sha256": sha(rec_src), "columns": {
                                "inputs": ["detected", "error_x", "error_y"],
                                "expected": ["forward_cmd", "right_cmd", "descend_cmd", "state"]},
                            "cases": regroup(rc, run_octave("recenter", rec_src.stem, [RECENTER], flatten(rc)))}

    ex_src = STAGE6 / "chart_13_fcn.m"
    data, rows = extract_cases()
    res = run_octave("extract", ex_src.stem, [STAGE6], rows, data=data)
    fixtures["extract_object_data"] = {**meta, "source": ex_src.relative_to(ROOT).as_posix(),
                                       "source_sha256": sha(ex_src), "data": data,
                                       "rows": [{"data": int(r[1]) - 1, "index": r[2], "expected": e}
                                                for r, e in zip(rows, res, strict=True)]}

    cl_src = STAGE6 / "chart_33_fcn.m"
    rows = commloss_rows()
    res = run_octave("commloss", cl_src.stem, [STAGE6], rows)
    fixtures["comm_loss"] = {**meta, "source": cl_src.relative_to(ROOT).as_posix(),
                             "source_sha256": sha(cl_src),
                             "rows": [{"inputs": r[1:], "expected": e} for r, e in zip(rows, res, strict=True)]}

    wp, al, de, gr = legacy_rows(random.Random(7))
    legacy = {**meta, "source": "reference/matlab/legacy (config.m effective values)",
              "source_sha256": {p.name: sha(p) for p in sorted(LEGACY.glob("*.m"))}}
    for name, rows in (("waypoint", wp), ("align", al), ("descend", de), ("grid", gr)):
        res = run_octave(name, "config", [LEGACY], rows)
        legacy[name] = [{"inputs": r[1:], "expected": e} for r, e in zip(rows, res, strict=True)]
    with tempfile.TemporaryDirectory() as td:
        script = Path(td) / "dump_config.m"
        script.write_text(f"addpath('{LEGACY.as_posix()}'); p = config(); f = fieldnames(p);\n"
                          "for i = 1:numel(f), printf('%s %.17g\\n', f{i}, double(p.(f{i}))); end\n")
        out = subprocess.run(["octave-cli", "--no-gui", "--norc", "-q", str(script)],
                             capture_output=True, text=True, check=True).stdout
    legacy["config"] = {ln.split()[0]: float(ln.split()[1]) for ln in out.splitlines() if ln.strip()}
    fixtures["legacy"] = legacy
    return fixtures


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if shutil.which("octave-cli") is None:
        print("octave-cli not found; install GNU Octave to regenerate fixtures", file=sys.stderr)
        return 2
    fixtures = build()
    stale = []
    for name, fx in fixtures.items():
        text = json.dumps(fx, indent=None, separators=(",", ":")) + "\n"
        target = OUT / f"{name}.json"
        if args.check:
            if not target.exists():
                stale.append(target.name)
                continue
            old = json.loads(target.read_text())
            # A different Octave version is fine as long as every number matches.
            if old.get("oracle") != fx.get("oracle"):
                print(f"note: {target.name} was generated by {old.get('oracle')!r}, now {fx.get('oracle')!r}")
            def canon(d):
                return json.dumps({**d, "oracle": None}, separators=(",", ":"))
            if canon(old) != canon(json.loads(text)):
                stale.append(target.name)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(text)
    total = sum(len(c["inputs"]) for fx in fixtures.values() for c in fx.get("cases", []))
    if args.check and stale:
        print("fixtures differ from a fresh Octave run:", ", ".join(stale))
        return 1
    print(f"{'checked' if args.check else 'wrote'} {len(fixtures)} fixture files; "
          f"{total} stateful chart steps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
