"""Row adapters between parity fixtures and the Python block ports.

Column layouts are the ones ``tools/octave/parity_driver.m`` uses: one row per
model tick, block inputs in port order (flattened), outputs in port order.
Every sequence starts from a freshly reset block, as a model start would.
"""
from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from pathlib import Path

import numpy as np

from .config import DEFAULT, Config
from .logic.abort_diagnostic import AbortDiagnostic
from .logic.arbiter import AutolandArbiter
from .logic.comm_loss import MissionCommLoss
from .logic.extract_object import extract_object_data
from .logic.grid_manager import GridSearchManager
from .logic.landing_gate import LandingReadinessGate
from .logic.yolo_confirmation import YoloConfirmation

Row = Sequence[float]


def _grid(cfg: Config):
    blk = GridSearchManager(cfg)

    def f(x: Row):
        o = blk.step(x[0:4], x[4:8], x[8:12], x[12] != 0, x[13:17], x[17], x[18], x[19] != 0, x[20])
        return [*o.desired_pose, o.waypoint_index, o.grid_complete, o.visited_count, o.centering_active,
                o.centered, *o.pixel_error, o.descent_stage, o.descent_abort]
    return f


def _confirm(cfg: Config):
    m, a = cfg.model, cfg.auto
    blk = YoloConfirmation(a.yolo_confidence, a.yolo_confirm_frames, a.yolo_release_frames,
                           a.yolo_stale_seconds, m.image_width, m.image_height)

    def f(x: Row):
        o = blk.step(x[0:4], x[4:10], x[10] != 0, x[11], x[12:16])
        return [o.confirmed_pad, *o.hold_pose, o.confidence, o.pad_u, o.pad_v, o.new_pad_sample, o.vision_age]
    return f


def _gate(cfg: Config):
    blk = LandingReadinessGate()

    def f(x: Row):
        o = blk.step(x[0], x[1] != 0, x[2] != 0, x[3] != 0, x[4] != 0, x[5], x[6:10], x[10:14], x[14])
        return [o.landing_ready, o.fresh_count, o.reason_code]
    return f


def _abort(cfg: Config):
    blk = AbortDiagnostic()

    def f(x: Row):
        o = blk.step(x[0], x[1] != 0, x[2] != 0, x[3], x[4:8], x[8], x[9])
        return [o.cause_bits, o.age_at_abort, o.time_at_abort]
    return f


def _arbiter(cfg: Config):
    blk = AutolandArbiter()

    def f(x: Row):
        o = blk.step(x[0:4], x[4] != 0)
        return [*o.commander_triggers, o.autoland_latched]
    return f


def _extract(cfg: Config):
    def f(x: Row):
        p, o, t = extract_object_data(x[0:7], x[7])
        return [*p, *o, t]
    return f


def _mscomm(cfg: Config):
    blk = MissionCommLoss()

    def f(x: Row):
        return [blk.step(x[0], x[1], x[2])]
    return f


BLOCKS: dict[str, Callable[[Config], Callable[[Row], list]]] = {
    "grid": _grid, "confirm": _confirm, "gate": _gate, "abort": _abort, "arbiter": _arbiter,
    "extract": _extract, "mscomm": _mscomm,
}


def run_sequence(block: str, rows: Iterable[Row], cfg: Config = DEFAULT) -> np.ndarray:
    f = BLOCKS[block](cfg)
    return np.array([[float(v) for v in f([float(c) for c in r])] for r in rows], dtype=float)


def load_csv(path: Path) -> np.ndarray:
    return np.loadtxt(path, delimiter=",", ndmin=2)


def compare(expected: np.ndarray, actual: np.ndarray) -> float:
    """Max abs error, treating NaN==NaN as equal and a NaN mismatch as inf."""
    if expected.shape != actual.shape:
        return float("inf")
    en, an = np.isnan(expected), np.isnan(actual)
    if (en != an).any():
        return float("inf")
    if (np.isinf(expected) | np.isinf(actual)).any():
        mask = np.isinf(expected) | np.isinf(actual)
        if (expected[mask] != actual[mask]).any():
            return float("inf")
    ok = ~(en | np.isinf(expected))
    if not ok.any():
        return 0.0
    return float(np.max(np.abs(expected[ok] - actual[ok])))


def fixture_pairs(root: Path):
    for inp in sorted(root.glob("*/*.in.csv")):
        yield inp.parent.name, inp, inp.with_name(inp.name.replace(".in.csv", ".out.csv"))
