"""Golden parity: Python block ports vs. Octave running the model's own chart code.

Fixtures are produced by ``tools/make_fixtures.py`` (inputs from Python
generators, outputs from Octave only). Discrete outputs must match exactly;
continuous outputs within 1e-12 (observed max error: ~6e-17, from BLAS/FMA
evaluation order in the 2x2 centring products).
"""
from __future__ import annotations

import gzip
import io
from pathlib import Path

import numpy as np
import pytest

from qdrone_autoland import parity

FIX = Path(__file__).parent / "fixtures" / "parity"
TOL = 1e-12

# Output columns that are discrete (flags, counters, indices, reason codes).
DISCRETE = {
    "grid": [4, 5, 6, 7, 8, 11, 12],
    "confirm": [0, 8],
    "gate": [0, 1, 2],
    "abort": [0],
    "arbiter": [0, 1, 2, 3, 4],
    "extract": [6],
    "mscomm": [0],
}


def _load(p: Path) -> np.ndarray:
    if p.suffix == ".gz":
        return np.loadtxt(io.BytesIO(gzip.decompress(p.read_bytes())), delimiter=",", ndmin=2)
    return parity.load_csv(p)


def _pairs():
    for inp in sorted([*FIX.glob("*/*.in.csv"), *FIX.glob("*/*.in.csv.gz")]):
        out = inp.with_name(inp.name.replace(".in.", ".out."))
        yield pytest.param(inp.parent.name, inp, out, id=f"{inp.parent.name}/{inp.name.split('.')[0]}")


def test_fixture_set_is_complete():
    blocks = {p.parent.name for p in FIX.glob("*/*.in.csv*")}
    assert blocks == set(parity.BLOCKS)
    for b in ("grid", "confirm", "gate", "abort", "arbiter"):
        assert (FIX / b / "scn_nominal-autoland.in.csv.gz").exists()


@pytest.mark.parametrize("block,inp,out", list(_pairs()))
def test_block_matches_octave(block, inp, out):
    x, expected = _load(inp), _load(out)
    actual = parity.run_sequence(block, x)
    assert actual.shape == expected.shape
    d = DISCRETE[block]
    np.testing.assert_array_equal(actual[:, d], expected[:, d])
    assert parity.compare(expected, actual) <= TOL


def test_nominal_scenario_fixture_reaches_every_stage():
    out = _load(FIX / "grid" / "scn_nominal-autoland.out.csv.gz")
    assert set(np.unique(out[:, 11])) == {0, 1, 2, 3, 4, 5}
    gate = _load(FIX / "gate" / "scn_nominal-autoland.out.csv.gz")
    assert gate[:, 0].max() == 1                      # landing ready
    arb = _load(FIX / "arbiter" / "scn_nominal-autoland.out.csv.gz")
    assert arb[:, 4].max() == 1 and arb[:, 1].min() == 0   # latched and LAND applied
