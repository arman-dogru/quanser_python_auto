"""Live Octave checks: the original MATLAB tests, template equality, fixture freshness."""
from __future__ import annotations

import subprocess
import sys

import pytest

pytestmark = pytest.mark.octave


def _octave(root, expr: str) -> str:
    r = subprocess.run(["octave-cli", "--no-gui", "--quiet", "--eval", expr], cwd=root, capture_output=True,
                       text=True, timeout=600)
    assert r.returncode == 0, r.stdout + r.stderr
    return r.stdout


def test_original_matlab_tests_pass_in_octave(root):
    out = _octave(root, f"addpath('{root / 'tools' / 'octave'}'); run_original_tests('{root}')")
    assert "ALL ORIGINAL MATLAB TESTS PASSED IN OCTAVE" in out
    assert out.count("PASS:") >= 10


def test_model_charts_equal_templates(root):
    out = _octave(root, f"addpath('{root / 'tools' / 'octave'}'); check_templates('{root}')")
    assert "FAIL" not in out


def _octave_version() -> str:
    r = subprocess.run(["octave-cli", "--version"], capture_output=True, text=True)
    return r.stdout.split("\n")[0].split()[-1]


def test_python_matches_this_octave(root, tmp_path):
    """The oracle property itself, on whatever Octave is installed: regenerate
    every committed fixture's outputs with it and compare the Python port."""
    import numpy as np
    from test_parity import DISCRETE, _load

    from qdrone_autoland import parity
    out = tmp_path / "fx"
    r = subprocess.run([sys.executable, str(root / "tools" / "make_fixtures.py"), "--out", str(out)], cwd=root,
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout + r.stderr
    n = 0
    for inp in sorted([*out.glob("*/*.in.csv"), *out.glob("*/*.in.csv.gz")]):
        block = inp.parent.name
        x, e = _load(inp), _load(inp.with_name(inp.name.replace(".in.", ".out.")))
        a = parity.run_sequence(block, x)
        np.testing.assert_array_equal(a[:, DISCRETE[block]], e[:, DISCRETE[block]], err_msg=f"{block}/{inp.name}")
        assert parity.compare(e, a) <= 1e-12, f"{block}/{inp.name}"
        n += len(x)
    assert n > 40000


def test_fixtures_regenerate_from_octave(root):
    """Regenerate every committed fixture with this Octave and compare.

    The committed outputs were produced with Octave 9.4, where they must
    regenerate byte for byte. Another Octave/BLAS build may round the 2x2
    centring products in the last bit; there the check still runs and
    allows at most 1e-12 (anything larger fails)."""
    r = subprocess.run([sys.executable, str(root / "tools" / "make_fixtures.py"), "--check"], cwd=root,
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout + r.stderr
    if _octave_version().startswith("9."):
        assert "byte-identically" in r.stdout, r.stdout
    print(_octave_version(), r.stdout.strip().splitlines()[-1])
