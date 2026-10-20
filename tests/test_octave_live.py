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


def test_fixtures_regenerate_byte_identically(root):
    r = subprocess.run([sys.executable, str(root / "tools" / "make_fixtures.py"), "--check"], cwd=root,
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout + r.stderr
