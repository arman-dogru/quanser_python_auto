from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"


def load_fixture(name: str) -> dict:
    return json.loads((FIXTURES / f"{name}.json").read_text())


def same(a: float, b: float, ulps: int = 0) -> bool:
    """Equal including NaN == NaN; optionally within ``ulps`` units in the last place."""
    a, b = float(a), float(b)
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if a == b:
        return True
    if ulps == 0 or math.isinf(a) or math.isinf(b):
        return False
    return abs(a - b) <= ulps * math.ulp(max(abs(a), abs(b)))


@pytest.fixture(scope="session")
def root() -> Path:
    return ROOT
