from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
# tools/mutation_sweep.py points QDRONE_AUTOLAND_SRC at a mutated copy.
SRC = Path(os.environ.get("QDRONE_AUTOLAND_SRC", ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(SRC))
import qdrone_autoland  # noqa: E402

if not Path(qdrone_autoland.__file__).resolve().is_relative_to(SRC.resolve()):
    raise RuntimeError(f"qdrone_autoland imported from {qdrone_autoland.__file__}, expected under {SRC}")

HAVE_OCTAVE = shutil.which("octave-cli") is not None


@pytest.fixture(scope="session")
def root() -> Path:
    return ROOT


def pytest_collection_modifyitems(config, items):
    skip_oct = pytest.mark.skip(reason="octave-cli not on PATH")
    for item in items:
        if "octave" in item.keywords and not HAVE_OCTAVE:
            item.add_marker(skip_oct)
