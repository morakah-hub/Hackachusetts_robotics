import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armstrong.sim import SimWorker  # noqa: E402


@pytest.fixture
def worker():
    """Fresh simulation worker driven synchronously on the test thread (renderer included)."""
    w = SimWorker(realtime=False)
    w.setup_sync()
    yield w
    if w.camera is not None:
        w.camera.close()
