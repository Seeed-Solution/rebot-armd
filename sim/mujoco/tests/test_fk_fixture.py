"""tests/fixtures/mujoco_fk_samples.json (read by armd's tests/test_sim_fk.rs) must match the scene."""
import json
from pathlib import Path

import numpy as np

from rebot_sim.fk import fk_samples

FIXTURE = Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "mujoco_fk_samples.json"


def test_fixture_is_current():
    stored = json.loads(FIXTURE.read_text())["samples"]
    fresh = fk_samples()
    assert [s["q"] for s in stored] == [s["q"] for s in fresh]
    for a, b in zip(stored, fresh):
        assert np.allclose(a["p"], b["p"], atol=1e-6)
        assert np.allclose(a["r"], b["r"], atol=1e-6)
