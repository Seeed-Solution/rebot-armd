"""MuJoCo end_link poses for fixed joint vectors (fixture for armd's kinematics test).

`uv run rebot-sim-fk > ../../tests/fixtures/mujoco_fk_samples.json`
"""
from __future__ import annotations

import json

import mujoco
import numpy as np

from .model import Indexer, load_model

SAMPLES = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963],
    [0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    [0.3, 0.8, 1.2, -0.4, 0.5, 1.0],
    [-1.2, 1.4, 0.6, 0.9, -1.1, -2.0],
    [2.5, -0.5, -1.5, 2.0, 1.8, 4.0],
    [0.7, 0.35, 1.9, -1.0, -0.6, 0.2],
]


def fk_samples() -> list[dict]:
    model = load_model(devarm_dir="")
    data = mujoco.MjData(model)
    idx = Indexer(model)
    out = []
    for q in SAMPLES:
        idx.set_motor_positions(data, q)
        mujoco.mj_kinematics(model, data)
        body = data.body("end_link")
        out.append({"q": q, "p": [round(float(v), 7) for v in body.xpos],
                    "r": np.round(body.xmat.reshape(3, 3), 7).tolist()})
    return out


def main() -> None:
    print(json.dumps({"source": "sim/mujoco/models/rebot_b601dm_scene.xml (MuJoCo %s)" % mujoco.__version__,
                      "frame": "end_link in base_link", "samples": fk_samples()}, indent=1))
