"""Detection noise layer (camera.perturb, CameraRig.publish_frame) and the mask orientation of
camera.detections."""
from __future__ import annotations

import copy
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
from test_perception import set_arm

from rebot_sim import camera as cam
from rebot_sim.camera import CameraMount, DetNoise
from rebot_sim.model import load_model
from rebot_sim.perception import CameraRig, SharedState, SimLink

W, H = cam.DETECTION_SIZE


def box(i: int = 0, x: float = 0.5, y: float = 0.5, w: float = 0.1, h: float = 0.15) -> dict:
    return {"class_id": i, "format": "xywh_center", "class_name": "cube", "name": f"cube_{i}", "confidence": 0.99,
            "x": x, "y": y, "w": w, "h": h, "pixels": 500, "angle_deg": 10.0, "angle_conf": 0.8}


def test_default_noise_returns_the_detections_untouched():
    dets = [box(0), box(1, 0.2, 0.3)]
    before = copy.deepcopy(dets)
    assert not DetNoise().perturbs
    assert cam.perturb(dets, DetNoise(), 7) is dets and dets == before
    assert cam.perturb(dets, DetNoise(latency_ms=100.0, seed=3), 7) is dets  # latency is not a perturbation


def test_same_seed_and_frame_give_the_same_output():
    cfg = DetNoise(jitter_px=3, size_jitter=0.05, drop_rate=0.2, false_rate=0.5, angle_jitter_deg=5, seed=1)
    dets = [box(0), box(1, 0.2, 0.3)]
    a, b = cam.perturb(dets, cfg, 11), cam.perturb(dets, cfg, 11)
    assert a == b and a != dets
    assert any(cam.perturb(dets, cfg, f) != a for f in range(12, 20))
    assert any(cam.perturb(dets, DetNoise(**{**cfg.__dict__, "seed": 2}), f) != cam.perturb(dets, cfg, f) for f in range(11, 20))


def test_centre_and_size_jitter_match_their_sigma():
    n = 4000
    cfg = DetNoise(jitter_px=3.0, size_jitter=0.05, seed=5)
    outs = [cam.perturb([box()], cfg, f)[0] for f in range(n)]
    dx = np.array([o["x"] - 0.5 for o in outs]) * W
    dy = np.array([o["y"] - 0.5 for o in outs]) * H
    rw = np.array([o["w"] for o in outs]) / 0.1 - 1
    rh = np.array([o["h"] for o in outs]) / 0.15 - 1
    assert abs(dx.mean()) < 0.3 and abs(dy.mean()) < 0.3
    assert dx.std() == pytest.approx(3.0, rel=0.1) and dy.std() == pytest.approx(3.0, rel=0.1)
    assert rw.std() == pytest.approx(0.05, rel=0.1) and rh.std() == pytest.approx(0.05, rel=0.1)
    assert abs(np.corrcoef(rw, rh)[0, 1]) < 0.1  # independent


def test_drop_rate_and_false_detections():
    n = 4000
    dropped = sum(not cam.perturb([box()], DetNoise(drop_rate=0.1, seed=2), f) for f in range(n))
    assert dropped / n == pytest.approx(0.1, abs=0.02)
    outs = [cam.perturb([box()], DetNoise(false_rate=0.25, seed=2), f) for f in range(n)]
    extra = [o[1] for o in outs if len(o) == 2]
    assert len(extra) / n == pytest.approx(0.25, abs=0.03)
    assert all(0.3 <= d["confidence"] <= 0.6 and d["class_name"] == "cube" for d in extra)
    assert all(-1e-4 <= d["x"] - d["w"] / 2 and d["x"] + d["w"] / 2 <= 1.0 + 1e-4 for d in extra)
    assert cam.perturb([], DetNoise(false_rate=1.0), 1, labels=[(2, "cup")])[0]["class_name"] == "cup"
    assert cam.perturb([], DetNoise(false_rate=1.0), 1) == []  # no label to draw from


def test_boxes_are_clipped_and_degenerate_ones_dropped():
    cfg = DetNoise(jitter_px=40.0, seed=1)
    for f in range(300):
        for d in cam.perturb([box(w=0.2, h=0.3)], cfg, f):
            assert d["x"] - d["w"] / 2 >= -1e-4 and d["x"] + d["w"] / 2 <= 1 + 1e-4
            assert d["y"] - d["h"] / 2 >= -1e-4 and d["y"] + d["h"] / 2 <= 1 + 1e-4
            assert d["w"] * W >= 1.9 and d["h"] * H >= 1.9
    assert cam.perturb([box(x=0.5, w=0.001)], DetNoise(jitter_px=1e-9), 1) == []


def test_angle_jitter_wraps_into_the_range():
    cfg = DetNoise(angle_jitter_deg=30.0, seed=4)
    angles = np.array([cam.perturb([dict(box(), angle_deg=85.0)], cfg, f)[0]["angle_deg"] for f in range(2000)])
    assert angles.min() >= -90.0 and angles.max() < 90.0
    assert (angles < 0).any() and (angles > 80).any()


def test_latency_delays_by_the_right_number_of_frames():
    got = []
    rig = SimpleNamespace(noise=DetNoise(latency_ms=100.0), delayed=[], perception=SimpleNamespace(on_frame=lambda *a: got.append(a)))
    # frames captured 40 ms apart, each published when the next is rendered (as run() does)
    for k in range(6):
        CameraRig.publish_frame(rig, (k, 0, time.monotonic() - 0.04 * (5 - k), [], 0.0, k))
    # frame k was captured 40 * (5 - k) ms before the call it was published in; published once >= 100 ms old
    assert [g[0] for g in got] == [0, 1, 2] and all(time.monotonic() - g[2] >= 0.1 for g in got)
    assert [f[0] for f in rig.delayed] == [3, 4, 5]
    off = SimpleNamespace(noise=DetNoise(), delayed=[], perception=SimpleNamespace(on_frame=lambda *a: got.append(a)))
    CameraRig.publish_frame(off, (9, 0, time.monotonic(), [], 0.0, 9))
    assert got[-1][0] == 9 and not off.delayed


def _scene(masks):
    return SimpleNamespace(objects=[SimpleNamespace(kind="block", name=f"block_{i}") for i in range(len(masks))],
                           geom_ids=list(range(1, len(masks) + 1)))


def _rect_mask(angle_deg: float, length: float = 40.0, width: float = 14.0) -> np.ndarray:
    yy, xx = np.mgrid[0:H, 0:W]
    a = math.radians(angle_deg)
    u = (xx - 320) * math.cos(a) + (yy - 180) * math.sin(a)
    v = -(xx - 320) * math.sin(a) + (yy - 180) * math.cos(a)
    return (abs(u) <= length / 2) & (abs(v) <= width / 2)


@pytest.mark.parametrize("angle", [-80, -45, 0, 30, 60, 89])
def test_mask_angle_is_measured_from_x_toward_y_down(angle):
    ids = np.zeros((H, W), np.int32)
    ids[_rect_mask(angle)] = 1
    d = cam.detections(ids, _scene([1]))[0]
    err = (d["angle_deg"] - angle + 90) % 180 - 90
    assert abs(err) < 2.0, d
    assert -90 <= d["angle_deg"] < 90 and d["angle_conf"] > 0.6


def test_square_mask_has_no_angle_confidence():
    ids = np.zeros((H, W), np.int32)
    ids[_rect_mask(30, 30, 30)] = 1
    assert cam.detections(ids, _scene([1]))[0]["angle_conf"] < 0.3


def _block_angle(rig, yaw_deg: float) -> dict:
    qa = rig.scene.qpos[0]
    y = math.radians(yaw_deg)
    rig.data.qpos[qa + 3:qa + 7] = [math.cos(y / 2), 0, 0, math.sin(y / 2)]
    set_arm(rig.model, rig.data, [0.46, -0.63, -1.05, 1.48, 0.43, -2.2307963])  # J6 -0.66 - pi/2 (official J6 convention)
    dets = rig.detect()
    assert dets, "block not in view"
    return dets[0]


def test_rendered_block_angle_follows_its_yaw():
    m = CameraMount()
    cfg = {"fps": 15, "size": [640, 360], "mount": [m.flip, m.rot_deg, m.tilt_deg, m.offset_m],
           "objects": "block", "scene_stream": False, "has_meshes": False}
    st = SharedState(load_model(devarm_dir="", camera=m).nq, load_model(devarm_dir="", camera=m).nv)
    rig = CameraRig(cfg, st, SimLink(None))
    try:
        try:
            rig.setup()
        except Exception as exc:  # no GL backend
            pytest.skip(repr(exc))
        base = _block_angle(rig, 0.0)
        assert base["angle_conf"] > 0.3
        for yaw in (30.0, -30.0, 60.0):
            d = _block_angle(rig, yaw)
            delta = (d["angle_deg"] - base["angle_deg"] + 90) % 180 - 90
            # the camera looks (nearly) straight down, mirrored against the base frame: the image angle turns by -yaw
            assert abs(delta + yaw) < 5.0, (yaw, base, d)
    finally:
        st.close()
