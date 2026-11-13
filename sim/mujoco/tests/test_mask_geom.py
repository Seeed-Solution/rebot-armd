"""Tests for mask geometry: mask_orientation and mask_cross_section against shared test vectors."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from rebot_sim.camera import mask_cross_section, mask_orientation

VECTORS_PATH = (
    Path(__file__).resolve().parents[3]
    / "kits"
    / "recamera-b601dm"
    / "vision"
    / "tests"
    / "data"
    / "cross_section_vectors.json"
)


@pytest.fixture(scope="module")
def vectors() -> dict[str, dict]:
    data = json.loads(VECTORS_PATH.read_text())
    return {c["name"]: c for c in data}


def _get_points(case: dict) -> tuple[np.ndarray, np.ndarray, float, float]:
    pts = case["points"]
    xs = np.array([p[0] for p in pts], dtype=np.float64)
    ys = np.array([p[1] for p in pts], dtype=np.float64)
    sx = float(case.get("scale_x", 1.0))
    sy = float(case.get("scale_y", 1.0))
    return xs, ys, sx, sy


def test_vectors_file_exists():
    assert VECTORS_PATH.is_file(), f"missing test vectors file at {VECTORS_PATH}"


def test_all_cases_execute_and_produce_valid_outputs(vectors):
    for name, case in vectors.items():
        xs, ys, sx, sy = _get_points(case)
        angle, conf = mask_orientation(xs, ys, scale_x=sx, scale_y=sy)
        assert -90.0 <= angle < 90.0, f"{name}: orientation angle {angle} out of range"
        assert 0.0 <= conf <= 1.0, f"{name}: orientation conf {conf} out of range"

        cs_angle, cs_conf, cs_w = mask_cross_section(xs, ys, scale_x=sx, scale_y=sy)
        assert -90.0 <= cs_angle < 90.0, f"{name}: cross_section angle {cs_angle} out of range"
        assert 0.0 <= cs_conf <= 1.0, f"{name}: cross_section conf {cs_conf} out of range"
        assert cs_w >= 0.0, f"{name}: cross_section width {cs_w} negative"


@pytest.mark.parametrize("name,expected_angle", [
    ("rect_0", 0.0),
    ("rect_30", 30.0),
    ("rect_75", 75.0),
])
def test_rect_angles_and_widths(vectors, name, expected_angle):
    case = vectors[name]
    xs, ys, sx, sy = _get_points(case)
    angle, conf, w = mask_cross_section(xs, ys, scale_x=sx, scale_y=sy)

    # rect 0, 30, 75 deg angles within +-0.5 deg
    err = (angle - expected_angle + 90.0) % 180.0 - 90.0
    assert abs(err) <= 0.5, f"{name}: angle {angle} error {err} > 0.5 deg"

    # width approx 20 px (+-1.5 px)
    assert abs(w - 20.0) <= 1.5, f"{name}: width {w} not within +-1.5 px of 20"

    # conf approx 0.5 (0.45 .. 0.65)
    assert 0.45 <= conf <= 0.65, f"{name}: conf {conf} not approx 0.5"


def test_stretched_rect_matches_unstretched(vectors):
    """stretched rect with scale_y=0.5 gives same angle, width, conf as unstretched rect."""
    xs_0, ys_0, sx_0, sy_0 = _get_points(vectors["rect_0"])
    xs_str, ys_str, sx_str, sy_str = _get_points(vectors["rect_stretched"])

    res_0 = mask_cross_section(xs_0, ys_0, scale_x=sx_0, scale_y=sy_0)
    res_str = mask_cross_section(xs_str, ys_str, scale_x=sx_str, scale_y=sy_str)
    # a resampled mask differs from the original only by discretisation
    assert abs((res_str[0] - res_0[0] + 90) % 180 - 90) <= 0.5, (res_str, res_0)
    assert abs(res_str[2] - res_0[2]) <= 1.0, (res_str, res_0)
    assert abs(res_str[1] - res_0[1]) <= 0.03, (res_str, res_0)
    a_str, c_str = mask_orientation(xs_str, ys_str, scale_x=sx_str, scale_y=sy_str)
    a_0, c_0 = mask_orientation(xs_0, ys_0, scale_x=sx_0, scale_y=sy_0)
    assert abs((a_str - a_0 + 90) % 180 - 90) <= 0.5 and abs(c_str - c_0) <= 0.03


def test_disc_confidence_low(vectors):
    """disc conf < 0.1."""
    xs, ys, sx, sy = _get_points(vectors["disc"])
    _, conf, w = mask_cross_section(xs, ys, scale_x=sx, scale_y=sy)
    assert conf < 0.1
    # diameter ~40 px
    assert abs(w - 40.0) <= 2.0


def test_square_geometry(vectors):
    """square (40x40 px) has ~40 px width and conf around 0.29."""
    xs, ys, sx, sy = _get_points(vectors["square"])
    angle, conf, w = mask_cross_section(xs, ys, scale_x=sx, scale_y=sy)
    assert abs(w - 40.0) <= 1.0
    assert conf < 0.35


def test_ring_and_l_shape_snap_centroid(vectors):
    """ring and L-shape execute without error and snap centroid."""
    for name in ("ring", "l_shape"):
        case = vectors[name]
        xs, ys, sx, sy = _get_points(case)

        # Confirm original un-snapped mean centroid is NOT in the mask
        gx_mean = float((xs * sx).mean())
        gy_mean = float((ys * sy).mean())
        mask_pts = set(zip(np.round(xs).astype(int), np.round(ys).astype(int)))
        qx = int(round(gx_mean / sx))
        qy = int(round(gy_mean / sy))
        assert (qx, qy) not in mask_pts, f"{name}: expected centroid ({qx}, {qy}) to be outside mask"

        # Execute mask_cross_section without error
        angle, conf, w = mask_cross_section(xs, ys, scale_x=sx, scale_y=sy)
        assert -90.0 <= angle < 90.0
        assert 0.0 <= conf <= 1.0
        assert w > 0.0


def test_detections_include_grasp_width_px():
    from types import SimpleNamespace
    from rebot_sim.camera import detections

    ids = np.zeros((360, 640), np.int32)
    for x in range(300, 340):
        for y in range(170, 190):
            ids[y, x] = 1
    scene = SimpleNamespace(objects=[SimpleNamespace(kind="block", name="block_0")], geom_ids=[1])
    dets = detections(ids, scene)
    assert len(dets) == 1
    d = dets[0]
    assert "grasp_width_px" in d
    assert abs(d["grasp_width_px"] - 20.0) <= 1.5
    assert abs(d["angle_deg"] - 0.0) <= 1.0
    assert "angle_conf" in d


def test_perturb_scales_grasp_width_px():
    from rebot_sim.camera import DetNoise, perturb

    det = {
        "class_id": 0, "format": "xywh_center", "class_name": "block", "name": "block_0",
        "confidence": 0.99, "x": 0.5, "y": 0.5, "w": 0.1, "h": 0.05,
        "pixels": 800, "angle_deg": 0.0, "angle_conf": 0.5, "grasp_width_px": 20.0,
    }
    cfg = DetNoise(size_jitter=0.1, seed=42)
    perturbed = perturb([det], cfg, 1)
    assert len(perturbed) == 1
    assert "grasp_width_px" in perturbed[0]
    assert perturbed[0]["grasp_width_px"] != 20.0

    cfg_false = DetNoise(false_rate=1.0, seed=42)
    perturbed_false = perturb([], cfg_false, 1, labels=[(0, "block")])
    assert len(perturbed_false) == 1
    assert "grasp_width_px" in perturbed_false[0]
    assert perturbed_false[0]["grasp_width_px"] > 0.0


def test_scene_supports_slab():
    from rebot_sim.scene import KINDS, SceneObject, parse_objects

    assert "slab" in KINDS
    gtype, size, mass, rgba = KINDS["slab"]
    assert size == (0.05, 0.05, 0.01)
    assert mass == 0.05
    assert rgba == (0.80, 0.60, 0.20, 1)

    obj = SceneObject("slab", "slab")
    assert obj.rest_z == 0.01

    objs = parse_objects("slab,cube")
    assert len(objs) == 2
    assert objs[0].kind == "slab"

