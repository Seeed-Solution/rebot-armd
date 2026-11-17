"""Emulated depth pipeline (camera.DepthModel / depth_result, perception `depth`): the rendered
depth against the scene geometry, the disparity models against armd's calibration equations
(src/grasp.rs calibrate_depth / measure_detection_depth_with_cal), and the perception API
contract with depth on and off."""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request

import mujoco
import numpy as np
import pytest
from test_perception import FakeSim, set_arm

from rebot_sim import camera as cam
from rebot_sim.camera import CameraMount, DepthModel
from rebot_sim.model import load_model
from rebot_sim.perception import CameraRig, Perception, PerceptionServer, SharedState, SimLink

# J1..J6 (armd motor angles) with the default mount's optical axis within 0.7 deg of straight
# down, camera at about (0.236, 0.066, 0.314) m
DOWN_Q = [0.0, -1.5273, -1.7182, 1.75, 0.0, 0.0]
OBLIQUE_Q = [0.46, -0.63, -1.05, 1.48, 0.43, -0.66]
W, H = cam.DETECTION_SIZE
F = cam.focal_px(H)
FX_PER_W = F / W


def http(url: str, body: dict | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _rig(depth: dict | None = None, objects: str = "cube"):
    m = CameraMount()
    model = load_model(devarm_dir="", camera=m)
    st = SharedState(model.nq, model.nv)
    cfg = {"fps": 15, "size": [640, 360], "mount": [m.flip, m.rot_deg, m.tilt_deg, m.offset_m],
           "objects": objects, "scene_stream": False, "has_meshes": False, "depth": depth or {}}
    return CameraRig(cfg, st, SimLink(None)), st


def _cv_rotation(data: mujoco.MjData, cid: int) -> np.ndarray:
    """World rotation of the OpenCV camera frame (x right, y down, z forward) of MuJoCo camera cid."""
    r = data.cam_xmat[cid].reshape(3, 3)
    return np.column_stack([r[:, 0], -r[:, 1], -r[:, 2]])


def table_z(data: mujoco.MjData, cid: int, u, v) -> np.ndarray:
    """Optical depth Z where the rays through content pixels (u, v) meet the table (z = 0)."""
    ray = _cv_rotation(data, cid) @ np.vstack([(np.asarray(u) - W / 2) / F, (np.asarray(v) - H / 2) / F, np.ones(np.size(u))])
    return -data.cam_xpos[cid][2] / ray[2]


def ray_z(data: mujoco.MjData, cid: int, u: float, v: float) -> float:
    """z component of armd's ray_world_per_z through (u, v)."""
    return float((_cv_rotation(data, cid) @ np.array([(u - W / 2) / F, (v - H / 2) / F, 1.0]))[2])


def _pose(rig, q, cube_xy=None):
    """Arm at q; the cube under the optical axis (cube_xy "axis"), at cube_xy, or out of view."""
    set_arm(rig.model, rig.data, q)
    cid = rig.model.camera("wrist").id
    qa = rig.model.jnt_qposadr[rig.model.joint("cube").id]
    if cube_xy == "axis":
        p, d = rig.data.cam_xpos[cid], -rig.data.cam_xmat[cid].reshape(3, 3)[:, 2]
        t = (0.04 - p[2]) / d[2]
        cube_xy = (p[0] + d[0] * t, p[1] + d[1] * t)
    xy = cube_xy if cube_xy is not None else (-0.5, -0.5)
    rig.data.qpos[qa:qa + 7] = [xy[0], xy[1], 0.02, 1, 0, 0, 0]
    mujoco.mj_forward(rig.model, rig.data)
    return cid


def _render(rig):
    r = rig.seg_renderer
    r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
    ids = cam.render_segmentation(r)
    return ids, cam.render_depth(r)


@pytest.fixture
def rig():
    rig, st = _rig({"kind": "metric"})
    rig.setup()
    yield rig
    st.close()


def test_metric_model_matches_armds_metric_prior():
    z = np.array([0.1, 0.2, 0.5])
    a_prior = (600.0 / 640.0) / FX_PER_W  # src/grasp.rs calibrate_depth
    for s in (1.0, 1.5):
        d = DepthModel("metric", scale=s).disparity(z)
        assert np.allclose(1.0 / (a_prior * d), z / s)
    rel = DepthModel("relative", a=2.7, b=0.45)
    assert np.allclose(rel.disparity(z), 2.7 / z + 0.45)
    assert np.allclose(rel.inverse_z(rel.disparity(z)), 1 / z)
    assert np.isnan(DepthModel("metric").disparity(np.array([np.inf, 0.0, -1.0]))).all()
    noisy = DepthModel("metric", noise_rel=0.02, seed=4)
    assert np.array_equal(noisy.disparity(np.full(50, 0.2), 3), noisy.disparity(np.full(50, 0.2), 3))
    assert not np.array_equal(noisy.disparity(np.full(50, 0.2), 3), noisy.disparity(np.full(50, 0.2), 4))


@pytest.mark.parametrize("q", [DOWN_Q, OBLIQUE_Q], ids=["down", "oblique"])
def test_rendered_depth_is_finite_optical_axis_z(rig, q):
    """Renderer depth = Z along the optical axis (not the range along the ray) on the table."""
    cid = _pose(rig, q)
    ids, z = _render(rig)
    assert z.shape == (H, W) and np.isfinite(z).all() and (z > 0).all()
    ys, xs = np.nonzero(ids == rig.scene.table_id)
    assert len(xs) > 1000
    sel = np.linspace(0, len(xs) - 1, 2000).astype(int)
    want = table_z(rig.data, cid, xs[sel] + 0.5, ys[sel] + 0.5)
    err = np.abs(z[ys[sel], xs[sel]] - want) / want
    assert np.percentile(err, 99) < 2e-3, (err.max(), np.percentile(err, 99))


def test_metric_table_samples_on_a_flat_table_seen_straight_down(rig):
    cid = _pose(rig, DOWN_Q)
    ids, z = _render(rig)
    d = cam.depth_result(ids, z, rig.scene, DepthModel("metric"))
    s = np.array(d["table_samples"])
    assert d["finite"] and d["objects"] == {} and len(s) >= 30, len(s)
    want = DepthModel("metric").disparity(table_z(rig.data, cid, s[:, 0], s[:, 1]))
    assert np.abs(s[:, 2] / want - 1).max() < 0.01
    print(f"\nstraight down: {len(s)} samples, max disparity error {np.abs(s[:, 2] / want - 1).max() * 100:.3f} %")
    zs = 1 / DepthModel("metric").inverse_z(s[:, 2])
    assert zs.max() - zs.min() < 0.01  # straight down: one optical depth over the frame


def test_samples_follow_the_relative_model(rig):
    cid = _pose(rig, OBLIQUE_Q)
    ids, z = _render(rig)
    m = DepthModel("relative", a=2.7, b=0.45)
    s = np.array(cam.depth_result(ids, z, rig.scene, m)["table_samples"])
    assert len(s) >= 8
    inv_z = 1 / table_z(rig.data, cid, s[:, 0], s[:, 1])
    assert np.abs(m.inverse_z(s[:, 2]) / inv_z - 1).max() < 0.01
    print(f"\noblique pose: {len(s)} samples, inverse-Z spread {inv_z.max() - inv_z.min():.3f} 1/m")


@pytest.mark.parametrize("kind", ["metric", "relative"])
def test_object_height_from_disp_fg_and_disp_bg(rig, kind):
    """armd's relation h = (Z_bg - Z_fg) |ray_world_per_z.z| at the box centre gives the cube's
    4 cm within 2 mm (noise-free model, generator inverse as the calibration)."""
    cid = _pose(rig, DOWN_Q, "axis")
    ids, z = _render(rig)
    m = DepthModel(kind)
    d = cam.depth_result(ids, z, rig.scene, m)
    det = next(x for x in cam.detections(ids, rig.scene) if x["name"] == "cube")
    f = d["objects"]["cube"]
    assert f["disp_fg"] > f["disp_bg"] and f["depth_conf"] == 0.95 and f["jaw_clear"] is True
    z_fg, z_bg = 1 / m.inverse_z(f["disp_fg"]), 1 / m.inverse_z(f["disp_bg"])
    h = (z_bg - z_fg) * abs(ray_z(rig.data, cid, det["x"] * W, det["y"] * H))
    print(f"\n{kind}: h {h * 1000:.2f} mm (truth 40 mm), {len(d['table_samples'])} table samples")
    assert abs(h - 0.04) <= 0.002, h
    # length: the cube's 4 cm side at the top's depth, in pixels
    assert abs(f["length_px"] * z_fg / F - 0.04) < 0.006, f["length_px"] * z_fg / F
    assert len(d["table_samples"]) >= 3



def test_depth_angle_and_width_from_the_mask_cross_section(rig):
    """depth_angle_deg / depth_width_px are the mask's cross section (as angle_deg / grasp_width_px),
    depth_angle_conf = angle_conf x depth_conf."""
    _pose(rig, DOWN_Q, "axis")
    ids, z = _render(rig)
    d = cam.depth_result(ids, z, rig.scene, DepthModel("metric"))
    det = next(x for x in cam.detections(ids, rig.scene) if x["name"] == "cube")
    f = d["objects"]["cube"]
    assert f["depth_angle_deg"] == det["angle_deg"] and f["depth_width_px"] == det["grasp_width_px"], (f, det)
    assert f["depth_angle_conf"] == round(det["angle_conf"] * 0.95, 3), (f, det)

def test_detect_renders_depth_only_when_wanted_on_the_same_physics_step(rig):
    _pose(rig, DOWN_Q, "axis")
    assert rig.detect() is not None and rig.frame_depth is None
    dets = rig.detect(want_depth=True, physics_step=7)
    assert dets and rig.frame_depth["physics_step"] == 7 and "cube" in rig.frame_depth["objects"]
    p = rig.perception
    out = p.result({"frame_id": 1, "ts_ms": 0, "mono": 0.0, "physics_time": 0.0, "physics_step": 7,
                    "detections": dets, "depth": rig.frame_depth}, ["yolo", "depth"])
    assert out["pipelines"]["depth"]["ran"] and "disp_fg" in out["detections"][0]
    stale = p.result({"frame_id": 1, "ts_ms": 0, "mono": 0.0, "physics_time": 0.0, "physics_step": 8,
                      "detections": dets, "depth": rig.frame_depth}, ["yolo", "depth"])
    assert not stale["pipelines"]["depth"]["ran"] and "depth" not in stale and "disp_fg" not in stale["detections"][0]


# ---------------------------------------------------------------- perception API contract

DEPTH = {"table_samples": [[40.0, 20.0, 5.0], [320.0, 180.0, 5.1], [600.0, 340.0, 4.9]], "physics_step": 0,
         "objects": {"cube": {"disp_fg": 6.2, "disp_bg": 5.0, "depth_conf": 0.95, "length_px": 50.0, "jaw_clear": True}}}
DETS = [{"class_name": "cube", "name": "cube", "x": 0.5, "y": 0.5, "w": 0.1, "h": 0.1},
        {"class_name": "cube", "name": "false_cube", "x": 0.2, "y": 0.2, "w": 0.1, "h": 0.1}]


def _api(depth):
    srv = PerceptionServer(Perception(max_fps=15, depth=depth), FakeSim(), "127.0.0.1", 0)
    return srv, f"http://127.0.0.1:{srv.httpd.server_address[1]}"


@pytest.fixture
def api_depth():
    srv, base = _api(DepthModel("metric"))
    yield srv, base
    srv.httpd.shutdown()
    srv.httpd.server_close()


def _trigger(base, body, out, key):
    out[key] = http(base + "/api/trigger", body)[1]


def _frame(p, fid, depth=DEPTH):
    p.on_frame(fid, 1000 + fid, time.monotonic(), DETS, 0.0, 0, depth)


def test_depth_off_keeps_mode_and_trigger_bodies():
    """Without a depth model (default, or kind off) /api/mode and trigger bodies are those of
    the yolo / aruco API: no depth keys anywhere, depth rejected as a pipeline."""
    mode_keys = {"api_version", "mode", "modes", "pipelines", "last_active_mode", "enabled", "processing", "fps",
                 "active_fps", "idle_fps", "fps_range", "camera_fps", "measured", "trigger", "per_request_pipelines",
                 "subscriptions", "max_fps", "continuous", "available", "model", "family", "mask_focus", "models", "ion",
                 "intrinsics"}  # /api/calibration/intrinsics body (calibration wizard)
    result_keys = {"api_version", "success", "mode", "frame_id", "ts_ms", "mono", "physics_time", "physics_step",
                   "inference_time_ms", "width", "height", "pipelines", "detections", "count", "family",
                   "undistorted", "rejected", "markers", "model_generation"}
    for depth in (None, DepthModel("off")):
        srv, base = _api(depth)
        try:
            p = srv.p
            mode = http(base + "/api/mode")[1]
            assert set(mode) == mode_keys and mode["pipelines"] == {"yolo": True, "aruco": True}
            assert mode["available"] == {"yolo": True, "aruco": True}
            assert http(base + "/api/trigger", {"pipelines": ["depth"], "wait_ms": 100})[0] == 400
            out = {}
            t = threading.Thread(target=_trigger, args=(base, {"pipelines": ["yolo"]}, out, "y"))
            t.start()
            time.sleep(0.3)
            _frame(p, 1)  # depth data on the frame is ignored without a model
            t.join(5)
            body = out["y"]
            assert set(body) - {"requested", "age_ms"} == result_keys - {"mono"}
            assert set(body["pipelines"]) == {"yolo", "aruco"} and body["detections"] == DETS
            assert set(p.result(p.frame, ["yolo"])) == result_keys
        finally:
            srv.httpd.shutdown()
            srv.httpd.server_close()


def test_mode_reports_depth_and_subscriptions_stay_yolo_aruco(api_depth):
    srv, base = api_depth
    mode = http(base + "/api/mode")[1]
    assert mode["available"]["depth"] is True and mode["pipelines"]["depth"] is True
    assert mode["depth_model"] == {"name": "rebot-sim-depth-metric", "path": "sim:metric", "metric": True}
    assert http(base + "/api/subscribe", {"client": "t", "pipelines": ["depth"]})[0] == 400  # trigger-only
    assert http(base + "/api/mode", {"mode": "both", "enabled": True})[0] == 200
    assert http(base + "/api/mode")[1]["continuous"]["pipelines"] == ["yolo", "aruco"]


def test_trigger_depth_alone(api_depth):
    srv, base = api_depth
    out = {}
    t = threading.Thread(target=_trigger, args=(base, {"pipelines": ["depth"]}, out, "d"))
    t.start()
    time.sleep(0.3)
    assert srv.p.wants_depth(time.monotonic())
    _frame(srv.p, 1)
    t.join(5)
    d = out["d"]
    assert d["success"] and d["requested"] == ["depth"] and d["pipelines"]["depth"]["ran"]
    assert not d["pipelines"]["yolo"]["ran"] and d["detections"] == []
    assert d["depth"] == {"metric_model": True, "grid_w": 384, "grid_h": 224, "table_samples": DEPTH["table_samples"]}


def test_pending_yolo_and_depth_triggers_share_one_frame(api_depth):
    """Union of pending triggers: a yolo and a depth trigger are answered by the same frame,
    which carries the depth fields on the masked detection only (the false one has no mask)."""
    srv, base = api_depth
    out = {}
    ts = [threading.Thread(target=_trigger, args=(base, {"pipelines": ["yolo"]}, out, "y")),
          threading.Thread(target=_trigger, args=(base, {"pipelines": ["yolo", "depth"]}, out, "yd"))]
    for t in ts:
        t.start()
    time.sleep(0.3)
    _frame(srv.p, 1)
    for t in ts:
        t.join(5)
    assert out["y"]["frame_id"] == out["yd"]["frame_id"] == 1
    for body in out.values():
        assert body["pipelines"]["yolo"]["ran"] and body["pipelines"]["depth"]["ran"] and "depth" in body
        real, false = body["detections"]
        assert real["disp_fg"] == 6.2 and real["jaw_clear"] is True and real["length_px"] == 50.0
        assert not {"disp_fg", "disp_bg", "depth_conf", "length_px", "jaw_clear"} & set(false)


def test_depth_fields_only_when_depth_ran(api_depth):
    srv, base = api_depth
    out = {}
    t = threading.Thread(target=_trigger, args=(base, {"pipelines": ["yolo"]}, out, "y"))
    t.start()
    time.sleep(0.3)
    assert not srv.p.wants_depth(time.monotonic())
    _frame(srv.p, 1)
    t.join(5)
    assert "depth" not in out["y"] and out["y"]["pipelines"]["depth"] == {"ran": False, "ms": 0.0}
    assert all("disp_fg" not in d for d in out["y"]["detections"])
    # a depth trigger on a frame without depth data waits for the next one
    t = threading.Thread(target=_trigger, args=(base, {"pipelines": ["depth"], "wait_ms": 3000}, out, "d"))
    t.start()
    time.sleep(0.3)
    _frame(srv.p, 2, depth=None)
    time.sleep(0.1)
    _frame(srv.p, 3)
    t.join(5)
    assert out["d"]["frame_id"] == 3 and out["d"]["pipelines"]["depth"]["ran"]
