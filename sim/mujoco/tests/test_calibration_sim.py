"""Calibration wizard support in the simulator (fast): virtual GridBoard projection and marker
JSON, the truth camera model (--fx, q_end_cam) and its IPC, and GET/PUT
/api/calibration/intrinsics."""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from types import SimpleNamespace

import numpy as np
import pytest

from rebot_sim import aruco
from rebot_sim import camera as cam
from rebot_sim.camera import CameraMount
from rebot_sim.perception import Perception, PerceptionServer

F = 800.0
DOWN = np.column_stack([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # camera axes = board axes at yaw 0


def test_board_geometry_and_ids():
    assert aruco.Board(0.0, 0.0).side_m == 0.025 and aruco.board_object_points().keys() == aruco.board_object_points(0.025).keys()
    pts = aruco.board_object_points(0.025)
    assert sorted(pts) == list(range(12))
    allp = np.concatenate(list(pts.values()))
    assert allp.shape == (48, 3)
    w, h = 4 * 0.025 + 3 * 0.005, 3 * 0.025 + 2 * 0.005  # 115 x 85 mm, landscape
    assert np.allclose(allp[:, 0].min(), -w / 2) and np.allclose(allp[:, 1].max(), h / 2)
    assert np.allclose(np.ptp(allp[:, 0]), w) and np.allclose(np.ptp(allp[:, 1]), h)
    # id = row * 4 + col: id 0 top-left of the page, id 3 top-right, id 8 bottom-left; corner 3 = top-left
    assert np.allclose(pts[0][3][:2], [-w / 2, -h / 2]) and np.allclose(pts[3][2][:2], [w / 2, -h / 2])
    assert np.allclose(pts[8][0][:2], [-w / 2, h / 2]) and np.allclose(pts[11][1][:2], [w / 2, h / 2])
    # yaw 0 on the table: page x = base +x, page y = base -y, z down
    r, o = aruco.Board(0.25, 0.0).pose()
    assert np.allclose(r, DOWN) and np.allclose(o, [0.25, 0.0, 0.0])
    r90, _ = aruco.Board(0.0, 0.0, 90.0).pose()
    assert np.allclose(r90[:, 0], [0, 1, 0], atol=1e-12) and np.allclose(np.linalg.det(r90), 1.0)
    assert aruco.Board.parse("0.3,-0.05,15") == aruco.Board(0.3, -0.05, 15.0)
    with pytest.raises(ValueError):
        aruco.Board.parse("0.3")


def test_projection_of_a_known_board():
    board = aruco.Board(0.25, 0.02, 0.0)
    origin = np.array([0.25, 0.02, 0.6])  # 0.6 m straight above the board centre
    ms = aruco.visible_markers(board, DOWN, origin, F)
    assert [m["id"] for m in ms] == list(range(12)), "the whole board is in view"
    obj = aruco.board_object_points()
    for m in ms:
        want = np.column_stack([F * obj[m["id"]][:, 0] / 0.6 + 320, F * obj[m["id"]][:, 1] / 0.6 + 180])
        assert np.allclose(m["corners"], want, atol=1e-9)
    # shifted view: only fully visible markers, never a partial one
    part = aruco.visible_markers(board, DOWN, origin + [0.1, 0.0, -0.25], F)
    assert 0 < len(part) < 12
    for m in part:
        c = m["corners"]
        assert (c[:, 0] >= 0).all() and (c[:, 0] <= 640).all() and (c[:, 1] >= 0).all() and (c[:, 1] <= 360).all()
    # noise: seeded per frame, sigma as asked
    n1 = aruco.visible_markers(board, DOWN, origin, F, noise_px=0.5, seed=3, frame_id=7)
    n2 = aruco.visible_markers(board, DOWN, origin, F, noise_px=0.5, seed=3, frame_id=7)
    n3 = aruco.visible_markers(board, DOWN, origin, F, noise_px=0.5, seed=3, frame_id=8)
    assert all(np.array_equal(a["corners"], b["corners"]) for a, b in zip(n1, n2))
    assert not np.array_equal(n1[0]["corners"], n3[0]["corners"])
    d = np.concatenate([a["corners"] - b["corners"] for a, b in zip(n1, ms)])
    assert 0.35 < d.std() < 0.65


def test_marker_pose_with_true_and_estimated_intrinsics():
    board = aruco.Board(0.25, 0.02, 0.0)
    origin = np.array([0.20, 0.0, 0.5])
    m = next(x for x in aruco.visible_markers(board, DOWN, origin, F) if x["id"] == 4)
    truth = {"fx": F, "fy": F, "cx": 320.0, "cy": 180.0}
    p = aruco.marker_pose(m["corners"], truth, 0.025)
    centre = aruco.board_world_points(board)[4].mean(axis=0)
    want_t = (centre - origin) @ DOWN
    assert np.allclose(p["t"], want_t, atol=2e-4), (p, want_t)
    assert np.allclose(p["r_deg"], [0.0, 0.0, 0.0], atol=0.05) and p["error"] < 1e-10
    # estimated fx 5 % low: same direction, distance scaled by fx_est / fx_true
    est = aruco.marker_pose(m["corners"], {**truth, "fx": 0.95 * F, "fy": 0.95 * F}, 0.025)
    assert est["t"][2] == pytest.approx(0.95 * want_t[2], abs=2e-3)
    # tilted view: the rotation comes back
    tilt = cam.axis_r((1.0, 0.0, 0.0), math.radians(20.0))
    r = DOWN @ tilt
    m = aruco.visible_markers(board, r, np.array([0.25, -0.15, 0.45]), F)[0]
    q = aruco.marker_pose(m["corners"], truth, 0.025)
    assert abs(abs(q["r_deg"][0]) - 20.0) < 0.1, q
    # JSON entry as rebot-vision writes it
    j = aruco.marker_json(m, truth, 0.025)
    assert set(j) == {"id", "margin", "center", "corners", "pose"} and len(j["corners"]) == 4
    assert set(j["pose"]) == {"t", "r_deg", "error"}


def test_undistort_inverts_the_distortion_model():
    k = {"fx": 820.0, "fy": 815.0, "cx": 318.0, "cy": 182.0, "k1": -0.2, "k2": 0.08, "p1": 0.001, "p2": -0.002, "k3": 0.0}
    xy = np.array([[0.1, -0.2], [-0.3, 0.15], [0.0, 0.0]])
    r2 = (xy ** 2).sum(axis=1)
    rad = 1 + k["k1"] * r2 + k["k2"] * r2 ** 2
    xd = xy[:, 0] * rad + 2 * k["p1"] * xy[:, 0] * xy[:, 1] + k["p2"] * (r2 + 2 * xy[:, 0] ** 2)
    yd = xy[:, 1] * rad + k["p1"] * (r2 + 2 * xy[:, 1] ** 2) + 2 * k["p2"] * xy[:, 0] * xy[:, 1]
    uv = np.column_stack([k["fx"] * xd + k["cx"], k["fy"] * yd + k["cy"]])
    assert np.allclose(aruco.undistort_normalised(uv, k), xy, atol=1e-9)


def test_truth_model_fx_and_quaternion_ipc():
    from rebot_sim.model import load_model

    model = load_model(devarm_dir="")
    legacy = CameraMount(False, 0.0, -38.0, (-0.0731, 0.0005, 0.066))
    r_legacy = cam.mount_rotation(model, legacy)
    q = cam.r_to_quat(r_legacy)
    m = CameraMount(False, 0.0, -38.0, (-0.0731, 0.0005, 0.066), q, 760.0)
    assert np.allclose(cam.mount_rotation(model, m), r_legacy, atol=1e-9)
    assert CameraMount(*m.ipc()) == m, "the camera process rebuilds the same mount"
    assert CameraMount(*legacy.ipc()[:4]) == legacy, "old 4-entry configs still work"
    # q replaces flip / rot / tilt
    m2 = CameraMount(True, 40.0, 10.0, None, q)
    assert np.allclose(cam.mount_rotation(model, m2), r_legacy, atol=1e-9)
    # fx drives focal length, FOV and the grip pixel
    assert m.focal() == 760.0 and m.focal(720) == 1520.0
    assert m.vfov_deg == pytest.approx(2 * math.degrees(math.atan(180 / 760.0)))
    assert legacy.focal() == pytest.approx(cam.focal_px(360)) and legacy.vfov_deg == pytest.approx(cam.VFOV_DEG)
    g = cam.grip_pixel_norm(r_legacy, legacy.offset, 0.30, 760.0)
    g0 = cam.grip_pixel_norm(r_legacy, legacy.offset, 0.30)
    assert (g[1] - 0.5) == pytest.approx((g0[1] - 0.5) * 760.0 / cam.focal_px(360), abs=2e-4)
    with pytest.raises(ValueError):
        CameraMount(q_end_cam=(1.1, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        CameraMount(fx_px=50.0)
    # the rendered camera has the truth FOV
    import mujoco

    model_fx = load_model(devarm_dir="", camera=m)
    assert model_fx.cam_fovy[mujoco.mj_name2id(model_fx, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")] == pytest.approx(m.vfov_deg)


def _server(p: Perception):
    return SimpleNamespace(p=p, sim=None, dispatch=lambda *a: PerceptionServer.dispatch(SimpleNamespace(p=p, sim=None), *a))


def test_intrinsics_get_put_revision():
    p = Perception(max_fps=10, nominal_fx=824.4)
    s = _server(p)
    code, body = s.dispatch("GET", "/api/calibration/intrinsics", {}, {})
    assert code == 200 and body["fx"] == 824.4 and body["revision"] == 0 and (body["width"], body["height"]) == (640, 360)
    new = {**{k: body[k] for k in aruco.INTRINSIC_KEYS}, "fx": 801.0, "fy": 799.5, "width": 640, "height": 360,
           "marker_size": 0.049, "expected_revision": 0, "revision": 0, "api_version": 2}
    code, out = s.dispatch("PUT", "/api/calibration/intrinsics", {}, new)
    assert code == 200 and out["fx"] == 801.0 and out["revision"] == 1 and out["marker_size"] == 0.049
    code, out = s.dispatch("PUT", "/api/calibration/intrinsics", {}, new)  # stale expected_revision
    assert code == 409 and out["error"] == "REVISION_MISMATCH" and out["revision"] == 1
    for k, v in [("fx", 50.0), ("cx", 640.0), ("k1", 11.0), ("p1", 2.0), ("width", 1280), ("marker_size", 0.0), ("bogus", 1)]:
        code, out = s.dispatch("PUT", "/api/calibration/intrinsics", {}, {**new, "expected_revision": 1, k: v})
        assert code == 400 and out["field"] == k, (k, out)
    assert p.intrinsics["fx"] == 801.0 and p.intrinsics_revision == 1
    code, mode = s.dispatch("GET", "/api/mode", {}, {})
    assert mode["intrinsics"]["fx"] == 801.0 and mode["intrinsics"]["revision"] == 1
    # marker poses use the estimated intrinsics
    board = aruco.Board(0.25, 0.0)
    markers = aruco.visible_markers(board, DOWN, np.array([0.25, 0.0, 0.6]), F)
    frame = {"frame_id": 1, "ts_ms": 0, "mono": 0.0, "detections": [], "physics_time": 0.0, "physics_step": 1,
             "depth": None, "markers": markers}
    res = p.result(frame, ["aruco"])
    assert res["pipelines"]["aruco"]["ran"] and res["count"] == 12 and len(res["markers"]) == 12
    m0 = res["markers"][0]
    assert m0["pose"]["t"][2] == pytest.approx(0.6 * 801.0 / F * 0.049 / 0.025, abs=2e-3)
    assert p.result(frame, ["yolo"])["markers"] == [], "aruco did not run"


def test_intrinsics_put_over_http():
    from test_armd_sim import free_port

    p = Perception(max_fps=10)
    port = free_port()
    PerceptionServer(p, SimpleNamespace(), "127.0.0.1", port)
    url = f"http://127.0.0.1:{port}/api/calibration/intrinsics"
    cur = json.loads(urllib.request.urlopen(url, timeout=5).read())
    body = {**cur, "fx": 790.0, "expected_revision": cur["revision"]}
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="PUT", headers={"Content-Type": "application/json"})
    out = json.loads(urllib.request.urlopen(req, timeout=5).read())
    assert out["fx"] == 790.0 and out["revision"] == 1
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(urllib.request.Request(url, data=json.dumps(body).encode(), method="PUT",
                                                      headers={"Content-Type": "application/json"}), timeout=5)
    assert e.value.code == 409
