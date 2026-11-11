"""Wrist camera: stream framing and decoding, ground-truth detections, perception API 2,
seeded resets. Runs the simulator in-process without the physics loop (the scene stands still).

The stream is decoded with rebot-companion's snapshot decoder when a checkout is found
(REBOT_COMPANION_DIR, or ../rebot-companion next to this repository or its worktree parent).
"""
from __future__ import annotations

import importlib.util
import json
import os
import socket
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import mujoco
import numpy as np
import pytest
from websockets.sync.client import connect

from rebot_sim.camera import DEFAULT_OFFSET_M, CameraMount, detections, project_points, segmentation_ids
from rebot_sim.model import load_model
from rebot_sim.scene import Scene, parse_objects, sample_poses
from rebot_sim.server import Simulator

REPO = Path(__file__).resolve().parents[3]


def free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def companion_snapshot():
    """rebot_companion/snapshot.py loaded by path (the package __init__ needs scipy)."""
    cands = [os.environ.get("REBOT_COMPANION_DIR")] + [str(p / "rebot-companion") for p in (REPO.parent, REPO.parent.parent)]
    for c in filter(None, cands):
        path = Path(c) / "rebot_companion" / "snapshot.py"
        if path.exists():
            sys.path.insert(0, str(Path(c)))  # for its `rebot` imports
            spec = importlib.util.spec_from_file_location("companion_snapshot", path)
            mod = importlib.util.module_from_spec(spec)
            sys.modules["companion_snapshot"] = mod
            spec.loader.exec_module(mod)
            return mod
    pytest.skip("no rebot-companion checkout (set REBOT_COMPANION_DIR)")


def http(url: str, body: dict | None = None, method: str | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method or ("GET" if body is None else "POST"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@pytest.fixture(scope="module")
def live():
    from rebot_sim.perception import CameraRig, PerceptionServer, serve_video

    sim = Simulator(free_port(socket.SOCK_DGRAM), objects="cube,cup", seed=None, camera_mount=CameraMount())
    sim.rig = CameraRig(sim, 15, (640, 360), scene_stream=False)
    sim.rig.ready.wait(30)
    if sim.rig.error:
        pytest.skip(sim.rig.error)
    vport, pport = free_port(), free_port()
    ws = serve_video("127.0.0.1", vport, sim.rig.streams)
    ps = PerceptionServer(sim.rig.perception, sim, "127.0.0.1", pport)
    yield sim, f"ws://127.0.0.1:{vport}/", f"http://127.0.0.1:{pport}"
    sim.stopping = True
    ws.shutdown()
    ps.httpd.shutdown()
    sim.sock.close()


def test_stream_framing_and_decode(live):
    import av

    _, url, _ = live
    msgs = []
    with connect(url, max_size=None) as ws:
        t0 = time.time()
        while len(msgs) < 20 and time.time() - t0 < 10:
            msgs.append(ws.recv(timeout=5))
    assert len(msgs) == 20 and all(isinstance(m, bytes) for m in msgs)
    now_ms = time.time() * 1000
    ctx = av.CodecContext.create("h264", "r")
    frames = []
    for i, m in enumerate(msgs):
        au, ts = m[:-8], int.from_bytes(m[-8:], "little")
        assert abs(now_ms - ts) < 10_000, "tail is not unix ms"
        assert au.startswith(b"\x00\x00\x00\x01") or au.startswith(b"\x00\x00\x01"), "not Annex-B"
        types = [au[j + 3] & 0x1F for j in range(len(au) - 3) if au[j:j + 3] == b"\x00\x00\x01"]
        if i == 0:
            assert {5, 7, 8} <= set(types), f"first message must be an IDR with SPS/PPS, got NAL types {types}"
        for pkt in ctx.parse(au):
            frames += ctx.decode(pkt)
    assert len(frames) >= 18 and (frames[0].width, frames[0].height) == (640, 360)


def test_stream_decodes_with_rebot_companion_snapshot(live):
    snapshot = companion_snapshot()
    _, url, _ = live
    snap = snapshot.snapshot_once(url, timeout=5, backend="pyav")
    assert (snap.width, snap.height) == (640, 360) and snap.jpeg[:2] == b"\xff\xd8"
    assert abs(snap.host_rx_ms - snap.device_ts_ms) < 2000


def test_detections_contain_the_projected_object_centre_and_match_the_render():
    objects = parse_objects("cube,cup,block")
    model = load_model(devarm_dir="", objects=objects, camera=CameraMount())
    data = mujoco.MjData(model)
    scene = Scene(model, objects)
    scene.reset(data, seed=None)
    # a pose (found by sampling) with the cube whole in view and the block cut by the image edge
    for j, q in zip(range(1, 7), [0.46, -0.63, -1.05, 1.48, 0.43, -0.66]):
        data.qpos[model.jnt_qposadr[model.joint(f"joint{j}").id]] = q
    mujoco.mj_forward(model, data)
    cam = model.camera("wrist").id
    renderer = mujoco.Renderer(model, 360, 640)
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="wrist")
    dets = detections(segmentation_ids(renderer.render()), scene)
    assert [d["name"] for d in dets] == ["cube", "block"], dets
    for d in dets:
        gid = model.geom(d["name"]).id
        (cx, cy), = project_points(data, cam, data.geom_xpos[gid][None])[0]
        x0, x1 = (d["x"] - d["w"] / 2) * 640, (d["x"] + d["w"] / 2) * 640
        y0, y1 = (d["y"] - d["h"] / 2) * 360, (d["y"] + d["h"] / 2) * 360
        assert 0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 360
        if 0 <= cx <= 640 and 0 <= cy <= 360:
            assert x0 <= cx <= x1 and y0 <= cy <= y1, (d, cx, cy)
        else:  # centre outside the image: the box is the visible part, ending at the edge
            assert x0 <= 0.5 or y0 <= 0.5 or x1 >= 639.5 or y1 >= 359.5, d
    cube = dets[0]
    (cx, cy), = project_points(data, cam, data.geom_xpos[model.geom("cube").id][None])[0]
    assert abs(cube["x"] * 640 - cx) < 0.25 * cube["w"] * 640 and abs(cube["y"] * 360 - cy) < 0.25 * cube["h"] * 360


def test_reset_is_deterministic_by_seed():
    objs = parse_objects("cube,cup,block")
    assert sample_poses(objs, 7) == sample_poses(objs, 7)
    assert sample_poses(objs, 7) != sample_poses(objs, 8)
    for seed in range(50):
        p = sample_poses(objs, seed)
        assert all(np.hypot(a[0] - b[0], a[1] - b[1]) >= 0.09 for i, a in enumerate(p) for b in p[i + 1:])
    sim = Simulator(free_port(socket.SOCK_DGRAM), objects="cube,cup,block")
    try:
        a = sim.request_reset(7, timeout=0.05)
        sim.request_reset(8, timeout=0.05)
        b = sim.request_reset(7, timeout=0.05)
        assert a["seed"] == 7 and [o["pos"] for o in a["objects"]] == [o["pos"] for o in b["objects"]]
        assert a["objects"][0]["pos"][:2] == pytest.approx(list(sample_poses(objs, 7)[0][:2]), abs=1e-5)
    finally:
        sim.sock.close()


def test_perception_api_v2(live):
    sim, _, base = live
    code, mode = http(base + "/api/mode")
    assert code == 200 and mode["api_version"] == 2 and mode["per_request_pipelines"] and mode["subscriptions"]
    code, r1 = http(base + "/api/trigger", {"pipelines": ["yolo"], "wait_ms": 2000})
    assert code == 200 and r1["success"] and r1["requested"] == ["yolo"] and r1["width"] == 640
    assert r1["pipelines"]["yolo"]["ran"] and not r1["pipelines"]["aruco"]["ran"] and r1["markers"] == []
    code, r2 = http(base + "/api/trigger", {"pipelines": ["aruco"]})
    assert r2["frame_id"] > r1["frame_id"] and r2["detections"] == []
    assert http(base + "/api/trigger", {"pipelines": ["sift"]})[0] == 400
    # continuous: nothing until a subscription
    code, lat = http(base + "/api/results/latest?after=%d&wait_ms=300" % r2["frame_id"])
    assert lat.get("timeout") and lat["enabled"] is False
    code, sub = http(base + "/api/subscribe", {"client": "t", "pipelines": ["yolo"], "fps": 5, "ttl_ms": 2000})
    assert code == 200 and sub["continuous"]["active"] and sub["continuous"]["pipelines"] == ["yolo"]
    code, lat = http(base + "/api/results/latest?after=%d&wait_ms=2000" % r2["frame_id"])
    assert lat["enabled"] and not lat.get("timeout") and lat["frame_id"] > r2["frame_id"]
    assert {d["class_name"] for d in lat["detections"]} <= {"cube", "cup"}
    assert http(base + f"/api/subscribe/{sub['id']}/keepalive", {})[0] == 200
    code, subs = http(base + "/api/subscriptions")
    assert [s["client"] for s in subs["subscriptions"]] == ["t"]
    assert http(base + f"/api/subscribe/{sub['id']}", method="DELETE")[0] == 200
    assert http(base + f"/api/subscribe/{sub['id']}/keepalive", {})[0] == 404
    # sim routes
    code, st = http(base + "/sim/reset", {"seed": 5})
    assert code == 200 and st["seed"] == 5 and len(st["objects"]) == 2
    code, cam = http(base + "/sim/camera")
    assert cam["camera_mount"]["offset_m"] == list(DEFAULT_OFFSET_M) and cam["fx_px"] == pytest.approx(823.8, abs=0.1)
    assert cam["render_backend"] in ("egl", "osmesa", "glfw", "default")
