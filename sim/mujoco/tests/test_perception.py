"""Perception API and camera-process plumbing without the physics loop running: HTTP
validation, the simulator pipe, trigger / subscription semantics, startup and shutdown."""
from __future__ import annotations

import json
import multiprocessing as mp
import os
import signal
import socket
import threading
import time
import urllib.error
import urllib.request
from multiprocessing import shared_memory
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest
from test_armd_sim import free_port

from rebot_sim import camera as cam
from rebot_sim.camera import CameraMount
from rebot_sim.model import load_model
from rebot_sim.perception import (
    CameraRig,
    Perception,
    PerceptionServer,
    SharedState,
    SimLink,
)
from rebot_sim.server import Simulator


def http(url: str, body: dict | None = None, method: str | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method or ("GET" if body is None else "POST"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def set_arm(model: mujoco.MjModel, data: mujoco.MjData, q) -> None:
    """Put J1..J6 at `q` (rad) and update the kinematics."""
    for j, v in zip(range(1, 7), q):
        data.qpos[model.jnt_qposadr[model.joint(f"joint{j}").id]] = v
    mujoco.mj_forward(model, data)


def camera_sim() -> tuple[Simulator, int]:
    """A one-cube simulator with its camera process started (skips without GL / libx264);
    returns it and the perception port."""
    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    pport = free_port()
    ready = s.start_camera(15, (640, 360), "127.0.0.1", free_port(), pport, objects="cube")
    if "error" in ready:
        s.close()
        pytest.skip(ready["error"])
    return s, pport


@pytest.fixture
def sim():
    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube,cup")
    yield s
    s.close()


def test_bad_reset_seed_is_answered_and_physics_keeps_running(sim):
    parent, child = mp.Pipe()
    sim.cam_conn = parent
    child.send((1, "reset", -1))
    sim.service_camera()  # must not raise into the physics loop
    rid, reply = child.recv()
    assert rid == 1 and "error" in reply and "seed" in reply["error"] and sim.scene.seed is None
    t = sim.data.time
    sim.step()
    assert sim.data.time > t and sim.cam_conn is parent
    child.send((2, "reset", 4))
    sim.service_camera()
    assert child.recv() == (2, sim.sim_state())


class FakeSim:
    """PerceptionServer's simulator interface, recording calls."""

    def __init__(self):
        self.resets = []

    def request_reset(self, seed):
        self.resets.append(seed)
        return {"seed": seed}

    def sim_state(self):
        return {"t": 0.0}

    def camera_info(self):
        return {}


@pytest.fixture
def api():
    fake = FakeSim()
    srv = PerceptionServer(Perception(max_fps=15), fake, "127.0.0.1", 0)
    yield srv, fake, f"http://127.0.0.1:{srv.httpd.server_address[1]}"
    srv.httpd.shutdown()
    srv.httpd.server_close()


@pytest.mark.parametrize("seed", [-1, 1.5, "3", True])
def test_reset_seed_must_be_a_non_negative_integer(api, seed):
    _, fake, base = api
    assert http(base + "/sim/reset", {"seed": seed})[0] == 400
    assert fake.resets == []
    assert http(base + "/sim/reset", {"seed": 0}) == (200, {"seed": 0})
    assert http(base + "/sim/reset", {}) == (200, {"seed": None})


def test_late_simulator_reply_does_not_answer_the_next_request(sim):
    parent, child = mp.Pipe()
    sim.cam_conn, link = parent, SimLink(child)
    with pytest.raises(TimeoutError):
        link.call(("state",), timeout=0.1)  # nobody serves the pipe
    sim.service_camera()  # the physics loop answers the timed-out request late
    out = {}
    t = threading.Thread(target=lambda: out.update(link.call(("reset", 3), timeout=2.0)))
    t.start()
    while t.is_alive():
        sim.service_camera()
        t.join(0.01)
    assert out["seed"] == 3


def test_simulator_timeout_is_a_server_error(api):
    _, fake, base = api

    def stuck():
        raise TimeoutError("simulator did not answer")

    fake.sim_state = stuck
    code, body = http(base + "/sim/state")
    assert code == 504 and "simulator" in body["error"]


def test_subscription_fps_follows_the_current_cap(api):
    _, _, base = api
    assert http(base + "/api/mode", {"fps": 5})[0] == 200
    code, sub = http(base + "/api/subscribe", {"client": "t", "pipelines": ["yolo"], "fps": 10})
    assert code == 200 and sub["fps"] == 5 and sub["requested_fps"] == 10 and sub["continuous"]["fps"] == 5
    assert http(base + "/api/mode", {"fps": 15})[0] == 200
    code, subs = http(base + "/api/subscriptions")
    assert subs["subscriptions"][0]["fps"] == 10 and subs["continuous"]["fps"] == 10
    assert http(base + "/api/mode", {"fps": 3})[0] == 200
    code, subs = http(base + "/api/subscriptions")
    assert subs["subscriptions"][0]["fps"] == 3 and subs["continuous"]["fps"] == 3


@pytest.mark.parametrize("body", [{"mode": "aruco", "fps": 99}, {"mode": "both", "enabled": "yes"},
                                  {"enabled": True, "fps": 0}, {"mode": "sift", "enabled": True}])
def test_mode_body_is_validated_before_anything_changes(api, body):
    _, _, base = api
    before = http(base + "/api/mode")[1]
    assert http(base + "/api/mode", body)[0] == 400
    after = http(base + "/api/mode")[1]
    assert {k: after[k] for k in ("mode", "enabled", "fps", "last_active_mode", "max_fps")} == \
        {k: before[k] for k in ("mode", "enabled", "fps", "last_active_mode", "max_fps")}


def test_trigger_adds_its_pipelines_to_the_next_frame_and_shares_the_result(api):
    """API v2 union (kits/recamera-b601dm/vision VisionControl::frame_plan): a trigger's
    pipelines join the next frame's continuous plan; that one result answers the trigger and
    becomes `latest`."""
    srv, _, base = api
    assert http(base + "/api/subscribe", {"client": "t", "pipelines": ["yolo"], "fps": 15})[0] == 200
    dets = [{"class_name": "cube"}]
    srv.p.on_frame(1, 1000, time.monotonic(), dets)  # continuous result before the trigger
    out = {}
    t = threading.Thread(target=lambda: out.update(http(base + "/api/trigger", {"pipelines": ["aruco"]})[1]))
    t.start()
    time.sleep(0.3)  # trigger registered
    srv.p.on_frame(2, 1100, time.monotonic(), dets)
    t.join(5)
    assert out["success"] and out["frame_id"] == 2 and out["requested"] == ["aruco"]
    assert out["pipelines"]["yolo"]["ran"] and out["pipelines"]["aruco"]["ran"] and out["detections"] == dets
    code, lat = http(base + "/api/results/latest")
    assert lat["frame_id"] == 2 and lat["pipelines"] == out["pipelines"] and "requested" not in lat


def test_camera_publishes_no_frame_without_new_physics():
    """The camera process renders only physics states it has not rendered yet; results and
    /sim/state carry the physics time of the frame."""
    sim, pport = camera_sim()
    base, stop, mu = f"http://127.0.0.1:{pport}", threading.Event(), threading.Lock()

    def pipe_only():  # answers /sim requests; the physics does not step
        while not stop.is_set():
            with mu:
                sim.service_camera()
            time.sleep(0.005)

    server = threading.Thread(target=pipe_only, daemon=True)
    server.start()
    try:
        deadline = time.time() + 20
        while http(base + "/sim/camera")[1]["render"]["frames"] < 1 and time.time() < deadline:
            time.sleep(0.05)
        code, r = http(base + "/api/trigger", {"pipelines": ["yolo"], "wait_ms": 800})
        assert code == 200 and r["success"] is False and r["timeout"], "frame published without new physics"
        with mu:
            for _ in range(3):
                sim.step()
        t = round(sim.data.time, 6)
        code, r = http(base + "/api/trigger", {"pipelines": ["yolo"], "wait_ms": 2000})
        assert r["success"] and r["physics_time"] == pytest.approx(t, abs=1e-6)
        code, st = http(base + "/sim/state")
        assert code == 200 and st["camera"]["physics_time"] == pytest.approx(t, abs=1e-6)
        assert st["camera"]["lag_s"] == pytest.approx(0.0, abs=1e-6) and st["camera"]["stale"] is False
        assert st["camera"]["skipped"] > 0
    finally:
        stop.set()
        server.join(2)
        sim.close()


def test_camera_reports_ready_only_after_its_encoders_exist():
    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    try:  # libx264 rejects an odd frame size when the encoder opens
        ready = s.start_camera(15, (641, 361), "127.0.0.1", free_port(), free_port(), objects="cube", timeout=30)
        assert "error" in ready, ready
    finally:
        s.close()


def test_physics_loop_reports_a_dead_camera_process():
    s, _ = camera_sim()
    try:
        s.cam_proc.kill()
        t0 = time.perf_counter()
        s.run(duration=10)
        assert time.perf_counter() - t0 < 5 and "camera process exited" in s.camera_error
    finally:
        s.close()


def _shm_exists(name: str) -> bool:
    try:
        shared_memory.SharedMemory(name=name, track=False).close()
    except FileNotFoundError:
        return False
    return True


def test_close_stops_the_camera_and_frees_everything():
    s, pport = camera_sim()
    proc, name = s.cam_proc, s.shared.shm.name
    s.close()
    assert not proc.is_alive() and not _shm_exists(name) and s.sock.fileno() == -1
    with pytest.raises(OSError):
        urllib.request.urlopen(f"http://127.0.0.1:{pport}/api/mode", timeout=2)
    s.close()  # idempotent


def test_failed_camera_start_frees_the_child_and_shared_memory():
    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    try:
        assert "error" in s.start_camera(15, (641, 361), "127.0.0.1", free_port(), free_port(), objects="cube")
        assert s.cam_proc is None and s.shared is None and s.cam_conn is None
    finally:
        s.close()


def test_camera_dying_mid_request_does_not_break_the_physics_loop(sim):
    parent, child = mp.Pipe()
    sim.cam_conn = parent
    child.send((1, "state"))
    child.close()  # the camera process is gone before the reply
    sim.service_camera()
    assert sim.cam_conn is None and "camera process exited" in sim.camera_error
    t = sim.data.time
    sim.step()
    assert sim.data.time > t


def _write_states(name: str, nq: int, nv: int, lock, n: int) -> None:
    st = SharedState(nq, nv, name, lock)
    for k in range(1, n + 1):
        st.write(SimpleNamespace(time=float(k), qpos=np.full(nq, float(k)), qvel=np.full(nv, float(k))))
    st.close()


def test_shared_state_is_consistent_across_processes():
    """Writer and reader serialise on a multiprocessing lock: a read never mixes two states,
    and a read waits while a write holds the lock."""
    ctx = mp.get_context("spawn")
    nq, nv = 30, 28
    st = SharedState(nq, nv, lock=ctx.Lock())
    data = SimpleNamespace(time=0.0, qpos=np.zeros(nq), qvel=np.zeros(nv))
    try:
        writer = ctx.Process(target=_write_states, args=(st.shm.name, nq, nv, st.lock, 200_000))
        writer.start()
        reads = 0
        while writer.is_alive() or reads == 0:
            if st.read(data) is not None:
                reads += 1
                assert (data.qpos == data.time).all() and (data.qvel == data.time).all()
        writer.join()
        assert writer.exitcode == 0 and reads > 0
        got = []
        with st.lock:  # a read waits for the lock, at most LOCK_TIMEOUT_S, then gives up
            t = threading.Thread(target=lambda: got.append(st.read(data)))
            t.start()
            t.join(0.1)
            assert t.is_alive(), "read did not wait for the lock"
            t.join(2)
            assert got == [None]
        assert st.read(data) == 200_000 and data.time == 200_000
    finally:
        st.close()


def _rig():
    m = CameraMount()
    model = load_model(devarm_dir="", camera=m)
    st = SharedState(model.nq, model.nv)
    cfg = {"fps": 15, "size": [640, 360], "mount": [m.flip, m.rot_deg, m.tilt_deg, m.offset_m],
           "objects": "cube", "scene_stream": False, "has_meshes": False}
    return CameraRig(cfg, st, SimLink(None)), st


def test_segmentation_decode_rejects_out_of_range_ids_and_matches_mujoco():
    rig, st = _rig()
    try:
        rig.setup()  # also renders once
        set_arm(rig.model, rig.data, [0.46, -0.63, -1.05, 1.48, 0.43, -0.66])  # cube in view
        r = rig.seg_renderer
        r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        ours = cam.render_segmentation(r)
        ref = mujoco.Renderer(rig.model, 360, 640)
        ref.enable_segmentation_rendering()
        ref.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        assert (ours == cam.segmentation_ids(ref.render())).all() and (ours >= 0).any()
        garbage = np.zeros((360, 640, 3), np.uint8)
        garbage[5, 7] = (0xB0, 0x0A, 0xFE)  # an id far above ngeom, as seen from mujoco 4.0.0
        with pytest.raises(cam.SegmentationError) as e:
            cam.decode_segmentation(garbage, r.scene)
        assert e.value.diag["max_value"] == 0xFE0AB0 and e.value.diag["bad_pixels"] == 1
    finally:
        st.close()


def _segmentation_startup(frames: int) -> dict:
    """One camera start-up (run in a fresh process): setup, then `frames` segmentations of
    different arm poses against mujoco's own segmentation render of the same scene."""
    rig, st = _rig()
    try:
        rig.setup()
        ref = mujoco.Renderer(rig.model, 360, 640)
        ref.enable_segmentation_rendering()
        rng = np.random.default_rng(os.getpid())
        out = {"frames": 0, "mismatched": 0, "errors": [], "visible": 0}
        for i in range(frames):
            q = np.array([0.46, -0.63, -1.05, 1.48, 0.43, -0.66]) + (rng.normal(0, 0.05, 6) if i else 0)
            set_arm(rig.model, rig.data, q)
            r = rig.seg_renderer
            r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
            try:
                ours = cam.render_segmentation(r)
            except cam.SegmentationError as exc:
                out["errors"].append(str(exc))
                continue
            ref.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
            out["frames"] += 1
            out["mismatched"] += int(not (ours == cam.segmentation_ids(ref.render())).all())
            out["visible"] += int((ours >= 0).any())
        ref.close()
        return out
    finally:
        st.close()


@pytest.mark.slow
def test_segmentation_over_many_camera_startups_matches_mujoco():
    """20 start-ups in fresh processes, 10 frames each: no failed segmentation, every frame equal
    to mujoco's segmentation (the canary row is cropped from the readback)."""
    ctx = mp.get_context("spawn")
    with ctx.Pool(1, maxtasksperchild=1) as pool:
        runs = [pool.apply(_segmentation_startup, (10,)) for _ in range(20)]
    assert [r["errors"] for r in runs] == [[]] * 20
    assert sum(r["frames"] for r in runs) == 200 and sum(r["mismatched"] for r in runs) == 0
    assert all(r["visible"] == 10 for r in runs)


def _drop_gpu_commands(monkeypatch, on: bool, names=("mjr_render", "mjr_rectangle")) -> None:
    """Stand-in for the GPU driver out of memory: the named draw calls do nothing, so the
    readback keeps what the previous frame left there (all valid ids, the previous canary)."""
    for name in names:
        monkeypatch.setattr(mujoco, name, (lambda *a: None) if on else _GL_DRAW[name])


_GL_DRAW = {name: getattr(mujoco, name) for name in ("mjr_render", "mjr_rectangle")}


def test_a_dropped_segmentation_render_is_detected_by_its_canary(monkeypatch):
    rig, st = _rig()
    try:
        rig.setup()
        set_arm(rig.model, rig.data, [0.46, -0.63, -1.05, 1.48, 0.43, -0.66])  # cube in view
        r = rig.seg_renderer
        r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        before = cam.render_segmentation(r)
        _drop_gpu_commands(monkeypatch, True)
        with pytest.raises(cam.SegmentationError, match="not executed") as e:
            cam.render_segmentation(r)  # the stale buffer holds only valid ids: the bounds check alone passes it
        assert e.value.diag["canary_read"] != e.value.diag["canary_expected"]
        _drop_gpu_commands(monkeypatch, False)
        assert (cam.render_segmentation(r) == before).all()
    finally:
        st.close()


def test_a_dropped_scene_with_the_canary_drawn_is_not_detected(monkeypatch):
    """Known gap: when only the scene draw is lost and the stale buffer holds valid ids, the
    canary (drawn) and the bounds check (valid ids) both pass and the stale frame is returned."""
    rig, st = _rig()
    try:
        rig.setup()
        r = rig.seg_renderer
        set_arm(rig.model, rig.data, [0.46, -0.63, -1.05, 1.48, 0.43, -0.66])  # cube in view
        r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        before = cam.render_segmentation(r)
        set_arm(rig.model, rig.data, [0.0, -0.63, -1.05, 1.48, 0.43, -0.66])  # a different view
        r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        _drop_gpu_commands(monkeypatch, True, ("mjr_render",))
        stale = cam.render_segmentation(r)  # no SegmentationError
        _drop_gpu_commands(monkeypatch, False)
        truth = cam.render_segmentation(r)
        assert (stale == before).all() and not (stale == truth).all()
    finally:
        st.close()


def test_a_dropped_canary_with_the_scene_drawn_fails_the_segmentation(monkeypatch):
    rig, st = _rig()
    try:
        rig.setup()
        set_arm(rig.model, rig.data, [0.46, -0.63, -1.05, 1.48, 0.43, -0.66])
        r = rig.seg_renderer
        r.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
        cam.render_segmentation(r)
        _drop_gpu_commands(monkeypatch, True, ("mjr_rectangle",))
        with pytest.raises(cam.SegmentationError, match="not executed") as e:
            cam.render_segmentation(r)
        assert e.value.diag["canary_read"] != e.value.diag["canary_expected"]
    finally:
        st.close()


def test_missing_renderer_internals_fail_loudly():
    class Bare:
        height, width, scene = 360, 640, None

    with pytest.raises(RuntimeError, match=r"lacks \['_mjr_context', '_rect', '_gl_context'\]"):
        cam.check_renderer_internals(Bare())
    with pytest.raises(RuntimeError, match="lacks"):
        cam.render_segmentation(Bare())


def test_a_bad_segmentation_frame_is_skipped_and_ten_in_a_row_fail_the_camera(monkeypatch):
    rig, st = _rig()
    try:
        rig.setup()
        _drop_gpu_commands(monkeypatch, True)
        assert rig.detect() is None and rig.seg_failures == 1
        _drop_gpu_commands(monkeypatch, False)
        assert rig.detect() is not None and rig.seg_failures == 1 and rig.seg_consecutive == 0
        _drop_gpu_commands(monkeypatch, True)
        for _ in range(9):
            assert rig.detect() is None
        with pytest.raises(RuntimeError, match="10 consecutive"):
            rig.detect()
    finally:
        st.close()


def _hold_lock(lock, held) -> None:
    lock.acquire()
    held.set()
    time.sleep(60)


def test_physics_stops_when_the_camera_dies_holding_the_shared_state_lock(sim):
    ctx = mp.get_context("spawn")
    sim.shared = SharedState(sim.model.nq, sim.model.nv, lock=ctx.Lock())
    held = ctx.Event()
    holder = ctx.Process(target=_hold_lock, args=(sim.shared.lock, held))
    holder.start()
    assert held.wait(30)
    os.kill(holder.pid, signal.SIGKILL)  # dies with the lock held
    holder.join(5)
    t0 = time.perf_counter()
    sim.run(duration=5)
    assert time.perf_counter() - t0 < 1.0 and sim.camera_error == "shared-state lock timeout"
