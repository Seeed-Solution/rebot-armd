"""Perception API and camera-process plumbing without the physics loop running: HTTP
validation, the simulator pipe, trigger / subscription semantics, startup and shutdown."""
from __future__ import annotations

import json
import multiprocessing as mp
import socket
import urllib.error
import urllib.request

import pytest

from rebot_sim.server import Simulator


def free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http(url: str, body: dict | None = None, method: str | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method or ("GET" if body is None else "POST"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@pytest.fixture
def sim():
    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube,cup")
    yield s
    s.sock.close()


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
    from rebot_sim.perception import Perception, PerceptionServer

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
    import threading

    from rebot_sim.perception import SimLink

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
    import threading
    import time

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
    import threading
    import time

    from rebot_sim.camera import CameraMount

    sim = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    pport = free_port()
    ready = sim.start_camera(15, (640, 360), "127.0.0.1", free_port(), pport, objects="cube")
    if "error" in ready:
        sim.sock.close()
        pytest.skip(ready["error"])
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
        sim.cam_proc.terminate()
        sim.cam_proc.join(5)
        sim.sock.close()


def test_camera_reports_ready_only_after_its_encoders_exist():
    from rebot_sim.camera import CameraMount

    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    try:  # libx264 rejects an odd frame size when the encoder opens
        ready = s.start_camera(15, (641, 361), "127.0.0.1", free_port(), free_port(), objects="cube", timeout=30)
        assert "error" in ready, ready
        s.cam_proc.join(5)
        assert not s.cam_proc.is_alive()
    finally:
        if s.cam_proc is not None and s.cam_proc.is_alive():
            s.cam_proc.kill()
        s.sock.close()


def test_physics_loop_reports_a_dead_camera_process():
    import time

    from rebot_sim.camera import CameraMount

    s = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    try:
        ready = s.start_camera(15, (640, 360), "127.0.0.1", free_port(), free_port(), objects="cube")
        if "error" in ready:
            pytest.skip(ready["error"])
        s.cam_proc.kill()
        t0 = time.perf_counter()
        s.run(duration=10)
        assert time.perf_counter() - t0 < 5 and "camera process exited" in s.camera_error
    finally:
        if s.cam_proc.is_alive():
            s.cam_proc.kill()
        s.sock.close()
