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
