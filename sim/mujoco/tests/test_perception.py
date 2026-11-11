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
    child.send(("reset", -1))
    sim.service_camera()  # must not raise into the physics loop
    reply = child.recv()
    assert "error" in reply and "seed" in reply["error"] and sim.scene.seed is None
    t = sim.data.time
    sim.step()
    assert sim.data.time > t and sim.cam_conn is parent
    child.send(("reset", 4))
    sim.service_camera()
    assert child.recv()["seed"] == 4


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
