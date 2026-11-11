"""Malformed command datagrams are ignored and counted; valid ones are applied whole."""
import json
import socket

import numpy as np

from rebot_sim.server import DOF, PROTOCOL_VERSION, Simulator


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def cmd(**kw) -> bytes:
    msg = {"v": PROTOCOL_VERSION, "type": "cmd", "seq": 1, "enable": True}
    msg.update(kw)
    return json.dumps(msg).encode()


def pos(q: float) -> dict:
    return {"mode": "pos_vel", "q": q, "vlim": 1.0}


def test_malformed_commands_change_nothing():
    sim = Simulator(free_udp_port())
    try:
        good = [pos(0.1 * i) for i in range(DOF)]
        sim.handle(cmd(seq=5, targets=good), ("127.0.0.1", 1))
        assert sim.ack == 5 and sim.motors.enabled
        q_before = sim.motors.q_t.copy()
        bad_targets = [pos(0.5)] * (DOF - 1) + [{"mode": "pos_vel", "q": "x"}]
        nan = [pos(0.5)] * (DOF - 1) + [{"mode": "pos_vel", "q": 1e400}]
        mit_bad = [{"mode": "mit", "q": 0.2, "kp": [1]}] * DOF
        bad = [
            b"\xff\xfe", b"[1, 2]", b"null", b"\"cmd\"", b"{\"v\": 1, \"type\": \"cmd\", \"targets\": [",
            cmd(targets="abc"), cmd(targets=[1] * DOF), cmd(targets=[pos(0.5)] * (DOF + 1)),
            cmd(targets=bad_targets), cmd(targets=nan), cmd(targets=mit_bad),
            cmd(enable="false", targets=good), cmd(seq="7", targets=good), cmd(seq=True), cmd(seq=-1),
            json.dumps({"v": PROTOCOL_VERSION, "type": "cmd", "seq": 9, "enable": False, "targets": [{"mode": "x"}] * DOF}).encode(),
        ]
        for i, payload in enumerate(bad):
            sim.handle(payload, ("127.0.0.1", 1))
            assert sim.rejected == i + 1, payload
        assert np.array_equal(sim.motors.q_t, q_before), "a malformed packet changed targets"
        assert sim.ack == 5 and sim.motors.enabled, "a malformed packet changed ack or enable"
        sim.handle(cmd(seq=6, enable=False), ("127.0.0.1", 1))
        assert sim.ack == 6 and not sim.motors.enabled
    finally:
        sim.sock.close()
