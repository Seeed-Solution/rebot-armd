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


def test_j2_j3_rest_on_their_stops_at_home():
    # at q = 0 (J2 / J3 upper stops) gravity pushes both into the stop: it carries the load
    # and the motor torque stays near 0, as on the real arm; off the stop the motor holds it
    sim = Simulator(free_udp_port())
    sim.motors.set_enabled(True)
    for _ in range(1000):
        sim.step()
    tau = sim.motors.reported_torque()
    bias = sim.data.qfrc_bias[sim.motors.dofs[:6]]
    assert bias[2] < -5.0 and bias[1] < -0.5, bias
    assert abs(tau[1]) < 0.5 and abs(tau[2]) < 1.0, tau
    q = sim.motors.idx.motor_positions(sim.data)
    assert 0.0 <= q[2] < 0.005 and 0.0 <= q[1] < 0.005, q
    sim.motors.q_t[2] = -0.3
    for _ in range(1500):
        sim.step()
    q = sim.motors.idx.motor_positions(sim.data)
    tau = sim.motors.reported_torque()
    assert abs(q[2] + 0.3) < 0.01 and tau[2] < -3.0, (q, tau)
    sim.sock.close()
