"""Tests for SO-101 6-DOF simulation in MuJoCo (so_arm101_scene.xml, server, camera)."""
import json
import socket
import numpy as np
import pytest
import mujoco

from rebot_sim.model import load_model, Indexer, SO101_JOINTS
from rebot_sim.server import Simulator, PROTOCOL_VERSION


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def cmd(seq=1, enable=True, targets=None) -> bytes:
    msg = {"v": PROTOCOL_VERSION, "type": "cmd", "seq": seq, "enable": enable}
    if targets is not None:
        msg["targets"] = targets
    return json.dumps(msg).encode()


def pos(q: float, vlim: float = 1.0) -> dict:
    return {"mode": "pos_vel", "q": q, "vlim": vlim}


def test_load_so101_model():
    model = load_model(robot="so-arm101")
    assert model is not None
    assert model.nu == 6
    # Verify joint names
    joints = [model.joint(i).name for i in range(model.njnt)]
    for j in SO101_JOINTS:
        assert j in joints
    # Verify wrist_roll range is [-2.74, 2.74]
    j_wr = model.joint("wrist_roll")
    assert np.isclose(model.jnt_range[j_wr.id, 0], -2.74, atol=1e-2)
    assert np.isclose(model.jnt_range[j_wr.id, 1], 2.74, atol=1e-2)


def test_so101_indexer():
    model = load_model(robot="so-arm101")
    data = mujoco.MjData(model)
    idx = Indexer(model)
    assert idx.robot == "so-arm101"
    assert idx.dof == 6

    q_test = np.array([0.1, -0.2, 0.3, -0.4, 0.5, 0.8])
    idx.set_motor_positions(data, q_test)
    q_read = idx.motor_positions(data)
    assert np.allclose(q_read, q_test, atol=1e-5)

    data.qvel[idx.qvel] = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    dq_read = idx.motor_velocities(data)
    assert np.allclose(dq_read, [0.2, 0.3, 0.4, 0.5, 0.6, 0.7], atol=1e-5)


def test_so101_server_udp_and_loads():
    port = free_udp_port()
    sim = Simulator(port, robot="so-arm101")
    try:
        assert sim.motors.dof == 6
        assert sim.motors.robot == "so-arm101"

        # 6 valid targets
        targets_6 = [pos(0.05 * i) for i in range(6)]
        sim.handle(cmd(seq=1, enable=True, targets=targets_6), ("127.0.0.1", 12345))
        assert sim.ack == 1
        assert sim.motors.enabled
        assert len(sim.motors.q_t) == 6

        # Mismatched 7 targets rejected
        rejected_before = sim.rejected
        sim.handle(cmd(seq=2, targets=targets_6 + [pos(0.1)]), ("127.0.0.1", 12345))
        assert sim.rejected == rejected_before + 1

        # Step simulation and check state datagram
        for _ in range(50):
            sim.step()

        raw_state = sim.state_datagram()
        state = json.loads(raw_state)
        assert state["type"] == "state"
        assert len(state["q"]) == 6
        assert len(state["dq"]) == 6
        assert len(state["tau"]) == 6
        assert "loads" in state
        assert len(state["loads"]) == 6

        # Check loads and torques are valid finite numbers
        for ld in state["loads"]:
            assert 0.0 <= ld <= 1.0
            assert np.isfinite(ld)
        for tau in state["tau"]:
            assert np.isfinite(tau)
    finally:
        sim.sock.close()


def test_so101_hand_cam_rendering():
    model = load_model(robot="so-arm101")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Check hand_cam camera exists
    cam_names = [model.camera(i).name for i in range(model.ncam)]
    assert "hand_cam" in cam_names or "wrist" in cam_names

    cam_name = "hand_cam" if "hand_cam" in cam_names else "wrist"
    renderer = mujoco.Renderer(model, 360, 640)
    renderer.update_scene(data, camera=cam_name)
    img = renderer.render()

    assert img.shape == (360, 640, 3)
    # The image must not be completely black (tabletop and background are visible)
    assert img.mean() > 20.0
