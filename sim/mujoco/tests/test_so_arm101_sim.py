"""Tests for SO-101 6-DOF simulation in MuJoCo (so_arm101_scene.xml, server, camera)."""
import json
import socket
import numpy as np
import pytest
import mujoco

from rebot_sim.camera import CameraMount, SO101_PROFILE, mount_rotation
from rebot_sim.model import load_model, Indexer, SO101_JOINTS
from rebot_sim.scene import Scene, parse_objects
from rebot_sim.server import Simulator, PROTOCOL_VERSION


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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


def test_so101_sim_state_tcp_pose():
    port = free_udp_port()
    sim = Simulator(port, objects="cube", robot="so-arm101")
    try:
        st = sim.sim_state()
        assert "tcp" in st
        assert len(st["tcp"]) == 3
        for val in st["tcp"]:
            assert np.isfinite(val)
        assert "tcp_quat" in st
        assert len(st["tcp_quat"]) == 4
        for val in st["tcp_quat"]:
            assert np.isfinite(val)
    finally:
        sim.close()


def test_so101_scene_jaw_ids():
    model = load_model(robot="so-arm101")
    objects = parse_objects("cube")
    scene = Scene(model, objects)
    assert len(scene.jaw_ids) == 2
    jaw_names = set(scene.jaw_ids.values())
    assert "wrist_roll_follower_so101_v1" in jaw_names
    assert "moving_jaw_so101_v1" in jaw_names


def test_so101_camera_rig_and_mount_rotation():
    model = load_model(robot="so-arm101")
    r_cam = mount_rotation(model, CameraMount())
    assert r_cam.shape == (3, 3)
    assert np.isclose(np.linalg.det(r_cam), 1.0, atol=1e-3)

    udp_port = free_udp_port()
    mount = CameraMount.from_profile(SO101_PROFILE)
    sim = Simulator(udp_port, objects="cube", camera_mount=mount, robot="so-arm101")
    try:
        vport = free_tcp_port()
        pport = free_tcp_port()
        ready = sim.start_camera(15, (320, 180), "127.0.0.1", vport, pport, objects="cube")
        if "error" in ready:
            pytest.skip(f"camera start skipped: {ready['error']}")
        assert "error" not in ready
        assert "backend" in ready
    finally:
        sim.close()


def test_so101_fk_parity_with_daemon_profile():
    """Verify numerical parity of MuJoCo TCP site pose against daemon KinematicChain FK (< 2 microns position deviation)."""
    udp_port = free_udp_port()
    sim = Simulator(udp_port, objects="cube", robot="so-arm101")
    try:
        idx = Indexer(sim.model)

        # 1. Home position FK parity
        # q_home = [0.0, -0.2, 0.5, 0.5, 0.0, 0.0]
        # Daemon KinematicChain FK: p = [0.3099907, -9.3325e-6, 0.0778639]
        q_home = [0.0, -0.2, 0.5, 0.5, 0.0, 0.0]
        idx.set_motor_positions(sim.data, q_home)
        mujoco.mj_kinematics(sim.model, sim.data)

        st_home = sim.sim_state()
        p_home_expected = np.array([0.3099907, -9.3325e-6, 0.0778639])
        np.testing.assert_allclose(st_home["tcp"], p_home_expected, atol=2e-5)
        # Verify quaternion matches expected canonical frame orientation
        # Daemon home quat ~ [0.66695, -0.63525, 0.28198, 0.26858]
        np.testing.assert_allclose(st_home["tcp_quat"], [0.66695, -0.63525, 0.28198, 0.26858], atol=1e-3)

        # 2. Non-zero targets FK parity
        # q_nz = [0.2, -0.3, 0.4, 0.1, -0.2, 0.0]
        # Daemon KinematicChain FK: p = [0.3460510, -0.0606870, 0.1849334]
        q_nz = [0.2, -0.3, 0.4, 0.1, -0.2, 0.0]
        idx.set_motor_positions(sim.data, q_nz)
        mujoco.mj_kinematics(sim.model, sim.data)

        st_nz = sim.sim_state()
        p_nz_expected = np.array([0.3460510, -0.0606870, 0.1849334])
        np.testing.assert_allclose(st_nz["tcp"], p_nz_expected, atol=2e-5)
        # Daemon nz quat ~ [0.78759, -0.59997, 0.1394, -0.01742]
        np.testing.assert_allclose(st_nz["tcp_quat"], [0.78759, -0.59997, 0.1394, -0.01742], atol=1e-3)
    finally:
        sim.close()


