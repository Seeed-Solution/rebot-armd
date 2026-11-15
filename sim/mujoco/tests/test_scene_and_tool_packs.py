"""Tests for composable scene packs, custom end-effectors, virtual IO, and hot-swap."""

from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import mujoco
import numpy as np
import pytest
from rebot_sim.camera import CameraMount, detections, render_segmentation
from rebot_sim.env import SimEnv
from rebot_sim.model import Indexer, load_model
from rebot_sim.pack import PackScene, load_scene_pack, load_tool_pack
from rebot_sim.perception import Perception, PerceptionServer
from rebot_sim.server import Motors, Simulator


def _free_port(kind: int = socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _http_json(
    url: str,
    method: str = "GET",
    body: dict | None = None,
    headers: dict | None = None,
) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    hdr = {"Content-Type": "application/json"} if data is not None else {}
    if headers is not None:
        hdr.update(headers)
    req = urllib.request.Request(url, data=data, headers=hdr, method=method)
    try:
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            return exc.code, json.loads(exc.read().decode("utf-8"))
        except Exception:
            return exc.code, {"error": exc.reason}


def test_builtin_coffee_kiosk_and_wide_cup_gripper_compile():
    scene_pack = load_scene_pack("coffee_kiosk")
    tool_pack = load_tool_pack("wide_cup_gripper")
    assert scene_pack is not None
    assert tool_pack.id == "wide_cup_gripper"

    model = load_model(
        camera=CameraMount(), robot="b601dm", scene_pack=scene_pack, tool_pack=tool_pack
    )
    data = mujoco.MjData(model)
    idx = Indexer(model, tool_pack=tool_pack)
    assert idx.dof == 7
    assert idx.joints[-1] == "cup_jaw_left"
    assert idx.right_qpos is not None

    q_cmd = np.array([0.1, -0.2, 0.3, 0.0, 0.2, 0.0, -4.0])
    idx.set_motor_positions(data, q_cmd)
    mujoco.mj_forward(model, data)
    q_got = idx.motor_positions(data)
    assert np.allclose(q_got, q_cmd, atol=1e-5)
    assert np.isclose(data.qpos[idx.qpos[6]], data.qpos[idx.right_qpos], atol=1e-6)

    motors = Motors(model, data, tool_pack=tool_pack)
    assert motors.dof == 7
    ps = PackScene(model, scene_pack, tool_pack)
    state = ps.reset(data, seed=42)
    assert "cup" in state and "pod" in state
    assert not np.allclose(state["cup"]["pos"][:2], [0.27, -0.10])


def test_local_custom_user_scene_and_tool_directories(tmp_path: Path):
    scene_dir = tmp_path / "pcb_station"
    scene_dir.mkdir()
    (scene_dir / "world.xml").write_text(
        """<mujoco model="pcb_world">
  <worldbody>
    <body name="pcb_board" pos="0.26 0.02 0.01">
      <freejoint name="pcb_joint"/>
      <inertial pos="0 0 0" mass="0.04" diaginertia="1e-5 1e-5 1e-5"/>
      <geom name="pcb_geom" type="box" size="0.03 0.02 0.005" rgba="0.1 0.6 0.2 1"/>
    </body>
  </worldbody>
</mujoco>""",
        encoding="utf-8",
    )
    (scene_dir / "scene.json").write_text(
        json.dumps(
            {
                "id": "pcb_station",
                "name": "Local PCB Station",
                "world_xml": "world.xml",
                "objects": [
                    {
                        "name": "pcb_board",
                        "body": "pcb_board",
                        "geom": "pcb_geom",
                        "class_name": "pcb",
                        "default_pos": [0.26, 0.02, 0.01],
                        "jitter_xy": 0.01,
                    }
                ],
                "zones": {
                    "tray": {
                        "center": [0.25, -0.12, 0.01],
                        "half_extents": [0.05, 0.05, 0.03],
                    }
                },
                "task_stages": [
                    {
                        "id": "placed",
                        "type": "object_in_zone",
                        "object": "pcb_board",
                        "zone": "tray",
                        "latch": True,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    tool_dir = tmp_path / "my_gripper"
    tool_dir.mkdir()
    (tool_dir / "tool.xml").write_text(
        """<mujoco model="user_gripper">
  <worldbody>
    <body name="user_finger_l" pos="0 0 0">
      <joint name="user_finger_l" type="slide" axis="0 1 0" range="0 0.05"/>
      <inertial pos="-0.02 0.01 0" mass="0.02" diaginertia="1e-5 1e-5 1e-5"/>
      <geom name="user_pad_l" type="box" pos="-0.015 0.007 0" size="0.015 0.007 0.012"/>
    </body>
  </worldbody>
  <actuator>
    <position name="m7_user" joint="user_finger_l" kp="50"/>
  </actuator>
</mujoco>""",
        encoding="utf-8",
    )
    (tool_dir / "tool.json").write_text(
        json.dumps(
            {
                "id": "my_gripper",
                "name": "User Custom Gripper",
                "kind": "gripper",
                "tool_xml": "tool.xml",
                "driven_joint": "user_finger_l",
                "actuator": "m7_user",
                "motor_to_joint_scale": -0.008,
                "finger_geoms": ["user_pad_l"],
            }
        ),
        encoding="utf-8",
    )

    # Load from explicit directory manifests
    sp = load_scene_pack(scene_dir)
    tp = load_tool_pack(tool_dir)
    assert sp is not None and sp.id == "pcb_station"
    assert tp.id == "my_gripper"
    model = load_model(scene_pack=sp, tool_pack=tp)
    data = mujoco.MjData(model)
    ps = PackScene(model, sp, tp)
    ps.reset(data, seed=None)
    assert "pcb_board" in ps.state(data)

    # Also test standalone raw XML discovery without JSON
    raw_sp = load_scene_pack(scene_dir / "world.xml")
    assert raw_sp is not None and raw_sp.objects[0]["body"] == "pcb_board"
    raw_tp = load_tool_pack(tool_dir / "tool.xml")
    assert raw_tp.driven_joint == "user_finger_l"


def test_suction_tool_virtual_io_weld_attach_and_release():
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", tool="suction")
    try:
        # Move arm to a lowered extended pose, then place cube ahead of the suction nozzle at TCP
        sim.motors.idx.set_motor_positions(
            sim.data, [0.0, 0.4, 0.4, 0.0, 0.0, 0.0, 0.0]
        )
        mujoco.mj_forward(sim.model, sim.data)
        tcp = sim.data.site("tcp").xpos.copy()
        qadr = sim.scene._free_qpos[0]
        sim.data.qpos[qadr : qadr + 3] = [tcp[0] + 0.025, tcp[1], tcp[2]]
        sim.data.qpos[qadr + 3 : qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(sim.model, sim.data)

        io_res = sim.set_io({"vacuum": True})
        assert io_res["io"]["vacuum"] is True
        assert io_res["io"]["vacuum_engaged"] is True

        # Enable & command shoulder upward; welded cube must track TCP up cleanly
        sim.motors.set_enabled(True)
        q_cur = sim.motors.idx.motor_positions(sim.data)
        q_cur[1] -= 0.35
        sim.motors.q_t = q_cur
        for _ in range(150):
            sim.step()

        tcp_after = sim.data.site("tcp").xpos.copy()
        cube_z_held = sim.sim_state()["objects"]["cube"]["pos"][2]
        assert tcp_after[2] > tcp[2] + 0.05
        assert abs(cube_z_held - tcp_after[2]) < 0.01

        sim.set_io({"vacuum": False})
        assert sim.get_io()["io"]["vacuum_engaged"] is False
        for _ in range(250):
            sim.step()
        cube_z_released = sim.sim_state()["objects"]["cube"]["pos"][2]
        assert cube_z_released < cube_z_held - 0.05
    finally:
        sim.close()


def test_coffee_kiosk_task_evaluator_and_reset():
    scene_pack = load_scene_pack("coffee_kiosk")
    model = load_model(scene_pack=scene_pack)
    data = mujoco.MjData(model)
    ps = PackScene(model, scene_pack, load_tool_pack("default"))
    ps.reset(data, seed=None)

    cup_qadr = ps._free_qpos[0]
    btn_qadr = model.jnt_qposadr[model.joint("brew_btn_joint").id]
    led_gid = model.geom("brew_led").id
    initial_led = model.geom_rgba[led_gid].copy()

    t0 = ps.task_state(data)
    assert t0["completed"] is False
    assert t0["current_stage"] == "cup_grasped"

    # Stage 1: cup lifted next to TCP
    tcp = data.site("tcp").xpos.copy()
    data.qpos[cup_qadr : cup_qadr + 3] = tcp
    mujoco.mj_forward(model, data)
    assert ps.task_state(data)["stages"]["cup_grasped"] is True

    # Stage 2: cup placed on drip_tray
    data.qpos[cup_qadr : cup_qadr + 3] = [0.31, 0.10, 0.043]
    mujoco.mj_forward(model, data)
    assert ps.task_state(data)["stages"]["cup_on_tray"] is True

    # Stage 3: brew button pressed
    data.qpos[btn_qadr] = -0.009
    mujoco.mj_forward(model, data)
    t3 = ps.task_state(data)
    assert t3["stages"]["brew_pressed"] is True
    assert not np.allclose(model.geom_rgba[led_gid], initial_led)

    # Stage 4: cup placed on pickup counter
    data.qpos[cup_qadr : cup_qadr + 3] = [0.25, -0.16, 0.035]
    mujoco.mj_forward(model, data)
    t4 = ps.task_state(data)
    assert t4["completed"] is True
    assert t4["score"] == 1.0

    # Reset clears latched stages & restores LED color
    ps.reset(data, seed=1)
    t_reset = ps.task_state(data)
    assert t_reset["completed"] is False
    assert t_reset["score"] == 0.0
    assert np.allclose(model.geom_rgba[led_gid], initial_led)


def test_http_scene_hot_swap_and_invalid_rollback():
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube")
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        code, info = _http_json(f"{base}/sim/scene")
        assert code == 200
        assert info["scene"] == "default"
        assert info["scene_pack"]["id"] == "default"

        code, swapped = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk", "tool": "wide_cup_gripper", "seed": 7},
        )
        assert code == 200
        assert swapped["scene"] == "coffee_kiosk"
        assert swapped["tool"] == "wide_cup_gripper"
        assert "cup" in swapped["objects"]
        assert "task" in swapped and swapped["task"]["current_stage"] == "cup_grasped"

        # Invalid scene must return 400 and leave coffee_kiosk running intact
        code, err = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "/nonexistent/scene/pack"},
        )
        assert code == 400 and "error" in err
        code, still = _http_json(f"{base}/sim/scene")
        assert code == 200 and still["scene"] == "coffee_kiosk"
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_live_camera_subprocess_hot_swap():
    sim = Simulator(
        _free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount()
    )
    vport, pport = _free_port(), _free_port()
    ready = sim.start_camera(10, (320, 180), "127.0.0.1", vport, pport)
    if "error" in ready:
        sim.close()
        pytest.skip(ready["error"])
    runner = threading.Thread(target=sim.run, daemon=True)
    runner.start()
    try:
        base = f"http://127.0.0.1:{pport}"
        code, res = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk", "tool": "wide_cup_gripper", "seed": 3},
        )
        assert code == 200, res
        assert res["scene"] == "coffee_kiosk"
        assert res["tool"] == "wide_cup_gripper"
        time.sleep(0.25)
        code, st = _http_json(f"{base}/sim/state")
        assert code == 200
        assert st["scene"] == "coffee_kiosk"
        assert "cup" in st["objects"]
        assert st["camera"]["seg_failures"] == 0
    finally:
        sim.stopping = True
        runner.join(2)
        sim.close()


def test_segmentation_handles_multi_geom_cup():
    scene_pack = load_scene_pack("coffee_kiosk")
    model = load_model(
        camera=CameraMount(),
        scene_pack=scene_pack,
        tool_pack=load_tool_pack("wide_cup_gripper"),
    )
    data = mujoco.MjData(model)
    ps = PackScene(model, scene_pack, load_tool_pack("wide_cup_gripper"))
    ps.reset(data, seed=None)
    # Position arm looking straight down at cup at (0.27, -0.10)
    data.qpos[0:6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    try:
        r = mujoco.Renderer(model, 180, 320)
    except Exception as exc:
        pytest.skip(f"MuJoCo GL unavailable: {exc}")
    try:
        model.vis.global_.offheight = max(model.vis.global_.offheight, 181)
        opt = mujoco.MjvOption()
        r.update_scene(data, camera="scene", scene_option=opt)
        ids = render_segmentation(r)
        dets = detections(ids, ps, min_pixels=5)
        assert any(d["name"] == "cup" for d in dets)
    finally:
        r.close()


def test_pcb_suction_scene_compiles_with_suction_tool():
    sp = load_scene_pack("pcb_suction")
    tp = load_tool_pack("suction")
    assert sp is not None and sp.id == "pcb_suction"
    assert sp.default_tool == "suction"
    assert "infeed_tray" in sp.zones
    assert "test_fixture" in sp.zones
    assert "outfeed_bin" in sp.zones
    assert tp.kind == "suction"

    model = load_model(robot="b601dm", scene_pack=sp, tool_pack=tp)
    data = mujoco.MjData(model)
    ps = PackScene(model, sp, tp)
    state = ps.reset(data, seed=42)
    assert "pcb" in state
    assert len(state["pcb"]["pos"]) == 3
    task = ps.task_state(data)
    assert task is not None
    assert task["stages"] == {
        "pcb_picked": False,
        "pcb_in_fixture": False,
        "pcb_in_outfeed": False,
    }


def test_sim_env_fast_training_loop_timing():
    env = SimEnv(scene="pcb_suction", tool="suction")
    try:
        obs, info = env.reset(seed=42)
        assert obs["state"].shape == (7,)
        assert obs["velocity"].shape == (7,)
        assert "pcb" in obs["objects"]
        assert "io" in obs
        assert info["task"]["scene"] == "pcb_suction"

        # Execute 200 steps (4.0s sim time at control_dt=0.02) commanding joint position targets
        target = np.array([0.2, -0.45, -0.35, 0.0, 0.0, 0.0, 0.0])
        t0 = time.perf_counter()
        for _ in range(200):
            obs, _reward, _done, _info = env.step(target)
        elapsed = time.perf_counter() - t0

        # Non-realtime simulation must execute way faster than real time (e.g. < 2.0s wall clock)
        # and POS_VEL tracking should move shoulder joint (index 1, range [-2.1944, 0]) toward target=-0.45 rad.
        assert elapsed < 2.0
        assert env.data.time >= 3.99
        assert abs(float(obs["state"][1]) - (-0.45)) < 0.05
    finally:
        env.close()


def test_sim_env_suction_stages_pcb_picked_fixture_and_outfeed():
    env = SimEnv(scene="pcb_suction", tool="suction")
    try:
        obs, info = env.reset(seed=42)
        assert info["task"]["stages"]["pcb_picked"] is False
        assert info["task"]["stages"]["pcb_in_fixture"] is False
        assert info["task"]["stages"]["pcb_in_outfeed"] is False
        assert not info["task"]["completed"]

        # Position PCB workpiece right beneath the suction nozzle at TCP
        tcp = env.data.site("tcp").xpos.copy()
        qadr = env.pack_scene._free_qpos[0]
        env.data.qpos[qadr : qadr + 3] = [tcp[0], tcp[1], tcp[2] - 0.005]
        env.data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(env.model, env.data)

        # Engage vacuum suction: should weld and set vacuum_engaged
        io_res = env.set_io({"vacuum": True})
        assert io_res["vacuum"] is True
        assert io_res["vacuum_engaged"] is True

        # Lift TCP up above 0.055 m
        env.indexer.set_motor_positions(env.data, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        mujoco.mj_forward(env.model, env.data)
        obs, reward, done, info = env.step(np.zeros(env.model.nu))
        assert info["task"]["stages"]["pcb_picked"] is True
        assert not info["task"]["stages"]["pcb_in_fixture"]

        # Place PCB onto test fixture zone: release vacuum so PCB rests on fixture
        env.set_io({"vacuum": False})
        env.data.qpos[qadr : qadr + 3] = [0.27, 0.01, 0.024]
        env.data.qvel[qadr : qadr + 6] = 0.0
        mujoco.mj_forward(env.model, env.data)
        obs, reward, done, info = env.step(np.zeros(env.model.nu))
        assert info["task"]["stages"]["pcb_in_fixture"] is True
        led_id = env.model.geom("test_pass_led").id
        assert np.allclose(env.model.geom_rgba[led_id], [0.15, 0.9, 0.35, 1.0])

        # Transfer PCB into outfeed bin zone: should latch pcb_in_outfeed and mark task complete
        env.data.qpos[qadr : qadr + 3] = [0.23, 0.15, 0.022]
        env.data.qvel[qadr : qadr + 6] = 0.0
        mujoco.mj_forward(env.model, env.data)
        _obs, reward, done, info = env.step(np.zeros(env.model.nu))
        assert info["task"]["stages"]["pcb_in_outfeed"] is True
        assert info["task"]["completed"] is True
        assert done is True
        assert reward > 1.0
    finally:
        env.close()


def test_http_scene_slug_validation_origin_and_content_type():
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube")
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        # 1. Path traversal & arbitrary filesystem paths rejected with 400
        for bad_slug in (
            "/etc/passwd",
            "../relative/pack",
            "/tmp/nonexistent",
            "bad/slash",
            "with space",
            "-lead-dash",
            "_lead_under",
            "a" * 65,
        ):
            code, res = _http_json(
                f"{base}/sim/scene", method="POST", body={"scene": bad_slug}
            )
            assert code == 400, (
                f"Expected 400 for bad scene {bad_slug!r}, got {code}: {res}"
            )
            assert "error" in res

        for bad_tool in ("/etc/shadow", "../tool.xml", "/tmp/tool.xml", "tool/slash"):
            code, res = _http_json(
                f"{base}/sim/scene", method="POST", body={"tool": bad_tool}
            )
            assert code == 400, (
                f"Expected 400 for bad tool {bad_tool!r}, got {code}: {res}"
            )
            assert "error" in res

        # 2. Content-Type must be application/json on mutating POST
        code, res = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk"},
            headers={"Content-Type": "text/plain"},
        )
        assert code == 415, f"Expected 415 for text/plain, got {code}: {res}"

        code, res = _http_json(
            f"{base}/sim/io",
            method="POST",
            body={"vacuum": True},
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )
        assert code == 415, f"Expected 415 for x-www-form-urlencoded, got {code}: {res}"

        # 3. Foreign Origin rejected with 403; same origin / localhost accepted
        code, res = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk"},
            headers={"Origin": "http://evil.invalid"},
        )
        assert code == 403, f"Expected 403 for evil Origin, got {code}: {res}"

        code, res = _http_json(
            f"{base}/sim/io",
            method="POST",
            body={"test": True},
            headers={"Origin": "http://attacker.com:8090"},
        )
        assert code == 403, f"Expected 403 for attacker Origin, got {code}: {res}"

        # Valid Origin matching Host or localhost loopback is permitted
        code, res = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk", "tool": "wide_cup_gripper"},
            headers={"Origin": f"http://127.0.0.1:{port}"},
        )
        assert code == 200, f"Expected 200 for matching Origin, got {code}: {res}"
        assert res["scene"] == "coffee_kiosk"

        code, res = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "default", "tool": "default"},
            headers={"Origin": f"http://localhost:{port}"},
        )
        assert code == 200, f"Expected 200 for localhost Origin, got {code}: {res}"
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_composed_8_actuator_scene_and_tool_steps_cleanly():
    spec = mujoco.MjSpec.from_file("models/rebot_b601dm_scene.xml")
    extra_body = spec.worldbody.add_body(name="turntable_stage")
    extra_body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.05, 0.01, 0.0], mass=0.2
    )
    extra_body.add_joint(
        name="turntable_joint", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1]
    )
    extra_act = spec.add_actuator(
        name="turntable_motor", trntype=mujoco.mjtTrn.mjTRN_JOINT
    )
    extra_act.target = "turntable_joint"
    model = spec.compile()
    data = mujoco.MjData(model)
    assert model.nu == 8

    motors = Motors(model, data)
    assert motors.dof == 7
    assert len(motors.actuator_ids) == 7

    # Controller.apply() and mj_step() must run cleanly without shape mismatch ValueError
    motors.set_enabled(True)
    motors.apply()
    for _ in range(20):
        mujoco.mj_step(model, data)
        motors.apply()

    targets = [{"mode": "pos_vel", "q": 0.05 * i, "vlim": 1.5} for i in range(7)]
    assert motors.set_targets(targets) is True
    motors.apply()
    for _ in range(20):
        mujoco.mj_step(model, data)
        motors.apply()


def test_object_in_zone_not_latching_while_grasped():
    from rebot_sim.pack import ScenePack, TaskEvaluator

    sp = ScenePack(
        id="zone_test",
        name="Zone Test Scene",
        world_xml_path=Path("models/rebot_b601dm_scene.xml"),
        zones={
            "assembly_zone": {
                "center": [0.30, 0.0, 0.02],
                "half_extents": [0.05, 0.05, 0.04],
            }
        },
        task_stages=[
            {
                "id": "part_placed",
                "type": "object_in_zone",
                "object": "part_a",
                "zone": "assembly_zone",
                "latch": True,
            },
            {
                "id": "part_held_in_zone",
                "type": "object_in_zone",
                "object": "part_b",
                "zone": "assembly_zone",
                "allow_grasped": True,
                "latch": True,
            },
        ],
    )
    model = mujoco.MjModel.from_xml_path("models/rebot_b601dm_scene.xml")
    data = mujoco.MjData(model)

    class MockPackScene:
        def __init__(self, pack):
            self.pack = pack

    mock_scene = MockPackScene(sp)
    evaluator = TaskEvaluator(model, sp, mock_scene)

    # 1. Part inside zone geometry, but currently grasped (held by gripper/suction)
    obj_state = {
        "part_a": {"pos": [0.30, 0.0, 0.02], "grasped": True},
        "part_b": {"pos": [0.30, 0.0, 0.02], "grasped": True},
    }
    res1 = evaluator.evaluate(data, obj_state, {})
    # part_placed must NOT pass while grasped (default allow_grasped=False)
    assert res1["stages"]["part_placed"] is False
    # part_held_in_zone DOES pass because allow_grasped=True
    assert res1["stages"]["part_held_in_zone"] is True
    assert res1["completed"] is False

    # 2. Part A is still inside zone, now released (grasped=False)
    obj_state["part_a"]["grasped"] = False
    res2 = evaluator.evaluate(data, obj_state, {})
    assert res2["stages"]["part_placed"] is True
    assert res2["completed"] is True

    # 3. Part A moved out of zone after latching: remains completed because latch=True
    obj_state["part_a"]["pos"] = [0.60, 0.50, 0.02]
    res3 = evaluator.evaluate(data, obj_state, {})
    assert res3["stages"]["part_placed"] is True


def test_camera_rig_reload_cancellation():
    import multiprocessing as mp
    import unittest.mock as mock

    from rebot_sim.perception import CameraRig, SharedState

    lock = mp.get_context("spawn").Lock()
    st = SharedState(10, 10, lock=lock)
    calls = []

    class DummyLink:
        def call(self, req, timeout=None):
            calls.append(req[0])
            if req[0] == "scene_prepare":
                return {
                    "_shm": {"nq": 10, "nv": 10, "name": st.shm.name},
                    "_scene_ref": "coffee_kiosk",
                    "_tool_ref": "wide_cup_gripper",
                }
            if req[0] == "scene_abort":
                return {"ok": True}
            if req[0] == "scene_commit":
                return {"scene": "coffee_kiosk"}
            return {}

    cfg = {
        "fps": 10,
        "size": [320, 180],
        "mount": ("wrist",),
        "scene_stream": False,
        "noise": {},
    }
    rig = CameraRig(cfg, st, DummyLink())
    try:
        old_model = rig.model
        old_data = rig.data
        old_scene = rig.scene

        # 1. Test cancelled reload in _apply_reload():
        # When cancelled is True, _apply_reload() must NOT swap model/data/scene.
        dummy_req = {
            "done": threading.Event(),
            "shm_info": {"nq": 10, "nv": 10, "name": st.shm.name},
            "scene_ref": "coffee_kiosk",
            "tool_ref": "wide_cup_gripper",
            "objects_str": "cube",
            "err_box": {},
            "cancelled": True,
        }
        rig._apply_reload(dummy_req)
        assert rig.model is old_model
        assert rig.data is old_data
        assert rig.scene is old_scene

        # 2. Test timeout in load_scene():
        # If camera thread does not finish within timeout, load_scene marks cancelled and calls scene_abort.
        orig_event = threading.Event

        def mock_event():
            ev = orig_event()
            ev.wait = lambda timeout=None: False
            return ev

        with mock.patch("threading.Event", side_effect=mock_event):
            res = rig.load_scene({"scene": "coffee_kiosk"})
            assert "error" in res
            assert "camera reload timed out" in res["error"]
            assert "scene_abort" in calls
            assert rig._reload_req is None

        # 3. Test edge case: done.is_set() is True right as wait returns
        calls.clear()

        def mock_event_late():
            ev = orig_event()

            def wait_late(timeout=None):
                ev.set()
                return False

            ev.wait = wait_late
            return ev

        with mock.patch("threading.Event", side_effect=mock_event_late):
            res = rig.load_scene({"scene": "coffee_kiosk"})
            assert "error" not in res
            assert "scene_commit" in calls
    finally:
        st.close()
