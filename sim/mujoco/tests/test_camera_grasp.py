"""End to end: tap-to-grasp in simulation. rebot-sim --camera serves the wrist stream and the
perception API; armd proxies it (ARMD_DETECTOR_URL) and runs the grasp macro, whose centring
loop triggers ground-truth detections on fresh frames.

armd gets a copy of the kit profile with camera_mount.offset_m set to the simulator's lens
centre (the kit profile leaves it null, unmeasured); the grasp request overrides the setup's
grip pixel and focal length with the simulator's values (GET /sim/camera).

The v8 cases run the simulator and armd with the v8 holder (rot 90, tilt -38): armd's
grasp then acquires camera-down (`acquire_camera_down: auto`) and re-orients the tool
vertical before descending.
"""
from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import time
import urllib.request

import mujoco
import numpy as np
import pytest

from rebot_sim.camera import CameraMount, axis_r, mount_rotation
from rebot_sim.model import load_model
from test_armd_sim import PROFILE, TOOL_DOWN, Client, armd_binary, free_port


def http(url: str, body: dict | None = None, timeout: float = 10.0) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def wait_http(url: str, proc: subprocess.Popen, log, timeout: float = 30.0) -> None:
    deadline = time.time() + timeout
    while True:
        try:
            urllib.request.urlopen(url, timeout=1)
            return
        except OSError:
            if time.time() > deadline or proc.poll() is not None:
                raise RuntimeError(log.read_text())
            time.sleep(0.2)


# (simulator scene arguments, camera mount (rot_deg, tilt_deg) or None for the kit profile's v5
# mount). v5: optical axis along the tool, look-down acquisition onto the grip pixel. v8 (38 deg
# toward the jaws): with the tool pointing down the table point under the jaws leaves the image
# above about 0.06 m, so armd acquires camera-down.
# v5_noise / v8_noise: detector-like errors (jitter, size jitter, drops, 100 ms latency).
NOISE = ["--det-jitter-px", "3", "--det-size-jitter", "0.05", "--det-drop-rate", "0.1", "--det-latency-ms", "100",
         "--det-noise-seed", "1"]
# axis_*: lens 45 mm off the tool axis (AXIS_LENS, end_link) instead of the v8 holder's 66 mm,
# just outside the gripper (the kit check points reach 34.7 mm), so the object under the jaws
# stays inside the image at the approach height and armd's height probe runs. With the 66 mm
# lens centre the box reaches the image border (u about 0.04) and the probe falls back with
# height_reason "border"; a lens on the tool axis sees only the gripper.
AXIS_LENS = ["--camera-offset=-0.0731,0.0005,0.045"]
CASES = {
    "default": ([], None),
    "seed3_cube_cup": (["--objects", "cube,cup", "--seed", "3"], None),
    "v8_default": ([], (90.0, -38.0)),
    "v8_seed3_cube_cup": (["--objects", "cube,cup", "--seed", "3"], (90.0, -38.0)),
    "v5_noise": (NOISE, None),
    "v8_noise": (NOISE, (90.0, -38.0)),
    "axis_cube": (AXIS_LENS, None),
    "axis_cup": ([*AXIS_LENS, "--objects", "cup"], None),
}
# object the case grasps (default: the cube) and its height (scene.py KINDS)
TARGETS = {"axis_cup": "cup"}
HEIGHTS_M = {"cube": 0.04, "cup": 0.07, "block": 0.04}


def view_pose(cam: dict, cube_xy, z: float, miss_m=(0.0, 0.0)):
    """TCP position and orientation (quat w, x, y, z) of a view with the tool pitched half way
    from vertical toward camera-down, the image centre ray `miss_m` from the cube's centre."""
    m = cam["camera_mount"]
    mount = CameraMount(m["flip"], m["rot_deg"], m["tilt_deg"], tuple(m["offset_m"]))
    rec = mount_rotation(load_model(devarm_dir=""), mount)
    r_tool = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # rpy (0, pi/2, 0)
    z_cam = r_tool @ rec[:, 2]
    axis = np.cross(z_cam, [0.0, 0.0, -1.0])
    angle = math.atan2(np.linalg.norm(axis), -z_cam[2])
    r = axis_r(axis / np.linalg.norm(axis), angle / 2) @ r_tool if angle > 1e-6 else r_tool
    off, d = r @ mount.offset, r @ rec[:, 2]
    t = (0.02 - (z + off[2])) / d[2]  # cube centre plane
    p = [cube_xy[0] + miss_m[0] - off[0] - d[0] * t, cube_xy[1] + miss_m[1] - off[1] - d[1] * t, z]
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, r.flatten())
    return p, [float(v) for v in q]


@pytest.fixture(params=list(CASES))
def cam_rig(request, tmp_path):
    extra, v8 = CASES[request.param]
    if v8:
        extra = [*extra, "--camera-rot-deg", str(v8[0]), "--camera-tilt-deg", str(v8[1])]
    udp, http_port, video, percep = free_port(socket.SOCK_DGRAM), free_port(), free_port(), free_port()
    sim = subprocess.Popen([sys.executable, "-m", "rebot_sim.server", "--port", str(udp), "--headless", "--stats", "2",
                            "--camera", "--video-port", str(video), "--perception-port", str(percep), *extra],
                           stdout=open(tmp_path / "sim.log", "w"), stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{percep}"
    wait_http(base + "/sim/camera", sim, tmp_path / "sim.log")
    cam = http(base + "/sim/camera")
    profile = json.loads(PROFILE.read_text())
    profile["camera_mount"]["offset_m"] = cam["camera_mount"]["offset_m"]
    if v8:
        profile["camera_mount"].update(rot_deg=v8[0], tilt_deg=v8[1])
    (tmp_path / "profile.json").write_text(json.dumps(profile))
    env = dict(os.environ, ARMD_POSES=str(tmp_path / "poses.json"), ARMD_KEEPOUT=str(tmp_path / "keepout.json"),
               ARMD_GRASP_SETUP=str(tmp_path / "grasp.json"), ARMD_DETECTOR_URL=base, RUST_LOG="armd=info")
    armd = subprocess.Popen([str(armd_binary()), "--follower", "sim", "--follower-port", f"udp://127.0.0.1:{udp}",
                             "--config", str(tmp_path / "profile.json"), "--port", str(http_port),
                             "--log-dir", "off", "--blackbox-dir", "off"],
                            env=env, stdout=open(tmp_path / "armd.log", "w"), stderr=subprocess.STDOUT)
    wait_http(f"http://127.0.0.1:{http_port}/api/v1/observation", armd, tmp_path / "armd.log")
    client = Client(http_port)
    yield client, base, cam, tmp_path, v8 is not None, request.param
    client.close()
    for p in (armd, sim):
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


@pytest.mark.slow
def test_tap_to_grasp_the_cube(cam_rig):
    client, base, cam, logs, v8, case = cam_rig
    target = TARGETS.get(case, "cube")
    deadline = time.time() + 10
    while not client.observation()["feedback_valid"] and time.time() < deadline:
        time.sleep(0.1)
    client.command("/api/v1/torque", {"enable": True})
    assert client.wait_action(client.command("/api/v1/motion/home", {})["action_id"])["state"] == "done"

    # the operator brings the cube into view: v5 tool down above it; v8 tool pitched half way
    # toward camera-down, the image centre 2 cm from the cube
    cube_xy = next(o for o in http(base + "/sim/state")["objects"] if o["name"] == target)["pos"][:2]
    if v8:
        view, orientation = view_pose(cam, cube_xy, 0.20, (0.02, -0.01))
        orientation = {"quat": orientation}
    else:
        view, orientation = [cube_xy[0], cube_xy[1], 0.20], TOOL_DOWN
    rec = client.wait_action(client.command("/api/v1/action/move_to", {"position": view, "orientation": orientation,
                                                                       "speed": {"linear": 0.1}})["action_id"])
    assert rec["state"] == "done", json.dumps(rec)
    time.sleep(0.3)

    # detection through armd's proxy (single trigger, perception API 2)
    for _ in range(5):  # a noisy detector may miss the cube in a frame (--det-drop-rate)
        det = http(client.base + "/api/v1/detection/trigger?wait_ms=2000", {"pipelines": ["yolo"]})
        print(f"\ntrigger via armd: frame {det.get('frame_id')} detections {det.get('detections')}")
        cubes = [d for d in det["detections"] if d["class_name"] == target]
        if cubes:
            break
    assert cubes, det
    d = cubes[0]
    box = [min(1.0, max(0.0, v)) for v in (d["x"] - d["w"] / 2, d["y"] - d["h"] / 2, d["x"] + d["w"] / 2, d["y"] + d["h"] / 2)]
    schema = http(client.base + "/api/v1/observation/schema")
    print(f"schema vision caps: {schema.get('capabilities', {}).get('vision')}")

    approach = 0.15
    overrides = {"fx_px": cam["fx_px"], "fy_px": cam["fx_px"], "approach_height_m": approach}
    if not v8:  # v8: the point under the jaws is outside the image at 0.15 m; camera-down does not use it
        overrides["grip_pixel_norm"] = cam["grip_pixel_norm_at"][f"{approach:.2f}"]
    t0 = time.time()
    ack = client.command("/api/v1/action/grasp", {"label": target, "box_norm": box, "frame_id": det["frame_id"],
                                                  "overrides": overrides})
    rec = None
    while time.time() - t0 < 120:
        obs = client.observation()
        a = obs.get("action")
        if a and a["id"] == ack["action_id"]:
            if a["state"] != "running":
                rec = a
                break
        elif (p := obs.get("previous_action")) and p["id"] == ack["action_id"]:
            rec = p
            break
        time.sleep(0.2)
    dt = time.time() - t0
    state = http(base + "/sim/state")
    cube = next(o for o in state["objects"] if o["name"] == target)
    r = rec or {}
    print(f"grasp: {r.get('state')} code={r.get('code')} holding={r.get('holding')} "
          f"iterations={r.get('iteration')} error_px={r.get('error_px')} "
          f"final_offset_px={r.get('final_offset_px')} expected={ack.get('expected_duration_s')} s in {dt:.1f} s "
          f"height_source={r.get('height_source')} height_reason={r.get('height_reason')} "
          f"object_height_m={r.get('object_height_m')} close_height_m={r.get('close_height_m')} probe_dz_m={r.get('probe_dz_m')} "
          f"align={r.get('align')} recentre={r.get('recentre')} post_lift={r.get('post_lift')}")
    print(f"{target} after the grasp: {cube}")
    assert rec is not None and rec["state"] == "done" and rec["holding"] is True, \
        json.dumps(rec) + "\n" + (logs / "armd.log").read_text()[-3000:]
    assert cube["grasped"] and cube["lifted_m"] > 0.05, cube
    assert math.hypot(cube["pos"][0] - state["tcp"][0], cube["pos"][1] - state["tcp"][1]) < 0.03
    others = [o for o in state["objects"] if o["name"] != target]
    assert all(not o["contacts"]["jaws"] for o in others), others
    print(f"{target} started at {cube_xy}")
    if case.startswith("axis_"):
        # the height probe ran: the object's top within 10 mm, the jaws closed part way up
        h = HEIGHTS_M[target]
        assert rec["height_source"] == "scale", json.dumps(rec)
        assert abs(rec["object_height_m"] - h) <= 0.01, json.dumps(rec)
        expect_close = min(max(rec["object_height_m"] * 0.5, 0.015), 0.06)
        assert abs(rec["close_height_m"] - expect_close) < 1e-3, json.dumps(rec)
