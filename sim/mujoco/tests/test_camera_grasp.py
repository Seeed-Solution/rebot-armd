"""End to end: tap-to-grasp in simulation. rebot-sim --camera serves the wrist stream and the
perception API; armd proxies it (ARMD_DETECTOR_URL) and runs the grasp macro, whose centring
loop triggers ground-truth detections on fresh frames.

armd gets a copy of the kit profile with camera_mount.offset_m set to the simulator's lens
centre (the kit profile leaves it null, unmeasured); the grasp request overrides the setup's
grip pixel and focal length with the simulator's values (GET /sim/camera).
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

import pytest

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


# simulator arguments: the default cube, and two seeded objects. Both use the kit profile's
# mount (v5, optical axis along the tool). The v8 38 deg holder cannot run this macro: with the
# tool pointing down, the table point under the jaws leaves the image above 0.03 m (docs).
CASES = {
    "default": [],
    "seed3_cube_cup": ["--objects", "cube,cup", "--seed", "3"],
}


@pytest.fixture(params=list(CASES))
def cam_rig(request, tmp_path):
    extra = CASES[request.param]
    udp, http_port, video, percep = free_port(socket.SOCK_DGRAM), free_port(), free_port(), free_port()
    sim = subprocess.Popen([sys.executable, "-m", "rebot_sim.server", "--port", str(udp), "--headless", "--stats", "2",
                            "--camera", "--video-port", str(video), "--perception-port", str(percep), *extra],
                           stdout=open(tmp_path / "sim.log", "w"), stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{percep}"
    wait_http(base + "/sim/camera", sim, tmp_path / "sim.log")
    cam = http(base + "/sim/camera")
    profile = json.loads(PROFILE.read_text())
    profile["camera_mount"]["offset_m"] = cam["camera_mount"]["offset_m"]
    (tmp_path / "profile.json").write_text(json.dumps(profile))
    env = dict(os.environ, ARMD_POSES=str(tmp_path / "poses.json"), ARMD_KEEPOUT=str(tmp_path / "keepout.json"),
               ARMD_GRASP_SETUP=str(tmp_path / "grasp.json"), ARMD_DETECTOR_URL=base, RUST_LOG="armd=info")
    armd = subprocess.Popen([str(armd_binary()), "--follower", "sim", "--follower-port", f"udp://127.0.0.1:{udp}",
                             "--config", str(tmp_path / "profile.json"), "--port", str(http_port),
                             "--log-dir", "off", "--blackbox-dir", "off"],
                            env=env, stdout=open(tmp_path / "armd.log", "w"), stderr=subprocess.STDOUT)
    wait_http(f"http://127.0.0.1:{http_port}/api/v1/observation", armd, tmp_path / "armd.log")
    client = Client(http_port)
    yield client, base, cam, tmp_path
    client.close()
    for p in (armd, sim):
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


@pytest.mark.slow
def test_tap_to_grasp_the_cube(cam_rig):
    client, base, cam, logs = cam_rig
    deadline = time.time() + 10
    while not client.observation()["feedback_valid"] and time.time() < deadline:
        time.sleep(0.1)
    client.command("/api/v1/torque", {"enable": True})
    assert client.wait_action(client.command("/api/v1/motion/home", {})["action_id"])["state"] == "done"

    # look at the table from above, tool down
    cube_xy = next(o for o in http(base + "/sim/state")["objects"] if o["name"] == "cube")["pos"][:2]
    view = [cube_xy[0], cube_xy[1], 0.20]  # the operator brings the cube into view
    rec = client.wait_action(client.command("/api/v1/action/move_to", {"position": view, "orientation": TOOL_DOWN,
                                                                       "speed": {"linear": 0.1}})["action_id"])
    assert rec["state"] == "done", json.dumps(rec)
    time.sleep(0.3)

    # detection through armd's proxy (single trigger, perception API 2)
    det = http(client.base + "/api/v1/detection/trigger?wait_ms=2000", {"pipelines": ["yolo"]})
    print(f"\ntrigger via armd: frame {det.get('frame_id')} detections {det.get('detections')}")
    cubes = [d for d in det["detections"] if d["class_name"] == "cube"]
    assert cubes, det
    d = cubes[0]
    box = [min(1.0, max(0.0, v)) for v in (d["x"] - d["w"] / 2, d["y"] - d["h"] / 2, d["x"] + d["w"] / 2, d["y"] + d["h"] / 2)]
    schema = http(client.base + "/api/v1/observation/schema")
    print(f"schema vision caps: {schema.get('capabilities', {}).get('vision')}")

    approach = 0.15
    overrides = {"grip_pixel_norm": cam["grip_pixel_norm_at"][f"{approach:.2f}"], "fx_px": cam["fx_px"],
                 "fy_px": cam["fx_px"], "approach_height_m": approach}
    t0 = time.time()
    ack = client.command("/api/v1/action/grasp", {"label": "cube", "box_norm": box, "frame_id": det["frame_id"],
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
    cube = next(o for o in state["objects"] if o["name"] == "cube")
    print(f"grasp: {rec and rec['state']} code={rec and rec.get('code')} holding={rec and rec.get('holding')} "
          f"iterations={rec and rec.get('iteration')} error_px={rec and rec.get('error_px')} in {dt:.1f} s")
    print(f"cube after the grasp: {cube}")
    assert rec is not None and rec["state"] == "done" and rec["holding"] is True, \
        json.dumps(rec) + "\n" + (logs / "armd.log").read_text()[-3000:]
    assert cube["grasped"] and cube["lifted_m"] > 0.05, cube
    assert math.hypot(cube["pos"][0] - state["tcp"][0], cube["pos"][1] - state["tcp"][1]) < 0.03
    others = [o for o in state["objects"] if o["name"] != "cube"]
    assert all(not o["contacts"]["jaws"] for o in others), others
    print(f"cube started at {cube_xy}")
