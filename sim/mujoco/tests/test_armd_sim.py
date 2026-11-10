"""End to end: rebot-sim + `armd --follower sim` with the B601-DM profile.

Builds armd (cargo, debug) unless ARMD_BIN names a binary. Checks enable, a
Cartesian move_to over the WebSocket, a grasp of the cube reported as holding,
non-zero gravity torques on J2 / J3, and BUS_LOSS after the simulator is killed.
Run: `uv run pytest -s tests/test_armd_sim.py` in sim/mujoco.
"""
from __future__ import annotations

import json
import math
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import mujoco
import numpy as np
import pytest
from websockets.sync.client import connect

from rebot_sim.model import Indexer, load_model

REPO = Path(__file__).resolve().parents[3]
PROFILE = REPO / "kits" / "recamera-b601dm" / "profile" / "rebot-b601dm.json"
TOOL_DOWN = {"rpy": [0.0, math.pi / 2, 0.0]}  # end_link x (tool axis) pointing down
CUBE_XY = (0.30, 0.0)


def free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def armd_binary() -> Path:
    if os.environ.get("ARMD_BIN"):
        return Path(os.environ["ARMD_BIN"])
    subprocess.run(["cargo", "build", "--quiet", "--bin", "armd"], cwd=REPO, check=True)
    return REPO / "target" / "debug" / "armd"


class Client:
    """WebSocket controller: lease, heartbeats, typed replies, latest observation."""

    def __init__(self, http_port: int):
        self.base = f"http://127.0.0.1:{http_port}"
        self.ws = connect(f"ws://127.0.0.1:{http_port}/api/v1/ws", max_size=None)
        self.replies: queue.Queue = queue.Queue()
        self.obs: dict | None = None
        self.request_id = 0
        self.generation = 0
        self.running = True
        threading.Thread(target=self._recv, daemon=True).start()
        self.send({"type": "acquire_lease", "client_id": "sim_e2e"})
        self.generation = self.reply({"lease_granted"})["generation"]
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _recv(self):
        try:
            for text in self.ws:
                msg = json.loads(text)
                if "type" in msg:
                    self.replies.put(msg)
                elif "positions" in msg:
                    self.obs = msg
        except Exception:  # connection closed at teardown
            pass

    def _heartbeat(self):
        while self.running:
            if self.obs is not None:
                try:
                    self.send({"type": "heartbeat", "generation": self.generation, "epoch_id": self.obs["epoch_id"]})
                except Exception:
                    return
            time.sleep(0.3)

    def send(self, msg: dict):
        self.ws.send(json.dumps(msg))

    def reply(self, kinds: set, timeout: float = 5.0) -> dict:
        deadline = time.time() + timeout
        while True:
            msg = self.replies.get(timeout=max(0.01, deadline - time.time()))
            if msg["type"] in kinds:
                return msg

    def observation(self) -> dict:
        with urllib.request.urlopen(self.base + "/api/v1/observation", timeout=2) as r:
            return json.loads(r.read())

    def command(self, path: str, body: dict) -> dict:
        self.request_id += 1
        epoch = self.observation()["epoch_id"]
        self.send({"type": "command", "request_id": self.request_id, "generation": self.generation,
                   "epoch_id": epoch, "path": path, "body": body})
        msg = self.reply({"command_ack", "command_reject"})
        assert msg["type"] == "command_ack", msg
        return msg

    def wait_action(self, action_id: int, timeout: float = 30.0) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            obs = self.observation()
            for rec in (obs.get("action"), obs.get("previous_action")):
                if rec and rec["id"] == action_id and rec["state"] != "running":
                    return rec
            time.sleep(0.05)
        raise TimeoutError(f"action {action_id} still running")

    def close(self):
        self.running = False
        self.ws.close()


class Fk:
    def __init__(self):
        self.model = load_model(devarm_dir="")
        self.data = mujoco.MjData(self.model)
        self.idx = Indexer(self.model)

    def tcp(self, positions) -> np.ndarray:
        self.idx.set_motor_positions(self.data, positions[:6])
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.body("end_link").xpos.copy()


@pytest.fixture()
def rig(tmp_path):
    udp, http = free_port(socket.SOCK_DGRAM), free_port()
    sim = subprocess.Popen([sys.executable, "-m", "rebot_sim.server", "--port", str(udp), "--headless", "--stats", "2"],
                           stdout=open(tmp_path / "sim.log", "w"), stderr=subprocess.STDOUT)
    env = dict(os.environ, ARMD_POSES=str(tmp_path / "poses.json"), ARMD_KEEPOUT=str(tmp_path / "keepout.json"),
               ARMD_GRASP_SETUP=str(tmp_path / "grasp.json"), RUST_LOG="armd=info")
    armd = subprocess.Popen([str(armd_binary()), "--follower", "sim", "--follower-port", f"udp://127.0.0.1:{udp}",
                             "--config", str(PROFILE), "--port", str(http), "--log-dir", "off", "--blackbox-dir", "off"],
                            env=env, stdout=open(tmp_path / "armd.log", "w"), stderr=subprocess.STDOUT)
    deadline = time.time() + 20
    while True:
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{http}/api/v1/observation", timeout=1)
            break
        except OSError:
            if time.time() > deadline or armd.poll() is not None:
                raise RuntimeError((tmp_path / "armd.log").read_text())
            time.sleep(0.2)
    client = Client(http)
    yield client, sim, tmp_path
    client.close()
    for p in (armd, sim):
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def test_armd_on_simulator(rig):
    client, sim, logs = rig
    fk = Fk()
    obs = client.observation()
    print(f"\nboot: feedback_valid={obs['feedback_valid']} enable_state={obs['enable_state']} positions={np.round(obs['positions'], 3).tolist()}")
    assert obs["feedback_valid"], "simulator feedback not reaching armd"

    # 1. enable
    client.command("/api/v1/torque", {"enable": True})
    obs = client.observation()
    print(f"enable: is_torqued={obs['is_torqued']} enable_state={obs['enable_state']} safety={obs['safety_state']}")
    assert obs["is_torqued"] and obs["enable_state"] == "Enabled"

    # The wrist droops while the motors are off, by an amount that depends on timing:
    # start every run from the profile's home pose.
    rec = client.wait_action(client.command("/api/v1/motion/home", {})["action_id"])
    print(f"home: {rec['state']} positions={np.round(client.observation()['positions'], 3).tolist()}")
    assert rec["state"] == "done", json.dumps(rec)

    # 2. open the jaws, then move_to above the cube with the tool pointing down
    rec = client.wait_action(client.command("/api/v1/action/gripper", {"command": "open", "speed": 2.0, "timeout_s": 10})["action_id"])
    print(f"gripper open: {rec['state']} gap={rec['gripper_gap_rad']}")
    assert rec["state"] == "done", json.dumps(rec)
    above = [CUBE_XY[0], CUBE_XY[1], 0.12]
    ack = client.command("/api/v1/action/move_to", {"position": above, "orientation": TOOL_DOWN, "speed": {"linear": 0.1}})
    rec = client.wait_action(ack["action_id"])
    time.sleep(0.3)
    obs = client.observation()
    reached = fk.tcp(obs["positions"])
    err_mm = float(np.linalg.norm(reached - above) * 1000)
    print(f"move_to {above}: {rec['state']} settled={rec['settled']} tcp={np.round(reached, 4).tolist()} error={err_mm:.2f} mm")
    assert rec["state"] == "done", json.dumps(rec)
    assert err_mm < 5.0

    # 3. gravity torques on J2 / J3 while holding the pose
    tau = obs["torques"]
    print(f"torques at hold: {np.round(tau, 3).tolist()} residuals={obs.get('residual_torques')}")
    assert abs(tau[1]) > 0.3 and abs(tau[2]) > 0.3

    # 4. descend to grasp height and close on the cube
    grasp = [CUBE_XY[0], CUBE_XY[1], 0.015]
    rec = client.wait_action(client.command("/api/v1/action/move_to", {"position": grasp, "orientation": TOOL_DOWN})["action_id"])
    print(f"descend: {rec['state']} tcp={np.round(fk.tcp(client.observation()['positions']), 4).tolist()}")
    assert rec["state"] == "done", json.dumps(rec)
    rec = client.wait_action(client.command("/api/v1/action/gripper", {"command": "close", "grasp": True})["action_id"])
    obs = client.observation()
    print(f"gripper close: {rec['state']} holding={rec['holding']} gap={rec['gripper_gap_rad']} q7={obs['positions'][6]:.3f} tau7={obs['torques'][6]:.3f}")
    assert rec["state"] == "done" and rec["holding"] is True, json.dumps(rec)

    # 5. lift: the cube comes along (jaw gap stays)
    rec = client.wait_action(client.command("/api/v1/action/move_relative", {"frame": "base", "d": [0, 0, 0.08]})["action_id"])
    time.sleep(0.5)
    obs = client.observation()
    print(f"lift: {rec['state']} q7={obs['positions'][6]:.3f} tau7={obs['torques'][6]:.3f} torques={np.round(obs['torques'], 3).tolist()}")
    assert rec["state"] == "done", json.dumps(rec)
    assert obs["positions"][6] < -1.5, "jaws closed: the cube slipped out"
    time.sleep(2.2)  # one more simulator stats line (every 2 s) with the cube position
    cube_z = float([ln for ln in (logs / "sim.log").read_text().splitlines() if "cube=(" in ln][-1].split("cube=(")[1].split(")")[0].split(",")[2])
    print(f"cube z in the simulator after the lift: {cube_z:.3f} m")
    assert cube_z > 0.06, "the cube did not come up with the gripper"

    # 6. kill the simulator: armd must latch BUS_LOSS
    sim.kill()
    sim.wait()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        obs = client.observation()
        if obs.get("fault"):
            break
        time.sleep(0.05)
    print(f"after simulator kill ({time.time() - t0:.2f} s): safety={obs['safety_state']} fault={obs.get('fault')} feedback_valid={obs['feedback_valid']}")
    assert obs.get("fault") and obs["fault"]["code"] == "BUS_LOSS"
    print("simulator log tail:\n" + "".join((logs / "sim.log").read_text().splitlines(True)[-3:]))
