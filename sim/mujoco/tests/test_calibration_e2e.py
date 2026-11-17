"""End to end: the cockpit's camera calibration wizard against `armd --follower sim`, then a v8
camera grasp that relies on the applied calibration.

The simulator is the truth: v8 holder (rot 0, tilt -38, the simulator's lens centre), fx 800 (the
nominal is 824), the virtual GridBoard (`--aruco-board`, 4 x 3, 25 mm markers) on the table with
0.3 px corner noise. armd starts with a profile whose camera_mount is off (offset +10 mm along
end_link x, tilt -35) and no calibration file (temporary ARMD_CALIBRATION / ARMD_GRASP_SETUP).

tests/calib_wizard_e2e.mjs drives the wizard flow from node with src/api/calib_wizard.js and the
cockpit IK (no browser): start pose -> bootstrap -> generated poses -> WS move_to (0.03 m/s,
0.3 rad/s) -> stationary -> /api/v1/detection/trigger ["aruco"] -> solve -> gates -> WS
calibration/apply (dry run, then real). Checked against the truth: fx within 1 %, offset within
3 mm, mount rotation within 1 deg, table_z within 3 mm; the apply ack reads back the solved
values and GET /api/v1/calibration reports calibrated. Then the v8 camera grasp of the cube runs
against the same armd without the fx_px / grip_pixel_norm / camera overrides
test_camera_grasp.py passes, and must hold and lift the cube.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from test_armd_sim import PROFILE, Client, armd_binary, free_port
from test_camera_grasp import http, view_pose, wait_http

HERE = Path(__file__).resolve().parent
TRUTH_FX = 800.0
BOARD = "0.30,0.02,-80"
OFFSET_ERR_M = (0.010, 0.0, 0.0)
TILT_NOMINAL = -35.0


def quat_r(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_deg(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1) / 2))))


@pytest.fixture()
def calib_rig(tmp_path):
    if shutil.which("node") is None:
        pytest.skip("node not installed")
    udp, http_port, video, percep = free_port(socket.SOCK_DGRAM), free_port(), free_port(), free_port()
    sim = subprocess.Popen([sys.executable, "-m", "rebot_sim.server", "--port", str(udp), "--headless", "--stats", "2",
                            "--camera", "--video-port", str(video), "--perception-port", str(percep),
                            "--camera-rot-deg", "0", "--camera-tilt-deg", "-38", "--fx", str(TRUTH_FX),
                            "--aruco-board", BOARD, "--marker-noise-px", "0.3"],
                           stdout=open(tmp_path / "sim.log", "w"), stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{percep}"
    wait_http(base + "/sim/camera", sim, tmp_path / "sim.log", udp_hold=udp)
    cam = http(base + "/sim/camera")
    profile = json.loads(PROFILE.read_text())
    truth_off = cam["camera_mount"]["offset_m"]
    profile["camera_mount"].update({"rot_deg": 0.0, "tilt_deg": TILT_NOMINAL,
                                    "offset_m": [t + e for t, e in zip(truth_off, OFFSET_ERR_M)]})
    (tmp_path / "profile.json").write_text(json.dumps(profile))
    env = dict(os.environ, ARMD_POSES=str(tmp_path / "poses.json"), ARMD_KEEPOUT=str(tmp_path / "keepout.json"),
               ARMD_GRASP_SETUP=str(tmp_path / "grasp.json"), ARMD_CALIBRATION=str(tmp_path / "calibration.json"),
               ARMD_DETECTOR_URL=base, RUST_LOG="armd=info")
    armd = subprocess.Popen([str(armd_binary()), "--follower", "sim", "--follower-port", f"udp://127.0.0.1:{udp}",
                             "--config", str(tmp_path / "profile.json"), "--port", str(http_port),
                             "--log-dir", "off", "--blackbox-dir", "off"],
                            env=env, stdout=open(tmp_path / "armd.log", "w"), stderr=subprocess.STDOUT)
    wait_http(f"http://127.0.0.1:{http_port}/api/v1/observation", armd, tmp_path / "armd.log")
    yield http_port, base, cam, tmp_path
    for p in (armd, sim):
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


@pytest.mark.slow
def test_calibration_wizard_then_v8_grasp(calib_rig):
    http_port, base, cam, logs = calib_rig
    armd = f"http://127.0.0.1:{http_port}"
    deadline = time.time() + 10
    while not http(armd + "/api/v1/observation")["feedback_valid"] and time.time() < deadline:
        time.sleep(0.1)
    before = http(armd + "/api/v1/calibration")
    assert before["calibrated"] is False and before["camera_mount"]["tilt_deg"] == TILT_NOMINAL, before

    # ---- the wizard, from node
    out = logs / "wizard.json"
    t0 = time.time()
    run = subprocess.run(["node", str(HERE / "calib_wizard_e2e.mjs"), "--armd", armd, "--board", BOARD, "--out", str(out)],
                         capture_output=True, text=True, timeout=900)
    print(run.stdout[-6000:], run.stderr[-2000:])
    res = json.loads(out.read_text())
    assert run.returncode == 0, res.get("error", "") + "\n" + (logs / "armd.log").read_text()[-3000:]
    print(f"wizard run {time.time() - t0:.0f} s, timings {res['timings']}, views {len(res['views'])}, skipped {res['skipped']}")

    # ---- recovered vs truth
    a = res["solve"]["apply"]
    k, m = a["intrinsics"], a["camera_mount"]
    r_true = np.array(cam["R_end_cam"])
    off_true = np.array(cam["camera_mount"]["offset_m"])
    fx_err = k["fx"] / TRUTH_FX - 1
    off_err = float(np.linalg.norm(np.array(m["offset_m"]) - off_true))
    r_err = rot_deg(quat_r(m["q_end_cam"]), r_true)
    z_err = a["table_z_m"] - 0.0
    sep = res["solve"]["separate"]
    print(f"truth: fx {TRUTH_FX} cx 320 cy 180, offset {np.round(off_true * 1000, 1).tolist()} mm, R_end_cam from /sim/camera, table_z 0")
    print(f"nominal armd mount: offset {np.round(np.array(before['camera_mount']['offset_m']) * 1000, 1).tolist()} mm, tilt {TILT_NOMINAL}; "
          f"nominal error {np.linalg.norm(np.array(before['camera_mount']['offset_m']) - off_true) * 1000:.1f} mm")
    print(f"recovered: fx {k['fx']:.2f} fy {k['fy']:.2f} cx {k['cx']:.2f} cy {k['cy']:.2f} k1 {k['k1']:.4f} k2 {k['k2']:.4f}; "
          f"offset {np.round(np.array(m['offset_m']) * 1000, 2).tolist()} mm; table_z {a['table_z_m'] * 1000:.2f} mm")
    print(f"errors: fx {fx_err * 100:+.3f} %, offset {off_err * 1000:.2f} mm, mount rotation {r_err:.3f} deg, table_z {z_err * 1000:+.2f} mm")
    print(f"separate solve: fx {sep['K']['fx']:.2f}, offset err {np.linalg.norm(np.array(sep['offset_m']) - off_true) * 1000:.2f} mm, "
          f"rotation err {rot_deg(quat_r(sep['q_end_cam']), r_true):.3f} deg; joint RMS {res['solve']['joint']['rmsBefore']:.3f} -> "
          f"{res['solve']['joint']['rmsAfter']:.3f} px")
    print("gates: " + " ".join(f"{g['key']}={'ok' if g['ok'] else 'FAIL'}({g['value']:.4g})" if isinstance(g['value'], float) else f"{g['key']}={g['ok']}({g['value']})" for g in res["solve"]["gates"]))
    assert res["solve"]["ok"]
    assert abs(fx_err) < 0.01 and abs(k["fy"] / TRUTH_FX - 1) < 0.01
    assert off_err < 0.003
    assert r_err < 1.0
    assert abs(z_err) < 0.003

    # ---- dry run, ack read-back, GET /api/v1/calibration
    assert res["dry"]["before"]["camera_mount"]["q_end_cam"] is None, res["dry"]["before"]
    ack = res["ack"]
    vi = ack["vision_intrinsics"]
    for key in ("fx", "fy", "cx", "cy", "k1", "k2"):
        assert vi[key] == pytest.approx(k[key], abs=1e-9), (key, vi, k)
    assert ack["camera_mount"]["offset_m"] == pytest.approx(m["offset_m"], abs=1e-6)  # armd keeps the mount in f32
    assert ack["camera_mount"]["q_end_cam"] == pytest.approx(m["q_end_cam"], abs=1e-6)
    g = ack["grasp"]
    assert (g["fx_px"], g["fy_px"], g["cx_px"], g["cy_px"]) == pytest.approx((k["fx"], k["fy"], k["cx"], k["cy"]), abs=1e-9)
    assert g["table_z_m"] == pytest.approx(a["table_z_m"], abs=1e-9)
    print(f"ack: vision revision {vi.get('revision')}, grasp camera_tilt_deg {g['camera_tilt_deg']:.2f}, grip_pixel_norm {g['grip_pixel_norm']}, "
          f"grip_pixel_model {ack.get('grip_pixel_model')}, backup {ack['backup']}")
    cal = http(armd + "/api/v1/calibration")
    assert cal["calibrated"] is True and cal["camera_mount"]["q_end_cam"] == pytest.approx(m["q_end_cam"], abs=1e-6), cal
    assert len(cal["backups"]) == 1, cal["backups"]
    sim_intr = http(base + "/api/calibration/intrinsics")
    assert sim_intr["fx"] == pytest.approx(k["fx"]) and sim_intr["revision"] == 1, sim_intr

    # ---- v8 camera grasp with the calibration only (no fx_px / grip_pixel_norm / camera overrides)
    client = Client(http_port)
    try:
        client.command("/api/v1/torque", {"enable": True})
        rec = client.wait_action(client.command("/api/v1/motion/home", {})["action_id"])
        assert rec["state"] == "done", json.dumps(rec)
        cube_xy = next(o for o in http(base + "/sim/state")["objects"] if o["name"] == "cube")["pos"][:2]
        view, q = view_pose(cam, cube_xy, 0.20, (0.02, -0.01))
        rec = client.wait_action(client.command("/api/v1/action/move_to", {"position": view, "orientation": {"quat": q},
                                                                           "speed": {"linear": 0.1}})["action_id"])
        assert rec["state"] == "done", json.dumps(rec)
        time.sleep(0.3)
        det = http(client.base + "/api/v1/detection/trigger?wait_ms=2000", {"pipelines": ["yolo"]})
        cubes = [d for d in det["detections"] if d["class_name"] == "cube"]
        assert cubes, det
        d = cubes[0]
        box = [min(1.0, max(0.0, v)) for v in (d["x"] - d["w"] / 2, d["y"] - d["h"] / 2, d["x"] + d["w"] / 2, d["y"] + d["h"] / 2)]
        request = {"label": "cube", "box_norm": box, "frame_id": det["frame_id"], "overrides": {"approach_height_m": 0.15}}
        ack = client.command("/api/v1/action/grasp", request)
        rec = client.wait_action(ack["action_id"], timeout=120)
        state = http(base + "/sim/state")
        cube = next(o for o in state["objects"] if o["name"] == "cube")
        print(f"grasp: {rec.get('state')} code={rec.get('code')} holding={rec.get('holding')} iterations={rec.get('iteration')} "
              f"error_px={rec.get('error_px')} final_offset_px={rec.get('final_offset_px')} acquire={rec.get('acquire')} "
              f"height_source={rec.get('height_source')}; cube {cube}")
        assert rec["state"] == "done" and rec["holding"] is True, json.dumps(rec) + "\n" + (logs / "armd.log").read_text()[-3000:]
        assert cube["grasped"] and cube["lifted_m"] > 0.05, cube
        assert math.hypot(cube["pos"][0] - state["tcp"][0], cube["pos"][1] - state["tcp"][1]) < 0.03
    finally:
        client.close()
