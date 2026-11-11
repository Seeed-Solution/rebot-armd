"""rebot-sysid recovers known joint friction from `armd --follower sim`.

The simulator runs with damping and friction loss set per joint; rebot-sysid drives armd
as a normal client and its Coulomb / viscous fit must match within 15 %. Slow (about
2.5 min): deselect with `-m "not slow"`.
"""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

from rebot_sim import sysid
from test_armd_sim import PROFILE, armd_binary, free_port

DAMPING = [1.5, 1.2, 1.5, 0.3, 0.25, 0.3]       # N·m·s/rad
FRICTIONLOSS = [0.3, 0.4, 0.3, 0.1, 0.12, 0.1]  # N·m
POSE = [0.0, -1.6, -1.2, 0.9, 0.0, 0.0]


@pytest.mark.slow
def test_sysid_recovers_sim_friction(tmp_path):
    udp, http = free_port(socket.SOCK_DGRAM), free_port()
    sim = subprocess.Popen([sys.executable, "-m", "rebot_sim.server", "--port", str(udp),
                            "--torque-report", "mean", "--damping", ",".join(map(str, DAMPING)), "--frictionloss", ",".join(map(str, FRICTIONLOSS))],
                           stdout=open(tmp_path / "sim.log", "w"), stderr=subprocess.STDOUT)
    env = dict(os.environ, ARMD_POSES=str(tmp_path / "poses.json"), ARMD_KEEPOUT=str(tmp_path / "keepout.json"),
               ARMD_GRASP_SETUP=str(tmp_path / "grasp.json"))
    armd = subprocess.Popen([str(armd_binary()), "--follower", "sim", "--follower-port", f"udp://127.0.0.1:{udp}",
                             "--config", str(PROFILE), "--port", str(http), "--log-dir", "off", "--blackbox-dir", "off"],
                            env=env, stdout=open(tmp_path / "armd.log", "w"), stderr=subprocess.STDOUT)
    url = f"http://127.0.0.1:{http}"
    try:
        deadline = time.time() + 20
        while True:
            try:
                import json
                with urllib.request.urlopen(url + "/api/v1/observation", timeout=1) as r:
                    if json.loads(r.read())["feedback_valid"]:
                        break
            except OSError:
                pass
            if time.time() > deadline or armd.poll() is not None:
                raise RuntimeError((tmp_path / "armd.log").read_text())
            time.sleep(0.2)
        report = sysid.main(["--url", url, "--i-am-at-the-arm", "--yes", "--enable", "--pose", ",".join(map(str, POSE)),
                             "--out", str(tmp_path / "out")])
    finally:
        for p in (armd, sim):
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
    worst = 0.0
    for j in report["joints"]:
        i = j["joint"] - 1
        e_fc = abs(j["coulomb_nm"] - FRICTIONLOSS[i]) / FRICTIONLOSS[i]
        e_fv = abs(j["viscous_nm_s_rad"] - DAMPING[i]) / DAMPING[i]
        worst = max(worst, e_fc, e_fv)
        print(f"J{i + 1}: Fc {j['coulomb_nm']:.4f} (set {FRICTIONLOSS[i]}, {e_fc:+.1%}) "
              f"Fv {j['viscous_nm_s_rad']:.4f} (set {DAMPING[i]}, {e_fv:+.1%}) "
              f"threshold {j['collision']['suggested_threshold_nm']:.3f} N·m")
    print(f"worst relative error {worst:.1%}; plots: {report['plots'][:1]}")
    assert worst < 0.15
