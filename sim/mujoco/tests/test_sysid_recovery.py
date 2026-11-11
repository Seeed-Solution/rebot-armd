"""rebot-sysid error paths against a fake armd link (no simulator, no armd).

Partial results after an abort, the slow return / home / unload path after errors and on
success, joint vector length checks, viscous reliability flag.
"""
from __future__ import annotations

import json

import numpy as np
import pytest

from rebot_sim import sysid

POSE = [0.0, -1.0, -1.2, 0.6, 0.0, 0.0, 0.0]
HOME = [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0, -0.5]
REST = {"position_tolerances": [0.15, 0.08, 0.08, 0.08, 0.08, 0.08]}
SCHEMA = {"dof": 7, "home_positions": HOME, "resting_pose": REST}
ARGS = ["--i-am-at-the-arm", "--yes", "--amplitude", "0.1", "--speeds", "0.05,0.1", "--cycles", "1"]


class FakeLink:
    """Servo frames move the arm at once; residual = fc * sign(v) + fv * v."""

    def __init__(self, fc=0.3, fv=0.5, on_servo=None, positions=POSE):
        self.obs = {"positions": list(positions), "velocities": [0.0] * len(positions), "torques": [0.0] * len(positions),
                    "residual_torques": [0.0] * 6, "timestamp": 0.0, "is_torqued": True, "safety_state": "Armed",
                    "epoch_id": 1}
        self.samples: list[dict] = []
        self.recording = False
        self.calls: list[tuple] = []
        self.fc, self.fv, self.on_servo = fc, fv, on_servo
        self.servos = 0

    def servo(self, positions):
        self.servos += 1
        q = [float(x) for x in positions]
        v = [(a - b) / 0.02 for a, b in zip(q, self.obs["positions"])]
        res = [self.fc * float(np.sign(x)) + self.fv * x for x in v[:6]]
        self.obs = {**self.obs, "positions": q, "velocities": v, "residual_torques": res,
                    "timestamp": self.obs["timestamp"] + 0.02}
        if self.on_servo:
            self.on_servo(self)
        if self.recording:
            self.samples.append({"t": self.obs["timestamp"], "q": q, "dq": v, "tau": [0.0] * len(q),
                                 "res": self.obs["residual_torques"], "cmd": q, "state": self.obs["safety_state"]})
        self.calls.append(("servo",))

    def command(self, path, body, timeout=5.0):
        self.calls.append((path, body))
        if path == "/api/v1/motion/unload":
            self.obs = {**self.obs, "is_torqued": False, "safety_state": "Disabled"}
        return {}

    def jog(self, positions, duration=None):
        self.calls.append(("jog", list(positions), duration))
        self.obs = {**self.obs, "positions": list(positions)}

    def wait_action(self, action_id, timeout):
        return {}

    def stop(self):
        self.calls.append(("stop",))

    def close(self):
        self.calls.append(("close",))


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setattr(sysid.time, "sleep", lambda s: None)

    def make(link, schema):
        monkeypatch.setattr(sysid, "ArmdLink", lambda url: link)
        monkeypatch.setattr(sysid, "fetch_schema", lambda url: schema)
        return link
    return make


def after(calls, kind):
    """Calls after the last servo frame, by kind."""
    last = max((i for i, c in enumerate(calls) if c[0] == "servo"), default=-1)
    return [c for c in calls[last + 1:] if c[0] == kind or c[0] == "/api/v1/motion/unload"]


def test_abort_keeps_finished_joints_and_returns_slowly_home_and_unloads(rig, tmp_path):
    def spike(link):
        # J2 sweep: J2 has left the start pose; J4 residual jumps by 3 N·m (abort level 2.5)
        if abs(link.obs["positions"][1] - POSE[1]) > 0.05:
            link.obs["residual_torques"] = list(link.obs["residual_torques"])
            link.obs["residual_torques"][3] += 3.0
    link = rig(FakeLink(on_servo=spike), SCHEMA)
    with pytest.raises(sysid.SysidError, match="J4 residual changed"):
        sysid.main(ARGS + ["--joints", "1,2", "--unload", "--out", str(tmp_path)])
    report = json.loads((tmp_path / "report.json").read_text())
    assert "J4 residual changed" in report["aborted"]
    assert report["joints_finished"] == [1]
    assert [j["joint"] for j in report["joints"]] == [1]
    assert abs(report["joints"][0]["coulomb_nm"] - 0.3) < 0.02
    assert abs(report["joints"][0]["viscous_nm_s_rad"] - 0.5) < 0.05
    assert "J4 residual changed" in json.loads((tmp_path / "samples.json").read_text())["aborted"]
    tail = after(link.calls, "jog")
    stop_at = max(i for i, c in enumerate(link.calls) if c[0] == "servo") + 1
    assert link.calls[stop_at] == ("stop",)
    # back to the sweep start (J2 at most 0.1 rad away: the 3 s minimum), then home, then unload
    assert [c[0] for c in tail] == ["jog", "jog", "/api/v1/motion/unload"]
    assert tail[0][1][:6] == pytest.approx(POSE[:6]) and tail[0][2] == pytest.approx(3.0)
    home = HOME[:6] + POSE[6:]  # the gripper is not moved
    assert tail[1][1] == pytest.approx(home)
    assert tail[1][2] == pytest.approx(max(abs(h - p) for h, p in zip(HOME[:6], POSE[:6])) / 0.1)


def test_fault_exit_does_not_move_the_arm(rig, tmp_path):
    def fault(link):
        if link.servos == 40:
            link.obs["safety_state"] = {"Fault": {"code": "TRACKING_ERROR"}}
    link = rig(FakeLink(on_servo=fault), SCHEMA)
    with pytest.raises(sysid.SysidError, match="safety state"):
        sysid.main(ARGS + ["--joints", "1", "--unload", "--out", str(tmp_path)])
    report = json.loads((tmp_path / "report.json").read_text())
    assert "safety state" in report["aborted"] and report["joints_finished"] == [] and report["joints"] == []
    assert after(link.calls, "jog") == []


def test_success_returns_home_before_unload_and_flags_negative_viscous(rig, tmp_path):
    link = rig(FakeLink(fc=0.4, fv=-0.2), SCHEMA)
    report = sysid.main(ARGS + ["--joints", "5", "--unload", "--out", str(tmp_path)])
    j = report["joints"][0]
    assert report["aborted"] is None
    assert j["viscous_nm_s_rad"] < 0 and j["viscous_reliable"] is False and "unreliable" in j["viscous_note"]
    assert j["collision"]["basis"] == "coulomb_only"
    assert abs(j["coulomb_only"]["coulomb_nm"] - 0.4) < 0.05
    tail = after(link.calls, "jog")
    assert [c[0] for c in tail] == ["jog", "jog", "/api/v1/motion/unload"]
    assert tail[1][1] == pytest.approx(HOME[:6] + POSE[6:])


@pytest.mark.parametrize("schema", [{"dof": 7}, {"dof": 7, "home_positions": HOME},
                                    {"dof": 7, "home_positions": HOME, "resting_pose": {"position_tolerances": [0.1] * 3}}])
def test_unload_without_authoritative_home_is_refused_before_moving(rig, tmp_path, schema):
    # no home / resting tolerances from armd: no fallback to the sweep start, nothing moves
    link = rig(FakeLink(), schema)
    with pytest.raises(sysid.SysidError, match="--unload needs the home pose"):
        sysid.main(ARGS + ["--joints", "6", "--unload", "--out", str(tmp_path)])
    assert [c for c in link.calls if c[0] in ("servo", "jog", "/api/v1/motion/unload")] == []


def test_recovery_without_authoritative_home_does_not_unload(rig, tmp_path):
    # the recovery path itself never unloads without a schema home
    link = FakeLink()
    a = sysid.parse_args(ARGS + ["--unload"])
    with pytest.raises(sysid.SysidError, match="not unloading"):
        sysid.finish(link, a, POSE, None, 7, log=lambda m: None)
    assert [c[0] for c in link.calls] == ["jog"]


def test_unload_refused_when_home_jog_ends_outside_tolerance(rig, tmp_path):
    class Short(FakeLink):
        def jog(self, positions, duration=None):  # the home jog stops 0.2 rad short on J2
            super().jog(positions, duration)
            if positions[1] == HOME[1]:
                q = list(self.obs["positions"]); q[1] -= 0.2
                self.obs = {**self.obs, "positions": q}
    link = rig(Short(), SCHEMA)
    with pytest.raises(sysid.SysidError, match="not unloading.*not within the resting tolerances"):
        sysid.main(ARGS + ["--joints", "6", "--unload", "--out", str(tmp_path)])
    assert [c[0] for c in after(link.calls, "jog")] == ["jog", "jog"]
    assert link.obs["is_torqued"] is True


def test_no_unload_returns_to_start_and_holds(rig, tmp_path):
    link = rig(FakeLink(), {"dof": 7})
    sysid.main(ARGS + ["--joints", "6", "--out", str(tmp_path)])
    tail = after(link.calls, "jog")
    assert [c[0] for c in tail] == ["jog"] and tail[0][1] == pytest.approx(POSE)


@pytest.mark.parametrize("flag,value", [
    ("--abort-nm", "4,4,nan,2.5,2.5,2.5"), ("--abort-nm", "4,4,4,2.5,inf,2.5"), ("--abort-nm", "4,4,4,-1,2.5,2.5"),
    ("--amplitude", "nan"), ("--amplitude", "-0.1"), ("--amplitude", "inf"),
    ("--speeds", "0.05,nan"), ("--speeds", "0.05,-0.1"), ("--speeds", "inf,0.1"),
    ("--accel", "nan"), ("--accel", "-1"), ("--accel", "inf"),
    ("--approach-speed", "nan"), ("--approach-speed", "-0.1"), ("--approach-speed", "inf"),
])
def test_non_finite_or_negative_arguments_are_rejected(flag, value):
    with pytest.raises(SystemExit):
        sysid.parse_args(["--i-am-at-the-arm", f"{flag}={value}"])


def test_approach_jog_duration_from_approach_speed(rig, tmp_path):
    link = rig(FakeLink(), {"dof": 7})
    pose = "0.0,-1.0,-0.6,0.6,0.0,0.0"  # J3 0.6 rad away
    sysid.main(ARGS + ["--joints", "6", "--pose", pose, "--approach-speed", "0.05", "--out", str(tmp_path)])
    first = next(c for c in link.calls if c[0] == "jog")
    assert first[2] == pytest.approx(0.6 / 0.05)


def test_joint_vector_length_is_checked_before_sending(rig, tmp_path):
    link = FakeLink()
    with pytest.raises(sysid.SysidError, match="8 values, armd expects 7"):
        sysid.slow_jog(link, POSE + [0.0], 7, 0.1)
    assert link.calls == []
    # schema dof that does not match the observation: refused before anything moves
    link = rig(FakeLink(), {"dof": 8})
    with pytest.raises(sysid.SysidError, match="schema dof 8"):
        sysid.main(ARGS + ["--joints", "6", "--out", str(tmp_path)])
    assert [c for c in link.calls if c[0] in ("servo", "jog")] == []
    # a home pose of the wrong length is refused too
    link = rig(FakeLink(), {"dof": 7, "home_positions": HOME + [0.0]})
    with pytest.raises(sysid.SysidError, match="home_positions has 8 values"):
        sysid.main(ARGS + ["--joints", "6", "--unload", "--out", str(tmp_path)])
    assert [c for c in link.calls if c[0] in ("servo", "jog")] == []


def test_defaults():
    a = sysid.parse_args(["--i-am-at-the-arm"])
    assert a.abort_nm == [4.0, 4.0, 4.0, 2.5, 2.5, 2.5]
    assert a.approach_speed == 0.1 and a.unload is False
