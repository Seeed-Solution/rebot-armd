"""rebot-sysid: joint friction identification for the reBot B601-DM through armd.

Runs on a PC (numpy, matplotlib) and talks to armd as an ordinary WebSocket client: it
takes the control lease, streams servo targets that sweep one joint at a time back and
forth at constant velocities around a start pose, records q, dq and torque from the
observations (obs_hz 50) and fits per joint

    r(v, q) = Fc * sign(v) + Fv * v + offset + kq * (q - q_center)

with v from position differences (not the reported velocity, see diff_velocity)

where r is armd's residual torque (measured torque minus its gravity model, J1..J6) or,
without a gravity model, the raw torque. The kq term absorbs gravity-model error that
varies with position over the small sweep. Per direction it also fits r = a + b * v. It
writes a JSON report and one plot per joint, and prints suggested collision thresholds
= max |r - Coulomb-only fit| over all recorded samples x a safety factor (a negative
viscous term is reported as unreliable). After an abort or error the joints already
finished are still fitted and written (report field `aborted`), and a torqued, Armed arm
is jogged slowly back to the start pose (with --unload on to the schema home pose, and
unloaded only when it is then within the schema resting tolerances).

Nothing is installed on the arm side: armd only sees servo frames and commands.
See docs/simulation.md ("Friction identification") for the procedure on the real arm.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ARM_JOINTS = 6
SERVO_HZ = 50.0
OBS_HZ = 50
# Hard caps on the user-supplied sweep (rad, rad/s, rad/s²); armd checks joint limits and
# the workspace on every servo frame as well.
MAX_AMPLITUDE = 0.35
MAX_SPEED = 0.6
MAX_ACCEL = 3.0
# Default abort level on |residual change| (N·m) per joint: below half the DM4340P / DM4310
# rated torque (9 / 3 N·m). 1.0 on J4..J6 tripped on normal J4 friction swings on the real
# arm (B601-DM, 2026-11-11).
DEFAULT_ABORT_NM = [4.0, 4.0, 4.0, 2.5, 2.5, 2.5]
# Speed of the approach / return / home jogs (rad/s, largest J1..J6 change) and their
# shortest duration (s).
DEFAULT_APPROACH_SPEED = 0.1
MIN_APPROACH_S = 3.0


class SysidError(RuntimeError):
    pass


# --------------------------------------------------------------------------- armd link

class ArmdLink:
    """Minimal armd WebSocket client: lease, heartbeat, commands, servo, observation log."""

    def __init__(self, url: str, client_id: str = "rebot-sysid", obs_hz: int = OBS_HZ, timeout: float = 5.0):
        from websockets.sync.client import connect

        base = url.rstrip("/")
        ws_url = base.replace("https://", "wss://").replace("http://", "ws://") + f"/api/v1/ws?obs_hz={obs_hz}"
        self.ws = connect(ws_url, max_size=None, open_timeout=timeout)
        self.replies: queue.Queue = queue.Queue()
        self.obs: dict | None = None
        self.samples: list[dict] = []
        self.recording = False
        self.running = True
        self.request_id = 0
        self.servo_seq = 0
        self.servo_error: dict | None = None
        self.lock = threading.Lock()
        self.reader = threading.Thread(target=self._recv, daemon=True)
        self.reader.start()
        deadline = time.time() + timeout
        while self.obs is None:
            if time.time() > deadline:
                raise SysidError("no observation from armd")
            time.sleep(0.02)
        self._send({"type": "acquire_lease", "client_id": client_id})
        msg = self._reply({"lease_granted", "lease_rejected"}, timeout)
        if msg["type"] != "lease_granted":
            raise SysidError(f"lease rejected: {msg}")
        self.generation = msg["generation"]
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _recv(self):
        try:
            for text in self.ws:
                msg = json.loads(text)
                if "type" in msg:
                    if msg["type"] == "servo_reject":
                        self.servo_error = msg
                    elif msg["type"] == "lease_lost":
                        self.servo_error = msg
                    self.replies.put(msg)
                elif "positions" in msg:
                    self.obs = msg
                    if self.recording:
                        self.samples.append({
                            "t": msg.get("timestamp", 0.0), "q": msg["positions"], "dq": msg["velocities"],
                            "tau": msg["torques"], "res": msg.get("residual_torques"),
                            "cmd": msg.get("commanded_positions"), "state": msg.get("safety_state"),
                        })
        except Exception:  # closed
            self.running = False

    def _heartbeat(self):
        while self.running:
            obs = self.obs
            if obs is not None:
                try:
                    self._send({"type": "heartbeat", "generation": self.generation, "epoch_id": obs["epoch_id"]})
                except Exception:
                    return
            time.sleep(0.15)

    def _send(self, msg: dict):
        with self.lock:
            self.ws.send(json.dumps(msg))

    def _reply(self, kinds: set, timeout: float) -> dict:
        deadline = time.time() + timeout
        while True:
            try:
                msg = self.replies.get(timeout=max(0.01, deadline - time.time()))
            except queue.Empty:
                raise SysidError(f"no {kinds} from armd within {timeout} s") from None
            if msg["type"] in kinds:
                return msg

    def command(self, path: str, body: dict, timeout: float = 5.0) -> dict:
        self.request_id += 1
        self._send({"type": "command", "request_id": self.request_id, "generation": self.generation,
                    "epoch_id": self.obs["epoch_id"], "path": path, "body": body})
        while True:
            msg = self._reply({"command_ack", "command_reject"}, timeout)
            if msg.get("request_id") == self.request_id:
                break
        if msg["type"] != "command_ack":
            raise SysidError(f"{path} rejected: {msg.get('code')} {msg.get('message')}")
        return msg

    def servo(self, positions) -> None:
        if self.servo_error is not None:
            raise SysidError(f"servo stream rejected: {self.servo_error}")
        self.servo_seq += 1
        self._send({"type": "servo", "generation": self.generation, "sequence_id": self.servo_seq,
                    "epoch_id": self.obs["epoch_id"], "positions": [float(x) for x in positions]})

    def wait_action(self, action_id: int, timeout: float) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            obs = self.obs
            for rec in (obs.get("action"), obs.get("previous_action")):
                if rec and rec.get("id") == action_id and rec.get("state") != "running":
                    return rec
            time.sleep(0.05)
        raise SysidError(f"action {action_id} still running after {timeout} s")

    def jog(self, positions, duration: float | None = None) -> dict:
        body = {"target_positions": [float(x) for x in positions]}
        if duration:
            body["duration_secs"] = duration
        ack = self.command("/api/v1/motion/jog", body)
        rec = self.wait_action(ack["action_id"], 2 * float(ack.get("expected_duration_s") or 10) + 10)
        if rec.get("state") != "done":
            raise SysidError(f"jog ended {rec.get('state')}: {rec}")
        return rec

    def stop(self):
        try:
            self.command("/api/v1/motion/stop", {}, timeout=2.0)
        except Exception:
            pass

    def close(self):
        try:
            self._send({"type": "release_lease", "generation": self.generation})
            time.sleep(0.1)
        except Exception:
            pass
        self.running = False
        try:
            self.ws.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- sweep

def ramp(x0: float, x1: float, v: float, a: float, dt: float) -> list[float]:
    """Positions every dt from x0 to x1 with a trapezoidal velocity profile (v, a)."""
    d = abs(x1 - x0)
    s = 1.0 if x1 >= x0 else -1.0
    if d == 0:
        return []
    t_acc = v / a
    if a * t_acc ** 2 > d:  # triangle profile
        t_acc = math.sqrt(d / a)
        v = a * t_acc
    t_flat = (d - a * t_acc ** 2) / v
    total = 2 * t_acc + t_flat
    out = []
    for k in range(1, int(math.ceil(total / dt)) + 1):
        t = min(k * dt, total)
        if t < t_acc:
            p = 0.5 * a * t * t
        elif t < t_acc + t_flat:
            p = 0.5 * a * t_acc ** 2 + v * (t - t_acc)
        else:
            tr = total - t
            p = d - 0.5 * a * tr * tr
        out.append(x0 + s * p)
    return out


@dataclass
class Segment:
    joint: int
    speed: float
    t0: float  # armd observation timestamps bounding the segment
    t1: float


@dataclass
class SweepConfig:
    joints: list[int]
    amplitude: float
    speeds: list[float]
    cycles: int
    accel: float
    abort_nm: list[float]
    settle_s: float = 0.5


def run_sweeps(link: ArmdLink, start: list[float], cfg: SweepConfig, log=print,
               segments: list[Segment] | None = None) -> list[Segment]:
    """Sweep every joint in cfg.joints around start; returns the time windows per speed.

    Each finished speed is appended to `segments` (when given) as it ends, so a caller that
    sees an exception still has the windows recorded so far.
    """
    dt = 1.0 / SERVO_HZ
    segments = [] if segments is None else segments
    link.recording = True
    try:
        _sweep(link, start, cfg, log, segments, dt)
    finally:
        link.recording = False
    return segments


def finished_joints(segments: list[Segment], cfg: SweepConfig) -> list[int]:
    """Joints whose sweeps at every speed ended."""
    return [j for j in cfg.joints if sorted(s.speed for s in segments if s.joint == j) == sorted(cfg.speeds)]


def _sweep(link, start, cfg, log, segments, dt):
    baseline = np.array(link.obs.get("residual_torques") or link.obs["torques"][:ARM_JOINTS], dtype=float)
    period = dt
    for j in cfg.joints:
        c = start[j]
        for v in cfg.speeds:
            path = ramp(c, c + cfg.amplitude, v, cfg.accel, dt)
            for _ in range(cfg.cycles):
                path += ramp(c + cfg.amplitude, c - cfg.amplitude, v, cfg.accel, dt)
                path += ramp(c - cfg.amplitude, c + cfg.amplitude, v, cfg.accel, dt)
            path += ramp(c + cfg.amplitude, c, v, cfg.accel, dt)
            log(f"J{j + 1}: sweep ±{cfg.amplitude:.3f} rad at {v:.3f} rad/s, {cfg.cycles} cycle(s), {len(path) * dt:.1f} s")
            t0 = link.obs["timestamp"]
            target = list(start)
            next_t = time.monotonic()
            for p in path + [c] * int(cfg.settle_s / dt):
                target[j] = p
                link.servo(target)
                obs = link.obs
                state = obs.get("safety_state")
                if isinstance(state, dict) or state not in ("Armed", "Moving"):
                    raise SysidError(f"armd safety state {state} (fault {obs.get('fault')})")
                res = np.array(obs.get("residual_torques") or obs["torques"][:ARM_JOINTS], dtype=float)
                over = np.abs(res - baseline) > np.array(cfg.abort_nm)
                if over.any():
                    k = int(np.argmax(over))
                    raise SysidError(f"J{k + 1} residual changed by {res[k] - baseline[k]:+.2f} N·m "
                                     f"(abort level {cfg.abort_nm[k]} N·m)")
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()
            segments.append(Segment(j, v, t0, link.obs["timestamp"]))


# --------------------------------------------------------------------------- fit

def samples_array(samples: list[dict]) -> dict[str, np.ndarray]:
    t = np.array([s["t"] for s in samples])
    q = np.array([s["q"][:ARM_JOINTS] for s in samples])
    dq = np.array([s["dq"][:ARM_JOINTS] for s in samples])
    has_res = all(s["res"] is not None for s in samples)
    r = np.array([s["res"] if has_res else s["tau"][:ARM_JOINTS] for s in samples], dtype=float)
    return {"t": t, "q": q, "dq": dq, "v": diff_velocity(t, q), "r": r,
            "residual_source": "residual_torques" if has_res else "torques"}


def diff_velocity(t: np.ndarray, q: np.ndarray, k: int = 2) -> np.ndarray:
    """Velocity from position differences over +-k samples (80 ms at 50 Hz).

    armd samples feedback once per control tick at a fixed phase of the command period,
    so the reported instantaneous velocity can carry a bias when the motor's velocity
    ripples within a tick; positions sampled at the same phase difference to the mean.
    """
    v = np.zeros_like(q)
    if len(t) <= 2 * k:
        return v
    dt = (t[2 * k:] - t[:-2 * k])[:, None]
    v[k:-k] = (q[2 * k:] - q[:-2 * k]) / np.where(dt > 0, dt, np.inf)
    v[:k], v[-k:] = v[k], v[-k - 1]
    return v


def fit_joint(j: int, data: dict, segments: list[Segment], amplitude: float, center: float, accel: float,
              steady_tol: float = 0.1, lag_s: float = 0.15, margin: float = 0.01) -> dict:
    """Least-squares friction fit for joint j on its constant-velocity samples.

    A sample counts when its velocity is within steady_tol of the sweep speed and its
    position is clear of the turnaround ramps: |q - center| <= amplitude - v²/(2a)
    - v * lag_s - margin, so no acceleration torque enters the fit.
    """
    t, q, dq, r = data["t"], data["q"][:, j], data["v"][:, j], data["r"][:, j]
    mask = np.zeros(len(t), bool)
    for seg in segments:
        if seg.joint != j:
            continue
        in_seg = (t >= seg.t0) & (t <= seg.t1)
        steady = np.abs(np.abs(dq) - seg.speed) <= steady_tol * seg.speed
        bound = amplitude - seg.speed ** 2 / (2 * accel) - seg.speed * lag_s - margin
        mask |= in_seg & steady & (np.abs(q - center) <= bound)
    n = int(mask.sum())
    if n < 12:
        raise SysidError(f"J{j + 1}: only {n} constant-velocity samples; use more cycles or a lower speed")
    v, x, y = dq[mask], q[mask] - center, r[mask]
    sgn = np.sign(v)
    A = np.column_stack([sgn, v, np.ones(n), x])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    rmse = float(np.sqrt(np.mean((A @ coef - y) ** 2)))
    fc, fv, off, kq = (float(c) for c in coef)
    pos, neg = sgn > 0, sgn < 0
    B = np.column_stack([pos, pos * v, neg, neg * v, x]).astype(float)
    d, *_ = np.linalg.lstsq(B, y, rcond=None)
    speeds = sorted({s.speed for s in segments if s.joint == j})
    # Coulomb-only fit r = Fc * sign(v) + offset + kq * x: the collision thresholds are based
    # on it (a negative viscous term, seen on the real arm, is not physical)
    C = np.column_stack([sgn, np.ones(n), x])
    cc, *_ = np.linalg.lstsq(C, y, rcond=None)
    c_rmse = float(np.sqrt(np.mean((C @ cc - y) ** 2)))
    return {
        "joint": j + 1, "samples": n, "speeds_rad_s": speeds,
        "coulomb_nm": fc, "viscous_nm_s_rad": fv, "offset_nm": off, "position_slope_nm_rad": kq, "rmse_nm": rmse,
        "viscous_reliable": fv >= 0.0,
        **({} if fv >= 0.0 else {"viscous_note": "Fv < 0: viscous fit unreliable (not physical); thresholds use the Coulomb-only fit"}),
        "coulomb_only": {"coulomb_nm": float(cc[0]), "offset_nm": float(cc[1]), "position_slope_nm_rad": float(cc[2]), "rmse_nm": c_rmse},
        "positive": {"intercept_nm": float(d[0]), "slope_nm_s_rad": float(d[1]), "coulomb_nm": float(d[0]) - off},
        "negative": {"intercept_nm": float(d[2]), "slope_nm_s_rad": float(d[3]), "coulomb_nm": off - float(d[2])},
        "_mask": mask,
    }


def friction_model(fit: dict, v: np.ndarray, x: np.ndarray, v_eps: float = 0.01) -> np.ndarray:
    """Fitted r(v, q); sign(v) smoothed over |v| < v_eps so standstill samples are not ±Fc."""
    return fit["coulomb_nm"] * np.tanh(v / v_eps) + fit["viscous_nm_s_rad"] * v + fit["offset_nm"] + fit["position_slope_nm_rad"] * x


def coulomb_model(fit: dict, v: np.ndarray, x: np.ndarray, v_eps: float = 0.01) -> np.ndarray:
    """Coulomb-only r(v, q) (no viscous term); sign(v) smoothed as in friction_model."""
    c = fit["coulomb_only"]
    return c["coulomb_nm"] * np.tanh(v / v_eps) + c["offset_nm"] + c["position_slope_nm_rad"] * x


def thresholds(fits: dict[int, dict], data: dict, centers: list[float], factor: float, floor_nm: float) -> dict[int, dict]:
    """max |r - Coulomb-only fit| over every recorded sample (all sweeps, moving and standing)
    x factor: Coulomb friction plus the residual spread, no viscous term."""
    out = {}
    for j, fit in fits.items():
        dev = np.abs(data["r"][:, j] - coulomb_model(fit, data["v"][:, j], data["q"][:, j] - centers[j]))
        mx = float(dev.max())
        out[j] = {"basis": "coulomb_only", "max_abs_deviation_nm": mx, "p99_abs_deviation_nm": float(np.percentile(dev, 99)),
                  "factor": factor, "suggested_threshold_nm": max(mx * factor, floor_nm)}
    return out


def plot(fits: dict[int, dict], data: dict, centers: list[float], out_dir: Path) -> list[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    files = []
    for j, fit in fits.items():
        m = fit["_mask"]
        fig, ax = plt.subplots(figsize=(6, 4), dpi=110)
        ax.scatter(data["v"][:, j], data["r"][:, j], s=4, c="#bbbbbb", label="all samples")
        ax.scatter(data["v"][m, j], data["r"][m, j], s=6, c="#2a6fdb", label="constant velocity")
        v = np.linspace(data["v"][:, j].min(), data["v"][:, j].max(), 400)
        ax.plot(v, friction_model(fit, v, np.zeros_like(v)), c="#d1495b", lw=1.5,
                label=f"Fc {fit['coulomb_nm']:.3f} N·m, Fv {fit['viscous_nm_s_rad']:.3f} N·m·s/rad")
        ax.set_xlabel(f"J{j + 1} velocity from position differences (rad/s)")
        ax.set_ylabel(f"J{j + 1} {data['residual_source']} (N·m)")
        ax.legend(fontsize=7, loc="upper left")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        path = out_dir / f"friction_J{j + 1}.png"
        fig.savefig(path)
        plt.close(fig)
        files.append(str(path))
    return files


# --------------------------------------------------------------------------- CLI

def floats(text: str, n: int | None = None, name: str = "") -> list[float]:
    vals = [float(x) for x in text.split(",") if x.strip()]
    if n is not None and len(vals) != n:
        raise argparse.ArgumentTypeError(f"{name} needs {n} comma-separated values")
    return vals


def finite_in(x: float, hi: float = math.inf) -> bool:
    """x finite and in (0, hi] (NaN, inf and negative values fail)."""
    return math.isfinite(x) and 0 < x <= hi


def parse_args(argv):
    p = argparse.ArgumentParser(prog="rebot-sysid", description="Joint friction identification through armd (B601-DM).")
    p.add_argument("--url", default="http://127.0.0.1:8765", help="armd base URL (default http://127.0.0.1:8765)")
    p.add_argument("--i-am-at-the-arm", action="store_true", dest="at_arm",
                   help="required: a person watches the arm with the e-stop in reach")
    p.add_argument("--yes", action="store_true", help="do not ask for confirmation before moving")
    p.add_argument("--enable", action="store_true", help="enable torque if it is off (default: refuse)")
    p.add_argument("--pose", type=lambda s: floats(s, ARM_JOINTS, "--pose"), default=None,
                   help="start pose J1..J6 in rad (comma-separated); jogs there first. Default: the current pose")
    p.add_argument("--joints", default="1,2,3,4,5,6", help="joints to identify (default 1,2,3,4,5,6)")
    p.add_argument("--amplitude", type=float, default=0.15, help=f"sweep half-range in rad (default 0.15, max {MAX_AMPLITUDE})")
    p.add_argument("--speeds", default="0.1,0.2,0.3",
                   help=f"sweep speeds in rad/s (default 0.1,0.2,0.3, max {MAX_SPEED}); on the real arm use at most "
                        "0.08 rad/s for J1..J3 (run them separately, e.g. --joints 1,2,3 --speeds 0.04,0.08)")
    p.add_argument("--cycles", type=int, default=2, help="back-and-forth cycles per speed (default 2)")
    p.add_argument("--accel", type=float, default=1.0, help=f"turnaround acceleration rad/s² (default 1.0, max {MAX_ACCEL})")
    p.add_argument("--abort-nm", type=lambda s: floats(s, ARM_JOINTS, "--abort-nm"), default=DEFAULT_ABORT_NM,
                   help="stop if a J1..J6 residual moves this far from its value at the start (N·m, default 4,4,4,2.5,2.5,2.5)")
    p.add_argument("--safety-factor", type=float, default=1.5, help="collision threshold = max deviation x this (default 1.5)")
    p.add_argument("--threshold-floor", type=float, default=0.1, help="minimum suggested threshold in N·m (default 0.1)")
    p.add_argument("--out", default="sysid-out", help="output directory for report.json, samples.json, plots")
    p.add_argument("--unload", action="store_true",
                   help="at the end, also after an error or abort, jog slowly to the profile home pose and park / torque off "
                        "there (default: return to the start pose and hold). Needs home_positions and "
                        "resting_pose in armd's observation schema (refused before moving otherwise); when the "
                        "arm does not end within the resting tolerances of home it holds and the run fails")
    p.add_argument("--approach-speed", type=float, default=DEFAULT_APPROACH_SPEED,
                   help=f"speed of the jogs to the start pose, back to it and home: duration = largest J1..J6 change / this, "
                        f"at least {MIN_APPROACH_S:g} s (rad/s, default {DEFAULT_APPROACH_SPEED})")
    a = p.parse_args(argv)
    if not finite_in(a.approach_speed, MAX_SPEED):
        p.error(f"--approach-speed must be in (0, {MAX_SPEED}] rad/s")
    a.joints = sorted({int(x) - 1 for x in a.joints.split(",") if x.strip()})
    a.speeds = floats(a.speeds)
    if not a.joints or not all(0 <= j < ARM_JOINTS for j in a.joints):
        p.error("--joints takes values 1..6")
    if not finite_in(a.amplitude, MAX_AMPLITUDE):
        p.error(f"--amplitude must be in (0, {MAX_AMPLITUDE}] rad")
    if len(a.speeds) < 2 or not all(finite_in(v, MAX_SPEED) for v in a.speeds):
        p.error(f"--speeds needs at least two speeds in (0, {MAX_SPEED}] rad/s (Coulomb and viscous need two)")
    if not finite_in(a.accel, MAX_ACCEL):
        p.error(f"--accel must be in (0, {MAX_ACCEL}] rad/s²")
    if a.cycles < 1:
        p.error("--cycles must be >= 1")
    if not all(finite_in(x) for x in a.abort_nm):
        p.error("--abort-nm values must be finite and > 0 N·m")
    return a


def fetch_schema(url: str, timeout: float = 5.0) -> dict | None:
    """GET /api/v1/observation/schema; None when it cannot be read."""
    import urllib.request
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/api/v1/observation/schema", timeout=timeout) as r:
            return json.loads(r.read())
    except (OSError, ValueError):
        return None


def slow_jog(link, target, dof: int, speed: float, log=print) -> None:
    """Jog to `target` (dof values) with duration = largest J1..J6 change / speed, at least
    MIN_APPROACH_S. The vector length is checked before anything is sent."""
    target = [float(x) for x in target]
    if len(target) != dof:
        raise SysidError(f"joint vector has {len(target)} values, armd expects {dof} (schema dof)")
    current = link.obs["positions"]
    delta = max((abs(t - c) for t, c in zip(target[:ARM_JOINTS], current[:ARM_JOINTS])), default=0.0)
    duration = max(MIN_APPROACH_S, delta / speed)
    log(f"jog to {np.round(target[:ARM_JOINTS], 3).tolist()} in {duration:.1f} s")
    link.jog(target, duration)


def armed(link) -> bool:
    """Torque on and armd Armed or Moving (no fault, E-stop or other stop)."""
    obs = link.obs or {}
    state = obs.get("safety_state")
    return bool(obs.get("is_torqued")) and not isinstance(state, dict) and state in ("Armed", "Moving")


def unload_home(schema: dict, dof: int, current: list[float]) -> tuple[list[float] | None, list[float] | None, str | None]:
    """Home pose and J1..J6 resting tolerances for --unload, both from the observation schema.

    Returns (home, tolerances, None) or (None, None, reason). armd parks at the profile home
    at its own speed, so an unload is only safe after a slow jog to a home that armd itself
    published; a guessed home (e.g. the sweep start) could turn the park into a large fast move.
    """
    home = schema.get("home_positions")
    rest = schema.get("resting_pose")
    if not isinstance(home, list):
        return None, None, "armd publishes no home_positions in its observation schema"
    if len(home) != dof:
        raise SysidError(f"schema home_positions has {len(home)} values, dof {dof}")
    tol = rest.get("position_tolerances") if isinstance(rest, dict) else None
    if not isinstance(tol, list) or len(tol) < ARM_JOINTS:
        return None, None, "armd publishes no resting_pose.position_tolerances for J1..J6 in its observation schema"
    home = [float(x) for x in home]
    home[ARM_JOINTS:] = current[ARM_JOINTS:]  # the gripper is not moved
    return home, [float(x) for x in tol[:ARM_JOINTS]], None


def within(q, target, tol) -> bool:
    """Every J1..J6 position within its tolerance of target."""
    return all(abs(float(q[i]) - target[i]) <= tol[i] for i in range(ARM_JOINTS))


UNLOAD_REFUSED = ("not unloading: {why}. The arm stays torqued and holds its pose. "
                  "Jog it slowly to the home pose from the cockpit, then park / torque off there "
                  "(or support the arm by hand before switching torque off).")


def finish(link, a, ret: list[float], home: list[float] | None, dof: int, log=print,
           tolerances: list[float] | None = None) -> None:
    """End of a run, also after an error or abort: when the arm is torqued and armd Armed /
    Moving, jog slowly back to `ret`; with --unload jog slowly on to `home` (from the schema)
    and unload only when the arm is then within the resting tolerances of it. Otherwise
    nothing is moved; a refused unload raises SysidError (the arm holds its pose)."""
    if not armed(link):
        obs = link.obs or {}
        log(f"not moving the arm: torque {'on' if obs.get('is_torqued') else 'off'}, safety state {obs.get('safety_state')}")
        return
    deadline = time.time() + 2.0  # a stopped servo stream brakes first (Moving)
    while link.obs.get("safety_state") != "Armed" and time.time() < deadline:
        time.sleep(0.05)
    slow_jog(link, ret, dof, a.approach_speed, log)
    if not a.unload:
        return
    if home is None or tolerances is None:
        raise SysidError(UNLOAD_REFUSED.format(why="no authoritative home pose / resting tolerances from armd"))
    slow_jog(link, home, dof, a.approach_speed, log)
    time.sleep(0.5)
    q = link.obs["positions"]
    if not armed(link) or not within(q, home, tolerances):
        raise SysidError(UNLOAD_REFUSED.format(
            why=f"after the home jog the arm is at {np.round(q[:ARM_JOINTS], 3).tolist()}, not within the resting "
                f"tolerances {tolerances} of home {np.round(home[:ARM_JOINTS], 3).tolist()}"))
    ack = link.command("/api/v1/motion/unload", {})
    if ack.get("action_id"):
        link.wait_action(ack["action_id"], 60)


def write_results(a, out_dir: Path, link, segments: list[Segment], cfg: SweepConfig, start: list[float],
                  aborted: str | None) -> dict:
    """Fit the joints whose sweeps finished, write report.json and samples.json."""
    done = finished_joints(segments, cfg)
    fits, th, plots, data, errors = {}, {}, [], None, {}
    if done and link.samples:
        data = samples_array(link.samples)
        if data["residual_source"] != "residual_torques":
            print("warning: armd reports no residual_torques (no gravity model); fitting raw torques", file=sys.stderr)
        for j in done:
            try:
                fits[j] = fit_joint(j, data, segments, a.amplitude, start[j], a.accel)
            except SysidError as e:
                errors[j] = str(e)
        th = thresholds(fits, data, start, a.safety_factor, a.threshold_floor)
        plots = plot(fits, data, start, out_dir)
    report = {
        "tool": "rebot-sysid", "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "armd_url": a.url,
        "aborted": aborted,
        "joints_requested": [j + 1 for j in cfg.joints], "joints_finished": [j + 1 for j in done],
        "start_pose_rad": start[:ARM_JOINTS], "amplitude_rad": a.amplitude, "speeds_rad_s": a.speeds,
        "cycles": a.cycles, "accel_rad_s2": a.accel, "abort_nm": a.abort_nm,
        "residual_source": data["residual_source"] if data else None,
        "samples": len(link.samples),
        "model": "r = coulomb_nm * sign(v) + viscous_nm_s_rad * v + offset_nm + position_slope_nm_rad * (q - q_start)",
        "threshold_basis": "max |r - coulomb_only fit| x factor (Coulomb + residual spread, no viscous term)",
        "joints": [{**{k: v for k, v in fits[j].items() if not k.startswith("_")}, "collision": th[j]} for j in done if j in fits],
        "fit_errors": {f"J{j + 1}": e for j, e in errors.items()},
        "plots": plots,
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    (out_dir / "samples.json").write_text(json.dumps({"aborted": aborted, "segments": [s.__dict__ for s in segments],
                                                      "samples": link.samples}))
    if fits:
        print(f"\n{'joint':>5} {'Fc N·m':>8} {'Fv N·m·s/rad':>13} {'offset':>8} {'Fc+':>7} {'Fc-':>7} {'rmse':>6} {'n':>5} {'threshold':>10}")
        for j, f in fits.items():
            print(f"J{j + 1:>4} {f['coulomb_nm']:8.4f} {f['viscous_nm_s_rad']:13.4f}{'' if f['viscous_reliable'] else '?'} "
                  f"{f['offset_nm']:8.4f} {f['positive']['coulomb_nm']:7.4f} {f['negative']['coulomb_nm']:7.4f} "
                  f"{f['rmse_nm']:6.4f} {f['samples']:5d} {th[j]['suggested_threshold_nm']:10.3f}")
        if not all(f["viscous_reliable"] for f in fits.values()):
            print("?: Fv < 0, viscous fit unreliable")
        print(f"\nsuggested collision thresholds (N·m, max |residual - Coulomb-only fit| x {a.safety_factor}): "
              f"{[round(th[j]['suggested_threshold_nm'], 3) for j in fits]}")
    if aborted:
        print(f"aborted: {aborted}; joints finished and fitted: {[j + 1 for j in fits]}")
    print(f"report: {out_dir / 'report.json'}")
    return report


def main(argv: list[str] | None = None) -> dict:
    """Run the identification. After an error or abort the joints already finished are
    fitted and written (report `aborted`), the arm is brought back (see finish) and the
    error is raised again."""
    a = parse_args(argv)
    if not a.at_arm:
        sys.exit("rebot-sysid moves the arm. Run it only with a person at the arm and the e-stop in reach, "
                 "and pass --i-am-at-the-arm.")
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    schema = fetch_schema(a.url) or {}
    link = ArmdLink(a.url)
    cfg = SweepConfig(a.joints, a.amplitude, a.speeds, a.cycles, a.accel, a.abort_nm)
    segments: list[Segment] = []
    start: list[float] | None = None
    ret: list[float] | None = None
    home = None
    tolerances = None
    dof: int | None = None
    swept = False
    try:
        obs = link.obs
        dof = int(schema.get("dof") or len(obs["positions"]))
        if len(obs["positions"]) != dof:
            raise SysidError(f"observation has {len(obs['positions'])} joints, schema dof {dof}")
        current = list(obs["positions"])
        if a.unload:
            home, tolerances, why = unload_home(schema, dof, current)
            if why is not None:  # refused before anything moves
                raise SysidError(f"--unload needs the home pose and resting tolerances from armd: {why}. "
                                 "Update armd, or run without --unload and park from the cockpit")
        if not obs.get("is_torqued"):
            if not a.enable:
                raise SysidError("torque is off: enable it in the cockpit or pass --enable")
            link.command("/api/v1/torque", {"enable": True})
            time.sleep(0.3)
        start = list(current)
        if a.pose is not None:
            start[:ARM_JOINTS] = a.pose
        per_joint = sum(4 * a.amplitude * a.cycles / v + 2 * a.amplitude / v + 0.5 for v in a.speeds)
        print(f"start pose (rad): {np.round(start[:ARM_JOINTS], 3).tolist()}")
        for j in a.joints:
            print(f"  J{j + 1}: {start[j] - a.amplitude:+.3f} .. {start[j] + a.amplitude:+.3f} rad at {a.speeds} rad/s")
        print(f"about {per_joint * len(a.joints):.0f} s of motion; abort levels {a.abort_nm} N·m; "
              f"residual source: {'residual_torques' if obs.get('residual_torques') is not None else 'torques (no gravity model)'}")
        if not a.yes and input("move the arm? type 'yes': ").strip() != "yes":
            raise SysidError("cancelled")
        if a.pose is not None:
            ret = list(current)  # an error during the approach returns to where the arm was
            slow_jog(link, start, dof, a.approach_speed)
            time.sleep(0.5)
        start = list(link.obs["positions"])  # sweep around where the arm actually is
        ret = list(start)
        swept = True
        run_sweeps(link, start, cfg, segments=segments)
    except BaseException as e:
        link.stop()
        aborted = f"{type(e).__name__}: {e}" if not isinstance(e, SysidError) else str(e)
        try:
            if swept:
                write_results(a, out_dir, link, segments, cfg, start, aborted)
        except Exception as we:  # never mask the original error
            print(f"rebot-sysid: writing the partial results failed: {we}", file=sys.stderr)
        try:
            if ret is not None and dof is not None:
                finish(link, a, ret, home, dof, log=lambda m: print(f"recovery: {m}", file=sys.stderr),
                       tolerances=tolerances)
        except BaseException as fe:
            print(f"rebot-sysid: recovery failed: {fe}", file=sys.stderr)
        link.close()
        raise
    try:
        report = write_results(a, out_dir, link, segments, cfg, start, None)
        finish(link, a, ret, home, dof, tolerances=tolerances)
        return report
    except BaseException:
        link.stop()
        raise
    finally:
        link.close()


def cli() -> None:
    try:
        main()
    except SysidError as e:
        sys.exit(f"rebot-sysid: {e} (motion stopped)")
    except KeyboardInterrupt:
        sys.exit("rebot-sysid: interrupted (motion stopped)")


if __name__ == "__main__":
    cli()
