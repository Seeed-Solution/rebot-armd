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
= max |r - friction fit| over all recorded samples x a safety factor.

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
# Default abort level on |residual| (N·m) per joint: about 1/3 of the DM4340P / DM4310
# rated torque (9 / 3 N·m).
DEFAULT_ABORT_NM = [3.0, 3.0, 3.0, 1.0, 1.0, 1.0]


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


def run_sweeps(link: ArmdLink, start: list[float], cfg: SweepConfig, log=print) -> list[Segment]:
    """Sweep every joint in cfg.joints around start; returns the time windows per speed."""
    dt = 1.0 / SERVO_HZ
    segments: list[Segment] = []
    link.recording = True
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
    link.recording = False
    return segments


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
    return {
        "joint": j + 1, "samples": n, "speeds_rad_s": speeds,
        "coulomb_nm": fc, "viscous_nm_s_rad": fv, "offset_nm": off, "position_slope_nm_rad": kq, "rmse_nm": rmse,
        "positive": {"intercept_nm": float(d[0]), "slope_nm_s_rad": float(d[1]), "coulomb_nm": float(d[0]) - off},
        "negative": {"intercept_nm": float(d[2]), "slope_nm_s_rad": float(d[3]), "coulomb_nm": off - float(d[2])},
        "_mask": mask,
    }


def friction_model(fit: dict, v: np.ndarray, x: np.ndarray, v_eps: float = 0.01) -> np.ndarray:
    """Fitted r(v, q); sign(v) smoothed over |v| < v_eps so standstill samples are not ±Fc."""
    return fit["coulomb_nm"] * np.tanh(v / v_eps) + fit["viscous_nm_s_rad"] * v + fit["offset_nm"] + fit["position_slope_nm_rad"] * x


def thresholds(fits: dict[int, dict], data: dict, centers: list[float], factor: float, floor_nm: float) -> dict[int, dict]:
    """max |r - fit| over every recorded sample (all sweeps, moving and standing) x factor."""
    out = {}
    for j, fit in fits.items():
        dev = np.abs(data["r"][:, j] - friction_model(fit, data["v"][:, j], data["q"][:, j] - centers[j]))
        mx = float(dev.max())
        out[j] = {"max_abs_deviation_nm": mx, "p99_abs_deviation_nm": float(np.percentile(dev, 99)),
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
    p.add_argument("--speeds", default="0.1,0.2,0.3", help=f"sweep speeds in rad/s (default 0.1,0.2,0.3, max {MAX_SPEED})")
    p.add_argument("--cycles", type=int, default=2, help="back-and-forth cycles per speed (default 2)")
    p.add_argument("--accel", type=float, default=1.0, help=f"turnaround acceleration rad/s² (default 1.0, max {MAX_ACCEL})")
    p.add_argument("--abort-nm", type=lambda s: floats(s, ARM_JOINTS, "--abort-nm"), default=DEFAULT_ABORT_NM,
                   help="stop if a J1..J6 residual moves this far from its value at the start (N·m, default 3,3,3,1,1,1)")
    p.add_argument("--safety-factor", type=float, default=1.5, help="collision threshold = max deviation x this (default 1.5)")
    p.add_argument("--threshold-floor", type=float, default=0.1, help="minimum suggested threshold in N·m (default 0.1)")
    p.add_argument("--out", default="sysid-out", help="output directory for report.json, samples.json, plots")
    p.add_argument("--unload", action="store_true", help="park and torque off at the end (default: return to the start pose and hold)")
    a = p.parse_args(argv)
    a.joints = sorted({int(x) - 1 for x in a.joints.split(",") if x.strip()})
    a.speeds = floats(a.speeds)
    if not a.joints or not all(0 <= j < ARM_JOINTS for j in a.joints):
        p.error("--joints takes values 1..6")
    if not 0 < a.amplitude <= MAX_AMPLITUDE:
        p.error(f"--amplitude must be in (0, {MAX_AMPLITUDE}] rad")
    if len(a.speeds) < 2 or not all(0 < v <= MAX_SPEED for v in a.speeds):
        p.error(f"--speeds needs at least two speeds in (0, {MAX_SPEED}] rad/s (Coulomb and viscous need two)")
    if not 0 < a.accel <= MAX_ACCEL:
        p.error(f"--accel must be in (0, {MAX_ACCEL}] rad/s²")
    if a.cycles < 1:
        p.error("--cycles must be >= 1")
    return a


def main(argv: list[str] | None = None) -> dict:
    a = parse_args(argv)
    if not a.at_arm:
        sys.exit("rebot-sysid moves the arm. Run it only with a person at the arm and the e-stop in reach, "
                 "and pass --i-am-at-the-arm.")
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    link = ArmdLink(a.url)
    try:
        obs = link.obs
        if not obs.get("is_torqued"):
            if not a.enable:
                raise SysidError("torque is off: enable it in the cockpit or pass --enable")
            link.command("/api/v1/torque", {"enable": True})
            time.sleep(0.3)
        current = list(link.obs["positions"])
        start = list(current)
        if a.pose is not None:
            start[:ARM_JOINTS] = a.pose
        cfg = SweepConfig(a.joints, a.amplitude, a.speeds, a.cycles, a.accel, a.abort_nm)
        per_joint = sum(4 * a.amplitude * a.cycles / v + 2 * a.amplitude / v + 0.5 for v in a.speeds)
        print(f"start pose (rad): {np.round(start[:ARM_JOINTS], 3).tolist()}")
        for j in a.joints:
            print(f"  J{j + 1}: {start[j] - a.amplitude:+.3f} .. {start[j] + a.amplitude:+.3f} rad at {a.speeds} rad/s")
        print(f"about {per_joint * len(a.joints):.0f} s of motion; abort levels {a.abort_nm} N·m; "
              f"residual source: {'residual_torques' if obs.get('residual_torques') is not None else 'torques (no gravity model)'}")
        if not a.yes and input("move the arm? type 'yes': ").strip() != "yes":
            raise SysidError("cancelled")
        if a.pose is not None:
            link.jog(start)
            time.sleep(0.5)
        start = list(link.obs["positions"])  # sweep around where the arm actually is
        segments = run_sweeps(link, start, cfg)
        link.jog(start)
        data = samples_array(link.samples)
        if data["residual_source"] != "residual_torques":
            print("warning: armd reports no residual_torques (no gravity model); fitting raw torques", file=sys.stderr)
        fits = {j: fit_joint(j, data, segments, a.amplitude, start[j], a.accel) for j in a.joints}
        th = thresholds(fits, data, start, a.safety_factor, a.threshold_floor)
        plots = plot(fits, data, start, out_dir)
        report = {
            "tool": "rebot-sysid", "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "armd_url": a.url,
            "start_pose_rad": start[:ARM_JOINTS], "amplitude_rad": a.amplitude, "speeds_rad_s": a.speeds,
            "cycles": a.cycles, "accel_rad_s2": a.accel, "residual_source": data["residual_source"],
            "samples": len(link.samples),
            "model": "r = coulomb_nm * sign(v) + viscous_nm_s_rad * v + offset_nm + position_slope_nm_rad * (q - q_start)",
            "joints": [{**{k: v for k, v in fits[j].items() if not k.startswith("_")}, "collision": th[j]} for j in a.joints],
            "plots": plots,
        }
        (out_dir / "report.json").write_text(json.dumps(report, indent=2))
        (out_dir / "samples.json").write_text(json.dumps({"segments": [s.__dict__ for s in segments], "samples": link.samples}))
        print(f"\n{'joint':>5} {'Fc N·m':>8} {'Fv N·m·s/rad':>13} {'offset':>8} {'Fc+':>7} {'Fc-':>7} {'rmse':>6} {'n':>5} {'threshold':>10}")
        for j in a.joints:
            f = fits[j]
            print(f"J{j + 1:>4} {f['coulomb_nm']:8.4f} {f['viscous_nm_s_rad']:13.4f} {f['offset_nm']:8.4f} "
                  f"{f['positive']['coulomb_nm']:7.4f} {f['negative']['coulomb_nm']:7.4f} {f['rmse_nm']:6.4f} "
                  f"{f['samples']:5d} {th[j]['suggested_threshold_nm']:10.3f}")
        print(f"\nsuggested collision thresholds (N·m, max |residual - fit| x {a.safety_factor}): "
              f"{[round(th[j]['suggested_threshold_nm'], 3) for j in a.joints]}")
        print(f"report: {out_dir / 'report.json'}")
        if a.unload:
            ack = link.command("/api/v1/motion/unload", {})
            if ack.get("action_id"):
                link.wait_action(ack["action_id"], 60)
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
