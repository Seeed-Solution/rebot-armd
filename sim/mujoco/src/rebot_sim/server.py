"""rebot-sim: MuJoCo B601-DM simulator for `armd --follower sim`.

Physics runs at 500 Hz in real time. armd sends command datagrams; the simulator
answers every second step (250 Hz) with joint state to the address of the last
datagram. Wire protocol: see src/sim_driver.rs and docs/simulation.md.

Motor model per joint, mirroring the Damiao modes armd uses:
- POS_VEL: v_des = clip(KPOS * (q_target - q), +-vlim); torque = KV * (v_des - dq)
  plus the model's bias torque (gravity, Coriolis) on J1..J6, which stands in for the
  motor's integrator. v_des is also capped at the motor's no-load speed; torque is
  limited to the motor's peak torque. A joint that rests on its mechanical stop (the joint
  range end, within STOP_EPS) with its target at that stop and gravity pushing it into the
  stop gets no bias torque: the stop carries the load, as on the real arm at the folded home
  pose (J2 / J3 on their supports, motor torque near 0).
- MIT: torque = kp * (q_target - q) + kd * (dq_target - dq) + tau_ff.
- Disabled: no torque (the arm falls under gravity, as on hardware).
Reported torque is the total motor torque; temperatures are a constant 35 C.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import random
import socket
import sys
import threading
import time

import mujoco
import numpy as np

from .camera import CameraMount
from .model import DOF, Indexer, load_model
from .scene import Scene, parse_objects

PROTOCOL_VERSION = 1
STATE_EVERY = 2  # physics steps per state datagram (250 Hz at 500 Hz physics)
TEMPERATURE_C = 35.0
# POS_VEL loop gains (assumed; not identified on the motors). KPOS in 1/s, KV in N·m·s/rad.
KPOS = np.array([25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0])
KV = np.array([8.0, 8.0, 8.0, 1.5, 1.5, 1.5, 0.5])
# Damiao datasheet values (Seeed wiki damiao_series, Damiao user manuals): J1..J3
# DM-J4340P-2EC, J4..J7 DM-J4310-2EC V1.1. Peak torque in N·m, no-load speed at 24 V in
# rad/s (52 rpm and 200 rpm), both output side.
TORQUE_LIMIT = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0, 7.0])
NO_LOAD_SPEED = np.array([52.0] * 3 + [200.0] * 4) * 2 * np.pi / 60
# A joint within this distance (rad) of a range end, with its target there too, rests on
# the stop (see the module docstring).
STOP_EPS = 0.005


def _finite(v) -> float | None:
    """`v` as a finite float (bool and non-numbers are None)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    f = float(v)
    return f if np.isfinite(f) else None


class Motors:
    """Damiao-like motor state and control law for the 7 actuators."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model, self.data = model, data
        self.idx = Indexer(model)
        self.enabled = False
        self.mode = ["pos_vel"] * DOF
        self.q_t = np.zeros(DOF)
        self.vlim = np.full(DOF, 2.0)
        self.dq_t = np.zeros(DOF)
        self.kp = np.zeros(DOF)
        self.kd = np.zeros(DOF)
        self.tau_ff = np.zeros(DOF)
        self.extra = np.zeros(DOF)
        # generalized-force mapping of each actuator (gear) on its joint dof
        self.gear = model.actuator_gear[:, 0].copy()
        self.dofs = np.array([model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)])
        # J1..J6 range ends (rad, motor angle = joint angle); +-inf for unlimited joints
        jnts = [model.actuator_trnid[i, 0] for i in range(6)]
        self.stop_lo = np.array([model.jnt_range[j, 0] if model.jnt_limited[j] else -np.inf for j in jnts])
        self.stop_hi = np.array([model.jnt_range[j, 1] if model.jnt_limited[j] else np.inf for j in jnts])

    def set_enabled(self, enabled: bool) -> None:
        if enabled and not self.enabled:
            # On enable the motors hold where they are until a target arrives.
            self.q_t = self.idx.motor_positions(self.data)
            self.mode = ["pos_vel"] * DOF
        self.enabled = enabled

    @staticmethod
    def parse_targets(targets) -> list[tuple] | None:
        """Validated targets (one tuple per joint), or None when any field is malformed."""
        if not isinstance(targets, list) or len(targets) != DOF:
            return None
        out = []
        for t in targets:
            if not isinstance(t, dict):
                return None
            mode = t.get("mode")
            if mode == "pos_vel":
                q, vlim = _finite(t.get("q")), _finite(t.get("vlim", 2.0))
                if q is None or vlim is None:
                    return None
                out.append((mode, q, abs(vlim)))
            elif mode == "mit":
                vals = [_finite(t.get("q"))] + [_finite(t.get(k, 0.0)) for k in ("dq", "kp", "kd", "tau")]
                if any(v is None for v in vals):
                    return None
                out.append((mode, *vals))
            else:
                return None
        return out

    def set_targets(self, targets) -> bool:
        """Apply all targets, or none of them when one is malformed."""
        parsed = self.parse_targets(targets)
        if parsed is None:
            return False
        for i, t in enumerate(parsed):
            if t[0] == "pos_vel":
                self.mode[i], self.q_t[i], self.vlim[i] = t
            else:
                self.mode[i], self.q_t[i], self.dq_t[i], self.kp[i], self.kd[i], self.tau_ff[i] = t
        return True

    def apply(self) -> None:
        """Set actuator gains / controls and applied forces for the next step."""
        m, d = self.model, self.data
        q = self.idx.motor_positions(d)
        d.qfrc_applied[self.dofs] = 0.0
        if not self.enabled:
            m.actuator_gainprm[:, 0] = 0.0
            m.actuator_biasprm[:, 2] = 0.0
            d.ctrl[:] = 0.0
            m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1] = -TORQUE_LIMIT, TORQUE_LIMIT
            self.extra = np.zeros(DOF)
            return
        extra = np.zeros(DOF)  # torque applied outside the velocity servo (N·m, motor side)
        kv, ctrl = np.zeros(DOF), np.zeros(DOF)
        bias = d.qfrc_bias[self.dofs[:6]]
        for i in range(DOF):
            if self.mode[i] == "pos_vel":
                vmax = min(self.vlim[i], NO_LOAD_SPEED[i])
                ctrl[i] = np.clip(KPOS[i] * (self.q_t[i] - q[i]), -vmax, vmax)
                kv[i] = KV[i]
                if i < 6 and not self.on_stop(i, q[i], bias[i]):
                    extra[i] = bias[i]
            else:
                ctrl[i] = self.dq_t[i]
                kv[i] = max(self.kd[i], 0.0)
                extra[i] = self.kp[i] * (self.q_t[i] - q[i]) + self.tau_ff[i]
        extra = np.clip(extra, -TORQUE_LIMIT, TORQUE_LIMIT)
        m.actuator_gainprm[:, 0] = kv
        m.actuator_biasprm[:, 2] = -kv
        # the servo gets whatever torque the limit leaves after the extra term
        m.actuator_forcerange[:, 0] = -TORQUE_LIMIT - extra
        m.actuator_forcerange[:, 1] = TORQUE_LIMIT - extra
        d.ctrl[:] = ctrl
        d.qfrc_applied[self.dofs] = self.gear * extra
        self.extra = extra

    def on_stop(self, i: int, q: float, bias: float) -> bool:
        """Joint i (J1..J6) rests on a range end: position and target at it and the load
        (bias torque: what a motor needs to hold) pushing into it."""
        hi, lo = self.stop_hi[i] - STOP_EPS, self.stop_lo[i] + STOP_EPS
        return (q >= hi and self.q_t[i] >= hi and bias < 0) or (q <= lo and self.q_t[i] <= lo and bias > 0)

    def reported_torque(self) -> np.ndarray:
        if not self.enabled:
            return np.zeros(DOF)
        return self.data.actuator_force[:DOF] + self.extra


def set_joint_friction(model: mujoco.MjModel, damping=None, frictionloss=None) -> None:
    """Override J1..J6 joint damping (N·m·s/rad) and friction loss (N·m), 6 values each."""
    dofs = Indexer(model).qvel[:6]
    if damping is not None:
        model.dof_damping[dofs] = damping
    if frictionloss is not None:
        model.dof_frictionloss[dofs] = frictionloss


class Simulator:
    def __init__(self, port: int, host: str = "127.0.0.1", viewer: bool = False, damping=None, frictionloss=None,
                 torque_report: str = "instant", objects: str = "cube", seed: int | None = None,
                 camera_mount: CameraMount | None = None):
        self.objects = parse_objects(objects)
        self.mount = camera_mount
        self.has_meshes = bool(os.environ.get("REBOT_DEVARM_DIR"))
        self.model = load_model(objects=self.objects, camera=camera_mount)
        # physics runs under this lock; the camera thread and the HTTP routes read under it
        self.lock = threading.Lock()
        self.stopping = False
        self._resets: list[tuple[int | None, threading.Event, dict]] = []
        self.shared = None  # SharedState for the camera process
        self.cam_conn = None
        self.cam_proc = None
        self.torque_report = torque_report
        # --torque-report mean: torques of the last physics steps and the command period in steps
        self.tau_hist: collections.deque = collections.deque(maxlen=256)
        self.steps_since_cmd, self.cmd_period = 0, 10
        set_joint_friction(self.model, damping, frictionloss)
        self.data = mujoco.MjData(self.model)
        self.motors = Motors(self.model, self.data)
        self.scene = Scene(self.model, self.objects)
        self.scene.reset(self.data, seed)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.peer = None
        self.ack = 0
        self.seq = 0
        self.session = random.getrandbits(62)
        self.rejected = 0
        self.viewer = None
        if viewer:
            from mujoco import viewer as mj_viewer  # needs a display; imported only here
            self.viewer = mj_viewer.launch_passive(self.model, self.data)

    def handle(self, payload: bytes, addr) -> None:
        """Apply one datagram. Malformed packets change nothing and are counted in `rejected`."""
        try:
            msg = json.loads(payload)
        except (ValueError, TypeError, UnicodeDecodeError, RecursionError):
            msg = None
        if not isinstance(msg, dict) or msg.get("v") != PROTOCOL_VERSION or msg.get("type") not in ("cmd", "poll"):
            self.rejected += 1
            return
        if msg["type"] == "poll":
            self.peer = addr
            return
        enable, seq, targets = msg.get("enable", False), msg.get("seq", 0), msg.get("targets")
        parsed = None if targets is None else Motors.parse_targets(targets)
        if (not isinstance(enable, bool) or isinstance(seq, bool) or not isinstance(seq, int) or seq < 0
                or (targets is not None and parsed is None)):
            self.rejected += 1
            return
        self.peer = addr
        self.motors.set_enabled(enable)
        if targets is not None:
            self.motors.set_targets(targets)
        self.ack = seq
        if self.steps_since_cmd:
            self.cmd_period = min(self.steps_since_cmd, self.tau_hist.maxlen)
        self.steps_since_cmd = 0

    def poll_socket(self) -> None:
        while True:
            try:
                payload, addr = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                return
            except ConnectionError:
                continue
            self.handle(payload, addr)

    def state_datagram(self) -> bytes:
        idx, d = self.motors.idx, self.data
        self.seq += 1
        state = {
            "v": PROTOCOL_VERSION, "type": "state", "session": self.session, "seq": self.seq,
            "ack": self.ack, "enabled": self.motors.enabled,
            "q": idx.motor_positions(d).round(6).tolist(),
            "dq": idx.motor_velocities(d).round(6).tolist(),
            "tau": self.torque_feedback().round(5).tolist(),
            "temp": [TEMPERATURE_C] * DOF,
            "t": round(d.time, 4),
        }
        return json.dumps(state, separators=(",", ":")).encode()

    def torque_feedback(self) -> np.ndarray:
        """Instantaneous motor torque, or with --torque-report mean its mean over the last command
        period (sliding window of the physics steps between the two latest command datagrams)."""
        if self.torque_report == "mean" and self.tau_hist:
            return np.mean(list(self.tau_hist)[-self.cmd_period:], axis=0)
        return self.motors.reported_torque()

    def step(self) -> None:
        with self.lock:
            if self._resets:
                for seed, done, out in self._resets:
                    out.update(self._reset_locked(seed))
                    done.set()
                self._resets.clear()
            self.motors.apply()
            mujoco.mj_step(self.model, self.data)
        if self.shared is not None:
            self.shared.write(self.data)
        if self.torque_report == "mean":
            self.tau_hist.append(self.motors.reported_torque())
            self.steps_since_cmd += 1

    # ---- scene control (HTTP threads)
    def request_reset(self, seed: int | None, timeout: float = 2.0) -> dict:
        """Place the objects for `seed` at the next physics step; returns the state after it."""
        done, out = threading.Event(), {}
        with self.lock:
            self._resets.append((seed, done, out))
        if not done.wait(timeout):  # physics loop not running (tests): apply here
            with self.lock:
                if (seed, done, out) in self._resets:
                    self._resets.remove((seed, done, out))
                    out.update(self._reset_locked(seed))
        return out

    def _reset_locked(self, seed) -> dict:
        """Reset the scene; a seed the scene rejects gives {"error": ...} and changes nothing."""
        try:
            self.scene.reset(self.data, seed)
        except (ValueError, TypeError, OverflowError) as exc:
            return {"error": f"reset with seed {seed!r} failed: {exc}"}
        return self._state_locked()

    def _state_locked(self) -> dict:
        d = self.data
        return {"t": round(d.time, 4), "seed": self.scene.seed, "enabled": self.motors.enabled,
                "q": self.motors.idx.motor_positions(d).round(5).tolist(),
                "tcp": [round(float(v), 5) for v in d.body("end_link").xpos],
                "objects": self.scene.state(d)}

    def sim_state(self) -> dict:
        with self.lock:
            return self._state_locked()

    def start_camera(self, fps: int, size: tuple[int, int], host: str, video_port: int, perception_port: int,
                     scene_stream: bool = False, objects: str = "cube", timeout: float = 60.0) -> dict:
        """Spawn the camera process (perception.camera_main); returns its ready message."""
        import multiprocessing as mp

        from .perception import SharedState, camera_main

        self.shared = SharedState(self.model.nq, self.model.nv)
        self.shared.write(self.data)
        ctx = mp.get_context("spawn")
        self.cam_conn, child = ctx.Pipe()
        ready_r, ready_w = ctx.Pipe(duplex=False)
        m = self.mount
        cfg = {"fps": fps, "size": list(size), "host": host, "video_port": video_port,
               "perception_port": perception_port, "scene_stream": scene_stream, "objects": objects,
               "mount": [m.flip, m.rot_deg, m.tilt_deg, m.offset_m], "has_meshes": self.has_meshes}
        self.cam_proc = ctx.Process(target=camera_main, name="rebot-sim-camera", daemon=True,
                                    args=(cfg, self.shared.shm.name, self.model.nq, self.model.nv, child, ready_w, os.getpid()))
        self.cam_proc.start()
        if not ready_r.poll(timeout):
            return {"error": "camera process did not start"}
        return ready_r.recv()

    def service_camera(self) -> None:
        """Answer the camera process's /sim requests (physics thread)."""
        while self.cam_conn is not None and self.cam_conn.poll():
            try:
                msg = self.cam_conn.recv()
            except EOFError:
                self.cam_conn = None
                return
            rid, op = msg[0], msg[1]
            if op == "reset":
                self.cam_conn.send((rid, self._reset_locked(msg[2])))
            else:
                self.cam_conn.send((rid, self._state_locked()))

    def run(self, duration: float | None = None, stats_every: float = 0.0) -> None:
        dt = self.model.opt.timestep
        start = time.perf_counter()
        next_t = start
        steps, last_stats, last_steps = 0, start, 0
        while (duration is None or time.perf_counter() - start < duration) and not self.stopping:
            self.poll_socket()
            self.step()
            steps += 1
            if steps % 10 == 0 and self.cam_conn is not None:
                self.service_camera()
            if steps % STATE_EVERY == 0 and self.peer is not None:
                try:
                    self.sock.sendto(self.state_datagram(), self.peer)
                except OSError:
                    pass
            if self.viewer is not None and steps % 16 == 0:
                if not self.viewer.is_running():
                    return
                self.viewer.sync()
            next_t += dt
            now = time.perf_counter()
            if next_t > now:
                time.sleep(next_t - now)
            elif now - next_t > 0.05:
                next_t = now  # fell behind by > 50 ms: drop the backlog instead of racing
            if stats_every and now - last_stats >= stats_every:
                rate = (steps - last_steps) / (now - last_stats)
                obj = self.objects[0].name
                cube = self.data.body(obj).xpos
                cam = ""
                if self.shared is not None:
                    cam = f" camera={self.shared.arr[2]:.0f} frames {self.shared.arr[3]:.1f} ms/frame"
                print(f"t={self.data.time:8.2f}s rate={rate:6.1f} Hz enabled={self.motors.enabled} "
                      f"q={np.round(self.motors.idx.motor_positions(self.data), 3).tolist()} "
                      f"{obj}=({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) rejected={self.rejected}{cam}", flush=True)
                last_stats, last_steps = now, steps


def benchmark(steps: int) -> float:
    """Physics steps per second without real-time pacing (motors enabled, holding)."""
    model = load_model()
    data = mujoco.MjData(model)
    motors = Motors(model, data)
    mujoco.mj_forward(model, data)
    motors.set_enabled(True)
    t0 = time.perf_counter()
    for _ in range(steps):
        motors.apply()
        mujoco.mj_step(model, data)
    return steps / (time.perf_counter() - t0)


def six_floats(text: str) -> list[float]:
    vals = [float(x) for x in text.split(",")]
    if len(vals) != 6 or not all(np.isfinite(v) and v >= 0 for v in vals):
        raise argparse.ArgumentTypeError("needs 6 comma-separated non-negative values")
    return vals


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="rebot-sim", description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=47600, help="UDP port (default 47600)")
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default 127.0.0.1)")
    parser.add_argument("--headless", action="store_true", help="no viewer (the default; kept for scripts)")
    parser.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer (needs a display)")
    parser.add_argument("--stats", type=float, default=0.0, metavar="S", help="print step rate and state every S seconds")
    parser.add_argument("--duration", type=float, default=None, help="exit after this many seconds")
    parser.add_argument("--benchmark", type=int, default=0, metavar="N", help="time N unpaced physics steps and exit")
    parser.add_argument("--damping", type=six_floats, default=None, metavar="D1,..,D6",
                        help="J1..J6 joint damping in N·m·s/rad (default: scene values)")
    parser.add_argument("--frictionloss", type=six_floats, default=None, metavar="F1,..,F6",
                        help="J1..J6 joint friction loss in N·m (default: scene values)")
    parser.add_argument("--torque-report", choices=("instant", "mean"), default="instant",
                        help="torque in state datagrams: instantaneous (default) or mean over the last command period")
    parser.add_argument("--objects", default="cube", help="scene objects, 1..3 of cube, cup, block (default cube)")
    parser.add_argument("--seed", type=int, default=None, help="object placement seed (default: fixed layout)")
    parser.add_argument("--camera", action="store_true",
                        help="render the wrist camera: H.264 stream, perception API, /sim routes")
    parser.add_argument("--video-port", type=int, default=8001, help="camera WebSocket port (default 8001)")
    parser.add_argument("--perception-port", type=int, default=8090, help="perception / sim HTTP port (default 8090)")
    parser.add_argument("--http-host", default="127.0.0.1", help="bind address of the camera and HTTP servers")
    parser.add_argument("--fps", type=int, default=15, help="camera frames per second (default 15)")
    parser.add_argument("--camera-size", default="1280x720", choices=("1280x720", "640x360"), help="stream size")
    parser.add_argument("--scene-stream", action="store_true", help="also stream the overview camera at /scene")
    parser.add_argument("--profile", default=None, help="profile JSON whose camera_mount is used (default: the kit profile)")
    parser.add_argument("--camera-rot-deg", type=float, default=None, help="override camera_mount.rot_deg")
    parser.add_argument("--camera-tilt-deg", type=float, default=None,
                        help="override camera_mount.tilt_deg (armd sign: negative turns the axis toward the jaws)")
    parser.add_argument("--camera-offset", default=None, metavar="X,Y,Z",
                        help="lens centre in end_link (m); default camera_mount.offset_m, else the v8 holder value")
    args = parser.parse_args(argv)
    if not 1 <= args.fps <= 30:
        parser.error("--fps must be 1..30")
    try:
        parse_objects(args.objects)
    except ValueError as exc:
        parser.error(f"--objects: {exc}")
    if args.seed is not None and args.seed < 0:
        parser.error("--seed must be non-negative")
    mount = CameraMount.from_profile(args.profile) if args.profile else CameraMount.from_profile()
    if args.camera_rot_deg is not None or args.camera_tilt_deg is not None or args.camera_offset:
        off = mount.offset_m
        if args.camera_offset:
            off = tuple(float(v) for v in args.camera_offset.split(","))
            if len(off) != 3:
                parser.error("--camera-offset needs 3 values")
        mount = CameraMount(mount.flip, mount.rot_deg if args.camera_rot_deg is None else args.camera_rot_deg,
                            mount.tilt_deg if args.camera_tilt_deg is None else args.camera_tilt_deg, off)
    if args.benchmark:
        print(f"{benchmark(args.benchmark):.0f} physics steps/s (timestep 2 ms, unpaced)")
        return
    viewer = args.viewer and not args.headless
    if viewer and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or sys.platform == "darwin"):
        parser.error("--viewer needs a display (DISPLAY / WAYLAND_DISPLAY not set)")
    sim = Simulator(args.port, args.host, viewer=viewer, damping=args.damping, frictionloss=args.frictionloss,
                    torque_report=args.torque_report, objects=args.objects, seed=args.seed,
                    camera_mount=mount if args.camera else None)
    print(f"rebot-sim listening on udp://{args.host}:{args.port} (session {sim.session})", flush=True)
    if args.camera:
        w, h = (int(v) for v in args.camera_size.split("x"))
        ready = sim.start_camera(args.fps, (w, h), args.http_host, args.video_port, args.perception_port,
                                 args.scene_stream, args.objects)
        if "error" in ready:
            print(f"rebot-sim: {ready['error']}", file=sys.stderr, flush=True)
            sys.exit(1)
        print(f"camera: {w}x{h} @ {args.fps} fps ({ready['backend']}, own process), mount rot {mount.rot_deg:g} tilt {mount.tilt_deg:g} "
              f"offset {mount.offset.round(4).tolist()}; video ws://{args.http_host}:{args.video_port}/, "
              f"perception http://{args.http_host}:{args.perception_port}", flush=True)
    try:
        sim.run(args.duration, args.stats)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stopping = True


if __name__ == "__main__":
    main()
