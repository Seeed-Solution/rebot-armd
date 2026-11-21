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
import uuid

import mujoco
import numpy as np

from .camera import PROFILE, SO101_PROFILE, CameraMount
from .model import DOF, MODEL_OFFSETS_RAD, Indexer, load_model
from .pack import (
    PackValidationError,
    is_valid_pack_slug,
    normalize_robot_id,
    validate_pack_security,
)
from .perception import (
    DomainRandomizationConfig,
    SimDomainRandomizer,
    coerce_dr_config,
    normalize_detector_mode,
)
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
    """Motor state and control law for actuators (B601-DM 7-DOF or SO-101 6-DOF)."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, tool_pack=None):
        self.model, self.data = model, data
        self.idx = Indexer(model, tool_pack=tool_pack)
        self.dof = self.idx.dof
        self.robot = self.idx.robot
        self.enabled = False
        self.mode = ["pos_vel"] * self.dof
        self.q_t = np.zeros(self.dof)
        self.vlim = np.full(self.dof, 2.0)
        self.dq_t = np.zeros(self.dof)
        self.kp = np.zeros(self.dof)
        self.kd = np.zeros(self.dof)
        self.tau_ff = np.zeros(self.dof)
        self.extra = np.zeros(self.dof)
        # generalized-force mapping of each actuator (gear) on its joint dof
        self.gear = model.actuator_gear[: self.dof, 0].copy()
        self.dofs = np.array(
            [model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(self.dof)]
        )
        self.actuator_ids = np.arange(self.dof, dtype=int)
        if self.robot == "so-arm101":
            self.torque_limit = np.full(self.dof, 2.94)
            self.no_load_speed = np.full(self.dof, 50.0 * 2 * np.pi / 60)
            self.kpos = np.full(self.dof, 25.0)
            self.kv = np.full(self.dof, 2.0)
            jnts = [model.actuator_trnid[i, 0] for i in range(min(self.dof, 5))]
            self.stop_lo = np.array(
                [
                    model.jnt_range[j, 0] if model.jnt_limited[j] else -np.inf
                    for j in jnts
                ]
            )
            self.stop_hi = np.array(
                [
                    model.jnt_range[j, 1] if model.jnt_limited[j] else np.inf
                    for j in jnts
                ]
            )
        else:
            self.torque_limit = TORQUE_LIMIT[: self.dof].copy()
            self.no_load_speed = NO_LOAD_SPEED[: self.dof].copy()
            self.kpos = KPOS[: self.dof].copy()
            self.kv = KV[: self.dof].copy()
            # J1..J6 range ends in motor angles (joint angle - model offset); +-inf for unlimited joints
            jnts = [model.actuator_trnid[i, 0] for i in range(6)]
            self.stop_lo = (
                np.array(
                    [
                        model.jnt_range[j, 0] if model.jnt_limited[j] else -np.inf
                        for j in jnts
                    ]
                )
                - MODEL_OFFSETS_RAD
            )
            self.stop_hi = (
                np.array(
                    [
                        model.jnt_range[j, 1] if model.jnt_limited[j] else np.inf
                        for j in jnts
                    ]
                )
                - MODEL_OFFSETS_RAD
            )

    def set_enabled(self, enabled: bool) -> None:
        if enabled and not self.enabled:
            # On enable the motors hold where they are until a target arrives.
            self.q_t = self.idx.motor_positions(self.data)
            self.mode = ["pos_vel"] * self.dof
        self.enabled = enabled

    @classmethod
    def parse_targets(cls, targets, dof: int | None = None) -> list[tuple] | None:
        """Validated targets (one tuple per joint), or None when any field is malformed."""
        target_dof = DOF if dof is None else dof
        if not isinstance(targets, list) or len(targets) != target_dof:
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
                vals = [_finite(t.get("q"))] + [
                    _finite(t.get(k, 0.0)) for k in ("dq", "kp", "kd", "tau")
                ]
                if any(v is None for v in vals):
                    return None
                out.append((mode, *vals))
            else:
                return None
        return out

    def set_targets(self, targets) -> bool:
        """Apply all targets, or none of them when one is malformed."""
        parsed = self.parse_targets(targets, self.dof)
        if parsed is None:
            return False
        for i, t in enumerate(parsed):
            if t[0] == "pos_vel":
                self.mode[i], self.q_t[i], self.vlim[i] = t
            else:
                (
                    self.mode[i],
                    self.q_t[i],
                    self.dq_t[i],
                    self.kp[i],
                    self.kd[i],
                    self.tau_ff[i],
                ) = t
        return True

    def apply(self) -> None:
        """Set actuator gains / controls and applied forces for the next step."""
        m, d = self.model, self.data
        q = self.idx.motor_positions(d)
        d.qfrc_applied[self.dofs] = 0.0
        if not self.enabled:
            m.actuator_gainprm[self.actuator_ids, 0] = 0.0
            m.actuator_biasprm[self.actuator_ids, 2] = 0.0
            d.ctrl[self.actuator_ids] = 0.0
            m.actuator_forcerange[self.actuator_ids, 0] = -self.torque_limit
            m.actuator_forcerange[self.actuator_ids, 1] = self.torque_limit
            self.extra = np.zeros(self.dof)
            return
        extra = np.zeros(
            self.dof
        )  # torque applied outside the velocity servo (N·m, motor side)
        kv, ctrl = np.zeros(self.dof), np.zeros(self.dof)
        bias = d.qfrc_bias[self.dofs[: min(6, self.dof)]]
        arm_dof = 5 if self.robot == "so-arm101" else 6
        for i in range(self.dof):
            if self.mode[i] == "pos_vel":
                vmax = min(self.vlim[i], self.no_load_speed[i])
                ctrl[i] = np.clip(self.kpos[i] * (self.q_t[i] - q[i]), -vmax, vmax)
                kv[i] = self.kv[i]
                if i < arm_dof and not self.on_stop(i, q[i], bias[i]):
                    extra[i] = bias[i]
            else:
                ctrl[i] = self.dq_t[i]
                kv[i] = max(self.kd[i], 0.0)
                extra[i] = self.kp[i] * (self.q_t[i] - q[i]) + self.tau_ff[i]
        extra = np.clip(extra, -self.torque_limit, self.torque_limit)
        m.actuator_gainprm[self.actuator_ids, 0] = kv
        m.actuator_biasprm[self.actuator_ids, 2] = -kv
        # the servo gets whatever torque the limit leaves after the extra term
        m.actuator_forcerange[self.actuator_ids, 0] = -self.torque_limit - extra
        m.actuator_forcerange[self.actuator_ids, 1] = self.torque_limit - extra
        d.ctrl[self.actuator_ids] = ctrl
        d.qfrc_applied[self.dofs] = self.gear * extra
        self.extra = extra

    def on_stop(self, i: int, q: float, bias: float) -> bool:
        """Joint i (J1..J6 or J1..J5) rests on a range end: position and target at it and the load
        (bias torque: what a motor needs to hold) pushing into it."""
        if i >= len(self.stop_hi):
            return False
        hi, lo = self.stop_hi[i] - STOP_EPS, self.stop_lo[i] + STOP_EPS
        return (q >= hi and self.q_t[i] >= hi and bias < 0) or (
            q <= lo and self.q_t[i] <= lo and bias > 0
        )

    def reported_torque(self) -> np.ndarray:
        if not self.enabled:
            return np.zeros(self.dof)
        return self.data.actuator_force[: self.dof] + self.extra


def set_joint_friction(model: mujoco.MjModel, damping=None, frictionloss=None) -> None:
    """Override J1..J6 joint damping (N·m·s/rad) and friction loss (N·m), 6 values each."""
    dofs = Indexer(model).qvel[:6]
    if damping is not None:
        model.dof_damping[dofs] = damping
    if frictionloss is not None:
        model.dof_frictionloss[dofs] = frictionloss


class Simulator:
    def __init__(
        self,
        port: int,
        host: str = "127.0.0.1",
        viewer: bool = False,
        damping=None,
        frictionloss=None,
        torque_report: str = "instant",
        objects: str = "cube",
        seed: int | None = None,
        camera_mount: CameraMount | None = None,
        robot: str = "b601dm",
        scene: str | None = None,
        tool: str | None = None,
        dr_config: DomainRandomizationConfig | dict | None = None,
        detector_mode: str = "gt",
        detector_model: str | None = None,
        detector_labels: list[str] | tuple[str, ...] | None = None,
        detector_callable=None,
        detector_conf: float = 0.25,
        detector_iou: float = 0.45,
    ):
        self.robot = (robot or os.environ.get("REBOT_SIM_ROBOT") or "b601dm").lower()
        self.robot_id = normalize_robot_id(self.robot) or "b601dm"
        self.damping = damping
        self.frictionloss = frictionloss
        self.mount = camera_mount
        self.has_meshes = bool(os.environ.get("REBOT_DEVARM_DIR"))
        self.scene_ref = scene
        self.tool_ref = tool
        self.objects_spec = objects
        self.detector_mode = normalize_detector_mode(detector_mode)
        self.detector_model = detector_model
        self.detector_labels = list(detector_labels or [])
        self.detector_callable = detector_callable
        self.detector_conf = float(detector_conf)
        self.detector_iou = float(detector_iou)
        self.dr_config = coerce_dr_config(dr_config)
        self.dr: SimDomainRandomizer | None = None
        self.dr_summary: dict | None = None
        # physics and the scene resets run under this lock; sim_state reads under it
        self.lock = threading.Lock()
        self.stopping = False
        self._resets: list[tuple] = []
        self.shared = None  # SharedState for the camera process
        self._old_shared = []
        self.cam_conn = None
        self.cam_proc = None
        self.camera_error: str | None = (
            None  # set when the camera process died while running
        )
        self._pending_scene: dict | None = None
        self.active_tx_id: str | None = None
        self.camera_tx_id: str | None = None
        self.activation_state: str = "COMMITTED"
        self._staged_tx: dict | None = None
        self._tx_lock = threading.Lock()
        self._committed_tx_payloads: dict[str, tuple] = {}
        self._camera_prepare_cb = None
        self._camera_commit_cb = None
        self._camera_abort_cb = None
        self.torque_report = torque_report
        # --torque-report mean: torques of the last physics steps and the command period in steps
        self.tau_hist: collections.deque = collections.deque(maxlen=256)
        self.steps_since_cmd, self.cmd_period = 0, 10
        (
            self.model,
            self.data,
            self.motors,
            self.scene,
            self.scene_pack,
            self.tool_pack,
            self.objects,
        ) = self._build_world(scene=scene, tool=tool, objects=objects, seed=seed)
        self.dr = SimDomainRandomizer(self.model, self.dr_config)
        self.dr_generation = 0
        self.dr_seed: int | None = None
        if self.dr_config.enabled:
            self._apply_dr_locked(seed)
            mujoco.mj_forward(self.model, self.data)
        self.active_scene_digest: str | None = getattr(self.scene_pack, "digest", None)
        self.active_tool_digest: str | None = getattr(self.tool_pack, "digest", None)
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
            from mujoco import (
                viewer as mj_viewer,
            )  # needs a display; imported only here

            self.viewer = mj_viewer.launch_passive(self.model, self.data)

    def _build_world(
        self,
        scene: str | None,
        tool: str | None,
        objects: str,
        seed: int | None,
        *,
        scene_is_dir: bool = False,
        tool_is_dir: bool = False,
    ):
        from .pack import PackScene, load_scene_pack, load_tool_pack

        scene_pack = load_scene_pack(scene, prefer_disk=scene_is_dir)
        effective_tool = (
            tool
            if tool is not None
            else (scene_pack.default_tool if scene_pack else "default")
        )
        tool_pack = load_tool_pack(effective_tool, prefer_disk=tool_is_dir and tool is not None)

        if scene_pack is not None:
            model = load_model(
                camera=self.mount,
                robot=self.robot,
                scene_pack=scene_pack,
                tool_pack=tool_pack,
            )
            set_joint_friction(model, self.damping, self.frictionloss)
            data = mujoco.MjData(model)
            motors = Motors(model, data, tool_pack=tool_pack)
            pack_scene = PackScene(model, scene_pack, tool_pack)
            pack_scene.reset(data, seed)
            return (
                model,
                data,
                motors,
                pack_scene,
                scene_pack,
                tool_pack,
                pack_scene.objects,
            )

        parsed_objs = parse_objects(objects)
        model = load_model(
            objects=parsed_objs,
            camera=self.mount,
            robot=self.robot,
            tool_pack=tool_pack,
        )
        set_joint_friction(model, self.damping, self.frictionloss)
        data = mujoco.MjData(model)
        motors = Motors(model, data, tool_pack=tool_pack)
        if not tool_pack.is_default:
            pack_scene = PackScene(model, None, tool_pack, fallback_objects=parsed_objs)
        else:
            pack_scene = Scene(model, parsed_objs)
        pack_scene.reset(data, seed)
        return model, data, motors, pack_scene, None, tool_pack, parsed_objs

    def handle(self, payload: bytes, addr) -> None:
        """Apply one datagram. Malformed packets change nothing and are counted in `rejected`."""
        try:
            msg = json.loads(payload)
        except (ValueError, TypeError, UnicodeDecodeError, RecursionError):
            msg = None
        if (
            not isinstance(msg, dict)
            or msg.get("v") != PROTOCOL_VERSION
            or msg.get("type") not in ("cmd", "poll")
        ):
            self.rejected += 1
            return
        if msg["type"] == "poll":
            self.peer = addr
            return
        enable, seq, targets = (
            msg.get("enable", False),
            msg.get("seq", 0),
            msg.get("targets"),
        )
        parsed = (
            None
            if targets is None
            else self.motors.parse_targets(targets, self.motors.dof)
        )
        if (
            not isinstance(enable, bool)
            or isinstance(seq, bool)
            or not isinstance(seq, int)
            or seq < 0
            or (targets is not None and parsed is None)
        ):
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
        tau = self.torque_feedback()
        state = {
            "v": PROTOCOL_VERSION,
            "type": "state",
            "session": self.session,
            "seq": self.seq,
            "ack": self.ack,
            "enabled": self.motors.enabled,
            "q": idx.motor_positions(d).round(6).tolist(),
            "dq": idx.motor_velocities(d).round(6).tolist(),
            "tau": tau.round(5).tolist(),
            "temp": [TEMPERATURE_C] * self.motors.dof,
            "t": round(d.time, 4),
        }
        if self.motors.robot == "so-arm101":
            state["loads"] = np.clip(np.abs(tau) / 2.94, 0.0, 1.0).round(4).tolist()
        return json.dumps(state, separators=(",", ":")).encode()

    def torque_feedback(self) -> np.ndarray:
        """Instantaneous motor torque, or with --torque-report mean its mean over the last command
        period (sliding window of the physics steps between the two latest command datagrams)."""
        if self.torque_report == "mean" and self.tau_hist:
            return np.mean(list(self.tau_hist)[-self.cmd_period :], axis=0)
        return self.motors.reported_torque()

    def step(self) -> None:
        with self.lock:
            try:
                for item in self._resets:
                    if len(item) == 4:
                        seed, dr_cfg, done, out = item
                    else:
                        seed, done, out = item
                        dr_cfg = None
                    try:
                        out.update(self._reset_locked(seed, dr=dr_cfg))
                    except Exception as exc:
                        out["error"] = f"reset failed: {exc}"
                    finally:
                        done.set()
            finally:
                self._resets.clear()
            self.motors.apply()
            mujoco.mj_step(self.model, self.data)
            if hasattr(self.scene, "step_io"):
                self.scene.step_io(self.data)
            shared, data = self.shared, self.data
        if (
            shared is not None
            and self.camera_error is None
            and shared.nq == len(data.qpos)
        ):
            try:
                shared.write(data)
            except TimeoutError:  # the camera process died holding the lock
                self.camera_error = "shared-state lock timeout"
        if self.torque_report == "mean":
            self.tau_hist.append(self.motors.reported_torque())
            self.steps_since_cmd += 1

    # ---- scene control (HTTP threads)
    @staticmethod
    def _validate_seed_arg(seed) -> int | None:
        if seed is None:
            return None
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError("seed must be a non-negative integer or null")
        return seed

    def request_reset(self, seed: int | None, timeout: float = 2.0, dr=None) -> dict:
        """Place the objects for `seed` at the next physics step; returns the state after it."""
        self._validate_seed_arg(seed)
        dr_cfg = coerce_dr_config(dr) if dr is not None else None
        done, out = threading.Event(), {}
        item = (seed, dr_cfg, done, out) if dr_cfg is not None else (seed, done, out)
        with self.lock:
            self._resets.append(item)
        if not done.wait(timeout):  # physics loop not running (tests): apply here
            with self.lock:
                if item in self._resets:
                    self._resets.remove(item)
                    out.update(self._reset_locked(seed, dr=dr_cfg))
        return out

    def _apply_dr_locked(self, seed: int | None) -> None:
        if self.dr is None:
            self.dr = SimDomainRandomizer(self.model, self.dr_config)
        self.dr.config = self.dr_config
        self.dr_generation += 1
        if self.dr_config.enabled:
            eff_seed = seed if seed is not None else int(np.random.SeedSequence().entropy & 0x7FFFFFFF)
            self.dr_seed = eff_seed
            summary = self.dr.apply(self.model, seed=eff_seed)
            summary["generation"] = self.dr_generation
            self.dr_summary = summary
        else:
            self.dr_seed = None
            self.dr.restore_baseline(self.model)
            self.dr_summary = None
        mujoco.mj_forward(self.model, self.data)

    def _refresh_dr_baseline_locked(self, *, seed: int | None = None) -> None:
        """Re-snapshot DR baseline from the current `self.model` after a scene/model commit."""
        self.dr = SimDomainRandomizer(self.model, self.dr_config)
        if self.dr_config.enabled:
            self._apply_dr_locked(seed if seed is not None else self.scene.seed)
        else:
            self.dr_seed = None
            self.dr_summary = None

    def _reset_locked(self, seed, dr=None) -> dict:
        """Reset the scene; a seed the scene rejects gives {"error": ...} and changes nothing."""
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
            return {"error": f"reset with seed {seed!r} failed: seed must be a non-negative integer or null"}
        if dr is not None:
            try:
                new_dr = coerce_dr_config(dr)
            except (ValueError, TypeError) as exc:
                return {"error": f"invalid dr config: {exc}"}
        else:
            new_dr = None
        try:
            self.scene.reset(self.data, seed)
        except (ValueError, TypeError, OverflowError) as exc:
            return {"error": f"reset with seed {seed!r} failed: {exc}"}
        if new_dr is not None:
            self.dr_config = new_dr
        if self.dr_config.enabled or (self.dr is not None and self.dr._dirty):
            self._apply_dr_locked(seed)
        return self._state_locked()

    def get_dr(self) -> dict:
        with self.lock:
            return {
                "enabled": self.dr_config.enabled,
                "config": self.dr_config.to_dict(),
                "summary": self.dr_summary,
                "generation": self.dr_generation,
            }

    def set_dr(self, dr_config, seed: int | None = None, *, use_seed: bool = False) -> dict:
        if use_seed:
            self._validate_seed_arg(seed)
        new_dr = coerce_dr_config(dr_config)
        with self.lock:
            self.dr_config = new_dr
            eff_seed = seed if use_seed else self.scene.seed
            self._apply_dr_locked(eff_seed)
            return self._state_locked()

    def _state_locked(self) -> dict:
        d = self.data
        tcp_pos, tcp_quat = self.motors.idx.tcp_pose(d)
        out = {
            "t": round(d.time, 4),
            "seed": self.scene.seed,
            "enabled": self.motors.enabled,
            "q": self.motors.idx.motor_positions(d).round(5).tolist(),
            "tcp": [round(float(v), 5) for v in tcp_pos],
            "tcp_quat": [round(float(v), 5) for v in tcp_quat],  # w, x, y, z
            "objects": self.scene.state(d),
            "scene": self.scene_pack.id if self.scene_pack else "default",
            "tool": self.tool_pack.id,
            "robot": self.robot_id,
            "detector_mode": self.detector_mode,
            "tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
            "scene_digest": self.active_scene_digest,
            "tool_digest": self.active_tool_digest,
            "activation_state": self.activation_state,
            "degraded": self.activation_state == "DEGRADED_SPLIT",
        }
        if self.dr_config.enabled and self.dr_summary is not None:
            out["dr"] = self.dr_summary
        if hasattr(self.scene, "io_state"):
            out["io"] = self.scene.io_state(d)
        if hasattr(self.scene, "task_state"):
            task = self.scene.task_state(d)
            if task is not None:
                out["task"] = task
        return out

    def sim_state(self) -> dict:
        with self.lock:
            return self._state_locked()

    def _scene_info_locked(self) -> dict:
        info = self._state_locked()
        info["scene_pack"] = (
            self.scene_pack.summary()
            if self.scene_pack
            else {"id": "default", "objects": [o.name for o in self.objects]}
        )
        info["tool_pack"] = self.tool_pack.summary()
        return info

    def get_scene(self) -> dict:
        with self.lock:
            return self._scene_info_locked()

    def get_io(self) -> dict:
        with self.lock:
            io = (
                self.scene.io_state(self.data)
                if hasattr(self.scene, "io_state")
                else {}
            )
            return {"io": io}

    def set_io(self, updates: dict) -> dict:
        if not isinstance(updates, dict):
            return {"error": "IO body must be a JSON object"}
        with self.lock:
            if not hasattr(self.scene, "set_io"):
                return {
                    "error": "active scene/tool does not expose virtual I/O",
                    "io": {},
                }
            io = self.scene.set_io(updates, self.data)
            if self.shared is not None and self.shared.nq == len(self.data.qpos):
                try:
                    self.shared.write(self.data)
                except TimeoutError:
                    pass
            return {"io": io}

    def set_camera_hooks(self, prepare_cb=None, commit_cb=None, abort_cb=None) -> None:
        """Register optional in-process camera prepare/commit/abort callbacks."""
        self._camera_prepare_cb = prepare_cb
        self._camera_commit_cb = commit_cb
        self._camera_abort_cb = abort_cb

    def _cleanup_staged_tx_locked(self) -> None:
        if self._staged_tx is not None:
            new_shared = self._staged_tx.get("new_shared")
            if new_shared is not None and new_shared is not self.shared:
                try:
                    new_shared.close()
                except Exception:
                    pass
            staged_cam = self._staged_tx.get("staged_cam")
            if staged_cam is not None and self._camera_abort_cb is not None:
                try:
                    self._camera_abort_cb(staged_cam)
                except Exception:
                    pass
            self._staged_tx = None

    def _snapshot_active_locked(self) -> dict:
        return {
            "model": self.model,
            "data": self.data,
            "motors": self.motors,
            "scene": self.scene,
            "scene_pack": self.scene_pack,
            "tool_pack": self.tool_pack,
            "objects": self.objects,
            "scene_ref": self.scene_ref,
            "tool_ref": self.tool_ref,
            "objects_spec": self.objects_spec,
            "shared": self.shared,
            "tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
            "scene_digest": self.active_scene_digest,
            "tool_digest": self.active_tool_digest,
            "dr": self.dr,
            "dr_summary": self.dr_summary,
            "dr_seed": self.dr_seed,
            "dr_generation": self.dr_generation,
        }

    def _restore_snapshot_locked(self, snap: dict) -> None:
        self.tau_hist.clear()
        self.steps_since_cmd = 0
        self.model = snap["model"]
        self.data = snap["data"]
        self.motors = snap["motors"]
        self.scene = snap["scene"]
        self.scene_pack = snap["scene_pack"]
        self.tool_pack = snap["tool_pack"]
        self.objects = snap["objects"]
        self.scene_ref = snap["scene_ref"]
        self.tool_ref = snap["tool_ref"]
        self.objects_spec = snap["objects_spec"]
        self.shared = snap["shared"]
        self.active_tx_id = snap["tx_id"]
        self.camera_tx_id = snap["camera_tx_id"]
        self.active_scene_digest = snap["scene_digest"]
        self.active_tool_digest = snap["tool_digest"]
        self.dr = snap["dr"]
        self.dr_summary = snap["dr_summary"]
        self.dr_seed = snap["dr_seed"]
        self.dr_generation = snap["dr_generation"]

    def _close_old_shared_locked(self) -> None:
        for old_s in self._old_shared:
            if old_s is not self.shared:
                try:
                    old_s.close()
                except Exception:
                    pass
        self._old_shared.clear()

    @staticmethod
    def _tx_fingerprint(params: dict, scene_ref, tool_ref, objects_str, seed) -> tuple:
        return (
            str(scene_ref) if scene_ref is not None else None,
            str(tool_ref) if tool_ref is not None else None,
            params.get("scene_dir"),
            params.get("tool_dir"),
            params.get("scene_digest"),
            params.get("tool_digest"),
            normalize_robot_id(params.get("robot")) if isinstance(params.get("robot"), str) else params.get("robot"),
            str(objects_str),
            seed,
        )

    def _record_committed_tx_locked(self, tx_id: str, fingerprint: tuple) -> None:
        self._committed_tx_payloads[tx_id] = fingerprint
        if len(self._committed_tx_payloads) > 256:
            oldest = next(iter(self._committed_tx_payloads))
            self._committed_tx_payloads.pop(oldest, None)

    def _tx_prepare_locked(self, params: dict) -> dict:
        if self.activation_state == "DEGRADED_SPLIT":
            return {
                "_http_status": 503,
                "error": "SIM_ACTIVATION_DEGRADED",
                "code": "SIM_ACTIVATION_DEGRADED",
                "state": "DEGRADED_SPLIT",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
                "robot": self.robot_id,
            }
        if self.activation_state == "COMMITTING":
            return {
                "_http_status": 409,
                "error": "commit in progress",
                "code": "TX_BUSY",
                "state": "ABORTED",
            }
        if not isinstance(params, dict):
            return {
                "_http_status": 400,
                "error": "request body must be a JSON object",
                "code": "INVALID_REQUEST",
                "state": "ABORTED",
            }
        tx_id = params.get("tx_id")
        if not isinstance(tx_id, str) or not tx_id.strip():
            return {
                "_http_status": 400,
                "error": "tx_id must be a non-empty string",
                "code": "INVALID_TX_ID",
                "state": "ABORTED",
            }
        tx_id = tx_id.strip()

        req_robot = params.get("robot")
        if req_robot is not None:
            norm_req = normalize_robot_id(req_robot) if isinstance(req_robot, str) else None
            if norm_req != self.robot_id:
                return {
                    "_http_status": 400,
                    "error": f"Robot {req_robot!r} is incompatible with running robot {self.robot_id!r}",
                    "code": "ROBOT_INCOMPATIBLE",
                    "expected_robot": self.robot_id,
                    "got_robot": req_robot,
                    "state": "ABORTED",
                }

        scene_param = params.get("scene")
        tool_param = params.get("tool")
        scene_dir = params.get("scene_dir")
        tool_dir = params.get("tool_dir")

        if scene_param is not None and not is_valid_pack_slug(scene_param):
            return {
                "_http_status": 400,
                "error": f"invalid scene slug: {scene_param!r}",
                "code": "INVALID_PACK",
                "state": "ABORTED",
            }
        if tool_param is not None and not is_valid_pack_slug(tool_param):
            return {
                "_http_status": 400,
                "error": f"invalid tool slug: {tool_param!r}",
                "code": "INVALID_PACK",
                "state": "ABORTED",
            }

        if scene_dir is not None:
            if not isinstance(scene_dir, str) or not scene_dir.strip():
                return {
                    "_http_status": 400,
                    "error": "scene_dir must be a non-empty string",
                    "code": "INVALID_PACK",
                    "state": "ABORTED",
                }
            scene_ref = scene_dir.strip()
        elif "scene" in params:
            scene_ref = scene_param
        else:
            scene_ref = self.scene_ref

        if tool_dir is not None:
            if not isinstance(tool_dir, str) or not tool_dir.strip():
                return {
                    "_http_status": 400,
                    "error": "tool_dir must be a non-empty string",
                    "code": "INVALID_PACK",
                    "state": "ABORTED",
                }
            tool_ref = tool_dir.strip()
        elif "tool" in params:
            tool_ref = tool_param
        elif "scene" in params or scene_dir is not None:
            tool_ref = None
        else:
            tool_ref = self.tool_ref

        seed = params.get("seed", self.scene.seed)
        objects_str = params.get("objects", self.objects_spec)
        fingerprint = self._tx_fingerprint(params, scene_ref, tool_ref, objects_str, seed)
        if tx_id in self._committed_tx_payloads:
            if self._committed_tx_payloads[tx_id] != fingerprint or tx_id != self.active_tx_id:
                return {
                    "_http_status": 409,
                    "error": f"tx_id {tx_id!r} was already committed",
                    "code": "TX_ID_CONFLICT",
                    "state": "ABORTED",
                }

        exp_scene_digest = params.get("scene_digest")
        exp_tool_digest = params.get("tool_digest")

        try:
            if scene_dir is not None:
                validate_pack_security(
                    scene_ref,
                    "scene",
                    expected_digest=exp_scene_digest if isinstance(exp_scene_digest, str) else None,
                )
            if tool_dir is not None:
                validate_pack_security(
                    tool_ref,
                    "tool",
                    expected_digest=exp_tool_digest if isinstance(exp_tool_digest, str) else None,
                )
            model, data, motors, scene, scene_pack, tool_pack, parsed_objs = (
                self._build_world(
                    scene=scene_ref,
                    tool=tool_ref,
                    objects=objects_str,
                    seed=seed,
                    scene_is_dir=(scene_dir is not None),
                    tool_is_dir=(tool_dir is not None),
                )
            )
            if (
                scene_dir is not None
                and scene_param
                and scene_param not in {"default", "default_world"}
                and scene_pack is not None
                and scene_pack.id != scene_param
            ):
                raise PackValidationError(
                    f"Scene pack id {scene_pack.id!r} does not match requested {scene_param!r}",
                    code="INVALID_PACK",
                )
            if (
                tool_dir is not None
                and tool_param
                and tool_param != "default"
                and tool_pack.id != tool_param
            ):
                raise PackValidationError(
                    f"Tool pack id {tool_pack.id!r} does not match requested {tool_param!r}",
                    code="INVALID_PACK",
                )
            if scene_pack is not None and scene_pack.robots:
                allowed = {normalize_robot_id(r) for r in scene_pack.robots}
                if self.robot_id not in allowed:
                    return {
                        "_http_status": 400,
                        "error": f"Scene {scene_pack.id!r} does not support robot {self.robot_id!r}",
                        "code": "ROBOT_INCOMPATIBLE",
                        "expected_robot": self.robot_id,
                        "got_robot": scene_pack.robots,
                        "state": "ABORTED",
                    }
            if tool_pack.robots:
                allowed = {normalize_robot_id(r) for r in tool_pack.robots}
                if self.robot_id not in allowed:
                    return {
                        "_http_status": 400,
                        "error": f"Tool {tool_pack.id!r} does not support robot {self.robot_id!r}",
                        "code": "ROBOT_INCOMPATIBLE",
                        "expected_robot": self.robot_id,
                        "got_robot": tool_pack.robots,
                        "state": "ABORTED",
                    }
            actual_scene_digest = scene_pack.digest if scene_pack is not None else None
            actual_tool_digest = tool_pack.digest if tool_pack is not None else None
            if exp_scene_digest is not None:
                if not isinstance(exp_scene_digest, str):
                    raise PackValidationError("scene_digest must be a string or null", code="DIGEST_MISMATCH")
                if exp_scene_digest.strip():
                    if actual_scene_digest is None or actual_scene_digest != exp_scene_digest.strip().lower():
                        raise PackValidationError(
                            f"Scene digest mismatch: expected {exp_scene_digest!r}, got {actual_scene_digest!r}",
                            code="DIGEST_MISMATCH",
                        )
            if exp_tool_digest is not None:
                if not isinstance(exp_tool_digest, str):
                    raise PackValidationError("tool_digest must be a string or null", code="DIGEST_MISMATCH")
                if exp_tool_digest.strip():
                    if actual_tool_digest is None or actual_tool_digest != exp_tool_digest.strip().lower():
                        raise PackValidationError(
                            f"Tool digest mismatch: expected {exp_tool_digest!r}, got {actual_tool_digest!r}",
                            code="DIGEST_MISMATCH",
                        )
        except PackValidationError as exc:
            return {
                "_http_status": exc.status,
                "error": str(exc),
                "code": exc.code,
                "state": "ABORTED",
            }
        except FileNotFoundError as exc:
            return {
                "_http_status": 400,
                "error": str(exc),
                "code": "PACK_NOT_FOUND",
                "state": "ABORTED",
            }
        except (ValueError, TypeError) as exc:
            return {
                "_http_status": 400,
                "error": f"scene prepare failed: {exc}",
                "code": "INVALID_PACK",
                "state": "ABORTED",
            }
        except Exception as exc:
            return {
                "_http_status": 422,
                "error": f"scene compile failed: {exc}",
                "code": "COMPILE_FAILED",
                "state": "ABORTED",
            }

        new_shared = None
        shm_dict = None
        if self.shared is not None:
            try:
                from .perception import SharedState

                new_shared = SharedState(model.nq, model.nv, lock=self.shared.lock)
                new_shared.write(data)
                shm_dict = {"name": new_shared.shm.name, "nq": model.nq, "nv": model.nv}
            except Exception as exc:
                if new_shared is not None:
                    try:
                        new_shared.close()
                    except Exception:
                        pass
                return {
                    "_http_status": 500,
                    "error": f"shared memory allocation failed: {exc}",
                    "code": "SHM_ALLOC_FAILED",
                    "state": "ABORTED",
                }

        staged_cam = None
        if self._camera_prepare_cb is not None:
            try:
                staged_cam = self._camera_prepare_cb(
                    {
                        "tx_id": tx_id,
                        "model": model,
                        "data": data,
                        "scene": scene,
                        "scene_pack": scene_pack,
                        "tool_pack": tool_pack,
                        "scene_ref": scene_ref,
                        "tool_ref": tool_ref,
                        "objects_str": objects_str,
                        "scene_digest": actual_scene_digest,
                        "tool_digest": actual_tool_digest,
                    }
                )
            except Exception as exc:
                if new_shared is not None:
                    try:
                        new_shared.close()
                    except Exception:
                        pass
                return {
                    "_http_status": 422,
                    "error": f"camera prepare failed: {exc}",
                    "code": "CAMERA_PREPARE_FAILED",
                    "state": "ABORTED",
                }

        self._cleanup_staged_tx_locked()
        self._staged_tx = {
            "tx_id": tx_id,
            "fingerprint": fingerprint,
            "physics_committed": False,
            "model": model,
            "data": data,
            "motors": motors,
            "scene": scene,
            "scene_pack": scene_pack,
            "tool_pack": tool_pack,
            "parsed_objs": parsed_objs,
            "scene_ref": scene_ref,
            "tool_ref": tool_ref,
            "objects_str": objects_str,
            "scene_digest": actual_scene_digest,
            "tool_digest": actual_tool_digest,
            "new_shared": new_shared,
            "staged_cam": staged_cam,
            "prev_snapshot": None,
        }

        out = {
            "_http_status": 200,
            "ok": True,
            "state": "PREPARED",
            "tx_id": tx_id,
            "robot": self.robot_id,
            "scene": scene_pack.id if scene_pack else "default",
            "tool": tool_pack.id,
            "scene_digest": actual_scene_digest,
            "tool_digest": actual_tool_digest,
            "_scene_ref": scene_ref,
            "_tool_ref": tool_ref,
            "_scene_is_dir": scene_dir is not None,
            "_tool_is_dir": tool_dir is not None,
            "_objects_spec": objects_str,
            "_scene_digest": actual_scene_digest,
            "_tool_digest": actual_tool_digest,
        }
        if shm_dict is not None:
            out["_shm"] = shm_dict
        return out

    def _tx_commit_physics_locked(self, params: dict) -> dict:
        if not isinstance(params, dict):
            return {
                "_http_status": 400,
                "error": "request body must be a JSON object",
                "code": "INVALID_REQUEST",
            }
        tx_id = params.get("tx_id")
        if not isinstance(tx_id, str) or not tx_id.strip():
            return {
                "_http_status": 400,
                "error": "tx_id must be a non-empty string",
                "code": "INVALID_TX_ID",
            }
        tx_id = tx_id.strip()
        if self.activation_state == "COMMITTING":
            return {
                "_http_status": 409,
                "error": "commit already in progress",
                "code": "TX_BUSY",
            }
        if self.activation_state == "DEGRADED_SPLIT":
            return {
                "_http_status": 503,
                "error": "SIM_ACTIVATION_DEGRADED",
                "code": "SIM_ACTIVATION_DEGRADED",
                "state": "DEGRADED_SPLIT",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
                "robot": self.robot_id,
            }
        if self._staged_tx is None or self._staged_tx["tx_id"] != tx_id:
            return {
                "_http_status": 409,
                "error": f"tx {tx_id!r} is not prepared",
                "code": "TX_NOT_PREPARED",
            }
        staged = self._staged_tx
        if staged.get("physics_committed"):
            return {
                "_http_status": 409,
                "error": f"tx {tx_id!r} physics already committed",
                "code": "TX_BUSY",
            }
        snap = self._snapshot_active_locked()
        self.activation_state = "COMMITTING"
        try:
            self.tau_hist.clear()
            self.steps_since_cmd = 0
            self.model = staged["model"]
            self.data = staged["data"]
            self.motors = staged["motors"]
            self.scene = staged["scene"]
            self.scene_pack = staged["scene_pack"]
            self.tool_pack = staged["tool_pack"]
            self.objects = staged["parsed_objs"]
            self.scene_ref = staged["scene_ref"]
            self.tool_ref = staged["tool_ref"]
            self.objects_spec = staged["objects_str"]
            self.active_scene_digest = staged["scene_digest"]
            self.active_tool_digest = staged["tool_digest"]
            if staged["new_shared"] is not None:
                self.shared = staged["new_shared"]
            self._refresh_dr_baseline_locked(seed=self.scene.seed)
            staged["prev_snapshot"] = snap
            staged["physics_committed"] = True
        except Exception as exc:
            self._restore_snapshot_locked(snap)
            self.activation_state = "COMMITTED"
            self._cleanup_staged_tx_locked()
            return {
                "_http_status": 500,
                "error": f"physics commit failed: {exc}",
                "code": "PHYSICS_COMMIT_FAILED",
                "state": "ABORTED",
            }
        return {
            "_http_status": 200,
            "ok": True,
            "tx_id": tx_id,
            "robot": self.robot_id,
            "scene": self.scene_pack.id if self.scene_pack else "default",
            "tool": self.tool_pack.id,
            "scene_digest": self.active_scene_digest,
            "tool_digest": self.active_tool_digest,
        }

    def _tx_mark_degraded_locked(self, params: dict) -> dict:
        tx_id = str(params.get("tx_id") or "")
        err = str(params.get("error") or "camera commit failed")
        self.active_tx_id = tx_id or self.active_tx_id
        self.activation_state = "DEGRADED_SPLIT"
        return {
            "_http_status": 503,
            "error": err,
            "code": "SIM_ACTIVATION_DEGRADED",
            "state": "DEGRADED_SPLIT",
            "tx_id": self.active_tx_id,
            "physics_tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
            "robot": self.robot_id,
        }

    def _tx_finalize_locked(self, params: dict) -> dict:
        tx_id = params.get("tx_id")
        if (
            not isinstance(tx_id, str)
            or self._staged_tx is None
            or self._staged_tx.get("tx_id") != tx_id
        ):
            return {
                "_http_status": 409,
                "error": f"tx {tx_id!r} is not staged for finalization",
                "code": "TX_NOT_PREPARED",
            }
        staged = self._staged_tx
        if staged.get("prev_snapshot") is not None:
            old_shared = staged["prev_snapshot"].get("shared")
            if old_shared is not None and old_shared is not self.shared:
                self._old_shared.append(old_shared)
        self._close_old_shared_locked()
        self._record_committed_tx_locked(tx_id, staged["fingerprint"])
        self.active_tx_id = tx_id
        self.camera_tx_id = tx_id
        self.activation_state = "COMMITTED"
        self._staged_tx = None
        return {
            "_http_status": 200,
            "ok": True,
            "state": "COMMITTED",
            "tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
            "robot": self.robot_id,
            "scene": self.scene_pack.id if self.scene_pack else "default",
            "tool": self.tool_pack.id,
            "scene_digest": self.active_scene_digest,
            "tool_digest": self.active_tool_digest,
        }

    def _tx_rollback_physics_locked(self, params: dict) -> dict:
        tx_id = params.get("tx_id")
        if (
            self.activation_state not in {"DEGRADED_SPLIT", "COMMITTING"}
            or self._staged_tx is None
            or self._staged_tx.get("tx_id") != tx_id
            or self._staged_tx.get("prev_snapshot") is None
        ):
            return {
                "_http_status": 409,
                "error": f"tx {tx_id!r} is not in DEGRADED_SPLIT",
                "code": "TX_NOT_PREPARED",
            }
        staged = self._staged_tx
        if staged.get("staged_cam") is not None and self._camera_abort_cb is not None:
            try:
                self._camera_abort_cb(staged["staged_cam"])
            except Exception as exc:
                self.activation_state = "DEGRADED_SPLIT"
                return {
                    "_http_status": 503,
                    "error": f"camera abort failed: {exc}",
                    "code": "CAMERA_ABORT_FAILED",
                    "state": "DEGRADED_SPLIT",
                }
        snap = staged["prev_snapshot"]
        self._restore_snapshot_locked(snap)
        if staged.get("new_shared") is not None and staged["new_shared"] is not self.shared:
            try:
                staged["new_shared"].close()
            except Exception:
                pass
        self._staged_tx = None
        self.activation_state = "COMMITTED"
        return {
            "_http_status": 200,
            "ok": True,
            "state": "ABORTED",
            "restored_tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
            "robot": self.robot_id,
            "scene": self.scene_pack.id if self.scene_pack else "default",
            "tool": self.tool_pack.id,
            "scene_digest": self.active_scene_digest,
            "tool_digest": self.active_tool_digest,
        }

    def _tx_commit_locked(self, params: dict) -> dict:
        if not isinstance(params, dict):
            return {
                "_http_status": 400,
                "error": "request body must be a JSON object",
                "code": "INVALID_REQUEST",
            }
        tx_id = params.get("tx_id")
        if not isinstance(tx_id, str) or not tx_id.strip():
            return {
                "_http_status": 400,
                "error": "tx_id must be a non-empty string",
                "code": "INVALID_TX_ID",
            }
        tx_id = tx_id.strip()
        phase = params.get("phase", "all")
        if not isinstance(phase, str) or phase not in {"all", "camera_only", "rollback_physics"}:
            return {
                "_http_status": 400,
                "error": f"invalid commit phase: {phase!r}",
                "code": "INVALID_PHASE",
            }

        if phase == "all" and self.activation_state == "COMMITTING":
            return {
                "_http_status": 409,
                "error": "commit already in progress",
                "code": "TX_BUSY",
            }

        if (
            phase in {"all", "camera_only"}
            and tx_id == self.active_tx_id
            and self.camera_tx_id == tx_id
            and self.activation_state == "COMMITTED"
        ):
            return {
                "_http_status": 200,
                "ok": True,
                "state": "COMMITTED",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
                "robot": self.robot_id,
                "scene": self.scene_pack.id if self.scene_pack else "default",
                "tool": self.tool_pack.id,
                "scene_digest": self.active_scene_digest,
                "tool_digest": self.active_tool_digest,
            }

        if phase == "rollback_physics":
            return self._tx_rollback_physics_locked({"tx_id": tx_id})

        if phase == "camera_only":
            if (
                self.activation_state != "DEGRADED_SPLIT"
                or self._staged_tx is None
                or self._staged_tx.get("tx_id") != tx_id
            ):
                return {
                    "_http_status": 409,
                    "error": f"tx {tx_id!r} is not in DEGRADED_SPLIT",
                    "code": "TX_NOT_PREPARED",
                }
            if self._camera_commit_cb is not None:
                try:
                    self._camera_commit_cb(self._staged_tx.get("staged_cam"))
                except Exception as exc:
                    return self._tx_mark_degraded_locked({"tx_id": tx_id, "error": str(exc)})
            return self._tx_finalize_locked({"tx_id": tx_id})

        # phase == "all"
        phys = self._tx_commit_physics_locked({"tx_id": tx_id})
        if phys.get("_http_status", 200) != 200:
            return phys

        if self._camera_commit_cb is not None:
            try:
                self._camera_commit_cb(self._staged_tx.get("staged_cam"))
            except Exception as exc:
                return self._tx_mark_degraded_locked({"tx_id": tx_id, "error": str(exc)})

        return self._tx_finalize_locked({"tx_id": tx_id})

    def _tx_abort_locked(self, params: dict) -> dict:
        if not isinstance(params, dict):
            return {
                "_http_status": 400,
                "error": "request body must be a JSON object",
                "code": "INVALID_REQUEST",
            }
        tx_id = params.get("tx_id")
        if not isinstance(tx_id, str) or not tx_id.strip():
            return {
                "_http_status": 400,
                "error": "tx_id must be a non-empty string",
                "code": "MISSING_TX_ID",
            }
        tx_id = tx_id.strip()
        if self.activation_state == "COMMITTING":
            return {
                "_http_status": 409,
                "error": "commit in progress",
                "code": "TX_BUSY",
            }
        if self.activation_state == "DEGRADED_SPLIT":
            return {
                "_http_status": 503,
                "error": "SIM_ACTIVATION_DEGRADED",
                "code": "SIM_ACTIVATION_DEGRADED",
                "state": "DEGRADED_SPLIT",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
                "robot": self.robot_id,
            }
        if self._staged_tx is not None and self._staged_tx.get("tx_id") == tx_id:
            self._cleanup_staged_tx_locked()
            self._close_old_shared_locked()
        return {"_http_status": 200, "ok": True, "state": "ABORTED", "tx_id": tx_id}

    @staticmethod
    def _public_tx_reply(res: dict) -> tuple[int, dict]:
        status = int(res.pop("_http_status", 200))
        cleaned = {k: v for k, v in res.items() if not k.startswith("_")}
        return status, cleaned

    def prepare_scene(self, params: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY", "state": "ABORTED"}
        try:
            with self.lock:
                return self._public_tx_reply(self._tx_prepare_locked(params))
        finally:
            self._tx_lock.release()

    def commit_scene(self, params: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY"}
        try:
            with self.lock:
                return self._public_tx_reply(self._tx_commit_locked(params))
        finally:
            self._tx_lock.release()

    def abort_scene(self, params: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY"}
        try:
            with self.lock:
                return self._public_tx_reply(self._tx_abort_locked(params))
        finally:
            self._tx_lock.release()

    def _prepare_scene_locked(self, params: dict) -> dict:
        if self.activation_state == "DEGRADED_SPLIT":
            return {
                "_http_status": 503,
                "error": "SIM_ACTIVATION_DEGRADED",
                "code": "SIM_ACTIVATION_DEGRADED",
                "state": "DEGRADED_SPLIT",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
            }
        if not isinstance(params, dict):
            return {"error": "request body must be a JSON object"}
        req_robot = params.get("robot")
        if req_robot is not None:
            norm_req = normalize_robot_id(req_robot) if isinstance(req_robot, str) else None
            if norm_req != self.robot_id:
                return {
                    "_http_status": 400,
                    "error": f"Robot {req_robot!r} is incompatible with running robot {self.robot_id!r}",
                    "code": "ROBOT_INCOMPATIBLE",
                    "expected_robot": self.robot_id,
                    "got_robot": req_robot,
                    "state": "ABORTED",
                }
        if "scene" in params and params["scene"] is not None:
            if not is_valid_pack_slug(params["scene"]):
                return {"error": f"invalid scene slug: {params['scene']!r}", "code": "INVALID_PACK"}
        if "tool" in params and params["tool"] is not None:
            if not is_valid_pack_slug(params["tool"]):
                return {"error": f"invalid tool slug: {params['tool']!r}", "code": "INVALID_PACK"}
        scene_ref = params["scene"] if "scene" in params else self.scene_ref
        if "tool" in params:
            tool_ref = params["tool"]
        elif "scene" in params:
            tool_ref = None
        else:
            tool_ref = self.tool_ref
        seed = params.get("seed", self.scene.seed)
        objects_str = params.get("objects", self.objects_spec)
        try:
            model, data, motors, scene, scene_pack, tool_pack, parsed_objs = (
                self._build_world(
                    scene=scene_ref,
                    tool=tool_ref,
                    objects=objects_str,
                    seed=seed,
                )
            )
        except PackValidationError as exc:
            return {"_http_status": exc.status, "error": str(exc), "code": exc.code}
        except Exception as exc:
            return {"error": f"scene load failed: {exc}"}

        new_shared = None
        shm_dict = None
        if self.shared is not None:
            try:
                from .perception import SharedState

                new_shared = SharedState(model.nq, model.nv, lock=self.shared.lock)
                new_shared.write(data)
                shm_dict = {"name": new_shared.shm.name, "nq": model.nq, "nv": model.nv}
            except Exception as exc:
                if new_shared is not None:
                    try:
                        new_shared.close()
                    except Exception:
                        pass
                return {"_http_status": 500, "error": f"shared memory allocation failed: {exc}", "code": "SHM_ALLOC_FAILED"}

        self._pending_scene = {
            "model": model,
            "data": data,
            "motors": motors,
            "scene": scene,
            "scene_pack": scene_pack,
            "tool_pack": tool_pack,
            "parsed_objs": parsed_objs,
            "scene_ref": scene_ref,
            "tool_ref": tool_ref,
            "objects_str": objects_str,
            "new_shared": new_shared,
        }

        out = {
            "ok": True,
            "_scene_ref": scene_ref,
            "_tool_ref": tool_ref,
            "_objects_spec": objects_str,
        }
        if shm_dict is not None:
            out["_shm"] = shm_dict
        return out

    def _commit_scene_locked(self) -> dict:
        if self._pending_scene is None:
            return {"error": "no pending scene to commit"}
        p = self._pending_scene
        self._pending_scene = None

        self.tau_hist.clear()
        self.steps_since_cmd = 0
        self.model, self.data, self.motors, self.scene = (
            p["model"],
            p["data"],
            p["motors"],
            p["scene"],
        )
        self.scene_pack, self.tool_pack, self.objects = (
            p["scene_pack"],
            p["tool_pack"],
            p["parsed_objs"],
        )
        self.scene_ref, self.tool_ref, self.objects_spec = (
            p["scene_ref"],
            p["tool_ref"],
            p["objects_str"],
        )
        self.active_scene_digest = getattr(self.scene_pack, "digest", None)
        self.active_tool_digest = getattr(self.tool_pack, "digest", None)

        if p["new_shared"] is not None:
            old_shared = self.shared
            self.shared = p["new_shared"]
            if old_shared is not None:
                self._old_shared.append(old_shared)
        self._close_old_shared_locked()
        self._refresh_dr_baseline_locked(seed=self.scene.seed)

        self._cleanup_staged_tx_locked()
        legacy_tx = f"legacy-{uuid.uuid4().hex[:12]}"
        self.active_tx_id = legacy_tx
        self.camera_tx_id = legacy_tx
        self.activation_state = "COMMITTED"
        self._record_committed_tx_locked(
            legacy_tx,
            (
                str(self.scene_ref) if self.scene_ref is not None else None,
                str(self.tool_ref) if self.tool_ref is not None else None,
                None,
                None,
                None,
                None,
                self.robot_id,
                str(self.objects_spec),
                self.scene.seed,
            ),
        )

        return self._scene_info_locked()

    def _abort_scene_locked(self) -> dict:
        if self._pending_scene is not None:
            new_shared = self._pending_scene.get("new_shared")
            if new_shared is not None:
                new_shared.close()
            self._pending_scene = None
        return {"ok": True}

    def _load_scene_locked(self, params: dict, *, include_shm: bool = False) -> dict:
        prep = self._prepare_scene_locked(params)
        if "error" in prep:
            return prep
        shm_dict = prep.get("_shm")
        scene_ref = prep.get("_scene_ref")
        tool_ref = prep.get("_tool_ref")
        objects_str = prep.get("_objects_spec")
        committed = self._commit_scene_locked()
        if "error" in committed:
            return committed
        if include_shm:
            if shm_dict is not None:
                committed["_shm"] = shm_dict
            committed["_scene_ref"] = scene_ref
            committed["_tool_ref"] = tool_ref
            committed["_objects_spec"] = objects_str
        return committed

    def load_scene(self, params: dict) -> dict:
        with self.lock:
            return self._load_scene_locked(params, include_shm=False)

    def start_camera(
        self,
        fps: int,
        size: tuple[int, int],
        host: str,
        video_port: int,
        perception_port: int,
        scene_stream: bool = False,
        objects: str = "cube",
        timeout: float = 60.0,
        noise: dict | None = None,
        depth: dict | None = None,
        mask_angle: bool = True,
        aruco_board: dict | None = None,
        detector_mode: str | None = None,
        detector_model: str | None = None,
        detector_labels: list[str] | tuple[str, ...] | None = None,
        detector_callable=None,
        detector_conf: float | None = None,
        detector_iou: float | None = None,
        dr_config: DomainRandomizationConfig | dict | None = None,
    ) -> dict:
        """Spawn the camera process (perception.camera_main); returns its ready message.
        `aruco_board`: the virtual GridBoard ({x, y, yaw_deg, side_m, noise_px, seed}) or None."""
        import multiprocessing as mp
        import pickle

        from .perception import SharedState, camera_main

        eff_mode = normalize_detector_mode(detector_mode) if detector_mode is not None else self.detector_mode
        eff_model = detector_model if detector_model is not None else self.detector_model
        eff_labels = list(detector_labels) if detector_labels is not None else self.detector_labels
        eff_callable = detector_callable if detector_callable is not None else self.detector_callable
        eff_conf = float(detector_conf) if detector_conf is not None else self.detector_conf
        eff_iou = float(detector_iou) if detector_iou is not None else self.detector_iou

        # Validate detector inputs and picklability BEFORE allocating shared memory or pipes.
        if eff_mode == "onnx" and not eff_model:
            raise ValueError("detector_mode='onnx' requires detector_model")
        if eff_mode == "callable":
            if eff_callable is None:
                raise ValueError("detector_mode='callable' requires detector_callable")
            if not isinstance(eff_callable, str):
                if not callable(eff_callable):
                    raise TypeError("detector_callable must be callable or 'module:func'")
                pickle.dumps(eff_callable)
        if dr_config is not None:
            self.set_dr(dr_config, seed=self.scene.seed, use_seed=True)

        ctx = mp.get_context("spawn")
        child = ready_r = ready_w = None
        try:
            self.shared = SharedState(self.model.nq, self.model.nv, lock=ctx.Lock())
            self.shared.write(self.data)
            self.cam_conn, child = ctx.Pipe()
            ready_r, ready_w = ctx.Pipe(duplex=False)
            m = self.mount
            cfg = {
                "fps": fps,
                "size": list(size),
                "host": host,
                "video_port": video_port,
                "perception_port": perception_port,
                "scene_stream": scene_stream,
                "objects": objects,
                "mount": m.ipc(),  # flip, rot_deg, tilt_deg, offset_m, q_end_cam, fx_px
                "aruco_board": aruco_board,
                "has_meshes": self.has_meshes,
                "noise": noise or {},
                "depth": depth or {},
                "mask_angle": mask_angle,
                "robot": self.robot,
                "scene": self.scene_ref,
                "tool": self.tool_ref
                if self.tool_ref is not None
                else self.tool_pack.source_ref,
                "detector_mode": eff_mode,
                "detector_model": eff_model,
                "detector_labels": eff_labels,
                "detector_callable": eff_callable,
                "detector_conf": eff_conf,
                "detector_iou": eff_iou,
                "dr": self.dr_config.to_dict(),
                "seed": self.scene.seed,
                "dr_seed": self.dr_seed,
                "dr_generation": self.dr_generation,
            }
            self.cam_proc = ctx.Process(
                target=camera_main,
                name="rebot-sim-camera",
                daemon=True,
                args=(
                    cfg,
                    self.shared.shm.name,
                    self.shared.lock,
                    self.model.nq,
                    self.model.nv,
                    child,
                    ready_w,
                    os.getpid(),
                ),
            )
            self.cam_proc.start()
        except Exception:
            for conn in (child, ready_w, ready_r):
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
            self.stop_camera()
            raise
        child.close()
        ready_w.close()
        try:
            ready = self._wait_ready(ready_r, timeout)
        finally:
            ready_r.close()
        if "error" in ready:
            self.stop_camera()
        return ready

    def _wait_ready(self, ready_r, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        while not ready_r.poll(0.1):  # watch the child: it can die before it reports
            if not self.cam_proc.is_alive():
                return {
                    "error": f"camera process exited during startup (code {self.cam_proc.exitcode})"
                }
            if time.monotonic() > deadline:
                return {"error": "camera process did not start"}
        try:
            return ready_r.recv()
        except EOFError:
            return {"error": "camera process exited during startup"}

    def stop_camera(self) -> None:
        """Stop and join the camera process, close its pipe, free the shared state. Idempotent."""
        if self.cam_proc is not None:
            self.cam_proc.terminate()
            self.cam_proc.join(5)
            if self.cam_proc.is_alive():
                self.cam_proc.kill()
                self.cam_proc.join(5)
            self.cam_proc = None
        if self.cam_conn is not None:
            self.cam_conn.close()
            self.cam_conn = None
        if self._pending_scene is not None:
            self._abort_scene_locked()
        if self._staged_tx is not None:
            snap = self._staged_tx.get("prev_snapshot")
            if snap is not None and snap.get("shared") is not None and snap["shared"] is not self.shared:
                try:
                    snap["shared"].close()
                except Exception:
                    pass
            self._cleanup_staged_tx_locked()
        if self.shared is not None:
            shared, self.shared = self.shared, None
            shared.close()
        for old_s in self._old_shared:
            old_s.close()
        self._old_shared.clear()

    def close(self) -> None:
        """Stop the physics loop and the camera, release the UDP socket. Idempotent; call it
        after run() has returned."""
        self.stopping = True
        self.stop_camera()
        self.sock.close()
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def service_camera(self) -> None:
        """Answer the camera process's /sim requests (physics thread)."""
        try:
            while self.cam_conn is not None and self.cam_conn.poll():
                msg = self.cam_conn.recv()
                rid, op = msg[0], msg[1]
                if op == "reset":
                    dr_arg = msg[3] if len(msg) > 3 else None
                    self.cam_conn.send((rid, self._reset_locked(msg[2], dr=dr_arg)))
                elif op == "dr_set":
                    dr_arg = msg[2]
                    seed_arg = msg[3] if len(msg) > 3 else None
                    use_seed_arg = bool(msg[4]) if len(msg) > 4 else False
                    self.cam_conn.send((rid, self.set_dr(dr_arg, seed=seed_arg, use_seed=use_seed_arg)))
                elif op == "dr_get":
                    self.cam_conn.send((rid, self.get_dr()))
                elif op == "scene_get":
                    self.cam_conn.send((rid, self._scene_info_locked()))
                elif op == "scene_prepare":
                    self.cam_conn.send((rid, self._prepare_scene_locked(msg[2])))
                elif op == "scene_commit":
                    self.cam_conn.send((rid, self._commit_scene_locked()))
                elif op == "scene_abort":
                    self.cam_conn.send((rid, self._abort_scene_locked()))
                elif op == "scene_load":
                    self.cam_conn.send(
                        (rid, self._load_scene_locked(msg[2], include_shm=True))
                    )
                elif op == "tx_prepare":
                    self.cam_conn.send((rid, self._tx_prepare_locked(msg[2])))
                elif op == "tx_commit":
                    self.cam_conn.send((rid, self._tx_commit_locked(msg[2])))
                elif op == "tx_commit_physics":
                    self.cam_conn.send((rid, self._tx_commit_physics_locked(msg[2])))
                elif op == "tx_mark_degraded":
                    self.cam_conn.send((rid, self._tx_mark_degraded_locked(msg[2])))
                elif op == "tx_finalize":
                    self.cam_conn.send((rid, self._tx_finalize_locked(msg[2])))
                elif op == "tx_rollback_physics":
                    self.cam_conn.send((rid, self._tx_rollback_physics_locked(msg[2])))
                elif op == "tx_abort":
                    self.cam_conn.send((rid, self._tx_abort_locked(msg[2])))
                elif op == "io_get":
                    io = (
                        self.scene.io_state(self.data)
                        if hasattr(self.scene, "io_state")
                        else {}
                    )
                    self.cam_conn.send((rid, {"io": io}))
                elif op == "io_set":
                    updates = msg[2]
                    if not isinstance(updates, dict) or not hasattr(
                        self.scene, "set_io"
                    ):
                        self.cam_conn.send(
                            (rid, {"error": "cannot set virtual I/O", "io": {}})
                        )
                    else:
                        io = self.scene.set_io(updates, self.data)
                        if self.shared is not None and self.shared.nq == len(
                            self.data.qpos
                        ):
                            self.shared.write(self.data)
                        self.cam_conn.send((rid, {"io": io}))
                else:
                    self.cam_conn.send((rid, self._state_locked()))
        except (
            EOFError,
            OSError,
        ) as exc:  # the camera process is gone (BrokenPipeError: mid-request)
            self.cam_conn.close()
            self.cam_conn = None
            self.camera_error = f"camera process exited (pipe closed: {exc!r})"

    def run(self, duration: float | None = None, stats_every: float = 0.0) -> None:
        dt = self.model.opt.timestep
        start = time.perf_counter()
        next_t = start
        steps, last_stats, last_steps = 0, start, 0
        while (
            duration is None or time.perf_counter() - start < duration
        ) and not self.stopping:
            self.poll_socket()
            self.step()
            steps += 1
            if self.camera_error:
                print(f"rebot-sim: {self.camera_error}", file=sys.stderr, flush=True)
                return
            if steps % 10 == 0 and self.cam_conn is not None:
                self.service_camera()  # a closed pipe sets camera_error: the loop ends after the next step
            if (
                steps % 100 == 0
                and self.cam_proc is not None
                and not self.cam_proc.is_alive()
            ):
                self.camera_error = (
                    f"camera process exited (code {self.cam_proc.exitcode})"
                )
                print(f"rebot-sim: {self.camera_error}", file=sys.stderr, flush=True)
                return
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
                next_t = (
                    now  # fell behind by > 50 ms: drop the backlog instead of racing
                )
            if stats_every and now - last_stats >= stats_every:
                rate = (steps - last_steps) / (now - last_stats)
                if self.objects:
                    obj = self.objects[0].name
                    cube = self.data.body(obj).xpos
                    obj_str = f" {obj}=({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f})"
                else:
                    obj_str = ""
                cam = ""
                if self.shared is not None:
                    cam = f" camera={self.shared.arr[2]:.0f} frames {self.shared.arr[3]:.1f} ms/frame"
                print(
                    f"t={self.data.time:8.2f}s rate={rate:6.1f} Hz enabled={self.motors.enabled} "
                    f"q={np.round(self.motors.idx.motor_positions(self.data), 3).tolist()}"
                    f"{obj_str} rejected={self.rejected}{cam}",
                    flush=True,
                )
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
    parser = argparse.ArgumentParser(
        prog="rebot-sim", description=__doc__.splitlines()[0]
    )
    parser.add_argument(
        "--robot",
        choices=("b601dm", "so-arm101", "so_arm101"),
        default=os.environ.get("REBOT_SIM_ROBOT", "b601dm"),
        help="robot model: b601dm (default) or so-arm101",
    )
    parser.add_argument(
        "--port", type=int, default=47600, help="UDP port (default 47600)"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="bind address (default 127.0.0.1)"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="no viewer (the default; kept for scripts)",
    )
    parser.add_argument(
        "--viewer", action="store_true", help="open the MuJoCo viewer (needs a display)"
    )
    parser.add_argument(
        "--stats",
        type=float,
        default=0.0,
        metavar="S",
        help="print step rate and state every S seconds",
    )
    parser.add_argument(
        "--duration", type=float, default=None, help="exit after this many seconds"
    )
    parser.add_argument(
        "--benchmark",
        type=int,
        default=0,
        metavar="N",
        help="time N unpaced physics steps and exit",
    )
    parser.add_argument(
        "--damping",
        type=six_floats,
        default=None,
        metavar="D1,..,D6",
        help="J1..J6 joint damping in N·m·s/rad (default: scene values)",
    )
    parser.add_argument(
        "--frictionloss",
        type=six_floats,
        default=None,
        metavar="F1,..,F6",
        help="J1..J6 joint friction loss in N·m (default: scene values)",
    )
    parser.add_argument(
        "--torque-report",
        choices=("instant", "mean"),
        default="instant",
        help="torque in state datagrams: instantaneous (default) or mean over the last command period",
    )
    parser.add_argument(
        "--objects",
        default="cube",
        help="scene objects, 1..3 of cube, cup, block (default cube)",
    )
    parser.add_argument(
        "--scene",
        default=None,
        help="composable scene pack ID, directory, or world XML",
    )
    parser.add_argument(
        "--tool",
        default=None,
        help="custom tool/gripper pack ID, preset ('default', 'suction', 'screwdriver'), directory, or XML",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="object placement seed (default: fixed layout)",
    )
    parser.add_argument(
        "--camera",
        action="store_true",
        help="render the wrist camera: H.264 stream, perception API, /sim routes",
    )
    parser.add_argument(
        "--video-port",
        type=int,
        default=8001,
        help="camera WebSocket port (default 8001)",
    )
    parser.add_argument(
        "--perception-port",
        type=int,
        default=8090,
        help="perception / sim HTTP port (default 8090)",
    )
    parser.add_argument(
        "--http-host",
        default="127.0.0.1",
        help="bind address of the camera and HTTP servers",
    )
    parser.add_argument(
        "--fps", type=int, default=15, help="camera frames per second (default 15)"
    )
    parser.add_argument(
        "--camera-size",
        default="1280x720",
        choices=("1280x720", "640x360"),
        help="stream size",
    )
    parser.add_argument(
        "--scene-stream",
        action="store_true",
        help="also stream the overview camera at /scene",
    )
    parser.add_argument(
        "--profile",
        default=None,
        help="profile JSON whose camera_mount is used (default: the kit profile)",
    )
    parser.add_argument(
        "--camera-rot-deg",
        type=float,
        default=None,
        help="override camera_mount.rot_deg",
    )
    parser.add_argument(
        "--camera-tilt-deg",
        type=float,
        default=None,
        help="override camera_mount.tilt_deg (armd sign: negative turns the axis toward the jaws)",
    )
    parser.add_argument(
        "--camera-offset",
        default=None,
        metavar="X,Y,Z",
        help="lens centre in end_link (m); default camera_mount.offset_m, else the v8 holder value",
    )
    parser.add_argument(
        "--camera-q",
        default=None,
        metavar="W,X,Y,Z",
        help="camera_mount.q_end_cam (R_end_cam as a unit quaternion); replaces flip / rot / tilt "
        "(default: the profile's q_end_cam, if any)",
    )
    parser.add_argument(
        "--fx",
        type=float,
        default=None,
        help="true focal length (px at 640x360, square pixels, centred, no distortion) for the render, "
        "detections, markers, depth and /sim/camera (default 824.4, the OV5647 stream value)",
    )
    parser.add_argument(
        "--aruco-board",
        default=None,
        metavar="X,Y[,YAW]",
        help="virtual 4x3 ArUco GridBoard (4 columns x 3 rows, aruco4x4_50 ids 0..11, 25 mm markers, 5 mm gap) lying on the "
        "table, centre at base X,Y (m), turned YAW deg about base z; the aruco pipeline reports its markers",
    )
    parser.add_argument(
        "--aruco-marker-m",
        type=float,
        default=0.025,
        help="marker side of the virtual board (m); the gap is 0.2 x the side",
    )
    parser.add_argument(
        "--marker-noise-px",
        type=float,
        default=0.0,
        help="Gaussian noise (sigma, px) added to each virtual marker corner, seeded by --seed and the frame",
    )
    det = parser.add_argument_group(
        "detection noise (the perception API's detections; all off by default)"
    )
    det.add_argument(
        "--det-jitter-px",
        type=float,
        default=0.0,
        metavar="S",
        help="Gaussian sigma (pixels at 640x360) added to the box centre x and y",
    )
    det.add_argument(
        "--det-size-jitter",
        type=float,
        default=0.0,
        metavar="F",
        help="w and h are multiplied by (1 + N(0, F)) independently",
    )
    det.add_argument(
        "--det-drop-rate",
        type=float,
        default=0.0,
        metavar="P",
        help="drop each detection with probability P per frame",
    )
    det.add_argument(
        "--det-latency-ms",
        type=float,
        default=0.0,
        metavar="L",
        help="publish detections L ms after their frame was captured (age_ms includes it)",
    )
    det.add_argument(
        "--det-false-rate",
        type=float,
        default=0.0,
        metavar="P",
        help="per frame, add one false detection (random label and place, score 0.3..0.6) with probability P",
    )
    det.add_argument(
        "--det-angle-jitter-deg",
        type=float,
        default=0.0,
        metavar="S",
        help="Gaussian sigma on angle_deg",
    )
    det.add_argument(
        "--no-mask-angle",
        action="store_true",
        help="a plain detection model: detections omit angle_deg / angle_conf / grasp_width_px",
    )
    det.add_argument(
        "--det-noise-seed",
        type=int,
        default=None,
        metavar="N",
        help="noise RNG seed (default: --seed, else 0)",
    )
    dep = parser.add_argument_group(
        "depth pipeline (emulated monocular depth model; off by default)"
    )
    dep.add_argument(
        "--depth",
        choices=("off", "metric", "relative"),
        default="off",
        help="offer the perception API's depth pipeline with this model kind (default off)",
    )
    dep.add_argument(
        "--depth-scale",
        type=float,
        default=1.0,
        metavar="S",
        help="metric: disparity scale error (1 = matches armd's metric prior)",
    )
    dep.add_argument(
        "--depth-noise-rel",
        type=float,
        default=0.0,
        metavar="F",
        help="Gaussian relative noise per grid cell",
    )
    dep.add_argument(
        "--depth-a",
        type=float,
        default=2.7,
        metavar="A",
        help="relative: disp = A / Z + B",
    )
    dep.add_argument(
        "--depth-b",
        type=float,
        default=0.45,
        metavar="B",
        help="relative: disp = A / Z + B",
    )
    dep.add_argument(
        "--depth-conf",
        type=float,
        default=0.95,
        metavar="C",
        help="depth_conf reported per detection",
    )
    det_eval = parser.add_argument_group(
        "detector evaluation backend (optional real-detector mode; default gt)"
    )
    det_eval.add_argument(
        "--detector",
        choices=("gt", "onnx", "callable"),
        default="gt",
        help="detector backend: gt (synthetic ground truth, default), onnx, or callable",
    )
    det_eval.add_argument(
        "--detector-model",
        default=None,
        metavar="PATH",
        help="path to ONNX detector model when --detector onnx",
    )
    det_eval.add_argument(
        "--detector-labels",
        default=None,
        metavar="L1,L2,...",
        help="comma-separated class label names for --detector onnx/callable",
    )
    det_eval.add_argument(
        "--detector-callable",
        default=None,
        metavar="MOD:FUNC",
        help="module:func callable path when --detector callable",
    )
    det_eval.add_argument(
        "--detector-conf",
        type=float,
        default=0.25,
        help="confidence threshold for ONNX detector (default 0.25)",
    )
    det_eval.add_argument(
        "--detector-iou",
        type=float,
        default=0.45,
        help="NMS IoU threshold for ONNX detector (default 0.45)",
    )
    dr_grp = parser.add_argument_group(
        "domain randomization (seed-driven; disabled by default)"
    )
    dr_grp.add_argument(
        "--dr-lighting",
        action="store_true",
        help="enable deterministic per-seed MuJoCo lighting domain randomization",
    )
    dr_grp.add_argument(
        "--dr-camera-jitter-m",
        type=float,
        default=0.0,
        metavar="M",
        help="max camera position jitter in metres (default 0.0)",
    )
    dr_grp.add_argument(
        "--dr-camera-jitter-deg",
        type=float,
        default=0.0,
        metavar="DEG",
        help="max camera orientation jitter in degrees (default 0.0)",
    )
    dr_grp.add_argument(
        "--dr-latency-ms",
        type=float,
        default=0.0,
        metavar="MS",
        help="simulated perception/observation delay in ms (default 0.0)",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.fps <= 30:
        parser.error("--fps must be 1..30")
    try:
        parse_objects(args.objects)
    except ValueError as exc:
        parser.error(f"--objects: {exc}")
    if args.seed is not None and args.seed < 0:
        parser.error("--seed must be non-negative")
    default_profile = (
        SO101_PROFILE if args.robot in ("so-arm101", "so_arm101") else PROFILE
    )
    mount = CameraMount.from_profile(args.profile or default_profile)
    if (
        args.camera_rot_deg is not None
        or args.camera_tilt_deg is not None
        or args.camera_offset
        or args.camera_q
        or args.fx is not None
    ):
        off = mount.offset_m
        if args.camera_offset:
            off = tuple(float(v) for v in args.camera_offset.split(","))
            if len(off) != 3:
                parser.error("--camera-offset needs 3 values")
        q = mount.q_end_cam
        if args.camera_q:
            q = tuple(float(v) for v in args.camera_q.split(","))
        try:
            mount = CameraMount(
                mount.flip,
                mount.rot_deg if args.camera_rot_deg is None else args.camera_rot_deg,
                mount.tilt_deg if args.camera_tilt_deg is None else args.camera_tilt_deg,
                off,
                q,
                args.fx,
            )
        except ValueError as exc:
            parser.error(f"camera mount: {exc}")
    aruco_board = None
    if args.aruco_board:
        from .aruco import Board

        if not 0.005 <= args.aruco_marker_m <= 0.2:
            parser.error("--aruco-marker-m must be in [0.005, 0.2]")
        if args.marker_noise_px < 0:
            parser.error("--marker-noise-px must be >= 0")
        try:
            b = Board.parse(args.aruco_board, side_m=args.aruco_marker_m)
        except ValueError as exc:
            parser.error(str(exc))
        aruco_board = {"x": b.x, "y": b.y, "yaw_deg": b.yaw_deg, "z": 0.0, "side_m": b.side_m,
                       "noise_px": args.marker_noise_px, "seed": args.seed or 0}
    for flag, value in (
        ("jitter-px", args.det_jitter_px),
        ("size-jitter", args.det_size_jitter),
        ("latency-ms", args.det_latency_ms),
        ("angle-jitter-deg", args.det_angle_jitter_deg),
    ):
        if value < 0:
            parser.error(f"--det-{flag} must be >= 0")
    for flag, value in (
        ("drop-rate", args.det_drop_rate),
        ("false-rate", args.det_false_rate),
    ):
        if not 0.0 <= value <= 1.0:
            parser.error(f"--det-{flag} must be 0..1")
    noise = {
        "jitter_px": args.det_jitter_px,
        "size_jitter": args.det_size_jitter,
        "drop_rate": args.det_drop_rate,
        "latency_ms": args.det_latency_ms,
        "false_rate": args.det_false_rate,
        "angle_jitter_deg": args.det_angle_jitter_deg,
        "seed": args.det_noise_seed
        if args.det_noise_seed is not None
        else (args.seed or 0),
    }
    if args.depth_scale <= 0 or args.depth_noise_rel < 0 or args.depth_a <= 0 or not 0.0 <= args.depth_conf <= 1.0:
        parser.error("--depth-scale and --depth-a must be > 0, --depth-noise-rel >= 0, --depth-conf 0..1")
    if args.detector == "onnx" and not args.detector_model:
        parser.error("--detector onnx requires --detector-model")
    if args.dr_camera_jitter_m < 0 or not 0.0 <= args.dr_camera_jitter_deg <= 180.0 or args.dr_latency_ms < 0:
        parser.error("--dr-camera-jitter-m and --dr-latency-ms must be >= 0, --dr-camera-jitter-deg 0..180")
    detector_labels = (
        [s.strip() for s in args.detector_labels.split(",") if s.strip()]
        if args.detector_labels
        else []
    )
    dr_enabled = bool(
        args.dr_lighting
        or args.dr_camera_jitter_m > 0.0
        or args.dr_camera_jitter_deg > 0.0
        or args.dr_latency_ms > 0.0
    )
    dr_config = DomainRandomizationConfig(
        enabled=dr_enabled,
        lighting=args.dr_lighting,
        camera_jitter_m=args.dr_camera_jitter_m,
        camera_jitter_deg=args.dr_camera_jitter_deg,
        latency_ms=args.dr_latency_ms,
    )
    depth = {
        "kind": args.depth,
        "scale": args.depth_scale,
        "noise_rel": args.depth_noise_rel,
        "a": args.depth_a,
        "b": args.depth_b,
        "conf": args.depth_conf,
        "seed": noise["seed"],
    }
    if args.benchmark:
        print(
            f"{benchmark(args.benchmark):.0f} physics steps/s (timestep 2 ms, unpaced)"
        )
        return
    viewer = args.viewer and not args.headless
    if viewer and not (
        os.environ.get("DISPLAY")
        or os.environ.get("WAYLAND_DISPLAY")
        or sys.platform == "darwin"
    ):
        parser.error("--viewer needs a display (DISPLAY / WAYLAND_DISPLAY not set)")
    sim = Simulator(
        args.port,
        args.host,
        viewer=viewer,
        damping=args.damping,
        frictionloss=args.frictionloss,
        torque_report=args.torque_report,
        objects=args.objects,
        seed=args.seed,
        camera_mount=mount if args.camera else None,
        robot=args.robot,
        scene=args.scene,
        tool=args.tool,
        dr_config=dr_config,
        detector_mode=args.detector,
        detector_model=args.detector_model,
        detector_labels=detector_labels,
        detector_callable=args.detector_callable,
        detector_conf=args.detector_conf,
        detector_iou=args.detector_iou,
    )
    print(
        f"rebot-sim listening on udp://{args.host}:{args.port} (session {sim.session})",
        flush=True,
    )
    if args.camera:
        w, h = (int(v) for v in args.camera_size.split("x"))
        ready = sim.start_camera(
            args.fps,
            (w, h),
            args.http_host,
            args.video_port,
            args.perception_port,
            args.scene_stream,
            args.objects,
            noise=noise,
            depth=depth,
            mask_angle=not args.no_mask_angle,
            aruco_board=aruco_board,
        )
        if "error" in ready:
            print(f"rebot-sim: {ready['error']}", file=sys.stderr, flush=True)
            sim.close()
            sys.exit(1)
        print(
            f"camera: {w}x{h} @ {args.fps} fps ({ready['backend']}, own process), mount rot {mount.rot_deg:g} tilt {mount.tilt_deg:g} "
            f"q {mount.q_end_cam} fx {mount.focal():.1f} board {aruco_board} "
            f"offset {mount.offset.round(4).tolist()}; video ws://{args.http_host}:{args.video_port}/, "
            f"perception http://{args.http_host}:{args.perception_port}",
            flush=True,
        )
    try:
        sim.run(args.duration, args.stats)
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()
    if sim.camera_error:
        sys.exit(1)


if __name__ == "__main__":
    main()
