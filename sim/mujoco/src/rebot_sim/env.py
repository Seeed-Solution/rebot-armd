"""Non-realtime fast training environment wrapping composable scene packs, tools, and kinematics."""

from __future__ import annotations

import types
from collections.abc import Sequence
from typing import Any, Self

import mujoco
import numpy as np

from .camera import CameraMount
from .domain_rand import ActionDelayBuffer, DomainRandomizationConfig, DomainRandomizer
from .model import load_model
from .pack import PackScene, load_scene_pack, load_tool_pack
from .server import Motors


def _is_real_scalar(val: Any) -> bool:
    return isinstance(val, (int, float, np.integer, np.floating)) and not isinstance(
        val, (bool, np.bool_)
    )


def validate_real_vector(
    values: Any,
    expected_len: int,
    label: str = "action",
    exc_type: type[ValueError] = ValueError,
) -> np.ndarray:
    """Validate a 1D (or batch-1 ``(1, N)``) real-numeric vector of exact length ``expected_len``."""
    if values is None or isinstance(
        values, (bool, np.bool_, str, bytes, bytearray, dict, set, complex, np.complexfloating)
    ):
        raise exc_type(
            f"{label} must be a real-numeric 1D sequence/array of length {expected_len}, "
            f"got {type(values).__name__}"
        )

    if not isinstance(values, np.ndarray):
        if not isinstance(values, Sequence):
            raise exc_type(
                f"{label} must be a real-numeric 1D sequence/array of length {expected_len}, "
                f"got {type(values).__name__}"
            )
        if len(values) == 1 and isinstance(values[0], (Sequence, np.ndarray)) and not isinstance(
            values[0], (str, bytes, bytearray)
        ):
            inner = values[0]
            if not isinstance(inner, np.ndarray):
                for idx, item in enumerate(inner):
                    if not _is_real_scalar(item):
                        raise exc_type(
                            f"{label}[0][{idx}] must be a real number, got {type(item).__name__}"
                        )
        else:
            for idx, item in enumerate(values):
                if not _is_real_scalar(item):
                    raise exc_type(
                        f"{label}[{idx}] must be a real number, got {type(item).__name__}"
                    )

    try:
        raw = np.asarray(values)
    except (TypeError, ValueError) as exc:
        raise exc_type(f"{label} could not be converted to a numeric array: {exc}") from exc
    if np.iscomplexobj(raw) or raw.dtype.kind not in ("i", "u", "f"):
        raise exc_type(
            f"{label} must contain real numeric values (int/float), got dtype {raw.dtype!r}"
        )

    if raw.ndim == 2 and raw.shape[0] == 1:
        arr = raw[0]
    elif raw.ndim == 1:
        arr = raw
    else:
        raise exc_type(
            f"Expected 1D {label} of length {expected_len} or shape (1, {expected_len}), "
            f"got shape {raw.shape}"
        )

    if arr.shape[0] != expected_len:
        raise exc_type(
            f"{label} length mismatch: expected {expected_len}, got {arr.shape[0]}"
        )

    out = np.asarray(arr, dtype=np.float64)
    if not np.all(np.isfinite(out)):
        raise exc_type(f"{label} contains non-finite values (NaN or Inf)")
    return out


class SimEnv:
    """Non-realtime fast training environment for policy learning and fast evaluation.

    Executes physics substeps synchronously without wall-clock sleeps, drives actuators with the
    same POS_VEL control law + bias compensation as ``rebot_sim.server.Motors``, updates virtual
    end-effector I/O (such as suction welds), and evaluates task stage progression.
    """

    def __init__(
        self,
        scene: str | None = "pcb_suction",
        tool: str | None = None,
        robot: str = "b601dm",
        sim_dt: float = 0.002,
        control_dt: float = 0.02,
        control_mode: str = "position",
        render_camera: str | None = None,
        render_width: int = 160,
        render_height: int = 120,
        headless: bool = True,
        dr_config: DomainRandomizationConfig | None = None,
    ) -> None:
        if control_mode not in ("position", "velocity"):
            raise ValueError(
                f"Unsupported control_mode {control_mode!r}: expected 'position' or 'velocity'"
            )
        if dr_config is not None and not isinstance(dr_config, DomainRandomizationConfig):
            raise TypeError(
                f"dr_config must be DomainRandomizationConfig or None, got {type(dr_config).__name__}"
            )
        self.scene_ref = scene
        self.robot = robot.lower()
        self.sim_dt = float(sim_dt)
        self.control_dt = float(control_dt)
        self.control_mode = control_mode
        self.n_substeps = max(1, round(self.control_dt / self.sim_dt))
        self.render_camera = render_camera
        self.render_width = int(render_width)
        self.render_height = int(render_height)
        self.headless = headless
        self.dr_config = dr_config

        self.scene_pack = load_scene_pack(scene) if scene is not None else None
        if tool is None and self.scene_pack is not None:
            tool_ref = getattr(self.scene_pack, "default_tool", "default")
        else:
            tool_ref = tool or "default"
        self.tool_pack = load_tool_pack(tool_ref)

        cam_mount = CameraMount() if render_camera is not None else None
        self.model = load_model(
            camera=cam_mount,
            robot=self.robot,
            scene_pack=self.scene_pack,
            tool_pack=self.tool_pack,
        )
        self.model.opt.timestep = self.sim_dt
        self.data = mujoco.MjData(self.model)
        self.motors = Motors(self.model, self.data, tool_pack=self.tool_pack)
        # Ensure any tool <position> actuator (biasprm[:, 1] != 0) behaves cleanly under Motors POS_VEL loop.
        self.model.actuator_biasprm[:, 1] = 0.0
        self.indexer = self.motors.idx
        self.pack_scene = PackScene(self.model, self.scene_pack, self.tool_pack)
        arm_dof_count = 5 if self.robot in ("so-arm101", "so_arm101") else 6
        self.arm_dofs = self.indexer.qvel[: min(arm_dof_count, self.indexer.dof)]

        self.renderer: mujoco.Renderer | None = None
        if render_camera is not None:
            try:
                self.model.vis.global_.offheight = max(
                    self.model.vis.global_.offheight, self.render_height + 1
                )
                self.model.vis.global_.offwidth = max(
                    self.model.vis.global_.offwidth, self.render_width + 1
                )
                self.renderer = mujoco.Renderer(
                    self.model, height=self.render_height, width=self.render_width
                )
            except RuntimeError:
                self.renderer = None

        self._prev_score = 0.0
        self._randomizer: DomainRandomizer | None = None
        self._action_delay_buffer: ActionDelayBuffer | None = None
        self._obs_rng: np.random.Generator | None = None
        if self.dr_config is not None:
            self._randomizer = DomainRandomizer(self.model, self.dr_config)
            self._action_delay_buffer = ActionDelayBuffer(0)
            if self.dr_config.enabled:
                self._obs_rng = np.random.default_rng()

    @property
    def domain_randomizer(self) -> DomainRandomizer | None:
        """Active ``DomainRandomizer`` instance when ``dr_config`` is provided."""
        return self._randomizer

    @property
    def n_joints(self) -> int:
        """Number of controlled robot/tool joints exposed by the indexer."""
        return self.indexer.dof

    @property
    def action_dim(self) -> int:
        """Exact action vector length expected by ``step()``."""
        return self.motors.dof if self.control_mode == "position" else self.model.nu

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names matching ``obs['state']`` and ``action``."""
        return list(self.indexer.joints)

    def _validate_action(self, action: Sequence[float] | np.ndarray) -> np.ndarray:
        """Validate action dimensionality, length, numeric type, and finiteness."""
        return validate_real_vector(
            action,
            self.action_dim,
            label=f"action ({self.control_mode} mode)",
            exc_type=ValueError,
        )

    def _get_obs(self) -> dict[str, Any]:
        obs: dict[str, Any] = {
            "state": np.asarray(
                self.indexer.motor_positions(self.data), dtype=np.float32
            ),
            "velocity": np.asarray(
                self.indexer.motor_velocities(self.data), dtype=np.float32
            ),
            "objects": self.pack_scene.state(self.data),
            "io": self.pack_scene.io_state(self.data),
        }
        if self.render_camera is not None and self.renderer is not None:
            self.renderer.update_scene(self.data, camera=self.render_camera)
            obs["image"] = self.renderer.render()
        if self._randomizer is not None and self.dr_config is not None and self.dr_config.enabled:
            rng = self._obs_rng if self._obs_rng is not None else np.random.default_rng()
            obs = self._randomizer.apply_obs_noise(obs, rng)
            if "image" in obs:
                obs["image"] = self._randomizer.apply_image_noise(obs["image"], rng)
        return obs

    def reset(self, seed: int | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        """Reset arm joints to home neutral pose, reset scene objects, IO, and return (obs, info)."""
        dr_params: dict[str, Any] | None = None
        if self._randomizer is not None and self.dr_config is not None:
            if self.dr_config.enabled:
                dr_params = self._randomizer.apply(self.model, seed=seed, data=self.data)
                self._obs_rng = (
                    np.random.default_rng(np.random.SeedSequence(seed).spawn(1)[0])
                    if seed is not None
                    else np.random.default_rng()
                )
                if self._action_delay_buffer is not None:
                    self._action_delay_buffer.reset(delay_steps=dr_params["action_delay_steps"])
            else:
                self._randomizer.restore_baseline(self.model, data=self.data)
                self._obs_rng = None
                if self._action_delay_buffer is not None:
                    self._action_delay_buffer.reset(delay_steps=0)

        self.data.time = 0.0
        self.data.qpos[:] = self.model.qpos0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qfrc_applied[:] = 0.0

        self.indexer.set_motor_positions(self.data, np.zeros(self.indexer.dof))
        self.data.qvel[self.indexer.qvel] = 0.0

        self.pack_scene.reset(self.data, seed=seed)
        mujoco.mj_forward(self.model, self.data)
        self.motors.enabled = False
        self.motors.set_enabled(True)
        self.motors.vlim[:] = 2.5

        task = self.pack_scene.task_state(self.data)
        self._prev_score = float(task.get("score", 0.0)) if task else 0.0

        obs = self._get_obs()
        info: dict[str, Any] = {
            "task": task,
            "sim_time_s": float(self.data.time),
        }
        if dr_params is not None:
            info["dr_params"] = dr_params
        return obs, info

    def step(
        self,
        action: Sequence[float] | np.ndarray,
        io_updates: dict[str, Any] | None = None,
    ) -> tuple[dict[str, Any], float, bool, dict[str, Any]]:
        """Execute one control step (n_substeps physics steps) without real-time delays."""
        act = self._validate_action(action)
        if (
            self._action_delay_buffer is not None
            and self.dr_config is not None
            and self.dr_config.enabled
        ):
            act = self._action_delay_buffer.push(act)
        if io_updates:
            self.pack_scene.set_io(io_updates, self.data)

        if self.control_mode == "position":
            self.motors.q_t = act.copy()
            self.motors.mode = ["pos_vel"] * self.motors.dof
            for _ in range(self.n_substeps):
                self.motors.apply()
                mujoco.mj_step(self.model, self.data)
                self.pack_scene.step_io(self.data)
        else:
            for i in range(self.model.nu):
                val = float(act[i])
                if self.model.actuator_ctrllimited[i]:
                    val = float(
                        np.clip(
                            val,
                            self.model.actuator_ctrlrange[i, 0],
                            self.model.actuator_ctrlrange[i, 1],
                        )
                    )
                self.data.ctrl[i] = val

            for _ in range(self.n_substeps):
                self.data.qfrc_applied[:] = 0.0
                self.data.qfrc_applied[self.arm_dofs] = self.data.qfrc_bias[
                    self.arm_dofs
                ]
                mujoco.mj_step(self.model, self.data)
                self.pack_scene.step_io(self.data)

        task = self.pack_scene.task_state(self.data)
        score = float(task.get("score", 0.0)) if task else 0.0
        delta_score = score - self._prev_score
        self._prev_score = score

        completed = bool(task and task.get("completed", False))
        reward = float(delta_score + 0.01 * score + (1.0 if completed else 0.0))
        done = bool(task and task.get("completed"))

        obs = self._get_obs()
        info = {
            "task": task,
            "sim_time_s": float(self.data.time),
        }
        return obs, reward, done, info

    def set_io(self, updates: dict[str, Any]) -> dict[str, Any]:
        """Apply virtual tool or scene I/O updates and return current I/O state."""
        return self.pack_scene.set_io(updates, self.data)

    def close(self) -> None:
        """Release renderer and simulation resources."""
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: types.TracebackType | None,
    ) -> None:
        self.close()
