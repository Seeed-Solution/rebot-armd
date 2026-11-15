"""Non-realtime fast training environment wrapping composable scene packs, tools, and kinematics."""

from __future__ import annotations

import types
from collections.abc import Sequence
from typing import Any, Self

import mujoco
import numpy as np

from .camera import CameraMount
from .model import load_model
from .pack import PackScene, load_scene_pack, load_tool_pack
from .server import Motors


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
    ) -> None:
        if control_mode not in ("position", "velocity"):
            raise ValueError(
                f"Unsupported control_mode {control_mode!r}: expected 'position' or 'velocity'"
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
        return obs

    def reset(self, seed: int | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        """Reset arm joints to home neutral pose, reset scene objects, IO, and return (obs, info)."""
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
        info = {
            "task": task,
            "sim_time_s": float(self.data.time),
        }
        return obs, info

    def step(
        self,
        action: Sequence[float] | np.ndarray,
        io_updates: dict[str, Any] | None = None,
    ) -> tuple[dict[str, Any], float, bool, dict[str, Any]]:
        """Execute one control step (n_substeps physics steps) without real-time delays."""
        act = np.asarray(action, dtype=float).ravel()
        if io_updates:
            self.pack_scene.set_io(io_updates, self.data)

        if self.control_mode == "position":
            q_t = self.motors.q_t.copy()
            n = min(len(act), len(q_t))
            q_t[:n] = act[:n]
            self.motors.q_t = q_t
            self.motors.mode = ["pos_vel"] * self.motors.dof
            for _ in range(self.n_substeps):
                self.motors.apply()
                mujoco.mj_step(self.model, self.data)
                self.pack_scene.step_io(self.data)
        else:
            n_act = min(len(act), self.model.nu)
            for i in range(n_act):
                val = act[i]
                if self.model.actuator_ctrllimited[i]:
                    val = np.clip(
                        val,
                        self.model.actuator_ctrlrange[i, 0],
                        self.model.actuator_ctrlrange[i, 1],
                    )
                self.data.ctrl[i] = val
            if n_act < self.model.nu:
                self.data.ctrl[n_act:] = 0.0

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
