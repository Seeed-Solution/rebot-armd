"""Multi-seed headless MuJoCo evaluation CLI (`python -m rebot_sim.eval_cli`)."""
from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .camera import CameraMount
from .model import (
    SelfCollisionConfig,
    TorqueNoiseConfig,
    apply_torque_noise,
    detect_link_self_collisions,
    load_model,
)
from .pack import PackScene, load_scene_pack, load_tool_pack, normalize_robot_id
from .perception import DomainRandomizationConfig as PerceptionDRConfig
from .perception import SimDomainRandomizer
from .scene import Scene, parse_objects
from .server import Motors

EVAL_REPORT_SCHEMA = "rebot-sim-eval/v1"
_TORQUE_NOISE_SALT = 0x544F52515F4E53  # "TORQ_NS"
_VISUAL_OBS_KEYS = frozenset({"image", "camera_state"})

_B601DM_WAYPOINTS = (
    np.array([0.0, -0.5, -0.8, 0.0, 0.2, 0.0, -3.0], dtype=np.float64),
    np.array([0.2, -0.7, -1.1, 0.0, 0.25, 0.0, -3.0], dtype=np.float64),
    np.array([-0.2, -0.7, -1.1, 0.0, 0.25, 0.0, -3.0], dtype=np.float64),
    np.array([0.0, -0.5, -0.8, 0.0, 0.2, 0.0, -3.0], dtype=np.float64),
)

_SO101_WAYPOINTS = (
    np.array([0.0, -0.4, 0.6, 0.5, 0.0, 0.4], dtype=np.float64),
    np.array([0.25, -0.3, 0.7, 0.4, 0.0, 0.4], dtype=np.float64),
    np.array([-0.25, -0.3, 0.7, 0.4, 0.0, 0.4], dtype=np.float64),
    np.array([0.0, -0.4, 0.6, 0.5, 0.0, 0.4], dtype=np.float64),
)


def _is_real_scalar(val: Any) -> bool:
    return not isinstance(val, (bool, np.bool_)) and isinstance(
        val, (int, float, np.integer, np.floating)
    )


def _validate_non_negative_float(name: str, val: Any) -> float:
    if not _is_real_scalar(val):
        raise ValueError(f"{name} must be a real number, got {type(val).__name__}")
    out = float(val)
    if not math.isfinite(out) or out < 0.0:
        raise ValueError(f"{name} must be finite and >= 0.0, got {val!r}")
    return out


class _TrackedObs(dict):
    """Observation dict that records whether visual/camera fields were read by a policy."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.visual_keys_read: set[str] = set()

    def __getitem__(self, key: str) -> Any:
        if key in _VISUAL_OBS_KEYS:
            self.visual_keys_read.add(key)
        return super().__getitem__(key)

    def get(self, key: str, default: Any = None) -> Any:
        if key in _VISUAL_OBS_KEYS and key in self:
            self.visual_keys_read.add(key)
        return super().get(key, default)


@dataclass(frozen=True)
class SimEvalConfig:
    """Configuration for deterministic multi-seed headless evaluation."""

    seeds: int = 20
    start_seed: int = 0
    robot: str = "b601dm"
    scene: str | None = None
    tool: str | None = None
    objects: str = "cube"
    dr_lighting: bool = False
    dr_camera_jitter_m: float = 0.0
    dr_camera_jitter_deg: float = 0.0
    torque_noise_std: float = 0.0
    torque_noise_bias: float = 0.0
    max_steps: int = 150
    control_dt: float = 0.02
    sim_dt: float = 0.002
    self_collision: bool = True
    fail_on_self_collision: bool | None = None
    render_camera: str | None = None
    render_width: int = 160
    render_height: int = 120
    output: str | None = None

    def __post_init__(self) -> None:
        if isinstance(self.seeds, bool) or not isinstance(self.seeds, int) or self.seeds <= 0:
            raise ValueError(f"seeds must be a positive int, got {self.seeds!r}")
        if (
            isinstance(self.start_seed, bool)
            or not isinstance(self.start_seed, int)
            or self.start_seed < 0
        ):
            raise ValueError(f"start_seed must be a non-negative int, got {self.start_seed!r}")
        norm_robot = normalize_robot_id(self.robot)
        if norm_robot not in ("b601dm", "so-arm101"):
            raise ValueError(f"robot must be 'b601dm' or 'so-arm101', got {self.robot!r}")
        object.__setattr__(self, "robot", norm_robot)
        if not isinstance(self.dr_lighting, bool):
            raise ValueError(f"dr_lighting must be a bool, got {type(self.dr_lighting).__name__}")
        object.__setattr__(
            self,
            "dr_camera_jitter_m",
            _validate_non_negative_float("dr_camera_jitter_m", self.dr_camera_jitter_m),
        )
        object.__setattr__(
            self,
            "dr_camera_jitter_deg",
            _validate_non_negative_float("dr_camera_jitter_deg", self.dr_camera_jitter_deg),
        )
        object.__setattr__(
            self,
            "torque_noise_std",
            _validate_non_negative_float("torque_noise_std", self.torque_noise_std),
        )
        if not _is_real_scalar(self.torque_noise_bias) or not math.isfinite(float(self.torque_noise_bias)):
            raise ValueError(f"torque_noise_bias must be finite, got {self.torque_noise_bias!r}")
        object.__setattr__(self, "torque_noise_bias", float(self.torque_noise_bias))
        if (
            isinstance(self.max_steps, bool)
            or not isinstance(self.max_steps, int)
            or self.max_steps <= 0
        ):
            raise ValueError(f"max_steps must be a positive int, got {self.max_steps!r}")
        cdt = _validate_non_negative_float("control_dt", self.control_dt)
        sdt = _validate_non_negative_float("sim_dt", self.sim_dt)
        if cdt <= 0.0 or sdt <= 0.0:
            raise ValueError("control_dt and sim_dt must be > 0.0")
        object.__setattr__(self, "control_dt", cdt)
        object.__setattr__(self, "sim_dt", sdt)
        if not isinstance(self.self_collision, bool):
            raise ValueError(
                f"self_collision must be a bool, got {type(self.self_collision).__name__}"
            )
        if self.fail_on_self_collision is None:
            object.__setattr__(self, "fail_on_self_collision", self.self_collision)
        elif not isinstance(self.fail_on_self_collision, bool):
            raise ValueError(
                f"fail_on_self_collision must be a bool or None, got {type(self.fail_on_self_collision).__name__}"
            )
        elif not self.self_collision and self.fail_on_self_collision:
            raise ValueError("fail_on_self_collision=True requires self_collision=True")
        if (
            isinstance(self.render_width, bool)
            or not isinstance(self.render_width, int)
            or self.render_width <= 0
        ):
            raise ValueError(f"render_width must be a positive int, got {self.render_width!r}")
        if (
            isinstance(self.render_height, bool)
            or not isinstance(self.render_height, int)
            or self.render_height <= 0
        ):
            raise ValueError(f"render_height must be a positive int, got {self.render_height!r}")
        if self.output is not None:
            object.__setattr__(self, "output", str(self.output))


def _scripted_baseline_policy(obs: dict[str, Any]) -> np.ndarray:
    """Lightweight deterministic scripted waypoint policy for headless CI evaluation."""
    robot = obs["robot"]
    dof = int(obs["dof"])
    step = int(obs["step"])
    max_steps = max(1, int(obs["max_steps"]))
    waypoints = _SO101_WAYPOINTS if robot == "so-arm101" else _B601DM_WAYPOINTS
    idx = min(len(waypoints) - 1, (step * len(waypoints)) // max_steps)
    wp = waypoints[idx]
    if len(wp) >= dof:
        return wp[:dof].copy()
    out = np.zeros(dof, dtype=np.float64)
    out[: len(wp)] = wp
    return out


def _load_policy_callable(spec: str) -> Callable[[dict[str, Any]], Any]:
    if ":" not in spec:
        raise ValueError(f"policy spec must be 'module:func', got {spec!r}")
    mod_name, attr = spec.split(":", 1)
    mod = importlib.import_module(mod_name)
    fn = getattr(mod, attr)
    if not callable(fn):
        raise TypeError(f"policy {spec!r} is not callable")
    return fn


def _parse_policy_output(raw: Any, dof: int) -> tuple[np.ndarray, dict[str, Any] | None, bool]:
    io_updates: dict[str, Any] | None = None
    stop_early = False
    act_raw = raw
    if isinstance(raw, dict):
        act_raw = raw.get("action")
        io_updates = raw.get("io")
        stop_early = bool(raw.get("done", False))
    elif isinstance(raw, tuple) and len(raw) == 2 and isinstance(raw[1], dict):
        act_raw, io_updates = raw
    arr = np.asarray(act_raw, dtype=np.float64)
    if arr.ndim != 1 or arr.shape[0] != dof or not np.all(np.isfinite(arr)):
        raise ValueError(
            f"policy action must be a finite 1D vector of length {dof}, got shape {arr.shape}"
        )
    return arr, io_updates, stop_early


def _camera_lighting_state(model: mujoco.MjModel) -> dict[str, Any]:
    return {
        "cam_pos": model.cam_pos.copy(),
        "cam_quat": model.cam_quat.copy(),
        "light_diffuse": model.light_diffuse.copy(),
        "light_specular": model.light_specular.copy(),
        "light_ambient": model.light_ambient.copy(),
        "light_pos": model.light_pos.copy(),
    }


def _safe_tcp_list(tcp_vec: np.ndarray) -> tuple[list[float | None], bool]:
    out: list[float | None] = []
    all_finite = True
    for v in tcp_vec:
        fv = float(v)
        if math.isfinite(fv):
            out.append(round(fv, 6))
        else:
            out.append(None)
            all_finite = False
    return out, all_finite


def run_sim_eval(
    config: SimEvalConfig | None = None,
    *,
    policy: Callable[[dict[str, Any]], Any] | None = None,
    success_fn: Callable[[dict[str, Any]], bool] | None = None,
) -> dict[str, Any]:
    """Run deterministic multi-seed headless evaluation and return a ``rebot-sim-eval/v1`` report."""
    cfg = config if config is not None else SimEvalConfig()
    active_policy = policy if policy is not None else _scripted_baseline_policy
    using_default_policy = policy is None

    scene_pack = load_scene_pack(cfg.scene) if cfg.scene is not None else None
    effective_tool = (
        cfg.tool
        if cfg.tool is not None
        else (scene_pack.default_tool if scene_pack is not None else "default")
    )
    tool_pack = load_tool_pack(effective_tool)
    sc_cfg = SelfCollisionConfig(enabled=cfg.self_collision)

    cam_mount = (
        CameraMount()
        if cfg.robot == "b601dm" and scene_pack is None
        else None
    )
    if scene_pack is not None:
        model = load_model(
            robot=cfg.robot,
            scene_pack=scene_pack,
            tool_pack=tool_pack,
            camera=cam_mount,
            self_collision=sc_cfg,
        )
        scene_mgr: PackScene | Scene = PackScene(model, scene_pack, tool_pack)
    else:
        parsed_objs = parse_objects(cfg.objects)
        model = load_model(
            objects=parsed_objs,
            camera=cam_mount,
            robot=cfg.robot,
            tool_pack=tool_pack,
            self_collision=sc_cfg,
        )
        if not tool_pack.is_default:
            scene_mgr = PackScene(model, None, tool_pack, fallback_objects=parsed_objs)
        else:
            scene_mgr = Scene(model, parsed_objs)

    model.opt.timestep = cfg.sim_dt
    model.actuator_biasprm[:, 1] = 0.0
    n_substeps = max(1, round(cfg.control_dt / cfg.sim_dt))

    renderer: mujoco.Renderer | None = None
    if cfg.render_camera is not None:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cfg.render_camera)
        if cam_id < 0:
            raise ValueError(f"Unknown render_camera {cfg.render_camera!r}")
        model.vis.global_.offheight = max(model.vis.global_.offheight, cfg.render_height + 1)
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, cfg.render_width + 1)
        renderer = mujoco.Renderer(model, height=cfg.render_height, width=cfg.render_width)

    dr_enabled = bool(
        cfg.dr_lighting
        or cfg.dr_camera_jitter_m > 0.0
        or cfg.dr_camera_jitter_deg > 0.0
    )
    dr_cfg = PerceptionDRConfig(
        enabled=dr_enabled,
        lighting=cfg.dr_lighting,
        camera_jitter_m=cfg.dr_camera_jitter_m,
        camera_jitter_deg=cfg.dr_camera_jitter_deg,
    )
    randomizer = SimDomainRandomizer(model, dr_cfg)
    torque_cfg = TorqueNoiseConfig(
        enabled=cfg.torque_noise_std > 0.0 or cfg.torque_noise_bias != 0.0,
        std_nm=cfg.torque_noise_std,
        bias_nm=cfg.torque_noise_bias,
    )

    results: list[dict[str, Any]] = []
    try:
        for offset in range(cfg.seeds):
            seed = cfg.start_seed + offset
            t0 = time.perf_counter()

            # Fresh MjData + Motors per seed prevents solver warm-start state from leaking across seeds.
            data = mujoco.MjData(model)
            motors = Motors(model, data, tool_pack=tool_pack)
            indexer = motors.idx

            indexer.set_motor_positions(data, np.zeros(indexer.dof, dtype=np.float64))
            data.qvel[indexer.qvel] = 0.0

            if dr_enabled:
                dr_summary = randomizer.apply(model, seed=seed)
                dr_applied = bool(dr_summary.get("enabled", False))
            else:
                randomizer.restore_baseline(model)
                dr_summary = None
                dr_applied = False

            scene_mgr.reset(data, seed=seed)
            mujoco.mj_forward(model, data)

            motors.enabled = False
            motors.set_enabled(True)
            motors.vlim[:] = 2.5

            torque_rng = (
                np.random.default_rng(np.random.SeedSequence([seed, _TORQUE_NOISE_SALT]))
                if torque_cfg.enabled
                else None
            )

            self_collision_steps = 0
            colliding_pairs: set[tuple[str, str]] = set()
            visual_keys_read: set[str] = set()
            steps_run = 0
            error: str | None = None
            last_target: np.ndarray | None = None
            task_state: dict[str, Any] | None = None
            success = False

            try:
                for step_idx in range(cfg.max_steps):
                    tcp_pos, tcp_quat = indexer.tcp_pose(data)
                    obs = _TrackedObs(
                        {
                            "state": indexer.motor_positions(data).astype(np.float32),
                            "velocity": indexer.motor_velocities(data).astype(np.float32),
                            "tcp": tcp_pos.copy(),
                            "tcp_quat": tcp_quat.copy(),
                            "step": step_idx,
                            "max_steps": cfg.max_steps,
                            "sim_time_s": float(data.time),
                            "seed": seed,
                            "dof": motors.dof,
                            "robot": cfg.robot,
                            "objects": scene_mgr.state(data),
                            "io": scene_mgr.io_state(data) if isinstance(scene_mgr, PackScene) else {},
                            "camera_state": _camera_lighting_state(model),
                            "dr_summary": dr_summary,
                        }
                    )
                    if renderer is not None and cfg.render_camera is not None:
                        renderer.update_scene(data, camera=cfg.render_camera)
                        obs["image"] = renderer.render()

                    raw_action = active_policy(obs)
                    visual_keys_read.update(obs.visual_keys_read)
                    if raw_action is None:
                        break
                    target, io_updates, stop_early = _parse_policy_output(raw_action, motors.dof)
                    last_target = target
                    if io_updates and isinstance(scene_mgr, PackScene):
                        scene_mgr.set_io(io_updates, data)

                    motors.q_t = target.copy()
                    motors.mode = ["pos_vel"] * motors.dof

                    step_collided = False
                    for _ in range(n_substeps):
                        motors.apply()
                        if torque_cfg.enabled:
                            noisy_extra = apply_torque_noise(
                                motors.extra, rng=torque_rng, config=torque_cfg
                            )
                            data.qfrc_applied[motors.dofs] = motors.gear * noisy_extra
                        mujoco.mj_step(model, data)
                        mujoco.mj_forward(model, data)
                        if isinstance(scene_mgr, PackScene):
                            scene_mgr.step_io(data)
                        if cfg.self_collision:
                            sc = detect_link_self_collisions(model, data, config=sc_cfg)
                            if sc.count > 0:
                                step_collided = True
                                colliding_pairs.update(sc.pairs)

                    steps_run += 1
                    if step_collided:
                        self_collision_steps += 1

                    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                        error = "non-finite simulation state"
                        break

                    if isinstance(scene_mgr, PackScene):
                        task_state = scene_mgr.task_state(data)
                        if task_state and task_state.get("completed"):
                            break
                    if stop_early:
                        break

                if isinstance(scene_mgr, PackScene) and task_state is None:
                    task_state = scene_mgr.task_state(data)

                final_tcp_vec, _ = indexer.tcp_pose(data)
                final_tcp, tcp_finite = _safe_tcp_list(final_tcp_vec)

                collision_ok = (
                    True
                    if not cfg.self_collision or not cfg.fail_on_self_collision
                    else self_collision_steps == 0
                )
                if error is not None or not tcp_finite:
                    success = False
                elif success_fn is not None:
                    success = bool(
                        success_fn(
                            {
                                "seed": seed,
                                "steps": steps_run,
                                "self_collisions": self_collision_steps if cfg.self_collision else None,
                                "self_collision_pairs": sorted(colliding_pairs) if cfg.self_collision else None,
                                "final_tcp": final_tcp,
                                "task": task_state,
                                "state": indexer.motor_positions(data),
                                "last_target": last_target,
                            }
                        )
                    ) and collision_ok
                elif task_state is not None:
                    success = bool(task_state.get("completed", False)) and collision_ok
                elif steps_run == 0:
                    success = False
                elif using_default_policy and last_target is not None:
                    q_err = float(np.max(np.abs(indexer.motor_positions(data) - last_target)))
                    success = collision_ok and q_err <= 0.15
                else:
                    success = collision_ok
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                success = False
                final_tcp_vec, _ = indexer.tcp_pose(data)
                final_tcp, _ = _safe_tcp_list(final_tcp_vec)

            duration_s = round(time.perf_counter() - t0, 6)
            visual_dr_observed = bool(dr_applied and visual_keys_read)

            results.append(
                {
                    "seed": seed,
                    "success": bool(success),
                    "steps": steps_run,
                    "duration_s": duration_s,
                    "self_collisions": self_collision_steps if cfg.self_collision else None,
                    "self_collision_pairs": (
                        [[a, b] for a, b in sorted(colliding_pairs)]
                        if cfg.self_collision
                        else None
                    ),
                    "dr_applied": dr_applied,
                    "visual_dr_observed": visual_dr_observed,
                    "torque_noise_std": cfg.torque_noise_std,
                    "final_tcp": final_tcp,
                    "error": error,
                }
            )
    finally:
        if renderer is not None:
            renderer.close()

    total_seeds = len(results)
    passed_seeds = sum(1 for r in results if r["success"])
    if cfg.self_collision:
        total_self_collisions: int | None = sum(
            int(r["self_collisions"] or 0) for r in results
        )
        collision_free_seeds = sum(1 for r in results if r["self_collisions"] == 0)
        collision_free_rate: float | None = (
            round(collision_free_seeds / total_seeds, 6) if total_seeds else 0.0
        )
    else:
        total_self_collisions = None
        collision_free_rate = None
    visual_dr_observed_seeds = sum(1 for r in results if r["visual_dr_observed"])
    mean_steps = (
        round(sum(r["steps"] for r in results) / total_seeds, 4) if total_seeds else 0.0
    )

    report: dict[str, Any] = {
        "schema": EVAL_REPORT_SCHEMA,
        "total_seeds": total_seeds,
        "passed_seeds": passed_seeds,
        "success_rate": round(passed_seeds / total_seeds, 6) if total_seeds else 0.0,
        "total_self_collisions": total_self_collisions,
        "collision_free_rate": collision_free_rate,
        "visual_dr_observed_seeds": visual_dr_observed_seeds,
        "mean_steps": mean_steps,
        "config": asdict(cfg),
        "results": results,
    }

    if cfg.output is not None:
        out_path = Path(cfg.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    return report


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rebot-sim-eval",
        description="Deterministic multi-seed MuJoCo evaluation harness for rebot-sim.",
    )
    parser.add_argument("--seeds", type=int, default=20, help="number of seeds to evaluate (default 20)")
    parser.add_argument("--start-seed", type=int, default=0, help="starting seed (default 0)")
    parser.add_argument(
        "--robot",
        choices=("b601dm", "so-arm101", "so_arm101"),
        default="b601dm",
        help="robot model (default b601dm)",
    )
    parser.add_argument("--scene", default=None, help="optional scene pack id, directory, or XML")
    parser.add_argument("--tool", default=None, help="optional tool pack id, preset, or XML")
    parser.add_argument("--objects", default="cube", help="scene objects when --scene is omitted (default cube)")
    parser.add_argument("--dr-lighting", action="store_true", help="enable deterministic lighting DR")
    parser.add_argument(
        "--dr-camera-jitter-m",
        type=float,
        default=0.0,
        help="max camera position jitter in metres (default 0.0)",
    )
    parser.add_argument(
        "--dr-camera-jitter-deg",
        type=float,
        default=0.0,
        help="max camera orientation jitter in degrees (default 0.0)",
    )
    parser.add_argument(
        "--torque-noise-std",
        type=float,
        default=0.0,
        help="actuator torque noise std in N*m (default 0.0)",
    )
    parser.add_argument(
        "--torque-noise-bias",
        type=float,
        default=0.0,
        help="actuator torque noise bias in N*m (default 0.0)",
    )
    parser.add_argument("--max-steps", type=int, default=150, help="max control steps per seed (default 150)")
    parser.add_argument(
        "--render-camera",
        default=None,
        help="optional camera name to render into obs['image'] for visual policies",
    )
    parser.add_argument(
        "--policy",
        default=None,
        help="optional policy callable 'module:func' (defaults to built-in scripted baseline)",
    )
    parser.add_argument(
        "--no-self-collision",
        action="store_true",
        help="disable link self-collision checking (marks collision_free_rate as null)",
    )
    parser.add_argument(
        "--no-fail-on-self-collision",
        action="store_true",
        help="record self-collisions without automatically marking the episode failed",
    )
    parser.add_argument("--output", default=None, help="write JSON evaluation report to this path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        self_collision = not bool(args.no_self_collision)
        fail_on_sc = False if not self_collision else not bool(args.no_fail_on_self_collision)
        cfg = SimEvalConfig(
            seeds=args.seeds,
            start_seed=args.start_seed,
            robot=args.robot,
            scene=args.scene,
            tool=args.tool,
            objects=args.objects,
            dr_lighting=bool(args.dr_lighting),
            dr_camera_jitter_m=args.dr_camera_jitter_m,
            dr_camera_jitter_deg=args.dr_camera_jitter_deg,
            torque_noise_std=args.torque_noise_std,
            torque_noise_bias=args.torque_noise_bias,
            max_steps=args.max_steps,
            self_collision=self_collision,
            fail_on_self_collision=fail_on_sc,
            render_camera=args.render_camera,
            output=args.output,
        )
        policy_fn = _load_policy_callable(args.policy) if args.policy else None
    except ValueError as exc:
        parser.error(str(exc))

    report = run_sim_eval(cfg, policy=policy_fn)
    print(
        json.dumps(
            {
                "schema": report["schema"],
                "total_seeds": report["total_seeds"],
                "passed_seeds": report["passed_seeds"],
                "success_rate": report["success_rate"],
                "total_self_collisions": report["total_self_collisions"],
                "collision_free_rate": report["collision_free_rate"],
                "visual_dr_observed_seeds": report["visual_dr_observed_seeds"],
                "mean_steps": report["mean_steps"],
            },
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
