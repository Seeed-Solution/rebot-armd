"""Physics and visual domain randomization for ``SimEnv``."""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np


def _is_bool(val: Any) -> bool:
    return isinstance(val, (bool, np.bool_))


def _is_real_scalar(val: Any) -> bool:
    return isinstance(val, (int, float, np.integer, np.floating)) and not _is_bool(val)


def _is_int_scalar(val: Any) -> bool:
    return isinstance(val, (int, np.integer)) and not _is_bool(val)


def _validate_float_range(
    name: str,
    val: Any,
    *,
    min_val: float = 0.0,
    strict_positive: bool = False,
) -> tuple[float, float]:
    if isinstance(val, (str, bytes, bytearray)) or not isinstance(val, (Sequence, np.ndarray)):
        raise ValueError(f"{name} must be a 2-element sequence of floats, got {type(val).__name__}")
    if isinstance(val, np.ndarray) and val.ndim != 1:
        raise ValueError(f"{name} must be 1D with length 2, got shape {val.shape}")
    if len(val) != 2:
        raise ValueError(f"{name} must have exactly 2 elements (min, max), got {len(val)}")
    raw_lo, raw_hi = val[0], val[1]
    if not _is_real_scalar(raw_lo) or not _is_real_scalar(raw_hi):
        raise ValueError(f"{name} elements must be real numbers, got ({type(raw_lo).__name__}, {type(raw_hi).__name__})")
    lo, hi = float(raw_lo), float(raw_hi)
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError(f"{name} bounds must be finite, got ({lo}, {hi})")
    if lo > hi:
        raise ValueError(f"{name}: min ({lo}) must be <= max ({hi})")
    if strict_positive:
        if lo <= min_val:
            raise ValueError(f"{name}: min ({lo}) must be > {min_val}")
    elif lo < min_val:
        raise ValueError(f"{name}: min ({lo}) must be >= {min_val}")
    return (lo, hi)


def _validate_int_range(name: str, val: Any, *, min_val: int = 0) -> tuple[int, int]:
    if isinstance(val, (str, bytes, bytearray)) or not isinstance(val, (Sequence, np.ndarray)):
        raise ValueError(f"{name} must be a 2-element sequence of ints, got {type(val).__name__}")
    if isinstance(val, np.ndarray) and val.ndim != 1:
        raise ValueError(f"{name} must be 1D with length 2, got shape {val.shape}")
    if len(val) != 2:
        raise ValueError(f"{name} must have exactly 2 elements (min, max), got {len(val)}")
    raw_lo, raw_hi = val[0], val[1]
    if not _is_int_scalar(raw_lo) or not _is_int_scalar(raw_hi):
        raise ValueError(f"{name} elements must be integers, got ({type(raw_lo).__name__}, {type(raw_hi).__name__})")
    lo, hi = int(raw_lo), int(raw_hi)
    if lo < min_val:
        raise ValueError(f"{name}: min ({lo}) must be >= {min_val}")
    if lo > hi:
        raise ValueError(f"{name}: min ({lo}) must be <= max ({hi})")
    return (lo, hi)


def _validate_non_negative_float(name: str, val: Any) -> float:
    if not _is_real_scalar(val):
        raise ValueError(f"{name} must be a real number, got {type(val).__name__}")
    out = float(val)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {out}")
    if out < 0.0:
        raise ValueError(f"{name} must be >= 0.0, got {out}")
    return out


@dataclass
class DomainRandomizationConfig:
    """Configuration for physics and visual domain randomization in ``SimEnv``.

    Image parameters (``image_brightness_delta``, ``image_noise_std``) are expressed in
    pixel intensity units on the ``[0, 255]`` ``uint8`` scale.
    """

    enabled: bool = True

    # Physics randomization
    friction_scale_range: tuple[float, float] = (0.7, 1.3)
    mass_scale_range: tuple[float, float] = (0.85, 1.15)
    damping_scale_range: tuple[float, float] = (0.8, 1.2)
    action_delay_steps_range: tuple[int, int] = (0, 2)
    obs_pos_noise_std: float = 0.002
    obs_vel_noise_std: float = 0.01

    # Visual randomization
    cam_pos_jitter_m: float = 0.005
    cam_rot_jitter_deg: float = 1.5
    light_intensity_scale_range: tuple[float, float] = (0.7, 1.3)
    image_brightness_delta: float = 15.0
    image_contrast_scale_range: tuple[float, float] = (0.85, 1.15)
    image_noise_std: float = 4.0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError(f"enabled must be a bool, got {type(self.enabled).__name__}")
        self.friction_scale_range = _validate_float_range(
            "friction_scale_range", self.friction_scale_range, min_val=0.0
        )
        self.mass_scale_range = _validate_float_range(
            "mass_scale_range", self.mass_scale_range, min_val=0.0, strict_positive=True
        )
        self.damping_scale_range = _validate_float_range(
            "damping_scale_range", self.damping_scale_range, min_val=0.0
        )
        self.action_delay_steps_range = _validate_int_range(
            "action_delay_steps_range", self.action_delay_steps_range, min_val=0
        )
        self.obs_pos_noise_std = _validate_non_negative_float(
            "obs_pos_noise_std", self.obs_pos_noise_std
        )
        self.obs_vel_noise_std = _validate_non_negative_float(
            "obs_vel_noise_std", self.obs_vel_noise_std
        )
        self.cam_pos_jitter_m = _validate_non_negative_float(
            "cam_pos_jitter_m", self.cam_pos_jitter_m
        )
        self.cam_rot_jitter_deg = _validate_non_negative_float(
            "cam_rot_jitter_deg", self.cam_rot_jitter_deg
        )
        self.light_intensity_scale_range = _validate_float_range(
            "light_intensity_scale_range", self.light_intensity_scale_range, min_val=0.0
        )
        self.image_brightness_delta = _validate_non_negative_float(
            "image_brightness_delta", self.image_brightness_delta
        )
        self.image_contrast_scale_range = _validate_float_range(
            "image_contrast_scale_range", self.image_contrast_scale_range, min_val=0.0
        )
        self.image_noise_std = _validate_non_negative_float(
            "image_noise_std", self.image_noise_std
        )


class ActionDelayBuffer:
    """FIFO action delay buffer supporting ``delay_steps`` steps and episode reset."""

    def __init__(self, delay_steps: int = 0) -> None:
        self._delay_steps = self._validate_delay(delay_steps)
        self._queue: deque[np.ndarray] = deque()

    @staticmethod
    def _validate_delay(delay_steps: int) -> int:
        if not _is_int_scalar(delay_steps):
            raise ValueError(f"delay_steps must be an int, got {type(delay_steps).__name__}")
        val = int(delay_steps)
        if val < 0:
            raise ValueError(f"delay_steps must be >= 0, got {val}")
        return val

    @property
    def delay_steps(self) -> int:
        """Configured delay in control steps."""
        return self._delay_steps

    def __len__(self) -> int:
        return len(self._queue)

    def reset(self, delay_steps: int | None = None) -> None:
        """Clear queued actions and optionally update ``delay_steps``."""
        self._queue.clear()
        if delay_steps is not None:
            self._delay_steps = self._validate_delay(delay_steps)

    def push(
        self,
        action: Sequence[float] | np.ndarray,
        default_action: Sequence[float] | np.ndarray | None = None,
    ) -> np.ndarray:
        """Enqueue ``action`` and return the delayed action ready for this step."""
        arr = np.asarray(action, dtype=np.float64).copy()
        if self._delay_steps == 0:
            return arr
        self._queue.append(arr)
        if len(self._queue) <= self._delay_steps:
            if default_action is not None:
                fallback = np.asarray(default_action, dtype=np.float64).copy()
                if fallback.shape != arr.shape:
                    raise ValueError(
                        f"default_action shape {fallback.shape} does not match action shape {arr.shape}"
                    )
                return fallback
            return np.zeros_like(arr)
        return self._queue.popleft()


class DomainRandomizer:
    """Snapshots baseline ``MjModel`` parameters and applies drift-free per-episode DR."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config: DomainRandomizationConfig | None = None,
    ) -> None:
        self.config = config if config is not None else DomainRandomizationConfig()
        if not isinstance(self.config, DomainRandomizationConfig):
            raise TypeError(
                f"config must be DomainRandomizationConfig, got {type(self.config).__name__}"
            )
        self._baseline: dict[str, np.ndarray] = {
            "body_mass": model.body_mass.copy(),
            "body_inertia": model.body_inertia.copy(),
            "geom_friction": model.geom_friction.copy(),
            "dof_damping": model.dof_damping.copy(),
            "cam_pos": model.cam_pos.copy(),
            "cam_quat": model.cam_quat.copy(),
            "light_diffuse": model.light_diffuse.copy(),
            "light_ambient": model.light_ambient.copy(),
        }
        self._scratch_data: mujoco.MjData | None = None
        self._episode_brightness_delta: float | None = None
        self._episode_contrast_scale: float | None = None

    def _get_scratch_data(self, model: mujoco.MjModel) -> mujoco.MjData:
        if self._scratch_data is None:
            self._scratch_data = mujoco.MjData(model)
        return self._scratch_data

    def restore_baseline(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData | None = None,
    ) -> None:
        """Restore all snapshotted ``MjModel`` parameters back to their exact baseline."""
        model.body_mass[:] = self._baseline["body_mass"]
        model.body_inertia[:] = self._baseline["body_inertia"]
        model.geom_friction[:] = self._baseline["geom_friction"]
        model.dof_damping[:] = self._baseline["dof_damping"]
        model.cam_pos[:] = self._baseline["cam_pos"]
        model.cam_quat[:] = self._baseline["cam_quat"]
        model.light_diffuse[:] = self._baseline["light_diffuse"]
        model.light_ambient[:] = self._baseline["light_ambient"]
        mujoco.mj_setConst(model, data if data is not None else self._get_scratch_data(model))
        self._episode_brightness_delta = None
        self._episode_contrast_scale = None

    def apply(
        self,
        model: mujoco.MjModel,
        seed: int | None = None,
        data: mujoco.MjData | None = None,
    ) -> dict[str, Any]:
        """Restore baseline, sample new DR parameters from ``seed``, apply to ``model``, and return ``dr_params``."""
        if seed is not None:
            if _is_bool(seed) or not isinstance(seed, (int, np.integer)):
                raise ValueError(f"seed must be an int or None, got {type(seed).__name__}")
            seed = int(seed)

        self.restore_baseline(model, data=data)
        cfg = self.config
        if not cfg.enabled:
            self._episode_brightness_delta = 0.0
            self._episode_contrast_scale = 1.0
            return {
                "enabled": False,
                "seed": seed,
                "action_delay_steps": 0,
            }

        rng = np.random.default_rng(seed)

        # Physics parameters
        if model.ngeom > 0:
            friction_scales = rng.uniform(*cfg.friction_scale_range, size=model.ngeom)
            model.geom_friction[:] = self._baseline["geom_friction"] * friction_scales[:, None]
        else:
            friction_scales = np.empty(0, dtype=np.float64)

        if model.nbody > 1:
            body_scales = rng.uniform(*cfg.mass_scale_range, size=model.nbody - 1)
            model.body_mass[1:] = self._baseline["body_mass"][1:] * body_scales
            model.body_inertia[1:] = self._baseline["body_inertia"][1:] * body_scales[:, None]
        else:
            body_scales = np.empty(0, dtype=np.float64)

        if model.nv > 0:
            damping_scales = rng.uniform(*cfg.damping_scale_range, size=model.nv)
            model.dof_damping[:] = self._baseline["dof_damping"] * damping_scales
        else:
            damping_scales = np.empty(0, dtype=np.float64)

        delay_lo, delay_hi = cfg.action_delay_steps_range
        action_delay_steps = int(rng.integers(delay_lo, delay_hi + 1))

        # Visual parameters (cameras & lights)
        if model.ncam > 0:
            cam_pos_offsets = rng.uniform(
                -cfg.cam_pos_jitter_m, cfg.cam_pos_jitter_m, size=(model.ncam, 3)
            )
            model.cam_pos[:] = self._baseline["cam_pos"] + cam_pos_offsets

            cam_rot_offsets_deg = rng.uniform(
                -cfg.cam_rot_jitter_deg, cfg.cam_rot_jitter_deg, size=(model.ncam, 3)
            )
            q_delta = np.empty(4, dtype=np.float64)
            q_out = np.empty(4, dtype=np.float64)
            for i in range(model.ncam):
                rotvec = np.deg2rad(cam_rot_offsets_deg[i])
                angle = float(np.linalg.norm(rotvec))
                if angle > 1e-12:
                    mujoco.mju_axisAngle2Quat(q_delta, rotvec / angle, angle)
                    mujoco.mju_mulQuat(q_out, self._baseline["cam_quat"][i], q_delta)
                    norm = float(np.linalg.norm(q_out))
                    if norm > 0.0:
                        q_out /= norm
                    model.cam_quat[i] = q_out
        else:
            cam_pos_offsets = np.empty((0, 3), dtype=np.float64)
            cam_rot_offsets_deg = np.empty((0, 3), dtype=np.float64)

        if model.nlight > 0:
            light_scales = rng.uniform(*cfg.light_intensity_scale_range, size=model.nlight)
            light_scales_f32 = light_scales[:, None].astype(np.float32)
            model.light_diffuse[:] = np.clip(
                self._baseline["light_diffuse"] * light_scales_f32, 0.0, None
            )
            model.light_ambient[:] = np.clip(
                self._baseline["light_ambient"] * light_scales_f32, 0.0, None
            )
        else:
            light_scales = np.empty(0, dtype=np.float64)

        brightness_delta = float(
            rng.uniform(-cfg.image_brightness_delta, cfg.image_brightness_delta)
        )
        contrast_scale = float(rng.uniform(*cfg.image_contrast_scale_range))
        self._episode_brightness_delta = brightness_delta
        self._episode_contrast_scale = contrast_scale

        # Recompute derived inertial constants (body_subtreemass, dof_invweight0, etc.).
        mujoco.mj_setConst(model, data if data is not None else self._get_scratch_data(model))

        return {
            "enabled": True,
            "seed": seed,
            "friction_scales": [float(x) for x in friction_scales],
            "mass_scales": [float(x) for x in body_scales],
            "damping_scales": [float(x) for x in damping_scales],
            "action_delay_steps": action_delay_steps,
            "cam_pos_offsets_m": [[float(v) for v in row] for row in cam_pos_offsets],
            "cam_rot_offsets_deg": [[float(v) for v in row] for row in cam_rot_offsets_deg],
            "light_intensity_scales": [float(x) for x in light_scales],
            "image_brightness_delta": brightness_delta,
            "image_contrast_scale": contrast_scale,
            "obs_pos_noise_std": cfg.obs_pos_noise_std,
            "obs_vel_noise_std": cfg.obs_vel_noise_std,
            "image_noise_std": cfg.image_noise_std,
        }

    def apply_obs_noise(
        self,
        obs: dict[str, Any],
        rng: np.random.Generator,
    ) -> dict[str, Any]:
        """Return a new observation dict with Gaussian noise added to state and velocity."""
        out = dict(obs)
        if not self.config.enabled:
            return out
        if "state" in obs and obs["state"] is not None:
            state = np.asarray(obs["state"], dtype=np.float32).copy()
            if self.config.obs_pos_noise_std > 0.0:
                noise = rng.normal(
                    0.0, self.config.obs_pos_noise_std, size=state.shape
                ).astype(np.float32)
                state = state + noise
            out["state"] = state
        if "velocity" in obs and obs["velocity"] is not None:
            vel = np.asarray(obs["velocity"], dtype=np.float32).copy()
            if self.config.obs_vel_noise_std > 0.0:
                noise = rng.normal(
                    0.0, self.config.obs_vel_noise_std, size=vel.shape
                ).astype(np.float32)
                vel = vel + noise
            out["velocity"] = vel
        return out

    def apply_image_noise(
        self,
        img: np.ndarray,
        rng: np.random.Generator,
        *,
        brightness_delta: float | None = None,
        contrast_scale: float | None = None,
    ) -> np.ndarray:
        """Apply brightness/contrast perturbation and pixel noise without mutating ``img``."""
        arr = np.asarray(img, dtype=np.float32).copy()
        if not np.all(np.isfinite(arr)):
            arr = np.nan_to_num(arr, nan=0.0, posinf=255.0, neginf=0.0)
        if not self.config.enabled:
            np.clip(arr, 0.0, 255.0, out=arr)
            return np.rint(arr).astype(np.uint8)

        cfg = self.config
        if contrast_scale is None:
            if self._episode_contrast_scale is not None:
                contrast_scale = self._episode_contrast_scale
            else:
                contrast_scale = float(rng.uniform(*cfg.image_contrast_scale_range))
        if brightness_delta is None:
            if self._episode_brightness_delta is not None:
                brightness_delta = self._episode_brightness_delta
            else:
                brightness_delta = float(
                    rng.uniform(-cfg.image_brightness_delta, cfg.image_brightness_delta)
                )

        if contrast_scale != 1.0:
            arr = (arr - 127.5) * float(contrast_scale) + 127.5
        if brightness_delta != 0.0:
            arr = arr + float(brightness_delta)
        if cfg.image_noise_std > 0.0:
            arr = arr + rng.normal(0.0, cfg.image_noise_std, size=arr.shape).astype(np.float32)

        np.clip(arr, 0.0, 255.0, out=arr)
        return np.rint(arr).astype(np.uint8)
