"""Physics and visual domain randomization for ``SimEnv``."""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

MAX_ACTION_DELAY_STEPS = 1000
MAX_OBS_NOISE_STD = 1e6
MAX_SCALE_FACTOR = 1e6
_DR_SEED_SALT = 0x44525F50485953  # "DR_PHYS"


def _is_bool(val: Any) -> bool:
    return isinstance(val, (bool, np.bool_))


def _is_real_scalar(val: Any) -> bool:
    return isinstance(val, (int, float, np.integer, np.floating)) and not _is_bool(val)


def _is_int_scalar(val: Any) -> bool:
    return isinstance(val, (int, np.integer)) and not _is_bool(val)


def validate_seed(seed: int | None) -> int | None:
    """Validate and normalize an RNG seed to a Python non-negative ``int`` or ``None``."""
    if seed is None:
        return None
    if _is_bool(seed) or not isinstance(seed, (int, np.integer)):
        raise ValueError(f"seed must be a non-negative int or None, got {type(seed).__name__}")
    val = int(seed)
    if val < 0:
        raise ValueError(f"seed must be >= 0, got {val}")
    return val


def _validate_float_range(
    name: str,
    val: Any,
    *,
    min_val: float = 0.0,
    max_val: float | None = None,
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
        raise ValueError(
            f"{name} elements must be real numbers, got ({type(raw_lo).__name__}, {type(raw_hi).__name__})"
        )
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
    if max_val is not None and hi > max_val:
        raise ValueError(f"{name}: max ({hi}) must be <= {max_val}")
    return (lo, hi)


def _validate_int_range(
    name: str,
    val: Any,
    *,
    min_val: int = 0,
    max_val: int = MAX_ACTION_DELAY_STEPS,
) -> tuple[int, int]:
    if isinstance(val, (str, bytes, bytearray)) or not isinstance(val, (Sequence, np.ndarray)):
        raise ValueError(f"{name} must be a 2-element sequence of ints, got {type(val).__name__}")
    if isinstance(val, np.ndarray) and val.ndim != 1:
        raise ValueError(f"{name} must be 1D with length 2, got shape {val.shape}")
    if len(val) != 2:
        raise ValueError(f"{name} must have exactly 2 elements (min, max), got {len(val)}")
    raw_lo, raw_hi = val[0], val[1]
    if not _is_int_scalar(raw_lo) or not _is_int_scalar(raw_hi):
        raise ValueError(
            f"{name} elements must be integers, got ({type(raw_lo).__name__}, {type(raw_hi).__name__})"
        )
    lo, hi = int(raw_lo), int(raw_hi)
    if lo < min_val:
        raise ValueError(f"{name}: min ({lo}) must be >= {min_val}")
    if lo > hi:
        raise ValueError(f"{name}: min ({lo}) must be <= max ({hi})")
    if hi > max_val:
        raise ValueError(f"{name}: max ({hi}) must be <= {max_val}")
    return (lo, hi)


def _validate_non_negative_float(
    name: str,
    val: Any,
    *,
    max_val: float | None = None,
) -> float:
    if not _is_real_scalar(val):
        raise ValueError(f"{name} must be a real number, got {type(val).__name__}")
    out = float(val)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {out}")
    if out < 0.0:
        raise ValueError(f"{name} must be >= 0.0, got {out}")
    if max_val is not None and out > max_val:
        raise ValueError(f"{name} must be <= {max_val}, got {out}")
    return out


@dataclass(frozen=True)
class DomainRandomizationConfig:
    """Immutable configuration for physics and visual domain randomization in ``SimEnv``.

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
        object.__setattr__(
            self,
            "friction_scale_range",
            _validate_float_range(
                "friction_scale_range",
                self.friction_scale_range,
                min_val=0.0,
                max_val=MAX_SCALE_FACTOR,
            ),
        )
        object.__setattr__(
            self,
            "mass_scale_range",
            _validate_float_range(
                "mass_scale_range",
                self.mass_scale_range,
                min_val=0.0,
                max_val=MAX_SCALE_FACTOR,
                strict_positive=True,
            ),
        )
        object.__setattr__(
            self,
            "damping_scale_range",
            _validate_float_range(
                "damping_scale_range",
                self.damping_scale_range,
                min_val=0.0,
                max_val=MAX_SCALE_FACTOR,
            ),
        )
        object.__setattr__(
            self,
            "action_delay_steps_range",
            _validate_int_range(
                "action_delay_steps_range",
                self.action_delay_steps_range,
                min_val=0,
                max_val=MAX_ACTION_DELAY_STEPS,
            ),
        )
        object.__setattr__(
            self,
            "obs_pos_noise_std",
            _validate_non_negative_float(
                "obs_pos_noise_std", self.obs_pos_noise_std, max_val=MAX_OBS_NOISE_STD
            ),
        )
        object.__setattr__(
            self,
            "obs_vel_noise_std",
            _validate_non_negative_float(
                "obs_vel_noise_std", self.obs_vel_noise_std, max_val=MAX_OBS_NOISE_STD
            ),
        )
        object.__setattr__(
            self,
            "cam_pos_jitter_m",
            _validate_non_negative_float("cam_pos_jitter_m", self.cam_pos_jitter_m),
        )
        object.__setattr__(
            self,
            "cam_rot_jitter_deg",
            _validate_non_negative_float("cam_rot_jitter_deg", self.cam_rot_jitter_deg),
        )
        object.__setattr__(
            self,
            "light_intensity_scale_range",
            _validate_float_range(
                "light_intensity_scale_range",
                self.light_intensity_scale_range,
                min_val=0.0,
                max_val=MAX_SCALE_FACTOR,
            ),
        )
        object.__setattr__(
            self,
            "image_brightness_delta",
            _validate_non_negative_float("image_brightness_delta", self.image_brightness_delta),
        )
        object.__setattr__(
            self,
            "image_contrast_scale_range",
            _validate_float_range(
                "image_contrast_scale_range",
                self.image_contrast_scale_range,
                min_val=0.0,
                max_val=MAX_SCALE_FACTOR,
            ),
        )
        object.__setattr__(
            self,
            "image_noise_std",
            _validate_non_negative_float("image_noise_std", self.image_noise_std),
        )


class ActionDelayBuffer:
    """FIFO action delay buffer supporting ``delay_steps`` steps and episode reset."""

    def __init__(self, delay_steps: int = 0) -> None:
        self._delay_steps = self._validate_delay(delay_steps)
        self._queue: deque[np.ndarray] = deque()
        self._action_shape: tuple[int, ...] | None = None

    @staticmethod
    def _validate_delay(delay_steps: int) -> int:
        if not _is_int_scalar(delay_steps):
            raise ValueError(f"delay_steps must be an int, got {type(delay_steps).__name__}")
        val = int(delay_steps)
        if val < 0 or val > MAX_ACTION_DELAY_STEPS:
            raise ValueError(
                f"delay_steps must be in [0, {MAX_ACTION_DELAY_STEPS}], got {val}"
            )
        return val

    @property
    def delay_steps(self) -> int:
        """Configured delay in control steps."""
        return self._delay_steps

    def __len__(self) -> int:
        return len(self._queue)

    def reset(self, delay_steps: int | None = None) -> None:
        """Validate ``delay_steps`` (if given), then clear queued actions."""
        validated_delay = self._validate_delay(delay_steps) if delay_steps is not None else None
        self._queue.clear()
        self._action_shape = None
        if validated_delay is not None:
            self._delay_steps = validated_delay

    def push(
        self,
        action: Sequence[float] | np.ndarray,
        default_action: Sequence[float] | np.ndarray | None = None,
    ) -> np.ndarray:
        """Validate inputs first, then enqueue ``action`` and return the delayed action."""
        arr = np.asarray(action, dtype=np.float64).copy()
        if not np.all(np.isfinite(arr)):
            raise ValueError("action must contain only finite values")
        if self._action_shape is not None and arr.shape != self._action_shape:
            raise ValueError(
                f"action shape {arr.shape} does not match expected shape {self._action_shape}"
            )

        fallback: np.ndarray | None = None
        if default_action is not None:
            fallback = np.asarray(default_action, dtype=np.float64).copy()
            if fallback.shape != arr.shape:
                raise ValueError(
                    f"default_action shape {fallback.shape} does not match action shape {arr.shape}"
                )
            if not np.all(np.isfinite(fallback)):
                raise ValueError("default_action must contain only finite values")

        self._action_shape = arr.shape
        if self._delay_steps == 0:
            return arr
        self._queue.append(arr)
        if len(self._queue) <= self._delay_steps:
            return fallback if fallback is not None else np.zeros_like(arr)
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
        self._scratch_model: mujoco.MjModel | None = None
        self._scratch_data: mujoco.MjData | None = None
        self._episode_brightness_delta: float | None = None
        self._episode_contrast_scale: float | None = None

    def _get_scratch_data(self, model: mujoco.MjModel) -> mujoco.MjData:
        """Return a private scratch ``MjData`` so ``mj_setConst`` never mutates caller ``MjData``."""
        if self._scratch_data is None or self._scratch_model is not model:
            self._scratch_model = model
            self._scratch_data = mujoco.MjData(model)
        return self._scratch_data

    def restore_baseline(self, model: mujoco.MjModel) -> None:
        """Restore all snapshotted ``MjModel`` parameters back to their exact baseline."""
        model.body_mass[:] = self._baseline["body_mass"]
        model.body_inertia[:] = self._baseline["body_inertia"]
        model.geom_friction[:] = self._baseline["geom_friction"]
        model.dof_damping[:] = self._baseline["dof_damping"]
        model.cam_pos[:] = self._baseline["cam_pos"]
        model.cam_quat[:] = self._baseline["cam_quat"]
        model.light_diffuse[:] = self._baseline["light_diffuse"]
        model.light_ambient[:] = self._baseline["light_ambient"]
        mujoco.mj_setConst(model, self._get_scratch_data(model))
        self._episode_brightness_delta = None
        self._episode_contrast_scale = None

    def apply(
        self,
        model: mujoco.MjModel,
        seed: int | None = None,
    ) -> dict[str, Any]:
        """Validate ``seed``, sample all DR parameters, then atomically restore + apply to ``model``."""
        validated_seed = validate_seed(seed)
        cfg = self.config
        if not cfg.enabled:
            self.restore_baseline(model)
            self._episode_brightness_delta = 0.0
            self._episode_contrast_scale = 1.0
            return {
                "enabled": False,
                "seed": validated_seed,
                "action_delay_steps": 0,
            }

        rng = (
            np.random.default_rng(np.random.SeedSequence([validated_seed, _DR_SEED_SALT]))
            if validated_seed is not None
            else np.random.default_rng()
        )

        # Sample and stage all randomized arrays before touching `model`.
        with np.errstate(over="ignore", invalid="ignore"):
            if model.ngeom > 0:
                friction_scales = rng.uniform(*cfg.friction_scale_range, size=model.ngeom)
                geom_friction_new = self._baseline["geom_friction"] * friction_scales[:, None]
            else:
                friction_scales = np.empty(0, dtype=np.float64)
                geom_friction_new = None

            if model.nbody > 1:
                body_scales = rng.uniform(*cfg.mass_scale_range, size=model.nbody - 1)
                body_mass_new = self._baseline["body_mass"][1:] * body_scales
                body_inertia_new = self._baseline["body_inertia"][1:] * body_scales[:, None]
            else:
                body_scales = np.empty(0, dtype=np.float64)
                body_mass_new = None
                body_inertia_new = None

            if model.nv > 0:
                damping_scales = rng.uniform(*cfg.damping_scale_range, size=model.nv)
                dof_damping_new = self._baseline["dof_damping"] * damping_scales
            else:
                damping_scales = np.empty(0, dtype=np.float64)
                dof_damping_new = None

            delay_lo, delay_hi = cfg.action_delay_steps_range
            action_delay_steps = int(rng.integers(delay_lo, delay_hi + 1))

            if model.ncam > 0:
                cam_pos_offsets = rng.uniform(
                    -cfg.cam_pos_jitter_m, cfg.cam_pos_jitter_m, size=(model.ncam, 3)
                )
                cam_pos_new = self._baseline["cam_pos"] + cam_pos_offsets

                cam_rot_offsets_deg = rng.uniform(
                    -cfg.cam_rot_jitter_deg, cfg.cam_rot_jitter_deg, size=(model.ncam, 3)
                )
                cam_quat_new = self._baseline["cam_quat"].copy()
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
                        cam_quat_new[i] = q_out
            else:
                cam_pos_offsets = np.empty((0, 3), dtype=np.float64)
                cam_rot_offsets_deg = np.empty((0, 3), dtype=np.float64)
                cam_pos_new = None
                cam_quat_new = None

            if model.nlight > 0:
                light_scales = rng.uniform(*cfg.light_intensity_scale_range, size=model.nlight)
                light_scales_f32 = light_scales[:, None].astype(np.float32)
                light_diffuse_new = np.clip(
                    self._baseline["light_diffuse"] * light_scales_f32, 0.0, None
                )
                light_ambient_new = np.clip(
                    self._baseline["light_ambient"] * light_scales_f32, 0.0, None
                )
            else:
                light_scales = np.empty(0, dtype=np.float64)
                light_diffuse_new = None
                light_ambient_new = None

            brightness_delta = float(
                rng.uniform(-cfg.image_brightness_delta, cfg.image_brightness_delta)
            )
            contrast_scale = float(rng.uniform(*cfg.image_contrast_scale_range))

        # Verify all staged arrays are finite (and positive-mass bodies stay > 0) before mutating model.
        for label, staged in (
            ("geom_friction", geom_friction_new),
            ("body_mass", body_mass_new),
            ("body_inertia", body_inertia_new),
            ("dof_damping", dof_damping_new),
            ("cam_pos", cam_pos_new),
            ("cam_quat", cam_quat_new),
            ("light_diffuse", light_diffuse_new),
            ("light_ambient", light_ambient_new),
        ):
            if staged is not None and not np.all(np.isfinite(staged)):
                raise ValueError(f"DomainRandomizer produced non-finite {label} values")
        if body_mass_new is not None:
            pos_mask = self._baseline["body_mass"][1:] > 0.0
            if np.any(body_mass_new[pos_mask] <= 0.0) or np.any(body_mass_new < 0.0):
                raise ValueError("DomainRandomizer produced non-positive body_mass")

        # Commit all staged baseline + randomized values atomically.
        model.body_mass[:] = self._baseline["body_mass"]
        model.body_inertia[:] = self._baseline["body_inertia"]
        model.geom_friction[:] = self._baseline["geom_friction"]
        model.dof_damping[:] = self._baseline["dof_damping"]
        model.cam_pos[:] = self._baseline["cam_pos"]
        model.cam_quat[:] = self._baseline["cam_quat"]
        model.light_diffuse[:] = self._baseline["light_diffuse"]
        model.light_ambient[:] = self._baseline["light_ambient"]

        if geom_friction_new is not None:
            model.geom_friction[:] = geom_friction_new
        if body_mass_new is not None and body_inertia_new is not None:
            model.body_mass[1:] = body_mass_new
            model.body_inertia[1:] = body_inertia_new
        if dof_damping_new is not None:
            model.dof_damping[:] = dof_damping_new
        if cam_pos_new is not None and cam_quat_new is not None:
            model.cam_pos[:] = cam_pos_new
            model.cam_quat[:] = cam_quat_new
        if light_diffuse_new is not None and light_ambient_new is not None:
            model.light_diffuse[:] = light_diffuse_new
            model.light_ambient[:] = light_ambient_new

        mujoco.mj_setConst(model, self._get_scratch_data(model))
        if not np.all(np.isfinite(model.body_subtreemass)):
            self.restore_baseline(model)
            raise ValueError("mj_setConst produced non-finite body_subtreemass")
        self._episode_brightness_delta = brightness_delta
        self._episode_contrast_scale = contrast_scale

        return {
            "enabled": True,
            "seed": validated_seed,
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

    @staticmethod
    def _perturb_float32(
        values: Any,
        std: float,
        rng: np.random.Generator,
        label: str,
    ) -> np.ndarray:
        with np.errstate(over="ignore", invalid="ignore"):
            arr = np.asarray(values, dtype=np.float32).copy()
            if std > 0.0:
                noise = rng.normal(0.0, std, size=arr.shape).astype(np.float32)
                arr = arr + noise
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"apply_obs_noise produced non-finite float32 {label} values")
        return arr

    def apply_obs_noise(
        self,
        obs: dict[str, Any],
        rng: np.random.Generator,
    ) -> dict[str, Any]:
        """Return a new observation dict with finite ``float32`` Gaussian noise on state/velocity."""
        out = dict(obs)
        if not self.config.enabled:
            return out
        if "state" in obs and obs["state"] is not None:
            out["state"] = self._perturb_float32(
                obs["state"], self.config.obs_pos_noise_std, rng, "state"
            )
        if "velocity" in obs and obs["velocity"] is not None:
            out["velocity"] = self._perturb_float32(
                obs["velocity"], self.config.obs_vel_noise_std, rng, "velocity"
            )
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
