"""Wrist camera stream and perception API for rebot-sim.

- Video: `ws://<host>:<video-port>/` (default 8001) as the reCamera debug stream: one binary
  message per H.264 access unit (Annex-B, SPS + PPS before every IDR) followed by the capture
  time as 8-byte little-endian unix ms. A new client starts at a forced IDR. `/scene` serves
  the fixed overview camera when it is enabled.
- HTTP `:<perception-port>` (default 8090): rebot-vision perception API version 2
  (rebot-armd docs/detection_api.md) with detections computed from the simulator's ground
  truth, plus `POST /sim/reset`, `GET /sim/state`, `GET /sim/camera`. With a depth model
  (`--depth metric|relative`, camera.DepthModel) the trigger also accepts the `depth` pipeline
  (trigger-only, as rebot-vision on demand): detections then carry disp_fg / disp_bg / depth_conf
  / length_px / jaw_clear and the body a `depth` object with table_samples, computed from the
  rendered depth and the ground-truth masks (camera.depth_result). Box noise does not change
  the depth fields; detections without a mask (false detections) get none.
- `POST /api/models` as rebot-vision (202 + generation, SWAP_BUSY, ION_BUDGET from the catalogue
  ion_mb): catalogue `seg` (kind segment), `det` (kind detect: no angle_deg / angle_conf /
  grasp_width_px, as --no-mask-angle) and, with a depth model, `depth`. The swap is applied on the
  next frame. The startup set (--no-mask-angle, --depth) is not budget-checked.
"""

from __future__ import annotations

import importlib
import itertools
import json
import math
import os
import queue
import sys
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import numpy as np

API_VERSION = 2
PIPELINES = ("yolo", "aruco")
MODES = ("off", "yolo", "aruco", "both")
DETECTOR_MODES = ("gt", "onnx", "callable")
MAX_SUBSCRIPTIONS = 16
IDLE_FPS = 1
MODE_PIPELINES = {
    "off": [],
    "yolo": ["yolo"],
    "aruco": ["aruco"],
    "both": ["yolo", "aruco"],
}
# model catalogue (kind, ion_mb) and ION budget as rebot-vision's defaults: seg + depth does not fit
MODELS = {"seg": ("segment", 19.5), "det": ("detect", 8.5), "depth": ("depth", 20.5)}
ION = {"base_mb": 21.5, "total_mb": 62.9, "reserve_mb": 8.0}
MASK_FIELDS = ("angle_deg", "angle_conf", "grasp_width_px")
_SIM_DR_SALT = 0x5245424F545F4452  # "REBOT_DR"


def _is_bool(val: Any) -> bool:
    return isinstance(val, (bool, np.bool_))


def _is_real_scalar(val: Any) -> bool:
    return isinstance(val, (int, float, np.integer, np.floating)) and not _is_bool(val)


def _validate_dr_range(name: str, val: Any, *, min_val: float = 0.0, max_val: float | None = None) -> tuple[float, float]:
    if isinstance(val, (str, bytes, bytearray)) or not isinstance(val, (Sequence, np.ndarray)):
        raise ValueError(f"{name} must be a 2-element sequence of numbers")
    if len(val) != 2 or not _is_real_scalar(val[0]) or not _is_real_scalar(val[1]):
        raise ValueError(f"{name} must contain 2 finite real numbers")
    lo, hi = float(val[0]), float(val[1])
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError(f"{name} bounds must be finite")
    if lo < min_val or lo > hi or (max_val is not None and hi > max_val):
        raise ValueError(f"{name} bounds out of range: ({lo}, {hi})")
    return (lo, hi)


def _validate_dr_float(name: str, val: Any, *, min_val: float = 0.0, max_val: float | None = None) -> float:
    if not _is_real_scalar(val):
        raise ValueError(f"{name} must be a finite real number, got {type(val).__name__}")
    f = float(val)
    if not math.isfinite(f) or f < min_val or (max_val is not None and f > max_val):
        raise ValueError(f"{name} out of range: {f}")
    return f


_DR_RANGE_SPECS: tuple[tuple[str, float, float], ...] = (
    ("light_diffuse_range", 0.0, 10.0),
    ("light_specular_range", 0.0, 10.0),
    ("light_ambient_range", 0.0, 10.0),
)
_DR_FLOAT_SPECS: tuple[tuple[str, float, float], ...] = (
    ("light_ambient_delta", 0.0, 1.0),
    ("light_pos_jitter_m", 0.0, 5.0),
    ("camera_jitter_m", 0.0, 1.0),
    ("camera_jitter_deg", 0.0, 180.0),
    ("latency_ms", 0.0, 5000.0),
)


@dataclass(frozen=True)
class DomainRandomizationConfig:
    """Deterministic, seed-driven Domain Randomization config for rebot-sim (disabled by default)."""

    enabled: bool = False
    lighting: bool = False
    light_diffuse_range: tuple[float, float] = (0.6, 1.4)
    light_specular_range: tuple[float, float] = (0.5, 1.5)
    light_ambient_range: tuple[float, float] = (0.5, 1.5)
    light_ambient_delta: float = 0.12
    light_pos_jitter_m: float = 0.15
    camera_jitter_m: float = 0.0
    camera_jitter_deg: float = 0.0
    latency_ms: float = 0.0
    latency_ms_range: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        for fname in ("enabled", "lighting"):
            val = getattr(self, fname)
            if not isinstance(val, bool):
                raise ValueError(f"{fname} must be a bool, got {type(val).__name__}")
        for fname, min_val, max_val in _DR_RANGE_SPECS:
            object.__setattr__(
                self, fname, _validate_dr_range(fname, getattr(self, fname), min_val=min_val, max_val=max_val)
            )
        for fname, min_val, max_val in _DR_FLOAT_SPECS:
            object.__setattr__(
                self, fname, _validate_dr_float(fname, getattr(self, fname), min_val=min_val, max_val=max_val)
            )
        if self.latency_ms_range is not None:
            object.__setattr__(
                self, "latency_ms_range",
                _validate_dr_range("latency_ms_range", self.latency_ms_range, min_val=0.0, max_val=5000.0),
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "lighting": self.lighting,
            "light_diffuse_range": list(self.light_diffuse_range),
            "light_specular_range": list(self.light_specular_range),
            "light_ambient_range": list(self.light_ambient_range),
            "light_ambient_delta": self.light_ambient_delta,
            "light_pos_jitter_m": self.light_pos_jitter_m,
            "camera_jitter_m": self.camera_jitter_m,
            "camera_jitter_deg": self.camera_jitter_deg,
            "latency_ms": self.latency_ms,
            "latency_ms_range": list(self.latency_ms_range) if self.latency_ms_range is not None else None,
        }

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> "DomainRandomizationConfig":
        if raw is None:
            return cls(enabled=False)
        if not isinstance(raw, dict):
            raise ValueError("dr config must be a JSON object or null")
        known = {
            "enabled", "lighting", "light_diffuse_range", "light_specular_range",
            "light_ambient_range", "light_ambient_delta", "light_pos_jitter_m",
            "camera_jitter_m", "cam_pos_jitter_m", "camera_jitter_deg", "cam_rot_jitter_deg",
            "latency_ms", "latency_ms_range",
        }
        unknown = set(raw) - known
        if unknown:
            raise ValueError(f"unknown DR config keys: {sorted(unknown)}")
        kwargs: dict[str, Any] = {}
        for k in (
            "lighting", "light_diffuse_range", "light_specular_range",
            "light_ambient_range", "light_ambient_delta", "light_pos_jitter_m",
            "latency_ms", "latency_ms_range",
        ):
            if k in raw:
                kwargs[k] = raw[k]
        if "camera_jitter_m" in raw:
            kwargs["camera_jitter_m"] = raw["camera_jitter_m"]
        elif "cam_pos_jitter_m" in raw:
            kwargs["camera_jitter_m"] = raw["cam_pos_jitter_m"]
        if "camera_jitter_deg" in raw:
            kwargs["camera_jitter_deg"] = raw["camera_jitter_deg"]
        elif "cam_rot_jitter_deg" in raw:
            kwargs["camera_jitter_deg"] = raw["cam_rot_jitter_deg"]
        if "enabled" in raw:
            kwargs["enabled"] = raw["enabled"]
        else:
            # Enable automatically when any active DR field is specified in a partial update.
            has_active = bool(
                kwargs.get("lighting")
                or float(kwargs.get("camera_jitter_m", 0.0) or 0.0) > 0.0
                or float(kwargs.get("camera_jitter_deg", 0.0) or 0.0) > 0.0
                or float(kwargs.get("latency_ms", 0.0) or 0.0) > 0.0
                or kwargs.get("latency_ms_range") is not None
            )
            kwargs["enabled"] = has_active
        return cls(**kwargs)


def coerce_dr_config(cfg: DomainRandomizationConfig | dict | Any | None) -> DomainRandomizationConfig:
    """Coerce None, dict, or domain_rand.DomainRandomizationConfig into a perception DomainRandomizationConfig."""
    if cfg is None:
        return DomainRandomizationConfig(enabled=False)
    if isinstance(cfg, DomainRandomizationConfig):
        return cfg
    if isinstance(cfg, dict):
        return DomainRandomizationConfig.from_dict(cfg)
    # Support domain_rand.DomainRandomizationConfig instances cleanly.
    if hasattr(cfg, "enabled") and hasattr(cfg, "cam_pos_jitter_m"):
        return DomainRandomizationConfig(
            enabled=bool(cfg.enabled),
            lighting=True,
            light_diffuse_range=tuple(getattr(cfg, "light_intensity_scale_range", (0.6, 1.4))),
            light_ambient_range=tuple(getattr(cfg, "light_intensity_scale_range", (0.5, 1.5))),
            camera_jitter_m=float(getattr(cfg, "cam_pos_jitter_m", 0.0)),
            camera_jitter_deg=float(getattr(cfg, "cam_rot_jitter_deg", 0.0)),
        )
    raise TypeError(f"dr_config must be DomainRandomizationConfig, dict, or None, got {type(cfg).__name__}")


def _unit_vec3(rng: np.random.Generator) -> np.ndarray:
    v = rng.normal(size=3)
    norm = float(np.linalg.norm(v))
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    return v / norm


class SimDomainRandomizer:
    """Snapshots MuJoCo lighting & camera extrinsics baseline and applies drift-free per-seed DR."""

    def __init__(self, model, config: DomainRandomizationConfig | None = None) -> None:
        self.config = coerce_dr_config(config)
        self._baseline: dict[str, np.ndarray] = {
            "light_diffuse": model.light_diffuse.copy(),
            "light_specular": model.light_specular.copy(),
            "light_ambient": model.light_ambient.copy(),
            "light_pos": model.light_pos.copy(),
            "cam_pos": model.cam_pos.copy(),
            "cam_quat": model.cam_quat.copy(),
        }
        self._dirty = False
        self.last_summary: dict[str, Any] | None = None

    def restore_baseline(self, model) -> None:
        """Restore exact nominal lighting & camera extrinsics back to baseline."""
        if self._dirty:
            model.light_diffuse[:] = self._baseline["light_diffuse"]
            model.light_specular[:] = self._baseline["light_specular"]
            model.light_ambient[:] = self._baseline["light_ambient"]
            model.light_pos[:] = self._baseline["light_pos"]
            model.cam_pos[:] = self._baseline["cam_pos"]
            model.cam_quat[:] = self._baseline["cam_quat"]
            self._dirty = False
        self.last_summary = None

    def apply(self, model, seed: int | None = None) -> dict[str, Any]:
        """Apply deterministic per-seed DR to `model` (or restore baseline if disabled)."""
        import mujoco

        if seed is not None:
            if _is_bool(seed) or not isinstance(seed, (int, np.integer)) or int(seed) < 0:
                raise ValueError(f"seed must be a non-negative int or None, got {seed!r}")
            seed = int(seed)
        cfg = self.config
        if not cfg.enabled:
            self.restore_baseline(model)
            return {"enabled": False, "seed": seed, "latency_ms": 0.0}

        rng = (
            np.random.default_rng(np.random.SeedSequence([seed, _SIM_DR_SALT]))
            if seed is not None
            else np.random.default_rng()
        )

        # Always start staged arrays from exact nominal baseline (zero drift across resets).
        light_diffuse_new = self._baseline["light_diffuse"].copy()
        light_specular_new = self._baseline["light_specular"].copy()
        light_ambient_new = self._baseline["light_ambient"].copy()
        light_pos_new = self._baseline["light_pos"].copy()
        cam_pos_new = self._baseline["cam_pos"].copy()
        cam_quat_new = self._baseline["cam_quat"].copy()

        diffuse_scales = np.ones_like(light_diffuse_new, dtype=np.float32)
        specular_scales = np.ones_like(light_specular_new, dtype=np.float32)
        light_pos_offsets = np.zeros_like(light_pos_new, dtype=np.float64)

        if cfg.lighting and model.nlight > 0:
            diffuse_scales = rng.uniform(
                *cfg.light_diffuse_range, size=(model.nlight, 3)
            ).astype(np.float32)
            specular_scales = rng.uniform(
                *cfg.light_specular_range, size=(model.nlight, 3)
            ).astype(np.float32)
            ambient_scales = rng.uniform(
                *cfg.light_ambient_range, size=(model.nlight, 3)
            ).astype(np.float32)
            ambient_deltas = rng.uniform(
                0.0, cfg.light_ambient_delta, size=(model.nlight, 3)
            ).astype(np.float32)
            light_diffuse_new = np.clip(light_diffuse_new * diffuse_scales, 0.05, 2.0)
            light_specular_new = np.clip(light_specular_new * specular_scales, 0.0, 2.0)
            light_ambient_new = np.clip(light_ambient_new * ambient_scales + ambient_deltas, 0.0, 0.5)
            if cfg.light_pos_jitter_m > 0.0:
                light_pos_offsets = rng.uniform(
                    -cfg.light_pos_jitter_m, cfg.light_pos_jitter_m, size=(model.nlight, 3)
                )
                light_pos_new = light_pos_new + light_pos_offsets
                light_pos_new[:, 2] = np.maximum(light_pos_new[:, 2], 0.01)
                light_pos_offsets = light_pos_new - self._baseline["light_pos"]

        cam_pos_offsets = np.zeros((model.ncam, 3), dtype=np.float64)
        cam_rot_angles_deg: list[float] = [0.0] * model.ncam
        if model.ncam > 0:
            q_delta = np.empty(4, dtype=np.float64)
            q_out = np.empty(4, dtype=np.float64)
            for i in range(model.ncam):
                if cfg.camera_jitter_m > 0.0:
                    u = _unit_vec3(rng)
                    r = float(rng.uniform(0.0, cfg.camera_jitter_m))
                    cam_pos_offsets[i] = u * r
                    cam_pos_new[i] = self._baseline["cam_pos"][i] + cam_pos_offsets[i]
                if cfg.camera_jitter_deg > 0.0:
                    axis = _unit_vec3(rng)
                    angle_deg = float(rng.uniform(0.0, cfg.camera_jitter_deg))
                    angle_rad = math.radians(angle_deg)
                    if angle_rad > 1e-12:
                        mujoco.mju_axisAngle2Quat(q_delta, axis, angle_rad)
                        mujoco.mju_mulQuat(q_out, self._baseline["cam_quat"][i], q_delta)
                        norm = float(np.linalg.norm(q_out))
                        if norm > 0.0:
                            q_out /= norm
                        cam_quat_new[i] = q_out
                    cam_rot_angles_deg[i] = round(angle_deg, 6)

        if cfg.latency_ms_range is not None:
            latency_ms = float(rng.uniform(*cfg.latency_ms_range))
        else:
            latency_ms = float(cfg.latency_ms)

        # Atomic commit to model
        model.light_diffuse[:] = light_diffuse_new
        model.light_specular[:] = light_specular_new
        model.light_ambient[:] = light_ambient_new
        model.light_pos[:] = light_pos_new
        model.cam_pos[:] = cam_pos_new
        model.cam_quat[:] = cam_quat_new
        self._dirty = True

        summary: dict[str, Any] = {
            "enabled": True,
            "seed": seed,
            "config": cfg.to_dict(),
            "lighting": cfg.lighting,
            "light_diffuse_scales": [[round(float(v), 6) for v in row] for row in diffuse_scales] if cfg.lighting else [],
            "light_specular_scales": [[round(float(v), 6) for v in row] for row in specular_scales] if cfg.lighting else [],
            "light_ambient_values": [[round(float(v), 6) for v in row] for row in light_ambient_new] if cfg.lighting else [],
            "light_pos_offsets_m": [[round(float(v), 6) for v in row] for row in light_pos_offsets] if cfg.lighting else [],
            "cam_pos_offsets_m": [[round(float(v), 6) for v in row] for row in cam_pos_offsets],
            "cam_rot_angles_deg": cam_rot_angles_deg,
            "latency_ms": round(latency_ms, 3),
        }
        self.last_summary = summary
        return summary


def normalize_detector_mode(mode: str | None) -> str:
    if mode is None:
        return "gt"
    m = str(mode).strip().lower()
    if m not in DETECTOR_MODES:
        raise ValueError(f"detector_mode must be one of {DETECTOR_MODES}, got {mode!r}")
    return m


def load_callable(spec: str) -> Callable[[np.ndarray], list[dict]]:
    """Load a callable from `'module.path:attr'`."""
    if not isinstance(spec, str) or ":" not in spec:
        raise ValueError(f"detector callable spec must be 'module:func', got {spec!r}")
    mod_name, attr = spec.split(":", 1)
    mod = importlib.import_module(mod_name.strip())
    fn = getattr(mod, attr.strip())
    if not callable(fn):
        raise TypeError(f"{spec!r} is not callable")
    return fn


def normalize_detection(
    det: dict[str, Any],
    index: int = 0,
    labels: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Normalize a detector output dict into rebot-vision `xywh_center` schema."""
    if not isinstance(det, dict):
        raise ValueError(f"detection must be a dict, got {type(det).__name__}")
    out = dict(det)
    cid = int(out.get("class_id", index))
    cname = out.get("class_name") or out.get("label")
    if not cname:
        if labels and 0 <= cid < len(labels):
            cname = str(labels[cid])
        else:
            cname = out.get("name") or f"class_{cid}"
    name = out.get("name") or cname
    conf = float(out.get("confidence", out.get("score", 1.0)))

    if all(k in out for k in ("x", "y", "w", "h")):
        cx, cy, bw, bh = float(out["x"]), float(out["y"]), float(out["w"]), float(out["h"])
    elif "box_norm" in out and len(out["box_norm"]) == 4:
        x0, y0, x1, y1 = (float(v) for v in out["box_norm"])
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        bw, bh = max(0.0, x1 - x0), max(0.0, y1 - y0)
    else:
        raise ValueError("detection must include x,y,w,h or box_norm")

    out["class_id"] = cid
    out["format"] = out.get("format", "xywh_center")
    out["class_name"] = str(cname)
    out["name"] = str(name)
    out["confidence"] = round(conf, 4)
    out["x"] = round(cx, 4)
    out["y"] = round(cy, 4)
    out["w"] = round(bw, 4)
    out["h"] = round(bh, 4)
    return out


def _nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    if len(boxes) == 0:
        return []
    order = np.argsort(-scores)
    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx0 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy0 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx1 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy1 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0.0, xx1 - xx0) * np.maximum(0.0, yy1 - yy0)
        area_i = max(0.0, boxes[i, 2] - boxes[i, 0]) * max(0.0, boxes[i, 3] - boxes[i, 1])
        area_r = np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) * np.maximum(0.0, boxes[rest, 3] - boxes[rest, 1])
        union = area_i + area_r - inter
        iou = np.where(union > 0.0, inter / union, 0.0)
        order = rest[iou <= iou_threshold]
    return keep


class CallableDetector:
    """Adapter around an injectable Python callable `fn(rgb) -> list[dict]`."""

    def __init__(self, fn: Callable[[np.ndarray], Sequence[dict]], labels: Sequence[str] | None = None):
        if not callable(fn):
            raise TypeError("detector callable must be callable")
        self.fn = fn
        self.labels = list(labels or [])

    def detect(self, rgb: np.ndarray) -> list[dict]:
        raw = self.fn(rgb)
        return [normalize_detection(d, index=i, labels=self.labels) for i, d in enumerate(raw or ())]

    def __call__(self, rgb: np.ndarray) -> list[dict]:
        return self.detect(rgb)


class OnnxDetector:
    """ONNXRuntime object detector running on rendered RGB uint8 frames (YOLO-compatible)."""

    OUTPUT_FORMATS = ("auto", "nms_xyxy", "yolov8", "yolov5")

    def __init__(
        self,
        model_path: str | Path,
        labels: Sequence[str] | None = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        input_size: tuple[int, int] | None = None,
        output_format: str = "auto",
    ) -> None:
        self.model_path = str(model_path)
        self.labels = list(labels or [])
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        fmt = str(output_format).strip().lower()
        if fmt not in self.OUTPUT_FORMATS:
            raise ValueError(f"output_format must be one of {self.OUTPUT_FORMATS}, got {output_format!r}")
        self.output_format = fmt
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "onnxruntime is required for detector_mode='onnx'; install onnxruntime or use detector_mode='callable'/'gt'"
            ) from exc
        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"ONNX detector model not found: {self.model_path}")
        providers = ["CPUExecutionProvider"]
        avail = set(ort.get_available_providers())
        use_providers = [p for p in providers if p in avail] or list(avail)
        self.session = ort.InferenceSession(self.model_path, providers=use_providers)
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        if input_size is not None:
            self.input_hw = (int(input_size[0]), int(input_size[1]))
        elif (
            isinstance(inp.shape, (list, tuple))
            and len(inp.shape) == 4
            and isinstance(inp.shape[2], int)
            and isinstance(inp.shape[3], int)
        ):
            self.input_hw = (int(inp.shape[2]), int(inp.shape[3]))
        else:
            self.input_hw = (640, 640)

    def preprocess(self, rgb: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        h_src, w_src = rgb.shape[:2]
        h_in, w_in = self.input_hw
        scale = min(w_in / w_src, h_in / h_src)
        new_w, new_h = max(1, int(round(w_src * scale))), max(1, int(round(h_src * scale)))
        pad_x = (w_in - new_w) / 2.0
        pad_y = (h_in - new_h) / 2.0

        try:
            from PIL import Image

            resized = np.asarray(
                Image.fromarray(rgb, mode="RGB").resize((new_w, new_h), Image.BILINEAR),
                dtype=np.uint8,
            )
        except ImportError:
            yi = np.minimum((np.arange(new_h) / scale).astype(int), h_src - 1)
            xi = np.minimum((np.arange(new_w) / scale).astype(int), w_src - 1)
            resized = rgb[np.ix_(yi, xi)]

        canvas = np.full((h_in, w_in, 3), 114, dtype=np.uint8)
        top, left = int(round(pad_y)), int(round(pad_x))
        canvas[top : top + new_h, left : left + new_w] = resized
        tensor = (canvas.astype(np.float32) / 255.0).transpose(2, 0, 1)[None, ...]
        return np.ascontiguousarray(tensor), scale, (float(left), float(top))

    def _resolve_output_format(self, raw: np.ndarray) -> tuple[str, np.ndarray]:
        fmt = self.output_format
        if fmt == "nms_xyxy":
            return "nms_xyxy", raw
        if fmt == "yolov5":
            return "yolov5", raw
        if fmt == "auto" and raw.shape[1] == 6 and not (self.labels and len(self.labels) == 1):
            is_int_cls = bool(np.all(np.abs(raw[:, 5] - np.round(raw[:, 5])) < 1e-4))
            is_xyxy = bool(np.all(raw[:, 2] >= raw[:, 0]) and np.all(raw[:, 3] >= raw[:, 1]))
            if raw.shape[0] <= 300 and is_int_cls and is_xyxy:
                return "nms_xyxy", raw
            if raw.shape[0] > 6 or not is_int_cls:
                return "yolov5", raw
            return "nms_xyxy", raw
        if fmt == "auto" and self.labels:
            if raw.shape[1] == 5 + len(self.labels):
                return "yolov5", raw
            if raw.shape[1] == 4 + len(self.labels):
                return "yolov8", raw
        preds = raw.T if (raw.shape[0] < raw.shape[1] and raw.shape[0] <= 256) else raw
        if fmt != "auto":
            return fmt, preds
        ncols = preds.shape[1]
        if self.labels:
            if ncols == 5 + len(self.labels):
                return "yolov5", preds
            if ncols == 4 + len(self.labels):
                return "yolov8", preds
        if raw.shape[0] < raw.shape[1] and raw.shape[0] <= 256:
            return "yolov8", preds
        if ncols >= 6 and np.all((preds[:, 4] >= 0.0) & (preds[:, 4] <= 1.0)) and np.all(
            (preds[:, 5:] >= 0.0) & (preds[:, 5:] <= 1.0)
        ):
            if raw.shape[0] > 300:
                return "yolov5", preds
        return "yolov8", preds

    def postprocess(
        self,
        outputs: Sequence[np.ndarray],
        src_hw: tuple[int, int],
        scale: float,
        pad: tuple[float, float],
    ) -> list[dict]:
        if not outputs:
            return []
        raw = np.asarray(outputs[0], dtype=np.float32)
        if raw.ndim == 3:
            raw = raw[0]
        if raw.ndim != 2 or raw.size == 0:
            return []

        h_src, w_src = src_hw
        pad_x, pad_y = pad
        boxes_xyxy: list[list[float]] = []
        scores_list: list[float] = []
        cls_list: list[int] = []

        fmt, preds = self._resolve_output_format(raw)
        if fmt == "nms_xyxy":
            if preds.shape[1] < 6:
                return []
            for row in preds:
                sc = float(row[4])
                if sc < self.conf_threshold:
                    continue
                x0 = (float(row[0]) - pad_x) / scale
                y0 = (float(row[1]) - pad_y) / scale
                x1 = (float(row[2]) - pad_x) / scale
                y1 = (float(row[3]) - pad_y) / scale
                boxes_xyxy.append([x0, y0, x1, y1])
                scores_list.append(sc)
                cls_list.append(int(round(float(row[5]))))
        else:
            ncols = preds.shape[1]
            if fmt == "yolov5":
                if ncols < 6:
                    return []
                cls_scores = preds[:, 4:5] * preds[:, 5:]
            else:
                if ncols < 5:
                    return []
                cls_scores = preds[:, 4:]
            best_cls = np.argmax(cls_scores, axis=1)
            best_score = cls_scores[np.arange(len(preds)), best_cls]
            mask = best_score >= self.conf_threshold
            preds = preds[mask]
            best_cls = best_cls[mask]
            best_score = best_score[mask]
            for row, sc, cid in zip(preds, best_score, best_cls):
                cx, cy, bw, bh = float(row[0]), float(row[1]), float(row[2]), float(row[3])
                x0 = (cx - bw / 2.0 - pad_x) / scale
                y0 = (cy - bh / 2.0 - pad_y) / scale
                x1 = (cx + bw / 2.0 - pad_x) / scale
                y1 = (cy + bh / 2.0 - pad_y) / scale
                boxes_xyxy.append([x0, y0, x1, y1])
                scores_list.append(float(sc))
                cls_list.append(int(cid))

        if not boxes_xyxy:
            return []
        boxes_arr = np.asarray(boxes_xyxy, dtype=np.float32)
        scores_arr = np.asarray(scores_list, dtype=np.float32)
        cls_arr = np.asarray(cls_list, dtype=np.int64)

        keep_all: list[int] = []
        for cid in np.unique(cls_arr):
            idxs = np.nonzero(cls_arr == cid)[0]
            kept_sub = _nms_xyxy(boxes_arr[idxs], scores_arr[idxs], self.iou_threshold)
            keep_all.extend(int(idxs[k]) for k in kept_sub)
        keep_all.sort(key=lambda idx: float(scores_arr[idx]), reverse=True)

        dets: list[dict] = []
        for idx in keep_all:
            x0 = float(np.clip(boxes_arr[idx, 0] / w_src, 0.0, 1.0))
            y0 = float(np.clip(boxes_arr[idx, 1] / h_src, 0.0, 1.0))
            x1 = float(np.clip(boxes_arr[idx, 2] / w_src, 0.0, 1.0))
            y1 = float(np.clip(boxes_arr[idx, 3] / h_src, 0.0, 1.0))
            if x1 <= x0 or y1 <= y0:
                continue
            cid = int(cls_arr[idx])
            dets.append(
                normalize_detection(
                    {
                        "class_id": cid,
                        "confidence": float(scores_arr[idx]),
                        "x": (x0 + x1) / 2.0,
                        "y": (y0 + y1) / 2.0,
                        "w": x1 - x0,
                        "h": y1 - y0,
                    },
                    index=cid,
                    labels=self.labels,
                )
            )
        return dets

    def detect(self, rgb: np.ndarray) -> list[dict]:
        tensor, scale, pad = self.preprocess(rgb)
        outputs = self.session.run(None, {self.input_name: tensor})
        return self.postprocess(outputs, rgb.shape[:2], scale, pad)

    def __call__(self, rgb: np.ndarray) -> list[dict]:
        return self.detect(rgb)


def depth_result_from_boxes(
    z: np.ndarray,
    dets: Sequence[dict],
    model,
    frame_id: int = 0,
    fx_per_w: float | None = None,
) -> dict:
    """Compute depth fields & table samples purely from rendered depth `z` and predicted bboxes (no GT leakage)."""
    from .camera import DEPTH_GRID, DETECTION_SIZE, _mad, _plane_fit, grid_cells

    w, h = DETECTION_SIZE
    gw, gh = DEPTH_GRID
    cw, ch = w / gw, h / gh
    u, v, ix, iy = grid_cells()
    z_g = z[np.ix_(iy, ix)].astype(np.float64)
    finite = bool(np.isfinite(z_g).all() and (z_g > 0).all())
    disp = model.disparity(z_g, frame_id, fx_per_w)
    uu, vv = np.meshgrid(u, v)
    valid = np.isfinite(disp)

    boxes: list[tuple[float, float, float, float]] = []
    any_box = np.zeros_like(valid)
    excl = np.zeros_like(valid)
    for d in dets:
        cx, cy, bw, bh = float(d["x"]) * w, float(d["y"]) * h, float(d["w"]) * w, float(d["h"]) * h
        x0, y0 = max(0.0, cx - bw / 2.0), max(0.0, cy - bh / 2.0)
        x1, y1 = min(float(w), cx + bw / 2.0), min(float(h), cy + bh / 2.0)
        boxes.append((x0, y0, x1, y1))
        any_box |= (uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)
        excl |= (uu >= x0 - 0.15 * bw) & (uu <= x1 + 0.15 * bw) & (vv >= y0 - 0.15 * bh) & (vv <= y1 + 0.15 * bh)

    by_index: dict[int, dict[str, Any]] = {}
    for idx, (x0, y0, x1, y1) in enumerate(boxes):
        in_box = (uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1) & valid
        if in_box.sum() < 4:
            continue
        mx, my = max(0.25 * (x1 - x0), 4 * cw), max(0.25 * (y1 - y0), 4 * ch)
        in_ring = (uu >= x0 - mx) & (uu <= x1 + mx) & (vv >= y0 - my) & (vv <= y1 + my)
        ring = in_ring & ~any_box & valid
        if not ring.any():
            continue
        uc, vc = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        p0, pu, pv = _plane_fit(uu[ring] - uc, vv[ring] - vc, disp[ring])
        bu, bv, bd = uu[in_box], vv[in_box], disp[in_box]
        deltas = bd - (p0 + pu * (bu - uc) + pv * (bv - vc))
        fg_mask = deltas > 0.0
        if fg_mask.sum() < 2:
            fg_mask = deltas >= float(np.median(deltas))
        fu, fv, fdeltas = bu[fg_mask], bv[fg_mask], deltas[fg_mask]
        disp_bg = max(0.0, p0 + pu * (float(fu.mean()) - uc) + pv * (float(fv.mean()) - vc))
        disp_fg = disp_bg + max(0.0, float(np.percentile(fdeltas, 75)))
        clear = True
        for j, (ox0, oy0, ox1, oy1) in enumerate(boxes):
            if j != idx and ox0 < x1 + 18.0 and ox1 > x0 - 18.0 and oy0 < y1 + 18.0 and oy1 > y0 - 18.0:
                clear = False
                break
        by_index[idx] = {
            "disp_fg": round(disp_fg, 5),
            "disp_bg": round(disp_bg, 5),
            "depth_conf": model.conf,
            "length_px": round(float(max(x1 - x0, y1 - y0)), 2),
            "jaw_clear": clear,
            "depth_width_px": round(float(min(x1 - x0, y1 - y0)), 2),
        }

    usable = valid & ~excl
    samples = []
    for jy in range(6):
        cgy = int(round((0.08 + 0.84 * jy / 5) * (gh - 1)))
        for jx in range(8):
            cgx = int(round((0.08 + 0.84 * jx / 7) * (gw - 1)))
            sl = (slice(max(cgy - 2, 0), cgy + 3), slice(max(cgx - 2, 0), cgx + 3))
            keep = usable[sl]
            if keep.sum() < 9:
                continue
            d = disp[sl][keep]
            med = float(np.median(d))
            if not math.isfinite(med) or _mad(d) > 0.05 * max(med, 1e-3):
                continue
            samples.append([round(float(uu[sl][keep].mean()), 2), round(float(vv[sl][keep].mean()), 2), round(med, 5)])
    return {"table_samples": samples, "objects": {}, "by_index": by_index, "finite": finite}


def build_detector_backend(
    mode: str,
    *,
    model_path: str | Path | None = None,
    labels: Sequence[str] | None = None,
    callable_fn: Callable[[np.ndarray], Sequence[dict]] | str | None = None,
    conf_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    output_format: str = "auto",
):
    """Build detector backend for `mode` ('gt' -> None, 'onnx' -> OnnxDetector, 'callable' -> CallableDetector)."""
    m = normalize_detector_mode(mode)
    if m == "gt":
        return None
    if m == "onnx":
        if not model_path:
            raise ValueError("detector_mode='onnx' requires detector_model")
        return OnnxDetector(
            model_path,
            labels=labels,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            output_format=output_format,
        )
    if callable_fn is None:
        raise ValueError("detector_mode='callable' requires a callable")
    fn = load_callable(callable_fn) if isinstance(callable_fn, str) else callable_fn
    return CallableDetector(fn, labels=labels)


def _box_iou(b1: list[float], b2: list[float]) -> float:
    ix0 = max(b1[0], b2[0])
    iy0 = max(b1[1], b2[1])
    ix1 = min(b1[2], b2[2])
    iy1 = min(b1[3], b2[3])
    iw = max(0.0, ix1 - ix0)
    ih = max(0.0, iy1 - iy0)
    inter = iw * ih
    a1 = max(0.0, b1[2] - b1[0]) * max(0.0, b1[3] - b1[1])
    a2 = max(0.0, b2[2] - b2[0]) * max(0.0, b2[3] - b2[1])
    union = a1 + a2 - inter
    return inter / union if union > 0 else 0.0


def _apply_mask_focus(detections: list[dict], focus: dict | None) -> list[dict]:
    out = [dict(d) for d in detections]
    for d in out:
        d.pop("mask_focus", None)
    if not focus or "box_norm" not in focus:
        return out
    fbox = focus["box_norm"]
    flabel = focus.get("label")
    best_iou = 0.099999
    best_det = None
    for d in out:
        if flabel and d.get("class_name") != flabel and d.get("name") != flabel:
            continue
        if "x" not in d or "y" not in d or "w" not in d or "h" not in d:
            continue
        cx, cy, w, h = d["x"], d["y"], d["w"], d["h"]
        dbox = [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]
        iou = _box_iou(fbox, dbox)
        if iou > best_iou:
            best_iou = iou
            best_det = d
    if best_det is not None:
        best_det["mask_focus"] = True
    return out


# --------------------------------------------------------------------------- video


class VideoStream:
    """Fan-out of encoded access units to WebSocket clients of one path."""

    QUEUE = 60

    def __init__(self):
        self.clients: list[queue.Queue] = []
        self.lock = threading.Lock()
        self.want_idr = True
        self.sent = 0

    def add(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(self.QUEUE)
        with self.lock:
            self.clients.append(q)
            self.want_idr = True
        return q

    def remove(self, q: queue.Queue) -> None:
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    @property
    def active(self) -> bool:
        return bool(self.clients)

    def take_idr_request(self) -> bool:
        with self.lock:
            want, self.want_idr = self.want_idr, False
        return want

    def publish(self, au: bytes, ts_ms: int, idr: bool) -> None:
        msg = au + int(ts_ms).to_bytes(8, "little")
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            if getattr(q, "started", False) is False:
                if not idr:
                    continue
                q.started = True
            try:
                q.put_nowait(msg)
            except (
                queue.Full
            ):  # slow client: drop its backlog, restart it at the next IDR
                with q.mutex:
                    q.queue.clear()
                q.started = False
                with self.lock:
                    self.want_idr = True
        self.sent += 1


def serve_video(host: str, port: int, streams: dict[str, VideoStream]):
    """Start the WebSocket server in a thread; returns the server."""
    from websockets.exceptions import ConnectionClosed
    from websockets.sync.server import serve

    def handler(ws):
        path = urlparse(ws.request.path).path.rstrip("/") or "/"
        stream = streams.get(path)
        if stream is None:
            ws.close(1008, "unknown path")
            return
        q = stream.add()
        try:
            while True:
                try:
                    msg = q.get(timeout=1.0)
                except queue.Empty:
                    continue
                ws.send(msg)
        except ConnectionClosed:
            pass
        finally:
            stream.remove(q)

    server = serve(handler, host, port, compression=None, max_size=2**20)
    threading.Thread(target=server.serve_forever, name="video-ws", daemon=True).start()
    return server


# --------------------------------------------------------------------------- perception


class Perception:
    """Perception API v2 state: default subscription (mode), subscriptions, frames, triggers."""

    def __init__(
        self,
        max_fps: int,
        depth=None,
        model_kind: str = "segment",
        nominal_fx: float = 824.0,
        detector_mode: str = "gt",
        detector_model: str | None = None,
        detector_labels: Sequence[str] | None = None,
    ):
        self.max_fps = max_fps
        # X8: the ESTIMATED intrinsics (rebot-vision's /api/calibration/intrinsics); the truth is
        # the camera model in camera.py. Starts at the nominal stream values, like rebot-vision.
        self.intrinsics = {"fx": nominal_fx, "fy": nominal_fx, "cx": 320.0, "cy": 180.0,
                           "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0, "marker_size": 0.05}
        self.intrinsics_revision = 0
        self._depth = depth if depth is not None and depth.enabled else None  # what `depth` loads
        # resident models: "seg" (mask angle and width) or "det" (--no-mask-angle), and "depth"
        self.detector: str | None = "seg" if model_kind == "segment" else "det"
        self.detector_mode = normalize_detector_mode(detector_mode)
        self.detector_model = detector_model
        self.detector_labels = list(detector_labels or [])
        self.depth_model = self._depth
        self.catalogue = {n: v for n, v in MODELS.items() if n != "depth" or self._depth}
        self.generation = 0
        self.swap: dict | None = None  # the pending swap: {"detector"?: name|None, "depth"?: name|None}
        self.swap_state = "idle"
        self.mode, self.enabled, self.last_active = "off", False, "yolo"
        self.subs: dict[str, dict] = {}
        self._ids = itertools.count(1)
        self.cond = threading.Condition()
        self.frame: dict | None = (
            None  # newest rendered frame: id, ts_ms, mono, detections
        )
        self.latest: dict | None = None  # newest published result
        self.triggers = 0
        self.pending: list[
            dict
        ] = []  # registered triggers: {"called": mono, "pipelines": [...]}
        self.mask_focus: dict | None = None
        self._gate = 0.0  # mono of the last frame the continuous rate let through
        self.frames_in: list[float] = []
        self.results_out: list[float] = []

    @property
    def model_kind(self) -> str | None:
        """/api/mode model.kind of the resident detector (None with the slot empty, as rebot-vision)."""
        return MODELS[self.detector][0] if self.detector else None

    @property
    def pipelines(self) -> tuple[str, ...]:
        # "depth" only with a depth model: without one every body is as before
        return PIPELINES + ("depth",) if self.depth_model else PIPELINES

    # ---- models
    def _budget(self, detector: str | None, depth: str | None) -> tuple[bool, float]:
        need = ION["base_mb"] + ION["reserve_mb"] + sum(self.catalogue[n][1] for n in (detector, depth) if n)
        return need <= ION["total_mb"] + 1e-9, need

    def request_swap(self, body: dict) -> tuple[int, dict]:
        """POST /api/models: validate, budget-check the resident set with the request applied and
        store it as the single pending swap (applied by the next frame)."""
        if not body or not set(body) <= {"detector", "depth"}:
            raise ValueError("body needs detector or depth (and nothing else)")
        for slot, name in body.items():
            if name is not None and (name not in self.catalogue or (self.catalogue[name][0] == "depth") != (slot == "depth")):
                raise ValueError(f"no {slot} model named {name!r}")
        if self.swap is not None:
            return 409, {"api_version": API_VERSION, "error": "SWAP_BUSY"}
        ok, need = self._budget(body.get("detector", self.detector), body.get("depth", self._resident_depth()))
        if not ok:
            return 409, {"api_version": API_VERSION, "error": "ION_BUDGET", "need_mb": round(need, 3), "limit_mb": ION["total_mb"]}
        self.swap, self.swap_state = dict(body), "pending"
        return 202, {"api_version": API_VERSION, "generation": self.generation + 1}

    def _resident_depth(self) -> str | None:
        return "depth" if self.depth_model else None

    def _apply_swap(self) -> None:
        """The pending swap, on the frame thread (under cond)."""
        if "detector" in self.swap:
            self.detector = self.swap["detector"]
        if "depth" in self.swap:
            self.depth_model = self._depth if self.swap["depth"] else None
        self.swap, self.swap_state = None, "idle"
        self.generation += 1

    def models_body(self) -> dict:
        return {"generation": self.generation,
                "swap": {"state": self.swap_state, "generation": self.generation + (self.swap is not None)},
                "resident": {"detector": self.detector, "depth": self._resident_depth()},
                "catalogue": [{"name": n, "kind": k, "path": f"sim:{n}", "ion_mb": mb,
                               "resident": n in (self.detector, self._resident_depth())} for n, (k, mb) in self.catalogue.items()]}

    # ---- subscriptions
    def _expire(self) -> None:
        now = time.monotonic()
        for sid in [s for s, v in self.subs.items() if v["expires"] <= now]:
            del self.subs[sid]
        if self.mask_focus is not None and self.mask_focus["expires"] <= now:
            self.mask_focus = None

    def mask_focus_body(self) -> dict | None:
        self._expire()
        if self.mask_focus is None:
            return None
        now = time.monotonic()
        left = max(0, int((self.mask_focus["expires"] - now) * 1000))
        out = {
            "box_norm": list(self.mask_focus["box_norm"]),
            "ttl_ms": self.mask_focus["ttl_ms"],
            "expires_in_ms": left,
            "remaining_ttl_ms": left,
        }
        if self.mask_focus.get("label"):
            out["label"] = self.mask_focus["label"]
        return out

    def continuous(self) -> dict:
        self._expire()
        pipes = set(MODE_PIPELINES[self.mode]) if self.enabled else set()
        fps = self.max_fps if pipes else 0  # every term is capped at max_fps
        for s in self.subs.values():
            pipes |= set(s["pipelines"])
            fps = max(fps, self.sub_fps(s))
        return {
            "active": bool(pipes),
            "pipelines": [p for p in PIPELINES if p in pipes],
            "fps": fps,
            "subscriptions": len(self.subs),
        }

    def sub_fps(self, s: dict) -> int:
        """Effective rate of a subscription: its requested rate under the current cap."""
        return min(s["requested_fps"], self.max_fps)

    def sub_body(self, s: dict) -> dict:
        return {
            "id": s["id"],
            "client": s["client"],
            "pipelines": s["pipelines"],
            "fps": self.sub_fps(s),
            "requested_fps": s["requested_fps"],
            "ttl_ms": s["ttl_ms"],
            "expires_in_ms": max(0, int((s["expires"] - time.monotonic()) * 1000)),
        }

    # ---- frames
    def wants_depth(self, captured: float) -> bool:
        """A registered trigger asks for depth on a frame captured at `captured` (the plan of
        that frame will include it): the render thread then renders depth for it."""
        with self.cond:
            return self.depth_model is not None and any(
                "depth" in t["pipelines"] and t["called"] < captured for t in self.pending
            )

    def clear_latest(self) -> None:
        with self.cond:
            self.latest = None
            self.frame = None
            self.cond.notify_all()

    def on_frame(
        self,
        frame_id: int,
        ts_ms: int,
        captured: float,
        detections: list[dict],
        physics_time: float = 0.0,
        physics_step: int = 0,
        depth: dict | None = None,
        markers: list[dict] | None = None,
        inference_time_ms: float | None = None,
    ) -> None:
        """A rendered wrist frame (called by the render thread); `captured` = time.monotonic()
        when the simulator state (physics step `physics_step` at `physics_time` s) was copied.
        `depth`: camera.depth_result of the same render (with its physics_step), or None."""
        now = time.monotonic()
        delivery_delay_s = max(0.0, now - captured)
        with self.cond:
            if self.swap is not None:
                self._apply_swap()
            self.frame = {
                "frame_id": frame_id,
                "ts_ms": ts_ms,
                "mono": captured,
                "detections": detections,
                "physics_time": round(physics_time, 6),
                "physics_step": physics_step,
                "depth": depth,
                "markers": markers or [],
                "inference_time_ms": 0.1 if inference_time_ms is None else float(inference_time_ms),
                "delivery_delay_s": delivery_delay_s,
            }
            self.frames_in = [t for t in self.frames_in if now - t < 2.0] + [now]
            # One plan per frame, as rebot-vision's frame_plan: the continuous pipelines when the
            # rate lets this frame through, plus those of every trigger registered before it.
            cont, plan = self.continuous(), set()
            if cont["active"] and now - self._gate >= 1.0 / max(cont["fps"], 1) - 0.005:
                plan |= set(cont["pipelines"])
                self._gate = now
            frame_focus = None
            for t in self.pending:
                if t["called"] < captured:
                    plan |= set(t["pipelines"])
                    if t.get("mask_focus"):
                        frame_focus = t["mask_focus"]
            if frame_focus is None:
                self._expire()
                if self.mask_focus is not None:
                    frame_focus = self.mask_focus
            if plan:
                self.latest = self.result(
                    self.frame, [p for p in self.pipelines if p in plan], focus=frame_focus
                )
                self.results_out = [t for t in self.results_out if now - t < 2.0] + [
                    now
                ]
            self.cond.notify_all()

    def result(
        self, frame: dict, pipelines: list[str], focus: dict | None = None
    ) -> dict:
        ran = {p: p in pipelines for p in self.pipelines}
        depth = frame.get("depth")
        if "depth" in ran:  # depth runs only on a frame whose depth render belongs to it
            ran["depth"] = (
                ran["depth"]
                and depth is not None
                and depth.get("physics_step") == frame["physics_step"]
            )
        ran["yolo"] = (
            ran["yolo"] and self.detector is not None
        )  # an empty slot reports ran: false
        detections = (
            _apply_mask_focus(frame["detections"], focus) if ran["yolo"] else []
        )
        if self.model_kind != "segment":  # a plain detection model has no mask angle / width
            detections = [
                {k: v for k, v in d.items() if k not in MASK_FIELDS}
                for d in detections
            ]
        if ran.get("depth"):
            by_idx = depth.get("by_index") if isinstance(depth, dict) else None
            if by_idx is not None:
                detections = [
                    {**d, **by_idx[idx]} if idx in by_idx else d
                    for idx, d in enumerate(detections)
                ]
            else:
                detections = [
                    {**d, **depth["objects"][d.get("name")]}
                    if d.get("name") in depth["objects"]
                    else d
                    for d in detections
                ]
        markers = []
        if ran.get("aruco"):  # pose with the estimated intrinsics, as rebot-vision
            from .aruco import marker_json

            markers = [marker_json(m, self.intrinsics, self.intrinsics["marker_size"]) for m in frame.get("markers", [])]
        inf_ms = round(float(frame.get("inference_time_ms", 0.1)), 3)
        body = {
            "api_version": API_VERSION,
            "success": True,
            "mode": self.mode,
            "frame_id": frame["frame_id"],
            "ts_ms": frame["ts_ms"],
            "mono": frame["mono"],
            "physics_time": frame["physics_time"],
            "physics_step": frame["physics_step"],
            "inference_time_ms": inf_ms,
            "width": 640,
            "height": 360,
            "pipelines": {
                p: {"ran": ran[p], "ms": (inf_ms if p == "yolo" else 0.1) if ran[p] else 0.0}
                for p in self.pipelines
            },
            "detections": detections,
            "count": len(markers),
            "family": "aruco4x4_50",
            "undistorted": any(self.intrinsics[k] != 0.0 for k in ("k1", "k2", "p1", "p2", "k3")),
            "rejected": 0,
            "markers": markers,
            "model_generation": self.generation,
        }
        if ran.get("depth"):
            from .camera import DEPTH_GRID  # mujoco: imported only where depth runs

            gw, gh = DEPTH_GRID
            body["depth"] = {"metric_model": self.depth_model.metric, "grid_w": gw, "grid_h": gh,
                             "table_samples": [list(t) for t in depth["table_samples"]]}
        return body

    def intrinsics_body(self) -> dict:
        """GET /api/calibration/intrinsics (rebot-vision W7 shape)."""
        return {"api_version": API_VERSION, **self.intrinsics, "width": 640, "height": 360,
                "revision": self.intrinsics_revision, "overridden_by_cli": False, "config_path": None}

    @staticmethod
    def public(body: dict) -> dict:
        out = {k: v for k, v in body.items() if k != "mono"}
        if "mono" in body:
            out["age_ms"] = int((time.monotonic() - body["mono"]) * 1000)
        return out

    def mode_body(self) -> dict:
        cont = self.continuous()
        now = time.monotonic()
        if self.detector_mode == "gt":
            model_name, model_path = "rebot-sim-ground-truth", None
        elif self.detector_mode == "onnx":
            model_name = os.path.basename(self.detector_model) if self.detector_model else "onnx"
            model_path = self.detector_model
        else:
            model_name, model_path = "rebot-sim-callable", None
        body = {
            "api_version": API_VERSION,
            "mode": self.mode,
            "modes": list(MODES),
            "detector_mode": self.detector_mode,
            "pipelines": {"yolo": True, "aruco": True},
            "last_active_mode": self.last_active,
            "enabled": self.enabled,
            "processing": cont["active"],
            "fps": cont["fps"] or IDLE_FPS,
            "active_fps": self.max_fps,
            "idle_fps": IDLE_FPS,
            "fps_range": [1, 15],
            "camera_fps": self.max_fps,
            "measured": {
                "input_fps": round(
                    len([t for t in self.frames_in if now - t < 2.0]) / 2, 1
                ),
                "result_fps": round(
                    len([t for t in self.results_out if now - t < 2.0]) / 2, 1
                ),
            },
            "trigger": True,
            "per_request_pipelines": True,
            "subscriptions": True,
            "max_fps": self.max_fps,
            "continuous": cont,
            "available": {"yolo": self.detector is not None, "aruco": True},
            "model": {
                "name": model_name,
                "path": model_path,
                "kind": self.model_kind,
                "mask_max_boxes": 3,
            },
            "family": "aruco4x4_50",
            "intrinsics": self.intrinsics_body(),
            "mask_focus": self.mask_focus_body(),
            "models": self.models_body(),
            "ion": {**ION, "used_mb": round(self._budget(self.detector, self._resident_depth())[1] - ION["reserve_mb"], 3)},
        }
        if self.depth_model:
            dm = self.depth_model
            body["pipelines"]["depth"] = body["available"]["depth"] = True
            body["depth_model"] = {"name": f"rebot-sim-depth-{dm.kind}", "path": f"sim:{dm.kind}", "metric": dm.metric}
        return body


def _is_same_origin(origin: str | None, host: str | None) -> bool:
    if not origin:
        return True
    try:
        uri = urlparse(origin)
        if uri.scheme not in ("http", "https"):
            return False
        if uri.path and uri.path != "/":
            return False
        if not host:
            return False
        if uri.netloc.lower() == host.lower():
            return True
        origin_host = (uri.hostname or "").lower()
        origin_port = uri.port or (80 if uri.scheme == "http" else 443)
        host_parts = host.split(":")
        h_name = host_parts[0].lower()
        h_port = (
            int(host_parts[1])
            if len(host_parts) > 1
            else (80 if uri.scheme == "http" else 443)
        )
        loopback = {"localhost", "127.0.0.1", "::1"}
        return origin_host in loopback and h_name in loopback and origin_port == h_port
    except Exception:
        return False


class _Handler(BaseHTTPRequestHandler):
    server_version = "rebot-sim"
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):  # quiet
        pass

    @property
    def app(self) -> "PerceptionServer":
        return self.server.app  # type: ignore[attr-defined]

    def _send(self, code: int, body: dict) -> None:
        data = json.dumps(body, separators=(",", ":")).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict | None:
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        if not raw.strip():
            return {}
        try:
            v = json.loads(raw)
        except ValueError:
            return None
        return v if isinstance(v, dict) else None

    def do_GET(self):
        self._route("GET")

    def do_POST(self):
        self._route("POST")

    def do_DELETE(self):
        self._route("DELETE")

    def do_PUT(self):
        self._route("PUT")

    def _route(self, method: str) -> None:
        if method != "GET":
            if not _is_same_origin(
                self.headers.get("Origin"), self.headers.get("Host")
            ):
                return self._send(
                    403, {"error": "Cross-origin control requests are not allowed"}
                )
        if method in ("POST", "PUT"):
            ct = self.headers.get("Content-Type", "")
            media_type = ct.split(";")[0].strip().lower()
            if media_type != "application/json":
                return self._send(
                    415, {"error": "Content-Type must be application/json"}
                )
        url = urlparse(self.path)
        qs = {k: v[-1] for k, v in parse_qs(url.query).items()}
        body = self._body() if method in ("POST", "PUT") else {}
        if body is None:
            return self._send(400, {"error": "body must be a JSON object"})
        try:
            code, out = self.app.dispatch(method, url.path.rstrip("/") or "/", qs, body)
        except ValueError as exc:
            code, out = 400, {"error": str(exc)}
        except TimeoutError as exc:  # the simulator's physics loop did not answer
            code, out = 504, {"error": str(exc)}
        except (OSError, EOFError) as exc:  # simulator pipe closed
            code, out = 503, {"error": f"simulator unavailable: {exc!r}"}
        self._send(code, out)


def _wait_ms(qs: dict, body: dict, default: int) -> int:
    v = body.get("wait_ms", qs.get("wait_ms", default))
    try:
        v = int(v)
    except (TypeError, ValueError):
        raise ValueError("wait_ms must be an integer") from None
    return max(0, min(5000, v))


def _pipelines(v, allowed: tuple[str, ...] = PIPELINES) -> list[str]:
    if not isinstance(v, list) or not v or any(p not in allowed for p in v):
        raise ValueError(f"pipelines must be a non-empty list of {' / '.join(allowed)}")
    return [p for p in allowed if p in v]


def _is_degraded(sim) -> bool:
    return getattr(sim, "activation_state", "COMMITTED") == "DEGRADED_SPLIT"


def _degraded_response(sim) -> tuple[int, dict]:
    body: dict = {
        "error": "SIM_ACTIVATION_DEGRADED",
        "code": "SIM_ACTIVATION_DEGRADED",
        "state": "DEGRADED_SPLIT",
    }
    if hasattr(sim, "active_tx_id"):
        body["tx_id"] = sim.active_tx_id
    if hasattr(sim, "camera_tx_id"):
        body["camera_tx_id"] = sim.camera_tx_id
    if hasattr(sim, "robot_id"):
        body["robot"] = sim.robot_id
    return 503, body


_DEGRADED_ROUTES = frozenset(
    {
        ("GET", "/sim/camera"),
        ("POST", "/api/detect"),
        ("GET", "/api/detections"),
        ("POST", "/api/trigger"),
        ("GET", "/api/results/latest"),
        ("POST", "/sim/scene"),
        ("POST", "/sim/scene/prepare"),
    }
)


def _validate_optional_seed(seed: Any) -> None:
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
        raise ValueError("seed must be a non-negative integer or null")


class PerceptionServer:
    """HTTP routes over a Perception state and the simulator (reset / state / camera info)."""

    def __init__(self, perception: Perception, sim, host: str, port: int):
        self.p, self.sim = perception, sim
        self.httpd = ThreadingHTTPServer((host, port), _Handler)
        self.httpd.daemon_threads = True
        self.httpd.app = self  # type: ignore[attr-defined]
        threading.Thread(
            target=self.httpd.serve_forever, name="perception-http", daemon=True
        ).start()

    def dispatch(
        self, method: str, path: str, qs: dict, body: dict
    ) -> tuple[int, dict]:
        p = self.p
        if _is_degraded(self.sim) and (method, path) in _DEGRADED_ROUTES:
            return _degraded_response(self.sim)
        if (path == "/api/mode" or path == "/api/status") and method == "GET":
            with p.cond:
                return 200, p.mode_body()
        if path == "/api/mode" and method == "POST":
            if not body or not set(body) <= {"mode", "enabled", "fps"}:
                raise ValueError("body needs mode, enabled or fps")
            if "mode" in body and body["mode"] not in MODES:
                raise ValueError("bad mode")
            if "enabled" in body and not isinstance(body["enabled"], bool):
                raise ValueError("enabled must be a bool")
            if "fps" in body and (
                isinstance(body["fps"], bool)
                or not isinstance(body["fps"], int)
                or not 1 <= body["fps"] <= 15
            ):
                raise ValueError("fps must be 1..15")
            with p.cond:  # the whole body is valid: apply it
                if "mode" in body:
                    p.mode = body["mode"]
                    if p.mode == "off":
                        p.enabled = False
                    else:
                        p.last_active = p.mode
                if "enabled" in body:
                    p.enabled = body["enabled"]
                    if p.enabled and p.mode == "off":
                        p.mode = p.last_active
                if "fps" in body:
                    p.max_fps = body["fps"]
                return 200, p.mode_body()
        if path == "/api/calibration/intrinsics":
            from .aruco import IntrinsicsError, check_intrinsics_put

            with p.cond:
                if method == "GET":
                    return 200, p.intrinsics_body()
                if method == "PUT":
                    try:
                        new = check_intrinsics_put(body, p.intrinsics, p.intrinsics_revision)
                    except IntrinsicsError as e:
                        out = {"api_version": API_VERSION, "error": e.code, "code": e.code, "message": str(e)}
                        if e.field:
                            out["field"] = e.field
                        if e.code == "REVISION_MISMATCH":
                            out["revision"] = p.intrinsics_revision
                        return e.status, out
                    p.intrinsics = new
                    p.intrinsics_revision += 1
                    return 200, p.intrinsics_body()
        if path == "/api/models" and method == "POST":
            with p.cond:
                return p.request_swap(body)
        if path == "/api/mask_focus":
            if method == "GET":
                with p.cond:
                    return 200, {
                        "api_version": API_VERSION,
                        "mask_focus": p.mask_focus_body(),
                    }
            if method == "POST":
                with p.cond:
                    if not body or (
                        isinstance(body.get("ttl_ms"), int) and body["ttl_ms"] == 0
                    ):
                        p.mask_focus = None
                        return 200, {"api_version": API_VERSION, "mask_focus": None}
                    box_norm = body.get("box_norm")
                    if (
                        not isinstance(box_norm, list)
                        or len(box_norm) != 4
                        or not all(isinstance(x, (int, float)) for x in box_norm)
                    ):
                        raise ValueError("box_norm must be a list of 4 numbers")
                    ttl = body.get("ttl_ms", 2000)
                    if not isinstance(ttl, int):
                        raise ValueError("ttl_ms must be an integer")
                    ttl = max(100, min(10000, ttl))
                    label = body.get("label")
                    if label is not None and not isinstance(label, str):
                        raise ValueError("label must be a string")
                    p.mask_focus = {
                        "box_norm": [float(x) for x in box_norm],
                        "ttl_ms": ttl,
                        "expires": time.monotonic() + ttl / 1000.0,
                        "label": label if label else None,
                    }
                    return 200, {
                        "api_version": API_VERSION,
                        "mask_focus": p.mask_focus_body(),
                    }
        if path == "/api/inference":
            with p.cond:
                if method == "POST":
                    if not isinstance(body.get("enabled"), bool):
                        raise ValueError("enabled must be a bool")
                    p.enabled = body["enabled"]
                    if p.enabled and p.mode == "off":
                        p.mode = p.last_active
                return 200, {
                    "api_version": API_VERSION,
                    "enabled": p.enabled,
                    "fps": p.max_fps if p.enabled else IDLE_FPS,
                    "active_fps": p.max_fps,
                    "idle_fps": IDLE_FPS,
                }
        if path in {"/api/trigger", "/api/detect"} and method == "POST":
            return self.trigger(qs, body)
        if path in {"/api/results/latest", "/api/detections"} and method == "GET":
            return self.latest(qs)
        if path == "/api/subscribe" and method == "POST":
            return self.subscribe(body)
        if path.startswith("/api/subscribe/"):
            parts = path.split("/")
            sid = parts[3]
            with p.cond:
                p._expire()
                s = p.subs.get(sid)
                if s is None:
                    return 404, {
                        "api_version": API_VERSION,
                        "error": "unknown subscription",
                    }
                if method == "POST" and len(parts) == 5 and parts[4] == "keepalive":
                    s["expires"] = time.monotonic() + s["ttl_ms"] / 1000
                    return 200, {
                        "api_version": API_VERSION,
                        **p.sub_body(s),
                        "continuous": p.continuous(),
                    }
                if method == "DELETE" and len(parts) == 4:
                    del p.subs[sid]
                    return 200, {
                        "api_version": API_VERSION,
                        "deleted": sid,
                        "continuous": p.continuous(),
                    }
        if path == "/api/subscriptions" and method == "GET":
            with p.cond:
                cont = p.continuous()
                return 200, {
                    "api_version": API_VERSION,
                    "max_fps": p.max_fps,
                    "max_subscriptions": MAX_SUBSCRIPTIONS,
                    "default": {"mode": p.mode, "enabled": p.enabled, "fps": p.max_fps},
                    "continuous": cont,
                    "subscriptions": [p.sub_body(s) for s in p.subs.values()],
                }
        if path == "/sim/reset" and method == "POST":
            seed = body.get("seed")
            _validate_optional_seed(seed)
            if "dr" in body:
                dr_cfg = DomainRandomizationConfig.from_dict(body.get("dr"))
                if not hasattr(self.sim, "request_reset"):
                    return 404, {"error": "not found"}
                import inspect

                sig = inspect.signature(self.sim.request_reset)
                if "dr" not in sig.parameters and not any(
                    par.kind == inspect.Parameter.VAR_KEYWORD for par in sig.parameters.values()
                ):
                    raise ValueError("simulator does not support per-reset dr config")
                out = self.sim.request_reset(seed, dr=dr_cfg)
            else:
                out = self.sim.request_reset(seed)
            return (500 if "error" in out else 200), out
        if path == "/sim/dr":
            if method == "GET":
                if not hasattr(self.sim, "get_dr"):
                    return 404, {"error": "not found"}
                return 200, self.sim.get_dr()
            if method == "POST":
                if not hasattr(self.sim, "set_dr"):
                    return 404, {"error": "not found"}
                has_seed = "seed" in body
                seed = body.get("seed")
                _validate_optional_seed(seed)
                dr_body = {k: v for k, v in body.items() if k != "seed"}
                dr_cfg = DomainRandomizationConfig.from_dict(dr_body)
                return 200, self.sim.set_dr(dr_cfg, seed=seed, use_seed=has_seed)
        if path == "/sim/detector":
            if method == "GET":
                registered = (
                    self.sim.registered_detector_ids()
                    if hasattr(self.sim, "registered_detector_ids")
                    else []
                )
                with p.cond:
                    return 200, {
                        "detector_mode": p.detector_mode,
                        "detector_model": p.detector_model,
                        "detector_labels": list(p.detector_labels),
                        "registered": registered,
                    }
            if method == "POST":
                forbidden = {
                    "callable",
                    "detector_callable",
                    "module",
                    "model_path",
                    "detector_model",
                    "model",
                }
                present_forbidden = sorted(forbidden & set(body))
                if present_forbidden:
                    return 403, {
                        "error": (
                            "dynamic detector loading/import over HTTP is forbidden "
                            f"({', '.join(present_forbidden)})"
                        ),
                        "code": "FORBIDDEN",
                    }
                allowed = {"detector_mode", "mode", "detector_id", "id", "labels", "detector_labels"}
                unknown = sorted(set(body) - allowed)
                if unknown:
                    raise ValueError(f"unknown detector fields: {unknown}")
                if not hasattr(self.sim, "select_detector"):
                    return 404, {"error": "not found"}
                det_id = body.get("detector_id", body.get("id"))
                raw_mode = body.get("detector_mode", body.get("mode"))
                mode = normalize_detector_mode(raw_mode) if raw_mode is not None else None
                labels = body.get("labels", body.get("detector_labels"))
                if isinstance(labels, str):
                    labels = [s.strip() for s in labels.split(",") if s.strip()]
                elif labels is not None and not isinstance(labels, list):
                    raise ValueError("labels must be a list of strings or comma-separated string")
                return 200, self.sim.select_detector(
                    mode=mode,
                    detector_id=det_id,
                    labels=labels,
                )
        if path == "/sim/state" and method == "GET":
            return 200, self.sim.sim_state()
        if path == "/sim/camera" and method == "GET":
            if not hasattr(self.sim, "camera_info"):
                return 404, {"error": "camera not enabled"}
            return 200, self.sim.camera_info()
        if path == "/sim/scene/prepare" and method == "POST":
            if not hasattr(self.sim, "prepare_scene"):
                return 404, {"error": "not found"}
            return self.sim.prepare_scene(body)
        if path == "/sim/scene/commit" and method == "POST":
            if not hasattr(self.sim, "commit_scene"):
                return 404, {"error": "not found"}
            status, out = self.sim.commit_scene(body)
            if status == 200:
                p.clear_latest()
            return status, out
        if path == "/sim/scene/abort" and method == "POST":
            if not hasattr(self.sim, "abort_scene"):
                return 404, {"error": "not found"}
            return self.sim.abort_scene(body)
        if path == "/sim/scene":
            if method == "GET":
                if not hasattr(self.sim, "get_scene"):
                    return 404, {"error": "not found"}
                return 200, self.sim.get_scene()
            if method == "POST":
                if not hasattr(self.sim, "load_scene"):
                    return 404, {"error": "not found"}
                from .pack import is_valid_pack_slug

                for key in ("scene", "tool"):
                    if key in body:
                        val = body[key]
                        if val is not None and not is_valid_pack_slug(val):
                            return 400, {"error": f"invalid {key} slug: {val!r}", "code": "INVALID_PACK"}
                out = self.sim.load_scene(body)
                if isinstance(out, dict) and "_http_status" in out:
                    status = int(out.pop("_http_status"))
                    cleaned = {k: v for k, v in out.items() if not k.startswith("_")}
                    if status == 200:
                        p.clear_latest()
                    return status, cleaned
                status = 400 if "error" in out else 200
                if status == 200:
                    p.clear_latest()
                return status, out
        if path == "/sim/io":
            if method == "GET":
                if not hasattr(self.sim, "get_io"):
                    return 404, {"error": "not found"}
                return 200, self.sim.get_io()
            if method == "POST":
                if not hasattr(self.sim, "set_io"):
                    return 404, {"error": "not found"}
                out = self.sim.set_io(body)
                return (400 if "error" in out else 200), out
        return 404, {"error": "not found"}

    def trigger(self, qs: dict, body: dict) -> tuple[int, dict]:
        p = self.p
        wait = _wait_ms(qs, body, 2500)
        with p.cond:
            req = _pipelines(body["pipelines"], p.pipelines) if "pipelines" in body else None
            if req and "yolo" in req and p.detector is None:
                return 409, {"api_version": API_VERSION, "error": "yolo unavailable"}
            pipes = (
                req
                if req is not None
                else (MODE_PIPELINES[p.mode] or MODE_PIPELINES[p.last_active])
            )
            called = time.monotonic()
            after = p.latest["frame_id"] if p.latest else 0
            focus = None
            if "mask_focus" in body and isinstance(body["mask_focus"], dict):
                mf = body["mask_focus"]
                box_norm = mf.get("box_norm")
                if (
                    isinstance(box_norm, list)
                    and len(box_norm) == 4
                    and all(isinstance(x, (int, float)) for x in box_norm)
                ):
                    label = mf.get("label")
                    focus = {
                        "box_norm": [float(x) for x in box_norm],
                        "label": label if isinstance(label, str) and label else None,
                    }
            entry = {"called": called, "pipelines": pipes, "mask_focus": focus}
            p.pending.append(entry)  # the next frame's plan includes these pipelines

            def fresh() -> bool:
                r = p.latest
                return (
                    r is not None
                    and r["frame_id"] > after
                    and r["mono"] > called
                    and all(r["pipelines"][x]["ran"] for x in pipes)
                )

            try:
                ok = p.cond.wait_for(fresh, wait / 1000)
            finally:
                p.pending.remove(entry)
            if _is_degraded(self.sim):
                return _degraded_response(self.sim)
            if not ok:
                return 200, {
                    "api_version": API_VERSION,
                    "success": False,
                    "timeout": True,
                    "count": 0,
                    "detections": [],
                    "markers": [],
                }
            p.triggers += 1
            return 200, p.public({**p.latest, "requested": pipes})

    def latest(self, qs: dict) -> tuple[int, dict]:
        p = self.p
        wait = _wait_ms(qs, {}, 0)
        try:
            after = int(qs.get("after", 0))
        except ValueError:
            raise ValueError("after must be an integer") from None
        with p.cond:
            ok = p.cond.wait_for(
                lambda: p.latest is not None and p.latest["frame_id"] > after,
                wait / 1000,
            )
            if _is_degraded(self.sim):
                return _degraded_response(self.sim)
            enabled = p.continuous()["active"]
            if p.latest is None:
                return 200, {
                    "api_version": API_VERSION,
                    "enabled": enabled,
                    "frame_id": 0,
                    "ts_ms": 0,
                    "age_ms": 0,
                    "inference_time_ms": 0,
                    "detections": [],
                    "markers": [],
                    "timeout": True,
                }
            out = p.public(p.latest)
            out.pop("success", None)
            out["enabled"] = enabled
            if not ok:
                out["timeout"] = True
            return 200, out

    def subscribe(self, body: dict) -> tuple[int, dict]:
        p = self.p
        client = body.get("client")
        if not isinstance(client, str) or not 1 <= len(client) <= 64:
            raise ValueError("client must be 1..64 characters")
        pipes = _pipelines(body.get("pipelines"))
        fps = body.get("fps", p.max_fps)
        ttl = body.get("ttl_ms", 3000)
        if not isinstance(fps, int) or not 1 <= fps <= 15:
            raise ValueError("fps must be 1..15")
        if not isinstance(ttl, int) or not 500 <= ttl <= 60000:
            raise ValueError("ttl_ms must be 500..60000")
        with p.cond:
            p._expire()
            s = next(
                (
                    s
                    for s in p.subs.values()
                    if s["client"] == client or s["id"] == body.get("id")
                ),
                None,
            )
            if s is None:
                if len(p.subs) >= MAX_SUBSCRIPTIONS:
                    return 503, {
                        "api_version": API_VERSION,
                        "error": "too many subscriptions",
                    }
                s = {"id": f"s{next(p._ids)}"}
                p.subs[s["id"]] = s
            s.update(
                client=client,
                pipelines=pipes,
                requested_fps=fps,
                ttl_ms=ttl,
                expires=time.monotonic() + ttl / 1000,
            )
            return 200, {
                "api_version": API_VERSION,
                **p.sub_body(s),
                "continuous": p.continuous(),
            }


# --------------------------------------------------------------------------- render process
#
# The camera runs in a child process (spawned, so it gets its own GL context and GIL): on a
# loaded host a render thread in the simulator process cut the 500 Hz physics loop to about
# 220 Hz, while a separate rendering process left it at 500 Hz. The simulator publishes
# its state in shared memory under a multiprocessing lock and answers /sim requests over a pipe.

HDR = 5  # shared float64 slots: writes, sim time, camera frames, camera ms/frame, physics step


class SharedState:
    """Simulator state in shared memory: [writes, time, frames, frame_ms, step, qpos..., qvel...].
    `step` counts the physics states written, so a reader can tell whether physics advanced.
    Writes and reads hold `lock` (a multiprocessing lock: it orders the memory accesses across
    processes, which a seqlock on plain numpy stores does not)."""

    def __init__(self, nq: int, nv: int, name: str | None = None, lock=None):
        from multiprocessing import shared_memory

        size = (HDR + nq + nv) * 8
        self.owner = (
            name is None
        )  # the creator unlinks; an attached reader is not tracked
        self.shm = shared_memory.SharedMemory(
            name=name, create=self.owner, size=size, track=self.owner
        )
        self.arr = np.ndarray((HDR + nq + nv,), dtype=np.float64, buffer=self.shm.buf)
        self.nq, self.nv = nq, nv
        if lock is None:
            import multiprocessing as mp

            lock = mp.get_context("spawn").Lock()
        self.lock = lock
        if self.owner:
            self.arr[:] = 0.0

    def close(self) -> None:
        """Release the mapping; the owner also unlinks the segment. Idempotent."""
        if self.arr is None:
            return
        self.arr = None
        self.shm.close()
        if self.owner:
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass

    LOCK_TIMEOUT_S = 0.2  # a process that died holding the lock never releases it

    def write(self, data) -> None:
        """Publish `data`; raises TimeoutError when the lock is not had within LOCK_TIMEOUT_S."""
        a = self.arr
        if not self.lock.acquire(timeout=self.LOCK_TIMEOUT_S):
            raise TimeoutError("shared-state lock timeout")
        try:
            a[0] += 1
            a[1] = data.time
            a[4] += 1
            a[HDR : HDR + self.nq] = data.qpos
            a[HDR + self.nq :] = data.qvel
        finally:
            self.lock.release()

    def read(self, data) -> int | None:
        """Copy the state into `data`; returns its physics step, or None (and leaves `data`
        alone) when nothing was written yet or the lock was not had within LOCK_TIMEOUT_S."""
        a = self.arr
        if not self.lock.acquire(timeout=self.LOCK_TIMEOUT_S):
            return None
        try:
            if a[0] == 0:
                return None
            data.qpos[:], data.qvel[:], data.time = (
                a[HDR : HDR + self.nq],
                a[HDR + self.nq :],
                a[1],
            )
            return int(a[4])
        finally:
            self.lock.release()


class SimLink:
    """Child side of the simulator pipe: /sim/reset and /sim/state go to the physics loop.
    Requests are `(id, op, *args)`, replies `(id, body)`; a reply to an earlier request that
    timed out is dropped, so it never answers a later one."""

    def __init__(self, conn):
        self.conn, self.lock = conn, threading.Lock()
        self._ids = itertools.count(1)

    def call(self, msg: tuple, timeout: float = 3.0) -> dict:
        with self.lock:
            while self.conn.poll():  # late replies to timed-out requests
                self.conn.recv()
            rid = next(self._ids)
            self.conn.send((rid, *msg))
            deadline = time.monotonic() + timeout
            while True:
                left = deadline - time.monotonic()
                if left <= 0 or not self.conn.poll(left):
                    raise TimeoutError("simulator did not answer")
                got, body = self.conn.recv()
                if got == rid:
                    return body


class CameraRig:
    """Renders the wrist (and optionally scene) camera from the shared state, computes
    ground-truth or real-detector detections, encodes and publishes. Runs in the camera process."""

    def __init__(self, cfg: dict, state: SharedState, link: SimLink, detector_fn=None):
        from . import camera as cam

        self.cfg, self.state, self.link = cfg, state, link
        self.fps, self.size = cfg["fps"], tuple(cfg["size"])
        self.mount = cam.CameraMount(*cfg["mount"])
        self._cam = cam
        self.reload_lock = threading.Lock()
        self._reload_op_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._reload_req = None
        self.reload_wake_event = threading.Event()
        self.activation_state = "COMMITTED"
        self.active_tx_id: str | None = None
        self.pending_tx_id: str | None = None
        self.pending_cam_cleanup_tx_id: str | None = None
        self.camera_tx_id: str | None = None
        self.camera_committed_tx_id: str | None = None
        self._staged_cam_tx: dict | None = None
        self._prev_cam_gen: dict | None = None
        self._legacy_prev_cam: dict | None = None
        self.model, self.data, self.scene, self.labels = self._build_camera_world(
            scene_ref=cfg.get("scene"),
            tool_ref=cfg.get("tool"),
            objects_str=cfg.get("objects", "cube"),
        )
        self.detector_mode = normalize_detector_mode(cfg.get("detector_mode", "gt"))
        self.detector_model = cfg.get("detector_model")
        raw_labels = cfg.get("detector_labels")
        if isinstance(raw_labels, str):
            self.detector_labels = [s.strip() for s in raw_labels.split(",") if s.strip()]
        else:
            self.detector_labels = list(raw_labels or [])
        self.detector_conf = float(cfg.get("detector_conf", 0.25))
        self.detector_iou = float(cfg.get("detector_iou", 0.45))
        self.detector_output_format = str(cfg.get("detector_output_format", "auto"))
        self._detector_generation = 0
        self._delayed_lock = threading.Lock()
        self._delayed_gens: list[int] = []
        self._detector_backend = build_detector_backend(
            self.detector_mode,
            model_path=self.detector_model,
            labels=self.detector_labels,
            callable_fn=detector_fn if detector_fn is not None else cfg.get("detector_callable"),
            conf_threshold=self.detector_conf,
            iou_threshold=self.detector_iou,
            output_format=self.detector_output_format,
        )
        self._registered_detectors: dict[str, dict[str, Any]] = {
            "gt": {"mode": "gt", "backend": None, "model": None, "labels": []},
        }
        if self.detector_mode != "gt":
            entry = {
                "mode": self.detector_mode,
                "backend": self._detector_backend,
                "model": self.detector_model,
                "labels": list(self.detector_labels),
            }
            self._registered_detectors[self.detector_mode] = entry
            if cfg.get("detector_id"):
                self._registered_detectors[str(cfg["detector_id"])] = entry
        self.last_inference_ms = 0.1
        self.dr_config = coerce_dr_config(cfg.get("dr"))
        self.dr = SimDomainRandomizer(self.model, self.dr_config)
        self.dr_summary: dict | None = None
        self.dr_generation = int(cfg.get("dr_generation", 0) or 0)
        self.dr_seed = cfg.get("dr_seed")
        self._dr_latency_ms = 0.0
        if self.dr_config.enabled:
            init_seed = self.dr_seed if self.dr_seed is not None else cfg.get("seed")
            self.dr_summary = self.dr.apply(self.model, seed=init_seed)
            if self.dr_generation:
                self.dr_summary["generation"] = self.dr_generation
            self._dr_latency_ms = float(self.dr_summary.get("latency_ms", 0.0))
        self.streams = {"/": VideoStream()}
        if cfg["scene_stream"]:
            self.streams["/scene"] = VideoStream()
        self.depth_model = cam.DepthModel(**cfg.get("depth", {}))
        # False: a plain detection model (no mask, model "det"): detections carry no angle_deg /
        # angle_conf / grasp_width_px (the depth pipeline still reports its depth_* fields)
        self.perception = Perception(
            max_fps=min(15, self.fps),
            depth=self.depth_model,
            model_kind="segment" if cfg.get("mask_angle", True) else "detect",
            nominal_fx=round(cam.focal_px(cam.DETECTION_SIZE[1]), 2),
            detector_mode=self.detector_mode,
            detector_model=self.detector_model,
            detector_labels=self.detector_labels,
        )
        # virtual GridBoard (--aruco-board): {"x", "y", "yaw_deg", "side_m", "noise_px", "seed"} or None
        board = cfg.get("aruco_board")
        if board:
            from .aruco import SIDE_M as BOARD_SIDE_M, Board

            self.board = Board(board["x"], board["y"], board.get("yaw_deg", 0.0), board.get("z", 0.0), board.get("side_m", BOARD_SIDE_M))
            self.board_noise = (float(board.get("noise_px", 0.0)), int(board.get("seed", 0)))
        else:
            self.board, self.board_noise = None, (0.0, 0)
        self.noise = cam.DetNoise(**cfg.get("noise", {}))
        self.delayed: list[
            tuple
        ] = []  # frames (on_frame arguments) waiting out latency_ms
        self.frame_id = 0
        self.physics_step: int | None = None  # physics step of the newest frame
        self.skipped = 0  # camera ticks without a new physics state (nothing published)
        self.seg_failures = (
            0  # segmentation renders that failed (frame published without a result)
        )
        self.seg_consecutive = 0
        self.seg_last_error: str | None = None
        self.render_ms: list[float] = []
        self.depth_ms: list[float] = []  # depth render + fields, frames that had a depth trigger
        self.depth_failures = 0  # depth renders with non-finite or non-positive Z (frame gets no depth)
        self.frame_depth: dict | None = None  # depth_result of the newest detect(want_depth=True)
        self.backend = os.environ.get("MUJOCO_GL", "default")

    def _build_camera_world(
        self,
        scene_ref,
        tool_ref,
        objects_str: str,
        *,
        scene_is_dir: bool = False,
        tool_is_dir: bool = False,
        expected_scene_digest: str | None = None,
        expected_tool_digest: str | None = None,
        verify_digests: bool = False,
    ):
        import mujoco

        from .model import load_model
        from .pack import PackScene, PackValidationError, load_scene_pack, load_tool_pack
        from .scene import Scene, parse_objects

        scene_pack = load_scene_pack(scene_ref, prefer_disk=scene_is_dir)
        effective_tool = (
            tool_ref
            if tool_ref is not None
            else (scene_pack.default_tool if scene_pack else "default")
        )
        tool_pack = load_tool_pack(effective_tool, prefer_disk=tool_is_dir and tool_ref is not None)

        if verify_digests:
            actual_sd = scene_pack.digest if scene_pack is not None else None
            actual_td = tool_pack.digest if tool_pack is not None else None
            if actual_sd != expected_scene_digest:
                raise PackValidationError(
                    f"Camera scene digest mismatch: expected {expected_scene_digest!r}, got {actual_sd!r}",
                    code="DIGEST_MISMATCH",
                )
            if actual_td != expected_tool_digest:
                raise PackValidationError(
                    f"Camera tool digest mismatch: expected {expected_tool_digest!r}, got {actual_td!r}",
                    code="DIGEST_MISMATCH",
                )

        if scene_pack is not None:
            model = load_model(
                camera=self.mount,
                robot=self.cfg.get("robot"),
                scene_pack=scene_pack,
                tool_pack=tool_pack,
            )
            data = mujoco.MjData(model)
            scene = PackScene(model, scene_pack, tool_pack)
        else:
            objects = parse_objects(objects_str)
            model = load_model(
                objects=objects,
                camera=self.mount,
                robot=self.cfg.get("robot"),
                tool_pack=tool_pack,
            )
            data = mujoco.MjData(model)
            scene = (
                PackScene(model, None, tool_pack, fallback_objects=objects)
                if not tool_pack.is_default
                else Scene(model, objects)
            )
        labels = [(i, o.kind) for i, o in enumerate(scene.objects)]
        return model, data, scene, labels

    # PerceptionServer's simulator interface
    def apply_dr(
        self,
        seed: int | None = None,
        dr_config: DomainRandomizationConfig | dict | None = None,
        *,
        generation: int | None = None,
    ) -> dict:
        """Apply (or disable & restore baseline) DR on the camera model under reload_lock."""
        new_cfg = coerce_dr_config(dr_config) if dr_config is not None else None
        with self.reload_lock:
            if new_cfg is not None:
                self.dr_config = new_cfg
            if self.dr is None:
                self.dr = SimDomainRandomizer(self.model, self.dr_config)
            self.dr.config = self.dr_config
            if generation is not None:
                self.dr_generation = int(generation)
            else:
                self.dr_generation += 1
            if self.dr_config.enabled:
                summary = self.dr.apply(self.model, seed=seed)
                summary["generation"] = self.dr_generation
                self.dr_seed = summary.get("seed")
                self.dr_summary = summary
                self._dr_latency_ms = float(summary.get("latency_ms", 0.0))
            else:
                self.dr.restore_baseline(self.model)
                self.dr_seed = None
                self.dr_summary = None
                self._dr_latency_ms = 0.0
                summary = {"enabled": False, "seed": seed, "latency_ms": 0.0}
            self.cfg["dr"] = self.dr_config.to_dict()
            self.cfg["seed"] = seed
            self.cfg["dr_seed"] = self.dr_seed
            self.cfg["dr_generation"] = self.dr_generation
            return summary

    def _clear_delayed_queue(self) -> None:
        lock = getattr(self, "_delayed_lock", None)
        if lock is not None:
            with lock:
                self.delayed.clear()
                if hasattr(self, "_delayed_gens"):
                    self._delayed_gens.clear()
        else:
            self.delayed.clear()

    def _apply_detector_entry(self, entry: dict[str, Any], labels: Sequence[str] | None = None) -> dict:
        m = entry["mode"]
        eff_labels = list(labels) if labels is not None else list(entry.get("labels") or [])
        eff_model = entry.get("model")
        backend = entry.get("backend")
        if backend is not None and labels is not None and hasattr(backend, "labels"):
            backend.labels = list(eff_labels)
        with self.reload_lock:
            self._detector_generation += 1
            self.detector_mode = m
            self.detector_model = eff_model
            self.detector_labels = eff_labels
            self._detector_backend = backend
            self.cfg["detector_mode"] = m
            self.cfg["detector_model"] = eff_model
            self.cfg["detector_labels"] = eff_labels
        self._clear_delayed_queue()
        with self.perception.cond:
            self.perception.detector_mode = m
            self.perception.detector_model = eff_model
            self.perception.detector_labels = list(eff_labels)
        self.perception.clear_latest()
        return {
            "detector_mode": m,
            "detector_model": eff_model,
            "detector_labels": list(eff_labels),
            "detector_generation": self._detector_generation,
        }

    def registered_detector_ids(self) -> list[str]:
        with self.reload_lock:
            return sorted(self._registered_detectors.keys())

    def register_detector(
        self,
        detector_id: str,
        mode: str,
        *,
        fn=None,
        model_path: str | Path | None = None,
        labels: Sequence[str] | None = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        output_format: str = "auto",
    ) -> str:
        """Trusted startup/test helper to pre-register a detector backend under `detector_id`."""
        if not isinstance(detector_id, str) or not detector_id.strip():
            raise ValueError("detector_id must be a non-empty string")
        m = normalize_detector_mode(mode)
        eff_labels = list(labels or [])
        eff_model = str(model_path) if model_path is not None else None
        backend = build_detector_backend(
            m,
            model_path=eff_model,
            labels=eff_labels,
            callable_fn=fn,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            output_format=output_format,
        )
        key = detector_id.strip()
        entry = {
            "mode": m,
            "backend": backend,
            "model": eff_model,
            "labels": eff_labels,
        }
        with self.reload_lock:
            self._registered_detectors[key] = entry
        return key

    def select_detector(
        self,
        *,
        mode: str | None = None,
        detector_id: str | None = None,
        labels: Sequence[str] | None = None,
    ) -> dict:
        """HTTP-safe switcher: selects only between 'gt' and pre-registered detector IDs/modes."""
        key = detector_id.strip() if isinstance(detector_id, str) and detector_id.strip() else mode
        if key is None:
            key = self.detector_mode
        with self.reload_lock:
            entry = self._registered_detectors.get(key)
        if entry is None:
            raise ValueError(f"detector {key!r} is not registered")
        if mode is not None and entry["mode"] != normalize_detector_mode(mode):
            raise ValueError(f"detector_id {key!r} has mode {entry['mode']!r}, not {mode!r}")
        return self._apply_detector_entry(entry, labels=labels)

    def set_detector(
        self,
        mode: str,
        *,
        fn=None,
        model_path: str | Path | None = None,
        labels: Sequence[str] | None = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        output_format: str = "auto",
        detector_id: str | None = None,
    ) -> dict:
        """Trusted Python detector mode / backend switcher."""
        m = normalize_detector_mode(mode)
        eff_labels = list(labels) if labels is not None else self.detector_labels
        eff_model = str(model_path) if model_path is not None else self.detector_model
        backend = build_detector_backend(
            m,
            model_path=eff_model,
            labels=eff_labels,
            callable_fn=fn,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            output_format=output_format,
        )
        entry = {
            "mode": m,
            "backend": backend,
            "model": eff_model,
            "labels": list(eff_labels),
        }
        with self.reload_lock:
            self.detector_conf = float(conf_threshold)
            self.detector_iou = float(iou_threshold)
            self.detector_output_format = str(output_format)
            self._registered_detectors[m] = entry
            if detector_id:
                self._registered_detectors[str(detector_id)] = entry
        return self._apply_detector_entry(entry, labels=eff_labels)

    def request_reset(self, seed, dr: DomainRandomizationConfig | None = None):
        if dr is not None:
            out = self.link.call(("reset", seed, dr.to_dict()))
        else:
            out = self.link.call(("reset", seed))
        if "error" not in out:
            dr_info = out.get("dr") if isinstance(out, dict) else None
            eff_seed = dr_info["seed"] if isinstance(dr_info, dict) and "seed" in dr_info else seed
            dr_gen = dr_info.get("generation") if isinstance(dr_info, dict) else None
            self.apply_dr(eff_seed, dr_config=dr, generation=dr_gen)
            if self.dr_config.enabled and self.dr_summary is not None:
                out["dr"] = self.dr_summary
            else:
                out.pop("dr", None)
        return out

    def get_dr(self) -> dict:
        with self.reload_lock:
            return {
                "enabled": self.dr_config.enabled,
                "config": self.dr_config.to_dict(),
                "summary": self.dr_summary,
                "generation": self.dr_generation,
            }

    def set_dr(self, dr_config: DomainRandomizationConfig | dict | None, seed: int | None = None, *, use_seed: bool = False) -> dict:
        cfg = coerce_dr_config(dr_config)
        out = self.link.call(("dr_set", cfg.to_dict(), seed, use_seed))
        dr_info = out.get("dr") if isinstance(out, dict) else None
        if isinstance(dr_info, dict) and "seed" in dr_info:
            eff_seed = dr_info["seed"]
        elif use_seed:
            eff_seed = seed
        else:
            eff_seed = out.get("seed") if isinstance(out, dict) else None
        dr_gen = dr_info.get("generation") if isinstance(dr_info, dict) else None
        self.apply_dr(eff_seed, dr_config=cfg, generation=dr_gen)
        self._flush_delayed()
        return self.sim_state()

    def get_scene(self):
        return self.link.call(("scene_get",))

    def get_io(self):
        return self.link.call(("io_get",))

    def set_io(self, body):
        return self.link.call(("io_set", body))

    def _run_reload_op(self, req: dict, timeout: float = 5.0) -> str | None:
        done = req["done"]
        with self._reload_op_lock:
            with self.reload_lock:
                self._reload_req = req
            self.reload_wake_event.set()
            if not done.wait(timeout=timeout):
                with self.reload_lock:
                    if req.get("committed"):
                        # Swap already occurred atomically under reload_lock; wait for done without cancelling.
                        pass
                    elif not done.is_set():
                        req["cancelled"] = True
                        if self._reload_req is req:
                            self._reload_req = None
                        req["err_box"]["error"] = "camera reload timed out"
                if req.get("committed") and not done.is_set():
                    done.wait()
            return req["err_box"].get("error")

    def _close_staged_cam(self, staged: dict | None) -> None:
        if not staged:
            return
        for k in ("renderer", "seg_renderer", "state"):
            obj = staged.get(k)
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass

    def _stage_camera_tx(self, req: dict) -> None:
        import mujoco

        err_box = req["err_box"]
        shm_info = req["shm_info"]
        scene_ref = req["scene_ref"]
        tool_ref = req["tool_ref"]
        objects_str = req["objects_str"]
        scene_is_dir = bool(req.get("scene_is_dir", False))
        tool_is_dir = bool(req.get("tool_is_dir", False))
        tx_id = req["tx_id"]
        exp_sd = req.get("expected_scene_digest")
        exp_td = req.get("expected_tool_digest")
        verify_digests = bool(req.get("verify_digests", False))

        with self.reload_lock:
            if req.get("cancelled"):
                return

        new_renderer = None
        new_seg = None
        new_state = None
        try:
            new_model, new_data, new_scene, new_labels = self._build_camera_world(
                scene_ref,
                tool_ref,
                objects_str,
                scene_is_dir=scene_is_dir,
                tool_is_dir=tool_is_dir,
                expected_scene_digest=exp_sd,
                expected_tool_digest=exp_td,
                verify_digests=verify_digests,
            )
            w, h = self.size
            new_model.vis.global_.offwidth = max(new_model.vis.global_.offwidth, w)
            new_model.vis.global_.offheight = max(
                new_model.vis.global_.offheight, h, self._cam.DETECTION_SIZE[1] + 1
            )
            new_renderer = mujoco.Renderer(new_model, h, w)
            new_seg = mujoco.Renderer(
                new_model, self._cam.DETECTION_SIZE[1], self._cam.DETECTION_SIZE[0]
            )
            self._cam.check_renderer_internals(new_seg)
            new_state = SharedState(
                shm_info["nq"], shm_info["nv"], shm_info["name"], lock=self.state.lock
            )
            with self.reload_lock:
                if req.get("cancelled"):
                    self._close_staged_cam(
                        {"renderer": new_renderer, "seg_renderer": new_seg, "state": new_state}
                    )
                    return
                if self._staged_cam_tx is not None:
                    self._close_staged_cam(self._staged_cam_tx)
                self._staged_cam_tx = {
                    "tx_id": tx_id,
                    "model": new_model,
                    "data": new_data,
                    "scene": new_scene,
                    "labels": new_labels,
                    "renderer": new_renderer,
                    "seg_renderer": new_seg,
                    "state": new_state,
                    "scene_ref": scene_ref,
                    "tool_ref": tool_ref,
                    "objects_str": objects_str,
                }
        except Exception as exc:
            self._close_staged_cam(
                {"renderer": new_renderer, "seg_renderer": new_seg, "state": new_state}
            )
            err_box["error"] = f"camera stage failed: {exc}"
            if hasattr(exc, "code"):
                err_box["code"] = exc.code

    def _refresh_dr_for_current_model_locked(self) -> None:
        """Re-snapshot DR baseline from `self.model` after a scene/model commit under reload_lock."""
        self.dr = SimDomainRandomizer(self.model, self.dr_config)
        if self.dr_config.enabled:
            seed = self.dr_seed if self.dr_seed is not None else self.cfg.get("seed")
            summary = self.dr.apply(self.model, seed=seed)
            if self.dr_generation:
                summary["generation"] = self.dr_generation
            self.dr_summary = summary
            self._dr_latency_ms = float(summary.get("latency_ms", 0.0))
        else:
            self.dr_summary = None
            self._dr_latency_ms = 0.0

    def _snapshot_cam_gen_locked(self, state_override=None) -> dict:
        return {
            "tx_id": self.camera_tx_id,
            "state": state_override if state_override is not None else self.state,
            "model": self.model,
            "data": self.data,
            "scene": self.scene,
            "labels": self.labels,
            "renderer": getattr(self, "renderer", None),
            "seg_renderer": getattr(self, "seg_renderer", None),
            "scene_ref": self.cfg.get("scene"),
            "tool_ref": self.cfg.get("tool"),
            "objects_str": self.cfg.get("objects"),
            "dr": self.dr,
            "dr_summary": self.dr_summary,
            "dr_seed": self.dr_seed,
            "dr_generation": self.dr_generation,
            "dr_latency_ms": self._dr_latency_ms,
        }

    def _finish_cam_generation_swap_locked(self) -> None:
        self.physics_step = None
        self._clear_delayed_queue()
        self.perception.clear_latest()
        for stream in self.streams.values():
            with stream.lock:
                stream.want_idr = True

    def _restore_cam_gen_locked(self, prev: dict) -> dict:
        to_close = {
            "state": self.state,
            "renderer": getattr(self, "renderer", None),
            "seg_renderer": getattr(self, "seg_renderer", None),
        }
        self.state = prev["state"]
        self.model = prev["model"]
        self.data = prev["data"]
        self.scene = prev["scene"]
        self.labels = prev["labels"]
        self.renderer = prev["renderer"]
        self.seg_renderer = prev["seg_renderer"]
        self.cfg["scene"] = prev["scene_ref"]
        self.cfg["tool"] = prev["tool_ref"]
        self.cfg["objects"] = prev["objects_str"]
        self.dr = prev["dr"]
        self.dr_summary = prev["dr_summary"]
        self.dr_seed = prev["dr_seed"]
        self.dr_generation = prev["dr_generation"]
        self._dr_latency_ms = prev["dr_latency_ms"]
        self._finish_cam_generation_swap_locked()
        return to_close

    def _commit_staged_camera_tx(self, req: dict) -> None:
        err_box = req["err_box"]
        tx_id = req["tx_id"]
        with self.reload_lock:
            if req.get("cancelled"):
                err_box["error"] = "camera commit cancelled"
                return
            staged = self._staged_cam_tx
            if staged is None or staged.get("tx_id") != tx_id:
                err_box["error"] = f"camera tx {tx_id!r} not staged"
                return
            try:
                hook = getattr(self, "_before_camera_commit_hook", None)
                if hook is not None:
                    hook(staged)
                prev_gen = self._snapshot_cam_gen_locked()

                self.state = staged["state"]
                self.model = staged["model"]
                self.data = staged["data"]
                self.scene = staged["scene"]
                self.labels = staged["labels"]
                self.renderer = staged["renderer"]
                self.seg_renderer = staged["seg_renderer"]
                self.cfg["scene"] = staged["scene_ref"]
                self.cfg["tool"] = staged["tool_ref"]
                self.cfg["objects"] = staged["objects_str"]
                self._refresh_dr_for_current_model_locked()
                self._finish_cam_generation_swap_locked()
                self._prev_cam_gen = prev_gen
                self.camera_committed_tx_id = tx_id
                self._staged_cam_tx = None
                # Mark committed and done atomically under reload_lock before any teardown
                req["committed"] = True
                req["done"].set()
            except Exception as exc:
                err_box["error"] = f"camera commit failed: {exc}"
                return

    def _finalize_camera_tx(self, req: dict) -> None:
        prev = None
        with self.reload_lock:
            if req.get("cancelled"):
                return
            prev = self._prev_cam_gen
            self._prev_cam_gen = None
            self.camera_committed_tx_id = None
            req["done"].set()
        if prev is not None:
            self._close_staged_cam(prev)

    def _restore_prev_camera_tx(self, req: dict) -> None:
        err_box = req["err_box"]
        to_close = None
        with self.reload_lock:
            prev = self._prev_cam_gen
            if prev is None:
                err_box["error"] = "no previous camera generation to restore"
                return
            try:
                hook = getattr(self, "_before_camera_abort_hook", None)
                if hook is not None:
                    hook(prev)
                to_close = self._restore_cam_gen_locked(prev)
                self._prev_cam_gen = None
                self.camera_committed_tx_id = None
                req["done"].set()
            except Exception as exc:
                err_box["error"] = f"camera restore failed: {exc}"
                return
        if to_close is not None:
            self._close_staged_cam(to_close)

    def _abort_staged_camera_tx(self, req: dict) -> None:
        tx_id = req.get("tx_id")
        err_box = req["err_box"]
        to_close = None
        with self.reload_lock:
            if self._staged_cam_tx is not None and (
                tx_id is None or self._staged_cam_tx.get("tx_id") == tx_id
            ):
                try:
                    hook = getattr(self, "_before_camera_abort_hook", None)
                    if hook is not None:
                        hook(self._staged_cam_tx)
                except Exception as exc:
                    err_box["error"] = f"camera abort failed: {exc}"
                    return
                to_close = self._staged_cam_tx
                self._staged_cam_tx = None
            req["done"].set()
        if to_close is not None:
            self._close_staged_cam(to_close)

    def _close_legacy_prev_cam(self, req: dict) -> None:
        with self.reload_lock:
            prev = self._legacy_prev_cam
            self._legacy_prev_cam = None
            req["done"].set()
        if prev is not None:
            self._close_staged_cam(prev)

    def _restore_legacy_prev_cam(self, req: dict) -> None:
        to_close = None
        with self.reload_lock:
            prev = self._legacy_prev_cam
            self._legacy_prev_cam = None
            if prev is not None:
                to_close = self._restore_cam_gen_locked(prev)
            req["done"].set()
        if to_close is not None:
            self._close_staged_cam(to_close)

    @staticmethod
    def _strip_internal(res: dict) -> dict:
        return {k: v for k, v in res.items() if not k.startswith("_")}

    def _enter_degraded_split(self, tx_id: str, err_msg: str) -> tuple[int, dict]:
        self.activation_state = "DEGRADED_SPLIT"
        self.pending_tx_id = tx_id
        try:
            self.link.call(("tx_mark_degraded", {"tx_id": tx_id, "error": err_msg}), timeout=1.0)
        except Exception:
            pass
        return 503, {
            "error": err_msg,
            "code": "SIM_ACTIVATION_DEGRADED",
            "state": "DEGRADED_SPLIT",
            "tx_id": tx_id,
            "pending_tx_id": tx_id,
            "physics_tx_id": self.active_tx_id,
            "camera_tx_id": self.camera_tx_id,
        }

    def prepare_scene(self, body: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY", "state": "ABORTED"}
        try:
            if self.activation_state == "DEGRADED_SPLIT":
                return 503, {
                    "error": "SIM_ACTIVATION_DEGRADED",
                    "code": "SIM_ACTIVATION_DEGRADED",
                    "state": "DEGRADED_SPLIT",
                    "tx_id": self.pending_tx_id or self.active_tx_id,
                    "camera_tx_id": self.camera_tx_id,
                }
            res = self.link.call(("tx_prepare", body), timeout=8.0)
            status = int(res.get("_http_status", 200))
            if status != 200:
                return status, self._strip_internal(res)
            shm_info = res.get("_shm")
            if shm_info is not None:
                err_box: dict = {}
                err = self._run_reload_op(
                    {
                        "op": "stage",
                        "done": threading.Event(),
                        "tx_id": res["tx_id"],
                        "shm_info": shm_info,
                        "scene_ref": res.get("_scene_ref"),
                        "tool_ref": res.get("_tool_ref"),
                        "scene_is_dir": res.get("_scene_is_dir", False),
                        "tool_is_dir": res.get("_tool_is_dir", False),
                        "objects_str": res.get("_objects_spec", self.cfg.get("objects", "cube")),
                        "expected_scene_digest": res.get("_scene_digest"),
                        "expected_tool_digest": res.get("_tool_digest"),
                        "verify_digests": True,
                        "err_box": err_box,
                        "cancelled": False,
                    },
                    timeout=5.0,
                )
                if err:
                    try:
                        self.link.call(("tx_abort", {"tx_id": res["tx_id"]}), timeout=2.0)
                    except Exception:
                        pass
                    code = err_box.get("code", "CAMERA_PREPARE_FAILED")
                    ret_status = 400 if code == "DIGEST_MISMATCH" else 422
                    return ret_status, {"error": err, "code": code, "state": "ABORTED"}
            return 200, self._strip_internal(res)
        finally:
            self._tx_lock.release()

    def _finish_finalize(self, tx_id: str) -> tuple[int, dict]:
        try:
            fin = self.link.call(("tx_finalize", {"tx_id": tx_id}), timeout=1.0)
        except Exception as exc:
            return self._enter_degraded_split(tx_id, str(exc))
        fin_status = int(fin.get("_http_status", 200))
        if fin_status != 200 or "error" in fin:
            return self._enter_degraded_split(tx_id, str(fin.get("error", "finalize failed")))
        self.pending_cam_cleanup_tx_id = tx_id
        cam_err = self._run_reload_op(
            {"op": "finalize_cam", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
            timeout=2.0,
        )
        if cam_err is not None:
            return self._enter_degraded_split(tx_id, f"camera finalize failed: {cam_err}")
        self.pending_cam_cleanup_tx_id = None
        self.active_tx_id = tx_id
        self.camera_tx_id = tx_id
        self.pending_tx_id = None
        self.activation_state = "COMMITTED"
        return 200, self._strip_internal(fin)

    def _rollback_camera_for_tx(self, tx_id: str) -> tuple[int, dict] | None:
        if self.camera_committed_tx_id == tx_id and self._prev_cam_gen is not None:
            cam_op = "restore_prev_cam"
        elif self._staged_cam_tx is not None and self._staged_cam_tx.get("tx_id") == tx_id:
            cam_op = "abort_staged"
        else:
            return None
        abort_err = self._run_reload_op(
            {"op": cam_op, "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
            timeout=3.0,
        )
        if abort_err:
            # M04: Keep physics rollback snapshot intact so retry succeeds cleanly
            return 503, {
                "error": abort_err,
                "code": "CAMERA_ABORT_FAILED",
                "state": "DEGRADED_SPLIT",
                "tx_id": self.active_tx_id,
                "camera_tx_id": self.camera_tx_id,
            }
        return None

    def commit_scene(self, body: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY"}
        try:
            if not isinstance(body, dict):
                return 400, {"error": "request body must be a JSON object", "code": "INVALID_REQUEST"}
            tx_id = body.get("tx_id")
            if not isinstance(tx_id, str) or not tx_id.strip():
                return 400, {"error": "tx_id must be a non-empty string", "code": "INVALID_TX_ID"}
            tx_id = tx_id.strip()
            phase = body.get("phase", "all")
            if not isinstance(phase, str) or phase not in {"all", "camera_only", "rollback_physics"}:
                return 400, {"error": f"invalid commit phase: {phase!r}", "code": "INVALID_PHASE"}

            if (
                phase in {"all", "camera_only"}
                and tx_id == self.active_tx_id
                and self.camera_tx_id == tx_id
                and self.activation_state == "COMMITTED"
            ):
                res = self.link.call(("tx_commit", body), timeout=3.0)
                return int(res.get("_http_status", 200)), self._strip_internal(res)

            if self.activation_state == "DEGRADED_SPLIT":
                expected_tx = self.pending_tx_id if self.pending_tx_id is not None else self.active_tx_id
                if tx_id != expected_tx:
                    if phase == "all":
                        return 503, {
                            "error": "SIM_ACTIVATION_DEGRADED",
                            "code": "SIM_ACTIVATION_DEGRADED",
                            "state": "DEGRADED_SPLIT",
                            "tx_id": self.pending_tx_id or self.active_tx_id,
                            "camera_tx_id": self.camera_tx_id,
                        }
                    return 409, {"error": f"tx {tx_id!r} is not in DEGRADED_SPLIT", "code": "TX_NOT_PREPARED"}

                # ALWAYS query physics state FIRST before mutating or rolling back camera
                phys = self.link.call(("state",), timeout=2.0)
                staged_tx_id = phys.get("staged_tx_id")
                staged_committed = bool(phys.get("staged_physics_committed", False))

                # (a) Physics already finalized tx_id
                if staged_tx_id is None and phys.get("tx_id") == tx_id and phys.get("camera_tx_id") == tx_id:
                    if phase == "rollback_physics":
                        return 409, {
                            "error": f"tx {tx_id!r} is already committed and cannot be rolled back",
                            "code": "TX_ALREADY_COMMITTED",
                        }
                    if self.camera_committed_tx_id != tx_id and self._staged_cam_tx is not None:
                        err = self._run_reload_op(
                            {"op": "commit_staged", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
                            timeout=5.0,
                        )
                        if err:
                            return self._enter_degraded_split(tx_id, err)
                    return self._finish_finalize(tx_id)

                # (b) Physics committed tx_id (awaiting finalize or rollback_physics)
                if staged_tx_id == tx_id and staged_committed:
                    if phase in {"camera_only", "all"}:
                        if self.camera_committed_tx_id != tx_id:
                            err = self._run_reload_op(
                                {"op": "commit_staged", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
                                timeout=5.0,
                            )
                            if err:
                                return self._enter_degraded_split(tx_id, err)
                        return self._finish_finalize(tx_id)
                    cam_err = self._rollback_camera_for_tx(tx_id)
                    if cam_err is not None:
                        return cam_err
                    res = self.link.call(("tx_rollback_physics", {"tx_id": tx_id}), timeout=3.0)
                    status = int(res.get("_http_status", 200))
                    if status != 200 or "error" in res:
                        return (status if status != 200 else 500), self._strip_internal(res)
                    self.active_tx_id = res.get("restored_tx_id")
                    self.camera_tx_id = res.get("camera_tx_id")
                    self.camera_committed_tx_id = None
                    self.pending_tx_id = None
                    self.activation_state = "COMMITTED"
                    self.perception.clear_latest()
                    return 200, self._strip_internal(res)

                # (c) Physics has NOT committed tx_id
                if staged_tx_id == tx_id and not staged_committed:
                    if phase == "camera_only":
                        return 409, {
                            "error": f"tx {tx_id!r} physics has not been committed",
                            "code": "PHYSICS_NOT_COMMITTED",
                        }
                    if phase == "rollback_physics":
                        cam_err = self._rollback_camera_for_tx(tx_id)
                        if cam_err is not None:
                            return cam_err
                        abort_res = self.link.call(("tx_abort", {"tx_id": tx_id}), timeout=3.0)
                        a_status = int(abort_res.get("_http_status", 200))
                        if a_status != 200:
                            return a_status, self._strip_internal(abort_res)
                        self.active_tx_id = phys.get("tx_id")
                        self.camera_tx_id = phys.get("camera_tx_id")
                        self.camera_committed_tx_id = None
                        self.pending_tx_id = None
                        self.activation_state = "COMMITTED"
                        self.perception.clear_latest()
                        return 200, {
                            "ok": True,
                            "state": "ABORTED",
                            "tx_id": tx_id,
                            "restored_tx_id": self.active_tx_id,
                            "camera_tx_id": self.camera_tx_id,
                            "robot": phys.get("robot"),
                            "scene": phys.get("scene"),
                            "tool": phys.get("tool"),
                            "scene_digest": phys.get("scene_digest"),
                            "tool_digest": phys.get("tool_digest"),
                        }
                    # phase == "all": fall through to normal commit_physics -> commit_staged -> finalize
                # (d) Physics never staged or applied tx_id (lost NACK or already rolled back)
                elif staged_tx_id != tx_id and phys.get("tx_id") != tx_id:
                    if phase == "rollback_physics":
                        cam_err = self._rollback_camera_for_tx(tx_id)
                        if cam_err is not None:
                            return cam_err
                        if phys.get("activation_state") == "DEGRADED_SPLIT":
                            abort_res = self.link.call(("tx_abort", {"tx_id": tx_id}), timeout=3.0)
                            a_status = int(abort_res.get("_http_status", 200))
                            if a_status != 200:
                                return a_status, self._strip_internal(abort_res)
                        self.active_tx_id = phys.get("tx_id")
                        self.camera_tx_id = phys.get("camera_tx_id")
                        self.camera_committed_tx_id = None
                        self.pending_tx_id = None
                        self.pending_cam_cleanup_tx_id = None
                        self.activation_state = "COMMITTED"
                        self.perception.clear_latest()
                        out = {
                            "ok": True,
                            "state": "ABORTED",
                            "tx_id": tx_id,
                            "restored_tx_id": self.active_tx_id,
                            "camera_tx_id": self.camera_tx_id,
                            "robot": phys.get("robot"),
                            "scene": phys.get("scene"),
                            "tool": phys.get("tool"),
                            "scene_digest": phys.get("scene_digest"),
                            "tool_digest": phys.get("tool_digest"),
                        }
                        if staged_tx_id is None and phys.get("activation_state") == "COMMITTED":
                            out["idempotent"] = True
                        return 200, out
                    return 409, {"error": f"tx {tx_id!r} is not prepared", "code": "TX_NOT_PREPARED"}
                else:
                    if phase in {"camera_only", "rollback_physics"}:
                        return 409, {"error": f"tx {tx_id!r} is not in DEGRADED_SPLIT", "code": "TX_NOT_PREPARED"}

            if phase in {"rollback_physics", "camera_only"}:
                return 409, {"error": f"tx {tx_id!r} is not in DEGRADED_SPLIT", "code": "TX_NOT_PREPARED"}

            # phase == "all"
            # M03: Gate frame/video publication BEFORE physics commit
            prev_state = self.activation_state
            self.activation_state = "COMMITTING"
            try:
                phys = self.link.call(("tx_commit_physics", {"tx_id": tx_id}), timeout=3.0)
            except Exception as exc:
                # Reply lost/ambiguous: enter DEGRADED_SPLIT retaining pending_tx_id so recovery reconciles
                return self._enter_degraded_split(tx_id, f"physics commit failed: {exc}")
            p_status = int(phys.get("_http_status", 200))
            if p_status != 200:
                self.activation_state = prev_state
                if p_status == 500:
                    self._run_reload_op(
                        {"op": "abort_staged", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
                        timeout=3.0,
                    )
                return p_status, self._strip_internal(phys)

            err = self._run_reload_op(
                {"op": "commit_staged", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
                timeout=5.0,
            )
            if err:
                return self._enter_degraded_split(tx_id, err)

            return self._finish_finalize(tx_id)
        finally:
            self._tx_lock.release()

    def abort_scene(self, body: dict) -> tuple[int, dict]:
        if not self._tx_lock.acquire(blocking=False):
            return 409, {"error": "transaction in progress", "code": "TX_BUSY"}
        try:
            if not isinstance(body, dict):
                return 400, {"error": "request body must be a JSON object", "code": "INVALID_REQUEST"}
            tx_id = body.get("tx_id")
            if not isinstance(tx_id, str) or not tx_id.strip():
                return 400, {"error": "tx_id must be a non-empty string", "code": "MISSING_TX_ID"}
            tx_id = tx_id.strip()
            if self.activation_state == "DEGRADED_SPLIT":
                return 503, {
                    "error": "SIM_ACTIVATION_DEGRADED",
                    "code": "SIM_ACTIVATION_DEGRADED",
                    "state": "DEGRADED_SPLIT",
                    "tx_id": self.pending_tx_id or self.active_tx_id,
                    "camera_tx_id": self.camera_tx_id,
                }
            res = self.link.call(("tx_abort", {"tx_id": tx_id}), timeout=3.0)
            status = int(res.get("_http_status", 200))
            if status == 200:
                self._run_reload_op(
                    {"op": "abort_staged", "done": threading.Event(), "tx_id": tx_id, "err_box": {}, "cancelled": False},
                    timeout=3.0,
                )
                if self.pending_tx_id == tx_id:
                    self.pending_tx_id = None
            return status, self._strip_internal(res)
        finally:
            self._tx_lock.release()

    def load_scene(self, body):
        if not self._tx_lock.acquire(blocking=False):
            return {"_http_status": 409, "error": "transaction in progress", "code": "TX_BUSY"}
        try:
            res = self.link.call(("scene_prepare", body), timeout=8.0)
            if "error" in res:
                return res
            shm_info = res.pop("_shm", None)
            scene_ref = res.pop("_scene_ref", None)
            tool_ref = res.pop("_tool_ref", None)
            objects_str = res.pop("_objects_spec", self.cfg.get("objects", "cube"))
            swapped_cam = False
            if shm_info is not None:
                err = self._run_reload_op(
                    {
                        "done": threading.Event(),
                        "shm_info": shm_info,
                        "scene_ref": scene_ref,
                        "tool_ref": tool_ref,
                        "objects_str": objects_str,
                        "defer_close": True,
                        "err_box": {},
                        "cancelled": False,
                    },
                    timeout=5.0,
                )
                if err:
                    try:
                        self.link.call(("scene_abort",), timeout=2.0)
                    except Exception:
                        pass
                    return {"error": err}
                swapped_cam = True
            try:
                commit_res = self.link.call(("scene_commit",), timeout=3.0)
            except Exception as exc:
                if swapped_cam:
                    self._run_reload_op(
                        {"op": "restore_legacy_prev", "done": threading.Event(), "err_box": {}, "cancelled": False},
                        timeout=3.0,
                    )
                    try:
                        self.link.call(("scene_abort",), timeout=2.0)
                    except Exception:
                        pass
                return {"error": f"scene commit failed: {exc}"}
            if "error" in commit_res:
                if swapped_cam:
                    self._run_reload_op(
                        {"op": "restore_legacy_prev", "done": threading.Event(), "err_box": {}, "cancelled": False},
                        timeout=3.0,
                    )
                    try:
                        self.link.call(("scene_abort",), timeout=2.0)
                    except Exception:
                        pass
                return commit_res
            if swapped_cam:
                self._run_reload_op(
                    {"op": "close_legacy_prev", "done": threading.Event(), "err_box": {}, "cancelled": False},
                    timeout=2.0,
                )
            self.active_tx_id = commit_res.get("tx_id")
            self.camera_tx_id = commit_res.get("camera_tx_id")
            self.activation_state = "COMMITTED"
            return commit_res
        finally:
            self._tx_lock.release()

    def _apply_reload(self, req: dict | tuple) -> None:
        import mujoco

        if isinstance(req, dict):
            shm_info = req["shm_info"]
            scene_ref = req["scene_ref"]
            tool_ref = req["tool_ref"]
            objects_str = req["objects_str"]
            err_box = req["err_box"]
            defer_close = bool(req.get("defer_close", False))
        else:
            shm_info, scene_ref, tool_ref, objects_str, err_box = req
            req = {"cancelled": False}
            defer_close = False

        new_renderer = None
        new_seg = None
        new_state = None
        old_to_close = None
        try:
            new_model, new_data, new_scene, new_labels = self._build_camera_world(
                scene_ref, tool_ref, objects_str
            )
            w, h = self.size
            new_model.vis.global_.offwidth = max(new_model.vis.global_.offwidth, w)
            new_model.vis.global_.offheight = max(
                new_model.vis.global_.offheight, h, self._cam.DETECTION_SIZE[1] + 1
            )

            new_renderer = mujoco.Renderer(new_model, h, w)
            new_seg = mujoco.Renderer(
                new_model, self._cam.DETECTION_SIZE[1], self._cam.DETECTION_SIZE[0]
            )
            self._cam.check_renderer_internals(new_seg)

            old_state = self.state
            new_state = SharedState(
                shm_info["nq"], shm_info["nv"], shm_info["name"], lock=old_state.lock
            )

            with self.reload_lock:
                if req.get("cancelled"):
                    self._close_staged_cam(
                        {"renderer": new_renderer, "seg_renderer": new_seg, "state": new_state}
                    )
                    return

                prev_cam = self._snapshot_cam_gen_locked(state_override=old_state)

                self.state = new_state
                self.model, self.data, self.scene, self.labels = (
                    new_model,
                    new_data,
                    new_scene,
                    new_labels,
                )
                self.renderer, self.seg_renderer = new_renderer, new_seg
                self.cfg["scene"], self.cfg["tool"], self.cfg["objects"] = (
                    scene_ref,
                    tool_ref,
                    objects_str,
                )
                self._refresh_dr_for_current_model_locked()
                self._finish_cam_generation_swap_locked()
                if defer_close:
                    self._legacy_prev_cam = prev_cam
                else:
                    old_to_close = prev_cam
                if isinstance(req, dict):
                    req["committed"] = True
                    if "done" in req:
                        req["done"].set()
            if old_to_close is not None:
                self._close_staged_cam(old_to_close)
        except Exception as exc:
            self._close_staged_cam(
                {"renderer": new_renderer, "seg_renderer": new_seg, "state": new_state}
            )
            err_box["error"] = f"camera reload failed: {exc}"

    def sim_state(self):
        """The simulator's state plus the camera's: the newest frame's physics time and how far
        it is behind the physics now (`stale` past two camera periods)."""
        out = self.link.call(("state",))
        with self.perception.cond:
            frame = self.perception.frame
        lag = float(self.state.arr[1]) - frame["physics_time"] if frame else None
        if lag is not None and self._dr_latency_ms > 0.0 and frame is not None:
            lag = max(lag, float(frame.get("delivery_delay_s", 0.0)))
        cam_body = {
            "frame_id": frame["frame_id"] if frame else 0,
            "physics_time": frame["physics_time"] if frame else None,
            "physics_step": frame["physics_step"] if frame else None,
            "lag_s": None if lag is None else round(lag, 6),
            "skipped": self.skipped,
            "seg_failures": self.seg_failures,
            "seg_last_error": self.seg_last_error,
            "stale": frame is None or lag > 2.0 / self.fps,
        }
        if frame is not None and self._dr_latency_ms > 0.0:
            cam_body["age_ms"] = int((time.monotonic() - frame["mono"]) * 1000)
            cam_body["delivery_delay_ms"] = round(float(frame.get("delivery_delay_s", 0.0)) * 1000.0, 3)
        out["camera"] = cam_body
        out["detector_mode"] = self.detector_mode
        if self.activation_state == "DEGRADED_SPLIT":
            out["activation_state"] = "DEGRADED_SPLIT"
            out["degraded"] = True
            out["pending_tx_id"] = self.pending_tx_id
            if self.pending_tx_id and not out.get("tx_id"):
                out["tx_id"] = self.pending_tx_id
        if self.pending_cam_cleanup_tx_id is not None:
            out["pending_cam_cleanup_tx_id"] = self.pending_cam_cleanup_tx_id
        if self.dr_config.enabled and self.dr_summary is not None:
            out["dr"] = self.dr_summary
        else:
            out.pop("dr", None)
        return out

    def camera_info(self) -> dict:
        cam, m = self._cam, self.mount
        r = cam.mount_rotation(self.model, m)
        out_board = None
        if self.board is not None:
            rb, ob = self.board.pose()
            out_board = {"x": self.board.x, "y": self.board.y, "yaw_deg": self.board.yaw_deg, "z": self.board.z,
                         "side_m": self.board.side_m, "gap_m": self.board.gap_m, "noise_px": self.board_noise[0],
                         "R_base_board": rb.round(9).tolist(), "origin": ob.tolist()}
        return {
            "camera_mount": {
                "flip": m.flip,
                "rot_deg": m.rot_deg,
                "tilt_deg": m.tilt_deg,
                "offset_m": [round(float(v), 5) for v in m.offset],
                "q_end_cam": list(m.q_end_cam) if m.q_end_cam is not None else list(cam.r_to_quat(r)),
                "q_end_cam_set": m.q_end_cam is not None,
            },
            "R_end_cam": r.round(9).tolist(),
            "intrinsics": {"fx": round(m.focal(), 4), "fy": round(m.focal(), 4), "cx": 320.0, "cy": 180.0,
                           "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360},
            "aruco_board": out_board,
            "hfov_deg": round(m.hfov_deg, 3),
            "vfov_deg": round(m.vfov_deg, 3),
            "stream_size": list(self.size),
            "fps": self.fps,
            "detection_size": list(cam.DETECTION_SIZE),
            "fx_px": round(m.focal(), 2),
            "grip_pixel_norm_at": {
                f"{h:.2f}": cam.grip_pixel_norm(r, m.offset, h, m.focal())
                for h in (0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30)
            },
            "render_backend": self.backend,
            "render": self.stats(),
            "depth": {
                "kind": self.depth_model.kind,
                "scale": self.depth_model.scale,
                "noise_rel": self.depth_model.noise_rel,
                "a": self.depth_model.a,
                "b": self.depth_model.b,
                "conf": self.depth_model.conf,
                "grid": list(cam.DEPTH_GRID),
                "failures": self.depth_failures,
            },
        }

    def setup(self) -> None:
        """Create the renderers and encoders (raises when no GL backend or libx264 works)."""
        import mujoco

        w, h = self.size
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, w)
        # + 1: render_segmentation's canary row above the detection image
        self.model.vis.global_.offheight = max(
            self.model.vis.global_.offheight, h, self._cam.DETECTION_SIZE[1] + 1
        )
        self.renderer = mujoco.Renderer(self.model, h, w)
        # plain renderer: render_segmentation sets the ID-colour flags, checks a canary pixel and decodes with bounds checks
        self.seg_renderer = mujoco.Renderer(
            self.model, self._cam.DETECTION_SIZE[1], self._cam.DETECTION_SIZE[0]
        )
        self._cam.check_renderer_internals(
            self.seg_renderer
        )  # fail at start-up, not on the first frame
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0
        if self.cfg["has_meshes"]:
            self.opt.geomgroup[3] = (
                0  # box primitives hidden behind the upstream meshes
            )
        self.encoders = {
            path: self._cam.H264Encoder(w, h, self.fps) for path in self.streams
        }
        # one RGB and one segmentation render succeed before readiness is reported
        self.state.read(self.data)
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="wrist", scene_option=self.opt)
        self.renderer.render()
        while self.detect() is None:  # raises after SEG_MAX_CONSECUTIVE failures
            pass

    SEG_MAX_CONSECUTIVE = 10

    def detect(self, want_depth: bool = False, physics_step: int = 0) -> list[dict] | None:
        """Detections of the wrist view of self.data using either ground truth (`gt`) or real-detector
        evaluation backend (`onnx` / `callable`). Returns None when segmentation fails in GT mode."""
        self.frame_depth = None
        r = self.seg_renderer
        r.update_scene(self.data, camera="wrist", scene_option=self.opt)
        if self.detector_mode != "gt":
            rgb = np.ascontiguousarray(r.render(), dtype=np.uint8)
            t0 = time.perf_counter()
            raw_dets = self._detector_backend(rgb) if self._detector_backend is not None else []
            self.last_inference_ms = round((time.perf_counter() - t0) * 1000.0, 3)
            dets = [
                normalize_detection(d, index=i, labels=self.detector_labels)
                for i, d in enumerate(raw_dets or ())
            ]
            if want_depth and self.depth_model.enabled:
                t_d = time.perf_counter()
                z = self._cam.render_depth(r)
                d = depth_result_from_boxes(
                    z,
                    dets,
                    self.depth_model,
                    self.frame_id + 1,
                    fx_per_w=self.mount.focal() / self._cam.DETECTION_SIZE[0],
                )
                if d["finite"]:
                    self.frame_depth = {**d, "physics_step": physics_step}
                else:
                    self.depth_failures += 1
                    print(
                        f"rebot-sim camera: depth render not finite on frame {self.frame_id + 1}",
                        file=sys.stderr,
                        flush=True,
                    )
                self.depth_ms = (self.depth_ms + [(time.perf_counter() - t_d) * 1000])[-100:]
            return dets
        try:
            ids = self._cam.render_segmentation(r)
        except Exception as exc:  # SegmentationError (bad readback) or a GL error
            self.seg_failures += 1
            self.seg_consecutive += 1
            self.seg_last_error = repr(exc)
            print(
                f"rebot-sim camera: segmentation failed on frame {self.frame_id + 1} "
                f"({self.seg_consecutive} in a row, {self.seg_failures} total): {exc!r}; "
                f"diag {getattr(exc, 'diag', {})} nbody {self.model.nbody} ngeom {self.model.ngeom} "
                f"scene.ngeom {r.scene.ngeom}",
                file=sys.stderr,
                flush=True,
            )
            if self.seg_consecutive >= self.SEG_MAX_CONSECUTIVE:
                raise RuntimeError(
                    f"segmentation failed on {self.seg_consecutive} consecutive frames: {exc!r}"
                ) from exc
            return None
        self.seg_consecutive = 0
        self.last_inference_ms = 0.1
        if want_depth and self.depth_model.enabled:
            t0 = time.perf_counter()
            z = self._cam.render_depth(r)  # same mjvScene as the segmentation: same physics state
            d = self._cam.depth_result(ids, z, self.scene, self.depth_model, self.frame_id + 1,
                                       fx_per_w=self.mount.focal() / self._cam.DETECTION_SIZE[0])
            if d["finite"]:
                self.frame_depth = {**d, "physics_step": physics_step}
            else:
                self.depth_failures += 1
                print(f"rebot-sim camera: depth render not finite on frame {self.frame_id + 1}", file=sys.stderr, flush=True)
            self.depth_ms = (self.depth_ms + [(time.perf_counter() - t0) * 1000])[-100:]
        return self._cam.detections(ids, self.scene)

    def board_markers(self, frame_id: int) -> list[dict]:
        """Markers of the virtual board seen by the wrist camera of self.data (truth model)."""
        if self.board is None:
            return []
        from .aruco import visible_markers

        cid = self.model.camera("wrist").id
        r_mj = self.data.cam_xmat[cid].reshape(3, 3)
        r_cv = np.column_stack([r_mj[:, 0], -r_mj[:, 1], -r_mj[:, 2]])
        noise, seed = self.board_noise
        return visible_markers(self.board, r_cv, self.data.cam_xpos[cid].copy(), self.mount.focal(),
                               noise, seed, frame_id)

    def _effective_latency_ms(self) -> float:
        noise_obj = getattr(self, "noise", None)
        noise_ms = float(getattr(noise_obj, "latency_ms", 0.0) or 0.0)
        dr_ms = float(getattr(self, "_dr_latency_ms", 0.0) or 0.0)
        return noise_ms + dr_ms

    def _flush_delayed(self) -> None:
        latency_s = CameraRig._effective_latency_ms(self) / 1000.0
        now = time.monotonic()
        ready: list[tuple] = []
        lock = getattr(self, "_delayed_lock", None)
        if lock is not None:
            with lock:
                cur_gen = getattr(self, "_detector_generation", 0)
                gens = getattr(self, "_delayed_gens", None)
                while self.delayed and now - self.delayed[0][2] >= latency_s:
                    frame = self.delayed.pop(0)
                    f_gen = gens.pop(0) if gens else cur_gen
                    if f_gen == cur_gen:
                        ready.append(frame)
        else:
            while self.delayed and now - self.delayed[0][2] >= latency_s:
                ready.append(self.delayed.pop(0))
        for frame in ready:
            self.perception.on_frame(*frame)

    def _next_delayed_due_t(self) -> float | None:
        latency_s = CameraRig._effective_latency_ms(self) / 1000.0
        lock = getattr(self, "_delayed_lock", None)
        if lock is not None:
            with lock:
                if not self.delayed:
                    return None
                captured = self.delayed[0][2]
        else:
            if not self.delayed:
                return None
            captured = self.delayed[0][2]
        rem_s = max(0.0, latency_s - (time.monotonic() - captured))
        return time.perf_counter() + rem_s

    def publish_frame(self, frame: tuple, detector_generation: int | None = None) -> None:
        """Hand a frame (on_frame's arguments) to perception, latency_ms after its capture:
        the frame keeps its capture time, so its age_ms includes the delay."""
        latency_ms = CameraRig._effective_latency_ms(self)
        if latency_ms <= 0:
            if detector_generation is not None and detector_generation != getattr(self, "_detector_generation", 0):
                return
            self.perception.on_frame(*frame)
            return
        lock = getattr(self, "_delayed_lock", None)
        if lock is not None:
            with lock:
                cur_gen = getattr(self, "_detector_generation", 0)
                if detector_generation is not None and detector_generation != cur_gen:
                    return
                self.delayed.append(frame)
                if hasattr(self, "_delayed_gens"):
                    self._delayed_gens.append(cur_gen if detector_generation is None else detector_generation)
        else:
            self.delayed.append(frame)
        CameraRig._flush_delayed(self)

    def process_once(self) -> bool:
        """Read latest shared state, render/detect/publish one frame if physics stepped; returns True if rendered."""
        import mujoco

        t0 = time.perf_counter()
        self._flush_delayed()
        with self.reload_lock:
            renderer = self.renderer
            opt = self.opt
            encoders = self.encoders
            det_gen = self._detector_generation
            det_mode = self.detector_mode
            step = self.state.read(self.data)
            if (
                step is None or step == self.physics_step
            ):  # no new physics: publish nothing
                self.skipped += 1
                return False
            if self.activation_state in {"DEGRADED_SPLIT", "COMMITTING"}:
                # Never publish mixed-generation detections or video frames while degraded/committing
                self.physics_step = step
                return False
            self.physics_step = step
            captured = time.monotonic()
            ts_ms = int(time.time() * 1000)
            mujoco.mj_forward(self.model, self.data)
            dets = self.detect(self.perception.wants_depth(captured), step)
            self.frame_id += 1
            if (
                dets is not None
            ):  # a failed segmentation: the video frame goes out, no result
                if det_mode == "gt":
                    dets = self._cam.perturb(
                        dets, self.noise, self.frame_id, self.labels
                    )
                self.publish_frame(
                    (self.frame_id, ts_ms, captured, dets, self.data.time, step, self.frame_depth,
                     self.board_markers(self.frame_id), self.last_inference_ms),
                    detector_generation=det_gen,
                )
            for path, stream in self.streams.items():
                if not stream.active:
                    continue
                renderer.update_scene(
                    self.data,
                    camera="wrist" if path == "/" else "scene",
                    scene_option=opt,
                )
                au = encoders[path].encode(
                    renderer.render(), keyframe=stream.take_idr_request()
                )
                if au:
                    stream.publish(au, ts_ms, 5 in _nal_types(au))
            self.render_ms = (self.render_ms + [(time.perf_counter() - t0) * 1000])[
                -100:
            ]
            self.state.arr[2], self.state.arr[3] = (
                self.frame_id,
                float(np.mean(self.render_ms)),
            )
            return True

    def run(self, parent_pid: int) -> None:
        reload_ops = {
            "stage": self._stage_camera_tx,
            "commit_staged": self._commit_staged_camera_tx,
            "finalize_cam": self._finalize_camera_tx,
            "restore_prev_cam": self._restore_prev_camera_tx,
            "abort_staged": self._abort_staged_camera_tx,
            "close_legacy_prev": self._close_legacy_prev_cam,
            "restore_legacy_prev": self._restore_legacy_prev_cam,
        }
        period = 1.0 / self.fps
        next_capture_t = time.perf_counter()
        while os.getppid() == parent_pid:
            req = None
            with self.reload_lock:
                if self._reload_req is not None:
                    req, self._reload_req = self._reload_req, None
            if req is not None:
                if isinstance(req, dict) and "op" in req:
                    if not req.get("cancelled"):
                        handler = reload_ops.get(req["op"])
                        if handler is not None:
                            handler(req)
                    req["done"].set()
                elif isinstance(req, dict):
                    self._apply_reload(req)
                    req["done"].set()
                else:
                    done, shm_info, scene_ref, tool_ref, objects_str, err_box = req
                    self._apply_reload(req)
                    done.set()

            now = time.perf_counter()
            if now >= next_capture_t:
                self.process_once()
                next_capture_t += period
                after = time.perf_counter()
                if next_capture_t <= after:
                    next_capture_t = after + period
            else:
                self._flush_delayed()

            now = time.perf_counter()
            wake_t = next_capture_t
            next_delivery_t = self._next_delayed_due_t()
            if next_delivery_t is not None:
                wake_t = min(wake_t, next_delivery_t)
            if wake_t > now:
                self.reload_wake_event.wait(timeout=wake_t - now)
                self.reload_wake_event.clear()

    def stats(self) -> dict:
        ms = np.array(self.render_ms) if self.render_ms else np.zeros(1)
        out = {
            "frames": self.frame_id,
            "frame_ms_mean": round(float(ms.mean()), 2),
            "frame_ms_max": round(float(ms.max()), 2),
        }
        if self.depth_ms:
            out["depth_ms_mean"] = round(float(np.mean(self.depth_ms)), 2)
            out["depth_frames"] = len(self.depth_ms)
        return out


def camera_main(
    cfg: dict, shm_name: str, lock, nq: int, nv: int, conn, ready_conn, parent_pid: int
) -> None:
    """Camera process entry: build the rig with its renderers and encoders, start the servers,
    then report readiness and render."""
    state = SharedState(nq, nv, shm_name, lock)
    try:
        rig = CameraRig(cfg, state, SimLink(conn))
        rig.setup()  # renderers and encoders before readiness is reported
        serve_video(cfg["host"], cfg["video_port"], rig.streams)
        PerceptionServer(rig.perception, rig, cfg["host"], cfg["perception_port"])
    except Exception as exc:  # no GL backend, no libx264, port in use
        ready_conn.send({"error": f"camera init failed: {exc!r}"})
        return
    ready_conn.send({"backend": rig.backend})
    rig.run(parent_pid)


def _nal_types(au: bytes) -> list[int]:
    out, i = [], au.find(b"\x00\x00\x01")
    while 0 <= i < len(au) - 3:
        out.append(au[i + 3] & 0x1F)
        i = au.find(b"\x00\x00\x01", i + 3)
    return out
