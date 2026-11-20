"""Model-level dynamic collision threshold calibration and 30s factory self-check.

Fits per-joint dynamic residual models on obstacle-free multi-pose / multi-speed
excitation trajectories:

    tau_dyn = tau_gravity + f_c * sign_smooth(dq) + f_v * dq + I_eff * ddq

with physical non-negativity constraints (f_c >= 0, f_v >= 0, I_eff >= 0).
Produces profile ``collision`` patch candidates (low / medium / high tiers,
``persist_ticks``, ``calibrated: true``), ``sim_backfill`` suggestions
(``frictionloss``, ``damping``, ``armature``), and evaluates single-unit
unloaded factory self-check runs into ``PASS`` / ``WARN`` / ``FAIL`` verdicts.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

ARM_JOINTS = 6
DEFAULT_V_EPS = 0.01
DEFAULT_SAFETY_FACTOR = 1.5
DEFAULT_FLOOR_NM = 0.1
DEFAULT_PERSIST_TICKS = 4
DEFAULT_CLEAR_RATIO = 0.5
# Matches armd Sensitivity::factor() in src/collision.rs: High=0.7, Medium=1.0, Low=1.5.
DEFAULT_HIGH_SCALE = 0.7
DEFAULT_LOW_SCALE = 1.5
DEFAULT_WARN_RATIO = 0.80
DEFAULT_FAIL_RATIO = 0.95
MAX_PERSIST_TICKS = 1000


class CollisionCalibError(ValueError):
    """Fail-closed validation or calibration error."""


def _is_bool(val: Any) -> bool:
    return isinstance(val, (bool, np.bool_))


def _require_positive_finite_scalar(name: str, val: Any, *, min_val: float = 0.0, strict_gt: bool = True) -> float:
    if _is_bool(val) or not isinstance(val, (int, float, np.integer, np.floating)):
        raise CollisionCalibError(f"{name} must be a real number, got {type(val).__name__}")
    out = float(val)
    if not math.isfinite(out):
        raise CollisionCalibError(f"{name} must be finite, got {out}")
    if strict_gt:
        if out <= min_val:
            raise CollisionCalibError(f"{name} must be > {min_val}, got {out}")
    elif out < min_val:
        raise CollisionCalibError(f"{name} must be >= {min_val}, got {out}")
    return out


def _require_persist_ticks(val: Any) -> int:
    if _is_bool(val) or not isinstance(val, (int, np.integer)):
        raise CollisionCalibError(f"persist_ticks must be an integer, got {type(val).__name__}")
    ticks = int(val)
    if ticks < 1 or ticks > MAX_PERSIST_TICKS:
        raise CollisionCalibError(f"persist_ticks must be in 1..={MAX_PERSIST_TICKS}, got {ticks}")
    return ticks


def _as_2d_float_array(name: str, val: Any) -> np.ndarray:
    arr = np.asarray(val)
    if arr.dtype == object or np.issubdtype(arr.dtype, np.bool_) or not (
        np.issubdtype(arr.dtype, np.number)
    ):
        raise CollisionCalibError(f"{name} must be a numeric 2D array, got dtype {arr.dtype}")
    if np.issubdtype(arr.dtype, np.complexfloating):
        raise CollisionCalibError(f"{name} must be real-valued, got complex dtype {arr.dtype}")
    arr = arr.astype(float, copy=False)
    if arr.ndim != 2:
        raise CollisionCalibError(f"{name} must be 2D (N, J), got shape {arr.shape}")
    if arr.shape[0] == 0 or arr.shape[1] == 0:
        raise CollisionCalibError(f"{name} must not be empty, got shape {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise CollisionCalibError(f"{name} contains NaN or Inf")
    return arr


def _as_1d_float_vector(
    name: str,
    val: Any,
    expected_len: int,
    *,
    strict_positive: bool = False,
    non_negative: bool = False,
) -> np.ndarray:
    arr = np.asarray(val)
    if arr.dtype == object or np.issubdtype(arr.dtype, np.bool_) or not (
        np.issubdtype(arr.dtype, np.number)
    ):
        raise CollisionCalibError(f"{name} must be a numeric 1D vector, got dtype {arr.dtype}")
    if np.issubdtype(arr.dtype, np.complexfloating):
        raise CollisionCalibError(f"{name} must be real-valued, got complex dtype {arr.dtype}")
    arr = arr.astype(float, copy=False)
    if arr.ndim != 1:
        raise CollisionCalibError(f"{name} must be 1D of length {expected_len}, got shape {arr.shape}")
    if arr.shape[0] != expected_len:
        raise CollisionCalibError(f"{name} length mismatch: expected {expected_len}, got {arr.shape[0]}")
    if not np.all(np.isfinite(arr)):
        raise CollisionCalibError(f"{name} contains NaN or Inf")
    if strict_positive and np.any(arr <= 0.0):
        raise CollisionCalibError(f"{name} values must all be > 0")
    if non_negative and np.any(arr < 0.0):
        raise CollisionCalibError(f"{name} values must all be >= 0")
    return arr


def _validate_joint_names(joint_names: Sequence[str] | None, n_joints: int) -> list[str]:
    if joint_names is None:
        return [f"joint_{i + 1}" for i in range(n_joints)]
    if isinstance(joint_names, (str, bytes)) or len(joint_names) != n_joints:
        raise CollisionCalibError(f"joint_names must have {n_joints} entries")
    names = []
    for idx, name in enumerate(joint_names):
        if not isinstance(name, str) or not name.strip():
            raise CollisionCalibError(f"joint_names[{idx}] must be a non-empty string")
        names.append(name.strip())
    return names


def sign_smooth(dq: np.ndarray | float, v_eps: float = DEFAULT_V_EPS) -> np.ndarray:
    """Smooth sign function ``tanh(dq / v_eps)`` avoiding discontinuity at zero velocity."""
    eps = _require_positive_finite_scalar("v_eps", v_eps)
    arr = np.asarray(dq, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise CollisionCalibError("dq contains NaN or Inf")
    return np.tanh(arr / eps)


def _solve_nnls_3(A: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """Exact non-negative least squares for 3 parameters ``min_{x >= 0} ||A x - y||_2^2``."""
    best_x = np.zeros(3, dtype=float)
    best_rss = float(np.dot(y, y))
    for k in (1, 2, 3):
        for cols in itertools.combinations((0, 1, 2), k):
            sub = A[:, cols]
            coef, *_ = np.linalg.lstsq(sub, y, rcond=None)
            if np.all(coef >= 0.0):
                x = np.zeros(3, dtype=float)
                for idx, val in zip(cols, coef):
                    x[idx] = float(val)
                res = A @ x - y
                rss = float(np.dot(res, res))
                if rss < best_rss:
                    best_rss = rss
                    best_x = x
    return float(best_x[0]), float(best_x[1]), float(best_x[2])


def predict_dynamic_torque(
    dq: np.ndarray,
    ddq: np.ndarray,
    tau_gravity: np.ndarray,
    params: dict[str, Sequence[float]] | Sequence[tuple[float, float, float]],
    *,
    v_eps: float = DEFAULT_V_EPS,
) -> np.ndarray:
    """Predict ``tau_dyn = tau_gravity + f_c * sign_smooth(dq) + f_v * dq + I_eff * ddq``."""
    dq_a = _as_2d_float_array("dq", dq)
    ddq_a = _as_2d_float_array("ddq", ddq)
    tau_g = _as_2d_float_array("tau_gravity", tau_gravity)
    if not (dq_a.shape == ddq_a.shape == tau_g.shape):
        raise CollisionCalibError(
            f"shape mismatch: dq={dq_a.shape}, ddq={ddq_a.shape}, tau_gravity={tau_g.shape}"
        )
    n_joints = dq_a.shape[1]
    if isinstance(params, dict):
        src = params.get("params", params)
        fc = _as_1d_float_vector("f_c_nm", src["f_c_nm"], n_joints, non_negative=True)
        fv = _as_1d_float_vector("f_v_nm_s_rad", src["f_v_nm_s_rad"], n_joints, non_negative=True)
        ieff = _as_1d_float_vector("i_eff_kg_m2", src["i_eff_kg_m2"], n_joints, non_negative=True)
    else:
        if len(params) != n_joints:
            raise CollisionCalibError(f"params length {len(params)} != n_joints {n_joints}")
        mat = np.asarray(params, dtype=float)
        if mat.shape != (n_joints, 3) or not np.all(np.isfinite(mat)) or np.any(mat < 0.0):
            raise CollisionCalibError("params rows must be non-negative finite (f_c, f_v, I_eff) tuples")
        fc, fv, ieff = mat[:, 0], mat[:, 1], mat[:, 2]
    return tau_g + sign_smooth(dq_a, v_eps=v_eps) * fc + dq_a * fv + ddq_a * ieff


def compute_dynamic_residual(
    dq: np.ndarray,
    ddq: np.ndarray,
    tau_meas: np.ndarray,
    tau_gravity: np.ndarray,
    params: dict[str, Any] | Sequence[tuple[float, float, float]],
    *,
    v_eps: float = DEFAULT_V_EPS,
) -> np.ndarray:
    """Return dynamic residual ``tau_meas - tau_dyn`` of shape ``(N, J)``."""
    tau_m = _as_2d_float_array("tau_meas", tau_meas)
    tau_dyn = predict_dynamic_torque(dq, ddq, tau_gravity, params, v_eps=v_eps)
    if tau_m.shape != tau_dyn.shape:
        raise CollisionCalibError(f"shape mismatch: tau_meas={tau_m.shape} != tau_dyn={tau_dyn.shape}")
    return tau_m - tau_dyn


def detect_collisions(
    residuals: np.ndarray,
    thresholds_nm: Sequence[float] | np.ndarray,
    *,
    persist_ticks: int = DEFAULT_PERSIST_TICKS,
    clear_ratio: float = DEFAULT_CLEAR_RATIO,
) -> dict[str, Any]:
    """Simulate armd's per-tick persistence collision check on residual torques ``(N, J)``."""
    res = _as_2d_float_array("residuals", residuals)
    n_samples, n_joints = res.shape
    thresh = _as_1d_float_vector("thresholds_nm", thresholds_nm, n_joints, strict_positive=True)
    ticks = _require_persist_ticks(persist_ticks)
    c_ratio = _require_positive_finite_scalar("clear_ratio", clear_ratio)
    if c_ratio >= 1.0:
        raise CollisionCalibError(f"clear_ratio must be in (0, 1), got {c_ratio}")

    abs_res = np.abs(res)
    exceeded_mask = abs_res > thresh[None, :]
    exceeded_ticks = [int(x) for x in np.sum(exceeded_mask, axis=0)]
    max_abs = [float(x) for x in np.max(abs_res, axis=0)]
    max_consec = [0] * n_joints
    over = [0] * n_joints
    trip_events: list[dict[str, Any]] = []

    for t in range(n_samples):
        for j in range(n_joints):
            if exceeded_mask[t, j]:
                over[j] += 1
                if over[j] > max_consec[j]:
                    max_consec[j] = over[j]
                if over[j] == ticks:
                    trip_events.append({
                        "tick": t,
                        "joint": j + 1,
                        "residual_nm": float(abs_res[t, j]),
                        "threshold_nm": float(thresh[j]),
                    })
            else:
                over[j] = 0

    return {
        "tripped": len(trip_events) > 0,
        "trip_events": trip_events,
        "exceeded_ticks": exceeded_ticks,
        "max_consecutive_ticks": max_consec,
        "max_abs_residual_nm": max_abs,
    }


def fit_dynamic_collision_thresholds(
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    tau_meas: np.ndarray,
    tau_gravity: np.ndarray,
    *,
    v_eps: float = DEFAULT_V_EPS,
    safety_factor: float = DEFAULT_SAFETY_FACTOR,
    floor_nm: float = DEFAULT_FLOOR_NM,
    persist_ticks: int = DEFAULT_PERSIST_TICKS,
    clear_ratio: float = DEFAULT_CLEAR_RATIO,
    high_scale: float = DEFAULT_HIGH_SCALE,
    low_scale: float = DEFAULT_LOW_SCALE,
    joint_names: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Fit per-joint dynamic model and derive profile collision thresholds & sim backfill.

    Fits ``tau_meas - tau_gravity = f_c * tanh(dq / v_eps) + f_v * dq + I_eff * ddq``
    per joint subject to ``f_c >= 0``, ``f_v >= 0``, ``I_eff >= 0``.
    Generates ``high < medium < low`` thresholds consistent with armd's ``Sensitivity``
    multipliers (``high = high_scale * medium``, ``low = low_scale * medium``), where
    ``high = max(max_abs_residual * safety_factor, floor_nm)``.
    """
    q_a = _as_2d_float_array("q", q)
    dq_a = _as_2d_float_array("dq", dq)
    ddq_a = _as_2d_float_array("ddq", ddq)
    tau_m = _as_2d_float_array("tau_meas", tau_meas)
    tau_g = _as_2d_float_array("tau_gravity", tau_gravity)

    shapes = {q_a.shape, dq_a.shape, ddq_a.shape, tau_m.shape, tau_g.shape}
    if len(shapes) != 1:
        raise CollisionCalibError(
            "input array shapes must match: "
            f"q={q_a.shape}, dq={dq_a.shape}, ddq={ddq_a.shape}, "
            f"tau_meas={tau_m.shape}, tau_gravity={tau_g.shape}"
        )

    n_samples, n_joints = q_a.shape
    eps = _require_positive_finite_scalar("v_eps", v_eps)
    sf = _require_positive_finite_scalar("safety_factor", safety_factor)
    floor = _require_positive_finite_scalar("floor_nm", floor_nm)
    ticks = _require_persist_ticks(persist_ticks)
    c_ratio = _require_positive_finite_scalar("clear_ratio", clear_ratio)
    if c_ratio >= 1.0:
        raise CollisionCalibError(f"clear_ratio must be in (0, 1), got {c_ratio}")
    h_scale = _require_positive_finite_scalar("high_scale", high_scale)
    l_scale = _require_positive_finite_scalar("low_scale", low_scale)
    if not (h_scale < 1.0 < l_scale):
        raise CollisionCalibError(
            f"sensitivity scales must satisfy 0 < high_scale < 1.0 < low_scale, got high={h_scale}, low={l_scale}"
        )
    names = _validate_joint_names(joint_names, n_joints)

    fc_list: list[float] = []
    fv_list: list[float] = []
    ieff_list: list[float] = []
    p99_list: list[float] = []
    max_abs_list: list[float] = []
    rms_list: list[float] = []
    mean_abs_list: list[float] = []
    high_list: list[float] = []
    medium_list: list[float] = []
    low_list: list[float] = []
    joints_report: list[dict[str, Any]] = []

    smooth_s = sign_smooth(dq_a, v_eps=eps)
    y_all = tau_m - tau_g

    for j in range(n_joints):
        A = np.column_stack([smooth_s[:, j], dq_a[:, j], ddq_a[:, j]])
        y = y_all[:, j]
        fc, fv, ieff = _solve_nnls_3(A, y)
        res = y - (fc * smooth_s[:, j] + fv * dq_a[:, j] + ieff * ddq_a[:, j])
        abs_res = np.abs(res)

        p99 = float(np.percentile(abs_res, 99))
        max_abs = float(np.max(abs_res))
        rms = float(np.sqrt(np.mean(res ** 2)))
        mean_abs = float(np.mean(abs_res))

        high_th = max(max_abs * sf, floor)
        med_th = high_th / h_scale
        low_th = med_th * l_scale

        fc_list.append(fc)
        fv_list.append(fv)
        ieff_list.append(ieff)
        p99_list.append(p99)
        max_abs_list.append(max_abs)
        rms_list.append(rms)
        mean_abs_list.append(mean_abs)
        high_list.append(high_th)
        medium_list.append(med_th)
        low_list.append(low_th)

        joints_report.append({
            "joint": j + 1,
            "name": names[j],
            "f_c_nm": fc,
            "f_v_nm_s_rad": fv,
            "i_eff_kg_m2": ieff,
            "rms_residual_nm": rms,
            "mean_abs_residual_nm": mean_abs,
            "p99_abs_residual_nm": p99,
            "max_abs_residual_nm": max_abs,
            "thresholds_nm": {
                "high": high_th,
                "medium": med_th,
                "low": low_th,
            },
            "excitation": {
                "q_span_rad": float(np.max(q_a[:, j]) - np.min(q_a[:, j])),
                "max_abs_dq_rad_s": float(np.max(np.abs(dq_a[:, j]))),
                "max_abs_ddq_rad_s2": float(np.max(np.abs(ddq_a[:, j]))),
            },
        })

    return {
        "tool": "rebot-collision-calib",
        "model": "tau_dyn = tau_gravity + f_c * tanh(dq / v_eps) + f_v * dq + I_eff * ddq",
        "v_eps": eps,
        "num_samples": n_samples,
        "num_joints": n_joints,
        "joint_names": names,
        "safety_factor": sf,
        "floor_nm": floor,
        "params": {
            "f_c_nm": fc_list,
            "f_v_nm_s_rad": fv_list,
            "i_eff_kg_m2": ieff_list,
        },
        "residual_stats": {
            "p99_abs_nm": p99_list,
            "max_abs_nm": max_abs_list,
            "rms_nm": rms_list,
            "mean_abs_nm": mean_abs_list,
        },
        "thresholds_nm": {
            "high": high_list,
            "medium": medium_list,
            "low": low_list,
        },
        "joints": joints_report,
        "profile_patch": {
            "collision": {
                "thresholds_nm": medium_list,
                "persist_ticks": ticks,
                "clear_ratio": c_ratio,
                "calibrated": True,
                "sensitivity_thresholds_nm": {
                    "high": high_list,
                    "medium": medium_list,
                    "low": low_list,
                },
            }
        },
        "sim_backfill": {
            "frictionloss": fc_list,
            "damping": fv_list,
            "armature": ieff_list,
        },
    }


def evaluate_factory_self_check(
    residual_torques: np.ndarray,
    thresholds_nm: Sequence[float] | np.ndarray,
    *,
    baseline_nm: Sequence[float] | np.ndarray | None = None,
    persist_ticks: int = DEFAULT_PERSIST_TICKS,
    warn_ratio: float = DEFAULT_WARN_RATIO,
    fail_ratio: float = DEFAULT_FAIL_RATIO,
    baseline_warn_factor: float | None = None,
    joint_names: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Evaluate a single unit's 30s unloaded self-check residual trajectory.

    Parameters
    ----------
    residual_torques:
        2D array ``(N, J)`` of measured residual torques during unloaded self-check.
    thresholds_nm:
        Positive thresholds (length ``J``), typically the profile ``high`` sensitivity
        thresholds.
    baseline_nm:
        Optional model-level free-space baseline residual per joint (length ``J``).
    persist_ticks:
        Consecutive ticks above ``thresholds_nm`` that trigger a collision trip.
    warn_ratio:
        Utilization ratio ``max_residual / threshold`` above which a joint is flagged
        ``WARN`` (default ``0.80``).
    fail_ratio:
        Utilization ratio at or above which a joint is flagged ``FAIL`` (default ``0.95``).
    baseline_warn_factor:
        Optional multiplier over ``baseline_nm``; if set and ``max_residual > factor * baseline``,
        raises at least ``WARN``.

    Returns
    -------
    dict
        Report with ``verdict`` (``"PASS"`` | ``"WARN"`` | ``"FAIL"``), per-joint
        ``max_residual_nm``, ``margin_ratio`` (``(threshold - max_res) / threshold``),
        ``exceeded_ticks``, and diagnostic reasons.
    """
    res = _as_2d_float_array("residual_torques", residual_torques)
    n_samples, n_joints = res.shape
    thresh = _as_1d_float_vector("thresholds_nm", thresholds_nm, n_joints, strict_positive=True)
    ticks = _require_persist_ticks(persist_ticks)
    w_ratio = _require_positive_finite_scalar("warn_ratio", warn_ratio)
    f_ratio = _require_positive_finite_scalar("fail_ratio", fail_ratio)
    if not (0.0 < w_ratio < f_ratio <= 1.0):
        raise CollisionCalibError(
            f"ratios must satisfy 0 < warn_ratio < fail_ratio <= 1.0, got warn={w_ratio}, fail={f_ratio}"
        )
    base_vec: np.ndarray | None = None
    if baseline_nm is not None:
        base_vec = _as_1d_float_vector("baseline_nm", baseline_nm, n_joints, non_negative=True)
    bw_factor: float | None = None
    if baseline_warn_factor is not None:
        bw_factor = _require_positive_finite_scalar("baseline_warn_factor", baseline_warn_factor)
    names = _validate_joint_names(joint_names, n_joints)

    det = detect_collisions(res, thresh, persist_ticks=ticks)
    abs_res = np.abs(res)

    max_res_list: list[float] = det["max_abs_residual_nm"]
    p99_res_list: list[float] = [float(x) for x in np.percentile(abs_res, 99, axis=0)]
    exceeded_list: list[int] = det["exceeded_ticks"]
    max_consec_list: list[int] = det["max_consecutive_ticks"]
    margin_ratio_list: list[float] = []
    threshold_ratio_list: list[float] = []
    joint_verdicts: list[str] = []
    joints_out: list[dict[str, Any]] = []
    reasons: list[str] = []

    for j in range(n_joints):
        max_r = max_res_list[j]
        th = float(thresh[j])
        t_ratio = max_r / th
        m_ratio = (th - max_r) / th
        exc = exceeded_list[j]
        consec = max_consec_list[j]

        base_val = float(base_vec[j]) if base_vec is not None else None
        base_delta = (max_r - base_val) if base_val is not None else None
        base_ratio = (max_r / base_val) if (base_val is not None and base_val > 0.0) else None

        if consec >= ticks or exc > 0 or t_ratio >= f_ratio:
            jv = "FAIL"
            if consec >= ticks:
                reason = (
                    f"{names[j]}: triggered collision during unloaded run "
                    f"({consec} consecutive ticks > {th:.4f} N·m, max {max_r:.4f} N·m)"
                )
            elif exc > 0:
                reason = (
                    f"{names[j]}: exceeded threshold ({exc} ticks > {th:.4f} N·m, max {max_r:.4f} N·m)"
                )
            else:
                reason = (
                    f"{names[j]}: approached collision threshold "
                    f"({t_ratio:.1%} of {th:.4f} N·m, margin {m_ratio:.1%})"
                )
            reasons.append(reason)
        elif t_ratio > w_ratio or (
            bw_factor is not None and base_val is not None and max_r > bw_factor * base_val
        ):
            jv = "WARN"
            if t_ratio > w_ratio:
                reason = (
                    f"{names[j]}: elevated unloaded residual "
                    f"({t_ratio:.1%} of {th:.4f} N·m > {w_ratio:.0%}; tight reducer or friction bias)"
                )
            else:
                reason = (
                    f"{names[j]}: residual {max_r:.4f} N·m exceeds {bw_factor:.2f}x baseline "
                    f"({base_val:.4f} N·m)"
                )
            reasons.append(reason)
        else:
            jv = "PASS"
            reason = f"{names[j]}: within safe envelope ({t_ratio:.1%} of threshold, margin {m_ratio:.1%})"

        margin_ratio_list.append(m_ratio)
        threshold_ratio_list.append(t_ratio)
        joint_verdicts.append(jv)
        joints_out.append({
            "joint": j + 1,
            "name": names[j],
            "verdict": jv,
            "max_residual_nm": max_r,
            "p99_residual_nm": p99_res_list[j],
            "threshold_nm": th,
            "margin_ratio": m_ratio,
            "threshold_ratio": t_ratio,
            "exceeded_ticks": exc,
            "max_consecutive_ticks": consec,
            "baseline_nm": base_val,
            "baseline_delta_nm": base_delta,
            "baseline_ratio": base_ratio,
            "reason": reason,
        })

    if "FAIL" in joint_verdicts:
        overall = "FAIL"
    elif "WARN" in joint_verdicts:
        overall = "WARN"
    else:
        overall = "PASS"

    return {
        "tool": "rebot-collision-calib",
        "verdict": overall,
        "num_samples": n_samples,
        "num_joints": n_joints,
        "persist_ticks": ticks,
        "warn_ratio": w_ratio,
        "fail_ratio": f_ratio,
        "tripped": det["tripped"],
        "max_residual_nm": max_res_list,
        "p99_residual_nm": p99_res_list,
        "thresholds_nm": [float(x) for x in thresh],
        "margin_ratio": margin_ratio_list,
        "threshold_ratio": threshold_ratio_list,
        "exceeded_ticks": exceeded_list,
        "max_consecutive_ticks": max_consec_list,
        "joint_verdicts": joint_verdicts,
        "joints": joints_out,
        "reasons": reasons,
    }


# --------------------------------------------------------------------------- I/O & CLI

def _central_diff(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Central difference dx/dt on uniform or non-uniform timestamps."""
    t_a = np.asarray(t, dtype=float)
    x_a = np.asarray(x, dtype=float)
    if t_a.ndim != 1 or x_a.ndim != 2 or t_a.shape[0] != x_a.shape[0] or t_a.shape[0] < 2:
        raise CollisionCalibError("cannot differentiate trajectory with fewer than 2 valid samples")
    if not np.all(np.isfinite(t_a)) or np.any(np.diff(t_a) <= 0.0):
        raise CollisionCalibError("timestamps must be finite and strictly increasing")
    return np.gradient(x_a, t_a, axis=0)


def load_trajectory(path: str | Path) -> dict[str, np.ndarray]:
    """Load trajectory arrays from ``.npz`` or ``.json``.

    Supports direct keys ``(q, dq, ddq, tau_meas, tau_gravity)``, self-check
    ``residual_torques`` arrays, and ``rebot-sysid`` ``samples.json`` logs.
    """
    p = Path(path)
    if not p.exists():
        raise CollisionCalibError(f"input file not found: {p}")

    suffix = p.suffix.lower()
    if suffix == ".npz":
        try:
            with np.load(p, allow_pickle=False) as z:
                raw: dict[str, Any] = {k: z[k] for k in z.files}
        except Exception as exc:
            raise CollisionCalibError(f"failed to read NPZ {p}: {exc}") from exc
    elif suffix == ".json":
        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except Exception as exc:
            raise CollisionCalibError(f"failed to read JSON {p}: {exc}") from exc
        if not isinstance(raw, dict):
            raise CollisionCalibError(f"JSON root in {p} must be an object")
    else:
        raise CollisionCalibError(f"unsupported input extension '{p.suffix}' (expected .json or .npz)")

    # Support rebot-sysid samples.json format
    if "samples" in raw and isinstance(raw["samples"], list):
        samples = raw["samples"]
        if not samples:
            raise CollisionCalibError("samples list is empty")
        try:
            t = np.asarray([s["t"] for s in samples], dtype=float)
            q = np.asarray([s["q"][:ARM_JOINTS] for s in samples], dtype=float)
            dq = np.asarray([s["dq"][:ARM_JOINTS] for s in samples], dtype=float)
            tau_meas = np.asarray([s["tau"][:ARM_JOINTS] for s in samples], dtype=float)
            has_res = all(s.get("res") is not None for s in samples)
            if has_res:
                res = np.asarray([s["res"][:ARM_JOINTS] for s in samples], dtype=float)
                tau_gravity = tau_meas - res
            else:
                res = None
                tau_gravity = np.zeros_like(tau_meas)
        except (KeyError, TypeError, ValueError) as exc:
            raise CollisionCalibError(f"malformed samples entry in {p}: {exc}") from exc
        ddq = _central_diff(t, dq)
        out = {
            "q": _as_2d_float_array("q", q),
            "dq": _as_2d_float_array("dq", dq),
            "ddq": _as_2d_float_array("ddq", ddq),
            "tau_meas": _as_2d_float_array("tau_meas", tau_meas),
            "tau_gravity": _as_2d_float_array("tau_gravity", tau_gravity),
        }
        if res is not None:
            out["residual_torques"] = _as_2d_float_array("residual_torques", res)
        return out

    out: dict[str, np.ndarray] = {}
    for key in ("q", "dq", "ddq", "tau_meas", "tau_gravity", "residual_torques"):
        if key in raw and raw[key] is not None:
            out[key] = _as_2d_float_array(key, raw[key])

    if "tau_gravity" not in out and "tau_meas" in out and "residual_torques" in out:
        if out["tau_meas"].shape != out["residual_torques"].shape:
            raise CollisionCalibError("tau_meas and residual_torques shapes must match")
        out["tau_gravity"] = out["tau_meas"] - out["residual_torques"]

    if "ddq" not in out and "dq" in out and "t" in raw:
        out["ddq"] = _central_diff(raw["t"], out["dq"])

    return out


def _save_report(report: dict[str, Any], out_path: str | Path | None) -> None:
    if out_path is None:
        print(json.dumps(report, indent=2))
        return
    p = Path(out_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.suffix.lower() == ".npz":
        arrays: dict[str, Any] = {"report_json": np.array(json.dumps(report))}
        if "params" in report:
            arrays["f_c_nm"] = np.asarray(report["params"]["f_c_nm"], dtype=float)
            arrays["f_v_nm_s_rad"] = np.asarray(report["params"]["f_v_nm_s_rad"], dtype=float)
            arrays["i_eff_kg_m2"] = np.asarray(report["params"]["i_eff_kg_m2"], dtype=float)
        if "thresholds_nm" in report and isinstance(report["thresholds_nm"], dict):
            for tier, vals in report["thresholds_nm"].items():
                arrays[f"thresholds_{tier}_nm"] = np.asarray(vals, dtype=float)
        elif "thresholds_nm" in report:
            arrays["thresholds_nm"] = np.asarray(report["thresholds_nm"], dtype=float)
        if "max_residual_nm" in report:
            arrays["max_residual_nm"] = np.asarray(report["max_residual_nm"], dtype=float)
            arrays["margin_ratio"] = np.asarray(report["margin_ratio"], dtype=float)
            arrays["exceeded_ticks"] = np.asarray(report["exceeded_ticks"], dtype=int)
        np.savez(p, **arrays)
    else:
        p.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def _load_report(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        raise CollisionCalibError(f"calibration/profile file not found: {p}")
    if p.suffix.lower() == ".npz":
        try:
            with np.load(p, allow_pickle=False) as z:
                if "report_json" not in z.files:
                    raise CollisionCalibError(f"NPZ {p} missing 'report_json'")
                return json.loads(str(z["report_json"].item()))
        except CollisionCalibError:
            raise
        except Exception as exc:
            raise CollisionCalibError(f"failed to read NPZ report {p}: {exc}") from exc
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:
        raise CollisionCalibError(f"failed to read JSON {p}: {exc}") from exc
    if not isinstance(data, dict):
        raise CollisionCalibError(f"JSON root in {p} must be an object")
    return data


def apply_profile_patch(profile_path: str | Path, calib_report: dict[str, Any], out_profile: str | Path) -> dict[str, Any]:
    """Merge ``calib_report['profile_patch']['collision']`` into a profile JSON and write it."""
    p = Path(profile_path)
    if not p.exists():
        raise CollisionCalibError(f"profile file not found: {p}")
    profile = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise CollisionCalibError("profile JSON root must be an object")
    collision = dict(profile.get("collision") or {})
    collision.update(calib_report["profile_patch"]["collision"])
    profile["collision"] = collision
    out_p = Path(out_profile)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    return profile


def _parse_float_csv(text: str) -> list[float]:
    try:
        vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    except ValueError as exc:
        raise CollisionCalibError(f"invalid float list '{text}': {exc}") from exc
    if not vals:
        raise CollisionCalibError("float list must not be empty")
    return vals


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rebot-collision-calib",
        description="Dynamic collision threshold calibration and 30s factory self-check.",
    )
    sub = p.add_subparsers(dest="command", required=True)

    fit_p = sub.add_parser("fit", help="Fit dynamic friction/inertia model and collision thresholds.")
    fit_p.add_argument("input", help="Input trajectory (.json or .npz) with q, dq, ddq, tau_meas, tau_gravity")
    fit_p.add_argument("--out", default=None, help="Output calibration report path (.json or .npz; default stdout JSON)")
    fit_p.add_argument("--profile", default=None, help="Optional profile JSON to patch")
    fit_p.add_argument("--out-profile", default=None, help="Output path for patched profile JSON")
    fit_p.add_argument("--v-eps", type=float, default=DEFAULT_V_EPS, help="Smooth sign velocity scale (rad/s)")
    fit_p.add_argument("--safety-factor", type=float, default=DEFAULT_SAFETY_FACTOR, help="Safety multiplier on max residual")
    fit_p.add_argument("--floor-nm", type=float, default=DEFAULT_FLOOR_NM, help="Minimum high-sensitivity threshold (N·m)")
    fit_p.add_argument("--persist-ticks", type=int, default=DEFAULT_PERSIST_TICKS, help="Consecutive ticks to trip")
    fit_p.add_argument("--clear-ratio", type=float, default=DEFAULT_CLEAR_RATIO, help="Auto-resume clear ratio")

    sc_p = sub.add_parser("self-check", help="Evaluate a single unit's 30s unloaded self-check run.")
    sc_p.add_argument("input", help="Input trajectory (.json or .npz)")
    sc_p.add_argument("--calib", default=None, help="Calibration report (.json or .npz) from 'fit'")
    sc_p.add_argument("--profile", default=None, help="Profile JSON containing collision.thresholds_nm")
    sc_p.add_argument("--sensitivity", choices=("high", "medium", "low"), default="high", help="Threshold tier")
    sc_p.add_argument("--thresholds", default=None, help="Explicit comma-separated thresholds (N·m)")
    sc_p.add_argument("--baseline", default=None, help="Explicit comma-separated baseline residuals (N·m)")
    sc_p.add_argument("--persist-ticks", type=int, default=DEFAULT_PERSIST_TICKS, help="Consecutive ticks to trip")
    sc_p.add_argument("--warn-ratio", type=float, default=DEFAULT_WARN_RATIO, help="WARN threshold ratio (default 0.80)")
    sc_p.add_argument("--fail-ratio", type=float, default=DEFAULT_FAIL_RATIO, help="FAIL threshold ratio (default 0.95)")
    sc_p.add_argument("--out", default=None, help="Output self-check report path (.json or .npz; default stdout JSON)")

    return p


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entrypoint for ``rebot-collision-calib``."""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        if args.command == "fit":
            traj = load_trajectory(args.input)
            for req in ("q", "dq", "ddq", "tau_meas", "tau_gravity"):
                if req not in traj:
                    raise CollisionCalibError(f"input trajectory missing required array '{req}'")
            report = fit_dynamic_collision_thresholds(
                traj["q"],
                traj["dq"],
                traj["ddq"],
                traj["tau_meas"],
                traj["tau_gravity"],
                v_eps=args.v_eps,
                safety_factor=args.safety_factor,
                floor_nm=args.floor_nm,
                persist_ticks=args.persist_ticks,
                clear_ratio=args.clear_ratio,
            )
            if args.profile is not None:
                out_prof = args.out_profile or (args.profile + ".calibrated.json")
                apply_profile_patch(args.profile, report, out_prof)
            elif args.out_profile is not None:
                raise CollisionCalibError("--out-profile requires --profile")
            _save_report(report, args.out)
            return 0

        if args.command == "self-check":
            traj = load_trajectory(args.input)
            calib_report = _load_report(args.calib) if args.calib is not None else None
            profile_data = _load_report(args.profile) if args.profile is not None else None

            if args.thresholds is not None:
                thresholds = _parse_float_csv(args.thresholds)
            elif calib_report is not None:
                thresholds = calib_report["thresholds_nm"][args.sensitivity]
            elif profile_data is not None:
                col = profile_data.get("collision") or {}
                if "sensitivity_thresholds_nm" in col and args.sensitivity in col["sensitivity_thresholds_nm"]:
                    thresholds = col["sensitivity_thresholds_nm"][args.sensitivity]
                elif "thresholds_nm" in col:
                    scale = {"high": DEFAULT_HIGH_SCALE, "medium": 1.0, "low": DEFAULT_LOW_SCALE}[args.sensitivity]
                    thresholds = [float(x) * scale for x in col["thresholds_nm"]]
                else:
                    raise CollisionCalibError("profile has no collision.thresholds_nm")
            else:
                raise CollisionCalibError("self-check requires --calib, --profile, or --thresholds")

            baseline = None
            if args.baseline is not None:
                baseline = _parse_float_csv(args.baseline)
            elif calib_report is not None and "residual_stats" in calib_report:
                baseline = calib_report["residual_stats"].get("max_abs_nm")

            if (
                calib_report is not None
                and "params" in calib_report
                and all(k in traj for k in ("dq", "ddq", "tau_meas", "tau_gravity"))
            ):
                residuals = compute_dynamic_residual(
                    traj["dq"],
                    traj["ddq"],
                    traj["tau_meas"],
                    traj["tau_gravity"],
                    calib_report["params"],
                    v_eps=float(calib_report.get("v_eps", DEFAULT_V_EPS)),
                )
            elif "residual_torques" in traj:
                residuals = traj["residual_torques"]
            elif "tau_meas" in traj and "tau_gravity" in traj:
                residuals = traj["tau_meas"] - traj["tau_gravity"]
            else:
                raise CollisionCalibError(
                    "self-check input must provide residual_torques or (dq, ddq, tau_meas, tau_gravity)"
                )

            joint_names = calib_report.get("joint_names") if calib_report is not None else None
            report = evaluate_factory_self_check(
                residuals,
                thresholds,
                baseline_nm=baseline,
                persist_ticks=args.persist_ticks,
                warn_ratio=args.warn_ratio,
                fail_ratio=args.fail_ratio,
                joint_names=joint_names,
            )
            _save_report(report, args.out)
            return 1 if report["verdict"] == "FAIL" else 0

        raise CollisionCalibError(f"unknown command: {args.command}")
    except (CollisionCalibError, OSError, KeyError, TypeError) as exc:
        print(f"rebot-collision-calib: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
