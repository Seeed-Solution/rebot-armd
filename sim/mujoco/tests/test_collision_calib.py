"""Tests for dynamic collision threshold calibration and 30s factory self-check."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from rebot_sim import collision_calib as cc
from rebot_sim.collision_calib import (
    CollisionCalibError,
    compute_dynamic_residual,
    detect_collisions,
    evaluate_factory_self_check,
    fit_dynamic_collision_thresholds,
    main as cli_main,
)

KIT_PROFILE = Path(__file__).resolve().parents[3] / "kits" / "recamera-b601dm" / "profile" / "rebot-b601dm.json"

TRUE_FC = np.array([0.35, 0.42, 0.30, 0.12, 0.10, 0.08])
TRUE_FV = np.array([1.40, 1.25, 1.30, 0.32, 0.28, 0.25])
TRUE_IEFF = np.array([0.045, 0.055, 0.040, 0.008, 0.006, 0.004])
NOISE_STD = np.array([0.015, 0.015, 0.015, 0.006, 0.005, 0.005])


def _synthetic_excitation(
    *,
    n_samples: int = 2000,
    dt: float = 0.02,
    speed_scale: float = 1.0,
    seed: int = 42,
    fc: np.ndarray = TRUE_FC,
    fv: np.ndarray = TRUE_FV,
    ieff: np.ndarray = TRUE_IEFF,
    noise_std: np.ndarray = NOISE_STD,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples, dtype=float) * dt
    freqs_1 = np.array([0.35, 0.45, 0.50, 0.80, 0.90, 1.05])
    freqs_2 = np.array([1.15, 1.35, 1.45, 2.10, 2.40, 2.75])
    amps_1 = np.array([0.55, 0.45, 0.50, 0.65, 0.60, 0.70]) * speed_scale
    amps_2 = np.array([0.18, 0.15, 0.16, 0.22, 0.20, 0.25]) * speed_scale
    phases = np.array([0.0, 0.4, 0.9, 1.3, 1.8, 2.2])

    w1 = 2.0 * np.pi * freqs_1[None, :]
    w2 = 2.0 * np.pi * freqs_2[None, :]
    arg1 = t[:, None] * w1 + phases[None, :]
    arg2 = t[:, None] * w2 + 0.5 * phases[None, :]

    q = amps_1 * np.sin(arg1) + amps_2 * np.sin(arg2)
    dq = amps_1 * w1 * np.cos(arg1) + amps_2 * w2 * np.cos(arg2)
    ddq = -amps_1 * (w1 ** 2) * np.sin(arg1) - amps_2 * (w2 ** 2) * np.sin(arg2)

    # Smooth pose-dependent gravity load
    tau_gravity = np.column_stack([
        0.1 * np.sin(q[:, 0]),
        4.5 * np.cos(q[:, 1]) + 1.8 * np.cos(q[:, 1] + q[:, 2]),
        2.2 * np.cos(q[:, 1] + q[:, 2]),
        0.4 * np.sin(q[:, 3]),
        0.3 * np.cos(q[:, 4]),
        0.05 * np.sin(q[:, 5]),
    ])
    noise = rng.normal(0.0, noise_std[None, :], size=q.shape)
    tau_meas = (
        tau_gravity
        + np.tanh(dq / cc.DEFAULT_V_EPS) * fc[None, :]
        + dq * fv[None, :]
        + ddq * ieff[None, :]
        + noise
    )
    return q, dq, ddq, tau_meas, tau_gravity


def test_fit_recovers_dynamic_parameters_and_thresholds():
    q, dq, ddq, tau_meas, tau_g = _synthetic_excitation(seed=101)
    report = fit_dynamic_collision_thresholds(q, dq, ddq, tau_meas, tau_g)

    fc_fit = np.asarray(report["params"]["f_c_nm"])
    fv_fit = np.asarray(report["params"]["f_v_nm_s_rad"])
    ieff_fit = np.asarray(report["params"]["i_eff_kg_m2"])

    np.testing.assert_allclose(fc_fit, TRUE_FC, atol=0.015, rtol=0.08)
    np.testing.assert_allclose(fv_fit, TRUE_FV, atol=0.015, rtol=0.08)
    np.testing.assert_allclose(ieff_fit, TRUE_IEFF, atol=0.002, rtol=0.08)

    # sim_backfill mirrors non-negative physical parameters
    backfill = report["sim_backfill"]
    assert backfill["frictionloss"] == report["params"]["f_c_nm"]
    assert backfill["damping"] == report["params"]["f_v_nm_s_rad"]
    assert backfill["armature"] == report["params"]["i_eff_kg_m2"]
    assert all(x >= 0.0 for x in backfill["frictionloss"] + backfill["damping"] + backfill["armature"])

    # Threshold hierarchy: 0 < high < medium < low
    high = report["thresholds_nm"]["high"]
    med = report["thresholds_nm"]["medium"]
    low = report["thresholds_nm"]["low"]
    assert len(high) == len(med) == len(low) == 6
    for j in range(6):
        assert 0.0 < high[j] < med[j] < low[j]
        assert report["residual_stats"]["p99_abs_nm"][j] <= report["residual_stats"]["max_abs_nm"][j]
        assert high[j] >= report["residual_stats"]["max_abs_nm"][j] * report["safety_factor"] - 1e-12

    patch = report["profile_patch"]["collision"]
    assert patch["calibrated"] is True
    assert patch["persist_ticks"] == 4
    assert patch["thresholds_nm"] == med
    assert patch["sensitivity_thresholds_nm"] == report["thresholds_nm"]


def test_nnls_enforces_non_negative_physical_constraints():
    # Joint 0 has zero viscous damping and zero inertia; noise would push unconstrained OLS negative.
    fc = np.array([0.30, 0.25])
    fv = np.array([0.0, 0.80])
    ieff = np.array([0.0, 0.02])
    rng = np.random.default_rng(7)
    t = np.arange(800, dtype=float) * 0.02
    q = np.column_stack([0.5 * np.sin(2.0 * t), 0.5 * np.cos(2.5 * t)])
    dq = np.column_stack([1.0 * np.cos(2.0 * t), -1.25 * np.sin(2.5 * t)])
    ddq = np.column_stack([-2.0 * np.sin(2.0 * t), -3.125 * np.cos(2.5 * t)])
    tau_g = np.zeros_like(q)
    noise = rng.normal(0.0, 0.01, size=q.shape)
    # Bias noise slightly against positive velocity on joint 0 to push unconstrained fv negative
    noise[:, 0] -= 0.02 * dq[:, 0]
    tau_meas = tau_g + np.tanh(dq / cc.DEFAULT_V_EPS) * fc + dq * fv + ddq * ieff + noise

    report = fit_dynamic_collision_thresholds(q, dq, ddq, tau_meas, tau_g)
    assert report["params"]["f_c_nm"][0] >= 0.0
    assert report["params"]["f_v_nm_s_rad"][0] >= 0.0
    assert report["params"]["i_eff_kg_m2"][0] >= 0.0
    assert report["params"]["f_v_nm_s_rad"][1] == pytest.approx(0.80, abs=0.03)


def test_high_speed_zero_false_positives_and_collision_100_percent_detection():
    q_train, dq_train, ddq_train, tau_train, tg_train = _synthetic_excitation(seed=11)
    report = fit_dynamic_collision_thresholds(q_train, dq_train, ddq_train, tau_train, tg_train)
    high_th = np.asarray(report["thresholds_nm"]["high"])
    low_th = np.asarray(report["thresholds_nm"]["low"])

    # Unseen high-speed obstacle-free trajectory (1.6x training velocity & acceleration)
    _, dq_fast, ddq_fast, tau_fast, tg_fast = _synthetic_excitation(
        n_samples=1500, speed_scale=1.6, seed=202
    )
    res_fast = compute_dynamic_residual(dq_fast, ddq_fast, tau_fast, tg_fast, report)
    free_det = detect_collisions(res_fast, high_th, persist_ticks=4)
    assert free_det["tripped"] is False
    assert free_det["exceeded_ticks"] == [0] * 6
    assert free_det["max_consecutive_ticks"] == [0] * 6
    assert free_det["trip_events"] == []

    # Inject 6 real collision pulses (one per joint, 6 ticks each, above even low threshold)
    tau_col = tau_fast.copy()
    pulse_starts = [100, 300, 500, 700, 900, 1100]
    for j, start in enumerate(pulse_starts):
        sign = 1.0 if j % 2 == 0 else -1.0
        tau_col[start:start + 6, j] += sign * (low_th[j] + 0.5)

    res_col = compute_dynamic_residual(dq_fast, ddq_fast, tau_col, tg_fast, report)
    for tier in ("high", "medium", "low"):
        det = detect_collisions(res_col, report["thresholds_nm"][tier], persist_ticks=4)
        assert det["tripped"] is True
        tripped_joints = {ev["joint"]: ev["tick"] for ev in det["trip_events"]}
        assert set(tripped_joints.keys()) == {1, 2, 3, 4, 5, 6}
        for j, start in enumerate(pulse_starts):
            assert tripped_joints[j + 1] == start + 3  # 4th consecutive tick


def test_factory_self_check_pass_warn_fail():
    thresholds = np.array([0.20, 0.25, 0.22, 0.15, 0.15, 0.12])
    baseline = thresholds * 0.55
    n_ticks = 1500  # 30s at 50 Hz
    rng = np.random.default_rng(303)

    # 1. PASS: all joints stably inside 55% of threshold
    res_pass = rng.uniform(-0.50, 0.50, size=(n_ticks, 6)) * thresholds[None, :]
    r_pass = evaluate_factory_self_check(res_pass, thresholds, baseline_nm=baseline)
    assert r_pass["verdict"] == "PASS"
    assert r_pass["tripped"] is False
    assert r_pass["exceeded_ticks"] == [0] * 6
    assert all(m > 0.2 for m in r_pass["margin_ratio"])
    assert r_pass["joint_verdicts"] == ["PASS"] * 6

    # 2. WARN: J3 reaches 86% of threshold (above 80% warn_ratio, below 95% fail_ratio)
    res_warn = res_pass.copy()
    res_warn[400:420, 2] = 0.86 * thresholds[2]
    r_warn = evaluate_factory_self_check(res_warn, thresholds, baseline_nm=baseline)
    assert r_warn["verdict"] == "WARN"
    assert r_warn["tripped"] is False
    assert r_warn["exceeded_ticks"] == [0] * 6
    assert r_warn["joint_verdicts"] == ["PASS", "PASS", "WARN", "PASS", "PASS", "PASS"]
    assert 0.0 < r_warn["margin_ratio"][2] < 0.20
    assert r_warn["max_residual_nm"][2] == pytest.approx(0.86 * thresholds[2])

    # 3. FAIL (approaching threshold >= 95%)
    res_near = res_pass.copy()
    res_near[500:506, 1] = 0.96 * thresholds[1]
    r_near = evaluate_factory_self_check(res_near, thresholds, baseline_nm=baseline)
    assert r_near["verdict"] == "FAIL"
    assert r_near["joint_verdicts"][1] == "FAIL"
    assert r_near["margin_ratio"][1] == pytest.approx(0.04)

    # 4. FAIL (collision triggered during unloaded self-check)
    res_fail = res_pass.copy()
    res_fail[700:706, 3] = -1.25 * thresholds[3]
    r_fail = evaluate_factory_self_check(res_fail, thresholds, baseline_nm=baseline, persist_ticks=4)
    assert r_fail["verdict"] == "FAIL"
    assert r_fail["tripped"] is True
    assert r_fail["exceeded_ticks"][3] == 6
    assert r_fail["max_consecutive_ticks"][3] == 6
    assert r_fail["margin_ratio"][3] == pytest.approx(-0.25)
    assert r_fail["joint_verdicts"][3] == "FAIL"


def test_negative_validation_fail_closed():
    q, dq, ddq, tau_m, tau_g = _synthetic_excitation(n_samples=50, seed=1)

    # Shape / dimension mismatch
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q[:, :5], dq, ddq, tau_m, tau_g)
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q[:-1], dq, ddq, tau_m, tau_g)
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q[:, 0], dq[:, 0], ddq[:, 0], tau_m[:, 0], tau_g[:, 0])

    # Empty samples
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q[:0], dq[:0], ddq[:0], tau_m[:0], tau_g[:0])

    # NaN / Inf in every trajectory input
    for idx in range(5):
        args = [q.copy(), dq.copy(), ddq.copy(), tau_m.copy(), tau_g.copy()]
        args[idx][10, 2] = np.nan
        with pytest.raises(CollisionCalibError):
            fit_dynamic_collision_thresholds(*args)
        args[idx][10, 2] = np.inf
        with pytest.raises(CollisionCalibError):
            fit_dynamic_collision_thresholds(*args)

    # Bad scalar parameters
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q, dq, ddq, tau_m, tau_g, safety_factor=-1.0)
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q, dq, ddq, tau_m, tau_g, floor_nm=0.0)
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q, dq, ddq, tau_m, tau_g, persist_ticks=0)
    with pytest.raises(CollisionCalibError):
        fit_dynamic_collision_thresholds(q, dq, ddq, tau_m, tau_g, v_eps=np.nan)

    # Self-check negative / zero / NaN thresholds & invalid residuals
    res = np.zeros((20, 6))
    valid_th = np.full(6, 0.2)
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(res, [-0.1, 0.2, 0.2, 0.2, 0.2, 0.2])
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(res, [0.0, 0.2, 0.2, 0.2, 0.2, 0.2])
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(res, [np.nan, 0.2, 0.2, 0.2, 0.2, 0.2])
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(res, [0.2, 0.2])
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(np.empty((0, 6)), valid_th)
    bad_res = res.copy()
    bad_res[3, 1] = np.inf
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(bad_res, valid_th)
    with pytest.raises(CollisionCalibError):
        evaluate_factory_self_check(res, valid_th, warn_ratio=0.95, fail_ratio=0.90)


def test_cli_fit_and_self_check_json_npz(tmp_path):
    q, dq, ddq, tau_m, tau_g = _synthetic_excitation(n_samples=600, seed=77)
    train_npz = tmp_path / "train.npz"
    np.savez(train_npz, q=q, dq=dq, ddq=ddq, tau_meas=tau_m, tau_gravity=tau_g)

    calib_json = tmp_path / "calib.json"
    patched_profile = tmp_path / "patched_profile.json"
    rc = cli_main([
        "fit",
        str(train_npz),
        "--out",
        str(calib_json),
        "--profile",
        str(KIT_PROFILE),
        "--out-profile",
        str(patched_profile),
    ])
    assert rc == 0
    calib = json.loads(calib_json.read_text(encoding="utf-8"))
    prof = json.loads(patched_profile.read_text(encoding="utf-8"))
    assert prof["collision"]["calibrated"] is True
    assert prof["collision"]["thresholds_nm"] == calib["thresholds_nm"]["medium"]
    assert prof["collision"]["sensitivity_thresholds_nm"] == calib["thresholds_nm"]
    # Existing profile fields preserved
    assert prof["collision"]["rest_exempt_joints"] == [False, True, True, True, False, False]

    # Also test NPZ report output and JSON sysid-style trajectory input
    samples = [
        {
            "t": float(i * 0.02),
            "q": q[i].tolist(),
            "dq": dq[i].tolist(),
            "tau": tau_m[i].tolist(),
            "res": (tau_m[i] - tau_g[i]).tolist(),
        }
        for i in range(100)
    ]
    sysid_json = tmp_path / "samples.json"
    sysid_json.write_text(json.dumps({"samples": samples}), encoding="utf-8")
    calib_npz = tmp_path / "calib.npz"
    assert cli_main(["fit", str(sysid_json), "--out", str(calib_npz)]) == 0
    with np.load(calib_npz, allow_pickle=False) as z:
        assert "f_c_nm" in z.files
        assert "thresholds_high_nm" in z.files

    # Self-check PASS via JSON input using fitted calib
    _, dq_ok, ddq_ok, tau_ok, tg_ok = _synthetic_excitation(n_samples=300, speed_scale=1.2, seed=88)
    pass_json = tmp_path / "self_pass.json"
    pass_json.write_text(
        json.dumps({
            "q": np.zeros_like(dq_ok).tolist(),
            "dq": dq_ok.tolist(),
            "ddq": ddq_ok.tolist(),
            "tau_meas": tau_ok.tolist(),
            "tau_gravity": tg_ok.tolist(),
        }),
        encoding="utf-8",
    )
    sc_pass_out = tmp_path / "sc_pass.json"
    rc_pass = cli_main([
        "self-check",
        str(pass_json),
        "--calib",
        str(calib_json),
        "--sensitivity",
        "high",
        "--out",
        str(sc_pass_out),
    ])
    assert rc_pass == 0
    assert json.loads(sc_pass_out.read_text(encoding="utf-8"))["verdict"] == "PASS"

    # Self-check FAIL via NPZ input with injected collision
    tau_bad = tau_ok.copy()
    tau_bad[50:60, 0] += 2.0
    fail_npz = tmp_path / "self_fail.npz"
    np.savez(fail_npz, dq=dq_ok, ddq=ddq_ok, tau_meas=tau_bad, tau_gravity=tg_ok)
    sc_fail_out = tmp_path / "sc_fail.npz"
    rc_fail = cli_main([
        "self-check",
        str(fail_npz),
        "--calib",
        str(calib_npz),
        "--out",
        str(sc_fail_out),
    ])
    assert rc_fail == 1
    with np.load(sc_fail_out, allow_pickle=False) as z:
        assert json.loads(str(z["report_json"].item()))["verdict"] == "FAIL"

    # Missing file returns error exit code 2
    assert cli_main(["self-check", str(tmp_path / "missing.json"), "--calib", str(calib_json)]) == 2
