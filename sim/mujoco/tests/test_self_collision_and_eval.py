"""Tests for MJCF link self-collision detection, torque noise, and 20-seed sim_eval CLI."""
from __future__ import annotations

import json
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from rebot_sim.eval_cli import (
    EVAL_REPORT_SCHEMA,
    SimEvalConfig,
    main as eval_main,
    run_sim_eval,
)
from rebot_sim.model import (
    Indexer,
    SelfCollisionConfig,
    TorqueNoiseConfig,
    apply_torque_noise,
    detect_link_self_collisions,
    load_model,
)
from rebot_sim.scene import JAW_GEOMS, Scene, parse_objects


def test_self_collision_home_and_ready_poses_are_zero_on_both_robots() -> None:
    for robot, ready_q in (
        ("b601dm", [0.0, -0.8, -1.0, 0.0, 0.2, 0.0, -3.0]),
        ("so-arm101", [0.0, -0.4, 0.6, 0.5, 0.0, 0.4]),
    ):
        model = load_model(robot=robot, self_collision=True)
        data = mujoco.MjData(model)
        idx = Indexer(model)

        # Home pose (q = 0)
        idx.set_motor_positions(data, np.zeros(idx.dof))
        mujoco.mj_forward(model, data)
        home_summary = detect_link_self_collisions(model, data)
        assert home_summary.count == 0, f"{robot} home unexpected self-collision: {home_summary}"
        assert home_summary.pairs == []
        assert home_summary.min_dist_m is None

        # Ready workspace pose
        idx.set_motor_positions(data, ready_q)
        mujoco.mj_forward(model, data)
        ready_summary = detect_link_self_collisions(model, data)
        assert ready_summary.count == 0, f"{robot} ready unexpected self-collision: {ready_summary}"
        assert ready_summary.pairs == []
        assert ready_summary.min_dist_m is None


def test_self_collision_detects_forced_intersection_and_excludes_adjacent_and_object_contacts() -> None:
    # 1. B601-DM forced intersections: folded [0, -1, -2, 0, 1, 0, -3] and wrist-into-link3
    model = load_model(robot="b601dm", self_collision=True)
    data = mujoco.MjData(model)
    idx = Indexer(model)

    idx.set_motor_positions(data, [0.0, -1.0, -2.0, 0.0, 1.0, 0.0, -3.0])
    mujoco.mj_forward(model, data)
    folded_summary = detect_link_self_collisions(model, data)
    assert folded_summary.count >= 1
    assert ("link3", "link5") in folded_summary.pairs
    assert folded_summary.min_dist_m is not None and folded_summary.min_dist_m < 0.0

    idx.set_motor_positions(data, [0.0, -0.5, -0.3, 0.0, 2.0, 0.0, -3.0])
    mujoco.mj_forward(model, data)
    summary = detect_link_self_collisions(model, data)
    assert summary.count >= 1
    assert any(p in summary.pairs for p in (("end_link", "link3"), ("link3", "link6")))
    assert summary.min_dist_m is not None and summary.min_dist_m < 0.0
    # Adjacent parent-child links are never flagged
    assert ("link2", "link3") not in summary.pairs
    assert ("link3", "link4") not in summary.pairs
    assert ("link5", "link6") not in summary.pairs
    assert ("end_link", "link6") not in summary.pairs

    # 2. SO-ARM101 forced self-intersection (shoulder <-> gripper / moving_jaw)
    so_model = load_model(robot="so-arm101", self_collision=True)
    so_data = mujoco.MjData(so_model)
    so_idx = Indexer(so_model)
    so_idx.set_motor_positions(so_data, [-1.082, 0.474, 1.007, 1.53, -1.887, 0.751])
    mujoco.mj_forward(so_model, so_data)
    so_summary = detect_link_self_collisions(so_model, so_data)
    assert so_summary.count >= 1
    assert any("shoulder" in p for p in so_summary.pairs)
    assert so_summary.min_dist_m is not None and so_summary.min_dist_m < 0.0
    assert ("shoulder", "upper_arm") not in so_summary.pairs
    assert ("lower_arm", "upper_arm") not in so_summary.pairs

    # 3. Gripper-to-object contact is verified in data.contact and excluded from self-collisions
    objs = parse_objects("cube")
    obj_model = load_model(objects=objs, robot="b601dm", self_collision=True)
    obj_data = mujoco.MjData(obj_model)
    obj_idx = Indexer(obj_model)
    scene = Scene(obj_model, objs)
    scene.reset(obj_data, seed=None)
    # Place cube directly inside jaw space at ready pose so gripper geoms touch cube
    obj_idx.set_motor_positions(obj_data, [0.0, -0.8, -1.0, 0.0, 0.2, 0.0, -1.0])
    mujoco.mj_forward(obj_model, obj_data)
    tcp_pos, _ = obj_idx.tcp_pose(obj_data)
    obj_data.qpos[scene.qpos[0] : scene.qpos[0] + 3] = tcp_pos
    mujoco.mj_forward(obj_model, obj_data)

    gripper_geoms = set(JAW_GEOMS) | {"palm", "left_carriage", "right_carriage"}
    contact_geom_pairs = {
        tuple(sorted((obj_model.geom(int(c.geom1)).name, obj_model.geom(int(c.geom2)).name)))
        for c in obj_data.contact[: obj_data.ncon]
    }
    assert any(
        "cube" in pair and any(g in gripper_geoms for g in pair)
        for pair in contact_geom_pairs
    ), f"Expected gripper-to-cube contact, got {contact_geom_pairs}"

    obj_summary = detect_link_self_collisions(obj_model, obj_data)
    assert obj_summary.count == 0


def test_self_collision_respects_disabled_config_and_custom_exclusions() -> None:
    model = load_model(robot="b601dm", self_collision=True)
    data = mujoco.MjData(model)
    idx = Indexer(model)
    idx.set_motor_positions(data, [0.0, -0.5, -0.3, 0.0, 2.0, 0.0, -3.0])
    mujoco.mj_forward(model, data)

    disabled = detect_link_self_collisions(
        model, data, config=SelfCollisionConfig(enabled=False)
    )
    assert disabled.count == 0 and disabled.pairs == [] and disabled.min_dist_m is None

    baseline = detect_link_self_collisions(model, data)
    assert baseline.count >= 1
    # Excluding all reported pairs yields count == 0
    filtered = detect_link_self_collisions(
        model,
        data,
        excluded_adjacent_pairs=baseline.pairs,
    )
    assert filtered.count == 0


def test_apply_torque_noise_disabled_is_exact_and_enabled_is_seed_deterministic() -> None:
    ctrl_f32 = np.array([1.25, -0.5, 0.0, 0.35, -0.8, 0.1, 0.0], dtype=np.float32)

    # Disabled default & disabled with nonzero std/bias return exact unmodified copy & preserve dtype
    out_none = apply_torque_noise(ctrl_f32, np.random.default_rng(7), None)
    assert out_none.dtype == np.float32
    assert np.array_equal(out_none, ctrl_f32)
    assert out_none is not ctrl_f32

    cfg_off = TorqueNoiseConfig(enabled=False, std_nm=0.5, bias_nm=0.2)
    out_off = apply_torque_noise(ctrl_f32, np.random.default_rng(7), cfg_off)
    assert out_off.dtype == np.float32
    assert np.array_equal(out_off, ctrl_f32)

    ctrl = ctrl_f32.astype(np.float64)
    # Enabled: deterministic per seed, bounded with clip_nm, differs across seeds
    cfg_on = TorqueNoiseConfig(enabled=True, std_nm=0.15, bias_nm=0.05, clip_nm=0.45)
    a1 = apply_torque_noise(ctrl, np.random.default_rng(11), cfg_on)
    a2 = apply_torque_noise(ctrl, np.random.default_rng(11), cfg_on)
    b1 = apply_torque_noise(ctrl, np.random.default_rng(22), cfg_on)

    assert np.array_equal(a1, a2)
    assert not np.array_equal(a1, ctrl)
    assert not np.array_equal(a1, b1)
    assert np.all(np.abs(a1 - ctrl) <= 0.45 + 1e-12)

    with pytest.raises(ValueError):
        TorqueNoiseConfig(enabled=True, std_nm=-0.1)
    with pytest.raises(ValueError):
        apply_torque_noise([1.0, float("nan")], np.random.default_rng(0), cfg_on)


def test_run_sim_eval_20_seeds_and_cli_report(tmp_path) -> None:
    report_path = tmp_path / "sim_eval_20.json"
    exit_code = eval_main(
        [
            "--seeds", "20",
            "--start-seed", "10",
            "--robot", "b601dm",
            "--dr-lighting",
            "--dr-camera-jitter-m", "0.005",
            "--dr-camera-jitter-deg", "2.0",
            "--torque-noise-std", "0.05",
            "--max-steps", "120",
            "--output", str(report_path),
        ]
    )
    assert exit_code == 0
    assert report_path.is_file()

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema"] == EVAL_REPORT_SCHEMA
    assert report["total_seeds"] == 20
    assert report["passed_seeds"] == 20
    assert report["success_rate"] == pytest.approx(1.0)
    assert report["total_self_collisions"] == 0
    assert report["collision_free_rate"] == pytest.approx(1.0)
    assert report["visual_dr_observed_seeds"] == 0
    assert report["mean_steps"] == pytest.approx(120.0)
    assert len(report["results"]) == 20

    for idx_i, rec in enumerate(report["results"]):
        assert rec["seed"] == 10 + idx_i
        assert rec["success"] is True
        assert rec["steps"] == 120
        assert rec["duration_s"] >= 0.0
        assert rec["self_collisions"] == 0
        assert rec["self_collision_pairs"] == []
        assert rec["dr_applied"] is True
        assert rec["visual_dr_observed"] is False
        assert rec["torque_noise_std"] == pytest.approx(0.05)
        assert len(rec["final_tcp"]) == 3
        assert rec["error"] is None

    # Determinism check: re-running same config produces identical final_tcp per seed
    report_b = run_sim_eval(
        SimEvalConfig(
            seeds=20,
            start_seed=10,
            robot="b601dm",
            dr_lighting=True,
            dr_camera_jitter_m=0.005,
            dr_camera_jitter_deg=2.0,
            torque_noise_std=0.05,
            max_steps=120,
        )
    )
    assert [r["final_tcp"] for r in report_b["results"]] == [
        r["final_tcp"] for r in report["results"]
    ]


def test_sim_eval_edge_cases_zero_step_unmeasured_sc_visual_dr_and_success_fn_errors(tmp_path) -> None:
    # 1. Zero-step policy returning None on step 0 does NOT default to success=True
    zero_step_report = run_sim_eval(
        SimEvalConfig(seeds=2, robot="b601dm", max_steps=20),
        policy=lambda _obs: None,
    )
    assert zero_step_report["total_seeds"] == 2
    assert zero_step_report["passed_seeds"] == 0
    assert zero_step_report["success_rate"] == pytest.approx(0.0)
    assert all(r["steps"] == 0 and r["success"] is False for r in zero_step_report["results"])

    # 2. self_collision=False sets collision_free_rate=None and rejects fail_on_self_collision=True
    with pytest.raises(ValueError, match="fail_on_self_collision"):
        SimEvalConfig(self_collision=False, fail_on_self_collision=True)

    unmeasured_report = run_sim_eval(
        SimEvalConfig(seeds=2, robot="b601dm", max_steps=10, self_collision=False)
    )
    assert unmeasured_report["collision_free_rate"] is None
    assert unmeasured_report["total_self_collisions"] is None
    assert all(
        r["self_collisions"] is None and r["self_collision_pairs"] is None
        for r in unmeasured_report["results"]
    )

    # 3. Visual policy observing camera_state marks visual_dr_observed=True
    def visual_policy(obs):
        _ = obs["camera_state"]
        return np.array([0.0, -0.5, -0.8, 0.0, 0.2, 0.0, -3.0], dtype=np.float64)

    vis_report = run_sim_eval(
        SimEvalConfig(seeds=2, robot="b601dm", dr_lighting=True, max_steps=10),
        policy=visual_policy,
    )
    assert vis_report["visual_dr_observed_seeds"] == 2
    assert all(r["dr_applied"] is True and r["visual_dr_observed"] is True for r in vis_report["results"])

    # 4. Exception inside success_fn on seed 0 records error and continues remaining seeds
    def flaky_success(ctx):
        if ctx["seed"] == 0:
            raise RuntimeError("boom on seed 0")
        return True

    err_report = run_sim_eval(
        SimEvalConfig(seeds=2, start_seed=0, robot="b601dm", max_steps=10),
        success_fn=flaky_success,
    )
    assert err_report["total_seeds"] == 2
    assert err_report["results"][0]["success"] is False
    assert "RuntimeError: boom on seed 0" in (err_report["results"][0]["error"] or "")
    assert err_report["results"][1]["success"] is True
    assert err_report["results"][1]["error"] is None


def test_sim_eval_fails_episode_on_self_collision_and_supports_so101_and_module_cli(tmp_path) -> None:
    def colliding_policy(_obs):
        return np.array([0.0, -0.5, -0.3, 0.0, 2.0, 0.0, -3.0], dtype=np.float64)

    bad_report = run_sim_eval(
        SimEvalConfig(seeds=3, robot="b601dm", max_steps=90),
        policy=colliding_policy,
    )
    assert bad_report["total_seeds"] == 3
    assert bad_report["passed_seeds"] == 0
    assert bad_report["success_rate"] == pytest.approx(0.0)
    assert bad_report["total_self_collisions"] > 0
    assert bad_report["collision_free_rate"] == pytest.approx(0.0)
    assert all(r["self_collisions"] > 0 and r["self_collision_pairs"] for r in bad_report["results"])

    # SO-ARM101 via `python -m rebot_sim.eval_cli`
    so_out = tmp_path / "so101_eval.json"
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "rebot_sim.eval_cli",
            "--seeds",
            "5",
            "--robot",
            "so-arm101",
            "--dr-lighting",
            "--max-steps",
            "80",
            "--output",
            str(so_out),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    summary_line = json.loads(proc.stdout.strip().splitlines()[-1])
    assert summary_line["schema"] == EVAL_REPORT_SCHEMA
    assert summary_line["total_seeds"] == 5
    assert summary_line["passed_seeds"] == 5
    assert summary_line["total_self_collisions"] == 0
    so_report = json.loads(so_out.read_text(encoding="utf-8"))
    assert so_report["total_seeds"] == 5
    assert so_report["passed_seeds"] == 5
