"""Tests for physics and visual domain randomization in ``rebot_sim``."""

from __future__ import annotations

import json
import math
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from rebot_sim.domain_rand import (
    ActionDelayBuffer,
    DomainRandomizationConfig,
    DomainRandomizer,
)
from rebot_sim.env import SimEnv

_BASELINE_FIELDS = (
    "body_mass",
    "body_inertia",
    "geom_friction",
    "dof_damping",
    "cam_pos",
    "cam_quat",
    "light_diffuse",
    "light_ambient",
    "body_subtreemass",
)


def _snapshot_model(model) -> dict[str, np.ndarray]:
    return {name: getattr(model, name).copy() for name in _BASELINE_FIELDS}


def test_same_seed_reset_and_rollout_are_identical() -> None:
    cfg = DomainRandomizationConfig()
    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        render_camera="policy_cam",
        render_width=160,
        render_height=120,
        dr_config=cfg,
    ) as env:
        obs_a, info_a = env.reset(seed=123)
        snap_a = _snapshot_model(env.model)
        # Ensure dr_params is JSON-serializable for logging/auditing
        json.dumps(info_a["dr_params"])

        step_action = np.array([0.15, -0.2, 0.1, 0.05, -0.1, 0.05, 0.0], dtype=np.float64)
        step_obs_a, rew_a, _, _ = env.step(step_action)

        # Perturb with a different seed in between to verify no state leaks
        env.reset(seed=999)
        env.step(step_action * -1.0)

        obs_b, info_b = env.reset(seed=123)
        snap_b = _snapshot_model(env.model)
        step_obs_b, rew_b, _, _ = env.step(step_action)

        assert info_a["dr_params"] == info_b["dr_params"]
        for field in _BASELINE_FIELDS:
            assert np.array_equal(snap_a[field], snap_b[field]), f"mismatch in {field}"

        assert np.array_equal(obs_a["state"], obs_b["state"])
        assert np.array_equal(obs_a["velocity"], obs_b["velocity"])
        assert np.array_equal(obs_a["image"], obs_b["image"])
        assert np.array_equal(step_obs_a["state"], step_obs_b["state"])
        assert np.array_equal(step_obs_a["velocity"], step_obs_b["velocity"])
        assert np.array_equal(step_obs_a["image"], step_obs_b["image"])
        assert rew_a == rew_b


def test_different_seeds_vary_physics_camera_and_image() -> None:
    cfg = DomainRandomizationConfig(
        friction_scale_range=(0.6, 1.4),
        mass_scale_range=(0.75, 1.25),
        damping_scale_range=(0.7, 1.3),
        cam_pos_jitter_m=0.02,
        cam_rot_jitter_deg=4.0,
        light_intensity_scale_range=(0.5, 1.5),
        image_brightness_delta=25.0,
        image_contrast_scale_range=(0.75, 1.25),
        image_noise_std=5.0,
    )
    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        render_camera="policy_cam",
        render_width=160,
        render_height=120,
        dr_config=cfg,
    ) as env:
        obs_1, info_1 = env.reset(seed=10)
        snap_1 = _snapshot_model(env.model)

        obs_2, info_2 = env.reset(seed=20)
        snap_2 = _snapshot_model(env.model)

        assert info_1["dr_params"] != info_2["dr_params"]
        assert not np.array_equal(snap_1["geom_friction"], snap_2["geom_friction"])
        assert not np.array_equal(snap_1["body_mass"], snap_2["body_mass"])
        assert not np.array_equal(snap_1["body_inertia"], snap_2["body_inertia"])
        assert not np.array_equal(snap_1["dof_damping"], snap_2["dof_damping"])
        assert not np.array_equal(snap_1["cam_pos"], snap_2["cam_pos"])
        assert not np.array_equal(snap_1["cam_quat"], snap_2["cam_quat"])
        assert not np.array_equal(snap_1["light_diffuse"], snap_2["light_diffuse"])
        assert not np.array_equal(obs_1["image"], obs_2["image"])
        assert info_1["dr_params"]["image_brightness_delta"] != info_2["dr_params"]["image_brightness_delta"]


def test_twenty_resets_and_restore_baseline_zero_drift() -> None:
    cfg = DomainRandomizationConfig()
    with SimEnv(scene="pcb_suction", tool="suction", render_camera="policy_cam") as base_env:
        baseline = _snapshot_model(base_env.model)

    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        render_camera="policy_cam",
        dr_config=cfg,
    ) as dr_env:
        assert dr_env.domain_randomizer is not None
        action = np.zeros(dr_env.action_dim, dtype=np.float64)
        for seed in range(20):
            _, info = dr_env.reset(seed=seed)
            assert info["dr_params"]["enabled"] is True
            dr_env.step(action)

        # Direct restore_baseline
        dr_env.domain_randomizer.restore_baseline(dr_env.model)
        restored = _snapshot_model(dr_env.model)
        for field in _BASELINE_FIELDS:
            assert np.array_equal(restored[field], baseline[field]), f"drift in {field}"

        # Also test disabling DR via enabled=False on reset restores baseline cleanly
        dr_env.reset(seed=777)
        assert not np.array_equal(dr_env.model.geom_friction, baseline["geom_friction"])
        dr_env.dr_config = DomainRandomizationConfig(enabled=False)
        dr_env.domain_randomizer.config = dr_env.dr_config
        _, info_off = dr_env.reset(seed=777)
        assert "dr_params" not in info_off
        restored_off = _snapshot_model(dr_env.model)
        for field in _BASELINE_FIELDS:
            assert np.array_equal(restored_off[field], baseline[field]), f"drift after disable in {field}"


def test_dr_config_none_matches_baseline_exactly() -> None:
    action = np.array([0.1, -0.15, 0.2, 0.0, 0.05, -0.05, 0.0], dtype=np.float64)
    with (
        SimEnv(
            scene="pcb_suction",
            tool="suction",
            render_camera="policy_cam",
            dr_config=None,
        ) as env_a,
        SimEnv(
            scene="pcb_suction",
            tool="suction",
            render_camera="policy_cam",
        ) as env_b,
    ):
        assert env_a.domain_randomizer is None
        obs_a, info_a = env_a.reset(seed=42)
        obs_b, info_b = env_b.reset(seed=42)
        assert "dr_params" not in info_a
        assert info_a == info_b

        snap_a = _snapshot_model(env_a.model)
        snap_b = _snapshot_model(env_b.model)
        for field in _BASELINE_FIELDS:
            assert np.array_equal(snap_a[field], snap_b[field]), f"mismatch in {field}"

        assert np.array_equal(obs_a["state"], obs_b["state"])
        assert np.array_equal(obs_a["velocity"], obs_b["velocity"])
        # Two separate EGL Renderer contexts can occasionally differ by 1 LSB in rasterization.
        assert np.allclose(obs_a["image"], obs_b["image"], atol=1)

        for _ in range(5):
            oa, ra, da, ia = env_a.step(action)
            ob, rb, db, ib = env_b.step(action)
            assert ra == rb and da == db and ia == ib
            assert np.array_equal(oa["state"], ob["state"])
            assert np.array_equal(oa["velocity"], ob["velocity"])
            assert np.allclose(oa["image"], ob["image"], atol=1)


def test_action_delay_buffer_exact_fifo_and_reset() -> None:
    buf0 = ActionDelayBuffer(delay_steps=0)
    a0 = np.array([1.0, 2.0])
    out0 = buf0.push(a0)
    assert np.array_equal(out0, a0)
    out0[0] = 999.0
    assert a0[0] == 1.0  # returned copy does not alias input

    buf = ActionDelayBuffer(delay_steps=2)
    cmds = [np.array([float(i + 1), float(-(i + 1))]) for i in range(5)]
    outputs = [buf.push(c) for c in cmds]
    assert np.array_equal(outputs[0], np.zeros(2))
    assert np.array_equal(outputs[1], np.zeros(2))
    assert np.array_equal(outputs[2], cmds[0])
    assert np.array_equal(outputs[3], cmds[1])
    assert np.array_equal(outputs[4], cmds[2])
    assert len(buf) == 2

    # Reset clears stale queued commands and supports updating delay_steps
    buf.reset(delay_steps=1)
    assert len(buf) == 0
    fresh = np.array([42.0, -42.0])
    assert np.array_equal(buf.push(fresh), np.zeros(2))
    assert np.array_equal(buf.push(np.array([7.0, 8.0])), fresh)


def test_sim_env_action_delay_exact_steps() -> None:
    identity_kwargs = dict(
        friction_scale_range=(1.0, 1.0),
        mass_scale_range=(1.0, 1.0),
        damping_scale_range=(1.0, 1.0),
        obs_pos_noise_std=0.0,
        obs_vel_noise_std=0.0,
        cam_pos_jitter_m=0.0,
        cam_rot_jitter_deg=0.0,
        light_intensity_scale_range=(1.0, 1.0),
        image_brightness_delta=0.0,
        image_contrast_scale_range=(1.0, 1.0),
        image_noise_std=0.0,
    )
    cfg_delay0 = DomainRandomizationConfig(action_delay_steps_range=(0, 0), **identity_kwargs)
    cfg_delay2 = DomainRandomizationConfig(action_delay_steps_range=(2, 2), **identity_kwargs)

    with (
        SimEnv(scene="pcb_suction", tool="suction", dr_config=cfg_delay0) as env0,
        SimEnv(scene="pcb_suction", tool="suction", dr_config=cfg_delay2) as env2,
    ):
        env0.reset(seed=7)
        _, info2 = env2.reset(seed=7)
        assert info2["dr_params"]["action_delay_steps"] == 2

        act = np.array([0.4, -0.3, 0.25, 0.1, -0.15, 0.2, 0.0], dtype=np.float64)
        zeros = np.zeros_like(act)
        obs0_s0, _, _, _ = env0.step(zeros)
        obs0_s1, _, _, _ = env0.step(zeros)
        obs0_s2, _, _, _ = env0.step(act)

        # First 2 steps on delay=2 must execute zero hold, not `act`; step 2 applies `act`
        obs2_s0, _, _, _ = env2.step(act)
        obs2_s1, _, _, _ = env2.step(act * 2.0)
        obs2_s2, _, _, _ = env2.step(zeros)
        assert np.array_equal(obs2_s0["state"], obs0_s0["state"])
        assert np.array_equal(obs2_s1["state"], obs0_s1["state"])
        assert np.array_equal(obs2_s2["state"], obs0_s2["state"])
        assert not np.allclose(obs2_s2["state"], obs2_s1["state"], atol=1e-3)

        # Invalid action must still fail immediately even with active delay
        with pytest.raises(ValueError):
            env2.step([0.0, 0.0])

        # Reset must clear remaining queued commands (old `act * 2` and `zeros` must not leak)
        env0.reset(seed=7)
        obs0_fresh, _, _, _ = env0.step(zeros)
        env2.reset(seed=7)
        obs2_fresh, _, _, _ = env2.step(act)
        assert np.array_equal(obs2_fresh["state"], obs0_fresh["state"])


def test_obs_and_image_noise_bounds_and_non_mutation() -> None:
    cfg = DomainRandomizationConfig(
        obs_pos_noise_std=0.05,
        obs_vel_noise_std=0.1,
        image_brightness_delta=120.0,
        image_contrast_scale_range=(1.8, 2.0),
        image_noise_std=25.0,
    )
    with SimEnv(scene="pcb_suction", tool="suction") as env:
        rand = DomainRandomizer(env.model, cfg)
        rng = np.random.default_rng(5)

        obs_in = {
            "state": np.zeros(7, dtype=np.float32),
            "velocity": np.ones(7, dtype=np.float32),
            "io": {"vacuum_on": False},
        }
        state_copy = obs_in["state"].copy()
        vel_copy = obs_in["velocity"].copy()
        obs_out = rand.apply_obs_noise(obs_in, rng)
        assert np.array_equal(obs_in["state"], state_copy)
        assert np.array_equal(obs_in["velocity"], vel_copy)
        assert obs_out["state"].dtype == np.float32
        assert obs_out["velocity"].dtype == np.float32
        assert not np.array_equal(obs_out["state"], state_copy)
        assert not np.array_equal(obs_out["velocity"], vel_copy)

        img_in = np.array(
            [[[0, 128, 255], [10, 200, 250]], [[5, 64, 240], [255, 0, 127]]],
            dtype=np.uint8,
        )
        img_copy = img_in.copy()
        img_out = rand.apply_image_noise(img_in, rng)
        assert np.array_equal(img_in, img_copy)
        assert img_out.dtype == np.uint8
        assert img_out.shape == img_in.shape
        assert int(img_out.min()) >= 0 and int(img_out.max()) <= 255


@pytest.mark.parametrize(
    "kwargs",
    [
        {"enabled": "true"},
        {"friction_scale_range": (1.3, 0.7)},
        {"friction_scale_range": (-0.1, 1.0)},
        {"friction_scale_range": (1.0,)},
        {"friction_scale_range": (math.nan, 1.0)},
        {"friction_scale_range": (1e308, 1e308)},
        {"mass_scale_range": (0.0, 1.2)},
        {"mass_scale_range": (-0.5, 1.2)},
        {"mass_scale_range": (1.2, 0.8)},
        {"mass_scale_range": (1e308, 1e308)},
        {"damping_scale_range": (-0.1, 1.0)},
        {"damping_scale_range": (1.2, 0.8)},
        {"damping_scale_range": (1e308, 1e308)},
        {"action_delay_steps_range": (2, 0)},
        {"action_delay_steps_range": (-1, 2)},
        {"action_delay_steps_range": (0, 1001)},
        {"action_delay_steps_range": (2**63, 2**63)},
        {"action_delay_steps_range": (0.5, 2.0)},
        {"action_delay_steps_range": (False, True)},
        {"obs_pos_noise_std": -0.001},
        {"obs_pos_noise_std": 1e40},
        {"obs_pos_noise_std": math.inf},
        {"obs_pos_noise_std": True},
        {"obs_vel_noise_std": -0.1},
        {"obs_vel_noise_std": 1e40},
        {"cam_pos_jitter_m": -0.01},
        {"cam_rot_jitter_deg": -1.0},
        {"light_intensity_scale_range": (-0.1, 1.0)},
        {"light_intensity_scale_range": (1.5, 0.5)},
        {"light_intensity_scale_range": (1e308, 1e308)},
        {"image_brightness_delta": -5.0},
        {"image_contrast_scale_range": (-0.2, 1.2)},
        {"image_contrast_scale_range": (1.2, 0.8)},
        {"image_contrast_scale_range": (1e308, 1e308)},
        {"image_noise_std": -1.0},
    ],
)
def test_invalid_config_fails_closed(kwargs: dict) -> None:
    with pytest.raises(ValueError):
        DomainRandomizationConfig(**kwargs)


def test_invalid_delay_buffer_and_env_args_fail_closed() -> None:
    with pytest.raises(ValueError):
        ActionDelayBuffer(delay_steps=-1)
    with pytest.raises(ValueError):
        ActionDelayBuffer(delay_steps=True)
    with pytest.raises(ValueError):
        ActionDelayBuffer(delay_steps=1.5)  # type: ignore[arg-type]

    with pytest.raises(TypeError):
        SimEnv(scene="pcb_suction", tool="suction", dr_config={"enabled": True})  # type: ignore[arg-type]


def test_config_is_frozen_immutable() -> None:
    cfg = DomainRandomizationConfig()
    with pytest.raises(FrozenInstanceError):
        cfg.mass_scale_range = (-1.0, 1.0)  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        cfg.enabled = False  # type: ignore[misc]


def test_restore_baseline_and_invalid_seed_do_not_mutate_caller_data_or_model() -> None:
    cfg = DomainRandomizationConfig()
    with SimEnv(scene="pcb_suction", tool="suction", dr_config=cfg) as env:
        assert env.domain_randomizer is not None
        _, info = env.reset(seed=11)
        assert info["dr_params"]["seed"] == 11
        model_before = _snapshot_model(env.model)

        env.data.qpos[0] = 0.7
        env.data.qvel[0] = 0.8
        qpos_before = env.data.qpos.copy()
        qvel_before = env.data.qvel.copy()

        # Invalid seed must fail closed before touching model or caller data
        with pytest.raises(ValueError):
            env.reset(seed=-1)
        with pytest.raises(ValueError):
            env.domain_randomizer.apply(env.model, seed=True)  # type: ignore[arg-type]

        assert np.array_equal(env.data.qpos, qpos_before)
        assert np.array_equal(env.data.qvel, qvel_before)
        for field in _BASELINE_FIELDS:
            assert np.array_equal(getattr(env.model, field), model_before[field])

        # restore_baseline must not mutate caller data.qpos/qvel via mj_setConst
        env.domain_randomizer.restore_baseline(env.model)
        assert np.array_equal(env.data.qpos, qpos_before)
        assert np.array_equal(env.data.qvel, qvel_before)

        # apply(model, seed) must also leave caller data untouched
        env.domain_randomizer.apply(env.model, seed=22)
        assert np.array_equal(env.data.qpos, qpos_before)
        assert np.array_equal(env.data.qvel, qvel_before)


def test_dr_rng_is_decoupled_from_scene_default_rng() -> None:
    cfg = DomainRandomizationConfig()
    with SimEnv(scene="pcb_suction", tool="suction", dr_config=cfg) as env:
        _, info = env.reset(seed=42)
        raw_rng = np.random.default_rng(42)
        raw_first_draws = raw_rng.uniform(*cfg.friction_scale_range, size=env.model.ngeom)
        assert not np.allclose(info["dr_params"]["friction_scales"], raw_first_draws)


def test_obs_and_image_noise_rng_streams_are_independent() -> None:
    cfg = DomainRandomizationConfig()
    action = np.array([0.1, -0.2, 0.15, 0.0, 0.05, -0.05, 0.0], dtype=np.float64)
    with (
        SimEnv(
            scene="pcb_suction",
            tool="suction",
            render_camera=None,
            dr_config=cfg,
        ) as env_headless,
        SimEnv(
            scene="pcb_suction",
            tool="suction",
            render_camera="policy_cam",
            render_width=160,
            render_height=120,
            dr_config=cfg,
        ) as env_cam,
    ):
        obs_h, info_h = env_headless.reset(seed=55)
        obs_c, info_c = env_cam.reset(seed=55)
        assert "image" not in obs_h and "image" in obs_c
        assert info_h["dr_params"]["friction_scales"] == info_c["dr_params"]["friction_scales"]
        assert np.array_equal(obs_h["state"], obs_c["state"])
        assert np.array_equal(obs_h["velocity"], obs_c["velocity"])

        for _ in range(4):
            sh, _, _, _ = env_headless.step(action)
            sc, _, _, _ = env_cam.step(action)
            assert np.array_equal(sh["state"], sc["state"])
            assert np.array_equal(sh["velocity"], sc["velocity"])


def test_obs_noise_nonfinite_overflow_and_delay_buffer_atomic_errors() -> None:
    cfg = DomainRandomizationConfig(obs_pos_noise_std=0.1, obs_vel_noise_std=0.1)
    with SimEnv(scene="pcb_suction", tool="suction") as env:
        rand = DomainRandomizer(env.model, cfg)
        rng = np.random.default_rng(0)
        with pytest.raises(ValueError, match="non-finite"):
            rand.apply_obs_noise({"state": np.array([1e40], dtype=np.float64)}, rng)
        with pytest.raises(ValueError, match="non-finite"):
            rand.apply_obs_noise({"velocity": np.array([np.nan], dtype=np.float32)}, rng)

    # ActionDelayBuffer must not mutate or clear queue on invalid reset/push
    buf = ActionDelayBuffer(delay_steps=2)
    a0 = np.array([1.0, 2.0])
    a1 = np.array([3.0, 4.0])
    assert np.array_equal(buf.push(a0), np.zeros(2))
    assert np.array_equal(buf.push(a1), np.zeros(2))
    assert len(buf) == 2

    with pytest.raises(ValueError):
        buf.reset(delay_steps=-1)
    assert len(buf) == 2 and buf.delay_steps == 2

    with pytest.raises(ValueError):
        buf.push(np.array([5.0, 6.0]), default_action=np.array([0.0]))
    assert len(buf) == 2

    with pytest.raises(ValueError):
        buf.push(np.array([5.0, 6.0, 7.0]))
    assert len(buf) == 2

    with pytest.raises(ValueError):
        buf.push(np.array([np.nan, 6.0]))
    assert len(buf) == 2

    # Valid next push still yields a0 untouched
    assert np.array_equal(buf.push(np.array([5.0, 6.0])), a0)


def test_apply_rejects_mass_overflow_without_mutating_model() -> None:
    cfg = DomainRandomizationConfig(mass_scale_range=(2.0, 2.0))
    with SimEnv(scene="pcb_suction", tool="suction") as env:
        rand = DomainRandomizer(env.model, cfg)
        rand._baseline["body_mass"][1] = 1e308
        env.model.body_mass[1] = 1e308
        before = _snapshot_model(env.model)
        with pytest.raises(ValueError, match="non-finite"):
            rand.apply(env.model, seed=1)
        after = _snapshot_model(env.model)
        for field in _BASELINE_FIELDS:
            assert np.array_equal(after[field], before[field]), f"mutated {field} on overflow"


def test_reset_validates_seed_without_dr_and_normalizes_np_int() -> None:
    with SimEnv(scene="pcb_suction", tool="suction", dr_config=None) as env:
        obs_py, info_py = env.reset(seed=42)
        env.data.qpos[0] = 0.7
        env.data.qvel[0] = 0.8
        qpos_before = env.data.qpos.copy()
        qvel_before = env.data.qvel.copy()

        for bad_seed in (-1, True, False, 1.5, "42"):
            with pytest.raises(ValueError):
                env.reset(seed=bad_seed)  # type: ignore[arg-type]
            assert np.array_equal(env.data.qpos, qpos_before)
            assert np.array_equal(env.data.qvel, qvel_before)

        obs_np, info_np = env.reset(seed=np.int64(42))  # type: ignore[arg-type]
        assert info_np == info_py
        assert np.array_equal(obs_np["state"], obs_py["state"])
        assert np.array_equal(obs_np["velocity"], obs_py["velocity"])

    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        dr_config=DomainRandomizationConfig(),
    ) as dr_env:
        obs_py, info_py = dr_env.reset(seed=42)
        obs_np, info_np = dr_env.reset(seed=np.int64(42))  # type: ignore[arg-type]
        assert type(info_np["dr_params"]["seed"]) is int
        assert info_np == info_py
        assert np.array_equal(obs_np["state"], obs_py["state"])
        assert np.array_equal(obs_np["velocity"], obs_py["velocity"])
