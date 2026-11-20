"""Tests for SimEnv strict action validation and LeRobotSimCollector (LeRobot v3.0)."""

from __future__ import annotations

import hashlib
import json
import os
import struct
from pathlib import Path
from unittest.mock import patch

import av
import numpy as np
import pytest

from rebot_sim.collector import CollectorError, LeRobotSimCollector, probe_video
from rebot_sim.env import SimEnv


# --------------------------------------------------------------------------- minimal Parquet reader for tests


class _CompactReader:
    """Minimal Thrift TCompactProtocol reader to verify Parquet FileMetaData and pages in tests."""

    def __init__(self, data: bytes, pos: int = 0) -> None:
        self.data = data
        self.pos = pos

    def _u8(self) -> int:
        b = self.data[self.pos]
        self.pos += 1
        return b

    def _uvarint(self) -> int:
        res = 0
        shift = 0
        while True:
            b = self._u8()
            res |= (b & 0x7F) << shift
            if not (b & 0x80):
                return res
            shift += 7

    def _zigzag_dec(self, n: int) -> int:
        return (n >> 1) ^ -(n & 1)

    def read_i32(self) -> int:
        return self._zigzag_dec(self._uvarint())

    def read_i64(self) -> int:
        return self._zigzag_dec(self._uvarint())

    def read_binary(self) -> bytes:
        n = self._uvarint()
        out = self.data[self.pos : self.pos + n]
        self.pos += n
        return out

    def read_list_hdr(self) -> tuple[int, int]:
        b = self._u8()
        sz = (b >> 4) & 0x0F
        etype = b & 0x0F
        if sz == 15:
            sz = self._uvarint()
        return etype, sz

    def skip(self, ttype: int) -> None:
        if ttype in (1, 2):
            return
        if ttype == 4:
            self._uvarint()
        elif ttype in (5, 6):
            self._uvarint()
        elif ttype == 7:
            self.pos += 8
        elif ttype == 8:
            self.read_binary()
        elif ttype == 9:
            et, cnt = self.read_list_hdr()
            for _ in range(cnt):
                self.skip(et)
        elif ttype == 12:
            self.read_struct()
        else:
            raise AssertionError(f"Unexpected Thrift type {ttype}")

    def read_struct(self) -> dict[int, tuple[int, object]]:
        fields: dict[int, tuple[int, object]] = {}
        last_fid = 0
        while True:
            b = self._u8()
            if b == 0:
                return fields
            delta = (b >> 4) & 0x0F
            ttype = b & 0x0F
            fid = last_fid + delta if delta else self.read_i32()
            last_fid = fid
            if ttype in (1, 2):
                val: object = ttype == 1
            elif ttype in (5, 6):
                val = self._zigzag_dec(self._uvarint())
            elif ttype == 8:
                val = self.read_binary()
            elif ttype == 9:
                et, cnt = self.read_list_hdr()
                items = []
                for _ in range(cnt):
                    if et in (5, 6):
                        items.append(self._zigzag_dec(self._uvarint()))
                    elif et == 8:
                        items.append(self.read_binary())
                    elif et == 12:
                        items.append(self.read_struct())
                    else:
                        self.skip(et)
                val = items
            elif ttype == 12:
                val = self.read_struct()
            else:
                self.skip(ttype)
                val = None
            fields[fid] = (ttype, val)


def _decode_rle_levels(buf: bytes, num_values: int, bit_width: int) -> list[int]:
    if bit_width == 0:
        return [0] * num_values
    out: list[int] = []
    pos = 0
    mask = (1 << bit_width) - 1
    while pos < len(buf) and len(out) < num_values:
        hdr = 0
        shift = 0
        while True:
            b = buf[pos]
            pos += 1
            hdr |= (b & 0x7F) << shift
            if not (b & 0x80):
                break
            shift += 7
        if hdr & 1:
            n_groups = hdr >> 1
            for _ in range(n_groups):
                acc = int.from_bytes(buf[pos : pos + bit_width], "little")
                pos += bit_width
                for bit in range(8):
                    if len(out) < num_values:
                        out.append((acc >> (bit * bit_width)) & mask)
        else:
            cnt = hdr >> 1
            val = buf[pos]
            pos += 1
            out.extend([val] * min(cnt, num_values - len(out)))
    return out


def _read_parquet_columns(path: Path) -> tuple[dict[str, list], int, dict[str, str]]:
    """Read an uncompressed PLAIN Parquet file (scalar + nested list columns) without pyarrow."""
    raw = path.read_bytes()
    assert raw[:4] == b"PAR1" and raw[-4:] == b"PAR1"
    meta_len = struct.unpack("<I", raw[-8:-4])[0]
    meta_start = len(raw) - 8 - meta_len
    meta = _CompactReader(raw, meta_start).read_struct()
    num_rows = int(meta[3][1])
    schema_elems = meta[2][1]
    kv_meta: dict[str, str] = {}
    if 5 in meta:
        for kv in meta[5][1]:
            kv_meta[kv[1][1].decode("utf-8")] = kv[2][1].decode("utf-8")

    # Determine nesting depth for each leaf column from schema_elems (DFS pre-order)
    col_depth: dict[tuple[str, ...], int] = {}

    def _walk_schema(idx: int, path_prefix: tuple[str, ...], rep_depth: int) -> int:
        se = schema_elems[idx]
        name = se[4][1].decode("utf-8")
        cur_path = path_prefix + (name,)
        is_repeated = 3 in se and se[3][1] == 2
        next_rep = rep_depth + (1 if is_repeated else 0)
        n_children = se[5][1] if 5 in se else 0
        if n_children == 0:
            col_depth[cur_path] = next_rep
            return idx + 1
        cur = idx + 1
        for _ in range(n_children):
            cur = _walk_schema(cur, cur_path, next_rep)
        return cur

    cur_idx = 1
    for _ in range(schema_elems[0][5][1]):
        cur_idx = _walk_schema(cur_idx, (), 0)

    rg = meta[4][1][0]
    out: dict[str, list] = {}
    for cc in rg[1][1]:
        cm = cc[3][1]
        phys_type = cm[1][1]
        path_parts = tuple(p.decode("utf-8") for p in cm[3][1])
        num_values = int(cm[5][1])
        page_off = int(cm[9][1])

        pr = _CompactReader(raw, page_off)
        ph = pr.read_struct()
        assert ph[1][1] == 0  # DATA_PAGE
        payload_start = pr.pos
        payload_end = payload_start + int(ph[2][1])
        payload = raw[payload_start:payload_end]

        pos = 0
        depth = col_depth[path_parts]
        rep_levels: list[int] = []
        if depth > 0:
            rep_len = struct.unpack_from("<I", payload, pos)[0]
            pos += 4
            rep_levels = _decode_rle_levels(payload[pos : pos + rep_len], num_values, depth.bit_length())
            pos += rep_len
            def_len = struct.unpack_from("<I", payload, pos)[0]
            pos += 4 + def_len

        val_bytes = payload[pos:]
        if phys_type == 4:  # FLOAT
            flat = np.frombuffer(val_bytes, dtype="<f4", count=num_values).tolist()
        elif phys_type == 5:  # DOUBLE
            flat = np.frombuffer(val_bytes, dtype="<f8", count=num_values).tolist()
        elif phys_type == 2:  # INT64
            flat = np.frombuffer(val_bytes, dtype="<i8", count=num_values).tolist()
        elif phys_type == 6:  # BYTE_ARRAY
            flat = []
            vp = 0
            for _ in range(num_values):
                slen = struct.unpack_from("<I", val_bytes, vp)[0]
                vp += 4
                flat.append(val_bytes[vp : vp + slen].decode("utf-8"))
                vp += slen
        else:
            raise AssertionError(f"Unsupported physical type {phys_type}")

        col_name = path_parts[0]
        if depth == 0:
            out[col_name] = flat
        elif depth == 1:
            rows: list[list] = []
            for rbit, v in zip(rep_levels, flat):
                if rbit == 0:
                    rows.append([v])
                else:
                    rows[-1].append(v)
            out[col_name] = rows
        else:
            # Reconstruct arbitrary depth nested list using rep_levels
            rows = []
            stack: list[list] = []
            for r, v in zip(rep_levels, flat):
                if r == 0:
                    stack = [[] for _ in range(depth)]
                    rows.append(stack[0])
                    for lvl in range(depth - 1):
                        stack[lvl].append(stack[lvl + 1])
                else:
                    for lvl in range(r, depth):
                        new_list: list = []
                        stack[lvl - 1].append(new_list)
                        stack[lvl] = new_list
                stack[depth - 1].append(v)
            out[col_name] = rows

    return out, num_rows, kv_meta


def _snapshot_tree(root: Path) -> dict[str, str]:
    snap: dict[str, str] = {}
    for p in sorted(root.rglob("*")):
        if p.is_file() and p.name != ".collector.lock":
            snap[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return snap


# --------------------------------------------------------------------------- SimEnv.step() & symmetric validation


def test_sim_env_step_rejects_short_long_nonfinite_and_bad_shapes():
    with SimEnv(scene="pcb_suction", tool="suction", control_mode="position") as env:
        env.reset(seed=42)
        assert env.action_dim == 7
        assert env.n_joints == 7
        assert len(env.joint_names) == 7

        init_target = np.array([0.15, -0.25, 0.1, 0.0, 0.0, 0.0, 0.0])
        env.step(init_target)
        saved_qt = env.motors.q_t.copy()
        saved_time = float(env.data.time)
        saved_io = env.pack_scene.io_state(env.data)

        # Short & empty actions must fail without mutating targets, sim time, or I/O
        for bad_act in ([0.9, 0.9], []):
            with pytest.raises(ValueError, match="length mismatch"):
                env.step(bad_act, io_updates={"vacuum": True})
            assert np.allclose(env.motors.q_t, saved_qt)
            assert float(env.data.time) == saved_time
            assert env.pack_scene.io_state(env.data) == saved_io

        # Over-long action must fail instead of truncating
        with pytest.raises(ValueError, match="length mismatch"):
            env.step(np.zeros(8))
        assert np.allclose(env.motors.q_t, saved_qt)
        assert float(env.data.time) == saved_time

        # Non-finite values (NaN, +Inf, -Inf)
        for bad in (np.nan, np.inf, -np.inf):
            vec = np.zeros(7)
            vec[3] = bad
            with pytest.raises(ValueError, match="non-finite"):
                env.step(vec)
            assert np.allclose(env.motors.q_t, saved_qt)
            assert float(env.data.time) == saved_time

        # (1, dof) ndarray AND nested (1, dof) list are both squeezed and accepted
        obs2, _r, _d, info2 = env.step([[0.0] * 7])
        assert obs2["state"].shape == (7,)
        assert info2["sim_time_s"] > saved_time
        assert np.allclose(env.motors.q_t, np.zeros(7))

        obs3, _r, _d, _info3 = env.step(np.zeros((1, 7)))
        assert obs3["state"].shape == (7,)


def test_sim_env_step_velocity_mode_and_so101_dof():
    with SimEnv(scene="pcb_suction", tool="suction", control_mode="velocity") as env:
        env.reset(seed=7)
        assert env.action_dim == env.model.nu == 7
        env.step([0.1] * 7)
        saved_ctrl = env.data.ctrl.copy()
        saved_time = float(env.data.time)

        with pytest.raises(ValueError, match="length mismatch"):
            env.step([0.1, 0.2, 0.3])
        with pytest.raises(ValueError, match="length mismatch"):
            env.step(np.zeros(9))
        with pytest.raises(ValueError, match="non-finite"):
            env.step([0.0, np.nan, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert np.allclose(env.data.ctrl, saved_ctrl)
        assert float(env.data.time) == saved_time

        obs, _r, _d, _i = env.step(np.zeros((1, 7)))
        assert obs["state"].shape == (7,)

    with SimEnv(scene=None, robot="so-arm101", control_mode="position") as so_env:
        so_env.reset(seed=1)
        assert so_env.action_dim == 6
        with pytest.raises(ValueError, match="length mismatch"):
            so_env.step(np.zeros(7))
        with pytest.raises(ValueError, match="length mismatch"):
            so_env.step(np.zeros(5))
        obs, _r, _d, _i = so_env.step(np.zeros(6))
        assert obs["state"].shape == (6,)


def test_symmetric_strict_real_numeric_validation_across_env_and_collector(tmp_path: Path):
    dof = 7
    invalid_inputs = [
        ["0.0"] * dof,
        np.array(["0.0"] * dof),
        np.array([0.0] * (dof - 1) + [None], dtype=object),
        np.array([0.0] * dof, dtype=object),
        [1 + 0j] + [0.0] * (dof - 1),
        np.array([1 + 2j] * dof),
        [True] + [0.0] * (dof - 1),
        [[True] + [0.0] * (dof - 1)],
        np.array([False] * dof),
        0.0,
        np.zeros((2, dof)),
        np.zeros((dof, 1)),
        [[0.0] * dof, [0.0] * dof],
        "not_an_action",
        None,
    ]
    with SimEnv(scene="pcb_suction", tool="suction") as env:
        env.reset(seed=1)
        saved_qt = env.motors.q_t.copy()
        with LeRobotSimCollector(
            tmp_path / "sym",
            fps=50,
            profile_id="rebot-b601dm",
            unit_mode="rad",
            joint_names=env.joint_names,
        ) as collector:
            for bad in invalid_inputs:
                with pytest.raises(ValueError):
                    env.step(bad)
                assert np.allclose(env.motors.q_t, saved_qt)

                collector.start_episode(task="sym_check")
                with pytest.raises(CollectorError):
                    collector.record_step(0.0, [0.0] * dof, bad)
                with pytest.raises(CollectorError, match="invalidated"):
                    collector.save_episode()
                collector.discard_episode()

            # Nested (1, dof) Python list is accepted symmetrically by both
            collector.start_episode(task="nested_ok")
            obs, _r, _d, info = env.step([[0.0] * dof])
            idx = collector.record_step(0.0, [[0.0] * dof], [[0.0] * dof])
            assert idx == 0
            assert obs["state"].shape == (dof,)
            assert info["sim_time_s"] > 0.0
            assert collector.save_episode() == 0


# --------------------------------------------------------------------------- Collector pre-checks, lock & path safety


def test_collector_requires_profile_id_unit_mode_and_safe_camera_roles(tmp_path: Path):
    joints = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    with pytest.raises(CollectorError, match="profile_id"):
        LeRobotSimCollector(tmp_path / "ds1", fps=50, profile_id=None, unit_mode="rad", joint_names=joints)
    with pytest.raises(CollectorError, match="profile_id"):
        LeRobotSimCollector(tmp_path / "ds2", fps=50, profile_id="   ", unit_mode="rad", joint_names=joints)
    with pytest.raises(CollectorError, match="unit_mode"):
        LeRobotSimCollector(tmp_path / "ds3", fps=50, profile_id="rebot-b601dm", unit_mode=None, joint_names=joints)
    with pytest.raises(CollectorError, match="unit_mode"):
        LeRobotSimCollector(tmp_path / "ds4", fps=50, profile_id="rebot-b601dm", unit_mode="auto", joint_names=joints)
    with pytest.raises(CollectorError, match="fps"):
        LeRobotSimCollector(tmp_path / "ds5", fps=0, profile_id="rebot-b601dm", unit_mode="rad", joint_names=joints)

    # BLOCKER #1: reject path-traversal or unsafe camera role names
    for bad_role in ("../evil", "cam/wrist", "cam\\wrist", "/abs", "a..b", ".hidden", ""):
        with pytest.raises(CollectorError, match="Invalid camera role"):
            LeRobotSimCollector(
                tmp_path / f"bad_role_{abs(hash(bad_role))}",
                fps=50,
                profile_id="rebot-b601dm",
                unit_mode="rad",
                joint_names=joints,
                required_camera_roles=(bad_role,),
            )
        with pytest.raises(CollectorError, match="Invalid camera role"):
            LeRobotSimCollector(
                tmp_path / f"bad_shape_{abs(hash(bad_role))}",
                fps=50,
                profile_id="rebot-b601dm",
                unit_mode="rad",
                joint_names=joints,
                camera_shapes={bad_role: (32, 32, 3)},
            )


def test_collector_exclusive_root_lock_prevents_concurrent_writers(tmp_path: Path):
    ds_root = tmp_path / "locked_ds"
    joints = ["j1", "j2"]
    with LeRobotSimCollector(ds_root, fps=20, profile_id="p1", unit_mode="rad", joint_names=joints) as c1:
        with pytest.raises(CollectorError, match="locked"):
            LeRobotSimCollector(ds_root, fps=20, profile_id="p1", unit_mode="rad", joint_names=joints)
        c1.start_episode("t")
        c1.record_step(0.0, [0.0, 0.0], [0.1, 0.1])
        assert c1.save_episode() == 0

    # After c1 closes, root is non-empty so a second collector cannot overwrite it either
    with pytest.raises(CollectorError, match="not empty"):
        LeRobotSimCollector(ds_root, fps=20, profile_id="p1", unit_mode="rad", joint_names=joints)

    # When closed with zero episodes saved, lock is released while keeping lock inode stable
    empty_root = tmp_path / "reusable_ds"
    with LeRobotSimCollector(empty_root, fps=20, profile_id="p1", unit_mode="rad", joint_names=joints):
        lock_ino_1 = (empty_root / ".collector.lock").stat().st_ino
    assert (empty_root / ".collector.lock").exists()
    with LeRobotSimCollector(empty_root, fps=20, profile_id="p1", unit_mode="rad", joint_names=joints) as c2:
        assert (empty_root / ".collector.lock").stat().st_ino == lock_ino_1
        c2.start_episode("t")
        c2.record_step(0.0, [0.0, 0.0], [0.1, 0.1])
        assert c2.save_episode() == 0


# --------------------------------------------------------------------------- Multi-episode collection + video alignment


def test_collector_multi_episode_writes_valid_lerobot_v3_dataset(tmp_path: Path):
    ds_root = tmp_path / "pcb_dataset"
    ep0_states: list[np.ndarray] = []
    ep0_actions: list[np.ndarray] = []
    ep0_images: list[np.ndarray] = []
    ep1_states: list[np.ndarray] = []
    ep1_actions: list[np.ndarray] = []

    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        robot="b601dm",
        render_camera="wrist",
        render_width=64,
        render_height=48,
    ) as env:
        with LeRobotSimCollector(
            ds_root,
            fps=50,
            profile_id="rebot-b601dm",
            unit_mode="rad",
            robot_type="rebot_b601_dm",
            joint_names=env.joint_names,
            required_camera_roles=("wrist",),
            arm_dof=6,
            gripper_joint=6,
            env=env,
        ) as collector:
            # Episode 0: 5 steps with strong arm sweep so frame 0 != frame 4 visually
            def policy0(obs: dict, step: int) -> np.ndarray:
                act = np.array([0.35 * step, -0.45, -0.35, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                ep0_states.append(obs["state"].copy())
                ep0_actions.append(act.copy())
                ep0_images.append(obs["image"].copy())
                return act

            idx0 = collector.collect_episode(env, policy0, max_steps=5, task="pick_pcb", seed=10)
            assert idx0 == 0

            # Episode 1: 4 steps with different task
            def policy1(obs: dict, step: int) -> np.ndarray:
                act = np.array([-0.15 * step, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                ep1_states.append(obs["state"].copy())
                ep1_actions.append(act.copy())
                return act

            idx1 = collector.collect_episode(env, policy1, max_steps=4, task="place_pcb", seed=20)
            assert idx1 == 1

        expected_joints = env.joint_names

    # Verify meta/info.json
    info = json.loads((ds_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["codebase_version"] == "v3.0"
    assert info["robot_type"] == "rebot_b601_dm"
    assert info["profile_id"] == "rebot-b601dm"
    assert info["unit_mode"] == "rad"
    assert info["rebot_unit_mode"] == "rad"
    assert info["fps"] == 50
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 9
    assert info["total_tasks"] == 2
    assert info["splits"] == {"train": "0:2"}
    assert info["features"]["observation.state"]["names"] == expected_joints
    assert info["features"]["action"]["names"] == expected_joints
    assert info["features"]["observation.images.wrist"]["dtype"] == "video"
    assert info["features"]["observation.images.wrist"]["shape"] == [48, 64, 3]
    assert info["video_path"] is not None

    # Verify meta/stats.json matches exact aggregate statistics
    stats = json.loads((ds_root / "meta" / "stats.json").read_text(encoding="utf-8"))
    all_states = np.stack(ep0_states + ep1_states).astype(np.float64)
    all_actions = np.stack(ep0_actions + ep1_actions).astype(np.float64)
    assert np.allclose(stats["observation.state"]["mean"], all_states.mean(axis=0), atol=1e-6)
    assert np.allclose(stats["observation.state"]["std"], all_states.std(axis=0), atol=1e-6)
    assert np.allclose(stats["observation.state"]["min"], all_states.min(axis=0), atol=1e-6)
    assert np.allclose(stats["observation.state"]["max"], all_states.max(axis=0), atol=1e-6)
    assert stats["observation.state"]["count"] == [9]
    assert np.allclose(stats["action"]["mean"], all_actions.mean(axis=0), atol=1e-6)
    assert np.asarray(stats["observation.images.wrist"]["mean"]).shape == (3, 1, 1)
    assert stats["observation.images.wrist"]["count"] == [9]

    # Pre-step timestamp alignment: frame 0 starts at 0.0 (matching video pts=0)
    expected_ep0_ts = [0.0, 0.02, 0.04, 0.06, 0.08]
    expected_ep1_ts = [0.0, 0.02, 0.04, 0.06]

    pq0, nrows0, kv0 = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-000.parquet")
    assert nrows0 == 5
    assert "huggingface" in kv0
    assert pq0["frame_index"] == [0, 1, 2, 3, 4]
    assert pq0["episode_index"] == [0] * 5
    assert pq0["index"] == [0, 1, 2, 3, 4]
    assert pq0["task_index"] == [0] * 5
    assert np.allclose(pq0["timestamp"], expected_ep0_ts, atol=1e-6)
    assert np.allclose(pq0["observation.state"], np.stack(ep0_states), atol=1e-6)
    assert np.allclose(pq0["action"], np.stack(ep0_actions), atol=1e-6)

    pq1, nrows1, _kv1 = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-001.parquet")
    assert nrows1 == 4
    assert pq1["frame_index"] == [0, 1, 2, 3]
    assert pq1["episode_index"] == [1] * 4
    assert pq1["index"] == [5, 6, 7, 8]
    assert pq1["task_index"] == [1] * 4
    assert np.allclose(pq1["timestamp"], expected_ep1_ts, atol=1e-6)
    assert np.allclose(pq1["observation.state"], np.stack(ep1_states), atol=1e-6)
    assert np.allclose(pq1["action"], np.stack(ep1_actions), atol=1e-6)

    tasks_pq, tasks_rows, tasks_kv = _read_parquet_columns(ds_root / "meta" / "tasks.parquet")
    assert tasks_rows == 2
    assert "pandas" in tasks_kv
    assert tasks_pq["task_index"] == [0, 1]
    assert tasks_pq["task"] == ["pick_pcb", "place_pcb"]

    eps_pq, eps_rows, _ = _read_parquet_columns(ds_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
    assert eps_rows == 2
    assert eps_pq["episode_index"] == [0, 1]
    assert eps_pq["length"] == [5, 4]
    assert eps_pq["dataset_from_index"] == [0, 5]
    assert eps_pq["dataset_to_index"] == [5, 9]
    assert np.asarray(eps_pq["stats/observation.images.wrist/mean"]).shape == (2, 3, 1, 1)

    # Verify video timestamps and decoded first/last frame contents align with pre-step observations
    vid0 = ds_root / "videos" / "observation.images.wrist" / "chunk-000" / "file-000.mp4"
    with av.open(str(vid0)) as container:
        stream = container.streams.video[0]
        tb = float(stream.time_base)
        frames = [(float(f.pts * tb), f.to_ndarray(format="rgb24")) for f in container.decode(video=0)]
    assert len(frames) == 5
    assert frames[0][0] == pytest.approx(pq0["timestamp"][0], abs=1e-4)
    assert frames[-1][0] == pytest.approx(pq0["timestamp"][-1], abs=1e-4)
    assert np.mean(np.abs(frames[0][1].astype(float) - ep0_images[0].astype(float))) < 12.0
    assert np.mean(np.abs(frames[-1][1].astype(float) - ep0_images[-1].astype(float))) < 12.0

    vid1 = ds_root / "videos" / "observation.images.wrist" / "chunk-000" / "file-001.mp4"
    with av.open(str(vid1)) as container:
        assert sum(1 for _ in container.decode(video=0)) == 4


def test_collector_1dof_list_feature_and_state_only_roundtrip(tmp_path: Path):
    ds_root = tmp_path / "one_dof"
    with LeRobotSimCollector(
        ds_root,
        fps=25,
        profile_id="test-1dof",
        unit_mode="so101_norm",
        joint_names=["j1"],
        required_camera_roles=(),
    ) as collector:
        collector.start_episode(task="reach")
        collector.record_step(0.0, [1.0], [1.5])
        collector.record_step(0.04, [1.5], [2.0])
        assert collector.save_episode() == 0

    info = json.loads((ds_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["video_path"] is None
    assert info["unit_mode"] == "so101_norm"
    assert not (ds_root / "videos").exists()

    pq0, nrows, kv0 = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-000.parquet")
    assert nrows == 2
    assert pq0["observation.state"] == [[1.0], [1.5]]
    assert pq0["action"] == [[1.5], [2.0]]
    hf_feats = json.loads(kv0["huggingface"])["info"]["features"]
    assert hf_feats["observation.state"]["_type"] == "List"
    assert hf_feats["observation.state"]["length"] == 1
    assert hf_feats["action"]["_type"] == "List"
    assert hf_feats["action"]["length"] == 1
    assert hf_feats["timestamp"]["_type"] == "Value"


# --------------------------------------------------------------------------- Camera identity, unit contract, buffer copy


def test_collector_collect_episode_checks_camera_identity_and_unit_mode(tmp_path: Path):
    with SimEnv(
        scene="pcb_suction",
        tool="suction",
        render_camera="wrist",
        render_width=64,
        render_height=48,
    ) as env:
        # Mismatched required camera role vs env.render_camera must fail
        with LeRobotSimCollector(
            tmp_path / "bad_cam_role",
            fps=50,
            profile_id="rebot-b601dm",
            unit_mode="rad",
            joint_names=env.joint_names,
            required_camera_roles=("scene",),
        ) as c_cam:
            with pytest.raises(CollectorError, match="does not match"):
                c_cam.collect_episode(env, lambda o, i: np.zeros(7), max_steps=2)

        # Non-rad unit_mode without explicit converter must fail
        with LeRobotSimCollector(
            tmp_path / "bad_unit",
            fps=50,
            profile_id="rebot-b601dm",
            unit_mode="deg",
            joint_names=env.joint_names,
            required_camera_roles=("wrist",),
            env=env,
        ) as c_deg:
            with pytest.raises(CollectorError, match="unit_converter"):
                c_deg.collect_episode(env, lambda o, i: np.zeros(7), max_steps=2)

            # With explicit unit_converter it succeeds
            idx = c_deg.collect_episode(
                env,
                lambda o, i: np.ones(7) * 0.1,
                max_steps=2,
                unit_converter=lambda s, a: (np.degrees(s), np.degrees(a)),
            )
            assert idx == 0

        pq0, _n, _kv = _read_parquet_columns(tmp_path / "bad_unit" / "data" / "chunk-000" / "file-000.parquet")
        assert np.allclose(pq0["action"][0], np.degrees(np.ones(7) * 0.1), atol=1e-4)


def test_collector_deep_copies_reused_camera_buffer(tmp_path: Path):
    ds_root = tmp_path / "buffer_copy"
    buf = np.zeros((16, 24, 3), dtype=np.uint8)
    with LeRobotSimCollector(
        ds_root,
        fps=20,
        profile_id="rebot-test",
        unit_mode="rad",
        joint_names=["j1", "j2"],
        required_camera_roles=("wrist",),
        camera_shapes={"wrist": (16, 24, 3)},
    ) as collector:
        collector.start_episode("color")
        buf[:, :, 0], buf[:, :, 1], buf[:, :, 2] = 0, 100, 200
        collector.record_step(0.0, [0.0, 0.0], [0.0, 0.0], cameras={"wrist": buf})
        # Mutate the exact same buffer in place before step 1
        buf[:, :, 0], buf[:, :, 1], buf[:, :, 2] = 200, 0, 100
        collector.record_step(0.05, [0.1, 0.1], [0.1, 0.1], cameras={"wrist": buf})
        assert collector.save_episode() == 0

    stats = json.loads((ds_root / "meta" / "stats.json").read_text(encoding="utf-8"))
    expected_mean = np.array([[[100.0 / 255.0]], [[50.0 / 255.0]], [[150.0 / 255.0]]])
    assert np.allclose(stats["observation.images.wrist"]["mean"], expected_mean, atol=1e-6)


# --------------------------------------------------------------------------- Float32 finiteness/monotonicity, probe fallback, atomic rollback


def test_collector_float32_overflow_quantization_and_fps_dt_gates(tmp_path: Path):
    # 1. Vector overflow in float32 (1e39 finite in float64 -> inf in float32)
    with LeRobotSimCollector(
        tmp_path / "f32_vec",
        fps=20,
        profile_id="p",
        unit_mode="rad",
        joint_names=["j1", "j2"],
    ) as c:
        c.start_episode("t")
        with pytest.raises(CollectorError, match="float32"):
            c.record_step(0.0, [1e39, 0.0], [0.0, 0.0])
        with pytest.raises(CollectorError, match="invalidated"):
            c.save_episode()
        c.discard_episode()

    # 2. Timestamp overflow in float32
    with LeRobotSimCollector(
        tmp_path / "f32_ts",
        fps=20,
        profile_id="p",
        unit_mode="rad",
        joint_names=["j1", "j2"],
    ) as c:
        c.start_episode("t")
        with pytest.raises(CollectorError, match="float32"):
            c.record_step(1e39, [0.0, 0.0], [0.0, 0.0])
        c.discard_episode()

    # 3. Timestamp loses strict monotonicity after float32 quantization (100.0 + 1e-8 == 100.0 in float32)
    with LeRobotSimCollector(
        tmp_path / "f32_mono",
        fps=100_000_000,
        profile_id="p",
        unit_mode="rad",
        joint_names=["j1", "j2"],
    ) as c:
        c.start_episode("t")
        c.record_step(100.0, [0.0, 0.0], [0.0, 0.0])
        with pytest.raises(CollectorError, match="float32"):
            c.record_step(100.0 + 1e-8, [0.0, 0.0], [0.0, 0.0])
        c.discard_episode()

    # 4. Timestamp dt deviates from 1/fps
    with LeRobotSimCollector(
        tmp_path / "fps_dt",
        fps=50,
        profile_id="p",
        unit_mode="rad",
        joint_names=["j1", "j2"],
    ) as c:
        c.start_episode("t")
        c.record_step(0.0, [0.0, 0.0], [0.0, 0.0])
        with pytest.raises(CollectorError, match="deviates from expected 1/fps"):
            c.record_step(0.10, [0.0, 0.0], [0.0, 0.0])
        c.discard_episode()


def test_probe_video_decodes_frames_when_container_frames_is_zero(tmp_path: Path):
    ds_root = tmp_path / "probe_fallback"
    img = np.full((16, 24, 3), 64, dtype=np.uint8)
    with LeRobotSimCollector(
        ds_root,
        fps=20,
        profile_id="p",
        unit_mode="rad",
        joint_names=["j1"],
        required_camera_roles=("wrist",),
        camera_shapes={"wrist": (16, 24, 3)},
    ) as c:
        c.start_episode("t")
        c.record_step(0.0, [0.0], [0.0], cameras={"wrist": img})
        c.record_step(0.05, [0.1], [0.1], cameras={"wrist": img})
        assert c.save_episode() == 0

    vid = ds_root / "videos" / "observation.images.wrist" / "chunk-000" / "file-000.mp4"
    orig_open = av.open

    class _ZeroFramesStream:
        def __init__(self, inner):
            self._inner = inner
            self.frames = 0

        def __getattr__(self, name):
            return getattr(self._inner, name)

    class _WrappedContainer:
        def __init__(self, inner):
            self._inner = inner
            self.streams = type(
                "S",
                (),
                {"video": [_ZeroFramesStream(inner.streams.video[0])], "audio": inner.streams.audio},
            )()

        def seek(self, *a, **kw):
            return self._inner.seek(*a, **kw)

        def decode(self, *a, **kw):
            return self._inner.decode(*a, **kw)

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return self._inner.__exit__(*a)

    with patch("av.open", side_effect=lambda *a, **kw: _WrappedContainer(orig_open(*a, **kw))):
        probed = probe_video(vid)
    assert probed["frames"] == 2


def test_collector_fail_closed_and_save_fault_injection_zero_pollution(tmp_path: Path):
    ds_root = tmp_path / "guarded_ds"
    joints = ["j1", "j2", "j3", "j4"]
    img_w = np.full((32, 48, 3), 128, dtype=np.uint8)
    img_s = np.full((32, 48, 3), 64, dtype=np.uint8)

    with LeRobotSimCollector(
        ds_root,
        fps=20,
        profile_id="rebot-test",
        unit_mode="rad",
        joint_names=joints,
        required_camera_roles=("wrist", "scene"),
        camera_shapes={"wrist": (32, 48, 3), "scene": (32, 48, 3)},
    ) as collector:
        # Commit episode 0 cleanly
        collector.start_episode(task="clean_ep")
        collector.record_step(
            0.0, [0.0, 0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4], cameras={"wrist": img_w, "scene": img_s}
        )
        collector.record_step(
            0.05, [0.1, 0.2, 0.3, 0.4], [0.2, 0.3, 0.4, 0.5], cameras={"wrist": img_w, "scene": img_s}
        )
        assert collector.save_episode() == 0
        baseline_snap = _snapshot_tree(ds_root)

        # Helper to assert poisoned episode cannot save and leaves baseline intact
        def _assert_poisoned_and_unchanged():
            with pytest.raises(CollectorError, match="invalidated"):
                collector.save_episode()
            collector.discard_episode()
            assert _snapshot_tree(ds_root) == baseline_snap

        # Step-level rejections
        collector.start_episode(task="bad_cam_missing")
        collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        with pytest.raises(CollectorError, match="camera"):
            collector.record_step(0.05, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w})
        _assert_poisoned_and_unchanged()

        collector.start_episode(task="bad_cam_shape")
        with pytest.raises(CollectorError, match="shape mismatch"):
            collector.record_step(
                0.0,
                [0.0] * 4,
                [0.0] * 4,
                cameras={"wrist": np.zeros((16, 48, 3), dtype=np.uint8), "scene": img_s},
            )
        _assert_poisoned_and_unchanged()

        collector.start_episode(task="bad_cam_nan")
        nan_img = np.zeros((32, 48, 3), dtype=np.float32)
        nan_img[0, 0, 0] = np.nan
        with pytest.raises(CollectorError, match="non-finite"):
            collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": nan_img, "scene": img_s})
        _assert_poisoned_and_unchanged()

        # Initial frame timestamp != 0.0 must be rejected when recording cameras (PTS=0 alignment)
        collector.start_episode(task="bad_ts0")
        with pytest.raises(CollectorError, match="Initial frame timestamp must be 0.0"):
            collector.record_step(1.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        _assert_poisoned_and_unchanged()

        for bad_ts in (0.0, -0.02, np.nan):
            collector.start_episode(task="bad_ts")
            collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
            with pytest.raises(CollectorError, match="(Non-monotonic|finite|non-negative)"):
                collector.record_step(bad_ts, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
            _assert_poisoned_and_unchanged()

        collector.start_episode(task="bad_dim")
        with pytest.raises(CollectorError, match="length mismatch"):
            collector.record_step(0.0, [0.0] * 4, [0.0] * 3, cameras={"wrist": img_w, "scene": img_s})
        _assert_poisoned_and_unchanged()

        collector.start_episode(task="bad_state_nan")
        with pytest.raises(CollectorError, match="non-finite"):
            collector.record_step(0.0, [0.0, np.inf, 0.0, 0.0], [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        _assert_poisoned_and_unchanged()

        # BLOCKER #2 fault-injection tests during save_episode():
        # (a) Fault when staging the 2nd video file
        collector.start_episode(task="fail_second_video")
        collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        import shutil

        orig_copyfile = shutil.copyfile
        call_cnt = 0

        def _fail_second_copy(src, dst, *a, **kw):
            nonlocal call_cnt
            call_cnt += 1
            if call_cnt == 2:
                raise OSError("simulated disk error on 2nd video")
            return orig_copyfile(src, dst, *a, **kw)

        with patch("rebot_sim.collector.shutil.copyfile", side_effect=_fail_second_copy):
            with pytest.raises(OSError, match="simulated disk error on 2nd video"):
                collector.save_episode()
        _assert_poisoned_and_unchanged()

        # (b) Fault during tasks.parquet staging write
        collector.start_episode(task="fail_tasks_pq")
        collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        with patch("rebot_sim.collector._write_tasks_parquet", side_effect=OSError("simulated tasks write failure")):
            with pytest.raises(OSError, match="simulated tasks write failure"):
                collector.save_episode()
        _assert_poisoned_and_unchanged()

        # (c) Fault mid-atomic os.replace after info.json/data.parquet were replaced
        collector.start_episode(task="fail_mid_replace")
        collector.record_step(0.0, [0.0] * 4, [0.0] * 4, cameras={"wrist": img_w, "scene": img_s})
        orig_replace = os.replace
        rep_cnt = 0

        def _fail_fourth_replace(src, dst):
            nonlocal rep_cnt
            rep_cnt += 1
            if rep_cnt == 4:
                raise OSError("simulated replace failure mid-commit")
            return orig_replace(src, dst)

        with patch("rebot_sim.collector.os.replace", side_effect=_fail_fourth_replace):
            with pytest.raises(OSError, match="simulated replace failure mid-commit"):
                collector.save_episode()
        _assert_poisoned_and_unchanged()

        # After all failed saves, next valid episode MUST receive continuous episode_index == 1
        collector.start_episode(task="recovered_ep")
        collector.record_step(0.0, [0.2] * 4, [0.3] * 4, cameras={"wrist": img_w, "scene": img_s})
        assert collector.save_episode() == 1

    info = json.loads((ds_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 3
    eps_pq, eps_rows, _ = _read_parquet_columns(ds_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
    assert eps_rows == 2
    assert eps_pq["episode_index"] == [0, 1]
    assert (ds_root / "data" / "chunk-000" / "file-001.parquet").is_file()


def test_collector_dual_backend_schema_parity(tmp_path: Path):
    """Verify PyArrow and pure-Python backends emit consistent 1-DOF list features and 3D cam stats."""
    import sys
    import types

    from rebot_sim.collector import _has_pyarrow, _write_data_parquet, _write_episodes_parquet

    columns = {
        "observation.state": np.array([[0.5], [0.6]], dtype=np.float32),
        "action": np.array([[0.7], [0.8]], dtype=np.float32),
        "timestamp": np.array([0.0, 0.04], dtype=np.float32),
        "frame_index": np.array([0, 1], dtype=np.int64),
    }
    hf_meta = {
        "info": {
            "features": {
                "observation.state": {"feature": {"dtype": "float32", "_type": "Value"}, "length": 1, "_type": "List"},
                "action": {"feature": {"dtype": "float32", "_type": "Value"}, "length": 1, "_type": "List"},
                "timestamp": {"dtype": "float32", "_type": "Value"},
                "frame_index": {"dtype": "int64", "_type": "Value"},
            }
        }
    }
    ep_rows = [
        {
            "episode_index": 0,
            "tasks": ["t1"],
            "length": 2,
            "stats/observation.images.wrist/mean": [[[0.2]], [[0.4]], [[0.6]]],
        }
    ]

    # Pure-Python path
    pure_data = tmp_path / "pure_data.parquet"
    pure_ep = tmp_path / "pure_ep.parquet"
    with patch("rebot_sim.collector._has_pyarrow", return_value=False):
        _write_data_parquet(pure_data, columns, hf_meta)
        _write_episodes_parquet(pure_ep, ep_rows)

    pq_d, nrows_d, kv_d = _read_parquet_columns(pure_data)
    assert nrows_d == 2
    assert pq_d["observation.state"] == [[pytest.approx(0.5)], [pytest.approx(0.6)]]
    assert json.loads(kv_d["huggingface"])["info"]["features"]["observation.state"]["_type"] == "List"
    pq_e, nrows_e, _ = _read_parquet_columns(pure_ep)
    assert nrows_e == 1
    assert np.asarray(pq_e["stats/observation.images.wrist/mean"]).shape == (1, 3, 1, 1)

    # PyArrow path (real if installed, else stubbed to verify exact API calls & shapes)
    if _has_pyarrow():
        import pyarrow.parquet as pq

        pa_data = tmp_path / "pa_data.parquet"
        pa_ep = tmp_path / "pa_ep.parquet"
        _write_data_parquet(pa_data, columns, hf_meta)
        _write_episodes_parquet(pa_ep, ep_rows)
        td = pq.read_table(pa_data)
        te = pq.read_table(pa_ep)
        assert td["observation.state"].to_pylist() == [[pytest.approx(0.5)], [pytest.approx(0.6)]]
        assert np.asarray(te["stats/observation.images.wrist/mean"].to_pylist()).shape == (1, 3, 1, 1)
    else:
        recorded: dict[str, object] = {}

        class _FakeArray:
            def __init__(self, kind, size, inner=None):
                self.type = (kind, size, inner)

        class _FakePa(types.ModuleType):
            __version__ = "stub"

            class FixedSizeListArray:
                @staticmethod
                def from_arrays(arr, size):
                    return _FakeArray("fixed_size_list", size, arr.type)

            class Table:
                @staticmethod
                def from_arrays(arrays, schema=None):
                    return ("table", arrays, schema)

                @staticmethod
                def from_pylist(rows):
                    return ("pylist_table", rows)

            @staticmethod
            def float32():
                return "float32"

            @staticmethod
            def array(vals, type=None):
                return _FakeArray("scalar", len(vals), str(type or getattr(vals, "dtype", "")))

            @staticmethod
            def field(name, typ):
                return (name, typ)

            @staticmethod
            def schema(fields, metadata=None):
                return ("schema", fields, metadata)

        class _FakePq(types.ModuleType):
            @staticmethod
            def write_table(tbl, path):
                recorded[Path(path).name] = tbl
                Path(path).write_bytes(b"PAR1stubPAR1")

        fake_pa = _FakePa("pyarrow")
        fake_pq = _FakePq("pyarrow.parquet")
        fake_pa.parquet = fake_pq
        with patch.dict(sys.modules, {"pyarrow": fake_pa, "pyarrow.parquet": fake_pq}):
            pa_data = tmp_path / "pa_data.parquet"
            pa_ep = tmp_path / "pa_ep.parquet"
            _write_data_parquet(pa_data, columns, hf_meta)
            _write_episodes_parquet(pa_ep, ep_rows)

        tbl_d = next(v for k, v in recorded.items() if k.startswith("pa_data"))
        # observation.state and action are FixedSizeListArray(size=1)
        assert tbl_d[1][0].type[0] == "fixed_size_list" and tbl_d[1][0].type[1] == 1
        assert tbl_d[1][1].type[0] == "fixed_size_list" and tbl_d[1][1].type[1] == 1
        tbl_e = next(v for k, v in recorded.items() if k.startswith("pa_ep"))
        assert np.asarray(tbl_e[1][0]["stats/observation.images.wrist/mean"]).shape == (3, 1, 1)
