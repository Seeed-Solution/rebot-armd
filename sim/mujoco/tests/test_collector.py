"""Tests for SimEnv strict action validation and LeRobotSimCollector (LeRobot v3.0)."""

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import av
import numpy as np
import pytest

from rebot_sim.collector import CollectorError, LeRobotSimCollector
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


def _read_parquet_columns(path: Path) -> tuple[dict[str, list], int, dict[str, str]]:
    """Read an uncompressed PLAIN Parquet file (scalar + list columns) without pyarrow."""
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

    # Map column leaf paths to list/scalar structure from schema_elems
    col_is_list: dict[tuple[str, ...], bool] = {}
    idx = 1
    while idx < len(schema_elems):
        se = schema_elems[idx]
        name = se[4][1].decode("utf-8")
        if 5 in se and se[5][1] > 0:
            # Group (LIST -> list -> element)
            col_is_list[(name, "list", "element")] = True
            idx += 3
        else:
            col_is_list[(name,)] = False
            idx += 1

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
        is_list = col_is_list[path_parts]
        rep_bits: list[int] = []
        if is_list:
            rep_len = struct.unpack_from("<I", payload, pos)[0]
            pos += 4
            # Decode RLE/bit-pack (bit_width=1)
            rep_end = pos + rep_len
            while pos < rep_end and len(rep_bits) < num_values:
                hdr = 0
                shift = 0
                while True:
                    b = payload[pos]
                    pos += 1
                    hdr |= (b & 0x7F) << shift
                    if not (b & 0x80):
                        break
                    shift += 7
                if hdr & 1:
                    n_groups = hdr >> 1
                    for _ in range(n_groups):
                        byte = payload[pos]
                        pos += 1
                        for bit in range(8):
                            if len(rep_bits) < num_values:
                                rep_bits.append((byte >> bit) & 1)
                else:
                    cnt = hdr >> 1
                    val = payload[pos]
                    pos += 1
                    rep_bits.extend([val] * min(cnt, num_values - len(rep_bits)))

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
        if is_list:
            rows: list[list] = []
            for rbit, v in zip(rep_bits, flat):
                if rbit == 0:
                    rows.append([v])
                else:
                    rows[-1].append(v)
            out[col_name] = rows
        else:
            out[col_name] = flat

    return out, num_rows, kv_meta


def _snapshot_tree(root: Path) -> dict[str, str]:
    snap: dict[str, str] = {}
    for p in sorted(root.rglob("*")):
        if p.is_file():
            snap[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return snap


# --------------------------------------------------------------------------- SimEnv.step() validation


def test_sim_env_step_rejects_short_long_nonfinite_and_bad_shapes():
    with SimEnv(scene="pcb_suction", tool="suction", control_mode="position") as env:
        obs, info = env.reset(seed=42)
        assert env.action_dim == 7
        assert env.n_joints == 7
        assert len(env.joint_names) == 7

        # Establish a known non-zero motor target first
        init_target = np.array([0.15, -0.25, 0.1, 0.0, 0.0, 0.0, 0.0])
        env.step(init_target)
        saved_qt = env.motors.q_t.copy()
        saved_time = float(env.data.time)
        saved_io = env.pack_scene.io_state(env.data)

        # Short action must fail instead of silently copying prefix and keeping old targets
        with pytest.raises(ValueError, match="length mismatch"):
            env.step([0.9, 0.9], io_updates={"vacuum": True})
        assert np.allclose(env.motors.q_t, saved_qt)
        assert float(env.data.time) == saved_time
        assert env.pack_scene.io_state(env.data) == saved_io

        # Empty action
        with pytest.raises(ValueError, match="length mismatch"):
            env.step([])

        # Over-long action must fail instead of truncating
        with pytest.raises(ValueError, match="length mismatch"):
            env.step(np.zeros(8))

        # Non-finite values (NaN, +Inf, -Inf)
        for bad in (np.nan, np.inf, -np.inf):
            vec = np.zeros(7)
            vec[3] = bad
            with pytest.raises(ValueError, match="non-finite"):
                env.step(vec)

        # Invalid shapes and types
        with pytest.raises(ValueError, match="1D action"):
            env.step(0.0)
        with pytest.raises(ValueError, match="1D action"):
            env.step(np.zeros((2, 7)))
        with pytest.raises(ValueError, match="1D action"):
            env.step(np.zeros((7, 1)))
        with pytest.raises(ValueError, match="boolean"):
            env.step([True] + [0.0] * 6)

        # (1, dof) batch shape is squeezed and accepted
        obs2, _r, _d, info2 = env.step(np.zeros((1, 7)))
        assert obs2["state"].shape == (7,)
        assert info2["sim_time_s"] > saved_time
        assert np.allclose(env.motors.q_t, np.zeros(7))


def test_sim_env_step_velocity_mode_and_so101_dof():
    with SimEnv(scene="pcb_suction", tool="suction", control_mode="velocity") as env:
        env.reset(seed=7)
        assert env.action_dim == env.model.nu == 7
        with pytest.raises(ValueError, match="length mismatch"):
            env.step([0.1, 0.2, 0.3])
        with pytest.raises(ValueError, match="length mismatch"):
            env.step(np.zeros(9))
        with pytest.raises(ValueError, match="non-finite"):
            env.step([0.0, np.nan, 0.0, 0.0, 0.0, 0.0, 0.0])
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


# --------------------------------------------------------------------------- LeRobotSimCollector validation


def test_collector_requires_profile_id_and_unit_mode(tmp_path: Path):
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


# --------------------------------------------------------------------------- Multi-episode collection + verification


def test_collector_multi_episode_writes_valid_lerobot_v3_dataset(tmp_path: Path):
    ds_root = tmp_path / "pcb_dataset"
    ep0_states: list[np.ndarray] = []
    ep0_actions: list[np.ndarray] = []
    ep0_ts: list[float] = []
    ep1_states: list[np.ndarray] = []
    ep1_actions: list[np.ndarray] = []
    ep1_ts: list[float] = []

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
            # Episode 0: 5 steps
            def policy0(obs: dict, step: int) -> np.ndarray:
                act = np.array([0.05 * step, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                ep0_states.append(obs["state"].copy())
                ep0_actions.append(act.copy())
                return act

            idx0 = collector.collect_episode(env, policy0, max_steps=5, task="pick_pcb", seed=10)
            assert idx0 == 0
            ep0_ts.extend([0.02 * (k + 1) for k in range(5)])

            # Episode 1: 4 steps with different task
            def policy1(obs: dict, step: int) -> np.ndarray:
                act = np.array([-0.05 * step, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                ep1_states.append(obs["state"].copy())
                ep1_actions.append(act.copy())
                return act

            idx1 = collector.collect_episode(env, policy1, max_steps=4, task="place_pcb", seed=20)
            assert idx1 == 1
            ep1_ts.extend([0.02 * (k + 1) for k in range(4)])

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
    assert info["features"]["observation.state"]["shape"] == [7]
    assert info["features"]["action"]["names"] == expected_joints
    assert info["features"]["action"]["shape"] == [7]
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
    assert np.allclose(stats["action"]["min"], all_actions.min(axis=0), atol=1e-6)
    assert np.allclose(stats["action"]["max"], all_actions.max(axis=0), atol=1e-6)
    assert np.asarray(stats["observation.images.wrist"]["mean"]).shape == (3, 1, 1)
    assert stats["observation.images.wrist"]["count"] == [9]

    # Verify Parquet files (row counts, exact values, and metadata)
    pq0, nrows0, kv0 = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-000.parquet")
    assert nrows0 == 5
    assert "huggingface" in kv0
    assert pq0["frame_index"] == [0, 1, 2, 3, 4]
    assert pq0["episode_index"] == [0] * 5
    assert pq0["index"] == [0, 1, 2, 3, 4]
    assert pq0["task_index"] == [0] * 5
    assert np.allclose(pq0["timestamp"], ep0_ts, atol=1e-6)
    assert np.allclose(pq0["observation.state"], np.stack(ep0_states), atol=1e-6)
    assert np.allclose(pq0["action"], np.stack(ep0_actions), atol=1e-6)

    pq1, nrows1, _kv1 = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-001.parquet")
    assert nrows1 == 4
    assert pq1["frame_index"] == [0, 1, 2, 3]
    assert pq1["episode_index"] == [1] * 4
    assert pq1["index"] == [5, 6, 7, 8]
    assert pq1["task_index"] == [1] * 4
    assert np.allclose(pq1["timestamp"], ep1_ts, atol=1e-6)
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

    # Verify video frame counts match parquet row counts
    for ep_i, expected_frames in ((0, 5), (1, 4)):
        vid = ds_root / "videos" / "observation.images.wrist" / "chunk-000" / f"file-{ep_i:03d}.mp4"
        assert vid.is_file()
        with av.open(str(vid)) as container:
            decoded = sum(1 for _ in container.decode(video=0))
        assert decoded == expected_frames


def test_collector_state_only_without_cameras(tmp_path: Path):
    ds_root = tmp_path / "state_only"
    joints = ["j1", "j2", "j3"]
    with LeRobotSimCollector(
        ds_root,
        fps=25,
        profile_id="test-profile",
        unit_mode="so101_norm",
        joint_names=joints,
        required_camera_roles=(),
    ) as collector:
        collector.start_episode(task="reach")
        collector.record_step(0.04, [1.0, 2.0, 3.0], [1.5, 2.5, 3.5])
        collector.record_step(0.08, [1.5, 2.5, 3.5], [2.0, 3.0, 4.0])
        assert collector.save_episode() == 0

    info = json.loads((ds_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["video_path"] is None
    assert info["unit_mode"] == "so101_norm"
    assert not (ds_root / "videos").exists()
    pq0, nrows, _ = _read_parquet_columns(ds_root / "data" / "chunk-000" / "file-000.parquet")
    assert nrows == 2
    assert np.allclose(pq0["observation.state"], [[1.0, 2.0, 3.0], [1.5, 2.5, 3.5]], atol=1e-6)


# --------------------------------------------------------------------------- Fail-closed & zero-pollution tests


def test_collector_fail_closed_does_not_pollute_committed_dataset(tmp_path: Path):
    ds_root = tmp_path / "guarded_ds"
    joints = ["j1", "j2", "j3", "j4"]
    valid_img = np.full((32, 48, 3), 128, dtype=np.uint8)

    with LeRobotSimCollector(
        ds_root,
        fps=20,
        profile_id="rebot-test",
        unit_mode="rad",
        joint_names=joints,
        required_camera_roles=("wrist",),
        camera_shapes={"wrist": (32, 48, 3)},
    ) as collector:
        # Save one clean baseline episode
        collector.start_episode(task="clean_ep")
        collector.record_step(0.05, [0.0, 0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4], cameras={"wrist": valid_img})
        collector.record_step(0.10, [0.1, 0.2, 0.3, 0.4], [0.2, 0.3, 0.4, 0.5], cameras={"wrist": valid_img})
        assert collector.save_episode() == 0

        baseline_snap = _snapshot_tree(ds_root)

        # 1. Missing camera frame -> fail-closed
        collector.start_episode(task="bad_cam_missing")
        collector.record_step(0.05, [0.0] * 4, [0.0] * 4, cameras={"wrist": valid_img})
        with pytest.raises(CollectorError, match="camera"):
            collector.record_step(0.10, [0.0] * 4, [0.0] * 4, cameras=None)
        with pytest.raises(CollectorError, match="invalidated"):
            collector.save_episode()
        collector.discard_episode()
        assert _snapshot_tree(ds_root) == baseline_snap

        # 2. Wrong camera shape -> fail-closed
        collector.start_episode(task="bad_cam_shape")
        with pytest.raises(CollectorError, match="shape mismatch"):
            collector.record_step(
                0.05, [0.0] * 4, [0.0] * 4, cameras={"wrist": np.zeros((16, 48, 3), dtype=np.uint8)}
            )
        collector.discard_episode()
        assert _snapshot_tree(ds_root) == baseline_snap

        # 3. Non-finite camera image -> fail-closed
        collector.start_episode(task="bad_cam_nan")
        nan_img = np.zeros((32, 48, 3), dtype=np.float32)
        nan_img[0, 0, 0] = np.nan
        with pytest.raises(CollectorError, match="non-finite"):
            collector.record_step(0.05, [0.0] * 4, [0.0] * 4, cameras={"wrist": nan_img})
        collector.discard_episode()
        assert _snapshot_tree(ds_root) == baseline_snap

        # 4. Non-monotonic timestamps (duplicate and decreasing) -> fail-closed
        for bad_ts in (0.05, 0.02, np.nan):
            collector.start_episode(task="bad_ts")
            collector.record_step(0.05, [0.0] * 4, [0.0] * 4, cameras={"wrist": valid_img})
            with pytest.raises(CollectorError, match="(Non-monotonic|finite)"):
                collector.record_step(bad_ts, [0.0] * 4, [0.0] * 4, cameras={"wrist": valid_img})
            collector.discard_episode()
            assert _snapshot_tree(ds_root) == baseline_snap

        # 5. Dimension mismatch and non-finite action/state -> fail-closed
        collector.start_episode(task="bad_dim")
        with pytest.raises(CollectorError, match="length mismatch"):
            collector.record_step(0.05, [0.0] * 4, [0.0] * 3, cameras={"wrist": valid_img})
        collector.discard_episode()
        assert _snapshot_tree(ds_root) == baseline_snap

        collector.start_episode(task="bad_state_nan")
        with pytest.raises(CollectorError, match="non-finite"):
            collector.record_step(0.05, [0.0, np.inf, 0.0, 0.0], [0.0] * 4, cameras={"wrist": valid_img})
        collector.discard_episode()
        assert _snapshot_tree(ds_root) == baseline_snap

    # Final check: committed dataset still has exactly 1 episode, 2 frames
    info = json.loads((ds_root / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 2
