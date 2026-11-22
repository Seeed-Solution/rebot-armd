"""Tests for Boundary 3 pack security/budget validation, robot identity, and two-phase tx_id activation."""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest
from rebot_sim.camera import CameraMount
from rebot_sim.pack import (
    MAX_MESH_FILES,
    MAX_PACK_BODIES,
    MAX_PACK_GEOMS,
    MAX_PACK_TOTAL_BYTES,
    MAX_SINGLE_MESH_BYTES,
    MAX_TOTAL_VERTICES,
    MAX_XML_BYTES,
    MAX_XML_ELEMENTS,
    PackValidationError,
    compose_spec,
    compute_pack_digest,
    load_scene_pack,
    load_tool_pack,
    validate_pack_security,
)
from rebot_sim.perception import Perception, PerceptionServer
from rebot_sim.server import Simulator


def _free_port(kind: int = socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _http_json(
    url: str,
    method: str = "GET",
    body: dict | None = None,
) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    hdr = {"Content-Type": "application/json"} if data is not None else {}
    req = urllib.request.Request(url, data=data, headers=hdr, method=method)
    try:
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def _make_binary_stl(tris: list[tuple[tuple[float, float, float], ...]]) -> bytes:
    header = b"rebot-sim test binary stl".ljust(80, b"\x00")
    buf = bytearray(header + struct.pack("<I", len(tris)))
    for v1, v2, v3 in tris:
        buf.extend(struct.pack("<12fH", 0.0, 0.0, 1.0, *v1, *v2, *v3, 0))
    return bytes(buf)


def _write_minimal_scene(
    pack_dir: Path,
    *,
    pack_id: str = "custom_scene",
    world_xml: str | None = None,
    extra_manifest: dict | None = None,
) -> Path:
    pack_dir.mkdir(parents=True, exist_ok=True)
    xml = world_xml or (
        '<mujoco model="custom_world">\n'
        '  <worldbody>\n'
        '    <body name="part" pos="0.25 0.0 0.02">\n'
        '      <freejoint name="part_joint"/>\n'
        '      <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>\n'
        '      <geom name="part_geom" type="box" size="0.02 0.02 0.02" rgba="0.2 0.7 0.3 1"/>\n'
        '    </body>\n'
        '  </worldbody>\n'
        '</mujoco>\n'
    )
    (pack_dir / "world.xml").write_text(xml, encoding="utf-8")
    manifest = {
        "id": pack_id,
        "name": "Custom Scene",
        "world_xml": "world.xml",
        "objects": [
            {
                "name": "part",
                "body": "part",
                "geom": "part_geom",
                "class_name": "part",
                "default_pos": [0.25, 0.0, 0.02],
            }
        ],
    }
    if extra_manifest:
        manifest.update(extra_manifest)
    (pack_dir / "scene.json").write_text(json.dumps(manifest), encoding="utf-8")
    return pack_dir


# ---------------------------------------------------------------------------
# 1. Pack / MJCF / STL / OBJ security & budget validation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad_id", ["", "-bad", "_bad", "bad/id", "bad.id", "a" * 65])
def test_pack_id_regex_rejection(tmp_path: Path, bad_id: str):
    pdir = _write_minimal_scene(tmp_path / "pack", pack_id=bad_id)
    with pytest.raises(PackValidationError) as exc:
        validate_pack_security(pdir, "scene")
    assert exc.value.code == "INVALID_PACK"


def test_urdf_file_and_reference_rejected(tmp_path: Path):
    pdir = _write_minimal_scene(tmp_path / "urdf_pack")
    (pdir / "arm.urdf").write_text("<robot name='x'/>", encoding="utf-8")
    with pytest.raises(PackValidationError) as exc:
        validate_pack_security(pdir, "scene")
    assert exc.value.code == "UNSUPPORTED_FORMAT"

    pdir2 = _write_minimal_scene(
        tmp_path / "urdf_ref_pack",
        world_xml='<mujoco><asset><mesh name="m" file="arm.urdf"/></asset><worldbody/></mujoco>',
    )
    with pytest.raises(PackValidationError) as exc2:
        validate_pack_security(pdir2, "scene")
    assert exc2.value.code == "UNSUPPORTED_FORMAT"


@pytest.mark.parametrize(
    "bad_xml",
    [
        '<!DOCTYPE foo [ <!ENTITY xxe "evil"> ]><mujoco><worldbody/></mujoco>',
        '<!ENTITY foo "bar"><mujoco><worldbody/></mujoco>',
        '<mujoco><include file="other.xml"/><worldbody/></mujoco>',
        '<mujoco><extension><plugin plugin="mujoco.sensor"/></extension><worldbody/></mujoco>',
        '<mujoco><plugin name="p"/><worldbody/></mujoco>',
        '<mujoco><custom><numeric name="x"/></custom><worldbody/></mujoco>',
        '<mujoco><asset><mesh name="m" file="../escape.stl"/></asset><worldbody/></mujoco>',
        '<mujoco><asset><mesh name="m" file="/etc/passwd"/></asset><worldbody/></mujoco>',
        '<mujoco><compiler meshdir="../outside"/><worldbody/></mujoco>',
        '<mujoco><compiler assetdir="/tmp"/><worldbody/></mujoco>',
    ],
)
def test_mjcf_forbidden_constructs_and_path_escapes(tmp_path: Path, bad_xml: str):
    pdir = _write_minimal_scene(tmp_path / "bad_xml", world_xml=bad_xml)
    with pytest.raises(PackValidationError):
        validate_pack_security(pdir, "scene")


def test_mjcf_size_and_depth_budgets(tmp_path: Path):
    # > 512 KiB XML
    big_comment = "<!-- " + ("a" * (MAX_XML_BYTES + 64)) + " -->"
    pdir_big = _write_minimal_scene(
        tmp_path / "big_xml",
        world_xml=f"<mujoco>{big_comment}<worldbody/></mujoco>",
    )
    with pytest.raises(PackValidationError) as exc_big:
        validate_pack_security(pdir_big, "scene")
    assert exc_big.value.code == "PACK_TOO_LARGE"

    # Depth > 32
    nested_open = "".join(f'<body name="b{i}">' for i in range(33))
    nested_close = "</body>" * 33
    pdir_deep = _write_minimal_scene(
        tmp_path / "deep_xml",
        world_xml=f"<mujoco><worldbody>{nested_open}{nested_close}</worldbody></mujoco>",
    )
    with pytest.raises(PackValidationError) as exc_deep:
        validate_pack_security(pdir_deep, "scene")
    assert exc_deep.value.code == "INVALID_PACK"


def test_binary_stl_validation_valid_truncated_nan_and_budget(tmp_path: Path):
    # Valid binary STL
    tri = ((0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.0, 0.05, 0.0))
    pdir = _write_minimal_scene(
        tmp_path / "bin_stl_ok",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="tri.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir / "tri.stl").write_bytes(_make_binary_stl([tri]))
    digest = validate_pack_security(pdir, "scene")
    assert len(digest) == 64 and digest == compute_pack_digest(pdir)

    # Truncated binary STL (bad byte length)
    pdir_trunc = _write_minimal_scene(
        tmp_path / "bin_stl_trunc",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="tri.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir_trunc / "tri.stl").write_bytes(_make_binary_stl([tri])[:-5])
    with pytest.raises(PackValidationError) as exc_trunc:
        validate_pack_security(pdir_trunc, "scene")
    assert exc_trunc.value.code == "INVALID_PACK"

    # NaN / Inf in binary STL
    pdir_nan = _write_minimal_scene(
        tmp_path / "bin_stl_nan",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="tri.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    bad_tri = ((math.nan, 0.0, 0.0), (0.05, 0.0, 0.0), (0.0, 0.05, 0.0))
    (pdir_nan / "tri.stl").write_bytes(_make_binary_stl([bad_tri]))
    with pytest.raises(PackValidationError) as exc_nan:
        validate_pack_security(pdir_nan, "scene")
    assert exc_nan.value.code == "INVALID_PACK"

    # Vertex budget overflow (> 150_000 vertices = > 50_000 triangles)
    pdir_over = _write_minimal_scene(tmp_path / "bin_stl_over")
    over_tris = (MAX_TOTAL_VERTICES // 3) + 1
    fake_header = b"h" * 80 + struct.pack("<I", over_tris)
    (pdir_over / "huge.stl").write_bytes(fake_header + b"\x00" * (50 * over_tris))
    with pytest.raises(PackValidationError) as exc_over:
        validate_pack_security(pdir_over, "scene")
    assert exc_over.value.code == "PACK_TOO_LARGE"


def test_ascii_stl_validation(tmp_path: Path):
    pdir = _write_minimal_scene(
        tmp_path / "ascii_stl_ok",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="tri.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir / "tri.stl").write_text(
        "solid tri\n"
        "  facet normal 0 0 1\n"
        "    outer loop\n"
        "      vertex 0 0 0\n"
        "      vertex 0.05 0 0\n"
        "      vertex 0 0.05 0\n"
        "    endloop\n"
        "  endfacet\n"
        "endsolid tri\n",
        encoding="utf-8",
    )
    assert len(validate_pack_security(pdir, "scene")) == 64

    # Missing endsolid
    pdir_bad = _write_minimal_scene(
        tmp_path / "ascii_stl_bad",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="tri.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir_bad / "tri.stl").write_text(
        "solid tri\n"
        "  facet normal 0 0 1\n"
        "    outer loop\n"
        "      vertex 0 0 0\n"
        "      vertex 0.05 0 0\n"
        "      vertex 0 0.05 0\n"
        "    endloop\n"
        "  endfacet\n",
        encoding="utf-8",
    )
    with pytest.raises(PackValidationError) as exc:
        validate_pack_security(pdir_bad, "scene")
    assert exc.value.code == "INVALID_PACK"


def test_obj_validation_and_dangling_refs_and_file_limits(tmp_path: Path):
    # Valid OBJ
    pdir = _write_minimal_scene(
        tmp_path / "obj_ok",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="box.obj"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir / "box.obj").write_text(
        "# cube face\n"
        "o box\n"
        "g box_grp\n"
        "s off\n"
        "usemtl default\n"
        "v 0 0 0\n"
        "v 0.05 0 0\n"
        "v 0 0.05 0\n"
        "vn 0 0 1\n"
        "vt 0 0\n"
        "f 1/1/1 2/1/1 3/1/1\n",
        encoding="utf-8",
    )
    assert len(validate_pack_security(pdir, "scene")) == 64

    # Forbidden mtllib in OBJ
    pdir_mtl = _write_minimal_scene(
        tmp_path / "obj_mtllib",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="box.obj"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir_mtl / "box.obj").write_text("mtllib /etc/passwd\nv 0 0 0\n", encoding="utf-8")
    with pytest.raises(PackValidationError):
        validate_pack_security(pdir_mtl, "scene")

    # Dangling <mesh file="...">
    pdir_dangling = _write_minimal_scene(
        tmp_path / "dangling_ref",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="missing.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    with pytest.raises(PackValidationError):
        validate_pack_security(pdir_dangling, "scene")

    # Symlink escape inside pack_dir
    outside = tmp_path / "outside.stl"
    outside.write_bytes(_make_binary_stl([((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))]))
    pdir_sym = _write_minimal_scene(
        tmp_path / "sym_escape",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="link.stl"/></asset>'
            '<worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
        ),
    )
    (pdir_sym / "link.stl").symlink_to(outside)
    with pytest.raises(PackValidationError) as exc_sym:
        validate_pack_security(pdir_sym, "scene")
    assert exc_sym.value.code == "PATH_ESCAPE"

    # Too many mesh files (> 16)
    pdir_many = _write_minimal_scene(tmp_path / "too_many_meshes")
    small_stl = _make_binary_stl([((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0))])
    for i in range(MAX_MESH_FILES + 1):
        (pdir_many / f"m{i}.stl").write_bytes(small_stl)
    with pytest.raises(PackValidationError) as exc_many:
        validate_pack_security(pdir_many, "scene")
    assert exc_many.value.code == "PACK_TOO_LARGE"

    # Single mesh > 4 MiB
    pdir_single = _write_minimal_scene(tmp_path / "single_mesh_huge")
    (pdir_single / "big.obj").write_bytes(b"# " + b"a" * MAX_SINGLE_MESH_BYTES)
    with pytest.raises(PackValidationError) as exc_single:
        validate_pack_security(pdir_single, "scene")
    assert exc_single.value.code == "PACK_TOO_LARGE"

    # Total pack size > 10 MiB
    pdir_total = _write_minimal_scene(tmp_path / "total_pack_huge")
    (pdir_total / "blob.bin").write_bytes(b"\x00" * MAX_PACK_TOTAL_BYTES)
    with pytest.raises(PackValidationError) as exc_total:
        validate_pack_security(pdir_total, "scene")
    assert exc_total.value.code == "PACK_TOO_LARGE"

    # Digest mismatch rejected
    with pytest.raises(PackValidationError) as exc_dig:
        validate_pack_security(pdir, "scene", expected_digest="0" * 64)
    assert exc_dig.value.code == "DIGEST_MISMATCH"


# ---------------------------------------------------------------------------
# 2. Robot identity + Two-phase tx_id activation + DEGRADED_SPLIT
# ---------------------------------------------------------------------------


def test_sim_state_robot_identity_and_so101_normalization():
    sim_b = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    try:
        st = sim_b.sim_state()
        assert st["robot"] == "b601dm"
        assert st["tx_id"] is None
        assert st["camera_tx_id"] is None
        assert st["activation_state"] == "COMMITTED"
        assert st["degraded"] is False
    finally:
        sim_b.close()

    sim_so = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="so_arm101")
    try:
        st_so = sim_so.sim_state()
        assert st_so["robot"] == "so-arm101"
        assert st_so["activation_state"] == "COMMITTED"
    finally:
        sim_so.close()


def test_two_phase_prepare_commit_abort_and_robot_mismatch(tmp_path: Path):
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")
    digest_a = compute_pack_digest(scene_a)
    digest_b = compute_pack_digest(scene_b)

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        # 1. GET /sim/state reports robot identity and tx fields
        code, st = _http_json(f"{base}/sim/state")
        assert code == 200
        assert st["robot"] == "b601dm"
        assert st["tx_id"] is None
        assert st["activation_state"] == "COMMITTED"
        assert st["degraded"] is False

        # 2. Mismatched robot in prepare -> 400 ROBOT_INCOMPATIBLE before compiling
        code, err = _http_json(
            f"{base}/sim/scene/prepare",
            method="POST",
            body={
                "tx_id": "tx-bad-robot",
                "scene": "scene_a",
                "scene_dir": str(scene_a),
                "tool": "default",
                "robot": "so-arm101",
            },
        )
        assert code == 400
        assert err["code"] == "ROBOT_INCOMPATIBLE"
        assert err["expected_robot"] == "b601dm"
        assert err["state"] == "ABORTED"
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "default"

        # Also legacy /sim/scene rejects mismatched robot with 400 ROBOT_INCOMPATIBLE
        code, legacy_err = _http_json(
            f"{base}/sim/scene",
            method="POST",
            body={"scene": "coffee_kiosk", "robot": "so-arm101"},
        )
        assert code == 400
        assert legacy_err["code"] == "ROBOT_INCOMPATIBLE"
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "default"

        # 3. Commit without prepare -> 409 TX_NOT_PREPARED
        code, not_prep = _http_json(
            f"{base}/sim/scene/commit",
            method="POST",
            body={"tx_id": "tx-missing"},
        )
        assert code == 409
        assert not_prep["code"] == "TX_NOT_PREPARED"

        # 4. Prepare scene_a -> active still 'default' until commit
        code, prep = _http_json(
            f"{base}/sim/scene/prepare",
            method="POST",
            body={
                "tx_id": "tx-1",
                "scene": "scene_a",
                "scene_dir": str(scene_a),
                "scene_digest": digest_a,
                "tool": "default",
                "robot": "b601-dm",  # normalized alias matches b601dm
            },
        )
        assert code == 200
        assert prep["state"] == "PREPARED"
        assert prep["tx_id"] == "tx-1"
        assert prep["scene_digest"] == digest_a
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "default"

        # 5. Commit tx-1 -> switches physics & camera tx_id; idempotent on repeat
        code, comm = _http_json(
            f"{base}/sim/scene/commit",
            method="POST",
            body={"tx_id": "tx-1"},
        )
        assert code == 200
        assert comm["state"] == "COMMITTED"
        assert comm["tx_id"] == "tx-1"
        assert comm["camera_tx_id"] == "tx-1"
        assert comm["scene"] == "scene_a"
        assert comm["scene_digest"] == digest_a

        code, comm_idem = _http_json(
            f"{base}/sim/scene/commit",
            method="POST",
            body={"tx_id": "tx-1"},
        )
        assert code == 200 and comm_idem["tx_id"] == "tx-1"

        code, st_after = _http_json(f"{base}/sim/state")
        assert st_after["scene"] == "scene_a"
        assert st_after["tx_id"] == "tx-1"
        assert st_after["camera_tx_id"] == "tx-1"
        assert st_after["scene_digest"] == digest_a

        # 6. Prepare then abort -> active stays scene_a / tx-1
        code, prep_b = _http_json(
            f"{base}/sim/scene/prepare",
            method="POST",
            body={
                "tx_id": "tx-2",
                "scene": "scene_b",
                "scene_dir": str(scene_b),
                "scene_digest": digest_b,
                "tool": "default",
            },
        )
        assert code == 200
        code, aborted = _http_json(
            f"{base}/sim/scene/abort",
            method="POST",
            body={"tx_id": "tx-2"},
        )
        assert code == 200 and aborted["state"] == "ABORTED"
        assert _http_json(f"{base}/sim/state")[1]["tx_id"] == "tx-1"
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "scene_a"

        # 7. Failed prepare (bad digest) leaves active scene untouched
        code, bad_prep = _http_json(
            f"{base}/sim/scene/prepare",
            method="POST",
            body={
                "tx_id": "tx-bad-digest",
                "scene": "scene_b",
                "scene_dir": str(scene_b),
                "scene_digest": "f" * 64,
                "tool": "default",
            },
        )
        assert code == 400
        assert bad_prep["code"] == "DIGEST_MISMATCH"
        assert bad_prep["state"] == "ABORTED"
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "scene_a"
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_degraded_split_blocks_mixed_serving_and_recovers_via_camera_only(tmp_path: Path):
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    cam_state = {"fail_commit": False, "commits": []}
    sim.set_camera_hooks(
        prepare_cb=lambda info: {"tx_id": info["tx_id"]},
        commit_cb=lambda staged: (
            (_ for _ in ()).throw(RuntimeError("GL swap error"))
            if cam_state["fail_commit"]
            else cam_state["commits"].append(staged["tx_id"])
        ),
        abort_cb=lambda staged: None,
    )
    sim.camera_info = lambda: {"fps": 15, "tx_id": sim.camera_tx_id}  # type: ignore[attr-defined]
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        # Commit tx-1 cleanly
        assert _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-1", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"},
        )[0] == 200
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-1"})[0] == 200

        # Next commit fails camera swap -> DEGRADED_SPLIT
        assert _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-2", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"},
        )[0] == 200
        cam_state["fail_commit"] = True
        code, deg = _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-2"})
        assert code == 503
        assert deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert deg["state"] == "DEGRADED_SPLIT"
        assert deg["physics_tx_id"] == "tx-2"
        assert deg["camera_tx_id"] == "tx-1"

        # Mixed-generation endpoints MUST return 503 SIM_ACTIVATION_DEGRADED
        for method, path, body in (
            ("GET", "/sim/camera", None),
            ("POST", "/api/detect", {}),
            ("GET", "/api/detections", None),
            ("POST", "/sim/scene", {"scene": "default"}),
            ("POST", "/sim/scene/prepare", {"tx_id": "tx-3", "scene": "default", "tool": "default"}),
        ):
            c, r = _http_json(f"{base}{path}", method=method, body=body)
            assert c == 503, f"Expected 503 on {method} {path}, got {c}: {r}"
            assert r["code"] == "SIM_ACTIVATION_DEGRADED"

        # GET /sim/state MUST still return 200 for reconciliation
        c, st = _http_json(f"{base}/sim/state")
        assert c == 200
        assert st["activation_state"] == "DEGRADED_SPLIT"
        assert st["degraded"] is True
        assert st["tx_id"] == "tx-2"
        assert st["camera_tx_id"] == "tx-1"
        assert st["robot"] == "b601dm"

        # Recovery path 1: camera_only succeeds (and is idempotent)
        cam_state["fail_commit"] = False
        c, rec = _http_json(
            f"{base}/sim/scene/commit",
            "POST",
            {"tx_id": "tx-2", "phase": "camera_only"},
        )
        assert c == 200
        assert rec["state"] == "COMMITTED"
        assert rec["tx_id"] == "tx-2"
        assert rec["camera_tx_id"] == "tx-2"

        # Idempotent second call
        c2, rec2 = _http_json(
            f"{base}/sim/scene/commit",
            "POST",
            {"tx_id": "tx-2", "phase": "camera_only"},
        )
        assert c2 == 200 and rec2["state"] == "COMMITTED"

        # Endpoints serve normally again
        assert _http_json(f"{base}/sim/camera")[0] == 200
        assert _http_json(f"{base}/api/detections")[0] == 200
        c_st, st_ok = _http_json(f"{base}/sim/state")
        assert c_st == 200 and st_ok["degraded"] is False and st_ok["camera_tx_id"] == "tx-2"
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_degraded_split_recovers_via_rollback_physics(tmp_path: Path):
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    cam_state = {"fail_commit": False}
    sim.set_camera_hooks(
        prepare_cb=lambda info: {"tx_id": info["tx_id"]},
        commit_cb=lambda staged: (
            (_ for _ in ()).throw(RuntimeError("camera process stalled"))
            if cam_state["fail_commit"]
            else None
        ),
        abort_cb=lambda staged: None,
    )
    sim.camera_info = lambda: {"fps": 15}  # type: ignore[attr-defined]
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        assert _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-1", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"},
        )[0] == 200
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-1"})[0] == 200

        # Prepare tx-2, fail camera commit -> DEGRADED_SPLIT
        assert _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-2", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"},
        )[0] == 200
        cam_state["fail_commit"] = True
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-2"})[0] == 503

        # Recovery path 2: rollback_physics restores scene_a / tx-1
        c, rb = _http_json(
            f"{base}/sim/scene/commit",
            "POST",
            {"tx_id": "tx-2", "phase": "rollback_physics"},
        )
        assert c == 200
        assert rb["state"] == "ABORTED"
        assert rb["restored_tx_id"] == "tx-1"
        assert rb["camera_tx_id"] == "tx-1"

        c_st, st = _http_json(f"{base}/sim/state")
        assert c_st == 200
        assert st["activation_state"] == "COMMITTED"
        assert st["degraded"] is False
        assert st["tx_id"] == "tx-1"
        assert st["camera_tx_id"] == "tx-1"
        assert st["scene"] == "scene_a"
        assert _http_json(f"{base}/sim/camera")[0] == 200
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_live_camera_subprocess_two_phase_tx():
    sim = Simulator(
        _free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount()
    )
    vport, pport = _free_port(), _free_port()
    ready = sim.start_camera(10, (320, 180), "127.0.0.1", vport, pport)
    if "error" in ready:
        sim.close()
        pytest.skip(ready["error"])
    runner = threading.Thread(target=sim.run, daemon=True)
    runner.start()
    try:
        base = f"http://127.0.0.1:{pport}"
        c, prep = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {
                "tx_id": "live-tx-1",
                "scene": "coffee_kiosk",
                "tool": "wide_cup_gripper",
                "robot": "b601dm",
                "seed": 5,
            },
        )
        assert c == 200, prep
        assert prep["state"] == "PREPARED"
        # Still on default until commit
        assert _http_json(f"{base}/sim/state")[1]["scene"] == "default"

        c, comm = _http_json(
            f"{base}/sim/scene/commit", "POST", {"tx_id": "live-tx-1"}
        )
        assert c == 200, comm
        assert comm["state"] == "COMMITTED"
        assert comm["tx_id"] == "live-tx-1"
        assert comm["camera_tx_id"] == "live-tx-1"

        st = _http_json(f"{base}/sim/state")[1]
        assert st["scene"] == "coffee_kiosk"
        assert st["tx_id"] == "live-tx-1"
        assert st["camera_tx_id"] == "live-tx-1"
        assert st["degraded"] is False
    finally:
        sim.stopping = True
        runner.join(2)
        sim.close()


# ---------------------------------------------------------------------------
# 3. Codex R1 Regression & Parity Tests (B1, B3, B4, B5, M6..M20, m1..m3)
# ---------------------------------------------------------------------------


def test_b1_preset_null_digests_and_m10_legacy_tx_identity(tmp_path: Path):
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        # B1: preset "default" scene + "default" / "suction" tool with null digests succeed
        c, prep = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {
                "tx_id": "preset-tx-1",
                "scene": "default",
                "tool": "suction",
                "scene_digest": None,
                "tool_digest": None,
            },
        )
        assert c == 200, prep
        assert prep["scene_digest"] is None and prep["tool_digest"] is None
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "preset-tx-1"})[0] == 200

        # Supplying a non-empty digest for a preset with no disk pack fails DIGEST_MISMATCH
        c_bad, bad = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {
                "tx_id": "preset-tx-bad",
                "scene": "default",
                "tool": "default",
                "scene_digest": "a" * 64,
            },
        )
        assert c_bad == 400 and bad["code"] == "DIGEST_MISMATCH"

        # M10: legacy POST /sim/scene assigns a fresh tx_id so replaying preset-tx-1 cannot return COMMITTED
        c_leg, leg = _http_json(
            f"{base}/sim/scene",
            "POST",
            {"scene": str(scene_a)} if False else {"scene": "coffee_kiosk", "tool": "wide_cup_gripper"},
        )
        assert c_leg == 200, leg
        assert leg["tx_id"].startswith("legacy-")
        assert leg["tx_id"] == leg["camera_tx_id"]

        # Replaying old tx_id must NOT falsely succeed as already COMMITTED
        c_rep, rep = _http_json(
            f"{base}/sim/scene/commit", "POST", {"tx_id": "preset-tx-1"}
        )
        assert c_rep == 409 and rep["code"] == "TX_NOT_PREPARED"
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_b3_concurrent_prepare_tx_busy_and_m6_ownership_replay_and_m3_phase(tmp_path: Path):
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    entered = threading.Event()
    release = threading.Event()

    def slow_cam_prepare(_info):
        entered.set()
        release.wait(timeout=5.0)
        return {"tx": _info["tx_id"]}

    sim.set_camera_hooks(prepare_cb=slow_cam_prepare)
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        # B3: concurrent prepare while another is PREPARING returns 409 TX_BUSY
        res1: dict = {}

        def run_prep1():
            c, r = _http_json(
                f"{base}/sim/scene/prepare",
                "POST",
                {"tx_id": "tx-1", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"},
            )
            res1["code"] = c
            res1["body"] = r

        t = threading.Thread(target=run_prep1)
        t.start()
        assert entered.wait(timeout=3.0)
        c2, r2 = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-2", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"},
        )
        assert c2 == 409 and r2["code"] == "TX_BUSY"
        release.set()
        t.join(timeout=5.0)
        assert res1["code"] == 200

        # M6: abort requires non-empty tx_id, and aborting a different tx_id does NOT discard tx-1
        c_no_id, r_no_id = _http_json(f"{base}/sim/scene/abort", "POST", {})
        assert c_no_id == 400 and r_no_id["code"] == "MISSING_TX_ID"
        c_other, _ = _http_json(f"{base}/sim/scene/abort", "POST", {"tx_id": "tx-other"})
        assert c_other == 200

        # m3: non-string phase returns 400 INVALID_PHASE instead of TypeError/500
        c_bad_phase, r_bad_phase = _http_json(
            f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-1", "phase": ["all"]}
        )
        assert c_bad_phase == 400 and r_bad_phase["code"] == "INVALID_PHASE"

        # tx-1 is still staged and commits cleanly
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "tx-1"})[0] == 200

        # M6: reusing already-committed tx_id with a different payload returns 409 TX_ID_CONFLICT
        c_conf, r_conf = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {"tx_id": "tx-1", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"},
        )
        assert c_conf == 409 and r_conf["code"] == "TX_ID_CONFLICT"
    finally:
        release.set()
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_m9_latest_cleared_on_swap_and_b4_b5_live_camera_rig(tmp_path: Path):
    import multiprocessing as mp
    import time

    from rebot_sim.perception import CameraRig, SharedState

    # Part A: M9 - Perception.latest cleared on scene swap
    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    perc = Perception(max_fps=15)
    perc.on_frame(1, 1000, time.monotonic(), [{"class_name": "cube"}])
    assert perc.latest is None  # continuous is off
    perc.mode, perc.enabled = "yolo", True
    perc.on_frame(2, 1010, time.monotonic(), [{"class_name": "cube"}])
    assert perc.latest is not None and perc.latest["frame_id"] == 2
    port = _free_port()
    srv = PerceptionServer(perc, sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        assert _http_json(f"{base}/api/detections")[1]["frame_id"] == 2
        assert _http_json(f"{base}/sim/scene", "POST", {"scene": "coffee_kiosk"})[0] == 200
        code, dets = _http_json(f"{base}/api/detections")
        assert code == 200 and dets["frame_id"] == 0 and dets.get("timeout") is True
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()

    # Part B: B4 (old camera stays alive after failed camera commit + rollback_physics) & B5 (digest mismatch)
    scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
    sim2 = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    vport, pport = _free_port(), _free_port()
    ready = sim2.start_camera(10, (320, 180), "127.0.0.1", vport, pport)
    if "error" in ready:
        sim2.close()
        pytest.skip(ready["error"])
    runner = threading.Thread(target=sim2.run, daemon=True)
    runner.start()
    try:
        cbase = f"http://127.0.0.1:{pport}"
        # B5: CameraRig._build_camera_world rejects digest mismatch between physics & camera
        lock = mp.get_context("spawn").Lock()
        st_dummy = SharedState(sim2.model.nq, sim2.model.nv, lock=lock)
        try:
            class _NoopLink:
                def call(self, *a, **kw):
                    return {}
            rig = CameraRig(
                {"fps": 10, "size": [320, 180], "mount": CameraMount().ipc(), "scene_stream": False, "noise": {}},
                st_dummy,
                _NoopLink(),
            )
            with pytest.raises(PackValidationError) as exc_b5:
                rig._build_camera_world(
                    str(scene_a),
                    "default",
                    "cube",
                    expected_scene_digest="0" * 64,
                    expected_tool_digest=None,
                    verify_digests=True,
                )
            assert exc_b5.value.code == "DIGEST_MISMATCH"
        finally:
            st_dummy.close()

        # Verify live camera renders after failed commit + rollback
        c, prep = _http_json(
            f"{cbase}/sim/scene/prepare",
            "POST",
            {"tx_id": "live-ok-1", "scene": "coffee_kiosk", "tool": "wide_cup_gripper", "robot": "b601dm"},
        )
        assert c == 200, prep
        assert _http_json(f"{cbase}/sim/scene/commit", "POST", {"tx_id": "live-ok-1"})[0] == 200
        time.sleep(0.2)
        st1 = _http_json(f"{cbase}/sim/state")[1]
        assert st1["scene"] == "coffee_kiosk" and st1["camera"]["frame_id"] > 0
    finally:
        sim2.stopping = True
        runner.join(2)
        sim2.close()


def test_b4_camera_commit_failure_preserves_old_renderer_and_rollback(tmp_path: Path):
    import multiprocessing as mp

    from rebot_sim.perception import CameraRig, SharedState

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    lock = mp.get_context("spawn").Lock()
    st = SharedState(sim.model.nq, sim.model.nv, lock=lock)
    st.write(sim.data)
    # Assign sim.shared so _tx_prepare_locked allocates _shm and CameraRig actually stages
    sim.shared = st

    fin_ctl = {"fail": False}
    rb_calls = []

    class _PipeAdapter:
        def call(self, msg, timeout=None):
            op = msg[0]
            with sim.lock:
                if op == "tx_prepare":
                    return sim._tx_prepare_locked(msg[1])
                if op == "tx_commit_physics":
                    return sim._tx_commit_physics_locked(msg[1])
                if op == "tx_mark_degraded":
                    return sim._tx_mark_degraded_locked(msg[1])
                if op == "tx_finalize":
                    if fin_ctl["fail"]:
                        return {"_http_status": 500, "error": "injected finalize fail"}
                    return sim._tx_finalize_locked(msg[1])
                if op == "tx_rollback_physics":
                    rb_calls.append(msg[1])
                    return sim._tx_rollback_physics_locked(msg[1])
                if op == "tx_abort":
                    return sim._tx_abort_locked(msg[1])
                if op == "state":
                    return sim._state_locked()
            return {}

    rig = CameraRig(
        {
            "fps": 10,
            "size": [320, 180],
            "mount": CameraMount().ipc(),
            "scene_stream": False,
            "has_meshes": False,
            "noise": {},
        },
        SharedState(sim.model.nq, sim.model.nv, name=st.shm.name, lock=lock),
        _PipeAdapter(),
    )
    setup_err: dict = {}
    setup_done = threading.Event()
    stop = threading.Event()

    def cam_thread():
        try:
            rig.setup()
        except Exception as exc:
            setup_err["err"] = exc
            setup_done.set()
            return
        setup_done.set()
        try:
            while not stop.is_set():
                req = None
                with rig.reload_lock:
                    if rig._reload_req is not None:
                        req, rig._reload_req = rig._reload_req, None
                if req is not None:
                    op = req["op"]
                    if op == "stage":
                        rig._stage_camera_tx(req)
                    elif op == "commit_staged":
                        rig._commit_staged_camera_tx(req)
                    elif op == "finalize_cam":
                        rig._finalize_camera_tx(req)
                    elif op == "restore_prev_cam":
                        rig._restore_prev_camera_tx(req)
                    elif op == "abort_staged":
                        rig._abort_staged_camera_tx(req)
                    elif op == "render_check":
                        try:
                            rig.renderer.update_scene(rig.data, camera="wrist", scene_option=rig.opt)
                            req["shape"] = rig.renderer.render().shape
                        except Exception as exc:
                            req["err_box"]["error"] = str(exc)
                    req["done"].set()
                stop.wait(0.01)
        finally:
            for obj in (getattr(rig, "renderer", None), getattr(rig, "seg_renderer", None), rig.state):
                if obj is not None:
                    try:
                        obj.close()
                    except Exception:
                        pass

    t = threading.Thread(target=cam_thread, daemon=True)
    t.start()
    setup_done.wait(timeout=10.0)
    if "err" in setup_err:
        stop.set()
        t.join(2)
        sim.close()
        pytest.skip(f"MuJoCo GL unavailable: {setup_err['err']}")
    try:
        old_renderer = rig.renderer
        old_seg = rig.seg_renderer
        old_state = rig.state
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")

        c, prep = rig.prepare_scene(
            {"tx_id": "tx-cam-fail", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"}
        )
        assert c == 200, prep
        assert rig._staged_cam_tx is not None  # camera actually staged!

        # M12: Force camera commit hook to raise and verify it WAS actually invoked
        hook_calls = {"count": 0}

        def fail_once(_staged):
            hook_calls["count"] += 1
            raise RuntimeError("swap fail")

        rig._before_camera_commit_hook = fail_once
        c_com, deg = rig.commit_scene({"tx_id": "tx-cam-fail"})
        assert hook_calls["count"] == 1
        assert c_com == 503 and deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.renderer is old_renderer
        assert rig.seg_renderer is old_seg
        assert rig.state is old_state
        chk1 = {"op": "render_check", "done": threading.Event(), "err_box": {}}
        assert rig._run_reload_op(chk1, timeout=5.0) is None
        assert chk1["shape"] == (180, 320, 3)

        # M04: If camera abort fails during rollback_physics, return 503 CAMERA_ABORT_FAILED WITHOUT calling tx_rollback_physics
        abort_calls = {"count": 0}

        def abort_fail(_staged):
            abort_calls["count"] += 1
            raise RuntimeError("cam abort failed")

        rig._before_camera_abort_hook = abort_fail
        c_rb_fail, rb_fail = rig.commit_scene({"tx_id": "tx-cam-fail", "phase": "rollback_physics"})
        assert abort_calls["count"] == 1
        assert c_rb_fail == 503 and rb_fail["code"] == "CAMERA_ABORT_FAILED"
        assert len(rb_calls) == 0  # physics rollback snapshot preserved!

        # Retry rollback_physics cleanly -> succeeds
        rig._before_camera_abort_hook = None
        c_rb, rb = rig.commit_scene({"tx_id": "tx-cam-fail", "phase": "rollback_physics"})
        assert c_rb == 200 and rb["state"] == "ABORTED"
        assert len(rb_calls) == 1
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
        assert sim.scene_pack is None
        chk2 = {"op": "render_check", "done": threading.Event(), "err_box": {}}
        assert rig._run_reload_op(chk2, timeout=5.0) is None
        assert chk2["shape"] == (180, 320, 3)

        # B03: If camera swap succeeds but tx_finalize fails:
        # 1) Retry via camera_only skips re-swapping and succeeds once finalize recovers
        assert rig.prepare_scene(
            {"tx_id": "tx-fin-retry", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"}
        )[0] == 200
        rig._before_camera_commit_hook = None
        fin_ctl["fail"] = True
        c_fin_fail, deg2 = rig.commit_scene({"tx_id": "tx-fin-retry"})
        assert c_fin_fail == 503 and deg2["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.camera_committed_tx_id == "tx-fin-retry"
        assert rig._prev_cam_gen is not None  # previous renderer still preserved!

        fin_ctl["fail"] = False
        c_fin_ok, rec_ok = rig.commit_scene({"tx_id": "tx-fin-retry", "phase": "camera_only"})
        assert c_fin_ok == 200 and rec_ok["state"] == "COMMITTED"
        assert rig.camera_committed_tx_id is None and rig._prev_cam_gen is None

        # 2) If tx_finalize fails and caller chooses rollback_physics, BOTH camera and physics roll back
        assert rig.prepare_scene(
            {"tx_id": "tx-fin-rb", "scene": "default", "tool": "default"}
        )[0] == 200
        fin_ctl["fail"] = True
        assert rig.commit_scene({"tx_id": "tx-fin-rb"})[0] == 503
        assert rig.camera_committed_tx_id == "tx-fin-rb"
        assert rig._prev_cam_gen is not None

        fin_ctl["fail"] = False
        c_rb2, rb2 = rig.commit_scene({"tx_id": "tx-fin-rb", "phase": "rollback_physics"})
        assert c_rb2 == 200 and rb2["state"] == "ABORTED"
        assert rig.activation_state == "COMMITTED"
        assert sim.scene_pack is not None and sim.scene_pack.id == "scene_a"
        assert rig.camera_committed_tx_id is None and rig._prev_cam_gen is None
    finally:
        stop.set()
        t.join(2)
        sim.close()


def test_m12_m13_m15_m16_m18_m19_m20_m1_m2_validation_edge_cases(tmp_path: Path):
    from rebot_sim.pack import is_valid_pack_slug, normalize_robot_id

    # m1: ASCII-only lowercase normalization
    assert normalize_robot_id("B601_DM") == "b601dm"
    assert normalize_robot_id("SO_ARM101") == "so-arm101"
    assert normalize_robot_id("\u0130") == "\u0130"  # Turkish capital I with dot not mangled by .lower()

    # M13: fullmatch & no .strip()
    assert is_valid_pack_slug("good_pack-1") is True
    assert is_valid_pack_slug("bad_pack\n") is False
    assert is_valid_pack_slug(" bad_pack") is False

    # m2: binary STL whose 80-byte header starts with b"solid " is still validated as binary STL
    tri = ((0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.0, 0.05, 0.0))
    bin_with_solid_hdr = bytearray(b"solid binary header".ljust(80, b"\x00"))
    bin_with_solid_hdr.extend(struct.pack("<I", 1))
    bin_with_solid_hdr.extend(struct.pack("<12fH", 0.0, 0.0, 1.0, *tri[0], *tri[1], *tri[2], 0))
    p_m2 = _write_minimal_scene(
        tmp_path / "m2_pack",
        world_xml='<mujoco><asset><mesh name="m" file="s.stl"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>',
    )
    (p_m2 / "s.stl").write_bytes(bytes(bin_with_solid_hdr))
    assert len(validate_pack_security(p_m2, "scene")) == 64

    # M12: hidden dotfiles (e.g. .DS_Store) are skipped and don't affect digest
    (p_m2 / ".DS_Store").write_bytes(b"junk")
    d_with_dot = compute_pack_digest(p_m2)
    (p_m2 / ".DS_Store").unlink()
    assert d_with_dot == compute_pack_digest(p_m2)

    # M19: inline <mesh vertex="..."> counts toward 16 mesh limit & 150_000 vertex budget
    many_inline = "".join(
        f'<mesh name="im{i}" vertex="0 0 0 1 0 0 0 1 0"/>' for i in range(17)
    )
    p_m19 = _write_minimal_scene(
        tmp_path / "m19_pack",
        world_xml=f"<mujoco><asset>{many_inline}</asset><worldbody/></mujoco>",
    )
    with pytest.raises(PackValidationError) as exc_m19:
        validate_pack_security(p_m19, "scene")
    assert exc_m19.value.code == "PACK_TOO_LARGE"

    # M20: malformed ASCII STL (vertex outside outer loop / missing endloop)
    p_m20_stl = _write_minimal_scene(
        tmp_path / "m20_stl",
        world_xml='<mujoco><asset><mesh name="m" file="bad.stl"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>',
    )
    (p_m20_stl / "bad.stl").write_text(
        "solid bad\n  facet normal 0 0 1\n    vertex 0 0 0\n  endfacet\nendsolid bad\n",
        encoding="utf-8",
    )
    with pytest.raises(PackValidationError) as exc_stl:
        validate_pack_security(p_m20_stl, "scene")
    assert exc_stl.value.code == "INVALID_PACK"

    # M20: OBJ out-of-bounds face index
    p_m20_obj = _write_minimal_scene(
        tmp_path / "m20_obj",
        world_xml='<mujoco><asset><mesh name="m" file="bad.obj"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>',
    )
    (p_m20_obj / "bad.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n", encoding="utf-8"
    )
    with pytest.raises(PackValidationError) as exc_obj:
        validate_pack_security(p_m20_obj, "scene")
    assert exc_obj.value.code == "INVALID_PACK"

    # M15 + M16: secondary XML in pack & <compiler> appearing after <asset> & texture fileright
    p_m16 = _write_minimal_scene(
        tmp_path / "m16_pack",
        world_xml=(
            '<mujoco><asset><mesh name="m" file="sub.stl"/></asset>'
            '<compiler meshdir="meshes"/><worldbody/></mujoco>'
        ),
    )
    (p_m16 / "meshes").mkdir()
    (p_m16 / "meshes" / "sub.stl").write_bytes(_make_binary_stl([tri]))
    (p_m16 / "extra.xml").write_text('<mujoco><include file="evil.xml"/></mujoco>', encoding="utf-8")
    with pytest.raises(PackValidationError):
        validate_pack_security(p_m16, "scene")
    (p_m16 / "extra.xml").unlink()
    assert len(validate_pack_security(p_m16, "scene")) == 64

    # M18: standalone XML validates stem slug and includes referenced mesh in digest
    sa_dir = tmp_path / "standalone"
    sa_dir.mkdir()
    (sa_dir / "cube.stl").write_bytes(_make_binary_stl([tri]))
    sa_xml = sa_dir / "valid_standalone.xml"
    sa_xml.write_text(
        '<mujoco><asset><mesh name="m" file="cube.stl"/></asset><worldbody/></mujoco>',
        encoding="utf-8",
    )
    sp_sa = load_scene_pack(sa_xml)
    assert sp_sa is not None and sp_sa.digest is not None
    (sa_dir / "cube.stl").write_bytes(_make_binary_stl([((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0))]))
    sp_sa2 = load_scene_pack(sa_xml)
    assert sp_sa2 is not None and sp_sa2.digest != sp_sa.digest

    # M11: hidden XML or hidden referenced mesh rejected (cannot bypass digest)
    p_m11 = _write_minimal_scene(
        tmp_path / "m11_hidden",
        world_xml='<mujoco><asset><mesh name="m" file=".secret.stl"/></asset><worldbody/></mujoco>',
    )
    (p_m11 / ".secret.stl").write_bytes(_make_binary_stl([tri]))
    with pytest.raises(PackValidationError) as exc_m11:
        validate_pack_security(p_m11, "scene")
    assert exc_m11.value.code == "PATH_ESCAPE"

    p_m11_xml = _write_minimal_scene(
        tmp_path / "m11_hidden_xml",
        extra_manifest={"world_xml": ".world.xml"},
    )
    (p_m11_xml / ".world.xml").write_text("<mujoco><worldbody/></mujoco>", encoding="utf-8")
    with pytest.raises(PackValidationError) as exc_m11_xml:
        validate_pack_security(p_m11_xml, "scene")
    assert exc_m11_xml.value.code == "PATH_ESCAPE"


def test_m01_explicit_tool_dir_suction_loads_from_disk_with_digest(tmp_path: Path):
    tool_dir = tmp_path / "suction"
    tool_dir.mkdir()
    (tool_dir / "tool.xml").write_text(
        '<mujoco model="disk_suction"><worldbody>'
        '<body name="suction_valve_body"><joint name="suction_valve" type="slide" axis="0 1 0" range="-0.05 0.05"/>'
        '<inertial pos="0 0 0" mass="0.01" diaginertia="1e-6 1e-6 1e-6"/>'
        '<geom name="cup" type="sphere" size="0.01"/></body></worldbody>'
        '<actuator><position name="m7_suction" joint="suction_valve" kp="20"/></actuator></mujoco>',
        encoding="utf-8",
    )
    (tool_dir / "tool.json").write_text(
        json.dumps(
            {
                "id": "suction",
                "name": "Disk Suction",
                "kind": "suction",
                "tool_xml": "tool.xml",
                "driven_joint": "suction_valve",
                "actuator": "m7_suction",
            }
        ),
        encoding="utf-8",
    )
    disk_digest = compute_pack_digest(tool_dir)

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", robot="b601dm")
    port = _free_port()
    srv = PerceptionServer(Perception(max_fps=15), sim, "127.0.0.1", port)
    base = f"http://127.0.0.1:{port}"
    try:
        c, prep = _http_json(
            f"{base}/sim/scene/prepare",
            "POST",
            {
                "tx_id": "m01-disk-suction",
                "scene": "default",
                "tool": "suction",
                "tool_dir": str(tool_dir),
                "tool_digest": disk_digest,
            },
        )
        assert c == 200, prep
        assert prep["tool"] == "suction"
        assert prep["tool_digest"] == disk_digest
        assert _http_json(f"{base}/sim/scene/commit", "POST", {"tx_id": "m01-disk-suction"})[0] == 200
    finally:
        srv.httpd.shutdown()
        srv.httpd.server_close()
        sim.close()


def test_b02_camera_commit_not_cancelled_after_effective_swap():
    import multiprocessing as mp

    from rebot_sim.perception import CameraRig, SharedState

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube")
    lock = mp.get_context("spawn").Lock()
    st = SharedState(sim.model.nq, sim.model.nv, lock=lock)
    rig = CameraRig(
        {
            "fps": 10,
            "size": [320, 180],
            "mount": CameraMount().ipc(),
            "scene_stream": False,
            "has_meshes": False,
            "noise": {},
        },
        st,
        type("_L", (), {"call": lambda *a, **kw: {}})(),
    )
    try:
        # Simulate a swap that sets committed=True right as timeout expires
        ev = threading.Event()
        req = {"op": "commit_staged", "done": ev, "err_box": {}, "cancelled": False, "committed": False}

        def fake_worker():
            with rig.reload_lock:
                req["committed"] = True
                ev.set()

        orig_wait = ev.wait

        def timed_wait(timeout=None):
            fake_worker()
            return False  # simulate wait returning False right at deadline

        ev.wait = timed_wait  # type: ignore[method-assign]
        err = rig._run_reload_op(req, timeout=0.01)
        assert err is None
        assert req["cancelled"] is False
    finally:
        st.close()
        sim.close()


@pytest.mark.parametrize(
    "bad_xml",
    [
        '<mujoco><worldbody><replicate count="10"><body name="r"/></replicate></worldbody></mujoco>',
        '<mujoco><worldbody><frame pos="0 0 0"><body name="f"/></frame></worldbody></mujoco>',
        '<mujoco><include file="other.xml"/><worldbody/></mujoco>',
        '<mujoco><worldbody><composite type="grid" count="2 2 1"/></worldbody></mujoco>',
        '<mujoco><worldbody><flexcomp name="fx" type="grid" count="2 2 1"/></worldbody></mujoco>',
        '<mujoco><worldbody><attach model="m" body="b"/></worldbody></mujoco>',
        '<mujoco><asset><model name="m" file="sub.xml"/></asset><worldbody/></mujoco>',
    ],
)
def test_m5_rejects_expansion_constructs(tmp_path: Path, bad_xml: str):
    pdir = _write_minimal_scene(tmp_path / "exp_scene", world_xml=bad_xml)
    with pytest.raises(PackValidationError) as exc:
        validate_pack_security(pdir, "scene")
    assert exc.value.code == "INVALID_PACK"


def test_m5_enforces_exact_and_combined_element_body_geom_budgets(tmp_path: Path):
    import mujoco

    from rebot_sim.model import SCENE_XML

    assert MAX_XML_ELEMENTS == 4_096
    assert MAX_PACK_BODIES == 512
    assert MAX_PACK_GEOMS == 1_024

    def make_bodies_xml(n: int) -> str:
        inner = "".join(f'<body name="b{i}"/>' for i in range(n))
        return f"<mujoco><worldbody>{inner}</worldbody></mujoco>"

    def make_geoms_xml(n: int) -> str:
        inner = "".join(
            f'<geom name="g{i}" type="box" size="0.01 0.01 0.01"/>' for i in range(n)
        )
        return f"<mujoco><worldbody>{inner}</worldbody></mujoco>"

    # Exact MAX_PACK_BODIES accepted, MAX_PACK_BODIES + 1 rejected
    ok_b = _write_minimal_scene(tmp_path / "ok_bodies", world_xml=make_bodies_xml(MAX_PACK_BODIES))
    assert len(validate_pack_security(ok_b, "scene")) == 64

    over_b = _write_minimal_scene(tmp_path / "over_bodies", world_xml=make_bodies_xml(MAX_PACK_BODIES + 1))
    with pytest.raises(PackValidationError) as exc_b:
        validate_pack_security(over_b, "scene")
    assert exc_b.value.code == "INVALID_PACK"

    # Exact MAX_PACK_GEOMS accepted, MAX_PACK_GEOMS + 1 rejected
    ok_g = _write_minimal_scene(tmp_path / "ok_geoms", world_xml=make_geoms_xml(MAX_PACK_GEOMS))
    assert len(validate_pack_security(ok_g, "scene")) == 64

    over_g = _write_minimal_scene(tmp_path / "over_geoms", world_xml=make_geoms_xml(MAX_PACK_GEOMS + 1))
    with pytest.raises(PackValidationError) as exc_g:
        validate_pack_security(over_g, "scene")
    assert exc_g.value.code == "INVALID_PACK"

    # Combined scene (300 bodies) + tool (250 bodies) = 550 > 512: individually valid,
    # rejected by compose_spec BEFORE MuJoCo compilation!
    scene_dir = _write_minimal_scene(
        tmp_path / "scene_300", pack_id="scene_300", world_xml=make_bodies_xml(300)
    )
    tool_dir = tmp_path / "tool_250"
    tool_dir.mkdir(parents=True, exist_ok=True)
    (tool_dir / "tool.xml").write_text(make_bodies_xml(250), encoding="utf-8")
    (tool_dir / "tool.json").write_text(
        json.dumps({"id": "tool_250", "name": "Tool 250", "kind": "gripper", "tool_xml": "tool.xml"}),
        encoding="utf-8",
    )

    sp = load_scene_pack(scene_dir, prefer_disk=True)
    tp = load_tool_pack(tool_dir, prefer_disk=True)
    assert sp is not None and tp is not None

    spec = mujoco.MjSpec.from_file(str(SCENE_XML))
    with pytest.raises(PackValidationError) as exc_comb:
        compose_spec(spec, scene_pack=sp, tool_pack=tp)
    assert exc_comb.value.code == "INVALID_PACK"


def _make_lost_ack_harness(tmp_path: Path):
    import multiprocessing as mp

    from rebot_sim.perception import CameraRig, SharedState

    sim = Simulator(_free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=CameraMount())
    lock = mp.get_context("spawn").Lock()
    st = SharedState(sim.model.nq, sim.model.nv, lock=lock)
    st.write(sim.data)
    sim.shared = st

    faults: dict[str, str | None] = {
        "tx_commit_physics": None,  # "drop_before" | "drop_after"
        "tx_finalize": None,        # "drop_after"
        "tx_rollback_physics": None,  # "drop_after"
        "tx_mark_degraded": None,   # "drop_after"
    }

    class _FaultyPipeAdapter:
        def call(self, msg, timeout=None):
            op = msg[0]
            mode = faults.get(op)
            if mode == "drop_before":
                faults[op] = None
                raise TimeoutError(f"simulated request loss before {op}")
            with sim.lock:
                if op == "tx_prepare":
                    res = sim._tx_prepare_locked(msg[1])
                elif op == "tx_commit_physics":
                    res = sim._tx_commit_physics_locked(msg[1])
                elif op == "tx_mark_degraded":
                    res = sim._tx_mark_degraded_locked(msg[1])
                elif op == "tx_finalize":
                    res = sim._tx_finalize_locked(msg[1])
                elif op == "tx_rollback_physics":
                    res = sim._tx_rollback_physics_locked(msg[1])
                elif op == "tx_abort":
                    res = sim._tx_abort_locked(msg[1])
                elif op == "state":
                    res = sim._state_locked()
                elif op == "tx_commit":
                    res = sim._tx_commit_locked(msg[1])
                else:
                    res = {}
            if mode == "drop_after":
                faults[op] = None
                raise TimeoutError(f"simulated reply loss after {op}")
            return res

    rig = CameraRig(
        {
            "fps": 10,
            "size": [320, 180],
            "mount": CameraMount().ipc(),
            "scene_stream": False,
            "has_meshes": False,
            "noise": {},
        },
        SharedState(sim.model.nq, sim.model.nv, name=st.shm.name, lock=lock),
        _FaultyPipeAdapter(),
    )
    setup_err: dict = {}
    setup_done = threading.Event()
    stop = threading.Event()

    def cam_thread():
        try:
            rig.setup()
        except Exception as exc:
            setup_err["err"] = exc
            setup_done.set()
            return
        setup_done.set()
        try:
            while not stop.is_set():
                req = None
                with rig.reload_lock:
                    if rig._reload_req is not None:
                        req, rig._reload_req = rig._reload_req, None
                if req is not None:
                    op = req["op"]
                    if op == "stage":
                        rig._stage_camera_tx(req)
                    elif op == "commit_staged":
                        rig._commit_staged_camera_tx(req)
                    elif op == "finalize_cam":
                        rig._finalize_camera_tx(req)
                    elif op == "restore_prev_cam":
                        rig._restore_prev_camera_tx(req)
                    elif op == "abort_staged":
                        rig._abort_staged_camera_tx(req)
                    req["done"].set()
                stop.wait(0.01)
        finally:
            for obj in (getattr(rig, "renderer", None), getattr(rig, "seg_renderer", None), rig.state):
                if obj is not None:
                    try:
                        obj.close()
                    except Exception:
                        pass

    t = threading.Thread(target=cam_thread, daemon=True)
    t.start()
    setup_done.wait(timeout=10.0)
    if "err" in setup_err:
        stop.set()
        t.join(2)
        sim.close()
        pytest.skip(f"MuJoCo GL unavailable: {setup_err['err']}")

    def cleanup():
        stop.set()
        t.join(2)
        sim.close()

    return sim, rig, faults, cleanup


def _cam_scene_id(rig) -> str:
    sp = getattr(rig.scene, "scene_pack", None)
    return sp.id if sp is not None else "default"


def _phys_scene_id(sim) -> str:
    return sim.scene_pack.id if sim.scene_pack is not None else "default"


def test_m6_request_lost_before_physics_applies_recovers_via_rollback_or_all(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        # Establish initial committed baseline (scene_a, tx-init)
        assert rig.prepare_scene({"tx_id": "tx-init", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-init"})[0] == 200

        # Subtest 1: request lost BEFORE tx_commit_physics applies -> camera_only refused 409,
        # rollback_physics restores both to scene_a / tx-init.
        assert rig.prepare_scene({"tx_id": "tx-lost-req-1", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        faults["tx_commit_physics"] = "drop_before"
        c_fail, d_fail = rig.commit_scene({"tx_id": "tx-lost-req-1"})
        assert c_fail == 503 and d_fail["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert rig.pending_tx_id == "tx-lost-req-1"

        c_cam, r_cam = rig.commit_scene({"tx_id": "tx-lost-req-1", "phase": "camera_only"})
        assert c_cam == 409 and r_cam["code"] == "PHYSICS_NOT_COMMITTED"
        assert _cam_scene_id(rig) == "scene_a"

        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-lost-req-1", "phase": "rollback_physics"})
        assert c_rb == 200 and r_rb["state"] == "ABORTED"
        assert _phys_scene_id(sim) == "scene_a" and sim.active_tx_id == "tx-init"
        assert _cam_scene_id(rig) == "scene_a" and rig.camera_tx_id == "tx-init"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"

        # Subtest 2: request lost BEFORE tx_commit_physics applies -> phase="all" retries & commits both to scene_b
        assert rig.prepare_scene({"tx_id": "tx-lost-req-2", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        faults["tx_commit_physics"] = "drop_before"
        c_fail2, _ = rig.commit_scene({"tx_id": "tx-lost-req-2"})
        assert c_fail2 == 503

        c_all, r_all = rig.commit_scene({"tx_id": "tx-lost-req-2", "phase": "all"})
        assert c_all == 200 and r_all["state"] == "COMMITTED"
        assert _phys_scene_id(sim) == "scene_b" and sim.active_tx_id == "tx-lost-req-2"
        assert _cam_scene_id(rig) == "scene_b" and rig.camera_tx_id == "tx-lost-req-2"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
    finally:
        cleanup()


def test_m6_reply_lost_after_tx_commit_physics_recovers_via_camera_only_and_rollback(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        assert rig.prepare_scene({"tx_id": "tx-base", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-base"})[0] == 200

        # Case 1: reply lost AFTER tx_commit_physics applies -> recovers via camera_only (both on scene_b)
        assert rig.prepare_scene({"tx_id": "tx-phys-ack-1", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        faults["tx_commit_physics"] = "drop_after"
        c1, d1 = rig.commit_scene({"tx_id": "tx-phys-ack-1"})
        assert c1 == 503 and d1["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert rig.pending_tx_id == "tx-phys-ack-1"
        assert _phys_scene_id(sim) == "scene_b"
        assert _cam_scene_id(rig) == "scene_a"

        c_rec, r_rec = rig.commit_scene({"tx_id": "tx-phys-ack-1", "phase": "camera_only"})
        assert c_rec == 200 and r_rec["state"] == "COMMITTED"
        assert _phys_scene_id(sim) == "scene_b" and sim.active_tx_id == "tx-phys-ack-1"
        assert _cam_scene_id(rig) == "scene_b" and rig.camera_tx_id == "tx-phys-ack-1"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"

        # Case 2: reply lost AFTER tx_commit_physics applies -> recovers via rollback_physics (both on scene_b)
        assert rig.prepare_scene({"tx_id": "tx-phys-ack-2", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        faults["tx_commit_physics"] = "drop_after"
        c2, _ = rig.commit_scene({"tx_id": "tx-phys-ack-2"})
        assert c2 == 503 and rig.pending_tx_id == "tx-phys-ack-2"

        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-phys-ack-2", "phase": "rollback_physics"})
        assert c_rb == 200 and r_rb["state"] == "ABORTED"
        assert _phys_scene_id(sim) == "scene_b" and sim.active_tx_id == "tx-phys-ack-1"
        assert _cam_scene_id(rig) == "scene_b" and rig.camera_tx_id == "tx-phys-ack-1"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
    finally:
        cleanup()


def test_m6_reply_lost_after_tx_finalize_refuses_rollback_and_heals_idempotently(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        assert rig.prepare_scene({"tx_id": "tx-base", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-base"})[0] == 200

        # Reply lost AFTER tx_finalize commits on physics (also drop tx_mark_degraded reply to test try/except)
        assert rig.prepare_scene({"tx_id": "tx-fin-lost", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        faults["tx_finalize"] = "drop_after"
        faults["tx_mark_degraded"] = "drop_after"
        c_deg, d_deg = rig.commit_scene({"tx_id": "tx-fin-lost"})
        assert c_deg == 503 and d_deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert rig.pending_tx_id == "tx-fin-lost"
        assert rig.sim_state()["activation_state"] == "DEGRADED_SPLIT"

        # rollback_physics MUST be refused 409 TX_ALREADY_COMMITTED and camera NOT rolled back!
        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-fin-lost", "phase": "rollback_physics"})
        assert c_rb == 409 and r_rb["code"] == "TX_ALREADY_COMMITTED"
        assert _cam_scene_id(rig) == "scene_b"

        # camera_only calls idempotent finalize -> 200 COMMITTED, both on scene_b, serving resumes!
        c_fin, r_fin = rig.commit_scene({"tx_id": "tx-fin-lost", "phase": "camera_only"})
        assert c_fin == 200 and r_fin["state"] == "COMMITTED"
        assert _phys_scene_id(sim) == "scene_b" and sim.active_tx_id == "tx-fin-lost"
        assert _cam_scene_id(rig) == "scene_b" and rig.camera_tx_id == "tx-fin-lost"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
        assert rig.sim_state()["degraded"] is False
    finally:
        cleanup()


def test_m6_reply_lost_after_tx_rollback_physics_reconciles_idempotently(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        assert rig.prepare_scene({"tx_id": "tx-base", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-base"})[0] == 200

        assert rig.prepare_scene({"tx_id": "tx-rb-lost", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        rig._before_camera_commit_hook = lambda _staged: (_ for _ in ()).throw(RuntimeError("cam swap err"))
        c_deg, _ = rig.commit_scene({"tx_id": "tx-rb-lost"})
        assert c_deg == 503 and rig.activation_state == "DEGRADED_SPLIT"
        rig._before_camera_commit_hook = None

        # Reply lost AFTER tx_rollback_physics completes on physics
        faults["tx_rollback_physics"] = "drop_after"
        with pytest.raises(TimeoutError):
            rig.commit_scene({"tx_id": "tx-rb-lost", "phase": "rollback_physics"})
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert sim.activation_state == "COMMITTED"
        assert _phys_scene_id(sim) == "scene_a" and sim.active_tx_id == "tx-base"

        # Retry rollback_physics reconciles cleanly -> 200 ABORTED, both on scene_a
        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-rb-lost", "phase": "rollback_physics"})
        assert c_rb == 200 and r_rb["state"] == "ABORTED"
        assert _phys_scene_id(sim) == "scene_a" and sim.active_tx_id == "tx-base"
        assert _cam_scene_id(rig) == "scene_a" and rig.camera_tx_id == "tx-base"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
    finally:
        cleanup()


def test_m09a_lost_nack_preserves_physics_active_tx_and_recovers(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        assert rig.prepare_scene({"tx_id": "tx-good", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-good"})[0] == 200

        # Physics rejects unprepared tx-bad with 409, and reply is lost -> camera enters DEGRADED_SPLIT
        faults["tx_commit_physics"] = "drop_after"
        c_deg, d_deg = rig.commit_scene({"tx_id": "tx-bad"})
        assert c_deg == 503 and d_deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert sim.active_tx_id == "tx-good"
        assert sim.degraded_tx_id == "tx-bad"
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert rig.pending_tx_id == "tx-bad"
        assert rig.active_tx_id == "tx-good"

        # camera_only for unprepared tx-bad must be refused 409 TX_NOT_PREPARED
        c_cam, r_cam = rig.commit_scene({"tx_id": "tx-bad", "phase": "camera_only"})
        assert c_cam == 409 and r_cam["code"] == "TX_NOT_PREPARED"

        # rollback_physics for tx-bad restores both to COMMITTED on tx-good's scene_a
        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-bad", "phase": "rollback_physics"})
        assert c_rb == 200 and r_rb["state"] == "ABORTED"
        assert sim.activation_state == "COMMITTED" and sim.active_tx_id == "tx-good"
        assert sim.degraded_tx_id is None
        assert rig.activation_state == "COMMITTED" and rig.active_tx_id == "tx-good"
        assert rig.camera_tx_id == "tx-good" and rig.pending_tx_id is None
        assert _phys_scene_id(sim) == "scene_a" and _cam_scene_id(rig) == "scene_a"
    finally:
        cleanup()


def test_m09b_lost_nack_preserves_unrelated_staged_tx(tmp_path: Path):
    sim, rig, faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        assert rig.prepare_scene({"tx_id": "tx-good", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-good"})[0] == 200

        # Stage a different tx-other on scene_b
        assert rig.prepare_scene({"tx_id": "tx-other", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200
        assert sim._staged_tx is not None and sim._staged_tx["tx_id"] == "tx-other"

        # Unrelated tx-bad commit hits 409 on physics and reply is lost
        faults["tx_commit_physics"] = "drop_after"
        c_deg, d_deg = rig.commit_scene({"tx_id": "tx-bad"})
        assert c_deg == 503 and d_deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert sim.active_tx_id == "tx-good"
        assert sim._staged_tx is not None and sim._staged_tx["tx_id"] == "tx-other"
        assert sim.degraded_tx_id == "tx-bad"
        assert rig.pending_tx_id == "tx-bad"

        # rollback_physics for tx-bad clears DEGRADED_SPLIT while preserving tx-good AND staged tx-other!
        c_rb, r_rb = rig.commit_scene({"tx_id": "tx-bad", "phase": "rollback_physics"})
        assert c_rb == 200 and r_rb["state"] == "ABORTED"
        assert sim.activation_state == "COMMITTED"
        assert sim.active_tx_id == "tx-good"
        assert sim.degraded_tx_id is None
        assert sim._staged_tx is not None and sim._staged_tx["tx_id"] == "tx-other"
        assert rig.activation_state == "COMMITTED" and rig.active_tx_id == "tx-good"
        assert rig.pending_tx_id is None

        # The preserved staged tx-other can still commit cleanly!
        c_ok, r_ok = rig.commit_scene({"tx_id": "tx-other"})
        assert c_ok == 200 and r_ok["state"] == "COMMITTED"
        assert _phys_scene_id(sim) == "scene_b" and sim.active_tx_id == "tx-other"
        assert _cam_scene_id(rig) == "scene_b" and rig.camera_tx_id == "tx-other"
    finally:
        cleanup()


def test_m16_finalize_cam_timeout_tracks_pending_cleanup_and_retries_cleanly(tmp_path: Path):
    sim, rig, _faults, cleanup = _make_lost_ack_harness(tmp_path)
    try:
        scene_a = _write_minimal_scene(tmp_path / "scene_a", pack_id="scene_a")
        scene_b = _write_minimal_scene(tmp_path / "scene_b", pack_id="scene_b")

        assert rig.prepare_scene({"tx_id": "tx-base", "scene": "scene_a", "scene_dir": str(scene_a), "tool": "default"})[0] == 200
        assert rig.commit_scene({"tx_id": "tx-base"})[0] == 200

        assert rig.prepare_scene({"tx_id": "tx-m16", "scene": "scene_b", "scene_dir": str(scene_b), "tool": "default"})[0] == 200

        orig_run_reload_op = rig._run_reload_op
        fail_once = {"armed": True}

        def _injected_reload_op(req, timeout=5.0):
            if req.get("op") == "finalize_cam" and fail_once["armed"]:
                fail_once["armed"] = False
                return "camera reload timed out"
            return orig_run_reload_op(req, timeout=timeout)

        rig._run_reload_op = _injected_reload_op
        c_deg, d_deg = rig.commit_scene({"tx_id": "tx-m16"})
        rig._run_reload_op = orig_run_reload_op

        assert c_deg == 503 and d_deg["code"] == "SIM_ACTIVATION_DEGRADED"
        assert rig.activation_state == "DEGRADED_SPLIT"
        assert rig.pending_tx_id == "tx-m16"
        assert rig.pending_cam_cleanup_tx_id == "tx-m16"
        assert rig._prev_cam_gen is not None
        st_mid = rig.sim_state()
        assert st_mid["pending_cam_cleanup_tx_id"] == "tx-m16"

        # Retry camera_only -> idempotent physics finalize + completes finalize_cam + clears cleanup state
        c_ok, r_ok = rig.commit_scene({"tx_id": "tx-m16", "phase": "camera_only"})
        assert c_ok == 200 and r_ok["state"] == "COMMITTED"
        assert rig.pending_cam_cleanup_tx_id is None
        assert rig.pending_tx_id is None
        assert rig._prev_cam_gen is None
        assert rig.active_tx_id == "tx-m16"
        assert rig.camera_tx_id == "tx-m16"
        assert rig.activation_state == "COMMITTED"
        assert sim.activation_state == "COMMITTED"
        st_end = rig.sim_state()
        assert "pending_cam_cleanup_tx_id" not in st_end
    finally:
        cleanup()

