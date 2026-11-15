#!/usr/bin/env python3
"""Simplify the B601-DM visual meshes for the cockpit 3D view and pack them into one binary file.

    uv run --with trimesh --with meshoptimizer==0.2.30a0 --with rtree --with scipy \
        python scripts/decimate_meshes.py \
        /path/to/reBot-DevArm/Rebot_Arm_description/DM \
        kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/meshes

Reads DM/urdf/ReBot_Arm_DM.urdf (visual meshes and material colours per link). Each part is welded,
simplified by meshoptimizer with an absolute error bound of TOL_MM (no face-count target, so thin
plates keep their faces), its triangles reordered for the vertex cache and its vertices renumbered in
first-use order, then written to arm.bin in the layout below. manifest.json maps the links of
assets/reBot-DevArm_gripper.urdf to their parts. The arm chain and end_link frame of both URDFs are
identical, so the parts need no transform. The script prints, per part, the faces before and after and
the measured deviation (99th percentile and maximum distance between the surfaces, both directions).

arm.bin, little-endian, one block per part at manifest offset (a multiple of 4):
    Uint16 position[verts][3]   q = round((x - min) / (max - min) * 65535) per axis, min/max = part bbox
    zero padding to a multiple of 4
    Uint16 or Uint32 index[faces][3]  (Uint32 when verts > 65535), counter-clockwise = outside
Normals are not stored; the page computes them.
"""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

TOL_MM = 0.5
BIN = "arm.bin"
# DM link name -> link name in reBot-DevArm_gripper.urdf
LINK_MAP = {"finger_left_link": "left_finger", "finger_right_link": "right_finger"}


def simplify(m):
    import meshoptimizer as mo
    import trimesh

    v = np.ascontiguousarray(m.vertices, dtype=np.float32)
    idx = np.ascontiguousarray(m.faces.reshape(-1), dtype=np.uint32)
    out = np.zeros_like(idx)
    n = mo.simplify(out, idx, v, target_index_count=0, target_error=TOL_MM / 1000,
                    options=mo.SIMPLIFY_ERROR_ABSOLUTE)
    tri = np.zeros(n, dtype=np.uint32)
    mo.optimize_vertex_cache(tri, out[:n].copy(), vertex_count=len(v))
    # renumber vertices in first-use order, drop unused ones
    order, first = np.unique(tri, return_index=True)
    used = order[np.argsort(first)]
    remap = np.empty(len(v), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return trimesh.Trimesh(m.vertices[used], remap[tri].reshape(-1, 3), process=False)


def deviation_mm(a, b, n: int = 8000) -> tuple[float, float]:
    import trimesh
    d = []
    for x, y in ((a, b), (b, a)):
        p, _ = trimesh.sample.sample_surface(x, n, seed=0)
        d.append(trimesh.proximity.closest_point(y, p)[1])
    d = np.concatenate(d) * 1000
    return float(np.percentile(d, 99)), float(d.max())


def main(src: Path, dst: Path) -> None:
    import trimesh
    urdf = ET.parse(src / "urdf" / "ReBot_Arm_DM.urdf").getroot()
    colours = {}
    for m in urdf.iter("material"):
        c = m.find("color")
        if c is not None and m.get("name"):
            colours[m.get("name")] = [round(float(v), 4) for v in c.get("rgba").split()[:3]]
    dst.mkdir(parents=True, exist_ok=True)
    links, blob, faces_in_total, faces_out_total = {}, bytearray(), 0, 0
    for link in urdf.findall("link"):
        name = LINK_MAP.get(link.get("name"), link.get("name"))
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            for tag in ("xyz", "rpy"):
                o = vis.find("origin")
                assert o is None or all(float(v) == 0 for v in o.get(tag, "0 0 0").split()), "non-zero visual origin"
            path = (src / "urdf" / mesh.get("filename")).resolve()
            m = trimesh.load(path, force="mesh")  # welds the STL triangle soup
            mat = vis.find("material")
            rgb = colours.get(mat.get("name") if mat is not None else None, [0.5, 0.5, 0.5])
            s = simplify(m)
            lo, hi = s.vertices.min(0), s.vertices.max(0)
            span = np.where(hi > lo, hi - lo, 1.0)
            q = np.round((s.vertices - lo) / span * 65535).astype("<u2")
            itype = "<u2" if len(q) <= 65536 else "<u4"
            offset = len(blob)
            blob += q.tobytes()
            blob += b"\0" * (-len(blob) % 4)
            blob += s.faces.astype(itype).tobytes()
            blob += b"\0" * (-len(blob) % 4)
            p99, dmax = deviation_mm(m, s)
            part = path.name.replace("colored_", "").removesuffix(".stl")
            links.setdefault(name, []).append({
                "part": part, "rgb": rgb, "offset": offset, "verts": len(q), "faces": len(s.faces),
                "index_bytes": 2 if itype == "<u2" else 4,
                "bbox": [round(float(x), 7) for x in (*lo, *hi)],
            })
            faces_in_total += len(m.faces)
            faces_out_total += len(s.faces)
            print(f"{name:14s} {part:34s} {len(m.faces):6d} -> {len(s.faces):5d} faces  deviation p99 {p99:.2f} max {dmax:.2f} mm")
    (dst / BIN).write_bytes(bytes(blob))
    manifest = {
        "source": "Seeed-Projects/reBot-DevArm Rebot_Arm_description/DM (CERN-OHL-W-2.0)",
        "note": f"visual meshes simplified to an absolute error of {TOL_MM} mm by scripts/decimate_meshes.py "
                "(meshoptimizer); layout of the binary file in that script's docstring",
        "format": "rbm1",
        "file": BIN,
        "bytes": len(blob),
        "links": links,
    }
    (dst / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"total {faces_in_total} -> {faces_out_total} faces, {BIN} {len(blob)} bytes")


def decimate_so101(src_dir: Path, dst_dir: Path, grid_size: float = 0.0008) -> None:
    """Decimate SO-101 STL meshes into rbm1 arm.bin and manifest.json without non-numpy deps."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    stl_files = [
        "base_motor_holder_so101_v1.stl",
        "base_so101_v2.stl",
        "motor_holder_so101_base_v1.stl",
        "motor_holder_so101_wrist_v1.stl",
        "moving_jaw_so101_v1.stl",
        "rotation_pitch_so101_v1.stl",
        "sts3215_03a_no_horn_v1.stl",
        "sts3215_03a_v1.stl",
        "under_arm_so101_v1.stl",
        "upper_arm_so101_v1.stl",
        "waveshare_mounting_plate_so101_v2.stl",
        "wrist_roll_follower_so101_v1.stl",
        "wrist_roll_pitch_so101_v2.stl",
    ]

    def load_stl(p: Path) -> np.ndarray:
        with open(p, "rb") as f:
            f.seek(80)
            n = int(np.fromfile(f, dtype="<u4", count=1)[0])
            data = np.fromfile(
                f,
                dtype=np.dtype([("norm", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
                count=n,
            )
        return data["v"].reshape(-1, 3)

    links, blob = {}, bytearray()
    faces_in_total, faces_out_total = 0, 0
    for name in stl_files:
        base = name.removesuffix(".stl")
        pts = load_stl(src_dir / name)
        faces_in_total += len(pts) // 3
        grid = np.round(pts / grid_size).astype(np.int64)
        _, inverse = np.unique(grid, axis=0, return_inverse=True)
        num_u = inverse.max() + 1
        sums = np.zeros((num_u, 3), dtype=np.float64)
        cnt = np.zeros(num_u, dtype=np.int64)
        np.add.at(sums, inverse, pts)
        np.add.at(cnt, inverse, 1)
        uniq_v = (sums / cnt[:, None]).astype(np.float32)
        faces = inverse.reshape(-1, 3)
        valid = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
        faces = faces[valid]
        used, remap = np.unique(faces, return_inverse=True)
        v = uniq_v[used]
        faces = remap.reshape(-1, 3)
        faces_out_total += len(faces)

        lo, hi = v.min(0), v.max(0)
        span = np.where(hi > lo, hi - lo, 1.0)
        q = np.round((v - lo) / span * 65535).astype("<u2")
        itype = "<u2" if len(q) <= 65536 else "<u4"
        offset = len(blob)
        blob += q.tobytes()
        blob += b"\0" * (-len(blob) % 4)
        blob += faces.astype(itype).tobytes()
        blob += b"\0" * (-len(blob) % 4)

        rgb = [0.12, 0.12, 0.14] if "sts3215" in base else [1.0, 0.82, 0.12]
        links[base] = [{
            "part": base,
            "rgb": rgb,
            "offset": offset,
            "verts": len(q),
            "faces": len(faces),
            "index_bytes": 2 if itype == "<u2" else 4,
            "bbox": [round(float(x), 7) for x in (*lo, *hi)],
        }]
        print(f"{base:36s} {len(pts)//3:6d} -> {len(faces):5d} faces  offset {offset}")

    (dst_dir / BIN).write_bytes(bytes(blob))
    manifest = {
        "source": "TheRobotStudio/SO-ARM100 / Seeed Studio SO-101",
        "note": "visual meshes decimated for web cockpit 3D view; layout in scripts/decimate_meshes.py",
        "format": "rbm1",
        "file": BIN,
        "bytes": len(blob),
        "links": links,
    }
    (dst_dir / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"total {faces_in_total} -> {faces_out_total} faces, {BIN} {len(blob)} bytes")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--so101":
        src = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("assets/so101")
        dst = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("assets/so101_meshes")
        decimate_so101(src, dst)
    else:
        main(Path(sys.argv[1]), Path(sys.argv[2]))
