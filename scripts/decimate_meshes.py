#!/usr/bin/env python3
"""Decimate the B601-DM visual meshes for the cockpit 3D view.

    uv run --with trimesh --with fast-simplification python scripts/decimate_meshes.py \
        /path/to/reBot-DevArm/Rebot_Arm_description/DM \
        kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/meshes

Reads DM/urdf/ReBot_Arm_DM.urdf (visual meshes and material colours per link), simplifies each part
to TARGET_RATIO of its faces (at least MIN_FACES), writes binary STL files and manifest.json
mapping the links of assets/reBot-DevArm_gripper.urdf to their parts. The arm chain and end_link
frame of both URDFs are identical, so the parts need no transform.
"""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

TARGET_RATIO = 0.12
MIN_FACES = 300
# DM link name -> link name in reBot-DevArm_gripper.urdf
LINK_MAP = {"finger_left_link": "left_finger", "finger_right_link": "right_finger"}


def main(src: Path, dst: Path) -> None:
    urdf = ET.parse(src / "urdf" / "ReBot_Arm_DM.urdf").getroot()
    colours = {}
    for m in urdf.iter("material"):
        c = m.find("color")
        if c is not None and m.get("name"):
            colours[m.get("name")] = [round(float(v), 4) for v in c.get("rgba").split()[:3]]
    dst.mkdir(parents=True, exist_ok=True)
    links, total_in, total_out = {}, 0, 0
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
            m = trimesh.load(path, force="mesh")
            mat = vis.find("material")
            mname = mat.get("name") if mat is not None else None
            rgb = colours.get(mname, [0.5, 0.5, 0.5])
            faces_in = len(m.faces)
            target = max(MIN_FACES, int(faces_in * TARGET_RATIO))
            if faces_in > target:
                v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32),
                                                    target_reduction=1 - target / faces_in)
                m = trimesh.Trimesh(v, f, process=True)
            out = path.name.replace("colored_", "")
            m.export(dst / out, file_type="stl")
            total_in += path.stat().st_size
            total_out += (dst / out).stat().st_size
            links.setdefault(name, []).append({"file": out, "rgb": rgb, "faces": len(m.faces)})
            print(f"{name:14s} {out:42s} {faces_in:7d} -> {len(m.faces):6d}")
    manifest = {
        "source": "Seeed-Projects/reBot-DevArm Rebot_Arm_description/DM (CERN-OHL-W-2.0)",
        "note": f"visual meshes simplified to {TARGET_RATIO:.0%} of their faces (min {MIN_FACES}) by scripts/decimate_meshes.py",
        "links": links,
    }
    (dst / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"total {total_in / 1e6:.1f} MB -> {total_out / 1e6:.2f} MB")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
