"""Simulated reCamera view: DM gripper (+ holder) rendered from the lens pose with the real FOV.

Usage: MUJOCO_GL=egl python camera_view.py
Writes camera_view_<variant>.png (one panel per jaw opening) and prints per-image numbers:
tip pixel (projected end_link origin), gripper pixel coverage from the MuJoCo segmentation
buffer, and the highest image row the gripper reaches.

Rendering: MuJoCo offscreen (EGL), pinhole, fovy = vertical FOV, 1280x720 (landscape) or
720x1280 (camera rolled 90 deg about the optical axis). No lens distortion (Seeed: < 1 %).
Overlay: red cross = jaw tip (end_link origin) projected with optics.project(); dashed
lines = 80 % of the half-height (v = +-0.8); green box = image centre.
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import trimesh

from generate_recamera_mount import tilted_cradle_location
from optics import HALF_H_DEG, HALF_V_DEG, MOUNT_P_MM, MOUNT_R, OPENINGS_MM, dm_gripper, lens_frame, project

HERE = Path(__file__).resolve().parent
V7 = HERE.parent / "v7_tilt"

# name, tilt, roll90, holder STL prefix (mount frame) or None
VARIANTS = [
    ("v7_tilt30_realFOV", 30.0, False, V7 / "reCamera_B601_Mount_v7_tilt30"),
    ("v8_tilt38", 38.0, False, HERE / "reCamera_B601_Mount_v8_tilt38"),
    ("v8_tilt42", 42.0, False, HERE / "reCamera_B601_Mount_v8_tilt42"),
    ("roll90_tilt30", 30.0, True, V7 / "reCamera_B601_Mount_v7_tilt30"),
]
OPEN_SET = (0.0, 20.0, OPENINGS_MM[-1])   # per-finger slide; span 0 / 40 / 85 mm


def mjcf(tmp: Path, meshes: dict[str, trimesh.Trimesh], lens, fwd, up, right, roll90):
    w, h = (720, 1280) if roll90 else (1280, 720)
    fovy = 2 * (HALF_H_DEG if roll90 else HALF_V_DEG)
    assets, geoms = [], []
    colors = {"gripper_base": ".24 .28 .27 1", "link6": ".75 .77 .75 1", "finger_left": ".85 .55 .10 1",
              "finger_right": ".85 .55 .10 1", "holder_body": ".10 .45 .82 1", "holder_top": ".66 .33 .97 1"}
    for name, m in meshes.items():
        f = tmp / f"{name}.stl"
        mm = m.copy(); mm.apply_scale(0.001); mm.export(f)
        assets.append(f'<mesh name="{name}" file="{f}"/>')
        geoms.append(f'<geom name="{name}" type="mesh" mesh="{name}" rgba="{colors[name]}"/>')
    xy = " ".join(f"{c:.6f}" for c in np.r_[right, up])
    pos = " ".join(f"{c:.6f}" for c in lens / 1000.0)
    # a floor 150 mm below the jaw axis would only clutter; plain background instead.
    xml = f"""<mujoco><compiler meshdir="/"/>
<visual><global offwidth="{w}" offheight="{h}"/><headlight ambient=".45 .45 .45" diffuse=".6 .6 .6"/>
<map znear="0.001"/></visual>
<asset><texture type="skybox" builtin="flat" rgb1=".86 .88 .91" rgb2=".86 .88 .91" width="16" height="16"/>{''.join(assets)}</asset>
<worldbody><light pos="{lens[0] / 1000:.4f} 0 0.4" dir="0 0 -1"/>{''.join(geoms)}
<camera name="cam" pos="{pos}" xyaxes="{xy}" fovy="{fovy:.4f}"/></worldbody></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml), w, h


def render(variant):
    name, tilt, roll90, holder = variant
    lens, fwd, up, right = lens_frame(tilted_cradle_location(tilt))
    panels = []
    for op in OPEN_SET:
        meshes = dm_gripper(op)
        if holder is not None:
            for part, key in (("body", "holder_body"), ("top_bar", "holder_top")):
                p = Path(f"{holder}_{part}.stl")
                if p.exists():
                    m = trimesh.load_mesh(p, process=False)
                    m.vertices = (MOUNT_R @ m.vertices.T).T + MOUNT_P_MM
                    meshes[key] = m
        with tempfile.TemporaryDirectory() as td:
            model, w, h = mjcf(Path(td), meshes, lens, fwd, up, right, roll90)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            r = mujoco.Renderer(model, h, w)
            r.update_scene(data, camera="cam")
            img = r.render().copy()
            r.close()
            rs = mujoco.Renderer(model, h, w)   # separate renderer for the segmentation pass
            rs.enable_segmentation_rendering()
            rs.update_scene(data, camera="cam")
            seg = rs.render()[:, :, 0].copy()
            rs.close()
        names = [model.geom(i).name for i in range(model.ngeom)]
        grip_ids = [i for i, n in enumerate(names) if n in ("gripper_base", "link6", "finger_left", "finger_right")]
        hold_ids = [i for i, n in enumerate(names) if n.startswith("holder")]
        gmask = np.isin(seg, grip_ids)
        hmask = np.isin(seg, hold_ids)
        u, v, _ = project(np.zeros((1, 3)), lens, fwd, up, right, roll90)
        px, py = (u[0] + 1) / 2 * w, (v[0] + 1) / 2 * h
        rows = np.where(gmask.any(axis=1))[0]
        top_row = rows.min() if len(rows) else None
        top_v = (top_row + 0.5) / h * 2 - 1 if top_row is not None else float("nan")
        print(f"{name} opening {op:g}: tip pixel ({px:.0f},{py:.0f}) of {w}x{h} -> v={v[0]:+.3f} u={u[0]:+.3f}; "
              f"gripper cover {gmask.mean() * 100:.1f}%, holder cover {hmask.mean() * 100:.2f}%, "
              f"highest gripper row {top_row} (v={top_v:+.2f})", flush=True)
        panels.append((op, img, px, py, w, h, gmask.mean(), v[0]))
    return name, tilt, roll90, panels


def figure(result):
    name, tilt, roll90, panels = result
    fig, axes = plt.subplots(1, len(panels), figsize=(20 if not roll90 else 12, 5 if not roll90 else 8.5), dpi=130)
    for ax, (op, img, px, py, w, h, cover, v) in zip(axes, panels):
        ax.imshow(img)
        ax.plot([px], [py], "r+", ms=22, mew=2.5)
        for vv in (-0.8, 0.8):
            y = (vv + 1) / 2 * h
            ax.axhline(y, color="orange", ls="--", lw=1)
        ax.add_patch(plt.Rectangle((w / 2 - 6, h / 2 - 6), 12, 12, fill=False, ec="lime", lw=1.5))
        ax.set_xlim(0, w); ax.set_ylim(h, 0)
        ax.set_title(f"jaw opening {2 * op:g} mm: tip v={v:+.2f}, gripper {cover * 100:.0f}% of image", fontsize=10)
        ax.set_xticks([0, w // 2, w]); ax.set_yticks([0, h // 2, h])
    orient = "rolled 90 deg (portrait, long axis = gripper up/down)" if roll90 else "landscape (as v5)"
    fig.suptitle(f"Simulated reCamera view {name}: pitch {tilt:g} deg, {orient}; "
                 f"FOV {2 * HALF_H_DEG:.1f} x {2 * HALF_V_DEG:.1f} deg (OV5647 1080p crop, f 3.46 mm, derived)\n"
                 "red + = jaw tip; orange dashed = 80 % of half-height; DM gripper meshes; lens at front-face centre (assumed)",
                 fontsize=9)
    fig.tight_layout()
    out = HERE / f"camera_view_{name}.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"image: {out}")


if __name__ == "__main__":
    sel = sys.argv[1:]
    for var in VARIANTS:
        if sel and var[0] not in sel:
            continue
        figure(render(var))
