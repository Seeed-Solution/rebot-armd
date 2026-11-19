"""Jaw visibility model for holder candidates (DM gripper, real 1080p-crop FOV).

Usage: python visibility.py > visibility_output.txt

For each candidate: lens pose from the v7/v8 cradle transform (tilt about the v7 pivot),
optionally translated (dx along end_link X, dz along Z) or rolled 90 deg about the optical
axis. Reports, per jaw opening (per-finger slide 0 and 42.5 mm):
- tip: jaw tip point (end_link origin) in normalised image coords (u right, v down, +-1 edge)
- tipzone: all finger points with x > -5 mm (the last 5 mm of both fingers): v range, and
  the fraction of those points that are inside the image and not occluded
- cover: fraction of image pixels whose ray hits the gripper (128x72 ray grid, trimesh)
- clr: min distance camera envelope (40x36.5x40 box) -> gripper surface samples, mm
  (only meaningful for translated candidates; tilt-only candidates are validated exactly by
  validate_tilt.py)
Also: v7 numbers with its assumed FOV, and sensitivity of the tip position to lens height
in the housing (+-5 mm) and mounted pitch (+-2 deg).
"""

from __future__ import annotations

import sys
from math import degrees, radians, tan

import numpy as np
import trimesh

from optics import (CAM_DEPTH, CAM_H, CAM_W, HALF_H_DEG, HALF_V_DEG, OPENINGS_MM, V7_ASSUMED_HALF_V,
                    dm_gripper, lens_frame, project)
from generate_recamera_mount import tilted_cradle_location

OPEN_SET = (0.0, OPENINGS_MM[-1])
GRID = (128, 72)


def rot_about(axis, deg):
    return trimesh.transformations.rotation_matrix(radians(deg), axis)[:3, :3]


def camera_pose(tilt, dx=0.0, dz=0.0, lens_up_mm=0.0, pitch_err=0.0):
    lens, fwd, up, right = lens_frame(tilted_cradle_location(tilt))
    lens = lens + up * lens_up_mm + np.array([dx, 0.0, dz])
    if pitch_err:
        R = rot_about(right, -pitch_err)   # positive pitch_err = more nose-down
        fwd, up = R @ fwd, R @ up
    return lens, fwd, up, right


def samples(meshes):
    pts = [np.vstack([m.sample(30000, seed=1) if hasattr(m, "sample") else m.vertices, m.vertices])
           for m in meshes.values()]
    return np.vstack(pts)


def tipzone(meshes):
    pts = np.vstack([np.vstack([m.sample(40000), m.vertices]) for k, m in meshes.items() if k.startswith("finger")])
    return pts[pts[:, 0] > -5.0]


def ray_grid(lens, fwd, up, right, roll90):
    th, tv = tan(radians(HALF_H_DEG)), tan(radians(HALF_V_DEG))
    if roll90:
        th, tv = tv, th
    w, h = GRID
    us = (np.arange(w) + 0.5) / w * 2 - 1
    vs = (np.arange(h) + 0.5) / h * 2 - 1
    U, V = np.meshgrid(us, vs)
    d = fwd[None] + (U.ravel() * th)[:, None] * right[None] - (V.ravel() * tv)[:, None] * up[None]
    d /= np.linalg.norm(d, axis=1)[:, None]
    return np.repeat(lens[None], len(d), 0), d, V.ravel()


def evaluate(name, tilt, dx=0.0, dz=0.0, roll90=False, lens_up_mm=0.0, pitch_err=0.0, full=True):
    lens, fwd, up, right = camera_pose(tilt, dx, dz, lens_up_mm, pitch_err)
    pitch = degrees(np.arcsin(-fwd[2]))
    u0, v0, _ = project(np.zeros((1, 3)), lens, fwd, up, right, roll90)
    half_img_v = HALF_H_DEG if roll90 else HALF_V_DEG
    top_edge = pitch - half_img_v   # deg below the gripper axis (end_link X) of the image top edge
    line = (f"{name:<28} tilt={tilt:4.1f} pitch={pitch:5.1f} lens=({lens[0]:6.1f},{lens[1]:4.1f},{lens[2]:5.1f}) "
            f"tip v={v0[0]:+.3f} u={u0[0]:+.3f} topEdge={top_edge:+5.1f}deg")
    if not full:
        return line, v0[0]
    extra = []
    for op in OPEN_SET:
        g = dm_gripper(op)
        mesh = trimesh.util.concatenate(list(g.values()))
        tz = tipzone(g)
        u, v, z = project(tz, lens, fwd, up, right, roll90)
        inside = (np.abs(u) <= 1) & (np.abs(v) <= 1) & (z > 0)
        # occlusion of tip-zone points (ray lens -> point, any hit closer than point - 0.5 mm)
        vis = 0
        if inside.any():
            p = tz[inside]
            d = p - lens
            dist = np.linalg.norm(d, axis=1)
            d /= dist[:, None]
            locs, idx, _ = mesh.ray.intersects_location(np.repeat(lens[None], len(p), 0), d, multiple_hits=False)
            first = np.full(len(p), np.inf)
            first[idx] = np.linalg.norm(locs - lens, axis=1)
            vis = int((first >= dist - 0.5).sum())
        o, dirs, vrow = ray_grid(lens, fwd, up, right, roll90)
        hit = mesh.ray.intersects_any(o, dirs)
        cover = hit.mean()
        top_hit = vrow[hit].min() if hit.any() else np.nan
        extra.append(f"open{op:g}: tipzone v[{np.nanmin(v):+.2f},{np.nanmax(v):+.2f}] u[{np.nanmin(u):+.2f},{np.nanmax(u):+.2f}] "
                     f"in-image {inside.mean() * 100:5.1f}% visible {vis / len(tz) * 100:5.1f}% "
                     f"cover {cover * 100:4.1f}% highest-gripper-row v={top_hit:+.2f}")
    if dx or dz:
        extra.append(f"camera-envelope clearance to gripper (opening 0) mm: {envelope_clearance(tilt, dx, dz):.1f}")
    return line + "\n    " + "\n    ".join(extra), v0[0]


def envelope_clearance(tilt, dx, dz):
    import build123d as bd
    from optics import MOUNT_P_MM, MOUNT_R
    loc = tilted_cradle_location(tilt)
    box = bd.Box(CAM_W, CAM_DEPTH, CAM_H, align=(bd.Align.MIN,) * 3)
    corners = []
    for sx in (-CAM_W / 2, CAM_W / 2):
        for sy in (3.5, 3.5 + CAM_DEPTH):
            for sz in (4.0, 4.0 + CAM_H):
                p = (loc * bd.Location((sx, sy, sz))).position
                corners.append(MOUNT_R @ np.array([p.X, p.Y, p.Z]) + MOUNT_P_MM + np.array([dx, 0, dz]))
    hull = trimesh.convex.convex_hull(np.array(corners))
    g = dm_gripper(0.0)
    pts = np.vstack([m.sample(60000, seed=2) for m in g.values()])
    sd = trimesh.proximity.signed_distance(hull, pts)   # >0 inside
    return float(-sd.max())


def main():
    print(f"FOV (derived, 1080p crop): half H {HALF_H_DEG:.2f} deg, half V {HALF_V_DEG:.2f} deg")
    print("== v7 prediction vs real FOV (tip = jaw tip point, v = fraction of half-height below centre) ==")
    for t in (0, 20, 25, 30):
        lens, fwd, up, right = camera_pose(t)
        rel = -lens
        ang = degrees(np.arctan2(-(rel @ up), rel @ fwd))
        row = " ".join(f"v@{hf:g}={tan(radians(ang)) / tan(radians(hf)):.3f}" for hf in V7_ASSUMED_HALF_V + (HALF_V_DEG,))
        print(f"  tilt {t:>2}: tip {ang:6.2f} deg below axis; {row}")
    print("== candidates ==")
    cands = [("v5 (tilt 0)", 0, {}), ("v7 tilt30", 30, {})]
    cands += [(f"tilt{t}", t, {}) for t in (34, 36, 38, 40, 42, 45, 50)]
    cands += [(f"roll90 tilt{t}", t, {"roll90": True}) for t in (20, 25, 30)]
    cands += [("tilt30 back20", 30, {"dx": -20.0}), ("tilt30 down10", 30, {"dz": -10.0}),
              ("tilt30 fwd15", 30, {"dx": 15.0}), ("tilt35 back20 down5", 35, {"dx": -20.0, "dz": -5.0})]
    for name, t, kw in cands:
        line, _ = evaluate(name, t, **kw)
        print(line, flush=True)
    print("== sensitivity of tip v (no occlusion/cover) ==")
    for t in (30, 36, 38, 40):
        row = []
        for lu in (-5.0, 0.0, 5.0):
            for pe in (-2.0, 0.0, 2.0):
                _, v = evaluate("", t, lens_up_mm=lu, pitch_err=pe, full=False)
                row.append(f"lens{lu:+g}/pitch{pe:+g}:{v:+.2f}")
        print(f"  tilt {t}: " + " ".join(row))


if __name__ == "__main__":
    sys.exit(main())
