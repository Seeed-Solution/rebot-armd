"""Validation for v8 variants (v7_tilt checks, DM gripper, real FOV).

Usage: python validate_v8.py TILT_DEG [TILT_DEG ...]

Derived from v7_tilt/validate_tilt.py. Changes:
- Gripper = DM meshes in end_link (optics.dm_gripper): gripper_base, link6 (motor housing),
  finger_left / finger_right; per-finger openings 0, 25, 42.5 mm (armd URDF limit 42.5).
- FOV = OV5647 1080p crop with f 3.46 mm: half V 12.33 deg, half H 21.23 deg (derived, see
  optics.py), not the v7 assumptions (17/25 V, 40 H).
- Optics section adds: tip-zone (finger points x > -5 mm) image extent per opening, the
  largest per-finger opening with both tip zones inside 80 % of the half-width, and the
  armd camera_mount values for this variant.

Sections: A. exact B-rep checks (shared generator code); B. gripper clearance (same method as
v7: 0.5 mm surface sampling + vertices, bbox prefilter, exact refinement); C. optics;
D. FOV pyramid intrusion (manifold3d boolean) + angular margin.
"""

from __future__ import annotations

import sys
import time
from math import degrees

import build123d as bd
import numpy as np
import trimesh
from scipy.spatial import cKDTree

from generate_recamera_mount import brep_checks, build_parts
from optics import (HALF_H_DEG, HALF_V_DEG, MOUNT_P_MM, MOUNT_R, dm_gripper, finger_slides, lens_frame,
                    project, to_end)

OPENINGS_MM = [0.0, 25.0, 42.5]
SAMPLE_SPACING = 0.5
PAD_MM = 30.0
FIXED = ("gripper_base", "link6")
T0 = time.time()


def stamp(msg):
    print(f"  [t={time.time() - T0:6.1f}s] {msg}", flush=True)


def shape_mesh(shape) -> trimesh.Trimesh:
    verts, faces = shape.tessellate(0.05, 0.2)
    v = np.array([[p.X, p.Y, p.Z] for p in verts])
    m = trimesh.Trimesh(to_end(v), np.array(faces), process=True)
    m.merge_vertices()
    return m


def sample(mesh, spacing):
    n = int(max(2000, mesh.area / spacing ** 2))
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=0)
    return np.vstack([pts, mesh.vertices])


def pair_check(obj, obj_tree, pts):
    lo, hi = obj.bounds
    near = pts[np.all((pts >= lo - PAD_MM) & (pts <= hi + PAD_MM), axis=1)]
    if len(near) == 0:
        return None, 0
    d, _ = obj_tree.query(near)
    cand = near[d <= d.min() + 2.0][:4000]
    _, exact, _ = trimesh.proximity.closest_point(obj, cand)
    inbox = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    inside = int(obj.contains(inbox).sum()) if len(inbox) else 0
    return float(min(exact.min(), d.min())), inside


def camera_mount(lens, fwd, up, right):
    """armd camera_mount for this pose (src/kinematics.rs cam_mount, OpenCV camera frame).
    armd default (flip false, rot 0): z_cam = end_link +X, y_cam = (0,-1,0), x_cam = (0,0,1)
    (J5 axis in end_link at q6 = pi/2 is (0,-1,0) for the DM chain; computed in README 1.5).
    Here the holder gives x_cam = right, y_cam = -up, z_cam = fwd."""
    x0, y0, z0 = np.array([0, 0, 1.0]), np.array([0, -1.0, 0]), np.array([1.0, 0, 0])
    xc, yc = right, -up
    rot = degrees(np.arctan2(xc @ y0, xc @ x0))          # x' = cos x0 + sin y0
    zc = fwd
    # armd: R = [x0 y0 z0] Rz(rot) Rx(tilt) -> x_cam = cos(rot) x0 + sin(rot) y0;
    # z_cam = cos(t) z0 - sin(t) y1 with y1 = z0 x x_cam.
    assert np.allclose(np.cross(xc, yc), fwd, atol=1e-6), "camera frame not right-handed"
    y_flat = np.cross(z0, xc)
    tilt = degrees(np.arctan2(-(zc @ y_flat), zc @ z0))
    return rot, tilt, lens / 1000.0


def validate(tilt):
    global T0
    T0 = time.time()
    body, top, interface, camera, can, holes, loc = build_parts(tilt)
    print(f"=== v8 TILT_DEG {tilt:g} ===", flush=True)
    print("  -- A. B-rep checks (exact OCC booleans) --")
    checks = brep_checks(body, top, interface, camera, can, holes, loc, out=lambda s: print("  " + s))
    failed = [k for k, ok in checks.items() if not ok]
    print(f"  BREP_CHECKS: {'ALL PASS' if not failed else 'FAILED ' + ', '.join(failed)}")
    stamp("B-rep checks done")

    print(f"  -- B. DM gripper clearance (per-finger openings {OPENINGS_MM} mm, sampling {SAMPLE_SPACING} mm"
          f" + vertices, bbox prefilter {PAD_MM:g} mm, exact refinement) --")
    upper = body & bd.Box(300, 200, 300).located(bd.Location((0, 100 + 0.5, 0)))
    objs = {"body": shape_mesh(body), "top_bar": shape_mesh(top), "camera_envelope": shape_mesh(camera),
            "B3_CAN": shape_mesh(can), "body_above_y0.5": shape_mesh(upper)}
    trees = {k: cKDTree(sample(m, 1.0)) for k, m in objs.items()}
    g0 = dm_gripper(0.0)
    slides = finger_slides()
    gpts = {n: sample(m, SAMPLE_SPACING) for n, m in g0.items()}
    print("  gripper meshes: " + ", ".join(f"{n}:watertight={m.is_watertight},pts={len(gpts[n])}" for n, m in g0.items()))
    worst = {k: [np.inf, 0, ""] for k in objs}
    for opening in OPENINGS_MM:
        for pname in g0:
            if opening > 0 and pname in FIXED:
                continue
            pts = gpts[pname] + slides.get(pname, np.zeros(3)) * opening
            for k, m in objs.items():
                c, ins = pair_check(m, trees[k], pts)
                w = worst[k]
                if c is not None and c < w[0]:
                    w[0], w[2] = c, f"{pname}@{opening:g}mm"
                w[1] += ins
    for k, (c, ins, where) in worst.items():
        cs = f">{PAD_MM:g}" if np.isinf(c) else f"{c:.3f}"
        print(f"  RESULT {k} vs DM gripper: min_clearance_mm={cs} ({where}) gripper_points_inside={ins}")
    stamp("gripper clearance done")

    print(f"  -- C. optics (pinhole, half V {HALF_V_DEG:.2f} deg, half H {HALF_H_DEG:.2f} deg; "
          "lens = centre of camera-envelope front face, ASSUMED) --")
    lens, fwd, up, right = lens_frame(loc)
    print(f"  lens_centre_end_link_mm: {np.round(lens, 2).tolist()}  optical_axis: {np.round(fwd, 4).tolist()}  "
          f"pitch_below_end_X_deg={degrees(np.arcsin(-fwd[2])):.2f}")
    print(f"  image top edge {degrees(np.arcsin(-fwd[2])) - HALF_V_DEG:.2f} deg below end_link X, "
          f"bottom edge {degrees(np.arcsin(-fwd[2])) + HALF_V_DEG:.2f} deg")
    u, v, z = project(np.zeros((1, 3)), lens, fwd, up, right)
    print(f"  jaw_tip (end_link origin): v={v[0]:+.3f} (fraction of half-height, + = below centre), "
          f"u={u[0]:+.3f}, distance {np.linalg.norm(lens):.1f} mm  -> {'INSIDE 80%' if abs(v[0]) <= 0.8 else 'OUTSIDE 80%'}")
    print(f"  grip_pixel_norm estimate (jaw tip): [{(u[0] + 1) / 2:.3f}, {(v[0] + 1) / 2:.3f}]")
    for op in OPENINGS_MM:
        g = dm_gripper(op)
        rows = []
        for f in ("finger_left", "finger_right"):
            m = g[f]
            p = np.vstack([m.sample(40000, seed=3), m.vertices])
            p = p[p[:, 0] > -5.0]
            fu, fv, _ = project(p, lens, fwd, up, right)
            rows.append(f"{f} u[{fu.min():+.2f},{fu.max():+.2f}] v[{fv.min():+.2f},{fv.max():+.2f}]")
        print(f"  tip_zone (x>-5mm) opening {op:g}: " + "; ".join(rows))
    ok_open = 0.0
    for op in np.arange(0, 42.6, 0.5):
        g = dm_gripper(op)
        p = np.vstack([np.vstack([g[f].vertices, g[f].sample(20000, seed=4)]) for f in ("finger_left", "finger_right")])
        p = p[p[:, 0] > -5.0]
        fu, fv, _ = project(p, lens, fwd, up, right)
        if np.abs(fu).max() <= 0.8 and np.abs(fv).max() <= 1.0:
            ok_open = op
    print(f"  largest per-finger opening with both tip zones inside 80% half-width: {ok_open:.1f} mm "
          f"(jaw span {2 * ok_open:.0f} mm)")
    d = -lens / np.linalg.norm(lens)
    far = float(np.linalg.norm(lens))
    start = lens + d * 0.05
    blockers = {"body": objs["body"], "top_bar": objs["top_bar"]}
    for op in OPENINGS_MM:
        blockers[f"gripper@{op:g}mm"] = trimesh.util.concatenate(list(dm_gripper(op).values()))
    for nm, m in blockers.items():
        hits, _, _ = m.ray.intersects_location([start], [d])
        dist = np.linalg.norm(hits - start, axis=1) if len(hits) else np.array([])
        before = dist[dist < far - 1.0]
        print(f"  line_of_sight_lens_to_tip vs {nm}: "
              + (f"BLOCKED {len(before)} crossings (nearest {before.min():.1f} of {far:.1f} mm)"
                 if len(before) else f"clear ({far:.1f} mm, tip-end 1 mm excluded)"))
    rot, tl, off = camera_mount(lens, fwd, up, right)
    print(f"  armd camera_mount (derived, see README 1.5): flip=false rot_deg={rot:.1f} tilt_deg={tl:.1f} "
          f"offset_m=[{off[0]:.4f}, {off[1]:.4f}, {off[2]:.4f}]")
    stamp("optics done")

    print("  -- D. FOV pyramid (real FOV, exact mesh boolean, manifold3d) --")
    verts = []
    th, tv = np.tan(np.radians(HALF_H_DEG)), np.tan(np.radians(HALF_V_DEG))
    for dd in (0.01, 250.0):
        for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            verts.append(lens + fwd * dd + right * sx * dd * th + up * sz * dd * tv)
    pyr = trimesh.convex.convex_hull(np.array(verts))
    for nm in ("body", "top_bar"):
        x = trimesh.boolean.intersection([objs[nm], pyr], engine="manifold")
        iv = 0.0 if x.is_empty else float(x.volume)
        print(f"  FOV_pyramid(+-{HALF_V_DEG:.2f} V, +-{HALF_H_DEG:.2f} H, 250 mm) {nm}_intrusion_mm3: {iv:.6f}")
    mp = np.vstack([sample(objs["body"], 0.5), sample(objs["top_bar"], 0.5)])
    rel = mp - lens
    fw = rel @ fwd
    sel = fw > 1e-6
    av = np.degrees(np.arctan2(np.abs(rel[sel] @ up), fw[sel]))
    ah = np.degrees(np.arctan2(np.abs(rel[sel] @ right), fw[sel]))
    margin = np.maximum(av - HALF_V_DEG, ah - HALF_H_DEG)
    i = int(np.argmin(margin))
    print(f"  FOV_margin_closest_body_or_top_point_deg: {margin[i]:.3f} ({int((margin < 0).sum())} of "
          f"{int(sel.sum())} sampled points inside)")
    stamp("FOV done")


if __name__ == "__main__":
    for a in sys.argv[1:] or ["38"]:
        validate(float(a))
