"""Validation for a v7_tilt variant (v5 clip-on holder, pitched down).

Usage: python validate_tilt.py TILT_DEG [TILT_DEG ...] | v5   (v5 = v5 STEP body/top bar, untilted)

Image orientation (confirmed on the real arm with v5): image is upright; gripper
up/down = image vertical (short) axis, left/right = image horizontal (long) axis.

Sections:
A. Exact B-rep checks (OCC booleans, shared with generate_recamera_mount.py):
   single valid solids, clamp interface vs reference and vs v5, camera envelope
   and B3 CAN cover collision, top-bar lift/sweep along its tilted install axis,
   camera insertion path, side/top connector keep-outs, retention vs v5.
B. Gripper clearance (gripper frame, mm, jaw tip at origin, X forward, Z up):
   body, top bar, camera envelope, B3 CAN cover vs pla7_green, cnc7, motor_7 and
   both fingers at jaw openings OPENINGS_MM (joint7, both fingers slide 1:1).
   Gripper surfaces sampled at SAMPLE_SPACING mm (+ all mesh vertices); points
   pre-filtered to the object's bounding box (+PAD_MM); nearest candidates
   refined with exact point-to-triangle distance; point-in-solid test only for
   points inside the object's bounding box.
C. Optics: jaw-tip angle vs the tilted optical axis, image position for half
   vertical FOV 17 and 25 deg, line of sight lens -> tip.
D. FOV: rectangular pyramid +-25 deg vertical (short axis), +-40 deg horizontal
   (long axis), apex at the lens-face centre, 250 mm; exact mesh boolean
   (manifold3d) vs body and top bar tessellations, plus angular margin.

Old version (sample_surface_even at 0.4 mm, 11 openings, point-in-solid on every
gripper sample) ran ~16 M ray-cast point-in-solid tests per angle and did not
finish; see README.txt.
"""

from __future__ import annotations

import sys
import time
import xml.etree.ElementTree as ET
from math import atan2, degrees, radians, tan
from pathlib import Path

import build123d as bd
import numpy as np
import trimesh
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from generate_recamera_mount import (  # noqa: E402
    CAM_DEPTH, CAM_H, V5_BODY_STEP, V5_TOP_STEP, brep_checks, build_parts)
from render_preview import (  # noqa: E402
    MOUNT_P_MM, MOUNT_R, XML, apply_trimesh, compose, mesh_file_map, pose)

PLATE_T, BOTTOM_T = 3.5, 4.0  # camera envelope starts at local y=3.5, z=4.0 (v5)
OPENINGS_MM = [0, 25, 50]
FOV_V, FOV_H = 25.0, 40.0
SAMPLE_SPACING = 0.5  # mm, gripper surface sampling
PAD_MM = 30.0         # bounding-box prefilter margin for clearance
FIXED = ("pla7_green", "cnc7", "motor_7")
FINGERS = ("pla_left", "cnc_left", "pla_right", "cnc_right")
T0 = time.time()


def stamp(msg):
    print(f"  [t={time.time() - T0:6.1f}s] {msg}", flush=True)


def to_gripper(pts):
    return (MOUNT_R @ np.asarray(pts, float).T).T + MOUNT_P_MM


def shape_mesh(shape) -> trimesh.Trimesh:
    verts, faces = shape.tessellate(0.05, 0.2)
    v = np.array([[p.X, p.Y, p.Z] for p in verts])
    m = trimesh.Trimesh(to_gripper(v), np.array(faces), process=True)
    m.merge_vertices()
    return m


def gripper_base():
    """Gripper meshes at opening 0 plus each finger's slide direction."""
    root = ET.parse(XML).getroot()
    files = mesh_file_map(root)
    end = root.find(".//body[@name='gripper_end']")
    base = (np.zeros(3), np.eye(3))
    parts = {}
    for geom in end.findall("./geom"):
        name = geom.attrib.get("mesh")
        if name in FIXED:
            p, r = compose(base, pose(geom))
            parts[name] = (apply_trimesh(trimesh.load_mesh(files[name], process=False), p, r), np.zeros(3))
    for side in ("gripper_left", "gripper_right"):
        body = end.find(f"./body[@name='{side}']")
        bp, br = compose(base, pose(body))
        slide = br @ np.array([0.0, 0.0, 1.0])  # slide joint axis = body z
        for geom in body.findall("./geom"):
            name = geom.attrib.get("mesh")
            if name in FINGERS:
                p, r = compose((bp, br), pose(geom))
                parts[name] = (apply_trimesh(trimesh.load_mesh(files[name], process=False), p, r), slide)
    return parts


def sample(mesh: trimesh.Trimesh, spacing: float) -> np.ndarray:
    n = int(max(2000, mesh.area / spacing ** 2))
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=0)
    return np.vstack([pts, mesh.vertices])


def pair_check(obj: trimesh.Trimesh, obj_tree, pts: np.ndarray):
    lo, hi = obj.bounds
    near = pts[np.all((pts >= lo - PAD_MM) & (pts <= hi + PAD_MM), axis=1)]
    if len(near) == 0:
        return None, 0, 0
    d, _ = obj_tree.query(near)
    cand = near[d <= d.min() + 2.0][:4000]
    _, exact, _ = trimesh.proximity.closest_point(obj, cand)
    clearance = float(min(exact.min(), d.min()))
    inbox = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    inside = int(obj.contains(inbox).sum()) if len(inbox) else 0
    return clearance, inside, len(near)


def lens_frame(loc):
    lens_l = loc * bd.Location((0, PLATE_T + CAM_DEPTH, BOTTOM_T + CAM_H / 2))
    o = np.array(tuple(lens_l.position))

    def local_dir(v):
        return np.array(tuple((loc * bd.Location(v)).position)) - np.array(tuple(loc.position))
    fwd, up, right = (MOUNT_R @ local_dir(v) for v in ((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    return to_gripper(o[None])[0], fwd, up, right


def image_coords(p, lens, fwd, up, right):
    v = p - lens
    f_, u_, r_ = v @ fwd, v @ up, v @ right
    return degrees(atan2(-u_, f_)), degrees(atan2(r_, f_)), degrees(np.arccos(f_ / np.linalg.norm(v)))


def fov_pyramid(lens, fwd, up, right, d0=0.01, d1=250.0):
    verts = []
    for dd in (d0, d1):
        hw, hh = dd * tan(radians(FOV_H)), dd * tan(radians(FOV_V))
        for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            verts.append(lens + fwd * dd + right * sx * hw + up * sz * hh)
    return trimesh.convex.convex_hull(np.array(verts))


def validate(tilt: float, v5_parts: bool = False):
    global T0
    T0 = time.time()
    body, top, interface, camera, can, holes, loc = build_parts(tilt)
    if v5_parts:
        body = bd.import_step(str(V5_BODY_STEP)).solids()[0]
        top = bd.import_step(str(V5_TOP_STEP)).solids()[0]
    print(f"=== TILT_DEG {tilt:g}{' (v5 STEP body + top bar)' if v5_parts else ''} ===", flush=True)

    # ---- A. exact B-rep checks ----
    print("  -- A. B-rep checks (exact OCC booleans) --")
    checks = brep_checks(body, top, interface, camera, can, holes, loc, out=lambda s: print("  " + s))
    failed = [k for k, ok in checks.items() if not ok]
    print(f"  BREP_CHECKS: {'ALL PASS' if not failed else 'FAILED ' + ', '.join(failed)}")
    stamp("B-rep checks done")

    # ---- B. gripper clearance ----
    print(f"  -- B. gripper clearance (openings {OPENINGS_MM} mm, sampling {SAMPLE_SPACING} mm"
          f" + vertices, bbox prefilter {PAD_MM:g} mm, exact refinement) --")
    upper = body & bd.Box(300, 200, 300).located(bd.Location((0, 100 + 0.5, 0)))
    objs = {"body": shape_mesh(body), "top_bar": shape_mesh(top), "camera_envelope": shape_mesh(camera),
            "B3_CAN": shape_mesh(can), "body_above_y0.5": shape_mesh(upper)}
    for k, m in objs.items():
        print(f"  {k}: watertight={m.is_watertight} faces={len(m.faces)}")
    trees = {k: cKDTree(sample(m, 1.0)) for k, m in objs.items()}
    gparts = gripper_base()
    gpts = {n: sample(m, SAMPLE_SPACING) for n, (m, _) in gparts.items()}
    print("  gripper meshes: " + ", ".join(f"{n}:watertight={m.is_watertight},pts={len(gpts[n])}"
                                          for n, (m, _) in gparts.items()))
    worst = {k: [np.inf, 0, ""] for k in objs}
    for opening in OPENINGS_MM:
        for pname, (_, slide) in gparts.items():
            if opening > 0 and pname in FIXED:
                continue  # fixed parts do not move with the jaws
            pts = gpts[pname] + slide * opening
            for k, m in objs.items():
                c, ins, _ = pair_check(m, trees[k], pts)
                w = worst[k]
                if c is not None and c < w[0]:
                    w[0], w[2] = c, f"{pname}@{opening}mm"
                w[1] += ins
                if (c is not None and c < 1.0) or ins:
                    print(f"  contact {k} vs {pname} opening={opening}mm: clearance={c:.3f} inside_pts={ins}")
    for k, (c, ins, where) in worst.items():
        cs = f">{PAD_MM:g}" if np.isinf(c) else f"{c:.3f}"
        print(f"  RESULT {k} vs gripper (openings {OPENINGS_MM}): min_clearance_mm={cs} ({where}) "
              f"gripper_points_inside={ins}")
    stamp("gripper clearance done")

    # ---- C. optics ----
    print("  -- C. optics (pinhole, lens = centre of camera-envelope front face) --")
    lens, fwd, up, right = lens_frame(loc)
    print(f"  lens_centre_gripper_mm: {np.round(lens, 3).tolist()}")
    print(f"  optical_axis_gripper: {np.round(fwd, 5).tolist()}  "
          f"pitch_below_horizontal_deg={degrees(np.arcsin(-fwd[2])):.3f}")
    tip = np.zeros(3)
    vt = tip - lens
    vert, horiz, total = image_coords(tip, lens, fwd, up, right)
    print(f"  tip_from_lens_horizontal_deg_below: {degrees(atan2(-vt[2], np.hypot(vt[0], vt[1]))):.3f}")
    print(f"  tip_angle_to_axis_deg: {total:.3f} (vertical {vert:+.3f} below centre, horizontal {horiz:+.3f})")
    for hf in (17.0, 25.0):
        frac = tan(radians(vert)) / tan(radians(hf))
        print(f"  half_vFOV {hf:g}: tip at {frac:.3f} of half-height below centre -> "
              f"{'INSIDE' if abs(frac) <= 1 else 'OUTSIDE'} image (vertical/short axis)")
    # Front-most vertex (max X) of each finger (pla_* + cnc_*), per opening, in image coordinates.
    for opening in OPENINGS_MM:
        row = []
        for pname in ("left", "right"):
            v = np.vstack([gparts[n][0].vertices + gparts[n][1] * opening
                           for n in (f"pla_{pname}", f"cnc_{pname}")])
            p = v[np.argmax(v[:, 0])]
            fv, fh, _ = image_coords(p, lens, fwd, up, right)
            row.append(f"{pname} x={p[0]:.1f} v={fv:+.2f} h={fh:+.2f} "
                       f"(v/17:{tan(radians(fv)) / tan(radians(17)):.3f} v/25:{tan(radians(fv)) / tan(radians(25)):.3f} "
                       f"h/40:{tan(radians(fh)) / tan(radians(40)):+.3f})")
        print(f"  finger_front_vertex opening={opening}mm: " + "; ".join(row))
    # Line of sight lens -> tip vs gripper (each opening), body and top bar.
    d = vt / np.linalg.norm(vt)
    far = float(np.linalg.norm(vt))
    start = lens + d * 0.05
    blockers = {"body": objs["body"], "top_bar": objs["top_bar"]}
    for opening in OPENINGS_MM:
        blockers[f"gripper@{opening}mm"] = trimesh.util.concatenate(
            [m.copy().apply_translation(s * opening) for m, s in gparts.values()])
    for nm, m in blockers.items():
        hits, _, _ = m.ray.intersects_location([start], [d])
        dist = np.linalg.norm(hits - start, axis=1) if len(hits) else np.array([])
        before = dist[dist < far - 1.0]
        print(f"  line_of_sight_lens_to_tip vs {nm}: "
              + (f"BLOCKED {len(before)} crossings (nearest {before.min():.1f} of {far:.1f} mm)"
                 if len(before) else f"clear ({far:.1f} mm, tip-end 1 mm excluded)"))
    stamp("optics done")

    # ---- D. FOV ----
    print("  -- D. FOV pyramid (exact mesh boolean, manifold3d) --")
    pyr = fov_pyramid(lens, fwd, up, right)
    for nm in ("body", "top_bar"):
        x = trimesh.boolean.intersection([objs[nm], pyr], engine="manifold")
        iv = 0.0 if x.is_empty else float(x.volume)
        print(f"  FOV_pyramid(+-{FOV_V:g} vertical/short axis, +-{FOV_H:g} horizontal/long axis, 250 mm) "
              f"{nm}_intrusion_mm3: {iv:.6f}")
    mp = np.vstack([sample(objs["body"], 0.5), sample(objs["top_bar"], 0.5)])
    rel = mp - lens
    fw = rel @ fwd
    sel = fw > 1e-6
    av = np.degrees(np.arctan2(np.abs(rel[sel] @ up), fw[sel]))
    ah = np.degrees(np.arctan2(np.abs(rel[sel] @ right), fw[sel]))
    margin = np.maximum(av - FOV_V, ah - FOV_H)
    i = int(np.argmin(margin))
    print(f"  FOV_margin_closest_body_or_top_point_deg: {margin[i]:.3f} "
          f"(point v={av[i]:.2f} h={ah[i]:.2f} deg, {fw[sel][i]:.2f} mm ahead of lens plane; "
          f"{int((margin < 0).sum())} of {int(sel.sum())} sampled points inside)")
    stamp("FOV done")


if __name__ == "__main__":
    for a in sys.argv[1:] or ["25"]:
        if a == "v5":
            validate(0.0, v5_parts=True)
        else:
            validate(float(a))
