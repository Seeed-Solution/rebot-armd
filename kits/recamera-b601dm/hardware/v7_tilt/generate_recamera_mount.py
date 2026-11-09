"""v7_tilt reCamera B601 holder: v5 clip-on holder with the camera pitched down.

Derived from v5/generate_recamera_mount.py. Changes vs v5:
- TILT_DEG: every cradle feature (rear stop, rails, side guards/ears, bottom
  support, front lip, camera cavity, bottom port, ear bores, nut pockets), the
  removable top bar, the camera envelope and the B3 CAN cover are placed in a
  tilted cradle frame = v5 cradle frame rotated by TILT_DEG about the cradle's
  lateral axis (local x) through PIVOT. The optical axis (+y local) goes forward
  and down. Retention geometry relative to the camera is therefore unchanged.
- PIVOT: front-bottom edge of the v5 rails/bottom support (local y = 43,
  z = 0.7). The rear rises; no tilted part goes below the v5 support level.
- Support wedge: 2D convex hull (cradle side profile) of the v5 support band
  (y 0..43, z 0.7..3.7) and the same band tilted, extruded over the rear-stop
  width (46.6 mm). It keeps the v5 footprint, so the v5 bridge and the y <= 0
  clamp interface connect as in v5.
- Bottom port (under the CAN rear cover): the v5 cut (tilted z -16..4) is kept;
  where the wedge is thicker than that, the port is also cut through the wedge
  only (bridge and interface are not cut further), so the port stays open.
- The clamp interface (mount y <= 0) is replaced by the reference interface
  exactly, as in v5.

Usage: python generate_recamera_mount.py [TILT_DEG]   (default 25)
"""

import sys
from math import cos, radians, sin
from pathlib import Path

import build123d as bd
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
REFERENCE_STEP = ROOT / "b601-camera-mounts" / "D435_Gemini2_Mount.step"
CAN_STEP = Path(__file__).resolve().parent.parent / "reference" / "B3_CAN.step"
V5_DIR = Path(__file__).resolve().parent.parent / "v5"
V5_BODY_STEP = V5_DIR / "reCamera_B601_Mount_v5_body.step"
V5_TOP_STEP = V5_DIR / "reCamera_B601_Mount_v5_top_bar.step"
OUT_DIR = Path(__file__).resolve().parent

TILT_DEG = 25.0

CAM_W, CAM_H, CAM_DEPTH = 40.0, 40.0, 36.5
CLEARANCE = 0.3
INNER_W = CAM_W + 2 * CLEARANCE
INNER_H = CAM_H + 2 * CLEARANCE
WALL = 3.0
EAR_T = 7.0
OUTER_W = INNER_W + 2 * WALL
SUPPORT_Z0, SUPPORT_Z1 = 0.7, 3.7
BODY_Z0 = 4.0 - CLEARANCE
BODY_Z1 = 44.0 + CLEARANCE
GUARD_Y0, FRONT_Y1 = 17.0, 43.0
TOP_Z0, TOP_T = BODY_Z1, 3.0
INTERFACE_Y = 0.0
CRADLE_ORIGIN = (0.0, 0.0, -8.0)
EAR_X = INNER_W / 2 + EAR_T / 2
PIVOT = (0.0, FRONT_Y1, SUPPORT_Z0)


def out_paths(tilt_deg: float):
    tag = f"v7_tilt{tilt_deg:g}"
    return {k: OUT_DIR / f"reCamera_B601_Mount_{tag}_{k}" for k in
            ("body.step", "body.stl", "top_bar.step", "top_bar.stl")}


def cradle_location():
    R = Rotation.from_quat([.507205266179, .498692788059,
                             .491587190903, .502384668195]).as_matrix()
    target = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    local = R.T @ target
    return bd.Plane(origin=CRADLE_ORIGIN, x_dir=tuple(local[:, 0]),
                    z_dir=tuple(local[:, 2])).location


def tilt_location(tilt_deg: float) -> bd.Location:
    """Rotate about local x through PIVOT; -tilt maps +y to (0, cos, -sin)."""
    return (bd.Location(PIVOT) * bd.Location((0, 0, 0), (-tilt_deg, 0, 0))
            * bd.Location(tuple(-c for c in PIVOT)))


def tilted_cradle_location(tilt_deg: float) -> bd.Location:
    return cradle_location() * tilt_location(tilt_deg)


def rot_yz(y, z, tilt_deg):
    t = radians(tilt_deg)
    dy, dz = y - PIVOT[1], z - PIVOT[2]
    return PIVOT[1] + dy * cos(t) + dz * sin(t), PIVOT[2] - dy * sin(t) + dz * cos(t)


def support_wedge(tilt_deg: float) -> bd.Solid:
    rect = [(0.0, SUPPORT_Z0), (FRONT_Y1, SUPPORT_Z0), (FRONT_Y1, SUPPORT_Z1), (0.0, SUPPORT_Z1)]
    pts = np.array(rect + [rot_yz(y, z, tilt_deg) for y, z in rect])
    hull = pts[ConvexHull(pts).vertices]
    face = bd.Plane.YZ * bd.Polygon(*[tuple(p) for p in hull], align=None)
    return bd.extrude(face, amount=OUTER_W / 2, both=True)


def vol(a, b):
    x = a & b
    return 0.0 if x is None else x.volume


def cover_location(loc):
    # v5 roll: original -X connector side faces upward; +/-Y sides left/right.
    return loc * bd.Plane(origin=(0, 21.8, 24), x_dir=(0, 0, -1), z_dir=(0, 1, 0)).location


def build_parts(tilt_deg: float = TILT_DEG):
    ref = bd.import_step(str(REFERENCE_STEP)).solids()[0]
    interface = ref & bd.Box(100, 30, 100).located(bd.Location((0, -15, 0)))
    loc0 = cradle_location()
    loc = loc0 * tilt_location(tilt_deg)

    bridge = bd.Box(39.0, 5.0, 14.0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        bd.Location((0, INTERFACE_Y, 7.0)))
    back_stop = bd.Box(INNER_W + 2 * WALL, 3.5, BODY_Z1 - SUPPORT_Z0,
                       align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                           loc * bd.Location((0, 0, SUPPORT_Z0)))
    rails = [bd.Box(3, 43, 3, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((x, 0, 0.7))) for x in (-21.8, 21.8)]
    side_l = bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, BODY_Z1 - BODY_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((-EAR_X, GUARD_Y0, BODY_Z0)))
    side_r = bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, BODY_Z1 - BODY_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((EAR_X, GUARD_Y0, BODY_Z0)))
    bottom = bd.Box(INNER_W, FRONT_Y1 - GUARD_Y0, SUPPORT_Z1 - SUPPORT_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((0, GUARD_Y0, SUPPORT_Z0)))
    lip = bd.Box(INNER_W, FRONT_Y1 - CAM_DEPTH - 3.5, 1.5,
                 align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                     loc * bd.Location((0, CAM_DEPTH + 3.5, SUPPORT_Z1)))
    wedge = support_wedge(tilt_deg).located(loc0)

    body = (interface + bridge + wedge + back_stop + rails[0] + rails[1]
            + side_l + side_r + bottom + lip)
    cavity = bd.Box(INNER_W, 36.5, 40.6, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((0, 3.5, 3.7)))
    bottom_port = bd.Box(20, 13.5, 20, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((0, 3.5, -16)))
    port_through_wedge = wedge & bd.Box(20, 13.5, 80, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((0, 3.5, -76)))
    body = body - cavity - bottom_port
    if port_through_wedge is not None and port_through_wedge.volume > 0:
        body = body - port_through_wedge
    rear_region = bd.Box(200, 100, 200).located(bd.Location((0, -50, 0)))
    body = (body - rear_region) + interface
    for x in (-EAR_X, EAR_X):
        bore = bd.Cylinder(1.6, 12, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).located(
            loc * bd.Location((x, 35, 37)))
        nut = bd.Box(9, 5.8, 2.6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).located(
            loc * bd.Location((x, 35, 38)))
        body = body - bore - nut

    body_solids = body.solids()
    if len(body_solids) != 1 or not body_solids[0].is_valid:
        raise RuntimeError(f"body expected one valid solid, got {len(body_solids)}")

    top = bd.Box(INNER_W, FRONT_Y1 - GUARD_Y0, TOP_T,
                 align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                     loc * bd.Location((0, GUARD_Y0, TOP_Z0)))
    for x in (-EAR_X, EAR_X):
        top += bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, TOP_T,
                      align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                          loc * bd.Location((x, GUARD_Y0, TOP_Z0)))
    hole_tools = []
    for x in (-EAR_X, EAR_X):
        tool = bd.Cylinder(1.6, TOP_T * 3,
                           align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.CENTER)).located(
                               loc * bd.Location((x, 35.0, TOP_Z0 + TOP_T / 2)))
        top = top - tool
        hole_tools.append(tool)
    top_solids = top.solids()
    if len(top_solids) != 1 or not top_solids[0].is_valid:
        raise RuntimeError(f"top bar expected one valid solid, got {len(top_solids)}")

    camera = bd.Box(CAM_W, CAM_DEPTH, CAM_H,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((0, 3.5, 4.0)))
    can = bd.import_step(str(CAN_STEP)).solids()[0]
    can_aligned = can.located(cover_location(loc))
    return body_solids[0], top_solids[0], interface, camera, can_aligned, hole_tools, loc


def local_vec(loc, v):
    return (loc * bd.Location(v)).position - loc.position


def brep_checks(body, top, interface, camera, can, top_holes, loc, out=print):
    """Exact B-rep checks (OCC booleans). Returns {check_name: passed}."""
    original = bd.import_step(str(REFERENCE_STEP)).solids()[0]
    preserved = bd.Box(200, 100, 200).located(bd.Location((0, -50, 0)))
    added = (body & preserved) - original
    added_v = 0.0 if added is None else added.volume
    lost_v = vol(interface, original - body)

    v5_body = bd.import_step(str(V5_BODY_STEP)).solids()[0]
    v5_iface, new_iface = v5_body & preserved, body & preserved
    vs_v5_added = (new_iface - v5_iface).volume
    vs_v5_lost = (v5_iface - new_iface).volume

    body_camera, top_camera = vol(body, camera), vol(top, camera)
    body_can, top_can = vol(body, can), vol(top, can)
    top_body = vol(top, body)
    top_hole_collision = sum(vol(h, camera) for h in top_holes)

    # Top-bar removal along its own tilted install axis (local +z): lifted copies
    # and a continuous swept prism of the bar footprint, 0..60 mm.
    up = local_vec(loc, (0, 0, 1))
    lift_collisions = [vol(top.moved(bd.Location(up * d)), body) + vol(top.moved(bd.Location(up * d)), camera)
                       for d in (0.5, 2.0, 5.0, 10.0, 20.0)]
    bar_sweep = bd.Box(INNER_W + 2 * EAR_T, FRONT_Y1 - GUARD_Y0, 60.0,
                       align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                           loc * bd.Location((0, GUARD_Y0, TOP_Z0 + TOP_T)))
    bar_sweep_body = vol(bar_sweep, body)
    bar_sweep_can = vol(bar_sweep, can)

    insertion_collisions = []
    for offset in (0.0, 5.0, 17.0, 40.0):
        ic = bd.Box(CAM_W, CAM_DEPTH, CAM_H, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
            loc * bd.Location((0, 3.5, 4.0 + offset)))
        insertion_collisions.append(vol(body, ic))
    sweep = bd.Box(40, 36.5, 90, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((0, 3.5, 4)))
    camera_sweep_body = vol(body, sweep)

    # Retention features relative to the camera: transform the body back by the
    # tilt and compare the cradle region (local z >= 0.7 band above the wedge) with v5.

    loc0 = cradle_location()
    cradle_box_local = bd.Box(70, 43, 60, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        bd.Location((0, 0, SUPPORT_Z1)))
    def cradle_part(solid, l):
        return solid & cradle_box_local.located(l * bd.Location((0, 0, SUPPORT_Z1)))
    new_cradle = cradle_part(body, loc)
    old_cradle = cradle_part(v5_body, loc0)
    # Move the new cradle region into the untilted frame for a direct diff.
    new_back = new_cradle.moved(loc0 * loc.inverse())
    cradle_diff_add = (new_back - old_cradle).volume
    cradle_diff_lost = (old_cradle - new_back).volume
    v5_top = bd.import_step(str(V5_TOP_STEP)).solids()[0]
    top_back = top.moved(loc0 * loc.inverse())
    top_diff = (top_back - v5_top).volume + (v5_top - top_back).volume

    keepouts = {}
    for name, pos, size in (
        ("left", (-40.3, 3.6, 4.1), (20, 13.3, 39.8)),
        ("right", (20.3, 3.6, 4.1), (20, 13.3, 39.8)),
        ("upper", (-19.9, 3.6, 44.1), (39.8, 13.3, 20)),
    ):
        keepout = bd.Box(*size, align=(bd.Align.MIN,) * 3).located(loc * bd.Location(pos))
        keepouts[name] = vol(body, keepout) + vol(top, keepout)

    checks = {
        "single_valid_solids": (len(body.solids()) == 1 and body.is_valid
                                and len(top.solids()) == 1 and top.is_valid),
        "added": abs(added_v) < 1e-6, "lost": abs(lost_v) < 1e-6,
        "vs_v5": abs(vs_v5_added) < 1e-6 and abs(vs_v5_lost) < 1e-6,
        "camera": body_camera < 1e-6 and top_camera < 1e-6,
        "can": body_can < 1e-6 and top_can < 1e-6,
        "top_body": top_body < 1e-6, "holes": top_hole_collision < 1e-6,
        "lift": max(lift_collisions) < 1e-6, "bar_sweep": bar_sweep_body < 1e-6 and bar_sweep_can < 1e-6,
        "insertion": max(insertion_collisions) < 1e-6 and camera_sweep_body < 1e-6,
        "keepouts": all(v < 1e-6 for v in keepouts.values()),
        "retention_vs_v5": cradle_diff_add < 1e-3 and cradle_diff_lost < 1e-3 and top_diff < 1e-3,
    }

    port_col = bd.Box(20, 13.5, 80, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        loc * bd.Location((0, 3.5, -76.3)))
    port_material = vol(body, port_col)
    out(f"body_solid_count: {len(body.solids())}  body_valid: {body.is_valid}")
    out(f"top_solid_count: {len(top.solids())}  top_valid: {top.is_valid}")
    out(f"interface_y_le_0_added_vs_reference_mm3: {added_v:.6f}")
    out(f"interface_y_le_0_lost_vs_reference_mm3: {lost_v:.6f}")
    out(f"interface_y_le_0_added_vs_v5_body_mm3: {vs_v5_added:.6f}")
    out(f"interface_y_le_0_lost_vs_v5_body_mm3: {vs_v5_lost:.6f}")
    out(f"body_camera_collision_mm3: {body_camera:.6f}")
    out(f"top_camera_collision_mm3: {top_camera:.6f}")
    out(f"body_CAN_collision_mm3: {body_can:.6f}")
    out(f"top_CAN_collision_mm3: {top_can:.6f}")
    out(f"top_body_overlap_mm3: {top_body:.6f}")
    out(f"top_hole_camera_collision_mm3: {top_hole_collision:.6f}")
    out(f"top_lift_along_tilted_axis_collision_mm3 (0.5,2,5,10,20 mm): {[round(v, 6) for v in lift_collisions]}")
    out(f"top_bar_60mm_sweep_vs_body_mm3: {bar_sweep_body:.6f}  vs_CAN_mm3: {bar_sweep_can:.6f}")
    out(f"camera_insertion_path_max_collision_mm3: {max(insertion_collisions):.6f}")
    out(f"camera_90mm_insertion_sweep_vs_body_mm3: {camera_sweep_body:.6f}")
    for k, v in keepouts.items():
        out(f"{k}_20mm_connector_keepout_collision_mm3: {v:.6f}")
    out(f"retention_cradle_vs_v5_in_camera_frame_added_mm3: {cradle_diff_add:.6f}")
    out(f"retention_cradle_vs_v5_in_camera_frame_lost_mm3: {cradle_diff_lost:.6f}")
    out(f"top_bar_vs_v5_in_camera_frame_symdiff_mm3: {top_diff:.6f}")
    out(f"retention_region_volume_compared_mm3: new {new_back.volume:.3f} v5 {old_cradle.volume:.3f}")
    out(f"bottom_port_column_(tilted z -76.3..3.7)_body_material_mm3: {port_material:.3f}"
          f" (material of bridge/interface below the opening, if any)")
    out(f"body_volume_mm3: {body.volume:.3f} (v5 {v5_body.volume:.3f})  top_volume_mm3: {top.volume:.3f} (v5 {v5_top.volume:.3f})")
    out(f"body_bbox: {body.bounding_box()}")
    return checks


def main():
    tilt = float(sys.argv[1]) if len(sys.argv) > 1 else TILT_DEG
    paths = out_paths(tilt)
    body, top, interface, camera, can, top_holes, loc = build_parts(tilt)
    bd.export_step(body, str(paths["body.step"])); bd.export_stl(body, str(paths["body.stl"]))
    bd.export_step(top, str(paths["top_bar.step"])); bd.export_stl(top, str(paths["top_bar.stl"]))
    print(f"TILT_DEG: {tilt:g}")
    for k, p in paths.items():
        print(f"{k}: {p.name}")
    checks = brep_checks(body, top, interface, camera, can, top_holes, loc)
    failed = [k for k, ok in checks.items() if not ok]
    print(f"CHECKS: {'ALL PASS' if not failed else 'FAILED ' + ', '.join(failed)}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
