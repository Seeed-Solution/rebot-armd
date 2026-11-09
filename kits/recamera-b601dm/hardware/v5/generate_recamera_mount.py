"""v5 clip-on reCamera B601 holder.

The body and removable top pressure bar are exported separately.  No B3_CAN
rear-cover fixing holes are used; retention is entirely by the printed U
guard, front lip, and top bar.
"""

from pathlib import Path

import build123d as bd
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
REFERENCE_STEP = ROOT / "b601-camera-mounts" / "D435_Gemini2_Mount.step"
CAN_STEP = Path(__file__).resolve().parent.parent / "reference" / "B3_CAN.step"
OUT_DIR = Path(__file__).resolve().parent
BODY_STEP = OUT_DIR / "reCamera_B601_Mount_v5_body.step"
BODY_STL = OUT_DIR / "reCamera_B601_Mount_v5_body.stl"
TOP_STEP = OUT_DIR / "reCamera_B601_Mount_v5_top_bar.step"
TOP_STL = OUT_DIR / "reCamera_B601_Mount_v5_top_bar.stl"

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


def cradle_location():
    R = Rotation.from_quat([.507205266179, .498692788059,
                             .491587190903, .502384668195]).as_matrix()
    target = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    local = R.T @ target
    return bd.Plane(origin=CRADLE_ORIGIN, x_dir=tuple(local[:, 0]),
                    z_dir=tuple(local[:, 2])).location


def vol(a, b):
    x = a & b
    return 0.0 if x is None else x.volume

def cover_location(loc):
    # Roll camera 90 degrees: original -X connector side faces upward;
    # original +/-Y connector sides face left/right. Lens still faces +Y.
    return loc * bd.Plane(origin=(0,21.8,24), x_dir=(0,0,-1), z_dir=(0,1,0)).location


def build_parts():
    ref = bd.import_step(str(REFERENCE_STEP)).solids()[0]
    interface = ref & bd.Box(100, 30, 100).located(bd.Location((0, -15, 0)))
    loc = cradle_location()

    # Existing compact v4 backend, without any CAN-cover hole alignment.
    bridge = bd.Box(39.0, 5.0, 14.0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        bd.Location((0, INTERFACE_Y, 7.0)))
    # A solid rear stop has no CAN holes.  It ends at local y=3.5, so the
    # complete CAN side/underside channel y=3.5..15.2 remains open.
    back_stop = bd.Box(INNER_W + 2 * WALL, 3.5, BODY_Z1 - SUPPORT_Z0,
                       align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                           loc * bd.Location((0, 0, SUPPORT_Z0)))
    # Low outside rails connect the rear stop to the front guard.
    rails = [bd.Box(3,43,3,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)).located(
        loc * bd.Location((x,0,0.7))) for x in (-21.8,21.8)]

    ear_x = INNER_W / 2 + EAR_T / 2
    side_l = bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, BODY_Z1 - BODY_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((-ear_x, GUARD_Y0, BODY_Z0)))
    side_r = bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, BODY_Z1 - BODY_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((ear_x, GUARD_Y0, BODY_Z0)))
    # Bottom support starts only at y=17, clear of the rear-cover underside.
    bottom = bd.Box(INNER_W, FRONT_Y1 - GUARD_Y0, SUPPORT_Z1 - SUPPORT_Z0,
                    align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                        loc * bd.Location((0, GUARD_Y0, SUPPORT_Z0)))
    # Front lip prevents forward sliding; it is beyond the camera envelope.
    lip = bd.Box(INNER_W, FRONT_Y1 - CAM_DEPTH - 3.5, 1.5,
                 align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                     loc * bd.Location((0, CAM_DEPTH + 3.5, SUPPORT_Z1)))

    body = (interface + bridge + back_stop + rails[0] + rails[1] + side_l + side_r + bottom + lip)
    cavity = bd.Box(INNER_W,36.5,40.6,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)).located(loc*bd.Location((0,3.5,3.7)))
    bottom_port = bd.Box(20,13.5,20,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)).located(loc*bd.Location((0,3.5,-16)))
    body = body - cavity - bottom_port
    rear_region = bd.Box(200, 100, 200).located(bd.Location((0, -50, 0)))
    body = (body - rear_region) + interface
    for x in (-ear_x,ear_x):
        bore = bd.Cylinder(1.6,12,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).located(loc*bd.Location((x,35,37)))
        # Side-open square nut pocket, outside camera volume.
        nut = bd.Box(9,5.8,2.6,align=(bd.Align.CENTER,bd.Align.CENTER,bd.Align.MIN)).located(loc*bd.Location((x,35,38)))
        body = body - bore - nut

    body_solids = body.solids()
    if len(body_solids) != 1 or not body_solids[0].is_valid:
        raise RuntimeError(f"body expected one valid solid, got {len(body_solids)}")

    # Removable top bar.  Its holes are only in the two printed ears.
    top = bd.Box(INNER_W, FRONT_Y1 - GUARD_Y0, TOP_T,
                 align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                     loc * bd.Location((0, GUARD_Y0, TOP_Z0)))
    for x in (-ear_x, ear_x):
        top += bd.Box(EAR_T, FRONT_Y1 - GUARD_Y0, TOP_T,
                      align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                          loc * bd.Location((x, GUARD_Y0, TOP_Z0)))
    hole_tools = []
    for x in (-ear_x, ear_x):
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


def main():
    body, top, interface, camera, can, top_holes, loc = build_parts()
    original = bd.import_step(str(REFERENCE_STEP)).solids()[0]
    preserved = bd.Box(200, 100, 200).located(bd.Location((0, -50, 0)))
    added = (body & preserved) - original
    added_v = 0.0 if added is None else added.volume
    lost_v = vol(interface, original - body)
    body_camera = vol(body, camera)
    top_camera = vol(top, camera)
    body_can = vol(body, can)
    top_can = vol(top, can)
    top_hole_collision = sum(vol(h, camera) for h in top_holes)
    # Vertical removal path: lift the bar by 10 mm; it must clear the camera.
    up = (loc * bd.Location((0,0,10))).position - loc.position
    lifted_top = top.moved(bd.Location(up))
    lift_camera = vol(lifted_top, camera)
    insertion_collisions = []
    for offset in (0.0, 5.0, 17.0, 40.0):
        insertion_camera = bd.Box(CAM_W, CAM_DEPTH, CAM_H,
                                  align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
                                      loc * bd.Location((0, 3.5, 4.0 + offset)))
        insertion_collisions.append(vol(body, insertion_camera))
    max_insertion_collision = max(insertion_collisions)

    assert abs(added_v) < 1e-6, f"added material in preserved interface: {added_v}"
    assert abs(lost_v) < 1e-6, f"lost preserved interface material: {lost_v}"
    assert body_camera < 1e-6 and top_camera < 1e-6
    assert body_can < 1e-6 and top_can < 1e-6
    assert top_hole_collision < 1e-6 and lift_camera < 1e-6
    assert max_insertion_collision < 1e-6
    for name, pos, size in (
        ("left", (-40.3,3.6,4.1), (20,13.3,39.8)),
        ("right", (20.3,3.6,4.1), (20,13.3,39.8)),
        ("upper", (-19.9,3.6,44.1), (39.8,13.3,20)),
    ):
        keepout = bd.Box(*size, align=(bd.Align.MIN,)*3).located(loc*bd.Location(pos))
        collision = vol(body,keepout) + vol(top,keepout)
        assert collision < 1e-6, f"Blocked connector corridor: {name}"
        print(f"{name}_20mm_connector_corridor_collision_mm3: {collision:.6f}")
    sweep = bd.Box(40,36.5,90,align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.MIN)).located(loc*bd.Location((0,3.5,4)))
    assert vol(body,sweep) < 1e-6, "Blocked continuous top insertion path"

    bd.export_step(body, str(BODY_STEP)); bd.export_stl(body, str(BODY_STL))
    bd.export_step(top, str(TOP_STEP)); bd.export_stl(top, str(TOP_STL))
    print(f"BODY_STEP: {BODY_STEP}")
    print(f"BODY_STL: {BODY_STL}")
    print(f"TOP_STEP: {TOP_STEP}")
    print(f"TOP_STL: {TOP_STL}")
    print(f"body_solid_count: {len(body.solids())}")
    print(f"top_solid_count: {len(top.solids())}")
    print(f"body_valid: {body.is_valid}")
    print(f"top_valid: {top.is_valid}")
    print(f"interface_y_le_0_added_mm3: {added_v:.6f}")
    print(f"interface_y_le_0_lost_mm3: {lost_v:.6f}")
    print(f"body_camera_collision_mm3: {body_camera:.6f}")
    print(f"top_camera_collision_mm3: {top_camera:.6f}")
    print(f"body_CAN_collision_mm3: {body_can:.6f}")
    print(f"top_CAN_collision_mm3: {top_can:.6f}")
    print(f"top_hole_camera_collision_mm3: {top_hole_collision:.6f}")
    print(f"vertical_removal_collision_mm3: {lift_camera:.6f}")
    print(f"camera_insertion_path_max_collision_mm3: {max_insertion_collision:.6f}")
    print(f"body_bbox: {body.bounding_box()}")
    print(f"top_bbox: {top.bounding_box()}")


if __name__ == "__main__":
    main()
