"""Shared camera and DM-gripper model for v8 (no build123d needed except lens_frame()).

Frames
- end_link (= MJCF DM `end_link` = armd URDF `end_link` = v7 "gripper_end"): mm, jaw tip at the
  origin, +X = approach direction (away from the arm), Y = jaw open/close axis, Z = gripper
  thickness; the holder sits on +Z.  DM gripper meshes coincide with the RS meshes v7 used
  (median nearest-neighbour distance 0.78 mm, identity transform; see README section 1.3).
- Holder mount frame -> end_link: MOUNT_R, MOUNT_P_MM (same as v7, from the RS MJCF D435 mount
  geom pose).

Camera (see README section 1.1)
- OV5647, 1.4 um pixels, 2592x1944 array.  Stream mode 1920x1080p30 is a 1:1 window (no
  binning, registers 0x3800-0x3807 = x 348..2275, y 434..1521), so the 1080p image covers
  2.688 x 1.512 mm of the sensor.  1280x720 / 640x360 are scaled from it (no further crop).
- f = 3.46 mm (Seeed spec).  Derived: HFOV 42.47 deg, VFOV 24.65 deg, DFOV 48.1 deg for the
  16:9 stream; fx = fy = 2471 px at 1920x1080 (1648 at 1280x720, 824 at 640x360).
- Lens = centre of the 40 x 40 front face of the camera envelope (ASSUMPTION, as in v7).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from math import atan, degrees
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
DM = REPO / "Rebot_Arm_description/DM"
DM_XML = DM / "mjcf/rebotarm_b601_stl.xml"

PIXEL_MM = 0.0014
FOCAL_MM = 3.46
SENSOR_W_PX, SENSOR_H_PX = 1920, 1080          # 1080p30 window
HALF_H_DEG = degrees(atan(SENSOR_W_PX * PIXEL_MM / 2 / FOCAL_MM))   # 21.23
HALF_V_DEG = degrees(atan(SENSOR_H_PX * PIXEL_MM / 2 / FOCAL_MM))   # 12.32
FX_1080 = FOCAL_MM / PIXEL_MM                                         # 2471.4 px
V7_ASSUMED_HALF_V = (17.0, 25.0)
V7_ASSUMED_HALF_H = 40.0

MOUNT_R = np.array([
    [0.019295, 0.011947, 0.999742],
    [0.999811, 0.002170, -0.019322],
    [-0.0024, 0.999926, -0.011903],
])
MOUNT_P_MM = np.array([-120.1, 0.3, 45.0])

PLATE_T, BOTTOM_T = 3.5, 4.0
CAM_W, CAM_H, CAM_DEPTH = 40.0, 40.0, 36.5
OPENINGS_MM = (0.0, 25.0, 42.5)   # per-finger slide; 42.5 = armd URDF upper limit (85 mm span)


def quat_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _load(rel: str) -> trimesh.Trimesh:
    m = trimesh.load_mesh(DM / "meshes" / rel, process=False)
    m.apply_scale(1000.0)
    return m


def dm_gripper(opening_mm: float = 0.0) -> dict[str, trimesh.Trimesh]:
    """DM gripper in end_link (mm): base, link6 (motor housing), both fingers at `opening_mm`
    per-finger slide (left +Y, right -Y, MJCF finger_left/finger_right slide joints)."""
    root = ET.parse(DM_XML).getroot()
    end = root.find(".//body[@name='end_link']")
    p = np.fromstring(end.attrib["pos"], sep=" ") * 1000.0
    r = quat_matrix(np.fromstring(end.attrib["quat"], sep=" "))
    link6 = _load("collision/link6.STL")
    link6.vertices = (link6.vertices - p) @ r          # R^T (v - p), row-vector form
    left = _load("collision/left_finger.stl")
    right = _load("collision/right_finger.stl")
    left.apply_translation((0, opening_mm, 0))
    right.apply_translation((0, -opening_mm, 0))
    return {"gripper_base": _load("collision/gripper_base.stl"), "link6": link6,
            "finger_left": left, "finger_right": right}


def finger_slides() -> dict[str, np.ndarray]:
    return {"finger_left": np.array([0.0, 1.0, 0.0]), "finger_right": np.array([0.0, -1.0, 0.0])}


def to_end(pts_mount):
    return (MOUNT_R @ np.asarray(pts_mount, float).T).T + MOUNT_P_MM


def lens_frame(loc):
    """Lens centre, forward, up, right (end_link) from a build123d cradle location."""
    import build123d as bd
    lens_l = loc * bd.Location((0, PLATE_T + CAM_DEPTH, BOTTOM_T + CAM_H / 2))
    o = np.array(tuple(lens_l.position))

    def local_dir(v):
        return np.array(tuple((loc * bd.Location(v)).position)) - np.array(tuple(loc.position))
    fwd, up, right = (MOUNT_R @ local_dir(v) for v in ((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    return to_end(o[None])[0], fwd, up, right


def project(pts, lens, fwd, up, right, roll90: bool = False):
    """Normalised image coordinates (u right, v down; +-1 = image edge) and depth.
    roll90: camera rolled 90 deg about its optical axis, so the long image axis spans
    the gripper up/down direction (image 'down' = -up still maps to +v)."""
    rel = np.asarray(pts, float) - lens
    z = rel @ fwd
    x = rel @ right
    y = -(rel @ up)
    th, tv = np.tan(np.radians(HALF_H_DEG)), np.tan(np.radians(HALF_V_DEG))
    if roll90:
        th, tv = tv, th
    with np.errstate(divide="ignore", invalid="ignore"):
        u = x / z / th
        v = y / z / tv
    return u, v, z
