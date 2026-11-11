"""Simulated reCamera on the gripper: mount, intrinsics, ground-truth detections, H.264.

Mount convention = armd `kinematics::cam_mount` (OpenCV camera frame: x right, y down,
z = optical axis): R_end_cam = [x0 y0 z0] · Rz(rot_deg) · Rx(tilt_deg) with z0 = end_link x,
y0 = flip · (J5 axis in end_link at q6 = pi/2, x part removed), x0 = y0 × z0. The optical
centre sits at `offset_m` in end_link. armd's profile leaves `offset_m` null (not measured);
the simulator then uses the v8 holder's assumed lens centre (reBot-DevArm
hardware/camera-mounts/recamera-mounts/v8/README.txt section 4).

Stream geometry: OV5647 1080p window 2.688 x 1.512 mm (1:1 centre crop) behind f = 3.46 mm,
so HFOV 42.47 deg, VFOV 24.65 deg (derived in the same README, section 1.1; not measured on a
calibration target). Pixels are square: fx = fy = (H / 2) / tan(VFOV / 2), 824 px at 640x360.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import mujoco
import numpy as np

FOCAL_MM, WINDOW_W_MM, WINDOW_H_MM = 3.46, 2.688, 1.512
VFOV_DEG = 2 * math.degrees(math.atan(WINDOW_H_MM / 2 / FOCAL_MM))
HFOV_DEG = 2 * math.degrees(math.atan(WINDOW_W_MM / 2 / FOCAL_MM))
DEFAULT_OFFSET_M = (-0.0731, 0.0005, 0.0660)  # v8 holder lens centre in end_link (assumed)
# Detections are normalised to the visible 640x360 band, like rebot-vision.
DETECTION_SIZE = (640, 360)
PROFILE = Path(__file__).resolve().parents[4] / "kits" / "recamera-b601dm" / "profile" / "rebot-b601dm.json"


@dataclass(frozen=True)
class CameraMount:
    flip: bool = False
    rot_deg: float = 0.0
    tilt_deg: float = 0.0
    offset_m: tuple[float, float, float] | None = None

    @classmethod
    def from_profile(cls, path: Path | str = PROFILE) -> "CameraMount":
        m = json.loads(Path(path).read_text())["camera_mount"]
        off = m.get("offset_m")
        return cls(bool(m.get("flip", False)), float(m.get("rot_deg", 0.0)), float(m.get("tilt_deg", 0.0)),
                   tuple(float(v) for v in off) if off else None)

    @property
    def offset(self) -> np.ndarray:
        return np.array(self.offset_m if self.offset_m is not None else DEFAULT_OFFSET_M, dtype=float)


def axis_r(k, a: float) -> np.ndarray:
    """Rotation about unit axis k by a (rad), as armd `kinematics::axis_r`."""
    x, y, z = k
    c, s = math.cos(a), math.sin(a)
    cc = 1 - c
    return np.array([[c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s],
                     [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s],
                     [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc]])


def mount_rotation(model: mujoco.MjModel, mount: CameraMount) -> np.ndarray:
    """R_end_cam (OpenCV camera axes as columns, in end_link), armd `cam_mount`."""
    data = mujoco.MjData(model)
    for j in range(1, 6):
        data.qpos[model.jnt_qposadr[model.joint(f"joint{j}").id]] = 0.0
    data.qpos[model.jnt_qposadr[model.joint("joint6").id]] = math.pi / 2
    mujoco.mj_kinematics(model, data)
    end_r = data.body("end_link").xmat.reshape(3, 3)
    a_end = end_r.T @ data.xaxis[model.joint("joint5").id]
    sgn = -1.0 if mount.flip else 1.0
    y = np.array([0.0, a_end[1] * sgn, a_end[2] * sgn])
    y /= np.linalg.norm(y)
    z = np.array([1.0, 0.0, 0.0])
    x = np.cross(y, z)
    r = np.column_stack([x, y, z])
    if mount.rot_deg:
        r = r @ axis_r((0.0, 0.0, 1.0), math.radians(mount.rot_deg))
    if mount.tilt_deg:
        r = r @ axis_r((1.0, 0.0, 0.0), math.radians(mount.tilt_deg))
    return r


def add_cameras(spec: mujoco.MjSpec, r_end_cam: np.ndarray, offset: np.ndarray) -> None:
    """`wrist` on end_link (MuJoCo camera: x right, y up, looks along -z) and a fixed `scene`."""
    r_mj = np.column_stack([r_end_cam[:, 0], -r_end_cam[:, 1], -r_end_cam[:, 2]])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, r_mj.flatten())
    cam = spec.body("end_link").add_camera(name="wrist", pos=list(offset), quat=list(quat))
    cam.fovy = VFOV_DEG
    eye, target = np.array([0.85, -0.65, 0.55]), np.array([0.22, 0.0, 0.05])
    fwd = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    sq = np.zeros(4)
    mujoco.mju_mat2Quat(sq, np.column_stack([right, up, -fwd]).flatten())
    scene = spec.worldbody.add_camera(name="scene", pos=list(eye), quat=list(sq))
    scene.fovy = 45.0
    spec.visual.map.znear = 0.002  # fraction of the model extent: about 3 mm, the jaws are close


def focal_px(height: int) -> float:
    return height / 2 / math.tan(math.radians(VFOV_DEG) / 2)


def project_points(data: mujoco.MjData, cam_id: int, pts: np.ndarray, size=DETECTION_SIZE) -> tuple[np.ndarray, np.ndarray]:
    """Pixels (N x 2) at `size` and depths of world points in MuJoCo camera `cam_id`."""
    pos, r = data.cam_xpos[cam_id], data.cam_xmat[cam_id].reshape(3, 3)
    pc = (np.asarray(pts, float) - pos) @ r  # camera-frame coordinates (rows)
    depth = -pc[:, 2]
    f = focal_px(size[1])
    with np.errstate(divide="ignore", invalid="ignore"):
        u = size[0] / 2 + f * pc[:, 0] / depth
        v = size[1] / 2 - f * pc[:, 1] / depth
    return np.column_stack([u, v]), depth


def grip_pixel_norm(r_end_cam: np.ndarray, offset: np.ndarray, height_m: float) -> list[float]:
    """Where the table point `height_m` below the jaw tips (along the tool axis, end_link x)
    appears in the image, normalised: armd's `grasp.grip_pixel_norm` for this mount."""
    pc = r_end_cam.T @ (np.array([height_m, 0.0, 0.0]) - offset)
    f = focal_px(DETECTION_SIZE[1])
    return [round(float((DETECTION_SIZE[0] / 2 + f * pc[0] / pc[2]) / DETECTION_SIZE[0]), 4),
            round(float((DETECTION_SIZE[1] / 2 + f * pc[1] / pc[2]) / DETECTION_SIZE[1]), 4)]


def segmentation_ids(seg: np.ndarray) -> np.ndarray:
    """Geom id per pixel (-1 elsewhere) from a MuJoCo segmentation render (H x W x 2)."""
    return np.where(seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM), seg[:, :, 0], -1)  # int: an enum compares per element in Python


class SegmentationError(RuntimeError):
    """An ID-colour readback with values that are no segmentation id; `diag` describes it."""

    def __init__(self, msg: str, diag: dict):
        super().__init__(msg)
        self.diag = diag


def decode_segmentation(rgb: np.ndarray, scene: mujoco.MjvScene) -> np.ndarray:
    """Geom id per pixel (-1 elsewhere) from an ID-colour readback (H x W x 3 uint8) of `scene`.
    Unlike mujoco's Renderer (4.0.0 indexes its lookup table with the raw values), values above
    the scene's geom count raise SegmentationError."""
    seg = rgb[..., 0].astype(np.int64) | (rgb[..., 1].astype(np.int64) << 8) | (rgb[..., 2].astype(np.int64) << 16)
    n = scene.ngeom
    bad = seg > n
    if bad.any():
        diag = {"bad_pixels": int(bad.sum()), "max_value": int(seg.max()), "ngeom": int(n)}
        raise SegmentationError(f"segmentation readback out of range: {diag}", diag)
    lut = np.full(n + 1, -1, np.int32)  # id 0: background
    for g in scene.geoms[:n]:
        if g.segid != -1 and g.objtype == int(mujoco.mjtObj.mjOBJ_GEOM):
            lut[g.segid + 1] = g.objid
    return lut[seg]


def render_segmentation(renderer: mujoco.Renderer) -> np.ndarray:
    """Segmentation (geom id per pixel) of the renderer's current scene: an ID-colour render
    read back as RGB and decoded with bounds checks (decode_segmentation)."""
    flags = renderer.scene.flags
    flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True
    try:
        rgb = renderer.render()
    finally:
        flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = False
    return decode_segmentation(rgb, renderer.scene)


def detections(geom_ids: np.ndarray, scene, min_pixels: int = 20) -> list[dict]:
    """Ground-truth boxes of the scene objects (rebot-vision format) from a segmentation
    render at DETECTION_SIZE: the bounds of each object's visible pixels, so occlusion (by the
    gripper or another object) and the image edge are accounted for. Objects with fewer than
    `min_pixels` visible pixels are not reported."""
    h, w = geom_ids.shape
    out = []
    for i, (obj, gid) in enumerate(zip(scene.objects, scene.geom_ids)):
        ys, xs = np.nonzero(geom_ids == gid)
        if len(xs) < min_pixels:
            continue
        n0, n1 = round(int(xs.min()) / w, 4), round((int(xs.max()) + 1) / w, 4)
        m0, m1 = round(int(ys.min()) / h, 4), round((int(ys.max()) + 1) / h, 4)
        out.append({"class_id": i, "format": "xywh_center", "class_name": obj.kind, "name": obj.name,
                    "confidence": 0.99, "x": (n0 + n1) / 2, "y": (m0 + m1) / 2,
                    "w": round(n1 - n0, 4), "h": round(m1 - m0, 4), "pixels": int(len(xs))})
    return out


class H264Encoder:
    """libx264 (PyAV), one Annex-B access unit per frame, SPS / PPS before every IDR."""

    def __init__(self, width: int, height: int, fps: int):
        import av

        self.ctx = av.CodecContext.create("libx264", "w")
        self.ctx.width, self.ctx.height, self.ctx.pix_fmt = width, height, "yuv420p"
        self.ctx.time_base, self.ctx.framerate = Fraction(1, fps), Fraction(fps, 1)
        self.ctx.gop_size, self.ctx.max_b_frames = fps, 0
        self.ctx.options = {"preset": "ultrafast", "tune": "zerolatency", "forced-idr": "1", "crf": "23"}
        self.ctx.open()
        self.pts = 0
        self._av = av

    def encode(self, rgb: np.ndarray, keyframe: bool = False) -> bytes:
        frame = self._av.VideoFrame.from_ndarray(rgb, format="rgb24").reformat(format="yuv420p")
        frame.pts = self.pts
        self.pts += 1
        if keyframe:
            frame.pict_type = self._av.video.frame.PictureType.I
        return b"".join(bytes(p) for p in self.ctx.encode(frame))
