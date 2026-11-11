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


def geom_points(model: mujoco.MjModel, data: mujoco.MjData, gid: int) -> np.ndarray:
    """Points on the geom's surface whose projection bounds its image box (world frame)."""
    size, pos, r = model.geom_size[gid], data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3)
    t = model.geom_type[gid]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * size
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        ring = np.column_stack([np.cos(a), np.sin(a)]) * size[0]
        local = np.vstack([np.column_stack([ring, np.full(24, z)]) for z in (-size[1], size[1])])
    else:  # sphere and others: bounding sphere samples
        g = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)], float)
        local = g / np.linalg.norm(g, axis=1, keepdims=True) * model.geom_rbound[gid]
    return pos + local @ r.T


def detections(model: mujoco.MjModel, data: mujoco.MjData, cam_id: int, scene) -> list[dict]:
    """Ground-truth boxes of the scene objects in the wrist image (rebot-vision format).

    Box = bounds of the projected object surface points, clipped to the image; an object with
    any point behind the camera or entirely outside the image is not reported. Occlusion (by
    the gripper or other objects) is not modelled.
    """
    out = []
    w, h = DETECTION_SIZE
    for i, (obj, gid) in enumerate(zip(scene.objects, scene.geom_ids)):
        px, depth = project_points(data, cam_id, geom_points(model, data, gid))
        if np.any(depth <= 0.01):
            continue
        x0, y0 = np.clip(px.min(axis=0), 0, [w, h])
        x1, y1 = np.clip(px.max(axis=0), 0, [w, h])
        if x1 - x0 < 1 or y1 - y0 < 1:
            continue
        out.append({"class_id": i, "format": "xywh_center", "class_name": obj.kind, "name": obj.name,
                    "confidence": 0.99, "x": round((x0 + x1) / 2 / w, 4), "y": round((y0 + y1) / 2 / h, 4),
                    "w": round((x1 - x0) / w, 4), "h": round((y1 - y0) / h, 4)})
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
