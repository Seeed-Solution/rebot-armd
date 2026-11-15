"""Simulated reCamera on the gripper: mount, intrinsics, ground-truth detections, H.264.

Mount convention = armd `kinematics::cam_mount` (OpenCV camera frame: x right, y down,
z = optical axis): R_end_cam = [x0 y0 z0] · Rz(rot_deg) · Rx(tilt_deg) with z0 = end_link x,
y0 = flip · (J5 axis in end_link at the J6 rest angle, motor pi/2 = URDF pi/2 + offset; x part
removed), x0 = y0 × z0. The optical
centre sits at `offset_m` in end_link. armd's profile leaves `offset_m` null (not measured);
the simulator then uses the v8 holder's assumed lens centre (reBot-DevArm
hardware/camera-mounts/recamera-mounts/v8/README.txt section 4).

Stream geometry: OV5647 1080p window 2.688 x 1.512 mm (1:1 centre crop) behind f = 3.46 mm,
so HFOV 42.47 deg, VFOV 24.65 deg (derived in the same README, section 1.1; not measured on a
calibration target). Pixels are square: fx = fy = (H / 2) / tan(VFOV / 2), 824 px at 640x360.
"""
from __future__ import annotations

import itertools
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
SO101_PROFILE = Path(__file__).resolve().parents[4] / "profiles" / "so_arm101.json"


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
    joint_names = {model.joint(i).name for i in range(model.njnt)}
    if "shoulder_pan" in joint_names:
        mujoco.mj_forward(model, data)
        end_r = data.body("gripper").xmat.reshape(3, 3)
        cam_r = data.cam_xmat[model.camera("wrist").id].reshape(3, 3)
        r_end_mj = end_r.T @ cam_r
        return np.column_stack([r_end_mj[:, 0], -r_end_mj[:, 1], -r_end_mj[:, 2]])
    from .model import MODEL_OFFSETS_RAD
    for j, motor in enumerate((0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2), start=1):
        data.qpos[model.jnt_qposadr[model.joint(f"joint{j}").id]] = motor + MODEL_OFFSETS_RAD[j - 1]
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
    if not any(c.name == "scene" for c in spec.worldbody.cameras):
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


_canary_frames = itertools.count(1)

# Private mujoco.Renderer attributes render_segmentation uses (checked against mujoco 4.0.0).
_RENDERER_INTERNALS = ("_mjr_context", "_rect", "_gl_context", "height", "width", "scene")


def check_renderer_internals(renderer: mujoco.Renderer) -> None:
    """Raise RuntimeError naming the missing attributes when this mujoco's Renderer lacks the
    internals render_segmentation relies on (they are not public API)."""
    missing = [a for a in _RENDERER_INTERNALS if not hasattr(renderer, a)]
    if missing:
        raise RuntimeError(f"mujoco {mujoco.__version__}: Renderer lacks {missing}, used by render_segmentation "
                           "(written against mujoco 4.0.0); update rebot_sim.camera for this version")


def render_segmentation(renderer: mujoco.Renderer) -> np.ndarray:
    """Segmentation (geom id per pixel) of the renderer's current scene: an ID-colour render
    read back as RGB and decoded with bounds checks (decode_segmentation).

    When the GPU driver runs out of memory (NVRM NV_ERR_NO_MEMORY in the kernel log, seen on
    the GB10 when other processes hold most of the unified memory), it drops rendering commands
    without a GL or EGL error, and the readback holds stale or uninitialised memory. To detect
    that, a canary pixel with a colour that changes every frame and is no segmentation id is
    drawn after the scene, in the offscreen buffer row just above the image. One readback covers
    the image and that row; a missing canary raises SegmentationError, then the row is cropped
    away. Needs an offscreen buffer taller than the renderer
    (model.vis.global_.offheight > height)."""
    check_renderer_internals(renderer)
    con, rect = renderer._mjr_context, renderer._rect
    if con.offHeight <= renderer.height:
        raise ValueError(f"offscreen buffer height {con.offHeight} leaves no canary row above {renderer.height}")
    canary = (0xFE, next(_canary_frames) % 256, 0xFE)  # value 0xFExxFE: above any geom count
    canary_rect = mujoco.MjrRect(0, renderer.height, 1, 1)
    read_rect = mujoco.MjrRect(0, 0, renderer.width, renderer.height + 1)  # image + canary row
    flags = renderer.scene.flags
    flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True
    try:
        if renderer._gl_context:
            renderer._gl_context.make_current()
        mujoco.mjr_render(rect, renderer.scene, con)
        mujoco.mjr_rectangle(canary_rect, *(c / 255 for c in canary), 1.0)
        buf = np.empty((renderer.height + 1, renderer.width, 3), np.uint8)
        mujoco.mjr_readPixels(buf, None, read_rect, con)
    finally:
        flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = False
    seen = buf[renderer.height, 0]  # glReadPixels rows run bottom-up: the last row is y = height
    if tuple(int(c) for c in seen) != canary:
        diag = {"canary_expected": list(canary), "canary_read": seen.tolist()}
        raise SegmentationError(f"segmentation render not executed (GPU out of memory?): {diag}", diag)
    rgb = buf[:renderer.height]
    if renderer._gl_context:  # EGL / OSMesa / GLFW read back bottom-up
        rgb = np.flipud(rgb)
    return decode_segmentation(rgb, renderer.scene)


def mask_orientation(xs: np.ndarray, ys: np.ndarray, scale_x: float = 1.0, scale_y: float = 1.0) -> tuple[float, float]:
    """Orientation of the long side of a pixel set (second moments): (angle_deg, angle_conf).
    The angle runs from image +x toward image +y (y down) in [-90, 90); angle_conf is
    1 - minor/major eigenvalue ratio, 0 for a round or square footprint (angle ambiguous)."""
    px = np.asarray(xs, dtype=np.float64) * scale_x
    py = np.asarray(ys, dtype=np.float64) * scale_y
    if len(px) == 0:
        return 0.0, 0.0
    dx, dy = px - px.mean(), py - py.mean()
    cxx, cyy, cxy = float((dx * dx).mean()), float((dy * dy).mean()), float((dx * dy).mean())
    angle = math.degrees(0.5 * math.atan2(2 * cxy, cxx - cyy))  # (-90, 90]
    if angle >= 90.0 - 1e-9:
        angle -= 180.0
    half = math.hypot((cxx - cyy) / 2, cxy)
    major, minor = (cxx + cyy) / 2 + half, (cxx + cyy) / 2 - half
    conf = 0.0 if major <= 1e-9 else 1.0 - max(minor, 0.0) / major
    return round(angle, 2), round(min(max(conf, 0.0), 1.0), 3)


def mask_cross_section(
    xs: np.ndarray,
    ys: np.ndarray,
    step_deg: int = 5,
    band_px: float = 8.0,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
) -> tuple[float, float, float]:
    """Orientation and grasp width of a pixel set via minimum cross section.
    Returns (angle_deg, angle_conf, grasp_width_px).
    angle_deg is the orientation of the long side in [-90, 90) (closing direction perpendicular).
    grasp_width_px is the cross-section width along the short axis.
    angle_conf is 1 - w_min / w_max in [0, 1] (0.0 if w_max < 12.0)."""
    px = np.asarray(xs, dtype=np.float64) * scale_x
    py = np.asarray(ys, dtype=np.float64) * scale_y
    if len(px) == 0:
        return 0.0, 0.0, 0.0

    gx, gy = float(px.mean()), float(py.mean())
    mask_pts = set(zip(np.round(xs).astype(int), np.round(ys).astype(int)))
    qx, qy = int(round(gx / scale_x)), int(round(gy / scale_y))
    if (qx, qy) not in mask_pts:
        dists_sq = (px - gx) ** 2 + (py - gy) ** 2
        best_idx = int(np.argmin(dists_sq))
        gx, gy = float(px[best_idx]), float(py[best_idx])

    dx = px - gx
    dy = py - gy
    thetas = list(range(0, 180, step_deg))
    widths: dict[int, float] = {}
    for theta in thetas:
        rad = math.radians(theta)
        cos_t, sin_t = math.cos(rad), math.sin(rad)
        n_dot = -sin_t * dx + cos_t * dy
        in_band = np.abs(n_dot) <= (band_px / 2.0)
        if np.any(in_band):
            proj = cos_t * dx[in_band] + sin_t * dy[in_band]
            # + one sample's extent along the closing direction: a mask sample is scale_x x scale_y px
            widths[theta] = float(proj.max() - proj.min() + abs(cos_t) * scale_x + abs(sin_t) * scale_y)
        else:
            widths[theta] = 0.0

    vals = list(widths.values())
    w_min = min(vals)
    w_max = max(vals)

    thresh = max(w_min + 1.0, w_min * 1.05)
    candidates = [t for t in thetas if widths[t] <= thresh]

    moments_angle, _ = mask_orientation(xs, ys, scale_x=scale_x, scale_y=scale_y)
    pca_short = moments_angle + 90.0

    best_cand = candidates[0]
    best_key = None
    for theta in candidates:
        ang_dist = abs((theta - pca_short + 90.0) % 180.0 - 90.0)
        key = (round(ang_dist, 5), round(widths[theta], 5))
        if best_key is None or key < best_key:
            best_key = key
            best_cand = theta

    theta_star = best_cand
    angle_deg = ((theta_star + 90.0 + 90.0) % 180.0) - 90.0
    if angle_deg >= 90.0 - 1e-9:
        angle_deg = -90.0

    grasp_width_px = widths[theta_star]
    angle_conf = 0.0 if w_max < 12.0 else min(max(1.0 - w_min / w_max, 0.0), 1.0)
    return round(angle_deg, 2), round(angle_conf, 3), round(grasp_width_px, 2)


def detections(geom_ids: np.ndarray, scene, min_pixels: int = 20) -> list[dict]:
    """Ground-truth boxes of the scene objects (rebot-vision format) from a segmentation
    render at DETECTION_SIZE: the bounds of each object's visible pixels, so occlusion (by the
    gripper or another object) and the image edge are accounted for. Objects with fewer than
    `min_pixels` visible pixels are not reported. `angle_deg` / `angle_conf` / `grasp_width_px`
    give the in-image orientation and grasp width of the object's mask (mask_cross_section)."""
    h, w = geom_ids.shape
    out = []
    for i, (obj, gid) in enumerate(zip(scene.objects, scene.geom_ids)):
        body_ids = getattr(scene, "body_ids", None)
        body_geom_ids = getattr(scene, "body_geom_ids", None)
        if body_ids is not None and body_geom_ids is not None and i < len(body_ids):
            gids = body_geom_ids.get(body_ids[i], [gid])
            ys, xs = np.nonzero(np.isin(geom_ids, gids))
        else:
            ys, xs = np.nonzero(geom_ids == gid)
        if len(xs) < min_pixels:
            continue
        n0, n1 = round(int(xs.min()) / w, 4), round((int(xs.max()) + 1) / w, 4)
        m0, m1 = round(int(ys.min()) / h, 4), round((int(ys.max()) + 1) / h, 4)
        angle, conf, grasp_width = mask_cross_section(xs.astype(np.float64), ys.astype(np.float64))
        out.append({"class_id": i, "format": "xywh_center", "class_name": obj.kind, "name": obj.name,
                    "confidence": 0.99, "x": (n0 + n1) / 2, "y": (m0 + m1) / 2,
                    "w": round(n1 - n0, 4), "h": round(m1 - m0, 4), "pixels": int(len(xs)),
                    "angle_deg": angle, "angle_conf": conf, "grasp_width_px": round(grasp_width, 2)})
    return out


# ---------------------------------------------------------------- monocular depth (emulated)
#
# rebot-vision's depth pipeline (kits/recamera-b601dm/vision/main/depth_geom.cpp) runs a
# monocular model on a 384x224 grid over the 640x360 content and reports per-detection
# disparities and table samples (docs/detection_api.md). The simulator renders the true
# optical-axis depth Z, turns it into the disparity a model would output (DepthModel) and
# computes the same fields from the ground-truth segmentation: it emits what a correct
# rebot-vision would report, without its foreground heuristics.

DEPTH_GRID = (384, 224)
# armd's metric prior (calibrate_depth a_prior): a metric model trained at fx / W = 600 / 640
METRIC_FX_PER_W = 600.0 / 640.0


@dataclass(frozen=True)
class DepthModel:
    """Disparity a depth model outputs for optical depth Z (m). `kind` off / metric / relative.
    metric: disp = scale * (fx / W) / (600 / 640) / Z, times (1 + N(0, noise_rel)) per cell; scale 1
    matches armd's metric prior exactly. relative: disp = a / Z + b (plus the same noise).
    Seeded per (seed, frame_id)."""
    kind: str = "off"
    scale: float = 1.0
    noise_rel: float = 0.0
    a: float = 2.7
    b: float = 0.45
    conf: float = 0.95
    seed: int = 0

    @property
    def enabled(self) -> bool:
        return self.kind != "off"

    @property
    def metric(self) -> bool:
        return self.kind == "metric"

    def disparity(self, z: np.ndarray, frame_id: int = 0, fx_per_w: float | None = None) -> np.ndarray:
        """Disparity for optical depths `z` (non-finite or non-positive z: NaN)."""
        z = np.asarray(z, dtype=np.float64)
        ok = np.isfinite(z) & (z > 0)
        zz = np.where(ok, z, 1.0)
        if self.metric:
            fxw = focal_px(DETECTION_SIZE[1]) / DETECTION_SIZE[0] if fx_per_w is None else fx_per_w
            d = self.scale * fxw / METRIC_FX_PER_W / zz
        else:
            d = self.a / zz + self.b
        if self.noise_rel:
            rng = np.random.default_rng((self.seed, frame_id))
            d = d * (1.0 + rng.normal(0.0, self.noise_rel, d.shape))
        return np.where(ok, d, np.nan)

    def inverse_z(self, disp, fx_per_w: float | None = None):
        """1 / Z of a noise-free disparity (the generator's inverse; tests and diagnostics)."""
        disp = np.asarray(disp, dtype=np.float64)
        if self.metric:
            fxw = focal_px(DETECTION_SIZE[1]) / DETECTION_SIZE[0] if fx_per_w is None else fx_per_w
            return disp * METRIC_FX_PER_W / fxw / self.scale
        return (disp - self.b) / self.a


def render_depth(renderer: mujoco.Renderer) -> np.ndarray:
    """Optical-axis depth Z (m) per pixel of the renderer's current scene (H x W float32, rows top
    down). mujoco.Renderer.render() with depth rendering already returns metric Z (mujoco 4.0.0:
    it maps the depth buffer through znear / zfar); the flags it changes are restored."""
    renderer.enable_depth_rendering()
    try:
        return renderer.render()
    finally:
        renderer.disable_depth_rendering()


def grid_cells(size=DETECTION_SIZE, grid=DEPTH_GRID) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Grid cell centres in content pixels (u, v: 1-D) and the pixel indices they sample (ix, iy)."""
    cw, ch = size[0] / grid[0], size[1] / grid[1]
    u = (np.arange(grid[0]) + 0.5) * cw
    v = (np.arange(grid[1]) + 0.5) * ch
    return u, v, np.minimum(u.astype(int), size[0] - 1), np.minimum(v.astype(int), size[1] - 1)


def _plane_fit(du: np.ndarray, dv: np.ndarray, d: np.ndarray) -> tuple[float, float, float]:
    """Least-squares disp = p0 + pu du + pv dv; median (flat) below 12 cells, as depth_geom.cpp."""
    if len(d) >= 12:
        a = np.column_stack([np.ones_like(du), du, dv])
        sol, *_ = np.linalg.lstsq(a, d, rcond=None)
        return float(sol[0]), float(sol[1]), float(sol[2])
    return float(np.median(d)), 0.0, 0.0


def _mad(x: np.ndarray) -> float:
    return float(np.median(np.abs(x - np.median(x))))


def depth_result(geom_ids: np.ndarray, z: np.ndarray, scene, model: DepthModel, frame_id: int = 0,
                 min_pixels: int = 20) -> dict:
    """Depth fields of a frame: {"table_samples": [[u, v, disp], ...], "objects": {name: fields},
    "finite": bool}. `geom_ids` / `z` at DETECTION_SIZE; fields per object as rebot-vision
    reports them (disp_fg, disp_bg, depth_conf, length_px, jaw_clear, depth_angle_deg,
    depth_angle_conf, depth_width_px), computed on the 384x224 grid from the ground-truth mask:
    - foreground: the object's cells (none reported under 6 cells);
    - table plane: least-squares plane over table cells in the ring box expanded by
      max(25 %, 4 cells) minus the box; disp_bg = plane at the foreground centroid, disp_fg =
      disp_bg + P75(foreground disp - plane);
    - length_px: extent of the full-resolution mask along its long axis (mask_orientation);
    - depth_angle_deg / depth_width_px: the full-resolution mask's cross section
      (mask_cross_section, as angle_deg / grasp_width_px); depth_angle_conf = its angle_conf x
      depth_conf (as rebot-vision scales the PCA confidence by depth_conf);
    - jaw_clear: simplified, False when another object's pixels lie in the box expanded by 18 px.
    Table samples as sample_table_disparity: 8 x 6 patches of 5 x 5 cells over [8 %, 92 %], table
    cells outside every object box expanded by 15 %, >= 9 cells, MAD <= 0.05 median, coordinates the
    mean of the retained cells."""
    w, h = DETECTION_SIZE
    gw, gh = DEPTH_GRID
    cw, ch = w / gw, h / gh
    u, v, ix, iy = grid_cells()
    ids_g = geom_ids[np.ix_(iy, ix)]
    z_g = z[np.ix_(iy, ix)].astype(np.float64)
    finite = bool(np.isfinite(z_g).all() and (z_g > 0).all())
    disp = model.disparity(z_g, frame_id)
    uu, vv = np.meshgrid(u, v)
    table = (ids_g == scene.table_id) & np.isfinite(disp)

    boxes: dict[str, tuple[float, float, float, float]] = {}
    masks: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for obj, gid in zip(scene.objects, scene.geom_ids):
        ys, xs = np.nonzero(geom_ids == gid)
        if len(xs) < min_pixels:
            continue
        boxes[obj.name] = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))
        masks[obj.name] = (xs, ys)

    objects: dict[str, dict] = {}
    for obj, gid in zip(scene.objects, scene.geom_ids):
        if obj.name not in boxes:
            continue
        x0, y0, x1, y1 = boxes[obj.name]
        fg = (ids_g == gid) & np.isfinite(disp)
        if fg.sum() < 6:
            continue
        mx, my = max(0.25 * (x1 - x0), 4 * cw), max(0.25 * (y1 - y0), 4 * ch)
        in_ring = (uu >= x0 - mx) & (uu <= x1 + mx) & (vv >= y0 - my) & (vv <= y1 + my)
        in_box = (uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)
        ring = in_ring & ~in_box & table
        if not ring.any():
            continue
        uc, vc = (x0 + x1) / 2, (y0 + y1) / 2
        p0, pu, pv = _plane_fit(uu[ring] - uc, vv[ring] - vc, disp[ring])
        fu, fv, fd = uu[fg], vv[fg], disp[fg]
        deltas = fd - (p0 + pu * (fu - uc) + pv * (fv - vc))
        disp_bg = max(0.0, p0 + pu * (fu.mean() - uc) + pv * (fv.mean() - vc))
        disp_fg = disp_bg + float(np.percentile(deltas, 75))
        xs, ys = masks[obj.name]
        angle, _ = mask_orientation(xs, ys)
        t = math.radians(angle)
        proj = xs * math.cos(t) + ys * math.sin(t)
        length = float(proj.max() - proj.min() + abs(math.cos(t)) + abs(math.sin(t)))
        clear = True
        for other, (ox, oy) in masks.items():
            if other != obj.name and ((ox >= x0 - 18) & (ox < x1 + 18) & (oy >= y0 - 18) & (oy < y1 + 18)).any():
                clear = False
        d_angle, d_conf, d_width = mask_cross_section(xs.astype(np.float64), ys.astype(np.float64))
        objects[obj.name] = {"disp_fg": round(disp_fg, 5), "disp_bg": round(disp_bg, 5), "depth_conf": model.conf,
                             "length_px": round(length, 2), "jaw_clear": clear, "depth_angle_deg": d_angle,
                             "depth_angle_conf": round(d_conf * model.conf, 3), "depth_width_px": round(d_width, 2)}

    excl = np.zeros_like(table)
    for x0, y0, x1, y1 in boxes.values():
        bw, bh = x1 - x0, y1 - y0
        excl |= (uu >= x0 - 0.15 * bw) & (uu <= x1 + 0.15 * bw) & (vv >= y0 - 0.15 * bh) & (vv <= y1 + 0.15 * bh)
    usable = table & ~excl
    samples = []
    for jy in range(6):
        cgy = int(round((0.08 + 0.84 * jy / 5) * (gh - 1)))
        for jx in range(8):
            cgx = int(round((0.08 + 0.84 * jx / 7) * (gw - 1)))
            sl = (slice(max(cgy - 2, 0), cgy + 3), slice(max(cgx - 2, 0), cgx + 3))
            keep = usable[sl]
            if keep.sum() < 9:
                continue
            d = disp[sl][keep]
            med = float(np.median(d))
            if not math.isfinite(med) or _mad(d) > 0.05 * max(med, 1e-3):
                continue
            samples.append([round(float(uu[sl][keep].mean()), 2), round(float(vv[sl][keep].mean()), 2), round(med, 5)])
    return {"table_samples": samples, "objects": objects, "finite": finite}


@dataclass(frozen=True)
class DetNoise:
    """Detector-like errors added to the ground-truth detections (all off by default).
    Pixels refer to DETECTION_SIZE, the size the perception API reports."""
    jitter_px: float = 0.0  # Gaussian sigma on the box centre x and y
    size_jitter: float = 0.0  # w and h are multiplied by (1 + N(0, size_jitter)) independently
    drop_rate: float = 0.0  # probability to drop a detection, per frame
    latency_ms: float = 0.0  # detections published this long after the frame was captured
    false_rate: float = 0.0  # probability per frame of one false detection (score 0.3..0.6)
    angle_jitter_deg: float = 0.0  # Gaussian sigma on angle_deg
    seed: int = 0

    @property
    def perturbs(self) -> bool:
        return bool(self.jitter_px or self.size_jitter or self.drop_rate or self.false_rate or self.angle_jitter_deg)


def perturb(dets: list[dict], cfg: DetNoise, frame_id: int, labels: list[tuple[int, str]] | None = None) -> list[dict]:
    """Detections as a noisy detector would report them. Without noise (cfg.perturbs False) `dets`
    is returned unchanged. The random stream is seeded by (cfg.seed, frame_id), so a frame gets the
    same errors on every run. `labels` are the (class_id, class_name) a false detection may carry
    (default: those of `dets`). Boxes are clipped to the image; degenerate boxes (under 2 px) are dropped."""
    if not cfg.perturbs:
        return dets
    rng = np.random.default_rng((cfg.seed, frame_id))
    w_img, h_img = DETECTION_SIZE
    out = []
    for d in dets:
        if cfg.drop_rate and rng.random() < cfg.drop_rate:
            continue
        cx, cy, bw, bh = d["x"], d["y"], d["w"], d["h"]
        if cfg.jitter_px:
            cx += rng.normal(0.0, cfg.jitter_px) / w_img
            cy += rng.normal(0.0, cfg.jitter_px) / h_img
        if cfg.size_jitter:
            bw *= 1.0 + rng.normal(0.0, cfg.size_jitter)
            bh *= 1.0 + rng.normal(0.0, cfg.size_jitter)
        n = dict(d)
        if cfg.size_jitter and "grasp_width_px" in d:
            n["grasp_width_px"] = round(float(d["grasp_width_px"] * (1.0 + rng.normal(0.0, cfg.size_jitter))), 2)
        if cfg.angle_jitter_deg and "angle_deg" in d:
            a = d["angle_deg"] + rng.normal(0.0, cfg.angle_jitter_deg)
            n["angle_deg"] = round((a + 90.0) % 180.0 - 90.0, 2)
        if _set_box(n, cx, cy, bw, bh):
            out.append(n)
    if cfg.false_rate and rng.random() < cfg.false_rate:
        pool = labels or sorted({(d["class_id"], d["class_name"]) for d in dets})
        if pool:
            class_id, kind = pool[int(rng.integers(len(pool)))]
            bw, bh = rng.uniform(0.04, 0.2), rng.uniform(0.06, 0.3)
            gw = round(float(min(bw * w_img, bh * h_img) * rng.uniform(0.7, 1.0)), 2)
            f = {"class_id": class_id, "format": "xywh_center", "class_name": kind, "name": f"false_{kind}",
                 "confidence": round(float(rng.uniform(0.3, 0.6)), 3), "pixels": int(bw * bh * w_img * h_img),
                 "angle_deg": round(float(rng.uniform(-90.0, 90.0)), 2), "angle_conf": 0.0,
                 "grasp_width_px": gw}
            if _set_box(f, rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), bw, bh):
                out.append(f)
    return out


def _set_box(d: dict, cx: float, cy: float, bw: float, bh: float) -> bool:
    """Write the box (centre / size, normalised) into d clipped to the image; False when it is degenerate."""
    x0, x1 = max(0.0, cx - bw / 2), min(1.0, cx + bw / 2)
    y0, y1 = max(0.0, cy - bh / 2), min(1.0, cy + bh / 2)
    if (x1 - x0) * DETECTION_SIZE[0] < 2.0 or (y1 - y0) * DETECTION_SIZE[1] < 2.0:
        return False
    d.update(x=round((x0 + x1) / 2, 4), y=round((y0 + y1) / 2, 4), w=round(x1 - x0, 4), h=round(y1 - y0, 4))
    return True


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
