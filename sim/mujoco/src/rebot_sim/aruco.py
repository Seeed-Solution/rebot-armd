"""Virtual ArUco GridBoard on the table and rebot-vision's `aruco` output for it (numpy only).

Board (calibration wizard, W2): 3 columns x 4 rows of aruco4x4_50 markers, ids 0..11 in reading
order (id = row * 3 + col, row 0 at the top of the printed page, col 0 at the left), marker side
`side_m` (default 0.05 m), gap 0.2 * side (10 mm). Board frame (= the page): origin at the board
centre, x to the right of the page, y down the page, z into the table (x, y, z right-handed).
On the table with yaw 0 the page's x is base +x and its y is base -y (the "up" arrow points to
base +y); `yaw` turns the board about base z.

Each marker's corners follow rebot-vision (AprilTag order): marker frame x right, y down, z
into the marker; corner k is at side/2 * [(-1, 1), (1, 1), (1, -1), (-1, -1)][k] from the
marker centre (bottom-left, bottom-right, top-right, top-left as printed). Board-frame corner
coordinates: `board_object_points()`.

`marker_pose` computes `pose` as rebot-vision does from the (raw) corners with the intrinsics it
holds: undistorted corners, homography to the marker square, R | t (marker frame in the
camera frame, m), `r_deg` = roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

COLS, ROWS = 3, 4
IDS = tuple(range(COLS * ROWS))
CORNER_SIGNS = np.array([(-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)])
SIZE = (640, 360)


@dataclass(frozen=True)
class Board:
    x: float
    y: float
    yaw_deg: float = 0.0
    z: float = 0.0
    side_m: float = 0.05

    @property
    def gap_m(self) -> float:
        return 0.2 * self.side_m

    @classmethod
    def parse(cls, text: str, z: float = 0.0, side_m: float = 0.05) -> "Board":
        v = [float(t) for t in text.split(",")]
        if len(v) not in (2, 3) or not all(math.isfinite(t) for t in v):
            raise ValueError("--aruco-board needs X,Y[,YAW_DEG]")
        return cls(v[0], v[1], v[2] if len(v) == 3 else 0.0, z, side_m)

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(R_base_board, origin) of the board frame."""
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        x, z = np.array([c, s, 0.0]), np.array([0.0, 0.0, -1.0])
        return np.column_stack([x, np.cross(z, x), z]), np.array([self.x, self.y, self.z])


def board_object_points(side_m: float = 0.05) -> dict[int, np.ndarray]:
    """Marker id -> its 4 corners (4 x 3) in the board frame (z = 0)."""
    gap, pitch = 0.2 * side_m, 1.2 * side_m
    w, h = COLS * side_m + (COLS - 1) * gap, ROWS * side_m + (ROWS - 1) * gap
    out = {}
    for r in range(ROWS):
        for c in range(COLS):
            centre = np.array([-w / 2 + c * pitch + side_m / 2, -h / 2 + r * pitch + side_m / 2])
            xy = centre + CORNER_SIGNS * side_m / 2
            out[r * COLS + c] = np.column_stack([xy, np.zeros(4)])
    return out


def board_world_points(board: Board) -> dict[int, np.ndarray]:
    r, o = board.pose()
    return {i: p @ r.T + o for i, p in board_object_points(board.side_m).items()}


def project(points: np.ndarray, r_world_cam: np.ndarray, origin: np.ndarray, fx: float, fy: float,
            cx: float, cy: float) -> tuple[np.ndarray, np.ndarray]:
    """Pinhole (no distortion) pixels and depths of world points; r_world_cam has the OpenCV
    camera axes (x right, y down, z optical) as columns."""
    pc = (np.asarray(points, float) - origin) @ r_world_cam
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = np.column_stack([fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy])
    return uv, pc[:, 2]


def visible_markers(board: Board, r_world_cam: np.ndarray, origin: np.ndarray, f: float,
                    noise_px: float = 0.0, seed: int = 0, frame_id: int = 0, size=SIZE) -> list[dict]:
    """Markers whose 4 corners are in front of the camera and inside the image, with Gaussian
    corner noise (sigma `noise_px`, seeded per (seed, frame_id)): [{id, corners (4 x 2 px)}]."""
    rng = np.random.default_rng((seed, frame_id)) if noise_px > 0 else None
    out = []
    for mid, pts in board_world_points(board).items():
        uv, depth = project(pts, r_world_cam, origin, f, f, size[0] / 2, size[1] / 2)
        if not (np.all(depth > 0.01) and np.all(np.isfinite(uv))):
            continue
        if rng is not None:
            uv = uv + rng.normal(0.0, noise_px, uv.shape)
        if np.all((uv[:, 0] >= 0) & (uv[:, 0] <= size[0]) & (uv[:, 1] >= 0) & (uv[:, 1] <= size[1])):
            out.append({"id": mid, "corners": uv})
    return out


def undistort_normalised(uv: np.ndarray, k: dict) -> np.ndarray:
    """Pixels -> ideal normalised coordinates with OpenCV's radial-tangential model (k1 k2 p1 p2 k3)."""
    x = (uv[:, 0] - k["cx"]) / k["fx"]
    y = (uv[:, 1] - k["cy"]) / k["fy"]
    k1, k2, p1, p2, k3 = (float(k.get(n, 0.0)) for n in ("k1", "k2", "p1", "p2", "k3"))
    if not any((k1, k2, p1, p2, k3)):
        return np.column_stack([x, y])
    xd, yd = x.copy(), y.copy()
    for _ in range(20):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = (xd - dx) / rad, (yd - dy) / rad
    return np.column_stack([x, y])


def homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """DLT homography (3 x 3) with dst ~ H src for 4+ point pairs."""
    a = []
    for (x, y), (u, v) in zip(src, dst):
        a.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        a.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.asarray(a, float))
    return vt[-1].reshape(3, 3)


def rot_to_euler(r: np.ndarray) -> tuple[float, float, float]:
    """(roll, pitch, yaw) rad of R = Rz(yaw) Ry(pitch) Rx(roll) (rebot-vision rot_to_euler)."""
    pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
    if abs(r[2, 0]) < 0.999999:
        return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])
    return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0


def marker_pose(corners_px: np.ndarray, intr: dict, size_m: float) -> dict:
    """{t, r_deg, error} of one marker from its raw corners with the intrinsics `intr`
    (fx, fy, cx, cy, k1..k3). `error`: mean squared reprojection error of the corners in
    normalised image units (rebot-vision reports its pose solver's object-space error)."""
    n = undistort_normalised(np.asarray(corners_px, float), intr)
    obj = CORNER_SIGNS * size_m / 2
    h = homography(obj, n)
    lam = 2.0 / (np.linalg.norm(h[:, 0]) + np.linalg.norm(h[:, 1]))
    if h[2, 2] * lam < 0:
        lam = -lam
    r1, r2, t = h[:, 0] * lam, h[:, 1] * lam, h[:, 2] * lam
    u, _, vt = np.linalg.svd(np.column_stack([r1, r2, np.cross(r1, r2)]))
    r = u @ vt
    if np.linalg.det(r) < 0:
        r = u @ np.diag([1.0, 1.0, -1.0]) @ vt
    pc = np.column_stack([obj, np.zeros(4)]) @ r.T + t
    err = float(np.mean(np.sum((pc[:, :2] / pc[:, 2:3] - n) ** 2, axis=1)))
    roll, pitch, yaw = rot_to_euler(r)
    return {"t": [round(float(v), 4) for v in t], "r_deg": [round(math.degrees(a), 2) for a in (roll, pitch, yaw)],
            "error": round(err, 8)}


def marker_json(m: dict, intr: dict, size_m: float) -> dict:
    """One `markers` entry as rebot-vision writes it (pixels at 640 x 360)."""
    c = np.asarray(m["corners"], float)
    return {"id": int(m["id"]), "margin": 100.0, "center": [round(float(v), 1) for v in c.mean(axis=0)],
            "corners": [[round(float(u), 2), round(float(v), 2)] for u, v in c],
            "pose": marker_pose(c, intr, size_m)}


# ---- intrinsics API (rebot-vision W7 contract; X8: no loopback rule in the simulator) ----

INTRINSIC_KEYS = ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")
READ_ONLY = ("revision", "overridden_by_cli", "config_path", "api_version")


class IntrinsicsError(ValueError):
    def __init__(self, status: int, code: str, message: str, field: str = ""):
        super().__init__(message)
        self.status, self.code, self.field = status, code, field


def check_intrinsics_put(body: dict, current: dict, revision: int) -> dict:
    """Validated new intrinsics for a PUT body (rebot-vision rules); raises IntrinsicsError."""
    if not isinstance(body, dict):
        raise IntrinsicsError(400, "INVALID_INTRINSICS", "body must be a JSON object")
    known = set(INTRINSIC_KEYS) | {"width", "height", "marker_size", "expected_revision"} | set(READ_ONLY)
    for k in body:
        if k not in known:
            raise IntrinsicsError(400, "INVALID_INTRINSICS", f'unknown field "{k}"', k)

    def num(k):
        v = body.get(k)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise IntrinsicsError(400, "INVALID_INTRINSICS", f'"{k}" must be a finite number', k)
        return float(v)

    out = {k: num(k) for k in INTRINSIC_KEYS}
    w, h = num("width"), num("height")
    if w != 640:
        raise IntrinsicsError(400, "INVALID_INTRINSICS", '"width" must be 640', "width")
    if h != 360:
        raise IntrinsicsError(400, "INVALID_INTRINSICS", '"height" must be 360', "height")
    rules = {"fx": (100, 5000), "fy": (100, 5000), "k1": (-10, 10), "k2": (-10, 10), "k3": (-10, 10), "p1": (-1, 1), "p2": (-1, 1)}
    for k, (lo, hi) in rules.items():
        if not lo <= out[k] <= hi:
            raise IntrinsicsError(400, "INVALID_INTRINSICS", f'"{k}" must be in [{lo}, {hi}]', k)
    if not 0 < out["cx"] < w:
        raise IntrinsicsError(400, "INVALID_INTRINSICS", '"cx" must be in (0, 640)', "cx")
    if not 0 < out["cy"] < h:
        raise IntrinsicsError(400, "INVALID_INTRINSICS", '"cy" must be in (0, 360)', "cy")
    out["marker_size"] = current.get("marker_size", 0.05)
    if "marker_size" in body:
        ms = num("marker_size")
        if not 0 < ms <= 1:
            raise IntrinsicsError(400, "INVALID_INTRINSICS", '"marker_size" must be in (0, 1] m', "marker_size")
        out["marker_size"] = ms
    er = body.get("expected_revision")
    if isinstance(er, bool) or not isinstance(er, int) or er < 0:
        raise IntrinsicsError(400, "INVALID_INTRINSICS", '"expected_revision" must be an unsigned integer', "expected_revision")
    if er != revision:
        raise IntrinsicsError(409, "REVISION_MISMATCH", f"expected_revision {er} is not the current revision {revision}")
    return out
