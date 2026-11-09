"""Render the v5 clip-on body, removable top bar, B3 CAN cover, and gripper."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
XML = REPO / "Rebot_Arm_description/RS/mjcf/rs_arm.xml"
MESH_ROOT = REPO / "Rebot_Arm_description/RS/meshes"
OUT = HERE / "assembly_preview.png"

MOUNT_R = np.array(
    [
        [0.019295, 0.011947, 0.999742],
        [0.999811, 0.002170, -0.019322],
        [-0.0024, 0.999926, -0.011903],
    ]
)
MOUNT_P_MM = np.array([-120.1, 0.3, 45.0])


def mesh_file_map(root: ET.Element) -> dict[str, Path]:
    return {m.attrib["name"]: MESH_ROOT / m.attrib["file"] for m in root.findall("./asset/mesh")}


def quat_matrix(q: list[float]) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    p = np.fromstring(element.attrib.get("pos", "0 0 0"), sep=" ") * 1000.0
    q = np.fromstring(element.attrib.get("quat", "1 0 0 0"), sep=" ").tolist()
    return p, quat_matrix(q)


def compose(parent: tuple[np.ndarray, np.ndarray], child: tuple[np.ndarray, np.ndarray]):
    pp, pr = parent
    cp, cr = child
    return pp + pr @ cp, pr @ cr


def load_gripper_meshes() -> list[tuple[str, trimesh.Trimesh, np.ndarray, np.ndarray, str]]:
    root = ET.parse(XML).getroot()
    files = mesh_file_map(root)
    wanted = {"pla7_green", "cnc7", "motor_7", "pla_left", "cnc_left", "pla_right", "cnc_right"}
    end = root.find(".//body[@name='gripper_end']")
    if end is None:
        raise RuntimeError("gripper_end body not found")
    result = []
    end_pose = (np.zeros(3), np.eye(3))
    for geom in end.findall("./geom"):
        name = geom.attrib.get("mesh")
        if name in wanted:
            p, r = compose(end_pose, pose(geom))
            result.append((name, trimesh.load_mesh(files[name], process=False), p, r, "arm"))
    for side in ("gripper_left", "gripper_right"):
        body = end.find(f"./body[@name='{side}']")
        if body is None:
            continue
        body_pose = compose(end_pose, pose(body))
        for geom in body.findall("./geom"):
            name = geom.attrib.get("mesh")
            if name in {"pla_left", "cnc_left", "pla_right", "cnc_right"}:
                p, r = compose(body_pose, pose(geom))
                result.append((name, trimesh.load_mesh(files[name], process=False), p, r, "arm"))
    return result


def apply_trimesh(mesh: trimesh.Trimesh, p: np.ndarray, r: np.ndarray) -> trimesh.Trimesh:
    out = mesh.copy()
    out.apply_scale(1000.0)
    out.apply_transform(np.block([[r, p[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]))
    return out


def shape_triangles(shape, transform=None):
    vertices, faces = shape.tessellate(0.35, 0.5)
    pts = np.array([[v.X, v.Y, v.Z] for v in vertices], dtype=float)
    if transform is not None:
        pts = (transform[:3, :3] @ pts.T).T + transform[:3, 3]
    return [[pts[i] for i in face] for face in faces]


def translate_faces(faces, delta: np.ndarray):
    return [[np.asarray(p) + delta for p in face] for face in faces]


def side_faces(faces):
    return [[(p[0], p[2]) for p in face] for face in faces]


def add_3d(ax, faces, color, alpha=0.9, linewidths=0.05):
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, alpha=alpha, linewidths=linewidths))


def add_side(ax, faces, color, alpha=0.9, edgecolors="none"):
    ax.add_collection(PolyCollection(side_faces(faces), facecolors=color, alpha=alpha, edgecolors=edgecolors, linewidths=0.25))


def limits(points):
    xyz = np.vstack([p for faces in points for face in faces for p in face])
    mins, maxs = xyz.min(axis=0), xyz.max(axis=0)
    center = (mins + maxs) / 2
    radius = max(maxs - mins) / 2
    return mins, maxs, center, radius


def style_3d(ax, center, radius, title):
    ax.set_title(title)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=22, azim=-58)


def main() -> None:
    sys.path.insert(0, str(HERE))
    import build123d as bd
    from generate_recamera_mount import CAM_DEPTH, CAM_H, CAM_W, FRONT_Y1, GUARD_Y0, TOP_Z0, build_parts

    # v5 interface: body, removable top bar, preserved interface, camera,
    # already-aligned CAN cover, top-bar hole tools, cradle location.
    body, top, _, camera, can, _, cradle_location = build_parts()
    mount_tf = np.eye(4)
    mount_tf[:3, :3] = MOUNT_R
    mount_tf[:3, 3] = MOUNT_P_MM

    body_faces = shape_triangles(body, mount_tf)
    top_faces = shape_triangles(top, mount_tf)
    camera_faces = shape_triangles(camera, mount_tf)
    can_faces = shape_triangles(can, mount_tf)

    # Cyan volumes mark the intentionally open CAN/USB side interface zones.
    keepout_left = bd.Box(8.0, 11.7, CAM_H, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        cradle_location * bd.Location((-CAM_W / 2 - 4.0, 3.5, 4.0))
    )
    keepout_right = bd.Box(8.0, 11.7, CAM_H, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).located(
        cradle_location * bd.Location((CAM_W / 2 + 4.0, 3.5, 4.0))
    )
    keepout_faces = [shape_triangles(keepout_left, mount_tf), shape_triangles(keepout_right, mount_tf)]

    axis_a = cradle_location * bd.Location((0, 3.5 + CAM_DEPTH, 4.0 + CAM_H / 2))
    axis_b = cradle_location * bd.Location((0, 3.5 + CAM_DEPTH + 18, 4.0 + CAM_H / 2))
    lens = MOUNT_P_MM + MOUNT_R @ np.array([axis_a.position.X, axis_a.position.Y, axis_a.position.Z])
    axis_tip = MOUNT_P_MM + MOUNT_R @ np.array([axis_b.position.X, axis_b.position.Y, axis_b.position.Z])

    gripper = [apply_trimesh(mesh, p, r) for _, mesh, p, r, _ in load_gripper_meshes()]
    gripper_side = [m.triangles[:, :, [0, 2]] for m in gripper]
    # Lift along the actual cradle local Z axis, then map that vector into the
    # gripper frame.  This is intentionally not a CAD-global-Z translation.
    lift_local = cradle_location * bd.Location((0.0, 0.0, 16.0))
    local_z_delta = np.array([
        lift_local.position.X - cradle_location.position.X,
        lift_local.position.Y - cradle_location.position.Y,
        lift_local.position.Z - cradle_location.position.Z,
    ])
    exploded_delta = MOUNT_R @ local_z_delta
    top_exploded_faces = translate_faces(top_faces, exploded_delta)
    arrow_start = np.mean(np.vstack([p for f in top_faces for p in f]), axis=0)
    arrow_end = arrow_start + exploded_delta
    assembly_points = [body_faces, top_faces, camera_faces, can_faces] + keepout_faces + [m.triangles.tolist() for m in gripper]
    _, _, center, radius = limits(assembly_points)

    fig = plt.figure(figsize=(18, 11), dpi=180)
    side = fig.add_subplot(2, 2, 1)
    add_side(side, body_faces, "#1976d2", 0.9)
    add_side(side, top_faces, "#a855f7", 0.95)
    add_side(side, camera_faces, "#f59e0b", 0.18)
    add_side(side, can_faces, "#22a447", 0.9)
    for keepout in keepout_faces:
        add_side(side, keepout, "#06b6d4", 0.22, "#0891b2")
    for mesh in gripper_side:
        side.add_collection(PolyCollection(mesh, facecolors="#777777", alpha=0.72, linewidths=0))
    side.plot([lens[0], axis_tip[0]], [lens[2], axis_tip[2]], color="#dc2626", lw=2)
    side.set_title("Assembly: body + top pressure bar + CAN cover")
    side.set_xlabel("gripper X (mm)"); side.set_ylabel("gripper Z (mm)")
    side.set_aspect("equal"); side.set_xlim(center[0] - radius - 15, center[0] + radius + 15); side.set_ylim(center[2] - radius - 15, center[2] + radius + 15); side.grid(alpha=0.2)
    side.text(0.02, 0.03, "camera rolled 90°; red = optical axis forward; cyan = reserved side space", transform=side.transAxes, fontsize=8)

    perspective = fig.add_subplot(2, 2, 2, projection="3d")
    add_3d(perspective, body_faces, "#1976d2")
    add_3d(perspective, top_faces, "#a855f7")
    add_3d(perspective, camera_faces, "#f59e0b", 0.18)
    add_3d(perspective, can_faces, "#22a447")
    for keepout in keepout_faces:
        add_3d(perspective, keepout, "#06b6d4", 0.22)
    for mesh in gripper:
        add_3d(perspective, mesh.triangles, "#777777", 0.72)
    perspective.plot([lens[0], axis_tip[0]], [lens[1], axis_tip[1]], [lens[2], axis_tip[2]], color="#dc2626", lw=2)
    style_3d(perspective, center, radius, "Assembly perspective")

    exploded = fig.add_subplot(2, 2, 3, projection="3d")
    add_3d(exploded, body_faces, "#1976d2")
    add_3d(exploded, camera_faces, "#f59e0b", 0.14)
    add_3d(exploded, can_faces, "#22a447")
    for mesh in gripper:
        add_3d(exploded, mesh.triangles, "#777777", 0.55)
    add_3d(exploded, top_exploded_faces, "#a855f7", 0.98)
    exploded.quiver(arrow_start[0], arrow_start[1], arrow_start[2], exploded_delta[0], exploded_delta[1], exploded_delta[2], color="#7e22ce", arrow_length_ratio=0.18, linewidth=2)
    style_3d(exploded, center, radius, "Remove/install top bar: lift vertically")

    detail = fig.add_subplot(2, 2, 4)
    add_side(detail, can_faces, "#22a447", 0.9)
    add_side(detail, body_faces, "#1976d2", 0.22)
    add_side(detail, top_exploded_faces, "#a855f7", 0.88)
    for keepout in keepout_faces:
        add_side(detail, keepout, "#06b6d4", 0.25, "#0891b2")
    detail.annotate("lift off / install", xy=(arrow_end[0], arrow_end[2]), xytext=(arrow_start[0], arrow_start[2]), arrowprops={"arrowstyle": "->", "color": "#7e22ce", "lw": 2}, color="#7e22ce", ha="center")
    cover_xyz = np.asarray([p for f in can_faces for p in f], dtype=float).reshape(-1, 3)
    cover_center = cover_xyz.mean(axis=0)
    detail.annotate("rear-cover side insertion channel", xy=(cover_xyz[:, 0].min(), cover_center[2]),
                    xytext=(cover_xyz[:, 0].min() - 28, cover_center[2] + 18),
                    arrowprops={"arrowstyle": "->", "color": "#ea580c", "lw": 1.8}, color="#c2410c", fontsize=8, ha="right")
    detail.annotate("rear-cover upper insertion channel", xy=(cover_center[0], cover_xyz[:, 2].max()),
                    xytext=(cover_center[0] - 25, cover_xyz[:, 2].max() + 18),
                    arrowprops={"arrowstyle": "->", "color": "#ea580c", "lw": 1.8}, color="#c2410c", fontsize=8, ha="left")
    detail.set_title("Removal direction + side interface clearance")
    detail.set_xlabel("gripper X (mm)"); detail.set_ylabel("gripper Z (mm)")
    detail.set_aspect("equal"); detail.set_xlim(cover_xyz[:, 0].min() - 35, cover_xyz[:, 0].max() + 35); detail.set_ylim(cover_xyz[:, 2].min() - 28, cover_xyz[:, 2].max() + 28); detail.grid(alpha=0.2)
    detail.text(0.02, 0.03, "orange = rear-cover insertion channels; cyan = reserved side space; purple = removable bar", transform=detail.transAxes, fontsize=8)

    fig.suptitle("reCamera B601 v5 — camera rolled 90°, optical axis forward; clip-on body, removable top pressure bar")
    fig.tight_layout(); fig.savefig(OUT, bbox_inches="tight")
    print(f"preview: {OUT}")


if __name__ == "__main__":
    main()
