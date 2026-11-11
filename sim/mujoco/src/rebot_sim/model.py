"""B601-DM scene loading and the armd motor <-> MuJoCo joint mapping."""
from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[2] / "models"
SCENE_XML = MODEL_DIR / "rebot_b601dm_scene.xml"

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
DOF = 7
# Jaw opening per side (m) per rad of J7: 85 mm clear span at the profile's open_rad -5.7.
JAW_M_PER_RAD = 0.0425 / 5.7

# Upstream link meshes (reBot-DevArm, DM/meshes/collision) attached as visuals when
# REBOT_DEVARM_DIR is set; their frames are the link frames.
_UPSTREAM_MESHES = {
    "base_link": "base_link.STL", "link1": "link1.STL", "link2": "link2.STL",
    "link3": "link3.STL", "link4": "link4.STL", "link5": "link5.STL",
    "link6": "link6.STL", "end_link": "gripper_base.stl",
    "finger_left": "left_finger.stl", "finger_right": "right_finger.stl",
}
# With the upstream meshes, the box primitives of the arm move to this geom group, which the
# camera renderer hides (the boxes are thicker than the tapered jaws; physics is unchanged).
BOX_GROUP = 3


def load_model(devarm_dir: str | None = None, objects=None, camera=None) -> mujoco.MjModel:
    """Compile the scene; with a reBot-DevArm checkout, add its link meshes as visuals.

    `objects` (list of `scene.SceneObject`) replaces the XML's cube; `camera` (a
    `camera.CameraMount`) adds the `wrist` and `scene` cameras.
    """
    devarm_dir = devarm_dir if devarm_dir is not None else os.environ.get("REBOT_DEVARM_DIR")
    if not devarm_dir and objects is None and camera is None:
        return mujoco.MjModel.from_xml_path(str(SCENE_XML))
    spec = mujoco.MjSpec.from_file(str(SCENE_XML))
    if devarm_dir:
        mesh_dir = Path(devarm_dir) / "Rebot_Arm_description" / "DM" / "meshes" / "collision"
        for body_name, file in _UPSTREAM_MESHES.items():
            path = mesh_dir / file
            if not path.exists():
                raise FileNotFoundError(f"REBOT_DEVARM_DIR: {path} not found")
            mesh = spec.add_mesh()
            mesh.name, mesh.file = f"{body_name}_mesh", str(path)
            geom = spec.body(body_name).add_geom()
            geom.type, geom.meshname = mujoco.mjtGeom.mjGEOM_MESH, mesh.name
            geom.contype = geom.conaffinity = 0
            geom.group = 2
            geom.rgba = [0.78, 0.80, 0.78, 1.0]
        for geom in spec.geoms:
            if geom.contype == 2 or geom.name in ("palm", "motor_housing") or geom.name.split("_")[0] in ("left", "right"):
                geom.group = BOX_GROUP
    if objects is not None:
        from .scene import add_objects
        add_objects(spec, objects)
    if camera is not None:
        from .camera import add_cameras, mount_rotation
        add_cameras(spec, mount_rotation(spec.compile(), camera), camera.offset)
    return spec.compile()


class Indexer:
    """qpos / qvel addresses of the arm joints and the left jaw slide."""

    def __init__(self, model: mujoco.MjModel):
        joints = ARM_JOINTS + ["finger_left"]
        self.qpos = np.array([model.jnt_qposadr[model.joint(j).id] for j in joints])
        self.qvel = np.array([model.jnt_dofadr[model.joint(j).id] for j in joints])
        self.right_qpos = model.jnt_qposadr[model.joint("finger_right").id]

    def motor_positions(self, data: mujoco.MjData) -> np.ndarray:
        """armd motor angles J1..J7 (rad)."""
        q = data.qpos[self.qpos].copy()
        q[6] = -q[6] / JAW_M_PER_RAD
        return q

    def motor_velocities(self, data: mujoco.MjData) -> np.ndarray:
        dq = data.qvel[self.qvel].copy()
        dq[6] = -dq[6] / JAW_M_PER_RAD
        return dq

    def set_motor_positions(self, data: mujoco.MjData, q) -> None:
        q = np.asarray(q, dtype=float)
        data.qpos[self.qpos[:6]] = q[:6]
        if len(q) > 6:
            jaw = -q[6] * JAW_M_PER_RAD
            data.qpos[self.qpos[6]] = jaw
            data.qpos[self.right_qpos] = jaw
