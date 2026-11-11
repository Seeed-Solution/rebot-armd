"""Graspable scene objects: kinds, placement by seed, and the success query.

Each object is a free body with one geom, both named after the object (`cube`, `cup`,
`cube_2`, ...); its detection label is the kind. Without a seed the objects take the fixed
default layout (the first one at (0.30, 0) m, where the scene XML had its cube); with a seed
the poses are drawn from `numpy.random.default_rng(seed)`, so a seed always gives the same
layout.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

# kind: geom type, size (MuJoCo half-sizes, m), mass (kg), rgba
KINDS = {
    "cube": (mujoco.mjtGeom.mjGEOM_BOX, (0.02, 0.02, 0.02), 0.03, (0.92, 0.18, 0.12, 1)),
    "cup": (mujoco.mjtGeom.mjGEOM_CYLINDER, (0.025, 0.035, 0.0), 0.05, (0.15, 0.40, 0.85, 1)),
    "block": (mujoco.mjtGeom.mjGEOM_BOX, (0.03, 0.015, 0.02), 0.04, (0.20, 0.70, 0.25, 1)),
}
MAX_OBJECTS = 3
DEFAULT_XY = [(0.30, 0.0), (0.24, 0.12), (0.24, -0.12)]
# seeded placement region (base frame, m): in front of the arm, inside the reach at the
# approach height with the tool pointing down
X_RANGE, Y_RANGE = (0.22, 0.36), (-0.14, 0.14)
MIN_SEPARATION_M = 0.09
LIFTED_M = 0.02  # an object counts as lifted this far above its resting height
JAW_GEOMS = ("left_front", "left_mid", "left_rear", "right_front", "right_mid", "right_rear")


@dataclass(frozen=True)
class SceneObject:
    name: str
    kind: str

    @property
    def rest_z(self) -> float:
        """Centre height when the object stands on the table (z = 0)."""
        return KINDS[self.kind][1][2] if self.kind != "cup" else KINDS["cup"][1][1]


def parse_objects(text: str) -> list[SceneObject]:
    """`cube,cup` -> objects named cube, cup; repeated kinds get `_2`, `_3`."""
    kinds = [k.strip() for k in text.split(",") if k.strip()]
    if not 1 <= len(kinds) <= MAX_OBJECTS:
        raise ValueError(f"1..{MAX_OBJECTS} objects")
    out, seen = [], {}
    for k in kinds:
        if k not in KINDS:
            raise ValueError(f"unknown object {k!r} (kinds: {', '.join(KINDS)})")
        seen[k] = seen.get(k, 0) + 1
        out.append(SceneObject(k if seen[k] == 1 else f"{k}_{seen[k]}", k))
    return out


def add_objects(spec: mujoco.MjSpec, objects: list[SceneObject]) -> None:
    """Replace the scene XML's cube with `objects` (contact parameters as the XML cube)."""
    spec.delete(spec.body("cube"))
    mat = spec.material("cube_mat")
    for i, obj in enumerate(objects):
        gtype, size, mass, rgba = KINDS[obj.kind]
        x, y = DEFAULT_XY[i]
        body = spec.worldbody.add_body(name=obj.name, pos=[x, y, obj.rest_z])
        body.add_freejoint(name=obj.name)
        geom = body.add_geom(name=obj.name, type=gtype, size=list(size), mass=mass, rgba=list(rgba))
        geom.contype = geom.conaffinity = 1
        geom.condim = 6
        geom.friction = [1.2, 0.04, 0.01]
        geom.solref = [0.002, 1]
        geom.solimp = [0.98, 0.995, 0.0005, 0.5, 2]
        if obj.kind == "cube" and mat is not None:
            geom.material = "cube_mat"


def sample_poses(objects: list[SceneObject], seed: int | None) -> list[tuple[float, float, float]]:
    """(x, y, yaw) per object: the default layout without a seed, else seeded and separated."""
    if seed is None:
        return [(x, y, 0.0) for x, y in DEFAULT_XY[: len(objects)]]
    rng = np.random.default_rng(seed)
    poses: list[tuple[float, float, float]] = []
    for _ in objects:
        for _attempt in range(1000):
            x, y = rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE)
            if all(math.hypot(x - px, y - py) >= MIN_SEPARATION_M for px, py, _ in poses):
                break
        poses.append((float(x), float(y), float(rng.uniform(-math.pi / 2, math.pi / 2))))
    return poses


class Scene:
    """Object poses in a model/data pair: reset by seed, report state."""

    def __init__(self, model: mujoco.MjModel, objects: list[SceneObject]):
        self.model, self.objects = model, objects
        self.qpos = [model.jnt_qposadr[model.joint(o.name).id] for o in objects]
        self.qvel = [model.jnt_dofadr[model.joint(o.name).id] for o in objects]
        self.geom_ids = [model.geom(o.name).id for o in objects]
        self.jaw_ids = {model.geom(g).id: g for g in JAW_GEOMS}
        self.table_id = model.geom("table").id
        self.seed: int | None = None

    def reset(self, data: mujoco.MjData, seed: int | None) -> None:
        """Place the objects for `seed` at rest (zero velocity); the arm is not touched."""
        self.seed = seed
        for obj, qa, va, (x, y, yaw) in zip(self.objects, self.qpos, self.qvel, sample_poses(self.objects, seed)):
            data.qpos[qa:qa + 7] = [x, y, obj.rest_z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
            data.qvel[va:va + 6] = 0.0
        mujoco.mj_forward(self.model, data)

    def state(self, data: mujoco.MjData) -> list[dict]:
        out = []
        for obj, gid, qa in zip(self.objects, self.geom_ids, self.qpos):
            jaws, on_table = set(), False
            for c in data.contact[: data.ncon]:
                pair = {int(c.geom1), int(c.geom2)}
                if gid not in pair:
                    continue
                other = (pair - {gid}).pop() if len(pair) == 2 else gid
                if other in self.jaw_ids:
                    jaws.add("left" if self.jaw_ids[other].startswith("left") else "right")
                elif other == self.table_id:
                    on_table = True
            z = float(data.qpos[qa + 2])
            lifted = z - obj.rest_z
            out.append({
                "name": obj.name, "label": obj.kind,
                "pos": [round(float(v), 5) for v in data.qpos[qa:qa + 3]],
                "quat": [round(float(v), 5) for v in data.qpos[qa + 3:qa + 7]],
                "lifted_m": round(lifted, 5), "contacts": {"jaws": sorted(jaws), "table": on_table},
                "grasped": len(jaws) == 2 and lifted > LIFTED_M,
            })
        return out
