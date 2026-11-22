"""B601-DM / SO-ARM101 scene loading, self-collision checking, torque noise, and motor mapping."""
from __future__ import annotations

import json
import math
import os
import warnings
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[2] / "models"
SCENE_XML = MODEL_DIR / "rebot_b601dm_scene.xml"
SO101_SCENE_XML = MODEL_DIR / "so_arm101_scene.xml"

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
SO101_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
DOF = 7
KIT_PROFILE = Path(__file__).resolve().parents[4] / "kits" / "recamera-b601dm" / "profile" / "rebot-b601dm.json"
SO101_PROFILE = Path(__file__).resolve().parents[4] / "profiles" / "so_arm101.json"
# armd joint -> URDF (MuJoCo) joint angle offsets J1..J6: URDF = joint + offset. B601-DM: all 0,
# the official convention (armd kinematics::B601DM_MODEL_OFFSETS_RAD, profile field
# model_offsets_rad): the simulated arm at URDF 0 is the folded rest pose and reads 0 on every joint.
B601DM_MODEL_OFFSETS_RAD = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def model_offsets(profile: Path | str = KIT_PROFILE) -> np.ndarray:
    """`model_offsets_rad` of an armd profile; the B601-DM values when the file or field is absent."""
    try:
        off = json.loads(Path(profile).read_text()).get("model_offsets_rad")
    except (OSError, ValueError):
        off = None
    return np.array(off if off and len(off) == 6 else B601DM_MODEL_OFFSETS_RAD, dtype=float)


MODEL_OFFSETS_RAD = model_offsets()
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


def _canonical_pair(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a <= b else (b, a)


def _normalize_pair_set(pairs: Iterable[tuple[str, str]]) -> set[tuple[str, str]]:
    out: set[tuple[str, str]] = set()
    for item in pairs:
        if isinstance(item, (str, bytes)) or len(item) != 2:
            raise ValueError(f"Expected 2-element body-name pair, got {item!r}")
        a, b = str(item[0]).strip(), str(item[1]).strip()
        if not a or not b:
            raise ValueError(f"Body-name pair cannot contain empty names: {item!r}")
        out.add(_canonical_pair(a, b))
    return out


@dataclass(frozen=True)
class SelfCollisionConfig:
    """Configuration for arm link self-collision physics and contact checking."""

    enabled: bool = True
    contact_dist_threshold_m: float = 1e-6
    excluded_pairs: tuple[tuple[str, str], ...] = ()
    exclude_adjacent: bool = True
    exclude_sibling_jaws: bool = True
    exclude_rest_overlaps: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError(f"enabled must be a bool, got {type(self.enabled).__name__}")
        thresh = float(self.contact_dist_threshold_m)
        if not math.isfinite(thresh):
            raise ValueError(f"contact_dist_threshold_m must be finite, got {self.contact_dist_threshold_m!r}")
        object.__setattr__(self, "contact_dist_threshold_m", thresh)
        normalized = tuple(sorted(_normalize_pair_set(self.excluded_pairs)))
        object.__setattr__(self, "excluded_pairs", normalized)


@dataclass(frozen=True)
class SelfCollisionSummary:
    """Summary of non-adjacent arm link self-collisions detected in MuJoCo contacts."""

    count: int
    pairs: list[tuple[str, str]]
    min_dist_m: float | None
    contact_count: int = 0
    geom_pairs: list[tuple[str, str]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "pairs": [[a, b] for a, b in self.pairs],
            "min_dist_m": self.min_dist_m,
            "contact_count": self.contact_count,
            "geom_pairs": [[a, b] for a, b in self.geom_pairs],
        }


def _arm_root_name(model: mujoco.MjModel) -> str | None:
    for candidate in ("base_link", "base"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, candidate) >= 0:
            return candidate
    return None


def arm_body_ids(model: mujoco.MjModel) -> set[int]:
    """Return body IDs belonging to the robot arm kinematic subtree (excluding world & scene objects)."""
    root_name = _arm_root_name(model)
    if root_name is None:
        return set()
    root_id = int(model.body(root_name).id)
    ids: set[int] = set()
    for bid in range(1, model.nbody):
        cur = bid
        while cur > 0:
            if cur == root_id:
                ids.add(bid)
                break
            cur = int(model.body_parentid[cur])
    return ids


_SIBLING_JAW_PAIRS = (
    ("finger_left", "finger_right"),
    ("cup_jaw_left", "cup_jaw_right"),
)


def _sibling_jaw_pairs(body_names: set[str]) -> set[tuple[str, str]]:
    return {
        _canonical_pair(left, right)
        for left, right in _SIBLING_JAW_PAIRS
        if left in body_names and right in body_names
    }


def _validate_real_float(
    name: str, val: Any, *, non_negative: bool = False, positive: bool = False
) -> float:
    if isinstance(val, bool) or not isinstance(val, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be a real number, got {type(val).__name__}")
    out = float(val)
    if not math.isfinite(out) or (non_negative and out < 0.0) or (positive and out <= 0.0):
        raise ValueError(f"{name} out of range or non-finite, got {val!r}")
    return out


def default_excluded_adjacent_pairs(
    model: mujoco.MjModel,
    *,
    include_adjacent: bool = True,
    include_sibling_jaws: bool = True,
) -> set[tuple[str, str]]:
    """Compute canonical parent-child and sibling-jaw body pairs excluded from self-collision."""
    ids = arm_body_ids(model)
    excluded: set[tuple[str, str]] = set()
    names = {bid: (model.body(bid).name or f"body_{bid}") for bid in ids}

    if include_adjacent:
        for bid in ids:
            pid = int(model.body_parentid[bid])
            if pid in ids and pid != bid:
                excluded.add(_canonical_pair(names[pid], names[bid]))

    if include_sibling_jaws:
        excluded |= _sibling_jaw_pairs(set(names.values()))

    return excluded


def detect_link_self_collisions(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    excluded_adjacent_pairs: Iterable[tuple[str, str]] | None = None,
    config: SelfCollisionConfig | None = None,
) -> SelfCollisionSummary:
    """Inspect ``data.contact[:data.ncon]`` for non-adjacent arm link geom-to-geom contacts."""
    cfg = config if config is not None else SelfCollisionConfig()
    if not cfg.enabled:
        return SelfCollisionSummary(count=0, pairs=[], min_dist_m=None)

    arm_ids = arm_body_ids(model)
    if not arm_ids:
        return SelfCollisionSummary(count=0, pairs=[], min_dist_m=None)

    if excluded_adjacent_pairs is not None:
        excluded = _normalize_pair_set(excluded_adjacent_pairs)
    else:
        excluded = default_excluded_adjacent_pairs(
            model,
            include_adjacent=cfg.exclude_adjacent,
            include_sibling_jaws=cfg.exclude_sibling_jaws,
        )
    if cfg.excluded_pairs:
        excluded |= set(cfg.excluded_pairs)

    seen_pairs: set[tuple[str, str]] = set()
    seen_geom_pairs: set[tuple[str, str]] = set()
    min_dist = float("inf")
    contact_count = 0

    for i in range(int(data.ncon)):
        contact = data.contact[i]
        dist = float(contact.dist)
        if dist > cfg.contact_dist_threshold_m:
            continue
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if g1 < 0 or g2 < 0 or g1 >= model.ngeom or g2 >= model.ngeom:
            continue
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if b1 == b2 or b1 not in arm_ids or b2 not in arm_ids:
            continue

        n1 = model.body(b1).name or f"body_{b1}"
        n2 = model.body(b2).name or f"body_{b2}"
        pair = _canonical_pair(n1, n2)
        if pair in excluded:
            continue

        seen_pairs.add(pair)
        gn1 = model.geom(g1).name or f"geom_{g1}"
        gn2 = model.geom(g2).name or f"geom_{g2}"
        seen_geom_pairs.add(_canonical_pair(gn1, gn2))
        contact_count += 1
        if dist < min_dist:
            min_dist = dist

    return SelfCollisionSummary(
        count=len(seen_pairs),
        pairs=sorted(seen_pairs),
        min_dist_m=min_dist if seen_pairs else None,
        contact_count=contact_count,
        geom_pairs=sorted(seen_geom_pairs),
    )


def enable_self_collision_spec(
    spec: mujoco.MjSpec,
    config: SelfCollisionConfig | None = None,
) -> None:
    """Enable arm-to-arm collision bitmasks on an ``MjSpec`` and register adjacent/rest excludes."""
    cfg = config if config is not None else SelfCollisionConfig(enabled=True)
    if not cfg.enabled:
        return

    root_body = None
    for candidate in ("base_link", "base"):
        try:
            b = spec.body(candidate)
            if b is not None:
                root_body = b
                break
        except Exception:
            pass
    if root_body is None:
        return

    arm_bodies: list[Any] = []
    adjacent_pairs: set[tuple[str, str]] = set()
    stack = [root_body]
    while stack:
        body = stack.pop()
        arm_bodies.append(body)
        for child in body.bodies:
            if body.name and child.name:
                adjacent_pairs.add(_canonical_pair(body.name, child.name))
            stack.append(child)

    body_names = {b.name for b in arm_bodies if b.name}
    for body in arm_bodies:
        for geom in body.geoms:
            if int(geom.contype) != 0:
                geom.contype = int(geom.contype) | 2
                geom.conaffinity = int(geom.conaffinity) | 2

    excludes: set[tuple[str, str]] = set()
    if cfg.exclude_adjacent:
        excludes |= adjacent_pairs
    if cfg.exclude_sibling_jaws:
        excludes |= _sibling_jaw_pairs(body_names)
    if cfg.excluded_pairs:
        excludes |= set(cfg.excluded_pairs)

    for b1, b2 in sorted(excludes):
        spec.add_exclude(bodyname1=b1, bodyname2=b2)

    if cfg.exclude_rest_overlaps:
        # Exclude only body pairs whose coarse primitives already overlap at the nominal qpos0 rest pose.
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message=".*empty prefix.*", category=UserWarning)
            rest_model = spec.compile()
        rest_data = mujoco.MjData(rest_model)
        mujoco.mj_forward(rest_model, rest_data)
        rest_summary = detect_link_self_collisions(
            rest_model,
            rest_data,
            config=SelfCollisionConfig(
                enabled=True,
                contact_dist_threshold_m=cfg.contact_dist_threshold_m,
                excluded_pairs=tuple(sorted(excludes)),
                exclude_adjacent=cfg.exclude_adjacent,
                exclude_sibling_jaws=cfg.exclude_sibling_jaws,
                exclude_rest_overlaps=False,
            ),
        )
        for b1, b2 in rest_summary.pairs:
            if (b1, b2) not in excludes:
                excludes.add((b1, b2))
                spec.add_exclude(bodyname1=b1, bodyname2=b2)


@dataclass(frozen=True)
class TorqueNoiseConfig:
    """Configuration for deterministic actuator/torque noise injection (disabled by default)."""

    enabled: bool = False
    std_nm: float = 0.0
    bias_nm: float = 0.0
    clip_nm: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError(f"enabled must be a bool, got {type(self.enabled).__name__}")
        object.__setattr__(self, "std_nm", _validate_real_float("std_nm", self.std_nm, non_negative=True))
        object.__setattr__(self, "bias_nm", _validate_real_float("bias_nm", self.bias_nm))
        if self.clip_nm is not None:
            object.__setattr__(self, "clip_nm", _validate_real_float("clip_nm", self.clip_nm, positive=True))


def apply_torque_noise(
    ctrl_or_torque: Sequence[float] | np.ndarray,
    rng: np.random.Generator | None = None,
    config: TorqueNoiseConfig | None = None,
    *,
    seed: int | None = None,
) -> np.ndarray:
    """Inject deterministic Gaussian + bias torque noise when enabled; return exact copy otherwise."""
    raw = np.asarray(ctrl_or_torque)
    if raw.dtype == object or np.issubdtype(raw.dtype, np.bool_) or raw.dtype.kind not in ("i", "u", "f"):
        raise ValueError(f"ctrl_or_torque must be numeric, got dtype {raw.dtype!r}")
    if not np.all(np.isfinite(raw)):
        raise ValueError("ctrl_or_torque contains non-finite values")

    if config is None or not config.enabled:
        return np.array(raw, copy=True)
    if config.std_nm == 0.0 and config.bias_nm == 0.0:
        return np.array(raw, copy=True)

    arr = np.asarray(raw, dtype=np.float64).copy()
    gen = rng if rng is not None else np.random.default_rng(seed)
    noise = np.zeros_like(arr, dtype=np.float64)
    if config.std_nm > 0.0:
        noise += gen.normal(0.0, config.std_nm, size=arr.shape)
    if config.bias_nm != 0.0:
        noise += config.bias_nm
    if config.clip_nm is not None:
        noise = np.clip(noise, -config.clip_nm, config.clip_nm)
    return arr + noise


def _coerce_self_collision_config(
    self_collision: bool | SelfCollisionConfig | None,
) -> SelfCollisionConfig | None:
    if self_collision is None or self_collision is False:
        return None
    if self_collision is True:
        return SelfCollisionConfig(enabled=True)
    if isinstance(self_collision, SelfCollisionConfig):
        return self_collision if self_collision.enabled else None
    raise TypeError(
        f"self_collision must be bool, SelfCollisionConfig, or None, got {type(self_collision).__name__}"
    )


def load_model(
    devarm_dir: str | None = None,
    objects=None,
    camera=None,
    robot: str | None = None,
    scene_pack=None,
    tool_pack=None,
    self_collision: bool | SelfCollisionConfig = False,
) -> mujoco.MjModel:
    """Compile the scene; with a reBot-DevArm checkout, add its link meshes as visuals.

    `objects` (list of `scene.SceneObject`) replaces the XML's cube; `camera` (a
    `camera.CameraMount`) adds the `wrist` and `scene` cameras; `scene_pack` and
    `tool_pack` attach composable world XMLs and custom end-effectors.
    """
    sc_cfg = _coerce_self_collision_config(self_collision)
    has_tool = tool_pack is not None and not getattr(tool_pack, "is_default", True)
    target_robot = (robot or os.environ.get("REBOT_SIM_ROBOT") or "b601dm").lower()
    if target_robot in ("so-arm101", "so_arm101"):
        scene_xml = SO101_SCENE_XML
        if objects is None and camera is None and scene_pack is None and not has_tool and sc_cfg is None:
            return mujoco.MjModel.from_xml_path(str(scene_xml))
        spec = mujoco.MjSpec.from_file(str(scene_xml))
        if scene_pack is not None:
            from .pack import compose_spec

            compose_spec(spec, scene_pack=scene_pack, tool_pack=tool_pack, robot=target_robot)
        else:
            if objects is not None:
                from .scene import add_objects

                add_objects(spec, objects)
            if has_tool:
                from .pack import compose_spec

                compose_spec(spec, scene_pack=None, tool_pack=tool_pack, robot=target_robot)
        if sc_cfg is not None:
            enable_self_collision_spec(spec, sc_cfg)
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message=".*empty prefix.*", category=UserWarning)
            return spec.compile()

    devarm_dir = devarm_dir if devarm_dir is not None else os.environ.get("REBOT_DEVARM_DIR")
    if not devarm_dir and objects is None and camera is None and scene_pack is None and not has_tool and sc_cfg is None:
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
    if scene_pack is not None:
        from .pack import compose_spec

        compose_spec(spec, scene_pack=scene_pack, tool_pack=tool_pack, robot=target_robot)
    else:
        if objects is not None:
            from .scene import add_objects

            add_objects(spec, objects)
        if has_tool:
            from .pack import compose_spec

            compose_spec(spec, scene_pack=None, tool_pack=tool_pack, robot=target_robot)
    if sc_cfg is not None:
        enable_self_collision_spec(spec, sc_cfg)
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message=".*empty prefix.*", category=UserWarning)
        if camera is not None:
            from .camera import add_cameras, mount_rotation

            add_cameras(spec, mount_rotation(spec.compile(), camera), camera.offset, camera.vfov_deg)
        return spec.compile()


class Indexer:
    """qpos / qvel addresses of the arm joints and gripper."""

    def __init__(self, model: mujoco.MjModel, tool_pack=None):
        joint_names = {model.joint(i).name for i in range(model.njnt)}
        if "shoulder_pan" in joint_names:
            self.robot = "so-arm101"
            self.dof = 6
            self.joints = SO101_JOINTS
            self.qpos = np.array([model.jnt_qposadr[model.joint(j).id] for j in self.joints])
            self.qvel = np.array([model.jnt_dofadr[model.joint(j).id] for j in self.joints])
            self.right_qpos = None
            self.tool_scale = 1.0
        else:
            self.robot = "b601dm"
            driven = getattr(tool_pack, "driven_joint", None) if tool_pack is not None else None
            coupled = getattr(tool_pack, "coupled_joint", None) if tool_pack is not None else None
            has_custom_tool = tool_pack is not None and not getattr(tool_pack, "is_default", True)
            if has_custom_tool:
                tool_joint = [driven] if driven and driven in joint_names else []
                self.joints = ARM_JOINTS + tool_joint
                self.tool_scale = float(getattr(tool_pack, "motor_to_joint_scale", 1.0) or 1.0)
                coupled_name = coupled
            elif "finger_left" in joint_names:
                self.joints = ARM_JOINTS + ["finger_left"]
                self.tool_scale = -JAW_M_PER_RAD
                coupled_name = "finger_right"
            else:
                inferred = [model.joint(int(model.actuator_trnid[6, 0])).name] if model.nu >= 7 else []
                self.joints = ARM_JOINTS + inferred
                self.tool_scale = -JAW_M_PER_RAD
                coupled_name = None
            self.dof = len(self.joints)
            self.qpos = np.array([model.jnt_qposadr[model.joint(j).id] for j in self.joints])
            self.qvel = np.array([model.jnt_dofadr[model.joint(j).id] for j in self.joints])
            self.right_qpos = model.jnt_qposadr[model.joint(coupled_name).id] if coupled_name and coupled_name in joint_names else None

    def motor_positions(self, data: mujoco.MjData) -> np.ndarray:
        """armd motor angles (rad)."""
        q = data.qpos[self.qpos].copy()
        if self.robot == "b601dm":
            q[:6] -= MODEL_OFFSETS_RAD
            if len(q) > 6:
                q[6] = q[6] / self.tool_scale
        return q

    def motor_velocities(self, data: mujoco.MjData) -> np.ndarray:
        dq = data.qvel[self.qvel].copy()
        if self.robot == "b601dm" and len(dq) > 6:
            dq[6] = dq[6] / self.tool_scale
        return dq

    def set_motor_positions(self, data: mujoco.MjData, q) -> None:
        q = np.asarray(q, dtype=float)
        if self.robot == "b601dm":
            data.qpos[self.qpos[:6]] = q[:6] + MODEL_OFFSETS_RAD
            if len(q) > 6 and len(self.qpos) > 6:
                jaw = q[6] * self.tool_scale
                data.qpos[self.qpos[6]] = jaw
                if self.right_qpos is not None:
                    data.qpos[self.right_qpos] = jaw
        else:
            data.qpos[self.qpos[:len(q)]] = q

    def tcp_pose(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        if self.robot == "so-arm101":
            pos = data.site("tcp").xpos.copy()
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, data.site("tcp").xmat)
            return pos, quat
        return data.body("end_link").xpos.copy(), data.body("end_link").xquat.copy()
