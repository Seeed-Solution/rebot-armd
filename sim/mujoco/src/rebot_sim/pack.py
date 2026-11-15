"""Composable scene packs, custom end-effectors, virtual I/O, and task evaluation."""

from __future__ import annotations

import json
import os
import re
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .scene import JAW_GEOMS, LIFTED_M

PACK_SLUG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


def is_valid_pack_slug(val: Any) -> bool:
    """Return True if val is a strict slug ID (1..64 chars, starting with alphanumeric), empty, or 'default'."""
    if val is None:
        return True
    if not isinstance(val, str):
        return False
    raw = val.strip()
    if not raw or raw in {"default", "default_world"}:
        return True
    return bool(PACK_SLUG_RE.match(raw))


_DEFAULT_SUCTION_XML = """<mujoco model="builtin_suction_tool">
  <worldbody>
    <geom name="suction_manifold" type="box" pos="-0.080 0 0" size="0.018 0.032 0.032"
          rgba="0.22 0.25 0.30 1"/>
    <geom name="suction_stem" type="cylinder" pos="-0.040 0 0" euler="0 90 0" size="0.012 0.025"
          rgba="0.58 0.62 0.68 1"/>
    <geom name="suction_cup_bellows" type="cylinder" pos="-0.008 0 0" euler="0 90 0" size="0.016 0.008"
          rgba="0.14 0.45 0.72 1" friction="1.4 0.01 0.001"/>
    <body name="suction_valve_body" pos="-0.080 0 0">
      <joint name="suction_valve" type="slide" axis="0 1 0" range="-0.06 0.06" damping="2"/>
      <inertial pos="0 0 0" mass="0.01" diaginertia="1e-6 1e-6 1e-6"/>
      <geom name="suction_valve_dummy" type="sphere" size="0.004" rgba="0.2 0.8 0.9 1"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="m7_suction" joint="suction_valve" gear="1" kp="20" dampratio="1" forcerange="-5 5"/>
  </actuator>
</mujoco>
"""

_DEFAULT_SCREWDRIVER_XML = """<mujoco model="builtin_screwdriver_tool">
  <worldbody>
    <geom name="driver_housing" type="cylinder" pos="-0.085 0 0" euler="0 90 0" size="0.022 0.040"
          rgba="0.20 0.24 0.28 1"/>
    <body name="driver_spindle" pos="-0.022 0 0">
      <joint name="spindle_joint" type="hinge" axis="1 0 0" damping="0.1"/>
      <inertial pos="0 0 0" mass="0.03" diaginertia="1e-5 1e-5 1e-5"/>
      <geom name="driver_bit" type="cylinder" pos="0 0 0" euler="0 90 0" size="0.005 0.022"
            rgba="0.72 0.74 0.78 1" friction="1.0 0.01 0.001"/>
    </body>
  </worldbody>
  <actuator>
    <position name="m7_spindle" joint="spindle_joint" kp="10" dampratio="1" forcerange="-5 5"/>
  </actuator>
</mujoco>
"""


def _search_pack_roots() -> list[Path]:
    """Return candidate directories holding named scene/tool packs."""
    roots: list[Path] = []
    env_dir = os.environ.get("REBOT_SCENES_DIR")
    if env_dir:
        roots.append(Path(env_dir).expanduser())
    here = Path(__file__).resolve()
    # Sibling rebot-companion checkout during local development
    for parent in here.parents:
        candidate = (
            parent / "rebot-companion" / "rebot_companion" / "scenes" / "builtin"
        )
        if candidate.is_dir():
            roots.append(candidate)
            break
        alt = parent / "rebot_companion" / "scenes" / "builtin"
        if alt.is_dir():
            roots.append(alt)
            break
    try:
        import importlib.util

        spec = importlib.util.find_spec("rebot_companion.scenes")
        if spec and spec.submodule_search_locations:
            pkg_builtin = Path(next(iter(spec.submodule_search_locations))) / "builtin"
            if pkg_builtin.is_dir():
                roots.append(pkg_builtin)
    except Exception:
        pass
    return roots


def _cache_scene_dir(pack_id: str) -> Path:
    env = os.environ.get("REBOT_SCENE_CACHE")
    base = (
        Path(env).expanduser()
        if env
        else Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        / "rebot"
        / "scenes"
    )
    return base / pack_id


def resolve_pack_path(ref: str | Path | None, kind: str) -> Path | None:
    """Resolve a local path or named built-in pack id ('coffee_kiosk', 'wide_cup_gripper')."""
    if ref is None:
        return None
    raw = str(ref).strip()
    if not raw or raw in {"default", "default_world"}:
        return None
    direct = Path(raw).expanduser()
    if direct.exists():
        return direct.resolve()
    sub = "scenes" if kind in {"scene", "scenes"} else "tools"
    for root in _search_pack_roots():
        candidate = root / sub / raw
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(f"Unknown {kind} pack or missing path: {ref!r}")


def _resolve_within(root: Path, rel: str) -> Path:
    p = (root / rel).resolve()
    root_resolved = root.resolve()
    if p != root_resolved and root_resolved not in p.parents:
        raise ValueError(f"Relative path {rel!r} escapes {root}")
    return p


@dataclass
class ToolPack:
    """Custom or preset end-effector definition for rebot-sim."""

    id: str
    name: str
    kind: str = "gripper"
    tool_xml_path: Path | None = None
    tool_spec_xml: str | None = None
    attach_body: str = "end_link"
    replace_bodies: list[str] = field(
        default_factory=lambda: ["finger_left", "finger_right"]
    )
    replace_geoms: list[str] = field(default_factory=lambda: ["palm", "motor_housing"])
    replace_equalities: list[str] = field(default_factory=lambda: ["jaw_coupling"])
    replace_actuators: list[str] = field(default_factory=lambda: ["m7"])
    driven_joint: str | None = None
    coupled_joint: str | None = None
    actuator: str | None = None
    motor_to_joint_scale: float = 1.0
    finger_geoms: list[str] = field(default_factory=list)
    tcp_site: str = "tcp"
    tcp_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    suction_range_m: float = 0.030
    io: list[dict[str, Any]] = field(default_factory=list)
    source_ref: str = "default"

    @property
    def is_default(self) -> bool:
        return (
            self.id == "default"
            and self.tool_xml_path is None
            and self.tool_spec_xml is None
        )

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "kind": self.kind,
            "attach_body": self.attach_body,
            "driven_joint": self.driven_joint,
            "coupled_joint": self.coupled_joint,
            "actuator": self.actuator,
            "motor_to_joint_scale": self.motor_to_joint_scale,
            "tcp_offset": list(self.tcp_offset),
            "io": list(self.io),
            "source_ref": self.source_ref,
        }


def _infer_tool_joints_from_xml(
    xml_path: Path,
) -> tuple[str | None, str | None, str | None]:
    root = ET.parse(xml_path).getroot()
    joints = [
        el.attrib["name"] for el in root.findall(".//joint") if "name" in el.attrib
    ]
    actuators = [
        (el.attrib.get("name"), el.attrib.get("joint"))
        for el in root.findall(".//actuator/*")
    ]
    actuator_name = actuators[0][0] if actuators else None
    driven = (
        actuators[0][1]
        if actuators and actuators[0][1]
        else (joints[0] if joints else None)
    )
    coupled = next((j for j in joints if j != driven), None)
    return driven, coupled, actuator_name


def load_tool_pack(ref: str | Path | None) -> ToolPack:
    """Load a preset tool ('default', 'suction', 'screwdriver'), pack dir, tool.json, or raw XML."""
    raw = str(ref).strip() if ref is not None else "default"
    if not raw or raw == "default":
        return ToolPack(
            id="default",
            name="Stock Parallel Jaw Gripper",
            kind="gripper",
            source_ref="default",
        )

    if raw == "suction":
        return ToolPack(
            id="suction",
            name="Vacuum Suction End-Effector",
            kind="suction",
            tool_spec_xml=_DEFAULT_SUCTION_XML,
            driven_joint="suction_valve",
            actuator="m7_suction",
            motor_to_joint_scale=0.007456140350877194,
            io=[
                {
                    "name": "vacuum",
                    "kind": "bool",
                    "direction": "output",
                    "default": False,
                },
                {
                    "name": "vacuum_engaged",
                    "kind": "bool",
                    "direction": "input",
                    "default": False,
                },
            ],
            source_ref="suction",
        )

    if raw == "screwdriver":
        return ToolPack(
            id="screwdriver",
            name="Spindle Electric Screwdriver",
            kind="screwdriver",
            tool_spec_xml=_DEFAULT_SCREWDRIVER_XML,
            driven_joint="spindle_joint",
            actuator="m7_spindle",
            motor_to_joint_scale=1.0,
            io=[
                {
                    "name": "spindle_rpm",
                    "kind": "float",
                    "direction": "output",
                    "default": 0.0,
                }
            ],
            source_ref="screwdriver",
        )

    path = resolve_pack_path(raw, "tool")
    if path is None:
        return ToolPack(id="default", name="Stock Parallel Jaw Gripper", kind="gripper")

    if path.is_file() and path.suffix.lower() == ".xml":
        driven, coupled, act = _infer_tool_joints_from_xml(path)
        return ToolPack(
            id=path.stem,
            name=path.stem.replace("_", " ").title(),
            kind="custom",
            tool_xml_path=path,
            driven_joint=driven,
            coupled_joint=coupled,
            actuator=act,
            motor_to_joint_scale=-0.007456140350877194,
            source_ref=str(path),
        )

    tool_json = (
        path if path.is_file() and path.name == "tool.json" else path / "tool.json"
    )
    if not tool_json.is_file():
        raise FileNotFoundError(f"Expected tool.json or .xml file in {path}")
    data = json.loads(tool_json.read_text(encoding="utf-8"))
    tool_dir = tool_json.parent
    tool_xml = _resolve_within(tool_dir, str(data.get("tool_xml", "tool.xml")))
    if not tool_xml.is_file():
        raise FileNotFoundError(f"Tool XML not found: {tool_xml}")

    return ToolPack(
        id=str(data["id"]),
        name=str(data.get("name", data["id"])),
        kind=str(data.get("kind", "gripper")).lower(),
        tool_xml_path=tool_xml,
        attach_body=str(data.get("attach_body", "end_link")),
        replace_bodies=[
            str(b) for b in data.get("replace_bodies", ["finger_left", "finger_right"])
        ],
        replace_geoms=[
            str(g) for g in data.get("replace_geoms", ["palm", "motor_housing"])
        ],
        replace_equalities=[
            str(e) for e in data.get("replace_equalities", ["jaw_coupling"])
        ],
        replace_actuators=[str(a) for a in data.get("replace_actuators", ["m7"])],
        driven_joint=data.get("driven_joint"),
        coupled_joint=data.get("coupled_joint"),
        actuator=data.get("actuator"),
        motor_to_joint_scale=float(data.get("motor_to_joint_scale", 1.0)),
        finger_geoms=[str(g) for g in data.get("finger_geoms", [])],
        tcp_site=str(data.get("tcp_site", "tcp")),
        tcp_offset=[float(v) for v in data.get("tcp_offset", [0.0, 0.0, 0.0])],
        suction_range_m=float(data.get("suction_range_m", 0.030)),
        io=[dict(ch) for ch in data.get("io", [])],
        source_ref=str(tool_dir),
    )


@dataclass
class ScenePack:
    """Declarative world geometry, objects, virtual I/O, and staged task evaluation."""

    id: str
    name: str
    world_xml_path: Path
    objects: list[dict[str, Any]] = field(default_factory=list)
    zones: dict[str, Any] = field(default_factory=dict)
    io: list[dict[str, Any]] = field(default_factory=list)
    task_stages: list[dict[str, Any]] = field(default_factory=list)
    poses: dict[str, Any] = field(default_factory=dict)
    keepout_boxes: list[dict[str, Any]] = field(default_factory=list)
    default_tool: str = "default"
    assets: list[dict[str, Any]] = field(default_factory=list)
    source_ref: str = ""

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "default_tool": self.default_tool,
            "objects": [o.get("name") for o in self.objects],
            "zones": self.zones,
            "io": self.io,
            "task_stages": [st.get("id") for st in self.task_stages],
            "poses": self.poses,
            "keepout_boxes": self.keepout_boxes,
            "source_ref": self.source_ref,
        }


def _discover_freejoint_objects(world_xml: Path) -> list[dict[str, Any]]:
    root = ET.parse(world_xml).getroot()
    objs: list[dict[str, Any]] = []
    for body in root.findall(".//worldbody//body"):
        if body.find("freejoint") is None and body.find("joint[@type='free']") is None:
            continue
        bname = body.attrib.get("name")
        if not bname:
            continue
        first_geom = body.find("geom")
        gname = (
            first_geom.attrib.get("name", bname) if first_geom is not None else bname
        )
        pos_str = body.attrib.get("pos", "0.25 0.0 0.02")
        pos = [float(x) for x in pos_str.split()]
        objs.append(
            {
                "name": bname,
                "body": bname,
                "geom": gname,
                "class_name": bname,
                "default_pos": pos[:3],
                "jitter_xy": 0.02,
            }
        )
    return objs


def load_scene_pack(ref: str | Path | None) -> ScenePack | None:
    """Load a scene pack from built-in id, directory, scene.json, or standalone world XML."""
    path = resolve_pack_path(ref, "scene")
    if path is None:
        return None
    if path.is_file() and path.suffix.lower() == ".xml":
        return ScenePack(
            id=path.stem,
            name=path.stem.replace("_", " ").title(),
            world_xml_path=path,
            objects=_discover_freejoint_objects(path),
            source_ref=str(path),
        )
    scene_json = (
        path if path.is_file() and path.name == "scene.json" else path / "scene.json"
    )
    if not scene_json.is_file():
        raise FileNotFoundError(f"Expected scene.json or .xml in {path}")
    data = json.loads(scene_json.read_text(encoding="utf-8"))
    scene_dir = scene_json.parent
    world_xml = _resolve_within(scene_dir, str(data.get("world_xml", "world.xml")))
    if not world_xml.is_file():
        raise FileNotFoundError(f"Scene world XML not found: {world_xml}")
    objects = [dict(o) for o in data.get("objects", [])] or _discover_freejoint_objects(
        world_xml
    )
    return ScenePack(
        id=str(data["id"]),
        name=str(data.get("name", data["id"])),
        world_xml_path=world_xml,
        objects=objects,
        zones=dict(data.get("zones", {})),
        io=[dict(ch) for ch in data.get("io", [])],
        task_stages=[dict(st) for st in data.get("task_stages", [])],
        poses=dict(data.get("poses", {})),
        keepout_boxes=[dict(b) for b in data.get("keepout_boxes", [])],
        default_tool=str(data.get("default_tool", "default")),
        assets=[dict(a) for a in data.get("assets", [])],
        source_ref=str(scene_dir),
    )


def _delete_named(spec: mujoco.MjSpec, kind: str, names: list[str]) -> None:
    items = getattr(spec, kind)
    by_name = {getattr(el, "name", None): el for el in items}
    for name in names:
        el = by_name.get(name)
        if el is not None:
            spec.delete(el)


def compose_spec(
    spec: mujoco.MjSpec,
    *,
    scene_pack: ScenePack | None = None,
    tool_pack: ToolPack | None = None,
    robot: str = "b601dm",
) -> None:
    """Attach optional ScenePack world XML and ToolPack end-effector XML onto base robot MjSpec."""
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", message=".*empty prefix.*", category=UserWarning
        )

        if scene_pack is not None:
            _delete_named(spec, "bodies", ["cube"])
            world_spec = mujoco.MjSpec.from_file(str(scene_pack.world_xml_path))
            cache_dir = _cache_scene_dir(scene_pack.id)
            if cache_dir.is_dir() and not world_spec.meshdir:
                world_spec.meshdir = str(cache_dir)
            frame = spec.worldbody.add_frame()
            spec.attach(world_spec, frame=frame, prefix="", suffix="")

        if tool_pack is not None and not tool_pack.is_default:
            if (
                robot.lower() in {"so-arm101", "so_arm101"}
                and tool_pack.attach_body == "end_link"
            ):
                attach_target = next(
                    (
                        b.name
                        for b in spec.bodies
                        if b.name in {"end_link", "gripper_link"}
                    ),
                    tool_pack.attach_body,
                )
            else:
                attach_target = tool_pack.attach_body

            _delete_named(spec, "equalities", tool_pack.replace_equalities)
            _delete_named(spec, "actuators", tool_pack.replace_actuators)
            _delete_named(spec, "bodies", tool_pack.replace_bodies)
            _delete_named(spec, "geoms", tool_pack.replace_geoms)

            if tool_pack.tool_spec_xml is not None:
                tool_spec = mujoco.MjSpec.from_string(tool_pack.tool_spec_xml)
            elif tool_pack.tool_xml_path is not None:
                tool_spec = mujoco.MjSpec.from_file(str(tool_pack.tool_xml_path))
            else:
                raise ValueError(
                    f"ToolPack {tool_pack.id!r} has neither tool_xml_path nor tool_spec_xml"
                )

            # Preserve the sensor-bound 'tcp' site on attach_body rather than duplicating it
            tool_tcp = next(
                (s for s in tool_spec.sites if s.name == tool_pack.tcp_site), None
            )
            tcp_pos_override = list(tool_pack.tcp_offset)
            tcp_quat_override = None
            if tool_tcp is not None:
                if tcp_pos_override == [0.0, 0.0, 0.0]:
                    tcp_pos_override = list(tool_tcp.pos)
                tcp_quat_override = list(tool_tcp.quat)
                tool_spec.delete(tool_tcp)

            attach_body = spec.body(attach_target)
            attach_frame = attach_body.add_frame()
            spec.attach(tool_spec, frame=attach_frame, prefix="", suffix="")

            base_tcp = next((s for s in spec.sites if s.name == "tcp"), None)
            if base_tcp is not None:
                base_tcp.pos = tcp_pos_override
                if tcp_quat_override is not None:
                    base_tcp.quat = tcp_quat_override

        if tool_pack is not None and tool_pack.kind == "suction":
            free_bodies = [
                b.name
                for b in spec.bodies
                if b.name != "world"
                and any(j.type == mujoco.mjtJoint.mjJNT_FREE for j in b.joints)
            ]
            attach_name = tool_pack.attach_body
            for i, bname in enumerate(free_bodies):
                eq = spec.add_equality()
                eq.name = f"suction_weld_{i}_{bname}"
                eq.type = mujoco.mjtEq.mjEQ_WELD
                eq.name1 = attach_name
                eq.name2 = bname
                eq.objtype = mujoco.mjtObj.mjOBJ_BODY
                eq.active = False
                eq.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0]
                eq.solref = [0.02, 1.0]
                eq.solimp = [0.9, 0.95, 0.001, 0.5, 2.0]


@dataclass
class PackSceneObject:
    """Object descriptor exposed by PackScene (compatible with SceneObject)."""

    name: str
    kind: str
    body: str
    geom: str
    default_pos: tuple[float, float, float]
    jitter_xy: float = 0.02


class TaskEvaluator:
    """Evaluates scene `task_stages` and drives optional visual indicator geoms."""

    def __init__(self, model: mujoco.MjModel, scene_pack: ScenePack, scene: PackScene):
        self.model = model
        self.pack = scene_pack
        self.scene = scene
        self._latched: dict[str, bool] = {}
        self._saved_rgba: dict[int, np.ndarray] = {}
        for st in scene_pack.task_stages:
            gname = st.get("indicator_geom")
            if gname:
                try:
                    gid = model.geom(gname).id
                    self._saved_rgba[gid] = model.geom_rgba[gid].copy()
                except KeyError:
                    pass

    def reset(self) -> None:
        self._latched.clear()
        for gid, rgba in self._saved_rgba.items():
            self.model.geom_rgba[gid] = rgba

    def evaluate(
        self,
        data: mujoco.MjData,
        obj_state: dict[str, dict[str, Any]],
        io_state: dict[str, Any],
    ) -> dict[str, Any]:
        stages_out: dict[str, bool] = {}
        for st in self.pack.task_stages:
            sid = str(st["id"])
            if self._latched.get(sid):
                stages_out[sid] = True
                continue
            reqs = [str(r) for r in st.get("requires", [])]
            if any(not stages_out.get(r, False) for r in reqs):
                stages_out[sid] = False
                continue

            stype = st.get("type")
            passed = False
            if stype == "object_grasped":
                oname = str(st["object"])
                info = obj_state.get(oname)
                if info is not None:
                    min_z = st.get("min_z")
                    z_ok = (
                        True if min_z is None else float(info["pos"][2]) >= float(min_z)
                    )
                    passed = bool(info["grasped"]) and z_ok
            elif stype == "object_in_zone":
                oname = str(st["object"])
                zname = str(st["zone"])
                info = obj_state.get(oname)
                zone = self.pack.zones.get(zname)
                if info is not None and zone is not None:
                    pos = np.asarray(info["pos"], dtype=float)
                    center = np.asarray(zone["center"], dtype=float)
                    half = np.asarray(zone["half_extents"], dtype=float)
                    in_zone = bool(np.all(np.abs(pos - center) <= half))
                    allow_grasped = bool(st.get("allow_grasped", False))
                    released_ok = allow_grasped or not bool(info.get("grasped", False))
                    passed = in_zone and released_ok
            elif stype == "joint_threshold":
                jname = str(st["joint"])
                try:
                    jid = self.model.joint(jname).id
                    val = float(data.qpos[self.model.jnt_qposadr[jid]])
                    thresh = float(st["threshold"])
                    op = str(st.get("op", "<="))
                    if op == "<=":
                        passed = val <= thresh
                    elif op == "<":
                        passed = val < thresh
                    elif op == ">=":
                        passed = val >= thresh
                    elif op == ">":
                        passed = val > thresh
                except KeyError:
                    passed = False
            elif stype == "io_state":
                key = str(st["channel"])
                passed = io_state.get(key) == st.get("value")

            if passed and st.get("latch", True):
                self._latched[sid] = True
            stages_out[sid] = passed

            if passed and st.get("indicator_geom") and st.get("indicator_rgba"):
                try:
                    gid = self.model.geom(st["indicator_geom"]).id
                    self.model.geom_rgba[gid] = np.asarray(
                        st["indicator_rgba"], dtype=np.float32
                    )
                except KeyError:
                    pass

        total = len(self.pack.task_stages)
        done_count = sum(1 for v in stages_out.values() if v)
        current_stage = next(
            (
                st["id"]
                for st in self.pack.task_stages
                if not stages_out.get(st["id"], False)
            ),
            None,
        )
        return {
            "scene": self.pack.id,
            "completed": total > 0 and done_count == total,
            "score": round(done_count / total, 4) if total else 0.0,
            "current_stage": current_stage,
            "stages": stages_out,
        }


class PackScene:
    """Runtime scene manager supporting Pack-defined worlds, custom tools, virtual IO, and stages."""

    def __init__(
        self,
        model: mujoco.MjModel,
        scene_pack: ScenePack | None,
        tool_pack: ToolPack | None,
        fallback_objects: list[Any] | None = None,
    ):
        self.model = model
        self.scene_pack = scene_pack
        self.tool_pack = tool_pack or load_tool_pack("default")
        self.seed: int | None = None

        raw_objs: list[PackSceneObject] = []
        if scene_pack is not None:
            for item in scene_pack.objects:
                raw_objs.append(
                    PackSceneObject(
                        name=str(item["name"]),
                        kind=str(item.get("class_name", item["name"])),
                        body=str(item.get("body", item["name"])),
                        geom=str(item.get("geom", item["name"])),
                        default_pos=tuple(
                            float(v) for v in item.get("default_pos", [0.25, 0.0, 0.02])
                        ),  # type: ignore[arg-type]
                        jitter_xy=float(item.get("jitter_xy", 0.02)),
                    )
                )
        elif fallback_objects is not None:
            for item in fallback_objects:
                bname = getattr(item, "body", item.name)
                gname = getattr(item, "geom", item.name)
                def_pos = getattr(item, "default_pos", None)
                if def_pos is None:
                    try:
                        def_pos = tuple(
                            float(v) for v in model.body_pos[model.body(bname).id]
                        )
                    except KeyError:
                        def_pos = (0.25, 0.0, 0.02)
                raw_objs.append(
                    PackSceneObject(
                        name=item.name,
                        kind=item.kind,
                        body=bname,
                        geom=gname,
                        default_pos=tuple(float(v) for v in def_pos),  # type: ignore[arg-type]
                        jitter_xy=float(getattr(item, "jitter_xy", 0.03)),
                    )
                )

        self.objects = raw_objs
        self.body_ids = [model.body(o.body).id for o in self.objects]
        self.geom_ids = np.array(
            [model.geom(o.geom).id for o in self.objects], dtype=int
        )
        self.body_geom_ids: dict[int, list[int]] = {
            bid: [g for g in range(model.ngeom) if model.geom_bodyid[g] == bid]
            for bid in self.body_ids
        }
        self._free_qpos = [
            model.jnt_qposadr[model.body_jntadr[bid]] for bid in self.body_ids
        ]
        self._obj_default_quats = [
            model.body_quat[bid].copy()
            if np.any(model.body_quat[bid])
            else np.array([1.0, 0.0, 0.0, 0.0])
            for bid in self.body_ids
        ]

        # Non-object, non-arm joints inside scene XML (e.g. spring buttons)
        self._scene_joints: list[tuple[int, float]] = []
        if scene_pack is not None:
            obj_qpos = set(self._free_qpos)
            arm_prefix = (
                "joint",
                "shoulder",
                "elbow",
                "wrist",
                "gripper",
                "finger",
                "cup_jaw",
                "suction_valve",
                "spindle",
            )
            for j in range(model.njnt):
                jname = model.joint(j).name or ""
                adr = int(model.jnt_qposadr[j])
                if adr in obj_qpos or model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    continue
                if any(jname.startswith(p) for p in arm_prefix):
                    continue
                self._scene_joints.append((adr, float(model.qpos0[adr])))

        # End-effector contact geoms used to detect grasped objects
        finger_names = (
            list(self.tool_pack.finger_geoms)
            if self.tool_pack.finger_geoms
            else list(JAW_GEOMS)
        )
        self._tool_contact_geoms: set[int] = set()
        for gname in finger_names:
            try:
                self._tool_contact_geoms.add(model.geom(gname).id)
            except KeyError:
                pass
        if not self._tool_contact_geoms:
            attach_name = self.tool_pack.attach_body
            try:
                attach_bid = model.body(attach_name).id
                for g in range(model.ngeom):
                    bname = model.body(model.geom_bodyid[g]).name
                    if model.geom_bodyid[g] == attach_bid or (
                        self.tool_pack.driven_joint
                        and self.tool_pack.driven_joint.split("_")[0] in bname
                    ):
                        self._tool_contact_geoms.add(g)
            except KeyError:
                pass

        # Pre-allocated suction weld equalities
        self._suction_welds: dict[str, int] = {}
        self._attached_object: str | None = None
        for eq_id in range(model.neq):
            ename = model.eq(eq_id).name or ""
            if ename.startswith("suction_weld_"):
                body2_id = int(model.eq_obj2id[eq_id])
                self._suction_welds[model.body(body2_id).name] = eq_id

        # Virtual IO defaults
        self._io_defaults: dict[str, Any] = {}
        for src in (scene_pack.io if scene_pack else []) + self.tool_pack.io:
            if src.get("direction", "output") == "output" or "default" in src:
                self._io_defaults[str(src["name"])] = src.get("default", False)
        self._io: dict[str, Any] = dict(self._io_defaults)

        self.evaluator = (
            TaskEvaluator(model, scene_pack, self)
            if scene_pack is not None and scene_pack.task_stages
            else None
        )

    def _release_suction(self, data: mujoco.MjData) -> None:
        for eq_id in self._suction_welds.values():
            data.eq_active[eq_id] = 0
        self._attached_object = None
        if "vacuum_engaged" in self._io or self.tool_pack.kind == "suction":
            self._io["vacuum_engaged"] = False

    def _try_engage_suction(self, data: mujoco.MjData) -> None:
        if self._attached_object is not None or not self._suction_welds:
            return
        try:
            tcp_pos = data.site("tcp").xpos
            attach_bid = self.model.body(self.tool_pack.attach_body).id
        except KeyError:
            return

        best_obj: PackSceneObject | None = None
        best_dist = float("inf")
        for obj, bid in zip(self.objects, self.body_ids):
            if obj.body not in self._suction_welds:
                continue
            for gid in self.body_geom_ids.get(bid, []):
                dist = float(
                    np.linalg.norm(tcp_pos - data.geom_xpos[gid])
                    - self.model.geom_rbound[gid]
                )
                if dist < best_dist:
                    best_dist = dist
                    best_obj = obj

        if best_obj is None or best_dist > self.tool_pack.suction_range_m:
            return

        eq_id = self._suction_welds[best_obj.body]
        bid2 = self.model.body(best_obj.body).id
        r1 = data.xmat[attach_bid].reshape(3, 3)
        p_rel = r1.T @ (data.xpos[bid2] - data.xpos[attach_bid])
        q1_inv = np.zeros(4)
        q_rel = np.zeros(4)
        mujoco.mju_negQuat(q1_inv, data.xquat[attach_bid])
        mujoco.mju_mulQuat(q_rel, q1_inv, data.xquat[bid2])
        self.model.eq_data[eq_id, 0:3] = 0.0
        self.model.eq_data[eq_id, 3:6] = p_rel
        self.model.eq_data[eq_id, 6:10] = q_rel
        self.model.eq_data[eq_id, 10] = 1.0
        data.eq_active[eq_id] = 1
        self._attached_object = best_obj.name
        self._io["vacuum_engaged"] = True

    def reset(
        self, data: mujoco.MjData, seed: int | None = None
    ) -> dict[str, dict[str, Any]]:
        """Reset free objects (with optional jittered seed), spring joints, IO, and stages."""
        if seed is not None and (
            isinstance(seed, bool) or not isinstance(seed, int) or seed < 0
        ):
            raise ValueError("seed must be a non-negative integer or None")
        self.seed = seed
        rng = np.random.default_rng(seed) if seed is not None else None
        for adr, obj, def_q in zip(
            self._free_qpos, self.objects, self._obj_default_quats
        ):
            pos = np.array(obj.default_pos, dtype=float)
            if rng is not None and obj.jitter_xy > 0:
                pos[:2] += rng.uniform(-obj.jitter_xy, obj.jitter_xy, size=2)
            data.qpos[adr : adr + 3] = pos
            data.qpos[adr + 3 : adr + 7] = (
                [1.0, 0.0, 0.0, 0.0] if len(def_q) != 4 else def_q
            )
            dof = self.model.jnt_dofadr[
                self.model.body_jntadr[self.model.body(obj.body).id]
            ]
            data.qvel[dof : dof + 6] = 0.0

        for adr, q0 in self._scene_joints:
            data.qpos[adr] = q0

        self._io = dict(self._io_defaults)
        self._release_suction(data)
        if self.evaluator is not None:
            self.evaluator.reset()
        mujoco.mj_forward(self.model, data)
        return self.state(data)

    def step_io(self, data: mujoco.MjData) -> None:
        """Apply active virtual tool I/O physics (suction weld / spindle) and read scene inputs."""
        if self.tool_pack.kind == "suction":
            if bool(self._io.get("vacuum", False)):
                self._try_engage_suction(data)
            elif self._attached_object is not None or any(
                bool(data.eq_active[e]) for e in self._suction_welds.values()
            ):
                self._release_suction(data)

        if "brew_button_pressed" in self._io:
            try:
                jid = self.model.joint("brew_btn_joint").id
                pressed = float(data.qpos[self.model.jnt_qposadr[jid]]) <= -0.005
                self._io["brew_button_pressed"] = pressed
                if pressed:
                    self._io["brew_led"] = True
            except KeyError:
                pass

    def io_state(self, data: mujoco.MjData) -> dict[str, Any]:
        self.step_io(data)
        return dict(self._io)

    def set_io(self, updates: dict[str, Any], data: mujoco.MjData) -> dict[str, Any]:
        for k, v in updates.items():
            self._io[str(k)] = v
        self.step_io(data)
        return dict(self._io)

    def _grasped_set(self, data: mujoco.MjData) -> set[str]:
        held: set[str] = set()
        if self._attached_object is not None:
            held.add(self._attached_object)

        geom_to_name: dict[int, str] = {}
        for obj, bid in zip(self.objects, self.body_ids):
            for g in self.body_geom_ids.get(bid, []):
                geom_to_name[g] = obj.name

        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in geom_to_name and g2 in self._tool_contact_geoms:
                held.add(geom_to_name[g1])
            elif g2 in geom_to_name and g1 in self._tool_contact_geoms:
                held.add(geom_to_name[g2])

        for adr, obj in zip(self._free_qpos, self.objects):
            if float(data.qpos[adr + 2]) >= float(obj.default_pos[2]) + LIFTED_M:
                try:
                    tcp = data.site("tcp").xpos
                    if float(np.linalg.norm(data.qpos[adr : adr + 3] - tcp)) <= 0.08:
                        held.add(obj.name)
                except KeyError:
                    pass
        return held

    def state(self, data: mujoco.MjData) -> dict[str, dict[str, Any]]:
        grasped = self._grasped_set(data)
        out: dict[str, dict[str, Any]] = {}
        for obj, adr in zip(self.objects, self._free_qpos):
            out[obj.name] = {
                "pos": data.qpos[adr : adr + 3].round(5).tolist(),
                "quat": data.qpos[adr + 3 : adr + 7].round(5).tolist(),
                "grasped": obj.name in grasped,
            }
        return out

    def task_state(self, data: mujoco.MjData) -> dict[str, Any] | None:
        if self.evaluator is None:
            return None
        obj_st = self.state(data)
        io_st = self.io_state(data)
        return self.evaluator.evaluate(data, obj_st, io_st)
