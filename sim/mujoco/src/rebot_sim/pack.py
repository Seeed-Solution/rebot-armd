"""Composable scene packs, custom end-effectors, virtual I/O, and task evaluation."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import struct
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .scene import JAW_GEOMS, LIFTED_M

PACK_SLUG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")

MAX_MANIFEST_BYTES = 1024 * 1024  # 1 MiB
MAX_XML_BYTES = 512 * 1024  # 512 KiB
MAX_XML_DEPTH = 32
MAX_XML_ELEMENTS = 4_096
MAX_PACK_BODIES = 512
MAX_PACK_GEOMS = 1_024
MAX_PACK_TOTAL_BYTES = 10 * 1024 * 1024  # 10 MiB
MAX_SINGLE_MESH_BYTES = 4 * 1024 * 1024  # 4 MiB
MAX_MESH_FILES = 16
MAX_TOTAL_VERTICES = 150_000

_FORBIDDEN_XML_TAGS = frozenset(
    {
        "include",
        "plugin",
        "custom",
        "extension",
        "replicate",
        "frame",
        "composite",
        "flexcomp",
        "attach",
        "model",
    }
)
_PATH_ATTRS = frozenset(
    {
        "file",
        "fileright",
        "fileleft",
        "fileup",
        "filedown",
        "filefront",
        "fileback",
        "meshdir",
        "texturedir",
        "assetdir",
        "filename",
    }
)
_TEXTURE_FILE_ATTRS = (
    "file",
    "fileright",
    "fileleft",
    "fileup",
    "filedown",
    "filefront",
    "fileback",
)
_ALLOWED_OBJ_DIRECTIVES = frozenset({"v", "vn", "vt", "f", "g", "o", "s", "#", "usemtl"})
_ALLOWED_STL_KEYWORDS = frozenset(
    {"solid", "endsolid", "facet", "endfacet", "outer", "endloop", "vertex"}
)


class PackValidationError(ValueError):
    """Raised when a scene or tool pack fails security, format, or budget validation."""

    def __init__(self, message: str, code: str = "INVALID_PACK", status: int = 400):
        super().__init__(message)
        self.code = code
        self.status = status


def _ascii_lower(s: str) -> str:
    return "".join(chr(ord(c) + 32) if "A" <= c <= "Z" else c for c in s)


def normalize_robot_id(raw: str | None) -> str | None:
    """Normalize robot/profile identifiers to canonical IDs ('b601dm', 'so-arm101')."""
    if raw is None or not isinstance(raw, str):
        return None
    norm = _ascii_lower(raw.strip())
    if not norm:
        return None
    collapsed = norm.replace("_", "-")
    if collapsed in {"b601dm", "b601-dm", "rebot-dm", "rebot-b601dm", "rebot-b601-dm"}:
        return "b601dm"
    if collapsed in {"so-arm101", "soarm101", "so101", "so-101"}:
        return "so-arm101"
    return collapsed


def is_valid_pack_slug(val: Any) -> bool:
    """Return True if val is a strict slug ID (1..64 chars, starting with alphanumeric), empty, or 'default'."""
    if val is None:
        return True
    if not isinstance(val, str):
        return False
    if not val or val in {"default", "default_world"}:
        return True
    return bool(PACK_SLUG_RE.fullmatch(val))


def _is_unsafe_rel_path(val: str, *, allow_dot_dir: bool = False) -> bool:
    s = val.strip()
    if not s or "\x00" in s:
        return True
    if s.startswith(("/", "\\")):
        return True
    if re.match(r"^[A-Za-z]:", s):
        return True
    if re.match(r"^[A-Za-z][A-Za-z0-9+.-]*://", s):
        return True
    norm = s.replace("\\", "/")
    if allow_dot_dir:
        norm = norm.rstrip("/")
        if norm == ".":
            return False
    parts = norm.split("/")
    return any(seg in {"", ".", ".."} or seg.startswith(".") for seg in parts)


def _local_tag(tag: str) -> str:
    if "}" in tag:
        tag = tag.split("}", 1)[1]
    return tag.lower()


def _check_contained(root: Path, candidate: Path, label: str) -> Path:
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise PackValidationError(
            f"{label} does not exist or cannot be resolved: {exc}",
            code="INVALID_PACK",
        ) from None
    root_resolved = root.resolve()
    if resolved != root_resolved and not resolved.is_relative_to(root_resolved):
        raise PackValidationError(
            f"{label} escapes pack directory via symlink",
            code="PATH_ESCAPE",
        )
    return resolved


def _validate_binary_stl(data: bytes, label: str, vertex_budget_used: int) -> int:
    if len(data) < 84:
        raise PackValidationError(f"Truncated binary STL {label}", code="INVALID_PACK")
    tri_count = struct.unpack_from("<I", data, 80)[0]
    expected_len = 84 + 50 * tri_count
    if len(data) != expected_len:
        raise PackValidationError(
            f"Binary STL {label} byte length {len(data)} != expected {expected_len}",
            code="INVALID_PACK",
        )
    verts = tri_count * 3
    if vertex_budget_used + verts > MAX_TOTAL_VERTICES:
        raise PackValidationError(
            f"Total vertex count {vertex_budget_used + verts} exceeds budget {MAX_TOTAL_VERTICES}",
            code="PACK_TOO_LARGE",
        )
    for i in range(tri_count):
        vals = struct.unpack_from("<12f", data, 84 + i * 50)
        if not all(math.isfinite(v) for v in vals):
            raise PackValidationError(
                f"Non-finite float in binary STL {label} triangle {i}",
                code="INVALID_PACK",
            )
    return verts


def _validate_ascii_stl(data: bytes, label: str, vertex_budget_used: int) -> int:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise PackValidationError(
            f"Invalid UTF-8 in ASCII STL {label}: {exc}", code="INVALID_PACK"
        ) from None
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if not lines:
        raise PackValidationError(f"Empty ASCII STL {label}", code="INVALID_PACK")
    if lines[0].split()[0].lower() != "solid":
        raise PackValidationError(
            f"ASCII STL {label} must start with 'solid'", code="INVALID_PACK"
        )
    if lines[-1].split()[0].lower() != "endsolid":
        raise PackValidationError(
            f"ASCII STL {label} must end with 'endsolid'", code="INVALID_PACK"
        )
    verts = 0
    # Strict ASCII STL state machine: solid -> (facet -> outer loop -> 3x vertex -> endloop -> endfacet)* -> endsolid
    state = "EXPECT_SOLID"
    loop_verts = 0
    for ln in lines:
        parts = ln.split()
        kw = parts[0].lower()
        if kw not in _ALLOWED_STL_KEYWORDS:
            raise PackValidationError(
                f"Unexpected directive {kw!r} in ASCII STL {label}",
                code="INVALID_PACK",
            )
        if kw == "solid":
            if state != "EXPECT_SOLID":
                raise PackValidationError(
                    f"Unexpected 'solid' in ASCII STL {label}", code="INVALID_PACK"
                )
            state = "IN_SOLID"
        elif kw == "endsolid":
            if state != "IN_SOLID":
                raise PackValidationError(
                    f"Unexpected 'endsolid' while in state {state} in ASCII STL {label}",
                    code="INVALID_PACK",
                )
            state = "DONE"
        elif kw == "facet":
            if state != "IN_SOLID" or len(parts) != 5 or parts[1].lower() != "normal":
                raise PackValidationError(
                    f"Invalid or unexpected facet normal in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                )
            try:
                normals = [float(parts[2]), float(parts[3]), float(parts[4])]
            except ValueError:
                raise PackValidationError(
                    f"Non-numeric normal in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                ) from None
            if not all(math.isfinite(v) for v in normals):
                raise PackValidationError(
                    f"Non-finite normal in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                )
            state = "IN_FACET"
        elif kw == "outer":
            if state != "IN_FACET" or len(parts) != 2 or parts[1].lower() != "loop":
                raise PackValidationError(
                    f"Invalid or unexpected outer loop in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                )
            state = "IN_LOOP"
            loop_verts = 0
        elif kw == "vertex":
            if state != "IN_LOOP" or len(parts) != 4:
                raise PackValidationError(
                    f"Invalid or unexpected vertex line in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                )
            try:
                coords = [float(parts[1]), float(parts[2]), float(parts[3])]
            except ValueError:
                raise PackValidationError(
                    f"Non-numeric vertex in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                ) from None
            if not all(math.isfinite(v) for v in coords):
                raise PackValidationError(
                    f"Non-finite vertex in ASCII STL {label}: {ln!r}",
                    code="INVALID_PACK",
                )
            loop_verts += 1
            if loop_verts > 3:
                raise PackValidationError(
                    f"Too many vertices in loop in ASCII STL {label}",
                    code="INVALID_PACK",
                )
            verts += 1
            if vertex_budget_used + verts > MAX_TOTAL_VERTICES:
                raise PackValidationError(
                    f"Total vertex count {vertex_budget_used + verts} exceeds budget {MAX_TOTAL_VERTICES}",
                    code="PACK_TOO_LARGE",
                )
        elif kw == "endloop":
            if state != "IN_LOOP" or loop_verts != 3:
                raise PackValidationError(
                    f"Invalid endloop (expected 3 vertices, got {loop_verts}) in ASCII STL {label}",
                    code="INVALID_PACK",
                )
            state = "AFTER_LOOP"
        elif kw == "endfacet":
            if state != "AFTER_LOOP":
                raise PackValidationError(
                    f"Unexpected 'endfacet' in ASCII STL {label}", code="INVALID_PACK"
                )
            state = "IN_SOLID"
    if state != "DONE":
        raise PackValidationError(
            f"Incomplete ASCII STL {label} (ended in state {state})",
            code="INVALID_PACK",
        )
    return verts


def _read_mesh_bytes(path: Path, label: str) -> bytes:
    try:
        size = path.stat().st_size
    except OSError as exc:
        raise PackValidationError(f"Cannot stat mesh {label}: {exc}", code="INVALID_PACK") from None
    if size > MAX_SINGLE_MESH_BYTES:
        raise PackValidationError(
            f"Mesh {label} size {size} exceeds {MAX_SINGLE_MESH_BYTES} bytes",
            code="PACK_TOO_LARGE",
        )
    return path.read_bytes()


def _validate_stl_file(path: Path, label: str, vertex_budget_used: int) -> int:
    data = _read_mesh_bytes(path, label)
    # Distinguish binary vs ASCII STL: check exact binary STL length BEFORE startswith(b"solid")
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * tri_count:
            return _validate_binary_stl(data, label, vertex_budget_used)
    stripped_l = data.lstrip()
    if stripped_l[:5].lower() == b"solid" and (
        b"facet" in data.lower()
        or b"endsolid" in data.lower()
        or b"vertex" in data.lower()
        or len(data) < 84
    ):
        return _validate_ascii_stl(data, label, vertex_budget_used)
    return _validate_binary_stl(data, label, vertex_budget_used)


_OBJ_VEC_SPECS: dict[str, tuple[str, tuple[int, ...]]] = {
    "v": ("vertex", (4, 5)),
    "vn": ("normal", (4,)),
    "vt": ("texcoord", (2, 3, 4)),
}


def _check_obj_index(
    raw_idx: int, total_count: int, cur_count: int, kind: str, label: str
) -> None:
    if (raw_idx > 0 and raw_idx > total_count) or (raw_idx < 0 and -raw_idx > cur_count):
        raise PackValidationError(
            f"Out-of-bounds {kind} index {raw_idx} in OBJ {label}",
            code="INVALID_PACK",
        )


def _validate_obj_file(path: Path, label: str, vertex_budget_used: int) -> int:
    raw = _read_mesh_bytes(path, label)
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise PackValidationError(
            f"Invalid UTF-8 in OBJ {label}: {exc}", code="INVALID_PACK"
        ) from None
    verts = 0
    vt_count = 0
    vn_count = 0
    faces: list[tuple[int, int, int, list[str]]] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        directive = parts[0]
        if directive == "mtllib":
            raise PackValidationError(
                f"Forbidden 'mtllib' directive in OBJ {label}", code="INVALID_PACK"
            )
        if directive not in _ALLOWED_OBJ_DIRECTIVES:
            raise PackValidationError(
                f"Forbidden directive {directive!r} in OBJ {label}",
                code="INVALID_PACK",
            )
        if directive in _OBJ_VEC_SPECS:
            kind_name, valid_lens = _OBJ_VEC_SPECS[directive]
            if len(parts) not in valid_lens:
                raise PackValidationError(
                    f"Invalid {kind_name} line in OBJ {label}: {stripped!r}",
                    code="INVALID_PACK",
                )
            try:
                coords = [float(x) for x in parts[1:]]
            except ValueError:
                raise PackValidationError(
                    f"Non-numeric coordinate in OBJ {label}: {stripped!r}",
                    code="INVALID_PACK",
                ) from None
            if not all(math.isfinite(v) for v in coords):
                raise PackValidationError(
                    f"Non-finite coordinate in OBJ {label}: {stripped!r}",
                    code="INVALID_PACK",
                )
            if directive == "v":
                verts += 1
                if vertex_budget_used + verts > MAX_TOTAL_VERTICES:
                    raise PackValidationError(
                        f"Total vertex count {vertex_budget_used + verts} exceeds budget {MAX_TOTAL_VERTICES}",
                        code="PACK_TOO_LARGE",
                    )
            elif directive == "vt":
                vt_count += 1
            else:
                vn_count += 1
        elif directive == "f":
            if len(parts) < 4:
                raise PackValidationError(
                    f"Invalid face line in OBJ {label}: {stripped!r}",
                    code="INVALID_PACK",
                )
            for tok in parts[1:]:
                idx_parts = tok.split("/")
                if len(idx_parts) > 3 or not idx_parts[0]:
                    raise PackValidationError(
                        f"Invalid face vertex index {tok!r} in OBJ {label}",
                        code="INVALID_PACK",
                    )
                for p in idx_parts:
                    if not p:
                        continue
                    try:
                        val = int(p)
                    except ValueError:
                        raise PackValidationError(
                            f"Invalid face index {tok!r} in OBJ {label}",
                            code="INVALID_PACK",
                        ) from None
                    if val == 0:
                        raise PackValidationError(
                            f"Zero face index {tok!r} in OBJ {label}",
                            code="INVALID_PACK",
                        )
            faces.append((verts, vt_count, vn_count, parts[1:]))

    for cur_v, cur_vt, cur_vn, ftoks in faces:
        for tok in ftoks:
            idx_parts = tok.split("/")
            _check_obj_index(int(idx_parts[0]), verts, cur_v, "vertex", label)
            if len(idx_parts) >= 2 and idx_parts[1]:
                _check_obj_index(int(idx_parts[1]), vt_count, cur_vt, "texcoord", label)
            if len(idx_parts) == 3 and idx_parts[2]:
                _check_obj_index(int(idx_parts[2]), vn_count, cur_vn, "normal", label)
    return verts


def _validate_mesh_file(path: Path, label: str, vertex_budget_used: int) -> int:
    suffix = path.suffix.lower()
    if suffix == ".urdf":
        raise PackValidationError(
            f"URDF files are not supported: {label}", code="UNSUPPORTED_FORMAT"
        )
    if suffix == ".stl":
        return _validate_stl_file(path, label, vertex_budget_used)
    if suffix == ".obj":
        return _validate_obj_file(path, label, vertex_budget_used)
    raise PackValidationError(
        f"Unsupported mesh format {suffix!r} for {label}", code="UNSUPPORTED_FORMAT"
    )


def _walk_pack_files(root: Path, *, check_urdf: bool = True) -> list[tuple[str, Path]]:
    """Walk pack directory matching Rust enumeration: skip hidden entries, reject symlinks escaping root."""
    root_resolved = root.resolve()
    files: list[tuple[str, Path]] = []
    for dirpath, dirnames, filenames in os.walk(root_resolved, followlinks=False):
        dirnames[:] = sorted(d for d in dirnames if not d.startswith("."))
        filenames = sorted(f for f in filenames if not f.startswith("."))
        cur = Path(dirpath)
        for dname in list(dirnames):
            dp = cur / dname
            if dp.is_symlink():
                _check_contained(root_resolved, dp, dp.relative_to(root_resolved).as_posix())
                # Do not traverse symlinked directories repeatedly; reject or verify contained
                dirnames.remove(dname)
        for fname in filenames:
            fp = cur / fname
            rel = fp.relative_to(root_resolved).as_posix()
            if check_urdf and fname.lower().endswith(".urdf"):
                raise PackValidationError(
                    f"URDF files are not supported: {rel}", code="UNSUPPORTED_FORMAT"
                )
            resolved = _check_contained(root_resolved, fp, rel)
            if resolved.is_file():
                if check_urdf and resolved.suffix.lower() == ".urdf":
                    raise PackValidationError(
                        f"URDF files are not supported: {rel}", code="UNSUPPORTED_FORMAT"
                    )
                files.append((rel, resolved))
    files.sort(key=lambda item: item[0])
    return files


def _check_element_budgets(elements: int, bodies: int, geoms: int, label: str) -> None:
    if elements > MAX_XML_ELEMENTS:
        raise PackValidationError(
            f"XML element count {elements} exceeds budget {MAX_XML_ELEMENTS} in {label}",
            code="INVALID_PACK",
        )
    if bodies > MAX_PACK_BODIES:
        raise PackValidationError(
            f"Body count {bodies} exceeds budget {MAX_PACK_BODIES} in {label}",
            code="INVALID_PACK",
        )
    if geoms > MAX_PACK_GEOMS:
        raise PackValidationError(
            f"Geom count {geoms} exceeds budget {MAX_PACK_GEOMS} in {label}",
            code="INVALID_PACK",
        )


def _validate_xml_text_tree(
    xml_text: str,
    label: str,
    *,
    root_resolved: Path | None = None,
    xml_parent: Path | None = None,
    vertex_budget_used: int = 0,
    mesh_count_used: int = 0,
    element_budget_used: int = 0,
    body_budget_used: int = 0,
    geom_budget_used: int = 0,
) -> tuple[
    list[tuple[str, str, str]],
    int,
    int,
    int,
    int,
    int,
    Path | None,
    Path | None,
    Path | None,
    bool,
    bool,
]:
    upper = xml_text.upper()
    if "<!DOCTYPE" in upper:
        raise PackValidationError(
            f"Forbidden <!DOCTYPE construct in {label}", code="INVALID_PACK"
        )
    if "<!ENTITY" in upper:
        raise PackValidationError(
            f"Forbidden <!ENTITY construct in {label}", code="INVALID_PACK"
        )

    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        raise PackValidationError(
            f"Malformed XML in {label}: {exc}", code="INVALID_PACK"
        ) from None

    if _local_tag(root.tag) == "robot":
        raise PackValidationError(
            f"URDF <robot> XML is not supported: {label}",
            code="UNSUPPORTED_FORMAT",
        )

    mesh_base = xml_parent
    tex_base = xml_parent
    asset_base = xml_parent
    has_meshdir = False
    has_texturedir = False

    ref_elements: list[tuple[str, str, str]] = []
    inline_mesh_count = 0
    inline_verts = 0
    elem_count = 0
    body_count = 0
    geom_count = 0

    stack: list[tuple[ET.Element, int]] = [(root, 1)]
    while stack:
        el, depth = stack.pop()
        if depth > MAX_XML_DEPTH:
            raise PackValidationError(
                f"XML nesting depth {depth} exceeds limit {MAX_XML_DEPTH} in {label}",
                code="INVALID_PACK",
            )
        tag = _local_tag(el.tag)
        if tag in _FORBIDDEN_XML_TAGS:
            raise PackValidationError(
                f"Forbidden XML element <{tag}> in {label}",
                code="INVALID_PACK",
            )
        elem_count += 1
        if tag == "body":
            body_count += 1
        elif tag == "geom":
            geom_count += 1
        _check_element_budgets(elem_count, body_count, geom_count, label)
        _check_element_budgets(
            element_budget_used + elem_count,
            body_budget_used + body_count,
            geom_budget_used + geom_count,
            label,
        )
        for attr, val in el.attrib.items():
            attr_l = _local_tag(attr)
            if attr_l in _PATH_ATTRS:
                is_dir_attr = attr_l in {"meshdir", "texturedir", "assetdir"}
                if _is_unsafe_rel_path(val, allow_dot_dir=is_dir_attr):
                    raise PackValidationError(
                        f"Unsafe path {val!r} in attribute {attr_l!r} of <{tag}>",
                        code="PATH_ESCAPE",
                    )
                if val.strip().lower().endswith(".urdf"):
                    raise PackValidationError(
                        f"URDF reference {val!r} is not supported",
                        code="UNSUPPORTED_FORMAT",
                    )

        if tag == "compiler" and root_resolved is not None and xml_parent is not None:
            for dir_attr in ("assetdir", "meshdir", "texturedir"):
                if dir_attr in el.attrib:
                    d_val = el.attrib[dir_attr].strip()
                    d_resolved = _check_contained(
                        root_resolved, xml_parent / d_val, f"{dir_attr}={d_val!r}"
                    )
                    if not d_resolved.is_dir():
                        raise PackValidationError(
                            f"Compiler {dir_attr}={d_val!r} is not a directory",
                            code="INVALID_PACK",
                        )
                    if dir_attr == "assetdir":
                        asset_base = d_resolved
                    elif dir_attr == "meshdir":
                        mesh_base = d_resolved
                        has_meshdir = True
                    else:
                        tex_base = d_resolved
                        has_texturedir = True

        if tag == "mesh" and "vertex" in el.attrib:
            raw_v = el.attrib["vertex"].strip()
            toks = raw_v.split()
            try:
                floats = [float(t) for t in toks]
            except ValueError:
                raise PackValidationError(
                    f"Non-numeric inline mesh vertex data in {label}",
                    code="INVALID_PACK",
                ) from None
            if not floats or len(floats) % 3 != 0 or not all(math.isfinite(x) for x in floats):
                raise PackValidationError(
                    f"Invalid inline mesh vertex data in {label}",
                    code="INVALID_PACK",
                )
            inline_mesh_count += 1
            if mesh_count_used + inline_mesh_count > MAX_MESH_FILES:
                raise PackValidationError(
                    f"Mesh count {mesh_count_used + inline_mesh_count} exceeds limit {MAX_MESH_FILES}",
                    code="PACK_TOO_LARGE",
                )
            v_add = len(floats) // 3
            inline_verts += v_add
            if vertex_budget_used + inline_verts > MAX_TOTAL_VERTICES:
                raise PackValidationError(
                    f"Total vertex count {vertex_budget_used + inline_verts} exceeds budget {MAX_TOTAL_VERTICES}",
                    code="PACK_TOO_LARGE",
                )

        if tag in {"mesh", "hfield", "skin", "model"} and "file" in el.attrib:
            ref_elements.append((tag, "file", el.attrib["file"].strip()))
        elif tag == "texture":
            for t_attr in _TEXTURE_FILE_ATTRS:
                if t_attr in el.attrib:
                    ref_elements.append((tag, t_attr, el.attrib[t_attr].strip()))

        for child in reversed(list(el)):
            stack.append((child, depth + 1))

    return (
        ref_elements,
        inline_mesh_count,
        inline_verts,
        elem_count,
        body_count,
        geom_count,
        mesh_base,
        tex_base,
        asset_base,
        has_meshdir,
        has_texturedir,
    )


def validate_xml_file_security(
    xml_path: Path,
    root_dir: Path,
    *,
    vertex_budget_used: int = 0,
    mesh_count_used: int = 0,
    element_budget_used: int = 0,
    body_budget_used: int = 0,
    geom_budget_used: int = 0,
    validate_refs: bool = True,
    validate_mesh_files: bool = True,
    counts_out: dict[str, int] | None = None,
) -> tuple[list[Path], int, int, list[Path]]:
    """Validate XML security/budget rules, inline mesh budgets, and referenced asset closure within root_dir.

    Returns (referenced_meshes, inline_mesh_count, inline_verts + ref_mesh_verts, all_referenced_files).
    """
    if xml_path.suffix.lower() == ".urdf":
        raise PackValidationError(
            f"URDF files are not supported: {xml_path.name}", code="UNSUPPORTED_FORMAT"
        )
    root_resolved = root_dir.resolve()
    xml_resolved = _check_contained(root_resolved, xml_path, xml_path.name)
    try:
        xml_size = xml_resolved.stat().st_size
    except OSError as exc:
        raise PackValidationError(
            f"Cannot stat XML file {xml_path.name}: {exc}", code="INVALID_PACK"
        ) from None
    if xml_size > MAX_XML_BYTES:
        raise PackValidationError(
            f"XML file {xml_path.name} size {xml_size} exceeds {MAX_XML_BYTES} bytes",
            code="PACK_TOO_LARGE",
        )
    raw_bytes = xml_resolved.read_bytes()
    if len(raw_bytes) > MAX_XML_BYTES:
        raise PackValidationError(
            f"XML file {xml_path.name} size {len(raw_bytes)} exceeds {MAX_XML_BYTES} bytes",
            code="PACK_TOO_LARGE",
        )
    try:
        xml_text = raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise PackValidationError(
            f"XML file {xml_path.name} is not valid UTF-8: {exc}", code="INVALID_PACK"
        ) from None

    (
        ref_elements,
        inline_mesh_count,
        inline_verts,
        elem_count,
        body_count,
        geom_count,
        mesh_base,
        tex_base,
        asset_base,
        has_meshdir,
        has_texturedir,
    ) = _validate_xml_text_tree(
        xml_text,
        xml_path.name,
        root_resolved=root_resolved,
        xml_parent=xml_resolved.parent,
        vertex_budget_used=vertex_budget_used,
        mesh_count_used=mesh_count_used,
        element_budget_used=element_budget_used,
        body_budget_used=body_budget_used,
        geom_budget_used=geom_budget_used,
    )
    if counts_out is not None:
        counts_out["elements"] = elem_count
        counts_out["bodies"] = body_count
        counts_out["geoms"] = geom_count

    # Pass 2: resolve references relative to XML's own parent dir / compiler dirs
    referenced_meshes: list[Path] = []
    all_referenced_files: list[Path] = []
    verts_added = inline_verts
    if validate_refs:
        for tag, attr_name, rel_file in ref_elements:
            if tag == "mesh":
                base = mesh_base if has_meshdir else asset_base
            elif tag == "texture":
                base = tex_base if has_texturedir else asset_base
            else:
                base = asset_base
            assert base is not None
            resolved_ref = _check_contained(
                root_resolved, base / rel_file, f"<{tag} {attr_name}={rel_file!r}>"
            )
            if not resolved_ref.is_file():
                raise PackValidationError(
                    f"Referenced <{tag} {attr_name}={rel_file!r}> is not a regular file",
                    code="INVALID_PACK",
                )
            if resolved_ref not in all_referenced_files:
                all_referenced_files.append(resolved_ref)
            if tag == "mesh":
                if resolved_ref not in referenced_meshes:
                    referenced_meshes.append(resolved_ref)
                if validate_mesh_files:
                    verts_added += _validate_mesh_file(
                        resolved_ref, rel_file, vertex_budget_used + verts_added
                    )

    return referenced_meshes, inline_mesh_count, verts_added, all_referenced_files


def _hash_pack_entries(
    entries: list[tuple[str, Path]], *, max_file_bytes: int | None = None
) -> str:
    h = hashlib.sha256()
    for rel, resolved in entries:
        if max_file_bytes is not None and resolved.stat().st_size > max_file_bytes:
            raise PackValidationError(
                f"File {rel} exceeds max pack size {max_file_bytes} bytes",
                code="PACK_TOO_LARGE",
            )
        data = resolved.read_bytes()
        h.update(rel.encode("utf-8"))
        h.update(b"\x00")
        h.update(len(data).to_bytes(8, "little"))
        h.update(data)
    return h.hexdigest()


def compute_pack_digest(pack_dir: Path | str) -> str:
    """Compute deterministic SHA-256 hex digest over sorted relative paths and file bytes."""
    root = Path(pack_dir).resolve()
    if root.is_file():
        return _hash_pack_entries([(root.name, root)])
    files = _walk_pack_files(root, check_urdf=False)
    return _hash_pack_entries(files, max_file_bytes=MAX_PACK_TOTAL_BYTES)


def _compute_standalone_xml_digest(xml_path: Path, ref_files: list[Path]) -> str:
    root = xml_path.parent.resolve()
    entries: list[tuple[str, Path]] = []
    seen: set[Path] = set()
    for p in [xml_path.resolve(), *ref_files]:
        rp = p.resolve()
        if rp in seen:
            continue
        seen.add(rp)
        entries.append((rp.relative_to(root).as_posix(), rp))
    entries.sort(key=lambda item: item[0])
    return _hash_pack_entries(entries)


def _validate_standalone_xml(xml_path: Path) -> str:
    """Validate standalone XML slug, security, aggregate asset budgets, and compute digest."""
    if not PACK_SLUG_RE.fullmatch(xml_path.stem):
        raise PackValidationError(
            f"Invalid standalone pack id {xml_path.stem!r}: must match {PACK_SLUG_RE.pattern}",
            code="INVALID_PACK",
        )
    root = xml_path.parent.resolve()
    ref_meshes, inline_count, total_verts, all_refs = validate_xml_file_security(
        xml_path, root, validate_refs=True, validate_mesh_files=True
    )
    if inline_count + len(ref_meshes) > MAX_MESH_FILES:
        raise PackValidationError(
            f"Mesh count {inline_count + len(ref_meshes)} exceeds limit {MAX_MESH_FILES}",
            code="PACK_TOO_LARGE",
        )
    if total_verts > MAX_TOTAL_VERTICES:
        raise PackValidationError(
            f"Total vertex count {total_verts} exceeds budget {MAX_TOTAL_VERTICES}",
            code="PACK_TOO_LARGE",
        )
    total_bytes = xml_path.stat().st_size + sum(p.stat().st_size for p in all_refs)
    if total_bytes > MAX_PACK_TOTAL_BYTES:
        raise PackValidationError(
            f"Standalone XML pack size {total_bytes} exceeds budget {MAX_PACK_TOTAL_BYTES} bytes",
            code="PACK_TOO_LARGE",
        )
    return _compute_standalone_xml_digest(xml_path, all_refs)


def _extract_manifest_asset(entry: Any) -> tuple[str | None, bool]:
    if isinstance(entry, str):
        return entry, True
    if not isinstance(entry, dict):
        return None, True
    required = bool(entry.get("required", True))
    for k in ("path", "target", "filename"):
        if isinstance(entry.get(k), str) and entry[k].strip():
            return entry[k], required
    return None, required


def validate_pack_security(
    pack_dir: Path | str,
    kind: str,
    manifest: dict[str, Any] | None = None,
    *,
    expected_digest: str | None = None,
) -> str:
    """Validate pack ID, XML security/budgets, mesh budgets/geometry, and reference closure.

    Returns the deterministic SHA-256 pack digest.
    """
    raw_path = Path(pack_dir)
    if str(pack_dir).strip().lower().endswith(".urdf"):
        raise PackValidationError(
            f"URDF files are not supported: {pack_dir}", code="UNSUPPORTED_FORMAT"
        )
    if not raw_path.exists():
        raise PackValidationError(
            f"Pack directory not found: {pack_dir}", code="PACK_NOT_FOUND"
        )
    root = raw_path.resolve()
    if not root.is_dir():
        raise PackValidationError(
            f"Pack path is not a directory: {pack_dir}", code="INVALID_PACK"
        )

    is_scene = kind in {"scene", "scenes"}
    manifest_name = "scene.json" if is_scene else "tool.json"
    if manifest is None and (root / manifest_name).is_file():
        manifest_file = _check_contained(root, root / manifest_name, manifest_name)
        if manifest_file.stat().st_size > MAX_MANIFEST_BYTES:
            raise PackValidationError(
                f"Manifest {manifest_name} size exceeds {MAX_MANIFEST_BYTES} bytes",
                code="PACK_TOO_LARGE",
            )
        try:
            manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise PackValidationError(
                f"Invalid {manifest_name}: {exc}", code="INVALID_PACK"
            ) from None
        if not isinstance(manifest, dict):
            raise PackValidationError(
                f"{manifest_name} must be a JSON object", code="INVALID_PACK"
            )

    pack_id = manifest.get("id") if isinstance(manifest, dict) else root.name
    if not isinstance(pack_id, str) or not PACK_SLUG_RE.fullmatch(pack_id):
        raise PackValidationError(
            f"Invalid pack id {pack_id!r}: must match {PACK_SLUG_RE.pattern}",
            code="INVALID_PACK",
        )

    # Walk pack_dir: skip hidden entries, reject escaping symlinks & .urdf, check sizes before read
    walked_files = _walk_pack_files(root, check_urdf=True)
    total_bytes = 0
    mesh_files: dict[Path, str] = {}
    xml_files: list[Path] = []
    for rel, resolved in walked_files:
        total_bytes += resolved.stat().st_size
        if total_bytes > MAX_PACK_TOTAL_BYTES:
            raise PackValidationError(
                f"Total pack size {total_bytes} exceeds budget {MAX_PACK_TOTAL_BYTES} bytes",
                code="PACK_TOO_LARGE",
            )
        suf = resolved.suffix.lower()
        if suf in {".stl", ".obj"}:
            mesh_files[resolved] = rel
        elif suf == ".xml":
            xml_files.append(resolved)

    if len(mesh_files) > MAX_MESH_FILES:
        raise PackValidationError(
            f"Mesh file count {len(mesh_files)} exceeds limit {MAX_MESH_FILES}",
            code="PACK_TOO_LARGE",
        )

    # Validate required manifest assets
    if isinstance(manifest, dict) and isinstance(manifest.get("assets"), list):
        for entry in manifest["assets"]:
            rel_asset, required = _extract_manifest_asset(entry)
            if rel_asset is None:
                continue
            if _is_unsafe_rel_path(rel_asset):
                raise PackValidationError(
                    f"Unsafe asset path {rel_asset!r} in {manifest_name}",
                    code="PATH_ESCAPE",
                )
            if not required:
                continue
            resolved_asset = _check_contained(
                root, root / rel_asset.strip(), f"manifest asset {rel_asset!r}"
            )
            if not resolved_asset.is_file():
                raise PackValidationError(
                    f"Required manifest asset not found: {rel_asset!r}",
                    code="INVALID_PACK",
                )

    # Primary XML check
    default_xml = "world.xml" if is_scene else "tool.xml"
    xml_key = "world_xml" if is_scene else "tool_xml"
    xml_rel = str(manifest.get(xml_key, default_xml)) if isinstance(manifest, dict) else default_xml
    if _is_unsafe_rel_path(xml_rel):
        raise PackValidationError(
            f"Unsafe XML path {xml_rel!r} in {manifest_name}", code="PATH_ESCAPE"
        )
    if xml_rel.strip().lower().endswith(".urdf"):
        raise PackValidationError(
            f"URDF XML {xml_rel!r} is not supported", code="UNSUPPORTED_FORMAT"
        )
    primary_xml = _check_contained(root, root / xml_rel, xml_rel)
    if not primary_xml.is_file():
        raise PackValidationError(
            f"Pack XML file not found: {xml_rel}", code="INVALID_PACK"
        )
    if primary_xml not in xml_files:
        xml_files.append(primary_xml)

    # Validate ALL XML files in the pack (including inline mesh counts/vertices, element budgets, + reference closure)
    inline_mesh_total = 0
    total_vertices = 0
    total_elements = 0
    total_bodies = 0
    total_geoms = 0
    for xf in xml_files:
        xf_counts: dict[str, int] = {}
        ref_meshes, inline_cnt, inline_verts, _ = validate_xml_file_security(
            xf,
            root,
            vertex_budget_used=total_vertices,
            mesh_count_used=len(mesh_files) + inline_mesh_total,
            element_budget_used=total_elements,
            body_budget_used=total_bodies,
            geom_budget_used=total_geoms,
            validate_refs=True,
            validate_mesh_files=False,
            counts_out=xf_counts,
        )
        inline_mesh_total += inline_cnt
        total_vertices += inline_verts
        total_elements += xf_counts.get("elements", 0)
        total_bodies += xf_counts.get("bodies", 0)
        total_geoms += xf_counts.get("geoms", 0)
        for rm in ref_meshes:
            if rm not in mesh_files:
                mesh_files[rm] = rm.relative_to(root).as_posix()

    if len(mesh_files) + inline_mesh_total > MAX_MESH_FILES:
        raise PackValidationError(
            f"Mesh count {len(mesh_files) + inline_mesh_total} exceeds limit {MAX_MESH_FILES}",
            code="PACK_TOO_LARGE",
        )

    # Sum vertices across ALL mesh files in pack_dir (referenced or unreferenced) + inline meshes
    for mpath, mrel in sorted(mesh_files.items(), key=lambda item: item[1]):
        total_vertices += _validate_mesh_file(mpath, mrel, total_vertices)

    digest = compute_pack_digest(root)
    if expected_digest is not None and expected_digest.strip():
        if digest != expected_digest.strip().lower():
            raise PackValidationError(
                f"Pack digest mismatch: expected {expected_digest!r}, got {digest!r}",
                code="DIGEST_MISMATCH",
            )
    return digest


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


def resolve_pack_path(
    ref: str | Path | None, kind: str, *, prefer_disk: bool = False
) -> Path | None:
    """Resolve a local path or named built-in pack id ('coffee_kiosk', 'wide_cup_gripper')."""
    if ref is None:
        return None
    raw = str(ref).strip()
    if not prefer_disk and (not raw or raw in {"default", "default_world"}):
        return None
    if not raw:
        raise FileNotFoundError(f"Empty {kind} pack path: {ref!r}")
    if raw.lower().endswith(".urdf"):
        raise PackValidationError(
            f"URDF files are not supported: {ref!r}", code="UNSUPPORTED_FORMAT"
        )
    direct = Path(raw).expanduser()
    if direct.exists():
        resolved = direct.resolve()
        if resolved.suffix.lower() == ".urdf":
            raise PackValidationError(
                f"URDF files are not supported: {ref!r}", code="UNSUPPORTED_FORMAT"
            )
        return resolved
    if prefer_disk:
        raise FileNotFoundError(f"Missing {kind} directory: {ref!r}")
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
    robots: list[str] = field(default_factory=list)
    digest: str | None = None

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


def _load_manifest_json(path: Path, filename: str) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Expected {filename} or .xml file in {path.parent}")
    if path.stat().st_size > MAX_MANIFEST_BYTES:
        raise PackValidationError(
            f"{filename} size exceeds {MAX_MANIFEST_BYTES} bytes", code="PACK_TOO_LARGE"
        )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except ValueError as exc:
        raise PackValidationError(f"Invalid {filename}: {exc}", code="INVALID_PACK") from None
    if not isinstance(data, dict):
        raise PackValidationError(f"{filename} must be a JSON object", code="INVALID_PACK")
    return data


def _pack_root_dir(source_ref: str, xml_path: Path) -> Path:
    return (
        Path(source_ref)
        if source_ref and Path(source_ref).is_dir()
        else xml_path.parent
    )


def load_tool_pack(ref: str | Path | None, *, prefer_disk: bool = False) -> ToolPack:
    """Load a preset tool ('default', 'suction', 'screwdriver'), pack dir, tool.json, or raw XML."""
    raw = str(ref).strip() if ref is not None else "default"
    has_path_hint = (
        prefer_disk
        or isinstance(ref, Path)
        or os.sep in raw
        or (bool(os.altsep) and os.altsep in raw)
    )
    if not has_path_hint:
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

    path = resolve_pack_path(ref if prefer_disk else raw, "tool", prefer_disk=prefer_disk)
    if path is None:
        return ToolPack(id="default", name="Stock Parallel Jaw Gripper", kind="gripper")

    if path.is_file() and path.suffix.lower() == ".xml":
        digest = _validate_standalone_xml(path)
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
            digest=digest,
        )

    tool_json = (
        path if path.is_file() and path.name == "tool.json" else path / "tool.json"
    )
    data = _load_manifest_json(tool_json, "tool.json")
    tool_dir = tool_json.parent
    digest = validate_pack_security(tool_dir, "tool", manifest=data)
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
        robots=[str(r) for r in data.get("robots", [])],
        digest=digest,
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
    robots: list[str] = field(default_factory=list)
    digest: str | None = None

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


def load_scene_pack(
    ref: str | Path | None, *, prefer_disk: bool = False
) -> ScenePack | None:
    """Load a scene pack from built-in id, directory, scene.json, or standalone world XML."""
    path = resolve_pack_path(ref, "scene", prefer_disk=prefer_disk)
    if path is None:
        return None
    if path.is_file() and path.suffix.lower() == ".xml":
        digest = _validate_standalone_xml(path)
        return ScenePack(
            id=path.stem,
            name=path.stem.replace("_", " ").title(),
            world_xml_path=path,
            objects=_discover_freejoint_objects(path),
            source_ref=str(path),
            digest=digest,
        )
    scene_json = (
        path if path.is_file() and path.name == "scene.json" else path / "scene.json"
    )
    data = _load_manifest_json(scene_json, "scene.json")
    scene_dir = scene_json.parent
    digest = validate_pack_security(scene_dir, "scene", manifest=data)
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
        robots=[str(r) for r in data.get("robots", [])],
        digest=digest,
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
    scene_counts: dict[str, int] = {"elements": 0, "bodies": 0, "geoms": 0}
    tool_counts: dict[str, int] = {"elements": 0, "bodies": 0, "geoms": 0}
    scene_ref_meshes: list[Path] = []
    scene_root: Path | None = None

    if scene_pack is not None:
        scene_root = _pack_root_dir(scene_pack.source_ref, scene_pack.world_xml_path)
        scene_ref_meshes, _, _, _ = validate_xml_file_security(
            scene_pack.world_xml_path, scene_root, counts_out=scene_counts
        )

    if tool_pack is not None and not tool_pack.is_default:
        if tool_pack.tool_spec_xml is not None:
            _, _, _, t_el, t_bo, t_ge, *_ = _validate_xml_text_tree(
                tool_pack.tool_spec_xml, f"tool preset {tool_pack.id}"
            )
            tool_counts = {"elements": t_el, "bodies": t_bo, "geoms": t_ge}
        elif tool_pack.tool_xml_path is not None:
            tool_root = _pack_root_dir(tool_pack.source_ref, tool_pack.tool_xml_path)
            validate_xml_file_security(
                tool_pack.tool_xml_path, tool_root, counts_out=tool_counts
            )
        else:
            raise ValueError(
                f"ToolPack {tool_pack.id!r} has neither tool_xml_path nor tool_spec_xml"
            )

    _check_element_budgets(
        scene_counts["elements"] + tool_counts["elements"],
        scene_counts["bodies"] + tool_counts["bodies"],
        scene_counts["geoms"] + tool_counts["geoms"],
        "combined scene + tool packs",
    )

    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", message=".*empty prefix.*", category=UserWarning
        )

        if scene_pack is not None:
            assert scene_root is not None
            _delete_named(spec, "bodies", ["cube"])
            has_local_meshes = bool(scene_ref_meshes) or (
                bool(scene_pack.source_ref)
                and Path(scene_pack.source_ref).is_dir()
                and any(
                    p.suffix.lower() in {".stl", ".obj"}
                    for _, p in _walk_pack_files(scene_root, check_urdf=False)
                )
            )
            world_spec = mujoco.MjSpec.from_file(str(scene_pack.world_xml_path))
            cache_dir = _cache_scene_dir(scene_pack.id)
            if not has_local_meshes and cache_dir.is_dir() and not world_spec.meshdir:
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
