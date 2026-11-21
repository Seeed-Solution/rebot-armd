//! Pure MJCF composition and manifests.
use std::collections::BTreeMap;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Security & budget constants
// ---------------------------------------------------------------------------

pub const MAX_XML_BYTES: usize = 524_288; // 512 KiB
pub const MAX_XML_DEPTH: usize = 32;
pub const MAX_PACK_TOTAL_BYTES: usize = 10 * 1024 * 1024; // 10 MiB
pub const MAX_MESH_FILE_BYTES: usize = 4 * 1024 * 1024; // 4 MiB
pub const MAX_MESH_FILES: usize = 16;
pub const MAX_TOTAL_MESH_VERTICES: usize = 150_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PackValidationError {
    UnsupportedFormat(String),
    InvalidPack(String),
}

impl PackValidationError {
    pub fn code(&self) -> &'static str {
        match self {
            PackValidationError::UnsupportedFormat(_) => "UNSUPPORTED_FORMAT",
            PackValidationError::InvalidPack(_) => "INVALID_PACK",
        }
    }

    pub fn message(&self) -> &str {
        match self {
            PackValidationError::UnsupportedFormat(m) | PackValidationError::InvalidPack(m) => m,
        }
    }
}

impl std::fmt::Display for PackValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.message())
    }
}

// ---------------------------------------------------------------------------
// Path and ID validation
// ---------------------------------------------------------------------------

pub fn validate_pack_id(id: &str) -> bool {
    if id.is_empty() || id.len() > 64 {
        return false;
    }
    let b = id.as_bytes();
    if !b[0].is_ascii_alphanumeric() {
        return false;
    }
    b.iter().all(|&c| c.is_ascii_alphanumeric() || c == b'_' || c == b'-')
}

pub fn is_safe_relative_path(rel: &str) -> bool {
    if rel.is_empty() || rel.starts_with('/') || rel.starts_with('\\') {
        return false;
    }
    let segs: Vec<&str> = rel.split('/').collect();
    if segs.len() > 8 {
        return false;
    }
    segs.iter().all(|s| {
        !s.is_empty()
            && *s != "."
            && *s != ".."
            && !s.starts_with('.')
            && s.len() <= 128
            && s.bytes().all(|c| c.is_ascii_alphanumeric() || matches!(c, b'.' | b'_' | b'-'))
    })
}

pub fn is_urdf_path(path: &str) -> bool {
    path.trim().to_ascii_lowercase().ends_with(".urdf")
}

fn validate_attr_rel_path(attr_name: &str, raw: &str, allow_dot_dir: bool) -> Result<String, PackValidationError> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Err(PackValidationError::InvalidPack(format!(
            "XML attribute '{attr_name}' cannot be empty"
        )));
    }
    if trimmed.contains('\0')
        || trimmed.starts_with('/')
        || trimmed.starts_with('\\')
        || (trimmed.len() >= 2 && trimmed.as_bytes()[1] == b':')
        || trimmed.contains("://")
    {
        return Err(PackValidationError::InvalidPack(format!(
            "XML attribute '{attr_name}' must be a relative path within the pack: {trimmed:?}"
        )));
    }
    if is_urdf_path(trimmed) {
        return Err(PackValidationError::UnsupportedFormat(format!(
            "URDF references are not supported ({attr_name}={trimmed:?})"
        )));
    }
    let norm_slashes = trimmed.replace('\\', "/");
    let norm = if allow_dot_dir {
        norm_slashes.trim_end_matches('/')
    } else {
        norm_slashes.as_str()
    };
    if allow_dot_dir && norm == "." {
        return Ok(String::new());
    }
    if norm.split('/').any(|seg| seg == ".." || seg == "." || seg.is_empty() || seg.starts_with('.')) {
        return Err(PackValidationError::InvalidPack(format!(
            "XML attribute '{attr_name}' contains traversal or invalid path segment: {trimmed:?}"
        )));
    }
    if !is_safe_relative_path(norm) {
        return Err(PackValidationError::InvalidPack(format!(
            "XML attribute '{attr_name}' is not a safe relative path: {trimmed:?}"
        )));
    }
    Ok(norm.to_string())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct XmlAssetRef {
    pub tag: String,
    pub path: String,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct XmlValidationSummary {
    pub refs: Vec<XmlAssetRef>,
    pub inline_mesh_count: usize,
    pub inline_vertex_count: usize,
}

fn xml_parent_dir(xml_rel: &str) -> &str {
    match xml_rel.rsplit_once('/') {
        Some((parent, _)) => parent,
        None => "",
    }
}

fn join_rel_parts(parts: &[&str]) -> Result<String, PackValidationError> {
    let mut joined = String::new();
    for &p in parts {
        if p.is_empty() {
            continue;
        }
        if !joined.is_empty() {
            joined.push('/');
        }
        joined.push_str(p);
    }
    if !is_safe_relative_path(&joined) {
        return Err(PackValidationError::InvalidPack(format!(
            "Resolved asset path '{joined}' is invalid or exceeds depth limit"
        )));
    }
    Ok(joined)
}

#[derive(Default)]
struct CompilerDirs {
    meshdir: Option<String>,
    texturedir: Option<String>,
    assetdir: Option<String>,
}

fn pass1_xml_security<'a>(
    node: roxmltree::Node<'a, 'a>,
    depth: usize,
    dirs: &mut CompilerDirs,
) -> Result<(), PackValidationError> {
    if !node.is_element() {
        return Ok(());
    }
    if depth > MAX_XML_DEPTH {
        return Err(PackValidationError::InvalidPack(format!(
            "XML nesting depth {depth} exceeds maximum of {MAX_XML_DEPTH}"
        )));
    }

    let tag = node.tag_name().name();
    let tag_lower = tag.to_ascii_lowercase();
    if tag_lower == "urdf" || (depth == 1 && tag_lower == "robot") {
        return Err(PackValidationError::UnsupportedFormat(format!(
            "URDF content (<{tag}>) is not supported; expected MJCF <mujoco>"
        )));
    }
    if matches!(tag_lower.as_str(), "include" | "plugin" | "custom" | "extension") {
        return Err(PackValidationError::InvalidPack(format!(
            "Forbidden XML construct <{tag}>"
        )));
    }

    for attr in node.attributes() {
        let aname = attr.name();
        let aname_lower = aname.to_ascii_lowercase();
        let aval = attr.value();
        match aname_lower.as_str() {
            "meshdir" => {
                let d = validate_attr_rel_path(aname, aval, true)?;
                if tag_lower == "compiler" {
                    dirs.meshdir = if d.is_empty() { None } else { Some(d) };
                }
            }
            "texturedir" => {
                let d = validate_attr_rel_path(aname, aval, true)?;
                if tag_lower == "compiler" {
                    dirs.texturedir = if d.is_empty() { None } else { Some(d) };
                }
            }
            "assetdir" => {
                let d = validate_attr_rel_path(aname, aval, true)?;
                if tag_lower == "compiler" {
                    dirs.assetdir = if d.is_empty() { None } else { Some(d) };
                }
            }
            "file" => {
                let _ = validate_attr_rel_path(aname, aval, false)?;
            }
            "fileright" | "fileleft" | "fileup" | "filedown" | "filefront" | "fileback"
                if tag_lower == "texture" =>
            {
                let _ = validate_attr_rel_path(aname, aval, false)?;
            }
            _ => {}
        }
    }

    for child in node.children() {
        if child.is_element() {
            pass1_xml_security(child, depth + 1, dirs)?;
        }
    }
    Ok(())
}

fn pass2_collect_refs<'a>(
    node: roxmltree::Node<'a, 'a>,
    xml_parent: &str,
    dirs: &CompilerDirs,
    summary: &mut XmlValidationSummary,
) -> Result<(), PackValidationError> {
    if !node.is_element() {
        return Ok(());
    }
    let tag_lower = node.tag_name().name().to_ascii_lowercase();

    if tag_lower == "mesh" {
        if let Some(vertex_attr) = node.attribute("vertex") {
            let mut float_count = 0usize;
            for tok in vertex_attr.split_ascii_whitespace() {
                parse_finite_f32(tok, "<mesh vertex>")?;
                float_count += 1;
            }
            if float_count == 0 || !float_count.is_multiple_of(3) {
                return Err(PackValidationError::InvalidPack(format!(
                    "Inline <mesh vertex> must contain a non-empty multiple of 3 finite floats (got {float_count})"
                )));
            }
            summary.inline_mesh_count += 1;
            if summary.inline_mesh_count > MAX_MESH_FILES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Pack exceeds maximum of {MAX_MESH_FILES} mesh files/definitions"
                )));
            }
            summary.inline_vertex_count = summary.inline_vertex_count.saturating_add(float_count / 3);
            if summary.inline_vertex_count > MAX_TOTAL_MESH_VERTICES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Inline mesh vertices ({}) exceed budget of {MAX_TOTAL_MESH_VERTICES}",
                    summary.inline_vertex_count
                )));
            }
        }
    }

    let dir_for_tag = match tag_lower.as_str() {
        "mesh" => dirs.meshdir.as_deref().or(dirs.assetdir.as_deref()).unwrap_or(""),
        "texture" => dirs.texturedir.as_deref().or(dirs.assetdir.as_deref()).unwrap_or(""),
        "hfield" | "skin" | "model" => dirs.assetdir.as_deref().unwrap_or(""),
        _ => "",
    };

    if matches!(tag_lower.as_str(), "mesh" | "texture" | "hfield" | "skin" | "model") {
        for attr in node.attributes() {
            let aname_lower = attr.name().to_ascii_lowercase();
            let is_file_attr = aname_lower == "file"
                || (tag_lower == "texture"
                    && matches!(
                        aname_lower.as_str(),
                        "fileright" | "fileleft" | "fileup" | "filedown" | "filefront" | "fileback"
                    ));
            if is_file_attr {
                let f = validate_attr_rel_path(attr.name(), attr.value(), false)?;
                let resolved = join_rel_parts(&[xml_parent, dir_for_tag, &f])?;
                summary.refs.push(XmlAssetRef {
                    tag: tag_lower.clone(),
                    path: resolved,
                });
            }
        }
    }

    for child in node.children() {
        if child.is_element() {
            pass2_collect_refs(child, xml_parent, dirs, summary)?;
        }
    }
    Ok(())
}

pub fn validate_xml_and_collect_summary_for_path(
    xml_rel: &str,
    xml: &str,
    required_elem: Option<&str>,
) -> Result<XmlValidationSummary, PackValidationError> {
    if xml.len() > MAX_XML_BYTES {
        return Err(PackValidationError::InvalidPack(format!(
            "XML '{xml_rel}' exceeds {MAX_XML_BYTES} byte limit (got {} bytes)",
            xml.len()
        )));
    }
    let upper = xml.to_ascii_uppercase();
    if upper.contains("<!DOCTYPE") || upper.contains("<!ENTITY") {
        return Err(PackValidationError::InvalidPack(
            "XML DOCTYPE and ENTITY declarations are forbidden".into(),
        ));
    }
    let doc = roxmltree::Document::parse(xml)
        .map_err(|e| PackValidationError::InvalidPack(format!("XML parse failure in '{xml_rel}': {e}")))?;

    let mut dirs = CompilerDirs::default();
    pass1_xml_security(doc.root_element(), 1, &mut dirs)?;

    if let Some(req) = required_elem {
        if !doc.descendants().any(|n| n.is_element() && n.tag_name().name() == req) {
            return Err(PackValidationError::InvalidPack(format!(
                "Missing required <{req}> element in '{xml_rel}'"
            )));
        }
    }

    let mut summary = XmlValidationSummary::default();
    let parent = xml_parent_dir(xml_rel);
    pass2_collect_refs(doc.root_element(), parent, &dirs, &mut summary)?;
    Ok(summary)
}

pub fn validate_xml_and_collect_refs(
    xml: &str,
    required_elem: Option<&str>,
) -> Result<Vec<XmlAssetRef>, PackValidationError> {
    validate_xml_and_collect_summary_for_path("world.xml", xml, required_elem).map(|s| s.refs)
}

pub fn validate_xml_security(xml: &str, required_elem: Option<&str>) -> Result<(), String> {
    validate_xml_and_collect_refs(xml, required_elem)
        .map(|_| ())
        .map_err(|e| e.message().to_string())
}

// ---------------------------------------------------------------------------
// Mesh (STL / OBJ) & Pack validation
// ---------------------------------------------------------------------------

fn parse_finite_f32(tok: &str, ctx: &str) -> Result<f32, PackValidationError> {
    let v: f32 = tok
        .parse()
        .map_err(|_| PackValidationError::InvalidPack(format!("{ctx}: invalid float '{tok}'")))?;
    if !v.is_finite() {
        return Err(PackValidationError::InvalidPack(format!(
            "{ctx}: non-finite float '{tok}'"
        )));
    }
    Ok(v)
}

fn validate_ascii_stl(path: &str, text: &str) -> Result<usize, PackValidationError> {
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum AsciiStlState {
        BeforeSolid,
        InSolid,
        InFacet,
        InLoop(usize),
        AfterLoop,
        AfterEndSolid,
    }

    let mut state = AsciiStlState::BeforeSolid;
    let mut vertex_count: usize = 0;

    for raw_line in text.lines() {
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_ascii_whitespace();
        let Some(kw) = parts.next() else {
            continue;
        };
        match (state, kw) {
            (AsciiStlState::BeforeSolid | AsciiStlState::AfterEndSolid, "solid") => {
                state = AsciiStlState::InSolid;
            }
            (AsciiStlState::InSolid, "endsolid") => {
                state = AsciiStlState::AfterEndSolid;
            }
            (AsciiStlState::InSolid, "facet") => {
                if parts.next() != Some("normal") {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': expected 'facet normal'"
                    )));
                }
                let coords: Vec<&str> = parts.collect();
                if coords.len() != 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': 'facet normal' requires 3 floats"
                    )));
                }
                for c in coords {
                    parse_finite_f32(c, path)?;
                }
                state = AsciiStlState::InFacet;
            }
            (AsciiStlState::InFacet, "outer") => {
                if parts.next() != Some("loop") || parts.next().is_some() {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': expected 'outer loop'"
                    )));
                }
                state = AsciiStlState::InLoop(0);
            }
            (AsciiStlState::InLoop(n), "vertex") => {
                if n >= 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': facet loop has more than 3 vertices"
                    )));
                }
                let coords: Vec<&str> = parts.collect();
                if coords.len() != 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': 'vertex' requires 3 floats"
                    )));
                }
                for c in coords {
                    parse_finite_f32(c, path)?;
                }
                vertex_count += 1;
                if vertex_count > MAX_TOTAL_MESH_VERTICES {
                    return Err(PackValidationError::InvalidPack(format!(
                        "STL '{path}' exceeds vertex budget of {MAX_TOTAL_MESH_VERTICES}"
                    )));
                }
                state = AsciiStlState::InLoop(n + 1);
            }
            (AsciiStlState::InLoop(3), "endloop") => {
                if parts.next().is_some() {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': unexpected trailing token after 'endloop'"
                    )));
                }
                state = AsciiStlState::AfterLoop;
            }
            (AsciiStlState::AfterLoop, "endfacet") => {
                if parts.next().is_some() {
                    return Err(PackValidationError::InvalidPack(format!(
                        "ASCII STL '{path}': unexpected trailing token after 'endfacet'"
                    )));
                }
                state = AsciiStlState::InSolid;
            }
            _ => {
                return Err(PackValidationError::InvalidPack(format!(
                    "ASCII STL '{path}': invalid or out-of-order directive '{line}'"
                )));
            }
        }
    }

    if state != AsciiStlState::AfterEndSolid {
        return Err(PackValidationError::InvalidPack(format!(
            "ASCII STL '{path}' is incomplete or missing 'endsolid'"
        )));
    }
    Ok(vertex_count)
}

pub fn validate_stl_bytes(path: &str, bytes: &[u8]) -> Result<usize, PackValidationError> {
    if bytes.len() > MAX_MESH_FILE_BYTES {
        return Err(PackValidationError::InvalidPack(format!(
            "STL '{path}' exceeds single mesh limit of {MAX_MESH_FILE_BYTES} bytes"
        )));
    }
    if bytes.len() < 5 {
        return Err(PackValidationError::InvalidPack(format!(
            "STL '{path}' is truncated ({} bytes)",
            bytes.len()
        )));
    }

    // Disambiguate ASCII vs Binary STL:
    // Treat as ASCII only if valid UTF-8 without NUL/unexpected control characters AND starts with `solid`.
    if let Ok(text) = std::str::from_utf8(bytes) {
        let no_ctrl = !text
            .bytes()
            .any(|b| b < 0x20 && !matches!(b, b'\t' | b'\n' | b'\r'));
        let trimmed = text.trim_start();
        if no_ctrl
            && (trimmed == "solid"
                || trimmed.starts_with("solid ")
                || trimmed.starts_with("solid\t")
                || trimmed.starts_with("solid\n")
                || trimmed.starts_with("solid\r"))
        {
            return validate_ascii_stl(path, text);
        }
    }

    if bytes.len() < 84 {
        return Err(PackValidationError::InvalidPack(format!(
            "Binary STL '{path}' is truncated ({} bytes < 84)",
            bytes.len()
        )));
    }
    let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected_len = tri_count
        .checked_mul(50)
        .and_then(|n| n.checked_add(84))
        .ok_or_else(|| {
            PackValidationError::InvalidPack(format!("Binary STL '{path}' triangle count overflow"))
        })?;
    if bytes.len() != expected_len {
        return Err(PackValidationError::InvalidPack(format!(
            "Binary STL '{path}' length {} does not match header tri_count {} (expected {expected_len} bytes)",
            bytes.len(),
            tri_count
        )));
    }
    let vertex_count = tri_count.checked_mul(3).ok_or_else(|| {
        PackValidationError::InvalidPack(format!("Binary STL '{path}' vertex count overflow"))
    })?;
    if vertex_count > MAX_TOTAL_MESH_VERTICES {
        return Err(PackValidationError::InvalidPack(format!(
            "Binary STL '{path}' vertex count {vertex_count} exceeds budget of {MAX_TOTAL_MESH_VERTICES}"
        )));
    }
    for i in 0..tri_count {
        let base = 84 + i * 50;
        for j in 0..12 {
            let off = base + j * 4;
            let f = f32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
            if !f.is_finite() {
                return Err(PackValidationError::InvalidPack(format!(
                    "Binary STL '{path}' contains non-finite float in triangle {i}"
                )));
            }
        }
    }
    Ok(vertex_count)
}

fn check_obj_index_in_bounds(
    raw: &str,
    table_len: usize,
    kind: &str,
    path: &str,
    line_no: usize,
) -> Result<(), PackValidationError> {
    let parsed: isize = raw.parse().map_err(|_| {
        PackValidationError::InvalidPack(format!(
            "OBJ '{path}' line {line_no}: non-integer {kind} index '{raw}'"
        ))
    })?;
    if parsed == 0 || parsed.unsigned_abs() > table_len {
        return Err(PackValidationError::InvalidPack(format!(
            "OBJ '{path}' line {line_no}: out-of-bounds {kind} index {parsed} (table size {table_len})"
        )));
    }
    Ok(())
}

fn validate_obj_face_vertex(
    tok: &str,
    v_count: usize,
    vt_count: usize,
    vn_count: usize,
    path: &str,
    line_no: usize,
) -> Result<(), PackValidationError> {
    let segs: Vec<&str> = tok.split('/').collect();
    if segs.is_empty() || segs.len() > 3 || segs[0].is_empty() {
        return Err(PackValidationError::InvalidPack(format!(
            "OBJ '{path}' line {line_no}: invalid face token '{tok}'"
        )));
    }
    check_obj_index_in_bounds(segs[0], v_count, "v", path, line_no)?;
    if segs.len() >= 2 {
        if segs[1].is_empty() {
            if segs.len() == 2 {
                return Err(PackValidationError::InvalidPack(format!(
                    "OBJ '{path}' line {line_no}: empty vt index in face token '{tok}'"
                )));
            }
        } else {
            check_obj_index_in_bounds(segs[1], vt_count, "vt", path, line_no)?;
        }
    }
    if segs.len() == 3 {
        if segs[2].is_empty() {
            return Err(PackValidationError::InvalidPack(format!(
                "OBJ '{path}' line {line_no}: empty vn index in face token '{tok}'"
            )));
        }
        check_obj_index_in_bounds(segs[2], vn_count, "vn", path, line_no)?;
    }
    Ok(())
}

pub fn validate_obj_bytes(path: &str, bytes: &[u8]) -> Result<usize, PackValidationError> {
    if bytes.len() > MAX_MESH_FILE_BYTES {
        return Err(PackValidationError::InvalidPack(format!(
            "OBJ '{path}' exceeds single mesh limit of {MAX_MESH_FILE_BYTES} bytes"
        )));
    }
    let text = std::str::from_utf8(bytes).map_err(|_| {
        PackValidationError::InvalidPack(format!("OBJ '{path}' is not valid UTF-8"))
    })?;
    let mut vertex_count: usize = 0;
    let mut vt_count: usize = 0;
    let mut vn_count: usize = 0;

    for (idx, raw_line) in text.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_ascii_whitespace();
        let directive = parts.next().unwrap();
        match directive {
            "mtllib" => {
                return Err(PackValidationError::InvalidPack(format!(
                    "OBJ '{path}' line {line_no}: 'mtllib' directive is forbidden"
                )));
            }
            "v" => {
                let coords: Vec<&str> = parts.collect();
                if coords.len() != 3 && coords.len() != 4 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' line {line_no}: 'v' requires 3 or 4 floats"
                    )));
                }
                for c in coords {
                    parse_finite_f32(c, path)?;
                }
                vertex_count += 1;
                if vertex_count > MAX_TOTAL_MESH_VERTICES {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' exceeds vertex budget of {MAX_TOTAL_MESH_VERTICES}"
                    )));
                }
            }
            "vn" => {
                let coords: Vec<&str> = parts.collect();
                if coords.len() != 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' line {line_no}: 'vn' requires 3 floats"
                    )));
                }
                for c in coords {
                    parse_finite_f32(c, path)?;
                }
                vn_count += 1;
            }
            "vt" => {
                let coords: Vec<&str> = parts.collect();
                if !(1..=3).contains(&coords.len()) {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' line {line_no}: 'vt' requires 1..=3 floats"
                    )));
                }
                for c in coords {
                    parse_finite_f32(c, path)?;
                }
                vt_count += 1;
            }
            "f" => {
                let verts: Vec<&str> = parts.collect();
                if verts.len() < 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' line {line_no}: 'f' requires at least 3 vertices"
                    )));
                }
                for vtok in verts {
                    validate_obj_face_vertex(vtok, vertex_count, vt_count, vn_count, path, line_no)?;
                }
            }
            "g" | "o" | "s" | "usemtl" => {}
            other => {
                return Err(PackValidationError::InvalidPack(format!(
                    "OBJ '{path}' line {line_no}: unsupported directive '{other}'"
                )));
            }
        }
    }

    Ok(vertex_count)
}

pub fn compute_pack_digest<'a, I>(files: I) -> String
where
    I: IntoIterator<Item = (&'a str, &'a [u8])>,
{
    let sorted: BTreeMap<&'a str, &'a [u8]> = files.into_iter().collect();
    let mut hasher = crate::gravity::Sha256::new();
    for (rel, bytes) in sorted {
        hasher.update(rel.as_bytes());
        hasher.update(&[0]);
        hasher.update(&(bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);
    }
    hasher.finalize_hex()
}

pub fn validate_pack_map(
    primary_xml_rel: &str,
    required_elem: Option<&str>,
    files: &BTreeMap<String, Vec<u8>>,
) -> Result<String, PackValidationError> {
    if !is_safe_relative_path(primary_xml_rel) {
        return Err(PackValidationError::InvalidPack(format!(
            "Invalid primary XML path '{primary_xml_rel}'"
        )));
    }
    if is_urdf_path(primary_xml_rel) {
        return Err(PackValidationError::UnsupportedFormat(format!(
            "URDF files are not supported: '{primary_xml_rel}'"
        )));
    }

    let mut total_bytes: usize = 0;
    for (rel, bytes) in files {
        if !is_safe_relative_path(rel) {
            return Err(PackValidationError::InvalidPack(format!(
                "Invalid or escaping relative path '{rel}'"
            )));
        }
        if is_urdf_path(rel) {
            return Err(PackValidationError::UnsupportedFormat(format!(
                "URDF files are not supported in pack: '{rel}'"
            )));
        }
        total_bytes = total_bytes.checked_add(bytes.len()).ok_or_else(|| {
            PackValidationError::InvalidPack("Pack size overflow".into())
        })?;
        if total_bytes > MAX_PACK_TOTAL_BYTES {
            return Err(PackValidationError::InvalidPack(format!(
                "Total decoded pack size exceeds {MAX_PACK_TOTAL_BYTES} byte limit"
            )));
        }
    }

    if !files.contains_key(primary_xml_rel) {
        return Err(PackValidationError::InvalidPack(format!(
            "Missing primary XML file '{primary_xml_rel}'"
        )));
    }

    let mut mesh_refs = std::collections::BTreeSet::new();
    let mut mesh_count: usize = 0;
    let mut total_vertices: usize = 0;

    for (rel, bytes) in files {
        if rel == primary_xml_rel || rel.to_ascii_lowercase().ends_with(".xml") {
            let xml_str = std::str::from_utf8(bytes).map_err(|_| {
                PackValidationError::InvalidPack(format!("XML '{rel}' is not valid UTF-8"))
            })?;
            let req = if rel == primary_xml_rel { required_elem } else { None };
            let summary = validate_xml_and_collect_summary_for_path(rel, xml_str, req)?;
            mesh_count = mesh_count.saturating_add(summary.inline_mesh_count);
            if mesh_count > MAX_MESH_FILES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Pack exceeds maximum of {MAX_MESH_FILES} mesh files/definitions"
                )));
            }
            total_vertices = total_vertices.saturating_add(summary.inline_vertex_count);
            if total_vertices > MAX_TOTAL_MESH_VERTICES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Total mesh vertices ({total_vertices}) exceed budget of {MAX_TOTAL_MESH_VERTICES}"
                )));
            }
            for asset_ref in &summary.refs {
                if is_urdf_path(&asset_ref.path) {
                    return Err(PackValidationError::UnsupportedFormat(format!(
                        "URDF reference '{}' is not supported",
                        asset_ref.path
                    )));
                }
                if !files.contains_key(&asset_ref.path) {
                    return Err(PackValidationError::InvalidPack(format!(
                        "Dangling <{} file=\"{}\"> reference in '{rel}'",
                        asset_ref.tag, asset_ref.path
                    )));
                }
                if asset_ref.tag == "mesh" {
                    mesh_refs.insert(asset_ref.path.clone());
                }
            }
        }
    }

    for (rel, bytes) in files {
        let lower = rel.to_ascii_lowercase();
        let is_stl = lower.ends_with(".stl");
        let is_obj = lower.ends_with(".obj");
        let is_referenced_mesh = mesh_refs.contains(rel);
        if is_stl || is_obj || is_referenced_mesh {
            mesh_count += 1;
            if mesh_count > MAX_MESH_FILES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Pack exceeds maximum of {MAX_MESH_FILES} mesh files/definitions"
                )));
            }
            let verts = if is_stl {
                validate_stl_bytes(rel, bytes)?
            } else if is_obj {
                validate_obj_bytes(rel, bytes)?
            } else {
                return Err(PackValidationError::UnsupportedFormat(format!(
                    "Unsupported mesh format for '{rel}'; expected .stl or .obj"
                )));
            };
            total_vertices = total_vertices.saturating_add(verts);
            if total_vertices > MAX_TOTAL_MESH_VERTICES {
                return Err(PackValidationError::InvalidPack(format!(
                    "Total mesh vertices ({total_vertices}) exceed budget of {MAX_TOTAL_MESH_VERTICES}"
                )));
            }
        }
    }

    Ok(compute_pack_digest(
        files.iter().map(|(k, v)| (k.as_str(), v.as_slice())),
    ))
}

// ---------------------------------------------------------------------------
// Manifest models
// ---------------------------------------------------------------------------

fn default_version() -> String { "1.0.0".into() }
fn default_world_xml() -> String { "world.xml".into() }
fn default_tool_xml() -> String { "tool.xml".into() }
fn default_tool() -> String { "default".into() }
fn default_kind_gripper() -> String { "gripper".into() }
fn default_attach_body() -> String { "end_link".into() }
fn default_replace_bodies() -> Vec<String> { vec!["finger_left".into(), "finger_right".into()] }
fn default_replace_geoms() -> Vec<String> { vec!["palm".into(), "motor_housing".into()] }
fn default_replace_equalities() -> Vec<String> { vec!["jaw_coupling".into()] }
fn default_replace_actuators() -> Vec<String> { vec!["m7".into()] }
fn default_scale_one() -> f64 { 1.0 }
fn default_tcp_site() -> String { "tcp".into() }
fn default_tcp_offset() -> [f64; 3] { [0.0, 0.0, 0.0] }
fn default_suction_range() -> f64 { 0.030 }
fn default_robot_name() -> String { "b601dm".into() }
fn default_unit_mode() -> String { "rad".into() }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneManifest {
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_version")]
    pub version: String,
    #[serde(default)]
    pub description: String,
    #[serde(default = "default_world_xml")]
    pub world_xml: String,
    #[serde(default)]
    pub robots: Vec<String>,
    #[serde(default = "default_tool")]
    pub default_tool: String,
    #[serde(default)]
    pub recommended_tools: Vec<String>,
    #[serde(default)]
    pub objects: Vec<serde_json::Value>,
    #[serde(default)]
    pub zones: serde_json::Value,
    #[serde(default)]
    pub io: Vec<serde_json::Value>,
    #[serde(default)]
    pub task_stages: Vec<serde_json::Value>,
    #[serde(default)]
    pub poses: serde_json::Value,
    #[serde(default)]
    pub keepout_boxes: Vec<serde_json::Value>,
    #[serde(default)]
    pub training: serde_json::Value,
    #[serde(default)]
    pub assets: Vec<serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolManifest {
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_kind_gripper")]
    pub kind: String,
    #[serde(default = "default_version")]
    pub version: String,
    #[serde(default)]
    pub description: String,
    #[serde(default = "default_tool_xml")]
    pub tool_xml: String,
    #[serde(default)]
    pub robots: Vec<String>,
    #[serde(default = "default_attach_body")]
    pub attach_body: String,
    #[serde(default = "default_replace_bodies")]
    pub replace_bodies: Vec<String>,
    #[serde(default = "default_replace_geoms")]
    pub replace_geoms: Vec<String>,
    #[serde(default = "default_replace_equalities")]
    pub replace_equalities: Vec<String>,
    #[serde(default = "default_replace_actuators")]
    pub replace_actuators: Vec<String>,
    #[serde(default)]
    pub driven_joint: Option<String>,
    #[serde(default)]
    pub coupled_joint: Option<String>,
    #[serde(default)]
    pub actuator: Option<String>,
    #[serde(default = "default_scale_one")]
    pub motor_to_joint_scale: f64,
    #[serde(default)]
    pub finger_geoms: Vec<String>,
    #[serde(default = "default_tcp_site")]
    pub tcp_site: String,
    #[serde(default = "default_tcp_offset")]
    pub tcp_offset: [f64; 3],
    #[serde(default = "default_suction_range")]
    pub suction_range_m: f64,
    #[serde(default)]
    pub io: Vec<serde_json::Value>,
    #[serde(default)]
    pub assets: Vec<serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyManifest {
    #[serde(default)]
    pub id: String,
    #[serde(default)]
    pub name: String,
    pub format: String, // must be "rebot-onnx-policy/v1"
    #[serde(default = "default_robot_name")]
    pub robot: String,
    pub action_dim: usize,
    #[serde(default = "default_unit_mode")]
    pub unit_mode: String,
    pub inputs: serde_json::Value,
    pub outputs: serde_json::Value,
    pub model_file: String,
    #[serde(default)]
    pub fps: Option<f64>,
    #[serde(default)]
    pub normalization: serde_json::Value,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub metadata: serde_json::Value,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vision: Option<serde_json::Value>,
}
// ---------------------------------------------------------------------------
// Builtin presets (matching sim/mujoco/src/rebot_sim/pack.py verbatim)
// ---------------------------------------------------------------------------

pub const PRESET_SUCTION_XML: &str = r#"<mujoco model="builtin_suction_tool">
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
    <position name="m7_suction" joint="suction_valve" kp="20" dampratio="1" forcerange="-5 5"/>
  </actuator>
</mujoco>
"#;

pub const PRESET_SCREWDRIVER_XML: &str = r#"<mujoco model="builtin_screwdriver_tool">
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
"#;

pub const PRESET_PCB_SUCTION_XML: &str = r#"<mujoco model="pcb_suction_world">
  <worldbody>
    <!-- Infeed station base tray -->
    <geom name="infeed_tray" type="box" pos="0.23 -0.14 0.004" size="0.055 0.055 0.004"
          rgba="0.28 0.30 0.34 1"/>

    <!-- Free-joint PCB workpiece resting on infeed tray -->
    <body name="pcb" pos="0.23 -0.14 0.011">
      <freejoint name="pcb_joint"/>
      <inertial pos="0 0 0" mass="0.06" diaginertia="2e-5 2e-5 4e-5"/>
      <geom name="pcb" type="box" size="0.030 0.022 0.003" rgba="0.08 0.45 0.18 1"
            friction="1.4 0.01 0.001" condim="4"/>
      <!-- Decorative gold edge connector strip -->
      <geom name="pcb_connector" type="box" pos="0.027 0 0.001" size="0.003 0.018 0.0022"
            rgba="0.85 0.72 0.20 1" contype="0" conaffinity="0"/>
    </body>

    <!-- Physical divider wall separating infeed area from test fixture -->
    <geom name="divider_wall" type="box" pos="0.25 -0.065 0.025" size="0.075 0.005 0.025"
          rgba="0.40 0.42 0.46 1"/>

    <!-- Test fixture station with alignment pad and status indicator LED -->
    <body name="test_fixture" pos="0.27 0.01 0">
      <geom name="fixture_base" type="box" pos="0 0 0.008" size="0.055 0.045 0.008"
            rgba="0.22 0.24 0.27 1"/>
      <geom name="fixture_pad" type="box" pos="0 0 0.0165" size="0.035 0.026 0.0005"
            rgba="0.25 0.50 0.80 0.8" contype="0" conaffinity="0"/>
      <geom name="test_pass_led" type="sphere" pos="0.042 -0.030 0.022" size="0.007"
            rgba="0.85 0.20 0.20 1" contype="0" conaffinity="0"/>
    </body>

    <!-- Outfeed bin station with low border walls and catch mat -->
    <body name="outfeed_bin" pos="0.23 0.15 0">
      <geom name="outfeed_bottom" type="box" pos="0 0 0.003" size="0.055 0.055 0.003"
            rgba="0.20 0.22 0.25 1"/>
      <geom name="outfeed_wall_n" type="box" pos="0 0.052 0.016" size="0.055 0.003 0.013"
            rgba="0.32 0.34 0.38 1"/>
      <geom name="outfeed_wall_s" type="box" pos="0 -0.052 0.016" size="0.055 0.003 0.013"
            rgba="0.32 0.34 0.38 1"/>
      <geom name="outfeed_wall_e" type="box" pos="0.052 0 0.016" size="0.003 0.049 0.013"
            rgba="0.32 0.34 0.38 1"/>
      <geom name="outfeed_wall_w" type="box" pos="-0.052 0 0.016" size="0.003 0.049 0.013"
            rgba="0.32 0.34 0.38 1"/>
      <geom name="outfeed_mat" type="box" pos="0 0 0.0065" size="0.048 0.048 0.0005"
            rgba="0.16 0.48 0.26 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;
// ---------------------------------------------------------------------------
// Base Robot XML resolution
// ---------------------------------------------------------------------------

pub const FALLBACK_BASE_XML: &str = include_str!("../sim/mujoco/models/rebot_b601dm_scene.xml");

// ---------------------------------------------------------------------------
// Lightweight MJCF Composer
// ---------------------------------------------------------------------------

struct XmlEdit {
    start: usize,
    end: usize,
    replacement: String,
}

fn extract_children_xml(node: roxmltree::Node, raw_xml: &str, skip_site: Option<&str>) -> (String, Option<String>) {
    let mut out = String::new();
    let mut site_pos = None;
    for child in node.children() {
        if child.is_element() {
            if let Some(sname) = skip_site {
                if child.tag_name().name() == "site" && child.attribute("name") == Some(sname) {
                    if let Some(pos) = child.attribute("pos") {
                        site_pos = Some(pos.to_string());
                    }
                    continue;
                }
            }
            out.push_str(&raw_xml[child.range()]);
            out.push('\n');
        }
    }
    (out, site_pos)
}

fn find_free_body_names(doc: &roxmltree::Document) -> Vec<String> {
    let mut names = Vec::new();
    for node in doc.descendants() {
        if node.is_element() && node.tag_name().name() == "body" {
            let has_free = node.children().any(|c| {
                c.is_element()
                    && (c.tag_name().name() == "freejoint"
                        || (c.tag_name().name() == "joint" && c.attribute("type") == Some("free")))
            });
            if has_free {
                if let Some(name) = node.attribute("name") {
                    if !names.contains(&name.to_string()) {
                        names.push(name.to_string());
                    }
                }
            }
        }
    }
    names
}

pub fn compose_mjcf(
    base_xml: &str,
    scene_world_xml: Option<&str>,
    tool_manifest: Option<&ToolManifest>,
    tool_xml: Option<&str>,
) -> Result<String, String> {
    let base_doc = roxmltree::Document::parse(base_xml)
        .map_err(|e| format!("Base MJCF is invalid XML: {e}"))?;

    let scene_doc = if let Some(s) = scene_world_xml {
        if !s.trim().is_empty() {
            Some(roxmltree::Document::parse(s).map_err(|e| format!("Scene world XML is invalid: {e}"))?)
        } else {
            None
        }
    } else {
        None
    };

    let tool_doc = if let Some(t) = tool_xml {
        if !t.trim().is_empty() {
            Some(roxmltree::Document::parse(t).map_err(|e| format!("Tool XML is invalid: {e}"))?)
        } else {
            None
        }
    } else {
        None
    };

    let mut edits: Vec<XmlEdit> = Vec::new();

    // 1. Remove cube if scene is active
    if scene_doc.is_some() {
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "body" && node.attribute("name") == Some("cube") {
                edits.push(XmlEdit {
                    start: node.range().start,
                    end: node.range().end,
                    replacement: String::new(),
                });
            }
        }
    }

    // 2. Tool elements to remove & inject
    let is_tool_active = tool_manifest.is_some()
        && tool_manifest.map(|m| m.id.as_str()) != Some("default")
        && tool_doc.is_some();

    let mut tool_worldbody_xml = String::new();
    let mut tool_asset_xml = String::new();
    let mut tool_equality_xml = String::new();
    let mut tool_actuator_xml = String::new();
    let mut tool_tendon_xml = String::new();
    let mut tool_tcp_pos = None;

    if is_tool_active {
        let tm = tool_manifest.unwrap();
        let td = tool_doc.as_ref().unwrap();

        // Extract tool sections
        for node in td.descendants() {
            if node.is_element() {
                match node.tag_name().name() {
                    "worldbody" => {
                        let (wb, pos) = extract_children_xml(node, tool_xml.unwrap(), Some(&tm.tcp_site));
                        tool_worldbody_xml.push_str(&wb);
                        if pos.is_some() {
                            tool_tcp_pos = pos;
                        }
                    }
                    "asset" => {
                        let (a, _) = extract_children_xml(node, tool_xml.unwrap(), None);
                        tool_asset_xml.push_str(&a);
                    }
                    "equality" => {
                        let (eq, _) = extract_children_xml(node, tool_xml.unwrap(), None);
                        tool_equality_xml.push_str(&eq);
                    }
                    "actuator" => {
                        let (act, _) = extract_children_xml(node, tool_xml.unwrap(), None);
                        tool_actuator_xml.push_str(&act);
                    }
                    "tendon" => {
                        let (ten, _) = extract_children_xml(node, tool_xml.unwrap(), None);
                        tool_tendon_xml.push_str(&ten);
                    }
                    _ => {}
                }
            }
        }

        // Delete replaced bodies, geoms, equalities, actuators
        for node in base_doc.descendants() {
            if !node.is_element() {
                continue;
            }
            let tag = node.tag_name().name();
            let name = node.attribute("name").unwrap_or("");
            let should_delete = (tag == "body" && tm.replace_bodies.iter().any(|b| b == name))
                || (tag == "geom" && tm.replace_geoms.iter().any(|g| g == name))
                || (node.parent().map(|p| p.tag_name().name()) == Some("equality")
                    && tm.replace_equalities.iter().any(|eq| eq == name))
                || (node.parent().map(|p| p.tag_name().name()) == Some("actuator")
                    && tm.replace_actuators.iter().any(|act| act == name));

            if should_delete {
                edits.push(XmlEdit {
                    start: node.range().start,
                    end: node.range().end,
                    replacement: String::new(),
                });
            }
        }

        // Insert tool worldbody children inside <body name="{attach_body}">
        let attach_name = &tm.attach_body;
        let mut attached = false;
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "body" && node.attribute("name") == Some(attach_name) {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</body") {
                    let insert_pos = range.start + close_idx;
                    edits.push(XmlEdit {
                        start: insert_pos,
                        end: insert_pos,
                        replacement: format!("\n{}\n", tool_worldbody_xml),
                    });
                    attached = true;
                    break;
                }
            }
        }
        if !attached && !tool_worldbody_xml.is_empty() {
            return Err(format!("Could not find attach body <body name=\"{attach_name}\"> in base XML"));
        }

        // Update TCP site pos if specified
        let tcp_override = if tm.tcp_offset != [0.0, 0.0, 0.0] {
            Some(format!("{} {} {}", tm.tcp_offset[0], tm.tcp_offset[1], tm.tcp_offset[2]))
        } else {
            tool_tcp_pos
        };

        if let Some(pos_val) = tcp_override {
            for node in base_doc.descendants() {
                if node.is_element() && node.tag_name().name() == "site" && node.attribute("name") == Some("tcp") {
                    let range = node.range();
                    let site_str = &base_xml[range.clone()];
                    if let Some(pos_idx) = site_str.find("pos=\"") {
                        let after = &site_str[pos_idx + 5..];
                        if let Some(end_quote) = after.find('"') {
                            let start = range.start + pos_idx + 5;
                            let end = start + end_quote;
                            edits.push(XmlEdit {
                                start,
                                end,
                                replacement: pos_val,
                            });
                            break;
                        }
                    }
                }
            }
        }
    }

    // 3. Extract scene sections
    let mut scene_worldbody_xml = String::new();
    let mut scene_asset_xml = String::new();
    let mut scene_equality_xml = String::new();
    let mut scene_actuator_xml = String::new();
    let mut scene_tendon_xml = String::new();

    if let Some(sd) = scene_doc.as_ref() {
        for node in sd.descendants() {
            if node.is_element() {
                match node.tag_name().name() {
                    "worldbody" => {
                        let (wb, _) = extract_children_xml(node, scene_world_xml.unwrap(), None);
                        scene_worldbody_xml.push_str(&wb);
                    }
                    "asset" => {
                        let (a, _) = extract_children_xml(node, scene_world_xml.unwrap(), None);
                        scene_asset_xml.push_str(&a);
                    }
                    "equality" => {
                        let (eq, _) = extract_children_xml(node, scene_world_xml.unwrap(), None);
                        scene_equality_xml.push_str(&eq);
                    }
                    "actuator" => {
                        let (act, _) = extract_children_xml(node, scene_world_xml.unwrap(), None);
                        scene_actuator_xml.push_str(&act);
                    }
                    "tendon" => {
                        let (ten, _) = extract_children_xml(node, scene_world_xml.unwrap(), None);
                        scene_tendon_xml.push_str(&ten);
                    }
                    _ => {}
                }
            }
        }
    }

    // 4. Inject scene worldbody before </worldbody>
    if !scene_worldbody_xml.is_empty() {
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "worldbody" {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</worldbody") {
                    let insert_pos = range.start + close_idx;
                    edits.push(XmlEdit {
                        start: insert_pos,
                        end: insert_pos,
                        replacement: format!("\n{}\n", scene_worldbody_xml),
                    });
                    break;
                }
            }
        }
    }

    // 5. Suction tool weld injection
    let mut suction_welds_xml = String::new();
    if is_tool_active && tool_manifest.unwrap().kind == "suction" {
        let attach_name = &tool_manifest.unwrap().attach_body;
        let mut free_bodies = Vec::new();
        if let Some(sd) = scene_doc.as_ref() {
            free_bodies.extend(find_free_body_names(sd));
        } else {
            free_bodies.extend(find_free_body_names(&base_doc));
        }
        for (i, bname) in free_bodies.iter().enumerate() {
            suction_welds_xml.push_str(&format!(
                "    <weld name=\"suction_weld_{i}_{bname}\" body1=\"{attach_name}\" body2=\"{bname}\" active=\"false\" solref=\"0.02 1\" solimp=\"0.9 0.95 0.001 0.5 2\"/>\n"
            ));
        }
    }

    // 6. Assets injection
    let combined_assets = format!("{}{}", scene_asset_xml, tool_asset_xml);
    if !combined_assets.trim().is_empty() {
        let mut injected = false;
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "asset" {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</asset") {
                    edits.push(XmlEdit {
                        start: range.start + close_idx,
                        end: range.start + close_idx,
                        replacement: format!("\n{}\n", combined_assets),
                    });
                    injected = true;
                    break;
                }
            }
        }
        if !injected {
            for node in base_doc.descendants() {
                if node.is_element() && node.tag_name().name() == "worldbody" {
                    edits.push(XmlEdit {
                        start: node.range().start,
                        end: node.range().start,
                        replacement: format!("<asset>\n{}\n</asset>\n", combined_assets),
                    });
                    break;
                }
            }
        }
    }

    // 7. Equalities injection
    let combined_equality = format!("{}{}{}", scene_equality_xml, tool_equality_xml, suction_welds_xml);
    if !combined_equality.trim().is_empty() {
        let mut injected = false;
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "equality" {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</equality") {
                    edits.push(XmlEdit {
                        start: range.start + close_idx,
                        end: range.start + close_idx,
                        replacement: format!("\n{}\n", combined_equality),
                    });
                    injected = true;
                    break;
                }
            }
        }
        if !injected {
            for node in base_doc.descendants() {
                if node.is_element() && node.tag_name().name() == "actuator" {
                    edits.push(XmlEdit {
                        start: node.range().start,
                        end: node.range().start,
                        replacement: format!("<equality>\n{}\n</equality>\n", combined_equality),
                    });
                    injected = true;
                    break;
                }
            }
        }
        if !injected {
            if let Some(idx) = base_xml.rfind("</mujoco") {
                edits.push(XmlEdit {
                    start: idx,
                    end: idx,
                    replacement: format!("<equality>\n{}\n</equality>\n", combined_equality),
                });
            }
        }
    }

    // 8. Actuators injection
    let combined_actuator = format!("{}{}", scene_actuator_xml, tool_actuator_xml);
    if !combined_actuator.trim().is_empty() {
        let mut injected = false;
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "actuator" {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</actuator") {
                    edits.push(XmlEdit {
                        start: range.start + close_idx,
                        end: range.start + close_idx,
                        replacement: format!("\n{}\n", combined_actuator),
                    });
                    injected = true;
                    break;
                }
            }
        }
        if !injected {
            if let Some(idx) = base_xml.rfind("</mujoco") {
                edits.push(XmlEdit {
                    start: idx,
                    end: idx,
                    replacement: format!("<actuator>\n{}\n</actuator>\n", combined_actuator),
                });
            }
        }
    }

    // 9. Tendons injection
    let combined_tendon = format!("{}{}", scene_tendon_xml, tool_tendon_xml);
    if !combined_tendon.trim().is_empty() {
        let mut injected = false;
        for node in base_doc.descendants() {
            if node.is_element() && node.tag_name().name() == "tendon" {
                let range = node.range();
                let slice = &base_xml[range.clone()];
                if let Some(close_idx) = slice.rfind("</tendon") {
                    edits.push(XmlEdit {
                        start: range.start + close_idx,
                        end: range.start + close_idx,
                        replacement: format!("\n{}\n", combined_tendon),
                    });
                    injected = true;
                    break;
                }
            }
        }
        if !injected {
            if let Some(idx) = base_xml.rfind("</mujoco") {
                edits.push(XmlEdit {
                    start: idx,
                    end: idx,
                    replacement: format!("<tendon>\n{}\n</tendon>\n", combined_tendon),
                });
            }
        }
    }

    // Apply all edits in ascending offset order
    edits.sort_by_key(|e| e.start);

    let mut composed = String::with_capacity(base_xml.len() + 4096);
    let mut cursor = 0;
    for edit in edits {
        if edit.start < cursor {
            // Overlapping edits - skip or advance
            continue;
        }
        composed.push_str(&base_xml[cursor..edit.start]);
        composed.push_str(&edit.replacement);
        cursor = edit.end;
    }
    composed.push_str(&base_xml[cursor..]);

    // Validation: verify valid XML document
    roxmltree::Document::parse(&composed)
        .map_err(|e| format!("Composed MJCF failed XML validation: {e}"))?;

    Ok(composed)
}

pub fn builtin_tool_preset(id: &str) -> Option<(ToolManifest, &'static str)> {
    if !validate_pack_id(id) {
        return None;
    }
    if id == "default" {
        let manifest = ToolManifest {
            id: "default".into(),
            name: "Stock Parallel Jaw Gripper".into(),
            kind: "gripper".into(),
            version: "1.0.0".into(),
            description: "Factory parallel gripper shipped with B601-DM or SO-ARM101.".into(),
            tool_xml: "tool.xml".into(),
            robots: vec!["b601dm".into(), "so-arm101".into()],
            attach_body: "end_link".into(),
            replace_bodies: default_replace_bodies(),
            replace_geoms: default_replace_geoms(),
            replace_equalities: default_replace_equalities(),
            replace_actuators: default_replace_actuators(),
            driven_joint: Some("finger_left".into()),
            coupled_joint: Some("finger_right".into()),
            actuator: Some("m7".into()),
            motor_to_joint_scale: 1.0,
            finger_geoms: vec!["left_front".into(), "right_front".into()],
            tcp_site: "tcp".into(),
            tcp_offset: [0.0, 0.0, 0.0],
            suction_range_m: 0.030,
            io: vec![],
            assets: vec![],
        };
        return Some((manifest, ""));
    }

    if id == "suction" {
        let manifest = ToolManifest {
            id: "suction".into(),
            name: "Vacuum Suction End-Effector".into(),
            kind: "suction".into(),
            version: "1.0.0".into(),
            description: "Flat vacuum cup with runtime mjEQ_WELD suction controlled via /sim/io {vacuum: true|false}.".into(),
            tool_xml: "tool.xml".into(),
            robots: vec!["b601dm".into()],
            attach_body: "end_link".into(),
            replace_bodies: default_replace_bodies(),
            replace_geoms: default_replace_geoms(),
            replace_equalities: default_replace_equalities(),
            replace_actuators: default_replace_actuators(),
            driven_joint: Some("suction_valve".into()),
            coupled_joint: None,
            actuator: Some("m7_suction".into()),
            motor_to_joint_scale: -0.007456140350877194,
            finger_geoms: vec![],
            tcp_site: "tcp".into(),
            tcp_offset: [0.0, 0.0, 0.0],
            suction_range_m: 0.030,
            io: vec![
                serde_json::json!({"name": "vacuum", "kind": "bool", "direction": "output", "default": false}),
                serde_json::json!({"name": "vacuum_engaged", "kind": "bool", "direction": "input", "default": false}),
            ],
            assets: vec![],
        };
        return Some((manifest, PRESET_SUCTION_XML));
    }

    if id == "screwdriver" {
        let manifest = ToolManifest {
            id: "screwdriver".into(),
            name: "Spindle Electric Screwdriver".into(),
            kind: "screwdriver".into(),
            version: "1.0.0".into(),
            description: "Rotating electric screwdriver spindle controlled via /sim/io {spindle_rpm: ...}.".into(),
            tool_xml: "tool.xml".into(),
            robots: vec!["b601dm".into()],
            attach_body: "end_link".into(),
            replace_bodies: default_replace_bodies(),
            replace_geoms: default_replace_geoms(),
            replace_equalities: default_replace_equalities(),
            replace_actuators: default_replace_actuators(),
            driven_joint: Some("spindle_joint".into()),
            coupled_joint: None,
            actuator: Some("m7_spindle".into()),
            motor_to_joint_scale: 1.0,
            finger_geoms: vec![],
            tcp_site: "tcp".into(),
            tcp_offset: [0.0, 0.0, 0.0],
            suction_range_m: 0.030,
            io: vec![
                serde_json::json!({"name": "spindle_rpm", "kind": "float", "direction": "output", "default": 0.0}),
            ],
            assets: vec![],
        };
        return Some((manifest, PRESET_SCREWDRIVER_XML));
    }

    None
}

pub fn default_tool_manifest() -> ToolManifest {
    builtin_tool_preset("default").expect("built-in default tool").0
}
