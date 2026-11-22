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

fn parse_exact_floats<'a>(
    parts: impl Iterator<Item = &'a str>,
    min_count: usize,
    max_count: usize,
    path: &str,
    err_msg: impl FnOnce() -> String,
) -> Result<(), PackValidationError> {
    let mut count = 0usize;
    for tok in parts {
        count += 1;
        if count > max_count {
            return Err(PackValidationError::InvalidPack(err_msg()));
        }
        parse_finite_f32(tok, path)?;
    }
    if count < min_count {
        return Err(PackValidationError::InvalidPack(err_msg()));
    }
    Ok(())
}

fn check_mesh_budgets(mesh_count: usize, total_vertices: usize) -> Result<(), PackValidationError> {
    if mesh_count > MAX_MESH_FILES {
        return Err(PackValidationError::InvalidPack(format!(
            "Pack exceeds maximum of {MAX_MESH_FILES} mesh files/definitions"
        )));
    }
    if total_vertices > MAX_TOTAL_MESH_VERTICES {
        return Err(PackValidationError::InvalidPack(format!(
            "Total mesh vertices ({total_vertices}) exceed budget of {MAX_TOTAL_MESH_VERTICES}"
        )));
    }
    Ok(())
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
                parse_exact_floats(parts, 3, 3, path, || {
                    format!("ASCII STL '{path}': 'facet normal' requires 3 floats")
                })?;
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
                parse_exact_floats(parts, 3, 3, path, || {
                    format!("ASCII STL '{path}': 'vertex' requires 3 floats")
                })?;
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
                parse_exact_floats(parts, 3, 4, path, || {
                    format!("OBJ '{path}' line {line_no}: 'v' requires 3 or 4 floats")
                })?;
                vertex_count += 1;
                if vertex_count > MAX_TOTAL_MESH_VERTICES {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' exceeds vertex budget of {MAX_TOTAL_MESH_VERTICES}"
                    )));
                }
            }
            "vn" => {
                parse_exact_floats(parts, 3, 3, path, || {
                    format!("OBJ '{path}' line {line_no}: 'vn' requires 3 floats")
                })?;
                vn_count += 1;
            }
            "vt" => {
                parse_exact_floats(parts, 1, 3, path, || {
                    format!("OBJ '{path}' line {line_no}: 'vt' requires 1..=3 floats")
                })?;
                vt_count += 1;
            }
            "f" => {
                let mut f_count = 0usize;
                for vtok in parts {
                    f_count += 1;
                    validate_obj_face_vertex(vtok, vertex_count, vt_count, vn_count, path, line_no)?;
                }
                if f_count < 3 {
                    return Err(PackValidationError::InvalidPack(format!(
                        "OBJ '{path}' line {line_no}: 'f' requires at least 3 vertices"
                    )));
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
            total_vertices = total_vertices.saturating_add(summary.inline_vertex_count);
            check_mesh_budgets(mesh_count, total_vertices)?;
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
        if is_stl || is_obj || mesh_refs.contains(rel) {
            mesh_count += 1;
            check_mesh_budgets(mesh_count, total_vertices)?;
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
            check_mesh_budgets(mesh_count, total_vertices)?;
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
#[serde(deny_unknown_fields)]
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tcp_site: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tcp_offset: Option<[f64; 3]>,
    #[serde(default = "default_suction_range")]
    pub suction_range_m: f64,
    #[serde(default)]
    pub io: Vec<serde_json::Value>,
    #[serde(default)]
    pub assets: Vec<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub armd: Option<crate::tool::ArmdToolOverlay>,
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

/// Resolve effective `tcp_offset` for `manifest`: stock `"default"` defaults to `[0, 0, 0]`,
/// while any non-stock tool must explicitly declare `tcp_offset: [x, y, z]`.
pub fn resolve_tool_tcp_offset(
    manifest: &ToolManifest,
    _tool_xml: Option<&str>,
) -> Result<[f64; 3], String> {
    let resolved = if manifest.id == crate::tool::STOCK_TOOL_ID {
        manifest.tcp_offset.unwrap_or([0.0; 3])
    } else {
        manifest.tcp_offset.ok_or_else(|| {
            format!(
                "non-stock tool '{}' must declare explicit tcp_offset: [x, y, z]",
                manifest.id
            )
        })?
    };
    crate::tool::validate_tcp_offset(&resolved)?;
    Ok(resolved)
}

fn manifest_to_tool_pack(manifest: &ToolManifest, resolved_tcp: [f64; 3]) -> crate::tool::ToolPackConfig {
    crate::tool::ToolPackConfig {
        id: manifest.id.clone(),
        name: manifest.name.clone(),
        kind: manifest.kind.clone(),
        version: manifest.version.clone(),
        description: manifest.description.clone(),
        tool_xml: String::new(),
        robots: manifest.robots.clone(),
        attach_body: manifest.attach_body.clone(),
        replace_bodies: manifest.replace_bodies.clone(),
        replace_geoms: manifest.replace_geoms.clone(),
        replace_equalities: manifest.replace_equalities.clone(),
        replace_actuators: manifest.replace_actuators.clone(),
        driven_joint: manifest.driven_joint.clone(),
        coupled_joint: manifest.coupled_joint.clone(),
        actuator: manifest.actuator.clone(),
        motor_to_joint_scale: Some(manifest.motor_to_joint_scale),
        finger_geoms: manifest.finger_geoms.clone(),
        tcp_site: None,
        tcp_offset: Some(resolved_tcp),
        suction_range_m: Some(manifest.suction_range_m),
        io: manifest.io.clone(),
        assets: manifest.assets.clone(),
        armd: manifest.armd.clone(),
    }
}

/// Validate a [`ToolManifest`] against a specific robot profile using the same hardware merge
/// rules as `armd --tool`.
pub fn validate_tool_manifest_for_robot(
    manifest: &ToolManifest,
    tool_xml: Option<&str>,
    robot_id: &str,
) -> Result<[f64; 3], String> {
    let resolved_tcp = validate_tool_manifest_independent(manifest, tool_xml)?;
    let base = crate::tool::baseline_arm_config_for_robot(robot_id)?;
    if manifest.id == crate::tool::STOCK_TOOL_ID {
        if !manifest.robots.is_empty() {
            let norm_target = crate::tool::normalize_robot_id(robot_id)
                .ok_or_else(|| format!("unknown robot profile '{robot_id}'"))?;
            let mut allowed = Vec::with_capacity(manifest.robots.len());
            for raw in &manifest.robots {
                let norm = crate::tool::normalize_robot_id(raw)
                    .ok_or_else(|| format!("tool '{}': unknown robot target '{raw}'", manifest.id))?;
                allowed.push(norm);
            }
            if !allowed.contains(&norm_target) {
                return Err(format!(
                    "tool '{}': robot '{norm_target}' is not in supported robots {:?}",
                    manifest.id, allowed
                ));
            }
        }
        return Ok(resolved_tcp);
    }
    let pack = manifest_to_tool_pack(manifest, resolved_tcp);
    crate::tool::apply_tool_overlay(&base, &pack)?;
    Ok(resolved_tcp)
}

fn validate_tool_manifest_independent(
    manifest: &ToolManifest,
    tool_xml: Option<&str>,
) -> Result<[f64; 3], String> {
    if !crate::model::valid_tool_id(&manifest.id) {
        return Err("tool id must be 1..64 characters of A-Z, a-z, 0-9, _, -, .".into());
    }
    if !crate::tool::is_valid_tool_kind(&manifest.kind) {
        return Err(format!(
            "tool '{}' has unsupported kind '{}': must be one of gripper, suction, screwdriver",
            manifest.id, manifest.kind
        ));
    }
    crate::tool::validate_attach_body(&manifest.attach_body)?;

    if manifest.id == crate::tool::STOCK_TOOL_ID {
        if manifest.kind != "gripper" {
            return Err("reserved tool id 'default' must have kind 'gripper'".into());
        }
        if tool_xml.is_some_and(|x| !x.trim().is_empty()) {
            return Err("reserved tool id 'default' cannot carry custom tool_xml".into());
        }
        if manifest.tcp_offset.is_some_and(|o| o != [0.0; 3]) {
            return Err("reserved tool id 'default' cannot override tcp_offset".into());
        }
        if manifest.tcp_site.as_deref().is_some_and(|s| s.trim() != "tcp") {
            return Err("reserved tool id 'default' cannot override tcp_site".into());
        }
        if manifest.armd.is_some() {
            return Err("reserved tool id 'default' cannot carry an 'armd' overlay block".into());
        }
    }

    let resolved_tcp = resolve_tool_tcp_offset(manifest, tool_xml)?;
    if manifest.id == crate::tool::STOCK_TOOL_ID && resolved_tcp != [0.0; 3] {
        return Err("reserved tool id 'default' cannot resolve a non-zero tcp_offset".into());
    }

    if let Some(a) = manifest.armd.as_ref() {
        if manifest.kind != "gripper" && (a.gripper_policy.is_some() || a.grasp.is_some()) {
            return Err(format!(
                "tool '{}' with kind '{}' cannot specify armd.gripper_policy or armd.grasp",
                manifest.id, manifest.kind
            ));
        }
        crate::tool::validate_check_points_f64(
            &a.extra_check_points_end_m,
            "armd.extra_check_points_end_m",
        )?;
        if let Some(pc) = &a.physical_contact_end_m {
            crate::tool::validate_physical_contact_end_m(pc, "armd.physical_contact_end_m")?;
        }
        if let Some(co) = a.contact_opening_m {
            crate::tool::validate_contact_opening_m(co, "armd.contact_opening_m")?;
        }
        if let Some(gr) = &a.grasp {
            if let Some(e) = gr.edge_to_low_m {
                crate::tool::checked_to_micro_f64(e, "armd.grasp.edge_to_low_m")?;
                if !(-0.10..=0.10).contains(&e) {
                    return Err("armd.grasp.edge_to_low_m must be finite and in [-0.10, 0.10] m".into());
                }
            }
            if let Some(w) = gr.max_grasp_width_m {
                crate::tool::checked_to_micro_f64(w, "armd.grasp.max_grasp_width_m")?;
                if !(0.0..=0.2).contains(&w) {
                    return Err("armd.grasp.max_grasp_width_m must be in [0, 0.2] m".into());
                }
            }
            if let Some(h) = gr.grasp_height_m {
                crate::tool::checked_to_micro_f64(h, "armd.grasp.grasp_height_m")?;
                if !(0.005..=0.2).contains(&h) {
                    return Err("armd.grasp.grasp_height_m must be in [0.005, 0.2] m".into());
                }
            }
        }
    }

    Ok(resolved_tcp)
}

/// Validate a [`ToolManifest`] (and optional `tool_xml`) for kind, attachment body, reserved
/// `"default"` identity, TCP resolution, and typed `armd` overlay bounds.
pub fn validate_tool_manifest(
    manifest: &ToolManifest,
    tool_xml: Option<&str>,
) -> Result<[f64; 3], String> {
    let resolved_tcp = validate_tool_manifest_independent(manifest, tool_xml)?;

    if manifest.id == crate::tool::STOCK_TOOL_ID {
        for raw in &manifest.robots {
            crate::tool::baseline_arm_config_for_robot(raw)?;
        }
        return Ok(resolved_tcp);
    }

    let pack = manifest_to_tool_pack(manifest, resolved_tcp);
    if manifest.robots.is_empty() {
        const SUPPORTED_ROBOTS: [&str; 4] = ["b601dm", "so101", "so100", "b601rs"];
        let mut last_err = None;
        let mut any_ok = false;
        for robot_id in SUPPORTED_ROBOTS {
            let base = crate::tool::baseline_arm_config_for_robot(robot_id)?;
            match crate::tool::apply_tool_overlay(&base, &pack) {
                Ok(_) => {
                    any_ok = true;
                    break;
                }
                Err(e) => {
                    if last_err.is_none() {
                        last_err = Some(e);
                    }
                }
            }
        }
        if !any_ok {
            return Err(last_err.unwrap_or_else(|| "tool overlay failed on all robot baselines".into()));
        }
    } else {
        for raw in &manifest.robots {
            let base = crate::tool::baseline_arm_config_for_robot(raw)?;
            crate::tool::apply_tool_overlay(&base, &pack)?;
        }
    }

    Ok(resolved_tcp)
}

fn parse_mjcf_vec3(raw: &str, label: &str) -> Result<[f64; 3], String> {
    let vals: Vec<f64> = raw
        .split_whitespace()
        .map(|s| s.parse::<f64>().map_err(|e| format!("invalid {label} float '{s}': {e}")))
        .collect::<Result<_, _>>()?;
    if vals.len() != 3 || !vals.iter().all(|v| v.is_finite()) {
        return Err(format!("base MJCF <site name=\"tcp\"> {label} must have 3 finite numbers"));
    }
    Ok([vals[0], vals[1], vals[2]])
}

fn parse_mjcf_compiler_euler(doc: &roxmltree::Document) -> Result<(f64, String), String> {
    let mut angle_scale = std::f64::consts::PI / 180.0; // MuJoCo default: degree
    let mut eulerseq = "xyz".to_string(); // MuJoCo default: xyz (intrinsic)
    for node in doc.descendants() {
        if node.is_element() && node.tag_name().name() == "compiler" {
            if let Some(a) = node.attribute("angle") {
                match a.trim() {
                    "degree" => angle_scale = std::f64::consts::PI / 180.0,
                    "radian" => angle_scale = 1.0,
                    other => return Err(format!("base MJCF <compiler> uses unsupported angle unit '{other}'")),
                }
            }
            if let Some(seq) = node.attribute("eulerseq") {
                eulerseq = seq.trim().to_string();
            }
        }
    }
    let bytes = eulerseq.as_bytes();
    let all_lower = bytes.len() == 3 && bytes.iter().all(|b| matches!(b, b'x' | b'y' | b'z'));
    let all_upper = bytes.len() == 3 && bytes.iter().all(|b| matches!(b, b'X' | b'Y' | b'Z'));
    if !(all_lower || all_upper) || bytes[0] == bytes[1] || bytes[1] == bytes[2] {
        return Err(format!("base MJCF <compiler> uses unsupported eulerseq '{eulerseq}'"));
    }
    Ok((angle_scale, eulerseq))
}

fn axis_rot(axis: u8, angle_rad: f64) -> crate::kinematics::Mat3 {
    let (c, s) = (angle_rad.cos(), angle_rad.sin());
    match axis.to_ascii_lowercase() {
        b'x' => [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        b'y' => [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        b'z' => [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        _ => crate::kinematics::IDENTITY,
    }
}

fn parse_mjcf_site_rotation(doc: &roxmltree::Document, node: roxmltree::Node) -> Result<crate::kinematics::Mat3, String> {
    if let Some(q_raw) = node.attribute("quat") {
        let vals: Vec<f64> = q_raw
            .split_whitespace()
            .map(|s| s.parse::<f64>().map_err(|e| format!("invalid tcp site quat float '{s}': {e}")))
            .collect::<Result<_, _>>()?;
        if vals.len() != 4 || !vals.iter().all(|v| v.is_finite()) {
            return Err("base MJCF <site name=\"tcp\"> quat must have 4 finite numbers".into());
        }
        let norm = (vals[0] * vals[0] + vals[1] * vals[1] + vals[2] * vals[2] + vals[3] * vals[3]).sqrt();
        if norm <= 1e-12 {
            return Err("base MJCF <site name=\"tcp\"> quat has zero norm".into());
        }
        let (w, x, y, z) = (vals[0] / norm, vals[1] / norm, vals[2] / norm, vals[3] / norm);
        return Ok([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]);
    }
    if let Some(e_raw) = node.attribute("euler") {
        let vals = parse_mjcf_vec3(e_raw, "euler")?;
        let (scale, seq) = parse_mjcf_compiler_euler(doc)?;
        let ang = [vals[0] * scale, vals[1] * scale, vals[2] * scale];
        let b = seq.as_bytes();
        let r0 = axis_rot(b[0], ang[0]);
        let r1 = axis_rot(b[1], ang[1]);
        let r2 = axis_rot(b[2], ang[2]);
        // MuJoCo convention: lowercase = intrinsic (R0 * R1 * R2), uppercase = extrinsic (R2 * R1 * R0).
        let rot = if b[0].is_ascii_lowercase() {
            crate::kinematics::mat_mul(&crate::kinematics::mat_mul(&r0, &r1), &r2)
        } else {
            crate::kinematics::mat_mul(&crate::kinematics::mat_mul(&r2, &r1), &r0)
        };
        return Ok(rot);
    }
    for unsupported in ["axisangle", "xyaxes", "zaxis"] {
        if node.attribute(unsupported).is_some() {
            return Err(format!(
                "base MJCF <site name=\"tcp\"> uses unsupported orientation attribute '{unsupported}'"
            ));
        }
    }
    Ok(crate::kinematics::IDENTITY)
}

fn check_tcp_site_not_in_frame(node: roxmltree::Node) -> Result<(), String> {
    if node
        .ancestors()
        .skip(1)
        .any(|a| a.is_element() && a.tag_name().name() == "frame")
    {
        return Err(
            "base MJCF <site name=\"tcp\"> inside unsupported <frame> ancestor; declare <site name=\"tcp\"> directly inside its host <body>"
                .into(),
        );
    }
    Ok(())
}

fn resolve_attach_body_name(base_doc: &roxmltree::Document, requested: &str) -> Result<String, String> {
    let req = requested.trim();
    if req.is_empty() || req == "end_link" {
        for site in base_doc.descendants() {
            if site.is_element() && site.tag_name().name() == "site" && site.attribute("name") == Some("tcp") {
                check_tcp_site_not_in_frame(site)?;
                for anc in site.ancestors().skip(1) {
                    if anc.is_element() && anc.tag_name().name() == "body" {
                        if let Some(bname) = anc.attribute("name").map(str::trim).filter(|s| !s.is_empty()) {
                            return Ok(bname.to_string());
                        }
                    }
                }
            }
        }
        if base_doc
            .descendants()
            .any(|n| n.is_element() && n.tag_name().name() == "body" && n.attribute("name") == Some("end_link"))
        {
            return Ok("end_link".to_string());
        }
    } else if base_doc
        .descendants()
        .any(|n| n.is_element() && n.tag_name().name() == "body" && n.attribute("name") == Some(req))
    {
        return Ok(req.to_string());
    }
    let label = if req.is_empty() { "end_link" } else { req };
    Err(format!("Could not find attach body <body name=\"{label}\"> in base XML"))
}

fn find_xml_attr_value_span(tag_xml: &str, attr_name: &str) -> Option<(usize, usize)> {
    let bytes = tag_xml.as_bytes();
    let mut i = 0;
    if bytes.get(i) != Some(&b'<') {
        return None;
    }
    i += 1;
    while i < bytes.len() && !bytes[i].is_ascii_whitespace() && bytes[i] != b'>' && bytes[i] != b'/' {
        i += 1;
    }
    while i < bytes.len() {
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if i >= bytes.len() || bytes[i] == b'>' || bytes[i] == b'/' {
            break;
        }
        let name_start = i;
        while i < bytes.len()
            && !bytes[i].is_ascii_whitespace()
            && bytes[i] != b'='
            && bytes[i] != b'>'
            && bytes[i] != b'/'
        {
            i += 1;
        }
        let name = &tag_xml[name_start..i];
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if bytes.get(i) != Some(&b'=') {
            continue;
        }
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        let Some(&quote) = bytes.get(i) else { break };
        if quote != b'"' && quote != b'\'' {
            break;
        }
        i += 1;
        let val_start = i;
        while i < bytes.len() && bytes[i] != quote {
            i += 1;
        }
        if i >= bytes.len() {
            break;
        }
        let val_end = i;
        i += 1;
        if name == attr_name {
            return Some((val_start, val_end));
        }
    }
    None
}

fn extract_children_xml(
    node: roxmltree::Node,
    raw_xml: &str,
    skip_range: Option<std::ops::Range<usize>>,
) -> String {
    let mut out = String::new();
    for child in node.children() {
        if child.is_element() {
            let r = child.range();
            if let Some(ref sr) = skip_range {
                if sr.start >= r.start && sr.end <= r.end {
                    if sr.start == r.start && sr.end == r.end {
                        continue;
                    }
                    out.push_str(&raw_xml[r.start..sr.start]);
                    out.push_str(&raw_xml[sr.end..r.end]);
                    out.push('\n');
                    continue;
                }
            }
            out.push_str(&raw_xml[r]);
            out.push('\n');
        }
    }
    out
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

    if let Some(tm) = tool_manifest {
        validate_tool_manifest(tm, tool_xml)?;
    }

    // 2. Tool elements to remove & inject
    let is_non_stock_tool = tool_manifest.is_some_and(|m| m.id.as_str() != crate::tool::STOCK_TOOL_ID);
    let is_tool_active = is_non_stock_tool && tool_doc.is_some();

    let mut tool_worldbody_xml = String::new();
    let mut tool_asset_xml = String::new();
    let mut tool_equality_xml = String::new();
    let mut tool_actuator_xml = String::new();
    let mut tool_tendon_xml = String::new();
    let mut resolved_attach_body: Option<String> = None;

    if is_non_stock_tool {
        let tm = tool_manifest.unwrap();
        let resolved_tcp = resolve_tool_tcp_offset(tm, tool_xml)?;
        let skip_range = if let (Some(td), Some(raw_tool_xml)) = (tool_doc.as_ref(), tool_xml) {
            let site_name = tm.tcp_site.as_deref().unwrap_or("tcp");
            let skip_range = if site_name.trim().is_empty() {
                None
            } else {
                td.descendants()
                    .find(|n| {
                        n.is_element()
                            && n.tag_name().name() == "site"
                            && n.attribute("name") == Some(site_name)
                    })
                    .map(|n| n.range())
            };

            // Extract tool sections
            for node in td.descendants() {
                if node.is_element() {
                    match node.tag_name().name() {
                        "worldbody" => {
                            let wb = extract_children_xml(node, raw_tool_xml, skip_range.clone());
                            tool_worldbody_xml.push_str(&wb);
                        }
                        "asset" => {
                            tool_asset_xml.push_str(&extract_children_xml(node, raw_tool_xml, None));
                        }
                        "equality" => {
                            tool_equality_xml.push_str(&extract_children_xml(node, raw_tool_xml, None));
                        }
                        "actuator" => {
                            tool_actuator_xml.push_str(&extract_children_xml(node, raw_tool_xml, None));
                        }
                        "tendon" => {
                            tool_tendon_xml.push_str(&extract_children_xml(node, raw_tool_xml, None));
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

            // Insert tool worldbody children inside resolved attach_body (e.g. `end_link` or SO-101's `gripper`).
            // If the attach body's `<site name="tcp">` has a non-zero zero-offset pos or non-identity orientation,
            // wrap the tool worldbody XML in `<body name="__tool_mount_frame" pos="..." quat="...">` so tool
            // geometries declared relative to tool mount origin keep their exact pose relative to `<site name="tcp">`.
            if !tool_worldbody_xml.is_empty() {
                let attach_name = resolve_attach_body_name(&base_doc, &tm.attach_body)?;
                resolved_attach_body = Some(attach_name.clone());
                let mount_wrapped_xml = {
                    let tcp_node = base_doc.descendants().find(|n| {
                        n.is_element()
                            && n.tag_name().name() == "site"
                            && n.attribute("name") == Some("tcp")
                            && !edits.iter().any(|e| {
                                e.replacement.is_empty()
                                    && n.range().start >= e.start
                                    && n.range().end <= e.end
                            })
                    });
                    if let Some(site_node) = tcp_node {
                        check_tcp_site_not_in_frame(site_node)?;
                        let site_host = site_node
                            .ancestors()
                            .skip(1)
                            .find(|a| a.is_element() && a.tag_name().name() == "body")
                            .and_then(|b| b.attribute("name").map(str::trim));
                        if site_host == Some(attach_name.as_str()) {
                            let base_pos = match site_node.attribute("pos") {
                                Some(raw) => parse_mjcf_vec3(raw, "pos")?,
                                None => [0.0; 3],
                            };
                            let r_site = parse_mjcf_site_rotation(&base_doc, site_node)?;
                            let has_pos = base_pos.iter().any(|v| v.abs() > 1e-12);
                            let has_rot = (0..3).any(|r| {
                                (0..3).any(|c| (r_site[r][c] - crate::kinematics::IDENTITY[r][c]).abs() > 1e-9)
                            });
                            if has_pos || has_rot {
                                let q = crate::kinematics::r_to_quat(&r_site);
                                let fmt = |v: f64| if v == 0.0 { 0.0 } else { v };
                                format!(
                                    "<body name=\"__tool_mount_frame\" pos=\"{} {} {}\" quat=\"{} {} {} {}\">\n{}</body>",
                                    fmt(base_pos[0]),
                                    fmt(base_pos[1]),
                                    fmt(base_pos[2]),
                                    fmt(q[0]),
                                    fmt(q[1]),
                                    fmt(q[2]),
                                    fmt(q[3]),
                                    tool_worldbody_xml
                                )
                            } else {
                                tool_worldbody_xml.clone()
                            }
                        } else {
                            tool_worldbody_xml.clone()
                        }
                    } else {
                        tool_worldbody_xml.clone()
                    }
                };
                let mut attached = false;
                for node in base_doc.descendants() {
                    if node.is_element() && node.tag_name().name() == "body" && node.attribute("name") == Some(attach_name.as_str()) {
                        let range = node.range();
                        let slice = &base_xml[range.clone()];
                        if let Some(close_idx) = slice.rfind("</body") {
                            let insert_pos = range.start + close_idx;
                            edits.push(XmlEdit {
                                start: insert_pos,
                                end: insert_pos,
                                replacement: format!("\n{}\n", mount_wrapped_xml),
                            });
                            attached = true;
                            break;
                        }
                    }
                }
                if !attached {
                    return Err(format!("Could not find attach body <body name=\"{attach_name}\"> in base XML"));
                }
            }

            skip_range
        } else {
            None
        };

        // Update base TCP site pos relative to the base site's zero-offset pose
        // (p_tcp = p_base_site + R_base_site * tcp_offset), preserving base pose for [0, 0, 0].
        let should_override_tcp_site = tm.tcp_offset.is_some() || skip_range.is_some();
        if should_override_tcp_site {
            let mut tcp_site_updated = false;
            for node in base_doc.descendants() {
                if node.is_element() && node.tag_name().name() == "site" && node.attribute("name") == Some("tcp") {
                    let range = node.range();
                    let overlaps_delete = edits.iter().any(|e| {
                        e.replacement.is_empty() && range.start >= e.start && range.end <= e.end
                    });
                    if overlaps_delete {
                        continue;
                    }
                    check_tcp_site_not_in_frame(node)?;
                    let base_pos = match node.attribute("pos") {
                        Some(raw) => parse_mjcf_vec3(raw, "pos")?,
                        None => [0.0; 3],
                    };
                    let composed_pos = if resolved_tcp == [0.0; 3] {
                        base_pos
                    } else {
                        let r_site = parse_mjcf_site_rotation(&base_doc, node)?;
                        crate::kinematics::add(
                            &base_pos,
                            &crate::kinematics::mat_vec(&r_site, &resolved_tcp),
                        )
                    };
                    let fmt = |v: f64| if v == 0.0 { 0.0 } else { v };
                    let pos_val = format!(
                        "{} {} {}",
                        fmt(composed_pos[0]),
                        fmt(composed_pos[1]),
                        fmt(composed_pos[2])
                    );
                    let site_str = &base_xml[range.clone()];
                    if let Some((val_start, val_end)) = find_xml_attr_value_span(site_str, "pos") {
                        edits.push(XmlEdit {
                            start: range.start + val_start,
                            end: range.start + val_end,
                            replacement: pos_val,
                        });
                        tcp_site_updated = true;
                        break;
                    } else if let Some((_name_start, name_end)) = find_xml_attr_value_span(site_str, "name") {
                        let insert_pos = range.start + name_end + 1;
                        edits.push(XmlEdit {
                            start: insert_pos,
                            end: insert_pos,
                            replacement: format!(" pos=\"{pos_val}\""),
                        });
                        tcp_site_updated = true;
                        break;
                    }
                }
            }
            if !tcp_site_updated {
                return Err("Base MJCF is missing an active <site name=\"tcp\"> element to update tcp_offset".into());
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
                        scene_worldbody_xml.push_str(&extract_children_xml(node, scene_world_xml.unwrap(), None));
                    }
                    "asset" => {
                        scene_asset_xml.push_str(&extract_children_xml(node, scene_world_xml.unwrap(), None));
                    }
                    "equality" => {
                        scene_equality_xml.push_str(&extract_children_xml(node, scene_world_xml.unwrap(), None));
                    }
                    "actuator" => {
                        scene_actuator_xml.push_str(&extract_children_xml(node, scene_world_xml.unwrap(), None));
                    }
                    "tendon" => {
                        scene_tendon_xml.push_str(&extract_children_xml(node, scene_world_xml.unwrap(), None));
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
        let mut free_bodies = Vec::new();
        if let Some(sd) = scene_doc.as_ref() {
            free_bodies.extend(find_free_body_names(sd));
        } else {
            free_bodies.extend(find_free_body_names(&base_doc));
        }
        if !free_bodies.is_empty() {
            let attach_name = match resolved_attach_body.as_deref() {
                Some(name) => name.to_string(),
                None => resolve_attach_body_name(&base_doc, &tool_manifest.unwrap().attach_body)?,
            };
            for (i, bname) in free_bodies.iter().enumerate() {
                suction_welds_xml.push_str(&format!(
                    "    <weld name=\"suction_weld_{i}_{bname}\" body1=\"{attach_name}\" body2=\"{bname}\" active=\"false\" solref=\"0.02 1\" solimp=\"0.9 0.95 0.001 0.5 2\"/>\n"
                ));
            }
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
            tcp_site: None,
            tcp_offset: None,
            suction_range_m: 0.030,
            io: vec![],
            assets: vec![],
            armd: None,
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
            tcp_site: None,
            tcp_offset: Some([0.0, 0.0, 0.0]),
            suction_range_m: 0.030,
            io: vec![
                serde_json::json!({"name": "vacuum", "kind": "bool", "direction": "output", "default": false}),
                serde_json::json!({"name": "vacuum_engaged", "kind": "bool", "direction": "input", "default": false}),
            ],
            assets: vec![],
            armd: None,
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
            tcp_site: None,
            tcp_offset: Some([0.0, 0.0, 0.0]),
            suction_range_m: 0.030,
            io: vec![
                serde_json::json!({"name": "spindle_rpm", "kind": "float", "direction": "output", "default": 0.0}),
            ],
            assets: vec![],
            armd: None,
        };
        return Some((manifest, PRESET_SCREWDRIVER_XML));
    }

    None
}

pub fn default_tool_manifest() -> ToolManifest {
    builtin_tool_preset("default").expect("built-in default tool").0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tool_manifest_preserves_armd_block_across_roundtrip() {
        let json = serde_json::json!({
            "id": "soft_finray_gripper",
            "name": "Soft FinRay",
            "kind": "gripper",
            "version": "1.0.0",
            "tool_xml": "tool.xml",
            "robots": ["b601dm"],
            "tcp_offset": [0.0288, 0.0, 0.0],
            "armd": {
                "physical_contact_end_m": [0.0288, 0.0, 0.0],
                "contact_opening_m": 0.0036
            }
        });
        let manifest: ToolManifest = serde_json::from_value(json.clone()).unwrap();
        assert_eq!(manifest.armd.as_ref().unwrap().physical_contact_end_m, Some([0.0288, 0.0, 0.0]));
        assert_eq!(manifest.armd.as_ref().unwrap().contact_opening_m, Some(0.0036));
        let rt = serde_json::to_value(&manifest).unwrap();
        assert_eq!(rt["armd"]["physical_contact_end_m"], json["armd"]["physical_contact_end_m"]);
        assert_eq!(rt["armd"]["contact_opening_m"], json["armd"]["contact_opening_m"]);
        assert_eq!(rt["tcp_offset"], json["tcp_offset"]);
    }

    #[test]
    fn non_stock_requires_explicit_tcp_offset_and_writes_tcp_site() {
        let xml_tool = r#"<mujoco model="finray_tool">
          <worldbody>
            <site name="finray_tcp" pos="0.0288 0.0 0.0"/>
            <body name="tip" pos="0.02 0 0"><geom type="box" size="0.01 0.01 0.01"/></body>
          </worldbody>
        </mujoco>"#;

        // 1. Omitted tcp_offset on non-stock tool MUST be rejected even if tool.xml has a tcp_site
        let mut manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "finray_test",
            "kind": "gripper",
            "tcp_site": "finray_tcp"
        })).unwrap();
        assert!(manifest.tcp_offset.is_none());
        let err_resolve = resolve_tool_tcp_offset(&manifest, Some(xml_tool)).unwrap_err();
        assert!(err_resolve.contains("tcp_offset"), "{err_resolve}");
        let err_validate = validate_tool_manifest(&manifest, Some(xml_tool)).unwrap_err();
        assert!(err_validate.contains("tcp_offset"), "{err_validate}");
        let err_compose = compose_mjcf(FALLBACK_BASE_XML, None, Some(&manifest), Some(xml_tool)).unwrap_err();
        assert!(err_compose.contains("tcp_offset"), "{err_compose}");

        // 2. Explicit tcp_offset writes <site name="tcp" pos="..."/> and strips declared tcp_site
        manifest.tcp_offset = Some([0.0288, 0.0, 0.0]);
        let composed = compose_mjcf(FALLBACK_BASE_XML, None, Some(&manifest), Some(xml_tool)).unwrap();
        let doc = roxmltree::Document::parse(&composed).unwrap();
        let tcp_site = doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        assert_eq!(tcp_site.attribute("pos"), Some("0.0288 0 0"));
        assert!(!composed.contains("finray_tcp"), "tool tcp_site must be stripped");

        // 3. Explicit [0,0,0] forces <site name="tcp" pos="0 0 0"/> even when XML site has nonzero pos
        manifest.tcp_offset = Some([0.0, 0.0, 0.0]);
        let composed_zero = compose_mjcf(FALLBACK_BASE_XML, None, Some(&manifest), Some(xml_tool)).unwrap();
        let doc_zero = roxmltree::Document::parse(&composed_zero).unwrap();
        let tcp_zero = doc_zero
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        assert_eq!(tcp_zero.attribute("pos"), Some("0 0 0"));

        // 4. M12: Sandbox manifest validation merges armd.gripper_policy & armd.grasp using hardware rules
        let bad_gp_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "bad_gp_tool",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.0288, 0.0, 0.0],
            "armd": {
                "gripper_policy": {
                    "max_close_torque_nm": -0.5
                }
            }
        })).unwrap();
        assert!(validate_tool_manifest(&bad_gp_manifest, None).is_err());

        // 5. M12 (R3): Empty `robots: []` valid only under SO-101 positive-open jaw bounds must
        // pass `validate_tool_manifest_for_robot(..., "so101")` without B601-DM false rejection.
        let so101_only_overlay: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "so101_positive_jaw",
            "kind": "gripper",
            "robots": [],
            "tcp_offset": [0.02, 0.0, 0.0],
            "armd": {
                "gripper_policy": {
                    "closed_rad": 0.0,
                    "open_rad": 1.4,
                    "jaw_opening_nominal": [[0.0, 0.0], [1.4, 0.065]]
                }
            }
        })).unwrap();
        assert!(validate_tool_manifest_for_robot(&so101_only_overlay, None, "so101").is_ok());
        assert!(validate_tool_manifest(&so101_only_overlay, None).is_ok());
        assert!(validate_tool_manifest_for_robot(&so101_only_overlay, None, "b601dm").is_err());

        // 6. M3: Without `armd` overlay, robot compatibility and gripper capability are still enforced.
        let gripper_no_armd_rs: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "cup_gripper_no_armd",
            "kind": "gripper",
            "robots": ["b601rs"],
            "tcp_offset": [0.02, 0.0, 0.0]
        })).unwrap();
        assert!(validate_tool_manifest(&gripper_no_armd_rs, None).is_err());
        assert!(validate_tool_manifest_for_robot(&gripper_no_armd_rs, None, "b601rs").is_err());
        let gripper_no_armd_any: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "cup_gripper_any",
            "kind": "gripper",
            "robots": [],
            "tcp_offset": [0.02, 0.0, 0.0]
        })).unwrap();
        assert!(validate_tool_manifest_for_robot(&gripper_no_armd_any, None, "b601rs").is_err());
        let unknown_robot_no_armd: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "suction_unknown_bot",
            "kind": "suction",
            "robots": ["mystery_bot"],
            "tcp_offset": [0.0, 0.0, 0.0]
        })).unwrap();
        assert!(validate_tool_manifest(&unknown_robot_no_armd, None).is_err());

        // 7. M4: Single-quoted <site name='tcp' pos='0 0 0'/> updates properly; missing tcp site errors.
        let single_quote_base = r#"<mujoco model="sq">
          <worldbody>
            <body name="end_link">
              <site name='tcp' pos='0 0 0'/>
            </body>
          </worldbody>
        </mujoco>"#;
        let sq_tool_xml = r#"<mujoco><worldbody><body name="t"><geom type="box" size="0.01 0.01 0.01"/></body></worldbody></mujoco>"#;
        let sq_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "sq_tool",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.025, 0.0, 0.0]
        })).unwrap();
        let sq_composed = compose_mjcf(single_quote_base, None, Some(&sq_manifest), Some(sq_tool_xml)).unwrap();
        let sq_doc = roxmltree::Document::parse(&sq_composed).unwrap();
        let sq_site = sq_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        assert_eq!(sq_site.attribute("pos"), Some("0.025 0 0"));

        let missing_tcp_base = r#"<mujoco model="no_tcp">
          <worldbody>
            <body name="end_link">
              <geom type="box" size="0.01 0.01 0.01"/>
            </body>
          </worldbody>
        </mujoco>"#;
        let missing_err = compose_mjcf(missing_tcp_base, None, Some(&sq_manifest), Some(sq_tool_xml)).unwrap_err();
        assert!(missing_err.contains("tcp"), "{missing_err}");

        // 8. m1: ToolManifest rejects unknown top-level fields (e.g. "armd_").
        assert!(serde_json::from_value::<ToolManifest>(serde_json::json!({
            "id": "typo_tool",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.02, 0.0, 0.0],
            "armd_": {}
        })).is_err());

        // 9. R2 M3: Non-stock manifest with empty/whitespace/None tool_xml STILL updates base tcp site
        // and fails closed if base MJCF lacks <site name="tcp">.
        for empty_xml in [None, Some(""), Some("   \n\t ")] {
            let composed_empty = compose_mjcf(single_quote_base, None, Some(&sq_manifest), empty_xml).unwrap();
            let doc_empty = roxmltree::Document::parse(&composed_empty).unwrap();
            let site_empty = doc_empty
                .descendants()
                .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
                .unwrap();
            assert_eq!(site_empty.attribute("pos"), Some("0.025 0 0"), "tool_xml={empty_xml:?}");

            let missing_err_empty = compose_mjcf(missing_tcp_base, None, Some(&sq_manifest), empty_xml).unwrap_err();
            assert!(missing_err_empty.contains("tcp"), "{missing_err_empty}");
        }

        // 10. R3 M3: SO-101 base MJCF TCP site preserves its non-zero zero-offset pos for [0,0,0]
        // and rotates non-zero tcp_offset through the site quat matching ArmConfig::end_to_tcp.
        let so101_base_xml = include_str!("../sim/mujoco/models/so_arm101_scene.xml");
        let so101_zero_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "so101_zero_offset",
            "kind": "gripper",
            "robots": ["so-arm101"],
            "tcp_offset": [0.0, 0.0, 0.0],
            "armd": {
                "gripper_policy": {
                    "jaw_opening_nominal": [[0.0, 0.0], [1.5, 0.065]]
                }
            }
        })).unwrap();
        let so101_zero_xml = compose_mjcf(so101_base_xml, None, Some(&so101_zero_manifest), None).unwrap();
        let so101_zero_doc = roxmltree::Document::parse(&so101_zero_xml).unwrap();
        let so101_zero_site = so101_zero_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        let zero_pos = parse_mjcf_vec3(so101_zero_site.attribute("pos").unwrap(), "pos").unwrap();
        assert!((zero_pos[0] - (-0.0079)).abs() < 1e-9, "zero_pos={zero_pos:?}");
        assert!((zero_pos[1] - (-0.000218121)).abs() < 1e-9, "zero_pos={zero_pos:?}");
        assert!((zero_pos[2] - (-0.0981274)).abs() < 1e-9, "zero_pos={zero_pos:?}");

        let so101_offset_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "so101_offset_tool",
            "kind": "gripper",
            "robots": ["so-arm101"],
            "tcp_offset": [0.025, 0.010, -0.005],
            "armd": {
                "gripper_policy": {
                    "jaw_opening_nominal": [[0.0, 0.0], [1.5, 0.065]]
                }
            }
        })).unwrap();
        let so101_offset_xml = compose_mjcf(so101_base_xml, None, Some(&so101_offset_manifest), None).unwrap();
        let so101_offset_doc = roxmltree::Document::parse(&so101_offset_xml).unwrap();
        let so101_offset_site = so101_offset_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        let offset_pos = parse_mjcf_vec3(so101_offset_site.attribute("pos").unwrap(), "pos").unwrap();
        let r_site = parse_mjcf_site_rotation(&so101_zero_doc, so101_zero_site).unwrap();
        let expected_offset_pos = crate::kinematics::add(
            &zero_pos,
            &crate::kinematics::mat_vec(&r_site, &[0.025, 0.010, -0.005]),
        );
        for axis in 0..3 {
            assert!(
                (offset_pos[axis] - expected_offset_pos[axis]).abs() < 1e-6,
                "axis={axis} offset_pos={offset_pos:?} expected={expected_offset_pos:?}"
            );
        }
        // Site x-axis in SO-101 gripper body points along -Z (0, 0, -1), y along -X (-1, 0, 0), z along +Y (0, 1, 0).
        assert!((offset_pos[0] - (-0.0079 - 0.010)).abs() < 1e-5, "offset_pos={offset_pos:?}");
        assert!((offset_pos[1] - (-0.000218121 - 0.005)).abs() < 1e-5, "offset_pos={offset_pos:?}");
        assert!((offset_pos[2] - (-0.0981274 - 0.025)).abs() < 1e-5, "offset_pos={offset_pos:?}");

        // 11. R4 M5 + m2 & R5 M6: Shipped SO-101 (no `end_link`, tcp site inside `gripper` with non-zero pos+quat)
        // resolves attach_body="end_link" (or empty) to `gripper`, wraps tool worldbody children in
        // `<body name="__tool_mount_frame">` matching base TCP site pos+quat so tool geom pose relative to
        // updated `<site name="tcp">` matches declared `tcp_offset`, and sets suction weld body1="gripper".
        let so101_geom_tool = r#"<mujoco><worldbody><geom name="custom_pad" type="box" size="0.01 0.01 0.01" pos="0 0 0"/></worldbody></mujoco>"#;
        let so101_geom_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "so101_custom_pad",
            "kind": "gripper",
            "robots": ["so-arm101"],
            "attach_body": "end_link",
            "tcp_offset": [0.01, 0.0, 0.0],
            "armd": { "gripper_policy": { "jaw_opening_nominal": [[0.0, 0.0], [1.5, 0.065]] } }
        })).unwrap();
        let so101_with_geom = compose_mjcf(so101_base_xml, None, Some(&so101_geom_manifest), Some(so101_geom_tool)).unwrap();
        let so101_geom_doc = roxmltree::Document::parse(&so101_with_geom).unwrap();
        let pad_geom = so101_geom_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "geom" && n.attribute("name") == Some("custom_pad"))
            .expect("custom_pad geom inserted");
        let mount_frame = pad_geom.parent().expect("mount frame parent");
        assert_eq!(mount_frame.attribute("name"), Some("__tool_mount_frame"));
        assert_eq!(
            mount_frame.parent().and_then(|p| p.attribute("name")),
            Some("gripper"),
            "SO-101 attach_body='end_link' must attach under `gripper`"
        );
        let mf_pos = parse_mjcf_vec3(mount_frame.attribute("pos").unwrap(), "pos").unwrap();
        for axis in 0..3 {
            assert!((mf_pos[axis] - zero_pos[axis]).abs() < 1e-6);
        }
        let mf_r = parse_mjcf_site_rotation(&so101_geom_doc, mount_frame).unwrap();
        for r in 0..3 {
            for c in 0..3 {
                assert!((mf_r[r][c] - r_site[r][c]).abs() < 1e-6);
            }
        }
        let pad_tcp = so101_geom_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
            .unwrap();
        let pad_tcp_pos = parse_mjcf_vec3(pad_tcp.attribute("pos").unwrap(), "pos").unwrap();
        // Relative vector (pad_tcp_pos - mf_pos) transformed into mount_frame local frame equals [0.01, 0, 0].
        let rel_gripper = crate::kinematics::sub(&pad_tcp_pos, &mf_pos);
        let rel_local = crate::kinematics::mat_vec(&crate::kinematics::transpose(&mf_r), &rel_gripper);
        assert!(
            (rel_local[0] - 0.01).abs() < 1e-6 && rel_local[1].abs() < 1e-6 && rel_local[2].abs() < 1e-6,
            "rel_local={rel_local:?}"
        );

        let so101_suction_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "so101_suction_cup",
            "kind": "suction",
            "robots": ["so-arm101"],
            "attach_body": "end_link",
            "tcp_offset": [0.02, 0.0, 0.0]
        })).unwrap();
        let prop_scene = r#"<mujoco><worldbody><body name="chip" pos="0.2 0 0.02"><freejoint/><geom type="box" size="0.01 0.01 0.01"/></body></worldbody></mujoco>"#;
        let so101_suction_xml = compose_mjcf(so101_base_xml, Some(prop_scene), Some(&so101_suction_manifest), Some(so101_geom_tool)).unwrap();
        let so101_suction_doc = roxmltree::Document::parse(&so101_suction_xml).unwrap();
        let weld = so101_suction_doc
            .descendants()
            .find(|n| n.is_element() && n.tag_name().name() == "weld" && n.attribute("body2") == Some("chip"))
            .expect("suction weld inserted");
        assert_eq!(weld.attribute("body1"), Some("gripper"));

        // 12. R4 m1 & R5 m5: `euler` respects <compiler angle="degree|radian" eulerseq="..."/>,
        // verifies multi-axis intrinsic xyz vs extrinsic XYZ rotations, and fails closed on unsupported eulerseq.
        let deg_base = r#"<mujoco><worldbody><body name="end_link"><site name="tcp" pos="0 0 0" euler="0 0 90"/></body></worldbody></mujoco>"#;
        let deg_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "deg_tool", "kind": "suction", "robots": ["b601dm"], "tcp_offset": [0.02, 0.0, 0.0]
        })).unwrap();
        let deg_out = compose_mjcf(deg_base, None, Some(&deg_manifest), None).unwrap();
        let deg_doc = roxmltree::Document::parse(&deg_out).unwrap();
        let deg_pos = parse_mjcf_vec3(
            deg_doc.descendants().find(|n| n.attribute("name") == Some("tcp")).unwrap().attribute("pos").unwrap(),
            "pos",
        ).unwrap();
        // Default degree + xyz: 90 deg about z maps +x -> +y: [0, 0.02, 0]
        assert!(deg_pos[0].abs() < 1e-6 && (deg_pos[1] - 0.02).abs() < 1e-6 && deg_pos[2].abs() < 1e-6, "deg_pos={deg_pos:?}");

        // Multi-axis intrinsic xyz with explicit angle="radian": R = Rx(pi/2) * Ry(pi/2) * Rz(0)
        // [0.02, 0, 0] -> Ry(pi/2)*[0.02,0,0]=[0,0,-0.02] -> Rx(pi/2)*[0,0,-0.02]=[0,0.02,0]
        let rad_xyz_base = r#"<mujoco><compiler angle="radian" eulerseq="xyz"/><worldbody><body name="end_link"><site name="tcp" pos="0 0 0" euler="1.5707963267948966 1.5707963267948966 0"/></body></worldbody></mujoco>"#;
        let rad_xyz_out = compose_mjcf(rad_xyz_base, None, Some(&deg_manifest), None).unwrap();
        let rad_xyz_doc = roxmltree::Document::parse(&rad_xyz_out).unwrap();
        let rad_xyz_pos = parse_mjcf_vec3(
            rad_xyz_doc.descendants().find(|n| n.attribute("name") == Some("tcp")).unwrap().attribute("pos").unwrap(),
            "pos",
        ).unwrap();
        assert!(
            rad_xyz_pos[0].abs() < 1e-6 && (rad_xyz_pos[1] - 0.02).abs() < 1e-6 && rad_xyz_pos[2].abs() < 1e-6,
            "rad_xyz_pos={rad_xyz_pos:?}"
        );

        // Multi-axis extrinsic XYZ with explicit angle="radian": R = Rz(0) * Ry(pi/2) * Rx(pi/2)
        // Rx(pi/2)*[0.02,0,0]=[0.02,0,0] -> Ry(pi/2)*[0.02,0,0]=[0,0,-0.02]
        let rad_ext_base = r#"<mujoco><compiler angle="radian" eulerseq="XYZ"/><worldbody><body name="end_link"><site name="tcp" pos="0 0 0" euler="1.5707963267948966 1.5707963267948966 0"/></body></worldbody></mujoco>"#;
        let rad_ext_out = compose_mjcf(rad_ext_base, None, Some(&deg_manifest), None).unwrap();
        let rad_ext_doc = roxmltree::Document::parse(&rad_ext_out).unwrap();
        let rad_ext_pos = parse_mjcf_vec3(
            rad_ext_doc.descendants().find(|n| n.attribute("name") == Some("tcp")).unwrap().attribute("pos").unwrap(),
            "pos",
        ).unwrap();
        assert!(
            rad_ext_pos[0].abs() < 1e-6 && rad_ext_pos[1].abs() < 1e-6 && (rad_ext_pos[2] + 0.02).abs() < 1e-6,
            "rad_ext_pos={rad_ext_pos:?}"
        );

        let bad_seq_base = r#"<mujoco><compiler angle="radian" eulerseq="xYz"/><worldbody><body name="end_link"><site name="tcp" pos="0 0 0" euler="0.1 0.2 0.3"/></body></worldbody></mujoco>"#;
        assert!(compose_mjcf(bad_seq_base, None, Some(&deg_manifest), None).unwrap_err().contains("eulerseq"));

        // 13. R5 M7: When base MJCF has BOTH <body name="end_link"> and another TCP-hosting body <body name="gripper">,
        // both attach_body="" and attach_body="end_link" unify on the TCP-hosting body (`gripper`).
        let dual_body_base = r#"<mujoco><worldbody>
          <body name="end_link">
            <body name="gripper"><site name="tcp" pos="0 0 0"/></body>
          </body>
        </worldbody></mujoco>"#;
        for attach_val in ["", "end_link"] {
            let m: ToolManifest = serde_json::from_value(serde_json::json!({
                "id": "dual_body_tool",
                "kind": "suction",
                "robots": ["b601dm"],
                "attach_body": attach_val,
                "tcp_offset": [0.01, 0.0, 0.0]
            })).unwrap();
            let composed = compose_mjcf(dual_body_base, Some(prop_scene), Some(&m), Some(so101_geom_tool)).unwrap();
            let doc = roxmltree::Document::parse(&composed).unwrap();
            let pad = doc.descendants().find(|n| n.attribute("name") == Some("custom_pad")).unwrap();
            assert_eq!(pad.parent().and_then(|p| p.attribute("name")), Some("gripper"), "attach_body={attach_val:?}");
            let w = doc.descendants().find(|n| n.attribute("name") == Some("suction_weld_0_chip")).unwrap();
            assert_eq!(w.attribute("body1"), Some("gripper"), "attach_body={attach_val:?}");
        }

        // 14. R6 M3: Base <site name="tcp"> nested inside <frame> fails closed instead of silently losing
        // the <frame> transform on composed tool geometries.
        let framed_tcp_base = r#"<mujoco><worldbody>
          <body name="end_link">
            <frame pos="0 0 0.10"><site name="tcp" pos="0 0 0"/></frame>
          </body>
        </worldbody></mujoco>"#;
        let framed_manifest: ToolManifest = serde_json::from_value(serde_json::json!({
            "id": "framed_tool",
            "kind": "suction",
            "robots": ["b601dm"],
            "tcp_offset": [0.02, 0.0, 0.0]
        })).unwrap();
        for tool_xml in [None, Some(so101_geom_tool)] {
            let err = compose_mjcf(framed_tcp_base, None, Some(&framed_manifest), tool_xml).unwrap_err();
            assert!(err.contains("frame"), "expected <frame> rejection, got: {err}");
        }
    }
}
