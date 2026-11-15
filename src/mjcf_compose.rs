//! Pure MJCF composition and manifests.
use serde::{Deserialize, Serialize};

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

pub fn validate_xml_security(xml: &str, required_elem: Option<&str>) -> Result<(), String> {
    if xml.contains("<!DOCTYPE") || xml.contains("<!ENTITY") {
        return Err("XML DOCTYPE and ENTITY declarations are forbidden".into());
    }
    let doc = roxmltree::Document::parse(xml).map_err(|e| format!("XML parse failure: {e}"))?;
    if let Some(req) = required_elem {
        if !doc.descendants().any(|n| n.is_element() && n.tag_name().name() == req) {
            return Err(format!("Missing required <{req}> element"));
        }
    }
    Ok(())
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
