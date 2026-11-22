//! Integration tests for sandbox builtin scene/tool catalog alignment, asset validation,
//! symlink/path containment, reserved ID conflict handling, atomic active+MJCF snapshots,
//! and robot/profile/policy/sim-upstream compatibility in `activate` & `bundle`.

use std::path::Path;
use std::sync::{Arc, Mutex};

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::routing::post;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tower::ServiceExt;

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::{
    is_reserved_builtin_id, load_builtin_scene_in_roots, load_builtin_tool_in_roots, SandboxState,
};
use armd::{ArmConfig, ArmSupervisor, OperatorAuth};

static SIM_ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
static TEST_UPLOAD_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

fn write_scene_pack(dir: &Path, id: &str, robots: &[&str], default_tool: &str, extra_xml: &str) {
    std::fs::create_dir_all(dir).unwrap();
    let scene_json = json!({
        "id": id,
        "name": format!("Scene {id}"),
        "version": "1.0.0",
        "world_xml": "world.xml",
        "robots": robots,
        "default_tool": default_tool,
        "recommended_tools": [default_tool, "default"],
        "assets": []
    });
    std::fs::write(
        dir.join("scene.json"),
        serde_json::to_string_pretty(&scene_json).unwrap(),
    )
    .unwrap();
    let world_xml = format!(
        r#"<mujoco model="{id}_world"><worldbody>{extra_xml}</worldbody></mujoco>"#
    );
    std::fs::write(dir.join("world.xml"), world_xml).unwrap();
}

fn write_tool_pack(dir: &Path, id: &str, kind: &str, robots: &[&str], extra_xml: &str) {
    std::fs::create_dir_all(dir).unwrap();
    let tool_json = json!({
        "id": id,
        "name": format!("Tool {id}"),
        "kind": kind,
        "version": "1.0.0",
        "tool_xml": "tool.xml",
        "robots": robots,
        "tcp_offset": [0.0, 0.0, 0.0],
        "assets": []
    });
    std::fs::write(
        dir.join("tool.json"),
        serde_json::to_string_pretty(&tool_json).unwrap(),
    )
    .unwrap();
    let tool_xml = format!(
        r#"<mujoco model="{id}_tool"><worldbody>{extra_xml}</worldbody></mujoco>"#
    );
    std::fs::write(dir.join("tool.xml"), tool_xml).unwrap();
}

/// Self-contained builtin root fixture with all reserved scenes and tools.
fn create_builtin_fixture_root() -> tempfile::TempDir {
    let root = tempfile::tempdir().unwrap();
    let scenes = root.path().join("scenes");
    let tools = root.path().join("tools");

    write_scene_pack(
        &scenes.join("coffee_kiosk"),
        "coffee_kiosk",
        &["b601dm"],
        "default",
        r#"<body name="cup" pos="0.27 -0.10 0.035"><freejoint/><geom type="cylinder" size="0.03 0.04"/></body>"#,
    );
    write_scene_pack(
        &scenes.join("pcb_suction"),
        "pcb_suction",
        &["b601dm"],
        "suction",
        r#"<body name="pcb" pos="0.23 -0.14 0.011"><freejoint/><geom type="box" size="0.03 0.02 0.003"/></body>"#,
    );
    write_scene_pack(
        &scenes.join("pcb_inspection"),
        "pcb_inspection",
        &["b601dm"],
        "suction",
        r#"<body name="aoi_pcb" pos="0.25 0.0 0.02"><freejoint/><geom type="box" size="0.03 0.02 0.003"/></body>"#,
    );
    write_scene_pack(
        &scenes.join("screw_fastening"),
        "screw_fastening",
        &["b601dm"],
        "screwdriver",
        r#"<body name="carrier" pos="0.25 0.0 0.02"><geom type="box" size="0.04 0.04 0.01"/></body>"#,
    );
    write_scene_pack(
        &scenes.join("lab_desktop"),
        "lab_desktop",
        &["so-arm101"],
        "default",
        r#"<geom name="feeder_tray" type="box" pos="0.22 -0.05 0.004" size="0.045 0.045 0.004"/>"#,
    );

    write_tool_pack(
        &tools.join("wide_cup_gripper"),
        "wide_cup_gripper",
        "gripper",
        &["b601dm"],
        r#"<body name="cup_jaw_left" pos="0 0.02 0"><geom name="cup_pad_left" type="box" size="0.01 0.01 0.02"/></body>"#,
    );
    write_tool_pack(
        &tools.join("suction"),
        "suction",
        "suction",
        &["b601dm"],
        r#"<geom name="suction_manifold" type="box" pos="-0.08 0 0" size="0.018 0.032 0.032"/>"#,
    );
    write_tool_pack(
        &tools.join("screwdriver"),
        "screwdriver",
        "screwdriver",
        &["b601dm"],
        r#"<geom name="driver_housing" type="cylinder" pos="-0.085 0 0" size="0.02 0.04"/>"#,
    );

    root
}

fn so_arm101_config() -> ArmConfig {
    let mut cfg: ArmConfig =
        serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf")))
        .unwrap();
    cfg
}

fn app_for(config: ArmConfig, state: Arc<SandboxState>) -> axum::Router {
    let supervisor = Arc::new(ArmSupervisor::new(config));
    create_router_with_sandbox(supervisor, OperatorAuth::disabled(), state)
}

async fn send_json(
    app: &axum::Router,
    method: &str,
    uri: &str,
    body: Option<Value>,
) -> (StatusCode, Value) {
    let mut builder = Request::builder().method(method).uri(uri);
    let body = match body {
        Some(v) => {
            builder = builder.header("Content-Type", "application/json");
            Body::from(serde_json::to_vec(&v).unwrap())
        }
        None => Body::empty(),
    };
    let res = app.clone().oneshot(builder.body(body).unwrap()).await.unwrap();
    let status = res.status();
    let bytes = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
    let parsed = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
    (status, parsed)
}

#[tokio::test]
async fn sandbox_catalog_aligns_with_registry_and_default_virtual_entry() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (status, catalog) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(status, StatusCode::OK, "{catalog}");
    assert_eq!(catalog["robot"], "b601dm");

    let builtin_ids: Vec<&str> = catalog["builtin_scenes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert_eq!(
        builtin_ids,
        vec![
            "default",
            "coffee_kiosk",
            "pcb_suction",
            "pcb_inspection",
            "screw_fastening",
            "lab_desktop",
        ]
    );

    for id in &builtin_ids {
        assert!(is_reserved_builtin_id("scene", id), "missing reserved id {id}");
    }
    assert!(!is_reserved_builtin_id("scene", "custom_station"));

    let scenes = catalog["scenes"].as_array().unwrap();
    let find_scene = |id: &str| -> &Value {
        scenes
            .iter()
            .find(|s| s["id"] == id)
            .unwrap_or_else(|| panic!("scene {id} missing from catalog: {catalog}"))
    };

    let default_scene = find_scene("default");
    assert_eq!(default_scene["builtin"], true);
    assert_eq!(default_scene["virtual"], true);
    assert_eq!(default_scene["installed"], true);
    assert_eq!(default_scene["robots"], json!(["b601dm", "so-arm101"]));

    let expected_robots = [
        ("coffee_kiosk", json!(["b601dm"])),
        ("pcb_suction", json!(["b601dm"])),
        ("pcb_inspection", json!(["b601dm"])),
        ("screw_fastening", json!(["b601dm"])),
        ("lab_desktop", json!(["so-arm101"])),
    ];
    for (id, robots) in expected_robots {
        let entry = find_scene(id);
        assert_eq!(entry["builtin"], true, "{id}: {entry}");
        assert_eq!(entry["virtual"], false, "{id}: {entry}");
        assert_eq!(entry["robots"], robots, "{id}: {entry}");
        assert_eq!(entry["installed"], true, "{id} should be installed: {entry}");
    }
}

#[tokio::test]
async fn sandbox_catalog_reports_uninstalled_and_blocks_fake_activation() {
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );

    // Check under B601-DM: coffee_kiosk/pcb_inspection/screw_fastening/lab_desktop and wide_cup_gripper are not installed
    let app_b601 = app_for(ArmConfig::rebot_dm(), state.clone());
    let (status, catalog) = send_json(&app_b601, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(status, StatusCode::OK);
    let scenes = catalog["scenes"].as_array().unwrap();
    for id in ["coffee_kiosk", "pcb_inspection", "screw_fastening", "lab_desktop"] {
        let entry = scenes.iter().find(|s| s["id"] == id).unwrap();
        assert_eq!(entry["installed"], false, "{id} must not be faked as installed: {entry}");
    }
    let default_entry = scenes.iter().find(|s| s["id"] == "default").unwrap();
    assert_eq!(default_entry["virtual"], true);
    assert_eq!(default_entry["installed"], true);

    // B4: wide_cup_gripper has no compiled preset -> installed: false and activate -> 404 TOOL_NOT_INSTALLED;
    // suction/screwdriver/default have compiled presets -> installed: true
    let tools = catalog["tools"].as_array().unwrap();
    let find_tool = |id: &str| tools.iter().find(|t| t["id"] == id).unwrap();
    assert_eq!(find_tool("default")["installed"], true);
    assert_eq!(find_tool("suction")["installed"], true);
    assert_eq!(find_tool("screwdriver")["installed"], true);
    assert_eq!(find_tool("wide_cup_gripper")["installed"], false);

    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "wide_cup_gripper"})),
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND, "{err}");
    assert_eq!(err["code"], "TOOL_NOT_INSTALLED", "{err}");

    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "coffee_kiosk", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND, "{err}");
    assert_eq!(err["code"], "SCENE_NOT_INSTALLED", "{err}");

    // Check under SO-ARM101: uninstalled lab_desktop also refuses activation
    let app_so101 = app_for(so_arm101_config(), state);
    let (st, err) = send_json(
        &app_so101,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND, "{err}");
    assert_eq!(err["code"], "SCENE_NOT_INSTALLED", "{err}");
}

#[tokio::test]
async fn sandbox_user_dir_conflict_with_reserved_builtin_id_is_flagged_and_rejected() {
    let _upload_guard = TEST_UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let user_conflict_dir = tmp.path().join("scenes/lab_desktop");
    std::fs::create_dir_all(&user_conflict_dir).unwrap();
    std::fs::write(
        user_conflict_dir.join("scene.json"),
        r#"{"id":"lab_desktop","name":"Legacy User Pack","world_xml":"world.xml","robots":["so-arm101"]}"#,
    )
    .unwrap();
    std::fs::write(
        user_conflict_dir.join("world.xml"),
        r#"<mujoco model="legacy"><worldbody/></mujoco>"#,
    )
    .unwrap();

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    assert!(
        state.resolve_scene("lab_desktop").is_none(),
        "resolve_scene must refuse to silently override conflicting user pack"
    );

    let app = app_for(so_arm101_config(), state);

    let (st, cat) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(st, StatusCode::OK);
    let lab = cat["scenes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|s| s["id"] == "lab_desktop")
        .unwrap();
    assert_eq!(lab["conflict"], true, "{lab}");
    assert!(lab["conflict_reason"].as_str().unwrap().contains("lab_desktop"));
    assert_eq!(cat["conflicts"].as_array().unwrap().len(), 1, "{cat}");

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/scenes/lab_desktop", None).await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");

    // Uploading under any newly reserved builtin ID must also be refused
    for reserved in ["pcb_inspection", "screw_fastening", "lab_desktop"] {
        let (st, _) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": reserved,
                "scene_json": {"id": reserved, "name": "Hijack", "world_xml": "world.xml"},
                "world_xml": "<mujoco><worldbody/></mujoco>"
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "upload {reserved} must be rejected");
    }
}

#[tokio::test]
async fn sandbox_b3_user_tool_dir_conflict_flags_catalog_and_returns_409_everywhere() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let user_tool_dir = tmp.path().join("tools/suction");
    write_tool_pack(&user_tool_dir, "suction", "suction", &["b601dm"], "");

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    assert!(
        state.resolve_tool("suction").is_none(),
        "resolve_tool must refuse to silently fall back to user or builtin tool on conflict"
    );
    assert_eq!(
        state.resolve_tool_checked("suction").unwrap_err().0,
        StatusCode::CONFLICT
    );

    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, cat) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(st, StatusCode::OK);
    let suction = cat["tools"]
        .as_array()
        .unwrap()
        .iter()
        .find(|t| t["id"] == "suction")
        .unwrap();
    assert_eq!(suction["conflict"], true, "{suction}");
    assert_eq!(suction["installed"], false, "{suction}");
    assert!(cat["conflicts"]
        .as_array()
        .unwrap()
        .iter()
        .any(|c| c["kind"] == "tool" && c["id"] == "suction"));

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/tools/suction", None).await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/tools/suction/files/tool.xml", None).await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");

    state.active.write().await.active_tool = "suction".to_string();
    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "BUILTIN_ID_CONFLICT", "{err}");
}

#[tokio::test]
async fn sandbox_b601_positive_coffee_kiosk_and_pcb_suction() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state);

    // 1. pcb_suction + suction
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["robot"], "b601dm");
    assert_eq!(act["active_scene"], "pcb_suction");

    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    assert_eq!(bundle["robot"], "b601dm");
    assert_eq!(bundle["base_assets"], json!([]));
    assert_eq!(bundle["missing_base_assets"], json!([]));
    let xml = bundle["composed_xml"].as_str().unwrap();
    assert!(xml.contains("rebot_b601dm_scene"));
    assert!(xml.contains("suction_weld_0_pcb"));
    assert!(!xml.contains("so_arm101_scene"));

    // 2. coffee_kiosk + wide_cup_gripper
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "coffee_kiosk", "tool": "wide_cup_gripper"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["active_scene"], "coffee_kiosk");
    assert_eq!(act["active_tool"], "wide_cup_gripper");

    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    let xml = bundle["composed_xml"].as_str().unwrap();
    assert!(xml.contains("rebot_b601dm_scene"));
    assert!(xml.contains("cup_jaw_left"));
    assert!(!xml.contains("so_arm101_scene"));
}

#[tokio::test]
async fn sandbox_so_arm101_positive_lab_desktop_uses_so101_base_xml_and_exposes_base_meshes() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(so_arm101_config(), state.clone());

    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({
            "mode": "wasm",
            "scene": "lab_desktop",
            "tool": "default",
            "robot": "so-arm101"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["robot"], "so-arm101");
    assert_eq!(act["active_scene"], "lab_desktop");

    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle?robot=so-arm101", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    assert_eq!(bundle["robot"], "so-arm101");
    let xml = bundle["composed_xml"].as_str().unwrap();
    assert!(xml.contains("so_arm101_scene"), "must select SO-ARM101 base XML");
    assert!(xml.contains("shoulder_pan"));
    assert!(xml.contains("feeder_tray"));
    assert!(!xml.contains("rebot_b601dm_scene"), "must never fall back to B601 XML");

    // M06: bundle exposes the 13 STL base meshes with valid /api/v1/sandbox/base-assets/ URLs & memfs_path
    assert_eq!(bundle["base_meshdir"], "../../../assets/so101");
    let base_assets = bundle["base_assets"].as_array().unwrap();
    assert_eq!(base_assets.len(), 13, "{bundle}");
    let missing_base_assets = bundle["missing_base_assets"].as_array().unwrap();
    assert!(missing_base_assets.is_empty(), "expected no missing base assets: {bundle}");
    assert!(bundle["base_assets_note"].is_null(), "{bundle}");
    let base_entry = base_assets
        .iter()
        .find(|a| a["file"] == "base_so101_v2.stl")
        .expect("base_so101_v2.stl must be listed");
    assert_eq!(base_entry["http_accessible"], true, "{base_entry}");
    assert_eq!(base_entry["local_exists"], true, "{base_entry}");
    assert_eq!(base_entry["memfs_path"], "../../../assets/so101/base_so101_v2.stl");
    let stl_url = base_entry["url"].as_str().unwrap();
    assert_eq!(stl_url, "/api/v1/sandbox/base-assets/so101/base_so101_v2.stl");

    let stl_res = app
        .clone()
        .oneshot(Request::builder().uri(stl_url).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(stl_res.status(), StatusCode::OK);
    let stl_bytes = axum::body::to_bytes(stl_res.into_body(), usize::MAX).await.unwrap();
    assert!(!stl_bytes.is_empty(), "served STL must be non-empty");

    // Base assets endpoint rejects traversal & non-mesh files
    for bad_uri in [
        "/api/v1/sandbox/base-assets/../../Cargo.toml",
        "/api/v1/sandbox/base-assets/so101_new_calib.urdf",
    ] {
        let bad_res = app
            .clone()
            .oneshot(Request::builder().uri(bad_uri).body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(bad_res.status(), StatusCode::NOT_FOUND, "{bad_uri}");
    }

    // Cold bundle (empty cached_mjcf, e.g. after daemon restart) must also compose SO-ARM101 XML
    state.active.write().await.cached_mjcf.clear();
    let (st, cold_bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{cold_bundle}");
    let cold_xml = cold_bundle["composed_xml"].as_str().unwrap();
    assert!(cold_xml.contains("so_arm101_scene"));
    assert!(cold_xml.contains("feeder_tray"));
    assert!(!cold_xml.contains("rebot_b601dm_scene"));
}

#[tokio::test]
async fn sandbox_robot_incompatible_rejects_with_400_and_never_falls_back_to_b601() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app_b601 = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. B601 requests lab_desktop -> 400 ROBOT_INCOMPATIBLE
    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 2. B601 tries to spoof requested robot="so-arm101" for lab_desktop -> 400 ROBOT_INCOMPATIBLE
    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default", "robot": "so-arm101"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // B10: requested robot mismatch MUST fail 400 ROBOT_INCOMPATIBLE even before scene/tool lookup
    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "nonexistent_scene", "tool": "default", "robot": "so-arm101"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 3. Persisted/active state set to lab_desktop under B601 -> bundle must 400 ROBOT_INCOMPATIBLE
    state.active.write().await.active_scene = "lab_desktop".to_string();
    state.active.write().await.cached_mjcf.clear();
    let (st, err) = send_json(&app_b601, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // B10: bundle with mismatched ?profile=so-arm101 rejects 400 ROBOT_INCOMPATIBLE even when active_scene is missing
    state.active.write().await.active_scene = "nonexistent_scene".to_string();
    let (st, err) = send_json(&app_b601, "GET", "/api/v1/sandbox/bundle?profile=so-arm101", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 4. Compatible default scene, but bundle explicitly asks for mismatched ?profile=so-arm101 -> 400 ROBOT_INCOMPATIBLE
    state.active.write().await.active_scene = "default".to_string();
    let (st, err) = send_json(&app_b601, "GET", "/api/v1/sandbox/bundle?profile=so-arm101", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 5. SO-ARM101 requests B601-only scene (pcb_suction) -> 400 ROBOT_INCOMPATIBLE
    let app_so101 = app_for(so_arm101_config(), state.clone());
    let (st, err) = send_json(
        &app_so101,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 6. SO-ARM101 bundle with mismatched persisted B601 scene -> 400 ROBOT_INCOMPATIBLE
    state.active.write().await.active_scene = "pcb_suction".to_string();
    let (st, err) = send_json(&app_so101, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
}

#[cfg(unix)]
#[tokio::test]
async fn sandbox_b1_rejects_traversal_and_symlink_escapes_in_scenes_and_tools() {
    let outside = tempfile::tempdir().unwrap();
    let secret_xml = outside.path().join("secret.xml");
    std::fs::write(&secret_xml, r#"<mujoco model="evil"><worldbody/></mujoco>"#).unwrap();
    let secret_asset = outside.path().join("secret.stl");
    std::fs::write(&secret_asset, "solid secret").unwrap();

    // 1. User scene with symlinked world.xml escaping pack dir
    let tmp = tempfile::tempdir().unwrap();
    let user_scene = tmp.path().join("scenes/symlink_scene");
    std::fs::create_dir_all(&user_scene).unwrap();
    std::fs::write(
        user_scene.join("scene.json"),
        r#"{"id":"symlink_scene","name":"Evil","world_xml":"world.xml"}"#,
    )
    .unwrap();
    std::os::unix::fs::symlink(&secret_xml, user_scene.join("world.xml")).unwrap();

    // 2. User scene with .. in world_xml
    let dotdot_scene = tmp.path().join("scenes/dotdot_scene");
    std::fs::create_dir_all(&dotdot_scene).unwrap();
    std::fs::write(
        dotdot_scene.join("scene.json"),
        r#"{"id":"dotdot_scene","name":"DotDot","world_xml":"../secret.xml"}"#,
    )
    .unwrap();

    // 3. User tool with symlinked tool.xml escaping pack dir
    let user_tool = tmp.path().join("tools/symlink_tool");
    std::fs::create_dir_all(&user_tool).unwrap();
    std::fs::write(
        user_tool.join("tool.json"),
        r#"{"id":"symlink_tool","name":"Evil Tool","kind":"gripper","tool_xml":"tool.xml"}"#,
    )
    .unwrap();
    std::os::unix::fs::symlink(&secret_xml, user_tool.join("tool.xml")).unwrap();

    // 4. Builtin root with symlinked world.xml & symlinked asset
    let bad_builtins = tempfile::tempdir().unwrap();
    let b_scene = bad_builtins.path().join("scenes/coffee_kiosk");
    std::fs::create_dir_all(&b_scene).unwrap();
    std::fs::write(
        b_scene.join("scene.json"),
        r#"{"id":"coffee_kiosk","name":"Coffee","world_xml":"world.xml","assets":["escape.stl"]}"#,
    )
    .unwrap();
    std::os::unix::fs::symlink(&secret_xml, b_scene.join("world.xml")).unwrap();
    std::os::unix::fs::symlink(&secret_asset, b_scene.join("escape.stl")).unwrap();

    let b_tool = bad_builtins.path().join("tools/wide_cup_gripper");
    std::fs::create_dir_all(&b_tool).unwrap();
    std::fs::write(
        b_tool.join("tool.json"),
        r#"{"id":"wide_cup_gripper","name":"Cup","kind":"gripper","tool_xml":"../secret.xml"}"#,
    )
    .unwrap();

    assert!(load_builtin_scene_in_roots("coffee_kiosk", &[bad_builtins.path().to_path_buf()]).is_none());
    assert!(load_builtin_tool_in_roots("wide_cup_gripper", &[bad_builtins.path().to_path_buf()]).is_none());

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![bad_builtins.path().to_path_buf()]),
    );
    assert!(state.resolve_scene("symlink_scene").is_none());
    assert!(state.resolve_scene("dotdot_scene").is_none());
    assert!(state.resolve_tool("symlink_tool").is_none());

    assert_eq!(
        state.resolve_scene_checked("symlink_scene").unwrap_err().0,
        StatusCode::BAD_REQUEST
    );
    assert_eq!(
        state.resolve_scene_checked("dotdot_scene").unwrap_err().0,
        StatusCode::BAD_REQUEST
    );
    assert_eq!(
        state.resolve_tool_checked("symlink_tool").unwrap_err().0,
        StatusCode::BAD_REQUEST
    );
}

#[tokio::test]
async fn sandbox_b2_builtin_scene_clamps_authoritative_id_and_robots() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = tempfile::tempdir().unwrap();
    let lab_dir = builtins.path().join("scenes/lab_desktop");
    std::fs::create_dir_all(&lab_dir).unwrap();
    // Tampered disk manifest tries to hijack id and widen robots to b601dm
    std::fs::write(
        lab_dir.join("scene.json"),
        serde_json::to_string_pretty(&json!({
            "id": "hijacked_id",
            "name": "Lab Desktop",
            "world_xml": "world.xml",
            "robots": ["b601dm", "so-arm101"]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(
        lab_dir.join("world.xml"),
        r#"<mujoco model="lab"><worldbody/></mujoco>"#,
    )
    .unwrap();

    let (manifest, _) =
        load_builtin_scene_in_roots("lab_desktop", &[builtins.path().to_path_buf()]).unwrap();
    assert_eq!(manifest.id, "lab_desktop");
    assert_eq!(manifest.robots, vec!["so-arm101".to_string()]);

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app_b601 = app_for(ArmConfig::rebot_dm(), state);
    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
}

#[tokio::test]
async fn sandbox_b5_declared_assets_must_exist_for_catalog_and_activation() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();

    // Builtin scene coffee_kiosk declares an asset that is missing on disk
    let ck_dir = builtins.path().join("scenes/coffee_kiosk");
    std::fs::write(
        ck_dir.join("scene.json"),
        serde_json::to_string_pretty(&json!({
            "id": "coffee_kiosk",
            "name": "Coffee Kiosk",
            "world_xml": "world.xml",
            "robots": ["b601dm"],
            "assets": [{"path": "meshes/cup.stl"}]
        }))
        .unwrap(),
    )
    .unwrap();

    // Custom user scene declares an asset that is missing on disk
    let custom_dir = tmp.path().join("scenes/custom_mesh_scene");
    std::fs::create_dir_all(&custom_dir).unwrap();
    std::fs::write(
        custom_dir.join("scene.json"),
        serde_json::to_string_pretty(&json!({
            "id": "custom_mesh_scene",
            "name": "Custom Mesh",
            "world_xml": "world.xml",
            "robots": ["b601dm"],
            "assets": [{"path": "meshes/part.stl"}]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(
        custom_dir.join("world.xml"),
        r#"<mujoco model="custom"><worldbody/></mujoco>"#,
    )
    .unwrap();

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, cat) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(st, StatusCode::OK);
    let scenes = cat["scenes"].as_array().unwrap();
    assert_eq!(
        scenes.iter().find(|s| s["id"] == "coffee_kiosk").unwrap()["installed"],
        false
    );
    assert_eq!(
        scenes.iter().find(|s| s["id"] == "custom_mesh_scene").unwrap()["installed"],
        false
    );

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "coffee_kiosk", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND, "{err}");
    assert_eq!(err["code"], "SCENE_NOT_INSTALLED", "{err}");

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "custom_mesh_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND, "{err}");

    // Once the declared asset file is created inside the pack, catalog marks installed and activate succeeds
    std::fs::create_dir_all(custom_dir.join("meshes")).unwrap();
    std::fs::write(custom_dir.join("meshes/part.stl"), "solid part\nendsolid part\n").unwrap();

    let (_, cat2) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(
        cat2["scenes"]
            .as_array()
            .unwrap()
            .iter()
            .find(|s| s["id"] == "custom_mesh_scene")
            .unwrap()["installed"],
        true
    );
    let (st, ok) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "custom_mesh_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{ok}");
}

#[tokio::test]
async fn sandbox_b7_sim_mode_sends_robot_and_rejects_mismatched_upstream_robot() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let seen_requests: Arc<Mutex<Vec<Value>>> = Arc::new(Mutex::new(Vec::new()));
    let seen_c = seen_requests.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            axum::routing::get(|| async { axum::Json(json!({"robot": "b601dm"})) }),
        )
        .route(
            "/sim/scene",
            post(move |axum::Json(body): axum::Json<Value>| {
                let seen_c = seen_c.clone();
                async move {
                    seen_c.lock().unwrap().push(body);
                    // Upstream responds with mismatched b601dm while running_robot is so-arm101
                    axum::Json(json!({"ok": true, "robot": "b601dm"}))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        axum::serve(listener, mock_sim).await.unwrap();
    });

    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(so_arm101_config(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    {
        let reqs = seen_requests.lock().unwrap();
        assert_eq!(reqs.len(), 0, "legacy POST /sim/scene must not be called when /sim/state mismatches robot");
    }

    // Active state must NOT have been mutated on upstream mismatch
    let st_after = state.active.read().await;
    assert_eq!(st_after.mode, "none");
    assert_eq!(st_after.active_scene, "default");
}

#[tokio::test]
async fn sandbox_b8_policy_robot_compatibility_enforced_in_activate_and_bundle() {
    let _upload_guard = TEST_UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app_so101 = app_for(so_arm101_config(), state.clone());

    // Upload a B601-DM policy
    let (st, up) = send_json(
        &app_so101,
        "POST",
        "/api/v1/sandbox/policies",
        Some(json!({
            "id": "b601_only_pol",
            "policy_json": {
                "id": "b601_only_pol",
                "format": "rebot-onnx-policy/v1",
                "robot": "b601dm",
                "action_dim": 7,
                "inputs": {},
                "outputs": {},
                "model_file": "model.onnx"
            },
            "model_onnx_base64": "AQID"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{up}");

    // 1. activate under SO-ARM101 with B601 policy -> 400 ROBOT_INCOMPATIBLE
    let (st, err) = send_json(
        &app_so101,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({
            "mode": "wasm",
            "scene": "lab_desktop",
            "tool": "default",
            "policy": "b601_only_pol"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    // 2. persisted active_policy set to B601 policy -> bundle under SO-ARM101 -> 400 ROBOT_INCOMPATIBLE
    {
        let mut st_w = state.active.write().await;
        st_w.active_scene = "lab_desktop".to_string();
        st_w.active_policy = Some("b601_only_pol".to_string());
    }
    let (st, err) = send_json(&app_so101, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
}

#[tokio::test]
async fn sandbox_b9_concurrent_activate_and_bundle_never_observe_torn_mjcf_snapshot() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state);

    let mut tasks = Vec::new();
    for i in 0..20 {
        let app_a = app.clone();
        tasks.push(tokio::spawn(async move {
            let payload = if i % 2 == 0 {
                json!({"mode": "wasm", "scene": "pcb_suction", "tool": "suction"})
            } else {
                json!({"mode": "wasm", "scene": "default", "tool": "default"})
            };
            let (st, _) = send_json(&app_a, "POST", "/api/v1/sandbox/activate", Some(payload)).await;
            assert_eq!(st, StatusCode::OK);
        }));

        let app_b = app.clone();
        tasks.push(tokio::spawn(async move {
            let (st, bundle) = send_json(&app_b, "GET", "/api/v1/sandbox/bundle", None).await;
            assert_eq!(st, StatusCode::OK, "{bundle}");
            let scene = bundle["active_scene"].as_str().unwrap();
            let tool = bundle["active_tool"].as_str().unwrap();
            let xml = bundle["composed_xml"].as_str().unwrap();
            match (scene, tool) {
                ("pcb_suction", "suction") => {
                    assert!(
                        xml.contains("suction_weld_0_pcb") && xml.contains("suction_manifold"),
                        "torn snapshot: pcb_suction/suction paired with wrong XML"
                    );
                }
                ("default", "default") => {
                    assert!(
                        !xml.contains("suction_weld_0_pcb") && !xml.contains("suction_manifold"),
                        "torn snapshot: default/default paired with pcb_suction XML"
                    );
                }
                other => panic!("unexpected scene/tool pair in snapshot: {other:?}"),
            }
        }));
    }

    for t in tasks {
        t.await.unwrap();
    }
}

#[tokio::test]
async fn sandbox_b11_b12_serve_skips_incomplete_roots_and_get_scene_distinguishes_virtual_and_preset() {
    let tmp = tempfile::tempdir().unwrap();
    let empty_or_broken_root = tempfile::tempdir().unwrap();
    // Create an empty/incomplete coffee_kiosk and wide_cup_gripper dir in root 0
    std::fs::create_dir_all(empty_or_broken_root.path().join("scenes/coffee_kiosk")).unwrap();
    std::fs::create_dir_all(empty_or_broken_root.path().join("tools/wide_cup_gripper")).unwrap();

    let valid_root = create_builtin_fixture_root();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()).with_builtin_roots(vec![
        empty_or_broken_root.path().to_path_buf(),
        valid_root.path().to_path_buf(),
    ]));
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // B11: serve_scene_file & serve_tool_file skip broken root 0 and serve from valid_root
    let res = app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/api/v1/sandbox/scenes/coffee_kiosk/files/world.xml")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::OK);

    let res = app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/api/v1/sandbox/tools/wide_cup_gripper/files/tool.xml")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::OK);

    // B12: virtual default returns files: [] & virtual: true; disk lab_desktop returns actual files
    let (st, def) = send_json(&app, "GET", "/api/v1/sandbox/scenes/default", None).await;
    assert_eq!(st, StatusCode::OK, "{def}");
    assert_eq!(def["files"], json!([]));
    assert_eq!(def["virtual"], true);
    assert_eq!(def["compiled_preset"], false);
    assert!(def["path"].is_null());

    let (st, lab) = send_json(&app, "GET", "/api/v1/sandbox/scenes/lab_desktop", None).await;
    assert_eq!(st, StatusCode::OK, "{lab}");
    assert_eq!(lab["files"], json!(["scene.json", "world.xml"]));
    assert_eq!(lab["virtual"], false);
    assert_eq!(lab["compiled_preset"], false);
    assert!(lab["path"].as_str().unwrap().contains("lab_desktop"));

    // B12: compiled preset pcb_suction with no disk directory returns files: [] & compiled_preset: true
    let empty_builtins = tempfile::tempdir().unwrap();
    let state_preset_only = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app_preset = app_for(ArmConfig::rebot_dm(), state_preset_only);
    let (st, pcb) = send_json(&app_preset, "GET", "/api/v1/sandbox/scenes/pcb_suction", None).await;
    assert_eq!(st, StatusCode::OK, "{pcb}");
    assert_eq!(pcb["files"], json!([]));
    assert_eq!(pcb["virtual"], false);
    assert_eq!(pcb["compiled_preset"], true);
    assert!(pcb["path"].is_null());
}

#[cfg(unix)]
#[tokio::test]
async fn sandbox_m01_rejects_cross_category_symlink_in_file_serving() {
    let tmp = tempfile::tempdir().unwrap();
    let tools_dir = tmp.path().join("tools/secret_tool");
    write_tool_pack(&tools_dir, "secret_tool", "gripper", &["b601dm"], "");
    std::fs::write(tools_dir.join("secret.txt"), "tool-secret").unwrap();

    let scenes_dir = tmp.path().join("scenes");
    std::fs::create_dir_all(&scenes_dir).unwrap();
    std::os::unix::fs::symlink(&tools_dir, scenes_dir.join("cross_scene")).unwrap();

    let custom_scene = tmp.path().join("scenes/legit_scene");
    write_scene_pack(&custom_scene, "legit_scene", &["b601dm"], "default", "");
    std::os::unix::fs::symlink(&custom_scene, tmp.path().join("tools/cross_tool")).unwrap();

    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let res = app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/api/v1/sandbox/scenes/cross_scene/files/secret.txt")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::NOT_FOUND, "scene serving must enforce scenes/ root");

    let res = app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/api/v1/sandbox/tools/cross_tool/files/world.xml")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::NOT_FOUND, "tool serving must enforce tools/ root");
}

#[cfg(unix)]
#[tokio::test]
async fn sandbox_m02_collect_pack_files_terminates_on_ancestor_symlink_cycle() {
    let tmp = tempfile::tempdir().unwrap();
    let scene_dir = tmp.path().join("scenes/cycle_scene");
    write_scene_pack(&scene_dir, "cycle_scene", &["b601dm"], "default", "");
    std::fs::create_dir_all(scene_dir.join("sub")).unwrap();
    std::fs::write(scene_dir.join("sub/mesh.stl"), "solid m\nendsolid m\n").unwrap();
    // Symlink inside sub pointing back to pack root (inside pack boundary -> ancestor cycle!)
    std::os::unix::fs::symlink(&scene_dir, scene_dir.join("sub/loop")).unwrap();

    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, detail) = send_json(&app, "GET", "/api/v1/sandbox/scenes/cycle_scene", None).await;
    assert_eq!(st, StatusCode::OK, "{detail}");
    assert_eq!(detail["files"], json!(["scene.json", "sub/mesh.stl", "world.xml"]));
}

#[tokio::test]
async fn sandbox_m03_builtin_empty_robot_intersection_is_rejected() {
    let tmp = tempfile::tempdir().unwrap();
    let builtins = tempfile::tempdir().unwrap();
    let lab_dir = builtins.path().join("scenes/lab_desktop");
    std::fs::create_dir_all(&lab_dir).unwrap();
    // Registry robots for lab_desktop is ["so-arm101"], disk manifest specifies ["b601dm"] -> empty intersection!
    std::fs::write(
        lab_dir.join("scene.json"),
        serde_json::to_string_pretty(&json!({
            "id": "lab_desktop",
            "name": "Lab Desktop",
            "world_xml": "world.xml",
            "robots": ["b601dm"]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(lab_dir.join("world.xml"), r#"<mujoco model="lab"><worldbody/></mujoco>"#).unwrap();

    let (manifest, _) =
        load_builtin_scene_in_roots("lab_desktop", &[builtins.path().to_path_buf()]).unwrap();
    assert!(manifest.robots.is_empty(), "empty intersection must NOT fall back to registry set: {:?}", manifest.robots);

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );

    // Rejected for SO-ARM101 AND for B601-DM
    let app_so101 = app_for(so_arm101_config(), state.clone());
    let (st, err) = send_json(
        &app_so101,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");

    let app_b601 = app_for(ArmConfig::rebot_dm(), state);
    let (st, err) = send_json(
        &app_b601,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "lab_desktop", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
}

#[tokio::test]
async fn sandbox_m04_m05_optional_assets_and_normalized_path_in_bundle() {
    let tmp = tempfile::tempdir().unwrap();
    let scene_dir = tmp.path().join("scenes/asset_norm_scene");
    std::fs::create_dir_all(scene_dir.join("meshes")).unwrap();
    std::fs::write(scene_dir.join("meshes/present.stl"), "solid present\nendsolid present\n").unwrap();
    std::fs::write(scene_dir.join("extra.bin"), "bin").unwrap();
    std::fs::write(
        scene_dir.join("scene.json"),
        serde_json::to_string_pretty(&json!({
            "id": "asset_norm_scene",
            "name": "Asset Norm Scene",
            "world_xml": "world.xml",
            "robots": ["b601dm"],
            "assets": [
                {"target": "meshes/present.stl", "required": true},
                {"filename": "extra.bin"},
                {"target": "meshes/absent_optional.stl", "required": false}
            ]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(
        scene_dir.join("world.xml"),
        r#"<mujoco model="norm"><asset><mesh file="meshes/present.stl"/></asset><worldbody/></mujoco>"#,
    )
    .unwrap();

    let tool_dir = tmp.path().join("tools/asset_norm_tool");
    std::fs::create_dir_all(&tool_dir).unwrap();
    std::fs::write(tool_dir.join("jaw.stl"), "solid jaw\nendsolid jaw\n").unwrap();
    std::fs::write(
        tool_dir.join("tool.json"),
        serde_json::to_string_pretty(&json!({
            "id": "asset_norm_tool",
            "name": "Asset Norm Tool",
            "kind": "gripper",
            "tool_xml": "tool.xml",
            "robots": ["b601dm"],
            "tcp_offset": [0.0, 0.0, 0.0],
            "assets": ["jaw.stl", {"path": "opt_missing.stl", "required": false}]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(
        tool_dir.join("tool.xml"),
        r#"<mujoco model="norm_tool"><worldbody/></mujoco>"#,
    )
    .unwrap();

    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    // M04: catalog marks installed: true even when optional asset is absent
    let (st, cat) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(st, StatusCode::OK);
    let sc = cat["scenes"].as_array().unwrap().iter().find(|s| s["id"] == "asset_norm_scene").unwrap();
    assert_eq!(sc["installed"], true, "{sc}");

    // M04: activate succeeds
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "asset_norm_scene", "tool": "asset_norm_tool"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");

    // M05: bundle normalizes target/filename/string entries to have "path" and skips absent optional assets
    // m02: custom scene <mesh file="meshes/present.stl"/> is NOT treated as missing base asset!
    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    assert_eq!(bundle["base_assets"], json!([]));
    assert_eq!(bundle["missing_base_assets"], json!([]));

    let scene_assets = bundle["scene_manifest"]["assets"].as_array().unwrap();
    assert_eq!(scene_assets.len(), 2, "{scene_assets:?}");
    assert_eq!(scene_assets[0]["path"], "meshes/present.stl");
    assert_eq!(scene_assets[1]["path"], "extra.bin");

    let tool_assets = bundle["tool_manifest"]["assets"].as_array().unwrap();
    assert_eq!(tool_assets.len(), 1, "{tool_assets:?}");
    assert_eq!(tool_assets[0]["path"], "jaw.stl");
}

#[tokio::test]
async fn sandbox_m09_same_id_overwrite_invalidates_cached_mjcf() {
    let _upload_guard = TEST_UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let upload_v1 = json!({
        "id": "live_scene",
        "scene_json": {"id": "live_scene", "name": "Live v1", "version": "1.0.0", "world_xml": "world.xml"},
        "world_xml": r#"<mujoco model="v1"><worldbody><body name="v1_marker" pos="0.2 0 0"><geom type="box" size="0.01 0.01 0.01"/></body></worldbody></mujoco>"#
    });
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/scenes", Some(upload_v1)).await;
    assert_eq!(st, StatusCode::OK);

    let upload_tool_v1 = json!({
        "id": "live_tool",
        "tool_json": {"id": "live_tool", "name": "Live Tool v1", "kind": "gripper", "version": "1.0.0", "tool_xml": "tool.xml", "tcp_offset": [0.0, 0.0, 0.0]},
        "tool_xml": r#"<mujoco model="tv1"><worldbody><geom name="tool_v1_marker" type="box" size="0.01 0.01 0.01"/></worldbody></mujoco>"#
    });
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/tools", Some(upload_tool_v1)).await;
    assert_eq!(st, StatusCode::OK);

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "live_scene", "tool": "live_tool"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let (st, b1) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(b1["scene_manifest"]["version"], "1.0.0");
    assert_eq!(b1["tool_manifest"]["version"], "1.0.0");
    assert!(b1["composed_xml"].as_str().unwrap().contains("v1_marker"));
    assert!(b1["composed_xml"].as_str().unwrap().contains("tool_v1_marker"));

    // Overwrite same scene ID and same tool ID with v2
    let upload_v2 = json!({
        "id": "live_scene",
        "scene_json": {"id": "live_scene", "name": "Live v2", "version": "2.0.0", "world_xml": "world.xml"},
        "world_xml": r#"<mujoco model="v2"><worldbody><body name="v2_marker" pos="0.2 0 0"><geom type="box" size="0.01 0.01 0.01"/></body></worldbody></mujoco>"#
    });
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/scenes", Some(upload_v2)).await;
    assert_eq!(st, StatusCode::OK);

    let upload_tool_v2 = json!({
        "id": "live_tool",
        "tool_json": {"id": "live_tool", "name": "Live Tool v2", "kind": "gripper", "version": "2.0.0", "tool_xml": "tool.xml", "tcp_offset": [0.0, 0.0, 0.0]},
        "tool_xml": r#"<mujoco model="tv2"><worldbody><geom name="tool_v2_marker" type="box" size="0.01 0.01 0.01"/></worldbody></mujoco>"#
    });
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/tools", Some(upload_tool_v2)).await;
    assert_eq!(st, StatusCode::OK);

    let (st, b2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(b2["scene_manifest"]["version"], "2.0.0");
    assert_eq!(b2["tool_manifest"]["version"], "2.0.0");
    let xml2 = b2["composed_xml"].as_str().unwrap();
    assert!(xml2.contains("v2_marker"), "bundle must re-compose scene after overwrite");
    assert!(!xml2.contains("v1_marker"), "stale v1 scene XML must not linger");
    assert!(xml2.contains("tool_v2_marker"), "bundle must re-compose tool after overwrite");
    assert!(!xml2.contains("tool_v1_marker"), "stale v1 tool XML must not linger");
}

#[tokio::test]
async fn sandbox_m10_m12_stale_policy_rejected_and_policy_null_clears() {
    let _upload_guard = TEST_UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let builtins = create_builtin_fixture_root();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // Upload a policy for so-arm101 (incompatible with b601dm)
    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/policies",
        Some(json!({
            "id": "so_pol",
            "policy_json": {
                "id": "so_pol",
                "format": "rebot-onnx-policy/v1",
                "robot": "so-arm101",
                "action_dim": 6,
                "inputs": {},
                "outputs": {},
                "model_file": "model.onnx"
            },
            "model_onnx_base64": "AQID"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    // M10: missing inherited active_policy -> activate (policy omitted) and bundle both reject!
    state.active.write().await.active_policy = Some("vanished_pol".to_string());
    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "ACTIVE_POLICY_MISSING", "{err}");

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "ACTIVE_POLICY_MISSING", "{err}");

    // M12: "policy": null explicitly clears active_policy even when old active_policy was incompatible or missing!
    state.active.write().await.active_policy = Some("so_pol".to_string());
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "default", "policy": null})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert!(act["active_policy"].is_null(), "{act}");

    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    assert!(bundle["active_policy"].is_null());
    assert!(bundle["policy_manifest"].is_null());
    assert!(bundle["policy_model_url"].is_null());
}

#[tokio::test]
async fn sandbox_m08_m11_activate_and_delete_policy_are_serialized() {
    let _upload_guard = TEST_UPLOAD_LOCK.lock().await;
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let (entered_tx, entered_rx) = tokio::sync::oneshot::channel::<()>();
    let entered_tx = Arc::new(Mutex::new(Some(entered_tx)));
    let release = Arc::new(tokio::sync::Notify::new());
    let release_c = release.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            axum::routing::get(|| async { axum::Json(json!({"robot": "b601dm"})) }),
        )
        .route(
            "/sim/scene",
            post(move || {
                let entered_tx = entered_tx.clone();
                let release_c = release_c.clone();
                async move {
                    if let Some(tx) = entered_tx.lock().unwrap().take() {
                        let _ = tx.send(());
                    }
                    release_c.notified().await;
                    axum::Json(json!({"ok": true, "robot": "b601dm"}))
                }
            }),
        );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        axum::serve(listener, mock_sim).await.unwrap();
    });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/policies",
        Some(json!({
            "id": "race_pol",
            "policy_json": {
                "id": "race_pol",
                "format": "rebot-onnx-policy/v1",
                "robot": "b601dm",
                "action_dim": 7,
                "inputs": {},
                "outputs": {},
                "model_file": "model.onnx"
            },
            "model_onnx_base64": "AQID"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let app_act = app.clone();
    let act_task = tokio::spawn(async move {
        send_json(
            &app_act,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "default", "tool": "default", "policy": "race_pol"})),
        )
        .await
    });

    entered_rx.await.unwrap();

    // Start delete_policy while activate is blocked in sim HTTP round-trip:
    // it must serialize behind activate_lock and then fail 409 POLICY_IN_USE.
    let app_del = app.clone();
    let del_task = tokio::spawn(async move {
        send_json(&app_del, "DELETE", "/api/v1/sandbox/policies/race_pol", None).await
    });

    tokio::task::yield_now().await;
    release.notify_one();

    let (act_st, act_body) = act_task.await.unwrap();
    let (del_st, del_body) = del_task.await.unwrap();
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(act_st, StatusCode::OK, "{act_body}");
    assert_eq!(del_st, StatusCode::CONFLICT, "{del_body}");
    assert_eq!(del_body["code"], "POLICY_IN_USE", "{del_body}");
    assert!(state.resolve_policy("race_pol").is_some());
}

#[tokio::test]
async fn sandbox_m01_minor_catalog_uses_canonical_dir_id() {
    let tmp = tempfile::tempdir().unwrap();
    write_scene_pack(
        &tmp.path().join("scenes/dir_scene_id"),
        "conflicting_manifest_id",
        &["b601dm"],
        "default",
        "",
    );
    write_tool_pack(
        &tmp.path().join("tools/dir_tool_id"),
        "conflicting_tool_manifest_id",
        "gripper",
        &["b601dm"],
        "",
    );

    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, cat) = send_json(&app, "GET", "/api/v1/sandbox/catalog", None).await;
    assert_eq!(st, StatusCode::OK);
    assert!(cat["scenes"].as_array().unwrap().iter().any(|s| s["id"] == "dir_scene_id"));
    assert!(!cat["scenes"].as_array().unwrap().iter().any(|s| s["id"] == "conflicting_manifest_id"));
    assert!(cat["tools"].as_array().unwrap().iter().any(|t| t["id"] == "dir_tool_id"));
    assert!(!cat["tools"].as_array().unwrap().iter().any(|t| t["id"] == "conflicting_tool_manifest_id"));
}
