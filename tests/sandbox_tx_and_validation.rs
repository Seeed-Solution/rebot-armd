//! Integration tests for P0-2:
//! - Boundary 1 (upload) & Boundary 2 (disk reload / resolve / activate / bundle)
//!   MJCF + STL/OBJ security, format, reference-closure, and budget validation
//! - Two-phase `tx_id` sim activation state machine (`prepare -> commit -> abort -> reconcile`),
//!   `DEGRADED_SPLIT` automatic convergence / rollback, and `/api/v1/sandbox/recover-split`.

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::routing::{get, post};
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tower::ServiceExt;

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::{normalize_robot_id, ActivationTxState, SandboxState, SimTxTimeouts};
use armd::mjcf_compose::{
    compose_mjcf, compute_pack_digest, FALLBACK_BASE_XML, MAX_PACK_BODIES, MAX_PACK_GEOMS,
    MAX_XML_BYTES, MAX_XML_ELEMENTS,
};
use armd::{ArmConfig, ArmSupervisor, OperatorAuth};

static SIM_ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
static UPLOAD_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

async fn read_http_request(sock: &mut tokio::net::TcpStream) -> Option<(String, Value)> {
    let mut raw = Vec::new();
    let mut buf = [0u8; 4096];
    let head_end = loop {
        if let Some(pos) = raw.windows(4).position(|w| w == b"\r\n\r\n") {
            break pos;
        }
        let n = sock.read(&mut buf).await.ok()?;
        if n == 0 {
            return None;
        }
        raw.extend_from_slice(&buf[..n]);
    };
    let head_str = String::from_utf8_lossy(&raw[..head_end]).to_string();
    let first_line = head_str.lines().next().unwrap_or("").to_string();
    let content_len: usize = head_str
        .lines()
        .find_map(|l| {
            let (k, v) = l.split_once(':')?;
            k.trim().eq_ignore_ascii_case("content-length").then(|| v.trim().parse().ok())?
        })
        .unwrap_or(0);
    let body_start = head_end + 4;
    while raw.len() < body_start + content_len {
        let n = sock.read(&mut buf).await.ok()?;
        if n == 0 {
            break;
        }
        raw.extend_from_slice(&buf[..n]);
    }
    let body_bytes = &raw[body_start..(body_start + content_len).min(raw.len())];
    let parsed = serde_json::from_slice(body_bytes).unwrap_or(Value::Null);
    Some((first_line, parsed))
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

fn b64_encode(data: &[u8]) -> String {
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len() * 4 / 3 + 4);
    for chunk in data.chunks(3) {
        let b0 = chunk[0];
        let b1 = if chunk.len() > 1 { chunk[1] } else { 0 };
        let b2 = if chunk.len() > 2 { chunk[2] } else { 0 };
        out.push(CHARS[(b0 >> 2) as usize] as char);
        out.push(CHARS[(((b0 & 0x03) << 4) | (b1 >> 4)) as usize] as char);
        if chunk.len() > 1 {
            out.push(CHARS[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char);
        } else {
            out.push('=');
        }
        if chunk.len() > 2 {
            out.push(CHARS[(b2 & 0x3F) as usize] as char);
        } else {
            out.push('=');
        }
    }
    out
}

fn make_binary_stl(tris: &[([f32; 3], [[f32; 3]; 3])]) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for (n, verts) in tris {
        for &f in n {
            out.extend_from_slice(&f.to_le_bytes());
        }
        for v in verts {
            for &f in v {
                out.extend_from_slice(&f.to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

fn valid_asymmetric_stl() -> Vec<u8> {
    make_binary_stl(&[
        (
            [0.0, 0.0, 1.0],
            [[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.03, 0.02]],
        ),
        (
            [0.0, -1.0, 0.0],
            [[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.02, 0.0, 0.04]],
        ),
    ])
}

fn write_disk_scene(dir: &Path, id: &str, world_xml: &str, extra_files: &[(&str, &[u8])]) {
    std::fs::create_dir_all(dir).unwrap();
    let scene_json = json!({
        "id": id,
        "name": format!("Scene {id}"),
        "version": "1.0.0",
        "world_xml": "world.xml",
        "robots": ["b601dm"],
        "assets": []
    });
    std::fs::write(
        dir.join("scene.json"),
        serde_json::to_string_pretty(&scene_json).unwrap(),
    )
    .unwrap();
    std::fs::write(dir.join("world.xml"), world_xml).unwrap();
    for (rel, bytes) in extra_files {
        let p = dir.join(rel);
        if let Some(parent) = p.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(p, bytes).unwrap();
    }
}

fn write_disk_tool(dir: &Path, id: &str, tool_xml: &str, extra_files: &[(&str, &[u8])]) {
    std::fs::create_dir_all(dir).unwrap();
    let tool_json = json!({
        "id": id,
        "name": format!("Tool {id}"),
        "kind": "gripper",
        "version": "1.0.0",
        "tool_xml": "tool.xml",
        "robots": ["b601dm"],
        "tcp_offset": [0.0, 0.0, 0.0],
        "assets": []
    });
    std::fs::write(
        dir.join("tool.json"),
        serde_json::to_string_pretty(&tool_json).unwrap(),
    )
    .unwrap();
    std::fs::write(dir.join("tool.xml"), tool_xml).unwrap();
    for (rel, bytes) in extra_files {
        let p = dir.join(rel);
        if let Some(parent) = p.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(p, bytes).unwrap();
    }
}

async fn assert_boundary2_rejected(state: &SandboxState, app: &axum::Router, scene_id: &str) {
    assert!(
        state.resolve_scene_checked(scene_id).is_err(),
        "{scene_id}: resolve_scene_checked must reject"
    );
    let (st, _) = send_json(app, "GET", &format!("/api/v1/sandbox/scenes/{scene_id}"), None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{scene_id}: get_scene must reject");
}

#[tokio::test]
async fn sandbox_validation_rejects_forbidden_xml_paths_urdf_meshes_on_boundary1_and_boundary2() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let deep_xml = {
        let mut s = String::from("<mujoco><worldbody>");
        for i in 0..33 {
            s.push_str(&format!("<body name=\"b{i}\">"));
        }
        for _ in 0..33 {
            s.push_str("</body>");
        }
        s.push_str("</worldbody></mujoco>");
        s
    };
    let oversized_xml = format!(
        "<mujoco><worldbody><!-- {} --></worldbody></mujoco>",
        "a".repeat(MAX_XML_BYTES)
    );

    let xml_cases: Vec<(&str, String)> = vec![
        (
            "doctype",
            r#"<!DOCTYPE mujoco><mujoco><worldbody/></mujoco>"#.into(),
        ),
        (
            "entity",
            r#"<!DOCTYPE foo [ <!ENTITY xxe "evil"> ]><mujoco><worldbody/></mujoco>"#.into(),
        ),
        (
            "include",
            r#"<mujoco><include file="other.xml"/><worldbody/></mujoco>"#.into(),
        ),
        (
            "plugin",
            r#"<mujoco><extension><plugin plugin="evil"/></extension><worldbody/></mujoco>"#.into(),
        ),
        (
            "custom",
            r#"<mujoco><custom><numeric name="x" size="1"/></custom><worldbody/></mujoco>"#.into(),
        ),
        (
            "dotdot_file",
            r#"<mujoco><asset><mesh file="../escape.stl"/></asset><worldbody/></mujoco>"#.into(),
        ),
        (
            "abs_file",
            r#"<mujoco><asset><mesh file="/etc/passwd"/></asset><worldbody/></mujoco>"#.into(),
        ),
        (
            "dotdot_meshdir",
            r#"<mujoco><compiler meshdir="../meshes"/><worldbody/></mujoco>"#.into(),
        ),
        (
            "abs_texturedir",
            r#"<mujoco><compiler texturedir="/tmp/tex"/><worldbody/></mujoco>"#.into(),
        ),
        (
            "dotdot_assetdir",
            r#"<mujoco><compiler assetdir="sub/../../etc"/><worldbody/></mujoco>"#.into(),
        ),
        (
            "dangling_mesh",
            r#"<mujoco><asset><mesh file="meshes/missing.stl"/></asset><worldbody/></mujoco>"#.into(),
        ),
        ("xml_depth_over_32", deep_xml),
        ("xml_over_512k", oversized_xml),
    ];

    for (name, xml) in &xml_cases {
        let id = format!("bad_{name}");
        let (st, body) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": id,
                "scene_json": {"id": id, "name": name, "world_xml": "world.xml"},
                "world_xml": xml
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "Boundary 1 ({name}): {body}");
        assert_eq!(body["code"], "INVALID_PACK", "Boundary 1 ({name}): {body}");

        let disk_id = format!("disk_{name}");
        write_disk_scene(&tmp.path().join("scenes").join(&disk_id), &disk_id, xml, &[]);
        assert_boundary2_rejected(&state, &app, &disk_id).await;
        let (st2, body2) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "wasm", "scene": disk_id, "tool": "default"})),
        )
        .await;
        assert_eq!(st2, StatusCode::BAD_REQUEST, "Boundary 2 activate ({name}): {body2}");
    }

    // Also test Boundary 1 & 2 tool XML rejection (e.g. <plugin> and dangling <mesh>)
    let (st, body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/tools",
        Some(json!({
            "id": "bad_tool_plugin",
            "tool_json": {"id": "bad_tool_plugin", "name": "Bad", "kind": "gripper", "tool_xml": "tool.xml"},
            "tool_xml": r#"<mujoco><plugin plugin="evil"/><worldbody/></mujoco>"#
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{body}");
    assert_eq!(body["code"], "INVALID_PACK", "{body}");

    write_disk_tool(
        &tmp.path().join("tools/disk_bad_tool"),
        "disk_bad_tool",
        r#"<mujoco><asset><mesh file="missing.stl"/></asset><worldbody/></mujoco>"#,
        &[],
    );
    assert!(state.resolve_tool_checked("disk_bad_tool").is_err());

    // URDF rejection -> 400 UNSUPPORTED_FORMAT on both Boundary 1 and Boundary 2
    let (st, body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "urdf_scene",
            "scene_json": {"id": "urdf_scene", "name": "URDF", "world_xml": "world.xml"},
            "world_xml": r#"<mujoco><worldbody/></mujoco>"#,
            "assets": {"robot.urdf": "<robot name=\"x\"/>"}
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{body}");
    assert_eq!(body["code"], "UNSUPPORTED_FORMAT", "{body}");

    write_disk_scene(
        &tmp.path().join("scenes/disk_urdf_scene"),
        "disk_urdf_scene",
        r#"<mujoco><worldbody/></mujoco>"#,
        &[("extra.urdf", b"<robot name=\"x\"/>")],
    );
    let err = state.resolve_scene_checked("disk_urdf_scene").unwrap_err();
    assert_eq!(err.0, StatusCode::BAD_REQUEST);
    assert_eq!(err.1 .0["code"], "UNSUPPORTED_FORMAT");
}

#[tokio::test]
async fn sandbox_validation_rejects_bad_stl_obj_and_vertex_budgets_on_both_boundaries() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. Truncated binary STL (header says 2 tris, body only has 1)
    let mut truncated_stl = make_binary_stl(&[(
        [0.0, 0.0, 1.0],
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    )]);
    truncated_stl[80..84].copy_from_slice(&2u32.to_le_bytes());

    // 2. NaN binary STL
    let nan_stl = make_binary_stl(&[(
        [0.0, 0.0, 1.0],
        [[f32::NAN, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    )]);

    // 3. Vertex budget > 150_000 (50_001 triangles => 150_003 vertices)
    let over_budget_tris = vec![
        (
            [0.0, 0.0, 1.0],
            [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0]],
        );
        50_001
    ];
    let over_budget_stl = make_binary_stl(&over_budget_tris);

    // 4. ASCII STL missing endsolid
    let bad_ascii_stl = b"solid part\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\n".to_vec();

    // 5. OBJ with mtllib
    let mtllib_obj = b"mtllib evil.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n".to_vec();

    let mesh_cases: Vec<(&str, &str, Vec<u8>)> = vec![
        ("truncated_stl", "meshes/part.stl", truncated_stl),
        ("nan_stl", "meshes/part.stl", nan_stl),
        ("over_budget_stl", "meshes/part.stl", over_budget_stl),
        ("bad_ascii_stl", "meshes/part.stl", bad_ascii_stl),
        ("mtllib_obj", "meshes/part.obj", mtllib_obj),
    ];

    for (name, rel, bytes) in mesh_cases {
        let xml = format!(r#"<mujoco><asset><mesh name="p" file="{rel}"/></asset><worldbody/></mujoco>"#);
        let id = format!("mesh_{name}");
        let mut assets = serde_json::Map::new();
        assets.insert(
            rel.to_string(),
            json!({ "encoding": "base64", "data": b64_encode(&bytes) }),
        );
        let (st, body) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": id,
                "scene_json": {"id": id, "name": name, "world_xml": "world.xml"},
                "world_xml": xml,
                "assets": Value::Object(assets)
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "Boundary 1 ({name}): {body}");
        assert_eq!(body["code"], "INVALID_PACK", "Boundary 1 ({name}): {body}");

        let disk_id = format!("disk_mesh_{name}");
        write_disk_scene(
            &tmp.path().join("scenes").join(&disk_id),
            &disk_id,
            &xml,
            &[(rel, &bytes)],
        );
        assert_boundary2_rejected(&state, &app, &disk_id).await;
        let (st2, body2) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "wasm", "scene": disk_id, "tool": "default"})),
        )
        .await;
        assert_eq!(st2, StatusCode::BAD_REQUEST, "Boundary 2 ({name}): {body2}");
    }

    // Also test > 16 mesh files
    let mut many_assets = serde_json::Map::new();
    let mut disk_extras: Vec<(String, Vec<u8>)> = Vec::new();
    let tiny_stl = valid_asymmetric_stl();
    for i in 0..17 {
        let rel = format!("meshes/m{i}.stl");
        many_assets.insert(
            rel.clone(),
            json!({ "encoding": "base64", "data": b64_encode(&tiny_stl) }),
        );
        disk_extras.push((rel, tiny_stl.clone()));
    }
    let (st, body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "too_many_meshes",
            "scene_json": {"id": "too_many_meshes", "name": "Too Many", "world_xml": "world.xml"},
            "world_xml": "<mujoco><worldbody/></mujoco>",
            "assets": Value::Object(many_assets)
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{body}");
    assert_eq!(body["code"], "INVALID_PACK", "{body}");
}

#[tokio::test]
async fn sandbox_valid_asymmetric_stl_upload_and_activate_passes_both_boundaries_with_deterministic_digest() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl_bytes = valid_asymmetric_stl();
    let world_xml = r#"<mujoco model="asym_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;

    let (st, up) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "asym_scene",
            "scene_json": {"id": "asym_scene", "name": "Asym Scene", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": world_xml,
            "assets": {
                "meshes/asym.stl": { "encoding": "base64", "data": b64_encode(&stl_bytes) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{up}");
    let upload_digest = up["pack_digest"].as_str().unwrap().to_string();
    assert_eq!(upload_digest.len(), 64);
    assert_eq!(up["sha256"], upload_digest);

    // Verify Boundary 2 activate + bundle
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "asym_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["scene_digest"], upload_digest);

    // Also compute expected digest directly from files on disk
    let dir = tmp.path().join("scenes/asym_scene");
    let mut map = BTreeMap::new();
    for rel in ["scene.json", "world.xml", "meshes/asym.stl"] {
        map.insert(rel, std::fs::read(dir.join(rel)).unwrap());
    }
    let expected_digest = compute_pack_digest(map.iter().map(|(k, v)| (*k, v.as_slice())));
    assert_eq!(upload_digest, expected_digest);

    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");
    assert!(bundle["composed_xml"].as_str().unwrap().contains("asym.stl"));

    // Post-activation tampering on disk (Boundary 2!) must be rejected by get_bundle!
    std::fs::write(dir.join("meshes/asym.stl"), b"solid tampered_no_endsolid").unwrap();
    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "INVALID_PACK", "{err}");
}

#[tokio::test]
async fn sandbox_two_phase_prepare_commit_happy_path() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;
    let seen: Arc<Mutex<Vec<(String, Value)>>> = Arc::new(Mutex::new(Vec::new()));
    let seen_p = seen.clone();
    let seen_c = seen.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(body): axum::Json<Value>| {
                let seen_p = seen_p.clone();
                async move {
                    seen_p.lock().unwrap().push(("prepare".into(), body.clone()));
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": body["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(body): axum::Json<Value>| {
                let seen_c = seen_c.clone();
                async move {
                    seen_c.lock().unwrap().push(("commit".into(), body.clone()));
                    let sd = seen_c.lock().unwrap().first().map(|p| p.1["scene_digest"].clone()).unwrap_or(Value::Null);
                    axum::Json(json!({
                        "ok": true,
                        "state": "COMMITTED",
                        "tx_id": body["tx_id"],
                        "robot": "b601dm",
                        "scene_digest": sd,
                        "tool_digest": Value::Null
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl = valid_asymmetric_stl();
    let (st, up) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "custom_sim_scene",
            "scene_json": {"id": "custom_sim_scene", "name": "Custom Sim", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": r#"<mujoco><asset><mesh name="m" file="m.stl"/></asset><worldbody/></mujoco>"#,
            "assets": {"m.stl": {"encoding": "base64", "data": b64_encode(&stl)}}
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{up}");
    let scene_digest = up["pack_digest"].as_str().unwrap().to_string();

    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "custom_sim_scene", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["mode"], "sim");
    assert_eq!(act["active_scene"], "custom_sim_scene");
    assert_eq!(act["active_tool"], "suction");
    assert_eq!(act["scene_digest"], scene_digest);
    let tx_id = act["tx_id"].as_str().unwrap().to_string();

    {
        let calls = seen.lock().unwrap();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0].0, "prepare");
        assert_eq!(calls[0].1["tx_id"], tx_id);
        assert_eq!(calls[0].1["scene"], "custom_sim_scene");
        assert_eq!(calls[0].1["tool"], "suction");
        assert_eq!(calls[0].1["robot"], "b601dm");
        assert_eq!(calls[0].1["scene_digest"], scene_digest);
        assert!(calls[0].1["tool_digest"].is_null(), "B1: preset suction tool without disk dir must send null tool_digest");
        assert!(calls[0].1["scene_dir"].as_str().unwrap().contains("custom_sim_scene"));
        assert_eq!(calls[1].0, "commit");
        assert_eq!(calls[1].1["tx_id"], tx_id);
        assert_eq!(calls[1].1["phase"], "all");
    }

    let tx_state = state.tx_state.read().await.clone();
    assert!(matches!(tx_state, ActivationTxState::Committed { .. }));
}

#[tokio::test]
async fn sandbox_prepare_rejected_robot_incompatible_calls_abort_and_preserves_active_state() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let aborted_txs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let aborted_c = aborted_txs.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(|| async {
                (
                    StatusCode::BAD_REQUEST,
                    axum::Json(json!({"error": "robot mismatch on sim", "code": "ROBOT_INCOMPATIBLE"})),
                )
            }),
        )
        .route(
            "/sim/scene/abort",
            post(move |axum::Json(body): axum::Json<Value>| {
                let aborted_c = aborted_c.clone();
                async move {
                    if let Some(tx) = body["tx_id"].as_str() {
                        aborted_c.lock().unwrap().push(tx.to_string());
                    }
                    axum::Json(json!({"ok": true, "state": "ABORTED"}))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
    let tx_id = err["tx_id"].as_str().unwrap().to_string();
    assert_eq!(aborted_txs.lock().unwrap().as_slice(), &[tx_id]);

    let st_after = state.active.read().await;
    assert_eq!(st_after.mode, "none");
    assert_eq!(st_after.active_scene, "default");
}

#[tokio::test]
async fn sandbox_legacy_fallback_checks_sim_state_robot_before_calling_sim_scene() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let legacy_scene_called = Arc::new(Mutex::new(false));
    let called_c = legacy_scene_called.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            get(|| async { axum::Json(json!({"robot": "so-arm101"})) }),
        )
        .route(
            "/sim/scene",
            post(move || {
                let called_c = called_c.clone();
                async move {
                    *called_c.lock().unwrap() = true;
                    axum::Json(json!({"ok": true}))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
    assert!(
        !*legacy_scene_called.lock().unwrap(),
        "legacy POST /sim/scene must NOT be called when GET /sim/state reports mismatched robot"
    );
}

#[tokio::test]
async fn sandbox_commit_degraded_split_auto_converges_via_camera_only() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let phases_c = phases.clone();
    let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let lp_p = last_prep.clone();
    let lp_c = last_prep.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let lp_p = lp_p.clone();
                async move {
                    *lp_p.lock().unwrap() = Some(b.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let phases_c = phases_c.clone();
                let lp_c = lp_c.clone();
                async move {
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    phases_c.lock().unwrap().push(phase.clone());
                    let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                    if phase == "all" {
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "error": "camera swap failed",
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "robot": "b601dm",
                                "tx_id": b["tx_id"],
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })),
                        )
                    } else {
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "COMMITTED",
                                "tx_id": b["tx_id"],
                                "robot": "b601dm"
                            })),
                        )
                    }
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{body}");
    assert_eq!(body["active_scene"], "pcb_suction");
    assert_eq!(body["activation_state"], "COMMITTED");
    assert_eq!(phases.lock().unwrap().as_slice(), &["all", "camera_only"]);
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::Committed { .. }));
}

#[tokio::test]
async fn sandbox_commit_degraded_split_falls_back_to_rollback_physics_cleanly() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let phases_c = phases.clone();
    let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let lp_p = last_prep.clone();
    let lp_c = last_prep.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let lp_p = lp_p.clone();
                async move {
                    *lp_p.lock().unwrap() = Some(b.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let phases_c = phases_c.clone();
                let lp_c = lp_c.clone();
                async move {
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    phases_c.lock().unwrap().push(phase.clone());
                    let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                    match phase.as_str() {
                        "all" | "camera_only" => (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "robot": "b601dm",
                                "tx_id": b["tx_id"],
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })),
                        ),
                        "rollback_physics" => (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "ABORTED",
                                "tx_id": b["tx_id"],
                                "restored_tx_id": "prev-tx-committed-001"
                            })),
                        ),
                        _ => (StatusCode::BAD_REQUEST, axum::Json(json!({"error": "bad phase"}))),
                    }
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_GATEWAY, "{err}");
    assert_eq!(err["code"], "SIM_CAMERA_COMMIT_FAILED", "{err}");
    assert_eq!(
        phases.lock().unwrap().as_slice(),
        &["all", "camera_only", "rollback_physics"]
    );
    assert_eq!(state.active.read().await.active_scene, "default");
    assert_eq!(state.active.read().await.mode, "none");
    match state.tx_state.read().await.clone() {
        ActivationTxState::Committed { tx_id, .. } => {
            assert_eq!(tx_id, "prev-tx-committed-001", "M4: rollback_physics must record restored_tx_id");
        }
        other => panic!("expected Committed(restored_tx_id), got {other:?}"),
    };
}

#[tokio::test]
async fn sandbox_degraded_split_blocks_new_activate_exposes_state_and_recovers_via_endpoint() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let allow_recovery = Arc::new(Mutex::new(false));
    let allow_c = allow_recovery.clone();
    let seen_digests: Arc<Mutex<(Value, Value)>> = Arc::new(Mutex::new((Value::Null, Value::Null)));
    let sd_p = seen_digests.clone();
    let sd_c = seen_digests.clone();

    let seen_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let sp_p = seen_prep.clone();
    let sp_c = seen_prep.clone();
    let sp_s = seen_prep.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let sd_p = sd_p.clone();
                let sp_p = sp_p.clone();
                async move {
                    *sd_p.lock().unwrap() = (b["scene_digest"].clone(), b["tool_digest"].clone());
                    *sp_p.lock().unwrap() = Some(b.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let allow_c = allow_c.clone();
                let sd_c = sd_c.clone();
                let sp_c = sp_c.clone();
                async move {
                    if *allow_c.lock().unwrap() {
                        let (sd, td) = sd_c.lock().unwrap().clone();
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "COMMITTED",
                                "tx_id": b["tx_id"],
                                "robot": "b601dm",
                                "scene_digest": sd,
                                "tool_digest": td
                            })),
                        )
                    } else {
                        let prep = sp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "robot": "b601dm",
                                "tx_id": b["tx_id"],
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })),
                        )
                    }
                }
            }),
        )
        .route(
            "/sim/state",
            get(move || {
                let sp_s = sp_s.clone();
                async move {
                    let prep = sp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                    axum::Json(json!({
                        "activation_state": "DEGRADED_SPLIT",
                        "robot": "b601dm",
                        "tx_id": prep["tx_id"],
                        "scene": prep["scene"],
                        "tool": prep["tool"],
                        "scene_digest": prep["scene_digest"],
                        "tool_digest": prep["tool_digest"]
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    // Use OperatorAuth with PIN to also verify recover-split is operator-gated in api::mod router!
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("1234", 1800);
    let app = create_router_with_sandbox(supervisor, auth.clone(), state.clone());

    let (token, _) = auth.unlock("1234").unwrap();

    // 1. Activate -> stuck in DEGRADED_SPLIT (503)
    let req = Request::builder()
        .method("POST")
        .uri("/api/v1/sandbox/activate")
        .header("Content-Type", "application/json")
        .header("Authorization", format!("Bearer {token}"))
        .body(Body::from(
            serde_json::to_vec(&json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})).unwrap(),
        ))
        .unwrap();
    let res = app.clone().oneshot(req).await.unwrap();
    assert_eq!(res.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body: Value =
        serde_json::from_slice(&axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap()).unwrap();
    assert_eq!(body["code"], "SIM_ACTIVATION_DEGRADED", "{body}");
    assert_eq!(body["state"], "DEGRADED_SPLIT", "{body}");
    let degraded_tx = body["tx_id"].as_str().unwrap().to_string();

    // 2. GET /api/v1/sandbox/state reports DEGRADED_SPLIT
    let (st, sbody) = send_json(&app, "GET", "/api/v1/sandbox/state", None).await;
    assert_eq!(st, StatusCode::OK, "{sbody}");
    assert_eq!(sbody["activation_state"], "DEGRADED_SPLIT", "{sbody}");
    assert_eq!(sbody["degraded"], true, "{sbody}");
    assert_eq!(sbody["degraded_tx_id"], degraded_tx, "{sbody}");

    // 3. Subsequent conflicting activate returns 503 SIM_ACTIVATION_DEGRADED
    let req2 = Request::builder()
        .method("POST")
        .uri("/api/v1/sandbox/activate")
        .header("Content-Type", "application/json")
        .header("Authorization", format!("Bearer {token}"))
        .body(Body::from(
            serde_json::to_vec(&json!({"mode": "wasm", "scene": "default", "tool": "default"})).unwrap(),
        ))
        .unwrap();
    let res2 = app.clone().oneshot(req2).await.unwrap();
    assert_eq!(res2.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body2: Value =
        serde_json::from_slice(&axum::body::to_bytes(res2.into_body(), usize::MAX).await.unwrap()).unwrap();
    assert_eq!(body2["code"], "SIM_ACTIVATION_DEGRADED", "{body2}");

    // 4. recover-split without operator token is rejected (401/403)
    let unauth_req = Request::builder()
        .method("POST")
        .uri("/api/v1/sandbox/recover-split")
        .header("Content-Type", "application/json")
        .body(Body::from(r#"{"action":"replay_camera"}"#))
        .unwrap();
    let unauth_res = app.clone().oneshot(unauth_req).await.unwrap();
    assert!(
        unauth_res.status() == StatusCode::UNAUTHORIZED || unauth_res.status() == StatusCode::FORBIDDEN,
        "recover-split must require operator auth, got {}",
        unauth_res.status()
    );

    // 5. Enable recovery on sim and call /api/v1/sandbox/recover-split -> converges cleanly!
    *allow_recovery.lock().unwrap() = true;
    let rec_req = Request::builder()
        .method("POST")
        .uri("/api/v1/sandbox/recover-split")
        .header("Content-Type", "application/json")
        .header("Authorization", format!("Bearer {token}"))
        .body(Body::from(r#"{"action":"replay_camera"}"#))
        .unwrap();
    let rec_res = app.clone().oneshot(rec_req).await.unwrap();
    let rec_status = rec_res.status();
    let rec_body: Value =
        serde_json::from_slice(&axum::body::to_bytes(rec_res.into_body(), usize::MAX).await.unwrap()).unwrap();

    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(rec_status, StatusCode::OK, "{rec_body}");
    assert_eq!(rec_body["state"], "COMMITTED", "{rec_body}");
    assert_eq!(rec_body["active_scene"], "pcb_suction", "{rec_body}");
    assert_eq!(state.active.read().await.active_scene, "pcb_suction");
    assert_eq!(state.active.read().await.mode, "sim");

    let (st, sbody2) = send_json(&app, "GET", "/api/v1/sandbox/state", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(sbody2["degraded"], false);
    assert_eq!(sbody2["activation_state"], "COMMITTED");
}

#[tokio::test]
async fn sandbox_lost_commit_ack_reconciles_via_sim_state_to_committed() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let committed_tx: Arc<Mutex<Option<(String, Value, Value, Value, Value)>>> = Arc::new(Mutex::new(None));
    let committed_tx_c = committed_tx.clone();
    let prep_meta: Arc<Mutex<Option<(Value, Value, Value, Value)>>> = Arc::new(Mutex::new(None));
    let prep_meta_c = prep_meta.clone();

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();

    // Raw TCP mock server that responds to prepare, drops connection on commit ACK,
    // and answers GET /sim/state with COMMITTED for that tx_id + scene/tool/digests.
    let server = tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = listener.accept().await else {
                break;
            };
            let committed_tx_c = committed_tx_c.clone();
            let prep_meta_c = prep_meta_c.clone();
            tokio::spawn(async move {
                let Some((first_line, parsed)) = read_http_request(&mut sock).await else {
                    return;
                };

                if first_line.starts_with("POST /sim/scene/prepare ") {
                    *prep_meta_c.lock().unwrap() = Some((
                        parsed["scene"].clone(),
                        parsed["tool"].clone(),
                        parsed["scene_digest"].clone(),
                        parsed["tool_digest"].clone(),
                    ));
                    let resp_body = json!({
                        "ok": true,
                        "state": "PREPARED",
                        "tx_id": parsed["tx_id"],
                        "robot": "b601dm"
                    })
                    .to_string();
                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        resp_body.len(),
                        resp_body
                    );
                    let _ = sock.write_all(resp.as_bytes()).await;
                } else if first_line.starts_with("POST /sim/scene/commit ") {
                    // Sim committed, then dropped TCP connection before sending HTTP response (lost ACK)!
                    if let Some(tx) = parsed["tx_id"].as_str() {
                        let (sc, tl, sd, td) = prep_meta_c
                            .lock()
                            .unwrap()
                            .clone()
                            .unwrap_or((Value::Null, Value::Null, Value::Null, Value::Null));
                        *committed_tx_c.lock().unwrap() = Some((tx.to_string(), sc, tl, sd, td));
                    }
                    drop(sock);
                } else if first_line.starts_with("GET /sim/state ") {
                    let (tx, sc, tl, sd, td) = committed_tx_c
                        .lock()
                        .unwrap()
                        .clone()
                        .unwrap_or((String::new(), Value::Null, Value::Null, Value::Null, Value::Null));
                    let resp_body = json!({
                        "tx_id": tx,
                        "activation_state": "COMMITTED",
                        "robot": "b601dm",
                        "scene": sc,
                        "tool": tl,
                        "scene_digest": sd,
                        "tool_digest": td
                    })
                    .to_string();
                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        resp_body.len(),
                        resp_body
                    );
                    let _ = sock.write_all(resp.as_bytes()).await;
                }
            });
        }
    });

    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf()).with_sim_timeouts(SimTxTimeouts {
            prepare: Duration::from_millis(500),
            commit: Duration::from_millis(500),
            abort: Duration::from_millis(200),
            reconcile: Duration::from_millis(500),
        }),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["active_scene"], "pcb_suction");
    assert_eq!(act["activation_state"], "COMMITTED");
    assert_eq!(state.active.read().await.active_scene, "pcb_suction");
    assert_eq!(state.active.read().await.mode, "sim");
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::Committed { .. }));
}

#[tokio::test]
async fn sandbox_pack_digest_matches_python_length_prefixed_vector() {
    // Matches Python rebot_sim.pack.compute_pack_digest:
    // rel.encode("utf-8") + b"\x00" + len(data).to_bytes(8, "little") + data
    let expected = "1dae3c1afa949a5be15f903a1ab6a50b9100e12e310d6007f3d5b719f3ab65c2";

    let digest = compute_pack_digest([
        ("world.xml", b"<mujoco><worldbody/></mujoco>".as_slice()),
        ("meshes/part.stl", b"solid p\nendsolid p\n".as_slice()),
        ("scene.json", br#"{"id":"vec_scene"}"#.as_slice()),
    ]);
    assert_eq!(digest, expected);

    // Also verify Boundary 2 disk reload + activate produces the exact same digest
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("scenes/vec_scene");
    std::fs::create_dir_all(dir.join("meshes")).unwrap();
    std::fs::write(dir.join("scene.json"), br#"{"id":"vec_scene"}"#).unwrap();
    std::fs::write(dir.join("world.xml"), b"<mujoco><worldbody/></mujoco>").unwrap();
    std::fs::write(dir.join("meshes/part.stl"), b"solid p\nendsolid p\n").unwrap();

    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "vec_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");
    assert_eq!(act["scene_digest"], expected);
}

#[tokio::test]
async fn sandbox_b1_default_preset_sends_null_digests_to_prepare() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let seen_prepare: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let seen_p = seen_prepare.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let seen_p = seen_p.clone();
                async move {
                    *seen_p.lock().unwrap() = Some(b.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(|axum::Json(b): axum::Json<Value>| async move {
                axum::Json(json!({
                    "ok": true,
                    "state": "COMMITTED",
                    "tx_id": b["tx_id"],
                    "robot": "b601dm",
                    "scene_digest": Value::Null,
                    "tool_digest": Value::Null
                }))
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "default", "tool": "default"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{act}");
    let prep = seen_prepare.lock().unwrap().clone().unwrap();
    assert!(prep["scene_digest"].is_null(), "default scene must send null digest: {prep}");
    assert!(prep["tool_digest"].is_null(), "default tool must send null digest: {prep}");
}

#[tokio::test]
async fn sandbox_b2_legacy_404_rejects_missing_or_failing_sim_state_without_calling_sim_scene() {
    let _env_guard = SIM_ENV_LOCK.lock().await;

    for (label, state_resp, expected_status, expected_code) in [
        ("missing_robot", (StatusCode::OK, r#"{"ok":true}"#), StatusCode::BAD_REQUEST, Some("ROBOT_INCOMPATIBLE")),
        ("invalid_json", (StatusCode::OK, "not-json"), StatusCode::BAD_GATEWAY, Some("SIM_STATE_INVALID")),
        ("not_found", (StatusCode::NOT_FOUND, "missing"), StatusCode::BAD_GATEWAY, Some("SIM_STATE_UNAVAILABLE")),
    ] {
        let scene_called = Arc::new(Mutex::new(false));
        let called_c = scene_called.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(move || async move { (state_resp.0, state_resp.1) }),
            )
            .route(
                "/sim/scene",
                post(move || {
                    let called_c = called_c.clone();
                    async move {
                        *called_c.lock().unwrap() = true;
                        axum::Json(json!({"ok": true}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = app_for(ArmConfig::rebot_dm(), state);

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, expected_status, "{label}: {err}");
        if let Some(code) = expected_code {
            assert_eq!(err["code"], code, "{label}: {err}");
        }
        assert!(!*scene_called.lock().unwrap(), "{label}: POST /sim/scene must not be called");
    }
}

#[tokio::test]
async fn sandbox_m1_m2_unknown_504_and_committing_and_unknown_to_degraded_split_recovery() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let sim_state_mode = Arc::new(Mutex::new("COMMITTING"));
    let sim_state_c = sim_state_mode.clone();
    let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let seen_tx_p = seen_tx.clone();
    let seen_tx_s = seen_tx.clone();
    let commit_calls: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let commit_calls_c = commit_calls.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let seen_tx_p = seen_tx_p.clone();
                async move {
                    *seen_tx_p.lock().unwrap() = b["tx_id"].as_str().map(str::to_string);
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let commit_calls_c = commit_calls_c.clone();
                async move {
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    commit_calls_c.lock().unwrap().push(phase.clone());
                    if phase == "all" {
                        (StatusCode::GATEWAY_TIMEOUT, axum::Json(json!({"error": "commit gateway timeout"})))
                    } else {
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "COMMITTED",
                                "tx_id": b["tx_id"],
                                "robot": "b601dm"
                            })),
                        )
                    }
                }
            }),
        )
        .route(
            "/sim/state",
            get(move || {
                let sim_state_c = sim_state_c.clone();
                let seen_tx_s = seen_tx_s.clone();
                async move {
                    let mode = *sim_state_c.lock().unwrap();
                    let cur_tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                    match mode {
                        "COMMITTING" => axum::Json(json!({
                            "tx_id": "older-tx-000",
                            "activation_state": "COMMITTING",
                            "robot": "b601dm"
                        })),
                        "DEGRADED_SPLIT" => axum::Json(json!({
                            "tx_id": cur_tx,
                            "activation_state": "DEGRADED_SPLIT",
                            "robot": "b601dm",
                            "scene": "pcb_suction",
                            "tool": "suction"
                        })),
                        _ => axum::Json(json!({
                            "tx_id": cur_tx,
                            "activation_state": "COMMITTED",
                            "scene": "pcb_suction",
                            "tool": "suction"
                        })),
                    }
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. commit returns 504 -> reconcile sees COMMITTING -> stays Unknown (503 SIM_ACTIVATION_UNKNOWN)
    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{err}");
    assert_eq!(err["code"], "SIM_ACTIVATION_UNKNOWN", "{err}");
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::Unknown { .. }));

    // 2. Remote finishes physics and lands in DEGRADED_SPLIT -> next activate reconciles Unknown -> DegradedSplit (503)
    *sim_state_mode.lock().unwrap() = "DEGRADED_SPLIT";
    let (st2, err2) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "default"})),
    )
    .await;
    assert_eq!(st2, StatusCode::SERVICE_UNAVAILABLE, "{err2}");
    assert_eq!(err2["code"], "SIM_ACTIVATION_DEGRADED", "{err2}");
    assert!(
        matches!(*state.tx_state.read().await, ActivationTxState::DegradedSplit { .. }),
        "M1: must transition Unknown -> DegradedSplit so recover-split succeeds"
    );

    // 3. Now recover-split must succeed (not 409)!
    let (st3, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st3, StatusCode::OK, "{rec}");
    assert_eq!(rec["state"], "COMMITTED", "{rec}");
    assert_eq!(state.active.read().await.active_scene, "pcb_suction");
}

#[tokio::test]
async fn sandbox_m3_generic_503_not_degraded_and_prepare_split_records_remote_owner_tx() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let mode = Arc::new(Mutex::new("GENERIC_503"));
    let mode_c = mode.clone();
    let aborted = Arc::new(Mutex::new(false));
    let aborted_c = aborted.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move || {
                let mode_c = mode_c.clone();
                async move {
                    if *mode_c.lock().unwrap() == "GENERIC_503" {
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({"error": "upstream busy"})),
                        )
                    } else {
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "error": "existing split",
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "tx_id": "remote-split-owner-99"
                            })),
                        )
                    }
                }
            }),
        )
        .route(
            "/sim/scene/abort",
            post(move || {
                let aborted_c = aborted_c.clone();
                async move {
                    *aborted_c.lock().unwrap() = true;
                    axum::Json(json!({"ok": true, "state": "ABORTED"}))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. Generic 503 without SIM_ACTIVATION_DEGRADED/DEGRADED_SPLIT must NOT enter DegradedSplit
    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert!(*aborted.lock().unwrap());
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::Aborted { .. }));

    // 2. prepare returning SIM_ACTIVATION_DEGRADED records remote owner tx_id (NOT newly generated tx_id)
    *mode.lock().unwrap() = "REMOTE_SPLIT";
    let (st2, err2) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st2, StatusCode::SERVICE_UNAVAILABLE, "{err2}");
    assert_eq!(err2["tx_id"], "remote-split-owner-99", "{err2}");
    match state.tx_state.read().await.clone() {
        ActivationTxState::DegradedSplit { tx_id, target_matches_tx, .. } => {
            assert_eq!(tx_id.as_deref(), Some("remote-split-owner-99"));
            assert!(!target_matches_tx);
        }
        other => panic!("expected DegradedSplit, got {other:?}"),
    };
}

#[tokio::test]
async fn sandbox_m4_commit_ack_rejects_mismatched_tx_or_digest_or_state() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(|axum::Json(b): axum::Json<Value>| async move {
                axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
            }),
        )
        .route(
            "/sim/scene/commit",
            post(|| async move {
                axum::Json(json!({
                    "ok": true,
                    "state": "COMMITTED",
                    "tx_id": "wrong-tx-id",
                    "robot": "b601dm"
                }))
            }),
        )
        .route("/sim/scene/abort", post(|| async { axum::Json(json!({"ok": true})) }));

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_GATEWAY, "{err}");
    assert_eq!(err["code"], "SIM_INVALID_COMMIT_ACK", "{err}");
    assert_eq!(state.active.read().await.mode, "none");
}

#[tokio::test]
async fn sandbox_m5_restart_recovery_adopts_remote_degraded_split_from_idle() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let commit_txs: Arc<Mutex<Vec<(String, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let commit_txs_c = commit_txs.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            get(|| async {
                axum::Json(json!({
                    "tx_id": "pre-restart-split-tx",
                    "activation_state": "DEGRADED_SPLIT",
                    "robot": "b601dm",
                    "scene": "pcb_suction",
                    "tool": "suction"
                }))
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let commit_txs_c = commit_txs_c.clone();
                async move {
                    let tx = b["tx_id"].as_str().unwrap_or("").to_string();
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    commit_txs_c.lock().unwrap().push((tx.clone(), phase));
                    axum::Json(json!({
                        "ok": true,
                        "state": "ABORTED",
                        "tx_id": tx,
                        "restored_tx_id": "pre-split-healthy-tx"
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    // Fresh SandboxState starts in ActivationTxState::Idle (simulating armd restart)
    let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, res) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "rollback_physics"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{res}");
    assert_eq!(res["state"], "ROLLED_BACK", "{res}");
    assert_eq!(res["tx_id"], "pre-restart-split-tx", "{res}");
    assert_eq!(res["restored_tx_id"], "pre-split-healthy-tx", "{res}");
    assert_eq!(
        commit_txs.lock().unwrap().as_slice(),
        &[("pre-restart-split-tx".to_string(), "rollback_physics".to_string())]
    );
    match state.tx_state.read().await.clone() {
        ActivationTxState::Committed { tx_id, .. } => assert_eq!(tx_id, "pre-split-healthy-tx"),
        other => panic!("expected Committed(pre-split-healthy-tx), got {other:?}"),
    };
}

#[tokio::test]
async fn sandbox_m12_m13_m14_m15_m16_m19_m20_pack_validation_parity_and_budgets() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. M13: payload.id override rewrites manifest["id"] on disk and matches pack_digest
    let (st, up) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "override_id",
            "scene_json": {"id": "original_id", "name": "Override", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": "<mujoco><worldbody/></mujoco>"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{up}");
    let disk_json: Value =
        serde_json::from_slice(&std::fs::read(tmp.path().join("scenes/override_id/scene.json")).unwrap()).unwrap();
    assert_eq!(disk_json["id"], "override_id");

    // 2. M15 & M16: Subdirectory XML + two-pass <compiler> after <asset> + XML-relative resolution
    let stl = valid_asymmetric_stl();
    let (st, sub_up) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "subdir_xml_scene",
            "files": {
                "scene.json": serde_json::to_string(&json!({
                    "id": "subdir_xml_scene",
                    "name": "Subdir XML",
                    "world_xml": "sub/world.xml",
                    "robots": ["b601dm"]
                })).unwrap(),
                "sub/world.xml": r#"<mujoco><asset><mesh name="p" file="part.stl"/></asset><compiler meshdir="meshes"/><worldbody/></mujoco>"#,
                "sub/meshes/part.stl": { "encoding": "base64", "data": b64_encode(&stl) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{sub_up}");
    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "subdir_xml_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act}");

    // Secondary XML inside pack with forbidden <plugin> must be rejected (M15)
    let (st, bad_sec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "bad_secondary_xml",
            "files": {
                "scene.json": serde_json::to_string(&json!({
                    "id": "bad_secondary_xml",
                    "name": "Bad Sec",
                    "world_xml": "world.xml"
                })).unwrap(),
                "world.xml": "<mujoco><worldbody/></mujoco>",
                "sub/extra.xml": r#"<mujoco><plugin plugin="evil"/><worldbody/></mujoco>"#
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{bad_sec}");
    assert_eq!(bad_sec["code"], "INVALID_PACK", "{bad_sec}");

    // 3. M19: Inline <mesh vertex="..."> exceeding 150,000 vertices is rejected
    let huge_verts = "0 0 0 ".repeat(150_001);
    let huge_inline_xml = format!(
        "<mujoco><asset><mesh name=\"big\" vertex=\"{huge_verts}\"/></asset><worldbody/></mujoco>"
    );
    if huge_inline_xml.len() <= MAX_XML_BYTES {
        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": "inline_over_verts",
                "scene_json": {"id": "inline_over_verts", "name": "Inline", "world_xml": "world.xml"},
                "world_xml": huge_inline_xml
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    }
    // Also test 17 inline <mesh vertex="..."> elements > MAX_MESH_FILES (16)
    let inline_17: String = (0..17)
        .map(|i| format!(r#"<mesh name="m{i}" vertex="0 0 0 1 0 0 0 1 0"/>"#))
        .collect();
    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "inline_17_meshes",
            "scene_json": {"id": "inline_17_meshes", "name": "17 Inline", "world_xml": "world.xml"},
            "world_xml": format!("<mujoco><asset>{inline_17}</asset><worldbody/></mujoco>")
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "INVALID_PACK", "{err}");

    // 4. M20: Malformed ASCII STL (stray vertex outside facet, and 2-vertex facet)
    for (name, bad_stl) in [
        ("stray_vertex", "solid a\nvertex 0 0 0\nendsolid a\n"),
        (
            "two_vertex_facet",
            "solid a\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid a\n",
        ),
    ] {
        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": format!("bad_ascii_{name}"),
                "scene_json": {"id": format!("bad_ascii_{name}"), "name": name, "world_xml": "world.xml"},
                "world_xml": r#"<mujoco><asset><mesh name="m" file="m.stl"/></asset><worldbody/></mujoco>"#,
                "assets": {"m.stl": bad_stl}
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "{name}: {err}");
        assert_eq!(err["code"], "INVALID_PACK", "{name}: {err}");
    }

    // 5. M20: OBJ out-of-bounds face index (positive > len, and negative < -len)
    for (name, bad_obj) in [
        ("pos_oob", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n"),
        ("neg_oob", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -1 -2 -4\n"),
        ("vt_oob", "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nf 1/1 2/1 3/2\n"),
    ] {
        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": format!("bad_obj_{name}"),
                "scene_json": {"id": format!("bad_obj_{name}"), "name": name, "world_xml": "world.xml"},
                "world_xml": r#"<mujoco><asset><mesh name="m" file="m.obj"/></asset><worldbody/></mujoco>"#,
                "assets": {"m.obj": bad_obj}
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "{name}: {err}");
        assert_eq!(err["code"], "INVALID_PACK", "{name}: {err}");
    }

    // 6. m1: Non-ASCII robot ID rejected
    assert!(normalize_robot_id("b601dm\u{00e9}").is_none());
    assert!(normalize_robot_id("   ").is_none());
    assert_eq!(normalize_robot_id("B601_DM").as_deref(), Some("b601dm"));
}

#[tokio::test]
async fn sandbox_b01_foreign_split_owner_never_adopts_local_target_config() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let replay_txs: Arc<Mutex<Vec<(String, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let replay_c = replay_txs.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(|axum::Json(b): axum::Json<Value>| async move {
                axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let replay_c = replay_c.clone();
                async move {
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    let tx = b["tx_id"].as_str().unwrap_or("").to_string();
                    replay_c.lock().unwrap().push((tx.clone(), phase.clone()));
                    if phase == "all" {
                        (
                            StatusCode::GATEWAY_TIMEOUT,
                            axum::Json(json!({"error": "commit timed out"})),
                        )
                    } else {
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "COMMITTED",
                                "tx_id": tx,
                                "robot": "b601dm",
                                "scene_digest": Value::Null,
                                "tool_digest": Value::Null
                            })),
                        )
                    }
                }
            }),
        )
        .route(
            "/sim/state",
            get(|| async {
                axum::Json(json!({
                    "tx_id": "foreign-owner-tx-B",
                    "activation_state": "DEGRADED_SPLIT",
                    "robot": "b601dm",
                    "scene": "pcb_suction",
                    "tool": "suction"
                }))
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{err}");
    assert_eq!(err["code"], "SIM_ACTIVATION_DEGRADED", "{err}");
    assert_eq!(err["tx_id"], "foreign-owner-tx-B", "{err}");

    match state.tx_state.read().await.clone() {
        ActivationTxState::DegradedSplit { tx_id, target_matches_tx, target_composed_mjcf, .. } => {
            assert_eq!(tx_id.as_deref(), Some("foreign-owner-tx-B"));
            assert!(!target_matches_tx, "B01: foreign split owner must NOT mark A's target config as matching");
            assert!(target_composed_mjcf.is_empty());
        }
        other => panic!("expected foreign DegradedSplit, got {other:?}"),
    }

    let (st_rec, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st_rec, StatusCode::CONFLICT, "{rec}");
    assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
    assert_eq!(rec["state"], "DEGRADED_SPLIT", "{rec}");
    assert_eq!(rec["tx_id"], "foreign-owner-tx-B", "{rec}");
    // R3 M1: replay_camera must NOT send /sim/scene/commit when target_matches_tx is false
    assert!(
        replay_txs.lock().unwrap().iter().all(|(_, phase)| phase != "camera_only"),
        "camera_only commit must NOT be sent on mismatched split target: {:?}",
        *replay_txs.lock().unwrap()
    );
    assert!(matches!(
        *state.tx_state.read().await,
        ActivationTxState::DegradedSplit { ref tx_id, target_matches_tx: false, .. } if tx_id.as_deref() == Some("foreign-owner-tx-B")
    ));
    // B01: ActiveState must NOT have been overwritten with A's scene/mode!
    assert_eq!(state.active.read().await.mode, "none");
    assert_eq!(state.active.read().await.active_scene, "default");
}

#[tokio::test]
async fn sandbox_m01_builtin_disk_tool_sends_both_tool_dir_and_tool_digest() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let seen_prepare: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let seen_p = seen_prepare.clone();
    let seen_c = seen_prepare.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let seen_p = seen_p.clone();
                async move {
                    *seen_p.lock().unwrap() = Some(b.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let seen_c = seen_c.clone();
                async move {
                    let prep = seen_c.lock().unwrap().clone().unwrap();
                    axum::Json(json!({
                        "ok": true,
                        "state": "COMMITTED",
                        "tx_id": b["tx_id"],
                        "robot": "b601dm",
                        "scene_digest": prep["scene_digest"],
                        "tool_digest": prep["tool_digest"]
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let builtin_root = tempfile::tempdir().unwrap();
    let suction_dir = builtin_root.path().join("tools/suction");
    std::fs::create_dir_all(&suction_dir).unwrap();
    std::fs::write(
        suction_dir.join("tool.json"),
        serde_json::to_vec(&json!({
            "id": "suction",
            "name": "Suction Cup",
            "kind": "suction",
            "robots": ["b601dm"],
            "tcp_offset": [0.0, 0.0, 0.0]
        }))
        .unwrap(),
    )
    .unwrap();
    std::fs::write(suction_dir.join("tool.xml"), "<mujoco><worldbody/></mujoco>").unwrap();

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![builtin_root.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state);

    let (st, act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "default", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::OK, "{act}");
    let prep = seen_prepare.lock().unwrap().clone().unwrap();
    assert!(prep["tool_dir"].as_str().unwrap().contains("suction"), "M01: must send tool_dir when backed by disk dir: {prep}");
    assert_eq!(prep["tool_digest"].as_str().unwrap().len(), 64, "M01: must send SHA-256 tool_digest alongside tool_dir: {prep}");
    assert!(prep["scene_digest"].is_null());
}

#[tokio::test]
async fn sandbox_m02_m01_legacy_202_or_invalid_sim_state_aborts_cleanly() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for (label, status, body, exp_status) in [
        ("accepted_202", StatusCode::ACCEPTED, r#"{"robot":"b601dm"}"#, StatusCode::BAD_GATEWAY),
        ("invalid_json", StatusCode::OK, "{not-json", StatusCode::BAD_GATEWAY),
        ("missing_robot", StatusCode::OK, r#"{"ok":true}"#, StatusCode::BAD_REQUEST),
    ] {
        let scene_called = Arc::new(Mutex::new(false));
        let sc = scene_called.clone();
        let mock_sim = axum::Router::new()
            .route("/sim/state", get(move || async move { (status, body) }))
            .route(
                "/sim/scene",
                post(move || {
                    let sc = sc.clone();
                    async move {
                        *sc.lock().unwrap() = true;
                        axum::Json(json!({"ok": true}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, exp_status, "{label}: {err}");
        assert!(!*scene_called.lock().unwrap(), "{label}: /sim/scene must not be called");
        assert!(
            matches!(*state.tx_state.read().await, ActivationTxState::Aborted { .. }),
            "{label}: m01 must transition tx_state to Aborted"
        );
    }
}

#[tokio::test]
async fn sandbox_m05_commit_ack_requires_robot_and_expected_digests() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;

    for (case, omit_robot, omit_digest) in [
        ("missing_robot", true, false),
        ("missing_scene_digest", false, true),
    ] {
        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(|axum::Json(b): axum::Json<Value>| async move {
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| async move {
                    let mut resp = json!({
                        "ok": true,
                        "state": "COMMITTED",
                        "tx_id": b["tx_id"],
                    });
                    if !omit_robot {
                        resp["robot"] = json!("b601dm");
                    }
                    if !omit_digest {
                        resp["scene_digest"] = json!("0".repeat(64));
                    }
                    axum::Json(resp)
                }),
            )
            .route("/sim/scene/abort", post(|| async { axum::Json(json!({"ok": true})) }));

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st_up, _) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": "m05_custom",
                "scene_json": {"id": "m05_custom", "name": "M05", "world_xml": "world.xml", "robots": ["b601dm"]},
                "world_xml": "<mujoco><worldbody/></mujoco>"
            })),
        )
        .await;
        assert_eq!(st_up, StatusCode::OK);

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "m05_custom", "tool": "default"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::BAD_GATEWAY, "{case}: {err}");
        assert_eq!(err["code"], "SIM_INVALID_COMMIT_ACK", "{case}: {err}");
        assert_eq!(state.active.read().await.active_scene, "default");
    }
}

#[tokio::test]
async fn sandbox_m06_commit_502_and_damaged_2xx_reconcile_via_sim_state() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for mode in ["502", "damaged_2xx"] {
        let last_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let tx_p = last_tx.clone();
        let tx_s = last_tx.clone();
        let aborted = Arc::new(Mutex::new(false));
        let aborted_c = aborted.clone();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let tx_p = tx_p.clone();
                    async move {
                        *tx_p.lock().unwrap() = b["tx_id"].as_str().map(str::to_string);
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || async move {
                    if mode == "502" {
                        (StatusCode::BAD_GATEWAY, "{\"error\":\"bad gateway\"}".to_string())
                    } else {
                        (StatusCode::OK, "{\"state\":\"COMMITTED".to_string())
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let tx_s = tx_s.clone();
                    async move {
                        let tx = tx_s.lock().unwrap().clone().unwrap_or_default();
                        axum::Json(json!({
                            "tx_id": tx,
                            "activation_state": "COMMITTED",
                            "robot": "b601dm",
                            "scene": "pcb_suction",
                            "tool": "suction"
                        }))
                    }
                }),
            )
            .route(
                "/sim/scene/abort",
                post(move || {
                    let aborted_c = aborted_c.clone();
                    async move {
                        *aborted_c.lock().unwrap() = true;
                        axum::Json(json!({"ok": true}))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, res) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::OK, "{mode}: {res}");
        assert_eq!(res["activation_state"], "COMMITTED");
        assert!(!*aborted.lock().unwrap(), "{mode}: must reconcile via /sim/state, NOT blindly abort");
        assert!(matches!(*state.tx_state.read().await, ActivationTxState::Committed { .. }));
    }
}

#[tokio::test]
async fn sandbox_m07_rollback_ack_rejects_missing_or_mismatched_restored_tx_id() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let first_tx = Arc::new(Mutex::new(String::new()));
    let first_c = first_tx.clone();
    let call_idx = Arc::new(Mutex::new(0usize));
    let idx_c = call_idx.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(|axum::Json(b): axum::Json<Value>| async move {
                axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let first_c = first_c.clone();
                let idx_c = idx_c.clone();
                async move {
                    let mut n = idx_c.lock().unwrap();
                    *n += 1;
                    let cur = *n;
                    let tx = b["tx_id"].as_str().unwrap_or("").to_string();
                    let phase = b["phase"].as_str().unwrap_or("");
                    if cur == 1 {
                        *first_c.lock().unwrap() = tx.clone();
                        (
                            StatusCode::OK,
                            axum::Json(json!({"ok": true, "state": "COMMITTED", "tx_id": tx, "robot": "b601dm"})),
                        )
                    } else if phase == "rollback_physics" {
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "ok": true,
                                "state": "ABORTED",
                                "tx_id": tx,
                                "restored_tx_id": "wrong-previous-tx"
                            })),
                        )
                    } else {
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({"code": "SIM_ACTIVATION_DEGRADED", "state": "DEGRADED_SPLIT"})),
                        )
                    }
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st1, r1) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "default", "tool": "default"})),
    )
    .await;
    assert_eq!(st1, StatusCode::OK, "{r1}");

    let (st2, err2) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st2, StatusCode::SERVICE_UNAVAILABLE, "{err2}");
    assert_eq!(err2["code"], "SIM_ACTIVATION_DEGRADED", "{err2}");
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::DegradedSplit { .. }));
}

#[tokio::test]
async fn sandbox_m08_restart_recovery_digest_mismatch_never_publishes_stale_mjcf() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state_setup = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app_setup = app_for(ArmConfig::rebot_dm(), state_setup);

    let (st, up) = send_json(
        &app_setup,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "m08_scene",
            "scene_json": {"id": "m08_scene", "name": "M08", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": "<mujoco><worldbody><body name=\"old_body\"/></worldbody></mujoco>"
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{up}");
    let remote_digest = up["pack_digest"].as_str().unwrap().to_string();

    // Mutate scene on disk after the split happened so local digest differs from remote_digest!
    std::fs::write(
        tmp.path().join("scenes/m08_scene/world.xml"),
        "<mujoco><worldbody><body name=\"mutated_local_body\"/></worldbody></mujoco>",
    )
    .unwrap();

    let remote_d_clone = remote_digest.clone();
    let commit_called = Arc::new(Mutex::new(false));
    let commit_c = commit_called.clone();
    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            get(move || {
                let rd = remote_d_clone.clone();
                async move {
                    axum::Json(json!({
                        "tx_id": "split-tx-m08",
                        "activation_state": "DEGRADED_SPLIT",
                        "robot": "b601dm",
                        "scene": "m08_scene",
                        "tool": "default",
                        "scene_digest": rd,
                        "tool_digest": Value::Null
                    }))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let rd = remote_digest.clone();
                let commit_c = commit_c.clone();
                async move {
                    *commit_c.lock().unwrap() = true;
                    axum::Json(json!({
                        "ok": true,
                        "state": "COMMITTED",
                        "tx_id": b["tx_id"],
                        "robot": "b601dm",
                        "scene_digest": rd,
                        "tool_digest": Value::Null
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    // Fresh SandboxState starts Idle (restarted armd)
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st_rec, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st_rec, StatusCode::CONFLICT, "{rec}");
    assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
    assert_eq!(rec["state"], "DEGRADED_SPLIT", "{rec}");
    assert_eq!(rec["tx_id"], "split-tx-m08", "{rec}");
    assert!(!*commit_called.lock().unwrap(), "M08: /sim/scene/commit must NOT be called when digests mismatch");
    assert!(matches!(
        *state.tx_state.read().await,
        ActivationTxState::DegradedSplit { ref tx_id, target_matches_tx: false, .. } if tx_id.as_deref() == Some("split-tx-m08")
    ));
    let active = state.active.read().await;
    assert!(
        !active.cached_mjcf.contains("mutated_local_body"),
        "M08: replay_camera must NEVER publish mutated local MJCF when digests mismatch"
    );
    assert_eq!(active.active_scene, "default");
}

#[tokio::test]
async fn sandbox_m09_prepare_ack_rejects_202_missing_state_or_wrong_tx_id() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for (case, status, missing_state, wrong_tx) in [
        ("status_202", StatusCode::ACCEPTED, false, false),
        ("missing_state", StatusCode::OK, true, false),
        ("wrong_tx_id", StatusCode::OK, false, true),
    ] {
        let commit_called = Arc::new(Mutex::new(false));
        let cc = commit_called.clone();
        let aborted = Arc::new(Mutex::new(false));
        let ab = aborted.clone();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| async move {
                    let mut body = json!({
                        "ok": true,
                        "robot": "b601dm",
                        "tx_id": if wrong_tx { json!("other-tx") } else { b["tx_id"].clone() }
                    });
                    if !missing_state {
                        body["state"] = json!("PREPARED");
                    }
                    (status, axum::Json(body))
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() = true;
                        axum::Json(json!({"ok": true}))
                    }
                }),
            )
            .route(
                "/sim/scene/abort",
                post(move || {
                    let ab = ab.clone();
                    async move {
                        *ab.lock().unwrap() = true;
                        axum::Json(json!({"ok": true}))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::BAD_GATEWAY, "{case}: {err}");
        assert_eq!(err["code"], "SIM_INVALID_PREPARE_ACK", "{case}: {err}");
        assert!(!*commit_called.lock().unwrap(), "{case}: commit must NOT be called");
        assert!(*aborted.lock().unwrap(), "{case}: abort must be called");
        assert!(matches!(*state.tx_state.read().await, ActivationTxState::Aborted { .. }));
    }
}

#[tokio::test]
async fn sandbox_bundle_cache_hits_after_activate_and_reflects_live_mutable_fields() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl_bytes = valid_asymmetric_stl();
    let world_xml = r#"<mujoco model="cached_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "cached_scene",
            "scene_json": {"id": "cached_scene", "name": "Cached Scene", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": world_xml,
            "assets": {
                "meshes/asym.stl": { "encoding": "base64", "data": b64_encode(&stl_bytes) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "cached_scene", "tool": "default", "seed": 100})),
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    let initial_rebuilds = state.bundle_rebuild_count();
    assert_eq!(initial_rebuilds, 0, "activate must populate bundle cache without extra rebuild");

    for _ in 0..5 {
        let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
        assert_eq!(st, StatusCode::OK, "{bundle}");
        assert_eq!(bundle["seed"], 100);
        assert_eq!(bundle["reset_seq"], 0);
        assert!(bundle["composed_xml"].as_str().unwrap().contains("asym.stl"));
    }
    assert_eq!(
        state.bundle_rebuild_count(),
        initial_rebuilds,
        "repeated GET /bundle calls must hit cache with 0 rebuilds"
    );

    // Mutate live fields (seed/reset_seq via reset, desired_io via io)
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/reset", Some(json!({"seed": 777}))).await;
    assert_eq!(st, StatusCode::OK);
    let (st, _) = send_json(&app, "POST", "/api/v1/sandbox/io", Some(json!({"vacuum": true}))).await;
    assert_eq!(st, StatusCode::OK);

    let (st, bundle2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle2}");
    assert_eq!(bundle2["seed"], 777);
    assert_eq!(bundle2["reset_seq"], 1);
    assert_eq!(bundle2["desired_io"]["vacuum"], true);
    assert_eq!(
        state.bundle_rebuild_count(),
        initial_rebuilds,
        "live field updates must not invalidate immutable bundle cache"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn sandbox_bundle_concurrent_cold_gets_coalesce_into_single_rebuild() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();

    let stl_bytes = valid_asymmetric_stl();
    let world_xml = r#"<mujoco model="cold_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    write_disk_scene(
        &tmp.path().join("scenes/cold_scene"),
        "cold_scene",
        world_xml,
        &[("meshes/asym.stl", &stl_bytes)],
    );

    // Cold SandboxState with active_scene pointing to disk pack (no prior activate in memory)
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    state.active.write().await.active_scene = "cold_scene".to_string();
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let mut handles = Vec::new();
    for _ in 0..8 {
        let app_c = app.clone();
        handles.push(tokio::spawn(async move {
            send_json(&app_c, "GET", "/api/v1/sandbox/bundle", None).await
        }));
    }
    for h in handles {
        let (st, body) = h.await.unwrap();
        assert_eq!(st, StatusCode::OK, "{body}");
        assert!(body["composed_xml"].as_str().unwrap().contains("asym.stl"));
    }
    assert!(
        state.bundle_rebuild_count() <= 1,
        "concurrent cold GETs must coalesce into at most 1 rebuild, got {}",
        state.bundle_rebuild_count()
    );
}

#[tokio::test]
async fn sandbox_bundle_rejects_same_byte_size_valid_replacement_tampering_on_disk() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl_v1 = valid_asymmetric_stl();
    let stl_v2 = make_binary_stl(&[
        (
            [0.0, 0.0, 1.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.0, 0.07, 0.02]],
        ),
        (
            [0.0, -1.0, 0.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.03, 0.0, 0.06]],
        ),
    ]);
    assert_eq!(stl_v1.len(), stl_v2.len(), "test requires identical byte length");
    assert_ne!(stl_v1, stl_v2);

    let world_xml = r#"<mujoco model="tamper_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "tamper_scene",
            "scene_json": {"id": "tamper_scene", "name": "Tamper Scene", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": world_xml,
            "assets": {
                "meshes/asym.stl": { "encoding": "base64", "data": b64_encode(&stl_v1) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "tamper_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    // Replace STL via temp file + rename (same byte length, valid STL, different content & inode)
    let target_stl = tmp.path().join("scenes/tamper_scene/meshes/asym.stl");
    let tmp_stl = tmp.path().join("scenes/tamper_scene/meshes/.asym.stl.tmp");
    std::fs::write(&tmp_stl, &stl_v2).unwrap();
    std::fs::rename(&tmp_stl, &target_stl).unwrap();

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "INVALID_PACK", "{err}");
}

#[tokio::test]
async fn sandbox_m5_rejects_expansion_tags_and_enforces_single_and_combined_element_budgets() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // 1. Reject expansion/forbidden XML constructs on both Boundary 1 and Boundary 2
    let expansion_cases = [
        ("replicate", r#"<mujoco><worldbody><replicate count="10"><body name="r"/></replicate></worldbody></mujoco>"#),
        ("frame", r#"<mujoco><worldbody><frame pos="0 0 0"><body name="f"/></frame></worldbody></mujoco>"#),
        ("include", r#"<mujoco><include file="other.xml"/><worldbody/></mujoco>"#),
        ("composite", r#"<mujoco><worldbody><composite type="grid" count="2 2 1"/></worldbody></mujoco>"#),
        ("flexcomp", r#"<mujoco><worldbody><flexcomp name="fx" type="grid" count="2 2 1"/></worldbody></mujoco>"#),
        ("attach", r#"<mujoco><worldbody><attach model="m" body="b"/></worldbody></mujoco>"#),
        ("model", r#"<mujoco><asset><model name="m" file="sub.xml"/></asset><worldbody/></mujoco>"#),
    ];
    for (name, xml) in expansion_cases {
        let id = format!("exp_{name}");
        let (st, body) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/scenes",
            Some(json!({
                "id": id,
                "scene_json": {"id": id, "name": name, "world_xml": "world.xml"},
                "world_xml": xml
            })),
        )
        .await;
        assert_eq!(st, StatusCode::BAD_REQUEST, "Boundary 1 ({name}): {body}");
        assert_eq!(body["code"], "INVALID_PACK", "Boundary 1 ({name}): {body}");

        let disk_id = format!("disk_exp_{name}");
        write_disk_scene(&tmp.path().join("scenes").join(&disk_id), &disk_id, xml, &[]);
        assert_boundary2_rejected(&state, &app, &disk_id).await;
    }

    // 2. Accept exact MAX_PACK_BODIES and MAX_PACK_GEOMS, reject limit + 1
    let make_bodies_xml = |n: usize| {
        let mut s = String::from("<mujoco><worldbody>");
        for i in 0..n {
            s.push_str(&format!(r#"<body name="b{i}"/>"#));
        }
        s.push_str("</worldbody></mujoco>");
        s
    };
    let make_geoms_xml = |n: usize| {
        let mut s = String::from("<mujoco><worldbody>");
        for i in 0..n {
            s.push_str(&format!(r#"<geom name="g{i}" type="box" size="0.01 0.01 0.01"/>"#));
        }
        s.push_str("</worldbody></mujoco>");
        s
    };

    let (st, ok_bodies) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "exact_bodies",
            "scene_json": {"id": "exact_bodies", "name": "Exact Bodies", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": make_bodies_xml(MAX_PACK_BODIES)
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{ok_bodies}");

    let (st, over_bodies) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "over_bodies",
            "scene_json": {"id": "over_bodies", "name": "Over Bodies", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": make_bodies_xml(MAX_PACK_BODIES + 1)
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{over_bodies}");
    assert_eq!(over_bodies["code"], "INVALID_PACK");

    let (st, ok_geoms) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "exact_geoms",
            "scene_json": {"id": "exact_geoms", "name": "Exact Geoms", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": make_geoms_xml(MAX_PACK_GEOMS)
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{ok_geoms}");

    let (st, over_geoms) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "over_geoms",
            "scene_json": {"id": "over_geoms", "name": "Over Geoms", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": make_geoms_xml(MAX_PACK_GEOMS + 1)
        })),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{over_geoms}");
    assert_eq!(over_geoms["code"], "INVALID_PACK");

    // 3. Combined scene (300 bodies) + tool (250 bodies) = 550 > 512: each uploads fine individually,
    //    but compose_mjcf AND POST /activate reject before compilation!
    let scene_300 = make_bodies_xml(300);
    let tool_250 = make_bodies_xml(250);
    assert_eq!(MAX_XML_ELEMENTS, 4_096);

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "scene_300_bodies",
            "scene_json": {"id": "scene_300_bodies", "name": "Scene 300", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": scene_300
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let (st, tool_body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/tools",
        Some(json!({
            "id": "tool_250_bodies",
            "tool_json": {"id": "tool_250_bodies", "name": "Tool 250", "kind": "gripper", "tool_xml": "tool.xml", "robots": ["b601dm"], "tcp_offset": [0.02, 0.0, 0.0]},
            "tool_xml": tool_250
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{tool_body}");

    // Direct compose_mjcf rejects combined overflow
    assert!(compose_mjcf(FALLBACK_BASE_XML, Some(&scene_300), None, Some(&tool_250)).is_err());

    // Activate rejects combined overflow with 400 INVALID_PACK
    let (st, err_act) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "scene_300_bodies", "tool": "tool_250_bodies"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err_act}");
    assert_eq!(err_act["code"], "INVALID_PACK", "{err_act}");
}

#[tokio::test]
async fn sandbox_m07_concurrent_warm_bundle_gets_coalesce_stat_walks() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let stl_bytes = valid_asymmetric_stl();
    let world_xml = r#"<mujoco model="warm_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    write_disk_scene(
        &tmp.path().join("scenes/warm_scene"),
        "warm_scene",
        world_xml,
        &[("meshes/asym.stl", &stl_bytes)],
    );

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    state.active.write().await.active_scene = "warm_scene".to_string();
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // Cold GET warms the bundle cache.
    let (st0, body0) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st0, StatusCode::OK, "{body0}");
    assert_eq!(state.bundle_rebuild_count(), 1);
    let expected_sha = body0["composed_xml_sha256"].as_str().unwrap().to_string();

    // Concurrent burst of 12 warm GETs must coalesce onto 1 stat walk and 0 rebuilds.
    state.set_stat_walk_delay_ms(80);
    let walks_before = state.bundle_stat_walk_count();
    let rebuilds_before = state.bundle_rebuild_count();
    let barrier = Arc::new(tokio::sync::Barrier::new(12));
    let mut handles = Vec::new();
    for _ in 0..12 {
        let app_c = app.clone();
        let b_c = barrier.clone();
        handles.push(tokio::spawn(async move {
            b_c.wait().await;
            send_json(&app_c, "GET", "/api/v1/sandbox/bundle", None).await
        }));
    }
    for h in handles {
        let (st, body) = h.await.unwrap();
        assert_eq!(st, StatusCode::OK, "{body}");
        assert_eq!(body["composed_xml_sha256"].as_str(), Some(expected_sha.as_str()));
    }
    assert_eq!(state.bundle_rebuild_count(), rebuilds_before);
    assert_eq!(
        state.bundle_stat_walk_count() - walks_before,
        1,
        "12 concurrent warm GETs must coalesce onto 1 stat walk"
    );

    // Sequential GET after burst must still run a fresh stat check (not stale-cached).
    state.set_stat_walk_delay_ms(0);
    let (st_seq, _) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st_seq, StatusCode::OK);
    assert_eq!(state.bundle_stat_walk_count() - walks_before, 2);
}

#[tokio::test]
async fn sandbox_m08_recover_split_shares_rebuild_semaphore_with_cold_get_bundle() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _upload_guard = UPLOAD_LOCK.lock().await;

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let stl_bytes = valid_asymmetric_stl();
    let world_xml = r#"<mujoco model="split_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    write_disk_scene(
        &tmp.path().join("scenes/split_scene"),
        "split_scene",
        world_xml,
        &[("meshes/asym.stl", &stl_bytes)],
    );

    // replay_camera adopts the remote target only when its digests match locally verified
    // packs: read them from a separate state on the same root so this state's cache stays cold.
    let digests = {
        let probe = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        probe.active.write().await.active_scene = "split_scene".to_string();
        let probe_app = app_for(ArmConfig::rebot_dm(), probe);
        let (st, b) = send_json(&probe_app, "GET", "/api/v1/sandbox/bundle", None).await;
        assert_eq!(st, StatusCode::OK, "{b}");
        (b["scene_digest"].clone(), b["tool_digest"].clone())
    };
    let (scene_digest, tool_digest) = digests;
    let (scene_digest_c, tool_digest_c) = (scene_digest.clone(), tool_digest.clone());

    let commit_done = Arc::new(tokio::sync::Notify::new());
    let commit_done_c = commit_done.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/commit",
            post(move || {
                let commit_done_c = commit_done_c.clone();
                let (sd, td) = (scene_digest_c.clone(), tool_digest_c.clone());
                async move {
                    commit_done_c.notify_waiters();
                    (
                        StatusCode::OK,
                        axum::Json(json!({
                            "ok": true,
                            "state": "COMMITTED",
                            "tx_id": "tx-m08",
                            "camera_tx_id": "tx-m08",
                            "robot": "b601dm",
                            "scene": "split_scene",
                            "tool": "default",
                            "scene_digest": sd,
                            "tool_digest": td
                        })),
                    )
                }
            }),
        )
        .route(
            "/sim/state",
            get(move || {
                let (sd, td) = (scene_digest.clone(), tool_digest.clone());
                async move {
                    axum::Json(json!({
                        "tx_id": "tx-m08",
                        "activation_state": "DEGRADED_SPLIT",
                        "robot": "b601dm",
                        "scene": "split_scene",
                        "tool": "default",
                        "scene_digest": sd,
                        "tool_digest": td
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    state.active.write().await.active_scene = "split_scene".to_string();
    state.set_bundle_rebuild_delay_ms(80);
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let notified = commit_done.notified();
    let app_rec = app.clone();
    let mut rec_handle = tokio::spawn(async move {
        send_json(
            &app_rec,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "replay_camera"})),
        )
        .await
    });

    // Wait until recover_split receives 200 from /sim/scene/commit and enters build_bundle_from_disk,
    // then fire concurrent cold GET /api/v1/sandbox/bundle requests against the empty cache.
    // A recover_split that ends before reaching the commit fails the test instead of hanging it.
    tokio::select! {
        () = notified => {}
        res = &mut rec_handle => panic!("recover-split finished before /sim/scene/commit: {:?}", res.unwrap()),
    }
    tokio::time::sleep(Duration::from_millis(10)).await;
    let mut get_handles = Vec::new();
    for _ in 0..4 {
        let app_get = app.clone();
        get_handles.push(tokio::spawn(async move {
            send_json(&app_get, "GET", "/api/v1/sandbox/bundle", None).await
        }));
    }

    let (st_rec, body_rec) = rec_handle.await.unwrap();
    assert_eq!(st_rec, StatusCode::OK, "{body_rec}");
    assert_eq!(body_rec["state"], "COMMITTED", "{body_rec}");

    for h in get_handles {
        let (st_get, body_get) = h.await.unwrap();
        assert_eq!(st_get, StatusCode::OK, "{body_get}");
        assert!(body_get["composed_xml"].as_str().unwrap().contains("asym.stl"));
    }

    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(
        state.peak_concurrent_rebuilds(),
        1,
        "recover_split and cold GET /bundle must share bundle_rebuild_sem so peak_concurrent_rebuilds == 1"
    );
}

#[tokio::test]
async fn sandbox_bundle_invalidates_cache_on_inplace_same_length_mtime_preserving_overwrite() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl_v1 = valid_asymmetric_stl();
    let stl_v2 = make_binary_stl(&[
        (
            [0.0, 0.0, 1.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.0, 0.07, 0.02]],
        ),
        (
            [0.0, -1.0, 0.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.03, 0.0, 0.06]],
        ),
    ]);
    assert_eq!(stl_v1.len(), stl_v2.len());
    assert_ne!(stl_v1, stl_v2);

    let world_xml = r#"<mujoco model="mtime_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "mtime_scene",
            "scene_json": {"id": "mtime_scene", "name": "Mtime Scene", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": world_xml,
            "assets": {
                "meshes/asym.stl": { "encoding": "base64", "data": b64_encode(&stl_v1) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "mtime_scene", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    // Warm the cache via GET /bundle.
    let (st, bundle) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::OK, "{bundle}");

    // Overwrite meshes/asym.stl IN-PLACE with identical length and restore exact original mtime.
    let target_stl = tmp.path().join("scenes/mtime_scene/meshes/asym.stl");
    let orig_mtime = std::fs::metadata(&target_stl).unwrap().modified().unwrap();
    tokio::time::sleep(Duration::from_millis(5)).await;
    {
        use std::io::Write;
        let mut f = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open(&target_stl)
            .unwrap();
        f.write_all(&stl_v2).unwrap();
        f.set_modified(orig_mtime).unwrap();
        f.sync_all().unwrap();
    }
    assert_eq!(
        std::fs::metadata(&target_stl).unwrap().modified().unwrap(),
        orig_mtime
    );

    let (st, err) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "INVALID_PACK", "{err}");
}

#[tokio::test]
async fn sandbox_bundle_cold_rebuild_rejects_mid_build_modification_without_caching() {
    let _guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let stl_v1 = valid_asymmetric_stl();
    let stl_v2 = make_binary_stl(&[
        (
            [0.0, 0.0, 1.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.0, 0.07, 0.02]],
        ),
        (
            [0.0, -1.0, 0.0],
            [[0.0, 0.0, 0.0], [0.09, 0.0, 0.0], [0.03, 0.0, 0.06]],
        ),
    ]);

    let world_xml = r#"<mujoco model="midbuild_world"><compiler meshdir="meshes"/><asset><mesh name="asym" file="asym.stl"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="asym"/></body></worldbody></mujoco>"#;
    let (st, _) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/scenes",
        Some(json!({
            "id": "midbuild_scene",
            "scene_json": {"id": "midbuild_scene", "name": "Midbuild Scene", "world_xml": "world.xml", "robots": ["b601dm"]},
            "world_xml": world_xml,
            "assets": {
                "meshes/asym.stl": { "encoding": "base64", "data": b64_encode(&stl_v1) }
            }
        })),
    )
    .await;
    assert_eq!(st, StatusCode::OK);

    state.active.write().await.active_scene = "midbuild_scene".to_string();
    state.set_bundle_rebuild_delay_ms(80);

    let app_get = app.clone();
    let get_handle = tokio::spawn(async move {
        send_json(&app_get, "GET", "/api/v1/sandbox/bundle", None).await
    });

    // Wait until cold build has collected pre_fp and entered its sleep window, then mutate disk.
    tokio::time::sleep(Duration::from_millis(25)).await;
    let target_stl = tmp.path().join("scenes/midbuild_scene/meshes/asym.stl");
    std::fs::write(&target_stl, &stl_v2).unwrap();

    let (st, err) = get_handle.await.unwrap();
    assert!(
        matches!(st, StatusCode::CONFLICT | StatusCode::BAD_REQUEST),
        "unexpected status {st}: {err}"
    );
    assert_eq!(err["code"], "INVALID_PACK", "{err}");

    // Cache must NOT have been populated with a mismatched entry: restoring v1 and re-querying rebuilds cleanly.
    state.set_bundle_rebuild_delay_ms(0);
    std::fs::write(&target_stl, &stl_v1).unwrap();
    let (st2, body2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st2, StatusCode::OK, "{body2}");
    assert!(body2["composed_xml"].as_str().unwrap().contains("asym.stl"));
}

#[tokio::test]
async fn sandbox_r2_m2_recover_split_rejects_incompatible_remote_tool_for_running_robot() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let so101_cfg = armd::tool::baseline_arm_config_for_robot("so101").unwrap();

    for (tool_id, tool_json) in [
        (
            "dm_only_gripper",
            json!({
                "id": "dm_only_gripper",
                "name": "dm_only_gripper",
                "kind": "gripper",
                "robots": ["b601dm"],
                "tcp_offset": [0.02, 0.0, 0.0]
            }),
        ),
        (
            "dm_limits_unrestricted_gripper",
            json!({
                "id": "dm_limits_unrestricted_gripper",
                "name": "dm_limits_unrestricted_gripper",
                "kind": "gripper",
                "robots": [],
                "tcp_offset": [0.02, 0.0, 0.0],
                "armd": {
                    "gripper_policy": {
                        "closed_rad": -2.0
                    }
                }
            }),
        ),
    ] {
        let upload_state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let upload_app = app_for(so101_cfg.clone(), upload_state.clone());
        let (st, up) = send_json(
            &upload_app,
            "POST",
            "/api/v1/sandbox/tools",
            Some(json!({
                "id": tool_id,
                "tool_json": tool_json,
                "tool_xml": "<mujoco><worldbody><body name=\"dm_only_body\"><geom type=\"box\" size=\"0.01 0.01 0.01\"/></body></worldbody></mujoco>"
            })),
        )
        .await;
        assert_eq!(st, StatusCode::OK, "{tool_id}: {up}");
        let remote_td = up["pack_digest"].as_str().unwrap().to_string();

        let td_state = remote_td.clone();
        let tid = tool_id.to_string();
        let td_commit = remote_td.clone();
        let commit_called = Arc::new(Mutex::new(false));
        let commit_c = commit_called.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(move || {
                    let td = td_state.clone();
                    let tname = tid.clone();
                    async move {
                        axum::Json(json!({
                            "tx_id": "split-tx-r2-m2",
                            "activation_state": "DEGRADED_SPLIT",
                            "robot": "so-arm101",
                            "scene": "default",
                            "tool": tname,
                            "scene_digest": Value::Null,
                            "tool_digest": td
                        }))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let td = td_commit.clone();
                    let commit_c = commit_c.clone();
                    async move {
                        *commit_c.lock().unwrap() = true;
                        axum::Json(json!({
                            "ok": true,
                            "state": "COMMITTED",
                            "tx_id": b["tx_id"],
                            "robot": "so-arm101",
                            "scene_digest": Value::Null,
                            "tool_digest": td
                        }))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let rec_state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let rec_app = app_for(so101_cfg.clone(), rec_state.clone());
        let (st_rec, rec) = send_json(
            &rec_app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "replay_camera"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st_rec, StatusCode::CONFLICT, "{tool_id}: {rec}");
        assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{tool_id}: {rec}");
        assert_eq!(rec["state"], "DEGRADED_SPLIT", "{tool_id}: {rec}");
        assert_eq!(rec["tx_id"], "split-tx-r2-m2", "{tool_id}: {rec}");
        assert!(!*commit_called.lock().unwrap(), "{tool_id}: /sim/scene/commit must NOT be called");
        assert!(matches!(
            *rec_state.tx_state.read().await,
            ActivationTxState::DegradedSplit { ref tx_id, target_matches_tx: false, .. } if tx_id.as_deref() == Some("split-tx-r2-m2")
        ));
        let active = rec_state.active.read().await;
        assert_eq!(active.active_tool, "default", "{tool_id} must not be adopted on so-arm101");
        assert!(!active.cached_mjcf.contains("dm_only_body"), "{tool_id} MJCF must not be published");
    }
}

#[tokio::test]
async fn sandbox_r3_m2_unknown_same_tx_target_mismatch_never_commits_local_mjcf() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let dm_cfg = ArmConfig::rebot_dm();

    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(dm_cfg, state.clone());

    let (st_a, up_a) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/tools",
        Some(json!({
            "id": "r3_tool_a",
            "tool_json": {
                "id": "r3_tool_a",
                "name": "r3_tool_a",
                "kind": "gripper",
                "robots": ["b601dm"],
                "tcp_offset": [0.02, 0.0, 0.0]
            },
            "tool_xml": "<mujoco><worldbody><body name=\"r3_tool_a_body\"><geom type=\"box\" size=\"0.01 0.01 0.01\"/></body></worldbody></mujoco>"
        })),
    )
    .await;
    assert_eq!(st_a, StatusCode::OK, "{up_a}");

    let (st_b, up_b) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/tools",
        Some(json!({
            "id": "r3_tool_b",
            "tool_json": {
                "id": "r3_tool_b",
                "name": "r3_tool_b",
                "kind": "gripper",
                "robots": ["b601dm"],
                "tcp_offset": [0.03, 0.0, 0.0]
            },
            "tool_xml": "<mujoco><worldbody><body name=\"r3_tool_b_body\"><geom type=\"box\" size=\"0.01 0.01 0.01\"/></body></worldbody></mujoco>"
        })),
    )
    .await;
    assert_eq!(st_b, StatusCode::OK, "{up_b}");
    let tool_b_digest = up_b["pack_digest"].as_str().unwrap().to_string();

    let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let seen_tx_p = seen_tx.clone();
    let seen_tx_s = seen_tx.clone();
    let sim_mode = Arc::new(Mutex::new("COMMITTING"));
    let sim_mode_s = sim_mode.clone();
    let commit_phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let commit_phases_c = commit_phases.clone();
    let td_b_s = tool_b_digest.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let seen_tx_p = seen_tx_p.clone();
                async move {
                    let tx = b["tx_id"].as_str().unwrap().to_string();
                    *seen_tx_p.lock().unwrap() = Some(tx.clone());
                    axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": tx, "robot": "b601dm"}))
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let commit_phases_c = commit_phases_c.clone();
                async move {
                    let phase = b["phase"].as_str().unwrap_or("").to_string();
                    commit_phases_c.lock().unwrap().push(phase);
                    (StatusCode::GATEWAY_TIMEOUT, axum::Json(json!({"error": "commit timeout"})))
                }
            }),
        )
        .route(
            "/sim/state",
            get(move || {
                let seen_tx_s = seen_tx_s.clone();
                let sim_mode_s = sim_mode_s.clone();
                let td_b = td_b_s.clone();
                async move {
                    let tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                    let m = *sim_mode_s.lock().unwrap();
                    if m == "COMMITTING" {
                        axum::Json(json!({
                            "tx_id": tx,
                            "activation_state": "COMMITTING",
                            "robot": "b601dm"
                        }))
                    } else {
                        axum::Json(json!({
                            "tx_id": tx,
                            "activation_state": "DEGRADED_SPLIT",
                            "robot": "b601dm",
                            "scene": "default",
                            "tool": "r3_tool_b",
                            "scene_digest": Value::Null,
                            "tool_digest": td_b
                        }))
                    }
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    // 1. Activate r3_tool_a -> 504 on commit, /sim/state COMMITTING -> Unknown state caching r3_tool_a's MJCF.
    let (st_act, act_err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "default", "tool": "r3_tool_a"})),
    )
    .await;
    assert_eq!(st_act, StatusCode::SERVICE_UNAVAILABLE, "{act_err}");
    assert_eq!(act_err["code"], "SIM_ACTIVATION_UNKNOWN", "{act_err}");
    assert!(matches!(*state.tx_state.read().await, ActivationTxState::Unknown { .. }));

    // 2. Remote /sim/state now reports DEGRADED_SPLIT under the SAME tx_id, but naming r3_tool_b!
    *sim_mode.lock().unwrap() = "DEGRADED_SPLIT";
    let (st_rec, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st_rec, StatusCode::CONFLICT, "{rec}");
    assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
    assert_eq!(rec["state"], "DEGRADED_SPLIT", "{rec}");
    assert!(
        commit_phases.lock().unwrap().as_slice() == ["all"],
        "replay_camera must NOT send camera_only commit on Unknown same-tx tool mismatch: {:?}",
        *commit_phases.lock().unwrap()
    );
    assert!(matches!(
        *state.tx_state.read().await,
        ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
    ));
    let active = state.active.read().await;
    assert_eq!(active.active_tool, "default");
    assert!(!active.cached_mjcf.contains("r3_tool_a_body"));
    assert!(!active.cached_mjcf.contains("r3_tool_b_body"));
}

#[tokio::test]
async fn sandbox_r4_m2_lost_commit_ack_rejects_mismatched_committed_target() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let committed_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let committed_tx_c = committed_tx.clone();

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = listener.accept().await else {
                break;
            };
            let committed_tx_c = committed_tx_c.clone();
            tokio::spawn(async move {
                let Some((first_line, parsed)) = read_http_request(&mut sock).await else {
                    return;
                };
                if first_line.starts_with("POST /sim/scene/prepare ") {
                    let resp_body = json!({
                        "ok": true,
                        "state": "PREPARED",
                        "tx_id": parsed["tx_id"],
                        "robot": "b601dm"
                    })
                    .to_string();
                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        resp_body.len(),
                        resp_body
                    );
                    let _ = sock.write_all(resp.as_bytes()).await;
                } else if first_line.starts_with("POST /sim/scene/commit ") {
                    if let Some(tx) = parsed["tx_id"].as_str() {
                        *committed_tx_c.lock().unwrap() = Some(tx.to_string());
                    }
                    drop(sock);
                } else if first_line.starts_with("GET /sim/state ") {
                    let tx = committed_tx_c.lock().unwrap().clone().unwrap_or_default();
                    // Same tx_id and COMMITTED, but wrong tool reported!
                    let resp_body = json!({
                        "tx_id": tx,
                        "activation_state": "COMMITTED",
                        "robot": "b601dm",
                        "scene": "pcb_suction",
                        "tool": "other_tool"
                    })
                    .to_string();
                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        resp_body.len(),
                        resp_body
                    );
                    let _ = sock.write_all(resp.as_bytes()).await;
                }
            });
        }
    });

    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()])
            .with_sim_timeouts(SimTxTimeouts {
                prepare: Duration::from_millis(500),
                commit: Duration::from_millis(500),
                abort: Duration::from_millis(200),
                reconcile: Duration::from_millis(500),
            }),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::CONFLICT, "{err}");
    assert_eq!(err["code"], "SIM_RECOVERY_INCOMPATIBLE", "{err}");
    assert_eq!(state.active.read().await.mode, "none");
    assert_eq!(state.active.read().await.active_scene, "default");
}

#[tokio::test]
async fn sandbox_r4_m3_prepare_degraded_split_requires_explicit_owner_and_matching_target() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for case in ["missing_owner_tx", "same_tx_mismatched_target"] {
        let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let seen_tx_p = seen_tx.clone();
        let commit_called = Arc::new(Mutex::new(false));
        let commit_c = commit_called.clone();
        let case_s = case.to_string();
        let case_p = case.to_string();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let seen_tx_p = seen_tx_p.clone();
                    let case_p = case_p.clone();
                    async move {
                        let tx = b["tx_id"].as_str().unwrap().to_string();
                        *seen_tx_p.lock().unwrap() = Some(tx.clone());
                        if case_p == "missing_owner_tx" {
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT"
                                })),
                            )
                        } else {
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": tx,
                                    "scene": "pcb_suction",
                                    "tool": "different_tool"
                                })),
                            )
                        }
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let case_s = case_s.clone();
                    async move {
                        if case_s == "missing_owner_tx" {
                            axum::Json(json!({
                                "activation_state": "DEGRADED_SPLIT",
                                "robot": "b601dm",
                                "scene": "pcb_suction",
                                "tool": "suction"
                            }))
                        } else {
                            axum::Json(json!({"activation_state": "DEGRADED_SPLIT"}))
                        }
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let commit_c = commit_c.clone();
                    async move {
                        *commit_c.lock().unwrap() = true;
                        axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{case}: {err}");
        let _ = seen_tx.lock().unwrap().clone().unwrap();
        match state.tx_state.read().await.clone() {
            ActivationTxState::DegradedSplit { tx_id, target_matches_tx, target_composed_mjcf, .. } => {
                assert!(!target_matches_tx, "{case}: target_matches_tx must be false");
                assert!(target_composed_mjcf.is_empty(), "{case}: unverified MJCF must be empty");
                if case == "missing_owner_tx" {
                    assert_eq!(tx_id, None, "unknown remote owner must be stored as None, not fabricated");
                }
            }
            other => panic!("{case}: expected DegradedSplit, got {other:?}"),
        }

        let (st_rec, rec) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "replay_camera"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        if case == "missing_owner_tx" {
            assert_eq!(st_rec, StatusCode::BAD_GATEWAY, "{case}: {rec}");
            assert_eq!(rec["code"], "SIM_STATE_INVALID", "{case}: {rec}");
        } else {
            assert_eq!(st_rec, StatusCode::CONFLICT, "{case}: {rec}");
            assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{case}: {rec}");
        }
        assert!(!*commit_called.lock().unwrap(), "{case}: /sim/scene/commit must NOT be called");
    }
}

#[tokio::test]
async fn sandbox_r4_m4_recovery_rejects_omitted_scene_tool_or_missing_digest() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let _up_guard = UPLOAD_LOCK.lock().await;

    // Case 1 (from Idle restart): /sim/state omits `scene`/`tool` -> must NOT fall back to active "default".
    {
        let commit_called = Arc::new(Mutex::new(false));
        let commit_c = commit_called.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(|| async {
                    axum::Json(json!({
                        "tx_id": "restart-missing-target",
                        "activation_state": "DEGRADED_SPLIT",
                        "robot": "b601dm"
                    }))
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let commit_c = commit_c.clone();
                    async move {
                        *commit_c.lock().unwrap() = true;
                        axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, rec) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "replay_camera"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::CONFLICT, "{rec}");
        assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
        assert!(!*commit_called.lock().unwrap());
    }

    // Case 2 (from Unknown same-tx): digest-bearing uploaded tool where /sim/state omits `tool_digest` -> must NOT match.
    {
        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st_up, up) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/tools",
            Some(json!({
                "id": "r4_m4_tool",
                "tool_json": {
                    "id": "r4_m4_tool",
                    "kind": "gripper",
                    "robots": ["b601dm"],
                    "tcp_offset": [0.02, 0.0, 0.0]
                },
                "tool_xml": "<mujoco><worldbody/></mujoco>"
            })),
        )
        .await;
        assert_eq!(st_up, StatusCode::OK, "{up}");

        let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let seen_tx_p = seen_tx.clone();
        let seen_tx_s = seen_tx.clone();
        let sim_mode = Arc::new(Mutex::new("COMMITTING"));
        let sim_mode_s = sim_mode.clone();
        let commit_phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let commit_phases_c = commit_phases.clone();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let seen_tx_p = seen_tx_p.clone();
                    async move {
                        let tx = b["tx_id"].as_str().unwrap().to_string();
                        *seen_tx_p.lock().unwrap() = Some(tx.clone());
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": tx, "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let commit_phases_c = commit_phases_c.clone();
                    async move {
                        commit_phases_c.lock().unwrap().push(b["phase"].as_str().unwrap_or("").to_string());
                        (StatusCode::GATEWAY_TIMEOUT, axum::Json(json!({"error": "timeout"})))
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let seen_tx_s = seen_tx_s.clone();
                    let sim_mode_s = sim_mode_s.clone();
                    async move {
                        let tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                        if *sim_mode_s.lock().unwrap() == "COMMITTING" {
                            axum::Json(json!({"tx_id": tx, "activation_state": "COMMITTING", "robot": "b601dm"}))
                        } else {
                            // Omitted tool_digest even though local pack has a digest!
                            axum::Json(json!({
                                "tx_id": tx,
                                "activation_state": "DEGRADED_SPLIT",
                                "robot": "b601dm",
                                "scene": "default",
                                "tool": "r4_m4_tool"
                            }))
                        }
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let (st_act, act_err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "default", "tool": "r4_m4_tool"})),
        )
        .await;
        assert_eq!(st_act, StatusCode::SERVICE_UNAVAILABLE);
        assert_eq!(act_err["code"], "SIM_ACTIVATION_UNKNOWN");

        *sim_mode.lock().unwrap() = "DEGRADED_SPLIT";
        let (st_rec, rec) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "replay_camera"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st_rec, StatusCode::CONFLICT, "{rec}");
        assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
        assert_eq!(commit_phases.lock().unwrap().as_slice(), &["all".to_string()]);
    }
}

#[tokio::test]
async fn sandbox_r5_m8_commit_degraded_split_without_owner_or_matching_target_never_runs_camera_only() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for case in ["missing_owner_tx", "mismatched_target"] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let case_c = case.to_string();
        let case_s = case.to_string();
        let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let seen_tx_p = seen_tx.clone();
        let seen_tx_s = seen_tx.clone();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let seen_tx_p = seen_tx_p.clone();
                    async move {
                        *seen_tx_p.lock().unwrap() = b["tx_id"].as_str().map(str::to_string);
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        phases_c.lock().unwrap().push(b["phase"].as_str().unwrap_or("").to_string());
                        let body = if case_c == "missing_owner_tx" {
                            json!({"code": "SIM_ACTIVATION_DEGRADED", "state": "DEGRADED_SPLIT"})
                        } else {
                            json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "tx_id": b["tx_id"],
                                "scene": "pcb_suction",
                                "tool": "different_tool"
                            })
                        };
                        (StatusCode::SERVICE_UNAVAILABLE, axum::Json(body))
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let case_s = case_s.clone();
                    let seen_tx_s = seen_tx_s.clone();
                    async move {
                        let tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                        if case_s == "missing_owner_tx" {
                            axum::Json(json!({"activation_state": "DEGRADED_SPLIT"}))
                        } else {
                            axum::Json(json!({
                                "activation_state": "DEGRADED_SPLIT",
                                "tx_id": tx,
                                "scene": "pcb_suction",
                                "tool": "different_tool"
                            }))
                        }
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{case}: {err}");
        assert_eq!(err["code"], "SIM_ACTIVATION_DEGRADED", "{case}: {err}");
        assert_eq!(
            phases.lock().unwrap().as_slice(),
            &["all".to_string()],
            "{case}: camera_only must NOT run without verified owner + matching target"
        );
        assert!(matches!(
            *state.tx_state.read().await,
            ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
        ));
    }
}

#[tokio::test]
async fn sandbox_r5_m9_prepare_degraded_split_does_not_merge_mismatched_sim_state() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for case in ["different_owner_in_sim_state", "committed_in_sim_state"] {
        let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let seen_tx_p = seen_tx.clone();
        let seen_tx_s = seen_tx.clone();
        let case_p = case.to_string();
        let case_s = case.to_string();
        let prep_payload: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let pp_p = prep_payload.clone();
        let pp_s = prep_payload.clone();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let seen_tx_p = seen_tx_p.clone();
                    let case_p = case_p.clone();
                    let pp_p = pp_p.clone();
                    async move {
                        let tx = b["tx_id"].as_str().unwrap().to_string();
                        *seen_tx_p.lock().unwrap() = Some(tx.clone());
                        *pp_p.lock().unwrap() = Some(b);
                        let body = if case_p == "different_owner_in_sim_state" {
                            // Prepare reports tx as owner, omits scene/tool
                            json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "tx_id": tx
                            })
                        } else {
                            // Prepare omits tx_id/scene/tool
                            json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT"
                            })
                        };
                        (StatusCode::SERVICE_UNAVAILABLE, axum::Json(body))
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let case_s = case_s.clone();
                    let seen_tx_s = seen_tx_s.clone();
                    let pp_s = pp_s.clone();
                    async move {
                        let tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                        let prep = pp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        if case_s == "different_owner_in_sim_state" {
                            // Different owner_tx reports matching target fields — must NOT merge onto prepare's tx!
                            axum::Json(json!({
                                "activation_state": "DEGRADED_SPLIT",
                                "tx_id": "foreign-owner-B",
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            }))
                        } else {
                            // COMMITTED state — must NOT be merged onto prepare's DEGRADED_SPLIT!
                            axum::Json(json!({
                                "activation_state": "COMMITTED",
                                "tx_id": tx,
                                "robot": "b601dm",
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            }))
                        }
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{case}: {err}");
        assert_eq!(err["code"], "SIM_ACTIVATION_DEGRADED", "{case}: {err}");
        match state.tx_state.read().await.clone() {
            ActivationTxState::DegradedSplit { tx_id, target_matches_tx, target_composed_mjcf, .. } => {
                assert!(!target_matches_tx, "{case}: mismatched /sim/state must NOT verify target_matches_tx");
                assert!(target_composed_mjcf.is_empty(), "{case}: MJCF must remain empty");
                if case == "committed_in_sim_state" {
                    assert_eq!(tx_id, None, "COMMITTED /sim/state must NOT supply owner tx_id");
                }
            }
            other => panic!("{case}: expected DegradedSplit, got {other:?}"),
        };
    }
}

#[tokio::test]
async fn sandbox_r5_m10_unknown_owner_degraded_split_stores_none_and_recover_split_discovers_owner() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let state_ready = Arc::new(Mutex::new(false));
    let sr_s = state_ready.clone();
    let prep_seen: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let ps_p = prep_seen.clone();
    let ps_s = prep_seen.clone();
    let ps_c = prep_seen.clone();
    let commit_reqs: Arc<Mutex<Vec<Value>>> = Arc::new(Mutex::new(Vec::new()));
    let cr_c = commit_reqs.clone();

    let mock_sim = axum::Router::new()
        .route(
            "/sim/scene/prepare",
            post(move |axum::Json(b): axum::Json<Value>| {
                let ps_p = ps_p.clone();
                async move {
                    *ps_p.lock().unwrap() = Some(b);
                    (
                        StatusCode::SERVICE_UNAVAILABLE,
                        axum::Json(json!({"code": "SIM_ACTIVATION_DEGRADED", "state": "DEGRADED_SPLIT"})),
                    )
                }
            }),
        )
        .route(
            "/sim/state",
            get(move || {
                let sr_s = sr_s.clone();
                let ps_s = ps_s.clone();
                async move {
                    if !*sr_s.lock().unwrap() {
                        (StatusCode::SERVICE_UNAVAILABLE, axum::Json(json!({"error": "starting"})))
                    } else {
                        let prep = ps_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        (
                            StatusCode::OK,
                            axum::Json(json!({
                                "activation_state": "DEGRADED_SPLIT",
                                "tx_id": "real-remote-owner-77",
                                "robot": "b601dm",
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })),
                        )
                    }
                }
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move |axum::Json(b): axum::Json<Value>| {
                let cr_c = cr_c.clone();
                let ps_c = ps_c.clone();
                async move {
                    cr_c.lock().unwrap().push(b.clone());
                    let prep = ps_c.lock().unwrap().clone().unwrap_or(Value::Null);
                    axum::Json(json!({
                        "ok": true,
                        "state": "COMMITTED",
                        "tx_id": b["tx_id"],
                        "robot": "b601dm",
                        "scene_digest": prep["scene_digest"],
                        "tool_digest": prep["tool_digest"]
                    }))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{err}");
    assert!(err["tx_id"].is_null(), "unknown split tx_id must serialize as null, got {}", err["tx_id"]);
    assert!(matches!(
        *state.tx_state.read().await,
        ActivationTxState::DegradedSplit { tx_id: None, target_matches_tx: false, .. }
    ));

    *state_ready.lock().unwrap() = true;
    let (st_rec, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st_rec, StatusCode::OK, "{rec}");
    assert_eq!(rec["state"], "COMMITTED", "{rec}");
    assert_eq!(rec["tx_id"], "real-remote-owner-77", "{rec}");
    let commits = commit_reqs.lock().unwrap();
    assert_eq!(commits.len(), 1);
    assert_eq!(commits[0]["tx_id"], "real-remote-owner-77");
    assert_eq!(commits[0]["phase"], "camera_only");
}

#[tokio::test]
async fn sandbox_r5_m11_lost_commit_ack_and_reconcile_unknown_require_matching_robot() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for case in ["reconcile_lost_ack_wrong_robot", "reconcile_unknown_missing_robot"] {
        let seen_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let sp_p = seen_prep.clone();
        let sp_s = seen_prep.clone();
        let sim_phase = Arc::new(Mutex::new("COMMITTING"));
        let sph_s = sim_phase.clone();
        let case_c = case.to_string();
        let case_s = case.to_string();

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move {
            loop {
                let Ok((mut sock, _)) = listener.accept().await else { break };
                let sp_p = sp_p.clone();
                let sp_s = sp_s.clone();
                let sph_s = sph_s.clone();
                let case_c = case_c.clone();
                let case_s = case_s.clone();
                tokio::spawn(async move {
                    let Some((first_line, parsed)) = read_http_request(&mut sock).await else { return };
                    if first_line.starts_with("POST /sim/scene/prepare ") {
                        *sp_p.lock().unwrap() = Some(parsed.clone());
                        let body = json!({
                            "ok": true, "state": "PREPARED", "tx_id": parsed["tx_id"], "robot": "b601dm"
                        }).to_string();
                        let resp = format!("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                        let _ = sock.write_all(resp.as_bytes()).await;
                    } else if first_line.starts_with("POST /sim/scene/commit ") {
                        if case_c == "reconcile_lost_ack_wrong_robot" {
                            drop(sock);
                        } else {
                            let body = json!({"error": "timeout"}).to_string();
                            let resp = format!("HTTP/1.1 504 Gateway Timeout\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                            let _ = sock.write_all(resp.as_bytes()).await;
                        }
                    } else if first_line.starts_with("GET /sim/state ") {
                        let prep = sp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        let body = if case_s == "reconcile_unknown_missing_robot" && *sph_s.lock().unwrap() == "COMMITTING" {
                            json!({"tx_id": prep["tx_id"], "activation_state": "COMMITTING", "robot": "b601dm"})
                        } else if case_s == "reconcile_unknown_missing_robot" {
                            json!({
                                "tx_id": prep["tx_id"],
                                "activation_state": "COMMITTED",
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })
                        } else {
                            json!({
                                "tx_id": prep["tx_id"],
                                "activation_state": "COMMITTED",
                                "robot": "so-arm101",
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            })
                        }
                        .to_string();
                        let resp = format!("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                        let _ = sock.write_all(resp.as_bytes()).await;
                    }
                });
            }
        });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;

        if case == "reconcile_lost_ack_wrong_robot" {
            std::env::remove_var("ARMD_SIM_URL");
            server.abort();
            assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
            assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
            assert_eq!(state.active.read().await.mode, "none");
        } else {
            assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{err}");
            assert_eq!(err["code"], "SIM_ACTIVATION_UNKNOWN", "{err}");
            *sim_phase.lock().unwrap() = "COMMITTED";
            let (st2, err2) = send_json(
                &app,
                "POST",
                "/api/v1/sandbox/activate",
                Some(json!({"mode": "wasm", "scene": "default", "tool": "default"})),
            )
            .await;
            std::env::remove_var("ARMD_SIM_URL");
            server.abort();
            assert_eq!(st2, StatusCode::BAD_GATEWAY, "{err2}");
            assert_eq!(err2["code"], "SIM_RECOVERY_INCOMPATIBLE", "{err2}");
            assert_eq!(state.active.read().await.mode, "none");
        }
    }
}

#[tokio::test]
async fn sandbox_r5_m1_non_string_digest_rejected_everywhere() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    let commit_called = Arc::new(Mutex::new(false));
    let commit_c = commit_called.clone();
    let mock_sim = axum::Router::new()
        .route(
            "/sim/state",
            get(|| async {
                axum::Json(json!({
                    "tx_id": "split-bad-digest-type",
                    "activation_state": "DEGRADED_SPLIT",
                    "robot": "b601dm",
                    "scene": "default",
                    "tool": "default",
                    "scene_digest": serde_json::Value::Null,
                    "tool_digest": 12345
                }))
            }),
        )
        .route(
            "/sim/scene/commit",
            post(move || {
                let commit_c = commit_c.clone();
                async move {
                    *commit_c.lock().unwrap() = true;
                    axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                }
            }),
        );

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st_rec, rec) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/recover-split",
        Some(json!({"action": "replay_camera"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st_rec, StatusCode::CONFLICT, "{rec}");
    assert_eq!(rec["code"], "SIM_SPLIT_RECOVERY_REJECTED", "{rec}");
    assert!(!*commit_called.lock().unwrap());
}

#[tokio::test]
async fn sandbox_r6_m1_conflicting_split_observations_never_adopt_mixed_owner_and_target() {
    let _env_guard = SIM_ENV_LOCK.lock().await;
    for case in ["converge_conflict", "prepare_conflict"] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let seen_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let seen_tx_p = seen_tx.clone();
        let seen_tx_s = seen_tx.clone();
        let case_p = case.to_string();
        let case_c = case.to_string();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let seen_tx_p = seen_tx_p.clone();
                    let case_p = case_p.clone();
                    async move {
                        let tx = b["tx_id"].as_str().unwrap().to_string();
                        *seen_tx_p.lock().unwrap() = Some(tx.clone());
                        if case_p == "prepare_conflict" {
                            // Prepare omits tx_id/tool, claims scene=pcb_suction (local target), while /sim/state has conflicting scene!
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "scene": "pcb_suction"
                                })),
                            )
                        } else {
                            (
                                StatusCode::OK,
                                axum::Json(json!({
                                    "ok": true,
                                    "state": "PREPARED",
                                    "tx_id": tx,
                                    "robot": "b601dm"
                                })),
                            )
                        }
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        phases_c.lock().unwrap().push(b["phase"].as_str().unwrap_or("").to_string());
                        let _ = case_c;
                        // Commit omits tx_id/tool, claims scene=pcb_suction while /sim/state has conflicting scene!
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "scene": "pcb_suction"
                            })),
                        )
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let seen_tx_s = seen_tx_s.clone();
                    async move {
                        let tx = seen_tx_s.lock().unwrap().clone().unwrap_or_default();
                        axum::Json(json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": tx,
                            "robot": "b601dm",
                            "scene": "other_conflicting_scene",
                            "tool": "suction"
                        }))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());

        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{case}: {err}");
        assert_eq!(err["code"], "SIM_ACTIVATION_DEGRADED", "{case}: {err}");
        if case == "converge_conflict" {
            assert_eq!(
                phases.lock().unwrap().as_slice(),
                &["all".to_string()],
                "conflicting /sim/state must NOT converge via camera_only"
            );
        }
        match state.tx_state.read().await.clone() {
            ActivationTxState::DegradedSplit { tx_id, target_matches_tx, .. } => {
                assert!(!target_matches_tx, "{case}: conflicting observations must not set target_matches_tx=true");
                assert_eq!(tx_id, None, "{case}: conflicting /sim/state must not adopt mixed owner tx_id");
            }
            other => panic!("{case}: expected DegradedSplit, got {other:?}"),
        };
    }
}

#[tokio::test]
async fn sandbox_r6_m2_degraded_split_recovery_requires_matching_robot_before_commit_posts() {
    let _env_guard = SIM_ENV_LOCK.lock().await;

    // 1. Lost commit ACK -> /sim/state reports DEGRADED_SPLIT for same tx_id & target, but wrong robot ("so-arm101").
    // Must fail BEFORE sending camera_only or rollback_physics POSTs!
    let seen_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let sp_p = seen_prep.clone();
    let sp_s = seen_prep.clone();
    let commit_phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let cp_c = commit_phases.clone();

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = listener.accept().await else { break };
            let sp_p = sp_p.clone();
            let sp_s = sp_s.clone();
            let cp_c = cp_c.clone();
            tokio::spawn(async move {
                let Some((first_line, parsed)) = read_http_request(&mut sock).await else { return };
                if first_line.starts_with("POST /sim/scene/prepare ") {
                    *sp_p.lock().unwrap() = Some(parsed.clone());
                    let body = json!({
                        "ok": true, "state": "PREPARED", "tx_id": parsed["tx_id"], "robot": "b601dm"
                    }).to_string();
                    let resp = format!("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                    let _ = sock.write_all(resp.as_bytes()).await;
                } else if first_line.starts_with("POST /sim/scene/commit ") {
                    cp_c.lock().unwrap().push(parsed["phase"].as_str().unwrap_or("").to_string());
                    if parsed["phase"] == "all" {
                        drop(sock);
                    } else {
                        let body = json!({"ok": true, "state": "COMMITTED"}).to_string();
                        let resp = format!("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                        let _ = sock.write_all(resp.as_bytes()).await;
                    }
                } else if first_line.starts_with("GET /sim/state ") {
                    let prep = sp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                    let body = json!({
                        "activation_state": "DEGRADED_SPLIT",
                        "tx_id": prep["tx_id"],
                        "robot": "so-arm101",
                        "scene": prep["scene"],
                        "tool": prep["tool"],
                        "scene_digest": prep["scene_digest"],
                        "tool_digest": prep["tool_digest"]
                    }).to_string();
                    let resp = format!("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
                    let _ = sock.write_all(resp.as_bytes()).await;
                }
            });
        }
    });
    std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let state = Arc::new(
        SandboxState::new(tmp.path().to_path_buf())
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());
    let (st, err) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
    )
    .await;
    std::env::remove_var("ARMD_SIM_URL");
    server.abort();

    assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
    assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
    assert_eq!(
        commit_phases.lock().unwrap().as_slice(),
        &["all".to_string()],
        "lost-ACK DEGRADED_SPLIT with wrong robot must NOT send camera_only or rollback_physics"
    );

    // 2. Manual recover_split rediscovery when /sim/state reports DEGRADED_SPLIT with wrong or missing robot
    // must reject BOTH replay_camera and rollback_physics BEFORE sending any /sim/scene/commit POST.
    for robot_val in [json!("so-arm101"), Value::Null] {
        let rec_commits = Arc::new(Mutex::new(0usize));
        let rc_c = rec_commits.clone();
        let rv = robot_val.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(move || {
                    let rv = rv.clone();
                    async move {
                        let mut body = json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": "split-wrong-robot-1",
                            "scene": "default",
                            "tool": "default",
                            "scene_digest": Value::Null,
                            "tool_digest": Value::Null
                        });
                        if !rv.is_null() {
                            body["robot"] = rv;
                        }
                        axum::Json(body)
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let rc_c = rc_c.clone();
                    async move {
                        *rc_c.lock().unwrap() += 1;
                        axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let rec_tmp = tempfile::tempdir().unwrap();
        let rec_empty = tempfile::tempdir().unwrap();
        let rec_state = Arc::new(
            SandboxState::new(rec_tmp.path().to_path_buf())
                .with_builtin_roots(vec![rec_empty.path().to_path_buf()]),
        );
        let rec_app = app_for(ArmConfig::rebot_dm(), rec_state);

        for action in ["replay_camera", "rollback_physics"] {
            let (st_rec, rec) = send_json(
                &rec_app,
                "POST",
                "/api/v1/sandbox/recover-split",
                Some(json!({"action": action})),
            )
            .await;
            if robot_val.is_null() {
                assert_eq!(st_rec, StatusCode::BAD_GATEWAY, "{action}: {rec}");
                assert_eq!(rec["code"], "SIM_RECOVERY_INCOMPATIBLE", "{action}: {rec}");
            } else {
                assert_eq!(st_rec, StatusCode::BAD_REQUEST, "{action}: {rec}");
                assert_eq!(rec["code"], "ROBOT_INCOMPATIBLE", "{action}: {rec}");
            }
        }
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();
        assert_eq!(*rec_commits.lock().unwrap(), 0, "no commit POST allowed on robot mismatch");
    }
}

#[tokio::test]
async fn sandbox_r7_m1_m2_m3_m4_split_null_conflict_robot_normalization_and_state_transition() {
    let _env_guard = SIM_ENV_LOCK.lock().await;

    // 1. M1 + m2: explicit null scene_digest in 503 vs non-null in /sim/state rejects conflict;
    // robot alias "rebot-dm" vs "b601dm" is compatible and converges.
    for case in ["explicit_null_conflict", "robot_alias_match"] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let lp_p = last_prep.clone();
        let lp_c = last_prep.clone();
        let lp_s = last_prep.clone();
        let case_c = case.to_string();
        let case_s = case.to_string();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let lp_p = lp_p.clone();
                    async move {
                        *lp_p.lock().unwrap() = Some(b.clone());
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let lp_c = lp_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        let phase = b["phase"].as_str().unwrap_or("").to_string();
                        phases_c.lock().unwrap().push(phase.clone());
                        let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                        if phase == "all" {
                            let body = if case_c == "explicit_null_conflict" {
                                // Missing `tool` triggers /sim/state merge, and explicit `scene_digest: null`
                                // must conflict with /sim/state's non-null digest!
                                json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": b["tx_id"],
                                    "robot": "b601dm",
                                    "scene": prep["scene"],
                                    "scene_digest": Value::Null,
                                    "tool_digest": prep["tool_digest"]
                                })
                            } else {
                                // Alias robot "rebot-dm" + missing tool -> merges /sim/state "b601dm" + tool!
                                json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": b["tx_id"],
                                    "robot": "rebot-dm",
                                    "scene": prep["scene"]
                                })
                            };
                            (StatusCode::SERVICE_UNAVAILABLE, axum::Json(body))
                        } else {
                            (
                                StatusCode::OK,
                                axum::Json(json!({
                                    "ok": true,
                                    "state": "COMMITTED",
                                    "tx_id": b["tx_id"],
                                    "robot": "b601dm",
                                    "scene_digest": prep["scene_digest"],
                                    "tool_digest": prep["tool_digest"]
                                })),
                            )
                        }
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let lp_s = lp_s.clone();
                    let case_s = case_s.clone();
                    async move {
                        let prep = lp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        let scene_digest = if case_s == "explicit_null_conflict" {
                            json!("sha256:conflicting_digest")
                        } else {
                            prep["scene_digest"].clone()
                        };
                        axum::Json(json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": prep["tx_id"],
                            "robot": "b601dm",
                            "scene": prep["scene"],
                            "tool": prep["tool"],
                            "scene_digest": scene_digest,
                            "tool_digest": prep["tool_digest"]
                        }))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, resp) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        if case == "explicit_null_conflict" {
            assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{resp}");
            assert_eq!(phases.lock().unwrap().as_slice(), &["all".to_string()]);
            assert!(matches!(
                *state.tx_state.read().await,
                ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
            ));
        } else {
            assert_eq!(st, StatusCode::OK, "{resp}");
            assert_eq!(phases.lock().unwrap().as_slice(), &["all".to_string(), "camera_only".to_string()]);
        }
    }

    // 2. M2 + M4:
    // (a) Direct commit 503 omitting `robot` queries /sim/state and rejects mismatched robot BEFORE camera_only,
    //     transitioning local state out of Committing -> DegradedSplit { target_matches_tx: false }.
    // (b) When camera_only ACK returns 200 OK with wrong robot, rejects directly WITHOUT falling back to rollback_physics.
    for case in ["missing_robot_in_503_then_wrong_in_state", "camera_only_ack_wrong_robot"] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let lp_p = last_prep.clone();
        let lp_c = last_prep.clone();
        let lp_s = last_prep.clone();
        let case_c = case.to_string();

        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let lp_p = lp_p.clone();
                    async move {
                        *lp_p.lock().unwrap() = Some(b.clone());
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let lp_c = lp_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        let phase = b["phase"].as_str().unwrap_or("").to_string();
                        phases_c.lock().unwrap().push(phase.clone());
                        let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                        if phase == "all" {
                            let mut body = json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "tx_id": b["tx_id"],
                                "scene": prep["scene"],
                                "tool": prep["tool"],
                                "scene_digest": prep["scene_digest"],
                                "tool_digest": prep["tool_digest"]
                            });
                            if case_c == "camera_only_ack_wrong_robot" {
                                body["robot"] = json!("b601dm");
                            }
                            (StatusCode::SERVICE_UNAVAILABLE, axum::Json(body))
                        } else {
                            (
                                StatusCode::OK,
                                axum::Json(json!({
                                    "ok": true,
                                    "state": "COMMITTED",
                                    "tx_id": b["tx_id"],
                                    "robot": "so-arm101",
                                    "scene_digest": prep["scene_digest"],
                                    "tool_digest": prep["tool_digest"]
                                })),
                            )
                        }
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let lp_s = lp_s.clone();
                    async move {
                        let prep = lp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        axum::Json(json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": prep["tx_id"],
                            "robot": "so-arm101",
                            "scene": prep["scene"],
                            "tool": prep["tool"],
                            "scene_digest": prep["scene_digest"],
                            "tool_digest": prep["tool_digest"]
                        }))
                    }
                }),
            );

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::BAD_REQUEST, "{case}: {err}");
        assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{case}: {err}");
        if case == "missing_robot_in_503_then_wrong_in_state" {
            assert_eq!(phases.lock().unwrap().as_slice(), &["all".to_string()]);
        } else {
            assert_eq!(
                phases.lock().unwrap().as_slice(),
                &["all".to_string(), "camera_only".to_string()],
                "wrong robot on camera_only ACK must NOT fall back to rollback_physics"
            );
        }
        assert!(
            matches!(
                *state.tx_state.read().await,
                ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
            ),
            "{case}: local tx_state must transition out of Committing into DegradedSplit"
        );
    }

    // 3. M3: prepare-503 with mismatched robot must NOT cache target_matches_tx=true, and manual recover_split
    //    must block rollback_physics too before posting /sim/scene/commit.
    {
        let commit_called = Arc::new(Mutex::new(false));
        let cc = commit_called.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(|axum::Json(b): axum::Json<Value>| async move {
                    (
                        StatusCode::SERVICE_UNAVAILABLE,
                        axum::Json(json!({
                            "code": "SIM_ACTIVATION_DEGRADED",
                            "state": "DEGRADED_SPLIT",
                            "tx_id": b["tx_id"],
                            "robot": "so-arm101",
                            "scene": b["scene"],
                            "tool": b["tool"],
                            "scene_digest": b["scene_digest"],
                            "tool_digest": b["tool_digest"]
                        })),
                    )
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() = true;
                        axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, _) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
        assert!(matches!(
            *state.tx_state.read().await,
            ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
        ));
        let (st_rec, rec_err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "rollback_physics"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();
        assert_eq!(st_rec, StatusCode::BAD_REQUEST, "{rec_err}");
        assert_eq!(rec_err["code"], "ROBOT_INCOMPATIBLE", "{rec_err}");
        assert!(!*commit_called.lock().unwrap());
    }
}

#[tokio::test]
async fn sandbox_r8_m1_m2_prepare_omitted_robot_plain_detail_and_camera_only_503_guards() {
    let _env_guard = SIM_ENV_LOCK.lock().await;

    // 1. M1(a): prepare 503 omitting `robot` (and /sim/state also omitting `robot`) must NOT set
    // target_matches_tx=true; subsequent recover_split (replay_camera & rollback_physics) must fail
    // BEFORE sending any /sim/scene/commit POST.
    {
        let commit_called = Arc::new(Mutex::new(false));
        let cc = commit_called.clone();
        let last_tx: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let lt_p = last_tx.clone();
        let lt_s = last_tx.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let lt_p = lt_p.clone();
                    async move {
                        *lt_p.lock().unwrap() = b["tx_id"].as_str().map(str::to_string);
                        (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(json!({
                                "code": "SIM_ACTIVATION_DEGRADED",
                                "state": "DEGRADED_SPLIT",
                                "tx_id": b["tx_id"],
                                "scene": b["scene"],
                                "tool": b["tool"],
                                "scene_digest": b["scene_digest"],
                                "tool_digest": b["tool_digest"]
                            })),
                        )
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let lt_s = lt_s.clone();
                    async move {
                        let tx = lt_s.lock().unwrap().clone();
                        axum::Json(json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": tx,
                            "scene": "pcb_suction",
                            "tool": "suction"
                        }))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() = true;
                        axum::Json(json!({"ok": true, "state": "COMMITTED"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, _) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
        assert!(matches!(
            *state.tx_state.read().await,
            ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
        ));
        for action in ["replay_camera", "rollback_physics"] {
            let (st_rec, rec) = send_json(
                &app,
                "POST",
                "/api/v1/sandbox/recover-split",
                Some(json!({"action": action})),
            )
            .await;
            assert!(st_rec == StatusCode::CONFLICT || st_rec == StatusCode::BAD_GATEWAY, "{action}: {rec}");
        }
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();
        assert!(!*commit_called.lock().unwrap(), "M1: prepare 503 missing robot must block commit POST");
    }

    // 2. M1(b): known-owner DegradedSplit with plain non-JSON `detail` must positively verify `robot`
    // via /sim/state before rollback_physics (and fail closed if /sim/state lacks/mismatches robot).
    for sim_robot in [None, Some("b601dm")] {
        let commit_called = Arc::new(Mutex::new(false));
        let cc = commit_called.clone();
        let sr = sim_robot.map(str::to_string);
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(move || {
                    let sr = sr.clone();
                    async move {
                        let mut body = json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": "known-plain-tx"
                        });
                        if let Some(r) = sr {
                            body["robot"] = json!(r);
                        }
                        axum::Json(body)
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() = true;
                        axum::Json(json!({
                            "ok": true,
                            "state": "ABORTED",
                            "tx_id": b["tx_id"],
                            "restored_tx_id": "prev-tx-1"
                        }))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        {
            let mut tx_w = state.tx_state.write().await;
            *tx_w = ActivationTxState::DegradedSplit {
                tx_id: Some("known-plain-tx".into()),
                prev_tx_id: Some("prev-tx-1".into()),
                prev_scene: "default".into(),
                prev_tool: "default".into(),
                prev_policy: None,
                prev_mode: "wasm".into(),
                prev_seed: 42,
                prev_auto_start_policy: false,
                prev_cached_mjcf: String::new(),
                prev_scene_digest: None,
                prev_tool_digest: None,
                target_scene: "pcb_suction".into(),
                target_tool: "suction".into(),
                target_policy: None,
                target_seed: 42,
                target_auto_start_policy: false,
                target_composed_mjcf: String::new(),
                target_scene_digest: None,
                target_tool_digest: None,
                target_matches_tx: true,
                detail: "plain-text degraded split detail without json".into(),
            };
        }
        let app = app_for(ArmConfig::rebot_dm(), state);
        let (st, resp) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "rollback_physics"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();
        if sim_robot.is_none() {
            assert_eq!(st, StatusCode::BAD_GATEWAY, "{resp}");
            assert_eq!(resp["code"], "SIM_RECOVERY_INCOMPATIBLE", "{resp}");
            assert!(!*commit_called.lock().unwrap(), "plain-text detail without robot must NOT rollback");
        } else {
            assert_eq!(st, StatusCode::OK, "{resp}");
            assert!(*commit_called.lock().unwrap());
        }
    }

    // 3. M2: camera_only returning 503 DEGRADED_SPLIT with wrong robot or foreign tx_id must fail
    // immediately WITHOUT falling back to rollback_physics.
    for case in ["camera_only_503_wrong_robot", "camera_only_503_foreign_owner"] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let lp_p = last_prep.clone();
        let lp_c = last_prep.clone();
        let case_c = case.to_string();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let lp_p = lp_p.clone();
                    async move {
                        *lp_p.lock().unwrap() = Some(b.clone());
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let lp_c = lp_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        let phase = b["phase"].as_str().unwrap_or("").to_string();
                        phases_c.lock().unwrap().push(phase.clone());
                        let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                        if phase == "all" {
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": b["tx_id"],
                                    "robot": "b601dm",
                                    "scene": prep["scene"],
                                    "tool": prep["tool"],
                                    "scene_digest": prep["scene_digest"],
                                    "tool_digest": prep["tool_digest"]
                                })),
                            )
                        } else if phase == "camera_only" {
                            let (robot, owner_tx) = if case_c == "camera_only_503_wrong_robot" {
                                ("so-arm101", b["tx_id"].clone())
                            } else {
                                ("b601dm", json!("foreign-owner-tx"))
                            };
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": owner_tx,
                                    "robot": robot,
                                    "scene": prep["scene"],
                                    "tool": prep["tool"]
                                })),
                            )
                        } else {
                            (
                                StatusCode::OK,
                                axum::Json(json!({
                                    "ok": true,
                                    "state": "ROLLED_BACK",
                                    "tx_id": b["tx_id"]
                                })),
                            )
                        }
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, err) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(
            phases.lock().unwrap().as_slice(),
            &["all".to_string(), "camera_only".to_string()],
            "{case}: must NOT fall back to rollback_physics ({err})"
        );
        if case == "camera_only_503_wrong_robot" {
            assert_eq!(st, StatusCode::BAD_REQUEST, "{err}");
            assert_eq!(err["code"], "ROBOT_INCOMPATIBLE", "{err}");
        } else {
            assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{err}");
            assert_eq!(err["code"], "SIM_ACTIVATION_DEGRADED", "{err}");
            assert_eq!(err["tx_id"], "foreign-owner-tx", "{err}");
        }
        assert!(matches!(
            *state.tx_state.read().await,
            ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
        ));
    }
}

#[tokio::test]
async fn sandbox_r9_m1_m2_m3_m1_split_owner_target_state_and_robot_null_guards() {
    let _env_guard = SIM_ENV_LOCK.lock().await;

    // 1. M1: known-owner recover_split querying /sim/state for missing robot must reject when
    // /sim/state returns COMMITTED (not DEGRADED_SPLIT) or omits tx_id.
    for (state_str, include_tx) in [("COMMITTED", true), ("DEGRADED_SPLIT", false)] {
        let commit_called = Arc::new(Mutex::new(false));
        let cc = commit_called.clone();
        let st_s = state_str.to_string();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(move || {
                    let st_s = st_s.clone();
                    async move {
                        let mut body = json!({
                            "activation_state": st_s,
                            "robot": "b601dm"
                        });
                        if include_tx {
                            body["tx_id"] = json!("known-plain-tx-r9");
                        }
                        axum::Json(body)
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() = true;
                        axum::Json(json!({"ok": true, "state": "ABORTED", "tx_id": "known-plain-tx-r9"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        {
            let mut tx_w = state.tx_state.write().await;
            *tx_w = ActivationTxState::DegradedSplit {
                tx_id: Some("known-plain-tx-r9".into()),
                prev_tx_id: Some("prev-tx-1".into()),
                prev_scene: "default".into(),
                prev_tool: "default".into(),
                prev_policy: None,
                prev_mode: "wasm".into(),
                prev_seed: 42,
                prev_auto_start_policy: false,
                prev_cached_mjcf: String::new(),
                prev_scene_digest: None,
                prev_tool_digest: None,
                target_scene: "pcb_suction".into(),
                target_tool: "suction".into(),
                target_policy: None,
                target_seed: 42,
                target_auto_start_policy: false,
                target_composed_mjcf: String::new(),
                target_scene_digest: None,
                target_tool_digest: None,
                target_matches_tx: true,
                detail: "plain-text detail".into(),
            };
        }
        let app = app_for(ArmConfig::rebot_dm(), state);
        let (st, resp) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "rollback_physics"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();
        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{state_str}/{include_tx}: {resp}");
        assert!(!*commit_called.lock().unwrap(), "must NOT post rollback_physics without DEGRADED_SPLIT + matching tx_id");
    }

    // 2. M2 + M3 + m1:
    // - M2: camera_only 503 with (a) tx_id=null + degraded_tx_id="foreign-tx", or
    //       (b) tx_id=local + degraded_tx_id="foreign-tx" aborts WITHOUT rollback_physics.
    // - M3: camera_only 503 with mismatched scene/tool metadata aborts WITHOUT rollback_physics.
    // - m1: commit 503 with robot: null merges real robot from /sim/state and converges via camera_only!
    for case in [
        "cam503_null_tx_foreign_degraded_tx",
        "cam503_conflicting_tx_and_degraded_tx",
        "cam503_conflicting_target_tool",
        "commit503_robot_null_merges_from_sim_state",
    ] {
        let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let phases_c = phases.clone();
        let last_prep: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
        let lp_p = last_prep.clone();
        let lp_c = last_prep.clone();
        let lp_s = last_prep.clone();
        let case_c = case.to_string();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/scene/prepare",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let lp_p = lp_p.clone();
                    async move {
                        *lp_p.lock().unwrap() = Some(b.clone());
                        axum::Json(json!({"ok": true, "state": "PREPARED", "tx_id": b["tx_id"], "robot": "b601dm"}))
                    }
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move |axum::Json(b): axum::Json<Value>| {
                    let phases_c = phases_c.clone();
                    let lp_c = lp_c.clone();
                    let case_c = case_c.clone();
                    async move {
                        let phase = b["phase"].as_str().unwrap_or("").to_string();
                        phases_c.lock().unwrap().push(phase.clone());
                        let prep = lp_c.lock().unwrap().clone().unwrap_or(Value::Null);
                        if phase == "all" {
                            let robot_val = if case_c == "commit503_robot_null_merges_from_sim_state" {
                                Value::Null
                            } else {
                                json!("b601dm")
                            };
                            (
                                StatusCode::SERVICE_UNAVAILABLE,
                                axum::Json(json!({
                                    "code": "SIM_ACTIVATION_DEGRADED",
                                    "state": "DEGRADED_SPLIT",
                                    "tx_id": b["tx_id"],
                                    "robot": robot_val,
                                    "scene": prep["scene"],
                                    "tool": prep["tool"],
                                    "scene_digest": prep["scene_digest"],
                                    "tool_digest": prep["tool_digest"]
                                })),
                            )
                        } else if phase == "camera_only" {
                            match case_c.as_str() {
                                "cam503_null_tx_foreign_degraded_tx" => (
                                    StatusCode::SERVICE_UNAVAILABLE,
                                    axum::Json(json!({
                                        "code": "SIM_ACTIVATION_DEGRADED",
                                        "state": "DEGRADED_SPLIT",
                                        "tx_id": Value::Null,
                                        "degraded_tx_id": "foreign-degraded-tx",
                                        "robot": "b601dm"
                                    })),
                                ),
                                "cam503_conflicting_tx_and_degraded_tx" => (
                                    StatusCode::SERVICE_UNAVAILABLE,
                                    axum::Json(json!({
                                        "code": "SIM_ACTIVATION_DEGRADED",
                                        "state": "DEGRADED_SPLIT",
                                        "tx_id": b["tx_id"],
                                        "degraded_tx_id": "foreign-degraded-tx",
                                        "robot": "b601dm"
                                    })),
                                ),
                                "cam503_conflicting_target_tool" => (
                                    StatusCode::SERVICE_UNAVAILABLE,
                                    axum::Json(json!({
                                        "code": "SIM_ACTIVATION_DEGRADED",
                                        "state": "DEGRADED_SPLIT",
                                        "tx_id": b["tx_id"],
                                        "robot": "b601dm",
                                        "scene": prep["scene"],
                                        "tool": "other_wrong_tool"
                                    })),
                                ),
                                _ => (
                                    StatusCode::OK,
                                    axum::Json(json!({
                                        "ok": true,
                                        "state": "COMMITTED",
                                        "tx_id": b["tx_id"],
                                        "robot": "b601dm",
                                        "scene_digest": prep["scene_digest"],
                                        "tool_digest": prep["tool_digest"]
                                    })),
                                ),
                            }
                        } else {
                            (
                                StatusCode::OK,
                                axum::Json(json!({"ok": true, "state": "ABORTED", "tx_id": b["tx_id"]})),
                            )
                        }
                    }
                }),
            )
            .route(
                "/sim/state",
                get(move || {
                    let lp_s = lp_s.clone();
                    async move {
                        let prep = lp_s.lock().unwrap().clone().unwrap_or(Value::Null);
                        axum::Json(json!({
                            "activation_state": "DEGRADED_SPLIT",
                            "tx_id": prep["tx_id"],
                            "robot": "b601dm",
                            "scene": prep["scene"],
                            "tool": prep["tool"],
                            "scene_digest": prep["scene_digest"],
                            "tool_digest": prep["tool_digest"]
                        }))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state.clone());
        let (st, resp) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/activate",
            Some(json!({"mode": "sim", "scene": "pcb_suction", "tool": "suction"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(
            phases.lock().unwrap().as_slice(),
            &["all".to_string(), "camera_only".to_string()],
            "{case}: {resp}"
        );
        if case == "commit503_robot_null_merges_from_sim_state" {
            assert_eq!(st, StatusCode::OK, "{resp}");
        } else {
            assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE, "{case}: {resp}");
            assert!(matches!(
                *state.tx_state.read().await,
                ActivationTxState::DegradedSplit { target_matches_tx: false, .. }
            ));
        }
    }

    // 3. Restart / rediscovery in Idle state: /sim/state returning conflicting
    // `tx_id: "A"` and `degraded_tx_id: "B"` must reject rollback_physics with 0 commit POSTs.
    {
        let commit_calls = Arc::new(Mutex::new(0usize));
        let cc = commit_calls.clone();
        let mock_sim = axum::Router::new()
            .route(
                "/sim/state",
                get(|| async {
                    axum::Json(json!({
                        "activation_state": "DEGRADED_SPLIT",
                        "tx_id": "A",
                        "degraded_tx_id": "B",
                        "robot": "b601dm",
                        "scene": "pcb_suction",
                        "tool": "suction"
                    }))
                }),
            )
            .route(
                "/sim/scene/commit",
                post(move || {
                    let cc = cc.clone();
                    async move {
                        *cc.lock().unwrap() += 1;
                        axum::Json(json!({"ok": true, "state": "ABORTED"}))
                    }
                }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = tokio::spawn(async move { axum::serve(listener, mock_sim).await.unwrap() });
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = app_for(ArmConfig::rebot_dm(), state);
        let (st, resp) = send_json(
            &app,
            "POST",
            "/api/v1/sandbox/recover-split",
            Some(json!({"action": "rollback_physics"})),
        )
        .await;
        std::env::remove_var("ARMD_SIM_URL");
        server.abort();

        assert_eq!(st, StatusCode::BAD_GATEWAY, "{resp}");
        assert_eq!(resp["code"], "SIM_STATE_INVALID", "{resp}");
        assert_eq!(*commit_calls.lock().unwrap(), 0, "conflicting tx_id/degraded_tx_id must send 0 commit POSTs");
    }
}
