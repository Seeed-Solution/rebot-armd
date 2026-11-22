//! Integration tests verifying that editing the selected base robot XML on disk
//! invalidates the sandbox bundle cache (`GET /api/v1/sandbox/bundle`).

use std::ffi::OsString;
use std::path::Path;
use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use serde_json::{json, Value};
use tower::ServiceExt;

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::{SandboxState, SO_ARM101_BASE_XML};
use armd::mjcf_compose::FALLBACK_BASE_XML;
use armd::{ArmConfig, ArmSupervisor, OperatorAuth};

/// Serializes env-var mutations across tests in this integration test binary.
static BASE_XML_ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

struct EnvVarGuard {
    key: &'static str,
    prev: Option<OsString>,
}

impl EnvVarGuard {
    fn set_path(key: &'static str, path: &Path) -> Self {
        let prev = std::env::var_os(key);
        std::env::set_var(key, path);
        Self { key, prev }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prev {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

fn app_for(config: ArmConfig, state: Arc<SandboxState>) -> axum::Router {
    let supervisor = Arc::new(ArmSupervisor::new(config));
    create_router_with_sandbox(supervisor, OperatorAuth::disabled(), state)
}

fn so_arm101_config() -> ArmConfig {
    let mut cfg: ArmConfig =
        serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf")))
        .unwrap();
    cfg
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
    let bytes = axum::body::to_bytes(res.into_body(), usize::MAX)
        .await
        .unwrap();
    let parsed = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
    (status, parsed)
}

#[tokio::test]
async fn editing_selected_b601dm_base_xml_after_first_get_invalidates_bundle_cache() {
    let _env_guard = BASE_XML_ENV_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let base_xml_path = tmp.path().join("custom_b601dm_base.xml");

    let initial_xml = FALLBACK_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="base_xml_marker_v1" pos="0.10 0.00 0.05" size="0.01"/>
  </worldbody>"#,
        1,
    );
    assert!(initial_xml.contains("base_xml_marker_v1"));
    std::fs::write(&base_xml_path, &initial_xml).unwrap();
    let _var_guard = EnvVarGuard::set_path("ARMD_SANDBOX_BASE_XML", &base_xml_path);

    let state = Arc::new(
        SandboxState::new(tmp.path().join("sandbox"))
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    let (st, act_body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "default"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act_body}");
    assert_eq!(state.bundle_rebuild_count(), 0);

    // First GET /bundle hits the pre-populated cache from activate and returns v1 XML.
    let (st1, bundle1) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st1, StatusCode::OK, "{bundle1}");
    let xml1 = bundle1["composed_xml"].as_str().unwrap();
    let sha1 = bundle1["composed_xml_sha256"].as_str().unwrap().to_string();
    assert!(xml1.contains("base_xml_marker_v1"), "{xml1}");
    assert_eq!(state.bundle_rebuild_count(), 0);

    // Second GET /bundle without editing must remain a cache hit.
    let (st2, bundle2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st2, StatusCode::OK, "{bundle2}");
    assert_eq!(bundle2["composed_xml_sha256"].as_str(), Some(sha1.as_str()));
    assert_eq!(state.bundle_rebuild_count(), 0);

    // Edit the selected base robot XML file on disk (both content and size change).
    let edited_xml = FALLBACK_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="edited_base_marker_v2_extra" pos="0.15 0.05 0.10" size="0.02"/>
  </worldbody>"#,
        1,
    );
    assert!(edited_xml.contains("edited_base_marker_v2_extra"));
    std::fs::write(&base_xml_path, &edited_xml).unwrap();

    // Third GET /bundle must detect fingerprint mismatch, rebuild, and return the edited XML.
    let (st3, bundle3) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st3, StatusCode::OK, "{bundle3}");
    let xml3 = bundle3["composed_xml"].as_str().unwrap();
    let sha3 = bundle3["composed_xml_sha256"].as_str().unwrap().to_string();
    assert_ne!(sha3, sha1, "composed_xml_sha256 must change after editing base XML");
    assert!(!xml3.contains("base_xml_marker_v1"), "{xml3}");
    assert!(xml3.contains("edited_base_marker_v2_extra"), "{xml3}");
    assert_eq!(state.bundle_rebuild_count(), 1);

    // Fourth GET /bundle without further edits hits the newly rebuilt cache.
    let (st4, bundle4) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st4, StatusCode::OK, "{bundle4}");
    assert_eq!(bundle4["composed_xml_sha256"].as_str(), Some(sha3.as_str()));
    assert_eq!(state.bundle_rebuild_count(), 1);
}

#[cfg(unix)]
#[tokio::test]
async fn editing_symlinked_base_xml_target_after_cold_get_invalidates_bundle_cache() {
    let _env_guard = BASE_XML_ENV_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let target_xml_path = tmp.path().join("target_b601dm.xml");
    let link_xml_path = tmp.path().join("link_b601dm.xml");

    let initial_xml = FALLBACK_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="symlink_base_v1" pos="0.10 0.00 0.05" size="0.01"/>
  </worldbody>"#,
        1,
    );
    std::fs::write(&target_xml_path, &initial_xml).unwrap();
    std::os::unix::fs::symlink(&target_xml_path, &link_xml_path).unwrap();
    let _var_guard = EnvVarGuard::set_path("ARMD_SANDBOX_BASE_XML", &link_xml_path);

    let state = Arc::new(
        SandboxState::new(tmp.path().join("sandbox"))
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(ArmConfig::rebot_dm(), state.clone());

    // Cold GET /bundle (via build_bundle_from_disk) warms cache.
    let (st1, bundle1) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st1, StatusCode::OK, "{bundle1}");
    let sha1 = bundle1["composed_xml_sha256"].as_str().unwrap().to_string();
    assert!(bundle1["composed_xml"].as_str().unwrap().contains("symlink_base_v1"));
    assert_eq!(state.bundle_rebuild_count(), 1);

    // Edit the symlink target directly; canonical path watch must catch the change.
    let edited_xml = FALLBACK_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="symlink_base_v2_updated" pos="0.12 0.02 0.08" size="0.015"/>
  </worldbody>"#,
        1,
    );
    std::fs::write(&target_xml_path, &edited_xml).unwrap();

    let (st2, bundle2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st2, StatusCode::OK, "{bundle2}");
    let sha2 = bundle2["composed_xml_sha256"].as_str().unwrap().to_string();
    assert_ne!(sha2, sha1);
    assert!(bundle2["composed_xml"]
        .as_str()
        .unwrap()
        .contains("symlink_base_v2_updated"));
    assert_eq!(state.bundle_rebuild_count(), 2);
}

#[tokio::test]
async fn editing_selected_so_arm101_base_xml_after_first_get_invalidates_bundle_cache() {
    let _env_guard = BASE_XML_ENV_LOCK.lock().await;
    let tmp = tempfile::tempdir().unwrap();
    let empty_builtins = tempfile::tempdir().unwrap();
    let base_xml_path = tmp.path().join("custom_so_arm101_base.xml");

    let initial_xml = SO_ARM101_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="so101_marker_v1" pos="0.10 0.00 0.05" size="0.01"/>
  </worldbody>"#,
        1,
    );
    assert!(initial_xml.contains("so101_marker_v1"));
    std::fs::write(&base_xml_path, &initial_xml).unwrap();
    let _var_guard = EnvVarGuard::set_path("ARMD_SANDBOX_SO_ARM101_BASE_XML", &base_xml_path);

    let state = Arc::new(
        SandboxState::new(tmp.path().join("sandbox"))
            .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
    );
    let app = app_for(so_arm101_config(), state.clone());

    let (st, act_body) = send_json(
        &app,
        "POST",
        "/api/v1/sandbox/activate",
        Some(json!({"mode": "wasm", "scene": "default", "tool": "default", "robot": "so-arm101"})),
    )
    .await;
    assert_eq!(st, StatusCode::OK, "{act_body}");

    let (st1, bundle1) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st1, StatusCode::OK, "{bundle1}");
    let sha1 = bundle1["composed_xml_sha256"].as_str().unwrap().to_string();
    assert!(bundle1["composed_xml"].as_str().unwrap().contains("so101_marker_v1"));
    assert_eq!(state.bundle_rebuild_count(), 0);

    let (st2, bundle2) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st2, StatusCode::OK, "{bundle2}");
    assert_eq!(bundle2["composed_xml_sha256"].as_str(), Some(sha1.as_str()));
    assert_eq!(state.bundle_rebuild_count(), 0);

    let edited_xml = SO_ARM101_BASE_XML.replacen(
        "</worldbody>",
        r#"  <site name="so101_edited_marker_v2" pos="0.14 0.04 0.09" size="0.02"/>
  </worldbody>"#,
        1,
    );
    std::fs::write(&base_xml_path, &edited_xml).unwrap();

    let (st3, bundle3) = send_json(&app, "GET", "/api/v1/sandbox/bundle", None).await;
    assert_eq!(st3, StatusCode::OK, "{bundle3}");
    let xml3 = bundle3["composed_xml"].as_str().unwrap();
    let sha3 = bundle3["composed_xml_sha256"].as_str().unwrap().to_string();
    assert_ne!(sha3, sha1);
    assert!(!xml3.contains("so101_marker_v1"));
    assert!(xml3.contains("so101_edited_marker_v2"));
    assert_eq!(state.bundle_rebuild_count(), 1);
}
