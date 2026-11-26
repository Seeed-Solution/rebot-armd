//! Integration tests for multi-arm cell configuration (`rebot-cell/v1`), `capabilities.cell`,
//! `GET /api/v1/cell/status`, CLI `--cell` / `--cell-host`, and profile `cell` config key.

use std::sync::Arc;
use std::time::Duration;

use armd::cell::{CellConfig, CellContext, CELL_CONFIG_FORMAT};
use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth};
use serde_json::{json, Value};
use tokio::net::TcpListener;

fn two_so101_cell() -> Value {
    json!({
        "format": CELL_CONFIG_FORMAT,
        "cell_id": "two_so101",
        "coordinator": "pc_coord",
        "hosts": {
            "pc_coord": { "address": "192.168.42.10", "role": "pc" },
            "recamera_left": { "address": "192.168.42.11", "port": 8765, "role": "arm_host" },
            "recamera_right": { "address": "192.168.42.12", "port": 8765, "role": "arm_host" }
        },
        "arms": [
            {
                "id": "left",
                "type": "so-arm101",
                "profile": "so_arm101",
                "host": "recamera_left",
                "device": "/dev/ttySC0",
                "camera": "left_wrist",
                "T_cell_to_base": { "xyz": [0.0, -0.22, 0.0], "rpy_rad": [0.0, 0.0, 0.0] }
            },
            {
                "id": "right",
                "type": "so-arm101",
                "profile": "so_arm101",
                "host": "recamera_right",
                "device": "/dev/ttySC0",
                "camera": "right_wrist",
                "T_cell_to_base": { "xyz": [0.0, 0.22, 0.0], "rpy_rad": [0.0, 0.0, 0.0] }
            }
        ],
        "links": [
            {
                "from": "pc_coord",
                "to": "recamera_left",
                "endpoint": "udp://192.168.42.11:9100",
                "auth_ref": "env:REBOT_CELL_HMAC_KEY"
            },
            {
                "from": "pc_coord",
                "to": "recamera_right",
                "endpoint": "udp://192.168.42.12:9100",
                "auth_ref": "env:REBOT_CELL_HMAC_KEY"
            }
        ]
    })
}

fn b601_plus_so101_cell() -> Value {
    json!({
        "format": CELL_CONFIG_FORMAT,
        "cell_id": "b601_plus_so101",
        "coordinator": "pc_coord",
        "hosts": {
            "pc_coord": { "address": "192.168.42.10", "role": "pc" },
            "recamera_b601": { "address": "192.168.42.11", "port": 8765, "role": "arm_host" },
            "recamera_so101": { "address": "192.168.42.12", "port": 8765, "role": "arm_host" }
        },
        "arms": [
            {
                "id": "b601",
                "type": "b601dm",
                "profile": "rebot_b601dm",
                "host": "recamera_b601",
                "device": "can0",
                "camera": "b601_wrist",
                "T_cell_to_base": [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, -0.25],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ]
            },
            {
                "id": "so101",
                "type": "so-arm101",
                "profile": "so_arm101",
                "host": "recamera_so101",
                "device": "/dev/ttySC0",
                "camera": "so101_wrist",
                "T_cell_to_base": { "xyz": [0.0, 0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0] }
            }
        ],
        "links": [
            {
                "from": "pc_coord",
                "to": "recamera_b601",
                "endpoint": "udp://192.168.42.11:9100",
                "auth_ref": "env:REBOT_CELL_HMAC_KEY"
            },
            {
                "from": "pc_coord",
                "to": "recamera_so101",
                "endpoint": "udp://192.168.42.12:9100",
                "auth_ref": "env:REBOT_CELL_HMAC_KEY"
            }
        ]
    })
}

fn so101_profile() -> ArmConfig {
    let mut cfg: ArmConfig =
        serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf")))
        .unwrap();
    cfg.validate().unwrap();
    cfg
}

async fn spawn_server(sup: Arc<ArmSupervisor>) -> (String, tokio::task::JoinHandle<()>) {
    let router = create_router(sup, OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let handle = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });
    (format!("http://{addr}"), handle)
}

#[tokio::test]
async fn single_arm_default_omits_capabilities_cell_and_reports_disabled_status() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let (base, srv) = spawn_server(sup).await;
    let client = reqwest::Client::new();

    let schema: Value = client
        .get(format!("{base}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert!(
        schema["capabilities"].get("cell").is_none(),
        "capabilities.cell must be omitted when no cell config is loaded"
    );

    let status_resp = client
        .get(format!("{base}/api/v1/cell/status"))
        .send()
        .await
        .unwrap();
    assert_eq!(status_resp.status(), 200);
    let status: Value = status_resp.json().await.unwrap();
    assert_eq!(status, json!({ "enabled": false }));

    // Endpoint is strictly read-only.
    let post_resp = client
        .post(format!("{base}/api/v1/cell/status"))
        .json(&json!({}))
        .send()
        .await
        .unwrap();
    assert_eq!(post_resp.status(), 405);

    srv.abort();
}

#[tokio::test]
async fn cell_capabilities_and_status_reflect_arm_host_and_switchable_coordinator() {
    let so101 = so101_profile();

    // 1. PC coordinator, local host = recamera_left (arm_host)
    let pc_cell = CellConfig::from_value(&two_so101_cell(), None).unwrap();
    let pc_ctx = Arc::new(
        CellContext::resolve(pc_cell, Some("recamera_left"), None, &[], None, &so101, None)
            .unwrap(),
    );
    let sup = Arc::new(ArmSupervisor::new(so101.clone()));
    sup.set_cell(Some(pc_ctx.clone())).await.unwrap();
    let (base, srv) = spawn_server(sup).await;
    let client = reqwest::Client::new();

    let schema: Value = client
        .get(format!("{base}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let cell_cap = &schema["capabilities"]["cell"];
    assert_eq!(cell_cap["format"], CELL_CONFIG_FORMAT);
    assert_eq!(cell_cap["enabled"], true);
    assert_eq!(cell_cap["cell_id"], "two_so101");
    assert_eq!(
        cell_cap["config_digest"],
        "4f13e1e9c588f124cce8273a100c851be90acf125e30edc404e1031b275ff965"
    );
    assert_eq!(cell_cap["host_id"], "recamera_left");
    assert_eq!(cell_cap["arm_id"], "left");
    assert_eq!(cell_cap["coordinator"], "pc_coord");
    assert_eq!(cell_cap["is_coordinator"], false);
    assert_eq!(cell_cap["arm_ids"], json!(["left", "right"]));
    assert_eq!(cell_cap["total_dof"], 12);
    assert_eq!(cell_cap["peer_networking"], false);

    let status: Value = client
        .get(format!("{base}/api/v1/cell/status"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(status["enabled"], true);
    assert_eq!(status["format"], CELL_CONFIG_FORMAT);
    assert_eq!(status["cell_id"], "two_so101");
    assert_eq!(status["coordinator"], "pc_coord");
    assert_eq!(status["local_host"], "recamera_left");
    assert_eq!(status["local_role"], "arm_host");
    assert_eq!(status["is_coordinator"], false);
    assert_eq!(status["local_arm_id"], "left");
    assert_eq!(status["local_arm"]["id"], "left");
    assert_eq!(status["local_arm"]["dof"], 6);
    assert_eq!(status["local_arm"]["gripper_index"], 5);
    assert_eq!(status["local_arm"]["is_local"], true);
    assert_eq!(status["total_dof"], 12);
    assert_eq!(
        status["peer_networking"],
        json!({ "enabled": false, "state": "disabled" })
    );
    assert_eq!(status["arms"][0]["is_local"], true);
    assert_eq!(status["arms"][1]["is_local"], false);
    assert_eq!(status["links"].as_array().unwrap().len(), 2);
    srv.abort();

    // 2. Switch coordinator to recamera_left by config only
    let mut switched = two_so101_cell();
    switched["coordinator"] = json!("recamera_left");
    switched["links"] = json!([{
        "from": "recamera_left",
        "to": "recamera_right",
        "endpoint": "udp://192.168.42.12:9100",
        "auth_ref": "env:REBOT_CELL_HMAC_KEY"
    }]);
    let switched_cell = CellConfig::from_value(&switched, None).unwrap();
    let switched_ctx = Arc::new(
        CellContext::resolve(
            switched_cell,
            Some("recamera_left"),
            None,
            &[],
            None,
            &so101,
            None,
        )
        .unwrap(),
    );
    let sup2 = Arc::new(ArmSupervisor::new(so101.clone()));
    sup2.set_cell(Some(switched_ctx.clone())).await.unwrap();
    let (base2, srv2) = spawn_server(sup2).await;

    let status2: Value = client
        .get(format!("{base2}/api/v1/cell/status"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(status2["coordinator"], "recamera_left");
    assert_eq!(status2["is_coordinator"], true);
    assert_eq!(status2["local_role"], "coordinator_arm_host");
    assert_eq!(status2["local_arm_id"], "left");
    assert_ne!(status2["config_digest"], status["config_digest"]);
    srv2.abort();

    // 3. Coordinator-only PC host (same code path, no local arm)
    let coord_cell = CellConfig::from_value(&two_so101_cell(), None).unwrap();
    let coord_ctx = Arc::new(
        CellContext::resolve(coord_cell, Some("pc_coord"), None, &[], None, &so101, None).unwrap(),
    );
    let sup3 = Arc::new(ArmSupervisor::new(so101));
    sup3.set_cell(Some(coord_ctx)).await.unwrap();
    let (base3, srv3) = spawn_server(sup3).await;

    let status3: Value = client
        .get(format!("{base3}/api/v1/cell/status"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(status3["local_host"], "pc_coord");
    assert_eq!(status3["is_coordinator"], true);
    assert_eq!(status3["local_role"], "coordinator");
    assert!(status3.get("local_arm_id").is_none());
    assert!(status3.get("local_arm").is_none());
    assert_eq!(status3["arms"][0]["is_local"], false);
    assert_eq!(status3["arms"][1]["is_local"], false);
    srv3.abort();
}

#[tokio::test]
async fn executor_owner_serves_cell_capabilities_and_status_and_fences_post_transfer_mutation() {
    let cfg = ArmConfig::rebot_dm();
    let het_cell = CellConfig::from_value(&b601_plus_so101_cell(), None).unwrap();
    let ctx = Arc::new(
        CellContext::resolve(het_cell, Some("recamera_b601"), None, &[], None, &cfg, None).unwrap(),
    );
    let sup = Arc::new(ArmSupervisor::new(cfg));
    sup.start_dispatcher().unwrap();
    sup.set_cell(Some(ctx.clone())).await.unwrap();

    let exec_cfg = armd::rt_core::executor::ExecutorConfig {
        period: Duration::from_millis(20),
        boot: armd::rt_core::broker::BootId(1),
        capacities: armd::rt_core::broker::Capacities::default(),
        rt_sched: armd::rt_core::sched::RtSchedConfig::default(),
        memory_lock: armd::rt_core::sched::MemoryLock::Skipped("test".into()),
        alloc_timing: false,
        panic_policy: armd::rt_core::executor::PanicPolicy::AbortProcess,
        apply_rt_sched: false,
    };
    let handle = sup
        .start_executor_with_hooks(exec_cfg, armd::rt_core::executor::PersistTestHooks::default())
        .await
        .unwrap();

    // Mutating cell after ownership transfer must fail closed.
    assert!(sup.set_cell(None).await.is_err());

    let (base, srv) = spawn_server(sup.clone()).await;
    let client = reqwest::Client::new();

    // Wait briefly for the first executor tick to publish a fresh snapshot.
    tokio::time::sleep(Duration::from_millis(60)).await;

    let schema_resp = client
        .get(format!("{base}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap();
    assert_eq!(schema_resp.status(), 200);
    let schema: Value = schema_resp.json().await.unwrap();
    assert_eq!(schema["capabilities"]["cell"]["cell_id"], "b601_plus_so101");
    assert_eq!(schema["capabilities"]["cell"]["arm_id"], "b601");
    assert_eq!(schema["capabilities"]["cell"]["total_dof"], 13);
    assert_eq!(
        schema["capabilities"]["cell"]["config_digest"],
        "1592599dab0cb34b8e0c6c6085de5ab19e811991428606e8365d6aae41afde44"
    );

    let status_resp = client
        .get(format!("{base}/api/v1/cell/status"))
        .send()
        .await
        .unwrap();
    assert_eq!(status_resp.status(), 200);
    let status: Value = status_resp.json().await.unwrap();
    assert_eq!(status["enabled"], true);
    assert_eq!(status["local_host"], "recamera_b601");
    assert_eq!(status["local_arm_id"], "b601");
    assert_eq!(status["local_arm"]["dof"], 7);
    assert_eq!(status["total_dof"], 13);

    srv.abort();
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    assert!(sup.request_shutdown_until(deadline).await.unwrap());
    handle.shutdown_and_join_until(deadline).unwrap();
}

#[cfg(target_os = "linux")]
#[tokio::test]
async fn armd_cli_cell_flag_and_config_key_and_rejections() {
    fn free_port() -> u16 {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        assert!(!matches!(port, 18765 | 18001 | 18990));
        port
    }

    async fn wait_json(url: &str) -> Option<Value> {
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            if let Ok(resp) = reqwest::get(url).await {
                if resp.status().is_success() {
                    return resp.json::<Value>().await.ok();
                }
            }
            if std::time::Instant::now() > deadline {
                return None;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    let dir = tempfile::tempdir().unwrap();
    let cell_path = dir.path().join("two_so101.json");
    std::fs::write(&cell_path, serde_json::to_string_pretty(&two_so101_cell()).unwrap()).unwrap();
    let so101_path = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/so_arm101.json");

    // 1. CLI --cell + --cell-host works and exposes capabilities.cell + GET /api/v1/cell/status
    let port = free_port();
    let mut child = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &port.to_string(),
            "--config",
            so101_path,
            "--cell",
            cell_path.to_str().unwrap(),
            "--cell-host",
            "recamera_left",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .env_remove("ARMD_CELL")
        .env_remove("ARMD_CELL_HOST")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .unwrap();

    let schema = wait_json(&format!("http://127.0.0.1:{port}/api/v1/observation/schema")).await;
    let status = wait_json(&format!("http://127.0.0.1:{port}/api/v1/cell/status")).await;
    let _ = child.kill();
    let _ = child.wait();

    let schema = schema.expect("armd --cell did not serve schema");
    let status = status.expect("armd --cell did not serve /api/v1/cell/status");
    assert_eq!(schema["capabilities"]["cell"]["cell_id"], "two_so101");
    assert_eq!(schema["capabilities"]["cell"]["arm_id"], "left");
    assert_eq!(status["enabled"], true);
    assert_eq!(status["local_host"], "recamera_left");
    assert_eq!(status["local_arm_id"], "left");

    // 2. Profile config key ("cell" relative to profile + "cell_host") works without --cell
    let mut prof_val: Value =
        serde_json::from_str(&std::fs::read_to_string(so101_path).unwrap()).unwrap();
    // Point URDF to absolute path so the copied profile loads from tempdir.
    prof_val["urdf"] = json!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/assets/so101_new_calib.urdf"
    ));
    prof_val["cell"] = json!("two_so101.json");
    prof_val["cell_host"] = json!("recamera_right");
    let custom_prof_path = dir.path().join("so101_in_cell.json");
    std::fs::write(
        &custom_prof_path,
        serde_json::to_string_pretty(&prof_val).unwrap(),
    )
    .unwrap();

    let port2 = free_port();
    let mut child2 = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &port2.to_string(),
            "--config",
            custom_prof_path.to_str().unwrap(),
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .env_remove("ARMD_CELL")
        .env_remove("ARMD_CELL_HOST")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .unwrap();

    let status2 = wait_json(&format!("http://127.0.0.1:{port2}/api/v1/cell/status")).await;
    let _ = child2.kill();
    let _ = child2.wait();

    let status2 = status2.expect("armd profile cell key did not serve /api/v1/cell/status");
    assert_eq!(status2["enabled"], true);
    assert_eq!(status2["local_host"], "recamera_right");
    assert_eq!(status2["local_arm_id"], "right");

    // 3. Invalid cell JSON fails fast before binding/starting
    let bad_cell_path = dir.path().join("bad_cell.json");
    let mut bad = two_so101_cell();
    bad["arms"][1]["host"] = json!("recamera_left"); // violates one arm per host
    std::fs::write(&bad_cell_path, serde_json::to_string(&bad).unwrap()).unwrap();
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &free_port().to_string(),
            "--config",
            so101_path,
            "--cell",
            bad_cell_path.to_str().unwrap(),
            "--cell-host",
            "recamera_left",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .output()
        .unwrap();
    assert!(!out.status.success(), "invalid cell must fail startup");

    // 4. Mismatched local profile (SO-101 config vs B601 arm entry) fails fast
    let het_path = dir.path().join("b601_plus_so101.json");
    std::fs::write(&het_path, serde_json::to_string(&b601_plus_so101_cell()).unwrap()).unwrap();
    let out_mismatch = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &free_port().to_string(),
            "--config",
            so101_path,
            "--cell",
            het_path.to_str().unwrap(),
            "--cell-host",
            "recamera_b601",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .output()
        .unwrap();
    assert!(
        !out_mismatch.status.success(),
        "mismatched profile DOF must fail startup"
    );

    // 5. --cell-host without --cell fails fast
    let out_orphan = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &free_port().to_string(),
            "--cell-host",
            "recamera_left",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .env_remove("ARMD_CELL")
        .env_remove("ARMD_CELL_HOST")
        .output()
        .unwrap();
    assert!(
        !out_orphan.status.success(),
        "--cell-host without cell config must fail startup"
    );
}
