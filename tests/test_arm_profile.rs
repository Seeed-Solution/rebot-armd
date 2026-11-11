//! The reCamera B601-DM kit ships the built-in `ArmConfig::rebot_dm()` as data
//! (kits/recamera-b601dm/profile/rebot-b601dm.json). These tests keep the two equal.
//! Regenerate the file after changing `rebot_dm()`:
//! `UPDATE_PROFILE=1 cargo test --test test_arm_profile`.

use armd::{ArmConfig, ARM_PROFILE_VERSION};

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");

/// JSON text form of a config, parsed back. Going through text keeps the f32 values in
/// their shortest form (3.14159, not 3.141590118408203), as written in the file; equal
/// text forms mean the parsed f32 values are bit-identical.
fn json_of(config: &ArmConfig) -> serde_json::Value {
    serde_json::from_str(&serde_json::to_string(config).unwrap()).unwrap()
}

fn builtin_json() -> serde_json::Value {
    json_of(&ArmConfig::rebot_dm())
}

#[test]
fn kit_profile_equals_builtin_rebot_dm() {
    if std::env::var_os("UPDATE_PROFILE").is_some() {
        let text = serde_json::to_string_pretty(&ArmConfig::rebot_dm()).unwrap() + "\n";
        std::fs::write(PROFILE_PATH, text).unwrap();
    }
    let file: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(PROFILE_PATH).unwrap()).unwrap();
    assert_eq!(file, builtin_json(), "{PROFILE_PATH} differs from ArmConfig::rebot_dm(); rerun with UPDATE_PROFILE=1");
}

#[test]
fn kit_profile_loads_like_config_flag_and_round_trips() {
    // Same parse and validation as `armd --config`.
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(PROFILE_PATH).unwrap()).unwrap();
    config.validate().unwrap();
    assert_eq!(config.profile_version, ARM_PROFILE_VERSION);
    assert_eq!(json_of(&config), builtin_json());
}

#[test]
fn profile_without_version_is_version_1() {
    let mut value = builtin_json();
    value.as_object_mut().unwrap().remove("profile_version");
    let mut config: ArmConfig = serde_json::from_value(value).unwrap();
    assert_eq!(config.profile_version, 1);
    config.validate().unwrap();
}

#[test]
fn newer_profile_version_is_refused() {
    let mut config = ArmConfig::rebot_dm();
    config.profile_version = ARM_PROFILE_VERSION + 1;
    assert!(config.validate().unwrap_err().contains("profile_version"));
}

const INIT_SCRIPT: &str = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");

/// The init script passes the installed profile to the rebot-dm and mock followers, only when
/// the file exists, and --follower-port to every follower except mock; build_deb.sh installs
/// the kit profile at that path.
#[cfg(unix)]
#[test]
fn init_script_passes_kit_profile_to_rebot_dm_and_mock_only() {
    assert!(INIT_SCRIPT.contains("PROFILE=/usr/share/$NAME/profile/rebot-b601dm.json"));
    assert!(include_str!("../scripts/build_deb.sh")
        .contains("/usr/share/rebot-teleop/profile/rebot-b601dm.json\""));
    let start = INIT_SCRIPT.find("    DAEMON_OPTS=\"--follower").unwrap();
    // The snippet ends with the `fi` that closes the `--config` block.
    let config_at = start + INIT_SCRIPT[start..].find("--config $PROFILE").unwrap();
    let end = config_at + INIT_SCRIPT[config_at..].find("\n    fi\n").unwrap() + "\n    fi\n".len();
    let snippet = &INIT_SCRIPT[start..end];
    let run = |model: &str, profile: &str| {
        let script = format!(
            "FOLLOWER_MODEL={model}\nFOLLOWER_PORT=can0\nPORT=8765\nPROFILE='{profile}'\n{snippet}printf '%s' \"$DAEMON_OPTS\"\n"
        );
        let out = std::process::Command::new("sh").args(["-c", &script]).output().unwrap();
        assert!(out.status.success());
        String::from_utf8(out.stdout).unwrap()
    };
    assert_eq!(
        run("rebot-dm", PROFILE_PATH),
        format!("--follower rebot-dm --follower-port can0 --port 8765 --config {PROFILE_PATH}")
    );
    assert_eq!(run("rebot-dm", "/nonexistent/profile.json"), "--follower rebot-dm --follower-port can0 --port 8765");
    assert_eq!(run("so-arm101", PROFILE_PATH), "--follower so-arm101 --follower-port can0 --port 8765");
    assert_eq!(run("mock", PROFILE_PATH), format!("--follower mock --port 8765 --config {PROFILE_PATH}"));
    assert_eq!(run("mock", "/nonexistent/profile.json"), "--follower mock --port 8765");
}

/// armd started with `--config <kit profile>` serves the B601-DM schema (mock follower).
#[cfg(target_os = "linux")]
#[tokio::test]
async fn armd_serves_the_kit_profile_from_config_flag() {
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = reservation.local_addr().unwrap().port();
    drop(reservation);
    let mut child = std::process::Command::new(env!("CARGO_BIN_EXE_armd"))
        .args(["--follower", "mock", "--port", &port.to_string(), "--config", PROFILE_PATH])
        .env_remove("PORT")
        .env("ARMD_GRASP_SETUP", "/nonexistent/grasp_setup.json")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .unwrap();
    let url = format!("http://127.0.0.1:{port}/api/v1/observation/schema");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let schema = loop {
        if let Ok(resp) = reqwest::get(&url).await {
            break resp.json::<serde_json::Value>().await.ok();
        }
        if std::time::Instant::now() > deadline {
            break None;
        }
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    };
    let _ = child.kill();
    let _ = child.wait();
    let schema = schema.expect("armd with the kit profile did not serve the schema");
    let builtin = builtin_json();
    assert_eq!(schema["dof"], builtin["dof"]);
    assert_eq!(schema["joint_names"], builtin["joint_names"]);
    assert_eq!(schema["limits"], builtin["limits"]);
    assert_eq!(schema["workspace"], builtin["workspace"]);
}

const KIT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm");

/// calibration/grasp_setup.default.json is the profile's grasp section, in the format
/// armd reads from ARMD_GRASP_SETUP.
#[test]
fn kit_grasp_defaults_equal_profile_grasp() {
    let text = std::fs::read_to_string(format!("{KIT}/calibration/grasp_setup.default.json")).unwrap();
    let setup: armd::grasp::GraspSetup = serde_json::from_str(&text).unwrap();
    assert_eq!(Some(setup), ArmConfig::rebot_dm().grasp);
}

/// calibration/camera_mounts.json: every entry is a valid camera_mount; v5 is the
/// profile's default mount.
#[test]
fn kit_camera_mounts_parse() {
    let text = std::fs::read_to_string(format!("{KIT}/calibration/camera_mounts.json")).unwrap();
    let mounts: std::collections::BTreeMap<String, serde_json::Value> = serde_json::from_str(&text).unwrap();
    assert_eq!(mounts.len(), 4);
    for (name, entry) in &mounts {
        let mount: armd::model::CameraMount = serde_json::from_value(entry["camera_mount"].clone()).unwrap();
        let pitch = entry["pitch_down_deg"].as_f64().unwrap() as f32;
        assert_eq!(mount.tilt_deg, -pitch, "{name}");
        assert!(std::path::Path::new(&format!("{KIT}/{}", entry["mount_dir"].as_str().unwrap())).is_dir(), "{name}");
    }
    let v5: armd::model::CameraMount = serde_json::from_value(mounts["v5"]["camera_mount"].clone()).unwrap();
    assert_eq!(v5, ArmConfig::rebot_dm().camera_mount);
}

/// kits/_template/profile/example.json (written from `ArmConfig::default()` with
/// UPDATE_PROFILE=1) loads and validates like a `--config` profile.
#[test]
fn template_profile_is_a_valid_profile() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/_template/profile/example.json");
    if std::env::var_os("UPDATE_PROFILE").is_some() {
        std::fs::write(path, serde_json::to_string_pretty(&ArmConfig::default()).unwrap() + "\n").unwrap();
    }
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(path).unwrap()).unwrap();
    config.validate().unwrap();
    assert_eq!(config.profile_version, ARM_PROFILE_VERSION);
    assert_eq!(config.dof, 6);
}
