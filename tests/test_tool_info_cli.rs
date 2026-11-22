use std::process::Command;

use serde_json::Value;

fn run_armd_tool_info(args: &[&str]) -> (i32, String, String) {
    let bin = env!("CARGO_BIN_EXE_armd");
    let out = Command::new(bin).args(args).output().expect("execute armd");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

#[test]
fn cli_tool_info_covers_finray_stock_robots_and_rejects_invalid() {
    let dir = tempfile::tempdir().unwrap();

    // 1. soft_finray_gripper (with explicit --robot b601dm and inferred from robots: ["b601dm"])
    let finray_path = dir.path().join("finray.json");
    std::fs::write(
        &finray_path,
        r#"{
          "id": "soft_finray_gripper",
          "kind": "gripper",
          "robots": ["b601dm"],
          "tcp_offset": [0.0288, 0.0, 0.0],
          "armd": {
            "extra_check_points_end_m": [
              [0.032,  0.082,  0.022],
              [0.032,  0.082, -0.022],
              [0.032, -0.082,  0.022],
              [0.032, -0.082, -0.022],
              [0.032,  0.000,  0.022],
              [0.032,  0.000, -0.022],
              [0.000,  0.082,  0.022],
              [0.000,  0.082, -0.022],
              [0.000, -0.082,  0.022],
              [0.000, -0.082, -0.022]
            ],
            "physical_contact_end_m": [0.0288, 0.0, 0.0],
            "contact_opening_m": 0.0036,
            "dynamics": { "mass_delta_kg": 0.035, "com_moment_delta_nm": 0.018 },
            "gripper_policy": {
              "jaw_opening_nominal": [[-5.7, 0.0886], [0.0, 0.0036]],
              "holding_gap_rad": 0.30,
              "holding_torque_nm": 0.0667,
              "contact_torque_nm": 0.6,
              "squeeze_rad": 0.15,
              "max_close_torque_nm": 1.0
            },
            "grasp": { "max_grasp_width_m": 0.075, "grasp_height_m": 0.015, "edge_to_low_m": null }
          }
        }"#,
    )
    .unwrap();
    for args in [
        vec!["--tool-info", finray_path.to_str().unwrap(), "--robot", "b601dm"],
        vec!["--tool-info", finray_path.to_str().unwrap()],
    ] {
        let (code, stdout, stderr) = run_armd_tool_info(&args);
        assert_eq!(code, 0, "stderr: {stderr}");
        assert_eq!(stdout.lines().count(), 1, "expected single-line JSON: {stdout}");
        let v: Value = serde_json::from_str(stdout.trim()).unwrap();
        assert_eq!(v["ok"], true);
        assert_eq!(v["id"], "soft_finray_gripper");
        assert_eq!(v["robot_id"], "b601dm");
        assert_eq!(v["is_stock"], false);
        assert_eq!(v["kind"], "gripper");
        assert_eq!(v["tcp_offset_m"], serde_json::json!([0.0288, 0.0, 0.0]));
        assert_eq!(
            v["config_digest"],
            "v1:bc4e643de8bf65ec9537c2b22ddbb1ae230e14a091a169cd4d199b955af49776"
        );
        assert!(v["canonical_json"].as_str().unwrap().contains("\"tool_id\":\"soft_finray_gripper\""));
    }

    // 2. Stock `default` across b601dm, so101, so100, b601rs
    let stock_path = dir.path().join("default.json");
    std::fs::write(&stock_path, r#"{"id":"default","kind":"gripper"}"#).unwrap();

    // Empty `robots` without `--robot` must fail with non-zero exit code.
    let (code, _stdout, stderr) = run_armd_tool_info(&["--tool-info", stock_path.to_str().unwrap()]);
    assert_ne!(code, 0);
    assert!(stderr.contains("--robot"), "{stderr}");

    for (robot, expected_digest) in [
        ("b601dm", "v1:ac4d69ae16085196687d381a4dbfa89a1940a08522dd088f2459645fd91d1064"),
        ("so101", "v1:71cfcf4dafaa5a52c2366c3ace757f714628321b761f195f4456a7ae2ba26a42"),
        ("so100", "v1:efbb9e76729b3b748959c032a789412e0ef0224d197079d16f688a32f82463d4"),
        ("b601rs", "v1:940a10d6d0dadaa2c6d2fdf7771cb8e226805c3300233dff9661fc5b8cf89636"),
    ] {
        let (code, stdout, stderr) =
            run_armd_tool_info(&["--tool-info", stock_path.to_str().unwrap(), "--robot", robot]);
        assert_eq!(code, 0, "robot={robot} stderr={stderr}");
        let v: Value = serde_json::from_str(stdout.trim()).unwrap();
        assert_eq!(v["ok"], true);
        assert_eq!(v["id"], "default");
        assert_eq!(v["robot_id"], robot);
        assert_eq!(v["is_stock"], true);
        assert_eq!(v["config_digest"], expected_digest);
    }

    // 3. Invalid tool.json (non-stock missing tcp_offset, unknown armd field, missing file) -> exit != 0
    let missing_tcp_path = dir.path().join("missing_tcp.json");
    std::fs::write(
        &missing_tcp_path,
        r#"{"id":"no_tcp","kind":"gripper","robots":["b601dm"],"tool_xml":"tool.xml","tcp_site":"tcp"}"#,
    )
    .unwrap();
    let (code, _stdout, stderr) =
        run_armd_tool_info(&["--tool-info", missing_tcp_path.to_str().unwrap(), "--robot", "b601dm"]);
    assert_ne!(code, 0);
    assert!(stderr.contains("tcp_offset"), "{stderr}");

    let bad_field_path = dir.path().join("bad_field.json");
    std::fs::write(
        &bad_field_path,
        r#"{"id":"bad","kind":"gripper","robots":["b601dm"],"tcp_offset":[0,0,0],"armd":{"bogus":1}}"#,
    )
    .unwrap();
    let (code, _stdout, _stderr) =
        run_armd_tool_info(&["--tool-info", bad_field_path.to_str().unwrap()]);
    assert_ne!(code, 0);

    let (code, _stdout, _stderr) =
        run_armd_tool_info(&["--tool-info", dir.path().join("nonexistent.json").to_str().unwrap()]);
    assert_ne!(code, 0);

    // 4. M2: B601-RS suction manifest passes both --tool-info --robot b601rs and daemon profile load.
    let rs_suction_path = dir.path().join("rs_suction.json");
    std::fs::write(
        &rs_suction_path,
        r#"{"id":"rs_suction","kind":"suction","robots":["b601rs"],"tcp_offset":[0.0,0.0,0.0]}"#,
    )
    .unwrap();
    let (code, stdout, stderr) =
        run_armd_tool_info(&["--tool-info", rs_suction_path.to_str().unwrap(), "--robot", "b601rs"]);
    assert_eq!(code, 0, "stderr: {stderr}");
    let rs_info: Value = serde_json::from_str(stdout.trim()).unwrap();
    let rs_profile = armd::ArmConfig::from_profile_file(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/profiles/rebot_b601_rs.json"
    ))
    .unwrap();
    let rs_pack = armd::tool::load_tool_file(&rs_suction_path).unwrap();
    let rs_applied = armd::tool::apply_tool_overlay(&rs_profile, &rs_pack).unwrap();
    assert_eq!(rs_profile.profile_id.as_deref(), Some("rebot_b601_rs"));
    assert_eq!(rs_info["config_digest"], rs_applied.active_tool.config_digest);

    // 5. B1: Non-UTF-8 ARMD_TOOL env var must NOT silently fall back to stock and skip commissioning.
    #[cfg(unix)]
    {
        use std::os::unix::ffi::OsStrExt;
        let bin = env!("CARGO_BIN_EXE_armd");
        let bad_os = std::ffi::OsStr::from_bytes(b"/nonexistent_\xff/tool.json");
        let out = Command::new(bin)
            .env("ARMD_TOOL", bad_os)
            .args(["--follower", "mock", "--port", "0"])
            .output()
            .expect("execute armd");
        assert!(!out.status.success(), "non-UTF-8 ARMD_TOOL must fail, not fall back to stock");
    }
}
