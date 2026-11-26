//! Startup control-loop owner selection (`Auto` default) matrix, CLI/env/config precedence,
//! ineligible explicit-executor refusals, observation/diagnostics exposure, and cockpit rules.

use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use armd::rt_core::executor::{
    resolve_startup_owner, OwnerSource, RequestedRtOwner, RtOwner,
};
use armd::{ArmConfig, EffectiveOwner, OwnerReason};
use serde_json::Value;

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn mit_configured_rebot_dm() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    let jt = cfg
        .joint_tracking
        .as_mut()
        .expect("rebot_dm has joint_tracking seed");
    jt.enabled = true;
    jt.joints[0].strategy = armd::model::TrackingStrategy::Mit;
    jt.joints[0].kp = 10.0;
    jt.joints[0].kd = 0.5;
    jt.joints[0].effort_limit_nm = 5.0;
    jt.joints[0].hold_kp = Some(10.0);
    jt.joints[0].hold_kd = Some(0.5);
    cfg
}

fn mit_configured_rebot_dm_json() -> String {
    let mut cfg = mit_configured_rebot_dm();
    // Ensure valid config before serializing.
    cfg.validate().expect("MIT test profile must validate");
    serde_json::to_string_pretty(&cfg).unwrap()
}

fn spawn_armd_and_wait_port(
    args: &[&str],
    envs: &[(&str, &str)],
    remove_envs: &[&str],
) -> (ChildGuard, u16, std::sync::Arc<std::sync::Mutex<Vec<String>>>) {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_armd"));
    cmd.args(args)
        .args([
            "--port",
            "0",
            "--no-operator-auth",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .env_remove("ARMD_RT_OWNER")
        .env("RUST_LOG", "armd=info")
        .env("ARMD_APP_CONFIG", "off")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for k in remove_envs {
        cmd.env_remove(k);
    }
    for (k, v) in envs {
        cmd.env(k, v);
    }
    let mut child = cmd.spawn().expect("spawn armd");
    let stdout = child.stdout.take().expect("piped stdout");
    let stderr = child.stderr.take().expect("piped stderr");
    let logs = std::sync::Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let logs_out = logs.clone();
    let logs_err = logs.clone();
    let (port_tx, port_rx) = std::sync::mpsc::channel::<u16>();

    std::thread::spawn(move || {
        let mut sent = false;
        for line in BufReader::new(stdout).lines().map_while(Result::ok) {
            if !sent {
                if let Some(idx) = line.find("listening on http://") {
                    let rest = &line[idx + "listening on http://".len()..];
                    if let Some(colon) = rest.rfind(':') {
                        let digits: String = rest[colon + 1..]
                            .chars()
                            .take_while(|c| c.is_ascii_digit())
                            .collect();
                        if let Ok(port) = digits.parse::<u16>() {
                            let _ = port_tx.send(port);
                            sent = true;
                        }
                    }
                }
            }
            logs_out.lock().unwrap().push(line);
        }
    });
    std::thread::spawn(move || {
        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
            logs_err.lock().unwrap().push(line);
        }
    });

    let guard = ChildGuard(child);
    let port = port_rx
        .recv_timeout(Duration::from_secs(10))
        .unwrap_or_else(|_| {
            panic!(
                "armd did not listen in time; logs:\n{}",
                logs.lock().unwrap().join("\n")
            )
        });
    assert!(
        port != 18765 && port != 18001 && port != 18990,
        "test must never bind forbidden ports, got {port}"
    );
    (guard, port, logs)
}

fn run_armd_expect_fail(args: &[&str], envs: &[(&str, &str)]) -> String {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_armd"));
    cmd.args(args)
        .args([
            "--port",
            "0",
            "--no-operator-auth",
            "--log-dir",
            "off",
            "--blackbox-dir",
            "off",
            "--episodes-dir",
            "off",
        ])
        .env_remove("PORT")
        .env_remove("ARMD_RT_OWNER")
        .env("RUST_LOG", "armd=info")
        .env("ARMD_APP_CONFIG", "off")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for (k, v) in envs {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("run armd");
    assert!(
        !out.status.success(),
        "expected failure for args={args:?} envs={envs:?}, stdout={}, stderr={}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

#[test]
fn startup_owner_matrix_cli_env_config_auto_by_follower_and_profile() {
    let dm = ArmConfig::rebot_dm();
    let generic = ArmConfig::default();
    let mit_dm = mit_configured_rebot_dm();
    let mut over_dof = ArmConfig::default();
    over_dof.dof = armd::rt_core::mailbox::MAX_DOF + 1;

    let qualified_followers = ["rebot-dm", "mock"];
    let unqualified_followers = [
        "so-arm101",
        "so-arm100",
        "rebot-rs",
        "fashionstar",
        "star-arm102",
        "sim",
    ];

    // 1. Omitted (Default Auto) × follower kinds:
    //    - rebot-dm and mock -> Executor (AutoQualified)
    //    - all others -> Legacy (AutoFallbackUnqualifiedFollower) with fallback detail naming follower
    for follower in qualified_followers {
        let cfg = if follower == "rebot-dm" { &dm } else { &generic };
        let res = resolve_startup_owner(None, None, None, follower, cfg).unwrap();
        assert_eq!(res.requested, RequestedRtOwner::Auto);
        assert_eq!(res.source, OwnerSource::Default);
        assert_eq!(res.effective, RtOwner::Executor, "{follower}");
        assert_eq!(res.reason, OwnerReason::AutoQualified, "{follower}");
        assert!(res.fallback_detail.is_none());
        assert!(!res.reason.is_fallback());
    }
    for follower in unqualified_followers {
        let res = resolve_startup_owner(None, None, None, follower, &generic).unwrap();
        assert_eq!(res.requested, RequestedRtOwner::Auto);
        assert_eq!(res.source, OwnerSource::Default);
        assert_eq!(res.effective, RtOwner::Legacy, "{follower}");
        assert_eq!(
            res.reason,
            OwnerReason::AutoFallbackUnqualifiedFollower,
            "{follower}"
        );
        assert!(res.reason.is_fallback());
        let detail = res.fallback_detail.as_deref().expect("fallback detail");
        assert!(detail.contains(follower), "detail should name {follower}: {detail}");
    }

    // 2. MIT-configured profile and over-MAX_DOF profile under Auto -> Legacy with specific fallback reason.
    for follower in qualified_followers {
        let res_mit = resolve_startup_owner(None, None, None, follower, &mit_dm).unwrap();
        assert_eq!(res_mit.effective, RtOwner::Legacy);
        assert_eq!(res_mit.reason, OwnerReason::AutoFallbackMitProfile);
        assert!(res_mit.reason.is_fallback());
        assert!(res_mit.fallback_detail.as_deref().unwrap().contains("MIT"));

        let res_dof = resolve_startup_owner(None, None, None, follower, &over_dof).unwrap();
        assert_eq!(res_dof.effective, RtOwner::Legacy);
        assert_eq!(res_dof.reason, OwnerReason::AutoFallbackDofLimit);
        assert!(res_dof.reason.is_fallback());
        assert!(res_dof.fallback_detail.as_deref().unwrap().contains("DOF"));
    }

    // 3. Explicit precedence: CLI > env > config > Auto, across all follower kinds.
    for follower in qualified_followers.iter().chain(unqualified_followers.iter()) {
        let cfg = if *follower == "rebot-dm" { &dm } else { &generic };
        let qualified = qualified_followers.contains(follower);

        // Config legacy wins over default auto.
        let r_cfg_leg = resolve_startup_owner(None, None, Some("legacy"), follower, cfg).unwrap();
        assert_eq!(r_cfg_leg.effective, RtOwner::Legacy);
        assert_eq!(r_cfg_leg.source, OwnerSource::Config);
        assert_eq!(r_cfg_leg.reason, OwnerReason::ExplicitConfig);
        assert!(!r_cfg_leg.reason.is_fallback());

        // Env legacy wins over config executor.
        let r_env_leg =
            resolve_startup_owner(None, Some("legacy"), Some("executor"), follower, cfg).unwrap();
        assert_eq!(r_env_leg.effective, RtOwner::Legacy);
        assert_eq!(r_env_leg.source, OwnerSource::Env);
        assert_eq!(r_env_leg.reason, OwnerReason::ExplicitEnv);

        // CLI legacy wins over env executor and config executor.
        let r_cli_leg = resolve_startup_owner(
            Some("legacy"),
            Some("executor"),
            Some("executor"),
            follower,
            cfg,
        )
        .unwrap();
        assert_eq!(r_cli_leg.effective, RtOwner::Legacy);
        assert_eq!(r_cli_leg.source, OwnerSource::Cli);
        assert_eq!(r_cli_leg.reason, OwnerReason::ExplicitCli);

        // CLI auto wins over env legacy and config legacy.
        let r_cli_auto =
            resolve_startup_owner(Some("auto"), Some("legacy"), Some("legacy"), follower, cfg)
                .unwrap();
        assert_eq!(r_cli_auto.source, OwnerSource::Cli);
        if qualified {
            assert_eq!(r_cli_auto.effective, RtOwner::Executor);
            assert_eq!(r_cli_auto.reason, OwnerReason::AutoQualified);
        } else {
            assert_eq!(r_cli_auto.effective, RtOwner::Legacy);
            assert_eq!(r_cli_auto.reason, OwnerReason::AutoFallbackUnqualifiedFollower);
        }

        // Env auto wins over config legacy.
        let r_env_auto =
            resolve_startup_owner(None, Some("auto"), Some("legacy"), follower, cfg).unwrap();
        assert_eq!(r_env_auto.source, OwnerSource::Env);
        if qualified {
            assert_eq!(r_env_auto.effective, RtOwner::Executor);
            assert_eq!(r_env_auto.reason, OwnerReason::AutoQualified);
        } else {
            assert_eq!(r_env_auto.effective, RtOwner::Legacy);
            assert_eq!(r_env_auto.reason, OwnerReason::AutoFallbackUnqualifiedFollower);
        }

        // Explicit executor via CLI, env, or config:
        // - succeeds on qualified followers with explicit reason;
        // - REFUSED with clear error on every unqualified follower!
        for (cli, env, conf, expect_reason) in [
            (Some("executor"), Some("legacy"), Some("legacy"), OwnerReason::ExplicitCli),
            (None, Some("executor"), Some("legacy"), OwnerReason::ExplicitEnv),
            (None, None, Some("executor"), OwnerReason::ExplicitConfig),
        ] {
            let res = resolve_startup_owner(cli, env, conf, follower, cfg);
            if qualified {
                let ok = res.unwrap();
                assert_eq!(ok.effective, RtOwner::Executor);
                assert_eq!(ok.reason, expect_reason);
                assert!(ok.fallback_detail.is_none());
            } else {
                let err = res.unwrap_err();
                assert!(err.contains(follower), "refusal must name {follower}: {err}");
            }
        }
    }

    // 4. Explicit executor on MIT profile or over-MAX_DOF profile is refused across CLI/env/config.
    for (cli, env, conf, label) in [
        (Some("executor"), None, None, "--owner"),
        (None, Some("executor"), None, "ARMD_RT_OWNER"),
        (None, None, Some("executor"), "arm.armd_owner"),
    ] {
        let err_mit = resolve_startup_owner(cli, env, conf, "rebot-dm", &mit_dm).unwrap_err();
        assert!(err_mit.contains("MIT") && err_mit.contains(label), "{err_mit}");
        let err_dof = resolve_startup_owner(cli, env, conf, "mock", &over_dof).unwrap_err();
        assert!(err_dof.contains("DOF") && err_dof.contains(label), "{err_dof}");
    }
}

#[tokio::test]
async fn binary_startup_precedence_auto_resolution_and_diagnostics() {
    let tmp = tempfile::tempdir().unwrap();
    let app_cfg_legacy = tmp.path().join("app_legacy.json");
    std::fs::write(&app_cfg_legacy, r#"{"arm":{"armd_owner":"legacy"}}"#).unwrap();
    let app_cfg_exec = tmp.path().join("app_exec.json");
    std::fs::write(&app_cfg_exec, r#"{"arm":{"armd_owner":"executor"}}"#).unwrap();
    let mit_profile = tmp.path().join("mit_dm.json");
    std::fs::write(&mit_profile, mit_configured_rebot_dm_json()).unwrap();

    let client = reqwest::Client::builder()
        .timeout(Duration::from_secs(2))
        .build()
        .unwrap();

    let fetch_owner_fields = |port: u16| {
        let client = client.clone();
        async move {
            let deadline = Instant::now() + Duration::from_secs(5);
            loop {
                if let Ok(resp) = client
                    .get(format!("http://127.0.0.1:{port}/api/v1/observation"))
                    .send()
                    .await
                {
                    if resp.status().is_success() {
                        let obs: Value = resp.json().await.unwrap();
                        let diag: Value = client
                            .get(format!("http://127.0.0.1:{port}/api/v1/diagnostics"))
                            .send()
                            .await
                            .unwrap()
                            .json()
                            .await
                            .unwrap();
                        let tick: Value = client
                            .get(format!("http://127.0.0.1:{port}/api/v1/diagnostics/tick"))
                            .send()
                            .await
                            .unwrap()
                            .json()
                            .await
                            .unwrap();
                        return (obs, diag, tick);
                    }
                }
                assert!(Instant::now() < deadline, "timed out waiting for HTTP on port {port}");
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        }
    };

    // (a) Omitted owner on mock -> Auto -> executor (auto_qualified).
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(&["--follower", "mock"], &[], &[]);
        let (obs, diag, tick) = fetch_owner_fields(port).await;
        assert_eq!(obs["effective_owner"], EffectiveOwner::Executor.as_str());
        assert_eq!(obs["owner_reason"], OwnerReason::AutoQualified.as_str());
        assert_eq!(diag["effective_owner"], "executor");
        assert_eq!(diag["owner_reason"], "auto_qualified");
        assert_eq!(tick["owner_mode"], "executor");
        assert_eq!(tick["effective_owner"], "executor");
        assert_eq!(tick["owner_reason"], "auto_qualified");
    }

    // (b) Config arm.armd_owner="legacy" wins over default Auto.
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(
            &["--follower", "mock"],
            &[("ARMD_APP_CONFIG", app_cfg_legacy.to_str().unwrap())],
            &[],
        );
        let (obs, diag, tick) = fetch_owner_fields(port).await;
        assert_eq!(obs["effective_owner"], "legacy");
        assert_eq!(obs["owner_reason"], "explicit_config");
        assert_eq!(diag["effective_owner"], "legacy");
        assert_eq!(diag["owner_reason"], "explicit_config");
        assert_eq!(tick["owner_mode"], "legacy");
        assert_eq!(tick["effective_owner"], "legacy");
        assert_eq!(tick["owner_reason"], "explicit_config");
    }

    // (c) Env ARMD_RT_OWNER="executor" beats config arm.armd_owner="legacy".
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(
            &["--follower", "mock"],
            &[
                ("ARMD_APP_CONFIG", app_cfg_legacy.to_str().unwrap()),
                ("ARMD_RT_OWNER", "executor"),
            ],
            &[],
        );
        let (obs, diag, tick) = fetch_owner_fields(port).await;
        assert_eq!(obs["effective_owner"], "executor");
        assert_eq!(obs["owner_reason"], "explicit_env");
        assert_eq!(diag["effective_owner"], "executor");
        assert_eq!(diag["owner_reason"], "explicit_env");
        assert_eq!(tick["effective_owner"], "executor");
        assert_eq!(tick["owner_reason"], "explicit_env");
    }

    // (d) CLI --owner legacy beats env ARMD_RT_OWNER=executor and config arm.armd_owner=executor.
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(
            &["--follower", "mock", "--owner", "legacy"],
            &[
                ("ARMD_APP_CONFIG", app_cfg_exec.to_str().unwrap()),
                ("ARMD_RT_OWNER", "executor"),
            ],
            &[],
        );
        let (obs, diag, tick) = fetch_owner_fields(port).await;
        assert_eq!(obs["effective_owner"], "legacy");
        assert_eq!(obs["owner_reason"], "explicit_cli");
        assert_eq!(diag["effective_owner"], "legacy");
        assert_eq!(diag["owner_reason"], "explicit_cli");
        assert_eq!(tick["effective_owner"], "legacy");
        assert_eq!(tick["owner_reason"], "explicit_cli");
    }

    // (e) Omitted owner on MIT-configured profile -> Auto falls back to legacy with one log line naming reason.
    {
        let (_child, port, logs) = spawn_armd_and_wait_port(
            &["--follower", "mock", "--config", mit_profile.to_str().unwrap()],
            &[],
            &[],
        );
        let (obs, diag, tick) = fetch_owner_fields(port).await;
        assert_eq!(obs["effective_owner"], "legacy");
        assert_eq!(obs["owner_reason"], "auto_fallback_mit_profile");
        assert_eq!(diag["effective_owner"], "legacy");
        assert_eq!(diag["owner_reason"], "auto_fallback_mit_profile");
        assert_eq!(tick["effective_owner"], "legacy");
        assert_eq!(tick["owner_reason"], "auto_fallback_mit_profile");
        let joined = logs.lock().unwrap().join("\n");
        let fallback_lines: Vec<&str> = joined
            .lines()
            .filter(|l| l.contains("Startup owner auto resolved to legacy"))
            .collect();
        assert_eq!(
            fallback_lines.len(),
            1,
            "expected exactly one fallback log line, got:\n{joined}"
        );
        assert!(
            fallback_lines[0].contains("auto_fallback_mit_profile")
                && fallback_lines[0].contains("MIT"),
            "fallback log line must name reason: {}",
            fallback_lines[0]
        );
    }

    // (f) Explicit executor refused on ineligible followers & MIT profiles before opening hardware.
    for follower in ["so-arm101", "so-arm100", "rebot-rs", "fashionstar", "star-arm102", "sim"] {
        let err_cli = run_armd_expect_fail(&["--follower", follower, "--owner", "executor"], &[]);
        assert!(err_cli.contains(follower), "CLI refusal must name {follower}: {err_cli}");

        let err_env = run_armd_expect_fail(&["--follower", follower], &[("ARMD_RT_OWNER", "executor")]);
        assert!(err_env.contains(follower), "env refusal must name {follower}: {err_env}");

        let err_cfg = run_armd_expect_fail(
            &["--follower", follower],
            &[("ARMD_APP_CONFIG", app_cfg_exec.to_str().unwrap())],
        );
        assert!(err_cfg.contains(follower), "config refusal must name {follower}: {err_cfg}");
    }
    let err_mit_dm = run_armd_expect_fail(
        &[
            "--follower",
            "rebot-dm",
            "--config",
            mit_profile.to_str().unwrap(),
            "--owner",
            "executor",
        ],
        &[],
    );
    assert!(err_mit_dm.contains("MIT"), "MIT explicit executor refusal: {err_mit_dm}");

    // (f2) Invalid saved arm.armd_owner (e.g. typo "legcay") fails closed before opening hardware
    // instead of silently falling through to Auto -> executor.
    let app_cfg_typo = tmp.path().join("app_typo.json");
    std::fs::write(&app_cfg_typo, r#"{"arm":{"armd_owner":"legcay"}}"#).unwrap();
    let err_typo = run_armd_expect_fail(
        &["--follower", "mock"],
        &[("ARMD_APP_CONFIG", app_cfg_typo.to_str().unwrap())],
    );
    assert!(
        err_typo.contains("arm.armd_owner") && err_typo.contains("legcay"),
        "invalid saved arm.armd_owner must fail closed: {err_typo}"
    );

    // (g) Omitted owner on rebot-dm (non-MIT) positively resolves to executor (auto_qualified)
    // before opening the requested CAN port.
    #[cfg(target_os = "linux")]
    {
        let out_dm = run_armd_expect_fail(
            &["--follower", "rebot-dm", "--follower-port", "no_such_can_owner_auto"],
            &[],
        );
        assert!(
            out_dm.contains("Startup owner resolved: effective=executor reason=auto_qualified follower=rebot-dm"),
            "rebot-dm with omitted owner must positively log executor resolution:\n{out_dm}"
        );
        assert!(
            !out_dm.contains("Startup owner auto resolved to legacy"),
            "rebot-dm without MIT must not fall back to legacy under Auto:\n{out_dm}"
        );

        // With MIT profile, omitted owner on rebot-dm logs the MIT fallback reason before CAN open.
        let out_dm_mit = run_armd_expect_fail(
            &[
                "--follower",
                "rebot-dm",
                "--config",
                mit_profile.to_str().unwrap(),
                "--follower-port",
                "no_such_can_owner_auto",
            ],
            &[],
        );
        assert!(
            out_dm_mit.contains("Startup owner auto resolved to legacy (reason=auto_fallback_mit_profile)"),
            "expected MIT fallback log line on rebot-dm:\n{out_dm_mit}"
        );
    }

    // (h) Omitted owner on unqualified hardware follower logs unqualified-follower fallback before serial open.
    let absent_serial = tmp.path().join("missing-serial");
    let out_so101 = run_armd_expect_fail(
        &[
            "--follower",
            "so-arm101",
            "--follower-port",
            absent_serial.to_str().unwrap(),
        ],
        &[],
    );
    assert!(
        out_so101.contains("Startup owner auto resolved to legacy (reason=auto_fallback_unqualified_follower)"),
        "expected unqualified follower fallback log line on so-arm101:\n{out_so101}"
    );

    // (i) Forced OWNER_FAILED on mock: GET / and GET /api/v1/owner/status remain 200 OK while
    //     /api/v1/observation and /api/v1/diagnostics return 503 EXECUTOR_UNAVAILABLE.
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(
            &["--follower", "mock"],
            &[("ARMD_MOCK_TEST_OWNER_MODE", "failed")],
            &[],
        );
        let status_resp = client
            .get(format!("http://127.0.0.1:{port}/api/v1/owner/status"))
            .send()
            .await
            .unwrap();
        assert_eq!(status_resp.status(), 200);
        let owner_st: Value = status_resp.json().await.unwrap();
        assert_eq!(owner_st["effective_owner"], "failed");
        assert_eq!(owner_st["owner_state"], "failed");
        assert_eq!(owner_st["owner_reason"], "startup_failed_detached_safe_hold");
        assert_eq!(owner_st["is_fallback"], true);
        assert!(
            owner_st["legacy_only_routes"]["ws"]
                .as_array()
                .unwrap()
                .iter()
                .any(|v| v == "/api/v1/action/gripper_calibrate")
        );

        let index_resp = client
            .get(format!("http://127.0.0.1:{port}/"))
            .send()
            .await
            .unwrap();
        assert_eq!(index_resp.status(), 200);

        for gated in ["/api/v1/observation", "/api/v1/diagnostics"] {
            let r = client
                .get(format!("http://127.0.0.1:{port}{gated}"))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 503, "{gated} must return 503 under OWNER_FAILED");
            let body = r.text().await.unwrap();
            assert!(body.contains("EXECUTOR_UNAVAILABLE"), "{gated}: {body}");
        }
    }

    // (j) Forced startup_fallback on mock: GET /api/v1/owner/status reports legacy + startup_fallback_to_legacy.
    {
        let (_child, port, _logs) = spawn_armd_and_wait_port(
            &["--follower", "mock"],
            &[("ARMD_MOCK_TEST_OWNER_MODE", "startup_fallback")],
            &[],
        );
        let owner_st: Value = client
            .get(format!("http://127.0.0.1:{port}/api/v1/owner/status"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(owner_st["effective_owner"], "legacy");
        assert_eq!(owner_st["owner_state"], "legacy");
        assert_eq!(owner_st["owner_reason"], "startup_fallback_to_legacy");
        assert_eq!(owner_st["is_fallback"], true);
    }

    // (k) ARMD_MOCK_TEST_OWNER_MODE is refused on any non-mock follower.
    let err_non_mock = run_armd_expect_fail(
        &["--follower", "so-arm101", "--follower-port", absent_serial.to_str().unwrap()],
        &[("ARMD_MOCK_TEST_OWNER_MODE", "failed")],
    );
    assert!(
        err_non_mock.contains("ARMD_MOCK_TEST_OWNER_MODE") && err_non_mock.contains("mock"),
        "ARMD_MOCK_TEST_OWNER_MODE must refuse non-mock followers: {err_non_mock}"
    );
}

#[test]
fn cockpit_owner_badge_shows_in_expert_and_only_on_fallback_in_simple() {
    let html = include_str!("../src/api/index.html");
    assert!(html.contains("<button type=\"button\" class=\"sc-status-badge plain\" id=\"owner-badge\" hidden></button>"));
    assert!(html.contains("body.ui-simple #owner-badge:not(.is-fallback) { display: none !important; }"));
    assert!(html.contains("function renderOwnerStatus()"));
    for reason in [
        "default",
        "explicit_cli",
        "explicit_env",
        "explicit_config",
        "auto_qualified",
        "auto_fallback_unqualified_follower",
        "auto_fallback_mit_profile",
        "auto_fallback_dof_limit",
        "startup_fallback_to_legacy",
        "startup_failed_detached_safe_hold",
    ] {
        let key = format!("'owner.reason.{reason}'");
        assert!(
            html.matches(&key).count() >= 2,
            "missing bilingual I18N entry for {key} in src/api/index.html"
        );
    }

    // Execute the exact JS helper extracted from index.html when Node is available.
    if Command::new("node").arg("--version").output().is_ok_and(|o| o.status.success()) {
        let start = html
            .find("function isValidEffectiveOwner(")
            .expect("isValidEffectiveOwner present");
        let end = html[start..]
            .find("i18nHooks.push(renderOwnerStatus);")
            .expect("i18nHooks registration present")
            + start;
        let snippet = &html[start..end];
        let script = format!(
            r#"
let lastToast = null;
const mkEl = () => ({{
  hidden: true, textContent: '', className: '', title: '', attrs: {{}},
  setAttribute(k, v) {{ this.attrs[k] = v; }},
  addEventListener(ev, fn) {{ this['on_' + ev] = fn; }},
  classList: {{ contains(c) {{ return this.className.split(/\s+/).includes(c); }} }},
}});
const badge = mkEl();
badge.classList = {{ contains(c) {{ return badge.className.split(/\s+/).includes(c); }} }};
const banner = mkEl();
const bannerTitle = mkEl();
const bannerSub = mkEl();
const S = {{ obs: null, ownerDiag: null }};
const $ = (id) => ({{ 'owner-badge': badge, 'owner-banner': banner, 'owner-banner-title': bannerTitle, 'owner-banner-sub': bannerSub }}[id] || null);
const setText = (el, v) => {{ if (el) el.textContent = v; }};
const setCls = (el, v) => {{ if (el) el.className = v; }};
const toast = (msg, kind) => {{ lastToast = {{ msg, kind }}; }};
const apiCalls = [];
const api = async (path) => {{
  apiCalls.push(path);
  return {{
    effective_owner: 'failed',
    owner_state: 'failed',
    owner_reason: 'startup_failed_detached_safe_hold',
    detached_safe_hold: true,
    legacy_only_routes: {{ rest: ['/api/v1/diagnostics/power'], ws: ['/api/v1/action/gripper_calibrate'] }},
  }};
}};
const dict = {{
  'owner.badge.executor': 'RT',
  'owner.badge.legacy': 'Legacy',
  'owner.badge.fallback': 'Legacy · fallback',
  'owner.badge.failed': 'Owner · failed',
  'owner.badge.transferring': 'Switching',
  'owner.name.executor': 'RT executor',
  'owner.name.legacy': 'Legacy loop',
  'owner.name.failed': 'Owner failed (SAFE-HOLD)',
  'owner.name.transferring': 'Owner transferring',
  'owner.title': 'Control owner: {{owner}} ({{reason}})',
  'owner.reason.default': 'default',
  'owner.reason.auto_qualified': 'auto qualified',
  'owner.reason.explicit_cli': 'explicit cli',
  'owner.reason.auto_fallback_mit_profile': 'mit fallback',
  'owner.reason.startup_fallback_to_legacy': 'startup fallback',
  'owner.reason.startup_failed_detached_safe_hold': 'detached safe hold',
  'owner.banner.failed.title': 'RT executor failed to start (SAFE-HOLD)',
  'owner.banner.failed.sub': 'Cause: {{reason}}. Set arm.armd_owner=legacy and restart.',
  'owner.banner.failed.sub_generic': 'Cause: {{reason}}. Restart armd.',
  'owner.banner.startup_fallback.title': 'RT executor failed to start; fell back to legacy holding: {{reason}}',
  'owner.banner.startup_fallback.sub': 'Inspect armd logs or set arm.armd_owner=legacy.',
  'owner.banner.auto_fallback.title': 'Auto fallback to legacy: {{reason}}',
  'owner.banner.auto_fallback.sub': 'Use a qualified profile for RT executor.',
  'gb.legacy_only': 'Stroke calibration needs legacy mode: set arm.armd_owner=legacy and restart.',
  'gb.owner_unavailable': 'Control owner unavailable.',
  'owner.legacy_only.generic': 'Requires legacy mode.',
}};
const t = (k, p) => {{
  let s = dict[k] ?? k;
  if (p) for (const [a, b] of Object.entries(p)) s = s.replace('{{' + a + '}}', b);
  return s;
}};
{snippet}
function assert(cond, msg) {{ if (!cond) {{ console.error(msg); process.exit(1); }} }}

(async () => {{
  // 1. Executor auto_qualified: shown in expert, NOT marked is-fallback, banner hidden, LegacyOnly gated.
  S.obs = {{ effective_owner: 'executor', owner_reason: 'auto_qualified' }};
  renderOwnerStatus();
  assert(badge.hidden === false, 'executor badge must be unhidden');
  assert(badge.textContent === 'RT', 'executor text: ' + badge.textContent);
  assert(!badge.className.includes('is-fallback'), 'executor must not be is-fallback');
  assert(badge.attrs['aria-label'] === badge.title, 'aria-label must match title');
  assert(banner.hidden === true, 'normal executor must hide owner banner');
  assert(
    legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only')?.includes('arm.armd_owner=legacy'),
    'gripper_calibrate must be blocked on executor'
  );
  assert(
    legacyOnlyRouteBlock('/api/v1/action/gripper_zero_check', 'gb.legacy_only') === null,
    'non-LegacyOnly route must not be blocked on executor'
  );

  // 2. Explicit legacy: shown in expert, NOT marked is-fallback, banner hidden, LegacyOnly allowed.
  S.obs = {{ effective_owner: 'legacy', owner_reason: 'explicit_cli' }};
  renderOwnerStatus();
  assert(badge.hidden === false, 'explicit legacy badge must be unhidden');
  assert(badge.textContent === 'Legacy', 'explicit legacy text: ' + badge.textContent);
  assert(!badge.className.includes('is-fallback'), 'explicit legacy must not be is-fallback');
  assert(banner.hidden === true, 'explicit legacy must hide owner banner');
  assert(
    legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only') === null,
    'gripper_calibrate must be enabled on legacy'
  );

  // 3. Auto fallback to legacy (MIT): badge is-fallback + is-warn AND banner visible with cause + next step.
  S.obs = {{ effective_owner: 'legacy', owner_reason: 'auto_fallback_mit_profile' }};
  renderOwnerStatus();
  assert(badge.hidden === false, 'fallback badge must be unhidden');
  assert(badge.textContent === 'Legacy · fallback', 'fallback text: ' + badge.textContent);
  assert(badge.className.includes('is-fallback') && badge.className.includes('is-warn'), 'fallback classes: ' + badge.className);
  assert(badge.title.includes('mit fallback'), 'fallback title: ' + badge.title);
  assert(banner.hidden === false && bannerTitle.textContent.includes('mit fallback'), 'auto fallback banner must show');
  badge.on_click();
  assert(lastToast && lastToast.kind === 'warn' && lastToast.msg.includes('mit fallback'), 'click must surface reason via toast');

  // 4. Startup DRV-F1 fallback to legacy: badge + startup fallback banner visible.
  S.obs = {{ effective_owner: 'legacy', owner_reason: 'startup_fallback_to_legacy' }};
  renderOwnerStatus();
  assert(badge.className.includes('is-fallback') && badge.className.includes('is-warn'), 'startup fallback classes: ' + badge.className);
  assert(banner.hidden === false && bannerTitle.textContent.includes('startup fallback'), 'startup fallback banner title: ' + bannerTitle.textContent);
  assert(bannerSub.textContent.includes('arm.armd_owner'), 'startup fallback banner must include next step');

  // 5. Detached SAFE-HOLD failed owner with NO observation (OWNER_FAILED rejects /observation & /diagnostics):
  //    loadOwnerDiagnostics() fetches /api/v1/owner/status and renders failed badge + danger banner.
  S.obs = null;
  S.ownerDiag = null;
  await loadOwnerDiagnostics();
  assert(apiCalls[0] === '/api/v1/owner/status', 'must query /api/v1/owner/status first, got: ' + apiCalls[0]);
  assert(badge.hidden === false, 'failed owner via owner/status must be unhidden');
  assert(badge.textContent === 'Owner · failed', 'failed text: ' + badge.textContent);
  assert(badge.className.includes('is-fallback') && badge.className.includes('is-warn'), 'failed classes: ' + badge.className);
  assert(badge.title.includes('detached safe hold'), 'failed title: ' + badge.title);
  assert(banner.hidden === false && banner.className.includes('is-danger'), 'failed banner must be visible danger');
  assert(banner.attrs.role === 'alert', 'failed banner must use role=alert');
  assert(bannerSub.textContent.includes('detached safe hold') && bannerSub.textContent.includes('arm.armd_owner'), 'failed banner must explain cause + next step');

  // 6. Dynamic capability-driven LegacyOnly gating: if armd migrates a route out of legacy_only_routes, gate lifts.
  S.obs = {{ effective_owner: 'executor', owner_reason: 'auto_qualified' }};
  S.ownerDiag = {{ effective_owner: 'executor', owner_reason: 'auto_qualified', legacy_only_routes: {{ rest: [], ws: [] }} }};
  assert(legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only') === null, 'gate must lift when route leaves legacy_only_routes');
}})().catch((e) => {{ console.error(e); process.exit(1); }});
"#
        );
        let out = Command::new("node").args(["-e", &script]).output().unwrap();
        assert!(
            out.status.success(),
            "cockpit owner badge JS test failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
}
