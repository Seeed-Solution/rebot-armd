//! `--test-fail-executor-startup`: validation of the flag (binary, before any hardware is
//! opened) and default-off behavior. The recoverable handback triggered by
//! `PersistTestHooks::fail_control_spawn` is tested through `start_control_owner_with_hooks`
//! in `src/main.rs`.

use std::process::{Command, Output, Stdio};
use std::time::{Duration, Instant};

/// Runs armd with `args` under a pinned environment, killing it after 20 s; never binds a fixed port.
fn run_armd(args: &[&str]) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_armd"));
    cmd.env_clear();
    if let Some(path) = std::env::var_os("PATH") {
        cmd.env("PATH", path);
    }
    if let Some(ld) = std::env::var_os("LD_LIBRARY_PATH") {
        cmd.env("LD_LIBRARY_PATH", ld);
    }
    cmd.env("RUST_LOG", "info");
    let mut child = cmd
        .args(["--port", "0", "--log-dir", "off", "--blackbox-dir", "off", "--episodes-dir", "off"])
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(20);
    while child.try_wait().unwrap().is_none() {
        if Instant::now() > deadline {
            let _ = child.kill();
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    child.wait_with_output().unwrap()
}

fn text(o: &Output) -> String {
    format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr))
}

#[test]
fn flag_is_refused_for_followers_other_than_rebot_dm() {
    for follower in ["mock", "sim", "so-arm101"] {
        let o = run_armd(&["--follower", follower, "--test-fail-executor-startup"]);
        assert!(!o.status.success(), "{follower}: {}", text(&o));
        assert!(text(&o).contains("--test-fail-executor-startup is only permitted with '--follower rebot-dm'"), "{follower}: {}", text(&o));
    }
}

#[test]
fn flag_is_refused_when_the_resolved_owner_is_legacy() {
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "legacy", "--test-fail-executor-startup"]);
    assert!(!o.status.success(), "{}", text(&o));
    assert!(text(&o).contains("needs the RT executor as owner (resolved owner: legacy)"), "{}", text(&o));
}

#[test]
fn flag_is_off_by_default_and_armed_only_by_its_explicit_name() {
    // Without the flag armd on rebot-dm reaches the missing CAN interface and never logs ARMED.
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "executor"]);
    let out = text(&o);
    assert!(!o.status.success(), "{out}");
    assert!(out.contains("can_missing0"), "expected missing-CAN failure: {out}");
    assert!(!out.contains("ARMED"), "{out}");
    // a near-miss spelling is an unknown argument
    let o = run_armd(&["--follower", "mock", "--fail-executor-startup"]);
    assert!(!o.status.success());
    assert!(text(&o).contains("Unknown argument: --fail-executor-startup"), "{}", text(&o));
}

#[test]
fn flag_with_rebot_dm_and_executor_owner_passes_validation_and_logs_error_before_hardware() {
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "executor", "--test-fail-executor-startup"]);
    let out = text(&o);
    assert!(!o.status.success(), "{out}");
    assert!(!out.contains("is only permitted with") && !out.contains("needs the RT executor as owner"), "{out}");
    assert!(out.contains("--test-fail-executor-startup is ARMED"), "{out}");
    assert!(out.contains("can_missing0"), "expected missing-CAN failure: {out}");
}
