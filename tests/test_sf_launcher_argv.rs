//! End-to-end tests of `K92rebot-teleop` config loading and argv construction (`set --` + fixed
//! exec wrapper): captures exact NUL-separated `argv` at a fake daemon on both `start` and
//! `restart`, verifies spaces, shell metacharacters, embedded and standalone
//! `--test-fail-executor-startup` spellings never split into extra flags or execute shell syntax,
//! and feeds captured argv to the real `armd` binary to confirm D2 stays disarmed.

#![cfg(unix)]

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};
use std::time::{Duration, Instant};

const K92_SRC: &str =
    include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");

struct LauncherHarness {
    dir: tempfile::TempDir,
    bin_dir: PathBuf,
    init_path: PathBuf,
    config_path: PathBuf,
    pidfile: PathBuf,
    capture_path: PathBuf,
}

impl LauncherHarness {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();
        for sub in [
            "bin",
            "etc/init.d",
            "userdata/local/apps",
            "userdata/rebot-teleop/logs",
            "userdata/rebot-teleop/blackbox",
            "userdata/rebot-teleop/episodes",
            "usr/local/bin",
            "usr/share/rebot-teleop/profile",
            "var/run",
            "var/log",
            "dev",
        ] {
            std::fs::create_dir_all(root.join(sub)).unwrap();
        }
        let r = root.display().to_string();
        let rebased = K92_SRC
            .replace("/etc/init.d/", &format!("{r}/etc/init.d/"))
            .replace("/userdata/", &format!("{r}/userdata/"))
            .replace("/usr/share/", &format!("{r}/usr/share/"))
            .replace("/usr/bin/", &format!("{r}/usr/bin/"))
            .replace("/usr/local/bin/", &format!("{r}/usr/local/bin/"))
            .replace("/var/run/", &format!("{r}/var/run/"))
            .replace("/var/log/", &format!("{r}/var/log/"))
            .replace("/dev/tty", &format!("{r}/dev/tty"));
        let init_path = root.join("etc/init.d/K92rebot-teleop");
        write_exec(&init_path, &rebased);

        let bin_dir = root.join("bin");
        let pidfile = root.join("var/run/rebot-teleop.pid");
        let daemon = root.join("usr/local/bin/armd");
        let capture_path = root.join("argv_capture.bin");

        // Fake armd: writes NUL-separated sys.argv[1:] atomically, then waits for SIGTERM.
        write_exec(
            &daemon,
            r#"#!/usr/bin/env python3
import os, signal, sys, time
out = os.environ["ARGV_CAPTURE"]
tmp = out + ".tmp"
with open(tmp, "wb") as f:
    f.write(b"\x00".join(a.encode("utf-8") for a in sys.argv[1:]))
os.replace(tmp, out)
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
for _ in range(300):
    time.sleep(0.1)
"#,
        );

        // Minimal start-stop-daemon matching K92's `-S -b -m -p <pidfile> -a /bin/sh -- ...`.
        write_exec(
            &bin_dir.join("start-stop-daemon"),
            "#!/bin/sh\n\
             pidfile=''\n\
             prog=''\n\
             while [ $# -gt 0 ]; do\n\
                 case \"$1\" in\n\
                     -p) pidfile=\"$2\"; shift 2 ;;\n\
                     -a) prog=\"$2\"; shift 2 ;;\n\
                     --) shift; break ;;\n\
                     *) shift ;;\n\
                 esac\n\
             done\n\
             \"$prog\" \"$@\" </dev/null >/dev/null 2>&1 &\n\
             pid=$!\n\
             [ -n \"$pidfile\" ] && printf '%s\\n' \"$pid\" > \"$pidfile\"\n\
             exit 0\n",
        );

        // Map `/proc/<fake_pid>/exe` to the fake daemon path so `stop` identity check passes.
        write_exec(
            &bin_dir.join("readlink"),
            &format!(
                "#!/bin/sh\n\
                 if [ \"$#\" -eq 1 ] && [ -f '{pf}' ]; then\n\
                     pid=$(cat '{pf}' 2>/dev/null)\n\
                     if [ \"$1\" = \"/proc/$pid/exe\" ]; then\n\
                         printf '%s\\n' '{dm}'\n\
                         exit 0\n\
                     fi\n\
                 fi\n\
                 exec /usr/bin/readlink \"$@\"\n",
                pf = pidfile.display(),
                dm = daemon.display()
            ),
        );

        // Isolate `pidof armd` to this test harness's pidfile so host processes never interfere.
        write_exec(
            &bin_dir.join("pidof"),
            &format!(
                "#!/bin/sh\n\
                 if [ \"$1\" = armd ] && [ -f '{pf}' ]; then\n\
                     pid=$(cat '{pf}' 2>/dev/null)\n\
                     if [ -n \"$pid\" ] && [ -d \"/proc/$pid\" ]; then\n\
                         printf '%s\\n' \"$pid\"\n\
                         exit 0\n\
                     fi\n\
                 fi\n\
                 exit 1\n",
                pf = pidfile.display()
            ),
        );

        // Fast bounded sleep so K92 start/restart completes quickly while letting the fake daemon exec.
        write_exec(
            &bin_dir.join("sleep"),
            "#!/bin/sh\nexec /usr/bin/python3 -c 'import time; time.sleep(0.05)'\n",
        );
        for noop in ["ip", "iw", "ifconfig", "logger", "chrt"] {
            write_exec(&bin_dir.join(noop), "#!/bin/sh\nexit 0\n");
        }

        let config_path = root.join("userdata/local/apps/rebot-teleop.config.json");
        Self {
            dir,
            bin_dir,
            init_path,
            config_path,
            pidfile,
            capture_path,
        }
    }

    fn write_config(&self, json: &serde_json::Value) {
        std::fs::write(&self.config_path, serde_json::to_vec(json).unwrap()).unwrap();
    }

    fn run_k92(&self, action: &str) -> Output {
        let base_path = std::env::var("PATH").unwrap_or_default();
        Command::new("/bin/sh")
            .arg(&self.init_path)
            .arg(action)
            .env("PATH", format!("{}:{base_path}", self.bin_dir.display()))
            .env("ARGV_CAPTURE", &self.capture_path)
            .env_remove("ARMD_RT_OWNER")
            .output()
            .unwrap()
    }

    fn capture_argv(&self, action: &str) -> (u32, Vec<String>) {
        let _ = std::fs::remove_file(&self.capture_path);
        let out = self.run_k92(action);
        let log = std::fs::read_to_string(self.dir.path().join("var/log/rebot-teleop.log"))
            .unwrap_or_default();
        assert!(
            out.status.success(),
            "K92 {action} failed: stdout={} stderr={} log={log}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let deadline = Instant::now() + Duration::from_secs(3);
        while !self.capture_path.exists() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        let raw = std::fs::read(&self.capture_path).expect("fake daemon must write NUL argv");
        let tokens = raw
            .split(|&b| b == 0)
            .map(|s| String::from_utf8(s.to_vec()).unwrap())
            .collect();
        let pid: u32 = std::fs::read_to_string(&self.pidfile)
            .unwrap()
            .trim()
            .parse()
            .unwrap();
        (pid, tokens)
    }

    fn expected_tail(&self) -> Vec<String> {
        let r = self.dir.path();
        vec![
            "--log-dir".into(),
            r.join("userdata/rebot-teleop/logs").display().to_string(),
            "--blackbox-dir".into(),
            r.join("userdata/rebot-teleop/blackbox").display().to_string(),
            "--episodes-dir".into(),
            r.join("userdata/rebot-teleop/episodes").display().to_string(),
        ]
    }
}

impl Drop for LauncherHarness {
    fn drop(&mut self) {
        if let Ok(text) = std::fs::read_to_string(&self.pidfile) {
            if let Ok(pid) = text.trim().parse::<i32>() {
                unsafe {
                    libc::kill(pid, libc::SIGKILL);
                }
            }
        }
    }
}

fn write_exec(path: &Path, body: &str) {
    std::fs::write(path, body).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

/// Run the real `armd` binary with captured argv (swapping `--port` only when `override_port` is
/// requested) and assert it exits non-zero without arming `--test-fail-executor-startup`.
fn assert_armd_disarmed(argv: &[String], override_port: bool) -> String {
    let mut args = argv.to_vec();
    if override_port {
        if let Some(pos) = args.iter().position(|a| a == "--port") {
            if pos + 1 < args.len() {
                args[pos + 1] = "0".into();
            }
        }
    }
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_armd"));
    cmd.env_clear();
    if let Some(p) = std::env::var_os("PATH") {
        cmd.env("PATH", p);
    }
    if let Some(ld) = std::env::var_os("LD_LIBRARY_PATH") {
        cmd.env("LD_LIBRARY_PATH", ld);
    }
    cmd.env("RUST_LOG", "info");
    let mut child = cmd
        .args(&args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while child.try_wait().unwrap().is_none() {
        if Instant::now() > deadline {
            let _ = child.kill();
            break;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
    let out = child.wait_with_output().unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(!out.status.success(), "unexpected armd success: {combined}");
    assert!(
        !combined.contains("ARMED"),
        "D2 flag must stay disarmed for packaged launcher argv {args:?}: {combined}"
    );
    combined
}

#[test]
fn packaged_launcher_never_emits_d2_flag_or_uses_shell_reparsed_daemon_opts() {
    assert!(
        !K92_SRC.contains("--test-fail-executor-startup"),
        "packaged K92 launcher must never emit the attended D2 flag"
    );
    assert!(
        !K92_SRC.contains("DAEMON_OPTS"),
        "packaged K92 launcher must build argv with positional parameters, not DAEMON_OPTS"
    );
}

#[test]
fn k92_start_and_restart_preserve_metacharacters_spaces_and_embedded_d2_tokens() {
    let h = LauncherHarness::new();
    let sentinel = h.dir.path().join("pwned");
    let follower_meta = format!("rebot-dm ; touch {} $(echo x) 'a b' \"c\" *", sentinel.display());
    let port_meta = format!("can_missing0 | touch {} --test-fail-executor-startup", sentinel.display());
    let http_meta = format!("8765 $(touch {}) --test-fail-executor-startup", sentinel.display());
    h.write_config(&serde_json::json!({
        "arm": {
            "follower_model": follower_meta,
            "serial_port": port_meta,
            "armd_owner": "executor"
        },
        "port": http_meta
    }));

    let mut expected = vec![
        "--follower".into(),
        follower_meta,
        "--follower-port".into(),
        port_meta,
        "--port".into(),
        http_meta,
    ];
    expected.extend(h.expected_tail());
    expected.extend(["--owner".into(), "executor".into()]);

    let (pid1, argv1) = h.capture_argv("start");
    assert_eq!(argv1, expected, "start argv must match exact single tokens");
    assert!(!sentinel.exists(), "shell metacharacters must not execute on start");
    let (pid2, argv2) = h.capture_argv("restart");
    assert_ne!(pid1, pid2, "restart must spawn a new daemon instance");
    assert_eq!(argv2, expected, "restart argv must match exact single tokens");
    assert!(!sentinel.exists(), "shell metacharacters must not execute on restart");
    assert!(
        !argv2.iter().any(|t| t == "--test-fail-executor-startup"),
        "embedded D2 spelling must never split into a standalone flag: {argv2:?}"
    );
    let out_port = assert_armd_disarmed(&argv2, false);
    assert!(out_port.contains("Invalid port"), "expected invalid port rejection: {out_port}");
    let out_follower = assert_armd_disarmed(&argv2, true);
    assert!(
        out_follower.contains(&expected[1].replace('"', "\\\"")),
        "expected intact metacharacter follower string in armd rejection: {out_follower}"
    );
}

#[test]
fn k92_embedded_and_standalone_d2_spellings_keep_d2_disarmed_on_start_and_restart() {
    let h = LauncherHarness::new();

    // Embedded D2 spelling inside serial_port with real `rebot-dm` + `executor`:
    let embedded = "can_missing0 --test-fail-executor-startup";
    h.write_config(&serde_json::json!({
        "arm": { "follower_model": "rebot-dm", "serial_port": embedded, "armd_owner": "executor" },
        "port": 8765
    }));
    let mut want_embedded = vec![
        "--follower".into(),
        "rebot-dm".into(),
        "--follower-port".into(),
        embedded.into(),
        "--port".into(),
        "8765".into(),
    ];
    want_embedded.extend(h.expected_tail());
    want_embedded.extend(["--owner".into(), "executor".into()]);

    let (p1, a1) = h.capture_argv("start");
    assert_eq!(a1, want_embedded);
    let (p2, a2) = h.capture_argv("restart");
    assert_ne!(p1, p2);
    assert_eq!(a2, want_embedded);
    assert!(!a2.iter().any(|t| t == "--test-fail-executor-startup"));
    assert_armd_disarmed(&a2, true);

    // Standalone D2 spelling saved directly as serial_port (with valid rebot-dm + executor):
    h.write_config(&serde_json::json!({
        "arm": {
            "follower_model": "rebot-dm",
            "serial_port": "--test-fail-executor-startup",
            "armd_owner": "executor"
        },
        "port": 8765
    }));
    let mut want_standalone_port = vec![
        "--follower".into(),
        "rebot-dm".into(),
        "--follower-port".into(),
        "--test-fail-executor-startup".into(),
        "--port".into(),
        "8765".into(),
    ];
    want_standalone_port.extend(h.expected_tail());
    want_standalone_port.extend(["--owner".into(), "executor".into()]);

    let (_, a3) = h.capture_argv("restart");
    assert_eq!(a3, want_standalone_port);
    let out3 = assert_armd_disarmed(&a3, true);
    assert!(out3.contains("Missing value for --follower-port"), "{out3}");

    // Standalone D2 spelling saved across follower_model, serial_port, and port:
    h.write_config(&serde_json::json!({
        "arm": {
            "follower_model": "--test-fail-executor-startup",
            "serial_port": "--test-fail-executor-startup"
        },
        "port": "--test-fail-executor-startup"
    }));
    let mut want_all_standalone = vec![
        "--follower".into(),
        "--test-fail-executor-startup".into(),
        "--follower-port".into(),
        "--test-fail-executor-startup".into(),
        "--port".into(),
        "--test-fail-executor-startup".into(),
    ];
    want_all_standalone.extend(h.expected_tail());

    let (_, a4) = h.capture_argv("restart");
    assert_eq!(a4, want_all_standalone);
    let out4 = assert_armd_disarmed(&a4, false);
    assert!(out4.contains("Missing value for --follower"), "{out4}");
}
