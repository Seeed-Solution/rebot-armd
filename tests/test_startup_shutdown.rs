use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

fn rejected(args: &[&str]) -> String {
    let mut child = Command::new(env!("CARGO_BIN_EXE_armd"))
        .args(args)
        .env_remove("PORT")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(
                !status.success(),
                "invalid hardware/configuration unexpectedly succeeded"
            );
            let output = child.wait_with_output().unwrap();
            return format!(
                "{}{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
        }
        if Instant::now() > deadline {
            // Only this test's child, started with invalid/nonexistent hardware parameters.
            let _ = child.kill();
            let _ = child.wait();
            panic!("invalid configuration started a long-running daemon: {args:?}");
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn malformed_or_unimplemented_cli_modes_fail_closed() {
    assert!(rejected(&["--follower", "unknown", "--port", "0"]).contains("Unknown follower"));
    assert!(rejected(&["--follower"]).contains("Missing value"));
    assert!(rejected(&["--port", "not-a-port"]).contains("Invalid port"));
    assert!(rejected(&["--leader", "star-arm102"]).contains("not implemented"));
    assert!(rejected(&["--unexpected"]).contains("Unknown argument"));
    assert!(rejected(&["--follower", "mock", "--follower-port", "can0"])
        .contains("requires a hardware follower"));
}

#[test]
fn hardware_open_errors_never_fall_back_to_mock() {
    let temp = tempfile::tempdir().unwrap();
    let absent = temp.path().join("missing-serial-port");
    for model in ["so-arm101", "fashionstar", "star-arm102"] {
        let output = rejected(&[
            "--follower",
            model,
            "--follower-port",
            absent.to_str().unwrap(),
            "--port",
            "0",
        ]);
        assert!(
            !output.contains("Unknown follower"),
            "{model} must be a supported selector"
        );
        assert!(!output.contains("mock simulation"));
    }
    #[cfg(target_os = "linux")]
    for model in ["rebot-dm", "rebot-rs"] {
        let output = rejected(&[
            "--follower",
            model,
            "--follower-port",
            "no_such_can",
            "--port",
            "0",
        ]);
        assert!(!output.contains("mock simulation"));
    }
}

#[test]
fn profile_is_validated_before_hardware_is_opened() {
    let temp = tempfile::tempdir().unwrap();
    let file = temp.path().join("profile.json");
    let mut config = armd::ArmConfig::default();
    std::fs::write(&file, serde_json::to_vec(&config).unwrap()).unwrap();
    let output = rejected(&[
        "--follower",
        "rebot-dm",
        "--config",
        file.to_str().unwrap(),
        "--port",
        "0",
    ]);
    assert!(output.contains("does not match"));
    config.control_frequency_hz = 0;
    std::fs::write(&file, serde_json::to_vec(&config).unwrap()).unwrap();
    assert!(
        rejected(&["--config", file.to_str().unwrap(), "--port", "0"]).contains("Invalid control")
    );
}

#[cfg(target_os = "linux")]
#[tokio::test]
async fn mock_shutdown_does_not_wait_for_an_open_websocket() {
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = reservation.local_addr().unwrap().port();
    drop(reservation);
    let mut child = Command::new(env!("CARGO_BIN_EXE_armd"))
        .args(["--follower", "mock", "--port", &port.to_string()])
        .env_remove("PORT")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let url = format!("http://127.0.0.1:{port}/api/v1/observation");
    let client = reqwest::Client::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let ready = loop {
        if client.get(&url).send().await.is_ok() {
            break true;
        }
        if Instant::now() > deadline {
            break false;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    if !ready {
        let _ = child.kill();
        let _ = child.wait();
        panic!("mock server did not start");
    }
    let connection =
        tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/api/v1/ws")).await;
    if let Err(error) = connection {
        let _ = child.kill();
        let _ = child.wait();
        panic!("mock websocket did not connect: {error}");
    }
    let (mut socket, _) = connection.unwrap();
    use futures_util::{SinkExt, StreamExt};
    use tokio_tungstenite::tungstenite::Message;
    let armed = tokio::time::timeout(Duration::from_secs(2), async {
        socket.send(Message::Text(serde_json::json!({"type":"acquire_lease", "client_id":"shutdown-test"}).to_string().into())).await?;
        let generation = loop {
            let message = socket.next().await.ok_or("socket closed")??;
            let value: serde_json::Value = serde_json::from_str(message.to_text()?)?;
            if value["type"] == "lease_granted" { break value["generation"].as_u64().ok_or("generation missing")?; }
        };
        socket.send(Message::Text(serde_json::json!({"type":"command", "generation":generation, "epoch_id":1, "request_id":1,
            "path":"/api/v1/torque", "body":{"enable":true}}).to_string().into())).await?;
        loop {
            let message = socket.next().await.ok_or("socket closed")??;
            let value: serde_json::Value = serde_json::from_str(message.to_text()?)?;
            if value["type"] == "command_ack" {
                if value["is_torqued"] != true { return Err("mock was not armed".into()); }
                break;
            }
            if value["type"] == "command_reject" { return Err(format!("arm rejected: {value}").into()); }
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    }).await;
    if !matches!(armed, Ok(Ok(()))) {
        let _ = child.kill();
        let _ = child.wait();
        panic!("mock setup failed: {armed:?}");
    }
    assert_eq!(unsafe { libc::kill(child.id() as i32, libc::SIGTERM) }, 0);
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        if Instant::now() > deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("open websocket blocked safe mock shutdown");
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[cfg(unix)]
#[test]
fn init_stop_refuses_unknown_or_uncooperative_processes_without_killing() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    assert!(source.contains("restart) stop || exit $?;"));
    let start = source.find("stop_video() {").unwrap();
    let end = source.find("\nstatus() {").unwrap();
    for mode in ["missing", "wrong_identity", "timeout"] {
        let temp = tempfile::tempdir().unwrap();
        let proc_root = temp.path().join("proc");
        let pidfile = temp.path().join("armd.pid");
        let signals = temp.path().join("signals");
        if mode != "missing" {
            std::fs::create_dir_all(proc_root.join("424242")).unwrap();
            std::fs::write(&pidfile, "424242\n").unwrap();
            std::os::unix::fs::symlink(
                if mode == "wrong_identity" {
                    "/unrelated/process"
                } else {
                    "/mock/armd"
                },
                proc_root.join("424242/exe"),
            )
            .unwrap();
        }
        let functions = source[start..end].replace("/proc/", &format!("{}/", proc_root.display()));
        let script = format!("NAME=test\nDAEMON=/mock/armd\nPIDFILE='{}'\nkill() {{ echo \"$*\" >> '{}'; }}\nsleep() {{ :; }}\npidof() {{ echo 424242; }}\n{}\nstop\n", pidfile.display(), signals.display(), functions);
        // This isolated copy stubs every process signal and delay and uses a fake /proc tree.
        let output = Command::new("sh").args(["-c", &script]).output().unwrap();
        assert!(
            !output.status.success(),
            "{mode}: {}",
            String::from_utf8_lossy(&output.stdout)
        );
        if mode == "timeout" {
            assert_eq!(
                std::fs::read_to_string(&signals).unwrap().trim(),
                "-TERM 424242"
            );
            assert!(
                pidfile.exists(),
                "failed stop must preserve ownership record"
            );
        } else {
            assert!(
                !signals.exists(),
                "unverified process must not receive signals"
            );
        }
    }
}

#[cfg(unix)]
#[test]
fn init_start_does_not_reconfigure_buses_while_an_untracked_daemon_exists() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    let start = source.find("start() {").unwrap();
    let end = source.find("\n# Stop only the video process").unwrap();
    let temp = tempfile::tempdir().unwrap();
    let marker = temp.path().join("network-touched");
    let script = format!("NAME=test\nPIDFILE='{}'\npidof() {{ echo 424242; }}\nstart_wifi() {{ echo called > '{}'; }}\n{}\nstart\n", temp.path().join("missing.pid").display(), marker.display(), &source[start..end]);
    let result = Command::new("sh").args(["-c", &script]).output().unwrap();
    assert!(!result.status.success());
    assert!(
        !marker.exists(),
        "start must refuse before network/CAN setup"
    );
}

#[cfg(unix)]
fn init_section(source: &str, from: &str, to: &str) -> String {
    let start = source.find(from).unwrap();
    let end = start + source[start..].find(to).unwrap();
    source[start..end].to_string()
}

#[cfg(unix)]
#[test]
fn init_stops_only_running_camera_holders_with_their_own_scripts() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    let temp = tempfile::tempdir().unwrap();
    let (etc, run, proc_root, calls) = (temp.path().join("init.d"), temp.path().join("run"), temp.path().join("proc"), temp.path().join("calls"));
    for d in [&etc, &run, &proc_root] {
        std::fs::create_dir_all(d).unwrap();
    }
    // yolo-detector runs (live pid), aruco-detector is installed but stopped, fall-detection has a stale pidfile
    for (app, pid) in [("yolo-detector", Some("1001")), ("aruco-detector", None), ("fall-detection", Some("1003"))] {
        let script = etc.join(format!("K92{app}"));
        std::fs::write(&script, format!("#!/bin/sh\necho \"{app} $1\" >> '{}'\n", calls.display())).unwrap();
        std::fs::set_permissions(&script, std::os::unix::fs::PermissionsExt::from_mode(0o755)).unwrap();
        if let Some(p) = pid {
            std::fs::write(run.join(format!("{app}.pid")), format!("{p}\n")).unwrap();
        }
    }
    std::fs::create_dir_all(proc_root.join("1001")).unwrap();
    let functions = init_section(source, "stop_camera_holders() {", "\nport_busy() {")
        .replace("/var/run/", &format!("{}/", run.display()))
        .replace("/etc/init.d/", &format!("{}/", etc.display()))
        .replace("/proc/", &format!("{}/", proc_root.display()));
    let apps = source.lines().find(|l| l.starts_with("CAMERA_APPS=")).unwrap();
    let script = format!("NAME=test\n{apps}\nsleep() {{ :; }}\nlogger() {{ :; }}\n{functions}\nstop_camera_holders\n");
    let out = Command::new("sh").args(["-c", &script]).output().unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    assert_eq!(std::fs::read_to_string(&calls).unwrap(), "yolo-detector stop\n");
    assert!(String::from_utf8_lossy(&out.stdout).contains("stopped camera holder yolo-detector"));
}

#[cfg(unix)]
#[test]
fn init_stop_vision_signals_only_its_respawn_loop() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    // (cmdline of pid 777, a child of 777 exists, expected signals)
    for (cmdline, with_child, expected) in [
        ("/bin/sh\0/usr/share/rebot-teleop/rebot-vision-run\0", true, Some("-TERM 777")),
        // alive without a child for 1 s after SIGTERM: hung, SIGKILL to the loop only
        ("/bin/sh\0/usr/share/rebot-teleop/rebot-vision-run\0", false, Some("-TERM 777\n-KILL 777")),
        ("/usr/bin/other\0", false, None),
    ] {
        let temp = tempfile::tempdir().unwrap();
        let proc_root = temp.path().join("proc");
        std::fs::create_dir_all(proc_root.join("777")).unwrap();
        std::fs::write(proc_root.join("777/cmdline"), cmdline).unwrap();
        std::fs::write(proc_root.join("777/status"), "Name:\tsh\nPPid:\t1\n").unwrap();
        if with_child {
            std::fs::create_dir_all(proc_root.join("778")).unwrap();
            std::fs::write(proc_root.join("778/status"), "Name:\trebot-vision\nPPid:\t777\n").unwrap();
        }
        let pidfile = temp.path().join("vision.pid");
        std::fs::write(&pidfile, "777\n").unwrap();
        let signals = temp.path().join("signals");
        let functions = init_section(source, "has_child() {", "\nstop() {").replace("/proc/", &format!("{}/", proc_root.display()));
        let script = format!(
            "NAME=test\nVISION_PIDFILE='{}'\nVISION_RUNNER=/usr/share/rebot-teleop/rebot-vision-run\nVISION_STOP_TICKS=8\nkill() {{ echo \"$*\" >> '{}'; }}\nsleep() {{ :; }}\nlogger() {{ :; }}\n{functions}\nstop_vision\n",
            pidfile.display(),
            signals.display()
        );
        let out = Command::new("sh").args(["-c", &script]).output().unwrap();
        assert!(out.status.success());
        match expected {
            Some(e) => assert_eq!(std::fs::read_to_string(&signals).unwrap().trim(), e, "child: {with_child}"),
            None => {
                assert!(!signals.exists(), "a pidfile naming another process must not be signalled");
                assert!(pidfile.exists());
            }
        }
    }
}

/// `busybox ash` (the reCamera /bin/sh) when the host has busybox, else `sh`.
#[cfg(unix)]
fn device_shell() -> Command {
    if Command::new("busybox").arg("true").status().map(|s| s.success()).unwrap_or(false) {
        let mut c = Command::new("busybox");
        c.arg("ash");
        c
    } else {
        Command::new("sh")
    }
}

/// utime + stime of a process in clock ticks (/proc/<pid>/stat fields 14 and 15).
#[cfg(target_os = "linux")]
fn cpu_ticks(pid: u32) -> Option<u64> {
    let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let fields: Vec<&str> = stat[stat.rfind(')')? + 2..].split_whitespace().collect();
    Some(fields.get(11)?.parse::<u64>().ok()? + fields.get(12)?.parse::<u64>().ok()?)
}

#[cfg(target_os = "linux")]
fn wait_exit(child: &mut std::process::Child, secs: u64) -> Option<std::process::ExitStatus> {
    let deadline = Instant::now() + Duration::from_secs(secs);
    loop {
        if let Some(s) = child.try_wait().unwrap() {
            return Some(s);
        }
        if Instant::now() > deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// The device state after the 1.2.1 upgrade: the vision loop alive at 100 % CPU, no child,
/// SIGTERM without effect. `stop` must end it (SIGKILL to the loop, which holds no camera)
/// instead of reporting "still stopping" forever, and must leave a loop that still waits for
/// rebot-vision alone (no SIGKILL while a child exists).
#[cfg(target_os = "linux")]
#[test]
fn init_stop_vision_kills_a_hung_loop_without_a_child_only() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    let functions = init_section(source, "has_child() {", "\nstop() {");
    for hung in [true, false] {
        let temp = tempfile::tempdir().unwrap();
        let runner = temp.path().join("rebot-vision-run");
        let body = if hung {
            "trap '' TERM\nwhile :; do :; done\n"
        } else {
            // still waiting for a rebot-vision that takes long to exit
            "trap '' TERM\nsleep 30 &\nwhile :; do wait; done\n"
        };
        std::fs::write(&runner, body).unwrap();
        let mut loop_proc = Command::new("sh").arg(&runner).spawn().unwrap();
        let pid = loop_proc.id();
        std::thread::sleep(Duration::from_millis(300));
        let pidfile = temp.path().join("vision.pid");
        std::fs::write(&pidfile, format!("{pid}\n")).unwrap();
        let script = format!(
            "NAME=test\nVISION_PIDFILE='{}'\nVISION_RUNNER='{}'\nVISION_STOP_TICKS=12\nlogger() {{ :; }}\n{functions}\nstop_vision\n",
            pidfile.display(),
            runner.display()
        );
        let out = Command::new("sh").args(["-c", &script]).output().unwrap();
        let stdout = String::from_utf8_lossy(&out.stdout).to_string();
        assert!(out.status.success());
        if hung {
            let status = wait_exit(&mut loop_proc, 3);
            assert!(status.is_some(), "hung loop still running after stop: {stdout}");
            assert!(stdout.contains("sending SIGKILL"), "{stdout}");
            assert!(!pidfile.exists());
        } else {
            assert!(wait_exit(&mut loop_proc, 0).is_none(), "a loop with a child must not be killed: {stdout}");
            assert!(stdout.contains("still stopping"), "{stdout}");
            assert!(pidfile.exists());
            // clean up the loop and its sleep
            let _ = Command::new("pkill").args(["-KILL", "-P", &pid.to_string()]).status();
            let _ = loop_proc.kill();
            let _ = loop_proc.wait();
        }
    }
}

/// The runner is parsed whole before its loop starts: replacing the file in place while it
/// runs (as an upgrade can) changes nothing, SIGTERM still stops it.
#[cfg(target_os = "linux")]
#[test]
fn vision_runner_ignores_a_replaced_script_file() {
    let source = include_str!("../kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-vision-run");
    let temp = tempfile::tempdir().unwrap();
    let runner = temp.path().join("rebot-vision-run");
    std::fs::write(&runner, source).unwrap();
    let marks = temp.path().join("marks");
    let bin = temp.path().join("fake-vision");
    std::fs::write(&bin, format!("#!/bin/sh\necho start >> '{0}'\ntrap 'echo term >> \"{0}\"; exit 0' TERM\nwhile :; do sleep 0.05; done\n", marks.display())).unwrap();
    std::fs::set_permissions(&bin, std::os::unix::fs::PermissionsExt::from_mode(0o755)).unwrap();
    let mut child = device_shell()
        .arg(&runner)
        .env("REBOT_VISION_BIN", &bin)
        .env("REBOT_VISION_CONF", temp.path().join("vision.json"))
        .env("REBOT_VISION_LOG", temp.path().join("log"))
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !std::fs::read_to_string(&marks).unwrap_or_default().contains("start") {
        assert!(Instant::now() < deadline, "fake vision never started");
        std::thread::sleep(Duration::from_millis(20));
    }
    // in-place rewrite (same inode): text that would spin if the shell read it
    let garbage = "while :; do :; done\n".repeat(source.len() / 20 + 200);
    std::fs::write(&runner, garbage).unwrap();
    Command::new("kill").args(["-TERM", &child.id().to_string()]).status().unwrap();
    let status = wait_exit(&mut child, 5);
    let ticks = cpu_ticks(child.id());
    if status.is_none() {
        let _ = child.kill();
        panic!("runner did not exit after its file was replaced (cpu ticks {ticks:?})");
    }
    assert!(status.unwrap().success());
    assert_eq!(std::fs::read_to_string(&marks).unwrap(), "start\nterm\n");
}

/// While rebot-vision takes its time to exit after SIGTERM (camera release), the runner
/// waits without spinning, and exits once it is gone.
#[cfg(target_os = "linux")]
#[test]
fn vision_runner_waits_without_spinning_while_rebot_vision_stops() {
    let runner = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-vision-run");
    let temp = tempfile::tempdir().unwrap();
    let marks = temp.path().join("marks");
    let bin = temp.path().join("fake-vision");
    // exits 2 s after the first SIGTERM; further SIGTERMs are ignored
    std::fs::write(
        &bin,
        format!("#!/bin/sh\necho start >> '{0}'\nt=0\ntrap 't=1' TERM\nwhile [ $t = 0 ]; do sleep 0.05; done\ntrap '' TERM\nsleep 2\necho done >> '{0}'\n", marks.display()),
    )
    .unwrap();
    std::fs::set_permissions(&bin, std::os::unix::fs::PermissionsExt::from_mode(0o755)).unwrap();
    let mut child = device_shell()
        .arg(runner)
        .env("REBOT_VISION_BIN", &bin)
        .env("REBOT_VISION_CONF", temp.path().join("vision.json"))
        .env("REBOT_VISION_LOG", temp.path().join("log"))
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !std::fs::read_to_string(&marks).unwrap_or_default().contains("start") {
        assert!(Instant::now() < deadline, "fake vision never started");
        std::thread::sleep(Duration::from_millis(20));
    }
    let before = cpu_ticks(child.id()).unwrap();
    for _ in 0..3 {
        Command::new("kill").args(["-TERM", &child.id().to_string()]).status().unwrap();
        std::thread::sleep(Duration::from_millis(300));
    }
    std::thread::sleep(Duration::from_millis(800));
    let during = cpu_ticks(child.id()).unwrap();
    let status = wait_exit(&mut child, 6);
    assert!(status.is_some_and(|s| s.success()), "runner did not exit after rebot-vision");
    assert_eq!(std::fs::read_to_string(&marks).unwrap(), "start\ndone\n");
    assert!(during - before < 20, "runner used {} cpu ticks in 1.7 s while waiting", during - before);
}

#[cfg(unix)]
#[test]
fn vision_runner_forwards_sigterm_and_exits() {
    let runner = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-vision-run");
    let temp = tempfile::tempdir().unwrap();
    let marks = temp.path().join("marks");
    let bin = temp.path().join("fake-vision");
    std::fs::write(&bin, format!("#!/bin/sh\necho start >> '{0}'\ntrap 'echo term >> \"{0}\"; exit 0' TERM\nwhile :; do sleep 0.05; done\n", marks.display())).unwrap();
    std::fs::set_permissions(&bin, std::os::unix::fs::PermissionsExt::from_mode(0o755)).unwrap();
    let mut child = Command::new("sh")
        .arg(runner)
        .env("REBOT_VISION_BIN", &bin)
        .env("REBOT_VISION_CONF", temp.path().join("vision.json"))
        .env("REBOT_VISION_LOG", temp.path().join("log"))
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !std::fs::read_to_string(&marks).unwrap_or_default().contains("start") {
        assert!(Instant::now() < deadline, "fake vision never started");
        std::thread::sleep(Duration::from_millis(20));
    }
    Command::new("kill").args(["-TERM", &child.id().to_string()]).status().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let status = loop {
        if let Some(s) = child.try_wait().unwrap() {
            break s;
        }
        assert!(Instant::now() < deadline, "runner did not exit after SIGTERM");
        std::thread::sleep(Duration::from_millis(20));
    };
    assert!(status.success());
    assert_eq!(std::fs::read_to_string(&marks).unwrap(), "start\nterm\n", "child got SIGTERM once and was not restarted");
}

/// A reCamera root in a temp dir: postinst and K92rebot-teleop with their absolute paths moved
/// under it, a supervisor main.sh stub, and a log of the stub's calls.
#[cfg(unix)]
struct PackageRoot {
    dir: tempfile::TempDir,
}

#[cfg(unix)]
impl PackageRoot {
    const INIT: &'static str = include_str!("../kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop");
    const POSTINST: &'static str = include_str!("../kits/recamera-b601dm/packaging/debian/postinst");

    fn new(console: bool) -> Self {
        use std::os::unix::fs::PermissionsExt;
        let root = PackageRoot { dir: tempfile::tempdir().unwrap() };
        for d in ["etc/init.d", "userdata/local/apps", "usr/share/rebot-teleop", "usr/share/supervisor/scripts", "var/log"] {
            std::fs::create_dir_all(root.p(d)).unwrap();
        }
        std::fs::write(root.p("usr/share/rebot-teleop/rebot-teleop.json"), "{}").unwrap();
        std::fs::write(root.p("usr/share/rebot-teleop/vision.json.default"), "{}").unwrap();
        let init = root.p("etc/init.d/K92rebot-teleop");
        std::fs::write(&init, root.rebase(Self::INIT)).unwrap();
        std::fs::set_permissions(&init, std::fs::Permissions::from_mode(0o755)).unwrap();
        if console {
            // Same dispatch ("$1 "$@"") and argument use ($2) as the console's main.sh.
            let stub = format!(
                "#!/bin/bash\nSTATE='{}'\nfunction app_write_state() {{\n  echo \"app_write_state $#\" >> '{}'\n  local content=\"$2\"\n  [ -z \"$content\" ] && {{ echo Failed; return 1; }}\n  printf '%s' \"$content\" > \"$STATE.tmp\" && mv -f \"$STATE.tmp\" \"$STATE\" && echo OK\n}}\n$1 \"$@\"\n",
                root.p("userdata/local/apps/state.json").display(),
                root.p("calls").display()
            );
            let main = root.p("usr/share/supervisor/scripts/main.sh");
            std::fs::write(&main, stub).unwrap();
            std::fs::set_permissions(&main, std::fs::Permissions::from_mode(0o755)).unwrap();
        }
        root
    }

    fn p(&self, rel: &str) -> std::path::PathBuf {
        self.dir.path().join(rel)
    }

    /// Absolute device paths in a script -> paths under this root.
    fn rebase(&self, script: &str) -> String {
        let r = self.dir.path().display().to_string();
        script
            .replace("/etc/init.d/", &format!("{r}/etc/init.d/"))
            .replace("/userdata/", &format!("{r}/userdata/"))
            .replace("/usr/share/", &format!("{r}/usr/share/"))
            .replace("/usr/bin/", &format!("{r}/usr/bin/"))
            .replace("/usr/local/bin/", &format!("{r}/usr/local/bin/"))
            .replace("/proc/sys/", &format!("{r}/proc/sys/"))
            .replace("/var/log/", &format!("{r}/var/log/"))
    }

    /// Replace K92rebot-teleop with a stub that logs its argument to `init-calls` and runs the
    /// real script only for `register` (and `stop-vision`, a no-op here), so a postinst `start`
    /// never touches the host's network, CAN or processes.
    fn stub_init(&self) {
        use std::os::unix::fs::PermissionsExt;
        let real = self.p("etc/init.d/real-K92rebot-teleop");
        std::fs::write(&real, self.rebase(Self::INIT)).unwrap();
        let stub = format!(
            "#!/bin/sh\necho \"$1\" >> '{}'\ncase \"$1\" in register|stop-vision) exec sh '{}' \"$1\" ;; esac\nexit 0\n",
            self.p("init-calls").display(),
            real.display()
        );
        let init = self.p("etc/init.d/K92rebot-teleop");
        std::fs::write(&init, stub).unwrap();
        std::fs::set_permissions(&init, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    /// Arguments the stub init script was called with; waits up to 3 s for `want` (the
    /// postinst start runs in the background).
    fn init_calls(&self, want: Option<&str>) -> String {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            let calls = std::fs::read_to_string(self.p("init-calls")).unwrap_or_default();
            if want.is_none_or(|w| calls.lines().any(|l| l == w)) || Instant::now() > deadline {
                return calls;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    fn init_path(&self) -> String {
        self.p("etc/init.d/K92rebot-teleop").display().to_string()
    }

    fn run(&self, script: &str) -> std::process::Output {
        let path = self.p("script.sh");
        std::fs::write(&path, self.rebase(script)).unwrap();
        // logger / chown stubs keep the run inside the temp root
        let wrapped = format!(
            "logger() {{ :; }}\nchown() {{ :; }}\nsync() {{ :; }}\n. '{}'\n",
            path.display()
        );
        Command::new("sh").args(["-c", &wrapped]).output().unwrap()
    }

    fn postinst(&self) -> std::process::Output {
        self.run(Self::POSTINST)
    }

    fn register(&self) -> std::process::Output {
        Command::new("sh").arg(self.init_path()).arg("register").output().unwrap()
    }

    fn state(&self) -> serde_json::Value {
        serde_json::from_str(&std::fs::read_to_string(self.p("userdata/local/apps/state.json")).unwrap()).unwrap()
    }

    fn calls(&self) -> String {
        std::fs::read_to_string(self.p("calls")).unwrap_or_default()
    }
}

#[cfg(unix)]
#[test]
fn postinst_with_console_removes_every_boot_link_and_registers_the_app() {
    let root = PackageRoot::new(true);
    root.stub_init();
    let init = root.init_path();
    for link in ["S92rebot-teleop", "S94rebot-teleop"] {
        std::os::unix::fs::symlink(&init, root.p(&format!("etc/init.d/{link}"))).unwrap();
    }
    // another app's boot link and init script stay as they are
    std::fs::write(root.p("etc/init.d/K92yolo-detector"), "#!/bin/sh\n").unwrap();
    std::os::unix::fs::symlink(root.p("etc/init.d/K92yolo-detector"), root.p("etc/init.d/S92yolo-detector")).unwrap();
    // the console's state.json format (nlohmann dump(2)) with a model selection
    std::fs::write(
        root.p("userdata/local/apps/state.json"),
        "{\n  \"active_app\": \"yolo-detector\",\n  \"active_script\": \"/etc/init.d/K92yolo-detector\",\n  \"models\": {\n    \"yolo-detector\": \"/userdata/local/models/yolo26n_cv181x_int8.cvimodel\"\n  },\n  \"updated_at\": 1762600000\n}",
    )
    .unwrap();

    let out = root.postinst();
    assert!(out.status.success(), "{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(!root.p("etc/init.d/S92rebot-teleop").exists() && root.p("etc/init.d/S92rebot-teleop").symlink_metadata().is_err());
    assert!(root.p("etc/init.d/S94rebot-teleop").symlink_metadata().is_err(), "S94 link from an old install must go");
    assert!(root.p("etc/init.d/K92rebot-teleop").is_file());
    assert!(root.p("etc/init.d/S92yolo-detector").symlink_metadata().unwrap().file_type().is_symlink());
    assert!(root.p("etc/init.d/K92yolo-detector").is_file());

    let s = root.state();
    assert_eq!(s["active_app"], "rebot-teleop");
    assert_eq!(s["active_script"], init.as_str());
    assert_eq!(s["models"]["yolo-detector"], "/userdata/local/models/yolo26n_cv181x_int8.cvimodel");
    assert!(s["updated_at"].as_u64().unwrap() > 1762600000);
    assert_eq!(s.as_object().unwrap().len(), 4);
    // written through the console's app_write_state with exactly one JSON argument
    assert_eq!(root.calls(), "app_write_state 2\n");
    assert!(String::from_utf8_lossy(&out.stdout).contains("recorded rebot-teleop as the console's active app"));
    // another app was active: the install does not start rebot-teleop (it starts at the next
    // boot, or when the console switches to it)
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(root.init_calls(None), "stop-vision\nregister\n");
}

#[cfg(unix)]
#[test]
fn postinst_upgrade_starts_the_app_when_it_is_the_console_active_app() {
    let root = PackageRoot::new(true);
    root.stub_init();
    std::fs::write(
        root.p("userdata/local/apps/state.json"),
        "{\n  \"active_app\": \"rebot-teleop\",\n  \"active_script\": \"/etc/init.d/K92rebot-teleop\",\n  \"models\": {},\n  \"updated_at\": 1762600000\n}",
    )
    .unwrap();
    let started = Instant::now();
    let out = root.postinst();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    assert!(started.elapsed() < Duration::from_secs(2), "postinst must not wait for start");
    assert!(String::from_utf8_lossy(&out.stdout).contains("Starting rebot-teleop in the background"));
    assert_eq!(root.init_calls(Some("start")), "stop-vision\nregister\nstart\n");
    assert_eq!(root.state()["active_app"], "rebot-teleop");
}

#[cfg(unix)]
#[test]
fn postinst_does_not_start_the_app_without_an_active_app_record() {
    for content in [None, Some("{\"active_app\": null, \"active_script\": null, \"models\": {}, \"updated_at\": 0}"), Some("garbage")] {
        let root = PackageRoot::new(true);
        root.stub_init();
        if let Some(c) = content {
            std::fs::write(root.p("userdata/local/apps/state.json"), c).unwrap();
        }
        let out = root.postinst();
        assert!(out.status.success(), "{content:?}: {}", String::from_utf8_lossy(&out.stderr));
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(root.init_calls(None), "stop-vision\nregister\n", "{content:?}");
    }
}

#[cfg(unix)]
#[test]
fn postinst_without_console_creates_the_s92_link_and_leaves_state_alone() {
    let root = PackageRoot::new(false);
    root.stub_init();
    std::fs::write(root.p("userdata/local/apps/state.json"), "{\"active_app\": \"yolo-detector\"}").unwrap();
    let out = root.postinst();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    // no console: the S92 link is the app's boot start, so the install starts it as well
    assert_eq!(root.init_calls(Some("start")), "stop-vision\nstart\n");
    std::fs::write(root.p("etc/init.d/K92rebot-teleop"), root.rebase(PackageRoot::INIT)).unwrap();
    let link = root.p("etc/init.d/S92rebot-teleop");
    assert_eq!(std::fs::read_link(&link).unwrap().display().to_string(), root.init_path());
    assert_eq!(std::fs::read_to_string(root.p("userdata/local/apps/state.json")).unwrap(), "{\"active_app\": \"yolo-detector\"}");
    // register is a no-op without the console
    assert!(root.register().status.success());
    assert_eq!(std::fs::read_to_string(root.p("userdata/local/apps/state.json")).unwrap(), "{\"active_app\": \"yolo-detector\"}");
}

#[cfg(unix)]
#[test]
fn register_handles_every_state_shape_and_is_idempotent() {
    for (name, content) in [
        ("missing", None),
        ("one line", Some("{\"active_app\": null, \"active_script\": null, \"models\": {\"a\": \"b\"}, \"updated_at\": 0}")),
        ("empty object", Some("{}")),
        ("only models", Some("{\n  \"models\": {}\n}")),
        ("corrupt", Some("garbage")),
    ] {
        let root = PackageRoot::new(true);
        if let Some(c) = content {
            std::fs::write(root.p("userdata/local/apps/state.json"), c).unwrap();
        }
        let out = root.register();
        assert!(out.status.success(), "{name}: {}", String::from_utf8_lossy(&out.stdout));
        let s = root.state();
        assert_eq!(s["active_app"], "rebot-teleop", "{name}");
        assert_eq!(s["active_script"], root.init_path().as_str(), "{name}");
        if name == "one line" {
            assert_eq!(s["models"]["a"], "b");
        }
        // already recorded: no second write
        assert_eq!(root.calls().lines().count(), 1, "{name}");
        assert!(root.register().status.success());
        assert_eq!(root.calls().lines().count(), 1, "{name}: second register must not rewrite");
    }
}

#[cfg(unix)]
#[test]
fn register_falls_back_to_an_atomic_write_when_the_console_write_fails() {
    use std::os::unix::fs::PermissionsExt;
    let root = PackageRoot::new(true);
    let main = root.p("usr/share/supervisor/scripts/main.sh");
    std::fs::write(&main, "#!/bin/bash\necho Failed\n").unwrap();
    std::fs::set_permissions(&main, std::fs::Permissions::from_mode(0o755)).unwrap();
    std::fs::write(root.p("userdata/local/apps/state.json"), "{\"active_app\": \"face-recognition\", \"active_script\": \"/etc/init.d/K92face-recognition\", \"models\": {}, \"updated_at\": 5}").unwrap();
    let out = root.register();
    assert!(out.status.success());
    let s = root.state();
    assert_eq!(s["active_app"], "rebot-teleop");
    assert_eq!(s["active_script"], root.init_path().as_str());
    assert!(!root.p("userdata/local/apps/.state.json.tmp").exists());
}
