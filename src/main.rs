use std::net::SocketAddr;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;
use tracing::{error, info, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::{create_router_with_auth, ArmConfig, ArmSupervisor, OperatorAuth, StopPolicy, SupervisorInner};

#[derive(Debug, Default)]
struct CliArgs {
    follower: Option<String>,
    follower_port: Option<String>,
    port: Option<u16>,
    config: Option<String>,
    log_dir: Option<String>,
    blackbox_dir: Option<String>,
    episodes_dir: Option<String>,
    operator_pin: Option<String>,
    operator_pin_file: Option<String>,
    no_operator_auth: bool,
}

fn parse_cli_args() -> Result<CliArgs, String> {
    let mut args = std::env::args().skip(1);
    let mut cli = CliArgs::default();
    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!("armd - Robotic arm daemon\n\nUSAGE: armd [OPTIONS]\n\n\
                --follower <MODEL>           rebot-rs, rebot-dm, so-arm101, fashionstar, star-arm102, sim, mock (default)\n\
                --follower-port <PORT>       CAN interface, serial device, or udp://HOST:PORT for sim (default udp://127.0.0.1:47600)\n\
                --config <JSON>              ArmConfig profile (must match follower joint count)\n\
                --port <PORT>                HTTP / WebSocket port (default 8765)\n\
                --log-dir <DIR>              rotating log armd.log (default /userdata/rebot-teleop/logs when /userdata exists; off disables)\n\
                --blackbox-dir <DIR>         black box and boot record (default /userdata/rebot-teleop/blackbox when /userdata exists; off disables)\n\
                --episodes-dir <DIR>         recorded episodes (default /userdata/rebot-teleop/episodes when /userdata exists, else ./data/episodes; off disables recording and replay)\n\
                --operator-pin <PIN>         operator unlock PIN (or ARMD_OPERATOR_PIN; enforces operator auth)\n\
                --operator-pin-file <PATH>   path to write generated PIN with 0600 permissions (or ARMD_OPERATOR_PIN_FILE; default /userdata/rebot-teleop/operator.pin)\n\
                --no-operator-auth           disable operator auth (unsafe on real hardware; mock/sim default is disabled)\n\
                --help                       Print help\n\n\
                Leader-following is not implemented. Hardware failures never fall back to mock.");
            std::process::exit(0);
        }
        if arg == "--leader" || arg == "--leader-port" {
            return Err(
                "Leader-following is not implemented; refusing to start a simulated leader".into(),
            );
        }
        if arg == "--no-operator-auth" {
            cli.no_operator_auth = true;
            continue;
        }
        if !matches!(
            arg.as_str(),
            "--follower"
                | "--follower-port"
                | "--port"
                | "--config"
                | "--log-dir"
                | "--blackbox-dir"
                | "--episodes-dir"
                | "--operator-pin"
                | "--operator-pin-file"
        ) {
            return Err(format!("Unknown argument: {arg}"));
        }
        let value = args
            .next()
            .filter(|value| !value.starts_with("--"))
            .ok_or_else(|| format!("Missing value for {arg}"))?;
        match arg.as_str() {
            "--follower" => cli.follower = Some(value),
            "--follower-port" => cli.follower_port = Some(value),
            "--config" => cli.config = Some(value),
            "--log-dir" => cli.log_dir = Some(value),
            "--blackbox-dir" => cli.blackbox_dir = Some(value),
            "--episodes-dir" => cli.episodes_dir = Some(value),
            "--operator-pin" => {
                if value.is_empty() {
                    return Err("--operator-pin must not be empty".into());
                }
                cli.operator_pin = Some(value);
            }
            "--operator-pin-file" => cli.operator_pin_file = Some(value),
            "--port" => {
                cli.port = Some(
                    value
                        .parse::<u16>()
                        .map_err(|_| format!("Invalid port: {value}"))?,
                )
            }
            _ => unreachable!(),
        }
    }
    Ok(cli)
}

fn resolve_operator_auth(follower: &str, cli: &CliArgs) -> Result<OperatorAuth, String> {
    let is_mock_or_sim = matches!(follower, "mock" | "sim");
    if let Some(pin) = cli.operator_pin.as_deref() {
        if pin.is_empty() {
            return Err("--operator-pin must not be empty".into());
        }
    }
    if cli.no_operator_auth {
        if !is_mock_or_sim {
            warn!("Operator authentication disabled by --no-operator-auth");
        }
        return Ok(OperatorAuth::disabled());
    }
    let env_pin = std::env::var("ARMD_OPERATOR_PIN").ok().filter(|s| !s.is_empty());
    let configured_pin = cli.operator_pin.as_deref().or(env_pin.as_deref());
    if let Some(pin) = configured_pin {
        info!("Operator authentication enforced (configured PIN)");
        return Ok(OperatorAuth::enforced(pin, armd::auth::DEFAULT_TTL_S));
    }
    if is_mock_or_sim {
        return Ok(OperatorAuth::disabled());
    }
    let pin = armd::auth::generate_pin_6_digits()
        .map_err(|e| format!("Failed to generate operator PIN: {e}"))?;
    let pin_path = armd::auth::resolve_pin_file_path(cli.operator_pin_file.as_deref());
    armd::auth::write_pin_file_atomic(&pin_path, &pin)
        .map_err(|e| format!("Failed to write operator PIN file {}: {e}", pin_path.display()))?;
    info!("Operator authentication enforced; PIN written to {}", pin_path.display());
    Ok(OperatorAuth::enforced(pin, armd::auth::DEFAULT_TTL_S))
}

/// Called while holding the supervisor write lock so stop authority cannot race motion.
fn request_shutdown(inner: &mut SupervisorInner) -> bool {
    if inner.driver.is_none() || inner.shutdown_ready() {
        inner.e_stop_latched.store(true, Ordering::SeqCst);
        return true;
    }
    if !inner.safety.state().is_stopped() {
        inner.execute_stop(StopPolicy::EStopLatch);
    }
    false
}

fn set_realtime_priority() {
    #[cfg(target_os = "linux")]
    {
        // musl adds sporadic-server fields; zero them and set only the priority.
        // SAFETY: sched_param is plain old data, all-zero is a valid value.
        let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
        param.sched_priority = 50;
        // SAFETY: plain syscall on the calling thread with a valid param struct.
        // musl stubs sched_setscheduler() with ENOSYS; the pthread call is implemented.
        let rc = unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
        if rc == 0 {
            info!("Control loop running at SCHED_FIFO priority 50");
        } else {
            warn!("Control loop real-time priority unavailable ({}); using normal scheduling", std::io::Error::from_raw_os_error(rc));
        }
    }
}

/// Boot record, then the black box; any failure leaves armd running without them.
fn start_blackbox(supervisor: &ArmSupervisor, cli_dir: Option<&str>, system: &armd::diagnostics::boot::SystemInfo, started: std::time::Instant) {
    use armd::diagnostics::{self, DiagnosticsState};
    let Some(dir) = diagnostics::resolve_dir(cli_dir, "ARMD_BLACKBOX_DIR", diagnostics::DEFAULT_BLACKBOX_DIR) else {
        return;
    };
    let result = diagnostics::prepare_dir(&dir)
        .and_then(|()| diagnostics::boot::record_boot(&dir, system))
        .and_then(|rec| diagnostics::blackbox::open(&dir, rec.boot, started).map(|bb| (rec, bb)));
    match result {
        Ok((rec, bb)) => {
            info!("Previous start: {}", diagnostics::boot::summary(&rec));
            info!("Black box recording to {} (boot {})", dir.display(), rec.boot);
            diagnostics::blackbox::spawn_recorder(supervisor, bb);
            DiagnosticsState::set_global(DiagnosticsState { blackbox_dir: Some(dir), boot: rec.boot });
        }
        Err(e) => warn!("Black box directory {} unavailable ({e}); black box off", dir.display()),
    }
}

fn resolve_serial_port(requested: Option<&str>) -> String {
    if let Some(p) = requested {
        return p.to_string();
    }
    let probes = ["/dev/ttySC0", "/dev/ttyACM0", "/dev/ttyUSB0"];
    for p in probes {
        if std::path::Path::new(p).exists() {
            return p.to_string();
        }
    }
    "/dev/ttySC0".to_string()
}

fn resolve_follower_config(follower: &str, config_path: Option<&str>) -> Result<ArmConfig, String> {
    let expected_dof = match follower {
        "rebot-dm" => Some(7),
        "rebot-rs" | "so-arm101" | "so-arm100" | "fashionstar" | "star-arm102" => Some(6),
        "sim" | "mock" => None,
        other => return Err(format!("Unknown follower model: {other}")),
    };
    let mut config = if let Some(path) = config_path {
        let config = ArmConfig::from_profile_file(path)?;
        if config.kinematics.is_none() || config.workspace.is_none() {
            tracing::warn!("Profile {path} has no kinematics/workspace: the Cartesian workspace check and Cartesian actions are off");
        }
        config
    } else if follower == "rebot-rs" {
        return Err(
            "Follower 'rebot-rs' requires --config <path> (e.g. profiles/rebot_b601_rs.json); refusing to fall back to unconfigured defaults"
                .into(),
        );
    } else if follower == "so-arm101" || follower == "so-arm100" {
        let name = if follower == "so-arm100" { "so_arm100.json" } else { "so_arm101.json" };
        let candidates = [
            format!("/userdata/rebot-teleop/profile/{name}"),
            format!("profiles/{name}"),
            format!("/usr/share/rebot-teleop/profile/{name}"),
        ];
        if std::path::Path::new(&candidates[0]).exists() {
            if let Some(bundled) = candidates[1..].iter().find(|p| std::path::Path::new(p).exists()) {
                tracing::warn!("User profile {} overrides bundled profile {}", candidates[0], bundled);
            }
        }
        let path = candidates.iter().find(|p| std::path::Path::new(p).exists())
            .unwrap_or(&candidates[1]);
        tracing::info!("Loaded profile for {follower} from {path}");
        ArmConfig::from_profile_file(path)?
    } else if follower == "rebot-dm" || follower == "sim" {
        ArmConfig::rebot_dm()
    } else {
        ArmConfig::default()
    };
    config.validate()?;
    if expected_dof.is_some_and(|dof| config.dof != dof) {
        return Err(format!(
            "Profile DOF {} does not match {follower} hardware DOF {}",
            config.dof,
            expected_dof.unwrap()
        ));
    }
    Ok(config)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let started = std::time::Instant::now();
    let system = armd::diagnostics::boot::SystemInfo::read();
    let cli = parse_cli_args()?;

    // stdout as before (the init script redirects it); a copy in a rotating file on
    // /userdata that survives a reboot.
    let log_dir = armd::diagnostics::resolve_dir(cli.log_dir.as_deref(), "ARMD_LOG_DIR", armd::diagnostics::DEFAULT_LOG_DIR);
    let mut log_warning = None;
    let log_file = log_dir.as_deref().and_then(|dir| match armd::diagnostics::open_log(dir) {
        Ok(file) => Some(file),
        Err(e) => {
            log_warning = Some(format!("Log directory {} unavailable ({e}); logging to stdout only", dir.display()));
            None
        }
    });
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "armd=info,tower_http=info".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .with(log_file.map(|file| tracing_subscriber::fmt::layer().with_ansi(false).with_writer(std::sync::Mutex::new(file))))
        .init();
    if let Some(warning) = log_warning {
        warn!("{warning}");
    }
    let follower = cli.follower.as_deref().unwrap_or("mock");
    let config = resolve_follower_config(follower, cli.config.as_deref())?;
    if follower == "mock" && cli.follower_port.is_some() {
        return Err("--follower-port requires a hardware follower".into());
    }
    let period = Duration::from_secs_f64(1.0 / f64::from(config.control_frequency_hz));
    let port = match cli.port {
        Some(port) => port,
        None => match std::env::var("PORT") {
            Ok(value) => value
                .parse::<u16>()
                .map_err(|_| format!("Invalid PORT: {value}"))?,
            Err(_) => 8765,
        },
    };
    // Bind before mounting hardware; an unavailable API must not leave a hardware loop running.
    let listener = tokio::net::TcpListener::bind(SocketAddr::from(([0, 0, 0, 0], port))).await?;
    #[cfg_attr(not(target_os = "linux"), allow(unused_variables))]
    let damiao_limits: Option<Vec<_>> = config.damiao_ranges.as_ref().map(|r| r.iter().map(|r| r.limits()).collect());
    let config_dof = config.dof;
    let feetech_config = config.feetech.clone();
    let gripper_index = config.gripper_index();
    let supervisor = Arc::new(ArmSupervisor::new(config));
    supervisor.set_follower(follower).await;
    start_blackbox(&supervisor, cli.blackbox_dir.as_deref(), &system, started);
    let episodes = armd::recording::EpisodeStore::resolve(cli.episodes_dir.as_deref());
    match episodes.dir() {
        Some(dir) => info!("Episodes directory {}", dir.display()),
        None => info!("Episode storage off: recording and replay are refused"),
    }
    supervisor.set_episodes_dir(episodes.dir().map(std::path::Path::to_path_buf)).await;
    let grasp_setup_path = armd::grasp::setup_path_from_env();
    armd::grasp::load_into(&supervisor, &grasp_setup_path).await;
    // after the grasp setup: the calibration wins for camera_mount and camera_tilt_deg
    if let Err(e) = armd::calibration::load_into(&supervisor, &armd::calibration::path_from_env(), Some(&grasp_setup_path)).await {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, e).into());
    }
    if let Err(e) = armd::keepout::load_into(&supervisor, &armd::keepout::path_from_env()).await {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, e).into());
    }
    armd::poses::load_into(&supervisor, &armd::poses::path_from_env()).await;
    armd::collision::load_into(&supervisor, &armd::collision::path_from_env()).await;
    match follower {
        "rebot-rs" | "rebot-dm" => {
            #[cfg(target_os = "linux")]
            {
                let port = cli.follower_port.as_deref().unwrap_or("can0");
                let can = armd::transport::SocketCanTransport::open(port)?;
                if follower == "rebot-dm" {
                    let mut driver = armd::driver::DamiaoArmDriver::new(can, (1..=7).collect());
                    if let Some(limits) = damiao_limits {
                        driver = driver.with_joint_limits(limits)?;
                    }
                    // Read-only register check before anything can enable the motors.
                    driver.check_mapping_ranges();
                    supervisor.set_driver(Box::new(driver)).await;
                } else {
                    supervisor
                        .set_driver(Box::new(armd::driver::RobStrideArmDriver::new(
                            can,
                            (1..=6).collect(),
                        )))
                        .await;
                }
                info!("Mounted {} on {}", follower, port);
            }
            #[cfg(not(target_os = "linux"))]
            return Err(format!("{follower} requires Linux SocketCAN").into());
        }
        "so-arm101" | "so-arm100" => {
            let Some(cfg) = feetech_config else {
                return Err(format!("{follower} requires feetech configuration in profile").into());
            };
            let port = resolve_serial_port(cli.follower_port.as_deref());
            let baud = 1_000_000;
            let transport = armd::transport::SerialTransport::open_default(&port, baud)?;
            let mut driver = armd::driver::FeetechArmDriver::new(transport, (1..=6).collect())
                .with_port_name(&port)
                .with_config(cfg)
                .with_gripper_index(gripper_index);
            driver.configure_servos().map_err(|e| format!("Feetech servo configuration failed on {port}: {e}"))?;
            supervisor.set_driver(Box::new(driver)).await;
            info!("Mounted {} on {}", follower, port);
        }
        "fashionstar" | "star-arm102" => {
            let port = cli.follower_port.as_deref().unwrap_or("/dev/ttyUSB0");
            let transport = armd::transport::SerialTransport::open_default(port, 115_200)?;
            supervisor
                .set_driver(Box::new(armd::driver::FashionStarArmDriver::new(
                    transport,
                    (1..=6).collect(),
                )))
                .await;
            info!("Mounted {} on {}", follower, port);
        }
        "sim" => {
            let addr = cli.follower_port.as_deref().unwrap_or(armd::sim_driver::DEFAULT_SIM_ADDR);
            let is_feetech = feetech_config.is_some();
            let driver = armd::sim_driver::SimArmDriver::connect(addr, config_dof)?
                .with_load_only_feedback(is_feetech);
            supervisor.set_driver(Box::new(driver)).await;
            if is_feetech {
                info!("Mounted sim follower (physics simulator at {addr}, SO-ARM Feetech load-only feedback semantics)");
            } else {
                info!("Mounted sim follower (physics simulator at {addr}, B601-DM semantics)");
            }
        }
        "mock" => info!("Follower configured as mock simulation"),
        _ => unreachable!(),
    }
    armd::gripper_calib::load_into(&supervisor, &armd::gripper_calib::path_from_env()).await;

    // The control loop runs on its own OS thread at real-time priority. On the
    // single-core SG2002 a tokio task shares one worker with HTTP/WS/video proxies
    // and with yolo-detector, so a tick could slip past the stall deadline.
    let control_supervisor = supervisor.clone();
    let control_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let control_stop_flag = control_stop.clone();
    let runtime = tokio::runtime::Handle::current();
    std::thread::Builder::new().name("armd-control".into()).spawn(move || {
        set_realtime_priority();
        let mut next = std::time::Instant::now() + period;
        let mut last_tick = std::time::Instant::now();
        let mut last_late_log: Option<std::time::Instant> = None;
        while !control_stop_flag.load(Ordering::SeqCst) {
            let now = std::time::Instant::now();
            if next > now { std::thread::sleep(next - now); }
            next += period;
            let now = std::time::Instant::now();
            if next < now { next = now + period; } // skip missed ticks
            let dt = last_tick.elapsed().as_secs_f64();
            last_tick = now;
            if dt > 0.080 && last_late_log.is_none_or(|t| t.elapsed() > Duration::from_secs(1)) {
                warn!("Control tick late: {:.0} ms since previous tick", dt * 1000.0);
                last_late_log = Some(now);
            }
            // Preserve real elapsed time so the supervisor can detect stalled control ticks.
            let tick_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                runtime.block_on(control_supervisor.tick(dt));
            }));
            if let Err(e) = tick_res {
                control_supervisor.e_stop_latched.store(true, Ordering::SeqCst);
                error!("Control thread panicked during tick: {:?}", e);
                std::process::exit(1);
            }
        }
    })?;

    let operator_auth = resolve_operator_auth(follower, &cli)?;
    supervisor.set_operator_auth_enforced(operator_auth.is_enforced()).await;
    info!(
        "armd API server listening on http://{}",
        listener.local_addr()?
    );
    let router = create_router_with_auth(supervisor.clone(), operator_auth);
    let mut server_handle = tokio::spawn(async move { axum::serve(listener, router).await });
    let mut server_finished = false;
    #[cfg(unix)]
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;

    loop {
        tokio::select! {
            result = &mut server_handle, if !server_finished => {
                server_finished = true;
                warn!("HTTP server exited: {:?}", result);
            }
            _ = async {
                #[cfg(unix)]
                tokio::select! {
                    _ = tokio::signal::ctrl_c() => {},
                    _ = sigterm.recv() => {},
                }
                #[cfg(not(unix))]
                { let _ = tokio::signal::ctrl_c().await; }
            } => {}
        }

        let mut inner = supervisor.inner().write().await;
        if request_shutdown(&mut inner) {
            break;
        }
        warn!("Shutdown refused: hardware disable is not confirmed. Stop is latched; control loop and API remain available. Explicit fault recovery and rearming are required before further motion.");
    }

    // A confirmed-safe exit must not wait indefinitely on connected WebSockets.
    server_handle.abort();
    control_stop.store(true, Ordering::SeqCst);
    info!("armd stopped after shutdown readiness confirmation");
    Ok(())
}

#[cfg(test)]
mod shutdown_tests {
    use super::*;
    use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
    use armd::model::JointState;
    use armd::safety::{FaultReason, SafetyState};

    struct ConfirmedDriver(EnableState);
    impl ArmDriver for ConfirmedDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState {
            self.0
        }
        fn is_enabled(&self) -> bool {
            self.0 == EnableState::Enabled
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 6]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); 6])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            self.0 = if enable {
                EnableState::Enabled
            } else {
                EnableState::Disabled
            };
            Ok(())
        }
    }

    fn physical(state: EnableState) -> SupervisorInner {
        let mut inner = SupervisorInner::new(ArmConfig::default())
            .with_driver(Box::new(ConfirmedDriver(state)));
        if state == EnableState::Enabled {
            inner.is_torqued = true;
            inner.safety.set_torque(true).unwrap();
        }
        inner
    }

    #[test]
    fn refused_shutdown_latches_stop_until_explicit_recovery_and_rearm() {
        let mut inner = physical(EnableState::Enabled);
        let old_epoch = inner.safety.epoch_id();
        inner.servo(vec![0.1; 6], old_epoch).unwrap();
        assert!(!request_shutdown(&mut inner));
        assert_eq!(inner.safety.state(), &SafetyState::EStop);
        let new_epoch = inner.safety.epoch_id();
        assert_ne!(new_epoch, old_epoch);
        assert!(inner.servo(vec![0.1; 6], old_epoch).is_err());
        assert!(inner.servo(vec![0.1; 6], new_epoch).is_err());
        assert!(inner.set_torque(true).is_err());
        inner.clear_fault().unwrap();
        assert_eq!(inner.safety.state(), &SafetyState::Idle);
        assert!(inner.servo(vec![0.1; 6], inner.safety.epoch_id()).is_err());
        inner.set_torque(true).unwrap();
        assert!(inner.servo(vec![0.1; 6], inner.safety.epoch_id()).is_ok());
    }

    #[test]
    fn refused_shutdown_preserves_existing_fault_and_epoch() {
        let mut inner = physical(EnableState::Enabled);
        inner.safety.trigger_fault(FaultReason::ThermalTrip {
            joint_idx: 0,
            temperature: 80.0,
            limit: 70.0,
        });
        let fault = inner.safety.state().clone();
        let epoch = inner.safety.epoch_id();
        assert!(!request_shutdown(&mut inner));
        assert_eq!(inner.safety.state(), &fault);
        assert_eq!(inner.safety.epoch_id(), epoch);
    }

    #[test]
    fn confirmed_disabled_hardware_can_exit_and_cannot_reenable_during_exit() {
        let mut inner = physical(EnableState::Disabled);
        assert!(request_shutdown(&mut inner));
        assert!(inner.set_torque(true).is_err());
        let mut unknown = physical(EnableState::Unknown);
        assert!(!request_shutdown(&mut unknown));
    }

    #[test]
    fn rebot_rs_requires_config_while_mock_defaults() {
        let err = resolve_follower_config("rebot-rs", None).unwrap_err();
        assert!(err.contains("rebot-rs"), "error should name follower: {err}");
        assert!(err.contains("--config"), "error should mention --config: {err}");
        assert!(
            err.contains("profiles/rebot_b601_rs.json"),
            "error should include example profile path: {err}"
        );

        let mock = resolve_follower_config("mock", None).unwrap();
        assert_eq!(mock.dof, 6);

        let profile = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/rebot_b601_rs.json");
        let rs = resolve_follower_config("rebot-rs", Some(profile)).unwrap();
        assert_eq!(rs.dof, 6);
        assert!(rs.kinematics.is_some() && rs.workspace.is_some());
    }
}
