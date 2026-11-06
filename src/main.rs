use std::net::SocketAddr;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;
use tracing::{info, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::{create_router, ArmConfig, ArmSupervisor, StopPolicy, SupervisorInner};

#[derive(Debug, Default)]
struct CliArgs {
    follower: Option<String>,
    follower_port: Option<String>,
    port: Option<u16>,
    config: Option<String>,
}

fn parse_cli_args() -> Result<CliArgs, String> {
    let mut args = std::env::args().skip(1);
    let mut cli = CliArgs::default();
    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!("armd - Robotic arm daemon\n\nUSAGE: armd [OPTIONS]\n\n\
                --follower <MODEL>       rebot-rs, rebot-dm, so-arm101, fashionstar, star-arm102, mock (default)\n\
                --follower-port <PORT>   CAN interface or serial device\n\
                --config <JSON>          ArmConfig profile (must match follower joint count)\n\
                --port <PORT>            HTTP / WebSocket port (default 8765)\n\
                --help                   Print help\n\n\
                Leader-following is not implemented. Hardware failures never fall back to mock.");
            std::process::exit(0);
        }
        if arg == "--leader" || arg == "--leader-port" {
            return Err(
                "Leader-following is not implemented; refusing to start a simulated leader".into(),
            );
        }
        if !matches!(
            arg.as_str(),
            "--follower" | "--follower-port" | "--port" | "--config"
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "armd=info,tower_http=info".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    let cli = parse_cli_args()?;
    let follower = cli.follower.as_deref().unwrap_or("mock");
    let expected_dof = match follower {
        "rebot-dm" => Some(7),
        "rebot-rs" | "so-arm101" | "fashionstar" | "star-arm102" => Some(6),
        "mock" => None,
        other => return Err(format!("Unknown follower model: {other}").into()),
    };
    if follower == "mock" && cli.follower_port.is_some() {
        return Err("--follower-port requires a hardware follower".into());
    }
    let mut config = if let Some(path) = cli.config {
        serde_json::from_reader::<_, ArmConfig>(std::fs::File::open(path)?)?
    } else if follower == "rebot-dm" {
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
        )
        .into());
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
    let supervisor = Arc::new(ArmSupervisor::new(config));
    match follower {
        "rebot-rs" | "rebot-dm" => {
            #[cfg(target_os = "linux")]
            {
                let port = cli.follower_port.as_deref().unwrap_or("can0");
                let can = armd::transport::SocketCanTransport::open(port)?;
                if follower == "rebot-dm" {
                    supervisor
                        .set_driver(Box::new(armd::driver::DamiaoArmDriver::new(
                            can,
                            (1..=7).collect(),
                        )))
                        .await;
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
        "so-arm101" | "fashionstar" | "star-arm102" => {
            let port = cli.follower_port.as_deref().unwrap_or("/dev/ttyUSB0");
            let baud = if follower == "so-arm101" {
                1_000_000
            } else {
                115_200
            };
            let transport = armd::transport::SerialTransport::open_default(port, baud)?;
            if follower == "so-arm101" {
                supervisor
                    .set_driver(Box::new(armd::driver::FeetechArmDriver::new(
                        transport,
                        (1..=6).collect(),
                    )))
                    .await;
            } else {
                supervisor
                    .set_driver(Box::new(armd::driver::FashionStarArmDriver::new(
                        transport,
                        (1..=6).collect(),
                    )))
                    .await;
            }
            info!("Mounted {} on {}", follower, port);
        }
        "mock" => info!("Follower configured as mock simulation"),
        _ => unreachable!(),
    }

    let control_supervisor = supervisor.clone();
    let control_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut last_tick = std::time::Instant::now();
        loop {
            interval.tick().await;
            let dt = last_tick.elapsed().as_secs_f64();
            last_tick = std::time::Instant::now();
            // Preserve real elapsed time so the supervisor can detect stalled control ticks.
            control_supervisor.tick(dt).await;
        }
    });

    info!(
        "armd API server listening on http://{}",
        listener.local_addr()?
    );
    let router = create_router(supervisor.clone());
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
    control_handle.abort();
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
}
