use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use tracing::{info, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::{create_router, ArmConfig, ArmSupervisor, StopPolicy};

#[derive(Debug, Default)]
struct CliArgs {
    follower: Option<String>,
    follower_port: Option<String>,
    leader: Option<String>,
    leader_port: Option<String>,
    port: Option<u16>,
}

fn parse_cli_args() -> CliArgs {
    let args: Vec<String> = std::env::args().collect();
    let mut cli = CliArgs::default();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--help" | "-h" => {
                println!(
                    "armd - Minimal Robotic Arm Daemon & Teleop HAL\n\n\
                    USAGE:\n    \
                        armd [OPTIONS]\n\n\
                    OPTIONS:\n    \
                        --follower <MODEL>       Follower arm model: rebot-rs, rebot-dm, so-arm101, mock [default: mock]\n    \
                        --follower-port <PORT>   Follower interface: can0, /dev/ttyUSB0, /dev/ttyS1\n    \
                        --leader <MODEL>         Leader teleop arm model: star-arm102, so-arm100\n    \
                        --leader-port <PORT>     Leader serial interface: /dev/ttyACM0, /dev/ttyUSB1\n    \
                        --port <PORT>            HTTP / WebSocket service port [default: 8765]\n    \
                        -h, --help               Print help information\n"
                );
                std::process::exit(0);
            }
            "--follower" if i + 1 < args.len() => {
                cli.follower = Some(args[i + 1].clone());
                i += 2;
            }
            "--follower-port" if i + 1 < args.len() => {
                cli.follower_port = Some(args[i + 1].clone());
                i += 2;
            }
            "--leader" if i + 1 < args.len() => {
                cli.leader = Some(args[i + 1].clone());
                i += 2;
            }
            "--leader-port" if i + 1 < args.len() => {
                cli.leader_port = Some(args[i + 1].clone());
                i += 2;
            }
            "--port" if i + 1 < args.len() => {
                cli.port = args[i + 1].parse().ok();
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }
    cli
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

    info!("Starting armd (Minimal Robotic Arm Daemon) - M0");

    let cli = parse_cli_args();
    let config = ArmConfig::default();
    let hz = config.control_frequency_hz;
    let period_ms = 1000 / hz as u64;

    let supervisor = Arc::new(ArmSupervisor::new(config));

    // Mount hardware driver from CLI arguments if specified
    if let Some(ref follower) = cli.follower {
        match follower.as_str() {
            "rebot-rs" => {
                #[cfg(target_os = "linux")]
                {
                    let port = cli.follower_port.as_deref().unwrap_or("can0");
                    match armd::transport::SocketCanTransport::open(port) {
                        Ok(can) => {
                            let motor_ids = vec![1, 2, 3, 4, 5, 6];
                            let driver = armd::driver::RobStrideArmDriver::new(can, motor_ids);
                            supervisor.set_driver(Box::new(driver)).await;
                            info!("Mounted RobStride (reBot Arm B601-RS) follower driver on {}", port);
                        }
                        Err(e) => warn!("Failed to open SocketCAN port {}: {}", port, e),
                    }
                }
                #[cfg(not(target_os = "linux"))]
                {
                    warn!("rebot-rs driver requires Linux SocketCAN");
                }
            }
            "rebot-dm" => {
                #[cfg(target_os = "linux")]
                {
                    let port = cli.follower_port.as_deref().unwrap_or("can0");
                    match armd::transport::SocketCanTransport::open(port) {
                        Ok(can) => {
                            let motor_ids = vec![1, 2, 3, 4, 5, 6];
                            let driver = armd::driver::DamiaoArmDriver::new(can, motor_ids);
                            supervisor.set_driver(Box::new(driver)).await;
                            info!("Mounted Damiao (reBot Arm B601-DM) follower driver on {}", port);
                        }
                        Err(e) => warn!("Failed to open SocketCAN port {}: {}", port, e),
                    }
                }
                #[cfg(not(target_os = "linux"))]
                {
                    warn!("rebot-dm driver requires Linux SocketCAN");
                }
            }
            "so-arm101" => {
                let port = cli.follower_port.as_deref().unwrap_or("/dev/ttyUSB0");
                match armd::transport::SerialTransport::open_default(port, 1_000_000) {
                    Ok(transport) => {
                        let servo_ids = vec![1, 2, 3, 4, 5, 6];
                        let driver = armd::driver::FeetechArmDriver::new(transport, servo_ids);
                        supervisor.set_driver(Box::new(driver)).await;
                        info!("Mounted Feetech (SO-ARM 101) follower driver on {}", port);
                    }
                    Err(e) => warn!("Failed to open serial port {}: {}", port, e),
                }
            }
            "mock" => {
                info!("Follower configured as mock simulation");
            }
            other => {
                warn!("Unknown follower type: {}", other);
            }
        }
    }

    if let Some(ref leader) = cli.leader {
        let port = cli.leader_port.as_deref().unwrap_or("/dev/ttyUSB1");
        info!("Leader teleoperation device configured: {} on {}", leader, port);
    }

    // Spawn real-time control loop
    let sup_clone = supervisor.clone();
    let control_handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(period_ms));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut last_tick = std::time::Instant::now();
        loop {
            interval.tick().await;
            let dt = last_tick.elapsed().as_secs_f64().clamp(0.001, 0.05);
            last_tick = std::time::Instant::now();
            sup_clone.tick(dt).await;
        }
    });

    let router = create_router(supervisor.clone());

    let port: u16 = cli
        .port
        .or_else(|| std::env::var("PORT").ok().and_then(|p| p.parse().ok()))
        .unwrap_or(8765);
    let addr = SocketAddr::from(([0, 0, 0, 0], port));

    info!("armd API server listening on http://{}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await?;

    let sup_shutdown = supervisor.clone();
    let server = axum::serve(listener, router).with_graceful_shutdown(async move {
        let _ = tokio::signal::ctrl_c().await;
        info!("Shutdown signal received. Initiating NormalUnload...");
        sup_shutdown.apply_stop_policy(StopPolicy::NormalUnload).await;
    });

    if let Err(e) = server.await {
        warn!("Server terminated with error: {}", e);
    }

    control_handle.abort();
    info!("armd daemon stopped cleanly");
    Ok(())
}
