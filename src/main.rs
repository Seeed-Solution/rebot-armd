use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use tracing::{error, info, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::{create_router_with_auth, ArmConfig, ArmSupervisor, OperatorAuth};
#[cfg(test)]
use armd::SupervisorInner;
#[cfg(test)]
use std::sync::atomic::Ordering;

#[derive(Debug, Default)]
struct CliArgs {
    follower: Option<String>,
    follower_port: Option<String>,
    owner: Option<String>,
    port: Option<u16>,
    config: Option<String>,
    cell: Option<String>,
    cell_host: Option<String>,
    tool: Option<String>,
    tool_info: Option<String>,
    robot: Option<String>,
    log_dir: Option<String>,
    blackbox_dir: Option<String>,
    episodes_dir: Option<String>,
    operator_pin: Option<String>,
    operator_pin_file: Option<String>,
    no_operator_auth: bool,
}

fn parse_cli_args() -> Result<CliArgs, String> {
    parse_cli_args_from(std::env::args().skip(1))
}

fn parse_cli_args_from<I, S>(iter: I) -> Result<CliArgs, String>
where
    I: IntoIterator<Item = S>,
    S: Into<String>,
{
    let mut args = iter.into_iter().map(Into::into);
    let mut cli = CliArgs::default();
    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!("armd - Robotic arm daemon\n\nUSAGE: armd [OPTIONS]\n\n\
                --follower <MODEL>           rebot-rs, rebot-dm, so-arm101, fashionstar, star-arm102, sim, mock (default)\n\
                --follower-port <PORT>       CAN interface, serial device, or udp://HOST:PORT for sim (default udp://127.0.0.1:47600)\n\
                --owner <OWNER>              control-loop owner: auto (default: executor for rebot-dm and mock, legacy otherwise), legacy, or executor (precedence: CLI > ARMD_RT_OWNER > arm.armd_owner > auto)\n\
                --config <JSON>              ArmConfig profile (must match follower joint count)\n\
                --cell <JSON>                Multi-arm cell topology file (rebot-cell/v1; or ARMD_CELL; off by default)\n\
                --cell-host <ID>             Local host ID inside --cell hosts map (or ARMD_CELL_HOST; default auto-identify)\n\
                --tool <JSON>                Tool pack manifest (tool.json; or ARMD_TOOL)\n\
                --tool-info <JSON>           Validate tool.json and print single-line summary JSON to stdout\n\
                --robot <ID>                 Target robot profile for --tool-info (b601dm, so101, so100, b601rs)\n\
                --port <PORT>                HTTP / WebSocket port (default 8765)\n\
                --log-dir <DIR>              rotating log armd.log (default /userdata/rebot-teleop/logs when /userdata exists; off disables)\n\
                --blackbox-dir <DIR>         black box and boot record (default /userdata/rebot-teleop/blackbox when /userdata exists; off disables)\n\
                --episodes-dir <DIR>         episode store (teach v1) (default /userdata/rebot-teleop/episodes when /userdata exists, else ./data/episodes; off disables recording and replay)\n\
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
                | "--owner"
                | "--port"
                | "--config"
                | "--cell"
                | "--cell-host"
                | "--tool"
                | "--tool-info"
                | "--robot"
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
            "--owner" => {
                if value.trim().is_empty() {
                    return Err("--owner must not be empty".into());
                }
                cli.owner = Some(value);
            }
            "--config" => cli.config = Some(value),
            "--cell" => {
                if value.trim().is_empty() {
                    return Err("--cell must not be empty".into());
                }
                cli.cell = Some(value);
            }
            "--cell-host" => {
                if value.trim().is_empty() {
                    return Err("--cell-host must not be empty".into());
                }
                cli.cell_host = Some(value);
            }
            "--tool" => {
                if value.is_empty() {
                    return Err("--tool must not be empty".into());
                }
                cli.tool = Some(value);
            }
            "--tool-info" => {
                if value.is_empty() {
                    return Err("--tool-info must not be empty".into());
                }
                cli.tool_info = Some(value);
            }
            "--robot" => {
                if value.is_empty() {
                    return Err("--robot must not be empty".into());
                }
                cli.robot = Some(value);
            }
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
#[cfg(test)]
fn request_shutdown(inner: &mut SupervisorInner) -> bool {
    inner.request_shutdown()
}

/// Requests SCHED_FIFO 50 for the calling thread and reads the policy back
/// (`control_thread` in `GET /api/v1/diagnostics/tick`).
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
            #[cfg(not(target_arch = "wasm32"))]
            let bb = bb.with_shadow_atomics(supervisor.torque_shadow_overflow_atomics());
            info!("Previous start: {}", diagnostics::boot::summary(&rec));
            info!("Black box recording to {} (boot {})", dir.display(), rec.boot);
            if let Err(e) = diagnostics::blackbox::spawn_recorder(supervisor, bb) {
                warn!("Black box off: {e}");
                return;
            }
            // Start decoupled shadow blackbox AFTER primary recorder is live; file open happens
            // inside the `"armd-shadow-bb"` thread so shadow disk I/O never gates primary startup.
            #[cfg(not(target_arch = "wasm32"))]
            if let Err(e) = supervisor.torque_shadow_hub().attach_blackbox_dir(&dir, rec.boot, started) {
                warn!("Torque shadow blackbox {} unavailable ({e})", dir.display());
            }
            DiagnosticsState::set_global(DiagnosticsState { blackbox_dir: Some(dir), boot: rec.boot });
        }
        Err(e) => warn!("Black box directory {} unavailable ({e}); black box off", dir.display()),
    }
}

const DEFAULT_APP_CONFIG_PATH: &str = "/userdata/local/apps/rebot-teleop.config.json";

/// Reads `arm.armd_owner` (falling back to top-level `armd_owner`) from the app config JSON
/// (`ARMD_APP_CONFIG`, or `/userdata/local/apps/rebot-teleop.config.json` when present).
/// Fails closed on unreadable/malformed files or invalid values so a typo (e.g. `"legcay"`)
/// never silently falls through to `auto` and switches real hardware to `executor`.
fn read_app_config_armd_owner_from_path(path: &std::path::Path) -> Result<Option<String>, String> {
    let raw = match std::fs::read_to_string(path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(format!(
                "Failed to read app config {} for arm.armd_owner: {e}",
                path.display()
            ));
        }
    };
    let parsed: serde_json::Value = serde_json::from_str(&raw).map_err(|e| {
        format!(
            "Failed to parse app config {} for arm.armd_owner: {e}",
            path.display()
        )
    })?;
    let root = parsed.as_object().ok_or_else(|| {
        format!(
            "App config {} must be a JSON object when checking arm.armd_owner",
            path.display()
        )
    })?;
    let arm_owner_val = match root.get("arm") {
        None | Some(serde_json::Value::Null) => None,
        Some(arm) => {
            let arm_obj = arm.as_object().ok_or_else(|| {
                format!(
                    "App config {} field 'arm' must be a JSON object when checking arm.armd_owner",
                    path.display()
                )
            })?;
            arm_obj.get("armd_owner").filter(|v| !v.is_null())
        }
    };
    let owner_val = arm_owner_val.or_else(|| root.get("armd_owner").filter(|v| !v.is_null()));
    let Some(val) = owner_val else {
        return Ok(None);
    };
    let Some(s) = val.as_str() else {
        return Err(format!(
            "arm.armd_owner in {} must be 'auto', 'legacy', or 'executor' (got non-string JSON value)",
            path.display()
        ));
    };
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }
    Ok(Some(trimmed.to_string()))
}

fn read_app_config_armd_owner() -> Result<Option<String>, String> {
    if let Some(raw) = std::env::var_os("ARMD_APP_CONFIG") {
        let s = raw.to_string_lossy();
        let trimmed = s.trim();
        if trimmed.is_empty()
            || trimmed.eq_ignore_ascii_case("off")
            || trimmed.eq_ignore_ascii_case("none")
        {
            return Ok(None);
        }
        return read_app_config_armd_owner_from_path(std::path::Path::new(&raw));
    }
    read_app_config_armd_owner_from_path(std::path::Path::new(DEFAULT_APP_CONFIG_PATH))
}

fn read_env_rt_owner_os(raw: Option<&std::ffi::OsStr>) -> Result<Option<String>, String> {
    let Some(os) = raw else {
        return Ok(None);
    };
    let Some(s) = os.to_str() else {
        return Err(
            "ARMD_RT_OWNER must be valid UTF-8 ('auto', 'legacy', or 'executor')".into(),
        );
    };
    let trimmed = s.trim();
    if trimmed.is_empty() {
        Ok(None)
    } else {
        Ok(Some(trimmed.to_string()))
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

#[cfg(test)]
fn resolve_follower_config(follower: &str, config_path: Option<&str>) -> Result<ArmConfig, String> {
    resolve_follower_config_with_path(follower, config_path).map(|(cfg, _)| cfg)
}

fn resolve_follower_config_with_path(
    follower: &str,
    config_path: Option<&str>,
) -> Result<(ArmConfig, Option<std::path::PathBuf>), String> {
    let expected_dof = match follower {
        "rebot-dm" => Some(7),
        "rebot-rs" | "so-arm101" | "so-arm100" | "fashionstar" | "star-arm102" => Some(6),
        "sim" | "mock" => None,
        other => return Err(format!("Unknown follower model: {other}")),
    };
    let (mut config, loaded_profile_path) = if let Some(path) = config_path {
        let config = ArmConfig::from_profile_file(path)?;
        if config.kinematics.is_none() || config.workspace.is_none() {
            tracing::warn!("Profile {path} has no kinematics/workspace: the Cartesian workspace check and Cartesian actions are off");
        }
        (config, Some(std::path::PathBuf::from(path)))
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
        (ArmConfig::from_profile_file(path)?, Some(std::path::PathBuf::from(path)))
    } else if follower == "rebot-dm" || follower == "sim" {
        (ArmConfig::rebot_dm(), None)
    } else {
        (ArmConfig::default(), None)
    };
    config.validate()?;
    if expected_dof.is_some_and(|dof| config.dof != dof) {
        return Err(format!(
            "Profile DOF {} does not match {follower} hardware DOF {}",
            config.dof,
            expected_dof.unwrap()
        ));
    }
    config.validate_step1_driver_parity(follower)?;
    Ok((config, loaded_profile_path))
}

fn apply_cli_tool(config: ArmConfig, cli_tool: Option<&str>) -> Result<ArmConfig, String> {
    let env_tool = std::env::var_os("ARMD_TOOL").filter(|s| !s.is_empty());
    apply_cli_tool_with_env(config, cli_tool, env_tool.as_deref())
}

fn apply_cli_tool_with_env(
    config: ArmConfig,
    cli_tool: Option<&str>,
    env_tool: Option<&std::ffi::OsStr>,
) -> Result<ArmConfig, String> {
    let path: std::path::PathBuf = match cli_tool {
        Some(p) => std::path::PathBuf::from(p),
        None => match env_tool {
            Some(p) if !p.is_empty() => std::path::PathBuf::from(p),
            _ => return Ok(config),
        },
    };
    let pack = armd::tool::load_tool_file(&path)?;
    armd::tool::validate_reserved_stock_pack(&pack, None)?;
    let updated = armd::tool::apply_tool_overlay(&config, &pack)?;
    info!(
        "Loaded tool '{}' (digest {}, dynamics_qualified={}) from {}",
        updated.active_tool.id,
        updated.active_tool.config_digest,
        updated.dynamics_qualified,
        path.display()
    );
    Ok(updated)
}

fn run_tool_info(path: &str, robot: Option<&str>) -> Result<String, String> {
    let pack = armd::tool::load_tool_file(path)?;
    let summary = armd::tool::resolve_tool_config_summary(&pack, robot)?;
    serde_json::to_string(&summary).map_err(|e| format!("serialize tool info: {e}"))
}

fn resolve_hardware_follower_device(follower: &str, requested: Option<&str>) -> Option<String> {
    match follower {
        "rebot-rs" | "rebot-dm" => Some(requested.unwrap_or("can0").to_string()),
        "so-arm101" | "so-arm100" => Some(resolve_serial_port(requested)),
        "fashionstar" | "star-arm102" => Some(requested.unwrap_or("/dev/ttyUSB0").to_string()),
        "mock" | "sim" => None,
        _ => None,
    }
}

fn resolve_cell_context(
    cli: &CliArgs,
    config: &ArmConfig,
    loaded_profile_path: Option<&std::path::Path>,
    follower: &str,
    resolved_hw_device: Option<&str>,
    port: u16,
) -> Result<Option<Arc<armd::cell::CellContext>>, String> {
    let env_cell = std::env::var("ARMD_CELL")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty());
    let env_cell_host = std::env::var("ARMD_CELL_HOST")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty());
    resolve_cell_context_with_env(
        cli,
        config,
        loaded_profile_path,
        follower,
        resolved_hw_device,
        port,
        env_cell.as_deref(),
        env_cell_host.as_deref(),
    )
}

fn resolve_cell_context_with_env(
    cli: &CliArgs,
    config: &ArmConfig,
    loaded_profile_path: Option<&std::path::Path>,
    follower: &str,
    resolved_hw_device: Option<&str>,
    port: u16,
    env_cell: Option<&str>,
    env_cell_host: Option<&str>,
) -> Result<Option<Arc<armd::cell::CellContext>>, String> {
    let cell_host = cli
        .cell_host
        .as_deref()
        .or(env_cell_host)
        .or(config.cell_host.as_deref())
        .map(str::trim)
        .filter(|s| !s.is_empty());

    let cell_path = if let Some(p) = cli.cell.as_deref() {
        Some(std::path::PathBuf::from(p))
    } else if let Some(p) = env_cell {
        Some(std::path::PathBuf::from(p))
    } else if let Some(p) = config.cell.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
        let candidate = std::path::Path::new(p);
        if candidate.is_absolute() {
            Some(candidate.to_path_buf())
        } else if let Some(dir) = loaded_profile_path.and_then(std::path::Path::parent).filter(|d| !d.as_os_str().is_empty()) {
            Some(dir.join(candidate))
        } else {
            Some(candidate.to_path_buf())
        }
    } else {
        None
    };

    let Some(path) = cell_path else {
        if cell_host.is_some() {
            return Err(
                "--cell-host / ARMD_CELL_HOST / profile cell_host requires a cell config (--cell, ARMD_CELL, or profile 'cell')"
                    .into(),
            );
        }
        return Ok(None);
    };

    let cell_cfg = armd::cell::CellConfig::from_json_file(&path)?;
    let local_port = (port != 0).then_some(port);
    let ctx = armd::cell::CellContext::resolve_local(
        cell_cfg,
        cell_host,
        local_port,
        config,
        Some(follower),
        resolved_hw_device,
    )?;
    info!(
        "Loaded cell '{}' from {} (host={}, role={}, arm={:?}, coordinator={}, digest={})",
        ctx.config.cell_id,
        path.display(),
        ctx.local_host_id,
        ctx.local_role(),
        ctx.local_arm_id,
        ctx.config.coordinator,
        ctx.config_digest
    );
    Ok(Some(Arc::new(ctx)))
}

/// The system allocator, timed on the control thread with `ARMD_ALLOC_TIMING=1`
/// (src/diagnostics/alloc.rs).
#[global_allocator]
static GLOBAL: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

/// Finalizes daemon shutdown after `supervisor.request_shutdown_until(deadline)` confirmed readiness.
///
/// The API server stays live until the active owner's final shutdown commit succeeds within ONE
/// shared `deadline`. Resolves the authoritative active writer dynamically from `supervisor`
/// (Executor, Legacy, or DetachedSafeHold) so post-handback Legacy and detached startup-loser
/// SAFE-HOLD writers are always reached on deliberate process exit (B3 / B4).
#[cfg(test)]
async fn finalize_shutdown(
    supervisor: &Arc<ArmSupervisor>,
    server_handle: &mut tokio::task::JoinHandle<std::io::Result<()>>,
    executor_handle: Option<armd::rt_core::executor::RtExecutorHandle>,
    legacy_control_stop: Option<Arc<std::sync::atomic::AtomicBool>>,
) -> Result<(), String> {
    let deadline = std::time::Instant::now()
        + armd::supervisor::PANIC_DISABLE_TIMEOUT
        + Duration::from_millis(100);
    finalize_shutdown_with_hook_until(
        supervisor,
        server_handle,
        executor_handle,
        legacy_control_stop,
        deadline,
        || async {},
    )
    .await
}

#[cfg(test)]
async fn finalize_shutdown_with_hook<F, Fut>(
    supervisor: &Arc<ArmSupervisor>,
    server_handle: &mut tokio::task::JoinHandle<std::io::Result<()>>,
    executor_handle: Option<armd::rt_core::executor::RtExecutorHandle>,
    legacy_control_stop: Option<Arc<std::sync::atomic::AtomicBool>>,
    after_legacy_commit_before_abort: F,
) -> Result<(), String>
where
    F: FnOnce() -> Fut,
    Fut: std::future::Future<Output = ()>,
{
    let deadline = std::time::Instant::now()
        + armd::supervisor::PANIC_DISABLE_TIMEOUT
        + Duration::from_millis(100);
    finalize_shutdown_with_hook_until(
        supervisor,
        server_handle,
        executor_handle,
        legacy_control_stop,
        deadline,
        after_legacy_commit_before_abort,
    )
    .await
}

async fn finalize_shutdown_with_hook_until<F, Fut>(
    supervisor: &Arc<ArmSupervisor>,
    server_handle: &mut tokio::task::JoinHandle<std::io::Result<()>>,
    _executor_handle: Option<armd::rt_core::executor::RtExecutorHandle>,
    legacy_control_stop: Option<Arc<std::sync::atomic::AtomicBool>>,
    deadline: std::time::Instant,
    after_legacy_commit_before_abort: F,
) -> Result<(), String>
where
    F: FnOnce() -> Fut,
    Fut: std::future::Future<Output = ()>,
{
    // Reserve part of the ONE shared `deadline` for fallback emergency ProcessExit disable so
    // initial commit attempts never exhaust the entire budget and fallback never mints a fresh
    // timeout (DRV-F1 B11).
    let now = std::time::Instant::now();
    let total_rem = deadline.saturating_duration_since(now);
    let commit_deadline = now + total_rem / 2;

    // Resolve authoritative active writer dynamically from `supervisor` (never preferring a stale
    // startup `executor_handle` once handback switched ownership to Legacy — DRV-F1 B3).
    let active = supervisor.active_writer_endpoint();
    match active {
        armd::supervisor::WriterEndpoint::DetachedSafeHold { .. }
        | armd::supervisor::WriterEndpoint::Transferring => {
            error!("Final shutdown with detached SAFE-HOLD or transferring writer: issuing deliberate ProcessExit protective disable");
            server_handle.abort();
            let outcome = supervisor.emergency_process_exit_disable_until(deadline).await;
            outcome.log(
                "Detached SAFE-HOLD process-exit torque-off",
                armd::supervisor::PANIC_DISABLE_TIMEOUT,
            );
            return Err(format!(
                "Daemon exiting from detached SAFE-HOLD (process-exit torque-off: {outcome:?})"
            ));
        }
        armd::supervisor::WriterEndpoint::Executor(handle) => {
            let h_commit = handle.clone();
            let commit_res = tokio::task::spawn_blocking(move || {
                h_commit.shutdown_and_join_until(commit_deadline)
            })
            .await;
            match commit_res {
                Ok(Ok(())) => {
                    server_handle.abort();
                    info!("armd stopped after shutdown readiness confirmation");
                    return Ok(());
                }
                Ok(Err(e)) => {
                    error!("Final shutdown commit refused/timed out by RT owner: {e}");
                    server_handle.abort();
                    let outcome = supervisor
                        .emergency_process_exit_disable_until(deadline)
                        .await;
                    outcome.log(
                        "Refused final shutdown torque-off",
                        armd::supervisor::PANIC_DISABLE_TIMEOUT,
                    );
                    return Err(format!(
                        "Final shutdown commit refused by RT owner: {e} (torque-off: {outcome:?})"
                    ));
                }
                Err(je) => {
                    error!("Final shutdown commit task failed: {je}");
                    server_handle.abort();
                    let outcome = supervisor
                        .emergency_process_exit_disable_until(deadline)
                        .await;
                    outcome.log(
                        "Failed final shutdown torque-off",
                        armd::supervisor::PANIC_DISABLE_TIMEOUT,
                    );
                    return Err(format!(
                        "Final shutdown commit task failed: {je} (torque-off: {outcome:?})"
                    ));
                }
            }
        }
        armd::supervisor::WriterEndpoint::Legacy {
            control_stop: ep_stop,
            ..
        } => {
            if let Some(control_stop) = ep_stop.or(legacy_control_stop) {
                let lock_rem = commit_deadline.saturating_duration_since(std::time::Instant::now());
                let confirmed = match tokio::time::timeout(lock_rem, supervisor.inner().write()).await
                {
                    Ok(mut inner) => inner.commit_shutdown_until(commit_deadline),
                    Err(_) => false,
                };
                if !confirmed {
                    error!("Final legacy shutdown recheck refused or timed out: hardware disable is not confirmed");
                    server_handle.abort();
                    supervisor.estop_state.latch();
                    supervisor.signal_legacy_stop(&control_stop);
                    let outcome = supervisor
                        .emergency_process_exit_disable_until(deadline)
                        .await;
                    outcome.log(
                        "Refused legacy final shutdown torque-off",
                        armd::supervisor::PANIC_DISABLE_TIMEOUT,
                    );
                    return Err(format!(
                        "Final legacy shutdown recheck refused: hardware disable is not confirmed (torque-off: {outcome:?})"
                    ));
                }
                after_legacy_commit_before_abort().await;
                server_handle.abort();
                supervisor.signal_legacy_stop(&control_stop);
                #[cfg(not(target_arch = "wasm32"))]
                supervisor
                    .torque_shadow_hub()
                    .stop_and_flush_bounded(Duration::from_millis(100));
                info!("armd stopped after shutdown readiness confirmation");
                return Ok(());
            }
        }
    }

    server_handle.abort();
    #[cfg(not(target_arch = "wasm32"))]
    supervisor
        .torque_shadow_hub()
        .stop_and_flush_bounded(Duration::from_millis(100));
    info!("armd stopped after shutdown readiness confirmation");
    Ok(())
}

struct StartedControlOwner {
    executor_handle: Option<armd::rt_core::executor::RtExecutorHandle>,
    legacy_control_stop: Option<Arc<std::sync::atomic::AtomicBool>>,
    #[cfg_attr(not(test), allow(dead_code))]
    fell_back_to_legacy: bool,
}

fn spawn_legacy_control_loop(
    supervisor: &Arc<ArmSupervisor>,
    period: Duration,
    rt_sched: armd::rt_core::sched::RtSchedConfig,
    memory_lock: armd::rt_core::sched::MemoryLock,
    alloc_timing: bool,
    apply_rt_sched: bool,
) -> Result<Arc<std::sync::atomic::AtomicBool>, String> {
    let control_supervisor = supervisor.clone();
    let control_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let control_stop_flag = control_stop.clone();
    let executor_stats = Arc::new(armd::rt_core::executor::ExecutorStats::default());
    supervisor.attach_legacy_loop_stats(executor_stats.clone());
    let loop_stats = executor_stats.clone();
    let runtime = tokio::runtime::Handle::current();
    std::thread::Builder::new()
        .name("armd-control".into())
        .spawn(move || {
            let sched = if apply_rt_sched {
                let s = armd::rt_core::sched::apply_to_current_thread(&rt_sched, &memory_lock);
                if s.realtime {
                    info!(
                        "Control loop running at SCHED_FIFO priority {}{}",
                        s.priority,
                        s.cpu.map(|c| format!(" on CPU {c}")).unwrap_or_default()
                    );
                } else {
                    warn!(
                        "Control loop real-time scheduling unavailable ({}); using normal scheduling, MIT tracking (when configured) refused",
                        s.errors.join("; ")
                    );
                }
                armd::rt_core::sched::set_readiness(armd::rt_core::sched::RtReadiness::from_sched(&s));
                s
            } else {
                armd::rt_core::sched::observe_current_thread(&memory_lock)
            };
            control_supervisor.set_control_thread_sched(sched);
            if alloc_timing {
                armd::diagnostics::alloc::time_this_thread();
                loop_stats
                    .alloc_timing_enabled
                    .store(true, std::sync::atomic::Ordering::Release);
                info!("Control loop allocator timing on (ARMD_ALLOC_TIMING=1)");
            }
            control_supervisor.run_legacy_control_loop_blocking(
                period,
                control_stop_flag,
                loop_stats,
                runtime,
                None,
            );
        })
        .map_err(|e| format!("Failed to spawn legacy control loop thread: {e}"))?;
    if let Err(e) = executor_stats.spawn_monitor(control_stop.clone()) {
        warn!(
            "Failed to spawn legacy control stats monitor ({e}); continuing with running Legacy control loop"
        );
    }
    Ok(control_stop)
}

async fn start_control_owner_with_hooks(
    supervisor: &Arc<ArmSupervisor>,
    follower: &str,
    rt_owner: armd::rt_core::executor::RtOwner,
    period: Duration,
    rt_sched: armd::rt_core::sched::RtSchedConfig,
    memory_lock: armd::rt_core::sched::MemoryLock,
    alloc_timing: bool,
    apply_rt_sched: bool,
    panic_policy: armd::rt_core::executor::PanicPolicy,
    hooks: armd::rt_core::executor::PersistTestHooks,
) -> Result<StartedControlOwner, String> {
    match rt_owner {
        armd::rt_core::executor::RtOwner::Legacy => {
            let stop = spawn_legacy_control_loop(
                supervisor,
                period,
                rt_sched,
                memory_lock,
                alloc_timing,
                apply_rt_sched,
            )?;
            Ok(StartedControlOwner {
                executor_handle: None,
                legacy_control_stop: Some(stop),
                fell_back_to_legacy: false,
            })
        }
        armd::rt_core::executor::RtOwner::Executor => {
            let fallback_sched = rt_sched;
            let fallback_mlock = memory_lock.clone();
            let raw_sub_release_margin = match std::env::var("ARMD_SUB_RELEASE_MARGIN_MS") {
                Ok(v) => Some(v),
                Err(std::env::VarError::NotPresent) => None,
                Err(std::env::VarError::NotUnicode(_)) => {
                    return Err("ARMD_SUB_RELEASE_MARGIN_MS must be valid UTF-8".into());
                }
            };
            let sub_release_margin = armd::rt_core::ops::parse_sub_release_margin_ms(
                raw_sub_release_margin.as_deref(),
                period,
            )?;
            let cfg = armd::rt_core::executor::ExecutorConfig {
                period,
                boot: armd::rt_core::broker::BootId(1),
                capacities: armd::rt_core::broker::Capacities::default(),
                rt_sched,
                memory_lock,
                alloc_timing,
                panic_policy,
                apply_rt_sched,
                sub_release_margin,
            };
            match supervisor.start_executor_with_hooks(cfg, hooks).await {
                Ok(handle) => {
                    info!(
                        "RT executor owns ArmCore and bus (follower={follower}, owner={})",
                        rt_owner.as_str()
                    );
                    Ok(StartedControlOwner {
                        executor_handle: Some(handle),
                        legacy_control_stop: None,
                        fell_back_to_legacy: false,
                    })
                }
                Err(e) if supervisor.has_detached_safe_hold() => {
                    // B4: Keep daemon alive while a detached startup loser exclusively holds an
                    // energized/unknown driver in SAFE-HOLD until deliberate process-exit signal.
                    error!(
                        "RT executor startup failed on follower '{follower}' while detached writer holds driver in SAFE-HOLD ({e}); keeping daemon alive until deliberate shutdown signal without torque-off"
                    );
                    supervisor
                        .set_owner_reason(armd::OwnerReason::StartupFailedDetachedSafeHold)
                        .await;
                    Ok(StartedControlOwner {
                        executor_handle: None,
                        legacy_control_stop: None,
                        fell_back_to_legacy: false,
                    })
                }
                Err(e)
                    if follower != "mock"
                        && supervisor.owner_mode() == armd::supervisor::OWNER_LEGACY =>
                {
                    // DRV-F1 handback: executor startup failed recoverably and returned `ArmCore`
                    // and the hardware driver cleanly to Legacy ownership. Start the legacy
                    // control loop holding immediately — never exit and never torque-off away from rest.
                    error!(
                        "RT executor startup failed on real follower '{follower}' ({e}); falling back to legacy owner holding (DRV-F1 handback) without torque-off"
                    );
                    supervisor
                        .set_owner_reason(armd::OwnerReason::StartupFallbackToLegacy)
                        .await;
                    let stop = spawn_legacy_control_loop(
                        supervisor,
                        period,
                        fallback_sched,
                        fallback_mlock,
                        alloc_timing,
                        apply_rt_sched,
                    )
                    .map_err(|leg_err| {
                        format!(
                            "RT executor startup failed ({e}) and legacy fallback control loop failed to start: {leg_err}"
                        )
                    })?;
                    Ok(StartedControlOwner {
                        executor_handle: None,
                        legacy_control_stop: Some(stop),
                        fell_back_to_legacy: true,
                    })
                }
                Err(e) => Err(format!("Failed to start RT executor: {e}")),
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let started = std::time::Instant::now();
    let _ = armd::supervisor::init_process_identity();
    let system = armd::diagnostics::boot::SystemInfo::read();
    let cli = parse_cli_args()?;

    if let Some(tool_path) = cli.tool_info.as_deref() {
        match run_tool_info(tool_path, cli.robot.as_deref()) {
            Ok(line) => {
                println!("{line}");
                std::process::exit(0);
            }
            Err(err) => {
                eprintln!("{err}");
                std::process::exit(1);
            }
        }
    }
    if cli.robot.is_some() {
        return Err("--robot is only valid with --tool-info".into());
    }

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
    let stdout_writer = armd::diagnostics::logwriter::NonBlocking::spawn("armd-log-stdout", std::io::stdout()).ok();
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "armd=info,tower_http=info".into()),
        )
        // Both writers run on threads of their own: the control tick logs while it holds the
        // supervisor lock, and a write to stdout or the flash took up to 45 ms (logwriter.rs).
        .with(stdout_writer.clone().map(|w| tracing_subscriber::fmt::layer().with_writer(w)))
        .with(stdout_writer.is_none().then(tracing_subscriber::fmt::layer))
        .with(log_file.and_then(|file| armd::diagnostics::logwriter::NonBlocking::spawn("armd-log-file", file).ok())
            .map(|w| tracing_subscriber::fmt::layer().with_ansi(false).with_writer(w)))
        .init();
    // Writes the queued log lines when main returns, also with an error.
    let _flush_log = armd::diagnostics::logwriter::FlushOnDrop(Duration::from_millis(500));
    if let Some(warning) = log_warning {
        warn!("{warning}");
    }
    // Memory lock before any other thread starts (MCL_FUTURE covers later mappings too).
    let rt_sched = armd::rt_core::sched::RtSchedConfig::from_env()?;
    let memory_lock = armd::rt_core::sched::lock_memory(&rt_sched);
    match &memory_lock {
        armd::rt_core::sched::MemoryLock::Locked => info!("Process memory locked (mlockall MCL_CURRENT|MCL_FUTURE)"),
        m => warn!("Process memory not locked ({}); MIT tracking, when configured, is refused", m.describe()),
    }
    let follower = cli.follower.as_deref().unwrap_or("mock");
    let env_rt_owner = if cli.owner.is_none() {
        read_env_rt_owner_os(std::env::var_os("ARMD_RT_OWNER").as_deref())?
    } else {
        None
    };
    let config_rt_owner = if cli.owner.is_none() && env_rt_owner.is_none() {
        read_app_config_armd_owner()?
    } else {
        None
    };
    let (requested_owner, _) = armd::rt_core::executor::RequestedRtOwner::resolve(
        cli.owner.as_deref(),
        env_rt_owner.as_deref(),
        config_rt_owner.as_deref(),
    )?;
    if requested_owner == armd::rt_core::executor::RequestedRtOwner::Executor {
        armd::rt_core::executor::validate_startup_owner(
            armd::rt_core::executor::RtOwner::Executor,
            follower,
        )?;
    }
    let (config, loaded_profile_path) =
        resolve_follower_config_with_path(follower, cli.config.as_deref())?;
    let mut config = apply_cli_tool(config, cli.tool.as_deref())?;
    let resolved_owner = armd::rt_core::executor::resolve_startup_owner(
        cli.owner.as_deref(),
        env_rt_owner.as_deref(),
        config_rt_owner.as_deref(),
        follower,
        &config,
    )?;
    let rt_owner = resolved_owner.effective;
    if let Some(detail) = resolved_owner.fallback_detail.as_deref() {
        info!(
            "Startup owner auto resolved to legacy (reason={}): {detail}",
            resolved_owner.reason.as_str()
        );
    } else {
        info!(
            "Startup owner resolved: effective={} reason={} follower={follower}",
            resolved_owner.effective.as_str(),
            resolved_owner.reason.as_str()
        );
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let shadow_env = std::env::var(armd::torque::shadow::ENV_TORQUE_SHADOW).ok();
        config.torque_shadow_enabled = armd::torque::shadow::resolve_shadow_enabled(
            config.torque_shadow_enabled,
            shadow_env.as_deref(),
        )?;
    }
    config.validate_step1_driver_parity(follower)?;
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
    // Resolve hardware device path once so cell validation and driver mounting use the exact same device.
    let resolved_hw_device = resolve_hardware_follower_device(follower, cli.follower_port.as_deref());
    // Validate optional multi-arm cell config before binding ports or mounting hardware.
    let cell_context = resolve_cell_context(
        &cli,
        &config,
        loaded_profile_path.as_deref(),
        follower,
        resolved_hw_device.as_deref(),
        port,
    )?;
    // Bind before mounting hardware; an unavailable API must not leave a hardware loop running.
    let bind_ip: std::net::IpAddr = match std::env::var("ARMD_BIND_HOST") {
        Ok(s) if !s.trim().is_empty() => s
            .trim()
            .parse()
            .map_err(|_| format!("Invalid ARMD_BIND_HOST: {s}"))?,
        _ => std::net::IpAddr::V4(std::net::Ipv4Addr::UNSPECIFIED),
    };
    let listener = tokio::net::TcpListener::bind(SocketAddr::new(bind_ip, port)).await?;
    #[cfg_attr(not(target_os = "linux"), allow(unused_variables))]
    let damiao_limits: Option<Vec<_>> = config.damiao_ranges.as_ref().map(|r| r.iter().map(|r| r.limits()).collect());
    let config_dof = config.dof;
    let feetech_config = config.feetech.clone();
    let gripper_index = config.gripper_index();
    #[cfg_attr(not(target_os = "linux"), allow(unused_variables))]
    let gripper_drive = armd::driver::gripper_drive_from_profile(&config)?;
    #[cfg_attr(not(target_os = "linux"), allow(unused_variables))]
    let damiao_tx_budget = config.damiao_tx_budget();
    let supervisor = Arc::new(ArmSupervisor::new(config));
    supervisor.set_owner_reason(resolved_owner.reason).await;
    // Install the process-wide lock-free/zero-alloc tiered panic coordinator strictly BEFORE
    // mounting or enabling any hardware (FU-PANIC / DRV-F1).
    armd::panic_coord::install_process_panic_coordinator(Arc::clone(
        supervisor.internal_fault_lane(),
    ));
    // before the control loop: frees retired observations, feeds telemetry and recordings
    supervisor.start_dispatcher()?;
    supervisor.set_follower(follower).await;
    if let Some(cell_ctx) = cell_context {
        supervisor.set_cell(Some(cell_ctx)).await?;
    }
    start_blackbox(&supervisor, cli.blackbox_dir.as_deref(), &system, started);
    let episodes = armd::recording::EpisodeStore::resolve(cli.episodes_dir.as_deref());
    match episodes.dir() {
        Some(dir) => info!("Episodes directory {}", dir.display()),
        None => info!("Episode storage off: recording and replay are refused"),
    }
    supervisor.set_episodes_dir(episodes.dir().map(std::path::Path::to_path_buf)).await;
    let calib_path = armd::calibration::path_from_env();
    let grasp_setup_path = armd::grasp::setup_path_from_env();
    let pending_marker_state = armd::calibration::inspect_pending_commit_marker(&calib_path);
    let pending_calib_commit = !matches!(
        pending_marker_state,
        armd::calibration::PendingMarkerState::Absent
    );
    if !pending_calib_commit {
        armd::grasp::load_into(&supervisor, &grasp_setup_path).await;
    }
    // after the grasp setup: the calibration wins for camera_mount and camera_tilt_deg
    // (when a pending calibration/zero commit marker is present or unreadable,
    // calibration::load_into_with_marker_state quarantines startup into Reconciling without
    // installing uncertain calibration/offsets).
    if let Err(e) = armd::calibration::load_into_with_marker_state(
        &supervisor,
        &calib_path,
        Some(&grasp_setup_path),
        &pending_marker_state,
    )
    .await
    {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, e).into());
    }
    // after the calibration: legacy episodes are adopted with the joint zero in effect;
    // skip adoption during quarantined startup so fallback zeros are never stamped onto sidecars.
    if !pending_calib_commit {
        supervisor.load_episodes().await;
    }
    if let Err(e) = armd::keepout::load_into(&supervisor, &armd::keepout::path_from_env()).await {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, e).into());
    }
    if !pending_calib_commit {
        armd::poses::load_into(&supervisor, &armd::poses::path_from_env()).await;
    }
    armd::sequences::load_into(&supervisor, &armd::sequences::path_from_env()).await;
    if !pending_calib_commit {
        armd::calib_hover::load_into(&supervisor, &calib_path).await;
    }
    armd::collision::load_into(&supervisor, &armd::collision::path_from_env()).await;
    armd::tool::load_commissioning_into(&supervisor, &armd::tool::commissioning_path_from_env()).await;
    match follower {
        "rebot-rs" | "rebot-dm" => {
            #[cfg(target_os = "linux")]
            {
                let port = resolved_hw_device.as_deref().unwrap_or("can0");
                let can = armd::transport::SocketCanTransport::open(port)?;
                if follower == "rebot-dm" {
                    let mut driver = armd::driver::DamiaoArmDriver::new(can, (1..=7).collect())
                        .with_tx_budget(damiao_tx_budget);
                    if let Some(limits) = damiao_limits {
                        driver = driver.with_joint_limits(limits)?;
                    }
                    driver = driver.with_gripper_drive(gripper_drive)?;
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
            let port = resolved_hw_device
                .as_deref()
                .map(str::to_string)
                .unwrap_or_else(|| resolve_serial_port(cli.follower_port.as_deref()));
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
            let port = resolved_hw_device.as_deref().unwrap_or("/dev/ttyUSB0");
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
    let gripper_calib_path = armd::gripper_calib::path_from_env();
    if !pending_calib_commit {
        armd::gripper_calib::load_into(&supervisor, &gripper_calib_path).await;
    } else {
        armd::gripper_calib::cleanup_staged_artifacts(&gripper_calib_path);
    }
    // Complete all startup configuration BEFORE transferring ownership or starting the control loop.
    let operator_auth = resolve_operator_auth(follower, &cli)?;
    supervisor.set_operator_auth_enforced(operator_auth.is_enforced()).await;

    // One owner selected at startup: legacy supervisor-lock loop (default) or the synchronous
    // RT executor OS thread owning `ArmCore` and the bus with zero Tokio entry.
    let alloc_timing = std::env::var("ARMD_ALLOC_TIMING").is_ok_and(|v| v == "1");
    let started_owner = start_control_owner_with_hooks(
        &supervisor,
        follower,
        rt_owner,
        period,
        rt_sched,
        memory_lock,
        alloc_timing,
        true,
        armd::rt_core::executor::PanicPolicy::AbortProcess,
        armd::rt_core::executor::PersistTestHooks::default(),
    )
    .await
    .map_err(|e| -> Box<dyn std::error::Error> { e.into() })?;
    let legacy_control_stop = started_owner.legacy_control_stop;
    let executor_handle = started_owner.executor_handle;
    // Start the off-control-thread automatic gripper baseline consumer for both owners.
    armd::gripper_calib::spawn_baseline_persister(supervisor.clone());

    info!(
        "armd API server listening on http://{}",
        listener.local_addr()?
    );
    let router = create_router_with_auth(supervisor.clone(), operator_auth);
    let mut server_handle = tokio::spawn(async move {
        axum::serve(
            listener,
            router.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .await
    });
    let mut server_finished = false;
    #[cfg(unix)]
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;

    let mut shutdown_deadline;
    let mut detached_safe_hold_sig_seen = false;
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

        // B3: Start ONE absolute deadline at daemon shutdown request entry and carry it across
        // admission (`request_shutdown_until`), commit (`CommitShutdown`), and teardown.
        shutdown_deadline = std::time::Instant::now()
            + armd::supervisor::PANIC_DISABLE_TIMEOUT
            + Duration::from_millis(100);
        if supervisor.has_detached_safe_hold() {
            if detached_safe_hold_sig_seen {
                warn!("Repeated shutdown signal while detached SAFE-HOLD holds driver: proceeding to deliberate ProcessExit protective disable");
                break;
            }
            detached_safe_hold_sig_seen = true;
            warn!("Shutdown signal received while detached SAFE-HOLD holds driver: arm stays held; send shutdown signal again for deliberate ProcessExit protective disable");
            continue;
        }
        match supervisor.request_shutdown_until(shutdown_deadline).await {
            Ok(true) => break,
            Ok(false) => {
                warn!("Shutdown refused: hardware disable is not confirmed. Stop is latched; control loop and API remain available. Explicit fault recovery and rearming are required before further motion.");
            }
            Err(e) => {
                warn!("Shutdown request failed/timed out ({e}); keeping daemon active");
            }
        }
    }

    finalize_shutdown_with_hook_until(
        &supervisor,
        &mut server_handle,
        executor_handle,
        legacy_control_stop,
        shutdown_deadline,
        || async {},
    )
    .await
    .map_err(|e| -> Box<dyn std::error::Error> { e.into() })?;
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
        fn has_driver_fresh_position_api(&self) -> bool {
            true
        }
        fn fresh_joint_positions_into(
            &self,
            joints: &[usize],
            _max_age: Duration,
            out: &mut [f32],
        ) -> bool {
            if out.len() < joints.len() {
                return false;
            }
            out[..joints.len()].fill(0.0);
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
        } else if state == EnableState::Disabled {
            inner.torque_intent = armd::TorqueIntent::Off;
        }
        inner
    }

    #[test]
    fn refused_shutdown_latches_stop_until_explicit_recovery_and_rearm() {
        let mut inner = physical(EnableState::Enabled);
        let old_epoch = inner.safety.epoch_id();
        let st = inner.current_admission_stamp();
        inner.check_admission(st).and_then(|a| SupervisorInner::servo(a, vec![0.1; 6], old_epoch)).unwrap();
        assert!(!request_shutdown(&mut inner));
        assert_eq!(inner.safety.state(), &SafetyState::EStop);
        let new_epoch = inner.safety.epoch_id();
        assert_ne!(new_epoch, old_epoch);
        let st = inner.current_admission_stamp();
        assert!(inner.check_admission(st).and_then(|a| SupervisorInner::servo(a, vec![0.1; 6], old_epoch)).is_err());
        let st = inner.current_admission_stamp();
        assert!(inner.check_admission(st).and_then(|a| SupervisorInner::servo(a, vec![0.1; 6], new_epoch)).is_err());
        let st = inner.current_admission_stamp();
        assert!(inner.check_admission(st).and_then(SupervisorInner::enable_torque).is_err());
        inner.clear_fault().unwrap();
        assert_eq!(inner.safety.state(), &SafetyState::Idle);
        let st = inner.current_admission_stamp();
        let ep_now = inner.safety.epoch_id();
        assert!(inner.check_admission(st).and_then(|a| SupervisorInner::servo(a, vec![0.1; 6], ep_now)).is_err());
        let st = inner.current_admission_stamp();
        let adm = inner.check_admission(st).unwrap();
        SupervisorInner::enable_torque(adm).unwrap();
        let st = inner.current_admission_stamp();
        let ep_now = inner.safety.epoch_id();
        assert!(inner.check_admission(st).and_then(|a| SupervisorInner::servo(a, vec![0.1; 6], ep_now)).is_ok());
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
        let st = inner.current_admission_stamp();
        assert!(inner.check_admission(st).and_then(SupervisorInner::enable_torque).is_err());
        let mut unknown = physical(EnableState::Unknown);
        assert!(!request_shutdown(&mut unknown));
    }

    struct ToggleableRtDriver {
        state: Arc<std::sync::Mutex<EnableState>>,
        enable_sent: Arc<std::sync::atomic::AtomicUsize>,
        disable_sent: Arc<std::sync::atomic::AtomicUsize>,
    }

    impl ArmDriver for ToggleableRtDriver {
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
            *self.state.lock().unwrap()
        }
        fn is_enabled(&self) -> bool {
            *self.state.lock().unwrap() == EnableState::Enabled
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn has_driver_fresh_position_api(&self) -> bool {
            true
        }
        fn fresh_joint_positions_into(
            &self,
            joints: &[usize],
            _max_age: Duration,
            out: &mut [f32],
        ) -> bool {
            if out.len() < joints.len() {
                return false;
            }
            out[..joints.len()].fill(0.0);
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 7]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); 7])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            if enable {
                self.enable_sent.fetch_add(1, Ordering::SeqCst);
            } else {
                self.disable_sent.fetch_add(1, Ordering::SeqCst);
            }
            *self.state.lock().unwrap() = if enable {
                EnableState::Enabled
            } else {
                EnableState::Disabled
            };
            Ok(())
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn finalize_shutdown_propagates_refusal_non_zero_and_keeps_torque_off_guarantee() {
        struct AliveGuard(Arc<std::sync::atomic::AtomicBool>);
        impl Drop for AliveGuard {
            fn drop(&mut self) {
                self.0.store(false, Ordering::SeqCst);
            }
        }

        // (a) Executor final commit refused (never confirmed / torqued): must return Err (non-zero
        // exit), log refusal, keep API alive WHILE the owner is handling `CommitShutdown`, AND
        // issue bounded disable through the RT owner before exiting!
        let state = Arc::new(std::sync::Mutex::new(EnableState::Enabled));
        let enable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let disable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let drv = ToggleableRtDriver {
            state: state.clone(),
            enable_sent: enable_sent.clone(),
            disable_sent: disable_sent.clone(),
        };
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let entered_tx = Arc::new(std::sync::Mutex::new(Some(entered_tx)));
        let release_rx = Arc::new(std::sync::Mutex::new(Some(release_rx)));
        let hooks = armd::rt_core::executor::PersistTestHooks {
            on_commit_shutdown: Some(Arc::new(move || {
                if let Some(tx) = entered_tx.lock().unwrap().take() {
                    let _ = tx.send(());
                }
                if let Some(rx) = release_rx.lock().unwrap().take() {
                    let _ = rx.recv_timeout(Duration::from_secs(5));
                }
            })),
            ..Default::default()
        };
        let handle = sup
            .start_executor_with_hooks(
                armd::rt_core::executor::ExecutorConfig::for_tests(Duration::from_millis(5)),
                hooks,
            )
            .await
            .unwrap();
        let server_alive = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let alive_for_task = server_alive.clone();
        let mut server_handle = tokio::spawn(async move {
            let _guard = AliveGuard(alive_for_task);
            std::future::pending::<std::io::Result<()>>().await
        });

        let sup_f = sup.clone();
        let handle_f = handle.clone();
        let fin_task = tokio::spawn(async move {
            let res = finalize_shutdown(&sup_f, &mut server_handle, Some(handle_f), None).await;
            let _ = server_handle.await;
            res
        });
        // Wait until `armd-control` is actively inside `CommitShutdown` and assert the API
        // server has NOT been aborted while commit is in flight.
        tokio::task::spawn_blocking(move || {
            entered_rx.recv_timeout(Duration::from_secs(2)).expect("CommitShutdown entered");
        })
        .await
        .unwrap();
        tokio::task::yield_now().await;
        assert!(
            server_alive.load(Ordering::SeqCst),
            "API server must stay alive while RT owner is handling CommitShutdown"
        );
        let _ = release_tx.send(());

        let err = fin_task
            .await
            .unwrap()
            .expect_err("refused final shutdown commit must return Err for non-zero exit");
        assert!(
            !server_alive.load(Ordering::SeqCst),
            "API server must be aborted once finalization finishes"
        );
        assert!(
            err.contains("shutdown has not been confirmed") || err.contains("refused"),
            "error must report refusal reason: {err}"
        );
        assert!(handle.is_exited(), "executor must have completed bounded teardown");
        assert!(
            disable_sent.load(Ordering::SeqCst) >= 1,
            "refused final shutdown must still issue torque-off through RT owner"
        );
        assert_eq!(*state.lock().unwrap(), EnableState::Disabled);

        // (b) Confirmed disabled executor: finalize_shutdown commits cleanly and returns Ok(()).
        let state_ok = Arc::new(std::sync::Mutex::new(EnableState::Disabled));
        let drv_ok = ToggleableRtDriver {
            state: state_ok,
            enable_sent: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            disable_sent: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let sup_ok = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_ok)));
        sup_ok.set_lease_enabled(false).await;
        sup_ok.disable_torque().await.unwrap();
        let handle_ok = sup_ok
            .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        assert!(sup_ok.request_shutdown().await.unwrap());
        let mut server_ok = tokio::spawn(async {
            std::future::pending::<std::io::Result<()>>().await
        });
        finalize_shutdown(&sup_ok, &mut server_ok, Some(handle_ok.clone()), None)
            .await
            .expect("confirmed shutdown must exit cleanly");
        assert!(handle_ok.is_exited());

        // (c) Legacy final shutdown recheck refused (hardware still enabled): returns Err,
        // stops legacy control loop, AND runs bounded legacy disable to keep torque-off guarantee!
        let state_leg = Arc::new(std::sync::Mutex::new(EnableState::Enabled));
        let disable_leg = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let drv_leg = ToggleableRtDriver {
            state: state_leg.clone(),
            enable_sent: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            disable_sent: disable_leg.clone(),
        };
        let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_leg)));
        sup_leg.set_lease_enabled(false).await;
        sup_leg.enable_torque(sup_leg.capture_admission_stamp()).await.unwrap();
        let legacy_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let mut server_leg = tokio::spawn(async {
            std::future::pending::<std::io::Result<()>>().await
        });
        let err_leg = finalize_shutdown(&sup_leg, &mut server_leg, None, Some(legacy_stop.clone()))
            .await
            .expect_err("refused legacy final shutdown must return Err");
        assert!(err_leg.contains("refused"), "{err_leg}");
        assert!(legacy_stop.load(Ordering::SeqCst));
        assert!(disable_leg.load(Ordering::SeqCst) >= 1);
        assert_eq!(*state_leg.lock().unwrap(), EnableState::Disabled);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn legacy_final_shutdown_commit_blocks_concurrent_clear_fault_and_reenable() {
        // Reproduce the legacy post-check race: arm is E-stopped AND disabled when the final
        // check commits shutdown under `inner.write()`, then releases the lock before stopping
        // the server and control loop. Concurrent `clear_fault()` + `set_torque(true)` must be
        // refused while hardware disable remains permitted.
        let state = Arc::new(std::sync::Mutex::new(EnableState::Disabled));
        let enable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let disable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let drv = ToggleableRtDriver {
            state: state.clone(),
            enable_sent: enable_sent.clone(),
            disable_sent: disable_sent.clone(),
        };
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.inner().write().await.torque_intent = armd::TorqueIntent::Off;
        let _ = sup.trigger_e_stop().await;
        assert!(sup.request_shutdown().await.unwrap());

        let legacy_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let mut server_handle = tokio::spawn(async {
            std::future::pending::<std::io::Result<()>>().await
        });

        let (committed_tx, committed_rx) = tokio::sync::oneshot::channel::<()>();
        let (release_tx, release_rx) = tokio::sync::oneshot::channel::<()>();
        let sup_fin = sup.clone();
        let stop_fin = legacy_stop.clone();
        let fin_task = tokio::spawn(async move {
            finalize_shutdown_with_hook(&sup_fin, &mut server_handle, None, Some(stop_fin), || async move {
                let _ = committed_tx.send(());
                let _ = release_rx.await;
            })
            .await
        });

        // Pause strictly AFTER `inner.commit_shutdown()` dropped `inner.write()` and BEFORE
        // `server_handle.abort()` / `control_stop.store(true)`.
        committed_rx.await.unwrap();
        assert!(!legacy_stop.load(Ordering::SeqCst));

        // Concurrent fault clear and torque enable MUST both be rejected!
        let cf_err = sup
            .clear_fault()
            .await
            .expect_err("clear_fault must be refused once legacy shutdown is committed");
        assert!(cf_err.to_string().contains("shutdown"), "unexpected clear_fault err: {cf_err}");
        let en_err = sup
            .enable_torque(sup.capture_admission_stamp())
            .await
            .expect_err("enable_torque must be refused once legacy shutdown is committed");
        assert!(
            matches!(en_err, armd::supervisor::ArmError::Safety(armd::safety::SafetyError::EStopActive)),
            "unexpected set_torque(true) err: {en_err:?}"
        );
        assert_eq!(
            enable_sent.load(Ordering::SeqCst),
            0,
            "zero torque-enable commands may reach driver after shutdown commit"
        );
        assert_eq!(*state.lock().unwrap(), EnableState::Disabled);

        // Disable access MUST still remain permitted even after shutdown commit.
        let sup_dis = sup.clone();
        let dis_out = tokio::task::spawn_blocking(move || {
            sup_dis.attempt_legacy_panic_disable(Duration::from_millis(200))
        })
        .await
        .unwrap();
        assert_eq!(dis_out, armd::supervisor::PanicDisableOutcome::Confirmed);
        assert!(disable_sent.load(Ordering::SeqCst) >= 1);

        let _ = release_tx.send(());
        fin_task.await.unwrap().expect("committed legacy shutdown exits cleanly");
        assert!(legacy_stop.load(Ordering::SeqCst));
        assert_eq!(enable_sent.load(Ordering::SeqCst), 0);
        assert_eq!(*state.lock().unwrap(), EnableState::Disabled);
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

        // Pre-transport validation rejects unsupported vendor bindings and conflicts.
        let dir = tempfile::tempdir().unwrap();
        let bad_rs = dir.path().join("bad_rs_mode.json");
        let mut val: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(profile).unwrap()).unwrap();
        val["urdf"] = serde_json::json!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/ReBot_Arm_RS.urdf"));
        val["control_mode"] = serde_json::json!([
            {"mode": "position"},
            null, null, null, null, null
        ]);
        std::fs::write(&bad_rs, serde_json::to_string_pretty(&val).unwrap()).unwrap();
        let err = resolve_follower_config("rebot-rs", Some(bad_rs.to_str().unwrap())).unwrap_err();
        assert!(err.contains("does not support control mode 'position'"), "{err}");

        // Cross-vendor profile (Feetech profile passed to rebot-rs) fails before transport open.
        let so101 = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/so_arm101.json");
        let err = resolve_follower_config("rebot-rs", Some(so101)).unwrap_err();
        assert!(err.contains("conflicts with feetech"), "{err}");

        // Blocker 1 regression: migrated B601-DM profile removing `damiao_gripper` and declaring
        // J7 `position_force` solely via `control_mode` MUST be refused before transport opens
        // in Step 1, because `src/driver.rs` still reads `damiao_gripper` and would otherwise
        // silently run J7 in uncapped legacy POS_VEL.
        let dm_profile = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/kits/recamera-b601dm/profile/rebot-b601dm.json"
        );
        let dm_base: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dm_profile).unwrap()).unwrap();

        let mut migrated_j7 = dm_base.clone();
        migrated_j7.as_object_mut().unwrap().remove("damiao_gripper");
        migrated_j7["control_mode"] = serde_json::json!([
            null, null, null, null, null, null,
            {
                "mode": "position_force",
                "limits": {
                    "current_ceiling_raw": 460,
                    "close_speed_rad_s": 0.3,
                    "calibrated": false
                }
            }
        ]);
        let migrated_j7_path = dir.path().join("migrated_j7_only.json");
        std::fs::write(&migrated_j7_path, serde_json::to_string_pretty(&migrated_j7).unwrap())
            .unwrap();
        let err = resolve_follower_config("rebot-dm", Some(migrated_j7_path.to_str().unwrap()))
            .unwrap_err();
        assert!(err.contains("Step-1 driver path"), "{err}");

        // Matching `damiao_gripper` + explicit `control_mode[6]` passes Step-1 parity check.
        let mut parity_j7 = dm_base.clone();
        parity_j7["control_mode"] = migrated_j7["control_mode"].clone();
        let parity_j7_path = dir.path().join("parity_j7.json");
        std::fs::write(&parity_j7_path, serde_json::to_string_pretty(&parity_j7).unwrap())
            .unwrap();
        let ok_parity =
            resolve_follower_config("rebot-dm", Some(parity_j7_path.to_str().unwrap())).unwrap();
        assert_eq!(ok_parity.dof, 7);

        // Major 1 regression: legacy Damiao profile with omitted `control_mode` and lower
        // commissioned `damiao_ranges[0].v_max = 1.5` remains accepted at daemon startup.
        let mut low_vmax = dm_base.clone();
        low_vmax["damiao_ranges"][0]["v_max"] = serde_json::json!(1.5);
        let low_vmax_path = dir.path().join("legacy_low_vmax.json");
        std::fs::write(&low_vmax_path, serde_json::to_string_pretty(&low_vmax).unwrap()).unwrap();
        let ok_low =
            resolve_follower_config("rebot-dm", Some(low_vmax_path.to_str().unwrap())).unwrap();
        assert_eq!(ok_low.dof, 7);

        // Major 6 regression: Damiao profile without `damiao_ranges` declaring explicit POS_VEL
        // velocity 31.0 (above default wire v_max 30.0) fails identically in `validate()` and startup.
        let mut over_vmax = dm_base;
        over_vmax.as_object_mut().unwrap().remove("damiao_ranges");
        over_vmax.as_object_mut().unwrap().remove("torque_guard");
        over_vmax["control_mode"] = serde_json::json!([
            { "mode": "position_velocity", "limits": { "max_velocity_rad_s": 31.0 } },
            null, null, null, null, null, null
        ]);
        let over_vmax_path = dir.path().join("over_vmax_dm.json");
        std::fs::write(&over_vmax_path, serde_json::to_string_pretty(&over_vmax).unwrap()).unwrap();
        let mut parsed_over = ArmConfig::from_profile_file(&over_vmax_path).unwrap();
        let err_val = parsed_over.validate().unwrap_err();
        let err_start =
            resolve_follower_config("rebot-dm", Some(over_vmax_path.to_str().unwrap())).unwrap_err();
        assert_eq!(err_val, err_start);
        assert!(err_start.contains("exceeds Damiao wire v_max"), "{err_start}");
    }

    #[test]
    fn apply_cli_tool_loads_valid_and_rejects_invalid_tool_json() {
        let dir = tempfile::tempdir().unwrap();
        let valid_path = dir.path().join("tool.json");
        std::fs::write(
            &valid_path,
            r#"{
              "id": "soft_finray_gripper",
              "kind": "gripper",
              "robots": ["b601dm"],
              "tcp_offset": [0.0288, 0.0, 0.0],
              "armd": {
                "extra_check_points_end_m": [[0.032, 0.082, 0.022]],
                "dynamics": {"mass_delta_kg": 0.035, "com_moment_delta_nm": 0.018}
              }
            }"#,
        )
        .unwrap();
        let base = ArmConfig::rebot_dm();
        let applied = apply_cli_tool(base.clone(), Some(valid_path.to_str().unwrap())).unwrap();
        assert_eq!(applied.active_tool.id, "soft_finray_gripper");
        assert_eq!(applied.tcp_offset, [0.0288, 0.0, 0.0]);

        let bad_path = dir.path().join("bad_tool.json");
        std::fs::write(
            &bad_path,
            r#"{"id":"bad","kind":"gripper","robots":["b601dm"],"armd":{"unknown_field":1}}"#,
        )
        .unwrap();
        assert!(apply_cli_tool(base.clone(), Some(bad_path.to_str().unwrap())).is_err());

        // Non-stock tools omitting tcp_offset must be rejected even if tool_xml / tcp_site are declared.
        let missing_tcp_tool = dir.path().join("missing_tcp_tool.json");
        std::fs::write(
            &missing_tcp_tool,
            r#"{"id":"need_tcp","kind":"gripper","robots":["b601dm"],"tool_xml":"absent.xml","tcp_site":"finray_tcp"}"#,
        )
        .unwrap();
        let err_missing_tcp = apply_cli_tool(base.clone(), Some(missing_tcp_tool.to_str().unwrap())).unwrap_err();
        assert!(err_missing_tcp.contains("tcp_offset"), "{err_missing_tcp}");

        // M04 (R3): Reserved `id="default"` with custom tool_xml must be refused.
        let default_spoof_tool = dir.path().join("default_spoof_tool.json");
        std::fs::write(
            &default_spoof_tool,
            r#"{"id":"default","kind":"gripper","tool_xml":"default_spoof.xml"}"#,
        )
        .unwrap();
        let err = apply_cli_tool(base, Some(default_spoof_tool.to_str().unwrap())).unwrap_err();
        assert!(err.contains("default"), "{err}");

        let out = run_tool_info(valid_path.to_str().unwrap(), None).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&out).unwrap();
        assert_eq!(parsed["ok"], true);
        assert_eq!(parsed["id"], "soft_finray_gripper");
        assert_eq!(parsed["robot_id"], "b601dm");
        assert_eq!(parsed["tcp_offset_m"], serde_json::json!([0.0288, 0.0, 0.0]));

        // B1: Non-UTF-8 ARMD_TOOL path must not be silently swallowed into stock fallback.
        #[cfg(unix)]
        {
            use std::os::unix::ffi::OsStrExt;
            let base = ArmConfig::rebot_dm();
            let non_utf8_missing = std::ffi::OsStr::from_bytes(b"/nonexistent_\xff/tool.json");
            assert!(apply_cli_tool_with_env(base.clone(), None, Some(non_utf8_missing)).is_err());

            let non_utf8_path = dir.path().join(std::ffi::OsStr::from_bytes(b"tool_\xff.json"));
            std::fs::copy(&valid_path, &non_utf8_path).unwrap();
            let loaded = apply_cli_tool_with_env(base, None, Some(non_utf8_path.as_os_str())).unwrap();
            assert_eq!(loaded.active_tool.id, "soft_finray_gripper");
            assert!(loaded.active_tool.commissioning_required);
        }
    }

    #[test]
    fn cli_owner_flag_and_follower_gate() {
        let dm = ArmConfig::rebot_dm();
        let parsed = parse_cli_args_from(["--follower", "rebot-dm", "--owner", "executor"]).unwrap();
        assert_eq!(parsed.follower.as_deref(), Some("rebot-dm"));
        assert_eq!(parsed.owner.as_deref(), Some("executor"));
        let resolved = armd::rt_core::executor::resolve_startup_owner(
            parsed.owner.as_deref(),
            Some("legacy"),
            Some("legacy"),
            "rebot-dm",
            &dm,
        )
        .unwrap();
        assert_eq!(resolved.effective, armd::rt_core::executor::RtOwner::Executor);
        assert_eq!(resolved.reason, armd::OwnerReason::ExplicitCli);

        // Missing or empty --owner value is rejected.
        assert!(parse_cli_args_from(["--owner"]).is_err());
        assert!(parse_cli_args_from(["--owner", ""]).is_err());

        // Omitted owner defaults to Auto: executor for rebot-dm and mock, legacy for others.
        let default_cli = parse_cli_args_from(["--follower", "rebot-dm"]).unwrap();
        let auto_dm = armd::rt_core::executor::resolve_startup_owner(
            default_cli.owner.as_deref(),
            None,
            None,
            "rebot-dm",
            &dm,
        )
        .unwrap();
        assert_eq!(auto_dm.effective, armd::rt_core::executor::RtOwner::Executor);
        assert_eq!(auto_dm.reason, armd::OwnerReason::AutoQualified);

        let auto_so101 = armd::rt_core::executor::resolve_startup_owner(
            None,
            None,
            None,
            "so-arm101",
            &ArmConfig::default(),
        )
        .unwrap();
        assert_eq!(auto_so101.effective, armd::rt_core::executor::RtOwner::Legacy);
        assert_eq!(auto_so101.reason, armd::OwnerReason::AutoFallbackUnqualifiedFollower);
        assert!(auto_so101.fallback_detail.as_deref().unwrap().contains("so-arm101"));

        // Explicit --owner auto overrides env/config.
        let cli_auto = parse_cli_args_from(["--follower", "rebot-dm", "--owner", "auto"]).unwrap();
        let auto_over_env = armd::rt_core::executor::resolve_startup_owner(
            cli_auto.owner.as_deref(),
            Some("legacy"),
            Some("legacy"),
            "rebot-dm",
            &dm,
        )
        .unwrap();
        assert_eq!(auto_over_env.effective, armd::rt_core::executor::RtOwner::Executor);
        assert_eq!(auto_over_env.reason, armd::OwnerReason::AutoQualified);

        // Unqualified real followers refuse explicit executor with clear error.
        let err = armd::rt_core::executor::validate_startup_owner(
            armd::rt_core::executor::RtOwner::Executor,
            "so-arm101",
        )
        .unwrap_err();
        assert!(err.contains("so-arm101"), "{err}");

        // App config lookup reads arm.armd_owner, falls back to top-level armd_owner, and fails
        // closed on non-string/malformed values (while unrecognized strings are rejected by resolver).
        let dir = tempfile::tempdir().unwrap();
        let cfg_path = dir.path().join("rebot-teleop.config.json");
        std::fs::write(&cfg_path, r#"{"arm":{"armd_owner":"legacy"}}"#).unwrap();
        assert_eq!(read_app_config_armd_owner_from_path(&cfg_path).unwrap().as_deref(), Some("legacy"));
        std::fs::write(&cfg_path, r#"{"armd_owner":"executor","arm":{}}"#).unwrap();
        assert_eq!(read_app_config_armd_owner_from_path(&cfg_path).unwrap().as_deref(), Some("executor"));
        std::fs::write(&cfg_path, r#"{"armd_owner":"executor","arm":{"armd_owner":"auto"}}"#).unwrap();
        assert_eq!(read_app_config_armd_owner_from_path(&cfg_path).unwrap().as_deref(), Some("auto"));
        std::fs::write(&cfg_path, r#"{"arm":{"armd_owner":"legcay"}}"#).unwrap();
        let bad_val = read_app_config_armd_owner_from_path(&cfg_path).unwrap();
        assert_eq!(bad_val.as_deref(), Some("legcay"));
        assert!(armd::rt_core::executor::RequestedRtOwner::resolve(None, None, bad_val.as_deref()).is_err());
        std::fs::write(&cfg_path, r#"{"arm":{"armd_owner":123}}"#).unwrap();
        assert!(read_app_config_armd_owner_from_path(&cfg_path).is_err());
        std::fs::write(&cfg_path, r#"{"arm":"legacy"}"#).unwrap();
        assert!(read_app_config_armd_owner_from_path(&cfg_path).is_err());
        std::fs::write(&cfg_path, r#"["legacy"]"#).unwrap();
        assert!(read_app_config_armd_owner_from_path(&cfg_path).is_err());
        std::fs::write(&cfg_path, r#"{malformed"#).unwrap();
        assert!(read_app_config_armd_owner_from_path(&cfg_path).is_err());

        // Non-UTF-8 ARMD_RT_OWNER fails closed instead of silently falling through to Auto.
        #[cfg(unix)]
        {
            use std::os::unix::ffi::OsStrExt;
            let non_utf8 = std::ffi::OsStr::from_bytes(b"leg\xffacy");
            let err = read_env_rt_owner_os(Some(non_utf8)).unwrap_err();
            assert!(err.contains("ARMD_RT_OWNER"), "{err}");
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn real_follower_executor_startup_failure_falls_back_to_legacy_holding_without_torque_off() {
        // Energized real follower (away from rest) where executor init fails recoverably:
        // must fall back to Legacy owner holding with zero torque-off commands sent!
        let state = Arc::new(std::sync::Mutex::new(EnableState::Enabled));
        let enable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let disable_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let drv = ToggleableRtDriver {
            state: state.clone(),
            enable_sent: enable_sent.clone(),
            disable_sent: disable_sent.clone(),
        };
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let hooks = armd::rt_core::executor::PersistTestHooks {
            fail_init: Some(true),
            ..Default::default()
        };
        let started = start_control_owner_with_hooks(
            &sup,
            "rebot-dm",
            armd::rt_core::executor::RtOwner::Executor,
            Duration::from_millis(10),
            armd::rt_core::sched::RtSchedConfig::default(),
            armd::rt_core::sched::MemoryLock::Skipped("test".into()),
            false,
            false,
            armd::rt_core::executor::PanicPolicy::StopExecutor,
            hooks,
        )
        .await
        .expect("recoverable executor startup failure on rebot-dm must fall back to legacy");

        assert!(started.fell_back_to_legacy);
        assert!(started.executor_handle.is_none());
        let leg_stop = started
            .legacy_control_stop
            .expect("fallback must start a live legacy control loop");
        assert_eq!(sup.owner_mode(), armd::supervisor::OWNER_LEGACY);

        // Wait for at least 3 legacy control ticks to prove the fallback loop is actively holding.
        let wait_deadline = std::time::Instant::now() + Duration::from_secs(2);
        while sup.tick_stats().releases_total < 3 && std::time::Instant::now() < wait_deadline {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let snap = sup.tick_stats();
        assert_eq!(snap.owner_mode, "legacy");
        assert!(snap.loop_active, "fallback legacy loop must be active: {snap:?}");
        assert!(snap.releases_total >= 3, "fallback legacy loop must advance releases: {snap:?}");
        assert_eq!(
            disable_sent.load(Ordering::SeqCst),
            0,
            "fallback to legacy holding must NEVER send torque-off away from rest"
        );
        assert_eq!(*state.lock().unwrap(), EnableState::Enabled);

        // Clean teardown via deliberate process-exit signal.
        sup.signal_legacy_stop(&leg_stop);
        let _ = sup
            .emergency_process_exit_disable_until(std::time::Instant::now() + Duration::from_millis(500))
            .await;

        // Also verify MIT-configured profile triggers recoverable refusal -> fallback to legacy holding.
        let mut mit_cfg = ArmConfig::rebot_dm();
        if let Some(jt) = mit_cfg.joint_tracking.as_mut() {
            jt.enabled = true;
            jt.joints[0].strategy = armd::model::TrackingStrategy::Mit;
            jt.joints[0].kp = 10.0;
            jt.joints[0].kd = 0.5;
            jt.joints[0].effort_limit_nm = 5.0;
            jt.joints[0].hold_kp = Some(10.0);
            jt.joints[0].hold_kd = Some(0.5);
        }
        let state_mit = Arc::new(std::sync::Mutex::new(EnableState::Disabled));
        let disable_mit = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let drv_mit = ToggleableRtDriver {
            state: state_mit,
            enable_sent: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            disable_sent: disable_mit.clone(),
        };
        let sup_mit = Arc::new(ArmSupervisor::new(mit_cfg).with_driver(Box::new(drv_mit)));
        sup_mit.start_dispatcher().unwrap();
        let started_mit = start_control_owner_with_hooks(
            &sup_mit,
            "rebot-dm",
            armd::rt_core::executor::RtOwner::Executor,
            Duration::from_millis(10),
            armd::rt_core::sched::RtSchedConfig::default(),
            armd::rt_core::sched::MemoryLock::Skipped("test".into()),
            false,
            false,
            armd::rt_core::executor::PanicPolicy::StopExecutor,
            armd::rt_core::executor::PersistTestHooks::default(),
        )
        .await
        .expect("MIT refusal on rebot-dm must fall back to legacy owner");
        assert!(started_mit.fell_back_to_legacy);
        assert_eq!(sup_mit.owner_mode(), armd::supervisor::OWNER_LEGACY);
        assert_eq!(disable_mit.load(Ordering::SeqCst), 0);
        if let Some(stop) = started_mit.legacy_control_stop {
            sup_mit.signal_legacy_stop(&stop);
        }

        // Mock follower (no real hardware) still fails fast on executor startup error.
        let sup_mock = Arc::new(ArmSupervisor::new(ArmConfig::default()));
        sup_mock.start_dispatcher().unwrap();
        let mock_err = start_control_owner_with_hooks(
            &sup_mock,
            "mock",
            armd::rt_core::executor::RtOwner::Executor,
            Duration::from_millis(10),
            armd::rt_core::sched::RtSchedConfig::default(),
            armd::rt_core::sched::MemoryLock::Skipped("test".into()),
            false,
            false,
            armd::rt_core::executor::PanicPolicy::StopExecutor,
            armd::rt_core::executor::PersistTestHooks {
                fail_init: Some(true),
                ..Default::default()
            },
        )
        .await
        .err()
        .expect("mock executor startup failure must fail fast");
        assert!(mock_err.contains("Failed to start RT executor"), "{mock_err}");
    }

    #[test]
    fn cli_cell_args_parse_and_resolve_precedence() {
        let cli = parse_cli_args_from(["--cell", "cell.json", "--cell-host", "recamera_left"]).unwrap();
        assert_eq!(cli.cell.as_deref(), Some("cell.json"));
        assert_eq!(cli.cell_host.as_deref(), Some("recamera_left"));

        assert!(parse_cli_args_from(["--cell"]).is_err());
        assert!(parse_cli_args_from(["--cell", "   "]).is_err());
        assert!(parse_cli_args_from(["--cell-host"]).is_err());
        assert!(parse_cli_args_from(["--cell-host", "  "]).is_err());

        // Without any cell config, returns Ok(None) (zero overhead default).
        let cfg = ArmConfig::rebot_dm();
        assert!(resolve_cell_context_with_env(&CliArgs::default(), &cfg, None, "mock", None, 8765, None, None)
            .unwrap()
            .is_none());

        // Orphan cell_host without cell path fails closed.
        assert!(resolve_cell_context_with_env(
            &CliArgs {
                cell_host: Some("recamera_left".into()),
                ..Default::default()
            },
            &cfg,
            None,
            "mock",
            None,
            8765,
            None,
            None
        )
        .is_err());
    }
}
