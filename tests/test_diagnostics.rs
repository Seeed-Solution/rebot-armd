//! Diagnostics routes and the black box fed by the supervisor's telemetry.

use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::api::{create_router_full, proxy::DetectorUpstream};
use armd::diagnostics::{blackbox, boot, DiagnosticsState};
use armd::{ArmConfig, ArmSupervisor};

async fn serve(supervisor: Arc<ArmSupervisor>, state: DiagnosticsState) -> std::net::SocketAddr {
    let grasp = armd::grasp::GraspDeps::http(DetectorUpstream::from_env(), None);
    let router = create_router_full(supervisor, DetectorUpstream::from_env(), grasp, state);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
    addr
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn diagnostics_routes_serve_boots_and_blackbox_records() {
    let dir = tempfile::tempdir().unwrap();
    let sys = boot::SystemInfo { uptime_s: Some(30.0), kernel_boot_id: Some("k".into()), reset_reason: None };
    // A previous start that left records, then this start.
    let first = boot::record_boot(dir.path(), &sys).unwrap();
    std::fs::write(dir.path().join(blackbox::BB_FILE), format!("{{\"boot\":{},\"t_ms\":900,\"is_torqued\":true}}\n", first.boot)).unwrap();
    let rec = boot::record_boot(dir.path(), &sys).unwrap();
    assert_eq!(rec.entry["prev_last"]["t_ms"], 900);

    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let start = Instant::now();
    blackbox::spawn_recorder(&supervisor, blackbox::open(dir.path(), rec.boot, start).unwrap());
    for _ in 0..5 {
        supervisor.tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    tokio::time::sleep(Duration::from_millis(100)).await;

    let state = DiagnosticsState { blackbox_dir: Some(dir.path().to_path_buf()), boot: rec.boot };
    let addr = serve(supervisor, state).await;
    let get = |path: &str| {
        let url = format!("http://{addr}{path}");
        async move { reqwest::get(url).await.unwrap().json::<serde_json::Value>().await.unwrap() }
    };
    let boots = get("/api/v1/diagnostics/boots").await;
    assert_eq!(boots["enabled"], true);
    let list = boots["boots"].as_array().unwrap();
    assert_eq!(list.len(), 2);
    assert_eq!(list[1]["prev_last"]["is_torqued"], true);

    let cur = get("/api/v1/diagnostics/blackbox?seconds=60").await;
    assert_eq!(cur["boot"], rec.boot);
    let records = cur["records"].as_array().unwrap();
    assert_eq!(records.len(), 1, "idle: one sample per 5 s, {records:?}");
    assert_eq!(records[0]["is_torqued"], false);
    assert!(records[0]["positions"].as_array().unwrap().len() == 7);

    let prev = get("/api/v1/diagnostics/blackbox?seconds=5&prev=1").await;
    assert_eq!(prev["boot"], first.boot);
    assert_eq!(prev["records"][0]["t_ms"], 900);

    let schema = get("/api/v1/observation/schema").await;
    assert_eq!(schema["capabilities"]["diagnostics"], true);
}

#[tokio::test]
async fn disabled_diagnostics_answer_with_empty_lists() {
    let addr = serve(Arc::new(ArmSupervisor::new(ArmConfig::default())), DiagnosticsState::default()).await;
    let v: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/blackbox")).await.unwrap().json().await.unwrap();
    assert_eq!(v["enabled"], false);
    assert!(v["records"].as_array().unwrap().is_empty());
    let v: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/boots")).await.unwrap().json().await.unwrap();
    assert_eq!(v["enabled"], false);
}

struct Slow;
impl blackbox::Sink for Slow {
    fn write_line(&mut self, _: &[u8], _: bool) -> std::io::Result<()> {
        std::thread::sleep(Duration::from_millis(500));
        Ok(())
    }
}

/// The control tick publishes into a broadcast; a stalled black-box writer only drops samples.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stalled_writer_does_not_slow_the_tick() {
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let bb = blackbox::BlackBox::start(Slow, 1, Instant::now()).unwrap();
    blackbox::spawn_recorder(&supervisor, bb);
    let mut worst = Duration::ZERO;
    for _ in 0..100 {
        let t = Instant::now();
        supervisor.tick(0.02).await;
        worst = worst.max(t.elapsed());
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(worst < Duration::from_millis(100), "tick took {worst:?}");
}

fn thread_cpu() -> Duration {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: valid out-pointer for the calling thread's CPU clock.
    unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
    Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
}

/// CPU cost of the black box per second of a torqued arm: 50 observations taken from the
/// broadcast (clone + sampler), 5 lines formatted, written and fdatasync'ed.
/// `cargo test --release --test test_diagnostics -- --ignored --nocapture`
#[test]
#[ignore]
fn blackbox_cpu_cost() {
    use armd::diagnostics::{blackbox::Sink, rotating::RotatingFile};
    let obs = ArmSupervisor::new(ArmConfig::rebot_dm());
    let rt = tokio::runtime::Runtime::new().unwrap();
    let mut o = rt.block_on(obs.get_observation());
    o.is_torqued = true;
    const N: u32 = 5000;
    let t = thread_cpu();
    let mut sampler = blackbox::Sampler::default();
    let start = Instant::now();
    let mut due = 0;
    for i in 0..N {
        let c = std::hint::black_box(o.clone());
        if sampler.due(start + Duration::from_millis(u64::from(i) * 20), blackbox::is_active(&c)) {
            due += 1;
        }
    }
    let per_obs = (thread_cpu() - t) / N;
    let t = thread_cpu();
    let mut bytes = 0;
    for i in 0..N {
        bytes += std::hint::black_box(blackbox::record_line(1, u64::from(i), 0, &o)).len();
    }
    let per_fmt = (thread_cpu() - t) / N;
    let dir = tempfile::tempdir().unwrap();
    let mut f = RotatingFile::open(dir.path().join(blackbox::BB_FILE), blackbox::BB_MAX_BYTES, blackbox::BB_KEEP).unwrap();
    let line = blackbox::record_line(1, 0, 0, &o);
    let (t, wall) = (thread_cpu(), Instant::now());
    for _ in 0..500 {
        f.write_line(line.as_bytes(), true).unwrap();
    }
    let per_sync_write = (thread_cpu() - t) / 500;
    let per_sync_wall = wall.elapsed() / 500;
    let t = thread_cpu();
    for _ in 0..500 {
        f.write_line(line.as_bytes(), false).unwrap();
    }
    let per_write = (thread_cpu() - t) / 500;
    let torqued = per_obs * 50 + (per_fmt + per_sync_write) * 5;
    let idle = per_obs * 50 + (per_fmt + per_write) / 5;
    println!("due {due}/{N}; line {} B (avg {} B)", line.len(), bytes / N as usize);
    println!("per observation (clone + sampler): {per_obs:?}; per line format: {per_fmt:?}");
    println!("per line write+fdatasync: cpu {per_sync_write:?}, wall {per_sync_wall:?}; write only: {per_write:?}");
    println!("CPU per second: torqued {torqued:?} ({:.4} %), idle {idle:?} ({:.4} %)", torqued.as_secs_f64() * 100.0, idle.as_secs_f64() * 100.0);
}
