//! Refusal replies (`command_reject`, `preset_reject`, `servo_reject`) are logged once per code
//! and connection, then at most every 5 s with a suppressed count; STOPPED is DEBUG, the rest WARN.
//! The reply bytes the client sees are not changed (the ws_replay corpus covers the rest).

mod common;

use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use tokio_tungstenite::connect_async;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::{Context, SubscriberExt};
use tracing_subscriber::Layer;

use armd::api::ws::refusal_log::{RefusalLog, REFUSAL_LOG_INTERVAL, REFUSAL_LOG_TARGET};
use armd::ArmConfig;
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::tungstenite::Message;

#[derive(Debug, Clone)]
struct Line {
    level: Level,
    fields: std::collections::HashMap<String, String>,
}

impl Line {
    fn f(&self, k: &str) -> &str {
        self.fields.get(k).map(String::as_str).unwrap_or("")
    }
}

#[derive(Default)]
struct Fields(std::collections::HashMap<String, String>);
impl Visit for Fields {
    fn record_debug(&mut self, f: &Field, v: &dyn std::fmt::Debug) {
        self.0.insert(f.name().to_owned(), format!("{v:?}").trim_matches('"').to_owned());
    }
    fn record_str(&mut self, f: &Field, v: &str) {
        self.0.insert(f.name().to_owned(), v.to_owned());
    }
}

struct Capture(Arc<Mutex<Vec<Line>>>);
impl<S: Subscriber> Layer<S> for Capture {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        if event.metadata().target() != REFUSAL_LOG_TARGET {
            return;
        }
        let mut v = Fields::default();
        event.record(&mut v);
        self.0.lock().unwrap().push(Line { level: *event.metadata().level(), fields: v.0 });
    }
}

fn lines() -> &'static Arc<Mutex<Vec<Line>>> {
    static LINES: OnceLock<Arc<Mutex<Vec<Line>>>> = OnceLock::new();
    LINES.get_or_init(|| {
        let store = Arc::new(Mutex::new(Vec::new()));
        let sub = tracing_subscriber::registry().with(Capture(store.clone()).with_filter(tracing_subscriber::filter::LevelFilter::DEBUG));
        tracing::subscriber::set_global_default(sub).expect("global subscriber set once per test process");
        store
    })
}

fn lines_for(connection: &str) -> Vec<Line> {
    lines().lock().unwrap().iter().filter(|l| l.f("connection") == connection).cloned().collect()
}

fn reject(kind: &str, code: &str, request_id: Option<u64>) -> Value {
    let mut v = json!({"type":kind, "status":409, "code":code, "error":code, "message":"m"});
    if let Some(id) = request_id {
        v["request_id"] = json!(id);
    }
    v
}

#[test]
fn first_refusal_logs_then_one_line_per_code_per_interval_with_suppressed_count() {
    let t0 = Instant::now();
    let mut log = RefusalLog::default();
    let r = reject("command_reject", "NOT_ARMED", Some(5));
    let first = log.observe(t0, &r).expect("first occurrence is logged at once");
    assert_eq!((first.kind.as_str(), first.code.as_str(), first.status, first.request_id, first.suppressed), ("command_reject", "NOT_ARMED", Some(409), Some(5), 0));
    for i in 1..=40u64 {
        assert!(log.observe(t0 + Duration::from_millis(100 * i), &r).is_none(), "inside the interval");
    }
    // another code on the same connection is independent
    let other = log.observe(t0 + Duration::from_millis(50), &reject("preset_reject", "IK_UNREACHABLE", None)).expect("separate code");
    assert_eq!((other.code.as_str(), other.request_id, other.suppressed), ("IK_UNREACHABLE", None, 0));
    // rollover: exactly at the interval the next line carries the 40 suppressed ones
    let again = log.observe(t0 + REFUSAL_LOG_INTERVAL, &r).expect("rollover");
    assert_eq!(again.suppressed, 40);
    assert!(log.observe(t0 + REFUSAL_LOG_INTERVAL + Duration::from_secs(1), &r).is_none());
    let third = log.observe(t0 + REFUSAL_LOG_INTERVAL * 2, &r).expect("second rollover");
    assert_eq!(third.suppressed, 1);
}

#[test]
fn only_refusals_are_logged_and_request_id_must_be_numeric() {
    let mut log = RefusalLog::default();
    let now = Instant::now();
    assert!(log.observe(now, &json!({"type":"command_ack", "request_id":1, "status":"accepted"})).is_none());
    assert!(log.observe(now, &json!({"type":"epoch_sync", "epoch_id":3})).is_none());
    assert!(log.observe(now, &json!({"status":409})).is_none());
    let l = log.observe(now, &reject("command_reject", "BAD", None).as_object().cloned().map(|mut m| { m.insert("request_id".into(), json!("abc")); Value::Object(m) }).unwrap()).unwrap();
    assert_eq!(l.request_id, None, "a non-numeric request_id is not copied into the log");
    let servo = log.observe(now, &json!({"type":"servo_reject", "reason":"missing_generation"})).unwrap();
    assert_eq!((servo.kind.as_str(), servo.code.as_str(), servo.status), ("servo_reject", "missing_generation", None));
}

#[test]
fn stopped_is_debug_everything_else_warn_and_connections_are_independent() {
    lines();
    let now = Instant::now();
    let mut log_a = RefusalLog::default();
    let mut log_b = RefusalLog::default();
    for (conn, log) in [("connection:t-a", &mut log_a), ("connection:t-b", &mut log_b)] {
        log.note_at(conn, &reject("command_reject", "STOPPED", Some(1)), now);
        log.note_at(conn, &reject("command_reject", "LEASE_LOST", Some(2)), now);
        log.note_at(conn, &reject("command_reject", "LEASE_LOST", Some(3)), now + Duration::from_secs(1));
    }
    for conn in ["connection:t-a", "connection:t-b"] {
        let l = lines_for(conn);
        assert_eq!(l.len(), 2, "{conn}: one line per code, the repeat is suppressed: {l:?}");
        let stopped = l.iter().find(|x| x.f("code") == "STOPPED").unwrap();
        let lost = l.iter().find(|x| x.f("code") == "LEASE_LOST").unwrap();
        assert_eq!(stopped.level, Level::DEBUG);
        assert_eq!(lost.level, Level::WARN);
        assert_eq!((lost.f("reply_type"), lost.f("status"), lost.f("request_id"), lost.f("suppressed")), ("command_reject", "409", "2", "0"));
    }
    log_a.note_at("connection:t-a", &reject("command_reject", "LEASE_LOST", Some(9)), now + REFUSAL_LOG_INTERVAL);
    let a = lines_for("connection:t-a");
    assert_eq!(a.last().unwrap().f("suppressed"), "1");
    assert_eq!(a.last().unwrap().f("request_id"), "9");
}

/// A refused command over a real socket: ordinary writer branch (home) and urgent branch (stop)
/// both log, and the reply the client reads matches the exact expected JSON.
#[tokio::test]
async fn refused_commands_over_websocket_are_logged_on_the_ordinary_and_urgent_paths() {
    lines();
    let (addr, _sup) = common::spawn_server(ArmConfig::default()).await;
    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let ordinary = common::ws_reply(&mut ws, json!({"type":"command", "request_id":7001, "generation":0, "epoch_id":0, "path":"/api/v1/motion/home", "body":{}}), &["command_reject", "command_ack"]).await;
    // a second client: the same code is logged again for it (state is per connection)
    let (mut ws2, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let urgent = common::ws_reply(&mut ws2, json!({"type":"command", "request_id":7002, "generation":0, "epoch_id":0, "path":"/api/v1/motion/stop", "body":{}}), &["command_reject", "command_ack"]).await;
    assert_eq!(
        ordinary,
        json!({
            "type": "command_reject",
            "request_id": 7001,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not own an active lease",
            "epoch_id": 1,
        })
    );
    assert_eq!(
        urgent,
        json!({
            "type": "command_reject",
            "request_id": 7002,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not own an active lease",
            "epoch_id": 1,
        })
    );
    let repeat = common::ws_reply(&mut ws, json!({"type":"command", "request_id":7003, "generation":0, "epoch_id":0, "path":"/api/v1/motion/home", "body":{}}), &["command_reject", "command_ack"]).await;
    assert_eq!(
        repeat,
        json!({
            "type": "command_reject",
            "request_id": 7003,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not own an active lease",
            "epoch_id": 1,
        })
    );
    // the writer logs before it writes, so both lines exist once both replies were read
    let all = lines().lock().unwrap().clone();
    for (id, reply) in [("7001", &ordinary), ("7002", &urgent)] {
        let l = all.iter().find(|l| l.f("request_id") == id).unwrap_or_else(|| panic!("no log line for request {id}: {all:?}"));
        assert_eq!(l.f("code"), reply["code"].as_str().unwrap());
        assert_eq!(l.f("reply_type"), "command_reject");
        assert_eq!(l.f("status"), reply["status"].as_u64().unwrap().to_string());
    }
    assert!(!all.iter().any(|l| l.f("request_id") == "7003"), "a repeat of the code inside 5 s is suppressed: {all:?}");
    assert_ne!(all.iter().find(|l| l.f("request_id") == "7001").unwrap().f("connection"), all.iter().find(|l| l.f("request_id") == "7002").unwrap().f("connection"));
}

struct WriteHookGuard(String);
impl Drop for WriteHookGuard {
    fn drop(&mut self) {
        armd::api::ws::set_test_ws_write_paused(&self.0, false);
        armd::api::ws::set_test_ws_write_timeout(&self.0, None);
    }
}

/// Force a rejected Stop through `flush_urgent_then_fatal_and_close` using the paused-writer
/// fixture, and verify the refusal log line, refusal frame, fatal error frame, and Close order.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rejected_stop_through_fatal_drain_is_logged_before_fatal_and_close() {
    lines();
    let (addr, sup) = common::spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::default() }).await;
    let (mut ws, gen) = common::controller(addr).await;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard = WriteHookGuard(conn_id.clone());
    let ep = sup.get_observation().await.epoch_id;

    armd::api::ws::set_test_ws_write_timeout(&conn_id, Some(Duration::from_millis(200)));
    armd::api::ws::set_test_ws_write_paused(&conn_id, true);
    ws.send(Message::Text(
        json!({"type": "command", "request_id": 7900, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {}}).to_string().into(),
    )).await.unwrap();
    let wait_deadline = Instant::now() + Duration::from_secs(1);
    while armd::api::ws::test_ws_writer_blocked_count(&conn_id) < 1 && Instant::now() < wait_deadline {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1, "writer must block on paused frame");

    // Urgent Stop with stale generation -> command_reject queued on stop_out_rx while writer is
    // blocked; when the 200 ms write timeout fires, `flush_urgent_then_fatal_and_close` drains it.
    ws.send(Message::Text(
        json!({"type": "command", "request_id": 7901, "generation": gen + 99, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}}).to_string().into(),
    )).await.unwrap();
    tokio::time::sleep(Duration::from_millis(215)).await;

    armd::api::ws::set_test_ws_write_paused(&conn_id, false);
    let mut stop_reject: Option<(usize, Value)> = None;
    let mut fatal_idx: Option<usize> = None;
    let mut close_idx: Option<usize> = None;
    let mut idx = 0usize;
    let mut seen = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_secs(1), ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                let v: Value = serde_json::from_str(&text).unwrap();
                seen.push(format!("{idx}:Text({}:{})", v["type"], v["code"]));
                if v["type"] == "command_reject" && v["request_id"] == 7901 && stop_reject.is_none() {
                    stop_reject = Some((idx, v.clone()));
                }
                if v["type"] == "error" && v["code"] == "WRITE_TIMEOUT" && fatal_idx.is_none() {
                    fatal_idx = Some(idx);
                }
                idx += 1;
            }
            Ok(Some(Ok(Message::Close(cf)))) => {
                seen.push(format!("{idx}:Close({cf:?})"));
                close_idx = Some(idx);
                break;
            }
            Ok(other) => {
                seen.push(format!("{idx}:Other({other:?})"));
                break;
            }
            Err(_) => {
                seen.push("TIMEOUT".into());
                break;
            }
        }
    }
    let (si, reject_val) = stop_reject.unwrap_or_else(|| panic!("rejected Stop must be delivered through fatal drain; seen={seen:?}"));
    let fi = fatal_idx.unwrap_or_else(|| panic!("fatal WRITE_TIMEOUT error frame must be delivered; seen={seen:?}"));
    let ci = close_idx.unwrap_or_else(|| panic!("Close frame must follow fatal error frame; seen={seen:?}"));
    assert!(si < fi && fi < ci, "expected stop_reject({si}) < fatal({fi}) < close({ci})");

    let all = lines().lock().unwrap().clone();
    let l = all.iter().find(|l| l.f("request_id") == "7901")
        .unwrap_or_else(|| panic!("no refusal log line for drained Stop 7901: {all:?}"));
    assert_eq!(l.f("reply_type"), "command_reject");
    assert_eq!(l.f("code"), reject_val["code"].as_str().unwrap());
    assert_eq!(l.f("status"), reject_val["status"].as_u64().unwrap().to_string());
}
