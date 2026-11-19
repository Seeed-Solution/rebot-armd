//! Calibration sessions: the raw data of a camera or fine calibration (board, intrinsics,
//! views with markers and joint angles, plan, solve summary, gates, levels) kept on the device
//! so a solve can be repeated later without capturing again.
//!
//! `POST /api/v1/calibration/sessions` stores one session (JSON object, at most
//! [`MAX_BODY_BYTES`]; `id` given = overwrite that session, else a new ULID), `GET` lists them
//! (newest first: id, kind, created_at, stored_at, views, size, summary), `GET .../<id>` returns
//! one, `DELETE .../<id>` removes one. No lease: the files only hold measurements, nothing is
//! applied. Files: `<dir>/<id>.json` (the body with `id` and `stored_at`) and
//! `<dir>/<id>.meta.json` (the list entry), written with temp + rename; only the newest
//! [`KEEP`] sessions are kept. `dir` is `ARMD_CALIBRATION_SESSIONS`, default
//! `calibration-sessions/` next to `calibration.json` (`/userdata/rebot-teleop/calibration-sessions`).

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use axum::extract::Path as UrlPath;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde_json::{json, Map, Value};

use crate::calibration::CalibrationDeps;

/// Largest session body (bytes); a larger POST is refused with 413.
pub const MAX_BODY_BYTES: usize = 8 * 1024 * 1024;
/// Sessions kept; older ones are deleted after a store.
pub const KEEP: usize = 10;
/// Session kinds.
pub const KINDS: [&str; 2] = ["camera", "fine"];

/// One store / prune at a time (all requests of this process).
static LOCK: Mutex<()> = Mutex::new(());

fn valid_id(id: &str) -> bool { crate::recording::valid_episode_id(id) }

fn err(status: StatusCode, code: &str, message: impl Into<String>) -> Response {
    (status, Json(json!({"error": code, "code": code, "message": message.into()}))).into_response()
}

fn now_ms() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64)
}

fn now_rfc3339() -> String { chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true) }

fn data_path(dir: &Path, id: &str) -> PathBuf { dir.join(format!("{id}.json")) }
fn meta_path(dir: &Path, id: &str) -> PathBuf { dir.join(format!("{id}.meta.json")) }

/// List entries in `dir`, newest (`stored_seq`) first.
fn read_metas(dir: &Path) -> Vec<Value> {
    let mut out: Vec<Value> = std::fs::read_dir(dir).into_iter().flatten().flatten().filter_map(|e| {
        let name = e.file_name().to_string_lossy().into_owned();
        let id = name.strip_suffix(".meta.json")?;
        if !valid_id(id) { return None; }
        let v: Value = serde_json::from_slice(&std::fs::read(e.path()).ok()?).ok()?;
        (v.get("id").and_then(Value::as_str) == Some(id)).then_some(v)
    }).collect();
    out.sort_by_key(|m| std::cmp::Reverse(m.get("stored_seq").and_then(Value::as_u64).unwrap_or(0)));
    out
}

/// Validate and store `body`; returns the list entry. Blocking (file IO).
pub fn store(dir: &Path, mut body: Map<String, Value>, size: usize) -> Result<Value, (StatusCode, &'static str, String)> {
    let bad = |m: String| (StatusCode::BAD_REQUEST, "INVALID", m);
    let kind = body.get("kind").and_then(Value::as_str).unwrap_or("").to_string();
    if !KINDS.contains(&kind.as_str()) { return Err(bad(format!("kind must be one of {KINDS:?}"))); }
    let views = body.get("views").and_then(Value::as_array).map(Vec::len).ok_or_else(|| bad("views must be an array".into()))?;
    let id = match body.get("id") {
        None | Some(Value::Null) => crate::recording::new_ulid(),
        Some(Value::String(s)) if valid_id(s) => s.clone(),
        Some(_) => return Err(bad("id must match ^[A-Za-z0-9_-]{1,64}$".into())),
    };
    let created_at = match body.get("created_at") {
        Some(Value::String(s)) if !s.is_empty() && s.len() <= 64 => s.clone(),
        None | Some(Value::Null) => now_rfc3339(),
        Some(_) => return Err(bad("created_at must be a string (RFC 3339)".into())),
    };
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let metas = read_metas(dir);
    // strictly increasing store order, also within one millisecond
    let seq = metas.iter().filter_map(|m| m.get("stored_seq").and_then(Value::as_u64)).max().map_or(now_ms(), |m| now_ms().max(m + 1));
    let stored_at = now_rfc3339();
    body.insert("id".into(), json!(id));
    body.insert("created_at".into(), json!(created_at));
    body.insert("stored_at".into(), json!(stored_at));
    let text = serde_json::to_string(&Value::Object(body.clone())).map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "INTERNAL", e.to_string()))?;
    let summary = body.get("summary").cloned().unwrap_or(Value::Null);
    let meta = json!({"id": id, "kind": kind, "created_at": created_at, "stored_at": stored_at, "stored_seq": seq,
        "views": views, "size": text.len().max(size), "summary": summary});
    let io = |e: std::io::Error| (StatusCode::INTERNAL_SERVER_ERROR, "IO", format!("{}: {e}", dir.display()));
    crate::persist::write_atomic(&data_path(dir, &id), &text).map_err(io)?;
    crate::persist::write_atomic(&meta_path(dir, &id), &meta.to_string()).map_err(io)?;
    // keep the newest KEEP (the one just written has the highest seq)
    for old in read_metas(dir).iter().skip(KEEP) {
        if let Some(old_id) = old.get("id").and_then(Value::as_str) {
            let _ = std::fs::remove_file(data_path(dir, old_id));
            let _ = std::fs::remove_file(meta_path(dir, old_id));
        }
    }
    Ok(meta)
}

/// Session list (newest first), the `stored_seq` ordering key left out.
pub fn list(dir: &Path) -> Vec<Value> {
    read_metas(dir).into_iter().map(|mut m| { if let Some(o) = m.as_object_mut() { o.remove("stored_seq"); } m }).collect()
}

/// `POST /api/v1/calibration/sessions` (body limit [`MAX_BODY_BYTES`]).
pub async fn post_session(axum::Extension(deps): axum::Extension<CalibrationDeps>, body: axum::body::Bytes) -> Response {
    let size = body.len();
    let value: Value = match serde_json::from_slice(&body) {
        Ok(v) => v,
        Err(e) => return err(StatusCode::BAD_REQUEST, "INVALID", format!("body is not JSON: {e}")),
    };
    let Value::Object(map) = value else { return err(StatusCode::BAD_REQUEST, "INVALID", "body must be a JSON object") };
    let dir = deps.sessions_dir.clone();
    match tokio::task::spawn_blocking(move || store(&dir, map, size)).await {
        Ok(Ok(mut meta)) => { if let Some(o) = meta.as_object_mut() { o.remove("stored_seq"); } Json(meta).into_response() }
        Ok(Err((s, c, m))) => err(s, c, m),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, "INTERNAL", e.to_string()),
    }
}

/// `GET /api/v1/calibration/sessions`: `{sessions: [...], keep, max_bytes}`.
pub async fn get_sessions(axum::Extension(deps): axum::Extension<CalibrationDeps>) -> Response {
    let dir = deps.sessions_dir.clone();
    let sessions = tokio::task::spawn_blocking(move || list(&dir)).await.unwrap_or_default();
    Json(json!({"sessions": sessions, "keep": KEEP, "max_bytes": MAX_BODY_BYTES})).into_response()
}

/// `GET /api/v1/calibration/sessions/<id>`: the stored session.
pub async fn get_session(axum::Extension(deps): axum::Extension<CalibrationDeps>, UrlPath(id): UrlPath<String>) -> Response {
    if !valid_id(&id) { return err(StatusCode::BAD_REQUEST, "INVALID", "invalid session id"); }
    let path = data_path(&deps.sessions_dir, &id);
    match tokio::task::spawn_blocking(move || std::fs::read(path)).await {
        Ok(Ok(bytes)) => ([(axum::http::header::CONTENT_TYPE, "application/json")], bytes).into_response(),
        Ok(Err(e)) if e.kind() == std::io::ErrorKind::NotFound => err(StatusCode::NOT_FOUND, "NOT_FOUND", format!("no session {id}")),
        Ok(Err(e)) => err(StatusCode::INTERNAL_SERVER_ERROR, "IO", e.to_string()),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, "INTERNAL", e.to_string()),
    }
}

/// `DELETE /api/v1/calibration/sessions/<id>`: `{deleted: id}`.
pub async fn delete_session(axum::Extension(deps): axum::Extension<CalibrationDeps>, UrlPath(id): UrlPath<String>) -> Response {
    if !valid_id(&id) { return err(StatusCode::BAD_REQUEST, "INVALID", "invalid session id"); }
    let dir = deps.sessions_dir.clone();
    let id2 = id.clone();
    let found = tokio::task::spawn_blocking(move || {
        let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let a = std::fs::remove_file(data_path(&dir, &id2)).is_ok();
        let b = std::fs::remove_file(meta_path(&dir, &id2)).is_ok();
        a || b
    }).await.unwrap_or(false);
    if found { Json(json!({"deleted": id})).into_response() } else { err(StatusCode::NOT_FOUND, "NOT_FOUND", format!("no session {id}")) }
}
