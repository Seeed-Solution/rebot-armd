use std::time::{Duration, Instant};

use serde_json::Value;

use crate::api::proxy::DetectorUpstream;

use super::frames::{DepthFrameInfo, DetectionFrame, MaskFocus, ModeInfo, VisionMode};
use super::gray::{
    is_octet_stream, parse_gray_frame, valid_instance_token, GrayFrame, GrayFrameError,
    GrayTriggerResult,
};
use super::tracking::{parse_detections, parse_markers};
use crate::grasp::{BoxFuture, CamModel};

/// The mode a grasp needs: YOLO, plus ArUco when it was running.
pub fn grasp_mode(current: &str) -> &'static str {
    match current {
        "aruco" | "both" => "both",
        _ => "yolo",
    }
}

/// Pipelines a grasp triggers per centring step. A marker target would add `"aruco"`.
pub const GRASP_PIPELINES: &[&str] = &["yolo"];
/// How long one centring trigger waits for a fresh frame.
pub(crate) const TRIGGER_WAIT_MS: u64 = 1000;

/// Where the orchestrator gets detections; tests inject a simulated camera.
pub trait DetectionSource: Send + Sync {
    /// The provider runs the pipelines a trigger names (perception API 2). The grasp then
    /// triggers [`GRASP_PIPELINES`] per centring step and leaves the vision mode alone.
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// Whether depth inference is available on the provider (`GET /api/mode` available.depth).
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// One inference running exactly `pipelines` on a frame newer than the call, waiting up to
    /// `wait_ms`. `Ok(None)` = no fresh frame in time.
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger_with_focus(pipelines, wait_ms, None)
    }
    /// Whether the provider runs the ArUco pipeline (`GET /api/mode` available.aruco).
    fn aruco_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// Whether raw gray frames are available (`GET /api/mode` available.gray, loopback only).
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// [`DetectionSource::trigger`] that also returns the ArUco markers of the frame (pixels of
    /// the frame's `image_size_px`) when `pipelines` names `"aruco"`. Default: no markers.
    fn trigger_markers(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<(DetectionFrame, Vec<crate::mview_stereo::Marker>)>, String>> {
        Box::pin(async move { Ok(self.trigger(pipelines, wait_ms).await?.map(|f| (f, Vec::new()))) })
    }
    /// Trigger returning detections, optional markers, and gray frame identity (`instance_id`,
    /// `pipelines.gray.ran`). Default delegates to [`Self::trigger_markers`] without gray.
    fn trigger_gray(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            Ok(self.trigger_markers(pipelines, wait_ms).await?.map(|(frame, markers)| GrayTriggerResult {
                frame, markers, instance_id: None, gray_ran: false, gray_ms: None,
            }))
        })
    }
    /// Fetch the exact raw gray frame `(instance_id, frame_id)` from `GET /api/frame/gray`.
    fn gray_frame(&self, _instance_id: &str, _frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        Box::pin(async { Err(GrayFrameError::Protocol("gray frames not supported by this detection source".to_string())) })
    }
    /// One inference with optional mask focus.
    fn trigger_with_focus(&self, _pipelines: &'static [&'static str], _wait_ms: u64, _focus: Option<&MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Err("per-request pipelines not supported".to_string()) })
    }
    /// Sets mask focus for subsequent frames until expiry or clears it (`ttl_ms` = 0).
    fn set_mask_focus(&self, _label: Option<String>, _box_norm: [f64; 4], _ttl_ms: u64) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>>;
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>>;
    /// The provider's vision mode; `Ok(None)` when it has no mode API (older detectors),
    /// in which case the grasp only switches inference on and off.
    fn vision_mode(&self) -> BoxFuture<'_, Result<Option<VisionMode>, String>> {
        Box::pin(async { Ok(None) })
    }
    /// Sets mode and continuous inference in one request.
    fn set_vision_mode(&self, _mode: &VisionMode) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Err("vision mode not supported".to_string()) })
    }
    /// `POST /api/models` with `body`: the reply's status and JSON body (`Err` = unreachable or
    /// timed out). Default: a provider without the route (404).
    fn set_models(&self, _body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        Box::pin(async { Ok((404, Value::Null)) })
    }
    /// Everything a grasp reads from the provider's mode, once per grasp (one `GET /api/mode`
    /// for the HTTP source). `Err` = provider unreachable.
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(ModeInfo::compose(self))
    }
    /// The next frame with `frame_id > after` (any frame when `after` is None), waiting up to
    /// `wait_ms`. `Ok(None)` = no new frame in time.
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>>;
    /// The current camera image as JPEG (rebot-vision `GET /snapshot.jpg`: the frame letterboxed
    /// into a square, used by the multi-view measurement). Default: not available.
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(async { Err("camera snapshots not supported by this detection source".to_string()) })
    }
    /// Keep the camera gate open without inference for `ttl_ms` (rebot-vision camera hold:
    /// `POST /api/subscribe {"client", "pipelines": [], "ttl_ms"}`), so a trigger is served by the
    /// next camera frame. Calling it again with the same `client` refreshes the same hold.
    /// `Ok(Some(ticket))` = held (the id may change when the provider restarted: keep the
    /// latest); `Ok(None)` = the provider has no camera hold (older rebot-vision answers 400, a
    /// provider without subscriptions 404). Default: no camera hold.
    fn camera_hold(&self, _client: &str, _ttl_ms: u64) -> BoxFuture<'_, Result<Option<CameraHoldTicket>, String>> {
        Box::pin(async { Ok(None) })
    }
    /// End `client`'s camera hold `ticket` (`DELETE /api/subscribe/<id>?client=&instance_id=`):
    /// a rebot-vision with owner checks deletes it only when it is still this client's
    /// subscription of the same provider instance (after a restart the ids start again at s1).
    fn camera_release(&self, _ticket: &CameraHoldTicket, _client: &str) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
}

/// A camera hold taken by [`DetectionSource::camera_hold`]: the subscription id and the
/// provider instance that issued it (rebot-vision subscription bodies' `instance_id`; None from
/// a provider that does not report it).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraHoldTicket {
    pub id: String,
    pub instance_id: Option<String>,
}

/// The camera app's detection API (the same upstream armd proxies under /api/v1/detection).
pub struct HttpDetectionSource {
    pub upstream: DetectorUpstream,
    /// Size used for detections reported in pixels when the frame carries no size.
    pub image_size_px: [f64; 2],
}

impl HttpDetectionSource {
    /// A result envelope (`latest` or `trigger`) as a frame; `None` when it carries no new frame.
    fn frame(
        &self,
        v: &Value,
        after: Option<u64>,
        req_started: Instant,
        received: Instant,
        wait_ms: u64,
    ) -> Option<DetectionFrame> {
        if v.get("timeout").and_then(Value::as_bool) == Some(true) || v.get("success").and_then(Value::as_bool) == Some(false) {
            return None;
        }
        let frame_id = v.get("frame_id").and_then(Value::as_u64)?;
        if after.is_some_and(|a| frame_id <= a) { return None; }
        let age_dur = v
            .get("age_ms")
            .and_then(Value::as_f64)
            .filter(|a| a.is_finite() && *a >= 0.0)
            .and_then(|age_ms| Duration::try_from_secs_f64(age_ms / 1000.0).ok());
        let captured_at = age_dur
            .and_then(|dur| received.checked_sub(dur))
            .unwrap_or(received);
        let conservative_extra_delay = if after.is_none() || wait_ms == 0 {
            received.saturating_duration_since(req_started)
        } else {
            received
                .saturating_duration_since(req_started)
                .saturating_sub(Duration::from_millis(wait_ms))
        };
        let policy_captured_at = age_dur
            .and_then(|dur| dur.checked_add(conservative_extra_delay))
            .and_then(|total_delay| received.checked_sub(total_delay));
        let size = match (v.get("width").and_then(Value::as_f64), v.get("height").and_then(Value::as_f64)) {
            (Some(w), Some(h)) if w.is_finite() && h.is_finite() && w > 0.0 && h > 0.0 => [w, h],
            _ => self.image_size_px,
        };
        let depth = v.get("depth").map(|d| {
            let metric_model = d.get("metric_model").and_then(Value::as_bool).unwrap_or(false);
            let table_samples = d.get("table_samples").and_then(Value::as_array).map(|arr| {
                arr.iter().filter_map(|item| {
                    let s = item.as_array()?;
                    if s.len() >= 3 {
                        let u = s[0].as_f64()?;
                        let v = s[1].as_f64()?;
                        let disp = s[2].as_f64()?;
                        if u.is_finite() && v.is_finite() && disp.is_finite() {
                            return Some([u, v, disp]);
                        }
                    }
                    None
                }).collect::<Vec<[f64; 3]>>()
            }).unwrap_or_default();
            DepthFrameInfo { metric_model, table_samples }
        });
        let detections = parse_detections(v.get("detections").unwrap_or(&Value::Null), size);
        let model_generation = v.get("model_generation").and_then(Value::as_u64);
        Some(DetectionFrame { frame_id, captured_at, policy_captured_at, image_size_px: size, detections, depth, model_generation })
    }
}

/// Rescales a long-side angle (deg) and the width across it (px) from a frame scaled by
/// (rx, ry) to the camera model's image; returns the scale along the long side (for a length),
/// None without an angle (the width then scales isotropically).
fn rescale_angle_width(angle_deg: &mut Option<f64>, width_px: &mut Option<f64>, rx: f64, ry: f64, cam: &CamModel) -> Option<f64> {
    let isotropic = (rx * ry).sqrt();
    let Some(deg) = *angle_deg else {
        if let Some(w) = width_px.as_mut() {
            *w *= isotropic;
        }
        return None;
    };
    let (sin_t, cos_t) = deg.to_radians().sin_cos();
    let sl = (cos_t * rx).hypot(sin_t * ry);
    let theta_cam = (sin_t * ry).atan2(cos_t * rx);
    let deg_cam = (theta_cam.to_degrees() + 90.0).rem_euclid(180.0) - 90.0;
    *angle_deg = Some(deg_cam);
    let (sin_cam, cos_cam) = deg_cam.to_radians().sin_cos();
    let denom = (-sin_cam / cam.fx).hypot(cos_cam / cam.fy);
    let sw = if denom > 1e-12 {
        (-sin_t * rx / cam.fx).hypot(cos_t * ry / cam.fy) / denom
    } else {
        isotropic
    };
    if let Some(w) = width_px.as_mut() {
        *w *= sw;
    }
    Some(sl)
}

pub fn rescale_frame_to_cam(frame: &mut DetectionFrame, cam: &CamModel) {
    if !(frame.image_size_px[0] > 0.0 && frame.image_size_px[1] > 0.0 && cam.size[0] > 0.0 && cam.size[1] > 0.0) {
        return;
    }
    let rx = cam.size[0] / frame.image_size_px[0];
    let ry = cam.size[1] / frame.image_size_px[1];
    if (rx - 1.0).abs() <= 1e-6 && (ry - 1.0).abs() <= 1e-6 {
        frame.image_size_px = cam.size;
        return;
    }
    if let Some(ref mut d) = frame.depth {
        for sample in &mut d.table_samples {
            sample[0] *= rx;
            sample[1] *= ry;
        }
    }
    for det in &mut frame.detections {
        let isotropic = (rx * ry).sqrt();
        let sl_mask = rescale_angle_width(&mut det.angle_deg, &mut det.grasp_width_px, rx, ry, cam);
        let sl_depth = rescale_angle_width(&mut det.depth_angle_deg, &mut det.depth_width_px, rx, ry, cam);
        // the length follows the detector angle, else the depth angle (the angle the vision
        // pipeline measured it along)
        if let Some(l) = det.length_px.as_mut() {
            *l *= sl_mask.or(sl_depth).unwrap_or(isotropic);
        }
    }
    frame.image_size_px = cam.size;
}

/// A subscription id, instance id or client name armd puts into a query: 1..64 of [A-Za-z0-9_-].
fn hold_token_ok(s: &str) -> bool {
    !s.is_empty() && s.len() <= 64 && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

impl DetectionSource for HttpDetectionSource {
    fn camera_hold(&self, client: &str, ttl_ms: u64) -> BoxFuture<'_, Result<Option<CameraHoldTicket>, String>> {
        let body = serde_json::json!({"client": client, "pipelines": [], "ttl_ms": ttl_ms}).to_string();
        Box::pin(async move {
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "POST", "/api/subscribe", Some(body.as_bytes()), Duration::from_secs(2)).await?;
            if status == axum::http::StatusCode::BAD_REQUEST || status == axum::http::StatusCode::NOT_FOUND { return Ok(None); }
            if !status.is_success() { return Err(format!("camera hold: detector returned {status}")); }
            // an older provider that took [] as an ordinary subscription does not report a hold
            if v.get("hold").and_then(Value::as_bool) != Some(true) { return Ok(None); }
            let id = v.get("id").and_then(Value::as_str).filter(|id| hold_token_ok(id)).ok_or_else(|| "camera hold: reply without a valid id".to_string())?;
            let instance_id = v.get("instance_id").and_then(Value::as_str).filter(|i| hold_token_ok(i)).map(str::to_string);
            Ok(Some(CameraHoldTicket { id: id.to_string(), instance_id }))
        })
    }
    fn camera_release(&self, ticket: &CameraHoldTicket, client: &str) -> BoxFuture<'_, Result<(), String>> {
        // an older rebot-vision ignores the query and deletes by id
        let mut path = format!("/api/subscribe/{}?client={client}", ticket.id);
        if let Some(i) = &ticket.instance_id { path.push_str("&instance_id="); path.push_str(i); }
        let (ok_client, id, client) = (hold_token_ok(client), ticket.id.clone(), client.to_string());
        Box::pin(async move {
            if !ok_client { return Err(format!("camera release: client {client:?} is not [A-Za-z0-9_-]{{1,64}}")); }
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "DELETE", &path, None, Duration::from_secs(2)).await?;
            // 404: already expired; 409: no longer ours (another client's id, or a restarted
            // provider whose ids started again): nothing of this hold is left to end
            if status == axum::http::StatusCode::CONFLICT {
                let why = v.get("error").and_then(serde_json::Value::as_str).unwrap_or("409");
                tracing::info!("camera release {id}: not deleted ({why})");
            }
            if status.is_success() || status == axum::http::StatusCode::NOT_FOUND || status == axum::http::StatusCode::CONFLICT { Ok(()) } else { Err(format!("camera release: detector returned {status}")) }
        })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            let mut info = ModeInfo::parse(v.as_ref());
            info.upstream_loopback = self.upstream.is_loopback();
            Ok(info)
        })
    }
    fn set_models(&self, body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        let body = body.to_string();
        Box::pin(async move {
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "POST", "/api/models", Some(body.as_bytes()), Duration::from_secs(2)).await?;
            Ok((status.as_u16(), v))
        })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.and_then(|v| v.get("per_request_pipelines").and_then(Value::as_bool)) == Some(true))
        })
    }
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref()
                .and_then(|v| v.get("available"))
                .and_then(|a| a.get("depth"))
                .and_then(Value::as_bool)
                .unwrap_or(false))
        })
    }
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger_with_focus(pipelines, wait_ms, None)
    }
    fn aruco_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref().and_then(|v| v.get("available")).and_then(|a| a.get("aruco")).and_then(Value::as_bool).unwrap_or(false))
        })
    }
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            if !self.upstream.is_loopback() {
                return Ok(false);
            }
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref().and_then(|v| v.get("available")).and_then(|a| a.get("gray")).and_then(Value::as_bool).unwrap_or(false))
        })
    }
    fn trigger_markers(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<(DetectionFrame, Vec<crate::mview_stereo::Marker>)>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let body = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, None, req_started, Instant::now(), wait_ms).map(|f| (f, parse_markers(v.get("markers").unwrap_or(&Value::Null)))))
        })
    }
    fn trigger_gray(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let body = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?;
            let Some(frame) = self.frame(&v, None, req_started, Instant::now(), wait_ms) else {
                return Ok(None);
            };
            let markers = parse_markers(v.get("markers").unwrap_or(&Value::Null));
            let instance_id = v.get("instance_id").and_then(Value::as_str).map(str::to_string);
            let gray_obj = v.get("pipelines").and_then(|p| p.get("gray"));
            let gray_ran = gray_obj.and_then(|g| g.get("ran")).and_then(Value::as_bool).unwrap_or(false);
            let gray_ms = gray_obj.and_then(|g| g.get("ms")).and_then(Value::as_f64).filter(|x| x.is_finite());
            Ok(Some(GrayTriggerResult { frame, markers, instance_id, gray_ran, gray_ms }))
        })
    }
    fn gray_frame(&self, instance_id: &str, frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        let instance_id = instance_id.to_string();
        Box::pin(async move {
            if !valid_instance_token(&instance_id) {
                return Err(GrayFrameError::Protocol("invalid gray frame instance_id".into()));
            }
            let path = format!("/api/frame/gray?instance_id={instance_id}&frame_id={frame_id}");
            let t_fetch = Instant::now();
            let (status, content_type, body) = crate::api::proxy::request_status_bytes(
                &self.upstream,
                "GET",
                &path,
                Duration::from_secs(2),
            )
            .await
            .map_err(|e| GrayFrameError::Transport(format!("gray frame {frame_id}: {e}")))?;
            let fetch_ms = t_fetch.elapsed().as_secs_f64() * 1000.0;
            if status != axum::http::StatusCode::OK {
                let err_code = serde_json::from_slice::<Value>(&body)
                    .ok()
                    .and_then(|v| v.get("error").and_then(Value::as_str).map(str::to_string))
                    .unwrap_or_else(|| status.canonical_reason().unwrap_or("error").to_string());
                let msg = format!("gray frame {frame_id}: {} {err_code}", status.as_u16());
                return Err(match status {
                    axum::http::StatusCode::NOT_FOUND => GrayFrameError::NotStored(msg),
                    axum::http::StatusCode::GONE => GrayFrameError::Expired(msg),
                    axum::http::StatusCode::FORBIDDEN => GrayFrameError::Forbidden(msg),
                    axum::http::StatusCode::SERVICE_UNAVAILABLE => GrayFrameError::Busy(msg),
                    _ => GrayFrameError::Protocol(msg),
                });
            }
            if !is_octet_stream(content_type.as_deref()) {
                return Err(GrayFrameError::Protocol(format!(
                    "gray frame {frame_id}: unexpected Content-Type {:?} (expected application/octet-stream)",
                    content_type.as_deref().unwrap_or("<missing>")
                )));
            }
            let mut frame = parse_gray_frame(&body, Some(&instance_id), Some(frame_id))?;
            frame.fetch_ms = fetch_ms;
            Ok(frame)
        })
    }
    fn trigger_with_focus(&self, pipelines: &'static [&'static str], wait_ms: u64, focus: Option<&MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let focus = focus.cloned();
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let mut obj = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms});
            if let Some(ref f) = focus {
                let mut mf = serde_json::json!({
                    "box_norm": f.box_norm,
                    "ttl_ms": f.ttl_ms,
                });
                if let Some(ref l) = f.label {
                    mf["label"] = serde_json::Value::String(l.clone());
                }
                obj["mask_focus"] = mf;
            }
            let body = obj.to_string();
            let req_started = Instant::now();
            let v = match crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await {
                Ok(v) => v,
                // an older rebot-vision ignores the unknown field; retry only on a rejected body
                Err(e) if focus.is_some() && e.starts_with("detector returned 400") => {
                    tracing::debug!("Trigger with mask_focus failed ({e}), retrying without focus");
                    let fallback = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
                    crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(fallback.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?
                }
                Err(e) => return Err(e),
            };
            Ok(self.frame(&v, None, req_started, Instant::now(), wait_ms))
        })
    }
    fn set_mask_focus(&self, label: Option<String>, box_norm: [f64; 4], ttl_ms: u64) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async move {
            let body = if ttl_ms == 0 {
                serde_json::json!({"ttl_ms": 0}).to_string()
            } else {
                let mut j = serde_json::json!({
                    "box_norm": box_norm,
                    "ttl_ms": ttl_ms,
                });
                if let Some(l) = label {
                    j["label"] = serde_json::Value::String(l);
                }
                j.to_string()
            };
            match crate::api::proxy::request_json_opt(&self.upstream, "POST", "/api/mask_focus", Some(body.as_bytes()), Duration::from_secs(2)).await {
                Ok(_) => Ok(()),
                Err(e) => {
                    tracing::debug!("POST /api/mask_focus failed ({e}), ignored");
                    Ok(())
                }
            }
        })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json(&self.upstream, "GET", "/api/inference", None, Duration::from_secs(2)).await?;
            v.get("enabled").and_then(Value::as_bool).ok_or_else(|| "inference state missing".to_string())
        })
    }
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async move {
            let body = serde_json::json!({"enabled": enabled}).to_string();
            crate::api::proxy::request_json(&self.upstream, "POST", "/api/inference", Some(body.as_bytes()), Duration::from_secs(2)).await.map(|_| ())
        })
    }
    fn vision_mode(&self) -> BoxFuture<'_, Result<Option<VisionMode>, String>> {
        Box::pin(async move {
            let Some(v) = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await? else {
                return Ok(None);
            };
            let mode = v.get("mode").and_then(Value::as_str).ok_or_else(|| "vision mode missing".to_string())?;
            Ok(Some(VisionMode { mode: mode.to_string(), enabled: v.get("enabled").and_then(Value::as_bool).unwrap_or(false) }))
        })
    }
    fn set_vision_mode(&self, m: &VisionMode) -> BoxFuture<'_, Result<(), String>> {
        let body = serde_json::json!({"mode": m.mode, "enabled": m.enabled}).to_string();
        Box::pin(async move {
            crate::api::proxy::request_json(&self.upstream, "POST", "/api/mode", Some(body.as_bytes()), Duration::from_secs(2)).await.map(|_| ())
        })
    }
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(crate::api::proxy::fetch_snapshot(&self.upstream, Duration::from_secs(3)))
    }
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let path = format!("/api/results/latest?after={}&wait_ms={wait_ms}", after.unwrap_or(0));
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "GET", &path, None, Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, after, req_started, Instant::now(), wait_ms))
        })
    }
}


// Private-seam unit test (`HttpDetectionSource::frame`); move to tests/grasp/ once S6 adds `feature = "testing"`.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::grasp::{measure_detection_depth_with_cal, DepthCalibration, Detection};
    use crate::kinematics::Pose;

    #[test]
    fn test_http_detection_source_rescaling() {
        let src = HttpDetectionSource {
            upstream: DetectorUpstream::parse("http://127.0.0.1:8090").unwrap(),
            image_size_px: [640.0, 360.0],
        };
        let raw = serde_json::json!({
            "frame_id": 1,
            "width": 1280,
            "height": 720,
            "depth": {
                "metric_model": true,
                "table_samples": [[100.0, 200.0, 0.85]],
            },
            "detections": [{
                "label": "cup",
                "score": 0.9,
                "format": "xywh_center",
                "x": 640.0,
                "y": 360.0,
                "w": 100.0,
                "h": 100.0,
                "length_px": 80.0,
                "grasp_width_px": 40.0,
            }]
        });
        let now = Instant::now();
        let mut frame = src.frame(&raw, None, now, now, 0).expect("frame parsed");
        assert_eq!(frame.image_size_px, [1280.0, 720.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [100.0, 200.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(80.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(40.0));

        let cam_iso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 900.0,
            fy: 900.0,
            cx: 480.0,
            cy: 270.0,
            size: [960.0, 540.0],
        };
        rescale_frame_to_cam(&mut frame, &cam_iso);
        assert_eq!(frame.image_size_px, [960.0, 540.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [75.0, 150.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(60.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(30.0));

        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "test" };
        let pose = Pose::new(crate::kinematics::IDENTITY, [0.0; 3]);
        let mut det_iso = frame.detections[0].clone();
        det_iso.depth_conf = Some(0.9);
        det_iso.disp_fg = Some(1.0);
        det_iso.disp_bg = Some(0.5);
        let meas_iso = measure_detection_depth_with_cal(&cam_iso, &pose, &det_iso, &cal).expect("measured");
        let expected_l_iso = 80.0 / 1200.0;
        let expected_w_iso = 40.0 / 1200.0;
        assert!((meas_iso.length_m.unwrap() - expected_l_iso).abs() < 1e-9);
        assert!((meas_iso.width_m.unwrap() - expected_w_iso).abs() < 1e-9);

        // Anisotropic rescaling:
        let now_aniso = Instant::now();
        let mut frame_aniso = DetectionFrame {
            frame_id: 2,
            captured_at: now_aniso,
            policy_captured_at: Some(now_aniso),
            image_size_px: [640.0, 360.0],
            detections: vec![Detection {
                label: "cup".into(),
                score: 0.9,
                raw_score_valid: true,
                box_norm: [0.3, 0.3, 0.7, 0.7],
                angle_deg: Some(45.0),
                angle_conf: Some(0.9),
                grasp_width_px: Some(120.0),
                mask_focus: false,
                disp_fg: Some(1.0),
                disp_bg: Some(0.5),
                depth_conf: Some(0.9),
                length_px: Some(240.0),
                jaw_clear: Some(true),
                depth_angle_deg: None,
                depth_angle_conf: None,
                depth_width_px: None, contour_norm: None,
            }],
            depth: None,
            model_generation: None,
        };
        let cam_aniso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 1200.0,
            fy: 1800.0,
            cx: 640.0,
            cy: 540.0,
            size: [1280.0, 1080.0],
        };
        rescale_frame_to_cam(&mut frame_aniso, &cam_aniso);
        assert_eq!(frame_aniso.image_size_px, [1280.0, 1080.0]);

        let meas_aniso = measure_detection_depth_with_cal(&cam_aniso, &pose, &frame_aniso.detections[0], &cal).expect("measured");
        let expected_l_aniso = 240.0 / 600.0;
        let expected_w_aniso = 120.0 / 600.0;
        assert!((meas_aniso.length_m.unwrap() - expected_l_aniso).abs() < 1e-9);
        assert!((meas_aniso.width_m.unwrap() - expected_w_aniso).abs() < 1e-9);
    }
}
