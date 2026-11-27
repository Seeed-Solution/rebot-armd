use std::time::Instant;

use serde_json::Value;

use super::frames::{DetectionFrame, GrayFormatDescriptor};

/// RGF1 raw gray frame wire constants (rebot-vision `GET /api/frame/gray`).
pub const GRAY_FRAME_MAGIC: &[u8; 4] = b"RGF1";
pub const GRAY_FRAME_ALIGN: usize = 8;
pub const GRAY_MAX_HEADER_BYTES: usize = 16384;
pub const GRAY_V1_WIDTH: usize = 640;
pub const GRAY_V1_HEIGHT: usize = 360;
pub const GRAY_V1_STRIDE: usize = 640;
pub const GRAY_V1_LUMA: &str = "(77*R+150*G+29*B)>>8";
pub const GRAY_V1_LUMA_RANGE: &str = "full_0_255";

/// One decoded raw gray frame from rebot-vision `GET /api/frame/gray`.
#[derive(Debug, Clone, PartialEq)]
pub struct GrayFrame {
    pub instance_id: String,
    pub frame_id: u64,
    pub img: crate::mview_stereo::Gray,
    pub frame_size: [usize; 2],
    pub crop: [usize; 4],
    pub ts_ms: u64,
    pub frame_mono_ms: u64,
    pub wait_mono_ms: u64,
    pub grab_mono_ms: u64,
    pub stored_mono_ms: u64,
    pub convert_ms: f64,
    /// HTTP transfer duration (ms), excluding RGF1 JSON parsing and pixel buffer copy.
    pub fetch_ms: f64,
    pub parse_ms: f64,
}

/// Trigger reply with provider instance identity and gray pipeline status.
#[derive(Debug, Clone)]
pub struct GrayTriggerResult {
    pub frame: DetectionFrame,
    pub markers: Vec<crate::mview_stereo::Marker>,
    pub instance_id: Option<String>,
    pub gray_ran: bool,
    pub gray_ms: Option<f64>,
}

/// Typed errors from fetching or parsing a gray frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GrayFrameError {
    /// 404 FRAME_NOT_STORED (retryable with a new bracket + trigger).
    NotStored(String),
    /// 410 FRAME_EXPIRED: evicted or instance mismatch (retryable with a new bracket + trigger).
    Expired(String),
    /// 503 GRAY_BUSY: concurrent transfer cap reached (retryable).
    Busy(String),
    /// 403 FORBIDDEN: non-loopback peer (non-retryable).
    Forbidden(String),
    /// Network or timeout error (retryable).
    Transport(String),
    /// Malformed RGF1, wrong media type, identity mismatch, or unsupported geometry (non-retryable).
    Protocol(String),
}

impl GrayFrameError {
    pub fn is_retryable(&self) -> bool {
        matches!(self, Self::NotStored(_) | Self::Expired(_) | Self::Busy(_) | Self::Transport(_))
    }

    pub fn retry_reason(&self) -> &'static str {
        match self {
            Self::NotStored(_) => "not_stored",
            Self::Expired(_) => "expired",
            Self::Busy(_) => "busy",
            Self::Forbidden(_) => "forbidden",
            Self::Transport(_) => "transport",
            Self::Protocol(_) => "protocol",
        }
    }
}

impl std::fmt::Display for GrayFrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotStored(s) | Self::Expired(s) | Self::Busy(s) | Self::Forbidden(s)
            | Self::Transport(s) | Self::Protocol(s) => f.write_str(s),
        }
    }
}

pub(super) fn is_octet_stream(content_type: Option<&str>) -> bool {
    let Some(ct) = content_type else { return false; };
    let media = ct.split(';').next().unwrap_or("").trim();
    media.eq_ignore_ascii_case("application/octet-stream")
}

pub(super) fn valid_instance_token(s: &str) -> bool {
    !s.is_empty() && s.len() <= 64 && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

/// Strictly parse an `RGF1` gray frame body (`"RGF1"` + u32 LE `N` + `N` space-padded UTF-8 JSON
/// bytes aligned to 8 + exact `height * stride` pixel bytes). Validates bounds, alignment,
/// checked arithmetic, required v1 metadata (encoding, source dims, crop bounds, luma, timing),
/// fixed v1 geometry (`640x360`, `stride=640`), exact payload length, and expected identity
/// before allocating the pixel buffer.
pub fn parse_gray_frame(
    body: &[u8],
    expected_instance_id: Option<&str>,
    expected_frame_id: Option<u64>,
) -> Result<GrayFrame, GrayFrameError> {
    parse_gray_frame_with_format(body, expected_instance_id, expected_frame_id, None)
}

/// Strict [`parse_gray_frame`] that additionally checks the frame metadata against a negotiated
/// [`GrayFormatDescriptor`] from `GET /api/mode`.
pub fn parse_gray_frame_with_format(
    body: &[u8],
    expected_instance_id: Option<&str>,
    expected_frame_id: Option<u64>,
    expected_format: Option<&GrayFormatDescriptor>,
) -> Result<GrayFrame, GrayFrameError> {
    let t0 = Instant::now();
    let proto = |m: String| GrayFrameError::Protocol(m);
    if body.len() < 8 {
        return Err(proto(format!("RGF1 body truncated ({} bytes)", body.len())));
    }
    if &body[..4] != GRAY_FRAME_MAGIC {
        return Err(proto("RGF1 bad magic".into()));
    }
    let header_len = u32::from_le_bytes([body[4], body[5], body[6], body[7]]) as usize;
    if header_len == 0 || header_len > GRAY_MAX_HEADER_BYTES {
        return Err(proto(format!("RGF1 invalid header length {header_len} (max {GRAY_MAX_HEADER_BYTES})")));
    }
    let off = 8usize
        .checked_add(header_len)
        .ok_or_else(|| proto("RGF1 header offset overflow".into()))?;
    if !off.is_multiple_of(GRAY_FRAME_ALIGN) {
        return Err(proto(format!("RGF1 pixel offset {off} is not {GRAY_FRAME_ALIGN}-byte aligned")));
    }
    if off > body.len() {
        return Err(proto(format!("RGF1 header truncated (need {off} bytes, got {})", body.len())));
    }
    let hdr_bytes = &body[8..off];
    let hdr_str = std::str::from_utf8(hdr_bytes).map_err(|_| proto("RGF1 header is not valid UTF-8".into()))?;
    // Trailing padding must be ASCII spaces only (no arbitrary trailing junk).
    let trimmed = hdr_str.trim_end_matches(' ');
    if trimmed.is_empty() || trimmed.bytes().last() != Some(b'}') {
        return Err(proto("RGF1 header padding or JSON terminator invalid".into()));
    }
    let v: Value = serde_json::from_str(trimmed).map_err(|e| proto(format!("RGF1 header JSON invalid: {e}")))?;
    if !v.is_object() {
        return Err(proto("RGF1 header must be a JSON object".into()));
    }
    if v.get("api_version").and_then(Value::as_u64) != Some(2) {
        return Err(proto("RGF1 unsupported api_version (expected 2)".into()));
    }
    if v.get("encoding").and_then(Value::as_str) != Some("RGF1") {
        return Err(proto("RGF1 missing or unsupported encoding (expected RGF1)".into()));
    }
    if v.get("format").and_then(Value::as_str) != Some("gray8") {
        return Err(proto("RGF1 unsupported format (expected gray8)".into()));
    }
    if v.get("frame_format").and_then(Value::as_str) != Some("RGB888") {
        return Err(proto("RGF1 missing or unsupported frame_format (expected RGB888)".into()));
    }
    if v.get("luma").and_then(Value::as_str) != Some(GRAY_V1_LUMA) {
        return Err(proto("RGF1 missing or unsupported luma equation".into()));
    }
    if v.get("luma_range").and_then(Value::as_str) != Some(GRAY_V1_LUMA_RANGE) {
        return Err(proto("RGF1 missing or unsupported luma_range (expected full_0_255)".into()));
    }
    let instance_id = v
        .get("instance_id")
        .and_then(Value::as_str)
        .filter(|s| valid_instance_token(s))
        .ok_or_else(|| proto("RGF1 missing or invalid instance_id".into()))?
        .to_string();
    let frame_id = v
        .get("frame_id")
        .and_then(Value::as_u64)
        .ok_or_else(|| proto("RGF1 missing frame_id".into()))?;
    if let Some(exp) = expected_instance_id {
        if instance_id != exp {
            return Err(proto(format!("RGF1 instance_id mismatch: expected {exp}, got {instance_id}")));
        }
    }
    if let Some(exp) = expected_frame_id {
        if frame_id != exp {
            return Err(proto(format!("RGF1 frame_id mismatch: expected {exp}, got {frame_id}")));
        }
    }
    let req_u64 = |k: &str| -> Result<u64, GrayFrameError> {
        v.get(k).and_then(Value::as_u64).ok_or_else(|| proto(format!("RGF1 missing or invalid {k}")))
    };
    let req_usize = |k: &str| -> Result<usize, GrayFrameError> {
        let n = req_u64(k)?;
        usize::try_from(n).map_err(|_| proto(format!("RGF1 {k} overflow")))
    };
    let req_usize_array = |k: &str, len: usize| -> Result<Vec<usize>, GrayFrameError> {
        let arr = v.get(k).and_then(Value::as_array).ok_or_else(|| proto(format!("RGF1 missing {k}")))?;
        if arr.len() != len {
            return Err(proto(format!("RGF1 {k} must have {len} elements")));
        }
        arr.iter()
            .map(|el| {
                let n = el.as_u64().ok_or_else(|| proto(format!("RGF1 {k} element must be non-negative integer")))?;
                usize::try_from(n).map_err(|_| proto(format!("RGF1 {k} element overflow")))
            })
            .collect()
    };
    let width = req_usize("width")?;
    let height = req_usize("height")?;
    let stride = req_usize("stride")?;
    if width != GRAY_V1_WIDTH || height != GRAY_V1_HEIGHT || stride != GRAY_V1_STRIDE {
        return Err(proto(format!(
            "RGF1 unsupported geometry {width}x{height} stride {stride} (expected {GRAY_V1_WIDTH}x{GRAY_V1_HEIGHT} stride {GRAY_V1_STRIDE})"
        )));
    }
    let fs_vec = req_usize_array("frame_size", 2)?;
    let frame_size = [fs_vec[0], fs_vec[1]];
    if frame_size[0] == 0 || frame_size[1] == 0 {
        return Err(proto(format!("RGF1 invalid frame_size {}x{}", frame_size[0], frame_size[1])));
    }
    let crop_vec = req_usize_array("crop", 4)?;
    let crop = [crop_vec[0], crop_vec[1], crop_vec[2], crop_vec[3]];
    if crop[2] != width || crop[3] != height {
        return Err(proto(format!(
            "RGF1 crop size {}x{} does not match output {}x{}",
            crop[2], crop[3], width, height
        )));
    }
    if crop[0].checked_add(crop[2]).is_none_or(|x| x > frame_size[0])
        || crop[1].checked_add(crop[3]).is_none_or(|y| y > frame_size[1])
    {
        return Err(proto(format!(
            "RGF1 crop {:?} exceeds frame_size {:?}",
            crop, frame_size
        )));
    }
    if let Some(fmt) = expected_format {
        if fmt.encoding != "RGF1" || fmt.format != "gray8" {
            return Err(proto("RGF1 negotiated descriptor encoding/format incompatible".into()));
        }
        if width != fmt.width || height != fmt.height || stride != fmt.stride {
            return Err(proto(format!(
                "RGF1 geometry {width}x{height} stride {stride} does not match negotiated {}x{} stride {}",
                fmt.width, fmt.height, fmt.stride
            )));
        }
        if let Some(exp_fs) = fmt.frame_size {
            if frame_size != exp_fs {
                return Err(proto(format!(
                    "RGF1 frame_size {:?} does not match negotiated {:?}",
                    frame_size, exp_fs
                )));
            }
        }
        if let Some(exp_crop) = fmt.crop {
            if crop != exp_crop {
                return Err(proto(format!(
                    "RGF1 crop {:?} does not match negotiated {:?}",
                    crop, exp_crop
                )));
            }
        }
    }
    let _channel = req_u64("channel")?;
    let _model_generation = req_u64("model_generation")?;
    let ts_ms = req_u64("ts_ms")?;
    let frame_mono_ms = req_u64("frame_mono_ms")?;
    let wait_mono_ms = req_u64("wait_mono_ms")?;
    let grab_mono_ms = req_u64("grab_mono_ms")?;
    let stored_mono_ms = req_u64("stored_mono_ms")?;
    if stored_mono_ms < grab_mono_ms {
        return Err(proto(format!("RGF1 stored_mono_ms {stored_mono_ms} precedes grab_mono_ms {grab_mono_ms}")));
    }
    let convert_ms = v
        .get("convert_ms")
        .and_then(Value::as_f64)
        .filter(|x| x.is_finite() && *x >= 0.0)
        .ok_or_else(|| proto("RGF1 missing or invalid convert_ms".into()))?;

    let payload_len = height
        .checked_mul(stride)
        .ok_or_else(|| proto("RGF1 payload size overflow".into()))?;
    let total = off
        .checked_add(payload_len)
        .ok_or_else(|| proto("RGF1 total size overflow".into()))?;
    if body.len() != total {
        return Err(proto(format!(
            "RGF1 payload length mismatch: expected {total} bytes total, got {}",
            body.len()
        )));
    }
    let img = crate::mview_stereo::Gray::new(width, height, body[off..total].to_vec())
        .ok_or_else(|| proto("RGF1 pixel buffer allocation mismatch".into()))?;
    let parse_ms = t0.elapsed().as_secs_f64() * 1000.0;
    Ok(GrayFrame {
        instance_id,
        frame_id,
        img,
        frame_size,
        crop,
        ts_ms,
        frame_mono_ms,
        wait_mono_ms,
        grab_mono_ms,
        stored_mono_ms,
        convert_ms,
        fetch_ms: 0.0,
        parse_ms,
    })
}
