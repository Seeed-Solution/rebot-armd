use std::collections::BTreeMap;
use std::time::Instant;

use serde_json::Value;

use super::gray::{
    GRAY_V1_HEIGHT, GRAY_V1_LUMA, GRAY_V1_LUMA_RANGE, GRAY_V1_STRIDE, GRAY_V1_WIDTH,
};
use super::http::DetectionSource;
use super::tracking::f;

// ---- detections ----

#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    pub label: String,
    pub score: f64,
    /// `true` when the upstream raw score was finite and in `[0.0, 1.0]` before clamping.
    pub raw_score_valid: bool,
    /// Normalised [x0, y0, x1, y1].
    pub box_norm: [f64; 4],
    /// Orientation of the object's long side in the image (deg, from +x toward +y, y down,
    /// wrapped to [-90, 90)) and its confidence [0, 1], when the detector reports them.
    pub angle_deg: Option<f64>,
    pub angle_conf: Option<f64>,
    pub grasp_width_px: Option<f64>,
    pub mask_focus: bool,
    pub disp_fg: Option<f64>,
    pub disp_bg: Option<f64>,
    pub depth_conf: Option<f64>,
    pub length_px: Option<f64>,
    pub jaw_clear: Option<bool>,
    /// The depth pipeline's own long-side angle (deg, as `angle_deg`), its confidence and
    /// cross-section width (px), when the depth pipeline ran (rebot-vision `depth_angle_deg`,
    /// `depth_angle_conf`, `depth_width_px`). Never mixed into `angle_deg` / `grasp_width_px`.
    pub depth_angle_deg: Option<f64>,
    pub depth_angle_conf: Option<f64>,
    pub depth_width_px: Option<f64>,
    /// Exterior contour of the segmentation mask (rebot-vision `contour`, [x, y] normalised to the
    /// visible image like `box_norm`, at most [`MAX_CONTOUR_VERTICES`]): None when absent,
    /// Some(Err(reason)) when malformed (that detection's contour only is unusable).
    pub contour_norm: Option<Result<Vec<[f64; 2]>, String>>,
}

/// Most vertices a detection contour may have.
pub const MAX_CONTOUR_VERTICES: usize = 128;

/// A detection's `contour` value: [[x, y], ...] normalised (pixels when the box was in pixels:
/// divided by `size`), 3..=[`MAX_CONTOUR_VERTICES`] finite vertices within [-0.01, 1.01] (clamped
/// to [0, 1]) and a non-zero area.
pub fn parse_contour(v: &Value, pixel: bool, size: [f64; 2]) -> Result<Vec<[f64; 2]>, String> {
    let arr = v.as_array().ok_or("contour_not_an_array")?;
    if arr.len() > MAX_CONTOUR_VERTICES { return Err("contour_too_many_vertices".into()); }
    let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
    let mut out = Vec::with_capacity(arr.len());
    for p in arr {
        let q = p.as_array().filter(|q| q.len() == 2).ok_or("contour_vertex_not_a_pair")?;
        let (Some(x), Some(y)) = (q[0].as_f64(), q[1].as_f64()) else { return Err("contour_vertex_not_numeric".into()) };
        let (x, y) = (x / sx, y / sy);
        if !(x.is_finite() && y.is_finite()) || !(-0.01..=1.01).contains(&x) || !(-0.01..=1.01).contains(&y) {
            return Err("contour_vertex_out_of_range".into());
        }
        out.push([x.clamp(0.0, 1.0), y.clamp(0.0, 1.0)]);
    }
    crate::mview_stereo::footprint::validate_polygon(&out, MAX_CONTOUR_VERTICES).map_err(str::to_string)?;
    Ok(out)
}

#[derive(Debug, Clone, PartialEq)]
pub struct DepthFrameInfo {
    pub metric_model: bool,
    pub table_samples: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MaskFocus {
    pub label: Option<String>,
    pub box_norm: [f64; 4],
    pub ttl_ms: u64,
}

#[derive(Debug, Clone)]
pub struct DetectionFrame {
    pub frame_id: u64,
    /// Capture time on this host's clock (response time − `age_ms`).
    pub captured_at: Instant,
    /// Conservative capture time for policy runtime staleness checks (`Some` when upstream
    /// reported a valid non-negative finite `age_ms` and subtraction succeeded, penalized for
    /// HTTP response latency beyond any server long-poll `wait_ms`).
    pub policy_captured_at: Option<Instant>,
    pub image_size_px: [f64; 2],
    pub detections: Vec<Detection>,
    pub depth: Option<DepthFrameInfo>,
    /// `model_generation`: the provider's model set the frame ran with, when reported.
    pub model_generation: Option<u64>,
}

/// A provider's vision mode (`GET /api/mode` of rebot-vision).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VisionMode {
    /// `off`, `yolo`, `aruco` or `both`.
    pub mode: String,
    /// Continuous inference on.
    pub enabled: bool,
}

/// Format descriptor advertised in `GET /api/mode` `gray_frames` for loopback peers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayFormatDescriptor {
    pub encoding: String,
    pub format: String,
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub ring: usize,
    pub exposure_qualified: bool,
    pub frame_size: Option<[usize; 2]>,
    pub crop: Option<[usize; 4]>,
    pub luma: Option<String>,
    pub luma_range: Option<String>,
}

impl GrayFormatDescriptor {
    /// `gray_frames` of a `GET /api/mode` body: `Ok(None)` when not advertised, `Err` when
    /// advertised but malformed (a wrong type, arity or a missing required field), which must not
    /// be read as "absent".
    fn parse_advertised(v: &Value) -> Result<Option<Self>, String> {
        let Some(g) = v.get("gray_frames") else { return Ok(None) };
        Self::parse(g).map(Some).ok_or_else(|| format!("malformed gray_frames descriptor: {g}"))
    }

    fn parse(g: &Value) -> Option<Self> {
        let g = g.as_object()?;
        let parse_u_array = |k: &str, len: usize| -> Option<Vec<usize>> {
            let a = g.get(k)?.as_array()?;
            if a.len() != len { return None; }
            a.iter().map(|x| usize::try_from(x.as_u64()?).ok()).collect()
        };
        let frame_size = match g.get("frame_size") {
            None => None,
            Some(_) => {
                let fs = parse_u_array("frame_size", 2)?;
                Some([fs[0], fs[1]])
            }
        };
        let crop = match g.get("crop") {
            None => None,
            Some(_) => {
                let c = parse_u_array("crop", 4)?;
                Some([c[0], c[1], c[2], c[3]])
            }
        };
        let luma = match g.get("luma") {
            None => None,
            Some(v) => Some(v.as_str()?.to_string()),
        };
        let luma_range = match g.get("luma_range") {
            None => None,
            Some(v) => Some(v.as_str()?.to_string()),
        };
        Some(Self {
            encoding: g.get("encoding")?.as_str()?.to_string(),
            format: g.get("format")?.as_str()?.to_string(),
            width: usize::try_from(g.get("width")?.as_u64()?).ok()?,
            height: usize::try_from(g.get("height")?.as_u64()?).ok()?,
            stride: usize::try_from(g.get("stride")?.as_u64()?).ok()?,
            ring: usize::try_from(g.get("ring")?.as_u64()?).ok()?,
            // optional (false when absent); any other type than bool makes the descriptor malformed
            exposure_qualified: match g.get("exposure_qualified") {
                None => false,
                Some(v) => v.as_bool()?,
            },
            frame_size,
            crop,
            luma,
            luma_range,
        })
    }

    /// True when the advertised descriptor matches the fixed v1 RGF1 contract.
    pub fn is_supported_v1(&self) -> bool {
        if self.encoding != "RGF1"
            || self.format != "gray8"
            || self.width != GRAY_V1_WIDTH
            || self.height != GRAY_V1_HEIGHT
            || self.stride != GRAY_V1_STRIDE
            || self.ring == 0
        {
            return false;
        }
        if let Some([fw, fh]) = self.frame_size {
            if fw == 0 || fh == 0 {
                return false;
            }
        }
        if let Some([cx, cy, cw, ch]) = self.crop {
            if cw != self.width || ch != self.height {
                return false;
            }
            if let Some([fw, fh]) = self.frame_size {
                if cx.checked_add(cw).is_none_or(|x| x > fw) || cy.checked_add(ch).is_none_or(|y| y > fh) {
                    return false;
                }
            }
        }
        if let Some(ref l) = self.luma {
            if l != GRAY_V1_LUMA {
                return false;
            }
        }
        if let Some(ref r) = self.luma_range {
            if r != GRAY_V1_LUMA_RANGE {
                return false;
            }
        }
        true
    }
}

/// One reading of the provider's `GET /api/mode`, shared by a grasp's task check and its
/// perception negotiation.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeInfo {
    /// The provider has a mode API (false: `/api/mode` 404, an older detector).
    pub reported: bool,
    /// `per_request_pipelines` (perception API 2).
    pub per_request_pipelines: bool,
    /// `available.yolo` (true when not reported): a detector is loaded. An older provider
    /// reports `model.kind` `detect` with the slot empty, so the kind alone does not tell.
    pub yolo_available: bool,
    /// `available.aruco`.
    pub aruco_available: bool,
    /// `available.depth`.
    pub depth_available: bool,
    /// `available.gray` (loopback peers only on rebot-vision).
    pub gray_available: bool,
    /// `gray_frames` format descriptor: `Ok(None)` when not advertised, `Err` when malformed.
    pub gray_format: Result<Option<GrayFormatDescriptor>, String>,
    /// True when the upstream detector endpoint is on loopback (local device).
    pub upstream_loopback: bool,
    /// `depth_model.metric`.
    pub depth_metric: bool,
    /// `model.kind` (`detect` | `segment` | `obb`), when reported (null with the detector slot empty).
    pub model_kind: Option<String>,
    /// `mode` / `enabled` (`Ok(None)` without a mode API); used only without per-request
    /// pipelines.
    pub vision: Result<Option<VisionMode>, String>,
    /// `models` and `ion` (a provider that can switch models, `POST /api/models`).
    pub models: Option<ProviderModels>,
}

/// Names of the provider's loaded models (`models.resident`); None = slot empty.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ResidentModels {
    pub detector: Option<String>,
    pub depth: Option<String>,
}

/// The provider's model state (`GET /api/mode` `models`, `ion`).
#[derive(Debug, Clone, PartialEq)]
pub struct ProviderModels {
    /// +1 per executed swap; every result carries the one it ran with (`model_generation`).
    pub generation: u64,
    pub resident: ResidentModels,
    /// `swap.state`: `idle`, `pending`, `running` or `failed`.
    pub swap_state: String,
    pub swap_error: Option<String>,
    /// Catalogue name -> `ion_mb` (None when not reported).
    pub catalogue: BTreeMap<String, Option<f64>>,
    /// Catalogue name -> `kind` (`detect`, `segment`, `obb`, `depth`), when reported.
    pub catalogue_kinds: BTreeMap<String, String>,
    /// `ion` (`base_mb`, `total_mb`, `reserve_mb`), when reported.
    pub ion: Option<[f64; 3]>,
}

impl ProviderModels {
    fn parse(v: &Value) -> Option<Self> {
        let m = v.get("models")?;
        let name = |s: Option<&Value>| s.and_then(Value::as_str).map(str::to_string);
        let resident = m.get("resident");
        let swap = m.get("swap");
        let entries = m.get("catalogue").and_then(Value::as_array);
        let catalogue = entries.map(|a| a.iter().filter_map(|e| {
            Some((e.get("name")?.as_str()?.to_string(), f(e, &["ion_mb"])))
        }).collect()).unwrap_or_default();
        let catalogue_kinds = entries.map(|a| a.iter().filter_map(|e| {
            Some((e.get("name")?.as_str()?.to_string(), e.get("kind")?.as_str()?.to_string()))
        }).collect()).unwrap_or_default();
        let ion = v.get("ion").and_then(|i| Some([f(i, &["base_mb"])?, f(i, &["total_mb"])?, f(i, &["reserve_mb"])?]));
        Some(Self {
            generation: m.get("generation").and_then(Value::as_u64)?,
            resident: ResidentModels { detector: name(resident.and_then(|r| r.get("detector"))), depth: name(resident.and_then(|r| r.get("depth"))) },
            swap_state: name(swap.and_then(|s| s.get("state"))).unwrap_or_else(|| "unknown".into()),
            swap_error: name(swap.and_then(|s| s.get("error"))),
            catalogue,
            catalogue_kinds,
            ion,
        })
    }

    /// The ION budget check of `set` from the catalogue (`Err(reason)` when it does not fit);
    /// None when the provider does not report the catalogue's `ion_mb` or its `ion` budget.
    #[doc(hidden)]
    pub fn budget(&self, set: &ResidentModels) -> Option<Result<(), String>> {
        let [base, total, reserve] = self.ion?;
        let mut need = base + reserve;
        for name in [&set.detector, &set.depth].into_iter().flatten() {
            need += (*self.catalogue.get(name)?)?;
        }
        Some(if need <= total + 1e-9 { Ok(()) } else { Err(refusal_message("ION_BUDGET", Some(need), Some(total))) })
    }
}

/// A provider's refusal (`error`, with `need_mb` / `limit_mb` for `ION_BUDGET`).
pub(in crate::grasp) fn refusal_message(error: &str, need: Option<f64>, limit: Option<f64>) -> String {
    match (need, limit) {
        (Some(n), Some(l)) => format!("{error} (need {n:.1} MB, limit {l:.1} MB)"),
        _ => error.to_string(),
    }
}

impl ModeInfo {
    /// Parse a `GET /api/mode` body (`None` = 404).
    pub fn parse(v: Option<&Value>) -> Self {
        let Some(v) = v else {
            return Self {
                reported: false, per_request_pipelines: false, yolo_available: true,
                aruco_available: false, depth_available: false, gray_available: false,
                gray_format: Ok(None), upstream_loopback: true, depth_metric: false,
                model_kind: None, vision: Ok(None), models: None,
            };
        };
        let vision = match v.get("mode").and_then(Value::as_str) {
            Some(mode) => Ok(Some(VisionMode { mode: mode.to_string(), enabled: v.get("enabled").and_then(Value::as_bool).unwrap_or(false) })),
            None => Err("vision mode missing".to_string()),
        };
        let avail = |k: &str| v.get("available").and_then(|a| a.get(k)).and_then(Value::as_bool);
        Self {
            reported: true,
            per_request_pipelines: v.get("per_request_pipelines").and_then(Value::as_bool) == Some(true),
            yolo_available: avail("yolo").unwrap_or(true),
            aruco_available: avail("aruco").unwrap_or(false),
            depth_available: avail("depth").unwrap_or(false),
            gray_available: avail("gray").unwrap_or(false),
            gray_format: GrayFormatDescriptor::parse_advertised(v),
            upstream_loopback: true,
            depth_metric: v.get("depth_model").and_then(|d| d.get("metric")).and_then(Value::as_bool).unwrap_or(false),
            model_kind: v.get("model").and_then(|m| m.get("kind")).and_then(Value::as_str).map(str::to_string),
            vision,
            models: ProviderModels::parse(v),
        }
    }

    /// From the separate queries of a source without its own [`DetectionSource::mode_info`]:
    /// a per-request query error is the error; depth/aruco/gray query errors read as unavailable;
    /// the vision mode is read only without per-request pipelines.
    pub async fn compose(source: &(impl DetectionSource + ?Sized)) -> Result<Self, String> {
        let per_request_pipelines = source.per_request_pipelines().await?;
        let aruco_available = source.aruco_available().await.unwrap_or(false);
        let depth_available = source.depth_available().await.unwrap_or(false);
        let gray_available = source.gray_available().await.unwrap_or(false);
        let vision = if per_request_pipelines { Ok(None) } else { source.vision_mode().await };
        Ok(Self {
            reported: per_request_pipelines || matches!(vision, Ok(Some(_))),
            per_request_pipelines, yolo_available: true, aruco_available, depth_available,
            gray_available, gray_format: Ok(None), upstream_loopback: true, depth_metric: false,
            model_kind: None, vision, models: None,
        })
    }
}

