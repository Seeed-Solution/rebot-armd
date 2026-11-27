use serde_json::Value;

use super::frames::{parse_contour, Detection, DetectionFrame};
use crate::grasp::{touches_border, GraspSetup};

/// ArUco markers of a trigger reply (`markers`: `[{id, corners: [[x, y] x 4]}]`, pixels of the
/// frame, the format calib_hover reads from rebot-vision's `aruco` pipeline). Entries without an
/// integer id or four finite corners are skipped.
pub fn parse_markers(raw: &Value) -> Vec<crate::mview_stereo::Marker> {
    raw.as_array().map(|a| a.iter().filter_map(|m| {
        let id = m.get("id").and_then(Value::as_i64)?;
        let c = m.get("corners").and_then(Value::as_array).filter(|c| c.len() == 4)?;
        let mut corners = [[0.0; 2]; 4];
        for (k, p) in c.iter().enumerate() {
            let p = p.as_array()?;
            corners[k] = [p.first()?.as_f64().filter(|x| x.is_finite())?, p.get(1)?.as_f64().filter(|x| x.is_finite())?];
        }
        Some(crate::mview_stereo::Marker { id, corners })
    }).collect()).unwrap_or_default()
}

pub(in crate::grasp) fn f(v: &Value, keys: &[&str]) -> Option<f64> {
    keys.iter().find_map(|k| v.get(*k).and_then(Value::as_f64)).filter(|x| x.is_finite())
}

/// Detector boxes → normalised boxes. Accepts `[cx, cy, w, h, score, label]` arrays and
/// objects with `box_norm`, or `format` `xyxy` / `xywh_center` / `xywh_top_left` (as the
/// cockpit's `parseDetections`), or centre `x`, `y`, `w`, `h` without `format` (rebot-vision
/// 1.2). Values above 1.05 are pixels of `size`. Objects may carry `angle_deg` (long side,
/// wrapped to [-90, 90)) and `angle_conf` (clamped to [0, 1]).
pub fn parse_detections(raw: &Value, size: [f64; 2]) -> Vec<Detection> {
    let Some(items) = raw.as_array() else { return Vec::new(); };
    let mut out = Vec::new();
    for item in items {
        let (label, raw_score_opt, cx, cy, w, h) = if let Some(a) = item.as_array().filter(|a| a.len() >= 5) {
            let n = |i: usize| a.get(i).and_then(Value::as_f64).filter(|x| x.is_finite());
            let (Some(cx), Some(cy), Some(w), Some(h), Some(s)) = (n(0), n(1), n(2), n(3), n(4)) else { continue };
            let label = a.get(5).map(|l| l.as_str().map(str::to_string).unwrap_or_else(|| l.to_string())).unwrap_or_else(|| "obj".into());
            (label, Some(s), cx, cy, w, h)
        } else if item.is_object() {
            let label = ["label", "name", "target", "class_name"].iter()
                .find_map(|k| item.get(*k).and_then(Value::as_str)).unwrap_or("obj").to_string();
            let raw_score_opt = f(item, &["score", "confidence"]);
            if let Some(b) = item.get("box_norm").and_then(Value::as_array).filter(|b| b.len() == 4) {
                let b: Vec<f64> = b.iter().filter_map(Value::as_f64).collect();
                if b.len() != 4 { continue; }
                (label, raw_score_opt, (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, b[2] - b[0], b[3] - b[1])
            } else {
                let arr = |k: &str| item.get(k).and_then(Value::as_array).filter(|b| b.len() >= 4)
                    .map(|b| b.iter().take(4).filter_map(Value::as_f64).collect::<Vec<_>>()).filter(|b| b.len() == 4);
                let bb = arr("bbox").or_else(|| arr("box"));
                match item.get("format").and_then(Value::as_str) {
                    Some("xyxy") => {
                        let (x0, y0, x1, y1) = match (&bb, f(item, &["x0", "x1"]), f(item, &["y0", "y1"])) {
                            (Some(b), _, _) => (b[0], b[1], b[2], b[3]),
                            _ => match (f(item, &["x1"]), f(item, &["x2"]), f(item, &["y1"]), f(item, &["y2"])) {
                                (Some(x1), Some(x2), Some(y1), Some(y2)) => (x1, y1, x2, y2),
                                _ => match (f(item, &["x0"]), f(item, &["x1"]), f(item, &["y0"]), f(item, &["y1"])) {
                                    (Some(a), Some(b), Some(c), Some(d)) => (a, c, b, d),
                                    _ => continue,
                                },
                            },
                        };
                        (label, raw_score_opt, (x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0)
                    }
                    Some("xywh_center") => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w", "width"]), f(item, &["h", "height"])) else { continue };
                        (label, raw_score_opt, cx, cy, w, h)
                    }
                    Some("xywh_top_left") => {
                        let (x0, y0, w, h) = match &bb {
                            Some(b) => (b[0], b[1], b[2], b[3]),
                            None => match (f(item, &["x", "x0"]), f(item, &["y", "y0"]), f(item, &["w", "width"]), f(item, &["h", "height"])) {
                                (Some(a), Some(b), Some(c), Some(d)) => (a, b, c, d),
                                _ => continue,
                            },
                        };
                        (label, raw_score_opt, x0 + w / 2.0, y0 + h / 2.0, w, h)
                    }
                    // rebot-vision 1.2 (perception API 1) sent centre x, y, w, h without `format`
                    None if bb.is_none() => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w"]), f(item, &["h"])) else { continue };
                        (label, raw_score_opt, cx, cy, w, h)
                    }
                    _ => continue,
                }
            }
        } else { continue };
        let score = raw_score_opt.unwrap_or(0.0);
        if !(w > 0.0 && h > 0.0 && score.is_finite()) { continue; }
        let pixel = cx > 1.05 || cy > 1.05 || w > 1.05 || h > 1.05;
        let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
        let b = [(cx - w / 2.0) / sx, (cy - h / 2.0) / sy, (cx + w / 2.0) / sx, (cy + h / 2.0) / sy].map(|v| v.clamp(0.0, 1.0));
        if b[2] <= b[0] || b[3] <= b[1] { continue; }
        let angle_deg = item.is_object().then(|| f(item, &["angle_deg"])).flatten().map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
        let angle_conf = item.is_object().then(|| f(item, &["angle_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let grasp_width_px = item.is_object().then(|| f(item, &["grasp_width_px", "width_px"])).flatten().filter(|w| *w > 0.0);
        let mask_focus = item.is_object().then(|| item.get("mask_focus").and_then(Value::as_bool)).flatten().unwrap_or(false);
        let disp_fg = item.is_object().then(|| f(item, &["disp_fg"])).flatten();
        let disp_bg = item.is_object().then(|| f(item, &["disp_bg"])).flatten();
        let depth_conf = item.is_object().then(|| f(item, &["depth_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let length_px = item.is_object().then(|| f(item, &["length_px"])).flatten().filter(|l| *l > 0.0);
        let jaw_clear = item.is_object().then(|| item.get("jaw_clear").and_then(Value::as_bool)).flatten();
        let depth_angle_deg = item.is_object().then(|| f(item, &["depth_angle_deg"])).flatten().map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
        let depth_angle_conf = item.is_object().then(|| f(item, &["depth_angle_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let depth_width_px = item.is_object().then(|| f(item, &["depth_width_px"])).flatten().filter(|w| *w > 0.0);
        let raw_score_valid = raw_score_opt.is_some_and(|s| s.is_finite() && (0.0..=1.0).contains(&s));
        let contour_norm = item.get("contour").filter(|c| !c.is_null()).map(|c| parse_contour(c, pixel, size));
        out.push(Detection { label, score: score.clamp(0.0, 1.0), raw_score_valid, box_norm: b, angle_deg, angle_conf, grasp_width_px, mask_focus, disp_fg, disp_bg, depth_conf, length_px, jaw_clear,
            depth_angle_deg, depth_angle_conf, depth_width_px, contour_norm });
    }
    out
}

pub fn box_changed_px(a: &[f64; 4], b: &[f64; 4], size: [f64; 2]) -> bool {
    let dx0 = (a[0] - b[0]).abs() * size[0];
    let dy0 = (a[1] - b[1]).abs() * size[1];
    let dx1 = (a[2] - b[2]).abs() * size[0];
    let dy1 = (a[3] - b[3]).abs() * size[1];
    dx0 > 5.0 || dy0 > 5.0 || dx1 > 5.0 || dy1 > 5.0
}

pub fn box_centre(b: &[f64; 4]) -> [f64; 2] { [(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0] }

pub fn iou(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    let w = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
    let h = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
    let inter = w * h;
    let union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    if union > 0.0 { inter / union } else { 0.0 }
}

pub(in crate::grasp) fn median(v: &mut [f64]) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2.0 }
}

/// Follows the picked target across frames: same label, and centre within the gate of the
/// predicted centre or IoU above `match_iou` with the predicted box. While the predicted box
/// touches the image border the gate is at least `acquire_dist_px`: the centre of a cut box
/// moves by up to half the hidden part as the box comes into view.
#[derive(Debug, Clone)]
pub struct Tracker {
    pub label: String,
    /// Predicted box (normalised) of the target in the next frame.
    pub predicted: [f64; 4],
    pub gate_px: f64,
}

impl Tracker {
    pub fn find(&self, frame: &DetectionFrame, setup: &GraspSetup) -> Option<Detection> {
        let [w, h] = setup.image_size_px;
        let pc = box_centre(&self.predicted);
        let gate = if touches_border(&self.predicted, [w, h]) { self.gate_px.max(setup.acquire_dist_px) } else { self.gate_px };
        frame.detections.iter()
            .filter(|d| d.label.eq_ignore_ascii_case(&self.label))
            .filter_map(|d| {
                let c = box_centre(&d.box_norm);
                let dist = ((c[0] - pc[0]) * w).hypot((c[1] - pc[1]) * h);
                (dist <= gate || iou(&d.box_norm, &self.predicted) > setup.match_iou).then_some((dist, d))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, d)| d.clone())
    }

    /// Re-centre the predicted box on `centre_norm`, keeping its size.
    pub fn predict_at(&mut self, centre_norm: [f64; 2]) {
        let c = box_centre(&self.predicted);
        let (dx, dy) = (centre_norm[0] - c[0], centre_norm[1] - c[1]);
        self.predicted = [self.predicted[0] + dx, self.predicted[1] + dy, self.predicted[2] + dx, self.predicted[3] + dy];
    }
}

