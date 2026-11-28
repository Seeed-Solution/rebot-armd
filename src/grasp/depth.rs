use crate::kinematics::Pose;

use super::{box_centre, median, CamModel, DepthFrameInfo, Detection};

// ---- depth calibration & measurement ----

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthCalibration {
    pub a: f64,
    pub b: f64,
    pub source: &'static str,
}

/// Start-frame depth kept for the overhead measurement (`two_height_fk` needs both frames).
#[derive(Default)]
pub(super) struct DepthCache {
    pub(super) prior: Option<TableObs>,
    pub(super) meas: Option<DepthMeasurement>,
    pub(super) det0: Option<Detection>,
    /// Why `meas` is None.
    pub(super) why: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TableObs {
    pub inv_z_fk: f64,
    pub disp_table: f64,
    pub samples: Vec<(f64, f64, f64)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DepthMeasurement {
    pub height_m: f64,
    pub depth_m: f64,
    pub width_m: Option<f64>,
    pub length_m: Option<f64>,
    /// Base-frame unit closing axis along which `width_m` was measured (perpendicular to the
    /// projected long-axis angle on the object top plane), when an orientation angle was present.
    pub closing_axis: Option<[f64; 2]>,
    pub source: &'static str,
    pub jaw_clear: Option<bool>,
}

/// Table samples of a depth frame that calibrate the disparity: `(1 / z_fk, disp, z_fk)` per
/// sample with a positive finite disparity whose ray meets the table (`table_z_m`) at an
/// optical depth z_fk in [0.08, 2.5] m (FK pose + camera model).
pub fn table_fk_samples(cam: &CamModel, pose: &Pose, depth: &DepthFrameInfo, table_z_m: f64) -> Vec<(f64, f64, f64)> {
    depth.table_samples.iter().filter_map(|&[u, v, disp]| {
        if disp <= 1e-3 || !disp.is_finite() { return None; }
        cam.table_optical_depth(pose, [u, v], table_z_m).map(|z_fk| (1.0 / z_fk, disp, z_fk))
    }).collect()
}

pub fn extract_table_obs(
    cam: &CamModel,
    pose: &Pose,
    depth: &DepthFrameInfo,
    table_z_m: f64,
) -> Option<TableObs> {
    let mut samples: Vec<(f64, f64, f64)> = Vec::new();
    let mut inv_zs = Vec::new();
    let mut disps = Vec::new();
    for sample in &depth.table_samples {
        let [u, v, disp] = *sample;
        if disp > 1e-3 && disp.is_finite() {
            if let Some(z_fk) = cam.table_optical_depth(pose, [u, v], table_z_m) {
                samples.push((1.0 / z_fk, disp, z_fk));
                inv_zs.push(1.0 / z_fk);
                disps.push(disp);
            }
        }
    }
    if samples.len() < 6 { return None; }
    Some(TableObs {
        inv_z_fk: median(&mut inv_zs),
        disp_table: median(&mut disps),
        samples,
    })
}

fn fit_oblique_ransac(samples: &[(f64, f64, f64)]) -> Option<DepthCalibration> {
    let n = samples.len();
    if n < 8 { return None; }

    let mut best_inliers: Vec<usize> = Vec::new();
    let mut best_pair_a = 0.0;
    let mut best_pair_b = 0.0;
    let mut best_err_sum = f64::INFINITY;

    for i in 0..n {
        for j in (i + 1)..n {
            let (y_i, x_i, _) = samples[i];
            let (y_j, x_j, _) = samples[j];
            let dx = x_j - x_i;
            if dx.abs() < 1e-4 { continue; }
            let a = (y_j - y_i) / dx;
            let b = y_i - a * x_i;
            if a <= 1e-4 || !b.is_finite() { continue; }

            let mut inliers = Vec::new();
            let mut err_sum = 0.0;
            for (k, &(_, x_k, z_fk_k)) in samples.iter().enumerate() {
                let pred_inv_z = a * x_k + b;
                if pred_inv_z <= 1e-4 { continue; }
                let z_pred = 1.0 / pred_inv_z;
                let rel_err = (z_pred - z_fk_k).abs() / z_fk_k;
                if rel_err <= 0.08 {
                    inliers.push(k);
                    err_sum += rel_err;
                }
            }

            if inliers.len() > best_inliers.len()
                || (inliers.len() == best_inliers.len() && err_sum < best_err_sum)
            {
                best_inliers = inliers;
                best_pair_a = a;
                best_pair_b = b;
                best_err_sum = err_sum;
            }
        }
    }

    let min_inliers_req = 6.max((0.6 * n as f64).ceil() as usize);
    if best_inliers.len() < min_inliers_req {
        return None;
    }

    let mut a = best_pair_a;
    let mut b = best_pair_b;

    for _ in 0..4 {
        let mut residuals: Vec<f64> = samples.iter().map(|&(y, x, _)| (y - (a * x + b)).abs()).collect();
        let mad = median(&mut residuals).max(1e-4);
        let delta = 1.345 * mad;

        let mut sum_w = 0.0;
        let mut sum_wx = 0.0;
        let mut sum_wy = 0.0;
        let mut sum_wxx = 0.0;
        let mut sum_wxy = 0.0;

        for &(y, x, _) in samples {
            let r = (y - (a * x + b)).abs();
            let w = if r <= delta { 1.0 } else { delta / r };
            sum_w += w;
            sum_wx += w * x;
            sum_wy += w * y;
            sum_wxx += w * x * x;
            sum_wxy += w * x * y;
        }

        let denom = sum_w * sum_wxx - sum_wx * sum_wx;
        if denom.abs() < 1e-12 { break; }
        let new_a = (sum_w * sum_wxy - sum_wx * sum_wy) / denom;
        let new_b = (sum_wy - new_a * sum_wx) / sum_w;
        if new_a <= 1e-4 || !new_b.is_finite() { break; }
        a = new_a;
        b = new_b;
    }

    if a <= 1e-4 || !b.is_finite() {
        return None;
    }

    let mut final_inliers = 0;
    let mut sum_sq_rel_err = 0.0;
    for &(_, x_k, z_fk_k) in samples {
        let pred_inv_z = a * x_k + b;
        if pred_inv_z <= 1e-4 { continue; }
        let z_pred = 1.0 / pred_inv_z;
        let rel_err = (z_pred - z_fk_k).abs() / z_fk_k;
        if rel_err <= 0.08 {
            final_inliers += 1;
            sum_sq_rel_err += rel_err * rel_err;
        }
    }

    if final_inliers < min_inliers_req {
        return None;
    }

    let rel_rmse = (sum_sq_rel_err / final_inliers as f64).sqrt();
    if rel_rmse <= 0.06 {
        Some(DepthCalibration { a, b, source: "oblique_ransac" })
    } else {
        None
    }
}

pub fn calibrate_depth(
    cam: &CamModel,
    pose: &Pose,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Option<DepthCalibration> {
    calibrate_depth_why(cam, pose, depth, prior, table_z_m).ok()
}

/// [`calibrate_depth`] with the reason it failed: `no_depth_frame` (the frame has no depth),
/// `too_few_samples` (metric: < 3 table samples; relative: < 8 and no two-height pair),
/// `table_contradicts_fk` (the table samples do not fit the FK table depths),
/// `no_oblique_spread` (relative: the table's inverse depth spans < 0.35 1/m),
/// `no_two_height_spread` / `two_height_fit` (relative with a start frame: the two views are
/// too close, or the combined fit fails).
pub fn calibrate_depth_why(
    cam: &CamModel,
    pose: &Pose,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Result<DepthCalibration, &'static str> {
    let depth = depth.ok_or("no_depth_frame")?;

    let valid_samples = table_fk_samples(cam, pose, depth, table_z_m);

    if depth.metric_model {
        let a_prior = (600.0 / 640.0) / (cam.fx / cam.size[0].max(1.0));
        if valid_samples.len() >= 3 {
            let mut ratios: Vec<f64> = valid_samples.iter().map(|&(inv_z, disp, _)| inv_z / disp).collect();
            let a = median(&mut ratios);
            let b = 0.0;
            if (0.25..=4.0).contains(&a) {
                // Ensure samples agree with both the metric prior and the fitted table scale a.
                let mut prior_errs: Vec<f64> = valid_samples.iter().map(|&(_, disp, z_fk)| {
                    let z_pred = 1.0 / (a_prior * disp);
                    (z_pred - z_fk).abs() / z_fk
                }).collect();
                let mut fit_errs: Vec<f64> = valid_samples.iter().map(|&(_, disp, z_fk)| {
                    let z_pred = 1.0 / (a * disp);
                    (z_pred - z_fk).abs() / z_fk
                }).collect();
                if median(&mut prior_errs) <= 0.12 && median(&mut fit_errs) <= 0.12 {
                    return Ok(DepthCalibration { a, b, source: "metric_table" });
                }
            }
            return Err("table_contradicts_fk");
        }
        // Fewer than 3 table samples (table occluded): the nominal scale alone is not checked
        // against FK, so it must not replace the mechanical probe.
        return Err("too_few_samples");
    }

    let mut why = "too_few_samples";
    if valid_samples.len() >= 8 {
        let mut min_inv_z = f64::INFINITY;
        let mut max_inv_z = f64::NEG_INFINITY;
        for &(inv_z, _, _) in &valid_samples {
            if inv_z < min_inv_z { min_inv_z = inv_z; }
            if inv_z > max_inv_z { max_inv_z = inv_z; }
        }
        if max_inv_z - min_inv_z >= 0.35 {
            if let Some(cal) = fit_oblique_ransac(&valid_samples) {
                return Ok(cal);
            }
            why = "table_contradicts_fk";
        } else {
            why = "no_oblique_spread";
        }
    }

    if let Some(prior_obs) = prior {
        if !depth.metric_model && prior_obs.samples.len() >= 6 && valid_samples.len() >= 6 {
            let mut inv_zs: Vec<f64> = valid_samples.iter().map(|s| s.0).collect();
            let mut disps: Vec<f64> = valid_samples.iter().map(|s| s.1).collect();
            let inv_z_2 = median(&mut inv_zs);
            let d_2 = median(&mut disps);
            let d_inv_z = inv_z_2 - prior_obs.inv_z_fk;
            let d_disp = d_2 - prior_obs.disp_table;

            why = "no_two_height_spread";
            if d_inv_z.abs() >= 0.35 && d_disp.abs() >= 0.05 && (d_inv_z * d_disp > 0.0) {
                why = "two_height_fit";
                let mut combined = prior_obs.samples.clone();
                combined.extend_from_slice(&valid_samples);
                if let Some(cal) = fit_oblique_ransac(&combined) {
                    if (0.05..=20.0).contains(&cal.a) {
                        let check_inliers = |samples: &[(f64, f64, f64)]| -> bool {
                            let inliers = samples.iter().filter(|&&(_, x, z_fk)| {
                                let pred_inv_z = cal.a * x + cal.b;
                                if pred_inv_z <= 1e-4 { return false; }
                                let z_pred = 1.0 / pred_inv_z;
                                (z_pred - z_fk).abs() / z_fk <= 0.08
                            }).count();
                            inliers as f64 >= 0.60 * samples.len() as f64
                        };
                        if check_inliers(&prior_obs.samples) && check_inliers(&valid_samples) {
                            return Ok(DepthCalibration { a: cal.a, b: cal.b, source: "two_height_fk" });
                        }
                    }
                }
            }
        }
    }

    Err(why)
}

pub(super) fn metric_table_contradicts_fk(cam: &CamModel, pose: &Pose, depth: Option<&DepthFrameInfo>, table_z_m: f64) -> bool {
    let Some(d) = depth else { return false; };
    if !d.metric_model { return false; }
    let valid_count = d.table_samples.iter().filter(|s| {
        let [u, v, disp] = **s;
        disp > 1e-3 && disp.is_finite() && cam.table_optical_depth(pose, [u, v], table_z_m).is_some()
    }).count();
    valid_count >= 3 && calibrate_depth(cam, pose, Some(d), None, table_z_m).is_none()
}

pub fn measure_detection_depth_with_cal(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    cal: &DepthCalibration,
) -> Option<DepthMeasurement> {
    measure_detection_depth_with_cal_why(cam, pose, det, cal).ok()
}

/// [`measure_detection_depth_with_cal`] with the reason it failed: `low_conf` (`depth_conf`
/// < 0.35), `no_disparity`, `bad_disparity` (foreground not nearer than the table, or depths
/// out of range), `too_flat` (< 2 mm above the table).
// `!(a > b)` also rejects NaN
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn measure_detection_depth_with_cal_why(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    cal: &DepthCalibration,
) -> Result<DepthMeasurement, &'static str> {
    if det.depth_conf.unwrap_or(0.0) < 0.35 { return Err("low_conf"); }
    let (Some(disp_fg), Some(disp_bg)) = (det.disp_fg, det.disp_bg) else { return Err("no_disparity"); };
    if !(disp_fg > disp_bg) { return Err("bad_disparity"); }
    let denom_fg = cal.a * disp_fg + cal.b;
    let denom_bg = cal.a * disp_bg + cal.b;
    if !(denom_fg > 0.1 && denom_bg > 0.1) { return Err("bad_disparity"); }
    let z_fg = 1.0 / denom_fg;
    let z_bg = 1.0 / denom_bg;
    if !( (0.06..=2.0).contains(&z_fg) && z_bg > z_fg ) { return Err("bad_disparity"); }

    let centre_px = cam.to_px(box_centre(&det.box_norm));
    let ray = cam.ray_world_per_z(pose, centre_px);
    let h_raw = (z_bg - z_fg) * ray[2].abs();
    if !(h_raw > 0.002) { return Err("too_flat"); }
    let h = h_raw.max(0.005);

    let depth_m = z_fg;
    // the detector's width and angle (a segmentation mask) when it reports them, else the
    // depth pipeline's own (a plain detection model): the same inputs as before rebot-vision
    // reported the depth values separately
    let (width_px, angle_deg) = if det.angle_deg.is_some() || det.grasp_width_px.is_some() {
        (det.grasp_width_px, det.angle_deg)
    } else {
        (det.depth_width_px, det.depth_angle_deg)
    };
    let (sin_t, cos_t) = angle_deg.map(|deg| deg.to_radians().sin_cos()).unwrap_or((0.0, 0.0));
    let f_geom = (cam.fx * cam.fy).sqrt();
    let width_m = width_px.filter(|w| *w > 0.0).map(|w| {
        if angle_deg.is_some() {
            w * z_fg * (-sin_t / cam.fx).hypot(cos_t / cam.fy)
        } else {
            w * z_fg / f_geom
        }
    });
    let length_m = det.length_px.filter(|l| *l > 0.0).map(|l| {
        if angle_deg.is_some() {
            l * z_fg * (cos_t / cam.fx).hypot(sin_t / cam.fy)
        } else {
            l * z_fg / f_geom
        }
    });
    let closing_axis = angle_deg.filter(|_| width_m.is_some()).and_then(|_| {
        let o = cam.origin(pose);
        let z_plane = o[2] + z_fg * ray[2];
        let p0 = cam.plane_point(pose, centre_px, z_plane)?;
        let p1 = cam.plane_point(pose, [centre_px[0] + 20.0 * cos_t, centre_px[1] + 20.0 * sin_t], z_plane)?;
        let long = [p1[0] - p0[0], p1[1] - p0[1]];
        let n = long[0].hypot(long[1]);
        (n > 1e-9).then_some([-long[1] / n, long[0] / n])
    });
    Ok(DepthMeasurement {
        height_m: h,
        depth_m,
        width_m,
        length_m,
        closing_axis,
        source: cal.source,
        jaw_clear: det.jaw_clear,
    })
}

pub fn measure_detection_depth(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Option<DepthMeasurement> {
    let cal = calibrate_depth(cam, pose, depth, prior, table_z_m)?;
    measure_detection_depth_with_cal(cam, pose, det, &cal)
}
