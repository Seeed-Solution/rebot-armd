/* =================================================================
   Hand-eye calibration, two setups (CAL.mode):
   - eye_in_hand: camera on the gripper, one ArUco marker fixed on the table. Each sample pairs
     T_base_gripper (full URDF FK J1..J6, CALM.fk) with the marker's 6-D pose T_cam_marker; Park & Martin
     AX = XB over all sample pairs gives T_cam_to_gripper. Quality: T_base_marker = G X M per sample must
     be constant (position / rotation RMSE about the mean).
   - eye_to_hand: camera fixed, marker centre at the TCP. Marker position in the camera frame vs TCP
     position in the base frame (CALM.fk); T_cam_to_base = rigid fit p_base = R p_cam + t (Horn 1987).
   ================================================================= */
const CAL_MODES = ['eye_in_hand', 'eye_to_hand'];
const CAL = { mode: 'eye_in_hand', samples: [], target: 9, result: null, markerId: null, resetArmedUntil: 0, modeArmedUntil: 0, busy: false, liveTimer: null };
(() => {
  const pref = store('rebot.calib.mode');
  if (CAL_MODES.includes(pref)) CAL.mode = pref;
  try {
    const sv = JSON.parse(store('rebot.calib') || 'null');
    if (sv && Array.isArray(sv.samples)) {
      CAL.samples = sv.samples; CAL.markerId = sv.markerId ?? null;
      // saved samples keep the mode they were taken in (saves without a mode predate eye_in_hand)
      if (sv.samples.length) CAL.mode = CAL_MODES.includes(sv.mode) ? sv.mode : 'eye_to_hand';
    }
  } catch (_) {}
})();
const calSave = () => store('rebot.calib', JSON.stringify({ mode: CAL.mode, samples: CAL.samples, markerId: CAL.markerId }));
const calEih = () => CAL.mode === 'eye_in_hand';
const calMin = () => (calEih() ? 5 : 3);
// eye_in_hand quality limits: position RMSE 5 mm, rotation RMSE 1 deg; eye_to_hand: point RMSE 10 mm
const calWarn = (r) => (r.rotRmse != null ? r.rmse > 0.005 || r.rotRmse > 1 : r.rmse > 0.01);
// T_base_gripper from raw motor positions through toModel6 (CFG.cart sign / offset, J1..J6); on 6-DOF
// arms index 5 is the gripper and URDF joint6 stays at 0
const calGripper = (pos) => { const q = toModel6(pos); if (S.dof !== 7) q[5] = 0; return CALM.fk(q); };
/* CALIB-MATH-BEGIN */
// Pure hand-eye calibration math: no DOM and no page state. scripts/calib_selftest.mjs evaluates this
// block in Node and checks it against the rebot-companion SDK. 4x4 transforms are row-major nested
// arrays; T_a_b maps frame-b coordinates into frame a.

// symmetric n x n Jacobi eigen-decomposition -> { vals, vecs }, eigenvector i is column i of vecs
function symEig(A) {
  const n = A.length, a = A.map((r) => r.slice()), V = A.map((_, i) => A.map((__, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < 1e-22) break;
    for (let p = 0; p < n - 1; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-30) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const tt = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(tt * tt + 1), sn = tt * c;
      for (let k = 0; k < n; k++) { // A <- A J
        const akp = a[k][p], akq = a[k][q];
        a[k][p] = c * akp - sn * akq; a[k][q] = sn * akp + c * akq;
      }
      for (let k = 0; k < n; k++) { // A <- J^T A
        const apk = a[p][k], aqk = a[q][k];
        a[p][k] = c * apk - sn * aqk; a[q][k] = sn * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - sn * vkq; V[k][q] = sn * vkp + c * vkq;
      }
    }
  }
  return { vals: a.map((r, i) => r[i]), vecs: V };
}
// eigenvector of the largest eigenvalue
function maxEigVec4(A) {
  const { vals, vecs } = symEig(A);
  let best = 0;
  for (let i = 1; i < vals.length; i++) if (vals[i] > vals[best]) best = i;
  return vecs.map((r) => r[best]);
}
// Horn (1987): rotation R minimising sum |R src_i - dst_i|^2 (closed-form quaternion)
function hornR(src, dst) {
  const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < src.length; i++) for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) M[r][c] += src[i][r] * dst[i][c];
  const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = M;
  const N = [
    [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
    [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy],
    [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz],
  ];
  let [w, x, y, z] = maxEigVec4(N);
  const qn = Math.hypot(w, x, y, z) || 1; w /= qn; x /= qn; y /= qn; z /= qn;
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
}
// p_base = R p_cam + t over point pairs; returns { R, t, rmse, maxErr, residuals } or { err }
function solveRigid(cam, base) {
  const n = cam.length;
  if (n < 3) return { err: 'calib.e.few' };
  const mean = (P) => [0, 1, 2].map((k) => P.reduce((s2, p) => s2 + p[k], 0) / n);
  const ca = mean(cam), cb = mean(base);
  const A = cam.map((p) => p.map((v, k) => v - ca[k])), B = base.map((p) => p.map((v, k) => v - cb[k]));
  // non-collinear: largest triangle area in the camera set must exceed 1 cm^2
  let area = 0;
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) for (let k = j + 1; k < n; k++) {
    const u = A[j].map((v, m) => v - A[i][m]), w = A[k].map((v, m) => v - A[i][m]);
    area = Math.max(area, Math.hypot(u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]) / 2);
  }
  if (area < 1e-4) return { err: 'calib.e.degenerate' };
  const R = hornR(A, B);
  const mul = (p) => [0, 1, 2].map((r) => R[r][0] * p[0] + R[r][1] * p[1] + R[r][2] * p[2]);
  const Rca = mul(ca), t = [0, 1, 2].map((k) => cb[k] - Rca[k]);
  const residuals = cam.map((p, i) => { const q = mul(p); return Math.hypot(q[0] + t[0] - base[i][0], q[1] + t[1] - base[i][1], q[2] + t[2] - base[i][2]); });
  const rmse = Math.sqrt(residuals.reduce((s2, e) => s2 + e * e, 0) / n);
  return { R, t, rmse, maxErr: Math.max(...residuals), residuals };
}

const CALM = (() => {
  const D = 180 / Math.PI;
  const mul = (A, B) => A.map((r) => B[0].map((_, c) => r.reduce((s, v, k) => s + v * B[k][c], 0)));
  const tr = (A) => A[0].map((_, c) => A.map((r) => r[c]));
  const mv = (R, v) => R.map((r) => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
  const rotOf = (T) => T.slice(0, 3).map((r) => r.slice(0, 3));
  const posOf = (T) => T.slice(0, 3).map((r) => r[3]);
  const make = (R, t) => [...R.map((r, i) => [...r, t[i]]), [0, 0, 0, 1]];
  const inv = (T) => { const Rt = tr(rotOf(T)), p = mv(Rt, posOf(T)); return make(Rt, p.map((v) => -v)); };
  // R = Rz(yaw) Ry(pitch) Rx(roll), radians (URDF rpy and the detector r_deg convention)
  const rpyR = (r, p, y) => {
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]];
  };
  // inverse of rpyR, degrees [roll, pitch, yaw]
  const rpyDeg = (R) => [Math.atan2(R[2][1], R[2][2]) * D, Math.asin(Math.min(1, Math.max(-1, -R[2][0]))) * D, Math.atan2(R[1][0], R[0][0]) * D];
  const axisR = (k, a) => { // Rodrigues, unit axis k
    const c = Math.cos(a), s = Math.sin(a), C = 1 - c, [x, y, z] = k;
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]];
  };
  // URDF chain base_link -> joint1..joint6 -> end_joint -> end_link of assets/reBot-DevArm_gripper.urdf
  // (same table as rebot_companion.kinematics.URDF_CHAIN): origin xyz, origin rpy, joint axis (null = fixed)
  const CHAIN = [
    [[-8.416e-05, 0.0, 0.08465], [0.0, 0.0, 0.0], [0, 0, 1]],
    [[0.020084, 0.031625, 0.05555], [-1.5708, 0.0, 0.0], [0, 0, -1]],
    [[-0.264, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 0, 1]],
    [[0.2426, -0.054, -0.001625], [0.0, 0.0, 0.0], [0, 0, 1]],
    [[0.078308, -0.0375, -0.03], [-1.5708, 0.0, 0.0], [0, 0, 1]],
    [[0.028008, 0.0, 0.04], [0.0, 1.5708, 0.0], [0, 0, 1]],
    [[0.0, 0.0, 0.15539], [0.0, -1.5708, 3.1415], null],
  ].map(([xyz, rpy, axis]) => [make(rpyR(...rpy), xyz), axis]);
  /* Kinematic correction (armd schema `kinematics_correction`, written by the fine calibration wizard): per
     joint i (0..5) { offset_rad, scale, tilt_rad: [x, y], origin_m: [dx, dy, dz] }. Walking the chain:
       M = M * F_i;  M = M * [Rx(tilt_x) Ry(tilt_y), origin_m];  M = M * Rot(axis_i, q_i (1 + scale) + offset_rad)
     (armd src/kinematics.rs and rebot_companion.kinematics use the same model). Null or all zero: the plain URDF
     chain, computed by the same code as without a correction. */
  const KC_BOUNDS = { offset_rad: 10 / D, scale: 0.08, tilt_rad: 3 / D, origin_m: 0.03 };
  // schema value -> [{ off, sc, R (tilt, nested 3x3) or null, d: [3] }] x 6, or null (none / all zero / malformed)
  function normCorrection(joints) {
    if (!Array.isArray(joints) || joints.length < 6) return null;
    const out = [];
    let any = false;
    for (let i = 0; i < 6; i++) {
      const j = joints[i] || {};
      const off = Number(j.offset_rad) || 0, sc = Number(j.scale) || 0;
      const tl = Array.isArray(j.tilt_rad) ? j.tilt_rad.map((v) => Number(v) || 0) : [0, 0];
      const d = Array.isArray(j.origin_m) ? j.origin_m.map((v) => Number(v) || 0) : [0, 0, 0];
      if (![off, sc, tl[0], tl[1], d[0], d[1], d[2]].every(Number.isFinite)) return null;
      const frame = !!(tl[0] || tl[1] || d[0] || d[1] || d[2]);
      if (off || sc || frame) any = true;
      out.push({ off, sc, R: frame ? mul(axisR([1, 0, 0], tl[0]), axisR([0, 1, 0], tl[1])) : null, d: [d[0], d[1], d[2]] });
    }
    return any ? out : null;
  }
  let KC = null, KCJ = null;
  // set the correction in use (schema value or null); returns true when it changed the model
  function setCorrection(joints) {
    const k = normCorrection(joints), was = JSON.stringify(KC);
    KC = k; KCJ = k ? joints : null;
    return JSON.stringify(KC) !== was;
  }
  const correction = () => KC;
  const correctionJoints = () => KCJ; // as set (schema shape), for a worker's copy of this block
  // URDF model angles [q1..q6] (rad; missing = 0) -> T_base_gripper (end_link in base_link, x = tool axis);
  // kc: normalised correction (default: the one set by setCorrection), null = plain URDF chain
  function fk(q, kc = KC) {
    let M = make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0]), i = 0;
    for (const [F, axis] of CHAIN) {
      M = mul(M, F);
      if (!axis) continue;
      const c = kc && kc[i];
      if (!c) { M = make(mul(rotOf(M), axisR(axis, Number(q[i++]) || 0)), posOf(M)); continue; }
      if (c.R) M = mul(M, make(c.R, c.d));
      M = make(mul(rotOf(M), axisR(axis, (Number(q[i++]) || 0) * (1 + c.sc) + c.off)), posOf(M));
    }
    return M;
  }
  // rotation matrix -> unit quaternion [w, x, y, z] with w >= 0 (Shepperd)
  function quat(R) {
    const s0 = R[0][0] + R[1][1] + R[2][2];
    let q;
    if (s0 > 0) { const s = Math.sqrt(s0 + 1) * 2; q = [s / 4, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s]; }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) { const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; q = [(R[2][1] - R[1][2]) / s, s / 4, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s]; }
    else if (R[1][1] > R[2][2]) { const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; q = [(R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, s / 4, (R[1][2] + R[2][1]) / s]; }
    else { const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; q = [(R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, s / 4]; }
    const n = Math.hypot(...q) * (q[0] < 0 ? -1 : 1);
    return q.map((v) => v / n);
  }
  const quatR = ([w, x, y, z]) => {
    const n = Math.hypot(w, x, y, z) || 1; w /= n; x /= n; y /= n; z /= n;
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
  };
  // rotation matrix -> rotation vector (axis * angle, rad)
  function logR(R) {
    const [w, x, y, z] = quat(R), s = Math.hypot(x, y, z);
    if (s < 1e-12) return [2 * x, 2 * y, 2 * z];
    const th = 2 * Math.atan2(s, w);
    return [x / s * th, y / s * th, z / s * th];
  }
  const expR = (v) => { const th = Math.hypot(...v); return th < 1e-15 ? [[1, 0, 0], [0, 1, 0], [0, 0, 1]] : axisR(v.map((c) => c / th), th); };
  const angDeg = (R) => Math.hypot(...logR(R)) * D;
  // chordal L2 mean rotation (same as scipy Rotation.mean): top eigenvector of sum q q^T
  function meanR(Rs) {
    const M = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (const R of Rs) { const q = quat(R); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) M[i][j] += q[i] * q[j]; }
    return quatR(maxEigVec4(M));
  }
  const solve3 = (A, b) => { // Cramer's rule; null when singular
    const det = (M) => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    const d = det(A), scale = Math.max(...A.flat().map(Math.abs));
    if (!(Math.abs(d) > 1e-9 * scale * scale * scale)) return null;
    return [0, 1, 2].map((c) => det(A.map((r, i) => r.map((v, j) => (j === c ? b[i] : v)))) / d);
  };
  // relative gripper rotations over all sample pairs (A_ij = G_j^-1 G_i): AX = XB needs at least two of
  // them >= minDeg and about non-parallel axes (singular-value ratio of the unit axes >= 0.1, as the SDK)
  function rotDiversity(G, minDeg = 10) {
    const alpha = [];
    for (let i = 0; i < G.length; i++) for (let j = i + 1; j < G.length; j++) alpha.push(logR(mul(tr(rotOf(G[j])), rotOf(G[i]))));
    const ang = alpha.map((a) => Math.hypot(...a) * D);
    const maxDeg = Math.max(0, ...ang);
    const big = alpha.filter((_, k) => ang[k] >= minDeg);
    if (big.length < 2) return { ok: false, err: 'calib.e.rot_small', maxDeg, ratio: 0 };
    const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (const a of big) { const n = Math.hypot(...a); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) S[i][j] += a[i] * a[j] / (n * n); }
    const vals = symEig(S).vals.sort((x, y) => y - x);
    const ratio = Math.sqrt(Math.max(0, vals[1]) / vals[0]);
    return ratio < 0.1 ? { ok: false, err: 'calib.e.rot_axis', maxDeg, ratio } : { ok: true, maxDeg, ratio };
  }
  // Eye-in-hand, Park & Martin (1994) AX = XB over all pairs (rebot_companion.calibration, mode eye_in_hand):
  // G_i = T_base_gripper, M_i = T_cam_marker, A = G_j^-1 G_i, B = M_j M_i^-1, X = T_gripper_cam
  // ("T_cam_to_gripper"). Quality: T_base_marker_i = G_i X M_i must be constant; residuals are the
  // per-sample position / rotation distance from their mean Y.
  function solveEyeInHand(G, M) {
    const n = G.length;
    if (n < 5) return { err: 'calib.e.few5' };
    const div = rotDiversity(G);
    if (!div.ok) return { err: div.err, div };
    const As = [], Bs = [];
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) { As.push(mul(inv(G[j]), G[i])); Bs.push(mul(M[j], inv(M[i]))); }
    const R = hornR(Bs.map((B) => logR(rotOf(B))), As.map((A) => logR(rotOf(A)))); // log A = R log B
    const N = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
    As.forEach((A, k) => { // (R_A - I) t = R t_B - t_A, normal equations
      const C = rotOf(A).map((r, i) => r.map((v, j) => v - (i === j ? 1 : 0)));
      const RtB = mv(R, posOf(Bs[k])), tA = posOf(A), d = RtB.map((v, i) => v - tA[i]);
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { N[i][j] += C.reduce((s, r) => s + r[i] * r[j], 0); }
      for (let i = 0; i < 3; i++) b[i] += C.reduce((s, r, m) => s + r[i] * d[m], 0);
    });
    const t = solve3(N, b);
    if (!t) return { err: 'calib.e.singular', div };
    const X = make(R, t);
    const Tbm = G.map((Gi, i) => mul(mul(Gi, X), M[i]));
    const pm = [0, 1, 2].map((k) => Tbm.reduce((s, T) => s + T[k][3], 0) / n);
    const Y = make(meanR(Tbm.map(rotOf)), pm);
    const RYt = tr(rotOf(Y));
    const residuals = Tbm.map((T) => Math.hypot(...posOf(T).map((v, k) => v - pm[k])));
    const rotResiduals = Tbm.map((T) => angDeg(mul(RYt, rotOf(T))));
    const rms = (a) => Math.sqrt(a.reduce((s, e) => s + e * e, 0) / a.length);
    return { R, t, X, Y, residuals, rotResiduals, rmse: rms(residuals), rotRmse: rms(rotResiduals),
             maxErr: Math.max(...residuals), maxRotErr: Math.max(...rotResiduals), div };
  }
  const ASSUMPTION = {
    eye_to_hand: 'ArUco marker centre coincides with the TCP; TCP from URDF forward kinematics (J1..J6)',
    eye_in_hand: 'ArUco marker fixed in the base frame; camera rigidly attached to the gripper',
  };
  // calibration file, loadable by rebot_companion.calibration.HandEyeCalibration.load_result / load_samples
  function buildExport({ mode, samples, result: r, markerId }) {
    const eih = mode === 'eye_in_hand';
    let result = null;
    if (r) {
      result = {
        [eih ? 'T_cam_to_gripper' : 'T_cam_to_base']: make(r.R, r.t),
        translation_m: r.t, euler_rpy_deg: rpyDeg(r.R), euler_convention: 'R = Rz(yaw) Ry(pitch) Rx(roll)',
        rmse_m: r.rmse, max_error_m: r.maxErr, residuals_m: r.residuals, samples: r.n, solved_at: r.at,
      };
      if (eih) Object.assign(result, {
        rot_residuals_deg: r.rotResiduals, rot_rmse_deg: r.rotRmse, T_marker_to_base: r.Y,
        residual_definition: 'distance of each T_base_marker = T_base_gripper * T_cam_to_gripper * T_cam_marker from their mean (T_marker_to_base)',
      });
    }
    return {
      version: 1, kind: 'hand_eye_calibration', setup: eih ? 'eye_in_hand' : 'eye_to_hand',
      method: eih ? 'axxb_park_martin' : 'point_registration_horn', assumption: ASSUMPTION[eih ? 'eye_in_hand' : 'eye_to_hand'],
      kinematics: 'URDF reBot-DevArm_gripper.urdf base_link -> end_link, J1..J6',
      units: { length: 'm', angle: 'deg' }, created: new Date().toISOString(), marker_id: markerId,
      result, samples,
    };
  }
  return { mul, inv, make, rotOf, posOf, rpyR, rpyDeg, axisR, expR, logR, angDeg, fk, rotDiversity, solveEyeInHand, buildExport, CHAIN,
    KC_BOUNDS, normCorrection, setCorrection, correction, correctionJoints };
})();
/* CALIB-MATH-END */

/* VIEW-IK-BEGIN */
// Camera-view cartesian frame (7-DOF arm): 6-joint damped-least-squares IK over URDF J1..J6 (model rad)
// on the full chain of CALM.fk, plus the camera-frame motion helpers used by cartTickView().
// Poses are 4x4 T_base_end in the CALM.fk convention (end_link x = tool axis). Camera frame: OpenCV
// (x right, y down, z = optical axis = end_link x).
const VIK = (() => {
  const D = 180 / Math.PI;
  // chain as flat row-major 3x3 + offset; every joint turns about its own +-z
  const CH = CALM.CHAIN.map(([F, axis]) => {
    if (axis && (axis[0] || axis[1])) throw new Error('VIK: joint axis must be +-z');
    return { R: CALM.rotOf(F).flat(), p: CALM.posOf(F), s: axis ? axis[2] : 0 };
  });
  const m3 = (A, B) => [
    A[0] * B[0] + A[1] * B[3] + A[2] * B[6], A[0] * B[1] + A[1] * B[4] + A[2] * B[7], A[0] * B[2] + A[1] * B[5] + A[2] * B[8],
    A[3] * B[0] + A[4] * B[3] + A[5] * B[6], A[3] * B[1] + A[4] * B[4] + A[5] * B[7], A[3] * B[2] + A[4] * B[5] + A[5] * B[8],
    A[6] * B[0] + A[7] * B[3] + A[8] * B[6], A[6] * B[1] + A[7] * B[4] + A[8] * B[7], A[6] * B[2] + A[7] * B[5] + A[8] * B[8]];
  const mv = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const norm = (a) => Math.hypot(a[0], a[1], a[2]);
  const unit = (a) => { const n = norm(a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
  const flat = (T) => [T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2]];
  const nest = (R) => [[R[0], R[1], R[2]], [R[3], R[4], R[5]], [R[6], R[7], R[8]]];
  const T4 = (R, p) => [[R[0], R[1], R[2], p[0]], [R[3], R[4], R[5], p[1]], [R[6], R[7], R[8], p[2]], [0, 0, 0, 1]];
  const posOf = (T) => [T[0][3], T[1][3], T[2][3]];
  const rotAxis = (k, a) => flat(CALM.axisR(k, a));
  // rotation vector of R (flat); fast path away from pi, CALM.logR near it
  function logv(R) {
    const c = (R[0] + R[4] + R[8] - 1) / 2;
    if (c > 0.999999) return [(R[7] - R[5]) / 2, (R[2] - R[6]) / 2, (R[3] - R[1]) / 2];
    if (c < -0.99) return CALM.logR(nest(R));
    const th = Math.acos(c), k = th / (2 * Math.sin(th));
    return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k];
  }
  const expv = (w) => { const th = norm(w); return th < 1e-15 ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : rotAxis(w.map((c) => c / th), th); };
  const mtR = (A, B) => [ // A * B^T
    A[0] * B[0] + A[1] * B[1] + A[2] * B[2], A[0] * B[3] + A[1] * B[4] + A[2] * B[5], A[0] * B[6] + A[1] * B[7] + A[2] * B[8],
    A[3] * B[0] + A[4] * B[1] + A[5] * B[2], A[3] * B[3] + A[4] * B[4] + A[5] * B[5], A[3] * B[6] + A[4] * B[7] + A[5] * B[8],
    A[6] * B[0] + A[7] * B[1] + A[8] * B[2], A[6] * B[3] + A[7] * B[4] + A[8] * B[5], A[6] * B[6] + A[7] * B[7] + A[8] * B[8]];

  // end pose + world joint axes / origins for q (model J1..J6); kc: kinematic correction (CALM.normCorrection
  // shape, default CALM's current one). g[j] = d(joint angle)/d(q_j) = 1 + scale_j (Jacobian column factor).
  function frames(q, kc = CALM.correction()) {
    let R = [1, 0, 0, 0, 1, 0, 0, 0, 1], p = [0, 0, 0], i = 0;
    const a = [], o = [], g = [];
    for (const L of CH) {
      const d = mv(R, L.p);
      p = [p[0] + d[0], p[1] + d[1], p[2] + d[2]];
      R = m3(R, L.R);
      if (L.s) {
        const k = kc && kc[i];
        let qi = Number(q[i++]) || 0;
        if (k) {
          if (k.R) {
            const e = mv(R, k.d);
            p = [p[0] + e[0], p[1] + e[1], p[2] + e[2]];
            R = m3(R, k.R.flat());
          }
          qi = qi * (1 + k.sc) + k.off;
        }
        g.push(k ? 1 + k.sc : 1);
        a.push([R[2] * L.s, R[5] * L.s, R[8] * L.s]); o.push(p);
        const th = L.s * qi, c = Math.cos(th), s = Math.sin(th);
        R = [R[0] * c + R[1] * s, -R[0] * s + R[1] * c, R[2], R[3] * c + R[4] * s, -R[3] * s + R[4] * c, R[5], R[6] * c + R[7] * s, -R[6] * s + R[7] * c, R[8]];
      }
    }
    return { R, p, a, o, g };
  }
  const fk = (q) => { const f = frames(q); return T4(f.R, f.p); };

  // 6x6 SPD solve (Cholesky), A flat row-major; returns null if not positive definite
  function chol6(A, b) {
    const L = new Float64Array(36);
    for (let i = 0; i < 6; i++) for (let j = 0; j <= i; j++) {
      let s = A[i * 6 + j];
      for (let k = 0; k < j; k++) s -= L[i * 6 + k] * L[j * 6 + k];
      if (i === j) { if (!(s > 1e-18)) return null; L[i * 6 + i] = Math.sqrt(s); } else L[i * 6 + j] = s / L[j * 6 + j];
    }
    const y = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 6; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * 6 + k] * y[k]; y[i] = s / L[i * 6 + i]; }
    for (let i = 5; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < 6; k++) s -= L[k * 6 + i] * y[k]; y[i] = s / L[i * 6 + i]; }
    return y;
  }
  const PAR = { w: 0.2, lamMin: 0.01, lamMax: 0.05, iters: 10, tolPos: 1e-3, tolRot: 0.5 / D, maxStep: 0.5 };
  // error twist [dp; w * log(Rg Rc^T)] and its parts
  function poseErr(f, Rg, pg, w) {
    const dp = [pg[0] - f.p[0], pg[1] - f.p[1], pg[2] - f.p[2]], dr = logv(mtR(Rg, f.R));
    return { e: [dp[0], dp[1], dp[2], w * dr[0], w * dr[1], w * dr[2]], ep: norm(dp), er: norm(dr) };
  }
  // Tg: 4x4 goal; q0: model J1..J6 start (previous solution); opt.lo / opt.hi: model limits (6), widened to
  // contain q0 so a joint already past its margin may stay but not go further.
  // returns { ok, q, ep (m), er (rad), it } or { ok: false, err: 'pose', ... }; never throws on bad targets
  function solve(Tg, q0, opt = {}) {
    const w = opt.w ?? PAR.w, iters = opt.iters ?? PAR.iters;
    const lo = [], hi = [];
    for (let i = 0; i < 6; i++) {
      const l = opt.lo ? opt.lo[i] : -Infinity, h = opt.hi ? opt.hi[i] : Infinity;
      lo.push(Math.min(l, q0[i])); hi.push(Math.max(h, q0[i]));
    }
    const Rg = flat(Tg), pg = posOf(Tg);
    let q = q0.slice(0, 6), f = frames(q), E = poseErr(f, Rg, pg, w), lam = PAR.lamMin, it = 0;
    if (!(E.ep < Infinity && E.er < Infinity)) return { ok: false, err: 'pose', q: q0.slice(0, 6), ep: Infinity, er: Infinity, it };
    // iterate to a 10x tighter tolerance while iterations remain (no tick-to-tick jitter inside the
    // acceptance band); accept at tolPos / tolRot
    while (it < iters && !(E.ep <= PAR.tolPos / 10 && E.er <= PAR.tolRot / 10)) {
      it++;
      // geometric Jacobian, angular rows scaled by w (6 x 6, column per joint)
      const J = [];
      for (let j = 0; j < 6; j++) {
        const a = f.a[j], v = cross(a, [f.p[0] - f.o[j][0], f.p[1] - f.o[j][1], f.p[2] - f.o[j][2]]), gj = f.g[j];
        J.push(gj === 1 ? [v[0], v[1], v[2], w * a[0], w * a[1], w * a[2]] : [gj * v[0], gj * v[1], gj * v[2], gj * w * a[0], gj * w * a[1], gj * w * a[2]]);
      }
      const A = new Float64Array(36);
      for (let r = 0; r < 6; r++) for (let c = r; c < 6; c++) {
        let s = 0;
        for (let j = 0; j < 6; j++) s += J[j][r] * J[j][c];
        A[r * 6 + c] = A[c * 6 + r] = s;
      }
      for (let r = 0; r < 6; r++) A[r * 6 + r] += lam * lam;
      const y = chol6(A, E.e);
      if (!y) break;
      const dq = J.map((col) => col.reduce((s, v, r) => s + v * y[r], 0));
      const big = Math.max(...dq.map(Math.abs));
      const k = big > PAR.maxStep ? PAR.maxStep / big : 1;
      const qn = q.map((x, i) => Math.min(hi[i], Math.max(lo[i], x + k * dq[i])));
      const fn = frames(qn), En = poseErr(fn, Rg, pg, w);
      if (En.ep + w * En.er < E.ep + w * E.er) { q = qn; f = fn; E = En; lam = PAR.lamMin; }
      else if (lam < PAR.lamMax) lam = PAR.lamMax; // adaptive damping: retry the step with heavier damping
      else { q = qn; f = fn; E = En; }              // still worse with lamMax: take it (clamped) and continue
    }
    const ok = E.ep <= PAR.tolPos && E.er <= PAR.tolRot;
    return ok ? { ok, q, ep: E.ep, er: E.er, it } : { ok, err: 'pose', q, ep: E.ep, er: E.er, it };
  }

  // camera mount R_end_cam (nested 3x3): z_cam = end_link x; y_cam = flip * J5 axis seen in end_link at
  // the model angles qRef (default: the B601-DM rest angle of J6, motor pi/2 = model 0; x part removed);
  // x_cam = y_cam x z_cam; then rotDeg about the optical axis. armd kinematics::cam_mount_with.
  // qEndCam [w, x, y, z] (armd camera_mount.q_end_cam, calibrated): R_end_cam itself, replacing flip / rotDeg.
  function camMount(flip, rotDeg, qRef = [0, 0, 0, 0, 0, 0], qEndCam = null) {
    if (Array.isArray(qEndCam) && qEndCam.length === 4 && qEndCam.every(Number.isFinite)) {
      const n = Math.hypot(...qEndCam) || 1, [w, x, y, z] = qEndCam.map((v) => v / n);
      return nest([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]);
    }
    const f = frames(qRef);
    const aEnd = mv([f.R[0], f.R[3], f.R[6], f.R[1], f.R[4], f.R[7], f.R[2], f.R[5], f.R[8]], f.a[4]); // R^T a
    const y = unit([0, aEnd[1] * (flip ? -1 : 1), aEnd[2] * (flip ? -1 : 1)]), z = [1, 0, 0], x = cross(y, z);
    let R = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
    if (rotDeg) R = m3(R, rotAxis([0, 0, 1], rotDeg / D));
    return nest(R);
  }
  // camera axes in the base frame for end pose T: { x, y, z } unit vectors
  function camAxes(T, Rec) {
    const R = m3(flat(T), flat(Rec));
    return { x: [R[0], R[3], R[6]], y: [R[1], R[4], R[7]], z: [R[2], R[5], R[8]], R };
  }
  // auto-level: roll about the optical axis so x_cam is horizontal and y_cam points down (y_cam . Z <= 0);
  // skipped when the optical axis is within acos(levelCos) of vertical
  function level(T, Rec, levelCos) {
    const c = camAxes(T, Rec);
    if (Math.abs(c.z[2]) >= levelCos) return T;
    const x = unit(cross(c.z, [0, 0, 1])), y = cross(c.z, x);
    const Rc = [x[0], y[0], c.z[0], x[1], y[1], c.z[1], x[2], y[2], c.z[2]];
    return T4(mtR(Rc, flat(Rec)), posOf(T)); // R_end = R_cam R_ec^T
  }
  // integrate operator input v = { x: forward, y: left (+), z: world up, p: look down (+), r: look right (+) }
  // over dt: translation in the camera frame (+ world Z), unit-ball normalised x vl; pitch about x_cam and
  // look-around about world Z (both centred on the end_link origin) x wl; then auto-level. A pure
  // translation keeps the orientation as it is (no auto-level), so it never re-rolls the camera.
  function integrate(T, v, dt, o) {
    const c = camAxes(T, o.Rec);
    const d = [0, 1, 2].map((k) => -v.y * c.x[k] + v.x * c.z[k]);
    d[2] += v.z;
    const dn = norm(d), ds = dn > 1 ? o.vl * dt / dn : o.vl * dt;
    const p = posOf(T).map((x, k) => x + d[k] * ds);
    let wp = -(v.p || 0), wr = -(v.r || 0);
    const wn = Math.hypot(wp, wr);
    if (wn > 1) { wp /= wn; wr /= wn; }
    let R = flat(T);
    if (wp) R = m3(rotAxis(c.x, wp * o.wl * dt), R);
    if (wr) R = m3(rotAxis([0, 0, 1], wr * o.wl * dt), R);
    return wp || wr ? level(T4(R, p), o.Rec, o.levelCos) : T4(R, p);
  }
  // position (m) and rotation (rad) difference B relative to A
  function diff(A, B) {
    const pa = posOf(A), pb = posOf(B);
    const dp = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]], dr = logv(mtR(flat(B), flat(A)));
    return { dp, dr, d: norm(dp), a: norm(dr) };
  }
  // move from A toward B by at most dmax (m) / amax (rad); returns B itself when it is within reach
  function approach(A, B, dmax, amax) {
    const e = diff(A, B);
    if (e.d <= dmax && e.a <= amax) return B;
    const fp = e.d > dmax ? dmax / e.d : 1, fr = e.a > amax ? amax / e.a : 1;
    const R = m3(expv(e.dr.map((x) => x * fr)), flat(A));
    return T4(R, posOf(A).map((x, k) => x + e.dp[k] * fp));
  }
  // anti-windup: keep goal B within dmax / amax of A
  function clampGoal(A, B, dmax, amax) {
    const e = diff(A, B);
    if (e.d <= dmax && e.a <= amax) return B;
    return approach(A, B, dmax, amax);
  }
  return { fk, frames, solve, camMount, camAxes, level, integrate, diff, approach, clampGoal, PAR };
})();
/* VIEW-IK-END */

// markers usable in the current mode (eye_in_hand needs the full 6-D pose); prefers the locked marker ID.
// Returns { m, other }: other = a usable marker with a different ID than the locked one.
function calPick(list) {
  const ms = (list || []).filter((d) => d.aruco && d.aruco.t && (!calEih() || d.aruco.R));
  if (CAL.markerId == null) return { m: ms[0] || null, other: null };
  return { m: ms.find((d) => String(d.aruco.id) === String(CAL.markerId)) || null, other: ms[0] || null };
}
// freshest detections -> calPick result plus the frame age (ms)
// with freshOnly=true, ignores S.detHoldUntil and enforces arrival within 500ms
function calMarker({ freshOnly = false } = {}) {
  let age = S.detLastReceivedAt ? performance.now() - S.detLastReceivedAt : Infinity;
  // device-reported frame age (at receive) + time since receive beats the browser receive time alone
  if (freshOnly && S.detDevAge && S.detDevAge.rx === S.detLastReceivedAt) age = S.detDevAge.age + (performance.now() - S.detDevAge.rx);
  const none = { m: null, other: null, age };
  if (freshOnly) {
    if (age > 500) return none;
  } else {
    if (!(performance.now() < (S.detHoldUntil || 0)) && age > 600) return none;
  }
  return { ...calPick(S.detections), age };
}
function calRender() {
  const n = CAL.samples.length, eih = calEih();
  for (const b of $('calib-mode').children) {
    const on = b.dataset.mode === CAL.mode;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
  setText($('calib-hint'), t(eih ? 'calib.hint.eih' : 'calib.hint'));
  setText($('calib-count'), t('calib.count', { n, total: CAL.target }));
  $('calib-bar').style.width = Math.min(100, n / CAL.target * 100) + '%';
  const list = $('calib-list');
  list.textContent = '';
  const r = CAL.result;
  CAL.samples.forEach((sm, i) => {
    const li = document.createElement('li');
    const res = r && r.residuals ? r.residuals[i] : null, rot = r && r.rotResiduals ? r.rotResiduals[i] : null;
    if ((res != null && r.rmse > 0 && res > 2.5 * r.rmse && res > 0.005) || (rot != null && r.rotRmse > 0 && rot > 2.5 * r.rotRmse && rot > 1)) li.className = 'bad';
    const b = document.createElement('b'); b.textContent = '#' + (i + 1);
    const a = document.createElement('span');
    a.textContent = 'TCP ' + sm.p_base.map((v) => (v * 1000).toFixed(0)).join(',') + ' · Zc ' + sm.p_cam[2].toFixed(3) + 'm'
      + (res != null ? ' · ' + (res * 1000).toFixed(1) + 'mm' : '') + (rot != null ? ' ' + rot.toFixed(2) + '°' : '');
    const x = document.createElement('button');
    x.type = 'button'; x.className = 'btn'; x.style.cssText = 'min-height:40px;padding:0 12px;font-size:12px';
    x.textContent = t('calib.del');
    x.addEventListener('click', () => { CAL.samples.splice(i, 1); CAL.result = null; if (!CAL.samples.length) CAL.markerId = null; calSave(); calRender(); });
    li.append(b, a, x);
    list.appendChild(li);
  });
  const isSoArm = isSoArmProfile() || (S.armDof != null && S.armDof !== 6);
  $('calib-solve').disabled = n < calMin() || CAL.busy || isSoArm;
  $('calib-export').disabled = n === 0;
  $('calib-cap').disabled = CAL.busy || isSoArm;
  if (isSoArm) {
    setText($('calib-cap'), 'Hand-Eye FK calibration supports 6-arm-DOF B601 only');
  } else {
    setText($('calib-cap'), t(CAL.busy ? 'calib.cap.busy' : 'calib.cap'));
  }
  const rb = $('calib-reset'), armed = Date.now() < CAL.resetArmedUntil;
  rb.classList.toggle('armed', armed);
  setText(rb, t(armed ? 'calib.reset.confirm' : 'calib.reset'));
  rb.disabled = n === 0 && !CAL.result;

  const box = $('calib-result');
  box.hidden = !r;
  if (r) {
    const warn = calWarn(r);
    box.className = 'calib-result mono' + (warn ? ' warn' : '');
    box.textContent = '';
    const row = (k, v) => { const d = document.createElement('div'); d.className = 'row'; const a = document.createElement('span'); a.textContent = k; const b = document.createElement('span'); b.textContent = v; d.append(a, b); box.appendChild(d); };
    row(t('calib.r.frame'), eih ? 'T_cam_to_gripper' : 'T_cam_to_base');
    row(t('calib.r.t'), r.t.map((v) => (v * 1000).toFixed(1)).join(', '));
    row(t('calib.r.e'), r.rpy.map((v) => v.toFixed(2)).join(', '));
    if (eih) {
      row(t('calib.r.rmse_pos'), (r.rmse * 1000).toFixed(2) + ' mm');
      row(t('calib.r.rmse_rot'), r.rotRmse.toFixed(2) + '°');
      row(t('calib.r.max'), (r.maxErr * 1000).toFixed(2) + ' mm · ' + r.maxRotErr.toFixed(2) + '°');
    } else {
      row(t('calib.r.rmse'), (r.rmse * 1000).toFixed(2) + ' mm');
      row(t('calib.r.max'), (r.maxErr * 1000).toFixed(2) + ' mm');
    }
    row(t('calib.r.n'), String(r.n) + (r.n < CAL.target ? ' · ' + t('calib.need_more', { total: CAL.target }) : ''));
    if (warn) { const p2 = document.createElement('div'); p2.textContent = t(eih ? 'calib.rmse.warn.eih' : 'calib.rmse.warn'); box.appendChild(p2); }
  }
  calLive();
}
function calLive() {
  const el = $('calib-live'), { m } = calMarker();
  el.classList.toggle('ok', !!m);
  setText(el, m ? t('calib.live.ok', { id: m.aruco.id ?? '?', z: m.aruco.t[2].toFixed(3) }) : t(calEih() ? 'calib.live.none.eih' : 'calib.live.none'));
}
async function calCapture() {
  if (CAL.busy) return;
  if (isSoArmProfile() || (S.armDof != null && S.armDof !== 6)) {
    toast('Hand-Eye FK calibration currently supports 6-arm-DOF B601 only', 'warn');
    return;
  }
  if (!isLive() || !S.obs || !Array.isArray(S.obs.positions) || S.obs.positions.length < 4) { toast(t('calib.e.offline'), 'warn'); return; }
  if (Array.isArray(S.obs.velocities) && S.obs.velocities.some((v) => Math.abs(v) > 0.02)) { toast(t('calib.e.moving'), 'warn'); return; }
  const eih = calEih();
  CAL.busy = true; calRender();
  try {
    const pos = S.obs.positions.slice(); // snapshot before the (1-4 s) detection so frame and pose belong together
    let pk = calMarker({ freshOnly: true }), source = 'stream';
    if (!pk.m && DET.avail) {
      const parsed = await triggerDet({ quiet: true, pipelines: ['aruco'] });
      if (Array.isArray(parsed)) { pk = { ...calPick(parsed), age: null }; source = 'snapshot'; }
    }
    const m = pk.m;
    if (!m) {
      if (pk.other) toast(t('calib.e.otherid', { id: pk.other.aruco.id ?? '?', lock: CAL.markerId }), 'warn');
      else toast(t(eih ? 'calib.e.nomarker6' : 'calib.e.nomarker'), 'warn');
      return;
    }
    // Re-verify telemetry validity and velocity after async detection trigger
    if (!isLive() || !S.obs || !Array.isArray(S.obs.positions) || S.obs.positions.length < 4) { toast(t('calib.e.offline'), 'warn'); return; }
    if (Array.isArray(S.obs.velocities) && S.obs.velocities.some((v) => Math.abs(v) > 0.02)) { toast(t('calib.e.moving'), 'warn'); return; }
    if (pos.slice(0, 6).some((v, i) => !Number.isFinite(S.obs.positions[i]) || Math.abs(S.obs.positions[i] - v) > 0.003)) { toast(t('calib.e.shifted'), 'warn'); return; }
    const G = calGripper(pos), p_base = CALM.posOf(G);
    const near = (sm) => Math.hypot(...sm.p_base.map((v, k) => v - p_base[k])) < 0.01;
    // eye_in_hand: a new pose may share the position if the view angle differs by >= 3 deg
    const dup = CAL.samples.findIndex((sm) => near(sm) && (!eih || !sm.T_base_gripper || CALM.angDeg(CALM.mul(CALM.rotOf(CALM.inv(sm.T_base_gripper)), CALM.rotOf(G))) < 3));
    if (dup >= 0) { toast(t(eih ? 'calib.e.dup.eih' : 'calib.e.dup', { i: dup + 1 }), 'warn'); return; }
    if (CAL.markerId == null && m.aruco.id != null) CAL.markerId = m.aruco.id;
    CAL.samples.push({
      joints: pos, p_base, T_base_gripper: G,
      marker_id: m.aruco.id, p_cam: m.aruco.t.slice(),
      R_cam: m.aruco.R, rpy_cam_deg: m.aruco.rpy, T_cam_marker: m.aruco.R ? CALM.make(m.aruco.R, m.aruco.t) : null, corners: m.aruco.corners,
      frame_source: source, frame_age_ms: pk.age != null && Number.isFinite(pk.age) ? Math.round(pk.age) : null,
      ts: new Date().toISOString(),
    });
    CAL.result = null; calSave();
    buzz(15);
    toast(t('calib.ok.cap', { n: CAL.samples.length }));
  } finally { CAL.busy = false; calRender(); }
}
function calSolve() {
  const n = CAL.samples.length, at = new Date().toISOString();
  if (calEih()) {
    const r = CALM.solveEyeInHand(CAL.samples.map((sm) => sm.T_base_gripper), CAL.samples.map((sm) => sm.T_cam_marker));
    if (r.err) { toast(t(r.err, { deg: r.div ? r.div.maxDeg.toFixed(1) : '0' }), 'warn'); return; }
    CAL.result = { ...r, rpy: CALM.rpyDeg(r.R), n, at };
    calRender();
    toast(t('calib.ok.solve.eih', { rmse: (r.rmse * 1000).toFixed(2), rot: r.rotRmse.toFixed(2) }), calWarn(CAL.result) ? 'warn' : '');
    return;
  }
  const r = solveRigid(CAL.samples.map((sm) => sm.p_cam), CAL.samples.map((sm) => sm.p_base));
  if (r.err) { toast(t(r.err), 'warn'); return; }
  CAL.result = { ...r, rpy: CALM.rpyDeg(r.R), n, at };
  calRender();
  toast(t('calib.ok.solve', { rmse: (r.rmse * 1000).toFixed(2) }), calWarn(CAL.result) ? 'warn' : '');
}
function calExport() {
  const out = CALM.buildExport({ mode: CAL.mode, samples: CAL.samples, result: CAL.result, markerId: CAL.markerId });
  const blob = new Blob([JSON.stringify(out, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'rebot_hand_eye_' + (calEih() ? 'eih_' : '') + new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19) + '.json';
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
// switching setups invalidates the samples: with samples, the first tap warns, a second tap within 3 s clears
function calSetMode(mode) {
  if (!CAL_MODES.includes(mode) || mode === CAL.mode || CAL.busy) return;
  if (CAL.samples.length || CAL.result) {
    if (Date.now() >= CAL.modeArmedUntil) { CAL.modeArmedUntil = Date.now() + 3000; toast(t('calib.mode.confirm', { n: CAL.samples.length }), 'warn'); return; }
    CAL.samples = []; CAL.result = null; CAL.markerId = null;
  }
  CAL.modeArmedUntil = 0; CAL.mode = mode; store('rebot.calib.mode', mode); calSave(); calRender();
}
function calReset() {
  // two-tap confirm against accidental touches
  if (Date.now() >= CAL.resetArmedUntil) { CAL.resetArmedUntil = Date.now() + 3000; calRender(); setTimeout(calRender, 3050); return; }
  CAL.resetArmedUntil = 0; CAL.samples = []; CAL.result = null; CAL.markerId = null; calSave(); calRender();
}
function calOpen(open) {
  const m = $('calib');
  m.hidden = !open;
  clearInterval(CAL.liveTimer); CAL.liveTimer = null;
  if (open) { calRender(); CAL.liveTimer = setInterval(calLive, 250); $('calib-cap').focus({ preventScroll: true }); }
  else $('btn-calib').focus({ preventScroll: true });
  if (open !== calOpen.was) { calOpen.was = open; calVisionMode(open); }
}
function initCalib() {
  $('btn-calib').addEventListener('click', () => cwOpen(true)); // the wizard; its "advanced" button opens this drawer
  $('calib-close').addEventListener('click', () => calOpen(false));
  $('calib').addEventListener('click', (e) => { if (e.target === e.currentTarget) calOpen(false); });
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('calib').hidden) calOpen(false); });
  $('calib-cap').addEventListener('click', calCapture);
  $('calib-solve').addEventListener('click', calSolve);
  $('calib-export').addEventListener('click', calExport);
  $('calib-reset').addEventListener('click', calReset);
  $('calib-mode').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) calSetMode(b.dataset.mode); });
  i18nHooks.push(() => { if (!$('calib').hidden) calRender(); });
}

