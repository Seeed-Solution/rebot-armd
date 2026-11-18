// Camera calibration wizard math (cockpit, served by armd as /assets/calib_wizard.js). ES module, no
// dependencies, no DOM: scripts/calib_wizard_selftest.mjs imports it in Node.
//
// One capture run serves intrinsics, hand-eye and table height: a printed ArUco GridBoard lies on the
// table, the arm visits generated poses, each pose gives the board's marker corners (640x360 pixels).
//   intrinsics  homography per view (normalised DLT) -> Zhang closed form -> LM over fx fy cx cy k1 k2
//               and per-view board poses (p1 p2 k3 fixed 0)
//   board pose  planar PnP with the new intrinsics (homography init + LM)
//   hand-eye    AX = XB with the cockpit's CALM.solveEyeInHand (passed in) on (FK pose, board pose)
//   table       plane through the board poses in base (flatness gates)
//   joint       one LM over all corners: intrinsics, X = T_end_cam, T_base_board (FK fixed); its values
//               are applied, table_z_m = refined board z - board thickness
// Conventions: 4x4 transforms are row-major nested arrays (as CALM); T_a_b maps frame-b coordinates
// into frame a. Camera frame OpenCV (x right, y down, z optical axis). Pixels at 640x360.

export const IMAGE = Object.freeze({ width: 640, height: 360 });
// nominal intrinsics for the bootstrap capture (reCamera at 640x360)
export const NOMINAL = Object.freeze({ fx: 824, fy: 824, cx: 320, cy: 180, k1: 0, k2: 0 });

// Gates (DESIGN W3, W6, X1). RMS values are per-corner distances in pixels.
export const GATES = Object.freeze({
  minViews: 15,          // accepted views
  minMarkers: 6,         // markers per view (fewer: view not accepted)
  rmsPx: 0.5,            // global reprojection RMS
  viewRmsPx: 1.0,        // per view; worse views are dropped once and the solve repeated
  heldOutRmsPx: 1.0,     // every 5th view left out of the solve, its corners predicted (pose by PnP)
  sigmaRel: 0.02,        // 1-sigma of fx, fy, cx, cy relative to the value
  handEyeRmsM: 0.003,    // hand-eye translation RMS (spread of T_base_board over the views)
  handEyeRotDeg: 1.0,    // hand-eye rotation RMS
  planeTiltDeg: 5,       // board normal from vertical
  planeSpreadM: 0.004,   // max - min of the per-view board z
});

// ---------------------------------------------------------------- small linear algebra
const D = 180 / Math.PI;
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm3 = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => { const n = norm3(a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add3 = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scl3 = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const mm = (A, B) => A.map((r) => B[0].map((_, c) => r.reduce((s, v, k) => s + v * B[k][c], 0)));
const tr = (A) => A[0].map((_, c) => A.map((r) => r[c]));
const mv3 = (R, v) => [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2], R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2], R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]];
export const rotOf = (T) => T.slice(0, 3).map((r) => r.slice(0, 3));
export const posOf = (T) => [T[0][3], T[1][3], T[2][3]];
export const make = (R, t) => [[...R[0], t[0]], [...R[1], t[1]], [...R[2], t[2]], [0, 0, 0, 1]];
export const mul = mm;
export const inv = (T) => { const Rt = tr(rotOf(T)), p = mv3(Rt, posOf(T)); return make(Rt, [-p[0], -p[1], -p[2]]); };
const apply = (T, p) => [T[0][0] * p[0] + T[0][1] * p[1] + T[0][2] * p[2] + T[0][3], T[1][0] * p[0] + T[1][1] * p[1] + T[1][2] * p[2] + T[1][3], T[2][0] * p[0] + T[2][1] * p[1] + T[2][2] * p[2] + T[2][3]];
const I3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

// symmetric Jacobi eigen-decomposition -> { vals, vecs } (eigenvector i = column i)
export function symEig(A) {
  const n = A.length, a = A.map((r) => r.slice()), V = A.map((_, i) => A.map((__, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0, diag = 0;
    for (let i = 0; i < n; i++) { diag += a[i][i] * a[i][i]; for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j]; }
    if (off <= 1e-30 * diag || off < 1e-300) break;
    for (let p = 0; p < n - 1; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
      for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
    }
  }
  return { vals: a.map((r, i) => r[i]), vecs: V };
}
const minEigVec = (A) => { const { vals, vecs } = symEig(A); let k = 0; for (let i = 1; i < vals.length; i++) if (vals[i] < vals[k]) k = i; return vecs.map((r) => r[k]); };
const maxEigVec = (A) => { const { vals, vecs } = symEig(A); let k = 0; for (let i = 1; i < vals.length; i++) if (vals[i] > vals[k]) k = i; return vecs.map((r) => r[k]); };

// Cholesky of a flat n x n SPD matrix; null when not positive definite
function chol(A, n) {
  const L = new Float64Array(n * n);
  for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
    let s = A[i * n + j];
    for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
    if (i === j) { if (!(s > 0)) return null; L[i * n + i] = Math.sqrt(s); } else L[i * n + j] = s / L[j * n + j];
  }
  return L;
}
function cholSolve(L, n, b) {
  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
  for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * y[k]; y[i] = s / L[i * n + i]; }
  return y;
}

// ---------------------------------------------------------------- rotations
export function rodrigues(r) {
  const th = Math.hypot(r[0], r[1], r[2]);
  if (th < 1e-12) return [[1, -r[2], r[1]], [r[2], 1, -r[0]], [-r[1], r[0], 1]];
  const [x, y, z] = [r[0] / th, r[1] / th, r[2] / th], c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]];
}
// rotation matrix -> unit quaternion [w, x, y, z], w >= 0
export function quatOf(R) {
  const s0 = R[0][0] + R[1][1] + R[2][2];
  let q;
  if (s0 > 0) { const s = Math.sqrt(s0 + 1) * 2; q = [s / 4, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s]; }
  else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) { const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; q = [(R[2][1] - R[1][2]) / s, s / 4, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s]; }
  else if (R[1][1] > R[2][2]) { const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; q = [(R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, s / 4, (R[1][2] + R[2][1]) / s]; }
  else { const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; q = [(R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, s / 4]; }
  const n = Math.hypot(...q) * (q[0] < 0 ? -1 : 1);
  return q.map((v) => v / n);
}
export function quatR([w, x, y, z]) {
  const n = Math.hypot(w, x, y, z) || 1; w /= n; x /= n; y /= n; z /= n;
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
}
export function logRot(R) {
  const [w, x, y, z] = quatOf(R), s = Math.hypot(x, y, z);
  if (s < 1e-12) return [2 * x, 2 * y, 2 * z];
  const th = 2 * Math.atan2(s, w);
  return [x / s * th, y / s * th, z / s * th];
}
export const angDeg = (R) => norm3(logRot(R)) * D;
// angle between two rotations (deg)
export const rotDist = (A, B) => angDeg(mm(tr(A), B));
// rotation nearest to M (Frobenius), Horn's quaternion method on S = M^T
export function nearestRot(M) {
  const S = tr(M);
  const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = S;
  const N = [[Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx], [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy], [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz]];
  return quatR(maxEigVec(N));
}
export const rpyR = (r, p, y) => {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]];
};
export const rpyDeg = (R) => [Math.atan2(R[2][1], R[2][2]) * D, Math.asin(Math.min(1, Math.max(-1, -R[2][0]))) * D, Math.atan2(R[1][0], R[0][0]) * D];

// ---------------------------------------------------------------- camera model
// intrinsics K = { fx, fy, cx, cy, k1, k2 }; R, t = T_cam_obj
export function project(K, R, t, X) {
  const xc = R[0][0] * X[0] + R[0][1] * X[1] + R[0][2] * X[2] + t[0];
  const yc = R[1][0] * X[0] + R[1][1] * X[1] + R[1][2] * X[2] + t[1];
  const zc = R[2][0] * X[0] + R[2][1] * X[1] + R[2][2] * X[2] + t[2];
  const x = xc / zc, y = yc / zc, r2 = x * x + y * y, d = 1 + (K.k1 || 0) * r2 + (K.k2 || 0) * r2 * r2;
  return [K.fx * x * d + K.cx, K.fy * y * d + K.cy, zc];
}
// distorted pixel -> ideal normalised coordinates (fixed-point inversion as cv::undistortPoints)
export function undistort(K, u, v) {
  const x0 = (u - K.cx) / K.fx, y0 = (v - K.cy) / K.fy;
  let x = x0, y = y0;
  for (let i = 0; i < 20; i++) { const r2 = x * x + y * y, d = 1 + (K.k1 || 0) * r2 + (K.k2 || 0) * r2 * r2; x = x0 / d; y = y0 / d; }
  return [x, y];
}

// ---------------------------------------------------------------- board (DESIGN W2)
// ArUco DICT_4X4_50 ids 0..11, 4x4 inner bits row-major from the top-left, MSB first, 1 = white
// (from OpenCV 4.x cv2.aruco.getPredefinedDictionary(DICT_4X4_50).generateImageMarker).
export const DICT_4X4_50 = [46386, 3994, 13101, 39238, 21662, 31181, 40494, 50418, 65242, 53078, 63889, 4519];
/* GridBoard, 4 columns x 3 rows, printed landscape (as the 16:9 image); the simulator's virtual board
   (sim/mujoco/src/rebot_sim/aruco.py) has the same layout. id = row * 4 + column, row 0 at the top of
   the page, column 0 at the left. Default marker side 25 mm, gap 0.2 x side (5 mm): about 115 x 85 mm.
   Board frame: origin at the board centre, x to the right of the page, y down the page, z into the
   table (the "up" arrow points to -y). Marker corners in rebot-vision's (AprilTag) order k0
   bottom-left, k1 bottom-right, k2 top-right, k3 top-left of the printed marker; the solver also
   detects other cyclic orders (detectCornerOrder). The gap scales with the measured marker side. */
export const BOARD_DEFAULT = Object.freeze({ cols: 4, rows: 3, markerSide: 0.025, gapRatio: 0.2 });
export function makeBoard({ markerSide = BOARD_DEFAULT.markerSide, gapRatio = BOARD_DEFAULT.gapRatio, cols = BOARD_DEFAULT.cols, rows = BOARD_DEFAULT.rows } = {}) {
  const L = markerSide, gap = L * gapRatio, pitch = L + gap, h = L / 2;
  const markers = [];
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const id = r * cols + c, cx = (c - (cols - 1) / 2) * pitch, cy = (r - (rows - 1) / 2) * pitch;
    markers.push({ id, row: r, col: c, center: [cx, cy, 0], corners: [[cx - h, cy + h, 0], [cx + h, cy + h, 0], [cx + h, cy - h, 0], [cx - h, cy - h, 0]] });
  }
  return { cols, rows, markerSide: L, gap, pitch, width: cols * L + (cols - 1) * gap, height: rows * L + (rows - 1) * gap, markers, byId: new Map(markers.map((m) => [m.id, m])) };
}
// printable SVG at 1:1 scale (mm units): title, an "up" arrow (page top = board -y), the markers with
// their ids, a 100 mm ruler under the board
export function boardSvg(board, { title = 'reBot calibration board' } = {}) {
  const mmL = board.markerSide * 1000, mmG = board.gap * 1000, W = board.width * 1000, H = board.height * 1000;
  const margin = 10, pageW = Math.max(W, 110) + 2 * margin, bx = (pageW - W) / 2, by = margin + 28, cell = mmL / 6;
  const ry = by + H + 12, pageH = ry + 12, fs = Math.max(2, Math.min(mmG, 8) * 0.6);
  let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${pageW}mm" height="${pageH}mm" viewBox="0 0 ${pageW} ${pageH}">`;
  s += `<rect width="${pageW}" height="${pageH}" fill="#fff"/>`;
  s += `<text x="${margin}" y="${margin + 4}" font-size="4.5" font-family="sans-serif">${title}</text>`;
  s += `<text x="${margin}" y="${margin + 9.5}" font-size="3" font-family="sans-serif">ArUco 4x4_50 ids 0-${board.markers.length - 1}, ${board.cols} x ${board.rows}, marker ${mmL.toFixed(1)} mm, gap ${mmG.toFixed(1)} mm</text>`;
  const ax = pageW / 2;
  s += `<path d="M ${ax} ${margin + 12} l -5 8 h 3.5 v 6 h 3 v -6 h 3.5 z" fill="#000"/>`;
  s += `<text x="${ax + 7}" y="${margin + 19}" font-size="3" font-family="sans-serif">up</text>`;
  for (const m of board.markers) {
    const x0 = bx + m.col * (mmL + mmG), y0 = by + m.row * (mmL + mmG), code = DICT_4X4_50[m.id];
    s += `<rect x="${x0}" y="${y0}" width="${mmL}" height="${mmL}" fill="#000"/>`;
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
      if ((code >> (15 - (r * 4 + c))) & 1) s += `<rect x="${(x0 + (c + 1) * cell).toFixed(3)}" y="${(y0 + (r + 1) * cell).toFixed(3)}" width="${cell.toFixed(3)}" height="${cell.toFixed(3)}" fill="#fff"/>`;
    }
    s += `<text x="${x0 + mmL / 2}" y="${y0 + mmL + fs * 1.1}" font-size="${fs}" text-anchor="middle" font-family="sans-serif" fill="#666">${m.id}</text>`;
  }
  // 100 mm ruler
  const rx = (pageW - 100) / 2;
  s += `<path d="M ${rx} ${ry} h 100" stroke="#000" stroke-width="0.4"/>`;
  for (let k = 0; k <= 10; k++) s += `<path d="M ${rx + 10 * k} ${ry} v ${k % 5 ? -2.5 : -4}" stroke="#000" stroke-width="0.3"/>`;
  s += `<text x="${rx}" y="${ry + 5}" font-size="3" font-family="sans-serif">100 mm: measure after printing (no page scaling)</text>`;
  return s + '</svg>';
}

// ---------------------------------------------------------------- correspondences
// A view: { markers: [{ id, corners: [[u, v] x 4] }], q?: model angles, G?: T_base_end }
// the 8 corner orders (4 cyclic shifts x 2 directions) of the detector relative to the board model
const ORDERS = [[0, 1, 2, 3], [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2], [0, 3, 2, 1], [3, 2, 1, 0], [2, 1, 0, 3], [1, 0, 3, 2]];
export function correspondences(view, board, order = ORDERS[0]) {
  const obj = [], img = [];
  for (const m of view.markers || []) {
    const bm = board.byId.get(Number(m.id));
    if (!bm || !Array.isArray(m.corners) || m.corners.length !== 4) continue;
    for (let k = 0; k < 4; k++) { obj.push(bm.corners[order[k]]); img.push(m.corners[k]); }
  }
  return { obj, img };
}
// board markers of a view (unique ids on the board)
export function boardMarkers(view, board) {
  const seen = new Set();
  return (view.markers || []).filter((m) => board.byId.has(Number(m.id)) && Array.isArray(m.corners) && m.corners.length === 4 && !seen.has(Number(m.id)) && seen.add(Number(m.id)));
}
// detector corner order: the order whose homography fits the view with the most markers best
export function detectCornerOrder(views, board) {
  const v = views.slice().sort((a, b) => boardMarkers(b, board).length - boardMarkers(a, board).length)[0];
  if (!v || boardMarkers(v, board).length < 2) return { order: ORDERS[0], rms: NaN, index: 0 };
  let best = { order: ORDERS[0], rms: Infinity, index: 0 };
  ORDERS.forEach((order, index) => {
    const { obj, img } = correspondences(v, board, order);
    const H = homography(obj, img);
    if (!H) return;
    let s = 0;
    obj.forEach((X, i) => { const p = hApply(H, X); s += (p[0] - img[i][0]) ** 2 + (p[1] - img[i][1]) ** 2; });
    const rms = Math.sqrt(s / obj.length);
    if (rms < best.rms) best = { order, rms, index };
  });
  return best;
}

// ---------------------------------------------------------------- homography (normalised DLT)
const hApply = (H, X) => { const w = H[2][0] * X[0] + H[2][1] * X[1] + H[2][2]; return [(H[0][0] * X[0] + H[0][1] * X[1] + H[0][2]) / w, (H[1][0] * X[0] + H[1][1] * X[1] + H[1][2]) / w]; };
function normT(P) {
  const n = P.length, mx = P.reduce((s, p) => s + p[0], 0) / n, my = P.reduce((s, p) => s + p[1], 0) / n;
  const d = P.reduce((s, p) => s + Math.hypot(p[0] - mx, p[1] - my), 0) / n || 1, k = Math.SQRT2 / d;
  return [[k, 0, -k * mx], [0, k, -k * my], [0, 0, 1]];
}
// H mapping board (X, Y) to (u, v); null when degenerate
export function homography(src, dst) {
  const n = src.length;
  if (n < 4) return null;
  const Ts = normT(src), Td = normT(dst);
  const A = Array.from({ length: 9 }, () => new Array(9).fill(0));
  const row = (r) => { for (let i = 0; i < 9; i++) for (let j = i; j < 9; j++) A[i][j] += r[i] * r[j]; };
  for (let i = 0; i < n; i++) {
    const X = Ts[0][0] * src[i][0] + Ts[0][2], Y = Ts[1][1] * src[i][1] + Ts[1][2];
    const u = Td[0][0] * dst[i][0] + Td[0][2], v = Td[1][1] * dst[i][1] + Td[1][2];
    row([-X, -Y, -1, 0, 0, 0, u * X, u * Y, u]);
    row([0, 0, 0, -X, -Y, -1, v * X, v * Y, v]);
  }
  for (let i = 0; i < 9; i++) for (let j = 0; j < i; j++) A[i][j] = A[j][i];
  const h = minEigVec(A);
  const Hn = [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], h[8]]];
  const Tdi = [[1 / Td[0][0], 0, -Td[0][2] / Td[0][0]], [0, 1 / Td[1][1], -Td[1][2] / Td[1][1]], [0, 0, 1]];
  const H = mm(mm(Tdi, Hn), Ts);
  const s = H[2][2];
  if (!Number.isFinite(s) || Math.abs(s) < 1e-15) return null;
  return H.map((r) => r.map((v) => v / s));
}

// ---------------------------------------------------------------- Zhang closed form
// Hs: homographies (board -> pixels). Pixels are first normalised (centre, scale) so the 6x6 system is
// well conditioned; zero skew is enforced as an extra equation. Returns { fx, fy, cx, cy } or null.
export function zhang(Hs, size = IMAGE) {
  const s = (size.width + size.height) / 2, N = [[1 / s, 0, -size.width / 2 / s], [0, 1 / s, -size.height / 2 / s], [0, 0, 1]];
  const rows = [];
  const vij = (H, i, j) => [H[0][i] * H[0][j], H[0][i] * H[1][j] + H[1][i] * H[0][j], H[1][i] * H[1][j], H[2][i] * H[0][j] + H[0][i] * H[2][j], H[2][i] * H[1][j] + H[1][i] * H[2][j], H[2][i] * H[2][j]];
  for (const H0 of Hs) {
    let H = mm(N, H0);
    const f = Math.hypot(...H.flat()); H = H.map((r) => r.map((v) => v / f));
    const v12 = vij(H, 0, 1), v11 = vij(H, 0, 0), v22 = vij(H, 1, 1);
    rows.push(v12, v11.map((v, k) => v - v22[k]));
  }
  rows.push([0, 10, 0, 0, 0, 0]); // B12 = 0 (no skew)
  const A = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (__, j) => rows.reduce((a, r) => a + r[i] * r[j], 0)));
  const b = minEigVec(A);
  const sgn = b[0] < 0 ? -1 : 1;
  const [B11, B12, B22, B13, B23, B33] = b.map((v) => v * sgn);
  const den = B11 * B22 - B12 * B12;
  const v0 = (B12 * B13 - B11 * B23) / den;
  const lam = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11;
  const a = Math.sqrt(lam / B11), be = Math.sqrt(lam * B11 / den);
  const g = -B12 * a * a * be / lam, u0 = g * v0 / be - B13 * a * a / lam;
  const K = { fx: a * s, fy: be * s, cx: u0 * s + size.width / 2, cy: v0 * s + size.height / 2 };
  return Object.values(K).every(Number.isFinite) ? K : null;
}
// fx, fy with the principal point at the image centre (2 linear equations per view); null when unobservable
export function focalCentre(Hs, size = IMAGE) {
  const cx = size.width / 2, cy = size.height / 2, A = [[0, 0], [0, 0]], r = [0, 0];
  for (const H0 of Hs) {
    const H = H0.map((row, i) => row.map((v, j) => (i === 0 ? v - cx * H0[2][j] : i === 1 ? v - cy * H0[2][j] : v)));
    const f = Math.hypot(...H.flat()), h = H.map((row) => row.map((v) => v / f));
    // a = 1/fx^2, b = 1/fy^2: a h00 h01 + b h10 h11 = -h20 h21 ; a (h00^2 - h01^2) + b (h10^2 - h11^2) = -(h20^2 - h21^2)
    for (const [p, q, c] of [[h[0][0] * h[0][1], h[1][0] * h[1][1], -h[2][0] * h[2][1]], [h[0][0] ** 2 - h[0][1] ** 2, h[1][0] ** 2 - h[1][1] ** 2, -(h[2][0] ** 2 - h[2][1] ** 2)]]) {
      A[0][0] += p * p; A[0][1] += p * q; A[1][1] += q * q; r[0] += p * c; r[1] += q * c;
    }
  }
  const det = A[0][0] * A[1][1] - A[0][1] * A[0][1];
  if (!(Math.abs(det) > 1e-30)) return null;
  const a = (r[0] * A[1][1] - r[1] * A[0][1]) / det, b = (A[0][0] * r[1] - A[0][1] * r[0]) / det;
  if (!(a > 0 && b > 0)) return null;
  return { fx: 1 / Math.sqrt(a), fy: 1 / Math.sqrt(b), cx, cy };
}
const plausible = (K, size = IMAGE) => K && K.fx >= 100 && K.fx <= 5000 && K.fy >= 100 && K.fy <= 5000 && K.cx > 0 && K.cx < size.width && K.cy > 0 && K.cy < size.height && Math.abs(K.fx / K.fy - 1) < 0.3;
// board pose from a homography and K (pixels, no distortion): { R, t } with the board in front
export function poseFromH(K, H) {
  const Ki = (p) => [(p[0] - K.cx * p[2]) / K.fx, (p[1] - K.cy * p[2]) / K.fy, p[2]];
  const col = (j) => [H[0][j], H[1][j], H[2][j]];
  const h1 = Ki(col(0)), h2 = Ki(col(1)), h3 = Ki(col(2));
  let lam = 2 / (norm3(h1) + norm3(h2));
  if (h3[2] * lam < 0) lam = -lam;
  const r1 = scl3(h1, lam), r2 = scl3(h2, lam), r3 = cross(r1, r2), t = scl3(h3, lam);
  const R = nearestRot([[r1[0], r2[0], r3[0]], [r1[1], r2[1], r3[1]], [r1[2], r2[2], r3[2]]]);
  return { R, t };
}

// ---------------------------------------------------------------- Levenberg-Marquardt
const INTR = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2'];
// views: [{ obj: [[X, Y, 0]], img: [[u, v]] }], K0 intrinsics, poses0: [{ R, t }]
// opt.fixIntr: true (all fixed) or array of 6 booleans. Numeric (central difference) Jacobian, block
// sparse normal equations (each pose block couples only to its own view and the intrinsics).
export function lmRefine(views, K0, poses0, opt = {}) {
  const nv = views.length;
  const fixI = opt.fixIntr === true ? [1, 1, 1, 1, 1, 1] : (opt.fixIntr || [0, 0, 0, 0, 0, 0]);
  const iIdx = INTR.map((_, k) => k).filter((k) => !fixI[k]), ni = iIdx.length, P = ni + 6 * nv;
  let intr = INTR.map((k) => K0[k] || 0), pose = poses0.map((p) => [...logRot(p.R), ...p.t]);
  const toK = (a) => ({ fx: a[0], fy: a[1], cx: a[2], cy: a[3], k1: a[4], k2: a[5] });
  const resView = (a, pz, v) => {
    const R = rodrigues(pz), t = [pz[3], pz[4], pz[5]], K = toK(a), V = views[v], out = new Float64Array(V.obj.length * 2);
    for (let j = 0; j < V.obj.length; j++) { const p = project(K, R, t, V.obj[j]); out[2 * j] = p[0] - V.img[j][0]; out[2 * j + 1] = p[1] - V.img[j][1]; }
    return out;
  };
  const sse = (a, pzs) => { let s = 0; for (let v = 0; v < nv; v++) for (const e of resView(a, pzs[v], v)) s += e * e; return s; };
  const hI = [1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6], hP = [1e-6, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7];
  // normal matrix and gradient at (intr, pose)
  function normals() {
    const N = new Float64Array(P * P), g = new Float64Array(P);
    for (let v = 0; v < nv; v++) {
      const r0 = resView(intr, pose[v], v), m = r0.length, JI = [], JP = [];
      for (const k of iIdx) {
        const ap = intr.slice(), am = intr.slice(); ap[k] += hI[k]; am[k] -= hI[k];
        const rp = resView(ap, pose[v], v), rm = resView(am, pose[v], v), c = new Float64Array(m);
        for (let i = 0; i < m; i++) c[i] = (rp[i] - rm[i]) / (2 * hI[k]);
        JI.push(c);
      }
      for (let k = 0; k < 6; k++) {
        const pp = pose[v].slice(), pm = pose[v].slice(); pp[k] += hP[k]; pm[k] -= hP[k];
        const rp = resView(intr, pp, v), rm = resView(intr, pm, v), c = new Float64Array(m);
        for (let i = 0; i < m; i++) c[i] = (rp[i] - rm[i]) / (2 * hP[k]);
        JP.push(c);
      }
      const cols = [...JI.map((c, a) => [a, c]), ...JP.map((c, k) => [ni + 6 * v + k, c])];
      for (let x = 0; x < cols.length; x++) {
        const [ix, cx] = cols[x];
        let gs = 0; for (let i = 0; i < m; i++) gs += cx[i] * r0[i];
        g[ix] += gs;
        for (let y = x; y < cols.length; y++) {
          const [iy, cy] = cols[y];
          let s = 0; for (let i = 0; i < m; i++) s += cx[i] * cy[i];
          N[ix * P + iy] += s; if (ix !== iy) N[iy * P + ix] += s;
        }
      }
    }
    return { N, g };
  }
  let cost = sse(intr, pose), lam = 1e-3, it = 0, Nlast = null;
  const maxIt = opt.iters ?? 60;
  for (; it < maxIt; it++) {
    const { N, g } = normals();
    Nlast = N;
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = N.slice();
      for (let i = 0; i < P; i++) A[i * P + i] += lam * (N[i * P + i] || 1e-12);
      const L = chol(A, P);
      if (!L) { lam *= 10; continue; }
      const dx = cholSolve(L, P, g.map((v) => -v));
      const a2 = intr.slice(); iIdx.forEach((k, j) => { a2[k] += dx[j]; });
      const p2 = pose.map((pz, v) => pz.map((x, k) => x + dx[ni + 6 * v + k]));
      const c2 = sse(a2, p2);
      if (Number.isFinite(c2) && c2 < cost) {
        const rel = (cost - c2) / Math.max(cost, 1e-300);
        intr = a2; pose = p2; cost = c2; lam = Math.max(lam / 10, 1e-9); improved = true;
        if (rel < 1e-12) { it = maxIt; }
      } else lam *= 10;
    }
    if (!improved) break;
  }
  // per-view RMS (per-corner distance), covariance of the free intrinsics
  const nPts = views.reduce((s, V) => s + V.obj.length, 0);
  const perView = views.map((V, v) => { const r = resView(intr, pose[v], v); let s = 0; for (const e of r) s += e * e; return Math.sqrt(s / V.obj.length); });
  const { N } = normals();
  Nlast = N;
  const dof = Math.max(1, 2 * nPts - P), s2 = cost / dof;
  let sigma = null, cond = Infinity, singular = false;
  const L = chol(N, P);
  if (!L) singular = true;
  else if (ni) {
    const C = iIdx.map((_, j) => { const e = new Float64Array(P); e[j] = 1; return cholSolve(L, P, e); });
    const Cii = iIdx.map((_, j) => C[j][j]);
    sigma = {};
    iIdx.forEach((k, j) => { sigma[INTR[k]] = Math.sqrt(Math.max(0, s2 * Cii[j])); });
    const Cn = iIdx.map((_, i) => iIdx.map((__, j) => C[i][j] / Math.sqrt(Cii[i] * Cii[j])));
    const ev = symEig(Cn).vals;
    cond = Math.max(...ev) / Math.max(Math.min(...ev), 1e-300);
  }
  return {
    K: toK(intr), poses: pose.map((pz) => ({ R: rodrigues(pz), t: [pz[3], pz[4], pz[5]] })),
    rms: Math.sqrt(cost / nPts), perView, sigma, cond, singular, iterations: it, nPts,
  };
}

// ---------------------------------------------------------------- PnP (planar board)
// obj: board points (z = 0), img: pixels, K intrinsics -> { R, t, rms } (T_cam_board) or null
export function solvePnP(obj, img, K) {
  if (obj.length < 4) return null;
  const nrm = img.map(([u, v]) => undistort(K, u, v));
  const H = homography(obj, nrm);
  if (!H) return null;
  const p0 = poseFromH({ fx: 1, fy: 1, cx: 0, cy: 0 }, H);
  if (!p0.t.every(Number.isFinite)) return null;
  const r = lmRefine([{ obj, img }], K, [p0], { fixIntr: true, iters: 30 });
  return { R: r.poses[0].R, t: r.poses[0].t, rms: r.rms };
}

// ---------------------------------------------------------------- joint refinement
// dense Levenberg-Marquardt with a central-difference Jacobian: res(x) -> Float64Array, x0, steps h
function denseLM(res, x0, h, iters = 50) {
  const P = x0.length;
  let x = x0.slice(), r = res(x), cost = r.reduce((s, e) => s + e * e, 0), lam = 1e-3, it = 0;
  for (; it < iters; it++) {
    const m = r.length, J = [];
    for (let k = 0; k < P; k++) {
      const xp = x.slice(), xm = x.slice(); xp[k] += h[k]; xm[k] -= h[k];
      const rp = res(xp), rm = res(xm), c = new Float64Array(m);
      for (let i = 0; i < m; i++) c[i] = (rp[i] - rm[i]) / (2 * h[k]);
      J.push(c);
    }
    const N = new Float64Array(P * P), g = new Float64Array(P);
    for (let a = 0; a < P; a++) {
      let gs = 0; for (let i = 0; i < m; i++) gs += J[a][i] * r[i]; g[a] = gs;
      for (let b = a; b < P; b++) { let v = 0; for (let i = 0; i < m; i++) v += J[a][i] * J[b][i]; N[a * P + b] = v; N[b * P + a] = v; }
    }
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = N.slice();
      for (let i = 0; i < P; i++) A[i * P + i] += lam * (N[i * P + i] || 1e-12);
      const L = chol(A, P);
      if (!L) { lam *= 10; continue; }
      const dx = cholSolve(L, P, g.map((v) => -v)), x2 = x.map((v, k) => v + dx[k]), r2 = res(x2);
      const c2 = r2.reduce((s, e) => s + e * e, 0);
      if (Number.isFinite(c2) && c2 < cost) {
        const rel = (cost - c2) / Math.max(cost, 1e-300);
        x = x2; r = r2; cost = c2; lam = Math.max(lam / 10, 1e-9); improved = true;
        if (rel < 1e-10) it = iters;
      } else lam *= 10;
    }
    if (!improved) break;
  }
  return { x, cost, r, iterations: it };
}
/* One LM over all corner observations after the separate solves: intrinsics (fx fy cx cy k1 k2),
   X = T_end_cam (6) and the board pose in base T_base_board (6); the FK poses G are fixed.
   views: [{ obj, img, G }] (board points, pixels, T_base_end). Returns { K, X, Tbb, rmsBefore,
   rmsAfter, perView, iterations } (RMS per corner, px). */
export function refineJoint(views, K0, X0, Tbb0, { iters = 100, fixK = false } = {}) {
  const RX0 = rotOf(X0), RB0 = rotOf(Tbb0);
  const K0v = [K0.fx, K0.fy, K0.cx, K0.cy, K0.k1 || 0, K0.k2 || 0];
  // fixK: only X and the board pose move (the intrinsics stay at K0)
  const full = (x) => (fixK ? [...K0v, ...x] : x);
  const unpack = (xs) => { const x = full(xs); return { K: { fx: x[0], fy: x[1], cx: x[2], cy: x[3], k1: x[4], k2: x[5] },
    X: make(mm(RX0, rodrigues([x[6], x[7], x[8]])), [x[9], x[10], x[11]]),
    B: make(mm(RB0, rodrigues([x[12], x[13], x[14]])), [x[15], x[16], x[17]]) }; };
  const nPts = views.reduce((s, V) => s + V.obj.length, 0);
  const res = (x) => {
    const { K, X, B } = unpack(x), out = new Float64Array(2 * nPts);
    let o = 0;
    for (const V of views) {
      const T = mm(inv(mm(V.G, X)), B), R = rotOf(T), t = posOf(T);
      for (let j = 0; j < V.obj.length; j++) { const p = project(K, R, t, V.obj[j]); out[o++] = p[0] - V.img[j][0]; out[o++] = p[1] - V.img[j][1]; }
    }
    return out;
  };
  const x0a = [...K0v, 0, 0, 0, ...posOf(X0), 0, 0, 0, ...posOf(Tbb0)];
  const ha = [1e-3, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7];
  const x0 = fixK ? x0a.slice(6) : x0a, h = fixK ? ha.slice(6) : ha;
  const r0 = res(x0), c0 = r0.reduce((s, e) => s + e * e, 0);
  const lm = denseLM(res, x0, h, iters);
  const { K, X, B } = unpack(lm.x);
  let o = 0;
  const perView = views.map((V) => { let s = 0; for (let j = 0; j < 2 * V.obj.length; j++) s += lm.r[o + j] ** 2; o += 2 * V.obj.length; return Math.sqrt(s / V.obj.length); });
  return { K, X, Tbb: B, rmsBefore: Math.sqrt(c0 / nPts), rmsAfter: Math.sqrt(lm.cost / nPts), perView, iterations: lm.iterations, nPts, fixK };
}

// ---------------------------------------------------------------- intrinsics with gates
// image 3x3 cells covered by the corners of the views
export function coverage(views, size = IMAGE) {
  const cells = new Set();
  for (const V of views) for (const [u, v] of V.img) {
    const i = Math.min(2, Math.max(0, Math.floor(u / size.width * 3))), j = Math.min(2, Math.max(0, Math.floor(v / size.height * 3)));
    cells.add(j * 3 + i);
  }
  return cells;
}
function calibrateRaw(cv, size, init) {
  const Hs = cv.map((V) => homography(V.obj, V.img));
  if (Hs.some((H) => !H)) return { err: 'homography' };
  let K0 = zhang(Hs, size), method = 'zhang';
  if (!plausible(K0, size)) { K0 = focalCentre(Hs, size); method = 'centre'; }
  if (!plausible(K0, size)) { K0 = { ...(init || NOMINAL) }; method = 'nominal'; }
  K0 = { ...K0, k1: 0, k2: 0 };
  const poses0 = cv.map((V, i) => poseFromH(K0, Hs[i]));
  const r = lmRefine(cv, K0, poses0, { iters: 80 });
  return { ...r, init: K0, method };
}
const gate = (key, ok, value, limit, unit = '') => ({ key, ok: !!ok, value, limit, unit });
/* Intrinsics from board views with the W3 / X1 gates.
   views: [{ markers }] (raw), board. Returns { ok, K, rms, perView, sigma, cond, gates, used: [view
   index], dropped: [{ i, rms }], skipped: [{ i, markers }], heldOut: { rms, n }, order }. */
export function calibrateIntrinsics(views, board, { size = IMAGE, gates = GATES, init = NOMINAL, order = null } = {}) {
  const ord = order || detectCornerOrder(views, board).order;
  const skipped = [], idx = [];
  views.forEach((V, i) => { const n = boardMarkers(V, board).length; if (n < gates.minMarkers) skipped.push({ i, markers: n }); else idx.push(i); });
  const cvOf = (ids) => ids.map((i) => correspondences({ markers: boardMarkers(views[i], board) }, board, ord));
  const out = { ok: false, order: ord, skipped, dropped: [], gates: [] };
  if (idx.length < 4) { out.gates.push(gate('views', false, idx.length, gates.minViews)); out.err = 'few_views'; return out; }
  let used = idx, r = calibrateRaw(cvOf(used), size, init);
  if (r.err) { out.err = r.err; out.gates.push(gate('views', false, idx.length, gates.minViews)); return out; }
  // drop views above the per-view limit once, then solve again
  const bad = used.filter((_, k) => !(r.perView[k] <= gates.viewRmsPx));
  if (bad.length && used.length - bad.length >= 4) {
    out.dropped = bad.map((i) => ({ i, rms: r.perView[used.indexOf(i)] }));
    used = used.filter((i) => !bad.includes(i));
    r = calibrateRaw(cvOf(used), size, init);
    if (r.err) { out.err = r.err; return out; }
  }
  // held out: solve without every 5th used view, predict its corners (pose by PnP with those intrinsics)
  let held = { rms: NaN, n: 0 };
  const hold = used.filter((_, k) => k % 5 === 4), keep = used.filter((_, k) => k % 5 !== 4);
  if (hold.length && keep.length >= 4) {
    const rk = calibrateRaw(cvOf(keep), size, init);
    if (!rk.err) {
      let s = 0, n = 0;
      for (const V of cvOf(hold)) { const p = solvePnP(V.obj, V.img, rk.K); if (p) { s += p.rms * p.rms * V.obj.length; n += V.obj.length; } else { s = Infinity; } }
      held = { rms: n ? Math.sqrt(s / n) : NaN, n: hold.length };
    }
  }
  const cells = coverage(cvOf(used), size);
  const sg = r.sigma || {}, rel = (k) => (sg[k] != null ? sg[k] / Math.abs(r.K[k]) : Infinity);
  const sigWorst = Math.max(...['fx', 'fy', 'cx', 'cy'].map(rel));
  const vmax = Math.max(...r.perView);
  const minMk = Math.min(...used.map((i) => boardMarkers(views[i], board).length));
  out.gates.push(
    gate('views', used.length >= gates.minViews, used.length, gates.minViews),
    gate('markers', minMk >= gates.minMarkers, minMk, gates.minMarkers),
    gate('coverage', cells.size === 9, cells.size, 9, 'cells'),
    gate('rms', r.rms <= gates.rmsPx, r.rms, gates.rmsPx, 'px'),
    gate('view_rms', vmax <= gates.viewRmsPx, vmax, gates.viewRmsPx, 'px'),
    gate('held_out', held.rms <= gates.heldOutRmsPx, held.rms, gates.heldOutRmsPx, 'px'),
    gate('sigma', !r.singular && sigWorst <= gates.sigmaRel, sigWorst, gates.sigmaRel, 'rel'),
  );
  Object.assign(out, { ok: out.gates.every((g) => g.ok), K: r.K, rms: r.rms, perView: r.perView, sigma: r.sigma, cond: r.cond, singular: r.singular,
    used, heldOut: held, method: r.method, init: r.init, iterations: r.iterations, cells: [...cells].sort() });
  return out;
}

// ---------------------------------------------------------------- hand-eye and table
// CALM: the cockpit's calibration math (index.html CALIB-MATH block). G: T_base_end per view, M: T_cam_board.
export function solveHandEye(CALM, G, M, gates = GATES) {
  const r = CALM.solveEyeInHand(G, M);
  if (r.err) return { ok: false, err: r.err, div: r.div, gates: [gate('rot_diversity', false, r.div ? r.div.ratio : 0, 0.1)] };
  const g = [
    gate('rot_diversity', r.div.ok, r.div.ratio, 0.1),
    gate('he_pos', r.rmse <= gates.handEyeRmsM, r.rmse, gates.handEyeRmsM, 'm'),
    gate('he_rot', r.rotRmse <= gates.handEyeRotDeg, r.rotRmse, gates.handEyeRotDeg, 'deg'),
  ];
  return { ok: g.every((x) => x.ok), X: r.X, Y: r.Y, rmse: r.rmse, rotRmse: r.rotRmse, residuals: r.residuals, rotResiduals: r.rotResiduals, div: r.div, gates: g };
}
// Tbb: T_base_board per view -> { table_z_m, tiltDeg, spread_m, normal, ok, gates }
export function fitTable(Tbb, board, thickness = 0.0005, gates = GATES) {
  const pts = [];
  for (const T of Tbb) for (const m of board.markers) for (const c of m.corners) pts.push(apply(T, c));
  const n = pts.length, c = [0, 1, 2].map((k) => pts.reduce((s, p) => s + p[k], 0) / n);
  const C = [0, 1, 2].map((i) => [0, 1, 2].map((j) => pts.reduce((s, p) => s + (p[i] - c[i]) * (p[j] - c[j]), 0) / n));
  let nrm = minEigVec(C);
  if (nrm[2] < 0) nrm = scl3(nrm, -1);
  const tiltDeg = Math.acos(Math.min(1, Math.abs(nrm[2]))) * D;
  const zs = Tbb.map((T) => T[2][3]).sort((a, b) => a - b);
  const med = zs.length % 2 ? zs[(zs.length - 1) / 2] : (zs[zs.length / 2 - 1] + zs[zs.length / 2]) / 2;
  const spread = zs[zs.length - 1] - zs[0];
  const g = [gate('plane_tilt', tiltDeg <= gates.planeTiltDeg, tiltDeg, gates.planeTiltDeg, 'deg'), gate('plane_spread', spread <= gates.planeSpreadM, spread, gates.planeSpreadM, 'm')];
  return { ok: g.every((x) => x.ok), table_z_m: med - thickness, board_z_m: med, tiltDeg, spread_m: spread, normal: nrm, centroid: c, gates: g };
}

/* Full solve of a capture run. views: [{ markers, G (T_base_end) }]; CALM; board; thickness (m).
   Separate solves (intrinsics, PnP per view, hand-eye, board plane), then one joint LM (refineJoint)
   over all corners: the refined intrinsics, X and board pose are the result (`final`); the hand-eye and
   table gates are evaluated with them (per-view PnP with the refined K, T_base_board_i = G_i X M_i
   against the refined board pose), table_z_m = refined board origin z - thickness.
   Returns { ok, intr, handEye, joint, final: { K, X, Tbb }, table, gates, apply (body for
   /api/v1/calibration/apply) }. */
export function solveAll({ views, board, CALM, thickness = 0.0005, gates = GATES, size = IMAGE, init = NOMINAL, solvedAt = null, joint = true }) {
  const intr = calibrateIntrinsics(views, board, { size, gates, init });
  const res = { ok: false, intr, gates: [...intr.gates] };
  if (!intr.K) return res;
  const G = [], M = [], vi = [], obs = [];
  for (const i of intr.used) {
    const V = views[i];
    if (!V.G) continue;
    const { obj, img } = correspondences({ markers: boardMarkers(V, board) }, board, intr.order);
    const p = solvePnP(obj, img, intr.K);
    if (p) { G.push(V.G); M.push(make(p.R, p.t)); vi.push(i); obs.push({ obj, img, G: V.G }); }
  }
  // one outlier pass: a view whose board pose in base is far from the others (arm positioning error
  // at that pose, e.g. the start view right after torque-on) is left out of the hand-eye once
  let he = CALM.solveEyeInHand(G, M);
  res.droppedHandEye = [];
  if (!he.err && he.X && G.length > 6) {
    const r = he.residuals, med = [...r].sort((a, b) => a - b)[Math.floor(r.length / 2)], w = r.indexOf(Math.max(...r));
    if (r[w] > 3 * med && r[w] > gates.handEyeRmsM) {
      res.droppedHandEye.push({ i: vi[w], m: r[w] });
      for (const a of [G, M, vi, obs]) a.splice(w, 1);
      he = CALM.solveEyeInHand(G, M);
    }
  }
  res.poseViews = vi;
  if (he.err || !he.X) { const h0 = solveHandEye(CALM, G, M, gates); res.handEye = h0; res.gates.push(...h0.gates); return res; }
  // joint refinement from the separate solve (board pose: CALM's mean T_base_board)
  let K = intr.K, X = he.X, Tbb = he.Y;
  if (joint) {
    // The intrinsics come from the images alone. When the joint LM moves them by more than 3 sigma it
    // is absorbing the arm's positioning error into the camera model (seen on a real arm: cy 185 -> 62);
    // then only X and the board pose are refined.
    let j = refineJoint(obs, intr.K, he.X, he.Y);
    const sg = intr.sigma || {}, moved = ['fx', 'fy', 'cx', 'cy'].filter((k) => !(Math.abs(j.K[k] - intr.K[k]) <= 3 * Math.max(sg[k] || 0, 1e-3 * Math.abs(intr.K[k]))));
    if (moved.length) j = { ...refineJoint(obs, intr.K, he.X, he.Y, { fixK: true }), kMoved: moved };
    res.joint = { rmsBefore: j.rmsBefore, rmsAfter: j.rmsAfter, perView: j.perView, iterations: j.iterations, fixK: !!j.fixK, kMoved: j.kMoved || [], separate: { K: intr.K, X: he.X, Tbb: he.Y } };
    if ([j.K.fx, j.K.fy, j.K.cx, j.K.cy, j.rmsAfter].every(Number.isFinite) && j.rmsAfter <= j.rmsBefore) { K = j.K; X = j.X; Tbb = j.Tbb; }
  }
  res.final = { K, X, Tbb };
  // hand-eye residuals with the final values: per-view board pose in base vs the refined one
  const Tbi = obs.map((o) => { const p = solvePnP(o.obj, o.img, K); return p ? mm(mm(o.G, X), make(p.R, p.t)) : null; }).filter(Boolean);
  const RBt = tr(rotOf(Tbb)), rms = (a) => Math.sqrt(a.reduce((s2, e) => s2 + e * e, 0) / Math.max(1, a.length));
  const residuals = Tbi.map((T) => norm3(sub3(posOf(T), posOf(Tbb)))), rotResiduals = Tbi.map((T) => angDeg(mm(RBt, rotOf(T))));
  const hg = [
    gate('rot_diversity', he.div.ok, he.div.ratio, 0.1),
    gate('he_pos', rms(residuals) <= gates.handEyeRmsM, rms(residuals), gates.handEyeRmsM, 'm'),
    gate('he_rot', rms(rotResiduals) <= gates.handEyeRotDeg, rms(rotResiduals), gates.handEyeRotDeg, 'deg'),
  ];
  res.handEye = { ok: hg.every((x) => x.ok), X, Y: Tbb, rmse: rms(residuals), rotRmse: rms(rotResiduals), residuals, rotResiduals, div: he.div, gates: hg,
    separate: { X: he.X, rmse: he.rmse, rotRmse: he.rotRmse } };
  res.gates.push(...hg);
  const table = fitTable(Tbi, board, thickness, gates);
  table.board_z_m = Tbb[2][3]; table.table_z_m = Tbb[2][3] - thickness;
  res.table = table; res.gates.push(...table.gates);
  res.ok = res.gates.every((g) => g.ok);
  res.apply = {
    intrinsics: { fx: K.fx, fy: K.fy, cx: K.cx, cy: K.cy, k1: K.k1, k2: K.k2, p1: 0, p2: 0, k3: 0, width: size.width, height: size.height, marker_size: board.markerSide },
    camera_mount: { offset_m: posOf(X), q_end_cam: quatOf(rotOf(X)) },
    table_z_m: table.table_z_m,
    rms: { reproj_px: intr.rms, held_out_px: intr.heldOut.rms, joint_before_px: res.joint ? res.joint.rmsBefore : null, joint_px: res.joint ? res.joint.rmsAfter : null,
      hand_eye_m: res.handEye.rmse, hand_eye_deg: res.handEye.rotRmse, views: intr.used.length,
      sigma: intr.sigma, cond: intr.cond, plane_tilt_deg: table.tiltDeg, plane_spread_m: table.spread_m },
    ...(solvedAt ? { solved_at: solvedAt } : {}),
  };
  return res;
}

// ---------------------------------------------------------------- pose generator (W5, X2)
// visible markers of the board at camera pose T_base_cam: markers with all corners inside the image
// (margin px) and in front of the camera
export function visibleMarkers(K, T_base_cam, Tbb, board, { size = IMAGE, margin = 8 } = {}) {
  const T = mm(inv(T_base_cam), Tbb), R = rotOf(T), t = posOf(T), out = [];
  for (const m of board.markers) {
    const px = m.corners.map((c) => project(K, R, t, c));
    if (px.every(([u, v, z]) => z > 0.02 && u >= margin && u <= size.width - margin && v >= margin && v <= size.height - margin)) out.push({ id: m.id, corners: px.map(([u, v]) => [u, v]) });
  }
  return out;
}
// interpolate end poses A -> B (straight line, rotation along the geodesic), f in [0, 1]
export function interpPose(A, B, f) {
  const Ra = rotOf(A), w = logRot(mm(tr(Ra), rotOf(B)));
  return make(mm(Ra, rodrigues(scl3(w, f))), add3(posOf(A), scl3(sub3(posOf(B), posOf(A)), f)));
}
// tool points to keep above the board: end_link origin (jaw tips), the wrist 0.155 m back along the
// tool axis (end_link x), the camera centre
export const toolPoints = (Tend, X) => { const p = posOf(Tend), x = [Tend[0][0], Tend[1][0], Tend[2][0]]; return [p, sub3(p, scl3(x, 0.155)), apply(Tend, posOf(X))]; };
/* Straight-line transition A -> B checked at interpolated points (<= stepM / stepRad apart) with the IK
   (ik(T, qSeed) -> { ok, q }); refuses an IK failure, a joint jump > maxJointStep between samples (branch
   flip), joint limits, a tool point below the clearance plane. Returns { ok, q, reason, at }. */
export function checkTransition(A, qA, B, { ik, X, plane, clearance, lo, hi, stepM = 0.01, stepRad = 0.05, maxJointStep = 0.25 }) {
  const d = norm3(sub3(posOf(B), posOf(A))), a = norm3(logRot(mm(tr(rotOf(A)), rotOf(B))));
  const n = Math.max(1, Math.ceil(Math.max(d / stepM, a / stepRad)));
  let q = qA.slice();
  for (let k = 1; k <= n; k++) {
    const T = interpPose(A, B, k / n), s = ik(T, q);
    if (!s || !s.ok) return { ok: false, reason: 'ik', at: k / n };
    if (s.q.some((v, i) => Math.abs(v - q[i]) > maxJointStep)) return { ok: false, reason: 'flip', at: k / n };
    if (lo && s.q.some((v, i) => v < lo[i] - 1e-6 || v > hi[i] + 1e-6)) return { ok: false, reason: 'limits', at: k / n };
    if (plane && toolPoints(T, X).some((p) => dot3(sub3(p, plane.p), plane.n) < clearance)) return { ok: false, reason: 'clearance', at: k / n };
    q = s.q;
  }
  return { ok: true, q };
}
/* Look-at poses toward the board. Inputs: Tbb (T_base_board estimate), board, K, X (T_end_cam), q0 (model
   J1..J6 at the start), fk(q) -> T_base_end, ik(T, seed) -> { ok, q }, lo / hi model limits.
   Camera 0.18..0.35 m from the board (dists), view tilt 0..30 deg (tilts) in 8 azimuths, the board centre
   placed at 5 image points, roll about the optical axis 0 / +-15 deg from the board-aligned roll;
   candidates need >= minMarkers fully visible markers (predicted with K), every tool point >= clearance
   above the board plane (0.07 m + 0.03 m margin for the estimate), and an IK path from the previous pose
   without branch flip. Farthest-point selection over (tilt direction, distance, roll, aim) for n poses,
   ordered as a nearest-neighbour tour in joint space from q0.
   Returns { poses: [{ T, q, camPos, dist, tiltDeg, azDeg, rollDeg, aim, markers }], stats }. */
export function generatePoses({ Tbb, board, K = NOMINAL, X, q0, fk, ik, lo = null, hi = null, n = 20, size = IMAGE,
  dists = [0.18, 0.20, 0.23, 0.26, 0.30, 0.35], tilts = [0, 10, 20, 30], azimuths = 8, rolls = [0, 15, -15],
  aims = [[0.5, 0.5], [0.3, 0.35], [0.7, 0.35], [0.3, 0.65], [0.7, 0.65]], minMarkers = GATES.minMarkers,
  clearance = 0.10, distRange = [0.18, 0.35], exclude = [] }) {
  const Rb = rotOf(Tbb), plane = boardPlane(Tbb), c = plane.p, nz = plane.n;
  const ex = unit(sub3([Rb[0][0], Rb[1][0], Rb[2][0]], scl3(nz, dot3([Rb[0][0], Rb[1][0], Rb[2][0]], nz)))), ey = cross(nz, ex);
  const T0 = fk(q0), Rc0 = mm(rotOf(T0), rotOf(X)), xc0 = [Rc0[0][0], Rc0[1][0], Rc0[2][0]];
  const Rec = rotOf(X), tec = posOf(X);
  const stats = { candidates: 0, markers: 0, clearance: 0, reach: 0, transition: 0, distance: 0 };
  const cand = [];
  for (const d of dists) for (const th of tilts) for (let ai = 0; ai < (th ? azimuths : 1); ai++) for (const [au, av] of aims) for (const roll of rolls) {
    stats.candidates++;
    if (d < distRange[0] || d > distRange[1]) { stats.distance++; continue; }
    const az = (360 / azimuths) * ai, thr = th / D, azr = az / D;
    const pc = add3(c, add3(scl3(nz, d * Math.cos(thr)), scl3(add3(scl3(ex, Math.cos(azr)), scl3(ey, Math.sin(azr))), d * Math.sin(thr))));
    const z = unit(sub3(c, pc));
    // image x along the board's long axis (columns), the sign nearer the start camera's x
    let xr = sub3(ex, scl3(z, dot3(ex, z)));
    if (dot3(xr, xc0) < 0) xr = scl3(xr, -1);
    let x = unit(xr), y = cross(z, x);
    if (roll) { const Rr = rodrigues(scl3(z, roll / D)); x = mv3(Rr, x); y = mv3(Rr, y); }
    let Rc = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
    // rotate the camera about its centre so the board centre lands at the aim pixel
    const dt = unit([(au * size.width - K.cx) / K.fx, (av * size.height - K.cy) / K.fy, 1]);
    const ax = cross(dt, [0, 0, 1]), sa = norm3(ax);
    if (sa > 1e-9) Rc = mm(Rc, rodrigues(scl3(ax, Math.asin(Math.min(1, sa)) / sa)));
    const Rend = mm(Rc, tr(Rec)), pend = sub3(pc, mv3(Rend, tec)), T = make(Rend, pend), Tcam = make(Rc, pc);
    const vis = visibleMarkers(K, Tcam, Tbb, board, { size });
    if (vis.length < minMarkers) { stats.markers++; continue; }
    if (toolPoints(T, X).some((p) => dot3(sub3(p, c), nz) < clearance)) { stats.clearance++; continue; }
    const s = ik(T, q0);
    if (!s || !s.ok || (lo && s.q.some((v, i) => v < lo[i] - 1e-6 || v > hi[i] + 1e-6))) { stats.reach++; continue; }
    if (exclude.some((E) => norm3(sub3(posOf(E), pend)) < 0.01 && rotDist(rotOf(E), Rend) < 3)) continue;
    cand.push({ T, q: s.q, camPos: pc, dist: d, tiltDeg: th, azDeg: az, rollDeg: roll, aim: [au, av], markers: vis.length,
      f: [th / 30 * Math.cos(azr), th / 30 * Math.sin(azr), (d - 0.265) / 0.085, roll / 15, (au - 0.5) * 2.5, (av - 0.5) * 2.5] });
  }
  // farthest-point selection, seeded with the candidate nearest to the start pose
  const fd = (a, b) => Math.hypot(...a.f.map((v, k) => v - b.f[k]));
  const qd = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
  const sel = [];
  if (cand.length) {
    let first = cand.reduce((b, x) => (qd(x.q, q0) < qd(b.q, q0) ? x : b));
    sel.push(first);
    const dmin = cand.map((x) => fd(x, first));
    while (sel.length < Math.min(n * 2, cand.length)) {
      let k = -1, best = -1;
      for (let i = 0; i < cand.length; i++) if (dmin[i] > best && !sel.includes(cand[i])) { best = dmin[i]; k = i; }
      if (k < 0 || best <= 1e-9) break;
      sel.push(cand[k]);
      for (let i = 0; i < cand.length; i++) dmin[i] = Math.min(dmin[i], fd(cand[i], cand[k]));
    }
  }
  // nearest-neighbour tour from the start, each transition checked; up to n poses
  const poses = [];
  let curT = T0, curQ = q0.slice();
  const left = sel.slice();
  while (poses.length < n && left.length) {
    left.sort((a, b) => qd(a.q, curQ) - qd(b.q, curQ));
    let took = -1;
    for (let i = 0; i < left.length && took < 0; i++) {
      const tr2 = checkTransition(curT, curQ, left[i].T, { ik, X, plane, clearance, lo, hi });
      if (tr2.ok) { took = i; left[i].q = tr2.q; } else stats.transition++;
    }
    if (took < 0) break;
    const p = left.splice(took, 1)[0];
    poses.push(p); curT = p.T; curQ = p.q;
  }
  return { poses: poses.map(({ f, ...p }) => p), stats: { ...stats, feasible: cand.length, selected: poses.length }, plane };
}

// bootstrap: board pose in base from the first capture with nominal values
export function bootstrapBoard(view, board, { K = NOMINAL, X, G }) {
  const order = detectCornerOrder([view], board).order;
  const { obj, img } = correspondences({ markers: boardMarkers(view, board) }, board, order);
  const p = solvePnP(obj, img, K);
  if (!p) return null;
  const M = make(p.R, p.t);
  return { Tbb: mm(mm(G, X), M), M, rms: p.rms, order, markers: obj.length / 4 };
}
// nominal R_end_cam from the legacy mount (VIK.camMount(flip, rot) then Rx(tilt)) or q_end_cam
export function mountRotation(VIK, mount, qRef) {
  if (mount && Array.isArray(mount.q_end_cam) && mount.q_end_cam.length === 4) return quatR(mount.q_end_cam);
  const R = VIK.camMount(!!(mount && mount.flip), (mount && Number(mount.rot_deg)) || 0, qRef);
  const tl = ((mount && Number(mount.tilt_deg)) || 0) / D;
  return tl ? mm(R, rodrigues([tl, 0, 0])) : R;
}

// ---- capture run loop (cockpit wiring in index.html, tested in scripts/calib_wizard_selftest.mjs)

// board plane (point, unit normal pointing up) from T_base_board, as used by generatePoses
export function boardPlane(Tbb) {
  const Rb = rotOf(Tbb);
  const nz = [Rb[0][2], Rb[1][2], Rb[2][2]];
  return { p: posOf(Tbb), n: nz[2] < 0 ? scl3(nz, -1) : nz };
}
/* move_to refusals that armd returns as a command_reject before any motion starts (planning in
   src/motion.rs plan_cartesian_ex, MotionError::code):
     IK_UNREACHABLE       no IK solution along the straight-line path
     IK_BRANCH_FLIP       a joint jumps > 0.10 rad between path samples
     WORKSPACE_VIOLATION  a check point leaves the workspace box / floor or enters a keep-out box
   Only these skip a pose. Everything else aborts the run: INVALID_COMMAND / UNSUPPORTED / POWER_BUDGET /
   PLAN_STALE / MOTION_BUSY / NOT_ARMED / lease and epoch rejects, a command timeout (no code), and every
   failure of a started action (TRACKING_STALL, COLLISION, STOPPED, SUPERSEDED, FOLLOWING_ERROR,
   TORQUE_LIMIT, POWER_LIMIT, WATCHDOG_TIMEOUT, faults, MOTION_TIMEOUT, ...). */
export const PLAN_REFUSAL_CODES = Object.freeze(['IK_UNREACHABLE', 'IK_BRANCH_FLIP', 'WORKSPACE_VIOLATION']);
// err: { code, phase } with phase 'reject' (command_reject, nothing moved) or 'action' (action record)
export const moveFailureSkips = (err) => !!err && err.phase === 'reject' && PLAN_REFUSAL_CODES.includes(err.code);
// safety states a normal run passes through
export const RUN_STATES = Object.freeze(['Armed', 'Moving']);
/* Why a running wizard must stop now (null: carry on). s: { stopReq, hidden, live (telemetry + WS),
   lease0 / lease (lease generation at Start / now), epoch0 / epoch (observation epoch_id at Start / now),
   state (safety state name), fault (observation fault or null), hold (collision hold active), action / actionState (id and state of
   the newest action in the observation), ownAction (id of the wizard's last move_to) }. Only a RUNNING action can be foreign: armd
   keeps the last finished action (done / cancelled) in the observation until the next one starts, and the move_to ack arrives
   before the observation that shows the new action. */
export function runAbortReason(s) {
  if (s.stopReq) return 'stop';
  if (s.hidden) return 'hidden';
  if (!s.live) return 'heartbeat';
  if (s.lease0 != null && s.lease !== s.lease0) return 'lease';
  if (s.state === 'EStop' || s.state === 'Fault' || s.fault || s.hold) return 'fault';
  if (s.epoch0 != null && s.epoch !== s.epoch0) return 'epoch';
  if (!RUN_STATES.includes(s.state)) return 'state';
  if (s.actionState === 'running' && s.action != null && s.ownAction != null && s.action !== s.ownAction) return 'foreign';
  return null;
}
export const runAbort = (why, detail = '') => Object.assign(new Error(detail ? `${why}: ${detail}` : why), { code: 'ABORTED', why, detail });
/* Visit poses from st.idx (st: { plan: [{ T, state, why }], idx }), one pose or all (runAll). io:
     abortReason() -> why | null       checked before each step and after settle / capture
     check(T) -> { ok, reason }        clearance / IK continuity from the arm's current pose to T
     move(T)                           resolves when the action is done; throws { code, phase }
     settle(), capture() -> view       settle throws { code: 'TIMEOUT' } after its timeout
     accept(view, i, pose) -> bool     stores an accepted view, sets pose.state / why
     after(i)                          per-pose hook (refinement, render)
     stop()                            motion stop; sent on every abort except a lost lease
     skipWhy(kind, detail) -> string   label of a skipped pose
   Returns 'done' (plan finished) or 'paused' (one pose in confirm mode); throws an ABORTED error. */
export async function runPoses(st, io, { runAll = true } = {}) {
  const guard = () => { const why = io.abortReason(); if (why) throw runAbort(why); };
  try {
    do {
      guard();
      const i = st.idx, p = st.plan[i];
      const c = io.check(p.T);
      if (!c.ok) { // planning refusal on the actual transition: skip, try the next pose from here
        p.state = 'skip'; p.why = io.skipWhy('transition', c.reason); st.idx++;
        if (st.idx >= st.plan.length) throw runAbort('nopose', c.reason);
        continue;
      }
      try { await io.move(p.T, i); }
      catch (e) {
        if (e && e.code === 'ABORTED') throw e;
        if (!moveFailureSkips(e)) throw runAbort('move', (e && (e.code || e.message)) || 'unknown');
        p.state = 'skip'; p.why = io.skipWhy('move', e.code); st.idx++;
        continue;
      }
      guard();
      await io.settle();
      guard();
      let v = await io.capture(i);
      if (v.err === 'moved') { guard(); await io.settle(); v = await io.capture(i); }
      guard();
      io.accept(v, i, p);
      st.idx++;
      io.after(i);
    } while (runAll && st.idx < st.plan.length);
    return st.idx >= st.plan.length ? 'done' : 'paused';
  } catch (e) {
    const err = e && e.code === 'ABORTED' ? e : runAbort(e && e.code === 'TIMEOUT' ? 'timeout' : 'error', (e && (e.message || e.code)) || '');
    if (err.why !== 'lease') { try { await io.stop(); } catch (_) {} }
    throw err;
  }
}
