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

/* Step generators: the long solvers are written as generators that yield at points a few ms apart, so the same
   code runs synchronously (runSync), in a Web Worker, or on the page's main thread in slices (runSliced, awaits a
   macrotask whenever a slice exceeds sliceMs) without blocking lease heartbeats. */
export function runSync(gen) { for (;;) { const r = gen.next(); if (r.done) return r.value; } }
export async function runSliced(gen, { sliceMs = 40, now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now()), pause = () => new Promise((r) => setTimeout(r, 0)) } = {}) {
  let t0 = now();
  for (;;) {
    const r = gen.next();
    if (r.done) return r.value;
    if (now() - t0 > sliceMs) { await pause(); t0 = now(); }
  }
}
// Cholesky of a flat n x n SPD matrix; null when not positive definite
const chol = (A, n) => runSync(cholSteps(A, n));
function* cholSteps(A, n) {
  const L = new Float64Array(n * n);
  for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
    if (j === 0 && n > 100) yield;
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

// fingertip sticker (T20; constants and measurement in the "fingertip sticker" section below)
// printable SVG at 1:1 scale (mm units): four stickers (solid outline to cut along, heavy bottom edge = tip edge),
// a 50 mm ruler, the instructions in both languages
export function stickerSvg(st = STICKER, { count = 4, title = 'reBot fingertip sticker / 指尖贴纸' } = {}) {
  const W = st.widthMm, H = st.heightMm, r = st.dotMm / 2, gapX = 14, margin = 10, topY = margin + 30;
  const pageW = Math.max(2 * margin + count * W + (count - 1) * gapX, 120), pageH = topY + H + 26;
  const lines = [
    'Print at 100 % (no fit-to-page). Check the 50 mm ruler. 打印比例 100%（不要缩放），用 50 mm 标尺核对。',
    'Cut along the thin outline. Heavy edge flush with the fingertip, dots toward the wrist. 沿细线剪下，粗边与指尖平齐，圆点朝手腕。',
    'The sticker must lie flat on the camera-facing finger face. 贴纸要平贴在朝向相机的手指面上。',
  ];
  let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${pageW}mm" height="${pageH}mm" viewBox="0 0 ${pageW} ${pageH}">`;
  s += `<rect width="${pageW}" height="${pageH}" fill="#fff"/>`;
  s += `<text x="${margin}" y="${margin + 4}" font-size="4.5" font-family="sans-serif">${title}</text>`;
  lines.forEach((t, i) => { s += `<text x="${margin}" y="${margin + 10 + i * 4.2}" font-size="2.6" font-family="sans-serif">${t}</text>`; });
  for (let k = 0; k < count; k++) {
    const x0 = margin + k * (W + gapX), y1 = topY + H; // y1 = bottom edge (tip)
    s += `<rect x="${x0}" y="${topY}" width="${W}" height="${H}" fill="none" stroke="#888" stroke-width="0.2"/>`;
    for (const L of st.rowsMm) for (const dx of [-st.columnsMm / 2, st.columnsMm / 2]) {
      s += `<circle cx="${(x0 + W / 2 + dx).toFixed(3)}" cy="${(y1 - L).toFixed(3)}" r="${r}" fill="#000"/>`;
    }
    s += `<path d="M ${x0} ${y1} h ${W}" stroke="#000" stroke-width="0.5"/>`;
    s += `<text x="${x0}" y="${y1 + 4}" font-size="2.4" font-family="sans-serif">▲ tip / 指尖</text>`;
  }
  const rx = margin, ry = pageH - 8;
  s += `<path d="M ${rx} ${ry} h 50" stroke="#000" stroke-width="0.4"/>`;
  for (let k = 0; k <= 10; k++) s += `<path d="M ${rx + 5 * k} ${ry} v ${k % 2 ? -2.5 : -4}" stroke="#000" stroke-width="0.3"/>`;
  s += `<text x="${rx + 53}" y="${ry}" font-size="3" font-family="sans-serif">50 mm · dots d ${st.dotMm} mm, rows ${st.rowsMm.join(' / ')} mm, columns ${st.columnsMm} mm</text>`;
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


/* Screen raw views before any solve. A view is dropped when the same marker id appears twice (a second board or a
   look-alike in the image; seen on a real capture: ids 0,0 and 8,8, which made the intrinsics drop 82 of 122
   views and pulled fx from 811 to 824), or when one homography cannot explain its board corners (> maxPx; a
   mis-identified marker: 170 px on the same capture). Returns { views (kept, original objects), rejected:
   [{ i, why: 'duplicate_ids'|'inconsistent', ids?, px? }] }. */
export function screenViews(views, board, { order = null, maxPx = 3 } = {}) {
  const ord = order || detectCornerOrder(views, board).order;
  const kept = [], rejected = [];
  views.forEach((V, i) => {
    const ids = (V.markers || []).map((m) => Number(m.id)).filter((id) => board.byId.has(id));
    const dup = ids.filter((id, k) => ids.indexOf(id) !== k);
    if (dup.length) { rejected.push({ i, why: 'duplicate_ids', ids: [...new Set(dup)] }); return; }
    const { obj, img } = correspondences({ markers: boardMarkers(V, board) }, board, ord);
    if (obj.length >= 8) {
      const H = homography(obj, img);
      if (H) {
        let s = 0;
        obj.forEach((X, k) => { const p = hApply(H, X); s += (p[0] - img[k][0]) ** 2 + (p[1] - img[k][1]) ** 2; });
        const px = Math.sqrt(s / obj.length);
        if (!(px <= maxPx)) { rejected.push({ i, why: 'inconsistent', px }); return; }
      }
    }
    kept.push(V);
  });
  return { views: kept, rejected };
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
export const lmRefine = (views, K0, poses0, opt = {}) => runSync(lmRefineSteps(views, K0, poses0, opt));
export function* lmRefineSteps(views, K0, poses0, opt = {}) {
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
  function* normals() {
    const N = new Float64Array(P * P), g = new Float64Array(P);
    for (let v = 0; v < nv; v++) {
      if (nv > 8) yield;
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
    const { N, g } = yield* normals();
    Nlast = N;
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = N.slice();
      for (let i = 0; i < P; i++) A[i * P + i] += lam * (N[i * P + i] || 1e-12);
      const L = yield* cholSteps(A, P);
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
  const { N } = yield* normals();
  Nlast = N;
  const dof = Math.max(1, 2 * nPts - P), s2 = cost / dof;
  let sigma = null, cond = Infinity, singular = false;
  const L = yield* cholSteps(N, P);
  if (!L) singular = true;
  else if (ni) {
    const C = [];
    for (let j = 0; j < iIdx.length; j++) { const e = new Float64Array(P); e[j] = 1; C.push(cholSolve(L, P, e)); yield; }
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
function* calibrateRawSteps(cv, size, init) {
  const Hs = cv.map((V) => homography(V.obj, V.img));
  if (Hs.some((H) => !H)) return { err: 'homography' };
  let K0 = zhang(Hs, size), method = 'zhang';
  if (!plausible(K0, size)) { K0 = focalCentre(Hs, size); method = 'centre'; }
  if (!plausible(K0, size)) { K0 = { ...(init || NOMINAL) }; method = 'nominal'; }
  K0 = { ...K0, k1: 0, k2: 0 };
  const poses0 = cv.map((V, i) => poseFromH(K0, Hs[i]));
  const r = yield* lmRefineSteps(cv, K0, poses0, { iters: 80 });
  return { ...r, init: K0, method };
}
const gate = (key, ok, value, limit, unit = '') => ({ key, ok: !!ok, value, limit, unit });
/* Intrinsics from board views with the W3 / X1 gates.
   views: [{ markers }] (raw), board. Returns { ok, K, rms, perView, sigma, cond, gates, used: [view
   index], dropped: [{ i, rms }], skipped: [{ i, markers }], heldOut: { rms, n }, order }. */
export const calibrateIntrinsics = (views, board, opt = {}) => runSync(calibrateIntrinsicsSteps(views, board, opt));
export function* calibrateIntrinsicsSteps(views, board, { size = IMAGE, gates = GATES, init = NOMINAL, order = null } = {}) {
  const ord = order || detectCornerOrder(views, board).order;
  const skipped = [], idx = [];
  views.forEach((V, i) => { const n = boardMarkers(V, board).length; if (n < gates.minMarkers) skipped.push({ i, markers: n }); else idx.push(i); });
  const cvOf = (ids) => ids.map((i) => correspondences({ markers: boardMarkers(views[i], board) }, board, ord));
  const out = { ok: false, order: ord, skipped, dropped: [], gates: [] };
  if (idx.length < 4) { out.gates.push(gate('views', false, idx.length, gates.minViews)); out.err = 'few_views'; return out; }
  let used = idx, r = yield* calibrateRawSteps(cvOf(used), size, init);
  if (r.err) { out.err = r.err; out.gates.push(gate('views', false, idx.length, gates.minViews)); return out; }
  // drop views above the per-view limit once, then solve again
  const bad = used.filter((_, k) => !(r.perView[k] <= gates.viewRmsPx));
  if (bad.length && used.length - bad.length >= 4) {
    out.dropped = bad.map((i) => ({ i, rms: r.perView[used.indexOf(i)] }));
    used = used.filter((i) => !bad.includes(i));
    r = yield* calibrateRawSteps(cvOf(used), size, init);
    if (r.err) { out.err = r.err; return out; }
  }
  // held out: solve without every 5th used view, predict its corners (pose by PnP with those intrinsics)
  let held = { rms: NaN, n: 0 };
  const hold = used.filter((_, k) => k % 5 === 4), keep = used.filter((_, k) => k % 5 !== 4);
  if (hold.length && keep.length >= 4) {
    const rk = yield* calibrateRawSteps(cvOf(keep), size, init);
    if (!rk.err) {
      let s = 0, n = 0;
      for (const V of cvOf(hold)) { yield; const p = solvePnP(V.obj, V.img, rk.K); if (p) { s += p.rms * p.rms * V.obj.length; n += V.obj.length; } else { s = Infinity; } }
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
  const screened = screenViews(views, board);
  views = screened.views;
  const intr = calibrateIntrinsics(views, board, { size, gates, init });
  intr.rejected = screened.rejected;
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
export const generatePoses = (args) => runSync(generatePosesSteps(args));
export function* generatePosesSteps({ Tbb, board, K = NOMINAL, X, q0, fk, ik, lo = null, hi = null, n = 20, size = IMAGE,
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
    yield;
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
      yield;
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

// ---------------------------------------------------------------- kinematic fine calibration
/* Fine calibration ("精校"): the camera views of two board placements A and B also identify the arm's geometry.
   Correction model (armd schema kinematics_correction, index.html CALM.fk, src/kinematics.rs): per joint i
   (0..5), walking the URDF chain, M = M F_i [Rx(tilt_x) Ry(tilt_y), origin_m] Rot(axis_i, q_i (1 + scale) +
   offset_rad). Parameter vector k (42): per joint 7 i + [offset, scale, tilt_x, tilt_y, dx, dy, dz].
   Level L3 identifies offset, scale, tilt x / y and origin dx / dy / dz of joints J2..J5 (28 parameters); J1's
   offset is absorbed by the board pose, J6's by the camera mount, both stay zero. Unknowns of one fit: X =
   T_end_cam (6), one board pose per placement (6 each), the kinematic parameters; intrinsics fixed.
   Model levels (KIN_LEVELS): L0 none (X and boards only), L1 zero offsets J2..J5, L2 L1 + J2 scale + J2 tilt x / y,
   L3 the full set above. Validation per level: fit on A, predict B (kinematics and X frozen, only B's board pose
   refit) and B -> A; the held-out reprojection error in mm (px * depth / fx). Level choice (chooseKinLevel): the
   smallest mean held-out of both directions among the levels where neither direction is worse than L0 by more than
   KIN_SELECT.maxWorseMm; a level within KIN_SELECT.tieMm of that mean -> the simplest of them. Gates: the chosen
   level's held-out <= 3 mm both ways (L0 chosen: L0's own held-out) and every parameter of the final fit on A + B at
   the chosen level inside the bounds armd accepts. The final fit is what is applied; L0 applies the camera only and
   removes a stored correction (kinematics null). Ported from real2/kincal2.mjs. */
export const KIN_BOUNDS = Object.freeze({ offset_rad: 10 / D, scale: 0.08, tilt_rad: 3 / D, origin_m: 0.03 });
// zeroWarnJoints: joints (1..6) whose zero offset is identifiable, the only ones a "re-zero the motor" warning is
// given for. J5: recovered within 0.01 deg in the synthetic test. J2..J4 (parallel axes) are not: their offsets trade
// against the next joint's origin shift (synthetic J2 error -0.52 deg with the prior, above the 0.5 deg needed).
// heldOutMm: hard limit on the chosen level's held-out error. On the office B601-DM the uncorrected
// held-out after a correct motor zero was 2.9 / 3.3 mm (backlash, J4 hunting): a 3 mm hard gate refused
// a good camera calibration, so 3 mm is a warning (accuracyWarnMm) and 5 mm the refusal.
export const KIN_GATES = Object.freeze({ heldOutMm: 5.0, accuracyWarnMm: 3.0, zeroWarnDeg: 5, zeroWarnJoints: Object.freeze([5]), minViewsPerPlacement: 10, pnpMaxPx: 2 });
export const KIN_PARAM_NAMES = Object.freeze(['offset', 'scale', 'tilt_x', 'tilt_y', 'dx', 'dy', 'dz']);
/* Weak L2 prior on the kinematic parameters (extra residual rows, px): weight * k / sigma, sigma per parameter type
   (offset, scale, tilt x / y, origin dx / dy / dz). L3 is not minimal: zero offsets of the parallel joints J2..J4
   trade against the next joint's origin shift; the prior picks the minimum-norm solution along such directions.
   weight 0 = no prior. `weight` applies to L3, `minimalWeight` to L1 / L2 (minimal sets, the prior only damps).
   L3 weight sweep (scripts/kincal_selftest.mjs, KIN_PRIOR_W), L3 held-out A->B / B->A mm:
     weight            0              1              3              10             30
     synthetic (a)     0.133 / 0.127  0.135 / 0.129  0.167 / 0.154  0.717 / 0.685  2.46 / 3.05
     noise only (e)    3.30 / 6.65    3.31 / 6.65    3.31 / 6.65    3.35 / 5.94    3.28 / 4.44
     real2 replay (b)  2.007 / 2.237  2.006 / 2.232  1.999 / 2.196  1.981 / 1.999  1.95 / 1.88
   Weight 3: the largest of these with the synthetic recovery check (< 0.5 mm) passing; L0 is chosen in (e) at every
   weight (its uncorrected held-out: 2.52 / 2.99 mm). */
export const KIN_PRIOR = Object.freeze({ weight: 3, minimalWeight: 1, sigma: [1 / D, 0.01, 1 / D, 1 / D, 0.005, 0.005, 0.005] });
export const KIN_L3 = Object.freeze([...[1, 2, 3, 4].map((j) => 7 * j), ...[1, 2, 3, 4].flatMap((j) => [1, 2, 3, 4, 5, 6].map((p) => 7 * j + p))]);
export const KIN_L1 = Object.freeze([1, 2, 3, 4].map((j) => 7 * j));
export const KIN_L2 = Object.freeze([...KIN_L1, 7 + 1, 7 + 2]);
// prior: L3 uses KIN_PRIOR.weight, the minimal levels L1 / L2 KIN_PRIOR.minimalWeight
export const KIN_LEVELS = Object.freeze([
  Object.freeze({ name: 'L0', kIdx: Object.freeze([]), prior: null }),
  Object.freeze({ name: 'L1', kIdx: KIN_L1, prior: 'minimal' }),
  Object.freeze({ name: 'L2', kIdx: KIN_L2, prior: 'minimal' }),
  Object.freeze({ name: 'L3', kIdx: KIN_L3, prior: 'full' }),
]);
export const KIN_SELECT = Object.freeze({ maxWorseMm: 0.1, tieMm: 0.1 });
/* Level choice. rows: [{ ab, ba }] (held-out mm per direction), rows[0] = L0. A level is eligible when neither
   direction is worse than L0 by more than maxWorseMm; among eligible levels the smallest mean (ab + ba) / 2 wins,
   and the simplest eligible level within tieMm of that mean is chosen. Returns { index, eligible, mean }. */
export function chooseKinLevel(rows, { maxWorseMm = KIN_SELECT.maxWorseMm, tieMm = KIN_SELECT.tieMm } = {}) {
  const l0 = rows[0], mean = rows.map((r) => (r.ab + r.ba) / 2);
  // rows[i].oob: the level's fit left the parameter bounds (on A, B or A + B) -> not eligible
  const eligible = rows.map((r, i) => i === 0 || (!r.oob && Number.isFinite(mean[i]) && r.ab <= l0.ab + maxWorseMm && r.ba <= l0.ba + maxWorseMm));
  let best = 0;
  for (let i = 1; i < rows.length; i++) if (eligible[i] && mean[i] < mean[best]) best = i;
  let index = best;
  for (let i = 0; i < best; i++) if (eligible[i] && mean[i] < mean[best] + tieMm) { index = i; break; }
  return { index, eligible, mean };
}
const NK = 42;
// flat 3x3 helpers (row-major)
const f3 = (R) => [R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]];
const mf = (A, B) => [
  A[0] * B[0] + A[1] * B[3] + A[2] * B[6], A[0] * B[1] + A[1] * B[4] + A[2] * B[7], A[0] * B[2] + A[1] * B[5] + A[2] * B[8],
  A[3] * B[0] + A[4] * B[3] + A[5] * B[6], A[3] * B[1] + A[4] * B[4] + A[5] * B[7], A[3] * B[2] + A[4] * B[5] + A[5] * B[8],
  A[6] * B[0] + A[7] * B[3] + A[8] * B[6], A[6] * B[1] + A[7] * B[4] + A[8] * B[7], A[6] * B[2] + A[7] * B[5] + A[8] * B[8]];
const mvf = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const mtvf = (R, v) => [R[0] * v[0] + R[3] * v[1] + R[6] * v[2], R[1] * v[0] + R[4] * v[1] + R[7] * v[2], R[2] * v[0] + R[5] * v[1] + R[8] * v[2]];
const axisF = (k, a) => { const c = Math.cos(a), s = Math.sin(a), C = 1 - c, [x, y, z] = k; return [c + x * x * C, x * y * C - z * s, x * z * C + y * s, y * x * C + z * s, c + y * y * C, y * z * C - x * s, z * x * C - y * s, z * y * C + x * s, c + z * z * C]; };
// chain: CALM.CHAIN ([[F (4x4), axis or null]]) -> flat links
export const kinChain = (chain) => chain.map(([F, axis]) => ({ R: f3(rotOf(F)), p: posOf(F), axis }));
// T_base_end of model angles q under parameter vector k (42) -> { R (flat), p }
export function kinFk(ch, q, k) {
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1], p = [0, 0, 0], i = 0;
  for (const L of ch) {
    const d = mvf(R, L.p);
    p = [p[0] + d[0], p[1] + d[1], p[2] + d[2]];
    R = mf(R, L.R);
    if (!L.axis) continue;
    const b = 7 * i;
    if (k[b + 2] || k[b + 3] || k[b + 4] || k[b + 5] || k[b + 6]) {
      const e = mvf(R, [k[b + 4], k[b + 5], k[b + 6]]);
      p = [p[0] + e[0], p[1] + e[1], p[2] + e[2]];
      R = mf(R, mf(axisF([1, 0, 0], k[b + 2]), axisF([0, 1, 0], k[b + 3])));
    }
    R = mf(R, axisF(L.axis, (Number(q[i]) || 0) * (1 + k[b + 1]) + k[b]));
    i++;
  }
  return { R, p };
}
// parameter vector <-> schema joints [{ offset_rad, scale, tilt_rad: [x, y], origin_m: [dx, dy, dz] }] x 6
export const kinJoints = (k) => [0, 1, 2, 3, 4, 5].map((j) => ({ offset_rad: k[7 * j] || 0, scale: k[7 * j + 1] || 0, tilt_rad: [k[7 * j + 2] || 0, k[7 * j + 3] || 0], origin_m: [k[7 * j + 4] || 0, k[7 * j + 5] || 0, k[7 * j + 6] || 0] }));
export function kinVector(joints) {
  const k = new Array(NK).fill(0);
  (joints || []).slice(0, 6).forEach((j, i) => {
    if (!j) return;
    k[7 * i] = Number(j.offset_rad) || 0; k[7 * i + 1] = Number(j.scale) || 0;
    (j.tilt_rad || []).slice(0, 2).forEach((v, a) => { k[7 * i + 2 + a] = Number(v) || 0; });
    (j.origin_m || []).slice(0, 3).forEach((v, a) => { k[7 * i + 4 + a] = Number(v) || 0; });
  });
  return k;
}
// parameters outside KIN_BOUNDS: [{ joint (1..6), param, value, limit }]
export function kinOutOfBounds(k, bounds = KIN_BOUNDS) {
  const lim = [bounds.offset_rad, bounds.scale, bounds.tilt_rad, bounds.tilt_rad, bounds.origin_m, bounds.origin_m, bounds.origin_m], out = [];
  for (let i = 0; i < NK; i++) if (!(Math.abs(k[i]) <= lim[i % 7] + 1e-12)) out.push({ joint: Math.floor(i / 7) + 1, param: KIN_PARAM_NAMES[i % 7], value: k[i], limit: lim[i % 7] });
  return out;
}

/* One residual problem: obs [{ obj, img, q, ds }], nds board poses. x = [X rotation vector (relative to RX0), X
   position (both left out when Xfix is given), per board: rotation vector (relative to RB0[d]), position,
   kinematic parameters kIdx of kFull]. col(c): what column c moves ('x', 'b' with its placement, 'k'). */
function kinProblem(obs, nds, ch, K, RX0, RB0, kFull, kIdx, Xfix = null, priorW = null) {
  const nx = Xfix ? 0 : 6, np = nx + 6 * nds + kIdx.length, off = [];
  let m = 0;
  for (const o of obs) { off.push(m); m += 2 * o.obj.length; }
  // prior rows (one per kinematic parameter, after the corner rows): priorW[i] * k[kIdx[i]]
  const mObs = m;
  if (priorW) m += kIdx.length;
  const kOf = (x) => { const k = kFull.slice(); kIdx.forEach((ki, i) => { k[ki] = x[nx + 6 * nds + i]; }); return k; };
  const fks = (k) => obs.map((o) => kinFk(ch, o.q, k));
  const col = (c) => (c < nx ? { kind: 'x' } : c < nx + 6 * nds ? { kind: 'b', d: Math.floor((c - nx) / 6) } : { kind: 'k' });
  const RX0f = f3(RX0), RB0f = RB0.map(f3);
  const Xof = (x) => (Xfix ? { R: f3(rotOf(Xfix)), t: posOf(Xfix) } : { R: mf(RX0f, f3(rodrigues([x[0], x[1], x[2]]))), t: [x[3], x[4], x[5]] });
  // residuals into out (rows of the views of placement `only`, or all when only < 0)
  function fill(out, x, F, only = -1) {
    const { R: RX, t: tX } = Xof(x), B = [];
    for (let d = 0; d < nds; d++) { const b = nx + 6 * d; B.push({ R: mf(RB0f[d], f3(rodrigues([x[b], x[b + 1], x[b + 2]]))), t: [x[b + 3], x[b + 4], x[b + 5]] }); }
    for (let n = 0; n < obs.length; n++) {
      const o = obs[n];
      if (only >= 0 && o.ds !== only) continue;
      const E = F[n], Rc = mf(E.R, RX), e = mvf(E.R, tX), pc = [E.p[0] + e[0], E.p[1] + e[1], E.p[2] + e[2]];
      const Bd = B[o.ds];
      // T_cam_board = (T_base_end X)^-1 B
      const Rt = [Rc[0], Rc[3], Rc[6], Rc[1], Rc[4], Rc[7], Rc[2], Rc[5], Rc[8]];
      const R = mf(Rt, Bd.R), t = mtvf(Rc, [Bd.t[0] - pc[0], Bd.t[1] - pc[1], Bd.t[2] - pc[2]]);
      let w = off[n];
      for (let j = 0; j < o.obj.length; j++) {
        const P = o.obj[j], I = o.img[j];
        const xc = R[0] * P[0] + R[1] * P[1] + R[2] * P[2] + t[0], yc = R[3] * P[0] + R[4] * P[1] + R[5] * P[2] + t[1], zc = R[6] * P[0] + R[7] * P[1] + R[8] * P[2] + t[2];
        const u = xc / zc, v = yc / zc, r2 = u * u + v * v, dd = 1 + (K.k1 || 0) * r2 + (K.k2 || 0) * r2 * r2;
        out[w++] = K.fx * u * dd + K.cx - I[0]; out[w++] = K.fy * v * dd + K.cy - I[1];
      }
    }
    if (priorW && only < 0) for (let i = 0; i < kIdx.length; i++) out[mObs + i] = priorW[i] * x[nx + 6 * nds + i];
    return out;
  }
  const unpack = (x) => ({
    X: Xfix || make(mm(RX0, rodrigues([x[0], x[1], x[2]])), [x[3], x[4], x[5]]),
    B: RB0.map((R, d) => { const b = nx + 6 * d; return make(mm(R, rodrigues([x[b], x[b + 1], x[b + 2]])), [x[b + 3], x[b + 4], x[b + 5]]); }),
    k: kOf(x),
  });
  return { np, m, nds, kIdx, kOf, fks, fill, unpack, col };
}
// Levenberg-Marquardt with a central-difference Jacobian (step 1e-6); FK is reused for the X / board columns and
// a board column only touches its own placement's rows. Stops when a step improves the cost by < 1e-9 (relative).
function* kinLM(P, x0, iters = 60) {
  const h = 1e-6, np = P.np, m = P.m;
  let x = x0.slice(), F = P.fks(P.kOf(x)), r = P.fill(new Float64Array(m), x, F);
  let cost = r.reduce((s, e) => s + e * e, 0), lam = 1e-3, it = 0;
  for (; it < iters; it++) {
    const J = [];
    for (let c = 0; c < np; c++) {
      yield;
      const xp = x.slice(), xm = x.slice(); xp[c] += h; xm[c] -= h;
      const cc = P.col(c);
      let rp, rm;
      if (cc.kind === 'k') { rp = P.fill(new Float64Array(m), xp, P.fks(P.kOf(xp))); rm = P.fill(new Float64Array(m), xm, P.fks(P.kOf(xm))); }
      else { const d = cc.kind === 'b' ? cc.d : -1; rp = P.fill(new Float64Array(m), xp, F, d); rm = P.fill(new Float64Array(m), xm, F, d); }
      for (let i = 0; i < m; i++) rp[i] = (rp[i] - rm[i]) / (2 * h);
      J.push(rp);
    }
    const N = new Float64Array(np * np), g = new Float64Array(np);
    for (let a = 0; a < np; a++) {
      if (a % 4 === 0) yield;
      const Ja = J[a];
      let gs = 0; for (let i = 0; i < m; i++) gs += Ja[i] * r[i]; g[a] = gs;
      for (let b = a; b < np; b++) { const Jb = J[b]; let v = 0; for (let i = 0; i < m; i++) v += Ja[i] * Jb[i]; N[a * np + b] = v; N[b * np + a] = v; }
    }
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = N.slice();
      for (let i = 0; i < np; i++) A[i * np + i] += lam * (N[i * np + i] + 1e-9);
      const L = yield* cholSteps(A, np);
      if (!L) { lam *= 10; continue; }
      yield;
      const dx = cholSolve(L, np, g.map((v) => -v)), x2 = x.map((v, k) => v + dx[k]);
      const F2 = P.kIdx.length ? P.fks(P.kOf(x2)) : F, r2 = P.fill(new Float64Array(m), x2, F2), c2 = r2.reduce((s, e) => s + e * e, 0);
      if (Number.isFinite(c2) && c2 < cost) {
        const rel = (cost - c2) / cost;
        x = x2; F = F2; r = r2; cost = c2; lam = Math.max(lam / 10, 1e-12);
        improved = rel > 1e-9;
        if (!improved) tries = 10;
      } else lam *= 10;
    }
    if (!improved) break;
  }
  return { x, r, cost, iterations: it };
}
// metrics of a residual vector: RMS px, RMS mm at each view's depth (px * depth / fx)
function kinErr(obs, r, K) {
  let o = 0, s = 0, sm = 0, n = 0;
  for (const v of obs) { const k = 2 * v.obj.length; for (let i = 0; i < k; i++) { const e = r[o + i]; s += e * e; sm += (e * v.depth / K.fx * 1000) ** 2; n++; } o += k; }
  return { px: Math.sqrt(s / Math.max(1, n)), mm: Math.sqrt(sm / Math.max(1, n)) };
}
// mean board pose per placement through FK, X and the per-view PnP pose
function kinBoards(obs, nds, ch, k, X) {
  const RX = f3(rotOf(X)), tX = posOf(X);
  return [...Array(nds)].map((_, d) => {
    const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], p = [0, 0, 0];
    let n = 0;
    for (const o of obs) {
      if (o.ds !== d) continue;
      const E = kinFk(ch, o.q, k), Rc = mf(E.R, RX), e = mvf(E.R, tX), pc = [E.p[0] + e[0], E.p[1] + e[1], E.p[2] + e[2]];
      const Rb = mf(Rc, f3(rotOf(o.M))), tb = mvf(Rc, posOf(o.M));
      for (let i = 0; i < 3; i++) { for (let j = 0; j < 3; j++) S[i][j] += Rb[3 * i + j]; p[i] += pc[i] + tb[i]; }
      n++;
    }
    return make(nearestRot(S), p.map((v) => v / Math.max(1, n)));
  });
}
// fit X, boards and kinematic parameters kIdx on obs (ds 0..nds-1) from X0 / boards B0 / parameters k0
function* kinFit(obs, nds, ch, K, X0, B0, k0, kIdx, iters, priorW = null) {
  const P = kinProblem(obs, nds, ch, K, rotOf(X0), B0.map(rotOf), k0, kIdx, null, priorW && kIdx.length ? priorW : null);
  const x0 = [0, 0, 0, ...posOf(X0), ...B0.flatMap((B) => [0, 0, 0, ...posOf(B)]), ...kIdx.map((ki) => k0[ki])];
  const f = yield* kinLM(P, x0, iters), u = P.unpack(f.x);
  return { ...u, r: f.r, iterations: f.iterations, err: kinErr(obs, f.r, K) };
}
// held out: kinematics k and X frozen, only the board pose of `obs` (one placement) refit
function* kinHeldOut(obs, ch, K, X, k) {
  const o1 = obs.map((o) => ({ ...o, ds: 0 })), B0 = kinBoards(o1, 1, ch, k, X);
  const P = kinProblem(o1, 1, ch, K, rotOf(X), B0.map(rotOf), k, [], X);
  const f = yield* kinLM(P, [0, 0, 0, ...posOf(B0[0])], 60);
  return { ...kinErr(o1, f.r, K), views: obs.length };
}

/* Kinematic fine calibration from board views of two placements.
   views: [{ markers, q (model J1..J6, rad), ds (0 = placement A, 1 = B) }]; board (makeBoard); K: intrinsics
   (fixed, from the image-only solve); chain: CALM.CHAIN; X0: T_end_cam start value (nominal or measured mount).
   levels: KIN_LEVELS (or a subset starting with L0). Returns { ok, gates, level (chosen name), levels: [{ name,
   params, ab, ba, mean (mm), eligible, chosen }], joints (schema, J1..J6), k, X, boards: [T_base_board A, B],
   heldOut: { ab, ba (chosen level), ab0, ba0 (L0), improve }, train: { px, mm }, final0 (uncorrected fit on A + B),
   views: { used, A, B, dropped }, warnings, ms }. */
export const solveKinematics = (args) => runSync(solveKinematicsSteps(args));
export function* solveKinematicsSteps({ views, board, K, chain, X0, order = null, levels = KIN_LEVELS, select = KIN_SELECT, gates = KIN_GATES, bounds = KIN_BOUNDS, iters = 80, minMarkers = GATES.minMarkers, prior = KIN_PRIOR, onProgress = null }) {
  const priorOf = (lv) => {
    const w = !prior || !lv.prior ? 0 : lv.prior === 'full' ? prior.weight : (prior.minimalWeight ?? prior.weight);
    return w > 0 ? lv.kIdx.map((ki) => w / prior.sigma[ki % 7]) : null;
  };
  const t0 = Date.now(), ch = kinChain(chain), say = (s) => { if (onProgress) onProgress(s); };
  const ord = order || detectCornerOrder(views, board).order;
  const obs = [], dropped = [];
  for (let i = 0; i < views.length; i++) {
    const v = views[i], bm = boardMarkers(v, board);
    if (i % 8 === 0) yield;
    if (bm.length < minMarkers || !Array.isArray(v.q) || !(v.ds === 0 || v.ds === 1)) { dropped.push({ i, why: 'markers' }); continue; }
    const { obj, img } = correspondences({ markers: bm }, board, ord);
    const p = solvePnP(obj, img, K);
    if (!p || !(p.rms <= gates.pnpMaxPx)) { dropped.push({ i, why: 'pnp', rms: p ? p.rms : null }); continue; }
    obs.push({ obj, img, q: v.q.slice(0, 6).map(Number), ds: v.ds, M: make(p.R, p.t), depth: norm3(p.t), i });
  }
  const A = obs.filter((o) => o.ds === 0), B = obs.filter((o) => o.ds === 1).map((o) => ({ ...o, ds: 0 }));
  const res = { ok: false, gates: [], views: { used: obs.length, A: A.length, B: B.length, dropped }, warnings: [] };
  const enough = A.length >= gates.minViewsPerPlacement && B.length >= gates.minViewsPerPlacement;
  res.gates.push(gate('kin_views', enough, Math.min(A.length, B.length), gates.minViewsPerPlacement));
  if (!enough) { res.ms = Date.now() - t0; return res; }
  const k0 = new Array(NK).fill(0);
  // L0 (X, boards), then every level from the L0 result
  function* fitL0(o, nds, label) { say(label + ' L0'); return yield* kinFit(o, nds, ch, K, X0, kinBoards(o, nds, ch, k0, X0), k0, [], 60); }
  function* fitLevel(o, nds, label, l0, lv) {
    if (!lv.kIdx.length) return l0;
    say(label + ' ' + lv.name);
    return yield* kinFit(o, nds, ch, K, l0.X, l0.B, k0, lv.kIdx, iters, priorOf(lv));
  }
  const fa0 = yield* fitL0(A, 1, 'A'), fb0 = yield* fitL0(B, 1, 'B'), fa = [], fb = [];
  for (const lv of levels) { fa.push(yield* fitLevel(A, 1, 'A', fa0, lv)); fb.push(yield* fitLevel(B, 1, 'B', fb0, lv)); }
  say('held-out');
  const ho = [];
  for (let i = 0; i < levels.length; i++) ho.push({ ab: yield* kinHeldOut(B, ch, K, fa[i].X, fa[i].k), ba: yield* kinHeldOut(A, ch, K, fb[i].X, fb[i].k) });
  // a level whose fit on A or on B leaves the bounds is not eligible; when the chosen level's final fit on A + B
  // leaves them, it is marked and the choice repeated (real 2026-11-19: L2 had the best held-out with a J2 tilt of
  // 15 deg, far outside the 3 deg bound, and the whole solve was refused instead of falling back to L1)
  const rows = ho.map((h, i) => ({ ab: h.ab.mm, ba: h.ba.mm, oob: kinOutOfBounds(fa[i].k, bounds).length + kinOutOfBounds(fb[i].k, bounds).length > 0 }));
  const fin0 = yield* fitL0(obs, 2, 'A+B');
  let pick, c, lv, fin;
  for (;;) {
    pick = chooseKinLevel(rows, select); c = pick.index; lv = levels[c];
    fin = yield* fitLevel(obs, 2, 'A+B', fin0, lv);
    if (c === 0 || kinOutOfBounds(fin.k, bounds).length === 0) break;
    rows[c].oob = true;
  }
  const k = fin.k, oob = kinOutOfBounds(k, bounds), { ab, ba } = ho[c], { ab: ab0, ba: ba0 } = ho[0];
  // improvement of the chosen level over the uncorrected model (information; the level choice is the rule). The
  // uncorrected model is the URDF with armd's model_joint_map, so a zero offset already set there is the baseline.
  const imp = (a, b) => (a > 0 ? 1 - b / a : 0), impAll = imp(ab0.mm + ba0.mm, ab.mm + ba.mm);
  res.gates.push(
    gate('kin_heldout_ab', ab.mm <= gates.heldOutMm, ab.mm, gates.heldOutMm, 'mm'),
    gate('kin_heldout_ba', ba.mm <= gates.heldOutMm, ba.mm, gates.heldOutMm, 'mm'),
    gate('kin_bounds', oob.length === 0, oob.length, 0, 'count'),
  );
  if (Math.max(ab.mm, ba.mm) > (gates.accuracyWarnMm ?? Infinity)) res.warnings.push({ key: 'accuracy', mm: Math.max(ab.mm, ba.mm), limit: gates.accuracyWarnMm });
  for (let j = 0; j < 6; j++) if ((gates.zeroWarnJoints || [5]).includes(j + 1) && Math.abs(k[7 * j]) * D > gates.zeroWarnDeg) res.warnings.push({ key: 'zero', joint: j + 1, deg: k[7 * j] * D });
  Object.assign(res, {
    ok: res.gates.every((g) => g.ok), level: lv.name, k, joints: kinJoints(k), outOfBounds: oob, X: fin.X, boards: fin.B,
    levels: levels.map((l, i) => ({ name: l.name, params: l.kIdx.length, ab: ho[i].ab.mm, ba: ho[i].ba.mm, mean: pick.mean[i], eligible: pick.eligible[i], outOfBounds: !!rows[i].oob, chosen: i === c })),
    heldOut: { ab, ba, ab0, ba0, improve: { ab: imp(ab0.mm, ab.mm), ba: imp(ba0.mm, ba.mm), mean: impAll } }, train: fin.err, final0: { X: fin0.X, boards: fin0.B, err: fin0.err },
    fits: { A: levels.map((l, i) => ({ level: l.name, err: fa[i].err, k: fa[i].k })), B: levels.map((l, i) => ({ level: l.name, err: fb[i].err, k: fb[i].k })) },
    iterations: fin.iterations, order: ord, ms: Date.now() - t0,
  });
  return res;
}

/* Verify view of a loaded calibration session: one capture at the current pose, reprojected with the solved values.
   view { markers }, board, order (corner order of the solve), K, Tend (T_base_end of the view's joints under the solved
   model), X (T_end_cam), boards: [T_base_board] (one per placement; the closest counts). Error: RMS per image
   coordinate (as the held-out error) in px and in mm at the board depth (px * depth / fx). ok: mm <= gate (the held-out
   gate, KIN_GATES.heldOutMm) with >= minMarkers board markers. Returns { ok, px, mm, board, depth, markers, gate }. */
export function verifyView({ view, board, order = ORDERS[0], K, Tend, X, boards, gate = KIN_GATES.heldOutMm, minMarkers = GATES.minMarkers }) {
  const bm = boardMarkers(view, board);
  if (bm.length < minMarkers || !boards || !boards.length) return { ok: false, why: 'markers', markers: bm.length, gate };
  const { obj, img } = correspondences({ markers: bm }, board, order);
  const Ci = inv(mm(Tend, X));
  let best = null;
  boards.forEach((Tbb, d) => {
    const Tcb = mm(Ci, Tbb), R = rotOf(Tcb), t = posOf(Tcb);
    let s = 0;
    for (let j = 0; j < obj.length; j++) { const [u, v] = project(K, R, t, obj[j]); s += (u - img[j][0]) ** 2 + (v - img[j][1]) ** 2; }
    const px = Math.sqrt(s / (2 * obj.length)), depth = norm3(t), e = px * depth / K.fx * 1000;
    if (Number.isFinite(e) && (!best || e < best.mm)) best = { px, mm: e, board: d, depth };
  });
  if (!best) return { ok: false, why: 'projection', markers: bm.length, gate };
  return { ...best, ok: best.mm <= gate, markers: bm.length, gate };
}

/* Fine calibration solve of the wizard (runs in a Web Worker in the cockpit). views: [{ markers, q, ds }] of both
   placements; markerSide (m); chain (CALM.CHAIN); X0 (T_end_cam start value); thickness (m). Intrinsics from the
   image-only solve over A + B (calibrateIntrinsics, its gates apply), then solveKinematics with them; X and the
   table height (mean board z of A and B - thickness) from the final kinematic fit. With level L0 chosen the apply
   body carries `kinematics: null` (camera only; armd removes a stored correction).
   Returns { ok, gates, intr, kin, final: { K, X, boards }, table, apply (body for /api/v1/calibration/apply) }. */
export const solveFine = (args) => runSync(solveFineSteps(args));
export function* solveFineSteps({ views, markerSide = BOARD_DEFAULT.markerSide, chain, X0, thickness = 0.0005, gates = GATES, kinGates = KIN_GATES, prior = KIN_PRIOR, size = IMAGE, init = NOMINAL, solvedAt = null, onProgress = null }) {
  const board = makeBoard({ markerSide });
  const screened = screenViews(views, board);
  views = screened.views;
  if (onProgress) onProgress('intrinsics');
  const intr = yield* calibrateIntrinsicsSteps(views, board, { size, gates, init });
  intr.rejected = screened.rejected;
  const res = { ok: false, intr, gates: [...intr.gates] };
  if (!intr.K) return res;
  const kin = yield* solveKinematicsSteps({ views, board, K: intr.K, chain, X0, order: intr.order, gates: kinGates, prior, onProgress });
  res.kin = kin; res.gates.push(...kin.gates);
  if (!kin.joints) return res;
  const zA = kin.boards[0][2][3], zB = kin.boards[1][2][3], X = kin.X, K = intr.K;
  res.final = { K, X, boards: kin.boards };
  res.table = { board_z_m: [zA, zB], table_z_m: (zA + zB) / 2 - thickness, dz_m: zB - zA };
  res.ok = res.gates.every((g) => g.ok);
  const levels = kin.levels.map((l) => ({ level: l.name, heldout_mm: [l.ab, l.ba], chosen: l.chosen }));
  const fit = { level: kin.level, heldout_mm: [kin.heldOut.ab.mm, kin.heldOut.ba.mm], heldout_uncorrected_mm: [kin.heldOut.ab0.mm, kin.heldOut.ba0.mm], train_mm: kin.train.mm, views: kin.views.used, levels, ...(solvedAt ? { solved_at: solvedAt } : {}) };
  res.apply = {
    intrinsics: { fx: K.fx, fy: K.fy, cx: K.cx, cy: K.cy, k1: K.k1, k2: K.k2, p1: 0, p2: 0, k3: 0, width: size.width, height: size.height, marker_size: board.markerSide },
    camera_mount: { offset_m: posOf(X), q_end_cam: quatOf(rotOf(X)) },
    table_z_m: res.table.table_z_m,
    kinematics: kin.level === 'L0' ? null : { joints: kin.joints, fit },
    rms: { reproj_px: intr.rms, held_out_px: intr.heldOut.rms, kin_level: kin.level, kin_train_px: kin.train.px, kin_train_mm: kin.train.mm, kin_heldout_mm: fit.heldout_mm, kin_levels: levels, views: intr.used.length, sigma: intr.sigma, cond: intr.cond },
    ...(solvedAt ? { solved_at: solvedAt } : {}),
  };
  return res;
}

// ---------------------------------------------------------------- fingertip sticker (T20)
/* A 2 x 2 dot sticker on the camera-facing finger face, its bottom edge flush with the fingertip, dots toward
   the wrist. The camera is ~9 cm from it and heavily defocused, so the dots are measured as darkness-weighted
   centroids (no edges): Otsu threshold in a bottom-centre ROI, connected components, the four largest blobs,
   weights (white - gray) inside each blob dilated by 5 px. The camera looks forward-down: the TOP image row of
   dots is the tip-side row (rows[0] mm from the edge). Planar PnP by IPPE gives two poses; the one with the
   lower reprojection wins and the runner-up must be clearly worse (ambiguity gap). Sticker frame: origin = the
   centre of the bottom edge (the fingertip), x = image left -> right across the columns, y = toward the wrist
   (along the rows), z = x × y. tip_cam = the sticker origin in the camera frame (OpenCV axes). */
export const STICKER = Object.freeze({ dotMm: 4, rowsMm: Object.freeze([10, 16]), columnsMm: 8.8, widthMm: 16, heightMm: 20.5 });
// ROI in content pixels (640 x 360): bottom centre, where the fingertip sits in the image of the v8 holder
export const STICKER_ROI = Object.freeze({ x0: 225, y0: 275, x1: 440, y1: 360 });
export const STICKER_GATES = Object.freeze({ reprojPx: 0.5, gapPx: 0.5, stdMm: 0.5, minFrames: 3, minArea: 60, maxAspect: 3.5 });

export function stickerObject(st = STICKER) {
  const s = st.columnsMm / 2000, [a, b] = st.rowsMm.map((v) => v / 1000);
  return [[-s, a, 0], [s, a, 0], [-s, b, 0], [s, b, 0]];
}

// rows of the 640 x 360 content inside a decoded frame: rebot-vision snapshots may be letterboxed to a square
// (640 x 640, content rows 140..499); other sizes are scaled to 640 wide
export function contentBand(w, h, size = IMAGE) {
  const scale = size.width / w, ch = size.height / scale;
  return { scale, y0: h > ch + 1 ? Math.round((h - ch) / 2) : 0, height: Math.min(h, Math.round(ch)) };
}
// RGBA pixels of a decoded frame -> 8-bit gray of the content image (640 x 360) using ITU-R BT.601 luma
export function grayContent(rgba, w, h, size = IMAGE) {
  const { scale, y0 } = contentBand(w, h, size), W = size.width, H = size.height, g = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) {
    const sy = Math.min(h - 1, y0 + Math.floor(y / scale));
    for (let x = 0; x < W; x++) {
      const sx = Math.min(w - 1, Math.floor(x / scale)), i = (sy * w + sx) * 4;
      g[y * W + x] = Math.round(0.299 * rgba[i] + 0.587 * rgba[i + 1] + 0.114 * rgba[i + 2]);
    }
  }
  return g;
}
// Otsu threshold of 8-bit values (as cv::threshold THRESH_OTSU): the level t maximising the between-class variance
// of {v <= t} and {v > t}
export function otsu(values) {
  const hist = new Float64Array(256);
  for (const v of values) hist[v]++;
  const n = values.length, EPS = 1.1920929e-7;
  let mu = 0; for (let i = 0; i < 256; i++) mu += i * hist[i] / n;
  let q1 = 0, mu1 = 0, best = 0, t = 0;
  for (let i = 0; i < 256; i++) {
    const p = hist[i] / n;
    mu1 *= q1; q1 += p;
    const q2 = 1 - q1;
    if (Math.min(q1, q2) < EPS || Math.max(q1, q2) > 1 - EPS) continue;
    mu1 = (mu1 + i * p) / q1;
    const mu2 = (mu - q1 * mu1) / q2, sigma = q1 * q2 * (mu1 - mu2) ** 2;
    if (sigma > best) { best = sigma; t = i; }
  }
  return t;
}
// percentile with linear interpolation (numpy default)
function percentile(values, p) {
  const a = Float64Array.from(values).sort(), x = (a.length - 1) * p / 100, i = Math.floor(x);
  return i + 1 < a.length ? a[i] + (a[i + 1] - a[i]) * (x - i) : a[i];
}
// 8-connected components of a 0/1 mask -> { labels (Int32Array, 0 = background), comps: [{ label, x, y, w, h, area }] }
export function components(mask, w, h) {
  const labels = new Int32Array(w * h), comps = [], stack = [];
  for (let s = 0; s < w * h; s++) {
    if (!mask[s] || labels[s]) continue;
    const label = comps.length + 1; let x0 = w, y0 = h, x1 = -1, y1 = -1, area = 0;
    labels[s] = label; stack.push(s);
    while (stack.length) {
      const p = stack.pop(), px = p % w, py = (p - px) / w;
      area++; if (px < x0) x0 = px; if (px > x1) x1 = px; if (py < y0) y0 = py; if (py > y1) y1 = py;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const nx = px + dx, ny = py + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const q = ny * w + nx;
        if (mask[q] && !labels[q]) { labels[q] = label; stack.push(q); }
      }
    }
    comps.push({ label, x: x0, y: y0, w: x1 - x0 + 1, h: y1 - y0 + 1, area });
  }
  return { labels, comps };
}
/* The four dot centroids in one content frame (gray 640 x 360). Returns { ok, points: [top-left, top-right,
   bottom-left, bottom-right] (content px), blobs, threshold, why } with why = 'dots' (fewer than four blobs). */
export function detectStickerDots(gray, { size = IMAGE, roi = STICKER_ROI, gates = STICKER_GATES, pad = 5 } = {}) {
  const W = size.width, rw = roi.x1 - roi.x0, rh = roi.y1 - roi.y0, g = new Float32Array(rw * rh), g8 = new Uint8Array(rw * rh);
  for (let y = 0; y < rh; y++) for (let x = 0; x < rw; x++) { const v = gray[(roi.y0 + y) * W + roi.x0 + x]; g[y * rw + x] = v; g8[y * rw + x] = v; }
  const t = otsu(g8), mask = new Uint8Array(rw * rh);
  for (let i = 0; i < mask.length; i++) mask[i] = g8[i] <= t ? 1 : 0;
  const { labels, comps } = components(mask, rw, rh), white = percentile(g, 90), blobs = [];
  for (const c of comps) {
    if (c.area < gates.minArea || c.w / c.h > gates.maxAspect || c.x === 0 || c.x + c.w >= rw) continue;
    const sx = Math.max(c.x - pad, 0), sy = Math.max(c.y - pad, 0), ex = Math.min(c.x + c.w + pad, rw), ey = Math.min(c.y + c.h + pad, rh);
    let s = 0, sxw = 0, syw = 0;
    for (let y = sy; y < ey; y++) for (let x = sx; x < ex; x++) {
      // dilation of the blob by a (2 pad + 1)^2 square inside the window
      let inBlob = false;
      for (let yy = Math.max(sy, y - pad); yy <= Math.min(ey - 1, y + pad) && !inBlob; yy++)
        for (let xx = Math.max(sx, x - pad); xx <= Math.min(ex - 1, x + pad); xx++) if (labels[yy * rw + xx] === c.label) { inBlob = true; break; }
      if (!inBlob) continue;
      const wgt = Math.max(white - g[y * rw + x], 0);
      s += wgt; sxw += wgt * x; syw += wgt * y;
    }
    if (s > 0) blobs.push({ area: c.area, u: sxw / s + roi.x0, v: syw / s + roi.y0, w: c.w, h: c.h });
  }
  blobs.sort((a, b) => b.area - a.area);
  const four = blobs.slice(0, 4);
  if (four.length < 4) return { ok: false, why: 'dots', blobs, threshold: t, points: null };
  four.sort((a, b) => a.v - b.v);
  const top = four.slice(0, 2).sort((a, b) => a.u - b.u), bot = four.slice(2).sort((a, b) => a.u - b.u);
  return { ok: true, points: [...top, ...bot].map((b) => [b.u, b.v]), blobs: four, threshold: t, white };
}

/* IPPE (Collins & Bartoli 2014, "Infinitesimal Plane-Based Pose Estimation") for points on the z = 0 plane.
   obj: [[X, Y, 0]] (n >= 4), nrm: ideal normalised image points. Returns both poses [{ R, t }] (T_cam_obj). */
export function ippe(obj, nrm) {
  const n = obj.length, m = [0, 0];
  for (const p of obj) { m[0] += p[0] / n; m[1] += p[1] / n; }
  const P = obj.map((p) => [p[0] - m[0], p[1] - m[1]]);
  const H = homography(P, nrm);
  if (!H) return [];
  const h22 = H[2][2], Hn = H.map((r) => r.map((v) => v / h22));
  const p = Hn[0][2], q = Hn[1][2];
  const J = [[Hn[0][0] - Hn[2][0] * p, Hn[0][1] - Hn[2][1] * p], [Hn[1][0] - Hn[2][0] * q, Hn[1][1] - Hn[2][1] * q]];
  // Rv: rotation taking the z axis onto the ray of the plane origin
  const tz = unit([p, q, 1]), ax = cross([0, 0, 1], tz), sn = norm3(ax), cs = tz[2];
  let Rv = I3();
  if (sn > 1e-12) {
    const k = scl3(ax, 1 / sn), Kx = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], K2 = mm(Kx, Kx);
    Rv = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + sn * Kx[i][j] + (1 - cs) * K2[i][j]));
  }
  // B = [I2 | -v] Rv[:, 0:2], A = B^-1 J
  const B = [[Rv[0][0] - p * Rv[2][0], Rv[0][1] - p * Rv[2][1]], [Rv[1][0] - q * Rv[2][0], Rv[1][1] - q * Rv[2][1]]];
  const det = B[0][0] * B[1][1] - B[0][1] * B[1][0];
  if (Math.abs(det) < 1e-15) return [];
  const Bi = [[B[1][1] / det, -B[0][1] / det], [-B[1][0] / det, B[0][0] / det]];
  const A = [[Bi[0][0] * J[0][0] + Bi[0][1] * J[1][0], Bi[0][0] * J[0][1] + Bi[0][1] * J[1][1]], [Bi[1][0] * J[0][0] + Bi[1][1] * J[1][0], Bi[1][0] * J[0][1] + Bi[1][1] * J[1][1]]];
  // largest singular value of A
  const fro = A[0][0] ** 2 + A[0][1] ** 2 + A[1][0] ** 2 + A[1][1] ** 2, dA = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  const gamma = Math.sqrt((fro + Math.sqrt(Math.max(fro * fro - 4 * dA * dA, 0))) / 2);
  const R2 = A.map((r) => r.map((v) => v / gamma));
  const b0 = Math.sqrt(Math.max(1 - R2[0][0] ** 2 - R2[1][0] ** 2, 0));
  let b1 = Math.sqrt(Math.max(1 - R2[0][1] ** 2 - R2[1][1] ** 2, 0));
  if (R2[0][0] * R2[0][1] + R2[1][0] * R2[1][1] > 0) b1 = -b1; // columns orthogonal: b0 b1 = -(r00 r01 + r10 r11)
  const sols = [];
  for (const sgn of [1, -1]) {
    const c1 = [R2[0][0], R2[1][0], sgn * b0], c2 = [R2[0][1], R2[1][1], sgn * b1], c3 = cross(c1, c2);
    const Rl = [[c1[0], c2[0], c3[0]], [c1[1], c2[1], c3[1]], [c1[2], c2[2], c3[2]]];
    const R = nearestRot(mm(Rv, Rl));
    // translation: least squares of x (R P + t)_z = (R P + t)_x, y likewise
    const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], rhs = [0, 0, 0];
    P.forEach((pp, i) => {
      const rp = mv3(R, [pp[0], pp[1], 0]), [u, v] = nrm[i];
      for (const [row, b] of [[[1, 0, -u], u * rp[2] - rp[0]], [[0, 1, -v], v * rp[2] - rp[1]]]) {
        for (let a = 0; a < 3; a++) { rhs[a] += row[a] * b; for (let c = 0; c < 3; c++) M[a][c] += row[a] * row[c]; }
      }
    });
    const t = solve3(M, rhs);
    if (!t) continue;
    // back to the object frame: R (X - m) + t = R X + (t - R m)
    const Rm = mv3(R, [m[0], m[1], 0]);
    sols.push({ R, t: [t[0] - Rm[0], t[1] - Rm[1], t[2] - Rm[2]] });
  }
  return sols;
}
function solve3(M, b) {
  const d = (A) => A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  const D0 = d(M);
  if (Math.abs(D0) < 1e-18) return null;
  return [0, 1, 2].map((k) => d(M.map((r, i) => r.map((v, j) => (j === k ? b[i] : v)))) / D0);
}
// reprojection error (px) of a pose as OpenCV's solvePnPGeneric reports it: sqrt(sum of squared residuals / 2n)
export function reprojRms(K, R, t, obj, img) {
  let s = 0;
  obj.forEach((X, i) => { const p = project(K, R, t, X); s += (p[0] - img[i][0]) ** 2 + (p[1] - img[i][1]) ** 2; });
  return Math.sqrt(s / (2 * obj.length));
}
// both IPPE poses of the sticker, best first: [{ R, t, reprojPx, front }]
export function stickerPoses(points, K, st = STICKER) {
  const obj = stickerObject(st), nrm = points.map(([u, v]) => undistort(K, u, v));
  return ippe(obj, nrm).map((s) => ({ ...s, reprojPx: reprojRms(K, s.R, s.t, obj, points), front: obj.every((X) => mv3(s.R, X)[2] + s.t[2] > 0) }))
    .sort((a, b) => (b.front - a.front) || (a.reprojPx - b.reprojPx));
}
/* Fingertip from the dot points of several stationary frames (each [4 x [u, v]]): the mean points give the pose;
   per-frame poses give the spread. Returns { ok, gates, tip_cam_m, reprojPx, gapPx, stdMm, frames, poses } with
   gates [{ key, ok, value, limit }]: frames, front (positive depth), reproj, ambiguity (runner-up gap), std. */
export function measureTip(framePoints, K, { st = STICKER, gates = STICKER_GATES } = {}) {
  const frames = framePoints.filter((p) => Array.isArray(p) && p.length === 4);
  const out = { ok: false, frames: frames.length, gates: [] };
  const gate = (key, ok, value, limit) => { out.gates.push({ key, ok, value, limit }); return ok; };
  if (!gate('frames', frames.length >= gates.minFrames, frames.length, gates.minFrames)) return out;
  const mean = [0, 1, 2, 3].map((k) => [0, 1].map((c) => frames.reduce((s, f) => s + f[k][c], 0) / frames.length));
  const poses = stickerPoses(mean, K, st);
  out.poses = poses.map((p) => ({ t: p.t, reprojPx: p.reprojPx, front: p.front }));
  const best = poses[0], second = poses[1];
  if (!gate('front', !!best && best.front, best ? +best.front : 0, 1)) return out;
  out.tip_cam_m = best.t; out.R = best.R; out.reprojPx = best.reprojPx; out.points = mean;
  out.gapPx = second ? second.reprojPx - best.reprojPx : Infinity;
  const tips = frames.map((f) => { const ps = stickerPoses(f, K, st); return ps.length ? ps[0].t : null; }).filter(Boolean);
  const std = [0, 1, 2].map((c) => { const m = tips.reduce((s, t) => s + t[c], 0) / tips.length; return Math.sqrt(tips.reduce((s, t) => s + (t[c] - m) ** 2, 0) / tips.length); });
  out.stdMm = Math.max(...std) * 1000;
  gate('reproj', best.reprojPx <= gates.reprojPx, best.reprojPx, gates.reprojPx);
  gate('ambiguity', out.gapPx >= gates.gapPx, out.gapPx, gates.gapPx);
  gate('std', out.stdMm <= gates.stdMm, out.stdMm, gates.stdMm);
  out.ok = out.gates.every((g) => g.ok);
  return out;
}
// stale frames: a snapshot repeats the previous frame for a while after motion; keep a frame only when its
// frame_id is newer than the last kept one and its content key (e.g. a hash of the JPEG bytes) is new
export function dedupeFrames(frames) {
  const seen = new Set(), out = [];
  let last = -Infinity;
  for (const f of frames) {
    const id = Number.isFinite(f.frame_id) ? f.frame_id : null;
    if ((id !== null && id <= last) || (f.key != null && seen.has(f.key))) continue;
    if (id !== null) last = id;
    if (f.key != null) seen.add(f.key);
    out.push(f);
  }
  return out;
}

// ---------------------------------------------------------------- hover acceptance (T20)
/* armd's calib_hover job brings the fingertip (tool down) above board corners and records the board markers seen
   there with the measured joints. Here: the board pose T_cam_board per frame from the raw corners (IPPE for one
   marker, homography + LM for more), the fingertip in the board frame p = inv(T_cb) · tip_cam (board z points
   into the table, so the height of the sticker edge above the paper is -p.z), its XY error against the requested
   corner, and h = -p.z - edge_to_low (the tool's lowest point). Working table height: table_z = mean(z_end - h)
   with z_end from the CANDIDATE kinematics at the measured joints (the end_link height at which the lowest point
   touches the paper); the optical board plane (T_base_end · X · T_cb, z of the board origin) is recorded beside it. */
export const HOVER_GATES = Object.freeze({ version: 1, minPoints: 4, spreadM: 0.003, xyM: 0.005, confirmM: 0.003, minPredictedClearanceM: 0.010 });
export const HOVER_PLAN = Object.freeze({ clearanceM: 0.012, approachM: 0.060, count: 6, minCount: 4, unknownEdgeM: 0.005, rpy: Object.freeze([0, Math.PI / 2, 0]) });

// fingertip in the board frame
export const tipInBoard = (T_cb, tipCam) => apply(inv(T_cb), tipCam);

// board pose of one frame: { T (T_cam_board), reprojPx, points } or null
export function fitBoardPose(markers, board, K, order = ORDERS[0]) {
  const { obj, img } = correspondences({ markers: boardMarkers({ markers }, board) }, board, order);
  if (obj.length < 4) return null;
  if (obj.length === 4) {
    const nrm = img.map(([u, v]) => undistort(K, u, v));
    const sols = ippe(obj, nrm).map((s) => ({ ...s, e: reprojRms(K, s.R, s.t, obj, img), front: s.t[2] > 0 })).filter((s) => s.front).sort((a, b) => a.e - b.e);
    if (!sols.length) return null;
    return { T: make(sols[0].R, sols[0].t), reprojPx: sols[0].e, points: 4, gapPx: sols[1] ? sols[1].e - sols[0].e : Infinity };
  }
  const p = solvePnP(obj, img, K);
  if (!p || !(p.t[2] > 0)) return null;
  return { T: make(p.R, p.t), reprojPx: p.rms, points: obj.length };
}

/* One hover point. observations: [{ markers, joints?, q? }]; tipCam (m, camera frame); edgeToLow (m or null);
   target: corner in the board frame [x, y] (m); Tend: T_base_end from the candidate kinematics at the measured
   joints (optional; gives z_end_m and the optical plane); X: T_end_cam of the candidate. */
export function hoverPoint({ observations, board, K, tipCam, edgeToLow = null, target = null, Tend = null, X = null, order = ORDERS[0] }) {
  const fits = (observations || []).map((o) => fitBoardPose(o.markers || [], board, K, order)).filter(Boolean);
  if (!fits.length) return { ok: false, why: 'markers', frames: 0 };
  const ps = fits.map((f) => tipInBoard(f.T, tipCam));
  const p = [0, 1, 2].map((c) => ps.reduce((s, q) => s + q[c], 0) / ps.length);
  const edge = -p[2];
  const out = { ok: true, frames: fits.length, tip_board_m: p, edge_height_m: edge, height_m: edgeToLow == null ? null : edge - edgeToLow,
    reproj_px: Math.max(...fits.map((f) => f.reprojPx)), frame_spread_m: Math.max(...[0, 1, 2].map((c) => Math.max(...ps.map((q) => q[c])) - Math.min(...ps.map((q) => q[c])))) };
  if (target) out.xy_err_m = [p[0] - target[0], p[1] - target[1]];
  if (target) out.target_xy_m = target;
  if (Tend) {
    out.z_end_m = Tend[2][3];
    if (X) { const Tbb = mm(mm(Tend, X), fits[0].T); out.optical_board_z_m = Tbb[2][3]; out.board_normal_z = Tbb[2][2]; }
  }
  return out;
}

/* Gates and the working table height over the hover points (results of hoverPoint with z_end_m). Returns
   { ok, gates, table_z_m (null without a known tool), optical_table_z_m, height_spread_m, max_xy_m, points }. */
export function evaluateHovers(points, { edgeToLow = null, gates = HOVER_GATES } = {}) {
  const ok = points.filter((p) => p && p.ok);
  const out = { gates: [], points: ok.length };
  const gate = (key, pass, value, limit, blocking = true) => out.gates.push({ key, ok: pass, value, limit, blocking });
  gate('points', ok.length >= gates.minPoints, ok.length, gates.minPoints);
  const hs = ok.map((p) => (edgeToLow == null ? p.edge_height_m : p.height_m));
  out.height_spread_m = hs.length ? Math.max(...hs) - Math.min(...hs) : NaN;
  gate('spread', hs.length > 0 && out.height_spread_m <= gates.spreadM, out.height_spread_m, gates.spreadM);
  out.max_xy_m = Math.max(0, ...ok.filter((p) => p.xy_err_m).map((p) => Math.hypot(p.xy_err_m[0], p.xy_err_m[1])));
  gate('xy', out.max_xy_m <= gates.xyM, out.max_xy_m, gates.xyM);
  out.min_height_m = hs.length ? Math.min(...hs) : NaN;
  gate('clearance', hs.length > 0 && out.min_height_m > 0, out.min_height_m, 0);
  gate('tool', edgeToLow != null, edgeToLow == null ? 'unknown' : edgeToLow, 'known edge_to_low_m', false);
  const withZ = ok.filter((p) => Number.isFinite(p.z_end_m));
  out.table_z_m = edgeToLow != null && withZ.length ? withZ.reduce((s, p) => s + p.z_end_m - p.height_m, 0) / withZ.length : null;
  const opt = ok.filter((p) => Number.isFinite(p.optical_board_z_m));
  out.optical_table_z_m = opt.length ? opt.reduce((s, p) => s + p.optical_board_z_m, 0) / opt.length : null;
  out.ok = out.gates.every((g) => g.ok || !g.blocking);
  return out;
}

// confirmation hovers at the new table height: measured lowest-point height within confirmM of the requested clearance
export function confirmHovers(points, clearance, { gates = HOVER_GATES } = {}) {
  const ok = points.filter((p) => p && p.ok && Number.isFinite(p.height_m));
  const errs = ok.map((p) => p.height_m - clearance);
  const worst = errs.length ? Math.max(...errs.map(Math.abs)) : Infinity;
  return { ok: ok.length >= 1 && worst <= gates.confirmM, worst_m: worst, errors_m: errs, points: ok.length };
}

// joint-space IK of the kinematic model (candidate parameters k): damped least squares on position + orientation
export function kinIk(ch, k, T, q0, { lo = null, hi = null, iters = 80, tolM = 1e-5, tolRad = 1e-4 } = {}) {
  let q = q0.slice(0, 6);
  const Rt = f3(rotOf(T)), pt = posOf(T);
  const err = (q) => {
    const f = kinFk(ch, q, k), Re = [f.R[0], f.R[3], f.R[6], f.R[1], f.R[4], f.R[7], f.R[2], f.R[5], f.R[8]]; // R^T
    const E = mf(Re, Rt), w = logRot([[E[0], E[1], E[2]], [E[3], E[4], E[5]], [E[6], E[7], E[8]]]), wb = mvf(f.R, w);
    return [pt[0] - f.p[0], pt[1] - f.p[1], pt[2] - f.p[2], wb[0], wb[1], wb[2]];
  };
  for (let it = 0; it < iters; it++) {
    const e = err(q);
    if (Math.hypot(e[0], e[1], e[2]) < tolM && Math.hypot(e[3], e[4], e[5]) < tolRad) return { q, ok: true };
    const J = [0, 1, 2, 3, 4, 5].map((j) => { const qq = q.slice(); qq[j] += 1e-6; const e2 = err(qq); return e.map((v, r) => (v - e2[r]) / 1e-6); });
    // (J^T J + l I) dq = J^T e, J[j] = column j
    const A = [], b = [];
    for (let i = 0; i < 6; i++) { A.push([]); b.push(J[i].reduce((s, v, r) => s + v * e[r], 0)); for (let j = 0; j < 6; j++) A[i].push(J[i].reduce((s, v, r) => s + v * J[j][r], 0) + (i === j ? 1e-6 : 0)); }
    const L = chol(A.flat(), 6);
    if (!L) return { q, ok: false };
    const dq = cholSolve(L, 6, b);
    q = q.map((v, j) => { let x = v + dq[j]; if (lo) x = Math.max(lo[j], x); if (hi) x = Math.min(hi[j], x); return x; });
  }
  const e = err(q);
  return { q, ok: Math.hypot(e[0], e[1], e[2]) < 1e-4 && Math.hypot(e[3], e[4], e[5]) < 1e-3 };
}
const kinT = (ch, q, k) => { const f = kinFk(ch, q, k); return [[f.R[0], f.R[1], f.R[2], f.p[0]], [f.R[3], f.R[4], f.R[5], f.p[1]], [f.R[6], f.R[7], f.R[8], f.p[2]], [0, 0, 0, 1]]; };
export const kinPose = kinT;

/* Hover targets over board corners. Tbb: T_base_board of the candidate (re-observed stationary board); X: candidate
   T_end_cam; tipCam: fingertip in the camera frame; edgeToLow: the tool's value (null: HOVER_PLAN.unknownEdgeM is
   assumed for the clearance); ch / kCand / kLive: kinematic chain and parameter vectors of the candidate and of the
   model armd runs now; q0: model angles to seed the IK; K: candidate intrinsics (a marker must be in view).
   For each corner the end_link is placed so that the PREDICTED sticker edge (tip_end = X · tip_cam) is above the
   corner and the tool's lowest point clearanceM above the paper; the candidate IK gives joints, the live FK of those
   joints is the target armd moves to. Corners are ranked by spread (farthest point first from the board centre);
   needs count >= minCount and both board axes covered (minCount 1 / count 2 for confirmation hovers, where tableZ
   puts the end_link at tableZ + clearanceM, the working table semantics of evaluateHovers). Returns { ok, why, points: [{ id, corner, approach_m, hover_m,
   rpy_rad, q, predicted_clearance_m, visible }] }. */
export function planHover({ Tbb, X, tipCam, edgeToLow = null, ch, kCand, kLive, q0, K, board, lo = null, hi = null, plan = HOVER_PLAN, gates = HOVER_GATES, size = IMAGE, tableZ = null }) {
  const R = rpyR(...plan.rpy), tipEnd = apply(X, tipCam), drop = edgeToLow == null ? plan.unknownEdgeM : edgeToLow;
  const rTip = mv3(R, tipEnd);
  const cands = [];
  for (const m of board.markers) m.corners.forEach((c, ci) => {
    const pb = apply(Tbb, c);
    // end_link origin: sticker edge above the corner, lowest point clearanceM above the paper
    // tableZ (working table height, confirmation hovers): the end_link at tableZ + clearance
    const pe = [pb[0] - rTip[0], pb[1] - rTip[1], tableZ != null ? tableZ + plan.clearanceM : pb[2] + plan.clearanceM + drop - rTip[2]];
    const T = make(R, pe), Ta = make(R, [pe[0], pe[1], pe[2] + plan.approachM]);
    const ika = kinIk(ch, kCand, Ta, q0, { lo, hi });
    const ikh = ika.ok ? kinIk(ch, kCand, T, ika.q, { lo, hi }) : ika;
    if (!ikh.ok) return;
    const Tl = kinT(ch, ikh.q, kLive), Tla = kinT(ch, ika.q, kLive);
    const vis = visibleMarkers(K, mm(T, X), Tbb, board, { size });
    if (!vis.length) return;
    cands.push({ id: `m${m.id}c${ci}`, corner: c.slice(0, 2), base: pb, q: ikh.q,
      approach_m: [Tl[0][3], Tl[1][3], Tla[2][3]], hover_m: posOf(Tl), rpy_rad: rpyDeg(rotOf(Tl)).map((v) => v / D),
      predicted_clearance_m: plan.clearanceM, visible: vis.map((v) => v.id) });
  });
  // vertical descent in the live frame: approach straight above the hover point
  cands.forEach((c) => { c.approach_m = [c.hover_m[0], c.hover_m[1], Math.max(c.approach_m[2], c.hover_m[2] + 0.02)]; });
  const chosen = [];
  const left = cands.slice();
  let ref = [0, 0];
  while (chosen.length < plan.count && left.length) {
    let bi = 0, bd = -1;
    left.forEach((c, i) => { const d = chosen.length ? Math.min(...chosen.map((s) => Math.hypot(s.corner[0] - c.corner[0], s.corner[1] - c.corner[1]))) : Math.hypot(c.corner[0] - ref[0], c.corner[1] - ref[1]); if (d > bd) { bd = d; bi = i; } });
    chosen.push(left.splice(bi, 1)[0]);
  }
  const xs = chosen.map((c) => c.corner[0]), ys = chosen.map((c) => c.corner[1]);
  const spanOk = plan.minCount < 4 ? chosen.length > 0 : chosen.length && Math.max(...xs) - Math.min(...xs) >= board.width / 2 && Math.max(...ys) - Math.min(...ys) >= board.height / 2;
  const ok = chosen.length >= plan.minCount && spanOk && plan.clearanceM >= gates.minPredictedClearanceM;
  return { ok, why: ok ? null : chosen.length < plan.minCount ? 'reach' : 'spread', points: chosen, candidates: cands.length };
}

/* The apply body of a verified calibration: the candidate's apply body (solveFine) with the fingertip, the
   acceptance record and, when the tool is known and the acceptance passed, the hover table height
   (table_z_source hover; the camera value otherwise). tool: { tool_id, tip_cam_m, x_fingerprint, gripper_opening?,
   fit? }; fingerprint: the candidate's (dry-run after.fingerprint with this tool); evaluation: evaluateHovers. */
export function verifiedApplyBody(apply, { tool, fingerprint, evaluation, points, board, jobId = null, measuredAt = null, confirm = null, gates = HOVER_GATES }) {
  const body = { ...apply, tool: { ...tool } };
  const hover = !!(evaluation.ok && evaluation.table_z_m != null && (!confirm || confirm.ok));
  const r = (v, n = 6) => (Number.isFinite(v) ? +v.toFixed(n) : v);
  body.acceptance = {
    fingerprint, gate_version: gates.version, passed: !!(evaluation.ok && (!confirm || confirm.ok)),
    board: { cols: board.cols, rows: board.rows, marker_side_m: board.markerSide, gap_m: board.gap },
    points: points.slice(0, 16).map((p) => ({ id: p.id, ok: !!p.ok, frames: p.frames, edge_height_m: r(p.edge_height_m), height_m: r(p.height_m), xy_err_m: p.xy_err_m && p.xy_err_m.map((v) => r(v)), z_end_m: r(p.z_end_m), optical_board_z_m: r(p.optical_board_z_m), skipped: p.skipped || undefined })),
    summary: { table_z_m: hover ? evaluation.table_z_m : null, optical_table_z_m: r(evaluation.optical_table_z_m), height_spread_m: r(evaluation.height_spread_m), max_xy_m: r(evaluation.max_xy_m),
      gates: evaluation.gates.map((g) => ({ key: g.key, ok: g.ok, value: typeof g.value === 'number' ? r(g.value) : g.value, limit: g.limit })), confirm: confirm ? { ok: confirm.ok, worst_m: r(confirm.worst_m) } : null },
    ...(jobId ? { job_id: jobId } : {}), ...(measuredAt ? { measured_at: measuredAt } : {}),
  };
  if (hover) { body.table_z_m = evaluation.table_z_m; body.table_z_source = 'hover'; } else body.table_z_source = 'camera';
  return body;
}
