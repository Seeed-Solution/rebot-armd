#!/usr/bin/env node
// Self-test of the self-service verification math in src/api/calib_wizard.js (T20): fingertip sticker and hover
// acceptance.   node scripts/calib_verify_selftest.mjs        (PLAYWRIGHT=/path/to/playwright/index.mjs optional)
//   a. sticker SVG: 4 stickers x 4 dots of d 4 mm at rows 10 / 16 mm, columns 8.8 mm, a 50 mm ruler
//   b. IPPE on exact synthetic points: the true pose is one of the two solutions (< 1e-6 m), ranked first
//   c. ambiguity: a small, nearly fronto-parallel sticker far away has a runner-up within 0.5 px -> gate fails;
//      fewer than 4 dots -> 'dots'; a pose behind the camera -> front gate
//   d. stale frames: duplicate frame ids / identical bytes are dropped before the mean (dedupeFrames)
//   e. real blurred JPEGs (tests/fixtures/calib_t20/f03 f04 f06.jpg, 640x640 letterboxed snapshots of the office
//      B601-DM): decoded by Chromium as the cockpit does (canvas getImageData), dots within 0.15 px of OpenCV,
//      tip_cam within 0.3 mm of OpenCV's IPPE (-0.12, 10.57, 95.42) mm and of the recorded (-0.10, 10.58, 95.41)
//   f. hover: board pose from the recorded marker corners (5 real hovers), height of the sticker edge above the
//      paper within 0.5 mm of 12.2 / 12.6 / 11.6 / 11.3 / 11.3 mm; signed arithmetic: h = -p.z - edge_to_low,
//      table_z = z_end - h (a tip below the paper gives a negative height); gates of evaluateHovers
//   g. hover plan on the B601-DM chain (CALM.CHAIN): >= 4 corners spread over both board axes, the predicted sticker
//      edge (X · tip_cam) above each corner with the lowest point 12 mm above the paper; with a live model that
//      differs from the candidate the targets are the live FK of the candidate IK joints; the apply body
// Exits non-zero when a check fails.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const FX = path.join(root, 'tests/fixtures/calib_t20');
const F = JSON.parse(fs.readFileSync(path.join(FX, 'fixtures.json'), 'utf8'));
let fails = 0;
const check = (name, ok, detail = '') => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
const mm = (v) => v.map((x) => (x * 1000).toFixed(3)).join(', ');
const dist = (a, b) => Math.hypot(...a.map((v, k) => v - b[k]));
const K = F.intrinsics;

// ---- a. SVG
{
  const svg = CW.stickerSvg();
  const circles = [...svg.matchAll(/<circle cx="([\d.]+)" cy="([\d.]+)" r="([\d.]+)"/g)].map((m) => m.slice(1).map(Number));
  const first = circles.slice(0, 4);
  const dx = Math.abs(first[1][0] - first[0][0]), rows = [...new Set(first.map((c) => c[1]))].sort((a, b) => a - b);
  check('a. sticker SVG', circles.length === 16 && circles.every((c) => c[2] === 2) && Math.abs(dx - 8.8) < 1e-9 && Math.abs(rows[1] - rows[0] - 6) < 1e-9 && /h 50"/.test(svg),
    `${circles.length} dots, columns ${dx.toFixed(2)} mm, rows ${(rows[1] - rows[0]).toFixed(2)} mm apart`);
}

// ---- b. IPPE exact
const synth = (R, t, Kx = K) => CW.stickerObject().map((X) => CW.project(Kx, R, t, X).slice(0, 2));
{
  const R = CW.rpyR(1.95, 0.02, -0.01), t = [-0.0001, 0.0106, 0.0954];
  const pts = synth(R, t);
  const sols = CW.stickerPoses(pts, K);
  const e = Math.min(...sols.map((s) => dist(s.t, t)));
  check('b. IPPE recovers the true pose', sols.length === 2 && dist(sols[0].t, t) < 1e-6 && CW.rotDist(sols[0].R, R) < 1e-4,
    `best ${mm(sols[0].t)} mm (err ${(e * 1e6).toFixed(3)} um), reproj ${sols[0].reprojPx.toExponential(2)} / ${sols[1].reprojPx.toFixed(3)} px`);
  const m = CW.measureTip([pts, pts, pts], K);
  check('b. measureTip on exact frames passes', m.ok && m.stdMm < 1e-6, m.gates.map((g) => `${g.key}=${g.ok}`).join(' '));
}

// ---- c. ambiguity, missing dots, behind the camera
{
  const R = CW.rpyR(Math.PI + 0.03, 0.0, 0.0), t = [0.0, 0.0, 0.45];
  const m = CW.measureTip([synth(R, t), synth(R, t), synth(R, t)], K);
  const amb = m.gates.find((g) => g.key === 'ambiguity');
  check('c. ambiguous IPPE (far, fronto-parallel) fails the gap gate', !m.ok && amb && !amb.ok, `gap ${amb && amb.value.toFixed(3)} px`);
  const g = new Uint8Array(640 * 360).fill(200);
  const d = CW.detectStickerDots(g);
  check('c. blank image: no dots', !d.ok && d.why === 'dots', `blobs ${d.blobs.length}`);
  const few = CW.measureTip([synth(CW.rpyR(1.95, 0, 0), [0, 0.01, 0.095])].slice(0, 1), K);
  check('c. one frame < minFrames', !few.ok && few.gates[0].key === 'frames' && !few.gates[0].ok, `${few.frames} frame`);
}

// ---- d. stale frames
{
  const a = { frame_id: 7, key: 'aa', points: [[0, 0], [1, 0], [0, 1], [1, 1]] };
  const kept = CW.dedupeFrames([a, { ...a }, { frame_id: 8, key: 'aa', points: a.points }, { frame_id: 9, key: 'bb', points: a.points }, { frame_id: 6, key: 'cc', points: a.points }]);
  check('d. duplicate / repeated / older frames dropped', kept.length === 2 && kept[0].frame_id === 7 && kept[1].frame_id === 9, `kept ${kept.map((f) => f.frame_id)}`);
}

// ---- e. real JPEGs through Chromium
{
  let chromium = null;
  try { ({ chromium } = await import(process.env.PLAYWRIGHT || 'playwright')); } catch (e) { check('e. playwright available', false, e.message); }
  if (chromium) {
    const browser = await chromium.launch(process.env.CHROME ? { executablePath: process.env.CHROME } : {});
    const page = await browser.newPage();
    await page.setContent('<html><body></body></html>');
    const frames = [];
    for (const f of ['f03', 'f04', 'f06']) {
      const b64 = fs.readFileSync(path.join(FX, `${f}.jpg`)).toString('base64');
      // decoded as the cockpit does: an <img> drawn into a canvas, getImageData
      const rgbaB64 = await page.evaluate(async (b64) => {
        const img = new Image();
        img.src = 'data:image/jpeg;base64,' + b64;
        await img.decode();
        const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
        const ctx = c.getContext('2d', { willReadFrequently: true }); ctx.drawImage(img, 0, 0);
        const d = ctx.getImageData(0, 0, c.width, c.height).data;
        let s = ''; for (let i = 0; i < d.length; i += 0x8000) s += String.fromCharCode.apply(null, d.subarray(i, i + 0x8000));
        return [c.width, c.height, btoa(s)];
      }, b64);
      const [w, h, data] = rgbaB64;
      const gray = CW.grayContent(new Uint8Array(Buffer.from(data, 'base64')), w, h);
      const d = CW.detectStickerDots(gray);
      const ref = F.sticker.points[f];
      const err = d.ok ? Math.max(...d.points.map((p, k) => dist(p, ref[k]))) : Infinity;
      check(`e. ${f}.jpg (${w}x${h}) dots vs OpenCV`, d.ok && err < 0.15, `max ${err.toFixed(3)} px, Otsu ${d.threshold}`);
      if (d.ok) frames.push(d.points);
    }
    await browser.close();
    const m = CW.measureTip(frames, K);
    const ref = F.sticker.tip_cam_mm.map((v) => v / 1000), rec = [-0.10, 10.58, 95.41].map((v) => v / 1000);
    check('e. tip_cam from the real JPEGs', m.ok && dist(m.tip_cam_m, ref) < 0.0003 && dist(m.tip_cam_m, rec) < 0.0003,
      `${mm(m.tip_cam_m)} mm (OpenCV ${mm(ref)}; ${(dist(m.tip_cam_m, ref) * 1000).toFixed(3)} mm), reproj ${m.reprojPx && m.reprojPx.toFixed(3)} px, gap ${m.gapPx && m.gapPx.toFixed(2)} px, std ${m.stdMm && m.stdMm.toFixed(3)} mm`);
  }
}

// ---- f. hover heights from real marker corners
{
  const board = CW.makeBoard();
  const tip = F.hover.tip_cam_mm.map((v) => v / 1000);
  const want = { h1: 12.2, h2: 12.6, h4: 11.6, h5: 11.3, h6: 11.3 };
  for (const [h, p] of Object.entries(F.hover.points)) {
    const r = CW.hoverPoint({ observations: [{ markers: p.markers }], board, K, tipCam: tip, edgeToLow: null });
    const hmm = r.ok ? r.edge_height_m * 1000 : NaN;
    check(`f. ${h} sticker-edge height`, r.ok && Math.abs(hmm - want[h]) <= 0.5 && Math.abs(hmm - p.height_mm) < 0.05,
      `${hmm.toFixed(2)} mm (recorded ${want[h]}, OpenCV ${p.height_mm.toFixed(2)}), markers ${p.markers.map((m) => m.id)}, xy ${r.ok ? mm(r.tip_board_m.slice(0, 2)) : '-'} mm`);
  }
  // signed arithmetic
  const T_cb = CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0.1]); // board 0.1 m in front of the camera, its z (into the table) along the optical axis
  const below = CW.tipInBoard(T_cb, [0.0, 0.0, 0.102]);   // 2 mm beyond the paper
  const above = CW.tipInBoard(T_cb, [0.0, 0.0, 0.090]);
  check('f. signed height: tip beyond the paper is negative', Math.abs(-below[2] - -0.002) < 1e-12 && Math.abs(-above[2] - 0.010) < 1e-12, `${(-below[2] * 1000).toFixed(1)} / ${(-above[2] * 1000).toFixed(1)} mm`);
  const pts = [0.0123, 0.0118, 0.0121, 0.0119].map((e, i) => ({ ok: true, edge_height_m: e, height_m: e - 0.0023, z_end_m: 0.0215 + i * 0.0002, xy_err_m: [0.001, -0.002], target_xy_m: [0, 0] }));
  const ev = CW.evaluateHovers(pts, { edgeToLow: 0.0023 });
  const zWant = pts.reduce((s, p) => s + p.z_end_m - p.height_m, 0) / pts.length;
  check('f. table_z = mean(z_end - h), gates pass', ev.ok && Math.abs(ev.table_z_m - zWant) < 1e-12, `table_z ${(ev.table_z_m * 1000).toFixed(2)} mm spread ${(ev.height_spread_m * 1000).toFixed(2)} mm`);
  const bad = CW.evaluateHovers([...pts, { ...pts[0], edge_height_m: 0.0172, height_m: 0.0149 }], { edgeToLow: 0.0023 });
  check('f. height spread > 3 mm fails', !bad.ok && bad.gates.some((g) => g.key === 'spread' && !g.ok), `spread ${(bad.height_spread_m * 1000).toFixed(2)} mm`);
  const neg = CW.evaluateHovers([...pts, { ...pts[0], edge_height_m: 0.0015, height_m: -0.0008 }], { edgeToLow: 0.0023 });
  check('f. negative clearance fails', !neg.ok && neg.gates.some((g) => g.key === 'clearance' && !g.ok), '');
  const unk = CW.evaluateHovers(pts.map((p) => ({ ...p, height_m: null })), { edgeToLow: null });
  check('f. unknown tool: no table_z', unk.table_z_m === null && unk.gates.some((g) => g.key === 'tool' && !g.ok), unk.gates.map((g) => `${g.key}=${g.ok}`).join(' '));
}


// ---- g. hover plan and apply body
{
  const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
  const block = (name) => html.slice(html.indexOf(`/* ${name}-BEGIN */`), html.indexOf(`/* ${name}-END */`));
  const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', vm.createContext({}));
  const ch = CW.kinChain(CALM.CHAIN), zero = new Array(42).fill(0);
  const board = CW.makeBoard();
  const Tbb = CW.make(CW.mul(CW.rpyR(0, 0, -80 / 180 * Math.PI), CW.rpyR(Math.PI, 0, 0)), [0.30, 0.02, 0.0105]);
  const X = CW.make(CW.mul(CW.mul(VIK.camMount(false, 0), CW.rodrigues([-38 / 180 * Math.PI, 0, 0])), CW.rodrigues([0, 0, 0.026])), [-0.077, 0.005, 0.069]);
  const tip = F.hover.tip_cam_mm.map((v) => v / 1000);
  const q0 = [0, 1.0, -1.2, 0.2, 0, 0];
  const lo = [-3.1, -2.0, -2.6, -3.1, -2.0, -6.2], hi = [3.1, 2.0, 2.6, 3.1, 2.0, 6.2];
  const plan = CW.planHover({ Tbb, X, tipCam: tip, edgeToLow: 0.0023, ch, kCand: zero, kLive: zero, q0, K: F.intrinsics, board, lo, hi });
  let worst = 0, hErr = 0;
  for (const p of plan.points) {
    const T = CW.kinPose(ch, p.q, zero), e = CW.mul(T, X), edge = [T[0][3], T[1][3], T[2][3]].map((v, k) => v + CW.mul(T, CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], CW.posOf(CW.mul(X, CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], tip)))))[k][3] - T[k][3]);
    void e;
    const c = CW.mul(Tbb, CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [...p.corner, 0]));
    worst = Math.max(worst, Math.hypot(edge[0] - c[0][3], edge[1] - c[1][3]));
    hErr = Math.max(hErr, Math.abs(edge[2] - 0.0023 - c[2][3] - 0.012));
  }
  const xs = plan.points.map((p) => p.corner[0]), ys = plan.points.map((p) => p.corner[1]);
  check('g. plan: >= 4 corners over both axes, edge above the corner, low point 12 mm up', plan.ok && plan.points.length >= 4 && worst < 1e-4 && hErr < 1e-4,
    `${plan.points.length} of ${plan.candidates} corners, x span ${((Math.max(...xs) - Math.min(...xs)) * 1000).toFixed(0)} mm, y span ${((Math.max(...ys) - Math.min(...ys)) * 1000).toFixed(0)} mm, xy ${(worst * 1000).toFixed(3)} mm, h ${(hErr * 1000).toFixed(3)} mm`);
  // a live model with a J2 offset of 0.3 deg and a J3 scale: the targets move, the joints stay
  const kLive = zero.slice(); kLive[7] = 0.3 / 180 * Math.PI; kLive[15] = 0.002;
  const plan2 = CW.planHover({ Tbb, X, tipCam: tip, edgeToLow: 0.0023, ch, kCand: zero, kLive, q0, K: F.intrinsics, board, lo, hi });
  const p1 = plan.points[0], p2 = plan2.points.find((p) => p.id === p1.id);
  const live = p2 && CW.posOf(CW.kinPose(ch, p2.q, kLive));
  const moved = p2 ? Math.hypot(...p2.hover_m.map((v, k) => v - p1.hover_m[k])) : 0;
  check('g. live target = live FK of the candidate joints', !!p2 && Math.hypot(...live.map((v, k) => v - p2.hover_m[k])) < 1e-9 && moved > 5e-4
    && Math.abs(p2.approach_m[0] - p2.hover_m[0]) < 1e-12 && p2.approach_m[2] - p2.hover_m[2] >= 0.02, `target moved ${(moved * 1000).toFixed(2)} mm`);
  const cp = CW.planHover({ Tbb, X, tipCam: tip, edgeToLow: 0.0023, ch, kCand: zero, kLive: zero, q0, K: F.intrinsics, board, lo, hi, tableZ: 0.0098, plan: { ...CW.HOVER_PLAN, count: 2, minCount: 1 } });
  check('g. confirmation plan: end_link at table_z + 12 mm', cp.ok && cp.points.length === 2 && cp.points.every((p) => Math.abs(p.hover_m[2] - 0.0218) < 1e-6), cp.points.map((p) => (p.hover_m[2] * 1000).toFixed(2)).join(' / ') + ' mm');
  const body = CW.verifiedApplyBody({ intrinsics: { fx: 1 }, camera_mount: {}, table_z_m: 0.024, kinematics: null }, {
    tool: { tool_id: 'b601dm_gripper', tip_cam_m: tip, x_fingerprint: 'xf' }, fingerprint: 'fp', jobId: 'hv-1', board,
    evaluation: { ok: true, table_z_m: 0.0098, optical_table_z_m: 0.022, height_spread_m: 0.0013, max_xy_m: 0.002, gates: [] }, points: [{ id: 'a', ok: true, height_m: 0.0099 }] });
  check('g. apply body: hover table height, tool, acceptance', body.table_z_source === 'hover' && body.table_z_m === 0.0098 && body.acceptance.summary.table_z_m === 0.0098
    && body.acceptance.fingerprint === 'fp' && body.tool.x_fingerprint === 'xf' && body.acceptance.passed && body.kinematics === null, JSON.stringify(body).length + ' bytes');
  const unk = CW.verifiedApplyBody({ table_z_m: 0.024 }, { tool: { tool_id: 'x', tip_cam_m: tip, x_fingerprint: 'xf' }, fingerprint: 'fp', board,
    evaluation: { ok: true, table_z_m: null, gates: [] }, points: [] });
  check('g. apply body: unknown tool keeps the camera table height', unk.table_z_source === 'camera' && unk.table_z_m === 0.024, '');
}

console.log(fails ? `${fails} check(s) FAILED` : 'all checks passed');
process.exit(fails ? 1 : 0);
