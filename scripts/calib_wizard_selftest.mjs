#!/usr/bin/env node
// Self-test of the calibration wizard math (src/api/calib_wizard.js) with synthetic cameras and boards.
// Runs without a browser:  node scripts/calib_wizard_selftest.mjs
//   a. homography / corner-order detection on exact data
//   b. intrinsics: fx 824, fy 821, cx 326, cy 176, k1 -0.1, 0.3 px noise, 20 generated views:
//      fx / fy within 1 %, cx / cy within 3 px, k1 within 0.03, all W3 / X1 gates pass (held-out RMS incl.)
//   c. planar PnP with the true intrinsics: < 2 mm / 0.3 deg
//   d. hand-eye on synthetic FK (CALM.fk) + mount through the full solve: < 2 mm / 0.3 deg
//   e. table plane: table_z within 1 mm; a tilted board is refused
//   f. gates reject an all fronto-parallel set and a 1.5 px noise set
//   h. joint refinement (intrinsics + X + board pose, FK fixed): a set whose separate hand-eye error
//      is > 2 mm comes back < 1 mm / 0.2 deg
//   i. capture run loop (CW.runPoses / runAbortReason / moveFailureSkips) on a fake arm: a started
//      action failing (TRACKING_STALL) aborts with no further move_to; a planning refusal
//      (IK_UNREACHABLE reject) skips the pose; an epoch change during settle aborts; a settle timeout
//      sends stop, then aborts; a refused transition from the current pose skips without moving
//   g. pose generator: >= 20 poses, camera 0.18..0.35 m from the board, tool points >= 0.10 m above the
//      plane, transitions without IK failure / branch flip, two or more tilt azimuths; bootstrap from
//      nominal values; re-solve after 8 views
//   j. re-plan from a pose 15 mm under the clearance (plane estimate moved up after the re-solve): the tour
//      completes with every move re-checked, no tool point lower than the start; a move to a target below
//      the clearance stays refused
// CALM / VIK come from src/api/index.html (blocks CALIB-MATH and VIEW-IK), as in the browser.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const block = (name) => {
  const a = html.indexOf(`/* ${name}-BEGIN */`), b = html.indexOf(`/* ${name}-END */`);
  if (a < 0 || b < a) throw new Error(`${name} block not found in index.html`);
  return html.slice(a, b);
};
const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', vm.createContext({}));

let fails = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
const D = 180 / Math.PI;
let seed = 0xca11b0a2;
const rnd = () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
const { make, mul, inv, posOf, rotOf, rodrigues } = CW;
const add = (a, b) => a.map((v, k) => v + b[k]);
const posErr = (A, B) => Math.hypot(...posOf(A).map((v, k) => v - posOf(B)[k]));
const rotErr = (A, B) => CW.rotDist(rotOf(A), rotOf(B));
const LIM = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]];
const lo = LIM.map((l) => l[0] + 0.05), hi = LIM.map((l) => l[1] - 0.05);
const ik = (T, q) => VIK.solve(T, q, { lo, hi, iters: 60 });
const fk = (q) => CALM.fk(q);
const gateStr = (gs) => gs.map((g) => `${g.key}=${g.ok ? 'ok' : 'FAIL'}(${typeof g.value === 'number' ? +g.value.toPrecision(4) : g.value}/${g.limit})`).join(' ');

// ---- truth
const Ktrue = { fx: 824, fy: 821, cx: 326, cy: 176, k1: -0.1, k2: 0 };
const board = CW.makeBoard(); // default 4 x 3, 25 mm markers, 5 mm gap (115 x 85 mm)
const thickness = 0.0005, tableZ = 0.0;
// board on the table, "up" arrow (board -y) pointing away from the arm turned 10 deg: board x = long
// side ~ -Y_base, board z into the table
const YAW = -80 / D;
const Tbb = make(mul(CW.rpyR(0, 0, YAW), CW.rpyR(Math.PI, 0, 0)), [0.30, 0.02, tableZ + thickness]);
// v8-like mount: optical axis pitched 38 deg toward the jaws, 1.5 deg roll, lens behind and above the tips
const Rmount = mul(mul(VIK.camMount(false, 0), rodrigues([-38 / D, 0, 0])), rodrigues([0, 0, 1.5 / D]));
const Xtrue = make(Rmount, [-0.085, 0.004, 0.052]);
// start pose: camera 0.30 m above the board centre looking straight down (the whole board in view),
// image x along the board's long axis (board x)
const camDown = (h) => { const z = [0, 0, -1], x = [Math.cos(YAW), Math.sin(YAW), 0], y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]; return make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [0.30, 0.02, h]); };
const T0 = mul(camDown(0.30), inv(Xtrue));
let q0 = null;
for (let k = 0; k < 400 && !q0; k++) { // IK from random seeds (deterministic PRNG)
  const s = LIM.map(([a, b]) => a * 0.6 + (b - a) * 0.6 * rnd());
  const r = VIK.solve(T0, s, { lo, hi, iters: 300 });
  if (r.ok) q0 = r.q;
}
if (!q0) { console.log('FAIL  start pose unreachable'); process.exit(1); }

// synthetic capture: board corners through the true camera at FK pose q, detector order = OpenCV (TL TR BR BL)
const CVORDER = [3, 2, 1, 0];
function synth(q, { K = Ktrue, X = Xtrue, B = Tbb, noise = 0.3, order = CVORDER } = {}) {
  const G = fk(q), Tcb = mul(inv(mul(G, X)), B), R = rotOf(Tcb), t = posOf(Tcb), markers = [];
  for (const m of board.markers) {
    const px = m.corners.map((c) => CW.project(K, R, t, c));
    if (!px.every(([u, v, z]) => z > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)) continue;
    markers.push({ id: m.id, corners: order.map((k) => [px[k][0] + noise * gauss(), px[k][1] + noise * gauss()]) });
  }
  return { markers, G, q };
}

// ---- a. homography + corner order
{
  const v = synth(q0, { noise: 0, K: { ...Ktrue, k1: 0 } });
  const { obj, img } = CW.correspondences(v, board, CVORDER);
  const H = CW.homography(obj, img);
  let e = 0; obj.forEach((X, i) => { const w = H[2][0] * X[0] + H[2][1] * X[1] + H[2][2]; e = Math.max(e, Math.hypot((H[0][0] * X[0] + H[0][1] * X[1] + H[0][2]) / w - img[i][0], (H[1][0] * X[0] + H[1][1] * X[1] + H[1][2]) / w - img[i][1])); });
  check('a. homography exact', e < 1e-6, `${v.markers.length} markers, max err ${e.toExponential(2)} px`);
  const od = CW.detectCornerOrder([synth(q0)], board);
  check('a. corner order detected (OpenCV TL,TR,BR,BL)', od.order.join() === CVORDER.join(), `order ${od.order} rms ${od.rms.toFixed(2)} px`);
  const od2 = CW.detectCornerOrder([synth(q0, { order: [0, 1, 2, 3] })], board);
  check('a. corner order detected (apriltag BL,BR,TR,TL)', od2.order.join() === '0,1,2,3', `order ${od2.order} rms ${od2.rms.toFixed(2)} px`);
}

// ---- g (first part). generator with the true values -> the 20 views of b..e
const t0 = Date.now();
const gen = CW.generatePoses({ Tbb, board, K: Ktrue, X: Xtrue, q0, fk, ik, lo, hi, n: 24 });
const genMs = Date.now() - t0;
const poses = gen.poses;
{
  const plane = { p: posOf(Tbb), n: [0, 0, 1] };
  let dmin = Infinity, dmax = -Infinity, clr = Infinity, flips = 0;
  let prevT = fk(q0), prevQ = q0;
  for (const p of poses) {
    const G = fk(p.q), cam = posOf(mul(G, Xtrue)), d = Math.hypot(...cam.map((v, k) => v - posOf(Tbb)[k]));
    dmin = Math.min(dmin, d); dmax = Math.max(dmax, d);
    for (const pt of CW.toolPoints(G, Xtrue)) clr = Math.min(clr, pt[2] - plane.p[2]);
    const tr = CW.checkTransition(prevT, prevQ, p.T, { ik, X: Xtrue, plane, clearance: 0.10, lo, hi });
    if (!tr.ok) flips++;
    prevT = p.T; prevQ = p.q;
  }
  const az = new Set(poses.filter((p) => p.tiltDeg > 0).map((p) => p.azDeg));
  const byDist = {}; for (const p of poses) byDist[p.dist] = (byDist[p.dist] || 0) + 1;
  check('g. generator pose count / limits', poses.length >= 20 && dmin >= 0.18 - 1e-3 && dmax <= 0.35 + 1e-3 && clr >= 0.10 - 1e-3 && flips === 0 && az.size >= 2,
    `n=${poses.length} cam-board ${dmin.toFixed(3)}..${dmax.toFixed(3)} m, min tool clearance ${clr.toFixed(3)} m, transition re-check failures ${flips}, tilt azimuths ${[...az].join('/')} deg, poses per distance ${JSON.stringify(byDist)}, min predicted markers ${Math.min(...poses.map((p) => p.markers))}, ${genMs} ms; stats ${JSON.stringify(gen.stats)}`);
}

// ---- b. intrinsics from the generated views (the start view + 20)
const qs = [q0, ...poses.slice(0, 20).map((p) => p.q)];
const views = qs.map((q) => synth(q));
{
  const t1 = Date.now();
  const r = CW.calibrateIntrinsics(views, board);
  const K = r.K;
  const ok = r.ok && Math.abs(K.fx / Ktrue.fx - 1) < 0.01 && Math.abs(K.fy / Ktrue.fy - 1) < 0.01 && Math.abs(K.cx - Ktrue.cx) < 3 && Math.abs(K.cy - Ktrue.cy) < 3 && Math.abs(K.k1 - Ktrue.k1) < 0.03;
  check('b. intrinsics 0.3 px noise', ok,
    `views ${r.used.length} (skipped ${r.skipped.length}, dropped ${r.dropped.length}), init ${r.method}; fx ${K.fx.toFixed(2)} fy ${K.fy.toFixed(2)} cx ${K.cx.toFixed(2)} cy ${K.cy.toFixed(2)} k1 ${K.k1.toFixed(4)} k2 ${K.k2.toFixed(4)}; rms ${r.rms.toFixed(3)} px, held-out ${r.heldOut.rms.toFixed(3)} px (${r.heldOut.n} views), sigma fx ${r.sigma.fx.toFixed(2)} cx ${r.sigma.cx.toFixed(2)} px, cond ${r.cond.toFixed(1)}, ${Date.now() - t1} ms; gates ${gateStr(r.gates)}`);
}

// ---- c. PnP
{
  let pe = 0, re = 0, n = 0;
  for (const q of qs) {
    const v = synth(q), { obj, img } = CW.correspondences(v, board, CVORDER);
    if (v.markers.length < CW.GATES.minMarkers || n >= 10) continue; // the wizard uses views with >= 6 markers
    n++;
    const p = CW.solvePnP(obj, img, Ktrue), Ttrue = mul(inv(mul(fk(q), Xtrue)), Tbb), Tp = make(p.R, p.t);
    pe = Math.max(pe, posErr(Tp, Ttrue)); re = Math.max(re, rotErr(Tp, Ttrue));
  }
  // 0.6 deg: a 115 x 85 mm board (25 mm markers) gives twice the rotation noise of the former 230 x 170 mm one
  check('c. PnP 10 views, 0.3 px', pe < 0.002 && re < 0.6, `max ${(pe * 1000).toFixed(3)} mm / ${re.toFixed(3)} deg`);
}

// ---- d / e. full solve: hand-eye and table
const solveViews = views;
{
  const t2 = Date.now();
  const s = CW.solveAll({ views: solveViews, board, CALM, thickness });
  const he = s.handEye;
  check('d. hand-eye (full solve)', s.handEye.ok && posErr(he.X, Xtrue) < 0.002 && rotErr(he.X, Xtrue) < 0.3,
    `X err ${(posErr(he.X, Xtrue) * 1000).toFixed(3)} mm / ${rotErr(he.X, Xtrue).toFixed(3)} deg; offset ${posOf(he.X).map((v) => (v * 1000).toFixed(1))} mm (true ${posOf(Xtrue).map((v) => (v * 1000).toFixed(1))}); rms ${(he.rmse * 1000).toFixed(3)} mm / ${he.rotRmse.toFixed(3)} deg; ${Date.now() - t2} ms`);
  check('e. table plane', s.table.ok && Math.abs(s.table.table_z_m - tableZ) < 0.001,
    `table_z ${(s.table.table_z_m * 1000).toFixed(3)} mm (true ${(tableZ * 1000).toFixed(1)}), tilt ${s.table.tiltDeg.toFixed(3)} deg, spread ${(s.table.spread_m * 1000).toFixed(3)} mm`);
  check('d. all gates pass, apply body', s.ok && s.apply && s.apply.intrinsics.width === 640 && s.apply.camera_mount.q_end_cam.length === 4,
    `${gateStr(s.gates)}; apply ${JSON.stringify({ ...s.apply, rms: undefined }).slice(0, 260)}`);
  // tilted board: 8 deg about board x
  const Tt = mul(Tbb, make(rodrigues([8 / D, 0, 0]), [0, 0, 0]));
  const tilted = CW.fitTable(qs.map((q) => mul(mul(fk(q), Xtrue), mul(inv(mul(fk(q), Xtrue)), Tt))), board, thickness);
  check('e. tilted board refused', !tilted.ok && tilted.gates.some((g) => g.key === 'plane_tilt' && !g.ok), `tilt ${tilted.tiltDeg.toFixed(2)} deg -> ${gateStr(tilted.gates)}`);
}

// ---- h. joint refinement: a set where the separate solve's hand-eye is > 2 mm off
{
  // 0.3 px corner noise (new draws) on the start view and the first 15 generated ones
  const hv = qs.slice(0, 16).map((q) => synth(q, { noise: 0.3 }));
  const th = Date.now();
  const sep = CW.solveAll({ views: hv, board, CALM, thickness, joint: false });
  const jnt = CW.solveAll({ views: hv, board, CALM, thickness });
  const jMs = Date.now() - th;
  const eS = posErr(sep.handEye.X, Xtrue), rS = rotErr(sep.handEye.X, Xtrue), eJ = posErr(jnt.final.X, Xtrue), rJ = rotErr(jnt.final.X, Xtrue);
  const Kf = jnt.final.K;
  check('h. joint refinement (separate hand-eye > 2 mm -> refined < 1 mm / 0.2 deg)', eS > 0.002 && eJ < 0.001 && rJ < 0.2 && jnt.ok,
    `separate X err ${(eS * 1000).toFixed(3)} mm / ${rS.toFixed(3)} deg, fx ${sep.intr.K.fx.toFixed(2)}; refined X err ${(eJ * 1000).toFixed(3)} mm / ${rJ.toFixed(3)} deg, fx ${Kf.fx.toFixed(2)} fy ${Kf.fy.toFixed(2)} cx ${Kf.cx.toFixed(2)} cy ${Kf.cy.toFixed(2)} k1 ${Kf.k1.toFixed(4)} (true 824/821/326/176/-0.1); joint RMS ${jnt.joint.rmsBefore.toFixed(3)} -> ${jnt.joint.rmsAfter.toFixed(3)} px in ${jnt.joint.iterations} it (both solves ${jMs} ms); board err ${(posErr(jnt.final.Tbb, Tbb) * 1000).toFixed(3)} mm, table_z ${(jnt.apply.table_z_m * 1000).toFixed(3)} mm; gates ${jnt.ok ? 'pass' : gateStr(jnt.gates)}`);
}

// ---- f. degenerate and noisy sets
{
  // fronto-parallel: camera straight down at 0.30..0.34 m over the board, shifted, rolled about the optical axis
  const fv = [];
  for (let i = 0; i < 20; i++) {
    const h = 0.32 + 0.0015 * i, roll = (i % 5 - 2) * 0.05;
    const Tc2 = make(mul(rotOf(camDown(0)), rodrigues([0, 0, roll])), add(posOf(camDown(0)), [0.01 * ((i % 4) - 1.5), 0.005 * ((i % 3) - 1), 0]).map((v, k) => (k === 2 ? h : v)));
    const G = mul(Tc2, inv(Xtrue)), Tcb = mul(inv(Tc2), Tbb), R = rotOf(Tcb), t = posOf(Tcb), markers = [];
    for (const m of board.markers) {
      const px = m.corners.map((c) => CW.project(Ktrue, R, t, c));
      if (px.every(([u, v]) => u >= 4 && u <= 636 && v >= 4 && v <= 356)) markers.push({ id: m.id, corners: CVORDER.map((k) => [px[k][0] + 0.3 * gauss(), px[k][1] + 0.3 * gauss()]) });
    }
    fv.push({ markers, G });
  }
  const r = CW.calibrateIntrinsics(fv, board);
  check('f. fronto-parallel set rejected', !r.ok, `views ${r.used ? r.used.length : 0}, init ${r.method}, fx ${r.K ? r.K.fx.toFixed(1) : '-'}; ${gateStr(r.gates)}`);
  const nv = qs.map((q) => synth(q, { noise: 1.5 }));
  const rn = CW.calibrateIntrinsics(nv, board);
  check('f. 1.5 px noise set rejected', !rn.ok && rn.gates.some((g) => g.key === 'rms' && !g.ok), `rms ${rn.rms.toFixed(3)} px; ${gateStr(rn.gates)}`);
}

// ---- g (second part). bootstrap from nominal values, re-solve after 8 views
{
  const Xnom = make(CW.mountRotation(VIK, { flip: false, rot_deg: 0, tilt_deg: -38 }), [-0.08, 0, 0.05]);
  const v0 = synth(q0);
  const bs = CW.bootstrapBoard(v0, board, { K: CW.NOMINAL, X: Xnom, G: fk(q0) });
  const g1 = CW.generatePoses({ Tbb: bs.Tbb, board, K: CW.NOMINAL, X: Xnom, q0, fk, ik, lo, hi, n: 20 });
  // checked against the truth: camera distance and clearance (limits hold on the estimate; report the truth)
  let dmin = Infinity, dmax = 0, clr = Infinity, vis = Infinity;
  for (const p of g1.poses) {
    const G = fk(p.q), cam = posOf(mul(G, Xtrue));
    const d = Math.hypot(...cam.map((v, k) => v - posOf(Tbb)[k])); dmin = Math.min(dmin, d); dmax = Math.max(dmax, d);
    for (const pt of CW.toolPoints(G, Xtrue)) clr = Math.min(clr, pt[2] - posOf(Tbb)[2]);
    vis = Math.min(vis, synth(p.q, { noise: 0 }).markers.length);
  }
  check('g. bootstrap (nominal fx 824, mount tilt -38, offset guess)', bs && g1.poses.length >= 20 && clr >= 0.07,
    `board est err ${(posErr(bs.Tbb, Tbb) * 1000).toFixed(1)} mm / ${rotErr(bs.Tbb, Tbb).toFixed(2)} deg; n=${g1.poses.length}; true cam-board ${dmin.toFixed(3)}..${dmax.toFixed(3)} m, true min clearance ${clr.toFixed(3)} m, min true visible markers ${vis}`);
  // re-solve after 8 accepted views, then regenerate the rest
  const v8 = [v0, ...g1.poses.slice(0, 7).map((p) => synth(p.q))];
  const r8 = CW.solveAll({ views: v8, board, CALM, thickness });
  const X8 = r8.handEye && r8.handEye.X;
  let g2 = null;
  if (X8) {
    const Tbb8 = mul(mul(v8[0].G, X8), make(...(() => { const { obj, img } = CW.correspondences(v0, board, r8.intr.order); const p = CW.solvePnP(obj, img, r8.intr.K); return [p.R, p.t]; })()));
    g2 = CW.generatePoses({ Tbb: Tbb8, board, K: r8.intr.K, X: X8, q0: g1.poses[6].q, fk, ik, lo, hi, n: 13, exclude: g1.poses.slice(0, 7).map((p) => p.T) });
  }
  check('g. re-solve after 8 views + regenerate', !!X8 && g2 && g2.poses.length >= 10,
    X8 ? `8-view fx ${r8.intr.K.fx.toFixed(1)} cx ${r8.intr.K.cx.toFixed(1)} k1 ${r8.intr.K.k1.toFixed(3)}; X err ${(posErr(X8, Xtrue) * 1000).toFixed(2)} mm / ${rotErr(X8, Xtrue).toFixed(2)} deg; regenerated ${g2 ? g2.poses.length : 0}` : `hand-eye failed: ${r8.handEye && r8.handEye.err}`);
}

// ---- i. capture run loop on a fake arm (observation: epoch, safety state, newest action id)
{
  const fake = ({ n = 5, failMove = {}, onSettle = null, checkFail = new Set() } = {}) => {
    const obs = { epoch: 7, state: 'Armed', fault: null, action: 40, actionState: 'done' }, log = [];
    const run = { lease0: 3, epoch0: obs.epoch, own: null };
    const st = { plan: Array.from({ length: n }, (_, i) => ({ T: i, state: 'pending' })), idx: 0 };
    let moves = 0;
    const io = {
      abortReason: () => CW.runAbortReason({ stopReq: false, hidden: false, live: true, lease0: run.lease0, lease: 3, epoch0: run.epoch0,
        epoch: obs.epoch, state: obs.state, fault: obs.fault, hold: false, action: obs.action, actionState: obs.actionState, ownAction: run.own }),
      check: (T) => (checkFail.has(T) ? { ok: false, reason: 'clearance' } : { ok: true }),
      move: async (T) => {
        moves++; log.push(`move ${T}`);
        const f = failMove[moves];
        if (f && f.phase === 'reject') throw Object.assign(new Error(f.code), f);
        obs.action++; run.own = obs.action;
        if (f) throw Object.assign(new Error(f.code), f);
      },
      settle: async () => { log.push('settle'); if (onSettle) onSettle(obs, moves); },
      capture: async () => ({ markers: [] }),
      accept: (v, i, p) => { p.state = 'ok'; },
      after: () => {},
      stop: async () => { log.push('stop'); },
      skipWhy: (k, d) => `${k}:${d}`,
    };
    return { st, io, log, obs, run, moves: () => moves };
  };
  const go = async (f) => { try { return { res: await CW.runPoses(f.st, f.io, { runAll: true }) }; } catch (e) { return { err: e }; } };
  // TRACKING_STALL on move 3 (action failed after it started)
  let f = fake({ failMove: { 3: { code: 'TRACKING_STALL', phase: 'action' } } }), r = await go(f);
  check('i. TRACKING_STALL on move 3 -> abort, no further move_to', r.err && r.err.why === 'move' && r.err.detail === 'TRACKING_STALL' && f.moves() === 3 && f.log.at(-1) === 'stop',
    `${r.err ? r.err.why + '/' + r.err.detail : 'no abort'}, moves ${f.moves()}, log ${f.log.join(',')}`);
  // other started-action failures and non-planning rejects abort too
  const ab = [];
  for (const [code, phase] of [['TORQUE_LIMIT', 'action'], ['SUPERSEDED', 'action'], ['IK_UNREACHABLE', 'action'], ['PLAN_STALE', 'reject'], ['MOTION_BUSY', 'reject'], [undefined, 'reject']]) {
    const g = fake({ failMove: { 2: { code, phase } } }), q = await go(g);
    ab.push(`${code}/${phase}:${q.err ? q.err.why : 'ran'}:${g.moves()}`);
    if (!(q.err && q.err.why === 'move' && g.moves() === 2)) fails++;
  }
  check('i. other move failures abort', ab.every((x) => x.endsWith(':move:2')), ab.join(' '));
  // IK_UNREACHABLE refusal (command_reject before motion) on move 2 -> skip, the run continues
  f = fake({ failMove: { 2: { code: 'IK_UNREACHABLE', phase: 'reject' } } }); r = await go(f);
  check('i. IK_UNREACHABLE refusal -> pose skipped, run continues', r.res === 'done' && f.moves() === 5 && f.st.plan[1].state === 'skip' && f.st.plan.filter((p) => p.state === 'ok').length === 4 && !f.log.includes('stop'),
    `${r.res || r.err.why}, moves ${f.moves()}, states ${f.st.plan.map((p) => p.state).join(',')}`);
  // epoch change (stop / e-stop from elsewhere) during settle after move 2
  f = fake({ onSettle: (o, m) => { if (m === 2) o.epoch++; } }); r = await go(f);
  check('i. epoch change during settle -> abort, no further move_to', r.err && r.err.why === 'epoch' && f.moves() === 2 && f.log.at(-1) === 'stop',
    `${r.err ? r.err.why : 'no abort'}, moves ${f.moves()}, log ${f.log.join(',')}`);
  // safety state leaves Armed / Moving, a fault, a foreign action
  const sv = [];
  for (const [k, fn] of [['state', (o) => { o.state = 'Idle'; }], ['fault', (o) => { o.fault = { code: 'THERMAL_TRIP' }; }], ['foreign', (o) => { o.action += 5; o.actionState = 'running'; }]]) {
    const g = fake({ onSettle: (o, m) => { if (m === 1) fn(o); } }), q = await go(g);
    sv.push(`${k}:${q.err ? q.err.why : 'ran'}:${g.moves()}`);
    if (!(q.err && q.err.why === k && g.moves() === 1)) fails++;
  }
  check('i. state / fault / foreign action during settle -> abort', sv.every((x) => x.endsWith(':1')), sv.join(' '));
  // real-arm regression (2026-11-17): the move_to ack arrives before the observation shows the new action, so the
  // observation still holds the previous FINISHED action (done / cancelled, other id) while the wizard waits -> not foreign
  {
    const g = fake();
    g.io.move = async (T) => {
      g.log.push(`move ${T}`);
      const prev = g.obs.action, own = prev + 1;
      g.run.own = own; // ack received; observation still shows prev (finished)
      const why = g.io.abortReason(); // what cwGuardedWait checks right after the ack
      if (why) { g.log.push('abort ' + why); throw CW.runAbort(why); }
      g.obs.action = own; g.obs.actionState = 'done';
    };
    const q = await go(g);
    check('i. ack before observation: previous finished action is not foreign', q.res === 'done' && !g.log.some((x) => x.startsWith('abort')),
      `${q.res || q.err.why}, log ${g.log.slice(0, 6).join(',')}`);
  }
  // settle timeout -> stop sent, then abort
  f = fake(); f.io.settle = async () => { f.log.push('settle'); throw Object.assign(new Error('timeout'), { code: 'TIMEOUT' }); }; r = await go(f);
  check('i. settle timeout -> stop sent, then abort', r.err && r.err.why === 'timeout' && f.moves() === 1 && f.log.join(',') === 'move 0,settle,stop',
    `${r.err ? r.err.why : 'no abort'}, log ${f.log.join(',')}`);
  // after a skipped pose the next transition is re-checked from the current pose: refused -> skipped without a move
  f = fake({ failMove: { 2: { code: 'IK_UNREACHABLE', phase: 'reject' } }, checkFail: new Set([2]) }); r = await go(f);
  check('i. refused transition after a skip -> skipped, not sent', r.res === 'done' && !f.log.includes('move 2') && f.st.plan[2].why === 'transition:clearance' && f.moves() === 4,
    `${r.res || r.err.why}, log ${f.log.join(',')}`);
  f = fake({ n: 2, checkFail: new Set([1]) }); r = await go(f);
  check('i. refused transition with no pose left -> abort', r.err && r.err.why === 'nopose' && f.moves() === 1 && f.log.at(-1) === 'stop', `${r.err ? r.err.why : r.res}, log ${f.log.join(',')}`);
  // R7 M8: checkTransition passes extraEndPoints to toolPoints so board-search/re-aim catches tool extensions below clearance
  {
    const id4 = CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.25, 0, 0.25]);
    const T_b = CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.25, 0.05, 0.25]);
    const X_up = CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0.05]);
    const plane = { p: [0, 0, 0], n: [0, 0, 1] };
    const ikOk = (_T, q) => ({ ok: true, q });
    const stockTr = CW.checkTransition(id4, [0, 0, 0, 0, 0, 0], T_b, { ik: ikOk, X: X_up, plane, clearance: 0.12 });
    const extTr = CW.checkTransition(id4, [0, 0, 0, 0, 0, 0], T_b, { ik: ikOk, X: X_up, plane, clearance: 0.12, extraEndPoints: [[0, 0, -0.15]] });
    check('R7 M8. checkTransition catches extraEndPoints below clearance', stockTr.ok && !extTr.ok && extTr.reason === 'clearance', `stock=${stockTr.ok} ext=${extTr.ok}/${extTr.reason}`);
  }
  // j. re-plan from a pose that reads below the clearance: the re-solve moved the plane estimate up (sim e2e:
  // bootstrap board z -16 mm, refined ~0), so the view the arm is at is 15 mm under the 0.10 m clearance.
  // Every move out of it used to fail 'clearance' at the first sample: the tour stopped with 0 poses and the
  // run aborted with 'nopose: clearance'. The tour must now complete, every consecutive move must pass
  // checkTransition, and none may take a tool point lower than the start.
  {
    const clearance = 0.10, h = (q, B) => Math.min(...CW.toolPoints(fk(q), Xtrue).map((p) => p[2] - posOf(B)[2]));
    const start = poses.find((p) => h(p.q, Tbb) < 0.12);
    const Tup = make(rotOf(Tbb), add(posOf(Tbb), [0, 0, h(start.q, Tbb) - (clearance - 0.015)]));
    const h0 = h(start.q, Tup);
    const g = CW.generatePoses({ Tbb: Tup, board, K: Ktrue, X: Xtrue, q0: start.q, fk, ik, lo, hi, n: 10, exclude: [start.T] });
    const plane = CW.boardPlane(Tup);
    const above = (T) => Math.min(...CW.toolPoints(T, Xtrue).map((pt) => pt[2] - plane.p[2])); // plane normal +z
    // the old rule (every sample >= clearance) refuses the first move at its first 1 cm sample
    const A0 = fk(start.q), B0 = g.poses.length ? g.poses[0].T : A0, d0 = Math.hypot(...posOf(B0).map((v, k) => v - posOf(A0)[k]));
    const strictRefuses = above(CW.interpPose(A0, B0, Math.min(1, 0.01 / Math.max(d0, 1e-9)))) < clearance;
    let prevT = fk(start.q), prevQ = start.q, bad = 0, lowest = Infinity;
    for (const p of g.poses) {
      const tr = CW.checkTransition(prevT, prevQ, p.T, { ik, X: Xtrue, plane, clearance, lo, hi });
      if (!tr.ok) bad++;
      for (let k = 1; k <= 50; k++) lowest = Math.min(lowest, above(CW.interpPose(prevT, p.T, k / 50)));
      prevT = p.T; prevQ = tr.ok ? tr.q : p.q;
    }
    check('j. tour from a start below the clearance completes', h0 < clearance - 0.01 && strictRefuses && g.poses.length === 10 && bad === 0 && lowest >= h0 - 1e-6 && g.stats.transition === 0,
      `start height ${h0.toFixed(3)} m (clearance ${clearance}), old rule refuses the first move ${strictRefuses}, tour ${g.poses.length}/10, re-check failures ${bad}, lowest tool point on the way ${lowest.toFixed(3)} m, stats ${JSON.stringify(g.stats)}`);
    const flat = CW.checkTransition(fk(start.q), start.q, CW.make(rotOf(fk(start.q)), add(posOf(fk(start.q)), [0.03, 0, 0])), { ik, X: Xtrue, plane, clearance, lo, hi });
    check('j. a start below the clearance may not move to a target below it', !flat.ok && flat.reason === 'clearance', `${flat.ok ? 'ok' : flat.reason}`);
  }
}

// ---- board SVG
{
  const bd = CW.makeBoard(), svg = CW.boardSvg(bd);
  check('board SVG', svg.startsWith('<svg') && (svg.match(/fill="#000"/g) || []).length >= 12 && svg.includes('100 mm') && Math.abs(bd.width - 0.115) < 1e-9 && Math.abs(bd.height - 0.085) < 1e-9 && bd.byId.get(3).row === 0 && bd.byId.get(3).col === 3, `${svg.length} bytes, ${bd.cols} x ${bd.rows}, ${(bd.width * 1000).toFixed(0)} x ${(bd.height * 1000).toFixed(0)} mm board, id 3 at row 0 col 3: ${bd.byId.get(3).row},${bd.byId.get(3).col}`);
}
// ---- the cockpit ArUco overlay's expected ids (ARU_BOARD_IDS in index.html) are the default board's ids 0..n-1
{
  const bd = CW.makeBoard(), m = html.match(/const ARU_BOARD_IDS = (\d+);/), n = m ? Number(m[1]) : NaN;
  const ids = bd.markers.map((x) => x.id).sort((a, b) => a - b);
  check('overlay board ids', n === bd.cols * bd.rows && ids.every((id, i) => id === i) && ids.length === n, `ARU_BOARD_IDS ${n}, board ${bd.cols} x ${bd.rows}, ids ${ids[0]}..${ids.at(-1)}`);
}

// ---- B8 / M4: physical_contact_end_m + thickness in hover/confirm, full check_points_end_m clearance, TCP boundary
{
  // 1. toolPoints includes extraEndPoints (extended corner fails clearance even when stock 3 points pass)
  const Tend = make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.25, 0.0, 0.12]);
  const Xid = make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.05]);
  const stockPts = CW.toolPoints(Tend, Xid);
  const fullPts = CW.toolPoints(Tend, Xid, [[0.032, 0.082, -0.05]]);
  const stockMinZ = Math.min(...stockPts.map((p) => p[2]));
  const fullMinZ = Math.min(...fullPts.map((p) => p[2]));
  check('B8. full envelope toolPoints catches extra check_points_end_m dip',
    stockMinZ >= 0.10 && fullMinZ < 0.08,
    `stockMinZ ${stockMinZ.toFixed(3)} m, fullMinZ ${fullMinZ.toFixed(3)} m`);

  // 2. Nonzero physical_contact_end_m [0.0288, 0, 0] + thickness 0.002 m in hover evaluation & confirmation plan
  const contactEnd = [0.0288, 0, 0];
  const thick = 0.002;
  const Rdown = CW.rpyR(0, Math.PI / 2, 0);
  // Tool pointing down (+X_end along -Z_base): contact at z = 0.014 (0.012 above paper at z = 0.002, table_z = 0.0)
  // => end_link z must be 0.014 + 0.0288 = 0.0428 m
  const Tvert = make(Rdown, [0.25, 0.0, 0.0428]);
  const Ktest = { fx: 500, fy: 500, cx: 320, cy: 180, k1: 0, k2: 0 };
  const bd = CW.makeBoard();
  // Board at z = table_z + thickness = 0.002, flipped into table (+Z_board = -Z_base)
  const TbbHover = make(CW.rpyR(Math.PI, 0, 0), [0.30, 0.0, 0.002]);
  // Camera mounted 0.15 m above tips looking down along +X_end (optical axis +Z_cam = +X_end)
  const Xhover = make(VIK.camMount(false, 0), [-0.15, 0.0, 0.0]);
  const tipEnd = [0.0288 - 0.0023, 0.0, 0.0]; // sticker edge 2.3 mm above contact in end_link frame
  const tipCam = CW.apply(inv(Xhover), tipEnd);
  const Tcb = mul(inv(mul(Tvert, Xhover)), TbbHover);
  const markers = bd.markers.map((m) => ({
    id: m.id,
    corners: [0, 1, 2, 3].map((k) => {
      const pr = CW.project(Ktest, rotOf(Tcb), posOf(Tcb), m.corners[k]);
      return [pr[0], pr[1]];
    }),
  }));
  const hp = CW.hoverPoint({
    observations: [{ markers }],
    board: bd,
    K: Ktest,
    tipCam,
    edgeToLow: 0.0023,
    Tend: Tvert,
    X: Xhover,
    physicalContactEnd: contactEnd,
  });
  const ev = CW.evaluateHovers([hp, hp, hp, hp], { edgeToLow: 0.0023, thickness: thick });
  check('B8. evaluateHovers uses physicalContactEnd and subtracts board thickness once',
    hp.ok && Math.abs(hp.z_end_m - 0.0428) < 1e-6 && Math.abs(hp.z_contact_m - 0.014) < 1e-6
      && Math.abs(hp.height_m - 0.012) < 1e-5 && Math.abs(ev.table_z_m - 0.0) < 1e-5,
    `z_end ${hp.z_end_m?.toFixed(4)}, z_contact ${hp.z_contact_m?.toFixed(4)}, height ${hp.height_m?.toFixed(4)}, table_z ${ev.table_z_m?.toFixed(5)}`);

  // Confirmation hover plan places physical contact at tableZ + thickness + clearanceM (0.014) => end_link at 0.0428
  const ch = CW.kinChain(CALM.CHAIN), zero = new Array(42).fill(0);
  const cp = CW.planHover({
    Tbb: TbbHover,
    X: Xhover,
    tipCam,
    edgeToLow: 0.0023,
    ch,
    kCand: zero,
    kLive: zero,
    q0,
    K: Ktest,
    board: bd,
    lo,
    hi,
    tableZ: 0.0,
    physicalContactEnd: contactEnd,
    thickness: thick,
    plan: { ...CW.HOVER_PLAN, count: 2, minCount: 1 },
  });
  check('B8. confirmation plan places rotated physical_contact at tableZ + thickness + clearance',
    cp.ok && cp.points.length >= 1 && cp.points.every((pt) => Math.abs(pt.hover_m[2] - 0.0428) < 1e-4),
    `confirm hover z_end ${(cp.points[0]?.hover_m[2] * 1000)?.toFixed(2)} mm (expected 42.80 mm)`);

  // 3. Static contract checks on index.html: cwMove converts EndLink -> TCP once, renderCart uses obs.tcp and tcpOffset
  check('B8/M4. index.html converts cwMove via CW.apply and uses obs.tcp / tcpOffset in renderCart',
    html.includes('position: CW.apply(T, off)') && html.includes('cwFineSegment') && html.includes('S.wsCheckPoints')
      && html.includes('(isSoArm || !C.sending) && hasObsTcp'),
    'cwMove + cwFineSegment + renderCart verified');
}
console.log(fails ? `${fails} checks failed` : 'all checks passed');
process.exit(fails ? 1 : 0);
