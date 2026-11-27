#!/usr/bin/env node
// Self-test of the kinematic fine calibration solver (src/api/calib_wizard.js solveKinematics / solveFine).
// Runs without a browser:  node scripts/kincal_selftest.mjs
//   a. synthetic: two board placements, 35 planned poses each (fine-mode generator options), 2 frames per pose,
//      0.3 px noise; views rendered through an arm whose geometry differs from the URDF (offsets J2..J5, a 1.5 deg
//      joint-origin tilt, a 10 mm origin shift). solveFine: intrinsics gates pass, held-out < 0.5 mm both ways,
//      the camera pose relative to each board predicted within 1 mm / 0.1 deg over all views and over joint sets
//      not captured, the non-redundant parameters (J5 offset, J3 tilt) within 0.1 deg, gates pass.
//      Zero injected correction: parameters ~0 (offsets < 0.3 deg, tilts < 0.3 deg, origins < 3 mm, scales < 0.5 %).
//   b. real data replay (B601-DM, 2026-11-18): real2/collect_b1.json (placement A) and collect_b2.json (B), K from
//      Kreal.json: uncorrected and corrected held-out mm both ways, gate decision, solve time. Skipped when the
//      files are missing (REAL2 = directory).
//   c. gates: a J5 zero offset of 12 deg (bound 10) is refused; unmodelled per-view joint errors that leave the
//      held-out above 3 mm are refused.
//   e. model level choice (L0 none, L1 zero offsets, L2 + J2 scale / tilt, L3 full): the injected correction of a.
//      chooses L3, zero correction L0, the real replay of b. a level above L0; a synthetic copy of the 2026-11-19 real
//      case (no systematic error, random per-pose joint errors KIN_NOISE_DEG (default 0.37 deg) on J2..J5, 56 views
//      per placement, uncorrected held-out ~3 mm) chooses L0 and its apply body carries kinematics null; the
//      chooseKinLevel rules on fixed held-out tables. Every solve prints its level table. KIN_E_SEEDS=n adds n noise
//      draws of the e. case and prints the chosen-level distribution.
//   a. also: verifyView (verify view of a loaded session) passes for the same board, refuses a board moved 20 mm.
// CALM comes from src/api/index.html (block CALIB-MATH), as in the browser.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { readCockpitHtml } from './source_paths.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const html = readCockpitHtml();
const block = (name) => html.slice(html.indexOf(`/* ${name}-BEGIN */`), html.indexOf(`/* ${name}-END */`));
const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', vm.createContext({}));

const D = 180 / Math.PI;
// KIN_PRIOR_W=<weight> overrides the prior weight of every solve (weight sweep)
const PRIOR = process.env.KIN_PRIOR_W != null ? { ...CW.KIN_PRIOR, weight: Number(process.env.KIN_PRIOR_W) } : CW.KIN_PRIOR;
const solveKinematics = (o) => CW.solveKinematics({ prior: PRIOR, ...o }), solveFine = (o) => CW.solveFine({ prior: PRIOR, ...o });
console.log(`prior weight ${PRIOR.weight}, sigma offset ${(PRIOR.sigma[0] * D).toFixed(2)} deg, scale ${PRIOR.sigma[1]}, tilt ${(PRIOR.sigma[2] * D).toFixed(2)} deg, origin ${PRIOR.sigma[4] * 1000} mm`);
let fails = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
let seed = 0x6b1ca1;
const rnd = () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
const { make, mul, inv, posOf, rotOf, rodrigues } = CW;
const LIM = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]];
const lo = LIM.map((l) => l[0] + 0.05), hi = LIM.map((l) => l[1] - 0.05);
const fk = (q) => CALM.fk(q, null); // planning model: plain URDF (no correction stored yet)
const ik = (T, q) => VIK.solve(T, q, { lo, hi, iters: 60 });
const ch = CW.kinChain(CALM.CHAIN);
const T4 = (E) => make([[E.R[0], E.R[1], E.R[2]], [E.R[3], E.R[4], E.R[5]], [E.R[6], E.R[7], E.R[8]]], E.p);
const fmtK = (k) => [1, 2, 3, 4].map((j) => `J${j + 1} off ${(k[7 * j] * D).toFixed(2)}° sc ${(k[7 * j + 1] * 100).toFixed(2)}% tilt ${(k[7 * j + 2] * D).toFixed(2)}/${(k[7 * j + 3] * D).toFixed(2)}° o ${[4, 5, 6].map((p) => (k[7 * j + p] * 1000).toFixed(1)).join('/')} mm`).join(' | ');
const gateStr = (gs) => gs.map((g) => `${g.key}=${g.ok ? 'ok' : 'FAIL'}(${typeof g.value === 'number' ? +g.value.toPrecision(4) : g.value}/${g.limit})`).join(' ');

// ---- synthetic rig
const Ktrue = { fx: 812, fy: 809, cx: 323, cy: 184, k1: 0.1, k2: -0.2 };
const board = CW.makeBoard();
const thickness = 0.0005;
const Rmount = mul(mul(VIK.camMount(false, 0), rodrigues([-38 / D, 0, 0])), rodrigues([0, 0, 1.5 / D]));
const Xtrue = make(Rmount, [-0.085, 0.004, 0.052]);
// wizard start value: nominal mount (tilt only, lens offset guessed)
const Xnom = make(mul(VIK.camMount(false, 0), rodrigues([-38 / D, 0, 0])), [-0.08, 0, 0.05]);
const boardAt = (x, y, yawDeg) => make(mul(CW.rpyR(0, 0, yawDeg / D), CW.rpyR(Math.PI, 0, 0)), [x, y, thickness]);
const PLACE = [boardAt(0.30, 0.02, -80), boardAt(0.27, -0.12, -105)];
const FINE = { n: 35, dists: [0.20, 0.24, 0.28, 0.32, 0.36, 0.40], tilts: [0, 12, 24, 36], azimuths: 8, rolls: [0, 20, -20], minMarkers: 8, clearance: 0.10, distRange: [0.2, 0.4] };
function startPose(Tbb) {
  const c = posOf(Tbb), Rb = rotOf(Tbb), x = [Rb[0][0], Rb[1][0], 0], z = [0, 0, -1], y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
  const T0 = mul(make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [c[0], c[1], 0.30]), inv(Xnom));
  for (let k = 0; k < 400; k++) { const s = LIM.map(([a, b]) => a * 0.6 + (b - a) * 0.6 * rnd()); const r = VIK.solve(T0, s, { lo, hi, iters: 300 }); if (r.ok) return r.q; }
  return null;
}
const plans = PLACE.map((Tbb) => { const q0 = startPose(Tbb); return CW.generatePoses({ ...FINE, Tbb, board, K: CW.NOMINAL, X: Xnom, q0, fk, ik, lo, hi }).poses.map((p) => p.q); });
check('setup. fine-mode plans for both placements', plans.every((p) => p.length >= 30), `poses ${plans.map((p) => p.length).join(' / ')}`);
// views of the real (corrected) arm: kTrue geometry, jointErr (rad std, per view, or per pose with perPose: both
// frames of a pose share it) unmodelled; poses: the first n poses of each plan
function render(kTrue, { noise = 0.3, jointErr = 0, perPose = false, poses = Infinity } = {}) {
  const views = [];
  plans.forEach((qs, ds) => {
    for (const q of qs.slice(0, poses)) {
      const e = q.map((_, i) => (jointErr && i >= 1 && i <= 4 ? jointErr * gauss() : 0));
      for (let f = 0; f < 2; f++) {
      const qa = q.map((v, i) => v + (perPose ? e[i] : jointErr && i >= 1 && i <= 4 ? jointErr * gauss() : 0));
      const G = T4(CW.kinFk(ch, qa, kTrue)), Tcb = mul(inv(mul(G, Xtrue)), PLACE[ds]), R = rotOf(Tcb), t = posOf(Tcb), markers = [];
      for (const m of board.markers) {
        const px = m.corners.map((c) => CW.project(Ktrue, R, t, c));
        if (px.every(([u, v, z]) => z > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)) markers.push({ id: m.id, corners: px.map(([u, v]) => [u + noise * gauss(), v + noise * gauss()]) });
      }
      views.push({ markers, q, ds });
      }
    }
  });
  return views;
}
// level table of a solveKinematics result
const levelTable = (r) => r.levels.map((l) => `${l.chosen ? '*' : ' '}${l.name} (${l.params} par) A->B ${l.ab.toFixed(3)} B->A ${l.ba.toFixed(3)} mean ${l.mean.toFixed(3)} mm${l.eligible ? '' : ' (worse than L0)'}`).join(' | ');
const kInj = new Array(42).fill(0);
kInj[7 * 1] = 1.2 / D; kInj[7 * 2] = -0.8 / D; kInj[7 * 3] = 1.5 / D; kInj[7 * 4] = -4.0 / D; // zero offsets J2..J5
kInj[7 * 2 + 2] = 1.5 / D;                                                                     // J3 origin tilt about x
kInj[7 * 3 + 4] = 0.010;                                                                       // J4 origin 10 mm along x
// camera pose relative to the board (gauge free) predicted by a solution vs the truth, over the views
function poseAgreement(views, r, kTrue) {
  let mmMax = 0, degMax = 0;
  for (const v of views) {
    const Tt = mul(inv(PLACE[v.ds]), mul(T4(CW.kinFk(ch, v.q, kTrue)), Xtrue));
    const Te = mul(inv(r.boards[v.ds]), mul(T4(CW.kinFk(ch, v.q, r.k)), r.X));
    mmMax = Math.max(mmMax, Math.hypot(...posOf(Tt).map((x, i) => x - posOf(Te)[i])) * 1000);
    degMax = Math.max(degMax, CW.rotDist(rotOf(Tt), rotOf(Te)));
  }
  return { mmMax, degMax };
}
// ---- a. synthetic recovery
{
  const views = render(kInj);
  const t0 = Date.now();
  const r = solveFine({ views, markerSide: board.markerSide, chain: CALM.CHAIN, X0: Xnom, thickness, solvedAt: '2026-11-18T00:00:00Z' });
  const ms = Date.now() - t0, kin = r.kin;
  const ag = poseAgreement(views, kin, kInj);
  const offErr = [1, 2, 3, 4].map((j) => (kin.k[7 * j] - kInj[7 * j]) * D);
  console.log(`      views ${kin.views.used} (A ${kin.views.A}, B ${kin.views.B}), solve ${ms} ms; K fx ${r.intr.K.fx.toFixed(1)} fy ${r.intr.K.fy.toFixed(1)} cx ${r.intr.K.cx.toFixed(1)} cy ${r.intr.K.cy.toFixed(1)} k1 ${r.intr.K.k1.toFixed(3)}`);
  console.log(`      injected  ${fmtK(kInj)}`);
  console.log(`      recovered ${fmtK(kin.k)}`);
  console.log(`      gates ${gateStr(r.gates)}`);
  check('a. held-out < 0.5 mm both ways (injected correction)', kin.heldOut.ab.mm < 0.5 && kin.heldOut.ba.mm < 0.5,
    `A->B ${kin.heldOut.ab0.mm.toFixed(2)} -> ${kin.heldOut.ab.mm.toFixed(3)} mm, B->A ${kin.heldOut.ba0.mm.toFixed(2)} -> ${kin.heldOut.ba.mm.toFixed(3)} mm (uncorrected -> corrected), train ${kin.train.mm.toFixed(3)} mm / ${kin.train.px.toFixed(3)} px`);
  console.log(`      levels ${levelTable(kin)}`);
  check('a. injected correction -> level L3 chosen', kin.level === 'L3' && r.apply.kinematics && r.apply.kinematics.fit.level === 'L3' && r.apply.rms.kin_level === 'L3', `level ${kin.level}`);
  check('a. camera-to-board pose predicted within 1 mm / 0.1 deg', ag.mmMax < 1 && ag.degMax < 0.1, `max ${ag.mmMax.toFixed(3)} mm, ${ag.degMax.toFixed(4)} deg over ${views.length} views`);
  // the L3 model is not minimal: a zero offset of J3 / J4 trades against the next joint's origin shift (parallel
  // axes J2..J4), so the parameters are compared through the poses they produce. J5's offset and the J3 tilt
  // have no such partner and are compared directly.
  check('a. J5 zero offset and the J3 tilt recovered (non-redundant parameters)', Math.abs(offErr[3]) < 0.1 && Math.abs((kin.k[7 * 2 + 2] - kInj[7 * 2 + 2]) * D) < 0.1,
    `J2 offset error ${offErr[0].toFixed(3)} deg (warning kept for J2 only if < 0.5), J5 offset error ${offErr[3].toFixed(3)} deg, J3 tilt_x error ${((kin.k[16] - kInj[16]) * D).toFixed(3)} deg; J2..J4 offset errors ${offErr.slice(0, 3).map((e) => e.toFixed(2)).join(', ')} deg (absorbed by origin shifts)`);
  // verify view of a loaded session: a fresh capture reprojects within the gate; the board moved by 20 mm does not
  {
    const v = render(kInj, { poses: 1 })[0], Tend = T4(CW.kinFk(ch, v.q, kin.k)), args = { view: v, board, order: kin.order, K: r.final.K, Tend, X: kin.X };
    const good = CW.verifyView({ ...args, boards: r.final.boards });
    const moved = CW.verifyView({ ...args, boards: r.final.boards.map((B) => mul(make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.02, 0, 0]), B)) });
    check('a. verify view: same board within the gate, board moved 20 mm refused', good.ok && !moved.ok && good.board === v.ds,
      `same board ${good.mm.toFixed(3)} mm (${good.px.toFixed(2)} px, placement ${good.board}, ${good.markers} markers); moved ${moved.mm.toFixed(2)} mm; gate ${good.gate} mm`);
  }
  // generalisation: joint sets not in the capture (planned poses +-0.15 rad on J2..J5)
  const extra = [];
  for (const v of views) if (rnd() < 0.5) extra.push({ ...v, q: v.q.map((x, i) => (i >= 1 && i <= 4 ? x + 0.3 * (rnd() - 0.5) : x)) });
  const ag2 = poseAgreement(extra, kin, kInj);
  check('a. new joint sets (+-0.15 rad around the plan) predicted within 1 mm / 0.1 deg', ag2.mmMax < 1 && ag2.degMax < 0.1, `max ${ag2.mmMax.toFixed(3)} mm, ${ag2.degMax.toFixed(4)} deg over ${extra.length} joint sets`);
  check('a. all gates pass, apply body with kinematics', r.ok && r.apply && r.apply.kinematics.joints.length === 6 && r.apply.kinematics.fit.heldout_mm.length === 2
    && ['offset_rad', 'scale'].every((f) => r.apply.kinematics.joints[0][f] === 0 && r.apply.kinematics.joints[5][f] === 0) && Number.isFinite(r.apply.table_z_m),
    `ok ${r.ok}, keys ${Object.keys(r.apply || {}).join(',')}, table_z ${(r.apply.table_z_m * 1000).toFixed(2)} mm (truth 0.00), J5 zero warning ${kin.warnings.map((w) => `J${w.joint} ${w.deg.toFixed(1)}°`).join(',') || 'none'}`);
  {
    const k = kInj.slice(); k[7 * 4] = -6 / D;
    const r6 = solveKinematics({ views: render(k), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
    check('a. J5 zero offset > 5 deg warns (J5 -6 deg injected)', r6.ok && r6.warnings.length === 1 && r6.warnings[0].joint === 5, `warnings ${r6.warnings.map((w) => `J${w.joint} ${w.deg.toFixed(2)}°`).join(',')}, gates ${r6.ok ? 'pass' : 'refuse'}`);
    const k2 = kInj.slice(); k2[7 * 1] = -6 / D;
    const r2 = solveKinematics({ views: render(k2), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
    check('a. J2 zero offset (not identifiable) gives no warning (J2 -6 deg injected)', r2.warnings.length === 0, `J2 offset found ${(r2.k[7] * D).toFixed(2)} deg, warnings ${r2.warnings.length}; warning joints ${CW.KIN_GATES.zeroWarnJoints.join(',')}`);
  }
}
{
  const views = render(new Array(42).fill(0));
  const r = solveKinematics({ views, board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
  const k = r.k, J = [1, 2, 3, 4];
  const mx = (p, s) => Math.max(...J.map((j) => Math.abs(k[7 * j + p]))) * s;
  const off = mx(0, D), sc = mx(1, 100), tl = Math.max(mx(2, D), mx(3, D)), org = Math.max(mx(4, 1000), mx(5, 1000), mx(6, 1000));
  console.log(`      levels ${levelTable(r)}`);
  check('a. zero correction -> parameters ~0', r.level === 'L0' && off < 0.3 && sc < 0.5 && tl < 0.3 && org < 3 && r.heldOut.ab.mm < 0.5 && r.heldOut.ba.mm < 0.5,
    `level ${r.level}, max |offset| ${off.toFixed(3)} deg, |scale| ${sc.toFixed(3)} %, |tilt| ${tl.toFixed(3)} deg, |origin| ${org.toFixed(2)} mm; held-out ${r.heldOut.ab.mm.toFixed(3)} / ${r.heldOut.ba.mm.toFixed(3)} mm; ${r.ms} ms`);
}

// ---- e. new real case (B601-DM 2026-11-19, motors zeroed at rest): no systematic error, only random per-pose joint
// errors (uncorrected held-out ~3 mm), ~55 views per placement. Every correction overfits: L0 must be chosen, and the
// apply body carries kinematics null (the camera calibration is applied, a stored correction removed).
// KIN_E_SEEDS=n: also solve n further noise draws and print the chosen-level distribution (no check)
const eViews = (sd) => { seed = sd; return render(new Array(42).fill(0), { jointErr: Number(process.env.KIN_NOISE_DEG || 0.37) / D, perPose: true, poses: 28 }); };
if (process.env.KIN_E_SEEDS) {
  const count = {}, rows = [];
  for (let i = 1; i <= Number(process.env.KIN_E_SEEDS); i++) {
    const r = solveKinematics({ views: eViews(0xe0 + i), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
    count[r.level] = (count[r.level] || 0) + 1;
    rows.push(`${r.level} ${r.levels.map((l) => l.mean.toFixed(2)).join('/')}`);
  }
  console.log(`      e. seeds ${process.env.KIN_E_SEEDS}: chosen ${JSON.stringify(count)}; mean held-out L0/L1/L2/L3 per draw: ${rows.join(' | ')}`);
}
{
  const views = eViews(0xe0);
  const r = solveFine({ views, markerSide: board.markerSide, chain: CALM.CHAIN, X0: Xnom, thickness, solvedAt: '2026-11-19T00:00:00Z' });
  const kin = r.kin, h = kin.heldOut;
  console.log(`      views ${kin.views.used} (A ${kin.views.A}, B ${kin.views.B}); per-pose joint error ${process.env.KIN_NOISE_DEG || 0.37} deg std on J2..J5`);
  console.log(`      levels ${levelTable(kin)}`);
  console.log(`      gates ${gateStr(r.gates)}`);
  check('e. random per-pose errors only -> L0 chosen', kin.level === 'L0' && kin.levels[0].chosen && h.ab0.mm > 1.5 && h.ab0.mm < 4.5,
    `level ${kin.level}; uncorrected held-out A->B ${h.ab0.mm.toFixed(2)} B->A ${h.ba0.mm.toFixed(2)} mm (target ~3); gates ${r.ok ? 'pass' : 'refuse'}`);
  check('e. L0 apply body: camera calibration with kinematics null', r.apply && r.apply.kinematics === null && Object.prototype.hasOwnProperty.call(r.apply, 'kinematics')
    && r.apply.intrinsics && r.apply.camera_mount && Number.isFinite(r.apply.table_z_m) && r.apply.rms.kin_level === 'L0' && kin.joints.every((j) => j.offset_rad === 0 && j.scale === 0)
    && kin.warnings.length === 0,
    `kinematics ${JSON.stringify(r.apply && r.apply.kinematics)}, keys ${Object.keys(r.apply || {}).join(',')}, fx ${r.final.K.fx.toFixed(1)}, table_z ${(r.apply.table_z_m * 1000).toFixed(2)} mm, held-out gate ${h.ab.mm.toFixed(2)} / ${h.ba.mm.toFixed(2)} mm`);
}
// chooseKinLevel rules (held-out mm per direction)
{
  const c = (rows) => CW.chooseKinLevel(rows).index;
  const cases = [
    [[{ ab: 2.85, ba: 3.33 }, { ab: 2.8, ba: 3.4 }, { ab: 2.8, ba: 3.3 }, { ab: 2.61, ba: 7.74 }], 0, 'real 2026-11-19 shape: L3 B->A worse by 4.4 mm, L2 mean within the 0.1 mm tie of L0'],
    [[{ ab: 2.76, ba: 3.64 }, { ab: 2.3, ba: 2.9 }, { ab: 2.2, ba: 2.6 }, { ab: 2.01, ba: 2.23 }], 3, 'real2: L3 best'],
    [[{ ab: 3, ba: 3 }, { ab: 1.98, ba: 1.98 }, { ab: 1.95, ba: 1.98 }, { ab: 1.9, ba: 1.9 }], 1, 'L1 within 0.1 mm of the best (L3) -> L1'],
    [[{ ab: 3, ba: 3 }, { ab: 2.0, ba: 3.2 }, { ab: 1.0, ba: 3.05 }, { ab: 0.5, ba: 5 }], 2, 'L1 B->A +0.2 -> not eligible; L2 +0.05 eligible'],
    [[{ ab: 3, ba: 3 }, { ab: NaN, ba: NaN }, { ab: 3.2, ba: 2 }, { ab: 2, ba: 2 }], 3, 'NaN level ignored, L2 A->B +0.2 not eligible'],
  ];
  const bad = cases.filter(([rows, want]) => c(rows) !== want);
  check('e. chooseKinLevel rules', bad.length === 0, cases.map(([rows, want, why]) => `${why}: L${c(rows)} (want L${want})`).join('; '));
}

// ---- b. real data replay
{
  const dir = process.env.REAL2 || '/home/harvest/project/_scratch/calib-wizard/real2';
  const files = ['collect_b1.json', 'collect_b2.json'].map((f) => path.join(dir, f));
  if (!files.every((f) => fs.existsSync(f)) || !fs.existsSync(path.join(dir, 'Kreal.json'))) console.log(`SKIP  b. real data replay  (no ${files.join(', ')})`);
  else {
    const K = JSON.parse(fs.readFileSync(path.join(dir, 'Kreal.json'), 'utf8'));
    const views = [];
    files.forEach((f, ds) => {
      const d = JSON.parse(fs.readFileSync(f, 'utf8')), jm = d.joint_map; // model = sign * motor + offset_rad, as recorded
      for (const s of d.samples) for (const fr of s.frames) if (fr.markers.length >= 6) views.push({ markers: fr.markers, q: fr.pos.slice(0, 6).map((v, i) => jm.sign[i] * v + jm.offset_rad[i]), ds });
    });
    // start value of the mount: the nominal v8 holder (as real2/kincal2.mjs)
    const X0 = make(mul(CALM.axisR([0, 0, 1], -Math.PI / 2), CALM.axisR([1, 0, 0], -128 / D)), [-0.07, 0, 0.07]);
    const r = solveKinematics({ views, board: CW.makeBoard({ markerSide: 0.025 }), K, chain: CALM.CHAIN, X0 });
    const h = r.heldOut;
    console.log(`      levels ${levelTable(r)}`);
    console.log(`      final fit on A + B (level ${r.level}): ${fmtK(r.k)}`);
    console.log(`      gates ${gateStr(r.gates)}; warnings ${r.warnings.map((w) => `J${w.joint} ${w.deg.toFixed(2)}°`).join(',') || 'none'}`);
    check('b. real replay: a correction level beats L0, held-out both ways, gates', r.ok && r.level !== 'L0' && h.ab.mm < h.ab0.mm && h.ba.mm < h.ba0.mm,
      `level ${r.level}; model angles with the recorded joint map (J5 -4.0 deg set in armd): A->B ${h.ab0.mm.toFixed(2)} -> ${h.ab.mm.toFixed(2)} mm, B->A ${h.ba0.mm.toFixed(2)} -> ${h.ba.mm.toFixed(2)} mm, mean improvement ${(h.improve.mean * 100).toFixed(1)} % (per direction ${(h.improve.ab * 100).toFixed(1)} / ${(h.improve.ba * 100).toFixed(1)} %); train ${r.train.mm.toFixed(2)} mm; views ${r.views.used} (A ${r.views.A}, B ${r.views.B}, dropped ${r.views.dropped.length}); gates ${r.ok ? 'pass' : 'refuse'}; solve ${r.ms} ms`);
    // kincal2.mjs convention: motor angles without the J5 map offset (the solver finds the -4 deg itself)
    const raw = views.map((v) => ({ ...v, q: v.q.map((x, i) => (i === 4 ? x + 0.0698 : x)) }));
    const r2 = solveKinematics({ views: raw, board: CW.makeBoard({ markerSide: 0.025 }), K, chain: CALM.CHAIN, X0 }), h2 = r2.heldOut;
    console.log(`      levels (no J5 map offset) ${levelTable(r2)}`);
    check('b. real replay without the J5 map offset (kincal2.mjs convention)', r2.ok && r2.level !== 'L0',
      `level ${r2.level}; A->B ${h2.ab0.mm.toFixed(2)} -> ${h2.ab.mm.toFixed(2)} mm, B->A ${h2.ba0.mm.toFixed(2)} -> ${h2.ba.mm.toFixed(2)} mm (kincal2.mjs: 4.43 -> 2.01, 5.91 -> 2.24), J5 offset ${(r2.k[28] * D).toFixed(2)} deg, warnings ${r2.warnings.map((w) => `J${w.joint} ${w.deg.toFixed(2)}°`).join(',') || 'none'}; gates ${r2.ok ? 'pass' : 'refuse'}; solve ${r2.ms} ms`);
  }
}

// ---- c. gate cases
{
  const k = kInj.slice(); k[7 * 4] = 12 / D;
  const r = solveKinematics({ views: render(k), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
  const g = r.gates.find((x) => x.key === 'kin_bounds');
  // a level whose fit leaves the bounds is not eligible: the J5 12 deg level is skipped, the remaining levels cannot
  // explain the error (held-out > 5 mm), so the solve is refused and no out-of-bounds correction is ever applied
  const oobLevels = r.levels.filter((l) => l.outOfBounds).map((l) => l.name);
  check('c. J5 zero offset 12 deg (bound 10) -> refused, out-of-bounds levels not chosen', !r.ok && oobLevels.length > 0 && !r.levels.find((l) => l.chosen).outOfBounds && r.outOfBounds.length === 0,
    `out-of-bounds levels ${oobLevels.join(',')}, chosen ${r.level}, J5 offset ${(r.k[28] * D).toFixed(2)} deg; gates ${gateStr(r.gates)}`);
}
{
  const r = solveKinematics({ views: render(kInj, { jointErr: 1.2 / D }), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
  const bad = r.gates.filter((x) => !x.ok).map((x) => x.key);
  console.log(`      levels ${levelTable(r)}`);
  check('c. unmodelled joint errors, held-out > 3 mm -> refused', !r.ok && (bad.includes('kin_heldout_ab') || bad.includes('kin_heldout_ba')),
    `held-out ${r.heldOut.ab.mm.toFixed(2)} / ${r.heldOut.ba.mm.toFixed(2)} mm; failed ${bad.join(',')}`);
}
{
  const r = solveKinematics({ views: render(kInj).filter((v) => v.ds === 0), board, K: Ktrue, chain: CALM.CHAIN, X0: Xnom });
  check('c. one placement only -> refused before solving', !r.ok && !r.joints && r.gates[0].key === 'kin_views' && !r.gates[0].ok, gateStr(r.gates));
}
// ---- d. step generators (the cockpit runs them in a Web Worker, or on the main thread in slices): the longest
// synchronous stretch between two yields, real-arm views through solveFine (intrinsics + kinematics) and a
// fine-mode plan. Measured on this machine; a phone / Pi is slower by a factor to be measured there.
{
  const longest = (gen) => { let t = performance.now(), worst = 0, steps = 0; for (;;) { const r = gen.next(); const n = performance.now(); worst = Math.max(worst, n - t); t = n; steps++; if (r.done) return { worst, steps, value: r.value }; } };
  const dir = process.env.REAL2 || '/home/harvest/project/_scratch/calib-wizard/real2';
  let views = render(kInj);
  if (fs.existsSync(path.join(dir, 'collect_b2.json'))) {
    views = [];
    ['collect_b1.json', 'collect_b2.json'].forEach((f, ds) => { const d = JSON.parse(fs.readFileSync(path.join(dir, f), 'utf8')), jm = d.joint_map; for (const sm of d.samples) for (const fr of sm.frames) if (fr.markers.length >= 6) views.push({ markers: fr.markers, q: fr.pos.slice(0, 6).map((v, i) => jm.sign[i] * v + jm.offset_rad[i]), ds }); });
  }
  const t0 = performance.now();
  const a = longest(CW.solveFineSteps({ views, markerSide: 0.025, chain: CALM.CHAIN, X0: make(mul(CALM.axisR([0, 0, 1], -Math.PI / 2), CALM.axisR([1, 0, 0], -128 / D)), [-0.07, 0, 0.07]), prior: PRIOR }));
  const total = performance.now() - t0;
  const q0 = plans[0][0];
  const b = longest(CW.generatePosesSteps({ ...FINE, Tbb: PLACE[0], board, K: CW.NOMINAL, X: Xnom, q0, fk, ik, lo, hi }));
  check('d. longest synchronous step < 50 ms (solveFine on the real views, fine-mode plan)', a.worst < 50 && b.worst < 50,
    `solveFine ${views.length} views: ${a.steps} steps, longest ${a.worst.toFixed(1)} ms, total ${(total / 1000).toFixed(1)} s (intrinsics included), gates ${a.value.ok ? 'pass' : 'refuse'}; generatePoses: ${b.steps} steps, longest ${b.worst.toFixed(1)} ms, ${b.value.poses.length} poses`);
}
// ---- screenViews: duplicate marker ids and a view no single homography explains are dropped before solving
{
  const sv = CW;
  const b = sv.makeBoard({ markerSide: 0.025 });
  const K = { fx: 810, fy: 808, cx: 320, cy: 185, k1: 0, k2: 0 };
  const R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], t = [0, 0, 0.3];
  const good = { markers: b.markers.map((m) => ({ id: m.id, corners: m.corners.map((c) => sv.project(K, R, t, c).slice(0, 2)) })) };
  const dup = { markers: [...good.markers, { ...good.markers[0], corners: good.markers[0].corners.map(([u, v]) => [u + 150, v + 40]) }] };
  const wrong = { markers: good.markers.map((m, k) => (k === 3 ? { ...m, id: 9 } : k === 9 ? { ...m, id: 3 } : m)) };
  const r = sv.screenViews([good, dup, wrong, good], b);
  const why = r.rejected.map((x) => `${x.i}:${x.why}`).join(',');
  if (r.views.length !== 2 || why !== '1:duplicate_ids,2:inconsistent') { console.log('FAIL  screenViews', why, r.views.length); fails++; }
  else console.log('PASS  screenViews drops duplicate ids and inconsistent views', why);
}
console.log(fails ? `${fails} checks failed` : 'all checks passed');
process.exit(fails ? 1 : 0);
