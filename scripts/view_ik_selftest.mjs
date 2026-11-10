#!/usr/bin/env node
// Self-test of the camera-view cartesian IK (src/api/index.html, blocks CALIB-MATH and VIEW-IK).
// Runs without a browser:  node scripts/view_ik_selftest.mjs
//   a. VIK.fk equals CALM.fk (full URDF chain)
//   b. FK -> IK round trip: 200 random joint sets (inside limits, away from singularities), perturbed start
//   c. continuity: 0.2 m forward / strafe lines and a 90 deg look-around at 40 Hz, solved tick by tick
//   d. semantics: forward = z_cam, strafe = x_cam, look-around / pitch keep the position, auto-level after a
//      rotation, none after a pure translation
//   e. unreachable / invalid targets return ok:false without throwing
//   f. timing: median per-solve time (target < 0.5 ms)
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const block = (name) => {
  const a = html.indexOf(`/* ${name}-BEGIN */`), b = html.indexOf(`/* ${name}-END */`);
  if (a < 0 || b < a) throw new Error(`${name} block not found in index.html`);
  return html.slice(a, b);
};
const ctx = vm.createContext({});
const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', ctx);

let fails = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
const D = 180 / Math.PI;
let seed = 0x7a11e109;
const rnd = () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const uni = (lo, hi) => lo + (hi - lo) * rnd();
const posOf = CALM.posOf;
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const errOf = (T, q) => { const e = VIK.diff(VIK.fk(q), T); return { mm: e.d * 1000, deg: e.a * D }; };

// model limits = default cockpit motor limits (S.limits) - limitMargin 0.02; J1..J4 sign 1 / offset 0
const LIM = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]]
  .map(([a, b]) => [a + 0.02, b - 0.02]);
const lo = LIM.map((l) => l[0]), hi = LIM.map((l) => l[1]);
const MAXJUMP = 0.3;

// smallest singular value of the (w-scaled) Jacobian, numerically, via Jacobi eigenvalues of J^T J
function sigmaMin(q, w = 0.2) {
  const T0 = VIK.fk(q), J = [];
  for (let j = 0; j < 6; j++) {
    const qq = q.slice(); qq[j] += 1e-6;
    const e = VIK.diff(T0, VIK.fk(qq));
    J.push([...e.dp, ...e.dr.map((v) => v * w)].map((v) => v / 1e-6));
  }
  const A = J.map((ci) => J.map((cj) => ci.reduce((s, v, k) => s + v * cj[k], 0)));
  for (let sweep = 0; sweep < 30; sweep++) for (let p = 0; p < 6; p++) for (let r = p + 1; r < 6; r++) {
    if (Math.abs(A[p][r]) < 1e-15) continue;
    const th = (A[r][r] - A[p][p]) / (2 * A[p][r]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
    const c = 1 / Math.sqrt(t * t + 1), s = t * c;
    for (let k = 0; k < 6; k++) { const a = A[k][p], b = A[k][r]; A[k][p] = c * a - s * b; A[k][r] = s * a + c * b; }
    for (let k = 0; k < 6; k++) { const a = A[p][k], b = A[r][k]; A[p][k] = c * a - s * b; A[r][k] = s * a + c * b; }
  }
  return Math.sqrt(Math.max(0, Math.min(...A.map((r, i) => r[i]))));
}

// ---- a. FK
let fkErr = 0;
for (let n = 0; n < 200; n++) {
  const q = LIM.map(([a, b]) => uni(a, b));
  const A = CALM.fk(q), B = VIK.fk(q);
  fkErr = Math.max(fkErr, dist(posOf(A), posOf(B)), ...A.flat().map((v, i) => Math.abs(v - B.flat()[i])));
}
check('a. VIK.fk == CALM.fk', fkErr < 1e-12, `max abs diff ${fkErr.toExponential(2)}`);

// ---- b. round trip
const samples = [];
let skipped = 0;
while (samples.length < 200) {
  const q = LIM.map(([a, b]) => uni(a + 0.15, b - 0.15));
  if (sigmaMin(q) < 0.02) { skipped++; continue; }
  samples.push(q);
}
let worstMm = 0, worstDeg = 0, bad = 0, maxIt = 0;
for (const q of samples) {
  const T = VIK.fk(q), q0 = q.map((x) => x + uni(-0.1, 0.1));
  const r = VIK.solve(T, q0, { lo, hi });
  const e = errOf(T, r.q);
  if (!r.ok || e.mm > 1 || e.deg > 0.5) bad++;
  worstMm = Math.max(worstMm, e.mm); worstDeg = Math.max(worstDeg, e.deg); maxIt = Math.max(maxIt, r.it);
}
check('b. FK->IK round trip (200, start +-0.1 rad)', bad === 0, `fail ${bad}, worst ${worstMm.toFixed(4)} mm / ${worstDeg.toFixed(4)} deg, max iters ${maxIt}, skipped near-singular ${skipped}`);

// ---- c. continuity
const Rec = VIK.camMount(false, 0);
const LEVEL = Math.cos(15 / D);
const START = [0, -1.167, -1.565, 1.435, 0, Math.PI / 2]; // cockpit "inspect" preset, J6 at pi/2
function runPath(name, v, nTicks, vl, wl) {
  let q = START.slice(), T = VIK.level(VIK.fk(q), Rec, LEVEL);
  const r0 = VIK.solve(T, q, { lo, hi });
  if (!r0.ok) return check(`c. ${name}`, false, 'start pose did not solve');
  q = r0.q;
  const p0 = posOf(T);
  let maxStep = 0, worst = 0, okAll = true, flip = false;
  const s3 = Math.sign(q[2]);
  for (let k = 0; k < nTicks; k++) {
    T = VIK.integrate(T, v, 1 / 40, { vl, wl, Rec, levelCos: LEVEL });
    const r = VIK.solve(T, q, { lo, hi });
    if (!r.ok) { okAll = false; break; }
    maxStep = Math.max(maxStep, ...r.q.map((x, i) => Math.abs(x - q[i])));
    const e = errOf(T, r.q); worst = Math.max(worst, e.mm);
    if (Math.sign(r.q[2]) !== s3 || Math.abs(r.q[4]) > 1.45) flip = true;
    q = r.q;
  }
  const moved = dist(posOf(T), p0);
  check(`c. ${name}`, okAll && maxStep < MAXJUMP && !flip, `${nTicks} ticks, moved ${(moved * 1000).toFixed(1)} mm, max joint step ${maxStep.toFixed(4)} rad (< ${MAXJUMP}), worst ${worst.toFixed(3)} mm, flip ${flip}`);
  return T;
}
runPath('forward 0.2 m', { x: 1, y: 0, z: 0, p: 0, r: 0 }, 80, 0.1, 0.8);
runPath('strafe right 0.2 m', { x: 0, y: -1, z: 0, p: 0, r: 0 }, 80, 0.1, 0.8);
runPath('look around 90 deg', { x: 0, y: 0, z: 0, p: 0, r: 1 }, Math.ceil((Math.PI / 2) / (0.8 / 40)), 0.1, 0.8);
runPath('pitch up 25 deg', { x: 0, y: 0, z: 0, p: -1, r: 0 }, Math.ceil((25 / D) / (0.8 / 40)), 0.1, 0.8);

// ---- d. semantics
const o = { vl: 0.1, wl: 0.8, Rec, levelCos: LEVEL };
const cRaw = VIK.camAxes(VIK.fk(START), Rec), cZero = VIK.camAxes(VIK.fk([0, 0, 0, 0, 0, Math.PI / 2]), Rec);
check('d. default mount (camFlip off) level and upright at q6 = pi/2', cRaw.y[2] < 0 && Math.abs(cRaw.x[2]) < 1e-4 && cZero.y[2] < -0.999,
  `inspect pose: x_cam.Z ${cRaw.x[2].toExponential(1)}, y_cam.Z ${cRaw.y[2].toFixed(4)}; zero pose: y_cam.Z ${cZero.y[2].toFixed(4)}`);
const T0 = VIK.level(VIK.fk(START), Rec, LEVEL), c0 = VIK.camAxes(T0, Rec);
const step = (v) => VIK.integrate(T0, { x: 0, y: 0, z: 0, p: 0, r: 0, ...v }, 0.1, o);
const disp = (T) => posOf(T).map((x, k) => x - posOf(T0)[k]);
const fw = disp(step({ x: 1 })), sr = disp(step({ y: -1 })), up = disp(step({ z: 1 }));
check('d. forward moves along z_cam', dot(fw, c0.z) / Math.hypot(...fw) > 0.999999 && Math.abs(Math.hypot(...fw) - 0.01) < 1e-9, `cos ${(dot(fw, c0.z) / Math.hypot(...fw)).toFixed(7)}, |d| ${(Math.hypot(...fw) * 1000).toFixed(3)} mm`);
check('d. strafe right moves along +x_cam', dot(sr, c0.x) / Math.hypot(...sr) > 0.999999, `cos ${(dot(sr, c0.x) / Math.hypot(...sr)).toFixed(7)}`);
check('d. lift moves along world Z', up[2] / Math.hypot(...up) > 0.999999, `dz ${(up[2] * 1000).toFixed(3)} mm`);
const yawOf = (T) => { const z = VIK.camAxes(T, Rec).z; return Math.atan2(z[1], z[0]); };
const Tr = step({ r: 1 }), cr = VIK.camAxes(Tr, Rec);
check('d. look right: position kept, heading turns right, x_cam level', dist(posOf(Tr), posOf(T0)) < 1e-12 && yawOf(Tr) < yawOf(T0) && Math.abs(cr.x[2]) < 1e-9 && cr.y[2] <= 0,
  `dpos ${dist(posOf(Tr), posOf(T0)).toExponential(1)} m, heading ${(yawOf(T0) * D).toFixed(2)} -> ${(yawOf(Tr) * D).toFixed(2)} deg, x_cam.Z ${cr.x[2].toExponential(1)}`);
const Tp = step({ p: -1 }), cp = VIK.camAxes(Tp, Rec);
check('d. look up: position kept, optical axis rises', dist(posOf(Tp), posOf(T0)) < 1e-12 && cp.z[2] > c0.z[2], `dpos ${dist(posOf(Tp), posOf(T0)).toExponential(1)} m, z_cam.Z ${c0.z[2].toFixed(4)} -> ${cp.z[2].toFixed(4)}`);
let lvWorst = 0, lvDown = true;
for (const rollDeg of [20, -35, 90, 170, -150]) {
  const Rrolled = CALM.mul(CALM.rotOf(T0), CALM.mul(Rec, CALM.mul(CALM.axisR([0, 0, 1], rollDeg / D), CALM.inv(CALM.make(Rec, [0, 0, 0])).slice(0, 3).map((r) => r.slice(0, 3)))));
  const Tl = VIK.level(CALM.make(Rrolled, posOf(T0)), Rec, LEVEL), cl = VIK.camAxes(Tl, Rec);
  lvWorst = Math.max(lvWorst, Math.abs(cl.x[2])); if (cl.y[2] > 0) lvDown = false;
}
check('d. auto-level from rolled poses (20..170 deg)', lvWorst < 1e-9 && lvDown, `max |x_cam.Z| ${lvWorst.toExponential(1)}, y_cam.Z <= 0: ${lvDown}`);
// a pure translation keeps the orientation as it is (the cockpit's look stick moves the wrist joints and may
// leave a roll; translation must not level it away)
const Trl = VIK.fk([0, -1.167, -1.565, 1.435, 0.3, Math.PI / 2]), Ttr = VIK.integrate(Trl, { x: 1, y: -0.5, z: 0.3, p: 0, r: 0 }, 0.1, o);
const rTr = VIK.diff(Trl, Ttr);
check('d. translation keeps a rolled orientation (no auto-level)', rTr.a < 1e-12 && rTr.d > 0.005, `rot change ${rTr.a.toExponential(1)} rad, moved ${(rTr.d * 1000).toFixed(1)} mm, x_cam.Z ${VIK.camAxes(Trl, Rec).x[2].toFixed(4)}`);
const Tdown = CALM.make(CALM.mul(CALM.axisR([0, 1, 0], 85 / D), CALM.rotOf(CALM.make([[1, 0, 0], [0, 0, -1], [0, 1, 0]], [0, 0, 0]))), [0.3, 0, 0.2]); // tool axis 85 deg down
const vd = VIK.camAxes(Tdown, Rec).z[2];
const Tdl = VIK.level(Tdown, Rec, LEVEL);
check('d. no auto-level near vertical optical axis', Math.abs(vd) < LEVEL || Tdl === Tdown, `z_cam.Z ${vd.toFixed(4)}, unchanged ${Tdl === Tdown}`);

// ---- e. unreachable / invalid
let threw = false, rf, rn;
try {
  rf = VIK.solve(CALM.make(CALM.rotOf(T0), [2, 0, 0.3]), START, { lo, hi });
  rn = VIK.solve(CALM.make(CALM.rotOf(T0), [NaN, 0, 0.3]), START, { lo, hi });
} catch (e) { threw = e.message; }
check('e. unreachable target rejected', !threw && rf.ok === false && rf.err === 'pose' && rn.ok === false, threw || `far: ok ${rf.ok} err ${rf.err} (${(rf.ep * 1000).toFixed(0)} mm left); NaN: ok ${rn.ok}`);
const Tlim = VIK.fk([0, -1.167, -1.565, 1.435, 2.3, Math.PI / 2]); // J5 beyond its limit
const rl = VIK.solve(Tlim, START, { lo, hi });
check('e. target needing J5 past its limit rejected', rl.ok === false && rl.q[4] <= hi[4] + 1e-12, `ok ${rl.ok}, q5 ${rl.q[4].toFixed(3)} <= ${hi[4].toFixed(3)}`);

// ---- f. timing
const timeIt = (fn, n) => { const t0 = performance.now(); for (let i = 0; i < n; i++) fn(i); return (performance.now() - t0) / n; };
const tick = samples.map((q) => { const T = VIK.fk(q); return [T, q.map((x) => x + 0.005)]; });
const pert = samples.map((q) => [VIK.fk(q), q.map((x, i) => x + 0.1 * Math.sin(7 * i + q[0]))]);
for (let w = 0; w < 3; w++) timeIt((i) => VIK.solve(...tick[i % 200], { lo, hi }), 2000); // warm up the JIT
const med = (fn) => { const b = []; for (let k = 0; k < 15; k++) b.push(timeIt(fn, 400)); b.sort((x, y) => x - y); return b[7]; };
const msTick = med((i) => VIK.solve(...tick[i % 200], { lo, hi }));
const msPert = med((i) => VIK.solve(...pert[i % 200], { lo, hi }));
check('f. median solve time < 0.5 ms', msTick < 0.5 && msPert < 0.5, `teleop tick (0.005 rad start error) ${(msTick * 1000).toFixed(1)} us, perturbed start (0.1 rad) ${(msPert * 1000).toFixed(1)} us`);

console.log(fails ? `\n${fails} FAILED` : '\nALL PASS');
process.exit(fails ? 1 : 0);
