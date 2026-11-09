#!/usr/bin/env node
// Self-test of the cockpit hand-eye calibration math (src/api/index.html, block CALIB-MATH-BEGIN .. END).
// Runs without a browser:  node scripts/calib_selftest.mjs
//   a. full URDF FK vs tests/fixtures/fk_samples.json (generated from rebot_companion.kinematics.tcp_pose)
//   b. synthetic eye-in-hand AX = XB recovery (zero noise and 0.5 mm / 0.2 deg noise)
//   c. the exported eye-in-hand / eye-to-hand JSON loads with rebot_companion HandEyeCalibration.load_result
//      (needs uv and the SDK checkout; REBOT_COMPANION=<dir>, default ../rebot-companion; skipped if missing)
//   d. the rotation diversity check rejects rotations about a single axis
//   e. eye-to-hand Horn point fit with the full FK still recovers T_cam_to_base
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import vm from 'node:vm';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const a = html.indexOf('/* CALIB-MATH-BEGIN */'), b = html.indexOf('/* CALIB-MATH-END */');
if (a < 0 || b < a) throw new Error('CALIB-MATH block not found in index.html');
const ctx = vm.createContext({});
const { CALM, solveRigid } = vm.runInContext(html.slice(a, b) + '\n({ CALM, solveRigid });', ctx);

let fails = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
const D = 180 / Math.PI;
const posErr = (A, B) => Math.hypot(...CALM.posOf(A).map((v, k) => v - CALM.posOf(B)[k]));
const rotErr = (A, B) => CALM.angDeg(CALM.mul(CALM.rotOf(CALM.inv(A)), CALM.rotOf(B)));

// deterministic PRNG (mulberry32) + Gaussian
let seed = 0x5eed1109;
const rnd = () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const uni = (lo, hi) => lo + (hi - lo) * rnd();
const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
const randT = (tr, rotRad) => CALM.make(CALM.expR([gauss(), gauss(), gauss()].map((v) => v * rotRad)), [uni(-tr, tr), uni(-tr, tr), uni(-tr, tr)]);
// perturb T by a random rotation of sigma_deg (per axis) and translation of sigma_m (per axis)
const noisy = (T, sm, sd) => CALM.mul(T, CALM.make(CALM.expR([gauss(), gauss(), gauss()].map((v) => v * sd / D)), [gauss() * sm, gauss() * sm, gauss() * sm]));

// ---- a. FK vs SDK fixture
const fx = JSON.parse(fs.readFileSync(path.join(root, 'tests/fixtures/fk_samples.json'), 'utf8'));
let fkPos = 0, fkRot = 0;
for (const s of fx.samples) { const T = CALM.fk(s.q); fkPos = Math.max(fkPos, posErr(T, s.T)); fkRot = Math.max(fkRot, rotErr(T, s.T)); }
check('a. FK vs SDK tcp_pose', fx.samples.length >= 50 && fkPos < 1e-4 && fkRot < 0.01, `n=${fx.samples.length} max pos ${(fkPos * 1000).toExponential(2)} mm, max rot ${fkRot.toExponential(2)} deg`);

// ---- b. synthetic eye-in-hand
const Xtrue = CALM.make(CALM.rpyR(uni(-3, 3), uni(-1.2, 1.2), uni(-3, 3)), [uni(-0.06, 0.06), uni(-0.06, 0.06), uni(0, 0.08)]);
const Ytrue = CALM.make(CALM.rpyR(uni(-0.3, 0.3), uni(-0.3, 0.3), uni(-3, 3)), [uni(0.25, 0.4), uni(-0.15, 0.15), 0.0]);
const qs = [];
while (qs.length < 12) {
  const q = [uni(-0.6, 0.6), uni(0.4, 1.2), uni(0.8, 1.8), uni(-0.6, 0.6), uni(-0.6, 0.6), uni(-1.2, 1.2)];
  if (Math.abs(q[4]) > 0.1 && Math.abs(q[5]) > 0.1) qs.push(q);
}
const G = qs.map((q) => CALM.fk(q));
const Mexact = G.map((Gi) => CALM.mul(CALM.inv(CALM.mul(Gi, Xtrue)), Ytrue)); // T_cam_marker = (G X)^-1 Y
const r0 = CALM.solveEyeInHand(G, Mexact);
check('b. eye-in-hand zero noise', !r0.err && posErr(r0.X, Xtrue) < 1e-6 && rotErr(r0.X, Xtrue) < 1e-6 && r0.rmse < 1e-6,
  r0.err || `X err ${posErr(r0.X, Xtrue).toExponential(2)} m / ${rotErr(r0.X, Xtrue).toExponential(2)} deg, rmse ${r0.rmse.toExponential(2)} m`);
const Gn = G.map((T) => noisy(T, 0.0005, 0.2)), Mn = Mexact.map((T) => noisy(T, 0.0005, 0.2));
const r1 = CALM.solveEyeInHand(Gn, Mn);
check('b. eye-in-hand 0.5 mm / 0.2 deg noise', !r1.err && posErr(r1.X, Xtrue) < 0.002 && rotErr(r1.X, Xtrue) < 0.5,
  r1.err || `n=${G.length} X err ${(posErr(r1.X, Xtrue) * 1000).toFixed(3)} mm / ${rotErr(r1.X, Xtrue).toFixed(3)} deg; quality rmse ${(r1.rmse * 1000).toFixed(2)} mm / ${r1.rotRmse.toFixed(3)} deg; diversity max ${r1.div.maxDeg.toFixed(1)} deg ratio ${r1.div.ratio.toFixed(2)}`);
// an outlier sample stands out in the residuals
const Mo = Mn.slice(); Mo[3] = noisy(Mn[3], 0.02, 3);
const r2 = CALM.solveEyeInHand(Gn, Mo);
const worst = r2.residuals.indexOf(Math.max(...r2.residuals));
check('b. outlier sample has the largest residual', !r2.err && worst === 3, r2.err || `worst #${worst + 1} ${(r2.residuals[worst] * 1000).toFixed(1)} mm, rmse ${(r2.rmse * 1000).toFixed(1)} mm`);

// ---- d. diversity check
const Gz = Array.from({ length: 9 }, (_, i) => CALM.fk([-0.8 + 0.2 * i, 0.8, 1.2, 0.2, 0, 0])); // J1 only: all about base z
const dz = CALM.rotDiversity(Gz), rz = CALM.solveEyeInHand(Gz, Gz.map((Gi) => CALM.mul(CALM.inv(CALM.mul(Gi, Xtrue)), Ytrue)));
check('d. single-axis set rejected', !dz.ok && dz.err === 'calib.e.rot_axis' && rz.err === 'calib.e.rot_axis', `err ${dz.err}, ratio ${dz.ratio.toExponential(2)}, max ${dz.maxDeg.toFixed(1)} deg`);
const Gs = Array.from({ length: 9 }, (_, i) => CALM.fk([0.01 * i, 0.8, 1.2, 0.2, 0.01 * (i % 3), 0]));
const ds = CALM.rotDiversity(Gs);
check('d. small-rotation set rejected', !ds.ok && ds.err === 'calib.e.rot_small', `err ${ds.err}, max ${ds.maxDeg.toFixed(1)} deg`);
const dg = CALM.rotDiversity(G);
check('d. diverse set accepted', dg.ok, `ratio ${dg.ratio.toFixed(2)}, max ${dg.maxDeg.toFixed(1)} deg`);

// ---- e. eye-to-hand point fit with the full FK (marker centre at the TCP)
const Tcb = CALM.make(CALM.rpyR(uni(-3, 3), uni(-1, 1), uni(-3, 3)), [0.4, -0.2, 0.6]); // T_cam_to_base
const pBase = qs.map((q) => CALM.posOf(CALM.fk(q)));
const pCam = pBase.map((p) => CALM.posOf(CALM.mul(CALM.inv(Tcb), CALM.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], p))));
const re = solveRigid(pCam, pBase);
check('e. eye-to-hand Horn fit', !re.err && posErr(CALM.make(re.R, re.t), Tcb) < 1e-6 && rotErr(CALM.make(re.R, re.t), Tcb) < 1e-6, re.err || `rmse ${re.rmse.toExponential(2)} m`);

// ---- c. SDK round-trip of the exports
const sdk = process.env.REBOT_COMPANION || path.resolve(root, '../rebot-companion');
const at = new Date().toISOString();
const eihFile = CALM.buildExport({
  mode: 'eye_in_hand', markerId: 7, result: { ...r1, n: G.length, at },
  samples: qs.map((q, i) => ({ joints: [...q, 0], p_base: CALM.posOf(Gn[i]), T_base_gripper: Gn[i], marker_id: 7, p_cam: CALM.posOf(Mn[i]), R_cam: CALM.rotOf(Mn[i]), rpy_cam_deg: CALM.rpyDeg(CALM.rotOf(Mn[i])), T_cam_marker: Mn[i], ts: at })),
});
const ethFile = CALM.buildExport({
  mode: 'eye_to_hand', markerId: 3, result: { ...re, n: qs.length, at },
  samples: qs.map((q, i) => ({ joints: [...q, 0], p_base: pBase[i], T_base_gripper: CALM.fk(q), marker_id: 3, p_cam: pCam[i], R_cam: null, rpy_cam_deg: null, T_cam_marker: null, ts: at })),
});
if (!fs.existsSync(path.join(sdk, 'rebot_companion/calibration.py'))) {
  console.log(`SKIP  c. SDK round-trip  (no rebot-companion at ${sdk})`);
} else {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'calib-selftest-'));
  const fEih = path.join(dir, 'eih.json'), fEth = path.join(dir, 'eth.json');
  fs.writeFileSync(fEih, JSON.stringify(eihFile, null, 2));
  fs.writeFileSync(fEth, JSON.stringify(ethFile, null, 2));
  const py = `
import json, sys, numpy as np
from rebot_companion.calibration import HandEyeCalibration
eih, eth = sys.argv[1], sys.argv[2]
r = HandEyeCalibration.load_result(eih)
assert r.setup == 'eye_in_hand', r.setup
X = np.array(json.load(open(eih))['result']['T_cam_to_gripper'])
d = float(np.abs(r.transform.matrix - X).max())
assert d < 1e-12, d
q = json.load(open(eih))['samples'][0]['joints']
cb = r.cam_to_base(q)  # T_base_gripper(q) @ X via the SDK FK
c = HandEyeCalibration.load_samples(eih, method='axxb')
c.solve()
dx = float(np.abs(c.result.transform.matrix - X).max())
r2 = HandEyeCalibration.load_result(eth)
assert r2.setup == 'eye_to_hand', r2.setup
d2 = float(np.abs(r2.transform.matrix - np.array(json.load(open(eth))['result']['T_cam_to_base'])).max())
assert d2 < 1e-12, d2
assert HandEyeCalibration.load_from_file(eth) is not None
c2 = HandEyeCalibration.load_samples(eth, method='point'); c2.solve()
d3 = float(np.abs(c2.result.transform.matrix - r2.transform.matrix).max())
print(f"eye_in_hand setup={r.setup} matrix_diff={d:.1e} cam_to_base(q0).t={np.round(cb.translation, 4).tolist()} sdk_axxb_resolve_diff={dx:.2e} n={len(c.samples)}")
print(f"eye_to_hand setup={r2.setup} matrix_diff={d2:.1e} sdk_point_resolve_diff={d3:.2e}")
`;
  try {
    const out = execFileSync('uv', ['run', '--quiet', 'python', '-c', py, fEih, fEth], { cwd: sdk, encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] });
    const m = /sdk_axxb_resolve_diff=([\d.e+-]+)/.exec(out), m3 = /sdk_point_resolve_diff=([\d.e+-]+)/.exec(out);
    process.stdout.write(out.trimEnd().replace(/^/gm, '      ') + '\n');
    check('c. SDK load_result round-trip', true, 'eye_in_hand + eye_to_hand files load');
    check('c. SDK re-solve of exported samples matches page', m && Number(m[1]) < 1e-6 && m3 && Number(m3[1]) < 1e-6, `axxb ${m && m[1]}, point ${m3 && m3[1]}`);
  } catch (e) {
    check('c. SDK load_result round-trip', false, String(e.stderr || e.message).trim());
  }
  fs.rmSync(dir, { recursive: true, force: true });
}

console.log(fails ? `${fails} FAILED` : 'ALL PASS');
process.exit(fails ? 1 : 0);
