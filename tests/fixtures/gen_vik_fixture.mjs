#!/usr/bin/env node
// Generates tests/fixtures/vik_samples.json from the cockpit's JS kinematics (src/api/index.html,
// blocks CALIB-MATH and VIEW-IK). The Rust port in src/kinematics.rs is cross-checked against it
// by tests/test_kinematics.rs.  Run:  node tests/fixtures/gen_vik_fixture.mjs
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '..', '..');
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const block = (name) => {
  const a = html.indexOf(`/* ${name}-BEGIN */`), b = html.indexOf(`/* ${name}-END */`);
  if (a < 0 || b < a) throw new Error(`${name} block not found in index.html`);
  return html.slice(a, b);
};
const ctx = vm.createContext({});
const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', ctx);

let seed = 20261109;
const rnd = () => { seed |= 0; seed = (seed + 0x6d2b79f5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const uni = (lo, hi) => lo + (hi - lo) * rnd();
const LIM = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]]
  .map(([a, b]) => [a + 0.02, b - 0.02]);
const lo = LIM.map((l) => l[0]), hi = LIM.map((l) => l[1]);
const pose = (T) => ({ r: CALM.rotOf(T), p: CALM.posOf(T) });

const fk = [];
for (let n = 0; n < 40; n++) {
  const q = LIM.map(([a, b]) => uni(a, b));
  const f = VIK.frames(q);
  fk.push({ q, ...pose(VIK.fk(q)), axes: f.a, origins: f.o });
}
const solve = [];
for (let n = 0; n < 40; n++) {
  const q = LIM.map(([a, b]) => uni(a + 0.15, b - 0.15));
  const q0 = q.map((x) => x + uni(-0.1, 0.1));
  const r = VIK.solve(VIK.fk(q), q0, { lo, hi });
  solve.push({ goal: pose(VIK.fk(q)), q0, ok: r.ok, q: r.q, ep: r.ep, er: r.er, it: r.it });
}
// a goal far outside the workspace and a start outside the limits
for (const [goalQ, q0, shift] of [[[0, 0, 0, 0, 0, 1.5708], [0, 0, 0, 0, 0, 1.5708], [0.8, 0, 0]],
                                 [[0.3, -1.0, -1.5, 1.4, 0.1, 1.5708], [0.3, -2.2, -1.5, 1.4, 0.1, 1.5708], [0, 0, 0]]]) {
  const T = VIK.fk(goalQ); for (let k = 0; k < 3; k++) T[k][3] += shift[k];
  const r = VIK.solve(T, q0, { lo, hi });
  solve.push({ goal: pose(T), q0, ok: r.ok, q: r.q, ep: r.ep, er: r.er, it: r.it });
}
const camMount = [[false, 0], [true, 0], [false, 30], [true, -45]].map(([flip, rot]) => ({ flip, rot_deg: rot, r: VIK.camMount(flip, rot) }));
const LEVEL = Math.cos(15 * Math.PI / 180);
const Rec = VIK.camMount(false, 0);
const level = [];
for (let n = 0; n < 10; n++) {
  const q = LIM.map(([a, b]) => uni(a, b));
  level.push({ q, ...pose(VIK.level(VIK.fk(q), Rec, LEVEL)) });
}
const rot = [];
for (let n = 0; n < 10; n++) {
  const R = CALM.rpyR(uni(-3, 3), uni(-1.5, 1.5), uni(-3, 3));
  rot.push({ r: R, log: CALM.logR(R) });
}
const out = {
  source: 'src/api/index.html VIK / CALM (node tests/fixtures/gen_vik_fixture.mjs)',
  seed: 20261109, lo, hi, level_cos: LEVEL, par: VIK.PAR,
  fk, solve, cam_mount: camMount, level, rot,
};
fs.writeFileSync(path.join(here, 'vik_samples.json'), JSON.stringify(out, null, 1) + '\n');
console.log(`wrote ${fk.length} fk, ${solve.length} solve (${solve.filter((s) => s.ok).length} ok), ${camMount.length} camMount, ${level.length} level, ${rot.length} rot`);
