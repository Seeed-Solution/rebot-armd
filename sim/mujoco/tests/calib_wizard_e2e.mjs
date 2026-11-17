#!/usr/bin/env node
// Camera calibration wizard flow against a running armd, without a browser (test_calibration_e2e.py).
//   node calib_wizard_e2e.mjs --armd http://127.0.0.1:PORT --board X,Y,YAW_DEG --out result.json
// The cockpit's steps with the same code: src/api/calib_wizard.js (CW) and the IK / FK blocks of
// src/api/index.html (CALM, VIK; as scripts/view_ik_selftest.mjs loads them). The operator's jog to the
// start pose is a move_to above the board (--board: where the board lies; only used for that pose).
//   start pose (camera 0.28 m above the board centre with armd's current mount) -> ["aruco"] capture ->
//   bootstrap with nominal values (fx 824, GET /api/v1/calibration camera_mount) -> generated poses ->
//   WS move_to at 0.03 m/s / 0.3 rad/s -> stationary 0.3 s -> POST /api/v1/detection/trigger ["aruco"] ->
//   after 8 accepted views re-solve and regenerate -> solveAll (gates) -> WS calibration/apply dry run,
//   then the real apply. Writes { solve, dry, ack, views, timings } to --out; exit 1 when a step fails.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';

const args = Object.fromEntries(process.argv.slice(2).reduce((a, v, i, all) => (v.startsWith('--') ? [...a, [v.slice(2), all[i + 1]]] : a), []));
const BASE = args.armd, OUT = args.out, BOARD_AT = (args.board || '0.30,0.02,-80').split(',').map(Number);
const THICK = Number(args.thickness ?? 0);
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const block = (name) => html.slice(html.indexOf(`/* ${name}-BEGIN */`), html.indexOf(`/* ${name}-END */`));
const { CALM, VIK } = vm.runInContext(block('CALIB-MATH') + '\n' + block('VIEW-IK') + '\n({ CALM, VIK });', vm.createContext({}));
const D = 180 / Math.PI;
const SPEED = { linear: 0.03, angular: 0.3 };
const t0 = Date.now();
const log = (...a) => console.log(`[${((Date.now() - t0) / 1000).toFixed(1)} s]`, ...a);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const getJ = async (p) => { const r = await fetch(BASE + p); if (!r.ok) throw new Error(`${p}: HTTP ${r.status}`); return r.json(); };
const postJ = async (p, b) => { const r = await fetch(BASE + p, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(b) }); return r.json(); };

// ---- joint map and limits from the schema (as the cockpit's applyJointMap / modelLimits6)
const schema = await getJ('/api/v1/observation/schema');
const sign = [1, 1, 1, 1, 1, 1], off = [0, 0, 0, 0, 0, Math.PI / 2];
if (Array.isArray(schema.model_joint_map)) schema.model_joint_map.slice(0, 6).forEach((m, i) => { sign[i] = m.sign; off[i] = -m.sign * m.offset_rad; });
const toModel6 = (pos) => [0, 1, 2, 3, 4, 5].map((i) => ((Number(pos[i]) || 0) - off[i]) / sign[i]);
const lo = [], hi = [];
for (let i = 0; i < 6; i++) {
  const L = (schema.limits || [])[i];
  if (!L) { lo.push(-Infinity); hi.push(Infinity); continue; }
  const a = (L.min_position + 0.02 - off[i]) / sign[i], b = (L.max_position - 0.02 - off[i]) / sign[i];
  lo.push(Math.min(a, b)); hi.push(Math.max(a, b));
}
const ik = (T, seed) => VIK.solve(T, seed, { lo, hi, iters: 60 });

// ---- WebSocket controller: lease, heartbeats, observations, command replies
const ws = new WebSocket(BASE.replace(/^http/, 'ws') + '/api/v1/ws');
let obs = null, generation = null, rid = 0;
const waiters = [];
ws.onmessage = (ev) => {
  const m = JSON.parse(ev.data);
  if (m.type) { for (const w of waiters.slice()) if (w.match(m)) { waiters.splice(waiters.indexOf(w), 1); w.resolve(m); } }
  else if (m.positions) obs = m;
};
const waitMsg = (match, ms = 10000) => new Promise((resolve, reject) => {
  const w = { match, resolve }; waiters.push(w);
  setTimeout(() => { const i = waiters.indexOf(w); if (i >= 0) { waiters.splice(i, 1); reject(new Error('reply timeout')); } }, ms);
});
await new Promise((r, j) => { ws.onopen = r; ws.onerror = () => j(new Error('ws connect')); });
ws.send(JSON.stringify({ type: 'acquire_lease', client_id: 'calib_wizard_e2e' }));
generation = (await waitMsg((m) => m.type === 'lease_granted')).generation;
while (!obs) await sleep(20);
const hb = setInterval(() => { if (obs) ws.send(JSON.stringify({ type: 'heartbeat', generation, epoch_id: obs.epoch_id })); }, 300);
async function command(p, body, ms = 15000) {
  const id = ++rid, epoch = (await getJ('/api/v1/observation')).epoch_id;
  ws.send(JSON.stringify({ type: 'command', request_id: id, generation, epoch_id: epoch, path: p, body }));
  const m = await waitMsg((x) => (x.type === 'command_ack' || x.type === 'command_reject') && x.request_id === id, ms);
  if (m.type !== 'command_ack') throw Object.assign(new Error(`${p}: ${m.code} ${m.message}`), { code: m.code, reply: m });
  return m;
}
async function waitAction(id, ms = 120000) {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    const o = await getJ('/api/v1/observation');
    for (const a of [o.action, o.previous_action]) if (a && a.id === id && a.state !== 'running') return a;
    await sleep(50);
  }
  throw new Error(`action ${id} still running`);
}
// wizard cwMove: move_to the end pose T with the speed caps
// (phase 'reject': refused before any motion, 'action': the started action failed)
let ownAction = null;
async function move(T, speed = SPEED) {
  let ack;
  try { ack = await command('/api/v1/action/move_to', { position: CW.posOf(T), orientation: { quat: CW.quatOf(CW.rotOf(T)) }, speed, tag: 'calib-wizard' }); }
  catch (e) { throw Object.assign(e, { phase: 'reject' }); }
  ownAction = ack.action_id;
  const rec = await waitAction(ack.action_id);
  if (rec.state !== 'done') throw Object.assign(new Error(rec.message || rec.code), { code: rec.code || rec.state, phase: 'action' });
}
// wizard cwSettle: all joint speeds <= 0.02 rad/s for 0.3 s
async function settle() {
  let since = 0;
  const end = Date.now() + 10000;
  while (Date.now() < end) {
    const v = obs && obs.velocities, still = Array.isArray(v) && v.every((x) => Math.abs(x) <= 0.02);
    if (!still) since = 0; else if (!since) since = Date.now(); else if (Date.now() - since >= 300) return;
    await sleep(30);
  }
  throw new Error('arm did not settle');
}
// wizard cwCapture: one ["aruco"] trigger through armd at a stationary pose
async function capture() {
  const pos = obs.positions.slice();
  const data = await postJ('/api/v1/detection/trigger?wait_ms=2000', { pipelines: ['aruco'] });
  if (!data || data.success === false || data.timeout) return { err: 'det', detail: JSON.stringify(data).slice(0, 200) };
  if (pos.slice(0, 6).some((v, i) => Math.abs(obs.positions[i] - v) > 0.003)) return { err: 'moved' };
  const markers = (data.markers || []).filter((m) => m.id >= 0 && m.id < 12 && m.corners && m.corners.length === 4).map((m) => ({ id: m.id, corners: m.corners }));
  const q = toModel6(pos);
  return { markers, G: CALM.fk(q), q, joints: pos, frame_id: data.frame_id };
}

const out = { timings: {}, views: [], skipped: [] };
let code = 0;
try {
  await command('/api/v1/torque', { enable: true });
  const h = await waitAction((await command('/api/v1/motion/home', {})).action_id);
  if (h.state !== 'done') throw new Error('home ' + JSON.stringify(h));
  // nominal mount (cwNominalMount): armd's current camera_mount
  const cal = await getJ('/api/v1/calibration');
  out.before = { calibrated: cal.calibrated, camera_mount: cal.camera_mount, grasp: cal.grasp };
  const Rn = CW.mountRotation(VIK, cal.camera_mount || {}, toModel6([0, 0, 0, 0, 0, Math.PI / 2]));
  const Xn = CW.make(Rn, cal.camera_mount.offset_m);
  // the operator's jog: camera 0.28 m above the board centre looking down, image x along the board's long side
  const [bx, by, byaw] = BOARD_AT, yaw = byaw / D, x = [Math.cos(yaw), Math.sin(yaw), 0], z = [0, 0, -1];
  const y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
  const Tstart = CW.mul(CW.make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [bx, by, 0.28]), CW.inv(Xn));
  await move(Tstart, { linear: 0.08, angular: 0.6 });
  await settle();
  const v0 = await capture();
  if (v0.err) throw new Error('start capture ' + v0.err + ' ' + (v0.detail || ''));
  const board = CW.makeBoard({ markerSide: 0.025 });
  const n0 = CW.boardMarkers(v0, board).length;
  log(`start view: ${n0} markers`);
  if (n0 < CW.GATES.minMarkers) throw new Error(`start view: ${n0} markers`);
  const bs = CW.bootstrapBoard(v0, board, { K: CW.NOMINAL, X: Xn, G: v0.G });
  let est = { Tbb: bs.Tbb, K: { ...CW.NOMINAL }, X: Xn };
  out.bootstrap = { board_pos: CW.posOf(bs.Tbb), markers: n0 };
  const views = [v0], target = 20;
  let genStats = null;
  const gen = (q, n, exclude) => { const g = CW.generatePoses({ Tbb: est.Tbb, board, K: est.K, X: est.X, q0: q, fk: CALM.fk, ik, lo, hi, n, exclude }); genStats = g.stats; return g.poses; };
  let plan = gen(v0.q, target, []), refined = false;
  out.timings.bootstrap_s = (Date.now() - t0) / 1000;
  log(`plan: ${plan.length} poses (generator ${JSON.stringify(genStats)}), cam-board ${Math.min(...plan.map((p) => p.dist))}..${Math.max(...plan.map((p) => p.dist))} m`);
  const accepted = () => views.filter((v) => CW.boardMarkers(v, board).length >= CW.GATES.minMarkers).length;
  // the cockpit's run loop (CW.runPoses): skip only planning refusals, abort on anything else
  const st = { plan, idx: 0 }, epoch0 = obs.epoch_id;
  ownAction = null;
  const res = await CW.runPoses(st, {
    abortReason: () => CW.runAbortReason({ live: true, epoch0, epoch: obs.epoch_id, state: typeof obs.safety_state === 'string' ? obs.safety_state : Object.keys(obs.safety_state || {})[0],
      fault: obs.fault || null, hold: !!(obs.collision && obs.collision.active), action: obs.action ? obs.action.id : null, ownAction }),
    check: (T) => { const q = toModel6(obs.positions); return CW.checkTransition(CALM.fk(q), q, T, { ik, X: est.X, plane: CW.boardPlane(est.Tbb), clearance: 0.10, lo, hi }); },
    move: (T) => move(T),
    settle, capture,
    accept: (v, i, p) => {
      const m = v.err ? 0 : CW.boardMarkers(v, board).length;
      if (v.err || m < CW.GATES.minMarkers) { p.state = 'skip'; out.skipped.push({ i, why: v.err || `markers ${m}` }); log(`#${i + 1} skipped: ${v.err || m + ' markers'}`); return; }
      p.state = 'ok'; views.push(v);
      log(`#${i + 1} ${p.dist} m tilt ${p.tiltDeg}/${p.azDeg} deg: ${m} markers`);
    },
    after: (i) => {
      if (refined || accepted() < 8) return; // cwRefine
      refined = true;
      const r = CW.solveAll({ views, board, CALM, thickness: THICK });
      if (!r.final) return;
      est = { Tbb: r.final.Tbb, K: r.final.K, X: r.final.X };
      const left = Math.max(0, target + 1 - views.length);
      const tail = gen(toModel6(obs.positions), left, st.plan.slice(0, i + 1).map((d) => d.T));
      log(`re-solve after ${views.length} views: fx ${r.final.K.fx.toFixed(1)}, offset ${CW.posOf(r.final.X).map((t) => (t * 1000).toFixed(1))} mm; ${tail.length} new poses (generator ${JSON.stringify(genStats)})`);
      if (tail.length) st.plan = [...st.plan.slice(0, i + 1), ...tail];
    },
    stop: () => command('/api/v1/motion/stop', {}),
    skipWhy: (kind, d) => { log(`skipped: ${kind} ${d}`); return `${kind} ${d}`; },
  }, { runAll: true });
  for (const [i, p] of st.plan.entries()) if (p.state === 'skip' && p.why && !out.skipped.some((x) => x.i === i)) out.skipped.push({ i, why: p.why });
  out.run = { result: res, poses: st.plan.length };
  out.timings.capture_s = (Date.now() - t0) / 1000;
  const solve = CW.solveAll({ views, board, CALM, thickness: THICK, solvedAt: new Date().toISOString() });
  out.views = views.map((v) => ({ markers: v.markers.length, joints: v.joints }));
  out.solve = { ok: solve.ok, gates: solve.gates, apply: solve.apply, joint: solve.joint && { rmsBefore: solve.joint.rmsBefore, rmsAfter: solve.joint.rmsAfter, iterations: solve.joint.iterations },
    separate: solve.joint && { K: solve.joint.separate.K, offset_m: CW.posOf(solve.joint.separate.X), q_end_cam: CW.quatOf(CW.rotOf(solve.joint.separate.X)) },
    intr_K: solve.intr.K, used: solve.intr.used.length };
  log(`solve: ok ${solve.ok}; ` + solve.gates.map((g) => `${g.key}=${g.ok ? 'ok' : 'FAIL'}(${typeof g.value === 'number' ? +g.value.toPrecision(4) : g.value})`).join(' '));
  if (!solve.ok) throw new Error('gates failed');
  const dry = await command('/api/v1/calibration/apply', { ...solve.apply, dry_run: true });
  out.dry = { before: dry.before, after: dry.after };
  const ack = await command('/api/v1/calibration/apply', { ...solve.apply, dry_run: false }, 30000);
  out.ack = ack;
  out.after = await getJ('/api/v1/calibration');
  out.timings.total_s = (Date.now() - t0) / 1000;
  log('applied, backup ' + ack.backup);
} catch (e) {
  out.error = String(e && e.stack || e);
  log('FAILED ' + out.error);
  code = 1;
} finally {
  clearInterval(hb);
  fs.writeFileSync(OUT, JSON.stringify(out, null, 1));
  ws.close();
}
process.exit(code);
