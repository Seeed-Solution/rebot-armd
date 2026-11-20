#!/usr/bin/env node
// Supervised hardware qualification of the T20 self-service verification on a real B601-DM (not run in CI).
//   ARMD=http://<armd host>:<port> node scripts/calib_hover_hwqual.mjs --i-am-at-the-arm [--points 4] [--out DIR]
// What it does, in order, and nothing else (it never writes the calibration):
//   1. reads GET /api/v1/calibration and the safety description; refuses without a camera calibration in effect,
//      without the calibration_verify capability, or when the arm has a fault
//   2. takes control (WS lease, heartbeats), turns torque on when it is off (the arm holds where it is)
//   3. fingertip: up to 40 snapshots (/api/v1/camera/snapshot) decoded by Chromium (canvas, as the cockpit), dots,
//      measureTip with the intrinsics in effect; logs every frame's dot centroids and the gates
//   4. re-observes the board (ArUco trigger at the current pose; >= 4 markers), plans at most --points (<= 4)
//      tool-down hovers 12 mm above board corners with the model in effect as the candidate, logs the plan
//   5. runs the calib_hover job (bounded by armd: <= 15 mm/s descent, 2.5 s settle, 2 frames discarded, 3 kept),
//      polls it, evaluates the recorded corners: per point sticker-edge height, height, XY error, the gates
//   6. parks (motion/unload), waits until torque is off and logs the final torque / safety state
// Ctrl+C sends motion/stop and exits; a lost connection leaves the arm to armd's lease watchdog (park).
// Log: <out>/hwqual_<time>.json (corners, gates, job record, final torque state). Needs Node >= 22 (WebSocket) and
// Playwright (PLAYWRIGHT=/path/to/playwright/index.mjs or a resolvable 'playwright').
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';

const args = process.argv.slice(2);
if (!args.includes('--i-am-at-the-arm')) {
  console.error('Refusing to run: pass --i-am-at-the-arm only when you are next to the arm and can cut its power.');
  process.exit(2);
}
const opt = (k, d) => { const i = args.indexOf(k); return i >= 0 && args[i + 1] ? args[i + 1] : d; };
const MAX_POINTS = Math.min(4, Math.max(1, Number(opt('--points', 4)) || 4));
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const out = opt('--out', path.join(root, '.cache/hwqual'));
fs.mkdirSync(out, { recursive: true });
const base = (process.env.ARMD || 'http://127.0.0.1:8765').replace(/\/$/, '');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const html = fs.readFileSync(path.join(root, 'src/api/index.html'), 'utf8');
const block = (n) => html.slice(html.indexOf(`/* ${n}-BEGIN */`), html.indexOf(`/* ${n}-END */`));
const { CALM } = vm.runInContext(block('CALIB-MATH') + '\n({ CALM });', vm.createContext({}));
const log = { started_at: new Date().toISOString(), armd: base, max_points: MAX_POINTS, steps: [] };
const step = (name, data) => { log.steps.push({ name, at: new Date().toISOString(), ...data }); console.log(`[${name}]`, JSON.stringify(data).slice(0, 400)); };
const save = () => { const f = path.join(out, `hwqual_${log.started_at.replace(/[:.]/g, '-')}.json`); fs.writeFileSync(f, JSON.stringify(log, null, 1)); return f; };
const get = async (p) => { const r = await fetch(base + p, { cache: 'no-store' }); if (!r.ok) throw new Error(`GET ${p}: ${r.status}`); return r.json(); };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---- 1. state
const cal = await get('/api/v1/calibration');
const schema = await get('/api/v1/observation/schema').catch(() => ({}));
const safety = await get('/api/v1/safety/describe').catch(() => null);
const caps = schema.capabilities || {};
step('state', { calibrated: cal.calibrated, verification: cal.verification, floor: cal.floor, tool: cal.calibration_tool, caps_verify: caps.calibration_verify,
  physical_estop: safety && safety.hardware ? safety.hardware.physical_estop : null });
if (!cal.calibrated || !cal.calibration || !cal.calibration.intrinsics) { step('refused', { why: 'no camera calibration in effect' }); console.log(save()); process.exit(1); }
if (!caps.calibration_verify) { step('refused', { why: 'armd without calibration_verify' }); console.log(save()); process.exit(1); }
const I = cal.calibration.intrinsics, K = { fx: I.fx, fy: I.fy, cx: I.cx, cy: I.cy, k1: I.k1, k2: I.k2 };
const X = CW.make(CW.quatR(cal.camera_mount.q_end_cam), cal.camera_mount.offset_m);
const ch = CW.kinChain(CALM.CHAIN), kLive = cal.kinematics_correction ? CW.kinVector(cal.kinematics_correction) : new Array(42).fill(0);
const edge = cal.calibration_tool ? cal.calibration_tool.edge_to_low_m ?? null : null;
// armd joint positions -> model angles (schema model_joint_map: model = sign * motor + offset_rad), as the cockpit's toModel6
const jm = Array.isArray(schema.model_joint_map) ? schema.model_joint_map : null;
const toModel = (pos) => [0, 1, 2, 3, 4, 5].map((i) => (jm && jm[i] ? jm[i].sign * pos[i] + jm[i].offset_rad : pos[i]));

// ---- 2. control
const ws = new WebSocket(base.replace(/^http/, 'ws') + '/api/v1/ws');
let obs = null, gen = null, epoch = null, req = 1;
const pending = new Map();
ws.onmessage = (m) => {
  const v = JSON.parse(m.data);
  if (v.positions) { obs = v; epoch = v.epoch_id; return; }
  if (v.type === 'lease_granted') gen = v.generation;
  if ((v.type === 'command_ack' || v.type === 'command_reject') && pending.has(v.request_id)) { pending.get(v.request_id)(v); pending.delete(v.request_id); }
};
await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; });
ws.send(JSON.stringify({ type: 'acquire_lease', client_id: 'calib_hover_hwqual' }));
for (let i = 0; i < 50 && (gen == null || !obs); i++) await sleep(100);
if (gen == null) { step('refused', { why: 'no lease (another client holds control)' }); console.log(save()); process.exit(1); }
const hb = setInterval(() => ws.send(JSON.stringify({ type: 'heartbeat', generation: gen, epoch_id: epoch })), 300);
const cmd = (p, body, ms = 15000) => new Promise((res, rej) => {
  const id = req++;
  const tm = setTimeout(() => { pending.delete(id); rej(new Error(`${p}: timeout`)); }, ms);
  pending.set(id, (v) => { clearTimeout(tm); v.type === 'command_ack' ? res(v) : rej(Object.assign(new Error(`${p}: ${v.code} ${v.message}`), { code: v.code })); });
  ws.send(JSON.stringify({ type: 'command', request_id: id, generation: gen, epoch_id: epoch, path: p, body }));
});
let stopping = false;
process.on('SIGINT', async () => { if (stopping) process.exit(130); stopping = true; console.log('Ctrl+C: motion/stop'); try { await cmd('/api/v1/motion/stop', {}, 2000); } catch (_) {} step('aborted', { by: 'SIGINT' }); console.log(save()); process.exit(130); });
const end = async (code) => {
  try {
    const ack = await cmd('/api/v1/motion/unload', {}, 5000);
    for (let i = 0; i < 600 && obs && obs.is_torqued; i++) await sleep(100);
    step('park', { action_id: ack.action_id, is_torqued: obs && obs.is_torqued, safety_state: obs && obs.safety_state, fault: obs && obs.fault });
  } catch (e) { step('park_failed', { error: e.message, is_torqued: obs && obs.is_torqued, fault: obs && obs.fault }); }
  clearInterval(hb); ws.close(); console.log(save()); process.exit(code);
};
if (obs.fault) { step('refused', { why: 'fault', fault: obs.fault }); await end(1); }
if (!obs.is_torqued) { await cmd('/api/v1/torque', { enable: true }); await sleep(1500); }

// ---- 3. fingertip through Chromium
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const browser = await chromium.launch(process.env.CHROME ? { executablePath: process.env.CHROME } : {});
const page = await browser.newPage();
await page.setContent('<html><body></body></html>');
const frames = [], seen = new Set(), q0pos = obs.positions.slice();
for (let i = 0; i < 40 && frames.length < 16; i++) {
  const r = await fetch(base + '/api/v1/camera/snapshot', { cache: 'no-store' });
  const buf = Buffer.from(await r.arrayBuffer());
  const key = buf.length + ':' + buf.subarray(buf.length - 64).toString('hex');
  if (seen.has(key)) { await sleep(60); continue; }
  seen.add(key);
  const [w, h, data] = await page.evaluate(async (b64) => {
    const img = new Image(); img.src = 'data:image/jpeg;base64,' + b64; await img.decode();
    const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
    const g = c.getContext('2d', { willReadFrequently: true }); g.drawImage(img, 0, 0);
    const d = g.getImageData(0, 0, c.width, c.height).data; let s = '';
    for (let k = 0; k < d.length; k += 0x8000) s += String.fromCharCode.apply(null, d.subarray(k, k + 0x8000));
    return [c.width, c.height, btoa(s)];
  }, buf.toString('base64'));
  const d = CW.detectStickerDots(CW.grayContent(new Uint8Array(Buffer.from(data, 'base64')), w, h));
  const moved = obs.positions.slice(0, 6).some((v, k) => Math.abs(v - q0pos[k]) > 0.003);
  step('tip_frame', { i, size: [w, h], ok: d.ok, points: d.points, blobs: d.blobs.length, threshold: d.threshold, moved });
  if (moved) { step('refused', { why: 'arm moved during the fingertip frames' }); await browser.close(); await end(1); }
  if (d.ok) frames.push(d.points);
}
await browser.close();
const tip = CW.measureTip(frames, K, { gates: { ...CW.STICKER_GATES, minFrames: 8 } });
step('tip', { ok: tip.ok, tip_cam_m: tip.tip_cam_m, reproj_px: tip.reprojPx, gap_px: tip.gapPx, std_mm: tip.stdMm, gates: tip.gates, poses: tip.poses });
if (!tip.ok) await end(1);

// ---- 4. board and plan
const trig = async () => {
  const r = await fetch(base + '/api/v1/detection/trigger', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ pipelines: ['aruco'], wait_ms: 2500 }) });
  return r.json();
};
await trig(); await trig(); // the first frames after a move lag
const tr = await trig();
const board = CW.makeBoard({ markerSide: I.marker_size || 0.025 });
const markers = (tr.markers || []).filter((m) => m.id >= 0 && m.id < 12 && m.corners && m.corners.length === 4);
const fit = CW.fitBoardPose(markers, board, K);
const qm = toModel(obs.positions);
step('board', { frame_id: tr.frame_id, markers: markers.map((m) => ({ id: m.id, corners: m.corners })), reproj_px: fit && fit.reprojPx });
if (!fit || markers.length < 4) { step('refused', { why: `board: ${markers.length} markers, need 4` }); await end(1); }
const Tbb = CW.mul(CW.mul(CW.kinPose(ch, qm, kLive), X), fit.T);
const plan = CW.planHover({ Tbb, X, tipCam: tip.tip_cam_m, edgeToLow: edge, ch, kCand: kLive, kLive, q0: qm, K, board, plan: { ...CW.HOVER_PLAN, count: MAX_POINTS, minCount: Math.min(4, MAX_POINTS) } });
step('plan', { ok: plan.ok, why: plan.why, candidates: plan.candidates, points: plan.points.map((p) => ({ id: p.id, corner: p.corner, approach_m: p.approach_m, hover_m: p.hover_m, rpy_rad: p.rpy_rad, yaw_deg: p.yaw_deg, visible: p.visible })) });
if (!plan.points.length) await end(1);

// ---- 5. hovers on the device
const fp = cal.verification ? cal.verification.fingerprint : 'hwqual';
const ack = await cmd('/api/v1/action/calib_hover', { candidate_fingerprint: fp, rpy_rad: plan.points[0].rpy_rad, approach_speed_mps: 0.03, descent_speed_mps: 0.01, settle_s: 2.5, frames: 3, discard: 2,
  points: plan.points.map((p) => ({ id: p.id, approach_m: p.approach_m, hover_m: p.hover_m, rpy_rad: p.rpy_rad })) });
step('hover_started', { job_id: ack.job_id, action_id: ack.action_id, bound_s: ack.expected_duration_s });
let job = null;
for (let i = 0; i < 2400; i++) {
  await sleep(500);
  job = await get('/api/v1/calibration/hover/' + ack.job_id);
  if (job.state !== 'running') break;
}
step('hover_job', { job });
const pts = job.points.map((p) => {
  const pl = plan.points.find((x) => x.id === p.id);
  if (p.state !== 'done') return { id: p.id, ok: false, state: p.state, code: p.code };
  const q = toModel(p.settled_joints);
  return { id: p.id, ...CW.hoverPoint({ observations: p.observations, board, K, tipCam: tip.tip_cam_m, edgeToLow: edge, target: pl.corner, Tend: CW.kinPose(ch, q, kLive), X }) };
});
const ev = CW.evaluateHovers(pts, { edgeToLow: edge });
step('hover_eval', { points: pts.map((p) => ({ id: p.id, ok: p.ok, edge_mm: p.edge_height_m * 1000, height_mm: p.height_m == null ? null : p.height_m * 1000, xy_mm: p.xy_err_m && p.xy_err_m.map((v) => v * 1000), frames: p.frames, reproj_px: p.reproj_px })),
  gates: ev.gates, table_z_m: ev.table_z_m, optical_table_z_m: ev.optical_table_z_m, ok: ev.ok });

// ---- 6. park, final torque state
await end(ev.ok ? 0 : 1);
