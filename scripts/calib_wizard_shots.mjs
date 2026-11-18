#!/usr/bin/env node
// Headless walk through the camera calibration wizard (no armd needed) with screenshots of every step.
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_wizard_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and a mock of GET /api/v1/calibration;
// the control WebSocket is a stub. In the page an arm and camera are simulated: move_to (commandRPC) puts
// the arm at the IK solution of the target, the ArUco trigger (detTriggerReq) projects the board through
// a camera with fx 824 / fy 821 / cx 326 / cy 176 / k1 -0.1, 0.3 px noise, mounted 38 deg toward the jaws.
// The run goes board -> place (bootstrap with nominal values) -> capture (run all) -> solve (gates) ->
// review (dry-run reply) -> applied -> restore, in zh / en at desktop 1400x900 and phone 390x844, and writes
// PNGs to .cache/calib-wizard-shots/. Also: Stop aborts a run, and armd without calibration_apply shows the
// update message. Fine calibration mode (camera + arm geometry), same viewports and languages: the simulated arm's
// geometry differs from the URDF (zero offsets, a joint-axis tilt, an origin shift); joint-space capture of
// placement A with an abort and Continue, the board move, placement B, the solve in a Web Worker (gates, held-out,
// per-joint parameters, the J5 zero warning), review with park / torque off, one apply carrying kinematics.
// FINE_ONLY=1 runs only the fine calibration walk-through. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-wizard-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(js); return; }
  if (u === '/api/v1/calibration') return send(res, 200, { calibrated: false, busy: false, camera_mount: { flip: false, rot_deg: 0, tilt_deg: -38, offset_m: null, q_end_cam: null }, backups: [] });
  if (u === '/api/v1/vision/mode') return send(res, 200, { api_version: 2, mode: 'aruco', modes: ['off', 'yolo', 'aruco', 'both'], enabled: true, per_request_pipelines: true, subscriptions: true, available: { yolo: true, aruco: true } });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
let bad = 0;
const shots = [];
const fail = (m) => { bad++; console.log('  FAIL ' + m); };
// waitForFunction that reports the wizard state on a timeout
async function waitW(page, fn, ms, label) {
  try { await page.waitForFunction(fn, null, { timeout: ms }); }
  catch (e) {
    const st = await page.evaluate(() => ({ calls: (window.__SIM ? __SIM.calls : []).map((c) => c[0]).slice(-6), stub: String(commandRPC).slice(0, 30), step: CWZ.step, idx: CWZ.idx, running: CWZ.running, busy: CWZ.busy, status: CWZ.status, n: CWZ.plan.length }));
    throw new Error(`${label}: ${e.message.split('\n')[0]} state ${JSON.stringify(st)}`);
  }
}

// in-page simulation (runs in the browser)
function setupSim() {
  const D = 180 / Math.PI;
  const W = window.__SIM = { calls: [], seed: 7 };
  const rnd = () => { W.seed = (W.seed * 1103515245 + 12345) % 2147483648; return W.seed / 2147483648; };
  const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
  return cwLoad().then((CW) => {
    W.K = { fx: 824, fy: 821, cx: 326, cy: 176, k1: -0.1, k2: 0 };
    W.X = CW.make(CW.mul(CW.mul(VIK.camMount(false, 0), CW.rodrigues([-38 / D, 0, 0])), CW.rodrigues([0, 0, 1.5 / D])), [-0.085, 0.004, 0.052]);
    const yaw = -80 / D;
    W.Tbb = CW.make(CW.mul(CW.rpyR(0, 0, yaw), CW.rpyR(Math.PI, 0, 0)), [0.30, 0.02, 0.0005]); // board z into the table
    const x = [Math.cos(yaw), Math.sin(yaw), 0], z = [0, 0, -1], y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
    const Tc = CW.make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [0.30, 0.02, 0.30]);
    const T0 = CW.mul(Tc, CW.inv(W.X));
    let q0 = null;
    for (let k = 0; k < 400 && !q0; k++) { const s = [0, 1, 2, 3, 4, 5].map(() => (rnd() - 0.5) * 3); const r = VIK.solve(T0, s, { iters: 300 }); if (r.ok) q0 = r.q; }
    // simulated arm state
    S.dof = 7; S.armDof = 6; S.profileId = null; S.state = 'Armed'; S.caps = { ...(S.caps || {}), calibration_apply: true };
    S.leaseGeneration = 1; S.epoch = 1;
    S.obs = { positions: [...toMotor6(q0), 0], velocities: [0, 0, 0, 0, 0, 0, 0], is_torqued: true, safety_state: 'Armed', epoch_id: 1, fault: null, action: null, previous_action: null };
    clearInterval(W.tick); W.tick = setInterval(() => { S.obsAt = Date.now(); }, 50);
    let aid = 0;
    window.commandRPC = async (p, body) => {
      W.calls.push([p, body]);
      if (p === '/api/v1/action/move_to') {
        const id = ++aid, T = CW.make(CW.quatR(body.orientation.quat), body.position);
        const s = cwIk(T, cwModelQ(S.obs.positions));
        // as armd: an IK failure is a planning refusal (command_reject), nothing moves
        if (!s.ok) throw Object.assign(new Error('IK unreachable'), { code: 'IK_UNREACHABLE' });
        S.obs.action = { id, kind: 'move_to', state: 'running' };
        setTimeout(() => {
          S.obs.positions = [...toMotor6(s.q), 0];
          S.obs.action = { id, kind: 'move_to', state: 'done', code: null };
        }, 60);
        return { action_id: id };
      }
      if (p === '/api/v1/motion/stop') return {};
      if (p === '/api/v1/calibration/apply') {
        const before = { vision_intrinsics: { fx: 824, fy: 824, cx: 320, cy: 180, k1: 0, k2: 0, p1: 0, p2: 0, k3: 0, width: 640, height: 360, revision: 3 },
          camera_mount: { flip: false, rot_deg: 0, tilt_deg: -38, offset_m: null, q_end_cam: null },
          grasp: { fx_px: 824, fy_px: 824, cx_px: null, cy_px: null, image_size_px: [640, 360], table_z_m: 0, camera_tilt_deg: -38, grip_pixel_norm: [0.5, 0.82] } };
        const i = body.intrinsics, m = body.camera_mount;
        const tilt = CW.rpyDeg(CW.quatR(m.q_end_cam))[0];
        const after = { vision_intrinsics: { ...before.vision_intrinsics, ...i }, camera_mount: { ...before.camera_mount, offset_m: m.offset_m, q_end_cam: m.q_end_cam, tilt_deg: tilt },
          grasp: { ...before.grasp, fx_px: i.fx, fy_px: i.fy, cx_px: i.cx, cy_px: i.cy, table_z_m: body.table_z_m, camera_tilt_deg: tilt } };
        // v8-like: the grip pixel of the new model is below the image, grip_pixel_norm stays
        if (body.dry_run) return { dry_run: true, before, after: { ...after, grip_pixel_model: [0.507, 1.184] } };
        return { backup: '/userdata/rebot-teleop/calibration-backups/20261117T101500Z', ...after, vision_intrinsics: { ...after.vision_intrinsics, revision: 4 } };
      }
      if (p === '/api/v1/calibration/restore') return { backup: '/userdata/rebot-teleop/calibration-backups/20261117T101620Z', restored: body.backup, vision_intrinsics: { fx: 824, fy: 824, cx: 320, cy: 180, k1: 0, k2: 0, revision: 5 } };
      return {};
    };
    window.detTriggerReq = async () => {
      await new Promise((r) => setTimeout(r, 30));
      const G = calGripper(S.obs.positions), Tcb = CW.mul(CW.inv(CW.mul(G, W.X)), W.Tbb), R = CW.rotOf(Tcb), t = CW.posOf(Tcb), markers = [];
      const board = CW.makeBoard({ markerSide: 0.025 });
      for (const m of board.markers) {
        const px = m.corners.map((c) => CW.project(W.K, R, t, c));
        if (px.every(([u, v, zz]) => zz > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)) markers.push({ id: m.id, corners: px.map(([u, v]) => [u + 0.3 * gauss(), v + 0.3 * gauss()]) });
      }
      return { api_version: 2, success: true, frame_id: ++aid + 100, width: 640, height: 360, markers, detections: [] };
    };
    // live overlay markers for the place step
    S.detections = Array.from({ length: 8 }, (_, i) => ({ aruco: { id: i, t: [0, 0, 0.3] } })); S.detLastReceivedAt = performance.now(); S.detHoldUntil = performance.now() + 1e7;
    return !!q0;
  });
}

for (const [vpName, viewport] of (process.env.FINE_ONLY ? [] : [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }]])) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'phone' ? 2 : 1 });
    await ctx.addInitScript(([l]) => {
      try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {}
      // control WebSocket stub: stays open, never answers
      const RealWS = window.WebSocket;
      window.WebSocket = function (u, p) {
        if (!String(u).includes('/api/v1/ws')) return new RealWS(u, p);
        const f = { readyState: 1, url: u, send() {}, close() {}, addEventListener() {} };
        return f;
      };
      window.WebSocket.OPEN = 1;
    }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(e.message));
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(500);
    const ok0 = await page.evaluate(setupSim);
    if (!ok0) fail('sim start pose');
    const shot = async (step, extra = '') => {
      await page.waitForTimeout(250);
      const info = await page.evaluate(() => ({
        open: !$('cwiz').hidden, on: [...document.querySelectorAll('#cwiz-steps li.on')].map((x) => x.textContent).join(),
        raw: /\bcw\.[a-z]/.test($('cwiz-panel').innerText), hscroll: $('cwiz-panel').scrollWidth > $('cwiz-panel').clientWidth + 1,
        status: CWZ.status,
      }));
      if (!info.open || info.raw || info.hscroll) fail(`${step} ${lang}/${vpName}: ${JSON.stringify(info)}`);
      const file = path.join(out, `cwiz_${step}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(`${path.relative(root, file)}  step="${info.on}" ${extra}`);
    };
    // 1 board
    await page.evaluate(() => cwOpen(true));
    await page.waitForTimeout(600);
    const hasSvg = await page.evaluate(() => !!document.querySelector('#cwiz-body .cwiz-board svg') && !!$('cwiz-next'));
    if (!hasSvg) fail('board svg / next');
    await shot('1_board');
    await page.fill('#cwiz-side', '24.8');
    await page.click('#cwiz-next');
    // 2 place
    await shot('2_place');
    await page.click('#cwiz-boot');
    await waitW(page, () => CWZ.step === 'capture' || (!CWZ.busy && CWZ.status), 15000, "() => CWZ.step === 'capture' || (!CWZ.busy && CWZ.status)");
    const plan = await page.evaluate(() => ({ step: CWZ.step, n: CWZ.plan.length, status: CWZ.status }));
    if (plan.step !== 'capture' || plan.n < 15) fail('bootstrap ' + JSON.stringify(plan));
    // 3 capture: one confirmed pose, then run all
    await page.click('#cwiz-go');
    await waitW(page, () => !CWZ.running && CWZ.idx === 1, 20000, '() => !CWZ.running && CWZ.idx === 1');
    await page.check('#cwiz-runall');
    await page.click('#cwiz-go');
    await waitW(page, () => !CWZ.running, 120000, '() => !CWZ.running');
    const run = await page.evaluate(() => ({ idx: CWZ.idx, n: CWZ.plan.length, acc: cwAccepted(), skip: CWZ.plan.filter((p) => p.state === 'skip').length, refined: CWZ.refined,
      moves: __SIM.calls.filter(([p]) => p === '/api/v1/action/move_to').map(([, b]) => b.speed), status: CWZ.status }));
    const speedsOk = run.moves.length > 0 && run.moves.every((s) => s.linear === 0.03 && s.angular === 0.3);
    if (run.idx !== run.n || run.acc < 15 || !run.refined || !speedsOk) fail('run ' + JSON.stringify({ ...run, moves: run.moves.length }));
    const lbl = await page.evaluate(() => document.querySelector('#cwiz-body .calib-count').textContent);
    if (!/^\d+ (views \(min 15\)|个视图（至少 15 个）)/.test(lbl)) fail('progress label ' + lbl);
    await shot('3_capture', `label "${lbl}", poses ${run.n}, accepted ${run.acc} (incl. start view), skipped ${run.skip}, refined ${run.refined}, move_to speed caps ok ${speedsOk}`);
    // 4 solve
    await page.click('#cwiz-solve');
    const sol = await page.evaluate(() => { const r = CWZ.solve, K = r.intr.K; return { ok: r.ok, fails: r.gates.filter((g) => !g.ok).map((g) => g.key), K: [K.fx, K.fy, K.cx, K.cy, K.k1].map((v) => +v.toFixed(3)), off: r.apply && r.apply.camera_mount.offset_m.map((v) => +(v * 1000).toFixed(2)), z: r.apply && +(r.apply.table_z_m * 1000).toFixed(2), rows: document.querySelectorAll('#cwiz-body .cwiz-table tr').length }; });
    if (!sol.ok || sol.rows < 13) fail('solve ' + JSON.stringify(sol));
    await shot('4_solve', `gates ok ${sol.ok} K ${sol.K} offset ${sol.off} mm table ${sol.z} mm`);
    // 5 review (dry run)
    await page.click('#cwiz-review');
    await waitW(page, () => !!CWZ.dry && !CWZ.busy, 10000, '() => !!CWZ.dry && !CWZ.busy');
    const rv = await page.evaluate(() => ({ grip: [...document.querySelectorAll('#cwiz-body .cwiz-table td.note')].map((x) => x.textContent).join(), rows: document.querySelectorAll('#cwiz-body .cwiz-table tr').length, chg: document.querySelectorAll('#cwiz-body .cwiz-table tr.chg').length, dry: __SIM.calls.filter(([p, b]) => p === '/api/v1/calibration/apply' && b.dry_run === true).length }));
    if (rv.rows < 10 || rv.chg < 5 || rv.dry !== 1 || !/camera-down|相机朝下/.test(rv.grip)) fail('review ' + JSON.stringify(rv));
    await shot('5_review', `diff rows ${rv.rows - 1}, changed ${rv.chg}, grip row "${rv.grip}"`);
    // 6 apply (two taps), applied, restore (two taps)
    await page.click('#cwiz-apply');
    const armed = await page.evaluate(() => $('cwiz-apply').textContent);
    await page.click('#cwiz-apply');
    await waitW(page, () => CWZ.step === 'applied' && !CWZ.busy, 10000, "() => CWZ.step === 'applied' && !CWZ.busy");
    const body = await page.evaluate(() => { const b = __SIM.calls.filter(([p, x]) => p === '/api/v1/calibration/apply' && x.dry_run === false)[0][1]; return { keys: Object.keys(b).sort(), intr: Object.keys(b.intrinsics).sort(), mount: Object.keys(b.camera_mount).sort(), w: b.intrinsics.width, h: b.intrinsics.height, ms: b.intrinsics.marker_size }; });
    if (body.w !== 640 || body.h !== 360 || Math.abs(body.ms - 0.0248) > 1e-9 || body.mount.join() !== 'offset_m,q_end_cam') fail('apply body ' + JSON.stringify(body));
    await shot('6_applied', `armed label "${armed}", apply body keys ${body.keys} intrinsics ${body.intr}`);
    await page.click('#cwiz-restore'); await page.click('#cwiz-restore');
    await waitW(page, () => CWZ.applied && CWZ.applied.restoredFrom, 10000, '() => CWZ.applied && CWZ.applied.restoredFrom');
    const rs = await page.evaluate(() => __SIM.calls.filter(([p]) => p === '/api/v1/calibration/restore').map(([, b]) => b));
    if (rs.length !== 1 || rs[0].backup !== '20261117T101500Z') fail('restore ' + JSON.stringify(rs));
    await shot('7_restored', `restore body ${JSON.stringify(rs[0])}`);
    // Stop aborts a run: new plan, run all, stop after the first move
    const ab = await page.evaluate(async () => {
      CWZ.step = 'capture'; CWZ.plan = CWZ.plan.map((p) => ({ ...p, state: 'pending' })); CWZ.idx = 0; CWZ.runAll = true; CWZ.refined = true; cwRender();
      __SIM.calls.length = 0;
      const run = cwRun();
      await new Promise((r) => setTimeout(r, 150));
      cwStop();
      await run;
      return { idx: CWZ.idx, running: CWZ.running, status: CWZ.status, stops: __SIM.calls.filter(([p]) => p === '/api/v1/motion/stop').length, moves: __SIM.calls.filter(([p]) => p === '/api/v1/action/move_to').length };
    });
    if (ab.running || ab.idx > 1 || !ab.stops || !/中止|Aborted/.test(ab.status)) fail('stop ' + JSON.stringify(ab));
    console.log(`  stop (${lang}/${vpName}): ${JSON.stringify(ab)}`);
    // lease loss aborts too
    const ll = await page.evaluate(async () => {
      CWZ.idx = 0; CWZ.plan = CWZ.plan.map((p) => ({ ...p, state: 'pending' })); cwRender();
      const run = cwRun();
      await new Promise((r) => setTimeout(r, 120));
      S.leaseGeneration = 2; // another generation: this run's lease is gone
      await run;
      const st = CWZ.status; S.leaseGeneration = 1;
      return { idx: CWZ.idx, running: CWZ.running, status: st };
    });
    if (ll.running || !/控制权|lease/.test(ll.status)) fail('lease ' + JSON.stringify(ll));
    console.log(`  lease loss (${lang}/${vpName}): ${JSON.stringify(ll)}`);
    // step bar: jump back to an earlier step; going back to Place discards the views only after a second click
    await page.evaluate(() => { CWZ.step = 'solve'; CWZ.status = ''; cwRender(); });
    const navs = await page.evaluate(() => [...document.querySelectorAll('#cwiz-steps li.nav')].map((x) => x.dataset.step).join());
    if (navs !== 'board,place,capture') fail('step bar nav from solve: ' + navs);
    await page.click('#cwiz-steps li[data-step="capture"]');
    const back = await page.evaluate(() => ({ step: CWZ.step, views: CWZ.views.length }));
    if (back.step !== 'capture' || !back.views) fail('step bar -> capture ' + JSON.stringify(back));
    await page.click('#cwiz-restart');
    const r1 = await page.evaluate(() => ({ step: CWZ.step, views: CWZ.views.length, label: $('cwiz-restart').textContent, status: CWZ.status }));
    if (r1.step !== 'capture' || !r1.views || !/再点|again/.test(r1.status)) fail('restart first click ' + JSON.stringify(r1));
    await shot('8_restart_confirm', `first click: "${r1.label}" · ${r1.status}`);
    await page.click('#cwiz-restart');
    const r2 = await page.evaluate(() => ({ step: CWZ.step, views: CWZ.views.length, plan: CWZ.plan.length, solve: !!CWZ.solve, navs: [...document.querySelectorAll('#cwiz-steps li.nav')].map((x) => x.dataset.step).join() }));
    if (r2.step !== 'place' || r2.views || r2.plan || r2.solve || r2.navs !== 'board') fail('restart second click ' + JSON.stringify(r2));
    console.log(`  restart (${lang}/${vpName}): ${JSON.stringify(r1.label)} -> ${JSON.stringify(r2)}`);
    // capability missing: update message
    await page.evaluate(() => { S.caps = {}; CWZ.step = 'board'; cwRender(); });
    const un = await page.evaluate(() => $('cwiz-body').innerText);
    if (!/calibration_apply/.test(un)) fail('unsupported text');
    await shot('0_unsupported');
    // the old drawer stays reachable
    await page.click('#cwiz-adv');
    await page.waitForTimeout(200);
    const adv = await page.evaluate(() => !$('calib').hidden && $('cwiz').hidden);
    if (!adv) fail('advanced drawer');
    if (errs.length) { fail(`page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`); }
    await ctx.close();
  }
}
// ---- fine calibration mode
function setupFine() {
  const W = __SIM, CW = CWZ.CW, D = 180 / Math.PI;
  CWF.n = 18; CWF.dwellMs = 60; CWF.settleMs = 150; // shorter waits for the walk-through
  // the simulated arm: real geometry = URDF + this correction (J5 zero -6 deg: the review shows the warning)
  W.kc = CALM.normCorrection([{}, { offset_rad: 0.8 / D, tilt_rad: [0, 0.6 / D] }, { offset_rad: -0.5 / D, tilt_rad: [1.0 / D, 0] }, { origin_m: [0.008, 0, 0] }, { offset_rad: -6 / D }, {}]);
  W.TbbB = CW.make(CW.mul(CW.rpyR(0, 0, -105 / D), CW.rpyR(Math.PI, 0, 0)), [0.27, -0.12, 0.0005]);
  W.armedOnce = true;
  const camAbove = (Tbb) => {
    const c = CW.posOf(Tbb), Rb = CW.rotOf(Tbb), x = [Rb[0][0], Rb[1][0], 0], z = [0, 0, -1], y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
    const T0 = CW.mul(CW.make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [c[0], c[1], 0.30]), CW.inv(W.X));
    // seeded from the arm's current pose first (same IK branch, as a user jogging there)
    const q0 = cwModelQ(S.obs.positions);
    for (let k = 0; k < 400; k++) { const s = k ? q0.map((v, i) => v + Math.sin(k * 7.3 + i * 1.7) * 0.4 * Math.min(1, k / 50)) : q0; const r = VIK.solve(T0, s, { iters: 300 }); if (r.ok) return r.q; }
    return null;
  };
  // the user moves the board and jogs the camera above it
  W.moveBoard = () => { W.Tbb = W.TbbB; const q = camAbove(W.TbbB); if (q) S.obs.positions = [...toMotor6(q), 0]; return !!q; };
  window.detTriggerReq = async () => {
    await new Promise((r) => setTimeout(r, 20));
    const G = CALM.fk(cwModelQ(S.obs.positions), W.kc), Tcb = CW.mul(CW.inv(CW.mul(G, W.X)), W.Tbb), R = CW.rotOf(Tcb), t = CW.posOf(Tcb), markers = [];
    const rnd = () => { W.seed = (W.seed * 1103515245 + 12345) % 2147483648; return W.seed / 2147483648; };
    const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
    for (const m of CW.makeBoard({ markerSide: 0.025 }).markers) {
      const px = m.corners.map((c) => CW.project(W.K, R, t, c));
      if (px.every(([u, v, zz]) => zz > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)) markers.push({ id: m.id, corners: px.map(([u, v]) => [u + 0.3 * gauss(), v + 0.3 * gauss()]) });
    }
    return { api_version: 2, success: true, frame_id: Date.now(), width: 640, height: 360, markers, detections: [] };
  };
  const base = window.commandRPC;
  let aid = 1000;
  window.commandRPC = async (p, body) => {
    if (p === '/api/v1/motion/jog' || p === '/api/v1/motion/unload') {
      W.calls.push([p, body]);
      const id = ++aid;
      S.obs.action = { id, kind: p.endsWith('jog') ? 'jog' : 'unload', state: 'running' };
      setTimeout(() => {
        if (p.endsWith('jog')) S.obs.positions = body.target_positions.slice();
        else { S.obs.is_torqued = false; S.state = 'Disabled'; }
        S.obs.action = { id, kind: 'jog', state: 'done', code: null };
      }, 60);
      return { action_id: id };
    }
    if (p === '/api/v1/calibration/apply' && !body.dry_run && W.armedOnce) { W.calls.push([p, body]); W.armedOnce = false; throw Object.assign(new Error('torque on'), { code: 'KINEMATICS_ARMED' }); }
    const r = await base(p, body);
    if (p === '/api/v1/calibration/apply') {
      if (body.dry_run) return { ...r, before: { ...r.before, kinematics: null }, after: { ...r.after, kinematics: body.kinematics } };
      return { ...r, kinematics: body.kinematics };
    }
    return r;
  };
  return true;
}
for (const [vpName, viewport] of [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }]]) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'phone' ? 2 : 1 });
    await ctx.addInitScript(([l]) => {
      try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {}
      const RealWS = window.WebSocket;
      window.WebSocket = function (u, p) { if (!String(u).includes('/api/v1/ws')) return new RealWS(u, p); return { readyState: 1, url: u, send() {}, close() {}, addEventListener() {} }; };
      window.WebSocket.OPEN = 1;
    }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(e.message));
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(500);
    if (!(await page.evaluate(setupSim)) || !(await page.evaluate(setupFine))) fail('fine sim start');
    const shot = async (step, extra = '') => {
      await page.waitForTimeout(250);
      const info = await page.evaluate(() => ({ open: !$('cwiz').hidden, on: [...document.querySelectorAll('#cwiz-steps li.on')].map((x) => x.textContent).join(),
        raw: /\bcw\.[a-z]/.test($('cwiz-panel').innerText), hscroll: $('cwiz-panel').scrollWidth > $('cwiz-panel').clientWidth + 1 }));
      if (!info.open || info.raw || info.hscroll) fail(`fine ${step} ${lang}/${vpName}: ${JSON.stringify(info)}`);
      const file = path.join(out, `cwiz_fine_${step}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(`${path.relative(root, file)}  step="${info.on}" ${extra}`);
    };
    const jogs = () => page.evaluate(() => __SIM.calls.filter(([p]) => p === '/api/v1/motion/jog').map(([, b]) => b));
    // board: pick the mode
    await page.evaluate(() => cwOpen(true));
    await page.waitForTimeout(500);
    await page.click('#cwiz-mode-fine');
    const md = await page.evaluate(() => ({ mode: CWZ.mode, steps: document.querySelectorAll('#cwiz-steps li').length, pressed: $('cwiz-mode-fine').getAttribute('aria-pressed') }));
    if (md.mode !== 'fine' || md.steps !== 8 || md.pressed !== 'true') fail('fine mode select ' + JSON.stringify(md));
    await shot('1_board', `mode ${md.mode}, ${md.steps} steps`);
    await page.click('#cwiz-next');
    await shot('2_placeA');
    await page.click('#cwiz-boot');
    await waitW(page, () => CWZ.step === 'capture' || (!CWZ.busy && CWZ.status), 20000, 'fine boot A');
    const pa = await page.evaluate(() => ({ step: CWZ.step, n: CWZ.plan.length, dmax: Math.max(...CWZ.plan.map((p) => p.dist)) }));
    if (pa.step !== 'capture' || pa.n < 15) fail('fine plan A ' + JSON.stringify(pa));
    // capture A: run all, Stop after 3 poses, Continue
    await page.check('#cwiz-runall');
    await page.evaluate(() => { __SIM.calls.length = 0; });
    await page.click('#cwiz-go');
    await waitW(page, () => CWZ.idx >= 3, 60000, 'fine 3 poses');
    await page.evaluate(() => cwStop());
    await waitW(page, () => !CWZ.running, 20000, 'fine stop');
    const ab = await page.evaluate(() => ({ idx: CWZ.idx, plan: CWZ.plan.length, views: CWZ.views.length, status: CWZ.status, go: $('cwiz-go') && $('cwiz-go').textContent, moveTo: __SIM.calls.filter(([p]) => p === '/api/v1/action/move_to').length }));
    if (!/中止|Aborted/.test(ab.status) || !/继续|Continue/.test(ab.go) || ab.views < 5 || ab.moveTo) fail('fine abort ' + JSON.stringify(ab));
    await shot('3_captureA_abort', `idx ${ab.idx}, views kept ${ab.views}, button "${ab.go}", move_to calls ${ab.moveTo}`);
    const planBefore = await page.evaluate(() => JSON.stringify(CWZ.plan.map((p) => p.q)));
    const nBefore = (await jogs()).length;
    await page.click('#cwiz-go');
    await waitW(page, () => !CWZ.running, 120000, 'fine capture A');
    const ra = await page.evaluate((i0) => ({ idx: CWZ.idx, n: CWZ.plan.length, ok: CWZ.plan.filter((p) => p.state === 'ok').length, views: CWZ.views.filter((v) => v.ds === 0).length, plan: JSON.stringify(CWZ.plan.map((p) => p.q)), firstQ: toMotor6(cwFinePre(CWZ.plan[i0].q)) }), ab.idx);
    const js = await jogs();
    // every pose: pre-pose 2 deg below the target on every joint (or above near the lower limit), then the target
    // (a run stopped during a pre-pose leaves one unpaired jog: pair every jog to a plan target with the one before it)
    const targets = await page.evaluate(() => CWZ.plan.map((p) => toMotor6(p.q)));
    let pairOk = true, durOk = true, nTarget = 0;
    for (let k = 1; k < js.length; k++) {
      const b = js[k].target_positions;
      if (!targets.some((t) => t.every((v, i) => Math.abs(v - b[i]) < 1e-9))) continue;
      nTarget++;
      const a = js[k - 1].target_positions;
      if (!a.slice(0, 6).every((v, i) => Math.abs(Math.abs(v - b[i]) - 2 / 180 * Math.PI) < 1e-6)) pairOk = false;
    }
    if (nTarget < ra.ok) pairOk = false;
    for (const b of js) { const d = b.duration_secs; if (!(d >= 1.5) || b.target_positions.length !== 7) durOk = false; }
    const resumeOk = js[nBefore] && js[nBefore].target_positions.slice(0, 6).every((v, i) => Math.abs(v - ra.firstQ[i]) < 1e-9);
    if (ra.idx !== ra.n || ra.plan !== planBefore || !pairOk || !durOk || !resumeOk || ra.ok < 10) fail('fine capture A ' + JSON.stringify({ ...ra, plan: ra.plan === planBefore, pairOk, durOk, resumeOk, jogs: js.length }));
    await shot('4_captureA_done', `poses ${ra.ok}/${ra.n}, views ${ra.views}, jog pairs (pre 2 deg, target) ${pairOk}, duration >= 1.5 s ${durOk}, resumed at pose ${ab.idx + 1} with the same plan ${resumeOk && ra.plan === planBefore}`);
    // move the board
    await page.click('#cwiz-tob');
    if (!(await page.evaluate(() => __SIM.moveBoard()))) fail('fine move board ik');
    await shot('5_move');
    await page.click('#cwiz-boot');
    await waitW(page, () => CWZ.step === 'captureb' || (!CWZ.busy && CWZ.status), 20000, 'fine boot B');
    if (!(await page.evaluate(() => CWZ.step === 'captureb' && CWZ.place === 1 && CWZ.views.some((v) => v.ds === 0)))) fail('fine boot B ' + JSON.stringify(await page.evaluate(() => ({ step: CWZ.step, status: CWZ.status, place: CWZ.place }))));
    await page.check('#cwiz-runall');
    await page.click('#cwiz-go');
    await waitW(page, () => !CWZ.running, 120000, 'fine capture B');
    const rb = await page.evaluate(() => ({ idx: CWZ.idx, n: CWZ.plan.length, A: CWZ.views.filter((v) => v.ds === 0).length, B: CWZ.views.filter((v) => v.ds === 1).length }));
    if (rb.idx !== rb.n || rb.B < 20) fail('fine capture B ' + JSON.stringify(rb));
    await shot('6_captureB_done', `views A ${rb.A}, B ${rb.B}`);
    // solve in the worker
    await page.click('#cwiz-solve');
    await waitW(page, () => !CWZ.solving && CWZ.step === 'solve', 180000, 'fine solve');
    const so = await page.evaluate(() => { const r = CWZ.solve; return r ? { ok: r.ok, worker: r.worker, secs: +r.secs.toFixed(1), fails: r.gates.filter((g) => !g.ok).map((g) => g.key), ho: [r.kin.heldOut.ab0.mm, r.kin.heldOut.ab.mm, r.kin.heldOut.ba0.mm, r.kin.heldOut.ba.mm].map((v) => +v.toFixed(2)), j5: +(r.kin.joints[4].offset_rad * 180 / Math.PI).toFixed(2), warn: document.querySelectorAll('#cwiz-body .cwiz-zero').length, tables: document.querySelectorAll('#cwiz-body .cwiz-table').length, flat: [...document.querySelectorAll('#cwiz-body .cwiz-table')].filter((d) => d.getBoundingClientRect().height < 40).length } : { status: CWZ.status }; });
    if (!so.ok || !so.worker || so.warn !== 1 || so.tables !== 3 || so.flat) fail('fine solve ' + JSON.stringify(so));
    await shot('7_solve', `gates ok ${so.ok}, Web Worker ${so.worker}, ${so.secs} s, held-out A->B ${so.ho[0]} -> ${so.ho[1]} mm, B->A ${so.ho[2]} -> ${so.ho[3]} mm, J5 zero ${so.j5} deg, warnings ${so.warn}`);
    await page.evaluate(() => { const b = $('cwiz-body'); b.scrollTop = b.scrollHeight; });
    await shot('7b_solve_params');
    // review: torque on -> park first
    await page.click('#cwiz-review');
    await waitW(page, () => !!CWZ.dry && !CWZ.busy, 10000, 'fine dry run');
    const rv = await page.evaluate(() => ({ kin: [...document.querySelectorAll('#cwiz-body .cwiz-table tr')].filter((tr) => /kinematics J/.test(tr.textContent)).length, applyDis: $('cwiz-apply').disabled, park: !!$('cwiz-park'), dryKin: !!__SIM.calls.find(([p, b]) => p === '/api/v1/calibration/apply' && b.dry_run && b.kinematics) }));
    if (rv.kin !== 6 || !rv.applyDis || !rv.park || !rv.dryKin) fail('fine review ' + JSON.stringify(rv));
    await shot('8_review_torque', `kinematics rows ${rv.kin}, apply disabled while torqued ${rv.applyDis}`);
    await page.click('#cwiz-park');
    await waitW(page, () => !CWZ.busy && !S.obs.is_torqued, 10000, 'fine park');
    // first apply: armd answers KINEMATICS_ARMED (simulated), the second goes through
    await page.click('#cwiz-apply'); await page.click('#cwiz-apply');
    await waitW(page, () => !CWZ.busy && /KINEMATICS_ARMED/.test(CWZ.status), 10000, 'fine armed refusal');
    await page.click('#cwiz-apply'); await page.click('#cwiz-apply');
    await waitW(page, () => CWZ.step === 'applied' && !CWZ.busy, 10000, 'fine apply');
    const body = await page.evaluate(() => { const b = __SIM.calls.filter(([p, x]) => p === '/api/v1/calibration/apply' && x.dry_run === false).map(([, x]) => x); const k = b[b.length - 1].kinematics; return { n: b.length, keys: Object.keys(b[b.length - 1]).sort().join(','), joints: k.joints.length, fit: Object.keys(k.fit).sort().join(','), j1: JSON.stringify(k.joints[0]) }; });
    if (body.n !== 2 || body.joints !== 6 || !/heldout_mm/.test(body.fit) || !/kinematics/.test(body.keys) || !/"offset_rad":0,"scale":0/.test(body.j1)) fail('fine apply body ' + JSON.stringify(body));
    await shot('9_applied', `apply body keys ${body.keys}; kinematics.fit ${body.fit}`);
    if (errs.length) fail(`fine page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
