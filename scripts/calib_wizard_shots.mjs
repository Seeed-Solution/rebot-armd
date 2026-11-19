#!/usr/bin/env node
// Headless walk through the camera calibration wizard (no armd needed) with screenshots of every step.
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_wizard_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and a mock of GET /api/v1/calibration;
// the control WebSocket is a stub. In the page an arm and camera are simulated: move_to (commandRPC) puts
// the arm at the IK solution of the target, the ArUco trigger (detTriggerReq) projects the board through
// a camera with fx 824 / fy 821 / cx 326 / cy 176 / k1 -0.1, 0.3 px noise, mounted 38 deg toward the jaws.
// The run goes board -> place (bootstrap with nominal values) -> capture (run all) -> solve (gates) ->
// review (dry-run reply) -> applied -> restore, in zh / en at desktop 1400x900, phone 390x844 and the WeChat-like 360x640 /
// 375x667 (CW_VIEWS=desktop,phone,wx360,wx375 picks some), checks every action row (act_rows_check.mjs), and writes
// PNGs to .cache/calib-wizard-shots/. Also: Stop aborts a run, and armd without calibration_apply shows the
// update message. Fine calibration mode (camera + arm geometry), same viewports and languages: the simulated arm's
// geometry differs from the URDF (zero offsets, a joint-axis tilt, an origin shift); joint-space capture of
// placement A with an abort and Continue, the board move (the camera loses the board; jog bar: one row docked on the video top, off the sticks;
// hand-guide enter / exit-and-lock, a search stopped by Stop and one that finds the board), placement B, the solve in a Web Worker (gates, held-out,
// per-joint parameters, the J5 zero warning), review with park / torque off, one apply carrying kinematics.
// Calibration sessions (mock of /api/v1/calibration/sessions in this server): both modes upload after a placement's
// capture and after the solve under one id. Load from device (desktop + phone, zh + en): the list on the board step,
// the stored fine session re-solved without capturing (level table), Apply disabled until a verify view passes (a
// moved board fails it), and a session of an arm without geometry error that chooses level L0 (review message,
// apply body with kinematics null).
// FINE_ONLY=1 runs only the fine calibration walk-through (and the load section). Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { actRowsCheck } from './act_rows_check.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-wizard-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
// calibration sessions as armd stores them (newest first, overwrite by id)
const sessions = new Map(), sessPosts = [];
let sessSeq = 0;
const sessMeta = (b) => ({ id: b.id, kind: b.kind, created_at: b.created_at, views: b.views.length, size: JSON.stringify(b).length, summary: b.summary || null });
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/api/v1/calibration/sessions' && req.method === 'POST') {
    let raw = '';
    req.on('data', (c) => { raw += c; });
    req.on('end', () => {
      const b = JSON.parse(raw);
      if (!['camera', 'fine'].includes(b.kind) || !Array.isArray(b.views)) return send(res, 400, { code: 'INVALID', message: 'kind / views' });
      b.id = b.id || 'S' + String(sessSeq + 1).padStart(4, '0');
      b.seq = ++sessSeq;
      sessions.set(b.id, b);
      sessPosts.push({ id: b.id, kind: b.kind, views: b.views.length, solved: !!b.solve, level: b.summary && b.summary.level });
      send(res, 200, sessMeta(b));
    });
    return;
  }
  if (u === '/api/v1/calibration/sessions') return send(res, 200, { sessions: [...sessions.values()].sort((a, b) => b.seq - a.seq).map(sessMeta), keep: 10 });
  if (u.startsWith('/api/v1/calibration/sessions/')) { const b = sessions.get(decodeURIComponent(u.split('/').pop())); return b ? send(res, 200, b) : send(res, 404, { code: 'NOT_FOUND', message: 'no session' }); }
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

// viewports: desktop, phone 390x844 and the WeChat-like 360x640 / 375x667; CW_VIEWS=desktop,phone picks a subset
const ALL_VIEWS = [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }], ['wx360', { width: 360, height: 640 }], ['wx375', { width: 375, height: 667 }]];
const VIEWS = process.env.CW_VIEWS ? ALL_VIEWS.filter(([n]) => process.env.CW_VIEWS.split(',').includes(n)) : ALL_VIEWS;
for (const [vpName, viewport] of (process.env.FINE_ONLY ? [] : VIEWS)) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'desktop' ? 1 : 2 });
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
      const rows = await page.evaluate(actRowsCheck, '#cwiz-panel, #cwiz-jogbar');
      if (rows.length) fail(`action rows ${step} ${lang}/${vpName}: ${rows.join(' | ')}`);
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
    await page.waitForTimeout(300);
    const cid = await page.evaluate(() => CWZ.sessionId), cposts = sessPosts.filter((x) => x.id === cid);
    if (!cid || cposts.length < 2 || !cposts[0].views || cposts[0].solved || !cposts[cposts.length - 1].solved || cposts.some((x) => x.kind !== 'camera')) fail('camera session uploads ' + JSON.stringify({ cid, cposts }));
    await shot('4_solve', `gates ok ${sol.ok} K ${sol.K} offset ${sol.off} mm table ${sol.z} mm; session ${cid}: ${cposts.length} uploads (${cposts.map((x) => x.views + (x.solved ? ' solved' : '')).join(', ')})`);
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
  // the camera above a table point where the board is not (jog / hand-guide / search start)
  W.camAt = (x, y) => { const q = camAbove(CW.make(CW.rotOf(W.TbbB), [x, y, 0.0005])); if (q) S.obs.positions = [...toMotor6(q), 0]; return !!q; };
  // board markers in the live overlay: the markers the camera sees at the arm's current pose (as rebot-vision's stream)
  const seen = () => {
    const G = CALM.fk(cwModelQ(S.obs.positions), W.kc), Tcb = CW.mul(CW.inv(CW.mul(G, W.X)), W.Tbb), R = CW.rotOf(Tcb), t = CW.posOf(Tcb);
    return CW.makeBoard({ markerSide: 0.025 }).markers.filter((m) => m.corners.map((c) => CW.project(W.K, R, t, c)).every(([u, v, zz]) => zz > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)).map((m) => m.id);
  };
  clearInterval(W.liveTick);
  W.liveTick = setInterval(() => { S.detections = seen().map((id) => ({ aruco: { id, t: [0, 0, 0.3] } })); S.detLastReceivedAt = performance.now(); }, 150);
  // gravity compensation (hand-guiding) as armd: enter -> ramp_in -> active; exit -> position, pose locked
  S.obs.gravity_comp = { supported: true, blocker: null, phase: null, ramp: 0, last_exit: null };
  S.obs.control_mode = 'position';
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
    if (p === '/api/v1/mode/gravity_comp') {
      W.calls.push([p, body]);
      const g = S.obs.gravity_comp;
      if (body.enable) { S.obs.control_mode = 'gravity_comp'; Object.assign(g, { phase: 'ramp_in', ramp: 0.4 }); setTimeout(() => Object.assign(g, { phase: 'active', ramp: 1 }), 300); }
      else { Object.assign(g, { phase: 'exiting' }); setTimeout(() => { S.obs.control_mode = 'position'; Object.assign(g, { phase: null, ramp: 0, last_exit: { code: 'REQUESTED', message: '', at: Date.now() } }); }, 200); }
      return {};
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
for (const [vpName, viewport] of VIEWS) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'desktop' ? 1 : 2 });
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
    // main-thread blocking during plan and solve (max gap of a 10 ms timer); the phone runs
    // use the no-worker fallback (sliced on the main thread)
    const noWorker = vpName === 'phone';
    await page.evaluate((nw) => {
      // probe: a 10 ms timer on the main thread (as the lease heartbeat timer); a gap between two ticks is the
      // time the thread was blocked (longest task + timer slack). PerformanceObserver('longtask') reported nothing
      // in headless Chromium here, not even for a synchronous control block.
      __SIM.long = []; __SIM.phase = null;
      let last = performance.now();
      setInterval(() => { const n = performance.now(); if (__SIM.phase) __SIM.long.push([__SIM.phase, n - last]); last = n; }, 10);
      if (nw) window.Worker = undefined;
    }, noWorker);
    const phase = (ph) => page.evaluate((x) => { __SIM.phase = x; }, ph);
    const longest = async (ph) => page.evaluate((x) => Math.max(0, ...__SIM.long.filter(([p]) => p === x).map(([, d]) => d)), ph);
    const longs = {};
    const shot = async (step, extra = '') => {
      await page.waitForTimeout(250);
      const info = await page.evaluate(() => ({ open: !$('cwiz').hidden, on: [...document.querySelectorAll('#cwiz-steps li.on')].map((x) => x.textContent).join(),
        raw: /\bcw\.[a-z]/.test($('cwiz-panel').innerText), hscroll: $('cwiz-panel').scrollWidth > $('cwiz-panel').clientWidth + 1 }));
      if (!info.open || info.raw || info.hscroll) fail(`fine ${step} ${lang}/${vpName}: ${JSON.stringify(info)}`);
      const rows = await page.evaluate(actRowsCheck, '#cwiz-panel, #cwiz-jogbar');
      if (rows.length) fail(`action rows ${step} ${lang}/${vpName}: ${rows.join(' | ')}`);
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
    await phase('planA');
    await page.click('#cwiz-boot');
    await waitW(page, () => CWZ.step === 'capture' || (!CWZ.busy && CWZ.status), 60000, 'fine boot A');
    await page.waitForTimeout(100); await phase(null); longs.planA = await longest('planA');
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
    // move the board: the camera no longer sees it; three ways to bring it back
    await page.click('#cwiz-tob');
    const st = (extra) => page.evaluate((x) => ({ live: cwLiveCount(), boot: $('cwiz-boot') && $('cwiz-boot').disabled, why: [...document.querySelectorAll('#cwiz-body .cwiz-why')].map((e) => e.textContent).join(' | '), status: CWZ.status, ...(x ? eval(x) : {}) }), extra || null);
    await page.evaluate(() => { __SIM.Tbb = __SIM.TbbB; });
    await page.waitForTimeout(900);
    const mv = await st("({ btns: ['jog', 'hand', 'search'].map((k) => !!$('cwiz-find-' + k)).join(), hint: $('cwiz-body').innerText.includes(t('cw.f.move.do')) })");
    if (mv.btns !== 'true,true,true' || !mv.boot || mv.live >= 6 || !mv.why || !mv.hint) fail('fine move step ' + JSON.stringify(mv));
    await shot('5_move', `three buttons ${mv.btns}, live ${mv.live}, capture disabled ${mv.boot}: "${mv.why}"`);
    // a) jog: panel hidden, floating bar above the sticks; the joystick stays reachable
    await page.click('#cwiz-find-jog');
    await page.waitForTimeout(400);
    const jogGeo = () => page.evaluate(() => {
      const bar = $('cwiz-jogbar'), b = bar.getBoundingClientRect(), H = innerHeight, W = innerWidth;
      const pads = [...document.querySelectorAll(CWJ_AVOID), $('vs-video').parentElement].map((e) => [e.id || e.className, e.getBoundingClientRect()]).filter(([, r]) => r.width > 0 && r.bottom > 0 && r.top < H);
      const cover = pads.filter(([, r]) => r.left < b.right && r.right > b.left && r.top < b.bottom && r.bottom > b.top).map(([n]) => n);
      // the centre of every visible stick is still the topmost element there (nothing of the wizard on it)
      const blocked = pads.filter(([, r]) => { const x = (r.left + r.right) / 2, y = (Math.max(0, r.top) + Math.min(H, r.bottom)) / 2, e = document.elementFromPoint(x, y); return e && (e.closest('#cwiz-jogbar') || e.closest('#cwiz-panel')); }).map(([n]) => n);
      // one row (<= 48 px) docked at the top edge of the video area: the lower part of the video (the board) stays free
      const v = $('video-box').getBoundingClientRect();
      const docked = b.height <= 48 && b.top >= v.top - 1 && b.bottom <= v.top + Math.max(140, v.height * 0.45);
      return { bar: !bar.hidden, panel: $('cwiz-panel').getBoundingClientRect().height, inView: b.left >= 0 && b.right <= W && b.top >= 0 && b.bottom <= H, docked, y: Math.round(b.top), h: Math.round(b.height), sticks: pads.length, cover, blocked,
        cap: $('cwiz-jog-cap').disabled, live: $('cwiz-jog-live').textContent, why: $('cwiz-jog-cap').title };
    });
    const jg = await jogGeo();
    if (!jg.bar || jg.panel || !jg.inView || !jg.docked || jg.cover.length || jg.blocked.length || !jg.cap || !jg.why || !jg.sticks) fail('fine jog bar ' + JSON.stringify(jg));
    await shot('5a_jog', `bar at y ${jg.y} (h ${jg.h}${jg.docked ? ', docked on the video top' : ''}), panel hidden, ${jg.sticks} sticks / toggles visible, covered ${jg.cover.length}, sticks blocked ${jg.blocked.length}, capture disabled ${jg.cap}: "${jg.why}"`);
    const qAway = await page.evaluate(() => S.obs.positions.slice());
    await page.evaluate(() => __SIM.moveBoard()); // the user jogs until the camera sees the board
    await waitW(page, () => !$('cwiz-jog-cap').disabled, 5000, 'fine jog ready');
    const jr = await jogGeo();
    if (jr.cap || !jr.docked || jr.cover.length || jr.blocked.length) fail('fine jog ready ' + JSON.stringify(jr));
    await shot('5b_jog_ready', `"${jr.live}", capture enabled ${!jr.cap}`);
    await page.click('#cwiz-jog-back');
    await page.waitForTimeout(300);
    const back = await page.evaluate(() => ({ panel: $('cwiz-panel').getBoundingClientRect().height > 100, bar: $('cwiz-jogbar').hidden, boot: $('cwiz-boot').disabled }));
    if (!back.panel || !back.bar || back.boot) fail('fine jog back ' + JSON.stringify(back));
    // b) hand-guide: enter, move by hand, exit and lock; capture only after the exit
    await page.evaluate((q) => { S.obs.positions = q.slice(); }, qAway);
    await page.waitForTimeout(400);
    await page.click('#cwiz-find-hand');
    await waitW(page, () => S.obs.gravity_comp.phase === 'active', 5000, 'fine gc active');
    await page.evaluate(() => __SIM.moveBoard()); // moved by hand
    await page.waitForTimeout(900);
    const hg = await st("({ hand: $('cwiz-find-hand').textContent, gc: __SIM.calls.filter(([p]) => p === '/api/v1/mode/gravity_comp').map(([, b]) => b.enable).join() })");
    if (!hg.boot || hg.live < 6 || !/退出并锁定|Exit and hold/.test(hg.hand) || hg.gc !== 'true') fail('fine hand-guide ' + JSON.stringify(hg));
    await shot('5c_handguide', `live ${hg.live}, capture disabled while hand-guiding ${hg.boot}: "${hg.why}"`);
    await page.click('#cwiz-find-hand');
    await waitW(page, () => S.obs.control_mode === 'position' && !S.obs.gravity_comp.phase && !$('cwiz-boot').disabled, 5000, 'fine gc exit');
    const hx = await st("({ gc: __SIM.calls.filter(([p]) => p === '/api/v1/mode/gravity_comp').map(([, b]) => b.enable).join() })");
    if (hx.boot || hx.gc !== 'true,false') fail('fine hand-guide exit ' + JSON.stringify(hx));
    await shot('5c2_handguide_locked', `after exit: capture enabled ${!hx.boot}, gravity_comp calls ${hx.gc}`);
    // c) search: Stop during a search, then a search that finds the board
    const search = async (stop) => {
      await page.evaluate(() => { __SIM.camAt(0.24, 0.24); __SIM.calls.length = 0; });
      await page.waitForTimeout(400);
      await page.click('#cwiz-find-search');
      if (stop) {
        await waitW(page, () => CWZ.search && CWZ.search.k >= 2, 30000, 'fine search progress');
        await page.waitForTimeout(250);
        await shot('5e_search_running', await page.evaluate(() => `"${document.querySelector('#cwiz-find-prog').innerText.split('\n')[0]}", Stop ${!!$('cwiz-find-stop')} / footer ${!$('cwiz-stop').hidden}`));
        await page.click('#cwiz-find-stop');
      }
      await waitW(page, () => CWZ.search && !CWZ.search.running, 60000, 'fine search end');
      await page.waitForTimeout(900);
      // found: the live overlay (150 ms mock stream) and the render tick (700 ms) catch up
      if (!stop) await page.waitForFunction(() => cwLiveCount() >= 6 && !$('cwiz-boot').disabled, null, { timeout: 5000 }).catch(() => {});
      return page.evaluate(() => ({ k: CWZ.search.k, n: CWZ.search.n, found: CWZ.search.found, status: CWZ.status, live: cwLiveCount(), boot: $('cwiz-boot').disabled,
        moves: __SIM.calls.filter(([p]) => p === '/api/v1/action/move_to').length, speeds: __SIM.calls.filter(([p]) => p === '/api/v1/action/move_to').every(([, b]) => b.speed.linear === 0.03 && b.speed.angular === 0.3 && b.tag === 'calib-wizard'),
        stops: __SIM.calls.filter(([p]) => p === '/api/v1/motion/stop').length, running: CWZ.running }));
    };
    const ss = await search(true);
    if (ss.running || ss.found || !ss.stops || !/中止|aborted/i.test(ss.status) || !/停止|Stop/.test(ss.status) || ss.k >= ss.n) fail('fine search stop ' + JSON.stringify(ss));
    await shot('5e_search_stopped', `stopped at candidate ${ss.k}/${ss.n}, move_to ${ss.moves}, motion/stop ${ss.stops}: "${ss.status}"`);
    const sf = await search(false);
    if (!sf.found || sf.found.k < 2 || sf.found.m < 6 || sf.live < 6 || sf.boot || !sf.speeds || sf.moves > sf.found.k + 1 || sf.running) fail('fine search found ' + JSON.stringify(sf));
    await shot('5d_search_found', `found after ${sf.found.k}/${sf.n} candidates, re-aim ${!!sf.found.aimed}, ${sf.found.m} markers, move_to ${sf.moves} (speed caps ${sf.speeds}), live ${sf.live}, capture enabled ${!sf.boot}`);
    await phase('planB');
    await page.click('#cwiz-boot');
    await waitW(page, () => CWZ.step === 'captureb' || (!CWZ.busy && CWZ.status), 60000, 'fine boot B');
    await page.waitForTimeout(100); await phase(null); longs.planB = await longest('planB');
    if (!(await page.evaluate(() => CWZ.step === 'captureb' && CWZ.place === 1 && CWZ.views.some((v) => v.ds === 0)))) fail('fine boot B ' + JSON.stringify(await page.evaluate(() => ({ step: CWZ.step, status: CWZ.status, place: CWZ.place }))));
    await page.check('#cwiz-runall');
    await page.click('#cwiz-go');
    await waitW(page, () => !CWZ.running, 120000, 'fine capture B');
    const rb = await page.evaluate(() => ({ idx: CWZ.idx, n: CWZ.plan.length, A: CWZ.views.filter((v) => v.ds === 0).length, B: CWZ.views.filter((v) => v.ds === 1).length }));
    if (rb.idx !== rb.n || rb.B < 20) fail('fine capture B ' + JSON.stringify(rb));
    await shot('6_captureB_done', `views A ${rb.A}, B ${rb.B}`);
    // solve in the worker
    await phase('solve');
    await page.click('#cwiz-solve');
    await waitW(page, () => !CWZ.solving && CWZ.step === 'solve', 300000, 'fine solve');
    await page.waitForTimeout(100); await phase(null); longs.solve = await longest('solve');
    // control: the same plan run synchronously on the main thread (what the page did before) must show up as a long task
    await phase('control');
    await page.evaluate(() => { cwPlan(CWZ.est, cwModelQ(S.obs.positions), CWF.n); });
    await page.waitForTimeout(200); await phase(null); longs.control = await longest('control');
    if (!(longs.control > 50)) fail('fine blocking probe control ' + longs.control);
    const pw = await page.evaluate(() => ({ plan: CWZ.planWorker, solve: CWZ.solve && CWZ.solve.worker }));
    const lmax = Math.max(longs.planA, longs.planB, longs.solve);
    if (lmax >= 300 || pw.plan !== !noWorker || pw.solve !== !noWorker) fail('fine main-thread long task ' + JSON.stringify({ longs, pw }));
    console.log(`  main-thread longest block (${lang}/${vpName}, ${noWorker ? 'no Worker: sliced on the main thread' : 'Web Worker'}): plan A ${longs.planA.toFixed(0)} ms, plan B ${longs.planB.toFixed(0)} ms, solve ${longs.solve.toFixed(0)} ms (max gap of a 10 ms timer); control: the same plan synchronously ${longs.control.toFixed(0)} ms; plan in worker ${pw.plan}, solve in worker ${pw.solve}`);
    const so = await page.evaluate(() => { const r = CWZ.solve; return r ? { ok: r.ok, worker: r.worker, secs: +r.secs.toFixed(1), fails: r.gates.filter((g) => !g.ok).map((g) => g.key), ho: [r.kin.heldOut.ab0.mm, r.kin.heldOut.ab.mm, r.kin.heldOut.ba0.mm, r.kin.heldOut.ba.mm].map((v) => +v.toFixed(2)), j5: +(r.kin.joints[4].offset_rad * 180 / Math.PI).toFixed(2), warn: document.querySelectorAll('#cwiz-body .cwiz-zero').length, tables: document.querySelectorAll('#cwiz-body .cwiz-table').length, flat: [...document.querySelectorAll('#cwiz-body .cwiz-table')].filter((d) => d.getBoundingClientRect().height < 40).length,
      level: r.kin.level, lvRows: document.querySelectorAll('#cwiz-body .cwiz-levels tr').length, lvChosen: document.querySelectorAll('#cwiz-body .cwiz-levels tr.chg').length, lvText: r.kin.levels.map((l) => `${l.chosen ? '*' : ''}${l.name} ${l.ab.toFixed(2)}/${l.ba.toFixed(2)}`).join(' ') } : { status: CWZ.status }; });
    if (!so.ok || so.worker !== !noWorker || so.warn !== 1 || so.tables !== 4 || so.flat || so.lvRows !== 5 || so.lvChosen !== 1 || so.level === 'L0') fail('fine solve ' + JSON.stringify(so));
    await shot('7_solve', `gates ok ${so.ok}, Web Worker ${so.worker}, ${so.secs} s, held-out A->B ${so.ho[0]} -> ${so.ho[1]} mm, B->A ${so.ho[2]} -> ${so.ho[3]} mm, J5 zero ${so.j5} deg, warnings ${so.warn}; levels ${so.lvText}`);
    const lvBox = await page.evaluate(() => { const e = document.querySelector('#cwiz-body .cwiz-levels'); e.scrollIntoView({ block: 'center' }); const b = e.getBoundingClientRect(), pb = $('cwiz-panel').getBoundingClientRect(); return { w: b.width, pw: pb.width, inside: b.left >= pb.left - 1 && b.right <= pb.right + 1 }; });
    if (!lvBox.inside) fail('fine levels table width ' + JSON.stringify(lvBox));
    await shot('7a_levels', `levels table ${Math.round(lvBox.w)} px in a ${Math.round(lvBox.pw)} px panel`);
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
    const fid = await page.evaluate(() => CWZ.sessionId), fposts = sessPosts.filter((x) => x.id === fid);
    // capture A done, capture B done (the B run ends the capture), the solve
    if (!fid || fposts.length < 3 || fposts.some((x) => x.kind !== 'fine') || !fposts[fposts.length - 1].solved || fposts[fposts.length - 1].level !== so.level) fail('fine session uploads ' + JSON.stringify({ fid, fposts }));
    await shot('9_applied', `apply body keys ${body.keys}; kinematics.fit ${body.fit}; session ${fid}: ${fposts.length} uploads (${fposts.map((x) => x.views + (x.solved ? ' solved ' + x.level : '')).join(', ')})`);
    if (errs.length) fail(`fine page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
// ---- load from device: re-solve a stored session, verify view, the L0 case
const l0Ids = new Set(); // sessions rendered here through the URDF arm (not the walk-through's simulated arm)
for (const [vpName, viewport] of VIEWS) {
  for (const lang of ['zh', 'en']) {
    const fineIds = [...sessions.values()].filter((x) => x.kind === 'fine' && x.solve && x.levels && !l0Ids.has(x.id)).sort((a, b) => b.seq - a.seq).map((x) => x.id);
    if (!fineIds.length) { fail('load: no stored fine session'); break; }
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'desktop' ? 1 : 2 });
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
    if (!(await page.evaluate(setupSim)) || !(await page.evaluate(setupFine))) fail('load sim start');
    const shot = async (step, extra = '') => {
      await page.waitForTimeout(250);
      const info = await page.evaluate(() => ({ open: !$('cwiz').hidden, on: [...document.querySelectorAll('#cwiz-steps li.on')].map((x) => x.textContent).join(),
        raw: /\bcw\.[a-z]/.test($('cwiz-panel').innerText), hscroll: $('cwiz-panel').scrollWidth > $('cwiz-panel').clientWidth + 1 }));
      if (!info.open || info.raw || info.hscroll) fail(`load ${step} ${lang}/${vpName}: ${JSON.stringify(info)}`);
      const rows = await page.evaluate(actRowsCheck, '#cwiz-panel, #cwiz-jogbar');
      if (rows.length) fail(`action rows ${step} ${lang}/${vpName}: ${rows.join(' | ')}`);
      const file = path.join(out, `cwiz_load_${step}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(`${path.relative(root, file)}  step="${info.on}" ${extra}`);
    };
    const posts0 = sessPosts.length;
    await page.evaluate(() => cwOpen(true));
    await page.waitForTimeout(500);
    await page.click('#cwiz-load');
    await waitW(page, () => CWZ.sessions && !CWZ.sessions.busy, 5000, 'load list');
    const li = await page.evaluate((id) => ({ n: CWZ.sessions.list.length, idx: CWZ.sessions.list.findIndex((x) => x.id === id), items: [...document.querySelectorAll('#cwiz-sess .cwiz-sess-item')].map((b) => b.textContent) }), fineIds[0]);
    if (li.n < 2 || li.idx < 0 || li.items.length !== li.n) fail('load list ' + JSON.stringify(li));
    await shot('1_list', `${li.n} sessions: ${li.items.slice(0, 3).map((x) => JSON.stringify(x)).join(', ')}`);
    // the stored fine session (corrected sim arm): re-solved without capturing
    await page.click('#cwiz-sess-' + li.idx);
    await waitW(page, () => CWZ.step === 'solve' && !CWZ.solving && !!CWZ.solve, 300000, 'load solve');
    const ls = await page.evaluate(() => ({ mode: CWZ.mode, loaded: !!CWZ.loaded, ok: CWZ.solve.ok, level: CWZ.solve.kin.level, lv: document.querySelectorAll('#cwiz-body .cwiz-levels tr').length, status: CWZ.status,
      jogs: __SIM.calls.filter(([p]) => p === '/api/v1/motion/jog' || p === '/api/v1/action/move_to').length, navs: [...document.querySelectorAll('#cwiz-steps li.nav')].map((x) => x.dataset.step).join() }));
    if (ls.mode !== 'fine' || !ls.loaded || !ls.ok || ls.lv !== 5 || ls.jogs || !/加载|Loaded/.test(ls.status) || /capture/.test(ls.navs)) fail('load solve ' + JSON.stringify(ls));
    await shot('2_solved', `mode ${ls.mode}, level ${ls.level}, gates ok ${ls.ok}, arm moves ${ls.jogs}, "${ls.status}"`);
    await page.evaluate(() => { const e = document.querySelector('#cwiz-body .cwiz-levels'); if (e) e.scrollIntoView({ block: 'center' }); });
    await shot('3_levels');
    // review: Apply waits for a verify view; the board moved -> fails; back in place -> passes
    await page.click('#cwiz-review');
    await waitW(page, () => !!CWZ.dry && !CWZ.busy, 10000, 'load dry run');
    const r0 = await page.evaluate(() => ({ apply: $('cwiz-apply').disabled, verify: !!$('cwiz-verify'), hint: $('cwiz-body').innerText.includes(t('cw.sess.verify')) }));
    if (!r0.apply || !r0.verify || !r0.hint) fail('load review before verify ' + JSON.stringify(r0));
    await shot('4_review_verify', `apply disabled ${r0.apply}, verify button ${r0.verify}`);
    await page.evaluate(() => { __SIM.Tbb0 = __SIM.Tbb; __SIM.Tbb = CWZ.CW.mul(CWZ.CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.03, 0.01, 0]), __SIM.Tbb); });
    await page.click('#cwiz-verify');
    await waitW(page, () => !CWZ.busy && !!CWZ.verify, 10000, 'load verify moved');
    const r1 = await page.evaluate(() => ({ v: CWZ.verify, apply: $('cwiz-apply').disabled }));
    if (r1.v.ok || !r1.apply) fail('load verify with a moved board ' + JSON.stringify(r1));
    await shot('5_verify_moved', `board moved 30 / 10 mm: ${r1.v.mm != null ? r1.v.mm.toFixed(2) + ' mm' : r1.v.why}, apply disabled ${r1.apply}`);
    await page.evaluate(() => { __SIM.Tbb = __SIM.Tbb0; });
    await page.click('#cwiz-verify');
    await waitW(page, () => !CWZ.busy && !!CWZ.verify, 10000, 'load verify');
    const r2 = await page.evaluate(() => ({ v: CWZ.verify, park: !!$('cwiz-park') }));
    if (!r2.v.ok || !r2.park) fail('load verify ' + JSON.stringify(r2));
    await shot('6_verified', `${r2.v.mm.toFixed(3)} mm (${r2.v.px.toFixed(2)} px, placement ${r2.v.board}), park offered ${r2.park}`);
    await page.click('#cwiz-park');
    await waitW(page, () => !CWZ.busy && !S.obs.is_torqued, 10000, 'load park');
    await page.click('#cwiz-apply'); await page.click('#cwiz-apply');
    await waitW(page, () => !CWZ.busy && (CWZ.step === 'applied' || /KINEMATICS_ARMED/.test(CWZ.status)), 10000, 'load apply');
    if (await page.evaluate(() => CWZ.step !== 'applied')) { await page.click('#cwiz-apply'); await page.click('#cwiz-apply'); await waitW(page, () => CWZ.step === 'applied' && !CWZ.busy, 10000, 'load apply 2'); }
    const ap = await page.evaluate(() => { const b = __SIM.calls.filter(([p, x]) => p === '/api/v1/calibration/apply' && x.dry_run === false).pop()[1]; return { kin: !!b.kinematics, level: b.rms.kin_level }; });
    if (!ap.kin || ap.level !== ls.level) fail('load apply body ' + JSON.stringify(ap));
    // re-solved session uploaded under the same id
    const same = sessPosts.slice(posts0).filter((x) => x.id === fineIds[0]).length;
    if (same < 1 || sessPosts.slice(posts0).some((x) => x.id !== fineIds[0])) fail('load re-solve upload ' + JSON.stringify(sessPosts.slice(posts0)));
    // L0: the same views rendered through an arm without geometry error (URDF), stored as a new session
    const l0id = await page.evaluate(async (id) => {
      const W = __SIM, CW = CWZ.CW, s = await (await fetch('/api/v1/calibration/sessions/' + id)).json();
      const boards = [W.Tbb0 || W.Tbb, W.TbbB], board = CW.makeBoard({ markerSide: 0.025 });
      const rnd = () => { W.seed = (W.seed * 1103515245 + 12345) % 2147483648; return W.seed / 2147483648; };
      const gauss = () => Math.sqrt(-2 * Math.log(1 - rnd())) * Math.cos(2 * Math.PI * rnd());
      const views = s.views.map((v) => {
        const G = CALM.fk(v.q, null), Tcb = CW.mul(CW.inv(CW.mul(G, W.X)), boards[v.ds]), R = CW.rotOf(Tcb), t = CW.posOf(Tcb), markers = [];
        for (const m of board.markers) { const px = m.corners.map((c) => CW.project(W.K, R, t, c)); if (px.every(([u, vv, zz]) => zz > 0 && u >= 4 && u <= 636 && vv >= 4 && vv <= 356)) markers.push({ id: m.id, corners: px.map(([u, vv]) => [u + 0.3 * gauss(), vv + 0.3 * gauss()]) }); }
        return { ...v, markers, T_base_end: G };
      });
      const { id: _, solve, levels, gates, ...rest } = s;
      const r = await fetch('/api/v1/calibration/sessions', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ ...rest, created_at: new Date().toISOString(), views, summary: { views: views.length } }) });
      return (await r.json()).id;
    }, fineIds[0]);
    l0Ids.add(l0id);
    await page.evaluate(() => { CWZ.step = 'board'; CWZ.views = []; CWZ.loaded = null; CWZ.solve = null; CWZ.sessionId = null; CWZ.dry = null; CWZ.applied = null; cwRender(); });
    await page.click('#cwiz-load');
    await waitW(page, () => CWZ.sessions && !CWZ.sessions.busy, 5000, 'load list 2');
    const i0 = await page.evaluate((id) => CWZ.sessions.list.findIndex((x) => x.id === id), l0id);
    await page.click('#cwiz-sess-' + i0);
    await waitW(page, () => CWZ.step === 'solve' && !CWZ.solving && !!CWZ.solve, 300000, 'load L0 solve');
    const l0 = await page.evaluate(() => ({ level: CWZ.solve.kin.level, ok: CWZ.solve.ok, kin: CWZ.solve.apply && CWZ.solve.apply.kinematics, msg: !!document.querySelector('#cwiz-body .cwiz-l0'), lv: CWZ.solve.kin.levels.map((l) => `${l.chosen ? '*' : ''}${l.name} ${l.ab.toFixed(3)}/${l.ba.toFixed(3)}`).join(' ') }));
    if (l0.level !== 'L0' || !l0.ok || l0.kin !== null || !l0.msg) fail('load L0 ' + JSON.stringify(l0));
    await page.evaluate(() => { const e = document.querySelector('#cwiz-body .cwiz-levels'); if (e) e.scrollIntoView({ block: 'center' }); });
    await shot('7_L0_solve', `levels ${l0.lv}; apply kinematics ${JSON.stringify(l0.kin)}`);
    // the L0 review: the message, the dry run with kinematics null (removes a stored correction)
    await page.evaluate(() => { S.obs.is_torqued = true; S.state = 'Armed'; });
    await page.click('#cwiz-review');
    await waitW(page, () => !!CWZ.dry && !CWZ.busy, 10000, 'load L0 dry run');
    const l0r = await page.evaluate(() => ({ msg: !!document.querySelector('#cwiz-body .cwiz-l0'), dry: __SIM.calls.filter(([p, b]) => p === '/api/v1/calibration/apply' && b.dry_run).pop()[1] }));
    if (!l0r.msg || l0r.dry.kinematics !== null || !('kinematics' in l0r.dry)) fail('load L0 review ' + JSON.stringify({ msg: l0r.msg, kin: l0r.dry.kinematics }));
    await page.evaluate(() => { const e = document.querySelector('#cwiz-body .cwiz-l0'); if (e) e.scrollIntoView({ block: 'center' }); });
    await shot('8_L0_review', `L0 message ${l0r.msg}, dry-run kinematics ${JSON.stringify(l0r.dry.kinematics)}`);
    if (errs.length) fail(`load page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
