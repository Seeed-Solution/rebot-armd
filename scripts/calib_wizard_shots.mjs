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
// update message. Exits non-zero if a check fails.
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

for (const [vpName, viewport] of [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }]]) {
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
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
