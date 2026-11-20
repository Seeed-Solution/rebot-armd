#!/usr/bin/env node
// Headless walk through the self-service verification steps of the fine calibration wizard (T20: fingertip sticker,
// hover acceptance, review / apply) with screenshots, no armd needed:
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_verify_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and the real sticker JPEGs of the office B601-DM
// (tests/fixtures/calib_t20) as /api/v1/camera/snapshot. In the page: the candidate of a finished fine solve (the
// office unit's intrinsics and camera mount), a simulated arm, an ArUco trigger that projects the board (the real
// paper 1.5 mm below the candidate's), armd's dry-run / apply replies and a simulated calib_hover job on the device
// (GET /api/v1/calibration/hover/<id>). Walk: board step (sticker print button) -> fingertip (sticker not found:
// ROI guidance; found: tip, gates) -> hover plan -> running (Stop visible) -> result and confirmation -> review (park
// and turn torque off, dry run with table_z_source hover) -> applied (read back, floor) and an interrupted job
// (cancelled LEASE_LOST, torque off) with Resume. Desktop 1400x900 expert and phone 390x844 simple, zh and en; checks
// every action row (act_rows_check.mjs), raw i18n keys, horizontal scroll, Stop in the viewport while hovering.
// PNGs go to .cache/calib-verify-shots/. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { actRowsCheck } from './act_rows_check.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-verify-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));
const FX = path.join(root, 'tests/fixtures/calib_t20');
const fixtures = JSON.parse(fs.readFileSync(path.join(FX, 'fixtures.json'), 'utf8'));
const jpegs = ['f03', 'f04', 'f06'].map((f) => fs.readFileSync(path.join(FX, f + '.jpg')));
let snapN = 0;
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(js); return; }
  // the three real frames in turn; a counter after the JPEG end marker makes every reply a new frame (decoders ignore it)
  if (u === '/fx/snapshot.jpg') { const n = snapN++; res.writeHead(200, { 'content-type': 'image/jpeg', 'cache-control': 'no-store' }); res.end(Buffer.concat([jpegs[n % 3], Buffer.from('frame' + n)])); return; }
  if (u === '/api/v1/vision/mode') { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify({ api_version: 2, mode: 'aruco', modes: ['off', 'yolo', 'aruco', 'both'], enabled: true, per_request_pipelines: true, subscriptions: true, available: { yolo: true, aruco: true } })); return; }
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
let bad = 0;
const shots = [];
const fail = (m) => { bad++; console.log('  FAIL ' + m); };

// in-page setup (runs in the browser): candidate, simulated arm, armd replies
function setupVerify([ui, intr]) {
  setUi(ui);
  return cwLoad().then((CW) => {
    const D = 180 / Math.PI;
    const W = window.__V = { calls: [], aid: 0, job: null, snap: 'fixture', drop: false, applied: null, seed: 3 };
    const rnd = () => { W.seed = (W.seed * 1103515245 + 12345) % 2147483648; return W.seed / 2147483648; };
    W.K = { fx: intr.fx, fy: intr.fy, cx: intr.cx, cy: intr.cy, k1: intr.k1, k2: intr.k2 };
    // the office unit's calibrated mount (cal_obs.txt)
    W.X = CW.make(CW.quatR([0.3046742640999801, -0.6364661370609692, 0.6289449528719083, -0.32636282788757687]), [-0.076939657, 0.005015815, 0.06891869]);
    const yaw = -80 / D, Rb = CW.mul(CW.rpyR(0, 0, yaw), CW.rpyR(Math.PI, 0, 0));
    W.Tbb = CW.make(Rb, [0.30, 0.02, 0.0105]);
    W.Ttrue = CW.make(Rb, [0.30, 0.02, 0.0090]); // the paper is 1.5 mm lower than the candidate says
    // start pose: camera 0.28 m above the board looking down
    const x = [Math.cos(yaw), Math.sin(yaw), 0], z = [0, 0, -1], y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
    const T0 = CW.mul(CW.make([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]], [0.30, 0.02, 0.28]), CW.inv(W.X));
    let q0 = null;
    // seeds around the usual elbow-up branch (J1 toward the board), as a real arm reaches this view
    const { lo, hi } = modelLimits6();
    for (let k = 0; k < 400 && !q0; k++) { const s = [0.07, 1.0, -1.2, 0.2, 0, 0].map((v) => v + (rnd() - 0.5) * 0.8 * Math.min(1, k / 40 + 0.2)); const r = VIK.solve(T0, s, { lo, hi, iters: 300 }); if (r.ok && Math.abs(r.q[0]) < 1.5) q0 = r.q; }
    S.dof = 7; S.armDof = 6; S.profileId = null; S.state = 'Armed';
    S.caps = { ...(S.caps || {}), calibration_apply: true, calibration_kinematics: true, calibration_verify: true };
    S.leaseGeneration = 1; S.epoch = 1;
    S.obs = { positions: [...toMotor6(q0), 0], velocities: [0, 0, 0, 0, 0, 0, 0], is_torqued: true, safety_state: 'Armed', epoch_id: 1, fault: null, action: null, previous_action: null, gripper: { fraction: 0 } };
    clearInterval(W.tick); W.tick = setInterval(() => { S.obsAt = Date.now(); }, 50);
    // the candidate of a finished fine solve
    const apply = { intrinsics: { ...W.K, p1: 0, p2: 0, k3: 0, width: 640, height: 360, marker_size: 0.025 },
      camera_mount: { offset_m: CW.posOf(W.X), q_end_cam: CW.quatOf(CW.rotOf(W.X)) }, table_z_m: 0.0241, kinematics: null, rms: { reproj_px: 0.21 } };
    Object.assign(CWZ, { mode: 'fine', markerMm: 25, thickMm: 0.5, step: 'board', place: 1, status: '' });
    CWZ.board = cwBoard();
    CWZ.solve = { ok: true, gates: [{ key: 'views', ok: true, value: 70, limit: 30, unit: 'count' }], final: { K: W.K, X: W.X, boards: [W.Tbb, W.Tbb] }, apply,
      kin: { levels: [], views: { used: 70 }, warnings: [] }, table: { board_z_m: [0.0246, 0.0247], table_z_m: 0.0241 }, secs: 11.2 };
    const board = CW.makeBoard({ markerSide: 0.025 });
    const project = (pos) => {
      const G = calGripper(pos), Tcb = CW.mul(CW.inv(CW.mul(G, W.X)), W.Ttrue), R = CW.rotOf(Tcb), t = CW.posOf(Tcb), markers = [];
      for (const m of board.markers) {
        const px = m.corners.map((c) => CW.project(W.K, R, t, c));
        if (px.every(([u, v, zz]) => zz > 0 && u >= 4 && u <= 636 && v >= 4 && v <= 356)) markers.push({ id: m.id, corners: px.map(([u, v]) => [u + 0.1 * (rnd() - 0.5), v + 0.1 * (rnd() - 0.5)]) });
      }
      return markers;
    };
    window.detTriggerReq = async () => { await new Promise((r) => setTimeout(r, 30)); return { api_version: 2, success: true, frame_id: ++W.aid + 100, width: 640, height: 360, markers: project(S.obs.positions), detections: [] }; };
    const fp = (b) => (b.tool ? 'f9e8d7c6b5a49382' : 'f0e1d2c3b4a59687');
    window.commandRPC = async (p, body) => {
      W.calls.push([p, body]);
      if (p === '/api/v1/motion/stop') { if (W.job && W.job.state === 'running') { W.job.state = 'cancelled'; W.job.code = 'STOPPED'; } return {}; }
      if (p === '/api/v1/torque') { S.obs.is_torqued = !!body.enable; return {}; }
      if (p === '/api/v1/action/gripper') { setTimeout(() => { S.obs.gripper.fraction = body.command === 'close' ? 0.01 : 1; }, 150); return { action_id: ++W.aid }; }
      if (p === '/api/v1/action/move_to') {
        const id = ++W.aid, T = CW.make(CW.quatR(body.orientation.quat), body.position), s = cwIk(T, cwModelQ(S.obs.positions));
        if (!s.ok) throw Object.assign(new Error('IK unreachable'), { code: 'IK_UNREACHABLE' });
        S.obs.action = { id, kind: 'move_to', state: 'running' };
        setTimeout(() => { S.obs.positions = [...toMotor6(s.q), 0]; S.obs.action = { id, kind: 'move_to', state: 'done', code: null }; }, 60);
        return { action_id: id };
      }
      if (p === '/api/v1/motion/unload') { const id = ++W.aid; setTimeout(() => { S.obs.is_torqued = false; S.obs.action = { id, kind: 'unload', state: 'done' }; }, 200); return { action_id: id }; }
      if (p === '/api/v1/calibration/apply') {
        const before = { vision_intrinsics: { fx: 824, fy: 824, cx: 320, cy: 180, k1: 0, k2: 0, p1: 0, p2: 0, k3: 0, width: 640, height: 360, revision: 3 },
          camera_mount: { flip: false, rot_deg: 0, tilt_deg: -38, offset_m: null, q_end_cam: null }, x_fingerprint: '0000000000000000', fingerprint: '0000000000000000',
          grasp: { fx_px: 824, fy_px: 824, cx_px: null, cy_px: null, image_size_px: [640, 360], table_z_m: 0, camera_tilt_deg: -38, grip_pixel_norm: [0.5, 0.82] }, table_z_m: null, table_z_source: null };
        const i = body.intrinsics, m = body.camera_mount, tilt = CW.rpyDeg(CW.quatR(m.q_end_cam))[0];
        const after = { vision_intrinsics: { ...before.vision_intrinsics, ...i }, camera_mount: { ...before.camera_mount, offset_m: m.offset_m, q_end_cam: m.q_end_cam, tilt_deg: tilt },
          grasp: { ...before.grasp, fx_px: i.fx, fy_px: i.fy, cx_px: i.cx, cy_px: i.cy, table_z_m: body.table_z_m, camera_tilt_deg: tilt },
          x_fingerprint: 'x1a2b3c4d5e6f708', fingerprint: fp(body), table_z_m: body.table_z_m, table_z_source: body.table_z_source || 'camera',
          tool: body.tool ? { ...body.tool, edge_to_low_m: 0.0023 } : null, acceptance: body.acceptance ? { passed: body.acceptance.passed, fingerprint: body.acceptance.fingerprint } : null,
          floor: { profile_floor_z_m: 0, effective_floor_z_m: Math.max(0, body.table_z_m - 0.003), source: body.table_z_source === 'hover' ? 'calibration_hover' : 'calibration_camera' } };
        if (body.dry_run) return { dry_run: true, before, after: { ...after, grip_pixel_model: [0.507, 1.184] } };
        W.applied = body;
        return { backup: '/userdata/rebot-teleop/calibration-backups/20261120T101500Z', ...after, vision_intrinsics: { ...after.vision_intrinsics, revision: 4 } };
      }
      if (p === '/api/v1/action/calib_hover') {
        const resume = body.resume;
        if (resume) { W.job.state = 'running'; W.job.code = null; W.job.runs++; W.job.points.forEach((x) => { if (x.state !== 'done') x.state = 'pending'; }); }
        else W.job = { id: 'hv-' + (++W.aid).toString(16).padStart(8, '0'), candidate_fingerprint: body.candidate_fingerprint, state: 'running', code: null, runs: 1,
          points: body.points.map((x) => ({ ...x, state: 'pending', observations: [], discarded: [] })) };
        const job = W.job;
        const step = () => {
          if (job !== W.job || job.state !== 'running') return;
          const pt = job.points.find((x) => x.state === 'pending');
          if (!pt) { job.state = 'done'; S.obs.action = { id: 1, kind: 'calib_hover', state: 'done' }; return; }
          if (W.drop && job.points.filter((x) => x.state === 'done').length === 2) {
            job.state = 'cancelled'; job.code = 'LEASE_LOST'; S.obs.is_torqued = false; W.drop = false; return; }
          const T = CW.make(CW.rpyR(...pt.rpy_rad), pt.hover_m), s = cwIk(T, cwModelQ(S.obs.positions));
          if (!s.ok) { pt.state = 'skipped'; pt.code = 'IK_UNREACHABLE'; pt.message = 'no IK solution'; }
          else {
            S.obs.positions = [...toMotor6(s.q), 0];
            S.obs.action = { id: 1, kind: 'calib_hover', state: 'running', phase: 'capture:' + pt.id };
            pt.observations = [0, 1, 2].map((k) => ({ frame_id: 200 + k, markers: project(S.obs.positions), joints: S.obs.positions.slice(), image_size: [640, 360] }));
            pt.discarded = [198, 199]; pt.settled_joints = S.obs.positions.slice(); pt.state = 'done';
          }
          setTimeout(step, 350);
        };
        setTimeout(step, 350);
        return { action_id: 1, job_id: job.id, expected_duration_s: 120 };
      }
      return {};
    };
    const realFetch = window.fetch.bind(window);
    const json = (o) => new Response(JSON.stringify(o), { status: 200, headers: { 'content-type': 'application/json' } });
    window.fetch = async (u, opt) => {
      const s = String(u);
      if (s.startsWith('/api/v1/calibration/hover/')) return json({ ...W.job, arm: { is_torqued: S.obs.is_torqued, safety_state: 'Armed', fault: null, watchdog_park: false } });
      if (s === '/api/v1/calibration') {
        const a = W.applied;
        return json({ calibrated: !!a, busy: false, camera_mount: { flip: false, rot_deg: 0, tilt_deg: -38 }, backups: [], calibration_tool: { tool_id: 'b601dm_gripper', edge_to_low_m: 0.0023 },
          calibration: a ? { table_z_m: a.table_z_m, table_z_source: a.table_z_source, acceptance: a.acceptance } : null,
          floor: a ? { profile_floor_z_m: 0, effective_floor_z_m: a.table_z_m - 0.003, source: a.table_z_source === 'hover' ? 'calibration_hover' : 'calibration_camera' } : null });
      }
      if (s === '/api/v1/camera/snapshot') {
        if (W.snap === 'fixture') return realFetch('/fx/snapshot.jpg', opt);
        const c = document.createElement('canvas'); c.width = 640; c.height = 640;
        const g = c.getContext('2d'); g.fillStyle = '#9a9a9a'; g.fillRect(0, 0, 640, 640); g.fillStyle = '#6b6b6b'; g.fillRect(250, 430, 40, 12);
        const blob = await new Promise((r) => c.toBlob(r, 'image/jpeg', 0.9));
        return new Response(blob, { status: 200, headers: { 'content-type': 'image/jpeg' } });
      }
      return realFetch(u, opt);
    };
    S.detections = Array.from({ length: 8 }, (_, i) => ({ aruco: { id: i, t: [0, 0, 0.3] } })); S.detLastReceivedAt = performance.now(); S.detHoldUntil = performance.now() + 1e7;
    return !!q0;
  });
}

const VIEWS = [['desktop', { width: 1400, height: 900 }, 'expert'], ['phone', { width: 390, height: 844 }, 'simple']];
for (const [vpName, viewport, ui] of VIEWS) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'desktop' ? 1 : 2 });
    await ctx.addInitScript(([l]) => {
      try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {}
      const RealWS = window.WebSocket;
      window.WebSocket = function (u, p) {
        if (!String(u).includes('/api/v1/ws')) return new RealWS(u, p);
        return { readyState: 1, url: u, send() {}, close() {}, addEventListener() {} };
      };
      window.WebSocket.OPEN = 1;
    }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(e.message));
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(500);
    if (!(await page.evaluate(setupVerify, [ui, fixtures.intrinsics]))) fail('sim start pose');
    const tag = `${lang}/${vpName}`;
    const shot = async (name, extra = '') => {
      await page.waitForTimeout(250);
      const info = await page.evaluate(() => ({
        open: !$('cwiz').hidden, on: [...document.querySelectorAll('#cwiz-steps li.on')].map((x) => x.textContent).join(),
        raw: /\bcw\.[a-z]/.test($('cwiz-panel').innerText), hscroll: $('cwiz-panel').scrollWidth > $('cwiz-panel').clientWidth + 1 || document.documentElement.scrollWidth > window.innerWidth + 1,
      }));
      if (!info.open || info.raw || info.hscroll) fail(`${name} ${tag}: ${JSON.stringify(info)}`);
      const rows = await page.evaluate(actRowsCheck, '#cwiz-panel');
      if (rows.length) fail(`action rows ${name} ${tag}: ${rows.join(' | ')}`);
      const file = path.join(out, `cwv_${name}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(`${path.relative(root, file)}  step="${info.on}" ${extra}`);
    };
    const waitFor = async (fn, ms, label) => {
      try { await page.waitForFunction(fn, null, { timeout: ms }); }
      catch (e) { const st = await page.evaluate(() => ({ step: CWZ.step, busy: CWZ.busy, status: CWZ.status, tip: CWV.tip && CWV.tip.ok, job: CWV.job && CWV.job.state })); fail(`${label} ${tag}: ${JSON.stringify(st)}`); }
    };
    await page.evaluate(() => cwOpen(true));
    await page.waitForTimeout(500);
    // board step: the sticker page is reachable here
    if (!(await page.evaluate(() => !!$('cwiz-sticker-b')))) fail('board step without the sticker button ' + tag);
    const svgOk = await page.evaluate(() => { const s = CWZ.CW.stickerSvg(); return (s.match(/<circle/g) || []).length === 16; });
    if (!svgOk) fail('sticker svg');
    await shot('1_board');
    // fingertip: not found -> ROI guidance
    await page.evaluate(() => { CWZ.step = 'tip'; __V.snap = 'blank'; cwRender(); });
    await shot('2_tip');
    // gripper open: the sticker spans both closed fingers -> refuse, offer "close the gripper"
    await page.evaluate(() => { S.obs.gripper.fraction = 0.6; cwRender(); });
    await page.click('#cwiz-tip-measure');
    await page.waitForTimeout(200);
    if (!(await page.evaluate(() => !!$('cwiz-grip-close') && !CWV.tip && !__V.calls.some((c) => c[0] === '/api/v1/calibration/apply')))) fail('tip with an open gripper ' + tag);
    await shot('2b_tip_gripper_open');
    await page.click('#cwiz-grip-close');
    await waitFor(() => !CWZ.busy && S.obs.gripper.fraction < 0.15 && !$('cwiz-grip-close'), 8000, 'close gripper');
    await page.click('#cwiz-tip-measure');
    await waitFor(() => !CWV.tipBusy && CWV.tip && !CWV.tip.ok, 20000, 'tip not found');
    if (!(await page.evaluate(() => !!$('cwiz-tip-roi') && /Align the sticker edge with the fingertip|将贴纸底边与指尖对齐/.test($('cwiz-panel').innerText)))) fail('not-found guidance ' + tag);
    await shot('3_tip_notfound');
    // found: the real JPEGs
    await page.evaluate(() => { __V.snap = 'fixture'; });
    await page.click('#cwiz-tip-measure');
    await waitFor(() => !CWV.tipBusy && CWV.tip && CWV.tip.ok && CWV.fp, 30000, 'tip measured');
    const tip = await page.evaluate(() => CWV.tip && CWV.tip.tip_cam_m);
    const ref = fixtures.sticker.tip_cam_mm;
    if (!tip || Math.hypot(...tip.map((v, k) => v * 1000 - ref[k])) > 0.3) fail(`tip ${tag}: ${tip && tip.map((v) => (v * 1000).toFixed(2))} vs ${ref.map((v) => v.toFixed(2))}`);
    if (!(await page.evaluate(() => { const e = $('cwiz-tip-remove'); return !!e && /撕下贴纸|remove the sticker/.test(e.textContent); }))) fail('no remove-sticker message ' + tag);
    await page.evaluate(() => $('cwiz-tip-remove').scrollIntoView({ block: 'center' }));
    await shot('4_tip_ok', `tip ${tip && tip.map((v) => (v * 1000).toFixed(2)).join(', ')} mm`);
    // hover: plan, run (Stop visible), result
    await page.click('#cwiz-tip-next');
    await page.click('#cwiz-hov-plan');
    await waitFor(() => !CWZ.busy && CWV.plan, 20000, 'plan');
    const plan = await page.evaluate(() => ({ ok: CWV.plan.ok, n: CWV.plan.points.length, why: CWV.plan.why, c: CWV.plan.candidates, corners: CWV.plan.points.map((p) => [...p.corner.map((v) => Math.round(v * 1000)), p.yaw_deg]) }));
    if (!plan.ok) fail(`plan ${tag}: ${JSON.stringify(plan)}`);
    // no sticker in the hover step: no print button, no sticker placement text
    if (await page.evaluate(() => !!$('cwiz-sticker') || /贴在|Stick the printed/.test($('cwiz-panel').innerText))) fail('hover asks for the sticker ' + tag);
    await shot('5_hover_plan', `${plan.n} corners`);
    await page.click('#cwiz-hov-start');
    await waitFor(() => CWV.job && CWV.job.state === 'running' && CWV.job.points && CWV.job.points.some((p) => p.state === 'done'), 15000, 'running');
    const stopVis = await page.evaluate(() => { const b = $('cwiz-stop'), r = b.getBoundingClientRect(); return !b.hidden && r.width > 0 && r.top >= 0 && r.bottom <= window.innerHeight; });
    if (!stopVis) fail('Stop not visible while hovering ' + tag);
    await shot('6_hover_running');
    await waitFor(() => CWV.job && CWV.job.state === 'done' && CWV.eval, 20000, 'hover done');
    const ev = await page.evaluate(() => ({ ok: CWV.eval.ok, z: CWV.eval.table_z_m, spread: CWV.eval.height_spread_m, xy: CWV.eval.max_xy_m, n: CWV.eval.points }));
    if (!ev.ok) fail(`eval ${tag}: ${JSON.stringify(ev)}`);
    await shot('7_hover_result', `table_z ${(ev.z * 1000).toFixed(1)} mm, spread ${(ev.spread * 1000).toFixed(2)} mm, xy ${(ev.xy * 1000).toFixed(2)} mm, ${ev.n} points`);
    await page.click('#cwiz-hov-confirm');
    await waitFor(() => CWV.confirm, 20000, 'confirm');
    const cf = await page.evaluate(() => CWV.confirm && { ok: CWV.confirm.ok, w: CWV.confirm.worst_m });
    if (!cf || !cf.ok) fail(`confirm ${tag}: ${JSON.stringify(cf)}`);
    await shot('8_hover_confirmed', `confirm worst ${cf && (cf.w * 1000).toFixed(2)} mm`);
    // review: park and turn torque off, then apply once
    await page.click('#cwiz-review');
    await waitFor(() => CWZ.step === 'review' && CWZ.dry, 10000, 'review');
    await shot('9_review_torqued');
    await page.click('#cwiz-park');
    await waitFor(() => !CWZ.busy && !S.obs.is_torqued, 10000, 'park');
    await page.click('#cwiz-apply'); await page.click('#cwiz-apply');
    await waitFor(() => CWZ.step === 'applied' && CWZ.readback, 10000, 'applied');
    const ab = await page.evaluate(() => ({ src: __V.applied && __V.applied.table_z_source, z: __V.applied && __V.applied.table_z_m, acc: __V.applied && __V.applied.acceptance && __V.applied.acceptance.passed, tool: __V.applied && __V.applied.tool && __V.applied.tool.tool_id, rb: CWZ.readback && CWZ.readback.ok,
      applies: __V.calls.filter((c) => c[0] === '/api/v1/calibration/apply' && !c[1].dry_run).length }));
    if (ab.src !== 'hover' || !ab.acc || ab.tool !== 'b601dm_gripper' || !ab.rb || ab.applies !== 1) fail(`apply body ${tag}: ${JSON.stringify(ab)}`);
    await shot('10_applied', `table_z ${(ab.z * 1000).toFixed(1)} mm source ${ab.src}`);
    // an interrupted job (page lost the lease): cancelled LEASE_LOST, torque off, resume
    await page.evaluate(() => { CWZ.step = 'hover'; CWV.eval = null; CWV.confirm = null; CWV.job = null; CWZ.status = ''; S.obs.is_torqued = true; __V.drop = true; cwRender(); });
    await page.click('#cwiz-hov-start');
    await waitFor(() => CWV.job && CWV.job.state === 'cancelled', 15000, 'interrupted');
    await page.waitForTimeout(300);
    if (!(await page.evaluate(() => !!$('cwiz-hov-resume')))) fail('no resume button ' + tag);
    if (await page.evaluate(() => /贴在|Stick the printed/.test($('cwiz-panel').innerText))) fail('resume asks for the sticker ' + tag);
    await shot('11_hover_interrupted');
    // the gripper opened meanwhile: resume is refused until it is closed again
    await page.evaluate(() => { S.obs.gripper.fraction = 0.5; cwRender(); });
    const nCalls = await page.evaluate(() => __V.calls.filter((c) => c[0] === '/api/v1/action/calib_hover').length);
    await page.click('#cwiz-hov-resume');
    await page.waitForTimeout(300);
    const gr = await page.evaluate((n) => ({ box: !!$('cwiz-grip-close'), sent: __V.calls.filter((c) => c[0] === '/api/v1/action/calib_hover').length - n, st: CWZ.status }), nCalls);
    if (!gr.box || gr.sent !== 0) fail(`resume with the gripper open ${tag}: ${JSON.stringify(gr)}`);
    await shot('11b_hover_gripper_open');
    await page.click('#cwiz-grip-close');
    await waitFor(() => !CWZ.busy && S.obs.gripper.fraction < 0.15, 8000, 'close gripper (hover)');
    await page.click('#cwiz-hov-resume');
    await waitFor(() => CWV.job && CWV.job.state === 'done' && CWV.eval, 20000, 'resumed');
    const rs = await page.evaluate(() => ({ runs: CWV.job.runs, ok: CWV.eval.ok, body: __V.calls.filter((c) => c[0] === '/api/v1/action/calib_hover').pop()[1] }));
    if (rs.runs !== 2 || !rs.ok || !rs.body.resume) fail(`resume ${tag}: ${JSON.stringify(rs)}`);
    await shot('12_hover_resumed');
    if (errs.length) fail(`page errors ${tag}: ${errs.join(' | ')}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(`${shots.length} screenshots in ${path.relative(root, out)}`);
console.log(bad ? `${bad} check(s) FAILED` : 'all checks passed');
process.exit(bad ? 1 : 0);
