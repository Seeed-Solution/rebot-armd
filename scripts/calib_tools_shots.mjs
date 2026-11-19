#!/usr/bin/env node
// Headless screenshots of the calibration tools (no armd needed): the Tools group of the settings
// drawer (entries from capabilities.calibration_tools, stale camera calibration warning, legacy J6 zero
// warning with its button, stale poses warning, release control), the stale banner in the cockpit, the
// gripper tool and the zero tool (park while torqued, the 3D view with the rest pose as the ghost, the
// method choice, joint table with a joint > 45 deg from the rest pose and the force box, two-tap confirm
// against stubbed dry-run / run replies, result with the invalidated calibrations and the undo).
// The 3D view loads src/api/vendor/three-urdf.min.js.gz and /assets/model/* from MODEL_DIR (default the
// recamera-b601dm rootfs assets; missing files give the stick figure).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_tools_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and mocks of GET /api/v1/calibration
// (with `stale`) and GET /api/v1/gripper/calibration; the control WebSocket is a stub and the page state
// (capabilities, observation, lease) is set in the page. zh / en at desktop 1400x900 and phone 390x844;
// PNGs go to .cache/calib-tools-shots/. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { actRowsCheck } from './act_rows_check.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-tools-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));
const bundle = fs.readFileSync(path.join(root, 'src/api/vendor/three-urdf.min.js.gz'));
const modelDir = process.env.MODEL_DIR || path.join(root, 'kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets');

// capabilities.calibration_tools as armd reports it for the B601-DM (mock follower)
const TOOLS = {
  camera: { supported: true },
  kinematics: { supported: true, joints: [2, 3, 4, 5] },
  motor_zero: {
    supported: true, method: 'set_current', persist: 'motor_flash', persist_verified: false,
    notes: ['Damiao set-zero frame', 'Not verified on hardware whether the zero survives a power cycle'],
    joints: [1, 2, 3, 4, 5, 6, 7], requires: ['lease', 'torque_off', 'no_action', 'no_fault'], max_offset_deg: 45, verify_mrad: 10,
    invalidates: ['kinematics', 'camera', 'poses', 'gripper_calibration'], motor: true, software: true, methods: ['software', 'motor'],
    zero_pose: {
      description: 'The folded rest pose (the home and park pose): upper arm level pointing back from the shoulder, forearm folded back over it, gripper level pointing straight forward, jaws opening sideways, gripper fully closed (tool about 0.26 m in front of the base, 0.19 m high). Every joint reads 0 here after the zero.',
      joints: ['J1 base: arm plane facing forward (+x)', 'J2 shoulder: upper arm level, pointing back', 'J3 elbow: forearm folded back over the upper arm',
        'J4 wrist pitch: gripper level', 'J5 wrist yaw: gripper pointing straight forward', 'J6 wrist roll: as folded in the rest pose, jaws opening sideways, camera holder on top'],
      gripper: 'Gripper fully closed',
    },
  },
  gripper: { supported: true, zero_check: true, calibration: true },
};
const STALE = { reason: 'motor zero of joints [2, 3]; the camera calibration was measured with the old zero', at: '2026-11-18T10:20:00Z' };
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(js); return; }
  if (u === '/api/v1/calibration') return send(res, 200, { calibrated: true, busy: false, stale: STALE, calibration_tools: TOOLS, backups: [],
    zero_convention: { status: 'legacy_j6', j6_rad: 1.574, message: 'J6 reads 90.2 deg in the rest pose' },
    stale_list: [{ item: 'camera', ...STALE }, { item: 'poses', reason: 'motor zero of joints [2, 3]; taught with the old zero', at: STALE.at, names: ['scan', 'bin_a'] }] });
  if (u === '/api/v1/gripper/calibration') return send(res, 200, { offset_rad: 0.012, calibrated: false });
  if (u === '/assets/three-urdf.min.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8', 'content-encoding': 'gzip' }); res.end(bundle); return; }
  if (u.startsWith('/assets/model/')) {
    const rel = u.slice('/assets/model/'.length), f = path.join(modelDir, rel);
    if (rel.includes('..') || !fs.existsSync(f) || !fs.statSync(f).isFile()) { res.writeHead(404); res.end(); return; }
    res.writeHead(200); res.end(fs.readFileSync(f)); return;
  }
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
let bad = 0;
const shots = [];
const fail = (m) => { bad++; console.log('  FAIL ' + m); };

// page state: B601-DM, this page holds the lease, torque off, the arm at a pose near zero
function setup(tools) {
  S.dof = 7; S.armDof = 6; S.profileId = null; S.state = 'Idle';
  S.caps = { ...(S.caps || {}), calibration_apply: true, calibration_kinematics: true, collision_guard: false, calibration_tools: tools };
  S.leaseGeneration = 1; S.epoch = 1;
  S.lastIntentAt = Date.now() + 1e9; // keep the idle release (CFG.leaseIdleMs) out of the way
  window.__sent = [];
  S.ws = { readyState: 1, send(m) { window.__sent.push(JSON.parse(m)); }, close() {}, addEventListener() {} };
  S.obs = { positions: [0.02, -0.05, 0.08, 0.01, -0.03, 0.9, 0.02], velocities: [0, 0, 0, 0, 0, 0, 0], is_torqued: false, safety_state: 'Idle', epoch_id: 1,
    fault: null, action: null, previous_action: null, gripper_zero: { state: 'ok', j7: 0.02, offset_rad: 0.012 } };
  clearInterval(window.__tick); window.__tick = setInterval(() => { S.obsAt = Date.now(); }, 50);
  colOnCaps();
}

// viewports: desktop, phone 390x844 and the WeChat-like 360x640 / 375x667; VIEWS=desktop,phone picks a subset
const ALL_VIEWS = [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }], ['wx360', { width: 360, height: 640 }], ['wx375', { width: 375, height: 667 }]];
for (const [vpName, viewport] of (process.env.VIEWS ? ALL_VIEWS.filter(([n]) => process.env.VIEWS.split(',').includes(n)) : ALL_VIEWS)) {
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
    await page.evaluate(setup, TOOLS);
    await page.waitForTimeout(400);
    const shot = async (name, panel) => {
      await page.waitForTimeout(300);
      const info = await page.evaluate((id) => {
        const el = document.getElementById(id);
        return { open: !!el && !el.closest('[hidden]'), raw: el ? /\b(tools|gt|mz)\.[a-z]/.test(el.innerText) : true,
          hscroll: el ? el.scrollWidth > el.clientWidth + 1 : true, page: document.documentElement.scrollWidth > window.innerWidth + 1 };
      }, panel);
      if (!info.open || info.raw || info.hscroll || info.page) fail(`${name} ${lang}/${vpName}: ${JSON.stringify(info)}`);
      const rows = await page.evaluate(actRowsCheck, '#mzero-panel, #gtool, #sset');
      if (rows.length) fail(`action rows ${name} ${lang}/${vpName}: ${rows.join(' | ')}`);
      const file = path.join(out, `tools_${name}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(path.relative(root, file));
    };
    // stale chip on the video
    const chip = await page.evaluate(() => !$('cal-stale').hidden && $('cal-stale-open').textContent.length > 0);
    if (!chip) fail(`stale chip ${lang}/${vpName}`);
    await shot('0_cockpit_stale', 'cal-stale');
    // 1 Tools group (stale warning, entries, release enabled)
    await page.evaluate(() => ssetOpen(true));
    const g = await page.evaluate(() => ({
      entries: [...document.querySelectorAll('#tools-list .tool-item')].map((b) => b.id),
      stale: !$('tools-stale').hidden, rel: !$('tools-release').disabled, btn: !$('btn-sset').hidden,
      legacy: !$('tools-zero').hidden && $('tools-zero').textContent.includes('90.2') && !!$('tools-zero-open'),
      poses: !$('tools-stale2').hidden && $('tools-stale2').textContent.includes('scan, bin_a'),
    }));
    if (!g.legacy || !g.poses) fail(`legacy J6 / stale poses warnings ${lang}/${vpName}: ${JSON.stringify(g)}`);
    const want = ['tool-camera', 'tool-kinematics', 'tool-gripper'];
    if (await page.evaluate(() => typeof mzOpen === 'function')) want.unshift('tool-motor_zero');
    if (JSON.stringify(g.entries) !== JSON.stringify(want) || !g.stale || !g.rel || !g.btn) fail(`tools group ${lang}/${vpName}: ${JSON.stringify(g)}`);
    await shot('1_group', 'sset');
    // torque on: release disabled with the hint
    await page.evaluate(() => { S.obs.is_torqued = true; toolsRenderRelease(); });
    const dis = await page.evaluate(() => ({ d: $('tools-release').disabled, h: $('tools-release-hint').textContent }));
    if (!dis.d || !dis.h) fail(`release while torqued ${lang}/${vpName}: ${JSON.stringify(dis)}`);
    await page.evaluate(() => { $('sset').querySelector('.calib-drawer').scrollTop = 1e4; });
    await shot('2_group_torqued', 'sset');
    // release with torque off sends release_lease
    await page.evaluate(() => { S.obs.is_torqued = false; toolsRenderRelease(); $('tools-release').click(); });
    const rel = await page.evaluate(() => ({ sent: window.__sent.map((m) => m.type), gen: S.leaseGeneration }));
    if (!rel.sent.includes('release_lease') || rel.gen != null) fail(`release ${lang}/${vpName}: ${JSON.stringify(rel)}`);
    await page.evaluate(() => { S.leaseGeneration = 1; ssetOpen(false); });
    // unsupported tools are not listed
    await page.evaluate((tl) => { S.caps.calibration_tools = { ...tl, kinematics: { supported: false, reason: 'URDF chain' }, gripper: { supported: false } }; ssetOpen(true); }, TOOLS);
    const few = await page.evaluate(() => [...document.querySelectorAll('#tools-list .tool-item')].map((b) => b.id));
    if (few.includes('tool-kinematics') || few.includes('tool-gripper')) fail(`unsupported listed ${lang}/${vpName}: ${few}`);
    await page.evaluate((tl) => { S.caps.calibration_tools = tl; ssetOpen(false); }, TOOLS);
    // 3 gripper tool (torque off: check disabled with the hint), then torqued with a result
    await page.evaluate(() => gtOpen(true));
    await page.waitForTimeout(300);
    await shot('3_gripper', 'gtool');
    await page.evaluate(() => { S.obs.is_torqued = true; GT.res = { stop_rad: 0.031, offset_rad: 0.019, pass: true }; gtRender(); });
    await shot('4_gripper_result', 'gtool');
    await page.evaluate(() => gtOpen(false));
    // 5 motor zero tool, torqued: park button, the table is shown
    await page.evaluate(() => { S.obs.is_torqued = true; S.obs.positions = [0.02, -0.05, 0.08, 0.01, -0.03, 0.9, 0.02]; ssetOpen(true); $('tool-motor_zero').click(); });
    await page.waitForTimeout(1500);
    const m1 = await page.evaluate(() => ({ park: !!$('mz-park'), rows: document.querySelectorAll('#mzero-body .mz-table tr').length, go: $('mz-go')?.disabled,
      v3: V3.open, ghost: !!(MZ.previewQ && MZ.previewQ.every((v) => v === 0)),
      methods: [...document.querySelectorAll('#mzero-body input[name=mz-method]')].map((r) => r.id + (r.checked ? '*' : '')) }));
    if (!m1.park || m1.rows !== 8 || !m1.go || !m1.v3 || !m1.ghost || JSON.stringify(m1.methods) !== '["mz-m-software*","mz-m-motor"]')
      fail(`motor zero torqued ${lang}/${vpName}: ${JSON.stringify(m1)}`);
    await shot('5_mz_torqued', 'mzero-panel');
    // 6 torque off, J2 / J3 / J6 selected; J6 reads 51.6 deg: force box, Set disabled until ticked
    await page.evaluate(() => { S.obs.is_torqued = false; mzRender(); for (const j of [2, 3, 6]) $('mz-c-' + j).click(); });
    await page.waitForTimeout(400);
    const m2 = await page.evaluate(() => ({ far: !$('mz-farbox').hidden, go: $('mz-go').disabled, farCell: $('mz-r-6').classList.contains('far') }));
    if (!m2.far || !m2.go || !m2.farCell) fail(`motor zero far ${lang}/${vpName}: ${JSON.stringify(m2)}`);
    await page.evaluate(() => { $('mz-force').click(); });
    if (await page.evaluate(() => $('mz-go').disabled)) fail(`motor zero force ${lang}/${vpName}`);
    await page.evaluate(() => { $('mzero-body').scrollTop = 1e4; });
    await shot('6_mz_select', 'mzero-panel');
    // 7 first tap: dry run (stub), button armed; second tap: the run
    await page.evaluate(() => {
      window.__mz = [];
      window.commandRPC = async (p, body) => {
        window.__mz.push([p, body]);
        if (p === '/api/v1/calibration/joint_zero/restore') return { restored: body.backup, offsets_after: [0, 0, 0, 0, 0, 0, 0] };
        if (p !== '/api/v1/calibration/motor_zero') throw Object.assign(new Error('unexpected ' + p), { code: 'X' });
        if (body.dry_run) return { dry_run: true, joints: body.joints.map((j) => ({ joint: j, reading_rad: S.obs.positions[j - 1] })), needs_force: true };
        const before = body.joints.map((j) => S.obs.positions[j - 1]);
        body.joints.forEach((j) => { S.obs.positions[j - 1] = 0.0004; });
        return { joints: body.joints.map((j, k) => ({ joint: j, before_rad: before[k], after_rad: 0.0004, sent: true, fresh: true, verified: true })),
          invalidated: { kinematics_removed: true, camera_stale: true, poses_stale: ['scan', 'bin_a'], gripper_calibration_stale: false },
          zero_method: body.method, persist: 'calibration_file', persist_verified: true, torque: 'off',
          restore: { path: '/api/v1/calibration/joint_zero/restore', backup: '20261119T101500Z' } };
      };
      $('mz-go').click();
    });
    await page.waitForTimeout(400);
    const m3 = await page.evaluate(() => ({ armed: Date.now() < MZ.armedUntil, label: $('mz-go').textContent, calls: window.__mz.map((c) => c[1]) }));
    if (!m3.armed || !m3.calls.length || m3.calls[0].dry_run !== true || m3.calls[0].force !== true || m3.calls[0].method !== 'software')
      fail(`motor zero first tap ${lang}/${vpName}: ${JSON.stringify(m3)}`);
    await page.evaluate(() => { $('mzero-body').scrollTop = 1e4; });
    await shot('7_mz_confirm', 'mzero-panel');
    await page.evaluate(() => $('mz-go').click());
    await page.waitForTimeout(500);
    await page.evaluate(() => { $('mzero-body').scrollTop = 0; });
    const m4 = await page.evaluate(() => ({ res: !!MZ.res, calls: window.__mz.map((c) => c[1]), kin: !!$('mz-open-kin'), cam: !!$('mz-open-cam'),
      rows: document.querySelectorAll('#mzero-body .cwiz-table tr').length, undo: !!$('mz-undo') }));
    if (!m4.res || m4.calls.length !== 2 || m4.calls[1].dry_run !== undefined || JSON.stringify(m4.calls[1].joints) !== '[2,3,6]' || !m4.kin || !m4.cam || m4.rows !== 4 || !m4.undo)
      fail(`motor zero run ${lang}/${vpName}: ${JSON.stringify(m4)}`);
    await shot('8_mz_result', 'mzero-panel');
    await page.evaluate(() => { $('mzero-body').scrollTop = 1e4; });
    await shot('9_mz_result_end', 'mzero-panel');
    // the kinematics button opens the wizard in fine mode
    await page.evaluate(() => $('mz-open-kin').click());
    const w = await page.evaluate(() => ({ cw: CWZ.open, mode: CWZ.mode, mz: MZ.open, ghost: MZ.previewQ }));
    if (!w.cw || w.mode !== 'fine' || w.mz || w.ghost) fail(`motor zero -> fine ${lang}/${vpName}: ${JSON.stringify(w)}`);
    await page.evaluate(() => cwOpen(false));
    // undo of a software zero: WS /api/v1/calibration/joint_zero/restore with the backup name
    await page.evaluate(() => { mzOpen(true); MZ.res = { ok: true, joints: [], invalidated: {}, restore: { backup: '20261119T101500Z' } }; mzRender(); $('mz-undo').click(); });
    await page.waitForTimeout(300);
    const u = await page.evaluate(() => ({ last: window.__mz[window.__mz.length - 1], res: MZ.res, msg: $('mzero-body').textContent }));
    if (u.last[0] !== '/api/v1/calibration/joint_zero/restore' || u.last[1].backup !== '20261119T101500Z' || u.res) fail(`zero undo ${lang}/${vpName}: ${JSON.stringify(u.last)}`);
    await shot('10_mz_undone', 'mzero-panel');
    await page.evaluate(() => mzOpen(false));
    if (errs.length) fail(`page errors ${lang}/${vpName}: ${errs.join(' | ')}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
