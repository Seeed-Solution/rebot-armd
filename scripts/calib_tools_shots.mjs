#!/usr/bin/env node
// Headless screenshots of the calibration tools (no armd needed): the Tools group of the settings
// drawer (entries from capabilities.calibration_tools, stale camera calibration warning, release
// control), the stale chip on the video and the gripper tool.
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_tools_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and mocks of GET /api/v1/calibration
// (with `stale`) and GET /api/v1/gripper/calibration; the control WebSocket is a stub and the page state
// (capabilities, observation, lease) is set in the page. zh / en at desktop 1400x900 and phone 390x844;
// PNGs go to .cache/calib-tools-shots/. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-tools-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));

// capabilities.calibration_tools as armd reports it for the B601-DM (mock follower)
const TOOLS = {
  camera: { supported: true },
  kinematics: { supported: true, joints: [2, 3, 4, 5] },
  motor_zero: {
    supported: true, method: 'set_current', persist: 'motor_flash', persist_verified: false,
    notes: ['Damiao set-zero frame', 'Not verified on hardware whether the zero survives a power cycle'],
    joints: [1, 2, 3, 4, 5, 6, 7], requires: ['lease', 'torque_off', 'no_action', 'no_fault'], max_offset_deg: 45, verify_mrad: 10,
    invalidates: ['kinematics', 'camera'],
    zero_pose: {
      description: 'Folded like the home pose: upper arm level pointing back from the shoulder, forearm folded back over it, gripper level pointing straight forward (tool about 0.26 m in front of the base, 0.19 m high).',
      joints: ['J1 base: arm plane facing forward (+x)', 'J2 shoulder: upper arm level, pointing back', 'J3 elbow: forearm folded back over the upper arm',
        'J4 wrist pitch: gripper level', 'J5 wrist yaw: gripper pointing straight forward', 'J6 wrist roll: motor 0, jaws open up/down (the home pose has J6 at +90 deg, jaws open sideways)'],
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
  if (u === '/api/v1/calibration') return send(res, 200, { calibrated: true, busy: false, stale: STALE, calibration_tools: TOOLS, backups: [] });
  if (u === '/api/v1/gripper/calibration') return send(res, 200, { offset_rad: 0.012, calibrated: false });
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
    }));
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
    if (errs.length) fail(`page errors ${lang}/${vpName}: ${errs.join(' | ')}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
