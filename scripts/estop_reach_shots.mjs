#!/usr/bin/env node
// The E-stop stays one tap away while a drawer / sheet is open (a touch screen has no Space / Escape key), no armd needed:
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/estop_reach_shots.mjs
// For each viewport (phone 390x844, tablet landscape 1180x820, tablet portrait 820x1180, desktop 1400x900), simple and
// expert layout, and each drawer (Positions & motions, camera calibration wizard, settings, zero calibration, gripper
// calibration, the old poses drawer), the script opens the drawer and looks for the E-stop the user can hit: the cockpit
// E-stop if it is on screen and not covered (document.elementFromPoint at its centre), else the compact E-stop in the
// drawer head. It taps that point with a real pointer and checks that eStop() ran. Also: the page does not scroll
// sideways. zh / en; PNGs go to .cache/estop-reach-shots/. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { readCockpitHtmlBuffer } from './source_paths.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/estop-reach-shots');
fs.mkdirSync(out, { recursive: true });
const html = readCockpitHtmlBuffer();
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));
const POSES = [{ id: 'P1', name: 'scan', positions: [0, -0.4, 0.8, 0, 0, 0, 0] }, { id: 'P2', name: 'bin_a', positions: [0.5, -0.3, 0.7, 0, 0, 0, 0] }];
const TOOLS = {
  camera: { supported: true }, kinematics: { supported: true, joints: [2, 3, 4, 5] },
  motor_zero: { supported: true, method: 'set_current', persist: 'motor_flash', joints: [1, 2, 3, 4, 5, 6, 7], requires: ['lease', 'torque_off'], max_offset_deg: 45, verify_mrad: 10,
    invalidates: ['kinematics', 'camera', 'poses'], motor: true, software: true, methods: ['software', 'motor'], zero_pose: { description: 'rest pose', joints: [], gripper: 'closed' } },
  gripper: { supported: true, zero_check: true, calibration: true },
};
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(js); return; }
  if (u === '/api/v1/sequences') return send(res, 200, { sequences: [] });
  if (u === '/api/v1/poses') return send(res, 200, { poses: POSES });
  if (u === '/api/v1/episodes') return send(res, 200, { episodes: [], recording: null });
  if (u === '/api/v1/datasets') return send(res, 200, { datasets: [] });
  if (u === '/api/v1/safety/collision') return send(res, 200, { enabled: false, sensitivity: 'medium' });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } return c; };

function setup(tools) {
  S.dof = 7; S.armDof = 6; S.state = 'Armed';
  S.caps = { ...(S.caps || {}), poses: true, replay: true, collision_guard: true, teach: { version: 1, episodes: true, sequences: true }, calibration_tools: tools };
  S.leaseGeneration = 1; S.epoch = 1; S.lastIntentAt = Date.now() + 1e9;
  S.ws = { readyState: 1, send() {}, close() {}, addEventListener() {} };
  S.obs = { positions: [0.02, -0.4, 0.8, 0.01, -0.03, 0.1, 0.02], velocities: [0, 0, 0, 0, 0, 0, 0], is_torqued: true, enable_state: 'Enabled', safety_state: 'Armed', epoch_id: 1,
    control_mode: 'position', fault: null, action: null, previous_action: null, timestamp: 1000, gravity_comp: { supported: true, phase: null, blocker: null } };
  clearInterval(window.__tick); window.__tick = setInterval(() => { S.obsAt = Date.now(); }, 50);
  window.commandRPC = async () => ({ epoch_id: 1 });
  window.__estop = 0; window.eStop = () => { window.__estop++; };
  posesOnCaps(); teachOnCaps(); colOnCaps();
}
const DRAWERS = [
  ['teach', 'teach', () => teachOpen(true, { tab: 'pose' }), () => teachOpen(false)],
  ['cwiz', 'cwiz', () => cwOpen(true), () => cwOpen(false)],
  ['settings', 'sset', () => ssetOpen(true), () => ssetOpen(false)],
  ['zero', 'mzero', () => mzOpen(true), () => mzOpen(false)],
  ['gripper', 'gtool', () => gtOpen(true), () => gtOpen(false)],
];
const VIEWS = [['phone', 390, 844, 2], ['tablet_land', 1180, 820, 1], ['tablet_port', 820, 1180, 1], ['desktop', 1400, 900, 1]];
const summary = [];
for (const [vp, w, h, dpr] of VIEWS) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport: { width: w, height: h }, deviceScaleFactor: dpr, hasTouch: vp !== 'desktop' });
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
    for (const ui of ['simple', 'expert']) {
      await page.evaluate((ui) => setUi(ui), ui);
      await page.waitForTimeout(300);
      // the header: no sideways scroll; the 点位 button has a text label from 800 px up (the short one below 1000 px), icon only below
      const hd = await page.evaluate(() => { const b = $('btn-pset'); const l = [...b.querySelectorAll('span')].find((x) => x.offsetWidth); return { page: document.documentElement.scrollWidth > innerWidth + 1, lbl: !!l, txt: l ? l.textContent : '' }; });
      ok(!hd.page, `${vp} ${lang} ${ui}: no horizontal scroll`);
      ok(hd.lbl === (w >= 800), `${vp} ${lang} ${ui}: 点位 label ${hd.lbl ? 'shown' : 'hidden'} "${hd.txt}"`);
      for (const [name, id, open, close] of DRAWERS) {
        let opened = true;
        try { await page.evaluate(`(${open.toString()})()`); } catch (e) { opened = false; console.log(`  note ${vp} ${lang} ${ui} ${name}: open failed ${String(e).slice(0, 120)}`); }
        await page.waitForTimeout(450);
        const r = await page.evaluate((id) => {
          const mask = $(id), drawer = mask.querySelector('.calib-drawer');
          const center = (el) => { const b = el.getBoundingClientRect(); return { x: b.left + b.width / 2, y: b.top + b.height / 2, w: b.width, h: b.height }; };
          const reach = (el) => { const c = center(el); if (!(c.w > 0 && c.h > 0) || c.x < 0 || c.y < 0 || c.x >= innerWidth || c.y >= innerHeight) return null; const at = document.elementFromPoint(c.x, c.y); return at && el.contains(at) ? c : null; };
          const real = reach($('btn-estop'));
          const pin = drawer.querySelector('.head-estop');
          const pinAt = pin && !pin.hidden ? reach(pin) : null;
          return { open: !mask.hidden, covered: !real, real, pin: pinAt, pinShown: !!pin && !pin.hidden };
        }, id);
        const target = r.real || r.pin;
        let hit = 0;
        if (target) {
          await page.evaluate(() => { window.__estop = 0; });
          if (vp === 'desktop') await page.mouse.click(target.x, target.y); else await page.touchscreen.tap(target.x, target.y);
          await page.waitForTimeout(100);
          hit = await page.evaluate(() => window.__estop);
        }
        const via = r.real ? 'cockpit E-stop' : r.pin ? 'head E-stop' : 'none';
        summary.push(`${vp.padEnd(11)} ${lang} ${ui.padEnd(6)} ${name.padEnd(8)} cockpit E-stop ${r.covered ? 'covered/off screen' : 'reachable'} -> ${via}, tap ${hit ? 'ran eStop' : 'did nothing'}`);
        ok(opened && r.open, `${vp} ${lang} ${ui} ${name}: drawer open`);
        ok(!!target && hit >= 1, `${vp} ${lang} ${ui} ${name}: an E-stop is visible and a tap runs it (${via}, ${hit})`);
        ok(!(r.real && r.pinShown), `${vp} ${lang} ${ui} ${name}: no duplicate E-stop while the cockpit one is reachable`);
        if (lang === 'zh' || name === 'teach') await page.screenshot({ path: path.join(out, `estop_${name}_${ui}_${lang}_${vp}.png`) });
        try { await page.evaluate(`(${close.toString()})()`); } catch (_) {}
        await page.waitForTimeout(150);
      }
    }
    ok(errs.length === 0, `${vp} ${lang}: page errors ${errs.join(' | ')}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
for (const l of summary) console.log(l);
console.log(bad ? `${bad}/${checks} checks FAILED` : `${checks} checks OK`);
process.exit(bad ? 1 : 0);
