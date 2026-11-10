#!/usr/bin/env node
// Headless check of the motion_limit hint chip in the cockpit (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/motion_limit_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF schema and feeds synthetic observations carrying
// observation.motion_limit (docs/control_safety_contract.md "Servo streaming guard"). For simple / expert
// x zh / en x 1280x800 / 375x812: joint_limit, blocked and workspace show at once with the joint name;
// speed / lead stay hidden for the first second and then show; the chip hides again about 1 s after the
// limit clears; the reworded ik.lag text has no "safely stopped". Writes PNGs (default .cache/motion-limit-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/motion-limit-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const json = (res, o) => { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
let bad = 0, checks = 0;
const fail = (m) => { bad++; console.log('  FAIL ' + m); };
const ok = (c, m) => { checks++; if (!c) fail(m); };
ok(/renderActionChip\(o\);\s*renderMotionLimit\(o\);/.test(String(html)), 'the telemetry handler calls renderMotionLimit(o)');

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
// synthetic telemetry at 50 Hz on the observation clock; window.__ml is the motion_limit to publish
const feed = () => {
  const t0 = performance.now();
  window.__ml = null;
  window.__mlSince = 0;
  setInterval(() => {
    const ts = (performance.now() - t0) / 1000;
    const ml = window.__ml ? { ...window.__ml, since_ms: window.__mlSince } : null;
    const o = { timestamp: ts, positions: [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0], velocities: [0, 0, 0, 0, 0, 0, 0],
      commanded_positions: [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0], safety_state: 'Armed', enable_state: 'Enabled',
      is_torqued: true, feedback_age_ms: [5, 5, 5, 5, 5, 5, 5], epoch_id: 1, motion_limit: ml, motor_errors: [] };
    S.obs = o; S.obsAt = Date.now(); S.state = 'Armed'; S.epoch = 1;
    renderMotionLimit(o);
  }, 20);
  window.__setMl = (ml) => { window.__ml = ml; window.__mlSince = Math.round(performance.now() - t0); };
};
const EXPECT = {
  zh: { joint_limit: 'J3 已到限位，该方向已停止', blocked: 'J2 受阻，已停在当前位置', workspace: '已到工作范围边界，该方向已停止', slow: '大关节跟不上，已减速', lag: '电机跟不上，已暂停该方向' },
  en: { joint_limit: 'J3 at its limit; this direction stopped', blocked: 'J2 blocked; holding position', workspace: 'At the workspace boundary; this direction stopped', slow: 'Slowed to what the motors can follow', lag: "Motors can't keep up; paused this direction" },
};
const shots = [];
for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); localStorage.setItem('rebot.ctl', 'cart'); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(300);
  const chip = () => page.evaluate(() => {
    const el = document.getElementById('ml-chip'); const r = el.getBoundingClientRect();
    return { hidden: el.hidden, visible: !el.hidden && r.width > 0 && r.height > 0, text: el.textContent,
      inView: r.left >= 0 && r.right <= window.innerWidth && r.top >= 0 && r.bottom <= window.innerHeight,
      sw: document.scrollingElement.scrollWidth, iw: window.innerWidth };
  });
  const tag = `${ui}_${lang}_${vp}`;
  const E = EXPECT[lang];
  let c = await chip();
  ok(c.hidden, `${tag} hidden without a limit`);
  for (const [reason, joint, text] of [['joint_limit', 2, E.joint_limit], ['workspace', null, E.workspace], ['blocked', 1, E.blocked]]) {
    await page.evaluate(([r, j]) => window.__setMl({ reason: r, joint: j }), [reason, joint]);
    await page.waitForTimeout(120);
    c = await chip();
    ok(c.visible && c.text === text, `${tag} ${reason}: "${c.text}" visible=${c.visible}`);
    ok(c.inView, `${tag} ${reason} chip inside the viewport`);
    ok(c.sw <= c.iw, `${tag} no horizontal scroll (${c.sw} > ${c.iw})`);
    if (reason === 'blocked') {
      const file = path.join(out, `motion_limit_${tag}_blocked.png`);
      await page.screenshot({ path: file }); shots.push(file);
    }
  }
  // clears about 1 s after the limit ends
  await page.evaluate(() => window.__setMl(null));
  await page.waitForTimeout(400);
  ok((await chip()).visible, `${tag} lingers right after the limit clears`);
  await page.waitForTimeout(900);
  ok((await chip()).hidden, `${tag} hidden 1.3 s after the limit cleared`);
  // speed / lead: hidden for the first second, then shown
  for (const reason of ['speed', 'lead']) {
    await page.evaluate((r) => window.__setMl({ reason: r, joint: 1 }), reason);
    await page.waitForTimeout(500);
    ok((await chip()).hidden, `${tag} ${reason} hidden before 1 s`);
    await page.waitForTimeout(700);
    c = await chip();
    ok(c.visible && c.text === E.slow, `${tag} ${reason} after 1 s: "${c.text}"`);
    if (reason === 'lead' && vp === '375x812') {
      const file = path.join(out, `motion_limit_${tag}_lead.png`);
      await page.screenshot({ path: file }); shots.push(file);
    }
    await page.evaluate(() => window.__setMl(null));
    await page.waitForTimeout(1300);
  }
  // a short speed limit (< 1 s) never shows
  await page.evaluate(() => window.__setMl({ reason: 'speed', joint: 0 }));
  await page.waitForTimeout(400);
  await page.evaluate(() => window.__setMl(null));
  await page.waitForTimeout(800);
  ok((await chip()).hidden, `${tag} a 0.4 s speed limit stays hidden`);
  const lag = await page.evaluate(() => t('ik.lag'));
  ok(lag === E.lag, `${tag} ik.lag "${lag}"`);
  ok(errs.length === 0, `${tag} page errors: ${errs.join(' | ')}`);
  await ctx.close();
}
await browser.close();
srv.close();
console.log(`${shots.length} screenshots in ${out}, ${checks} checks, ${bad ? bad + ' FAILED' : 'all passed'}`);
for (const s of shots) console.log('  ' + s);
process.exit(bad ? 1 : 0);
