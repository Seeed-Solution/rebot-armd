#!/usr/bin/env node
// Headless check of the camera-view / base frame toggle in the cartesian cockpit (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/view_frame_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF /api/v1/observation/schema, feeds a synthetic observation
// (state Armed, positions follow the streamed targets), and for simple / expert x zh / en x 1280x800 / 375x812
// x view / base frame writes a PNG (default .cache/view-shots/) and asserts: the frame toggle is visible,
// pad-srot is visible only in view mode, btn-pose is hidden only in view mode, no horizontal page scroll.
// Then drives W / A / ArrowRight / E in both frames and checks the motion direction of the commanded pose.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/view-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5, 5, 3]]
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
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
// synthetic arm: model "inspect" pose with J6 = pi/2; the observation follows whatever the page streams
const feed = () => {
  window.__sim = window.__sim || [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0.2];
  window.__sent = 0;
  window.cartSend = (t) => { window.__sim = t.slice(); window.__sent++; };
  setInterval(() => {
    S.obs = { positions: window.__sim.slice(), velocities: [0, 0, 0, 0, 0, 0, 0], safety_state: 'Armed', enable_state: 'Enabled', feedback_age_ms: [5, 5, 5, 5, 5, 5, 5], epoch_id: 1 };
    S.obsAt = Date.now(); S.state = 'Armed'; S.epoch = 1; S.leaseGeneration = 1;
  }, 20);
};
const shots = [];
let bad = 0;
const fail = (m) => { bad++; console.log('  FAIL ' + m); };
for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); localStorage.setItem('rebot.ctl', 'cart'); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  for (const frame of ['view', 'base']) {
    await page.evaluate((f) => { setFrame(f); }, frame);
    await page.waitForTimeout(300);
    const info = await page.evaluate((ui) => {
      const vis = (id) => { const el = document.getElementById(id); if (!el) return false; const r = el.getBoundingClientRect(); return r.width > 0 && r.height > 0 && getComputedStyle(el).visibility !== 'hidden'; };
      const seg = document.getElementById(ui === 'simple' ? 'frame-seg-s' : 'frame-seg-e');
      return {
        dof: S.dof, frame: C.frame, view: viewActive(),
        toggle: vis(seg.id), segOn: [...seg.children].filter((b) => b.classList.contains('on')).map((b) => b.dataset.frame),
        srot: vis('pad-srot'), pose: vis('btn-pose'),
        sw: document.scrollingElement.scrollWidth, iw: window.innerWidth,
        label: document.querySelector(ui === 'simple' ? '#pad-srot .dir.t .lbl' : '#vcol-r .vlab b').textContent,
        kb: document.querySelector('[data-i18n="kb.x"]').textContent,
      };
    }, ui);
    const tag = `${ui}_${lang}_${vp}_${frame}`;
    const file = path.join(out, `view_${tag}.png`);
    await page.screenshot({ path: file });
    shots.push(file);
    const exp = frame === 'view';
    if (info.dof !== 7) fail(`${tag} dof ${info.dof}`);
    if (!info.toggle || info.segOn.length !== 1 || info.segOn[0] !== frame) fail(`${tag} frame toggle ${info.toggle} ${info.segOn}`);
    if (ui === 'simple' && info.srot !== exp) fail(`${tag} pad-srot visible=${info.srot}`);
    if (ui === 'simple' && info.pose === exp) fail(`${tag} btn-pose visible=${info.pose}`);
    if (ui === 'expert' && info.srot) fail(`${tag} pad-srot visible in expert`);
    if (info.sw > info.iw) fail(`${tag} horizontal scroll ${info.sw} > ${info.iw}`);
    console.log(`${tag}  toggle=${info.toggle} on=${info.segOn} pad-srot=${info.srot} btn-pose=${info.pose} scrollWidth=${info.sw}<=${info.iw} label="${info.label}" kb.x="${info.kb}"`);
  }
  if (errs.length) fail(`page errors (${ui}/${lang}/${vp}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
  await ctx.close();
}

// motion directions (simple, zh, desktop): hold a key 400 ms, compare the commanded pose before / after
{
  const ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  await ctx.addInitScript(() => { try { localStorage.clear(); localStorage.setItem('lang', 'zh'); } catch (_) {} });
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base, { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  const snap = () => page.evaluate(() => {
    const q = window.__sim, T = VIK.fk([...toModel(q), q[4], q[5]]), c = VIK.camAxes(T, CFG.view.Rec);
    return { p: CALM.posOf(T), z: c.z, x: c.x, sent: window.__sent, status: C.status };
  });
  const hold = async (code, ms = 400) => {
    await page.evaluate((c) => window.dispatchEvent(new KeyboardEvent('keydown', { code: c, key: c })), code);
    await page.waitForTimeout(ms);
    await page.evaluate((c) => window.dispatchEvent(new KeyboardEvent('keyup', { code: c, key: c })), code);
    await page.waitForTimeout(700);
  };
  const sub = (a, b) => a.map((v, k) => v - b[k]), dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], nrm = (a) => Math.hypot(...a);
  const check = (name, ok, d) => { console.log(`${ok ? 'PASS' : 'FAIL'}  motion ${name}  ${d}`); if (!ok) bad++; };
  for (const frame of ['view', 'base']) {
    // each frame starts from the inspect pose (J5 = 0: the base-frame IK holds J5 and assumes it at 0)
    await page.evaluate((f) => { window.__sim = [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0.2]; setFrame(f); }, frame);
    await page.waitForTimeout(300);
    let s0 = await snap(); await hold('KeyW'); let s1 = await snap();
    let d = sub(s1.p, s0.p);
    if (frame === 'view') check('view W = forward along z_cam', nrm(d) > 0.005 && dot(d, s0.z) / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, cos(d, z_cam) ${(dot(d, s0.z) / nrm(d)).toFixed(4)}, frames sent ${s1.sent - s0.sent}`);
    else check('base W = +X', nrm(d) > 0.005 && d[0] / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, dir ${d.map((v) => (v / nrm(d)).toFixed(3))}`);
    s0 = await snap(); await hold('KeyA'); s1 = await snap(); d = sub(s1.p, s0.p);
    if (frame === 'view') check('view A = left along -x_cam', nrm(d) > 0.005 && -dot(d, s0.x) / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, cos(d, -x_cam) ${(-dot(d, s0.x) / nrm(d)).toFixed(4)}`);
    else check('base A = +Y', nrm(d) > 0.005 && d[1] / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, dir ${d.map((v) => (v / nrm(d)).toFixed(3))}`);
    if (frame === 'view') {
      s0 = await snap(); await hold('ArrowRight'); s1 = await snap(); d = sub(s1.p, s0.p);
      const h0 = Math.atan2(s0.z[1], s0.z[0]), h1 = Math.atan2(s1.z[1], s1.z[0]);
      check('view ArrowRight = look right, TCP fixed, level', nrm(d) < 0.0015 && h1 < h0 - 0.05 && Math.abs(s1.x[2]) < 0.01, `TCP moved ${(nrm(d) * 1000).toFixed(2)} mm, heading ${(h0 * 57.3).toFixed(1)} -> ${(h1 * 57.3).toFixed(1)} deg, x_cam.Z ${s1.x[2].toFixed(4)}`);
      s0 = await snap(); await hold('KeyE'); s1 = await snap(); d = sub(s1.p, s0.p);
      check('view E = look up, TCP fixed', nrm(d) < 0.0015 && s1.z[2] > s0.z[2] + 0.02, `TCP moved ${(nrm(d) * 1000).toFixed(2)} mm, z_cam.Z ${s0.z[2].toFixed(3)} -> ${s1.z[2].toFixed(3)}`);
    }
  }
  if (errs.length) { bad++; console.log('  FAIL page errors: ' + [...new Set(errs)].join(' | ').slice(0, 300)); }
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK, all checks passed`);
process.exit(bad ? 1 : 0);
