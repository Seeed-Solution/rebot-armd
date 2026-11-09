#!/usr/bin/env node
// Headless screenshots of the hand-eye calibration drawer (no armd needed; API calls other than a mocked
// /api/v1/vision/mode fail and are ignored). Checks that opening the drawer switches the vision mode to
// ArUco (Both when YOLO was on) with inference on, and that closing it restores the previous mode.
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/calib_drawer_shots.mjs
// Serves src/api/index.html on a local port, opens the drawer in both modes (zh / en, desktop 1400x900 and
// phone 390x844), once empty and once with synthetic eye-in-hand samples + result, and writes PNGs to
// .cache/calib-shots/. Exits non-zero if a key element is missing.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/calib-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
// stateful mock of rebot-vision's mode API; VPOSTS records the POST bodies
let VMODE = { mode: 'yolo', enabled: false };
const VPOSTS = [];
const vbody = () => JSON.stringify({ api_version: 1, mode: VMODE.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: VMODE.enabled, fps: VMODE.enabled ? 5 : 1, active_fps: 5, idle_fps: 1, available: { yolo: true, aruco: true } });
const srv = http.createServer((req, res) => {
  if (req.url === '/' || req.url.startsWith('/?') || req.url === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (req.url === '/api/v1/vision/mode') {
    let b = '';
    req.on('data', (c) => { b += c; });
    req.on('end', () => {
      if (req.method === 'POST') {
        const j = JSON.parse(b || '{}'); VPOSTS.push(j);
        if (j.mode) VMODE.mode = j.mode;
        if (typeof j.enabled === 'boolean') VMODE.enabled = j.enabled;
        if (VMODE.mode === 'off') VMODE.enabled = false;
      }
      res.writeHead(200, { 'content-type': 'application/json' }); res.end(vbody());
    });
    return;
  }
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
const shots = [];
let bad = 0;
for (const [vpName, viewport] of [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }]]) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'phone' ? 2 : 1 });
    await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(e.message));
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(400);
    for (const mode of ['eye_in_hand', 'eye_to_hand']) {
      for (const filled of [false, true]) {
        await page.evaluate(([mode, filled]) => {
          CAL.samples = []; CAL.result = null; CAL.markerId = null; CAL.mode = mode;
          if (filled) {
            const qs = [[0, 0.8, 1.4, 0, 0.3, 0.2], [0.4, 0.7, 1.3, 0.2, -0.3, -0.5], [-0.4, 0.9, 1.5, -0.3, 0.4, 0.9],
              [0.2, 0.6, 1.2, 0.4, 0, -1], [-0.2, 1, 1.6, -0.1, -0.5, 0.4], [0.5, 0.8, 1.5, -0.4, 0.2, 1.2], [-0.5, 0.7, 1.3, 0.3, -0.2, -0.8]];
            const X = CALM.make(CALM.rpyR(0.1, -1.4, 0.2), [0.03, 0, 0.05]), Y = CALM.make(CALM.rpyR(0, 0, 0.3), [0.35, 0, 0]);
            for (const [i, q] of qs.entries()) {
              const G = CALM.fk(q), M = CALM.mul(CALM.inv(CALM.mul(G, X)), Y);
              const n = CALM.make(CALM.expR([0.002, -0.003, 0.001].map((v, k) => v * Math.sin(3 * i + k) * (i === 4 ? 6 : 1))), [0.001, -0.0015, 0.002].map((v, k) => v * Math.cos(2 * i + k) * (i === 4 ? 6 : 1))), Mn = CALM.mul(M, n);
              CAL.samples.push({ joints: [...q, 0], p_base: CALM.posOf(G), T_base_gripper: G, marker_id: 7, p_cam: CALM.posOf(Mn), R_cam: CALM.rotOf(Mn), T_cam_marker: Mn });
            }
            CAL.markerId = 7;
            if (mode === 'eye_in_hand') calSolve(); else { const r = solveRigid(CAL.samples.map((s) => s.p_cam), CAL.samples.map((s) => s.p_base)); if (!r.err) CAL.result = { ...r, rpy: CALM.rpyDeg(r.R), n: CAL.samples.length, at: '' }; }
          }
          calOpen(true); calRender();
        }, [mode, filled]);
        await page.waitForTimeout(250);
        const info = await page.evaluate(() => ({
          drawer: !document.getElementById('calib').hidden,
          seg: [...document.querySelectorAll('#calib-mode button')].map((b) => [b.textContent, b.classList.contains('on')]),
          hint: document.getElementById('calib-hint').textContent.slice(0, 40),
          result: !document.getElementById('calib-result').hidden,
          hscroll: document.querySelector('.calib-drawer').scrollWidth > document.querySelector('.calib-drawer').clientWidth,
        }));
        const onOk = info.seg.length === 2 && info.seg.filter(([, on]) => on).length === 1;
        if (!info.drawer || !onOk || !info.hint || (filled && !info.result) || info.hscroll) bad++;
        const file = path.join(out, `calib_${mode}_${lang}_${vpName}${filled ? '_result' : ''}.png`);
        await page.screenshot({ path: file });
        shots.push(file);
        console.log(`${path.relative(root, file)}  drawer=${info.drawer} seg=${JSON.stringify(info.seg)} result=${info.result} hscroll=${info.hscroll}`);
      }
    }
    // vision mode: open from yolo/standby -> both + enabled; close -> yolo/standby again
    await page.evaluate(() => calOpen(false));
    await page.waitForTimeout(200);
    VMODE = { mode: 'yolo', enabled: false }; VPOSTS.length = 0;
    await page.evaluate(() => calOpen(true));
    await page.waitForTimeout(300);
    const during = { ...VMODE };
    const seg = await page.evaluate(() => [...document.querySelectorAll('#vmode button.on')].map((b) => b.dataset.vm));
    await page.evaluate(() => calOpen(false));
    await page.waitForTimeout(300);
    const vmOk = during.mode === 'both' && during.enabled && VMODE.mode === 'yolo' && !VMODE.enabled && seg.join() === 'both'
      && JSON.stringify(VPOSTS) === JSON.stringify([{ mode: 'both', enabled: true }, { mode: 'yolo', enabled: false }]);
    // from off: aruco while open, off after
    VMODE = { mode: 'off', enabled: false };
    await page.evaluate(() => calOpen(true));
    await page.waitForTimeout(300);
    const during2 = { ...VMODE };
    await page.evaluate(() => calOpen(false));
    await page.waitForTimeout(300);
    const vmOk2 = during2.mode === 'aruco' && during2.enabled && VMODE.mode === 'off';
    if (!vmOk || !vmOk2) bad++;
    console.log(`  vision mode (${lang}/${vpName}): open from yolo -> ${JSON.stringify(during)} hud=${seg} close -> ${JSON.stringify(VMODE)} posts=${JSON.stringify(VPOSTS)} ${vmOk && vmOk2 ? 'OK' : 'FAIL'}`);
    if (errs.length) console.log(`  page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
