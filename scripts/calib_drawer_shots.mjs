#!/usr/bin/env node
// Headless screenshots of the hand-eye calibration drawer (no armd needed; API calls fail and are ignored).
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
const srv = http.createServer((req, res) => {
  if (req.url === '/' || req.url.startsWith('/?') || req.url === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
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
    if (errs.length) console.log(`  page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
