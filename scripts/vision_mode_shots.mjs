#!/usr/bin/env node
// Headless check of the cockpit vision-mode selector and the "both" overlay (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/vision_mode_shots.mjs
// Serves src/api/index.html with a stateful mock of /api/v1/vision/mode (rebot-vision) and
// /api/v1/detection/inference. For simple / expert x zh / en x 1280x800 / 375x812 it checks the
// selector (Off / YOLO / ArUco / Both), that choosing a mode POSTs {mode, enabled:true} and Off POSTs
// {mode:"off"}, that a "both" /results payload draws YOLO boxes and ArUco markers together (each marker
// once), that YOLO choices are disabled when the model is missing, and that a provider without the
// route (404) hides the selector. Writes PNGs (default .cache/vision-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/vision-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));

let V = { mode: 'yolo', enabled: false, yolo: true, route: true };
const POSTS = [];
const vbody = () => ({ api_version: 1, mode: V.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: V.enabled, fps: V.enabled ? 5 : 1,
  active_fps: 5, idle_fps: 1, trigger: true, available: { yolo: V.yolo, aruco: true } });
const json = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    if (u === '/api/v1/detection/inference') {
      if (req.method === 'POST') { const j = JSON.parse(b || '{}'); V.enabled = !!j.enabled; if (V.enabled && V.mode === 'off') V.mode = 'yolo'; }
      return json(res, 200, { enabled: V.enabled, fps: V.enabled ? 5 : 1, active_fps: 5, idle_fps: 1, mode: V.mode });
    }
    if (u === '/api/v1/vision/mode') {
      if (!V.route) return json(res, 404, { error: 'not found' });
      if (req.method === 'POST') {
        const j = JSON.parse(b || '{}'); POSTS.push(j);
        if ((j.mode === 'yolo' || j.mode === 'both') && !V.yolo) return json(res, 409, { api_version: 1, error: 'yolo unavailable' });
        if (j.mode) V.mode = j.mode;
        if (typeof j.enabled === 'boolean') V.enabled = j.enabled;
        if (V.mode === 'off') V.enabled = false;
      }
      return json(res, 200, vbody());
    }
    res.writeHead(404); res.end();
  });
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

// Fake video frame, and a "both" /results envelope as rebot-vision sends it (display pixels 1280x720).
const feed = () => {
  const poster = document.createElement('canvas'); poster.width = 640; poster.height = 360;
  const g = poster.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 360); grad.addColorStop(0, '#b9a38a'); grad.addColorStop(1, '#8f7a63');
  g.fillStyle = grad; g.fillRect(0, 0, 640, 360);
  g.fillStyle = '#3d6fb4'; g.fillRect(420, 120, 70, 150);
  g.fillStyle = '#fff'; g.fillRect(150, 180, 64, 64); g.fillStyle = '#111'; g.fillRect(158, 188, 48, 48); g.fillStyle = '#fff'; g.fillRect(170, 200, 12, 12);
  const v = document.getElementById('video-stream');
  v.poster = poster.toDataURL();
  Object.defineProperty(v, 'videoWidth', { get: () => 640 });
  Object.defineProperty(v, 'videoHeight', { get: () => 360 });
  window.__both = { timestamp: 1, frame_id: 42, inference_time_ms: 80.5, resolution: [1280, 720],
    boxes: [[910, 390, 140, 300, 0.78, 'bottle']], labels: ['bottle'], mode: 'both',
    markers: [{ id: 7, margin: 61, center: [364, 424], corners: [[300, 488], [428, 488], [428, 360], [300, 360]],
      pose: { t: [-0.08, 0.06, 0.42], r_deg: [178.2, 1.5, 3.0], error: 0.0004 } }] };
  setInterval(() => {
    const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide');
    if (DET.enabled) { S.detections = parseDetections(window.__both, videoSrcRes()); S.detSource = 'mock'; S.detLastReceivedAt = performance.now(); }
  }, 30);
};

const shots = [];
let bad = 0, checks = 0;
const check = (name, ok, d = '') => { checks++; if (!ok) { bad++; console.log(`  FAIL ${name} ${d}`); } };
const T = { zh: ['关', 'YOLO', 'ArUco', '两者'], en: ['Off', 'YOLO', 'ArUco', 'Both'] };
const hud = (page) => page.evaluate(() => {
  const w = document.getElementById('vmode'), r = w.getBoundingClientRect(), vb = document.getElementById('video-box').getBoundingClientRect();
  return { shown: !w.closest('[hidden]') && r.width > 0, labels: [...w.querySelectorAll('button')].map((b) => b.textContent),
    on: [...w.querySelectorAll('button.on')].map((b) => b.dataset.vm), disabled: [...w.querySelectorAll('button:disabled')].map((b) => b.dataset.vm),
    inside: r.left >= vb.left - 1 && r.right <= vb.right + 1, det: document.getElementById('btn-det').textContent,
    sw: document.scrollingElement.scrollWidth, iw: window.innerWidth };
});

for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const tag = `${ui}_${lang}_${vp}`;
  V = { mode: 'yolo', enabled: false, yolo: true, route: true }; POSTS.length = 0;
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);

  // 1. standby in yolo
  let h = await hud(page);
  check(`${tag} selector`, h.shown && JSON.stringify(h.labels) === JSON.stringify(T[lang]) && h.on.join() === 'yolo' && h.inside, JSON.stringify(h));
  let file = path.join(out, `vision_standby_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 2. Both: POST {mode:'both', enabled:true}; overlay draws the box and the marker
  await page.click('#vmode button[data-vm="both"]');
  await page.waitForTimeout(400);
  h = await hud(page);
  const ov = await page.evaluate(() => ({ n: S.detections.length, aruco: S.detections.filter((d) => d.aruco).length, labels: S.detections.map((d) => d.label) }));
  check(`${tag} both post`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'both', enabled: true }), JSON.stringify(POSTS));
  check(`${tag} both state`, h.on.join() === 'both' && V.enabled, JSON.stringify(h));
  check(`${tag} both overlay`, ov.n === 2 && ov.aruco === 1, JSON.stringify(ov));
  check(`${tag} no h-scroll`, h.sw <= h.iw, JSON.stringify(h));
  file = path.join(out, `vision_both_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  hud=${JSON.stringify(h.labels)} on=${h.on} det="${h.det}" overlay=${JSON.stringify(ov)} scroll=${h.sw}<=${h.iw}`);

  // 3. ArUco, then Off (inference off, overlay cleared)
  await page.click('#vmode button[data-vm="aruco"]'); await page.waitForTimeout(250);
  check(`${tag} aruco`, V.mode === 'aruco' && V.enabled && JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'aruco', enabled: true }), JSON.stringify(V));
  await page.click('#vmode button[data-vm="off"]'); await page.waitForTimeout(300);
  h = await hud(page);
  const n = await page.evaluate(() => S.detections.length);
  check(`${tag} off`, V.mode === 'off' && !V.enabled && h.on.join() === 'off' && n === 0 && JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'off' }), JSON.stringify({ V, h, n }));

  // 4. Detect toggle while Off picks the last active mode (provider rule) and the selector follows
  await page.click('#btn-det'); await page.waitForTimeout(400);
  h = await hud(page);
  check(`${tag} detect from off`, V.enabled && h.on.join() === 'yolo', JSON.stringify({ V, h }));

  // 5. no YOLO model: YOLO / Both disabled
  V.yolo = false; V.mode = 'aruco';
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  h = await hud(page);
  check(`${tag} no model`, h.disabled.sort().join() === 'both,yolo' && h.on.join() === 'aruco', JSON.stringify(h));
  if (ui === 'expert') { file = path.join(out, `vision_nomodel_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }

  // 6. provider without /api/mode: selector hidden, Detect stays
  V.route = false;
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  h = await hud(page);
  check(`${tag} old provider hides selector`, !h.shown, JSON.stringify(h));

  if (errs.length) { bad++; console.log(`  page errors (${tag}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`); }
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} of ${checks} checks failed` : `${checks} checks, ${shots.length} screenshots OK (${path.relative(root, out)})`);
process.exit(bad ? 1 : 0);
