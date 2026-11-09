#!/usr/bin/env node
// Headless check of the cockpit vision-mode selector and the "both" overlay (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/vision_mode_shots.mjs
// Serves src/api/index.html with a stateful mock of rebot-vision 1.3 (perception API 2:
// /api/v1/vision/mode, /subscribe, /subscribe/<id>/keepalive, DELETE /subscribe/<id>, /detection/trigger)
// and /api/v1/detection/inference. For simple / expert x zh / en x 1280x800 / 375x812 it checks the
// selector (Off / YOLO / ArUco / Both): choosing a set creates this page's overlay subscription
// (kept alive, updated in place on a new choice, deleted on Off, on Detect off and while the page is
// hidden) and never POSTs the global mode; single shot sends the selected pipelines; a "both" payload
// draws YOLO boxes and ArUco markers together; YOLO choices are disabled without a model; an API 1
// provider still gets {mode, enabled:true}; a provider without the route (404) hides the selector.
// Writes PNGs (default .cache/vision-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/vision-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));

// V.v2: perception API 2 (subscriptions, per-request pipelines); false = API 1 (global mode)
let V = { mode: 'yolo', enabled: false, yolo: true, route: true, v2: true };
const POSTS = [];   // POST /api/v1/vision/mode bodies
const CALLS = [];   // subscription and trigger requests: [method, path, body]
const SUBS = new Map(); let nextSub = 1;
const vbody = () => ({ api_version: V.v2 ? 2 : 1, mode: V.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: V.enabled, fps: V.enabled || SUBS.size ? 5 : 1,
  active_fps: 5, idle_fps: 1, trigger: true, available: { yolo: V.yolo, aruco: true },
  ...(V.v2 ? { per_request_pipelines: true, subscriptions: true } : {}) });
const json = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    if (u === '/api/v1/detection/inference') {
      if (req.method === 'POST') { const j = JSON.parse(b || '{}'); V.enabled = !!j.enabled; if (V.enabled && V.mode === 'off') V.mode = 'yolo'; }
      return json(res, 200, { enabled: V.enabled, fps: V.enabled || SUBS.size ? 5 : 1, active_fps: 5, idle_fps: 1, mode: V.mode });
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
    const j = b ? JSON.parse(b) : null;
    if (u.startsWith('/api/v1/vision/subscribe') || u === '/api/v1/detection/trigger') CALLS.push([req.method, u, j]);
    if (u === '/api/v1/vision/subscribe' && req.method === 'POST') {
      if (j.pipelines.includes('yolo') && !V.yolo) return json(res, 409, { api_version: 2, error: 'yolo unavailable' });
      let id = [...SUBS].find(([, s]) => s.client === j.client)?.[0];
      if (!id) id = 's' + nextSub++;
      SUBS.set(id, { client: j.client, pipelines: j.pipelines });
      return json(res, 200, { api_version: 2, id, client: j.client, pipelines: j.pipelines, fps: 5, ttl_ms: j.ttl_ms });
    }
    let m = u.match(/^\/api\/v1\/vision\/subscribe\/([^/]+)(\/keepalive)?$/);
    if (m) {
      if (!SUBS.has(m[1])) return json(res, 404, { api_version: 2, error: 'no such subscription' });
      if (!m[2] && req.method === 'DELETE') SUBS.delete(m[1]);
      return json(res, 200, { api_version: 2, id: m[1] });
    }
    if (u === '/api/v1/detection/trigger') {
      const p = (j && j.pipelines) || ['yolo'];
      return json(res, 200, { ...window_both(p), api_version: 2, success: true, frame_id: 50 + CALLS.length });
    }
    res.writeHead(404); res.end();
  });
}).listen(0, '127.0.0.1');
// a trigger result as rebot-vision 1.3 sends it (visible 640x360 band)
const window_both = (p) => ({ width: 640, height: 360, pipelines: { yolo: { ran: p.includes('yolo') }, aruco: { ran: p.includes('aruco') } },
  detections: p.includes('yolo') ? [{ class_id: 39, format: 'xywh_center', class_name: 'bottle', confidence: 0.78, x: 0.71, y: 0.54, w: 0.11, h: 0.42 }] : [],
  markers: p.includes('aruco') ? [{ id: 7, margin: 61, center: [182, 212], corners: [[150, 244], [214, 244], [214, 180], [150, 180]], pose: { t: [-0.08, 0.06, 0.42], r_deg: [178.2, 1.5, 3.0], error: 0.0004 } }] : [] });
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

const subCalls = () => CALLS.filter(([, u]) => u.startsWith('/api/v1/vision/subscribe'));
const setHidden = (page, hidden) => page.evaluate((h) => {
  Object.defineProperty(document, 'hidden', { value: h, configurable: true });
  Object.defineProperty(document, 'visibilityState', { value: h ? 'hidden' : 'visible', configurable: true });
  document.dispatchEvent(new Event('visibilitychange'));
}, hidden);

for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const tag = `${ui}_${lang}_${vp}`;
  V = { mode: 'yolo', enabled: false, yolo: true, route: true, v2: true }; POSTS.length = 0; CALLS.length = 0; SUBS.clear();
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);

  // 1. standby: no subscription, selector shows Off (this page's overlay is off)
  let h = await hud(page);
  check(`${tag} selector`, h.shown && JSON.stringify(h.labels) === JSON.stringify(T[lang]) && h.on.join() === 'off' && h.inside, JSON.stringify(h));
  check(`${tag} standby`, SUBS.size === 0 && subCalls().length === 0, JSON.stringify(CALLS));
  let file = path.join(out, `vision_standby_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 2. Both: one subscription {client, pipelines [yolo, aruco], ttl_ms}; overlay draws box + marker; kept alive
  await page.click('#vmode button[data-vm="both"]');
  await page.waitForTimeout(1300);
  h = await hud(page);
  const ov = await page.evaluate(() => ({ n: S.detections.length, aruco: S.detections.filter((d) => d.aruco).length, labels: S.detections.map((d) => d.label) }));
  const sc = subCalls();
  const create = sc[0] || [];
  const sid = [...SUBS.keys()][0];
  check(`${tag} both subscribe`, create[0] === 'POST' && create[1] === '/api/v1/vision/subscribe' && JSON.stringify(create[2].pipelines) === '["yolo","aruco"]'
    && /^cockpit-overlay-/.test(create[2].client) && create[2].ttl_ms === 3000, JSON.stringify(sc));
  check(`${tag} keepalive`, sc.some(([m, u]) => m === 'POST' && u === `/api/v1/vision/subscribe/${sid}/keepalive`), JSON.stringify(sc));
  check(`${tag} no global mode`, POSTS.length === 0 && !V.enabled, JSON.stringify(POSTS));
  check(`${tag} both state`, h.on.join() === 'both' && SUBS.size === 1, JSON.stringify(h));
  check(`${tag} both overlay`, ov.n === 2 && ov.aruco === 1, JSON.stringify(ov));
  check(`${tag} no h-scroll`, h.sw <= h.iw, JSON.stringify(h));
  file = path.join(out, `vision_both_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  hud=${JSON.stringify(h.labels)} on=${h.on} det="${h.det}" overlay=${JSON.stringify(ov)} subs=${JSON.stringify([...SUBS])} calls=${sc.length} scroll=${h.sw}<=${h.iw}`);

  // 3. single shot with the selected pipelines
  CALLS.length = 0;
  await page.click('#btn-trigger'); await page.waitForTimeout(300);
  const trig = CALLS.find(([, u]) => u === '/api/v1/detection/trigger');
  check(`${tag} trigger body`, trig && JSON.stringify(trig[2]) === '{"pipelines":["yolo","aruco"]}', JSON.stringify(CALLS));

  // 4. ArUco updates the same subscription in place, Off deletes it
  CALLS.length = 0;
  await page.click('#vmode button[data-vm="aruco"]'); await page.waitForTimeout(250);
  check(`${tag} aruco`, SUBS.size === 1 && SUBS.get(sid) && JSON.stringify(SUBS.get(sid).pipelines) === '["aruco"]', JSON.stringify([...SUBS]));
  await page.click('#vmode button[data-vm="off"]'); await page.waitForTimeout(300);
  h = await hud(page);
  const n = await page.evaluate(() => S.detections.length);
  check(`${tag} off deletes`, SUBS.size === 0 && subCalls().some(([m, u]) => m === 'DELETE' && u === `/api/v1/vision/subscribe/${sid}`) && h.on.join() === 'off' && n === 0,
    JSON.stringify({ calls: subCalls(), h, n }));

  // 5. Detect toggle from Off subscribes with the last pipelines (ArUco); page hide deletes, visible again re-subscribes
  await page.click('#btn-det'); await page.waitForTimeout(300);
  h = await hud(page);
  check(`${tag} detect from off`, SUBS.size === 1 && JSON.stringify([...SUBS.values()][0].pipelines) === '["aruco"]' && h.on.join() === 'aruco', JSON.stringify({ subs: [...SUBS], h }));
  await setHidden(page, true); await page.waitForTimeout(300);
  check(`${tag} hidden deletes`, SUBS.size === 0, JSON.stringify([...SUBS]));
  await setHidden(page, false); await page.waitForTimeout(300);
  check(`${tag} visible resubscribes`, SUBS.size === 1, JSON.stringify([...SUBS]));
  await page.click('#btn-det'); await page.waitForTimeout(300);
  check(`${tag} detect off`, SUBS.size === 0 && POSTS.length === 0, JSON.stringify({ subs: [...SUBS], POSTS }));

  // 6. no YOLO model: YOLO / Both disabled
  V.yolo = false;
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  h = await hud(page);
  check(`${tag} no model`, h.disabled.sort().join() === 'both,yolo', JSON.stringify(h));
  if (ui === 'expert') { file = path.join(out, `vision_nomodel_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }

  // 7. API 1 provider (no subscriptions): the selector sets the global mode as before
  V = { ...V, v2: false, yolo: true, mode: 'yolo', enabled: false };
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  await page.click('#vmode button[data-vm="both"]'); await page.waitForTimeout(300);
  check(`${tag} api1 mode post`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'both', enabled: true }) && V.enabled, JSON.stringify(POSTS));

  // 8. provider without /api/mode: selector hidden, Detect stays
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
