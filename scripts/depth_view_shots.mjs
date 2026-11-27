#!/usr/bin/env node
// Headless check of the cockpit depth view (no armd needed):
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/depth_view_shots.mjs
// Serves src/api/index.html with a mocked rebot-vision (perception API 2, depth model resident and metric) and a fake
// GET /api/v1/vision/depth that sends a synthetic "RDM1" map: 384x224, depth 300 mm at the left edge to 600 mm at the
// right, a near block (cells x 100..149, y 50..99) at 250 mm and an empty block (x 250..279, y 100..129, value 0). It
// enforces one accepted request per second (429 + Retry-After otherwise). The page's telemetry WebSocket is answered
// with observations at 20 Hz (routeWebSocket).
// For simple / expert x zh / en x desktop 1280x800, phone portrait 390x844 and phone landscape 844x390:
// - 深度（估计）/ Depth (est.) is an aria-pressed toggle of the vision selector #vmode (in the recognition popover in simple
//   mode, and in expert mode on a phone);
// - one tap fetches one map; the panel is labelled as a model estimate, the legend reads near / far with
//   "≈ x.xx m (估计 / est.)" from the map's p2 / p98 and the not-calibrated note;
// - colour map alignment: canvas pixels at the screen centres of known cells have the colour of that cell (both sides
//   of the near block's left and bottom edges, the gradient), the empty block is transparent;
// - tap / hover readout at known cells ("≈ 250 mm (估计)", "此处无深度" / "No depth here");
// - the map canvas takes no pointer events (elementFromPoint at its centre is not the canvas);
// - opacity slider -> canvas opacity;
// - Live: requests at most 1 per second (start to start >= 1000 ms), never a 429 from the page's own schedule;
//   a forced 429 is retried after Retry-After;
// - the arm moves -> the map is dropped, Live turns off, "机械臂已移动" / "Arm moved" shows, no further requests;
// - the page hidden -> no requests; visible again -> Live resumes; closing the depth view (the panel's x, or the Depth
//   toggle again) -> the toggle released, no requests;
// - the E-stop is reachable (elementFromPoint at its centre, a real tap runs eStop) with the depth panel open;
// - no horizontal scroll, the panel inside the viewport.
// Writes PNGs (default .cache/depth-view-shots/). Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { readCockpitHtmlBuffer } from './source_paths.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/depth-view-shots');
fs.mkdirSync(out, { recursive: true });
const html = readCockpitHtmlBuffer();
const wizard = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));

// ---- synthetic depth map
const W = 384, H = 224;
const cellMm = (x, y) => (x >= 250 && x < 280 && y >= 100 && y < 130 ? 0 : x >= 100 && x < 150 && y >= 50 && y < 100 ? 250 : Math.round(300 + 300 * x / (W - 1)));
const MM = new Uint16Array(W * H);
for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) MM[y * W + x] = cellMm(x, y);
const valid = Array.from(MM).filter((v) => v > 0).sort((a, b) => a - b);
const pct = (q) => valid[Math.min(valid.length - 1, Math.round(q * (valid.length - 1)))];
const STATS = { valid: valid.length, min_mm: valid[0], p2_mm: pct(0.02), p50_mm: pct(0.5), p98_mm: pct(0.98), max_mm: valid[valid.length - 1] };
let frameId = 100;
function depthBody() {
  let js = JSON.stringify({ format: 'u16le_mm', width: W, height: H, units: 'mm', invalid: 0, frame_id: ++frameId, ts_ms: Date.now(), inference_ms: 173.0,
    model: 'zipdepth_metric_384x224_bf16_rgb', scale: { depth_m: 'scale_m / disparity', scale_m: 1.0, fx: 600, fx_width: 640 },
    mapping: { image_w: 640, image_h: 360, fit: 'stretch', x0: 0, y0: 0, x1: 640, y1: 360 }, stats: STATS });
  while ((8 + Buffer.byteLength(js)) % 8) js += ' ';
  const jb = Buffer.from(js);
  const buf = Buffer.alloc(8 + jb.length + W * H * 2);
  buf.write('RDM1', 0, 'latin1'); buf.writeUInt32LE(jb.length, 4); jb.copy(buf, 8);
  for (let i = 0; i < MM.length; i++) buf.writeUInt16LE(MM[i], 8 + jb.length + 2 * i);
  return buf;
}

const D = { reqs: [], last: -1e9, force429: false, n429: 0 };
const send = (res, code, o, headers = {}) => { res.writeHead(code, { 'content-type': 'application/json', ...headers }); res.end(JSON.stringify(o)); };
const SUBS = new Map(); let nextSub = 1;
const vbody = () => ({ api_version: 2, mode: 'yolo', modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: SUBS.size ? 5 : 1, active_fps: 5, idle_fps: 1, trigger: true,
  per_request_pipelines: true, subscriptions: true, available: { yolo: true, aruco: true, depth: true },
  depth_model: { name: 'zipdepth_metric_384x224_bf16_rgb', path: '/userdata/local/models/zipdepth_metric_384x224_bf16_rgb.cvimodel', metric: true } });
const LIMITS = [[-2.8, 2.8, 2], [-3.14, 0, 2], [-3.14, 0, 2], [-1.87, 1.57, 3], [-1.57, 1.57, 3], [-3.14, 3.14, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(wizard); return; }
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    const j = b ? JSON.parse(b) : null;
    if (u === '/api/v1/observation/schema') return send(res, 200, { dof: 7, limits: LIMITS, home_positions: [0, 0, 0, 0, 0, 1.5707963, 0], capabilities: { version: 9, poses: true, fault_codes: true } });
    if (u === '/api/v1/vision/mode') return send(res, 200, vbody());
    if (u === '/api/v1/detection/inference') return send(res, 200, { enabled: false, fps: 1, active_fps: 5, idle_fps: 1 });
    if (u === '/api/v1/vision/subscribe' && req.method === 'POST') { const id = 's' + nextSub++; SUBS.set(id, j); return send(res, 200, { api_version: 2, id, client: j.client, pipelines: j.pipelines, fps: 5, ttl_ms: j.ttl_ms }); }
    const m = u.match(/^\/api\/v1\/vision\/subscribe\/([^/]+)(\/keepalive)?$/);
    if (m) { if (!m[2] && req.method === 'DELETE') SUBS.delete(m[1]); return send(res, 200, { api_version: 2, id: m[1] }); }
    if (u === '/api/v1/vision/depth') {
      const now = performance.now();
      D.reqs.push(now);
      if (D.force429 || now - D.last < 1000) {
        D.force429 = false; D.n429++;
        const ra = Math.max(1, Math.ceil((D.last + 1000 - now) / 1000));
        return send(res, 429, { api_version: 2, error: 'depth rate limited', detail: 'at most one depth map per second', retry_after_ms: Math.max(1, Math.round(D.last + 1000 - now)) }, { 'retry-after': String(ra) });
      }
      D.last = now;
      setTimeout(() => { res.writeHead(200, { 'content-type': 'application/octet-stream', 'cache-control': 'no-store' }); res.end(depthBody()); }, 180); // ~ one inference
      return;
    }
    if (u === '/api/v1/poses') return send(res, 200, { poses: [] });
    res.writeHead(404); res.end();
  });
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } return c; };
const shots = [];
const TXT = {
  zh: { btn: '深度（估计）', tip: '深度图：模型估计，不是测量值', title: '深度（模型估计）', near: '近', far: '远', m: (mm) => `≈ ${(mm / 1000).toFixed(2)} m（估计）`, read: (mm) => `≈ ${mm} mm（估计）`,
    none: '此处无深度', note: '未校准，仅表示相对远近', moved: '机械臂已移动，已停止刷新', frame: '帧 #', live: '连续' },
  en: { btn: 'Depth (est.)', tip: 'Depth map: model estimate, not a measurement', title: 'Depth (model estimate)', near: 'Near', far: 'Far', m: (mm) => `≈ ${(mm / 1000).toFixed(2)} m (est.)`, read: (mm) => `≈ ${mm} mm (est.)`,
    none: 'No depth here', note: 'Uncalibrated, relative only', moved: 'Arm moved: refresh stopped', frame: 'Frame #', live: 'Live' },
};
const REST = [0, -0.4, 0.8, 0, 0, 0.1, 0];
const LAYOUTS = [['desktop', { width: 1280, height: 800 }, false], ['phone', { width: 390, height: 844 }, true], ['phone_land', { width: 844, height: 390 }, true]];
const reqsSince = (t0) => D.reqs.filter((t) => t >= t0);

// screen point of grid cell (gx, gy) centre: the video element's box and the page's letterbox rule (object-fit: contain)
const cellPt = (page, gx, gy) => page.evaluate(([gx, gy, W, H]) => {
  const vb = document.getElementById('video-stream').getBoundingClientRect();
  const r = getVideoContentRect(vb.width, vb.height);
  return { x: vb.left + r.x + (gx + 0.5) / W * r.w, y: vb.top + r.y + (gy + 0.5) / H * r.h };
}, [gx, gy, W, H]);
// canvas pixel (RGBA) under a screen point, and the colour the cell's depth should have
const pixelAt = (page, pt, mm) => page.evaluate(([pt, mm]) => {
  const cv = document.getElementById('depth-overlay'), cr = cv.getBoundingClientRect(), s = cv.width / cr.width;
  const px = Array.from(cv.getContext('2d').getImageData(Math.floor((pt.x - cr.left) * s), Math.floor((pt.y - cr.top) * s), 1, 1).data);
  const k = mm ? depthIdx(DEPTH.map, mm) * 3 : -1;
  return { px, want: k >= 0 ? [TURBO[k], TURBO[k + 1], TURBO[k + 2], 255] : [0, 0, 0, 0] };
}, [pt, mm]);
const same = (a, b) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) <= 2);

try {
  for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport, mobile] of LAYOUTS) {
    const tag = `${ui}_${lang}_${vp}`;
    const L = TXT[lang];
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: mobile ? 2 : 1, isMobile: mobile, hasTouch: mobile });
    await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push('pageerror: ' + e.message));
    page.on('console', (m) => { if (m.type() === 'error' && !/^Failed to load resource|WebSocket connection to/.test(m.text())) errs.push('console: ' + m.text()); });
    const P = { q: REST.slice(), t0: Date.now() };
    await page.routeWebSocket(/\/api\/v1\/ws/, (ws) => {
      ws.onMessage((raw) => {
        let msg; try { msg = JSON.parse(String(raw)); } catch (_) { return; }
        if (msg.type === 'acquire_lease') ws.send(JSON.stringify({ type: 'lease_granted', seq: msg.seq, generation: 7, epoch_id: 1, client_id: msg.client_id }));
        else if (msg.type === 'command') ws.send(JSON.stringify({ type: 'command_ack', request_id: msg.request_id, epoch_id: 1 }));
      });
      const feed = setInterval(() => {
        const q = P.q.slice();
        const o = { timestamp: (Date.now() - P.t0) / 1000, positions: q, velocities: q.map(() => 0), commanded_positions: q, safety_state: 'Armed', enable_state: 'Enabled', is_torqued: true,
          feedback_age_ms: q.map(() => 5), torques: q.map(() => 0.2), temperatures: q.map(() => 35), epoch_id: 1, lease_holder: null, motion_limit: null, motor_errors: [], action: null, fault: null };
        try { ws.send(JSON.stringify(o)); } catch (_) { clearInterval(feed); }
      }, 50);
    });
    await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
    // fake video frame 640x360 (the detection image), so getVideoContentRect has a source size
    await page.evaluate(() => {
      const c = document.createElement('canvas'); c.width = 640; c.height = 360;
      const g = c.getContext('2d'); const gr = g.createLinearGradient(0, 0, 0, 360); gr.addColorStop(0, '#b9a38a'); gr.addColorStop(1, '#8f7a63');
      g.fillStyle = gr; g.fillRect(0, 0, 640, 360); g.fillStyle = '#3d6fb4'; g.fillRect(167, 80, 83, 80);
      const v = document.getElementById('video-stream'); v.poster = c.toDataURL();
      Object.defineProperty(v, 'videoWidth', { get: () => 640 }); Object.defineProperty(v, 'videoHeight', { get: () => 360 });
      setInterval(() => { const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide'); }, 50);
      window.__estop = 0; window.eStop = () => { window.__estop++; };
    });
    await page.waitForFunction(() => VM.state === 'ok' && VM.depthMetric, null, { timeout: 8000 }).catch(() => {});
    await page.waitForTimeout(400);
    const openVis = () => page.evaluate(() => { const b = document.getElementById('btn-vis'); if (!document.body.classList.contains('vis-open') && b && !b.hidden && getComputedStyle(b).display !== 'none') b.click(); });
    const tap = async (pt) => { if (mobile) await page.touchscreen.tap(pt.x, pt.y); else await page.mouse.click(pt.x, pt.y); };
    const tapSel = async (sel) => { const r = await page.evaluate((s) => { const b = document.querySelector(s).getBoundingClientRect(); return { x: b.left + b.width / 2, y: b.top + b.height / 2 }; }, sel); await tap(r); };

    // ---- button with the vision selector
    await openVis();
    await page.waitForTimeout(200);
    const b = await page.evaluate(() => { const e = document.querySelector('#vmode button[data-vm="depth"]'), r = e.getBoundingClientRect();
      const at = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
      return { shown: !e.closest('[hidden]') && r.width > 0, txt: e.textContent, group: e.parentElement.getAttribute('role'), pressed: e.getAttribute('aria-pressed'), top: !!at && e.contains(at),
        prev: e.previousElementSibling && e.previousElementSibling.dataset.vm, disabled: e.disabled, title: e.title, inView: r.left >= 0 && r.right <= innerWidth && r.top >= 0 && r.bottom <= innerHeight }; });
    ok(b.shown && b.inView && b.top, `${tag} depth toggle visible in the viewport, not covered`);
    ok(b.txt === L.btn, `${tag} depth toggle label "${b.txt}"`);
    ok(b.group === 'group' && b.prev === 'aruco' && b.pressed === 'false' && !b.disabled && b.title === L.tip, `${tag} depth toggle in the selector after ArUco, enabled, released, tip "${b.title}"`);

    // ---- one tap: one map
    const t0 = performance.now();
    await tapSel('#vmode button[data-vm="depth"]');
    await page.waitForFunction(() => !!(DEPTH.map && DEPTH.map.img), null, { timeout: 4000 }).catch(() => {});
    await page.evaluate(() => visPopOpen(false));
    await page.waitForTimeout(500);
    ok(reqsSince(t0).length === 1, `${tag} one tap -> one request (${reqsSince(t0).length})`);
    const pan = await page.evaluate(() => { const p = document.getElementById('depth-panel'); p.scrollIntoView({ block: 'nearest' }); const r = p.getBoundingClientRect(), v = document.getElementById('video-box').getBoundingClientRect();
      return { shown: !p.hidden && r.width > 0, title: document.getElementById('dp-title').textContent, near: document.getElementById('dp-near').textContent, far: document.getElementById('dp-far').textContent,
        nearM: document.getElementById('dp-near-m').textContent, farM: document.getElementById('dp-far-m').textContent, note: document.getElementById('dp-note').textContent,
        status: document.getElementById('dp-status').textContent, inView: r.left >= 0 && r.right <= innerWidth + 0.5 && r.top >= 0 && r.bottom <= innerHeight + 0.5
          && r.left >= v.left && r.right <= v.right && r.top >= v.top && r.bottom <= v.bottom, share: (r.width * r.height) / (v.width * v.height),
        sw: document.scrollingElement.scrollWidth, iw: innerWidth, aria: document.querySelector('#vmode button[data-vm="depth"]').getAttribute('aria-pressed') }; });
    ok(pan.shown && pan.inView, `${tag} panel visible inside the video box and the viewport`);
    ok(pan.share <= 0.35, `${tag} panel covers ${(pan.share * 100).toFixed(0)} % of the video (<= 35 %)`);
    ok(pan.title === L.title, `${tag} panel title "${pan.title}"`);
    ok(pan.near === L.near && pan.far === L.far, `${tag} legend near / far "${pan.near}" / "${pan.far}"`);
    ok(pan.nearM === L.m(STATS.p2_mm) && pan.farM === L.m(STATS.p98_mm), `${tag} legend metres "${pan.nearM}" / "${pan.farM}" want p2 ${STATS.p2_mm} / p98 ${STATS.p98_mm}`);
    ok(pan.note === L.note, `${tag} not-calibrated note "${pan.note}"`);
    ok(pan.status.startsWith(L.frame + frameId), `${tag} frame / age "${pan.status}"`);
    ok(pan.aria === 'true', `${tag} depth toggle pressed`);
    ok(pan.sw <= pan.iw, `${tag} no horizontal scroll (${pan.sw} > ${pan.iw})`);

    // ---- colour map alignment
    for (const [gx, gy] of [[99, 70], [100, 70], [120, 99], [120, 100], [2, 150], [381, 30], [300, 60]]) {
      const pt = await cellPt(page, gx, gy);
      const r = await pixelAt(page, pt, cellMm(gx, gy));
      ok(same(r.px, r.want), `${tag} cell (${gx},${gy}) ${cellMm(gx, gy)} mm colour ${r.px} want ${r.want}`);
    }
    {
      const r = await pixelAt(page, await cellPt(page, 265, 115), 0);
      ok(r.px[3] === 0, `${tag} empty block transparent (alpha ${r.px[3]})`);
      const near = await pixelAt(page, await cellPt(page, 120, 70), 250), far = await pixelAt(page, await cellPt(page, 381, 30), 599);
      ok(near.px[0] > near.px[2] && far.px[2] > far.px[0], `${tag} near is warm (${near.px}), far is cool (${far.px})`);
    }
    // ---- no pointer events on the map canvas
    const pe = await page.evaluate(() => { const cv = document.getElementById('depth-overlay'), r = cv.getBoundingClientRect(); const at = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
      return { pe: getComputedStyle(cv).pointerEvents, notCanvas: at !== cv }; });
    ok(pe.pe === 'none' && pe.notCanvas, `${tag} map canvas takes no pointer events (${pe.pe})`);

    // ---- readout at known points (tap; hover with a mouse); the video on screen first (phone landscape scrolls)
    await page.evaluate(() => document.getElementById('video-box').scrollIntoView({ block: 'start' }));
    // interior cells of uniform regions (a tap lands on whole CSS pixels; a phone cell is about 1 px wide); the
    // gradient changes 0.78 mm per cell, so +-1 cell is +-1 mm there
    for (const [gx, gy, tol] of [[125, 75, 0], [200, 120, 1], [265, 115, 0], [230, 115, 1]]) {
      const pt = await cellPt(page, gx, gy);
      const free = await page.evaluate((p) => { const at = document.elementFromPoint(p.x, p.y); return !!at && !at.closest('.depth-panel, .hud, .view-switch, .grasp-bar'); }, pt);
      ok(free, `${tag} readout point (${gx},${gy}) not covered by a control`);
      await page.evaluate(() => { document.getElementById('depth-read').textContent = ''; });
      await tap(pt);
      await page.waitForTimeout(80);
      const rd = await page.evaluate(() => { const e = document.getElementById('depth-read'); return { shown: !e.hidden && e.getBoundingClientRect().width > 0, txt: e.textContent }; });
      const want = cellMm(gx, gy);
      const got = Number((rd.txt.match(/(\d+) mm/) || [])[1]);
      const good = want ? Math.abs(got - want) <= tol && rd.txt === L.read(got) : rd.txt === L.none;
      ok(rd.shown && good, `${tag} tap readout at (${gx},${gy}) "${rd.txt}" want ${want ? L.read(want) + (tol ? ' +-' + tol : '') : L.none}`);
    }
    if (!mobile) {
      const pt = await cellPt(page, 125, 60);
      await page.mouse.move(pt.x, pt.y);
      await page.waitForTimeout(80);
      const txt = await page.evaluate(() => document.getElementById('depth-read').textContent);
      ok(txt === L.read(cellMm(125, 60)), `${tag} hover readout "${txt}"`);
    }

    // ---- opacity
    await page.evaluate(() => { const r = document.getElementById('dp-op'); r.value = '30'; r.dispatchEvent(new Event('input', { bubbles: true })); });
    ok(await page.evaluate(() => document.getElementById('depth-overlay').style.opacity === '0.3'), `${tag} opacity slider -> canvas opacity 0.3`);
    await page.evaluate(() => { const r = document.getElementById('dp-op'); r.value = '60'; r.dispatchEvent(new Event('input', { bubbles: true })); });
    await page.waitForTimeout(200);
    const f1 = path.join(out, `${tag}_map.png`); await page.screenshot({ path: f1 }); shots.push(f1);

    // ---- E-stop reachable with the panel open
    const es = await page.evaluate(() => { const e = document.getElementById('btn-estop'), b = e.getBoundingClientRect(); const c = { x: b.left + b.width / 2, y: b.top + b.height / 2 };
      if (!(b.width > 0) || c.x < 0 || c.y < 0 || c.x >= innerWidth || c.y >= innerHeight) { e.scrollIntoView({ block: 'center' }); const b2 = e.getBoundingClientRect(); c.x = b2.left + b2.width / 2; c.y = b2.top + b2.height / 2; }
      const at = document.elementFromPoint(c.x, c.y); window.__estop = 0; return { c, top: !!at && e.contains(at) }; });
    if (es.top) await tap(es.c);
    await page.waitForTimeout(80);
    const hit = await page.evaluate(() => window.__estop);
    ok(es.top && hit >= 1, `${tag} E-stop on top and a tap runs it (on top ${es.top}, ran ${hit})`);
    await page.evaluate(() => window.scrollTo(0, 0));

    // ---- Live: at most 1 per second
    const tl = performance.now();
    await page.evaluate(() => document.getElementById('dp-live').click());
    await page.waitForTimeout(3600);
    const live = reqsSince(tl), gaps = live.slice(1).map((t, i) => t - live[i]);
    ok(live.length >= 2 && live.length <= 4, `${tag} Live: ${live.length} requests in 3.6 s`);
    ok(gaps.every((g) => g >= 1000), `${tag} Live: start-to-start gaps at the server ${gaps.map((g) => g.toFixed(0))} ms >= 1000`);
    ok(D.n429 === 0, `${tag} Live: the page never ran into the server's 1/s limit (${D.n429} x 429)`);
    // a forced 429 is retried after Retry-After
    D.force429 = true;
    const t429 = performance.now();
    await page.waitForTimeout(3200);
    const after429 = reqsSince(t429);
    ok(D.n429 === 1 && after429.length >= 2 && after429[1] - after429[0] >= 1000, `${tag} 429 retried after Retry-After (${after429.map((t) => (t - t429).toFixed(0))})`);
    D.n429 = 0;
    const fl = path.join(out, `${tag}_live.png`); await page.screenshot({ path: fl }); shots.push(fl);

    // ---- the arm moves: map dropped, Live off, no more requests
    P.q = REST.map((v, i) => (i === 0 ? v + 0.05 : v));
    await page.waitForTimeout(400);
    const tm = performance.now();
    const mv = await page.evaluate(() => ({ map: !!DEPTH.map, live: document.getElementById('dp-live').getAttribute('aria-pressed'), st: document.getElementById('dp-status').textContent }));
    ok(!mv.map && mv.live === 'false', `${tag} arm moved: map dropped (${mv.map}), Live off (${mv.live})`);
    ok(mv.st === L.moved, `${tag} arm moved status "${mv.st}"`);
    await page.waitForTimeout(1500);
    // a request already in flight when the arm moved may still finish; none may start afterwards
    ok(reqsSince(tm + 200).length === 0, `${tag} arm moved: no requests afterwards (${reqsSince(tm + 200).length})`);
    const fm = path.join(out, `${tag}_moved.png`); await page.screenshot({ path: fm }); shots.push(fm);

    // ---- hidden page pauses Live, visible resumes
    await page.evaluate(() => document.getElementById('dp-live').click());
    await page.waitForTimeout(1500);
    const setHidden = (h) => page.evaluate((h) => {
      Object.defineProperty(document, 'hidden', { value: h, configurable: true });
      Object.defineProperty(document, 'visibilityState', { value: h ? 'hidden' : 'visible', configurable: true });
      document.dispatchEvent(new Event('visibilitychange'));
    }, h);
    await setHidden(true);
    await page.waitForTimeout(300);
    const th = performance.now();
    await page.waitForTimeout(2200);
    ok(reqsSince(th).length === 0, `${tag} hidden: no requests (${reqsSince(th).length})`);
    await setHidden(false);
    const tv = performance.now();
    await page.waitForTimeout(1500);
    ok(reqsSince(tv).length >= 1, `${tag} visible again: Live resumes (${reqsSince(tv).length})`);

    // ---- leaving the depth view stops it
    await page.evaluate(() => document.getElementById('dp-x').click());
    await page.waitForTimeout(300);
    const tx = performance.now();
    await page.waitForTimeout(2200);
    const gone = await page.evaluate(() => ({ panel: document.getElementById('depth-panel').hidden, cv: document.getElementById('depth-overlay').hidden, aria: document.querySelector('#vmode button[data-vm="depth"]').getAttribute('aria-pressed'),
      off: document.querySelector('#vmode button[data-vm="off"]').getAttribute('aria-pressed') }));
    ok(gone.panel && gone.cv && gone.aria === 'false' && gone.off === 'true', `${tag} closed: panel and map hidden, toggle released, Off shown`);
    ok(reqsSince(tx).length === 0, `${tag} closed: no requests (${reqsSince(tx).length})`);
    // the toggle again: on (one map), then off from the selector itself
    await openVis();
    await page.waitForTimeout(150);
    const t2 = performance.now();
    // the earlier steps scrolled the page's scroller (phone landscape): bring the selector back
    await page.evaluate(() => document.querySelector('#vmode button[data-vm="depth"]').scrollIntoView({ block: 'nearest' }));
    await tapSel('#vmode button[data-vm="depth"]');
    await page.waitForTimeout(600);
    const re = await page.evaluate(() => ({ on: DEPTH.on, panel: !document.getElementById('depth-panel').hidden }));
    ok(re.on && re.panel && reqsSince(t2).length === 1, `${tag} toggle on again: panel and one request (${reqsSince(t2).length})`);
    await openVis();
    await page.waitForTimeout(150);
    await page.evaluate(() => document.querySelector('#vmode button[data-vm="depth"]').scrollIntoView({ block: 'nearest' }));
    await tapSel('#vmode button[data-vm="depth"]');
    await page.waitForTimeout(300);
    const t3 = performance.now();
    await page.waitForTimeout(1500);
    const re2 = await page.evaluate(() => ({ on: DEPTH.on, panel: !document.getElementById('depth-panel').hidden }));
    ok(!re2.on && !re2.panel && reqsSince(t3).length === 0, `${tag} toggle off from the selector: panel hidden, no requests`);

    ok(errs.length === 0, `${tag} page errors: ${errs.join(' | ')}`);
    await ctx.close();
  }
} finally {
  await browser.close();
  srv.close();
}
console.log(shots.join('\n'));
console.log(bad ? `${bad}/${checks} checks FAILED` : `${checks} checks OK`);
process.exit(bad ? 1 : 0);
