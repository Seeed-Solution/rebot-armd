#!/usr/bin/env node
// Headless check of the cockpit vision-mode selector and the "both" overlay (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/vision_mode_shots.mjs
// Serves src/api/index.html with a stateful mock of rebot-vision 1.3 (perception API 2:
// /api/v1/vision/mode, /subscribe, /subscribe/<id>/keepalive, DELETE /subscribe/<id>, /detection/trigger)
// and /api/v1/detection/inference, plus GET /api/v1/vision/depth (a synthetic RDM1 map). For simple / expert x zh / en x
// 1280x800 / 375x812 it checks the selector, a group of aria-pressed toggles (Off / YOLO / ArUco / Depth (est.), prefixed
// "Show" under API 2; no "Both" button): YOLO and ArUco on together create this page's overlay subscription with
// ["yolo","aruco"] (kept alive, updated in place on a new choice, deleted on Off, on Detect off and while the page is
// hidden) and never POST the global mode; single shot sends the selected pipelines; YOLO + ArUco + Depth draw boxes,
// markers and the depth map together, Depth never enters a subscription; Off clears all three; YOLO is disabled without
// a model; Depth is disabled with a reason without a depth model and when the provider's ION budget (models.catalogue
// ion_mb, ion) would be exceeded; an API 1 provider gets the device mode of the YOLO / ArUco set (both / yolo / aruco /
// off) and never a depth mode; a provider without the route (404) hides the selector.
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
// V.depth: depth model resident and metric; V.models / V.ion: the model catalogue and ION budget (null = not reported)
const V0 = { mode: 'yolo', enabled: false, yolo: true, route: true, v2: true, depth: true, metric: true, models: null, ion: null };
let V = { ...V0 };
const DEPTHREQ = []; // GET /api/v1/vision/depth times
const POSTS = [];   // POST /api/v1/vision/mode bodies
const CALLS = [];   // subscription and trigger requests: [method, path, body]
const SUBS = new Map(); let nextSub = 1;
const vbody = () => ({ api_version: V.v2 ? 2 : 1, mode: V.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: V.enabled, fps: V.enabled || SUBS.size ? 5 : 1,
  active_fps: 5, idle_fps: 1, trigger: true, available: { yolo: V.yolo, aruco: true, depth: V.depth },
  depth_model: V.depth ? { name: 'zipdepth', path: '/userdata/local/models/zipdepth.cvimodel', metric: V.metric } : null,
  ...(V.models ? { models: V.models } : {}), ...(V.ion ? { ion: V.ion } : {}),
  ...(V.v2 ? { per_request_pipelines: true, subscriptions: true } : {}) });
// RDM1 depth map 384x224: 300 mm at the left edge to 600 mm at the right
const DW = 384, DH = 224;
function depthBody() {
  let js = JSON.stringify({ format: 'u16le_mm', width: DW, height: DH, units: 'mm', invalid: 0, frame_id: 900 + DEPTHREQ.length, ts_ms: Date.now(), model: 'zipdepth',
    mapping: { image_w: 640, image_h: 360, fit: 'stretch', x0: 0, y0: 0, x1: 640, y1: 360 }, stats: { p2_mm: 306, p98_mm: 594 } });
  while ((8 + Buffer.byteLength(js)) % 8) js += ' ';
  const jb = Buffer.from(js), buf = Buffer.alloc(8 + jb.length + DW * DH * 2);
  buf.write('RDM1', 0, 'latin1'); buf.writeUInt32LE(jb.length, 4); jb.copy(buf, 8);
  for (let y = 0; y < DH; y++) for (let x = 0; x < DW; x++) buf.writeUInt16LE(Math.round(300 + 300 * x / (DW - 1)), 8 + jb.length + 2 * (y * DW + x));
  return buf;
}
const json = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    if (u === '/api/v1/vision/depth') {
      DEPTHREQ.push(performance.now());
      if (!V.depth) return json(res, 409, { api_version: 2, error: 'depth unavailable' });
      res.writeHead(200, { 'content-type': 'application/octet-stream', 'cache-control': 'no-store' }); res.end(depthBody()); return;
    }
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
const T = { zh: ['关', 'YOLO', 'ArUco', '深度（估计）'], en: ['Off', 'YOLO', 'ArUco', 'Depth (est.)'] };
// tooltips of disabled toggles
const TIP = { zh: { noyolo: '未加载 YOLO 模型', nodepth: '未加载深度模型', depth: '深度', ion: (w, n, l) => `${w}不可用：ION 预算不足（需要 ${n} MB，上限 ${l} MB）` },
  en: { noyolo: 'No YOLO model loaded', nodepth: 'No depth model loaded', depth: 'Depth', ion: (w, n, l) => `${w} unavailable: over the ION budget (needs ${n} MB of ${l} MB)` } };
// the catalogue as rebot-vision reports it (docs/detection_api.md "Model switching"), with `resident` per entry
const CAT = (res) => [{ name: 'depth', kind: 'depth', ion_mb: 20.5, metric: true }, { name: 'det', kind: 'detect', ion_mb: 8.5 }, { name: 'seg', kind: 'segment', ion_mb: 19.5 }]
  .map((e) => ({ ...e, resident: res.includes(e.name) }));
const ION = (res) => ({ base_mb: 21.5, used_mb: 21.5 + CAT(res).filter((e) => e.resident).reduce((a, e) => a + e.ion_mb, 0), total_mb: 62.9, reserve_mb: 8.0 });
// perception API 2: the selector is this page's overlay ("Show" prefix); API 1: the device mode (no prefix)
const P = { zh: ['显示', '本页画面显示', '视觉模式'], en: ['Show', 'Shown on this page', 'Vision mode'] };
const hud = (page) => page.evaluate(() => {
  const w = document.getElementById('vmode'), r = w.getBoundingClientRect(), vb = document.getElementById('video-box').getBoundingClientRect();
  const k = document.getElementById('vm-k'), kr = k.getBoundingClientRect();
  const bs = [...w.querySelectorAll('button')];
  return { shown: !w.closest('[hidden]') && r.width > 0, prefix: !k.hidden && kr.width > 0 ? k.textContent : null, aria: w.getAttribute('aria-label'), labels: bs.map((b) => b.textContent),
    on: bs.filter((b) => b.classList.contains('on')).map((b) => b.dataset.vm), disabled: bs.filter((b) => b.disabled).map((b) => b.dataset.vm),
    pressed: bs.filter((b) => b.getAttribute('aria-pressed') === 'true').map((b) => b.dataset.vm), tips: Object.fromEntries(bs.map((b) => [b.dataset.vm, b.title])),
    a11y: w.getAttribute('role') === 'group' && bs.every((b) => !b.hasAttribute('role') && ['true', 'false'].includes(b.getAttribute('aria-pressed'))),
    depth: { on: DEPTH.on, map: !!DEPTH.map, panel: !document.getElementById('depth-panel').hidden, cv: !document.getElementById('depth-overlay').hidden },
    inside: r.left >= vb.left - 1 && r.right <= vb.right + 1, det: document.getElementById('btn-det').textContent,
    sw: document.scrollingElement.scrollWidth, iw: window.innerWidth };
});

// simple mode, and expert mode on a phone: Detect, the vision mode selector, single shot and grasp setup live in the
// recognition popover (#btn-vis)
const openVis = (page) => page.evaluate(() => {
  const b = document.getElementById('btn-vis');
  if (!document.body.classList.contains('vis-open') && b && !b.hidden && getComputedStyle(b).display !== 'none') b.click();
});
const subCalls = () => CALLS.filter(([, u]) => u.startsWith('/api/v1/vision/subscribe'));
const setHidden = (page, hidden) => page.evaluate((h) => {
  Object.defineProperty(document, 'hidden', { value: h, configurable: true });
  Object.defineProperty(document, 'visibilityState', { value: h ? 'hidden' : 'visible', configurable: true });
  document.dispatchEvent(new Event('visibilitychange'));
}, hidden);

for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const tag = `${ui}_${lang}_${vp}`;
  V = { ...V0 }; POSTS.length = 0; CALLS.length = 0; SUBS.clear(); DEPTHREQ.length = 0;
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  const tog = async (m, ms = 300) => { await openVis(page); await page.click(`#vmode button[data-vm="${m}"]`); await page.waitForTimeout(ms); };
  const ovPx = () => page.evaluate(() => { const c = ov.cv, d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++; return n; });

  // 1. standby: no subscription, selector shows Off (this page's overlay is off); toggles in a group, no Both
  await openVis(page);
  let h = await hud(page);
  check(`${tag} selector`, h.shown && JSON.stringify(h.labels) === JSON.stringify(T[lang]) && h.on.join() === 'off' && h.pressed.join() === 'off' && h.inside && h.a11y && !h.disabled.length, JSON.stringify(h));
  check(`${tag} show prefix`, h.prefix === P[lang][0] && h.aria === P[lang][1], JSON.stringify(h));
  check(`${tag} standby`, SUBS.size === 0 && subCalls().length === 0, JSON.stringify(CALLS));
  let file = path.join(out, `vision_standby_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 2. YOLO then ArUco: one subscription {client, pipelines, ttl_ms}, updated in place to [yolo, aruco]; box + marker; kept alive
  await tog('yolo');
  await tog('aruco', 1300);
  await openVis(page);
  h = await hud(page);
  const ovd = await page.evaluate(() => ({ n: S.detections.length, aruco: S.detections.filter((d) => d.aruco).length, labels: S.detections.map((d) => d.label) }));
  const sc = subCalls();
  const create = sc[0] || [];
  const sid = [...SUBS.keys()][0];
  check(`${tag} yolo+aruco subscribe`, create[0] === 'POST' && create[1] === '/api/v1/vision/subscribe' && JSON.stringify(create[2].pipelines) === '["yolo"]'
    && /^cockpit-overlay-/.test(create[2].client) && create[2].ttl_ms === 3000 && SUBS.size === 1 && JSON.stringify(SUBS.get(sid).pipelines) === '["yolo","aruco"]', JSON.stringify(sc));
  check(`${tag} keepalive`, sc.some(([m, u]) => m === 'POST' && u === `/api/v1/vision/subscribe/${sid}/keepalive`), JSON.stringify(sc));
  check(`${tag} no global mode`, POSTS.length === 0 && !V.enabled, JSON.stringify(POSTS));
  check(`${tag} yolo+aruco state`, h.on.join() === 'yolo,aruco' && h.pressed.join() === 'yolo,aruco', JSON.stringify(h));
  check(`${tag} yolo+aruco overlay`, ovd.n === 2 && ovd.aruco === 1, JSON.stringify(ovd));
  check(`${tag} pipes remembered`, await page.evaluate(() => localStorage.getItem('rebot.vision.pipes')) === 'both');
  check(`${tag} no h-scroll`, h.sw <= h.iw, JSON.stringify(h));
  file = path.join(out, `vision_both_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  hud=${JSON.stringify(h.labels)} on=${h.on} det="${h.det}" overlay=${JSON.stringify(ovd)} subs=${JSON.stringify([...SUBS])} calls=${sc.length} scroll=${h.sw}<=${h.iw}`);

  // 3. single shot with the selected pipelines
  CALLS.length = 0;
  await openVis(page); await page.click('#btn-trigger'); await page.waitForTimeout(300);
  const trig = CALLS.find(([, u]) => u === '/api/v1/detection/trigger');
  check(`${tag} trigger body`, trig && JSON.stringify(trig[2]) === '{"pipelines":["yolo","aruco"]}', JSON.stringify(CALLS));

  // 4. + Depth: the three overlays together (map under boxes and markers); the subscription is unchanged
  CALLS.length = 0;
  await tog('depth', 600);
  await page.waitForTimeout(500);
  await openVis(page);
  h = await hud(page);
  const comb = await page.evaluate(() => ({ n: S.detections.length, aruco: S.detections.filter((d) => d.aruco).length,
    depthPx: (() => { const c = document.getElementById('depth-overlay'), d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++; return n; })(),
    under: (() => { const a = document.getElementById('depth-overlay'), b = document.getElementById('video-overlay'); return !!(a.compareDocumentPosition(b) & Node.DOCUMENT_POSITION_FOLLOWING); })() }));
  const px = await ovPx();
  check(`${tag} combined state`, h.on.join() === 'yolo,aruco,depth' && h.pressed.join() === 'yolo,aruco,depth' && h.depth.on && h.depth.map && h.depth.panel && h.depth.cv, JSON.stringify(h));
  check(`${tag} combined overlays drawn`, comb.n === 2 && comb.aruco === 1 && px > 200 && comb.depthPx > 1000 && comb.under, JSON.stringify({ comb, px }));
  check(`${tag} depth fetched, not subscribed`, DEPTHREQ.length === 1 && subCalls().every(([m, , j]) => m !== 'POST' || !j || !(j.pipelines || []).includes('depth'))
    && JSON.stringify(SUBS.get(sid).pipelines) === '["yolo","aruco"]' && POSTS.length === 0, JSON.stringify({ d: DEPTHREQ.length, calls: subCalls(), POSTS }));
  file = path.join(out, `vision_combined_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 5. YOLO off: ArUco in place, Depth stays; Off clears all three
  await tog('yolo', 300);
  h = await hud(page);
  check(`${tag} yolo off`, SUBS.size === 1 && JSON.stringify(SUBS.get(sid).pipelines) === '["aruco"]' && h.on.join() === 'aruco,depth' && h.depth.on, JSON.stringify({ subs: [...SUBS], h }));
  await tog('off', 400);
  await openVis(page);
  h = await hud(page);
  const n = await page.evaluate(() => S.detections.length);
  const dAfterOff = DEPTHREQ.length;
  check(`${tag} off clears all`, SUBS.size === 0 && subCalls().some(([m, u]) => m === 'DELETE' && u === `/api/v1/vision/subscribe/${sid}`) && h.on.join() === 'off' && h.pressed.join() === 'off' && n === 0
    && !h.depth.on && !h.depth.map && !h.depth.panel && !h.depth.cv && POSTS.length === 0, JSON.stringify({ calls: subCalls(), h, n }));

  // 6. Detect toggle from Off subscribes with the last pipelines (ArUco); page hide deletes, visible again re-subscribes
  await openVis(page); await page.click('#btn-det'); await page.waitForTimeout(300);
  await openVis(page);
  h = await hud(page);
  check(`${tag} detect from off`, SUBS.size === 1 && JSON.stringify([...SUBS.values()][0].pipelines) === '["aruco"]' && h.on.join() === 'aruco', JSON.stringify({ subs: [...SUBS], h }));
  await setHidden(page, true); await page.waitForTimeout(300);
  check(`${tag} hidden deletes`, SUBS.size === 0, JSON.stringify([...SUBS]));
  await setHidden(page, false); await page.waitForTimeout(300);
  check(`${tag} visible resubscribes`, SUBS.size === 1, JSON.stringify([...SUBS]));
  await openVis(page); await page.click('#btn-det'); await page.waitForTimeout(300);
  check(`${tag} detect off`, SUBS.size === 0 && POSTS.length === 0 && DEPTHREQ.length === dAfterOff, JSON.stringify({ subs: [...SUBS], POSTS, d: DEPTHREQ.length }));

  // 7. no YOLO model: YOLO disabled with a reason
  V.yolo = false;
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  await openVis(page);
  h = await hud(page);
  check(`${tag} no model`, h.disabled.join() === 'yolo' && h.tips.yolo === TIP[lang].noyolo, JSON.stringify(h));
  if (ui === 'expert') { file = path.join(out, `vision_nomodel_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }
  // no YOLO model and only the segment model (19.5) in the catalogue next to the resident depth model: 21.5 + 20.5 + 19.5 + 8 = 69.5 > 62.9
  V.models = { catalogue: CAT(['depth']).filter((e) => e.name !== 'det') }; V.ion = ION(['depth']);
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  await openVis(page);
  h = await hud(page);
  check(`${tag} yolo over budget`, h.disabled.join() === 'yolo' && h.tips.yolo === TIP[lang].ion('YOLO', '69.5', '62.9'), JSON.stringify(h.tips));
  V.models = null; V.ion = null;

  // 8. Depth availability and the ION budget: the toggle is disabled with the reason
  V.yolo = true;
  const depthCase = async (name, patch, wantTip) => {
    Object.assign(V, patch);
    await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
    await openVis(page);
    const g = await hud(page);
    const dis = g.disabled.includes('depth');
    check(`${tag} depth ${name}`, wantTip ? dis && g.tips.depth === wantTip : !dis && !g.disabled.length, JSON.stringify({ dis: g.disabled, tip: g.tips.depth }));
    return g;
  };
  // detect + depth resident: 21.5 + 8.5 + 20.5 + reserve 8 = 58.5 <= 62.9 (the device's default set) -> available
  await depthCase('det+depth fits', { depth: true, metric: true, models: { catalogue: CAT(['det', 'depth']) }, ion: ION(['det', 'depth']) }, null);
  // no depth model: the catalogue's depth (20.5) would fit next to det -> "no depth model"
  await depthCase('no model', { depth: false, models: { catalogue: CAT(['det']) }, ion: ION(['det']) }, TIP[lang].nodepth);
  // segment loaded, depth not loaded: 21.5 + 19.5 + 20.5 + 8 = 69.5 > 62.9
  await depthCase('seg over budget', { depth: false, models: { catalogue: CAT(['seg']) }, ion: ION(['seg']) }, TIP[lang].ion(TIP[lang].depth, '69.5', '62.9'));
  // an older provider that loaded segment + depth anyway (used 61.5 + reserve 8 > 62.9)
  const gs = await depthCase('seg+depth resident over budget', { depth: true, models: { catalogue: CAT(['seg', 'depth']) }, ion: ION(['seg', 'depth']) }, TIP[lang].ion(TIP[lang].depth, '69.5', '62.9'));
  check(`${tag} depth over budget stays off`, !gs.depth.on && !gs.pressed.includes('depth'), JSON.stringify(gs));
  if (ui === 'expert' || vp === '375x812') { file = path.join(out, `vision_depth_budget_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }
  // a relative depth model
  await depthCase('relative', { depth: true, metric: false, models: null, ion: null }, lang === 'zh' ? '深度图需要度量深度模型（当前为相对深度）' : 'The depth map needs a metric depth model (the loaded one is relative)');
  Object.assign(V, { metric: true });
  // Depth on, then the depth model goes (swapped out): the view ends
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  await tog('depth', 400);
  await depthCase('unloaded while on', { depth: false }, TIP[lang].nodepth);
  h = await hud(page);
  check(`${tag} depth ends when unloaded`, !h.depth.on && !h.depth.panel, JSON.stringify(h.depth));
  Object.assign(V, { depth: true });

  // 9. API 1 provider (no subscriptions): the YOLO / ArUco set maps to the device mode; Depth never goes there
  V = { ...V, v2: false, yolo: true, mode: 'yolo', enabled: false };
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  POSTS.length = 0;
  await tog('aruco');
  check(`${tag} api1 yolo+aruco -> both`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'both', enabled: true }) && V.enabled, JSON.stringify(POSTS));
  await openVis(page);
  h = await hud(page);
  check(`${tag} api1 no prefix`, h.prefix === null && h.aria === P[lang][2] && h.on.join() === 'yolo,aruco', JSON.stringify(h));
  await tog('yolo');
  check(`${tag} api1 aruco -> aruco`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'aruco', enabled: true }), JSON.stringify(POSTS));
  const np = POSTS.length;
  await tog('depth', 500);
  h = await hud(page);
  check(`${tag} api1 depth page-only`, POSTS.length === np && h.on.join() === 'aruco,depth' && h.depth.map && V.mode === 'aruco', JSON.stringify({ POSTS, h }));
  await tog('yolo');
  check(`${tag} api1 +yolo -> both`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'both', enabled: true }), JSON.stringify(POSTS));
  await tog('off', 400);
  h = await hud(page);
  check(`${tag} api1 off -> off`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'off' }) && V.mode === 'off' && h.on.join() === 'off' && !h.depth.on, JSON.stringify({ POSTS, h }));
  await tog('yolo');
  check(`${tag} api1 yolo -> yolo`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'yolo', enabled: true }), JSON.stringify(POSTS));
  await tog('yolo');
  check(`${tag} api1 none -> off`, JSON.stringify(POSTS.at(-1)) === JSON.stringify({ mode: 'off' }), JSON.stringify(POSTS));
  check(`${tag} api1 modes valid`, POSTS.every((j) => ['off', 'yolo', 'aruco', 'both'].includes(j.mode)), JSON.stringify(POSTS));

  // 10. provider without /api/mode: selector hidden, Detect stays
  V.route = false;
  await page.evaluate(() => window.__vision.vmRefresh()); await page.waitForTimeout(150);
  await openVis(page);
  h = await hud(page);
  check(`${tag} old provider hides selector`, !h.shown, JSON.stringify(h));

  if (errs.length) { bad++; console.log(`  page errors (${tag}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`); }
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} of ${checks} checks failed` : `${checks} checks, ${shots.length} screenshots OK (${path.relative(root, out)})`);
process.exit(bad ? 1 : 0);
