#!/usr/bin/env node
// Headless check of the grasp setup drawer and the single-shot detection overlay (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/vision_polish_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF schema, a grasp setup as a calibrated B601-DM reports it
// (camera_tilt_deg -36.995338439941406, grip_pixel_norm [0.5, 0.5]) and a perception API 2 rebot-vision
// mock whose global mode is "yolo"; replaces the control WebSocket with an in-page mock that
// acknowledges commands. For simple x zh / en x 1280x800 / 375x812 (and expert at 1280x800) it checks:
//   - grasp setup drawer: the tilt field shows 1 decimal; Save without an edit sends no camera_tilt_deg
//   - camera_down true: the grip point reads "not used: camera-down acquisition", the pick button is
//     hidden and no grip ring is drawn; camera_down false: the value, the pick button and the ring
//   - vision mode selector: with the provider's global mode "yolo" and this page's overlay off, the
//     selector reads "Show: Off" (it is this page's overlay, not the device mode)
// Writes PNGs (default .cache/vision-polish-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/vision-polish-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5, 5, 3]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const TILT = -36.995338439941406;
const SETUP = { table_z_m: 0.012, approach_height_m: 0.15, grasp_height_m: 0.015, lift_height_m: 0.12, grip_pixel_norm: [0.5, 0.5], camera_tilt_deg: TILT,
  image_size_px: [640, 360], fx_px: 600, fy_px: 600, centre_tol_px: 10, acquire_camera_down: 'auto' };
const V = { mode: 'yolo', subs: new Map(), trig: 0 };
const json = (res, o, code = 200) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS });
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    const j = b ? JSON.parse(b) : null;
    if (u === '/api/v1/detection/inference') return json(res, { enabled: false, fps: 1, active_fps: 5, idle_fps: 1, mode: V.mode });
    if (u === '/api/v1/vision/mode') return json(res, { api_version: 2, mode: V.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: 1, active_fps: 5, idle_fps: 1,
      trigger: true, available: { yolo: true, aruco: true }, per_request_pipelines: true, subscriptions: true });
    if (u === '/api/v1/vision/subscribe' && req.method === 'POST') { V.subs.set('s1', j.pipelines); return json(res, { api_version: 2, id: 's1', client: j.client, pipelines: j.pipelines, fps: 5, ttl_ms: j.ttl_ms }); }
    if (u.startsWith('/api/v1/vision/subscribe/')) { if (req.method === 'DELETE') V.subs.delete('s1'); return json(res, { api_version: 2, id: 's1' }); }
    if (u === '/api/v1/detection/trigger') {
      V.trig++;
      return json(res, { api_version: 2, success: true, frame_id: 100 + V.trig, width: 640, height: 360, pipelines: { yolo: { ran: true }, aruco: { ran: false } },
        detections: [{ class_id: 41, format: 'xywh_center', class_name: 'cup', confidence: 0.91, x: 250 / 640, y: 200 / 360, w: 100 / 640, h: 120 / 360 }], markers: [] });
    }
    res.writeHead(404); res.end();
  });
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

// In-page mock: arm observation (window.__q), a fake video frame, the grasp setup endpoint and a
// command-acknowledging socket.
const feed = ([setup, camDown]) => {
  window.__sent = [];
  window.__setup = setup;
  window.__camDown = camDown;
  window.__q = [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, -5.6];
  window.connectWS = () => {};
  if (S.ws && S.ws.close) { const old = S.ws; old.onclose = null; old.onerror = null; old.onmessage = null; try { old.close(); } catch (_) {} }
  const fake = {
    readyState: 1, close() {},
    send(m) {
      const o = JSON.parse(m);
      window.__sent.push(o);
      if (o.type !== 'command') return;
      setTimeout(() => {
        const p = pendingCommands.get(o.request_id);
        if (!p) return;
        clearTimeout(p.timer); pendingCommands.delete(o.request_id);
        const ack = { type: 'command_ack', request_id: o.request_id, status: 'accepted', epoch_id: 1 };
        if (o.path === '/api/v1/grasp/setup') { window.__setup = { ...window.__setup, ...o.body }; ack.setup = window.__setup; ack.persisted = true; }
        p.resolve(ack);
      }, 15);
    },
  };
  const realFetch = window.fetch.bind(window);
  window.fetch = (u, o) => (String(u).startsWith('/api/v1/grasp/setup')
    ? Promise.resolve(new Response(JSON.stringify({ supported: true, setup: window.__setup, path: '/userdata/rebot-teleop/grasp_setup.json', revision: 1,
      ...(window.__camDown === undefined ? {} : { camera_down: window.__camDown }) }), { status: 200, headers: { 'content-type': 'application/json' } }))
    : realFetch(u, o));
  const poster = document.createElement('canvas');
  poster.width = 640; poster.height = 360;
  const g = poster.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 360); grad.addColorStop(0, '#b9a38a'); grad.addColorStop(1, '#8f7a63');
  g.fillStyle = grad; g.fillRect(0, 0, 640, 360);
  g.fillStyle = '#e8eef4'; g.beginPath(); g.ellipse(250, 200, 48, 58, 0, 0, Math.PI * 2); g.fill();
  const v = document.getElementById('video-stream');
  v.poster = poster.toDataURL();
  Object.defineProperty(v, 'videoWidth', { get: () => 640 });
  Object.defineProperty(v, 'videoHeight', { get: () => 360 });
  setInterval(() => {
    if (S.ws !== fake) S.ws = fake;
    const q = window.__q;
    const o = { positions: q.slice(), commanded_positions: q.slice(), velocities: [0, 0, 0, 0, 0, 0, 0], safety_state: 'Armed', enable_state: 'Enabled', is_torqued: true,
      feedback_age_ms: [5, 5, 5, 5, 5, 5, 5], epoch_id: 1, timestamp: performance.now() / 1000, action: null, previous_action: null };
    S.obs = o; S.obsAt = Date.now(); S.state = 'Armed'; S.epoch = 1; S.leaseGeneration = 1;
    renderActionChip(o); graspOnObs(o);
    const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide');
  }, 30);
  // records every grip-ring draw (graspDraw strokes a 9 px arc at the grip pixel)
  window.__rings = 0;
  const arc = CanvasRenderingContext2D.prototype.arc;
  CanvasRenderingContext2D.prototype.arc = function (x, y, r, ...rest) { if (r === 9) window.__rings++; return arc.call(this, x, y, r, ...rest); };
};

const shots = [];
let bad = 0, checks = 0;
const check = (name, ok, d = '') => { checks++; if (!ok) { bad++; console.log(`  FAIL ${name} ${d}`); } };
const T = {
  zh: { cd: '未使用：相机朝下对准', gset: '抓取设置', show: '显示', off: '关', aria: '本页画面显示' },
  en: { cd: 'Not used: camera-down acquisition', gset: 'Grasp setup', show: 'Show', off: 'Off', aria: 'Shown on this page' },
};
const noHScroll = (page) => page.evaluate(() => ({ sw: document.scrollingElement.scrollWidth, iw: window.innerWidth }));
const sent = (page, p) => page.evaluate((p) => window.__sent.filter((m) => m.type === 'command' && m.path === p), p);
const openVis = (page) => page.evaluate(() => {
  const b = document.getElementById('btn-vis');
  if (document.body.classList.contains('ui-simple') && !document.body.classList.contains('vis-open') && b && !b.hidden) b.click();
});
const closeVis = (page) => page.evaluate(() => { if (document.body.classList.contains('vis-open')) document.getElementById('btn-vis').click(); });
const drawer = (page) => page.evaluate(() => {
  const vis = (id) => { const el = document.getElementById(id); const r = el.getBoundingClientRect(); return !el.closest('[hidden]') && r.width > 0 && r.height > 0; };
  const d = document.querySelector('#gset .calib-drawer');
  return { open: !document.getElementById('gset').hidden, grip: document.getElementById('gset-grip').textContent, pick: vis('gset-grip-pick'),
    hint: document.getElementById('gset-grip-hint').textContent, tilt: document.getElementById('gset-tilt').value, dOverflow: d.scrollWidth > d.clientWidth + 1 };
});
const rings = async (page) => { await page.evaluate(() => { window.__rings = 0; }); await page.waitForTimeout(300); return page.evaluate(() => window.__rings); };

const combos = [];
for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const vp of ['1280x800', '375x812']) {
  // expert at 375x812: the view switch covers the Detect controls (pre-existing, see grasp_shots.mjs)
  if (!(ui === 'expert' && vp === '375x812')) combos.push([ui, lang, vp]);
}
for (const [ui, lang, vp] of combos) {
  const tag = `${ui}_${lang}_${vp}`;
  const L = T[lang];
  const [w, h] = vp.split('x').map(Number);
  const ctx = await browser.newContext({ viewport: { width: w, height: h }, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  V.subs.clear(); V.trig = 0;
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed, [SETUP, true]);
  await page.waitForTimeout(700);

  // 3. vision mode selector: global mode yolo, overlay off -> "Show" + Off
  await openVis(page);
  const vm = await page.evaluate(() => {
    const w = document.getElementById('vmode'), k = document.getElementById('vm-k'), r = k.getBoundingClientRect();
    return { prefix: !k.hidden && r.width > 0 ? k.textContent : null, aria: w.getAttribute('aria-label'), on: [...w.querySelectorAll('button.on')].map((b) => b.textContent), mode: VM.mode };
  });
  check(`${tag} selector reads as this page's overlay`, vm.mode === 'yolo' && vm.prefix === L.show && vm.aria === L.aria && vm.on.join() === L.off, JSON.stringify(vm));
  await closeVis(page);

  // 1 + 2. grasp setup drawer, camera-down acquisition (calibrated 38 deg mount)
  await page.evaluate(() => graspLoadSetup(true));
  await openVis(page); await page.click('#btn-gset'); await page.waitForTimeout(300);
  let d = await drawer(page);
  let sc = await noHScroll(page);
  check(`${tag} tilt 1 decimal`, d.tilt === '-37.0', JSON.stringify(d));
  check(`${tag} grip unused (camera-down)`, d.open && d.grip === L.cd && !d.pick && d.hint === (lang === 'zh' ? '当前安装用相机朝下对准：抓取时相机先垂直朝下，把物体对准画面中心，再转为夹爪朝下。不使用这个点。'
    : 'This mount uses camera-down acquisition: a grasp first points the camera straight down and centres the object in the image, then turns the gripper down. This point is not used.') && !d.dOverflow, JSON.stringify(d));
  check(`${tag} no grip ring (camera-down)`, await rings(page) === 0);
  check(`${tag} no h-scroll (gset)`, sc.sw <= sc.iw, JSON.stringify(sc));
  let file = path.join(out, `gset_cameradown_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  gset tilt=${d.tilt} grip="${d.grip}" pick=${d.pick} overflow=${d.dOverflow} scroll=${sc.sw}<=${sc.iw}`);
  await page.fill('#gset-approach', '160');
  await page.click('#gset-save'); await page.waitForTimeout(200);
  const w1 = (await sent(page, '/api/v1/grasp/setup')).at(-1);
  check(`${tag} save keeps stored tilt`, w1 && !('camera_tilt_deg' in w1.body) && await page.evaluate(() => window.__setup.camera_tilt_deg) === -36.995338439941406
    && await page.evaluate(() => document.getElementById('gset-tilt').value) === '-37.0', JSON.stringify(w1 && w1.body));

  // camera_down false (look-down acquisition): the value, the pick button and the ring are back
  await page.evaluate(() => { window.__camDown = false; });
  await page.click('#gset-reload'); await page.waitForTimeout(300);
  d = await drawer(page);
  check(`${tag} grip used (look-down)`, d.grip === '50% , 50%' && d.pick && !d.dOverflow, JSON.stringify(d));
  check(`${tag} grip ring (look-down)`, await rings(page) > 0);
  if (lang === 'en' || vp === '375x812') { file = path.join(out, `gset_lookdown_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }
  await page.click('#gset-close'); await page.waitForTimeout(100);
  await page.evaluate(() => { window.__camDown = true; graspLoadSetup(true); });
  await page.waitForTimeout(150);

  check(`${tag} page errors`, errs.length === 0, [...new Set(errs)].join(' | ').slice(0, 300));
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} of ${checks} checks failed` : `${shots.length} screenshots, ${checks} checks passed`);
process.exit(bad ? 1 : 0);
