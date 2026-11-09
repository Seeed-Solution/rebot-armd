#!/usr/bin/env node
// Headless screenshots of the hand-eye calibration drawer (no armd needed; API calls other than the mocked
// vision routes fail and are ignored). With a perception API 2 provider it checks that opening the drawer
// creates an ArUco subscription for the live preview (kept alive), closing it deletes the subscription,
// the global mode is never POSTed, and a capture sends one trigger with {"pipelines":["aruco"]}. With an
// API 1 provider it checks the old behaviour: ArUco (Both when YOLO was on) while open, then restored.
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
// stateful mock of rebot-vision's mode API (VPOSTS records the POST bodies), subscriptions and trigger
// (CALLS records [method, path, body]); V2 = perception API 2
let VMODE = { mode: 'yolo', enabled: false };
let V2 = true;
const VPOSTS = [], CALLS = [], SUBS = new Map();
let nextSub = 1;
const vbody = () => JSON.stringify({ api_version: V2 ? 2 : 1, mode: VMODE.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: VMODE.enabled, fps: VMODE.enabled ? 5 : 1, active_fps: 5, idle_fps: 1, available: { yolo: true, aruco: true },
  ...(V2 ? { per_request_pipelines: true, subscriptions: true } : {}) });
const marker = { id: 7, margin: 61, center: [320, 180], corners: [[290, 210], [350, 210], [350, 150], [290, 150]], pose: { t: [0.01, -0.02, 0.31], r_deg: [178.2, 1.5, 3.0], error: 0.0004 } };
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(typeof o === 'string' ? o : JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  if (req.url === '/' || req.url.startsWith('/?') || req.url === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  const u = req.url.split('?')[0];
  let b = '';
  req.on('data', (c) => { b += c; });
  req.on('end', () => {
    if (u === '/api/v1/vision/mode') {
      if (req.method === 'POST') {
        const j = JSON.parse(b || '{}'); VPOSTS.push(j);
        if (j.mode) VMODE.mode = j.mode;
        if (typeof j.enabled === 'boolean') VMODE.enabled = j.enabled;
        if (VMODE.mode === 'off') VMODE.enabled = false;
      }
      return send(res, 200, vbody());
    }
    if (u === '/api/v1/detection/inference') return send(res, 200, { enabled: VMODE.enabled, fps: 1, active_fps: 5, idle_fps: 1 });
    const j = b ? JSON.parse(b) : null;
    if (u.startsWith('/api/v1/vision/subscribe') || u === '/api/v1/detection/trigger') CALLS.push([req.method, u, j]);
    if (u === '/api/v1/vision/subscribe' && req.method === 'POST') {
      const id = [...SUBS].find(([, x]) => x.client === j.client)?.[0] || 's' + nextSub++;
      SUBS.set(id, { client: j.client, pipelines: j.pipelines });
      return send(res, 200, { api_version: 2, id, client: j.client, pipelines: j.pipelines, fps: 5, ttl_ms: j.ttl_ms });
    }
    const m = u.match(/^\/api\/v1\/vision\/subscribe\/([^/]+)(\/keepalive)?$/);
    if (m) {
      if (!SUBS.has(m[1])) return send(res, 404, { error: 'no such subscription' });
      if (!m[2] && req.method === 'DELETE') SUBS.delete(m[1]);
      return send(res, 200, { api_version: 2, id: m[1] });
    }
    if (u === '/api/v1/detection/trigger') {
      const p = (j && j.pipelines) || ['yolo'];
      return send(res, 200, { api_version: 2, success: true, frame_id: 60 + CALLS.length, age_ms: 40, width: 640, height: 360,
        pipelines: { yolo: { ran: p.includes('yolo') }, aruco: { ran: p.includes('aruco') } }, detections: [], markers: p.includes('aruco') ? [marker] : [] });
    }
    res.writeHead(404); res.end();
  });
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
    // API 2: the drawer holds an ArUco subscription while open, the global mode is not touched
    await page.evaluate(() => calOpen(false));
    await page.waitForTimeout(200);
    V2 = true; VMODE = { mode: 'yolo', enabled: false }; VPOSTS.length = 0; CALLS.length = 0; SUBS.clear();
    await page.evaluate(() => window.__vision.vmRefresh());
    await page.evaluate(() => calOpen(true));
    await page.waitForTimeout(1300);
    const subOpen = [...SUBS.values()];
    const sid = [...SUBS.keys()][0];
    const kept = CALLS.some(([mm, uu]) => mm === 'POST' && uu === `/api/v1/vision/subscribe/${sid}/keepalive`);
    // capture with the arm "live" and still: one trigger with ["aruco"], one sample
    const cap = await page.evaluate(async () => {
      S.obs = { positions: [0, 0.8, 1.4, 0, 0.3, 0.2, 0], velocities: [0, 0, 0, 0, 0, 0, 0] };
      S.detections = []; S.detLastReceivedAt = 0;
      S.obsAt = Date.now() + 60000; // isLive(): fresh telemetry
      CAL.samples = []; CAL.result = null; CAL.markerId = null;
      DET.avail = true;
      const toasts = []; const t0 = window.toast; window.toast = (m) => toasts.push(m);
      await calCapture();
      window.toast = t0;
      return { n: CAL.samples.length, src: CAL.samples[0] && CAL.samples[0].frame_source, id: CAL.samples[0] && CAL.samples[0].marker_id, toasts };
    });
    const trig = CALLS.filter(([, uu]) => uu === '/api/v1/detection/trigger');
    const capFile = path.join(out, `calib_capture_${lang}_${vpName}.png`);
    await page.screenshot({ path: capFile }); shots.push(capFile);
    await page.evaluate(() => { CAL.samples = []; CAL.markerId = null; calOpen(false); });
    await page.waitForTimeout(300);
    const v2Ok = subOpen.length === 1 && /^cockpit-calib-/.test(subOpen[0].client) && JSON.stringify(subOpen[0].pipelines) === '["aruco"]' && kept
      && SUBS.size === 0 && CALLS.some(([mm, uu]) => mm === 'DELETE' && uu === `/api/v1/vision/subscribe/${sid}`) && VPOSTS.length === 0
      && trig.length === 1 && JSON.stringify(trig[0][2]) === '{"pipelines":["aruco"]}' && cap.n === 1 && cap.src === 'snapshot' && cap.id === 7;
    if (!v2Ok) bad++;
    console.log(`  api2 (${lang}/${vpName}): open -> subs=${JSON.stringify(subOpen)} keepalive=${kept} capture=${JSON.stringify(cap)} trigger=${JSON.stringify(trig.map((x) => x[2]))} close -> subs=${SUBS.size} mode posts=${VPOSTS.length} ${v2Ok ? 'OK' : 'FAIL'}`);

    // API 1: open from yolo/standby -> both + enabled; close -> yolo/standby again
    V2 = false;
    await page.evaluate(() => window.__vision.vmRefresh());
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
    console.log(`  api1 vision mode (${lang}/${vpName}): open from yolo -> ${JSON.stringify(during)} hud=${seg} close -> ${JSON.stringify(VMODE)} posts=${JSON.stringify(VPOSTS)} ${vmOk && vmOk2 ? 'OK' : 'FAIL'}`);
    if (errs.length) console.log(`  page errors (${lang}/${vpName}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
