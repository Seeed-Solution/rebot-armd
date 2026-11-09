#!/usr/bin/env node
// Headless check of the tap-to-grasp cockpit UI (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/grasp_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF schema, detection state and grasp setup; replaces the
// control WebSocket with an in-page mock that acknowledges commands; feeds synthetic observations and
// detections. For simple / expert x zh / en x 1280x800 / 375x812 it taps a box, starts a grasp, shows
// the running phase, opens the grasp setup drawer, and writes PNGs (default .cache/grasp-shots/).
// Asserts: selection + confirm bar, the grasp command body, phase text and stop button, stop on key
// input / stop button, the E-stop frame, result toasts, the setup drawer reads and writes, grip-pixel
// picking, S.grip synced from commanded_positions[6] after gripper / grasp actions, no horizontal scroll.
// Vision (perception API 2 mock): Detect creates this page's overlay subscription with ["yolo"] and keeps
// it alive; the grasp (armd triggers its own frames) changes neither the global mode nor the subscription.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/grasp-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5, 5, 3]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const SETUP = { table_z_m: 0.012, approach_height_m: 0.15, grasp_height_m: 0.015, lift_height_m: 0.12, grip_pixel_norm: [0.52, 0.64], camera_tilt_deg: -20,
  image_size_px: [640, 360], fx_px: 600, fy_px: 600, centre_tol_px: 10 };
const VCALLS = []; // /api/v1/vision/* requests: [method, path, body]
const json = (res, o) => { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS });
  if (u === '/api/v1/grasp/setup') return json(res, { supported: true, setup: SETUP, path: '/userdata/rebot-teleop/grasp_setup.json' });
  if (u === '/api/v1/safety/e_stop') return json(res, { safety_state: 'EStop', epoch_id: 2 });
  if (u === '/api/v1/detection/inference') return json(res, { enabled: true, fps: 5, active_fps: 5, idle_fps: 1 });
  if (u.startsWith('/api/v1/vision/')) {
    let b = '';
    req.on('data', (c) => { b += c; });
    req.on('end', () => {
      VCALLS.push([req.method, u, b ? JSON.parse(b) : null]);
      if (u === '/api/v1/vision/mode') return json(res, { api_version: 2, mode: 'off', modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: 5, active_fps: 5, idle_fps: 1,
        per_request_pipelines: true, subscriptions: true, available: { yolo: true, aruco: true } });
      if (u === '/api/v1/vision/subscribe') { const j = JSON.parse(b); return json(res, { api_version: 2, id: 's1', client: j.client, pipelines: j.pipelines, fps: 5, ttl_ms: j.ttl_ms }); }
      return json(res, { api_version: 2, id: 's1' });
    });
    return;
  }
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

// In-page mock: arm observation, detections, a fake video frame and a command-acknowledging socket.
const feed = (setup) => {
  window.__sent = [];
  window.__action = null;
  window.__prev = null;
  window.__cmd7 = -5.6;
  window.__setup = setup;
  // no real control socket: the mock below stands in for it (the page's reconnect loop would replace it)
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
        if (o.path === '/api/v1/action/grasp') {
          ack.action_id = 42;
          window.__action = { id: 42, kind: 'grasp', tag: 'cockpit', state: 'running', code: null, message: null, progress: 0.02, phase: 'open', iteration: null, error_px: null, started_at: 1, finished_at: null, holding: null };
        }
        if (o.path === '/api/v1/grasp/setup') {
          const s = { ...window.__setup, ...o.body };
          if (o.body.table_z_from_tcp) { delete s.table_z_from_tcp; s.table_z_m = 0.0213; }
          window.__setup = s; ack.setup = s; ack.persisted = true;
        }
        p.resolve(ack);
      }, 15);
    },
  };
  // the setup endpoint answers with the mock's current setup (as armd would after a write)
  const realFetch = window.fetch.bind(window);
  window.fetch = (u, o) => (String(u).startsWith('/api/v1/grasp/setup')
    ? Promise.resolve(new Response(JSON.stringify({ supported: true, setup: window.__setup, path: '/userdata/rebot-teleop/grasp_setup.json' }), { status: 200, headers: { 'content-type': 'application/json' } }))
    : realFetch(u, o));
  const poster = document.createElement('canvas');
  poster.width = 640; poster.height = 360;
  const g = poster.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 360); grad.addColorStop(0, '#b9a38a'); grad.addColorStop(1, '#8f7a63');
  g.fillStyle = grad; g.fillRect(0, 0, 640, 360);
  g.fillStyle = '#e8eef4'; g.beginPath(); g.ellipse(250, 200, 48, 58, 0, 0, Math.PI * 2); g.fill();
  g.fillStyle = '#3d6fb4'; g.fillRect(420, 120, 70, 150);
  const v = document.getElementById('video-stream');
  v.poster = poster.toDataURL();
  Object.defineProperty(v, 'videoWidth', { get: () => 640 });
  Object.defineProperty(v, 'videoHeight', { get: () => 360 });
  const dets = { width: 640, height: 360, detections: [
    { label: 'cup', score: 0.91, format: 'xyxy', bbox: [200, 140, 300, 260] },
    { label: 'bottle', score: 0.78, format: 'xyxy', bbox: [418, 116, 492, 274] },
  ] };
  setInterval(() => {
    if (S.ws !== fake) S.ws = fake;
    window.__fake = fake;
    const o = { positions: [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, window.__cmd7], commanded_positions: [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, window.__cmd7],
      velocities: [0, 0, 0, 0, 0, 0, 0], safety_state: 'Armed', enable_state: 'Enabled', is_torqued: true, feedback_age_ms: [5, 5, 5, 5, 5, 5, 5], epoch_id: 1,
      timestamp: performance.now() / 1000, action: window.__action, previous_action: window.__prev };
    S.obs = o; S.obsAt = Date.now(); S.state = 'Armed'; S.epoch = 1; S.leaseGeneration = 1;
    renderActionChip(o); graspOnObs(o);
    S.detections = parseDetections(dets, [640, 360]); S.detLastReceivedAt = performance.now();
    const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide');
  }, 30);
};

const shots = [];
let bad = 0, checks = 0;
const check = (name, ok, d = '') => { checks++; if (!ok) { bad++; console.log(`  FAIL ${name} ${d}`); } };
const T = {
  zh: { go: '抓取', stop: '停止', centring: '对准目标 · 第 2 次 · 14 px', ok: '已抓住，交还手动控制', fail: '抓取失败：目标丢失', gset: '抓取设置', pick: '点击画面中夹爪之间物体的中心' },
  en: { go: 'Grasp', stop: 'Stop', centring: 'Centring · step 2 · 14 px', ok: 'Object held, manual control is back', fail: 'Grasp failed: target lost', gset: 'Grasp setup', pick: 'Tap the centre of the object between the jaws' },
};
const noHScroll = (page) => page.evaluate(() => ({ sw: document.scrollingElement.scrollWidth, iw: window.innerWidth }));
const sent = (page, p) => page.evaluate((p) => window.__sent.filter((m) => m.type === 'command' && m.path === p), p);
// client point of a normalised video coordinate
const videoPoint = (page, u, v) => page.evaluate(([u, v]) => {
  const r = document.getElementById('video-box').getBoundingClientRect();
  const c = getVideoContentRect(r.width, r.height);
  return [r.left + c.x + u * c.w, r.top + c.y + v * c.h];
}, [u, v]);

for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const tag = `${ui}_${lang}_${vp}`;
  const L = T[lang];
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed, SETUP);
  await page.waitForTimeout(700);
  // overlay on: this page's subscription with the selected (default YOLO) pipelines
  VCALLS.length = 0;
  await page.click('#btn-det');
  await page.waitForTimeout(300);
  const sub0 = VCALLS.find(([m, u]) => m === 'POST' && u === '/api/v1/vision/subscribe');
  check(`${tag} overlay subscription`, sub0 && JSON.stringify(sub0[2].pipelines) === '["yolo"]' && /^cockpit-overlay-/.test(sub0[2].client)
    && await page.evaluate(() => DET.enabled), JSON.stringify(VCALLS));

  // 1. tap the cup box
  const [cx, cy] = await videoPoint(page, 250 / 640, 200 / 360);
  await page.mouse.click(cx, cy);
  await page.waitForTimeout(200);
  let info = await page.evaluate(() => {
    const vis = (id) => { const el = document.getElementById(id); if (!el) return false; const r = el.getBoundingClientRect(); return !el.closest('[hidden]') && r.width > 0 && r.height > 0; };
    const bar = document.getElementById('grasp-bar').getBoundingClientRect(), vb = document.getElementById('video-box').getBoundingClientRect();
    return { sel: GR.sel && GR.sel.label, bar: vis('grasp-bar'), go: vis('grasp-go') && document.getElementById('grasp-go').textContent, cancel: vis('grasp-cancel'), stop: vis('grasp-stop'),
      label: document.getElementById('grasp-label').textContent, score: document.getElementById('grasp-score').textContent, gset: vis('btn-gset') && document.getElementById('btn-gset').textContent,
      barInside: bar.left >= vb.left - 1 && bar.right <= vb.right + 1 && bar.bottom <= vb.bottom + 1 && bar.top >= vb.top - 1 };
  });
  let sc = await noHScroll(page);
  check(`${tag} select`, info.sel === 'cup' && info.bar && info.go === L.go && info.cancel && !info.stop && info.label === 'cup' && info.score === '91%', JSON.stringify(info));
  check(`${tag} bar inside video`, info.barInside, JSON.stringify(info));
  check(`${tag} setup button`, info.gset === L.gset, JSON.stringify(info));
  check(`${tag} no h-scroll (select)`, sc.sw <= sc.iw, JSON.stringify(sc));
  let file = path.join(out, `grasp_select_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  select=${info.sel} bar=${info.bar} go="${info.go}" score=${info.score} inside=${info.barInside} scroll=${sc.sw}<=${sc.iw}`);

  // tapping empty video clears the selection, tapping the bottle selects it, back to the cup
  const [ex, ey] = await videoPoint(page, 0.56, 0.55);
  await page.mouse.click(ex, ey); await page.waitForTimeout(100);
  check(`${tag} tap empty clears`, await page.evaluate(() => GR.sel === null));
  await page.mouse.click(cx, cy); await page.waitForTimeout(100);

  // 2. grasp -> command, running phase, stop button
  await page.click('#grasp-go');
  await page.waitForTimeout(200);
  const cmd = (await sent(page, '/api/v1/action/grasp'))[0];
  if (!cmd) console.log('  debug', JSON.stringify(await page.evaluate(() => ({ sel: GR.sel, sending: GR.sending, sent: window.__sent.slice(-5), locked: isLocked(), toasts: [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent) }))));
  const bn = cmd && cmd.body.box_norm;
  check(`${tag} grasp command`, cmd && cmd.body.label === 'cup' && Math.abs(bn[0] - 200 / 640) < 1e-3 && Math.abs(bn[3] - 260 / 360) < 1e-3 && typeof cmd.body.ts_ms === 'number', JSON.stringify(cmd && cmd.body));
  await page.evaluate(() => { Object.assign(window.__action, { phase: 'centring', iteration: 2, error_px: 14.3, progress: 0.3 }); });
  await page.waitForTimeout(250);
  info = await page.evaluate(() => {
    const vis = (id) => { const el = document.getElementById(id); const r = el.getBoundingClientRect(); return !el.closest('[hidden]') && r.width > 0 && r.height > 0; };
    return { phase: document.getElementById('grasp-phase').textContent, stop: vis('grasp-stop') && document.getElementById('grasp-stop').textContent, go: vis('grasp-go'),
      fill: document.getElementById('grasp-fill').style.width, gid: GR.id };
  });
  sc = await noHScroll(page);
  check(`${tag} running`, info.phase === L.centring && info.stop === L.stop && !info.go && info.fill === '30%' && info.gid === 42, JSON.stringify(info));
  check(`${tag} no h-scroll (running)`, sc.sw <= sc.iw, JSON.stringify(sc));
  file = path.join(out, `grasp_running_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  phase="${info.phase}" stop="${info.stop}" fill=${info.fill}`);

  // key input while running -> stop; stop button -> stop; E-stop -> estop frame
  const stops0 = (await sent(page, '/api/v1/motion/stop')).length;
  await page.evaluate(() => { window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', key: 'w', bubbles: true })); window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', key: 'w', bubbles: true })); });
  await page.waitForTimeout(600);
  const stops1 = (await sent(page, '/api/v1/motion/stop')).length;
  check(`${tag} key input stops`, stops1 > stops0, `${stops0} -> ${stops1}`);
  await page.click('#grasp-stop'); await page.waitForTimeout(600);
  const stops2 = (await sent(page, '/api/v1/motion/stop')).length;
  check(`${tag} stop button stops`, stops2 > stops1, `${stops1} -> ${stops2}`);

  // 3. done holding: toast + gripper handover sync (S.grip from commanded_positions[6])
  await page.evaluate(() => { S.grip = 1; window.__cmd7 = -1.5; Object.assign(window.__action, { state: 'done', phase: 'done', progress: 1, holding: true, finished_at: performance.now() / 1000 }); });
  await page.waitForTimeout(250);
  info = await page.evaluate(() => ({ grip: S.grip, toast: [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent), bar: !document.getElementById('grasp-bar').hidden, gid: GR.id }));
  check(`${tag} result toast`, info.toast.some((x) => x.includes(L.ok)), JSON.stringify(info.toast));
  const vAfter = VCALLS.filter(([m, u]) => !(m === 'POST' && u === '/api/v1/vision/subscribe/s1/keepalive') && !(m === 'GET'));
  check(`${tag} grasp leaves vision alone`, vAfter.length === 1 && vAfter[0] === sub0 && VCALLS.some(([, u]) => u.endsWith('/keepalive')), JSON.stringify(VCALLS.filter(([m]) => m !== 'GET')));
  check(`${tag} grip synced after grasp`, Math.abs(info.grip - 1.5 / 5.7) < 0.01, `S.grip ${info.grip}`);
  check(`${tag} bar closed after grasp`, !info.bar && info.gid === null, JSON.stringify(info));
  if (lang === 'zh' && vp === '375x812') { file = path.join(out, `grasp_done_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }

  // a teleop gripper change afterwards is not overridden; a later gripper action syncs again
  await page.evaluate(() => { S.grip = 0.9; });
  await page.waitForTimeout(150);
  check(`${tag} grip not re-synced for the same action`, Math.abs(await page.evaluate(() => S.grip) - 0.9) < 1e-9);
  await page.evaluate(() => { window.__prev = window.__action; window.__cmd7 = -5.61; window.__action = { id: 43, kind: 'gripper', state: 'done', progress: 1, finished_at: performance.now() / 1000 }; });
  await page.waitForTimeout(150);
  check(`${tag} grip synced after gripper action`, Math.abs(await page.evaluate(() => S.grip) - 5.61 / 5.7) < 0.01, String(await page.evaluate(() => S.grip)));

  // failed grasp from an agent: toast with the code text
  await page.evaluate(() => { window.__prev = null; window.__action = { id: 44, kind: 'grasp', state: 'failed', code: 'TARGET_LOST', message: 'x', progress: 0.3, finished_at: performance.now() / 1000 }; });
  await page.waitForTimeout(200);
  check(`${tag} failure toast`, (await page.evaluate(() => [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent))).some((x) => x.includes(L.fail)));

  // 4. setup drawer
  await page.click('#btn-gset');
  await page.waitForTimeout(300);
  info = await page.evaluate(() => {
    const d = document.querySelector('#gset .calib-drawer');
    return { open: !document.getElementById('gset').hidden, table: document.getElementById('gset-table').textContent, grip: document.getElementById('gset-grip').textContent,
      approach: document.getElementById('gset-approach').value, tilt: document.getElementById('gset-tilt').value, title: document.getElementById('gset-title').textContent,
      dOverflow: d.scrollWidth > d.clientWidth + 1 };
  });
  sc = await noHScroll(page);
  check(`${tag} setup drawer`, info.open && info.table === '12 mm' && info.grip === '52% , 64%' && info.approach === '150' && info.tilt === '-20' && info.title === L.gset && !info.dOverflow, JSON.stringify(info));
  check(`${tag} no h-scroll (setup)`, sc.sw <= sc.iw, JSON.stringify(sc));
  file = path.join(out, `grasp_setup_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  console.log(`${tag}  setup table=${info.table} grip=${info.grip} approach=${info.approach} tilt=${info.tilt}`);

  await page.click('#gset-table-tcp'); await page.waitForTimeout(150);
  const w1 = (await sent(page, '/api/v1/grasp/setup')).at(-1);
  check(`${tag} use TCP height`, w1 && w1.body.table_z_from_tcp === true && (await page.evaluate(() => document.getElementById('gset-table').textContent)) === '21 mm', JSON.stringify(w1 && w1.body));
  await page.fill('#gset-approach', '170'); await page.fill('#gset-grasp', '20');
  await page.click('#gset-save'); await page.waitForTimeout(150);
  const w2 = (await sent(page, '/api/v1/grasp/setup')).at(-1);
  check(`${tag} save heights`, w2 && Math.abs(w2.body.approach_height_m - 0.17) < 1e-9 && Math.abs(w2.body.grasp_height_m - 0.02) < 1e-9 && w2.body.camera_tilt_deg === -20, JSON.stringify(w2 && w2.body));

  // grip pixel: pick mode, tap, drawer reopens with the new value
  await page.click('#gset-grip-pick'); await page.waitForTimeout(200);
  info = await page.evaluate(() => ({ drawer: !document.getElementById('gset').hidden, pick: !document.getElementById('grasp-pick').hidden, text: document.querySelector('#grasp-pick span').textContent }));
  check(`${tag} pick mode`, !info.drawer && info.pick && info.text === L.pick, JSON.stringify(info));
  if (lang === 'en' || vp === '375x812') { file = path.join(out, `grasp_pick_${tag}.png`); await page.screenshot({ path: file }); shots.push(file); }
  const [gx, gy] = await videoPoint(page, 0.25, 0.75);
  await page.mouse.click(gx, gy); await page.waitForTimeout(250);
  const w3 = (await sent(page, '/api/v1/grasp/setup')).at(-1);
  info = await page.evaluate(() => ({ drawer: !document.getElementById('gset').hidden, grip: document.getElementById('gset-grip').textContent, sel: GR.sel }));
  check(`${tag} grip pixel set`, w3 && Math.abs(w3.body.grip_pixel_norm[0] - 0.25) < 0.01 && Math.abs(w3.body.grip_pixel_norm[1] - 0.75) < 0.01 && info.drawer && info.grip === '25% , 75%' && info.sel === null,
    JSON.stringify([w3 && w3.body, info]));
  sc = await noHScroll(page);
  check(`${tag} no h-scroll (end)`, sc.sw <= sc.iw, JSON.stringify(sc));
  if (ui === 'simple' && vp === '1280x800') {
    // E-stop during a running grasp sends the estop frame
    await page.click('#gset-close'); await page.waitForTimeout(100);
    await page.evaluate(() => { window.__action = { id: 45, kind: 'grasp', state: 'running', phase: 'descend', progress: 0.65 }; });
    await page.waitForTimeout(150);
    await page.click('#btn-estop');
    await page.waitForTimeout(100);
    check(`${tag} e-stop frame`, await page.evaluate(() => window.__sent.some((m) => m.type === 'estop')));
  }
  check(`${tag} page errors`, errs.length === 0, [...new Set(errs)].join(' | ').slice(0, 300));
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} of ${checks} checks failed` : `${shots.length} screenshots, ${checks} checks passed`);
process.exit(bad ? 1 : 0);
