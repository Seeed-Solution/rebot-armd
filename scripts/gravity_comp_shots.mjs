#!/usr/bin/env node
// Headless check of the gravity-compensation (hand-guiding) cockpit UI (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/gravity_comp_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF schema, replaces the control WebSocket with an
// in-page mock that acknowledges commands, and feeds synthetic observations. For simple / expert x
// zh / en x 1280x800 / 375x812 it shows the entry switch (ready and blocked), holds it to enter,
// shows the ramp and the active state with recording, and the exit toast; writes PNGs (default
// .cache/gravity-comp-shots/). Asserts: hold-to-confirm sends {enable:true} only after the hold,
// blocked reason text, banner + chip + ramp width, sticks/keys send no servo frames while active,
// the gripper key sends a gripper action, the exit button sends {enable:false}, exit toasts,
// no horizontal scroll.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/gravity-comp-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const json = (res, o) => { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS });
  if (u === '/api/v1/grasp/setup') return json(res, { supported: false, setup: null, path: null });
  if (u === '/api/v1/safety/e_stop') return json(res, { safety_state: 'EStop', epoch_id: 2 });
  if (u === '/api/v1/dataset/record/start') return json(res, { status: 'recording_started', frame_count: 0 });
  if (u === '/api/v1/dataset/record/save') return json(res, { status: 'saved', path: '/userdata/datasets/teach.parquet', frame_count: 912 });
  if (u === '/api/v1/detection/inference') return json(res, { enabled: false, fps: 0 });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

// In-page mock: observation stream and a command-acknowledging control socket.
const feed = () => {
  window.__sent = [];
  window.__state = 'Armed';
  window.__gc = { supported: true, blocker: null, phase: null, ramp: 0, scale: 1, damping_kd: [0.4, 0.6, 0.5, 0.2, 0.15, 0.1], tau_ff: [], last_exit: null };
  window.__mode = 'position';
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
        if (o.path === '/api/v1/mode/gravity_comp') {
          if (o.body.enable) { window.__mode = 'gravity_comp'; window.__gc = { ...window.__gc, phase: 'ramp_in', ramp: 0.42, blocker: null }; }
          else { window.__mode = 'position'; window.__gc = { ...window.__gc, phase: null, ramp: 0, last_exit: { code: 'REQUESTED', message: 'Exit requested', at: performance.now() / 1000 } }; }
          ack.control_mode = window.__mode;
        }
        p.resolve(ack);
      }, 15);
    },
  };
  const poster = document.createElement('canvas');
  poster.width = 640; poster.height = 360;
  const g = poster.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 360); grad.addColorStop(0, '#b9a38a'); grad.addColorStop(1, '#8f7a63');
  g.fillStyle = grad; g.fillRect(0, 0, 640, 360);
  g.fillStyle = '#3d6fb4'; g.fillRect(420, 120, 70, 150);
  const v = document.getElementById('video-stream');
  v.poster = poster.toDataURL();
  setInterval(() => {
    if (S.ws !== fake) S.ws = fake;
    const q = [0.3, -0.9, -1.2, 1.1, 0, Math.PI / 2, -2.0];
    const o = { positions: q, commanded_positions: q, velocities: [0, 0, 0, 0, 0, 0, 0], torques: [0, -2.1, -5.5, -1.3, 0, 0, 0], temperatures: Array(7).fill(32),
      safety_state: window.__state, enable_state: window.__state === 'Idle' ? 'Disabled' : 'Enabled', is_torqued: window.__state !== 'Idle', feedback_age_ms: Array(7).fill(5), epoch_id: 1,
      timestamp: performance.now() / 1000, action: null, control_mode: window.__mode, gravity_comp: window.__gc };
    S.obs = o; S.obsAt = Date.now(); S.state = window.__state; S.epoch = 1; S.leaseGeneration = 1;
    gcOnObs(o);
    const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide');
  }, 30);
};

const shots = [];
let bad = 0, checks = 0;
const check = (name, ok, d = '') => { checks++; if (!ok) { bad++; console.log(`  FAIL ${name} ${d}`); } };
const T = {
  zh: { title: '重力补偿 / 拖动示教', hold: '按住 0.6 秒进入', blocked: '先使能机械臂', active: '拖动示教中', ramp: '建立补偿 42%', free: '可以拖动', exit: '退出并锁定位置',
    overspeed: '关节速度过快（可能在下坠）：已锁定位置', off: '已退出拖动示教，位置已锁定', sticks: '摇杆已停用' },
  en: { title: 'Gravity comp / Hand-guide', hold: 'Hold 0.6 s to enter', blocked: 'Enable the arm first', active: 'Hand-guiding', ramp: 'Ramping 42%', free: 'Free to move', exit: 'Exit and hold pose',
    overspeed: 'Joint too fast (falling?): pose held', off: 'Hand-guiding off, pose held', sticks: 'Sticks are off' },
};
const noHScroll = (page) => page.evaluate(() => ({ sw: document.scrollingElement.scrollWidth, iw: window.innerWidth }));
const sentPath = (page, p) => page.evaluate((p) => window.__sent.filter((m) => m.type === 'command' && m.path === p), p);
const toasts = (page) => page.evaluate(() => [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent));
const vis = (page, id) => page.evaluate((id) => { const el = document.getElementById(id); if (!el) return false; const r = el.getBoundingClientRect(); return !el.closest('[hidden]') && r.width > 0 && r.height > 0 && getComputedStyle(el).display !== 'none'; }, id);
const text = (page, id) => page.evaluate((id) => document.getElementById(id).textContent, id);
const inView = (page, id) => page.evaluate((id) => { const r = document.getElementById(id).getBoundingClientRect(); return r.left >= -1 && r.right <= window.innerWidth + 1 && r.top >= -1 && r.bottom <= window.innerHeight + 1; }, id);

for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const tag = `${ui}_${lang}_${vp}`;
  const L = T[lang];
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1, hasTouch: false });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  const reveal = (id) => page.evaluate((id) => document.getElementById(id).scrollIntoView({ block: 'center' }), id);
  await reveal('btn-gc');

  // 1. blocked: not armed
  await page.evaluate(() => { window.__state = 'Idle'; window.__gc = { ...window.__gc, blocker: { code: 'NOT_ARMED', message: 'Enable the arm first' } }; });
  await page.waitForTimeout(200);
  check(`${tag} blocked reason`, (await vis(page, 'btn-gc')) && (await text(page, 'gc-sub')) === L.blocked, await text(page, 'gc-sub'));
  check(`${tag} blocked class`, await page.evaluate(() => document.getElementById('btn-gc').classList.contains('blocked')));
  let file = path.join(out, `gc_blocked_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  const box0 = await page.locator('#btn-gc').boundingBox();
  await page.mouse.move(box0.x + 20, box0.y + box0.height / 2); await page.mouse.down(); await page.waitForTimeout(700); await page.mouse.up();
  check(`${tag} blocked hold sends nothing`, (await sentPath(page, '/api/v1/mode/gravity_comp')).length === 0);
  await reveal('btn-gc');
  check(`${tag} blocked toast`, (await toasts(page)).some((x) => x.includes(L.blocked)), JSON.stringify(await toasts(page)));

  // 2. ready
  await page.evaluate(() => { window.__state = 'Armed'; window.__gc = { ...window.__gc, blocker: null }; document.getElementById('toasts').innerHTML = ''; });
  await page.waitForTimeout(200);
  check(`${tag} ready text`, (await text(page, 'gc-sub')) === L.hold && (await page.evaluate(() => document.querySelector('#btn-gc b').textContent)) === L.title);
  check(`${tag} switch in view`, await inView(page, 'btn-gc'));
  file = path.join(out, `gc_ready_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 3. hold to enter: a short press does nothing, a 0.6 s hold sends enable:true
  const box = await page.locator('#btn-gc').boundingBox();
  await page.mouse.move(box.x + 20, box.y + box.height / 2); await page.mouse.down(); await page.waitForTimeout(250); await page.mouse.up();
  await page.waitForTimeout(100);
  check(`${tag} short press ignored`, (await sentPath(page, '/api/v1/mode/gravity_comp')).length === 0);
  await page.mouse.down(); await page.waitForTimeout(750); await page.mouse.up();
  await page.waitForTimeout(200);
  let cmds = await sentPath(page, '/api/v1/mode/gravity_comp');
  check(`${tag} hold sends enable`, cmds.length === 1 && cmds[0].body.enable === true, JSON.stringify(cmds));
  check(`${tag} ramp banner`, (await vis(page, 'gc-banner')) && (await text(page, 'gc-phase')) === L.ramp, await text(page, 'gc-phase'));
  const rampW = await page.evaluate(() => document.getElementById('gc-ramp-bar').style.width);
  check(`${tag} ramp width`, rampW === '42%', rampW);
  check(`${tag} chip`, await vis(page, 'gc-chip'));
  await reveal('gc-banner');
  check(`${tag} banner in view`, await inView(page, 'gc-banner'));
  file = path.join(out, `gc_ramp_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 4. active + recording: sticks and keys send no servo frames; the gripper key sends an action
  await page.evaluate(() => { window.__gc = { ...window.__gc, phase: 'active', ramp: 1, tau_ff: [0, -2.1, -5.5, -1.3, 0, 0] }; });
  await page.locator('#btn-gc-rec').click();
  await page.waitForTimeout(300);
  check(`${tag} recording state`, await page.evaluate(() => S.rec.active && document.getElementById('btn-gc-rec').classList.contains('rec-on')));
  await page.evaluate(() => { window.__sent = []; document.getElementById('toasts').innerHTML = ''; });
  await page.keyboard.down('KeyW'); await page.keyboard.down('KeyA'); await page.waitForTimeout(300); await page.keyboard.up('KeyW'); await page.keyboard.up('KeyA');
  await page.keyboard.down('KeyX'); await page.waitForTimeout(150); await page.keyboard.up('KeyX');
  await page.waitForTimeout(200);
  const servoFrames = await page.evaluate(() => window.__sent.filter((m) => m.type === 'servo').length);
  check(`${tag} no servo frames`, servoFrames === 0, String(servoFrames));
  const grip = await sentPath(page, '/api/v1/action/gripper');
  check(`${tag} gripper action`, grip.length === 1 && grip[0].body.command === 'close', JSON.stringify(grip));
  check(`${tag} sticks-off toast`, (await toasts(page)).some((x) => x.includes(L.sticks)), JSON.stringify(await toasts(page)));
  check(`${tag} active phase`, (await text(page, 'gc-phase')) === L.free && (await page.evaluate(() => document.body.classList.contains('gc'))));
  await page.evaluate(() => { document.getElementById('toasts').innerHTML = ''; });
  await reveal('gc-banner');
  file = path.join(out, `gc_active_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  let sc = await noHScroll(page);
  check(`${tag} no h-scroll`, sc.sw <= sc.iw, JSON.stringify(sc));

  // 5. forced exit (overspeed) from the server: toast with the reason, banner gone
  await page.evaluate(() => { window.__mode = 'position'; window.__gc = { ...window.__gc, phase: null, ramp: 0, last_exit: { code: 'GRAVITY_COMP_OVERSPEED', message: 'Joint 3 at 1.9 rad/s', at: 123.4 } }; });
  await page.waitForTimeout(250);
  check(`${tag} overspeed toast`, (await toasts(page)).some((x) => x.includes(L.overspeed)), JSON.stringify(await toasts(page)));
  check(`${tag} banner hidden`, !(await vis(page, 'gc-banner')));
  file = path.join(out, `gc_exit_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);

  // 6. enter again, leave with the exit button
  await page.evaluate(() => { window.__mode = 'gravity_comp'; window.__gc = { ...window.__gc, phase: 'active', ramp: 1 }; window.__sent = []; document.getElementById('toasts').innerHTML = ''; });
  await page.waitForTimeout(150);
  await page.locator('#btn-gc-exit').click();
  await page.waitForTimeout(250);
  cmds = await sentPath(page, '/api/v1/mode/gravity_comp');
  check(`${tag} exit button`, cmds.length === 1 && cmds[0].body.enable === false, JSON.stringify(cmds));
  check(`${tag} exit toast`, (await toasts(page)).some((x) => x.includes(L.off)), JSON.stringify(await toasts(page)));
  check(`${tag} no page errors`, errs.length === 0, errs.join('; '));
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} of ${checks} checks failed` : `${shots.length} screenshots in ${out}, ${checks} checks passed`);
process.exit(bad ? 1 : 0);
