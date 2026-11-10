#!/usr/bin/env node
// Headless check of the cockpit's developer-platform UI (no armd needed):
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/devplat_shots.mjs
// Needs Playwright >= 1.48 (page.routeWebSocket). Serves src/api/index.html with a mocked 7-DOF schema,
// GET /api/v1/safety/collision and GET /api/v1/poses, and answers the page's WebSocket itself: lease grants,
// command acks, and synthetic observations at 50 Hz (docs: devplat contract sections 2, 3, 6, 11).
// Scenes: normal, collision protective stop (observation.collision.active), escalated Fault COLLISION,
// fault code text (BUS_LOSS), the settings drawer (collision guard toggle + sensitivity -> WS
// /api/v1/safety/collision), the poses drawer (name validation, save / move_to_pose / delete), and an
// armd without the guard or poses (everything hidden). zh and en, 1280x800 and 375x812.
// Writes PNGs (default .cache/devplat-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/devplat-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const CAPS_FULL = { version: 6, fault_codes: true, poses: true, residual_torques: true, collision_guard: true };
const CAPS_OLD = { version: 6, fault_codes: true, poses: false, residual_torques: true };
const BASE_THR = [6.0, 6.0, 6.0, 2.5, 2.5, 2.5];

// server-side mock state, reset per page
let caps = CAPS_FULL;
let colCfg, poses;
function resetMock() {
  colCfg = { enabled: true, sensitivity: 'medium', thresholds_nm: BASE_THR.slice(), persist_ticks: 4, clear_ratio: 0.5, resume_after_s: 2.0, max_auto_resumes: 3, escalate_window_s: 60 };
  poses = [
    { name: 'bin_a', positions: [0.1, -0.8, -1.2, 1.0, 0, 1.57, 0], tcp: { position: [0.25, 0.1, 0.12], rpy: [3.14, 0, 0] }, saved_at_ms: 1762828800000 },
    { name: 'inspect-left', positions: [0.6, -1.0, -1.4, 1.3, 0, 1.57, 0], tcp: null, saved_at_ms: 1762832400000 },
  ];
}
const json = (res, o, code = 200) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS, capabilities: caps });
  if (u === '/api/v1/safety/collision') {
    if (!caps.collision_guard) return json(res, { code: 'NOT_FOUND', message: 'not found' }, 404);
    const k = { low: 1.5, medium: 1.0, high: 0.7 }[colCfg.sensitivity];
    return json(res, { ...colCfg, effective_thresholds_nm: colCfg.thresholds_nm.map((v) => v * k) });
  }
  if (u === '/api/v1/poses') return caps.poses ? json(res, { poses }) : json(res, { code: 'NOT_FOUND' }, 404);
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } };
const shots = [];
const shot = async (page, name) => { const f = path.join(out, name + '.png'); await page.screenshot({ path: f }); shots.push(f); };

const Q = [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0];
const COL_IDLE = { active: false, joint: null, residual_nm: null, threshold_nm: null, since_s: null, resume_in_s: null, trips_in_window: 0, enabled: true, sensitivity: 'medium' };
const SCENES = {
  normal: { safety_state: 'Armed', fault: null, residual_torques: [1.0, 3.5, 0.4, 0.3, 1.6, 0.1], collision: COL_IDLE },
  active: { safety_state: 'Armed', fault: null, residual_torques: [1.2, 7.4, 0.8, 0.3, 0.9, 0.1],
    collision: { ...COL_IDLE, active: true, joint: 2, residual_nm: 7.4, threshold_nm: 6.0, since_s: 0.4, resume_in_s: 1.6, trips_in_window: 1 } },
  waiting: { safety_state: 'Armed', fault: null, residual_torques: [1.2, 7.4, 0.8, 0.3, 0.9, 0.1],
    collision: { ...COL_IDLE, active: true, joint: 2, residual_nm: -7.4, threshold_nm: 6.0, since_s: 0.2, resume_in_s: null, trips_in_window: 1 } },
  escalated: { safety_state: { Fault: { Other: 'Collision guard: 4 trips within 60 s' } }, fault: { code: 'COLLISION', message: 'Collision guard: 4 trips within 60 s', recoverable: false },
    residual_torques: [0.5, 0.6, 0.4, 0.3, 0.2, 0.1], collision: { ...COL_IDLE, trips_in_window: 4 } },
  bus: { safety_state: { Fault: 'BusLoss' }, fault: { code: 'BUS_LOSS', message: 'CAN bus loss', recoverable: false }, residual_torques: null, collision: COL_IDLE },
  old: { safety_state: 'Armed', fault: null, residual_torques: [1.0, 3.5, 0.4, 0.3, 1.6, 0.1], collision: undefined },
};

async function openPage({ lang, ui, viewport, scene = 'normal' }) {
  resetMock();
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const P = { page, ctx, scene, cmds: [], errs: [], t0: Date.now() };
  page.on('pageerror', (e) => P.errs.push(e.message));
  await page.routeWebSocket(/\/api\/v1\/ws/, (ws) => {
    ws.onMessage((raw) => {
      let m; try { m = JSON.parse(String(raw)); } catch (_) { return; }
      if (m.type === 'acquire_lease') { ws.send(JSON.stringify({ type: 'lease_granted', seq: m.seq, generation: 7, epoch_id: 1, client_id: m.client_id })); return; }
      if (m.type !== 'command') return;
      P.cmds.push({ path: m.path, body: m.body });
      const ack = { type: 'command_ack', request_id: m.request_id, epoch_id: 1 };
      if (m.path === '/api/v1/poses/save') {
        const p = { name: m.body.name, positions: Q.slice(), tcp: { position: [0.2, 0, 0.2], rpy: [3.14, 0, 0] }, saved_at_ms: Date.now() };
        poses = poses.filter((x) => x.name !== p.name).concat([p]); ack.pose = p;
      } else if (m.path === '/api/v1/poses/delete') {
        if (!poses.some((x) => x.name === m.body.name)) { ws.send(JSON.stringify({ type: 'command_reject', request_id: m.request_id, epoch_id: 1, code: 'POSE_NOT_FOUND', status: 404, message: 'no pose' })); return; }
        poses = poses.filter((x) => x.name !== m.body.name);
      } else if (m.path === '/api/v1/safety/collision') {
        Object.assign(colCfg, m.body);
      } else if (m.path === '/api/v1/action/move_to_pose') ack.action_id = 1;
      ws.send(JSON.stringify(ack));
    });
    P.feed = setInterval(() => {
      const sc = SCENES[P.scene];
      const o = { timestamp: (Date.now() - P.t0) / 1000, positions: Q, velocities: Q.map(() => 0), commanded_positions: Q, safety_state: sc.safety_state,
        enable_state: 'Enabled', is_torqued: true, feedback_age_ms: Q.map(() => 5), torques: Q.map(() => 0.5), temperatures: Q.map(() => 35), epoch_id: 1,
        lease_holder: null, driver_capabilities: { velocity_feedback: true, torque_feedback: true, temperature_feedback: true },
        motion_limit: null, motor_errors: [], action: null, fault: sc.fault, residual_torques: sc.residual_torques };
      if (sc.collision !== undefined) o.collision = sc.collision.enabled === false ? null : { ...sc.collision, enabled: colCfg.enabled, sensitivity: colCfg.sensitivity };
      try { ws.send(JSON.stringify(o)); } catch (_) {}
    }, 20);
  });
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 });
  await page.waitForTimeout(400);
  return P;
}
const closePage = async (P) => { clearInterval(P.feed); await P.ctx.close(); };
const vis = (page, id) => page.evaluate((i) => { const el = document.getElementById(i); if (!el) return false; const r = el.getBoundingClientRect(); return !el.hidden && getComputedStyle(el).display !== 'none' && r.width > 0 && r.height > 0; }, id);
const text = (page, id) => page.evaluate((i) => document.getElementById(i).textContent, id);
const inView = (page, id) => page.evaluate((i) => { const r = document.getElementById(i).getBoundingClientRect(); return r.left >= 0 && r.right <= innerWidth + 0.5 && r.top >= 0 && r.bottom <= innerHeight + 0.5; }, id);
const noHScroll = async (page) => {
  const r = await page.evaluate(() => ({ sw: document.scrollingElement.scrollWidth, iw: innerWidth,
    wide: [...document.querySelectorAll('.topbar *')].filter((e) => e.getBoundingClientRect().right > innerWidth + 0.5).map((e) => e.id || e.className).slice(0, 4) }));
  if (r.sw > r.iw) console.log('  overflow', JSON.stringify(r));
  return r.sw <= r.iw;
};

const EXPECT = {
  zh: {
    active: '碰撞保护：关节 J2 受阻（7.4 N·m），已停止；1.6 秒后自动恢复',
    waiting: '碰撞保护：关节 J2 受阻（7.4 N·m），已停止；受力消除后自动恢复',
    esc: '⚠️ 多次碰撞，机械臂已锁定', bus: '电机总线断开', busDo: '检查电机线缆和电源是否接好',
    clear: '⚠️ 点击清除故障', settings: '设置', poses: '点位', bad: '名称不符合规则',
    hold: '碰撞保护中', holdLock: '碰撞保护中：自动恢复前运动控制已锁定（急停可用）', armed: '已使能',
  },
  en: {
    active: 'Collision guard: joint J2 blocked (7.4 N·m), stopped; resuming in 1.6 s',
    waiting: 'Collision guard: joint J2 blocked (7.4 N·m), stopped; resumes once the force is gone',
    esc: '⚠️ Repeated collisions; arm locked', bus: 'Motor bus lost', busDo: 'Check the motor cable and power',
    clear: '⚠️ Tap to clear fault', settings: 'Settings', poses: 'Poses', bad: 'Invalid name',
    hold: 'Collision hold', holdLock: 'Collision hold: motion controls locked until it resumes (E-stop available)', armed: 'Enabled',
  },
};

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
try {
  for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
    const E = EXPECT[lang];
    // ---------- expert mode: bars, fault text, banner ----------
    let tag = `expert_${lang}_${vp}`;
    let P = await openPage({ lang, ui: 'expert', viewport });
    let page = P.page;
    ok(!(await vis(page, 'col-banner')), `${tag} normal: banner hidden`);
    ok(await vis(page, 'btn-sset') && await vis(page, 'btn-pset'), `${tag} normal: settings and poses buttons shown`);
    ok(await noHScroll(page), `${tag} normal: no horizontal scroll`);
    const bars = await page.evaluate(() => [...document.querySelectorAll('#rt-bars .rt-bar')].map((b) => b.className));
    ok(bars.length === 6, `${tag} 6 residual bars (${bars.length})`);
    ok(bars[1] === 'rt-bar warn' && bars[0] === 'rt-bar' && bars[4] === 'rt-bar warn', `${tag} bar colours normal: ${bars.join('|')}`);
    await page.evaluate(() => document.getElementById('rt-bars').scrollIntoView({ block: 'center' }));
    await shot(page, `devplat_normal_${tag}`);
    // bars render at most ~10 Hz
    const rate = await page.evaluate(async () => {
      const el = document.querySelector('#rt-bars .rt-bar:nth-child(3) i b'); let n = 0;
      const mo = new MutationObserver(() => n++); mo.observe(el, { attributes: true });
      const t0 = performance.now(); let k = 0;
      const iv = setInterval(() => { S.obs.residual_torques = S.obs.residual_torques.map((v, i) => (i === 1 ? 0.5 + (k++ % 10) * 0.5 : v)); }, 5);
      await new Promise((r) => setTimeout(r, 1000)); clearInterval(iv); mo.disconnect();
      return n / ((performance.now() - t0) / 1000);
    });
    ok(rate <= 12, `${tag} bar updates ${rate.toFixed(1)}/s <= 12`);

    P.scene = 'active'; await page.waitForTimeout(250);
    ok(await vis(page, 'col-banner'), `${tag} active: banner shown`);
    let tt = await text(page, 'col-title');
    ok(tt === E.active, `${tag} active banner "${tt}"`);
    ok(await inView(page, 'col-banner'), `${tag} active banner in the viewport`);
    ok(!(await vis(page, 'col-clear')), `${tag} active: no clear button`);
    const b2 = await page.evaluate(() => document.querySelector('#rt-bars .rt-bar:nth-child(3)').className);
    ok(b2 === 'rt-bar trip', `${tag} J2 bar tripped (${b2})`);
    // A1: status pill and motion controls during the hold; E-stop stays usable
    const hold = await page.evaluate(() => ({ badge: document.getElementById('state-badge').textContent, cls: document.getElementById('state-badge').className,
      locked: document.body.classList.contains('locked'), en: document.getElementById('btn-enable').classList.contains('on'),
      estop: !document.getElementById('btn-estop').disabled, jogOp: getComputedStyle(document.querySelector('.jbtn') || document.body).opacity }));
    ok(hold.badge === E.hold && hold.cls.includes('is-warn'), `${tag} A1 status pill "${hold.badge}" (${hold.cls})`);
    ok(hold.locked && hold.en && hold.estop, `${tag} A1 controls locked, enable stays on, E-stop usable ${JSON.stringify(hold)}`);
    ok(Number(hold.jogOp) < 1, `${tag} A1 jog buttons dimmed (opacity ${hold.jogOp})`);
    const nJ = P.cmds.length;
    await page.keyboard.down('KeyQ'); await page.waitForTimeout(250); await page.keyboard.up('KeyQ'); await page.waitForTimeout(100);
    ok(!P.cmds.slice(nJ).some((c) => /motion|action/.test(c.path)), `${tag} A1 jog key sends nothing during the hold ${JSON.stringify(P.cmds.slice(nJ))}`);
    await page.evaluate(() => window.scrollTo(0, 0));
    await shot(page, `devplat_collision_${tag}`);
    // not dismissable (Escape is the cockpit's E-stop key, so only a click is tried)
    await page.click('#col-banner', { force: true }); await page.waitForTimeout(150);
    ok(await vis(page, 'col-banner'), `${tag} banner stays after a click`);
    P.scene = 'waiting'; await page.waitForTimeout(250);
    tt = await text(page, 'col-title');
    ok(tt === E.waiting, `${tag} waiting banner "${tt}"`);

    P.scene = 'escalated'; await page.waitForTimeout(250);
    tt = await text(page, 'col-title');
    ok(tt === E.esc, `${tag} escalated title "${tt}"`);
    ok(await vis(page, 'col-clear') && (await text(page, 'col-clear')) === E.clear, `${tag} escalated: clear-fault button`);
    const fm = await text(page, 'fault-msg');
    ok(fm.includes(E.esc.slice(3)), `${tag} fault-msg names the collision fault: "${fm}"`);
    await shot(page, `devplat_escalated_${tag}`);
    const n0 = P.cmds.length;
    await page.click('#col-clear'); await page.waitForTimeout(300);
    ok(P.cmds.slice(n0).some((c) => c.path === '/api/v1/safety/clear_fault'), `${tag} clear button sends clear_fault (${JSON.stringify(P.cmds.slice(n0))})`);

    P.scene = 'bus'; await page.waitForTimeout(250);
    ok(!(await vis(page, 'col-banner')), `${tag} bus loss: no collision banner`);
    const fm2 = await text(page, 'fault-msg');
    ok(fm2.includes(E.bus) && fm2.includes(E.busDo), `${tag} BUS_LOSS fault text "${fm2}"`);
    ok(await noHScroll(page), `${tag} no horizontal scroll`);
    ok(P.errs.length === 0, `${tag} page errors: ${P.errs.join(' | ')}`);
    await closePage(P);

    // ---------- simple mode: banner, fault line, drawers ----------
    tag = `simple_${lang}_${vp}`;
    P = await openPage({ lang, ui: 'simple', viewport });
    page = P.page;
    P.scene = 'active'; await page.waitForTimeout(250);
    tt = await text(page, 'col-title');
    ok(tt === E.active && await inView(page, 'col-banner'), `${tag} active banner "${tt}" in view`);
    ok((await text(page, 's-ready-txt')) === E.holdLock, `${tag} A1 simple status "${await text(page, 's-ready-txt')}"`);
    await shot(page, `devplat_collision_${tag}`);
    {
      await page.click('#btn-pset'); await page.waitForTimeout(400);
      const goOp = await page.evaluate(() => getComputedStyle(document.querySelector('#pose-list .pose-go')).opacity);
      ok(Number(goOp) < 1, `${tag} A1 pose go dimmed during the hold (opacity ${goOp})`);
      const n2 = P.cmds.length;
      await page.click('#pose-list li[data-name="bin_a"] .pose-go'); await page.waitForTimeout(300);
      ok(P.cmds.length === n2, `${tag} A1 pose go sends nothing during the hold ${JSON.stringify(P.cmds.slice(n2))}`);
      await shot(page, `devplat_hold_poses_${tag}`);
      await page.click('#pset-close'); await page.waitForTimeout(200);
    }
    P.scene = 'bus'; await page.waitForTimeout(250);
    const rd = await text(page, 's-ready-txt');
    ok(rd.includes(E.bus) && rd.includes(E.busDo), `${tag} simple status line "${rd}"`);
    P.scene = 'normal'; await page.waitForTimeout(250);
    ok(!(await vis(page, 'col-banner')), `${tag} banner hides after auto-resume`);
    ok((await text(page, 'state-badge')) === E.armed && !(await page.evaluate(() => document.body.classList.contains('locked'))), `${tag} A1 enabled and unlocked after auto-resume`);

    // settings drawer
    ok((await page.getAttribute('#btn-sset', 'aria-label')) === E.settings, `${tag} settings aria-label`);
    await page.click('#btn-sset'); await page.waitForTimeout(300);
    ok(await vis(page, 'sset'), `${tag} settings drawer open`);
    ok(await page.isChecked('#col-en'), `${tag} guard toggle on`);
    ok((await page.evaluate(() => document.querySelector('#col-sens button.on')?.dataset.v)) === 'medium', `${tag} sensitivity medium`);
    let n1 = P.cmds.length;
    await page.click('#col-sens button[data-v="high"]'); await page.waitForTimeout(400);
    ok(JSON.stringify(P.cmds.slice(n1)) === JSON.stringify([{ path: '/api/v1/safety/collision', body: { sensitivity: 'high' } }]), `${tag} sensitivity command ${JSON.stringify(P.cmds.slice(n1))}`);
    ok((await page.evaluate(() => document.querySelector('#col-sens button.on')?.dataset.v)) === 'high', `${tag} sensitivity shows high`);
    ok(await page.evaluate(() => COL.thr && Math.abs(COL.thr[0] - 4.2) < 1e-6), `${tag} thresholds reloaded (x0.7)`);
    await shot(page, `devplat_settings_${tag}`);
    n1 = P.cmds.length;
    await page.click('#col-en', { force: true }); await page.waitForTimeout(400);
    ok(JSON.stringify(P.cmds.slice(n1)) === JSON.stringify([{ path: '/api/v1/safety/collision', body: { enabled: false } }]), `${tag} toggle command ${JSON.stringify(P.cmds.slice(n1))}`);
    ok(!(await page.isChecked('#col-en')), `${tag} toggle off`);
    const sens = await page.evaluate(() => ({ dis: [...document.querySelectorAll('#col-sens button')].map((b) => b.disabled), off: document.getElementById('col-sens').classList.contains('off'),
      op: getComputedStyle(document.getElementById('col-sens')).opacity }));
    ok(sens.dis.every(Boolean) && sens.off && Number(sens.op) < 1, `${tag} A2 sensitivity disabled and greyed while off ${JSON.stringify(sens)}`);
    n1 = P.cmds.length;
    await page.click('#col-sens button[data-v="low"]', { force: true }); await page.waitForTimeout(250);
    ok(P.cmds.length === n1, `${tag} A2 sensitivity click while off sends nothing`);
    await shot(page, `devplat_settings_off_${tag}`);
    await page.click('#col-en', { force: true }); await page.waitForTimeout(400);
    ok(await page.evaluate(() => [...document.querySelectorAll('#col-sens button')].every((b) => !b.disabled)), `${tag} A2 sensitivity re-enabled when the guard is back on`);
    await page.click('#sset-close'); await page.waitForTimeout(200);

    // poses drawer
    ok((await page.getAttribute('#btn-pset', 'aria-label')) === E.poses, `${tag} poses aria-label`);
    await page.click('#btn-pset'); await page.waitForTimeout(400);
    let names = await page.evaluate(() => [...document.querySelectorAll('#pose-list li')].map((li) => li.dataset.name));
    ok(JSON.stringify(names) === '["bin_a","inspect-left"]', `${tag} pose list ${JSON.stringify(names)}`);
    n1 = P.cmds.length;
    await page.fill('#pose-name', 'bad name!'); await page.click('#pose-save'); await page.waitForTimeout(200);
    ok(P.cmds.length === n1, `${tag} invalid name sends nothing`);
    ok((await text(page, 'pose-rule')).startsWith(E.bad), `${tag} invalid name hint "${await text(page, 'pose-rule')}"`);
    await page.fill('#pose-name', 'x'.repeat(65)); ok(!(await page.evaluate(() => POSE_NAME_RE.test(document.getElementById('pose-name').value))) || (await page.inputValue('#pose-name')).length === 64, `${tag} 65 chars rejected or truncated`);
    await page.fill('#pose-name', 'bin_b'); await page.click('#pose-save'); await page.waitForTimeout(400);
    ok(JSON.stringify(P.cmds.slice(n1)) === JSON.stringify([{ path: '/api/v1/poses/save', body: { name: 'bin_b' } }]), `${tag} save command ${JSON.stringify(P.cmds.slice(n1))}`);
    names = await page.evaluate(() => [...document.querySelectorAll('#pose-list li')].map((li) => li.dataset.name));
    ok(names.includes('bin_b') && names.length === 3, `${tag} list after save ${JSON.stringify(names)}`);
    n1 = P.cmds.length;
    await page.click('#pose-list li[data-name="bin_a"] .pose-go'); await page.waitForTimeout(300);
    const go = P.cmds.slice(n1);
    ok(go.length === 1 && go[0].path === '/api/v1/action/move_to_pose' && go[0].body.name === 'bin_a' && go[0].body.speed_scale > 0, `${tag} move_to_pose ${JSON.stringify(go)}`);
    n1 = P.cmds.length;
    await page.click('#pose-list li[data-name="inspect-left"] .pose-del'); await page.waitForTimeout(150);
    ok(P.cmds.length === n1, `${tag} first delete tap only arms`);
    await shot(page, `devplat_poses_${tag}`);
    await page.click('#pose-list li[data-name="inspect-left"] .pose-del'); await page.waitForTimeout(400);
    ok(JSON.stringify(P.cmds.slice(n1)) === JSON.stringify([{ path: '/api/v1/poses/delete', body: { name: 'inspect-left' } }]), `${tag} delete command ${JSON.stringify(P.cmds.slice(n1))}`);
    names = await page.evaluate(() => [...document.querySelectorAll('#pose-list li')].map((li) => li.dataset.name));
    ok(!names.includes('inspect-left'), `${tag} list after delete ${JSON.stringify(names)}`);
    ok(await noHScroll(page), `${tag} no horizontal scroll`);
    ok(P.errs.length === 0, `${tag} page errors: ${P.errs.join(' | ')}`);
    await closePage(P);
  }

  // ---------- older armd: no collision guard, no poses ----------
  caps = CAPS_OLD;
  const P = await openPage({ lang: 'zh', ui: 'expert', viewport: { width: 1280, height: 800 }, scene: 'old' });
  ok(!(await vis(P.page, 'btn-sset')) && !(await vis(P.page, 'btn-pset')), 'old armd: settings and poses buttons hidden');
  ok(!(await vis(P.page, 'rt-bars')), 'old armd: residual bars hidden (no thresholds)');
  ok(!(await vis(P.page, 'col-banner')), 'old armd: no banner');
  ok(P.errs.length === 0, `old armd page errors: ${P.errs.join(' | ')}`);
  await shot(P.page, 'devplat_old_armd_expert_zh_1280x800');
  await closePage(P);
} finally {
  await browser.close();
  srv.close();
}
console.log(`${shots.length} screenshots in ${out}, ${checks} checks, ${bad ? bad + ' FAILED' : 'all passed'}`);
for (const s of shots) console.log('  ' + s);
process.exit(bad ? 1 : 0);
