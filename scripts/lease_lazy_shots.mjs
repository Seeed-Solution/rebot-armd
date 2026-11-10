#!/usr/bin/env node
// Cockpit lease behaviour against a running armd with the mock follower:
//   ./target/debug/armd --follower mock --port 18766 &
//   ARMD=http://127.0.0.1:18766 PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome \
//     OUT=/dir node scripts/lease_lazy_shots.mjs
// The page is opened with ?lease_idle_ms=2000 (default 30000). WebSockets are not mocked: the
// page's frames are read from Playwright's websocket events, a second client is a Node
// WebSocket. Asserts: (a) no acquire_lease on open and observation lease_holder null;
// (b) the first jog acquires; no release while torqued; (c) release after the idle timeout
// once torque is off; (d) no release while an action runs (live), and each release blocker
// of the predicate; (e) a second cockpit tab and an agent holding the lease put the page in
// observer mode with the holder named, takeover with confirmation works; (f) E-stop works in
// observer mode. Writes 1280x800 and 375x812 PNGs (zh and en) to OUT
// (default .cache/lease-lazy-shots/).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/lease-lazy-shots');
fs.mkdirSync(out, { recursive: true });
const ARMD = (process.env.ARMD || 'http://127.0.0.1:18766').replace(/\/$/, '');
const WS_URL = ARMD.replace(/^http/, 'ws') + '/api/v1/ws';
const IDLE_MS = 2000;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const obs = async () => (await fetch(ARMD + '/api/v1/observation')).json();
const pass = (m) => console.log('PASS ' + m);
async function until(fn, ms, what) {
  const t0 = Date.now();
  for (;;) {
    const v = await fn();
    if (v) return v;
    if (Date.now() - t0 > ms) throw new Error('timeout: ' + what);
    await sleep(50);
  }
}

// A WebSocket client holding the lease the way rebot-companion does (heartbeat every 150 ms).
async function agent(label, { force = false } = {}) {
  const ws = new WebSocket(WS_URL);
  const a = { ws, gen: null, epoch: 1, req: 0, replies: [], closed: false };
  ws.onmessage = (ev) => {
    const o = JSON.parse(ev.data);
    if (o.epoch_id) a.epoch = o.epoch_id;
    if (o.type === 'lease_granted') a.gen = o.generation;
    if (o.type && o.type !== 'lease_granted') a.replies.push(o);
  };
  ws.onclose = () => { a.closed = true; };
  await new Promise((r) => { ws.onopen = r; });
  ws.send(JSON.stringify({ type: 'acquire_lease', seq: 1, client_id: label, force }));
  await until(() => a.gen != null, 3000, label + ' lease');
  a.hb = setInterval(() => {
    if (ws.readyState === 1 && a.gen != null) ws.send(JSON.stringify({ type: 'heartbeat', client_id: label, generation: a.gen, epoch_id: a.epoch }));
  }, 150);
  a.cmd = async (p, body) => {
    const request_id = ++a.req;
    ws.send(JSON.stringify({ type: 'command', request_id, path: p, body, generation: a.gen, epoch_id: a.epoch, client_id: label }));
    return until(() => a.replies.find((o) => o.request_id === request_id), 3000, p);
  };
  a.release = () => { ws.send(JSON.stringify({ type: 'release_lease', client_id: label, generation: a.gen })); a.gen = null; };
  a.close = () => { clearInterval(a.hb); try { ws.close(); } catch (_) {} };
  return a;
}

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
async function openPage(viewport, lang = 'zh') {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); localStorage.setItem('rebot.lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  const sent = [];
  page.on('websocket', (ws) => {
    if (!ws.url().endsWith('/api/v1/ws')) return;
    ws.on('framesent', (f) => { try { sent.push(JSON.parse(f.payload)); } catch (_) {} });
  });
  page.on('pageerror', (e) => { console.log('PAGE ERROR', e.message); });
  await page.goto(`${ARMD}/?lease_idle_ms=${IDLE_MS}`, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 });
  await page.evaluate((l) => setLang(l), lang);
  return { ctx, page, sent, kinds: () => sent.map((o) => o.type) };
}
const shot = async (page, name) => { const f = path.join(out, name + '.png'); await page.screenshot({ path: f }); console.log('SHOT ' + f); };
const clientId = (page) => page.evaluate(() => CLIENT_ID);
const held = (page) => page.evaluate(() => S.leaseGeneration != null);
const lockText = (page) => page.evaluate(() => document.getElementById('lock-txt').textContent);
async function resetArm() {
  // leave the mock Idle, torque off, no fault, no lease
  const a = await agent('test-reset', { force: true });
  let o = await obs();
  if (o.safety_state === 'EStop' || (o.safety_state && o.safety_state.Fault)) await a.cmd('/api/v1/safety/clear_fault', {});
  o = await obs();
  if (o.is_torqued) {
    a.ws.send(JSON.stringify({ type: 'preset', name: 'home', epoch_id: a.epoch, client_id: 'test-reset', generation: a.gen }));
    await until(async () => { const x = await obs(); return x.action?.state !== 'running' && x.positions.every((v) => Math.abs(v) < 1e-3); }, 15000, 'reset home');
    await a.cmd('/api/v1/torque', { enable: false });
  }
  a.release(); await sleep(100); a.close();
  await until(async () => { const x = await obs(); return !x.is_torqued && x.lease_holder == null; }, 5000, 'reset');
}

try {
  await resetArm();
  // ---------- (a) no acquire on open ----------
  const A = await openPage({ width: 1280, height: 800 });
  await sleep(3000);
  assert.ok(!A.kinds().includes('acquire_lease'), 'acquire_lease sent on open: ' + A.kinds().join(','));
  assert.equal((await obs()).lease_holder, null);
  assert.equal(await held(A.page), false);
  pass('(a) no acquire_lease within 3 s of opening; observation lease_holder null; frames sent: ' + JSON.stringify([...new Set(A.kinds())]));
  await shot(A.page, 'a-free-1280x800-zh');

  // ---------- (b) first jog acquires ----------
  // The mock arm needs torque for a jog: another client enables it and releases (free + torqued);
  // the page's jog must come within the 2.5 s lease watchdog.
  const pre = await agent('test-arm');
  await pre.cmd('/api/v1/torque', { enable: true });
  pre.release(); await sleep(50); pre.close();
  await until(async () => (await obs()).lease_holder == null, 2000, 'pre released');
  await A.page.waitForFunction(() => S.state === 'Armed' && !document.body.classList.contains('locked'), null, { timeout: 2000 });
  const before = A.sent.length;
  await A.page.keyboard.down('d');
  await sleep(500);
  await A.page.keyboard.up('d');
  const jogFrames = A.sent.slice(before).map((o) => o.type);
  assert.equal(jogFrames[0], 'acquire_lease', 'first frame of the jog: ' + jogFrames.slice(0, 5));
  assert.ok(jogFrames.includes('servo'), 'servo frames after the grant');
  await until(() => held(A.page), 2000, 'page lease');
  const idA = await clientId(A.page);
  assert.equal((await obs()).lease_holder, idA);
  pass(`(b) first jog sent acquire_lease then servo frames (${jogFrames.filter((k) => k === 'servo').length}); lease_holder = ${idA}`);

  // torque on: no idle release
  await sleep(IDLE_MS + 1500);
  assert.ok(!A.sent.some((o) => o.type === 'release_lease'), 'released while torqued');
  assert.equal((await obs()).lease_holder, idA);
  pass(`(b) no release_lease ${IDLE_MS + 1500} ms after the last input while torqued`);

  // ---------- (d) live: no release while an action runs ----------
  // a long joint move started by this page; its idle timer is then set as expired
  const actAck = await A.page.evaluate(async () => {
    await ensureLease();
    const q = S.obs.positions.map(() => 0); q[0] = 2.8;
    S.ws.send(JSON.stringify({ type: 'preset', name: 'lease-test', positions: q, epoch_id: S.epoch, client_id: CLIENT_ID, generation: S.leaseGeneration }));
    return true;
  });
  await until(async () => (await obs()).action?.state === 'running', 2000, 'action running');
  {
    const t0 = Date.now();
    let lastState = 'running', n = 0;
    while (Date.now() - t0 < 3000) {
      await A.page.evaluate(() => { S.lastIntentAt = Date.now() - 60000; });
      await sleep(300);
      const o = await obs();
      lastState = o.action?.state;
      if (lastState !== 'running') break;
      n++;
      assert.equal(o.lease_holder, idA, 'lease released during a running action');
    }
    assert.ok(n >= 2, 'action ran too briefly: ' + n);
    assert.ok(!A.sent.some((o) => o.type === 'release_lease'));
    pass(`(d) live: joint_move action running with the idle timer expired for ${n} checks over ${Date.now() - t0} ms, lease kept (state then ${lastState})`);
  }
  // back to the resting pose for torque off
  await until(async () => (await obs()).action?.state !== 'running', 15000, 'action end');
  await A.page.evaluate(() => runPreset('home'));
  await until(async () => { const o = await obs(); return o.action?.state !== 'running' && o.positions.every((x) => Math.abs(x) < 1e-3); }, 15000, 'home');
  // predicate: with torque off and the input idle, each blocker on its own keeps the lease
  const pred = await A.page.evaluate(() => {
    const saved = S.obs, savedAt = S.lastIntentAt;
    const base = { ...saved, is_torqued: false, enable_state: 'Disabled', disable_pending: false, action: null, control_mode: 'position', gravity_comp: { ...saved.gravity_comp, phase: null }, velocities: saved.velocities.map(() => 0), safety_state: 'Idle' };
    const st = S.state;
    S.lastIntentAt = Date.now() - 60000; S.state = 'Idle';
    const r = {};
    const cases = {
      none: {}, torqued: { is_torqued: true }, action_running: { action: { kind: 'joint_move', state: 'running' } }, action_done: { action: { kind: 'joint_move', state: 'done' } },
      gravity_comp: { control_mode: 'gravity_comp' }, moving: { velocities: base.velocities.map((v, i) => (i === 1 ? 0.2 : 0)) }, disable_pending: { disable_pending: true },
    };
    for (const [k, v] of Object.entries(cases)) { S.obs = { ...base, ...v }; r[k] = leaseIdleReleasable(); }
    S.obs = { ...base }; S.streamActive = true; r.stream = leaseIdleReleasable(); S.streamActive = false;
    S.obs = { ...base }; S.presetRunning = true; r.preset = leaseIdleReleasable(); S.presetRunning = false;
    S.obs = { ...base }; S.lastIntentAt = Date.now(); r.recent_input = leaseIdleReleasable();
    S.obs = saved; S.lastIntentAt = savedAt; S.state = st;
    return r;
  });
  assert.deepEqual(pred, { none: true, torqued: false, action_running: false, action_done: true, gravity_comp: false, moving: false, disable_pending: false, stream: false, preset: false, recent_input: false });
  pass('(d) predicate: releasable only without torque, running action, gravity comp, motion, stream, preset or recent input ' + JSON.stringify(pred));

  // ---------- (c) idle release after torque off ----------
  const off = await A.page.evaluate(async () => { try { await commandRPC('/api/v1/torque', { enable: false }); return 'ok'; } catch (e) { return e.code || e.message; } });
  assert.equal(off, 'ok', 'torque off: ' + off);
  const tOff = Date.now();
  await until(async () => !(await obs()).is_torqued, 2000, 'torque off');
  await until(() => A.sent.some((o) => o.type === 'release_lease'), IDLE_MS + 3000, 'idle release');
  const dt = Date.now() - tOff;
  assert.ok(dt >= IDLE_MS - 100, 'released too early: ' + dt);
  await until(async () => (await obs()).lease_holder == null, 1000, 'holder cleared');
  assert.equal(await held(A.page), false);
  pass(`(c) release_lease ${dt} ms after the last command (idle ${IDLE_MS} ms, torque off); observation lease_holder null`);
  await sleep(300);
  await shot(A.page, 'c-released-1280x800-zh');

  // ---------- (e) second cockpit tab -> observer ----------
  // tab A takes the lease again with its first intent (torque on via the enable hold)
  const enableBox = await A.page.locator('#btn-enable').boundingBox();
  await A.page.mouse.move(enableBox.x + enableBox.width / 2, enableBox.y + enableBox.height / 2);
  await A.page.mouse.down(); await sleep(800); await A.page.mouse.up();
  await until(async () => (await obs()).is_torqued && (await obs()).lease_holder === idA, 3000, 'tab A enable');
  pass('(e) enable hold acquired the lease for tab A and torqued the arm');
  const B = await openPage({ width: 375, height: 812 });
  await sleep(1000);
  assert.ok(!B.kinds().includes('acquire_lease'));
  const lockB = await lockText(B.page);
  assert.match(lockB, /另一个驾驶舱页面/);
  assert.equal(await B.page.locator('#btn-preempt').isVisible(), true, 'takeover button in observer mode');
  assert.equal(await B.page.evaluate(() => document.body.classList.contains('locked')), true);
  // a key jog in tab B sends nothing
  await B.page.keyboard.down('d'); await sleep(300); await B.page.keyboard.up('d');
  assert.ok(!B.kinds().some((k) => k === 'servo' || k === 'acquire_lease'), 'observer tab sent ' + B.kinds());
  pass(`(e) tab B is an observer while tab A holds the lease: "${lockB}"; jog sent no frames`);
  await shot(B.page, 'e-observer-cockpit-375x812-zh');

  // tab A releases by closing; agent takes the lease
  await A.page.evaluate(() => commandRPC('/api/v1/torque', { enable: false }).catch(() => {}));
  await until(async () => !(await obs()).is_torqued, 2000, 'A torque off');
  await A.ctx.close();
  await until(async () => (await obs()).lease_holder == null, 2000, 'A closed');
  const mcp = await agent('rebot-mcp');
  await B.page.waitForFunction(() => /AI/.test(document.getElementById('lock-txt').textContent), null, { timeout: 2000 });
  const lockMcp = await lockText(B.page);
  assert.match(lockMcp, /AI 智能体（rebot-mcp）/);
  pass(`(e) agent rebot-mcp holds the lease (plain acquire accepted while the cockpit is open): "${lockMcp}"`);
  await shot(B.page, 'e-observer-agent-375x812-zh');
  const D = await openPage({ width: 1280, height: 800 }, 'en');
  await D.page.waitForFunction(() => /AI agent/.test(document.getElementById('lock-txt').textContent), null, { timeout: 3000 });
  const lockEn = await lockText(D.page);
  assert.match(lockEn, /Observer mode: an AI agent \(rebot-mcp\) is controlling the arm/);
  const whoEn = await D.page.locator('#preempt-who').textContent();
  assert.equal(whoEn, 'Held by an AI agent (rebot-mcp)');
  assert.equal(await D.page.locator('#btn-preempt').isVisible(), true);
  pass(`(e) en text: "${lockEn}"; takeover button: "${whoEn}"`);
  await shot(D.page, 'e-observer-agent-1280x800-en');
  const E = await openPage({ width: 375, height: 812 }, 'en');
  await sleep(800);
  await shot(E.page, 'e-observer-agent-375x812-en');
  await E.ctx.close();

  // ---------- (f) E-stop in observer mode ----------
  await D.page.locator('#btn-estop').dispatchEvent('pointerdown');
  await until(async () => (await obs()).safety_state === 'EStop', 2000, 'estop');
  assert.ok(!D.kinds().includes('acquire_lease'), 'E-stop must not acquire');
  pass('(f) E-stop from the observer page latched EStop without a lease; frames: ' + JSON.stringify([...new Set(D.kinds())]));
  await sleep(400);
  await shot(D.page, 'f-estop-observer-1280x800-en');
  await mcp.cmd('/api/v1/safety/clear_fault', {});

  // ---------- (e) takeover with confirmation ----------
  await sleep(300);
  await B.page.locator('#btn-preempt').click();
  await B.page.locator('#preempt-modal').waitFor({ state: 'visible' });
  await shot(B.page, 'e-takeover-modal-375x812-zh');
  await B.page.locator('#preempt-confirm').click();
  await until(() => held(B.page), 3000, 'takeover');
  const forced = B.sent.filter((o) => o.type === 'acquire_lease');
  assert.equal(forced.length, 1); assert.equal(forced[0].force, true);
  assert.equal((await obs()).lease_holder, await clientId(B.page));
  await D.page.waitForFunction(() => /another cockpit page/.test(document.getElementById('lock-txt').textContent), null, { timeout: 2000 });
  pass('(e) takeover: one acquire_lease force=true after the confirm; lease_holder = tab B; tab D now shows "' + (await lockText(D.page)) + '"');
  mcp.close();
} finally {
  await browser.close();
  await resetArm().catch(() => {});
}
console.log('ALL PASS');
