// Cockpit connection-loss warning and reconnect (Playwright against a mock armd).
//
// A routed WebSocket proxy between page and armd simulates an outage: open sockets are closed and
// new ones are closed on arrival while armd keeps running (its lease watchdog parks the arm). Checks:
// - the lease status element shows "连接已断开 · 正在重连" / "Connection lost · reconnecting" for the whole outage;
// - one retry timer: attempts after 1, 2, 4, 8 s, then every 15 s; reset by the first valid observation;
// - recovery is read-only (0 acquire_lease) and shows the release notice with the arm state taken from
//   fresh telemetry ("parked" only when armd reports a finished park action and torque off; the lease
//   watchdog park is not an action in the observation, so after it the notice says "torque off");
// - "Take control" sends exactly one acquire_lease;
// - a retry socket that opens but delivers no observation is retired after the telemetry deadline and the
//   retries keep backing off (silent-open);
// - returning to the tab (visibilitychange) while a retry is queued does not add attempts (repeated resume);
// - observations without a boolean is_torqued are not accepted: the warning stays, no arm-state notice,
//   the backoff is not reset;
// - the lost status counts down to the next retry and "reconnect now" retries at once;
// - phone expert layout (400 px): the header badge carries the warning.
// Screenshots (zh) and the page console go to SHOT_DIR (default: a temp dir).
// Run: cargo build --bin armd && node tests/test_cockpit_reconnect_playwright.js
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');
const { spawn } = require('node:child_process');
const { chromium } = require('playwright');

const ROOT = path.resolve(__dirname, '..');
const ARMD_BIN = process.env.ARMD_BIN || path.join(ROOT, 'target', 'debug', 'armd');
const B601_CONFIG = path.join(ROOT, 'kits', 'recamera-b601dm', 'profile', 'rebot-b601dm.json');
const SHOT_DIR = process.env.SHOT_DIR || fs.mkdtempSync(path.join(os.tmpdir(), 'armd-cockpit-reconnect-'));
const FORBIDDEN_PORTS = new Set([18765, 18001, 18990, 8001, 8090, 8765]);
const RETRY_MS = [1000, 2000, 4000, 8000, 15000, 15000];
const LOST_ZH = '连接已断开 · 正在重连', LOST_EN = 'Connection lost · reconnecting', LOST_T_ZH = '连接已断开';
const PARKED_ZH = '机械臂已停靠，力矩已关闭', TORQUE_OFF_ZH = '机械臂力矩已关闭', TORQUED_ZH = '机械臂仍带力矩';

const log = (...a) => console.log(`[${new Date().toISOString().slice(11, 23)}]`, ...a);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function waitFor(fn, { timeoutMs = 10000, intervalMs = 50, label = 'condition' } = {}) {
  const end = Date.now() + timeoutMs;
  for (;;) {
    const v = await fn();
    if (v) return v;
    if (Date.now() > end) throw new Error(`timeout waiting for ${label}`);
    await sleep(intervalMs);
  }
}

async function freePort() {
  for (let i = 0; i < 16; i++) {
    const port = await new Promise((resolve, reject) => {
      const srv = net.createServer();
      srv.once('error', reject);
      srv.listen(0, '127.0.0.1', () => { const p = srv.address().port; srv.close(() => resolve(p)); });
    });
    if (!FORBIDDEN_PORTS.has(port)) return port;
  }
  throw new Error('no free port');
}

const getObs = async (port) => (await fetch(`http://127.0.0.1:${port}/api/v1/observation`, { cache: 'no-store' })).json();

async function startArmd(port) {
  assert(fs.existsSync(ARMD_BIN), `armd binary not found at ${ARMD_BIN} (cargo build --bin armd)`);
  const state = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-reconnect-state-'));
  const args = ['--follower', 'mock', '--config', B601_CONFIG, '--port', String(port), '--log-dir', 'off', '--blackbox-dir', 'off', '--episodes-dir', 'off'];
  const logs = [];
  const child = spawn(ARMD_BIN, args, {
    cwd: ROOT, stdio: ['ignore', 'pipe', 'pipe'],
    env: { ...process.env, RUST_LOG: 'warn', ARMD_APP_CONFIG: 'off', ARMD_POSES: path.join(state, 'poses.json'), ARMD_SEQUENCES: path.join(state, 'sequences.json') },
  });
  let spawnErr = null;
  child.once('error', (err) => { spawnErr = err; });
  child.stdout.on('data', (c) => logs.push(String(c)));
  child.stderr.on('data', (c) => logs.push(String(c)));
  // stop() ends the child and removes the temp state, also when spawn failed or the child already exited.
  const stop = () => new Promise((resolve) => {
    const done = () => { fs.rmSync(state, { recursive: true, force: true }); resolve(); };
    if (spawnErr || child.pid == null || child.exitCode !== null || child.signalCode !== null) { done(); return; }
    const kt = setTimeout(() => { try { child.kill('SIGKILL'); } catch (_) {} }, 3000);
    child.once('exit', () => { clearTimeout(kt); done(); });
    try { child.kill('SIGTERM'); } catch (_) { clearTimeout(kt); done(); }
  });
  try {
    await waitFor(async () => {
      if (spawnErr) throw new Error(`failed to spawn armd (${ARMD_BIN}): ${spawnErr.message}`);
      if (child.exitCode !== null) throw new Error(`armd exited early:\n${logs.join('')}`);
      try { const o = await getObs(port); return Array.isArray(o.positions) && o.positions.length > 0; } catch (_) { return false; }
    }, { timeoutMs: 15000, intervalMs: 100, label: 'armd ready' });
  } catch (e) {
    await stop();
    throw e;
  }
  return { stop };
}

// Page + routed WebSocket proxy. `link.outage = true` closes the open socket pair and every new one;
// `link.silent = true` lets new sockets open but forwards nothing from armd; `link.stripTorque = true`
// removes is_torqued from every observation armd sends.
async function openCockpit(browser, port, viewport = { width: 1280, height: 800 }) {
  const ctx = await browser.newContext({ viewport, locale: 'zh-CN', hasTouch: viewport.width < 1000 });
  const page = await ctx.newPage();
  const consoleLines = [], pageErrors = [];
  page.on('console', (m) => consoleLines.push(`${m.type()}: ${m.text()}`));
  page.on('pageerror', (e) => pageErrors.push(e.message || String(e)));
  page.on('dialog', (d) => { pageErrors.push(`dialog: ${d.message()}`); d.dismiss().catch(() => {}); });
  await page.route('**/*', (route) => {
    const u = new URL(route.request().url());
    if (u.protocol === 'data:' || u.protocol === 'blob:') return route.continue();
    if (u.hostname !== '127.0.0.1' || Number(u.port) !== port) return route.abort('blockedbyclient');
    if (u.pathname.startsWith('/api/v1/detection/') || u.pathname.startsWith('/api/v1/vision/')) {
      return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ enabled: false, mode: 'off', available: {} }) });
    }
    return route.continue();
  });
  const sent = [], attempts = [], link = { outage: false, silent: false, stripTorque: false, pair: null };
  await page.routeWebSocket(/.*/, (ws) => {
    const u = new URL(ws.url());
    if (u.hostname !== '127.0.0.1' || Number(u.port) !== port || u.pathname !== '/api/v1/ws') { ws.close(); return; }
    attempts.push(Date.now());
    if (link.outage) { ws.close({ code: 4000, reason: 'outage' }); return; }
    const server = ws.connectToServer();
    const pair = { ws, server };
    link.pair = pair;
    ws.onMessage((raw) => { let j = null; try { j = JSON.parse(String(raw)); } catch (_) {} sent.push({ ts: Date.now(), json: j }); server.send(raw); });
    server.onMessage((raw) => {
      if (link.silent && link.pair === pair) return;
      if (link.stripTorque) {
        let j = null; try { j = JSON.parse(String(raw)); } catch (_) {}
        if (j && !j.type && 'is_torqued' in j) { delete j.is_torqued; ws.send(JSON.stringify(j)); return; }
      }
      ws.send(raw);
    });
  });
  link.cut = async () => {
    link.outage = true;
    const p = link.pair; link.pair = null;
    if (p) { await p.server.close().catch(() => {}); await p.ws.close().catch(() => {}); }
  };
  const acquires = (since) => sent.filter((f) => f.ts >= since && f.json?.type === 'acquire_lease');
  return { ctx, page, link, attempts, acquires, consoleLines, pageErrors };
}

const live = (page, timeout = 15000) => page.waitForFunction(
  () => typeof obsFresh === 'function' && obsFresh() && S.dof > 0 && S.epoch != null && !S.connLost, null, { timeout });
const leaseStatus = (page) => page.evaluate(() => {
  const ls = document.getElementById('lease-status');
  return { hidden: ls.hidden, cls: ls.className, txt: document.getElementById('lease-status-txt').textContent.trim() };
});
const releaseNotice = (page) => page.evaluate(() => ({
  hidden: document.getElementById('lease-released').hidden,
  txt: document.getElementById('lease-released-txt').textContent.trim(),
}));

async function holdEnable(page) {
  const b = await (await page.waitForSelector('#btn-enable', { state: 'visible' })).boundingBox();
  await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2);
  await page.mouse.down(); await sleep(750); await page.mouse.up();
  await page.waitForFunction(() => S.state === 'Armed' && S.leaseGeneration != null && S.obs?.is_torqued === true, null, { timeout: 6000 });
}

async function assertLostShown(page, label) {
  const ls = await leaseStatus(page);
  assert(!ls.hidden && ls.cls.includes('is-lost') && ls.txt === LOST_ZH, `${label}: lease status ${JSON.stringify(ls)}`);
}

// Returning to the tab, as the page sees it (document.hidden false + visibilitychange).
const resume = (page) => page.evaluate(() => {
  Object.defineProperty(document, 'hidden', { configurable: true, get: () => false });
  document.dispatchEvent(new Event('visibilitychange'));
});
const gapsOf = (times) => times.slice(1).map((t, i) => t - times[i]);

async function main() {
  fs.mkdirSync(SHOT_DIR, { recursive: true });
  const port = await freePort();
  const armd = await startArmd(port);
  let browser = null, cp = null;
  try {
    browser = await chromium.launch({ executablePath: process.env.CHROME || undefined, headless: true, args: ['--no-sandbox', '--disable-setuid-sandbox'] });
    cp = await openCockpit(browser, port);
    await scenarios(browser, port, cp);
  } finally {
    if (cp) await cp.ctx.close().catch(() => {});
    if (browser) await browser.close().catch(() => {});
    await armd.stop();
  }
}

// ONLY=silent,resume,torque,countdown,phone,armtext runs just those cases (skips 1-4); default: all.
const ONLY = process.env.ONLY ? new Set(process.env.ONLY.split(',')) : null;
const want = (name) => !ONLY || ONLY.has(name);

async function scenarios(browser, port, cp) {
  const { page, link, attempts } = cp;
  if (ONLY) await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'domcontentloaded' });
  if (!ONLY) {
    await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'domcontentloaded' });
    await live(page);
    await holdEnable(page);
    // J1 away from rest so that the watchdog park has to move the arm.
    await page.evaluate(() => api('/api/v1/motion/jog', { target_positions: [1.2, 0, 0, 0, 0, 0, 0] }));
    await waitFor(async () => Math.abs((await getObs(port)).positions[0]) > 0.9, { timeoutMs: 12000, intervalMs: 100, label: 'J1 away' });
    const own = await leaseStatus(page);
    assert(!own.hidden && own.cls.includes('is-own'), `connected: ${JSON.stringify(own)}`);
    await page.screenshot({ path: path.join(SHOT_DIR, 's3_connected_zh.png') });
    log('PASS connected: in control, torqued, J1 away from rest');

    // 1. Prolonged outage: warning persists, retries back off 1, 2, 4, 8, 15, 15 s, armd parks the arm.
    const tCut = Date.now();
    const n0 = attempts.length;
    await link.cut();
    await page.waitForFunction((t) => document.getElementById('lease-status-txt').textContent.trim() === t, LOST_ZH, { timeout: 3000 });
    await assertLostShown(page, 'outage start');
    await page.screenshot({ path: path.join(SHOT_DIR, 's3_connection_lost_zh.png') });
    await page.click('#btn-lang');
    await page.waitForFunction((t) => document.getElementById('lease-status-txt').textContent.trim() === t, LOST_EN, { timeout: 1000 });
    await page.click('#btn-lang');
    await waitFor(() => attempts.length >= n0 + RETRY_MS.length, { timeoutMs: 60000, intervalMs: 100, label: 'retry attempts' });
    const times = [tCut, ...attempts.slice(n0, n0 + RETRY_MS.length)];
    const gaps = times.slice(1).map((t, i) => t - times[i]);
    log(`retry gaps (ms): ${gaps.join(', ')}`);
    gaps.forEach((g, i) => assert(g >= RETRY_MS[i] - 250 && g <= RETRY_MS[i] + 1000, `retry ${i + 1}: gap ${g} ms, want ~${RETRY_MS[i]}`));
    await assertLostShown(page, 'outage end');
    const parked = await getObs(port);
    assert(!parked.is_torqued && parked.lease_holder === null && Math.abs(parked.positions[0]) < 0.15, `watchdog park: ${JSON.stringify({ t: parked.is_torqued, h: parked.lease_holder, j1: parked.positions[0] })}`);
    assert.equal(cp.acquires(tCut).length, 0, 'acquire_lease during the outage');
    log('PASS outage: warning shown throughout, capped backoff, watchdog parked the arm, 0 acquire_lease');

    // 2. Recovery on the next retry: read-only, release notice with server-evidenced arm state.
    link.outage = false;
    await live(page, 20000);
    await page.waitForFunction(() => !document.getElementById('lease-released').hidden, null, { timeout: 3000 })
      .catch(async () => { throw new Error(`no release notice: ${JSON.stringify(await page.evaluate(() => ({ rel: S.leaseReleased, gen: S.leaseGeneration, holder: S.obs?.lease_holder, fresh: obsFresh() })))}`); });
    const rel = await releaseNotice(page);
    assert(!rel.hidden && rel.txt.includes('控制权已释放') && rel.txt.includes(TORQUE_OFF_ZH) && !rel.txt.includes(PARKED_ZH), `release notice: ${JSON.stringify(rel)}`);
    const ls2 = await leaseStatus(page);
    assert(ls2.cls.includes('is-released') && !ls2.cls.includes('is-lost'), `after recovery: ${JSON.stringify(ls2)}`);
    await page.screenshot({ path: path.join(SHOT_DIR, 's3_reconnected_released_zh.png') });
    await sleep(2000);
    assert.equal(cp.acquires(tCut).length, 0, 'acquire_lease before explicit retake');
    assert.equal(await page.evaluate(() => S.leaseGeneration), null);
    const tRetake = Date.now();
    await page.click('#btn-retake');
    await page.waitForFunction(() => S.leaseGeneration != null && document.getElementById('lease-released').hidden, null, { timeout: 5000 });
    const acq = cp.acquires(tRetake);
    assert.equal(acq.length, 1, 'one acquire_lease after explicit retake');
    assert.equal(acq[0].json.force, false);
    log(`PASS recovery: read-only, notice "${rel.txt}", retake sent 1 acquire_lease`);

    // 3. Retry counter reset by the valid observation; a quick clean reconnect while torqued says so.
    await holdEnable(page);
    const tDrop = Date.now(), n1 = attempts.length;
    await link.pair.ws.close();
    await waitFor(() => attempts.length > n1, { timeoutMs: 4000, label: 'first retry after reset' });
    const firstGap = attempts[n1] - tDrop;
    assert(firstGap >= 750 && firstGap <= 2000, `first retry after a reset: ${firstGap} ms, want ~1000`);
    await live(page);
    await page.waitForFunction(() => !document.getElementById('lease-released').hidden, null, { timeout: 4000 });
    const rel3 = await releaseNotice(page);
    const obs3 = await page.evaluate(() => ({ t: S.obs.is_torqued, park: S.obs.action?.kind === 'park' && S.obs.action?.state === 'running' }));
    if (obs3.t && !obs3.park) assert(rel3.txt.includes(TORQUED_ZH) && !rel3.txt.includes(PARKED_ZH), `torqued reconnect notice: ${rel3.txt}`);
    assert(!rel3.txt.includes(PARKED_ZH) || !obs3.t, `notice says parked while torqued: ${rel3.txt}`);
    log(`PASS reset: first retry after ${firstGap} ms; notice "${rel3.txt}" matches telemetry (torqued=${obs3.t})`);

    // 4. A park this page asked for (action park done, torque off) is the evidence for "parked".
    await page.click('#btn-retake');
    await page.waitForFunction(() => S.leaseGeneration != null, null, { timeout: 5000 });
    await page.evaluate(() => api('/api/v1/motion/unload', undefined, { quiet: true }));
    const done = await waitFor(async () => { const o = await getObs(port); return !o.is_torqued && o.action?.kind === 'park' && o.action?.state === 'done' ? o : null; },
      { timeoutMs: 20000, intervalMs: 100, label: 'park action done, torque off' });
    const n4 = attempts.length;
    await link.pair.ws.close();
    await waitFor(() => attempts.length > n4, { timeoutMs: 4000, label: 'retry after the park' });
    await live(page);
    await page.waitForFunction(() => !document.getElementById('lease-released').hidden, null, { timeout: 4000 });
    const rel4 = await releaseNotice(page);
    assert(rel4.txt.includes(PARKED_ZH), `parked notice: ${rel4.txt}`);
    log(`PASS parked evidence: action ${done.action.kind}/${done.action.state}, torque off -> "${rel4.txt}"`);
  }
  if (want('silent')) await silentOpen(page, link, attempts);
  if (want('resume')) await repeatedResume(page, link, attempts);
  if (want('torque')) await missingTorque(page, link, attempts);
  if (want('countdown')) await countdownAndReconnectNow(page, link, attempts, cp.acquires);
  if (want('phone')) await phoneExpert(browser, port);
  if (want('armtext')) { await live(page); await armTextCases(page, port, link); }

  assert.deepEqual(cp.pageErrors, [], 'page errors');
  const errors = cp.consoleLines.filter((l) => l.startsWith('error:'));
  fs.writeFileSync(path.join(SHOT_DIR, 's3_console.log'), cp.consoleLines.join('\n') + '\n');
  assert.deepEqual(errors, [], 'console errors');
  log(`ALL RECONNECT SCENARIOS PASSED (shots and console log in ${SHOT_DIR})`);
}

// Drop the live socket and wait for the warning (the drop itself counts as the start of the outage).
async function dropAndWaitLost(page, link, outage = true) {
  link.outage = outage;
  const t0 = Date.now();
  await link.cut();
  link.outage = outage;
  await page.waitForFunction(() => S.connLost === true, null, { timeout: 3000 });
  return t0;
}

// 5. Silent-open: the retry socket opens but armd's frames never arrive. The socket is retired after the
//    telemetry deadline (4 s) and the retries continue with the backoff; the warning stays.
async function silentOpen(page, link, attempts) {
  await live(page);
  link.silent = true;
  const n0 = attempts.length;
  const t0 = await dropAndWaitLost(page, link, false);
  await waitFor(() => attempts.length >= n0 + 3, { timeoutMs: 25000, intervalMs: 100, label: 'retries after silent sockets' });
  const gaps = gapsOf([t0, ...attempts.slice(n0, n0 + 3)]);
  log(`silent-open retry gaps (ms): ${gaps.join(', ')}`);
  // drop -> 1 s retry; each silent socket: 4 s deadline + the next backoff step (2 s, 4 s)
  [1000, 4000 + 2000, 4000 + 4000].forEach((want, i) => assert(gaps[i] >= want - 300 && gaps[i] <= want + 1500, `silent retry ${i + 1}: gap ${gaps[i]} ms, want ~${want}`));
  await assertLostShown(page, 'silent-open');
  link.silent = false;
  await live(page, 30000);
  log('PASS silent-open: silent sockets retired after the telemetry deadline, backoff kept, warning shown until valid telemetry');
}

// 6. Repeated resume while a retry is queued adds no attempt and does not shorten the backoff.
async function repeatedResume(page, link, attempts) {
  await live(page);
  const n0 = attempts.length;
  const t0 = await dropAndWaitLost(page, link, true);
  await waitFor(() => attempts.length >= n0 + 2, { timeoutMs: 6000, intervalMs: 50, label: 'two retries' });
  const tq = attempts[n0 + 1];
  for (let i = 0; i < 8; i++) { await resume(page); await sleep(250); }
  await waitFor(() => attempts.length >= n0 + 3, { timeoutMs: 8000, intervalMs: 50, label: 'third retry' });
  const gaps = gapsOf([t0, ...attempts.slice(n0, n0 + 3)]);
  log(`repeated-resume retry gaps (ms): ${gaps.join(', ')} (8 resumes during the 4 s gap)`);
  assert(attempts[n0 + 2] - tq >= 4000 - 250, `resume shortened the 4 s backoff: ${attempts[n0 + 2] - tq} ms`);
  await assertLostShown(page, 'repeated resume');
  link.outage = false;
  await live(page, 20000);
  log('PASS repeated resume: queued retry kept, no extra attempts');
}

// 7. Frames without is_torqued are not valid telemetry: no recovery, no arm-state notice, backoff not reset.
async function missingTorque(page, link, attempts) {
  await live(page);
  link.stripTorque = true;
  const n0 = attempts.length;
  const t0 = await dropAndWaitLost(page, link, false);
  await waitFor(() => attempts.length >= n0 + 3, { timeoutMs: 25000, intervalMs: 100, label: 'retries with frames lacking is_torqued' });
  const st = await page.evaluate(() => ({ lost: S.connLost, notice: !document.getElementById('lease-released').hidden, txt: document.getElementById('lease-released-txt').textContent }));
  assert(st.lost && !st.notice, `frames without is_torqued accepted: ${JSON.stringify(st)}`);
  const armTxt = await page.evaluate(() => connArmText({ ...S.obs, is_torqued: undefined, disable_pending: false, action: { kind: 'park', state: 'done' } }));
  assert(!armTxt.includes(PARKED_ZH) && !armTxt.includes(TORQUE_OFF_ZH), `arm text without is_torqued: ${armTxt}`);
  const gaps = gapsOf([t0, ...attempts.slice(n0, n0 + 3)]);
  log(`missing is_torqued retry gaps (ms): ${gaps.join(', ')}`);
  [1000, 4000 + 2000, 4000 + 4000].forEach((want, i) => assert(gaps[i] >= want - 300 && gaps[i] <= want + 1500, `missing is_torqued retry ${i + 1}: gap ${gaps[i]} ms, want ~${want}`));
  await assertLostShown(page, 'missing is_torqued');
  link.stripTorque = false;
  await live(page, 30000);
  log('PASS missing is_torqued: frames refused, warning kept, no parked notice, backoff intact');
}

// 8. The lost status counts down to the queued retry; "reconnect now" retries at once, sends
//    0 acquire_lease, and cancels the old queued timer (exactly 1 attempt through the old deadline).
async function countdownAndReconnectNow(page, link, attempts, acquires) {
  await live(page);
  const n0 = attempts.length;
  const tCut = await dropAndWaitLost(page, link, true);
  await waitFor(() => attempts.length >= n0 + 3, { timeoutMs: 10000, intervalMs: 50, label: 'three retries' });
  await sleep(300);
  const retryTxt = () => page.evaluate(() => { const e = document.getElementById('lease-retry-txt'); return { hidden: e.hidden, txt: e.textContent.trim(), btn: !document.getElementById('btn-reconnect-now').hidden }; });
  const a = await retryTxt();
  const m = /^(\d+) 秒后重试$/.exec(a.txt);
  assert(!a.hidden && m && a.btn, `countdown: ${JSON.stringify(a)}`);
  const clipped = await page.evaluate(() => [...document.querySelectorAll('#lease-status > *')].filter((e) => !e.hidden && e.scrollWidth > e.clientWidth + 1).map((e) => e.id));
  assert.deepEqual(clipped, [], 'lost status text clipped next to the countdown');
  await page.screenshot({ path: path.join(SHOT_DIR, 's3r2_countdown_zh.png') });
  await sleep(1200);
  const b = await retryTxt();
  const m2 = /^(\d+) 秒后重试$/.exec(b.txt);
  assert(m2 && Number(m2[1]) < Number(m[1]), `countdown did not decrease: ${a.txt} -> ${b.txt}`);
  const oldDueAt = await page.evaluate(() => wsReconnectDueAt);
  assert(oldDueAt > Date.now() + 2000, `queued retry deadline should be > 2 s ahead, got ${oldDueAt - Date.now()} ms`);
  link.outage = false;
  const n1 = attempts.length, tClick = Date.now();
  await page.click('#btn-reconnect-now');
  await waitFor(() => attempts.length > n1, { timeoutMs: 1000, intervalMs: 20, label: 'attempt after reconnect now' });
  await live(page, 3000);
  await page.waitForFunction(() => document.getElementById('btn-reconnect-now').hidden && document.getElementById('lease-retry-txt').hidden, null, { timeout: 2000 })
    .catch(() => { throw new Error('countdown and reconnect-now still shown after recovery'); });
  const waitPast = Math.max(0, oldDueAt + 500 - Date.now());
  if (waitPast > 0) await sleep(waitPast);
  assert.equal(attempts.length - n1, 1, `manual reconnect made ${attempts.length - n1} attempts through old retry deadline, want 1`);
  assert.equal(acquires(tCut).length, 0, `manual reconnect sent ${acquires(tCut).length} acquire_lease, want 0`);
  log(`PASS countdown ${a.txt} -> ${b.txt}; reconnect now: 1 attempt (${attempts[n1] - tClick} ms), 0 acquire_lease through old deadline`);
}

// Arm-state text of the release notice: pending torque off vs. a running park, and the actual
// rendered #lease-released notice when another real client holds the lease on reconnect.
async function armTextCases(page, port, link) {
  const r = await page.evaluate(() => {
    const o = { ...S.obs, watchdog_park_blocked_reason: null };
    return {
      pending: connArmText({ ...o, is_torqued: true, disable_pending: true, action: null }),
      parking: connArmText({ ...o, is_torqued: true, disable_pending: true, action: { kind: 'park', state: 'running' } }),
      pendingKey: t('conn.arm.disable_pending'), parkingKey: t('conn.arm.parking'),
    };
  });
  assert.equal(r.pending, r.pendingKey, 'disable_pending alone is a pending torque off');
  assert.equal(r.parking, r.parkingKey, 'a running park action is parking');

  // Hold the lease, drop the socket, let a second real WS client acquire the lease, then reconnect
  // and assert the rendered #lease-released notice (not just the i18n formatter).
  if (await page.evaluate(() => S.leaseGeneration == null)) {
    await page.evaluate(() => acquireLeaseDirect());
    await page.waitForFunction(() => S.leaseGeneration != null, null, { timeout: 5000 });
  }
  await dropAndWaitLost(page, link, true);
  const otherId = 'rebot-mcp-other';
  const otherWs = new WebSocket(`ws://127.0.0.1:${port}/api/v1/ws`);
  try {
    await new Promise((resolve, reject) => {
      otherWs.addEventListener('open', resolve, { once: true });
      otherWs.addEventListener('error', () => reject(new Error('other client WS open failed')), { once: true });
    });
    otherWs.send(JSON.stringify({ type: 'acquire_lease', seq: 1, client_id: otherId, force: true }));
    await waitFor(async () => (await getObs(port)).lease_holder === otherId, { timeoutMs: 5000, intervalMs: 50, label: 'other client holds lease' });
    link.outage = false;
    await live(page, 20000);
    await page.waitForFunction(() => !document.getElementById('lease-released').hidden, null, { timeout: 4000 });
    const rel = await releaseNotice(page);
    const obs = await getObs(port);
    const wantArm = !obs.is_torqued && obs.action?.kind === 'park' && obs.action?.state === 'done' ? PARKED_ZH : TORQUE_OFF_ZH;
    const ui = await page.evaluate(() => ({
      retakeDisabled: document.getElementById('btn-retake').disabled,
      lsCls: document.getElementById('lease-status').className,
    }));
    assert(!obs.is_torqued && !rel.hidden && rel.txt.includes(otherId) && rel.txt.includes(wantArm),
      `rendered other-holder release notice (want ${wantArm}): ${JSON.stringify(rel)}`);
    assert.equal(ui.retakeDisabled, true, '#btn-retake must be disabled while another client holds the lease');
    assert(ui.lsCls.includes('is-other'), `lease status class while other holds: ${ui.lsCls}`);
    log(`PASS arm text & rendered other-holder notice: pending "${r.pending}", parking "${r.parking}", rendered "${rel.txt}"`);
  } finally {
    otherWs.close();
    await waitFor(async () => (await getObs(port)).lease_holder === null, { timeoutMs: 5000, intervalMs: 50, label: 'other client lease released' }).catch(() => {});
  }
}

// 9. Phone expert layout: header badge shows the warning AND #btn-reconnect-now is exposed and usable
//    via the same handler (not trapped inside hidden #ex-sline).
async function phoneExpert(browser, port) {
  const cp = await openCockpit(browser, port, { width: 400, height: 860 });
  try {
    const { page, link, attempts } = cp;
    await page.goto(`http://127.0.0.1:${port}/?ui=expert`, { waitUntil: 'domcontentloaded' });
    await live(page);
    assert.equal(await page.locator('#lease-status').boundingBox(), null, 'connected phone expert keeps #lease-status hidden');
    const n0 = attempts.length;
    const tCut = await dropAndWaitLost(page, link, true);
    await waitFor(() => attempts.length >= n0 + 2, { timeoutMs: 8000, intervalMs: 50, label: 'phone retries' });
    // the badge is narrow: "连接已断开 · {s} 秒后重试" while a retry is queued, else the full warning
    await page.waitForFunction((t) => (document.getElementById('ex-ph-status')?.textContent || '').startsWith(t), LOST_T_ZH, { timeout: 2000 }).catch(() => {});
    const ph = await page.evaluate(() => { const e = document.getElementById('ex-ph-status'); const r = e.getBoundingClientRect(); return { txt: e.textContent.trim(), cls: e.className, visible: r.width > 0 && r.height > 0 && getComputedStyle(e).display !== 'none' }; });
    assert(ph.visible && ph.cls.includes('is-danger') && (ph.txt === LOST_ZH || /^连接已断开 · \d+ 秒后重试$/.test(ph.txt)), `phone expert badge: ${JSON.stringify(ph)}`);
    const fits = await page.evaluate(() => { const e = document.getElementById('ex-ph-status'); return e.scrollWidth <= e.clientWidth + 1; });
    assert(fits, `phone expert badge text is clipped: ${ph.txt}`);
    const btnBox = await page.locator('#btn-reconnect-now').boundingBox();
    assert(btnBox && btnBox.width > 0 && btnBox.height > 0, `phone expert #btn-reconnect-now must be visible, got ${JSON.stringify(btnBox)}`);
    const clipped = await page.evaluate(() => [...document.querySelectorAll('#lease-status > *')].filter((e) => !e.hidden && e.scrollWidth > e.clientWidth + 1).map((e) => e.id));
    assert.deepEqual(clipped, [], 'phone expert lost status text clipped');
    await page.screenshot({ path: path.join(SHOT_DIR, 's3r2_phone_expert_lost_zh.png') });
    link.outage = false;
    const n1 = attempts.length, tClick = Date.now();
    await page.click('#btn-reconnect-now');
    await waitFor(() => attempts.length > n1, { timeoutMs: 1000, intervalMs: 20, label: 'phone reconnect now attempt' });
    await live(page, 3000);
    assert.equal(cp.acquires(tCut).length, 0, 'phone reconnect now must send 0 acquire_lease');
    await waitFor(async () => (await page.locator('#lease-status').boundingBox()) === null, { timeoutMs: 2000, intervalMs: 50, label: 'phone #lease-status hidden after recovery' });
    assert.deepEqual(cp.pageErrors, [], 'phone page errors');
    log(`PASS phone expert: header badge "${ph.txt}" (${ph.cls}), #btn-reconnect-now clicked (${attempts[n1] - tClick} ms)`);
  } finally {
    await cp.ctx.close().catch(() => {});
  }
}

main().catch((e) => { console.error(e); process.exit(1); });
