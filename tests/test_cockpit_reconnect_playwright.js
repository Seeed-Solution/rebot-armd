// Cockpit connection-loss warning and reconnect (Playwright against a mock armd).
//
// A routed WebSocket proxy between page and armd simulates an outage: open sockets are closed and
// new ones are closed on arrival while armd keeps running (its lease watchdog parks the arm). Checks:
// - the lease status element shows "连接已断开 · 正在重连" / "Connection lost · reconnecting" for the whole outage;
// - one retry timer: attempts after 1, 2, 4, 8 s, then every 15 s; reset by the first valid observation;
// - recovery is read-only (0 acquire_lease) and shows the release notice with the arm state taken from
//   fresh telemetry ("parked" only when armd reports a finished park action and torque off; the lease
//   watchdog park is not an action in the observation, so after it the notice says "torque off");
// - "Take control" sends exactly one acquire_lease.
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
const LOST_ZH = '连接已断开 · 正在重连', LOST_EN = 'Connection lost · reconnecting';
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
  child.stdout.on('data', (c) => logs.push(String(c)));
  child.stderr.on('data', (c) => logs.push(String(c)));
  await waitFor(async () => {
    if (child.exitCode !== null) throw new Error(`armd exited early:\n${logs.join('')}`);
    try { const o = await getObs(port); return Array.isArray(o.positions) && o.positions.length > 0; } catch (_) { return false; }
  }, { timeoutMs: 15000, intervalMs: 100, label: 'armd ready' });
  const stop = () => new Promise((resolve) => {
    if (child.exitCode !== null || child.signalCode !== null) { resolve(); return; }
    const kt = setTimeout(() => child.kill('SIGKILL'), 3000);
    child.once('exit', () => { clearTimeout(kt); fs.rmSync(state, { recursive: true, force: true }); resolve(); });
    child.kill('SIGTERM');
  });
  return { stop };
}

// Page + routed WebSocket proxy. `net.outage = true` closes the open socket pair and every new one.
async function openCockpit(browser, port) {
  const ctx = await browser.newContext({ viewport: { width: 1280, height: 800 }, locale: 'zh-CN' });
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
  const sent = [], attempts = [], link = { outage: false, pair: null };
  await page.routeWebSocket(/.*/, (ws) => {
    const u = new URL(ws.url());
    if (u.hostname !== '127.0.0.1' || Number(u.port) !== port || u.pathname !== '/api/v1/ws') { ws.close(); return; }
    attempts.push(Date.now());
    if (link.outage) { ws.close({ code: 4000, reason: 'outage' }); return; }
    const server = ws.connectToServer();
    const pair = { ws, server };
    link.pair = pair;
    ws.onMessage((raw) => { let j = null; try { j = JSON.parse(String(raw)); } catch (_) {} sent.push({ ts: Date.now(), json: j }); server.send(raw); });
    server.onMessage((raw) => ws.send(raw));
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

async function main() {
  fs.mkdirSync(SHOT_DIR, { recursive: true });
  const port = await freePort();
  const armd = await startArmd(port);
  const browser = await chromium.launch({ executablePath: process.env.CHROME || undefined, headless: true, args: ['--no-sandbox', '--disable-setuid-sandbox'] });
  const cp = await openCockpit(browser, port);
  const { page, link, attempts } = cp;
  try {
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

    assert.deepEqual(cp.pageErrors, [], 'page errors');
    const errors = cp.consoleLines.filter((l) => l.startsWith('error:'));
    fs.writeFileSync(path.join(SHOT_DIR, 's3_console.log'), cp.consoleLines.join('\n') + '\n');
    assert.deepEqual(errors, [], 'console errors');
    log(`ALL RECONNECT SCENARIOS PASSED (shots and console log in ${SHOT_DIR})`);
  } finally {
    await cp.ctx.close().catch(() => {});
    await browser.close().catch(() => {});
    await armd.stop();
  }
}

main().catch((e) => { console.error(e); process.exit(1); });
