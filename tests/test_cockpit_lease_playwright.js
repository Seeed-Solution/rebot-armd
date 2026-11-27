#!/usr/bin/env node
'use strict';

/**
 * Playwright E2E verification for the cockpit explicit-lease & safety contract.
 * Spawns isolated `./target/debug/armd` instances ONLY on allowlisted ports
 * 18877, 18878, 18879 (never 18765, 18001, 18990, 8001, or 8090).
 */

const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const fs = require('node:fs');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');

let playwright;
try { playwright = require('playwright'); }
catch (_) { playwright = require('/usr/local/lib/node_modules/@playwright/cli/node_modules/playwright'); }
const { chromium } = playwright;

const ROOT = path.resolve(__dirname, '..');
const ARMD_BIN = process.env.ARMD_BIN || path.join(ROOT, 'target', 'debug', 'armd');
const B601_CONFIG = path.join(ROOT, 'kits', 'recamera-b601dm', 'profile', 'rebot-b601dm.json');
const SHOT_DIR = process.env.SHOT_DIR || path.join(os.tmpdir(), 'armd-cockpit-lease-shots');

const PORT_MAIN = 18877, PORT_AUTH = 18878, PORT_INTERACT = 18879;
const ALLOWED_PORTS = new Set([PORT_MAIN, PORT_AUTH, PORT_INTERACT]);
const FORBIDDEN_PORTS = new Set([18765, 18001, 18990, 8001, 8090]);

const MOTION_ACTUATOR_PATHS = new Set([
  '/api/v1/action/move_to', '/api/v1/motion/servo', '/api/v1/action/run_sequence',
  '/api/v1/motion/jog', '/api/v1/motion/home', '/api/v1/action/move_to_pose',
  '/api/v1/action/replay', '/api/v1/action/gripper', '/api/v1/action/move_relative',
  '/api/v1/torque', '/api/v1/mode/gravity_comp',
]);
const STOP_TOAST_RE = /Stopped, holding position|已停止，位置保持|Grasp stopped|已停止抓取|Stop not confirmed|停止未确认|cannot stop|无法停止|no control|没有控制权|Stop was not sent|未发送停止/i;
const MAL_GRANT_TOAST_RE = /Control grant was malformed|Incomplete control response|控制权确认异常|控制权响应不完整/i;
const TIMEOUT_TOAST_RE = /Control request timed out|获取\/接管控制权超时/i;

const activeProcs = new Set();
const activeClients = new Set();
const tempStateDir = fs.mkdtempSync(path.join(os.tmpdir(), 'armd-lease-test-'));
const tempDirs = [tempStateDir];
let browser = null, shuttingDown = false, cleanupPromise = null;

const log = (msg) => console.log(`[${new Date().toISOString().slice(11, 23)}] ${msg}`);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const waitFn = (page, fn, arg = null, timeout = 4000) => page.waitForFunction(fn, arg, { timeout });
const waitHidden = (page, sel, timeout = 2000) => waitFn(page, (s) => document.querySelector(s)?.hidden === true, sel, timeout);
const waitShown = (page, sel, timeout = 2000) => page.waitForSelector(`${sel}:not([hidden])`, { timeout });

function assertPortAllowed(port) {
  assert(ALLOWED_PORTS.has(port) && !FORBIDDEN_PORTS.has(port), `Port ${port} is not in ALLOWED_PORTS`);
}

function isMotionOrActuatorFrame(f) {
  if (!f || !f.json) return false;
  const { type, path: p, body } = f.json;
  if (type === 'servo' || type === 'preset' || type === 'cartesian_joint_stream') return true;
  if (type === 'command') {
    return MOTION_ACTUATOR_PATHS.has(p) || (p === '/api/v1/action/grasp' && !(body && body.dry_run === true));
  }
  return false;
}

async function waitFor(fn, { timeoutMs = 10000, intervalMs = 35, label = 'condition' } = {}) {
  const deadline = Date.now() + timeoutMs;
  let lastErr = null;
  while (Date.now() < deadline) {
    try { const v = await fn(); if (v) return v; } catch (e) { lastErr = e; }
    await sleep(intervalMs);
  }
  throw new Error(`Timed out after ${timeoutMs} ms waiting for ${label}${lastErr ? ` (${lastErr.message})` : ''}`);
}

function assertPortFree(port) {
  assertPortAllowed(port);
  return new Promise((resolve, reject) => {
    const srv = net.createServer();
    srv.once('error', (err) => reject(new Error(`Port ${port} is not free: ${err.message}`)));
    srv.once('listening', () => srv.close(resolve));
    srv.listen(port, '127.0.0.1');
  });
}

async function getObs(port) {
  assertPortAllowed(port);
  const res = await fetch(`http://127.0.0.1:${port}/api/v1/observation`, { cache: 'no-store' });
  if (!res.ok) throw new Error(`GET /api/v1/observation HTTP ${res.status}`);
  return res.json();
}

async function unlockOperatorToken(port, pin) {
  assertPortAllowed(port);
  const res = await fetch(`http://127.0.0.1:${port}/api/v1/auth/unlock`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ pin }),
  });
  const body = await res.json().catch(() => ({}));
  if (!res.ok || !body.token) throw new Error(`unlock failed (${res.status}): ${JSON.stringify(body)}`);
  return body.token;
}

async function allocFreePort() {
  for (let i = 0; i < 16; i++) {
    const port = await new Promise((resolve, reject) => {
      const srv = net.createServer();
      srv.once('error', reject);
      srv.once('listening', () => {
        const p = srv.address().port;
        srv.close(() => resolve(p));
      });
      srv.listen(0, '127.0.0.1');
    });
    if (!FORBIDDEN_PORTS.has(port)) {
      ALLOWED_PORTS.add(port);
      return port;
    }
  }
  throw new Error('Could not allocate a non-forbidden free port');
}

async function spawnArmd(port, extraArgs = [], { extraEnv = {}, waitOwnerStatus = false } = {}) {
  assertPortAllowed(port);
  if (shuttingDown) throw new Error('spawnArmd aborted: shutting down');
  await assertPortFree(port);
  assert(fs.existsSync(ARMD_BIN), `armd binary not found at ${ARMD_BIN}`);
  const args = ['--follower', 'mock', '--port', String(port), '--log-dir', 'off', '--blackbox-dir', 'off', '--episodes-dir', 'off', ...extraArgs];
  const logs = [];
  const child = spawn(ARMD_BIN, args, {
    cwd: ROOT, stdio: ['ignore', 'pipe', 'pipe'],
    env: {
      ...process.env, RUST_LOG: process.env.RUST_LOG || 'warn',
      ARMD_APP_CONFIG: 'off',
      ARMD_POSES: path.join(tempStateDir, `poses_${port}.json`),
      ARMD_SEQUENCES: path.join(tempStateDir, `sequences_${port}.json`),
      ...extraEnv,
    },
  });
  const recLog = (c) => { logs.push(c.toString('utf8')); if (logs.length > 120) logs.shift(); };
  child.stdout.on('data', recLog);
  child.stderr.on('data', recLog);
  activeProcs.add(child);
  child.once('exit', () => activeProcs.delete(child));
  if (shuttingDown) { await killArmd(child); throw new Error('spawnArmd aborted: shutting down'); }
  await waitFor(async () => {
    if (shuttingDown) throw new Error('shutting down');
    if (child.exitCode !== null) throw new Error(`armd exited early (${child.exitCode}):\n${logs.join('')}`);
    if (waitOwnerStatus) {
      const res = await fetch(`http://127.0.0.1:${port}/api/v1/owner/status`, { cache: 'no-store' });
      if (!res.ok) return null;
      const st = await res.json();
      return st && st.effective_owner ? st : null;
    }
    const o = await getObs(port);
    return Array.isArray(o?.positions) && o.positions.length > 0 ? o : null;
  }, { timeoutMs: 15000, intervalMs: 100, label: `armd ready on ${port}` });
  return { child, port, logs };
}

async function killArmd(proc) {
  if (!proc) return;
  const child = proc.child || proc;
  if (child.exitCode !== null || child.signalCode !== null) { activeProcs.delete(child); return; }
  await new Promise((resolve) => {
    let done = false, kt = null, ht = null;
    const finish = () => { if (!done) { done = true; clearTimeout(kt); clearTimeout(ht); activeProcs.delete(child); resolve(); } };
    child.once('exit', finish);
    try { child.kill('SIGTERM'); } catch (_) { finish(); return; }
    kt = setTimeout(() => { if (!done) { try { child.kill('SIGKILL'); } catch (_) { finish(); return; } ht = setTimeout(finish, 4000); } }, 2000);
  });
}

function cleanup() {
  if (cleanupPromise) return cleanupPromise;
  shuttingDown = true;
  cleanupPromise = (async () => {
    for (const c of [...activeClients]) await c.close().catch(() => {});
    if (browser) { const b = browser; browser = null; await b.close().catch(() => {}); }
    for (const p of [...activeProcs]) await killArmd(p).catch(() => {});
    for (const d of tempDirs) { try { fs.rmSync(d, { recursive: true, force: true }); } catch (_) {} }
  })();
  return cleanupPromise;
}

class ExternalWsClient {
  constructor(port, clientId) {
    assertPortAllowed(port);
    Object.assign(this, { port, clientId, ws: null, obs: null, epoch: null, generation: null, hbTimer: null, seq: 0, reqId: 0, waiters: [] });
    activeClients.add(this);
  }
  async connect() {
    this.ws = new WebSocket(`ws://127.0.0.1:${this.port}/api/v1/ws`);
    await new Promise((res, rej) => {
      const done = (fn, v) => { this.ws.removeEventListener('open', onO); this.ws.removeEventListener('error', onE); fn(v); };
      const onO = () => done(res), onE = (e) => done(rej, new Error(`WS connect failed: ${e.message || e}`));
      this.ws.addEventListener('open', onO); this.ws.addEventListener('error', onE);
    });
    this.ws.addEventListener('message', (ev) => {
      let m; try { m = JSON.parse(String(ev.data)); } catch (_) { return; }
      if (Array.isArray(m.positions)) { this.obs = m; if (typeof m.epoch_id === 'number') this.epoch = m.epoch_id; }
      if (m.type === 'lease_lost' && (m.generation == null || m.generation === this.generation)) { this.stopHeartbeat(); this.generation = null; }
      for (let i = this.waiters.length - 1; i >= 0; i--) if (this.waiters[i].pred(m)) { const w = this.waiters.splice(i, 1)[0]; clearTimeout(w.timer); w.resolve(m); }
    });
    this.ws.addEventListener('close', () => { this.stopHeartbeat(); this.generation = null; activeClients.delete(this); });
    await waitFor(() => this.obs && this.epoch != null, { timeoutMs: 5000, intervalMs: 25, label: `ext(${this.clientId}) obs` });
    return this;
  }
  waitMsg(pred, timeoutMs = 4000, label = 'ws message') {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => { const i = this.waiters.findIndex((w) => w.timer === timer); if (i >= 0) this.waiters.splice(i, 1); reject(new Error(`ext(${this.clientId}) timeout: ${label}`)); }, timeoutMs);
      this.waiters.push({ pred, resolve, reject, timer });
    });
  }
  send(obj) { assert(this.ws?.readyState === WebSocket.OPEN, `ext(${this.clientId}) not open`); this.ws.send(JSON.stringify(obj)); }
  async auth(token) {
    const seq = ++this.seq, p = this.waitMsg((m) => m.type === 'auth_ack' && m.seq === seq, 3000, 'auth_ack');
    this.send({ type: 'auth', seq, token });
    const ack = await p; assert.equal(ack.role, 'operator', `Expected operator auth_ack, got ${JSON.stringify(ack)}`); return ack;
  }
  sendHeartbeatNow() { if (this.ws?.readyState === WebSocket.OPEN && this.generation != null && this.epoch != null) this.send({ type: 'heartbeat', client_id: this.clientId, generation: this.generation, epoch_id: this.epoch }); }
  startHeartbeat() { this.stopHeartbeat(); this.hbTimer = setInterval(() => this.sendHeartbeatNow(), 150); }
  stopHeartbeat() { if (this.hbTimer) { clearInterval(this.hbTimer); this.hbTimer = null; } }
  async sendAcquireRaw({ force = false } = {}) {
    const seq = ++this.seq, p = this.waitMsg((m) => (m.type === 'lease_granted' || m.type === 'lease_rejected') && m.seq === seq, 4000, 'acquire_lease');
    this.send({ type: 'acquire_lease', seq, client_id: this.clientId, force });
    return p;
  }
  async acquireLease({ force = false, heartbeat = true } = {}) {
    const r = await this.sendAcquireRaw({ force });
    if (r.type !== 'lease_granted') throw new Error(`ext(${this.clientId}) rejected: ${JSON.stringify(r)}`);
    this.generation = r.generation; if (heartbeat) this.startHeartbeat(); return r;
  }
  async releaseLease() {
    this.stopHeartbeat(); const gen = this.generation;
    if (gen == null) return { type: 'lease_released', success: false };
    const p = this.waitMsg((m) => m.type === 'lease_released', 3000, 'lease_released');
    this.send({ type: 'release_lease', client_id: this.clientId, generation: gen }); this.generation = null; return p;
  }
  async command(cmdPath, body = {}) {
    assert(this.generation != null, `ext(${this.clientId}) needs lease for ${cmdPath}`);
    const request_id = ++this.reqId, p = this.waitMsg((m) => m.type === 'command_ack' && m.request_id === request_id, 5000, cmdPath);
    this.send({ type: 'command', request_id, path: cmdPath, body, client_id: this.clientId, generation: this.generation, epoch_id: this.epoch });
    const ack = await p;
    if (ack.ok === false || (typeof ack.status === 'number' && ack.status >= 400)) throw new Error(`${cmdPath} failed: ${JSON.stringify(ack)}`);
    if (typeof ack.epoch_id === 'number') this.epoch = ack.epoch_id;
    return ack;
  }
  eStop() { this.send({ type: 'e_stop', client_id: this.clientId }); }
  async close() {
    this.stopHeartbeat();
    if (!this.ws || this.ws.readyState === WebSocket.CLOSED) { activeClients.delete(this); return; }
    await new Promise((r) => { const t = setTimeout(r, 1500); this.ws.addEventListener('close', () => { clearTimeout(t); r(); }, { once: true }); try { this.ws.close(); } catch (_) { clearTimeout(t); r(); } });
    activeClients.delete(this);
  }
}

async function withExtClient(port, id, fn) {
  const ext = await new ExternalWsClient(port, id).connect();
  try { return await fn(ext); } finally { await ext.close(); }
}

function attachPageGuards(page) {
  const dialogs = [], pageErrors = [];
  page.on('dialog', async (d) => { dialogs.push(`${d.type()}: ${d.message()}`); await d.dismiss().catch(() => {}); });
  page.on('pageerror', (e) => pageErrors.push(e.message || String(e)));
  return {
    dialogs, pageErrors,
    assertClean(ctx) {
      assert.equal(dialogs.length, 0, `${ctx}: forbidden dialog(s): ${JSON.stringify(dialogs)}`);
      assert.equal(pageErrors.length, 0, `${ctx}: pageerror(s): ${JSON.stringify(pageErrors)}`);
    },
  };
}

async function newCockpit(browserInst, port, { width = 1280, height = 800, locale = 'en-US' } = {}) {
  assertPortAllowed(port);
  const ctx = await browserInst.newContext({ viewport: { width, height }, locale });
  const page = await ctx.newPage();
  const guards = attachPageGuards(page);
  const sent = [], received = [], hooks = { onClientMsg: null, onServerMsg: null };
  let wsCount = 0, currentPair = null;

  const pushFrame = (arr, wsIdx, raw) => {
    const str = String(raw); let json = null; try { json = JSON.parse(str); } catch (_) {}
    arr.push({ ts: Date.now(), wsIdx, raw: str, json });
    return { str, json };
  };

  // Register strict allowlist route FIRST so test-specific routes registered later run first (LIFO)
  await page.route('**/*', (route) => {
    let u; try { u = new URL(route.request().url()); } catch (_) { return route.abort('blockedbyclient'); }
    if (u.protocol === 'blob:' || u.protocol === 'data:' || u.protocol === 'about:') return route.continue();
    if ((u.protocol !== 'http:' && u.protocol !== 'https:') || u.hostname !== '127.0.0.1' || Number(u.port) !== port) return route.abort('blockedbyclient');
    if (u.pathname.startsWith('/api/v1/detection/') || u.pathname.startsWith('/api/v1/vision/')) {
      return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ enabled: false, mode: 'off', available: {} }) });
    }
    return route.continue();
  });

  // Strict WS allowlist: ONLY forward ws://127.0.0.1:<port>/api/v1/ws; immediately close detector/video/foreign WS
  await page.routeWebSocket(/.*/, (ws) => {
    let u; try { u = new URL(ws.url()); } catch (_) { ws.close(); return; }
    if (u.hostname !== '127.0.0.1' || Number(u.port) !== port || u.pathname !== '/api/v1/ws') { ws.close(); return; }
    const wsIdx = ++wsCount, server = ws.connectToServer(), pair = { ws, server, wsIdx };
    currentPair = pair;
    ws.onMessage((raw) => {
      const { str, json } = pushFrame(sent, wsIdx, raw);
      if (hooks.onClientMsg && hooks.onClientMsg(json, str, pair) === 'drop') return;
      server.send(str);
    });
    server.onMessage((raw) => {
      const { str, json } = pushFrame(received, wsIdx, raw);
      if (hooks.onServerMsg) {
        const res = hooks.onServerMsg(json, str, pair);
        if (res === 'drop') return;
        if (res !== 'forward' && typeof res === 'string') { ws.send(res); return; }
      }
      ws.send(str);
    });
  });

  const rec = {
    sent, received, hooks, get wsCount() { return wsCount; }, get current() { return currentPair; },
    mark: () => sent.length, markRecv: () => received.length,
    byType: (type, i = 0) => sent.slice(i).filter((f) => f.json?.type === type),
    commands: (p, i = 0) => sent.slice(i).filter((f) => f.json?.type === 'command' && f.json?.path === p),
    motionFrames: (i = 0) => sent.slice(i).filter(isMotionOrActuatorFrame),
    onWs: (wsIdx, types) => sent.filter((f) => f.wsIdx === wsIdx && f.json && types.includes(f.json.type)),
    ackFor: (reqId, recvIdx = 0) => received.slice(recvIdx).find((f) => f.json?.type === 'command_ack' && f.json?.request_id === reqId),
  };
  return { ctx, page, guards, rec };
}

const waitForCockpitLive = (page, timeout = 15000) =>
  waitFn(page, () => typeof isLive === 'function' && isLive() && S.wsObsSeen === true && S.obs && S.dof > 0 && S.epoch != null, null, timeout);

async function waitCommandAck(rec, cmdPath, markIdx, recvIdx, label = cmdPath, timeoutMs = 3000) {
  const req = await waitFor(() => rec.commands(cmdPath, markIdx).at(-1), { timeoutMs, label: `${label} outbound` });
  const ack = await waitFor(() => rec.ackFor(req.json.request_id, recvIdx), { timeoutMs, label: `${label} command_ack` });
  return { req: req.json, ack: ack.json };
}

async function holdButton(page, sel, holdMs = 750) {
  await waitFn(page, (s) => { const e = document.querySelector(s); return !!e && !e.hidden && !e.disabled; }, sel, 5000);
  const b = await (await page.waitForSelector(sel, { state: 'visible', timeout: 5000 })).boundingBox();
  assert(b && b.width > 0 && b.height > 0, `${sel} has no bounding box`);
  await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2);
  await page.mouse.down(); await page.waitForTimeout(holdMs); await page.mouse.up();
}

async function assertReachable(page, sel, label) {
  const info = await page.evaluate((s) => {
    const el = document.querySelector(s); if (!el) return { found: false };
    const r = el.getBoundingClientRect(), hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
    return { found: true, hidden: el.hidden, rect: { left: r.left, top: r.top, right: r.right, bottom: r.bottom, width: r.width, height: r.height }, vw: window.innerWidth, vh: window.innerHeight, hitMatches: !!hit && (hit === el || el.contains(hit)), hitDesc: hit ? `${hit.tagName}#${hit.id}.${hit.className}` : null };
  }, sel);
  assert(info.found && !info.hidden && info.rect.width > 0 && info.rect.height > 0, `${label}: ${sel} not visible`);
  assert(info.rect.left >= 0 && info.rect.top >= 0 && info.rect.right <= info.vw && info.rect.bottom <= info.vh, `${label}: ${sel} outside viewport`);
  assert(info.hitMatches, `${label}: elementFromPoint missed ${sel}, hit ${info.hitDesc}`);
}

async function assertControlInputsState(page, expectedState, label) {
  assert(expectedState === 'unavailable' || expectedState === 'active', `${label}: invalid expectedState=${expectedState}`);
  // The render loop (frame()) applies the control-input classes; let two frames pass under host load.
  await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
  const errors = await page.evaluate((exp) => {
    const errs = [];
    const wantActive = exp === 'active';
    const hasCtrl = typeof pageHasControl === 'function' ? pageHasControl() : null;
    if (hasCtrl !== wantActive) errs.push(`pageHasControl()=${hasCtrl} (expected ${wantActive})`);
    const bodyUnavail = document.body.classList.contains('controls-unavailable');
    if (bodyUnavail !== !wantActive) errs.push(`body.controls-unavailable=${bodyUnavail} (expected ${!wantActive})`);

    const coreSelectors = [
      '#pad-sxy', '#pad-srot', '#pad-sz', '#pad-sgrip',
      'button.preset[data-preset="home"]', 'button.preset[data-preset="inspect"]', 'button.preset[data-preset="park"]',
      '#btn-home', '#btn-pose', '#grip-range',
      '#pad-left', '#pad-wrist', '#pad-right', '#pad-cxy', '#pad-cz', '#pad-cp', '#btn-gc',
    ];
    for (const sel of coreSelectors) {
      const el = document.querySelector(sel);
      if (!el) { errs.push(`missing required element ${sel}`); continue; }
      if (el.dataset.controlState !== exp) errs.push(`${sel} controlState=${JSON.stringify(el.dataset.controlState)} (expected ${exp})`);
    }

    const jbtns = [...document.querySelectorAll('.jbtn')];
    if (jbtns.length < 1) {
      errs.push(`expected >= 1 .jbtn elements, found ${jbtns.length}`);
    } else {
      jbtns.forEach((el, idx) => {
        if (el.dataset.controlState !== exp) errs.push(`.jbtn[${idx}] controlState=${JSON.stringify(el.dataset.controlState)} (expected ${exp})`);
      });
    }

    const isVisible = (el) => {
      if (!el) return false;
      const cs = getComputedStyle(el);
      const r = el.getBoundingClientRect();
      return cs.display !== 'none' && cs.visibility !== 'hidden' && r.width > 0 && r.height > 0;
    };
    const checkOpacity = (el, desc) => {
      const opacity = parseFloat(getComputedStyle(el).opacity);
      if (exp === 'unavailable' && !(opacity < 0.75)) errs.push(`${desc} opacity=${opacity} (expected < 0.75)`);
      else if (exp === 'active' && !(opacity > 0.95)) errs.push(`${desc} opacity=${opacity} (expected > 0.95)`);
    };
    const checkKnobBg = (pad, desc) => {
      const knob = pad.matches('.disc, .pad') ? pad.querySelector('.knob') : pad.querySelector('.disc .knob, .pad .knob');
      if (!knob || !isVisible(knob)) { errs.push(`${desc} visible without visible .knob`); return false; }
      const knobBg = getComputedStyle(knob).backgroundColor;
      const m = /^rgba?\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)/i.exec(knobBg);
      if (!m) { errs.push(`${desc} .knob unparseable knobBg=${JSON.stringify(knobBg)}`); return true; }
      const r = Number(m[1]), g = Number(m[2]), b = Number(m[3]);
      const isGreen = (g - r >= 25 && g - b >= 60);
      if (exp === 'unavailable' && isGreen) errs.push(`${desc} .knob knobBg=${knobBg} is green (expected non-green/grey)`);
      else if (exp === 'active' && !isGreen) errs.push(`${desc} .knob knobBg=${knobBg} is not green (expected g-r>=25 && g-b>=60)`);
      return true;
    };

    const poseGos = [...document.querySelectorAll('.pose-go')];
    poseGos.forEach((el, idx) => {
      if (el.dataset.controlState !== exp) errs.push(`.pose-go[${idx}] controlState=${JSON.stringify(el.dataset.controlState)} (expected ${exp})`);
      if (isVisible(el)) checkOpacity(el, `.pose-go[${idx}]`);
    });

    const isSimple = document.body.classList.contains('ui-simple');
    const isExpert = document.body.classList.contains('ui-expert');

    const simpleSelectors = [
      '#pad-sxy', '#pad-srot', '#pad-sz', '#pad-sgrip',
      'button.preset[data-preset="home"]', 'button.preset[data-preset="inspect"]', 'button.preset[data-preset="park"]',
    ];
    let simpleChecked = 0;
    for (const sel of simpleSelectors) {
      const el = document.querySelector(sel);
      if (!el || !isVisible(el)) continue;
      simpleChecked++;
      checkOpacity(el, sel);
    }
    if (isSimple && simpleChecked < 6) errs.push(`expected >= 6 visible simple controls, checked ${simpleChecked}`);

    let knobsChecked = 0;
    for (const sel of ['#pad-sxy', '#pad-srot', '#pad-sz', '#pad-sgrip']) {
      const pad = document.querySelector(sel);
      if (!pad || !isVisible(pad)) continue;
      if (checkKnobBg(pad, sel)) knobsChecked++;
    }
    if (isSimple && knobsChecked < 3) errs.push(`expected >= 3 visible pad knobs, checked ${knobsChecked}`);

    if (isExpert) {
      const expertEls = [...new Set([...document.querySelectorAll('.pad, .grip-slider, #btn-home, .jbtn')])];
      let expertChecked = 0, expertKnobsChecked = 0;
      for (const el of expertEls) {
        if (!isVisible(el)) continue;
        const desc = el.id ? `#${el.id}` : `.${[...el.classList].join('.')}`;
        expertChecked++;
        checkOpacity(el, desc);
        if (el.classList.contains('pad') && checkKnobBg(el, desc)) expertKnobsChecked++;
      }
      if (expertChecked < 2) errs.push(`expected >= 2 visible expert controls, checked ${expertChecked}`);
      const isExpertStick = document.body.classList.contains('mode-stick') || document.body.classList.contains('input-stick');
      if (isExpertStick && expertKnobsChecked < 1) errs.push(`expected >= 1 visible expert .pad .knob, checked ${expertKnobsChecked}`);
    }

    return errs;
  }, expectedState);
  assert.equal(errors.length, 0, `${label} (${expectedState}) control inputs check failed:\n  - ${errors.join('\n  - ')}`);
}

const clearToasts = (page) => page.evaluate(() => { const tc = document.getElementById('toasts'); if (tc) tc.textContent = ''; });
const getToasts = (page) => page.evaluate(() => ({ all: [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent).join(' | '), warns: [...document.querySelectorAll('#toasts .toast.warn')].map((e) => e.textContent).join(' | '), count: document.querySelectorAll('#toasts .toast').length }));
const startToastWatch = (page) => page.evaluate(() => {
  const tc = document.getElementById('toasts'); if (tc) tc.textContent = '';
  if (window.__tmo) window.__tmo.disconnect();
  window.__tTexts = [];
  window.__tmo = new MutationObserver((rs) => { for (const r of rs) for (const n of r.addedNodes) if (n.nodeType === 1 && n.classList.contains('toast')) window.__tTexts.push(n.textContent || ''); });
  if (tc) window.__tmo.observe(tc, { childList: true });
});
const stopToastWatch = (page) => page.evaluate(() => { if (window.__tmo) { window.__tmo.disconnect(); window.__tmo = null; } return window.__tTexts || []; });

async function dragSlider(page, box, fromFrac, toFrac) {
  const y = box.y + box.height / 2;
  await page.mouse.move(box.x + box.width * fromFrac, y); await page.mouse.down();
  await page.mouse.move(box.x + box.width * toFrac, y, { steps: 6 }); await page.mouse.up();
}

async function holdKeyRepeat(page, key, repeats, afterDownFn) {
  await page.keyboard.down(key); if (afterDownFn) await afterDownFn();
  for (let i = 0; i < repeats; i++) await page.keyboard.down(key);
  await page.keyboard.up(key);
}

async function holdEscapeNoStop(page, rec, modalSel, { expectInitialBump = false, label = modalSel } = {}) {
  const m = rec.mark();
  const ep0 = await page.evaluate(() => { GR.stopAt = 0; const tc = document.getElementById('toasts'); if (tc) tc.textContent = ''; return S.controlEpoch; });
  await page.keyboard.down('Escape');
  await waitHidden(page, modalSel, 2000);
  const epAfterDismiss = await page.evaluate(() => S.controlEpoch);
  if (!expectInitialBump) assert.equal(epAfterDismiss, ep0, `${label}: initial Escape bumped S.controlEpoch`);
  for (let i = 0; i < 3; i++) await page.keyboard.down('Escape');
  await page.keyboard.up('Escape');
  await page.waitForTimeout(180);
  const after = await page.evaluate(() => ({ stopAt: GR.stopAt, controlEpoch: S.controlEpoch, toasts: [...document.querySelectorAll('#toasts .toast')].map((e) => e.textContent).join(' | ') }));
  assert.equal(rec.commands('/api/v1/motion/stop', m).length, 0, `${label}: held Escape sent /api/v1/motion/stop`);
  assert.equal(rec.byType('acquire_lease', m).length, 0, `${label}: held Escape sent acquire_lease`);
  assert.equal(after.stopAt, 0, `${label}: held Escape invoked graspStop()`);
  assert.equal(after.controlEpoch, epAfterDismiss, `${label}: Escape repeats bumped S.controlEpoch (${epAfterDismiss} -> ${after.controlEpoch})`);
  assert(!STOP_TOAST_RE.test(after.toasts), `${label}: held Escape showed stop toast: "${after.toasts}"`);
}

async function assertReadOnlyReconnect(page, rec, wsBefore, { port, waitAfterMs = 1600, checkHeartbeat = false, timeoutMs = 8000, label = 'reconnect' } = {}) {
  await waitFor(() => rec.wsCount > wsBefore, { timeoutMs, label });
  const wsIdx = rec.wsCount;
  await waitForCockpitLive(page, 6000);
  await page.waitForTimeout(waitAfterMs);
  const types = checkHeartbeat ? ['acquire_lease', 'heartbeat'] : ['acquire_lease'];
  assert.equal(rec.onWs(wsIdx, types).length, 0, `${label}: sent ${types.join('/')} on replacement WS`);
  assert.equal(await page.evaluate(() => S.leaseGeneration), null, `${label}: S.leaseGeneration must be null`);
  if (port != null) assert.equal((await getObs(port)).lease_holder, null, `${label}: server lease_holder must be null`);
  return wsIdx;
}

async function holdSimultaneousConfirm(page, rec, confirmSel, modalSel, { baseHook = null, testSpaceEstop = false, expectForce = false, label = modalSel } = {}) {
  let heldGrant = null;
  rec.hooks.onServerMsg = (m, raw, pair) => {
    if (m?.type === 'lease_granted' && !heldGrant) { heldGrant = { m, raw, pair }; return 'drop'; }
    return baseHook ? baseHook(m, raw, pair) : 'forward';
  };
  await page.focus(confirmSel);
  const mark = rec.mark();
  await page.keyboard.down('Enter');
  await waitFor(() => heldGrant != null, { timeoutMs: 4000, label: `${label}: hold lease_granted` });
  await page.keyboard.down('NumpadEnter');
  rec.hooks.onServerMsg = baseHook;
  heldGrant.pair.ws.send(heldGrant.raw);
  await waitFn(page, (msel) => S.leaseGeneration != null && document.querySelector(msel)?.hidden === true, modalSel, 5000);
  await page.keyboard.up('Enter'); // release Enter alone; NumpadEnter remains held
  for (let i = 0; i < 3; i++) await page.keyboard.down('NumpadEnter');
  await page.waitForTimeout(180);
  const acq = rec.byType('acquire_lease', mark);
  assert.equal(acq.length, 1, `${label}: expected 1 acquire_lease`);
  assert.equal(acq[0].json.force, expectForce, `${label}: force mismatch`);
  assert.equal(rec.motionFrames(mark).length, 0, `${label}: releasing Enter unblocked held NumpadEnter!`);
  if (testSpaceEstop) {
    await page.keyboard.press('Space');
    await waitFn(page, () => S.state === 'EStop', null, 3000);
  }
  await page.keyboard.up('NumpadEnter');
  return mark;
}

async function withDelayedGraspSetup(page, onHit) {
  let hit = false, release = null;
  const gate = new Promise((r) => { release = r; });
  const handler = async (route) => {
    hit = true;
    const resp = await route.fetch();
    let body = { setup: { table_z_m: 0.0, camera_tilt_deg: 25 } };
    if (resp.ok()) {
      try {
        const parsed = await resp.json();
        const s = parsed.setup || parsed;
        if (Number.isFinite(Number(s?.table_z_m))) body = parsed;
      } catch (_) {}
    }
    await gate;
    await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(body) }).catch(() => {});
  };
  await page.route('**/api/v1/grasp/setup', handler);
  try {
    await page.evaluate(() => { cwSearch(); });
    await waitFor(() => hit, { timeoutMs: 5000, label: '/api/v1/grasp/setup intercepted' });
    await onHit();
  } finally {
    release();
    await page.unroute('**/api/v1/grasp/setup', handler);
  }
  await waitFn(page, () => !CWZ.running, null, 5000);
}

async function ensureArmedUnleased(port, page) {
  await withExtClient(port, 'ext-prep', async (ext) => {
    await ext.acquireLease({ force: false, heartbeat: true });
    let o = await getObs(port);
    if (o.safety_state === 'EStop' || (typeof o.safety_state === 'string' && o.safety_state.startsWith('Fault')) || (o.safety_state && typeof o.safety_state === 'object')) {
      await ext.command('/api/v1/safety/clear_fault', {});
      await waitFor(async () => (await getObs(port)).safety_state === 'Idle', { timeoutMs: 3000, label: 'clear_fault -> Idle' });
    }
    o = await getObs(port);
    if (!o.is_torqued || o.safety_state !== 'Armed') {
      await ext.command('/api/v1/torque', { enable: true });
      await waitFor(async () => { const x = await getObs(port); return x.is_torqued && x.safety_state === 'Armed'; }, { timeoutMs: 3000, label: 'torque -> Armed' });
    }
    ext.sendHeartbeatNow();
    await sleep(30);
  });
  await waitFn(page, () => isLive() && S.wsObsSeen && S.state === 'Armed' && S.obs?.is_torqued === true
    && S.obs?.lease_holder == null && S.leaseGeneration === null && !document.body.classList.contains('locked'), null, 3000);
}

async function clearEstopExternal(port, { pin = null } = {}) {
  await withExtClient(port, 'ext-clear', async (ext) => {
    if (pin) await ext.auth(await unlockOperatorToken(port, pin));
    await ext.acquireLease({ force: false, heartbeat: true });
    await ext.command('/api/v1/safety/clear_fault', {});
    await waitFor(async () => (await getObs(port)).safety_state === 'Idle', { timeoutMs: 3000, label: 'external clear_fault -> Idle' });
    await ext.releaseLease();
  });
}

async function runScenario9OwnerHonesty(browserInst) {
  log('=== Scenario 9: Honest cockpit owner status, fallback banners, and up-front feature gating ===');
  const viewports = [
    { tag: '390', width: 390, height: 844 },
    { tag: '1024', width: 1024, height: 768 },
  ];
  const langs = [
    { code: 'zh', locale: 'zh-CN' },
    { code: 'en', locale: 'en-US' },
  ];

  // --- 9A. Executor default (auto -> executor): stroke & hardware-executor gravity gating ---
  {
    const port = await allocFreePort();
    const proc = await spawnArmd(port, ['--config', B601_CONFIG, '--no-operator-auth']);
    try {
      const st = await (await fetch(`http://127.0.0.1:${port}/api/v1/owner/status`, { cache: 'no-store' })).json();
      assert.equal(st.effective_owner, 'executor');
      assert.equal(st.owner_reason, 'auto_qualified');
      assert.equal(st.is_fallback, false);
      assert(st.legacy_only_routes.ws.includes('/api/v1/action/gripper_calibrate'));

      for (const lang of langs) {
        for (const vp of viewports) {
          const tag = `exec_${lang.code}_${vp.tag}`;
          const { ctx, page, guards, rec } = await newCockpit(browserInst, port, { width: vp.width, height: vp.height, locale: lang.locale });
          try {
            await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang.code);
            await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'domcontentloaded' });
            await waitForCockpitLive(page);
            await waitFn(page, () => S.ownerDiag && S.ownerDiag.effective_owner === 'executor', null, 5000);

            // Normal executor hides owner banner.
            assert.equal(await page.evaluate(() => document.getElementById('owner-banner').hidden), true, `${tag}: normal executor must hide owner banner`);

            // Mock driverless executor keeps #btn-gc available (capability-driven, NOT hard-coded to owner).
            assert.equal(await page.evaluate(() => S.caps?.gravity_comp === true && S.obs?.gravity_comp?.supported === true), true, `${tag}: mock executor should advertise gravity_comp`);

            // Ensure arm is Armed + torqued so torque precondition is satisfied.
            await ensureArmedUnleased(port, page);

            // Open Settings -> Gripper calibration: #gb-run must be disabled up front with specific legacy reason.
            await page.click('#btn-sset');
            await waitShown(page, '#sset', 3000);
            await page.click('#tool-gripper');
            await waitShown(page, '#gtool', 3000);
            await waitFn(page, () => !!document.getElementById('gb-run'), null, 3000);

            const gbInfo = await page.evaluate(() => {
              const btn = document.getElementById('gb-run');
              const why = document.getElementById('gb-owner-why');
              return {
                disabled: btn?.disabled,
                ariaDisabled: btn?.getAttribute('aria-disabled'),
                title: btn?.title || '',
                whyText: why?.textContent?.trim() || '',
                sw: document.documentElement.scrollWidth,
                iw: window.innerWidth,
              };
            });
            assert.equal(gbInfo.disabled, true, `${tag}: #gb-run must be disabled on executor`);
            assert.equal(gbInfo.ariaDisabled, 'true', `${tag}: #gb-run aria-disabled must be true`);
            assert(gbInfo.whyText.includes('arm.armd_owner') && /legacy/i.test(gbInfo.whyText), `${tag}: #gb-owner-why missing config fix: ${gbInfo.whyText}`);
            assert(lang.code === 'zh' ? /完整夹爪行程标定/.test(gbInfo.whyText) : /Full gripper stroke calibration/i.test(gbInfo.whyText), `${tag}: wrong lang in #gb-owner-why: ${gbInfo.whyText}`);
            assert(gbInfo.sw <= gbInfo.iw, `${tag}: horizontal overflow in gtool (${gbInfo.sw} > ${gbInfo.iw})`);
            const markGb = rec.mark();
            await page.evaluate(() => { document.getElementById('gb-run')?.click(); });
            await page.waitForTimeout(150);
            assert.equal(rec.commands('/api/v1/action/gripper_calibrate', markGb).length, 0, `${tag}: disabled #gb-run must not send WS command`);

            const strokeShot = path.join(SHOT_DIR, `owner_executor_stroke_gated_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: strokeShot });
            log(`Saved screenshot: ${strokeShot}`);
            await page.click('#gtool-close');
            await waitHidden(page, '#gtool', 2000);

            // Hardware-executor capability parity (B601-DM hardware reports gravity_comp.supported=false + EXECUTOR_CAPABILITY_UNSUPPORTED).
            rec.hooks.onServerMsg = (m) => {
              if (m && Array.isArray(m.positions) && m.gravity_comp) {
                const patched = {
                  ...m,
                  gravity_comp: {
                    ...m.gravity_comp,
                    supported: false,
                    blocker: {
                      code: 'EXECUTOR_CAPABILITY_UNSUPPORTED',
                      message: 'Gravity compensation hardware MIT mode-switch is not supported on the RT executor; set arm.armd_owner to legacy and restart armd',
                    },
                  },
                };
                return JSON.stringify(patched);
              }
              return 'forward';
            };
            await page.evaluate(() => { if (S.caps) S.caps.gravity_comp = false; });
            await waitFn(page, () => S.obs?.gravity_comp?.blocker?.code === 'EXECUTOR_CAPABILITY_UNSUPPORTED', null, 4000);
            await page.waitForTimeout(150);

            const gcInfo = await page.evaluate(() => {
              const sw = document.getElementById('btn-gc');
              const sub = document.getElementById('gc-sub');
              return {
                hidden: sw?.hidden,
                blocked: sw?.classList.contains('blocked'),
                ariaDisabled: sw?.getAttribute('aria-disabled'),
                subText: sub?.textContent?.trim() || '',
                sw: document.documentElement.scrollWidth,
                iw: window.innerWidth,
              };
            });
            assert.equal(gcInfo.hidden, false, `${tag}: #btn-gc must stay visible when gated by EXECUTOR_CAPABILITY_UNSUPPORTED`);
            assert.equal(gcInfo.blocked, true, `${tag}: #btn-gc must have .blocked`);
            assert.equal(gcInfo.ariaDisabled, 'true', `${tag}: #btn-gc must have aria-disabled=true`);
            assert(gcInfo.subText.includes('arm.armd_owner') && /legacy/i.test(gcInfo.subText), `${tag}: #gc-sub missing fix: ${gcInfo.subText}`);
            assert(gcInfo.sw <= gcInfo.iw, `${tag}: horizontal overflow on gravity gated (${gcInfo.sw} > ${gcInfo.iw})`);

            // Tapping #btn-gc (aria-disabled) shows toast with cause+fix and never submits /api/v1/mode/gravity_comp.
            const markGc = rec.mark();
            await clearToasts(page);
            await page.locator('#btn-gc').scrollIntoViewIfNeeded();
            await page.locator('#btn-gc').click({ force: true });
            await page.waitForTimeout(200);
            const gcToasts = await getToasts(page);
            assert(gcToasts.all.includes('arm.armd_owner') && /legacy/i.test(gcToasts.all), `${tag}: tapping gated #btn-gc must toast cause+fix, got: ${gcToasts.all}`);
            assert.equal(rec.commands('/api/v1/mode/gravity_comp', markGc).length, 0, `${tag}: gated #btn-gc must not send gravity_comp command`);

            const gcShot = path.join(SHOT_DIR, `owner_executor_gravity_gated_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: gcShot });
            log(`Saved screenshot: ${gcShot}`);

            // Calibration wizard hand-guide button is also disabled with specific executor reason.
            await clearToasts(page);
            await page.evaluate(async () => { cwOpen(true); await cwLoad(); CWZ.step = 'place'; cwRender(); });
            await waitShown(page, '#cwiz', 3000);
            await waitFn(page, () => !!document.getElementById('cwiz-find-hand'), null, 4000);
            const wizInfo = await page.evaluate(() => {
              const btn = document.getElementById('cwiz-find-hand');
              const whys = [...document.querySelectorAll('#cwiz .cwiz-why')].map((e) => e.textContent.trim()).join(' | ');
              return { disabled: btn?.disabled, title: btn?.title || '', whys };
            });
            assert.equal(wizInfo.disabled, true, `${tag}: #cwiz-find-hand must be disabled when EXECUTOR_CAPABILITY_UNSUPPORTED`);
            assert(wizInfo.whys.includes('arm.armd_owner') && /legacy/i.test(wizInfo.whys), `${tag}: wizard why missing fix: ${wizInfo.whys}`);
            const wizShot = path.join(SHOT_DIR, `owner_executor_wizard_hand_gated_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: wizShot });
            log(`Saved screenshot: ${wizShot}`);
            await page.evaluate(() => cwOpen(false));

            // Disarm/release cleanly before next viewport.
            rec.hooks.onServerMsg = null;
            guards.assertClean(`${tag}`);
          } finally {
            await ctx.close();
          }
        }
      }
    } finally {
      await killArmd(proc);
    }
  }

  // --- 9B. Forced OWNER_FAILED: fresh load shows owner badge + danger banner even while obs/diag are 503 ---
  {
    const port = await allocFreePort();
    const proc = await spawnArmd(port, ['--config', B601_CONFIG, '--no-operator-auth'], {
      extraEnv: { ARMD_MOCK_TEST_OWNER_MODE: 'failed' },
      waitOwnerStatus: true,
    });
    try {
      const obsRes = await fetch(`http://127.0.0.1:${port}/api/v1/observation`, { cache: 'no-store' });
      const diagRes = await fetch(`http://127.0.0.1:${port}/api/v1/diagnostics`, { cache: 'no-store' });
      assert.equal(obsRes.status, 503, 'OWNER_FAILED must gate /api/v1/observation with 503');
      assert.equal(diagRes.status, 503, 'OWNER_FAILED must gate /api/v1/diagnostics with 503');

      for (const lang of langs) {
        for (const vp of viewports) {
          const tag = `failed_${lang.code}_${vp.tag}`;
          const { ctx, page, guards } = await newCockpit(browserInst, port, { width: vp.width, height: vp.height, locale: lang.locale });
          const badAssets = [];
          const okAssets = [];
          page.on('response', (resp) => {
            try {
              const u = new URL(resp.url());
              if (u.pathname === '/' || u.pathname === '/favicon.ico' || u.pathname.startsWith('/assets/')) {
                if (resp.status() >= 400) badAssets.push(`${u.pathname}:${resp.status()}`);
                else okAssets.push(`${u.pathname}:${resp.status()}`);
              }
            } catch (_) {}
          });
          try {
            await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang.code);
            await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'load' });
            await waitShown(page, '#owner-banner', 5000);
            await waitShown(page, '#owner-badge', 5000);
            assert.equal(badAssets.length, 0, `${tag}: cockpit shell/assets failed under OWNER_FAILED: ${JSON.stringify(badAssets)}`);
            assert(okAssets.some((x) => x.startsWith('/assets/jmuxer.min.js:200')), `${tag}: /assets/jmuxer.min.js must load 200 under OWNER_FAILED, saw ${JSON.stringify(okAssets)}`);

            const info = await page.evaluate(() => {
              const badge = document.getElementById('owner-badge');
              const banner = document.getElementById('owner-banner');
              const title = document.getElementById('owner-banner-title');
              const sub = document.getElementById('owner-banner-sub');
              const br = banner.getBoundingClientRect();
              const bgr = badge.getBoundingClientRect();
              return {
                badgeText: badge.textContent.trim(),
                badgeCls: badge.className,
                badgeVisible: getComputedStyle(badge).display !== 'none' && bgr.width > 0 && bgr.height > 0,
                bannerCls: banner.className,
                bannerRole: banner.getAttribute('role'),
                bannerVisible: getComputedStyle(banner).display !== 'none' && br.width > 0 && br.height > 0,
                bannerInViewport: br.left >= 0 && br.right <= window.innerWidth + 1 && br.top >= 0 && br.bottom <= window.innerHeight + 1,
                titleText: title.textContent.trim(),
                subText: sub.textContent.trim(),
                obs: S.obs,
                sw: document.documentElement.scrollWidth,
                iw: window.innerWidth,
              };
            });
            assert.equal(info.obs, null, `${tag}: observation must be null under OWNER_FAILED`);
            assert.equal(info.badgeVisible, true, `${tag}: #owner-badge must be visible on fresh OWNER_FAILED load`);
            assert.equal(info.bannerVisible, true, `${tag}: #owner-banner must be visible on fresh OWNER_FAILED load`);
            assert.equal(info.bannerInViewport, true, `${tag}: #owner-banner must fit inside viewport without clipping`);
            assert(info.bannerCls.includes('is-danger') && info.bannerRole === 'alert', `${tag}: failed banner class/role: ${info.bannerCls} ${info.bannerRole}`);
            assert(info.subText.includes('arm.armd_owner') && /legacy/i.test(info.subText), `${tag}: failed banner must state next step: ${info.subText}`);
            assert(lang.code === 'zh' ? /实时执行器启动失败/.test(info.titleText) : /RT executor failed to start/i.test(info.titleText), `${tag}: failed title mismatch: ${info.titleText}`);
            assert(info.sw <= info.iw, `${tag}: horizontal overflow (${info.sw} > ${info.iw})`);

            const shot = path.join(SHOT_DIR, `owner_failed_banner_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: shot });
            log(`Saved screenshot: ${shot}`);
            guards.assertClean(`${tag}`);
          } finally {
            await ctx.close();
          }
        }
      }
    } finally {
      await killArmd(proc);
    }
  }

  // --- 9C. Forced startup_fallback: fresh load shows startup fallback banner + badge in zh and en ---
  {
    const port = await allocFreePort();
    const proc = await spawnArmd(port, ['--config', B601_CONFIG, '--no-operator-auth'], {
      extraEnv: { ARMD_MOCK_TEST_OWNER_MODE: 'startup_fallback' },
    });
    try {
      for (const lang of langs) {
        for (const vp of viewports) {
          const tag = `fallback_${lang.code}_${vp.tag}`;
          const { ctx, page, guards } = await newCockpit(browserInst, port, { width: vp.width, height: vp.height, locale: lang.locale });
          try {
            await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang.code);
            await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'domcontentloaded' });
            await waitForCockpitLive(page);
            await waitShown(page, '#owner-banner', 5000);
            await waitShown(page, '#owner-badge', 5000);

            const info = await page.evaluate(() => {
              const badge = document.getElementById('owner-badge');
              const banner = document.getElementById('owner-banner');
              const title = document.getElementById('owner-banner-title');
              const sub = document.getElementById('owner-banner-sub');
              const br = banner.getBoundingClientRect();
              return {
                badgeCls: badge.className,
                bannerInViewport: br.left >= 0 && br.right <= window.innerWidth + 1,
                titleText: title.textContent.trim(),
                subText: sub.textContent.trim(),
                sw: document.documentElement.scrollWidth,
                iw: window.innerWidth,
              };
            });
            assert.equal(info.bannerInViewport, true, `${tag}: startup fallback banner must fit inside viewport without clipping`);
            assert(info.badgeCls.includes('is-fallback') && info.badgeCls.includes('is-warn'), `${tag}: fallback badge classes: ${info.badgeCls}`);
            assert(
              lang.code === 'zh'
                ? /执行器启动失败，已回退到 legacy 保持状态/.test(info.titleText)
                : /RT executor failed to start; fell back to legacy holding/i.test(info.titleText),
              `${tag}: startup fallback title mismatch: ${info.titleText}`,
            );
            assert(info.subText.includes('arm.armd_owner'), `${tag}: startup fallback banner must explain next step: ${info.subText}`);
            assert(info.sw <= info.iw, `${tag}: horizontal overflow (${info.sw} > ${info.iw})`);

            const shot = path.join(SHOT_DIR, `owner_startup_fallback_banner_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: shot });
            log(`Saved screenshot: ${shot}`);
            guards.assertClean(`${tag}`);
          } finally {
            await ctx.close();
          }
        }
      }
    } finally {
      await killArmd(proc);
    }
  }

  // --- 9D. Legacy owner (--owner legacy): stroke & gravity controls remain enabled as before ---
  {
    const port = await allocFreePort();
    const proc = await spawnArmd(port, ['--config', B601_CONFIG, '--owner', 'legacy', '--no-operator-auth']);
    try {
      const st = await (await fetch(`http://127.0.0.1:${port}/api/v1/owner/status`, { cache: 'no-store' })).json();
      assert.equal(st.effective_owner, 'legacy');
      assert.equal(st.owner_reason, 'explicit_cli');
      assert.equal(st.is_fallback, false);

      for (const lang of langs) {
        for (const vp of viewports) {
          const tag = `legacy_${lang.code}_${vp.tag}`;
          const { ctx, page, guards } = await newCockpit(browserInst, port, { width: vp.width, height: vp.height, locale: lang.locale });
          try {
            await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang.code);
            await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'domcontentloaded' });
            await waitForCockpitLive(page);
            await waitFn(page, () => S.ownerDiag && S.ownerDiag.effective_owner === 'legacy', null, 5000);

            assert.equal(await page.evaluate(() => document.getElementById('owner-banner').hidden), true, `${tag}: explicit legacy must hide owner banner`);

            await ensureArmedUnleased(port, page);

            // Gravity switch is visible and enabled (not blocked).
            const gcLeg = await page.evaluate(() => {
              const sw = document.getElementById('btn-gc');
              return { hidden: sw?.hidden, blocked: sw?.classList.contains('blocked'), ariaDisabled: sw?.getAttribute('aria-disabled') };
            });
            assert.equal(gcLeg.hidden, false, `${tag}: #btn-gc must be visible on legacy`);
            assert.equal(gcLeg.blocked, false, `${tag}: #btn-gc must not be blocked on legacy`);
            assert.equal(gcLeg.ariaDisabled, 'false', `${tag}: #btn-gc aria-disabled must be false on legacy`);

            // Wizard hand-guide button is enabled on legacy.
            await page.evaluate(async () => { cwOpen(true); await cwLoad(); CWZ.step = 'place'; cwRender(); });
            await waitShown(page, '#cwiz', 3000);
            await waitFn(page, () => !!document.getElementById('cwiz-find-hand'), null, 4000);
            assert.equal(await page.evaluate(() => document.getElementById('cwiz-find-hand')?.disabled), false, `${tag}: #cwiz-find-hand must be enabled on legacy`);
            await page.evaluate(() => cwOpen(false));

            // Gripper stroke calibration button is enabled on legacy when torqued.
            await page.click('#btn-sset');
            await waitShown(page, '#sset', 3000);
            await page.click('#tool-gripper');
            await waitShown(page, '#gtool', 3000);
            await waitFn(page, () => !!document.getElementById('gb-run'), null, 3000);

            const gbLeg = await page.evaluate(() => ({
              disabled: document.getElementById('gb-run')?.disabled,
              hasOwnerWhy: !!document.getElementById('gb-owner-why'),
              sw: document.documentElement.scrollWidth,
              iw: window.innerWidth,
            }));
            assert.equal(gbLeg.disabled, false, `${tag}: #gb-run must be enabled on legacy when torqued`);
            assert.equal(gbLeg.hasOwnerWhy, false, `${tag}: #gb-owner-why must not appear on legacy`);
            assert(gbLeg.sw <= gbLeg.iw, `${tag}: horizontal overflow on legacy gtool (${gbLeg.sw} > ${gbLeg.iw})`);

            const shot = path.join(SHOT_DIR, `owner_legacy_enabled_${lang.code}_${vp.tag}.png`);
            await page.screenshot({ path: shot });
            log(`Saved screenshot: ${shot}`);
            await page.click('#gtool-close');
            await waitHidden(page, '#gtool', 2000);
            guards.assertClean(`${tag}`);
          } finally {
            await ctx.close();
          }
        }
      }
    } finally {
      await killArmd(proc);
    }
  }

  // --- 9E. Same-page lifecycle: hardware-gated executor -> restart into OWNER_FAILED -> restart into legacy (no reload) ---
  {
    const port = await allocFreePort();
    let proc = await spawnArmd(port, ['--config', B601_CONFIG, '--no-operator-auth']);
    const { ctx, page, guards, rec } = await newCockpit(browserInst, port, { width: 1024, height: 768, locale: 'en-US' });
    let simulateHwExecGravity = true;
    await page.route('**/api/v1/observation/schema', async (route) => {
      if (!simulateHwExecGravity) return route.continue();
      const resp = await route.fetch();
      if (!resp.ok()) return route.fulfill({ response: resp });
      const body = await resp.json();
      if (body && body.capabilities) body.capabilities.gravity_comp = false;
      return route.fulfill({ status: resp.status(), contentType: 'application/json', body: JSON.stringify(body) });
    });
    rec.hooks.onServerMsg = (m) => {
      if (simulateHwExecGravity && m && Array.isArray(m.positions) && m.gravity_comp) {
        return JSON.stringify({
          ...m,
          gravity_comp: {
            ...m.gravity_comp,
            supported: false,
            blocker: {
              code: 'EXECUTOR_CAPABILITY_UNSUPPORTED',
              message: 'Gravity compensation hardware MIT mode-switch is not supported on the RT executor; set arm.armd_owner to legacy and restart armd',
            },
          },
        });
      }
      return 'forward';
    };
    try {
      await ctx.addInitScript(() => { try { localStorage.setItem('lang', 'en'); } catch (_) {} });
      await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'load' });
      await waitForCockpitLive(page);
      await waitFn(
        page,
        () => S.ownerDiag?.effective_owner === 'executor'
          && S.caps?.gravity_comp === false
          && document.getElementById('owner-banner').hidden === true
          && document.getElementById('btn-gc').hidden === false
          && document.getElementById('btn-gc').classList.contains('blocked'),
        null,
        5000,
      );

      // Restart daemon into OWNER_FAILED on the same port without reloading the page.
      simulateHwExecGravity = false;
      rec.hooks.onServerMsg = null;
      await killArmd(proc);
      proc = await spawnArmd(port, ['--config', B601_CONFIG, '--no-operator-auth'], {
        extraEnv: { ARMD_MOCK_TEST_OWNER_MODE: 'failed' },
        waitOwnerStatus: true,
      });
      await waitFn(
        page,
        () => S.ownerDiag?.effective_owner === 'failed'
          && document.getElementById('owner-banner').hidden === false
          && document.getElementById('owner-banner').className.includes('is-danger'),
        null,
        10000,
      );

      // Restart daemon into legacy on the same port without reloading the page:
      // failed banner clears, schema reloads (S.caps.gravity_comp=true), stroke calibration and BOTH
      // gravity controls (#btn-gc + wizard hand-guide) are restored without a manual page reload.
      await killArmd(proc);
      proc = await spawnArmd(port, ['--config', B601_CONFIG, '--owner', 'legacy', '--no-operator-auth']);
      await waitForCockpitLive(page, 15000);
      await waitFn(
        page,
        () => S.ownerDiag?.effective_owner === 'legacy'
          && S.caps?.gravity_comp === true
          && document.getElementById('owner-banner').hidden === true
          && document.getElementById('btn-gc').hidden === false
          && legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only') === null,
        null,
        10000,
      );
      await ensureArmedUnleased(port, page);
      await waitFn(
        page,
        () => document.getElementById('btn-gc').hidden === false
          && !document.getElementById('btn-gc').classList.contains('blocked')
          && document.getElementById('btn-gc').getAttribute('aria-disabled') === 'false',
        null,
        5000,
      );
      await page.evaluate(async () => { cwOpen(true); await cwLoad(); CWZ.step = 'place'; cwRender(); });
      await waitShown(page, '#cwiz', 3000);
      await waitFn(page, () => document.getElementById('cwiz-find-hand')?.disabled === false, null, 4000);
      await page.evaluate(() => cwOpen(false));

      guards.assertClean('9E same-page lifecycle');
      log('PASS Scenario 9E: same-page hw-gated executor -> OWNER_FAILED -> recovered legacy reloaded schema and restored banner, stroke, and gravity controls without reload.');
    } finally {
      await ctx.close();
      await killArmd(proc);
    }
  }
  log('PASS Scenario 9: owner status, OWNER_FAILED/startup_fallback banners, and executor/legacy feature gating verified (zh/en, 390/1024).');
}

async function main() {
  fs.mkdirSync(SHOT_DIR, { recursive: true });
  log(`ARMD_BIN=${ARMD_BIN} | SHOT_DIR=${SHOT_DIR}`);

  let armdMain = null, armdInteract = null, armdAuth = null;

  try {
    browser = await chromium.launch({
      executablePath: process.env.CHROME || undefined,
      headless: true,
      args: ['--no-sandbox', '--disable-setuid-sandbox'],
    });

    if (process.env.ONLY_OWNER_HONESTY === '1') {
      await runScenario9OwnerHonesty(browser);
      return;
    }

    /* =====================================================================
       SCENARIOS 1–4 & 6: B601-DM mock profile on port 18877
       ===================================================================== */
    log(`Spawning armd (B601-DM mock) on port ${PORT_MAIN}...`);
    armdMain = await spawnArmd(PORT_MAIN, ['--config', B601_CONFIG]);

    // --- Scenario 1: Cold load (en-US & fresh zh-CN), reload & read-only status ---
    log('=== Scenario 1: Cold load (en-US & zh-CN), reload & read-only status ===');
    {
      const { ctx, page, guards, rec } = await newCockpit(browser, PORT_MAIN, { locale: 'en-US' });
      try {
        await page.goto(`http://127.0.0.1:${PORT_MAIN}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        await page.waitForTimeout(2200);
        const st = await page.evaluate(() => ({
          gen: S.leaseGeneration, lsHidden: document.getElementById('lease-status').hidden,
          lsCls: document.getElementById('lease-status').className, lsTxt: document.getElementById('lease-status-txt').textContent.trim(),
          lrHidden: document.getElementById('lease-released').hidden,
        }));
        assert.equal(rec.byType('acquire_lease').length, 0);
        assert.equal(st.gen, null);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, null);
        assert(!st.lsHidden && st.lsCls.includes('is-readonly') && /Read-only|No holder/i.test(st.lsTxt) && st.lrHidden);
        await assertControlInputsState(page, 'unavailable', 'Scenario 1 cold load en-US');

        await page.click('#btn-lang'); await page.waitForTimeout(120);
        assert(/只读/.test(await page.textContent('#lease-status-txt')));
        assert.equal(rec.byType('acquire_lease').length, 0);
        await page.click('#btn-lang'); await page.waitForTimeout(120);
        const shot1 = path.join(SHOT_DIR, 'cockpit_on_load_no_control.png');
        await page.screenshot({ path: shot1 }); log(`Saved screenshot: ${shot1}`);

        await page.reload({ waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        await page.waitForTimeout(2200);
        assert.equal(rec.byType('acquire_lease').length, 0);
        assert.equal(await page.evaluate(() => S.leaseGeneration), null);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, null);
        await assertControlInputsState(page, 'unavailable', 'Scenario 1 reload en-US');
        guards.assertClean('Scenario 1 en-US');
      } finally { await ctx.close(); }

      // Fresh zh-CN context: cold load + reload read-only check
      const { ctx: zhCtx, page: zhPage, guards: zhGuards, rec: zhRec } = await newCockpit(browser, PORT_MAIN, { locale: 'zh-CN' });
      try {
        for (const step of ['goto', 'reload']) {
          if (step === 'goto') await zhPage.goto(`http://127.0.0.1:${PORT_MAIN}/`, { waitUntil: 'domcontentloaded' });
          else await zhPage.reload({ waitUntil: 'domcontentloaded' });
          await waitForCockpitLive(zhPage); await zhPage.waitForTimeout(600);
          assert(zhRec.byType('acquire_lease').length === 0 && await zhPage.evaluate(() => S.leaseGeneration) === null
            && (await getObs(PORT_MAIN)).lease_holder === null && /只读|无控制方/.test(await zhPage.textContent('#lease-status-txt')));
          await assertControlInputsState(zhPage, 'unavailable', 'Scenario 1 ' + step + ' zh-CN');
        }
        zhGuards.assertClean('Scenario 1 zh-CN');
        log('PASS Scenario 1: en-US + fresh zh-CN cold load & reload sent 0 acquire_lease, read-only confirmed.');
      } finally { await zhCtx.close(); }
    }

    // --- Scenarios 2, 3, 4: Watchdog park overlap, Explicit Enable, Reconnect ---
    log('=== Scenario 2: Active watchdog park preservation ===');
    await withExtClient(PORT_MAIN, 'ext-wd', async (ext) => {
      await ext.acquireLease({ force: false, heartbeat: true });
      await ext.command('/api/v1/torque', { enable: true });
      await waitFor(async () => { const o = await getObs(PORT_MAIN); return o.is_torqued && o.safety_state === 'Armed'; }, { timeoutMs: 4000, label: 'ext-wd torque on' });
      await ext.command('/api/v1/motion/jog', { target_positions: [1.6, 0, 0, 0, 0, 0, 0] });
      await waitFor(async () => { const o = await getObs(PORT_MAIN); return Math.abs(o.positions[0]) > 1.3 && o.safety_state === 'Armed'; }, { timeoutMs: 12000, intervalMs: 100, label: 'ext-wd J1 away' });
    });
    await waitFor(async () => { const o = await getObs(PORT_MAIN); return o.lease_holder === null && o.is_torqued && o.safety_state === 'Moving'; }, { timeoutMs: 6000, intervalMs: 60, label: 'watchdog park active' });

    {
      const { ctx, page, guards, rec } = await newCockpit(browser, PORT_MAIN);
      try {
        await page.goto(`http://127.0.0.1:${PORT_MAIN}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        const obsDuringLoad = await getObs(PORT_MAIN);
        assert(obsDuringLoad.safety_state === 'Moving' && obsDuringLoad.lease_holder === null, `Scenario 2 load overlap: ${JSON.stringify(obsDuringLoad)}`);

        await page.reload({ waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        const obsDuringReload = await getObs(PORT_MAIN);
        assert(obsDuringReload.safety_state === 'Moving' && obsDuringReload.lease_holder === null, `Scenario 2 reload overlap: ${JSON.stringify(obsDuringReload)}`);

        const obsDone = await waitFor(async () => { const o = await getObs(PORT_MAIN); return !o.is_torqued && o.safety_state === 'Idle' ? o : null; }, { timeoutMs: 15000, intervalMs: 100, label: 'watchdog park completion' });
        assert.equal(rec.byType('acquire_lease').length, 0);
        assert.equal(obsDone.lease_holder, null);
        assert(Math.abs(obsDone.positions[0]) < 0.15, `Final J1=${obsDone.positions[0]}`);
        guards.assertClean('Scenario 2');
        log(`PASS Scenario 2: watchdog park overlapped load+reload (Moving, 0 acquire_lease) -> Idle J1=${obsDone.positions[0].toFixed(3)}.`);

        log('=== Scenario 3: Explicit Enable acquires exactly 1 lease ===');
        await waitFn(page, () => S.state === 'Idle' && S.obs?.is_torqued === false && S.obs?.lease_holder == null, null, 4000);
        const m3 = rec.mark(), clientId = await page.evaluate(() => CLIENT_ID);
        await holdButton(page, '#btn-enable', 750);
        await waitFn(page, () => S.state === 'Armed' && S.leaseGeneration != null && S.obs?.is_torqued === true, null, 6000);
        await waitFor(async () => { const o = await getObs(PORT_MAIN); return o.is_torqued && o.lease_holder === clientId; }, { timeoutMs: 4000, label: 'server Enable lease' });
        await page.waitForTimeout(120);
        const acq3 = rec.byType('acquire_lease', m3);
        assert.equal(acq3.length, 1); assert.equal(acq3[0].json.force, false);
        assert(await page.evaluate(() => !document.getElementById('lease-status').hidden && document.getElementById('lease-status').className.includes('is-own')));
        await assertControlInputsState(page, 'active', 'Scenario 3 after Enable');
        const shot3 = path.join(SHOT_DIR, 'cockpit_after_enable_in_control.png');
        await page.screenshot({ path: shot3 }); log(`Saved screenshot: ${shot3}`);
        guards.assertClean('Scenario 3');
        log(`PASS Scenario 3: explicit Enable sent 1 acquire_lease(force=false), holder=${clientId}.`);

        log('=== Scenario 4: Reconnect after holding lease ===');
        const m4 = rec.mark(), wsBefore4 = rec.wsCount;
        await page.evaluate(() => { S.ws.close(); });
        await waitFor(() => rec.wsCount > wsBefore4, { timeoutMs: 6000, label: 'Scenario 4 replacement WS' });
        await waitForCockpitLive(page, 6000);
        await waitFn(page, () => !document.getElementById('lease-released').hidden && S.leaseGeneration === null, null, 4000);
        const st4 = await page.evaluate(() => ({
          state: S.state,
          torqued: S.obs?.is_torqued === true,
          enableTxt: document.getElementById('enable-txt')?.textContent?.trim(),
          lrHidden: document.getElementById('lease-released').hidden,
          lrTxt: document.getElementById('lease-released-txt').textContent.trim(),
          retakeVis: document.getElementById('btn-retake').offsetParent !== null,
          retakeTxt: document.getElementById('btn-retake').textContent.trim(),
          lsCls: document.getElementById('lease-status').className,
        }));
        assert(st4.state === 'Armed' && st4.torqued, `Scenario 4 expected Armed/torqued on reconnect, got ${JSON.stringify(st4)}`);
        assert.equal(st4.enableTxt, 'Torque on · not in control', `Scenario 4 EN #enable-txt mismatch: "${st4.enableTxt}"`);
        assert(!st4.lrHidden && st4.retakeVis && st4.lsCls.includes('is-released'));
        assert(/Control released/i.test(st4.lrTxt), `Scenario 4 banner text: "${st4.lrTxt}"`);
        assert(/Take control/i.test(st4.retakeTxt), `Scenario 4 retake text: "${st4.retakeTxt}"`);

        await page.click('#btn-lang');
        await waitFn(page, () => document.getElementById('enable-txt')?.textContent?.trim() === '力矩已开 · 未控制', null, 1000);
        await page.click('#btn-lang');
        await waitFn(page, () => document.getElementById('enable-txt')?.textContent?.trim() === 'Torque on · not in control', null, 1000);

        await assertControlInputsState(page, 'unavailable', 'Scenario 4 reconnect while torqued');
        const shot4 = path.join(SHOT_DIR, 'cockpit_reconnect_control_released.png');
        await page.screenshot({ path: shot4 }); log(`Saved screenshot: ${shot4}`);

        for (const padSel of ['#pad-sxy', '#pad-sz', '#pad-sgrip']) {
          const box = await (await page.waitForSelector(padSel, { state: 'visible', timeout: 2000 })).boundingBox();
          assert(box && box.width > 0 && box.height > 0, `${padSel} has no boundingBox`);
          await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
          await page.mouse.down();
          await page.mouse.up();
          await waitShown(page, '#take-modal');
          await page.click('#take-cancel');
          await waitHidden(page, '#take-modal');
        }
        for (const presetSel of ['button[data-preset="home"]', 'button[data-preset="inspect"]', 'button[data-preset="park"]']) {
          await page.click(presetSel);
          await waitShown(page, '#take-modal');
          await page.click('#take-cancel');
          await waitHidden(page, '#take-modal');
        }
        assert.equal(rec.byType('acquire_lease', m4).length, 0);

        await page.waitForTimeout(2200);
        assert.equal(rec.byType('acquire_lease', m4).length, 0);
        assert.equal(await page.evaluate(() => S.leaseGeneration), null);
        assert.equal(await page.evaluate(() => !document.getElementById('lease-released').hidden), true);
        await assertControlInputsState(page, 'unavailable', 'Scenario 4 reconnect settled');

        const m4Retake = rec.mark();
        await page.click('#btn-retake');
        await waitFn(page, () => S.leaseGeneration != null && document.getElementById('lease-released').hidden && document.getElementById('lease-status').className.includes('is-own'), null, 5000);
        const acq4 = rec.byType('acquire_lease', m4Retake);
        assert.equal(acq4.length, 1); assert.equal(acq4[0].json.force, false);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, clientId);
        if (await page.evaluate(() => S.state === 'Moving')) {
          await waitFn(page, () => S.state === 'Idle' || S.state === 'Armed', null, 5000);
        }
        if (await page.evaluate(() => S.state !== 'Armed')) {
          await holdButton(page, '#btn-enable', 750);
          await waitFn(page, () => S.state === 'Armed' && S.obs?.is_torqued === true && S.leaseGeneration != null, null, 6000);
        }
        assert.equal(rec.byType('acquire_lease', m4Retake).length, 1);
        await assertControlInputsState(page, 'active', 'Scenario 4 after Retake + Armed');
        guards.assertClean('Scenario 4');
        log('PASS Scenario 4: reconnect sent 0 acquire_lease, showed localized Control released banner, #btn-retake sent 1 acquire_lease.');
      } finally { await ctx.close(); }
    }

    // --- Scenario 6: Protocol edge cases, malformed frames, release & stale telemetry ---
    log('=== Scenario 6: In-flight cancel, malformed ACK/obs, voluntary release & stale telemetry ===');
    await waitFor(async () => { const o = await getObs(PORT_MAIN); return o.lease_holder === null && !o.is_torqued && o.safety_state === 'Idle'; }, { timeoutMs: 8000, label: 'PORT_MAIN Idle before Scenario 6' });
    {
      const { ctx, page, guards, rec: wsRoute } = await newCockpit(browser, PORT_MAIN);
      try {
        await page.goto(`http://127.0.0.1:${PORT_MAIN}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        const clientId = await page.evaluate(() => CLIENT_ID);

        // 6(a): In-flight cancelled acquire via E-Stop
        log('--- Scenario 6(a): In-flight cancelled acquire reconciliation (E-Stop) ---');
        let held6a = null;
        wsRoute.hooks.onServerMsg = (m, raw, pair) => (m?.type === 'lease_granted' && !held6a ? (held6a = { m, raw, pair }, 'drop') : 'forward');
        const m6a = wsRoute.mark();
        await holdButton(page, '#btn-enable', 750);
        const gen6a = (await waitFor(() => held6a?.m, { timeoutMs: 4000, label: '6(a) lease_granted' })).generation;
        await page.click('#btn-estop');
        await waitFn(page, () => S.state === 'EStop', null, 3000);
        await page.waitForTimeout(200);
        wsRoute.hooks.onServerMsg = null;
        held6a.pair.ws.send(held6a.raw);
        await waitFor(() => wsRoute.byType('release_lease', m6a).some((f) => f.json.generation === gen6a), { timeoutMs: 3000, label: '6(a) release_lease' });
        await waitFn(page, () => !leaseOp && S.leaseGeneration === null);
        assert.equal(wsRoute.byType('heartbeat', m6a).filter((f) => f.json.generation === gen6a).length, 0);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, null);
        log(`PASS 6(a): cancelled acquire gen=${gen6a} reconciled with release_lease, 0 heartbeats.`);

        await clearEstopExternal(PORT_MAIN);
        await waitFn(page, () => S.state === 'Idle' && S.obs?.lease_holder == null);

        // 6(a2): Trusted held Escape while #take-modal acquire is pending
        log('--- Scenario 6(a2): Held Escape while #take-modal acquire pending ---');
        await ensureArmedUnleased(PORT_MAIN, page);
        await page.click('button[data-preset="home"]');
        await waitShown(page, '#take-modal');
        let held6a2 = null;
        wsRoute.hooks.onServerMsg = (m, raw, pair) => (m?.type === 'lease_granted' && !held6a2 ? (held6a2 = { m, raw, pair }, 'drop') : 'forward');
        const m6a2 = wsRoute.mark();
        await page.click('#take-confirm');
        const gen6a2 = (await waitFor(() => held6a2?.m, { timeoutMs: 4000, label: '6(a2) lease_granted' })).generation;
        await holdEscapeNoStop(page, wsRoute, '#take-modal', { expectInitialBump: true, label: '6(a2) acquire-pending' });
        wsRoute.hooks.onServerMsg = null;
        held6a2.pair.ws.send(held6a2.raw);
        await waitFor(() => wsRoute.byType('release_lease', m6a2).some((f) => f.json.generation === gen6a2), { timeoutMs: 3000, label: '6(a2) release_lease' });
        await waitFn(page, () => !leaseOp && S.leaseGeneration === null);
        assert.equal(wsRoute.byType('heartbeat', m6a2).filter((f) => f.json.generation === gen6a2).length, 0);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, null);
        await waitFor(async () => { const o = await getObs(PORT_MAIN); return !o.is_torqued && o.safety_state === 'Idle' && o.lease_holder === null; }, { timeoutMs: 8000, label: '6(a2) park -> Idle' });
        await waitFn(page, () => S.state === 'Idle' && S.obs?.lease_holder == null && document.getElementById('enable-txt')?.textContent?.trim() === 'Enable' && !document.getElementById('btn-enable').disabled);
        log(`PASS 6(a2): held Escape during pending acquire gen=${gen6a2} sent 0 stop, repeats preserved controlEpoch, reconciled with release_lease.`);

        // 6(b): Withheld ACK timeout recovery (bounded timing, pending check, timeout toast, 0 hb/motion)
        log('--- Scenario 6(b): Withheld ACK timeout recovery ---');
        await startToastWatch(page);
        let droppedGrant = null, tDrop6b = 0;
        wsRoute.hooks.onServerMsg = (m) => (m?.type === 'lease_granted' ? (droppedGrant = m, tDrop6b = Date.now(), 'drop') : 'forward');
        const wsBefore6b = wsRoute.wsCount, m6b = wsRoute.mark();
        await holdButton(page, '#btn-enable', 750);
        await waitFor(() => droppedGrant != null, { timeoutMs: 4000, label: '6(b) dropped grant' });
        assert.equal((await getObs(PORT_MAIN)).lease_holder, clientId);
        await page.waitForTimeout(500);
        const mid6b = await page.evaluate(() => ({ wsOpen: S.ws?.readyState === 1, opKind: leaseOp?.kind || null, gen: S.leaseGeneration }));
        assert(wsRoute.wsCount === wsBefore6b && mid6b.wsOpen && mid6b.opKind === 'acquire' && mid6b.gen === null, `6(b) mid-wait state: ${JSON.stringify(mid6b)}`);
        wsRoute.hooks.onServerMsg = null;
        await waitFor(() => wsRoute.wsCount > wsBefore6b, { timeoutMs: 8000, label: '6(b) timeout reconnect' });
        const dt6b = Date.now() - tDrop6b;
        assert(dt6b >= 3800 && dt6b <= 6500, `6(b): unexpected timeout+reconnect duration ${dt6b} ms`);
        const wsIdx6b = wsRoute.wsCount;
        await waitForCockpitLive(page, 6000);
        await page.waitForTimeout(2100);
        const toasts6b = await stopToastWatch(page);
        assert(toasts6b.some((t) => TIMEOUT_TOAST_RE.test(t)), `6(b): missing timeout toast in ${JSON.stringify(toasts6b)}`);
        assert.equal(wsRoute.onWs(wsIdx6b, ['acquire_lease', 'heartbeat']).length, 0);
        assert.equal(wsRoute.byType('heartbeat', m6b).length, 0);
        assert.equal(wsRoute.motionFrames(m6b).length, 0);
        assert.equal(await page.evaluate(() => S.leaseGeneration), null);
        assert.equal((await getObs(PORT_MAIN)).lease_holder, null);
        log(`PASS 6(b): withheld grant stayed pending at 500 ms, retired+reconnected in ${dt6b} ms with timeout toast & 0 hb/motion.`);

        // 6(b2): Uncorrelated malformed lease_granted (missing seq & generation) BOTH pending and unsolicited
        log('--- Scenario 6(b2): Uncorrelated malformed lease_granted (pending & unsolicited) ---');
        const checkMalformedGrant = async (label, triggerFn, timeoutMs = 6000) => {
          await startToastWatch(page);
          await page.evaluate(() => {
            window.__genGuard = { active: true, sawNonNull: S.leaseGeneration !== null };
            const tick = () => { if (!window.__genGuard.active) return; if (S.leaseGeneration !== null) window.__genGuard.sawNonNull = true; requestAnimationFrame(tick); };
            requestAnimationFrame(tick);
          });
          const wsBefore = wsRoute.wsCount, m = wsRoute.mark();
          await triggerFn();
          await assertReadOnlyReconnect(page, wsRoute, wsBefore, { port: PORT_MAIN, checkHeartbeat: true, timeoutMs, label });
          const toasts = await stopToastWatch(page);
          assert.equal(toasts.filter((t) => MAL_GRANT_TOAST_RE.test(t)).length, 1, `${label} toasts: ${JSON.stringify(toasts)}`);
          assert.equal(await page.evaluate(() => { window.__genGuard.active = false; return window.__genGuard.sawNonNull || S.leaseGeneration !== null; }), false, `${label}: transient non-null leaseGeneration`);
          assert(wsRoute.byType('heartbeat', m).length === 0 && wsRoute.motionFrames(m).length === 0);
        };
        await checkMalformedGrant('6(b2) pending malformed', async () => {
          let stripped = false;
          wsRoute.hooks.onServerMsg = (m) => (m?.type === 'lease_granted' ? (stripped = true, JSON.stringify({ type: 'lease_granted' })) : 'forward');
          await holdButton(page, '#btn-enable', 750);
          await waitFor(() => stripped, { timeoutMs: 4000, label: '6(b2) malformed pending grant' });
          wsRoute.hooks.onServerMsg = null;
        }, 6000);
        await checkMalformedGrant('6(b2) unsolicited malformed', async () => {
          assert.equal(await page.evaluate(() => leaseOp), null);
          wsRoute.current.ws.send(JSON.stringify({ type: 'lease_granted' }));
        }, 5000);
        log('PASS 6(b2): malformed lease_granted (pending + unsolicited) showed exact-once toast, 0 transient authority & 0 hb/motion.');

        async function acquireIdleLease() {
          if (await page.evaluate(() => !document.getElementById('sset').hidden)) { await page.click('#sset-close'); await waitHidden(page, '#sset'); }
          await page.click('#btn-estop');
          await waitFn(page, () => S.state === 'EStop' && !document.getElementById('btn-enable').disabled, null, 3000);
          await page.click('#btn-enable');
          await waitFn(page, () => S.state === 'Idle' && !S.obs?.is_torqued && S.leaseGeneration != null && S.obs?.lease_holder === CLIENT_ID, null, 5000);
          await page.click('#btn-sset');
          await page.waitForSelector('#tools-release', { state: 'visible', timeout: 3000 });
          await waitFn(page, () => !document.getElementById('tools-release').disabled, null, 3000);
        }

        // 6(c1): Voluntary release stale WS, gated backwards WS & correlated equal/older REST refresh
        log('--- Scenario 6(c1): Voluntary release stale WS, gated backwards WS & correlated equal/older REST ---');
        await acquireIdleLease();
        let cachedPreObs = null;
        wsRoute.hooks.onServerMsg = (m, raw) => { if (Array.isArray(m?.positions) && m.lease_holder === clientId) cachedPreObs = { m, raw }; return 'forward'; };
        await waitFor(() => cachedPreObs != null, { timeoutMs: 2000, label: '6(c1) pre-release obs' });
        await page.evaluate(() => {
          window.__flashLog = { sawOther: false, sawPreemptBtn: false, active: true };
          const tick = () => {
            if (!window.__flashLog.active) return;
            if (document.getElementById('lease-status').className.includes('is-other')) window.__flashLog.sawOther = true;
            if (!document.getElementById('btn-preempt').hidden) window.__flashLog.sawPreemptBtn = true;
            requestAnimationFrame(tick);
          };
          requestAnimationFrame(tick);
        });
        let relFwd = false;
        const replayTimers = [];
        wsRoute.hooks.onServerMsg = (m, raw, pair) => {
          if (m?.type === 'lease_released') {
            pair.ws.send(cachedPreObs.raw);
            for (const dt of [40, 90, 150, 220]) replayTimers.push(setTimeout(() => { try { pair.ws.send(cachedPreObs.raw); } catch (_) {} }, dt));
            relFwd = true;
            replayTimers.push(setTimeout(() => { try { pair.ws.send(raw); } catch (_) {} }, 60));
            return 'drop';
          }
          return 'forward';
        };
        await page.click('#tools-release');
        await waitFor(() => relFwd, { timeoutMs: 3000, label: '6(c1) lease_released' });
        await waitFn(page, () => !leaseOp && S.leaseGeneration === null && document.getElementById('lease-status').className.includes('is-readonly'), null, 5000);
        await page.waitForTimeout(250);
        for (const tm of replayTimers) clearTimeout(tm);
        wsRoute.hooks.onServerMsg = null;

        // Gated backwards WS frame: pause live WS, inject distinct holder + pos0, assert unchanged S.obs & S.obsAt
        const restTs = await page.evaluate(() => S.obsTs);
        await waitFn(page, (t0) => Number.isFinite(S.obsTs) && S.obsTs > t0, restTs, 3000);
        wsRoute.hooks.onServerMsg = (m) => (Array.isArray(m?.positions) ? 'drop' : 'forward');
        await page.waitForTimeout(60);
        const beforeBack = await page.evaluate(() => ({ obsTs: S.obsTs, obsAt: S.obsAt, pos0: S.obs.positions[0], holder: S.obs.lease_holder }));
        await page.waitForTimeout(100);
        const backPos = [0.777, ...cachedPreObs.m.positions.slice(1)];
        wsRoute.current.ws.send(JSON.stringify({ ...cachedPreObs.m, timestamp: restTs - 0.02, lease_holder: 'web-stale99', positions: backPos }));
        await page.waitForTimeout(150);
        const backRes = await page.evaluate(() => ({
          obsTs: S.obsTs, obsAt: S.obsAt, pos0: S.obs.positions[0], holder: S.obs?.lease_holder,
          flash: (window.__flashLog.active = false, { ...window.__flashLog }),
        }));
        wsRoute.hooks.onServerMsg = null;
        assert(backRes.holder === null && backRes.pos0 !== 0.777 && backRes.obsTs === beforeBack.obsTs && backRes.obsAt === beforeBack.obsAt
          && !backRes.flash.sawOther && !backRes.flash.sawPreemptBtn, `6(c1) backwards failed: ${JSON.stringify({ beforeBack, backRes })}`);

        // Correlated equal/older post-release REST refresh (only intercepts refresh after lease_released)
        for (const delta of [0, -0.05]) {
          await acquireIdleLease();
          let sawReleased = false, pauseWs = false, consumed = false, relRest = null;
          const gate = new Promise((r) => { relRest = r; });
          const handler = async (route) => {
            const isRef = sawReleased && await page.evaluate(() => leaseOp?.kind === 'refresh');
            if (!isRef) return route.fallback();
            consumed = true;
            const body = await (await route.fetch()).json();
            const ts = await gate;
            const fakePos = [0.654, ...body.positions.slice(1)];
            await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ ...body, timestamp: ts, lease_holder: 'web-stale-rest', positions: fakePos }) });
          };
          await page.route('**/api/v1/observation', handler);
          wsRoute.hooks.onServerMsg = (m) => {
            if (m?.type === 'lease_released') { sawReleased = true; return 'forward'; }
            return pauseWs && Array.isArray(m?.positions) ? 'drop' : 'forward';
          };
          await page.click('#tools-release');
          await waitFor(() => consumed, { timeoutMs: 3000, label: `6(c1) REST refresh delta=${delta}` });
          await page.waitForTimeout(150);
          pauseWs = true;
          await page.waitForTimeout(50);
          const before = await page.evaluate(() => ({ obsTs: S.obsTs, obsAt: S.obsAt, pos0: S.obs.positions[0], holder: S.obs.lease_holder }));
          await page.waitForTimeout(130);
          relRest(before.obsTs + delta);
          await waitFn(page, () => !leaseOp, null, 3000);
          const after = await page.evaluate(() => ({ obsTs: S.obsTs, obsAt: S.obsAt, pos0: S.obs.positions[0], holder: S.obs.lease_holder }));
          await page.unroute('**/api/v1/observation', handler);
          wsRoute.hooks.onServerMsg = null;
          assert(consumed && after.obsTs === before.obsTs && after.obsAt === before.obsAt && after.holder === null && after.pos0 !== 0.654,
            `6(c1) REST delta=${delta} failed: ${JSON.stringify({ before, after })}`);
        }
        log('PASS 6(c1): stale WS, gated backwards WS, and correlated equal/older REST refresh caused 0 mutations/flashes.');

        // 6(c2): lease_released { success: false }
        log('--- Scenario 6(c2): lease_released { success: false } ---');
        await acquireIdleLease();
        const wsBefore6c2 = wsRoute.wsCount;
        let rewroteFalse = false;
        wsRoute.hooks.onServerMsg = (m) => (m?.type === 'lease_released' ? (rewroteFalse = true, JSON.stringify({ type: 'lease_released', success: false })) : 'forward');
        await page.click('#tools-release');
        await waitFor(() => rewroteFalse, { timeoutMs: 3000, label: '6(c2) success:false' });
        wsRoute.hooks.onServerMsg = null;
        await waitFn(page, () => !leaseOp && S.leaseGeneration === null && document.getElementById('lease-status').className.includes('is-readonly'));
        assert.equal(wsRoute.wsCount, wsBefore6c2);
        assert(/no longer held|不持有控制权/i.test((await getToasts(page)).all));
        log('PASS 6(c2): lease_released { success: false } refreshed cleanly without reconnect.');

        // 6(c3): WS silence after lease_released succeeds via REST
        log('--- Scenario 6(c3): WS silence after lease_released succeeds via REST ---');
        await acquireIdleLease();
        const wsBefore6c3 = wsRoute.wsCount;
        let suppressWs = false, suppressed = 0;
        wsRoute.hooks.onServerMsg = (m) => {
          if (m?.type === 'lease_released') { suppressWs = true; return 'forward'; }
          if (suppressWs && Array.isArray(m?.positions)) { suppressed++; return 'drop'; }
          return 'forward';
        };
        await page.click('#tools-release');
        await waitFn(page, () => !leaseOp && S.leaseGeneration === null && document.getElementById('lease-status').className.includes('is-readonly'), null, 2500);
        await page.waitForTimeout(320);
        wsRoute.hooks.onServerMsg = null;
        assert(suppressed > 5 && wsRoute.wsCount === wsBefore6c3);
        log(`PASS 6(c3): release succeeded via REST under WS silence (${suppressed} WS obs suppressed).`);

        // 6(c4): Failed (500) & withheld GET /api/v1/observation -> bounded timeout reconnect
        log('--- Scenario 6(c4): Failed & withheld GET /api/v1/observation -> reconnect recovery ---');
        await acquireIdleLease();
        const wsBeforeFail = wsRoute.wsCount;
        let failHit = false;
        const failHandler = async (r) => { failHit = true; await r.fulfill({ status: 500, contentType: 'application/json', body: '{"error":"injected"}' }); };
        await page.route('**/api/v1/observation', failHandler);
        await page.click('#tools-release');
        await waitFor(() => failHit, { timeoutMs: 3000, label: '6(c4) 500 hit' });
        await page.unroute('**/api/v1/observation', failHandler);
        await assertReadOnlyReconnect(page, wsRoute, wsBeforeFail, { waitAfterMs: 400, timeoutMs: 5000, label: '6(c4) 500 reconnect' });

        await acquireIdleLease();
        const wsBeforeHung = wsRoute.wsCount;
        let hungHit = false, relHung = null;
        const hungHandler = async (r) => { hungHit = true; await new Promise((res) => { relHung = res; }); await r.abort('failed').catch(() => {}); };
        await page.route('**/api/v1/observation', hungHandler);
        const tHung = Date.now();
        await page.click('#tools-release');
        await waitFor(() => hungHit, { timeoutMs: 3000, label: '6(c4) hung hit' });
        await waitFor(() => wsRoute.wsCount > wsBeforeHung, { timeoutMs: 8000, label: '6(c4) hung reconnect' });
        const dtHung = Date.now() - tHung;
        assert(dtHung >= 2800 && dtHung < 7500, `6(c4): unexpected timeout duration ${dtHung} ms`);
        if (relHung) relHung();
        await page.unroute('**/api/v1/observation', hungHandler);
        const wsIdxHung = wsRoute.wsCount;
        await waitForCockpitLive(page, 6000);
        await page.waitForTimeout(2100);
        assert.equal(wsRoute.onWs(wsIdxHung, ['acquire_lease']).length, 0);
        log(`PASS 6(c4): failed (500) and withheld (${dtHung} ms) refresh recovered via reconnect with 0 acquire_lease.`);

        // 6(d): Stale telemetry disables #preempt-confirm & keeps #btn-stop enabled in observer mode
        log('--- Scenario 6(d): Stale telemetry disables #preempt-confirm & keeps #btn-stop enabled ---');
        if (await page.evaluate(() => !document.getElementById('sset').hidden)) { await page.click('#sset-close'); await waitHidden(page, '#sset'); }
        await withExtClient(PORT_MAIN, 'ext-stale', async (ext) => {
          await ext.acquireLease({ force: false, heartbeat: true });
          await waitShown(page, '#btn-preempt', 4000);
          await page.click('#btn-preempt');
          await waitShown(page, '#preempt-modal');
          await waitFn(page, () => !document.getElementById('preempt-confirm').disabled, null, 2000);
          wsRoute.hooks.onServerMsg = (m) => (Array.isArray(m?.positions) ? 'drop' : 'forward');
          await page.waitForTimeout(Math.max(1700, (await page.evaluate(() => CFG.staleMs)) + 300));
          const st = await page.evaluate(() => ({
            live: isLive(), wsOpen: S.ws?.readyState === 1,
            preemptDisabled: document.getElementById('preempt-confirm').disabled,
            stopDisabled: document.getElementById('btn-stop').disabled,
            modalHidden: document.getElementById('preempt-modal').hidden,
          }));
          assert(st.wsOpen && !st.live && st.preemptDisabled && !st.stopDisabled, `6(d) failed: ${JSON.stringify(st)}`);
          const mStale = wsRoute.mark();
          if (!st.modalHidden) await page.click('#preempt-confirm', { force: true });
          await page.waitForTimeout(150);
          assert.equal(wsRoute.byType('acquire_lease', mStale).length, 0);
          wsRoute.hooks.onServerMsg = null;
          await ext.releaseLease();
        });
        wsRoute.hooks.onServerMsg = null;
        await waitForCockpitLive(page, 4000);
        log('PASS 6(d): stale telemetry disabled #preempt-confirm (0 acquire_lease) and kept #btn-stop enabled.');

        // 6(e): Single-field malformed obs / independent epoch_sync & accepted-first equal-ts WS freshness
        log('--- Scenario 6(e): Single-field malformed obs/epoch_sync & equal-ts WS freshness ---');
        await waitFn(page, () => isLive() && S.obs && Number.isFinite(S.obsTs), null, 3000);
        let realObs = null;
        wsRoute.hooks.onServerMsg = (m) => { if (!realObs && Array.isArray(m?.positions)) realObs = m; return 'drop'; };
        await waitFor(() => realObs != null, { timeoutMs: 2000, label: '6(e) capture real obs' });
        await page.waitForTimeout(60);
        const beforeMal = await page.evaluate(() => ({ obsTs: S.obsTs, obsAt: S.obsAt, epoch: S.epoch, holder: S.obs?.lease_holder, state: S.state, pos0: S.obs?.positions?.[0] }));
        await page.waitForTimeout(100);
        const badPayloads = [
          { ...realObs, timestamp: beforeMal.obsTs + 0.1, positions: [] },
          { ...realObs, timestamp: beforeMal.obsTs + 0.2, safety_state: null },
          { ...realObs, timestamp: beforeMal.obsTs + 0.3, lease_holder: false },
          { ...realObs, timestamp: beforeMal.obsTs + 0.4, epoch_id: 'bad' },
          { ...realObs, timestamp: beforeMal.obsTs + 0.5, safety_state: { Fault: { JointLimitExceeded: { joint_name: 'j1', value: { toString: null } } } } },
          { type: 'epoch_sync', epoch_id: -1 },
          { type: 'epoch_sync', epoch_id: beforeMal.epoch + 50, safety_state: { Fault: null } },
        ];
        for (const payload of badPayloads) {
          wsRoute.current.ws.send(JSON.stringify(payload));
          await page.waitForTimeout(30);
          const cur = await page.evaluate(() => ({ obsTs: S.obsTs, obsAt: S.obsAt, epoch: S.epoch, holder: S.obs?.lease_holder, state: S.state, pos0: S.obs?.positions?.[0] }));
          assert.deepEqual(cur, beforeMal, `6(e): malformed payload mutated state: ${JSON.stringify(payload)}`);
          assert.equal(guards.pageErrors.length, 0);
        }

        // Inject ONE valid advancing frame first, confirm accepted, then send 5 equal-ts duplicates past CFG.staleMs
        const dupTs = beforeMal.obsTs + 0.05;
        const dupFrame = JSON.stringify({ ...realObs, timestamp: dupTs });
        const wsIdxDup = wsRoute.wsCount;
        wsRoute.current.ws.send(dupFrame);
        await waitFn(page, (ts) => S.obsTs === ts && isLive(), dupTs, 2000);
        const obsAt0 = await page.evaluate(() => S.obsAt);
        const staleWait = (await page.evaluate(() => CFG.staleMs)) + 400;
        const stepMs = Math.ceil(staleWait / 5);
        for (let i = 0; i < 5; i++) {
          await page.waitForTimeout(stepMs);
          wsRoute.current.ws.send(dupFrame);
        }
        const dupRes = await page.evaluate(() => ({ obsAt: S.obsAt, live: isLive(), wsOpen: S.ws?.readyState === 1 }));
        wsRoute.hooks.onServerMsg = null;
        assert(dupRes.obsAt === obsAt0 && !dupRes.live && dupRes.wsOpen && wsRoute.wsCount === wsIdxDup,
          `6(e): equal-ts duplicate extended freshness: ${JSON.stringify({ obsAt0, dupRes })}`);
        await waitForCockpitLive(page, 4000);
        guards.assertClean('Scenario 6');
        log('PASS 6(e): single-field malformed obs/epoch_sync rejected (0 pageerrors) & equal-ts duplicates did not extend S.obsAt.');
      } finally { await ctx.close(); }
    }
    await killArmd(armdMain); armdMain = null;

    /* =====================================================================
       SCENARIO 5: Modals, held keys, real drag, multistep Stop & WatchdogTimeout
       ===================================================================== */
    log(`=== Scenario 5: #take-modal, held keys, drag, Stop cancellation & WatchdogTimeout (port ${PORT_INTERACT}) ===`);
    armdInteract = await spawnArmd(PORT_INTERACT, ['--config', B601_CONFIG]);
    {
      const { ctx, page, guards, rec: wsRoute } = await newCockpit(browser, PORT_INTERACT);
      let injectGrasp = false;
      const baseHook = (m) => (injectGrasp && Array.isArray(m?.positions) ? JSON.stringify({ ...m, action: { id: 999, kind: 'grasp', state: 'running', phase: 'approach' } }) : 'forward');
      wsRoute.hooks.onServerMsg = baseHook;

      try {
        await page.goto(`http://127.0.0.1:${PORT_INTERACT}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);

        // 5(a) & 5(b): Preset & cwSearch unleased -> #take-modal + touch reachability
        log('--- Scenario 5(a)-(b): Preset & cwSearch unleased -> #take-modal ---');
        await ensureArmedUnleased(PORT_INTERACT, page);
        const m5a = wsRoute.mark();
        await page.click('button[data-preset="home"]');
        await waitShown(page, '#take-modal');
        await assertReachable(page, '#take-estop', '5(a)');
        await assertReachable(page, '#take-stop', '5(a)');
        await page.click('#take-cancel'); await waitHidden(page, '#take-modal');
        assert.equal(wsRoute.byType('acquire_lease', m5a).length, 0);
        assert.equal(wsRoute.byType('preset', m5a).length, 0);

        await ensureArmedUnleased(PORT_INTERACT, page);
        const m5b = wsRoute.mark();
        await page.evaluate(() => { cwSearch(); });
        await waitShown(page, '#take-modal'); await page.waitForTimeout(100);
        assert.equal(await page.evaluate(() => CWZ.running || !!CWZ.search?.running), false);
        assert.equal(wsRoute.byType('acquire_lease', m5b).length, 0);
        assert.equal(wsRoute.motionFrames(m5b).length, 0);
        await page.click('#take-cancel'); await waitHidden(page, '#take-modal');
        log('PASS 5(a)-(b): preset & cwSearch opened #take-modal (#take-estop/#take-stop reachable) with 0 acquire/motion.');

        // 5(c): Trusted held Escape on #take-modal & #preempt-modal (mocked grasp) + observer #grasp-stop warning
        log('--- Scenario 5(c): Held Escape without Stop + observer #grasp-stop warning ---');
        injectGrasp = true;
        await ensureArmedUnleased(PORT_INTERACT, page);
        await waitFn(page, () => graspRunningAction() != null, null, 2000);
        await page.click('button[data-preset="home"]');
        await waitShown(page, '#take-modal');
        await assertReachable(page, '#take-estop', '5(c) #take-modal');
        await assertReachable(page, '#take-stop', '5(c) #take-modal');
        await holdEscapeNoStop(page, wsRoute, '#take-modal', { label: '5(c) #take-modal' });

        await withExtClient(PORT_INTERACT, 'ext-esc', async (ext) => {
          await ext.acquireLease({ force: false, heartbeat: true });
          await waitShown(page, '#btn-preempt', 3000);
          await page.click('#btn-preempt');
          await waitShown(page, '#preempt-modal');
          await assertReachable(page, '#preempt-estop', '5(c) #preempt-modal');
          await assertReachable(page, '#preempt-stop', '5(c) #preempt-modal');
          await holdEscapeNoStop(page, wsRoute, '#preempt-modal', { label: '5(c) #preempt-modal' });

          await waitShown(page, '#grasp-stop');
          await clearToasts(page);
          const mGs = wsRoute.mark();
          await page.click('#grasp-stop'); await page.waitForTimeout(180);
          assert.equal(wsRoute.byType('acquire_lease', mGs).length, 0);
          assert.equal(wsRoute.commands('/api/v1/motion/stop', mGs).length, 0);
          const gs = await getToasts(page);
          assert(!/Grasp stopped|已停止抓取/i.test(gs.all) && /E-stop|急停/i.test(gs.warns), `5(c) #grasp-stop toast: ${JSON.stringify(gs)}`);
          await ext.releaseLease();
        });
        injectGrasp = false;
        log('PASS 5(c): held Escape sent 0 stop/toasts; #take-stop/#preempt-stop reachable; observer #grasp-stop warned.');

        // 5(d): Enter-repeat safety, Tab focus trap & Space E-Stop
        log('--- Scenario 5(d): #take-modal Enter-repeat, Tab trap & Space E-Stop ---');
        await ensureArmedUnleased(PORT_INTERACT, page);
        const m5d = wsRoute.mark();
        await page.focus('button[data-preset="home"]');
        await page.keyboard.down('Enter');
        await waitShown(page, '#take-modal');
        assert.equal(await page.evaluate(() => document.activeElement?.id), 'take-cancel');
        for (let i = 0; i < 3; i++) await page.keyboard.down('Enter');
        await page.keyboard.up('Enter');
        await page.waitForTimeout(120);
        assert.equal(await page.evaluate(() => !document.getElementById('take-modal').hidden && document.activeElement?.id === 'take-cancel'), true);
        assert.equal(wsRoute.byType('acquire_lease', m5d).length, 0);
        for (const k of ['Tab', 'Tab', 'Tab', 'Tab', 'Tab', 'Tab', 'Shift+Tab']) {
          await page.keyboard.press(k);
          assert.equal(await page.evaluate(() => document.getElementById('take-modal').contains(document.activeElement)), true, `5(d): ${k} escaped trap`);
        }
        await page.focus('#take-confirm');
        await page.keyboard.press('Space');
        await waitFn(page, () => S.state === 'EStop' && document.getElementById('take-modal').hidden, null, 3000);
        assert.equal((await getObs(PORT_INTERACT)).safety_state, 'EStop');
        assert.equal(wsRoute.byType('acquire_lease', m5d).length, 0);
        log('PASS 5(d): Enter-repeat stayed on #take-cancel, Tab trapped, Space latched E-Stop.');

        // 5(d2): Held Enter across #take-confirm + simultaneous Enter/NumpadEnter isolation (held grant) + Space while suppressed
        log('--- Scenario 5(d2): Held Enter across #take-confirm & simultaneous Enter+NumpadEnter isolation ---');
        await ensureArmedUnleased(PORT_INTERACT, page);
        await page.focus('button[data-preset="home"]'); await page.keyboard.press('Enter');
        await waitShown(page, '#take-modal'); await page.focus('#take-confirm');
        const mHeldEnt = wsRoute.mark();
        await holdKeyRepeat(page, 'Enter', 3, () => waitFn(page, () => S.leaseGeneration != null && document.getElementById('take-modal').hidden));
        await page.waitForTimeout(200);
        assert.equal(wsRoute.byType('acquire_lease', mHeldEnt).length, 1);
        assert.equal(wsRoute.motionFrames(mHeldEnt).length, 0, '5(d2): held Enter replayed preset/motion!');
        const mFreshEnt = wsRoute.mark();
        await page.focus('button[data-preset="home"]'); await page.keyboard.press('Enter');
        await waitFor(() => wsRoute.byType('preset', mFreshEnt).length > 0, { timeoutMs: 2000, label: '5(d2) fresh Enter preset' });

        await page.evaluate(() => { S.ws.close(); });
        await waitForCockpitLive(page, 6000);
        await ensureArmedUnleased(PORT_INTERACT, page);
        await page.focus('button[data-preset="home"]'); await page.keyboard.press('Enter');
        await waitShown(page, '#take-modal');
        await holdSimultaneousConfirm(page, wsRoute, '#take-confirm', '#take-modal', { baseHook, testSpaceEstop: true, expectForce: false, label: '5(d2) #take-modal' });
        await page.click('#btn-enable'); await waitFn(page, () => S.state === 'Idle');
        await page.waitForTimeout(380);
        await holdButton(page, '#btn-enable', 750); await waitFn(page, () => S.state === 'Armed' && S.leaseGeneration != null, null, 5000);
        const mFreshNum = wsRoute.mark();
        await page.focus('button[data-preset="home"]'); await page.keyboard.press('NumpadEnter');
        await waitFor(() => wsRoute.byType('preset', mFreshNum).length > 0, { timeoutMs: 2000, label: '5(d2) fresh NumpadEnter preset' });
        log('PASS 5(d2): held Enter & simultaneous Enter+NumpadEnter isolated (marked before press); Space latched E-Stop; fresh keys worked.');

        // 5(e): Pure real browser pointer drag & held ArrowRight repeat on #grip-range (marked before confirm)
        log('--- Scenario 5(e): Real pointer drag & ArrowRight repeat on #grip-range ---');
        if (await page.evaluate(() => S.ui !== 'expert')) await page.click('#ui-seg button[data-ui="expert"]');
        await page.click('#mode-seg button[data-mode="cart"]');
        await page.waitForSelector('#grip-range', { state: 'visible', timeout: 3000 });
        await page.evaluate(() => { S.ws.close(); });
        await waitForCockpitLive(page, 6000);
        await ensureArmedUnleased(PORT_INTERACT, page);
        await assertControlInputsState(page, 'unavailable', 'Scenario 5(e) expert cart unleased');
        await page.click('#input-seg button[data-input="stick"]');
        await assertControlInputsState(page, 'unavailable', 'Scenario 5(e) expert stick unleased');
        await page.click('#input-seg button[data-input="btn"]');
        const gripLoc = page.locator('#grip-range');
        await gripLoc.scrollIntoViewIfNeeded();
        const gripBoxUnleased = await gripLoc.boundingBox();
        const m5eOpen = wsRoute.mark();
        await dragSlider(page, gripBoxUnleased, 0.2, 0.65);
        await waitShown(page, '#take-modal');
        assert.equal(wsRoute.byType('acquire_lease', m5eOpen).length, 0);
        assert.equal(wsRoute.motionFrames(m5eOpen).length, 0);

        const m5e = wsRoute.mark();
        await page.keyboard.down('ArrowRight');
        await page.click('#take-confirm');
        await waitFn(page, () => S.leaseGeneration != null && document.getElementById('take-modal').hidden);
        await assertControlInputsState(page, 'active', 'Scenario 5(e) expert cart active');
        await page.click('#input-seg button[data-input="stick"]');
        await assertControlInputsState(page, 'active', 'Scenario 5(e) expert stick active');
        await page.click('#input-seg button[data-input="btn"]');
        assert.equal(wsRoute.byType('acquire_lease', m5e).length, 1);
        await page.focus('#grip-range');
        for (let i = 0; i < 3; i++) await page.keyboard.down('ArrowRight');
        await page.keyboard.up('ArrowRight');
        await page.waitForTimeout(350);
        assert.equal(wsRoute.motionFrames(m5e).length, 0, '5(e): held ArrowRight repeat across confirm emitted motion!');

        await gripLoc.scrollIntoViewIfNeeded();
        const gripBoxLeased = await gripLoc.boundingBox();
        const m5eFresh = wsRoute.mark();
        await dragSlider(page, gripBoxLeased, 0.25, 0.85);
        const servo5e = await waitFor(() => wsRoute.motionFrames(m5eFresh).find((f) => Array.isArray(f.json?.positions) && f.json.positions[6] < -0.5),
          { timeoutMs: 2000, label: '5(e) fresh real mouse drag gripper servo' });
        assert(servo5e.json.positions[6] < -0.5, `5(e): unexpected gripper target ${servo5e.json.positions[6]}`);
        log(`PASS 5(e): real unleased drag + held ArrowRight repeat sent 0 motion; fresh real drag sent servo (J7=${servo5e.json.positions[6].toFixed(2)}).`);

        // 5(e2): cwSearch Stop cancellation (with confirmed Stop ACK) & wizard hidden-tab single Stop through completion
        log('--- Scenario 5(e2): cwSearch Stop cancellation & wizard hidden-tab single Stop ---');
        const m5e2Pre = wsRoute.mark(), r5e2Pre = wsRoute.markRecv();
        await page.click('#btn-stop');
        await waitCommandAck(wsRoute, '/api/v1/motion/stop', m5e2Pre, r5e2Pre, '5(e2) pre-stop');
        await waitFn(page, () => S.state === 'Armed' && !S.streamActive && S.leaseGeneration != null, null, 5000);
        const mCw = wsRoute.mark(), rCw = wsRoute.markRecv();
        await withDelayedGraspSetup(page, async () => {
          await page.click('#btn-stop');
          await waitCommandAck(wsRoute, '/api/v1/motion/stop', mCw, rCw, '5(e2) cwSearch Stop');
        });
        await page.waitForTimeout(250);
        assert.equal(wsRoute.commands('/api/v1/action/move_to', mCw).length, 0);
        assert.equal(wsRoute.motionFrames(mCw).length, 0);

        const mHide = wsRoute.mark(), rHide = wsRoute.markRecv();
        await withDelayedGraspSetup(page, async () => {
          await page.evaluate(() => { Object.defineProperty(document, 'hidden', { configurable: true, get: () => true }); document.dispatchEvent(new Event('visibilitychange')); });
          await waitCommandAck(wsRoute, '/api/v1/motion/stop', mHide, rHide, '5(e2) hidden-tab Stop');
          await page.waitForTimeout(150);
          assert.equal(wsRoute.commands('/api/v1/motion/stop', mHide).length, 1, '5(e2): expected exact-once Stop while hidden');
          await page.evaluate(() => { delete document.hidden; document.dispatchEvent(new Event('visibilitychange')); });
        });
        await page.waitForTimeout(200);
        assert.equal(wsRoute.commands('/api/v1/motion/stop', mHide).length, 1, '5(e2): extra Stop emitted after cwSearch exit!');
        log('PASS 5(e2): Stop ACK cancelled cwSearch (0 move_to); hidden-tab emitted exact-once /api/v1/motion/stop through completion.');

        // 5(e3): teachPlayPicked Stop (valid sequence.id + Stop ACK) & #cwiz-stop during cwvStart dry-run (top-level after + Stop ACK)
        log('--- Scenario 5(e3): teachPlayPicked Stop & #cwiz-stop during cwvStart dry-run ---');
        await page.evaluate(() => teachSavePose('p01', true));
        await waitFn(page, () => TCH.poses.length > 0 && TCH.pick.length > 0, null, 5000);
        let saveReqId = null, heldSaveAck = null;
        wsRoute.hooks.onClientMsg = (m) => { if (m?.type === 'command' && m.path === '/api/v1/sequences/save') saveReqId = m.request_id; return 'forward'; };
        wsRoute.hooks.onServerMsg = (m, raw, pair) => (m?.type === 'command_ack' && saveReqId != null && m.request_id === saveReqId && !heldSaveAck ? (heldSaveAck = { m, raw, pair }, 'drop') : baseHook(m));
        const mTeach = wsRoute.mark(), rTeach = wsRoute.markRecv();
        await page.evaluate(() => { teachPlayPicked(false); });
        await waitFor(() => heldSaveAck != null, { timeoutMs: 5000, label: '5(e3) sequences/save ack' });
        assert(typeof heldSaveAck.m?.sequence?.id === 'string' && heldSaveAck.m.sequence.id.length > 0,
          `5(e3): sequences/save ack missing non-empty sequence.id: ${JSON.stringify(heldSaveAck.m)}`);
        await page.click('#btn-stop');
        await waitCommandAck(wsRoute, '/api/v1/motion/stop', mTeach, rTeach, '5(e3) teach Stop');
        wsRoute.hooks.onClientMsg = null; wsRoute.hooks.onServerMsg = baseHook;
        heldSaveAck.pair.ws.send(heldSaveAck.raw);
        await waitFn(page, () => !TCH.busy, null, 5000); await page.waitForTimeout(250);
        assert.equal(wsRoute.commands('/api/v1/action/run_sequence', mTeach).length, 0);

        await page.evaluate(() => commandRPC('/api/v1/torque', { enable: false }));
        await waitFn(page, () => S.state === 'Idle' && !S.obs?.is_torqued && S.leaseGeneration != null);
        let dryReq = null;
        wsRoute.hooks.onClientMsg = (m, raw, pair) => (m?.type === 'command' && m.path === '/api/v1/calibration/apply' && m.body?.dry_run === true ? (dryReq = { request_id: m.request_id, pair }, 'drop') : 'forward');
        const mCwv = wsRoute.mark(), rCwv = wsRoute.markRecv();
        await page.evaluate(() => {
          CWZ.open = true; CWZ.step = 'hover'; CWZ.busy = false; CWZ.solve = { apply: { camera_mount: {} } };
          CWV.fp = 'fp-1'; CWV.tool = null; CWV.job = null;
          document.getElementById('cwiz').hidden = false; cwRender(); cwvStart(false, 'job-1');
        });
        await waitFor(() => dryReq != null, { timeoutMs: 4000, label: '5(e3) cwvStart dry_run' });
        await waitShown(page, '#cwiz-stop');
        await page.click('#cwiz-stop');
        const cwvStopAck = await waitCommandAck(wsRoute, '/api/v1/motion/stop', mCwv, rCwv, '5(e3) #cwiz-stop');
        wsRoute.hooks.onClientMsg = null;
        const curEpoch = await page.evaluate(() => S.epoch);
        dryReq.pair.ws.send(JSON.stringify({ type: 'command_ack', request_id: dryReq.request_id, after: { fingerprint: 'fp-1' }, epoch_id: cwvStopAck.ack.epoch_id ?? curEpoch }));
        await waitFn(page, () => !CWZ.busy); await page.waitForTimeout(250);
        assert.equal(wsRoute.commands('/api/v1/torque', mCwv).length, 0);
        assert.equal(wsRoute.commands('/api/v1/action/move_to', mCwv).length, 0);
        await page.evaluate(() => { CWZ.open = false; document.getElementById('cwiz').hidden = true; });
        log('PASS 5(e3): teachPlayPicked Stop (valid seq id + Stop ACK) & #cwiz-stop (top-level after + Stop ACK) prevented follow-up motion.');

        // 5(e4): Per-gesture Stop (e.detail===0), active-grasp single Stop + ACK + GR.stopAt===0, & retry preserve/supersede with real ACKs
        log('--- Scenario 5(e4): Per-gesture Stop, active-grasp Stop ACK & retry preserve/supersede ---');
        await page.evaluate(() => commandRPC('/api/v1/torque', { enable: true }));
        await waitFn(page, () => S.state === 'Armed' && S.leaseGeneration != null);
        await page.evaluate(() => {
          const b = document.getElementById('btn-stop');
          b.dispatchEvent(new PointerEvent('pointerdown', { bubbles: true, cancelable: true, pointerId: 9 }));
          b.dispatchEvent(new PointerEvent('pointercancel', { bubbles: true, cancelable: true, pointerId: 9 }));
        });
        const mKeyStop = wsRoute.mark(), rKeyStop = wsRoute.markRecv();
        await page.focus('#btn-stop'); await page.keyboard.press('Enter');
        await waitCommandAck(wsRoute, '/api/v1/motion/stop', mKeyStop, rKeyStop, '5(e4) keyboard Enter Stop');

        injectGrasp = true;
        await waitFn(page, () => graspRunningAction() != null, null, 2000);
        for (const [label, trigger] of [
          ['keyboard Enter', async () => { await page.focus('#btn-stop'); await page.keyboard.press('Enter'); }],
          ['pointer click', async () => { await page.click('#btn-stop'); }],
        ]) {
          await page.evaluate(() => { GR.stopAt = 0; });
          const mG = wsRoute.mark(), rG = wsRoute.markRecv();
          await trigger();
          await waitCommandAck(wsRoute, '/api/v1/motion/stop', mG, rG, `5(e4) active grasp ${label}`);
          await page.waitForTimeout(200);
          assert.equal(wsRoute.commands('/api/v1/motion/stop', mG).length, 1, `5(e4): active grasp + ${label} must emit exact-once Stop`);
          assert.equal(await page.evaluate(() => GR.stopAt), 0, `5(e4): active grasp + ${label} triggered graspStop()`);
        }
        injectGrasp = false;

        const droppedStopIds = new Set();
        let dropStops = true;
        wsRoute.hooks.onClientMsg = (m) => { if (dropStops && m?.type === 'command' && m.path === '/api/v1/motion/stop') droppedStopIds.add(m.request_id); return 'forward'; };
        wsRoute.hooks.onServerMsg = (m) => (m?.type === 'command_ack' && droppedStopIds.has(m.request_id) ? 'drop' : baseHook(m));
        const mKeep = wsRoute.mark(), rKeep = wsRoute.markRecv();
        await page.click('#btn-stop');
        await waitFor(() => wsRoute.commands('/api/v1/motion/stop', mKeep).length === 1, { timeoutMs: 2000, label: '5(e4) 1st stop' });
        await page.waitForTimeout(150);
        await page.evaluate(() => commandRPC('/api/v1/poses/save', { name: 'p_keep' }));
        await waitCommandAck(wsRoute, '/api/v1/poses/save', mKeep, rKeep, '5(e4) poses/save');
        await waitFor(() => wsRoute.commands('/api/v1/motion/stop', mKeep).length >= 2, { timeoutMs: 3200, label: '5(e4) retry preserved across poses/save' });
        dropStops = false; await page.waitForTimeout(200);

        droppedStopIds.clear(); dropStops = true;
        const mSuper = wsRoute.mark(), rSuper = wsRoute.markRecv();
        const pos0Before = await page.evaluate(() => S.obs.positions[0]);
        await page.click('#btn-stop');
        await waitFor(() => wsRoute.commands('/api/v1/motion/stop', mSuper).length === 1, { timeoutMs: 2000, label: '5(e4) 1st stop superseded' });
        await page.waitForTimeout(150);
        await page.evaluate((t0) => { const p = S.obs.positions.slice(0, S.dof); p[0] = t0; return commandRPC('/api/v1/motion/jog', { target_positions: p, epoch_id: S.epoch }); }, pos0Before + 0.15);
        await waitCommandAck(wsRoute, '/api/v1/motion/jog', mSuper, rSuper, '5(e4) superseding jog');
        await waitFn(page, (p0) => Math.abs(S.obs.positions[0] - p0) > 0.05, pos0Before, 3000);
        await page.waitForTimeout(2400);
        assert.equal(wsRoute.commands('/api/v1/motion/stop', mSuper).length, 1, '5(e4): motion command did not supersede Stop retry!');
        wsRoute.hooks.onClientMsg = null; wsRoute.hooks.onServerMsg = baseHook;
        log('PASS 5(e4): per-gesture Stop, active-grasp exact-once Stop+ACK (GR.stopAt=0) & retry preserve/supersede with real ACKs verified.');
        guards.assertClean('Scenario 5');
      } finally { await ctx.close(); }
    }
    await killArmd(armdInteract); armdInteract = null;

    // 5(f): WatchdogTimeout first-tap Clear fault on bare mock
    log(`--- Scenario 5(f): First-tap Clear fault after WatchdogTimeout on bare mock (port ${PORT_INTERACT}) ---`);
    armdInteract = await spawnArmd(PORT_INTERACT);
    await withExtClient(PORT_INTERACT, 'ext-wd-fault', async (ext) => {
      await ext.acquireLease({ force: false, heartbeat: true });
      await ext.command('/api/v1/torque', { enable: true });
      await waitFor(async () => { const o = await getObs(PORT_INTERACT); return o.is_torqued && o.safety_state === 'Armed'; }, { timeoutMs: 4000, label: '5(f) torque on' });
    });
    await waitFor(async () => { const o = await getObs(PORT_INTERACT); return o.lease_holder === null && o.fault?.code === 'WATCHDOG_TIMEOUT' && o.safety_state?.Fault === 'WatchdogTimeout'; }, { timeoutMs: 7000, intervalMs: 80, label: '5(f) WatchdogTimeout latched' });
    {
      const { ctx, page, guards, rec } = await newCockpit(browser, PORT_INTERACT);
      try {
        await page.goto(`http://127.0.0.1:${PORT_INTERACT}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        await waitFn(page, () => S.state === 'Fault' && S.faultCode === 'WATCHDOG_TIMEOUT' && document.getElementById('btn-enable').classList.contains('clear') && !document.getElementById('btn-enable').disabled);
        await page.waitForTimeout(220);
        const clientId = await page.evaluate(() => CLIENT_ID);
        assert.equal(rec.byType('acquire_lease').length, 0);

        const m5f = rec.mark();
        await page.click('#btn-enable');
        await waitFn(page, () => S.state === 'Idle' && S.faultCode === null && S.leaseGeneration != null, null, 5000);
        await page.waitForTimeout(180);
        const acq5f = rec.byType('acquire_lease', m5f);
        assert.equal(acq5f.length, 1); assert.equal(acq5f[0].json.force, false);
        assert.equal(rec.commands('/api/v1/safety/clear_fault', m5f).length, 1);
        const acqIdx = rec.sent.findIndex((f, i) => i >= m5f && f.json?.type === 'acquire_lease');
        const clearIdx = rec.sent.findIndex((f, i) => i >= m5f && f.json?.type === 'command' && f.json?.path === '/api/v1/safety/clear_fault');
        const hbIdx = rec.sent.findIndex((f, i) => i > acqIdx && i < clearIdx && f.json?.type === 'heartbeat');
        assert(acqIdx >= m5f && hbIdx > acqIdx && clearIdx > hbIdx, `5(f): bad order acq=${acqIdx}, hb=${hbIdx}, clear=${clearIdx}`);
        const obs5f = await getObs(PORT_INTERACT);
        assert(obs5f.safety_state === 'Idle' && obs5f.fault === null && obs5f.lease_holder === clientId);
        assert(!/CLEAR_FAULT_REJECTED|Fresh heartbeat/i.test((await getToasts(page)).all));
        guards.assertClean('Scenario 5(f)');
        log(`PASS 5(f): first-tap Clear fault sent acquire_lease (${acqIdx}) -> heartbeat (${hbIdx}) -> clear_fault (${clearIdx}), cleared to Idle.`);
      } finally { await ctx.close(); }
    }
    await killArmd(armdInteract); armdInteract = null;

    /* =====================================================================
       SCENARIO 7: Auth-enforced PIN, held keys, cooldown & observer Stop/E-Stop
       ===================================================================== */
    log(`=== Scenario 7: PIN safety, auth-enforced takeover & observer Stop/E-Stop (port ${PORT_AUTH}) ===`);
    armdAuth = await spawnArmd(PORT_AUTH, ['--operator-pin', '123456']);
    {
      const { ctx, page, guards, rec } = await newCockpit(browser, PORT_AUTH);
      let extMcp = null, extOther = null, extObs = null;
      try {
        await page.goto(`http://127.0.0.1:${PORT_AUTH}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        await waitFn(page, () => S.auth.enforced === true && S.auth.role !== 'operator', null, 5000);
        const clientId = await page.evaluate(() => CLIENT_ID);

        // 7(a0): #op-modal #op-stop/#op-estop reachability, held Escape & Space in #op-pin
        log('--- Scenario 7(a0): #op-modal #op-stop reachability, held Escape & Space E-Stop ---');
        await page.click('#btn-op'); await waitShown(page, '#op-modal');
        await assertReachable(page, '#op-estop', '7(a0) #op-modal');
        await assertReachable(page, '#op-stop', '7(a0) #op-modal');
        await holdEscapeNoStop(page, rec, '#op-modal', { label: '7(a0) #op-modal' });

        await page.click('#btn-op');
        await waitFn(page, () => !document.getElementById('op-modal').hidden && document.activeElement?.id === 'op-pin', null, 2000);
        const m7a0 = rec.mark();
        await page.keyboard.press('Space');
        await waitFn(page, () => S.state === 'EStop', null, 3000);
        assert.equal((await getObs(PORT_AUTH)).safety_state, 'EStop');
        assert.equal(rec.byType('acquire_lease', m7a0).length, 0);
        if (await page.evaluate(() => !document.getElementById('op-modal').hidden)) { await page.click('#op-cancel'); await waitHidden(page, '#op-modal'); }
        await clearEstopExternal(PORT_AUTH, { pin: '123456' });
        await waitFn(page, () => S.state === 'Idle');
        log('PASS 7(a0): #op-stop/#op-estop reachable, held Escape sent 0 stop, Space in #op-pin latched E-Stop.');

        // 7(a): PIN cancellation across epoch change & held Enter on PIN unlock
        log('--- Scenario 7(a): PIN cancellation across epoch change & held Enter on unlock ---');
        await page.click('#btn-estop');
        await waitFn(page, () => S.state === 'EStop' && !document.getElementById('op-inline').hidden);
        const epochBefore = await page.evaluate(() => S.epoch);
        await page.click('#op-inline'); await waitShown(page, '#op-modal');
        await withExtClient(PORT_AUTH, 'ext-estop', async (ext) => { ext.eStop(); await waitFn(page, (p) => S.epoch > p, epochBefore); });
        const m7a = rec.mark();
        await page.fill('#op-pin', '123456');
        await holdKeyRepeat(page, 'Enter', 2, () => waitFn(page, () => document.getElementById('op-modal').hidden && S.auth.role === 'operator', null, 5000));
        await page.waitForTimeout(450);
        assert(rec.byType('acquire_lease', m7a).length === 0 && rec.commands('/api/v1/safety/clear_fault', m7a).length === 0 && rec.motionFrames(m7a).length === 0);
        assert.equal((await getObs(PORT_AUTH)).safety_state, 'EStop');
        log('PASS 7(a): epoch advance invalidated #op-inline PIN; held Enter across unlock sent 0 acquire/clear/motion.');

        await clearEstopExternal(PORT_AUTH, { pin: '123456' });
        await waitFn(page, () => S.state === 'Idle');
        await page.click('#btn-op'); await waitShown(page, '#op-lock', 3000);
        await page.click('#op-lock'); await waitFn(page, () => S.auth.role !== 'operator' && document.getElementById('op-modal').hidden);

        // 7(b): Auth-enforced takeover, 800 ms reaction duration, unauth rejection, real cooldown & simultaneous Enter+NumpadEnter
        log('--- Scenario 7(b): Auth takeover, 800 ms window, unauth rejection, cooldown & simultaneous Enter+NumpadEnter ---');
        const m7bStart = rec.mark();
        extMcp = await new ExternalWsClient(PORT_AUTH, 'rebot-mcp').connect();
        await extMcp.acquireLease({ force: false, heartbeat: true });
        await waitFn(page, () => document.getElementById('lease-status-txt').textContent.includes('rebot-mcp')
          && document.getElementById('preempt-who').textContent.includes('rebot-mcp') && !document.getElementById('btn-preempt').hidden && document.getElementById('btn-enable').disabled);
        assert.equal(rec.byType('acquire_lease', m7bStart).length, 0);

        await page.click('#btn-preempt'); await waitShown(page, '#preempt-modal');
        assert((await page.textContent('#preempt-body')).includes('rebot-mcp') && await page.evaluate(() => document.getElementById('preempt-changed').hidden));

        const opToken = await unlockOperatorToken(PORT_AUTH, '123456');
        extOther = await new ExternalWsClient(PORT_AUTH, 'web-other9').connect();
        await extOther.auth(opToken);
        const tChange = Date.now();
        await extOther.acquireLease({ force: true, heartbeat: true });
        await waitFn(page, () => !document.getElementById('preempt-modal').hidden
          && document.getElementById('preempt-body').textContent.includes('web-other9')
          && !document.getElementById('preempt-changed').hidden && document.getElementById('preempt-changed').textContent.includes('web-other9')
          && document.getElementById('lease-status-txt').textContent.includes('web-other9'));

        const mReact = rec.mark();
        assert.equal(await page.evaluate(() => document.getElementById('preempt-confirm').disabled), true);
        await page.click('#preempt-confirm', { force: true }); await page.waitForTimeout(80);
        assert(rec.byType('acquire_lease', mReact).length === 0 && await page.evaluate(() => document.getElementById('op-modal').hidden));
        await extMcp.close(); extMcp = null;
        const rem450 = 450 - (Date.now() - tChange);
        if (rem450 > 0) await page.waitForTimeout(rem450);
        assert.equal(await page.evaluate(() => document.getElementById('preempt-confirm').disabled), true, '7(b): re-enabled before 450 ms!');
        await waitFn(page, () => !document.getElementById('preempt-confirm').disabled, null, 2500);
        const dtReEnable = Date.now() - tChange;
        assert(dtReEnable >= 750, `7(b): reaction window too short (${dtReEnable} ms)`);

        const mPin = rec.mark();
        await page.click('#preempt-confirm'); await waitShown(page, '#op-modal');
        await page.fill('#op-pin', '123456');
        await holdKeyRepeat(page, 'Enter', 2, () => waitFn(page, () => document.getElementById('op-modal').hidden && S.auth.role === 'operator', null, 5000));
        await page.waitForTimeout(350);
        assert(rec.byType('acquire_lease', mPin).length === 0 && rec.motionFrames(mPin).length === 0);

        assert(Date.now() - tChange < 2800, '7(b): missed 3 s server cooldown window');
        const mCool = rec.mark(), recvCool = rec.markRecv();
        await page.click('#preempt-confirm');
        const coolReq = await waitFor(() => rec.byType('acquire_lease', mCool)[0], { timeoutMs: 2000, label: '7(b) cooldown acquire_lease' });
        await waitFor(() => rec.received.slice(recvCool).some((f) => f.json?.type === 'lease_rejected' && f.json?.error === 'preemption_cooldown' && f.json?.seq === coolReq.json.seq),
          { timeoutMs: 3000, label: '7(b) correlated preemption_cooldown' });
        await waitFn(page, () => !document.getElementById('preempt-cooldown').hidden && document.getElementById('preempt-confirm').disabled, null, 2000);

        // Server-side auth enforcement: unauthenticated force takeover rejected with OPERATOR_REQUIRED
        await withExtClient(PORT_AUTH, 'ext-unauth', async (unauth) => {
          const rej = await unauth.sendAcquireRaw({ force: true });
          assert(rej.type === 'lease_rejected' && rej.code === 'OPERATOR_REQUIRED', `7(b) unauth takeover: ${JSON.stringify(rej)}`);
          assert.equal((await getObs(PORT_AUTH)).lease_holder, 'web-other9');
        });

        await waitFn(page, () => document.getElementById('preempt-cooldown').hidden && !document.getElementById('preempt-confirm').disabled, null, 5000);
        await page.waitForTimeout(300);
        assert.equal(rec.byType('acquire_lease', mCool).length, 1, '7(b): cooldown auto-retried before explicit confirm!');

        await holdSimultaneousConfirm(page, rec, '#preempt-confirm', '#preempt-modal', { expectForce: true, label: '7(b) #preempt-modal' });
        await waitFn(page, () => document.getElementById('lease-status').className.includes('is-own'), null, 3000);
        await waitFor(async () => (await getObs(PORT_AUTH)).lease_holder === clientId, { timeoutMs: 3000, label: '7(b) server holder === CLIENT_ID' });
        await extOther.close(); extOther = null;
        log(`PASS 7(b): reaction window (${dtReEnable} ms), PIN, unauth OPERATOR_REQUIRED, correlated cooldown (0 auto-retry) & simultaneous Enter+NumpadEnter verified (holder=${clientId}).`);

        // 7(c): Observer Stop warning + Observer E-Stop
        log('--- Scenario 7(c): Observer Stop warning & Observer E-Stop ---');
        await page.waitForTimeout(3250);
        extObs = await new ExternalWsClient(PORT_AUTH, 'ext-obs').connect();
        await extObs.auth(opToken);
        await extObs.acquireLease({ force: true, heartbeat: true });
        await extObs.command('/api/v1/torque', { enable: true });
        await waitFn(page, () => S.leaseGeneration === null && S.obs?.lease_holder === 'ext-obs' && S.state === 'Armed' && S.obs?.is_torqued === true && document.getElementById('lease-status').className.includes('is-other') && document.getElementById('enable-txt')?.textContent?.trim() === 'Torque on · not in control', null, 5000);
        assert.equal(await page.evaluate(() => document.getElementById('enable-txt')?.textContent?.trim()), 'Torque on · not in control');
        assert.equal(await page.evaluate(() => document.getElementById('btn-enable').disabled), true);
        await assertControlInputsState(page, 'unavailable', 'Scenario 7(c) observer armed');

        await clearToasts(page);
        assert.equal(await page.evaluate(() => document.getElementById('btn-stop').disabled), false);
        const m7cStop = rec.mark();
        await page.click('#btn-stop'); await page.waitForTimeout(200);
        assert(rec.byType('acquire_lease', m7cStop).length === 0 && rec.commands('/api/v1/motion/stop', m7cStop).length === 0);
        const st7c = await getToasts(page);
        assert(!/Stopped, holding position|已停止，位置保持/i.test(st7c.all) && /E-stop|急停/i.test(st7c.warns), `7(c) observer Stop toast: ${JSON.stringify(st7c)}`);

        const m7c = rec.mark();
        await page.click('#btn-estop');
        await waitFn(page, () => S.state === 'EStop', null, 3000);
        assert.equal((await getObs(PORT_AUTH)).safety_state, 'EStop');
        assert.equal(rec.byType('acquire_lease', m7c).length, 0);
        guards.assertClean('Scenario 7');
        log('PASS 7(c): observer Stop warned toward E-Stop; observer E-Stop latched EStop with 0 acquire_lease.');
      } finally {
        if (extMcp) await extMcp.close();
        if (extOther) await extOther.close();
        if (extObs) await extObs.close();
        await ctx.close();
      }
    }
    await killArmd(armdAuth); armdAuth = null;

    /* =====================================================================
       SCENARIO 8: Phone/tablet header fit, favicon 404 check & park-off release-control state
       ===================================================================== */
    log(`=== Scenario 8: Phone/tablet header fit, favicon & park-off #tools-release (port ${PORT_AUTH}) ===`);
    armdAuth = await spawnArmd(PORT_AUTH, ['--config', B601_CONFIG, '--operator-pin', '123456']);
    {
      const favRes = await fetch(`http://127.0.0.1:${PORT_AUTH}/favicon.ico`, { cache: 'no-store' });
      assert.notEqual(favRes.status, 404, `GET /favicon.ico must not 404 (got ${favRes.status})`);

      const viewports = [
        { width: 360, height: 780, label: '360x780' },
        { width: 390, height: 844, label: '390x844' },
        { width: 1024, height: 768, label: '1024x768' },
      ];
      for (const vp of viewports) {
        for (const lang of ['en', 'zh']) {
          const { ctx, page, guards } = await newCockpit(browser, PORT_AUTH, {
            width: vp.width, height: vp.height, locale: lang === 'zh' ? 'zh-CN' : 'en-US',
          });
          const http404 = [];
          page.on('response', (res) => { if (res.status() === 404) http404.push(res.url()); });
          try {
            await page.goto(`http://127.0.0.1:${PORT_AUTH}/`, { waitUntil: 'domcontentloaded' });
            await waitForCockpitLive(page);
            await waitFn(page, () => S.auth?.enforced === true && !document.getElementById('btn-op')?.hidden, null, 5000);
            for (const mode of ['simple', 'expert']) {
              await page.click(`#ui-seg button[data-ui="${mode}"]`);
              await page.waitForTimeout(180);
              const metrics = await page.evaluate((m) => {
                const tb = document.querySelector('.topbar');
                const lb = document.getElementById('btn-lang');
                const seg = document.getElementById('ui-seg');
                const visHeight = (el) => {
                  if (!el || el.hidden) return 0;
                  const cs = getComputedStyle(el);
                  if (cs.display === 'none' || cs.visibility === 'hidden') return 0;
                  const b = el.getBoundingClientRect();
                  return b.width > 0 && b.height > 0 ? Math.round(b.height) : 0;
                };
                const refCandidates = m === 'simple'
                  ? [['#speed-pill', document.getElementById('speed-pill')]]
                  : [
                      ['#ex-ph-status', document.getElementById('ex-ph-status')],
                      ['.brand-mark', tb.querySelector('.brand-mark')],
                    ];
                let refLabel = 'none', refHeight = 0;
                for (const [label, el] of refCandidates) {
                  const h = visHeight(el);
                  if (h > 0) { refLabel = label; refHeight = h; break; }
                }
                const r = lb ? lb.getBoundingClientRect() : null;
                const segR = seg ? seg.getBoundingClientRect() : null;
                const tbR = tb ? tb.getBoundingClientRect() : null;
                const segBtns = seg ? [...seg.querySelectorAll('button')].map((b) => ({
                  ui: b.dataset.ui,
                  text: b.textContent.trim(),
                  on: b.classList.contains('on'),
                  checked: b.getAttribute('aria-checked'),
                })) : [];
                const clipped = [...tb.querySelectorAll(':scope > *, .head-stats > *')].filter((el) => {
                  const cs = getComputedStyle(el);
                  if (cs.display === 'none' || el.hidden) return false;
                  const b = el.getBoundingClientRect();
                  return b.width > 0 && (b.left < 0 || b.right > window.innerWidth + 0.5);
                }).map((el) => el.id || el.className);
                return {
                  innerWidth: window.innerWidth,
                  scrollWidth: document.documentElement.scrollWidth,
                  langRect: r ? { left: r.left, right: r.right, top: r.top, bottom: r.bottom, width: r.width, height: r.height } : null,
                  hasOldBtnUi: !!document.getElementById('btn-ui'),
                  segHeight: segR ? Math.round(segR.height) : 0,
                  segMidY: segR ? segR.top + segR.height / 2 : 0,
                  langMidY: r ? r.top + r.height / 2 : 0,
                  topbarHeight: tbR ? Math.round(tbR.height) : 0,
                  refLabel,
                  refHeight,
                  segBtns,
                  activeMode: S.ui,
                  clipped,
                };
              }, mode);
              const tag = `Scenario 8 ${vp.label} ${lang} ${mode}`;
              assert.equal(metrics.hasOldBtnUi, false, `${tag}: legacy #btn-ui must be removed`);
              assert.equal(metrics.activeMode, mode, `${tag}: clicking #ui-seg failed to switch to ${mode}`);
              assert.equal(metrics.segBtns.length, 2, `${tag}: #ui-seg must have 2 options`);
              const expectedLabels = lang === 'zh' ? ['简易', '专家'] : ['Simple', 'Expert'];
              assert.deepEqual(metrics.segBtns.map((b) => b.text), expectedLabels, `${tag}: unexpected #ui-seg labels`);
              assert(metrics.segBtns.every((b) => (b.ui === mode) === (b.on && b.checked === 'true')),
                `${tag}: #ui-seg active highlight mismatch: ${JSON.stringify(metrics.segBtns)}`);
              assert(metrics.segHeight >= 24 && metrics.segHeight <= 30 && metrics.refHeight > 0 && Math.abs(metrics.segHeight - metrics.refHeight) <= 4,
                `${tag}: #ui-seg height ${metrics.segHeight}px should match visible ${metrics.refLabel} height ${metrics.refHeight}px`);
              assert(metrics.topbarHeight <= 56 && Math.abs(metrics.segMidY - metrics.langMidY) <= 10,
                `${tag}: topbar must stay single-row (height=${metrics.topbarHeight}px, segMidY=${metrics.segMidY}, langMidY=${metrics.langMidY})`);
              assert.equal(metrics.scrollWidth, metrics.innerWidth, `${tag}: scrollWidth ${metrics.scrollWidth} !== innerWidth ${metrics.innerWidth}`);
              assert.equal(metrics.clipped.length, 0, `${tag}: clipped header items: ${JSON.stringify(metrics.clipped)}`);
              assert(metrics.langRect && metrics.langRect.left >= 0 && metrics.langRect.right <= metrics.innerWidth
                && metrics.langRect.top >= 0 && metrics.langRect.bottom <= vp.height,
                `${tag}: #btn-lang outside viewport: ${JSON.stringify(metrics.langRect)}`);
              await assertReachable(page, '#btn-lang', tag);

              // Open settings drawer: verify no floating controls overlap it and E-stop remains hit-testable immediately, settled, & scrolled
              if (mode === 'expert') {
                await page.click('#btn-ex-more');
                await page.waitForSelector('#ex-more:not([hidden])', { timeout: 2000 });
                await page.click('#ex-m-sset');
              } else {
                await page.click('#btn-sset');
              }
              await page.waitForSelector('#sset:not([hidden])', { timeout: 2000 });
              for (const settleLabel of ['immediate', 'settled', 'scrolled']) {
                if (settleLabel === 'settled') await page.waitForTimeout(260);
                if (settleLabel === 'scrolled') {
                  await page.evaluate(() => {
                    const d = document.querySelector('#sset .calib-drawer');
                    if (d) d.scrollTop = d.scrollHeight;
                  });
                  await page.waitForTimeout(60);
                }
                const dInfo = await page.evaluate(() => {
                  const vw = window.innerWidth, vh = window.innerHeight;
                  const sw = document.documentElement.scrollWidth;
                  const sset = document.getElementById('sset');
                  const drawer = sset.querySelector('.calib-drawer');
                  const dr = drawer.getBoundingClientRect();
                  const pts = [
                    ['top-left', dr.left + 20, dr.top + 20],
                    ['top-mid', dr.left + dr.width / 2, dr.top + 20],
                    ['top-right', dr.right - 20, dr.top + 20],
                    ['mid-right', dr.right - 24, dr.top + 120],
                    ['center', dr.left + dr.width / 2, dr.top + dr.height / 2],
                    ['bot-right', dr.right - 30, dr.bottom - 40],
                  ];
                  const overlaps = [];
                  for (const [name, x, y] of pts) {
                    if (x < 0 || y < 0 || x >= vw || y >= vh) continue;
                    const hit = document.elementFromPoint(x, y);
                    if (!hit || !sset.contains(hit)) {
                      overlaps.push({ name, hit: hit ? `${hit.tagName}#${hit.id}.${hit.className}` : null });
                    }
                  }
                  const estops = [...document.querySelectorAll('#btn-estop, #sset .head-estop')].map((el) => {
                    const cs = getComputedStyle(el);
                    const r = el.getBoundingClientRect();
                    const vis = !el.hidden && cs.display !== 'none' && cs.visibility !== 'hidden' && r.width > 0 && r.height > 0;
                    const inVp = vis && r.left >= 0 && r.top >= 0 && r.right <= vw + 0.5 && r.bottom <= vh + 0.5;
                    const hit = inVp ? document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2) : null;
                    return { id: el.id || el.className, vis, inVp, hitOk: !!hit && (hit === el || el.contains(hit)) };
                  });
                  return { vw, sw, overlaps, estops, drawerOpacity: parseFloat(getComputedStyle(drawer).opacity) };
                });
                assert.equal(dInfo.sw, dInfo.vw, `${tag} drawer(${settleLabel}): scrollWidth ${dInfo.sw} !== innerWidth ${dInfo.vw}`);
                assert.equal(dInfo.drawerOpacity, 1, `${tag} drawer(${settleLabel}): drawer opacity must be 1, got ${dInfo.drawerOpacity}`);
                assert.equal(dInfo.overlaps.length, 0, `${tag} drawer(${settleLabel}): controls overlapped drawer: ${JSON.stringify(dInfo.overlaps)}`);
                assert(dInfo.estops.some((e) => e.vis && e.inVp && e.hitOk),
                  `${tag} drawer(${settleLabel}): no E-stop visible & hit-testable: ${JSON.stringify(dInfo.estops)}`);
              }
              await page.click('#sset-close');
              await waitHidden(page, '#sset', 2000);
            }
            assert.equal(http404.length, 0, `Scenario 8 ${vp.label} ${lang}: unexpected 404s: ${JSON.stringify(http404)}`);
            guards.assertClean(`Scenario 8 ${vp.label} ${lang}`);
          } finally { await ctx.close(); }
        }
      }

      // Park-off -> #tools-release hint while parking and immediate enablement once done
      const { ctx, page, guards } = await newCockpit(browser, PORT_AUTH, { width: 1024, height: 768, locale: 'zh-CN' });
      try {
        await page.goto(`http://127.0.0.1:${PORT_AUTH}/`, { waitUntil: 'domcontentloaded' });
        await waitForCockpitLive(page);
        await holdButton(page, '#btn-enable', 750);
        await waitFn(page, () => S.state === 'Armed' && S.obs?.is_torqued === true && S.leaseGeneration != null, null, 6000);
        await page.waitForTimeout(400);

        await holdButton(page, '#btn-enable', 750);
        await page.click('#btn-sset');
        await page.waitForSelector('#tools-release', { state: 'visible', timeout: 3000 });

        await waitFn(page, () => {
          const btn = document.getElementById('tools-release');
          const hint = document.getElementById('tools-release-hint')?.textContent?.trim() || '';
          return btn?.disabled === true && /正在停靠并卸力/.test(hint);
        }, null, 3000);

        await waitFn(page, () => S.obs?.is_torqued === false && S.state === 'Idle'
          && !document.getElementById('tools-release')?.disabled
          && /扭矩关/.test(document.getElementById('lease-status-txt')?.textContent || ''), null, 8000);

        await page.click('#tools-release');
        await waitFn(page, () => S.leaseGeneration === null && document.getElementById('lease-status').className.includes('is-readonly'), null, 5000);
        guards.assertClean('Scenario 8 park-off release');
        log('PASS Scenario 8: header fits 360x780/390x844/1024x768 (en/zh, simple/expert), favicon ok, #tools-release explains parking and enables once done.');
      } finally { await ctx.close(); }
    }

    await killArmd(armdAuth);
    armdAuth = null;
    await runScenario9OwnerHonesty(browser);

    log('ALL 9 COCKPIT LEASE & OWNER HONESTY PLAYWRIGHT E2E SCENARIOS PASSED!');
  } finally {
    await cleanup();
  }
}

for (const [sig, code] of [['SIGINT', 130], ['SIGTERM', 143]]) {
  process.on(sig, () => { cleanup().finally(() => process.exit(code)); });
}

main().catch((err) => { console.error('TEST FAILED:', err && err.stack ? err.stack : err); cleanup().finally(() => process.exit(1)); });
