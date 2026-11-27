#!/usr/bin/env node
// External control monitor (#ext-banner, #act-chip, #task drawer) end to end against a mock armd:
//   PLAYWRIGHT=/path/to/playwright/index.mjs OUT=/dir node scripts/task_monitor_shots.mjs
// Starts target/debug/armd --follower mock (ARMD_BIN, profile kits/recamera-b601dm) on a free port, or uses
// BASE=http://127.0.0.1:<port>/ when given. The page is src/api/index.html read from disk (HTML=<file> to test
// another copy; armd's embedded copy is not used), so no rebuild is needed. External clients are plain
// WebSockets with their own client_id (rebot-mcp, rebot-app-demo).
// Checks:
//   1. the activity log has one entry per action id across an E-stop, a fault clear and a second E-stop; an
//      entry keeps the client that started it; "Clear" stays cleared; a dismissed banner stays dismissed;
//   2. a lease held by an idle client shows "holds control, idle", a servo stream shows real-time control;
//   3. a tagged ("cockpit") action from another page is not this page's own; this page's action is;
//   4. 375 px phone, expert mode, external holder: no horizontal scroll, language button and E-stop in view;
//   5. the drawer refresh pauses while the tab is hidden; the HUD chip has a status live region.
// zh and en, 1280x800 and 375x812 screenshots of the banner and the drawer. Exits non-zero on a failed check.
import fs from 'node:fs';
import net from 'node:net';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { readCockpitHtmlBuffer } from './source_paths.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/task-monitor-shots');
fs.mkdirSync(out, { recursive: true });
const html = readCockpitHtmlBuffer(process.env.HTML);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

let armd = null, base = process.env.BASE;
if (!base) {
  const port = await new Promise((r) => { const s = net.createServer().listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => r(p)); }); });
  armd = spawn(process.env.ARMD_BIN || path.join(root, 'target/debug/armd'),
    ['--follower', 'mock', '--config', path.join(root, 'kits/recamera-b601dm/profile/rebot-b601dm.json'), '--port', String(port),
      '--log-dir', 'off', '--blackbox-dir', 'off', '--episodes-dir', 'off'],
    // cwd = repo root: armd finds assets/reBot-DevArm_gripper.urdf, so the gravity model loads and the
    // safety card has its gravity-comp switch (the tallest safety card the layout checks must handle)
    { cwd: root, stdio: ['ignore', 'ignore', 'inherit'] });
  base = `http://127.0.0.1:${port}/`;
  for (let i = 0; i < 100; i++) { try { if ((await fetch(base + 'api/v1/observation')).ok) break; } catch (_) {} await sleep(100); }
}
const wsUrl = base.replace(/^http/, 'ws') + 'api/v1/ws';

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; console.log((c ? '  ok   ' : '  FAIL ') + m); if (!c) bad++; return c; };
const shots = [];
const shot = async (page, name) => { const f = path.join(out, name + '.png'); await page.screenshot({ path: f }); shots.push(f); };

// external client: lease, heartbeat, commands
async function extClient(clientId) {
  const ws = new WebSocket(wsUrl);
  const C = { obs: null, gen: null, rid: 0, waits: new Map(), hb: 0 };
  ws.onmessage = (ev) => {
    const o = JSON.parse(ev.data);
    if (Array.isArray(o.positions)) { C.obs = o; return; }
    if (o.type === 'lease_granted') C.gen = o.generation;
    if (o.request_id != null && C.waits.has(o.request_id)) { C.waits.get(o.request_id)(o); C.waits.delete(o.request_id); }
  };
  await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; });
  for (let i = 0; i < 50 && !C.obs; i++) await sleep(20);
  C.acquire = async () => {
    ws.send(JSON.stringify({ type: 'acquire_lease', seq: 1, client_id: clientId, force: false }));
    for (let i = 0; i < 50 && C.gen == null; i++) await sleep(20);
    C.hb = setInterval(() => { try { ws.send(JSON.stringify({ type: 'heartbeat', client_id: clientId, generation: C.gen, epoch_id: C.obs.epoch_id, timestamp_ms: Date.now() })); } catch (_) {} }, 150);
    return C.gen != null;
  };
  C.cmd = (p, body) => new Promise((r) => {
    const id = ++C.rid; C.waits.set(id, r);
    ws.send(JSON.stringify({ type: 'command', request_id: id, path: p, body, generation: C.gen, epoch_id: C.obs.epoch_id, client_id: clientId }));
  });
  C.release = async () => {
    clearInterval(C.hb);
    ws.send(JSON.stringify({ type: 'release_lease', client_id: clientId, generation: C.gen }));
    C.gen = null; await sleep(300);
  };
  C.close = () => { clearInterval(C.hb); try { ws.close(); } catch (_) {} };
  C.waitDone = async (id) => { for (let i = 0; i < 100; i++) { const a = C.obs.action; if (a && a.id === id && a.state !== 'running') return a; await sleep(50); } return null; };
  return C;
}
const eStop = () => fetch(base + 'api/v1/safety/e_stop', { method: 'POST', headers: { 'content-type': 'application/json', Origin: new URL(base).origin }, body: '{}' });
const waitSafety = async (C, re) => { for (let i = 0; i < 60; i++) { if (re.test(JSON.stringify(C.obs.safety_state))) return true; await sleep(50); } return false; };

// the page is fulfilled by Playwright, which Chromium's local-network checks would treat as a public origin
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined,
  args: ['--disable-features=LocalNetworkAccessChecks,BlockInsecurePrivateNetworkRequests,PrivateNetworkAccessChecks'] });
async function openPage({ lang = 'en', ui = 'expert', viewport = { width: 1280, height: 800 }, qs = '' } = {}) {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  await ctx.route((u) => u.pathname === '/' || u.pathname === '/index.html', (r) => r.fulfill({ status: 200, contentType: 'text/html; charset=utf-8', body: html }));
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + '?ui=' + ui + qs, { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 });
  await page.waitForTimeout(300);
  return { page, ctx, errs };
}
const tkActs = (page) => page.evaluate(() => TK.log.filter((e) => e.type === 'action').map((e) => ({ aid: e.aid, src: e.src, own: e.own, state: e.state })));
const dupes = (acts) => { const n = {}; for (const a of acts) n[a.aid] = (n[a.aid] || 0) + 1; return Object.entries(n).filter(([, c]) => c > 1); };
// E-stop fully inside the viewport and the topmost element at its centre; banner top edge not under another element.
// Run at the initial scroll, with the banner scrolled to the top of its scroller and to the centre, and with every
// scroller at its end. The banner is checked where it is scrolled to (initial, start, centre); at the end of the
// scroll it has moved up under the pinned safety card like any other content.
async function layoutCheck(page, tag) {
  const probe = (checkBanner) => page.evaluate((cb) => {
    const es = document.getElementById('btn-estop'), r = es.getBoundingClientRect();
    const hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
    const estop = { inView: r.width > 0 && r.top >= -0.5 && r.left >= -0.5 && r.bottom <= innerHeight + 0.5 && r.right <= innerWidth + 0.5,
      top: hit === es || es.contains(hit), hit: hit ? (hit.id || hit.className) : null, r: [r.left, r.top, r.right, r.bottom].map(Math.round) };
    const b = document.getElementById('ext-banner'), br = b.getBoundingClientRect();
    const shown = !b.hidden && getComputedStyle(b).display !== 'none' && br.height > 0;
    let banner = { shown };
    if (cb && shown) {
      const y = br.top + 6, xs = [br.left + 12, br.left + br.width / 2, br.right - 12];
      const hits = xs.map((x) => document.elementFromPoint(x, y));
      banner = { shown, top: Math.round(br.top), inView: br.top >= 0 && br.top < innerHeight,
        clear: hits.every((h) => h && b.contains(h)), hits: hits.map((h) => (h ? (h.id || h.className) : null)) };
    }
    return { estop, banner };
  }, checkBanner);
  const results = [];
  results.push(['initial', await probe(true)]);
  await page.evaluate(() => document.getElementById('ext-banner').scrollIntoView({ block: 'start' }));
  await page.waitForTimeout(150);
  results.push(['banner at start', await probe(true)]);
  await page.evaluate(() => document.getElementById('ext-banner').scrollIntoView({ block: 'center' }));
  await page.waitForTimeout(150);
  results.push(['banner at centre', await probe(true)]);
  await page.evaluate(() => { for (const e of [document.scrollingElement, ...document.querySelectorAll('*')]) if (e.scrollHeight > e.clientHeight + 1) e.scrollTop = e.scrollHeight; });
  await page.waitForTimeout(150);
  results.push(['scrolled to the end', await probe(false)]);
  for (const [st, r] of results) {
    ok(r.estop.inView && r.estop.top, `${tag} ${st}: E-stop fully in view and on top ${JSON.stringify(r.estop)}`);
    if (r.banner.top !== undefined && r.banner.inView) ok(r.banner.clear, `${tag} ${st}: banner top not covered ${JSON.stringify(r.banner)}`);
  }
  await page.evaluate(() => { for (const e of [document.scrollingElement, ...document.querySelectorAll('*')]) if (e.scrollTop) e.scrollTop = 0; });
}
const vis = (page, id) => page.evaluate((i) => { const el = document.getElementById(i); if (!el) return false; const r = el.getBoundingClientRect(); return !el.hidden && getComputedStyle(el).display !== 'none' && r.width > 0 && r.height > 0; }, id);

try {
  // ---------- 1. one entry per action id across E-stop / fault clear; attribution; Clear; dismissal ----------
  console.log('1. activity log across E-stop and fault clear');
  {
    const A = await openPage();
    const mcp = await extClient('rebot-mcp');
    await mcp.acquire();
    await mcp.cmd('/api/v1/arm', { enable: true });
    await sleep(300);
    const ack = await mcp.cmd('/api/v1/motion/jog', { target_positions: mcp.obs.positions.map((v, i) => (i === 0 ? v + 0.05 : v)), duration_secs: 0.4 });
    const aid = ack.action_id;
    await mcp.waitDone(aid);
    await mcp.cmd('/api/v1/arm', { enable: false });
    await sleep(400);
    await mcp.release();
    await A.page.waitForTimeout(300);
    let acts = await tkActs(A.page);
    ok(acts.filter((a) => a.aid === aid).length === 1 && acts.find((a) => a.aid === aid).src === 'rebot-mcp', `rebot-mcp action ${aid} logged once as rebot-mcp ${JSON.stringify(acts)}`);
    ok(await vis(A.page, 'ext-banner') && await vis(A.page, 'ext-dismiss'), 'banner shows the finished external action with a dismiss button');
    await A.page.click('#ext-dismiss');
    await A.page.waitForTimeout(200);
    ok(!(await vis(A.page, 'ext-banner')), 'banner hidden after dismiss');
    const app = await extClient('rebot-app-demo');
    await app.acquire();
    await A.page.waitForTimeout(300);
    await eStop();
    ok(await waitSafety(app, /EStop/), 'E-stop latched');
    await A.page.waitForTimeout(500);
    acts = await tkActs(A.page);
    ok(dupes(acts).length === 0, `after E-stop: no duplicated action ids ${JSON.stringify(dupes(acts))}`);
    ok(acts.filter((a) => a.aid === aid).every((a) => a.src === 'rebot-mcp'), `after E-stop: action ${aid} still attributed to rebot-mcp ${JSON.stringify(acts)}`);
    const clr = await app.cmd('/api/v1/safety/clear_fault', {});
    ok(clr.type === 'command_ack', `fault cleared by rebot-app-demo (${clr.type} ${clr.code || ''})`);
    await A.page.waitForTimeout(500);
    acts = await tkActs(A.page);
    ok(dupes(acts).length === 0, `after fault clear: no duplicated action ids ${JSON.stringify(dupes(acts))}`);
    ok(acts.filter((a) => a.aid === aid).every((a) => a.src === 'rebot-mcp'), `after fault clear: action ${aid} still attributed to rebot-mcp ${JSON.stringify(acts)}`);
    await app.release();
    await A.page.waitForTimeout(300);
    ok(!(await vis(A.page, 'ext-banner')), 'dismissed banner stays hidden after E-stop, fault clear and lease changes');
    // Clear, then E-stop: nothing comes back
    await A.page.click('#btn-task');
    await A.page.click('#task-clear');
    await A.page.click('#task-close');
    await eStop();
    await A.page.waitForTimeout(600);
    acts = await tkActs(A.page);
    ok(!acts.some((a) => a.aid <= aid), `after Clear + E-stop: cleared entries stay cleared ${JSON.stringify(acts)}`);
    // armd restart is seen as timestamps going backwards: tracking resets (simulated on the page)
    const reset = await A.page.evaluate(() => { const o = { ...S.obs, timestamp: 0.5, action: { ...(S.obs.previous_action || S.obs.action || { id: 1, state: 'done' }), id: 1, state: 'done', started_at: 0.1, finished_at: 0.2 } };
      tkOnObs(o); return TK.log.filter((e) => e.type === 'action' && e.aid === 1).length; });
    ok(reset === 1, `timestamp going backwards (armd restart): action id 1 tracked again (${reset})`);
    ok(!A.errs.length, `no page errors ${JSON.stringify(A.errs)}`);
    // recover for the next sections
    await app.acquire(); await app.cmd('/api/v1/safety/clear_fault', {}); await app.release();
    mcp.close(); app.close();
    await A.ctx.close();
  }

  // ---------- 2. idle holder vs. servo stream ----------
  console.log('2. idle lease holder');
  for (const lang of ['zh', 'en']) {
    const A = await openPage({ lang });
    const mcp = await extClient('rebot-mcp');
    await mcp.acquire();
    await A.page.waitForTimeout(800);
    const idle = await A.page.evaluate(() => ({ step: $('ext-step').textContent, chip: $('act-chip').textContent, mode: TK.k && TK.k.mode }));
    const want = { zh: '持有控制权，空闲', en: 'Holds control, idle' }[lang];
    ok(idle.step === want, `${lang} idle holder: banner says "${idle.step}" (want "${want}")`);
    await mcp.cmd('/api/v1/arm', { enable: true });
    await sleep(300);
    const q0 = mcp.obs.positions.slice();
    let k = 0;
    const iv = setInterval(() => { k++; mcp.cmd('/api/v1/motion/servo', { targets: q0.map((v, i) => (i === 0 ? v + 0.002 * k : v)) }); }, 40);
    await A.page.waitForTimeout(700);
    const live = await A.page.evaluate(() => ({ step: $('ext-step').textContent, mode: TK.k && TK.k.mode }));
    clearInterval(iv);
    const wantS = { zh: '外部实时控制中', en: 'Servo stream (external real-time control)' }[lang];
    ok(live.step === wantS, `${lang} streaming holder: banner says "${live.step}" (want "${wantS}")`);
    await A.page.waitForTimeout(2500);
    const after = await A.page.evaluate(() => $('ext-step').textContent);
    ok(after === want, `${lang} stream stopped: back to "${after}"`);
    await mcp.cmd('/api/v1/arm', { enable: false }); await sleep(300);
    await mcp.release(); mcp.close();
    await A.ctx.close();
  }

  // ---------- 3. another cockpit page's tagged action is not "this page" ----------
  console.log('3. own action decided by action id');
  {
    const A = await openPage({ qs: '&lease_idle_ms=500' });
    const B = await openPage({ qs: '&lease_idle_ms=500' });
    const bId = await B.page.evaluate(() => CLIENT_ID);
    const aId = await A.page.evaluate(() => CLIENT_ID);
    const bAid = await B.page.evaluate(async () => {
      await commandRPC('/api/v1/arm', { enable: true });
      await new Promise((r) => setTimeout(r, 300));
      const d = await commandRPC('/api/v1/action/move_relative', { frame: 'base', d: [0, 0, 0.01], tag: 'cockpit' });
      return d.action_id;
    });
    await B.page.waitForTimeout(1500);
    let a = (await tkActs(A.page)).find((x) => x.aid === bAid);
    let b = (await tkActs(B.page)).find((x) => x.aid === bAid);
    ok(a && a.own === false && a.src === bId, `page A: page B's move ${bAid} is external, from ${bId} ${JSON.stringify(a)}`);
    ok(b && b.own === true, `page B: its own move ${bAid} is own ${JSON.stringify(b)}`);
    await shot(A.page, 'task_other_page_banner_en_1280x800');
    // B: torque off; its idle lease is released after 0.5 s
    await B.page.evaluate(() => commandRPC('/api/v1/arm', { enable: false }));
    await B.page.waitForFunction(() => S.leaseGeneration == null, null, { timeout: 8000 }).catch(() => {});
    await A.page.waitForFunction(() => S.obs && S.obs.lease_holder == null, null, { timeout: 8000 }).catch(() => {});
    const aAid = await A.page.evaluate(async () => {
      await commandRPC('/api/v1/arm', { enable: true });
      await new Promise((r) => setTimeout(r, 300));
      const d = await commandRPC('/api/v1/action/move_relative', { frame: 'base', d: [0, 0, -0.01], tag: 'cockpit' });
      return d.action_id;
    });
    await A.page.waitForTimeout(1500);
    a = (await tkActs(A.page)).find((x) => x.aid === aAid);
    b = (await tkActs(B.page)).find((x) => x.aid === aAid);
    ok(a && a.own === true && a.src === aId, `page A: its own move ${aAid} is own ${JSON.stringify(a)}`);
    ok(b && b.own === false && b.src === aId, `page B: page A's move ${aAid} is external, from ${aId} ${JSON.stringify(b)}`);
    await A.page.evaluate(() => commandRPC('/api/v1/arm', { enable: false }));
    // the observation of an action can arrive before this page's ack of it: the ack makes the entry own
    const race = await A.page.evaluate(() => {
      const id = 900001, o = { ...S.obs, action: { id, kind: 'move_to', tag: 'cockpit', state: 'running', progress: 0.1, started_at: S.obs.timestamp, finished_at: null } };
      tkOnObs(o);
      const before = TK.tracked.get('a' + id).own;
      tkOwnAction(id);
      const e = TK.tracked.get('a' + id);
      return { before, after: e.own, src: e.src === CLIENT_ID, lastExt: TK.lastExt === e };
    }).catch((err) => ({ err: String(err) }));
    ok(race.before === false && race.after === true && race.src && !race.lastExt, `page A: ack after the first observation makes the entry own ${JSON.stringify(race)}`);
    await A.ctx.close(); await B.ctx.close();
  }

  // ---------- 4 + 5 + screenshots ----------
  console.log('4. header width, 5. drawer refresh and chip live region, screenshots');
  for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
    const tag = `${lang}_${vp}`;
    const A = await openPage({ lang, viewport });
    const mcp = await extClient('rebot-mcp');
    await mcp.acquire();
    await mcp.cmd('/api/v1/arm', { enable: true });
    await sleep(300);
    const ack = await mcp.cmd('/api/v1/motion/jog', { target_positions: mcp.obs.positions.map((v, i) => (i === 0 ? v + 0.6 : v)), duration_secs: 3 });
    await A.page.waitForTimeout(900);
    const lay = await A.page.evaluate(() => {
      const r = (id) => { const el = document.getElementById(id); const b = el.getBoundingClientRect(); return { l: b.left, r: b.right, t: b.top, b: b.bottom, w: b.width, shown: getComputedStyle(el).display !== 'none' && !el.hidden }; };
      return { sw: document.scrollingElement.scrollWidth, iw: innerWidth, lang: r('btn-lang'), estop: r('btn-estop'), task: r('btn-task'),
        wide: [...document.querySelectorAll('.topbar > *')].filter((e) => e.getBoundingClientRect().right > innerWidth + 0.5).map((e) => e.id || e.className) };
    });
    ok(lay.sw <= lay.iw, `${tag} expert, external action running: no horizontal scroll (${lay.sw} > ${lay.iw}? off-screen: ${lay.wide.join(',')})`);
    ok(lay.lang.r <= lay.iw + 0.5 && lay.lang.l >= 0, `${tag} language button inside the viewport ${JSON.stringify(lay.lang)}`);
    ok(lay.estop.shown && lay.estop.w > 0 && lay.estop.r <= lay.iw + 0.5, `${tag} E-stop shown and inside the viewport width ${JSON.stringify(lay.estop)}`);
    ok(await A.page.evaluate(() => !$('btn-gc').hidden), `${tag} gravity-comp row present in the safety card`);
    await layoutCheck(A.page, `${tag} expert`);
    await A.page.evaluate(() => { const el = $('ext-banner'); el.scrollIntoView({ block: 'center' }); });
    await A.page.waitForTimeout(150);
    ok(await vis(A.page, 'ext-banner'), `${tag} banner shown while the external action runs`);
    await shot(A.page, `task_banner_expert_${tag}`);
    // HUD chip: a polite live region announces the summary
    const live = await A.page.evaluate(() => { const c = $('act-chip'); const r = c.querySelector('[role="status"][aria-live]') || document.querySelector('[role="status"][aria-live][data-for="act-chip"]');
      return { has: !!r, text: r ? r.textContent : '', chipRole: c.getAttribute('role') }; });
    ok(live.has && live.text.length > 0, `${tag} act-chip live region ${JSON.stringify(live)}`);
    await mcp.waitDone(ack.action_id);
    // drawer: screenshot, refresh paused while the tab is hidden
    await A.page.click((await vis(A.page, 'btn-task')) ? '#btn-task' : '#ext-open-task');
    await A.page.waitForTimeout(400);
    ok(await vis(A.page, 'task'), `${tag} drawer opens from ${(await vis(A.page, 'btn-task')) ? 'the task button' : 'the banner'}`);
    await shot(A.page, `task_drawer_act_${tag}`);
    await A.page.click('#task-tabs [data-tab="env"]');
    await A.page.waitForTimeout(500);
    await shot(A.page, `task_drawer_env_${tag}`);
    if (vp === '1280x800') {
      let n = 0;
      const onReq = (r) => { if (r.url().includes('/api/v1/sandbox/catalog')) n++; };
      A.page.on('request', onReq);
      await A.page.evaluate(() => { Object.defineProperty(document, 'visibilityState', { configurable: true, get: () => 'hidden' }); Object.defineProperty(document, 'hidden', { configurable: true, get: () => true }); document.dispatchEvent(new Event('visibilitychange')); });
      await A.page.waitForTimeout(4500);
      const hiddenN = n; n = 0;
      await A.page.evaluate(() => { Object.defineProperty(document, 'visibilityState', { configurable: true, get: () => 'visible' }); Object.defineProperty(document, 'hidden', { configurable: true, get: () => false }); document.dispatchEvent(new Event('visibilitychange')); });
      await A.page.waitForTimeout(2500);
      A.page.off('request', onReq);
      ok(hiddenN === 0, `${tag} drawer refresh paused while hidden (${hiddenN} catalog requests in 4.5 s)`);
      ok(n >= 1, `${tag} drawer refresh resumes when visible (${n} catalog requests in 2.5 s)`);
    }
    await A.page.click('#task-close');
    await mcp.cmd('/api/v1/arm', { enable: false }); await sleep(300);
    await mcp.release(); mcp.close();
    ok(!A.errs.length, `${tag} no page errors ${JSON.stringify(A.errs)}`);
    await A.ctx.close();
    // simple mode with an external holder: header fits, the task button shows, E-stop and banner layout
    const Sm = await openPage({ lang, ui: 'simple', viewport });
    const mcp2 = await extClient('rebot-mcp');
    await mcp2.acquire();
    await Sm.page.waitForTimeout(600);
    const sl = await Sm.page.evaluate(() => ({ sw: document.scrollingElement.scrollWidth, iw: innerWidth, lang: document.getElementById('btn-lang').getBoundingClientRect().right,
      task: getComputedStyle(document.getElementById('btn-task')).display !== 'none' }));
    ok(sl.sw <= sl.iw && sl.lang <= sl.iw + 0.5, `${tag} simple, external holder: no horizontal scroll, language button in view ${JSON.stringify(sl)}`);
    ok(sl.task, `${tag} simple, external holder: task button shown`);
    ok(await Sm.page.evaluate(() => !$('btn-gc').hidden), `${tag} simple: gravity-comp row present`);
    await layoutCheck(Sm.page, `${tag} simple`);
    await shot(Sm.page, `task_banner_simple_${tag}`);
    await mcp2.release(); mcp2.close();
    await Sm.ctx.close();
  }
} finally {
  await browser.close();
  if (armd) armd.kill('SIGTERM');
}
for (const f of shots) console.log('  ' + f);
console.log(`${shots.length} screenshots in ${out}, ${checks} checks, ${bad ? bad + ' failed' : 'all passed'}`);
process.exit(bad ? 1 : 0);
