#!/usr/bin/env node
// Cockpit against armd --follower sim with the simulator's wrist camera and ground-truth detector, operator PIN on:
//   cd sim/mujoco && uv run rebot-sim --port 47671 --camera --video-port 18091 --perception-port 18090 --objects cube,cup --seed 3 &
//   ARMD_DETECTOR_URL=http://127.0.0.1:18090 ARMD_POSES=/tmp/p.json ARMD_KEEPOUT=/tmp/k.json ARMD_GRASP_SETUP=/tmp/g.json \
//     ./target/debug/armd --follower sim --follower-port udp://127.0.0.1:47671 \
//     --config kits/recamera-b601dm/profile/rebot-b601dm.json --port 18972 --operator-pin 246810 --log-dir off --blackbox-dir off &
//   BASE=http://127.0.0.1:18972/ SIM=http://127.0.0.1:18090 PIN=246810 PLAYWRIGHT=/path/to/playwright/index.mjs OUT=/dir \
//     node scripts/sim_cockpit_shots.mjs
// SRC=src/api serves the page from disk. Tablet 1024x768 zh and phone 390x844 en:
//  1. enable (hold), unlock with the PIN from the badge;
//  2. collision guard (settings drawer, locked first: the switch opens the PIN dialog): on, sensitivity high, off;
//     armd's GET /api/v1/safety/collision follows each step;
//  3. the arm looks down at the cube (move_to over the WebSocket, as the camera calibration jog does); Single shot
//     through armd returns the cube from the simulator's ground-truth detector;
//  4. tap-to-grasp on the cube's box from the cockpit (graspGo, the Grasp button): the action ends done with the cube
//     held, or the toast names the cause (grasp setup missing, unreachable); the result is printed either way.
// Exits non-zero if a check fails.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/sim-cockpit-shots');
const base = process.env.BASE || 'http://127.0.0.1:18972/';
const SIM = process.env.SIM || 'http://127.0.0.1:18090';
const PIN = process.env.PIN || '246810';
const SRC = process.env.SRC ? path.resolve(process.env.SRC) : null;
fs.mkdirSync(out, { recursive: true });
let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } return c; };
const origin = new URL(base).origin;
const browser = await chromium.launch({ headless: true, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader',
  ...(SRC ? ['--disable-features=LocalNetworkAccessChecks,BlockInsecurePrivateNetworkRequests,PrivateNetworkAccessChecks'] : [])] });
async function waitFor(page, fn, arg, ms = 8000) { try { await page.waitForFunction(fn, arg, { timeout: ms }); return true; } catch (_) { return false; } }
const settle = (page) => page.waitForFunction(() => document.getAnimations().every((a) => a.playState !== 'running' || a.effect?.getComputedTiming().iterations === Infinity), null, { timeout: 5000 }).catch(() => {});
const toasts = (page) => page.evaluate(() => [...document.querySelectorAll('#toasts .toast')].map((x) => x.innerText.replace(/\s*\n\s*/g, ' | ')));
const col = async () => (await (await fetch(new URL('/api/v1/safety/collision', base))).json());
const sim = async () => (await (await fetch(SIM + '/sim/state')).json());
async function hold(page, sel, ms = 900) { const b = await page.locator(sel).boundingBox(); await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2); await page.mouse.down(); await page.waitForTimeout(ms); await page.mouse.up(); }
async function pin(page) { await page.fill('#op-pin', PIN); await page.press('#op-pin', 'Enter'); return waitFor(page, () => $('op-modal').hidden && S.auth.role === 'operator'); }

for (const [name, view, lang] of [['tablet_zh', { width: 1024, height: 768 }, 'zh'], ['phone_en', { width: 390, height: 844 }, 'en']]) {
  await fetch(new URL('/api/v1/auth/lock', base), { method: 'POST', headers: { Origin: origin } }).catch(() => {});
  const ctx = await browser.newContext({ viewport: view, hasTouch: view.width < 1000 });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  page.on('pageerror', (e) => { console.log('  pageerror ' + e.message); ok(false, name + ' pageerror'); });
  if (SRC) {
    await page.route((u) => u.pathname === '/', (r) => r.fulfill({ status: 200, contentType: 'text/html; charset=utf-8', body: fs.readFileSync(path.join(SRC, 'index.html'), 'utf8') }));
    await page.route((u) => u.pathname === '/assets/calib_wizard.js', (r) => r.fulfill({ status: 200, contentType: 'application/javascript', body: fs.readFileSync(path.join(SRC, 'calib_wizard.js'), 'utf8') }));
  }
  await page.goto(base + '?ui=expert', { waitUntil: 'domcontentloaded' });
  ok(await waitFor(page, () => S.obs && S.dof === 7 && S.auth.enforced && S.caps && S.caps.collision_guard, null, 20000), name + ' sim telemetry, collision_guard capability');

  // 1. enable, home
  await hold(page, '#btn-enable');
  ok(await waitFor(page, () => S.state === 'Armed' && S.leaseGeneration != null, null, 10000), `${name} enabled (${await page.evaluate(() => S.state)})`);
  await page.click('#btn-home');
  await waitFor(page, () => S.state === 'Moving', null, 3000);
  ok(await waitFor(page, () => S.state === 'Armed', null, 20000), name + ' home done');

  // 2. collision guard, locked first
  await page.evaluate(() => ssetOpen(true)); await settle(page);
  ok(!(await page.evaluate(() => $('col-set').hidden)), name + ' collision guard in the settings');
  await page.click('#col-set .tgl');
  ok(await waitFor(page, () => !$('op-modal').hidden), name + ' collision switch while locked opens the PIN dialog');
  ok(await pin(page), name + ' PIN unlocks');
  await page.waitForTimeout(500);
  if (process.env.DEBUG) console.log('  dbg', JSON.stringify(await page.evaluate(() => ({ checked: $('col-en').checked, dis: $('col-en').disabled, busy: COL.busy, cfg: !!COL.cfg, st: $('col-status').textContent, state: S.state, act: S.obs.action }))), JSON.stringify(await toasts(page)));
  ok(await waitFor(page, () => $('col-en').checked && !COL.busy, null, 5000) && (await col()).enabled === true, `${name} guard on after the unlock (armd: ${JSON.stringify((await col()).enabled)})`);
  await page.click('#col-sens button[data-v="high"]');
  ok(await waitFor(page, () => !COL.busy && COL.cfg && COL.cfg.sensitivity === 'high', null, 5000) && (await col()).sensitivity === 'high', name + ' sensitivity high');
  await page.screenshot({ path: path.join(out, `${name}_1_collision_on.png`) });
  await page.click('#col-set .tgl');
  ok(await waitFor(page, () => !$('col-en').checked && !COL.busy, null, 5000) && (await col()).enabled === false, name + ' guard off');
  await page.evaluate(() => ssetOpen(false));

  // 3. look down at the cube, Single shot
  const cube = (await sim()).objects.find((o) => o.name === 'cube');
  const mv = await page.evaluate(async (xy) => {
    try { const a = await commandRPC('/api/v1/action/move_to', { position: [xy[0], xy[1], 0.25], orientation: { rpy: [0, Math.PI / 2, 0] }, speed: { linear: 0.1 } }, true, 5000); return { ok: true, id: a.action_id }; }
    catch (e) { return { ok: false, code: e.code, msg: e.message }; }
  }, cube.pos);
  ok(mv.ok, `${name} move_to over the cube ${JSON.stringify(mv)}`);
  ok(await waitFor(page, (id) => { const a = [S.obs.action, S.obs.previous_action].find((x) => x && x.id === id); return a && a.state === 'done'; }, mv.id, 25000), name + ' view pose reached');
  await page.waitForTimeout(400);
  await page.evaluate(() => { const b = $('btn-vis'); if (b && getComputedStyle(b).display !== 'none' && !document.body.classList.contains('vis-open')) b.click(); });
  await page.evaluate(() => $('btn-trigger').click());
  await page.waitForTimeout(2500);
  const det = { t: await toasts(page) };
  const trig = await (await page.evaluate(async () => { const r = await fetch('/api/v1/detection/trigger?wait_ms=2000', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ pipelines: ['yolo'] }) }); return { st: r.status, j: await r.json() }; }));
  const cubeDet = (trig.j.detections || []).find((d) => /cube/.test(d.class_name || d.label || d.name || ''));
  console.log(`  ${name} view: move ${JSON.stringify(mv)} tcp ${JSON.stringify((await sim()).tcp)} cube ${JSON.stringify(cube.pos)} detections ${JSON.stringify((trig.j.detections || []).map((d) => d.class_name))}`);
  ok(trig.st === 200 && !!cubeDet, `${name} Single shot through armd finds the cube (${trig.st}, ${(trig.j.detections || []).length} detections) ${JSON.stringify(det.t)}`);
  await page.screenshot({ path: path.join(out, `${name}_2_detect.png`) });

  // 4. tap-to-grasp the cube box: the cockpit's Grasp button path
  if (cubeDet) {
    // perception API 2: x, y, w, h normalised, box centre (format xywh_center)
    const { x, y, w, h } = cubeDet;
    const box = [x - w / 2, y - h / 2, x + w / 2, y + h / 2];
    const go = await page.evaluate(async (b) => { GR.sel = { label: 'cube', score: 0.99, box: b }; GR.sending = false; await graspGo(); return { id: GR.id, sel: !!GR.sel, locked: isLocked(), why: isLocked() ? lockReason() : '' }; }, box);
    console.log(`  ${name} graspGo ${JSON.stringify(go)}`);
    const done = await waitFor(page, () => { const a = S.obs && (S.obs.action || S.obs.previous_action); return a && a.kind === 'grasp' && a.state !== 'running'; }, null, 60000);
    const res = await page.evaluate(() => { const a = S.obs.action && S.obs.action.kind === 'grasp' ? S.obs.action : S.obs.previous_action; return a ? { state: a.state, code: a.code, holding: a.holding, msg: a.message } : null; });
    const t = await toasts(page);
    const st = await sim();
    console.log(`  ${name} grasp: ${JSON.stringify(res)} toasts ${JSON.stringify(t)} cube grasped ${st.objects.find((o) => o.name === 'cube').grasped}`);
    ok(done && res && (res.state === 'done' || t.length > 0), `${name} grasp ends done or with a message`);
    await page.screenshot({ path: path.join(out, `${name}_3_grasp.png`) });
    await page.evaluate(() => api('/api/v1/action/gripper', { action: 'open' }, { quiet: true }).catch(() => {}));
  }
  // park the arm for the next viewport
  const hid = await page.evaluate(() => commandRPC('/api/v1/motion/home', {}, true, 5000).then((a) => a.action_id, () => null));
  await waitFor(page, (id) => { const a = [S.obs.action, S.obs.previous_action].find((x) => x && x.id === id); return a && a.state !== 'running'; }, hid, 25000);
  console.log(`${name}: done`);
  await ctx.close();
  await new Promise((r) => setTimeout(r, 4000)); // lease of the closed page times out
}
await browser.close();
console.log(`${checks - bad}/${checks} checks passed`);
process.exit(bad ? 1 : 0);
