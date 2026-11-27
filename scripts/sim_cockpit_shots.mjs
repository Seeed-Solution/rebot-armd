#!/usr/bin/env node
// Cockpit against armd --follower sim with the simulator's wrist camera and ground-truth detector, operator PIN on:
//   cd sim/mujoco && uv run rebot-sim --port 47671 --camera --video-port 18091 --perception-port 18090 --objects cube,cup --seed 3 --depth metric &
//   ARMD_DETECTOR_URL=http://127.0.0.1:18090 ARMD_SNAPSHOT_PORT=18091 ARMD_POSES=/tmp/p.json ARMD_KEEPOUT=/tmp/k.json \
//     ARMD_GRASP_SETUP=/tmp/g.json ARMD_COLLISION=/tmp/c.json \
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
//  4. Measure size on the cube (the button beside Grasp): confirmation with range and duration before any motion,
//     progress, a result card near 40 x 40 x 40 mm with RMS and the calibration self-check, kept until closed, Copy;
//     the quick estimate labelled as an estimate;
//  5. tap-to-grasp on the cube's box (graspGo, the Grasp button): the action ends done, or a failure names the grasp
//     setup values that are still defaults and offers the grasp setup.
// Exits non-zero if a check fails.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { resolveCockpitInDir } from './source_paths.mjs';

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

// ONLY=tablet_zh|phone_en runs one viewport. Start each run on a fresh rebot-sim: after a park (torque off at the folded
// rest pose) the next Home in the simulator ends FOLLOWING_ERROR on J4 (J4 stays at 0.785 rad; seen 2026-11-23).
const RUNS = [['tablet_zh', { width: 1024, height: 768 }, 'zh'], ['phone_en', { width: 390, height: 844 }, 'en']].filter(([n]) => !process.env.ONLY || process.env.ONLY === n);
for (const [name, view, lang] of RUNS) {
  await fetch(new URL('/api/v1/auth/lock', base), { method: 'POST', headers: { Origin: origin } }).catch(() => {});
  const ctx = await browser.newContext({ viewport: view, hasTouch: view.width < 1000 });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  page.on('pageerror', (e) => { console.log('  pageerror ' + e.message); ok(false, name + ' pageerror'); });
  if (SRC) {
    await page.route((u) => u.pathname === '/', (r) => r.fulfill({ status: 200, contentType: 'text/html; charset=utf-8', body: fs.readFileSync(resolveCockpitInDir(SRC), 'utf8') }));
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

  // 3. look down at the cube: tool down, the simulator's lens (end_link (-0.0731, 0.0005, 0.066) m) right above it,
  // so the cube is near the image centre and stays in view at the measurement's side views, Single shot
  // armd turns the torque off once the arm has settled at the rest pose after a home: enable again
  if (!(await page.evaluate(() => S.obs.is_torqued))) { await hold(page, '#btn-enable'); await waitFor(page, () => S.state === 'Armed', null, 10000); }
  const cube = (await sim()).objects.find((o) => o.name === 'cube');
  const mv = await page.evaluate(async (xy) => {
    try { const a = await commandRPC('/api/v1/action/move_to', { position: [xy[0] - 0.066, xy[1] - 0.0005, 0.25], orientation: { rpy: [0, Math.PI / 2, 0] }, path: "joint", speed_scale: 0.5 }, true, 5000); return { ok: true, id: a.action_id }; }
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

  // perception API 2: x, y, w, h normalised, box centre (format xywh_center)
  const box = cubeDet ? [cubeDet.x - cubeDet.w / 2, cubeDet.y - cubeDet.h / 2, cubeDet.x + cubeDet.w / 2, cubeDet.y + cubeDet.h / 2] : null;

  // 4. Measure size on the selected cube: the button beside Grasp, the confirmation, progress, the result card
  if (cubeDet) {
    // detection on (the grasp bar shows only then), the cube selected as a tap on its box would
    await page.evaluate(() => { if (!DET.enabled) $('btn-det').click(); });
    // close the recognition popover (phones: it covers the grasp bar) as a tap on its chip does
    await page.evaluate(() => { if (document.body.classList.contains('vis-open')) $('btn-vis').click(); });
    ok(await waitFor(page, () => DET.enabled, null, 6000), name + ' detection on');
    await page.evaluate((b) => { GR.sel = { label: 'cube', score: 0.99, box: b, seenAt: performance.now() }; renderGrasp(); }, box);
    ok(await waitFor(page, () => !$('grasp-bar').hidden && !$('meas-go').hidden && !$('meas-go').disabled), name + ' Measure size beside Grasp');
    const sameWeight = await page.evaluate(() => $('meas-go').className === $('grasp-go').className);
    ok(sameWeight, name + ' Measure size has the same style as Grasp');
    await page.screenshot({ path: path.join(out, `${name}_3a_selected.png`) });
    await page.click('#meas-go');
    ok(await waitFor(page, () => !$('meas-card').hidden && !!$('ms-start')), name + ' confirmation before any motion');
    const conf = await page.evaluate(() => ({ text: $('meas-card').innerText, act: S.obs.action && S.obs.action.kind }));
    ok(/3 cm|3 cm/.test(conf.text) && /15/.test(conf.text) && conf.act !== 'measure_multiview', `${name} confirmation names range and duration ${JSON.stringify(conf.text.slice(0, 160))}`);
    await settle(page);
    await page.screenshot({ path: path.join(out, `${name}_3b_confirm.png`) });
    await page.click('#ms-start');
    ok(await waitFor(page, () => MS.state === 'running', null, 6000), `${name} measuring (${await page.evaluate(() => MS.state + ' ' + JSON.stringify(MS.err))})`);
    await page.waitForTimeout(2500);
    await page.screenshot({ path: path.join(out, `${name}_3c_running.png`) });
    ok(await waitFor(page, () => MS.state === 'done' || MS.state === 'failed', null, 90000), name + ' measurement ended');
    const mres = await page.evaluate(() => ({ state: MS.state, err: MS.err, o: MS.res && MS.res.objects && MS.res.objects[0], tc: MS.res && MS.res.table_check, card: $('meas-card').innerText }));
    console.log(`  ${name} measure: ${JSON.stringify({ state: mres.state, err: mres.err, h: mres.o && mres.o.height_m, w: mres.o && mres.o.width_m, l: mres.o && mres.o.length_m, n: mres.o && mres.o.n_points, tc: mres.tc })}`);
    const near = (v, want, tol) => Number.isFinite(v) && Math.abs(v - want) <= tol;
    ok(mres.state === 'done' && mres.o && near(mres.o.height_m, 0.04, 0.012) && near(mres.o.width_m, 0.04, 0.015) && near(mres.o.length_m, 0.04, 0.015),
      `${name} cube measured near 40 x 40 x 40 mm`);
    ok(/mm/.test(mres.card) && /RMS/.test(mres.card) && (lang === 'zh' ? /标定自检/ : /Calibration self-check/).test(mres.card), name + ' result card: size, RMS, self-check label');
    await settle(page);
    await page.screenshot({ path: path.join(out, `${name}_3d_result.png`) });
    // the card stays (no auto-dismiss) and copies its values
    await page.waitForTimeout(3000);
    ok(await page.evaluate(() => !$('meas-card').hidden && MS.state === 'done'), name + ' result card stays');
    await ctx.grantPermissions(['clipboard-read', 'clipboard-write']).catch(() => {});
    await page.click('#ms-copy');
    await page.waitForTimeout(300);
    const clip = await page.evaluate(() => navigator.clipboard.readText().catch(() => ''));
    ok(/H [\d.]+ x W [\d.]+ x L [\d.]+ mm/.test(clip), `${name} copy puts the values on the clipboard ${JSON.stringify(clip.slice(0, 60))}`);
    // quick estimate: labelled as an estimate
    await page.click('#ms-again'); await page.click('#ms-quick');
    ok(await waitFor(page, () => MS.state === 'estimate' || MS.state === 'failed', null, 10000), name + ' quick estimate answered');
    const est = await page.evaluate(() => $('meas-card').innerText);
    ok((lang === 'zh' ? /估计值/ : /Estimate/).test(est), `${name} quick estimate labelled as an estimate ${JSON.stringify(est.slice(0, 120))}`);
    await page.screenshot({ path: path.join(out, `${name}_3e_estimate.png`) });
    await page.click('#ms-close');
    ok(await page.evaluate(() => $('meas-card').hidden), name + ' card closes');
  }

  // 5. tap-to-grasp the cube box: the cockpit's Grasp button path; with the grasp setup at its defaults a failed
  // grasp names them and offers the grasp setup
  if (cubeDet) {
    const go = await page.evaluate(async (b) => { GR.sel = { label: 'cube', score: 0.99, box: b }; GR.sending = false; await graspGo(); return { id: GR.id, sel: !!GR.sel, locked: isLocked(), why: isLocked() ? lockReason() : '' }; }, box);
    console.log(`  ${name} graspGo ${JSON.stringify(go)}`);
    const done = await waitFor(page, () => { const a = S.obs && (S.obs.action || S.obs.previous_action); return a && a.kind === 'grasp' && a.state !== 'running'; }, null, 60000);
    const res = await page.evaluate(() => { const a = S.obs.action && S.obs.action.kind === 'grasp' ? S.obs.action : S.obs.previous_action; return a ? { state: a.state, code: a.code, holding: a.holding, msg: a.message } : null; });
    const t = await toasts(page);
    const st = await sim();
    console.log(`  ${name} grasp: ${JSON.stringify(res)} toasts ${JSON.stringify(t)} cube grasped ${st.objects.find((o) => o.name === 'cube').grasped}`);
    ok(done && res && (res.state === 'done' || t.length > 0), `${name} grasp ends done or with a message`);
    if (res && res.state === 'failed' && ['NOT_HOLDING', 'CENTRING_FAILED', 'TARGET_LOST'].includes(res.code)) {
      await waitFor(page, () => !!document.querySelector('#toasts .toast-act'), null, 4000);
      const hint = await page.evaluate(() => { const x = [...document.querySelectorAll('#toasts .toast')].find((e) => e.querySelector('.toast-act')); return x ? { text: x.innerText, btn: x.querySelector('.toast-act').textContent } : null; });
      ok(!!hint && (lang === 'zh' ? /默认值/ : /defaults/).test(hint.text) && (lang === 'zh' ? hint.btn === '抓取设置' : hint.btn === 'Grasp setup'), `${name} grasp failure names the default setup and offers it ${JSON.stringify(hint)}`);
      await page.screenshot({ path: path.join(out, `${name}_4_grasp_hint.png`) });
      if (hint) { await page.click('#toasts .toast-act'); ok(await waitFor(page, () => !$('gset').hidden), name + ' the button opens the grasp setup'); await page.evaluate(() => gsetOpen(false)); }
    }
    await page.screenshot({ path: path.join(out, `${name}_4b_grasp.png`) });
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
