#!/usr/bin/env node
// Cockpit Teach panel against a real armd (mock follower), no page mocks:
//   armd --follower mock --config kits/recamera-b601dm/profile/rebot-b601dm.json --port 18930 --episodes-dir <dir>
//     (ARMD_POSES, ARMD_SEQUENCES, ARMD_CALIBRATION, ARMD_GRIPPER_CALIB pointing into a scratch dir; a fresh dir is best)
//   ARMD=http://127.0.0.1:18930 PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/teach_live.mjs
// Expert mode, English, 1400x900. Steps: enable; open the Teach panel from the settings drawer; record a trajectory
// with the teach strip while the arm jogs (source "jog"); new sequence; close the panel (the strip stays, covers no
// control and not the video); two poses via "Save pose" on the strip after setting the gripper with the normal
// gripper slider (each adds a pose step with gripper "pose" and saves the sequence; the poses hold the opening);
// from the strip back to the editor: a gripper step (opening slider), a wait, the recorded trajectory; save; run with
// "record this run" for 2 repeats (2 new episodes, source "sequence", sequence_id, repeat_index 0 / 1); run again and
// Stop during the wait (parent action ends, the status shows it); delete a used pose -> IN_USE with the sequence
// name; rename a pose (id kept, the step title follows); software zero of a joint from Tools > Zero (torque off in
// place, so the reading is off the reference) -> the poses show the old-zero badge, Run -> DATA_STALE confirm ->
// continue -> the run starts with allow_stale. Screenshots go to .cache/teach-live/. Exits non-zero on a failed check.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/teach-live');
fs.mkdirSync(out, { recursive: true });
const base = (process.env.ARMD || 'http://127.0.0.1:18930').replace(/\/$/, '') + '/';
const get = async (p) => (await fetch(base + p.replace(/^\//, ''), { cache: 'no-store' })).json();

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; console.log((c ? '  ok   ' : '  FAIL ') + m); if (!c) bad++; return c; };
const step = (m) => console.log('- ' + m);
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
const errs = [];
const tag = Date.now().toString(36).slice(-5);

const ctx = await browser.newContext({ viewport: { width: 1400, height: 900 }, locale: 'en-GB' });
await ctx.addInitScript(() => { try { localStorage.setItem('lang', 'en'); } catch (_) {} });
const page = await ctx.newPage();
page.on('pageerror', (e) => errs.push(String(e)));
page.on('console', (m) => { if (m.type() === 'error') errs.push('console: ' + m.text()); });
await page.goto(base + '?ui=expert', { waitUntil: 'domcontentloaded' });
await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 15000 });

let shotN = 0;
const shot = async (name) => { await page.waitForTimeout(200); const f = path.join(out, `live_${shotN++}_${name}.png`); await page.screenshot({ path: f }); console.log('  shot ' + path.relative(root, f)); };
const status = () => page.evaluate(() => document.getElementById('teach-status').textContent);
const act = () => page.evaluate(() => S.obs && S.obs.action);
const idle = (ms = 30000) => page.waitForFunction(() => S.obs && (!S.obs.action || S.obs.action.state !== 'running'), null, { timeout: ms, polling: 50 });
const bodyClick = (txt) => page.locator('#teach-body button', { hasText: txt }).first().click();
async function enable() {
  const b = await page.locator('#btn-enable').boundingBox();
  await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2);
  await page.mouse.down(); await page.waitForTimeout(900); await page.mouse.up();
  await page.waitForFunction(() => isOperational(), null, { timeout: 10000 });
}
// a joint move through the page's own lease, as the jog sticks would send (the stick UI itself is not under test)
async function jog(q) {
  await page.evaluate((q) => commandRPC('/api/v1/motion/jog', { target_positions: q }), q);
  await page.waitForTimeout(300); await idle();
}

// ------------------------------------------------------------------ capability, enable, open
ok(await page.evaluate(() => teachOn() && S.caps.teach.episodes === true && S.caps.teach.sequences === true), 'capabilities.teach v1 with episodes + sequences: ' + JSON.stringify(await page.evaluate(() => S.caps.teach)));
await enable();
await page.evaluate(() => commandRPC('/api/v1/motion/home', {})).catch(() => {});
await page.waitForTimeout(300); await idle();
await page.evaluate(() => ssetOpen(true));
await page.waitForSelector('#teach-open', { state: 'visible' });
await page.click('#teach-open');
await page.waitForFunction(() => TCH.open && TCH.loaded, null, { timeout: 5000 });
ok(await page.locator('#teach-bar').isVisible() && !(await page.locator('#tb-pose').isDisabled()), 'teach strip shown with 记点 enabled while the arm is enabled and this page can move it');

// ------------------------------------------------------------------ record a trajectory (jog)
step('record a trajectory with the teach strip while jogging');
await page.click('#teach-tab-ep');
await page.fill('#tch-task', 'wave ' + tag);
const epsBefore = (await get('/api/v1/episodes?limit=500')).episodes.length;
await page.click('#tb-rec');
await page.waitForFunction(() => !!TCH.rec, null, { timeout: 3000 });
const recSrv = (await get('/api/v1/episodes?limit=1')).recording;
ok(recSrv && recSrv.owner === await page.evaluate(() => CLIENT_ID), 'armd recording owned by this page: ' + JSON.stringify(recSrv));
await jog([0.4, -0.5, -0.7, 0.5, 0.3, 1.0, -0.8]);
await jog([0.0, -0.3, -0.5, 0.3, 0.1, 0.6, -0.8]);
await shot('recording');
await page.click('#tb-rec');
await page.waitForFunction(() => !TCH.rec, null, { timeout: 15000 });
await page.waitForTimeout(800);
const eps1 = (await get('/api/v1/episodes?limit=500')).episodes;
const ep = eps1.find((e) => e.task === 'wave ' + tag);
ok(eps1.length === epsBefore + 1 && ep, 'one episode saved with the task: ' + JSON.stringify(ep && { id: ep.id, source: ep.source, frames: ep.frames, duration_s: ep.duration_s, stale: ep.stale }));
ok(ep && ep.source === 'jog' && ep.frames > 10 && ep.outcome === 'ok', 'episode source jog, frames, outcome ok');
ok(ep && (ep.stale === false || ep.stale === null), 'fresh episode not stale (stale = ' + JSON.stringify(ep && ep.stale) + ')');
ok(await page.locator(`#teach-body li[data-id="${ep && ep.id}"]`).count() === 1, 'episode listed in the Trajectories tab');
await shot('episode_saved');

// ------------------------------------------------------------------ sequence taught with the strip, panel closed
step('new sequence; close the panel; teach with the strip and the normal controls (gripper slider), 记点 x2');
await page.click('#teach-tab-seq');
const seqName = 'live ' + tag;
await page.fill('#tch-new-name', seqName);
await bodyClick('New sequence');
await page.waitForFunction(() => !!TCH.cur, null, { timeout: 2000 });
await page.click('#teach-close');
await page.waitForTimeout(300);
ok(await page.evaluate(() => !TCH.open && TCH.teach && !document.getElementById('teach-bar').hidden), 'panel closed, the strip stays');
ok((await page.locator('#tb-seq').textContent()).includes(seqName), 'strip names the open sequence: ' + await page.locator('#tb-seq').textContent());
ok(await page.locator('#teach-bar #tb-open, #teach-bar #tb-close').count() === 0, 'no gripper open / close buttons on the strip');
// the strip covers no control and not the video (expert: a row of the stage above the video)
const geo = await page.evaluate(() => {
  const bar = document.getElementById('teach-bar'), b = bar.getBoundingClientRect();
  const els = [...document.querySelectorAll(TCH_STRIP_AVOID + ', #video-box, .ops, .side > .sc-card')].filter((e) => !bar.contains(e) && !e.contains(bar));
  const rects = els.map((e) => [e.id || e.className, e.getBoundingClientRect()]).filter(([, r]) => r.width > 0 && r.height > 0);
  return { h: b.height, n: rects.length, cover: rects.filter(([, r]) => r.left < b.right - 0.5 && r.right > b.left + 0.5 && r.top < b.bottom - 0.5 && r.bottom > b.top + 0.5).map(([n]) => n) };
});
ok(geo.h <= 48 && geo.n >= 5 && !geo.cover.length, 'strip overlaps no control / the video: ' + JSON.stringify(geo));
// gripper with the normal control: the Cartesian tab's gripper slider (servo stream), as a user drags it
async function gripSlider(pct) {
  await page.click('[data-mode="cart"]');
  await page.evaluate((v) => { const g = document.getElementById('grip-range'); g.value = String(v); g.dispatchEvent(new Event('input', { bubbles: true })); }, pct);
  await page.waitForFunction((f) => Math.abs(S.obs.positions[GRIP] - (CFG.gripClosed + f * (CFG.gripOpen - CFG.gripClosed))) < 0.05, pct / 100, { timeout: 15000, polling: 50 });
  await page.waitForTimeout(600);
  await page.click('[data-mode="joint"]');
}
await jog([0.3, -0.4, -0.6, 0.4, 0.2, 0.8, 0]);
await gripSlider(40);
const g1 = await page.evaluate(() => S.obs.positions[GRIP]);
await page.click('#tb-pose');
await page.waitForFunction(() => TCH.cur.steps.length === 1 && TCH.cur.id && !TCH.dirty, null, { timeout: 5000 });
ok(true, '记点 with the panel closed: step added and the sequence saved (id ' + await page.evaluate(() => TCH.cur.id) + ')');
await jog([-0.3, -0.5, -0.7, 0.4, -0.2, 0.8, g1]);
await gripSlider(80);
const g2 = await page.evaluate(() => S.obs.positions[GRIP]);
await page.click('#tb-pose');
await page.waitForFunction(() => TCH.cur.steps.length === 2 && !TCH.dirty, null, { timeout: 5000 });
const poses1 = (await get('/api/v1/poses')).poses;
const stepPoseIds = await page.evaluate(() => TCH.cur.steps.filter((s) => s.type === 'pose').map((s) => s.pose_id));
const stepGrip = await page.evaluate(() => TCH.cur.steps.map((s) => s.gripper));
const pg = stepPoseIds.map((id) => (poses1.find((p) => p.id === id) || {}).positions);
ok(stepPoseIds.length === 2 && stepPoseIds.every((id) => poses1.some((p) => p.id === id)), 'pose steps reference armd pose ids: ' + stepPoseIds.join(','));
ok(JSON.stringify(stepGrip) === '["pose","pose"]', '记点 steps restore the opening (gripper "pose"): ' + JSON.stringify(stepGrip));
ok(pg.every(Boolean) && Math.abs(pg[0][6] - g1) < 0.02 && Math.abs(pg[1][6] - g2) < 0.02 && Math.abs(pg[0][6] - pg[1][6]) > 0.5,
  `poses hold the opening set with the gripper slider: ${pg.map((q) => q && q[6].toFixed(3))} (measured ${g1.toFixed(3)}, ${g2.toFixed(3)})`);
ok(poses1.filter((p) => stepPoseIds.includes(p.id)).every((p) => p.stale === null || p.stale === undefined), 'fresh poses: stale null');
await shot('strip_taught');
// back to the editor from the strip: a gripper step (opening slider), a wait, the recorded trajectory
await page.click('#tb-edit');
await page.waitForFunction(() => TCH.open, null, { timeout: 3000 });
await bodyClick('+ Gripper opening');
await bodyClick('+ Wait');
await page.selectOption('#teach-body select[aria-label="+ Trajectory"]', ep.id);
await bodyClick('+ Trajectory');
const gstep = page.locator('#tch-steps > li').nth(2);
await gstep.locator('input[type=range]').fill('0.25');
const types = await page.evaluate(() => TCH.cur.steps.map((s) => s.type + (s.type === 'gripper' ? ':' + s.action + ':' + s.fraction : '')).join(','));
ok(types === 'pose,pose,gripper:move:0.25,wait,replay', 'steps: ' + types);
await page.click('#tch-save');
await page.waitForFunction(() => TCH.cur && TCH.cur.id && !TCH.dirty, null, { timeout: 5000 });
const seqId = await page.evaluate(() => TCH.cur.id);
const srvSeq = await get('/api/v1/sequences/' + seqId);
const sq = srvSeq.sequence || srvSeq;
ok(sq.steps && sq.steps.length === 5 && sq.steps[2].action === 'move' && sq.steps[2].fraction === 0.25, 'sequence saved in armd with 5 steps (rev ' + sq.rev + ')');
await shot('editor_saved');

// ------------------------------------------------------------------ run x2 with record
step('run with record on, 2 repeats');
await page.fill('#tch-repeat', '2'); await page.locator('#tch-repeat').dispatchEvent('change');
await page.check('#tch-runrec');
await page.click('#tch-run');
await page.waitForFunction(() => S.obs.action && S.obs.action.kind === 'sequence' && S.obs.action.state === 'running', null, { timeout: 5000 });
const a0 = await act();
ok(a0.sequence_id === seqId && typeof a0.id === 'number', 'parent action: numeric id ' + a0.id + ', sequence_id ' + a0.sequence_id);
ok(await page.evaluate(() => TCH.run && S.obs.action.id === TCH.run.actionId), 'the page tracks its run by the action id');
await page.waitForFunction(() => S.obs.action.repeat_index === 1 || S.obs.action.state !== 'running', null, { timeout: 120000, polling: 50 });
ok((await act()).state === 'running', 'second repeat reached: ' + JSON.stringify(await act()));
const prog = await page.evaluate(() => document.getElementById('tch-prog').textContent);
ok(/repeat 2\/2/.test(prog), 'progress text: ' + prog);
ok(await page.locator('#tch-steps > li.run').count() === 1, 'one step highlighted while running');
await shot('running_repeat2');
await idle(90000);
await page.waitForTimeout(800);
const st1 = await status();
ok(/Sequence done/.test(st1), 'status after the run: ' + st1);
const eps2 = (await get('/api/v1/episodes?limit=500')).episodes.filter((e) => e.sequence_id === seqId);
ok(eps2.length === 2 && eps2.every((e) => e.source === 'sequence') && eps2.map((e) => e.repeat_index).sort().join() === '0,1',
  'two recorded repeats: ' + JSON.stringify(eps2.map((e) => ({ src: e.source, r: e.repeat_index, task: e.task, ds: e.dataset }))));
await page.click('#teach-tab-ep');
await page.waitForTimeout(300);
ok(await page.locator('#teach-body li .tch-bd', { hasText: 'sequence' }).count() >= 2, 'run episodes shown with the source badge "sequence"');
await shot('episodes_after_run');
await page.click('#teach-tab-seq');

// ------------------------------------------------------------------ stop during a wait
step('stop during the wait');
await page.evaluate(() => { const w = TCH.cur.steps.find((s) => s.type === 'wait'); w.s = 8; teachDirty(); });
await page.uncheck('#tch-runrec');
await page.fill('#tch-repeat', '1'); await page.locator('#tch-repeat').dispatchEvent('change');
await page.click('#tch-run');
await page.waitForFunction(() => S.obs.action && S.obs.action.kind === 'sequence' && S.obs.action.state === 'running' && S.obs.action.step_type === 'wait', null, { timeout: 60000, polling: 50 });
const waitAct = await act();
await shot('waiting');
await page.click('#tch-stop');
await idle(5000);
await page.waitForTimeout(500);
const ended = await page.evaluate((id) => [S.obs.action, S.obs.previous_action].find((a) => a && a.id === id), waitAct.id);
ok(ended && ended.state !== 'done' && ended.state !== 'running', 'stopped in the wait: ' + JSON.stringify(ended && { state: ended.state, code: ended.code, step: ended.step_index, failed_step: ended.failed_step }));
const st2 = await status();
ok(/Sequence ended/.test(st2), 'status after stop: ' + st2);
await shot('stopped');

// ------------------------------------------------------------------ IN_USE, rename
step('delete a used pose (IN_USE), rename a pose');
await page.click('#teach-tab-pose');
await page.waitForTimeout(300);
const p0 = (await get('/api/v1/poses')).poses.find((p) => p.id === stepPoseIds[0]);
const row = page.locator(`#teach-body li[data-id="${p0.id}"]`);
ok(/Used by: /.test(await row.locator('.tch-sub').textContent()), 'pose row shows "used by"');
await row.locator('button.tch-del').click();
await row.locator('button.tch-del').click();
await page.waitForTimeout(500);
const st3 = await status();
ok(st3.includes('Used by the sequences') && st3.includes(seqName), 'IN_USE text with the sequence name: ' + st3);
ok((await get('/api/v1/poses')).poses.some((p) => p.id === p0.id), 'pose still in armd');
await row.locator('button', { hasText: 'Rename' }).click();
const newName = 'renamed_' + tag;
await page.fill('#tch-rename', newName);
await page.locator(`#teach-body li[data-id="${p0.id}"] button`, { hasText: 'Save' }).click();
await page.waitForTimeout(600);
const p0b = (await get('/api/v1/poses')).poses.find((p) => p.id === p0.id);
ok(p0b && p0b.name === newName, 'renamed in armd, id kept: ' + JSON.stringify(p0b && { id: p0b.id, name: p0b.name }));
await shot('poses_renamed');
await page.click('#teach-tab-seq');
await page.waitForTimeout(300);
const t0 = await page.locator('#tch-steps > li').first().locator('b').textContent();
ok(t0.includes(newName), 'step title follows the rename: ' + t0);

// ------------------------------------------------------------------ software zero -> stale -> allow_stale
const poseName = Object.fromEntries((await get('/api/v1/poses')).poses.map((p) => [p.id, p.name]));
step('software zero from Tools > Zero, then stale data and the allow_stale confirm');
// J1 0.1 rad off the rest pose (inside its 0.15 rad rest tolerance): torque off in place there (the zero tool's park
// would move the arm onto the reference, so the zero would change nothing)
await jog([0.1, 0, 0, 0, 0, 0, 0]);
await page.waitForTimeout(1500);
await page.evaluate(() => api('/api/v1/torque', { enable: false }, { quiet: true }));
await page.waitForFunction(() => S.obs && !S.obs.is_torqued, null, { timeout: 10000 });
await page.evaluate(() => teachOpen(false));
await page.evaluate(() => ssetOpen(true));
await page.waitForSelector('#tool-motor_zero', { state: 'visible' });
await page.click('#tool-motor_zero');
await page.waitForFunction(() => MZ.open, null, { timeout: 3000 });
if (await page.locator('#mz-m-software').count()) await page.check('#mz-m-software');
await page.waitForTimeout(300);
// joint 1 (0.1 rad off its reference): unselect the others, select J1
await page.evaluate(() => { MZ.sel = new Set([1]); MZ.armedUntil = 0; mzRender(); });
if (await page.locator('#mz-farbox').isVisible()) await page.check('#mz-force');
await page.click('#mz-go');
await page.waitForFunction(() => Date.now() < MZ.armedUntil || MZ.status, null, { timeout: 5000 });
const mzSt = await page.evaluate(() => MZ.status);
ok(!mzSt, 'dry run accepted' + (mzSt ? ': ' + mzSt : ''));
await page.click('#mz-go');
await page.waitForFunction(() => MZ.res || !MZ.busy, null, { timeout: 15000 });
const mzRes = await page.evaluate(() => MZ.res && { ok: MZ.res.ok, inv: MZ.res.invalidated, method: MZ.res.method });
ok(mzRes && mzRes.inv && (mzRes.inv.poses_stale || []).length >= 2 && stepPoseIds.every((id) => mzRes.inv.poses_stale.includes(poseName[id])), 'software zero marked the poses stale: ' + JSON.stringify(mzRes) + ' ' + await page.evaluate(() => MZ.status));
await shot('zero_result');
const poses2 = (await get('/api/v1/poses')).poses.filter((p) => stepPoseIds.includes(p.id));
ok(poses2.every((p) => p.stale && typeof p.stale === 'object' && p.stale.reason && p.stale.at), 'armd poses: stale {reason, at}: ' + JSON.stringify(poses2[0] && poses2[0].stale));
const eps3 = (await get('/api/v1/episodes?limit=500')).episodes;
const ep3 = eps3.find((e) => e.id === ep.id);
ok(ep3 && ep3.stale === true, 'armd episode recorded before the zero: stale true (episodes keep a boolean): ' + JSON.stringify(ep3 && ep3.stale));
await page.evaluate(() => { mzOpen(false); });
await enable();
await page.evaluate(() => teachOpen(true, { tab: 'pose' }));
await page.waitForTimeout(800);
ok((await Promise.all(stepPoseIds.map((id) => page.locator(`#teach-body li[data-id="${id}"] .tch-bd.stale`).count()))).every((n) => n === 1), 'the two pose rows show the old-zero badge');
const title = await page.locator(`#teach-body li[data-id="${stepPoseIds[0]}"] .tch-bd.stale`).getAttribute('title');
ok(title && title.includes('zero'), 'badge title from stale.reason: ' + title);
await page.click('#teach-tab-seq');
await page.waitForTimeout(300);
ok(await page.locator('#tch-steps .tch-bd.stale').count() >= 2, 'pose steps show the old-zero badge');
await page.click('#teach-tab-ep');
await page.waitForTimeout(300);
ok(await page.locator(`#teach-body li[data-id="${ep.id}"] .tch-bd.stale`).count() === 1, 'the trajectory row shows the old-zero badge');
await page.click('#teach-tab-seq');
await page.waitForTimeout(300);
await page.evaluate(() => { const w = TCH.cur.steps.find((s) => s.type === 'wait'); w.s = 0.5; teachDirty(); });
await page.click('#tch-run');
await page.waitForFunction(() => !document.getElementById('teach-ask').hidden || (TCH.run && S.obs.action && S.obs.action.kind === 'sequence'), null, { timeout: 5000 });
const ask = await page.locator('#teach-ask').isVisible();
ok(ask, 'DATA_STALE -> confirm shown: ' + await status());
await shot('stale_confirm');
if (ask) {
  await page.click('#teach-ask-yes');
  await page.waitForFunction(() => S.obs.action && S.obs.action.kind === 'sequence' && S.obs.action.state === 'running', null, { timeout: 5000 });
  ok(true, 'run started after the confirm (allow_stale)');
  await idle(60000);
  await page.waitForTimeout(600);
  ok(/Sequence done/.test(await status()), 'stale run finished: ' + await status());
}
await shot('stale_run_done');

// ------------------------------------------------------------------ teardown
await page.evaluate(() => commandRPC('/api/v1/motion/home', {})).catch(() => {});
await page.waitForTimeout(300); await idle().catch(() => {});
const errsReal = errs.filter((e) => !/favicon|stream|video|WebSocket|ERR_CONNECTION|404|detector|9091|status of 50\d/i.test(e));
ok(errsReal.length === 0, 'no page errors' + (errsReal.length ? ': ' + errsReal.slice(0, 5).join(' | ') : ''));
await browser.close();
console.log(bad ? `${bad}/${checks} checks FAILED` : `${checks} checks OK`);
process.exit(bad ? 1 : 0);
