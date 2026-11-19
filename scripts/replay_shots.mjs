#!/usr/bin/env node
// Cockpit episode list and replay against a running armd (mock follower works):
//   armd --follower mock --config kits/recamera-b601dm/profile/rebot-b601dm.json --port 18950 --episodes-dir /tmp/eps
//   ARMD=http://127.0.0.1:18950 PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/replay_shots.mjs
// Expert mode at 1440x900: enable, record a joint move (with capabilities.teach through the teach recorder, the
// episode id is armd's ULID), save, the list shows it, replay at
// 0.5x with progress, stop mid-replay, replay while recording (MOTION_BUSY toast), replay at 1x to the end,
// delete with the in-page confirm. Then the card layout at phone landscape 844x390 and portrait 390x844,
// in zh and en. Writes PNGs (default .cache/replay-shots/) and exits non-zero on a failed check.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/replay-shots');
fs.mkdirSync(out, { recursive: true });
const base = (process.env.ARMD || 'http://127.0.0.1:18950').replace(/\/$/, '') + '/';

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; console.log((c ? '  ok   ' : '  FAIL ') + m); if (!c) bad++; };
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
const errs = [];
// a recording left running by an earlier run would refuse the start below
await fetch(base + 'api/v1/dataset/record/discard', { method: 'POST' }).catch(() => {});

async function open(viewport, lang) {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: viewport.width < 900 ? 2 : 1, locale: lang === 'en' ? 'en-GB' : 'zh-CN' });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  page.on('pageerror', (e) => errs.push(String(e)));
  await page.goto(base + '?ui=expert', { waitUntil: 'domcontentloaded' });
  await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 10000 });
  await page.waitForFunction(() => !document.getElementById('ep').hidden, null, { timeout: 5000 });
  return { ctx, page };
}
const shot = async (page, name) => {
  await page.waitForTimeout(250);
  const f = path.join(out, name + '.png');
  // the bottom of the viewport: phone layouts keep the safety card stuck at the top
  await page.evaluate(() => document.getElementById('ep').scrollIntoView({ block: 'end' }));
  await page.screenshot({ path: f });
  console.log('  shot ' + f);
};
const action = (page) => page.evaluate(() => S.obs && S.obs.action);
const toasts = (page) => page.evaluate(() => [...document.querySelectorAll('#toasts .toast')].map((t) => t.textContent));
async function enable(page) {
  // the page closed before this one lost its lease: armd parks the arm after the grace period (Armed, then Moving);
  // a hold on the button while the arm is still operational would stow it instead of enabling
  await page.waitForFunction(() => !isOperational() && !(S.obs.action && S.obs.action.state === 'running'), null, { timeout: 30000 });
  const b = await page.locator('#btn-enable').boundingBox();
  await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2);
  await page.mouse.down();
  await page.waitForTimeout(900);
  await page.mouse.up();
  await page.waitForFunction(() => isOperational(), null, { timeout: 10000 });
}
const waitAction = (page, pred, ms = 20000) => page.waitForFunction(pred, null, { timeout: ms, polling: 50 });

// ---------------------------------------------------------------- flow at 1440x900
{
  const { ctx, page } = await open({ width: 1440, height: 900 }, 'zh');
  await enable(page);
  await page.waitForTimeout(500);
  const before = await page.evaluate(() => EP.list.length), beforeIds = await page.evaluate(() => EP.list.map((e) => e.id));
  // teach v1 (capabilities.teach): the record button records into the teach store and armd names the episode (ULID);
  // without teach the page names it (#rec-id)
  const teach = await page.evaluate(() => teachOn());
  console.log('  teach v1 ' + teach);

  // record: start, move to the inspect preset, save
  let id = 'ui_' + Date.now();
  if (!teach) await page.fill('#rec-id', id);
  await page.click('#btn-rec');
  await page.waitForFunction(() => S.rec.active, null, { timeout: 3000 });
  // the arm moves by a joint jog through the page's lease (the move itself is not under test)
  await page.evaluate(() => commandRPC('/api/v1/motion/jog', { target_positions: [0.5, -0.6, -0.8, 0.6, 0.3, 1.2, -1.0] }));
  await page.waitForTimeout(400);
  await waitAction(page, () => S.obs.action && S.obs.action.state !== 'running', 20000);
  await page.waitForTimeout(500);
  // replay while recording: armd refuses (MOTION_BUSY; only a sequence run replays into its own recording)
  async function replayWhileRecording(go) {
    await go.click();
    await page.waitForTimeout(500);
    const t = await toasts(page);
    ok(t.some((x) => x.includes('MOTION_BUSY')), 'replay while recording refused with MOTION_BUSY: ' + JSON.stringify(t));
    await shot(page, '1_reject_while_recording');
  }
  if (before > 0) await replayWhileRecording(page.locator('#ep-list li .ep-go').first());
  await page.click('#btn-rec');
  await page.waitForFunction(() => !S.rec.active, null, { timeout: 5000 });
  await page.waitForFunction((n) => EP.list.length === n + 1, before, { timeout: 5000 });
  if (teach) id = await page.evaluate((b) => EP.list.find((e) => !b.includes(e.id)).id, beforeIds);
  const row = page.locator(`#ep-list li[data-id="${id}"]`);
  ok(await row.count() === 1, 'saved episode listed after save');
  const small = await row.locator('small').textContent();
  ok(/\d+(\.\d)? s/.test(small), 'row shows duration and time: ' + small);
  if (before === 0) {
    // replay-while-recording check needs an episode in the list: record again quickly
    await page.click('#btn-rec');
    await page.waitForFunction(() => S.rec.active, null, { timeout: 3000 });
    await page.waitForTimeout(300);
    await replayWhileRecording(row.locator('.ep-go'));
    await page.click('#btn-discard');
    await page.waitForFunction(() => !S.rec.active, null, { timeout: 3000 });
  }
  await shot(page, '2_list_after_save');

  // back home so the replay has an approach, then replay at 0.5x and stop mid-way
  await page.click('#btn-home');
  await page.waitForTimeout(300);
  await waitAction(page, () => S.obs.action && S.obs.action.state !== 'running', 20000);
  await page.waitForTimeout(600);
  ok(await page.evaluate(() => document.querySelector('#ep-speed .on').dataset.speed) === '0.5', 'default speed 0.5x');
  await row.locator('.ep-go').click();
  await waitAction(page, () => S.obs.action && S.obs.action.kind === 'replay' && S.obs.action.state === 'running', 5000);
  ok(!(await page.locator('#ep-play').isHidden()), 'progress bar visible while playing');
  await waitAction(page, () => S.obs.action.phase === 'replay' && S.obs.action.progress > 0.25, 20000);
  await page.waitForTimeout(200);
  const prog = await page.evaluate(() => {
    const bar = document.getElementById('ep-bar'), track = bar.parentElement.getBoundingClientRect();
    return [S.obs.action.phase, S.obs.action.progress, bar.getBoundingClientRect().width / track.width, document.getElementById('ep-play-text').textContent];
  });
  ok(Math.abs(prog[2] - prog[1]) < 0.08, 'drawn bar fraction follows progress: ' + JSON.stringify(prog));
  await shot(page, '3_playing_half_speed');
  await page.click('#ep-stop');
  await waitAction(page, () => S.obs.action.kind === 'replay' && S.obs.action.state !== 'running', 3000);
  const stopped = await action(page);
  ok(stopped.state === 'cancelled' && stopped.code === 'STOPPED', 'stop cancels: ' + stopped.state + ' ' + stopped.code);
  await page.waitForTimeout(400);
  ok(await page.locator('#ep-play').isHidden(), 'progress bar hidden after stop');
  await shot(page, '4_stopped');

  // 1x to the end
  await page.click('#ep-speed button[data-speed="1"]');
  await row.locator('.ep-go').click();
  await waitAction(page, () => S.obs.action && S.obs.action.kind === 'replay' && S.obs.action.state === 'running', 5000);
  ok(await page.evaluate(() => document.querySelector('#ep-list li.playing .ep-go').disabled), 'play buttons disabled while playing');
  await waitAction(page, () => S.obs.action.kind === 'replay' && S.obs.action.state !== 'running', 30000);
  const done = await action(page);
  ok(done.state === 'done', 'replay at 1x done: ' + done.state + ' ' + (done.code || ''));
  await page.waitForTimeout(300);
  await shot(page, '5_done_1x');

  // delete: first click arms, second deletes
  await row.locator('.ep-del').click();
  ok(await row.locator('.ep-del.armed').count() === 1, 'delete asks for confirmation in the page');
  await shot(page, '6_delete_confirm');
  await row.locator('.ep-del').click();
  await page.waitForFunction((x) => !EP.list.some((e) => e.id === x), id, { timeout: 5000 });
  ok(await row.count() === 0, 'episode removed from the list');
  await shot(page, '7_deleted');
  await ctx.close();
}

// ---------------------------------------------------------------- layouts and languages
{
  // a saved episode to show
  const { ctx, page } = await open({ width: 1440, height: 900 }, 'zh');
  await enable(page);
  const teach = await page.evaluate(() => teachOn()), n0 = await page.evaluate(() => EP.list.length);
  if (!teach) await page.fill('#rec-id', 'layout_check');
  await page.click('#btn-rec');
  await page.waitForTimeout(800);
  await page.click('#btn-rec');
  await page.waitForFunction(([tv, n]) => (tv ? EP.list.length > n : EP.list.some((e) => e.id === 'layout_check')), [teach, n0], { timeout: 5000 });
  await ctx.close();
}
for (const [name, vp] of [['desktop', { width: 1440, height: 900 }], ['land', { width: 844, height: 390 }], ['port', { width: 390, height: 844 }]]) {
  for (const lang of ['zh', 'en']) {
    const { ctx, page } = await open(vp, lang);
    await page.waitForFunction(() => EP.loaded, null, { timeout: 5000 });
    await page.evaluate(() => document.getElementById('ep').scrollIntoView({ block: 'end' }));
    const r = await page.evaluate(() => {
      const box = (s) => { const e = document.querySelector(s); const b = e.getBoundingClientRect(); return { l: b.left, r: b.right, w: b.width }; };
      const card = box('#ep');
      const rows = [...document.querySelectorAll('#ep-list li')].map((li) => { const b = li.getBoundingClientRect(); return { l: b.left, r: b.right }; });
      const btns = [...document.querySelectorAll('#ep-list .btn, #ep-head .btn, .ep-head .btn')].map((b) => { const x = b.getBoundingClientRect(); return { l: x.left, r: x.right, h: x.height }; });
      return { card, rows, btns, vw: innerWidth, sw: document.documentElement.scrollWidth };
    });
    ok(r.card.w > 200 && r.card.r <= r.vw + 1 && r.card.l >= -1, `${name}/${lang} card within the viewport ${JSON.stringify(r.card)} vw ${r.vw}`);
    ok(r.rows.every((x) => x.l >= r.card.l - 1 && x.r <= r.card.r + 1), `${name}/${lang} rows inside the card`);
    ok(r.btns.every((x) => x.r <= r.vw + 1 && x.h >= 30), `${name}/${lang} buttons visible and >= 30 px`);
    ok(r.sw <= r.vw + 1, `${name}/${lang} no horizontal scroll (${r.sw} / ${r.vw})`);
    const texts = await page.evaluate(() => [document.querySelector('[data-i18n="ep.title"]').textContent, document.querySelector('#ep-list .ep-go').textContent]);
    ok(lang === 'en' ? texts[0] === 'Saved episodes' && texts[1] === 'Play' : texts[0] === '已保存的录制' && texts[1] === '回放', `${name}/${lang} strings ${JSON.stringify(texts)}`);
    await shot(page, `8_${name}_${lang}`);
    await ctx.close();
  }
}
await browser.close();
ok(errs.length === 0, 'no page errors: ' + errs.join(' | '));
console.log(`${checks - bad}/${checks} checks passed`);
process.exit(bad ? 1 : 0);
