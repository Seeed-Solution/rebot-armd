#!/usr/bin/env node
// Operator-only controls in the cockpit (data-op), end to end against a mock armd with an operator PIN:
//   cargo run -- --follower mock --config kits/recamera-b601dm/profile/rebot-b601dm.json --port 18971 --operator-pin 246810 &
//   BASE=http://127.0.0.1:18971/ PIN=246810 PLAYWRIGHT=/path/to/playwright/index.mjs OUT=/dir node scripts/operator_gate_shots.mjs
// SRC=src/api serves index.html and calib_wizard.js from disk instead of the binary (no rebuild while editing).
// Phone 390x844 and tablet 1024x768, zh and en, vision stubbed behind armd (API 1: armd answers a POST first,
// 403 without an operator session; the stub after that):
//  1. locked: operator-only controls show a lock (::after), the badge says what the lock means;
//  2. camera calibration entry: the dialog explains before the wizard opens, Cancel reads "Not now"; Not now opens the
//     wizard, its vision switch is refused and the toast names the cause with an Unlock button (no bare "403");
//     Unlock + PIN switches the vision mode by itself (stub sees the POST);
//  3. lock from the dialog; a tap on ArUco opens the dialog (vision reason) instead of a refusal; the PIN runs the
//     switch; pose save while locked: dialog, Cancel saves nothing and shows no raw error, Save + PIN saves;
//  4. expired session (cookie dropped, page still shows unlocked): a vision switch is refused, the badge goes back
//     to locked, the dialog opens, the PIN runs the switch;
//  5. the badge shows the minutes left in the last 5 minutes of a session;
//  6. a failed fine calibration (stubbed result): no "can be applied" accuracy note, one line per failed cause;
//  7. the enable switch label in the E-stop state fits the switch and breaks only between words;
//  0. (tablet context) the header at 360-1440 px, simple and expert: no overlap, nothing off screen, title not clipped;
//  the unlock after an expired session removes the "expired" and Unlock toasts.
// The PIN dialog, the calibration drawer and the failure box are checked after their animations ended: the element
// at their centre and four inner points belongs to them.
// PNGs to OUT. Exits non-zero if a check fails.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/operator-gate-shots');
const base = process.env.BASE || 'http://127.0.0.1:18971/';
const PIN = process.env.PIN || '246810';
const SRC = process.env.SRC ? path.resolve(process.env.SRC) : null;
fs.mkdirSync(out, { recursive: true });
let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } return c; };
const origin = new URL(base).origin;
async function lockServer() { await fetch(new URL('/api/v1/auth/lock', base), { method: 'POST', headers: { Origin: origin } }).catch(() => {}); }
async function waitFor(page, fn, arg, ms = 8000) { try { await page.waitForFunction(fn, arg, { timeout: ms }); return true; } catch (_) { return false; } }
// a page served from disk counts as a public origin for Chromium's local network checks: allow its sockets
const browser = await chromium.launch({ headless: true, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader',
  ...(SRC ? ['--disable-features=LocalNetworkAccessChecks,BlockInsecurePrivateNetworkRequests,PrivateNetworkAccessChecks'] : [])] });
const toasts = (page) => page.evaluate(() => [...document.querySelectorAll('#toasts .toast')].map((x) => ({ text: x.innerText.replace(/\s*\n\s*/g, ' | '), act: !!x.querySelector('.toast-act') })));
const modal = (page) => page.evaluate(() => ({ open: !$('op-modal').hidden, why: $('op-why').textContent, retry: $('op-retry').textContent, cancel: $('op-cancel').textContent }));
const clearToasts = (page) => page.evaluate(() => document.querySelectorAll('#toasts .toast').forEach((x) => x.remove()));
const lockGlyph = (page, sel) => page.evaluate((s) => { const e = document.querySelector(s); return e ? getComputedStyle(e, '::after').content : null; }, sel);
// wait until fade-in / slide-in animations have ended (screenshots taken earlier show the dialog half transparent)
const settle = (page) => page.waitForFunction(() => document.getAnimations().every((a) => a.playState !== 'running' || a.effect?.getComputedTiming().iterations === Infinity), null, { timeout: 5000 }).catch(() => {});
// the element on top at the centre and four inner points of `sel` belongs to it (nothing of the page drawn over it)
const onTop = (page, sel) => page.evaluate((s) => {
  const d = document.querySelector(s); if (!d) return ['missing'];
  const r = d.getBoundingClientRect();
  return [[0.5, 0.5], [0.5, 0.2], [0.5, 0.8], [0.2, 0.5], [0.8, 0.5]].map(([x, y]) => {
    const h = document.elementFromPoint(r.left + r.width * x, r.top + r.height * y);
    return h && d.contains(h) ? 'in' : h ? (h.id || String(h.className) || h.tagName) : 'none';
  }).filter((v) => v !== 'in');
}, sel);
async function enterPin(page) { await page.fill('#op-pin', PIN); await page.press('#op-pin', 'Enter'); return waitFor(page, () => $('op-modal').hidden && S.auth.role === 'operator'); }

// header at 360-1440 px, simple and expert: no two header items overlap, nothing past the right edge, the title not clipped
const HEADER_W = [360, 390, 430, 560, 640, 768, 820, 900, 1000, 1024, 1180, 1280, 1366, 1440];
const headerCheck = (page) => page.evaluate(() => {
  const vis = (e) => { const c = getComputedStyle(e), q = e.getBoundingClientRect(); return c.display !== 'none' && c.visibility !== 'hidden' && q.width > 0 && !e.hidden; };
  const boxes = [...document.querySelector('.topbar').querySelectorAll(':scope > *, .brand-text, .head-stats > *')].filter(vis)
    .map((e) => ({ id: e.id || String(e.className).split(' ')[0] || e.tagName, r: e.getBoundingClientRect(), e }));
  const ov = [];
  for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++) {
    const A = boxes[i], B = boxes[j]; if (A.e.contains(B.e) || B.e.contains(A.e)) continue;
    const x = Math.min(A.r.right, B.r.right) - Math.max(A.r.left, B.r.left), y = Math.min(A.r.bottom, B.r.bottom) - Math.max(A.r.top, B.r.top);
    if (x > 1 && y > 1) ov.push(A.id + 'x' + B.id);
  }
  const bt = document.querySelector('.brand-text');
  return { ov, out: boxes.filter((x) => x.r.right > innerWidth + 0.5).map((x) => x.id), clip: bt && vis(bt) ? bt.scrollWidth - bt.clientWidth : 0 };
});
const VIEWS = [['phone', { width: 390, height: 844 }], ['tablet', { width: 1024, height: 768 }]];
for (const lang of ['zh', 'en']) for (const [vname, view] of VIEWS) {
  const name = `${vname}_${lang}`;
  await lockServer();
  const ctx = await browser.newContext({ viewport: view, deviceScaleFactor: 1, hasTouch: view.width < 1000, locale: lang === 'zh' ? 'zh-CN' : 'en-US' });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  page.on('pageerror', (e) => { console.log('  pageerror ' + e.message); ok(false, name + ' pageerror'); });
  if (SRC) {
    await page.route((u) => u.pathname === '/', (r) => r.fulfill({ status: 200, contentType: 'text/html; charset=utf-8', body: fs.readFileSync(path.join(SRC, 'index.html'), 'utf8') }));
    await page.route((u) => u.pathname === '/assets/calib_wizard.js', (r) => r.fulfill({ status: 200, contentType: 'application/javascript', body: fs.readFileSync(path.join(SRC, 'calib_wizard.js'), 'utf8') }));
  }
  const vmPosts = [];
  await page.route(/\/api\/v1\/(vision|detection)\//, async (route) => {
    const req = route.request(); const u = new URL(req.url()).pathname;
    const f = (o, s = 200) => route.fulfill({ status: s, contentType: 'application/json', body: JSON.stringify(o) });
    if (req.method() !== 'GET') { const r = await route.fetch().catch(() => null); if (r && r.status() === 403) return route.fulfill({ response: r }); }
    const vm = { api_version: 1, mode: 'yolo', modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: 1, trigger: true, available: { yolo: true, aruco: true } };
    if (u === '/api/v1/vision/mode' && req.method() === 'POST') { const b = JSON.parse(req.postData() || '{}'); vmPosts.push(b.mode); return f({ ...vm, mode: b.mode, enabled: !!b.enabled }); }
    if (u === '/api/v1/vision/mode') return f(vm);
    if (u === '/api/v1/detection/inference') return f({ enabled: false, fps: 1, active_fps: 5, idle_fps: 1, mode: 'yolo' });
    if (u === '/api/v1/detection/latest') return f({ timestamp: 1, frame_id: 1, boxes: [], resolution: [1280, 720] });
    return f({ error: 'not found' }, 404);
  });
  await page.goto(base + '?ui=expert', { waitUntil: 'domcontentloaded' });
  ok(await waitFor(page, () => S.obs && S.dof > 0 && S.auth.enforced && VM.state === 'ok', null, 15000), name + ' telemetry, auth enforced, vision stub');
  await page.waitForTimeout(400);

  // 0. header fit (once per language, on the tablet context; auth enforced, the operator chip shown)
  if (vname === 'tablet') {
    for (const ui of ['simple', 'expert']) {
      await page.evaluate((u) => setUi(u), ui);
      for (const w of HEADER_W) {
        await page.setViewportSize({ width: w, height: 800 }); await page.waitForTimeout(120);
        const h = await headerCheck(page);
        ok(!h.ov.length && !h.out.length && h.clip <= 0, `${name} header ${ui} ${w}px ${JSON.stringify(h)}`);
        if (w === 1024 || w === 768) await page.screenshot({ path: path.join(out, `${name}_0_header_${ui}_${w}.png`), clip: { x: 0, y: 0, width: w, height: 64 } });
      }
    }
    await page.evaluate(() => setUi('expert'));
    await page.setViewportSize(view); await page.waitForTimeout(200);
  }
  // 1. lock marks while locked
  ok(await page.evaluate(() => document.body.classList.contains('op-locked')), name + ' body.op-locked');
  ok(/🔒/.test(await lockGlyph(page, '#btn-calib') || ''), name + ' calibration button shows a lock');
  ok(/🔒/.test(await lockGlyph(page, '#vmode button[data-vm="aruco"]') || ''), name + ' ArUco shows a lock');
  ok(!/🔒/.test(await lockGlyph(page, '#vmode button[data-vm="off"]') || ''), name + ' Off shows no lock');
  ok(/🔒/.test(await page.getAttribute('#btn-op', 'title')), name + ' badge tooltip explains the lock mark');
  await page.screenshot({ path: path.join(out, `${name}_1_locked.png`) });

  // 2. calibration entry: dialog first; Not now opens the wizard; the refused switch names cause + Unlock; unlock runs it
  await page.click('#btn-calib');
  let m = await modal(page);
  ok(m.open && m.why.length > 20 && !m.retry, `${name} calibration entry explains before opening ${JSON.stringify(m)}`);
  ok(m.cancel === (lang === 'zh' ? '先不解锁' : 'Not now'), `${name} cancel reads Not now (${m.cancel})`);
  ok(await page.evaluate(() => $('cwiz').hidden), name + ' wizard not open while the dialog is up');
  await settle(page);
  let cover = await onTop(page, '.op-dialog');
  ok(!cover.length, `${name} PIN dialog on top of the page ${JSON.stringify(cover)}`);
  await page.screenshot({ path: path.join(out, `${name}_2_calib_entry.png`) });
  await page.click('#op-cancel');
  ok(await waitFor(page, () => !$('cwiz').hidden), name + ' Not now opens the wizard');
  await settle(page);
  cover = await onTop(page, '#cwiz-panel');
  ok(!cover.length, `${name} calibration drawer on top of the page ${JSON.stringify(cover)}`);
  ok(await waitFor(page, () => [...document.querySelectorAll('#toasts .toast')].some((x) => x.querySelector('.toast-act'))), name + ' refused vision switch: toast with Unlock');
  await page.waitForTimeout(500);
  ok(await page.evaluate(() => { const b = document.querySelector('#toasts .toast-act'), r = b.getBoundingClientRect(), h = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
    return h === b && r.right <= innerWidth && r.left >= 0; }), name + ' toast Unlock button on top and inside the viewport');
  const ts = await toasts(page);
  const vt = ts.find((x) => x.act) || { text: '' };
  ok(!/\b403\b/.test(vt.text) && vt.text.includes('ArUco') && vt.text.includes(lang === 'zh' ? '操作员权限' : 'operator access'), `${name} toast names cause and fix ${JSON.stringify(ts)}`);
  await page.screenshot({ path: path.join(out, `${name}_3_calib_refused_toast.png`) });
  const before = vmPosts.length;
  await page.click('#toasts .toast-act');
  ok((await modal(page)).open, name + ' toast Unlock opens the dialog');
  ok(await enterPin(page), name + ' PIN unlocks');
  ok(await waitFor(page, () => VM.mode === 'both' || VM.mode === 'aruco'), name + ' vision switched after the unlock');
  ok(vmPosts.length > before, `${name} vision POST after the unlock (${vmPosts.join(',')})`);
  ok(await page.evaluate(() => !document.body.classList.contains('op-locked')), name + ' unlocked: no lock marks');
  await page.screenshot({ path: path.join(out, `${name}_4_calib_unlocked.png`) });
  await page.evaluate(() => cwOpen(false));
  await page.waitForTimeout(400);

  // 3. lock; ArUco tap -> dialog (vision reason) -> PIN -> switch; collision switch -> dialog, Cancel -> unchanged
  await page.click('#btn-op'); await page.click('#op-lock');
  ok(await waitFor(page, () => S.auth.role !== 'operator' && document.body.classList.contains('op-locked')), name + ' locked again from the dialog');
  await clearToasts(page);
  await page.evaluate(() => { const b = $('btn-vis'); if (b && getComputedStyle(b).display !== 'none' && !document.body.classList.contains('vis-open')) b.click(); });
  await page.waitForTimeout(200);
  const n0 = vmPosts.length;
  await page.click('#vmode button[data-vm="aruco"]', { force: true });
  m = await modal(page);
  ok(m.open && /ArUco/.test(m.why) && m.retry.length > 0, `${name} ArUco while locked: dialog with the vision reason ${JSON.stringify(m)}`);
  ok(vmPosts.length === n0, name + ' nothing sent before the unlock');
  ok(await enterPin(page), name + ' PIN unlocks (vision)');
  await page.waitForTimeout(800);
  ok(vmPosts.length > n0, `${name} the tap ran after the unlock (${vmPosts.slice(n0).join(',')})`);
  await page.evaluate(() => document.body.classList.remove('vis-open'));
  await page.click('#btn-op'); await page.click('#op-lock');
  await waitFor(page, () => S.auth.role !== 'operator');
  // pose save (WS command, needs the lease too): Cancel sends nothing and shows no raw error; Save + PIN saves
  await page.evaluate(() => psetOpen(true));
  const pname = `gate_${vname}_${lang}_${Date.now() % 1e6}`;
  await page.fill('#pose-name', pname);
  await page.click('#pose-save');
  m = await modal(page);
  ok(m.open && m.why.length > 0 && m.retry.length > 0, `${name} pose save while locked: dialog ${JSON.stringify(m)}`);
  await page.screenshot({ path: path.join(out, `${name}_5_pose_locked.png`) });
  await page.click('#op-cancel');
  await page.waitForTimeout(400);
  ok(!(await toasts(page)).some((x) => /OPERATOR_REQUIRED|403/.test(x.text)), name + ' no raw error toast after Cancel');
  const has = async () => (await (await fetch(new URL('/api/v1/poses', base))).json()).poses.some((p) => p.name === pname);
  ok(!(await has()), name + ' Cancel saved nothing');
  await page.click('#pose-save');
  ok(await enterPin(page), name + ' PIN unlocks (pose)');
  await page.waitForTimeout(1200);
  ok(await has(), `${name} pose saved after the unlock ${JSON.stringify({ st: await page.evaluate(() => $('pose-status').textContent), t: await toasts(page) })}`);
  await page.evaluate(() => psetOpen(false));

  // 4. session gone while the page shows it unlocked: refused, badge back to locked, dialog, PIN, switch
  if (!(await page.evaluate(() => S.auth.role === 'operator'))) { await page.click('#btn-op'); ok(await enterPin(page), name + ' unlock for the expiry check'); }
  await ctx.clearCookies();
  await clearToasts(page);
  const n1 = vmPosts.length;
  await page.evaluate(() => setVisionMode(VM.mode === 'aruco' ? 'yolo' : 'aruco'));
  ok(await waitFor(page, () => !$('op-modal').hidden && S.auth.role !== 'operator' && document.body.classList.contains('op-locked')), name + ' expired session: badge locked, dialog open');
  ok((await toasts(page)).some((x) => x.text.includes(lang === 'zh' ? '过期' : 'expired')), name + ' expired session: toast says so');
  ok(await enterPin(page), name + ' PIN unlocks (expiry)');
  const stale = (await toasts(page)).filter((x) => x.act || x.text.includes(lang === 'zh' ? '过期' : 'expired'));
  ok(!stale.length, `${name} unlock dismisses the "expired" and Unlock toasts ${JSON.stringify(stale)}`);
  await page.waitForTimeout(800);
  ok(vmPosts.length > n1, `${name} switch ran after the new unlock (${vmPosts.slice(n1).join(',')})`);

  // 5. minutes left on the badge in the last 5 minutes
  await page.evaluate(() => { S.auth.expiresAt = Date.now() + 4 * 60000 - 1000; renderOpBadge(); });
  const badge = await page.evaluate(() => ({ text: $('btn-op').textContent, title: $('btn-op').title }));
  ok(/4/.test(badge.text) && /4/.test(badge.title), `${name} badge shows minutes left ${JSON.stringify(badge)}`);
  await page.screenshot({ path: path.join(out, `${name}_6_badge_minutes.png`), clip: { x: 0, y: 0, width: view.width, height: 120 } });
  // 6. failed fine calibration (solve result stubbed): no "can be applied" accuracy note, one line per failed cause,
  // and the statement that the calibration on the arm is unchanged
  await page.evaluate(() => cwOpen(true)); await page.waitForTimeout(2000); await page.evaluate(() => cwOpen(false));
  const fine = await page.evaluate(() => {
    CWZ.mode = 'fine'; CWZ.step = 'solve';
    CWZ.solve = { ok: false, gates: [{ key: 'views', ok: true, value: 70, limit: 20, unit: 'count' }, { key: 'rms', ok: false, value: 0.59, limit: 0.5, unit: 'px' },
      { key: 'kin_heldout_ab', ok: true, value: 3.1, limit: 5, unit: 'mm' }, { key: 'kin_heldout_ba', ok: false, value: 5.2, limit: 5, unit: 'mm' }],
      kin: { warnings: [{ key: 'accuracy', mm: 5.2, limit: 3 }] } };
    cwOpen(true);
    document.querySelector('#cwiz-body .cwiz-fail').scrollIntoView({ block: 'center' });
    const msgs = [...$('cwiz-body').querySelectorAll('.cwiz-msg')].map((x) => x.innerText);
    return { msgs, items: $('cwiz-body').querySelectorAll('.cwiz-fail li').length, acc: t('cw.f.acc', { mm: '5.2', lim: '3' }).slice(0, 12), same: t('cw.f.fail') };
  });
  ok(!fine.msgs.some((x) => x.startsWith(fine.acc)), `${name} failed fine calibration: no "can be applied" note ${JSON.stringify(fine.msgs)}`);
  await settle(page);
  cover = await onTop(page, '#cwiz-body .cwiz-fail');
  ok(!cover.length, `${name} fine calibration failure box on top of the page ${JSON.stringify(cover)}`);
  await page.screenshot({ path: path.join(out, `${name}_7_fine_fail.png`) });
  ok(fine.items === 2 && fine.msgs.some((x) => x.includes(fine.same) && x.includes('RMS') && x.includes('B → A')), `${name} failed fine calibration: one line per failed cause ${JSON.stringify(fine)}`);
  await page.evaluate(() => { cwOpen(false); CWZ.solve = null; CWZ.step = 'board'; });
  // 7. E-stop latched: the enable switch label is not cut off
  await page.click('#btn-estop');
  ok(await waitFor(page, () => S.state === 'EStop' && /⚠/.test($('enable-txt').textContent)), name + ' E-stop latched');
  await settle(page);
  const lbl = await page.evaluate(() => { const b = $('enable-txt'), sw = $('btn-enable'); const r = b.getBoundingClientRect(), q = sw.getBoundingClientRect();
    return { text: b.textContent, sw: b.scrollWidth, cw: b.clientWidth, inside: r.right <= q.right + 0.5 && r.bottom <= q.bottom + 0.5 }; });
  ok(lbl.text.length > 3 && lbl.sw <= lbl.cw + 1 && lbl.inside, `${name} E-stop reset label fits the switch ${JSON.stringify(lbl)}`);
  // breaks only between words: each word (and each CJK run) lies on one line, at most two lines
  const brk = await page.evaluate(() => {
    const el = $('enable-txt'), tn = el.firstChild, txt = tn.textContent, split = [];
    let i = 0; for (const w of txt.split(' ')) { const r = document.createRange(); r.setStart(tn, i); r.setEnd(tn, i + w.length);
      const tops = new Set([...r.getClientRects()].map((q) => Math.round(q.top))); if (tops.size > 1) split.push(w); i += w.length + 1; }
    const lh = parseFloat(getComputedStyle(el).lineHeight); return { split, lines: Math.round(el.getBoundingClientRect().height / lh) };
  });
  ok(!brk.split.length && brk.lines <= 2, `${name} E-stop reset label breaks between words only ${JSON.stringify(brk)}`);
  await page.screenshot({ path: path.join(out, `${name}_8_estop_label.png`), clip: { x: 0, y: 0, width: view.width, height: Math.min(view.height, 520) } });
  // reset (the session is unlocked here) so the next viewport starts without a latched E-stop
  await page.evaluate(() => clearFault());
  ok(await waitFor(page, () => S.state !== 'EStop'), name + ' E-stop reset');
  console.log(`${name}: done`);
  await ctx.close();
}
await lockServer();
await browser.close();
console.log(`${checks - bad}/${checks} checks passed`);
process.exit(bad ? 1 : 0);
