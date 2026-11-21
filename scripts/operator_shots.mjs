#!/usr/bin/env node
// Operator access in the cockpit, end to end against a mock armd with an operator PIN:
//   ARMD_OPERATOR_PIN=246810 cargo run -- --follower mock --config kits/recamera-b601dm/profile/rebot-b601dm.json --port 18961 &
//   BASE=http://127.0.0.1:18961/ PIN=246810 PLAYWRIGHT=/path/to/playwright/index.mjs OUT=/dir node scripts/operator_shots.mjs
// For phone 360x740, phone 390x844 and desktop 1400x900, zh and en: the header badge has a text label inside the
// viewport; enable, E-stop; the safety card shows the unlock entry; tapping reset is refused by armd
// (OPERATOR_REQUIRED) and opens the centred dialog with the reason, the PIN field focused, the background
// covered; a wrong PIN shows "PIN 错误" / "Wrong PIN" inline; Escape and the backdrop cancel without an E-stop; the
// right PIN (Enter) closes the dialog, the reset runs once more by itself, on the same socket and lease
// (no reconnect); then "Park and turn torque off" is offered and parks to torque off. A short viewport stands in
// for the on-screen keyboard (the dialog stays inside it). Detection trigger and gravity mode: refused, dialog
// with their reason, retried after unlock (detector stubbed behind armd's operator check). Lock goes back to
// program on the same socket. First, the header at 360 / 375 / 390 / 414 / 430 px, expert and simple, zh and en, with
// the operator chip, with a live task button, and with auth off: no horizontal scroll, every header control and the
// E-stop is the element at its centre; the recognition popover (vision stubbed: perception API 2 with a metric depth
// model) with its four selector toggles Off / YOLO / ArUco / Depth (est.) fits the viewport, each control at its centre. PNGs to OUT (default .cache/operator-shots/). Exits non-zero if a check fails.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/operator-shots');
const base = process.env.BASE || 'http://127.0.0.1:18961/';
const PIN = process.env.PIN || '246810';
fs.mkdirSync(out, { recursive: true });
let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } return c; };

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'] });
const VIEWS = [['phone360', { width: 360, height: 740 }], ['phone390', { width: 390, height: 844 }], ['desktop', { width: 1400, height: 900 }]];

async function lockServer() { await fetch(new URL('/api/v1/auth/lock', base), { method: 'POST', headers: { Origin: new URL(base).origin } }).catch(() => {}); }
async function holdOn(page, sel, ms = 900) {
  const b = await page.locator(sel).boundingBox();
  await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2); await page.mouse.down(); await page.waitForTimeout(ms); await page.mouse.up();
}
const st = (page) => page.evaluate(() => ({ state: S.state, torqued: !!S.obs?.is_torqued, role: S.auth.role, gen: S.leaseGeneration, ws: window.__wsN,
  open: !$('op-modal').hidden, why: $('op-why').textContent, err: $('op-status').textContent, focus: document.activeElement && document.activeElement.id }));
async function waitFor(page, fn, arg, ms = 8000) { try { await page.waitForFunction(fn, arg, { timeout: ms }); return true; } catch (_) { return false; } }
async function dialogInside(page) {
  return page.evaluate(() => {
    const r = document.querySelector('.op-dialog').getBoundingClientRect(), vv = window.visualViewport;
    const H = vv ? vv.height : innerHeight;
    return r.left >= 0 && r.right <= innerWidth + 0.5 && r.top >= -0.5 && r.bottom <= H + 0.5 && r.width > 200;
  });
}

// 0. header on phones (360-430 px), expert and simple, zh and en, with auth enforced (operator chip shown), auth
// enforced with a live task button, and auth off (chip hidden in the page): no horizontal scroll, and every header
// control and the E-stop is the element at its own centre
const headerFit = (page) => page.evaluate(() => {
  const ctl = [...document.querySelectorAll('.topbar button')].filter((b) => {
    const r = b.getBoundingClientRect(), cs = getComputedStyle(b);
    return !b.closest('[hidden]') && r.width > 0 && r.height > 0 && cs.display !== 'none' && cs.visibility !== 'hidden';
  });
  const es = document.getElementById('btn-estop'); es.scrollIntoView({ block: 'center' });
  const covered = [...ctl, es].map((b) => {
    const r = b.getBoundingClientRect(), h = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
    return { id: b.id || b.dataset.ui, ok: !!h && (h === b || b.contains(h)) && r.left >= 0 && r.right <= innerWidth + 0.5, hit: h ? (h.id || h.className || h.tagName) : null };
  }).filter((x) => !x.ok);
  window.scrollTo(0, 0);
  return { sw: document.scrollingElement.scrollWidth, iw: innerWidth, ids: ctl.map((b) => b.id || b.dataset.ui), covered };
});
await lockServer();
for (const ui of ['expert', 'simple']) for (const lang of ['zh', 'en']) for (const w of [360, 375, 390, 414, 430]) {
  const name = `header_${ui}_${lang}_${w}`;
  const ctx = await browser.newContext({ viewport: { width: w, height: 812 }, deviceScaleFactor: 1, hasTouch: true });
  await ctx.addInitScript((l) => { try { localStorage.setItem('lang', l); } catch (_) {} }, lang);
  const page = await ctx.newPage();
  page.on('pageerror', (e) => { console.log('  pageerror ' + e.message); ok(false, name + ' pageerror'); });
  await page.route(/\/api\/v1\/(vision|detection)\//, (route) => {
    const u = new URL(route.request().url()).pathname;
    const f = (o, st = 200) => route.fulfill({ status: st, contentType: 'application/json', body: JSON.stringify(o) });
    if (u === '/api/v1/detection/inference') return f({ enabled: false, fps: 1, active_fps: 5, idle_fps: 1 });
    if (u === '/api/v1/vision/mode') return f({ api_version: 2, mode: 'off', modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: 1, trigger: true, per_request_pipelines: true, subscriptions: true,
      available: { yolo: true, aruco: true, depth: true }, depth_model: { name: 'zipdepth', path: '/userdata/local/models/zipdepth.cvimodel', metric: true } });
    return f({ error: 'not found' }, 404);
  });
  await page.goto(base + '?ui=' + ui, { waitUntil: 'domcontentloaded' });
  ok(await waitFor(page, () => S.obs && S.dof > 0 && S.auth.enforced && !$('btn-op').hidden, null, 15000), name + ' auth enforced, operator chip shown');
  await page.waitForTimeout(300);
  for (const variant of ['auth', 'auth_task', 'noauth']) {
    if (variant === 'auth_task') await page.evaluate(() => { const b = $('btn-task'); b.hidden = false; b.classList.add('is-live'); });
    if (variant === 'noauth') await page.evaluate(() => { const b = $('btn-task'); b.classList.remove('is-live'); S.auth.enforced = false; renderOpBadge(); });
    await page.waitForTimeout(150);
    const h = await headerFit(page);
    const need = ['btn-lang', ...(variant === 'noauth' ? [] : ['btn-op'])];
    ok(h.sw <= h.iw, `${name} ${variant} no horizontal scroll (${h.sw} > ${h.iw})`);
    ok(need.every((id) => h.ids.includes(id)) && !h.covered.length, `${name} ${variant} header controls and E-stop reachable ${JSON.stringify(h)}`);
    if (variant === 'auth' && (w === 375 || w === 360)) await page.screenshot({ path: path.join(out, `${name}.png`), clip: { x: 0, y: 0, width: w, height: 160 } });
  }
  // recognition popover: every control, the four selector toggles included, inside the viewport and at its centre
  await waitFor(page, () => VM.state === 'ok' && !$('vmode').hidden, null, 6000);
  await page.evaluate(() => { const b = $('btn-vis'); if (!document.body.classList.contains('vis-open') && b && !b.hidden && getComputedStyle(b).display !== 'none') b.click(); });
  await page.waitForTimeout(200);
  const pop = await page.evaluate(() => {
    const els = [...document.querySelectorAll('#det-ctl button')].filter((e) => { const r = e.getBoundingClientRect(); return !e.closest('[hidden]') && r.width > 0 && r.height > 0; });
    const bad = els.map((e) => { const r = e.getBoundingClientRect(), h = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
      return { id: e.id || 'vm-' + e.dataset.vm, ok: !!h && (h === e || e.contains(h)) && r.left >= 0 && r.right <= innerWidth + 0.5 && r.top >= 0 && r.bottom <= innerHeight + 0.5 }; }).filter((x) => !x.ok);
    return { ids: els.map((e) => e.id || 'vm-' + e.dataset.vm), bad, sw: document.scrollingElement.scrollWidth, iw: innerWidth };
  });
  ok(['vm-off', 'vm-yolo', 'vm-aruco', 'vm-depth'].every((id) => pop.ids.includes(id)) && !pop.bad.length && pop.sw <= pop.iw, `${name} recognition popover fits, every control reachable ${JSON.stringify(pop)}`);
  if (w === 360 || w === 430) await page.screenshot({ path: path.join(out, `${name}_popover.png`), clip: { x: 0, y: 0, width: w, height: 400 } });
  console.log(`${name}: header ok`);
  await ctx.close();
}

for (const lang of ['zh', 'en']) {
  for (const [vname, view] of VIEWS) {
    const name = `${vname}_${lang}`;
    await lockServer();
    const ctx = await browser.newContext({ viewport: view, deviceScaleFactor: 2, hasTouch: view.width < 1000, locale: lang === 'zh' ? 'zh-CN' : 'en-US' });
    await ctx.addInitScript((l) => {
      try { localStorage.setItem('lang', l); } catch (_) {}
      // count telemetry sockets: an unlock / lock must not reconnect (that drops the lease)
      const Orig = window.WebSocket; window.__wsN = 0;
      window.WebSocket = class extends Orig { constructor(u, p) { super(u, p); if (String(u).includes('/api/v1/ws')) window.__wsN++; } };
    }, lang);
    const page = await ctx.newPage();
    page.on('pageerror', (e) => { console.log('  pageerror ' + e.message); ok(false, name + ' pageerror'); });
    // detector behind armd: armd answers first (403 without an operator session), the stub after that
    await page.route(/\/api\/v1\/(vision|detection)\//, async (route) => {
      const req = route.request(); const u = new URL(req.url()).pathname;
      const f = (o, s = 200) => route.fulfill({ status: s, contentType: 'application/json', body: JSON.stringify(o) });
      if (u === '/api/v1/detection/trigger') {
        const r = await route.fetch().catch(() => null);
        if (r && r.status() === 403) return route.fulfill({ response: r });
        page.__trig = (page.__trig || 0) + 1;
        return f({ api_version: 2, success: true, frame_id: 2, width: 640, height: 360, detections: [], markers: [], pipelines: {} });
      }
      if (u === '/api/v1/detection/inference') return f({ enabled: false, fps: 1, active_fps: 5, idle_fps: 1, mode: 'yolo' });
      if (u === '/api/v1/vision/mode') return f({ api_version: 1, mode: 'yolo', modes: ['off', 'yolo', 'aruco', 'both'], enabled: false, fps: 1, trigger: true, available: { yolo: true, aruco: true } });
      if (u === '/api/v1/detection/latest') return f({ timestamp: 1, frame_id: 1, boxes: [], resolution: [1280, 720] });
      return f({ error: 'not found' }, 404);
    });
    await page.goto(base + '?ui=simple', { waitUntil: 'domcontentloaded' });
    ok(await waitFor(page, () => S.obs && S.dof > 0 && S.auth.enforced, null, 15000), name + ' telemetry + auth enforced');

    // 1b. badge: text label, inside the viewport, not covered
    const badge = await page.evaluate(() => { const b = $('btn-op'), r = b.getBoundingClientRect(); const hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
      return { text: b.textContent.trim(), vis: !b.hidden && r.width > 0, inside: r.left >= 0 && r.right <= innerWidth && r.top >= 0, top: !!hit && b.contains(hit), w: Math.round(r.width) }; });
    ok(badge.vis && badge.inside && badge.top && /操作员|Operator/.test(badge.text), `${name} badge ${JSON.stringify(badge)}`);
    const hdr = await page.evaluate(() => [...document.querySelectorAll('.topbar button')].filter((b) => b.offsetParent !== null)
      .filter((b) => { const r = b.getBoundingClientRect(); return r.left < 0 || r.right > innerWidth + 0.5; }).map((b) => b.id || b.className));
    ok(!hdr.length, `${name} header buttons inside the viewport (outside: ${hdr.join(', ')})`);
    await page.screenshot({ path: path.join(out, `${name}_1_badge.png`) });

    // enable, E-stop: the safety card offers the unlock entry
    await holdOn(page, '#btn-enable');
    ok(await waitFor(page, () => S.state === 'Armed'), name + ' armed');
    await page.locator('#btn-estop').dispatchEvent('pointerdown');
    ok(await waitFor(page, () => S.state === 'EStop'), name + ' E-stop latched');
    ok(await waitFor(page, () => !$('op-inline').hidden, null, 3000), name + ' unlock entry in the safety card');
    await page.locator('#op-inline').scrollIntoViewIfNeeded();
    await page.screenshot({ path: path.join(out, `${name}_2_estop_unlock_entry.png`) });

    // 1a. reset refused by armd -> dialog with the reason
    const gen0 = await page.evaluate(() => S.leaseGeneration);
    await page.locator('#btn-enable').dispatchEvent('pointerdown');
    ok(await waitFor(page, () => !$('op-modal').hidden), name + ' dialog opens on OPERATOR_REQUIRED');
    let s = await st(page);
    ok(/复位急停需要操作员权限|Resetting the E-stop needs operator access/.test(s.why), `${name} reason "${s.why}"`);
    ok(s.focus === 'op-pin', `${name} PIN field focused (${s.focus})`);
    const pinAttrs = await page.evaluate(() => ({ im: $('op-pin').inputMode, type: $('op-pin').type }));
    ok(pinAttrs.im === 'numeric' && pinAttrs.type === 'password', name + ' numeric keypad PIN field');
    ok(await dialogInside(page), name + ' dialog inside the viewport');
    const covered = await page.evaluate(() => { const r = $('btn-enable').getBoundingClientRect(); const hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2); return document.body.classList.contains('op-open') && !!hit && !!hit.closest('#op-modal'); });
    ok(covered, name + ' background covered and locked');
    const centred = await page.evaluate(() => { const r = document.querySelector('.op-dialog').getBoundingClientRect(); return Math.abs((r.left + r.right) / 2 - innerWidth / 2) < 2; });
    ok(centred, name + ' dialog centred');
    await page.waitForTimeout(300); await page.screenshot({ path: path.join(out, `${name}_3_modal_empty.png`) });

    await page.fill('#op-pin', '000000'); await page.press('#op-pin', 'Enter');
    ok(await waitFor(page, () => $('op-status').textContent.length > 0), name + ' wrong PIN answered');
    s = await st(page);
    ok(/PIN 错误|Wrong PIN/.test(s.err) && s.open, `${name} inline error "${s.err}"`);
    await page.screenshot({ path: path.join(out, `${name}_4_modal_error.png`) });

    // on-screen keyboard: a short viewport, the dialog stays inside it
    await page.setViewportSize({ width: view.width, height: Math.min(view.height, 360) });
    await page.waitForTimeout(250);
    ok(await dialogInside(page), name + ' dialog inside a keyboard-sized viewport');
    if (view.width < 1000) await page.screenshot({ path: path.join(out, `${name}_5_modal_keyboard.png`) });
    await page.setViewportSize(view);
    await page.waitForTimeout(150);

    await page.fill('#op-pin', PIN); await page.press('#op-pin', 'Enter');
    ok(await waitFor(page, () => $('op-modal').hidden && S.auth.role === 'operator'), name + ' unlock closes the dialog');
    ok(await waitFor(page, () => S.state === 'Idle'), name + ' reset retried after unlock -> Idle');
    s = await st(page);
    ok(s.gen === gen0 && s.ws === 1, `${name} same socket and lease (generation ${gen0} -> ${s.gen}, sockets ${s.ws})`);
    ok(await waitFor(page, () => !$('btn-park-off').hidden, null, 3000), name + ' "Park and turn torque off" offered while holding');
    await page.locator('#btn-park-off').scrollIntoViewIfNeeded();
    await page.screenshot({ path: path.join(out, `${name}_6_reset_holding.png`) });

    // badge -> the unlocked dialog
    await page.click('#btn-op');
    ok(await waitFor(page, () => !$('op-modal').hidden && !$('op-lock').hidden), name + ' unlocked dialog');
    await page.waitForTimeout(300); await page.screenshot({ path: path.join(out, `${name}_7_modal_unlocked.png`) });
    await page.keyboard.press('Escape');
    ok(await waitFor(page, () => $('op-modal').hidden), name + ' Escape closes');
    ok((await st(page)).state === 'Idle', name + ' Escape in the dialog is no E-stop');

    // item 3: park and torque off from the hold after the clear
    await holdOn(page, '#btn-park-off');
    ok(await waitFor(page, () => S.obs && !S.obs.is_torqued && S.state === 'Idle', null, 15000), name + ' park ends torque off');

    // lock on the same socket, then: gravity mode and detection trigger are refused, prompt, retry after unlock
    await page.click('#btn-op'); await page.click('#op-lock');
    ok(await waitFor(page, () => S.auth.role === 'program' && $('op-modal').hidden), name + ' locked');
    ok((await st(page)).ws === 1, name + ' lock without reconnect');
    const gc = await page.evaluate(() => !!(S.obs.gravity_comp && S.obs.gravity_comp.supported) && !$('btn-gc').hidden);
    if (gc) {
      await holdOn(page, '#btn-enable');
      await waitFor(page, () => S.state === 'Armed');
      await page.locator('#btn-gc').scrollIntoViewIfNeeded();
      await holdOn(page, '#btn-gc');
      ok(await waitFor(page, () => !$('op-modal').hidden), name + ' gravity mode prompts');
      s = await st(page);
      ok(/重力补偿需要操作员权限|Gravity mode needs operator access/.test(s.why), `${name} gc reason "${s.why}"`);
      if (vname === 'phone360') await page.waitForTimeout(300); await page.screenshot({ path: path.join(out, `${name}_8_modal_gravity.png`) });
      // backdrop cancels
      await page.mouse.click(4, Math.round(view.height - 6));
      ok(await waitFor(page, () => $('op-modal').hidden), name + ' backdrop cancels');
      ok((await st(page)).state !== 'EStop', name + ' cancel is no E-stop');
      // the switch ignores presses for 300 ms after a refused request (inflight guard in the btn-gc handler; longer
      // under load): press again until the prompt opens
      let again = false;
      for (let k = 0; k < 3 && !again; k++) { await page.waitForTimeout(500); await holdOn(page, '#btn-gc'); again = await waitFor(page, () => !$('op-modal').hidden, null, 3000); }
      ok(again, name + ' gravity mode prompts again after a cancel');
      await page.fill('#op-pin', PIN); await page.press('#op-pin', 'Enter');
      ok(await waitFor(page, () => S.obs && S.obs.control_mode === 'gravity_comp', null, 8000), name + ' gravity mode on after unlock (retried)');
      await page.evaluate(() => api('/api/v1/mode/gravity_comp', { enable: false }, { quiet: true }).catch(() => {}));
      await waitFor(page, () => S.obs && S.obs.control_mode !== 'gravity_comp');
      await page.evaluate(() => fetch('/api/v1/auth/lock', { method: 'POST' }).then(() => { S.auth.role = 'program'; return opSyncSocket(null); }));
    } else console.log(`  ${name}: mock reports no gravity compensation; gravity prompt not exercised`);
    const det = await page.evaluate(() => DET.avail === true);
    if (det) {
      page.__trig = 0;
      await page.evaluate(() => { triggerDet(); });
      ok(await waitFor(page, () => !$('op-modal').hidden), name + ' detection trigger prompts');
      s = await st(page);
      ok(/识别需要操作员权限|Detection needs operator access/.test(s.why), `${name} det reason "${s.why}"`);
      await page.fill('#op-pin', PIN); await page.press('#op-pin', 'Enter');
      ok(await waitFor(page, () => $('op-modal').hidden && !DET.snap), name + ' trigger finished after unlock');
      ok(page.__trig === 1, `${name} trigger retried once (${page.__trig})`);
    } else console.log(`  ${name}: detector not available in the page; trigger prompt not exercised`);
    // leave the arm parked
    await page.evaluate(() => { if (S.obs && S.obs.is_torqued) controlledUnload(); });
    await waitFor(page, () => S.obs && !S.obs.is_torqued, null, 15000);
    console.log(`${name}: done`);
    await ctx.close();
  }
}
await browser.close();
await lockServer();
console.log(`${checks - bad}/${checks} checks passed; PNGs in ${out}`);
process.exit(bad ? 1 : 0);
