#!/usr/bin/env node
// Headless check of the cockpit view switch (Video | 3D) and the side-panel pose card (no armd needed):
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/cockpit_view_shots.mjs
// Serves src/api/index.html, the embedded three.js bundle, /assets/model/* (MODEL_DIR, default the
// recamera-b601dm rootfs assets) and a mocked 7-DOF schema; answers the page's WebSocket with observations at
// 50 Hz. For desktop 1440x900, phone landscape 844x390 and phone portrait 390x844 in simple and expert mode:
// the switch is visible and not covered, both views open from it, the joystick overlays are hidden while 3D
// is open (simple mode) and come back afterwards, the safety card stays visible, the pose card fits the
// viewport, and clicking the pose card opens 3D. Pose card states: moved pose, joint at a motion limit,
// collision joint, commanded differs from measured, gravity compensation, collapsed (persisted).
// Redraw rate: none while still, at most 15/s while moving, none while collapsed or hidden.
// Supply load meter (observation.power) in the safety card: visible in both modes, inside the viewport and
// the card, percent of rated, colour band (ok / warn / over), peak marker, '--' without an estimate,
// hidden without a power budget.
// Writes PNGs (default .cache/cockpit-view-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/cockpit-view-shots');
fs.mkdirSync(out, { recursive: true });
const modelDir = process.env.MODEL_DIR || path.join(root, 'kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets');
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const bundle = fs.readFileSync(path.join(root, 'src/api/vendor/three-urdf.min.js.gz'));
const LIMITS = [[-2.8, 2.8, 2], [-3.14, 0, 2], [-3.14, 0, 2], [-1.87, 1.57, 3], [-1.57, 1.57, 3], [-3.14, 3.14, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const WS = { floor_z_m: 0, box_min_m: [-0.6, -0.6, -1.0], box_max_m: [0.6, 0.6, 0.8], max_reach_m: 0.52, check_points_end_m: [], keepout_boxes: [] };
const HOME = [0, 0, 0, 0, 0, 1.5707963, 0];
const ONLY = process.env.ONLY || '';

const srv = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split('?')[0]);
  const json = (o, code = 200) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/three-urdf.min.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8', 'content-encoding': 'gzip' }); res.end(bundle); return; }
  if (u.startsWith('/assets/model/')) {
    const rel = u.slice('/assets/model/'.length);
    const f = path.join(modelDir, rel);
    if (rel.includes('..') || !fs.existsSync(f) || !fs.statSync(f).isFile()) { res.writeHead(404); res.end(); return; }
    res.writeHead(200); res.end(fs.readFileSync(f)); return;
  }
  if (u === '/api/v1/observation/schema') return json({ dof: 7, limits: LIMITS, workspace: WS, home_positions: HOME, capabilities: { version: 9, poses: true, fault_codes: true, power_budget: true },
    power_budget: { supply_rated_a: 6.5, supply_trip_a: 8.5, governor: { soft: 0.8, hard: 0.95 } } });
  if (u === '/api/v1/workspace/keepout') return json({ boxes: [], source: 'runtime' });
  if (u === '/api/v1/poses') return json({ poses: [] });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } };
const shots = [];
const REST = HOME.slice();
const MOVED = [0.6, -1.0, -1.4, 1.3, 0.3, 0.9, -2.5];
const LAYOUTS = [
  ['desktop_expert', { width: 1440, height: 900 }, 1, 'expert'],
  ['desktop_simple', { width: 1440, height: 900 }, 1, 'simple'],
  ['land_simple', { width: 844, height: 390 }, 3, 'simple'],
  ['land_expert', { width: 844, height: 390 }, 3, 'expert'],
  ['port_simple', { width: 390, height: 844 }, 3, 'simple'],
  ['port_expert', { width: 390, height: 844 }, 3, 'expert'],
];
const POWER = (a, extra = {}) => ({ valid: true, bus_a: a, peak_a: a, rated_a: 6.5, ratio: a / 6.5, regen_w: 0, per_joint_a: [0.1, 0.4, 0.3, 0.1, 0.05, 0.05, 0.05], scale: 1, limiting: false, hold: false, ...extra });
const STICKS = ['#pad-sxy', '#pad-sz', '#pad-sgrip', '#s-grip-readout', '.presets'];

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'] });
const rectOf = (page, sel) => page.evaluate((s) => { const e = document.querySelector(s); if (!e) return null; const r = e.getBoundingClientRect(); return { x: r.x, y: r.y, w: r.width, h: r.height, b: r.bottom, r: r.right }; }, sel);
// element at the centre of sel is sel itself or inside it (nothing covers it); scrolled into view if needed
const onTop = (page, sel) => page.evaluate((s) => {
  const e = document.querySelector(s); if (!e) return false;
  let r = e.getBoundingClientRect();
  // a scrolling page (phone portrait): bring the target on screen first, elementFromPoint only sees the viewport
  if (r.bottom > innerHeight || r.top < 0) { e.scrollIntoView({ block: 'center' }); r = e.getBoundingClientRect(); }
  const h = document.elementFromPoint(r.x + r.width / 2, r.y + r.height / 2);
  return !!h && (h === e || e.contains(h));
}, sel);
const visible = (page, sel) => page.evaluate((s) => { const e = document.querySelector(s); if (!e) return false; const r = e.getBoundingClientRect(); const cs = getComputedStyle(e); return r.width > 0 && r.height > 0 && cs.visibility !== 'hidden' && cs.display !== 'none'; }, sel);

try {
  for (const [tag, viewport, dsf, ui] of LAYOUTS) {
    if (ONLY && !tag.includes(ONLY)) continue;
    const mobile = dsf > 1;
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: dsf, isMobile: mobile, hasTouch: mobile });
    await ctx.addInitScript(() => { try { if (!sessionStorage.getItem('init')) { localStorage.clear(); localStorage.setItem('lang', 'zh'); sessionStorage.setItem('init', '1'); } } catch (_) {} });
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push('pageerror: ' + e.message));
    page.on('console', (m) => { if (m.type() === 'error' && !/^Failed to load resource|^WebSocket connection to .*\/detection\/ws/.test(m.text())) errs.push('console: ' + m.text()); });
    const P = { q: REST.slice(), qc: null, extra: {}, moving: false, t0: Date.now() };
    await page.routeWebSocket(/\/api\/v1\/ws/, (ws) => {
      ws.onMessage((raw) => {
        let m; try { m = JSON.parse(String(raw)); } catch (_) { return; }
        if (m.type === 'acquire_lease') ws.send(JSON.stringify({ type: 'lease_granted', seq: m.seq, generation: 7, epoch_id: 1, client_id: m.client_id }));
        else if (m.type === 'command') ws.send(JSON.stringify({ type: 'command_ack', request_id: m.request_id, epoch_id: 1 }));
      });
      P.feed = setInterval(() => {
        const t = (Date.now() - P.t0) / 1000;
        const q = P.moving ? P.q.map((v, i) => (i === 0 ? v + 0.3 * Math.sin(t * 2) : v)) : P.q;
        const o = { timestamp: t, positions: q, velocities: q.map(() => 0), commanded_positions: P.qc || q, safety_state: 'Armed', enable_state: 'Enabled', is_torqued: true,
          feedback_age_ms: q.map(() => 5), torques: q.map(() => 0.5), temperatures: q.map(() => 35), epoch_id: 1, lease_holder: null,
          driver_capabilities: { velocity_feedback: true, torque_feedback: true, temperature_feedback: true }, motion_limit: null, motor_errors: [], action: null, fault: null, power: POWER(1.3, { peak_a: 2.6 }), ...P.extra };
        try { ws.send(JSON.stringify(o)); } catch (_) {}
      }, 20);
    });
    const shot = async (name) => { await page.waitForTimeout(350); const f = path.join(out, `${tag}_${name}.png`); await page.screenshot({ path: f }); shots.push(f); };
    await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 }).catch((e) => { throw new Error(tag + ' not live: ' + errs.join(' | ')); });
    await page.waitForTimeout(800);

    // ---- video view
    ok(await visible(page, '#view-switch'), `${tag} view switch visible`);
    ok(await onTop(page, '#vs-video') && await onTop(page, '#vs-3d'), `${tag} view switch not covered`);
    ok(await page.evaluate(() => document.querySelector('#vs-video').getAttribute('aria-checked') === 'true'), `${tag} video checked on load`);
    const card = process.env.STAGE !== 'switch';
    if (card) {
    ok(await visible(page, '#pose-card'), `${tag} pose card visible`);
    const pc = await rectOf(page, '#pose-card');
    ok(pc && pc.h <= 150, `${tag} pose card height ${pc && pc.h}`);
    // the card is reachable: in the viewport, or in a scrollable column
    ok(await page.evaluate(() => { const e = document.getElementById('pose-card'); e.scrollIntoView({ block: 'nearest' }); const r = e.getBoundingClientRect(); return r.top >= 0 && r.bottom <= innerHeight + 1 && r.left >= 0 && r.right <= innerWidth + 1; }), `${tag} pose card scrolls into the viewport`);
    }
    // ---- supply load meter
    {
      const until = (fn, arg) => page.waitForFunction(fn, arg, { timeout: 4000 }).then(() => true, () => false);
      ok(await visible(page, '#pwr'), `${tag} supply meter visible`);
      ok(await until(() => document.getElementById('pwr-pct').textContent === '20%'), `${tag} meter shows 20% (1.3 of 6.5 A)`);
      const m = await rectOf(page, '#pwr'), sc = await rectOf(page, '#safety');
      ok(m && sc && m.x >= sc.x - 0.5 && m.r <= sc.r + 0.5 && m.y >= sc.y - 0.5 && m.b <= sc.b + 0.5, `${tag} meter inside the safety card ${JSON.stringify(m)} ${JSON.stringify(sc)}`);
      ok(m && m.h <= 20 && m.r <= viewport.width && m.x >= 0, `${tag} meter height ${m && m.h} within the viewport`);
      ok(await page.evaluate(() => { const e = document.getElementById('pwr-pct'); return e.scrollWidth <= e.clientWidth + 1; }), `${tag} meter text not clipped`);
      ok(await page.evaluate(() => { const pk = document.getElementById('pwr-peak'), bar = pk.parentElement.getBoundingClientRect(), r = pk.getBoundingClientRect();
        return getComputedStyle(pk).display !== 'none' && Math.abs((r.left + r.width / 2 - bar.left) / bar.width - (100 * 2.6 / 6.5) / 120) < 0.02; }), `${tag} peak marker at 40% of rated`);
      ok(await page.evaluate(() => !document.getElementById('pwr').classList.contains('warn') && !document.getElementById('pwr').classList.contains('over')), `${tag} meter band ok`);
      await shot('0_power_ok');
      P.extra = { power: POWER(5.6, { peak_a: 6.0, scale: 0.6, limiting: true }), motion_limit: { reason: 'power', joint: null, since_ms: 1 } };
      ok(await until(() => document.getElementById('pwr').classList.contains('warn') && document.getElementById('pwr-pct').textContent === '86%'), `${tag} meter band warn at 86%`);
      await shot('0_power_warn');
      P.extra = { power: POWER(6.3, { peak_a: 7.9, scale: 0.25, limiting: true, hold: true }), motion_limit: { reason: 'power', joint: 3, since_ms: 1 } };
      ok(await until(() => document.getElementById('pwr').classList.contains('over')), `${tag} meter band over while held`);
      ok(await until(() => { const e = document.getElementById('ml-chip'); return e && !e.hidden && /电源负载过高/.test(e.textContent); }), `${tag} power hold hint shown (zh)`);
      await shot('0_power_hold');
      P.extra = { power: { ...POWER(0), valid: false } };
      ok(await until(() => document.getElementById('pwr-pct').textContent === '--' && document.getElementById('pwr').classList.contains('na')), `${tag} meter shows -- without an estimate`);
      P.extra = { power: null };
      ok(await until(() => document.getElementById('pwr').hidden), `${tag} meter hidden without a power budget`);
      await page.click('#btn-lang');
      P.extra = { power: POWER(6.3, { hold: true, limiting: true }), motion_limit: { reason: 'power', joint: null, since_ms: 1 } };
      ok(await until(() => { const e = document.getElementById('ml-chip'); return e && !e.hidden && /Supply load too high/.test(e.textContent) && document.querySelector('#pwr .pwr-lbl').textContent === 'Supply load'; }), `${tag} power hint and label (en)`);
      await shot('0_power_hold_en');
      await page.click('#btn-lang');
      P.extra = {};
      await page.waitForTimeout(1200);
    }
    if (ui === 'simple') for (const s of STICKS) if (await page.$(s)) ok(await visible(page, s) || !(await page.evaluate((x) => document.querySelector(x).offsetParent, s)), `${tag} ${s} state in video view`);
    await page.evaluate(() => window.scrollTo(0, 0));
    await shot('1_video_rest');
    const sticksBefore = {};
    for (const s of STICKS) sticksBefore[s] = await visible(page, s);

    // ---- 3D view from the switch
    await page.click('#vs-3d');
    await page.waitForFunction(() => V3.renders > 0, null, { timeout: 15000 });
    ok(await page.evaluate(() => !document.getElementById('view3d').hidden && document.querySelector('#vs-3d').getAttribute('aria-checked') === 'true'), `${tag} 3D open, switch shows 3D`);
    ok(await onTop(page, '#vs-video'), `${tag} switch visible above the 3D view`);
    // the arm fills a large part of the view: projected bounding box of the robot
    const fill = await page.evaluate(() => {
      const T = V3.T, box = new T.Box3().setFromObject(V3.solid.obj), v = new T.Vector3(); let x0 = 1, x1 = -1, y0 = 1, y1 = -1;
      for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) { v.set(x, y, z).project(V3.cam); x0 = Math.min(x0, v.x); x1 = Math.max(x1, v.x); y0 = Math.min(y0, v.y); y1 = Math.max(y1, v.y); }
      return { w: (x1 - x0) / 2, h: (y1 - y0) / 2 };
    });
    ok(Math.max(fill.w, fill.h) > 0.45, `${tag} robot box spans ${JSON.stringify(fill)} of the view`);
    // the joysticks and sliders stay usable in 3D: same visibility as with the video, and on top of the 3D canvas
    if (ui === 'simple') for (const s of STICKS) {
      ok((await visible(page, s)) === sticksBefore[s], `${tag} ${s} same visibility in 3D`);
      if (sticksBefore[s]) ok(await onTop(page, s), `${tag} ${s} on top of the 3D view`);
    }
    if (ui === 'simple') {
      ok(await visible(page, '#btn-estop') && await onTop(page, '#btn-estop'), `${tag} E-stop visible in 3D`);
      ok(await visible(page, '#btn-enable'), `${tag} enable visible in 3D`);
    }
    await shot('2_3d_rest');
    P.q = MOVED.slice(); await page.waitForTimeout(300);
    await shot('3_3d_moved');
    // keyboard: arrow keys move between the two radios, Escape is not bound by the switch
    await page.focus('#vs-3d'); await page.keyboard.press('ArrowLeft');
    ok(await page.evaluate(() => document.getElementById('view3d').hidden && document.activeElement.id === 'vs-video'), `${tag} ArrowLeft selects video`);
    if (ui === 'simple') for (const s of STICKS) ok((await visible(page, s)) === sticksBefore[s], `${tag} ${s} restored after 3D`);

    if (!card) { ok(errs.length === 0, `${tag} console errors: ${errs.join(' | ')}`); clearInterval(P.feed); await ctx.close(); continue; }
    // ---- pose card states (video view)
    await page.evaluate(() => document.getElementById('pose-card').scrollIntoView({ block: 'nearest' }));
    await page.waitForTimeout(300);
    const until = (fn, arg) => page.waitForFunction(fn, arg, { timeout: 4000 }).then(() => true, () => false);
    const pcShot = async (name) => { await page.waitForTimeout(300); const f = path.join(out, `${tag}_${name}.png`); await page.locator('#pose-card').screenshot({ path: f }); shots.push(f); };
    await pcShot('4_card_moved');
    await shot('4_video_moved');
    P.extra = { motion_limit: { reason: 'joint_limit', joint: 2, since_ms: 1 } };
    ok(await until(() => PCARD.hot.join() === '2'), `${tag} motion_limit joint 2 (0-based) highlighted`);
    await pcShot('5_card_limit');
    P.extra = { collision: { active: true, joint: 4, residual_nm: 3.1, threshold_nm: 2.5, resume_in_s: 1.2, trips_in_window: 1, enabled: true, sensitivity: 'medium' } };
    ok(await until(() => PCARD.hot.join() === '3'), `${tag} collision joint 4 (1-based) highlighted`);
    await pcShot('6_card_collision');
    P.extra = {}; P.qc = [0.6, -0.6, -1.0, 1.0, 0.3, 0.9, -1.0];
    ok(await until(() => PCARD.ghost), `${tag} commanded pose drawn`);
    await pcShot('7_card_cmd');
    P.qc = null; P.extra = { control_mode: 'gravity_comp', gravity_comp: { phase: 'active', ramp: 1, blocker: null, last_exit: null } };
    await page.waitForTimeout(200);
    await pcShot('8_card_gc');
    P.extra = {};
    ok(await until(() => !PCARD.ghost && PCARD.hot.length === 0), `${tag} ghost / highlight cleared`);

    // ---- redraw rate
    await page.waitForTimeout(800);
    await page.evaluate(() => { window.__keys = []; const d = window.poseDraw; window.poseDraw = (...a) => { window.__keys.push(PCARD.key); return d(...a); }; });
    let n0 = await page.evaluate(() => PCARD.draws); await page.waitForTimeout(1000);
    const still = (await page.evaluate(() => PCARD.draws)) - n0;
    ok(still === 0, `${tag} no pose redraw while still (${still}: ${await page.evaluate(() => window.__keys.slice(0, 3).join(' / '))})`);
    P.moving = true; await page.waitForTimeout(200);
    n0 = await page.evaluate(() => PCARD.draws); await page.waitForTimeout(1000);
    const mv = (await page.evaluate(() => PCARD.draws)) - n0;
    ok(mv >= 5 && mv <= 16, `${tag} pose redraws while moving ${mv}/s`);

    // ---- collapse (persisted), no redraw while collapsed
    await page.click('#pose-toggle');
    n0 = await page.evaluate(() => PCARD.draws); await page.waitForTimeout(600);
    ok((await page.evaluate(() => PCARD.draws)) - n0 === 0, `${tag} no redraw while collapsed`);
    const ch = (await rectOf(page, '#pose-card')).h;
    ok(ch < 60, `${tag} collapsed height ${ch}`);
    await pcShot('9_card_collapsed');
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 });
    ok(await page.evaluate(() => document.getElementById('pose-card').classList.contains('collapsed')), `${tag} collapse state persisted`);
    await page.click('#pose-toggle');
    P.moving = false;
    await page.waitForTimeout(300);

    // ---- click on the card canvas opens 3D
    await page.evaluate(() => document.getElementById('pose-card').scrollIntoView({ block: 'nearest' }));
    await page.click('#pose-canvas');
    await page.waitForTimeout(300);
    ok(await page.evaluate(() => !document.getElementById('view3d').hidden), `${tag} tap on the pose card opens 3D`);
    await page.click('#vs-video');
    ok(await page.evaluate(() => document.getElementById('view3d').hidden), `${tag} video button closes 3D`);
    ok(await page.evaluate(() => document.scrollingElement.scrollWidth <= innerWidth), `${tag} no horizontal scroll`);
    ok(errs.length === 0, `${tag} console errors: ${errs.join(' | ')}`);
    clearInterval(P.feed); await ctx.close();
  }
} finally {
  await browser.close();
  srv.close();
}
console.log(`${shots.length} screenshots in ${out}, ${checks} checks, ${bad ? bad + ' FAILED' : 'all passed'}`);
process.exit(bad ? 1 : 0);
