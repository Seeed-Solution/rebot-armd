#!/usr/bin/env node
// Layout check of the simple-mode cockpit on phones and desktop, against a running armd (mock follower):
//   cargo run -- --follower mock --port 18877 &
//   BASE=http://127.0.0.1:18877/ PLAYWRIGHT=/path/to/playwright/index.mjs OUT=/dir node scripts/simple_layout_shots.mjs
// The camera and rebot-vision are faked in the page (video playing at 14 fps / 195 ms, detector API 2 with
// YOLO + ArUco, 7-DOF grasp settings), so every simple-mode chip is on screen. The arm is enabled (hold the
// switch). For 932x430, 844x390, 667x375 landscape, 390x844 portrait and 1440x900: no two visible controls
// overlap, every visible control (and its text) is inside the viewport, the E-stop is visible and on top.
// The recognition panel is opened once per viewport, checked the same way, used (switch on / off), closed.
// Portrait scrolls: checked horizontally, plus the E-stop on screen after scrolling to the bottom.
// Also writes expert-mode 1440x900 (EXPERT=1 only writes that one).
// Writes PNGs to OUT (default .cache/simple-layout-shots/).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/simple-layout-shots');
const base = process.env.BASE || 'http://127.0.0.1:8765/';
const tag = process.env.TAG ? process.env.TAG + '_' : '';
fs.mkdirSync(out, { recursive: true });

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } };

const V = { mode: 'yolo', enabled: false };
const vbody = () => ({ api_version: 2, mode: V.mode, modes: ['off', 'yolo', 'aruco', 'both'], enabled: V.enabled, fps: V.enabled ? 5 : 1,
  active_fps: 5, idle_fps: 1, trigger: true, available: { yolo: true, aruco: true }, per_request_pipelines: true, subscriptions: true });

const VIEWS = [
  ['land932', { width: 932, height: 430 }, 3],
  ['land844', { width: 844, height: 390 }, 3],
  ['land667', { width: 667, height: 375 }, 2],
  ['port390', { width: 390, height: 844 }, 3],
  ['desk1440', { width: 1440, height: 900 }, 1],
];

// Visible interactive controls and cards of the simple cockpit, as leaf "blocks" for the overlap test.
const collect = () => {
  const vis = (e) => {
    if (!e) return false;
    const r = e.getBoundingClientRect(); if (r.width < 1 || r.height < 1) return false;
    for (let n = e; n && n !== document.body; n = n.parentElement) {
      const cs = getComputedStyle(n);
      if (cs.display === 'none' || cs.visibility === 'hidden' || Number(cs.opacity) === 0) return false;
    }
    return true;
  };
  const SEL = ['#safety', '.topbar', '#view-switch', '#pose-card', '#pad-sxy', '#pad-srot', '#pad-sz', '#pad-sgrip', '#s-grip-readout',
    '.preset', '#btn-pose', '.hud > *', '#s-vis', '#grasp-bar', '#s-objhint', '#gc-banner', '#s-hint', '.toast'];
  // intended: the view switch sits in the pose card's header row
  const ALLOW = [['#view-switch', '#pose-card']];
  const scrolls = document.scrollingElement.scrollHeight > innerHeight + 1;
  const els = [...new Set(SEL.flatMap((s) => [...document.querySelectorAll(s)]))].filter(vis);
  // keep leaves: drop a block that contains another listed block
  const leaves = els.filter((e) => !els.some((o) => o !== e && e.contains(o)));
  const name = (e) => e.id ? '#' + e.id : (e.className && typeof e.className === 'string' ? '.' + e.className.trim().split(/\s+/).join('.') : e.tagName) + (e.dataset.preset ? '[' + e.dataset.preset + ']' : '');
  const rects = leaves.map((e) => { const r = e.getBoundingClientRect(); return { n: name(e), e, x: r.left, y: r.top, r: r.right, b: r.bottom }; });
  const W = innerWidth, H = innerHeight, issues = [];
  const tol = 0.5;
  for (const a of rects) {
    // a scrolling page (portrait) is only checked horizontally
    const outside = (x, y, r, b) => x < -tol || r > W + tol || (!scrolls && (y < -tol || b > H + tol));
    if (outside(a.x, a.y, a.r, a.b)) issues.push(`outside viewport: ${a.n} [${a.x|0},${a.y|0},${a.r|0},${a.b|0}] in ${W}x${H}`);
    // descendants (labels, text) inside the viewport, text not clipped
    for (const d of a.e.querySelectorAll('*')) {
      if (!vis(d)) continue;
      const r = d.getBoundingClientRect();
      if (outside(r.left, r.top, r.right, r.bottom)) { issues.push(`child outside viewport: ${a.n} > ${name(d)}`); break; }
    }
    for (const d of [a.e, ...a.e.querySelectorAll('button, span, b')]) {
      if (!vis(d) || d.children.length) continue;
      if (d.scrollWidth > d.clientWidth + 1 && getComputedStyle(d).overflow !== 'visible') issues.push(`text clipped: ${a.n} > ${name(d)} "${d.textContent.trim()}"`);
    }
  }
  for (let i = 0; i < rects.length; i++) for (let j = i + 1; j < rects.length; j++) {
    const a = rects[i], b = rects[j];
    if (a.e.contains(b.e) || b.e.contains(a.e)) continue;
    if (ALLOW.some(([p, q]) => (a.n === p && b.n === q) || (a.n === q && b.n === p))) continue;
    // stacked (portrait) layout: the recognition panel is a bottom sheet over the controls below the safety card
    if (document.body.classList.contains('oc-slot') && (a.n === '#s-vis' || b.n === '#s-vis')) continue;
    const ix = Math.min(a.r, b.r) - Math.max(a.x, b.x), iy = Math.min(a.b, b.b) - Math.max(a.y, b.y);
    if (ix > 1 && iy > 1) issues.push(`overlap ${a.n} x ${b.n} (${ix|0}x${iy|0})`);
  }
  const es = document.getElementById('btn-estop'); const er = es.getBoundingClientRect();
  const hit = document.elementFromPoint(er.left + er.width / 2, er.top + er.height / 2);
  return { scrolls, n: rects.length, names: rects.map((r) => r.n), issues, estop: vis(es) && er.left >= 0 && er.right <= W && er.top >= 0 && er.bottom <= H && !!hit && es.contains(hit) };
};

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'] });
async function open(view, scale, ui) {
  const ctx = await browser.newContext({ viewport: view, deviceScaleFactor: scale, hasTouch: view.width < 1000, isMobile: false, locale: 'zh-CN' });
  const page = await ctx.newPage();
  page.on('pageerror', (e) => console.log('  pageerror ' + e.message));
  await page.route(/\/api\/v1\/(vision|detection)\//, async (route) => {
    const req = route.request(); const u = new URL(req.url()).pathname; const j = req.postData() ? JSON.parse(req.postData()) : null;
    const f = (o, s = 200) => route.fulfill({ status: s, contentType: 'application/json', body: JSON.stringify(o) });
    if (u === '/api/v1/detection/inference') { if (j && 'enabled' in j) V.enabled = !!j.enabled; return f({ enabled: V.enabled, fps: V.enabled ? 5 : 1, active_fps: 5, idle_fps: 1, mode: V.mode }); }
    if (u === '/api/v1/vision/mode') { if (j && j.mode) V.mode = j.mode; return f(vbody()); }
    if (u === '/api/v1/vision/subscribe') return f({ api_version: 2, id: 's1', client: j && j.client, pipelines: (j && j.pipelines) || ['yolo'], fps: 5, ttl_ms: 10000 });
    if (u.startsWith('/api/v1/vision/subscribe/')) return f({ api_version: 2, id: 's1' });
    if (u === '/api/v1/detection/trigger') return f({ api_version: 2, success: true, frame_id: 1, width: 640, height: 360, detections: [], markers: [], pipelines: {} });
    if (u === '/api/v1/detection/latest') return f({ timestamp: 1, frame_id: 1, boxes: [], resolution: [1280, 720] });
    return f({ error: 'not found' }, 404);
  });
  await page.goto(base, { waitUntil: 'domcontentloaded' });
  await page.evaluate(() => {
    const poster = document.createElement('canvas'); poster.width = 640; poster.height = 360;
    const g = poster.getContext('2d'); const gr = g.createLinearGradient(0, 0, 0, 360);
    gr.addColorStop(0, '#b9a38a'); gr.addColorStop(1, '#6f5a43'); g.fillStyle = gr; g.fillRect(0, 0, 640, 360);
    g.fillStyle = '#3d6fb4'; g.fillRect(420, 120, 70, 150);
    document.getElementById('video-stream').poster = poster.toDataURL();
    setInterval(() => { V.playing = true; V.fps = 14; V.lat = 195; document.getElementById('vhint')?.classList.add('hide'); }, 50);
  });
  await page.evaluate((u) => setUi(u), ui);
  await page.waitForFunction(() => S.obs && S.dof > 0, null, { timeout: 15000 });
  // enable: hold the switch
  const en = await page.$('#btn-enable');
  if (await en.isVisible()) {
    const b = await en.boundingBox();
    await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2); await page.mouse.down(); await page.waitForTimeout(900); await page.mouse.up();
    // a page without control asks first (take-control dialog); then hold again
    await page.waitForTimeout(300);
    if (await page.isVisible('#take-confirm')) {
      await page.click('#take-confirm'); await page.waitForTimeout(500);
      await page.mouse.move(b.x + b.width / 2, b.y + b.height / 2); await page.mouse.down(); await page.waitForTimeout(900); await page.mouse.up();
    }
  }
  await page.waitForTimeout(1500);
  return { ctx, page };
}

try {
  for (const [name, view, scale] of process.env.EXPERT ? [] : VIEWS) {
    const { ctx, page } = await open(view, scale, 'simple');
    const st = await page.evaluate(() => S.obs?.enable_state);
    let r = await page.evaluate(collect);
    const file = path.join(out, `${tag}simple_${name}.png`);
    await page.screenshot({ path: file });
    console.log(`${name} simple: ${r.n} controls${r.scrolls ? ' (scrolling page)' : ''}, enable_state=${st}, estop visible=${r.estop}, issues=${r.issues.length}`);
    for (const i of r.issues) console.log('    ' + i);
    ok(r.estop, `${name}: E-stop visible and on top`);
    ok(r.issues.length === 0, `${name}: no overlap / overflow`);
    // recognition popover (if this build has one)
    const chip = await page.$('#btn-vis');
    if (chip && await chip.isVisible()) {
      // toasts are short-lived notifications (2.6-4.2 s, no pointer events) and may cover the open popover: let them clear
      await page.waitForFunction(() => !document.querySelector('.toast'), null, { timeout: 8000 }).catch(() => {});
      await chip.click(); await page.waitForTimeout(300);
      r = await page.evaluate(collect);
      await page.screenshot({ path: path.join(out, `${tag}simple_${name}_vispop.png`) });
      console.log(`${name} simple + recognition popover: ${r.n} controls, issues=${r.issues.length}`);
      for (const i of r.issues) console.log('    ' + i);
      ok(r.issues.length === 0, `${name}: popover open, no overlap / overflow`);
      ok(await page.isVisible('#sv-on'), `${name}: recognition switch in the panel`);
      ok(await page.isVisible('#sv-show'), `${name}: display options in the panel`);
      await page.click('#sv-on'); await page.waitForTimeout(600);
      const chipTxt = await page.textContent('#btn-vis');
      ok(/开|物体|On|objects/.test(chipTxt), `${name}: the panel switch turns recognition on (chip "${chipTxt}")`);
      await page.click('#sv-on'); await page.waitForTimeout(600);
      await page.click('#btn-vis'); await page.waitForTimeout(200);
      ok(!(await page.evaluate(() => document.body.classList.contains('vis-open'))) && !(await page.isVisible('#s-vis')), `${name}: the chip closes the panel`);
      await page.click('#btn-vis'); await page.waitForTimeout(200);
      // tap the video just below the panel (on the phone sheet: just above it)
      const pr = await page.evaluate(() => { const r = document.getElementById('s-vis').getBoundingClientRect(); return { x: r.left + r.width / 2, top: r.top, bottom: r.bottom }; });
      await page.mouse.click(pr.x, pr.top > 120 ? pr.top - 12 : pr.bottom + 10); await page.waitForTimeout(200);
      ok(!(await page.isVisible('#s-vis')), `${name}: an outside tap closes the panel`);
      ok(await page.evaluate(() => S.state !== 'EStop'), `${name}: popover use did not trigger the E-stop`);
    }
    if (r.scrolls) {
      await page.evaluate(() => window.scrollTo(0, document.scrollingElement.scrollHeight)); await page.waitForTimeout(300);
      const e2 = await page.evaluate(collect);
      await page.screenshot({ path: path.join(out, `${tag}simple_${name}_scrolled.png`) });
      console.log(`${name} simple scrolled to the bottom: estop visible=${e2.estop}`);
      ok(e2.estop, `${name}: E-stop visible after scrolling to the bottom`);
    }
    await ctx.close();
  }
  {
    const { ctx, page } = await open({ width: 1440, height: 900 }, 1, 'expert');
    await page.screenshot({ path: path.join(out, `${tag}expert_desk1440.png`) });
    await ctx.close();
  }
} finally {
  await browser.close();
}
console.log(`${checks - bad}/${checks} checks passed`);
process.exit(bad ? 1 : 0);
