#!/usr/bin/env node
// Headless check of the cockpit ArUco overlay with a full calibration board in view (no armd needed):
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/aruco_overlay_shots.mjs
// Feeds a rebot-vision "aruco" /results envelope (display pixels 1280x720) with the 12 board markers (ids 0..11,
// 4 x 3, slight perspective) plus 3 look-alike ids (17, 27, 34; a real arm reported such ids next to the board).
// For simple / expert x zh / en x desktop 1280x800 / phone 375x812 it checks, from the overlay's own layout record
// (ARU.layout, CSS px of the overlay canvas):
//   - no two labels (ID tags, the pose HUD, the summary) overlap, none lies under the chips / view switch / banner
//     over the picture, and every label lies inside the video picture
//   - exactly one full ID / Z / R P Y HUD, on a board marker (the one nearest the image centre by default)
//   - the look-alike ids are dimmed, tagged "<id>?" and never get the HUD; the summary counts 12 board markers
//     and lists 17 27 34 (full panel, or the short form where it does not fit)
//   - tapping a board marker moves the HUD there and keeps it after the pointer leaves; tapping it again
//     returns to the automatic choice; tapping a look-alike does not move it; on desktop hovering a marker shows its HUD
//   - the cockpit E-stop is the element at its centre and a tap runs eStop()
//   - the page does not scroll sideways
// Writes PNGs (default .cache/aruco-overlay-shots/). Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { readCockpitHtmlBuffer } from './source_paths.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const out = process.env.OUT || path.join(root, '.cache/aruco-overlay-shots');
fs.mkdirSync(out, { recursive: true });
const html = readCockpitHtmlBuffer();
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });

// markers in display pixels (1280x720), OpenCV corner order TL, TR, BR, BL
const FALSE_IDS = [17, 27, 34];
const quad = (cx, cy, s, skew = 0.06) => [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => {
  const y = cy + b * s / 2; return [Math.round((cx + a * s / 2 + skew * (y - 400)) * 10) / 10, Math.round(y * 10) / 10];
});
const MARKERS = [];
for (let r = 0; r < 3; r++) for (let c = 0; c < 4; c++) {
  const id = r * 4 + c, cx = 600 + (c - 1.5) * 78, cy = 400 + (r - 1) * 78;
  MARKERS.push({ id, margin: 60, center: [cx, cy], corners: quad(cx, cy, 64), pose: { t: [(cx - 640) / 2000, (cy - 360) / 2000, 0.31], r_deg: [178 + r, 2 - c, 4 + id * 0.5], error: 0.0004 } });
}
for (const [id, cx, cy, s] of [[17, 905, 300, 24], [27, 640, 552, 22], [34, 300, 250, 26]]) MARKERS.push({ id, margin: 9, center: [cx, cy], corners: quad(cx, cy, s, 0), pose: { t: [0.1, -0.05, 0.6], r_deg: [170, 10, 30], error: 0.004 } });
const ENV = { timestamp: 1, frame_id: 42, inference_time_ms: 40, resolution: [1280, 720], boxes: [], labels: [], mode: 'aruco', markers: MARKERS };

// fake video frame with the markers painted on it, the envelope re-fed every 30 ms, eStop stubbed
const feed = ([env, dict]) => {
  const poster = document.createElement('canvas'); poster.width = 640; poster.height = 360;
  const g = poster.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 360); grad.addColorStop(0, '#b9a38a'); grad.addColorStop(1, '#8f7a63');
  g.fillStyle = grad; g.fillRect(0, 0, 640, 360);
  g.fillStyle = '#f4f1ea'; g.fillRect(420 / 2 - 30, 400 / 2 - 140, 400, 280);
  for (const m of env.markers) {
    const p = m.corners.map(([x, y]) => [x / 2, y / 2]);
    g.fillStyle = '#111'; g.beginPath(); g.moveTo(...p[0]); p.slice(1).forEach((q) => g.lineTo(...q)); g.closePath(); g.fill();
    const code = dict[m.id] ?? (m.id * 2654435761) & 0xffff;
    const at = (u, v) => [p[0][0] + (p[1][0] - p[0][0]) * u + (p[3][0] - p[0][0]) * v, p[0][1] + (p[1][1] - p[0][1]) * u + (p[3][1] - p[0][1]) * v];
    g.fillStyle = '#fff';
    for (let k = 0; k < 16; k++) if (code & (1 << (15 - k))) {
      const i = k % 4, j = Math.floor(k / 4), q = [at((i + 1) / 6, (j + 1) / 6), at((i + 2) / 6, (j + 1) / 6), at((i + 2) / 6, (j + 2) / 6), at((i + 1) / 6, (j + 2) / 6)];
      g.beginPath(); g.moveTo(...q[0]); q.slice(1).forEach((x) => g.lineTo(...x)); g.closePath(); g.fill();
    }
  }
  const v = document.getElementById('video-stream');
  v.poster = poster.toDataURL();
  Object.defineProperty(v, 'videoWidth', { get: () => 640 });
  Object.defineProperty(v, 'videoHeight', { get: () => 360 });
  window.__estop = 0; window.eStop = () => { window.__estop++; };
  setInterval(() => {
    const vh = document.getElementById('vhint'); if (vh) vh.classList.add('hide');
    S.detections = parseDetections(env, videoSrcRes()); S.detSource = 'mock'; S.detLastReceivedAt = performance.now();
  }, 30);
};
// the overlay's layout record of a drawn frame (waits for one: a busy host can let the detections age out for a moment)
const layout = async (page) => { await page.waitForFunction(() => ARU.layout && ARU.layout.n > 0, null, { timeout: 5000 }); return layoutNow(page); };
const layoutNow = (page) => page.evaluate(() => {
  const b = document.getElementById('video-box').getBoundingClientRect(), r = getVideoContentRect(b.width, b.height);
  // chips, view switch and banner drawn over the picture (CSS px of the overlay canvas)
  const ui = [...document.querySelectorAll('#video-box .hud > *, #video-box .hud .det-ctl > *, #video-box .view-switch, #video-box .banner')]
    .filter((e) => !e.closest('[hidden]') && !e.classList.contains('det-ctl')).map((e) => ({ e: e.id || e.className, r: e.getBoundingClientRect() }))
    .filter(({ r }) => r.width > 0 && r.height > 0).map(({ e, r: q }) => ({ id: String(e).slice(0, 24), x: q.left - b.left, y: q.top - b.top, w: q.width, h: q.height }));
  return { ...ARU.layout, rect: r, ui, box: { left: b.left, top: b.top }, auto: ARU.pick == null, sw: document.scrollingElement.scrollWidth, iw: window.innerWidth };
});
// page point of a marker's centre (from the layout record)
const centre = (L, id) => { const m = L.markers.find((x) => x.id === String(id)); return { x: L.box.left + m.x + m.w / 2, y: L.box.top + m.y + m.h / 2 }; };
const overlaps = (labels) => {
  const bad = [];
  for (let i = 0; i < labels.length; i++) for (let j = i + 1; j < labels.length; j++) {
    const a = labels[i], b = labels[j];
    if (a.x < b.x + b.w - 0.5 && b.x < a.x + a.w - 0.5 && a.y < b.y + b.h - 0.5 && b.y < a.y + a.h - 0.5) bad.push(`${a.kind}:${a.id ?? 'sum'} x ${b.kind}:${b.id ?? 'sum'}`);
  }
  return bad;
};

const shots = [], rows = [];
let bad = 0, checks = 0;
const check = (name, ok, d = '') => { checks++; if (!ok) { bad++; console.log(`  FAIL ${name} ${d}`); } };
const BOARD = new Set(Array.from({ length: 12 }, (_, i) => String(i)));
const SUM = { zh: ['ArUco 标定板标记 12 个', '3 个不在标定板上（疑似误检）：17 27 34'], en: ['ArUco board markers: 12', '3 not on the board (likely false): 17 27 34'] };
// short forms (used where the full panel does not fit; the only form on a picture narrower than 560 px)
const SUM_S = { zh: ['标定板 12', '误检 17 27 34'], en: ['Board 12', 'false 17 27 34'] };
const sumOk = (lines, lang, narrow) => {
  const j = JSON.stringify, sh = SUM_S[lang];
  return !!lines && ((!narrow && j(lines.slice(0, 2)) === j(SUM[lang])) || j(lines) === j([sh.join(' · ')]) || j(lines) === j(sh));
};
const VIEWS = [['desktop', 1280, 800], ['phone', 375, 812]];
for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, w, h] of VIEWS) {
  const tag = `${ui}_${lang}_${vp}`;
  const touch = vp === 'phone';
  const ctx = await browser.newContext({ viewport: { width: w, height: h }, deviceScaleFactor: touch ? 2 : 1, hasTouch: touch, isMobile: touch });
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed, [ENV, CW.DICT_4X4_50]);
  await page.waitForTimeout(700);
  const tap = async (p) => { if (touch) await page.touchscreen.tap(p.x, p.y); else await page.mouse.click(p.x, p.y); await page.waitForTimeout(120); };

  // 1. default frame
  let L = await layout(page);
  const all = [...L.labels, ...(L.summary ? [{ ...L.summary, kind: 'summary', id: null }] : [])];
  const r = L.rect, outside = all.filter((b) => b.x < r.x - 0.5 || b.y < r.y - 0.5 || b.x + b.w > r.x + r.w + 0.5 || b.y + b.h > r.y + r.h + 0.5);
  const huds = L.labels.filter((b) => b.kind === 'hud');
  const dim = L.markers.filter((m) => m.dim).map((m) => m.id).sort();
  const dimTags = L.labels.filter((b) => b.kind === 'tag' && b.dim);
  const ov = overlaps(all);
  const underUi = all.filter((a) => L.ui.some((b) => a.x < b.x + b.w - 0.5 && b.x < a.x + a.w - 0.5 && a.y < b.y + b.h - 0.5 && b.y < a.y + a.h - 0.5))
    .map((a) => `${a.kind}:${a.id ?? 'sum'}`);
  check(`${tag} markers`, L.n === 15 && L.markers.length === 15, JSON.stringify(L.markers.map((m) => m.id)));
  check(`${tag} no label overlap`, ov.length === 0, ov.join(', '));
  check(`${tag} labels clear of the chips and banner`, underUi.length === 0, underUi.join(', '));
  check(`${tag} labels inside the picture`, outside.length === 0, JSON.stringify(outside));
  const narrow = L.rect.w < 560;
  const hudLines = narrow ? 2 : 3;
  check(`${tag} one HUD on a board marker`, huds.length === 1 && BOARD.has(huds[0].id) && L.hud === huds[0].id && huds[0].lines.length === hudLines, JSON.stringify(huds));
  check(`${tag} look-alikes dimmed`, dim.join() === FALSE_IDS.map(String).sort().join() && dimTags.every((b) => b.text === b.id + '?') && !huds.some((b) => FALSE_IDS.includes(Number(b.id))), JSON.stringify({ dim, dimTags }));
  check(`${tag} summary`, L.summary && sumOk(L.summary.lines, lang, narrow), JSON.stringify(L.summary));
  check(`${tag} no h-scroll`, L.sw <= L.iw, `${L.sw} > ${L.iw}`);
  const tags = L.labels.filter((b) => b.kind === 'tag');
  rows.push(`${tag.padEnd(22)} markers ${L.n}, HUD ${L.hud}, tags ${tags.length} (board ${tags.length - dimTags.length}, dim ${dimTags.length}), skipped ${L.skipped}, overlaps ${ov.length}, under UI ${underUi.length}, outside ${outside.length}`);
  let file = path.join(out, `aruco_${tag}.png`); await page.screenshot({ path: file }); shots.push(file);
  const auto = L.hud;

  // 2. tap board marker 3 (not the automatic choice) -> HUD on 3, kept after the pointer leaves; tap again -> automatic
  const pickId = auto === '3' ? '8' : '3';
  await tap(centre(L, pickId));
  if (!touch) await page.mouse.move(2, h - 2);
  await page.waitForTimeout(120);
  L = await layout(page);
  check(`${tag} tap selects`, L.hud === pickId && !L.auto && overlaps([...L.labels, ...(L.summary ? [{ ...L.summary, kind: 'summary' }] : [])]).length === 0, JSON.stringify({ hud: L.hud, auto: L.auto }));
  file = path.join(out, `aruco_${tag}_pick${pickId}.png`); await page.screenshot({ path: file }); shots.push(file);
  await tap(centre(L, pickId));
  if (!touch) await page.mouse.move(2, h - 2);
  await page.waitForTimeout(120);
  L = await layout(page);
  check(`${tag} tap again -> auto`, L.hud === auto && L.auto, JSON.stringify({ hud: L.hud, auto: L.auto }));
  await tap(centre(L, 34));
  if (!touch) await page.mouse.move(2, h - 2);
  await page.waitForTimeout(120);
  L = await layout(page);
  check(`${tag} look-alike tap ignored`, L.hud === auto && L.auto, JSON.stringify({ hud: L.hud, auto: L.auto }));
  if (!touch) {
    await page.mouse.move(centre(L, 11).x, centre(L, 11).y); await page.waitForTimeout(120);
    L = await layout(page);
    check(`${tag} hover shows HUD`, L.hud === '11' && overlaps([...L.labels, ...(L.summary ? [{ ...L.summary, kind: 'summary' }] : [])]).length === 0, JSON.stringify({ hud: L.hud }));
    file = path.join(out, `aruco_${tag}_hover11.png`); await page.screenshot({ path: file }); shots.push(file);
    await page.mouse.move(2, h - 2); await page.waitForTimeout(120);
  }

  // 3. E-stop reachable with the overlay on
  const es = await page.evaluate(() => {
    const el = document.getElementById('btn-estop'), b = el.getBoundingClientRect(), x = b.left + b.width / 2, y = b.top + b.height / 2;
    const at = document.elementFromPoint(x, y);
    return { x, y, ok: b.width > 0 && x >= 0 && y >= 0 && x < innerWidth && y < innerHeight && !!at && el.contains(at) };
  });
  if (es.ok) await tap(es);
  const ran = await page.evaluate(() => window.__estop);
  check(`${tag} E-stop reachable`, es.ok && ran >= 1, JSON.stringify({ ...es, ran }));
  check(`${tag} page errors`, errs.length === 0, [...new Set(errs)].join(' | ').slice(0, 300));
  await ctx.close();
}
await browser.close();
srv.close();
for (const l of rows) console.log(l);
console.log(bad ? `${bad} of ${checks} checks failed` : `${checks} checks, ${shots.length} screenshots OK (${path.relative(root, out)})`);
process.exit(bad ? 1 : 0);
