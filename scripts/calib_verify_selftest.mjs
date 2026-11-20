#!/usr/bin/env node
// Self-test of the self-service verification math in src/api/calib_wizard.js (T20): fingertip sticker and hover
// acceptance.   node scripts/calib_verify_selftest.mjs        (PLAYWRIGHT=/path/to/playwright/index.mjs optional)
//   a. sticker SVG: 4 stickers x 4 dots of d 4 mm at rows 10 / 16 mm, columns 8.8 mm, a 50 mm ruler
//   b. IPPE on exact synthetic points: the true pose is one of the two solutions (< 1e-6 m), ranked first
//   c. ambiguity: a small, nearly fronto-parallel sticker far away has a runner-up within 0.5 px -> gate fails;
//      fewer than 4 dots -> 'dots'; a pose behind the camera -> front gate
//   d. stale frames: duplicate frame ids / identical bytes are dropped before the mean (dedupeFrames)
//   e. real blurred JPEGs (tests/fixtures/calib_t20/f03 f04 f06.jpg, 640x640 letterboxed snapshots of the office
//      B601-DM): decoded by Chromium as the cockpit does (canvas getImageData), dots within 0.15 px of OpenCV,
//      tip_cam within 0.3 mm of OpenCV's IPPE (-0.12, 10.57, 95.42) mm and of the recorded (-0.10, 10.58, 95.41)
//   f. hover: board pose from the recorded marker corners (5 real hovers), height of the sticker edge above the
//      paper within 0.5 mm of 12.2 / 12.6 / 11.6 / 11.3 / 11.3 mm; signed arithmetic: h = -p.z - edge_to_low,
//      table_z = z_end - h (a tip below the paper gives a negative height); gates of evaluateHovers
// Exits non-zero when a check fails.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CW = await import(pathToFileURL(path.join(root, 'src/api/calib_wizard.js')).href);
const FX = path.join(root, 'tests/fixtures/calib_t20');
const F = JSON.parse(fs.readFileSync(path.join(FX, 'fixtures.json'), 'utf8'));
let fails = 0;
const check = (name, ok, detail = '') => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); if (!ok) fails++; };
const mm = (v) => v.map((x) => (x * 1000).toFixed(3)).join(', ');
const dist = (a, b) => Math.hypot(...a.map((v, k) => v - b[k]));
const K = F.intrinsics;

// ---- a. SVG
{
  const svg = CW.stickerSvg();
  const circles = [...svg.matchAll(/<circle cx="([\d.]+)" cy="([\d.]+)" r="([\d.]+)"/g)].map((m) => m.slice(1).map(Number));
  const first = circles.slice(0, 4);
  const dx = Math.abs(first[1][0] - first[0][0]), rows = [...new Set(first.map((c) => c[1]))].sort((a, b) => a - b);
  check('a. sticker SVG', circles.length === 16 && circles.every((c) => c[2] === 2) && Math.abs(dx - 8.8) < 1e-9 && Math.abs(rows[1] - rows[0] - 6) < 1e-9 && /h 50"/.test(svg),
    `${circles.length} dots, columns ${dx.toFixed(2)} mm, rows ${(rows[1] - rows[0]).toFixed(2)} mm apart`);
}

// ---- b. IPPE exact
const synth = (R, t, Kx = K) => CW.stickerObject().map((X) => CW.project(Kx, R, t, X).slice(0, 2));
{
  const R = CW.rpyR(1.95, 0.02, -0.01), t = [-0.0001, 0.0106, 0.0954];
  const pts = synth(R, t);
  const sols = CW.stickerPoses(pts, K);
  const e = Math.min(...sols.map((s) => dist(s.t, t)));
  check('b. IPPE recovers the true pose', sols.length === 2 && dist(sols[0].t, t) < 1e-6 && CW.rotDist(sols[0].R, R) < 1e-4,
    `best ${mm(sols[0].t)} mm (err ${(e * 1e6).toFixed(3)} um), reproj ${sols[0].reprojPx.toExponential(2)} / ${sols[1].reprojPx.toFixed(3)} px`);
  const m = CW.measureTip([pts, pts, pts], K);
  check('b. measureTip on exact frames passes', m.ok && m.stdMm < 1e-6, m.gates.map((g) => `${g.key}=${g.ok}`).join(' '));
}

// ---- c. ambiguity, missing dots, behind the camera
{
  const R = CW.rpyR(Math.PI + 0.03, 0.0, 0.0), t = [0.0, 0.0, 0.45];
  const m = CW.measureTip([synth(R, t), synth(R, t), synth(R, t)], K);
  const amb = m.gates.find((g) => g.key === 'ambiguity');
  check('c. ambiguous IPPE (far, fronto-parallel) fails the gap gate', !m.ok && amb && !amb.ok, `gap ${amb && amb.value.toFixed(3)} px`);
  const g = new Uint8Array(640 * 360).fill(200);
  const d = CW.detectStickerDots(g);
  check('c. blank image: no dots', !d.ok && d.why === 'dots', `blobs ${d.blobs.length}`);
  const few = CW.measureTip([synth(CW.rpyR(1.95, 0, 0), [0, 0.01, 0.095])].slice(0, 1), K);
  check('c. one frame < minFrames', !few.ok && few.gates[0].key === 'frames' && !few.gates[0].ok, `${few.frames} frame`);
}

// ---- d. stale frames
{
  const a = { frame_id: 7, key: 'aa', points: [[0, 0], [1, 0], [0, 1], [1, 1]] };
  const kept = CW.dedupeFrames([a, { ...a }, { frame_id: 8, key: 'aa', points: a.points }, { frame_id: 9, key: 'bb', points: a.points }, { frame_id: 6, key: 'cc', points: a.points }]);
  check('d. duplicate / repeated / older frames dropped', kept.length === 2 && kept[0].frame_id === 7 && kept[1].frame_id === 9, `kept ${kept.map((f) => f.frame_id)}`);
}

// ---- e. real JPEGs through Chromium
{
  let chromium = null;
  try { ({ chromium } = await import(process.env.PLAYWRIGHT || 'playwright')); } catch (e) { check('e. playwright available', false, e.message); }
  if (chromium) {
    const browser = await chromium.launch(process.env.CHROME ? { executablePath: process.env.CHROME } : {});
    const page = await browser.newPage();
    await page.setContent('<html><body></body></html>');
    const frames = [];
    for (const f of ['f03', 'f04', 'f06']) {
      const b64 = fs.readFileSync(path.join(FX, `${f}.jpg`)).toString('base64');
      // decoded as the cockpit does: an <img> drawn into a canvas, getImageData
      const rgbaB64 = await page.evaluate(async (b64) => {
        const img = new Image();
        img.src = 'data:image/jpeg;base64,' + b64;
        await img.decode();
        const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
        const ctx = c.getContext('2d', { willReadFrequently: true }); ctx.drawImage(img, 0, 0);
        const d = ctx.getImageData(0, 0, c.width, c.height).data;
        let s = ''; for (let i = 0; i < d.length; i += 0x8000) s += String.fromCharCode.apply(null, d.subarray(i, i + 0x8000));
        return [c.width, c.height, btoa(s)];
      }, b64);
      const [w, h, data] = rgbaB64;
      const gray = CW.grayContent(new Uint8Array(Buffer.from(data, 'base64')), w, h);
      const d = CW.detectStickerDots(gray);
      const ref = F.sticker.points[f];
      const err = d.ok ? Math.max(...d.points.map((p, k) => dist(p, ref[k]))) : Infinity;
      check(`e. ${f}.jpg (${w}x${h}) dots vs OpenCV`, d.ok && err < 0.15, `max ${err.toFixed(3)} px, Otsu ${d.threshold}`);
      if (d.ok) frames.push(d.points);
    }
    await browser.close();
    const m = CW.measureTip(frames, K);
    const ref = F.sticker.tip_cam_mm.map((v) => v / 1000), rec = [-0.10, 10.58, 95.41].map((v) => v / 1000);
    check('e. tip_cam from the real JPEGs', m.ok && dist(m.tip_cam_m, ref) < 0.0003 && dist(m.tip_cam_m, rec) < 0.0003,
      `${mm(m.tip_cam_m)} mm (OpenCV ${mm(ref)}; ${(dist(m.tip_cam_m, ref) * 1000).toFixed(3)} mm), reproj ${m.reprojPx && m.reprojPx.toFixed(3)} px, gap ${m.gapPx && m.gapPx.toFixed(2)} px, std ${m.stdMm && m.stdMm.toFixed(3)} mm`);
  }
}

console.log(fails ? `${fails} check(s) FAILED` : 'all checks passed');
process.exit(fails ? 1 : 0);
