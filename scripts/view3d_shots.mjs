#!/usr/bin/env node
// Headless check of the cockpit 3D view (no armd needed):
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/view3d_shots.mjs
// Serves src/api/index.html, the embedded bundle src/api/vendor/three-urdf.min.js.gz (Content-Encoding: gzip),
// /assets/model/* from the kit asset directory (MODEL_DIR, default the recamera-b601dm rootfs assets) and
// mocked schema / keep-out / poses endpoints; answers the page's WebSocket with observations at 50 Hz.
// Checks, at 375x812 (simple) and 1280x800 (expert), with meshes and without (lite: stick figure):
// no three.js or model request before the 3D panel opens; no console errors; the canvas has non-background
// pixels; ghost, workspace, keep-out and pose objects exist; end_link FK within 5 mm of
// tests/fixtures/fk_points.json (armd kinematics::fk); at most ~30 renders/s while moving and none while still.
// Writes PNGs (default .cache/view3d-shots/).
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/view3d-shots');
fs.mkdirSync(out, { recursive: true });
const modelDir = process.env.MODEL_DIR || path.join(root, 'kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets');
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const bundle = fs.readFileSync(path.join(root, 'src/api/vendor/three-urdf.min.js.gz'));
const FK = JSON.parse(fs.readFileSync(path.join(root, 'tests/fixtures/fk_points.json'), 'utf8')).points;
const LIMITS = [[-2.8, 2.8, 2], [-3.14, 0, 2], [-3.14, 0, 2], [-1.87, 1.57, 3], [-1.57, 1.57, 3], [-3.14, 3.14, 4], [-5.75, 0.1, 2]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const WS = { floor_z_m: 0, box_min_m: [-0.6, -0.6, -1.0], box_max_m: [0.6, 0.6, 0.8], max_reach_m: 0.52, check_points_end_m: [], keepout_boxes: [] };
const KEEPOUT = { boxes: [{ name: 'camera_post', min_m: [0.18, -0.32, 0.0], max_m: [0.26, -0.24, 0.45] }], source: 'runtime' };
const POSES = [{ name: 'bin_a', positions: [0.6, -1.0, -1.4, 1.3, 0, 1.57, 0], tcp: { position: [0.19848, 0.135846, 0.300004], rpy: [3.14, 0, 0] }, saved_at_ms: 1762828800000 }];

let lite = false;
const srv = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split('?')[0]);
  const json = (o, code = 200) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/three-urdf.min.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8', 'content-encoding': 'gzip' }); res.end(bundle); return; }
  if (u.startsWith('/assets/model/')) {
    const rel = u.slice('/assets/model/'.length);
    const f = path.join(modelDir, rel);
    if (lite || rel.includes('..') || !fs.existsSync(f) || !fs.statSync(f).isFile()) { res.writeHead(404); res.end(); return; }
    res.writeHead(200); res.end(fs.readFileSync(f)); return;
  }
  if (u === '/api/v1/observation/schema') return json({ dof: 7, limits: LIMITS, workspace: WS, capabilities: { version: 6, poses: true, fault_codes: true } });
  if (u === '/api/v1/workspace/keepout') return json(KEEPOUT);
  if (u === '/api/v1/poses') return json({ poses: POSES });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;

let bad = 0, checks = 0;
const ok = (c, m) => { checks++; if (!c) { bad++; console.log('  FAIL ' + m); } };
const shots = [];
const Q = [0, -1.167, -1.565, 1.435, 0, 1.5708, -2.85];
const QC = [0.35, -1.0, -1.3, 1.2, 0, 1.5708, -2.85]; // commanded: ghost differs from measured

const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined, args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'] });
try {
  for (const mode of ['mesh', 'lite']) for (const [vp, viewport, ui] of [['375x812', { width: 375, height: 812 }, 'simple'], ['1280x800', { width: 1280, height: 800 }, 'expert']]) {
    lite = mode === 'lite';
    const tag = `${mode}_${ui}_${vp}`;
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
    await ctx.addInitScript(() => { try { localStorage.clear(); localStorage.setItem('lang', 'zh'); } catch (_) {} });
    const page = await ctx.newPage();
    const errs = [], reqs = [];
    page.on('pageerror', (e) => errs.push('pageerror: ' + e.message));
    // resource 404s of endpoints this mock does not serve (video, detection) are logged by Chrome as console
    // errors; they are checked per response below instead
    page.on('console', (m) => { if (m.type() === 'error' && !/^Failed to load resource|^WebSocket connection to .*\/detection\/ws/.test(m.text())) errs.push('console: ' + m.text()); });
    const failed3d = [];
    page.on('response', (r) => { const u = new URL(r.url()).pathname; if (r.status() >= 400 && /^\/assets\/(three|model)/.test(u)) failed3d.push(u + ' ' + r.status()); });
    page.on('request', (r) => reqs.push(new URL(r.url()).pathname));
    const P = { moving: false, t0: Date.now() };
    await page.routeWebSocket(/\/api\/v1\/ws/, (ws) => {
      ws.onMessage((raw) => {
        let m; try { m = JSON.parse(String(raw)); } catch (_) { return; }
        if (m.type === 'acquire_lease') ws.send(JSON.stringify({ type: 'lease_granted', seq: m.seq, generation: 7, epoch_id: 1, client_id: m.client_id }));
        else if (m.type === 'command') ws.send(JSON.stringify({ type: 'command_ack', request_id: m.request_id, epoch_id: 1 }));
      });
      P.feed = setInterval(() => {
        const t = (Date.now() - P.t0) / 1000;
        const q = P.moving ? Q.map((v, i) => (i === 0 ? v + 0.3 * Math.sin(t * 2) : v)) : Q;
        const o = { timestamp: t, positions: q, velocities: q.map(() => 0), commanded_positions: QC, safety_state: 'Armed', enable_state: 'Enabled', is_torqued: true,
          feedback_age_ms: q.map(() => 5), torques: q.map(() => 0.5), temperatures: q.map(() => 35), epoch_id: 1, lease_holder: null,
          driver_capabilities: { velocity_feedback: true, torque_feedback: true, temperature_feedback: true }, motion_limit: null, motor_errors: [], action: null, fault: null };
        try { ws.send(JSON.stringify(o)); } catch (_) {}
      }, 20);
    });
    await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => typeof isLive === 'function' && isLive(), null, { timeout: 8000 });
    await page.waitForTimeout(800);
    const early = reqs.filter((r) => r.startsWith('/assets/three') || r.startsWith('/assets/model'));
    ok(early.length === 0, `${tag} no 3D request before opening (${early.join(',')})`);
    ok(await page.isVisible('#btn-3d'), `${tag} 3D button visible`);

    await page.click('#btn-3d');
    await page.waitForFunction(() => V3.renders > 0, null, { timeout: 15000 });
    await page.waitForTimeout(600);
    const st = await page.evaluate(() => ({ mode: V3.mode, status: document.getElementById('v3-status').textContent, hidden: document.getElementById('view3d').hidden }));
    ok(!st.hidden && st.mode === (lite ? 'stick' : 'mesh'), `${tag} 3D view open in ${st.mode} mode`);
    ok(lite ? st.status === '未安装机械臂网格，显示关节骨架' : st.status === '', `${tag} status "${st.status}"`);
    const loaded = reqs.filter((r) => r.startsWith('/assets/model/meshes/') && r.endsWith('.stl')).length;
    ok(lite ? loaded === 0 : loaded >= 30, `${tag} ${loaded} mesh files requested`);

    // canvas pixels (render and read back in the same task)
    const px = await page.evaluate(() => {
      V3.r.render(V3.scene, V3.cam);
      const c = document.getElementById('v3-canvas');
      const c2 = document.createElement('canvas'); c2.width = c.width; c2.height = c.height;
      const g = c2.getContext('2d'); g.drawImage(c, 0, 0);
      const d = g.getImageData(0, 0, c2.width, c2.height).data;
      let fg = 0, dark = 0, blue = 0;
      for (let i = 0; i < d.length; i += 4) {
        const r = d[i], gg = d[i + 1], b = d[i + 2];
        if (Math.abs(r - 0xee) + Math.abs(gg - 0xf2) + Math.abs(b - 0xea) > 24) fg++;
        if (r + gg + b < 200) dark++;
        if (b > r + 25 && b > gg + 10) blue++;
      }
      return { w: c2.width, h: c2.height, fg, dark, blue, n: d.length / 4 };
    });
    ok(px.w > 100 && px.h > 100 && px.fg > px.n * 0.03 && px.dark > 300, `${tag} canvas has drawn pixels ${JSON.stringify(px)}`);
    ok(px.blue > 200, `${tag} ghost pixels visible (${px.blue})`);
    const objs = await page.evaluate(() => {
      const names = []; V3.extras.traverse((o) => { if (o.name) names.push(o.name); });
      return { ghost: !!V3.ghost && V3.ghost.obj.visible && V3.ghost.obj.parent === V3.root, names };
    });
    ok(objs.ghost, `${tag} ghost object present and visible`);
    ok(objs.names.includes('workspace') && objs.names.filter((n) => n === 'keepout:camera_post').length === 2 && objs.names.includes('pose:bin_a'), `${tag} scene objects ${JSON.stringify(objs.names)}`);
    // FK: rendered end_link vs armd kinematics::fk
    const fk = await page.evaluate((pts) => pts.map((p) => {
      V3.solid.set(p.q.concat([0])); V3.solid.obj.updateMatrixWorld(true);
      const e = V3.solid.end();
      return Math.hypot(e[0] - p.end_m[0], e[1] - p.end_m[1], e[2] - p.end_m[2]) * 1000;
    }), FK);
    ok(fk.every((d) => d < 5), `${tag} FK end_link error mm ${fk.map((d) => d.toFixed(3)).join(', ')}`);
    console.log(`  ${tag} FK error mm: ${fk.map((d) => d.toFixed(4)).join(', ')}`);
    if (!lite) {
      const fing = await page.evaluate(() => { V3.solid.set([0, 0, 0, 0, 0, 0, CFG.gripOpen]); const a = V3.solid.obj.joints.left_finger_joint.angle; V3.solid.set([0, 0, 0, 0, 0, 0, CFG.gripClosed]); return [a, V3.solid.obj.joints.left_finger_joint.angle]; });
      ok(Math.abs(fing[0] - 0.0425) < 1e-6 && Math.abs(fing[1]) < 1e-6, `${tag} J7 open/closed -> finger ${fing}`);
    }
    await page.evaluate(() => { V3.key = ''; });
    await page.waitForTimeout(300);
    const f = path.join(out, `view3d_${tag}.png`); await page.screenshot({ path: f }); shots.push(f);

    // render rate: moving <= ~30 fps, still -> no renders
    P.moving = true; await page.waitForTimeout(300);
    let n0 = await page.evaluate(() => V3.renders); await page.waitForTimeout(1000);
    const moving = (await page.evaluate(() => V3.renders)) - n0;
    P.moving = false; await page.waitForTimeout(400);
    n0 = await page.evaluate(() => V3.renders); await page.waitForTimeout(1000);
    const still = (await page.evaluate(() => V3.renders)) - n0;
    ok(moving >= 5 && moving <= 31, `${tag} renders while moving ${moving}/s`);
    ok(still === 0, `${tag} renders while still ${still}/s`);
    await page.click('#v3-close');
    ok(await page.evaluate(() => document.getElementById('view3d').hidden && V3.raf === 0), `${tag} closed, render loop stopped`);
    const noH = await page.evaluate(() => document.scrollingElement.scrollWidth <= innerWidth);
    ok(noH, `${tag} no horizontal scroll`);
    ok(errs.length === 0, `${tag} console errors: ${errs.join(' | ')}`);
    // lite: only the URDF and manifest probes 404, no mesh request
    const want = lite ? ['/assets/model/meshes/manifest.json 404', '/assets/model/reBot-DevArm_gripper.urdf 404'] : [];
    ok(JSON.stringify(failed3d.sort()) === JSON.stringify(want), `${tag} failed 3D requests ${JSON.stringify(failed3d)}`);
    clearInterval(P.feed); await ctx.close();
  }
} finally {
  await browser.close();
  srv.close();
}
console.log(`${shots.length} screenshots in ${out}, ${checks} checks, ${bad ? bad + ' FAILED' : 'all passed'}`);
for (const s of shots) console.log('  ' + s);
process.exit(bad ? 1 : 0);
