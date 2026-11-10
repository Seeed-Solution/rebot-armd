#!/usr/bin/env node
// Headless check of the camera-view / base frame in the cartesian cockpit (no armd needed).
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome OUT=/dir node scripts/view_frame_shots.mjs
// Serves src/api/index.html with a mocked 7-DOF /api/v1/observation/schema, feeds a synthetic observation
// (state Armed, positions follow the streamed targets), and for simple / expert x zh / en x 1280x800 / 375x812
// writes a PNG (default .cache/view-shots/). Simple mode (stored rebot.cart.frame = base) must use the view
// frame with no frame toggle, pad-srot visible and btn-pose hidden. Expert mode, for view / base frame, must
// show the toggle with the matching option on and no pad-srot. No horizontal page scroll anywhere.
// Then drives W / A, the look stick (pad-srot) and E / ArrowRight / look step buttons, ArrowUp, W after a look,
// and W + look stick together in simple mode (view frame), and W / A in expert base frame. Look input must move
// only J4 (up / down) or J5 (left / right), others < 0.003 rad, with the matching optical-axis change.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = process.env.OUT || path.join(root, '.cache/view-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const LIMITS = [[-3.14159, 3.14159, 2], [-2.0944, 2.0944, 2], [-2.61799, 2.61799, 2], [-3.14159, 3.14159, 3], [-2.0944, 2.0944, 3], [-6.28318, 6.28318, 4], [-5, 5, 3]]
  .map(([a, b, v]) => ({ min_position: a, max_position: b, max_velocity: v }));
const json = (res, o) => { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/api/v1/observation/schema') return json(res, { dof: 7, limits: LIMITS });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const base = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
// synthetic arm: model "inspect" pose with J6 = pi/2; the observation follows whatever the page streams
const feed = () => {
  window.__sim = window.__sim || [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0.2];
  window.__sent = 0;
  window.cartSend = (t) => { window.__sim = t.slice(); window.__sent++; };
  setInterval(() => {
    S.obs = { positions: window.__sim.slice(), velocities: [0, 0, 0, 0, 0, 0, 0], safety_state: 'Armed', enable_state: 'Enabled', feedback_age_ms: [5, 5, 5, 5, 5, 5, 5], epoch_id: 1 };
    S.obsAt = Date.now(); S.state = 'Armed'; S.epoch = 1; S.leaseGeneration = 1;
  }, 20);
};
const shots = [];
let bad = 0;
const fail = (m) => { bad++; console.log('  FAIL ' + m); };
for (const ui of ['simple', 'expert']) for (const lang of ['zh', 'en']) for (const [vp, viewport] of [['1280x800', { width: 1280, height: 800 }], ['375x812', { width: 375, height: 812 }]]) {
  const ctx = await browser.newContext({ viewport, deviceScaleFactor: 1 });
  // a stored base frame from expert mode must not leak into simple mode
  await ctx.addInitScript(([l]) => { try { localStorage.clear(); localStorage.setItem('lang', l); localStorage.setItem('rebot.ctl', 'cart'); localStorage.setItem('rebot.cart.frame', 'base'); } catch (_) {} }, [lang]);
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  for (const frame of ui === 'simple' ? ['view'] : ['view', 'base']) {
    if (ui === 'expert') await page.evaluate((f) => { setFrame(f); }, frame);
    await page.waitForTimeout(300);
    const info = await page.evaluate((ui) => {
      const visEl = (el) => { const r = el.getBoundingClientRect(); return r.width > 0 && r.height > 0 && getComputedStyle(el).visibility !== 'hidden'; };
      const vis = (id) => { const el = document.getElementById(id); return !!el && visEl(el); };
      const seg = document.getElementById('frame-seg-e');
      return {
        dof: S.dof, frame: C.frame, view: viewActive(),
        toggle: [...document.querySelectorAll('.frame-seg, [data-frame]')].some(visEl),
        segOn: [...seg.children].filter((b) => b.classList.contains('on')).map((b) => b.dataset.frame),
        srot: vis('pad-srot'), pose: vis('btn-pose'),
        sw: document.scrollingElement.scrollWidth, iw: window.innerWidth,
        label: document.querySelector(ui === 'simple' ? '#pad-srot .dir.t .lbl' : '#vcol-r .vlab b').textContent,
        kb: document.querySelector('[data-i18n="kb.x"]').textContent,
      };
    }, ui);
    const tag = `${ui}_${lang}_${vp}_${frame}`;
    const file = path.join(out, `view_${tag}.png`);
    await page.screenshot({ path: file });
    shots.push(file);
    const exp = frame === 'view';
    if (info.dof !== 7) fail(`${tag} dof ${info.dof}`);
    if (info.view !== exp) fail(`${tag} viewActive=${info.view}`);
    if (ui === 'simple' && info.toggle) fail(`${tag} frame toggle visible in simple mode`);
    if (ui === 'expert' && (!info.toggle || info.segOn.length !== 1 || info.segOn[0] !== frame)) fail(`${tag} frame toggle ${info.toggle} ${info.segOn}`);
    if (ui === 'simple' && info.srot !== exp) fail(`${tag} pad-srot visible=${info.srot}`);
    if (ui === 'simple' && info.pose === exp) fail(`${tag} btn-pose visible=${info.pose}`);
    if (ui === 'expert' && info.srot) fail(`${tag} pad-srot visible in expert`);
    if (info.sw > info.iw) fail(`${tag} horizontal scroll ${info.sw} > ${info.iw}`);
    console.log(`${tag}  view=${info.view} toggle=${info.toggle} on=${info.segOn} pad-srot=${info.srot} btn-pose=${info.pose} scrollWidth=${info.sw}<=${info.iw} label="${info.label}" kb.x="${info.kb}"`);
  }
  if (errs.length) fail(`page errors (${ui}/${lang}/${vp}): ${[...new Set(errs)].join(' | ').slice(0, 300)}`);
  await ctx.close();
}

// motion directions (zh, desktop): hold a key 400 ms, compare the commanded pose before / after.
// view frame in simple mode (stored frame = base, ignored there), base frame in expert mode.
for (const [ui, frame] of [['simple', 'view'], ['expert', 'base']]) {
  const ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  await ctx.addInitScript(() => { try { localStorage.clear(); localStorage.setItem('lang', 'zh'); localStorage.setItem('rebot.ctl', 'cart'); localStorage.setItem('rebot.cart.frame', 'base'); } catch (_) {} });
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(e.message));
  await page.goto(base + (ui === 'expert' ? '?ui=expert' : ''), { waitUntil: 'domcontentloaded' });
  await page.evaluate(feed);
  await page.waitForTimeout(600);
  const snap = () => page.evaluate(() => {
    const q = window.__sim, T = VIK.fk([...toModel(q), q[4], q[5]]), c = VIK.camAxes(T, CFG.view.Rec);
    return { p: CALM.posOf(T), z: c.z, x: c.x, q: [...toModel(q), q[4], q[5]], sent: window.__sent, status: C.status };
  });
  const hold = async (code, ms = 400) => {
    await page.evaluate((c) => window.dispatchEvent(new KeyboardEvent('keydown', { code: c, key: c })), code);
    await page.waitForTimeout(ms);
    await page.evaluate((c) => window.dispatchEvent(new KeyboardEvent('keyup', { code: c, key: c })), code);
    await page.waitForTimeout(700);
  };
  const sub = (a, b) => a.map((v, k) => v - b[k]), dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], nrm = (a) => Math.hypot(...a);
  const check = (name, ok, d) => { console.log(`${ok ? 'PASS' : 'FAIL'}  motion ${name}  ${d}`); if (!ok) bad++; };
  {
    // start from the inspect pose (J5 = 0: the base-frame IK holds J5 and assumes it at 0)
    await page.evaluate(() => { window.__sim = [0, -1.167, -1.565, 1.435, 0, Math.PI / 2, 0.2]; cartSync(window.__sim); });
    await page.waitForTimeout(300);
    const active = await page.evaluate(() => viewActive());
    check(`${ui} uses the ${frame} frame`, active === (frame === 'view'), `viewActive=${active}`);
    let s0 = await snap(); await hold('KeyW'); let s1 = await snap();
    let d = sub(s1.p, s0.p);
    if (frame === 'view') check('view W = forward along z_cam', nrm(d) > 0.005 && dot(d, s0.z) / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, cos(d, z_cam) ${(dot(d, s0.z) / nrm(d)).toFixed(4)}, frames sent ${s1.sent - s0.sent}`);
    else check('base W = +X', nrm(d) > 0.005 && d[0] / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, dir ${d.map((v) => (v / nrm(d)).toFixed(3))}`);
    s0 = await snap(); await hold('KeyA'); s1 = await snap(); d = sub(s1.p, s0.p);
    if (frame === 'view') check('view A = left along -x_cam', nrm(d) > 0.005 && -dot(d, s0.x) / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, cos(d, -x_cam) ${(-dot(d, s0.x) / nrm(d)).toFixed(4)}`);
    else check('base A = +Y', nrm(d) > 0.005 && d[1] / nrm(d) > 0.99, `moved ${(nrm(d) * 1000).toFixed(1)} mm, dir ${d.map((v) => (v / nrm(d)).toFixed(3))}`);
    if (frame === 'view') {
      // look input = wrist only: J4 (up / down) or J5 (left / right); J1..J3 and J6 within 0.003 rad
      const J = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
      const dj = (a0, a1) => a1.q.slice(0, 6).map((v, i) => v - a0.q[i]);
      const fmt = (d) => d.map((v, i) => `${J[i]} ${v >= 0 ? '+' : ''}${v.toFixed(4)}`).join(' ');
      const only = (d, keep) => d.every((v, i) => keep.includes(i) ? Math.abs(v) > 0.05 : Math.abs(v) < 0.003);
      const heading = (z) => Math.atan2(z[1], z[0]);
      // drag the look stick (pad-srot) toward (ux, uy) of its radius for ms, then release
      const stick = async (id, ux, uy, ms = 400, onHold = null, onLeave = null) => {
        const r = await page.evaluate((i) => { const b = document.getElementById(i).getBoundingClientRect(); return { x: b.x + b.width / 2, y: b.y + b.height / 2, r: b.width / 2 }; }, id);
        await page.mouse.move(r.x, r.y); await page.mouse.down();
        await page.mouse.move(r.x + ux * r.r, r.y + uy * r.r, { steps: 4 });
        if (onHold) await onHold();
        await page.waitForTimeout(ms);
        if (onLeave) await onLeave();
        await page.mouse.up(); await page.waitForTimeout(700);
      };
      const look = async (name, act, keep, dirOk, dirTxt) => {
        const a0 = await snap(); await act(); const a1 = await snap();
        const d = dj(a0, a1), ok = only(d, keep) && dirOk(a0, a1);
        check(name, ok, `${fmt(d)} | ${dirTxt(a0, a1)}`);
        return a1;
      };
      const up = (a0, a1) => a1.z[2] > a0.z[2] + 0.02, upTxt = (a0, a1) => `z_cam.Z ${a0.z[2].toFixed(3)} -> ${a1.z[2].toFixed(3)}`;
      const rt = (a0, a1) => dot(sub(a1.z, a0.z), a0.x) > 0.02 && heading(a1.z) < heading(a0.z),
        rtTxt = (a0, a1) => `dz_cam . x_cam ${dot(sub(a1.z, a0.z), a0.x).toFixed(3)}, heading ${(heading(a0.z) * 57.3).toFixed(1)} -> ${(heading(a1.z) * 57.3).toFixed(1)} deg`;
      await look('view right stick up = J4 only, look up', () => stick('pad-srot', 0, -0.9), [3], up, upTxt);
      await look('view right stick right = J5 only, look right', () => stick('pad-srot', 0.9, 0), [4], rt, rtTxt);
      await look('view key E = J4 only, look up', () => hold('KeyE'), [3], up, upTxt);
      await look('view key ArrowRight = J5 only, look right', () => hold('ArrowRight'), [4], rt, rtTxt);
      const dn = (a0, a1) => a1.z[2] < a0.z[2] - 0.02, dnTxt = upTxt;
      await look('view step button look down = J4 only', async () => {
        await page.evaluate(() => { setStep(2); cartStep('p', 1); cartStep('p', 1); }); await page.waitForTimeout(900);
      }, [3], dn, dnTxt);
      // lift (world Z) through the IK, orientation held
      s0 = await snap(); await hold('ArrowUp'); s1 = await snap(); d = sub(s1.p, s0.p);
      check('view ArrowUp = lift along world Z, orientation held', nrm(d) > 0.005 && d[2] / nrm(d) > 0.99 && dot(s1.z, s0.z) > 0.9999 && dot(s1.x, s0.x) > 0.9999,
        `moved ${(nrm(d) * 1000).toFixed(1)} mm, dir.Z ${(d[2] / nrm(d)).toFixed(4)}, cos z_cam ${dot(s1.z, s0.z).toFixed(6)}, cos x_cam ${dot(s1.x, s0.x).toFixed(6)}`);
      // after the J5 moves above the camera carries a roll (x_cam.Z != 0): W must not re-level it
      s0 = await snap(); await hold('KeyW'); s1 = await snap(); d = sub(s1.p, s0.p);
      check('view W after look: no auto-level roll, orientation held', nrm(d) > 0.005 && dot(d, s0.z) / nrm(d) > 0.99 && Math.abs(s1.x[2] - s0.x[2]) < 0.002 && dot(s1.z, s0.z) > 0.9999,
        `x_cam.Z ${s0.x[2].toFixed(4)} -> ${s1.x[2].toFixed(4)}, cos(d, z_cam) ${(dot(d, s0.z) / nrm(d)).toFixed(4)}, joints ${fmt(dj(s0, s1))}`);
      // both sticks: forward key held only while the look stick is held right. The wrist delta is applied first,
      // then the translation with that orientation held (no auto-level), so J6 is not used to roll the camera
      // back. A/B from the same rolled start pose (J5 = 0.3): W alone vs W + look right, same duration.
      const START = [0, -1.167, -1.565, 1.435, 0.3, Math.PI / 2, 0.2];
      const reset = async () => { await page.evaluate((q) => { window.__sim = q.slice(); cartSync(q); }, START); await page.waitForTimeout(300); };
      const wDown = () => page.evaluate(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', key: 'KeyW' })));
      const wUp = () => page.evaluate(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', key: 'KeyW' })));
      await reset(); s0 = await snap(); await wDown(); await page.waitForTimeout(400); await wUp(); await page.waitForTimeout(700); s1 = await snap();
      const dA = dj(s0, s1);
      // B: record every wrist step (viewWrist) during the combined move; it must change J4 / J5 only
      await reset();
      await page.evaluate(() => {
        window.__ww = { n: 0, other: 0, j5: 0 };
        const f = window.viewWrist;
        window.viewWrist = (dq) => {
          const q0 = C.q6.slice(), r = f(dq), q1 = C.q6;
          window.__ww.n++; window.__ww.j5 += q1[4] - q0[4];
          for (const i of [0, 1, 2, 5]) window.__ww.other = Math.max(window.__ww.other, Math.abs(q1[i] - q0[i]));
          return r;
        };
      });
      const b0 = await snap(); await stick('pad-srot', 0.9, 0, 400, wDown, wUp); const b1 = await snap();
      const ww = await page.evaluate(() => window.__ww);
      const dB = dj(b0, b1), dpB = sub(b1.p, b0.p);
      check('view W alone (A/B reference)', nrm(sub(s1.p, s0.p)) > 0.005, `${fmt(dA)} | x_cam.Z ${s0.x[2].toFixed(4)} -> ${s1.x[2].toFixed(4)}`);
      check('view both sticks: look stick moves J4 / J5 only, look right, roll not levelled',
        ww.n > 5 && ww.other === 0 && ww.j5 > 0.05 && nrm(dpB) > 0.005 && rt(b0, b1) && Math.abs(b1.x[2]) >= Math.abs(b0.x[2]) - 0.002,
        `${fmt(dB)} | wrist steps ${ww.n}: J5 sum ${ww.j5.toFixed(4)}, max |dJ1..3, dJ6| ${ww.other}; J6 B-A ${(dB[5] - dA[5]).toFixed(4)} (translation IK holding the new orientation); moved ${(nrm(dpB) * 1000).toFixed(1)} mm, x_cam.Z ${b0.x[2].toFixed(4)} -> ${b1.x[2].toFixed(4)}`);
    }
  }
  if (errs.length) { bad++; console.log('  FAIL page errors: ' + [...new Set(errs)].join(' | ').slice(0, 300)); }
  await ctx.close();
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK, all checks passed`);
process.exit(bad ? 1 : 0);
