#!/usr/bin/env node
// Headless screenshots and checks of the cockpit Teach panel (capabilities.teach v1), no armd needed:
//   PLAYWRIGHT=/path/to/playwright/index.mjs CHROME=/path/to/chrome node scripts/teach_shots.mjs
// A local server serves src/api/index.html, /assets/calib_wizard.js and mocks of GET /api/v1/sequences, /poses,
// /episodes, /datasets; the control WebSocket is a stub, commandRPC (lease commands) and teachCmd (lease-free
// commands) are replaced in the page by mocks that record the calls. Checked: the entry is hidden without
// capabilities.teach and shown with it; the hand-guide record button keeps the legacy dataset recording without
// it, and with it records into the teach store (record/start|stop) and opens the panel collapsed; sequence list with a stale badge, editor (reorder, speed, delete), run with repeat + record body, live step
// highlight from the parent action progress, Stop, DATA_STALE confirm -> allow_stale, the run result; teach bar
// while hand-guiding ("Save pose" -> poses/save + a pose step, "Record" -> record/start / stop + a replay step,
// gripper -> a gripper step), the collapsed bar docked on the video top (<= 48 px, off the E-stop / Enable /
// sticks); poses with "used by", rename, delete refused with IN_USE; episodes grouped by dataset with source /
// stale / aborted badges, edit task + dataset (episodes/update without a lease), replay DATA_STALE confirm;
// server strings as text; Space / Escape E-stop with focus in the panel. zh / en at desktop 1400x900 and phone
// 390x844; PNGs go to .cache/teach-shots/. Exits non-zero if a check fails.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { chromium } = await import(process.env.PLAYWRIGHT || 'playwright');
const out = path.join(root, '.cache/teach-shots');
fs.mkdirSync(out, { recursive: true });
const html = fs.readFileSync(path.join(root, 'src/api/index.html'));
const js = fs.readFileSync(path.join(root, 'src/api/calib_wizard.js'));

const STALE = { reason: 'motor zero of joints [2]', at: '2026-11-18T10:20:00Z' };
const POSES = [
  { id: '01JD0000000000000000POSE01', name: 'scan', positions: [0, -0.4, 0.8, 0, 0, 0, 0], saved_at_ms: 1763550000000 },
  { id: '01JD0000000000000000POSE02', name: 'bin_a', positions: [0.5, -0.3, 0.7, 0, 0, 0, 0], saved_at_ms: 1763550100000, stale: STALE },
  { id: '01JD0000000000000000POSE03', name: 'bin_b', positions: [-0.5, -0.3, 0.7, 0, 0, 0, 0], saved_at_ms: 1763550200000 },
];
const [P1, P2, P3] = POSES.map((p) => p.id);
const EPS = [
  { id: '01JD0000000000000000EPIS01', dataset: 'default', task: 'pick cube', source: 'hand_guide', outcome: 'ok', recorded_at: '2026-11-19T09:12:00Z', duration_s: 12.4, frames: 3720, stale: false },
  { id: '01JD0000000000000000EPIS02', dataset: 'cube_bin', task: '<img src=x onerror="window.__xss=1">', source: 'sequence', sequence_id: 'S1', repeat_index: 0, outcome: 'ok', recorded_at: '2026-11-19T09:30:00Z', duration_s: 31.0, frames: 9300, stale: true },
  { id: '01JD0000000000000000EPIS03', dataset: 'cube_bin', task: '', source: 'jog', outcome: 'aborted', recorded_at: '2026-11-19T09:40:00Z', duration_s: 4.1, frames: 1230, stale: false },
];
const [E1, E2] = EPS.map((e) => e.id);
const SEQS = [
  { id: 'S1', name: 'pick_place', description: '', rev: 3, steps: [
    { type: 'pose', pose_id: P1, speed_scale: 0.5, gripper: 'keep' }, { type: 'gripper', action: 'open' }, { type: 'wait', s: 1.5 },
    { type: 'pose', pose_id: P2, speed_scale: 0.3, gripper: 'pose' }, { type: 'gripper', action: 'close' }, { type: 'replay', episode_id: E1, speed: 1, gripper: true }] },
  { id: 'S2', name: '挥手 demo', description: 'three poses', rev: 1, steps: [{ type: 'pose', pose_id: P3, speed_scale: 0.8, gripper: 'keep' }] },
];
const DSETS = [{ name: 'default', episodes: 1, duration_s: 12.4, bytes: 210000 }, { name: 'cube_bin', episodes: 2, duration_s: 35.1, bytes: 610000 }];
const send = (res, code, o) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const srv = http.createServer((req, res) => {
  const u = req.url.split('?')[0];
  if (u === '/' || u === '/index.html') { res.writeHead(200, { 'content-type': 'text/html; charset=utf-8' }); res.end(html); return; }
  if (u === '/assets/calib_wizard.js') { res.writeHead(200, { 'content-type': 'application/javascript; charset=utf-8' }); res.end(js); return; }
  if (u === '/api/v1/sequences') return send(res, 200, { sequences: SEQS });
  if (u === '/api/v1/poses') return send(res, 200, { poses: POSES });
  if (u === '/api/v1/episodes') return send(res, 200, { episodes: EPS, recording: null });
  if (u === '/api/v1/datasets') return send(res, 200, { datasets: DSETS });
  res.writeHead(404); res.end();
}).listen(0, '127.0.0.1');
await new Promise((r) => srv.once('listening', r));
const url = `http://127.0.0.1:${srv.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME || undefined });
let bad = 0;
const shots = [];
const fail = (m) => { bad++; console.log('  FAIL ' + m); };

// page state: this page holds the lease, torque on, idle; teach v1 when `teach`
function setup(teach) {
  S.dof = 7; S.armDof = 6; S.state = 'Armed';
  S.caps = { ...(S.caps || {}), poses: true, replay: true, collision_guard: false };
  if (teach) S.caps.teach = { version: 1, episodes: true, sequences: true }; else delete S.caps.teach;
  S.leaseGeneration = 1; S.epoch = 1;
  S.lastIntentAt = Date.now() + 1e9;
  window.__sent = []; window.__cmd = []; window.__tc = []; window.__estop = 0;
  S.ws = { readyState: 1, send(m) { window.__sent.push(JSON.parse(m)); }, close() {}, addEventListener() {} };
  S.obs = { positions: [0.02, -0.4, 0.8, 0.01, -0.03, 0.1, 0.02], velocities: [0, 0, 0, 0, 0, 0, 0], is_torqued: true, enable_state: 'Enabled', safety_state: 'Armed', epoch_id: 1,
    control_mode: 'position', fault: null, action: null, previous_action: null, timestamp: 1000,
    gravity_comp: { supported: true, phase: null, blocker: null } };
  clearInterval(window.__tick); window.__tick = setInterval(() => { S.obsAt = Date.now(); }, 50);
  // armd replies (lease commands / lease-free commands)
  window.__reply = {};
  window.commandRPC = async (p, body) => {
    window.__cmd.push([p, JSON.parse(JSON.stringify(body || {}))]);
    const r = window.__reply[p];
    if (typeof r === 'function') return r(body);
    if (r && r.err) throw Object.assign(new Error(r.err.message || r.err.code), { code: r.err.code, reply: r.err });
    return r || { epoch_id: 1 };
  };
  window.__realTeachCmd = window.__realTeachCmd || window.teachCmd;
  window.teachCmd = async (p, body) => {
    window.__tc.push([p, JSON.parse(JSON.stringify(body || {}))]);
    const r = window.__reply[p];
    if (typeof r === 'function') return r(body);
    if (r && r.err) throw Object.assign(new Error(r.err.message || r.err.code), { code: r.err.code, reply: r.err });
    return r || {};
  };
  window.eStop = () => { window.__estop++; };
  window.obsTick = () => { teachOnObs(S.obs); renderGc(); };
  teachOnCaps(); renderGc();
}

for (const [vpName, viewport] of [['desktop', { width: 1400, height: 900 }], ['phone', { width: 390, height: 844 }]]) {
  for (const lang of ['zh', 'en']) {
    const ctx = await browser.newContext({ viewport, deviceScaleFactor: vpName === 'phone' ? 2 : 1 });
    await ctx.addInitScript(([l]) => {
      try { localStorage.clear(); localStorage.setItem('lang', l); } catch (_) {}
      const RealWS = window.WebSocket;
      window.WebSocket = function (u, p) { if (!String(u).includes('/api/v1/ws')) return new RealWS(u, p); return { readyState: 1, url: u, send() {}, close() {}, addEventListener() {} }; };
      window.WebSocket.OPEN = 1;
    }, [lang]);
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(e.message));
    await page.goto(url, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(500);
    const ev = (fn, arg) => page.evaluate(fn, arg);
    const tag = `${lang}/${vpName}`;
    const shot = async (name, panel, note = '') => {
      await page.waitForTimeout(250);
      const info = await ev((id) => {
        const el = document.getElementById(id);
        return { open: !!el && !el.closest('[hidden]'), raw: el ? /\btch\.[a-z]/.test(el.innerText) : true,
          hscroll: el ? el.scrollWidth > el.clientWidth + 1 : true, page: document.documentElement.scrollWidth > window.innerWidth + 1 };
      }, panel);
      if (!info.open || info.raw || info.hscroll || info.page) fail(`${name} ${tag}: ${JSON.stringify(info)}`);
      const file = path.join(out, `teach_${name}_${lang}_${vpName}.png`);
      await page.screenshot({ path: file });
      shots.push(file);
      console.log(path.relative(root, file) + (note ? '  ' + note : ''));
    };

    // 0 without capabilities.teach: no entry; the hand-guide record button keeps the legacy dataset recording
    await ev(setup, false);
    await ev(() => { S.obs.control_mode = 'gravity_comp'; S.obs.gravity_comp.phase = 'active'; renderGc(); window.__legacy = 0; $('btn-rec').addEventListener('click', () => { window.__legacy++; }); ssetOpen(true); });
    const g0 = await ev(() => ({ set: !$('teach-set').hidden, panel: !$('teach').hidden }));
    await ev(() => { ssetOpen(false); $('btn-gc-rec').click(); });
    const g0b = await ev(() => ({ legacy: window.__legacy, tc: window.__tc.length, panel: !$('teach').hidden }));
    if (g0.set || g0.panel || g0b.legacy !== 1 || g0b.tc || g0b.panel) fail(`gating without teach ${tag}: ${JSON.stringify({ ...g0, ...g0b })}`);
    await ev(() => { S.obs.control_mode = 'position'; S.obs.gravity_comp.phase = null; renderGc(); });

    // 1 teach v1: the entry next to the Tools group
    await ev(setup, true);
    // with teach v1 the hand-guide record button records into the teach store and opens the panel collapsed
    await ev(() => {
      window.__reply['/api/v1/record/stop'] = { episode: { id: '01JD0000000000000000EPIS08', dataset: 'default', task: '', source: 'hand_guide', duration_s: 2, frames: 600 } };
      S.obs.control_mode = 'gravity_comp'; S.obs.gravity_comp.phase = 'active'; renderGc(); window.__legacy = 0; $('btn-gc-rec').click();
    });
    await page.waitForTimeout(200);
    const g1 = await ev(() => ({ open: TCH.open, teach: TCH.teach, rec: !!TCH.rec, gcrec: $('btn-gc-rec').classList.contains('rec-on'), bar: !$('teach-bar').hidden }));
    await shot('0_handguide_record_shortcut', 'teach-bar');
    await ev(() => $('btn-gc-rec').click());
    await page.waitForTimeout(200);
    const g2 = await ev(() => ({ tc: window.__tc.map(([p, b]) => p + ':' + (b.source || b.save)), legacy: window.__legacy, rec: !!TCH.rec }));
    if (g1.open || !g1.teach || !g1.rec || !g1.gcrec || !g1.bar || g2.legacy || g2.rec || JSON.stringify(g2.tc) !== '["/api/v1/record/start:hand_guide","/api/v1/record/stop:true"]')
      fail(`hand-guide record shortcut ${tag}: ${JSON.stringify({ g1, g2 })}`);
    await ev(() => { teachOpen(false); teachStrip(false); S.obs.control_mode = 'position'; S.obs.gravity_comp.phase = null; renderGc(); window.__tc = []; });
    await ev(() => ssetOpen(true));
    if (!(await ev(() => !$('teach-set').hidden && $('teach-set').nextElementSibling.id === 'tools-set'))) fail(`entry ${tag}`);
    await shot('0_settings_entry', 'sset');
    await ev(() => $('teach-open').click());
    await page.waitForTimeout(400);
    const l1 = await ev(() => ({ open: !$('teach').hidden, sset: $('sset').hidden, items: [...document.querySelectorAll('#tch-seqs > li')].map((li) => li.dataset.id),
      stale: !!document.querySelector('#tch-seqs > li[data-id="S1"] .tch-bd.stale') }));
    if (!l1.open || !l1.sset || JSON.stringify(l1.items) !== '["S1","S2"]' || !l1.stale) fail(`sequence list ${tag}: ${JSON.stringify(l1)}`);
    await shot('1_sequences', 'teach-panel');

    // 2 editor: steps, stale badge on the stale pose step, reorder, speed change marks dirty
    await ev(() => teachEdit('S1'));
    const e1 = await ev(() => ({ steps: document.querySelectorAll('#tch-steps > li').length, stale: [...document.querySelectorAll('#tch-steps > li')].map((li) => !!li.querySelector('.tch-bd.stale')) }));
    if (e1.steps !== 6 || JSON.stringify(e1.stale) !== '[false,false,false,true,false,false]') fail(`editor ${tag}: ${JSON.stringify(e1)}`);
    await ev(() => { document.querySelector('#tch-steps > li[data-i="2"] .tch-acts button').click(); }); // wait step up
    await ev(() => { const s = document.querySelector('#tch-steps > li[data-i="0"] select'); s.value = '0.3'; s.dispatchEvent(new Event('change')); });
    const e2 = await ev(() => ({ order: TCH.cur.steps.map((s) => s.type).join(','), speed: TCH.cur.steps[0].speed_scale, dirty: TCH.dirty, save: $('tch-save').textContent }));
    if (e2.order !== 'pose,wait,gripper,pose,gripper,replay' || e2.speed !== 0.3 || !e2.dirty) fail(`edit ${tag}: ${JSON.stringify(e2)}`);
    // gripper steps: an opening slider (open = 100 %, close = 0 %); moving it makes a move step; "+ Gripper opening" adds the measured opening
    const gs = await ev(() => {
      const li = [...document.querySelectorAll('#tch-steps > li')].filter((x) => TCH.cur.steps[+x.dataset.i].type === 'gripper');
      const vals = li.map((x) => x.querySelector('input[type=range]') && x.querySelector('input[type=range]').value);
      const sel = li.some((x) => x.querySelector('select'));
      const rg = li[1].querySelector('input[type=range]'); rg.value = '0.4'; rg.dispatchEvent(new Event('input'));
      const st = TCH.cur.steps[+li[1].dataset.i];
      return { vals, sel, st: { action: st.action, fraction: st.fraction }, title: li[1].querySelector('b').textContent, out: li[1].querySelector('output').textContent };
    });
    if (JSON.stringify(gs.vals) !== '["1","0"]' || gs.sel || gs.st.action !== 'move' || gs.st.fraction !== 0.4 || !gs.title.includes('40%') || gs.out !== '40%') fail(`gripper slider ${tag}: ${JSON.stringify(gs)}`);
    await shot('2_editor', 'teach-panel', `order ${e2.order}, gripper ${gs.vals.join('/')} -> ${gs.st.action} ${gs.st.fraction}`);

    // 3 run with repeat 3 and recording: saves first (rev), then run_sequence; live step highlight; Stop
    await ev(() => {
      window.__reply['/api/v1/sequences/save'] = (b) => { window.__seqSaved = JSON.parse(JSON.stringify({ ...b, id: 'S1', rev: 4 })); return { sequence: { ...b, id: 'S1', rev: 4 }, epoch_id: 1 }; };
      window.__reply['/api/v1/action/run_sequence'] = { action_id: 'A1', expected_duration_s: 40, epoch_id: 1 };
      const rep = $('tch-repeat'); rep.value = '3'; rep.dispatchEvent(new Event('change'));
      $('tch-runrec').click();
    });
    await ev(() => { const ds = document.querySelector('#teach-body .tch-box:last-of-type select'); ds.value = 'cube_bin'; ds.dispatchEvent(new Event('change')); });
    await ev(() => $('tch-run').click());
    await page.waitForTimeout(300);
    const r1 = await ev(() => window.__cmd.map(([p, b]) => [p, b]));
    const save = r1.find(([p]) => p === '/api/v1/sequences/save'), run = r1.find(([p]) => p === '/api/v1/action/run_sequence');
    if (!save || save[1].rev !== 3 || save[1].id !== 'S1' || !run || run[1].repeat !== 3 || run[1].from_step !== 0 || run[1].single_step !== false
      || run[1].allow_stale !== false || !run[1].record || run[1].record.dataset !== 'cube_bin' || run[1].record.task !== 'pick_place') fail(`run body ${tag}: ${JSON.stringify(r1)}`);
    await ev(() => { S.obs.action = { id: 'A1', kind: 'sequence', state: 'running', step_index: 3, step_type: 'pose', repeat_index: 1, repeats: 3, phase: 'move' }; })
    await ev(() => obsTick());
    const r2 = await ev(() => ({ run: [...document.querySelectorAll('#tch-steps > li.run')].map((li) => li.dataset.i), prog: $('tch-prog').textContent, stop: !$('tch-stop').hidden, runBtn: $('tch-run').disabled }));
    if (JSON.stringify(r2.run) !== '["3"]' || !r2.prog.includes('4/6') || !r2.prog.includes('2/3') || !r2.stop || !r2.runBtn) fail(`live step ${tag}: ${JSON.stringify(r2)}`);
    await ev(() => { document.querySelector('#tch-steps > li[data-i="3"]').scrollIntoView({ block: 'center' }); });
    await shot('3_running', 'teach-panel', `highlight step ${+r2.run[0] + 1}, "${r2.prog}"`);
    await ev(() => { window.__stops = 0; const realApi = window.__realApi || (window.__realApi = window.api); window.api = async (p, ...a) => { if (p === '/api/v1/motion/stop') { window.__stops++; return {}; } return realApi(p, ...a); }; $('tch-stop').click(); });
    // the parent action ends with STOPPED
    await ev(() => { S.obs.previous_action = { id: 'A1', kind: 'sequence', state: 'cancelled', code: 'STOPPED', message: 'stopped' }; S.obs.action = null; });
    await ev(() => obsTick());
    const r3 = await ev(() => ({ stops: window.__stops, st: $('teach-status').textContent, run: TCH.run, hl: document.querySelectorAll('#tch-steps > li.run').length }));
    if (r3.stops !== 1 || !r3.st.includes('STOPPED') || r3.run || r3.hl) fail(`stop / end ${tag}: ${JSON.stringify(r3)}`);

    // 4 DATA_STALE: confirm in the panel, then run_sequence with allow_stale
    await ev(() => {
      window.__cmd = [];
      window.__reply['/api/v1/action/run_sequence'] = (b) => { if (!b.allow_stale) throw Object.assign(new Error('pose bin_a is stale'), { code: 'DATA_STALE' }); return { action_id: 'A2', epoch_id: 1 }; };
      $('tch-run').click();
    });
    await page.waitForTimeout(300);
    const s1 = await ev(() => ({ ask: !$('teach-ask').hidden, txt: $('teach-ask-t').textContent }));
    if (!s1.ask || !s1.txt) fail(`stale ask ${tag}: ${JSON.stringify(s1)}`);
    await shot('4_stale_confirm', 'teach-panel');
    await ev(() => $('teach-ask-yes').click());
    await page.waitForTimeout(300);
    const s2 = await ev(() => ({ calls: window.__cmd.filter(([p]) => p === '/api/v1/action/run_sequence').map(([, b]) => b.allow_stale), ask: $('teach-ask').hidden, run: TCH.run && TCH.run.actionId }));
    if (JSON.stringify(s2.calls) !== '[false,true]' || !s2.ask || s2.run !== 'A2') fail(`allow_stale ${tag}: ${JSON.stringify(s2)}`);
    await ev(() => { S.obs.previous_action = { id: 'A2', kind: 'sequence', state: 'done' }; obsTick(); });

    // 5 teach strip while hand-guiding: Save pose / Record append steps after the selected one and save the sequence
    await ev(() => {
      window.__cmd = []; window.__tc = [];
      S.obs.control_mode = 'gravity_comp'; S.obs.gravity_comp.phase = 'active';
      document.querySelector('#tch-steps > li[data-i="5"]').click(); // select the last step
    });
    await ev(() => obsTick());
    const b0 = await ev(() => ({ bar: !$('teach-bar').hidden, inStage: $('teach-bar').parentElement.classList.contains('stage'), gripBtns: !!document.querySelector('#teach-bar #tb-open, #teach-bar #tb-close') }));
    if (!b0.bar || !b0.inStage || b0.gripBtns) fail(`strip ${tag}: ${JSON.stringify(b0)}`);
    await ev(() => {
      window.__reply['/api/v1/record/start'] = { recording_id: 'R1', owner: CLIENT_ID };
      window.__reply['/api/v1/record/stop'] = { episode: { id: '01JD0000000000000000EPIS09', dataset: 'default', task: 'wave', source: 'hand_guide', duration_s: 6.2, frames: 1860, recorded_at: '2026-11-19T10:00:00Z' } };
    });
    // poses/save: the next GET /api/v1/poses is served by the page mock below
    await ev(() => {
      window.POSESX = []; window.EPSX = []; window.__epsLag = true;
      const realGet = window.teachGet;
      window.teachGet = async (p) => { const d = await realGet(p); if (p === '/api/v1/sequences' && window.__seqSaved) d.sequences = d.sequences.map((x) => (x.id === window.__seqSaved.id ? window.__seqSaved : x)); if (p === '/api/v1/poses') d.poses = d.poses.concat(window.POSESX); if (p.startsWith('/api/v1/episodes') && !window.__epsLag) d.episodes = d.episodes.concat(window.EPSX); return d; };
      window.__reply['/api/v1/poses/save'] = (b) => { window.POSESX.push({ id: '01JD0000000000000000POSE09', name: b.name, positions: [] }); return { epoch_id: 1 }; };
      $('tb-pose').click();
    });
    await page.waitForTimeout(400);
    await ev(() => $('tb-rec').click());
    await page.waitForTimeout(200);
    const b1 = await ev(() => ({ rec: $('tb-rec').classList.contains('on'), chip: document.body.classList.contains('recording'), gcrec: $('btn-gc-rec').textContent }));
    await shot('5_bar_handguide_recording', 'teach-panel', `rec ${b1.rec}`);
    await ev(() => $('tb-rec').click());
    await page.waitForTimeout(400);
    // the appended replay step shows the episode (task, duration) although the list reply right after the save lags
    const rp = await ev(() => { const li = [...document.querySelectorAll('#tch-steps > li')].find((x) => TCH.cur.steps[+x.dataset.i].episode_id === '01JD0000000000000000EPIS09');
      return li ? { title: li.querySelector('b').textContent, miss: !!li.querySelector('.tch-bd.miss') } : null; });
    if (!rp || !rp.title.includes('wave') || !rp.title.includes('6.2 s') || rp.miss) fail(`new replay step ${tag}: ${JSON.stringify(rp)}`);
    console.log(`  appended replay step: "${rp && rp.title}", missing badge ${rp && rp.miss}`);
    // a list requested after the save that does list it keeps it; the store then serves it normally
    await ev(() => { window.EPSX = [{ id: '01JD0000000000000000EPIS09', dataset: 'default', task: 'wave', source: 'hand_guide', duration_s: 6.2, frames: 1860, recorded_at: '2026-11-19T10:00:00Z' }]; window.__epsLag = false; return teachLoad(true); });
    const rp2 = await ev(() => ({ acked: TCH.acked.size, miss: !!document.querySelector('#tch-steps .tch-bd.miss') }));
    if (rp2.acked || rp2.miss) fail(`replay step after the list ${tag}: ${JSON.stringify(rp2)}`);
    await page.waitForTimeout(200);
    const b2 = await ev(() => ({ steps: TCH.cur.steps.map((s) => s.type + (s.pose_id ? ':' + s.pose_id.slice(-6) : '') + (s.episode_id ? ':' + s.episode_id.slice(-6) : '') + (s.action ? ':' + s.action : '')),
      grip: (TCH.cur.steps.find((s) => s.pose_id === '01JD0000000000000000POSE09') || {}).gripper,
      cmd: window.__cmd.map(([p, b]) => p + (b.name ? ':' + b.name : '') + (b.command ? ':' + b.command : '')), tc: window.__tc.map(([p, b]) => p + ':' + JSON.stringify(b)), rec: TCH.rec, chip: document.body.classList.contains('recording'),
      dirty: TCH.dirty, seq: $('tb-seq').textContent }));
    const tail = b2.steps.slice(-3).join(',');
    // 记点 snapshots the opening too (gripper "pose"); each strip step is saved at once (2 x sequences/save); the strip sends no gripper command
    if (!b1.rec || !b1.chip || tail !== 'replay:EPIS01,pose:POSE09,replay:EPIS09' || b2.grip !== 'pose' || b2.rec || b2.chip || b2.dirty
      || !b2.cmd.includes('/api/v1/poses/save:p01') || b2.cmd.some((c) => c.startsWith('/api/v1/action/gripper')) || b2.cmd.filter((c) => c.startsWith('/api/v1/sequences/save')).length !== 2
      || !b2.tc.some((x) => x.startsWith('/api/v1/record/start:') && x.includes('"source":"hand_guide"') && x.includes('"dataset":"default"'))
      || !b2.tc.some((x) => x === '/api/v1/record/stop:{"save":true}')) fail(`teach bar ${tag}: ${JSON.stringify({ b1, b2 })}`);
    await shot('6_bar_steps_added', 'teach-panel', `steps ${b2.steps.length}: ...${tail}`);

    // 6 panel closed, teaching goes on: the strip (<= 48 px) overlaps no control (sticks, gripper slider, E-stop, Enable,
    // hand-guide switch, view switch, HUD, safety card, top bar) and, where the video has its own box, not the video;
    // in the simple and the expert layout. "Edit" opens the panel again.
    await ev(() => $('teach-close').click());
    await page.waitForTimeout(300);
    const geoOf = (ui) => ev(async (ui) => {
      if (S.ui !== ui) setUi(ui);
      await new Promise((r) => setTimeout(r, 250)); TCH.dockAt = 0; teachBarRender(); await new Promise((r) => setTimeout(r, 50));
      const bar = $('teach-bar'), b = bar.getBoundingClientRect(), v = $('video-box').getBoundingClientRect(), H = innerHeight, W = innerWidth;
      const fixed = getComputedStyle(bar).position === 'fixed';
      const sel = TCH_STRIP_AVOID + (fixed ? '' : ', #video-box');
      const els = [...document.querySelectorAll(sel)].filter((e) => !bar.contains(e) && !e.contains(bar));
      const rects = els.map((e) => [e.id || e.className, e.getBoundingClientRect()]).filter(([, r]) => r.width > 0 && r.height > 0 && r.bottom > 0 && r.top < H);
      const cover = rects.filter(([, r]) => r.left < b.right - 0.5 && r.right > b.left + 0.5 && r.top < b.bottom - 0.5 && r.bottom > b.top + 0.5).map(([n]) => n);
      return { ui, fixed, shown: !bar.hidden, panel: !$('teach').hidden, h: Math.round(b.height), y: Math.round(b.top), inView: b.left >= 0 && b.right <= W + 0.5 && b.top >= 0, checked: rects.length, cover,
        seq: $('tb-seq').textContent, pose: !!$('tb-pose').offsetWidth, rec: !!$('tb-rec').offsetWidth };
    }, ui);
    for (const ui of ['simple', 'expert']) {
      const geo = await geoOf(ui);
      if (!geo.shown || geo.panel || geo.h > 48 || !geo.inView || geo.cover.length || geo.checked < 4 || !geo.pose || !geo.rec || !geo.seq.includes('pick_place')) fail(`strip geometry ${tag}: ${JSON.stringify(geo)}`);
      await shot('7_strip_' + ui, 'teach-bar', `strip y ${geo.y} h ${geo.h} ${geo.fixed ? 'docked' : 'in flow'}, ${geo.checked} controls checked, covers ${geo.cover.length} "${geo.seq}"`);
    }
    await ev(() => setUi('simple'));
    await page.waitForTimeout(200);
    await ev(() => $('tb-edit').click());
    if (!(await ev(() => TCH.open && !$('teach').hidden))) fail(`edit from the strip ${tag}`);
    await ev(() => { S.obs.control_mode = 'position'; S.obs.gravity_comp.phase = null; obsTick(); });

    // 7 poses: used by, rename, delete refused with IN_USE
    await ev(() => { TCH.dirty = false; TCH.cur = null; $('teach-tab-pose').click(); });
    await page.waitForTimeout(300);
    const p1 = await ev(() => ({ rows: [...document.querySelectorAll('#tch-poses > li')].map((li) => li.querySelector('b').textContent + '|' + li.querySelector('.tch-sub').textContent + '|' + !!li.querySelector('.tch-bd.stale')) }));
    if (!p1.rows.length || !p1.rows[0].startsWith('scan|') || !p1.rows[0].includes('pick_place') || !p1.rows[1].endsWith('|true') || !p1.rows[2].includes('挥手 demo')) fail(`poses ${tag}: ${JSON.stringify(p1)}`);
    await ev(() => {
      window.__reply['/api/v1/poses/delete'] = { err: { code: 'IN_USE', message: 'pose is referenced', sequences: ['S1'] } };
      const del = () => [...document.querySelectorAll('#tch-poses > li')][0].querySelector('.tch-del');
      del().click(); del().click();
    });
    await page.waitForTimeout(300);
    const p2 = await ev(() => ({ st: $('teach-status').textContent, bad: $('teach-status').classList.contains('bad'), del: window.__cmd.filter(([p]) => p === '/api/v1/poses/delete').map(([, b]) => b.name) }));
    if (!p2.st.includes('pick_place') || !p2.bad || JSON.stringify(p2.del) !== '["scan"]') fail(`pose IN_USE ${tag}: ${JSON.stringify(p2)}`);
    await shot('8_poses_in_use', 'teach-panel', `"${p2.st}"`);
    await ev(() => { [...document.querySelectorAll('#tch-poses > li')][2].querySelectorAll('.tch-acts .btn')[1].click(); });
    await ev(() => { window.__reply['/api/v1/poses/rename'] = { epoch_id: 1 }; $('tch-rename').value = 'bin_c'; $('tch-rename').nextElementSibling.click(); });
    await page.waitForTimeout(300);
    const p3 = await ev(() => window.__cmd.filter(([p]) => p === '/api/v1/poses/rename').map(([, b]) => b));
    if (p3.length !== 1 || p3[0].name !== 'bin_c' || p3[0].id !== '01JD0000000000000000POSE03') fail(`rename ${tag}: ${JSON.stringify(p3)}`);

    // 8 episodes grouped by dataset; badges; server strings as text; edit task + dataset (lease-free update)
    await ev(() => $('teach-tab-ep').click());
    await page.waitForTimeout(300);
    const x1 = await ev(() => ({ groups: [...document.querySelectorAll('#teach-body ol[data-ds]')].map((o) => o.dataset.ds + ':' + o.querySelectorAll('li[data-id]').length),
      xss: window.__xss === 1 || !!document.querySelector('#teach-body img'), txt: document.querySelector('#teach-body li[data-id$="EPIS02"] b').textContent,
      badges: [...document.querySelectorAll('#teach-body li[data-id] .tch-li-h')].map((h) => [...h.querySelectorAll('.tch-bd')].map((b) => b.className.replace('tch-bd', '').trim() || 'src').join('+')) }));
    if (JSON.stringify(x1.groups) !== '["default:2","cube_bin:2"]' || x1.xss || !x1.txt.startsWith('<img') || JSON.stringify(x1.badges) !== '["src","src","src+stale","src+ab"]')
      fail(`episodes ${tag}: ${JSON.stringify(x1)}`);
    await shot('9_episodes', 'teach-panel', `groups ${x1.groups.join(', ')}`);
    await ev(() => { document.querySelector('#teach-body li[data-id$="EPIS03"] .tch-acts .btn:nth-child(2)').click(); });
    await ev(() => { $('tch-ep-task').value = 'stack cubes'; const d = $('tch-ep-ds'); d.value = 'default'; });
    await shot('10_episode_edit', 'teach-panel');
    await ev(() => { $('tch-ep-ds').nextElementSibling.click(); });
    await page.waitForTimeout(300);
    const x2 = await ev(() => window.__tc.filter(([p]) => p === '/api/v1/episodes/update').map(([, b]) => b));
    if (x2.length !== 1 || x2[0].task !== 'stack cubes' || x2[0].dataset !== 'default' || x2[0].id !== '01JD0000000000000000EPIS03') fail(`episode update ${tag}: ${JSON.stringify(x2)}`);
    // replay of the stale episode: DATA_STALE -> confirm -> allow_stale
    await ev(() => {
      window.__reply['/api/v1/action/replay'] = (b) => { if (!b.allow_stale) throw Object.assign(new Error('recorded with the old zero'), { code: 'DATA_STALE' }); return { action_id: 'A3', epoch_id: 1 }; };
      document.querySelector('#teach-body li[data-id$="EPIS02"] .tch-acts .btn').click();
    });
    await page.waitForTimeout(300);
    if (await ev(() => $('teach-ask').hidden)) fail(`replay stale ask ${tag}`);
    await shot('11_replay_stale_confirm', 'teach-panel');
    await ev(() => $('teach-ask-yes').click());
    await page.waitForTimeout(300);
    const x3 = await ev(() => window.__cmd.filter(([p]) => p === '/api/v1/action/replay').map(([, b]) => [b.episode.slice(-6), b.allow_stale]));
    if (JSON.stringify(x3) !== '[["EPIS02",false],["EPIS02",true]]') fail(`replay allow_stale ${tag}: ${JSON.stringify(x3)}`);
    await ev(() => { S.obs.action = { id: 'A3', kind: 'replay', state: 'running', episode: '01JD0000000000000000EPIS02', progress: 0.4 }; obsTick(); });
    if (!(await ev(() => !!document.querySelector('#teach-body li[data-id$="EPIS02"].run')))) fail(`replay highlight ${tag}`);
    await ev(() => { S.obs.previous_action = { ...S.obs.action, state: 'done' }; S.obs.action = null; obsTick(); });

    // 9 the lease-free command goes over the WebSocket without a generation
    const w = await ev(async () => {
      const p = window.__realTeachCmd('/api/v1/datasets/create', { name: 'x' });
      const m = window.__sent[window.__sent.length - 1];
      const pend = pendingCommands.get(m.request_id); pend.resolve({ type: 'command_ack' }); clearTimeout(pend.timer); pendingCommands.delete(m.request_id);
      await p;
      return m;
    }).catch((e) => ({ err: String(e) }));
    if (w.type !== 'command' || w.path !== '/api/v1/datasets/create' || 'generation' in w || w.client_id == null) fail(`lease-free command ${tag}: ${JSON.stringify(w)}`);

    // 10 Space / Escape trigger the E-stop with focus anywhere in the panel
    await ev(() => { window.eStop = () => { window.__estop++; }; window.__estop = 0; document.querySelector('#teach-body .btn').focus(); });
    await page.keyboard.press('Space');
    await ev(() => $('tch-task').focus());
    await page.keyboard.press('Escape');
    await ev(() => $('teach-tab-seq').focus());
    await page.keyboard.press('Escape');
    const es = await ev(() => ({ n: window.__estop, open: !$('teach').hidden }));
    if (es.n !== 3) fail(`E-stop keys ${tag}: ${JSON.stringify(es)}`);
    console.log(`  E-stop from the panel: ${es.n}/3 key presses`);
    if (errs.length) fail(`page errors ${tag}: ${errs.join(' | ')}`);
    await ctx.close();
  }
}
await browser.close();
srv.close();
console.log(bad ? `${bad} checks failed` : `${shots.length} screenshots OK`);
process.exit(bad ? 1 : 0);
