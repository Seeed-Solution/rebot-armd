/* =================================================================
   Calibration tools (capabilities.calibration_tools, armd capabilities version 10): the Tools
   group of the settings drawer lists the supported tools in the recommended order (motor zero ->
   fine calibration -> gripper), shows a stale camera calibration (GET /api/v1/calibration
   `stale`, set by a motor zero) here and as a chip on the video, and releases this page's lease.
   ================================================================= */
const TL = { st: null, live: null, dismissed: null };
const toolsCaps = () => (S.caps && S.caps.calibration_tools) || null;
// [descriptor key, title key, description key, recommended step (0 = none), open]
const TOOL_ENTRIES = [
  ['motor_zero', 'tools.mz', 'tools.mz.desc', 1, () => { ssetOpen(false); mzOpen(true); }],
  ['camera', 'tools.cam', 'tools.cam.desc', 0, () => toolsWizard('camera')],
  ['kinematics', 'tools.kin', 'tools.kin.desc', 2, () => toolsWizard('fine')],
  ['gripper', 'tools.grip', 'tools.grip.desc', 3, () => { ssetOpen(false); gtOpen(true); }],
];
$('tools-zero-open').addEventListener('click', () => { ssetOpen(false); MZ.sel = new Set([6]); MZ.method = 'software'; mzOpen(true); });
const TOOL_OP = { motor_zero: '/api/v1/calibration/motor_zero', camera: 'calib.open', kinematics: 'calib.open', gripper: '/api/v1/gripper/calibration' };
const toolShown = (key) => { const c = toolsCaps(); return !!(c && c[key] && c[key].supported); };
function toolsAvail() { return TOOL_ENTRIES.some(([k]) => toolShown(k)); }
function toolsOnCaps() {
  $('tools-set').hidden = !toolsAvail();
  if (toolsAvail()) toolsLoadStatus();
  else { TL.st = null; renderCalStale(); }
}
async function toolsLoadStatus() {
  if (!toolsCaps()) return;
  try { TL.st = await api('/api/v1/calibration', undefined, { quiet: true, method: 'GET' }); } catch (_) { return; }
  renderCalStale();
  if (!$('sset').hidden) toolsRender();
}
const calStale = () => (TL.st && TL.st.stale) || null;
function staleText(st) { return t('tools.stale', { at: (st.at || '').replace('T', ' ').replace(/(:\d\d)Z$/, '$1 UTC') }); }
// cockpit banner; closing it hides it until the stale time changes
function renderCalStale() {
  const st = calStale(), el = $('cal-stale');
  el.hidden = !st || TL.dismissed === st.at;
  if (st) { setText($('cal-stale-open'), t('tools.stale.chip')); $('cal-stale-open').title = staleText(st) + (st.reason ? ' — ' + st.reason : ''); }
}
function toolsRender() {
  const list = $('tools-list'), c = toolsCaps();
  list.textContent = '';
  for (const [key, title, desc, n, open] of TOOL_ENTRIES) {
    if (!toolShown(key)) continue;
    const b = cwEl('button', 'tool-item'); b.type = 'button'; b.id = 'tool-' + key;
    b.dataset.op = TOOL_OP[key]; b.dataset.opMode = 'offer'; // each tool writes calibration data
    b.append(cwEl('span', 'tool-n' + (n ? '' : ' none'), n ? String(n) : '–'), cwEl('b', '', t(title)), cwEl('small', '', t(desc)));
    b.addEventListener('click', open);
    list.appendChild(b);
  }
  const st = calStale(), w = $('tools-stale');
  w.hidden = !st || !(c && c.camera && c.camera.supported);
  if (st) setText(w, staleText(st));
  // legacy J6 zero (armd's zero check against the profile's zero_reference); never moves the arm
  const zc = TL.st && TL.st.zero_convention, zw = $('tools-zero');
  zw.hidden = !(zc && zc.status === 'legacy_j6') || !toolShown('motor_zero');
  if (!zw.hidden) setText(zw.querySelector('span'), t('mz.legacy', { d: mzDeg(zc.j6_rad).replace('°', '') }));
  // poses / gripper calibration taught with the old zero
  const at = (x) => (x.at || '').replace('T', ' ').replace(/(:\d\d)Z$/, '$1 UTC');
  const lines = ((TL.st && TL.st.stale_list) || []).filter((x) => x.item !== 'camera').map((x) => x.item === 'poses'
    ? t('tools.stale.poses', { names: (x.names || []).join(', '), at: at(x) }) : t('tools.stale.grip', { at: at(x) }));
  const w2 = $('tools-stale2');
  w2.hidden = !lines.length;
  setText(w2, lines.join(' '));
  toolsRenderRelease();
}
// release_lease: only with torque off and nothing moving (armd parks a torqued arm on release)
function toolsReleaseState() {
  const o = S.obs || {};
  if (S.leaseGeneration == null || !S.ws || S.ws.readyState !== 1) return 'none';
  if (leaseOp) return 'busy';
  if (unloading || o.disable_pending || (o.action?.kind === 'park' && o.action?.state === 'running')) return 'parking';
  if (o.is_torqued || o.enable_state === 'Enabled') return 'torqued';
  if (o.action?.state === 'running' || S.state === 'Moving' || S.presetRunning || S.streamActive || pendingCommands.size
    || (Array.isArray(o.velocities) && o.velocities.some((v) => !(Math.abs(v) < 0.05)))) return 'moving';
  return 'ok';
}
function toolsRenderRelease() {
  const st = toolsReleaseState();
  $('tools-release').disabled = st !== 'ok';
  setText($('tools-release-hint'), t('tools.rel.' + st));
}
function toolsRelease() {
  if (toolsReleaseState() !== 'ok') { toolsRenderRelease(); return; }
  sendReleaseLease('toast.lease_released');
  toolsRenderRelease();
}
// the calibration wizard in a mode (camera / fine); the mode stays when views were captured
function toolsWizard(mode) {
  ssetOpen(false);
  if (CWZ.running) { toast(t('cw.busy'), 'warn'); return; }
  if (!CWZ.views.length) { CWZ.mode = mode; CWZ.modeChosen = true; }
  cwOpen(true);
}
$('tools-release').addEventListener('click', toolsRelease);
$('cal-stale-open').addEventListener('click', () => ssetOpen(true));
$('cal-stale-x').addEventListener('click', () => { TL.dismissed = calStale()?.at; renderCalStale(); });

/* ---------- gripper tool: GET /api/v1/gripper/calibration, action gripper_zero_check ---------- */
const GT = { cal: null, busy: false, armedUntil: 0, res: null, status: '' };
async function gtLoad() {
  try { GT.cal = await api('/api/v1/gripper/calibration', undefined, { quiet: true, method: 'GET' }); } catch (_) { GT.cal = null; }
  gtRender();
}
function gtOpen(open) {
  $('gtool').hidden = !open;
  if (open) { GT.status = ''; GT.res = null; gtRender(); gtLoad(); $('gtool-close').focus({ preventScroll: true }); }
}
function gtRender() {
  const body = $('gtool-body'), c = toolsCaps()?.gripper || {}, o = S.obs || {};
  body.textContent = '';
  body.appendChild(cwEl('p', '', t('gt.lead')));
  const box = cwEl('div', 'calib-result mono');
  const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); box.appendChild(d); };
  const off = GT.cal && Number.isFinite(GT.cal.offset_rad) ? GT.cal.offset_rad : o.gripper_zero?.offset_rad;
  row(t('gt.cur'), Number.isFinite(off) ? off.toFixed(4) + ' rad' : '--');
  row(t('gt.calibrated'), GT.cal ? t(GT.cal.calibrated ? 'gt.yes' : 'gt.no') : '--');
  if (o.gripper_zero?.state) row(t('gt.state'), o.gripper_zero.state);
  body.appendChild(box);
  if (!c.zero_check) { body.appendChild(cwEl('div', 'cwiz-msg', t('gt.noz'))); gbRender(body, o); return; }
  const torqued = !!o.is_torqued;
  if (!torqued) body.appendChild(cwEl('div', 'cwiz-msg', t('gt.torque')));
  const acts = cwEl('div', 'cwiz-acts');
  const armed = Date.now() < GT.armedUntil;
  acts.append(cwBtn(t('gt.check'), () => gtRun(false), { disabled: GT.busy || GB.busy || !torqued, id: 'gt-check' }),
    cwBtn(t(armed ? 'gt.store.confirm' : 'gt.store'), () => gtRun(true), { cls: armed ? 'btn-primary' : '', disabled: GT.busy || GB.busy || !torqued, id: 'gt-store', op: '/api/v1/gripper/calibration' }));
  body.appendChild(acts);
  if (GT.res) body.appendChild(cwEl('div', 'cwiz-msg' + (GT.res.pass ? ' ok' : ''), t('gt.res', { s: cwFmt(GT.res.stop_rad, 4), o: cwFmt(GT.res.offset_rad, 4), p: t(GT.res.pass ? 'gt.pass' : 'gt.fail') })));
  if (GT.status) body.appendChild(cwEl('div', 'cwiz-msg', GT.status));
  gbRender(body, o);
}
/* empty-gripper calibration: one armd action (/api/v1/action/gripper_calibrate) runs the strokes,
   checks each one (target reached, free-motion torque) and learns only when all pass; this page
   starts it and shows its phase and result. An operator refusal opens the unlock dialog and stops
   here: it is never retried after the unlock. Stop (drawer header) cancels the action. */
const GB = { busy: false, armedUntil: 0, status: '', ok: false };
function gbRender(body, o) {
  const b = o.gripper && o.gripper.baseline;
  if (!b) return; // armd without the learned baseline
  const sec = cwEl('div', 'calib-result mono');
  const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); sec.appendChild(d); };
  body.appendChild(cwEl('h4', '', t('gb.title')));
  body.appendChild(cwEl('p', '', t('gb.lead')));
  row(t('gb.state'), t('gb.st.' + b.state) + ' · ' + t('gb.moves', { c: b.close_moves, o: b.open_moves }));
  body.appendChild(sec);
  const legacyWhy = legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only');
  const torqued = !!o.is_torqued;
  if (legacyWhy) {
    const whyEl = cwEl('div', 'cwiz-msg', legacyWhy);
    whyEl.id = 'gb-owner-why';
    body.appendChild(whyEl);
  } else if (!torqued) {
    body.appendChild(cwEl('div', 'cwiz-msg', t('gb.torque')));
  }
  const armed = !legacyWhy && Date.now() < GB.armedUntil;
  const acts = cwEl('div', 'cwiz-acts');
  const runBtn = cwBtn(t(armed ? 'gb.confirm' : 'gb.run'), gbRun, { cls: armed ? 'btn-primary' : '', disabled: !!legacyWhy || GB.busy || GT.busy || !torqued, id: 'gb-run', op: '/api/v1/action/gripper_calibrate' });
  runBtn.dataset.route = '/api/v1/action/gripper_calibrate';
  runBtn.dataset.legacyOnlyKey = 'gb.legacy_only';
  if (legacyWhy) { runBtn.title = legacyWhy; runBtn.setAttribute('aria-disabled', 'true'); }
  acts.append(runBtn);
  body.appendChild(acts);
  if (GB.status) body.appendChild(cwEl('div', 'cwiz-msg' + (GB.ok ? ' ok' : ''), GB.status));
}
async function gbRun() {
  if (GB.busy || GT.busy) return;
  const legacyWhy = legacyOnlyRouteBlock('/api/v1/action/gripper_calibrate', 'gb.legacy_only');
  if (legacyWhy) { GB.ok = false; GB.status = legacyWhy; gtRender(); return; }
  if (Date.now() >= GB.armedUntil) { GB.armedUntil = Date.now() + 4000; gtRender(); setTimeout(gtRender, 4100); return; }
  GB.armedUntil = 0; GB.busy = true; GB.ok = false; GB.status = t('gb.start'); gtRender();
  try {
    const ack = await commandRPC('/api/v1/action/gripper_calibrate', {}, true, 5000);
    const id = ack && ack.action_id;
    const limit = Date.now() + 1000 * Math.max(60, (ack && ack.expected_duration_s) || 0) * 2;
    let a = null, shown = '';
    while (Date.now() < limit) {
      const o = S.obs || {};
      a = [o.action, o.previous_action].find((x) => x && x.id === id) || null;
      if (a && a.state !== 'running') break;
      const m = a && /^stroke:(\d+)\/(\d+)$/.exec(a.phase || '');
      const st = m ? t('gb.prog', { i: m[1], n: m[2] }) : t('gb.start');
      if (st !== shown) { shown = st; GB.status = st; gtRender(); }
      await cwSleep(200);
    }
    if (!a || a.state === 'running') throw Object.assign(new Error('timeout'), { code: 'TIMEOUT' });
    if (a.state !== 'done') throw Object.assign(new Error(a.message || a.state), { code: a.code || '' });
    const b = S.obs && S.obs.gripper && S.obs.gripper.baseline;
    GB.ok = !!b && b.state === 'learned';
    GB.status = t('gb.done', { st: b ? t('gb.st.' + b.state) : '--' });
  } catch (e) {
    if (isCancelledErr(e)) { GB.status = ''; return; }
    GB.status = e.code === 'OPERATOR_REQUIRED' ? t('gb.op')
      : e.code === 'EXECUTOR_ROUTING_PENDING' ? t('gb.legacy_only')
      : e.code === 'CALIBRATION_OBSTRUCTED' ? t('gb.obst', { m: e.message })
      : t('gb.err', { m: (e.code ? e.code + ': ' : '') + e.message });
  } finally { GB.busy = false; gtRender(); }
}
async function gtRun(store) {
  if (GT.busy || GB.busy) return;
  if (store && Date.now() >= GT.armedUntil) { GT.armedUntil = Date.now() + 4000; gtRender(); setTimeout(gtRender, 4100); return; }
  GT.armedUntil = 0; GT.busy = true; GT.res = null; GT.status = t('gt.running'); gtRender();
  try {
    const ack = await commandRPC('/api/v1/action/gripper_zero_check', { store_offset: store }, true, 5000);
    const id = ack && ack.action_id;
    const t0 = Date.now();
    let a = null;
    while (Date.now() - t0 < 15000) {
      const o = S.obs || {};
      a = [o.action, o.previous_action].find((x) => x && x.id === id && x.state !== 'running') || null;
      if (a) break;
      await cwSleep(150);
    }
    if (!a) throw Object.assign(new Error('timeout'), { code: 'TIMEOUT' });
    if (a.state !== 'done') throw Object.assign(new Error(a.message || a.state), { code: a.code || '' });
    GT.res = { stop_rad: a.stop_rad, offset_rad: a.offset_rad, pass: !!a.pass };
    GT.status = '';
    gtLoad();
  } catch (e) { GT.status = isCancelledErr(e) ? '' : t('gt.err', { m: (e.code ? e.code + ': ' : '') + e.message }); }
  finally { GT.busy = false; gtRender(); }
}
/* ---------- zero tool: WS /api/v1/calibration/motor_zero (src/motor_zero.rs, src/joint_zero.rs) ----------
   Torque off (park button), the 3D view shows the zero reference (the rest pose = the schema's
   home_positions, mapped by the model) as the ghost next to the current pose, the method (software
   zero in calibration.json, reversible / motor zero in the motor flash), one checkbox per joint with
   the live reading, force for joints > max_offset_deg from the reference, two taps (the first is
   armd's dry run), then the before / after table, the invalidated calibrations and, for a software
   zero, the undo (WS /api/v1/calibration/joint_zero/restore). */
const MZ = { open: false, sel: new Set(), force: false, armedUntil: 0, busy: false, res: null, err: null, status: '', previewQ: null, live: null, key: '', method: null };
const mzMethod = () => { const m = mzTool()?.methods || []; return m.includes(MZ.method) ? MZ.method : (m[0] || 'motor'); };
const mzRef = (j) => Number(restJoints()[j - 1]) || 0;
const mzTool = () => toolsCaps()?.motor_zero || null;
const mzName = (j) => (j === GRIP + 1 && S.dof === 7 ? t('mz.grip') + ' (J' + j + ')' : 'J' + j);
const mzDeg = (r) => (Number.isFinite(r) ? (r * 180 / Math.PI).toFixed(1) + '°' : '--');
function mzReading(j) { const p = S.obs && S.obs.positions; return Array.isArray(p) ? p[j - 1] : NaN; }
function mzFar(j) { const tl = mzTool(); return Math.abs(mzReading(j) - mzRef(j)) > ((tl?.max_offset_deg ?? 45) * Math.PI / 180); }
function mzOpen(open) {
  MZ.open = open;
  $('mzero').hidden = !open;
  clearInterval(MZ.live); MZ.live = null;
  if (open) {
    $('mzero-panel').classList.remove('min');
    MZ.res = null; MZ.err = null; MZ.status = ''; MZ.armedUntil = 0; MZ.force = false;
    MZ.previewQ = Array.from({ length: S.dof || 7 }, (_, i) => Number(restJoints()[i]) || 0);
    if (!V3.open) v3Toggle(true);
    mzRender();
    MZ.live = setInterval(mzLive, 300);
  } else { MZ.previewQ = null; }
  V3.key = '';
}
// live readings without rebuilding the table (checkbox focus stays); a state change re-renders
function mzLive() {
  const o = S.obs || {}, key = [!!o.is_torqued, MZ.busy, Date.now() < MZ.armedUntil].join();
  if (key !== MZ.key) { mzRender(); return; }
  for (const j of mzTool()?.joints || []) {
    const c = document.getElementById('mz-r-' + j);
    if (c) { setText(c, mzDeg(mzReading(j))); c.classList.toggle('far', mzFar(j)); }
  }
  mzRenderFar();
}
function mzSelFar() { return [...MZ.sel].filter(mzFar); }
function mzRenderFar() {
  const box = document.getElementById('mz-farbox');
  if (!box) return;
  const far = mzSelFar();
  box.hidden = !far.length;
  if (far.length) setText(box.querySelector('span'), t('mz.far', { d: mzTool()?.max_offset_deg ?? 45, list: far.map(mzName).join(', ') }) + ' ' + t('mz.force'));
  const go = $('mz-go');
  if (go) go.disabled = MZ.busy || !!(S.obs && S.obs.is_torqued) || !MZ.sel.size || (far.length > 0 && !MZ.force);
}
function mzRender() {
  const body = $('mzero-body'), tl = mzTool(), o = S.obs || {};
  MZ.key = [!!o.is_torqued, MZ.busy, Date.now() < MZ.armedUntil].join();
  body.textContent = '';
  if (!tl || !tl.supported) { body.appendChild(cwEl('div', 'cwiz-msg', tl?.reason || '--')); return; }
  const p = (txt, cls = '') => body.appendChild(cwEl('p', cls, txt));
  const acts = (...b) => { const d = cwEl('div', 'cwiz-acts'); b.filter(Boolean).forEach((x) => d.appendChild(x)); body.appendChild(d); };
  if (MZ.res) { mzRenderResult(body, p, acts); return; }
  p(t('mz.lead'));
  const zc = TL.st && TL.st.zero_convention;
  if (zc && zc.status === 'legacy_j6') body.appendChild(cwEl('div', 'cwiz-msg', t('mz.legacy', { d: mzDeg(zc.j6_rad).replace('°', '') })));
  if ((tl.methods || []).length > 1) {
    const box2 = cwEl('div', 'mz-methods');
    for (const m of tl.methods) {
      const lab = cwEl('label'), rb = document.createElement('input');
      rb.type = 'radio'; rb.name = 'mz-method'; rb.id = 'mz-m-' + m; rb.checked = mzMethod() === m; rb.disabled = MZ.busy;
      rb.addEventListener('change', () => { MZ.method = m; MZ.armedUntil = 0; mzRender(); });
      lab.append(rb, cwEl('span', '', t('mz.m.' + m)));
      box2.appendChild(lab);
    }
    body.appendChild(box2);
  }
  const zp = tl.zero_pose, box = cwEl('div', 'mz-pose');
  box.appendChild(cwEl('b', '', t('mz.pose')));
  if (zp && zp.description) box.appendChild(cwEl('span', '', zp.description));
  if (zp && (zp.joints?.length || zp.gripper)) {
    const ul = cwEl('ul');
    for (const x of [...(zp.joints || []), ...(zp.gripper ? [zp.gripper] : [])]) ul.appendChild(cwEl('li', '', x));
    box.appendChild(ul);
  }
  const lg = cwEl('div', 'mz-legend');
  for (const [k, c] of [['mz.legend.cur', '#3a3f3b'], ['mz.legend.zero', 'rgba(59,130,246,0.45)']]) { const s1 = cwEl('span', '', t(k)); const i = cwEl('i'); i.style.background = c; s1.prepend(i); lg.appendChild(s1); }
  box.appendChild(lg);
  body.appendChild(box);
  if (o.is_torqued) {
    body.appendChild(cwEl('div', 'cwiz-msg', t('mz.torque')));
    acts(cwBtn(t(MZ.busy ? 'mz.parking' : 'mz.park'), mzPark, { cls: 'btn-primary wide', disabled: MZ.busy, id: 'mz-park' }));
  } else body.appendChild(cwEl('div', 'cwiz-msg ok', t('mz.off')));
  // joints
  const rows = tl.joints.map((j) => {
    const tr = document.createElement('tr'), td = cwEl('td'), lab = cwEl('label'), cb = document.createElement('input');
    cb.type = 'checkbox'; cb.id = 'mz-c-' + j; cb.checked = MZ.sel.has(j); cb.disabled = MZ.busy;
    cb.addEventListener('change', () => { if (cb.checked) MZ.sel.add(j); else MZ.sel.delete(j); MZ.armedUntil = 0; mzRenderFar(); });
    lab.append(cb, cwEl('span', '', mzName(j))); td.appendChild(lab);
    const r = cwEl('td', mzFar(j) ? 'far' : '', mzDeg(mzReading(j))); r.id = 'mz-r-' + j;
    tr.append(td, r);
    return tr;
  });
  body.appendChild(cwTable([t('mz.th.joint'), t('mz.th.now')], rows)).classList.add('mz-table');
  acts(cwBtn(t('mz.all'), () => { tl.joints.forEach((j) => MZ.sel.add(j)); MZ.armedUntil = 0; mzRender(); }, { disabled: MZ.busy }),
    cwBtn(t('mz.none'), () => { MZ.sel.clear(); MZ.armedUntil = 0; mzRender(); }, { disabled: MZ.busy }));
  const fb = cwEl('label', 'mz-force'); fb.id = 'mz-farbox';
  const fc = document.createElement('input'); fc.type = 'checkbox'; fc.id = 'mz-force'; fc.checked = MZ.force; fc.disabled = MZ.busy;
  fc.addEventListener('change', () => { MZ.force = fc.checked; MZ.armedUntil = 0; mzRenderFar(); });
  fb.append(fc, cwEl('span'));
  body.appendChild(fb);
  const armed = Date.now() < MZ.armedUntil;
  const list = [...MZ.sel].sort((a, b) => a - b).map(mzName).join(', ');
  acts(cwBtn(armed ? t('mz.confirm', { list }) : t(MZ.busy ? 'mz.running' : 'mz.set'), mzGo, { cls: 'btn-primary wide' + (armed ? ' armed' : ''), id: 'mz-go', op: '/api/v1/calibration/motor_zero' }));
  mzRenderFar();
  if (MZ.status) body.appendChild(cwEl('div', 'cwiz-msg', MZ.status));
}
function mzRenderResult(body, p, acts) {
  const r = MZ.res, tl = mzTool() || {}, inv = r.invalidated || {};
  body.appendChild(cwEl('div', 'cwiz-msg' + (r.ok ? ' ok' : ''), t(r.ok ? 'mz.done' : 'mz.unverified')));
  const rows = (r.joints || []).map((x) => {
    const tr = document.createElement('tr');
    tr.append(cwEl('td', '', mzName(x.joint)), cwEl('td', '', mzDeg(x.before_rad)), cwEl('td', '', mzDeg(x.after_rad)), cwEl('td', x.verified ? 'ok' : 'bad', x.verified ? '✓' : '✗'));
    return tr;
  });
  body.appendChild(cwTable([t('mz.th.joint'), t('mz.th.before'), t('mz.th.after'), t('mz.th.ok')], rows));
  if (inv.kinematics_removed) body.appendChild(cwEl('div', 'cwiz-msg', t('mz.inv.kin')));
  if (inv.camera_stale) body.appendChild(cwEl('div', 'cwiz-msg', t('mz.inv.cam')));
  if (inv.poses_stale && inv.poses_stale.length) body.appendChild(cwEl('div', 'cwiz-msg', t('mz.inv.poses', { n: inv.poses_stale.length })));
  if (inv.gripper_calibration_stale) body.appendChild(cwEl('div', 'cwiz-msg', t('mz.inv.grip')));
  if (inv.error) body.appendChild(cwEl('div', 'cwiz-msg', t('mz.inv.err', { m: inv.error })));
  const pk = r.persist || tl.persist || '';
  p(t('mz.persist', { p: I18N.zh['mz.p.' + pk] !== undefined ? t('mz.p.' + pk) : (pk || '--'), v: (r.persist_verified ?? tl.persist_verified) === false ? t('mz.persist.unv') : '' }));
  const caps = toolsCaps() || {};
  acts(caps.kinematics?.supported ? cwBtn(t('mz.open.kin'), () => { mzOpen(false); toolsWizard('fine'); }, { cls: 'btn-primary', id: 'mz-open-kin' }) : null,
    caps.camera?.supported ? cwBtn(t('mz.open.cam'), () => { mzOpen(false); toolsWizard('camera'); }, { id: 'mz-open-cam' }) : null,
    r.restore && r.restore.backup ? cwBtn(t('mz.undo'), () => mzUndo(r.restore.backup), { id: 'mz-undo', disabled: MZ.busy }) : null,
    cwBtn(t('mz.again'), () => { MZ.res = null; MZ.sel.clear(); mzRender(); }, { id: 'mz-again' }),
    cwBtn(t('mz.close'), () => mzOpen(false), { id: 'mz-close' }));
}
const mzErrText = (e) => { const k = 'mz.code.' + e.code; return t('mz.err', { m: (I18N.zh[k] !== undefined ? t(k) : e.message) + (e.code ? ' (' + e.code + ')' : '') }); };
async function mzPark() {
  if (MZ.busy) return;
  MZ.busy = true; MZ.status = ''; mzRender();
  try {
    const ack = await commandRPC('/api/v1/motion/unload', {}, true, 5000);
    const id = ack && ack.action_id;
    await cwGuardedWaitPlain(() => {
      const o = S.obs || {};
      if (id != null) for (const a of [o.action, o.previous_action]) if (a && a.id === id && a.state !== 'running') return true;
      return !o.is_torqued;
    }, 60000);
  } catch (e) { MZ.status = isCancelledErr(e) ? '' : mzErrText(e); }
  finally { MZ.busy = false; mzRender(); }
}
// first tap: armd's dry run (checks, readings); second tap within 5 s: the zero
async function mzGo() {
  if (MZ.busy) return;
  const joints = [...MZ.sel].sort((a, b) => a - b);
  if (!joints.length) { MZ.status = t('mz.pick'); mzRender(); return; }
  const armed = Date.now() < MZ.armedUntil;
  MZ.busy = true; MZ.status = armed ? t('mz.running') : ''; mzRender();
  try {
    if (!armed) {
      await commandRPC('/api/v1/calibration/motor_zero', { joints, dry_run: true, force: MZ.force, method: mzMethod() }, true, 3000);
      MZ.armedUntil = Date.now() + 5000;
      setTimeout(() => { if (MZ.open && !MZ.busy) mzRender(); }, 5100);
    } else {
      MZ.armedUntil = 0;
      const r = await commandRPC('/api/v1/calibration/motor_zero', { joints, force: MZ.force, method: mzMethod() }, true, 10000);
      MZ.res = { ...r, ok: true };
      MZ.status = '';
      $('mzero-body').scrollTop = 0;
      mzAfter();
    }
  } catch (e) {
    MZ.armedUntil = 0;
    if (isCancelledErr(e)) { MZ.status = ''; return; }
    const rep = e.reply || {};
    if (e.code === 'MOTOR_ZERO_UNVERIFIED' || (rep.joints && rep.invalidated)) { MZ.res = { ...rep, ok: false }; mzAfter(); }
    MZ.status = mzErrText(e);
  } finally { MZ.busy = false; mzRender(); }
}
// the zero changed: the schema (kinematics_correction removed, joint_zero), the stale calibrations and poses
function mzAfter() { loadSchema(); toolsLoadStatus(); posesLoad(true); }
async function mzUndo(backup) {
  if (MZ.busy) return;
  MZ.busy = true; MZ.status = ''; mzRender();
  try {
    await commandRPC('/api/v1/calibration/joint_zero/restore', { backup }, true, 10000);
    MZ.res = null; MZ.status = t('mz.undone');
    mzAfter();
  } catch (e) { MZ.status = isCancelledErr(e) ? '' : mzErrText(e); }
  finally { MZ.busy = false; mzRender(); }
}
$('mzero-close').addEventListener('click', () => { if (!MZ.busy) mzOpen(false); });
$('mzero-min').addEventListener('click', () => $('mzero-panel').classList.toggle('min'));
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && MZ.open && !MZ.busy) mzOpen(false); });
i18nHooks.push(() => { if (MZ.open) mzRender(); });
$('gtool-close').addEventListener('click', () => { if (!GB.busy) gtOpen(false); });
$('gtool').addEventListener('click', (e) => { if (e.target === e.currentTarget) gtOpen(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('gtool').hidden && !GT.busy && !GB.busy) gtOpen(false); });

