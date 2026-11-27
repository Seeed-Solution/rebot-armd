/* =================================================================
   Tap-to-grasp (docs/agent_action_api.md, /api/v1/action/grasp): with Detect on, a tapped
   box is selected and tracked; "Grasp" sends the action; any control input, the stop
   button or the E-stop stops it. The setup drawer reads / writes the armd grasp setup.
   ================================================================= */
const GR = {
  sel: null,        // {label, score, box:[x0,y0,x1,y1], seenAt}
  id: null,         // action id of the grasp this page started
  sending: false,
  stopAt: 0,        // last stop request (ms)
  toasted: 0,       // last grasp id whose result was shown
  gripSynced: 0,    // last gripper / grasp action id S.grip was synced from
  pickGrip: false,  // next video tap records the grip pixel
  tiltEdited: false, // the user typed in the camera tilt field since it was last filled
  setup: null, path: null, supported: null, rev: null,
  camDown: null,    // GET /api/v1/grasp/setup camera_down: true = camera-down acquisition, grip pixel unused; null = unknown
};
// size measurement state (msRender, below); declared here because renderGrasp reads it
const MS = { state: null, sel: null, id: null, res: null, err: null, est: null, phase: '', progress: 0 };
const graspCodeKey = (code) => (/^IK_/.test(code || '') ? 'IK' : code);
function graspCodeText(code, message, detail) {
  if (code === 'WORKSPACE_VIOLATION') {
    const wsBox = formatWorkspaceViolation(detail);
    if (wsBox) return wsBox;
  }
  const k = 'grasp.code.' + graspCodeKey(code);
  const s = t(k);
  // a refused request: armd's message says which value (e.g. the box ray misses the table)
  if (s !== k && code === 'INVALID_COMMAND' && message) return s + ': ' + message;
  return s !== k ? s : (message || code || t('err.rejected'));
}
function graspRunningAction() {
  const a = S.obs && S.obs.action;
  return a && a.kind === 'grasp' && a.state === 'running' ? a : null;
}
const graspDetOn = () => (S.dof === 7 || isSoArmProfile()) && DET.avail && (DET.enabled || detHeld());
function graspRPC(path, body) {
  return commandRPC(path, body).then((d) => { syncEpoch(d.epoch_id); return d; }, (e) => {
    if (!isCancelledErr(e)) toast(e.code && e.code !== 'LEASE_REQUIRED' && e.code !== 'COMMAND_TIMEOUT' ? graspCodeText(e.code, e.message, e.reply) : e.message, 'err', e.code || '');
    throw e;
  });
}
// After a gripper or grasp action the page's gripper target must equal the commanded jaw
// position, or the next teleop frame would drive the jaw back to the old S.grip.
function graspSyncGrip(o) {
  const cmd = o.commanded_positions;
  if (!Array.isArray(cmd) || cmd.length <= GRIP) return;
  for (const a of [o.action, o.previous_action]) {
    if (!a || (a.kind !== 'gripper' && a.kind !== 'grasp')) continue;
    if (a.state === 'running' || GR.gripSynced !== a.id) {
      S.grip = gripFrac(cmd[GRIP]);
      if (a.state !== 'running' && a === o.action) GR.gripSynced = a.id;
    }
  }
}
/* Failures that an unset grasp setup can cause (the aim is off): the toast says which values are still the
   defaults (grip pixel never picked, camera not calibrated) and opens the grasp setup. */
const GRASP_AIM_CODES = new Set(['NOT_HOLDING', 'CENTRING_FAILED', 'TARGET_LOST']);
async function graspSetupGaps() {
  const gaps = [];
  if (GR.setup == null) await graspLoadSetup(true);
  const g = GR.setup && GR.setup.grip_pixel_norm;
  if (GR.camDown !== true && Array.isArray(g) && g[0] === 0.5 && g[1] === 0.5) gaps.push(t('grasp.gap.grip'));
  try { const c = await (await fetch('/api/v1/calibration', { cache: 'no-store' })).json(); if (c && c.calibrated === false) gaps.push(t('grasp.gap.cam')); } catch (_) {}
  return gaps;
}
async function graspFailToast(r) {
  const msg = t('grasp.res.fail', { why: graspCodeText(r.code, r.message, r) });
  const gaps = await graspSetupGaps();
  if (!gaps.length) { toast(msg, 'err', r.code || ''); return; }
  toast(msg + ' ' + t('grasp.gap', { g: gaps.join(t('cw.f.sep')) }), 'err', r.code || '', { act: { label: t('gset.title'), fn: () => gsetOpen(true) } });
}
function graspOnObs(o) {
  graspSyncGrip(o);
  const a = o.action, p = o.previous_action;
  for (const r of [a, p]) {
    if (!r || r.kind !== 'grasp' || r.state === 'running' || r.id === GR.toasted) continue;
    // results of grasps started here, or finished within the last 3 s (agent-started)
    if (r.id !== GR.id && !(o.timestamp - (r.finished_at ?? -1e9) < 3)) continue;
    GR.toasted = r.id;
    if (r.id === GR.id) { GR.id = null; GR.sel = null; }
    if (r.state === 'done') toast(t('grasp.res.ok'), 'ok');
    else if (r.state === 'failed' && GRASP_AIM_CODES.has(r.code)) graspFailToast(r);
    else toast(t(r.state === 'failed' ? 'grasp.res.fail' : 'grasp.res.cancel', { why: graspCodeText(r.code, r.message, r) }), r.state === 'failed' ? 'err' : 'warn', r.code || '');
  }
}
function graspStop(reason) {
  if (reason === 'button') { stopMotion({ okKey: 'grasp.stopped', okLevel: 'warn' }); return; }
  if ((!graspRunningAction() && !GR.sending) || S.leaseGeneration == null) return;
  const now = Date.now();
  if (now - GR.stopAt < 400) return;
  GR.stopAt = now;
  api('/api/v1/motion/stop', {}, { quiet: true }).catch((e) => toastOnce(e));
}
// Stick / key / jog input while a grasp runs stops it (checked every frame).
function graspCheckInput(v) {
  if (!graspRunningAction()) return;
  const cart = S.mode === 'cart' && (Object.values(cartVelocity()).some((x) => x !== 0) || cartGripVel() !== 0);
  if (v.some((x) => x !== 0) || cart || C.gripJog) graspStop('input');
}
window.addEventListener('keydown', (e) => {
  if (opIsOpen() || takeModalOpen() || preemptModalOpen() || S.leaseGeneration == null) return;
  if (!graspRunningAction() || e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key === 'Escape' || e.key === ' ' || e.code === 'Space' || ['Shift', 'Control', 'Meta', 'Alt', 'Tab'].includes(e.key)) return;
  if (isDeliberateStopTarget(e.target) || (e.target && e.target.closest && e.target.closest('input, textarea, #gset'))) return;
  graspStop('input');
}, true);
document.addEventListener('pointerdown', (e) => {
  if (opIsOpen() || takeModalOpen() || preemptModalOpen() || S.leaseGeneration == null) return;
  if (!graspRunningAction() || isDeliberateStopTarget(e.target)) return;
  const el = e.target && e.target.closest ? e.target : null;
  if (el && el.closest('.cockpit, .ops, .safety') && !el.closest('#grasp-bar')) graspStop('input');
}, true);

function graspVideoPoint(e) {
  const box = $('video-box'), r = box.getBoundingClientRect();
  const rect = getVideoContentRect(r.width, r.height);
  if (!rect) return null;
  const u = (e.clientX - r.left - rect.x) / rect.w, v = (e.clientY - r.top - rect.y) / rect.h;
  return u >= 0 && u <= 1 && v >= 0 && v <= 1 ? [u, v] : null;
}
$('video-box').addEventListener('click', (e) => {
  if (e.target.closest && e.target.closest('.hud, .grasp-bar, .grasp-pick, .vhint-btn, .depth-panel, .meas-card')) return;
  const pt = graspVideoPoint(e);
  if (!pt) return;
  if (GR.pickGrip) { graspSetGrip(pt); return; }
  // tap an ArUco marker (or its tag) for its pose HUD; tap it again for the automatic choice
  const mk = aruAt(e.clientX, e.clientY);
  if (mk !== undefined) { ARU.pick = ARU.pick === mk ? null : mk; buzz(10); return; }
  if (!graspDetOn() || graspRunningAction() || GR.sending) return;
  const [u, v] = pt;
  const hits = (S.detections || []).filter((d) => !d.aruco && u >= d.x0 && u <= d.x1 && v >= d.y0 && v <= d.y1)
    .sort((a, b) => a.w * a.h - b.w * b.h);
  const d = hits[0];
  if (!d && !DET.enabled && detHeld()) { GR.sel = null; detClearOverlay(); renderGrasp(); return; } // empty-space tap dismisses a held single-shot result
  GR.sel = d ? {
    label: d.label, score: d.score, box: [d.x0, d.y0, d.x1, d.y1], seenAt: performance.now(),
    contour: d.contour || null, contourBox: d.contour ? [d.x0, d.y0, d.x1, d.y1] : null, contourAt: d.contour ? performance.now() : 0,
  } : null;
  if (d && !DET.enabled && detHeld()) S.detHoldUntil = performance.now() + 60000;
  if (d) { buzz(10); if (MS.state && MS.state !== 'running') msClose(); }
  renderGrasp();
});
$('video-box').addEventListener('pointermove', (e) => { ARU.hover = e.pointerType === 'mouse' ? (aruAt(e.clientX, e.clientY) ?? null) : null; });
$('video-box').addEventListener('pointerleave', () => { ARU.hover = null; });
function graspIou(a, b) {
  const w = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0])), h = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1]));
  const u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - w * h;
  return u > 0 ? (w * h) / u : 0;
}
// Follow the selected box across frames: same label, best IoU, or nearest centre within 8 % of the image.
function graspTrack() {
  const s = GR.sel;
  if (!s || !S.detections) return;
  const c = [(s.box[0] + s.box[2]) / 2, (s.box[1] + s.box[3]) / 2];
  let best = null, score = 0;
  for (const d of S.detections) {
    if (d.aruco || d.label !== s.label) continue;
    const b = [d.x0, d.y0, d.x1, d.y1];
    const iou = graspIou(b, s.box), dist = Math.hypot(d.cx - c[0], d.cy - c[1]);
    const sc = iou > 0.3 ? 1 + iou : dist < 0.08 ? 1 - dist : 0;
    if (sc > score) { score = sc; best = { b, d }; }
  }
  if (best) { s.box = best.b; s.score = best.d.score; s.seenAt = performance.now(); }
}
function graspDraw(g, rect, fresh) {
  if (fresh) graspTrack();
  const s = GR.sel;
  if (s) {
    const stale = performance.now() - s.seenAt > 800;
    const x = rect.x + s.box[0] * rect.w, y = rect.y + s.box[1] * rect.h, w = (s.box[2] - s.box[0]) * rect.w, h = (s.box[3] - s.box[1]) * rect.h;
    // the outline from the live stream when carried, else the /detection/latest fallback (selContour)
    const sc = showOpt('contour') ? ((() => {
      const d = (S.detections || []).find((q) => q.contour && q.label === s.label && graspIou(s.box, [q.x0, q.y0, q.x1, q.y1]) > 0.5);
      return d ? { pts: d.contour, shift: null } : null;
    })() || selContour(s)) : null;
    g.save();
    g.setLineDash(stale ? [6, 5] : []);
    if (sc) {
      contourPath(g, rect, sc.pts, sc.shift);
      g.fillStyle = 'rgba(140,192,32,0.16)'; g.fill();
      g.lineJoin = 'round';
      g.lineWidth = 5; g.strokeStyle = 'rgba(255,255,255,0.9)'; g.stroke();
      g.lineWidth = 3; g.strokeStyle = '#8CC020'; g.stroke();
      g.setLineDash([5, 4]); g.lineWidth = 1; g.strokeStyle = 'rgba(255,255,255,0.6)'; g.strokeRect(x, y, w, h);
    } else {
      g.lineWidth = 5; g.strokeStyle = 'rgba(255,255,255,0.95)'; g.strokeRect(x - 2, y - 2, w + 4, h + 4);
      g.lineWidth = 3; g.strokeStyle = '#8CC020'; g.strokeRect(x - 2, y - 2, w + 4, h + 4);
    }
    g.restore();
    graspContourPoll();
  }
  placeObjCard(rect);
  // grip point marker while selecting, grasping or picking it
  const gp = GR.camDown !== true && GR.setup && GR.setup.grip_pixel_norm;
  if (gp && (s || GR.pickGrip || graspRunningAction() || !$('gset').hidden)) {
    const x = rect.x + gp[0] * rect.w, y = rect.y + gp[1] * rect.h;
    g.save();
    g.lineWidth = 2; g.strokeStyle = 'rgba(255,255,255,0.9)';
    g.beginPath(); g.arc(x, y, 9, 0, Math.PI * 2); g.stroke();
    g.strokeStyle = '#8CC020';
    g.beginPath(); g.moveTo(x - 14, y); g.lineTo(x - 4, y); g.moveTo(x + 4, y); g.lineTo(x + 14, y);
    g.moveTo(x, y - 14); g.lineTo(x, y - 4); g.moveTo(x, y + 4); g.lineTo(x, y + 14); g.stroke();
    g.restore();
  }
}
async function graspGo() {
  const s = GR.sel;
  if (!s || GR.sending) return;
  if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
  if (leaseModalOrPending()) return;
  if (S.leaseGeneration == null) { promptTakeControl(); return; }
  releaseAll();
  GR.sending = true; renderGrasp();
  try {
    const b = s.box.map((v) => Math.round(Math.min(1, Math.max(0, v)) * 10000) / 10000);
    const task = graspTaskCur();
    const ack = await graspRPC('/api/v1/action/grasp', { label: s.label, box_norm: b, ts_ms: Date.now(), tag: 'cockpit', strategy: GR.strategy, ...(task ? { task } : {}) });
    GR.id = ack.action_id ?? null;
    if (GR.setup == null) graspLoadSetup(true);
  } catch (_) {
    // the selection stays so the user can retry or cancel
  } finally { GR.sending = false; renderGrasp(); }
}
function renderGrasp() {
  const bar = $('grasp-bar');
  if (!bar) return;
  const run = graspRunningAction();
  if (!run && !graspDetOn() && GR.sel && !GR.sending) GR.sel = null;
  const show = !!(run || (GR.sel && graspDetOn()) || GR.sending);
  bar.hidden = !show || GR.pickGrip;
  $('grasp-pick').hidden = !GR.pickGrip;
  const gs = $('btn-gset');
  if (gs) gs.hidden = !(GR.supported || (S.caps && S.caps.grasp));
  if (!show) { placeObjCard(null); if (typeof exSyncCtx === 'function') exSyncCtx(); return; }
  $('grasp-info').hidden = !GR.sel;
  if (GR.sel) {
    setText($('grasp-label'), GR.sel.label);
    setText($('grasp-score'), Math.round(GR.sel.score * 100) + '%');
    setText($('grasp-src'), VM.kind === 'segment' && selContour(GR.sel) ? t('oc.src.seg') : VM.kind === 'segment' ? '' : t('oc.src.box'));
  }
  // simple: measuring note under the measure button, the grasp task row (Settings opens the grasp settings)
  const measOk = !!(S.caps && S.caps.measure_multiview);
  $('grasp-sub').hidden = !!run || !measOk;
  setText($('meas-sub'), t('oc.meas.sub', { cm: (MS_REQ.baseline_m * 50).toFixed(0) }));
  $('grasp-task').hidden = !!run || !(GR.supported || (S.caps && S.caps.grasp));
  const tk = graspTaskCur();
  setText($('grasp-task-name'), tk || t('oc.task.def'));
  $('grasp-orient').hidden = $('grasp-task').hidden;
  setText($('grasp-orient-name'), t(GR.strategy === 'legacy' ? 'oc.orient.legacy' : 'oc.orient.adaptive'));
  bar.querySelector('.grasp-acts').classList.toggle('one', !!run || !measOk);
  $('grasp-prog').hidden = !run;
  $('grasp-go').hidden = !!run;
  $('grasp-cancel').hidden = !!run;
  $('grasp-stop').hidden = !run;
  $('grasp-go').disabled = GR.sending || !GR.sel;
  const mg = $('meas-go');
  mg.hidden = !!run || !(S.caps && S.caps.measure_multiview);
  mg.disabled = GR.sending || !GR.sel || MS.state === 'running' || MS.state === 'starting';
  if (run) {
    const ph = run.phase || 'start';
    const txt = ph === 'centring' && run.iteration != null && run.error_px != null
      ? t('grasp.ph.centring.n', { n: run.iteration, px: Math.round(run.error_px) }) : t('grasp.ph.' + ph);
    setText($('grasp-phase'), txt);
    $('grasp-fill').style.width = Math.round(Math.min(1, Math.max(0, run.progress || 0)) * 100) + '%';
  }
  if (typeof exSyncCtx === 'function') exSyncCtx();
}
$('grasp-go').addEventListener('click', graspGo);
$('grasp-cancel').addEventListener('click', () => { GR.sel = null; renderGrasp(); });
bindStopButton($('grasp-stop'), { okKey: 'grasp.stopped', okLevel: 'warn' });

/* ---- size measurement of the selected object (capability measure_multiview, WS /api/v1/action/measure_multiview):
   a confirmation with the range and duration, then the action's progress, then a result card that stays until it is
   closed or another object is selected. It moves the arm: never sent again by itself (an unlock asks for another tap).
   "Quick estimate" is the single-view depth-model estimate (POST /api/v1/grasp/measure), labelled as such. ---- */
const MS_REQ = { views: 3, baseline_m: 0.06, speed_scale: 0.3, return_to_start: true };
const msMm = (m) => (Number.isFinite(m) ? (m * 1000).toFixed(1) : '--');
function msClose() { Object.assign(MS, { state: null, sel: null, id: null, res: null, err: null, est: null }); msRender(); }
function msErrText(code, message) {
  const k = 'ms.e.' + code;
  const hint = I18N.zh[k] !== undefined ? t(k) : errWhy(code);
  return hint || message || code || t('err.rejected');
}
function msRow(dl, k, v) { dl.append(cwEl('dt', '', k), cwEl('dd', '', v)); }
function msResultText() {
  const o = MS.res && MS.res.objects && MS.res.objects[0];
  if (!o) return '';
  return [`${MS.sel ? MS.sel.label : o.label}: H ${msMm(o.height_m)} x W ${msMm(o.width_m)} x L ${msMm(o.length_m)} mm`,
    `centroid_m ${JSON.stringify(o.centroid_m)}`, `n_points ${o.n_points}`, `reproj_rms_px ${o.reproj_rms_px}`,
    `footprint ${o.footprint_source ?? '-'} views ${o.footprint_views ?? '-'} conf L ${o.length_confidence ?? '-'} W ${o.width_confidence ?? '-'}`,
    `table_check ${JSON.stringify(MS.res.table_check || null)}`, `total_ms ${MS.res.timing ? MS.res.timing.total_ms : '-'}`].join('\n');
}
async function msCopy() {
  const txt = msResultText();
  try { await navigator.clipboard.writeText(txt); toast(t('ms.copied'), 'ok'); return; } catch (_) {}
  // plain http to the device: no clipboard API; a selected text field and the copy command
  const ta = document.createElement('textarea'); ta.value = txt; ta.setAttribute('readonly', ''); ta.style.position = 'fixed'; ta.style.opacity = '0';
  document.body.appendChild(ta); ta.select();
  let ok = false; try { ok = document.execCommand('copy'); } catch (_) {}
  ta.remove(); toast(t(ok ? 'ms.copied' : 'ms.copy_fail'), ok ? 'ok' : 'warn');
}
function msRender() {
  const c = $('meas-card');
  if (!c) return;
  const wasHidden = c.hidden;
  c.hidden = !MS.state;
  if (typeof exSyncCtx === 'function') exSyncCtx();
  if (wasHidden && !c.hidden && typeof svSheetPlace === 'function') requestAnimationFrame(svSheetPlace);
  if (!MS.state) return;
  c.textContent = '';
  const head = cwEl('h4', '', t('ms.title'));
  head.appendChild(cwEl('small', '', MS.sel ? MS.sel.label : ''));
  c.appendChild(head);
  const acts = cwEl('div', 'ms-acts');
  const btn = (label, fn, cls = '', id = '') => { const b = cwEl('button', 'btn ' + cls, label); b.type = 'button'; if (id) b.id = id; if (fn) b.addEventListener('click', fn); acts.appendChild(b); return b; };
  if (MS.state === 'confirm' || MS.state === 'starting') {
    c.appendChild(cwEl('p', '', t('ms.confirm', { cm: (MS_REQ.baseline_m * 50).toFixed(0), n: MS_REQ.views, sp: Math.round(MS_REQ.speed_scale * 100) })));
    c.appendChild(cwEl('p', 'ms-warn', t('ms.stop_note')));
    btn(t('ms.start'), msStart, 'btn-primary', 'ms-start').disabled = MS.state === 'starting';
    btn(t('ms.quick'), msQuick, '', 'ms-quick');
    btn(t('cancel'), msClose, '', 'ms-cancel');
  } else if (MS.state === 'running') {
    const ph = MS.phase || '';
    const m = /^(view|capture):(\w+)/.exec(ph);
    const txt = m ? t('ms.ph.' + m[1], { k: m[2] === 'start' ? 1 : Number(m[2]) + 1, n: MS_REQ.views }) : t('ms.ph.' + (ph || 'start'));
    c.appendChild(cwEl('p', '', txt));
    const bar = cwEl('div', 'calib-bar'); const fill = document.createElement('i'); fill.style.width = Math.round(MS.progress * 100) + '%'; bar.appendChild(fill); c.appendChild(bar);
    bindStopButton(btn(t('grasp.stop'), null, 'grasp-stop', 'ms-stop'));
  } else if (MS.state === 'done') {
    const o = MS.res && MS.res.objects && MS.res.objects[0];
    if (!o) c.appendChild(cwEl('p', 'ms-err', t('ms.e.none')));
    else {
      c.appendChild(cwEl('div', 'ms-big', `${msMm(o.height_m)} × ${msMm(o.width_m)} × ${msMm(o.length_m)} mm`));
      c.appendChild(cwEl('div', 'ms-note', t('ms.hwl')));
      const dl = cwEl('dl', '');
      msRow(dl, t('ms.r.centroid'), Array.isArray(o.centroid_m) ? o.centroid_m.map(msMm).join(', ') + ' mm' : '--');
      msRow(dl, t('ms.r.points'), `${o.n_points ?? '--'}` + (o.n_edge_points != null ? ` (${t('ms.r.edges')} ${o.n_edge_points})` : ''));
      msRow(dl, t('ms.r.rms'), Number.isFinite(o.reproj_rms_px) ? o.reproj_rms_px.toFixed(2) + ' px' : '--');
      msRow(dl, t('ms.r.views'), String(o.views_used ?? '--'));
      msRow(dl, t('ms.r.footprint'), o.footprint_source ? `${o.footprint_source} · ${o.footprint_views ?? '?'} ${t('ms.r.views_u')}` : t('ms.r.fp_none'));
      if (o.length_confidence != null || o.width_confidence != null) msRow(dl, t('ms.r.conf'), `L ${o.length_confidence ?? '--'} · W ${o.width_confidence ?? '--'}`);
      const tc = MS.res.table_check;
      msRow(dl, t('ms.r.table'), tc ? t('ms.r.table_v', { med: Number(tc.median_abs_mm).toFixed(1), p90: Number(tc.p90_abs_mm).toFixed(1), n: tc.n_points }) : '--');
      const tm = MS.res.timing;
      msRow(dl, t('ms.r.time'), tm && Number.isFinite(tm.total_ms) ? (tm.total_ms / 1000).toFixed(1) + ' s' : '--');
      c.appendChild(dl);
      c.appendChild(cwEl('p', 'ms-note', t('ms.r.table_note')));
    }
    btn(t('ms.copy'), msCopy, '', 'ms-copy');
    btn(t('ms.again'), () => { MS.state = 'confirm'; MS.res = null; msRender(); }, '', 'ms-again');
    btn(t('op.close'), msClose, '', 'ms-close');
  } else if (MS.state === 'failed') {
    c.appendChild(cwEl('p', 'ms-err', t('ms.fail', { why: MS.err.text }) + (MS.err.code ? ` (${MS.err.code})` : '')));
    btn(t('ms.again'), () => { MS.state = 'confirm'; MS.err = null; msRender(); }, '', 'ms-again');
    btn(t('op.close'), msClose, '', 'ms-close');
  } else if (MS.state === 'estimate') {
    const box = cwEl('div', 'ms-est');
    box.appendChild(cwEl('b', '', t('ms.est.title')));
    const d = MS.est;
    if (d && d.measurement) box.appendChild(cwEl('div', 'ms-big', `${msMm(d.measurement.height_m)} × ${msMm(d.measurement.width_m)} × ${msMm(d.measurement.length_m)} mm`));
    else { const w = (d && d.why) || ''; box.appendChild(cwEl('p', '', t('ms.est.none', { why: I18N.zh['ms.why.' + w] !== undefined ? t('ms.why.' + w) : (w || '--') }))); }
    box.appendChild(cwEl('p', 'ms-note', t('ms.est.note')));
    c.appendChild(box);
    btn(t('ms.start'), () => { MS.state = 'confirm'; msRender(); }, 'btn-primary', 'ms-start-full');
    btn(t('op.close'), msClose, '', 'ms-close');
  }
  c.appendChild(acts);
}
function msOpen() {
  if (!GR.sel) return;
  if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
  MS.sel = { ...GR.sel }; MS.state = 'confirm'; MS.res = null; MS.err = null; MS.est = null;
  msRender();
}
async function msStart() {
  if (MS.state !== 'confirm' || !MS.sel) return;
  MS.state = 'starting'; msRender();
  const b = MS.sel.box.map((v) => Math.round(Math.min(1, Math.max(0, v)) * 10000) / 10000);
  try {
    const ack = await commandRPC('/api/v1/action/measure_multiview', { ...MS_REQ, label: MS.sel.label, target: { box_norm: b } }, true, 5000);
    MS.id = ack.action_id ?? null; MS.state = 'running'; MS.phase = ''; MS.progress = 0;
  } catch (e) {
    if (isCancelledErr(e)) { MS.state = 'confirm'; msRender(); return; }
    // OPERATOR_REQUIRED: commandRPC opened the dialog; a motion is not sent again by itself
    MS.state = 'failed'; MS.err = { code: e.code || '', text: e.code === 'OPERATOR_REQUIRED' ? t('ms.e.OPERATOR_REQUIRED') : msErrText(e.code, e.message) };
  }
  msRender();
}
async function msQuick() {
  if (!MS.sel) return;
  try {
    const d = await api('/api/v1/grasp/measure', { label: MS.sel.label }, { quiet: true });
    const dets = (d && d.detections) || [];
    const iou = (a, b) => { const x = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0])), y = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1])), i = x * y;
      return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i || 1); };
    MS.est = dets.slice().sort((p, q) => iou(q.box_norm || [0, 0, 0, 0], MS.sel.box) - iou(p.box_norm || [0, 0, 0, 0], MS.sel.box))[0] || { why: 'no_detection' };
    MS.state = 'estimate';
  } catch (e) {
    MS.state = 'failed'; MS.err = { code: e.code || '', text: msErrText(e.code, e.message) };
  }
  msRender();
}
function msOnObs(o) {
  if (MS.id == null) return;
  const r = [o.action, o.previous_action].find((x) => x && x.id === MS.id);
  if (!r) return;
  if (r.state === 'running') {
    if (MS.state === 'running' && (r.phase !== MS.phase || Math.abs((r.progress || 0) - MS.progress) > 0.01)) { MS.phase = r.phase || ''; MS.progress = r.progress || 0; msRender(); }
    return;
  }
  MS.id = null;
  if (r.state === 'done') { MS.state = 'done'; MS.res = r.measurement || null; }
  else { MS.state = 'failed'; MS.err = { code: r.code || '', text: r.state === 'cancelled' ? t('ms.cancelled') : msErrText(r.code, r.message) }; }
  msRender();
}
$('meas-go').addEventListener('click', msOpen);
i18nHooks.push(msRender);

// ---- grasp setup drawer ----
const mm = (m) => (Number.isFinite(m) ? Math.round(m * 1000) : null);
// the tilt field shows 1 decimal; Save sends camera_tilt_deg only after the user edited the field,
// so a calibrated mount's derived tilt (e.g. -36.995338) is not overwritten by its rounded display
const tilt1 = (v) => (Number.isFinite(v) ? (Math.round(v * 10) / 10).toFixed(1) : null);
function gsetRender(status) {
  const s = GR.setup;
  if (typeof gsetTasksRender === 'function') gsetTasksRender();
  setText($('gset-table'), s ? (mm(s.table_z_m) + ' mm') : '--');
  // camera-down acquisition centres on the image centre: the grip pixel is not used, so it is not shown
  const cd = !!s && GR.camDown === true;
  setText($('gset-grip'), !s ? '--' : cd ? t('gset.grip.cd') : s.grip_pixel_norm.map((v) => Math.round(v * 100) + '%').join(' , '));
  $('gset-grip').classList.toggle('na', cd);
  setText($('gset-grip-hint'), t(cd ? 'gset.grip.cd.hint' : 'gset.grip.hint'));
  $('gset-grip-pick').hidden = cd;
  for (const [id, v] of [['gset-approach', s && mm(s.approach_height_m)], ['gset-grasp', s && mm(s.grasp_height_m)], ['gset-tilt', s && tilt1(s.camera_tilt_deg)]]) {
    const el = $(id);
    if (document.activeElement !== el) { el.value = v == null ? '' : String(v); if (id === 'gset-tilt') GR.tiltEdited = false; }
    el.disabled = !s;
  }
  for (const id of ['gset-table-tcp', 'gset-grip-pick', 'gset-save']) $(id).disabled = !s;
  if (status !== undefined) {
    const st = $('gset-status');
    setText(st, status);
    st.classList.toggle('ok', !!s && status === t('gset.saved'));
  }
}
async function graspLoadSetup(quiet) {
  if (!quiet) gsetRender(t('gset.loading'));
  try {
    const res = await fetch('/api/v1/grasp/setup', { cache: 'no-store' });
    const d = await res.json();
    if (!res.ok) throw new Error(d && (d.message || d.error) || 'HTTP ' + res.status);
    GR.supported = !!d.supported; GR.setup = d.setup || null; GR.path = d.path || null;
    GR.camDown = typeof d.camera_down === 'boolean' ? d.camera_down : null;
    if (typeof d.revision === 'number') GR.rev = d.revision;
    if (typeof renderSVis === 'function') { renderSVis(); renderGrasp(); }
    if (!quiet) gsetRender(!GR.supported ? t('gset.na') : GR.path ? t('gset.path', { p: GR.path }) : '--');
  } catch (e) {
    if (!quiet) gsetRender(t('gset.err', { e: e.message }));
  }
}
async function gsetWrite(patch) {
  try {
    const body = GR.rev != null ? { ...patch, expected_revision: GR.rev } : patch;
    const ack = await graspRPC('/api/v1/grasp/setup', body);
    if (ack.setup) GR.setup = ack.setup;
    if (typeof ack.revision === 'number') GR.rev = ack.revision;
    const msg = ack.persisted === false ? t('gset.unsaved', { e: ack.persist_error || '' }) : t('gset.saved');
    gsetRender(msg);
    toast(msg, ack.persisted === false ? 'warn' : 'ok');
    // the tilt decides camera-down acquisition (camera_down is only in the GET)
    graspLoadSetup(true).then(() => { if (!$('gset').hidden) gsetRender(); });
    return true;
  } catch (e) {
    if (isCancelledErr(e)) return false;
    if (e.code === 'POLICY_REVISION_MISMATCH') graspLoadSetup(true);
    gsetRender(e.message);
    return false;
  }
}
function gsetOpen(open) {
  $('gset').hidden = !open;
  if (open) { gsetRender(); graspLoadSetup(false); $('gset-close').focus({ preventScroll: true }); }
}
function graspSetGrip(pt) {
  GR.pickGrip = false;
  renderGrasp();
  gsetWrite({ grip_pixel_norm: pt.map((v) => Math.round(v * 10000) / 10000) }).finally(() => gsetOpen(true));
}
$('btn-gset').addEventListener('click', () => gsetOpen(true));
$('gset-close').addEventListener('click', () => gsetOpen(false));
$('gset').addEventListener('click', (e) => { if (e.target === e.currentTarget) gsetOpen(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('gset').hidden) gsetOpen(false); });
$('gset-reload').addEventListener('click', () => graspLoadSetup(false));
$('gset-table-tcp').addEventListener('click', () => gsetWrite({ table_z_from_tcp: true }));
$('gset-grip-pick').addEventListener('click', () => { gsetOpen(false); GR.pickGrip = true; renderGrasp(); });
$('grasp-pick-x').addEventListener('click', () => { GR.pickGrip = false; renderGrasp(); gsetOpen(true); });
$('gset-tilt').addEventListener('input', () => { GR.tiltEdited = true; });
$('gset-save').addEventListener('click', () => {
  const a = Number($('gset-approach').value), g = Number($('gset-grasp').value), tl = Number($('gset-tilt').value);
  if (![a, g, tl].every(Number.isFinite) || $('gset-approach').value === '' || $('gset-grasp').value === '' || $('gset-tilt').value === '') { gsetRender(t('gset.bad')); return; }
  const patch = { approach_height_m: a / 1000, grasp_height_m: g / 1000 };
  if (GR.tiltEdited) patch.camera_tilt_deg = tl;
  gsetWrite(patch);
});
i18nHooks.push(() => { renderGrasp(); if (!$('gset').hidden) gsetRender(); });
graspLoadSetup(true);

/* ---- simple mode: recognition panel (#s-vis), detection kind switch, display options, object card, grasp tasks ----
   The kind switch (POST /api/v1/vision/detector) moves the detector between the two default models of armd; any other
   model (obb, a variant) shows as text and is changed in expert mode or by a grasp task. Grasp tasks whose required
   detection kind or detector model differs from the loaded one are not listed in simple mode (no switching from here). */
const SV = { kindBusy: false, scrollBack: null };
const SHOW = (() => { try { const v = JSON.parse(store('rebot.sv.show') || 'null'); if (v && typeof v === 'object') return { contour: v.contour !== false, box: !!v.box, label: v.label !== false }; } catch (_) {} return { contour: true, box: false, label: true }; })();
const EX_SHOW = (() => { try { const v = JSON.parse(store('rebot.ex.show') || 'null'); if (v && typeof v === 'object') return { contour: v.contour !== false, box: v.box !== false, label: v.label !== false }; } catch (_) {} return { contour: true, box: true, label: true }; })();
const showOpt = (k) => (S.ui === 'simple' ? SHOW[k] : EX_SHOW[k]);
const svArmBusy = () => !!(graspRunningAction() || ['starting', 'running'].includes(MS.state) || (S.obs && S.obs.action && S.obs.action.kind === 'measure_multiview' && S.obs.action.state === 'running'));
function svChipText() {
  if (!DET.avail) return t('det.na');
  if (!DET.enabled) return t('det.off');
  const age = S.detLastReceivedAt ? performance.now() - S.detLastReceivedAt : Infinity;
  if (!isFinite(age) || age > 2000) return t('sv.chip.wait');
  const n = (S.detections || []).filter((d) => !d.aruco).length;
  return n ? t('sv.chip.n', { n }) : t('sv.chip.0');
}
function svOn() { return VM.state === 'ok' ? vmSel().yolo : DET.enabled; }
function simpleVisSet(on) {
  if (!DET.avail) return;
  if (VM.state === 'ok') return setVisionMode(vmModeOf(on, vmSel().aruco));
  if (DET.enabled !== on) return toggleDet();
}
function renderSVis() {
  const p = $('s-vis');
  if (!p) return;
  p.hidden = DET.avail === null;
  const ok = VM.state === 'ok', on = !!DET.avail && svOn();
  const sw = $('sv-on');
  sw.setAttribute('aria-checked', String(on));
  sw.disabled = !DET.avail || VM.busy || DET.busy || DET.snap || (ok && !VM.yolo && !on);
  const mw = $('sv-model-warn');
  if (mw) {
    const mc = ok ? VM.modelConfig : null;
    const bad = !!(mc && mc.optimal === false);
    mw.hidden = !bad;
    if (bad) {
      const miss = Array.isArray(mc.missing) ? mc.missing.filter(Boolean) : [];
      const mism = Array.isArray(mc.mismatched) ? mc.mismatched.filter(Boolean) : [];
      const parts = [];
      if (miss.length) parts.push(t('sv.model.missing', { list: miss.join(', ') }));
      if (mism.length) parts.push(t('sv.model.mismatch', { list: mism.join(', ') }));
      setText(mw, parts.length ? t('sv.model.warn', { detail: parts.join(t('sv.model.sep')) }) : t('sv.model.warn_gen'));
    }
  }
  // detection kind: the provider's model.kind; the two defaults switch here, any other kind is shown as text
  const kind = VM.kind, std = kind === 'detect' || kind === 'segment';
  const capOff = S.caps && S.caps.vision && S.caps.vision.detector_switch === false;
  $('sv-kind-sec').hidden = !ok || capOff || (!kind && !VM.models);
  const armBusy = svArmBusy();
  for (const b of $('sv-kind').querySelectorAll('button[data-kind]')) {
    b.setAttribute('aria-checked', String(b.dataset.kind === kind));
    b.disabled = !VM.models || !std || SV.kindBusy || armBusy;
  }
  const note = $('sv-kind-note');
  const other = kind && !std;
  setText(note, SV.kindBusy ? t('sv.kind.busy') : !VM.models ? t('sv.kind.na')
    : other ? t('sv.kind.other', { name: VM.modelName || '--', kind }) : armBusy ? t('sv.kind.armbusy') : t('sv.kind.note'));
  note.classList.toggle('warn', !!(other || armBusy || !VM.models));
  for (const b of $('sv-show').querySelectorAll('button[data-show]')) {
    b.setAttribute('aria-pressed', String(!!SHOW[b.dataset.show]));
    if (b.dataset.show === 'contour') b.hidden = kind !== 'segment';
  }
  const sel = ok ? vmSel() : { aruco: false };
  $('sv-other').hidden = !ok;
  $('sv-aruco').setAttribute('aria-checked', String(!!sel.aruco));
  $('sv-aruco').disabled = VM.busy || DET.snap;
  const dBlock = ok ? depthBlock() : t('vm.tip.na');
  $('sv-depth').setAttribute('aria-checked', String(!!DEPTH.on));
  $('sv-depth').disabled = !DEPTH.on && !!dBlock;
  $('sv-depth-row').title = dBlock || t('depth.tip');
  $('sv-gset').closest('.sv-foot').hidden = !(GR.supported || (S.caps && S.caps.grasp));
}
function renderSVisMeta() {
  const m = $('sv-meta');
  if (!m || !document.body.classList.contains('vis-open')) return;
  const parts = [];
  if (V.playing && V.mode !== 'mjpeg') parts.push(t('sv.meta.video', { fps: V.fps == null ? '--' : V.fps, ms: V.lat == null ? '--' : Math.round(V.lat) }));
  parts.push(DET.enabled ? t('sv.meta.det', { fps: DET.fps ?? DET.activeFps ?? '--' }) : t('sv.meta.off'));
  setText(m, parts.join(' · '));
}
async function svKindSet(kind, retried = false) {
  if (SV.kindBusy || VM.kind === kind || !['detect', 'segment'].includes(kind)) return;
  if (svArmBusy()) { toast(t('sv.kind.fail'), 'warn', 'VISION_BUSY'); return; }
  SV.kindBusy = true; renderSVis();
  let unlock = false;
  try {
    const ctl = new AbortController();
    const tm = setTimeout(() => ctl.abort(), 9000); // armd waits up to 5 s for the provider's swap
    let res, data = null;
    try {
      res = await fetch('/api/v1/vision/detector', { method: 'POST', signal: ctl.signal, cache: 'no-store', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ kind }) });
      try { data = await res.json(); } catch (_) {}
    } finally { clearTimeout(tm); }
    const code = (data && data.code) || '';
    if (res.status === 403 && code === 'OPERATOR_REQUIRED' && !retried) unlock = true;
    else if (!res.ok) {
      const mb = /needs ([\d.]+) MB, limit ([\d.]+) MB/.exec((data && data.message) || '');
      const known = { DETECTION_UNAVAILABLE: 'VISION_DOWN' }[code] || code;
      const extra = EC_KEYS[known] ? (mb ? ` (${mb[1]} / ${mb[2]} MB)` : '') : code === 'UNSUPPORTED' ? ': ' + t('sv.kind.na') : (data && data.message ? ': ' + data.message : '');
      toast(t('sv.kind.fail') + extra, 'err', known || String(res.status), code === 'OPERATOR_REQUIRED' ? { path: '/api/v1/vision/detector' } : null);
    } else toast(t('sv.kind.ok', { k: t('sv.kind.' + kind) }), 'ok');
  } catch (e) {
    toast(t('sv.kind.fail'), 'err', e && e.name === 'AbortError' ? 'VISION_TIMEOUT' : 'VISION_DOWN');
  } finally {
    SV.kindBusy = false;
    await vmRefresh();
    renderSVis(); gsetTasksRender();
  }
  if (unlock && await opRequire('/api/v1/vision/detector')) return svKindSet(kind, true);
}
function initSVis() {
  $('sv-on').addEventListener('click', () => simpleVisSet(!svOn()));
  $('sv-x').addEventListener('click', () => visPopOpen(false));
  $('sv-kind').addEventListener('click', (e) => { const b = e.target.closest('button[data-kind]'); if (b && !b.disabled) svKindSet(b.dataset.kind); });
  $('sv-show').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-show]');
    if (!b) return;
    SHOW[b.dataset.show] = !SHOW[b.dataset.show];
    store('rebot.sv.show', JSON.stringify(SHOW));
    renderSVis();
  });
  $('sv-aruco').addEventListener('click', () => { if (VM.state === 'ok') { const c = vmSel(); setVisionMode(vmModeOf(c.yolo, !c.aruco)); } });
  $('sv-depth').addEventListener('click', () => vmToggle('depth'));
  $('sv-gset').addEventListener('click', () => { visPopOpen(false); gsetOpen(true); });
  $('grasp-x').addEventListener('click', () => { GR.sel = null; renderGrasp(); });
  $('grasp-task').addEventListener('click', () => gsetOpen(true));
  $('grasp-orient').addEventListener('click', () => { GR.strategy = GR.strategy === 'legacy' ? 'adaptive' : 'legacy'; store('rebot.grasp.strategy', GR.strategy); renderGrasp(); });
  i18nHooks.push(renderSVis);
  renderSVis();
}

/* ---- expert mode: Layout A (tablet/desktop) + Layout B-style (phone portrait), model variants + ION, diagnostics ---- */
const EX = {
  popOpen: false,
  busy: false,
  err: '',
  ptab: 'move',
  dtab: 'tele',
  lastGen: null,
  phoneMq: window.matchMedia('(max-width: 600px)'),
};
const exIsPhone = () => S.ui === 'expert' && EX.phoneMq.matches;
function exShortModelName(name) {
  if (!name) return '--';
  if (/seg.*384/i.test(name)) return 'seg_384';
  if (/segment/i.test(name)) return 'segment';
  if (/detection|detect/i.test(name)) return 'detect';
  if (/obb/i.test(name)) return 'obb';
  return name.length > 16 ? name.slice(0, 14) + '…' : name;
}
function exUpdateSafeBot() {
  if (!exIsPhone()) return;
  const s = $('safety');
  if (!s) return;
  const r = s.getBoundingClientRect();
  if (r.bottom > 48) document.documentElement.style.setProperty('--ex-safe-bot', Math.ceil(r.bottom) + 'px');
}
function exPlaceDom() {
  const expert = S.ui === 'expert';
  const phone = exIsPhone();
  const stage = document.querySelector('.stage');
  const main = document.querySelector('.main');
  const side = $('side');
  const ops = document.querySelector('.ops');
  const vb = $('video-box');
  const vs = $('view-switch');
  const trig = $('btn-trigger');
  const vcard = $('ex-vcard');
  const vpop = $('ex-vpop');
  const objHost = $('ex-obj-host');
  const safety = $('safety');
  const sline = $('ex-sline');
  const ls = $('lease-status');
  const pwr = $('pwr');
  const gc = $('btn-gc');
  if (!stage || !vb || !vcard || !safety) return;

  if (expert) {
    // Video + view-switch + trigger inside video card
    if (vb.parentElement !== vcard) vcard.insertBefore(vb, vpop);
    if (vs && vs.parentElement !== $('ex-vs-slot')) $('ex-vs-slot').appendChild(vs);
    if (trig && trig.parentElement !== $('ex-trig-slot')) $('ex-trig-slot').appendChild(trig);
    // Contextual object/measure/depth inside object card under video
    for (const id of ['grasp-bar', 'meas-card', 'depth-panel']) {
      const el = $(id);
      if (el && el.parentElement !== objHost) objHost.appendChild(el);
    }
    // Unified safety status line (#lease-status stays in #safety on phone so lost-connection + #btn-reconnect-now stay reachable)
    if (pwr && pwr.parentElement !== sline) sline.appendChild(pwr);
    // Diagnostics cards inside diagnostics drawer tabs
    const teleHost = $('ex-diag-tele'), recHost = $('ex-diag-rec'), kbdHost = $('ex-diag-kbd');
    if (teleHost && $('ex-card-comm') && $('ex-card-comm').parentElement !== teleHost) teleHost.appendChild($('ex-card-comm'));
    if (teleHost && $('ex-card-joints') && $('ex-card-joints').parentElement !== teleHost) teleHost.appendChild($('ex-card-joints'));
    if (recHost && $('ex-card-rec') && $('ex-card-rec').parentElement !== recHost) recHost.appendChild($('ex-card-rec'));
    if (kbdHost && $('ex-card-kbd') && $('ex-card-kbd').parentElement !== kbdHost) kbdHost.appendChild($('ex-card-kbd'));

    if (phone) {
      // Layout B-style phone portrait: motion inside Move tab, model picker inside Vision tab
      if (ls && ls.parentElement !== safety) safety.insertBefore(ls, sline);
      if (ops && ops.parentElement !== $('ex-ppane-move')) $('ex-ppane-move').appendChild(ops);
      if (vpop && vpop.parentElement !== $('ex-ppane-vision')) $('ex-ppane-vision').appendChild(vpop);
      if (vpop) vpop.hidden = false;
      const pposes = $('ex-ppane-poses')?.querySelector('.ex-pposes');
      if (gc && pposes && gc.parentElement !== pposes) pposes.appendChild(gc);
      EX.popOpen = false;
      $('ex-model-chip')?.setAttribute('aria-expanded', String(EX.ptab === 'vision'));
    } else {
      // Layout A tablet/desktop: motion card in right rail, model picker popover in video card
      if (ls && ls.parentElement !== sline) sline.insertBefore(ls, $('ex-link'));
      if (ops && ops.parentElement !== side) side.appendChild(ops);
      if (vpop && vpop.parentElement !== vcard) vcard.appendChild(vpop);
      if (vpop) vpop.hidden = !EX.popOpen;
      if (gc && $('ex-presets') && gc.parentElement !== $('ex-presets')) $('ex-presets').appendChild(gc);
      $('ex-model-chip')?.setAttribute('aria-expanded', String(EX.popOpen));
    }
    requestAnimationFrame(exUpdateSafeBot);
  } else {
    // Restore everything symmetrically to simple mode
    if (vb.parentElement !== stage) stage.insertBefore(vb, vcard);
    if (vs && vs.parentElement !== vb) vb.insertBefore(vs, vb.querySelector('.hud'));
    if (trig && $('det-ctl') && trig.parentElement !== $('det-ctl')) $('det-ctl').insertBefore(trig, $('fps-pill'));
    const gpick = $('grasp-pick');
    for (const id of ['grasp-bar', 'meas-card', 'depth-panel']) {
      const el = $(id);
      if (el && el.parentElement === objHost) vb.insertBefore(el, gpick || null);
    }
    if (typeof placeSlot === 'function') placeSlot();
    if (ls && ls.parentElement !== safety) safety.insertBefore(ls, sline);
    if (pwr && pwr.parentElement !== safety) safety.insertBefore(pwr, $('lock-note'));
    if (gc && gc.parentElement !== safety) safety.insertBefore(gc, $('lock-note'));
    if (ops && ops.parentElement !== main) main.insertBefore(ops, side);
    if (vpop && vpop.parentElement !== vcard) vcard.appendChild(vpop);
    EX.popOpen = false;
    if (vpop) vpop.hidden = true;
    exMoreOpen(false);
    exDiagOpen(false);
  }
  exSyncCtx();
  renderExpertVision();
}
function exSyncCtx() {
  const empty = $('ex-obj-empty');
  if (!empty) return;
  if (S.ui !== 'expert') { empty.hidden = true; return; }
  const hasActive = !($('grasp-bar')?.hidden) || !($('meas-card')?.hidden) || !($('depth-panel')?.hidden);
  empty.hidden = hasActive;
}
function exVpopOpen(open) {
  if (exIsPhone()) {
    if (open) exPhoneTab('vision');
    return;
  }
  EX.popOpen = !!open;
  const pop = $('ex-vpop'), chip = $('ex-model-chip');
  if (pop) pop.hidden = !EX.popOpen;
  if (chip) chip.setAttribute('aria-expanded', String(EX.popOpen));
  if (EX.popOpen) { EX.err = ''; renderExpertVision(); }
}
function exPhoneTab(tab) {
  if (!['move', 'vision', 'poses'].includes(tab)) return;
  EX.ptab = tab;
  for (const b of $('ex-ptabs').querySelectorAll('button[data-ptab]')) {
    b.setAttribute('aria-selected', String(b.dataset.ptab === tab));
  }
  for (const p of ['move', 'vision', 'poses']) {
    $('ex-ppane-' + p)?.classList.toggle('on', p === tab);
  }
  if (exIsPhone()) $('ex-model-chip')?.setAttribute('aria-expanded', String(tab === 'vision'));
}
function exMoreOpen(open) {
  const m = $('ex-more');
  if (!m) return;
  if (open) { exDiagOpen(false); exVpopOpen(false); exUpdateSafeBot(); }
  m.hidden = !open;
  if ($('ex-m-fs')) $('ex-m-fs').hidden = $('btn-fs') ? $('btn-fs').hidden : true;
  if ($('ex-m-gset')) $('ex-m-gset').hidden = !(GR.supported || (S.caps && S.caps.grasp));
  if ($('ex-m-sset')) $('ex-m-sset').hidden = $('btn-sset') ? $('btn-sset').hidden : false;
  if ($('ex-m-teach')) $('ex-m-teach').hidden = $('btn-pset') ? $('btn-pset').hidden : false;
}
function exDiagOpen(open, tab) {
  const m = $('ex-diag');
  if (!m) return;
  if (open) { exMoreOpen(false); exVpopOpen(false); exUpdateSafeBot(); }
  m.hidden = !open;
  if (tab) exDiagTab(tab);
}
function exDiagTab(tab) {
  if (!['tele', 'rec', 'kbd'].includes(tab)) return;
  EX.dtab = tab;
  for (const b of $('ex-diag-tabs').querySelectorAll('button[data-dtab]')) {
    b.setAttribute('aria-selected', String(b.dataset.dtab === tab));
  }
  for (const k of ['tele', 'rec', 'kbd']) {
    const el = $('ex-diag-' + k);
    if (el) el.hidden = k !== tab;
  }
}
function exCatalogue() {
  const raw = VM.models && Array.isArray(VM.models.catalogue) ? VM.models.catalogue : [];
  return raw.filter((e) => e && typeof e === 'object' && typeof e.name === 'string');
}
function exIonNumbers() {
  const num = (v) => (typeof v === 'number' && isFinite(v) ? v : null);
  const ion = VM.ion;
  if (!ion) return null;
  const total = num(ion.total_mb), base = num(ion.base_mb), reserve = num(ion.reserve_mb);
  if (total == null || base == null) return null;
  const res = reserve ?? 0;
  const budget = Math.max(0, total - base - res);
  const used = num(ion.used_mb);
  return { total, base, reserve: res, budget, used };
}
function exInvalidateHeldForModelChange() {
  const hadHeld = typeof detHeld === 'function' && detHeld();
  if (hadHeld && typeof detClearOverlay === 'function') detClearOverlay();
  if (typeof GR !== 'undefined' && GR.sel && (!DET.enabled || hadHeld)) {
    GR.sel = null;
    if (typeof renderGrasp === 'function') renderGrasp();
  }
}
function exModelDesc(kind) {
  if (kind === 'segment') return t('ex.det.seg');
  if (kind === 'detect') return t('ex.det.det');
  if (kind === 'obb') return t('ex.det.obb');
  return '';
}
function exKindLabel(kind) {
  if (kind === 'segment') return t('ex.kind.segment');
  if (kind === 'detect') return t('ex.kind.detect');
  if (kind === 'obb') return t('ex.kind.obb');
  return t('ex.kind.other');
}
function renderExpertVision() {
  const curGen = VM.models && typeof VM.models.generation === 'number' ? VM.models.generation : null;
  if (curGen != null) {
    if (EX.lastGen != null && curGen !== EX.lastGen) exInvalidateHeldForModelChange();
    EX.lastGen = curGen;
  }
  if (S.ui !== 'expert') return;
  const ok = VM.state === 'ok';
  const on = !!DET.avail && svOn();
  const sw = $('ex-vis-on');
  if (sw) {
    sw.setAttribute('aria-checked', String(on));
    sw.disabled = !DET.avail || VM.busy || DET.busy || DET.snap || (ok && !VM.yolo && !on);
  }
  const resDet = (VM.models && VM.models.resident && VM.models.resident.detector) || VM.modelName || null;
  const resDep = (VM.models && VM.models.resident && VM.models.resident.depth) || null;
  const kind = VM.kind || '';
  setText($('ex-mchip-kind'), kind ? exKindLabel(kind) : t('ex.kind.other'));
  setText($('ex-mchip-name'), resDet ? exShortModelName(resDet) : (DET.avail ? t('ex.det.unloaded') : '--'));
  const chip = $('ex-model-chip');
  if (chip) chip.disabled = DET.avail === false && !ok;

  const opTag = $('ex-op-tag');
  if (opTag) {
    opTag.hidden = !S.auth.enforced && !opLocked();
    if (!opTag.hidden) setText(opTag, opLocked() ? t('ex.op_req') : t('ex.op_ok'));
  }

  // Model config warning (missing / mismatched md5)
  const mw = $('ex-model-warn');
  if (mw) {
    const mc = ok ? VM.modelConfig : null;
    const bad = !!(mc && mc.optimal === false);
    mw.hidden = !bad;
    if (bad) {
      const miss = Array.isArray(mc.missing) ? mc.missing.filter(Boolean) : [];
      const mism = Array.isArray(mc.mismatched) ? mc.mismatched.filter(Boolean) : [];
      const parts = [];
      if (miss.length) parts.push(t('sv.model.missing', { list: miss.join(', ') }));
      if (mism.length) parts.push(t('sv.model.mismatch', { list: mism.join(', ') }));
      setText(mw, parts.length ? t('sv.model.warn', { detail: parts.join(t('sv.model.sep')) }) : t('sv.model.warn_gen'));
    }
  }
  const errEl = $('ex-model-err');
  if (errEl) { errEl.hidden = !EX.err; if (EX.err) setText(errEl, EX.err); }

  const cat = exCatalogue();
  const detModels = cat.filter((e) => e.kind !== 'depth');
  const depModels = cat.filter((e) => e.kind === 'depth');
  const ion = exIonNumbers();
  const armBusy = svArmBusy();
  const capOff = S.caps && S.caps.vision && S.caps.vision.model_switch === false;
  const curDetEntry = detModels.find((e) => e.name === resDet) || null;
  const curDepEntry = depModels.find((e) => e.name === resDep) || (resDep ? { name: resDep, ion_mb: null } : null);
  const depMb = resDep ? (curDepEntry && typeof curDepEntry.ion_mb === 'number' && isFinite(curDepEntry.ion_mb) ? curDepEntry.ion_mb : null) : 0;
  const detMb = resDet ? (curDetEntry && typeof curDetEntry.ion_mb === 'number' && isFinite(curDetEntry.ion_mb) ? curDetEntry.ion_mb : null) : 0;

  // Detector variant list
  const list = $('ex-det-list');
  if (list) {
    list.textContent = '';
    if (!detModels.length) {
      const note = document.createElement('div');
      note.className = 'ex-hint';
      note.textContent = resDet ? `${exKindLabel(kind)} · ${resDet} — ${t('ex.det.none')}` : t('ex.det.none');
      list.appendChild(note);
    } else {
      for (const m of detModels) {
        const isCur = m.name === resDet;
        const mMb = typeof m.ion_mb === 'number' && isFinite(m.ion_mb) ? m.ion_mb : null;
        const comb = ion && mMb != null && depMb != null ? mMb + depMb : null;
        const over = !!(ion && comb != null && comb > ion.budget + 1e-6);
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'ex-mrow' + (isCur ? ' cur' : '') + (over ? ' bad' : '');
        btn.setAttribute('role', 'radio');
        btn.setAttribute('aria-checked', String(isCur));
        btn.setAttribute('data-op-mark', '');
        btn.dataset.exDet = m.name;
        btn.disabled = EX.busy || armBusy || capOff || over;
        const radio = document.createElement('i'); radio.className = 'ex-radio';
        const nm = document.createElement('span'); nm.className = 'ex-nm';
        nm.textContent = `${exKindLabel(m.kind)} ${m.kind || ''} · ${m.name}`;
        const mb = document.createElement('span'); mb.className = 'ex-mb';
        mb.textContent = mMb != null ? `${mMb.toFixed(1)} MB` : '--';
        const sub = document.createElement('span'); sub.className = 'ex-sub';
        if (over) {
          sub.textContent = t('ex.det.over', { need: comb.toFixed(1), limit: ion.budget.toFixed(1) });
        } else {
          sub.textContent = (isCur ? t('ex.det.cur') : '') + exModelDesc(m.kind);
        }
        btn.append(radio, nm, mb, sub);
        btn.addEventListener('click', () => {
          if (isCur || EX.busy || over) return;
          exModelSwitch({ detector: m.name });
        });
        list.appendChild(btn);
      }
    }
  }

  // Depth model slot (toggle + variant picker when multiple depth variants exist)
  const depSec = $('ex-depth-sec'), depSep = $('ex-depth-sep'), depList = $('ex-dep-list');
  const firstDep = depModels.find((e) => e.metric !== false) || depModels[0] || null;
  if (depSec && depSep) {
    const showDep = !!(firstDep || resDep);
    depSec.hidden = !showDep;
    depSep.hidden = !showDep;
    if (showDep) {
      const targetDep = curDepEntry || firstDep;
      const targetMb = targetDep && typeof targetDep.ion_mb === 'number' && isFinite(targetDep.ion_mb) ? targetDep.ion_mb : null;
      const depOn = !!resDep;
      const wouldComb = ion && !depOn && targetMb != null && detMb != null ? detMb + targetMb : null;
      const depOver = !!(ion && wouldComb != null && wouldComb > ion.budget + 1e-6);
      setText($('ex-depth-name'), targetDep ? targetDep.name : 'Depth');
      setText($('ex-depth-mb'), targetMb != null ? `${targetMb.toFixed(1)} MB` : '--');
      const dsw = $('ex-depth-sw');
      dsw.setAttribute('aria-checked', String(depOn));
      dsw.disabled = EX.busy || armBusy || capOff || (!depOn && depOver);
      dsw.dataset.targetDep = targetDep ? targetDep.name : '';
      if (depList) {
        depList.textContent = '';
        depList.hidden = depModels.length <= 1;
        if (depModels.length > 1) {
          for (const dm of depModels) {
            const isCur = dm.name === resDep;
            const dmMb = typeof dm.ion_mb === 'number' && isFinite(dm.ion_mb) ? dm.ion_mb : null;
            const dComb = ion && dmMb != null && detMb != null ? detMb + dmMb : null;
            const dOver = !!(ion && dComb != null && dComb > ion.budget + 1e-6);
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'ex-mrow' + (isCur ? ' cur' : '') + (dOver ? ' bad' : '');
            btn.setAttribute('role', 'radio');
            btn.setAttribute('aria-checked', String(isCur));
            btn.setAttribute('data-op-mark', '');
            btn.dataset.exDep = dm.name;
            btn.disabled = EX.busy || armBusy || capOff || dOver;
            const radio = document.createElement('i'); radio.className = 'ex-radio';
            const nm = document.createElement('span'); nm.className = 'ex-nm';
            nm.textContent = dm.name;
            const mb = document.createElement('span'); mb.className = 'ex-mb';
            mb.textContent = dmMb != null ? `${dmMb.toFixed(1)} MB` : '--';
            const sub = document.createElement('span'); sub.className = 'ex-sub';
            sub.textContent = dOver ? t('ex.depth.over', { need: dComb.toFixed(1), limit: ion.budget.toFixed(1) }) : (isCur ? t('ex.det.cur').replace(/·\s*$/, '') : t('ex.depth.note'));
            btn.append(radio, nm, mb, sub);
            btn.addEventListener('click', () => {
              if (isCur || EX.busy || dOver) return;
              exModelSwitch({ depth: dm.name });
            });
            depList.appendChild(btn);
          }
        }
      }
      const dhint = $('ex-depth-hint');
      if (depOver) {
        dhint.className = 'ex-warn-box';
        setText(dhint, t('ex.depth.over', { need: wouldComb.toFixed(1), limit: ion.budget.toFixed(1) }));
      } else {
        dhint.className = 'ex-hint';
        setText(dhint, t('ex.depth.note'));
      }
    }
  }

  // ION memory bar (live provider numbers only; never fabricate 0.0 MB when a resident size is unknown)
  const ionBox = $('ex-ion-box');
  if (ionBox) {
    ionBox.hidden = !ion;
    if (ion) {
      const bothKnown = detMb != null && depMb != null;
      const providerModelsUsed = ion.used != null ? Math.max(0, ion.used - ion.base) : null;
      const usedModels = bothKnown ? (detMb + depMb) : providerModelsUsed;
      const detPct = ion.budget > 0 && detMb != null ? Math.min(100, (detMb / ion.budget) * 100) : (ion.budget > 0 && usedModels != null ? Math.min(100, (usedModels / ion.budget) * 100) : 0);
      const depPct = ion.budget > 0 && bothKnown ? Math.min(100 - detPct, (depMb / ion.budget) * 100) : 0;
      const rem = usedModels != null ? ion.budget - usedModels : null;
      $('ex-ion-bar').classList.toggle('over', rem != null && rem < -1e-6);
      $('ex-ion-det').style.width = detPct.toFixed(1) + '%';
      $('ex-ion-dep').style.width = depPct.toFixed(1) + '%';
      const usedStr = usedModels != null ? usedModels.toFixed(1) : '--';
      const sum = t('ex.ion.sum', { used: usedStr, budget: ion.budget.toFixed(1) });
      const parts = t('ex.ion.parts', { det: detMb != null ? detMb.toFixed(1) : '--', dep: depMb != null ? depMb.toFixed(1) : '--' });
      setText($('ex-ion-summary'), `${sum} · ${parts}`);
      setText($('ex-ion-rem'), rem == null ? '--' : (rem >= 0 ? t('ex.ion.rem', { rem: rem.toFixed(1) }) : t('ex.ion.over', { over: Math.abs(rem).toFixed(1) })));
      setText($('ex-ion-note'), t('ex.ion.note', { total: ion.total.toFixed(1), base: ion.base.toFixed(1), res: ion.reserve.toFixed(1) }));
    }
  }

  // Overlay chips
  const sel = ok ? vmSel() : { aruco: false };
  const dBlock = ok ? depthBlock() : t('vm.tip.na');
  for (const b of $('ex-ovl').querySelectorAll('button[data-ex-ovl]')) {
    const k = b.dataset.exOvl;
    if (k === 'box' || k === 'contour' || k === 'label') {
      b.setAttribute('aria-pressed', String(!!EX_SHOW[k]));
      if (k === 'contour') b.hidden = kind && kind !== 'segment';
    } else if (k === 'aruco') {
      b.setAttribute('aria-pressed', String(!!sel.aruco));
      b.disabled = !ok || VM.busy || DET.snap;
    } else if (k === 'depth') {
      b.setAttribute('aria-pressed', String(!!DEPTH.on));
      b.disabled = !DEPTH.on && !!dBlock;
      b.title = dBlock || t('depth.tip');
    }
  }
  const vsnap = $('ex-vpop-snap');
  if (vsnap) {
    vsnap.disabled = !DET.avail || DET.busy || DET.snap;
    setText(vsnap, t(DET.snap ? 'det.trig.busy' : 'det.trig'));
  }
  const vgset = $('ex-vpop-gset');
  if (vgset) vgset.hidden = !(GR.supported || (S.caps && S.caps.grasp));
}

async function exModelSwitch(patch, retried = false) {
  if (EX.busy) return;
  if (svArmBusy()) {
    EX.err = t('ec.vision_busy');
    renderExpertVision();
    toast(t('ex.det.fail'), 'warn', 'VISION_BUSY');
    return;
  }
  // Preserve requested patch across PIN unlock even if catalogue polling recreates buttons while the dialog is open.
  if (opLocked() && !retried) {
    const ok = await opRequire('/api/v1/vision/models');
    if (!ok) return;
    await vmRefresh();
    const cat = exCatalogue();
    if (patch.detector && !cat.some((e) => e.name === patch.detector)) {
      EX.err = t('ec.model_nf');
      renderExpertVision();
      toast(t('ex.det.fail'), 'err', 'MODEL_NOT_FOUND');
      return;
    }
    if (patch.depth && !cat.some((e) => e.name === patch.depth)) {
      EX.err = t('ec.model_nf');
      renderExpertVision();
      toast(t('ex.det.fail'), 'err', 'MODEL_NOT_FOUND');
      return;
    }
    return exModelSwitch(patch, true);
  }
  EX.busy = true; EX.err = '';
  renderExpertVision();
  let unlock = false;
  try {
    const ctl = new AbortController();
    const tm = setTimeout(() => ctl.abort(), 9000);
    let res, data = null;
    try {
      res = await fetch('/api/v1/vision/models', {
        method: 'POST', signal: ctl.signal, cache: 'no-store',
        headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(patch),
      });
      try { data = await res.json(); } catch (_) {}
    } finally { clearTimeout(tm); }
    const code = (data && data.code) || '';
    if (res.status === 403 && code === 'OPERATOR_REQUIRED' && !retried) {
      unlock = true;
    } else if (!res.ok) {
      const known = { DETECTION_UNAVAILABLE: 'VISION_DOWN' }[code] || code;
      const srvMsg = data && typeof data.message === 'string' && data.message.trim() ? data.message.trim() : '';
      const mb = /needs ([\d.]+) MB, limit ([\d.]+) MB/.exec(srvMsg);
      const localWhy = known === 'ION_BUDGET' && patch.depth && !patch.detector
        ? t('ex.depth.over', { need: mb ? mb[1] : '?', limit: mb ? mb[2] : '?' })
        : (EC_KEYS[known] ? t(EC_KEYS[known]) : '');
      EX.err = srvMsg || localWhy || ('HTTP ' + res.status);
      toast(t('ex.det.fail') + (mb ? ` (${mb[1]} / ${mb[2]} MB)` : ''), 'err', known || String(res.status),
        code === 'OPERATOR_REQUIRED' ? { path: '/api/v1/vision/models' } : null);
    } else {
      if (!data || data.changed !== false) exInvalidateHeldForModelChange();
      const loaded = patch.detector !== undefined
        ? (patch.detector || t('ex.det.unloaded'))
        : (patch.depth ? patch.depth : `${t('ex.models.depth')}: ${t('ex.det.unloaded')}`);
      toast(t('ex.det.ok', { name: loaded }), 'ok');
    }
  } catch (e) {
    const code = e && e.name === 'AbortError' ? 'VISION_TIMEOUT' : 'VISION_DOWN';
    EX.err = t(EC_KEYS[code]);
    toast(t('ex.det.fail'), 'err', code);
  } finally {
    EX.busy = false;
    await vmRefresh();
    renderExpertVision();
    renderSVis();
    gsetTasksRender();
  }
  if (unlock && await opRequire('/api/v1/vision/models')) return exModelSwitch(patch, true);
}

function renderExpertLive(live, dets, detAge) {
  if (S.ui !== 'expert') return;
  const lv = $('ex-live'), lvt = $('ex-live-txt');
  if (lv && lvt) {
    lv.classList.toggle('ok', !!V.playing);
    if (V.playing) {
      const fps = V.fps == null ? '--' : V.fps;
      const ms = V.lat == null ? '--' : Math.round(V.lat);
      setText(lvt, V.mode === 'mjpeg' ? `${t('v.live')} · MJPEG` : `${t('v.live')} · ${fps} fps · ${ms} ms`);
    } else {
      setText(lvt, t('v.chip.connecting'));
    }
  }
  const lk = $('ex-link');
  if (lk) setText(lk, live ? `${S.hz} Hz · ${S.rtt == null ? '--' : Math.round(S.rtt)} ms` : t('no_telemetry'));
  const ph = $('ex-ph-status');
  if (ph && S.connLost) {
    // Phone expert: topbar badge mirrors the lost-connection warning while #lease-status.is-lost exposes #btn-reconnect-now.
    const retryS = reconnectInS();
    setCls(ph, 'sc-status-badge only-expert is-danger');
    setText(ph, retryS != null ? t('lease.t.lost') + ' · ' + t('lease.retry_in', { s: retryS }) : t('lease.conn_lost'));
  } else if (ph) {
    const lsTxt = $('lease-status-txt')?.textContent?.trim() || t('st.Offline');
    const isOwn = $('lease-status')?.classList.contains('is-own');
    const isOther = $('lease-status')?.classList.contains('is-other') || $('lease-status')?.classList.contains('is-released');
    setCls(ph, 'sc-status-badge only-expert ' + (!live ? '' : isOwn ? 'is-ok' : isOther ? 'is-warn' : ''));
    setText(ph, live ? `${lsTxt} · ${S.hz} Hz` : t('st.Offline'));
  }
  // Empty object card state
  const detOn = !!DET.avail && svOn();
  const held = detHeld();
  const n = (dets || []).filter((d) => !d.aruco).length;
  const et = $('ex-empty-title'), eon = $('ex-empty-on'), esnap = $('ex-empty-snap');
  if (et) {
    if (!detOn && !(held && n > 0)) setText(et, t('ex.empty.off'));
    else setText(et, n > 0 && (held || detAge <= 2000) ? t('ex.empty.on_n', { n }) : t('ex.empty.on0'));
  }
  if (eon) { eon.hidden = detOn; eon.disabled = !DET.avail || VM.busy || DET.busy; }
  if (esnap) { esnap.hidden = detOn; esnap.disabled = !DET.avail || DET.busy || DET.snap; }
  exSyncCtx();
  exUpdateSafeBot();
}

function initExpertMode() {
  $('ex-vis-on')?.addEventListener('click', () => simpleVisSet(!svOn()));
  $('ex-empty-on')?.addEventListener('click', () => simpleVisSet(true));
  $('ex-empty-snap')?.addEventListener('click', () => triggerDet());
  $('ex-vpop-snap')?.addEventListener('click', () => triggerDet());
  $('ex-vpop-gset')?.addEventListener('click', () => { exVpopOpen(false); gsetOpen(true); });
  $('ex-model-chip')?.addEventListener('click', () => {
    if (exIsPhone()) exPhoneTab(EX.ptab === 'vision' ? 'move' : 'vision');
    else exVpopOpen(!EX.popOpen);
  });
  $('ex-vpop-x')?.addEventListener('click', () => exVpopOpen(false));
  $('ex-depth-sw')?.addEventListener('click', () => {
    const sw = $('ex-depth-sw');
    if (sw.disabled) return;
    const on = sw.getAttribute('aria-checked') === 'true';
    const target = sw.dataset.targetDep || '';
    if (!on && !target) return;
    exModelSwitch({ depth: on ? null : target });
  });
  $('ex-ovl')?.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-ex-ovl]');
    if (!b || b.disabled) return;
    const k = b.dataset.exOvl;
    if (k === 'box' || k === 'contour' || k === 'label') {
      EX_SHOW[k] = !EX_SHOW[k];
      store('rebot.ex.show', JSON.stringify(EX_SHOW));
      renderExpertVision();
    } else if (k === 'aruco') {
      if (VM.state === 'ok') { const c = vmSel(); setVisionMode(vmModeOf(c.yolo, !c.aruco)); }
    } else if (k === 'depth') {
      vmToggle('depth');
    }
  });
  document.addEventListener('pointerdown', (e) => {
    if (!EX.popOpen || exIsPhone()) return;
    if (e.target.closest && e.target.closest('#ex-vpop, #ex-model-chip')) return;
    exVpopOpen(false);
  }, true);

  $('ex-ptabs')?.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-ptab]');
    if (b) exPhoneTab(b.dataset.ptab);
  });
  $('ex-speed-btn')?.addEventListener('click', () => setSpeed((S.speedIdx + 1) % CFG.speeds.length));
  for (const b of document.querySelectorAll('.ex-pre-btn[data-ex-preset]')) {
    b.addEventListener('click', () => runPreset(b.dataset.exPreset));
  }
  $('ex-ph-open-teach')?.addEventListener('click', () => $('btn-pset')?.click());
  $('ex-ph-open-diag')?.addEventListener('click', () => exDiagOpen(true, 'tele'));
  $('ex-ph-open-gset')?.addEventListener('click', () => gsetOpen(true));

  $('btn-ex-more')?.addEventListener('click', () => exMoreOpen($('ex-more').hidden));
  $('ex-more-close')?.addEventListener('click', () => exMoreOpen(false));
  $('ex-more')?.addEventListener('click', (e) => { if (e.target === e.currentTarget) exMoreOpen(false); });
  $('ex-m-diag')?.addEventListener('click', () => exDiagOpen(true, 'tele'));
  $('ex-m-task')?.addEventListener('click', () => { exMoreOpen(false); $('btn-task')?.click(); });
  $('ex-m-teach')?.addEventListener('click', () => { exMoreOpen(false); $('btn-pset')?.click(); });
  $('ex-m-gset')?.addEventListener('click', () => { exMoreOpen(false); gsetOpen(true); });
  $('ex-m-calib')?.addEventListener('click', () => { exMoreOpen(false); $('btn-calib')?.click(); });
  $('ex-m-sset')?.addEventListener('click', () => { exMoreOpen(false); $('btn-sset')?.click(); });
  $('ex-m-simple')?.addEventListener('click', () => { exMoreOpen(false); S.uiUserSet = true; setUi('simple'); });
  $('ex-m-fs')?.addEventListener('click', () => { exMoreOpen(false); $('btn-fs')?.click(); });

  $('ex-diag-close')?.addEventListener('click', () => exDiagOpen(false));
  $('ex-diag')?.addEventListener('click', (e) => { if (e.target === e.currentTarget) exDiagOpen(false); });
  $('ex-diag-tabs')?.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-dtab]');
    if (b) exDiagTab(b.dataset.dtab);
  });

  EX.phoneMq.addEventListener('change', () => { if (S.ui === 'expert') exPlaceDom(); });
  window.addEventListener('resize', () => { if (exIsPhone()) exUpdateSafeBot(); });
  i18nHooks.push(() => {
    renderExpertVision();
    const exSp = $('ex-speed-btn');
    if (exSp) exSp.textContent = t('ex.speed.btn', { p: Math.round(CFG.speeds[S.speedIdx] * 100) });
  });
  exPlaceDom();
}

/* Outline fallback for older rebot-vision builds whose /results WebSocket carries boxes only; when WS frames lack
   `contours`, read `contour` from GET /api/v1/detection/latest for the selected object at most every 500 ms. */
const CT = { busy: false, at: 0 };
async function graspContourPoll() {
  const s = GR.sel;
  if (!s || CT.busy || !graspDetOn() || VM.kind !== 'segment' || performance.now() - CT.at < 500) return;
  if ((S.detections || []).some((d) => d.contour)) return; // the stream carries outlines itself
  CT.busy = true; CT.at = performance.now();
  try {
    const dets = parseDetections(await detLatest(''), videoSrcRes()).filter((d) => d.contour && d.label === s.label);
    let best = null, bi = 0.3;
    for (const d of dets) { const i = graspIou([d.x0, d.y0, d.x1, d.y1], s.box); if (i > bi) { bi = i; best = d; } }
    if (GR.sel === s) {
      if (best) { s.contour = best.contour; s.contourBox = [best.x0, best.y0, best.x1, best.y1]; s.contourAt = performance.now(); }
      else if (performance.now() - (s.contourAt || 0) > 2000) s.contour = null;
    }
  } catch (_) {} finally { CT.busy = false; }
}
// outline in canvas pixels; shifted by how far the tracked box moved since the outline was read
function contourPath(g, rect, pts, shift) {
  const [dx, dy] = shift || [0, 0];
  g.beginPath();
  pts.forEach(([u, v], i) => { const x = rect.x + (u + dx) * rect.w, y = rect.y + (v + dy) * rect.h; if (i) g.lineTo(x, y); else g.moveTo(x, y); });
  g.closePath();
}
function selContour(s) {
  if (!s || !s.contour || performance.now() - (s.contourAt || 0) > 3000) return null;
  const c = (b) => [(b[0] + b[2]) / 2, (b[1] + b[3]) / 2];
  const a = c(s.contourBox), b = c(s.box);
  return { pts: s.contour, shift: [b[0] - a[0], b[1] - a[1]] };
}

/* Object card placement (simple, landscape): beside the selected object (right, else left, below, above), clear of the
   safety capsule, the status island, the presets and the thumb controls; else the band between the thumb controls.
   The size measurement card opens under (or over) it. */
const OC_MQ = window.matchMedia('(orientation: landscape) and (max-height: 1200px)');
const OC = { x: null, y: null };
function ocObstacles() {
  const ids = ['safety', 'view-switch', 'pose-card', 'pad-sxy', 'btn-pose', 'pad-srot', 'pad-sz', 'pad-sgrip', 's-grip-readout', 'btn-vis', 's-objhint'];
  const els = ids.map((id) => $(id)).concat([document.querySelector('.topbar'), document.querySelector('.presets')]);
  return els.map((el) => { if (!el) return null; const r = el.getBoundingClientRect(); return r.width && r.height ? r : null; }).filter(Boolean);
}
function placeObjCard(rect) {
  const bar = $('grasp-bar');
  const on = S.ui === 'simple' && OC_MQ.matches && !bar.hidden;
  // layout reads are not free on a tablet: at most every 120 ms while the card is up (it follows a slow object)
  const now = performance.now();
  if (on && bar.classList.contains('placed') && now - (OC.at || 0) < 120) return;
  OC.at = now;
  if (!on) {
    if (bar.classList.contains('placed')) { bar.classList.remove('placed', 'flip', 'below', 'above'); OC.x = OC.y = null; }
    placeMeasCard(null);
    return;
  }
  const W = innerWidth, H = innerHeight, cw = bar.offsetWidth, ch = bar.offsetHeight, m = 8, gap = 14;
  const obs = ocObstacles();
  const free = (x, y) => x >= m && y >= m && x + cw <= W - m && y + ch <= H - m
    && !obs.some((r) => !(x + cw <= r.left || r.right <= x || y + ch <= r.top || r.bottom <= y));
  const s = GR.sel, vb = $('video-box').getBoundingClientRect();
  const cands = [];
  if (s && rect) {
    const l = vb.left + rect.x + s.box[0] * rect.w, tp = vb.top + rect.y + s.box[1] * rect.h;
    const r = vb.left + rect.x + s.box[2] * rect.w, b = vb.top + rect.y + s.box[3] * rect.h, cx = (l + r) / 2;
    const yS = Math.min(Math.max(tp + 10, 56), H - m - ch);
    cands.push(['', r + gap, yS], ['flip', l - gap - cw, yS], ['below', cx - cw / 2, b + 12], ['above', cx - cw / 2, tp - 12 - ch]);
    // nudge sideways placements up or down to clear an obstacle
    for (const dy of [-80, 80, -160, 160]) cands.push(['', r + gap, yS + dy], ['flip', l - gap - cw, yS + dy]);
  }
  const bl = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--band-l')) || 176;
  const br = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--band-r')) || 176;
  const hudH = parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--hud-h')) || 32;
  const fallback = ['below', (bl + W - br) / 2 - cw / 2, H - 16 - hudH - 8 - ch];
  const pick = cands.find(([, x, y]) => free(x, y)) || fallback;
  let [cls, x, y] = pick;
  x = Math.round(Math.min(Math.max(x, m), W - m - cw)); y = Math.round(Math.min(Math.max(y, m), H - m - ch));
  if (OC.x == null || Math.abs(OC.x - x) > 3 || Math.abs(OC.y - y) > 3 || OC.cls !== cls) {
    OC.x = x; OC.y = y; OC.cls = cls;
    bar.style.setProperty('--oc-x', x + 'px'); bar.style.setProperty('--oc-y', y + 'px');
    bar.classList.add('placed');
    bar.classList.toggle('flip', cls === 'flip'); bar.classList.toggle('below', cls === 'below'); bar.classList.toggle('above', cls === 'above');
  }
  if (s && rect && !cls) {
    const cy = vb.top + rect.y + (s.box[1] + s.box[3]) / 2 * rect.h;
    bar.style.setProperty('--oc-ay', Math.round(Math.min(Math.max(cy - y - 6, 14), ch - 26)) + 'px');
  } else if (s && rect && cls === 'flip') {
    const cy = vb.top + rect.y + (s.box[1] + s.box[3]) / 2 * rect.h;
    bar.style.setProperty('--oc-ay', Math.round(Math.min(Math.max(cy - y - 6, 14), ch - 26)) + 'px');
  }
  placeMeasCard({ x, y, w: cw, h: ch });
}
/* stacked layouts: the slot under the video holds the hint, the object card and the measurement card (page flow, the video
   stays clear); the recognition panel is a bottom sheet whose top edge stays below the safety card (svSheetPlace) */
function placeSlot() {
  if (S.ui === 'expert') {
    document.body.classList.remove('oc-slot');
    if (typeof exPlaceDom === 'function') exPlaceDom();
    return;
  }
  const slot = S.ui === 'simple' && !OC_MQ.matches;
  document.body.classList.toggle('oc-slot', slot);
  const host = slot ? $('s-slot') : $('video-box');
  for (const id of ['s-objhint', 'grasp-bar', 'meas-card']) { const el = $(id); if (el.parentElement !== host) host.appendChild(el); }
  const dp = $('depth-panel');
  if (dp && dp.parentElement !== $('video-box')) $('video-box').appendChild(dp);
  svSheetPlace();
}
function svSheetPlace() {
  const body = document.body;
  if (!body.classList.contains('oc-slot') || (!body.classList.contains('vis-open') && $('meas-card').hidden)) return;
  const need = 260; // room for the panel's switch, kind and display rows
  let sr = $('safety').getBoundingClientRect();
  if (sr.height && sr.bottom > innerHeight - need) {
    if (SV.scrollBack == null) SV.scrollBack = window.scrollY; // where the page was before the sheet scrolled it
    window.scrollBy(0, sr.bottom - (innerHeight - need)); sr = $('safety').getBoundingClientRect();
  }
  document.documentElement.style.setProperty('--sv-top', Math.round(Math.max(56, sr.height ? sr.bottom + 8 : innerHeight / 2)) + 'px');
}
addEventListener('scroll', () => requestAnimationFrame(svSheetPlace), { passive: true });
OC_MQ.addEventListener('change', placeSlot);
addEventListener('resize', () => { if (document.body.classList.contains('oc-slot')) placeSlot(); });
function placeMeasCard(card) {
  const mc = $('meas-card');
  if (!mc) return;
  if (!card || mc.hidden || document.body.classList.contains('oc-slot')) { mc.classList.remove('placed'); return; }
  // under the object card, else over it, else beside it; clear of the thumb controls and the safety capsule
  const W = innerWidth, H = innerHeight, m = 8;
  mc.style.setProperty('--ms-w', card.w + 'px');
  mc.style.setProperty('--ms-h', (H - 2 * m) + 'px');
  mc.classList.add('placed');
  const w = mc.offsetWidth, h = Math.min(mc.scrollHeight, H - 2 * m);
  const obs = ocObstacles();
  const free = (x, y) => x >= m && y >= m && x + w <= W - m && y + h <= H - m
    && !obs.some((r) => !(x + w <= r.left || r.right <= x || y + h <= r.top || r.bottom <= y));
  const cands = [[card.x, card.y + card.h + 8], [card.x, card.y - 8 - h], [card.x - 8 - w, card.y], [card.x + card.w + 8, card.y],
    [card.x - 8 - w, card.y + card.h - h], [card.x + card.w + 8, card.y + card.h - h]];
  const [x, y] = cands.find(([cx, cy]) => free(cx, cy)) || cands[0];
  mc.style.setProperty('--ms-x', Math.round(Math.min(Math.max(x, m), W - m - w)) + 'px');
  mc.style.setProperty('--ms-y', Math.round(Math.min(Math.max(y, m), H - m - h)) + 'px');
  mc.style.setProperty('--ms-h', Math.round(h) + 'px');
}

/* Grasp tasks (GET /api/v1/grasp/setup setup.tasks): "Default" (no task) plus every stored task. Simple mode lists only the
   tasks the loaded detector serves without a switch (requires.model_kind = the loaded kind, models.detector = the loaded
   model or unset). The choice stays in this browser (rebot.grasp.task) and goes into the grasp request as `task`. */
GR.task = store('rebot.grasp.task') || null;
// Grasp approach (`strategy` in the grasp request): legacy (top-down, default) or adaptive (may choose an
// angled approach; in 1.6.0 it can refuse wide objects that top-down grasps, see the 1.6.1 plan).
GR.strategy = store('rebot.grasp.strategy') === 'adaptive' ? 'adaptive' : 'legacy';
function graspTasks() { return (GR.setup && GR.setup.tasks && typeof GR.setup.tasks === 'object') ? GR.setup.tasks : {}; }
function taskUsable(tk) {
  if (S.ui !== 'simple' || !tk) return true;
  const kind = tk.requires && tk.requires.model_kind;
  if (kind && kind !== VM.kind) return false;
  const det = tk.models && tk.models.detector;
  const resident = VM.models && VM.models.resident ? VM.models.resident.detector : null;
  return det === undefined || det === resident;
}
function graspTaskCur() {
  const tasks = graspTasks();
  return GR.task && tasks[GR.task] && taskUsable(tasks[GR.task]) ? GR.task : null;
}
function gsetTasksRender() {
  const box = $('gset-tasks');
  if (!box) return;
  const tasks = graspTasks(), names = Object.keys(tasks);
  box.hidden = !GR.setup;
  const list = $('gset-task-list');
  list.textContent = '';
  const cur = graspTaskCur();
  const add = (name, title, desc) => {
    const b = document.createElement('button');
    b.type = 'button'; b.className = 'gset-task'; b.setAttribute('role', 'radio'); b.setAttribute('aria-checked', String(cur === name));
    const d = document.createElement('div');
    d.append(cwEl('b', '', title), cwEl('small', '', desc));
    b.appendChild(d);
    b.addEventListener('click', () => { GR.task = name; store('rebot.grasp.task', name || ''); gsetTasksRender(); renderGrasp(); });
    list.appendChild(b);
  };
  add(null, t('oc.task.def'), t('gset.task.def.d'));
  let hidden = 0;
  for (const n of names) {
    if (!taskUsable(tasks[n])) { hidden++; continue; }
    let desc = tasks[n].description || '';
    if (S.ui === 'expert') {
      const reqDet = tasks[n].models && tasks[n].models.detector;
      const reqKind = tasks[n].requires && tasks[n].requires.model_kind;
      const resident = VM.models && VM.models.resident ? VM.models.resident.detector : null;
      const willLoad = (reqDet && reqDet !== resident) ? reqDet : (reqKind && reqKind !== VM.kind) ? reqKind : null;
      if (willLoad) desc = (desc ? desc + ' · ' : '') + t('ex.task.loads', { m: willLoad });
    }
    add(n, n, desc);
  }
  const note = $('gset-tasks-note');
  note.hidden = !hidden;
  setText(note, t('gset.tasks.hidden', { n: hidden }));
}

