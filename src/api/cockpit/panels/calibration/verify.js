// ---- self-service verification (T20): fingertip sticker and hover acceptance (fine mode, armd calibration_verify)
/* After the fine solve: 'tip' measures the fingertip sticker in camera JPEGs (same-origin /api/v1/camera/snapshot,
   decoded into a canvas of its own) with the candidate intrinsics; 'hover' plans tool-down hovers over board corners
   with the candidate model, armd runs them (action calib_hover, a job on the device that survives a dropped page),
   the page evaluates the recorded marker corners. Review then applies the candidate with the fingertip, the acceptance
   and, when it passed with a known tool, the hover table height. */
const cwVerifyOn = () => cwFine() && !!(S.caps && S.caps.calibration_verify);
const cwExpert = () => !document.body.classList.contains('ui-simple');
const CWV = { tip: null, tipBusy: false, roi: null, frames: 0, hover: null, plan: null, job: null, poll: null, eval: null, confirm: null, confirmJob: null, phase: '', fp: null, xfp: null, edge: null, toolId: null, base: null };
/* The sticker spans both closed fingers, so the fingertip is measured with the gripper closed and tip_cam holds for
   that opening only: the measurement needs the gripper closed, the hovers (plan, start, resume, confirm) the opening
   of the measurement. */
const CWV_GRIP_CLOSED = 0.15, CWV_GRIP_TOL = 0.05;
const cwvGrip = () => { const g = S.obs && S.obs.gripper; return g && Number.isFinite(g.fraction) ? g.fraction : null; };
function cwvGripOff() {
  const now = cwvGrip(), then = CWV.tool ? CWV.tool.gripper_opening : null;
  return now != null && then != null && Math.abs(now - then) > CWV_GRIP_TOL ? { now, then } : null;
}
function cwvGripCheck() {
  const off = cwvGripOff();
  if (off) throw Object.assign(new Error(t('cw.hov.grip', { now: off.now.toFixed(2), then: off.then.toFixed(2) })), { code: 'GRIPPER_OPENING' });
}
async function cwvCloseGripper() {
  if (CWZ.busy || CWV.tipBusy) return;
  CWZ.busy = true; cwRender();
  try {
    await commandRPC('/api/v1/action/gripper', { command: 'close' }, true, 5000);
    const t0 = Date.now();
    while (Date.now() - t0 < 6000) { const g = cwvGrip(); if (g == null || g <= CWV_GRIP_CLOSED) break; await cwSleep(100); }
    const g = cwvGrip();
    CWZ.status = '';
    if (g != null && g > CWV_GRIP_CLOSED) cwSetStatus(t('cw.tip.open', { f: g.toFixed(2) }), 'warn');
  } catch (e) { if (isCancelledErr(e)) { cwSetStatus(''); return; } cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'); }
  finally { CWZ.busy = false; cwRender(); }
}
function cwvReset() { Object.assign(CWV, { tip: null, tipBusy: false, roi: null, frames: 0, plan: null, job: null, eval: null, confirm: null, confirmJob: null, phase: '', fp: null, xfp: null, edge: null, toolId: null, base: null }); clearInterval(CWV.poll); CWV.poll = null; }
// the candidate: intrinsics, X, kinematic parameter vectors (candidate and the model armd runs now)
function cwvCand() {
  const CW = CWZ.CW, r = CWZ.solve, ap = r.apply;
  const K = r.final.K, X = r.final.X;
  const kCand = ap.kinematics && ap.kinematics.joints ? CW.kinVector(ap.kinematics.joints) : new Array(42).fill(0);
  const lj = CALM.correctionJoints();
  const kLive = lj ? CW.kinVector(lj) : new Array(42).fill(0);
  return { K, X, kCand, kLive, ch: CW.kinChain(CALM.CHAIN) };
}
function cwPrintSticker() {
  const svg = CWZ.CW.stickerSvg();
  const w = window.open('', '_blank');
  if (!w) { toast('popup blocked', 'warn'); return; }
  w.document.write('<!doctype html><title>reBot fingertip sticker</title><style>@page{margin:0}body{margin:0}</style>' + svg);
  w.document.close(); w.focus(); setTimeout(() => w.print(), 300);
}
// FNV-1a of the JPEG bytes (repeated snapshots are dropped)
function cwvKey(buf) { let h = 0x811c9dc5; const b = new Uint8Array(buf); for (let i = 0; i < b.length; i += 7) { h ^= b[i]; h = Math.imul(h, 0x01000193); } return (h >>> 0).toString(16) + ':' + b.length; }
async function cwvSnapshot() {
  const r = await fetch('/api/v1/camera/snapshot', { cache: 'no-store' });
  if (!r.ok) throw Object.assign(new Error('snapshot ' + r.status), { code: 'SNAPSHOT' });
  const buf = await r.arrayBuffer();
  const bmp = await createImageBitmap(new Blob([buf], { type: 'image/jpeg' }));
  const c = (CWV.canvas ||= document.createElement('canvas'));
  c.width = bmp.width; c.height = bmp.height;
  const ctx = c.getContext('2d', { willReadFrequently: true });
  ctx.drawImage(bmp, 0, 0);
  const img = ctx.getImageData(0, 0, c.width, c.height);
  return { key: cwvKey(buf), gray: CWZ.CW.grayContent(img.data, c.width, c.height), w: c.width, h: c.height };
}
// ROI crop of the last frame (sticker-not-found guidance): data URL with the blobs found
function cwvRoiImage(gray, det) {
  const R = CWZ.CW.STICKER_ROI, w = R.x1 - R.x0, h = R.y1 - R.y0, c = document.createElement('canvas');
  c.width = w * 2; c.height = h * 2;
  const ctx = c.getContext('2d'), id = ctx.createImageData(w, h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { const v = gray[(R.y0 + y) * 640 + R.x0 + x], i = (y * w + x) * 4; id.data[i] = id.data[i + 1] = id.data[i + 2] = v; id.data[i + 3] = 255; }
  const t = document.createElement('canvas'); t.width = w; t.height = h; t.getContext('2d').putImageData(id, 0, 0);
  ctx.imageSmoothingEnabled = false; ctx.drawImage(t, 0, 0, w * 2, h * 2);
  ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2;
  for (const b of (det && det.blobs) || []) { ctx.beginPath(); ctx.arc((b.u - R.x0) * 2, (b.v - R.y0) * 2, 8, 0, 2 * Math.PI); ctx.stroke(); }
  return c.toDataURL('image/png');
}
async function cwvTipMeasure() {
  if (CWV.tipBusy || CWZ.busy) return;
  const CW = CWZ.CW, o = S.obs || {};
  if (!o.is_torqued) { cwSetStatus(t('cw.tip.torque'), 'warn'); return; }
  const g0 = cwvGrip();
  if (g0 != null && g0 > CWV_GRIP_CLOSED) { cwSetStatus(t('cw.tip.open', { f: g0.toFixed(2) }), 'warn'); cwRender(); return; }
  CWV.tipBusy = true; CWV.tip = null; CWV.roi = null; CWV.frames = 0; cwSetStatus(t('cw.tip.busy'));
  const want = 16, frames = [];
  let last = null, lastDet = null, tries = 0;
  let capToolId = null, capToolDigest = null;
  try {
    const r0 = await fetch('/api/v1/calibration', { cache: 'no-store' });
    if (!r0.ok) throw new Error('HTTP ' + r0.status);
    const st0 = await r0.json();
    if (!st0 || !st0.active_tool || !st0.active_tool.id) throw new Error('active_tool unavailable');
    S.activeTool = st0.active_tool;
    capToolId = !st0.active_tool.is_stock
      ? st0.active_tool.id
      : ((st0.calibration_tool && st0.calibration_tool.tool_id) || st0.active_tool.id);
    capToolDigest = st0.active_tool.config_digest || null;
  } catch (e) {
    CWV.tipBusy = false;
    cwSetStatus(t('cw.err', { m: 'active tool check failed: ' + (e && e.message ? e.message : e) }), 'warn');
    cwRender();
    return;
  }
  try {
    const pos0 = o.positions.slice();
    while (frames.length < want && tries++ < 60) {
      const f = await cwvSnapshot();
      const now = S.obs && S.obs.positions;
      if (!now || pos0.slice(0, 6).some((v, i) => Math.abs(now[i] - v) > 0.003) || !(S.obs.is_torqued)) throw Object.assign(new Error(t('cw.tip.moved')), { code: 'MOVED' });
      last = f;
      if (CW.dedupeFrames([...frames, { key: f.key }]).length === frames.length) continue;
      const d = CW.detectStickerDots(f.gray);
      lastDet = d;
      if (!d.ok) break;
      frames.push({ key: f.key, points: d.points });
      CWV.frames = frames.length; cwRender();
    }
    if (frames.length < 8) {
      CWV.roi = last ? cwvRoiImage(last.gray, lastDet) : null;
      CWV.tip = { ok: false, why: lastDet && !lastDet.ok ? 'dots' : 'frames', blobs: lastDet ? lastDet.blobs.length : 0 };
      cwSetStatus(t(CWV.tip.why === 'dots' ? 'cw.tip.notfound' : 'cw.tip.few', { n: frames.length, b: CWV.tip.blobs }), 'warn');
      return;
    }
    const { K } = cwvCand();
    const res = CW.measureTip(frames.map((f) => f.points), K, { gates: { ...CW.STICKER_GATES, minFrames: 8 } });
    res.opening = o.gripper && Number.isFinite(o.gripper.fraction) ? +o.gripper.fraction.toFixed(3) : null;
    CWV.tip = res;
    if (!res.ok) { CWV.roi = cwvRoiImage(last.gray, lastDet); cwSetStatus(t('cw.tip.gates'), 'warn'); return; }
    // the candidate's fingerprints from armd (dry runs): x_fingerprint binds the tip, fingerprint the hovers
    const base = { ...CWZ.solve.apply, dry_run: true };
    const d1 = await commandRPC('/api/v1/calibration/apply', base, true, 15000);
    CWV.xfp = d1.after && d1.after.x_fingerprint;
    const toolId = capToolId;
    const toolDigest = capToolDigest;
    const tool = { tool_id: toolId, ...(toolDigest ? { tool_config_digest: toolDigest } : {}), tip_cam_m: res.tip_cam_m.map((v) => +v.toFixed(6)), gripper_opening: res.opening, x_fingerprint: CWV.xfp,
      fit: { frames: res.frames, reproj_px: +res.reprojPx.toFixed(3), gap_px: +res.gapPx.toFixed(3), std_mm: +res.stdMm.toFixed(3) } };
    const d2 = await commandRPC('/api/v1/calibration/apply', { ...base, tool }, true, 15000);
    CWV.fp = d2.after && d2.after.fingerprint;
    CWV.edge = d2.after && d2.after.tool ? d2.after.tool.edge_to_low_m : null;
    CWV.tool = tool; CWV.toolId = toolId;
    cwSetStatus(t('cw.tip.ok', { mm: (res.tip_cam_m.map((v) => (v * 1000).toFixed(1))).join(', ') }), 'ok');
  } catch (e) { if (isCancelledErr(e)) { cwSetStatus(''); return; } cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'); }
  finally { CWV.tipBusy = false; cwRender(); }
}
// re-observe the stationary board at the current pose with the candidate camera -> T_base_board
async function cwvBoardNow() {
  const CW = CWZ.CW, v = await cwCapture();
  if (v.err) throw Object.assign(new Error(t('cw.skip.' + (v.err === 'moved' ? 'moved' : 'det'))), { code: 'BOARD' });
  const { K, X, kCand, ch } = cwvCand();
  const fit = CW.fitBoardPose(v.markers, cwBoard(), K);
  if (!fit || v.markers.length < 4) throw Object.assign(new Error(t('cw.hov.board', { n: v.markers.length })), { code: 'BOARD' });
  return { Tbb: CW.mul(CW.mul(CW.kinPose(ch, cwModelQ(v.joints), kCand), X), fit.T), markers: v.markers.length, q: cwModelQ(v.joints) };
}
async function cwvPlan() {
  if (CWZ.busy) return;
  const CW = CWZ.CW;
  CWZ.busy = true; cwSetStatus(t('cw.hov.planning'));
  try {
    cwvGripCheck();
    const b = await cwvBoardNow();
    const { K, X, kCand, kLive, ch } = cwvCand();
    const { lo, hi } = modelLimits6();
    const plan = CW.planHover({ Tbb: b.Tbb, X, tipCam: CWV.tool.tip_cam_m, edgeToLow: CWV.edge, ch, kCand, kLive, q0: b.q, K, board: cwBoard(), lo, hi, physicalContactEnd: S.physicalContactEnd, thickness: CWZ.thickMm / 1000 });
    CWV.plan = { ...plan, Tbb: b.Tbb, at: Date.now() };
    CWV.base = b;
    cwSetStatus(plan.ok ? t('cw.hov.planned', { n: plan.points.length }) : t('cw.hov.why.' + plan.why), plan.ok ? 'ok' : 'warn');
  } catch (e) { cwSetStatus(t('cw.err', { m: e.message }), 'warn'); }
  finally { CWZ.busy = false; cwRender(); }
}
function cwvBody(points) {
  return { candidate_fingerprint: CWV.fp, rpy_rad: points[0].rpy_rad, approach_speed_mps: 0.04, descent_speed_mps: 0.01, settle_s: 2.5, frames: 3, discard: 2,
    points: points.map((p) => ({ id: p.id, approach_m: p.approach_m.map((v) => +v.toFixed(5)), hover_m: p.hover_m.map((v) => +v.toFixed(5)), rpy_rad: p.rpy_rad })) };
}
async function cwvStart(confirm = false, resume = null) {
  if (CWZ.busy) return;
  const snap = requireLeaseSyncOrPrompt((e) => cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'));
  if (!snap) return;
  CWZ.controlSnap = snap; CWZ.lease = snap.leaseGen; CWZ.epoch = snap.epoch;
  CWZ.busy = true; CWV.phase = confirm ? 'confirm' : 'hover';
  try {
    cwvGripCheck();
    let body;
    if (resume) {
      // resume: check candidate fingerprint BEFORE moving or torquing on, then return to view pose and re-check board
      if (CWV.job && CWV.job.candidate_fingerprint && CWV.job.candidate_fingerprint !== CWV.fp) {
        throw Object.assign(new Error(t('cw.hov.fpchanged')), { code: 'FINGERPRINT' });
      }
      const dr = await commandRPC('/api/v1/calibration/apply', { ...CWZ.solve.apply, tool: CWV.tool, dry_run: true }, true, 15000, { controlSnap: snap });
      if (!isControlSnapValid(snap)) throw cancelledError();
      if (!dr.after || dr.after.fingerprint !== CWV.fp) throw Object.assign(new Error(t('cw.hov.fpchanged')), { code: 'FINGERPRINT' });
      if (!(S.obs && S.obs.is_torqued)) {
        await commandRPC('/api/v1/torque', { enable: true }, true, 5000, { controlSnap: snap });
        if (!isControlSnapValid(snap)) throw cancelledError();
        await cwSleep(800);
        if (!isControlSnapValid(snap)) throw cancelledError();
      }
      const { kLive, ch } = cwvCand();
      CWV.phase = 'view';
      await cwMove(CWZ.CW.kinPose(ch, CWV.base.q, kLive));
      if (!isControlSnapValid(snap)) throw cancelledError();
      await cwSettle();
      if (!isControlSnapValid(snap)) throw cancelledError();
      CWV.phase = 'hover';
      const b = await cwvBoardNow();
      if (!isControlSnapValid(snap)) throw cancelledError();
      const d = Math.hypot(...[0, 1, 2].map((k) => b.Tbb[k][3] - CWV.plan.Tbb[k][3]));
      if (d > 0.003) throw Object.assign(new Error(t('cw.hov.moved', { mm: (d * 1000).toFixed(1) })), { code: 'BOARD_MOVED' });
      body = { resume, candidate_fingerprint: CWV.fp };
    } else if (confirm) {
      if (!Array.isArray(S.physicalContactEnd) || S.physicalContactEnd.length !== 3) {
        throw Object.assign(new Error(t('cw.hov.noedge')), { code: 'CONTACT_UNKNOWN' });
      }
      const { K, X, kCand, kLive, ch } = cwvCand(), { lo, hi } = modelLimits6();
      const cp = CWZ.CW.planHover({ Tbb: CWV.plan.Tbb, X, tipCam: CWV.tool.tip_cam_m, edgeToLow: CWV.edge, ch, kCand, kLive, q0: CWV.base.q, K, board: cwBoard(), lo, hi, tableZ: CWV.eval.table_z_m, physicalContactEnd: S.physicalContactEnd, thickness: CWZ.thickMm / 1000, plan: { ...CWZ.CW.HOVER_PLAN, count: 2, minCount: 1 } });
      if (!cp.points.length) throw Object.assign(new Error(t('cw.hov.why.reach')), { code: 'PLAN' });
      CWV.confirmPlan = cp;
      body = cwvBody(cp.points);
    } else body = cwvBody(CWV.plan.points);
    if (!isControlSnapValid(snap)) throw cancelledError();
    cwvGripCheck();
    const ack = await commandRPC(CALIB_HOVER_PATH, body, true, 8000, { controlSnap: snap });
    if (!isControlSnapValid(snap)) throw cancelledError();
    const id = ack.job_id;
    if (confirm) { CWV.confirmJob = { id, state: 'running' }; CWV.confirm = null; } else { CWV.job = { id, state: 'running' }; CWV.eval = null; CWV.confirm = null; }
    cwvPoll();
  } catch (e) { CWV.phase = ''; if (isCancelledErr(e)) { cwSetStatus(''); return; } cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'); }
  finally { CWZ.busy = false; CWZ.controlSnap = null; cwRender(); }
}
const CALIB_HOVER_PATH = '/api/v1/action/calib_hover';
// the job lives on the device: poll it (a lost WebSocket only pauses the page; armd ends or parks on its own)
function cwvPoll() {
  clearInterval(CWV.poll);
  CWV.poll = setInterval(async () => {
    const which = CWV.phase === 'confirm' ? 'confirmJob' : 'job';
    const j = CWV[which];
    if (!j || !j.id) { clearInterval(CWV.poll); CWV.poll = null; return; }
    try {
      const r = await fetch('/api/v1/calibration/hover/' + encodeURIComponent(j.id), { cache: 'no-store' });
      if (!r.ok) return;
      const job = await r.json();
      CWV[which] = job;
      if (job.state !== 'running') { clearInterval(CWV.poll); CWV.poll = null; CWV.phase = ''; cwvEvaluate(which); }
      if (CWZ.step === 'hover') cwRender();
    } catch (_) { /* offline: keep polling */ }
  }, 600);
}
function cwvEvaluate(which) {
  const CW = CWZ.CW, job = CWV[which];
  const { K, X, kCand, ch } = cwvCand(), board = cwBoard();
  const plan = which === 'confirmJob' ? CWV.confirmPlan : CWV.plan;
  const pts = job.points.map((p) => {
    const pl = plan.points.find((x) => x.id === p.id);
    if (p.state !== 'done') return { id: p.id, ok: false, skipped: p.state, code: p.code };
    const q = cwModelQ(p.settled_joints || p.observations[p.observations.length - 1].joints);
    const r = CW.hoverPoint({ observations: p.observations, board, K, tipCam: CWV.tool.tip_cam_m, edgeToLow: CWV.edge, target: pl && pl.corner, Tend: CW.kinPose(ch, q, kCand), X, physicalContactEnd: S.physicalContactEnd });
    return { id: p.id, ...r };
  });
  if (which === 'confirmJob') { CWV.confirm = { ...CW.confirmHovers(pts, CW.HOVER_PLAN.clearanceM), pts }; cwSetStatus(t(CWV.confirm.ok ? 'cw.hov.conf.ok' : 'cw.hov.conf.bad', { mm: cwFmt(CWV.confirm.worst_m * 1000, 1) }), CWV.confirm.ok ? 'ok' : 'warn'); return; }
  CWV.pts = pts;
  CWV.eval = CW.evaluateHovers(pts, { edgeToLow: CWV.edge, physicalContactEnd: S.physicalContactEnd, thickness: CWZ.thickMm / 1000 });
  const ended = job.state === 'done' ? '' : t('cw.hov.ended', { s: job.state, c: job.code || '' });
  cwSetStatus(ended || t(CWV.eval.ok ? 'cw.hov.pass' : 'cw.hov.fail'), !ended && CWV.eval.ok ? 'ok' : 'warn');
}
function cwvApplyBody() {
  if (!CWV.eval || !CWV.tool) return CWZ.solve.apply;
  return CWZ.CW.verifiedApplyBody(CWZ.solve.apply, { tool: CWV.tool, fingerprint: CWV.fp, evaluation: CWV.eval, points: CWV.pts, board: cwBoard(),
    jobId: CWV.job && CWV.job.id, confirm: CWV.confirm, measuredAt: new Date().toISOString() });
}
// sticker placement illustration: finger side view, sticker flush with the tip
const CWV_ILLU = '<svg viewBox="0 0 220 90" width="220" height="90" role="img" aria-label="sticker"><path d="M10 20 L150 30 L205 70 L200 78 L140 55 L10 60 Z" fill="#cbd5e1" stroke="#475569"/>'
  + '<path d="M150 30 L205 70 L200 78 L148 42 Z" fill="#fff" stroke="#111" stroke-width="1"/><path d="M205 70 L200 78" stroke="#111" stroke-width="3"/>'
  + '<circle cx="176" cy="52" r="2.5"/><circle cx="168" cy="46" r="2.5"/><text x="120" y="88" font-size="10" fill="#475569">▲ tip / 指尖</text></svg>';
// gate row of the fingertip / hover gates (non-blocking gates are shown as a note)
function cwvGateRow(key, g, unit, scale, pre) {
  const tr = document.createElement('tr');
  const v = typeof g.value === 'number' ? (Number.isInteger(g.value) ? String(g.value) : cwFmt(g.value * scale, scale === 1000 ? 1 : 3)) + unit : t('cw.g.' + key + '.' + g.value) || String(g.value);
  const lim = typeof g.limit === 'number' ? pre + (+(g.limit * scale).toPrecision(3)) + unit : t('cw.g.' + key + '.lim');
  tr.append(cwEl('td', '', t('cw.g.' + key)), cwEl('td', g.ok ? 'ok' : g.blocking === false ? 'note' : 'bad', (g.ok ? '✓ ' : g.blocking === false ? '· ' : '✗ ') + v), cwEl('td', '', lim));
  return tr;
}
// gripper not at the measured (closed) opening: the reason and a close button
function cwvGripBox(body, msg) {
  const box = cwEl('div', 'cwiz-msg cwiz-grip');
  box.appendChild(cwEl('div', '', msg));
  box.appendChild(cwBtn(t('cw.tip.close'), cwvCloseGripper, { id: 'cwiz-grip-close', disabled: CWZ.busy || CWV.tipBusy }));
  body.appendChild(box);
}
function cwRenderTip(body, p, acts) {
  const CW = CWZ.CW, tip = CWV.tip;
  p(t('cw.tip.lead'), 'cwiz-lead');
  p(t('cw.tip.hint'));
  const ill = cwEl('div', 'cwiz-illu'); ill.innerHTML = CWV_ILLU; body.appendChild(ill);
  if (CWV.tipBusy) body.appendChild(cwEl('div', 'calib-count mono', t('cw.tip.count', { n: CWV.frames })));
  if (tip && !tip.ok) {
    body.appendChild(cwEl('div', 'cwiz-msg', t('cw.tip.align')));
    if (CWV.roi) { const im = document.createElement('img'); im.src = CWV.roi; im.alt = 'ROI'; im.className = 'cwiz-roi'; im.id = 'cwiz-tip-roi'; body.appendChild(im); }
  }
  if (tip && tip.gates) body.appendChild(cwTable([t('cw.th.check'), t('cw.th.value'), t('cw.th.limit')], tip.gates.map((g) => cwvGateRow('tip_' + g.key, g,
    g.key === 'std' ? ' mm' : g.key === 'frames' || g.key === 'front' ? '' : ' px', 1, g.key === 'frames' || g.key === 'ambiguity' ? '≥ ' : g.key === 'front' ? '= ' : '≤ '))));
  if (tip && tip.ok) {
    const box = cwEl('div', 'calib-result mono');
    const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); box.appendChild(d); };
    row(t('cw.tip.r.cam'), tip.tip_cam_m.map((v) => (v * 1000).toFixed(2)).join(', ') + ' mm');
    const te = CW.posOf(CW.mul(cwvCand().X, CW.make([[1, 0, 0], [0, 1, 0], [0, 0, 1]], tip.tip_cam_m)));
    row(t('cw.tip.r.end'), te.map((v) => (v * 1000).toFixed(1)).join(', ') + ' mm');
    row(t('cw.tip.r.tool'), (CWV.toolId || '--') + ' · ' + (CWV.edge != null ? t('cw.tip.r.edge', { mm: (CWV.edge * 1000).toFixed(1) }) : t('cw.tip.r.noedge')));
    if (cwExpert()) row('fingerprint', (CWV.xfp || '--') + ' / ' + (CWV.fp || '--'));
    body.appendChild(box);
    const rm = cwEl('div', 'cwiz-msg ok', t('cw.tip.remove')); rm.id = 'cwiz-tip-remove'; body.appendChild(rm);
  }
  const g = cwvGrip();
  if (!(tip && tip.ok) && g != null && g > CWV_GRIP_CLOSED) cwvGripBox(body, t('cw.tip.open', { f: g.toFixed(2) }));
  acts(cwBtn(t('cw.back'), () => { CWZ.step = 'solve'; CWZ.status = ''; cwRender(); }, { cls: 'act-back', disabled: CWV.tipBusy }),
    cwBtn(t('cw.board.sticker'), cwPrintSticker, { id: 'cwiz-sticker' }),
    cwBtn(t(tip ? 'cw.tip.again' : 'cw.tip.measure'), cwvTipMeasure, { cls: tip && tip.ok ? '' : 'btn-primary', disabled: CWV.tipBusy || CWZ.busy, id: 'cwiz-tip-measure' }),
    tip && tip.ok && CWV.fp ? cwBtn(t('cw.tip.next'), () => { CWZ.step = 'hover'; CWZ.status = ''; cwRender(); }, { cls: 'btn-primary', id: 'cwiz-tip-next' }) : null);
}
function cwRenderHover(body, p, acts) {
  const CW = CWZ.CW, job = CWV.job, running = !!(job && job.state === 'running') || !!(CWV.confirmJob && CWV.confirmJob.state === 'running');
  p(t('cw.hov.lead'), 'cwiz-lead');
  p(t('cw.hov.hint'));
  if (CWV.edge == null || !Array.isArray(S.physicalContactEnd)) body.appendChild(cwEl('div', 'cwiz-msg', t('cw.hov.noedge')));
  const off = cwvGripOff();
  if (off) cwvGripBox(body, t('cw.hov.grip', { now: off.now.toFixed(2), then: off.then.toFixed(2) }));
  if (CWV.plan && CWV.plan.points.length) {
    const ol = cwEl('ol', 'calib-list mono cwiz-poses');
    CWV.plan.points.forEach((pt, i) => {
      const jp = job && job.points ? job.points.find((x) => x.id === pt.id) : null;
      const ev = CWV.pts && CWV.pts.find((x) => x.id === pt.id);
      const st = jp ? jp.state : 'pending';
      const li = cwEl('li', st === 'done' ? 'ok' : st === 'skipped' || st === 'failed' ? 'skip' : '');
      let txt = `${(pt.corner[0] * 1000).toFixed(0)}, ${(pt.corner[1] * 1000).toFixed(0)} mm · `;
      if (ev && ev.ok) txt += t('cw.hov.pt', { h: cwFmt(((ev.height_m ?? ev.edge_height_m)) * 1000, 1), xy: cwFmt(Math.hypot(...ev.xy_err_m) * 1000, 1) }) + (cwExpert() ? ` · edge ${cwFmt(ev.edge_height_m * 1000, 1)} mm · ${ev.frames} fr · ${cwFmt(ev.reproj_px, 2)} px` : '');
      else if (st === 'skipped' || st === 'failed') txt += t('cw.pose.skip', { why: jp.code || st }) + (cwExpert() && jp.message ? ' · ' + jp.message : '');
      else txt += t(st === 'done' ? 'cw.hov.measured' : 'cw.pose.pending');
      li.append(cwEl('b', '', '#' + (i + 1)), cwEl('span', '', txt));
      ol.appendChild(li);
    });
    body.appendChild(ol);
  }
  if (running) body.appendChild(cwEl('div', 'calib-count mono', t('cw.hov.running', { ph: (S.obs && S.obs.action && S.obs.action.phase) || '…' })));
  const ev = CWV.eval;
  if (ev) {
    body.appendChild(cwTable([t('cw.th.check'), t('cw.th.value'), t('cw.th.limit')], ev.gates.map((g) => cwvGateRow('hov_' + g.key, g,
      g.key === 'points' ? '' : ' mm', g.key === 'points' ? 1 : 1000, g.key === 'points' ? '≥ ' : g.key === 'clearance' ? '> ' : g.key === 'tool' ? '' : '≤ '))));
    const box = cwEl('div', 'calib-result mono' + (ev.ok ? '' : ' warn'));
    const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); box.appendChild(d); };
    row(t('cw.hov.r.table'), ev.table_z_m != null ? (ev.table_z_m * 1000).toFixed(1) + ' mm' : t('cw.hov.r.notable'));
    row(t('cw.hov.r.camera'), (CWZ.solve.apply.table_z_m * 1000).toFixed(1) + ' mm');
    if (ev.optical_table_z_m != null) row(t('cw.hov.r.optical'), (ev.optical_table_z_m * 1000).toFixed(1) + ' mm');
    row(t('cw.hov.r.floor'), ev.table_z_m != null ? ((ev.table_z_m - 0.003) * 1000).toFixed(1) + ' mm' : '--');
    if (CWV.confirm) row(t('cw.hov.r.confirm'), (CWV.confirm.ok ? '✓ ' : '✗ ') + cwFmt(CWV.confirm.worst_m * 1000, 1) + ' mm');
    body.appendChild(box);
  }
  const ended = job && job.state && job.state !== 'running' && job.state !== 'done';
  if (ended && job.arm) body.appendChild(cwEl('div', 'cwiz-msg', t('cw.hov.armstate', { tq: t(job.arm.is_torqued ? 'cw.hov.tq.on' : 'cw.hov.tq.off'), f: job.arm.fault ? (job.arm.fault.code || 'fault') : '--' })));
  const canConfirm = ev && ev.ok && ev.table_z_m != null && Array.isArray(S.physicalContactEnd) && !CWV.confirm && !running;
  acts(cwBtn(t('cw.back'), () => { CWZ.step = 'tip'; CWZ.status = ''; cwRender(); }, { cls: 'act-back', disabled: running || CWZ.busy }),
    cwBtn(t('cw.hov.plan'), cwvPlan, { disabled: running || CWZ.busy, id: 'cwiz-hov-plan', cls: CWV.plan ? '' : 'btn-primary' }),
    CWV.plan && CWV.plan.ok && !ev && !ended ? cwBtn(t('cw.hov.start'), () => cwvStart(false), { cls: 'btn-primary', disabled: running || CWZ.busy, id: 'cwiz-hov-start' }) : null,
    ended ? cwBtn(t('cw.hov.resume'), () => cwvStart(false, job.id), { cls: 'btn-primary', disabled: CWZ.busy, id: 'cwiz-hov-resume' }) : null,
    canConfirm ? cwBtn(t('cw.hov.confirm'), () => cwvStart(true), { cls: 'btn-primary', disabled: CWZ.busy, id: 'cwiz-hov-confirm' }) : null,
    // review after a finished run; primary only when the check passed (and was confirmed when it gives a table height)
    ev && job && job.state === 'done' && (!canConfirm || CWV.confirm)
      ? cwBtn(t('cw.solve.review'), cwReview, { cls: ev.ok && (!CWV.confirm || CWV.confirm.ok) ? 'btn-primary wide' : '', disabled: running || CWZ.busy, id: 'cwiz-review' }) : null);
}

function cwExport() {
  const out = { kind: 'rebot_calibration_wizard', version: 1, created: new Date().toISOString(), mode: CWZ.mode, board: { cols: 4, rows: 3, marker_side_m: CWZ.markerMm / 1000, gap_ratio: 0.2, thickness_m: CWZ.thickMm / 1000 },
    views: CWZ.views.map(({ G, ...v }) => ({ ...v, T_base_end: G })), plan: CWZ.plan.map((p) => ({ T: p.T, q: p.q, dist: p.dist, tiltDeg: p.tiltDeg, azDeg: p.azDeg, rollDeg: p.rollDeg, state: p.state, why: p.why })),
    result: CWZ.solve ? { ok: CWZ.solve.ok, gates: CWZ.solve.gates, apply: CWZ.solve.apply, ...(CWZ.solve.kin ? { kinematics: { heldOut: CWZ.solve.kin.heldOut, train: CWZ.solve.kin.train, warnings: CWZ.solve.kin.warnings, views: CWZ.solve.kin.views } } : {}) } : null };
  if (cwFine()) out.plans = [CWZ.saved[0] ? CWZ.saved[0].plan : CWZ.plan, CWZ.place ? CWZ.plan : []].map((pl) => pl.map((p) => ({ q: p.q, dist: p.dist, tiltDeg: p.tiltDeg, azDeg: p.azDeg, rollDeg: p.rollDeg, state: p.state, why: p.why })));
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(out, null, 1)], { type: 'application/json' }));
  a.download = 'rebot_calib_wizard_' + new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19) + '.json';
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
function cwPrint() {
  const svg = CWZ.CW.boardSvg(cwBoard());
  const w = window.open('', '_blank');
  if (!w) { toast('popup blocked', 'warn'); return; }
  w.document.write('<!doctype html><title>reBot calibration board</title><style>@page{margin:0}body{margin:0}</style>' + svg);
  w.document.close(); w.focus(); setTimeout(() => w.print(), 300);
}
// 3D view: board, planned camera positions (pending / captured / skipped) and the previewed pose as the ghost
function cwV3Sync() {
  const T = V3.T;
  if (!T || !V3.root) return;
  if (V3.cwiz) { V3.root.remove(V3.cwiz); V3.cwiz.traverse((o) => { if (o.geometry) o.geometry.dispose(); }); V3.cwiz = null; }
  if (!CWZ.open || !CWZ.est || !CWZ.plan.length) { V3.dirty = true; return; }
  const CW = CWZ.CW, g = V3.cwiz = new T.Group(), b = CWZ.board, Tb = CWZ.est.Tbb;
  const corners = [[-b.width / 2, -b.height / 2, 0], [b.width / 2, -b.height / 2, 0], [b.width / 2, b.height / 2, 0], [-b.width / 2, b.height / 2, 0]]
    .map((c) => { const p = CW.mul(Tb, [[c[0]], [c[1]], [0], [1]]); return new T.Vector3(p[0][0], p[1][0], p[2][0]); });
  g.add(new T.LineLoop(new T.BufferGeometry().setFromPoints(corners), new T.LineBasicMaterial({ color: 0x111827 })));
  const dot = new T.SphereGeometry(0.008, 10, 6), col = { pending: 0x3b82f6, ok: 0x16a34a, skip: 0x9ca3af };
  const c0 = CW.posOf(Tb);
  CWZ.plan.forEach((p, i) => {
    const m = new T.Mesh(dot, new T.MeshBasicMaterial({ color: i === CWZ.idx && p.state === 'pending' ? 0xf59e0b : col[p.state] || col.pending }));
    m.position.set(p.camPos[0], p.camPos[1], p.camPos[2]); g.add(m);
    g.add(new T.Line(new T.BufferGeometry().setFromPoints([m.position.clone(), new T.Vector3(c0[0], c0[1], c0[2])]), new T.LineBasicMaterial({ color: 0x93c5fd, transparent: true, opacity: 0.5 })));
  });
  V3.root.add(g);
  const nxt = CWZ.plan[Math.min(CWZ.idx, CWZ.plan.length - 1)];
  CWZ.previewQ = nxt && nxt.state === 'pending' && !CWZ.running ? [...toMotor6(nxt.q), (S.obs && S.obs.positions && S.obs.positions[6]) || 0] : null;
  V3.key = ''; V3.dirty = true;
}
// rendering
function cwEl(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; }
function cwBtn(label, fn, { cls = '', disabled = false, id = '', op = '', opMode = '' } = {}) { const b = cwEl('button', 'btn ' + cls, label); b.type = 'button'; if (id) b.id = id; b.disabled = disabled; if (op) { b.dataset.op = op; if (opMode) b.dataset.opMode = opMode; } if (fn) b.addEventListener('click', fn); return b; }
const cwFmt = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '--');
/* Failed checks of a solve: what stays as it was, then one line per cause with the failed checks and what to change. */
const CW_GATE_HINT = { rms: 'img', view_rms: 'img', sigma: 'img', views: 'views', markers: 'views', coverage: 'views', kin_views: 'views',
  kin_heldout_ab: 'heldout', kin_heldout_ba: 'heldout', kin_improve: 'heldout', held_out: 'heldout', kin_bounds: 'bounds',
  he_pos: 'he', he_rot: 'he', rot_diversity: 'he', plane_spread: 'plane', plane_tilt: 'plane' };
function cwFailBox(gates, flatness = null) {
  const box = cwEl('div', 'cwiz-msg cwiz-fail');
  box.appendChild(cwEl('b', '', t('cw.f.fail')));
  const by = new Map();
  for (const g of gates || []) {
    if (g.ok || g.blocking === false) continue;
    const h = CW_GATE_HINT[g.key] || 'other';
    if (!by.has(h)) by.set(h, []);
    by.get(h).push(t('cw.g.' + g.key));
  }
  const ul = cwEl('ul', '');
  // a bent board explains the image error (calib_wizard.js boardFlatness): say so instead of the generic noise hint
  const flat = flatness && flatness.explains;
  const hint = (h) => (h === 'img' && flat ? t('cw.f.h.flat', { bow: flatness.bowMm.toFixed(1), max: Math.abs(flatness.worst.mm).toFixed(1), id: flatness.worst.id,
    r0: flatness.rms0.toFixed(2), r1: flatness.rms1.toFixed(2) }) : t('cw.f.h.' + h));
  for (const [h, names] of by) ul.appendChild(cwEl('li', h === 'img' && flat ? 'cwiz-flat' : '', names.join(t('cw.f.sep')) + t('cw.f.colon') + hint(h)));
  box.appendChild(ul);
  return box;
}
function cwGateRow(g) {
  const tr = document.createElement('tr');
  const unit = { px: ' px', m: ' mm', mm: ' mm', deg: '°', rel: ' %', frac: ' %', cells: '', count: '' }[g.unit] ?? '';
  const sc = (v) => (g.unit === 'm' ? v * 1000 : g.unit === 'rel' || g.unit === 'frac' ? v * 100 : v);
  const val = typeof g.value === 'number' ? (Number.isInteger(g.value) ? String(g.value) : cwFmt(sc(g.value), g.unit === 'rel' ? 2 : g.unit === 'frac' ? 1 : 3)) + unit : String(g.value);
  const lim = (g.key === 'views' || g.key === 'markers' || g.key === 'kin_views' || g.key === 'kin_improve' ? '≥ ' : g.key === 'coverage' || g.key === 'kin_bounds' ? '= ' : g.key === 'rot_diversity' ? '≥ ' : '≤ ') + (typeof g.limit === 'number' ? +sc(g.limit).toPrecision(3) : g.limit) + unit;
  tr.append(cwEl('td', '', t('cw.g.' + g.key)), cwEl('td', g.ok ? 'ok' : 'bad', (g.ok ? '✓ ' : '✗ ') + val), cwEl('td', '', lim));
  return tr;
}
function cwTable(head, rows) {
  const tb = cwEl('table', 'cwiz-table mono'), h = document.createElement('tr');
  for (const x of head) h.appendChild(cwEl('th', '', x));
  tb.appendChild(h);
  for (const r of rows) tb.appendChild(r);
  return tb;
}
// before / after rows from the dry-run reply: vision intrinsics, camera mount, grasp fields
function cwDiffRows(before, after) {
  const rows = [];
  const get = (o, p) => p.reduce((a, k) => (a == null ? a : a[k]), o);
  const fmt = (v, k) => (v == null ? '--' : Array.isArray(v) ? v.map((x) => (typeof x === 'number' ? (k === 'mm' ? (x * 1000).toFixed(1) : x.toFixed(4)) : String(x))).join(', ')
    : typeof v === 'number' ? (k === 'mm' ? (v * 1000).toFixed(1) + ' mm' : k === 'deg' ? v.toFixed(2) + '°' : Math.abs(v) < 10 ? v.toFixed(4) : v.toFixed(2)) : String(v));
  const add = (label, path, k) => {
    const a = get(before, path), b = get(after, path), tr = document.createElement('tr');
    if (JSON.stringify(a) !== JSON.stringify(b)) tr.className = 'chg';
    tr.append(cwEl('td', '', label), cwEl('td', '', fmt(a, k)), cwEl('td', '', fmt(b, k)));
    rows.push(tr);
  };
  for (const k of ['fx', 'fy', 'cx', 'cy', 'k1', 'k2']) add('vision ' + k, ['vision_intrinsics', k]);
  add('camera_mount.offset_m', ['camera_mount', 'offset_m'], 'mm');
  add('camera_mount.q_end_cam', ['camera_mount', 'q_end_cam']);
  add('camera_mount.tilt_deg', ['camera_mount', 'tilt_deg'], 'deg');
  for (const [k, u] of [['fx_px'], ['fy_px'], ['cx_px'], ['cy_px'], ['table_z_m', 'mm'], ['camera_tilt_deg', 'deg']]) add('grasp.' + k, ['grasp', k], u);
  // self-service verification (armd dry runs with calibration_verify): table height source, fingertip, acceptance, floor
  if (after && after.x_fingerprint) {
    add('table_z_source', ['table_z_source']);
    add('tool.tip_cam_m', ['tool', 'tip_cam_m'], 'mm');
    add('acceptance.passed', ['acceptance', 'passed']);
    add('floor.effective_floor_z_m', ['floor', 'effective_floor_z_m'], 'mm');
  }
  // armd's grip pixel of the new camera model; outside [0, 1] it keeps grip_pixel_norm (camera-down acquisition)
  const gm = after && after.grip_pixel_model;
  if (Array.isArray(gm) && gm.some((v) => !(v >= 0 && v <= 1))) {
    const tr = document.createElement('tr'), td = cwEl('td', 'note', t('cw.rev.gripOut'));
    tr.append(cwEl('td', '', 'grasp.grip_pixel_norm'), cwEl('td', '', fmt(get(before, ['grasp', 'grip_pixel_norm']))), td);
    td.title = 'grip_pixel_model ' + gm.map((v) => v.toFixed(3)).join(', ');
    rows.push(tr);
  } else add('grasp.grip_pixel_norm', ['grasp', 'grip_pixel_norm']);
  return rows;
}
// steps a click on the step bar may jump back to; going back before 'capture' discards the views (two clicks)
function cwCanGoto(k) {
  if (CWZ.running || CWZ.busy || CWZ.solving) return false;
  const steps = cwSteps(), ti = steps.indexOf(k), si = steps.indexOf(CWZ.step);
  if (ti < 0 || ti >= si) return false;
  // fine mode: the capture step of the current placement, the solve when there is one, or start over
  if (cwFine() && (k === 'capture' || k === 'captureb')) return CWZ.place === (k === 'captureb' ? 1 : 0) && CWZ.plan.length > 0;
  if (cwFine() && k === 'move') return false;
  if (k === 'capture') return CWZ.plan.length > 0;
  if (k === 'solve') return !!CWZ.solve;
  if (k === 'tip') return !!CWZ.solve && !(CWV.job && CWV.job.state === 'running');
  if (k === 'hover') return !!(CWV.tip && CWV.tip.ok && CWV.fp);
  return k === 'board' || k === 'place';
}
function cwGoto(k) {
  if (!cwCanGoto(k)) { if (CWZ.running) toast(t('cw.busy'), 'warn'); return; }
  const n = CWZ.views.length;
  if ((k === 'board' || k === 'place') && n > 0) {
    if (!(CWZ.resetArm && CWZ.resetArm.k === k && Date.now() < CWZ.resetArm.until)) {
      CWZ.resetArm = { k, until: Date.now() + 4000 };
      cwSetStatus(t('cw.restart.warn', { s: t('cw.step.' + k), n }), 'warn');
      setTimeout(() => { if (CWZ.resetArm && Date.now() >= CWZ.resetArm.until) { CWZ.resetArm = null; cwRender(); } }, 4050);
      return;
    }
    CWZ.views = []; CWZ.plan = []; CWZ.idx = 0; CWZ.est = null; CWZ.refined = false; CWZ.solve = null; CWZ.dry = null; CWZ.applied = null; CWZ.ownAction = null;
    CWZ.place = 0; CWZ.saved = [];
    CWZ.sessionId = null; CWZ.sessionAt = null; CWZ.sessionWarn = ''; CWZ.loaded = null; CWZ.verify = null; CWZ.X0 = null;
    cwvReset();
  }
  if (!CWZ.running) CWZ.search = null;
  CWZ.resetArm = null; CWZ.armedUntil = 0; CWZ.step = k; CWZ.status = ''; cwRender(); cwV3Sync();
}
function cwRestartBtn() {
  const armed = CWZ.resetArm && CWZ.resetArm.k === 'place' && Date.now() < CWZ.resetArm.until;
  return cwBtn(t(armed ? 'cw.restart.confirm' : 'cw.restart'), () => cwGoto('place'), { disabled: CWZ.running || CWZ.busy, id: 'cwiz-restart', cls: armed ? 'armed' : 'act-back' });
}
// fine calibration: per-joint correction rows (dry-run before / after, applied read-back)
function cwKinRows(before, after) {
  const joints = (x) => { const k = x && x.kinematics; return Array.isArray(k) ? k : k && Array.isArray(k.joints) ? k.joints : null; };
  const a = joints(before), b = joints(after), rows = [];
  const fmt = (j) => (!j ? '--' : `off ${((j.offset_rad || 0) * 180 / Math.PI).toFixed(2)}° · sc ${((j.scale || 0) * 100).toFixed(2)}% · tilt ${(j.tilt_rad || [0, 0]).map((v) => (v * 180 / Math.PI).toFixed(2)).join('/')}° · o ${(j.origin_m || [0, 0, 0]).map((v) => (v * 1000).toFixed(1)).join('/')} mm`);
  for (let i = 0; i < 6; i++) {
    const tr = document.createElement('tr'), ja = a && a[i], jb = b && b[i];
    if (JSON.stringify(ja || null) !== JSON.stringify(jb || null)) tr.className = 'chg';
    tr.append(cwEl('td', '', 'kinematics J' + (i + 1)), cwEl('td', '', fmt(ja)), cwEl('td', '', fmt(jb)));
    rows.push(tr);
  }
  const fit = after && after.kinematics && after.kinematics.fit;
  if (fit && Array.isArray(fit.heldout_mm)) {
    const tr = document.createElement('tr');
    tr.append(cwEl('td', '', 'kinematics.fit'), cwEl('td', '', ''), cwEl('td', '', t('cw.f.fitrow', { ab: cwFmt(fit.heldout_mm[0]), ba: cwFmt(fit.heldout_mm[1]), n: fit.views ?? '--' })));
    rows.push(tr);
  }
  return rows;
}
// fine calibration solve page: gates, camera, held-out (uncorrected -> corrected), per-joint parameters, warnings
function cwRenderFineSolve(body, p, acts) {
  const CW = CWZ.CW;
  if (CWZ.solving) {
    p(t('cw.f.solving', { s: Math.round((Date.now() - CWZ.solving.t0) / 1000), stage: CWZ.solving.stage || '…' }), 'cwiz-lead');
    p(t('cw.f.solving.hint'));
    return;
  }
  const r = CWZ.solve;
  if (!r) {
    acts(cwBtn(t('cw.solve.run'), cwFineSolve, { cls: 'btn-primary', id: 'cwiz-solve-again' }), cwBtn(t('cw.solve.export'), cwExport), cwRestartBtn());
    return;
  }
  p(t('cw.f.solve.hint'));
  body.appendChild(cwTable([t('cw.th.check'), t('cw.th.value'), t('cw.th.limit')], r.gates.map(cwGateRow)));
  const kin = r.kin || {};
  if (r.final) {
    const K = r.final.K, box = cwEl('div', 'calib-result mono' + (r.ok ? '' : ' warn'));
    const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); box.appendChild(d); };
    row(t('cw.r.intr'), `fx ${cwFmt(K.fx)} fy ${cwFmt(K.fy)}`);
    row('', `cx ${cwFmt(K.cx)} cy ${cwFmt(K.cy)} k1 ${cwFmt(K.k1, 4)} k2 ${cwFmt(K.k2, 4)}`);
    if (r.apply) {
      row(t('cw.r.mount'), r.apply.camera_mount.offset_m.map((v) => (v * 1000).toFixed(1)).join(', ') + ' mm');
      row('rpy', CW.rpyDeg(CW.quatR(r.apply.camera_mount.q_end_cam)).map((v) => v.toFixed(2)).join(', ') + '°');
      row(t('cw.r.table'), (r.apply.table_z_m * 1000).toFixed(1) + ' mm · ' + t('cw.f.r.tableAB', { a: (r.table.board_z_m[0] * 1000).toFixed(1), b: (r.table.board_z_m[1] * 1000).toFixed(1) }));
    }
    row(t('cw.f.r.time'), `${cwFmt(r.secs, 1)} s · ${r.worker ? 'Web Worker' : t('cw.f.r.main')} · ${kin.views ? kin.views.used : '--'} ${t('cw.f.r.views')}`);
    body.appendChild(box);
  }
  if (kin.heldOut) {
    const h = kin.heldOut, rowH = (dir, a, b, f) => { const tr = document.createElement('tr'); tr.append(cwEl('td', '', dir), cwEl('td', '', cwFmt(a.mm) + ' mm'), cwEl('td', b.mm <= CW.KIN_GATES.heldOutMm ? 'ok' : 'bad', cwFmt(b.mm) + ' mm'), cwEl('td', '', cwFmt(f * 100, 0) + ' %')); return tr; };
    body.appendChild(cwEl('h4', 'cwiz-sub', t('cw.f.ho.title')));
    body.appendChild(cwTable([t('cw.f.ho.dir'), t('cw.f.ho.before'), t('cw.f.ho.after'), t('cw.f.ho.imp')], [rowH('A → B', h.ab0, h.ab, h.improve.ab), rowH('B → A', h.ba0, h.ba, h.improve.ba)]));
  }
  cwLevelsTable(body, kin);
  if (kin.joints) {
    const D = 180 / Math.PI, bad = new Set((kin.outOfBounds || []).map((o) => `${o.joint}.${o.param}`));
    const cell = (j, keys, txt) => cwEl('td', keys.some((k) => bad.has(`${j}.${k}`)) ? 'bad' : '', txt);
    const rows = kin.joints.map((jt, i) => {
      const tr = document.createElement('tr'), j = i + 1, fixed = j === 1 || j === 6;
      tr.append(cwEl('td', '', 'J' + j + (fixed ? ' ' + t('cw.f.par.fixed') : '')),
        cell(j, ['offset'], (jt.offset_rad * D).toFixed(2)), cell(j, ['scale'], (jt.scale * 100).toFixed(2)),
        cell(j, ['tilt_x', 'tilt_y'], jt.tilt_rad.map((v) => (v * D).toFixed(2)).join(' / ')),
        cell(j, ['dx', 'dy', 'dz'], jt.origin_m.map((v) => (v * 1000).toFixed(1)).join(' / ')));
      return tr;
    });
    body.appendChild(cwEl('h4', 'cwiz-sub', t('cw.f.par.title')));
    const tb = cwTable([t('cw.f.par.joint'), t('cw.f.par.off'), t('cw.f.par.scale'), t('cw.f.par.tilt'), t('cw.f.par.origin')], rows);
    tb.classList.add('cwiz-par');
    body.appendChild(tb);
  }
  // the accuracy note says the result can be applied: only for a result that passed every check
  for (const wn of kin.warnings || []) {
    if (wn.key === 'accuracy' && !r.ok) continue;
    body.appendChild(cwEl('div', 'cwiz-msg cwiz-zero', wn.key === 'accuracy' ? t('cw.f.acc', { mm: wn.mm.toFixed(1), lim: wn.limit.toFixed(0) }) : t('cw.f.zero', { j: wn.joint, d: wn.deg.toFixed(1) })));
  }
  if (!r.ok) body.appendChild(cwFailBox(r.gates, r.flatness));
  acts(cwBtn(t('cw.solve.run'), cwFineSolve, { id: 'cwiz-solve-again' }), cwBtn(t('cw.solve.export'), cwExport), cwRestartBtn(),
    cwVerifyOn() ? cwBtn(t('cw.f.toTip'), () => { CWZ.step = 'tip'; CWZ.status = ''; cwRender(); }, { cls: 'btn-primary wide', disabled: !r.ok || CWZ.busy, id: 'cwiz-totip' })
      : cwBtn(t('cw.solve.review'), cwReview, { cls: 'btn-primary wide', disabled: !r.ok || CWZ.busy, id: 'cwiz-review' }));
}
/* action rows: see the .act-row CSS. A row is the buttons of one container; rows that also hold inputs, selects or text
   stay as they are. Runs on every change of the drawer bodies (MutationObserver) and on resize. */
const ACT_ROWS = '.cwiz-acts, .cwiz-find, .tch-acts, .tch-row';
const ACT_LAST = '.act-back, .tch-del, .calib-reset';
function tidyActs(row) {
  const kids = [...row.children].filter((e) => !e.hidden);
  const bs = kids.filter((e) => e.classList.contains('btn'));
  if (bs.length < 2 || bs.length !== kids.length) { row.classList.remove('act-row', 'one'); return; }
  row.classList.add('act-row');
  const prim = bs.find((b) => b.classList.contains('btn-primary') && !b.disabled) || bs.find((b) => b.classList.contains('btn-primary'));
  const sec = bs.filter((b) => b !== prim);
  const order = [...(prim ? [prim] : []), ...sec.filter((b) => !b.matches(ACT_LAST)), ...sec.filter((b) => b.matches(ACT_LAST))];
  order.forEach((b, i) => { if (row.children[i] !== b) row.insertBefore(b, row.children[i] || null); });
  for (const b of bs) b.classList.remove('act-full');
  if (prim) prim.classList.add('act-full');
  const rest = order.slice(prim ? 1 : 0);
  if (rest.length % 2) rest[rest.length - 1].classList.add('act-full');
  row.classList.remove('one');
  if (window.innerWidth > 480 && bs.length <= 4 && row.offsetWidth) {
    row.style.setProperty('--n', String(bs.length)); row.classList.add('one');
    const lh = (b) => parseFloat(getComputedStyle(b).lineHeight) || 18;
    if (bs.some((b) => b.scrollWidth > b.clientWidth + 1 || actLines(b) > 1 || b.clientWidth < lh(b) * 3)) row.classList.remove('one');
  }
}
// rendered text lines of a button
function actLines(b) {
  const r = document.createRange(); r.selectNodeContents(b);
  const tops = new Set([...r.getClientRects()].filter((x) => x.width > 0.5).map((x) => Math.round(x.top)));
  return tops.size;
}
function tidyAll(root = document) { for (const r of root.querySelectorAll(ACT_ROWS)) tidyActs(r); }
// step bar: full names when they all fit, else the compact form (current name + "3/10", dots)
function cwFitSteps() {
  const ol = $('cwiz-steps');
  if (!ol || !ol.children.length) return;
  ol.classList.remove('compact');
  ol.classList.toggle('compact', [...ol.children].some((li) => li.scrollWidth > li.clientWidth + 1));
}
addEventListener('resize', () => { if (CWZ.open) cwFitSteps(); }, { passive: true });
function cwRender() {
  if (!CWZ.open) return;
  const steps = $('cwiz-steps'), body = $('cwiz-body');
  steps.textContent = '';
  $('cwiz-stop').hidden = !cwCapStep() && !(CWZ.search && CWZ.search.running) && CWZ.step !== 'hover' && CWZ.step !== 'tip'; // Stop stays in the footer (visible also when collapsed)
  const stepList = cwSteps(), si = stepList.indexOf(CWZ.step);
  stepList.forEach((k, i) => {
    const li = cwEl('li', i === si ? 'on' : i < si ? 'done' : '', t((cwFine() ? 'cw.fstep.' : 'cw.step.') + k));
    if (cwCanGoto(k)) {
      li.classList.add('nav'); li.tabIndex = 0; li.setAttribute('role', 'button'); li.dataset.step = k;
      li.addEventListener('click', () => cwGoto(k));
      li.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); cwGoto(k); } });
    }
    li.dataset.pos = `${i + 1}/${stepList.length}`; li.title = li.textContent;
    steps.appendChild(li);
  });
  cwFitSteps();
  body.textContent = '';
  const p = (txt, cls = '') => body.appendChild(cwEl('p', cls, txt));
  const acts = (...bs) => { const d = cwEl('div', 'cwiz-acts'); for (const b of bs) if (b) d.appendChild(b); body.appendChild(d); return d; };
  if (!cwSupported() || !cwDofOk()) { body.appendChild(cwEl('div', 'cwiz-msg', t(cwSupported() ? 'cw.dof' : 'cw.unsupported'))); return; }
  if (!CWZ.CW) { p('…'); cwLoad().then(cwRender).catch((e) => { body.textContent = ''; body.appendChild(cwEl('div', 'cwiz-msg', t('cw.err', { m: e.message }))); }); return; }
  const CW = CWZ.CW;
  if (CWZ.step === 'board') {
    // mode: camera only (as before) or fine calibration (camera + arm geometry); fixed once views exist
    const md = cwEl('div', 'cwiz-mode'); md.setAttribute('role', 'group');
    for (const m of ['camera', 'fine']) {
      const b = cwBtn(t('cw.mode.' + m), () => { if (CWZ.views.length) return; CWZ.mode = m; CWZ.modeChosen = true; CWZ.status = ''; cwRender(); }, { id: 'cwiz-mode-' + m, disabled: CWZ.views.length > 0 && CWZ.mode !== m });
      b.setAttribute('aria-pressed', String(CWZ.mode === m)); if (CWZ.mode === m) b.classList.add('on');
      md.appendChild(b);
    }
    body.appendChild(md);
    p(t(cwFine() ? 'cw.mode.fine.hint' : 'cw.mode.camera.hint'), 'cwiz-mode-hint');
    p(t('cw.board.hint'));
    const box = cwEl('div', 'cwiz-board'); box.innerHTML = CW.boardSvg(cwBoard()); body.appendChild(box);
    const field = (label, id, val, step) => {
      const d = cwEl('label', 'cwiz-field'); d.appendChild(cwEl('span', '', label));
      const i = document.createElement('input'); i.type = 'number'; i.id = id; i.step = step; i.value = String(val); i.inputMode = 'decimal'; d.appendChild(i); body.appendChild(d); return i;
    };
    const iSide = field(t('cw.board.side'), 'cwiz-side', CWZ.markerMm, '0.1'), iTh = field(t('cw.board.thick'), 'cwiz-thick', CWZ.thickMm, '0.1');
    acts(cwBtn(t('cw.board.print'), cwPrint), cwVerifyOn() ? cwBtn(t('cw.board.sticker'), cwPrintSticker, { id: 'cwiz-sticker-b' }) : null, cwBtn(t('cw.sess.load'), cwSessionList, { id: 'cwiz-load', disabled: CWZ.views.length > 0 || !!(CWZ.sessions && CWZ.sessions.busy) }), cwBtn(t('cw.next'), () => {
      const a = Number(iSide.value), b = Number(iTh.value);
      if (!(a >= 20 && a <= 100 && b >= 0 && b <= 10)) { cwSetStatus(t('cw.board.bad'), 'warn'); return; }
      CWZ.markerMm = a; CWZ.thickMm = b; CWZ.board = cwBoard(); CWZ.step = 'place'; CWZ.status = ''; cwRender();
    }, { cls: 'btn-primary', id: 'cwiz-next' }));
    const ss = CWZ.sessions;
    if (ss) {
      const box = cwEl('div', 'cwiz-sess'); box.id = 'cwiz-sess';
      box.appendChild(cwEl('h4', 'cwiz-sub', t('cw.sess.title')));
      if (ss.busy && !ss.list.length) box.appendChild(cwEl('p', '', t('cw.sess.loading')));
      else if (ss.error) box.appendChild(cwEl('div', 'cwiz-msg', ss.error));
      else if (!ss.list.length) box.appendChild(cwEl('p', '', t('cw.sess.none')));
      const ol = cwEl('ol', 'calib-list mono');
      ss.list.forEach((x, i) => {
        const sm = x.summary || {}, li = cwEl('li');
        const ho = Array.isArray(sm.heldout_mm) ? ` · ${sm.level || ''} ${sm.heldout_mm.map((v) => cwFmt(v)).join(' / ')} mm` : '';
        const b = cwBtn(t('cw.sess.item', { k: t('cw.sess.kind.' + (x.kind === 'fine' ? 'fine' : 'camera')), at: cwWhen(x.created_at), n: x.views }) + ho, () => cwSessionLoad(x.id), { id: 'cwiz-sess-' + i, disabled: ss.busy, cls: 'cwiz-sess-item' });
        li.appendChild(b); ol.appendChild(li);
      });
      if (ss.list.length) box.appendChild(ol);
      box.appendChild(cwBtn(t('cw.sess.close'), () => { CWZ.sessions = null; cwRender(); }, { id: 'cwiz-sess-close' }));
      body.appendChild(box);
    }
  } else if (CWZ.step === 'place' || CWZ.step === 'move') {
    if (CWZ.step === 'move') p(t('cw.f.move.hint', { n: cwFinePoses(0), v: cwFineAccepted(0) }), 'cwiz-lead');
    p(t(CWZ.step === 'move' ? 'cw.f.move.do' : cwFine() ? 'cw.f.place.hint' : 'cw.place.hint'));
    const n = cwLiveCount(), why = cwBootWhy(n), se = CWZ.search, searching = !!(se && se.running), gc = cwGcBusy();
    body.appendChild(cwEl('div', 'calib-live mono' + (n >= CW.GATES.minMarkers ? ' ok' : ''), t('cw.place.live', { n })));
    // three ways to bring the camera onto the board
    body.appendChild(cwEl('h4', 'cwiz-sub', t('cw.find.title')));
    const fd = cwEl('div', 'cwiz-find');
    fd.append(
      cwBtn(t('cw.find.jog'), () => cwJogMode(true), { id: 'cwiz-find-jog', disabled: searching || gc }),
      (() => {
        const b = cwBtn(gc ? t('gc.exit') : t('cw.find.hand'), cwHandGuide, { id: 'cwiz-find-hand', cls: gc ? 'on' : '', disabled: searching || CWZ.busy || (!gc && !cwGcSupported()) });
        if (!gc && !cwGcSupported()) b.title = gcExecutorUnsupported() ? t('cw.find.hand.exec_unavail') : t('cw.find.hand.na');
        return b;
      })(),
      searching ? (() => { const b = cwBtn(t('cw.cap.stop'), null, { id: 'cwiz-find-stop', cls: 'cwiz-stop' }); bindStopButton(b, undefined, () => { CWZ.stopReq = true; cwRender(); }); return b; })()
        : cwBtn(t('cw.find.search'), cwSearch, { id: 'cwiz-find-search', disabled: CWZ.busy || gc }));
    body.appendChild(fd);
    if (gc) {
      const ph = (S.obs.gravity_comp && S.obs.gravity_comp.phase) || 'active';
      body.appendChild(cwEl('div', 'cwiz-msg ok', t('gc.ph.' + ph, { n: Math.round(((S.obs.gravity_comp && S.obs.gravity_comp.ramp) || 0) * 100) }) + ' · ' + t('cw.find.gc.hint')));
    } else if (!cwGcSupported()) body.appendChild(cwEl('p', 'cwiz-why', gcExecutorUnsupported() ? t('cw.find.hand.exec_unavail') : t('cw.find.hand.na')));
    if (se && (searching || se.n)) {
      const pr = cwEl('div', 'calib-prog');
      pr.appendChild(cwEl('div', 'calib-count mono', searching
        ? t(se.phase === 'aim' ? 'cw.find.aiming' : 'cw.find.prog', { k: se.k, n: se.n })
        : se.found ? t('cw.find.at', { k: se.found.k, n: se.n, m: se.found.m }) : t('cw.find.none', { n: se.n, min: CW.GATES.minMarkers })));
      const bar = cwEl('div', 'calib-bar'), bi = document.createElement('i'); bi.style.width = Math.min(100, (se.found && !searching ? 1 : se.k / Math.max(1, se.n)) * 100) + '%'; bar.appendChild(bi); pr.appendChild(bar);
      pr.id = 'cwiz-find-prog';
      body.appendChild(pr);
    }
    if (why) body.appendChild(cwEl('p', 'cwiz-why', why));
    acts(CWZ.step === 'move' ? null : cwBtn(t('cw.back'), () => { CWZ.step = 'board'; CWZ.status = ''; cwRender(); }, { disabled: searching, cls: 'act-back' }), cwBtn(CWZ.busy ? t('calib.cap.busy') : t('cw.place.cap'), cwBootstrap, { cls: 'btn-primary' + (CWZ.step === 'move' ? ' wide' : ''), disabled: CWZ.busy || !!why, id: 'cwiz-boot' }));
  } else if (cwCapStep()) {
    p(t(cwFine() ? 'cw.f.cap.hint' : 'cw.cap.hint', { d: cwPreDeg() }));
    const acc = cwAccepted(), skipped = CWZ.plan.filter((x) => x.state === 'skip').length;
    const pr = cwEl('div', 'calib-prog');
    pr.appendChild(cwEl('div', 'calib-count mono', cwFine()
      ? t('cw.f.count', { p: CWZ.place ? 'B' : 'A', n: CWZ.plan.filter((x) => x.state === 'ok').length, total: CWZ.plan.length, v: cwFineAccepted(CWZ.place), s: skipped })
      : t('cw.cap.count', { n: acc, target: CW.GATES.minViews, s: skipped })));
    const bar = cwEl('div', 'calib-bar'), bi = document.createElement('i'); bi.style.width = Math.min(100, CWZ.idx / Math.max(1, CWZ.plan.length) * 100) + '%'; bar.appendChild(bi); pr.appendChild(bar);
    body.appendChild(pr);
    const ol = cwEl('ol', 'calib-list mono cwiz-poses');
    CWZ.plan.forEach((x, i) => {
      const li = cwEl('li', (x.state === 'ok' ? 'ok' : x.state === 'skip' ? 'skip' : '') + (i === CWZ.idx ? ' cur' : ''));
      li.append(cwEl('b', '', '#' + (i + 1)), cwEl('span', '', `${Math.round(x.dist * 1000)} mm · ${x.tiltDeg}°/${Math.round(x.azDeg)}° · ` + (x.state === 'ok' ? t('cw.pose.ok', { m: x.markers }) + (x.settled === false ? ' · ' + t('cw.f.unsettled') : '') : x.state === 'skip' ? t('cw.pose.skip', { why: x.why }) : t('cw.pose.pending'))));
      ol.appendChild(li);
    });
    body.appendChild(ol);
    const tg = cwEl('label', 'cwiz-toggle'), cb = document.createElement('input');
    cb.type = 'checkbox'; cb.id = 'cwiz-runall'; cb.checked = CWZ.runAll; cb.disabled = CWZ.running;
    cb.addEventListener('change', () => { CWZ.runAll = cb.checked; cwRender(); });
    tg.append(cb, cwEl('span', '', t('cw.cap.runall'))); body.appendChild(tg);
    const left = CWZ.idx < CWZ.plan.length;
    if (cwFine()) {
      // Continue after an abort resumes at the next pose of the same plan
      const resume = CWZ.idx > 0 && left && !CWZ.running;
      const need = 2 * CW.KIN_GATES.minViewsPerPlacement, have = cwFineAccepted(CWZ.place);
      acts(
        left ? cwBtn(resume ? t('cw.f.continue', { i: CWZ.idx + 1 }) : t(CWZ.runAll ? 'cw.cap.start' : 'cw.cap.next'), cwRun, { cls: 'btn-primary wide', disabled: CWZ.running, id: 'cwiz-go', op: '/api/v1/detection/trigger', opMode: 'move' }) : null,
        cwBtn(t('cw.cap.preview'), () => { if (!V3.open) v3Toggle(true); setTimeout(cwV3Sync, 300); }, { id: 'cwiz-prev' }),
        CWZ.place === 0 ? cwBtn(t('cw.f.toB'), cwFineToB, { cls: left ? '' : 'btn-primary', disabled: CWZ.running || have < need, id: 'cwiz-tob' })
          : cwBtn(t('cw.cap.solve'), cwFineSolve, { cls: left ? '' : 'btn-primary', disabled: CWZ.running || have < need, id: 'cwiz-solve' }),
        cwRestartBtn(),
      );
      if (have < need && !left) body.appendChild(cwEl('div', 'cwiz-msg', t('cw.f.few', { v: have, need })));
    } else acts(
      left ? cwBtn(t(CWZ.runAll ? 'cw.cap.start' : 'cw.cap.next'), cwRun, { cls: 'btn-primary wide', disabled: CWZ.running, id: 'cwiz-go', op: '/api/v1/detection/trigger', opMode: 'move' }) : null,
      cwBtn(t('cw.cap.preview'), () => { if (!V3.open) v3Toggle(true); setTimeout(cwV3Sync, 300); }, { id: 'cwiz-prev' }),
      cwBtn(t('cw.cap.solve'), cwSolve, { disabled: CWZ.running || acc < 5, id: 'cwiz-solve' }),
      cwRestartBtn(),
    );
  } else if (CWZ.step === 'solve' && cwFine()) {
    cwRenderFineSolve(body, p, acts);
  } else if (CWZ.step === 'tip') {
    cwRenderTip(body, p, acts);
  } else if (CWZ.step === 'hover') {
    cwRenderHover(body, p, acts);
  } else if (CWZ.step === 'solve') {
    const r = CWZ.solve;
    p(t('cw.solve.hint'));
    body.appendChild(cwTable([t('cw.th.check'), t('cw.th.value'), t('cw.th.limit')], r.gates.map(cwGateRow)));
    if (r.intr && r.intr.K) {
      const K = (r.final && r.final.K) || r.intr.K, box = cwEl('div', 'calib-result mono' + (r.ok ? '' : ' warn'));
      const row = (k, v) => { const d = cwEl('div', 'row'); d.append(cwEl('span', '', k), cwEl('span', '', v)); box.appendChild(d); };
      row(t('cw.r.intr'), `fx ${cwFmt(K.fx)} fy ${cwFmt(K.fy)}`);
      row('', `cx ${cwFmt(K.cx)} cy ${cwFmt(K.cy)} k1 ${cwFmt(K.k1, 4)} k2 ${cwFmt(K.k2, 4)}`);
      const sg = r.intr.sigma || {};
      row('1σ', `fx ${cwFmt(sg.fx)} fy ${cwFmt(sg.fy)} cx ${cwFmt(sg.cx)} cy ${cwFmt(sg.cy)} px · cond ${cwFmt(r.intr.cond, 0)}`);
      if (r.apply) {
        row(t('cw.r.mount'), r.apply.camera_mount.offset_m.map((v) => (v * 1000).toFixed(1)).join(', ') + ' mm');
        row('rpy', CW.rpyDeg(CW.quatR(r.apply.camera_mount.q_end_cam)).map((v) => v.toFixed(2)).join(', ') + '°');
        row(t('cw.r.table'), (r.apply.table_z_m * 1000).toFixed(1) + ' mm');
      }
      if (r.joint) row(t('cw.r.joint'), `${cwFmt(r.joint.rmsBefore, 3)} → ${cwFmt(r.joint.rmsAfter, 3)} px`);
      if (r.intr.dropped && r.intr.dropped.length) row(t('cw.r.dropped', { list: r.intr.dropped.map((d) => '#' + (d.i + 1) + ' ' + d.rms.toFixed(2) + ' px').join(', ') }), '');
      body.appendChild(box);
    }
    if (!r.ok) body.appendChild(r.flatness && r.flatness.explains ? cwFailBox(r.gates, r.flatness) : cwEl('div', 'cwiz-msg', t('cw.solve.fail')));
    acts(CWZ.loaded || !CWZ.est || !CWZ.plan.length ? null : cwBtn(t('cw.cap.more'), () => {
      const q = S.obs && S.obs.positions ? cwModelQ(S.obs.positions) : CWZ.plan[CWZ.plan.length - 1].q;
      CWZ.plan.push(...cwPlan(CWZ.est, q, 8, CWZ.plan.map((x) => x.T))); CWZ.step = 'capture'; cwRender(); cwV3Sync();
    }), cwBtn(t('cw.solve.export'), cwExport), cwRestartBtn(),
    cwBtn(t('cw.solve.review'), cwReview, { cls: 'btn-primary wide', disabled: !r.ok || CWZ.busy, id: 'cwiz-review' }));
  } else if (CWZ.step === 'review') {
    p(t(cwFine() ? 'cw.f.rev.hint' : 'cw.rev.hint'));
    if (CWZ.dry) body.appendChild(cwTable([t('cw.th.field'), t('cw.th.before'), t('cw.th.after')], [...cwDiffRows(CWZ.dry.before, CWZ.dry.after), ...(cwFine() ? cwKinRows(CWZ.dry.before, CWZ.dry.after) : [])]));
    if (cwFine() && CWZ.solve) cwLevelsTable(body, CWZ.solve.kin);
    cwVerifyBlock(body);
    const armed = Date.now() < CWZ.armedUntil;
    const torqued = cwFine() && !!(S.obs && S.obs.is_torqued);
    if (torqued) body.appendChild(cwEl('div', 'cwiz-msg', t('cw.f.torque')));
    acts(cwBtn(t('cw.back'), () => { CWZ.step = 'solve'; CWZ.status = ''; cwRender(); }, { cls: 'act-back' }),
      CWZ.loaded ? cwBtn(t('cw.sess.verify'), cwVerify, { cls: cwNeedsVerify() ? 'btn-primary' : '', disabled: CWZ.busy, id: 'cwiz-verify' }) : null,
      torqued ? cwBtn(t('cw.f.park'), cwFinePark, { disabled: CWZ.busy, id: 'cwiz-park' }) : null,
      cwBtn(t(armed ? 'cw.rev.confirm' : 'cw.rev.apply'), cwApply, { cls: 'btn-primary' + (torqued ? ' wide' : ''), disabled: !CWZ.dry || CWZ.busy || torqued || cwNeedsVerify(), id: 'cwiz-apply', op: '/api/v1/calibration/apply' }));
  } else if (CWZ.step === 'applied') {
    const a = CWZ.applied || {};
    if (a.backup) p(t('cw.app.backup', { b: cwBackupName(a.backup) }));
    body.appendChild(cwTable([t('cw.th.field'), t('cw.th.after'), ''], [...cwDiffRows(a, a), ...(cwFine() ? cwKinRows(a, a) : [])].map((tr) => { tr.className = ''; tr.lastChild.textContent = ''; return tr; })));
    const vi = a.vision_intrinsics;
    if (vi && vi.revision != null) p('rebot-vision revision ' + vi.revision);
    const rb = CWZ.readback;
    if (rb && rb.floor) p(t('cw.app.floor', { f: cwFmt(rb.floor.effective_floor_z_m * 1000, 1), s: t('cw.app.floor.' + rb.floor.source) }));
    const armed = Date.now() < CWZ.armedUntil;
    acts(cwBtn(t(armed ? 'cw.app.restore.confirm' : 'cw.app.restore'), cwRestore, { disabled: CWZ.busy || !a.backup || !!a.restoredFrom, id: 'cwiz-restore', cls: armed ? 'armed' : 'act-back', op: '/api/v1/calibration/restore' }), cwRestartBtn(),
      cwBtn(t('cw.app.done'), () => cwOpen(false), { cls: 'btn-primary' }));
  }
  if (CWZ.status) body.appendChild(cwEl('div', 'cwiz-msg' + (CWZ.statusKind === 'ok' ? ' ok' : ''), CWZ.status));
  if (CWZ.sessionWarn) body.appendChild(cwEl('div', 'cwiz-msg cwiz-sess-warn', CWZ.sessionWarn));
  cwJogRender();
}
function cwOpen(open) {
  if (!open && CWZ.running) { cwStop(); }
  CWZ.open = open;
  $('cwiz').hidden = !open;
  clearInterval(CWZ.live); CWZ.live = null;
  CWZ.jog = false; $('cwiz-panel').classList.remove('jog'); $('cwiz-jogbar').hidden = true;
  if (open) {
    // armd with the self-service verification: the fine calibration (with fingertip and auto check) is the default
    if (S.caps && S.caps.calibration_verify && !CWZ.views.length && !CWZ.modeChosen && CWZ.step === 'board') CWZ.mode = 'fine';
    $('cwiz-panel').classList.remove('min');
    cwRender();
    // live marker count on the place step (and in the jog bar)
    CWZ.live = setInterval(() => { if (cwPlaceStep() && !CWZ.busy) { if (CWZ.jog) cwJogRender(); else cwRender(); } }, 700);
  } else { CWZ.previewQ = null; $('btn-calib').focus({ preventScroll: true }); }
  cwV3Sync();
  if (open !== cwOpen.was) { cwOpen.was = open; calVisionMode(open); }
}
function initCwiz() {
  $('cwiz-close').addEventListener('click', () => cwOpen(false));
  $('cwiz-min').addEventListener('click', () => { if (CWZ.jog) { cwJogMode(false); return; } $('cwiz-panel').classList.toggle('min'); cwFitSteps(); });
  $('cwiz-adv').addEventListener('click', () => { if (CWZ.running) { toast(t('cw.busy'), 'warn'); return; } cwOpen(false); calOpen(true); });
  bindStopButton($('cwiz-stop'), undefined, () => { CWZ.stopReq = true; cwRender(); });
  $('cwiz-jog-cap').addEventListener('click', cwJogCapture);
  $('cwiz-jog-back').addEventListener('click', () => cwJogMode(false));
  for (const ev of ['resize', 'scroll']) window.addEventListener(ev, () => { if (CWZ.jog) cwJogPlace(); }, { passive: true });
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && CWZ.open && !CWZ.running) cwOpen(false); });
  i18nHooks.push(cwRender);
}

// schema model_joint_map (armd: model = sign * motor + offset_rad) -> CFG.cart (motor = sign * model + off);
// the ?iksign / ?ikoff URL overrides still win for J1..J4
function applyJointMap(map) {
  const ok = map.slice(0, 6).every((m) => m && (m.sign === 1 || m.sign === -1) && Number.isFinite(m.offset_rad));
  if (!ok) return;
  for (let i = 0; i < 6; i++) { CFG.cart.sign[i] = map[i].sign; CFG.cart.off[i] = -map[i].sign * map[i].offset_rad; }
  cartUrlMap();
  camMountNow();
  V3.key = ''; PCARD.key = '';
}
function isValidEffectiveOwner(v) {
  return v === 'executor' || v === 'legacy' || v === 'failed' || v === 'transferring';
}
function isOwnerFallbackReason(reason) {
  return typeof reason === 'string' && (reason.startsWith('auto_fallback_') || reason === 'startup_fallback_to_legacy' || reason === 'startup_failed_detached_safe_hold');
}
function ownerReasonLabel(reason) {
  if (!reason) return '';
  const k = 'owner.reason.' + reason;
  const translated = t(k);
  return translated === k ? String(reason) : translated;
}
const DEFAULT_LEGACY_ONLY_ROUTES = { rest: ['/api/v1/diagnostics/power'], ws: ['/api/v1/action/gripper_calibrate'] };
function effectiveOwnerState() {
  const obs = S.obs;
  const diag = S.ownerDiag;
  const obsOwner = obs && isValidEffectiveOwner(obs.effective_owner) ? obs.effective_owner : null;
  const diagOwner = diag && isValidEffectiveOwner(diag.effective_owner) ? diag.effective_owner : null;
  const obsLive = typeof isLive === 'function' ? isLive() : true;
  const useObs = !!(obsOwner && obsLive);
  const owner = useObs ? obsOwner : (diagOwner || obsOwner);
  const reason = useObs
    ? (obs && typeof obs.owner_reason === 'string' ? obs.owner_reason : '')
    : (diagOwner
      ? (diag && typeof diag.owner_reason === 'string' ? diag.owner_reason : '')
      : (obs && typeof obs.owner_reason === 'string' ? obs.owner_reason : ''));
  const routes = diag && diag.legacy_only_routes && typeof diag.legacy_only_routes === 'object'
    ? diag.legacy_only_routes
    : DEFAULT_LEGACY_ONLY_ROUTES;
  const detachedSafeHold = !useObs && !!(diag && diag.detached_safe_hold);
  return { owner, reason, routes, detachedSafeHold };
}
function isRouteLegacyOnly(routePath) {
  if (!routePath) return false;
  const { routes } = effectiveOwnerState();
  const ws = Array.isArray(routes.ws) ? routes.ws : [];
  const rest = Array.isArray(routes.rest) ? routes.rest : [];
  return ws.includes(routePath) || rest.includes(routePath);
}
function legacyOnlyRouteBlock(routePath, specificKey) {
  if (!isRouteLegacyOnly(routePath)) return null;
  const { owner } = effectiveOwnerState();
  if (!owner || owner === 'legacy') return null;
  if (owner === 'failed' || owner === 'transferring') {
    return t(specificKey === 'gb.legacy_only' ? 'gb.owner_unavailable' : 'owner.unavailable.generic');
  }
  return t(specificKey || 'owner.legacy_only.generic');
}
function syncLegacyOnlyControls() {
  if (typeof document === 'undefined' || !document.querySelectorAll) return;
  for (const el of document.querySelectorAll('[data-route]')) {
    const why = legacyOnlyRouteBlock(el.dataset.route, el.dataset.legacyOnlyKey);
    if (why) {
      el.dataset.legacyGated = '1';
      el.disabled = true;
      el.setAttribute('aria-disabled', 'true');
      el.title = why;
    } else if (el.dataset.legacyGated === '1') {
      delete el.dataset.legacyGated;
      el.disabled = false;
      el.removeAttribute('aria-disabled');
      el.title = '';
    }
  }
}
let ownerDiagRetryTimer = null;
let ownerDiagInflight = false;
function scheduleOwnerDiagRetry(delayMs = 3000) {
  if (typeof setTimeout !== 'function' || ownerDiagRetryTimer) return;
  ownerDiagRetryTimer = setTimeout(() => { ownerDiagRetryTimer = null; loadOwnerDiagnostics(); }, delayMs);
}
async function loadOwnerDiagnostics() {
  if (ownerDiagInflight) return;
  ownerDiagInflight = true;
  let d = null;
  try {
    try {
      d = await api('/api/v1/owner/status', undefined, { quiet: true, method: 'GET' });
    } catch (_) {
      try {
        d = await api('/api/v1/diagnostics', undefined, { quiet: true, method: 'GET' });
      } catch (_) {}
    }
    if (d && isValidEffectiveOwner(d.effective_owner)) {
      const prevOwner = S.ownerDiag ? S.ownerDiag.effective_owner : null;
      const prevReason = S.ownerDiag ? S.ownerDiag.owner_reason : null;
      const prevRoutes = S.ownerDiag ? JSON.stringify(S.ownerDiag.legacy_only_routes || null) : null;
      const nextRoutes = d.legacy_only_routes && typeof d.legacy_only_routes === 'object' ? d.legacy_only_routes : null;
      S.ownerDiag = {
        effective_owner: d.effective_owner,
        owner_state: typeof d.owner_state === 'string' ? d.owner_state : d.effective_owner,
        owner_reason: typeof d.owner_reason === 'string' ? d.owner_reason : '',
        detached_safe_hold: !!d.detached_safe_hold,
        legacy_only_routes: nextRoutes,
      };
      renderOwnerStatus();
      if (prevOwner !== S.ownerDiag.effective_owner || prevReason !== S.ownerDiag.owner_reason || prevRoutes !== JSON.stringify(nextRoutes)) {
        if (prevOwner != null && prevOwner !== S.ownerDiag.effective_owner && (S.ownerDiag.effective_owner === 'executor' || S.ownerDiag.effective_owner === 'legacy') && typeof loadSchema === 'function') {
          loadSchema();
        }
        if (typeof renderGc === 'function') renderGc();
        if (typeof gtRender === 'function' && $('gtool') && !$('gtool').hidden) gtRender();
        if (typeof cwRender === 'function' && typeof CWZ !== 'undefined' && CWZ.open) cwRender();
      }
      if (S.ownerDiag.effective_owner === 'transferring' || S.ownerDiag.effective_owner === 'failed') {
        scheduleOwnerDiagRetry(2000);
      }
    } else {
      scheduleOwnerDiagRetry(2000);
    }
  } finally {
    ownerDiagInflight = false;
  }
}
function renderOwnerStatus() {
  const badge = $('owner-badge');
  const banner = $('owner-banner');
  const { owner, reason, detachedSafeHold } = effectiveOwnerState();
  if (!owner) {
    if (badge) badge.hidden = true;
    if (banner) banner.hidden = true;
    return;
  }
  const failed = owner === 'failed';
  const transferring = owner === 'transferring';
  const fallback = failed || transferring || (owner === 'legacy' && isOwnerFallbackReason(reason));
  const ownerName = t('owner.name.' + owner);
  const reasonTxt = ownerReasonLabel(reason) || t('owner.reason.default');
  const detail = reason ? t('owner.title', { owner: ownerName, reason: reasonTxt }) : ownerName;
  if (badge) {
    badge.hidden = false;
    setText(badge, failed ? t('owner.badge.failed') : transferring ? t('owner.badge.transferring') : fallback ? t('owner.badge.fallback') : t('owner.badge.' + owner));
    setCls(badge, 'sc-status-badge plain ' + (failed ? 'is-danger is-warn is-fallback' : fallback ? 'is-warn is-fallback' : owner === 'executor' ? 'is-ok' : ''));
    badge.title = detail;
    badge.setAttribute('aria-label', detail);
  }
  if (banner) {
    if (failed) {
      banner.hidden = false;
      setCls(banner, 'owner-banner is-danger');
      banner.setAttribute('role', 'alert');
      setText($('owner-banner-title'), t('owner.banner.failed.title'));
      const subKey = (detachedSafeHold || reason === 'startup_failed_detached_safe_hold') ? 'owner.banner.failed.sub' : 'owner.banner.failed.sub_generic';
      setText($('owner-banner-sub'), t(subKey, { reason: reasonTxt }));
    } else if (transferring) {
      banner.hidden = false;
      setCls(banner, 'owner-banner');
      banner.setAttribute('role', 'status');
      setText($('owner-banner-title'), t('owner.banner.transferring.title'));
      setText($('owner-banner-sub'), t('owner.banner.transferring.sub', { reason: reasonTxt }));
    } else if (owner === 'legacy' && isOwnerFallbackReason(reason)) {
      banner.hidden = false;
      setCls(banner, 'owner-banner');
      banner.setAttribute('role', 'status');
      const isStartup = reason === 'startup_fallback_to_legacy';
      setText($('owner-banner-title'), t(isStartup ? 'owner.banner.startup_fallback.title' : 'owner.banner.auto_fallback.title', { reason: reasonTxt }));
      setText($('owner-banner-sub'), t(isStartup ? 'owner.banner.startup_fallback.sub' : 'owner.banner.auto_fallback.sub'));
    } else {
      banner.hidden = true;
    }
  }
  syncLegacyOnlyControls();
}
if ($('owner-badge')) $('owner-badge').addEventListener('click', () => {
  const badge = $('owner-badge');
  if (badge && badge.title) toast(badge.title, badge.classList.contains('is-warn') ? 'warn' : 'info');
});
i18nHooks.push(renderOwnerStatus);
function renderToolStatus() {
  const tool = S.activeTool;
  const badge = $('tool-badge');
  const banner = $('tool-comm-banner');
  if (!tool || !tool.id) {
    if (badge) badge.hidden = true;
    if (banner) banner.hidden = true;
    return;
  }
  const req = tool.commissioning_required === true;
  if (badge) {
    badge.hidden = false;
    setText(badge, req ? t('tool.badge.comm', { id: tool.id }) : tool.id);
    setCls(badge, 'sc-status-badge plain ' + (req ? 'is-warn' : tool.is_stock ? 'is-stock' : 'is-ok'));
    badge.title = req ? t('tool.comm.hint') : tool.id;
  }
  if (banner) {
    banner.hidden = !req;
    if (req) {
      setText($('tool-comm-title'), t('tool.comm.title', { id: tool.id }));
      setText($('tool-comm-sub'), t('tool.comm.hint'));
    }
  }
}
i18nHooks.push(renderToolStatus);
let schemaLoadSeq = 0;
let schemaRetryTimer = null;
async function loadSchema() {
  const seq = ++schemaLoadSeq;
  if (schemaRetryTimer) { clearTimeout(schemaRetryTimer); schemaRetryTimer = null; }
  try {
    const sc = await api('/api/v1/observation/schema', undefined, { quiet: true, method: 'GET' });
    if (seq !== schemaLoadSeq) return;
    const oldProfile = S.profileId;
    const oldClosed = CFG.gripClosed;
    const oldOpen = CFG.gripOpen;
    if (sc) {
      S.profileId = sc.profile_id || null;
      S.armDof = sc.arm_dof || (sc.dof ? (sc.dof === 7 ? 6 : sc.dof - 1) : 6);
      S.gripperJoint = Number.isInteger(sc.gripper_joint) ? sc.gripper_joint : (sc.dof ? sc.dof - 1 : 6);
      S.orientationDof = sc.orientation_dof != null ? sc.orientation_dof : (S.armDof >= 6 ? 3 : 2);
      S.jointNames = sc.joint_names || null;
      S.caps = sc.capabilities || {};
      S.backlash = sc.backlash_approach || null; // same-side approach (fine calibration pre-pose, cwFinePre)
      S.zeroCheckSupported = sc.gripper_policy ? !!sc.gripper_policy.zero_check_supported : (sc.capabilities ? !!sc.capabilities.zero_check_supported : false);
      S.powerGov = (sc.power_budget && sc.power_budget.governor) || null;
      PWR.key = '';
      S.wsPolicy = sc.workspace || null;
      S.wsCheckPoints = (sc.workspace && Array.isArray(sc.workspace.check_points_end_m)) ? sc.workspace.check_points_end_m : null;
      S.activeTool = sc.active_tool || null;
      renderToolStatus();
      S.tcpOffset = (sc.active_tool && Array.isArray(sc.active_tool.tcp_offset_m)) ? sc.active_tool.tcp_offset_m.map(Number) : [0, 0, 0];
      S.physicalContactEnd = Array.isArray(sc.physical_contact_end_m) && sc.physical_contact_end_m.length === 3 && sc.physical_contact_end_m.every(Number.isFinite)
        ? sc.physical_contact_end_m.map(Number)
        : (sc.physical_contact_end_m === null || (sc.active_tool && sc.active_tool.is_stock === false) ? null : [0, 0, 0]);
      // calibrated mount (camera calibration wizard): q_end_cam replaces camFlip / camRotDeg
      const qEnd = sc.camera_mount && Array.isArray(sc.camera_mount.q_end_cam) ? sc.camera_mount.q_end_cam : null;
      if (JSON.stringify(qEnd) !== JSON.stringify(CFG.view.qEndCam || null)) { CFG.view.qEndCam = qEnd; camMountNow(); V3.key = ''; }
      // kinematic correction (fine calibration): CALM.fk / VIK.frames use the same model as armd
      if (CALM.setCorrection(sc.kinematics_correction || null)) { camMountNow(); V3.key = ''; PCARD.key = ''; }
      const homeKey = JSON.stringify(S.home);
      S.home = Array.isArray(sc.home_positions) ? sc.home_positions : null;
      if (JSON.stringify(S.home) !== homeKey) { camMountNow(); V3.key = ''; }
      S.rest = Array.isArray(sc.resting_pose?.positions) ? sc.resting_pose.positions : S.home;
      if (sc.gripper_policy && Number.isFinite(sc.gripper_policy.closed_rad) && Number.isFinite(sc.gripper_policy.open_rad)) {
        CFG.gripClosed = sc.gripper_policy.closed_rad;
        CFG.gripOpen = sc.gripper_policy.open_rad;
        if (oldClosed !== CFG.gripClosed || oldOpen !== CFG.gripOpen) {
          S.grip = null;
          PCARD.key = '';
        }
      }
      if (sc.workspace && Number.isFinite(sc.workspace.max_reach_m)) {
        CFG.cart.reachMax = sc.workspace.max_reach_m;
        const rmaxEl = $('c-rmax');
        if (rmaxEl) rmaxEl.textContent = Math.round(CFG.cart.reachMax * 1000);
      }
      colOnCaps(); posesOnCaps(); epOnCaps(); teachOnCaps(); V3.sceneAt = 0;
      if (typeof renderGc === 'function') renderGc();
      if (typeof cwRender === 'function' && typeof CWZ !== 'undefined' && CWZ.open) cwRender();
    }
    if (sc && Array.isArray(sc.model_joint_map) && sc.model_joint_map.length >= 6) applyJointMap(sc.model_joint_map);
    if (sc && Array.isArray(sc.limits) && sc.limits.length) {
      const oldDof = S.dof;
      S.limits = sc.limits;
      S.dof = sc.dof || sc.limits.length;
      S.schemaLoaded = true;
      if (S.obs && !isCompleteObservation(S.obs)) { S.obs = null; S.wsObsSeen = false; S.obsWs = null; }
      // mirror the server's Cartesian floor so the cockpit stops at the same height
      if (sc.workspace && Number.isFinite(sc.workspace.floor_z_m)) CFG.cart.zMin = sc.workspace.floor_z_m;
      GRIP = Number.isInteger(sc.gripper_joint) ? sc.gripper_joint : S.dof - 1;
      S.hold = Array.from({ length: S.dof }, () => [0, 0]);
      // S.dof defaults to 7, so the 7-DOF text variant is set on every schema load, not only on a DOF change.
      const variant = S.dof === 7 ? '@7' : '';
      document.querySelectorAll('.kb-dof7').forEach((el) => { el.hidden = S.dof !== 7; });
      if (oldDof !== S.dof || oldProfile !== S.profileId || variant !== I18N_VARIANT) {
        I18N_VARIANT = variant;
        renderJogRows();
        renderGripCol();
        if (typeof renderJtable === 'function') renderJtable();
        applyI18n();
      }
      if (isSoArmProfile() && oldProfile !== S.profileId && !QS.get('ui') && !S.uiUserSet) {
        setUi('expert');
      }
      if (oldProfile !== S.profileId && V3.r) {
        if (V3.solid) { V3.root.remove(V3.solid.obj); V3.solid = null; }
        if (V3.ghost) { V3.root.remove(V3.ghost.obj); V3.ghost = null; }
        V3.init = null;
        if (V3.open) v3Start();
      }
      applyFrame();
    }
  } catch (_) {
    if (seq === schemaLoadSeq && !schemaRetryTimer) {
      schemaRetryTimer = setTimeout(() => { schemaRetryTimer = null; loadSchema(); }, 3000);
    }
  }
}

// observation.fault.code -> { title, todo } in the current language; null for an unknown code
function faultText() {
  const c = S.faultCode;
  if (!c || c === 'ESTOP' || I18N.zh['fc.' + c] === undefined) return null;
  let title = t('fc.' + c, { d: S.fault || t('unknown') });
  // joint limit / over-temperature / failed-enable details name per-joint status
  if ((c === 'JOINT_LIMIT' || c === 'THERMAL_TRIP' || c === 'ENABLE_FAILED') && S.fault && !/^[A-Z][A-Za-z]+$/.test(S.fault)) title += (LANG === 'zh' ? '（' + S.fault + '）' : ' (' + S.fault + ')');
  return { title, todo: t('fc.' + c + '.do') };
}
function reportedMotorErrors() {
  const errs = S.obs && Array.isArray(S.obs.motor_errors) ? S.obs.motor_errors : [];
  const list = [];
  for (let i = 0; i < errs.length; i++) {
    const e = errs[i];
    if (e && typeof e === 'object' && e.code != null) {
      list.push({ joint: i + 1, code: Number(e.code), name: String(e.name || '') });
    }
  }
  return list;
}
function hasCarriedOverMotorErrors() {
  return !isStopped() && reportedMotorErrors().length > 0;
}
function motorErrorSummary(list = reportedMotorErrors()) {
  return list.map((e) => {
    const hex = '0x' + e.code.toString(16).toUpperCase();
    return e.name ? `J${e.joint} ${hex} (${e.name})` : `J${e.joint} ${hex}`;
  }).join(', ');
}
function mitFaultKey() {
  const f = S.obs && S.obs.fault;
  if (f && (f.code || f.message)) return String(f.code || '') + ':' + String(f.message || '');
  const errs = reportedMotorErrors();
  return errs.length ? 'motor:' + errs.map((e) => `${e.joint}:${e.code}`).join(',') : '';
}
function lockReason() {
  if (!isLive()) return t('lock.offline');
  if (!obsFresh()) return t('lease.checking');
  const other = leaseOther();
  if (other != null) return t('lock.observer_who', { who: leaseHolderDetail(other) });
  if (S.state === 'EStop') return t('lock.estop');
  if (S.state === 'Fault') { const ft = faultText(); return ft ? t('lock.fault_coded', ft) : t('lock.fault'); }
  if (isOperational() && colHold()) return t('lock.col_hold');
  if (hasCarriedOverMotorErrors()) return t('lock.motor_err', { d: motorErrorSummary() });
  return t('lock.disabled');
}
function flashLock() {
  const n = $('lock-note');
  n.classList.remove('flash'); void n.offsetWidth; n.classList.add('flash');
  buzz(40);
}
const isLocked = () => document.body.classList.contains('locked');

