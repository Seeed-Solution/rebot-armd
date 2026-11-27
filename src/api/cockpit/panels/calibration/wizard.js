/* =================================================================
   Camera calibration wizard (capability calibration_apply): one automatic capture run on a printed
   ArUco GridBoard gives intrinsics, hand-eye mount and table height; armd applies them with a backup.
   Solvers: /assets/calib_wizard.js (CW, imported on first open). Steps: board -> place -> capture ->
   solve -> review (armd dry run) -> applied. Motion: action move_to at 0.03 m/s / 0.3 rad/s; a run
   aborts on Stop, a hidden page, stale telemetry, lease loss or a fault and never resumes by itself.
   ================================================================= */
const CW_STEPS = ['board', 'place', 'capture', 'solve', 'review', 'applied'];
/* Fine calibration mode ("精校", camera + arm geometry): board -> place A -> capture A -> move the board ->
   capture B -> solve (Web Worker, CW.solveFine) -> review -> applied. Joint-space capture: each pose is
   approached from the same side (pre-pose cwPreDeg() below the target on every joint in motor angles, above it
   when the lower soft limit is near: armd's same-side approach rule, schema backlash_approach, src/backlash.rs) with /api/v1/motion/jog target_positions + duration_secs (largest joint <= CWF.maxRate);
   settled = every joint moved < CWF.settleRad over CWF.settleMs (CWF.settleMaxMs max, the sample is kept and
   marked), dwell CWF.dwellMs, CWF.frames frames per pose. Segments are checked at 20 steps (limits, tool points
   >= CWF.clear above the board plane, >= 0.02 m above the base plane). An aborted run keeps its views;
   Continue resumes at the next pose of the same plan. One apply carries intrinsics, camera_mount, table_z_m
   and kinematics (armd requires torque off: KINEMATICS_ARMED otherwise). */
const CW_STEPS_FINE = ['board', 'place', 'capture', 'move', 'captureb', 'solve', 'review', 'applied'];
// armd with calibration_verify: fingertip sticker and hover acceptance before the review (T20)
const CW_STEPS_VERIFY = ['board', 'place', 'capture', 'move', 'captureb', 'solve', 'tip', 'hover', 'review', 'applied'];
const CWF = { n: 35, preDeg: 0.5, maxRate: 0.25, settleRad: 0.003, settleMs: 500, settleMaxMs: 15000, dwellMs: 1500, frames: 2, clear: 0.08,
  gen: { dists: [0.20, 0.24, 0.28, 0.32, 0.36, 0.40], tilts: [0, 12, 24, 36], azimuths: 8, rolls: [0, 20, -20], minMarkers: 8, clearance: 0.10, distRange: [0.2, 0.4] } };
const CW_SPEED = { linear: 0.03, angular: 0.3 };
const CWZ = { open: false, step: 'board', CW: null, markerMm: 25, thickMm: 0.5, board: null, views: [], plan: [], idx: 0, target: 20,
  runAll: false, running: false, stopReq: false, lease: null, epoch: null, controlSnap: null, ownAction: null, est: null, refined: false, solve: null, dry: null, applied: null,
  status: '', statusKind: '', armedUntil: 0, resetArm: null, previewQ: null, busy: false, live: null,
  mode: 'camera', place: 0, saved: [], solving: null, settled: true,
  sessionId: null, sessionAt: null, sessionWarn: '', sessions: null, loaded: null, verify: null, X0: null };
const cwFine = () => CWZ.mode === 'fine';
const cwSteps = () => (cwFine() ? (cwVerifyOn() ? CW_STEPS_VERIFY : CW_STEPS_FINE) : CW_STEPS);
const cwCapStep = (k = CWZ.step) => k === 'capture' || k === 'captureb';
const cwSupported = () => !!(S.caps && S.caps.calibration_apply);
const cwDofOk = () => !(isSoArmProfile() || (S.armDof != null && S.armDof !== 6));
const cwModelQ = (pos) => { const q = toModel6(pos); if (S.dof !== 7) q[5] = 0; return q; };
const cwIk = (T, seed) => { const { lo, hi } = modelLimits6(); return VIK.solve(T, seed, { lo, hi, iters: 60 }); };
function cwSetStatus(msg, kind = '') { CWZ.status = msg; CWZ.statusKind = kind; cwRender(); }
async function cwLoad() { return (CWZ.CW ||= await import('/assets/calib_wizard.js')); }
function cwBoard() { return CWZ.CW.makeBoard({ markerSide: CWZ.markerMm / 1000 }); }
// board markers in the live overlay (ids 0..11 with corners)
function cwLiveCount() {
  const { age } = calMarker();
  if (!(age < 1500)) return 0;
  const ids = new Set();
  for (const d of S.detections || []) if (d.aruco && Number(d.aruco.id) >= 0 && Number(d.aruco.id) < ARU_BOARD_IDS) ids.add(Number(d.aruco.id));
  return ids.size;
}
// why a running wizard must stop now (null: carry on); CW.runAbortReason has the rules
function cwAbortReason() {
  if (CWZ.controlSnap && !isControlSnapValid(CWZ.controlSnap)) {
    if (CWZ.stopReq || S.controlEpoch !== CWZ.controlSnap.controlEpoch) return 'stop';
    if (S.ws !== CWZ.controlSnap.ws || S.leaseGeneration !== CWZ.controlSnap.leaseGen) return 'lease';
    return 'epoch';
  }
  const o = S.obs || {};
  return CWZ.CW.runAbortReason({ stopReq: CWZ.stopReq, hidden: document.hidden, live: !!isLive() && !!S.ws && S.ws.readyState === 1,
    lease0: CWZ.lease, lease: S.leaseGeneration, epoch0: CWZ.epoch, epoch: typeof o.epoch_id === 'number' ? o.epoch_id : S.epoch,
    state: S.state, fault: o.fault || null, hold: colHold(),
    action: o.action ? o.action.id : null, actionState: o.action ? o.action.state : null, ownAction: CWZ.ownAction });
}
function cwScopedStop(snap) {
  return () => {
    if (!snap || snap.stopped || S.ws !== snap.ws || S.leaseGeneration !== snap.leaseGen) return Promise.resolve();
    const curEpoch = S.obs && typeof S.obs.epoch_id === 'number' ? S.obs.epoch_id : S.epoch;
    if (curEpoch !== snap.epoch) return Promise.resolve();
    snap.stopped = true;
    return api('/api/v1/motion/stop', {}, { quiet: true });
  };
}
const cwSleep = (ms) => new Promise((r) => setTimeout(r, ms));
async function cwGuardedWait(cond, timeoutMs) {
  const t0 = Date.now();
  for (;;) {
    const why = cwAbortReason();
    if (why) throw CWZ.CW.runAbort(why);
    const v = cond();
    if (v) return v;
    if (Date.now() - t0 > timeoutMs) throw Object.assign(new Error('timeout'), { code: 'TIMEOUT' });
    await cwSleep(50);
  }
}
// move_to the end pose T (4x4 EndLink) and wait for the action record; converts once to TCP at the
// public action boundary. throws { code, phase } on failure:
// phase 'reject' (command_reject / no ack, nothing started) or 'action' (the started action failed)
async function cwMove(T) {
  if (CWZ.controlSnap && !isControlSnapValid(CWZ.controlSnap)) {
    throw CWZ.CW.runAbort(cwAbortReason() || 'stop');
  }
  const CW = CWZ.CW;
  const off = S.tcpOffset || [0, 0, 0];
  let ack;
  try {
    ack = await commandRPC('/api/v1/action/move_to', {
      position: CW.apply(T, off), orientation: { quat: CW.quatOf(CW.rotOf(T)) }, speed: CW_SPEED, tag: 'calib-wizard' },
      true, 2000, CWZ.controlSnap ? { controlSnap: CWZ.controlSnap } : null);
  } catch (e) {
    if (isCancelledErr(e)) throw CW.runAbort(cwAbortReason() || 'stop');
    throw Object.assign(e, { phase: 'reject' });
  }
  const id = ack && ack.action_id;
  if (id == null) throw Object.assign(new Error('move_to ack without action_id'), { code: 'NO_ACTION_ID', phase: 'reject' });
  CWZ.ownAction = id;
  const rec = await cwGuardedWait(() => {
    const o = S.obs || {};
    for (const a of [o.action, o.previous_action]) if (a && a.id === id && a.state !== 'running') return a;
    return null;
  }, 90000);
  if (rec.state !== 'done') throw Object.assign(new Error(rec.message || rec.code || rec.state), { code: rec.code || rec.state, phase: 'action' });
}
// stationary (all joint speeds < 0.02 rad/s) for 0.3 s
async function cwSettle() {
  let since = 0;
  await cwGuardedWait(() => {
    const v = S.obs && S.obs.velocities;
    const still = Array.isArray(v) && v.every((x) => Math.abs(x) <= 0.02);
    if (!still) { since = 0; return false; }
    if (!since) since = Date.now();
    return Date.now() - since >= 300;
  }, 10000);
}
// one ["aruco"] trigger at the current (stationary) pose -> view { markers, G, q, joints } or { err }
async function cwCapture() {
  const pos = S.obs.positions.slice();
  let data;
  try { data = await detTriggerReq(TRIG_WAIT_MS, ['aruco']); } catch (e) { return { err: 'det', detail: e.message }; }
  if (!data || data.success === false || data.timeout) return { err: 'det' };
  if ((data.width && data.width !== 640) || (data.height && data.height !== 360)) return { err: 'det', detail: `image ${data.width}x${data.height}` };
  const now = S.obs && S.obs.positions;
  if (!Array.isArray(now) || pos.slice(0, 6).some((v, i) => Math.abs(now[i] - v) > 0.003)) return { err: 'moved' };
  const markers = (data.markers || []).filter((m) => Number(m.id) >= 0 && Number(m.id) < 12 && Array.isArray(m.corners) && m.corners.length === 4)
    .map((m) => ({ id: Number(m.id), corners: m.corners.map((c) => [Number(c[0]), Number(c[1])]) }));
  return { markers, G: calGripper(pos), q: cwModelQ(pos), joints: pos, frame_id: data.frame_id, ts: new Date().toISOString() };
}
// current mount (armd GET /api/v1/calibration, else the grasp setup tilt) -> nominal T_end_cam
async function cwNominalMount() {
  let mount = null;
  try { const r = await fetch('/api/v1/calibration', { cache: 'no-store' }); if (r.ok) mount = (await r.json()).camera_mount || null; } catch (_) {}
  if (!mount) { try { const r = await fetch('/api/v1/grasp/setup', { cache: 'no-store' }); if (r.ok) { const g = await r.json(); mount = { tilt_deg: (g.setup || g).camera_tilt_deg }; } } catch (_) {} }
  const R = CWZ.CW.mountRotation(VIK, mount || {}, toModel6(restJoints()));
  const off = mount && Array.isArray(mount.offset_m) ? mount.offset_m.map(Number) : null;
  return { X: CWZ.CW.make(R, off || [-0.08, 0, 0.05]), guessed: !off };
}
function cwPlan(est, q0, n, exclude = []) {
  const { lo, hi } = modelLimits6();
  return CWZ.CW.generatePoses({ ...(cwFine() ? CWF.gen : {}), Tbb: est.Tbb, board: CWZ.board, K: est.K, X: est.X, q0, fk: CALM.fk, ik: cwIk, lo, hi, n, exclude, extraEndPoints: S.wsCheckPoints })
    .poses.map((p) => ({ ...p, state: 'pending' }));
}
async function cwBootstrap() {
  if (CWZ.busy || CWZ.running) return;
  if (!isLive() || !S.obs || !Array.isArray(S.obs.positions)) { cwSetStatus(t('calib.e.offline'), 'warn'); return; }
  if (Array.isArray(S.obs.velocities) && S.obs.velocities.some((v) => Math.abs(v) > 0.02)) { cwSetStatus(t('calib.e.moving'), 'warn'); return; }
  if (cwGcBusy()) { cwSetStatus(t('cw.find.why.gc'), 'warn'); return; } // hand-guiding: exit (pose locked) first
  CWZ.busy = true; cwSetStatus(t('calib.cap.busy'));
  try {
    const CW = await cwLoad();
    CWZ.board = cwBoard();
    const v = await cwCapture();
    if (v.err) { cwSetStatus(t('cw.skip.' + (v.err === 'moved' ? 'moved' : 'det')) + (v.detail ? ' · ' + v.detail : ''), 'warn'); return; }
    const n = CW.boardMarkers(v, CWZ.board).length;
    if (n < CW.GATES.minMarkers) { cwSetStatus(t('cw.place.few', { n, min: CW.GATES.minMarkers }), 'warn'); return; }
    const { X, guessed } = await cwNominalMount();
    const bs = CW.bootstrapBoard(v, CWZ.board, { K: CW.NOMINAL, X, G: v.G });
    if (!bs) { cwSetStatus(t('cw.place.few', { n, min: CW.GATES.minMarkers }), 'warn'); return; }
    const est = { Tbb: bs.Tbb, K: { ...CW.NOMINAL }, X };
    const plan = cwFine() ? await cwPlanFine(est, v.q, CWF.n) : cwPlan(est, v.q, CWZ.target);
    if (!plan.length) { cwSetStatus(t('cw.place.nogen', { why: 'IK / clearance / markers' }), 'warn'); return; }
    CWZ.est = est;
    if (cwFine()) {
      // placement A starts the view list; placement B adds to it (views of A stay)
      const ds = CWZ.place;
      CWZ.views = [...CWZ.views.filter((x) => x.ds !== ds && ds === 1), { ...v, pose: -1, ds }];
      CWZ.step = ds ? 'captureb' : 'capture';
    } else {
      CWZ.views = [{ ...v, pose: -1 }];
      CWZ.step = 'capture';
    }
    CWZ.plan = plan; CWZ.idx = 0; CWZ.refined = false; CWZ.solve = null; CWZ.dry = null;
    cwSetStatus(t('cw.place.ok', { n: plan.length }) + (guessed ? ' · ' + t('cw.mount.guess', { o: CW.posOf(X).map((x) => Math.round(x * 1000)).join(', ') }) : ''), 'ok');
    cwV3Sync();
  } finally { CWZ.busy = false; cwRender(); }
}
// after 8 accepted views: re-solve with what is there and regenerate the remaining poses
function cwRefine() {
  const CW = CWZ.CW, ok = CWZ.views.filter((v) => !v.err);
  const r = CW.solveAll({ views: ok, board: CWZ.board, CALM, thickness: CWZ.thickMm / 1000 });
  if (!r.final) return false;
  CWZ.est = { Tbb: r.final.Tbb, K: r.final.K, X: r.final.X }; // after the joint refinement
  const done = CWZ.plan.slice(0, CWZ.idx), left = Math.max(0, CWZ.target + 1 - ok.length);
  const q = cwModelQ(S.obs.positions);
  const tail = cwPlan(CWZ.est, q, left, done.map((d) => d.T));
  if (!tail.length) return false;
  CWZ.plan = [...done, ...tail];
  return true;
}
const cwAccepted = () => CWZ.views.filter((v) => !v.err && CWZ.CW.boardMarkers(v, CWZ.board).length >= CWZ.CW.GATES.minMarkers).length;
// clearance / IK continuity from the arm's current pose to T (same check as the pose generator)
function cwCheckFromHere(T) {
  const { lo, hi } = modelLimits6(), q = cwModelQ(S.obs.positions);
  return CWZ.CW.checkTransition(CALM.fk(q), q, T, { ik: cwIk, X: CWZ.est.X, plane: CWZ.CW.boardPlane(CWZ.est.Tbb), clearance: 0.10, lo, hi, extraEndPoints: S.wsCheckPoints });
}
// visit poses from CWZ.idx: one (confirm mode) or all (run all); CW.runPoses decides skip / abort
async function cwRun() {
  if (CWZ.running || CWZ.idx >= CWZ.plan.length) return;
  if (!canMove()) { cwSetStatus(lockReason(), 'warn'); return; }
  const snap = requireLeaseSyncOrPrompt((e) => cwSetStatus(t('cw.cap.aborted', { why: e.message }), 'warn'));
  if (!snap) return;
  CWZ.running = true; CWZ.stopReq = false;
  try {
    CWZ.lease = snap.leaseGen; CWZ.epoch = snap.epoch; CWZ.controlSnap = snap; CWZ.ownAction = null;
    const CW = CWZ.CW;
    const res = await CW.runPoses(CWZ, cwFine() ? cwFineIo() : {
      abortReason: cwAbortReason,
      check: cwCheckFromHere,
      move: (T) => { cwSetStatus(t('cw.cap.moving', { i: CWZ.idx + 1 })); return cwMove(T); },
      settle: cwSettle,
      capture: (i) => { cwSetStatus(t('cw.cap.capturing', { i: i + 1 })); return cwCapture(); },
      accept: (v, i, p) => {
        const m = v.err ? 0 : CW.boardMarkers(v, CWZ.board).length;
        if (v.err || m < CW.GATES.minMarkers) { p.state = 'skip'; p.why = v.err ? t('cw.skip.' + (v.err === 'moved' ? 'moved' : 'det')) : t('cw.skip.markers', { m }); }
        else { p.state = 'ok'; p.markers = m; CWZ.views.push({ ...v, pose: i }); }
      },
      after: () => {
        if (!CWZ.refined && cwAccepted() >= 8) {
          CWZ.refined = true;
          if (cwRefine()) { toast(t('cw.cap.refined', { n: cwAccepted() })); cwV3Sync(); }
        }
        cwRender();
      },
      stop: cwScopedStop(snap),
      skipWhy: (kind, d) => (kind === 'move' ? t('cw.skip.move', { code: d }) : t('cw.skip.transition', { r: d })),
    }, { runAll: CWZ.runAll });
    cwSetStatus(res === 'done' ? t('cw.cap.done') : '', res === 'done' ? 'ok' : '');
    if (res === 'done') cwSessionUpload(); // a placement's capture finished
  } catch (e) {
    if (isCancelledErr(e)) return;
    const why = e.code === 'ABORTED' ? t('cw.abort.' + e.why) + (e.detail ? ' (' + e.detail + ')' : '') : e.message;
    cwSetStatus(t('cw.cap.aborted', { why }) + (cwFine() && CWZ.idx < CWZ.plan.length ? ' · ' + t('cw.f.kept', { n: cwFineAccepted(CWZ.place) }) : ''), 'warn');
  } finally { CWZ.running = false; CWZ.stopReq = false; CWZ.controlSnap = null; cwRender(); cwV3Sync(); }
}
function cwStop() {
  CWZ.stopReq = true;
  stopMotion();
  cwRender();
}
function cwSolve() {
  const CW = CWZ.CW;
  CWZ.solve = CW.solveAll({ views: CWZ.views.filter((v) => !v.err), board: CWZ.board, CALM, thickness: CWZ.thickMm / 1000, solvedAt: new Date().toISOString() });
  CWZ.step = 'solve'; CWZ.dry = null; CWZ.status = ''; CWZ.verify = null; cwRender();
  cwSessionUpload();
}
// ---- fine calibration: worker (plan, solve), joint-space capture
/* One Blob module worker per call imports /assets/calib_wizard.js; for 'plan' it also evaluates this page's
   CALIB-MATH and VIEW-IK blocks (same FK / IK, the current kinematic correction). Without a worker the same step
   generators run here in slices of <= 40 ms (CW.runSliced), so lease heartbeats keep going. */
function cwMathSource() {
  if (cwMathSource.src) return cwMathSource.src;
  for (const sc of document.scripts) {
    const x = sc.textContent || '', a = x.indexOf('/* CALIB-MATH-BEGIN */'), b = x.indexOf('/* CALIB-MATH-END */'), c = x.indexOf('/* VIEW-IK-BEGIN */'), d = x.indexOf('/* VIEW-IK-END */');
    if (a >= 0 && b > a && c >= 0 && d > c) return (cwMathSource.src = x.slice(a, b) + '\n' + x.slice(c, d));
  }
  return null;
}
function cwWorkerCall(op, data, onProgress = () => {}) {
  let w = null;
  try {
    if (typeof Worker !== 'function') throw new Error('no Worker');
    const src = `import * as CW from ${JSON.stringify(new URL('/assets/calib_wizard.js', location.href).href)};
let M = null;
self.onmessage = (e) => {
  const { op, data, math } = e.data;
  try {
    let r;
    if (op === 'solveFine') r = CW.solveFine({ ...data, onProgress: (s) => self.postMessage({ progress: s }) });
    else if (op === 'plan') {
      if (!M) M = new Function(math + '\\nreturn { CALM, VIK };')();
      M.CALM.setCorrection(data.correction);
      const lo = data.lo, hi = data.hi;
      const g = CW.generatePoses({ ...data.opts, Tbb: data.Tbb, board: CW.makeBoard({ markerSide: data.markerSide }), K: data.K, X: data.X, q0: data.q0, lo, hi, n: data.n, exclude: data.exclude,
        extraEndPoints: data.extraEndPoints || null,
        fk: M.CALM.fk, ik: (T, seed) => M.VIK.solve(T, seed, { lo, hi, iters: 60 }) });
      r = { poses: g.poses, stats: g.stats };
    } else throw new Error('op ' + op);
    self.postMessage({ done: r });
  } catch (err) { self.postMessage({ error: String((err && err.message) || err) }); }
};`;
    w = new Worker(URL.createObjectURL(new Blob([src], { type: 'text/javascript' })), { type: 'module' });
  } catch (_) { w = null; }
  if (!w) return null;
  const math = op === 'plan' ? cwMathSource() : null;
  if (op === 'plan' && !math) { w.terminate(); return null; }
  return new Promise((resolve, reject) => {
    w.onmessage = (e) => {
      const m = e.data || {};
      if (m.progress) { onProgress(m.progress); return; }
      w.terminate();
      if (m.error) reject(new Error(m.error)); else resolve(m.done);
    };
    w.onerror = (e) => { w.terminate(); reject(new Error(e.message || 'worker error')); };
    w.postMessage({ op, data, math });
  });
}
// fine-mode plan (CWF.gen options): worker, else sliced on this thread
async function cwPlanFine(est, q0, n, exclude = []) {
  const CW = CWZ.CW, { lo, hi } = modelLimits6();
  const data = { opts: CWF.gen, Tbb: est.Tbb, markerSide: CWZ.board.markerSide, K: est.K, X: est.X, q0, lo, hi, n, exclude, correction: CALM.correctionJoints(), extraEndPoints: S.wsCheckPoints || null };
  let g = null, worker = true;
  try { const p = cwWorkerCall('plan', data); if (p) g = await p; } catch (e) { g = null; }
  if (!g) {
    worker = false;
    g = await CW.runSliced(CW.generatePosesSteps({ ...CWF.gen, Tbb: est.Tbb, board: CWZ.board, K: est.K, X: est.X, q0, fk: CALM.fk, ik: cwIk, lo, hi, n, exclude, extraEndPoints: S.wsCheckPoints }));
  }
  CWZ.planWorker = worker;
  return g.poses.map((p) => ({ ...p, state: 'pending' }));
}
const cwFineAccepted = (ds) => CWZ.views.filter((v) => v.ds === ds && !v.err && CWZ.CW.boardMarkers(v, CWZ.board).length >= CWZ.CW.GATES.minMarkers).length;
const cwFinePoses = (ds) => new Set(CWZ.views.filter((v) => v.ds === ds && v.pose >= 0).map((v) => v.pose)).size;
// pre-pose (model angles q in and out): armd's same-side approach rule (schema backlash_approach, src/backlash.rs)
// in motor angles, so calibration and run time see the same backlash side: delta_deg below the target on every
// listed joint, above it where target - delta is below the joint's soft limit. Without the schema section the
// armd default (CWF.preDeg, every arm joint).
const cwPreDeg = () => { const b = S.backlash; return b && Number.isFinite(b.delta_deg) ? b.delta_deg : CWF.preDeg; };
function cwFinePre(q) {
  const d = cwPreDeg() * Math.PI / 180, b = S.backlash, names = b && Array.isArray(b.joints) ? b.joints : null;
  const m = toMotor6(q).map((v, i) => {
    if (names && S.jointNames && !names.includes(S.jointNames[i])) return v;
    const L = S.limits && S.limits[i];
    return !L || v - d >= L.min_position ? v - d : v + d;
  });
  return toModel6(m);
}
// joint-space segment qa -> qb at 20 steps: limits, full effective envelope points above the board plane and the base plane
function cwFineSegment(qa, qb) {
  const CW = CWZ.CW, { lo, hi } = modelLimits6(), plane = CW.boardPlane(CWZ.est.Tbb), X = CWZ.est.X;
  for (let k = 0; k <= 20; k++) {
    const q = qa.map((v, i) => v + (qb[i] - v) * k / 20);
    if (q.some((v, i) => v < lo[i] - 1e-6 || v > hi[i] + 1e-6)) return 'limits';
    const pts = CW.toolPoints(CALM.fk(q), X, S.wsCheckPoints);
    if (pts.some((p) => (p[0] - plane.p[0]) * plane.n[0] + (p[1] - plane.p[1]) * plane.n[1] + (p[2] - plane.p[2]) * plane.n[2] < CWF.clear)) return 'clearance';
    if (pts.some((p) => p[2] < 0.02)) return 'base';
  }
  return null;
}
// the current pose -> pre-pose -> target of plan pose i
function cwFineCheck(i) {
  const q = cwModelQ(S.obs.positions), p = CWZ.plan[i], pre = cwFinePre(p.q);
  const why = cwFineSegment(q, pre) || cwFineSegment(pre, p.q);
  return why ? { ok: false, reason: why } : { ok: true };
}
// joint-space move to model angles q (J1..J6) with /api/v1/motion/jog; waits for the action record (or, when the
// ack carries no action id, for the joints to arrive). Throws { code, phase } like cwMove.
async function cwJog(q) {
  if (CWZ.controlSnap && !isControlSnapValid(CWZ.controlSnap)) {
    throw CWZ.CW.runAbort(cwAbortReason() || 'stop');
  }
  const pos = S.obs.positions, m = toMotor6(q), full = [...m, ...pos.slice(6)];
  const dq = Math.max(...m.map((v, i) => Math.abs(v - pos[i])));
  const dur = Math.max(1.5, dq / CWF.maxRate);
  let ack;
  try { ack = await commandRPC('/api/v1/motion/jog', { target_positions: full, duration_secs: dur }, true, 2000, CWZ.controlSnap ? { controlSnap: CWZ.controlSnap } : null); }
  catch (e) {
    if (isCancelledErr(e)) throw CWZ.CW.runAbort(cwAbortReason() || 'stop');
    throw Object.assign(e, { phase: 'reject' });
  }
  const id = ack && ack.action_id;
  if (id == null) {
    await cwGuardedWait(() => S.obs.positions.slice(0, 6).every((v, i) => Math.abs(v - m[i]) < 0.01), dur * 1000 + 10000);
    return;
  }
  CWZ.ownAction = id;
  const rec = await cwGuardedWait(() => {
    const o = S.obs || {};
    for (const a of [o.action, o.previous_action]) if (a && a.id === id && a.state !== 'running') return a;
    return null;
  }, dur * 1000 + 30000);
  if (rec.state !== 'done') throw Object.assign(new Error(rec.message || rec.code || rec.state), { code: rec.code || rec.state, phase: 'action' });
}
// settled: every joint moved < CWF.settleRad over the last CWF.settleMs; after CWF.settleMaxMs the sample is kept
// and marked (CWZ.settled = false). Then the dwell.
async function cwFineSettle() {
  const hist = [], t0 = Date.now();
  CWZ.settled = false;
  await cwGuardedWait(() => {
    const now = Date.now();
    hist.push([now, S.obs.positions.slice(0, 6)]);
    while (hist.length && hist[0][0] < now - CWF.settleMs) hist.shift();
    if (hist.length >= 3 && now - hist[0][0] >= CWF.settleMs - 60) {
      const span = Math.max(...[0, 1, 2, 3, 4, 5].map((i) => Math.max(...hist.map((h) => h[1][i])) - Math.min(...hist.map((h) => h[1][i]))));
      if (span < CWF.settleRad) { CWZ.settled = true; return true; }
    }
    return now - t0 >= CWF.settleMaxMs;
  }, CWF.settleMaxMs + 1000);
  const end = Date.now() + CWF.dwellMs;
  await cwGuardedWait(() => Date.now() >= end, CWF.dwellMs + 1000);
}
function cwFineIo() {
  const CW = CWZ.CW;
  return {
    abortReason: cwAbortReason,
    check: () => cwFineCheck(CWZ.idx),
    move: async (T, i) => {
      cwSetStatus(t('cw.f.moving', { i: i + 1 }));
      await cwJog(cwFinePre(CWZ.plan[i].q));
      const why = cwAbortReason();
      if (why) throw CW.runAbort(why);
      await cwJog(CWZ.plan[i].q);
    },
    settle: cwFineSettle,
    capture: async (i) => {
      cwSetStatus(t('cw.cap.capturing', { i: i + 1 }));
      const frames = [];
      for (let k = 0; k < CWF.frames; k++) { const v = await cwCapture(); if (v.err) return v; frames.push(v); }
      return { frames, settled: CWZ.settled };
    },
    accept: (v, i, p) => {
      const m = v.err ? 0 : Math.min(...v.frames.map((f) => CW.boardMarkers(f, CWZ.board).length));
      if (v.err || m < CW.GATES.minMarkers) { p.state = 'skip'; p.why = v.err ? t('cw.skip.' + (v.err === 'moved' ? 'moved' : 'det')) : t('cw.skip.markers', { m }); return; }
      p.state = 'ok'; p.markers = m; p.settled = v.settled;
      v.frames.forEach((f, k) => CWZ.views.push({ ...f, pose: i, frame: k, ds: CWZ.place, settled: v.settled }));
    },
    after: () => cwRender(),
    stop: cwScopedStop(CWZ.controlSnap),
    skipWhy: (kind, d) => (kind === 'move' ? t('cw.skip.move', { code: d }) : t('cw.skip.transition', { r: d })),
  };
}
// capture A finished: keep its plan, prompt for the board move
function cwFineToB() {
  CWZ.saved[0] = { plan: CWZ.plan, idx: CWZ.idx, est: CWZ.est };
  CWZ.place = 1; CWZ.plan = []; CWZ.idx = 0; CWZ.step = 'move'; CWZ.status = ''; CWZ.search = null;
  cwRender(); cwV3Sync();
}
/* ---- place / move steps: three ways to bring the camera onto the board
   Jog: the panel hides (.min .jog) and a one-row bar (#cwiz-jogbar) shows the live marker count, "Back" and "Capture
   start view"; it docks at the top edge of the video (dockOnVideo) so the board stays visible, never over the sticks /
   pads, E-stop and Enable (CWJ_AVOID).
   Hand-guide: the cockpit's gravity compensation (gcBlockText checks, POST /api/v1/mode/gravity_comp); the start view
   is captured only after the exit has locked the arm (control_mode position, no phase) and the arm is still.
   Search: move_to over a grid of board centres on the table (CW_SEARCH), camera CW_SEARCH.h above the table looking
   down, through CW.runPoses (same checks, speed caps, skip / abort rules and Stop as a capture run); one ["aruco"]
   trigger per candidate; at the first view with >= minMarkers board markers one re-aim at the ground point of the
   marker centroid, then stop. */
const CW_SEARCH = { h: 0.31, radii: [0.20, 0.285, 0.37, 0.45], azDeg: [-70, -42, -14, 14, 42, 70], max: 25, rollsDeg: [0, 180, 30, -30, 60, -60, 90], clearance: 0.10, nearA: 0.08 };
const CWJ_AVOID = '#btn-estop, #btn-stop, #btn-enable, #pad-sxy, #pad-srot, #pad-sz, #pad-sgrip, #btn-pose, .cart-ctl, #stick-zone, #pad-left, #pad-right, #pad-wrist, .grip-state';
const cwPlaceStep = (k = CWZ.step) => k === 'place' || k === 'move';
const cwGcBusy = () => gcOn() || !!(S.obs && S.obs.gravity_comp && S.obs.gravity_comp.phase);
const cwMoving = () => !!(S.obs && Array.isArray(S.obs.velocities) && S.obs.velocities.some((v) => Math.abs(v) > 0.02));
// why "Capture start view" is not available now (null: it is)
function cwBootWhy(n = cwLiveCount()) {
  if (CWZ.search && CWZ.search.running) return t('cw.find.why.search');
  if (cwGcBusy()) return t('cw.find.why.gc');
  if (cwMoving()) return t('calib.e.moving');
  const min = CWZ.CW ? CWZ.CW.GATES.minMarkers : 6;
  if (n < min) return t('cw.find.why.few', { n, min });
  return null;
}
function cwJogMode(on) {
  CWZ.jog = !!on && CWZ.open && cwPlaceStep();
  const panel = $('cwiz-panel');
  panel.classList.toggle('jog', CWZ.jog);
  if (CWZ.jog) panel.classList.add('min'); else panel.classList.remove('min');
  $('cwiz-jogbar').hidden = !CWZ.jog;
  cwJogRender();
  if (!CWZ.jog) cwRender();
}
function cwJogRender() {
  const bar = $('cwiz-jogbar');
  if (CWZ.jog && !(CWZ.open && cwPlaceStep())) { cwJogMode(false); return; }
  if (bar.hidden) return;
  const n = cwLiveCount(), why = cwBootWhy(n), min = CWZ.CW ? CWZ.CW.GATES.minMarkers : 6;
  const live = $('cwiz-jog-live');
  setText(live, t('cw.jog.live', { n, min }));
  live.classList.toggle('ok', n >= min);
  const w = $('cwiz-jog-why');
  setText(w, why || '');
  const cap = $('cwiz-jog-cap');
  cap.disabled = !!why || CWZ.busy;
  cap.title = why || '';
  live.title = why || '';
  setText(cap, CWZ.busy ? t('calib.cap.busy') : t('cw.place.cap'));
  setText($('cwiz-jog-back'), t('cw.find.back'));
  bar.setAttribute('aria-label', t(CWZ.step === 'move' ? 'cw.find.jog.ttl.b' : 'cw.find.jog.ttl'));
  cwJogPlace();
}
function cwJogPlace() { dockOnVideo($('cwiz-jogbar')); }
// A floating bar (position: fixed) docked at the top edge of the video area: centred on the video, pushed below the
// view switch / header and anything of DOCK_AVOID it would cover; at the top of the screen when the video is
// scrolled out of view.
const DOCK_AVOID = CWJ_AVOID + ', .topbar';
function dockOnVideo(bar, avoidSel = DOCK_AVOID) {
  if (!bar || bar.hidden) return;
  const W = window.innerWidth, H = window.innerHeight, v = $('video-box').getBoundingClientRect();
  const onVideo = v.width > 0 && v.bottom > 120 && v.top < H - 120;
  const cx = onVideo ? Math.min(W - 8, Math.max(8, v.left + v.width / 2)) : W / 2;
  bar.style.left = cx + 'px';
  bar.style.maxWidth = Math.max(160, (onVideo ? Math.min(v.width, W) : W) - 16) + 'px';
  let top = onVideo ? Math.max(0, v.top) + 8 : 8;
  bar.style.top = top + 'px';
  const vs = $('vs-video') && $('vs-video').parentElement;
  const avoid = [...document.querySelectorAll(avoidSel), vs].filter(Boolean).map((e) => e.getBoundingClientRect())
    .filter((r) => r.width > 0 && r.height > 0 && r.bottom > 0 && r.top < H);
  for (let k = 0; k < 6; k++) {
    const b = bar.getBoundingClientRect();
    const hit = avoid.filter((r) => r.left < b.right && r.right > b.left && r.top < b.bottom && r.bottom > b.top);
    if (!hit.length) break;
    top = Math.max(...hit.map((r) => r.bottom)) + 6;
    bar.style.top = top + 'px';
  }
}
async function cwJogCapture() { cwJogMode(false); await cwBootstrap(); }
// hand-guide: enter with the cockpit switch's checks; exit locks the pose
function cwHandGuide() {
  if (CWZ.busy || (CWZ.search && CWZ.search.running) || leaseModalOrPending()) return;
  if (gcOn()) { gcExit(); cwSetStatus(t('cw.find.gc.exiting')); return; }
  const why = gcBlockText();
  if (why) { cwSetStatus(t('gc.enter_fail') + why, 'warn'); return; }
  if (S.leaseGeneration == null) {
    requireLeaseOrPrompt().catch((err) => { if (!isCancelledErr(err)) cwSetStatus(t('gc.enter_fail') + err.message, 'warn'); });
    return;
  }
  releaseAll();
  cwSetStatus(t('gc.entering'));
  api('/api/v1/mode/gravity_comp', { enable: true }, { quiet: true })
    .then(() => cwSetStatus(t('cw.find.gc.on'), 'ok'))
    .catch((err) => { if (isCancelledErr(err)) return; const k = 'gc.b.' + err.code; cwSetStatus(t('gc.enter_fail') + (t(k) !== k ? t(k) : err.message), 'warn'); });
}
const cwGcSupported = () => { const gc = S.obs && S.obs.gravity_comp; return !!(gc && gc.supported) && (S.caps ? S.caps.gravity_comp !== false : true); };
// table height for the search: the board of the last estimate, else the grasp setup, else 0
async function cwTableZ() {
  const est = CWZ.est || (CWZ.saved[0] && CWZ.saved[0].est);
  if (est) return CWZ.CW.posOf(est.Tbb)[2];
  try { const r = await fetch('/api/v1/grasp/setup', { cache: 'no-store' }); if (r.ok) { const g = await r.json(), z = Number((g.setup || g).table_z_m); if (Number.isFinite(z)) return z; } } catch (_) {}
  return 0;
}
// end pose with the camera CW_SEARCH.h above (px, py, z) looking down; the first roll with an IK solution from seed q
function cwSearchPose(px, py, z, X, q) {
  const CW = CWZ.CW, base = Math.atan2(py, px) + Math.PI / 2; // image width along the azimuth
  for (const r of CW_SEARCH.rollsDeg) {
    const a = base + r * Math.PI / 180, x = [Math.cos(a), Math.sin(a), 0], zc = [0, 0, -1], y = [zc[1] * x[2] - zc[2] * x[1], zc[2] * x[0] - zc[0] * x[2], zc[0] * x[1] - zc[1] * x[0]];
    const T = CW.mul(CW.make([[x[0], y[0], zc[0]], [x[1], y[1], zc[1]], [x[2], y[2], zc[2]]], [px, py, z + CW_SEARCH.h]), CW.inv(X));
    const s = cwIk(T, q);
    if (s && s.ok) return T;
  }
  return null;
}
// ground point (table plane z) of pixel (u, v) seen from camera pose Tc
function cwGroundPoint(Tc, K, u, v, z) {
  const CW = CWZ.CW, [xn, yn] = CW.undistort(K, u, v), R = CW.rotOf(Tc), o = CW.posOf(Tc);
  const d = [R[0][0] * xn + R[0][1] * yn + R[0][2], R[1][0] * xn + R[1][1] * yn + R[1][2], R[2][0] * xn + R[2][1] * yn + R[2][2]];
  if (!(d[2] < -1e-3)) return null;
  const s = (z - o[2]) / d[2];
  return [o[0] + s * d[0], o[1] + s * d[1]];
}
// candidate board centres, nearest to the camera's current ground point first; for placement B the spot of A last
function cwSearchGrid(seed, avoidA) {
  const c = [];
  for (const r of CW_SEARCH.radii) for (const a of CW_SEARCH.azDeg) c.push([r * Math.cos(a * Math.PI / 180), r * Math.sin(a * Math.PI / 180)]);
  const key = (p) => (avoidA && Math.hypot(p[0] - avoidA[0], p[1] - avoidA[1]) < CW_SEARCH.nearA ? 10 : 0) + (seed ? Math.hypot(p[0] - seed[0], p[1] - seed[1]) : 0);
  return c.sort((p, q) => key(p) - key(q)).slice(0, CW_SEARCH.max);
}
async function cwSearch() {
  if (CWZ.running || CWZ.busy) return;
  if (!canMove()) { cwSetStatus(lockReason(), 'warn'); return; }
  if (cwGcBusy()) { cwSetStatus(t('cw.find.why.gc'), 'warn'); return; }
  if (!isLive() || !S.obs || !Array.isArray(S.obs.positions)) { cwSetStatus(t('calib.e.offline'), 'warn'); return; }
  const snap = requireLeaseSyncOrPrompt((e) => cwSetStatus(t('cw.find.aborted', { k: 0, n: 0, why: e.message }), 'warn'));
  if (!snap) return;
  CWZ.lease = snap.leaseGen; CWZ.epoch = snap.epoch; CWZ.controlSnap = snap; CWZ.ownAction = null;
  let CW;
  try {
    CW = await cwLoad();
    if (!isControlSnapValid(snap)) throw cancelledError();
  } catch (e) {
    CWZ.controlSnap = null;
    if (!isCancelledErr(e)) cwSetStatus(t('cw.find.aborted', { k: 0, n: 0, why: e.message }), 'warn');
    return;
  }
  CWZ.board ||= cwBoard();
  CWZ.running = true; CWZ.stopReq = false; CWZ.status = '';
  const se = CWZ.search = { running: true, k: 0, n: 0, phase: 'scan', found: null };
  cwRender();
  const min = CW.GATES.minMarkers, { lo, hi } = modelLimits6();
  try {
    const X = (CWZ.est && CWZ.est.X) || (CWZ.saved[0] && CWZ.saved[0].est.X) || (await cwNominalMount()).X;
    if (!isControlSnapValid(snap)) throw cancelledError();
    const K = (CWZ.est && CWZ.est.K) || CW.NOMINAL, z = await cwTableZ();
    if (!isControlSnapValid(snap)) throw cancelledError();
    const plane = { p: [0, 0, z], n: [0, 0, 1] };
    const q0 = cwModelQ(S.obs.positions), Tc0 = CW.mul(calGripper(S.obs.positions), X);
    const seed = cwGroundPoint(Tc0, K, K.cx, K.cy, z);
    const A = CWZ.place === 1 && CWZ.saved[0] ? CW.posOf(CWZ.saved[0].est.Tbb) : null;
    const plan = cwSearchGrid(seed, A).map(([x, y]) => ({ c: [x, y], T: cwSearchPose(x, y, z, X, q0), state: 'pending' })).filter((p) => p.T);
    se.n = plan.length;
    if (!plan.length) throw CW.runAbort('nopose', 'IK');
    CWZ.ownAction = null;
    const st = { plan, idx: 0 };
    let found = null;
    const io = {
      abortReason: cwAbortReason,
      check: (T) => { const q = cwModelQ(S.obs.positions); return CW.checkTransition(CALM.fk(q), q, T, { ik: cwIk, X, plane, clearance: CW_SEARCH.clearance, lo, hi, extraEndPoints: S.wsCheckPoints }); },
      move: (T, i) => { if (se.phase === 'scan') se.k = i + 1; cwRender(); return cwMove(T); },
      settle: cwSettle,
      capture: () => cwCapture(),
      accept: (v, i, p) => {
        const m = v.err ? 0 : CW.boardMarkers(v, CWZ.board).length;
        p.markers = m; p.state = m >= min ? 'ok' : 'skip';
        if (m >= min && se.phase === 'scan') found = { v, i, m };
        se.last = m;
      },
      after: () => { if (found && se.phase === 'scan') st.idx = st.plan.length; cwRender(); },
      stop: cwScopedStop(snap),
      skipWhy: (kind, d) => (kind === 'move' ? t('cw.skip.move', { code: d }) : t('cw.skip.transition', { r: d })),
    };
    try { await CW.runPoses(st, io, { runAll: true }); }
    catch (e) { if (!(e.code === 'ABORTED' && e.why === 'nopose')) throw e; }
    if (!found) { se.found = null; cwSetStatus(t('cw.find.none', { n: plan.length, min }), 'warn'); return; }
    se.found = { k: found.i + 1, m: found.m };
    // re-aim: camera above the ground point of the marker centroid
    const pts = found.v.markers.filter((mk) => CWZ.board.markers.some((b) => b.id === mk.id)).flatMap((mk) => mk.corners);
    const u = pts.reduce((s, c) => s + c[0], 0) / pts.length, v = pts.reduce((s, c) => s + c[1], 0) / pts.length;
    const g = cwGroundPoint(CW.mul(calGripper(found.v.joints), X), K, u, v, z);
    const T2 = g && cwSearchPose(g[0], g[1], z, X, cwModelQ(S.obs.positions));
    if (T2 && io.check(T2).ok) {
      if (!isControlSnapValid(snap)) throw cancelledError();
      se.phase = 'aim'; cwRender();
      const st2 = { plan: [{ c: g, T: T2, state: 'pending' }], idx: 0 };
      await CW.runPoses(st2, io, { runAll: false });
      if (st2.plan[0].markers != null) { se.found.m = st2.plan[0].markers; se.found.aimed = true; }
    }
    cwSetStatus(t('cw.find.found', { k: se.found.k, n: plan.length, m: se.found.m }), 'ok');
  } catch (e) {
    if (isCancelledErr(e)) return;
    const why = e.code === 'ABORTED' ? t('cw.abort.' + e.why) + (e.detail ? ' (' + e.detail + ')' : '') : e.message;
    cwSetStatus(t('cw.find.aborted', { k: se.k, n: se.n, why }), 'warn');
  } finally { se.running = false; CWZ.running = false; CWZ.stopReq = false; CWZ.controlSnap = null; cwRender(); }
}
// solve: worker, else sliced on this thread
async function cwFineSolveRun(data, onProgress) {
  try { const p = cwWorkerCall('solveFine', data, onProgress); if (p) return { r: await p, worker: true }; } catch (_) { /* worker failed to load: solve here */ }
  const CW = CWZ.CW;
  return { r: await CW.runSliced(CW.solveFineSteps({ ...data, onProgress })), worker: false };
}
async function cwFineSolve() {
  if (CWZ.solving || CWZ.running) return;
  const views = CWZ.views.filter((v) => !v.err && (v.ds === 0 || v.ds === 1)).map((v) => ({ markers: v.markers, q: v.q, ds: v.ds }));
  const X0 = (CWZ.saved[0] && CWZ.saved[0].est.X) || (CWZ.est && CWZ.est.X) || CWZ.X0;
  CWZ.solve = null; CWZ.dry = null; CWZ.verify = null; CWZ.step = 'solve'; CWZ.status = ''; cwvReset();
  CWZ.solving = { t0: Date.now(), stage: '', timer: setInterval(cwRender, 1000) };
  cwRender();
  try {
    const { r, worker } = await cwFineSolveRun({ views, markerSide: CWZ.markerMm / 1000, chain: CALM.CHAIN, X0, thickness: CWZ.thickMm / 1000, solvedAt: new Date().toISOString() },
      (stage) => { if (CWZ.solving) { CWZ.solving.stage = stage; cwRender(); } });
    CWZ.solve = { ...r, worker, secs: (Date.now() - CWZ.solving.t0) / 1000 };
  } catch (e) { CWZ.status = t('cw.err', { m: e.message }); CWZ.statusKind = 'warn'; }
  finally { clearInterval(CWZ.solving.timer); CWZ.solving = null; cwRender(); }
  if (CWZ.solve) cwSessionUpload();
}
/* ---- calibration sessions on the device (armd /api/v1/calibration/sessions, newest 10 kept): the raw data (board,
   views with markers, raw and model joints, plan, the solve's K, gates and level table) is uploaded after a
   placement's capture finished and after every solve, under one id (overwritten). A failed upload never blocks the
   wizard (a warning line). "Load from device" on the board step re-solves a stored session without capturing; Apply of
   a loaded session needs a fresh verify view (CW.verifyView: the board may have moved since). */
function cwSessionBody() {
  const r = CWZ.solve, kin = r && r.kin, fine = cwFine();
  const K = (r && ((r.final && r.final.K) || (r.intr && r.intr.K))) || null;
  const X0 = (CWZ.saved[0] && CWZ.saved[0].est && CWZ.saved[0].est.X) || (CWZ.est && CWZ.est.X) || CWZ.X0 || null;
  const pl = (ps) => (ps || []).map((p) => ({ q: p.q, T: p.T, dist: p.dist, tiltDeg: p.tiltDeg, azDeg: p.azDeg, rollDeg: p.rollDeg, state: p.state, why: p.why, markers: p.markers }));
  const n = (ds) => CWZ.views.filter((v) => v.ds === ds).length;
  return {
    ...(CWZ.sessionId ? { id: CWZ.sessionId } : {}), kind: CWZ.mode, created_at: CWZ.sessionAt || (CWZ.sessionAt = new Date().toISOString()),
    board: { cols: 4, rows: 3, marker_side_m: CWZ.markerMm / 1000, gap_ratio: 0.2, thickness_m: CWZ.thickMm / 1000 },
    frame: { width: 640, height: 360 }, intrinsics_K: K, X0,
    // joints: armd positions as captured (raw); q: model angles (joint map) the solver uses; T_base_end: FK at capture
    views: CWZ.views.map(({ G, ...v }) => ({ ...v, T_base_end: G })),
    plan: fine ? [pl(CWZ.saved[0] ? CWZ.saved[0].plan : CWZ.plan), pl(CWZ.place ? CWZ.plan : [])] : [pl(CWZ.plan)],
    solve: r ? { ok: r.ok, apply: r.apply || null, solved_at: (r.apply && r.apply.solved_at) || null, worker: r.worker ?? null, secs: r.secs ?? null } : null,
    gates: r ? r.gates : null, levels: (kin && kin.levels) || null,
    summary: { views: CWZ.views.length, ...(fine ? { A: n(0), B: n(1) } : {}), ok: r ? r.ok : null, level: kin ? kin.level || null : null,
      heldout_mm: kin && kin.heldOut ? [kin.heldOut.ab.mm, kin.heldOut.ba.mm] : null, uncorrected_mm: kin && kin.heldOut ? [kin.heldOut.ab0.mm, kin.heldOut.ba0.mm] : null,
      fx: K ? K.fx : null, ...(CWZ.loaded ? { loaded_from: CWZ.loaded.id } : {}) },
  };
}
async function cwSessionUpload() {
  if (!CWZ.views.length) return;
  try {
    const res = await fetch('/api/v1/calibration/sessions', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(cwSessionBody()) });
    const d = await res.json().catch(() => ({}));
    if (!res.ok) throw Object.assign(new Error((d && (d.message || d.code)) || 'HTTP ' + res.status), { code: d && d.code });
    CWZ.sessionId = d.id; CWZ.sessionWarn = '';
  } catch (e) {
    CWZ.sessionWarn = t('cw.sess.upfail', { m: e.code === 'OPERATOR_REQUIRED' ? t('err.operator_required') : e.message });
    // not a motion: stored again by itself after the next unlock (also one started from the badge)
    if (e.code === 'OPERATOR_REQUIRED') { CWZ.sessionWarn = t('cw.sess.queued'); opQueue('calib.session', cwSessionUpload); toast(CWZ.sessionWarn, 'warn', e.code, { path: '/api/v1/calibration/sessions' }); }
  }
  cwRender();
}
const cwWhen = (iso) => { const d = new Date(iso); return Number.isFinite(d.getTime()) ? d.toLocaleString(LANG === 'zh' ? 'zh-CN' : 'en-GB', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' }) : String(iso || '?'); };
async function cwSessionList() {
  CWZ.sessions = { busy: true, list: [], error: '' }; cwRender();
  try {
    const res = await fetch('/api/v1/calibration/sessions', { cache: 'no-store' });
    if (res.status === 404 || res.status === 405) { CWZ.sessions = { busy: false, list: [], error: t('cw.sess.unsupported') }; return; }
    const d = await res.json();
    if (!res.ok) throw new Error((d && d.message) || 'HTTP ' + res.status);
    CWZ.sessions = { busy: false, list: Array.isArray(d.sessions) ? d.sessions : [], error: '' };
  } catch (e) { CWZ.sessions = { busy: false, list: [], error: t('cw.sess.err', { m: e.message }) }; }
  finally { cwRender(); }
}
async function cwSessionLoad(id) {
  if (!CWZ.sessions || CWZ.sessions.busy || CWZ.running || CWZ.solving) return;
  CWZ.sessions.busy = true; cwRender();
  let s;
  try {
    const res = await fetch('/api/v1/calibration/sessions/' + encodeURIComponent(id), { cache: 'no-store' });
    s = await res.json();
    if (!res.ok) throw new Error((s && s.message) || 'HTTP ' + res.status);
    if (!Array.isArray(s.views) || !s.board) throw new Error('no views');
  } catch (e) { CWZ.sessions = { ...CWZ.sessions, busy: false, error: t('cw.sess.err', { m: e.message }) }; cwRender(); return; }
  const fine = s.kind === 'fine';
  CWZ.mode = fine ? 'fine' : 'camera';
  CWZ.markerMm = Number(s.board.marker_side_m) * 1000 || CWZ.markerMm; CWZ.thickMm = Number(s.board.thickness_m) * 1000 || 0;
  CWZ.board = cwBoard();
  CWZ.views = s.views.map(({ T_base_end, ...v }) => ({ ...v, G: T_base_end }));
  CWZ.plan = []; CWZ.idx = 0; CWZ.saved = []; CWZ.refined = true; CWZ.solve = null; CWZ.dry = null; CWZ.applied = null; CWZ.search = null;
  CWZ.X0 = Array.isArray(s.X0) ? s.X0 : null; CWZ.est = CWZ.X0 ? { X: CWZ.X0 } : null; CWZ.place = fine ? 1 : 0;
  CWZ.loaded = { id: s.id, created_at: s.created_at, kind: CWZ.mode, views: s.views.length };
  CWZ.sessionId = s.id; CWZ.sessionAt = s.created_at || null; CWZ.sessionWarn = ''; CWZ.verify = null; CWZ.sessions = null;
  if (fine) {
    if (!CWZ.X0) { CWZ.X0 = (await cwNominalMount()).X; CWZ.est = { X: CWZ.X0 }; }
    await cwFineSolve();
  } else cwSolve();
  if (CWZ.solve) { CWZ.status = t('cw.sess.loaded', { at: cwWhen(s.created_at), n: s.views.length }); CWZ.statusKind = 'ok'; cwRender(); }
}
// verify view of a loaded session: one capture here, reprojected with the solved values (CW.verifyView)
async function cwVerify() {
  const r = CWZ.solve;
  if (!r || !r.final || CWZ.busy || CWZ.running) return;
  if (cwMoving()) { cwSetStatus(t('calib.e.moving'), 'warn'); return; }
  CWZ.busy = true; CWZ.verify = null; cwSetStatus(t('calib.cap.busy'));
  try {
    const CW = CWZ.CW, v = await cwCapture();
    if (v.err) { cwSetStatus(t('cw.skip.' + (v.err === 'moved' ? 'moved' : 'det')) + (v.detail ? ' · ' + v.detail : ''), 'warn'); return; }
    const T4 = (E) => CW.make([[E.R[0], E.R[1], E.R[2]], [E.R[3], E.R[4], E.R[5]], [E.R[6], E.R[7], E.R[8]]], E.p);
    CWZ.verify = cwFine()
      ? CW.verifyView({ view: v, board: CWZ.board, order: r.kin.order, K: r.final.K, Tend: T4(CW.kinFk(CW.kinChain(CALM.CHAIN), v.q, r.kin.k)), X: r.final.X, boards: r.final.boards })
      : CW.verifyView({ view: v, board: CWZ.board, order: r.intr.order, K: r.final.K, Tend: v.G, X: r.final.X, boards: [r.final.Tbb] });
    cwSetStatus('');
  } catch (e) { cwSetStatus(t('cw.err', { m: e.message }), 'warn'); }
  finally { CWZ.busy = false; cwRender(); }
}
const cwNeedsVerify = () => !!CWZ.loaded && !(CWZ.verify && CWZ.verify.ok);
function cwVerifyBlock(body) {
  if (!CWZ.loaded) return;
  const g = CWZ.CW.KIN_GATES.heldOutMm, v = CWZ.verify;
  body.appendChild(cwEl('div', 'cwiz-msg', t('cw.sess.verify.hint', { g })));
  if (v) body.appendChild(cwEl('div', 'cwiz-msg' + (v.ok ? ' ok' : ''), v.ok ? t('cw.sess.verify.ok', { mm: cwFmt(v.mm), px: cwFmt(v.px), m: v.markers })
    : v.why === 'markers' ? t('cw.sess.verify.few', { m: v.markers }) : t('cw.sess.verify.bad', { mm: cwFmt(v.mm), g })));
}
// level table of the fine calibration (CW.solveKinematics levels)
function cwLevelsTable(body, kin) {
  if (!kin || !Array.isArray(kin.levels)) return;
  body.appendChild(cwEl('h4', 'cwiz-sub', t('cw.f.lv.title')));
  const rows = kin.levels.map((l) => {
    const tr = document.createElement('tr');
    if (l.chosen) tr.className = 'chg';
    tr.append(cwEl('td', '', t('cw.f.lv.' + l.name)), cwEl('td', '', cwFmt(l.ab) + ' mm'), cwEl('td', '', cwFmt(l.ba) + ' mm'), cwEl('td', '', cwFmt(l.mean) + ' mm'),
      cwEl('td', l.chosen ? 'ok' : '', l.chosen ? t('cw.f.lv.chosen') : l.eligible ? '' : t('cw.f.lv.worse')));
    return tr;
  });
  const tb = cwTable([t('cw.f.lv.level'), 'A → B', 'B → A', t('cw.f.lv.mean'), t('cw.f.lv.pick')], rows);
  tb.classList.add('cwiz-levels');
  body.appendChild(tb);
  body.appendChild(cwEl('p', 'cwiz-note', t('cw.f.lv.hint')));
  if (kin.level === 'L0') body.appendChild(cwEl('div', 'cwiz-msg ok cwiz-l0', t('cw.f.lv.l0')));
}
// park and turn torque off (armd applies a kinematic correction only with torque off)
async function cwFinePark() {
  if (CWZ.busy) return;
  CWZ.busy = true; cwSetStatus(t('cw.f.parking'));
  try {
    const ack = await commandRPC('/api/v1/motion/unload', {}, true, 5000);
    const id = ack && ack.action_id;
    await cwGuardedWaitPlain(() => {
      const o = S.obs || {};
      if (id != null) for (const a of [o.action, o.previous_action]) if (a && a.id === id && a.state !== 'running') return true;
      return !o.is_torqued;
    }, 60000);
    cwSetStatus(S.obs && S.obs.is_torqued ? t('cw.f.torque') : t('cw.f.parked'), S.obs && S.obs.is_torqued ? 'warn' : 'ok');
  } catch (e) { if (isCancelledErr(e)) { cwSetStatus(''); return; } cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'); }
  finally { CWZ.busy = false; cwRender(); }
}
// wait without the run-abort rules (parking is outside a run)
async function cwGuardedWaitPlain(cond, timeoutMs) {
  const t0 = Date.now();
  while (!cond()) { if (Date.now() - t0 > timeoutMs) throw Object.assign(new Error('timeout'), { code: 'TIMEOUT' }); await cwSleep(100); }
}
async function cwReview() {
  if (!CWZ.solve || !CWZ.solve.ok || CWZ.busy) return;
  CWZ.busy = true; CWZ.step = 'review'; CWZ.dry = null; cwSetStatus(t('cw.rev.loading'));
  try {
    CWZ.dry = await commandRPC('/api/v1/calibration/apply', { ...cwApplyBody(), dry_run: true }, true, 15000);
    cwSetStatus('');
  } catch (e) {
    if (isCancelledErr(e)) { if (CWZ.dry == null) CWZ.step = 'solve'; cwSetStatus(''); return; }
    cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn');
  }
  finally { CWZ.busy = false; cwRender(); }
}
async function cwApply() {
  if (!CWZ.dry || CWZ.busy || cwNeedsVerify()) return;
  if (Date.now() >= CWZ.armedUntil) { CWZ.armedUntil = Date.now() + 4000; cwRender(); setTimeout(cwRender, 4050); return; }
  if (cwFine() && S.obs && S.obs.is_torqued) { CWZ.armedUntil = 0; cwSetStatus(t('cw.f.torque'), 'warn'); return; }
  CWZ.armedUntil = 0; CWZ.busy = true; cwSetStatus(t('calib.cap.busy'));
  try {
    const body = cwApplyBody();
    CWZ.applied = await commandRPC('/api/v1/calibration/apply', { ...body, dry_run: false }, true, 30000);
    CWZ.step = 'applied'; cwSetStatus(t('cw.app.ok'), 'ok');
    // read back: what armd holds now must be what was applied (else restore the backup in the next step)
    if (cwVerifyOn() && body.acceptance) {
      try {
        const st = await (await fetch('/api/v1/calibration', { cache: 'no-store' })).json(), c = st.calibration || {};
        const ok = c.table_z_source === body.table_z_source && Math.abs((c.table_z_m ?? NaN) - body.table_z_m) < 1e-9 && c.acceptance && c.acceptance.fingerprint === body.acceptance.fingerprint;
        CWZ.readback = { ok, floor: st.floor || null, verification: st.verification || null };
        if (!ok) cwSetStatus(t('cw.app.readback'), 'warn');
      } catch (e) { CWZ.readback = { ok: false }; cwSetStatus(t('cw.app.readback'), 'warn'); }
    }
    if (cwFine()) loadSchema(); // the corrected model (kinematics_correction) for CALM / VIK
    toolsLoadStatus(); // a new calibration clears `stale`
  } catch (e) {
    if (isCancelledErr(e)) { cwSetStatus(''); return; }
    if (e.code === 'KINEMATICS_ARMED') cwSetStatus(t('cw.f.armed'), 'warn');
    else cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message + (e.step ? ' (' + e.step + ')' : '') }), 'warn');
  }
  finally { CWZ.busy = false; cwRender(); }
}
// the apply body: the candidate, with the fingertip / acceptance / hover table height once the hovers ran
const cwApplyBody = () => (cwVerifyOn() && CWV.eval ? cwvApplyBody() : CWZ.solve.apply);
const cwBackupName = (b) => (typeof b === 'string' ? b.split('/').filter(Boolean).pop() : '');
async function cwRestore() {
  const name = cwBackupName(CWZ.applied && CWZ.applied.backup);
  if (!name || CWZ.busy) return;
  if (Date.now() >= CWZ.armedUntil) { CWZ.armedUntil = Date.now() + 4000; cwRender(); setTimeout(cwRender, 4050); return; }
  CWZ.armedUntil = 0; CWZ.busy = true;
  try {
    const r = await commandRPC('/api/v1/calibration/restore', { backup: name }, true, 30000);
    CWZ.applied = { ...r, restoredFrom: name };
    toolsLoadStatus();
    cwSetStatus(t('cw.app.restored', { b: name }), 'ok');
    if (cwFine()) loadSchema();
  } catch (e) { if (isCancelledErr(e)) { cwSetStatus(''); return; } cwSetStatus(t('cw.err', { m: (e.code ? e.code + ': ' : '') + e.message }), 'warn'); }
  finally { CWZ.busy = false; cwRender(); }
}
