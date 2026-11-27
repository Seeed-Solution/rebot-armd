/* =================================================================
   Telemetry WebSocket (50 Hz) + heartbeat
   ================================================================= */
// Reconnect after a dropped socket: one timer, 1, 2, 4, 8 s, then every 15 s. The first valid observation
// on the new socket resets the backoff and clears S.connLost (the "connection lost" warning). A socket that
// delivers no valid observation for WS_TELEMETRY_DEADLINE_MS (opened but silent, or half-open) is retired
// through the same path, so its retry keeps the backoff.
const WS_RETRY_MS = [1000, 2000, 4000, 8000, 15000], WS_TELEMETRY_DEADLINE_MS = 4000;
let wsOpenedOnce = false, wsReconnectTimer = null, wsRetryN = 0;
const VALID_SAFETY_STR = new Set(['Booting', 'Idle', 'Armed', 'Moving', 'EStop']), VALID_FAULT_STR = new Set(['BusLoss', 'WatchdogTimeout', 'CanTxStalled']);
const isPlainObj = (v) => !!(v && typeof v === 'object' && !Array.isArray(v));
function isValidSafetyState(s) {
  if (typeof s === 'string') return VALID_SAFETY_STR.has(s);
  if (!isPlainObj(s) || Object.keys(s).length !== 1 || !('Fault' in s)) return false;
  const r = s.Fault;
  if (typeof r === 'string') return VALID_FAULT_STR.has(r);
  if (!isPlainObj(r)) return false;
  const rk = Object.keys(r), k = rk[0], v = r[k];
  if (rk.length !== 1) return false;
  if (k === 'Other') return typeof v === 'string';
  if (k === 'JointLimitExceeded') return isPlainObj(v) && Number.isSafeInteger(v.joint_idx) && v.joint_idx >= 0 && typeof v.joint_name === 'string' && Number.isFinite(v.value) && Number.isFinite(v.min) && Number.isFinite(v.max);
  return k === 'ThermalTrip' && isPlainObj(v) && Number.isSafeInteger(v.joint_idx) && v.joint_idx >= 0 && Number.isFinite(v.temperature) && Number.isFinite(v.limit);
}
const isCompleteObservation = (o) => isPlainObj(o) && Array.isArray(o.positions) && (S.schemaLoaded && S.dof > 0 ? o.positions.length === S.dof : o.positions.length > 0) && o.positions.every(Number.isFinite) && Number.isFinite(o.timestamp) && Number.isSafeInteger(o.epoch_id) && o.epoch_id >= 0 && typeof o.is_torqued === 'boolean' && isValidSafetyState(o.safety_state) && (o.lease_holder === null || typeof o.lease_holder === 'string');
const clearReconnectTimer = () => { if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; } };
const scheduleReconnect = () => {
  clearReconnectTimer();
  const delayMs = WS_RETRY_MS[Math.min(wsRetryN, WS_RETRY_MS.length - 1)];
  wsRetryN++;
  wsReconnectTimer = setTimeout(() => { wsReconnectTimer = null; if (!S.ws) connectWS(); }, delayMs);
};
// Per-socket telemetry deadline: retire `ws` when no valid observation arrived on it for the deadline.
function armTelemetryDeadline(ws) {
  ws.lastValidAt = Date.now();
  ws.deadlineTimer = setInterval(() => {
    if (S.ws !== ws) { clearInterval(ws.deadlineTimer); return; }
    if (Date.now() - ws.lastValidAt > WS_TELEMETRY_DEADLINE_MS) retireSocket(ws, { reconnect: true, markReleased: true, toastExpired: true });
  }, 500);
}
// Returning to the tab: an already-queued retry or a socket still connecting keeps its schedule; a dead or
// stale socket is retired like a drop, and only a socket that had valid telemetry retries at once.
function resumeConnection() {
  if (wsReconnectTimer) return false;
  const ws = S.ws;
  if (ws && ws.readyState === 0) return false;
  if (ws && ws.readyState === 1 && Date.now() - (ws.lastValidAt || 0) <= 1500) return false;
  const hadTelemetry = !ws || wsRetryN === 0;
  if (ws) retireSocket(ws, { reconnect: true, markReleased: true, toastExpired: true });
  if (hadTelemetry) connectWS();
  return true;
}
// Arm state for the release notice, from fresh telemetry only. "Parked" needs armd's evidence (a finished
// park action and torque off): a clean close can reconnect while the arm is still torqued.
function connArmText(o) {
  if (!o) return '';
  const a = o.action, parking = !!o.disable_pending || (a?.kind === 'park' && a?.state === 'running');
  const held = o.watchdog_park_blocked_reason;
  if (o.is_torqued && typeof held === 'string' && held) return t('conn.arm.held', { reason: held });
  if (parking) return t('conn.arm.parking');
  if (!o.is_torqued) return t(a?.kind === 'park' && a?.state === 'done' ? 'conn.arm.parked' : 'conn.arm.off');
  return t('conn.arm.torqued');
}
function retireSocket(ws, { reconnect = false, markReleased = false, toastExpired = false } = {}) {
  if (!ws || S.ws !== ws) return;
  const hadLease = S.leaseGeneration != null;
  // A dropped socket that had delivered telemetry (or a retry during an outage) shows the warning until the
  // next socket delivers a valid observation.
  if (reconnect && ((S.wsObsSeen && S.obsWs === ws) || S.connLost)) S.connLost = true;
  if (markReleased && hadLease) S.leaseReleased = { reason: 'reconnect' };
  if (toastExpired && hadLease) toast(t('toast.lease_expired'), 'warn');
  S.ws = null; S.wsObsSeen = false; S.obsWs = null; S.obsTs = -Infinity; S.minObsTs = -Infinity; S.epoch = null;
  clearInterval(ws.deadlineTimer);
  ws.onclose = null; ws.onerror = null; try { ws.close(); } catch (_) {}
  S.detections = []; S.detLastReceivedAt = 0; clearOverlayCanvas();
  if (leaseOp) { const op = leaseOp; leaseOp = null; if (op.timer) clearTimeout(op.timer); if (op.reject) op.reject(Object.assign(new Error(t('err.nolease')), { code: 'LEASE_REQUIRED' })); }
  invalidateControlIntents({ dropOp: true }); resetLease();
  if (reconnect) { scheduleReconnect(); loadOwnerDiagnostics(); }
}
function applyObservation(o, fromWs) {
  const prevObsOwner = S.obs && isValidEffectiveOwner(S.obs.effective_owner) ? S.obs.effective_owner : null;
  S.obs = o; S.obsAt = Date.now(); if (fromWs) S.wsCount++;
  if (preemptModalOpen() && !leaseOp && S.leaseGeneration == null) {
    const cur = (o.lease_holder === null || o.lease_holder === CLIENT_ID) ? null : o.lease_holder;
    if (cur == null) { closePreemptModal(); toast(t('preempt.cleared'), 'info'); }
    else if (cur !== preemptTargetHolder) { preemptTargetHolder = cur; preemptChangedAck = true; preemptChangedUntil = Date.now() + 800; renderPreemptModal(); }
  }
  detHoldOnObs(o); depthOnObs(o); gcOnObs(o);
  renderOwnerStatus();
  if (isValidEffectiveOwner(o.effective_owner) && (!S.ownerDiag || S.ownerDiag.effective_owner !== o.effective_owner || S.ownerDiag.owner_reason !== (typeof o.owner_reason === 'string' ? o.owner_reason : ''))) {
    loadOwnerDiagnostics();
  }
  if (prevObsOwner !== o.effective_owner && typeof gtRender === 'function' && $('gtool') && !$('gtool').hidden) gtRender();
  renderMotionLimit(o); tkOnObs(o); renderPower(o); graspOnObs(o); msOnObs(o); epOnObs(o); teachOnObs(o);
  syncEpoch(o.epoch_id);
  const st = parseState(o.safety_state), wasStopped = isStopped();
  S.state = st.name; S.fault = st.detail; S.faultCode = o.fault && typeof o.fault.code === 'string' ? o.fault.code : null;
  if (isStopped() && !wasStopped) { invalidateControlIntents(); resetPoseMode(); }
  if (S.presetRunning) {
    const presetAge = S.preset?.startedAt ? (Date.now() - S.preset.startedAt) : 0;
    const speedsNearZero = Array.isArray(o.velocities) && o.velocities.every((v) => Math.abs(v) < 0.05);
    if (st.name === 'EStop' || st.name === 'Fault' || (st.name === 'Armed' && o.action?.state !== 'running' && presetAge > 500) || (presetAge > 1000 && speedsNearZero)) { S.presetRunning = false; S.preset = null; }
  }
  if (fromWs && o.detections && !(performance.now() < (S.detHoldUntil || 0))) { S.detections = parseDetections(o.detections, [o.width, o.height]); S.detSource = 'telemetry_ws'; S.detLastReceivedAt = performance.now(); }
  if (o.positions.length > GRIP && S.grip == null) S.grip = gripFrac(o.positions[GRIP]);
}
function acceptObservation(o, fromWs, ws = S.ws) {
  if (!ws || S.ws !== ws || !isCompleteObservation(o)) return 'invalid';
  if (S.epoch != null && o.epoch_id < S.epoch) return 'stale';
  const ts = o.timestamp;
  if (fromWs) {
    if ((S.minObsTs !== -Infinity && ts <= S.minObsTs) || (S.obsTs !== -Infinity && ts <= S.obsTs)) return 'stale';
    S.wsObsSeen = true; S.obsWs = ws; S.obsTs = ts; ws.lastValidAt = Date.now(); wsRetryN = 0; S.connLost = false; applyObservation(o, true); return 'applied';
  }
  S.minObsTs = Math.max(S.minObsTs, ts);
  if (!S.wsObsSeen || S.obsWs !== ws || ts > S.obsTs) { S.wsObsSeen = true; S.obsWs = ws; S.obsTs = ts; applyObservation(o, false); return 'applied'; }
  if (ts === S.obsTs && S.obs && S.obs.lease_holder === CLIENT_ID && o.lease_holder !== CLIENT_ID) S.obs = { ...S.obs, lease_holder: o.lease_holder };
  return 'stale';
}
function connectWS() {
  clearReconnectTimer();
  if (S.ws) retireSocket(S.ws, { reconnect: false, markReleased: true });
  S.wsObsSeen = false; S.obsWs = null; S.obsTs = -Infinity; S.minObsTs = -Infinity; S.epoch = null;
  invalidateControlIntents({ dropOp: true });
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const ws = new WebSocket(`${proto}//${location.host}/api/v1/ws`);
  S.ws = ws;
  armTelemetryDeadline(ws);
  resetLease();
  S.sequenceId = 0;
  ws.onopen = () => {
    if (S.ws !== ws) return;
    if (wsOpenedOnce) { loadAuthStatus(); loadOwnerDiagnostics(); loadSchema(); }
    wsOpenedOnce = true;
  };
  // Never acquire_lease on open: the page connects read-only until explicit Enable or operator confirmation.
  ws.onmessage = (ev) => {
    let o; try { o = JSON.parse(ev.data); } catch (_) { return; }
    if (S.ws !== ws) return;
    if (o.type === 'command_ack' || o.type === 'command_reject') {
      const pending = pendingCommands.get(o.request_id);
      if (!pending || pending.ws !== ws) return;
      clearTimeout(pending.timer); pendingCommands.delete(o.request_id);
      if (o.type === 'command_ack') { if (o.action_id != null) tkOwnAction(o.action_id); pending.resolve(o); }
      else pending.reject(rpcError(o));
      syncEpoch(o.epoch_id);
      return;
    }
    if (o.type === 'lease_granted' || o.type === 'lease_rejected') {
      onAcquireResponse(ws, o);
      return;
    }
    if (o.type === 'lease_released') { onLeaseReleased(ws, o); return; }
    if (o.type === 'auth_ack') {
      if (OP.ack && OP.ack.ws === ws && OP.ack.seq === o.seq) OP.ack.done(o);
      return;
    }
    if (o.type === 'lease_lost') {
      onLeaseLost(ws, o);
      return;
    }
    if (o.type === 'epoch_sync') {
      if (!Number.isSafeInteger(o.epoch_id) || o.epoch_id < 0 || (S.epoch != null && o.epoch_id < S.epoch)) return;
      if (o.safety_state !== undefined && !isValidSafetyState(o.safety_state)) return;
      syncEpoch(o.epoch_id);
      if (o.safety_state) {
        const st = parseState(o.safety_state);
        S.state = st.name; S.fault = st.detail;
      }
      return;
    }
    if (o.type === 'servo_reject') {
      syncEpoch(o.epoch_id);
      // A jog frame that was already in flight when the stop (joystick release, Stop button)
      // was applied is refused with STOPPED. That is the expected preemption, not an error.
      if (o.code === 'STOPPED' || o.reason === 'stopped') return;
      if (o.code === 'WORKSPACE_VIOLATION') {
        const cid = o.constraint_id;
        const gid = o.geometry_id;
        const boxMsg = formatWorkspaceViolation(o);
        const nonFloor = !!boxMsg
          || (cid && cid !== 'floor_z')
          || (gid && gid.startsWith('capsule:'))
          || (!cid && typeof o.message === 'string' && /keepout|box_|reach|capsule:/i.test(o.message));
        const detail = boxMsg || o.reason || o.message;
        if (nonFloor && detail) {
          if (S.mode === 'cart') cartReject(detail);
          else toastOnce({ message: detail, code: o.code });
        } else {
          // server-side Cartesian safety box (floor / boundary incl. the gripper envelope)
          if (S.mode === 'cart') cartReject('floor');
          else toastOnce({ message: t('err.workspace'), code: o.code });
        }
        return;
      }
      const msg = o.code === 'HANDOVER_SETTLING'
        ? t('err.settling2')
        : o.code === 'COLLISION_HOLD' ? t('err.collision_hold')
        : t('err.servo_reject') + (o.message || o.error || t('err.state_limit'));
      toastOnce({ message: msg, code: o.code || 'SERVO_REJECT' });
      return;
    }
    if (o.type === 'preset_ack') {
      syncEpoch(o.epoch_id);
      if (o.action_id != null) tkOwnAction(o.action_id);
      toast(t('toast.goto', { name: presetName(o.name) || o.name || t('pn.default') }), 'ok');
      return;
    }
    if (o.type === 'preset_reject') {
      syncEpoch(o.epoch_id);
      S.presetRunning = false;
      S.preset = null;
      const boxMsg = formatWorkspaceViolation(o);
      const nonFloor = !!boxMsg
        || (o.constraint_id && o.constraint_id !== 'floor_z')
        || (o.geometry_id && o.geometry_id.startsWith('capsule:'))
        || (!o.constraint_id && typeof o.message === 'string' && /keepout|box_|reach|capsule:/i.test(o.message));
      const wsMsg = boxMsg || (nonFloor ? (o.reason || o.message || t('err.workspace')) : t('err.workspace'));
      const msg = o.code === 'HANDOVER_SETTLING'
        ? t('err.settling3')
        : o.code === 'COLLISION_HOLD' ? t('err.collision_hold')
        : o.code === 'WORKSPACE_VIOLATION' ? wsMsg
        : (o.code === 'UNSUPPORTED' ? t('err.preset_unsupported') : t('err.preset_reject') + (o.message || o.error || o.reason || t('err.state_limit')));
      toast(msg, 'err');
      return;
    }
    acceptObservation(o, true, ws);
  };
  ws.onclose = () => onSocketClose(ws);
  ws.onerror = () => {
    S.detections = [];
    S.detLastReceivedAt = 0;
    clearOverlayCanvas();
    try { ws.close(); } catch (_) {}
  };
}
// Toast key for a lost lease. armd sends reason "preempted" (forced acquire by another
// client) or "expired" (no heartbeat within the lease timeout); older armd builds omit the
// reason except on preemption, so a missing reason means the lease ran out.
function leaseLossKey(reason) {
  if (reason === 'preempted') return 'toast.lease_preempted';
  if (reason === 'released') return 'toast.lease_released';
  return 'toast.lease_expired';
}
function onLeaseLost(ws, o) {
  // Only a loss of the generation this page holds demotes it; a stale notice for an older generation is ignored.
  if (o.generation != null ? o.generation === S.leaseGeneration : S.leaseGeneration != null) {
    if (o.reason !== 'released' && o.reason !== 'preempted') S.leaseReleased = { reason: 'expired' };
    if (typeof S.controlEpoch === 'number') S.controlEpoch++;
    resetLease();
    if (typeof onLeaseLostCleanup === 'function') onLeaseLostCleanup();
    toast(t(leaseLossKey(o.reason)), 'warn');
  }
}
function onLeaseLostCleanup() { invalidateControlIntents(); }
function onSocketClose(ws) { retireSocket(ws, { reconnect: true, markReleased: true, toastExpired: true }); }
function resetLease() {
  rejectPendingCommands(t('err.disconnected'));
  S.leaseGeneration = null; S.hb.at = 0; S.hb.n = 0; S.hb.hz = 0;
  releaseAll({ sendStop: false }); resetPoseMode();
}

/* ---------- single exclusive leaseOp lifecycle, physical-key gate & stop dispatch ---------- */
const STOP_BTN_SEL = '#btn-stop, #btn-estop, .head-stop, .head-estop, .op-stop, .op-estop, #grasp-stop, #ms-stop, #ep-stop, #tch-stop, #tch-pick-stop, #cwiz-stop, #cwiz-find-stop';
const isDeliberateStopTarget = (el) => !!(el && el.closest && el.closest(STOP_BTN_SEL));
const bindStopButton = (el, opts, before = null) => { if (!el) return; const run = () => { if (before) before(); stopMotion(opts); }; el.addEventListener('pointerdown', (e) => { if (e.button === 0 || e.pointerType !== 'mouse') { e.preventDefault(); run(); } }); el.addEventListener('click', (e) => { if (e.detail === 0) run(); }); };
const bindEstopButton = (el) => { if (!el) return; el.addEventListener('pointerdown', (e) => { if (e.button === 0 || e.pointerType !== 'mouse') { e.preventDefault(); eStop(); } }); el.addEventListener('click', (e) => { if (e.detail === 0) eStop(); }); };
const holdCancels = [], heldCodes = new Set(), suppressedCodes = new Set();
const suppressHeldConfirmKeys = () => { for (const c of ['Enter', 'NumpadEnter']) if (heldCodes.has(c)) suppressedCodes.add(c); };
let acquireSeq = 0, leaseOp = null, leaseModalReturnFocus = null;
const takeModalOpen = () => !!$('take-modal') && !$('take-modal').hidden;
const preemptModalOpen = () => !!$('preempt-modal') && !$('preempt-modal').hidden;
const leaseModalOrPending = () => !!(takeModalOpen() || preemptModalOpen() || leaseOp);
const isCancelledErr = (e) => !!(e && (e.cancelled || e.code === 'CONTROL_CANCELLED'));
const cancelledError = () => Object.assign(new Error(t('err.take_cancelled')), { code: 'CONTROL_CANCELLED', cancelled: true });
const leaseUnavailableError = () => (S.ws && S.ws.readyState === 1 && !obsFresh() ? Object.assign(new Error(t('err.telemetry_stale')), { code: 'TELEMETRY_STALE' }) : Object.assign(new Error(t('err.nolease')), { code: 'LEASE_REQUIRED' }));
function cancelPendingAcquire() {
  if (leaseOp && leaseOp.kind === 'acquire') {
    leaseOp.cancelled = true;
    if (leaseOp.reject) { const rej = leaseOp.reject; leaseOp.reject = leaseOp.resolve = null; rej(cancelledError()); }
  }
}
function restoreLeaseModalFocus() {
  if (takeModalOpen() || preemptModalOpen() || opIsOpen()) return;
  const f = leaseModalReturnFocus; leaseModalReturnFocus = null;
  if (f && typeof f.focus === 'function' && document.contains(f)) { try { f.focus({ preventScroll: true }); } catch (_) {} }
}
const dismissTakeModal = () => { cancelPendingAcquire(); closeTakeModal(); };
const dismissPreemptModal = () => { cancelPendingAcquire(); closePreemptModal(); };
const captureControlSnap = () => ({ ws: S.ws, leaseGen: S.leaseGeneration, epoch: S.obs && typeof S.obs.epoch_id === 'number' ? S.obs.epoch_id : S.epoch, controlEpoch: S.controlEpoch, stopped: false });
function isControlSnapValid(snap) {
  if (!snap || snap.leaseGen == null) return false;
  const curEpoch = S.obs && typeof S.obs.epoch_id === 'number' ? S.obs.epoch_id : S.epoch;
  return S.ws === snap.ws && S.leaseGeneration === snap.leaseGen && curEpoch === snap.epoch && S.controlEpoch === snap.controlEpoch;
}
function invalidateControlIntents({ dropOp = false, keepReleaseOp = false } = {}) {
  S.controlEpoch++;
  for (const fn of holdCancels) { try { fn(); } catch (_) {} }
  closeTakeModal(); closePreemptModal(); releaseAll({ sendStop: false });
  if (leaseOp && leaseOp.kind === 'acquire') {
    if (dropOp) { const op = leaseOp; leaseOp = null; if (op.timer) clearTimeout(op.timer); if (op.reject) op.reject(cancelledError()); }
    else cancelPendingAcquire();
  } else if (dropOp && leaseOp && !(keepReleaseOp && leaseOp.kind === 'release')) {
    if (leaseOp.timer) clearTimeout(leaseOp.timer); leaseOp = null;
  }
}
function leaseOther() {
  if (S.leaseGeneration != null || !S.obs || !obsFresh() || leaseOp) return null;
  const h = S.obs.lease_holder;
  return (h == null || h === CLIENT_ID) ? null : h;
}
function leaseHolderText(h) {
  if (h == null) return t('who.other');
  if (h === '') return t('who.unnamed');
  if (/^rebot-policy/.test(h)) return t('who.policy', { id: h });
  if (/^rebot-(app|teleop)/.test(h)) return t('who.app', { id: h });
  if (/^rebot-(mcp|companion)/.test(h)) return t('who.agent', { id: h });
  if (/^cyclops_/.test(h)) return t('who.wizard', { id: h });
  if (/^web-/.test(h)) return t('who.cockpit');
  return t('who.named', { id: h });
}
const leaseHolderDetail = (h) => (typeof h === 'string' && /^web-/.test(h) ? t('who.cockpit_id', { id: h }) : leaseHolderText(h));
const leaseBusyError = (h) => Object.assign(new Error(t('toast.lease_busy', { who: leaseHolderDetail(h) })), { code: 'LEASE_ACTIVE' });

function acquireLeaseDirect() {
  const snap = { ws: S.ws, epoch: S.epoch, controlEpoch: S.controlEpoch };
  if (S.leaseGeneration != null) return Promise.resolve(S.leaseGeneration);
  if (!obsFresh()) return Promise.reject(leaseUnavailableError());
  if (leaseOp) return Promise.reject(cancelledError());
  const other = leaseOther();
  if (other != null) return Promise.reject(leaseBusyError(other));
  return new Promise((resolve, reject) => { startAcquireOp(S.ws, false, { resolve, reject }); }).then((gen) => {
    if (S.ws !== snap.ws || S.epoch !== snap.epoch || S.controlEpoch !== snap.controlEpoch) throw cancelledError();
    return gen;
  });
}
function promptTakeControl() {
  if (S.leaseGeneration != null || leaseModalOrPending()) return;
  if (!obsFresh()) { const err = leaseUnavailableError(); toast(err.message, 'err', err.code); return; }
  const other = leaseOther();
  if (other != null) { toast(leaseBusyError(other).message, 'err', 'LEASE_ACTIVE'); return; }
  releaseAll({ sendStop: false }); openTakeModal();
}
function requireLeaseOrPrompt() {
  if (S.leaseGeneration != null) return Promise.resolve(S.leaseGeneration);
  if (leaseModalOrPending()) return Promise.reject(cancelledError());
  if (!obsFresh()) return Promise.reject(leaseUnavailableError());
  const other = leaseOther();
  if (other != null) return Promise.reject(leaseBusyError(other));
  promptTakeControl();
  return Promise.reject(cancelledError());
}
function requireLeaseSyncOrPrompt(onError) {
  if (S.leaseGeneration != null) return captureControlSnap();
  if (leaseModalOrPending()) return null;
  if (!obsFresh()) { const err = leaseUnavailableError(); if (onError) onError(err); else toast(err.message, 'err', err.code); return null; }
  const other = leaseOther();
  if (other != null) { const err = leaseBusyError(other); if (onError) onError(err); else toast(err.message, 'err', err.code); return null; }
  promptTakeControl();
  return null;
}
function leaseIdleReleasable() {
  const o = S.obs;
  if (!o || !isLive() || S.leaseGeneration == null || !S.ws || S.ws.readyState !== 1) return false;
  if (Date.now() - S.lastIntentAt < CFG.leaseIdleMs) return false;
  if (o.is_torqued || o.disable_pending || o.enable_state === 'Enabled') return false;
  if (o.action?.state === 'running' || o.control_mode === 'gravity_comp' || o.gravity_comp?.phase) return false;
  if (S.presetRunning || S.streamActive || S.servoTarget || unloading || GR.sending || leaseOp || pendingCommands.size) return false;
  if (commandVector().some((x) => x !== 0) || Object.values(cartVelocity()).some((x) => x !== 0) || cartGripVel() !== 0) return false;
  if (S.state === 'Moving' || !Array.isArray(o.velocities) || o.velocities.some((v) => !(Math.abs(v) < 0.05))) return false;
  return true;
}
setInterval(() => { if (leaseIdleReleasable()) sendReleaseLease('toast.lease_idle', { s: Math.round(CFG.leaseIdleMs / 1000) }); }, 250);

// The SOLE sender of acquire_lease in the entire file. force=true only from #preempt-confirm.
function startAcquireOp(ws, force = false, { resolve = null, reject = null } = {}) {
  if (!ws || S.ws !== ws || !obsFresh() || leaseOp) {
    if (reject) reject(leaseOp ? cancelledError() : leaseUnavailableError());
    return false;
  }
  const seq = ++acquireSeq;
  const op = { kind: 'acquire', ws, seq, force: !!force, cancelled: false, timer: null, resolve, reject };
  op.timer = setTimeout(() => {
    if (leaseOp !== op || S.ws !== ws) return;
    clearTimeout(op.timer); toast(t('toast.take_timeout'), 'warn');
    retireSocket(ws, { reconnect: true, markReleased: false });
  }, CFG.leaseAckTimeoutMs);
  leaseOp = op;
  try { ws.send(JSON.stringify({ type: 'acquire_lease', seq, client_id: CLIENT_ID, force: !!force })); }
  catch (e) { clearTimeout(op.timer); leaseOp = null; if (reject) reject(e); renderPreemptModal(); renderTakeModal(); return false; }
  renderPreemptModal(); renderTakeModal();
  return true;
}
function onAcquireResponse(ws, msg) {
  if (S.ws !== ws) return;
  const validGen = typeof msg.generation === 'number' && Number.isSafeInteger(msg.generation) && msg.generation > 0;
  const correlated = !!(leaseOp && leaseOp.kind === 'acquire' && leaseOp.ws === ws && typeof msg.seq === 'number' && leaseOp.seq === msg.seq);
  if (msg.type === 'lease_granted' && (!validGen || !correlated)) {
    const pending = leaseOp && leaseOp.ws === ws ? leaseOp : null;
    let rej = null;
    if (pending) { clearTimeout(pending.timer); rej = pending.reject; pending.reject = pending.resolve = null; leaseOp = null; }
    closePreemptModal(); closeTakeModal();
    if (rej) rej(Object.assign(new Error(t('toast.take_ambiguous')), { code: 'LEASE_REJECTED' }));
    else toast(t('toast.take_ambiguous'), 'warn', 'LEASE_REJECTED');
    retireSocket(ws, { reconnect: true, markReleased: false });
    return;
  }
  if (!correlated) return;
  const op = leaseOp;
  clearTimeout(op.timer);
  if (op.cancelled) {
    leaseOp = null;
    if (msg.type === 'lease_granted') startReleaseOp(ws, msg.generation, null, null);
    renderPreemptModal(); renderTakeModal();
    return;
  }
  leaseOp = null;
  if (msg.type === 'lease_granted') {
    S.leaseGeneration = msg.generation; sendHeartbeat(ws); S.lastIntentAt = Date.now(); S.leaseReleased = null;
    closePreemptModal(); closeTakeModal();
    if (op.force) toast(t('toast.took'), 'ok');
    if (op.resolve) op.resolve(msg.generation);
    return;
  }
  if (msg.error === 'preemption_cooldown') {
    S.preemptCooldownUntil = Date.now() + Math.max(0, Number(msg.remaining_ms) || 0);
    closeTakeModal(); if (!preemptModalOpen()) openPreemptModal(); else renderPreemptModal();
    if (op.reject) op.reject(leaseBusyError(S.obs?.lease_holder));
    return;
  }
  if (msg.error === 'controller_busy') {
    if (op.force) { closePreemptModal(); toast(t('toast.take_failed'), 'err'); }
    const busy = leaseBusyError(S.obs?.lease_holder);
    if (!op.force && !op.reject) toastOnce(busy);
    if (op.reject) op.reject(busy);
    renderPreemptModal(); renderTakeModal();
    return;
  }
  const why = msg.message || msg.error || t('unknown_reason');
  if (op.force) {
    closePreemptModal();
    if (msg.code !== 'OPERATOR_REQUIRED') {
      if (msg.code === 'OWNER_TRANSFERRED' || msg.error === 'owner_transferred') toast(t('toast.take_owner_transferred'), 'err', msg.code || 'OWNER_TRANSFERRED');
      else toast(t('toast.take_rejected_fix', { why }), 'err', msg.code || 'LEASE_REJECTED');
    }
  }
  if (msg.code === 'OPERATOR_REQUIRED') opRequire('lease.force', undefined, { retry: false }).then((ok) => { if (ok) toast(t('op.again.take'), 'info'); });
  if (op.reject) op.reject(Object.assign(new Error(t('err.rejected_fix', { why })), { code: msg.code || 'LEASE_REJECTED' }));
  renderPreemptModal(); renderTakeModal();
}
function startReleaseOp(ws, generation, toastKey = null, toastArgs = null) {
  if (!ws || ws.readyState !== 1 || S.ws !== ws) return false;
  S.leaseGeneration = null; S.hb.at = 0; S.hb.n = 0; S.hb.hz = 0;
  const op = { kind: 'release', ws, cancelled: false, timer: null, toastKey, toastArgs };
  op.timer = setTimeout(() => {
    if (leaseOp !== op || S.ws !== ws) return;
    clearTimeout(op.timer); toast(t('toast.release_timeout'), 'warn'); retireSocket(ws, { reconnect: true, markReleased: false });
  }, CFG.leaseAckTimeoutMs);
  leaseOp = op;
  try { ws.send(JSON.stringify({ type: 'release_lease', client_id: CLIENT_ID, generation })); }
  catch (_) { clearTimeout(op.timer); leaseOp = null; retireSocket(ws, { reconnect: true, markReleased: false }); return false; }
  return true;
}
function sendReleaseLease(toastKey = null, toastArgs = null) {
  if (S.leaseGeneration == null || !S.ws || S.ws.readyState !== 1 || leaseOp) return false;
  const ws = S.ws, gen = S.leaseGeneration;
  invalidateControlIntents({ keepReleaseOp: true }); resetLease();
  return startReleaseOp(ws, gen, toastKey, toastArgs);
}
function onLeaseReleased(ws, msg) {
  if (!leaseOp || leaseOp.kind !== 'release' || leaseOp.ws !== ws || S.ws !== ws) return;
  const prev = leaseOp; clearTimeout(prev.timer);
  if (prev.toastKey) toast(msg.success === true ? t(prev.toastKey, prev.toastArgs) : t('toast.release_inactive'), msg.success === true ? 'info' : 'warn');
  const op = { kind: 'refresh', ws, cancelled: false, timer: null };
  const fail = () => { if (leaseOp !== op || S.ws !== ws) return; clearTimeout(op.timer); toast(t('toast.refresh_timeout'), 'warn'); retireSocket(ws, { reconnect: true, markReleased: false }); };
  op.timer = setTimeout(fail, CFG.leaseAckTimeoutMs);
  leaseOp = op;
  fetch('/api/v1/observation', { cache: 'no-store' }).then(async (res) => {
    if (leaseOp !== op || S.ws !== ws) return;
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const obs = await res.json();
    if (leaseOp !== op || S.ws !== ws) return;
    if (!isCompleteObservation(obs)) throw new Error('invalid observation');
    clearTimeout(op.timer); leaseOp = null; acceptObservation(obs, false, ws);
    if (typeof toolsRenderRelease === 'function' && !$('sset').hidden) toolsRenderRelease();
  }).catch(fail);
}
function openTakeModal() {
  if (S.leaseGeneration != null || leaseOp || preemptModalOpen()) return;
  if (!takeModalOpen() && !preemptModalOpen()) leaseModalReturnFocus = document.activeElement;
  $('take-modal').hidden = false; renderTakeModal(); $('take-cancel').focus({ preventScroll: true });
}
function closeTakeModal() { const was = takeModalOpen(); if ($('take-modal')) $('take-modal').hidden = true; if (was) { suppressHeldConfirmKeys(); restoreLeaseModalFocus(); } }
function renderTakeModal() {
  if (!takeModalOpen()) return;
  const acquiring = !!(leaseOp && leaseOp.kind === 'acquire' && !leaseOp.force), btn = $('take-confirm');
  setText($('take-body'), t(S.leaseReleased ? 'take.body.released' : 'take.body'));
  btn.disabled = acquiring || (!!leaseOp && !acquiring) || !obsFresh() || leaseOther() != null;
  setText(btn, acquiring ? t('take.taking') : t('take.confirm'));
}
$('take-cancel').addEventListener('click', dismissTakeModal);
$('take-modal').addEventListener('click', (e) => { if (e.target === e.currentTarget) dismissTakeModal(); });
bindStopButton($('take-stop')); bindEstopButton($('take-estop'));
$('take-confirm').addEventListener('click', () => {
  if (leaseOp || !obsFresh()) return;
  const other = leaseOther();
  if (other != null) { closeTakeModal(); toast(leaseBusyError(other).message, 'err', 'LEASE_ACTIVE'); return; }
  const btn = $('take-confirm'); btn.disabled = true; setText(btn, t('take.taking'));
  acquireLeaseDirect().then(() => toast(t('toast.take_acquired'), 'info')).catch((e) => { closeTakeModal(); if (!isCancelledErr(e)) toast(e.message, 'err', e.code); });
});
$('btn-retake').addEventListener('click', () => {
  if (leaseModalOrPending() || !obsFresh()) return;
  const other = leaseOther();
  if (other != null) { toast(leaseBusyError(other).message, 'err', 'LEASE_ACTIVE'); return; }
  acquireLeaseDirect().then(() => toast(t('toast.retake_ok'), 'ok')).catch((e) => { if (!isCancelledErr(e)) toast(e.message, 'err', e.code); });
});
$('btn-retake-x').addEventListener('click', () => { S.leaseReleased = null; });

let preemptTick = null, preemptTargetHolder = null, preemptChangedAck = false, preemptChangedUntil = 0;
function openPreemptModal() {
  const cur = leaseOther();
  if (cur == null) { toast(t('preempt.cleared'), 'info'); return; }
  if (!takeModalOpen() && !preemptModalOpen()) leaseModalReturnFocus = document.activeElement;
  preemptTargetHolder = cur; preemptChangedAck = false; preemptChangedUntil = 0;
  $('preempt-changed').hidden = true; setText($('preempt-changed'), '');
  $('preempt-modal').hidden = false; closeTakeModal(); renderPreemptModal(); $('preempt-cancel').focus({ preventScroll: true });
  if (!preemptTick) preemptTick = setInterval(renderPreemptModal, 200);
}
function closePreemptModal() {
  const was = preemptModalOpen();
  if ($('preempt-modal')) $('preempt-modal').hidden = true;
  preemptTargetHolder = null; preemptChangedAck = false; preemptChangedUntil = 0;
  if ($('preempt-changed')) { $('preempt-changed').hidden = true; setText($('preempt-changed'), ''); }
  clearInterval(preemptTick); preemptTick = null;
  if (was) { suppressHeldConfirmKeys(); restoreLeaseModalFocus(); }
}
function renderPreemptModal() {
  if (!preemptModalOpen()) return;
  const forcing = !!(leaseOp && leaseOp.kind === 'acquire' && leaseOp.force), fresh = obsFresh();
  if (!forcing && !leaseOp && fresh) {
    const cur = leaseOther();
    if (cur == null) { closePreemptModal(); toast(t('preempt.cleared'), 'info'); return; }
    if (cur !== preemptTargetHolder) { preemptTargetHolder = cur; preemptChangedAck = true; preemptChangedUntil = Date.now() + 800; }
  }
  const whoDetail = leaseHolderDetail(preemptTargetHolder);
  setText($('preempt-body'), t('preempt.body.who', { who: whoDetail }));
  const chg = $('preempt-changed'); chg.hidden = !preemptChangedAck;
  if (preemptChangedAck) setText(chg, t('preempt.changed', { who: whoDetail }));
  const now = Date.now(), left = S.preemptCooldownUntil - now, chgWait = preemptChangedUntil - now;
  const btn = $('preempt-confirm'), note = $('preempt-cooldown');
  btn.disabled = !fresh || left > 0 || chgWait > 0 || forcing || (!!leaseOp && !forcing);
  setText(btn, forcing ? t('preempt.taking') : t('preempt.confirm'));
  if (!forcing && !leaseOp && !fresh) { note.hidden = false; setText(note, t('preempt.stale')); }
  else { note.hidden = left <= 0; if (left > 0) setText(note, t('preempt.cool', { s: Math.ceil(left / 1000) })); }
}
$('btn-preempt').addEventListener('click', () => { if (S.leaseGeneration == null && !leaseModalOrPending() && leaseOther() != null) openPreemptModal(); });
$('preempt-cancel').addEventListener('click', dismissPreemptModal);
$('preempt-modal').addEventListener('click', (e) => { if (e.target === e.currentTarget) dismissPreemptModal(); });
bindStopButton($('preempt-stop')); bindEstopButton($('preempt-estop'));
$('preempt-confirm').addEventListener('click', () => {
  if (S.preemptCooldownUntil > Date.now() || preemptChangedUntil > Date.now() || leaseOp) return;
  if (!obsFresh()) { toast(t('toast.ws_not_ready'), 'err'); renderPreemptModal(); return; }
  const cur = leaseOther();
  if (cur == null) { closePreemptModal(); toast(t('preempt.cleared'), 'info'); return; }
  if (cur !== preemptTargetHolder) { preemptTargetHolder = cur; preemptChangedAck = true; preemptChangedUntil = Date.now() + 800; renderPreemptModal(); return; }
  startAcquireOp(S.ws, true);
});
window.addEventListener('keydown', (e) => {
  const opOpen = opIsOpen(), modalOpen = opOpen || takeModalOpen() || preemptModalOpen();
  if (e.key === ' ' || e.code === 'Space') {
    const typing = !modalOpen && e.target && (e.target.tagName === 'TEXTAREA' || (e.target.tagName === 'INPUT' && (e.target.type === 'text' || e.target.type === 'password' || e.target.type === 'number')));
    if (!typing) { e.preventDefault(); e.stopPropagation(); e.stopImmediatePropagation(); if (!e.repeat) eStop(); return; }
  }
  if (e.code) heldCodes.add(e.code);
  if (e.code && suppressedCodes.has(e.code)) { e.preventDefault(); e.stopPropagation(); e.stopImmediatePropagation(); return; }
  if (!modalOpen) return;
  if (e.key === 'Escape') {
    e.preventDefault(); e.stopPropagation(); e.stopImmediatePropagation(); suppressedCodes.add(e.code || 'Escape');
    if (opOpen) closeOperatorModal(false); else { if (takeModalOpen()) dismissTakeModal(); if (preemptModalOpen()) dismissPreemptModal(); }
    return;
  }
  if (e.repeat && (e.code === 'Enter' || e.code === 'NumpadEnter' || e.key === 'Enter')) { e.preventDefault(); e.stopPropagation(); e.stopImmediatePropagation(); return; }
  if (e.key === 'Tab') {
    const m = opOpen ? $('op-modal') : preemptModalOpen() ? $('preempt-modal') : $('take-modal');
    const f = [...m.querySelectorAll('button:not([hidden]):not([disabled]), input:not([hidden]):not([disabled])')].filter((x) => x.offsetParent !== null);
    if (!f.length) return;
    const i = f.indexOf(document.activeElement);
    if (i < 0) { e.preventDefault(); (e.shiftKey ? f[f.length - 1] : f[0]).focus({ preventScroll: true }); }
    else if (e.shiftKey && i === 0) { e.preventDefault(); f[f.length - 1].focus({ preventScroll: true }); }
    else if (!e.shiftKey && i === f.length - 1) { e.preventDefault(); f[0].focus({ preventScroll: true }); }
  }
}, true);
window.addEventListener('keyup', (e) => { if (e.code) { heldCodes.delete(e.code); suppressedCodes.delete(e.code); } }, true);
window.addEventListener('blur', () => { heldCodes.clear(); suppressedCodes.clear(); });
function sendHeartbeat(ws = S.ws) {
  if (!ws || ws.readyState !== 1 || S.ws !== ws || S.epoch == null || S.leaseGeneration == null || document.hidden) return false;
  try { ws.send(JSON.stringify({ type: 'heartbeat', client_id: CLIENT_ID, generation: S.leaseGeneration, epoch_id: S.epoch, timestamp_ms: Date.now() })); S.hb.n++; S.hb.at = Date.now(); return true; }
  catch (_) { return false; }
}
setInterval(() => { sendHeartbeat(); }, CFG.heartbeatMs);
setInterval(() => { S.hz = S.wsCount; S.wsCount = 0; S.hb.hz = S.hb.n; S.hb.n = 0; }, 1000);
setInterval(() => {
  // RTT probe when no other request has measured it recently
  if (Date.now() - S.lastReqAt > 1500 && !document.hidden) api('/api/v1/observation', undefined, { quiet: true, method: 'GET' }).catch(() => {});
}, 2000);

