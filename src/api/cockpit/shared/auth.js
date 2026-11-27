/* =================================================================
   Operator Auth Badge & Modal
   ================================================================= */
function reconnectWS() {
  if (S.ws) retireSocket(S.ws, { reconnect: false, markReleased: true });
  connectWS();
}
/* Operator access. The badge in the header opens the dialog; a privileged request refused with OPERATOR_REQUIRED
   opens it with the reason (opRequire). After an unlock the token goes to the open telemetry socket ({type:"auth"}),
   so the lease (and a torqued arm) stays; an armd without that message gets a reconnect, as before. */
const OP = { waiters: [], why: null, retry: false, skip: false, renew: false, ack: null, seq: 0, lastFocus: null, pass: null, warned: false, queue: new Map(), warnedFor: null };
const opLocked = () => S.auth.enforced && S.auth.role !== 'operator';
const opIsOpen = () => !!$('op-modal') && !$('op-modal').hidden;
// Resetting needs the operator: a latched E-stop, a non-recoverable fault, an escalated collision, or a carried-over motor error (armd lease.rs)
function clearNeedsOperator() {
  if (S.state === 'EStop') return true;
  const f = S.obs && S.obs.fault;
  if (S.state === 'Fault' && (!f || f.recoverable === false || S.faultCode === 'COLLISION')) return true;
  return hasCarriedOverMotorErrors();
}
function opReasonKey(path, body) {
  const p = String(path || '');
  if (p === 'lease.force') return 'op.why.take';
  if (p === 'calib.open') return 'op.why.calib_open';
  if (p === 'op.renew') return 'op.why.renew';
  if (p === '/api/v1/vision/models' || p === '/api/v1/vision/detector') return 'op.why.models';
  if (p === '/api/v1/vision/mode' || p === '/api/v1/vision/subscribe') return 'op.why.vision';
  if (p === '/api/v1/mode/gravity_comp') return 'op.why.gc';
  if (p === '/api/v1/safety/clear_fault') return S.state === 'EStop' ? 'op.why.estop' : hasCarriedOverMotorErrors() ? 'op.why.motor_err' : 'op.why.fault';
  if (/^\/api\/v1\/(detection|vision)\//.test(p) || p === '/api/v1/grasp/measure') return 'op.why.det';
  if (/^\/api\/v1\/(calibration|gripper\/calibration|grasp\/setup)/.test(p) || p === '/api/v1/action/gripper_zero_check' || p === '/api/v1/action/calib_hover') return 'op.why.calib';
  if (p === '/api/v1/action/gripper_calibrate') return 'op.why.calib';
  if (/^\/api\/v1\/(poses|sequences)\//.test(p)) return 'op.why.poses';
  if (p === '/api/v1/action/run_sequence' && body && body.record) return 'op.why.rec';
  if (/^\/api\/v1\/(record|dataset|episodes|datasets)\//.test(p)) return 'op.why.rec';
  if (p === '/api/v1/workspace/keepout' || p === '/api/v1/safety/collision') return 'op.why.safety';
  if (/^\/api\/v1\/sandbox\//.test(p)) return 'op.why.sandbox';
  return 'op.why.default';
}
// Run again after the unlock: only these user-initiated, repeatable requests; never a motion.
function opCanRetry(path, body) {
  return (path === '/api/v1/mode/gravity_comp' && !!body && body.enable === true)
    || path === '/api/v1/detection/trigger' || path === '/api/v1/vision/depth' || path === '/api/v1/vision/models' || path === '/api/v1/vision/detector' || path === '/api/v1/safety/clear_fault' || path === '/api/v1/calibration/apply';
}
/* Ask for the operator PIN because `path` was refused (or is about to be: a [data-op] control). Resolves true
   after an unlock, false on cancel. opt.retry: the dialog says the action continues after the unlock (default:
   opCanRetry); opt.skip: the action goes on either way, Cancel reads "Not now". */
function opRequire(path, body, opt = {}) {
  // refused while the page showed the session unlocked: it expired or was locked elsewhere
  if (S.auth.role === 'operator' && !OP.renew) { S.auth.role = 'program'; S.auth.expiresIn = null; scheduleAuthExpiryRefresh(); opToastExpired(); }
  S.auth.enforced = true;
  renderOpBadge();
  return new Promise((resolve) => {
    OP.waiters.push(resolve);
    const renew = OP.renew;
    openOperatorModal(opReasonKey(path, body), 'retry' in opt ? !!opt.retry : opCanRetry(path, body), !!opt.skip);
    OP.renew = renew; renderOperatorModal();
  });
}
/* "Session expired" and Unlock toasts describe a locked session: they go when it is unlocked again. */
function opToastExpired() { const el = toast(t('op.expired'), 'warn'); if (el) el.classList.add('op-stale'); }
function opDropStaleToasts() { document.querySelectorAll('#toasts .toast.op-stale, #toasts .toast.has-act').forEach((x) => x.remove()); }
/* armd refused a request with OPERATOR_REQUIRED: the page's session state is wrong (expired, locked elsewhere,
   or never known). Show the locked badge; no dialog (the caller decides). */
function opRefused() {
  if (S.auth.enforced && S.auth.role !== 'operator') return;
  const was = S.auth.role === 'operator';
  S.auth.enforced = true; S.auth.role = 'program'; S.auth.expiresIn = null; S.auth.expiresAt = null;
  scheduleAuthExpiryRefresh(); renderOpBadge();
  if (was) opToastExpired();
}
/* Long operator flows (camera / fine calibration, motor zero, gripper calibration, a recording, a size measurement)
   outlast the fixed operator session (armd auth.rs DEFAULT_TTL_S, 30 min, no renewal by design: a stolen or forgotten
   session ends). Before it ends the page asks for the PIN again (a fresh session; the flow keeps its progress), and an
   operator-only write that is not a motion and was refused meanwhile waits in OP.queue until the next unlock. */
const opFlowActive = () => !!((typeof CWZ !== 'undefined' && (CWZ.open || CWZ.running || CWZ.solving)) || (typeof MZ !== 'undefined' && MZ.open)
  || !$('gtool').hidden || (typeof TCH !== 'undefined' && TCH.rec) || (typeof MS !== 'undefined' && MS.state === 'running'));
const OP_WARN_MS = 3 * 60000;
function opRenew() {
  OP.renew = true;
  return opRequire('op.renew', undefined, { retry: false });
}
function opCheckExpiry() {
  if (!S.auth.enforced || S.auth.role !== 'operator' || !S.auth.expiresAt || !opFlowActive()) return;
  const left = S.auth.expiresAt - Date.now();
  if (left > OP_WARN_MS || OP.warnedFor === S.auth.expiresAt || opIsOpen()) return;
  OP.warnedFor = S.auth.expiresAt;
  const el = toast(t('op.renew_warn', { m: Math.max(1, Math.ceil(left / 60000)) }), 'warn', '', { act: { label: t('op.renew_btn'), fn: opRenew }, ms: 60000 });
  if (el) el.classList.add('op-stale');
}
// an operator-only write refused during a flow: run it again after the next unlock (key: one entry per kind of write)
function opQueue(key, fn) { OP.queue.set(key, fn); }
function opFlushQueue() {
  const q = [...OP.queue.values()]; OP.queue.clear();
  for (const fn of q) { try { Promise.resolve(fn()).catch(() => {}); } catch (_) {} }
}
// every same-origin API answer: a 403 OPERATOR_REQUIRED updates the badge, whichever code path made the request
{
  const nativeFetch = window.fetch.bind(window);
  window.fetch = async (input, init) => {
    const res = await nativeFetch(input, init);
    if (res.status === 403) {
      try {
        const u = new URL(typeof input === 'string' ? input : input.url, location.href);
        if (u.origin === location.origin && u.pathname.startsWith('/api/v1/') && u.pathname !== '/api/v1/auth/unlock') {
          res.clone().json().then((d) => { if (d && d.code === 'OPERATOR_REQUIRED') opRefused(); }).catch(() => {});
        }
      } catch (_) {}
    }
    return res;
  };
}
/* Operator-only controls carry data-op="<path>" (the reason in the dialog) and data-op-mode: "gate" (default; the
   tap runs again after the unlock, Cancel drops it), "offer" (a drawer that holds operator-only steps: the drawer
   opens after the dialog either way) or "move" (moves the arm or takes control: unlock only, the user taps again;
   data-op-again names the hint). While locked
   they show a lock (CSS body.op-locked) and a tap opens the dialog instead of being refused afterwards. */
document.addEventListener('click', (e) => {
  const el = e.target && e.target.closest ? e.target.closest('[data-op]') : null;
  if (!el || !opLocked() || el.disabled) return;
  if (OP.pass === el) return;
  e.preventDefault(); e.stopImmediatePropagation();
  const mode = el.dataset.opMode || 'gate';
  const snap = { controlEpoch: S.controlEpoch, ws: S.ws, epoch: S.epoch };
  opRequire(el.dataset.op, undefined, { retry: mode === 'gate', skip: mode === 'offer' }).then((ok) => {
    const tgt = document.contains(el) ? el : (el.id && $(el.id)) || null;
    if (!tgt || tgt.disabled) return;
    if (mode === 'offer') { OP.pass = tgt; try { tgt.click(); } finally { OP.pass = null; } }
    else if (ok && mode === 'gate') {
      if (S.controlEpoch !== snap.controlEpoch || S.ws !== snap.ws || S.epoch !== snap.epoch) return;
      OP.pass = tgt; try { tgt.click(); } finally { OP.pass = null; }
    }
    else if (ok) toast(t(el.dataset.opAgain || 'op.again'), 'info');
  });
}, true);
function opSettle(ok) {
  const w = OP.waiters; OP.waiters = [];
  for (const r of w) r(ok);
}
// Resolves once the telemetry socket is open (a reconnect after an unlock on an older armd), or after `ms`.
function wsOpen(ms) {
  const end = Date.now() + ms;
  return new Promise((resolve) => {
    const poll = () => { if ((S.ws && S.ws.readyState === 1 && S.epoch != null) || Date.now() > end) resolve(); else setTimeout(poll, 50); };
    poll();
  });
}
/* Hand `token` (null: none) to the open telemetry socket; fall back to a reconnect when armd does not answer. */
function opSyncSocket(token) {
  const ws = S.ws;
  if (!ws || ws.readyState !== 1) return Promise.resolve(); // the next socket carries the cookie
  return new Promise((resolve) => {
    const seq = ++OP.seq;
    const fallback = () => { OP.ack = null; reconnectWS(); resolve(); };
    const timer = setTimeout(fallback, 1500);
    OP.ack = { ws, seq, done: () => { clearTimeout(timer); OP.ack = null; resolve(); } };
    try { ws.send(JSON.stringify({ type: 'auth', seq, token })); } catch (_) { clearTimeout(timer); fallback(); }
  });
}
// minutes left in the operator session, null when unknown or locked
const opLeftMin = () => (S.auth.role === 'operator' && S.auth.expiresAt ? Math.max(0, Math.ceil((S.auth.expiresAt - Date.now()) / 60000)) : null);
function renderOpBadge() {
  const b = $('btn-op');
  document.body.classList.toggle('op-locked', opLocked());
  if (!b) return;
  b.hidden = !S.auth.enforced;
  if (!S.auth.enforced) return;
  const unlocked = S.auth.role === 'operator';
  const left = opLeftMin();
  b.classList.toggle('is-unlocked', unlocked);
  // "🔒 操作员": icon and label apart, so the simple landscape island can drop the label (tb-compact-3)
  // the last minutes of the session replace the label: an action may be refused soon
  const soon = unlocked && left != null && left <= 5;
  const txt = unlocked ? t('op.unlocked') : t('op.locked'), sp = txt.indexOf(' ');
  b.textContent = '';
  if (sp > 0) b.append(cwEl('span', 'op-ic', txt.slice(0, sp)), ' ', cwEl('span', 'op-lbl', soon ? t('op.left', { m: left }) : txt.slice(sp + 1)));
  else b.textContent = txt;
  const tip = unlocked ? (left != null ? t('op.badge.unlocked_m', { m: left }) : t('op.badge.unlocked')) : t('op.badge.locked');
  b.title = tip; b.setAttribute('aria-label', tip);
  if (opIsOpen()) renderOperatorModal();
}
let authRefreshTimer = null;
function scheduleAuthExpiryRefresh() {
  if (authRefreshTimer) {
    clearTimeout(authRefreshTimer);
    authRefreshTimer = null;
  }
  if (S.auth.enforced && S.auth.role === 'operator' && typeof S.auth.expiresIn === 'number' && S.auth.expiresIn > 0) {
    S.auth.expiresAt = Date.now() + S.auth.expiresIn * 1000;
    const delayMs = Math.min(S.auth.expiresIn * 1000 + 250, 2147483647);
    authRefreshTimer = setTimeout(() => {
      authRefreshTimer = null;
      loadAuthStatus();
    }, delayMs);
  }
}
async function loadAuthStatus() {
  try {
    const res = await fetch('/api/v1/auth/status', { cache: 'no-store' });
    if (!res.ok) return;
    const d = await res.json();
    const was = S.auth.role;
    S.auth.enforced = !!d.auth_enforced;
    S.auth.role = d.role || 'program';
    S.auth.expiresIn = d.expires_in_s ?? null;
    if (was === 'operator' && S.auth.role !== 'operator' && S.auth.enforced) opToastExpired();
    scheduleAuthExpiryRefresh();
    renderOpBadge();
  } catch (_) {}
}
// Fit the dialog's box to the visual viewport: an on-screen keyboard shrinks it, the dialog stays above it.
function opFit() {
  const m = $('op-modal'), vv = window.visualViewport;
  if (!m || m.hidden) return;
  const h = vv ? vv.height : innerHeight;
  if (vv) { m.style.top = Math.max(0, vv.offsetTop) + 'px'; m.style.height = h + 'px'; }
  m.classList.toggle('op-short', h < 520);
}
function renderOperatorModal() {
  // renew: a new unlock while the session still runs (the PIN starts a fresh session; nothing else changes)
  const unlocked = S.auth.role === 'operator' && !OP.renew;
  setText($('op-why'), OP.why ? t(OP.why) : '');
  setText($('op-retry'), OP.why && OP.retry && !unlocked ? t('op.retry') : '');
  $('op-pin-field').hidden = unlocked;
  $('op-unlock').hidden = unlocked;
  $('op-lock').hidden = !unlocked;
  $('op-hint').hidden = unlocked;
  setText($('op-cancel'), t(unlocked ? 'op.close' : OP.skip ? 'op.skip' : 'cancel'));
  const left = unlocked && S.auth.expiresAt ? Math.max(0, Math.round((S.auth.expiresAt - Date.now()) / 60000)) : null;
  setText($('op-state'), unlocked ? (left ? t('op.state_on', { m: left }) : t('op.state_on0')) : '');
}
function openOperatorModal(why = null, retry = false, skip = false) {
  const m = $('op-modal');
  if (!m) return;
  const wasOpen = !m.hidden;
  if (why || !wasOpen) { OP.why = why; OP.retry = retry; OP.skip = skip; }
  if (!wasOpen) {
    OP.lastFocus = document.activeElement;
    $('op-pin').value = '';
    setText($('op-status'), '');
  }
  m.hidden = false;
  document.body.classList.add('op-open');
  renderOperatorModal();
  opFit();
  if (S.auth.role !== 'operator') $('op-pin').focus({ preventScroll: true });
  else $('op-cancel').focus({ preventScroll: true });
}
function closeOperatorModal(ok = false) {
  const m = $('op-modal');
  if (!m || m.hidden) { opSettle(ok); return; }
  m.hidden = true;
  m.style.top = ''; m.style.height = '';
  document.body.classList.remove('op-open');
  $('op-pin').value = '';
  OP.why = null; OP.retry = false; OP.skip = false; OP.renew = false;
  suppressHeldConfirmKeys();
  const f = OP.lastFocus; OP.lastFocus = null;
  if (f && typeof f.focus === 'function' && document.contains(f)) { try { f.focus({ preventScroll: true }); } catch (_) {} }
  opSettle(ok);
}
async function submitOperatorPin(ev) {
  if (ev) ev.preventDefault();
  if (S.auth.role === 'operator' && !OP.renew) { closeOperatorModal(true); return; }
  const pin = ($('op-pin').value || '').trim();
  if (!pin) { setText($('op-status'), t('op.need_pin')); $('op-pin').focus({ preventScroll: true }); return; }
  $('op-unlock').disabled = true;
  try {
    const res = await fetch('/api/v1/auth/unlock', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pin }),
    });
    const d = await res.json().catch(() => ({}));
    if (!res.ok) {
      setText($('op-status'), d.code === 'INVALID_PIN' ? t('op.bad_pin') : d.code === 'RATE_LIMITED' ? t('op.rate', { s: d.retry_after_s ?? '?' }) : (d.message || t('op.bad_pin')));
      $('op-pin').select();
      return;
    }
    S.auth.role = d.role || 'operator';
    S.auth.expiresIn = d.expires_in_s ?? null;
    if (window.WASM_SIM) { window.WASM_SIM.telBlocked = false; window.WASM_SIM.telErrLogged = false; }
    scheduleAuthExpiryRefresh();
    renderOpBadge();
    await opSyncSocket(d.token || null);
    closeOperatorModal(true);
    opDropStaleToasts();
    toast(t('op.unlocked_msg'), 'ok');
    opFlushQueue();
  } catch (e) {
    setText($('op-status'), e.message || t('err.reqfail'));
  } finally {
    $('op-unlock').disabled = false;
  }
}
async function lockOperator() {
  try {
    const res = await fetch('/api/v1/auth/lock', { method: 'POST' });
    const d = await res.json().catch(() => ({}));
    if (!res.ok || d.role !== 'program') {
      toast(t('op.lock_fail'), 'err');
      return;
    }
    S.auth.role = 'program';
    S.auth.expiresIn = null;
    S.auth.expiresAt = null;
    scheduleAuthExpiryRefresh();
    renderOpBadge();
    await opSyncSocket(null);
    closeOperatorModal(false);
    toast(t('op.locked_msg'), 'warn');
  } catch (_) {
    toast(t('op.lock_fail'), 'err');
  }
}
$('btn-op')?.addEventListener('click', () => openOperatorModal());
$('op-cancel')?.addEventListener('click', () => closeOperatorModal(false));
$('op-modal')?.addEventListener('pointerdown', (e) => { if (e.target === e.currentTarget) closeOperatorModal(false); });
$('op-form')?.addEventListener('submit', submitOperatorPin);
$('op-lock')?.addEventListener('click', lockOperator);
bindStopButton($('op-stop'));
bindEstopButton($('op-estop'));
$('op-inline')?.addEventListener('click', () => {
  const snap = { controlEpoch: S.controlEpoch, faultKey: mitFaultKey() };
  opRequire('/api/v1/safety/clear_fault').then((ok) => {
    if (ok && (isStopped() || (!S.obs?.is_torqued && hasCarriedOverMotorErrors())) && S.controlEpoch === snap.controlEpoch && mitFaultKey() === snap.faultKey) {
      clearFault({ directLease: true });
    }
  });
});
if (window.visualViewport) for (const ev of ['resize', 'scroll']) window.visualViewport.addEventListener(ev, opFit);
i18nHooks.push(() => { if (opIsOpen()) renderOperatorModal(); });

