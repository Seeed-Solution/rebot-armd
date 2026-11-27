/* =================================================================
   Gravity compensation (hand-guiding): hold 600 ms to enter, tap to exit
   ================================================================= */
const gcOn = () => S.obs?.control_mode === 'gravity_comp';
const gcExecutorUnsupported = () => S.obs?.gravity_comp?.blocker?.code === 'EXECUTOR_CAPABILITY_UNSUPPORTED';
function gcBlockText() {
  if (!isLive()) return t('gc.b.offline');
  if (leaseOther() != null) return t('gc.b.observer');
  const b = S.obs?.gravity_comp?.blocker;
  // not held but free: the switch prompts to confirm taking control first
  if (!b || (b.code === 'LEASE_REQUIRED' && S.leaseGeneration == null)) return null;
  const k = 'gc.b.' + b.code;
  return t(k) !== k ? t(k) : (b.message || b.code);
}
function gcWorkspaceBlocked() {
  if (!gcOn()) return false;
  const gc = S.obs?.gravity_comp;
  const phase = (gc && gc.phase) || 'active';
  return (phase === 'ramp_in' || phase === 'active') && !!(gc && gc.workspace_outside);
}
function gcExit() {
  if (gcWorkspaceBlocked()) {
    toast(formatGcWorkspaceBanner(S.obs.gravity_comp) || t('err.gc_exit_outside_workspace'), 'err', 'GC_EXIT_OUTSIDE_WORKSPACE');
    return;
  }
  api('/api/v1/mode/gravity_comp', { enable: false }, { quiet: true }).catch((e) => { if (!isCancelledErr(e)) toast(e.message, 'err', e.code); });
}
let gcHintAt = 0;
function gcSticksOff() { if (Date.now() - gcHintAt > 2500) { gcHintAt = Date.now(); toast(t('gc.sticks_off'), 'warn'); } }
// Gripper while hand-guiding: one open / close action per press (servo frames are refused).
let gcGripDir = 0;
function gcGrip(dir) {
  if (!dir || dir === gcGripDir) return;
  if (leaseModalOrPending()) return;
  if (S.leaseGeneration == null) { promptTakeControl(); return; }
  gcGripDir = dir;
  S.grip = dir > 0 ? 0 : 1; S.gripCmdAt = Date.now();
  commandRPC('/api/v1/action/gripper', dir > 0 ? { command: 'close' } : { command: 'open' })
    .then((r) => syncEpoch(r.epoch_id))
    .catch((e) => { if (!isCancelledErr(e) && e.code !== 'MOTION_BUSY') toast(e.message, 'err', e.code); });
}
function gcInput(v) {
  const g = S.mode === 'cart' ? -cartGripVel() : (v[GRIP] || 0);
  if (g === 0) gcGripDir = 0; else gcGrip(Math.sign(g));
  if (v.some((x, i) => i !== GRIP && x !== 0)) gcSticksOff();
}
(() => {
  const btn = $('btn-gc');
  let timer = null, inflight = false;
  const cancel = () => { clearTimeout(timer); timer = null; btn.classList.remove('holding'); };
  holdCancels.push(cancel);
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (inflight || leaseModalOrPending()) return;
    if (gcOn()) { gcExit(); return; } // leaving is a single tap
    const why = gcBlockText();
    if (why) { toast(t('gc.enter_fail') + why, 'err', S.obs?.gravity_comp?.blocker?.code || ''); return; }
    btn.classList.add('holding');
    timer = setTimeout(() => {
      cancel(); buzz(20);
      if (leaseModalOrPending()) return;
      if (S.leaseGeneration == null) {
        requireLeaseOrPrompt().catch((err) => { if (!isCancelledErr(err)) toast(err.message, 'err', err.code); });
        return;
      }
      inflight = true;
      releaseAll();
      toast(t('gc.entering'), 'info');
      api('/api/v1/mode/gravity_comp', { enable: true }, { quiet: true })
        .then(() => { toast(t('gc.entered'), 'ok'); if (teachOn()) teachStrip(true); })
        .catch((err) => {
          if (isCancelledErr(err)) return;
          const k = 'gc.b.' + err.code;
          toast(t('gc.enter_fail') + (t(k) !== k ? t(k) : err.message), 'err', err.code);
        })
        .finally(() => { setTimeout(() => { inflight = false; }, 300); });
    }, 600);
  });
  for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) btn.addEventListener(ev, cancel);
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
  $('btn-gc-exit').addEventListener('click', gcExit);
  // teach v1: the record button records into the teach store (and opens the panel collapsed); else the legacy recording
  $('btn-gc-rec').addEventListener('click', () => {
    if (!teachOn()) { $('btn-rec').click(); return; }
    teachStrip(true);
    teachRecToggle();
  });
})();
// Exit reason toast: once per new `last_exit` (not for an exit that predates this page).
function gcOnObs(o) {
  const x = o.gravity_comp?.last_exit;
  if (S.gcExitAt === undefined) { S.gcExitAt = x ? x.at : null; return; }
  if (!x || x.at === S.gcExitAt) return;
  S.gcExitAt = x.at;
  const k = 'gc.x.' + x.code;
  const calm = x.code === 'REQUESTED' || x.code === 'STOPPED';
  toast(t(k) !== k ? t(k) : t('gc.x.other', { m: x.message || x.code }), calm ? 'ok' : 'warn', calm ? '' : x.code);
}
function renderGc() {
  const gc = S.obs?.gravity_comp, on = gcOn(), live = isLive();
  document.body.classList.toggle('gc', on && live);
  const sw = $('btn-gc');
  const supported = !!(gc && gc.supported) && (S.caps ? S.caps.gravity_comp !== false : true);
  const execUnsup = gcExecutorUnsupported();
  sw.hidden = !supported && !execUnsup;
  const why = on ? null : gcBlockText();
  const phase = (gc && gc.phase) || 'active';
  const wsOut = gcWorkspaceBlocked();
  const exitWhy = wsOut ? formatGcExitBlocked(gc) : '';
  sw.classList.toggle('on', on);
  sw.classList.toggle('blocked', !on && !!why);
  sw.setAttribute('aria-checked', String(on));
  sw.setAttribute('aria-disabled', String((!on && !!why) || wsOut));
  sw.title = !on ? (why || '') : exitWhy;
  setText($('gc-sub'), on ? (exitWhy || t('gc.exit')) : why || t('gc.hold'));
  const banner = $('gc-banner');
  banner.hidden = !on || !live;
  banner.classList.toggle('ws-outside', wsOut);
  if (!on) return;
  const pct = Math.round((phase === 'ramp_in' ? gc.ramp : phase === 'active' ? 1 : 1) * 100);
  banner.classList.toggle('active', phase === 'active');
  setText($('gc-phase'), t('gc.ph.' + phase, { n: Math.round((gc.ramp || 0) * 100) }));
  $('gc-ramp-bar').style.width = pct + '%';
  $('gc-ramp').setAttribute('aria-valuenow', String(pct));
  setText($('gc-hint'), phase === 'ramp_in' ? t('gc.hint.ramp') : t('gc.hint'));
  const wsWarn = $('gc-ws-warn');
  if (wsWarn) {
    wsWarn.hidden = !wsOut;
    setText(wsWarn, wsOut ? formatGcWorkspaceBanner(gc) : '');
  }
  const exitBtn = $('btn-gc-exit');
  if (exitBtn) {
    exitBtn.classList.toggle('blocked', wsOut);
    exitBtn.setAttribute('aria-disabled', String(wsOut));
    exitBtn.title = exitWhy;
  }
  const exitWhyEl = $('gc-exit-why');
  if (exitWhyEl) {
    exitWhyEl.hidden = !wsOut;
    setText(exitWhyEl, exitWhy);
  }
  const rb = $('btn-gc-rec');
  rb.classList.toggle('rec-on', !!S.rec.active);
  setText(rb, S.rec.active ? t('gc.rec.on', { t: fmtTime(Date.now() - S.rec.startedAt) }) : t('gc.rec.start'));
  setText($('gc-chip'), phase === 'ramp_in' ? t('gc.chip') + ' ' + pct + '%' : t('gc.chip'));
}

/* =================================================================
   Simple mode: pose toggle (click = lock, Shift = momentary), gripper toggle
   ================================================================= */
const POSE_LABELS = {
  move: { t: ['x', 1, 'd.fwd'], b: ['x', -1, 'd.back'], l: ['y', 1, 'd.left'], r: ['y', -1, 'd.right'] },
  pose: { t: ['p', -1, 'pl.up'], b: ['p', 1, 'pl.dn'], l: ['r', -1, 'pl.rl'], r: ['r', 1, 'pl.rr'] },
};
function renderPoseUi() {
  const btn = $('btn-pose');
  btn.classList.toggle('on', S.poseMode);
  btn.setAttribute('aria-pressed', String(S.poseMode));
  btn.title = S.poseLocked ? t('pose.locked') : S.shiftHeld ? t('pose.shift') : t('pose.hint');
  const set = POSE_LABELS[S.poseMode ? 'pose' : 'move'];
  for (const k of ['t', 'b', 'l', 'r']) {
    const el = $('s-dir-' + k), [ax, sg, txt] = set[k];
    el.dataset.ax = ax; el.dataset.s = String(sg);
    el.querySelector('.lbl').textContent = t(txt);
  }
  $('pad-sxy').setAttribute('aria-label', S.poseMode ? t('pad.pose') : t('pad.move'));
}
function syncPoseMode() {
  const next = !viewActive() && (S.poseLocked || S.shiftHeld); // view frame: rotation has its own stick
  if (next === S.poseMode) return;
  S.poseMode = next;
  C.axes.sxy = [0, 0]; C.axes.sp = 0; C.axes.sr = 0;
  cartHoldStill(); // drop motion queued under the old mapping
  leftPadRequiresNeutral = true;
  renderPoseUi();
}
function togglePoseLock() {
  if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
  if (leaseModalOrPending()) return;
  if (S.leaseGeneration == null) { promptTakeControl(); return; }
  S.poseLocked = !S.poseLocked;
  buzz(8);
  syncPoseMode();
  renderPoseUi();
}
function resetPoseMode() { S.poseLocked = false; S.shiftHeld = false; syncPoseMode(); renderPoseUi(); }
$('btn-pose').addEventListener('click', togglePoseLock);
window.addEventListener('keydown', (e) => {
  if (e.code !== 'ShiftLeft' && e.code !== 'ShiftRight') return;
  if (e.target && /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName)) return;
  if (S.shiftHeld) return;
  S.shiftHeld = true; syncPoseMode(); renderPoseUi();
});
window.addEventListener('keyup', (e) => {
  if (e.code !== 'ShiftLeft' && e.code !== 'ShiftRight') return;
  if (!S.shiftHeld) return;
  S.shiftHeld = false; syncPoseMode(); renderPoseUi();
});
renderPoseUi();
applyFrame();

const GRIP_ICON = {
  open: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M8 13V5.5a1.5 1.5 0 0 1 3 0V12"/><path d="M11 11.5v-7a1.5 1.5 0 0 1 3 0V12"/><path d="M14 11.5V6a1.5 1.5 0 0 1 3 0v8"/><path d="M8 12.5V10a1.5 1.5 0 0 0-3 0v4a7 7 0 0 0 7 7h1a5 5 0 0 0 4-2.5"/></svg>',
  close: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><rect x="5" y="9" width="12" height="7" rx="2.5"/><path d="M8 9V7.5M11 9V7M14 9V7.5"/><path d="M17 12h1.5a1.5 1.5 0 0 1 0 3H17"/><path d="M7 16v2.5A2.5 2.5 0 0 0 9.5 21h4a2.5 2.5 0 0 0 2.5-2.5V16"/></svg>',
};
const gripOpenNow = () => (S.obs && S.obs.positions.length > GRIP ? gripFrac(S.obs.positions[GRIP]) : (S.grip ?? 0));
if ($('s-grip')) {
  $('s-grip').addEventListener('click', () => {
    if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
    if (leaseModalOrPending()) return;
    if (S.leaseGeneration == null) { promptTakeControl(); return; }
    // target follows the last command while it is still travelling, otherwise the measured state
    const ref = S.grip != null ? S.grip : gripOpenNow();
    S.grip = ref > 0.5 ? 0 : 1;
    buzz(10);
    const k = $('s-grip'); k.classList.remove('pulse'); void k.offsetWidth; k.classList.add('pulse');
    commandGrip();
  });
}

const PRESET_NAME = { home: 'pn.home', inspect: 'pn.inspect', park: 'pn.park' };
function presetName(n) { return PRESET_NAME[n] ? t(PRESET_NAME[n]) : ''; }
function presetTarget(name) {
  const pos = (S.obs && Array.isArray(S.obs.positions)) ? S.obs.positions : [];
  if (isSoArmProfile() && name === 'inspect') {
    return clampTarget([0.0, -0.45, 0.45, 0.95, 0.0, pos[GRIP] ?? CFG.gripClosed]);
  }
  let t = pos.slice(0, S.dof);
  while (t.length < S.dof) t.push(0);
  if (name === 'park' && !CFG.presets.park) {
    const r = restJoints();
    for (let i = 0; i < t.length && i < GRIP && i < r.length; i++) t[i] = r[i];
  } else {
    const m = toMotor(CFG.presets[name] || []);
    for (let i = 0; i < m.length && i < t.length; i++) t[i] = m[i];
  }
  if (name === 'park' && t.length > GRIP) { t[GRIP] = CFG.gripClosed; S.grip = 0; S.gripCmdAt = Date.now(); }
  return clampTarget(t);
}
async function runPreset(name) {
  if (!canMove() || !isLive()) { toast(lockReason(), 'err'); flashLock(); return; }
  if (leaseModalOrPending()) return;
  const snap = requireLeaseSyncOrPrompt((err) => toast(t('toast.preset_fail') + (err.message || err), 'err'));
  if (!snap) return;
  releaseAll();
  if (!isControlSnapValid(snap)) return;
  S.presetRunning = true;
  S.preset = { name, until: Date.now() + 15000, startedAt: Date.now() };
  const timer = setTimeout(() => {
    if (S.preset && S.preset.name === name) S.preset = null;
    S.presetRunning = false;
  }, 15000);

  const resetPresetRunning = () => {
    clearTimeout(timer);
    S.presetRunning = false;
    S.preset = null;
  };

  try {
    if (S.ws && S.ws.readyState === 1 && S.leaseGeneration != null) {
      S.lastIntentAt = Date.now();
      S.ws.send(JSON.stringify({
        type: 'preset',
        name: name,
        positions: (name === 'home' || name === 'park') ? undefined : presetTarget(name),
        epoch_id: S.epoch,
        client_id: CLIENT_ID,
        generation: S.leaseGeneration,
      }));
      motionIntentSeq++;
    } else {
      if (name === 'home') {
        await api('/api/v1/motion/home', { epoch_id: S.epoch });
        toast(t('toast.goto', { name: presetName(name) }), 'ok');
      } else if (name === 'park') {
        await controlledUnload();
      } else {
        await api('/api/v1/motion/jog', { target_positions: presetTarget(name), epoch_id: S.epoch });
        toast(t('toast.goto', { name: presetName(name) }), 'ok');
      }
    }
  } catch (err) {
    resetPresetRunning();
    if (!isCancelledErr(err)) toast(t('toast.preset_fail') + (err.message || err), 'err');
  }
}
for (const b of document.querySelectorAll('.preset')) b.addEventListener('click', () => runPreset(b.dataset.preset));

/* UI mode: simple (default) | expert */
function setUi(ui) {
  ui = ui === 'expert' ? 'expert' : 'simple';
  if (ui !== S.ui) releaseAll();
  const wasView = viewActive();
  S.ui = ui;
  if (viewActive() !== wasView) onFrameChange();
  document.body.classList.toggle('ui-simple', ui === 'simple');
  document.body.classList.toggle('ui-expert', ui === 'expert');
  for (const b of $('ui-seg').children) {
    const on = b.dataset.ui === ui;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on)); b.tabIndex = on ? 0 : -1;
  }
  if (typeof renderSVis === 'function') { placeSlot(); renderSVis(); renderGrasp(); }
  // simple mode always drives the TCP (cartesian); expert restores its own last choice
  // simple: safety lives inside the cockpit; expert: first card of the side column
  if (ui !== 'simple') visPopOpen(false);
  if (ui === 'simple') $('ck-safety').appendChild($('safety'));
  else $('side').insertBefore($('safety'), $('side').firstChild);
  if (ui === 'simple') $('cockpit').insertBefore($('gc-banner'), $('cockpit').firstChild);
  else $('side').insertBefore($('gc-banner'), $('safety').nextSibling);
  if (ui === 'simple') $('cockpit').insertBefore($('ext-banner'), $('s-ready'));
  else $('side').insertBefore($('ext-banner'), $('gc-banner').nextSibling);
  if (ui === 'simple') $('ck-safety').appendChild($('pose-card'));
  else $('side').insertBefore($('pose-card'), $('ext-banner').nextSibling);
  if (ui === 'simple') setMode('cart', 'btn');
  else {
    const legacy = store('rebot.mode'); // pre-cartesian builds stored jog | stick
    setMode(store('rebot.ctl') || 'joint', store('rebot.input') || (legacy === 'stick' ? 'stick' : 'btn'));
  }
  if (typeof exPlaceDom === 'function') exPlaceDom();
}
$('ui-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { S.uiUserSet = true; setUi(b.dataset.ui); } });
$('ui-seg').addEventListener('keydown', (e) => {
  if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(e.key)) return;
  e.preventDefault();
  const next = S.ui === 'simple' ? 'expert' : 'simple';
  S.uiUserSet = true;
  setUi(next);
  const btn = $('ui-seg').querySelector(`button[data-ui="${next}"]`);
  if (btn) btn.focus();
});

