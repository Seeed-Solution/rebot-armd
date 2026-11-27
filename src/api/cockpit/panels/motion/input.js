/* =================================================================
   Part icons (side-view mini arm, highlighted part + motion arrow)
   ================================================================= */
function partIcon(j) {
  const hi = '#5B7D15', lo = '#cbd5e1';
  const c = (k) => (k === j ? hi : lo);
  const w = (k) => (k === j ? 3 : 2.2);
  const motion = [
    '<ellipse cx="13" cy="31" rx="9" ry="3" fill="none" stroke="#5B7D15" stroke-width="1.4" stroke-dasharray="20 4" marker-end="url(#ah)"/>',
    '<path d="M4.3 22 A10 10 0 0 1 11.3 17.2" fill="none" stroke="#5B7D15" stroke-width="1.5" marker-start="url(#ah)" marker-end="url(#ah)"/>',
    '<path d="M14.1 11.6 A8 8 0 0 1 26 6.1" fill="none" stroke="#5B7D15" stroke-width="1.5" marker-start="url(#ah)" marker-end="url(#ah)"/>',
    '<path d="M35.5 11.9 A7 7 0 0 1 38.1 21.5" fill="none" stroke="#5B7D15" stroke-width="1.5" marker-start="url(#ah)" marker-end="url(#ah)"/>',
    '<ellipse cx="33.6" cy="21" rx="5.8" ry="2.3" transform="rotate(62 33.6 21)" fill="none" stroke="#5B7D15" stroke-width="1.4" stroke-dasharray="14 4" marker-end="url(#ah)"/>',
    '<path d="M30 31.5h3M40.5 30.5h-3" stroke="#5B7D15" stroke-width="1.5" marker-end="url(#ah)"/>',
  ][j];
  return `<svg viewBox="0 0 40 40" fill="none" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <rect x="6" y="33" width="14" height="4" rx="1.5" fill="${j === 0 ? 'rgba(140,192,32,0.25)' : '#f8fafc'}" stroke="${c(0)}" stroke-width="1.4"/>
    <path d="M13 33V27" stroke="${c(0)}" stroke-width="${w(0)}"/>
    <path d="M13 27L22 13" stroke="${c(1)}" stroke-width="${w(1)}"/>
    <path d="M22 13L32 18" stroke="${c(2)}" stroke-width="${w(2)}"/>
    <path d="M32 18L35 24" stroke="${j === 3 || j === 4 ? hi : lo}" stroke-width="${j === 3 || j === 4 ? 3 : 2.2}"/>
    <path d="M35 24L33 29M35 24L38 28" stroke="${c(5)}" stroke-width="${w(5)}"/>
    <circle cx="13" cy="27" r="1.8" fill="#ffffff" stroke="${j === 1 ? hi : lo}" stroke-width="1.3"/>
    <circle cx="22" cy="13" r="1.8" fill="#ffffff" stroke="${j === 2 ? hi : lo}" stroke-width="1.3"/>
    <circle cx="32" cy="18" r="1.6" fill="#ffffff" stroke="${j === 3 ? hi : lo}" stroke-width="1.3"/>
    ${motion}
  </svg>`;
}
const CHEV_L = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"><path d="M15 5l-7 7 7 7"/></svg>';
const CHEV_R = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"><path d="M9 5l7 7-7 7"/></svg>';

/* =================================================================
   Hold-to-move buttons (pointer events, multi-touch safe)
   ================================================================= */
const holdButtons = [];
const cartButtons = [];
function bindHold(btn, j, s) {
  let pid = null;
  const idx = s > 0 ? 1 : 0;
  const stop = () => {
    if (pid === null) return;
    pid = null; btn.classList.remove('pressed');
    S.hold[j][idx] = Math.max(0, S.hold[j][idx] - 1);
  };
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (pid !== null) return;
    if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
    if (leaseModalOrPending()) return;
    if (S.leaseGeneration == null) { promptTakeControl(); return; }
    pid = e.pointerId;
    try { btn.setPointerCapture(pid); } catch (_) {}
    btn.classList.add('pressed');
    S.hold[j][idx]++;
    buzz(8);
  });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    btn.addEventListener(ev, (e) => { if (e.pointerId === pid) stop(); });
  }
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
  holdButtons.push({ btn, j, s, stop });
}
function jogButton(j, s, label, key) {
  const b = document.createElement('button');
  b.className = 'jbtn ' + (s > 0 ? 'pos' : 'neg');
  const list = activeJoints();
  const name = list[j]?.key ?? ('J' + (j + 1));
  b.setAttribute('aria-label', `${name} ${label}`);
  b.innerHTML = s > 0
    ? `<kbd>${key}</kbd>${CHEV_R}<span>${label}</span>`
    : `<kbd>${key}</kbd>${CHEV_L}<span>${label}</span>`;
  bindHold(b, j, s);
  return b;
}

const JOINTS_7DOF = [
  { key: 'J1', part: 'jd.base.part', motion: 'jd.base.motion', en: 'Base Yaw', neg: 'jd.base.neg', pos: 'jd.base.pos', kn: 'A', kp: 'D' },
  { key: 'J2', part: 'jd.shoulder.part', motion: 'jd.shoulder.motion', en: 'Shoulder Pitch', neg: 'jd.shoulder.neg', pos: 'jd.shoulder.pos', kn: 'S', kp: 'W' },
  { key: 'J3', part: 'jd.elbow.part', motion: 'jd.elbow.motion', en: 'Elbow Pitch', neg: 'jd.elbow.neg', pos: 'jd.elbow.pos', kn: '↓', kp: '↑' },
  { key: 'J4', part: 'jd.wpitch.part', motion: 'jd.wpitch.motion', en: 'Wrist Pitch', neg: 'jd.wpitch.neg', pos: 'jd.wpitch.pos', kn: 'Q', kp: 'E' },
  { key: 'J5', part: 'jd.wyaw.part', motion: 'jd.wyaw.motion', en: 'Wrist Swing', neg: 'jd.wyaw.neg', pos: 'jd.wyaw.pos', kn: '←', kp: '→' },
  { key: 'J6', part: 'jd.wroll.part', motion: 'jd.wroll.motion', en: 'Wrist Roll', neg: 'jd.wroll.neg', pos: 'jd.wroll.pos', kn: 'R', kp: 'F' },
  { key: 'J7', part: 'jd.grip.part', motion: 'jd.grip.motion', en: 'Gripper', neg: 'jd.grip.neg', pos: 'jd.grip.pos', kn: 'Z', kp: 'X' },
];

const JOINTS_SO_ARM = [
  { key: 'J1', part: 'jd.base.part', motion: 'jd.base.motion', en: 'Shoulder Pan', neg: 'jd.base.neg', pos: 'jd.base.pos', kn: 'A', kp: 'D' },
  { key: 'J2', part: 'jd.shoulder.part', motion: 'jd.shoulder.motion', en: 'Shoulder Lift', neg: 'jd.shoulder.neg', pos: 'jd.shoulder.pos', kn: 'S', kp: 'W' },
  { key: 'J3', part: 'jd.elbow.part', motion: 'jd.elbow.motion', en: 'Elbow Flex', neg: 'jd.elbow.neg', pos: 'jd.elbow.pos', kn: '↓', kp: '↑' },
  { key: 'J4', part: 'jd.wpitch.part', motion: 'jd.wpitch.motion', en: 'Wrist Flex', neg: 'jd.wpitch.neg', pos: 'jd.wpitch.pos', kn: 'Q', kp: 'E' },
  { key: 'J5', part: 'jd.wroll.part', motion: 'jd.wroll.motion', en: 'Wrist Roll', neg: 'jd.wroll.neg', pos: 'jd.wroll.pos', kn: '←', kp: '→' },
  { key: 'J6', part: 'jd.grip.part', motion: 'jd.grip.motion', en: 'Gripper', neg: 'jd.grip.neg', pos: 'jd.grip.pos', kn: 'Z', kp: 'X' },
];

const activeJoints = () => (S.dof === 7) ? JOINTS_7DOF : isSoArmProfile() ? JOINTS_SO_ARM : CFG.joints;

/* ---------- jog rows (dynamically rendered) ---------- */
let rows = [];
function renderJogRows() {
  const container = $('jog-list');
  if (!container) return;
  container.innerHTML = '';
  holdButtons.length = 0;
  const list = activeJoints();
  rows = list.map((jd, i) => {
    const row = document.createElement('div');
    row.className = 'jrow' + (i === GRIP ? ' grip' : '');
    const mid = document.createElement('div');
    mid.className = 'jmid';
    mid.innerHTML = `<div class="jtop">
        <span class="jicon">${partIcon(Math.min(i, 5))}</span>
        <span class="jtitle"><span class="l1"><b>${jd.key}</b>${t(jd.part)}</span><span class="l2">${LANG === 'zh' ? t(jd.motion) + ' · ' + jd.en : jd.en}</span></span>
        <span class="jval"><span class="n">--</span><small>${i === GRIP ? '%' : '°'}</small></span>
      </div>
      <div class="jgauge"><i class="fill"></i><i class="zero"></i><i class="mark"></i></div>`;
    const neg = jogButton(i, -1, t(jd.neg), jd.kn);
    const pos = jogButton(i, 1, t(jd.pos), jd.kp);
    row.append(neg, mid, pos);
    container.appendChild(row);
    return {
      row, neg, pos,
      val: mid.querySelector('.jval .n'), fill: mid.querySelector('.fill'),
      zero: mid.querySelector('.zero'), mark: mid.querySelector('.mark'),
    };
  });
  syncControlInputsUi(pageHasControl());
}
renderJogRows();

/* ---------- gripper buttons in joystick mode ---------- */
function renderGripCol() {
  const col = $('grip-col');
  if (!col) return;
  col.innerHTML = `<div class="stick-cap"><span class="ttl">${t('grip')} ${S.dof === 7 ? 'J7' : 'J6'}</span></div>`;
  const open = jogButton(GRIP, -1, t('jd.grip.neg'), 'Z');
  const close = jogButton(GRIP, 1, t('jd.grip.pos2'), 'X');
  const val = document.createElement('div');
  val.className = 'gval'; val.id = 'grip-val'; val.textContent = '--%';
  col.append(open, val, close);
  syncControlInputsUi(pageHasControl());
}
renderGripCol();

/* =================================================================
   Virtual joysticks (Touch.identifier per pad) + mouse fallback
   ================================================================= */
const pads = [];
class Pad {
  constructor(el, { axes = 'xy', onChange, onEnd }) {
    this.el = el; this.knob = el.querySelector('.knob');
    this.axes = axes; this.onChange = onChange; this.onEnd = onEnd;
    this.touchId = null; this.mouse = false; this.value = [0, 0];
    el.addEventListener('touchstart', (e) => {
      e.preventDefault();
      if (this.touchId !== null) return;
      const t = e.changedTouches[0];
      this.touchId = t.identifier;
      if (!this.begin(t.clientX, t.clientY)) this.touchId = null;
    }, { passive: false });
    el.addEventListener('mousedown', (e) => {
      e.preventDefault();
      this.mouse = true;
      if (!this.begin(e.clientX, e.clientY)) this.mouse = false;
    });
    pads.push(this);
  }
  begin(x, y) {
    if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return false; }
    if (leaseModalOrPending()) return false;
    if (S.leaseGeneration == null) { promptTakeControl(); return false; }
    this.el.classList.add('dragging'); this.move(x, y);
    return true;
  }
  geom() {
    const r = this.el.getBoundingClientRect(), k = this.knob.offsetWidth;
    return { cx: r.left + r.width / 2, cy: r.top + r.height / 2, rx: (r.width - k) / 2, ry: (r.height - k) / 2 };
  }
  move(x, y) {
    const g = this.geom();
    let dx = x - g.cx, dy = y - g.cy;
    if (this.axes === 'xy') {
      const d = Math.hypot(dx, dy) || 1, m = Math.min(d, g.rx);
      dx = dx / d * m; dy = dy / d * m;
      this.value = [dx / g.rx, -dy / g.rx];
    } else {
      dx = 0; dy = clamp(dy, -g.ry, g.ry);
      this.value = [0, -dy / g.ry];
    }
    this.knob.style.transform = `translate(${dx}px, ${dy}px)`;
    this.onChange(this.value);
  }
  end() {
    this.touchId = null; this.mouse = false; this.el.classList.remove('dragging');
    this.value = [0, 0];
    this.knob.style.transform = 'translate(0px, 0px)'; // CSS spring curve animates the return
    this.onChange(this.value);
    if (this.onEnd) this.onEnd();
  }
  active() { return this.touchId !== null || this.mouse; }
}
function findTouch(list, id) { for (const t of list) if (t.identifier === id) return t; return null; }
window.addEventListener('touchmove', (e) => {
  let used = false;
  for (const p of pads) {
    if (p.touchId === null) continue;
    const t = findTouch(e.changedTouches, p.touchId);
    if (t) { p.move(t.clientX, t.clientY); used = true; }
  }
  if (used || (e.target.closest && e.target.closest('.stick-zone, .cart-sticks, .move'))) e.preventDefault();
}, { passive: false });
const touchEnd = (e) => { for (const p of pads) if (p.touchId !== null && findTouch(e.changedTouches, p.touchId)) p.end(); };
window.addEventListener('touchend', touchEnd);
window.addEventListener('touchcancel', touchEnd);
window.addEventListener('mousemove', (e) => { for (const p of pads) if (p.mouse) p.move(e.clientX, e.clientY); });
window.addEventListener('mouseup', () => { for (const p of pads) if (p.mouse) p.end(); });

const shape = (v) => { const a = Math.abs(v); return a < CFG.deadzone ? 0 : Math.sign(v) * ((a - CFG.deadzone) / (1 - CFG.deadzone)) ** 1.6; };
new Pad($('pad-left'), { onChange: (v) => { S.axes.l = v; } });
new Pad($('pad-right'), { onChange: (v) => { S.axes.r = v; } });
new Pad($('pad-wrist'), { axes: 'y', onChange: (v) => { S.axes.wrist = v[1]; } });
const padDirs = Array.from(document.querySelectorAll('#stick-zone .pad .dir'));
const cartDirs = Array.from(document.querySelectorAll('.cart-sticks .pad .dir, #pad-sxy .dir, #pad-srot .dir, #pad-sz .vz'));

/* =================================================================
   Keyboard: [joint, direction] keyed by e.code for OS/layout immunity
   ================================================================= */
function getCodeMap() {
  const cm = {
    KeyA: [0, -1], KeyD: [0, 1], KeyS: [1, -1], KeyW: [1, 1],
    ArrowDown: [2, -1], ArrowUp: [2, 1],
    KeyQ: [3, -1], KeyE: [3, 1],
    ArrowLeft: [4, -1], ArrowRight: [4, 1],
    KeyZ: [GRIP, -1], KeyX: [GRIP, 1],
  };
  if (S.dof > 6) {
    cm['KeyR'] = [5, -1]; cm['KeyF'] = [5, 1];
  }
  return cm;
}
window.addEventListener('keydown', (e) => {
  // The operator dialog owns Escape (cancel; it has its own E-stop button): an Escape meant to dismiss
  // an automatically opened PIN prompt must not trigger Stop while the dialog is open.
  if (e.key === 'Escape' && opIsOpen()) return;
  const isTyping = e.target && (e.target.tagName === 'TEXTAREA' || (e.target.tagName === 'INPUT' && (e.target.type === 'text' || e.target.type === 'password' || e.target.type === 'number')));
  if (isTyping) {
    if (e.key === 'Escape') { e.preventDefault(); stopMotion(); }
    return;
  }
  // Escape triggers Stop regardless of slider focus (Space E-Stop is handled in the top-most capture listener)
  if (e.key === 'Escape') { e.preventDefault(); stopMotion(); return; }
  if (takeModalOpen() || preemptModalOpen()) return;
  // Ignore any hotkey pressed with modifier keys (Ctrl, Meta/Cmd, Alt)
  if (e.ctrlKey || e.metaKey || e.altKey) return;

  const cm = getCodeMap();
  if (e.repeat) { if (cm[e.code] || CART_CODES[e.code]) e.preventDefault(); return; }
  if (cm[e.code] || CART_CODES[e.code]) {
    e.preventDefault();
    if (isLocked()) { flashLock(); return; }
    if (leaseModalOrPending()) return;
    if (S.leaseGeneration == null) { promptTakeControl(); return; }
    S.keys.add(e.code); return;
  }
  if (e.code === 'KeyG') {
    if (!S.obs || !canMove()) { flashLock(); return; }
    if (leaseModalOrPending()) return;
    if (S.leaseGeneration == null) { promptTakeControl(); return; }
    S.grip = (S.grip ?? 0) > 0.5 ? 0 : 1;
    S.gripCmdAt = Date.now();
    commandGrip();
  }
  if (e.code === 'KeyH') home();
  if (e.key >= '1' && e.key <= '4' && S.ui === 'expert') setSpeed(Number(e.key) - 1);
});
window.addEventListener('keyup', (e) => {
  S.keys.delete(e.code);
  if (e.key === 'Control' || e.key === 'Meta' || e.key === 'Alt') releaseAll();
});
const isSameLease = (snap) => !!(snap && snap.leaseGen != null && S.ws === snap.ws && S.leaseGeneration === snap.leaseGen);
const hasActiveCartReq = () => !!(C.cartReq && isSameLease(C.cartReq.snap));
const hasRunningStep = () => !!(C.stepActionId != null && S.obs?.action?.id === C.stepActionId && S.obs?.action?.state === 'running');
function clearLocalMotionTargets() {
  S.streamActive = false; S.servoTarget = null; C.tailUntil = 0; C.lastInput = -Infinity;
  C.cartHoldActive = false; C.cartActionId = null; C.cartActionUntil = 0; C.stepActionId = null;
  C.gripJog = 0; C.gripDrag = false; C.gripGestureBlocked = true; S.gripCmdAt = 0;
  const gr = $('grip-range');
  if (S.obs && Array.isArray(S.obs.positions) && S.obs.positions.length > GRIP) { S.grip = gripFrac(S.obs.positions[GRIP]); if (gr) { const g = Math.round(S.grip * 100); gr.value = String(g); gr.style.setProperty('--v', g + '%'); } } else S.grip = null;
  if (gr && document.activeElement === gr) gr.blur();
  if (S.obs && Array.isArray(S.obs.positions) && S.obs.positions.length >= 5) cartSync(S.obs.positions);
}
function releaseAll({ sendStop = true } = {}) {
  S.keys.clear();
  C.gripJog = 0;
  for (const p of pads) if (p.active()) p.end();
  for (const h of holdButtons) h.stop();
  for (const h of S.hold) { h[0] = 0; h[1] = 0; }
  for (const b of cartButtons) b.stop();
  for (const k in C.hold) { C.hold[k][0] = 0; C.hold[k][1] = 0; }
  if (C.cmd) C.goal = { ...C.cmd }; // drop any queued step that has not been executed yet
  if (C.T) C.Tg = C.T;
  if (hasRunningStep()) C.cartHoldActive = true;
  if (sendStop) requestMotionStop();
  else clearLocalMotionTargets();
}
function requestMotionStop() {
  const activeReq = hasActiveCartReq() ? C.cartReq : null, unstoppedReq = !!(activeReq && !activeReq.stopPending);
  const shouldStop = (S.streamActive || C.cartHoldActive || unstoppedReq || hasRunningStep()) && !S.presetRunning && S.leaseGeneration != null;
  if (activeReq) { activeReq.stopPending = true; activeReq.stopIntent = motionIntentSeq; }
  clearLocalMotionTargets();
  if (shouldStop) {
    api('/api/v1/motion/stop', {}, { quiet: true }).catch((e) => {
      if (!isCancelledErr(e) && e?.code !== 'LEASE_REQUIRED' && e?.code !== 'LEASE_LOST') toastOnce(e);
    });
  }
}
function stopWhenInputsReleased() {
  queueMicrotask(() => {
    if (commandVector().some((v) => v !== 0) || Object.values(cartVelocity()).some((v) => v !== 0) || cartGripVel() !== 0) return;
    const activeReq = hasActiveCartReq() ? C.cartReq : null;
    if ((!S.streamActive && !C.cartHoldActive && !(activeReq && !activeReq.stopPending) && !hasRunningStep()) || S.presetRunning) return;
    requestMotionStop();
  });
}
function handleBackgroundHide() {
  const wizNeedStop = CWZ.controlSnap ? !CWZ.controlSnap.stopped : !!CWZ.running, activeReq = hasActiveCartReq() ? C.cartReq : null;
  const needStop = !!(wizNeedStop || S.streamActive || C.cartHoldActive || hasRunningStep() || (activeReq && !activeReq.stopPending));
  if (activeReq) { activeReq.stopPending = true; activeReq.stopIntent = motionIntentSeq; } if (CWZ.controlSnap) CWZ.controlSnap.stopped = true;
  releaseAll({ sendStop: false }); invalidateControlIntents(); resetPoseMode();
  if (needStop && canSendStop()) api('/api/v1/motion/stop', {}, { quiet: true }).catch(() => {});
}
for (const event of ['pointerup', 'pointercancel', 'mouseup', 'touchend', 'touchcancel', 'keyup']) window.addEventListener(event, stopWhenInputsReleased);
window.addEventListener('blur', () => { if (document.hidden) handleBackgroundHide(); else { releaseAll(); resetPoseMode(); } });
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    handleBackgroundHide();
  } else {
    loadAuthStatus();
    // Returning from background/sleep: refresh telemetry & reconnect if half-open
    if (!S.ws || S.ws.readyState !== WebSocket.OPEN || Date.now() - S.obsAt > 1500) {
      // Same retirement as a dropped socket (warning, release notice), then reconnect now instead of after the backoff.
      if (S.ws) retireSocket(S.ws, { reconnect: true, markReleased: true, toastExpired: true });
      connectWS();
      loadSchema();
    }
  }
});

/* =================================================================
   Motion commands
   ================================================================= */
// UI-semantic command vector: +1 = toward the right-hand label of each joint.
function commandVector() {
  const v = new Array(S.dof).fill(0);
  if (S.mode === 'cart') return v; // the cartesian loop owns all joints in this mode
  if (S.mode === 'stick') {
    v[0] = shape(S.axes.l[0]); v[1] = shape(S.axes.l[1]);
    v[2] = shape(S.axes.r[1]); v[4] = shape(S.axes.r[0]);
    v[3] = shape(S.axes.wrist);
  }
  for (let i = 0; i < S.dof; i++) {
    if (S.hold[i]) {
      v[i] = clamp(v[i] + (S.hold[i][1] > 0 ? 1 : 0) - (S.hold[i][0] > 0 ? 1 : 0), -1, 1);
    }
  }
  const cm = getCodeMap();
  for (const c of S.keys) {
    if (cm[c]) {
      const [j, s] = cm[c];
      if (j < S.dof) v[j] = clamp(v[j] + s, -1, 1);
    }
  }
  return v;
}
function clampTarget(t) {
  const curPos = (S.obs && Array.isArray(S.obs.positions)) ? S.obs.positions : [];
  const res = new Array(S.dof);
  for (let i = 0; i < S.dof; i++) {
    let val = (i < t.length && Number.isFinite(t[i])) ? t[i] : (curPos[i] || 0);
    const L = S.limits[i];
    if (L) val = clamp(val, L.min_position + CFG.limitMargin, L.max_position - CFG.limitMargin);
    if (i === GRIP) val = clamp(val, Math.min(CFG.gripClosed, CFG.gripOpen), Math.max(CFG.gripClosed, CFG.gripOpen));
    res[i] = val;
  }
  return res;
}

// Continuous joint-velocity teleop via /motion/servo (20 Hz); release = stop.
setInterval(() => {
  const v = commandVector();
  const moving = v.some((x) => x !== 0);
  const now = performance.now();
  if (leaseModalOrPending()) { S.servoTarget = null; return; }
  if (S.presetRunning) return;
  if (gcOn()) { S.servoTarget = null; gcInput(v); return; } // hand-guiding: no servo frames
  if (!moving || !isLive() || !canMove() || S.epoch == null) { S.servoTarget = null; return; }
  if (S.leaseGeneration == null) { S.servoTarget = null; promptTakeControl(); return; }
  const pos = S.obs.positions;
  if (!S.servoTarget || now - S.servoLastAt > 300) {
    S.servoTarget = pos.slice(0, S.dof);
    S.servoLastAt = now - CFG.servoPeriodMs;
    if (pos.length > GRIP && S.grip == null) S.grip = gripFrac(pos[GRIP]);
  }
  const dt = Math.min(0.1, (now - S.servoLastAt) / 1000);
  S.servoLastAt = now;
  const scale = speedScale();
  if (pos.length > GRIP && v[GRIP] !== 0) {
    // + = close (fraction toward 0), - = open
    const cur = S.grip ?? gripFrac(pos[GRIP]);
    S.grip = gripNoWindup(clamp(cur - v[GRIP] * CFG.gripRate * scale * dt, 0, 1), pos[GRIP]);
    S.gripCmdAt = Date.now();
  }
  let t = S.servoTarget.map((x, i) => {
    if (i === GRIP) return gripRad();
    const sign = CFG.jogSign[i] ?? 1;
    const nx = x + v[i] * sign * (S.limits[i]?.max_velocity ?? 1) * scale * dt;
    return clamp(nx, pos[i] - CFG.maxLead, pos[i] + CFG.maxLead);
  });
  t = clampTarget(t);
  S.servoTarget = t;
  // Streaming path: one fire-and-forget frame per tick, no request/response round trip
  if (sendServoFrame(t)) return;
  // No HTTP fallback: only the lease-owning socket may actuate.

}, CFG.servoPeriodMs);
let lastErrAt = 0;
function toastOnce(e) { if (isCancelledErr(e)) return; if (Date.now() - lastErrAt > 2000) { lastErrAt = Date.now(); toast(e.message, 'err', e.code); } }

