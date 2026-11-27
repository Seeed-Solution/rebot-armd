/* =================================================================
   5-DOF cartesian teleop: XYZ + wrist pitch -> client IK -> joint stream
   ================================================================= */
const toModel = (pos) => [0, 1, 2, 3].map((i) => (pos[i] - CFG.cart.off[i]) / CFG.cart.sign[i]);
// J1..J4 only; further entries (presets: J5, J6, gripper) are motor angles and pass through
const toMotor = (q) => q.map((x, i) => (i < 4 ? CFG.cart.sign[i] * x + CFG.cart.off[i] : x));
// motor J1..J6 (missing = 0) <-> model J1..J6
const toModel6 = (pos) => [0, 1, 2, 3, 4, 5].map((i) => ((Number(pos[i]) || 0) - CFG.cart.off[i]) / CFG.cart.sign[i]);
const toMotor6 = (q) => [0, 1, 2, 3, 4, 5].map((i) => CFG.cart.sign[i] * q[i] + CFG.cart.off[i]);
function inMotorLimits(q) {
  const m = toMotor(q);
  const curM = (S.obs && Array.isArray(S.obs.positions)) ? S.obs.positions : null;
  for (let i = 0; i < 4; i++) {
    const L = S.limits[i];
    if (L) {
      const min = L.min_position + CFG.limitMargin;
      const max = L.max_position - CFG.limitMargin;
      if (m[i] < min) {
        if (!curM || m[i] <= curM[i] - 1e-4) return false;
      }
      if (m[i] > max) {
        if (!curM || m[i] >= curM[i] + 1e-4) return false;
      }
    }
  }
  return true;
}
const IK_MSG = {
  reach: 'ik.reach', singular: 'ik.singular',
  limit: 'ik.limit', jump: 'ik.jump', floor: 'ik.floor', pose: 'ik.pose',
};
// camera-view frame: only on the 7-DOF arm (index 4 = URDF joint5, index 5 = joint6).
// Simple mode always uses it; the stored view/base choice applies to expert mode only.
const viewActive = () => S.dof === 7 && (S.ui === 'simple' || C.frame === 'view');
function cartSync(pos) {
  if (isSoArmProfile()) {
    const tcp = S.obs?.tcp;
    C.cmd = tcp && Array.isArray(tcp.xyz_m) ? { x: tcp.xyz_m[0], y: tcp.xyz_m[1], z: tcp.xyz_m[2], pitch: tcp.rpy_rad?.[1] ?? 0 } : { x: 0.25, y: 0, z: 0.10, pitch: 0 };
    C.goal = { ...C.cmd };
    C.j5 = pos[4] ?? 0;
    C.branch = 'URDF';
    C.T = null;
    C.Tg = null;
    C.q6 = null;
    C.wq = [0, 0];
    return;
  }
  C.q = toModel(pos);
  C.cmd = IK.fk(C.q);
  C.goal = { ...C.cmd };
  C.j5 = pos[4] ?? 0;
  C.branch = null;
  if (pos.length >= 6) { C.q6 = toModel6(pos); C.T = C.Tg = VIK.fk(C.q6); }
  C.wq = [0, 0];
}
// drop queued motion: the goal snaps back to the last solved pose (both frames)
function cartHoldStill() {
  if (C.goal && C.cmd) C.goal = { ...C.cmd };
  if (C.T) C.Tg = C.T;
  C.wq = [0, 0];
}
// observation.motion_limit (armd's servo guard) as a short hint chip. speed / lead show only
// after 1 s, and every hint stays 1 s after the limit clears, so short limits do not flicker.
const ML = { key: '', j: '', until: 0, blocked: false, gripTrips: null };
// Supply load meter: o.power (estimate) in % of the rating; ticks at the governor's soft / hard
// fractions (schema.power_budget.governor, defaults 0.8 / 0.95), peak marker = 5 s peak.
var PWR = { key: "", alarmKey: "" };
function renderPower(o) {
  const el = $('pwr');
  const alarmEl = $('pwr-alarm');
  const p = o.power;
  if (alarmEl) {
    const sev = p && (p.severity === 'trip' || p.severity === 'warning') ? p.severity : 'none';
    const aKey = sev + '|' + LANG;
    if (aKey !== PWR.alarmKey) {
      PWR.alarmKey = aKey;
      const show = sev !== 'none';
      alarmEl.hidden = !show;
      alarmEl.classList.toggle('warn', sev === 'warning');
      alarmEl.classList.toggle('trip', sev === 'trip');
      if (show) {
        setText($('pwr-alarm-title'), t(sev === 'trip' ? 'pwr.alarm.trip' : 'pwr.alarm.warn'));
        setText($('pwr-alarm-sub'), t('pwr.alarm.sub'));
      }
    }
  }
  if (!el) return;
  if (!p || !(p.rated_a > 0)) { if (!el.hidden) el.hidden = true; return; }
  if (el.hidden) el.hidden = false;
  const span = 120, pctOf = (a) => Math.max(0, Math.min(span, 100 * a / p.rated_a)) / span * 100;
  const gov = S.powerGov || {};
  const soft = 100 * (gov.soft || 0.8), hard = 100 * (gov.hard || 0.95);
  const pct = p.valid ? 100 * p.bus_a / p.rated_a : null;
  const band = pct == null ? 'na' : (p.hold || pct >= hard) ? 'over' : (p.limiting || pct >= soft) ? 'warn' : 'ok';
  const key = [band, pct == null ? '' : Math.round(pct), p.valid ? Math.round(10 * p.peak_a) : '', soft, hard, LANG].join('|');
  if (key === PWR.key) return;
  PWR.key = key;
  el.classList.toggle('na', band === 'na'); el.classList.toggle('warn', band === 'warn'); el.classList.toggle('over', band === 'over');
  $('pwr-fill').style.width = (pct == null ? 0 : pctOf(p.bus_a)) + '%';
  $('pwr-soft').style.left = (soft / span * 100) + '%';
  $('pwr-hard').style.left = (hard / span * 100) + '%';
  const pk = $('pwr-peak');
  pk.style.display = p.valid && p.peak_a > 0 ? 'block' : 'none';
  pk.style.left = pctOf(p.peak_a) + '%';
  setText($('pwr-pct'), pct == null ? '--' : Math.round(pct) + '%');
  el.setAttribute('aria-valuenow', pct == null ? '0' : String(Math.round(pct)));
  el.title = pct == null ? t('pwr.na') : t('pwr.title', { a: p.bus_a.toFixed(2), r: p.rated_a, pct: Math.round(pct), pk: p.peak_a.toFixed(2) });
}
function renderMotionLimit(o) {
  const el = $('ml-chip');
  if (!el) return;
  const ml = o.motion_limit, now = Date.now();
  // gripper torque cap trips (observation.gripper.force_limit_trips): a toast per new trip, so a
  // trip shorter than the observation interval is still reported
  const trips = o.gripper && o.gripper.force_limit_trips;
  if (typeof trips === 'number') {
    if (ML.gripTrips != null && trips > ML.gripTrips) toast(t('ml.grip_force'), 'err', 'GRIPPER_FORCE');
    ML.gripTrips = trips;
  }
  if (ml) {
    const age = o.timestamp * 1000 - ml.since_ms;
    const key = ml.reason === 'joint_limit' ? 'ml.limit' : ml.reason === 'blocked' ? 'ml.blocked'
      : ml.reason === 'torque' ? 'ml.torque' : ml.reason === 'workspace' ? 'ml.workspace'
      : ml.reason === 'gripper_force' ? 'ml.grip_force'
      : ml.reason === 'power' ? (o.power && o.power.hold ? 'ml.power' : 'ml.power_slow')
      : age >= 1000 ? (ml.joint != null && ml.joint < 3 ? 'ml.slow_big' : 'ml.slow') : '';
    if (key) {
      ML.key = key;
      ML.j = ml.joint == null ? '' : 'J' + (ml.joint + 1);
      ML.wsDetail = ml.reason === 'workspace' ? (o.workspace_violation || null) : null;
      ML.until = now + 1000;
    }
    // armd ended the stream in a hold: drop the queued motion once so the goal does not pull on
    const held = ml.reason === 'blocked' || ml.reason === 'torque' || (ml.reason === 'power' && !!(o.power && o.power.hold));
    if (held && !ML.blocked) cartHoldStill();
    ML.blocked = held;
  } else ML.blocked = false;
  const show = !!ML.key && now < ML.until;
  el.hidden = !show;
  if (show) {
    const wsBox = ML.key === 'ml.workspace' && ML.wsDetail ? formatWorkspaceViolation(ML.wsDetail) : '';
    setText(el, wsBox || t(ML.key, { j: ML.j }));
  }
}
function cartReject(err) {
  C.goal = { ...C.cmd };
  if (C.T) C.Tg = C.T;
  if (C.status !== err) { buzz(30); toastOnce({ message: ikText(IK_MSG[err] || err), code: 'IK' }); }
  C.status = err; C.msg = IK_MSG[err] || err;
}
// UI command in base-frame semantics: x forward, y left, z up, p = pitch down (+)
const CART_CODES = {
  KeyW: ['x', 1], KeyS: ['x', -1], KeyA: ['y', 1], KeyD: ['y', -1],
  ArrowUp: ['z', 1], ArrowDown: ['z', -1],
  KeyQ: ['p', 1], KeyE: ['p', -1],
  ArrowLeft: ['r', -1], ArrowRight: ['r', 1],
};
function cartVelocity() {
  let vx = shape(C.axes.xy[1]);
  let vy = -shape(C.axes.xy[0]);
  let vz = shape(C.axes.z);
  let vp = -shape(C.axes.p);
  let vr = 0;
  if (S.ui === 'simple') {
    vx = shape(C.axes.sxy[1]);
    vy = -shape(C.axes.sxy[0]);
    vz = shape(C.axes.sz);
    // translation is normalised below; pitch / J5 wrist are separate angular rates, clamped on their own
    vp = -shape(C.axes.sp);
    vr = shape(C.axes.sr);
  }
  const v = { x: vx, y: vy, z: vz, p: vp, r: vr };
  for (const a in v) v[a] = clamp(v[a] + (C.hold[a][1] > 0 ? 1 : 0) - (C.hold[a][0] > 0 ? 1 : 0), -1, 1);
  for (const c of S.keys) { const m = CART_CODES[c]; if (m) v[m[0]] = clamp(v[m[0]] + m[1], -1, 1); }
  // Keep combined Cartesian translation inside the unit ball.  Each axis is
  // still independently clamped above, but two or three simultaneous inputs
  // must not increase the commanded translational speed beyond one axis.
  const xyzNorm = Math.hypot(v.x, v.y, v.z);
  if (xyzNorm > 1) {
    v.x /= xyzNorm;
    v.y /= xyzNorm;
    v.z /= xyzNorm;
  }
  return v;
}
function cartGripVel() {
  const keyVel = (S.keys.has('KeyX') ? -1 : 0) + (S.keys.has('KeyZ') ? 1 : 0); // +1 = open, -1 = close
  return clamp(keyVel + (C.gripJog || 0), -1, 1);
}
function clampJ5(x) {
  const L = S.limits[4];
  let v = L ? clamp(x, L.min_position + CFG.limitMargin, L.max_position - CFG.limitMargin) : x;
  const pos = S.obs && S.obs.positions;
  if (pos && pos.length > 4) v = clamp(v, pos[4] - CFG.maxLead, pos[4] + CFG.maxLead);
  return v;
}
function cartKick() { C.lastInput = performance.now(); C.tailUntil = C.lastInput + CFG.cart.tailMs; }
function cartStep(ax, s) {
  if (leaseModalOrPending()) return;
  if (S.leaseGeneration == null) { promptTakeControl(); return; }
  if (isSoArmProfile()) {
    const n = CFG.cart.steps[C.stepIdx];
    if (ax === 'x' || ax === 'y' || ax === 'z') {
      if (C.cartReq) return;
      const req = { snap: captureControlSnap(), stopPending: false, stopIntent: null };
      C.cartReq = req;
      commandRPC('/api/v1/action/move_relative', {
        frame: 'base',
        d: [
          ax === 'x' ? s * n / 1000 : 0,
          ax === 'y' ? s * n / 1000 : 0,
          ax === 'z' ? s * n / 1000 : 0,
        ],
      }).then((ack) => {
        syncEpoch(ack?.epoch_id);
        if (req.stopPending) {
          req.stopPending = false; C.stepActionId = null; C.cartHoldActive = false; C.cartActionId = null; C.cartActionUntil = 0;
          if (isSameLease(req.snap) && S.epoch === req.snap.epoch && motionIntentSeq === req.stopIntent) api('/api/v1/motion/stop', {}, { quiet: true }).catch((e) => toastOnce(e));
        } else if (isControlSnapValid(req.snap)) {
          C.stepActionId = ack?.action_id ?? null;
        }
      }).catch(toastOnce).finally(() => { if (C.cartReq === req) C.cartReq = null; });
    } else if (ax === 'p' || ax === 'r') {
      const pos = (S.obs && Array.isArray(S.obs.positions)) ? S.obs.positions : (S.home || []);
      const jIdx = ax === 'p' ? 3 : 4;
      const nextPos = pos.slice(0, S.dof);
      while (nextPos.length < S.dof) nextPos.push(0);
      nextPos[jIdx] = (nextPos[jIdx] || 0) + s * n / DEG;
      commandRPC('/api/v1/motion/jog', { target_positions: clampTarget(nextPos) }).catch(toastOnce);
    }
    cartKick();
    return;
  }
  if (viewActive()) {
    if (!C.Tg) return;
    const n = CFG.cart.steps[C.stepIdx], v = { x: 0, y: 0, z: 0, p: 0, r: 0 };
    // look up / down and left / right: n degrees of J4 / J5, played out by cartTickView at the look rate
    if (ax === 'p' || ax === 'r') { C.wq[ax === 'p' ? 0 : 1] += s * n / DEG; cartKick(); return; }
    v[ax] = s; // one tap: n mm along the camera axis / world Z
    C.Tg = VIK.integrate(C.Tg, v, 1, { vl: n / 1000, wl: n / DEG, Rec: CFG.view.Rec, levelCos: CFG.view.levelCos });
    cartKick();
    return;
  }
  if (!C.goal) return;
  const n = CFG.cart.steps[C.stepIdx];
  if (ax === 'p') C.goal.pitch += s * n / DEG;
  else if (ax === 'r') C.j5 = clampJ5(C.j5 + s * n / DEG);
  else C.goal[ax] += s * n / 1000;
  cartKick();
}

function cartSend(t) {
  if (leaseModalOrPending() || S.leaseGeneration == null) return;
  if (CFG.cart.tx === 'rest') {
    if (C.restInflight) return;
    C.restInflight = true; S.streamActive = true;
    api('/api/v1/motion/servo', { targets: t, epoch_id: S.epoch }, { quiet: true })
      .catch((e) => { if (e.code && e.code !== 'EPOCH_MISMATCH') toastOnce(e); })
      .finally(() => { C.restInflight = false; });
    return;
  }
  sendServoFrame(t);
}

// The only WS servo sender. Never sends without a granted lease generation.
function sendServoFrame(positions) {
  if (gcOn() || leaseModalOrPending()) return false; // the server would answer MOTION_BUSY
  if (!S.ws || S.ws.readyState !== 1) return false;
  if (S.leaseGeneration == null) { promptTakeControl(); return false; }
  S.streamActive = true;
  S.lastIntentAt = Date.now();
  S.ws.send(JSON.stringify({
    type: 'servo',
    client_id: CLIENT_ID,
    generation: S.leaseGeneration,
    sequence_id: ++S.sequenceId,
    epoch_id: S.epoch,
    positions,
  }));
  motionIntentSeq++;
  return true;
}

function cartTick() {
  C.sending = false;
  if (gcOn()) {
    if (Object.values(cartVelocity()).some((x) => x !== 0)) gcSticksOff();
    if (S.obs && S.obs.positions.length >= 5) cartSync(S.obs.positions);
    C.tailUntil = 0; return;
  }
  if (S.mode !== 'cart' || !S.obs || S.obs.positions.length < 5) return;
  if (viewActive() && S.obs.positions.length >= 6) { cartTickView(); return; }
  const pos = S.obs.positions;
  const now = performance.now();
  const dt = Math.min(0.1, (now - (C.lastTick || now)) / 1000);
  C.lastTick = now;
  if (!isLive() || !canMove() || S.epoch == null) {
    cartSync(pos); C.tailUntil = 0; C.status = 'idle'; C.msg = '';
    return;
  }
  if (leaseModalOrPending() || S.leaseGeneration == null) {
    const v = cartVelocity();
    const gv = cartGripVel();
    if ((v.x || v.y || v.z || v.p || v.r || gv !== 0) && !leaseModalOrPending()) promptTakeControl();
    cartSync(pos); C.tailUntil = 0; C.status = 'idle'; C.msg = '';
    return;
  }
  if (isSoArmProfile()) {
    const tcp = S.obs.tcp;
    if (tcp && Array.isArray(tcp.xyz_m)) {
      C.cmd = { x: tcp.xyz_m[0], y: tcp.xyz_m[1], z: tcp.xyz_m[2], pitch: tcp.rpy_rad?.[1] ?? 0 };
      C.goal = { ...C.cmd };
    }
    const v = cartVelocity();
    const gv = cartGripVel();
    const scale = speedScale();
    const gripErr = pos.length > GRIP ? Math.abs(gripRad() - pos[GRIP]) : 0;
    const gripStalled = S.gripCmdAt > 0 && Date.now() - S.gripCmdAt > 1200 && Math.abs(S.obs.velocities?.[GRIP] ?? 0) < 0.05;
    if (gripStalled) S.gripCmdAt = 0;
    const gripMoving = (S.gripCmdAt > 0 && gripErr > 0.015 && !gripStalled) || gv !== 0;
    const hasInput = v.x || v.y || v.z || v.p || v.r || gripMoving;
    if (hasInput) cartKick();
    else if (now > C.tailUntil) { S.streamActive = false; S.gripCmdAt = 0; }

    if (pos.length > GRIP && gv) {
      const cur = S.grip ?? gripFrac(pos[GRIP]);
      S.grip = gripNoWindup(clamp(cur + gv * CFG.gripRate * scale * dt, 0, 1), pos[GRIP]);
      S.gripCmdAt = Date.now();
    }

    const actionRunning = S.obs?.action?.state === 'running';
    if (C.cartActionId != null && S.obs?.action?.id === C.cartActionId && !actionRunning) {
      C.cartActionId = null;
      C.cartActionUntil = 0;
    }
    const actionBusy = actionRunning || (C.cartActionId != null && now < C.cartActionUntil);

    if (v.x || v.y || v.z) {
      C.status = 'ok';
      C.msg = '';
      if (!C.cartReq && !actionBusy && !S.presetRunning) {
        const req = { snap: captureControlSnap(), stopPending: false, stopIntent: null };
        C.cartReq = req;
        C.cartHoldActive = true;
        const stepM = scale * 0.015;
        const speed = {
          linear: clamp(0.05 * scale, 0.01, 0.15),
          angular: clamp(0.5 * scale, 0.05, 1.0),
        };
        commandRPC('/api/v1/action/move_relative', {
          frame: 'base',
          d: [v.x * stepM, v.y * stepM, v.z * stepM],
          speed,
        }).then((ack) => {
          syncEpoch(ack?.epoch_id);
          if (req.stopPending) {
            req.stopPending = false; C.stepActionId = null; C.cartHoldActive = false; C.cartActionId = null; C.cartActionUntil = 0;
            if (isSameLease(req.snap) && S.epoch === req.snap.epoch && motionIntentSeq === req.stopIntent) api('/api/v1/motion/stop', {}, { quiet: true }).catch((e) => toastOnce(e));
          } else if (isControlSnapValid(req.snap)) {
            C.cartActionId = ack?.action_id ?? null;
            C.cartActionUntil = performance.now() + Math.min(400, (ack?.expected_duration_s || 0.2) * 1000);
          }
        }).catch((e) => {
          if (e && e.code === 'MOTION_BUSY') return;
          C.cartHoldActive = false;
          toastOnce(e);
        }).finally(() => { if (C.cartReq === req) C.cartReq = null; });
      }
    }

    if (v.p || v.r || gripMoving) {
      const wl = CFG.cart.wmax * scale;
      const cmdPos = (Array.isArray(S.obs?.commanded_positions) && S.obs.commanded_positions.length >= S.dof
        && S.obs.commanded_positions.slice(0, S.dof).every(Number.isFinite))
        ? S.obs.commanded_positions : pos;
      let t = cmdPos.slice(0, S.dof);
      while (t.length < S.dof) t.push(0);
      if (t.length > 3 && v.p) t[3] = clamp(t[3] + v.p * wl * dt, S.limits[3].min_position, S.limits[3].max_position);
      if (t.length > 4 && v.r) t[4] = clamp(t[4] + v.r * wl * dt, S.limits[4].min_position, S.limits[4].max_position);
      if (t.length > GRIP) t[GRIP] = gripRad();
      t = clampTarget(t);
      C.sending = true;
      if (!S.presetRunning && !(v.x || v.y || v.z) && !C.cartReq && !actionBusy) cartSend(t);
    }
    return;
  }
  const v = cartVelocity();
  const gv = cartGripVel();
  const gripErr = pos.length > GRIP ? Math.abs(gripRad() - pos[GRIP]) : 0;
  // stopped = no new command for 1.2 s AND the jaw is no longer moving (a full stroke takes ~4 s)
  const gripStalled = S.gripCmdAt > 0 && Date.now() - S.gripCmdAt > 1200 && Math.abs(S.obs.velocities?.[GRIP] ?? 0) < 0.05;
  if (gripStalled) S.gripCmdAt = 0;
  const gripMoving = (S.gripCmdAt > 0 && gripErr > 0.015 && !gripStalled) || gv !== 0;
  const input = v.x || v.y || v.z || v.p || v.r || gripMoving;
  if (input) cartKick();
  const settling = now - C.lastInput < CFG.cart.settleMs;
  const pending = !C.goal || C.goal.x !== C.cmd.x || C.goal.y !== C.cmd.y || C.goal.z !== C.cmd.z || C.goal.pitch !== C.cmd.pitch;
  if (!C.goal || (!input && !pending && !settling && now > C.tailUntil)) {
    S.streamActive = false; S.gripCmdAt = 0;
    cartSync(pos); // idle: track the real arm so the next command starts from where it is
    if (C.status !== 'ok' && C.status !== 'idle' && now - C.lastInput > 1500) { C.status = 'idle'; C.msg = ''; }
    return;
  }

  const scale = speedScale();
  const vl = CFG.cart.vmax * scale, wl = CFG.cart.wmax * scale;
  C.goal.x += v.x * vl * dt; C.goal.y += v.y * vl * dt; C.goal.z += v.z * vl * dt;
  C.goal.pitch += v.p * wl * dt;
  if (v.r) C.j5 = clampJ5(C.j5 + v.r * wl * dt);

  // Anti-windup: do not let C.goal drift more than 0.25 m away from current C.cmd while dragging/streaming
  const gDist = Math.hypot(C.goal.x - C.cmd.x, C.goal.y - C.cmd.y, C.goal.z - C.cmd.z);
  if (gDist > 0.25) {
    const r = 0.25 / gDist;
    C.goal.x = C.cmd.x + (C.goal.x - C.cmd.x) * r;
    C.goal.y = C.cmd.y + (C.goal.y - C.cmd.y) * r;
    C.goal.z = C.cmd.z + (C.goal.z - C.cmd.z) * r;
  }

  // rate-limited follower: step taps and joystick input both reach the arm as a smooth ramp
  const dx = C.goal.x - C.cmd.x, dy = C.goal.y - C.cmd.y, dz = C.goal.z - C.cmd.z;
  const d = Math.hypot(dx, dy, dz);
  let f = d > vl * dt ? vl * dt / d : 1;
  let dp = clamp(C.goal.pitch - C.cmd.pitch, -wl * dt, wl * dt);

  // lag guard: do not let the command run ahead of a slow / blocked arm
  const mq = toMotor(C.q);
  const lag = Math.max(...mq.map((x, i) => Math.abs(x - pos[i])));

  const a = 0.20, b = 0.35;
  const factor = clamp(1.0 - (lag - a) / (b - a), 0.0, 1.0);
  f *= factor; dp *= factor;

  // Hard stall & collision protection: if lag > 0.40 rad continuously for > 500 ms, emergency stop goal
  if (lag > 0.40) {
    if (!C.stallSince) C.stallSince = now;
    else if (now - C.stallSince > 500) {
      C.goal = { ...C.cmd };
      C.status = 'stall';
      C.msg = 'ik.lag';
    }
  } else {
    C.stallSince = 0;
  }
  const cand = { x: C.cmd.x + dx * f, y: C.cmd.y + dy * f, z: C.cmd.z + dz * f, pitch: C.cmd.pitch + dp };

  if (d > 1e-7 || Math.abs(dp) > 1e-7) {
    const sol = IK.solve(cand, { prev: C.q, branch: C.branch, inLimits: inMotorLimits });
    const r1 = IK.reach(cand), r0 = IK.reach(C.cmd);
    if (!sol.ok) cartReject(sol.err);
    else if (r1 > CFG.cart.reachMax && r1 > r0) cartReject('reach');
    else if (cand.z < CFG.cart.zMin && cand.z < C.cmd.z) cartReject('floor');
    else if (Math.max(...sol.q.map((x, i) => Math.abs(x - C.q[i]))) > CFG.cart.maxJump) cartReject('jump');
    else {
      C.cmd = cand; C.q = sol.q; C.branch = sol.branch;
      if (C.status !== 'ok') { C.status = 'ok'; C.msg = ''; }
    }
  } else if (lag > 0.05 && now > C.tailUntil) {
    // When idle and settled, gently resync cmd/q to measured position
    cartSync(pos);
  }

  // gripper (keys Z / X; rate jog or slider writes S.grip)
  if (pos.length > GRIP && gv) {
    const cur = S.grip ?? gripFrac(pos[GRIP]);
    S.grip = gripNoWindup(clamp(cur + gv * CFG.gripRate * scale * dt, 0, 1), pos[GRIP]);
    S.gripCmdAt = Date.now();
  }

  const m = toMotor(C.q);
  let t = pos.slice(0, S.dof);
  for (let i = 0; i < 4; i++) t[i] = m[i];
  if (t.length > 4) t[4] = C.j5;
  if (t.length > 5) t[5] = pos[5] || 0;
  if (t.length > GRIP) t[GRIP] = gripRad();
  t = clampTarget(t);
  const converging = t.some((x, i) => Math.abs(x - pos[i]) > 0.02);
  t = t.map((x, i) => clamp(x, pos[i] - (i === GRIP ? CFG.gripLead : CFG.maxLead), pos[i] + (i === GRIP ? CFG.gripLead : CFG.maxLead)));
  if (now <= C.tailUntil || (settling && converging)) { C.sending = true; if (!S.presetRunning) cartSend(t); }
}
// the rest pose (joint angles): the schema's home_positions (armd derives them from the profile's
// zero_reference), all 0 before the schema arrives (B601-DM official convention)
function restJoints() { return Array.isArray(S.home) && S.home.length >= 6 ? S.home : [0, 0, 0, 0, 0, 0, 0]; }
// camera mount at the rest pose, recomputed when the schema brings the joint map and home
const camMountNow = () => { CFG.view.Rec = VIK.camMount(CFG.view.camFlip, CFG.view.camRotDeg, toModel6(restJoints()), CFG.view.qEndCam); };
camMountNow();
// model-space limits for J1..J6: motor soft limits minus the margin, through sign / offset
function modelLimits6() {
  const lo = [], hi = [];
  for (let i = 0; i < 6; i++) {
    const L = S.limits[i];
    if (!L) { lo.push(-Infinity); hi.push(Infinity); continue; }
    let a = L.min_position + CFG.limitMargin, b = L.max_position - CFG.limitMargin;
    a = (a - CFG.cart.off[i]) / CFG.cart.sign[i]; b = (b - CFG.cart.off[i]) / CFG.cart.sign[i];
    lo.push(Math.min(a, b)); hi.push(Math.max(a, b));
  }
  return { lo, hi };
}
// Camera-view frame: left stick = forward (optical axis) / strafe (image x), lift = world Z, both through
// the 6-joint IK with the orientation held; right stick = wrist only: up / down drives J4, left / right
// drives J5 as joint velocities (J1..J3 and J6 stay put). Same guards as cartTick.
// Signs from VIK.fk at the inspect pose (J4 = 1.435, J5 = 0, J6 = pi/2) and at J4 = 0: J4 + moves the
// optical axis toward +y_cam (image down), J5 + moves it toward +x_cam (image right). With v.p = look
// down (+) and v.r = look right (+): dJ4 = +v.p, dJ5 = +v.r.
const VIEW_WRIST_SIGN = [1, 1];
// Move model J4 / J5 by dq (rad), clamped to the joint limits (a joint already past its margin may stay but
// not go further), and re-derive T from FK. A pending translation goal keeps its offset from T and takes
// the new orientation. Returns true if a joint moved.
function viewWrist(dq) {
  const { lo, hi } = modelLimits6();
  const q = C.q6.slice();
  for (const [k, i] of [[0, 3], [1, 4]]) q[i] = clamp(q[i] + dq[k], Math.min(lo[i], C.q6[i]), Math.max(hi[i], C.q6[i]));
  const moved = q[3] !== C.q6[3] || q[4] !== C.q6[4];
  if (!moved) {
    if (C.status !== 'limit') { buzz(30); toastOnce({ message: ikText(IK_MSG.limit), code: 'IK' }); }
    C.status = 'limit'; C.msg = IK_MSG.limit;
    return false;
  }
  const T = VIK.fk(q), p0 = CALM.posOf(C.T), p1 = CALM.posOf(T);
  if (p1[2] < CFG.cart.zMin && p1[2] < p0[2]) { cartReject('floor'); return false; }
  const pg = CALM.posOf(C.Tg);
  C.Tg = C.Tg === C.T ? T : CALM.make(CALM.rotOf(T), p1.map((x, k) => x + pg[k] - p0[k]));
  C.T = T; C.q6 = q;
  if (C.status !== 'ok') { C.status = 'ok'; C.msg = ''; }
  return true;
}
function cartTickView() {
  const pos = S.obs.positions;
  const now = performance.now();
  const dt = Math.min(0.1, (now - (C.lastTick || now)) / 1000);
  C.lastTick = now;
  if (!isLive() || !canMove() || S.epoch == null || !C.q6) {
    cartSync(pos); C.tailUntil = 0; C.status = 'idle'; C.msg = '';
    return;
  }
  if (leaseModalOrPending() || S.leaseGeneration == null) {
    const v = cartVelocity();
    const gv = cartGripVel();
    if ((v.x || v.y || v.z || v.p || v.r || gv !== 0) && !leaseModalOrPending()) promptTakeControl();
    cartSync(pos); C.tailUntil = 0; C.status = 'idle'; C.msg = '';
    return;
  }
  const v = cartVelocity();
  const gv = cartGripVel();
  const gripErr = pos.length > GRIP ? Math.abs(gripRad() - pos[GRIP]) : 0;
  const gripStalled = S.gripCmdAt > 0 && Date.now() - S.gripCmdAt > 1200 && Math.abs(S.obs.velocities?.[GRIP] ?? 0) < 0.05;
  if (gripStalled) S.gripCmdAt = 0;
  const gripMoving = (S.gripCmdAt > 0 && gripErr > 0.015 && !gripStalled) || gv !== 0;
  const motion = v.x || v.y || v.z || v.p || v.r;
  const input = motion || gripMoving;
  if (input) cartKick();
  const settling = now - C.lastInput < CFG.cart.settleMs;
  const pending = C.Tg !== C.T || C.wq[0] !== 0 || C.wq[1] !== 0;
  if (!C.Tg || (!input && !pending && !settling && now > C.tailUntil)) {
    S.streamActive = false; S.gripCmdAt = 0;
    cartSync(pos); // idle: track the real arm so the next command starts from where it is
    if (C.status !== 'ok' && C.status !== 'idle' && now - C.lastInput > 1500) { C.status = 'idle'; C.msg = ''; }
    return;
  }

  const scale = speedScale();
  const vl = CFG.cart.vmax * scale, wl = CFG.view.wmax * scale; // wl: rad/s of J4 / J5 for the look stick

  // lag guard over J1..J6: do not let the command run ahead of a slow / blocked arm
  const mq = toMotor6(C.q6);
  const lag = Math.max(...mq.map((x, i) => Math.abs(x - pos[i])));
  // tighter than base mode: a camera-frame move often needs one slow joint (J1) to cover most of the path
  const a = 0.08, b = 0.20;
  const factor = clamp(1.0 - (lag - a) / (b - a), 0.0, 1.0);
  let stalled = false;
  if (lag > 0.40) {
    if (!C.stallSince) C.stallSince = now;
    else if (now - C.stallSince > 500) stalled = true;
  } else {
    C.stallSince = 0;
  }

  // 1. look stick / look steps: wrist joints only, applied first; the result is the orientation held below
  let wp = v.p, wr = v.r;
  const wn = Math.hypot(wp, wr);
  if (wn > 1) { wp /= wn; wr /= wn; }
  const wmax = wl * dt * factor;
  const dq = [VIEW_WRIST_SIGN[0] * wp * wmax, VIEW_WRIST_SIGN[1] * wr * wmax];
  for (let k = 0; k < 2; k++) {
    const qs = clamp(C.wq[k], -wmax, wmax); // queued steps play out at the look rate
    C.wq[k] -= qs; dq[k] += qs;
  }
  if (stalled) C.wq = [0, 0];
  else if (dq[0] || dq[1]) { if (!viewWrist(dq)) C.wq = [0, 0]; }
  // 2. translation (left stick, lift) with the orientation held
  if (v.x || v.y || v.z) C.Tg = VIK.integrate(C.Tg, { x: v.x, y: v.y, z: v.z, p: 0, r: 0 }, dt, { vl, wl, Rec: CFG.view.Rec, levelCos: CFG.view.levelCos });
  if (stalled) { C.Tg = C.T; C.status = 'stall'; C.msg = 'ik.lag'; }
  // anti-windup: goal at most 0.25 m / 30 deg away from the solved pose
  C.Tg = VIK.clampGoal(C.T, C.Tg, 0.25, CFG.view.maxRot);
  // rate-limited follower: position <= vl dt, orientation <= wl dt (geodesic)
  const cand = VIK.approach(C.T, C.Tg, vl * dt * factor, wl * dt * factor);
  const step = VIK.diff(C.T, cand);

  if (step.d > 1e-7 || step.a > 1e-7) {
    const { lo, hi } = modelLimits6();
    const sol = VIK.solve(cand, C.q6, { lo, hi, w: CFG.view.wPos });
    const p1 = CALM.posOf(cand), p0 = CALM.posOf(C.T);
    const r1 = IK.reach({ x: p1[0], y: p1[1], z: p1[2] }), r0 = IK.reach({ x: p0[0], y: p0[1], z: p0[2] });
    if (!sol.ok) cartReject(sol.err);
    else if (r1 > CFG.cart.reachMax && r1 > r0) cartReject('reach');
    else if (p1[2] < CFG.cart.zMin && p1[2] < p0[2]) cartReject('floor'); // end_link origin (camera origin later)
    else if (Math.max(...sol.q.map((x, i) => Math.abs(x - C.q6[i]))) > CFG.cart.maxJump) cartReject('jump');
    else {
      C.T = cand; C.q6 = sol.q;
      if (C.status !== 'ok') { C.status = 'ok'; C.msg = ''; }
    }
  } else if (lag > 0.05 && !settling) {
    cartSync(pos); // still off after the settle window: the arm is blocked, give up the rest of the move
  }

  if (pos.length > GRIP && gv) {
    const cur = S.grip ?? gripFrac(pos[GRIP]);
    S.grip = gripNoWindup(clamp(cur + gv * CFG.gripRate * scale * dt, 0, 1), pos[GRIP]);
    S.gripCmdAt = Date.now();
  }

  const m = toMotor6(C.q6);
  let t = pos.slice(0, S.dof);
  for (let i = 0; i < 6; i++) t[i] = m[i];
  if (t.length > GRIP) t[GRIP] = gripRad();
  t = clampTarget(t);
  const converging = t.some((x, i) => Math.abs(x - pos[i]) > 0.02);
  // one shared lead factor for J1..J6: clamping each joint on its own lets the fast wrist joints arrive
  // while J1 trails, which turns a straight camera-frame move into a rotation
  const lead = Math.max(...t.slice(0, 6).map((x, i) => Math.abs(x - pos[i])));
  const k = lead > CFG.maxLead ? CFG.maxLead / lead : 1;
  for (let i = 0; i < 6; i++) t[i] = pos[i] + (t[i] - pos[i]) * k;
  if (t.length > GRIP) t[GRIP] = clamp(t[GRIP], pos[GRIP] - CFG.gripLead, pos[GRIP] + CFG.gripLead);
  if (now <= C.tailUntil || (settling && converging)) { C.sending = true; if (!S.presetRunning) cartSend(t); }
}
setInterval(cartTick, Math.round(1000 / CFG.cart.hz));
// performance.now() is clamped to ~100 us in browsers, so time a batch once at boot instead of per call
setTimeout(() => {
  const q = [0.2, -1.0, -2.2, 0.8], p = IK.fk(q), n = 20000;
  const t0 = performance.now();
  for (let i = 0; i < n; i++) IK.solve(p, { prev: q, inLimits: inMotorLimits });
  C.solveUs = (performance.now() - t0) * 1000 / n;
}, 1500);

/* step buttons: tap = one step, hold = continuous */
const ARROW = {
  up: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"><path d="M5 15l7-7 7 7"/></svg>',
  down: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"><path d="M5 9l7 7 7-7"/></svg>',
  left: CHEV_L, right: CHEV_R,
};
// step-button axis tags in the view frame: camera axes (Zc forward, Xc right), world Z, wrist joints
// (look up = J4−, turn right = J5+; see VIEW_WRIST_SIGN)
const VIEW_SUB = { 'X+': 'Zc+', 'X−': 'Zc−', 'Y+': 'Xc−', 'Y−': 'Xc+', 'P−': 'J4−', 'P+': 'J4+', 'R+': 'J5+', 'R−': 'J5−' };
function cartButton(parent, ax, s, label, sub, arrow) {
  const b = document.createElement('button');
  b.className = 'jbtn cbtn';
  b.dataset.ax = ax; b.dataset.s = s;
  const render = () => {
    const sb = (viewActive() && VIEW_SUB[sub]) || sub;
    b.setAttribute('aria-label', t(label) + ' ' + sb);
    b.innerHTML = `${ARROW[arrow]}<span>${t(label)}</span><b>${sb}</b>`;
  };
  render(); i18nHooks.push(render);
  let pid = null, timer = null, held = false;
  const idx = s > 0 ? 1 : 0;
  const stop = () => {
    if (pid === null) return;
    pid = null; clearTimeout(timer); b.classList.remove('pressed');
    if (held) { held = false; C.hold[ax][idx] = Math.max(0, C.hold[ax][idx] - 1); cartHoldStill(); }
  };
  b.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (pid !== null) return;
    if (isLocked()) { toast(lockReason(), 'err'); flashLock(); return; }
    if (leaseModalOrPending()) return;
    if (S.leaseGeneration == null) { promptTakeControl(); return; }
    pid = e.pointerId;
    try { b.setPointerCapture(pid); } catch (_) {}
    b.classList.add('pressed'); buzz(8);
    cartStep(ax, s);
    timer = setTimeout(() => { if (pid !== null) { held = true; C.hold[ax][idx]++; } }, CFG.cart.holdDelay);
  });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(ev, (e) => { if (e.pointerId === pid) stop(); });
  b.addEventListener('contextmenu', (e) => e.preventDefault());
  cartButtons.push({ stop });
  parent.appendChild(b);
}
(() => {
  const xy = $('dpad-xy');
  cartButton(xy, 'x', 1, 'd.fwd', 'X+', 'up');
  cartButton(xy, 'x', -1, 'd.back', 'X−', 'down');
  cartButton(xy, 'y', 1, 'd.left', 'Y+', 'left');
  cartButton(xy, 'y', -1, 'd.right', 'Y−', 'right');
  const z = $('vcol-z'), zl = z.firstElementChild;
  cartButton(z, 'z', 1, 'd.rise', 'Z+', 'up'); z.appendChild(zl);
  cartButton(z, 'z', -1, 'd.lower', 'Z−', 'down');
  const p = $('vcol-p'), pl = p.firstElementChild;
  cartButton(p, 'p', -1, 'cb.pup', 'P−', 'up'); p.appendChild(pl);
  cartButton(p, 'p', 1, 'cb.pdn', 'P+', 'down');
  const r = $('vcol-r'), rl = r.firstElementChild;
  cartButton(r, 'r', 1, 'cb.rr', 'R+', 'up'); r.appendChild(rl);
  cartButton(r, 'r', -1, 'cb.rl', 'R−', 'down');

  const seg = $('step-seg');
  CFG.cart.steps.forEach((n, i) => {
    const b = document.createElement('button');
    b.dataset.i = i; b.textContent = n + ' mm'; b.setAttribute('role', 'radio');
    b.dataset.n = n; b.title = t('step.title', { n });
    i18nHooks.push(() => { b.title = t('step.title', { n }); });
    seg.appendChild(b);
  });
  seg.addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setStep(Number(b.dataset.i)); });

  const gr = $('grip-range');
  const snapGripRange = () => {
    const g = S.obs && Array.isArray(S.obs.positions) && S.obs.positions.length > GRIP
      ? Math.round(gripFrac(S.obs.positions[GRIP]) * 100) : 0;
    gr.value = String(g);
    gr.style.setProperty('--v', g + '%');
  };
  gr.addEventListener('pointerdown', (e) => {
    if (isLocked()) { e.preventDefault(); toast(lockReason(), 'err'); flashLock(); return; }
    if (leaseModalOrPending()) { e.preventDefault(); return; }
    if (S.leaseGeneration == null) { e.preventDefault(); promptTakeControl(); return; }
    C.gripGestureBlocked = false;
    C.gripDrag = true;
  });
  gr.addEventListener('keydown', (e) => {
    const isSliderKey = ['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Home', 'End', 'PageUp', 'PageDown'].includes(e.key);
    if (!isSliderKey) return;
    if (isLocked()) { e.preventDefault(); toast(lockReason(), 'err'); flashLock(); return; }
    if (leaseModalOrPending()) { e.preventDefault(); return; }
    if (S.leaseGeneration == null) {
      e.preventDefault();
      if (!e.repeat) promptTakeControl();
      return;
    }
    if (e.repeat && C.gripGestureBlocked) { e.preventDefault(); return; }
    if (!e.repeat) C.gripGestureBlocked = false;
  });
  for (const ev of ['pointerup', 'pointercancel', 'blur']) gr.addEventListener(ev, () => { C.gripDrag = false; });
  gr.addEventListener('input', () => {
    if (isLocked() || leaseModalOrPending() || S.leaseGeneration == null || C.gripGestureBlocked) {
      snapGripRange();
      if (!isLocked() && !leaseModalOrPending() && S.leaseGeneration == null && !C.gripGestureBlocked) promptTakeControl();
      return;
    }
    S.grip = Number(gr.value) / 100;
    S.gripCmdAt = Date.now();
    cartKick();
  });
})();
function setStep(i) {
  C.stepIdx = clamp(i, 0, CFG.cart.steps.length - 1);
  store('rebot.step', String(C.stepIdx));
  for (const b of $('step-seg').children) {
    const on = Number(b.dataset.i) === C.stepIdx;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
}
new Pad($('pad-cxy'), { onChange: (v) => { C.axes.xy = v; }, onEnd: cartHoldStill });
new Pad($('pad-cz'), { axes: 'y', onChange: (v) => { C.axes.z = v[1]; }, onEnd: cartHoldStill });
new Pad($('pad-cp'), { axes: 'y', onChange: (v) => { C.axes.p = v[1]; }, onEnd: cartHoldStill });
// Pose state machine: S.poseMode = S.poseLocked || S.shiftHeld.  A mode flip zeroes every simple-cockpit axis
// and holds the left stick at zero until it is back near centre or re-touched, so the old stick
// position never carries over into the new mapping.
S.poseMode = false; S.poseLocked = false; S.shiftHeld = false;
let leftPadRequiresNeutral = false;
let padSxy = null;
for (const ev of ['touchstart', 'mousedown']) {
  $('pad-sxy').addEventListener(ev, () => { if (!padSxy.active()) leftPadRequiresNeutral = false; }); // fresh touch = deliberate
}
padSxy = new Pad($('pad-sxy'), { onChange: (v) => {
  if (leftPadRequiresNeutral) {
    if (Math.hypot(v[0], v[1]) >= CFG.deadzone) return; // still off-centre from before the flip
    leftPadRequiresNeutral = false;
  }
  if (!S.poseMode) C.axes.sxy = v;
  else { C.axes.sp = v[1]; C.axes.sr = v[0]; }
}, onEnd: cartHoldStill });
new Pad($('pad-sz'), { axes: 'y', onChange: (v) => { C.axes.sz = v[1]; } });
new Pad($('pad-sgrip'), {
  axes: 'y',
  onChange: (v) => {
    if (v[1] !== 0 && (leaseModalOrPending() || S.leaseGeneration == null)) {
      C.gripJog = 0;
      return;
    }
    C.gripJog = v[1]; // +1 is open, -1 is close
    if (v[1] !== 0) {
      S.gripCmdAt = Date.now();
      cartKick();
    }
  },
  onEnd: () => {
    C.gripJog = 0;
  }
});

// view frame look stick: up = look up (J4), right = turn right (J5); C.axes.sp / sr, same signs as the pose stick
let rotPadRequiresNeutral = false;
const padSrot = new Pad($('pad-srot'), { onChange: (v) => {
  if (rotPadRequiresNeutral) {
    if (Math.hypot(v[0], v[1]) >= CFG.deadzone) return;
    rotPadRequiresNeutral = false;
  }
  if (!viewActive()) return;
  C.axes.sp = v[1]; C.axes.sr = v[0];
}, onEnd: cartHoldStill });
for (const ev of ['touchstart', 'mousedown']) {
  $('pad-srot').addEventListener(ev, () => { if (!padSrot.active()) rotPadRequiresNeutral = false; });
}
// frame toggle (view = camera frame, base = base frame); expert mode on the 7-DOF arm only
function applyFrame() {
  const dof7 = S.dof === 7, view = viewActive();
  document.body.classList.toggle('cart-view', view);
  $('frame-seg-e').hidden = !dof7;
  $('frame-row-e').hidden = !dof7;
  for (const b of $('frame-seg-e').children) {
    const on = b.dataset.frame === (C.frame === 'view' ? 'view' : 'base');
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
  if (I18N_VIEW !== view) { I18N_VIEW = view; applyI18n(); }
  syncPoseMode(); renderPoseUi();
}
function setFrame(f) {
  f = f === 'base' ? 'base' : 'view';
  if (f === C.frame) return;
  releaseAll(); // drop keys, holds and sticks of the old mapping
  C.frame = f;
  store('rebot.cart.frame', f);
  onFrameChange();
}
// Effective frame changed (toggle, or a UI-mode switch with expert set to base): restart input mapping.
function onFrameChange() {
  C.axes.xy = [0, 0]; C.axes.z = 0; C.axes.p = 0; C.axes.sxy = [0, 0]; C.axes.sz = 0; C.axes.sp = 0; C.axes.sr = 0;
  leftPadRequiresNeutral = true; rotPadRequiresNeutral = true;
  resetPoseMode();
  if (S.obs && S.obs.positions.length >= 5) cartSync(S.obs.positions);
  C.status = 'idle'; C.msg = '';
  applyFrame();
}
$('frame-seg-e').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { buzz(8); setFrame(b.dataset.frame); } });

function jogTo(target) {
  if (!isLive()) return toast(lockReason(), 'err');
  return api('/api/v1/motion/jog', { target_positions: clampTarget(target), epoch_id: S.epoch }).catch(() => {});
}
function commandGrip() {
  if (leaseModalOrPending()) return;
  if (S.leaseGeneration == null) { promptTakeControl(); return; }
  if (gcOn()) { gcGrip(S.grip > 0.5 ? -1 : 1); return; }
  if (!S.obs || !canMove()) { flashLock(); return; }
  if (S.mode === 'cart') { S.gripCmdAt = Date.now(); cartKick(); return; } // the cartesian stream carries J6
  if (S.servoTarget) return; // while streaming, servo carries J6
  S.gripCmdAt = Date.now();
  const t = S.obs.positions.slice(0, S.dof);
  t[GRIP] = gripRad();
  jogTo(t);
}
function home() {
  if (leaseModalOrPending()) return;
  if (gcOn()) { gcSticksOff(); return; }
  if (!canMove()) { toast(lockReason(), 'err'); flashLock(); return; }
  api('/api/v1/motion/home', { epoch_id: S.epoch }).then(() => toast(t('toast.homing'), 'ok')).catch(() => {});
}
function stopMotion({ okKey = 'toast.stopped', okLevel = 'ok' } = {}) {
  const snap = { ws: S.ws, gen: S.leaseGeneration, epoch: S.epoch }, activeReq = hasActiveCartReq() ? C.cartReq : null;
  if (CWZ.controlSnap) CWZ.controlSnap.stopped = true;
  buzz([20]); invalidateControlIntents(); resetPoseMode();
  const stopIntent = ++motionIntentSeq;
  if (activeReq) { activeReq.stopPending = true; activeReq.stopIntent = stopIntent; }
  if (!snap.ws || snap.ws.readyState !== 1 || snap.gen == null || snap.epoch == null) { toast(t('toast.stop_no_control'), 'warn'); return; }
  const sendStop = (retries = 2) => {
    if (motionIntentSeq !== stopIntent) return;
    if (S.ws !== snap.ws || S.leaseGeneration !== snap.gen || S.epoch !== snap.epoch) { toast(t('toast.stop_no_control'), 'warn'); return; }
    api('/api/v1/motion/stop', {}, { quiet: true })
      .then(() => { if (motionIntentSeq === stopIntent) toast(t(okKey), okLevel); })
      .catch((e) => {
        if (motionIntentSeq !== stopIntent) return;
        if (S.ws !== snap.ws || S.leaseGeneration !== snap.gen || S.epoch !== snap.epoch
          || (e && (e.code === 'LEASE_REQUIRED' || e.code === 'LEASE_LOST' || isCancelledErr(e)))) { toast(t('toast.stop_no_control'), 'warn'); return; }
        if (retries > 0) setTimeout(() => sendStop(retries - 1), 100);
        else toast(t('toast.stop_unconfirmed'), 'err');
      });
  };
  sendStop();
}
function eStop() {
  buzz([30, 40, 30]);
  invalidateControlIntents();
  resetPoseMode();
  // Fast-path: WebSocket emergency frame
  if (S.ws && S.ws.readyState === WebSocket.OPEN) {
    try { S.ws.send(JSON.stringify({ type: 'estop' })); } catch (_) {}
  }
  // Reliable-path: REST emergency stop with automatic retry
  const sendRestEstop = (retries = 2) => {
    api('/api/v1/safety/e_stop')
      .then(() => {})
      .catch((e) => {
        if (retries > 0) setTimeout(() => sendRestEstop(retries - 1), 100);
        else toast(t('toast.estop_unconfirmed'), 'err');
      });
  };
  sendRestEstop();
  const started = Date.now();
  const confirmation = setInterval(() => {
    if (isLive() && S.state === 'EStop' && S.obsAt >= started) {
      clearInterval(confirmation); toast(t('toast.estop_confirmed'), 'err');
    } else if (Date.now() - started > 1800) {
      clearInterval(confirmation); toast(t('toast.estop_timeout'), 'err');
    }
  }, 50);
}
bindStopButton($('btn-stop'));
bindEstopButton($('btn-estop'));
$('btn-home').addEventListener('click', home);
/* a compact Stop and E-stop in the head of every drawer / sheet, shown while an open drawer covers the cockpit E-stop or it is off
   screen (phones: the bottom sheet; tablets and desktop: the side drawer over the right column). A touch screen has no
   Space / Escape key, so Stop and E-stop stay one tap away whatever is open. */
function estopReachable() {
  const real = $('btn-estop'), r = real.getBoundingClientRect();
  const x = r.left + r.width / 2, y = r.top + r.height / 2;
  if (!(r.width > 0 && r.height > 0) || x < 0 || y < 0 || x >= innerWidth || y >= innerHeight) return false;
  const el = document.elementFromPoint(x, y);
  return !!el && real.contains(el);
}
function estopPinSync() {
  const pins = document.querySelectorAll('.head-estop, .head-stop');
  for (const b of pins) b.hidden = true; // measure the cockpit E-stop without the pins
  const reach = estopReachable();
  for (const b of pins) b.hidden = reach || !!b.parentElement.closest('[hidden]');
}
(() => {
  for (const h of document.querySelectorAll('.calib-drawer > .calib-head')) {
    const b = document.createElement('button');
    b.type = 'button'; b.className = 'btn head-estop'; b.hidden = true;
    b.dataset.i18n = 'estop'; b.dataset.i18nAria = 'estop.aria'; b.textContent = t('estop'); b.setAttribute('aria-label', t('estop.aria'));
    bindEstopButton(b);
    const s = document.createElement('button');
    s.type = 'button'; s.className = 'btn head-stop'; s.hidden = true;
    s.dataset.i18n = 'stop'; s.dataset.i18nAria = 'stop.aria'; s.textContent = t('stop'); s.setAttribute('aria-label', t('stop.aria'));
    bindStopButton(s);
    const acts = h.querySelector('.cwiz-head-acts');
    if (acts) { acts.prepend(b); acts.prepend(s); } else { h.insertBefore(s, h.lastElementChild); h.insertBefore(b, h.lastElementChild); }
  }
  let timer = 0;
  const sync = () => { estopPinSync(); clearTimeout(timer); timer = setTimeout(estopPinSync, 260); }; // again after the sheet animation
  const mo = new MutationObserver(sync);
  for (const m of document.querySelectorAll('.calib-mask, .calib-drawer')) mo.observe(m, { attributes: true, attributeFilter: ['hidden', 'class'] });
  for (const ev of ['resize', 'scroll', 'orientationchange']) window.addEventListener(ev, sync, { passive: true });
  i18nHooks.push(sync);
})();
(() => {
  const roots = ['cwiz-body', 'mzero-body', 'gtool-body', 'teach-panel'].map((id) => $(id)).filter(Boolean);
  const mo = new MutationObserver((recs) => { for (const r of new Set(recs.map((x) => x.target.closest('.calib-drawer')))) if (r) tidyAll(r); });
  for (const r of roots) mo.observe(r, { childList: true, subtree: true, attributes: true, attributeFilter: ['hidden'] });
  window.addEventListener('resize', () => { for (const r of roots) tidyAll(r.closest('.calib-drawer') || r); }, { passive: true });
})();

/* fullscreen toggle (+ landscape lock on mobile) */
(() => {
  const btn = $('btn-fs');
  const root = document.documentElement;
  const req = root.requestFullscreen || root.webkitRequestFullscreen || root.mozRequestFullScreen || root.msRequestFullscreen;
  // Always show button
  btn.hidden = false;
  const fsEl = () => document.fullscreenElement || document.webkitFullscreenElement || null;
  const sync = () => {
    const isNativeFs = !!fsEl();
    const isHud = document.body.classList.contains('is-hud') || document.body.classList.contains('is-fs');
    const on = isNativeFs || isHud;
    btn.classList.toggle('on', on);
    btn.setAttribute('aria-pressed', String(on));
    const label = on ? t('fs.exit') : t('fs.hud');
    btn.setAttribute('aria-label', label); btn.title = label;
    if (isNativeFs) document.body.classList.add('is-fs');
    try {
      if (on) { const p = screen.orientation && screen.orientation.lock && screen.orientation.lock('landscape'); if (p && p.catch) p.catch(() => {}); }
      else if (screen.orientation && screen.orientation.unlock) screen.orientation.unlock();
    } catch (_) {}
  };
  btn.addEventListener('click', () => {
    const isCurrentActive = document.body.classList.contains('is-fs') || document.body.classList.contains('is-hud');
    if (isCurrentActive) {
      document.body.classList.remove('is-fs', 'is-hud');
      btn.classList.remove('on');
      try {
        if (fsEl()) {
          const p = (document.exitFullscreen || document.webkitExitFullscreen).call(document);
          if (p && p.catch) p.catch(() => {});
        }
      } catch (_) {}
    } else {
      document.body.classList.add('is-hud', 'is-fs');
      btn.classList.add('on');
      try {
        if (req) {
          const p = req.call(root);
          if (p && p.catch) p.catch(() => {});
        }
      } catch (_) {}
    }
  });
  i18nHooks.push(sync);
  document.addEventListener('fullscreenchange', sync);
  document.addEventListener('webkitfullscreenchange', sync);
  sync();
})();
$('fault-msg').addEventListener('click', () => { if (isStopped() || (!S.obs?.is_torqued && hasCarriedOverMotorErrors())) clearFault({ directLease: true }); });

/* speed */
(() => {
  const seg = $('speed-seg');
  CFG.speeds.forEach((sp, i) => {
    const b = document.createElement('button');
    b.dataset.i = i; b.textContent = Math.round(sp * 100) + '%';
    b.setAttribute('role', 'radio');
    seg.appendChild(b);
  });
  seg.addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setSpeed(Number(b.dataset.i)); });
  const pill = $('speed-pill');
  if (pill) {
    pill.addEventListener('click', () => {
      const nextIdx = (S.speedIdx + 1) % CFG.speeds.length;
      setSpeed(nextIdx);
    });
  }
})();
function setSpeed(i) {
  S.speedIdx = clamp(i, 0, CFG.speeds.length - 1);
  store('rebot.speed', String(S.speedIdx));
  const pill = $('speed-pill');
  if (pill) pill.textContent = Math.round(CFG.speeds[S.speedIdx] * 100) + '% ▾';
  const exSp = $('ex-speed-btn');
  if (exSp) exSp.textContent = t('ex.speed.btn', { p: Math.round(CFG.speeds[S.speedIdx] * 100) });
  for (const b of $('speed-seg').children) {
    const on = Number(b.dataset.i) === S.speedIdx;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
}

/* control mode */
// ctl: joint | cart (top segmented control); input: btn | stick (panel toggle, shared by both)
const PH_TITLE = {
  joint: () => '<b>J1–J6</b>' + t('ph.joint'),
  cart: () => '<b>X Y Z · Roll · Pitch</b>' + t('ph.cart'),
};
function setMode(ctl, input = S.input) {
  ctl = ctl === 'cart' ? 'cart' : 'joint';
  input = input === 'stick' ? 'stick' : 'btn';
  if (ctl !== S.ctl || input !== S.input) releaseAll();
  const enteringCart = ctl === 'cart' && S.mode !== 'cart';
  S.ctl = ctl; S.input = input;
  S.mode = ctl === 'cart' ? 'cart' : input === 'stick' ? 'stick' : 'jog';
  if (S.ui === 'expert') { store('rebot.ctl', ctl); store('rebot.input', input); }
  const body = document.body;
  body.classList.toggle('mode-cart', ctl === 'cart');
  body.classList.toggle('mode-stick', S.mode === 'stick');
  body.classList.toggle('input-stick', input === 'stick');
  $('ph-title').innerHTML = PH_TITLE[ctl]();
  for (const b of $('mode-seg').children) {
    const on = b.dataset.mode === ctl;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
  for (const b of $('input-seg').children) {
    const on = b.dataset.input === input;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
  if (enteringCart) {
    C.goal = null; C.status = 'idle'; C.msg = '';
    if (S.obs && S.obs.positions.length >= 5) cartSync(S.obs.positions);
  }
}
$('mode-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setMode(b.dataset.mode); });
$('input-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setMode(S.ctl, b.dataset.input); });
