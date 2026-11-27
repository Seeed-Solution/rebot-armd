/* =================================================================
   Configuration
   ================================================================= */
const QS = new URLSearchParams(location.search);
const CFG = {
  servoPeriodMs: 50,          // 20 Hz servo streaming while a button / stick / key is held
  heartbeatMs: 150,           // WS heartbeat, lease timeout on armd is 2500 ms
  // The lease is acquired on the first control input and released after this long without
  // one, only while torque is off (a release while torqued starts armd's watchdog park).
  // ?lease_idle_ms= shortens it for tests.
  leaseIdleMs: Math.max(500, Number(QS.get('lease_idle_ms')) || 30000),
  leaseAckTimeoutMs: 3000,    // no lease_granted/rejected for the current seq within this window -> onAcquireTimeout
  staleMs: 2000,              // telemetry older than this locks the controls (tolerance for Wi-Fi AP jitter)
  deadzone: 0.08,
  maxLead: 0.20,              // rad, servo target may lead measured position by at most this (safe margin under backend 0.35rad threshold)
  gripLead: 0.8,              // rad, gripper target lead; Damiao POS_VEL speed ~2.2/s x lead, so 0.8 rad ~ full 1.8 rad/s
  gripRate: 0.35,             // gripper stroke fraction per second at 100 % speed (≈2 rad/s motor limit over 5.7 rad)
  limitMargin: 0.02,          // rad, keep targets inside soft limits
  gripClosed: 0.0,            // J6 rad at 0 % open
  gripOpen: -5.7,             // J7 rad at 100 % open (measured mechanical stop -5.87; open is negative)
  speeds: [0.3, 0.6, 1.0],    // fraction of joint max velocity
  // UI direction (+1 = right-hand button / stick right or up) -> joint rad sign.
  // Matches the side-view pose preview. If a joint moves opposite to its label on the real arm,
  // flip it here or at runtime via ?sign=-1,1,-1,-1,1 (J1..J5).
  jogSign: [-1, 1, -1, -1, 1],
  joints: [
    { key: 'J1', part: 'jd.base.part', motion: 'jd.base.motion', en: 'Base Yaw', neg: 'jd.base.neg', pos: 'jd.base.pos', kn: 'A', kp: 'D' },
    { key: 'J2', part: 'jd.shoulder.part', motion: 'jd.shoulder.motion', en: 'Shoulder Pitch', neg: 'jd.shoulder.neg', pos: 'jd.shoulder.pos', kn: 'S', kp: 'W' },
    { key: 'J3', part: 'jd.elbow.part', motion: 'jd.elbow.motion', en: 'Elbow Pitch', neg: 'jd.elbow.neg', pos: 'jd.elbow.pos', kn: '↓', kp: '↑' },
    { key: 'J4', part: 'jd.wpitch.part', motion: 'jd.wpitch.motion', en: 'Wrist Pitch', neg: 'jd.wpitch.neg', pos: 'jd.wpitch.pos', kn: 'Q', kp: 'E' },
    { key: 'J5', part: 'jd.wroll.part', motion: 'jd.wroll.motion', en: 'Wrist Roll', neg: 'jd.wroll.neg', pos: 'jd.wroll.pos', kn: '←', kp: '→' },
    { key: 'J6', part: 'jd.grip.part', motion: 'jd.grip.motion', en: 'Gripper', neg: 'jd.grip.neg', pos: 'jd.grip.pos', kn: 'Z', kp: 'X' },
  ],
  video: {
    mjpeg: QS.get('video') || '',                           // explicit MJPEG URL overrides the ws stream
    // Camera app H.264-over-WebSocket (debug_stream, --stream-mode ws).
    ws: QS.get('ws') || `ws://${location.hostname}:${QS.get('wsport') || 8001}/`,
  },
  simpleSpeed: 0.3,           // simple mode: one fixed speed fraction (expert mode uses the 1-4 selector)
  // Quick presets, URDF model radians J1..J4 (mapped through CFG.cart.sign/off), J5 = 0.
  // inspect: TCP x 250 mm, z 300 mm, tool 60 deg below horizontal (IK solution).
  // park: the folded rest pose = the schema's home_positions (from the profile's zero_reference; joint
  //       angles, all 0 on the B601-DM), torque released once the arm arrives (same sequence armd runs
  //       on shutdown). Override with ?inspect=deg,deg,deg,deg / ?park=...
  presets: {
    inspect: [0, -1.167, -1.565, 1.435],
    park: null,
  },
};
for (const k of ['inspect', 'park']) {
  const a = (QS.get(k) || '').split(',').map(Number);
  if ((a.length === 4 || a.length === 6 || a.length === 7) && a.every(Number.isFinite)) {
    CFG.presets[k] = a.map((d) => d * Math.PI / 180);
  }
}
(() => {
  const s = QS.get('sign');
  if (!s) return;
  const a = s.split(',').map(Number);
  if (a.length >= 5 && a.every((x) => x === 1 || x === -1)) CFG.jogSign = a.slice(0, 5);
})();
let GRIP = 6;
const DEG = 180 / Math.PI;
const CLIENT_ID = 'web-' + Math.random().toString(36).slice(2, 8);
const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/* IK-BEGIN */
// Analytic IK for reBot B601-DM (J1 yaw, J2/J3/J4 pitch chain; J5 wrist swing held at 0;
// J6 wrist roll and J7 gripper are not part of the IK).
// Model angles follow the URDF (rebot-arm-workspace/assets/urdf_b601dm): metres, radians,
// base frame x forward / y left / z up. Lateral link offsets cancel to < 0.1 mm and are ignored.
// Pitch = tool axis angle below horizontal (0 = level, +pi/2 = pointing down).
const IK = (() => {
  const S0 = [0.0200, 0.1402];        // J2 axis (r, z) at J1 = 0
  const A = [-0.264, 0];              // J2 -> J3 at q = 0, (x, z)
  const B = [0.2426, 0.054];          // J3 -> J4
  const W = [0.2617, -0.0025];        // J4 -> TCP (end_link) with J5 = 0
  const LA = Math.hypot(...A), LB = Math.hypot(...B), LW = Math.hypot(...W);
  const DB = Math.atan2(B[1], B[0]), DW = Math.atan2(W[1], W[0]);
  const LIM = [[-Math.PI, Math.PI], [-2.5, 2.5], [-3.0, 3.0], [-Math.PI, Math.PI]]; // Kinematic bounds
  const TAU = 2 * Math.PI;
  // rotate (x, z) by phi about +y (positive phi tips +x downward)
  const rot = (v, p) => [v[0] * Math.cos(p) + v[1] * Math.sin(p), -v[0] * Math.sin(p) + v[1] * Math.cos(p)];
  const near = (a, ref) => a + TAU * Math.round((ref - a) / TAU);

  function fk(q) {
    const p2 = -q[1], p3 = p2 + q[2], p4 = p3 + q[3];
    const a = rot(A, p2), b = rot(B, p3), w = rot(W, p4);
    const r = S0[0] + a[0] + b[0] + w[0], z = S0[1] + a[1] + b[1] + w[1];
    return { x: r * Math.cos(q[0]), y: r * Math.sin(q[0]), z, pitch: p4 - DW };
  }
  // distance from the shoulder axis to the TCP (workspace sphere)
  function reach(p) {
    const r = Math.hypot(p.x, p.y);
    return Math.hypot(r - S0[0], p.z - S0[1]);
  }

  // opts: { prev: [q1..q4], branch, inLimits(q) -> bool, rMin, cosMax }
  // returns { ok, q, branch, err }  err in 'reach' | 'singular' | 'limit'
  function solve(p, opts) {
    const prev = opts.prev;
    const rMin = opts.rMin ?? 0.03, cosMax = opts.cosMax ?? 0.998;
    let yaw = Math.atan2(p.y, p.x), rr = Math.hypot(p.x, p.y);
    if (rr < rMin) { yaw = prev[0]; rr = p.x * Math.cos(yaw) + p.y * Math.sin(yaw); } // over the base axis
    const aw = -p.pitch;                              // CCW angle of the J4 -> TCP vector
    let best = null, err = 'reach';
    for (let k = 0; k < 2; k++) {                     // k = 1: reach over the back
      const q1 = near(k ? yaw + Math.PI : yaw, prev[0]);
      const r = k ? -rr : rr;
      const px = r - LW * Math.cos(aw) - S0[0], pz = p.z - LW * Math.sin(aw) - S0[1];
      const c = (px * px + pz * pz - LA * LA - LB * LB) / (2 * LA * LB);
      if (Math.abs(c) > 1) continue;
      if (Math.abs(c) > cosMax) { if (err === 'reach') err = 'singular'; continue; }
      for (const sg of [1, -1]) {
        const g = sg * Math.acos(c);
        const aa = Math.atan2(pz, px) - Math.atan2(LB * Math.sin(g), LA + LB * Math.cos(g));
        const ab = aa + g;
        const q2 = aa - Math.PI, p3 = DB - ab;
        const q = [q1, near(q2, prev[1]), 0, 0];
        q[2] = near(p3 + q[1], prev[2]);
        q[3] = near(DW - aw - (-q[1] + q[2]), prev[3]);
        let lim = true;
        if (opts.inLimits) {
          lim = opts.inLimits(q);
        } else {
          for (let i = 0; i < 4; i++) if (q[i] < LIM[i][0] || q[i] > LIM[i][1]) lim = false;
        }
        if (!lim) { err = 'limit'; continue; }
        let d = 0;
        for (let i = 0; i < 4; i++) d += (q[i] - prev[i]) ** 2;
        const branch = k * 2 + (sg > 0 ? 0 : 1);
        if (branch === opts.branch) d *= 0.8;         // hysteresis: prefer staying on the current branch
        if (!best || d < best.d) best = { q, branch, d };
      }
    }
    if (!best) return { ok: false, err };
    return { ok: true, q: best.q, branch: best.branch, dist: Math.sqrt(best.d) };
  }
  return { fk, solve, reach, LIM };
})();
/* IK-END */

CFG.cart = {
  hz: clamp(Number(QS.get('carthz')) || 40, 30, 50), // joint-target stream rate while moving
  vmax: 0.30,                 // m/s TCP speed at 100 % speed
  wmax: 1.5,                  // rad/s wrist pitch rate at 100 % speed
  steps: [1, 5, 10],          // tap step: mm for XYZ, degrees for pitch
  holdDelay: 350,             // ms before a held step button turns into continuous motion
  reachMax: (Number(QS.get('reach')) || 520) / 1000, // m, shoulder -> TCP sphere
  zMin: 0.0,                  // m, TCP may not go below the base mounting plane
  maxJump: 0.3,               // rad, a single IK update larger than this is treated as a branch flip
  tailMs: 300,                // keep streaming this long after the last change
  settleMs: 2000,             // ... and while the arm is still converging, up to this long
  tx: QS.get('tx') === 'rest' ? 'rest' : 'ws', // ws = cartesian_joint_stream frames, rest = /motion/servo
  // joint rad = sign * model rad + offset, J1..J6. Model = URDF joint angles. Set from the schema's
  // model_joint_map (armd: model = s * joint + o, so sign = s, off = -s * o); the defaults are the
  // B601-DM values (official convention: URDF angle = joint angle). J1..J4 can be overridden with
  // ?iksign=1,1,1,1&ikoff=0,0,0,0 (offset in degrees).
  sign: [1, 1, 1, 1, 1, 1],
  off: [0, 0, 0, 0, 0, 0],
};
function cartUrlMap() {
  const sg = (QS.get('iksign') || '').split(',').map(Number);
  if (sg.length === 4 && sg.every((x) => x === 1 || x === -1)) CFG.cart.sign.splice(0, 4, ...sg);
  const of = (QS.get('ikoff') || '').split(',').map(Number);
  if (of.length === 4 && of.every(Number.isFinite)) CFG.cart.off.splice(0, 4, ...of.map((d) => d / 180 * Math.PI));
}
cartUrlMap();
// camera-view cartesian frame (7-DOF arm only). camFlip / camRotDeg tune the camera mount on the real arm
// (?camflip=1 flips the image vertical, ?camrot=<deg> adds a roll about the optical axis).
CFG.view = {
  wmax: 0.8,                  // rad/s look-around / pitch rate at 100 % speed
  camFlip: QS.get('camflip') === '1',
  camRotDeg: Number(QS.get('camrot')) || 0,
  qEndCam: null, // camera_mount.q_end_cam from the schema (calibrated mount)
  levelCos: Math.cos(15 * Math.PI / 180), // no auto-level while the optical axis is within 15 deg of vertical
  wPos: 0.2,                  // m/rad, orientation weight in the IK error
  maxRot: 30 * Math.PI / 180, // anti-windup: goal orientation at most this far from the solved pose
  Rec: null,                  // R_end_cam, set once VIK is defined
};

/* =================================================================
   State
   ================================================================= */
const S = {
  obs: null, obsAt: 0, epoch: null,
  dof: 7, schemaLoaded: false,
  limits: [
    { min_position: -3.14159, max_position: 3.14159, max_velocity: 2.0 },
    { min_position: -2.0944, max_position: 2.0944, max_velocity: 2.0 },
    { min_position: -2.61799, max_position: 2.61799, max_velocity: 2.0 },
    { min_position: -3.14159, max_position: 3.14159, max_velocity: 3.0 },
    { min_position: -2.0944, max_position: 2.0944, max_velocity: 3.0 },
    { min_position: -6.28318, max_position: 6.28318, max_velocity: 4.0 },
    { min_position: -5.0, max_position: 5.0, max_velocity: 3.0 },
  ],
  ws: null, wsCount: 0, hz: 0,
  rtt: null, lastReqAt: 0,
  state: 'Offline', fault: '', faultCode: null,
  speedIdx: 1,
  clientId: CLIENT_ID,
  presetRunning: false,
  ctl: 'joint', input: 'btn',
  mode: 'jog',                // derived: jog | stick | cart
  axes: { l: [0, 0], r: [0, 0], wrist: 0 },
  hold: [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]], // per joint: [neg pressed count, pos pressed count]
  keys: new Set(),
  grip: null,                 // 0..1 gripper open fraction command
  gripCmdAt: 0,
  servoTarget: null, servoLastAt: 0, servoInflight: false,
  leaseGeneration: null,      // WS motion lease generation granted by armd (null = not held)
  wsObsSeen: false,           // true after at least one valid observation accepted on current S.ws
  obsWs: null,                // WebSocket instance that last accepted observation is bound to
  obsTs: -Infinity,           // last accepted observation timestamp
  minObsTs: -Infinity,        // post-release monotonic floor so pre-release WS frames cannot overwrite REST snapshot
  controlEpoch: 0,            // monotonic local control-intent epoch
  leaseReleased: null,        // null | { reason: 'reconnect' | 'expired' }
  connLost: false,            // a live socket dropped; true until a new socket delivers a valid observation
  lastIntentAt: 0,            // Date.now() of the last control input (idle lease release)
  preemptCooldownUntil: 0,    // Date.now() until which the preempt confirm button stays disabled
  sequenceId: 0,              // per-connection monotonic servo frame id
  rec: { active: false, startedAt: 0 },
  detections: [],
  detLastReceivedAt: 0,
  ui: 'simple',               // simple | expert
  uiUserSet: false,
  profileId: null,
  rest: null,
  hb: { n: 0, hz: 0, at: 0 }, // lease heartbeats sent: count this second, rate, last send time
  preset: null,               // { name, until } while a preset motion runs
  auth: { enforced: false, role: 'program', expiresIn: null },
};
const isSoArmProfile = () => !!(S.profileId && S.profileId.startsWith('so_arm'));
let schemaReady = null;
const speedScale = () => CFG.speeds[S.speedIdx];
// 5-DOF cartesian teleop state (model frame, metres / radians)
const C = {
  goal: null,                 // pose the operator asked for {x, y, z, pitch}
  cmd: null,                  // rate-limited pose that last solved; streamed as joint targets
  q: null, branch: null,      // IK solution (model J1..J4) for cmd
  j5: 0,                      // J5 (wrist swing on the 7-DOF arm), held while in this mode
  frame: store('rebot.cart.frame') === 'base' ? 'base' : 'view', // view (camera frame, 7-DOF only) | base
  T: null, Tg: null,          // view frame: solved / goal end_link pose (4x4, base frame)
  q6: null,                   // view frame: IK solution (model J1..J6) for T
  wq: [0, 0],                 // view frame: queued wrist look steps (model rad, J4 / J5) from step buttons
  hold: { x: [0, 0], y: [0, 0], z: [0, 0], p: [0, 0], r: [0, 0] }, // continuous holds per axis [neg, pos]
  axes: { xy: [0, 0], z: 0, p: 0, sxy: [0, 0], sz: 0, sp: 0, sr: 0 }, // s*: simple cockpit (sxy = X/Y, sz = lift, sp/sr = pitch/J5 wrist in pose mode)
  stepIdx: 1,
  lastTick: 0, lastInput: 0, tailUntil: 0, stallSince: 0,
  gripJog: 0,                 // -1 (close) .. +1 (open) continuous rate jog
  status: 'idle', msg: '', solveUs: null, restInflight: false, sending: false, gripDrag: false, gripGestureBlocked: false,
  cartHoldActive: false, cartActionId: null, cartActionUntil: 0, cartReq: null, stepActionId: null,
};

/* =================================================================
   Helpers
   ================================================================= */
/* Cause and next step for the codes armd and the page give for common refusals (errWhy). A toast with one of
   them shows it under its message; OPERATOR_REQUIRED adds an Unlock button (opts.path / opts.body name the
   refused request for the dialog's reason, opts.retry runs after the unlock: only for requests that do not move
   the arm). Numeric HTTP statuses are not mapped here: the call site knows what a 502 means. */
const EC_KEYS = {
  OPERATOR_REQUIRED: 'err.operator_required', LEASE_REQUIRED: 'err.nolease', NO_LEASE: 'err.nolease', LEASE_BUSY: 'ec.lease_busy',
  LEASE_LOST: 'ec.lease_lost', NOT_ARMED: 'ec.not_armed', RATE_LIMITED: 'ec.rate', INSECURE_TRANSPORT: 'ec.insecure',
  VISION_DOWN: 'ec.vision_down', VISION_TIMEOUT: 'ec.vision_timeout', MODEL_NOT_LOADED: 'vm.noyolo', COMMAND_TIMEOUT: 'err.cmdtimeout',
  VISION_BUSY: 'ec.vision_busy', ION_BUDGET: 'ec.ion_budget', MODEL_NOT_FOUND: 'ec.model_nf',
  MODEL_SWITCH_FAILED: 'ec.model_switch_failed', MODEL_SWITCH_REFUSED: 'ec.model_switch_refused', UNSUPPORTED: 'ec.unsupported',
};
const errWhy = (code) => (EC_KEYS[code] ? t(EC_KEYS[code]) : '');
function toast(msg, kind = '', code = '', opts = null) {
  const op = code === 'OPERATOR_REQUIRED';
  // the operator dialog is open and already says why
  if (op && typeof opIsOpen === 'function' && opIsOpen()) return null;
  const box = $('toasts');
  const msgText = String(msg);
  const codeText = String(code || '');
  const ms = opts && opts.ms ? opts.ms : op || (opts && opts.act) ? 9000 : kind === 'err' || kind === 'warn' ? 4200 : 2600;
  const dup = [...box.children].find((x) => x._toastMsg === msgText);
  if (dup) {
    const keep = (!codeText || dup._toastCode === codeText) && !(opts && (opts.act || opts.retry));
    clearTimeout(dup._toastTimer);
    if (keep) { dup._toastTimer = setTimeout(() => dup.remove(), ms); return dup; }
    dup.remove();
  }
  const el = document.createElement('div');
  el.className = 'toast ' + kind;
  el._toastMsg = msgText;
  el._toastCode = codeText;
  const why = errWhy(code);
  if (why && !msgText.includes(why)) {
    const tx = document.createElement('span'); tx.className = 'toast-txt';
    const a = document.createElement('span'); a.textContent = msg;
    const b = document.createElement('span'); b.className = 'toast-why'; b.textContent = why;
    tx.append(a, b); el.appendChild(tx);
  } else el.textContent = msg;
  if (code && !op) { const c = document.createElement('code'); c.textContent = code; el.appendChild(c); }
  if (opts && opts.act) {
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'btn btn-primary toast-act'; btn.textContent = opts.act.label;
    btn.addEventListener('click', () => { el.remove(); opts.act.fn(); });
    el.classList.add('has-act'); el.appendChild(btn);
  } else if (op && S.auth.enforced) {
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'btn btn-primary toast-act'; btn.textContent = t('op.unlock');
    btn.addEventListener('click', () => {
      el.remove();
      const o = opts || {};
      opRequire(o.path || '', o.body, { retry: !!o.retry }).then((ok) => { if (ok && o.retry) o.retry(); });
    });
    el.classList.add('has-act'); el.appendChild(btn);
  }
  box.appendChild(el);
  el._toastTimer = setTimeout(() => el.remove(), ms);
  while (box.children.length > 3) { clearTimeout(box.firstChild._toastTimer); box.firstChild.remove(); }
  return el;
}
function buzz(ms) { if (navigator.vibrate) try { navigator.vibrate(ms); } catch (_) {} }
function store(k, v) { try { if (v === undefined) return localStorage.getItem(k); localStorage.setItem(k, v); } catch (_) {} return null; }

const actuatorPaths = new Set(['/api/v1/mode/gravity_comp', '/api/v1/torque', '/api/v1/arm', '/api/v1/motion/home', '/api/v1/motion/jog', '/api/v1/motion/servo', '/api/v1/motion/unload', '/api/v1/motion/stop', '/api/v1/safety/clear_fault']);
const pendingCommands = new Map();
let nextRequestId = 0;
function rejectPendingCommands(message) {
  for (const pending of pendingCommands.values()) { clearTimeout(pending.timer); pending.reject(new Error(message)); }
  pendingCommands.clear();
}
function syncEpoch(epoch) {
  if (!Number.isSafeInteger(epoch) || epoch < 0) return;
  if (S.epoch != null && epoch < S.epoch) return;
  if (S.epoch != null && S.epoch !== epoch) {
    rejectPendingCommands(t('err.epoch'));
    invalidateControlIntents();
  }
  S.epoch = epoch;
}
function formatWorkspaceViolation(o) {
  if (!o || typeof o !== 'object' || !o.constraint_id) return '';
  const cid = o.constraint_id, gid = (typeof o.geometry_id === 'string' && o.geometry_id) || 'end_origin';
  const raw = String(o.reason || o.message || '');
  if (/^(?:invalid workspace check\b|.+sweep could not verify\b)/i.test(raw)) return '';
  if (/^box_(?:min|max)_[xyz]$/.test(cid)) {
    const partKey = gid === 'camera_mount' ? 'ws.part.camera' : gid.startsWith('capsule:') ? 'ws.part.arm' : 'ws.part.gripper';
    const distM = /\bis\s+(\d+(?:\.\d+)?)\s*(cm|mm)\b/i.exec(raw) || /(\d+(?:\.\d+)?)\s*(cm|mm)/i.exec(raw);
    const usePark = /\bpress Park\b/i.test(raw) || /would go .* deeper/i.test(raw);
    const args = { part: t(partKey), gid, edge: t('ws.edge.' + cid), dir: t('ws.dir.' + cid), cid, escape: t(usePark ? 'ws.escape.park' : 'ws.escape.home') };
    const wm = /box_(?:min|max)_[xyz]\s+(-?\d+(?:\.\d+)?)\s*m,\s*limit\s+(-?\d+(?:\.\d+)?)\s*m;\s*previous depth\s+(\d+(?:\.\d+)?\s*(?:m|mm)),\s*new depth\s+(\d+(?:\.\d+)?\s*(?:m|mm))(?:;\s*this move would go\s+(\d+(?:\.\d+)?\s*mm)\s+deeper)?/i.exec(raw);
    if (distM && wm) {
      const wArgs = { ...args, dist: distM[1] + ' ' + distM[2].toLowerCase(), val: wm[1], lim: wm[2], prev: wm[3], depth: wm[4] };
      return wm[5] ? t('ws.box_edge_witness_deep', { ...wArgs, delta: wm[5] }) : t('ws.box_edge_witness', wArgs);
    }
    if (distM) return t('ws.box_edge', { ...args, dist: distM[1] + ' ' + distM[2].toLowerCase() });
    return raw ? '' : t('ws.box_edge_nocm', args);
  }
  if (cid === 'sweep_budget') {
    const koM = /^path check near keep-out box '(.+)' needs more than (\d+) FK evaluations;\s*(.+)$/i.exec(raw);
    if (koM) {
      return t(/\bpress Park\b/i.test(koM[3]) ? 'ws.sweep_budget_keepout_park' : 'ws.sweep_budget_keepout', { name: koM[1], n: koM[2], gid });
    }
    const unM = /^return path could not be checked \(needs more than (\d+) FK evaluations\);.*?(?:\(near (box_(?:min|max)_[xyz]|max_reach|floor_z|self_collision:.+?) on (.+)\))?$/i.exec(raw);
    if (!unM) return '';
    const n = unM[1], blk = unM[2] || '', blkGid = unM[3] || gid;
    const jogKey = /^box_(?:min|max)_[xyz]$/.test(blk) ? 'ws.jog.' + blk : blk === 'max_reach' ? 'ws.jog.max_reach'
      : blk === 'floor_z' ? 'ws.jog.floor_z' : blk.startsWith('self_collision:') ? 'ws.jog.self_collision' : 'ws.jog.inward';
    const ctx = blk ? t('ws.sweep_budget_ctx', { blk, gid: blkGid }) : '';
    return t('ws.sweep_budget', { n, dir: t(jogKey), ctx });
  }
  if (cid === 'max_reach') {
    const rm = /reach\s+(\d+(?:\.\d+)?)\s*m\s*\(limit\s+(\d+(?:\.\d+)?)\s*m,\s*new depth\s+(\d+(?:\.\d+)?\s*(?:m|mm))\s+over,\s*previous depth\s+(\d+(?:\.\d+)?\s*(?:m|mm))(?:;\s*this move would go\s+(\d+(?:\.\d+)?\s*mm)\s+deeper)?\)/i.exec(raw);
    if (rm) {
      const rArgs = { gid, val: rm[1], lim: rm[2], depth: rm[3], prev: rm[4] };
      return rm[5] ? t('ws.max_reach_deep', { ...rArgs, delta: rm[5] }) : t('ws.max_reach', rArgs);
    }
  }
  return '';
}
function formatWsDist(depthM) {
  if (typeof depthM !== 'number' || !Number.isFinite(depthM) || depthM <= 0) return '';
  if (depthM >= 0.001) return (depthM * 100).toFixed(1) + ' cm';
  return Math.max(0.1, depthM * 1000).toFixed(1) + ' mm';
}
function wsEdgeAndDir(edgeId) {
  if (!edgeId || typeof edgeId !== 'string') {
    return { edge: t('ws.edge.unknown'), dir: t('ws.dir.unknown') };
  }
  if (edgeId.startsWith('keepout:')) {
    const name = edgeId.slice('keepout:'.length) || '?';
    return { edge: t('ws.edge.keepout', { name }), dir: t('ws.dir.keepout', { name }) };
  }
  const ek = 'ws.edge.' + edgeId, dk = 'ws.dir.' + edgeId;
  return {
    edge: t(ek) !== ek ? t(ek) : edgeId,
    dir: t(dk) !== dk ? t(dk) : t('ws.dir.unknown'),
  };
}
function formatGcWorkspaceBanner(gc) {
  if (!gc || !gc.workspace_outside) return '';
  const { edge } = wsEdgeAndDir(gc.workspace_edge);
  const dist = formatWsDist(gc.workspace_depth_m);
  return dist ? t('gc.ws.banner', { edge, dist }) : t('gc.ws.banner_nodist', { edge });
}
function formatGcExitBlocked(gc) {
  if (!gc || !gc.workspace_outside) return '';
  const { edge, dir: fallbackDir } = wsEdgeAndDir(gc.workspace_edge);
  const rk = gc.workspace_recovery_dir ? 'ws.rdir.' + gc.workspace_recovery_dir : '';
  const dir = rk && t(rk) !== rk ? t(rk) : fallbackDir;
  const dist = formatWsDist(gc.workspace_depth_m);
  return dist
    ? t('gc.ws.exit_blocked', { edge, dist, dir })
    : t('gc.ws.exit_blocked_nodist', { edge, dir });
}
function formatGcExitOutsideError(response) {
  if (!response || typeof response !== 'object') return t('err.gc_exit_outside_workspace');
  const raw = String(response.message || response.reason || '');
  if (response.workspace_unverified === true || /could not be verified/i.test(raw)) {
    return LANG === 'en' && raw ? raw : t('err.gc_exit_workspace_unverified');
  }
  if (LANG === 'en' && raw) return raw;
  const cid = response.workspace_edge || response.constraint_id || '';
  if (cid) {
    const { edge } = wsEdgeAndDir(cid);
    const dist = formatWsDist(response.workspace_depth_m);
    if (dist) return t('gc.ws.banner', { edge, dist });
    const distMatch = /(\d+(?:\.\d+)?)\s*(cm|mm)/i.exec(raw);
    return distMatch
      ? t('gc.ws.banner', { edge, dist: distMatch[1] + ' ' + distMatch[2].toLowerCase() })
      : t('gc.ws.banner_nodist', { edge });
  }
  const tailMatch = /\((box_(?:min|max)_[xyz]|max_reach|floor_z|keepout:.+?)\);/.exec(raw);
  if (tailMatch) {
    const { edge } = wsEdgeAndDir(tailMatch[1]);
    const distMatch = /(\d+(?:\.\d+)?)\s*(cm|mm)/i.exec(raw);
    return distMatch
      ? t('gc.ws.banner', { edge, dist: distMatch[1] + ' ' + distMatch[2].toLowerCase() })
      : t('gc.ws.banner_nodist', { edge });
  }
  return t('err.gc_exit_outside_workspace');
}
function rpcError(response) {
  let message;
  if (response.code === 'HANDOVER_SETTLING') {
    message = t('err.settling');
  } else if (response.code === 'UNSUPPORTED') {
    message = t('err.park_unsupported');
  } else if (response.code === 'COLLISION_HOLD') {
    message = t('err.collision_hold');
  } else if (response.code === 'OPERATOR_REQUIRED') {
    message = t('err.operator_required');
    opRefused();
  } else if (response.code === 'WORKSPACE_VIOLATION') {
    message = formatWorkspaceViolation(response) || response.reason || response.message || response.error || t('err.workspace');
  } else if (response.code === 'GC_EXIT_OUTSIDE_WORKSPACE') {
    message = formatGcExitOutsideError(response);
  } else {
    message = response.message || response.error || t('err.rejected');
  }
  return Object.assign(new Error(message), { code: response.code, status: response.status, reply: response });
}
/* A command refused with OPERATOR_REQUIRED opens the operator dialog with the reason for `path`. The few
   user-initiated, repeatable ones (opCanRetry) run once more after the unlock; every other one fails as before. */
let motionIntentSeq = 0;
const MOTION_START_PATHS = new Set(['/api/v1/motion/home', '/api/v1/motion/jog', '/api/v1/motion/servo', '/api/v1/motion/unload', '/api/v1/action/move_to', '/api/v1/action/move_to_pose', '/api/v1/action/move_relative', '/api/v1/action/run_sequence', '/api/v1/action/replay', '/api/v1/action/gripper', '/api/v1/action/gripper_zero_check', '/api/v1/action/gripper_calibrate', '/api/v1/action/measure_multiview', '/api/v1/action/calib_hover']);
const isMotionStartingCommand = (path, body) => MOTION_START_PATHS.has(path) || (path === '/api/v1/action/grasp' && !(body && body.dry_run === true)) || (path === '/api/v1/torque' && !!(body && body.enable === true)) || (path === '/api/v1/mode/gravity_comp' && !(body && body.enable === false));
function commandRPC(path, body, acquire = true, timeoutMs = 2000, opts = null) {
  const snap = { ws: S.ws, epoch: S.epoch, controlEpoch: S.controlEpoch, leaseGen: S.leaseGeneration, faultKey: path === '/api/v1/safety/clear_fault' ? mitFaultKey() : null };
  return commandRPCOnce(path, body, acquire, timeoutMs, opts).catch(async (e) => {
    if (path === '/api/v1/safety/clear_fault' && e?.code === 'CLEAR_FAULT_REJECTED' && /heartbeat/i.test(e.message || '')) {
      const retryGen = S.leaseGeneration;
      if (S.ws !== snap.ws || retryGen == null || (snap.leaseGen != null && retryGen !== snap.leaseGen)) throw cancelledError();
      sendHeartbeat(S.ws);
      await new Promise((r) => setTimeout(r, 60));
      if (S.ws !== snap.ws || S.epoch !== snap.epoch || S.controlEpoch !== snap.controlEpoch || S.leaseGeneration !== retryGen || (snap.faultKey != null && mitFaultKey() !== snap.faultKey)) throw cancelledError();
      return commandRPCOnce(path, body, false, timeoutMs, opts);
    }
    if (e.code !== 'OPERATOR_REQUIRED') throw e;
    const expectedGen = snap.leaseGen != null ? snap.leaseGen : S.leaseGeneration;
    const unlocked = opRequire(path, body);
    if (!opCanRetry(path, body)) throw e;
    return unlocked.then(async (ok) => {
      if (!ok) throw e;
      await wsOpen(5000);
      if ((opts && opts.controlSnap && !isControlSnapValid(opts.controlSnap))
        || S.ws !== snap.ws || S.epoch !== snap.epoch || S.controlEpoch !== snap.controlEpoch || S.leaseGeneration !== expectedGen) throw cancelledError();
      return commandRPCOnce(path, body, acquire, timeoutMs, opts);
    });
  });
}
function commandRPCOnce(path, body, acquire = true, timeoutMs = 2000, opts = null) {
  const isDryRunGrasp = path === '/api/v1/action/grasp' && !!(body && body.dry_run === true);
  if (isDryRunGrasp) {
    if (!S.ws || S.ws.readyState !== 1) return Promise.reject(Object.assign(new Error(t('err.disconnected')), { code: 'CONNECTION' }));
    const request_id = ++nextRequestId, ws = S.ws;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => { pendingCommands.delete(request_id); reject(Object.assign(new Error(t('err.cmdtimeout')), { code: 'COMMAND_TIMEOUT' })); }, timeoutMs);
      pendingCommands.set(request_id, { resolve, reject, timer, ws });
      const payload = { type: 'command', request_id, path, body: body || {}, client_id: CLIENT_ID };
      if (S.leaseGeneration != null) payload.generation = S.leaseGeneration;
      if (S.epoch != null) payload.epoch_id = S.epoch;
      try { ws.send(JSON.stringify(payload)); } catch (error) { clearTimeout(timer); pendingCommands.delete(request_id); reject(error); }
    });
  }
  if (opts && opts.controlSnap && !isControlSnapValid(opts.controlSnap)) return Promise.reject(cancelledError());
  if (path === '/api/v1/motion/stop') {
    if (!S.ws || S.ws.readyState !== 1 || S.leaseGeneration == null || S.epoch == null) return Promise.reject(Object.assign(new Error(t('err.nolease')), { code: 'LEASE_REQUIRED' }));
  } else if (acquire && S.leaseGeneration == null) {
    if (opts && opts.directLease) {
      const snap = { ws: S.ws, epoch: S.epoch, controlEpoch: S.controlEpoch, faultKey: path === '/api/v1/safety/clear_fault' ? mitFaultKey() : null };
      return acquireLeaseDirect().then(async () => {
        if (path === '/api/v1/safety/clear_fault' && S.faultCode === 'WATCHDOG_TIMEOUT') {
          await new Promise((r) => setTimeout(r, 40));
        }
        if (S.ws !== snap.ws || S.epoch !== snap.epoch || S.controlEpoch !== snap.controlEpoch || (snap.faultKey != null && mitFaultKey() !== snap.faultKey)) throw cancelledError();
        return commandRPCOnce(path, body, false, timeoutMs, opts);
      });
    }
    return requireLeaseOrPrompt();
  }
  if (!S.ws || S.ws.readyState !== 1 || S.leaseGeneration == null || S.epoch == null) {
    return Promise.reject(Object.assign(new Error(t('err.nolease')), { code: 'LEASE_REQUIRED' }));
  }
  const request_id = ++nextRequestId, ws = S.ws;
  if (path !== '/api/v1/motion/stop') S.lastIntentAt = Date.now();
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      pendingCommands.delete(request_id);
      reject(Object.assign(new Error(t('err.cmdtimeout')), { code: 'COMMAND_TIMEOUT' }));
    }, timeoutMs);
    pendingCommands.set(request_id, { resolve, reject, timer, ws });
    try { ws.send(JSON.stringify({ type: 'command', request_id, path, body: body || {}, generation: S.leaseGeneration, epoch_id: S.epoch, client_id: CLIENT_ID })); if (isMotionStartingCommand(path, body)) motionIntentSeq++; }
    catch (error) { clearTimeout(timer); pendingCommands.delete(request_id); reject(error); }
  });
}
async function api(path, body, { quiet = false, method = 'POST', directLease = false } = {}) {
  if (method === 'POST' && actuatorPaths.has(path)) {
    try { const data = await commandRPC(path, body, true, 2000, directLease ? { directLease: true } : null); syncEpoch(data.epoch_id); return data; }
    catch (error) { if (!quiet && !isCancelledErr(error)) toast(error.message, 'err', error.code); throw error; }
  }
  const t0 = performance.now(), reqWs = S.ws;
  S.lastReqAt = Date.now();
  let res;
  const controller = new AbortController();
  const requestTimeout = setTimeout(() => controller.abort(), path === '/api/v1/safety/e_stop' ? 700 : 4000);
  try {
    res = await fetch(path, {
      signal: controller.signal,
      method,
      headers: body !== undefined ? { 'Content-Type': 'application/json' } : undefined,
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
  } catch (e) {
    if (!quiet) toast(t('err.reqfail'), 'err', path);
    throw e;
  } finally { clearTimeout(requestTimeout); }
  const dt = performance.now() - t0;
  S.rtt = S.rtt == null ? dt : S.rtt * 0.8 + dt * 0.2;
  let data = null;
  try { data = await res.json(); } catch (_) {}
  if (!res.ok) {
    const err = new Error((data && (data.message || data.error)) || ('HTTP ' + res.status));
    err.code = (data && data.code) || String(res.status);
    err.status = res.status;
    if (err.code === 'OPERATOR_REQUIRED') { err.message = t('err.operator_required'); opRequire(path, body); }
    else if (!quiet) toast(err.message, 'err', err.code);
    throw err;
  }
  if (data && method !== 'GET' && S.ws === reqWs) syncEpoch(data.epoch_id);
  return data;
}

function parseState(s) {
  if (typeof s === 'string') return { name: s, detail: '' };
  if (s && typeof s === 'object') {
    const name = Object.keys(s)[0];
    if (name !== 'Fault') return { name, detail: '' };
    const r = s.Fault;
    if (typeof r === 'string') return { name, detail: r };
    const kind = Object.keys(r || {})[0] || 'Unknown';
    const v = r[kind];
    let detail = kind;
    if (kind === 'JointLimitExceeded' && v) detail = t('fault.limit', { name: v.joint_name, val: (v.value * DEG).toFixed(1) });
    else if (kind === 'ThermalTrip' && v) detail = t('fault.thermal', { j: v.joint_idx + 1, temp: Number(v.temperature).toFixed(1), limit: v.limit });
    else if (kind === 'Other') detail = String(v);
    return { name, detail };
  }
  return { name: 'Unknown', detail: '' };
}
const isLive = () => S.obs && Date.now() - S.obsAt < CFG.staleMs;
const obsFresh = () => !!(S.ws && S.ws.readyState === 1 && S.wsObsSeen && S.obsWs === S.ws && isLive());
const canSendStop = () => !!(S.ws && S.ws.readyState === 1 && S.leaseGeneration != null && S.epoch != null);
const isOperational = () => S.state === 'Armed' || S.state === 'Moving';
// collision guard protective stop (observation.collision.active): enabled, but motion commands are held
const colHold = () => !!(S.obs && S.obs.collision && S.obs.collision.active);
const canMove = () => isOperational() && !colHold();
const CONTROL_INPUT_SEL = '.disc, .pad, .preset, .ex-pre-btn, #btn-home, #btn-pose, #s-grip, #grip-range, .jbtn, .pose-go, #btn-gc';
const pageHasControl = () => !!(isLive() && isOperational() && !colHold() && S.leaseGeneration != null);
function syncControlInputsUi(active) {
  document.body.classList.toggle('controls-unavailable', !active);
  const state = active ? 'active' : 'unavailable';
  for (const el of document.querySelectorAll(CONTROL_INPUT_SEL)) {
    if (el.dataset.controlState !== state) el.dataset.controlState = state;
  }
}
const isStopped = () => S.state === 'EStop' || S.state === 'Fault';
const gripFrac = (rad) => clamp((rad - CFG.gripClosed) / (CFG.gripOpen - CFG.gripClosed), 0, 1);
const gripRad = () => CFG.gripClosed + (S.grip ?? 0) * (CFG.gripOpen - CFG.gripClosed);
// While rate-jogging, keep the commanded opening within gripLead of the real jaw so
// releasing stops promptly and a reversal acts at once (no wound-up target).
const gripNoWindup = (frac, measuredRad) => {
  const lead = CFG.gripLead / Math.abs(CFG.gripOpen - CFG.gripClosed), m = gripFrac(measuredRad);
  return clamp(frac, m - lead, m + lead);
};

