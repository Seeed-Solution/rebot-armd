/* =================================================================
   Pose card: side view in the arm's own plane (URDF chain through VIK.frames, rotated by -J1),
   J1 yaw dial, J6 roll mark, gripper opening; commanded pose dashed when it differs from the
   measured one (same thresholds as the 3D ghost), profile rest pose as a faint outline, a joint
   named by observation.motion_limit (0-based) or observation.collision (1-based) in red.
   Redraws at most ~15 fps and only when an angle moved > 0.00175 rad or a drawn state changed;
   nothing while the page is hidden, the card is collapsed or not displayed.
   ================================================================= */
const PCARD = { cv: $('pose-canvas'), g: null, last: 0, key: '', win: null, q: null, c: null, draws: 0, hot: [], ghost: false, collapsed: store('rebot.poseCard') === '0' };
PCARD.g = PCARD.cv.getContext('2d');
const PCARD_EPS = 0.00175;
// planar points [x along the arm's heading, z] for motor angles q: base, J1..J6 origins, end_link; plus the tool axis
function posePlanar(q) {
  if (isSoArmProfile() && Array.isArray(q) && q.length >= 5) {
    const rot = (a, dx, dz) => [dx * Math.cos(a) + dz * Math.sin(a), -dx * Math.sin(a) + dz * Math.cos(a)];
    const p0 = [0, 0], p1 = [0.03884, 0.0624], p2 = [0.06923, 0.1166];
    const a1 = Number(q[1]) || 0, a2 = a1 + (Number(q[2]) || 0), a3 = a2 + (Number(q[3]) || 0);
    const u2 = rot(a1, 0.028, 0.11257), p3 = [p2[0] + u2[0], p2[1] + u2[1]];
    const u3 = rot(a2, 0.1349, 0.0052), p4 = [p3[0] + u3[0], p3[1] + u3[1]];
    const u4 = rot(a3, 0.0611, 0.0), p5 = [p4[0] + u4[0], p4[1] + u4[1]];
    const p6 = p5;
    const u7 = rot(a3, 0.10352, -0.00573), p7 = [p5[0] + u7[0], p5[1] + u7[1]];
    return { pts: [p0, p1, p2, p3, p4, p5, p6, p7], ax: [Math.cos(a3), -Math.sin(a3)] };
  }
  if (S.armDof !== 6 || (S.profileId && S.profileId.startsWith('so_arm'))) {
    return { pts: [], ax: [1, 0] };
  }
  const qq = toModel6(q);
  const f = VIK.frames(qq), c = Math.cos(qq[0]), s = Math.sin(qq[0]);
  const pl = (p) => [p[0] * c + p[1] * s, p[2]];
  const ax = pl([f.R[0], f.R[3], f.R[6]]);
  return { pts: [[0, 0], ...f.o.map(pl), pl(f.p)], ax };
}
function poseSetCollapsed(on) {
  PCARD.collapsed = on; PCARD.key = '';
  $('pose-card').classList.toggle('collapsed', on);
  const b = $('pose-toggle');
  b.setAttribute('aria-expanded', String(!on));
  b.setAttribute('aria-label', t(on ? 'pc.expand' : 'pc.collapse'));
  store('rebot.poseCard', on ? '0' : '1');
}
function poseTick(ts) {
  if (PCARD.collapsed || document.hidden || ts - PCARD.last < 66) return;
  const o = S.obs, live = isLive();
  const q = o && Array.isArray(o.positions) ? o.positions : null;
  const rest = Array.isArray(S.rest) && S.rest.length >= S.dof ? S.rest : (Array.isArray(S.home) && S.home.length >= 6 ? S.home : (CFG.presets.park || restJoints()));
  const meas = q || rest;
  const cmd = o && Array.isArray(o.commanded_positions) ? o.commanded_positions : meas;
  const ghost = !!q && cmd.some((v, i) => Math.abs(v - (meas[i] ?? v)) > (i === GRIP ? 0.05 : 0.005));
  const hot = [];
  const ml = o && o.motion_limit;
  if (live && ml && Number.isInteger(ml.joint)) hot.push(ml.joint);
  const col = o && o.collision;
  if (live && col && col.active && Number.isInteger(col.joint) && !hot.includes(col.joint - 1)) hot.push(col.joint - 1);
  const gc = live && o && o.control_mode === 'gravity_comp';
  const cv = PCARD.cv;
  const key = [hot.join('.'), gc, ghost, live, cv.clientWidth, cv.clientHeight, devicePixelRatio, LANG, rest.join()].join('|');
  const moved = (a, b) => !b || a.length !== b.length || a.some((v, i) => Math.abs(v - b[i]) > PCARD_EPS);
  if (key === PCARD.key && !moved(meas, PCARD.q) && (!ghost || !moved(cmd, PCARD.c))) return;
  if (!cv.getClientRects().length || !cv.clientWidth) return; // not displayed
  PCARD.last = ts; PCARD.key = key; PCARD.q = meas.slice(); PCARD.c = cmd.slice(); PCARD.hot = hot; PCARD.ghost = ghost;
  PCARD.draws++;
  poseDraw(meas, ghost ? cmd : null, rest, hot, gc, live);
}
function poseDraw(meas, cmd, rest, hot, gc, live) {
  const cv = PCARD.cv, g = PCARD.g;
  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const w = Math.max(1, Math.round(cv.clientWidth * dpr)), h = Math.max(1, Math.round(cv.clientHeight * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, w, h);
  const u = dpr, font = (px, wt = 500) => `${wt} ${px * u}px ui-monospace, Menlo, monospace`;
  const M = posePlanar(meas), C = cmd ? posePlanar(cmd) : null, R = posePlanar(rest);
  if (!M.pts.length) return;
  // view window (m): the drawn poses plus a margin, at least 0.36 m tall; kept while the poses fit and
  // the window is less than 1.6x the needed height, so the scale does not change on every small move
  let x0 = -0.12, x1 = 0.12, z0 = 0, z1 = 0.1;
  for (const P of [M, C, R]) if (P) for (const [x, z] of P.pts) { x0 = Math.min(x0, x - 0.05); x1 = Math.max(x1, x + 0.05); z0 = Math.min(z0, z - 0.02); z1 = Math.max(z1, z + 0.05); }
  if (z1 - z0 < 0.36) z1 = z0 + 0.36;
  const W0 = PCARD.win;
  if (W0 && x0 >= W0[0] && x1 <= W0[1] && z0 >= W0[2] && z1 <= W0[3] && W0[3] - W0[2] < 1.6 * (z1 - z0)) [x0, x1, z0, z1] = W0;
  else PCARD.win = [x0 - 0.04, x1 + 0.04, z0, z1 + 0.04], [x0, x1, z0, z1] = PCARD.win;
  const dialR = Math.min(13 * u, h * 0.2), dialW = dialR * 2 + 12 * u;
  const pad = 4 * u, aw = w - dialW - pad * 2, ah = h - pad * 2;
  const k = Math.min(aw / (x1 - x0), ah / (z1 - z0));
  const ox = pad + (aw - (x1 - x0) * k) / 2 - x0 * k, oz = h - pad - (ah - (z1 - z0) * k) / 2 + z0 * k;
  const X = (p) => [ox + p[0] * k, oz - p[1] * k];
  const RED = '#dc2626', LINK = !live ? '#94a3b8' : gc ? '#b45309' : '#334155', JOINT = !live ? '#94a3b8' : gc ? '#d97706' : '#5B7D15';
  // ground
  g.strokeStyle = 'rgba(31,41,55,0.18)'; g.lineWidth = u;
  const [, gy] = X([0, 0]); g.beginPath(); g.moveTo(pad, gy); g.lineTo(w - dialW, gy); g.stroke();
  g.lineCap = 'round'; g.lineJoin = 'round';
  const path = (pts, a, b) => { g.beginPath(); pts.slice(a, b).forEach((p, i) => { const [x, y] = X(p); if (i) g.lineTo(x, y); else g.moveTo(x, y); }); };
  // rest pose: faint outline
  g.strokeStyle = 'rgba(100,116,139,0.22)'; g.lineWidth = 6 * u; path(R.pts, 1, 8); g.stroke();
  // measured
  const jw = Math.max(3, 4.5 * u);
  g.lineCap = 'butt'; g.strokeStyle = hot.includes(0) ? RED : LINK; g.lineWidth = jw * 1.8; path(M.pts, 0, 3); g.stroke(); g.lineCap = 'round'; // base column up to J2
  for (let i = 2; i < 7; i++) {
    g.strokeStyle = hot.includes(i - 1) ? RED : LINK; g.lineWidth = i >= 5 ? jw * 0.8 : jw;
    path(M.pts, i, i + 2); g.stroke();
  }
  // gripper jaws at end_link, along the tool axis, spread by the opening
  const end = M.pts[7];
  let ax = M.ax; const an = Math.hypot(ax[0], ax[1]);
  if (an < 0.2) { const d = [end[0] - M.pts[6][0], end[1] - M.pts[6][1]], dn = Math.hypot(...d) || 1; ax = [d[0] / dn, d[1] / dn]; } else ax = [ax[0] / an, ax[1] / an];
  const nrm = [-ax[1], ax[0]], open = meas.length > GRIP ? gripFrac(Number(meas[GRIP]) || 0) : 0, spread = 0.012 + 0.03 * open;
  g.strokeStyle = hot.includes(GRIP) ? RED : !live ? '#94a3b8' : '#d48806'; g.lineWidth = Math.max(2, 2.4 * u);
  for (const sg of [-1, 1]) {
    const a = [end[0] - ax[0] * 0.05 + nrm[0] * spread * sg, end[1] - ax[1] * 0.05 + nrm[1] * spread * sg], b = [end[0] + nrm[0] * spread * sg, end[1] + nrm[1] * spread * sg];
    const [axp, ayp] = X(a), [bxp, byp] = X(b), [cx, cy] = X([end[0] - ax[0] * 0.05, end[1] - ax[1] * 0.05]);
    g.beginPath(); g.moveTo(cx, cy); g.lineTo(axp, ayp); g.lineTo(bxp, byp); g.stroke();
  }
  // commanded: dashed, on top of the measured pose
  if (C) {
    g.strokeStyle = '#3b82f6'; g.lineWidth = Math.max(1.5, 2 * u); g.setLineDash([4 * u, 3 * u]);
    path(C.pts, 2, 8); g.stroke(); g.setLineDash([]);
  }
  // joints J2..J5 (J1 is the dial, J6 the roll mark)
  for (let i = 1; i <= 4; i++) {
    const [x, y] = X(M.pts[i + 1]), r = (i === 1 ? 4 : 3.4) * u, on = hot.includes(i);
    g.fillStyle = on ? RED : '#fff'; g.strokeStyle = on ? RED : JOINT; g.lineWidth = 1.6 * u;
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill(); g.stroke();
  }
  // J5/J6 roll mark: small dial on the gripper body, tick at the roll angle (0 = up)
  {
    const rollIdx = isSoArmProfile() ? 4 : 5;
    const m = [(M.pts[6][0] + end[0]) / 2, (M.pts[6][1] + end[1]) / 2], [x, y] = X(m), r = 4.5 * u, a = (Number(meas[rollIdx]) || 0) - Math.PI / 2;
    g.fillStyle = '#fff'; g.strokeStyle = hot.includes(rollIdx) ? RED : JOINT; g.lineWidth = 1.4 * u;
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill(); g.stroke();
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + Math.cos(a) * r, y + Math.sin(a) * r); g.stroke();
  }
  g.textBaseline = 'middle';
  // labels of highlighted joints
  // labels of highlighted joints: red text on a white pill, below-left of the joint
  g.font = font(10, 700); g.textAlign = 'left';
  for (const j of hot) {
    if (j < 1 || j > 6) continue;
    const p = j === 5 ? [(M.pts[6][0] + end[0]) / 2, (M.pts[6][1] + end[1]) / 2] : M.pts[j + 1], [x, y] = X(p);
    const txt = 'J' + (j + 1), tw = g.measureText(txt).width, bw = tw + 6 * u, bh = 13 * u;
    const bx = clamp(x - bw - 5 * u, 1, w - dialW - bw), by = clamp(y + 3 * u, 1, h - bh - 1);
    g.fillStyle = 'rgba(255,255,255,0.92)'; g.strokeStyle = RED; g.lineWidth = u;
    g.beginPath(); if (g.roundRect) g.roundRect(bx, by, bw, bh, 4 * u); else g.rect(bx, by, bw, bh); g.fill(); g.stroke();
    g.fillStyle = RED; g.fillText(txt, bx + 3 * u, by + bh / 2 + 0.5 * u);
  }
  // J1 yaw dial (top view, forward = up, positive = counter-clockwise)
  const dx = w - dialR - 6 * u, dy = dialR + 4 * u, q0 = Number(meas[0]) || 0, j1hot = hot.includes(0);
  g.strokeStyle = j1hot ? RED : 'rgba(100,116,139,0.55)'; g.lineWidth = 1.2 * u;
  g.beginPath(); g.arc(dx, dy, dialR, 0, Math.PI * 2); g.stroke();
  g.strokeStyle = j1hot ? RED : JOINT; g.lineWidth = 2.2 * u;
  g.beginPath(); g.moveTo(dx, dy); g.lineTo(dx + Math.cos(-Math.PI / 2 - q0) * dialR * 0.85, dy + Math.sin(-Math.PI / 2 - q0) * dialR * 0.85); g.stroke();
  g.font = font(10); g.textAlign = 'center'; g.fillStyle = j1hot ? RED : '#475569';
  g.fillText('J1 ' + (q0 >= 0 ? '+' : '−') + Math.abs(q0 * DEG).toFixed(0) + '°', dx - 2 * u, dy + dialR + 9 * u);
  // legend: commanded / gravity compensation
  g.textAlign = 'right'; g.font = font(10, 600);
  let ly = h - 7 * u;
  if (gc) { g.fillStyle = '#b45309'; g.fillText(t('gc.chip'), w - 4 * u, ly); ly -= 13 * u; }
  if (C) {
    g.fillStyle = '#2563eb'; const tw = g.measureText(t('pc.cmd')).width; g.fillText(t('pc.cmd'), w - 4 * u, ly);
    g.strokeStyle = '#3b82f6'; g.lineWidth = 2 * u; g.setLineDash([3 * u, 2 * u]);
    g.beginPath(); g.moveTo(w - 4 * u - tw - 16 * u, ly); g.lineTo(w - 4 * u - tw - 4 * u, ly); g.stroke(); g.setLineDash([]);
  }
  g.textAlign = 'left'; g.textBaseline = 'alphabetic';
}
$('pose-toggle').addEventListener('click', () => poseSetCollapsed(!PCARD.collapsed));
PCARD.cv.addEventListener('click', () => { if (!V3.open) v3Toggle(true); });
PCARD.cv.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); if (!V3.open) v3Toggle(true); } });
document.addEventListener('visibilitychange', () => { PCARD.key = ''; });
i18nHooks.push(() => { PCARD.key = ''; $('pose-toggle').setAttribute('aria-label', t(PCARD.collapsed ? 'pc.expand' : 'pc.collapse')); });
poseSetCollapsed(PCARD.collapsed);

let jtab = [];
function renderJtable() {
  const tbody = $('jtable-body');
  if (!tbody) return;
  tbody.innerHTML = '';
  const list = activeJoints();
  jtab = list.map((jd, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${jd.key}</td><td>--</td><td>--</td><td>--</td><td>--</td><td>--</td>`;
    tbody.appendChild(tr);
    return { tr, c: Array.from(tr.children) };
  });
}
renderJtable();

/* =================================================================
   Render loop
   ================================================================= */
// four ui-kit states: normal / warning / danger / disabled ('')
const STATE_BADGE = { Armed: 'is-ok', Moving: 'is-ok', Idle: '', Booting: 'is-warn', EStop: 'is-danger', Fault: 'is-danger', Offline: '' };
const STATE_TEXT = { EStop: 'st.EStop', Offline: 'st.Offline', Armed: 'st.Armed', Moving: 'st.Moving', Idle: 'st.Idle', Booting: 'st.Booting', Fault: 'st.Fault' };
function fmtTime(ms) { const s = Math.floor(ms / 1000); return String(Math.floor(s / 60)).padStart(2, '0') + ':' + String(s % 60).padStart(2, '0'); }
