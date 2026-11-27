function renderCollision(live, ts) {
  const c = live && S.obs ? S.obs.collision : null;
  if (c && $('btn-sset').hidden) $('btn-sset').hidden = false; // guard reported before the schema said so
  const escalated = live && S.state === 'Fault' && S.faultCode === 'COLLISION';
  const active = !!(c && c.active);
  const banner = $('col-banner');
  banner.hidden = !(active || escalated);
  if (escalated) {
    setText($('col-title'), '⚠️ ' + t('fc.COLLISION'));
    setText($('col-sub'), t('col.esc.sub', { todo: t('fc.COLLISION.do') }));
  } else if (active) {
    const r = Number.isFinite(c.residual_nm) ? Math.abs(c.residual_nm).toFixed(1) : '--';
    const vars = { j: c.joint ?? '?', r, s: Number.isFinite(c.resume_in_s) ? Math.max(0, c.resume_in_s).toFixed(1) : '' };
    setText($('col-title'), t(Number.isFinite(c.resume_in_s) ? 'col.active' : 'col.active_wait', vars));
    setText($('col-sub'), t('col.sub'));
  }
  $('col-clear').hidden = !escalated;
  if (escalated) setText($('col-clear'), t(opLocked() ? 'op.inline_fault' : 'en.clear_fault'));
  // residual torque bars at ~10 Hz (the observation arrives at up to 50 Hz)
  if (ts - COL.barAt < 100) return;
  COL.barAt = ts;
  const rt = live && S.obs ? S.obs.residual_torques : null;
  const thr = COL.thr || null;
  const box = $('rt-bars');
  const show = Array.isArray(rt) && rt.length >= 6 && !!thr && colAvail();
  box.hidden = !show;
  if (!show) return;
  if (box.childElementCount !== 7 || box.dataset.lang !== LANG) {
    box.dataset.lang = LANG;
    box.innerHTML = '<span class="rt-cap"></span>' + Array.from({ length: 6 }, (_, i) => `<span class="rt-bar"><i><b></b></i>J${i + 1}</span>`).join('');
    box.firstChild.textContent = t('rt.cap');
  }
  for (let i = 0; i < 6; i++) {
    const el = box.children[i + 1], v = rt[i], th = (c && c.active && c.joint === i + 1 && Number.isFinite(c.threshold_nm)) ? c.threshold_nm : thr[i];
    const frac = Number.isFinite(v) ? Math.abs(v) / th : 0;
    const cls = 'rt-bar' + (c && c.active && c.joint === i + 1 ? ' trip' : frac >= 0.5 ? ' warn' : '');
    if (el.className !== cls) el.className = cls;
    const w = Math.min(100, frac * 100).toFixed(0) + '%';
    const bar = el.firstChild.firstChild;
    if (bar.style.width !== w) bar.style.width = w;
    const tip = Number.isFinite(v) ? `J${i + 1} ${v.toFixed(2)} / ${th.toFixed(1)} N·m` : `J${i + 1} --`;
    if (el.title !== tip) el.title = tip;
  }
}

function setText(el, txt) { if (el.textContent !== txt) el.textContent = txt; }
function setCls(el, cls) { if (el.className !== cls) el.className = cls; }
const fmtDeg = (rad) => { const d = rad * DEG; return (d >= 0 ? '+' : '−') + Math.abs(d).toFixed(1); };
let lastUi = 0;
function frame(ts) {
  requestAnimationFrame(frame);
  const live = isLive();
  const q = S.obs ? S.obs.positions : [0, 0, 0, 0, 0, 0];
  const v = commandVector();
  const en = live && isOperational() && leaseOther() == null;
  const op = en && !colHold();
  const ctrl = pageHasControl();
  const torqued = !!(S.obs && S.obs.is_torqued);
  const act = v.map((x) => ctrl && x !== 0);
  if (S.mode === 'cart' && ctrl && C.sending) { for (let i = 0; i < 4; i++) act[i] = true; act[GRIP] = Date.now() - S.gripCmdAt < 400; }
  poseTick(ts);
  drawOverlay();
  graspCheckInput(v);
  if (ts - lastUi < 60) return; // DOM updates at ~16 Hz
  lastUi = ts;
  renderGrasp();

  const body = document.body;
  body.classList.toggle('offline', !live);
  body.classList.toggle('locked', !op);
  body.classList.toggle('stopped', live && isStopped());
  syncControlInputsUi(ctrl);
  const cartHeld = Object.values(C.hold).some((h) => h[0] || h[1]);
  if (!op && (S.keys.size || cartHeld || S.hold.some((h) => h[0] || h[1]) || pads.some((p) => p.active()))) releaseAll();
  setText($('lock-txt'), lockReason());
  renderGc();
  renderCollision(live, ts);

  const lb = $('link-badge');
  setText(lb, live ? `${S.hz} Hz · ${S.rtt == null ? '--' : Math.round(S.rtt)} ms` : t('no_telemetry'));
  setCls(lb, 'sc-status-badge ' + (!live ? '' : S.hz >= 45 ? 'is-ok' : 'is-warn'));
  const st = live ? S.state : 'Offline';
  const badge = $('state-badge');
  const enableUnknown = S.obs?.enable_state === 'Unknown';
  const hold = live && isOperational() && colHold();
  setText(badge, hold ? t('st.col_hold') : enableUnknown && torqued && (st === 'Armed' || st === 'Moving') ? t('st.unconfirmed') : STATE_TEXT[st] ? t(STATE_TEXT[st]) : String(st).toUpperCase());
  setCls(badge, 'sc-status-badge ' + (hold ? 'is-warn' : STATE_BADGE[st] ?? ''));

  // enable button morphs: hold-to-enable / tap-to-disable / tap-to-clear
  const fresh = obsFresh(), holder = leaseOther(), ls = $('lease-status');
  ls.hidden = !live && !S.connLost;
  const retryS = S.connLost ? reconnectInS() : null;
  $('lease-retry-txt').hidden = retryS == null;
  if (retryS != null) setText($('lease-retry-txt'), t('lease.retry_in', { s: retryS }));
  $('btn-reconnect-now').hidden = !S.connLost;
  if (S.connLost) {
    setCls(ls, 'lease-status is-lost'); setText($('lease-status-txt'), t('lease.conn_lost'));
  } else if (live) {
    const [cls, txt] = leaseOp && leaseOp.kind === 'release' ? ['is-checking', t('lease.releasing')]
      : leaseOp && leaseOp.kind === 'refresh' ? ['is-checking', t('lease.refreshing')]
      : !fresh ? ['is-checking', t('lease.checking')]
      : S.leaseGeneration != null ? ['is-own', t('lease.own') + (S.ui === 'simple' ? ' · ' + t(torqued ? 'lease.tq.on' : 'lease.tq.off') : '')]
      : holder != null ? ['is-other', t('lease.other', { who: leaseHolderDetail(holder) })]
      : S.leaseReleased != null ? ['is-released', t('lease.released.tag')]
      : ['is-readonly', t('lease.readonly')];
    setCls(ls, 'lease-status ' + cls); setText($('lease-status-txt'), txt);
  }
  if (typeof toolsRenderRelease === 'function' && !$('sset').hidden) toolsRenderRelease();
  const showReleased = fresh && !leaseOp && S.leaseGeneration == null && S.leaseReleased != null;
  $('lease-released').hidden = !showReleased;
  if (showReleased) {
    setText($('lease-released-txt'), holder != null ? t('lease.released.other', { who: leaseHolderDetail(holder), arm: connArmText(S.obs) }) : t(S.leaseReleased.reason === 'expired' ? 'lease.released.expired' : 'lease.released.reconnect', { arm: connArmText(S.obs) }));
    $('btn-retake').disabled = holder != null || !fresh || !!leaseOp;
  }

  const eb = $('btn-enable');
  const stopped = isStopped();
  const carriedErrs = live && !stopped && !torqued && hasCarriedOverMotorErrors();
  const armedReadonly = live && !stopped && isOperational() && S.leaseGeneration == null;
  eb.classList.toggle('on', en && S.leaseGeneration != null);
  eb.classList.toggle('clear', stopped || carriedErrs);
  eb.setAttribute('aria-checked', String(en && S.leaseGeneration != null));
  // After a fault is cleared the motors may still hold the arm (safety Idle, torque on): say so.
  const holdingIdle = live && !stopped && !en && S.state === 'Idle' && torqued;
  $('btn-park-off').hidden = !holdingIdle || holder != null;
  $('btn-park-off').disabled = !fresh || !!leaseOp || holder != null;
  const stoppedTorqueOff = live && stopped && torqued && holder == null;
  const tob = $('btn-torque-off');
  tob.hidden = !stoppedTorqueOff;
  tob.disabled = !stoppedTorqueOff || !fresh || !!leaseOp || torqueOffInflight;
  if (!stopped || !torqued) showTorqueOffInline('');
  const opClear = live && (stopped || carriedErrs) && opLocked() && clearNeedsOperator();
  $('op-inline').hidden = !opClear;
  if (opClear) setText($('op-inline'), t(S.state === 'EStop' ? 'op.inline' : carriedErrs ? 'op.inline_motor_err' : 'op.inline_fault'));
  setText($('enable-txt'), stopped ? (S.state === 'EStop' ? t('en.reset_estop_s') : t('en.clear_fault_s')) : carriedErrs ? t('en.clear_motor_err_s') : armedReadonly ? t('en.armed_readonly') : en ? (enableUnknown ? t('en.on_unconfirmed') : t('st.Armed')) : holdingIdle ? t('en.holding') : t('en.enable'));
  setText($('enable-sub'), stopped ? t('en.sub_reset') : carriedErrs ? t('en.sub_clear_motor_err') : armedReadonly ? (holder != null ? t('lease.other', { who: leaseHolderDetail(holder) }) : t('en.sub_readonly')) : en ? t('en.sub_off') : t('en.hold'));
  eb.disabled = !fresh || !!leaseOp || holder != null;
  $('btn-stop').disabled = false;
  $('btn-preempt').hidden = !fresh || !!leaseOp || holder == null;
  setText($('preempt-who'), holder == null ? '' : t('preempt.who', { who: leaseHolderDetail(holder) }));
  if (preemptModalOpen()) renderPreemptModal();
  if (takeModalOpen()) renderTakeModal();

  // jog rows
  rows.forEach((r, i) => {
    const L = S.limits[i];
    const p = q[i] ?? 0;
    if (i === GRIP) {
      const f = gripFrac(p);
      setText(r.val, live ? String(Math.round(f * 100)) : '--');
      r.fill.style.left = '0%'; r.fill.style.width = (f * 100).toFixed(1) + '%';
      setText($('grip-val'), live ? t('grip.openPct', { n: Math.round(f * 100) }) : '--%');
      r.neg.classList.toggle('at-limit', f > 0.98);
      r.pos.classList.toggle('at-limit', f < 0.02);
    } else {
      setText(r.val, live ? fmtDeg(p) : '--');
      if (L) {
        const span = L.max_position - L.min_position;
        const f = clamp((p - L.min_position) / span, 0, 1);
        const z = clamp((0 - L.min_position) / span, 0, 1);
        r.mark.style.left = (f * 100).toFixed(1) + '%';
        r.zero.style.left = (z * 100).toFixed(1) + '%';
        r.fill.style.left = (Math.min(f, z) * 100).toFixed(1) + '%';
        r.fill.style.width = (Math.abs(f - z) * 100).toFixed(1) + '%';
        const near = f < 0.05 || f > 0.95;
        r.row.classList.toggle('near', live && near);
        // which UI direction pushes toward each limit
        const sign = CFG.jogSign[i] ?? 1;
        const atMin = f < 0.03, atMax = f > 0.97;
        r.pos.classList.toggle('at-limit', live && (sign > 0 ? atMax : atMin));
        r.neg.classList.toggle('at-limit', live && (sign > 0 ? atMin : atMax));
      }
    }
    r.row.classList.toggle('active', act[i]);
  });
  for (const d of padDirs) d.classList.toggle('hot', ctrl && Math.sign(v[Number(d.dataset.j)]) === Number(d.dataset.s));
  if (S.mode === 'cart') renderCart(live, op, ctrl);

  // video HUD
  const cam = $('cam-chip');
  const simple = S.ui === 'simple';
  // simple mode: one camera chip, live + rate + latency (expert: separate #vstat chip)
  const camStat = V.mode === 'mjpeg' ? '' : ` · ${V.fps == null ? '--' : V.fps} fps · ${V.lat == null ? '--' : Math.round(V.lat)} ms`;
  if (V.playing) { setText(cam, simple ? t('v.live') + camStat : t('v.live') + ' · ' + (MODE_LABEL[V.mode] || '')); setCls(cam, 'sc-status-badge is-ok'); }
  else { setText(cam, t('v.chip.connecting')); setCls(cam, 'sc-status-badge is-warn'); }
  const vs = $('vstat');
  vs.hidden = !V.playing || (V.mode === 'mjpeg' && simple);
  if (V.playing) {
    const lat = V.lat == null ? '--' : Math.round(V.lat);
    setText(vs, V.mode === 'mjpeg' ? 'MJPEG' : `${V.fps == null ? '--' : V.fps} fps · ${lat} ms`);
  }
  const dets = S.detections;
  const detAge = S.detLastReceivedAt ? (performance.now() - S.detLastReceivedAt) : Infinity;
  const detActive = !!(dets && dets.length > 0 && detAge <= 400);
  $('det-chip').hidden = !detActive;
  if (detActive) setText($('det-chip'), t('det.n', { n: dets.length }));
  // simple: the recognition chip counts the objects; a hint above it until one is selected
  if (simple) {
    const vis = $('btn-vis');
    if (!vis.hidden) setText(vis, svChipText());
    const hint = $('s-objhint');
    const showHint = !!(DET.avail && DET.enabled && !GR.sel && !graspRunningAction() && !GR.pickGrip && !MS.state && !document.body.classList.contains('v3-open')
      && !document.body.classList.contains('vis-open') && S.detLastReceivedAt && performance.now() - S.detLastReceivedAt < 2000);
    hint.hidden = !showHint;
    if (showHint) setText(hint, (dets || []).some((d) => !d.aruco) && detAge <= 2000 ? t('sv.hint') : t('sv.hint0'));
    renderSVisMeta();
  } else {
    $('s-objhint').hidden = true;
    if (typeof renderExpertLive === 'function') renderExpertLive(live, dets, detAge);
  }

  // telemetry
  const vel = S.obs ? S.obs.velocities || [] : [];
  setText($('t-hz'), live ? S.hz + ' Hz' : '--');
  $('t-hz').className = !live ? 'bad' : S.hz >= 45 ? 'good' : 'warn';
  setText($('t-rtt'), S.rtt == null ? '--' : Math.round(S.rtt) + ' ms');
  $('t-rtt').className = S.rtt == null ? '' : S.rtt < 40 ? 'good' : S.rtt < 120 ? 'warn' : 'bad';
  setText($('t-torque'), S.obs?.enable_state === 'Enabled' ? t('tq.on') : S.obs?.enable_state === 'Disabled' ? t('tq.off') : t('tq.unknown'));
  $('t-torque').className = torqued ? 'good' : '';
  const hbAge = S.hb.at ? Date.now() - S.hb.at : null;
  const tl = $('t-lease');
  const [tlTxt, tlCls] = S.connLost ? [t('lease.t.lost'), 'bad']
    : !fresh ? [t('lease.t.checking'), '']
    : leaseOp && leaseOp.kind === 'release' ? [t('lease.releasing'), 'warn']
    : leaseOp && leaseOp.kind === 'refresh' ? [t('lease.refreshing'), 'warn']
    : S.leaseGeneration != null ? [hbAge == null ? t('lease.t.own') : `${S.hb.hz} Hz · ${hbAge} ms`, hbAge == null || hbAge < 300 ? 'good' : hbAge < 500 ? 'warn' : 'bad']
    : holder != null ? [leaseHolderDetail(holder), 'warn']
    : S.leaseReleased != null ? [t('lease.t.released'), 'warn']
    : [t('lease.t.readonly'), ''];
  setText(tl, tlTxt); tl.className = tlCls;
  const ages = S.obs?.feedback_age_ms || [];
  const feedbackFresh = live && ages.length === S.dof && ages.every((age) => Number.isFinite(age) && age <= 250);
  setText($('t-can'), !live ? t('no_telemetry') : feedbackFresh ? t('can.fresh') : t('can.stale'));
  $('t-can').className = feedbackFresh ? 'good' : 'bad';
  const cap = S.obs?.driver_capabilities || {};
  const freshJoint = (i) => live && Number.isFinite(S.obs?.feedback_age_ms?.[i]) && S.obs.feedback_age_ms[i] <= 250;
  const tq = S.obs && S.obs.torques ? S.obs.torques : [];
  const temps = S.obs && S.obs.temperatures ? S.obs.temperatures : [];
  if (!simple) {
    jtab.forEach((r, i) => {
      const p = q[i];
      const ok = live && p != null;
      setText(r.c[1], ok ? fmtDeg(p) : '--');
      setText(r.c[2], ok ? p.toFixed(3) : '--');
      setText(r.c[3], freshJoint(i) && cap.velocity_feedback && vel[i] != null ? (vel[i] * DEG).toFixed(0) : '--');
      let tqStr = '--';
      if (freshJoint(i)) {
        if (cap.torque_feedback && tq[i] != null) tqStr = tq[i].toFixed(2);
        else if (cap.load_feedback) {
          const ld = S.obs?.loads?.[i];
          if (ld != null && Number.isFinite(ld)) tqStr = Math.round(ld * 100) + '%';
        }
      }
      setText(r.c[4], tqStr);
      setText(r.c[5], freshJoint(i) && cap.temperature_feedback && temps[i] != null ? temps[i].toFixed(0) : '--');
      r.tr.classList.toggle('hot', act[i]);
    });
  }
  const validTemps = cap.temperature_feedback ? temps.filter((v, i) => freshJoint(i) && Number.isFinite(v)) : [];
  const tmax = validTemps.length ? Math.max(...validTemps) : null;
  setText($('t-temp'), tmax == null ? '--' : tmax.toFixed(1) + '°C');
  $('t-temp').className = tmax == null ? '' : tmax < 55 ? 'good' : tmax < 65 ? 'warn' : 'bad';
  setText($('epoch-txt'), 'epoch ' + (S.epoch ?? '--'));
  for (const sp of $('fsm').querySelectorAll('span[data-s]')) sp.classList.toggle('on', live && sp.dataset.s === S.state);
  const fm = $('fault-msg');
  const showFault = live && (isStopped() || carriedErrs);
  fm.classList.toggle('show', showFault);
  if (showFault) {
    if (isStopped()) {
      const ft = S.state === 'Fault' && faultText();
      setText(fm, S.state === 'EStop' ? t('fm.estop') : ft ? t('fm.coded', ft) : t('fm.fault', { d: S.fault || t('unknown') }));
    } else {
      setText(fm, t(opLocked() ? 'fm.motor_err_op' : 'fm.motor_err', { d: motorErrorSummary() }));
    }
  }

  if (S.rec.active) {
    const el = Date.now() - S.rec.startedAt;
    setText($('rec-time'), fmtTime(el));
    setText($('rec-frames'), '~' + Math.round(el / 1000 * 50));
    setText($('rec-chip'), t('rec.chip') + fmtTime(el));
  }
  if (simple) renderSimple(live, op, ctrl);
}

function renderSimple(live, op, ctrl) {
  const has = live && S.obs.positions.length > GRIP;
  const f = has ? gripFrac(S.obs.positions[GRIP]) : null;
  const travelling = Date.now() - S.gripCmdAt < 1500 && S.grip != null;
  const ref = travelling ? S.grip : f;
  const isOpen = ref != null && ref > 0.5;
  if ($('s-grip-txt')) setText($('s-grip-txt'), isOpen ? t('grip.close') : t('grip.open'));
  if ($('s-grip')) {
    $('s-grip').setAttribute('aria-label', isOpen ? t('grip.close_aria') : t('grip.open_aria'));
    $('s-grip').style.setProperty('--g', ((f ?? 0) * 100).toFixed(1));
  }
  const ico = isOpen ? 'close' : 'open';
  const ie = $('s-grip-ico');
  if (ie && ie.dataset.k !== ico) { ie.dataset.k = ico; ie.innerHTML = GRIP_ICON[ico]; }
  setText($('s-grip-state'), f == null ? '--' : f > 0.9 ? t('grip.is_open') : f < 0.1 ? t('grip.is_closed') : t('grip.half'));
  setText($('s-grip-pct'), f == null ? '--%' : Math.round(f * 100) + '%');
  if ($('s-grip-bar')) $('s-grip-bar').style.width = ((f ?? 0) * 100).toFixed(1) + '%';

  // Dual readout in rate control jog: commanded % and live measured bar
  const cmdFrac = S.grip != null ? S.grip : f;
  setText($('s-grip-cmd-pct'), cmdFrac == null ? '--%' : Math.round(cmdFrac * 100) + '%');
  if ($('s-grip-jog-bar')) $('s-grip-jog-bar').style.width = ((f ?? 0) * 100).toFixed(1) + '%';

  const rd = $('s-ready');
  const readonlyOp = live && op && !ctrl;
  setCls(rd, 'ready' + (op && !readonlyOp ? ' ok' : live && (isStopped() || colHold() || hasCarriedOverMotorErrors()) ? ' bad' : ''));
  setText($('s-ready-txt'), !live ? t('wait_arm') : readonlyOp ? t(S.leaseReleased ? 'ready.released' : 'ready.readonly') : op ? t('ready') : lockReason());
  if (S.preset && Date.now() > S.preset.until) { S.preset = null; S.presetRunning = false; }
  for (const b of document.querySelectorAll('.preset')) b.classList.toggle('busy', !!S.preset && S.preset.name === b.dataset.preset);
  const bad = C.status !== 'ok' && C.status !== 'idle';
  const hint = $('s-hint');
  setText(hint, !live ? t('wait_arm') : !op ? lockReason() : bad ? ikText(C.msg) : readonlyOp ? t(S.leaseReleased ? 'ready.released' : 'ready.readonly') : '');
  hint.classList.toggle('bad', live && op && bad);
  hint.classList.toggle('lock', !(live && op && bad));
}

function renderCart(live, op, ctrl) {
  // readouts from daemon obs.tcp for measured state; local targets apply active_tool.tcp_offset_m once
  const view = viewActive();
  const pos6 = S.obs && S.obs.positions;
  const isSoArm = S.profileId && S.profileId.startsWith('so_arm');
  const isCoupledYaw = S.orientationDof === 2 || isSoArm;
  const hasObsTcp = S.obs && S.obs.tcp && Array.isArray(S.obs.tcp.xyz_m) && Array.isArray(S.obs.tcp.rpy_rad);
  let p = null;
  if ((isSoArm || !C.sending) && hasObsTcp) {
    const tcp = S.obs.tcp;
    p = { x: tcp.xyz_m[0], y: tcp.xyz_m[1], z: tcp.xyz_m[2], pitch: tcp.rpy_rad[1], yaw: tcp.rpy_rad[2] };
  } else {
    const T = view ? C.T : (C.cmd && C.q ? VIK.fk(toModel6([...toMotor(C.q), C.j5, (pos6 && pos6[5]) || 0])) : null);
    if (T) {
      const off = S.tcpOffset || [0, 0, 0];
      const px = T[0][3] + T[0][0] * off[0] + T[0][1] * off[1] + T[0][2] * off[2];
      const py = T[1][3] + T[1][0] * off[0] + T[1][1] * off[1] + T[1][2] * off[2];
      const pz = T[2][3] + T[2][0] * off[0] + T[2][1] * off[1] + T[2][2] * off[2];
      p = { x: px, y: py, z: pz, pitch: Math.asin(clamp(-T[2][0], -1, 1)), yaw: Math.atan2(T[1][0], T[0][0]) };
    } else if (hasObsTcp) {
      const tcp = S.obs.tcp;
      p = { x: tcp.xyz_m[0], y: tcp.xyz_m[1], z: tcp.xyz_m[2], pitch: tcp.rpy_rad[1], yaw: tcp.rpy_rad[2] };
    }
  }
  const mm = (m) => (m >= 0 ? '+' : '−') + Math.abs(m * 1000).toFixed(1);
  setText($('c-x'), p ? mm(p.x) : '--'); setText($('c-y'), p ? mm(p.y) : '--');
  setText($('c-z'), p ? mm(p.z) : '--'); setText($('c-p'), p ? fmtDeg(p.pitch) + '°' : '--');
  const R = p ? Math.hypot(p.x, p.y) : null, frac = R == null ? 0 : R / CFG.cart.reachMax;
  setText($('c-r'), R == null ? '--' : Math.round(R * 1000) + ' mm');
  $('c-rfill').style.width = (clamp(frac, 0, 1) * 100).toFixed(1) + '%';
  setCls($('c-reach'), 'reach' + (frac > 1 ? ' bad' : frac > 0.92 ? ' warn' : ''));
  setText($('c-r5'), !live ? '--' : isSoArm ? (pos6 && pos6.length > 4 ? fmtDeg(pos6[4]) + '°' : '--') : view ? (C.q6 ? fmtDeg(toMotor6(C.q6)[5]) + '°' : '--') : fmtDeg(C.j5) + '°');
  setText($('c-yaw'), p ? fmtDeg(p.yaw) + '°' + (isCoupledYaw ? ' (auto)' : '') : '--');
  if ($('c-yaw')) {
    $('c-yaw').title = isCoupledYaw ? (LANG === 'zh' ? '5自由度机械臂偏航角跟随径向方位角' : '5-DOF yaw follows radial azimuth') : '';
  }
  const bad = C.status !== 'ok' && C.status !== 'idle';
  const badge = $('ik-badge');
  setText(badge, !live ? 'IK --' : bad ? t('ik.blocked') : C.sending ? t('ik.stream', { hz: CFG.cart.hz }) : t('ik.ready'));
  setCls(badge, 'sc-status-badge ' + (!live ? '' : bad ? 'is-warn' : C.sending ? 'is-info' : 'is-ok'));
  const msg = $('ik-msg');
  setText(msg, !live ? t('wait_telemetry') : bad ? ikText(C.msg) : !op ? t('cart.enable_first') : !ctrl ? t(S.leaseReleased ? 'ready.released' : 'ready.readonly') : t('cart.following', { branch: view ? 'DLS' : (C.branch ?? '-'), tx: CFG.cart.tx === 'ws' ? t('cart.tx_ws') : t('cart.tx_rest') }));
  msg.classList.toggle('bad', bad);
  setText($('ik-us'), C.solveUs == null ? '' : 'IK ' + (C.solveUs < 1 ? C.solveUs.toFixed(2) : C.solveUs.toFixed(1)) + t('ik.per_solve'));
  const gr = $('grip-range');
  if (S.grip != null && !C.gripDrag) {
    const g = Math.round(S.grip * 100);
    if (Number(gr.value) !== g) gr.value = g;
  }
  gr.style.setProperty('--v', gr.value + '%');
  setText($('grip-pct'), live && S.obs.positions.length > GRIP ? Math.round(gripFrac(S.obs.positions[GRIP]) * 100) + '%' : '--%');
  const cv = cartVelocity();
  for (const d of cartDirs) d.classList.toggle('hot', ctrl && Math.sign(cv[d.dataset.ax]) === Number(d.dataset.s));
}

// Expose core detection & overlay symbols for testing and telemetry inspection
window.parseDetections = parseDetections;
window.__vision = { VM, CALVM, OV, OVSUB, CALSUB, calVisionMode, setVisionMode, vmToggle, vmBudget, vmRefresh, ovSet };
window.getVideoContentRect = getVideoContentRect;
window.fitCanvas = fitCanvas;
window.drawOverlay = drawOverlay;
window.S = S;
window.V = V;

/* =================================================================
   Device Detection & Responsive HUD Controller
   ================================================================= */
(() => {
  const ua = navigator.userAgent || '';
  const isTouch = ('ontouchstart' in window) || (navigator.maxTouchPoints > 0);
  // Comprehensive UA & client hint check for mobile phones, pads & tablets
  const isMobileUA = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini|Tablet|Mobile|Silk|Kindle|HarmonyOS|OpenHarmony/i.test(ua)
    || Boolean(navigator.userAgentData && navigator.userAgentData.mobile);
  const isIPad = /Macintosh/i.test(ua) && isTouch; // iPadOS reports as Macintosh with multi-touch
  const isTabletOrMobile = isMobileUA || isIPad || isTouch;

  if (isTabletOrMobile) {
    document.body.classList.add('is-mobile-device');
  }

  function updateOrientation() {
    const w = window.innerWidth;
    const h = window.innerHeight;
    const isLandscape = window.matchMedia('(orientation: landscape)').matches || (w > h);
    document.body.classList.toggle('is-landscape', isLandscape);

    // Any mobile/tablet device in landscape, or any display in landscape with height <= 1080px:
    // Automatically enters the immersive full-screen video + glass HUD cockpit!
    const forceHud = isLandscape && (isTabletOrMobile || h <= 1080);
    document.body.classList.toggle('is-hud', forceHud);
    
    // Sync button state if present
    const btn = $('btn-fs');
    if (btn) {
      btn.classList.toggle('on', forceHud || document.body.classList.contains('is-fs'));
    }
  }

  window.addEventListener('resize', updateOrientation);
  window.addEventListener('orientationchange', updateOrientation);
  // Execute immediately and once DOM is completely settled
  updateOrientation();
  window.addEventListener('DOMContentLoaded', updateOrientation);
})();

