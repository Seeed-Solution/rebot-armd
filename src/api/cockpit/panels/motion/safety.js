
/* clear E-stop / fault (shared by the enable button and the status bar) */
/* Clear fault (including MIT expiry recovery): there is no second-tap override when torque
   would drop a raised arm; the server enforces Rule A' and returns TORQUE_OFF_WOULD_DROP,
   which is shown inline with the same text as stopped torque-off. */
function clearFault(opts) {
  const mitLost = !!(S.obs && S.obs.fault && /MIT_LOST/.test(S.obs.fault.message || ''));
  const carriedMotorErr = !!(S.obs && !S.obs.fault && Array.isArray(S.obs.motor_errors) && S.obs.motor_errors.some(Boolean));
  const apiOpts = opts && opts.directLease ? { quiet: true, directLease: true } : { quiet: true };
  return api('/api/v1/safety/clear_fault', mitLost ? { mit_recovery: 'torque_off_all' } : undefined, apiOpts)
    .then((d) => {
      showTorqueOffInline('');
      toast(t(d && d.is_torqued ? 'toast.reset_holding' : (mitLost ? 'mit.recovery.done' : (carriedMotorErr ? 'toast.motor_err_cleared' : 'toast.reset_ok'))), 'ok');
    })
    .catch((e) => {
      if (e && (e.cancelled || e.code === 'CONTROL_CANCELLED')) return;
      if (e && e.code === 'TORQUE_OFF_WOULD_DROP') {
        showTorqueOffInline(formatTorqueOffWouldDrop(e));
        return;
      }
      toast(e && e.message ? e.message : String(e), 'err', e && e.code);
    });
}

/* Controlled stow: return to safe resting pose, wait until the arm settles,
   and safely cut power/torque. Never cuts torque while the arm is still moving. */
function formatParkWorkspaceBlocked(d) {
  const geom = (d && typeof d.geometry_id === 'string' && d.geometry_id)
    || (d && typeof d.constraint_id === 'string' && d.constraint_id)
    || '';
  if (!geom) return t('toast.ws_blocked_generic');
  const cid = (d && typeof d.constraint_id === 'string' && d.constraint_id && d.constraint_id !== geom)
    ? d.constraint_id : '';
  return cid ? t('toast.park_ws_blocked_cid', { geom, cid }) : t('toast.park_ws_blocked', { geom });
}
let unloading = false;
async function controlledUnload() {
  if (unloading) return;
  unloading = true;
  if (typeof toolsRenderRelease === 'function' && !$('sset').hidden) toolsRenderRelease();
  try {
    await api('/api/v1/motion/unload', undefined, { quiet: true });
    // Wait for the backend to finish parking and turn torque off, or for the pose to settle
    const t0 = Date.now();
    let last = null, stableSince = 0;
    while (Date.now() - t0 < 20000) {
      await new Promise((r) => setTimeout(r, 100));
      if (!S.obs || isStopped()) return;
      const o = S.obs;
      if (o.action?.kind === 'park' && o.action?.state === 'failed') break;
      const parkActive = !!o.disable_pending || (o.action?.kind === 'park' && o.action?.state === 'running');
      if (!o.is_torqued && !parkActive && o.enable_state !== 'Enabled'
        && o.action?.state !== 'running' && S.state !== 'Moving') break;
      const pos = o.positions;
      const moved = last ? Math.max(...pos.slice(0, S.dof).map((x, i) => Math.abs(x - last[i]))) : 1;
      last = pos.slice();
      if (!parkActive && moved < 0.005) {
        if (!stableSince) stableSince = Date.now();
        else if (!o.is_torqued || Date.now() - stableSince > 2000) break;
      } else {
        stableSince = 0;
      }
    }
    const act = S.obs?.action;
    if (S.obs && !S.obs.is_torqued) {
      toast(t('toast.parked'), 'ok');
    } else if (act?.kind === 'park' && act?.state === 'failed' && act?.code === 'WORKSPACE_VIOLATION') {
      toast(formatParkWorkspaceBlocked(act), 'err', 'WORKSPACE_VIOLATION');
    } else {
      toast(t('toast.unload_unconfirmed'), 'err');
    }
  } catch (e) {
    if (e && (e.cancelled || e.code === 'CONTROL_CANCELLED')) return;
    if (e && e.code === 'WORKSPACE_VIOLATION') {
      toast(formatParkWorkspaceBlocked(e.reply || e), 'err', 'WORKSPACE_VIOLATION');
    } else {
      toast(t('toast.stow_failed') + (e.message || t('toast.stow_unconfirmed')), 'err', e.code);
    }
  } finally {
    unloading = false;
    if (typeof toolsRenderRelease === 'function' && !$('sset').hidden) toolsRenderRelease();
  }
}

let torqueOffInflight = false;
function formatTorqueOffWouldDrop(e) {
  const r = e && e.reply;
  if (r && r.joint != null && typeof r.position === 'number' && typeof r.rest === 'number') {
    return t('en.torque_off.would_drop', {
      joint: r.joint,
      pos: r.position.toFixed(3),
      rest: r.rest.toFixed(3),
    });
  }
  if (r && r.reason === 'no_resting_pose') return t('en.torque_off.no_rest_pose');
  if (r && r.reason === 'stale_feedback') return t('en.torque_off.would_drop_stale');
  return (e && e.message) || t('en.torque_off.would_drop_stale');
}
function showTorqueOffInline(msg) {
  const tom = $('torque-off-msg');
  if (!tom) return;
  if (!msg) {
    if (!tom.hidden) { tom.hidden = true; setText(tom, ''); }
    return;
  }
  setText(tom, msg);
  tom.hidden = false;
}

/* Torque off. A 409 ArmNotAtRestingPose means the arm is in the air: don't cut power,
   stow it first. A 409 TORQUE_OFF_WOULD_DROP while stopped has no override: show inline. */
async function disableTorque({ fallback = true } = {}) {
  try {
    await api('/api/v1/torque', { enable: false }, { quiet: true });
    showTorqueOffInline('');
    toast(t(isStopped() ? 'toast.torque_off_stopped' : 'toast.parked_off'), 'ok');
    return true;
  } catch (e) {
    if (e && (e.cancelled || e.code === 'CONTROL_CANCELLED')) return false;
    if (e && e.code === 'TORQUE_OFF_WOULD_DROP') {
      showTorqueOffInline(formatTorqueOffWouldDrop(e));
      return false;
    }
    const notAtRest = e.status === 409 && (e.code === 'ARM_NOT_AT_RESTING_POSE' || e.code === 'ArmNotAtRestingPose' || /ArmNotAtRestingPose/i.test(e.message || ''));
    if (notAtRest && fallback) {
      toast(t('toast.in_air'), 'err');
      controlledUnload();
    } else if (e.status) {
      toast(notAtRest ? t('toast.not_at_rest') : (e.message || t('toast.torque_off_fail')), 'err', e.code);
    } else {
      toast(e.message || t('toast.torque_off_unconfirmed'), 'err', e.code);
    }
    return false;
  }
}

// Enable: hold 600 ms to arm. Disable: hold 600 ms to unarm. Clear fault: single tap.
(() => {
  const btn = $('btn-enable');
  let timer = null;
  let inflight = false;
  const cancel = () => { clearTimeout(timer); timer = null; btn.classList.remove('holding'); };
  holdCancels.push(cancel);
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (inflight || leaseModalOrPending()) return;
    if (!obsFresh()) return toast(lockReason(), 'err');
    if (leaseOther() != null) return toast(lockReason(), 'err');
    if (isStopped() || (!S.obs?.is_torqued && hasCarriedOverMotorErrors())) {
      // Single tap in stopped state or disarmed carried-over motor error: ONLY clears fault / motor error; never enables
      inflight = true;
      clearFault({ directLease: true }).finally(() => { setTimeout(() => { inflight = false; }, 300); });
      return;
    }

    // Both enabling and disabling require holding 600ms to prevent accidental taps dropping the arm
    btn.classList.add('holding');
    timer = setTimeout(() => {
      cancel(); buzz(20);
      if (leaseModalOrPending() || !obsFresh() || leaseOther() != null) return;
      inflight = true;
      // Re-evaluate current state at timeout to prevent stale transition race
      const targetEnable = !isOperational();
      if (!targetEnable) {
        if (S.leaseGeneration == null) {
          requireLeaseOrPrompt().catch((err) => { if (!isCancelledErr(err)) toast(err.message, 'err', err.code); });
        } else {
          releaseAll();
          toast(t('toast.stowing'), 'info');
          controlledUnload();
        }
      } else {
        if (S.obs?.positions?.length > GRIP && S.grip == null) S.grip = gripFrac(S.obs.positions[GRIP]);
        api('/api/v1/torque', { enable: true }, { quiet: true, directLease: true })
          .then(() => toast(t('toast.enabled'), 'ok'))
          .catch((err) => { if (!isCancelledErr(err)) toast(t('toast.enable_fail') + (err.message || t('toast.enable_fail_def')), 'err', err.code); });
      }
      setTimeout(() => { inflight = false; }, 300);
    }, 600);
  });
  for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) btn.addEventListener(ev, cancel);
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
})();
// Park and turn torque off from the hold after a clear (safety Idle, torque on): hold 600 ms. armd re-arms
// through its verified enable path, parks at the resting pose and turns torque off.
(() => {
  const btn = $('btn-park-off');
  let timer = null;
  const cancel = () => { clearTimeout(timer); timer = null; btn.classList.remove('holding'); };
  holdCancels.push(cancel);
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (unloading || leaseModalOrPending()) return;
    if (!obsFresh()) return toast(lockReason(), 'err');
    if (leaseOther() != null) return toast(lockReason(), 'err');
    btn.classList.add('holding');
    timer = setTimeout(() => {
      cancel(); buzz(20);
      if (leaseModalOrPending() || !obsFresh() || leaseOther() != null) return;
      if (S.leaseGeneration == null) {
        requireLeaseOrPrompt().catch((err) => { if (!isCancelledErr(err)) toast(err.message, 'err', err.code); });
        return;
      }
      releaseAll();
      toast(t('toast.stowing'), 'info');
      controlledUnload();
    }, 600);
  });
  for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) btn.addEventListener(ev, cancel);
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
})();
// Torque off while stopped (E-stop / Fault): enabled at rest pose only; server refuses elsewhere
// with TORQUE_OFF_WOULD_DROP and there is no override.
(() => {
  const btn = $('btn-torque-off');
  btn.addEventListener('click', () => {
    if (torqueOffInflight || !isLive() || !isStopped()) return;
    torqueOffInflight = true;
    btn.disabled = true;
    showTorqueOffInline('');
    disableTorque({ fallback: false }).finally(() => {
      torqueOffInflight = false;
      btn.disabled = !isLive() || !isStopped() || !S.obs?.is_torqued || leaseOther() != null;
    });
  });
})();

