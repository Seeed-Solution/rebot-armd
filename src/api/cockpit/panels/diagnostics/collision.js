/* =================================================================
   Collision guard (capability collision_guard, observation.collision, WS /api/v1/safety/collision):
   protective-stop banner, settings drawer section, residual torque bars (J1..J6).
   Everything stays hidden on an armd without the guard.
   ================================================================= */
const COL = { cfg: null, thr: null, rev: null, barAt: 0, busy: false };
const colAvail = () => (S.caps ? !!S.caps.collision_guard : !!(S.obs && S.obs.collision));
function colThresholds(c) {
  if (!c) return null;
  const e = c.effective || c;
  for (const k of ['effective_thresholds_nm', 'scaled_thresholds_nm', 'thresholds_nm']) {
    const v = e[k] ?? c[k];
    if (Array.isArray(v) && v.length >= 6 && v.slice(0, 6).every((x) => Number.isFinite(x) && x > 0)) return v.slice(0, 6);
  }
  return null;
}
function colOnCaps() {
  $('btn-sset').hidden = !colAvail() && !toolsAvail();
  $('col-set').hidden = !colAvail();
  if (colAvail() && !COL.cfg) colLoad(true);
  toolsOnCaps();
}
async function colLoad(quiet) {
  if (!quiet) colRenderSet(t('col.st.loading'));
  try {
    const res = await fetch('/api/v1/safety/collision', { cache: 'no-store' });
    const d = await res.json();
    if (!res.ok) throw new Error(d && (d.message || d.error) || 'HTTP ' + res.status);
    COL.cfg = d; COL.thr = colThresholds(d);
    if (typeof d.revision === 'number') COL.rev = d.revision;
    colRenderSet(quiet ? undefined : '');
  } catch (e) { if (!quiet) colRenderSet(t('col.st.err', { e: e.message })); }
}
function colRenderSet(status) {
  const c = COL.cfg, o = S.obs && S.obs.collision;
  const enabled = c ? !!c.enabled : !!o;
  const sens = (c && c.sensitivity) || (o && o.sensitivity) || 'medium';
  const en = $('col-en');
  if (!COL.busy) en.checked = enabled;
  en.disabled = COL.busy || !c;
  const sensOff = !(COL.busy ? en.checked : enabled);
  $('col-sens').classList.toggle('off', sensOff);
  $('col-sens').setAttribute('aria-disabled', String(sensOff));
  for (const b of $('col-sens').querySelectorAll('button')) {
    const on = b.dataset.v === sens;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
    b.disabled = COL.busy || !c || sensOff;
  }
  const st = $('col-status');
  const txt = status || (c ? (enabled ? t('col.st.on', { s: t('col.sens.' + sens) }) : t('col.st.off')) : '--');
  setText(st, txt);
  st.classList.toggle('ok', !!c && enabled);
}
async function colWrite(patch) {
  COL.busy = true; colRenderSet();
  try {
    const body = COL.rev != null ? { ...patch, expected_revision: COL.rev } : patch;
    const ack = await commandRPC('/api/v1/safety/collision', body);
    syncEpoch(ack.epoch_id);
    const cfg = ack.collision || ack.config || (('enabled' in ack) ? ack : null);
    if (cfg) { COL.cfg = { ...(COL.cfg || {}), ...cfg }; COL.thr = colThresholds(COL.cfg) || COL.thr; }
    else if (COL.cfg) Object.assign(COL.cfg, patch);
    const rev = ack.revision ?? (cfg && cfg.revision);
    if (typeof rev === 'number') COL.rev = rev;
    COL.busy = false;
    toast(t('col.st.saved'), 'ok');
    colRenderSet();
    colLoad(true);
  } catch (e) {
    COL.busy = false;
    if (isCancelledErr(e)) { colRenderSet(); return; }
    toast(e.message, 'err', e.code || '');
    colRenderSet(e.message);
    if (e.code === 'POLICY_REVISION_MISMATCH') colLoad(true);
  }
}
function ssetOpen(open) {
  $('sset').hidden = !open;
  estopPinSync();
  clearInterval(TL.live); TL.live = null;
  if (open) {
    if (colAvail()) { colRenderSet(); colLoad(false); }
    toolsRender(); toolsLoadStatus();
    TL.live = setInterval(toolsRenderRelease, 400);
    $('sset-close').focus({ preventScroll: true });
  } else $('btn-sset').focus({ preventScroll: true });
}
$('btn-sset').addEventListener('click', () => ssetOpen(true));
$('sset-close').addEventListener('click', () => ssetOpen(false));
$('sset').addEventListener('click', (e) => { if (e.target === e.currentTarget) ssetOpen(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('sset').hidden) ssetOpen(false); });
$('col-en').addEventListener('change', (e) => colWrite({ enabled: e.target.checked }));
$('col-sens').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b && !b.disabled && !b.classList.contains('on')) colWrite({ sensitivity: b.dataset.v });
});
$('col-clear').addEventListener('click', () => clearFault());
i18nHooks.push(() => { if (!$('sset').hidden) { colRenderSet(); toolsRender(); } COL.barAt = 0; renderCalStale(); if (!$('gtool').hidden) gtRender(); });

