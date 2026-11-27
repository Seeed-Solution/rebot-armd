/* =================================================================
   Dataset recording
   ================================================================= */
function defaultEpisodeId() {
  const d = new Date(), p = (n) => String(n).padStart(2, '0');
  return `teleop_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}
$('rec-id').value = defaultEpisodeId();
function renderRecLast() {
  const r = S.recLast;
  $('rec-last').textContent = !r ? '' : r === 'discarded' ? t('rec.discarded') : t('rec.saved', r);
}
function setRecording(on) {
  S.rec.active = on; S.rec.startedAt = on ? Date.now() : 0;
  document.body.classList.toggle('recording', on);
  $('btn-rec').textContent = on ? t('rec.stop') : t('rec.start');
  $('btn-discard').disabled = !on;
  $('rec-id').disabled = on;
  $('rec-state').textContent = on ? t('rec.recording') : t('rec.idle');
  $('rec-state').className = 'sc-status-badge' + (on ? ' is-danger' : '');
}
$('btn-rec').addEventListener('click', async () => {
  if (teachOn()) { teachStrip(true); teachRecToggle(); return; }
  if (!S.rec.active) {
    const id = $('rec-id').value.trim() || defaultEpisodeId();
    try { await api('/api/v1/dataset/record/start', { episode_id: id }); setRecording(true); toast(t('toast.rec_start') + id, 'ok'); } catch (_) {}
  } else {
    try {
      const r = await api('/api/v1/dataset/record/save');
      setRecording(false);
      S.recLast = { n: r.frame_count ?? '?', path: r.path ?? '' }; renderRecLast();
      toast(t('rec.saved_toast', { n: r.frame_count ?? '?' }), 'ok');
      $('rec-id').value = defaultEpisodeId();
      epLoad(true);
    } catch (_) {}
  }
});
$('btn-discard').addEventListener('click', async () => {
  if (teachOn()) { teachRecDiscard(); return; }
  try { await api('/api/v1/dataset/record/discard'); setRecording(false); S.recLast = 'discarded'; renderRecLast(); } catch (_) {}
});


/* =================================================================
   Saved episodes and replay (capability replay): GET /api/v1/dataset/episodes,
   DELETE /api/v1/dataset/episodes/{id}, action /api/v1/action/replay.
   ================================================================= */
const EP = { list: [], loaded: false, info: null, delArmed: null, delTimer: 0, speed: 0.5, actionId: null, toasted: null, playing: null };
function epOnCaps() {
  const on = !!(S.caps && S.caps.replay);
  $('ep').hidden = !on;
  if (on && !EP.loaded) epLoad(true);
}
function epFmtSize(b) { return b < 1048576 ? Math.max(1, Math.round(b / 1024)) + ' KB' : (b / 1048576).toFixed(1) + ' MB'; }
function epFmtDur(s) { return Number.isFinite(s) ? (s < 60 ? s.toFixed(1) + ' s' : fmtTime(s * 1000)) : '--'; }
async function epLoad(quiet) {
  if (!quiet) setText($('ep-status'), t('ep.loading'));
  try {
    const res = await fetch('/api/v1/dataset/episodes', { cache: 'no-store' });
    const d = await res.json();
    if (!res.ok) throw new Error(d && (d.message || d.error) || 'HTTP ' + res.status);
    EP.info = d; EP.list = Array.isArray(d.episodes) ? d.episodes : []; EP.loaded = true;
    epRender(d.enabled === false ? t('ep.off') : t('ep.count', { n: EP.list.length, size: epFmtSize(d.total_bytes), max: Math.round(d.max_bytes / 1048576), files: d.max_files }));
  } catch (e) { epRender(t('ep.err', { e: e.message })); }
}
function epRender(status, bad) {
  const ol = $('ep-list');
  ol.textContent = '';
  if (!EP.list.length) {
    const li = document.createElement('li'); li.className = 'ep-empty'; li.textContent = EP.loaded ? t('ep.empty') : '--'; ol.appendChild(li);
  }
  for (const e of EP.list) {
    const li = document.createElement('li');
    li.dataset.id = e.id;
    li.classList.toggle('playing', EP.playing === e.id);
    li.innerHTML = '<span class="en"><b></b><small></small></span><span class="ea"><button class="btn ep-go" type="button"></button><button class="btn ep-del" type="button" data-op="/api/v1/dataset/episodes"></button></span>';
    li.querySelector('b').textContent = e.id;
    const when = e.created_ms > 0 ? new Date(e.created_ms).toLocaleString(LANG === 'zh' ? 'zh-CN' : 'en-GB', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' }) : '';
    li.querySelector('small').textContent = e.error ? t('ep.unreadable') : [epFmtDur(e.duration_s), when].filter(Boolean).join(' · ');
    const go = li.querySelector('.ep-go');
    go.textContent = t('ep.play');
    go.disabled = !!e.error || !!EP.playing;
    const del = li.querySelector('.ep-del');
    const armed = EP.delArmed === e.id;
    del.textContent = armed ? t('ep.del.confirm') : t('ep.del');
    del.classList.toggle('armed', armed);
    ol.appendChild(li);
  }
  if (status !== undefined) { setText($('ep-status'), status); $('ep-status').classList.toggle('bad', !!bad); }
}
function epCodeText(e) {
  const k = 'ep.code.' + (e.code || '');
  return I18N.zh[k] !== undefined ? t(k, { m: e.message || '' }) : (e.message || e.code || t('err.rejected'));
}
async function epPlay(id) {
  if (!isLive() || !canMove()) { toast(isLive() && !isStopped() && !colHold() && leaseOther() == null ? t('ep.enable_first') : lockReason(), 'err'); flashLock(); return; }
  try {
    const d = await commandRPC('/api/v1/action/replay', { episode: id, speed: EP.speed, gripper: $('ep-grip').checked, tag: 'cockpit' });
    syncEpoch(d.epoch_id);
    EP.actionId = d.action_id; EP.playing = id;
    toast(t('ep.started', { id, x: EP.speed }), 'ok');
    epRender(t('ep.started', { id, x: EP.speed }));
  } catch (e) { if (isCancelledErr(e)) return; const m = epCodeText(e); toast(m, 'err', e.code || ''); epRender(m, true); }
}
async function epDelete(id) {
  if (EP.delArmed !== id) {
    EP.delArmed = id; clearTimeout(EP.delTimer);
    EP.delTimer = setTimeout(() => { EP.delArmed = null; epRender(); }, 3000);
    epRender(); return;
  }
  EP.delArmed = null; clearTimeout(EP.delTimer);
  try {
    await api('/api/v1/dataset/episodes/' + encodeURIComponent(id), undefined, { method: 'DELETE' });
    toast(t('ep.deleted', { id }), 'ok');
    await epLoad(true);
    setText($('ep-status'), t('ep.deleted', { id }));
  } catch (e) { epRender(epCodeText(e), true); }
}
function epOnObs(o) {
  const a = o.action;
  const run = a && a.kind === 'replay' && a.state === 'running' ? a : null;
  $('ep-play').hidden = !run;
  if (run) {
    const info = EP.list.find((e) => e.id === run.episode);
    const total = info ? info.duration_s : null;
    const phase = t('ep.phase.' + (run.phase || 'replay'));
    setText($('ep-play-name'), run.episode || '');
    setText($('ep-play-text'), `${phase} · ${(run.t_s ?? 0).toFixed(1)}${total != null ? ' / ' + total.toFixed(1) : ''} s`);
    $('ep-bar').style.width = (100 * clamp(run.progress || 0, 0, 1)).toFixed(1) + '%';
    if (EP.playing !== run.episode) { EP.playing = run.episode; EP.actionId = run.id; epRender(); }
    return;
  }
  // ended: report the result of the replay started here (or seen running) once
  for (const r of [a, o.previous_action]) {
    if (!r || r.kind !== 'replay' || r.state === 'running' || r.id !== EP.actionId || EP.toasted === r.id) continue;
    EP.toasted = r.id;
    if (r.state === 'done') { toast(t('ep.done', { id: r.episode }), 'ok'); epRender(t('ep.done', { id: r.episode })); }
    else { const m = t('ep.ended', { code: r.code || r.state, m: epCodeText({ code: r.code, message: r.message }) }); toast(m, r.state === 'failed' ? 'err' : 'warn', r.code || ''); epRender(m, true); }
  }
  if (EP.playing) { EP.playing = null; epRender(); }
}
$('ep-speed').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  EP.speed = Number(b.dataset.speed);
  for (const x of $('ep-speed').children) { const on = x === b; x.classList.toggle('on', on); x.setAttribute('aria-checked', String(on)); }
});
$('ep-refresh').addEventListener('click', () => epLoad(false));
bindStopButton($('ep-stop'));
$('ep-list').addEventListener('click', (e) => {
  const b = e.target.closest('button'), li = e.target.closest('li');
  if (!b || !li || !li.dataset.id) return;
  if (b.classList.contains('ep-go')) epPlay(li.dataset.id);
  else if (b.classList.contains('ep-del')) epDelete(li.dataset.id);
});
i18nHooks.push(() => { if (EP.loaded) epLoad(true); });

