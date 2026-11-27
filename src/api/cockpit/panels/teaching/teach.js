/* =================================================================
   Named poses (capability poses): GET /api/v1/poses, WS /api/v1/poses/save|delete,
   action /api/v1/action/move_to_pose.
   ================================================================= */
const POSE_NAME_RE = /^[A-Za-z0-9_-]{1,64}$/;
const PS = { list: [], loaded: false, rev: null, delArmed: null, delTimer: 0 };
function posesOnCaps() { $('btn-pset').hidden = !(S.caps && S.caps.poses); }
function poseErr(e) {
  const k = 'pose.code.' + (e.code || '');
  return I18N.zh[k] !== undefined ? t(k, { m: e.message }) : e.message;
}
async function posesLoad(quiet) {
  if (!quiet) setText($('pose-status'), t('pose.loading'));
  try {
    const res = await fetch('/api/v1/poses', { cache: 'no-store' });
    const d = await res.json();
    if (!res.ok) throw new Error(d && (d.message || d.error) || 'HTTP ' + res.status);
    PS.list = Array.isArray(d.poses) ? d.poses : [];
    if (typeof d.revision === 'number') PS.rev = d.revision;
    PS.loaded = true;
    const nStale = PS.list.filter((p) => p.stale).length;
    posesRender(PS.list.length ? (nStale ? t('pose.stale.count', { n: PS.list.length, m: nStale }) : t('pose.count', { n: PS.list.length })) : '--');
  } catch (e) { posesRender(t('pose.err', { e: e.message })); }
}
function posesRender(status) {
  const ol = $('pose-list');
  ol.textContent = '';
  if (!PS.list.length) {
    const li = document.createElement('li'); li.className = 'pose-empty'; li.textContent = PS.loaded ? t('pose.empty') : '--'; ol.appendChild(li);
  }
  for (const p of PS.list) {
    const li = document.createElement('li');
    li.dataset.name = p.name;
    const tcp = p.tcp && Array.isArray(p.tcp.position) ? p.tcp.position.map((v) => Math.round(v * 1000)).join(', ') + ' mm' : '';
    const when = Number.isFinite(p.saved_at_ms) && p.saved_at_ms > 0 ? new Date(p.saved_at_ms).toLocaleString(LANG === 'zh' ? 'zh-CN' : 'en-GB', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' }) : '';
    li.innerHTML = '<span class="pn"><b></b><small></small></span><span class="pa"><button class="btn pose-go" type="button"></button><button class="btn pose-del" type="button" data-op="/api/v1/poses/delete"></button></span>';
    li.querySelector('b').textContent = p.name;
    const sm = li.querySelector('small');
    sm.textContent = p.stale ? t('pose.stale') : [tcp, when].filter(Boolean).join(' · ');
    sm.classList.toggle('stale', !!p.stale);
    if (p.stale) sm.title = p.stale.reason + ' (' + p.stale.at + ')';
    li.querySelector('.pose-go').textContent = t('pose.go');
    const del = li.querySelector('.pose-del');
    const armed = PS.delArmed === p.name;
    del.textContent = armed ? t('pose.del.confirm') : t('pose.del');
    del.classList.toggle('armed', armed);
    ol.appendChild(li);
  }
  if (status !== undefined) setText($('pose-status'), status);
  syncControlInputsUi(pageHasControl());
}
function poseNameCheck() {
  const v = $('pose-name').value.trim();
  const ok = POSE_NAME_RE.test(v);
  const rule = $('pose-rule');
  rule.classList.toggle('bad', v !== '' && !ok);
  setText(rule, v !== '' && !ok ? t('pose.bad') : t('pose.rule'));
  return ok ? v : null;
}
async function poseSave() {
  const name = poseNameCheck();
  if (!name) { $('pose-name').focus(); setText($('pose-rule'), t('pose.bad')); $('pose-rule').classList.add('bad'); return; }
  $('pose-save').disabled = true;
  try {
    const body = PS.rev != null ? { name, expected_revision: PS.rev } : { name };
    const ack = await commandRPC('/api/v1/poses/save', body);
    if (typeof ack.revision === 'number') PS.rev = ack.revision;
    toast(t('pose.saved', { name }), 'ok');
    $('pose-name').value = '';
    await posesLoad(true);
    posesRender(t('pose.saved', { name }));
  } catch (e) {
    if (isCancelledErr(e)) return;
    if (e.code === 'POLICY_REVISION_MISMATCH') posesLoad(true);
    const m = poseErr(e); toast(m, 'err', e.code || ''); posesRender(m);
  }
  finally { $('pose-save').disabled = false; }
}
async function poseDelete(name) {
  if (PS.delArmed !== name) {
    PS.delArmed = name; clearTimeout(PS.delTimer);
    PS.delTimer = setTimeout(() => { PS.delArmed = null; posesRender(); }, 3000);
    posesRender(); return;
  }
  PS.delArmed = null; clearTimeout(PS.delTimer);
  try {
    const body = PS.rev != null ? { name, expected_revision: PS.rev } : { name };
    const ack = await commandRPC('/api/v1/poses/delete', body);
    if (typeof ack.revision === 'number') PS.rev = ack.revision;
    toast(t('pose.deleted', { name }), 'ok');
    await posesLoad(true);
    posesRender(t('pose.deleted', { name }));
  } catch (e) {
    if (isCancelledErr(e)) return;
    if (e.code === 'POLICY_REVISION_MISMATCH') posesLoad(true);
    const m = poseErr(e); toast(m, 'err', e.code || ''); posesRender(m);
  }
}
async function poseGo(name) {
  if (!isLive() || !canMove()) { toast(isLive() && !isStopped() && !colHold() && leaseOther() == null ? t('pose.enable_first') : lockReason(), 'err'); flashLock(); return; }
  try {
    await commandRPC('/api/v1/action/move_to_pose', { name, speed_scale: clamp(speedScale(), 0.1, 1), tag: 'cockpit' });
    setText($('pose-status'), t('pose.moving', { name }));
    toast(t('pose.moving', { name }), 'ok');
  } catch (e) { if (isCancelledErr(e)) return; const m = poseErr(e); toast(m, 'err', e.code || ''); setText($('pose-status'), m); }
}
function psetOpen(open) {
  $('pset').hidden = !open;
  if (open) { PS.delArmed = null; posesRender(); posesLoad(false); $('pset-close').focus({ preventScroll: true }); }
  else $('btn-pset').focus({ preventScroll: true });
}
$('btn-pset').addEventListener('click', () => { if (!teachOn()) psetOpen(true); else if (TCH.open) teachOpen(false); else teachOpen(true, { tab: TCH.cur ? undefined : 'pose' }); });
$('pset-close').addEventListener('click', () => psetOpen(false));
$('pset').addEventListener('click', (e) => { if (e.target === e.currentTarget) psetOpen(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('pset').hidden) psetOpen(false); });
$('pose-name').addEventListener('input', poseNameCheck);
$('pose-name').addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); poseSave(); } });
$('pose-save').addEventListener('click', poseSave);
$('pose-list').addEventListener('click', (e) => {
  const b = e.target.closest('button'), li = e.target.closest('li');
  if (!b || !li || !li.dataset.name) return;
  if (b.classList.contains('pose-go')) poseGo(li.dataset.name);
  else if (b.classList.contains('pose-del')) poseDelete(li.dataset.name);
});
i18nHooks.push(() => { if (!$('pset').hidden) { posesRender(); poseNameCheck(); } });

/* =================================================================
   Teach panel (capabilities.teach.version >= 1): sequences / poses / trajectories (episodes).
   GET /api/v1/sequences, /api/v1/poses, /api/v1/episodes, /api/v1/datasets; WS (lease): sequences/save|delete,
   poses/save|delete, action run_sequence / replay / gripper; WS without a lease (recorder ownership): record/start|stop,
   episodes/update|delete, datasets/create. Server strings only through textContent (cwEl / setText).
   ================================================================= */
const TCH = {
  open: false, teach: false, tab: 'pose', loaded: false, // open: the panel; teach: the strip (teaching goes on with the panel closed)
  pick: [],             // pose ids ticked in the poses tab, in tick order ("play in ticked order")
  seqs: [], poses: [], eps: [], dsets: [], cur: null, dirty: false, sel: -1,
  ds: store('rebot.teach.ds') || 'default', task: store('rebot.teach.task') || '',
  repeat: 1, runRec: false, runDs: null, runTask: '',
  rec: null,            // { startedAt, dataset } while this page owns the recorder
  run: null,            // { actionId, seqId, n } of the sequence started here
  play: null,           // action id of a replay started here
  toasted: null, armed: null, armTimer: 0, edit: null, ask: null, busy: false,
  epSpeed: 0.5, epGrip: true, barKey: '',
  acked: new Map(),     // episode id -> { ep, at }: saved (record/stop ack) but not yet seen in a list fetched after the ack
};
const teachOn = () => !!(S.caps && S.caps.teach && Number(S.caps.teach.version) >= 1);
const TCH_SEQ_NAME = /^[A-Za-z0-9_\- \p{Script=Han}]{1,64}$/u;
// "play ticked poses": one sequence slot under this name, overwritten by every play and listed nowhere in the panel
// (armd's run_sequence takes the id of a saved sequence only)
const TCH_QUICK = '_quick';
const teachQuick = (s) => !!s && s.name === TCH_QUICK;
const teachSaved = () => TCH.seqs.filter((s) => !teachQuick(s));
const TCH_SPEEDS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
// commands that need no motion lease: sent on the control WebSocket without a generation (armd teach::is_lease_free).
// None of them moves the arm: a refusal for the operator session opens the dialog and the command runs once more after the unlock.
function teachCmd(path, body, timeoutMs = 5000) {
  return teachCmdOnce(path, body, timeoutMs).catch((e) => {
    if (e.code !== 'OPERATOR_REQUIRED') throw e;
    return opRequire(path, body, { retry: true }).then(async (ok) => {
      if (!ok) throw e;
      await wsOpen(5000);
      return teachCmdOnce(path, body, timeoutMs);
    });
  });
}
function teachCmdOnce(path, body, timeoutMs = 5000) {
  if (!S.ws || S.ws.readyState !== 1) return Promise.reject(Object.assign(new Error(t('gc.b.offline')), { code: 'CONNECTION' }));
  const request_id = ++nextRequestId, ws = S.ws;
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => { pendingCommands.delete(request_id); reject(Object.assign(new Error(t('err.cmdtimeout')), { code: 'COMMAND_TIMEOUT' })); }, timeoutMs);
    pendingCommands.set(request_id, { resolve, reject, timer, ws });
    try { ws.send(JSON.stringify({ type: 'command', request_id, path, body: body || {}, client_id: CLIENT_ID })); }
    catch (error) { clearTimeout(timer); pendingCommands.delete(request_id); reject(error); }
  });
}
// poses/save|rename|delete carry the pose store revision (armd requires it when operator auth is enforced)
async function teachPoseCmd(path, body) {
  if (PS.rev == null) { try { const d = await teachGet('/api/v1/poses'); if (typeof d.revision === 'number') PS.rev = d.revision; } catch (_) {} }
  const ack = await commandRPC(path, PS.rev != null ? { ...body, expected_revision: PS.rev } : body);
  if (ack && typeof ack.revision === 'number') PS.rev = ack.revision;
  return ack;
}
async function teachGet(path) {
  const res = await fetch(path, { cache: 'no-store' });
  let d = null; try { d = await res.json(); } catch (_) {}
  if (!res.ok) throw Object.assign(new Error((d && (d.message || d.error)) || 'HTTP ' + res.status), { code: (d && d.code) || String(res.status) });
  return d || {};
}
// error text: teach code, then the replay / pose texts, then armd's message
function teachErr(e) {
  const c = (e && e.code) || '', m = (e && e.message) || '';
  if (c === 'IN_USE') {
    const ids = (e.reply && Array.isArray(e.reply.sequences)) ? e.reply.sequences : [];
    return t('tch.e.IN_USE', { s: ids.map((id) => teachSeqName(id)).filter((n) => n !== TCH_QUICK).join(', ') || '?' });
  }
  if (c === 'SEQUENCE_REF_MISSING') { const st = e.reply && e.reply.step; return t('tch.e.SEQUENCE_REF_MISSING', { n: Number.isInteger(st) ? st + 1 : '?' }); }
  for (const k of ['tch.e.' + c, 'ep.code.' + c, 'pose.code.' + c]) if (I18N.zh[k] !== undefined) return t(k, { m });
  if (c === 'LEASE_ACTIVE' || c === 'LEASE_REQUIRED') return m;
  return m || c || t('err.rejected');
}
function teachStatus(msg, bad) { const el = $('teach-status'); setText(el, msg || '--'); el.classList.toggle('bad', !!bad); }
const teachSeqName = (id) => { const s = TCH.seqs.find((x) => x.id === id); return s ? s.name : id; };
const teachPose = (id) => TCH.poses.find((p) => p.id === id) || null;
const teachEp = (id) => TCH.eps.find((e) => e.id === id) || null;
const teachEpName = (e) => (e.task ? e.task : '#' + String(e.id).slice(-6));
function teachWhen(s) {
  const d = new Date(s); if (!(d.getTime() > 0)) return '';
  return d.toLocaleString(LANG === 'zh' ? 'zh-CN' : 'en-GB', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' });
}
async function teachLoad(quiet) {
  if (!quiet) teachStatus(t('tch.loading'));
  const t0 = Date.now();
  try {
    const [sq, ps, ep, ds] = await Promise.all([teachGet('/api/v1/sequences'), teachGet('/api/v1/poses'), teachGet('/api/v1/episodes?limit=500'), teachGet('/api/v1/datasets')]);
    TCH.seqs = Array.isArray(sq.sequences) ? sq.sequences : [];
    TCH.poses = Array.isArray(ps.poses) ? ps.poses : [];
    if (typeof ps.revision === 'number') PS.rev = ps.revision;
    TCH.eps = Array.isArray(ep.episodes) ? ep.episodes : [];
    // a just-saved episode stays listed until a list confirms it, or until a list requested more than 5 s after
    // its ack still lacks it (deleted elsewhere)
    for (const [id, k] of TCH.acked) {
      if (TCH.eps.some((x) => x.id === id) || t0 - k.at > 5000) TCH.acked.delete(id);
      else TCH.eps.unshift(k.ep);
    }
    TCH.dsets = Array.isArray(ds.datasets) ? ds.datasets : [];
    // the recorder this page owns is gone (owner lost, saved at the disk / length limit): only for a recording
    // started before this request, a newer one is not in the reply yet
    const r = ep.recording;
    if (TCH.rec && TCH.rec.startedAt < t0 && !(r && r.owner === CLIENT_ID)) teachRecSet(null);
    TCH.loaded = true;
    if (TCH.cur && TCH.cur.id && !TCH.dirty) { const s = TCH.seqs.find((x) => x.id === TCH.cur.id); if (s) TCH.cur = JSON.parse(JSON.stringify(s)); }
    if (!quiet) teachStatus(t('tch.counts', { s: teachSaved().length, p: TCH.poses.length, e: TCH.eps.length }));
  } catch (e) { teachStatus(t('tch.e.load', { e: teachErr(e) }), true); }
  teachRender();
}
// the 点位 button opens the poses drawer (#pset) on an armd without teach, else this panel on the poses tab
function teachOnCaps() {
  const on = teachOn(), b = $('btn-pset');
  if (on) b.hidden = false;
  b.dataset.i18nAria = b.dataset.i18nTitle = on ? 'tch.title' : 'pose.title';
  b.setAttribute('aria-label', t(b.dataset.i18nAria)); b.title = t(b.dataset.i18nTitle);
  b.setAttribute('aria-expanded', String(on && TCH.open));
  if (!on) { TCH.teach = false; TCH.pick = []; if (TCH.open) teachOpen(false); }
  teachBarRender();
}
// the panel (poses, motions, trajectories); opening it does not start the strip (前往 alone is not teaching)
function teachOpen(open, { tab } = {}) {
  TCH.open = !!open && teachOn();
  if (tab) TCH.tab = tab;
  $('teach').hidden = !TCH.open;
  if (TCH.open) { teachRender(); teachLoad(true); $('teach-close').focus({ preventScroll: true }); }
  $('btn-pset').setAttribute('aria-expanded', String(TCH.open));
  teachBarRender();
}
// the strip: started by hand-guiding, the hand-guide record button or 开始示教; stays with the panel closed; off ends
// teaching (not while this page records)
function teachStrip(on) {
  TCH.teach = !!on && teachOn();
  if (TCH.teach) TCH.gcHide = false;
  if (TCH.teach && !TCH.loaded) teachLoad(true);
  teachBarRender();
}
// the gripper opening now (0 closed .. 1 open), from the measured joint
function teachGripFrac() { const q = S.obs && Array.isArray(S.obs.positions) ? Number(S.obs.positions[GRIP]) : NaN; return Number.isFinite(q) ? Math.round(gripFrac(q) * 100) / 100 : 0.5; }
const teachDisarm = () => { TCH.armed = null; clearTimeout(TCH.armTimer); };
// two taps: the first arms the button for 3 s
function teachArm(key) {
  if (TCH.armed === key) { teachDisarm(); return true; }
  TCH.armed = key; clearTimeout(TCH.armTimer);
  TCH.armTimer = setTimeout(() => { TCH.armed = null; teachRender(); }, 3000);
  teachRender(); return false;
}
function tchBtn(label, fn, cls = '', disabled = false, op = '') { return cwBtn(label, fn, { cls, disabled, op }); }
function tchSelect(opts, value, onChange, aria) {
  const s = document.createElement('select');
  for (const [v, txt] of opts) { const o = document.createElement('option'); o.value = String(v); o.textContent = txt; s.appendChild(o); }
  s.value = String(value);
  if (aria) s.setAttribute('aria-label', aria);
  s.addEventListener('change', () => onChange(s.value));
  return s;
}
function tchInput(value, onInput, { type = 'text', aria = '', ph = '', min, max, step, cls = '' } = {}) {
  const i = document.createElement('input'); i.type = type; i.value = value == null ? '' : String(value);
  i.spellcheck = false; i.autocomplete = 'off'; if (cls) i.className = cls;
  if (aria) i.setAttribute('aria-label', aria); if (ph) i.placeholder = ph;
  if (min != null) i.min = String(min); if (max != null) i.max = String(max); if (step != null) i.step = String(step);
  i.addEventListener(type === 'number' ? 'change' : 'input', () => onInput(i.value, i));
  return i;
}
function tchBadge(text, cls = '') { return cwEl('span', 'tch-bd ' + cls, text); }
const tchSrc = (s) => (I18N.zh['tch.src.' + s] !== undefined ? t('tch.src.' + s) : s || '?');
function teachDsNames() {
  const n = new Set(['default', ...TCH.dsets.map((d) => d.name), ...TCH.eps.map((e) => e.dataset).filter(Boolean)]);
  if (TCH.ds) n.add(TCH.ds);
  return [...n];
}
// a sequence step references stale data (joint zero changed since taught / recorded)
function teachStepStale(st) {
  if (st.type === 'pose') { const p = teachPose(st.pose_id); return !!(p && p.stale); }
  if (st.type === 'replay') { const e = teachEp(st.episode_id); return !!(e && e.stale); }
  return false;
}
function teachStepMissing(st) {
  if (st.type === 'pose') return !teachPose(st.pose_id);
  if (st.type === 'replay') return !teachEp(st.episode_id);
  return false;
}
const teachSeqStale = (s) => (s.steps || []).some(teachStepStale);
function teachUsedBy(kind, id) {
  return teachSaved().filter((s) => (s.steps || []).some((st) => (kind === 'pose' ? st.type === 'pose' && st.pose_id === id : st.type === 'replay' && st.episode_id === id))).map((s) => s.name);
}

/* ---- render ---- */
function teachRender() {
  if (!TCH.open) return;
  for (const b of $('teach-tabs').children) {
    const on = b.dataset.tab === TCH.tab;
    b.classList.toggle('on', on); b.setAttribute('aria-selected', String(on)); b.tabIndex = on ? 0 : -1;
  }
  $('teach-body').setAttribute('aria-labelledby', 'teach-tab-' + TCH.tab);
  teachAskRender();
  const body = $('teach-body');
  const keep = body.scrollTop;
  body.textContent = '';
  if (TCH.tab === 'seq') { if (TCH.cur) teachRenderEditor(body); else teachRenderSeqs(body); }
  else if (TCH.tab === 'pose') teachRenderPoses(body);
  else teachRenderEps(body);
  body.scrollTop = keep;
  teachLive();
  teachBarRender();
}
function teachRenderSeqs(body) {
  const nw = cwEl('div', 'tch-row');
  const name = tchInput('', () => {}, { aria: t('tch.seq.name'), ph: t('tch.seq.new.ph'), cls: 'grow' });
  name.maxLength = 64; name.id = 'tch-new-name';
  name.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); teachNewSeq(name.value); } });
  nw.append(name, tchBtn(t('tch.seq.new'), () => teachNewSeq(name.value), 'btn-primary'));
  body.appendChild(nw);
  body.appendChild(cwEl('p', 'tch-k', t('tch.seq.hint')));
  const ol = cwEl('ol', 'tch-list'); ol.id = 'tch-seqs';
  if (!teachSaved().length) ol.appendChild(cwEl('li', 'empty', TCH.loaded ? t('tch.seq.empty') : '--'));
  for (const s of teachSaved()) {
    const li = cwEl('li'); li.dataset.id = s.id;
    const h = cwEl('div', 'tch-li-h');
    h.append(cwEl('b', '', s.name), cwEl('span', 'sp'));
    if (teachSeqStale(s)) h.appendChild(tchBadge(t('tch.stale'), 'stale'));
    h.appendChild(tchBadge(t('tch.seq.steps', { n: (s.steps || []).length })));
    li.appendChild(h);
    if (s.description) li.appendChild(cwEl('div', 'tch-sub', s.description));
    const a = cwEl('div', 'tch-acts');
    a.append(tchBtn(t('tch.seq.edit'), () => teachEdit(s.id)), tchBtn(t('tch.run'), () => { teachEdit(s.id); teachRun({}); }, 'btn-primary'));
    li.appendChild(a);
    ol.appendChild(li);
  }
  body.appendChild(ol);
}
function teachStepTitle(st) {
  if (st.type === 'pose') { const p = teachPose(st.pose_id); return t('tch.st.pose', { n: p ? p.name : t('tch.missing') }); }
  if (st.type === 'replay') { const e = teachEp(st.episode_id); return t('tch.st.replay', { n: e ? teachEpName(e) + (Number.isFinite(Number(e.duration_s)) && e.duration_s != null ? ' · ' + epFmtDur(Number(e.duration_s)) : '') : t('tch.missing') }); }
  if (st.type === 'gripper') return st.action === 'move' ? t('tch.st.grip.move', { p: Math.round((st.fraction ?? 0) * 100) }) : t('tch.st.grip.' + (st.action === 'close' ? 'close' : 'open'));
  if (st.type === 'wait') return t('tch.st.wait');
  return st.type;
}
function teachRenderEditor(body) {
  const c = TCH.cur, running = teachRunning();
  const head = cwEl('div', 'tch-row');
  head.appendChild(tchBtn(t('tch.back'), teachBackToList));
  const nm = tchInput(c.name, (v) => { c.name = v; teachDirty(); }, { aria: t('tch.seq.name'), cls: 'grow' }); nm.maxLength = 64; nm.id = 'tch-seq-name';
  head.appendChild(nm);
  body.appendChild(head);
  const acts = cwEl('div', 'tch-row');
  const save = tchBtn(TCH.dirty || !c.id ? t('tch.save') : t('tch.saved'), () => teachSave(), TCH.dirty || !c.id ? 'btn-primary' : '', TCH.busy || running, '/api/v1/sequences/save');
  save.id = 'tch-save';
  acts.appendChild(save);
  if (c.id) { const d = tchBtn(TCH.armed === 'seqdel' ? t('tch.del.confirm') : t('tch.del'), () => teachDelSeq(), 'tch-del' + (TCH.armed === 'seqdel' ? ' armed' : ''), running, '/api/v1/sequences/delete'); d.id = 'tch-seq-del'; acts.appendChild(d); }
  body.appendChild(acts);
  // steps
  const ol = cwEl('ol', 'tch-list'); ol.id = 'tch-steps';
  if (!c.steps.length) ol.appendChild(cwEl('li', 'empty', t('tch.st.empty')));
  c.steps.forEach((st, i) => {
    const li = cwEl('li'); li.dataset.i = String(i);
    li.classList.toggle('sel', TCH.sel === i);
    li.addEventListener('click', (e) => { if (e.target.closest('button, input, select')) return; TCH.sel = i; for (const x of ol.children) x.classList.toggle('sel', x === li); });
    const h = cwEl('div', 'tch-li-h');
    h.append(cwEl('span', 'tch-n', String(i + 1)), cwEl('b', '', teachStepTitle(st)), cwEl('span', 'sp'));
    if (teachStepMissing(st)) h.appendChild(tchBadge(t('tch.missing'), 'miss'));
    else if (teachStepStale(st)) h.appendChild(tchBadge(t('tch.stale'), 'stale'));
    li.appendChild(h);
    const p = cwEl('div', 'tch-row');
    const dirty = () => teachDirty();
    if (st.type === 'pose') {
      p.append(cwEl('span', 'tch-k', t('tch.speed')), tchSelect(TCH_SPEEDS.map((v) => [v, Math.round(v * 100) + '%']), TCH_SPEEDS.reduce((a, v) => (Math.abs(v - st.speed_scale) < Math.abs(a - st.speed_scale) ? v : a), 0.5), (v) => { st.speed_scale = Number(v); dirty(); }, t('tch.speed')));
      p.append(cwEl('span', 'tch-k', t('tch.grip')), tchSelect([['keep', t('tch.grip.keep')], ['pose', t('tch.grip.pose')]], st.gripper === 'pose' ? 'pose' : 'keep', (v) => { st.gripper = v; dirty(); }, t('tch.grip')));
    } else if (st.type === 'wait') {
      p.append(cwEl('span', 'tch-k', t('tch.wait.s')), tchInput(st.s, (v, el) => { const x = clamp(Number(v) || 0, 0, 60); st.s = x; el.value = String(x); dirty(); }, { type: 'number', min: 0, max: 60, step: 0.5, aria: t('tch.wait.s') }));
    } else if (st.type === 'gripper') {
      // a continuous opening (0 closed .. 1 open); an open / close step shows as 100 % / 0 % and becomes a move once changed
      const f = st.action === 'move' ? clamp(Number(st.fraction) || 0, 0, 1) : st.action === 'open' ? 1 : 0;
      const out = cwEl('output', 'tch-k', Math.round(f * 100) + '%');
      const rg = tchInput(f, (v) => { st.action = 'move'; st.fraction = Math.round(clamp(Number(v) || 0, 0, 1) * 100) / 100; setText(out, Math.round(st.fraction * 100) + '%'); setText(h.querySelector('b'), teachStepTitle(st)); dirty(); },
        { type: 'range', min: 0, max: 1, step: 0.05, aria: t('tch.grip.frac'), cls: 'grow' });
      p.append(cwEl('span', 'tch-k', t('tch.grip.frac')), rg, out);
    } else if (st.type === 'replay') {
      p.append(cwEl('span', 'tch-k', t('tch.speed')), tchSelect([[0.5, '0.5×'], [1, '1×']], st.speed === 1 ? 1 : 0.5, (v) => { st.speed = Number(v); dirty(); }, t('tch.speed')));
      const ck = cwEl('label', 'tch-chk'); const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = st.gripper !== false;
      cb.addEventListener('change', () => { st.gripper = cb.checked; dirty(); });
      ck.append(cb, cwEl('span', '', t('tch.grip')));
      p.appendChild(ck);
    }
    li.appendChild(p);
    const a = cwEl('div', 'tch-acts');
    const up = tchBtn('↑', () => teachMove(i, -1), '', i === 0 || running); up.setAttribute('aria-label', t('tch.up'));
    const dn = tchBtn('↓', () => teachMove(i, 1), '', i === c.steps.length - 1 || running); dn.setAttribute('aria-label', t('tch.down'));
    const from = tchBtn(t('tch.from'), () => teachRun({ from: i }), '', running || TCH.busy);
    const one = tchBtn(t('tch.single'), () => teachRun({ from: i, single: true }), '', running || TCH.busy);
    const del = tchBtn('✕', () => { c.steps.splice(i, 1); TCH.sel = -1; teachDirty(); teachRender(); }, 'tch-del', running); del.setAttribute('aria-label', t('tch.st.del'));
    a.append(from, one, up, dn, del);
    li.appendChild(a);
    ol.appendChild(li);
  });
  body.appendChild(ol);
  // add steps
  body.appendChild(cwEl('div', 'tch-sec', t('tch.add')));
  const add = cwEl('div', 'tch-box');
  const pr = cwEl('div', 'tch-row');
  const psel = tchSelect(TCH.poses.length ? TCH.poses.map((p) => [p.id, p.name + (p.stale ? ' ⚠' : '')]) : [['', t('tch.pose.empty')]], TCH.poses[0] ? TCH.poses[0].id : '', () => {}, t('tch.add.pose'));
  psel.classList.add('grow');
  pr.append(psel, tchBtn(t('tch.add.pose'), () => { if (psel.value) teachAddStep({ type: 'pose', pose_id: psel.value, speed_scale: 0.5, gripper: 'keep' }); }, '', !TCH.poses.length || running));
  const er = cwEl('div', 'tch-row');
  const esel = tchSelect(TCH.eps.length ? TCH.eps.map((e) => [e.id, teachEpName(e) + ' · ' + (e.dataset || 'default') + (e.stale ? ' ⚠' : '')]) : [['', t('tch.ep.empty')]], TCH.eps[0] ? TCH.eps[0].id : '', () => {}, t('tch.add.ep'));
  esel.classList.add('grow');
  er.append(esel, tchBtn(t('tch.add.ep'), () => { if (esel.value) teachAddStep({ type: 'replay', episode_id: esel.value, speed: 1, gripper: true }); }, '', !TCH.eps.length || running));
  const gr = cwEl('div', 'tch-row');
  gr.append(tchBtn(t('tch.add.grip'), () => teachAddStep({ type: 'gripper', action: 'move', fraction: teachGripFrac() }), '', running),
    tchBtn(t('tch.add.wait'), () => teachAddStep({ type: 'wait', s: 1 }), '', running));
  add.append(pr, er, gr);
  body.appendChild(add);
  // run
  body.appendChild(cwEl('div', 'tch-sec', t('tch.runbox')));
  const rb = cwEl('div', 'tch-box');
  const r1 = cwEl('div', 'tch-row');
  const rep = tchInput(TCH.repeat, (v, el) => { TCH.repeat = clamp(Math.round(Number(v) || 1), 1, 100); el.value = String(TCH.repeat); }, { type: 'number', min: 1, max: 100, step: 1, aria: t('tch.repeat') });
  rep.id = 'tch-repeat';
  const rc = cwEl('label', 'tch-chk'); const rcb = document.createElement('input'); rcb.type = 'checkbox'; rcb.id = 'tch-runrec'; rcb.checked = TCH.runRec;
  rcb.addEventListener('change', () => { TCH.runRec = rcb.checked; teachRender(); });
  rc.append(rcb, cwEl('span', '', t('tch.runrec')));
  r1.append(cwEl('span', 'tch-k', t('tch.repeat')), rep, rc);
  rb.appendChild(r1);
  if (TCH.runRec) {
    const r2 = cwEl('div', 'tch-row');
    const ds = TCH.runDs || TCH.ds;
    r2.append(cwEl('span', 'tch-k', t('tch.ds')), tchSelect(teachDsNames().map((n) => [n, n]), ds, (v) => { TCH.runDs = v; }, t('tch.ds')),
      tchInput(TCH.runTask, (v) => { TCH.runTask = v; }, { aria: t('tch.task'), ph: t('tch.task.ph', { n: c.name || '' }), cls: 'grow' }));
    rb.appendChild(r2);
  }
  const r3 = cwEl('div', 'tch-row');
  const go = tchBtn(t('tch.run'), () => teachRun({}), 'btn-primary', running || TCH.busy || !c.steps.length); go.id = 'tch-run';
  const one = tchBtn(t('tch.single.sel'), () => teachRun({ from: Math.max(0, TCH.sel), single: true }), '', running || TCH.busy || !c.steps.length); one.id = 'tch-step';
  const stop = tchBtn(t('tch.stop'), null, 'tch-stop'); stop.id = 'tch-stop'; stop.hidden = !running; bindStopButton(stop);
  r3.append(go, one, stop);
  rb.appendChild(r3);
  const pg = cwEl('div', 'tch-prog'); pg.id = 'tch-prog'; pg.setAttribute('role', 'status');
  rb.appendChild(pg);
  body.appendChild(rb);
}
function teachRenderPoses(body) {
  body.appendChild(cwEl('p', 'tch-k', t('tch.pose.hint')));
  const nw = cwEl('div', 'tch-row');
  const nm = tchInput('', () => {}, { aria: t('pose.name'), ph: 'bin_a', cls: 'grow' }); nm.maxLength = 64; nm.id = 'tch-pose-name';
  nm.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); teachSavePose(nm.value.trim()); } });
  nw.append(nm, tchBtn(t('pose.save'), () => teachSavePose(nm.value.trim()), 'btn-primary', false, '/api/v1/poses/save'));
  body.appendChild(nw);
  // ticked poses -> play them in tick order (saved as a motion, an identical one is reused) or open them as a new motion
  TCH.pick = TCH.pick.filter((id) => teachPose(id));
  const n = TCH.pick.length, running = teachRunning(), quick = running && TCH.run && TCH.run.quick;
  const pb = cwEl('div', 'tch-box tch-pick' + (n || quick ? ' on' : '')); pb.id = 'tch-pick';
  if (!n && !quick) pb.appendChild(cwEl('div', 'tch-k', t('tch.pick.none')));
  else {
    const r = cwEl('div', 'tch-row');
    const play = tchBtn(t('tch.pick.play', { n }), () => teachPlayPicked(false), 'btn-primary grow', !n || running || TCH.busy); play.id = 'tch-pick-play';
    const ed = tchBtn(t('tch.pick.edit'), teachPickToMotion, '', !n || running); ed.id = 'tch-pick-edit';
    r.append(play, ed, tchBtn(t('tch.pick.clear'), () => { TCH.pick = []; teachRender(); }, '', !n));
    pb.appendChild(r);
  }
  if (quick) {
    const r = cwEl('div', 'tch-row');
    const pg = cwEl('span', 'tch-prog grow'); pg.id = 'tch-pick-prog'; pg.setAttribute('role', 'status');
    const stop = tchBtn(t('tch.stop'), null, 'tch-stop'); stop.id = 'tch-pick-stop'; bindStopButton(stop);
    r.append(pg, stop);
    pb.appendChild(r);
  }
  body.appendChild(pb);
  const ol = cwEl('ol', 'tch-list'); ol.id = 'tch-poses';
  if (!TCH.poses.length) ol.appendChild(cwEl('li', 'empty', TCH.loaded ? t('pose.empty') : '--'));
  for (const p of TCH.poses) {
    const li = cwEl('li'); li.dataset.id = p.id || p.name;
    const h = cwEl('div', 'tch-li-h');
    const pi = TCH.pick.indexOf(p.id);
    li.classList.toggle('sel', pi >= 0);
    const ck = cwEl('label', 'tch-chk tch-pk'); const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = pi >= 0;
    cb.setAttribute('aria-label', t('tch.pick.aria', { n: p.name }));
    cb.addEventListener('change', () => { const j = TCH.pick.indexOf(p.id); if (cb.checked && j < 0) TCH.pick.push(p.id); else if (!cb.checked && j >= 0) TCH.pick.splice(j, 1); teachRender(); });
    ck.append(cb, cwEl('span', 'tch-n' + (pi >= 0 ? '' : ' off'), pi >= 0 ? String(pi + 1) : ''));
    h.append(ck, cwEl('b', '', p.name), cwEl('span', 'sp'));
    if (p.stale) { const b = tchBadge(t('tch.stale'), 'stale'); b.title = (p.stale.reason || '') + (p.stale.at ? ' (' + p.stale.at + ')' : ''); h.appendChild(b); }
    li.appendChild(h);
    const used = teachUsedBy('pose', p.id);
    const tcp = p.tcp && Array.isArray(p.tcp.position) ? p.tcp.position.map((v) => Math.round(v * 1000)).join(', ') + ' mm' : '';
    li.appendChild(cwEl('div', 'tch-sub', [used.length ? t('tch.usedby', { s: used.join(', ') }) : t('tch.unused'), tcp].filter(Boolean).join(' · ')));
    if (TCH.edit === 'pose:' + p.id) {
      const r = cwEl('div', 'tch-row');
      const inp = tchInput(p.name, () => {}, { aria: t('tch.rename'), cls: 'grow' }); inp.maxLength = 64; inp.id = 'tch-rename';
      r.append(inp, tchBtn(t('tch.save'), () => teachRenamePose(p, inp.value.trim()), 'btn-primary', false, '/api/v1/poses/rename'), tchBtn(t('tch.cancel'), () => { TCH.edit = null; teachRender(); }));
      li.appendChild(r);
    }
    const a = cwEl('div', 'tch-acts');
    const k = 'pose:' + p.name;
    a.append(tchBtn(t('pose.go'), () => poseGo(p.name)), tchBtn(t('tch.rename'), () => { TCH.edit = 'pose:' + p.id; teachRender(); }),
      tchBtn(TCH.armed === k ? t('tch.del.confirm') : t('tch.del'), () => teachDelPose(p), 'tch-del' + (TCH.armed === k ? ' armed' : ''), false, '/api/v1/poses/delete'));
    li.appendChild(a);
    ol.appendChild(li);
  }
  body.appendChild(ol);
}
function teachRenderEps(body) {
  // recording target for "● Record"
  const tgt = cwEl('div', 'tch-box');
  const r1 = cwEl('div', 'tch-row');
  r1.append(cwEl('span', 'tch-k', t('tch.rec.to')), tchSelect(teachDsNames().map((n) => [n, n]), TCH.ds, (v) => { TCH.ds = v; store('rebot.teach.ds', v); }, t('tch.ds')));
  const nds = tchInput('', () => {}, { aria: t('tch.ds.new'), ph: t('tch.ds.new'), cls: 'grow' }); nds.maxLength = 64; nds.id = 'tch-ds-new';
  r1.append(nds, tchBtn(t('tch.ds.create'), () => teachCreateDs(nds.value.trim()), '', false, '/api/v1/datasets/create'));
  const r2 = cwEl('div', 'tch-row');
  const task = tchInput(TCH.task, (v) => { TCH.task = v; store('rebot.teach.task', v); }, { aria: t('tch.task'), ph: t('tch.task.ph0'), cls: 'grow' }); task.id = 'tch-task';
  r2.append(cwEl('span', 'tch-k', t('tch.task')), task);
  const r3 = cwEl('div', 'tch-row');
  r3.append(cwEl('span', 'tch-k', t('tch.speed')), tchSelect([[0.5, '0.5×'], [1, '1×']], TCH.epSpeed, (v) => { TCH.epSpeed = Number(v); }, t('tch.speed')));
  const ck = cwEl('label', 'tch-chk'); const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = TCH.epGrip;
  cb.addEventListener('change', () => { TCH.epGrip = cb.checked; });
  ck.append(cb, cwEl('span', '', t('tch.grip')));
  r3.appendChild(ck);
  tgt.append(r1, r2, r3);
  body.appendChild(tgt);
  const playing = teachPlaying();
  const groups = new Map();
  for (const n of teachDsNames()) groups.set(n, []);
  for (const e of TCH.eps) { const n = e.dataset || 'default'; if (!groups.has(n)) groups.set(n, []); groups.get(n).push(e); }
  for (const [name, eps] of groups) {
    const info = TCH.dsets.find((d) => d.name === name);
    if (!eps.length && name !== TCH.ds && !info) continue;
    const h = cwEl('div', 'tch-sec');
    const dur = eps.reduce((a, e) => a + (Number(e.duration_s) || 0), 0);
    h.textContent = t('tch.ds.head', { n: name, c: eps.length, d: epFmtDur(dur) }) + (info && info.bytes > 0 ? ' · ' + epFmtSize(info.bytes) : '');
    body.appendChild(h);
    const ol = cwEl('ol', 'tch-list'); ol.dataset.ds = name;
    if (!eps.length) ol.appendChild(cwEl('li', 'empty', t('tch.ep.none')));
    for (const e of eps) {
      const li = cwEl('li'); li.dataset.id = e.id;
      li.classList.toggle('run', playing === e.id);
      const hh = cwEl('div', 'tch-li-h');
      hh.append(cwEl('b', '', teachEpName(e)), cwEl('span', 'sp'), tchBadge(tchSrc(e.source)));
      if (e.outcome === 'aborted') hh.appendChild(tchBadge(t('tch.ep.aborted'), 'ab'));
      if (e.stale) hh.appendChild(tchBadge(t('tch.stale'), 'stale'));
      li.appendChild(hh);
      const used = teachUsedBy('ep', e.id);
      li.appendChild(cwEl('div', 'tch-sub', [epFmtDur(Number(e.duration_s)), teachWhen(e.recorded_at), e.frames ? t('tch.frames', { n: e.frames }) : '', used.length ? t('tch.usedby', { s: used.join(', ') }) : ''].filter(Boolean).join(' · ')));
      if (TCH.edit === 'ep:' + e.id) {
        const r = cwEl('div', 'tch-row');
        const ti = tchInput(e.task || '', () => {}, { aria: t('tch.task'), ph: t('tch.task'), cls: 'grow' }); ti.id = 'tch-ep-task';
        const dsel = tchSelect(teachDsNames().map((n) => [n, n]), e.dataset || 'default', () => {}, t('tch.ds')); dsel.id = 'tch-ep-ds';
        r.append(ti, dsel, tchBtn(t('tch.save'), () => teachUpdateEp(e, ti.value.trim(), dsel.value), 'btn-primary', false, '/api/v1/episodes/update'), tchBtn(t('tch.cancel'), () => { TCH.edit = null; teachRender(); }));
        li.appendChild(r);
      }
      const a = cwEl('div', 'tch-acts');
      const k = 'ep:' + e.id;
      a.append(tchBtn(t('ep.play'), () => teachReplay(e.id, false), 'btn-primary', !!playing || teachRunning()), tchBtn(t('tch.ep.edit'), () => { TCH.edit = 'ep:' + e.id; teachRender(); }));
      const dl = cwEl('a', 'btn', t('tch.ep.dl')); dl.href = '/api/v1/episodes/' + encodeURIComponent(e.id) + '/data'; dl.download = e.id + '.parquet';
      a.append(dl, tchBtn(TCH.armed === k ? t('tch.del.confirm') : t('tch.del'), () => teachDelEp(e), 'tch-del' + (TCH.armed === k ? ' armed' : ''), false, '/api/v1/episodes/delete'));
      li.appendChild(a);
      ol.appendChild(li);
    }
    body.appendChild(ol);
  }
}
function teachAskRender() {
  const a = TCH.ask, el = $('teach-ask');
  el.hidden = !a;
  if (!a) return;
  setText($('teach-ask-t'), a.kind === 'ep' ? t('tch.ask.ep') : t('tch.ask.seq'));
  setText($('teach-ask-yes'), t('tch.ask.yes'));
  setText($('teach-ask-no'), t('tch.cancel'));
}

/* ---- sequences ---- */
function teachDirty() { TCH.dirty = true; const b = $('tch-save'); if (b) { setText(b, t('tch.save')); b.classList.add('btn-primary'); } }
function teachNewSeq(name) {
  name = (name || '').trim();
  if (!TCH_SEQ_NAME.test(name)) { teachStatus(t('tch.e.name'), true); $('tch-new-name') && $('tch-new-name').focus(); return; }
  if (TCH.seqs.some((s) => s.name === name)) { teachStatus(t('tch.e.dup'), true); return; }
  TCH.cur = { name, description: '', steps: [] }; TCH.dirty = true; TCH.sel = -1;
  teachStatus(t('tch.seq.created', { n: name }));
  teachRender();
}
function teachEdit(id) {
  const s = TCH.seqs.find((x) => x.id === id); if (!s) return;
  TCH.cur = JSON.parse(JSON.stringify(s)); TCH.cur.steps = TCH.cur.steps || []; TCH.dirty = false; TCH.sel = -1; TCH.tab = 'seq';
  teachRender();
}
function teachBackToList() {
  if (TCH.dirty && TCH.armed !== 'leave') { teachArm('leave'); teachStatus(t('tch.unsaved'), true); return; }
  teachDisarm(); TCH.cur = null; TCH.dirty = false; teachStatus('--'); teachRender();
}
function teachMove(i, d) {
  const s = TCH.cur.steps, j = i + d; if (j < 0 || j >= s.length) return;
  [s[i], s[j]] = [s[j], s[i]]; TCH.sel = j; teachDirty(); teachRender();
}
function teachAddStep(st) {
  if (!TCH.cur) return false;
  if (TCH.cur.steps.length >= 200) { teachStatus(t('tch.e.max'), true); return false; }
  const at = TCH.sel >= 0 && TCH.sel < TCH.cur.steps.length ? TCH.sel + 1 : TCH.cur.steps.length;
  TCH.cur.steps.splice(at, 0, st); TCH.sel = at; teachDirty();
  teachStatus(t('tch.st.added', { n: at + 1, s: teachStepTitle(st) }));
  teachRender();
  const li = document.querySelector('#tch-steps > li.sel');
  if (li && TCH.open) li.scrollIntoView({ block: 'nearest' });
  return true;
}
async function teachSave(controlSnap = null) {
  const c = TCH.cur; if (!c) return false;
  if (controlSnap && !isControlSnapValid(controlSnap)) return false;
  c.name = (c.name || '').trim();
  if (!TCH_SEQ_NAME.test(c.name)) { teachStatus(t('tch.e.name'), true); return false; }
  if (TCH.seqs.some((s) => s.name === c.name && s.id !== c.id)) { teachStatus(t('tch.e.dup'), true); return false; }
  const body = { name: c.name, description: c.description || '', steps: c.steps };
  if (c.id) { body.id = c.id; if (c.rev != null) body.rev = c.rev; }
  TCH.busy = true;
  try {
    const r = await commandRPC('/api/v1/sequences/save', body, true, 2000, controlSnap ? { controlSnap } : null);
    const s = r && r.sequence ? r.sequence : { ...c };
    TCH.cur = JSON.parse(JSON.stringify(s)); TCH.cur.steps = TCH.cur.steps || []; TCH.dirty = false;
    const k = TCH.seqs.findIndex((x) => x.id === s.id);
    if (k >= 0) TCH.seqs[k] = s; else TCH.seqs.push(s);
    teachStatus(t('tch.seq.saved', { n: s.name }));
    return true;
  } catch (e) {
    if (isCancelledErr(e)) return false;
    teachStatus(e.code === 'SEQUENCE_CONFLICT' ? t('tch.e.SEQUENCE_CONFLICT') : teachErr(e), true);
    if (e.code === 'SEQUENCE_REF_MISSING' && e.reply && Number.isInteger(e.reply.step)) TCH.sel = e.reply.step;
    return false;
  } finally { TCH.busy = false; teachRender(); }
}
async function teachDelSeq() {
  const c = TCH.cur; if (!c || !c.id) return;
  if (!teachArm('seqdel')) return;
  try {
    await commandRPC('/api/v1/sequences/delete', { id: c.id });
    TCH.seqs = TCH.seqs.filter((s) => s.id !== c.id); TCH.cur = null; TCH.dirty = false;
    teachStatus(t('tch.seq.deleted', { n: c.name }));
  } catch (e) { if (!isCancelledErr(e)) teachStatus(teachErr(e), true); }
  teachRender();
}
const teachRunning = () => { const a = S.obs && S.obs.action; return !!(a && a.kind === 'sequence' && a.state === 'running'); };
const teachPlaying = () => { const a = S.obs && S.obs.action; return a && a.kind === 'replay' && a.state === 'running' ? a.episode : null; };
function teachMoveBlock() {
  if (!isLive() || !canMove()) { toast(isLive() && !isStopped() && !colHold() && leaseOther() == null ? t('tch.enable_first') : lockReason(), 'err'); flashLock(); return true; }
  return false;
}
async function teachRun({ from = 0, single = false, allowStale = false }) {
  const c = TCH.cur; if (!c || TCH.busy) return;
  if (!c.steps.length) { teachStatus(t('tch.st.empty'), true); return; }
  if (teachMoveBlock()) return;
  const snap = requireLeaseSyncOrPrompt((e) => { teachStatus(teachErr(e), true); toast(teachErr(e), 'err', e.code || ''); });
  if (!snap) return;
  if ((TCH.dirty || !c.id) && !(await teachSave(snap))) return;
  if (!isControlSnapValid(snap)) return;
  const body = { id: TCH.cur.id, from_step: clamp(from, 0, TCH.cur.steps.length - 1), single_step: !!single, repeat: single ? 1 : TCH.repeat, allow_stale: !!allowStale, tag: 'cockpit' };
  if (TCH.runRec && !single) body.record = { dataset: TCH.runDs || TCH.ds, task: TCH.runTask.trim() || TCH.cur.name };
  TCH.busy = true;
  try {
    if (!isControlSnapValid(snap)) throw cancelledError();
    const r = await commandRPC('/api/v1/action/run_sequence', body, true, 2000, { controlSnap: snap });
    syncEpoch(r.epoch_id);
    TCH.run = { actionId: r.action_id, seqId: TCH.cur.id, n: TCH.cur.steps.length, record: !!body.record };
    TCH.ask = null;
    teachStatus(single ? t('tch.run.single', { n: body.from_step + 1 }) : t('tch.run.started', { n: TCH.cur.name, r: body.repeat }));
  } catch (e) {
    if (isCancelledErr(e)) return;
    if (e.code === 'DATA_STALE') { TCH.ask = { kind: 'seq', from, single }; teachStatus(teachErr(e), true); }
    else { teachStatus(teachErr(e), true); toast(teachErr(e), 'err', e.code || ''); }
  } finally { TCH.busy = false; teachRender(); }
}
function teachStop() { stopMotion(); }
// live step highlight and progress from the parent action (kind "sequence")
function teachLive() {
  const o = S.obs; if (!o || !TCH.open) return;
  const a = o.action, run = a && a.kind === 'sequence' && a.state === 'running' ? a : null;
  const mine = run && TCH.cur && ((TCH.run && run.id === TCH.run.actionId) || (run.sequence_id && run.sequence_id === TCH.cur.id));
  const ol = $('tch-steps');
  if (ol) for (const li of ol.children) li.classList.toggle('run', !!mine && Number(li.dataset.i) === run.step_index);
  const pg = $('tch-prog');
  if (pg) {
    setText(pg, mine ? t('tch.prog', { i: (run.step_index ?? 0) + 1, n: TCH.cur.steps.length, r: (run.repeat_index ?? 0) + 1, m: run.repeats ?? 1,
      s: run.phase ? (I18N.zh['tch.ph.' + run.phase] !== undefined ? t('tch.ph.' + run.phase) : run.phase) : '' }) : '');
    const st = $('tch-stop'); if (st) st.hidden = !run;
  }
  const qp = $('tch-pick-prog'), quick = run && TCH.run && TCH.run.quick && run.id === TCH.run.actionId ? run : null;
  if (qp) setText(qp, quick ? t('tch.pick.prog', { i: (quick.step_index ?? 0) + 1, n: TCH.run.n }) : '');
  const qid = quick && TCH.run.poses[quick.step_index ?? 0];
  for (const li of document.querySelectorAll('#tch-poses > li')) li.classList.toggle('run', !!qid && li.dataset.id === qid);
  const pl = teachPlaying();
  for (const li of document.querySelectorAll('#teach-body ol[data-ds] > li')) li.classList.toggle('run', !!pl && li.dataset.id === pl);
}
function teachOnObs(o) {
  if (!teachOn()) return;
  // result of a sequence / replay started here, once
  for (const r of [o.action, o.previous_action]) {
    if (!r || r.state === 'running' || TCH.toasted === r.id) continue;
    if (r.kind === 'sequence' && TCH.run && r.id === TCH.run.actionId) {
      TCH.toasted = r.id;
      const done = TCH.run.quick ? t('tch.pick.done') : t('tch.run.done');
      if (r.state === 'done') { toast(done, 'ok'); teachStatus(done); }
      else { const m = t('tch.run.ended', { code: r.code || r.state, m: teachErr({ code: r.code, message: r.message }) }); toast(m, r.state === 'failed' ? 'err' : 'warn', r.code || ''); teachStatus(m, true); }
      TCH.run = null;
      if (TCH.open) { teachLoad(true); }
    } else if (r.kind === 'replay' && TCH.play && r.id === TCH.play) {
      TCH.toasted = r.id; TCH.play = null;
      if (r.state === 'done') teachStatus(t('ep.done', { id: r.episode }));
      else teachStatus(t('ep.ended', { code: r.code || r.state, m: teachErr({ code: r.code, message: r.message }) }), true);
      if (TCH.open) teachRender();
    }
  }
  const running = teachRunning(), playing = teachPlaying();
  const key = (running ? 1 : 0) + ':' + (playing || '');
  if (TCH.open && key !== TCH.runKey) { TCH.runKey = key; teachRender(); }
  else if (TCH.open) teachLive();
  teachBarRender();
}

/* ---- poses ---- */
function teachAutoPoseName() {
  const used = new Set(TCH.poses.map((p) => p.name));
  for (let k = 1; k < 1000; k++) { const n = 'p' + String(k).padStart(2, '0'); if (!used.has(n)) return n; }
  return 'p' + Date.now();
}
// save the current position (all joints, the gripper opening included: armd stores the measured angles) as a pose;
// from the strip with a sequence open, append it as a pose step that also restores the opening (gripper "pose")
async function teachSavePose(name, append = false) {
  if (!POSE_NAME_RE.test(name || '')) { teachStatus(t('pose.bad'), true); return; }
  try {
    const r = await teachPoseCmd('/api/v1/poses/save', { name });
    const ps = await teachGet('/api/v1/poses').catch(() => null);
    if (ps && Array.isArray(ps.poses)) TCH.poses = ps.poses;
    if (ps && typeof ps.revision === 'number') PS.rev = ps.revision;
    const p = (r && r.pose && r.pose.id) ? r.pose : TCH.poses.find((x) => x.name === name);
    toast(t('pose.saved', { name }), 'ok');
    if (append && TCH.cur && p && p.id) { teachAddStep({ type: 'pose', pose_id: p.id, speed_scale: 0.5, gripper: 'pose' }); await teachStripSave(); }
    else {
      // from the strip without an open motion the new pose is ticked: record a few, then "play in ticked order"
      if (append && p && p.id && !TCH.pick.includes(p.id)) TCH.pick.push(p.id);
      teachStatus(t('pose.saved', { name }));
    }
    teachRender(); teachBarRender();
  } catch (e) { if (isCancelledErr(e)) return; const m = teachErr(e); toast(m, 'err', e.code || ''); teachStatus(m, true); }
}
async function teachRenamePose(p, name) {
  if (!POSE_NAME_RE.test(name)) { teachStatus(t('pose.bad'), true); return; }
  if (name === p.name) { TCH.edit = null; teachRender(); return; }
  try {
    await teachPoseCmd('/api/v1/poses/rename', { id: p.id, name });
    TCH.edit = null; teachStatus(t('tch.renamed', { a: p.name, b: name }));
    await teachLoad(true);
  } catch (e) { if (isCancelledErr(e)) return; teachStatus(e.code === 'UNKNOWN_COMMAND' || e.code === 'NOT_FOUND' || e.code === '404' || e.code === 'UNSUPPORTED' ? t('tch.e.rename_na') : teachErr(e), true); }
}
async function teachDelPose(p) {
  if (!teachArm('pose:' + p.name)) return;
  try {
    try { await teachPoseCmd('/api/v1/poses/delete', { name: p.name }); }
    catch (e) {
      // only the quick-play slot uses it: drop the slot (it is rebuilt by the next play), then delete again
      const ids = e.code === 'IN_USE' && e.reply && Array.isArray(e.reply.sequences) ? e.reply.sequences : [];
      if (!ids.length || !ids.every((id) => teachQuick(TCH.seqs.find((x) => x.id === id)))) throw e;
      for (const id of ids) await commandRPC('/api/v1/sequences/delete', { id });
      TCH.seqs = TCH.seqs.filter((x) => !ids.includes(x.id));
      await teachPoseCmd('/api/v1/poses/delete', { name: p.name });
    }
    teachStatus(t('pose.deleted', { name: p.name }));
    await teachLoad(true);
  } catch (e) { if (!isCancelledErr(e)) teachStatus(teachErr(e), true); teachRender(); }
}

// the ticked poses as steps of a motion (each restores the gripper opening saved with the pose)
const teachPickSteps = () => TCH.pick.filter((id) => teachPose(id)).map((id) => ({ type: 'pose', pose_id: id, speed_scale: 0.5, gripper: 'pose' }));
function teachAutoSeqName() {
  const used = new Set(TCH.seqs.map((s) => s.name));
  for (let k = 1; k < 1000; k++) { const n = t('tch.pick.auto', { n: k }); if (!used.has(n)) return n; }
  return 'motion ' + Date.now();
}
// "play in ticked order": the steps go into the quick-play slot (created once, then overwritten; no rev, the last play
// wins) and run once; the panel stays on the poses tab with the progress and Stop; the ticks stay for another play
async function teachPlayPicked(allowStale) {
  const steps = teachPickSteps();
  if (!steps.length || TCH.busy || teachRunning()) return;
  if (teachMoveBlock()) return;
  const snap = requireLeaseSyncOrPrompt((e) => { teachStatus(teachErr(e), true); toast(teachErr(e), 'err', e.code || ''); });
  if (!snap) return;
  TCH.busy = true; teachRender();
  try {
    const sq = await teachGet('/api/v1/sequences').catch(() => null);
    if (!isControlSnapValid(snap)) throw cancelledError();
    if (sq && Array.isArray(sq.sequences)) TCH.seqs = sq.sequences;
    const slot = TCH.seqs.find(teachQuick);
    const body = { name: TCH_QUICK, description: t('tch.pick.desc'), steps };
    if (slot) body.id = slot.id;
    const r = await commandRPC('/api/v1/sequences/save', body, true, 2000, { controlSnap: snap });
    if (!isControlSnapValid(snap)) throw cancelledError();
    const q = r && r.sequence && r.sequence.id ? r.sequence : { ...body, id: slot && slot.id };
    if (!q.id) throw Object.assign(new Error(t('err.rejected')), { code: 'SEQUENCE_NOT_FOUND' });
    const k = TCH.seqs.findIndex((x) => x.id === q.id);
    if (k >= 0) TCH.seqs[k] = q; else TCH.seqs.push(q);
    const d = await commandRPC('/api/v1/action/run_sequence', { id: q.id, from_step: 0, single_step: false, repeat: 1, allow_stale: !!allowStale, tag: 'cockpit' }, true, 2000, { controlSnap: snap });
    syncEpoch(d.epoch_id);
    TCH.run = { actionId: d.action_id, seqId: q.id, n: steps.length, record: false, quick: true, poses: steps.map((x) => x.pose_id) };
    TCH.ask = null;
    teachStatus(t('tch.pick.started', { n: steps.length }));
  } catch (e) {
    if (isCancelledErr(e)) return;
    if (e.code === 'DATA_STALE') { TCH.ask = { kind: 'quick' }; teachStatus(teachErr(e), true); }
    else { teachStatus(teachErr(e), true); toast(teachErr(e), 'err', e.code || ''); }
  } finally { TCH.busy = false; teachRender(); }
}
// "make a motion": a new unsaved motion with the ticked poses, opened in the editor (name it, add gripper / wait steps)
function teachPickToMotion() {
  const steps = teachPickSteps(); if (!steps.length) return;
  if (TCH.cur && TCH.dirty) { TCH.tab = 'seq'; teachStatus(t('tch.unsaved'), true); teachRender(); return; }
  TCH.cur = { name: teachAutoSeqName(), description: '', steps }; TCH.dirty = true; TCH.sel = -1; TCH.tab = 'seq'; TCH.pick = [];
  teachStatus(t('tch.seq.created', { n: TCH.cur.name }));
  teachRender();
}

/* ---- episodes / datasets ---- */
async function teachReplay(id, allowStale) {
  if (teachMoveBlock()) return;
  try {
    const d = await commandRPC('/api/v1/action/replay', { episode: id, speed: TCH.epSpeed, gripper: TCH.epGrip, allow_stale: !!allowStale, tag: 'cockpit' });
    syncEpoch(d.epoch_id);
    TCH.play = d.action_id; TCH.ask = null;
    const e = teachEp(id);
    teachStatus(t('ep.started', { id: e ? teachEpName(e) : id, x: TCH.epSpeed }));
  } catch (e) {
    if (isCancelledErr(e)) { teachRender(); return; }
    if (e.code === 'DATA_STALE') TCH.ask = { kind: 'ep', id };
    teachStatus(teachErr(e), true);
  }
  teachRender();
}
async function teachUpdateEp(e, task, dataset) {
  const body = { id: e.id };
  if (task !== (e.task || '')) body.task = task;
  if (dataset && dataset !== (e.dataset || 'default')) body.dataset = dataset;
  if (Object.keys(body).length === 1) { TCH.edit = null; teachRender(); return; }
  try {
    await teachCmd('/api/v1/episodes/update', body);
    TCH.edit = null; teachStatus(t('tch.ep.updated'));
    await teachLoad(true);
  } catch (err) { teachStatus(teachErr(err), true); }
}
async function teachDelEp(e) {
  if (!teachArm('ep:' + e.id)) return;
  try {
    await teachCmd('/api/v1/episodes/delete', { id: e.id });
    teachStatus(t('ep.deleted', { id: teachEpName(e) }));
    await teachLoad(true);
  } catch (err) { teachStatus(teachErr(err), true); teachRender(); }
}
async function teachCreateDs(name) {
  if (!TCH_SEQ_NAME.test(name || '')) { teachStatus(t('tch.e.name'), true); return; }
  try {
    await teachCmd('/api/v1/datasets/create', { name });
    TCH.ds = name; store('rebot.teach.ds', name);
    teachStatus(t('tch.ds.created', { n: name }));
    await teachLoad(true);
  } catch (e) { teachStatus(teachErr(e), true); }
}

/* ---- recording (this page owns the recorder; no lease needed) ---- */
function teachRecSet(r) {
  TCH.rec = r;
  setRecording(!!r);
  if (r) S.rec.startedAt = r.startedAt;
  teachBarRender();
}
async function teachRecToggle() {
  if (!TCH.rec) {
    const body = { dataset: TCH.ds || 'default', source: gcOn() ? 'hand_guide' : 'jog' };
    if (TCH.task.trim()) body.task = TCH.task.trim();
    try {
      await teachCmd('/api/v1/record/start', body);
      teachRecSet({ startedAt: Date.now(), dataset: body.dataset });
      toast(t('tch.rec.started', { d: body.dataset }), 'ok');
      teachStatus(t('tch.rec.started', { d: body.dataset }));
    } catch (e) {
      const m = e.code === 'RECORDER_BUSY' ? t('tch.e.RECORDER_BUSY', { o: (e.reply && e.reply.owner) ? leaseHolderText(e.reply.owner) : '?' }) : teachErr(e);
      toast(m, 'err', e.code || ''); teachStatus(m, true);
    }
    return;
  }
  try {
    const r = await teachCmd('/api/v1/record/stop', { save: true }, 15000);
    teachRecSet(null);
    const ep = r && r.episode;
    if (ep && ep.id) {
      TCH.acked.set(ep.id, { ep, at: Date.now() });
      if (!TCH.eps.some((x) => x.id === ep.id)) TCH.eps.unshift(ep);
      const msg = t('tch.rec.saved', { n: teachEpName(ep), d: epFmtDur(Number(ep.duration_s)) });
      toast(msg, 'ok');
      if (TCH.cur) { teachAddStep({ type: 'replay', episode_id: ep.id, speed: 1, gripper: true }); await teachStripSave(); }
      else teachStatus(msg);
    } else teachStatus(t('rec.discarded'));
    if (TCH.open) teachLoad(true);
    // the saved-episodes card lists the same store (GET /api/v1/dataset/episodes)
    if (EP.loaded) epLoad(true);
  } catch (e) {
    if (e.code === 'RECORDER_NOT_OWNER' || e.code === 'NOT_RECORDING') teachRecSet(null);
    const m = teachErr(e); toast(m, 'err', e.code || ''); teachStatus(m, true);
  }
}
async function teachRecDiscard() {
  if (!TCH.rec) return;
  try { await teachCmd('/api/v1/record/stop', { save: false }); teachRecSet(null); teachStatus(t('rec.discarded')); }
  catch (e) { teachStatus(teachErr(e), true); }
}

/* ---- teach strip: 记点 / ● 录轨迹 / the open sequence; shown while teaching, with the panel open or closed. The arm and
   the gripper are moved with the normal controls (sticks, gripper slider) or by hand while hand-guiding; the strip
   only snapshots and records, so every control works as without it. ---- */
const teachActive = () => gcOn() || (isLive() && canMove() && leaseOther() == null);
// a step added from the strip is saved at once (the panel may be closed); a failure stays visible as "unsaved"
async function teachStripSave() { if (TCH.cur && TCH.dirty && !TCH.busy) await teachSave(); teachBarRender(); }
// simple layout: the video fills the screen, so the strip floats (position: fixed) and docks clear of every control
const TCH_STRIP_AVOID = DOCK_AVOID + ', #btn-gc, #grip-range, #grip-col, #view-switch, #safety, #gc-banner, #pose-card, .presets, .hud, #btn-preempt';
function teachBarRender() {
  const bar = $('teach-bar');
  // while hand-guiding the strip shows by itself (also when hand-guiding started elsewhere) until its × hides it
  if (!gcOn()) TCH.gcHide = false;
  const show = teachOn() && (TCH.teach || !!TCH.rec || (gcOn() && !TCH.gcHide));
  if (bar.hidden !== !show) { bar.hidden = !show; TCH.dockAt = 0; }
  $('teach-start-row').hidden = show;
  if (!show) return;
  if (!TCH.loaded && !TCH.loadReq) { TCH.loadReq = true; teachLoad(true); }
  const can = teachActive() && !teachRunning();
  $('tb-pose').disabled = !can;
  const rec = $('tb-rec');
  rec.disabled = !TCH.rec && !can;
  rec.classList.toggle('on', !!TCH.rec);
  setText(rec, TCH.rec ? t('tch.b.recon', { t: fmtTime(Date.now() - TCH.rec.startedAt) }) : t('tch.b.rec'));
  const c = TCH.cur;
  setText($('tb-seq'), c ? t(TCH.dirty ? 'tch.b.seq.dirty' : 'tch.b.seq', { n: c.name || '?', c: c.steps.length }) : t('tch.b.noseq'));
  $('tb-x').disabled = !!TCH.rec;
  // docking reads layout: on show and at most every 500 ms from the observation stream (resize / scroll dock at once)
  if (getComputedStyle(bar).position === 'fixed') {
    if (Date.now() - (TCH.dockAt || 0) > 500) { TCH.dockAt = Date.now(); dockOnVideo(bar, TCH_STRIP_AVOID); }
  } else if (bar.style.top) { bar.style.top = ''; bar.style.left = ''; bar.style.maxWidth = ''; }
}
(() => {
  $('teach-close').addEventListener('click', () => teachOpen(false));
  $('tb-edit').addEventListener('click', () => { if (TCH.open) teachOpen(false); else teachOpen(true, { tab: TCH.cur ? 'seq' : 'pose' }); });
  $('tb-x').addEventListener('click', () => { if (TCH.rec) return; teachOpen(false); teachStrip(false); if (gcOn()) { TCH.gcHide = true; teachBarRender(); } });
  $('teach-start').addEventListener('click', () => teachStrip(true));
  $('teach-tabs').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { TCH.tab = b.dataset.tab; TCH.edit = null; teachDisarm(); teachRender(); } });
  $('teach-tabs').addEventListener('keydown', (e) => {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
    const tabs = ['pose', 'seq', 'ep'], k = (tabs.indexOf(TCH.tab) + (e.key === 'ArrowRight' ? 1 : 2)) % 3;
    TCH.tab = tabs[k]; teachRender(); $('teach-tab-' + TCH.tab).focus();
  });
  $('teach-ask-yes').addEventListener('click', () => { const a = TCH.ask; TCH.ask = null; if (!a) return; if (a.kind === 'ep') teachReplay(a.id, true); else if (a.kind === 'quick') teachPlayPicked(true); else teachRun({ from: a.from, single: a.single, allowStale: true }); });
  $('teach-ask-no').addEventListener('click', () => { TCH.ask = null; teachStatus('--'); teachRender(); });
  $('tb-pose').addEventListener('click', () => teachSavePose(teachAutoPoseName(), true));
  $('tb-rec').addEventListener('click', teachRecToggle);
  for (const ev of ['resize', 'scroll']) window.addEventListener(ev, () => { TCH.dockAt = 0; teachBarRender(); }, { passive: true });
  setInterval(() => { if (TCH.rec) teachBarRender(); }, 500);
  i18nHooks.push(() => { if (TCH.open) teachRender(); teachBarRender(); });
})();

