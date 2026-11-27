/* =================================================================
   External control monitor: #ext-banner (cockpit / side column), #act-chip (video HUD),
   topbar #btn-task + #task drawer (activity ring buffer, read-only policy & cell view).
   Every server-supplied string (lease_holder, tag, kind, phase, code, message, ids, names)
   is written with textContent only.
   ================================================================= */
const TK = {
  log: [], max: 40, tracked: new Map(), lastTs: null, seq: 0, ver: 0, // ring buffer of action / lease entries; ver bumps on any change
  holder: undefined, lastExtHolder: null,  // last observed lease_holder; last external one
  cmdRef: null, cmdAt: -Infinity,          // commanded positions at their last change; observation time of that change
  lastExt: null, dismissed: new Set(),     // newest external action entry; action ids whose banner the user dismissed
  ownIds: new Set(),                       // action ids from this page's own command / preset acks (every page tags 'cockpit')
  k: null, key: '', listKey: '', envKey: '',
  filterExt: store('rebot.task.f') === 'ext', tab: 'act', opener: null,
  timer: 0, inflight: false, env: null, envAt: 0,
};
function tkWhoShort(h) {
  if (h == null) return t('whos.other');
  if (h === CLIENT_ID) return t('who.self');
  if (/^rebot-policy/.test(h)) return t('whos.policy');
  if (/^rebot-(app|teleop)/.test(h)) return t('whos.app');
  if (/^rebot-(mcp|companion)/.test(h)) return t('whos.agent');
  if (/^cyclops_/.test(h)) return t('whos.wizard');
  if (/^web-/.test(h)) return t('whos.cockpit');
  return t('whos.other');
}
const tkWho = (h) => (h === CLIENT_ID ? t('who.self') : leaseHolderText(h));
const tkHms = (ms) => { const d = new Date(ms); return [d.getHours(), d.getMinutes(), d.getSeconds()].map((v) => String(v).padStart(2, '0')).join(':'); };
const tkLabel = (e) => e.tag || e.kind || '--';
const tkDur = (e) => (e.dur == null ? '' : e.state === 'running' ? Math.floor(e.dur) + ' s' : e.dur.toFixed(1) + ' s');
const tkPct = (e) => Math.round(100 * clamp(Number(e.progress) || 0, 0, 1));
const tkSt = (st) => (I18N.zh['ext.st.' + st] !== undefined ? t('ext.st.' + st) : String(st || '?'));
const TK_ICON = { running: '●', done: '✓', failed: '✕', cancelled: '⊘' };
function tkPush(e) {
  TK.log.push(e);
  if (TK.log.length > TK.max) TK.log.splice(0, TK.log.length - TK.max);
  TK.ver++;
}
// Insert or update one action entry in place (keyed by action.id).
function tkAction(a, o, now) {
  if (!a || a.id == null) return null;
  const id = 'a' + a.id;
  let e = TK.tracked.get(id);
  if (!e) {
    // own: acked to this page, or untagged and started while this page held the (exclusive) lease
    const own = TK.ownIds.has(a.id) || (!a.tag && S.leaseGeneration != null);
    const h = o.lease_holder;
    e = { id, aid: a.id, type: 'action', own, sig: '',
      src: own ? CLIENT_ID : (h != null && h !== CLIENT_ID ? h : TK.lastExtHolder),
      // client clock (lease events use it too): first sight minus the time already run
      at: now - 1000 * (Number.isFinite(a.started_at) ? Math.max(0, (a.finished_at ?? o.timestamp) - a.started_at) : 0) };
    TK.tracked.set(id, e);
    if (TK.tracked.size > 64) {
      for (const k of TK.tracked.keys()) {
        if (TK.lastExt && TK.lastExt.id === k) continue;
        TK.tracked.delete(k);
        break;
      }
    }
    tkPush(e);
  }
  e.seen = o.timestamp;
  const dur = Number.isFinite(a.started_at) ? Math.max(0, (a.finished_at ?? o.timestamp) - a.started_at) : null;
  const sig = [a.state, a.tag, a.kind, a.phase, a.code, a.message, a.holding, Math.round(100 * (a.progress || 0)),
    dur == null ? '' : a.state === 'running' ? Math.floor(dur) : dur.toFixed(1)].join('|');
  if (sig !== e.sig) {
    e.sig = sig;
    Object.assign(e, { kind: a.kind, tag: a.tag, state: a.state, phase: a.phase, code: a.code, message: a.message,
      holding: a.holding, progress: a.progress, finished_at: a.finished_at, dur });
    TK.ver++;
  }
  if (!e.own && (!TK.lastExt || Number(e.aid) >= Number(TK.lastExt.aid))) TK.lastExt = e;
  return e;
}
// An ack names an action this page started. It can arrive after the first observation of that action.
function tkOwnAction(aid) {
  TK.ownIds.add(aid);
  if (TK.ownIds.size > 64) TK.ownIds.delete(TK.ownIds.values().next().value);
  const e = TK.tracked.get('a' + aid);
  if (!e || e.own) return;
  e.own = true; e.src = CLIENT_ID; TK.ver++;
  if (TK.lastExt === e) {
    TK.lastExt = null;
    for (const x of TK.tracked.values()) if (!x.own && x.type === 'action' && (!TK.lastExt || Number(x.aid) >= Number(TK.lastExt.aid))) TK.lastExt = x;
  }
}
function tkPill() {
  const E = TK.env;
  const sb = E && ((E.state && E.state.v) || (E.catalog && E.catalog.v));
  if (!sb) return '';
  if (sb.active_policy) return t('ext.policy', { id: String(sb.active_policy) });
  return sb.mode && sb.mode !== 'none' ? t('ext.mode', { m: String(sb.mode) }) : '';
}
function tkOnObs(o) {
  // Action ids are unique for one armd run; an E-stop, fault or fault clear changes the epoch but keeps them, and the
  // observation keeps reporting the last action. Only an armd restart (its clock going backwards) starts the ids again.
  if (Number.isFinite(o.timestamp)) {
    if (TK.lastTs != null && o.timestamp < TK.lastTs) { TK.tracked.clear(); TK.lastExt = null; TK.dismissed.clear(); TK.ownIds.clear(); }
    TK.lastTs = o.timestamp;
  }
  const now = Date.now();
  const h = o.lease_holder ?? null;
  if (h !== TK.holder) {
    if (TK.holder != null) tkPush({ id: 'l' + (++TK.seq), type: 'lease', ev: 'release', src: TK.holder, own: TK.holder === CLIENT_ID, at: now });
    if (h != null) {
      tkPush({ id: 'l' + (++TK.seq), type: 'lease', ev: 'acquire', src: h, own: h === CLIENT_ID, at: now });
      if (h !== CLIENT_ID) TK.lastExtHolder = h;
    }
    TK.holder = h;
  }
  tkAction(o.previous_action, o, now);
  tkAction(o.action, o, now);
  // streaming evidence: commanded positions moved (> 2 mrad from the last change) while torqued in the last 1 s,
  // or an onboard policy run is active
  const cmd = o.commanded_positions;
  if (Array.isArray(cmd) && o.is_torqued) {
    const ref = TK.cmdRef;
    if (!ref || ref.length !== cmd.length) TK.cmdRef = cmd.slice();
    else if (cmd.some((v, i) => Math.abs(v - ref[i]) > 0.002)) { TK.cmdRef = cmd.slice(); TK.cmdAt = o.timestamp; }
  } else TK.cmdRef = null;
  const streaming = (Number.isFinite(o.timestamp) && o.timestamp - TK.cmdAt >= 0 && o.timestamp - TK.cmdAt <= 1) || !!(o.policy && o.policy.active);
  const e = TK.lastExt;
  const other = leaseOther();
  const running = !!e && e.state === 'running' && e.seen === o.timestamp; // a running entry no longer reported is stale
  const fin = !!e && e.state !== 'running' && !TK.dismissed.has(e.aid);
  const recent = fin && o.timestamp - (e.finished_at ?? o.timestamp) <= 10;
  const sticky = fin && (e.state === 'failed' || e.state === 'cancelled');
  const k = TK.k = { e, other, running, recent, sticky, show: other != null || running || recent || sticky,
    // a lease held without a running action: a servo stream when the arm is being commanded, else an idle holder,
    // unless the holder's own action just ended
    mode: running || other == null || (recent && e.src === other) ? 'act' : streaming ? 'servo' : 'idle' };
  const ml = $('ml-chip');
  k.ml = ml && !ml.hidden ? ml.textContent : '';
  k.pill = tkPill();
  const key = [k.show, k.mode, TK.ver, other, running, recent, sticky, k.ml, k.pill, LANG].join('|');
  if (key !== TK.key) {
    TK.key = key;
    tkRenderBanner(k);
    renderActionChip(k);
    const b = $('btn-task'), live = running || other != null;
    b.classList.toggle('is-live', live || sticky);
    b.classList.toggle('warn', !live && sticky);
  }
  if (!$('task').hidden) { tkRenderList(); if (TK.tab === 'env') tkRenderEnv(); }
}
function tkTagEl(text, cls) {
  const s = document.createElement('span');
  s.className = 'ext-tag' + (cls ? ' ' + cls : '');
  s.textContent = text;
  return s;
}
function tkRenderBanner(k) {
  const el = $('ext-banner');
  if (!k.show) { el.hidden = true; return; }
  const e = k.e, act = k.mode === 'act' && !!e;
  const bad = act && !k.running && (e.state === 'failed' || e.state === 'cancelled');
  el.hidden = false;
  el.classList.toggle('live', k.running || k.mode === 'servo');
  el.classList.toggle('bad', bad);
  el.classList.toggle('warn', !bad && !!k.ml);
  setText($('ext-who'), k.other != null ? leaseHolderText(k.other) : tkWho(e && e.src));
  $('ext-pill').hidden = !k.pill;
  setText($('ext-pill'), k.pill);
  $('ext-dismiss').hidden = k.running || k.other != null;
  const tags = $('ext-tags');
  tags.textContent = '';
  const prog = $('ext-prog'), err = $('ext-err'), hint = $('ext-hint');
  if (act) {
    setText($('ext-step'), tkLabel(e));
    tags.appendChild(tkTagEl(tkSt(e.state), e.state === 'done' ? 'ok' : e.state === 'running' ? 'warn' : ''));
    if (e.tag && e.kind) tags.appendChild(tkTagEl(e.kind));
    if (e.phase) tags.appendChild(tkTagEl(t('ext.phase', { p: e.phase })));
    if (e.holding === true) tags.appendChild(tkTagEl(t('ext.holding.yes'), 'ok'));
    else if (e.holding === false) tags.appendChild(tkTagEl(t('ext.holding.no'), 'warn'));
    const d = tkDur(e); if (d) tags.appendChild(tkTagEl(d));
    prog.hidden = bad;
    const pct = e.state === 'done' ? 100 : tkPct(e);
    $('ext-prog-bar').style.width = pct + '%';
    prog.setAttribute('aria-valuenow', String(pct));
    err.hidden = !bad;
    setText($('ext-err-code'), bad ? (e.code || e.state) : '');
    setText($('ext-err-msg'), bad ? (e.message || '') : '');
  } else {
    setText($('ext-step'), t(k.mode === 'servo' ? 'ext.servo' : 'ext.idle'));
    prog.hidden = true; err.hidden = true;
  }
  hint.hidden = !k.ml;
  setText(hint, k.ml ? t('ext.ml', { m: k.ml }) : '');
  // compact feed: the 3 newest external actions
  const feed = $('ext-feed');
  feed.textContent = '';
  let n = 0;
  for (let i = TK.log.length - 1; i >= 0 && n < 3; i--) {
    const x = TK.log[i];
    if (x.type !== 'action' || x.own) continue;
    n++;
    const li = document.createElement('li');
    const ic = document.createElement('span'); ic.className = 'st-ic st-' + x.state; ic.textContent = TK_ICON[x.state] || '·';
    ic.title = tkSt(x.state);
    const lb = document.createElement('span'); lb.className = 'lbl'; lb.textContent = tkLabel(x);
    const du = document.createElement('span'); du.className = 'mono'; du.textContent = tkDur(x);
    li.append(ic, lb, du);
    const extra = x.state === 'failed' || x.state === 'cancelled' ? (x.code || '')
      : x.holding === true ? t('ext.holding.yes') : x.holding === false ? t('ext.holding.no') : '';
    if (extra) { const ex = document.createElement('span'); ex.className = 'mono'; ex.textContent = extra; li.appendChild(ex); }
    feed.appendChild(li);
  }
}
// HUD chip: compact summary of the external controller; opens #task (Enter / click; Space = E-stop, Escape = Stop).
function renderActionChip(k) {
  const el = $('act-chip');
  if (!el) return;
  el.hidden = !k.show;
  el.classList.toggle('ext', k.show);
  // #act-chip is a button (opens #task); #act-live announces its summary without the progress percentage
  const live = $('act-live');
  if (!k.show) { setText(live, ''); return; }
  const e = k.e;
  const who = tkWhoShort(k.other != null ? k.other : e && e.src);
  let text, cls;
  if (k.mode === 'act' && e) {
    const lbl = tkLabel(e);
    if (e.state === 'running') { text = who + ' · ' + lbl + ' ' + tkPct(e) + '%'; cls = 'is-warn'; }
    else if (e.state === 'done') { text = who + ' · ' + lbl + ' ✓'; cls = 'is-ok'; }
    else { text = who + ' · ' + lbl + ' ✕' + (e.code ? ' ' + e.code : ''); cls = 'is-danger'; }
  } else if (k.mode === 'servo') { text = who + ' · ' + t('ext.servo.s'); cls = 'is-warn'; }
  else { text = who + ' · ' + t('ext.idle.s'); cls = ''; }
  setText(el, text);
  el.setAttribute('aria-label', t('ext.chip.aria', { s: text }));
  setText(live, k.mode === 'act' && e && e.state === 'running' ? who + ' · ' + tkLabel(e) + ' · ' + tkSt(e.state) : text);
  el.classList.toggle('is-warn', cls === 'is-warn');
  el.classList.toggle('is-ok', cls === 'is-ok');
  el.classList.toggle('is-danger', cls === 'is-danger');
}
function tkRerender() { TK.key = ''; TK.listKey = ''; TK.envKey = ''; if (S.obs) tkOnObs(S.obs); }

/* ---------- #task drawer ---------- */
function tkLine(e) {
  const parts = [tkHms(e.at), e.src == null ? '-' : e.src];
  if (e.type === 'lease') parts.push('lease', e.ev);
  else {
    parts.push(e.state || '-', [e.tag, e.kind, e.phase].filter(Boolean).join('/') || '-', tkDur(e) || '-');
    if (e.holding != null) parts.push('holding=' + e.holding);
    if (e.code) parts.push(e.code);
    if (e.message) parts.push(JSON.stringify(e.message));
  }
  return parts.join(' ');
}
const tkVisible = () => TK.log.filter((e) => !TK.filterExt || !e.own);
function tkLi(e) {
  const li = document.createElement('li');
  const span = (cls, txt) => { const s = document.createElement('span'); if (cls) s.className = cls; s.textContent = txt; return s; };
  li.appendChild(span('mono', tkHms(e.at)));
  if (e.type === 'lease') {
    li.append(span('st-ic st-lease', '⇄'), span('who', tkWho(e.src)), span('lbl', t(e.ev === 'acquire' ? 'task.lease.acquire' : 'task.lease.release')));
    return li;
  }
  const ic = span('st-ic st-' + e.state, TK_ICON[e.state] || '·');
  li.append(ic, span('who', tkWho(e.src)), span('lbl', [e.tag, e.kind, e.phase].filter(Boolean).join(' · ') || '--'));
  li.appendChild(tkTagEl(tkSt(e.state), e.state === 'done' ? 'ok' : e.state === 'running' ? 'warn' : ''));
  if (e.holding === true) li.appendChild(tkTagEl(t('ext.holding.yes'), 'ok'));
  else if (e.holding === false) li.appendChild(tkTagEl(t('ext.holding.no'), 'warn'));
  const d = tkDur(e); if (d) li.appendChild(span('mono', d));
  if (e.state === 'failed' || e.state === 'cancelled') {
    li.className = 'bad';
    if (e.code || e.message) li.appendChild(span('sub mono', [e.code, e.message].filter(Boolean).join(': ')));
  }
  return li;
}
function tkRenderList() {
  if ($('task').hidden || TK.tab !== 'act') return;
  const key = [TK.ver, TK.filterExt, LANG].join('|');
  if (key === TK.listKey) return;
  TK.listKey = key;
  const list = $('task-list'), items = tkVisible();
  list.textContent = '';
  for (let i = items.length - 1; i >= 0; i--) list.appendChild(tkLi(items[i]));
  list.hidden = !items.length;
  $('task-empty').hidden = items.length > 0;
  for (const b of $('task-filter').children) {
    const on = (b.dataset.f === 'ext') === TK.filterExt;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  }
}
async function tkGet(path) {
  const ac = new AbortController(), tm = setTimeout(() => ac.abort(), 3000);
  try {
    const r = await fetch(path, { signal: ac.signal, cache: 'no-store' });
    if (r.status === 404 || r.status === 405 || r.status === 501) return { na: true };
    if (!r.ok) return { err: 'HTTP ' + r.status };
    return { v: await r.json() };
  } catch (e) { return { err: e && e.name === 'AbortError' ? 'timeout' : 'error' }; }
  finally { clearTimeout(tm); }
}
async function tkEnvLoad() {
  if (TK.inflight || $('task').hidden) return;
  TK.inflight = true;
  try {
    const [catalog, state, keepout, poses] = await Promise.all(
      ['/api/v1/sandbox/catalog', '/api/v1/sandbox/state', '/api/v1/workspace/keepout', '/api/v1/poses'].map(tkGet));
    TK.env = { catalog, state, keepout, poses };
    TK.envAt = Date.now();
  } finally { TK.inflight = false; }
  TK.key = ''; // the banner pill reads TK.env
  if (S.obs) tkOnObs(S.obs);
  tkRenderEnv();
}
function tkEnvRows() {
  const E = TK.env, o = S.obs || {};
  const none = t('task.env.none');
  const val = (x) => (x == null || x === '' ? none : String(x));
  const rows = [['h', t('task.env.sec.sandbox')]];
  const sb = E && ((E.state && E.state.v) || (E.catalog && E.catalog.v));
  const daemonToolId = S.activeTool && S.activeTool.id;
  const sbToolId = sb && sb.active_tool;
  const toolCommReq = !!(S.activeTool && S.activeTool.commissioning_required === true);
  if (!E) rows.push(['r', t('task.env.mode'), t('task.env.loading')]);
  else if (!sb) {
    rows.push(['r', t('task.env.mode'), E.state.na && E.catalog.na ? t('task.env.na') : '--']);
    if (daemonToolId) rows.push(['r', t('task.env.daemon_tool'), val(daemonToolId)]);
    if (toolCommReq) rows.push(['r', t('task.env.tool_comm'), t('tool.comm.hint'), 'warn']);
  } else {
    rows.push(['r', t('task.env.mode'), sb.mode && sb.mode !== 'none' ? String(sb.mode) : none]);
    rows.push(['r', t('task.env.scene'), val(sb.active_scene)]);
    rows.push(['r', t('task.env.tool'), val(sbToolId)]);
    if (daemonToolId && daemonToolId !== sbToolId) rows.push(['r', t('task.env.daemon_tool'), val(daemonToolId)]);
    if (toolCommReq) rows.push(['r', t('task.env.tool_comm'), t('tool.comm.hint'), 'warn']);
    rows.push(['r', t('task.env.policy'), val(sb.active_policy)]);
    const ps = sb.policy_status ?? (sb.telemetry && typeof sb.telemetry === 'object' ? sb.telemetry.policy_status : undefined);
    rows.push(['r', t('task.env.pstatus'), ps && typeof ps === 'object' ? JSON.stringify(ps) : val(ps)]);
    const pmVis = (sb.policy_manifest && sb.policy_manifest.vision) || (typeof WASM_SIM !== 'undefined' && WASM_SIM.bundle && WASM_SIM.bundle.policy_manifest && WASM_SIM.bundle.policy_manifest.vision);
    if (pmVis && pmVis.source === 'video_embedding') {
      const dim = pmVis.embedding_dim != null && pmVis.history_len != null ? ` (${pmVis.embedding_dim}×${pmVis.history_len})` : '';
      rows.push(['r', t('task.env.pvision'), 'video_embedding' + dim]);
    }
  }
  rows.push(['h', t('task.env.sec.safety')]);
  const st = parseState(o.safety_state);
  rows.push(['r', t('task.env.safety'), st.name ? (st.detail ? st.name + ' · ' + st.detail : st.name) : '--', /fault|estop|stop/i.test(st.name || '') ? 'bad' : '']);
  const col = o.collision;
  rows.push(col && col.active ? ['r', t('task.env.col'), t('task.env.col.hold', { j: col.joint ?? '?' }), 'bad']
    : ['r', t('task.env.col'), col ? t('task.env.col.ok') : none]);
  const ml = $('ml-chip');
  const mlt = ml && !ml.hidden ? ml.textContent : (o.motion_limit && o.motion_limit.reason ? String(o.motion_limit.reason) : '');
  rows.push(['r', t('task.env.ml'), mlt || none, mlt ? 'warn' : '']);
  rows.push(['h', t('task.env.sec.cell')]);
  const cnt = (r, list, names) => !E ? t('task.env.loading') : r.na ? t('task.env.na') : r.err ? '--'
    : t('task.env.count', { n: list.length }) + (names && list.length ? ' · ' + list.map((b) => (b && b.name != null ? String(b.name) : '?')).join(', ') : '');
  const ko = E && E.keepout.v && Array.isArray(E.keepout.v.boxes) ? E.keepout.v.boxes : [];
  const ps = E && E.poses.v && Array.isArray(E.poses.v.poses) ? E.poses.v.poses : [];
  rows.push(['r', t('task.env.ko'), E ? cnt(E.keepout, ko, true) : t('task.env.loading')]);
  rows.push(['r', t('task.env.poses'), E ? cnt(E.poses, ps, false) : t('task.env.loading')]);
  return rows;
}
function tkRenderEnv() {
  if ($('task').hidden || TK.tab !== 'env') return;
  const rows = tkEnvRows(), E = TK.env;
  const errs = E ? Object.entries(E).filter(([, r]) => r.err).map(([n, r]) => n + ' ' + r.err) : [];
  const status = !E ? t('task.env.loading') : t('task.env.updated', { t: tkHms(TK.envAt) }) + (errs.length ? ' · ' + t('task.env.err', { list: errs.join(', ') }) : '');
  const posesOk = !!(E && E.poses.v) && !$('btn-pset').hidden;
  const key = JSON.stringify(rows) + '|' + status + '|' + posesOk;
  if (key === TK.envKey) return;
  TK.envKey = key;
  const body = $('task-env-body');
  body.textContent = '';
  for (const [kind, a, b, cls] of rows) {
    if (kind === 'h') { const h = document.createElement('h4'); h.textContent = a; body.appendChild(h); continue; }
    const row = document.createElement('div');
    row.className = 'row' + (cls ? ' ' + cls : '');
    const ka = document.createElement('span'); ka.textContent = a;
    const vb = document.createElement('span'); vb.textContent = b;
    row.append(ka, vb);
    body.appendChild(row);
  }
  setText($('task-env-status'), status);
  $('task-open-poses').disabled = !posesOk;
}
function tkTab(tab, focus) {
  TK.tab = tab === 'env' ? 'env' : 'act';
  for (const b of $('task-tabs').children) {
    const on = b.dataset.tab === TK.tab;
    b.classList.toggle('on', on); b.setAttribute('aria-selected', String(on)); b.tabIndex = on ? 0 : -1;
    if (on && focus) b.focus();
  }
  $('task-act').hidden = TK.tab !== 'act';
  $('task-env').hidden = TK.tab !== 'env';
  TK.listKey = ''; TK.envKey = '';
  tkRenderList(); tkRenderEnv();
}
// the 2 s refresh runs only while #task is open and the browser tab is visible
function tkTimer() {
  clearInterval(TK.timer);
  TK.timer = !$('task').hidden && !document.hidden ? setInterval(tkEnvLoad, 2000) : 0;
}
document.addEventListener('visibilitychange', () => { tkTimer(); if (TK.timer) tkEnvLoad(); });
function taskOpen(open, keepFocus) {
  if (open && $('task').hidden) TK.opener = document.activeElement;
  $('task').hidden = !open;
  tkTimer();
  if (open) {
    tkTab(TK.tab);
    tkEnvLoad();
    $('task-close').focus({ preventScroll: true });
    return;
  }
  if (keepFocus) return;
  const back = TK.opener && TK.opener.isConnected && TK.opener.offsetParent !== null ? TK.opener : $('btn-task');
  TK.opener = null;
  back.focus({ preventScroll: true });
}
async function tkCopy() {
  const txt = tkVisible().map(tkLine).join('\n');
  let ok = false;
  try { if (navigator.clipboard && window.isSecureContext) { await navigator.clipboard.writeText(txt); ok = true; } } catch (_) {}
  if (!ok) {
    const ta = document.createElement('textarea');
    ta.value = txt; ta.setAttribute('readonly', '');
    ta.style.cssText = 'position:fixed;top:0;left:0;width:1px;height:1px;opacity:0;-webkit-user-select:text;user-select:text;';
    document.body.appendChild(ta); ta.select();
    try { ok = document.execCommand('copy'); } catch (_) {}
    ta.remove();
    $('task-copy').focus({ preventScroll: true });
  }
  toast(t(ok ? 'task.copied' : 'task.copy_fail'), ok ? 'ok' : 'err');
}
$('btn-task').addEventListener('click', () => taskOpen(true));
$('ext-open-task').addEventListener('click', () => taskOpen(true));
$('act-chip').addEventListener('click', () => taskOpen(true));
// Enter only: Space = E-stop and Escape = Stop in the window keydown handler
$('act-chip').addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); taskOpen(true); } });
$('ext-dismiss').addEventListener('click', () => {
  if (!TK.lastExt) return;
  TK.dismissed.add(TK.lastExt.aid);
  if (TK.dismissed.size > 64) TK.dismissed.delete(TK.dismissed.values().next().value);
  tkRerender();
});
$('task-close').addEventListener('click', () => taskOpen(false));
$('task').addEventListener('click', (e) => { if (e.target === e.currentTarget) taskOpen(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('task').hidden) taskOpen(false); });
$('task-tabs').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) tkTab(b.dataset.tab); });
$('task-tabs').addEventListener('keydown', (e) => {
  if (e.key === 'ArrowLeft' || e.key === 'ArrowRight' || e.key === 'Home' || e.key === 'End') { e.preventDefault(); tkTab(TK.tab === 'act' ? 'env' : 'act', true); }
});
$('task-filter').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  TK.filterExt = b.dataset.f === 'ext';
  store('rebot.task.f', TK.filterExt ? 'ext' : 'all');
  tkRenderList();
});
$('task-copy').addEventListener('click', tkCopy);
$('task-clear').addEventListener('click', () => { TK.log = []; TK.ver++; tkRerender(); tkRenderList(); });
$('task-open-poses').addEventListener('click', () => { taskOpen(false, true); psetOpen(true); });
i18nHooks.push(() => { tkRerender(); tkRenderList(); tkRenderEnv(); });

