/* =================================================================
   Live video: camera WS H.264 (JMuxer), or ?video= MJPEG
   ================================================================= */
const videoEl = $('video-stream');
const mjpegEl = $('video-mjpeg');
const V = {
  gen: 0, mode: null, playing: false,
  ws: null, jmx: null, wsWatch: null, wsRetries: 0, retryTimer: null, retryDelay: 4000,
  frames: 0, lastTotal: 0, lastTime: 0, fps: null, lat: null, stall: 0,
};
const MODE_LABEL = { ws: 'WS·H.264', mjpeg: 'MJPEG' };
// ws mode thresholds, same as the supervisor console (useDebugStream.ts).
const WS_LIVE_EDGE_CHECK_MS = 500;  // live-edge keeper period
const WS_LIVE_EDGE_MAX_LAG = 0.6;   // s behind buffered end before snapping
const WS_LIVE_EDGE_TARGET = 0.15;   // s from the edge to land on after a snap
const WS_RECONNECT_GRACE = 3;       // ws-only retries after a working ws stream drops
const WS_RECONNECT_MAX_MS = 5000;   // backoff: min(1500 * n, max)
const WS_FRAME_GAP_MS = 3000;       // no frame for this long on a live ws stream = dropped
const WS_DEFAULT_FRAME_MS = 33;     // first frame, before a timestamp delta exists
const WS_MIN_FRAME_MS = 10;         // clamp for the timestamp-derived frame duration
const WS_MAX_FRAME_MS = 200;

let lastHint = null;
function videoHint(kind, title, sub) {
  lastHint = [kind, title, sub];
  const h = $('vhint');
  h.classList.toggle('hide', kind === 'hide');
  h.classList.toggle('connecting', kind === 'connecting');
  const r = (x) => typeof x === 'function' ? x() : x;
  if (title) $('vhint-title').textContent = r(title);
  $('vhint-sub').textContent = r(sub) || '';
}
function videoTeardown() {
  V.gen++;
  V.playing = false; V.mode = null; V.fps = null; V.lat = null; V.stall = 0;
  clearTimeout(V.retryTimer);
  if (V.ws) { try { V.ws.onclose = null; V.ws.close(); } catch (_) {} V.ws = null; }
  wsPlayerClose();
  try { videoEl.pause(); } catch (_) {}
  videoEl.srcObject = null;
  if (videoEl.getAttribute('src')) { videoEl.removeAttribute('src'); try { videoEl.load(); } catch (_) {} }
  V.lastTotal = 0;
  V.lastTime = 0;
  if (rvfcHandle != null && videoEl.cancelVideoFrameCallback) {
    try { videoEl.cancelVideoFrameCallback(rvfcHandle); } catch (_) {}
    rvfcHandle = null;
  }
}
// One attempt = resolve on the first 'playing', reject on error/timeout.
// `fail` is also used after success to report a dropped stream.
function attempt(kind, timeoutMs, setup) {
  const gen = V.gen;
  return new Promise((resolve, reject) => {
    let settled = false, ok = false;
    const onPlay = () => { ok = true; finish(); };
    const finish = (err) => {
      if (settled) return;
      settled = true; clearTimeout(timer);
      videoEl.removeEventListener('playing', onPlay);
      if (err) reject(err); else resolve();
    };
    const timer = setTimeout(() => finish(new Error(kind + ' timeout')), timeoutMs);
    videoEl.addEventListener('playing', onPlay);
    const fail = (err) => {
      if (gen !== V.gen) return;
      if (!settled) finish(err || new Error(kind + ' failed'));
      else if (ok) videoDropped(); // a rejected attempt never tears down a later one
    };
    try { setup(fail, gen); } catch (e) { finish(e); }
  });
}
function playVideo() { const p = videoEl.play(); if (p && p.catch) p.catch(() => {}); }

// Camera app debug_stream: each binary message is one H.264 Annex-B access unit
// (SPS+PPS prepended on keyframes) followed by an 8-byte little-endian unix-ms
// timestamp, which is stripped before feeding JMuxer.
function wsPlayerClose() {
  clearInterval(V.wsWatch); V.wsWatch = null;
  if (V.jmx) { try { V.jmx.destroy(); } catch (_) {} V.jmx = null; }
}
function setupWS(fail, gen) {
  if (!window.JMuxer) throw new Error('JMuxer not loaded');
  if (!(window.MediaSource || window.ManagedMediaSource)) throw new Error('MSE unsupported');
  V.jmx = new JMuxer({ node: videoEl, mode: 'video', flushingTime: 0, fps: 30, clearBuffer: true, debug: false });
  const ws = new WebSocket(CFG.video.ws);
  ws.binaryType = 'arraybuffer';
  V.ws = ws;
  let lastFrame = 0;
  let lastTs = null;
  ws.onmessage = (ev) => {
    if (gen !== V.gen || !(ev.data instanceof ArrayBuffer) || ev.data.byteLength <= 8) return;
    lastFrame = performance.now();
    // Frame duration from the device timestamps: the sensor may run below the
    // nominal 30 fps (ISP at 10 fps), and a fixed 1/30 s would make playback
    // drift to a third of real time between live-edge snaps.
    const n = ev.data.byteLength - 8;
    const ts = Number(new DataView(ev.data, n, 8).getBigUint64(0, true));
    const duration = lastTs === null ? WS_DEFAULT_FRAME_MS : Math.min(WS_MAX_FRAME_MS, Math.max(WS_MIN_FRAME_MS, ts - lastTs));
    lastTs = ts;
    if (V.jmx) V.jmx.feed({ video: new Uint8Array(ev.data, 0, n), duration });
  };
  ws.onerror = () => fail(new Error('ws unreachable'));
  ws.onclose = () => fail(new Error('ws closed'));
  V.wsWatch = setInterval(() => {
    if (gen !== V.gen) return;
    const now = performance.now();
    if (lastFrame && now - lastFrame > WS_FRAME_GAP_MS) { fail(new Error('ws frame gap')); return; }
    // Live-edge keeper: MSE buffering lets the playhead drift behind; snap back.
    if (videoEl.seeking || !videoEl.buffered || !videoEl.buffered.length) return;
    const end = videoEl.buffered.end(videoEl.buffered.length - 1);
    if (end - videoEl.currentTime > WS_LIVE_EDGE_MAX_LAG) {
      try { videoEl.currentTime = end - WS_LIVE_EDGE_TARGET; } catch (_) {}
    }
  }, WS_LIVE_EDGE_CHECK_MS);
  playVideo();
}

// wsOnly: reconnect after a working ws stream dropped (shows the "dropped" hint and
// the retry count); otherwise a failed attempt falls back to the backoff retry.
async function videoStart(wsOnly = false) {
  videoTeardown();
  const gen = V.gen;
  if (CFG.video.mjpeg) return startMJPEG();
  const retry = wsOnly ? ` · ${V.wsRetries}/${WS_RECONNECT_GRACE}` : '';
  videoHint('connecting', () => t(wsOnly ? 'v.dropped' : 'v.connecting'), `${MODE_LABEL.ws} · ${CFG.video.ws}${retry}`);
  try {
    // 6 s: at 10 fps MSE takes ~3 s from the first IDR to 'playing'.
    await attempt('ws', 6000, setupWS);
    if (gen !== V.gen) return;
    V.mode = 'ws'; V.playing = true; V.retryDelay = 4000; V.wsRetries = 0;
    videoHint('hide');
    return;
  } catch (_) {
    if (gen !== V.gen) return;
    if (V.ws) { try { V.ws.onclose = null; V.ws.close(); } catch (_) {} V.ws = null; }
    wsPlayerClose();
    if (wsOnly) { videoWsRetry(() => videoStart()); return; }
  }
  scheduleVideoRetry(() => t('v.noservice'));
}
function scheduleVideoRetry(title) {
  const gen = V.gen;
  const sec = Math.round(V.retryDelay / 1000);
  videoHint('idle', title, () => t('v.retry_sub', { sec }));
  clearTimeout(V.retryTimer);
  V.retryTimer = setTimeout(() => { if (gen === V.gen) videoStart(); }, V.retryDelay);
  V.retryDelay = Math.min(30000, V.retryDelay * 2);
}
// After a working ws stream drops: retry ws quickly a few times, then hand over to
// `exhausted` (one more attempt, then the slower scheduleVideoRetry backoff).
function videoWsRetry(exhausted) {
  if (V.wsRetries >= WS_RECONNECT_GRACE) { V.wsRetries = 0; exhausted(); return; }
  V.wsRetries++;
  const gen = V.gen;
  videoHint('connecting', () => t('v.dropped'), `${MODE_LABEL.ws} · ${CFG.video.ws} · ${V.wsRetries}/${WS_RECONNECT_GRACE}`);
  clearTimeout(V.retryTimer);
  V.retryTimer = setTimeout(() => { if (gen === V.gen) videoStart(true); }, Math.min(1500 * V.wsRetries, WS_RECONNECT_MAX_MS));
}
function videoDropped() {
  if (!V.playing) return;
  const wasWS = V.mode === 'ws';
  videoTeardown();
  if (wasWS) { videoWsRetry(() => videoStart()); return; }
  scheduleVideoRetry(() => t('v.dropped'));
}
function startMJPEG() {
  const gen = V.gen;
  videoHint('connecting', () => t('v.connecting'), 'MJPEG · ' + CFG.video.mjpeg);
  mjpegEl.style.display = 'block';
  videoEl.style.display = 'none';
  mjpegEl.onload = () => { if (gen !== V.gen) return; V.mode = 'mjpeg'; V.playing = true; videoHint('hide'); };
  mjpegEl.onerror = () => {
    if (gen !== V.gen) return;
    V.playing = false;
    videoHint('idle', () => t('v.cant_open'), 'MJPEG · ' + CFG.video.mjpeg);
    clearTimeout(V.retryTimer);
    V.retryTimer = setTimeout(() => { if (gen === V.gen) mjpegEl.src = CFG.video.mjpeg + (CFG.video.mjpeg.includes('?') ? '&' : '?') + '_=' + Date.now(); }, 5000);
  };
  mjpegEl.src = CFG.video.mjpeg;
}
$('vhint-retry').addEventListener('click', () => { V.retryDelay = 2000; if (CFG.video.mjpeg) { mjpegEl.src = CFG.video.mjpeg; } else videoStart(); });

// frame counter & stall detector
let rvfcHandle = null;
function armRVFC() {
  if (!('requestVideoFrameCallback' in HTMLVideoElement.prototype)) return;
  if (rvfcHandle != null && videoEl.cancelVideoFrameCallback) {
    try { videoEl.cancelVideoFrameCallback(rvfcHandle); } catch (_) {}
    rvfcHandle = null;
  }
  const cb = () => {
    V.frames++;
    if (V.playing) {
      rvfcHandle = videoEl.requestVideoFrameCallback(cb);
    } else {
      rvfcHandle = null;
    }
  };
  rvfcHandle = videoEl.requestVideoFrameCallback(cb);
}
videoEl.addEventListener('playing', () => {
  V.stall = 0;
  V.lastTime = videoEl.currentTime;
  armRVFC();
});

setInterval(() => {
  if (!V.playing || V.mode === 'mjpeg') return;
  let n = 0;
  if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
    n = V.frames;
    V.frames = 0;
  }
  if (videoEl.getVideoPlaybackQuality) {
    const tot = videoEl.getVideoPlaybackQuality().totalVideoFrames;
    if (typeof tot === 'number') {
      const diff = Math.max(0, tot - V.lastTotal);
      V.lastTotal = tot;
      if (n === 0) n = diff;
    }
  }
  const curTime = videoEl.currentTime;
  const timeProgress = curTime > (V.lastTime || 0);
  V.lastTime = curTime;
  V.fps = n;
  if (!document.hidden) {
    V.stall = (n === 0 && !timeProgress) ? V.stall + 1 : 0;
    if (V.stall >= 10) { videoDropped(); return; }
  }
  if (videoEl.buffered && videoEl.buffered.length) {
    const end = videoEl.buffered.end(videoEl.buffered.length - 1);
    // ws mode keeps the live edge with its own 500 ms keeper in setupWS.
    V.lat = Math.max(0, end - videoEl.currentTime) * 1000;
  }
}, 1000);

/* =================================================================
   Detection Overlay (CSS logical coordinates + Letterbox mapping)
   ================================================================= */
const ov = { cv: $('video-overlay'), ctx: null };
if (ov.cv) ov.ctx = ov.cv.getContext('2d');

const PALETTE = [
  '#10b981', // emerald green
  '#f59e0b', // amber yellow
  '#0ea5e9', // sky blue
  '#ef4444', // red / orange
  '#8b5cf6', // violet
  '#ec4899', // pink
  '#14b8a6', // teal
  '#6366f1', // indigo
];

function getLabelColor(label) {
  const str = String(label != null ? label : 'obj');
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    hash = (hash * 31 + str.charCodeAt(i)) & 0xffffffff;
  }
  return PALETTE[Math.abs(hash) % PALETTE.length];
}

function clearOverlayCanvas() {
  if (ov.cv && ov.ctx) {
    ov.ctx.setTransform(1, 0, 0, 1, 0, 0);
    ov.ctx.clearRect(0, 0, ov.cv.width, ov.cv.height);
  }
}

function fitCanvas(c, box) {
  const rect = box.getBoundingClientRect();
  const cssW = rect.width;
  const cssH = rect.height;
  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const bw = Math.max(1, Math.round(cssW * dpr));
  const bh = Math.max(1, Math.round(cssH * dpr));
  if (c.width !== bw || c.height !== bh) {
    c.width = bw;
    c.height = bh;
  }
  const ctx = c.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { cssW, cssH, w: bw, h: bh, dpr };
}

function getVideoContentRect(cssW, cssH) {
  const srcW = V.mode === 'mjpeg' ? (mjpegEl ? mjpegEl.naturalWidth : 0) : (videoEl ? videoEl.videoWidth : 0);
  const srcH = V.mode === 'mjpeg' ? (mjpegEl ? mjpegEl.naturalHeight : 0) : (videoEl ? videoEl.videoHeight : 0);
  // 源未解码就绪保护：绝不拉伸绘制
  if (!srcW || !srcH) return null;
  if (!cssW || !cssH || cssW <= 0 || cssH <= 0) return null;
  const scale = Math.min(cssW / srcW, cssH / srcH);
  const renderW = srcW * scale;
  const renderH = srcH * scale;
  const offsetX = (cssW - renderW) / 2;
  const offsetY = (cssH - renderH) / 2;
  return { x: offsetX, y: offsetY, w: renderW, h: renderH };
}

function drawOverlay() {
  const { cssW, cssH, dpr } = fitCanvas(ov.cv, $('video-box'));
  const g = ov.ctx;
  if (cssW <= 0 || cssH <= 0) return;

  // A single-shot result is held on screen (S.detHoldUntil) instead of fading after 400 ms.
  const now = performance.now();
  const held = now < (S.detHoldUntil || 0);
  const age = held ? 0 : S.detLastReceivedAt ? (now - S.detLastReceivedAt) : Infinity;
  if (!S.detections || S.detections.length === 0 || age > 400 || !isFinite(age)) {
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, cssW, cssH);
    ARU.items = []; ARU.layout = null;
    const r = getVideoContentRect(cssW, cssH);
    if (r) graspDraw(g, r, false);
    return;
  }

  const rect = getVideoContentRect(cssW, cssH);
  if (!rect) {
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, cssW, cssH);
    return;
  }

  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, cssW, cssH);

  // 250ms ~ 400ms 期间平滑淡出
  const alpha = age < 250 ? 1.0 : Math.max(0, 1.0 - (age - 250) / 150);

  for (const d of S.detections) {
    const bx = rect.x + d.x0 * rect.w;
    const by = rect.y + d.y0 * rect.h;
    const bw = d.w * rect.w;
    const bh = d.h * rect.h;
    const color = getLabelColor(d.label);
    if (d.aruco) continue; // drawn together after the boxes (shared label layout)

    g.save();
    g.globalAlpha = alpha;

    // 1. 目标边界框与半透明轻微底色
    if (d.obbCorners && Array.isArray(d.obbCorners) && d.obbCorners.length === 4) {
      g.strokeStyle = color;
      g.lineWidth = 2;
      g.beginPath();
      g.moveTo(rect.x + d.obbCorners[0][0] * rect.w, rect.y + d.obbCorners[0][1] * rect.h);
      for (let k = 1; k < 4; ++k) {
        g.lineTo(rect.x + d.obbCorners[k][0] * rect.w, rect.y + d.obbCorners[k][1] * rect.h);
      }
      g.closePath();
      g.stroke();

      g.fillStyle = color;
      g.globalAlpha = alpha * 0.08;
      g.beginPath();
      g.moveTo(rect.x + d.obbCorners[0][0] * rect.w, rect.y + d.obbCorners[0][1] * rect.h);
      for (let k = 1; k < 4; ++k) {
        g.lineTo(rect.x + d.obbCorners[k][0] * rect.w, rect.y + d.obbCorners[k][1] * rect.h);
      }
      g.closePath();
      g.fill();
      g.globalAlpha = alpha;
    } else if (showOpt('box')) {
      g.strokeStyle = color;
      g.lineWidth = 2;
      g.strokeRect(bx, by, bw, bh);

      g.fillStyle = color;
      g.globalAlpha = alpha * 0.08;
      g.fillRect(bx, by, bw, bh);
      g.globalAlpha = alpha;
    } else {
      // simple mode, boxes off: a faint dashed box (the tap target)
      g.save();
      g.setLineDash([5, 4]); g.lineWidth = 1; g.strokeStyle = 'rgba(255,255,255,0.55)';
      g.strokeRect(bx, by, bw, bh);
      g.restore();
    }
    // segment models: the mask outline (dashed and dimmer while another object is selected)
    if (d.contour && showOpt('contour')) {
      const other = !!GR.sel && !(GR.sel.label === d.label && graspIou(GR.sel.box, [d.x0, d.y0, d.x1, d.y1]) > 0.5);
      g.save();
      contourPath(g, rect, d.contour);
      g.fillStyle = 'rgba(255,255,255,0.10)'; if (!other) g.fill();
      g.setLineDash(other ? [6, 5] : []); g.lineJoin = 'round'; g.lineWidth = 2; g.strokeStyle = other ? 'rgba(255,255,255,0.6)' : 'rgba(255,255,255,0.9)';
      g.stroke();
      g.restore();
    }
    if (!showOpt('label')) { g.restore(); continue; }

    // 2. 目标中心微型十字准星 (半径 4px，带白色中心点与半透明外圈)
    const cx = rect.x + d.cx * rect.w;
    const cy = rect.y + d.cy * rect.h;

    if (S.ui !== 'simple') {
    // 半透明外圈
    g.strokeStyle = 'rgba(255, 255, 255, 0.6)';
    g.lineWidth = 1;
    g.beginPath();
    g.arc(cx, cy, 4, 0, Math.PI * 2);
    g.stroke();

    // 准星十字线
    g.strokeStyle = color;
    g.beginPath();
    g.moveTo(cx - 6, cy); g.lineTo(cx + 6, cy);
    g.moveTo(cx, cy - 6); g.lineTo(cx, cy + 6);
    g.stroke();

    // 中心白点
    g.fillStyle = '#ffffff';
    g.beginPath();
    g.arc(cx, cy, 1.5, 0, Math.PI * 2);
    g.fill();
    }

    // 3. 半透明圆角药丸 Label Badge (高度 18px，内嵌类别名与百分比置信度，贴顶 y < 22px 自动下翻至内部)
    const scorePct = Math.round(d.score * 100);
    const labelText = (d.track_id != null ? `#${d.track_id} ` : '') + `${d.label} ${scorePct}%`;
    g.font = '600 11px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace';

    const textMetrics = g.measureText(labelText);
    const badgeW = textMetrics.width + 12;
    const badgeH = 18;
    const badgeRadius = 4;

    let badgeX = bx;
    let badgeY = (by - rect.y < 22) ? by + 2 : by - badgeH - 2;

    if (badgeX < rect.x) badgeX = rect.x;
    if (badgeX + badgeW > rect.x + rect.w) badgeX = rect.x + rect.w - badgeW;

    g.fillStyle = 'rgba(0, 0, 0, 0.75)';
    if (typeof g.roundRect === 'function') {
      g.beginPath();
      g.roundRect(badgeX, badgeY, badgeW, badgeH, badgeRadius);
      g.fill();
    } else {
      g.fillRect(badgeX, badgeY, badgeW, badgeH);
    }

    g.strokeStyle = color;
    g.lineWidth = 1;
    if (typeof g.roundRect === 'function') {
      g.beginPath();
      g.roundRect(badgeX, badgeY, badgeW, badgeH, badgeRadius);
      g.stroke();
    } else {
      g.strokeRect(badgeX, badgeY, badgeW, badgeH);
    }

    g.fillStyle = '#ffffff';
    g.textBaseline = 'middle';
    g.fillText(labelText, badgeX + 6, badgeY + badgeH / 2);

    g.restore();
  }
  const markers = S.detections.filter((d) => d.aruco);
  if (markers.length) drawArucoAll(g, markers, rect, alpha, cssW, cssH); else { ARU.items = []; ARU.layout = null; }
  graspDraw(g, rect, true);
}

/* ArUco AR overlay: corner polygon, projected XYZ axes, labels.
   Axes use a weak-perspective projection: with the OpenCV camera frame (x right, y down, z forward)
   a marker axis R[:,i] maps to the image direction (R[0][i], R[1][i]); the out-of-plane part foreshortens it.
   With many markers in view (the 12-marker calibration board) one ID / Z / R P Y HUD at a time: the marker under
   the mouse, else the tapped one (tap again for automatic), else the expected marker nearest the image centre. Every
   other marker gets a compact ID tag; tags are placed greedily (slot around the marker that stays in the frame and
   clear of the HUD, the summary and earlier tags), a tag that fits nowhere is skipped. Ids outside the expected set
   (calibration board ids, plus the marker the hand-eye calibration locked) are drawn dimmed with "?" and no pose:
   DICT_4X4_50 decodes look-alikes (a real arm reported 17, 27, 34, 40 next to board ids 0..11). */
const AXIS_COLORS = ['#ef4444', '#22c55e', '#3b82f6'];
const ARU_BOARD_IDS = 12; // calib_wizard.js BOARD_DEFAULT cols x rows (4 x 3): ids 0..11; scripts/calib_wizard_selftest.mjs checks it
const ARU = { pick: null, hover: null, items: [], layout: null, sumAt: null, sumMiss: null };
function aruExpected(id) {
  if (id == null || id === '') return false;
  const n = Number(id);
  if (Number.isInteger(n) && n >= 0 && n < ARU_BOARD_IDS) return true;
  return typeof CAL !== 'undefined' && CAL.markerId != null && String(CAL.markerId) === String(id);
}
// screen geometry of one marker: corners (or null), centre, axis length, bounding box
function aruGeom(d, rect) {
  const a = d.aruco;
  const P = (u, v) => [rect.x + u * rect.w, rect.y + v * rect.h];
  const pts = a.corners ? a.corners.map(([u, v]) => P(u, v)) : null;
  let c = P(d.cx, d.cy), L = Math.max(18, Math.min(d.w * rect.w, d.h * rect.h) * 0.6);
  if (pts) { // centre: diagonal intersection
    const [p0, p1, p2, p3] = pts;
    const den = (p0[0] - p2[0]) * (p1[1] - p3[1]) - (p0[1] - p2[1]) * (p1[0] - p3[0]);
    if (Math.abs(den) > 1e-6) {
      const tt = ((p0[0] - p1[0]) * (p1[1] - p3[1]) - (p0[1] - p1[1]) * (p1[0] - p3[0])) / den;
      c = [p0[0] + tt * (p2[0] - p0[0]), p0[1] + tt * (p2[1] - p0[1])];
    }
    let side = 0;
    for (let i = 0; i < 4; i++) side += Math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1]);
    L = Math.max(18, side / 4 * 0.75);
  }
  const xs = pts ? pts.map((p) => p[0]) : [rect.x + d.x0 * rect.w, rect.x + d.x1 * rect.w];
  const ys = pts ? pts.map((p) => p[1]) : [rect.y + d.y0 * rect.h, rect.y + d.y1 * rect.h];
  return { pts, c, L, box: { x: Math.min(...xs), y: Math.min(...ys), w: Math.max(...xs) - Math.min(...xs), h: Math.max(...ys) - Math.min(...ys) } };
}
const aruHit = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
const ARU_FONT = '600 11px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';
/* Page UI over the picture (CSS px of the overlay canvas): the chips, panels and banner inside the video box (by their
   boxes, read every frame: they take no pointer events and change width with their text) and, on a 32 px grid
   refreshed every 2 s while markers are drawn and on a resize, any other element on top (simple mode's sticks and
   buttons over a full-screen picture; ~15 ms for 1280x800 in headless Chromium). Labels avoid these boxes. */
const ARU_OCC = { at: 0, w: 0, h: 0, rects: [] };
function aruObstacles(cssW, cssH) {
  const box = $('video-box'), r = box.getBoundingClientRect(), rects = [];
  for (const el of box.querySelectorAll('.hud > *, .hud .det-ctl > *, .view-switch, .grasp-bar, .grasp-pick, .depth-panel, .depth-read, .banner')) {
    if (el.classList.contains('det-ctl') || el.closest('[hidden]')) continue;
    const b = el.getBoundingClientRect();
    if (b.width > 0 && b.height > 0) rects.push({ x: b.left - r.left - 4, y: b.top - r.top - 4, w: b.width + 8, h: b.height + 8 });
  }
  const now = performance.now();
  if (!(now - ARU_OCC.at < 2000 && ARU_OCC.w === cssW && ARU_OCC.h === cssH)) {
    const free = new Set([box, ov.cv, $('video-stream'), $('video-mjpeg'), $('depth-overlay'), $('vhint')]), c = 32, grid = [];
    for (let y = c / 2; y < cssH; y += c) {
      let run = null;
      for (let x = c / 2; x < cssW + c; x += c) {
        const px = r.left + x, py = r.top + y, inView = x < cssW && px >= 0 && py >= 0 && px < innerWidth && py < innerHeight;
        const el = inView ? document.elementFromPoint(px, py) : null;
        const busy = x < cssW && (!inView || (el && !free.has(el) && !(el.closest && el.closest('.vhint'))));
        if (busy) { if (run) run.w += c; else run = { x: x - c / 2, y: y - c / 2, w: c, h: c }; } else if (run) { grid.push(run); run = null; }
      }
    }
    Object.assign(ARU_OCC, { at: now, w: cssW, h: cssH, rects: grid });
  }
  return rects.concat(ARU_OCC.rects);
}
// one frame's plan: which marker gets the HUD, where every tag and the summary go
function aruLayout(g, list, rect, cssW, cssH) {
  const top = Math.max(rect.y + 2, 44); // keep clear of the HUD chip row
  const narrow = rect.w < 560;
  const inFrame = (r) => r.x >= rect.x + 2 && r.x + r.w <= rect.x + rect.w - 2 && r.y >= rect.y + 2 && r.y + r.h <= rect.y + rect.h - 2;
  const items = list.map((d) => ({ d, id: d.aruco.id != null ? String(d.aruco.id) : null, ok: aruExpected(d.aruco.id), geo: aruGeom(d, rect), tag: null, hud: null }))
    .filter((it) => [it.geo.box.x, it.geo.box.y, it.geo.box.w, it.geo.box.h].every(Number.isFinite)); // no position: nothing to draw
  if (!items.length) return { items, sel: null, summary: null, skipped: 0 };
  const ui = aruObstacles(cssW, cssH);
  const mid = [rect.x + rect.w / 2, rect.y + rect.h / 2], dist = (it) => Math.hypot(it.geo.c[0] - mid[0], it.geo.c[1] - mid[1]);
  const ok = items.filter((it) => it.ok).sort((p, q) => dist(p) - dist(q));
  const unk = items.filter((it) => !it.ok);
  const by = (k) => (k == null ? null : ok.find((it) => it.id === k) || null);
  const sel = by(ARU.hover) || by(ARU.pick) || ok[0] || null;
  const boxes = items.map((it) => it.geo.box);
  const placed = [];
  const free = (s) => inFrame(s) && !placed.some((p) => aruHit(s, p)) && !ui.some((p) => aruHit(s, p));
  const clearOf = (s, own) => !boxes.some((o) => o !== own && aruHit(s, o));
  g.save();
  g.font = ARU_FONT;
  // full HUD for the selected marker: right, left, above, below; clear of other markers if possible; else clamped
  if (sel) {
    const a = sel.d.aruco, f = narrow ? 0 : 1, lines = [];
    if (narrow) {
      lines.push('ID ' + sel.id + (a.t ? '  Z ' + a.t[2].toFixed(3) + ' m' : ''));
      if (a.rpy) lines.push('R ' + a.rpy[0].toFixed(f) + ' P ' + a.rpy[1].toFixed(f) + ' Y ' + a.rpy[2].toFixed(f) + '°');
    } else {
      lines.push('ID ' + sel.id);
      if (a.t) lines.push('Z ' + a.t[2].toFixed(3) + ' m');
      if (a.rpy) lines.push('R ' + a.rpy[0].toFixed(1) + '° P ' + a.rpy[1].toFixed(1) + '° Y ' + a.rpy[2].toFixed(1) + '°');
    }
    const tw = Math.max(...lines.map((l) => g.measureText(l).width)) + 14, th = lines.length * 15 + 6, b = sel.geo.box, mcx = b.x + b.w / 2, mcy = b.y + b.h / 2, gp = 10;
    const slots = [[b.x + b.w + gp, b.y - 4], [b.x - tw - gp, b.y - 4], [mcx - tw / 2, b.y - th - gp], [mcx - tw / 2, b.y + b.h + gp],
      [b.x + b.w + gp, mcy - th / 2], [b.x - tw - gp, mcy - th / 2], [b.x + b.w + gp, b.y + b.h - th + 4], [b.x - tw - gp, b.y + b.h - th + 4]]
      .map(([x, y]) => ({ x, y, w: tw, h: th }));
    // farther out: beside the whole cluster of markers
    const U = items.reduce((u, it) => { const q = it.geo.box; return { x0: Math.min(u.x0, q.x), y0: Math.min(u.y0, q.y), x1: Math.max(u.x1, q.x + q.w), y1: Math.max(u.y1, q.y + q.h) }; }, { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity });
    slots.push({ x: U.x1 + gp, y: mcy - th / 2, w: tw, h: th }, { x: U.x0 - tw - gp, y: mcy - th / 2, w: tw, h: th }, { x: mcx - tw / 2, y: U.y0 - th - gp, w: tw, h: th }, { x: mcx - tw / 2, y: U.y1 + gp, w: tw, h: th });
    let r = slots.find((s) => free(s) && clearOf(s, b)) || slots.find(free) || slots.find(inFrame) || { ...slots[0] };
    r = { ...r, x: clamp(r.x, rect.x + 2, rect.x + rect.w - tw - 2), y: clamp(r.y, Math.min(top, rect.y + rect.h - th - 2), Math.max(top, rect.y + rect.h - th - 2)) };
    sel.hud = { ...r, lines };
    placed.push(sel.hud);
    // its corner indices (drawn 11 px outside each corner of a marker 40 px or larger)
    if (sel.geo.pts && Math.min(b.w, b.h) >= 40) for (const p of sel.geo.pts) {
      const ox = p[0] - sel.geo.c[0], oy = p[1] - sel.geo.c[1], on = Math.hypot(ox, oy) || 1;
      placed.push({ x: p[0] + ox / on * 11 - 7, y: p[1] + oy / on * 11 - 7, w: 14, h: 14 });
    }
  }
  // summary: board count, look-alike ids, how to pick. The first variant (full, one line, two short lines) and place
  // (below / above / beside the cluster of markers, the picture's corners, then any spot nearest the cluster) that is
  // free and off the markers; else a free spot on markers; else left out
  let summary = null;
  if (items.length > 1 || unk.length) {
    const ids = unk.map((it) => (it.id != null ? it.id : '?')).join(' ');
    const short = [t('aru.sum.s', { n: ok.length }), ...(unk.length ? [t('aru.unk.s', { ids })] : [])];
    const variants = [short.length > 1 ? [short.join(' · ')] : short, ...(short.length > 1 ? [short] : [])];
    if (!narrow) variants.unshift([t('aru.sum', { n: ok.length }), ...(unk.length ? [t('aru.unk', { n: unk.length, ids })] : []), ...(ok.length > 1 ? [t(ARU.pick != null && by(ARU.pick) ? 'aru.picked' : 'aru.tap')] : [])]);
    const U = items.reduce((u, it) => { const q = it.geo.box; return { x0: Math.min(u.x0, q.x), y0: Math.min(u.y0, q.y), x1: Math.max(u.x1, q.x + q.w), y1: Math.max(u.y1, q.y + q.h) }; }, { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity });
    const ucx = (U.x0 + U.x1) / 2, ucy = (U.y0 + U.y1) / 2, R = rect, m = 8;
    const cands = variants.map((lines) => {
      const sw = Math.max(...lines.map((l) => g.measureText(l).width)) + 14, sh = lines.length * 15 + 6;
      return [[ucx - sw / 2, U.y1 + 22], [ucx - sw / 2, U.y0 - sh - 22], [U.x1 + 22, ucy - sh / 2], [U.x0 - sw - 22, ucy - sh / 2],
        [R.x + R.w - sw - m, R.y + m], [R.x + m, R.y + m], [R.x + R.w - sw - m, R.y + R.h - sh - m], [R.x + m, R.y + R.h - sh - m],
        [R.x + R.w / 2 - sw / 2, R.y + m], [R.x + R.w / 2 - sw / 2, R.y + R.h - sh - m]].map(([x, y]) => ({ x, y, w: sw, h: sh, lines }));
    }).flat();
    let r = cands.find((s) => free(s) && clearOf(s, null));
    // else last frame's spot when it is still free (no jumping, no rescan), else any free spot on a 24 px grid,
    // nearest the cluster first
    const last = ARU.sumAt && cands.map((c) => ({ ...c, x: ARU.sumAt.x, y: ARU.sumAt.y })).find((c) => JSON.stringify(c.lines) === ARU.sumAt.key);
    if (!r && last && free(last) && clearOf(last, null) && inFrame(last)) r = last;
    const vkey = JSON.stringify(variants), now = performance.now();
    if (!r && !(ARU.sumMiss && ARU.sumMiss.key === vkey && now - ARU.sumMiss.at < 1000)) { // a failed scan is retried after 1 s
      const scan = [];
      variants.forEach((v, k) => {
        const { w: sw, h: sh } = cands.find((c) => c.lines === v);
        for (let y = R.y + m; y + sh <= R.y + R.h - m; y += 24) for (let x = R.x + m; x + sw <= R.x + R.w - m; x += 24) scan.push({ x, y, w: sw, h: sh, lines: v, k: k * 1e6 + Math.hypot(x + sw / 2 - ucx, y + sh / 2 - ucy) });
      });
      r = scan.sort((p, q) => p.k - q.k).find((s) => free(s) && clearOf(s, null)) || cands.find(free);
      ARU.sumMiss = r ? null : { key: vkey, at: now };
    }
    ARU.sumAt = r ? { x: r.x, y: r.y, key: JSON.stringify(r.lines) } : null;
    if (r) { summary = { x: r.x, y: r.y, w: r.w, h: r.h, lines: r.lines }; placed.push(summary); }
  }
  // compact tags: expected markers nearest the centre first, then look-alikes; a small marker gets its tag on top of
  // it first (neighbours' tags cannot collide there), a large one above it
  let skipped = 0;
  for (const it of [...ok, ...unk]) {
    if (it === sel) continue;
    const text = (it.id != null ? it.id : '') + (it.ok ? '' : '?');
    const w = Math.ceil(g.measureText(text).width) + 10, h = 16, b = it.geo.box, cx = b.x + b.w / 2, cy = b.y + b.h / 2;
    const inside = { x: cx - w / 2, y: cy - h / 2, w, h };
    const around = [[cx - w / 2, b.y - h - 3], [b.x + b.w + 3, cy - h / 2], [cx - w / 2, b.y + b.h + 3], [b.x - w - 3, cy - h / 2],
      [b.x + b.w + 3, b.y - h - 1], [b.x - w - 3, b.y - h - 1], [b.x + b.w + 3, b.y + b.h + 1], [b.x - w - 3, b.y + b.h + 1]].map(([x, y]) => ({ x, y, w, h }));
    const slots = Math.min(b.w, b.h) < 44 ? [inside, ...around] : [...around, inside];
    // never on another marker (it would read as that marker's id); none left: skipped
    const r = slots.find((s) => free(s) && (s === inside || clearOf(s, b)));
    if (!r) { skipped++; continue; }
    it.tag = { ...r, text };
    placed.push(it.tag);
  }
  g.restore();
  return { items, sel, summary, skipped };
}
function aruPill(g, r, stroke, fill = 'rgba(0, 0, 0, 0.75)') {
  g.fillStyle = fill;
  g.beginPath(); if (g.roundRect) g.roundRect(r.x, r.y, r.w, r.h, 5); else g.rect(r.x, r.y, r.w, r.h); g.fill();
  g.strokeStyle = stroke; g.lineWidth = 1; g.stroke();
}
function drawArucoAll(g, list, rect, alpha, cssW, cssH) {
  const L = aruLayout(g, list, rect, cssW, cssH);
  ARU.items = L.items;
  // dimmed look-alikes first so board markers stay on top
  for (const it of L.items) if (!it.ok) drawAruco(g, it, alpha, '#9ca3af');
  for (const it of L.items) if (it.ok) drawAruco(g, it, alpha, getLabelColor(it.d.label), it === L.sel);
  g.save();
  g.globalAlpha = alpha; g.font = ARU_FONT; g.textBaseline = 'middle';
  for (const it of L.items) {
    if (it.tag) {
      const r = it.tag;
      aruPill(g, r, it.ok ? getLabelColor(it.d.label) : 'rgba(156,163,175,0.7)', it.ok ? 'rgba(0, 0, 0, 0.72)' : 'rgba(0, 0, 0, 0.45)');
      g.fillStyle = it.ok ? '#fff' : 'rgba(229,231,235,0.75)'; g.fillText(r.text, r.x + 5, r.y + r.h / 2 + 0.5);
    }
    if (it.hud) {
      const r = it.hud, color = getLabelColor(it.d.label);
      aruPill(g, r, color);
      r.lines.forEach((l, i) => { g.fillStyle = i === 0 ? color : '#fff'; g.fillText(l, r.x + 7, r.y + 3 + 7.5 + i * 15); });
    }
  }
  if (L.summary) {
    const r = L.summary;
    aruPill(g, r, 'rgba(255,255,255,0.35)', 'rgba(0, 0, 0, 0.6)');
    r.lines.forEach((l, i) => { g.fillStyle = i === 1 && L.items.some((x) => !x.ok) ? '#d1d5db' : '#fff'; g.fillText(l, r.x + 7, r.y + 3 + 7.5 + i * 15); });
  }
  g.restore();
  // for tests: the labels drawn this frame (CSS px in the overlay canvas)
  const box = (r) => ({ x: Math.round(r.x * 10) / 10, y: Math.round(r.y * 10) / 10, w: Math.round(r.w * 10) / 10, h: Math.round(r.h * 10) / 10 });
  ARU.layout = { n: L.items.length, hud: L.sel ? L.sel.id : null, skipped: L.skipped, summary: L.summary ? { ...box(L.summary), lines: L.summary.lines } : null,
    labels: L.items.flatMap((it) => [it.tag && { id: it.id, kind: 'tag', dim: !it.ok, text: it.tag.text, ...box(it.tag) }, it.hud && { id: it.id, kind: 'hud', dim: false, lines: it.hud.lines, ...box(it.hud) }].filter(Boolean)),
    markers: L.items.map((it) => ({ id: it.id, dim: !it.ok, ...box(it.geo.box) })) };
}
// expected marker under a page point (tap / hover), or undefined; also matches its tag or HUD
function aruAt(clientX, clientY) {
  const r = $('video-box').getBoundingClientRect(), x = clientX - r.left, y = clientY - r.top;
  const inPoly = (pts) => { let c = false; for (let i = 0, j = 3; i < 4; j = i++) { const [xi, yi] = pts[i], [xj, yj] = pts[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
  const inBox = (b) => b && x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h;
  const cand = ARU.items.filter((o) => o.ok && o.id != null);
  const it = cand.find((o) => (o.geo.pts ? inPoly(o.geo.pts) : inBox(o.geo.box))) || cand.find((o) => inBox(o.tag) || inBox(o.hud));
  return it ? it.id : undefined;
}
function drawAruco(g, it, alpha, color, full = false) {
  const { d } = it, a = d.aruco, { pts, c, L } = it.geo, dim = !it.ok;
  g.save();
  g.globalAlpha = dim ? alpha * 0.55 : alpha;
  g.lineJoin = 'round';

  // 1. outline: corner polygon (fallback: bbox); look-alikes dashed
  g.beginPath();
  if (pts) { g.moveTo(...pts[0]); for (let i = 1; i < 4; i++) g.lineTo(...pts[i]); g.closePath(); }
  else g.rect(it.geo.box.x, it.geo.box.y, it.geo.box.w, it.geo.box.h);
  if (dim) {
    g.setLineDash([4, 3]); g.strokeStyle = color; g.lineWidth = 1.5; g.stroke(); g.setLineDash([]);
    g.restore();
    return;
  }
  g.fillStyle = color; g.globalAlpha = alpha * (full ? 0.2 : 0.12); g.fill(); g.globalAlpha = alpha;
  g.strokeStyle = 'rgba(255,255,255,0.85)'; g.lineWidth = full ? 5 : 4; g.stroke();
  g.strokeStyle = color; g.lineWidth = full ? 3 : 2; g.stroke();

  // 2. axes (screen vectors, px)
  let axes = null;
  if (a.R) axes = [0, 1, 2].map((i) => [a.R[0][i] * L, a.R[1][i] * L]);
  else if (pts) { // corners only: in-plane X (0->1) and Y (3->0, ArUco +Y is up); Z unknown
    const unit = (v) => { const n = Math.hypot(v[0], v[1]) || 1; return [v[0] / n * L, v[1] / n * L]; };
    axes = [unit([pts[1][0] - pts[0][0] + pts[2][0] - pts[3][0], pts[1][1] - pts[0][1] + pts[2][1] - pts[3][1]]),
            unit([pts[0][0] - pts[3][0] + pts[1][0] - pts[2][0], pts[0][1] - pts[3][1] + pts[1][1] - pts[2][1]]), null];
  }
  if (axes) {
    // draw the axis pointing most away from the viewer first so nearer ones stay on top
    const order = [0, 1, 2].filter((i) => axes[i]).sort((i, j) => (a.R ? a.R[2][j] - a.R[2][i] : 0));
    g.lineCap = 'round';
    for (const i of order) {
      const [vx, vy] = axes[i], ex = c[0] + vx, ey = c[1] + vy, len = Math.hypot(vx, vy);
      g.strokeStyle = 'rgba(255,255,255,0.9)'; g.lineWidth = full ? 5 : 4;
      g.beginPath(); g.moveTo(...c); g.lineTo(ex, ey); g.stroke();
      g.strokeStyle = AXIS_COLORS[i]; g.lineWidth = full ? 3 : 2;
      g.beginPath(); g.moveTo(...c); g.lineTo(ex, ey); g.stroke();
      if (len > 6) { // arrow head
        const ux = vx / len, uy = vy / len, hs = Math.min(full ? 8 : 6, len * 0.35);
        g.fillStyle = AXIS_COLORS[i];
        g.beginPath(); g.moveTo(ex, ey);
        g.lineTo(ex - ux * hs - uy * hs * 0.55, ey - uy * hs + ux * hs * 0.55);
        g.lineTo(ex - ux * hs + uy * hs * 0.55, ey - uy * hs - ux * hs * 0.55);
        g.closePath(); g.fill();
      }
      if (full) { // axis letters on the HUD marker only
        g.font = '700 10px ui-monospace, Menlo, monospace'; g.textBaseline = 'middle'; g.textAlign = 'center';
        g.fillStyle = AXIS_COLORS[i];
        g.fillText('XYZ'[i], ex + (len > 1 ? vx / len * 9 : 9), ey + (len > 1 ? vy / len * 9 : 0));
      }
    }
    g.textAlign = 'left';
  }
  if (!a.R) { // Z out of the image plane: "⊙"
    g.strokeStyle = AXIS_COLORS[2]; g.lineWidth = 2;
    g.beginPath(); g.arc(c[0], c[1], 5, 0, Math.PI * 2); g.stroke();
  }
  g.fillStyle = '#fff'; g.beginPath(); g.arc(c[0], c[1], 2, 0, Math.PI * 2); g.fill();

  // 3. corner dots (0 = marker top-left, yellow); indices 0-3 on the HUD marker only
  if (pts) {
    g.font = '700 10px ui-monospace, Menlo, monospace'; g.textBaseline = 'middle'; g.textAlign = 'center';
    pts.forEach((p, i) => {
      if (!full && i !== 0) return;
      g.fillStyle = i === 0 ? '#facc15' : color;
      g.strokeStyle = '#fff'; g.lineWidth = 1.5;
      g.beginPath(); g.arc(p[0], p[1], i === 0 ? 4.5 : 3.5, 0, Math.PI * 2); g.fill(); g.stroke();
      if (!full || Math.min(it.geo.box.w, it.geo.box.h) < 40) return;
      const ox = p[0] - c[0], oy = p[1] - c[1], on = Math.hypot(ox, oy) || 1;
      const lx = p[0] + ox / on * 11, ly = p[1] + oy / on * 11;
      g.fillStyle = 'rgba(0,0,0,0.7)'; g.beginPath(); g.arc(lx, ly, 7, 0, Math.PI * 2); g.fill();
      g.fillStyle = '#fff'; g.fillText(String(i), lx, ly + 0.5);
    });
    g.textAlign = 'left';
  }
  g.restore();
}

