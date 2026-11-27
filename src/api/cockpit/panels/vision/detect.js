/* =================================================================
   YOLO / SSCMA Detections Parser
   ================================================================= */
/* ArUco / 6-DoF pose fields on a detection item (top level, item.pose or item.detail).
   Returns { id, cornersRaw, t: [x,y,z] m | null, R: 3x3 camera-frame rotation | null } or null. */
const POSE = (() => {
  const num3 = (v) => {
    if (Array.isArray(v)) { const a = v.flat(2).slice(0, 3).map(Number); return a.length === 3 && a.every(Number.isFinite) ? a : null; }
    if (v && typeof v === 'object' && v.x !== undefined) { const a = [Number(v.x), Number(v.y), Number(v.z)]; return a.every(Number.isFinite) ? a : null; }
    return null;
  };
  const rodrigues = (r) => {
    const th = Math.hypot(r[0], r[1], r[2]);
    if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const [x, y, z] = [r[0] / th, r[1] / th, r[2] / th], c = Math.cos(th), sn = Math.sin(th), C1 = 1 - c;
    return [[c + x * x * C1, x * y * C1 - z * sn, x * z * C1 + y * sn],
            [y * x * C1 + z * sn, c + y * y * C1, y * z * C1 - x * sn],
            [z * x * C1 - y * sn, z * y * C1 + x * sn, c + z * z * C1]];
  };
  const quatR = (w, x, y, z) => {
    const n = Math.hypot(w, x, y, z) || 1; w /= n; x /= n; y /= n; z /= n;
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
  };
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians
  const eulerR = (r, p, y) => {
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]];
  };
  // inverse of eulerR, degrees [roll, pitch, yaw]
  const rpyDeg = (R) => {
    const D = 180 / Math.PI;
    return [Math.atan2(R[2][1], R[2][2]) * D, Math.asin(clamp(-R[2][0], -1, 1)) * D, Math.atan2(R[1][0], R[0][0]) * D];
  };
  function fromItem(it) {
    if (!it || typeof it !== 'object' || Array.isArray(it)) return null;
    const det = it.detail && typeof it.detail === 'object' ? it.detail : {};
    const pose = [it.pose, it.pose3d, it.pose_3d, det.pose].find((v) => v && typeof v === 'object') || {};
    const pick = (...ks) => { for (const src of [pose, it, det]) for (const k of ks) if (src[k] != null) return src[k]; return undefined; };

    let cornersRaw = null;
    const cr = pick('corners', 'corner', 'polygon');
    if (Array.isArray(cr)) {
      const f = cr.flat(2).map((v) => (v && typeof v === 'object' ? [v.x, v.y] : v)).flat().map(Number);
      if (f.length >= 8 && f.slice(0, 8).every(Number.isFinite)) cornersRaw = [[f[0], f[1]], [f[2], f[3]], [f[4], f[5]], [f[6], f[7]]];
    }

    // translation: tvec / translation / position, or x/y/z inside pose/detail (never the item's own bbox x/y)
    let t = num3(pick('tvec', 'translation', 'position', 'xyz', 't')); // rebot-vision / aruco-detector: pose.t (m)
    if (!t) for (const src of [pose, det]) if (!t && src.z !== undefined) t = num3(src);
    if (t) {
      const unit = String(pick('unit', 'units') || '').toLowerCase();
      if (unit === 'mm' || (unit !== 'm' && Math.abs(t[2]) > 20)) t = t.map((v) => v / 1000); // > 20 "m" in front of a camera is mm
    }

    let R = null;
    const Rm = pick('R', 'rotation_matrix', 'rmat');
    if (Array.isArray(Rm)) { const f = Rm.flat().map(Number); if (f.length === 9 && f.every(Number.isFinite)) R = [f.slice(0, 3), f.slice(3, 6), f.slice(6, 9)]; }
    if (!R) { const rv = num3(pick('rvec', 'rotation_vector', 'rotvec')); if (rv) R = rodrigues(rv); }
    if (!R) {
      const q = pick('quaternion', 'quat', 'orientation');
      if (Array.isArray(q) && q.length >= 4 && q.every((v) => Number.isFinite(Number(v)))) R = quatR(Number(q[3]), Number(q[0]), Number(q[1]), Number(q[2])); // [x, y, z, w]
      else if (q && typeof q === 'object' && q.w !== undefined) R = quatR(Number(q.w), Number(q.x), Number(q.y), Number(q.z));
    }
    if (!R) {
      const rad = String(pick('angle_unit') || '').toLowerCase().startsWith('rad');
      const k = rad ? 1 : Math.PI / 180;
      let e = num3(pick('rpy', 'euler', 'r_deg')); // reCamera detector: pose.r_deg (degrees)
      if (!e) { const r = [pick('roll', 'rx'), pick('pitch', 'ry'), pick('yaw', 'rz')].map(Number); if (r.every(Number.isFinite)) e = r; }
      if (e) R = eulerR(e[0] * k, e[1] * k, e[2] * k);
    }
    if (R && !R.flat().every(Number.isFinite)) R = null;
    if (!cornersRaw && !t && !R) return null;
    const id = pick('marker_id', 'aruco_id', 'id');
    return { id: id != null ? id : null, cornersRaw, t, R };
  }
  return { fromItem, rpyDeg };
})();

function parseDetections(rawData, refRes) {
  if (!rawData) return [];
  // detector envelopes: { result: {...} } / { results: {...} }
  if (!Array.isArray(rawData) && typeof rawData === 'object') {
    const inner = rawData.result || (rawData.results && !Array.isArray(rawData.results) ? rawData.results : null);
    if (inner && typeof inner === 'object' && !rawData.boxes && !rawData.detections) return parseDetections(inner, refRes || (rawData.width && rawData.height ? [rawData.width, rawData.height] : null));
  }
  let items = [];
  let rawContours = null;
  let boxCount = 0;
  let refW = null;
  let refH = null;

  if (Array.isArray(rawData)) {
    if (rawData.length >= 5 && typeof rawData[0] === 'number') {
      items = [rawData];
    } else {
      items = rawData;
    }
    boxCount = items.length;
  } else if (rawData && typeof rawData === 'object') {
    if (rawData.format) {
      items = [rawData];
    } else if (Array.isArray(rawData.boxes)) {
      items = rawData.boxes;
      if (Array.isArray(rawData.contours)) rawContours = rawData.contours;
    } else if (Array.isArray(rawData.detections)) {
      items = rawData.detections;
      if (Array.isArray(rawData.contours)) rawContours = rawData.contours;
    } else if (rawData.data && Array.isArray(rawData.data.boxes)) {
      items = rawData.data.boxes;
      if (Array.isArray(rawData.data.contours)) rawContours = rawData.data.contours;
    } else if (rawData.data && Array.isArray(rawData.data.detections)) {
      items = rawData.data.detections;
      if (Array.isArray(rawData.data.contours)) rawContours = rawData.data.contours;
    } else if (Array.isArray(rawData.data)) {
      items = rawData.data;
    } else if (Array.isArray(rawData.results)) {
      items = rawData.results;
    }
    boxCount = items.length;
    // ArUco markers may arrive in a separate list next to (or instead of) the boxes
    for (const k of ['markers', 'aruco']) {
      const m = Array.isArray(rawData[k]) ? rawData[k] : (rawData.data && Array.isArray(rawData.data[k]) ? rawData.data[k] : null);
      if (m) items = items.concat(m.map((x) => (x && typeof x === 'object' && !Array.isArray(x) ? { label: 'aruco', ...x } : x)));
    }

    refW = rawData.width || rawData.w || (rawData.data && (rawData.data.width || rawData.data.w));
    refH = rawData.height || rawData.h || (rawData.data && (rawData.data.height || rawData.data.h));
    if (!refW && Array.isArray(rawData.resolution) && rawData.resolution.length >= 2) {
      refW = rawData.resolution[0];
      refH = rawData.resolution[1];
    } else if (!refW && rawData.data && Array.isArray(rawData.data.resolution) && rawData.data.resolution.length >= 2) {
      refW = rawData.data.resolution[0];
      refH = rawData.data.resolution[1];
    }
  }

  if (refRes) {
    if (Array.isArray(refRes) && refRes.length >= 2) {
      refW = refW || refRes[0];
      refH = refH || refRes[1];
    } else if (typeof refRes === 'object') {
      refW = refW || refRes.width || refRes.w;
      refH = refH || refRes.height || refRes.h;
    }
  }
  refW = Number(refW);
  refH = Number(refH);
  const hasRef = Number.isFinite(refW) && Number.isFinite(refH) && refW > 0 && refH > 0;

  const results = [];
  for (let idx = 0; idx < items.length; idx++) {
    const item = items[idx];
    let rawCx, rawCy, rawW, rawH, score, label, track_id = null;
    const aru = POSE.fromItem(item);

    if (Array.isArray(item) && item.length >= 5) {
      // 格式 A：Seeed SSCMA 官方原生数组格式 [cx, cy, w, h, score, target] (中心点坐标)
      const rcx = Number(item[0]);
      const rcy = Number(item[1]);
      const rw = Number(item[2]);
      const rh = Number(item[3]);
      if (!Number.isFinite(rcx) || !Number.isFinite(rcy) || !Number.isFinite(rw) || !Number.isFinite(rh)) continue;
      if (rw <= 0 || rh <= 0) continue;
      rawCx = rcx;
      rawCy = rcy;
      rawW = rw;
      rawH = rh;
      score = Number(item[4]);
      label = item[5] !== undefined ? String(item[5]) : 'obj';
      if (item[6] !== undefined) track_id = item[6];
    } else if (item !== null && typeof item === 'object' && !Array.isArray(item)) {
      // 格式 B：标准通用字典格式
      // A marker without a bbox format takes its box from the corner hull.
      const fmt = item.format || (aru && aru.cornersRaw ? 'corners' : undefined);
      if (fmt !== 'xywh_center' && fmt !== 'xyxy' && fmt !== 'xywh_top_left' && fmt !== 'corners') continue;

      if (fmt === 'corners') {
        const xs = aru.cornersRaw.map((c) => c[0]), ys = aru.cornersRaw.map((c) => c[1]);
        const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
        if (x1 <= x0 || y1 <= y0) continue;
        rawW = x1 - x0; rawH = y1 - y0;
        rawCx = xs.reduce((a, b) => a + b, 0) / 4; rawCy = ys.reduce((a, b) => a + b, 0) / 4;
      } else if (fmt === 'xywh_center') {
        const cx = Number(item.cx ?? item.x);
        const cy = Number(item.cy ?? item.y);
        const w = Number(item.w ?? item.width);
        const h = Number(item.h ?? item.height);
        if (!Number.isFinite(cx) || !Number.isFinite(cy) || !Number.isFinite(w) || !Number.isFinite(h)) continue;
        if (w <= 0 || h <= 0) continue;
        rawCx = cx;
        rawCy = cy;
        rawW = w;
        rawH = h;
      } else if (fmt === 'xyxy') {
        let x0, y0, x1, y1;
        if (item.x1 !== undefined && item.x2 !== undefined) {
          x0 = Number(item.x1); x1 = Number(item.x2);
        } else if (item.x0 !== undefined && item.x1 !== undefined) {
          x0 = Number(item.x0); x1 = Number(item.x1);
        } else if (Array.isArray(item.bbox) && item.bbox.length >= 4) {
          x0 = Number(item.bbox[0]); x1 = Number(item.bbox[2]);
        } else if (Array.isArray(item.box) && item.box.length >= 4) {
          x0 = Number(item.box[0]); x1 = Number(item.box[2]);
        }
        if (item.y1 !== undefined && item.y2 !== undefined) {
          y0 = Number(item.y1); y1 = Number(item.y2);
        } else if (item.y0 !== undefined && item.y1 !== undefined) {
          y0 = Number(item.y0); y1 = Number(item.y1);
        } else if (Array.isArray(item.bbox) && item.bbox.length >= 4) {
          y0 = Number(item.bbox[1]); y1 = Number(item.bbox[3]);
        } else if (Array.isArray(item.box) && item.box.length >= 4) {
          y0 = Number(item.box[1]); y1 = Number(item.box[3]);
        }
        if (!Number.isFinite(x0) || !Number.isFinite(y0) || !Number.isFinite(x1) || !Number.isFinite(y1)) continue;
        if (x1 <= x0 || y1 <= y0) continue;
        rawW = x1 - x0;
        rawH = y1 - y0;
        rawCx = (x0 + x1) / 2;
        rawCy = (y0 + y1) / 2;
      } else if (fmt === 'xywh_top_left') {
        let x0 = Number(item.x ?? item.x0);
        let y0 = Number(item.y ?? item.y0);
        let w = Number(item.w ?? item.width);
        let h = Number(item.h ?? item.height);
        if (Array.isArray(item.bbox) && item.bbox.length >= 4) {
          x0 = Number(item.bbox[0]); y0 = Number(item.bbox[1]);
          w = Number(item.bbox[2]); h = Number(item.bbox[3]);
        }
        if (!Number.isFinite(x0) || !Number.isFinite(y0) || !Number.isFinite(w) || !Number.isFinite(h)) continue;
        if (w <= 0 || h <= 0) continue;
        rawW = w;
        rawH = h;
        rawCx = x0 + w / 2;
        rawCy = y0 + h / 2;
      }
      score = Number(item.score ?? item.confidence ?? (aru ? 1 : 0)); // marker decoders report no score
      label = item.label ?? item.name ?? item.target ?? item.class_name ?? (aru ? 'aruco' : 'obj');
      track_id = item.track_id ?? item.id ?? null;
    } else {
      continue;
    }

    // 置信度校验与防御
    if (!Number.isFinite(score)) continue;
    score = Math.max(0.0, Math.min(1.0, score));

    // 计算未截断的原始边界框
    const rawX0 = rawCx - rawW / 2;
    const rawY0 = rawCy - rawH / 2;
    const rawX1 = rawCx + rawW / 2;
    const rawY1 = rawCy + rawH / 2;

    // 归一化判定：依据原始中心点与尺寸本身是否超出 1.05 容差判定为像素单位。
    // 严禁使用反解后的 rawX1/rawY1 判断，避免合法贴边/越界出画目标（如 cx=0.95, w=0.4 -> rawX1=1.15）被误判为像素单位而发生二次重复归一化！
    const isPixelUnit = rawCx > 1.05 || rawCy > 1.05 || rawW > 1.05 || rawH > 1.05;

    let normCx, normCy, normX0, normY0, normX1, normY1;
    if (isPixelUnit) {
      if (!hasRef) continue; // 绝对像素但无有效分辨率，防御性丢弃
      normCx = rawCx / refW;
      normCy = rawCy / refH;
      normX0 = rawX0 / refW;
      normY0 = rawY0 / refH;
      normX1 = rawX1 / refW;
      normY1 = rawY1 / refH;
    } else {
      // 已经是归一化单位（0.0 ~ 1.0）
      normCx = rawCx;
      normCy = rawCy;
      normX0 = rawX0;
      normY0 = rawY0;
      normX1 = rawX1;
      normY1 = rawY1;
    }

    // 1. 边界框安全裁剪（clamp 至 [0.0, 1.0] 可见画幅）
    const x0 = Math.max(0.0, Math.min(1.0, normX0));
    const y0 = Math.max(0.0, Math.min(1.0, normY0));
    const x1 = Math.max(0.0, Math.min(1.0, normX1));
    const y1 = Math.max(0.0, Math.min(1.0, normY1));
    const w = x1 - x0;
    const h = y1 - y0;
    if (w <= 0.001 || h <= 0.001) continue;

    // 2. 独立保留物理探测中心（准星）：保留物体真实探测中心，绝不被裁剪框中点覆盖！
    const cx = Math.max(0.0, Math.min(1.0, normCx));
    const cy = Math.max(0.0, Math.min(1.0, normCy));

    // ArUco: corners share the bbox unit; normalise them with it (no clamp, so the polygon keeps its shape)
    let aruco = null;
    if (aru) {
      const corners = aru.cornersRaw ? aru.cornersRaw.map(([u, v]) => (isPixelUnit ? [u / refW, v / refH] : [u, v])) : null;
      aruco = { id: aru.id != null ? aru.id : track_id, corners, t: aru.t, R: aru.R, rpy: aru.R ? POSE.rpyDeg(aru.R) : null };
    }

    let obbCorners = null;
    if (Array.isArray(item.obb_corners) && item.obb_corners.length === 4) {
      obbCorners = item.obb_corners.map(([u, v]) => (isPixelUnit ? [Number(u) / refW, Number(v) / refH] : [Number(u), Number(v)]));
    }

    // segment models: exterior outline of the mask. HTTP detections carry `item.contour` in the box's
    // unit; WS frames carry top-level `contours` aligned with `boxes` in normalised [0, 1] coordinates.
    let contour = null;
    if (item && !Array.isArray(item) && Array.isArray(item.contour) && item.contour.length >= 3) {
      contour = item.contour.filter((c) => Array.isArray(c) && c.length >= 2).map(([u, v]) => (isPixelUnit ? [Number(u) / refW, Number(v) / refH] : [Number(u), Number(v)]))
        .filter(([u, v]) => Number.isFinite(u) && Number.isFinite(v));
      if (contour.length < 3) contour = null;
    } else if (rawContours && idx < boxCount && Array.isArray(rawContours[idx]) && rawContours[idx].length >= 3) {
      contour = rawContours[idx].filter((c) => Array.isArray(c) && c.length >= 2).map(([u, v]) => [Number(u), Number(v)])
        .filter(([u, v]) => Number.isFinite(u) && Number.isFinite(v));
      if (contour.length < 3) contour = null;
    }

    results.push({
      x0, y0, x1, y1,
      cx, cy, w, h,
      label: String(label),
      score,
      track_id: track_id != null ? track_id : null,
      aruco,
      obbCorners,
      contour
    });
  }
  return results;
}

function videoSrcRes() {
  const srcW = V.mode === 'mjpeg' ? (mjpegEl ? mjpegEl.naturalWidth : 0) : (videoEl ? videoEl.videoWidth : 0);
  const srcH = V.mode === 'mjpeg' ? (mjpegEl ? mjpegEl.naturalHeight : 0) : (videoEl ? videoEl.videoHeight : 0);
  return (srcW && srcH) ? [srcW, srcH] : null;
}

let detWsRetryCount = 0;
let detWsRetryTimer = null;
function scheduleDetWsRetry() {
  if (detWsRetryTimer) { clearTimeout(detWsRetryTimer); detWsRetryTimer = null; }
  detWsRetryCount++;
  const delay = Math.min(15000, Math.round(2000 * Math.pow(1.5, Math.min(detWsRetryCount, 5))));
  detWsRetryTimer = setTimeout(connectDetectionWS, delay);
}

function connectDetectionWS() {
  if (detWsRetryTimer) { clearTimeout(detWsRetryTimer); detWsRetryTimer = null; }
  const host = QS.get('det_ws') || location.host;
  const rawPath = QS.get('det_path') || '/api/v1/detection/ws';
  const path = rawPath.startsWith('/') ? rawPath : '/' + rawPath;
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  let ws;
  try {
    ws = new WebSocket(`${proto}//${host}${path}`);
  } catch (_) {
    scheduleDetWsRetry();
    return;
  }
  ws.onmessage = (ev) => {
    detWsRetryCount = 0; // Reset backoff only upon receiving active data from upstream
    if (DET.avail && !DET.enabled && !CALSUB.id) return; // overlay off: draw nothing, even if a stale frame arrives
    if (performance.now() < (S.detHoldUntil || 0)) return; // a snapshot result is on screen
    try {
      const data = JSON.parse(ev.data);
      const parsed = parseDetections(data, videoSrcRes());
      S.detections = parsed;
      S.detSource = 'det_ws';
      S.detLastReceivedAt = performance.now();
    } catch (_) {}
  };
  ws.onclose = () => {
    if (S.detSource === 'det_ws') {
      S.detections = [];
      S.detLastReceivedAt = 0;
      clearOverlayCanvas();
    }
    scheduleDetWsRetry();
  };
  ws.onerror = () => {
    if (S.detSource === 'det_ws') {
      S.detections = [];
      S.detLastReceivedAt = 0;
      clearOverlayCanvas();
    }
    try { ws.close(); } catch (_) {}
  };
}

/* Detect toggle: on-demand inference via /api/v1/detection/inference (armd proxies the detector).
   Plain fetch, not api(): this is not arm control and must not feed the RTT telemetry. */
// trigCap: does the detector have POST /api/trigger? null = not probed yet, false = 404 (older build).
const DET = { avail: null, enabled: false, busy: false, timer: null, snap: false, fps: null, activeFps: null, idleFps: null, trigCap: null };
async function detRequest(method, body) {
  const ctl = new AbortController();
  const tm = setTimeout(() => ctl.abort(), 4000);
  try {
    const res = await fetch('/api/v1/detection/inference', {
      method, signal: ctl.signal, cache: 'no-store',
      headers: body !== undefined ? { 'Content-Type': 'application/json' } : undefined,
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (!res.ok || !data || typeof data.enabled !== 'boolean') {
      const e = new Error((data && data.error) || 'HTTP ' + res.status); e.status = res.status; e.code = data && data.code; throw e;
    }
    return data;
  } finally { clearTimeout(tm); }
}
function detClearOverlay() {
  S.detHoldUntil = 0;
  S.detections = [];
  S.detLastReceivedAt = 0;
  clearOverlayCanvas();
}
function detApply(data) {
  DET.avail = !!data;
  if (!data) DET.trigCap = null; // re-probe once the detector (possibly upgraded) comes back
  // perception API 2: "enabled" is this page's overlay subscription, not the provider's default one
  DET.enabled = VM.subs ? !!(data && OV.on) : !!(data && data.enabled);
  const n = (v) => (Number.isFinite(Number(v)) && v !== null ? Number(v) : null);
  DET.fps = n(data && data.fps); DET.activeFps = n(data && data.active_fps); DET.idleFps = n(data && data.idle_fps);
  if (DET.avail && !DET.enabled && !detHeld()) detClearOverlay(); // a held single-shot result stays (detHoldOnObs)
  renderDetToggle();
}
/* simple mode: recognition popover (#det-ctl) opened from the HUD chip; closes on the chip, an outside tap, a mode switch
   (Escape = Stop, Space = E-stop) */
function visPopOpen(open) {
  const body = document.body, chip = $('btn-vis');
  if (open === body.classList.contains('vis-open')) return;

  body.classList.toggle('vis-open', open);
  if (chip) chip.setAttribute('aria-expanded', String(open));
  if (open && typeof svSheetPlace === 'function') svSheetPlace();
  if (!open && typeof SV !== 'undefined' && SV.scrollBack != null) { const y = SV.scrollBack; SV.scrollBack = null; if ($('meas-card').hidden) window.scrollTo(0, y); }
}
document.addEventListener('pointerdown', (e) => {
  if (!document.body.classList.contains('vis-open')) return;
  if (e.target.closest && e.target.closest('#det-ctl, #btn-vis, #s-vis')) return;
  visPopOpen(false);
}, true);
function renderDetToggle() {
  const wrap = $('det-ctl'), btn = $('btn-det');
  if (!wrap || !btn) return;
  wrap.hidden = DET.avail === null;
  const key = !DET.avail ? 'na' : DET.enabled ? 'on' : 'off';
  setText(btn, t('det.' + key));
  btn.disabled = !DET.avail || DET.busy;
  btn.classList.toggle('btn-primary', !!(DET.avail && DET.enabled));
  btn.setAttribute('aria-pressed', String(!!(DET.avail && DET.enabled)));
  // A disabled button gets no hover events, so the tooltip lives on the wrapper.
  wrap.title = t('det.tip.' + key);
  btn.setAttribute('aria-label', t('det.tip.' + key));
  btn.disabled = btn.disabled || DET.snap;
  // simple mode: the HUD chip mirrors the Detect state and opens the recognition popover
  const vis = $('btn-vis');
  if (vis) {
    vis.hidden = wrap.hidden;
    setText(vis, S.ui === 'simple' ? svChipText() : btn.textContent);
    vis.classList.toggle('btn-primary', !!(DET.avail && DET.enabled));
    vis.title = t('vis.tip');
    vis.setAttribute('aria-label', btn.textContent + ' · ' + t('vis.tip'));
    if (wrap.hidden) visPopOpen(false);
  }

  const trig = $('btn-trigger');
  if (trig) {
    setText(trig, t(DET.snap ? 'det.trig.busy' : 'det.trig'));
    trig.disabled = !DET.avail || DET.busy || DET.snap;
    trig.classList.toggle('busy', DET.snap);
    trig.title = t('det.trig.tip');
  }
  const pill = $('fps-pill');
  if (pill) {
    pill.hidden = !DET.avail;
    const idle = DET.idleFps ?? 1, active = DET.activeFps ?? '--';
    const cur = DET.fps;
    let cls = 'fps-pill', txt;
    if (DET.snap) { cls += ' is-snap'; txt = t('det.fps.snap'); }
    else if (DET.enabled) { cls += ' is-work'; txt = t('det.fps.work', { n: cur && cur > idle ? cur : active }); }
    else txt = t('det.fps.idle', { n: cur != null && cur <= idle ? cur : idle });
    setCls(pill, cls);
    setText(pill, txt);
    pill.title = t('det.fps.tip', { idle, active });
  }
  if (typeof renderVisionMode === 'function') renderVisionMode();
  if (typeof renderExpertVision === 'function') renderExpertVision();
}

/* Single-shot detection. Primary: POST /detection/trigger, which runs inference on a frame newer than the call
   without touching the continuous `enabled` flag, so other clients that enabled inference are unaffected.
   Fallback for detector builds that answer 404 (cached in DET.trigCap): (if off) POST enabled:true ->
   GET /detection/latest?wait_ms (newer than the pre-trigger result) -> POST enabled:false.
   A result taken while inference was off stays on screen for SNAP_HOLD_MS. Resolves to the parsed detections. */
const SNAP_HOLD_MS = 4000;
// The held result is dropped early when an arm joint (not the gripper) moved more than this from where the
// shot was taken (the boxes no longer match the camera view), and when the user taps the video.
const SNAP_MOVE_RAD = 0.01;
const detHeld = () => performance.now() < (S.detHoldUntil || 0);
function detHoldOnObs(o) {
  if (!detHeld()) return;
  const q0 = S.detHoldQ, q = o.positions;
  if (!q0) { S.detHoldQ = q.slice(); return; }
  if (q.some((v, i) => i !== GRIP && i < q0.length && Math.abs(v - q0[i]) > SNAP_MOVE_RAD)) detClearOverlay();
}
const TRIG_WAIT_MS = 3000;
async function detLatest(query) {
  const ctl = new AbortController();
  const tm = setTimeout(() => ctl.abort(), 6500);
  try {
    const res = await fetch('/api/v1/detection/latest' + query, { signal: ctl.signal, cache: 'no-store' });
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (!res.ok || data == null) { const e = new Error((data && data.error) || 'HTTP ' + res.status); e.status = res.status; throw e; }
    return data;
  } finally { clearTimeout(tm); }
}
async function detTriggerReq(waitMs, pipelines, retried = false) {
  const ctl = new AbortController();
  // armd waits wait_ms + 2 s for the detector; allow a little more for the browser round trip.
  const tm = setTimeout(() => ctl.abort(), waitMs + 3500);
  try {
    // perception API 2 runs exactly these pipelines on the frame; older providers ignore the body
    const res = await fetch('/api/v1/detection/trigger?wait_ms=' + waitMs, {
      method: 'POST', signal: ctl.signal, cache: 'no-store',
      headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(pipelines ? { pipelines } : {}),
    });
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (res.status === 403 && data && data.code === 'OPERATOR_REQUIRED' && !retried) {
      clearTimeout(tm);
      if (await opRequire('/api/v1/detection/trigger')) return detTriggerReq(waitMs, pipelines, true);
    }
    if (!res.ok || data == null) { const e = new Error((data && data.error) || 'HTTP ' + res.status); e.status = res.status; e.code = data && data.code; throw e; }
    return data;
  } finally { clearTimeout(tm); }
}
/* Frame age from the device result (age_ms, or ts_ms against now_ms); null when the result carries neither. */
function detFrameAgeMs(d) {
  if (!d || typeof d !== 'object') return null;
  if (Number.isFinite(d.age_ms)) return d.age_ms;
  if (Number.isFinite(d.ts_ms) && Number.isFinite(d.now_ms)) return d.now_ms - d.ts_ms;
  return null;
}
const detSeq = (d) => (d && typeof d === 'object' ? (d.seq ?? d.frame_id ?? d.timestamp ?? d.ts ?? null) : null);
/* Fallback single shot for detectors without /api/trigger. Sets ctx.enabledByUs when it turned inference on. */
async function detComposedShot(ctx) {
  let base = null;
  try { base = detSeq(await detLatest('')); } catch (_) {}
  // without a baseline `latest` would return a cached old result immediately
  if (base == null) throw new Error('Detection baseline unavailable');
  // fresh device state, not the page's cached 10 s poll
  let wasOn;
  try { wasOn = ctx.wasOn = (await detRequest('GET')).enabled; } catch (_) { throw new Error('Detection state unavailable'); }
  if (!wasOn) {
    ctx.enabledByUs = true; // set first: if the reply is lost the enable may still have applied, so restore anyway
    await detRequest('POST', { enabled: true });
  }
  const query = '?wait_ms=' + TRIG_WAIT_MS + (base != null ? '&after=' + encodeURIComponent(base) : '');
  const data = await detLatest(query);
  if (!data || data.timeout || data.frame_id === 0) {
    throw new Error('Detection timeout: no fresh frame received');
  }
  if (base != null) {
    const curSeq = detSeq(data);
    if (curSeq != null && Number(curSeq) <= Number(base)) {
      throw new Error('Detection stale: frame ID not newer than baseline');
    }
  }
  return data;
}
async function triggerDet({ quiet = false, pipelines = null } = {}) {
  if (!DET.avail || DET.busy || DET.snap) return null;
  DET.snap = true; renderDetToggle();
  const wasOn = DET.enabled;
  const qShot = S.obs && Array.isArray(S.obs.positions) ? S.obs.positions.slice() : null; // arm pose of the shot
  const ctx = { enabledByUs: false, wasOn };
  let parsed = null, err = null, parsedReceivedAt = 0, devAge = null;
  try {
    let data = null, viaTrigger = false;
    if (DET.trigCap !== false) {
      try {
        data = await detTriggerReq(TRIG_WAIT_MS, pipelines || (VM.subs ? vmPipelines(OV.pipes) : null));
        DET.trigCap = true; viaTrigger = true;
      } catch (e) {
        if (e.status !== 404) throw e;
        DET.trigCap = false;
      }
    }
    if (viaTrigger) {
      // The detector already guarantees frame_id > the pre-call frame; success=false means no fresh frame in time.
      if (data.success !== true || data.timeout || !data.frame_id) {
        throw new Error('Detection timeout: no fresh frame received');
      }
    } else {
      data = await detComposedShot(ctx);
    }
    const receivedAt = performance.now();
    parsed = parseDetections(data, videoSrcRes());
    parsedReceivedAt = receivedAt;
    const ageMs = detFrameAgeMs(data);
    devAge = ageMs == null ? null : { age: ageMs, rx: receivedAt };
  } catch (e) { err = e; }
  let restoreFailed = false;
  if (ctx.enabledByUs) {
    restoreFailed = true;
    for (let i = 0; i < 3 && restoreFailed; i++) {
      try { detApply(await detRequest('POST', { enabled: false })); restoreFailed = false; }
      catch (_) {}
    }
  }
  DET.snap = false;
  if (restoreFailed) {
    refreshDetState();
    toast(t('det.restore.fail'), 'warn');
  }
  if (parsed) {
    S.detections = parsed; S.detSource = 'snapshot'; S.detLastReceivedAt = parsedReceivedAt || performance.now();
    S.detDevAge = devAge;
    if (!ctx.wasOn) { S.detHoldUntil = performance.now() + SNAP_HOLD_MS; S.detHoldQ = qShot; }
    if (!quiet) toast(parsed.length ? t('det.trig.ok', { n: parsed.length }) : t('det.trig.none'), parsed.length ? '' : 'warn');
  } else if (!quiet) toast(t('det.trig.fail'), 'err', err && err.status ? String(err.status) : '');
  renderDetToggle();
  return parsed;
}
async function refreshDetState() {
  if (DET.busy || DET.snap) return;
  try { detApply(await detRequest('GET')); }
  catch (_) { detApply(null); }
  if (!VM.busy) await vmRefresh();
}
async function toggleDet() {
  if (!DET.avail || DET.busy || DET.snap) return;
  if (VM.subs) { ovSet(!OV.on); return; }
  DET.busy = true; renderDetToggle();
  try {
    const data = await detRequest('POST', { enabled: !DET.enabled }); DET.busy = false; S.detHoldUntil = 0; detApply(data);
    if (VM.state === 'ok') vmRefresh(); // enabling while Off selects the last active mode
  }
  catch (e) {
    DET.busy = false;
    if (e.code === 'OPERATOR_REQUIRED') { opRequire('/api/v1/detection/inference'); renderDetToggle(); return; }
    toast(t('det.fail'), 'err', e.status ? String(e.status) : '');
    if (e.status === 502 || e.status === 404) detApply(null); else renderDetToggle();
  }
}
function initDetToggle() {
  $('btn-det').addEventListener('click', toggleDet);
  $('btn-vis').addEventListener('click', () => visPopOpen(!document.body.classList.contains('vis-open')));
  $('btn-trigger').addEventListener('click', () => triggerDet());
  i18nHooks.push(renderDetToggle);
  // Polling picks up a detector that starts later and toggles made by agents over the API.
  // Slower while unavailable: each failed probe is a 502 that armd logs.
  const poll = async () => {
    if (!document.hidden) await refreshDetState();
    DET.timer = setTimeout(poll, DET.avail ? 10000 : 30000);
  };
  poll();
}

/* Vision mode (rebot-vision): GET/POST /api/v1/vision/mode, off | yolo | aruco | both.
   VM.state: null = not probed, 'ok', 'na' (route failed, shown disabled), 'none' (404: provider has no
   mode API, selector hidden). Selecting a mode other than Off also turns continuous inference on; the
   reply carries enabled/fps like /detection/inference and updates the Detect toggle.
   The "Show" selector (#vmode) is a group of toggles: YOLO, ArUco and Depth (est.) can be on together (boxes and
   markers over the depth map); Off turns all of them off. YOLO + ArUco is the pipeline set / device mode "both".
   Depth is this page's display only (GET /api/v1/vision/depth): it never goes into a subscription or the device mode.
   Persistence as before: the YOLO / ArUco choice (rebot.vision.pipes) is remembered, on/off and Depth are not, so a
   reload starts no inference (API 1: the selector shows the device mode). */
const VM_MODES = ['off', 'yolo', 'aruco', 'both'];
// subs: the provider has per-request pipelines and subscriptions (perception API 2); the selector
// then picks this page's overlay pipelines instead of the provider's global mode
const VM = { state: null, mode: null, yolo: true, busy: false, subs: false, depth: false, depthMetric: false, models: null, ion: null, modelConfig: null };
const vmPipelines = (m) => (m === 'both' ? ['yolo', 'aruco'] : [m]);

/* Vision subscriptions (perception API 2): continuous inference per caller, kept alive every
   SUB_KEEP_MS and expiring SUB_TTL_MS after the last keep-alive (a closed or sleeping tab stops
   on its own). The overlay holds OVSUB while it is on and the page is visible; the calibration
   drawer holds CALSUB (ArUco) while it is open. One subscription per client id. */
const SUB_TTL_MS = 3000, SUB_KEEP_MS = 1000;
const PAGE_ID = Math.random().toString(36).slice(2, 10);
const subOf = (name) => ({ client: name + '-' + PAGE_ID, id: null, pipelines: null, timer: null, task: Promise.resolve() });
const OVSUB = subOf('cockpit-overlay'), CALSUB = subOf('cockpit-calib');
// overlay preference: on/off and the pipelines the selector picked
const OV = { on: false, pipes: ['yolo', 'aruco', 'both'].includes(store('rebot.vision.pipes')) ? store('rebot.vision.pipes') : 'yolo' };
async function subRequest(method, path, body, keepalive = false) {
  const ctl = new AbortController();
  const tm = setTimeout(() => ctl.abort(), 4000);
  try {
    const res = await fetch('/api/v1/vision/' + path, {
      method, signal: ctl.signal, cache: 'no-store', keepalive,
      headers: body !== undefined ? { 'Content-Type': 'application/json' } : undefined,
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (!res.ok || !data) { const e = new Error((data && data.error) || 'HTTP ' + res.status); e.status = res.status; e.code = data && data.code; throw e; }
    return data;
  } finally { clearTimeout(tm); }
}
async function subPost(sub, pipelines) {
  const data = await subRequest('POST', 'subscribe', { client: sub.client, pipelines, ttl_ms: SUB_TTL_MS });
  sub.id = data.id; sub.pipelines = pipelines;
  return data;
}
async function subKeep(sub) {
  if (!sub.id) return;
  try { await subRequest('POST', 'subscribe/' + encodeURIComponent(sub.id) + '/keepalive', {}); }
  catch (e) {
    // expired (tab asleep, provider restarted): subscribe again under the same client
    if (e.status === 404 && sub.pipelines) { sub.id = null; try { await subPost(sub, sub.pipelines); } catch (_) {} }
  }
}
/* Hold `sub` with these pipelines (create, or update in place), or drop it with null. Serialised per
   subscription. Resolves to the error of a failed create, else null. */
function subHold(sub, pipelines) {
  sub.task = sub.task.then(async () => {
    if (!pipelines) {
      clearInterval(sub.timer); sub.timer = null;
      const id = sub.id; sub.id = null; sub.pipelines = null;
      if (id) { try { await subRequest('DELETE', 'subscribe/' + encodeURIComponent(id)); } catch (_) {} }
      return null;
    }
    if (sub.id && sub.pipelines && sub.pipelines.join() === pipelines.join()) return null;
    try { await subPost(sub, pipelines); } catch (e) { return e; }
    if (!sub.timer) sub.timer = setInterval(() => { sub.task = sub.task.then(() => subKeep(sub)); }, SUB_KEEP_MS);
    return null;
  });
  return sub.task;
}
/* Page closing: fire-and-forget DELETEs that outlive the page (fetch keepalive). */
function subDropNow(sub) {
  clearInterval(sub.timer); sub.timer = null;
  if (sub.id) { subRequest('DELETE', 'subscribe/' + encodeURIComponent(sub.id), undefined, true).catch(() => {}); sub.id = null; }
}
/* Bring OVSUB in line with the overlay preference and the page's visibility. */
/* Error code for a refused vision request: the cause (errWhy) instead of a bare HTTP status. */
function vmErrCode(e) {
  if (!e) return '';
  if (e.code === 'OPERATOR_REQUIRED') return e.code;
  if (e.name === 'AbortError') return 'VISION_TIMEOUT';
  if (e.status === 409) return 'MODEL_NOT_LOADED';
  if (e.status === 502 || e.status === 503 || e.status === 504) return 'VISION_DOWN';
  return e.code || (e.status ? String(e.status) : '');
}
async function ovSync(byUser = false) {
  const want = VM.subs && OV.on && !document.hidden;
  const pipes = OV.pipes;
  const err = await subHold(OVSUB, want ? vmPipelines(pipes) : null);
  if (err && OV.on) {
    OV.on = false;
    const again = () => ovSet(true, pipes);
    if (err.code === 'OPERATOR_REQUIRED' && byUser) opRequire('/api/v1/vision/subscribe', undefined, { retry: true }).then((ok) => { if (ok) again(); });
    else toast(t('vm.fail'), 'err', vmErrCode(err), { path: '/api/v1/vision/subscribe', retry: again });
    await subHold(OVSUB, null);
  }
  DET.enabled = !!(DET.avail && OV.on);
  if (!DET.enabled) detClearOverlay();
  renderDetToggle();
}
function ovSet(on, pipes) {
  if (pipes) { OV.pipes = pipes; store('rebot.vision.pipes', pipes); }
  OV.on = on; S.detHoldUntil = 0;
  return ovSync(true);
}
async function vmRequest(method, body, keepalive = false) {
  const ctl = new AbortController();
  const tm = setTimeout(() => ctl.abort(), 4000);
  try {
    const res = await fetch('/api/v1/vision/mode', {
      method, signal: ctl.signal, cache: 'no-store', keepalive,
      headers: body !== undefined ? { 'Content-Type': 'application/json' } : undefined,
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (!res.ok || !data || typeof data.mode !== 'string') {
      const e = new Error((data && data.error) || 'HTTP ' + res.status); e.status = res.status; e.code = data && data.code; throw e;
    }
    return data;
  } finally { clearTimeout(tm); }
}
function vmApply(data) {
  VM.state = 'ok';
  const subs = data.subscriptions === true && data.per_request_pipelines === true;
  if (subs !== VM.subs) { VM.subs = subs; if (subs) ovSync(); }
  VM.mode = VM_MODES.includes(data.mode) ? data.mode : null;
  VM.yolo = !(data.available && data.available.yolo === false);
  VM.depth = !!(data.available && data.available.depth === true);
  VM.depthMetric = VM.depth && !!(data.depth_model && data.depth_model.metric === true);
  VM.models = data.models && typeof data.models === 'object' ? data.models : null;
  VM.ion = data.ion && typeof data.ion === 'object' ? data.ion : null;
  VM.kind = data.model && typeof data.model.kind === 'string' ? data.model.kind : null;
  VM.modelName = data.model && typeof data.model.name === 'string' ? data.model.name : null;
  VM.modelConfig = data.model_config && typeof data.model_config === 'object' ? data.model_config : null;
  if (typeof data.enabled === 'boolean') detApply({ enabled: data.enabled, fps: data.fps, active_fps: data.active_fps, idle_fps: data.idle_fps });
  renderVisionMode();
}
function vmFailed(e) {
  VM.state = e && e.status === 404 ? 'none' : 'na';
  renderVisionMode();
}
async function vmRefresh() {
  try { vmApply(await vmRequest('GET')); } catch (e) { vmFailed(e); }
}
/* ION budget (rebot-vision with model switching: models.catalogue[].ion_mb, ion.base_mb / used_mb / total_mb /
   reserve_mb; docs/detection_api.md "Model switching"). slot 'depth' or 'detector'. A slot without a loaded model:
   would the smallest catalogue model of that slot fit next to the resident ones? The depth slot with a model loaded:
   does the resident set itself fit (an older provider could load segment + depth: 3 MB ION left, the encoder hung)?
   -> { need, limit } MB when over the budget, else null (also when the provider reports no budget). */
function vmBudget(slot) {
  const num = (v) => (typeof v === 'number' && isFinite(v) ? v : null);
  const ion = VM.ion, cat = VM.models && Array.isArray(VM.models.catalogue) ? VM.models.catalogue.filter((e) => e && typeof e === 'object') : null;
  const total = ion ? num(ion.total_mb) : null;
  if (!cat || total == null) return null;
  const reserve = num(ion.reserve_mb) || 0;
  const inSlot = (e) => (slot === 'depth') === (e.kind === 'depth');
  if (cat.some((e) => e.resident && inSlot(e))) {
    const used = num(ion.used_mb);
    return slot === 'depth' && used != null && used + reserve > total ? { need: used + reserve, limit: total } : null;
  }
  const base = num(ion.base_mb);
  const sizes = cat.filter((e) => inSlot(e) && num(e.ion_mb) != null && (slot !== 'depth' || e.metric !== false)).map((e) => e.ion_mb);
  if (base == null || !sizes.length) return null;
  const need = base + reserve + Math.min(...sizes) + cat.filter((e) => e.resident && !inSlot(e)).reduce((a, e) => a + (num(e.ion_mb) || 0), 0);
  return need > total ? { need, limit: total } : null;
}
const vmIonTip = (what, b) => t('vm.tip.ion', { what: t(what), need: b.need.toFixed(1), limit: b.limit.toFixed(1) });
/* Why the Depth toggle is off limits ('' = available): no depth model, a relative one, or over the ION budget. */
function depthBlock() {
  if (VM.state !== 'ok') return t('vm.tip.na');
  const b = vmBudget('depth');
  if (!VM.depth) return b ? vmIonTip('vm.what.depth', b) : t('depth.tip.none');
  if (!VM.depthMetric) return t('depth.tip.nm');
  return b ? vmIonTip('vm.what.depth', b) : '';
}
/* YOLO / ArUco shown: API 2 this page's overlay, API 1 the device mode. */
function vmSel() {
  const m = VM.subs ? (OV.on ? OV.pipes : 'off') : VM.mode;
  return { yolo: m === 'yolo' || m === 'both', aruco: m === 'aruco' || m === 'both' };
}
const vmModeOf = (yolo, aruco) => (yolo && aruco ? 'both' : yolo ? 'yolo' : aruco ? 'aruco' : 'off');
function renderVisionMode() {
  const wrap = $('vmode');
  if (!wrap) return;
  wrap.hidden = VM.state === null || VM.state === 'none';
  const ok = VM.state === 'ok';
  wrap.title = t(ok ? 'vm.tip' : 'vm.tip.na');
  // perception API 2: the selector shows this page's overlay, Off = overlay off. The "Show" prefix says so:
  // the provider's global mode (another client's) can be "yolo" while this page shows nothing.
  const k = $('vm-k');
  if (k) { k.hidden = !VM.subs; setText(k, t('vm.show')); }
  wrap.setAttribute('aria-label', t(VM.subs ? 'vm.label.show' : 'vm.label'));
  const sel = vmSel(), dBlock = depthBlock();
  for (const b of wrap.querySelectorAll('button[data-vm]')) {
    const m = b.dataset.vm;
    const on = ok && (m === 'off' ? !sel.yolo && !sel.aruco && !DEPTH.on : m === 'depth' ? DEPTH.on : sel[m]);
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
    let off = !ok || VM.busy || DET.snap, tip = '';
    if (m === 'yolo' && ok && !VM.yolo) { off = true; const bb = vmBudget('detector'); tip = bb ? vmIonTip('vm.what.yolo', bb) : t('vm.tip.noyolo'); }
    // Depth is this page's display: independent of a pending subscription change or a single shot
    if (m === 'depth') { off = !!dBlock; tip = dBlock || t('depth.tip'); }
    b.disabled = off;
    if (tip) b.title = tip; else b.removeAttribute('title');
  }
  depthRender();
  if (typeof renderSVis === 'function') renderSVis();
  if (typeof renderExpertVision === 'function') renderExpertVision();
}
/* A tap on the selector: Off clears everything, YOLO / ArUco / Depth toggle independently. */
function vmToggle(m) {
  if (VM.state !== 'ok') return;
  if (m === 'depth') { if (DEPTH.on || !depthBlock()) depthSet(!DEPTH.on); return; }
  if (m === 'off') depthSet(false);
  const cur = vmSel();
  return setVisionMode(m === 'off' ? 'off' : vmModeOf(m === 'yolo' ? !cur.yolo : cur.yolo, m === 'aruco' ? !cur.aruco : cur.aruco));
}
/* Detection pipelines (mode off | yolo | aruco | both): API 2 this page's overlay subscription, API 1 the device mode. */
async function setVisionMode(mode) {
  if (!VM_MODES.includes(mode)) return; // never send anything else (Depth is page-only)
  if (VM.state === 'ok' && VM.subs && !VM.busy) {
    VM.busy = true; renderVisionMode();
    try { await (mode === 'off' ? ovSet(false) : ovSet(true, mode)); } finally { VM.busy = false; renderVisionMode(); }
    return;
  }
  if (VM.state !== 'ok' || VM.busy || mode === VM.mode && (mode === 'off' || DET.enabled)) return;
  VM.busy = true; renderVisionMode();
  try {
    const data = await vmRequest('POST', mode === 'off' ? { mode } : { mode, enabled: true });
    VM.busy = false; S.detHoldUntil = 0; vmApply(data);
  } catch (e) {
    VM.busy = false;
    if (e.code === 'OPERATOR_REQUIRED') {
      renderVisionMode();
      opRequire('/api/v1/vision/mode', undefined, { retry: true }).then((ok) => { if (ok) setVisionMode(mode); });
      return;
    }
    toast(t('vm.fail'), 'err', vmErrCode(e));
    if (e.status === 502 || e.status === 404) vmFailed(e); else renderVisionMode();
  }
}
/* Hand-eye calibration needs markers for its live preview. Perception API 2: the drawer holds its own
   ArUco subscription while it is open and the page is visible; nothing global changes. API 1: the
   camera runs ArUco (Both when YOLO was on) with continuous inference, and the previous mode is
   restored on close or page hide. */
const CALVM = { prev: null, task: Promise.resolve(), open: false };
function calVisionMode(open) {
  CALVM.open = open;
  if (VM.subs) {
    CALVM.task = CALVM.task.then(async () => {
      const err = await subHold(CALSUB, open && !document.hidden ? ['aruco'] : null);
      if (err) toast(t('vm.fail.calib'), 'warn', vmErrCode(err), { path: '/api/v1/vision/subscribe', retry: () => { if (CALVM.open) calVisionMode(true); } });
    });
    return CALVM.task;
  }
  CALVM.task = CALVM.task.then(async () => {
    if (open) {
      if (CALVM.prev) return;
      let cur;
      try { cur = await vmRequest('GET'); vmApply(cur); } catch (e) { vmFailed(e); return; }
      const want = cur.mode === 'yolo' ? 'both' : cur.mode === 'off' ? 'aruco' : cur.mode;
      if (want === cur.mode && cur.enabled) return;
      CALVM.prev = { mode: cur.mode, enabled: !!cur.enabled };
      try { vmApply(await vmRequest('POST', { mode: want, enabled: true })); }
      catch (e) { CALVM.prev = null; toast(t('vm.fail.calib'), 'warn', vmErrCode(e), { path: '/api/v1/vision/mode', retry: () => { if (CALVM.open) calVisionMode(true); } }); }
    } else if (CALVM.prev) {
      const prev = CALVM.prev; CALVM.prev = null;
      const restore = () => vmRequest('POST', prev).then(vmApply);
      try { await restore(); }
      catch (e) { toast(t('vm.fail.restore', { m: prev.mode === 'both' ? 'YOLO + ArUco' : t('vm.' + prev.mode) }), 'warn', vmErrCode(e), { path: '/api/v1/vision/mode', retry: () => restore().catch(() => {}) }); }
    }
  });
  return CALVM.task;
}
function initVisionMode() {
  $('vmode').addEventListener('click', (e) => { const b = e.target.closest('button[data-vm]'); if (b && !b.disabled) vmToggle(b.dataset.vm); });
  i18nHooks.push(renderVisionMode);
  // a closed tab must not leave the camera in the calibration mode, nor keep its subscriptions
  window.addEventListener('pagehide', () => {
    if (CALVM.prev) { vmRequest('POST', CALVM.prev, true).catch(() => {}); CALVM.prev = null; }
    subDropNow(OVSUB); subDropNow(CALSUB);
  });
  // a hidden page infers nothing for itself: drop the subscriptions, take them again when visible
  document.addEventListener('visibilitychange', () => {
    if (!VM.subs) return;
    ovSync();
    subHold(CALSUB, CALVM.open && !document.hidden ? ['aruco'] : null);
  });
}

/* =================================================================
   Depth view: GET /api/v1/vision/depth (rebot-vision runs one depth inference on a fresh frame and sends a
   384x224 u16 millimetre map, "RDM1" body, docs/detection_api.md). One map per tap, or one per second at most
   while Live is on, the page is visible and the arm holds still. The values are the depth model's estimate
   (B601-DM, 2026-11-21: median error 89 mm at 0.32-0.45 m), so the legend is near / far in relative colour and
   metres only as "≈ x.xx m (估计)". The map canvas takes no pointer events: the readout listens on the video box.
   ================================================================= */
// rebot-vision accepts one map request per second, timed where it receives them. The gap is counted from the end of
// the previous exchange (DEPTH.lastAt), which is after the server received it, so jitter cannot cause a 429.
const DEPTH_MIN_GAP_MS = 1000;
const DEPTH = { on: false, live: false, busy: false, gen: 0, map: null, q: null, lastAt: -1e9, timer: null, st: '', stWarn: false, read: null };
/* Parse an RDM1 body -> { w, h, mm (Uint16Array, 0 = no depth), meta, near, far } (near / far: p2 / p98 mm). */
function depthParse(buf) {
  const u8 = new Uint8Array(buf);
  if (u8.length < 8 || String.fromCharCode(u8[0], u8[1], u8[2], u8[3]) !== 'RDM1') throw new Error('bad depth body');
  const dv = new DataView(buf), n = dv.getUint32(4, true);
  const meta = JSON.parse(new TextDecoder().decode(u8.subarray(8, 8 + n)));
  const w = meta.width | 0, h = meta.height | 0, off = 8 + n;
  if (w <= 0 || h <= 0 || u8.length < off + w * h * 2) throw new Error('short depth body');
  const mm = new Uint16Array(w * h);
  for (let i = 0; i < mm.length; i++) mm[i] = dv.getUint16(off + 2 * i, true);
  const st = meta.stats || {};
  let near = st.p2_mm, far = st.p98_mm;
  if (!(near > 0 && far > 0)) { // older body without stats: percentiles here
    const v = Array.from(mm).filter((x) => x > 0).sort((a, b) => a - b);
    near = v.length ? v[Math.round(0.02 * (v.length - 1))] : 0; far = v.length ? v[Math.round(0.98 * (v.length - 1))] : 0;
  }
  const mp = meta.mapping || {};
  const iw = mp.image_w || 640, ih = mp.image_h || 360;
  // the grid's place on the visible image, normalised (rebot-vision: the whole image, stretched)
  const box = [(mp.x0 ?? 0) / iw, (mp.y0 ?? 0) / ih, (mp.x1 ?? iw) / iw, (mp.y1 ?? ih) / ih];
  return { w, h, mm, meta, near, far, box };
}
// turbo colour map (Mikhailov 2019, polynomial fit), 256 entries over turbo 0.06 .. 0.94 (the near-black ends left
// out so far depths stay visible over the video); index 0 = far (blue) .. 255 = near (red)
const TURBO = (() => {
  const lut = new Uint8ClampedArray(256 * 3);
  for (let i = 0; i < 256; i++) {
    const x = 0.06 + 0.88 * i / 255;
    lut[i * 3] = 255 * (0.13572138 + x * (4.6153926 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943)))));
    lut[i * 3 + 1] = 255 * (0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604)))));
    lut[i * 3 + 2] = 255 * (0.1066733 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973)))));
  }
  return lut;
})();
// colour index of a depth: near (p2) -> 255, far (p98) -> 0, clamped
const depthIdx = (map, v) => Math.round(255 * (1 - Math.min(1, Math.max(0, (v - map.near) / Math.max(1, map.far - map.near)))));
function depthImage(map) {
  const c = document.createElement('canvas'); c.width = map.w; c.height = map.h;
  const g = c.getContext('2d'), img = g.createImageData(map.w, map.h), d = img.data;
  for (let i = 0; i < map.mm.length; i++) {
    const v = map.mm[i];
    if (!v) continue; // transparent: no depth
    const k = depthIdx(map, v) * 3;
    d[i * 4] = TURBO[k]; d[i * 4 + 1] = TURBO[k + 1]; d[i * 4 + 2] = TURBO[k + 2]; d[i * 4 + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  return c;
}
function depthDraw() {
  const cv = $('depth-overlay');
  if (!cv || cv.hidden) return;
  // sized from the canvas itself (the padding box of the video box, as the video element), not the bordered box
  const { cssW, cssH, dpr } = fitCanvas(cv, cv);
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, cssW, cssH);
  const m = DEPTH.map, r = getVideoContentRect(cssW, cssH);
  if (!m || !r) return;
  g.imageSmoothingEnabled = false; // one block per grid cell: what you see is what the readout reports
  g.drawImage(m.img, r.x + m.box[0] * r.w, r.y + m.box[1] * r.h, (m.box[2] - m.box[0]) * r.w, (m.box[3] - m.box[1]) * r.h);
}
/* Depth (mm, 0 = none) at a point normalised to the video image, or null outside the grid. */
function depthAt(u, v) {
  const m = DEPTH.map;
  if (!m) return null;
  const gx = Math.floor((u - m.box[0]) / (m.box[2] - m.box[0]) * m.w), gy = Math.floor((v - m.box[1]) / (m.box[3] - m.box[1]) * m.h);
  if (gx < 0 || gy < 0 || gx >= m.w || gy >= m.h) return null;
  return m.mm[gy * m.w + gx];
}
/* Pointer -> point normalised to the video image (same frame as depthDraw: the canvas box). */
function depthPoint(e) {
  const c = $('depth-overlay').getBoundingClientRect(), r = getVideoContentRect(c.width, c.height);
  if (!r) return null;
  const u = (e.clientX - c.left - r.x) / r.w, v = (e.clientY - c.top - r.y) / r.h;
  return u >= 0 && u <= 1 && v >= 0 && v <= 1 ? [u, v] : null;
}
function depthReadAt(e) {
  const el = $('depth-read'), pt = DEPTH.map ? depthPoint(e) : null;
  const mm = pt ? depthAt(pt[0], pt[1]) : null;
  if (mm == null) { el.hidden = true; DEPTH.read = null; return; }
  const box = $('video-box').getBoundingClientRect();
  DEPTH.read = { u: pt[0], v: pt[1], mm };
  setText(el, mm ? t('depth.read', { mm: String(mm) }) : t('depth.read.none'));
  el.style.left = Math.round(e.clientX - box.left) + 'px'; el.style.top = Math.round(e.clientY - box.top) + 'px';
  el.hidden = false;
  // keep the readout inside the video box (flip to the left of the pointer near the right edge)
  el.style.transform = e.clientX - box.left + el.offsetWidth + 16 > box.width ? 'translate(calc(-100% - 12px), -50%)' : '';
}
const depthM = (mm) => t('depth.m', { m: (mm / 1000).toFixed(2) });
function depthRender() {
  const panel = $('depth-panel');
  if (!panel) return;
  // the selector's Depth toggle (renderVisionMode) shows the state; depth lost (model unloaded, over budget) ends the view
  if (DEPTH.on && depthBlock()) { depthSet(false); return; }
  panel.hidden = !DEPTH.on;
  $('depth-overlay').hidden = !DEPTH.on;
  if (!DEPTH.on) { $('depth-read').hidden = true; if (typeof exSyncCtx === 'function') exSyncCtx(); return; }
  panel.setAttribute('aria-label', t('depth.title'));
  setText($('dp-title'), t('depth.title'));
  const live = $('dp-live');
  setText(live, t('depth.live')); live.title = t('depth.live.tip');
  live.setAttribute('aria-pressed', String(DEPTH.live));
  const rf = $('dp-refresh'); setText(rf, t('depth.refresh')); rf.disabled = DEPTH.busy;
  $('dp-x').setAttribute('aria-label', t('depth.close')); $('dp-x').title = t('depth.close');
  setText($('dp-op-k'), t('depth.opacity'));
  setText($('dp-note'), t('depth.note'));
  setText($('dp-near'), t('depth.near')); setText($('dp-far'), t('depth.far'));
  const m = DEPTH.map;
  setText($('dp-near-m'), m ? depthM(m.near) : '--'); setText($('dp-far-m'), m ? depthM(m.far) : '--');
  const st = $('dp-status');
  let txt = DEPTH.st ? t(DEPTH.st) : '';
  if (!txt && m) txt = t('depth.frame', { id: m.meta.frame_id, age: t('depth.age', { s: Math.max(0, (performance.now() - m.at) / 1000).toFixed(1) }) });
  setText(st, txt);
  st.classList.toggle('is-warn', DEPTH.stWarn);
  if (typeof exSyncCtx === 'function') exSyncCtx();
}
function depthStatus(key, warn = false) { DEPTH.st = key; DEPTH.stWarn = warn; depthRender(); }
function depthClearMap() {
  DEPTH.map = null; DEPTH.q = null; DEPTH.read = null;
  $('depth-read').hidden = true;
  depthDraw();
}
function depthStop() { clearTimeout(DEPTH.timer); DEPTH.timer = null; }
const depthMoved = (q0, q) => !!(q0 && q && q.some((v, i) => i !== GRIP && i < q0.length && Math.abs(v - q0[i]) > SNAP_MOVE_RAD));
/* Next request no sooner than DEPTH_MIN_GAP_MS after the previous one ended, and `ms` from now. */
function depthSchedule(ms = 0) {
  depthStop();
  if (!DEPTH.on || !DEPTH.live || document.hidden) return;
  const wait = Math.max(ms, DEPTH.lastAt + DEPTH_MIN_GAP_MS - performance.now(), 0);
  DEPTH.timer = setTimeout(() => { DEPTH.timer = null; depthShot(); }, wait);
}
async function depthFetch(retried = false) {
  const ctl = new AbortController();
  const tm = setTimeout(() => ctl.abort(), 4500); // armd gives the detector 3 s
  try {
    const res = await fetch('/api/v1/vision/depth', { signal: ctl.signal, cache: 'no-store' });
    if (res.ok) return { map: depthParse(await res.arrayBuffer()) };
    let data = null;
    try { data = await res.json(); } catch (_) {}
    if (res.status === 403 && data && data.code === 'OPERATOR_REQUIRED') {
      clearTimeout(tm);
      if (!retried && await opRequire('/api/v1/vision/depth')) return depthFetch(true);
      return { status: 403, key: 'depth.st.op' };
    }
    const ra = Number(res.headers.get('retry-after'));
    const err = data && data.error;
    const key = res.status === 429 ? 'depth.st.rate' : err === 'depth busy' ? 'depth.st.busy'
      : err === 'depth unavailable' || err === 'depth not metric' || res.status === 404 ? 'depth.st.na' : 'depth.st.fail';
    return { status: res.status, key, retryMs: (data && Number(data.retry_after_ms)) || (ra > 0 ? ra * 1000 : DEPTH_MIN_GAP_MS) };
  } catch (_) {
    return { status: 0, key: 'depth.st.fail' };
  } finally { clearTimeout(tm); }
}
async function depthShot() {
  if (!DEPTH.on || DEPTH.busy || document.hidden) return;
  const wait = DEPTH.lastAt + DEPTH_MIN_GAP_MS - performance.now();
  if (wait > 0) { // a tap within a second of the previous request: run it when the server takes it
    clearTimeout(DEPTH.timer); DEPTH.timer = setTimeout(() => { DEPTH.timer = null; depthShot(); }, wait);
    return;
  }
  const gen = DEPTH.gen, q0 = S.obs && Array.isArray(S.obs.positions) ? S.obs.positions.slice() : null;
  DEPTH.busy = true; DEPTH.lastAt = performance.now();
  if (!DEPTH.map) DEPTH.st = 'depth.st.load';
  depthRender();
  const r = await depthFetch();
  DEPTH.busy = false; DEPTH.lastAt = performance.now();
  if (gen !== DEPTH.gen || !DEPTH.on) { depthRender(); return; } // the view was closed meanwhile
  const qNow = S.obs && Array.isArray(S.obs.positions) ? S.obs.positions : null;
  if (r.map && depthMoved(q0, qNow)) { // the camera moved while the map was computed: it no longer lines up
    DEPTH.live = false; depthClearMap(); depthStatus('depth.st.moved', true); return;
  }
  if (r.map) {
    r.map.img = depthImage(r.map); r.map.at = performance.now();
    DEPTH.map = r.map; DEPTH.q = q0; DEPTH.st = ''; DEPTH.stWarn = false;
    $('depth-read').hidden = true; DEPTH.read = null;
    depthDraw(); depthRender();
    depthSchedule(DEPTH_MIN_GAP_MS);
    return;
  }
  if (r.status === 403 || r.key === 'depth.st.na') DEPTH.live = false;
  depthStatus(r.key, true);
  depthSchedule(r.status === 429 ? r.retryMs : DEPTH_MIN_GAP_MS);
}
function depthSet(on) {
  if (on === DEPTH.on) return;
  DEPTH.on = on; DEPTH.gen++;
  depthStop();
  if (!on) { DEPTH.live = false; DEPTH.st = ''; depthClearMap(); }
  renderVisionMode(); // the Depth toggle and the panel
  if (on) { depthDraw(); depthShot(); }
}
function depthSetLive(on) {
  DEPTH.live = on;
  if (on && DEPTH.st === 'depth.st.moved') DEPTH.st = '';
  depthRender();
  if (on) { if (!DEPTH.busy) depthSchedule(0); } else depthStop();
}
/* Telemetry: the arm moved away from the map's pose -> the map no longer lines up; stop Live and drop it. */
function depthOnObs(o) {
  if (!DEPTH.on || !DEPTH.q || !depthMoved(DEPTH.q, o.positions)) return;
  DEPTH.live = false; depthStop(); depthClearMap(); depthStatus('depth.st.moved', true);
}
function initDepth() {
  // on / off: the Depth toggle of the vision selector (vmToggle); the panel's x turns it off
  $('dp-x').addEventListener('click', () => depthSet(false));
  $('dp-live').addEventListener('click', () => depthSetLive(!DEPTH.live));
  $('dp-refresh').addEventListener('click', () => { if (DEPTH.st === 'depth.st.moved') DEPTH.st = ''; depthShot(); });
  const op = $('dp-op'), cv = $('depth-overlay');
  const pref = Number(store('rebot.depth.opacity'));
  if (pref >= 10 && pref <= 100) op.value = String(pref);
  const applyOp = () => { cv.style.opacity = String(Number(op.value) / 100); };
  op.addEventListener('input', () => { applyOp(); store('rebot.depth.opacity', op.value); });
  applyOp();
  // legend: near (red) on the left to far (blue) on the right
  const stops = [];
  for (let i = 0; i <= 8; i++) { const k = Math.round(255 * (1 - i / 8)) * 3; stops.push(`rgb(${TURBO[k]},${TURBO[k + 1]},${TURBO[k + 2]})`); }
  $('dp-bar').style.background = `linear-gradient(90deg, ${stops.join(',')})`;
  // readout: hover with a mouse, tap on touch; the canvas itself never takes pointer events
  const vb = $('video-box');
  vb.addEventListener('pointermove', (e) => { if (DEPTH.map && e.pointerType === 'mouse' && !(e.target.closest && e.target.closest('.hud, .depth-panel, .grasp-bar, .grasp-pick'))) depthReadAt(e); });
  vb.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') $('depth-read').hidden = true; });
  vb.addEventListener('click', (e) => { if (DEPTH.map && !(e.target.closest && e.target.closest('.hud, .depth-panel, .grasp-bar, .grasp-pick, .vhint-btn'))) depthReadAt(e); });
  if (window.ResizeObserver) new ResizeObserver(() => depthDraw()).observe(vb);
  for (const el of [videoEl, mjpegEl]) if (el) el.addEventListener(el === videoEl ? 'loadedmetadata' : 'load', () => depthDraw());
  setInterval(() => { if (DEPTH.on && DEPTH.map && !DEPTH.st) depthRender(); }, 1000); // frame age
  document.addEventListener('visibilitychange', () => {
    if (!DEPTH.on) return;
    if (document.hidden) { depthStop(); if (DEPTH.live) depthStatus('depth.st.hidden'); }
    else { if (DEPTH.st === 'depth.st.hidden') depthStatus(''); depthSchedule(0); }
  });
  i18nHooks.push(depthRender);
}

