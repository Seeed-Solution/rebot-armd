/* =================================================================
   3D arm view, loaded on first open: three.js + urdf-loader (/assets/three-urdf.min.js, embedded),
   URDF and decimated meshes from the kit asset directory (/assets/model/). Without meshes it draws a
   stick figure through the URDF joint frames (VIK). Angles: motor angle = URDF angle for J1..J6; J7 maps
   to the finger travel via the profile gripper_policy (CFG.gripClosed / gripOpen). Renders at most
   ~30 fps and only when the pose, the scene or the camera changed.
   ================================================================= */
const V3 = window.V3 = { open: false, init: null, T: null, r: null, scene: null, cam: null, ctl: null, root: null, solid: null, ghost: null,
  extras: null, sandboxGroup: null, sandboxMeshes: null, simSceneGroup: null, simFreeMap: null, simModeActive: false, simSceneKey: '',
  mode: '', dirty: true, last: 0, raf: 0, key: '', geomKey: '', sceneAt: 0, renders: 0, ghostMat: null };
const V3_FINGER_M = 0.0425; // URDF left/right_finger_joint upper limit (m)
const V3_LINK_OF = { gripper_body: 'end_link', finger_L: 'left_finger', finger_R: 'right_finger' };
const v3Geo = new Map();
function v3Toggle(open) {
  V3.open = open;
  $('view3d').hidden = !open;
  document.body.classList.toggle('v3-open', open);
  for (const [id, on] of [['vs-video', !open], ['vs-3d', open]]) { const b = $(id); b.setAttribute('aria-checked', String(on)); b.tabIndex = on ? 0 : -1; }
  if (open) v3Start(); // the joysticks and sliders stay on top and keep working; the view rotates outside them
  else { cancelAnimationFrame(V3.raf); V3.raf = 0; }
}
// height of the safety card, for the simple-mode overlay layout (view switch and pose card sit below it)
new ResizeObserver(() => { document.documentElement.style.setProperty('--safety-h', $('safety').offsetHeight + 'px'); }).observe($('safety'));
async function v3Start() {
  try { await (V3.init ||= v3Init()); } catch (e) { V3.init = null; setText($('v3-status'), t('v3.err', { m: e.message || String(e) })); return; }
  if (!V3.open) return;
  if (Date.now() - V3.sceneAt > 5000) v3Extras();
  cwV3Sync();
  v3Resize();
  v3Frame();
  if (!V3.raf) V3.raf = requestAnimationFrame(v3Tick);
}
async function v3Init() {
  setText($('v3-status'), t('v3.loading'));
  if (schemaReady) await Promise.race([schemaReady, new Promise((r) => setTimeout(r, 1500))]);
  const T = V3.T = await import('/assets/three-urdf.min.js');
  const cv = $('v3-canvas');
  V3.r = new T.WebGLRenderer({ canvas: cv, antialias: true, preserveDrawingBuffer: true });
  V3.r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  const scene = V3.scene = new T.Scene();
  scene.background = new T.Color(0xeef2ea);
  scene.add(new T.HemisphereLight(0xffffff, 0x80857c, 2.4));
  const sun = new T.DirectionalLight(0xffffff, 1.8); sun.position.set(1.2, 2.0, 1.0); scene.add(sun);
  V3.cam = new T.PerspectiveCamera(42, 1, 0.01, 20);
  V3.ctl = new T.OrbitControls(V3.cam, cv);
  V3.ctl.target.set(0.05, 0.22, 0); V3.ctl.minDistance = 0.25; V3.ctl.maxDistance = 4; V3.ctl.update();
  V3.ctl.addEventListener('change', () => { V3.dirty = true; });
  // URDF base_link (z up) -> three.js (y up)
  V3.root = new T.Group(); V3.root.rotation.x = -Math.PI / 2; scene.add(V3.root);
  V3.ghostMat = new T.MeshBasicMaterial({ color: 0x3b82f6, transparent: true, opacity: 0.28, depthWrite: false });
  let urdf = null, man = null;
  const isSoArm = S.profileId && S.profileId.startsWith('so_arm');
  const urdfFile = isSoArm ? '/assets/model/so101_new_calib.urdf' : '/assets/model/reBot-DevArm_gripper.urdf';
  const manFile = isSoArm ? '/assets/model/so101_meshes/manifest.json' : '/assets/model/meshes/manifest.json';
  const binDir = isSoArm ? '/assets/model/so101_meshes/' : '/assets/model/meshes/';
  try {
    const [u, m] = await Promise.all([fetch(urdfFile), fetch(manFile)]);
    if (u.ok && m.ok) { urdf = await u.text(); man = await m.json(); }
    if (man && man.format === 'rbm1' && /^[\w.-]+$/.test(man.file || '')) {
      const b = await fetch(binDir + man.file);
      man.buf = b.ok ? await b.arrayBuffer() : null;
    } else man = null;
  } catch (_) { man = null; /* stick figure */ }
  if (urdf && man && man.links && man.buf && man.buf.byteLength === man.bytes) {
    try { V3.solid = await v3Robot(T, urdf, man, false); V3.ghost = await v3Robot(T, urdf, man, true); V3.mode = 'mesh'; }
    catch (e) { console.warn('3D meshes', e); V3.solid = V3.ghost = null; }
  }
  if (!V3.solid) { V3.solid = v3Stick(T, false); V3.ghost = v3Stick(T, true); V3.mode = 'stick'; }
  V3.ghost.obj.renderOrder = 2;
  V3.root.add(V3.solid.obj, V3.ghost.obj);
  new ResizeObserver(() => { if (V3.open) v3Resize(); }).observe($('view3d'));
  V3.key = '';
}
// One part of meshes/arm.bin (layout: scripts/decimate_meshes.py): Uint16 positions quantized to the
// part's bbox, then Uint16/Uint32 triangle indices. Returns a non-indexed geometry whose normals are
// smoothed across edges flatter than V3_CREASE and kept sharp across steeper ones.
const V3_CREASE = Math.cos(35 * Math.PI / 180);
function v3Part(T, buf, pt) {
  const nv = pt.verts, nf = pt.faces, bb = pt.bbox;
  const q = new Uint16Array(buf, pt.offset, nv * 3);
  const idx = new (pt.index_bytes === 4 ? Uint32Array : Uint16Array)(buf, pt.offset + Math.ceil(nv * 6 / 4) * 4, nf * 3);
  const P = new Float32Array(nv * 3);
  for (let i = 0; i < P.length; i++) { const k = i % 3; P[i] = bb[k] + q[i] * (bb[k + 3] - bb[k]) / 65535; }
  const A = new Float32Array(nf * 3), N = new Float32Array(nf * 3); // area-weighted and unit face normals
  const cnt = new Uint32Array(nv + 1);
  for (let f = 0; f < nf; f++) {
    const a = idx[3 * f] * 3, b = idx[3 * f + 1] * 3, c = idx[3 * f + 2] * 3;
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    const x = uy * vz - uz * vy, y = uz * vx - ux * vz, z = ux * vy - uy * vx, l = Math.hypot(x, y, z) || 1;
    A[3 * f] = x; A[3 * f + 1] = y; A[3 * f + 2] = z; N[3 * f] = x / l; N[3 * f + 1] = y / l; N[3 * f + 2] = z / l;
    for (let j = 0; j < 3; j++) cnt[idx[3 * f + j] + 1]++;
  }
  for (let v = 0; v < nv; v++) cnt[v + 1] += cnt[v];
  const inc = new Uint32Array(nf * 3), fill = cnt.slice(0, nv); // faces around each vertex
  for (let f = 0; f < nf; f++) for (let j = 0; j < 3; j++) inc[fill[idx[3 * f + j]]++] = f;
  const pos = new Float32Array(nf * 9), nor = new Float32Array(nf * 9);
  for (let f = 0; f < nf; f++) for (let j = 0; j < 3; j++) {
    const v = idx[3 * f + j], o = 9 * f + 3 * j;
    let x = 0, y = 0, z = 0;
    for (let k = cnt[v]; k < cnt[v + 1]; k++) {
      const g = inc[k];
      if (N[3 * f] * N[3 * g] + N[3 * f + 1] * N[3 * g + 1] + N[3 * f + 2] * N[3 * g + 2] >= V3_CREASE) { x += A[3 * g]; y += A[3 * g + 1]; z += A[3 * g + 2]; }
    }
    const l = Math.hypot(x, y, z) || 1;
    pos[o] = P[3 * v]; pos[o + 1] = P[3 * v + 1]; pos[o + 2] = P[3 * v + 2];
    nor[o] = x / l; nor[o + 1] = y / l; nor[o + 2] = z / l;
  }
  const geo = new T.BufferGeometry();
  geo.setAttribute('position', new T.Float32BufferAttribute(pos, 3));
  geo.setAttribute('normal', new T.Float32BufferAttribute(nor, 3));
  return geo;
}
function v3Robot(T, urdf, man, ghost) {
  const loader = new T.URDFLoader();
  loader.packages = (pkg) => '/assets/model/' + pkg;
  loader.loadMeshCb = (path, _mgr, _mat, done) => {
    const base = path.split('/').pop().replace(/\.stl$/i, '');
    const parts = man.links[V3_LINK_OF[base] || base] || [];
    const g = new T.Group();
    for (const pt of parts) {
      if (!v3Geo.has(pt.part)) v3Geo.set(pt.part, v3Part(T, man.buf, pt));
      const mat = ghost ? V3.ghostMat
        : new T.MeshStandardMaterial({ color: new T.Color().setRGB(pt.rgb[0], pt.rgb[1], pt.rgb[2], T.SRGBColorSpace), metalness: 0.25, roughness: 0.6 });
      g.add(new T.Mesh(v3Geo.get(pt.part), mat));
    }
    done(g);
  };
  const robot = loader.parse(urdf);
  return Promise.resolve({
    obj: robot,
    set(q) {
      const isSoArm = S.profileId && S.profileId.startsWith('so_arm');
      if (isSoArm) {
        const jNames = S.jointNames || ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'];
        for (let i = 0; i < Math.min(q.length, jNames.length); i++) {
          robot.setJointValue(jNames[i], q[i]);
        }
      } else {
        const m = toModel6(q);
        for (let i = 0; i < 6; i++) robot.setJointValue('joint' + (i + 1), m[i]);
        if (q.length > 6) { const d = gripFrac(q[6]) * V3_FINGER_M; robot.setJointValue('left_finger_joint', d); robot.setJointValue('right_finger_joint', d); }
      }
    },
    setHideFingers(hide) {
      for (const k of ['left_finger', 'right_finger', 'left_finger_link', 'right_finger_link', 'finger_left', 'finger_right']) {
        if (robot.links && robot.links[k]) robot.links[k].visible = !hide;
      }
    },
    end() {
      const link = robot.links.end_link || robot.links.gripper_frame_link || robot.links.moving_jaw_so101_v1_link || robot.links.gripper_link;
      if (!link) return [0, 0, 0];
      const v = link.getWorldPosition(new T.Vector3());
      V3.root.worldToLocal(v);
      return [v.x, v.y, v.z];
    },
  });
}
// stick figure through the URDF joint frames: base, J1..J6 origins, end_link
function v3Stick(T, ghost) {
  const obj = new T.Group();
  const bone = ghost ? V3.ghostMat : new T.MeshStandardMaterial({ color: 0x3a3f3b, roughness: 0.6 });
  const joint = ghost ? V3.ghostMat : new T.MeshStandardMaterial({ color: 0x84b521, roughness: 0.5 });
  const cyl = new T.CylinderGeometry(0.014, 0.014, 1, 14), sph = new T.SphereGeometry(0.022, 18, 12);
  const mk = (geo, mat) => { const m = new T.Mesh(geo, mat); obj.add(m); return m; };
  const bones = Array.from({ length: 7 }, () => mk(cyl, bone)), balls = Array.from({ length: 8 }, () => mk(sph, joint));
  const up = new T.Vector3(0, 1, 0), a = new T.Vector3(), b = new T.Vector3(), d = new T.Vector3();
  return {
    obj,
    set(q) {
      if (S.armDof !== 6 || (S.profileId && S.profileId.startsWith('so_arm'))) {
        return;
      }
      const f = VIK.frames(toModel6(q));
      const pts = [[0, 0, 0], ...f.o, f.p];
      pts.forEach((p, i) => balls[i].position.set(p[0], p[1], p[2]));
      for (let i = 0; i < 7; i++) {
        a.set(...pts[i]); b.set(...pts[i + 1]); d.subVectors(b, a);
        const L = d.length(), m = bones[i];
        m.visible = L > 1e-4;
        if (!m.visible) continue;
        m.position.copy(a).addScaledVector(d, 0.5); m.scale.set(1, L, 1); m.quaternion.setFromUnitVectors(up, d.normalize());
      }
    },
    setHideFingers() {},
    end() { const p = balls[7].position; return [p.x, p.y, p.z]; },
  };
}
// floor grid, workspace box, keep-out boxes, named pose markers (URDF base frame)
async function v3Extras() {
  const T = V3.T;
  V3.sceneAt = Date.now();
  let boxes = null, poses = [];
  try { const r = await fetch('/api/v1/workspace/keepout'); if (r.ok) { const j = await r.json(); boxes = Array.isArray(j) ? j : j.boxes; } } catch (_) {}
  if (!Array.isArray(boxes)) boxes = (S.wsPolicy && S.wsPolicy.keepout_boxes) || [];
  if (S.caps && S.caps.poses) { try { const r = await fetch('/api/v1/poses'); if (r.ok) { const j = await r.json(); poses = Array.isArray(j) ? j : j.poses || []; } } catch (_) {} }
  if (V3.extras) { V3.root.remove(V3.extras); V3.extras.traverse((o) => { if (o.geometry) o.geometry.dispose(); }); }
  const g = V3.extras = new T.Group();
  const ws = S.wsPolicy;
  const floor = ws && Number.isFinite(ws.floor_z_m) ? ws.floor_z_m : 0;
  const grid = new T.GridHelper(1.2, 24, 0xb4bbae, 0xd9ded4);
  grid.rotation.x = Math.PI / 2; grid.position.z = floor; g.add(grid);
  const box = (lo, hi, color, fill, name, opacity = 1) => {
    const s = [0, 1, 2].map((k) => Math.max(1e-3, hi[k] - lo[k]));
    const geo = new T.BoxGeometry(s[0], s[1], s[2]);
    const c = [0, 1, 2].map((k) => (hi[k] + lo[k]) / 2);
    const edges = new T.LineSegments(new T.EdgesGeometry(geo), new T.LineBasicMaterial({ color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1 }));
    edges.position.set(c[0], c[1], c[2]); edges.name = name; g.add(edges);
    if (fill) { const m = new T.Mesh(geo, new T.MeshBasicMaterial({ color, transparent: true, opacity: 0.18, depthWrite: false })); m.position.copy(edges.position); m.name = name; g.add(m); }
  };
  if (ws && Array.isArray(ws.box_min_m) && Array.isArray(ws.box_max_m)) {
    const lo = ws.box_min_m.slice(), hi = ws.box_max_m.slice();
    lo[2] = Math.max(lo[2], floor);
    if (hi[2] > lo[2]) box(lo, hi, 0x6f9420, false, 'workspace', 0.3); // faint: the box is far larger than the arm
  }
  for (const b of boxes) if (Array.isArray(b.min_m) && Array.isArray(b.max_m)) box(b.min_m, b.max_m, 0xdc2626, true, 'keepout:' + b.name);
  const dot = new T.SphereGeometry(0.012, 12, 8), dm = new T.MeshBasicMaterial({ color: 0xf59e0b });
  for (const p of poses) {
    const v = p && p.tcp && p.tcp.position;
    if (Array.isArray(v)) { const m = new T.Mesh(dot, dm); m.position.set(v[0], v[1], v[2]); m.name = 'pose:' + p.name; g.add(m); }
  }
  V3.root.add(g);
  V3.dirty = true;
}
const V3_ARM_BODIES = new Set(['base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'finger_left', 'finger_right',
  'shoulder_link', 'upper_arm_link', 'lower_arm_link', 'wrist_link', 'gripper_link', 'gripper_frame_link', 'moving_jaw_so101_v1_link']);
const V3_SKIP_END_GEOMS = new Set(['palm', 'motor_housing', 'ee_mount', 'camera_mount']);
function _v3Mat3ToQuat(q, m9, mo = 0) {
  const n = m9[mo], s = m9[mo + 1], r = m9[mo + 2];
  const a = m9[mo + 3], o = m9[mo + 4], c = m9[mo + 5];
  const l = m9[mo + 6], u = m9[mo + 7], d = m9[mo + 8];
  const h = n + o + d;
  let qx, qy, qz, qw;
  if (h > 0) {
    const p = 0.5 / Math.sqrt(h + 1.0);
    qw = 0.25 / p; qx = (u - c) * p; qy = (r - l) * p; qz = (a - s) * p;
  } else if (n > o && n > d) {
    const p = 2.0 * Math.sqrt(1.0 + n - o - d);
    qw = (u - c) / p; qx = 0.25 * p; qy = (s + a) / p; qz = (r + l) / p;
  } else if (o > d) {
    const p = 2.0 * Math.sqrt(1.0 + o - n - d);
    qw = (r - l) / p; qx = (s + a) / p; qy = 0.25 * p; qz = (c + u) / p;
  } else {
    const p = 2.0 * Math.sqrt(1.0 + d - n - o);
    qw = (a - s) / p; qx = (r + l) / p; qy = (c + u) / p; qz = 0.25 * p;
  }
  q.set(qx, qy, qz, qw);
}
function _v3MakeMjGeo(T, type, sx, sy, sz) {
  let geo = null;
  if (type === 2) geo = new T.SphereGeometry(Math.max(1e-4, sx), 20, 14);
  else if (type === 3) {
    geo = typeof T.CapsuleGeometry === 'function'
      ? new T.CapsuleGeometry(Math.max(1e-4, sx), Math.max(1e-4, sy * 2), 6, 16)
      : new T.CylinderGeometry(Math.max(1e-4, sx), Math.max(1e-4, sx), Math.max(1e-4, (sy + sx) * 2), 18);
    geo.rotateX(Math.PI / 2);
  } else if (type === 4) {
    geo = new T.SphereGeometry(1, 20, 14);
    geo.scale(Math.max(1e-4, sx), Math.max(1e-4, sy), Math.max(1e-4, sz));
  } else if (type === 5) {
    geo = new T.CylinderGeometry(Math.max(1e-4, sx), Math.max(1e-4, sx), Math.max(1e-4, sy * 2), 20);
    geo.rotateX(Math.PI / 2);
  } else if (type === 6) {
    geo = new T.BoxGeometry(Math.max(1e-4, sx * 2), Math.max(1e-4, sy * 2), Math.max(1e-4, sz * 2));
  }
  return geo;
}
function v3ClearGroup(grp) {
  if (!grp) return;
  if (V3.root) V3.root.remove(grp);
  grp.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) o.material.dispose();
  });
}
function v3SyncWasmGeoms() {
  const T = V3.T, sim = window.WASM_SIM;
  if (!T || !V3.root || !sim || !sim.ready || !sim.model || !sim.data) return false;
  const m = sim.model, d = sim.data;
  const customTool = sim.bundle?.active_tool && sim.bundle.active_tool !== 'default';
  if (V3.solid?.setHideFingers) V3.solid.setHideFingers(Boolean(customTool));
  if (V3.ghost?.setHideFingers) V3.ghost.setHideFingers(Boolean(customTool));
  if (!V3.sandboxGroup || V3.sandboxKey !== sim.bundleKey) {
    v3ClearGroup(V3.sandboxGroup);
    const grp = V3.sandboxGroup = new T.Group();
    const list = V3.sandboxMeshes = [];
    for (let g = 0; g < m.ngeom; g++) {
      const type = m.geom_type[g];
      if (type === 0 || type === 1) continue; // skip infinite plane / heightfield
      const rgba = m.geom_rgba;
      if (rgba[g * 4 + 3] <= 0.01) continue;
      const bname = sim.mjName(m.name_bodyadr, m.geom_bodyid[g]);
      const gname = sim.mjName(m.name_geomadr, g);
      if (V3_ARM_BODIES.has(bname)) continue;
      if (bname === 'end_link' && V3_SKIP_END_GEOMS.has(gname)) continue;
      const sx = m.geom_size[g * 3], sy = m.geom_size[g * 3 + 1], sz = m.geom_size[g * 3 + 2];
      let geo = _v3MakeMjGeo(T, type, sx, sy, sz);
      if (!geo) {
        const rb = Math.max(0.01, m.geom_rbound ? m.geom_rbound[g] : 0.02);
        geo = new T.BoxGeometry(rb, rb, rb);
      }
      const r = rgba[g * 4], gr = rgba[g * 4 + 1], b = rgba[g * 4 + 2], a = rgba[g * 4 + 3];
      const mat = new T.MeshStandardMaterial({
        color: new T.Color().setRGB(r, gr, b, T.SRGBColorSpace),
        metalness: 0.15, roughness: 0.65,
        transparent: a < 0.99, opacity: a,
      });
      const mesh = new T.Mesh(geo, mat);
      mesh.name = 'wasm:' + (gname || g);
      grp.add(mesh);
      list.push({ g, mesh, mat, rgba: [r, gr, b, a] });
    }
    V3.root.add(grp);
    V3.sandboxKey = sim.bundleKey;
    V3.dirty = true;
  }
  const posArr = d.geom_xpos, matArr = d.geom_xmat, rgbaArr = m.geom_rgba;
  const qNew = V3._qTmp || (V3._qTmp = new T.Quaternion());
  let changed = false;
  for (const item of V3.sandboxMeshes) {
    const g = item.g, po = g * 3, mo = g * 9, ro = g * 4;
    const x = posArr[po], y = posArr[po + 1], z = posArr[po + 2];
    const mp = item.mesh.position;
    if (Math.abs(mp.x - x) > 1e-4 || Math.abs(mp.y - y) > 1e-4 || Math.abs(mp.z - z) > 1e-4) {
      mp.set(x, y, z); changed = true;
    }
    _v3Mat3ToQuat(qNew, matArr, mo);
    const qOld = item.mesh.quaternion;
    if (Math.abs(qOld.x - qNew.x) + Math.abs(qOld.y - qNew.y) + Math.abs(qOld.z - qNew.z) + Math.abs(qOld.w - qNew.w) > 5e-4) {
      qOld.copy(qNew); changed = true;
    }
    const r = rgbaArr[ro], gr = rgbaArr[ro + 1], b = rgbaArr[ro + 2], a = rgbaArr[ro + 3];
    if (Math.abs(item.rgba[0] - r) + Math.abs(item.rgba[1] - gr) + Math.abs(item.rgba[2] - b) + Math.abs(item.rgba[3] - a) > 1e-3) {
      item.rgba = [r, gr, b, a];
      item.mat.color.setRGB(r, gr, b, T.SRGBColorSpace);
      item.mat.opacity = a;
      item.mat.transparent = a < 0.99;
      changed = true;
    }
  }
  if (changed) V3.dirty = true;
  return changed;
}
function _v3ParseNums(s, def) {
  if (!s) return def;
  const a = s.trim().split(/\s+/).map(Number);
  return a.every(Number.isFinite) ? a : def;
}
function _v3TypeNum(typeStr) {
  return ({ sphere: 2, capsule: 3, ellipsoid: 4, cylinder: 5, box: 6 })[(typeStr || 'sphere').toLowerCase()] || 0;
}
async function v3SyncSimScene(bundle, simState) {
  const T = V3.T;
  if (!T || !V3.root || !bundle) return;
  const key = `${bundle.active_scene}:${bundle.active_tool}`;
  const customTool = bundle.active_tool && bundle.active_tool !== 'default';
  if (V3.solid?.setHideFingers) V3.solid.setHideFingers(Boolean(customTool));
  if (V3.ghost?.setHideFingers) V3.ghost.setHideFingers(Boolean(customTool));
  if (!V3.simSceneGroup || V3.simSceneKey !== key) {
    v3ClearGroup(V3.simSceneGroup);
    const grp = V3.simSceneGroup = new T.Group();
    const freeMap = V3.simFreeMap = new Map();
    if (bundle.composed_xml) {
      const doc = new DOMParser().parseFromString(bundle.composed_xml, 'text/xml');
      const wb = doc.querySelector('mujoco > worldbody');
      const degScale = (doc.querySelector('compiler')?.getAttribute('angle') || 'radian') === 'degree' ? Math.PI / 180 : 1;
      const walk = (el, parentObj, skipSubtree) => {
        for (const ch of el.children) {
          const tag = ch.tagName;
          if (tag === 'body') {
            const bname = ch.getAttribute('name') || '';
            const isArm = V3_ARM_BODIES.has(bname);
            if (isArm && bname !== 'end_link') continue;
            const bg = new T.Group();
            bg.name = 'simbody:' + bname;
            const p = _v3ParseNums(ch.getAttribute('pos'), [0, 0, 0]);
            bg.position.set(p[0] || 0, p[1] || 0, p[2] || 0);
            const qt = _v3ParseNums(ch.getAttribute('quat'), null);
            const eu = _v3ParseNums(ch.getAttribute('euler'), null);
            if (qt && qt.length === 4) bg.quaternion.set(qt[1], qt[2], qt[3], qt[0]);
            else if (eu && eu.length === 3) bg.rotation.set(eu[0] * degScale, eu[1] * degScale, eu[2] * degScale, 'XYZ');
            if (ch.querySelector(':scope > freejoint') || ch.querySelector(':scope > joint[type="free"]')) {
              freeMap.set(bname, bg);
            }
            if (!isArm) parentObj.add(bg);
            walk(ch, isArm ? parentObj : bg, isArm);
          } else if (tag === 'geom' && !skipSubtree) {
            const gname = ch.getAttribute('name') || '';
            if (V3_SKIP_END_GEOMS.has(gname)) continue;
            const t = _v3TypeNum(ch.getAttribute('type'));
            if (!t) continue;
            const sz = _v3ParseNums(ch.getAttribute('size'), [0.02, 0.02, 0.02]);
            const geo = _v3MakeMjGeo(T, t, sz[0] || 0.02, sz[1] ?? sz[0] ?? 0.02, sz[2] ?? sz[1] ?? sz[0] ?? 0.02);
            if (!geo) continue;
            const rgba = _v3ParseNums(ch.getAttribute('rgba'), [0.6, 0.6, 0.6, 1]);
            const mat = new T.MeshStandardMaterial({
              color: new T.Color().setRGB(rgba[0], rgba[1], rgba[2], T.SRGBColorSpace),
              metalness: 0.15, roughness: 0.65, transparent: rgba[3] < 0.99, opacity: rgba[3],
            });
            const mesh = new T.Mesh(geo, mat);
            const gp = _v3ParseNums(ch.getAttribute('pos'), [0, 0, 0]);
            mesh.position.set(gp[0] || 0, gp[1] || 0, gp[2] || 0);
            const geu = _v3ParseNums(ch.getAttribute('euler'), null);
            if (geu && geu.length === 3) mesh.rotation.set(geu[0] * degScale, geu[1] * degScale, geu[2] * degScale, 'XYZ');
            parentObj.add(mesh);
          }
        }
      };
      if (wb) walk(wb, grp, false);
    }
    V3.root.add(grp);
    V3.simSceneKey = key;
    V3.dirty = true;
  }
  const objs = simState?.objects;
  if (objs && V3.simFreeMap) {
    for (const [name, info] of Object.entries(objs)) {
      const grp = V3.simFreeMap.get(name);
      if (!grp || !Array.isArray(info.pos)) continue;
      const p = info.pos;
      if (Math.abs(grp.position.x - p[0]) + Math.abs(grp.position.y - p[1]) + Math.abs(grp.position.z - p[2]) > 1e-4) {
        grp.position.set(p[0], p[1], p[2]);
        V3.dirty = true;
      }
      if (Array.isArray(info.quat) && info.quat.length === 4) {
        const [qw, qx, qy, qz] = info.quat;
        grp.quaternion.set(qx, qy, qz, qw);
      }
    }
  }
}
function v3Resize() {
  const el = $('view3d'), w = el.clientWidth, h = el.clientHeight;
  if (!V3.r || !w || !h) return;
  V3.r.setSize(w, h, false);
  V3.cam.aspect = w / h; V3.cam.updateProjectionMatrix();
  if (!V3.framed) v3Frame();
  V3.dirty = true;
}
// on every open: aim at the robot's bounding box in the current pose and pull back until its corners
// fill about 75 % of the narrower side of the view (fixed viewing direction; orbit / zoom stay free)
function v3Frame() {
  const T = V3.T, el = $('view3d');
  if (!T || !V3.solid || !el.clientWidth || !el.clientHeight) return;
  const o = S.obs;
  if (o && Array.isArray(o.positions)) { V3.solid.set(o.positions); V3.key = ''; }
  V3.scene.updateMatrixWorld(true);
  const box = new T.Box3().setFromObject(V3.solid.obj);
  if (box.isEmpty()) return;
  const c = box.getCenter(new T.Vector3()), dir = new T.Vector3(0.62, 0.42, 0.66).normalize(), v = new T.Vector3();
  const corners = [];
  for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) corners.push(new T.Vector3(x, y, z));
  let d = box.getSize(v).length();
  for (let k = 0; k < 3; k++) { // the on-screen extent shrinks about linearly with the distance
    V3.cam.position.copy(dir).multiplyScalar(d).add(c); V3.cam.lookAt(c); V3.cam.updateMatrixWorld(true);
    let m = 0;
    for (const p of corners) { v.copy(p).project(V3.cam); m = Math.max(m, Math.abs(v.x), Math.abs(v.y)); }
    if (!(m > 0)) break;
    d = clamp(d * m / 0.75, V3.ctl.minDistance, V3.ctl.maxDistance);
  }
  V3.cam.position.copy(dir).multiplyScalar(d).add(c);
  V3.ctl.target.copy(c); V3.ctl.update();
  V3.framed = true; V3.dirty = true;
}
function v3Tick(ts) {
  V3.raf = V3.open ? requestAnimationFrame(v3Tick) : 0;
  if (!V3.open || !V3.solid || ts - V3.last < 33) return;
  const o = S.obs, live = isLive(), wasm = window.WASM_SIM;
  let st = '';
  if (wasm && wasm.ready && Array.isArray(wasm.positions)) {
    const meas = wasm.positions;
    const cmd = Array.isArray(wasm.targetPositions) ? wasm.targetPositions : meas;
    const key = meas.map((v) => Math.round(v * 2000)).join() + '|' + cmd.map((v) => Math.round(v * 2000)).join();
    if (key !== V3.key) {
      V3.key = key;
      V3.solid.set(meas); V3.ghost.set(cmd);
      V3.ghost.obj.visible = cmd.some((v, i) => Math.abs(v - (meas[i] ?? v)) > (i === GRIP ? 0.05 : 0.01));
      V3.dirty = true;
    }
    v3SyncWasmGeoms();
    st = wasm.policyEnabled && wasm.policyName ? t('v3.wasm_pol', { p: wasm.policyName }) : t('v3.wasm');
  } else {
    if (o && Array.isArray(o.positions)) {
      const meas = o.positions, cmd = Array.isArray(o.commanded_positions) ? o.commanded_positions : meas;
      const pv = CWZ.previewQ || MZ.previewQ; // calibration wizard: the next planned pose; motor zero tool: the zero pose
      const key = meas.map((v) => Math.round(v * 2000)).join() + '|' + cmd.map((v) => Math.round(v * 2000)).join() + (pv ? '|' + pv.join() : '');
      if (key !== V3.key) {
        V3.key = key;
        V3.solid.set(meas); V3.ghost.set(pv || cmd);
        V3.ghost.obj.visible = !!pv || cmd.some((v, i) => Math.abs(v - (meas[i] ?? v)) > (i === GRIP ? 0.05 : 0.005));
        V3.dirty = true;
      }
    }
    st = V3.mode === 'stick' ? t('v3.stick') : !live ? t('v3.offline') : (V3.simModeActive ? t('v3.sim_scene') : '');
  }
  if ($('v3-status').textContent !== st) setText($('v3-status'), st);
  if (!V3.dirty) return;
  V3.dirty = false; V3.last = ts; V3.renders++;
  V3.r.render(V3.scene, V3.cam);
}
// view switch (radiogroup): click, arrow keys / Home / End move the selection; Escape = Stop, Space = E-stop
$('vs-video').addEventListener('click', () => { if (V3.open) v3Toggle(false); });
$('vs-3d').addEventListener('click', () => { if (!V3.open) v3Toggle(true); });
$('view-switch').addEventListener('keydown', (e) => {
  const open = { ArrowLeft: false, ArrowUp: false, Home: false, ArrowRight: true, ArrowDown: true, End: true }[e.key];
  if (open === undefined) return;
  e.preventDefault(); e.stopPropagation();
  if (open !== V3.open) v3Toggle(open);
  $(open ? 'vs-3d' : 'vs-video').focus({ preventScroll: true });
});
i18nHooks.push(() => { V3.key = ''; });

