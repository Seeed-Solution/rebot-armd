/* =================================================================
   Browser WASM Physics Sandbox + Unified ONNX Runtime (onnxruntime-web)
   ================================================================= */
const WASM_SIM = window.WASM_SIM = (() => {
  const dec = new TextDecoder();
  const MODEL_OFFSETS_RAD = [0, 0, 0, 0, 0, 0]; // B601-DM official convention (URDF = joint angle)
  const JAW_M_PER_RAD = 0.0425 / 5.7;
  const KPOS_DM = [25, 25, 25, 25, 25, 25, 25];
  const KV_DM = [8, 8, 8, 1.5, 1.5, 1.5, 0.5];
  const TORQUE_DM = [27, 27, 27, 7, 7, 7, 7];
  const NO_LOAD_DM = [52, 52, 52, 200, 200, 200, 200].map((rpm) => rpm * 2 * Math.PI / 60);

  let ortLoader = null;
  let mujocoPromise = null;
  let imgCanvas = null;
  let imgCtx = null;

  async function ensureMujoco() {
    if (mujocoPromise) return mujocoPromise;
    mujocoPromise = (async () => {
      const mod = await import('/assets/wasm/mujoco_wasm.js');
      const mj = await mod.default();
      try { mj.FS.mkdir('/working'); } catch (_) {}
      try { mj.FS.mount(mj.MEMFS, { root: '.' }, '/working'); } catch (_) {}
      return mj;
    })();
    return mujocoPromise;
  }

  async function ensureOrt() {
    if (window.ort) return window.ort;
    if (!ortLoader) {
      ortLoader = new Promise((resolve, reject) => {
        const sc = document.createElement('script');
        sc.src = '/assets/wasm/ort.wasm.min.js';
        sc.onload = () => resolve(window.ort);
        sc.onerror = () => { ortLoader = null; reject(new Error('Failed to load ort.wasm.min.js')); };
        document.head.appendChild(sc);
      });
    }
    const ort = await ortLoader;
    if (ort && ort.env && ort.env.wasm) {
      ort.env.wasm.wasmPaths = '/assets/wasm/';
      ort.env.wasm.numThreads = 1;
    }
    return ort;
  }

  function writeMemfsFile(mj, relPath, bytes) {
    const stack = relPath.startsWith('/') ? [] : ['working'];
    for (const seg of relPath.split('/')) {
      if (!seg || seg === '.') continue;
      if (seg === '..') { if (stack.length > 0) stack.pop(); }
      else stack.push(seg);
    }
    if (stack.length === 0) return;
    let cur = '';
    for (let i = 0; i < stack.length - 1; i++) {
      cur += '/' + stack[i];
      try { mj.FS.mkdir(cur); } catch (_) {}
    }
    mj.FS.writeFile('/' + stack.join('/'), bytes);
  }

  function mulberry32(seed) {
    let a = (Number(seed) || 0) >>> 0;
    return () => {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function normVal(v, mean, std, i) {
    const m = Array.isArray(mean) && mean.length > i ? Number(mean[i]) || 0 : 0;
    const s = Array.isArray(std) && std.length > i ? Math.max(Math.abs(Number(std[i]) || 1), 1e-6) : 1;
    return (v - m) / s;
  }

  const st = {
    active: false,
    ready: false,
    starting: false,
    mujoco: null,
    model: null,
    data: null,
    bundle: null,
    bundleKey: '',
    positions: [0, 0, 0, 0, 0, 0, 0], // the folded rest pose
    targetPositions: null,
    overrideTargets: null,
    policyTarget: null,
    policySession: null,
    policyName: '',
    policyEnabled: false,
    policyBusy: false,
    policySteps: 0,
    policyEpoch: 0,
    simGeneration: 0,
    lastAction: null,
    lastPolicyAt: 0,
    io: {},
    attachedObject: null,
    suctionWelds: {},
    toolContactGeoms: new Set(),
    objects: [],
    sceneJoints: [],
    latched: {},
    savedRgba: {},
    idx: null,
    tcpSiteId: -1,
    attachBid: -1,
    seq: 0,
    resetSeq: -1,
    ioSeq: -1,
    seed: 0,
    rafId: 0,
    lastTs: 0,
    accDt: 0,
    lastSyncAt: 0,
    lastTelAt: 0,
    syncBusy: false,
    telBusy: false,

    mjName(adrArr, idx) {
      const m = this.model;
      if (!m || !adrArr || idx < 0 || idx >= adrArr.length) return '';
      const start = adrArr[idx];
      let end = start;
      while (end < m.names.length && m.names[end] !== 0) end++;
      return dec.decode(m.names.subarray(start, end));
    },

    findId(objTypeNum, name) {
      if (!this.mujoco || !this.model || !name) return -1;
      try { return this.mujoco.mj_name2id(this.model, objTypeNum, name); } catch (_) { return -1; }
    },

    buildIndexer() {
      const mj = this.mujoco, m = this.model;
      const obj = mj.mjtObj;
      const tm = this.bundle?.tool_manifest;
      const isSoArm = this.findId(obj.mjOBJ_JOINT.value, 'shoulder_pan') >= 0;
      const joints = [];
      let offsets = [0, 0, 0, 0, 0, 0];
      let toolScale = 1.0;
      let armDof = 5;
      let coupledName = null;

      if (isSoArm) {
        for (const n of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']) {
          if (this.findId(obj.mjOBJ_JOINT.value, n) >= 0) joints.push(n);
        }
      } else {
        armDof = 6;
        offsets = MODEL_OFFSETS_RAD.slice();
        for (let i = 1; i <= 6; i++) joints.push('joint' + i);
        const hasCustom = tm && tm.id && tm.id !== 'default';
        if (hasCustom) {
          if (tm.driven_joint && this.findId(obj.mjOBJ_JOINT.value, tm.driven_joint) >= 0) joints.push(tm.driven_joint);
          toolScale = Number(tm.motor_to_joint_scale) || 1.0;
          coupledName = tm.coupled_joint || null;
        } else if (this.findId(obj.mjOBJ_JOINT.value, 'finger_left') >= 0) {
          joints.push('finger_left');
          toolScale = -JAW_M_PER_RAD;
          coupledName = 'finger_right';
        }
      }

      const jids = joints.map((n) => this.findId(obj.mjOBJ_JOINT.value, n));
      const qposAdr = jids.map((j) => m.jnt_qposadr[j]);
      const dofAdr = jids.map((j) => m.jnt_dofadr[j]);
      const rightJid = coupledName ? this.findId(obj.mjOBJ_JOINT.value, coupledName) : -1;
      const acts = [];
      for (let i = 0; i < m.nu; i++) {
        const trnJ = m.actuator_trnid[i * 2];
        const jIdx = jids.indexOf(trnJ);
        const isPos = Math.abs(m.actuator_biasprm[i * 10 + 1]) > 1e-6;
        acts.push({
          a: i,
          jIdx: jIdx >= 0 ? jIdx : i,
          dof: m.jnt_dofadr[trnJ],
          gear: m.actuator_gear[i * 6] || 1.0,
          isPos,
          limLo: m.actuator_ctrllimited?.[i] ? m.actuator_ctrlrange[i * 2] : -Infinity,
          limHi: m.actuator_ctrllimited?.[i] ? m.actuator_ctrlrange[i * 2 + 1] : Infinity,
        });
      }
      this.idx = {
        robot: isSoArm ? 'so-arm101' : 'b601dm',
        armDof,
        dof: joints.length,
        joints,
        jids,
        qposAdr,
        dofAdr,
        offsets,
        toolScale,
        rightQpos: rightJid >= 0 ? m.jnt_qposadr[rightJid] : -1,
        acts,
      };
      this.tcpSiteId = this.findId(obj.mjOBJ_SITE.value, 'tcp');
      const attachBody = tm?.attach_body || 'end_link';
      this.attachBid = this.findId(obj.mjOBJ_BODY.value, attachBody);
    },

    motorPositions() {
      const d = this.data, ix = this.idx;
      if (!d || !ix) return [];
      const out = new Array(ix.dof);
      for (let i = 0; i < ix.dof; i++) {
        const v = d.qpos[ix.qposAdr[i]];
        out[i] = ix.robot === 'b601dm' ? (i < 6 ? v - ix.offsets[i] : v / ix.toolScale) : v;
      }
      return out;
    },

    indexSceneAndWelds() {
      const mj = this.mujoco, m = this.model;
      const objType = mj.mjtObj;
      const sm = this.bundle?.scene_manifest;
      const tm = this.bundle?.tool_manifest;

      this.objects = [];
      for (const item of (sm?.objects || [])) {
        const bid = this.findId(objType.mjOBJ_BODY.value, item.body || item.name);
        if (bid < 0) continue;
        const jid = m.body_jntadr[bid];
        if (jid < 0) continue;
        const qadr = m.jnt_qposadr[jid];
        const dadr = m.jnt_dofadr[jid];
        const geoms = [];
        for (let g = 0; g < m.ngeom; g++) if (m.geom_bodyid[g] === bid) geoms.push(g);
        const defQuat = [];
        for (let k = 0; k < 4; k++) defQuat.push(m.body_quat[bid * 4 + k]);
        const hasQuat = defQuat.some((v) => Math.abs(v) > 1e-6);
        this.objects.push({
          name: String(item.name),
          body: String(item.body || item.name),
          bid,
          qadr,
          dadr,
          geoms,
          defaultPos: Array.isArray(item.default_pos) ? item.default_pos.slice(0, 3) : [0.25, 0, 0.02],
          defaultQuat: hasQuat ? defQuat : [1, 0, 0, 0],
          jitterXy: Number(item.jitter_xy ?? 0.02),
        });
      }

      this.suctionWelds = {};
      for (let eq = 0; eq < m.neq; eq++) {
        const ename = this.mjName(m.name_eqadr, eq);
        if (ename.startsWith('suction_weld_')) {
          const b2 = m.eq_obj2id[eq];
          const b2Name = this.mjName(m.name_bodyadr, b2);
          if (b2Name) this.suctionWelds[b2Name] = eq;
        }
      }

      this.toolContactGeoms = new Set();
      const drivenPrefix = tm?.driven_joint ? String(tm.driven_joint).split('_')[0] : '';
      const fGeoms = new Set(tm?.finger_geoms || ['left_front', 'right_front']);
      for (let g = 0; g < m.ngeom; g++) {
        const gname = this.mjName(m.name_geomadr, g);
        const bname = this.mjName(m.name_bodyadr, m.geom_bodyid[g]);
        if (fGeoms.has(gname) || bname === 'end_link' || (drivenPrefix && bname.includes(drivenPrefix))) {
          this.toolContactGeoms.add(g);
        }
      }

      this.sceneJoints = [];
      const robotSet = new Set(this.idx.joints);
      if (this.bundle?.tool_manifest?.coupled_joint) robotSet.add(this.bundle.tool_manifest.coupled_joint);
      robotSet.add('finger_right');
      const objBodies = new Set(this.objects.map((o) => o.bid));
      for (let j = 0; j < m.njnt; j++) {
        const jname = this.mjName(m.name_jntadr, j);
        const jbid = m.jnt_bodyid[j];
        if (!robotSet.has(jname) && !objBodies.has(jbid) && m.jnt_type[j] !== mj.mjtJoint.mjJNT_FREE.value) {
          this.sceneJoints.push({ qadr: m.jnt_qposadr[j], q0: m.qpos0[m.jnt_qposadr[j]] });
        }
      }

      this.savedRgba = {};
      for (const stage of (sm?.task_stages || [])) {
        if (!stage.indicator_geom) continue;
        const gid = this.findId(objType.mjOBJ_GEOM.value, stage.indicator_geom);
        if (gid >= 0) this.savedRgba[gid] = Array.from(m.geom_rgba.subarray(gid * 4, gid * 4 + 4));
      }
      this.ioDefaults = {};
      for (const src of [...(sm?.io || []), ...(tm?.io || [])]) {
        if ((src.direction || 'output') === 'output' || 'default' in src) {
          this.ioDefaults[String(src.name)] = 'default' in src ? src.default : false;
        }
      }
    },

    releaseSuction() {
      if (!this.data) return;
      for (const eq of Object.values(this.suctionWelds)) this.data.eq_active[eq] = 0;
      this.attachedObject = null;
      if ('vacuum_engaged' in this.io || this.bundle?.tool_manifest?.kind === 'suction') {
        this.io.vacuum_engaged = false;
      }
    },

    tryEngageSuction() {
      const m = this.model, d = this.data, tm = this.bundle?.tool_manifest;
      if (!m || !d || this.attachedObject || !Object.keys(this.suctionWelds).length) return;
      if (this.tcpSiteId < 0 || this.attachBid < 0) return;
      const tcp = d.site_xpos.subarray(this.tcpSiteId * 3, this.tcpSiteId * 3 + 3);
      let bestObj = null, bestDist = Infinity;
      for (const obj of this.objects) {
        if (!(obj.body in this.suctionWelds)) continue;
        for (const gid of obj.geoms) {
          const gp = d.geom_xpos.subarray(gid * 3, gid * 3 + 3);
          const dist = Math.hypot(tcp[0] - gp[0], tcp[1] - gp[1], tcp[2] - gp[2]) - m.geom_rbound[gid];
          if (dist < bestDist) { bestDist = dist; bestObj = obj; }
        }
      }
      const rangeM = Number(tm?.suction_range_m ?? 0.030);
      if (!bestObj || bestDist > rangeM) return;

      const eqId = this.suctionWelds[bestObj.body];
      const b1 = this.attachBid, b2 = bestObj.bid;
      const r1 = d.xmat.subarray(b1 * 9, b1 * 9 + 9);
      const p1 = d.xpos.subarray(b1 * 3, b1 * 3 + 3);
      const p2 = d.xpos.subarray(b2 * 3, b2 * 3 + 3);
      const dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
      const px = r1[0] * dx + r1[3] * dy + r1[6] * dz;
      const py = r1[1] * dx + r1[4] * dy + r1[7] * dz;
      const pz = r1[2] * dx + r1[5] * dy + r1[8] * dz;

      const q1 = d.xquat.subarray(b1 * 4, b1 * 4 + 4);
      const q2 = d.xquat.subarray(b2 * 4, b2 * 4 + 4);
      const w1 = q1[0], x1 = -q1[1], y1 = -q1[2], z1 = -q1[3];
      const w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
      const rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
      const rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
      const ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
      const rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

      const eo = eqId * 11;
      m.eq_data[eo] = 0; m.eq_data[eo + 1] = 0; m.eq_data[eo + 2] = 0;
      m.eq_data[eo + 3] = px; m.eq_data[eo + 4] = py; m.eq_data[eo + 5] = pz;
      m.eq_data[eo + 6] = rw; m.eq_data[eo + 7] = rx; m.eq_data[eo + 8] = ry; m.eq_data[eo + 9] = rz;
      m.eq_data[eo + 10] = 1.0;
      d.eq_active[eqId] = 1;
      this.attachedObject = bestObj.name;
      this.io.vacuum_engaged = true;
    },

    stepIo() {
      const m = this.model, d = this.data;
      if (!m || !d) return;
      if (this.bundle?.tool_manifest?.kind === 'suction') {
        if (this.io.vacuum) this.tryEngageSuction();
        else if (this.attachedObject || Object.values(this.suctionWelds).some((eq) => d.eq_active[eq])) {
          this.releaseSuction();
        }
      }
      if ('brew_button_pressed' in this.io) {
        const jid = this.findId(this.mujoco.mjtObj.mjOBJ_JOINT.value, 'brew_btn_joint');
        if (jid >= 0) {
          const pressed = d.qpos[m.jnt_qposadr[jid]] <= -0.005;
          this.io.brew_button_pressed = pressed;
          if (pressed) this.io.brew_led = true;
        }
      }
    },

    graspedSet() {
      const d = this.data;
      const held = new Set();
      if (this.attachedObject) held.add(this.attachedObject);
      const geomObj = new Map();
      for (const obj of this.objects) for (const gid of obj.geoms) geomObj.set(gid, obj.name);
      for (let i = 0; i < d.ncon; i++) {
        const c = d.contact.get(i);
        const g1 = c.geom1, g2 = c.geom2;
        if (typeof c.delete === 'function') c.delete();
        if (geomObj.has(g1) && this.toolContactGeoms.has(g2)) held.add(geomObj.get(g1));
        else if (geomObj.has(g2) && this.toolContactGeoms.has(g1)) held.add(geomObj.get(g2));
      }
      if (this.tcpSiteId >= 0) {
        const tcp = d.site_xpos.subarray(this.tcpSiteId * 3, this.tcpSiteId * 3 + 3);
        for (const obj of this.objects) {
          const p = d.qpos.subarray(obj.qadr, obj.qadr + 3);
          if (p[2] >= obj.defaultPos[2] + 0.025 && Math.hypot(p[0] - tcp[0], p[1] - tcp[1], p[2] - tcp[2]) <= 0.08) {
            held.add(obj.name);
          }
        }
      }
      return held;
    },

    objectState() {
      const held = this.graspedSet();
      const out = {};
      for (const obj of this.objects) {
        out[obj.name] = {
          pos: Array.from(this.data.qpos.subarray(obj.qadr, obj.qadr + 3)).map((v) => Math.round(v * 1e5) / 1e5),
          quat: Array.from(this.data.qpos.subarray(obj.qadr + 3, obj.qadr + 7)).map((v) => Math.round(v * 1e5) / 1e5),
          grasped: held.has(obj.name),
        };
      }
      return out;
    },

    evaluateTask(objSt) {
      const sm = this.bundle?.scene_manifest;
      if (!sm || !Array.isArray(sm.task_stages) || !sm.task_stages.length) return null;
      const m = this.model, d = this.data;
      const stagesOut = {};
      for (const stg of sm.task_stages) {
        const sid = String(stg.id);
        if (this.latched[sid]) { stagesOut[sid] = true; continue; }
        if (Array.isArray(stg.requires) && stg.requires.some((r) => !stagesOut[String(r)])) {
          stagesOut[sid] = false; continue;
        }
        let passed = false;
        if (stg.type === 'object_grasped') {
          const info = objSt[String(stg.object)];
          if (info) passed = Boolean(info.grasped) && (stg.min_z == null || info.pos[2] >= Number(stg.min_z));
        } else if (stg.type === 'object_in_zone') {
          const oname = String(stg.object);
          const info = objSt[oname];
          const z = (sm.zones || {})[String(stg.zone)];
          if (info && z && Array.isArray(z.center) && Array.isArray(z.half_extents)) {
            const inZone = [0, 1, 2].every((k) => Math.abs(info.pos[k] - z.center[k]) <= z.half_extents[k]);
            const isAttached = Boolean(info.grasped) || this.attachedObject === oname || Boolean(this.io?.vacuum_engaged && this.attachedObject === oname);
            const requireReleased = stg.require_released !== false;
            passed = inZone && (!requireReleased || !isAttached);
          }
        } else if (stg.type === 'joint_threshold') {
          const jid = this.findId(this.mujoco.mjtObj.mjOBJ_JOINT.value, String(stg.joint));
          if (jid >= 0) {
            const val = d.qpos[m.jnt_qposadr[jid]], thr = Number(stg.threshold), op = String(stg.op || '<=');
            passed = op === '<' ? val < thr : op === '>=' ? val >= thr : op === '>' ? val > thr : val <= thr;
          }
        } else if (stg.type === 'io_state') {
          passed = this.io[String(stg.channel)] === stg.value;
        }
        if (passed && stg.latch !== false) this.latched[sid] = true;
        stagesOut[sid] = passed;
        if (passed && stg.indicator_geom && Array.isArray(stg.indicator_rgba)) {
          const gid = this.findId(this.mujoco.mjtObj.mjOBJ_GEOM.value, String(stg.indicator_geom));
          if (gid >= 0) {
            for (let k = 0; k < 4; k++) m.geom_rgba[gid * 4 + k] = Number(stg.indicator_rgba[k] ?? 1);
          }
        }
      }
      const total = sm.task_stages.length;
      const doneCount = Object.values(stagesOut).filter(Boolean).length;
      const current = sm.task_stages.find((s) => !stagesOut[String(s.id)]);
      return {
        scene: sm.id,
        completed: total > 0 && doneCount === total,
        success: total > 0 && doneCount === total,
        score: total ? Math.round((doneCount / total) * 1e4) / 1e4 : 0,
        current_stage: current ? String(current.id) : null,
        stages: stagesOut,
      };
    },

    reset(seed) {
      const m = this.model, d = this.data;
      if (!m || !d) return;
      this.simGeneration++;
      this.policyEpoch = this.simGeneration;
      this.seed = seed != null ? seed : this.seed;
      const rng = seed != null ? mulberry32(seed) : null;
      for (const obj of this.objects) {
        const px = obj.defaultPos[0] + (rng && obj.jitterXy > 0 ? (rng() * 2 - 1) * obj.jitterXy : 0);
        const py = obj.defaultPos[1] + (rng && obj.jitterXy > 0 ? (rng() * 2 - 1) * obj.jitterXy : 0);
        d.qpos[obj.qadr] = px; d.qpos[obj.qadr + 1] = py; d.qpos[obj.qadr + 2] = obj.defaultPos[2];
        for (let k = 0; k < 4; k++) d.qpos[obj.qadr + 3 + k] = obj.defaultQuat[k];
        for (let k = 0; k < 6; k++) d.qvel[obj.dadr + k] = 0;
      }
      for (const sj of this.sceneJoints) d.qpos[sj.qadr] = sj.q0;
      this.io = { ...this.ioDefaults };
      this.releaseSuction();
      this.attachedObject = null;
      this.latched = {};
      for (const [gidStr, rgba] of Object.entries(this.savedRgba)) {
        const gid = Number(gidStr);
        for (let k = 0; k < 4; k++) m.geom_rgba[gid * 4 + k] = rgba[k];
      }
      this.mujoco.mj_forward(m, d);
      this.positions = this.motorPositions();
      this.overrideTargets = null;
      this.policyTarget = null;
      this.policySteps = 0;
      V3.key = ''; V3.dirty = true;
    },

    applyMotors(targets) {
      const m = this.model, d = this.data, ix = this.idx;
      if (!m || !d || !ix) return;
      const q = this.positions = this.motorPositions();
      const isSo = ix.robot === 'so-arm101';
      for (const dof of ix.dofAdr) d.qfrc_applied[dof] = 0;
      for (const act of ix.acts) {
        const i = act.jIdx;
        const qt = i < targets.length && Number.isFinite(targets[i]) ? targets[i] : q[i];
        if (act.isPos) {
          let cmd = qt * ix.toolScale;
          if (act.limHi > act.limLo) cmd = clamp(cmd, act.limLo, act.limHi);
          d.ctrl[act.a] = cmd;
        } else {
          const vmax = isSo ? Math.min(2.5, 50 * 2 * Math.PI / 60) : (NO_LOAD_DM[i] ?? 20.0);
          const kv = isSo ? 2.0 : (KV_DM[i] ?? 1.5);
          const kpos = isSo ? 25.0 : (KPOS_DM[i] ?? 25.0);
          const tlim = isSo ? 2.94 : (TORQUE_DM[i] ?? 7.0);
          m.actuator_gainprm[act.a * 10] = kv;
          m.actuator_biasprm[act.a * 10 + 2] = -kv;
          d.ctrl[act.a] = clamp(kpos * (qt - q[i]), -vmax, vmax);
          let extra = i < ix.armDof ? clamp(d.qfrc_bias[act.dof], -tlim, tlim) : 0;
          m.actuator_forcerange[act.a * 2] = -tlim - extra;
          m.actuator_forcerange[act.a * 2 + 1] = tlim - extra;
          d.qfrc_applied[act.dof] = act.gear * extra;
        }
      }
    },

    async stepPolicy() {
      const pm = this.bundle?.policy_manifest;
      if (!this.policySession || !this.policyEnabled || !pm || this.policyBusy) return;
      this.policyBusy = true;
      const currentGen = this.simGeneration;
      try {
        const ort = await ensureOrt();
        const feeds = {};
        const stateSpec = pm.inputs?.state;
        const norm = pm.normalization || {};
        if (stateSpec) {
          const cur = this.positions;
          const dim = Number(stateSpec.shape?.[1] || cur.length);
          const arr = new Float32Array(dim);
          for (let i = 0; i < dim; i++) arr[i] = normVal(Number(cur[i] ?? 0), norm.state_mean, norm.state_std, i);
          feeds[stateSpec.name] = new ort.Tensor('float32', arr, [1, dim]);
        }
        const imgSpec = pm.inputs?.image;
        if (imgSpec && $('v3-canvas')) {
          const [, c, h, w] = imgSpec.shape.map(Number);
          if (!imgCanvas || imgCanvas.width !== w || imgCanvas.height !== h) {
            imgCanvas = document.createElement('canvas');
            imgCanvas.width = w; imgCanvas.height = h;
            imgCtx = imgCanvas.getContext('2d', { willReadFrequently: true });
          }
          if (V3.r && V3.scene && V3.cam) V3.r.render(V3.scene, V3.cam);
          imgCtx.drawImage($('v3-canvas'), 0, 0, w, h);
          const pixels = imgCtx.getImageData(0, 0, w, h).data;
          const scale = Number(imgSpec.scale ?? 1 / 255);
          const imMean = imgSpec.mean || [0, 0, 0], imStd = imgSpec.std || [1, 1, 1];
          const buf = new Float32Array(c * h * w);
          const hw = h * w;
          for (let p = 0; p < hw; p++) {
            for (let ch = 0; ch < Math.min(c, 3); ch++) {
              const v = pixels[p * 4 + ch] * scale;
              buf[ch * hw + p] = (v - Number(imMean[ch] ?? 0)) / Math.max(Number(imStd[ch] ?? 1), 1e-6);
            }
          }
          feeds[imgSpec.name] = new ort.Tensor('float32', buf, [1, c, h, w]);
        }
        const out = await this.policySession.run(feeds);
        if (!this.policyEnabled || this.simGeneration !== currentGen) return;
        const actTensor = out[pm.outputs.action.name];
        const rawAct = Array.from(actTensor.data);
        const outDim = Number(pm.action_dim || this.idx.dof);
        const target = new Array(outDim);
        for (let i = 0; i < outDim; i++) {
          const am = Array.isArray(norm.action_mean) && norm.action_mean.length > i ? Number(norm.action_mean[i]) || 0 : 0;
          const as = Array.isArray(norm.action_std) && norm.action_std.length > i ? Number(norm.action_std[i]) || 1 : 1;
          const v = rawAct[i] * as + am;
          const cur = this.positions[i] ?? 0;
          const lead = i === GRIP ? CFG.gripLead : CFG.maxLead;
          target[i] = clamp(v, cur - lead, cur + lead);
        }
        if (!this.policyEnabled || this.simGeneration !== currentGen) return;
        this.lastAction = target.slice();
        this.policyTarget = target;
        this.policySteps++;
        const ioOut = pm.outputs?.io;
        if (ioOut?.name && out[ioOut.name]) {
          const vals = out[ioOut.name].data;
          (ioOut.channels || []).forEach((ch, i) => {
            if (i >= vals.length) return;
            const name = typeof ch === 'string' ? ch : ch?.name;
            if (!name) return;
            const isBool = (typeof ch === 'object' && ch.kind === 'bool') || name === 'vacuum';
            this.io[name] = isBool ? vals[i] > 0.5 : Number(vals[i]);
          });
          this.stepIo();
        }
      } catch (e) {
        if (this.simGeneration === currentGen) {
          console.warn('WASM_SIM policy step error', e);
        }
      } finally {
        this.policyBusy = false;
      }
    },

    tick(ts) {
      if (!this.active || !this.ready) return;
      this.rafId = requestAnimationFrame((t2) => this.tick(t2));
      if (!this.lastTs) this.lastTs = ts;
      const dtReal = clamp((ts - this.lastTs) / 1000, 0, 0.05);
      this.lastTs = ts;
      this.accDt += dtReal;
      const h = this.model.opt?.timestep || 0.002;
      let targets = this.positions;
      if (Array.isArray(this.overrideTargets)) targets = this.overrideTargets;
      else if (this.policyEnabled && Array.isArray(this.policyTarget)) targets = this.policyTarget;
      else if (isLive() && S.obs && Array.isArray(S.obs.positions)) {
        targets = Array.isArray(S.obs.commanded_positions) ? S.obs.commanded_positions : S.obs.positions;
      }
      this.targetPositions = targets.slice();

      let steps = 0;
      while (this.accDt >= h && steps < 25) {
        this.applyMotors(targets);
        this.stepIo();
        this.mujoco.mj_step(this.model, this.data);
        this.accDt -= h;
        steps++;
      }
      this.positions = this.motorPositions();

      const fps = Number(this.bundle?.policy_manifest?.fps || 30);
      if (this.policyEnabled && this.policySession && ts - this.lastPolicyAt >= 1000 / Math.max(1, fps)) {
        this.lastPolicyAt = ts;
        this.stepPolicy();
      }
      if (ts - this.lastSyncAt >= 180) {
        this.lastSyncAt = ts;
        this.pollBackendState();
      }
      if (ts - this.lastTelAt >= 150) {
        this.lastTelAt = ts;
        this.pushTelemetry();
      }
    },

    getState() {
      if (!this.ready || !this.data) return null;
      const objSt = this.objectState();
      const task = this.evaluateTask(objSt);
      const tcp = this.tcpSiteId >= 0
        ? Array.from(this.data.site_xpos.subarray(this.tcpSiteId * 3, this.tcpSiteId * 3 + 3)).map((v) => Math.round(v * 1e5) / 1e5)
        : [0, 0, 0];
      return {
        seq: ++this.seq,
        sim_time_s: Math.round((this.data.time || 0) * 1e4) / 1e4,
        positions: this.positions.map((v) => Math.round(v * 1e5) / 1e5),
        tcp,
        objects: objSt,
        task,
        io: { ...this.io },
        policy: this.policyName ? { name: this.policyName, enabled: this.policyEnabled, steps: this.policySteps } : null,
      };
    },

    async pushTelemetry() {
      if (this.telBusy || this.telBlocked || !this.ready) return;
      this.telBusy = true;
      try {
        const stPayload = this.getState();
        if (!stPayload) return;
        const r = await fetch('/api/v1/sandbox/telemetry', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(stPayload),
        });
        if (!r.ok) {
          if (r.status === 401 || r.status === 403) {
            this.telBlocked = true;
          } else if (!this.telErrLogged) {
            this.telErrLogged = true;
            console.warn('WASM_SIM telemetry post failed:', r.status);
          }
          return;
        }
        this.telErrLogged = false;
      } catch (_) {} finally {
        this.telBusy = false;
      }
    },

    async pollBackendState() {
      if (this.syncBusy || !this.active) return;
      this.syncBusy = true;
      try {
        const r = await fetch('/api/v1/sandbox/state');
        if (!r.ok) return;
        const remote = await r.json();
        if (remote.mode === 'none') { this.stop(); return; }
        const rKey = `${remote.mode}:${remote.active_scene}:${remote.active_tool}:${remote.active_policy || ''}`;
        if (remote.mode === 'wasm' && this.bundleKey && rKey !== this.bundleKey) {
          await this.loadBundle();
          return;
        }
        if (remote.reset_seq !== this.resetSeq && remote.reset_seq > this.resetSeq) {
          this.resetSeq = remote.reset_seq;
          this.reset(remote.seed);
        }
        if (remote.io_seq !== this.ioSeq) {
          this.ioSeq = remote.io_seq;
          if (remote.desired_io && typeof remote.desired_io === 'object') {
            Object.assign(this.io, remote.desired_io);
            this.stepIo();
          }
        }
      } catch (_) {} finally {
        this.syncBusy = false;
      }
    },

    setPolicyEnabled(on) {
      const prev = this.policyEnabled;
      this.policyEnabled = Boolean(on);
      if (prev && !this.policyEnabled) {
        this.simGeneration++;
        this.policyEpoch = this.simGeneration;
        this.policyTarget = null;
      }
    },
    setTargetPositions(q) { this.overrideTargets = Array.isArray(q) ? q.slice() : null; },
    setIo(updates) {
      if (updates && typeof updates === 'object') Object.assign(this.io, updates);
      this.stepIo();
    },

    moveTcp(x, y, z, pitch = Math.PI / 2) {
      if (!this.model || !this.idx) return null;
      const curM = this.motorPositions();
      const prevRaw = toModel(curM);
      const prev = (Math.abs(prevRaw[1]) < 0.05 && Math.abs(prevRaw[2]) < 0.05) ? [0, -0.8, -1.5, 1.0] : prevRaw;
      const res = IK.solve({ x, y, z, pitch }, {
        prev,
        inLimits: (q) => {
          for (let i = 0; i < 4; i++) {
            const jid = this.idx.jids[i];
            if (jid >= 0 && this.model.jnt_limited?.[jid]) {
              const lo = this.model.jnt_range[jid * 2], hi = this.model.jnt_range[jid * 2 + 1];
              if (q[i] < lo + 1e-3 || q[i] > hi - 1e-3) return false;
            }
          }
          return true;
        },
      });
      if (!res.ok) return null;
      const full = toMotor(res.q);
      const q7 = [full[0], full[1], full[2], full[3], 0, curM[5] ?? restJoints()[5], curM[6] ?? 0];
      this.setTargetPositions(q7);
      return q7;
    },

    async loadBundle() {
      const r = await fetch('/api/v1/sandbox/bundle');
      if (!r.ok) throw new Error('sandbox bundle failed: ' + r.status);
      const bundle = await r.json();
      const mj = this.mujoco = await ensureMujoco();
      for (const a of (bundle.scene_manifest?.assets || [])) {
        if (a?.path) {
          const ar = await fetch(`/api/v1/sandbox/scenes/${bundle.active_scene}/files/${a.path}`);
          if (ar.ok) writeMemfsFile(mj, a.path, new Uint8Array(await ar.arrayBuffer()));
        }
      }
      for (const a of (bundle.tool_manifest?.assets || [])) {
        if (a?.path) {
          const ar = await fetch(`/api/v1/sandbox/tools/${bundle.active_tool}/files/${a.path}`);
          if (ar.ok) writeMemfsFile(mj, a.path, new Uint8Array(await ar.arrayBuffer()));
        }
      }
      for (const a of (bundle.base_assets || [])) {
        const dst = a?.memfs_path || a?.path;
        if (a?.url && dst) {
          const ar = await fetch(a.url);
          if (ar.ok) writeMemfsFile(mj, dst, new Uint8Array(await ar.arrayBuffer()));
        }
      }
      writeMemfsFile(mj, 'scene.xml', bundle.composed_xml);
      this.ready = false;
      const oldData = this.data;
      const oldModel = this.model;
      this.data = null;
      this.model = null;
      if (oldData) { try { oldData.delete(); } catch (_) {} }
      if (oldModel) { try { oldModel.delete(); } catch (_) {} }
      try {
        this.model = mj.MjModel.loadFromXML('/working/scene.xml');
        this.data = new mj.MjData(this.model);
        mj.mj_forward(this.model, this.data);

        this.simGeneration++;
        this.policyEpoch = this.simGeneration;
        this.bundle = bundle;
        this.bundleKey = `${bundle.mode}:${bundle.active_scene}:${bundle.active_tool}:${bundle.active_policy || ''}`;
        this.resetSeq = bundle.reset_seq ?? 0;
        this.ioSeq = bundle.io_seq ?? 0;
        this.seed = bundle.seed ?? 0;
        this.buildIndexer();
        this.indexSceneAndWelds();
        this.releaseSuction();
        this.attachedObject = null;
        this.latched = {};
        this.io = { ...this.ioDefaults, ...(bundle.desired_io || {}) };
        this.positions = this.motorPositions();
        this.overrideTargets = null;
        this.policyTarget = null;
        this.policySession = null;
        this.policyName = '';
        this.policyEnabled = false;
        this.policySteps = 0;

        if (bundle.policy_model_url && bundle.policy_manifest) {
          const ort = await ensureOrt();
          this.policySession = await ort.InferenceSession.create(bundle.policy_model_url, { executionProviders: ['wasm'] });
          this.policyName = bundle.policy_manifest.name || bundle.active_policy || 'onnx';
          this.policyEnabled = Boolean(bundle.auto_start_policy);
        }
        v3ClearGroup(V3.sandboxGroup);
        V3.sandboxGroup = null;
        V3.key = ''; V3.dirty = true;
        this.ready = true;
        this.stepIo();
        this.pushTelemetry();
      } catch (err) {
        this.ready = false;
        if (this.data) { try { this.data.delete(); } catch (_) {} this.data = null; }
        if (this.model) { try { this.model.delete(); } catch (_) {} this.model = null; }
        throw err;
      }
    },

    async start() {
      if (this.starting) return;
      this.starting = true;
      this.active = true;
      try {
        if (!V3.open) v3Toggle(true);
        await (V3.init ||= v3Init());
        await this.loadBundle();
        this.lastTs = 0;
        if (!this.rafId) this.rafId = requestAnimationFrame((ts) => this.tick(ts));
      } catch (e) {
        console.warn('WASM_SIM start error', e);
      } finally {
        this.starting = false;
      }
    },

    stop() {
      this.active = false;
      this.ready = false;
      if (this.rafId) { cancelAnimationFrame(this.rafId); this.rafId = 0; }
      v3ClearGroup(V3.sandboxGroup); V3.sandboxGroup = null;
      if (V3.solid?.setHideFingers) V3.solid.setHideFingers(false);
      if (V3.ghost?.setHideFingers) V3.ghost.setHideFingers(false);
    },
  };
  return st;
})();

async function initSandbox() {
  const wantWasm = QS.get('sim') === 'wasm';
  let cat = null;
  try {
    const r = await fetch('/api/v1/sandbox/catalog');
    if (r.ok) cat = await r.json();
  } catch (_) {}
  if (!cat) {
    if (wantWasm) WASM_SIM.start().catch(() => {});
    return;
  }
  // Only on request in this page's URL: the server-side mode is shared by every client,
  // and the WASM view draws a simulated arm while the controls still drive the real one.
  if (wantWasm) {
    WASM_SIM.start().catch(() => {});
    return;
  }
  if (cat.mode === 'sim') {
    V3.simModeActive = true;
    try {
      const br = await fetch('/api/v1/sandbox/bundle');
      const bundle = br.ok ? await br.json() : null;
      if (!V3.open) v3Toggle(true);
      await (V3.init ||= v3Init());
      if (bundle) await v3SyncSimScene(bundle, null);
      setInterval(async () => {
        if (!V3.simModeActive || !V3.open) return;
        try {
          const sr = await fetch('/api/v1/sandbox/state');
          if (!sr.ok) return;
          const s = await sr.json();
          if (s.mode === 'wasm') { V3.simModeActive = false; return; }
          if (s.mode !== 'sim') return;
          if (s.sim_state) await v3SyncSimScene(bundle, s.sim_state);
        } catch (_) {}
      }, 250);
    } catch (_) {}
  }
}

