// Run with node; executes the actual UI error decoder and recovery handlers.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const html = require('../src/api/cockpit/assembler/assemble.cjs').assemble();
function source(name) {
  const plain = html.indexOf('function ' + name + '(');
  assert.notEqual(plain, -1, name);
  const start = html.slice(plain - 6, plain) === 'async ' ? plain - 6 : plain;
  return html.slice(start, html.indexOf('\n}', plain) + 2);
}
// The handlers translate their messages with t(); load the real dictionary (Chinese UI).
function i18nSource() {
  const start = html.indexOf('const I18N = {};');
  assert.notEqual(start, -1, 'I18N');
  const end = html.indexOf('\nlet LANG', start);
  assert.notEqual(end, -1, 'LANG after the dictionaries');
  return html.slice(start, end + 1);
}
const context = vm.createContext({ console });
vm.runInContext(`
${i18nSource()}
let LANG = 'zh', I18N_VARIANT = '', I18N_VIEW = false;
${source('t')}
let unloading = false;
let messages = [], unloadCalls = 0, rejection;
const toast = (...args) => messages.push(args);
let api = async () => { throw rejection; };
let controlledUnload = () => { unloadCalls++; };
${source('formatWorkspaceViolation')}
${source('formatWsDist')}
${source('wsEdgeAndDir')}
${source('formatGcWorkspaceBanner')}
${source('formatGcExitBlocked')}
${source('formatGcExitOutsideError')}
${source('rpcError')}
${source('disableTorque')}
`, context);
(async () => {
  await vm.runInContext(`(async () => {
    rejection = rpcError({ status: 409, code: 'ARM_NOT_AT_RESTING_POSE', error: 'ArmNotAtRestingPose', message: 'Arm is not at resting pose' });
    await disableTorque();
  })()`, context);
  assert.equal(vm.runInContext('unloadCalls', context), 1, '409/not-resting starts controlled unload');
  assert.equal(vm.runInContext('rejection.status', context), 409);
  assert.equal(vm.runInContext('rejection.code', context), 'ARM_NOT_AT_RESTING_POSE');
  assert.equal(vm.runInContext("messages.some(m => /无法连接/.test(m[0]))", context), false);

  await vm.runInContext(`(async () => {
    messages = []; unloadCalls = 0;
    rejection = rpcError({ status: 400, code: 'INVALID_COMMAND', message: 'enable must be boolean' });
    await disableTorque();
  })()`, context);
  assert.equal(vm.runInContext('unloadCalls', context), 0);
  assert.equal(vm.runInContext('messages[0][0]', context), 'enable must be boolean');

  // Use the real controlledUnload handler and a rejected RPC: it must report
  // the business rejection and reset its guard without waiting for telemetry.
  vm.runInContext('controlledUnload = ' + source('controlledUnload') + ';', context);
  await vm.runInContext(`(async () => {
    messages = [];
    rejection = rpcError({ status: 422, code: 'UNSUPPORTED', message: 'Unsupported automatic unload' });
    await controlledUnload();
  })()`, context);
  assert.equal(vm.runInContext('unloading', context), false);
  assert.equal(vm.runInContext('messages.length', context), 1);
  assert.match(vm.runInContext('messages[0][0]', context), /未启动收纳/);
  assert.equal(vm.runInContext('messages[0][2]', context), 'UNSUPPORTED');
  console.log('PASS: typed RPC errors drive unload fallback and business-error UI; no network misclassification');

  // Lease loss: the toast follows armd's reason; a missing reason (older armd) means expiry.
  vm.runInContext(`
    const CFG = { leaseRetryMs: 500 };
    const S = { leaseGeneration: null };
    let resets = 0, retries = 0;
    const resetLease = () => { resets++; S.leaseGeneration = null; };
    const scheduleLeaseRetry = () => { retries++; };
    ${source('leaseLossKey')}
    ${source('onLeaseLost')}
  `, context);
  const lost = (lang, reason, generation = 7) => vm.runInContext(`
    LANG = ${JSON.stringify(lang)}; messages = []; S.leaseGeneration = 7;
    onLeaseLost({}, ${JSON.stringify({ type: 'lease_lost', generation, reason })});
    messages.length ? messages[0][0] : null`, context);
  assert.match(lost('zh', 'preempted'), /其他设备接管/);
  assert.match(lost('zh', 'expired'), /连接中断/);
  assert.doesNotMatch(lost('zh', 'expired'), /其他设备/);
  assert.match(lost('zh', undefined), /连接中断/);
  assert.match(lost('zh', 'released'), /本页面已释放/);
  assert.match(lost('en', 'preempted'), /another device/);
  assert.match(lost('en', 'expired'), /Connection dropped/);
  assert.doesNotMatch(lost('en', 'expired'), /another device/);
  assert.match(lost('en', 'released'), /This page released/);
  assert.equal(lost('zh', 'preempted', 3), null, 'stale generation is ignored');
  assert.equal(vm.runInContext('S.leaseGeneration', context), 7);
  console.log('PASS: lease_lost reason selects preempted / connection-dropped / released wording');

  // Workspace refusal formatting in zh and en: box edges (Home vs Park), sweep_budget, and max_reach.
  const wsErr = (lang, payload) => vm.runInContext(`
    LANG = ${JSON.stringify(lang)};
    rpcError(${JSON.stringify(payload)}).message`, context);
  const wsCases = [
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'box_max_x', geometry_id: 'end_point:5',
       reason: 'gripper (end_point:5) is 1.0 cm beyond the workspace front edge (box_max_x 0.610 m, limit 0.600 m; previous depth 0.000 m, new depth 0.010 m); move back or press Home' },
     '夹爪 (end_point:5) 超出工作空间前方边界 1.0 cm（box_max_x 0.610 m，上限 0.600 m；原超出 0.000 m，新超出 0.010 m），请向后移动或按 Home', null],
    // Case 2: J3 jog from already-outside pose suggests Park (never Home)
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'box_max_x', geometry_id: 'end_point:8',
       reason: 'gripper (end_point:8) is 4.5 cm beyond the workspace front edge (box_max_x 0.645 m, limit 0.600 m; previous depth 0.040 m, new depth 0.045 m; this move would go 5 mm deeper); move back or press Park' },
     '夹爪 (end_point:8) 超出工作空间前方边界 4.5 cm（box_max_x 0.645 m，上限 0.600 m；原超出 0.040 m，新超出 0.045 m，将加深 5 mm），请向后移动或按收纳（Park）', null],
    // Case 1: sweep_budget with no keep-out box (no obstacle/keep-out, suggests Park + jog direction, preserves context in both locales)
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'sweep_budget', geometry_id: 'end_origin',
       reason: 'return path could not be checked (needs more than 4000 FK evaluations); press Park or make a short jog inward toward the base (near max_reach on end_origin)' },
     '回程路径无法完成校验（超出 4000 次 FK 评估），请按收纳（Park）或向基座方向小步点动（max_reach，end_origin）', null],
    // Named keep-out box with apostrophe in name ("operator's bin") parses cleanly without truncation
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'sweep_budget', geometry_id: 'end_origin',
       reason: "path check near keep-out box 'operator's bin' needs more than 4 FK evaluations; take a shorter jog or move the obstacle (end_origin)" },
     '禁入区「operator\'s bin」附近路径无法完成校验（超出 4 次 FK 评估），请小步点动或调整障碍物（end_origin）', null],
    // Unboxed sweep with a capsule id containing "keep-out box 'fake'" must NOT be classified as an obstacle
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'sweep_budget', geometry_id: "capsule:keep-out box 'fake'",
       reason: "return path could not be checked (needs more than 4000 FK evaluations); press Park or make a short jog back (-X) (near box_max_x on capsule:keep-out box 'fake')" },
     "回程路径无法完成校验（超出 4000 次 FK 评估），请按收纳（Park）或向后（-X）小步点动（box_max_x，capsule:keep-out box 'fake'）", null],
    // Invalid workspace check with a box constraint id must pass through raw daemon reason (never suggest Home)
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'box_max_x', geometry_id: 'end_origin',
       reason: 'invalid workspace check for box_max_x (end_origin)' },
     'invalid workspace check for box_max_x (end_origin)', null],
    // Case 3: max_reach deepening from one witness (symmetric fields in zh and en)
    [{ code: 'WORKSPACE_VIOLATION', constraint_id: 'max_reach', geometry_id: 'end_origin',
       reason: 'end_origin reach 0.685 m (limit 0.520 m, new depth 0.165 m over, previous depth 0.164 m; this move would go 1.5 mm deeper)' },
     'end_origin 臂展 0.685 m（上限 0.520 m，新超出 0.165 m，原超出 0.164 m；此移动会再加深 1.5 mm）', null],
  ];
  for (const [payload, expZh, expEn] of wsCases) {
    assert.equal(wsErr('zh', payload), expZh);
    assert.equal(wsErr('en', payload), expEn || payload.reason);
  }
  assert.doesNotMatch(wsErr('zh', wsCases[1][0]), /Home/);
  assert.doesNotMatch(wsErr('zh', wsCases[2][0]), /障碍物|禁入区|Home/);
  assert.doesNotMatch(wsErr('en', wsCases[2][0]), /obstacle|keep-?out|Home/i);
  assert.doesNotMatch(wsErr('zh', wsCases[4][0]), /障碍物|禁入区|Home/);
  assert.doesNotMatch(wsErr('zh', wsCases[5][0]), /Home/);
  for (const [cid, zhEdge, zhDir, enEdge, enDir] of [
    ['box_max_x', '前方边界', '向后移动', 'front edge', 'move back'],
    ['box_min_x', '后方边界', '向前移动', 'back edge', 'move forward'],
    ['box_max_y', '左侧边界', '向右移动', 'left edge', 'move right'],
    ['box_min_y', '右侧边界', '向左移动', 'right edge', 'move left'],
    ['box_max_z', '顶部边界', '向下移动', 'top edge', 'move down'],
    ['box_min_z', '底部边界', '向上移动', 'bottom edge', 'move up'],
  ]) {
    const payload = {
      code: 'WORKSPACE_VIOLATION',
      constraint_id: cid,
      geometry_id: 'end_origin',
      reason: `gripper is 2.3 cm beyond the workspace ${enEdge} (${cid}); ${enDir} or press Home`,
    };
    assert.equal(wsErr('zh', payload), `夹爪超出工作空间${zhEdge} 2.3 cm，请${zhDir}或按 Home`);
    assert.equal(wsErr('en', payload), payload.reason);
  }
  // Sub-millimetre new intrusion preserves mm precision (never "0.0 cm") and graspCodeText uses it
  assert.equal(
    wsErr('zh', {
      code: 'WORKSPACE_VIOLATION',
      constraint_id: 'box_max_x',
      geometry_id: 'end_point:5',
      reason: 'gripper is 0.2 mm beyond the workspace front edge (box_max_x); move back or press Home',
    }),
    '夹爪超出工作空间前方边界 0.2 mm，请向后移动或按 Home',
  );
  vm.runInContext(`
    const graspCodeKey = (code) => (/^IK_/.test(code || '') ? 'IK' : code);
    ${source('graspCodeText')}
  `, context);
  const graspWs = vm.runInContext(`
    LANG = 'zh';
    graspCodeText('WORKSPACE_VIOLATION', 'raw', {
      code: 'WORKSPACE_VIOLATION',
      constraint_id: 'box_max_x',
      geometry_id: 'end_point:5',
      reason: 'gripper (end_point:5) is 1.0 cm beyond the workspace front edge (box_max_x 0.610 m, limit 0.600 m; previous depth 0.000 m, new depth 0.010 m); move back or press Home',
    })`, context);
  assert.equal(graspWs, '夹爪 (end_point:5) 超出工作空间前方边界 1.0 cm（box_max_x 0.610 m，上限 0.600 m；原超出 0.000 m，新超出 0.010 m），请向后移动或按 Home');
  console.log('PASS: workspace refusals (box edges + Park/Home, sweep_budget, max_reach) translate cleanly in zh/en');

  // Drag-teach outside-workspace banner, inline exit block reason, and GC_EXIT_OUTSIDE_WORKSPACE error strings (zh & en).
  const gcBanner = (lang, gc) => vm.runInContext(`LANG = ${JSON.stringify(lang)}; formatGcWorkspaceBanner(${JSON.stringify(gc)})`, context);
  const gcBlocked = (lang, gc) => vm.runInContext(`LANG = ${JSON.stringify(lang)}; formatGcExitBlocked(${JSON.stringify(gc)})`, context);
  assert.equal(
    gcBanner('zh', { workspace_outside: true, workspace_edge: 'box_max_x', workspace_depth_m: 0.045 }),
    '已拖出工作范围（前方边界 4.5 cm），请拖回范围内后再退出拖动',
  );
  assert.equal(
    gcBanner('en', { workspace_outside: true, workspace_edge: 'box_max_x', workspace_depth_m: 0.045 }),
    'Dragged outside workspace (front edge 4.5 cm) — drag back inside before exiting',
  );
  assert.equal(
    gcBanner('zh', { workspace_outside: true, workspace_edge: 'max_reach', workspace_depth_m: 0.031 }),
    '已拖出工作范围（最大臂展 3.1 cm），请拖回范围内后再退出拖动',
  );
  assert.equal(
    gcBanner('en', { workspace_outside: true, workspace_edge: 'floor_z', workspace_depth_m: 0.0005 }),
    'Dragged outside workspace (floor 0.5 mm) — drag back inside before exiting',
  );
  assert.equal(
    gcBanner('zh', { workspace_outside: true, workspace_edge: 'keepout:camera', workspace_depth_m: 0.02 }),
    '已拖出工作范围（禁区「camera」 2.0 cm），请拖回范围内后再退出拖动',
  );
  assert.equal(gcBanner('zh', { workspace_outside: false, workspace_edge: null, workspace_depth_m: null }), '');
  assert.match(gcBlocked('zh', { workspace_outside: true, workspace_edge: 'box_max_x', workspace_depth_m: 0.045 }), /前方边界 4\.5 cm.*向后移动/);
  assert.match(gcBlocked('en', { workspace_outside: true, workspace_edge: 'box_max_x', workspace_depth_m: 0.045 }), /4\.5 cm beyond front edge.*move back/);
  assert.equal(
    wsErr('zh', {
      code: 'GC_EXIT_OUTSIDE_WORKSPACE',
      message: 'gripper is 4.5 cm beyond the workspace front edge (box_max_x); move back inside the workspace before exiting',
    }),
    '已拖出工作范围（前方边界 4.5 cm），请拖回范围内后再退出拖动',
  );
  assert.equal(
    wsErr('en', {
      code: 'GC_EXIT_OUTSIDE_WORKSPACE',
      message: 'gripper is 4.5 cm beyond the workspace front edge (box_max_x); move back inside the workspace before exiting',
    }),
    'gripper is 4.5 cm beyond the workspace front edge (box_max_x); move back inside the workspace before exiting',
  );
  assert.equal(wsErr('zh', { code: 'GC_EXIT_OUTSIDE_WORKSPACE', message: '' }), '已拖出工作范围，请拖回范围内后再退出拖动');
  assert.equal(wsErr('en', { code: 'GC_EXIT_OUTSIDE_WORKSPACE', message: '' }), 'Arm is outside the workspace — drag back inside before exiting hand-guiding');
  // Structured fields handle keep-out names with parentheses and unverified feedback cleanly.
  assert.equal(
    wsErr('zh', {
      code: 'GC_EXIT_OUTSIDE_WORKSPACE',
      workspace_edge: 'keepout:rack (floor_z)',
      workspace_depth_m: 0.02,
      workspace_recovery_dir: 'move_left',
      message: "gripper is 2.0 cm inside keep-out 'rack (floor_z)' (keepout:rack (floor_z)); move left clear of keep-out 'rack (floor_z)' before exiting",
    }),
    '已拖出工作范围（禁区「rack (floor_z)」 2.0 cm），请拖回范围内后再退出拖动',
  );
  assert.equal(
    wsErr('zh', { code: 'GC_EXIT_OUTSIDE_WORKSPACE', workspace_unverified: true, message: 'measured pose workspace clearance could not be verified; wait for fresh feedback or use Stop/E-stop' }),
    '无法确认机械臂是否在工作范围内，请等待电机反馈恢复或按停止/急停',
  );
  assert.match(gcBlocked('zh', { workspace_outside: true, workspace_edge: 'keepout:rack', workspace_depth_m: 0.02, workspace_recovery_dir: 'move_left' }), /禁区「rack」 2\.0 cm.*向左移动/);
  assert.match(gcBlocked('en', { workspace_outside: true, workspace_edge: 'max_reach', workspace_depth_m: 0.031, workspace_recovery_dir: 'move_down' }), /3\.1 cm beyond max reach.*move down/);
  console.log('PASS: drag-teach outside-workspace banner, inline exit block, and GC_EXIT_OUTSIDE_WORKSPACE strings verified in zh/en');
})().catch(error => { console.error(error); process.exitCode = 1; });
