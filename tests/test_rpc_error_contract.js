// Run with node; executes the actual UI error decoder and recovery handlers.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../src/api/index.html'), 'utf8');
function source(name) {
  const plain = html.indexOf('function ' + name + '(');
  assert.notEqual(plain, -1, name);
  const start = html.slice(plain - 6, plain) === 'async ' ? plain - 6 : plain;
  return html.slice(start, html.indexOf('\n}', plain) + 2);
}
// The handlers translate their messages with t(); load the real dictionary (Chinese UI).
function i18nSource() {
  const start = html.indexOf('const I18N = {');
  assert.notEqual(start, -1, 'I18N');
  const end = html.indexOf('\n// ----', start);
  const assignEnd = html.indexOf('\n});', html.indexOf('Object.assign(I18N.en', start));
  return html.slice(start, assignEnd !== -1 ? assignEnd + 4 : html.indexOf('\n};', start) + 3);
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

  // Workspace box-edge rejection formatting in zh and en (all 6 edges + exact 1.0 cm wording).
  const wsErr = (lang, payload) => vm.runInContext(`
    LANG = ${JSON.stringify(lang)};
    rpcError(${JSON.stringify(payload)}).message`, context);
  assert.equal(
    wsErr('zh', {
      code: 'WORKSPACE_VIOLATION',
      constraint_id: 'box_max_x',
      geometry_id: 'end_point:5',
      reason: 'gripper is 1.0 cm beyond the workspace front edge (box_max_x); move back or press Home',
    }),
    '夹爪超出工作空间前方边界 1.0 cm，请向后移动或按 Home',
  );
  assert.equal(
    wsErr('en', {
      code: 'WORKSPACE_VIOLATION',
      constraint_id: 'box_max_x',
      geometry_id: 'end_point:5',
      reason: 'gripper is 1.0 cm beyond the workspace front edge (box_max_x); move back or press Home',
    }),
    'gripper is 1.0 cm beyond the workspace front edge (box_max_x); move back or press Home',
  );
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
      reason: 'gripper is 1.0 cm beyond the workspace front edge (box_max_x); move back or press Home',
    })`, context);
  assert.equal(graspWs, '夹爪超出工作空间前方边界 1.0 cm，请向后移动或按 Home');
  console.log('PASS: workspace box-edge rejections translate cleanly in zh/en for all 6 edges (cm/mm + grasp)');

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
  console.log('PASS: drag-teach outside-workspace banner, inline exit block, and GC_EXIT_OUTSIDE_WORKSPACE strings verified in zh/en');
})().catch(error => { console.error(error); process.exitCode = 1; });
