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
  return html.slice(start, html.indexOf('\n};', start) + 3);
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
})().catch(error => { console.error(error); process.exitCode = 1; });
