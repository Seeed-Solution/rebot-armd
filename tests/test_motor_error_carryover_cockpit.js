// Run with node; verifies the cockpit's carried-over motor error hints, operator-only gating,
// and clearFault toast handling against the real src/api/index.html source.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');

const html = fs.readFileSync(path.join(__dirname, '../src/api/index.html'), 'utf8');

function source(name) {
  const plain = html.indexOf('function ' + name + '(');
  assert.notEqual(plain, -1, 'missing function ' + name);
  return html.slice(plain, html.indexOf('\n}', plain) + 2);
}

function i18nSource() {
  const start = html.indexOf('const I18N = {');
  assert.notEqual(start, -1, 'missing I18N');
  return html.slice(start, html.indexOf('\n};', start) + 3);
}

const context = vm.createContext({ console });
vm.runInContext(`
${i18nSource()}
let LANG = 'en', I18N_VARIANT = '', I18N_VIEW = false;
${source('t')}
let S = {
  state: 'Idle',
  fault: null,
  faultCode: null,
  obs: {
    safety_state: 'Idle',
    fault: null,
    is_torqued: false,
    enable_state: 'Unknown',
    motor_errors: [{ code: 0xD, name: 'CAN Communication Loss' }, null, null, null, null, null, null],
  },
  auth: { enforced: true, role: 'program' },
  epoch: 1,
};
const isLive = () => true;
const obsFresh = () => true;
const leaseOther = () => null;
const isStopped = () => S.state === 'EStop' || S.state === 'Fault';
const isOperational = () => S.state === 'Armed' || S.state === 'Moving';
const colHold = () => false;
const faultText = () => null;
const leaseHolderDetail = () => '';
const showTorqueOffInline = () => {};
let toasts = [], calls = [];
const toast = (msg, kind, code) => { toasts.push({ msg, kind, code }); };
const api = async (path, body, opts) => { calls.push({ path, body, opts }); return { is_torqued: false }; };
${source('reportedMotorErrors')}
${source('hasCarriedOverMotorErrors')}
${source('motorErrorSummary')}
${source('lockReason')}
${source('clearNeedsOperator')}
${source('opReasonKey')}
${source('formatTorqueOffWouldDrop')}
${source('clearFault')}
`, context);

(async () => {
  const run = (code) => vm.runInContext(code, context);

  // 1. Carried-over 0xD while Idle/disarmed is detected and summarized.
  assert.equal(run('hasCarriedOverMotorErrors()'), true);
  assert.equal(run('motorErrorSummary()'), 'J1 0xD (CAN Communication Loss)');
  assert.equal(run('clearNeedsOperator()'), true, 'carried-over motor error must be operator-only');
  assert.equal(run("opReasonKey('/api/v1/safety/clear_fault')"), 'op.why.motor_err');

  // 2. English fault panel + lock hint contain cause, operator-only action, and fix / power-cycle text.
  const enFm = run("t('fm.motor_err', { d: motorErrorSummary() })");
  const enFmOp = run("t('fm.motor_err_op', { d: motorErrorSummary() })");
  const enLock = run('lockReason()');
  assert.match(enFm, /J1 0xD \(CAN Communication Loss\)/);
  assert.match(enFm, /operator-only/i);
  assert.match(enFm, /power-cycle the motor/i);
  assert.match(enFmOp, /Operator access required/i);
  assert.match(enFmOp, /power-cycle the motor/i);
  assert.match(enLock, /J1 0xD \(CAN Communication Loss\)/);
  assert.match(enLock, /operator-only/i);
  assert.match(enLock, /power-cycle the motor/i);

  // 3. Chinese fault panel + lock hint contain cause, operator-only action, and fix / power-cycle text.
  run("LANG = 'zh'");
  const zhFm = run("t('fm.motor_err', { d: motorErrorSummary() })");
  const zhFmOp = run("t('fm.motor_err_op', { d: motorErrorSummary() })");
  const zhLock = run('lockReason()');
  assert.match(zhFm, /J1 0xD \(CAN Communication Loss\)/);
  assert.match(zhFm, /操作员/);
  assert.match(zhFm, /断电重启/);
  assert.match(zhFmOp, /操作员权限/);
  assert.match(zhFmOp, /断电重启/);
  assert.match(zhLock, /操作员/);
  assert.match(zhLock, /断电重启/);

  // 4. clearFault calls /api/v1/safety/clear_fault with undefined body and toasts motor_err_cleared.
  run("LANG = 'en'");
  await run('clearFault({ directLease: true })');
  assert.equal(run('calls[0].path'), '/api/v1/safety/clear_fault');
  assert.equal(run('calls[0].body'), undefined);
  assert.match(run('toasts[0].msg'), /stays disabled|stay disabled/i);

  // 5. Once motor_errors are all null, carried-over flag and operator requirement clear.
  run('S.obs.motor_errors = [null, null, null, null, null, null, null]');
  assert.equal(run('hasCarriedOverMotorErrors()'), false);
  assert.equal(run('clearNeedsOperator()'), false);

  console.log('PASS: cockpit carried-over motor error hints, operator-only gate, and clearFault verified');
})().catch((e) => {
  console.error(e);
  process.exit(1);
});
