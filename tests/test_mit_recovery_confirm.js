// Run with node; executes the cockpit's real clearFault confirmation handling (MIT expiry recovery).
// The second tap carries {mit_recovery: "torque_off_all"} only while the warning toast is visible
// and only for the same fault instance; any state change disarms it.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../src/api/index.html'), 'utf8');
function source(name) {
  const plain = html.indexOf('function ' + name + '(');
  assert.notEqual(plain, -1, name);
  return html.slice(plain, html.indexOf('\n}', plain) + 2);
}
function stmt(prefix) {
  const i = html.indexOf(prefix);
  assert.notEqual(i, -1, prefix);
  return html.slice(i, html.indexOf('\n', i));
}
const context = vm.createContext({ console });
vm.runInContext(`
const t = (k) => k;
let calls = [], toasts = [];
// toast elements: connected until removed (expiry) or replaced
const toast = (msg, kind, code) => { const el = { isConnected: true, remove() { this.isConnected = false; } }; toasts.push({ msg, kind, code, el }); return el; };
let S = { obs: { fault: { code: 'BUS_LOSS', message: 'MIT_LOST: x' } }, epoch: 7 };
let reply = 'confirm';
const api = async (path, body) => { calls.push(body); if (reply === 'confirm') { const e = new Error('w'); e.code = 'MIT_RECOVERY_CONFIRM_REQUIRED'; throw e; } return { is_torqued: false }; };
${stmt('let mitRecovery')}
${source('mitFaultKey')}
${source('mitRecoveryObserve')}
${source('clearFault')}
`, context);
(async () => {
  const run = (code) => vm.runInContext(code, context);
  await run('clearFault()');
  assert.equal(run('calls[0]'), undefined, 'first tap has no confirmation');
  assert.equal(run('toasts[0].code'), 'MIT_RECOVERY_CONFIRM_REQUIRED');
  run("reply = 'ok'");
  await run('clearFault()');
  assert.deepEqual(JSON.parse(run('JSON.stringify(calls[1])')), { mit_recovery: 'torque_off_all' }, 'second tap while the warning is visible confirms');

  // the warning disappeared (toast expired): no confirmation
  run("calls = []; toasts = []; reply = 'confirm'");
  await run('clearFault()');
  run('toasts[0].el.remove()');
  run("reply = 'ok'");
  await run('clearFault()');
  assert.equal(run('calls[1]'), undefined, 'confirmation outlived the visible warning');

  // another fault instance (epoch changed): the observation disarms it
  run("calls = []; toasts = []; reply = 'confirm'");
  await run('clearFault()');
  run('S.epoch = 8; mitRecoveryObserve()');
  assert.equal(run('toasts[0].el.isConnected'), false, 'warning removed on the state change');
  run("reply = 'ok'");
  await run('clearFault()');
  assert.equal(run('calls[1]'), undefined, 'confirmation carried over to another fault');
  // the fault changes while the first tap's request is in flight: the late reply must not arm
  run("calls = []; toasts = []; reply = 'confirm'; S.epoch = 9");
  await run(`(async () => {
    const pending = clearFault();       // request for the fault at epoch 9
    S.epoch = 10;                         // observation switches before the reply arrives
    await pending;
  })()`);
  assert.equal(run('toasts.length'), 0, 'late reply for an old fault armed the confirmation');
  run("reply = 'ok'");
  await run('clearFault()');
  assert.equal(run('calls[1]'), undefined, 'second tap confirmed a fault the first tap was not about');
  console.log('PASS: MIT recovery confirmation is bound to the visible warning and the fault instance');
})().catch((e) => { console.error(e); process.exit(1); });
