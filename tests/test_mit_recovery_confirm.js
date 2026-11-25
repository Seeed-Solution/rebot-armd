// Run with node; executes the cockpit's real clearFault handling (MIT expiry recovery).
// There is NO second-tap override when TORQUE_OFF_WOULD_DROP is returned: the cockpit shows
// the same inline TORQUE_OFF_WOULD_DROP message as stopped torque-off.
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
assert.equal(html.indexOf('MIT_RECOVERY_CONFIRM_REQUIRED'), -1, 'cockpit must not arm a second-tap override for MIT_RECOVERY_CONFIRM_REQUIRED');
const context = vm.createContext({ console });
vm.runInContext(`
const t = (k, vars) => vars ? k + ':' + JSON.stringify(vars) : k;
let calls = [], toasts = [], inlineMsgs = [];
const toast = (msg, kind, code) => { toasts.push({ msg, kind, code }); return { isConnected: true, remove() {} }; };
const showTorqueOffInline = (msg) => { inlineMsgs.push(msg); };
let S = { obs: { fault: { code: 'BUS_LOSS', message: 'MIT_LOST: x' } }, epoch: 7 };
let reply = 'would_drop';
const api = async (path, body) => {
  calls.push({ path, body });
  if (reply === 'would_drop') {
    const e = new Error('Torque stays on');
    e.code = 'TORQUE_OFF_WOULD_DROP';
    e.reply = { reason: 'outside_rest_pose', joint: 'joint_2', position: 0.35, rest: 0.0 };
    throw e;
  }
  return { is_torqued: false };
};
${source('formatTorqueOffWouldDrop')}
${source('clearFault')}
`, context);
(async () => {
  const run = (code) => vm.runInContext(code, context);
  // 1. Raised MIT_LOST -> server refuses with TORQUE_OFF_WOULD_DROP -> shown inline, no second-tap override.
  await run('clearFault()');
  assert.deepEqual(JSON.parse(run('JSON.stringify(calls[0].body)')), { mit_recovery: 'torque_off_all' });
  assert.equal(run('toasts.length'), 0, 'TORQUE_OFF_WOULD_DROP must show inline rather than toast');
  assert.match(run('inlineMsgs[0]'), /en\.torque_off\.would_drop.*joint_2/);

  // 2. Repeated tap still hits server without any client-side override flag and keeps showing inline refusal.
  await run('clearFault()');
  assert.equal(run('inlineMsgs.length'), 2);

  // 3. Once at rest (or all arm motors already disabled/expired), clear succeeds and clears inline text.
  run("reply = 'ok'");
  await run('clearFault()');
  assert.equal(run('inlineMsgs[inlineMsgs.length - 1]'), '');
  assert.equal(run('toasts[0].msg'), 'mit.recovery.done');

  // 4. Non-MIT fault sends undefined body.
  run("calls = []; toasts = []; S = { obs: { fault: { code: 'ESTOP', message: 'E-stop' } }, epoch: 8 };");
  await run('clearFault()');
  assert.equal(run('calls[0].body'), undefined);
  assert.equal(run('toasts[0].msg'), 'toast.reset_ok');
  console.log('PASS: cockpit clearFault has no MIT override and renders TORQUE_OFF_WOULD_DROP inline');
})().catch((e) => { console.error(e); process.exit(1); });
