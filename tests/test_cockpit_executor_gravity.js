// Run with node; verifies that the cockpit hand-guide UI (#btn-gc and calibration wizard
// cwGcSupported) enables hand-guiding on OWNER_EXECUTOR when obs.gravity_comp.supported=true
// and schema.capabilities.gravity_comp=true, while still failing closed on synthetic
// EXECUTOR_CAPABILITY_UNSUPPORTED and keeping exit (enable:false) outside operator gating.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');

const html = fs.readFileSync(path.join(__dirname, '../src/api/index.html'), 'utf8');

function fnSource(name) {
  const plain = html.indexOf('function ' + name + '(');
  assert.notEqual(plain, -1, 'missing function ' + name);
  return html.slice(plain, html.indexOf('\n}', plain) + 2);
}

function lineMatching(prefix) {
  const idx = html.indexOf(prefix);
  assert.notEqual(idx, -1, 'missing line starting with ' + prefix);
  return html.slice(idx, html.indexOf('\n', idx));
}

function i18nSource() {
  const start = html.indexOf('const I18N = {');
  assert.notEqual(start, -1, 'missing I18N');
  return html.slice(start, html.indexOf('\n};', start) + 3);
}

function makeEl() {
  const classes = new Set();
  const attrs = new Map();
  return {
    hidden: false,
    title: '',
    textContent: '',
    style: {},
    classList: {
      toggle(k, force) {
        if (force) classes.add(k);
        else classes.delete(k);
      },
      contains(k) { return classes.has(k); },
    },
    setAttribute(k, v) { attrs.set(k, String(v)); },
    getAttribute(k) { return attrs.has(k) ? attrs.get(k) : null; },
  };
}

const els = {
  'btn-gc': makeEl(),
  'gc-sub': makeEl(),
  'gc-banner': makeEl(),
  'gc-phase': makeEl(),
  'gc-ramp-bar': makeEl(),
  'gc-ramp': makeEl(),
  'gc-hint': makeEl(),
  'btn-gc-rec': makeEl(),
  'gc-chip': makeEl(),
};
const bodyEl = makeEl();

const context = vm.createContext({
  console,
  document: { body: bodyEl },
  $: (id) => els[id],
  setText: (el, txt) => { el.textContent = String(txt); },
  fmtTime: () => '00:00',
});

vm.runInContext(`
${i18nSource()}
let LANG = 'en', I18N_VARIANT = '', I18N_VIEW = false;
${fnSource('t')}
let S = {
  leaseGeneration: 1,
  caps: { gravity_comp: true },
  rec: { active: false, startedAt: 0 },
  obs: {
    effective_owner: 'executor',
    control_mode: 'position',
    gravity_comp: { supported: true, active: false, phase: null, ramp: 0, blocker: null },
  },
};
const isLive = () => true;
const leaseOther = () => null;
${lineMatching('const gcOn = ')}
${lineMatching('const gcExecutorUnsupported = ')}
${lineMatching('const cwGcSupported = ')}
${fnSource('gcBlockText')}
${fnSource('renderGc')}
`, context);

const run = (code) => vm.runInContext(code, context);

// 1. Executor with Damiao / non-blocking driver: supported=true, blocker=null, caps.gravity_comp=true.
run('renderGc()');
assert.equal(run('cwGcSupported()'), true, 'calibration wizard hand-guide must be supported on executor');
assert.equal(run('gcExecutorUnsupported()'), false, 'must not report EXECUTOR_CAPABILITY_UNSUPPORTED');
assert.equal(run('gcBlockText()'), null, 'hand-guide must have no blocker on executor');
assert.equal(els['btn-gc'].hidden, false, '#btn-gc must be visible on executor');
assert.equal(els['btn-gc'].classList.contains('blocked'), false, '#btn-gc must not have blocked class');
assert.equal(els['btn-gc'].getAttribute('aria-disabled'), 'false', '#btn-gc must be enabled');
assert.equal(els['btn-gc'].getAttribute('aria-checked'), 'false');

// 2. Active gravity_comp on executor flips #btn-gc on and shows the banner.
run("S.obs.control_mode = 'gravity_comp'; S.obs.gravity_comp.active = true; S.obs.gravity_comp.phase = 'active'; S.obs.gravity_comp.ramp = 1; renderGc();");
assert.equal(els['btn-gc'].classList.contains('on'), true);
assert.equal(els['btn-gc'].getAttribute('aria-checked'), 'true');
assert.equal(els['btn-gc'].getAttribute('aria-disabled'), 'false');
assert.equal(els['gc-banner'].hidden, false);
assert.equal(bodyEl.classList.contains('gc'), true);

// 3. Generic fallback remains intact if a driver/profile still reports EXECUTOR_CAPABILITY_UNSUPPORTED.
run("S.obs.control_mode = 'position'; S.obs.gravity_comp = { supported: false, active: false, phase: null, ramp: 0, blocker: { code: 'EXECUTOR_CAPABILITY_UNSUPPORTED', message: 'unsupported' } }; S.caps.gravity_comp = false; renderGc();");
assert.equal(run('cwGcSupported()'), false);
assert.equal(run('gcExecutorUnsupported()'), true);
assert.equal(els['btn-gc'].hidden, false, 'unsupported blocker remains visible with blocked state for diagnostic toast');
assert.equal(els['btn-gc'].classList.contains('blocked'), true);
assert.equal(els['btn-gc'].getAttribute('aria-disabled'), 'true');

console.log('PASS: cockpit hand-guide enables on executor when gravity_comp.supported=true and fails closed on unsupported');
