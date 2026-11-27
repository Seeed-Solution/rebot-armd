// Run with node; verifies that the cockpit hand-guide UI (#btn-gc and calibration wizard
// cwGcSupported) enables hand-guiding on OWNER_EXECUTOR when obs.gravity_comp.supported=true
// and schema.capabilities.gravity_comp=true, while still failing closed on synthetic
// EXECUTOR_CAPABILITY_UNSUPPORTED and keeping exit (enable:false) outside operator gating.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');

const html = require('../src/api/cockpit/assembler/assemble.cjs').assemble();

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
  const start = html.indexOf('const I18N = {};');
  assert.notEqual(start, -1, 'missing I18N');
  const end = html.indexOf('\nlet LANG', start);
  assert.notEqual(end, -1, 'missing LANG after the dictionaries');
  return html.slice(start, end + 1);
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
  'gc-ws-warn': makeEl(),
  'btn-gc-exit': makeEl(),
  'gc-exit-why': makeEl(),
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
const isCancelledErr = () => false;
let apiCalls = [], toastCalls = [];
const api = (p, b) => { apiCalls.push([p, b]); return Promise.resolve({}); };
const toast = (...args) => toastCalls.push(args);
${lineMatching('const gcOn = ')}
${lineMatching('const gcExecutorUnsupported = ')}
${lineMatching('const cwGcSupported = ')}
${fnSource('gcBlockText')}
${fnSource('formatWsDist')}
${fnSource('wsEdgeAndDir')}
${fnSource('formatGcWorkspaceBanner')}
${fnSource('formatGcExitBlocked')}
${fnSource('gcWorkspaceBlocked')}
${fnSource('gcExit')}
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
assert.equal(els['gc-ws-warn'].hidden, true);
assert.equal(els['gc-exit-why'].hidden, true);
assert.equal(els['btn-gc-exit'].classList.contains('blocked'), false);
assert.equal(bodyEl.classList.contains('gc'), true);

// 2b. Dragged outside workspace shows banner and inline exit block reason (zh and en), then clears back inside.
run("LANG = 'zh'; S.obs.gravity_comp.workspace_outside = true; S.obs.gravity_comp.workspace_edge = 'box_max_x'; S.obs.gravity_comp.workspace_depth_m = 0.045; renderGc();");
assert.equal(els['gc-banner'].classList.contains('ws-outside'), true);
assert.equal(els['gc-ws-warn'].hidden, false);
assert.equal(els['gc-ws-warn'].textContent, '已拖出工作范围（前方边界 4.5 cm），请拖回范围内后再退出拖动');
assert.equal(els['btn-gc-exit'].classList.contains('blocked'), true);
assert.equal(els['btn-gc-exit'].getAttribute('aria-disabled'), 'true');
assert.equal(els['gc-exit-why'].hidden, false);
assert.match(els['gc-exit-why'].textContent, /前方边界 4\.5 cm/);
assert.match(els['gc-sub'].textContent, /前方边界 4\.5 cm/);
run("LANG = 'en'; renderGc();");
assert.equal(els['gc-ws-warn'].textContent, 'Dragged outside workspace (front edge 4.5 cm) — drag back inside before exiting');
assert.match(els['gc-exit-why'].textContent, /4\.5 cm beyond front edge/);
// Activating gcExit() while outside toasts the banner without firing an RPC; once back inside it fires exit.
run("apiCalls = []; toastCalls = []; gcExit();");
assert.equal(run('apiCalls.length'), 0, 'blocked gcExit must not send RPC while outside');
assert.equal(run('toastCalls[0][2]'), 'GC_EXIT_OUTSIDE_WORKSPACE');
run("S.obs.gravity_comp.workspace_outside = false; S.obs.gravity_comp.workspace_edge = null; S.obs.gravity_comp.workspace_depth_m = null; renderGc(); gcExit();");
assert.equal(run('apiCalls.length'), 1, 'gcExit once back inside sends exit RPC');
assert.equal(run('JSON.stringify(apiCalls[0])'), JSON.stringify(['/api/v1/mode/gravity_comp', { enable: false }]));
assert.equal(els['gc-banner'].classList.contains('ws-outside'), false);
assert.equal(els['gc-ws-warn'].hidden, true);
assert.equal(els['gc-exit-why'].hidden, true);
assert.equal(els['btn-gc-exit'].classList.contains('blocked'), false);
assert.equal(els['btn-gc-exit'].getAttribute('aria-disabled'), 'false');

// 3. Generic fallback remains intact if a driver/profile still reports EXECUTOR_CAPABILITY_UNSUPPORTED.
run("S.obs.control_mode = 'position'; S.obs.gravity_comp = { supported: false, active: false, phase: null, ramp: 0, blocker: { code: 'EXECUTOR_CAPABILITY_UNSUPPORTED', message: 'unsupported' } }; S.caps.gravity_comp = false; renderGc();");
assert.equal(run('cwGcSupported()'), false);
assert.equal(run('gcExecutorUnsupported()'), true);
assert.equal(els['btn-gc'].hidden, false, 'unsupported blocker remains visible with blocked state for diagnostic toast');
assert.equal(els['btn-gc'].classList.contains('blocked'), true);
assert.equal(els['btn-gc'].getAttribute('aria-disabled'), 'true');

console.log('PASS: cockpit hand-guide enables on executor when gravity_comp.supported=true and fails closed on unsupported');
