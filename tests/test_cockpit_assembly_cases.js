// Shared cockpit assembly cases (tests/fixtures/cockpit_assembly/cases.json) through the Node assembler
// (src/api/cockpit/assembler/assemble.cjs); tests/test_cockpit_assembly.rs runs the same cases through
// assemble.rs (build.rs) and runs this file. Run: node tests/test_cockpit_assembly_cases.js
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { assemble } = require('../src/api/cockpit/assembler/assemble.cjs');

const CASES = JSON.parse(fs.readFileSync(path.join(__dirname, 'fixtures', 'cockpit_assembly', 'cases.json'), 'utf8')).cases;

function writeCase(files) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'cockpit-assembly-'));
  for (const [rel, body] of Object.entries(files)) {
    const file = path.join(dir, rel);
    fs.mkdirSync(path.dirname(file), { recursive: true });
    fs.writeFileSync(file, typeof body === 'string' ? Buffer.from(body, 'utf8') : Buffer.from(body.hex, 'hex'));
  }
  return dir;
}

let n = 0;
for (const c of CASES) {
  const dir = writeCase(c.files);
  try {
    let page = null, error = null;
    try { page = assemble(dir); } catch (e) { error = e.message; }
    if ('page' in c) {
      assert.equal(error, null, `${c.name}: unexpected error`);
      assert(Buffer.from(page, 'utf8').equals(Buffer.from(c.page, 'utf8')), `${c.name}: page ${JSON.stringify(page)}, want ${JSON.stringify(c.page)}`);
    } else {
      assert(error !== null, `${c.name}: assembled ${JSON.stringify(page)}, want an error containing ${JSON.stringify(c.error)}`);
      assert(error.includes(c.error), `${c.name}: error ${JSON.stringify(error)}, want ${JSON.stringify(c.error)}`);
    }
    n++;
  } finally {
    fs.rmSync(dir, { recursive: true, force: true });
  }
}
console.log(`PASS ${n} shared cockpit assembly cases (Node assembler)`);
