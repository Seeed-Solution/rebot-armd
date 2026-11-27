'use strict';
// Node assembler of the cockpit page, same splice rule as assemble.rs (build.rs): a shell line that is
// exactly `<!-- @include cockpit/<path> -->` is replaced by that fragment's bytes.
// Library: require(...).assemble() -> page string. CLI: node assemble.cjs [out.html] (default stdout).
// tests/test_cockpit_assembly.rs checks that this output equals the page armd serves.
const fs = require('node:fs');
const path = require('node:path');

const API_DIR = path.resolve(__dirname, '..', '..');
const SPLICE = /^<!-- @include (\S+) -->$/;
const REL_OK = /^cockpit(\/[A-Za-z0-9_.-]+)+$/;

function readFragment(apiDir, rel) {
  if (!REL_OK.test(rel) || rel.split('/').some((s) => s === '.' || s === '..')) {
    throw new Error(`invalid fragment path ${rel}`);
  }
  const text = fs.readFileSync(path.join(apiDir, rel), 'utf8');
  if (!text.endsWith('\n')) throw new Error(`fragment ${rel} must be non-empty and end with a newline`);
  if (text.split('\n').some((l) => SPLICE.test(l))) throw new Error(`fragment ${rel} contains a splice line`);
  return text;
}

// Splices the fragments into the shell; returns { page, fragments } (fragments in splice order).
function assembleParts(apiDir = API_DIR) {
  const shell = fs.readFileSync(path.join(apiDir, 'index.html'), 'utf8');
  const lines = shell.split(/(?<=\n)/);
  const fragments = [];
  let page = '';
  for (const line of lines) {
    const m = SPLICE.exec(line.endsWith('\n') ? line.slice(0, -1) : line);
    if (!m) { page += line; continue; }
    if (!line.endsWith('\n')) throw new Error(`splice line for ${m[1]} must end with a newline`);
    if (fragments.includes(m[1])) throw new Error(`fragment ${m[1]} is included twice`);
    page += readFragment(apiDir, m[1]);
    fragments.push(m[1]);
  }
  return { page, fragments };
}

function assemble(apiDir = API_DIR) {
  return assembleParts(apiDir).page;
}

// Concatenation of the JS fragments only (what the Node VM tests search for functions).
function cockpitScripts(apiDir = API_DIR) {
  return assembleParts(apiDir).fragments
    .filter((rel) => rel.endsWith('.js'))
    .map((rel) => fs.readFileSync(path.join(apiDir, rel), 'utf8'))
    .join('');
}

module.exports = { API_DIR, assemble, assembleParts, cockpitScripts };

if (require.main === module) {
  const page = assemble();
  if (process.argv[2]) fs.writeFileSync(process.argv[2], page);
  else process.stdout.write(page);
}
