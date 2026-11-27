'use strict';
// Node assembler of the cockpit page, a line-by-line port of assemble.rs (build.rs): same splice rule,
// same validation (path grammar, non-empty fragments ending in LF, no nested splice line incl. CRLF,
// each fragment once, no orphan fragments) and the same error texts. Strict UTF-8, BOM kept.
// Library: require(...).assemble(apiDir) -> page string. CLI: node assemble.cjs [out.html] (default stdout).
// tests/test_cockpit_assembly.rs checks that this output equals the page armd serves, and both
// assemblers run the shared cases in tests/fixtures/cockpit_assembly/cases.json.
const fs = require('node:fs');
const path = require('node:path');

const API_DIR = path.resolve(__dirname, '..', '..');
const SHELL = 'index.html';
const FRAGMENT_ROOT = 'cockpit';
const ASSEMBLER_DIR = 'cockpit/assembler';
const PREFIX = '<!-- @include ';
const SUFFIX = ' -->';
const FRAGMENT_EXTS = new Set(['.css', '.js', '.html']);

// Fragment path of a splice line, null for any other line (assemble.rs splice_target).
function spliceTarget(line) {
  if (line.length < PREFIX.length + SUFFIX.length || !line.startsWith(PREFIX) || !line.endsWith(SUFFIX)) return null;
  return line.slice(PREFIX.length, line.length - SUFFIX.length);
}

function checkRel(rel) {
  const okSegment = (s) => s !== '' && s !== '.' && s !== '..' && /^[A-Za-z0-9_.-]+$/.test(s);
  const segments = rel.split('/');
  if (segments.length < 2 || segments[0] !== FRAGMENT_ROOT || !segments.slice(1).every(okSegment)) {
    throw new Error(`invalid fragment path \`${rel}\` (want cockpit/<segments>)`);
  }
}

const UTF8 = new TextDecoder('utf-8', { fatal: true, ignoreBOM: true });
function readUtf8(file, what) {
  try {
    return UTF8.decode(fs.readFileSync(file));
  } catch (e) {
    throw new Error(`cannot read ${what}${file}: ${e.message}`);
  }
}

// Rust str::lines(): split on LF, drop one trailing CR per line.
const rustLines = (text) => text.split('\n').map((l) => (l.endsWith('\r') ? l.slice(0, -1) : l));

function readFragment(apiDir, rel) {
  checkRel(rel);
  const text = readUtf8(path.join(apiDir, rel), 'fragment ');
  if (text === '' || !text.endsWith('\n')) throw new Error(`fragment ${rel} must be non-empty and end with a newline`);
  const nested = rustLines(text).find((l) => spliceTarget(l) !== null);
  if (nested !== undefined) throw new Error(`fragment ${rel} contains a splice line \`${nested}\` (no nested includes)`);
  return text;
}

function collectFiles(dir, apiDir, out) {
  let names;
  try { names = fs.readdirSync(dir); } catch (e) { throw new Error(`cannot list ${dir}: ${e.message}`); }
  for (const name of names) {
    const full = path.join(dir, name);
    const rel = path.relative(apiDir, full).split(path.sep).join('/');
    if (fs.statSync(full).isDirectory()) {
      if (rel !== ASSEMBLER_DIR) collectFiles(full, apiDir, out);
    } else if (FRAGMENT_EXTS.has(path.extname(name))) {
      out.push(rel);
    }
  }
}

// Every fragment-like file under cockpit/ (outside the assembler dir), sorted.
function fragmentFiles(apiDir) {
  const out = [];
  const root = path.join(apiDir, FRAGMENT_ROOT);
  if (fs.existsSync(root) && fs.statSync(root).isDirectory()) collectFiles(root, apiDir, out);
  return out.sort();
}

// Splices the fragments into the shell; returns { page, fragments } (fragments in splice order).
function assembleParts(apiDir = API_DIR) {
  const shell = readUtf8(path.join(apiDir, SHELL), '');
  const fragments = [];
  let page = '';
  for (const line of shell.split(/(?<=\n)/)) {
    const rel = spliceTarget(line.endsWith('\n') ? line.slice(0, -1) : line);
    if (rel === null) { page += line; continue; }
    if (!line.endsWith('\n')) throw new Error(`splice line for ${rel} must end with a newline`);
    if (fragments.includes(rel)) throw new Error(`fragment ${rel} is included twice`);
    page += readFragment(apiDir, rel);
    fragments.push(rel);
  }
  const orphans = fragmentFiles(apiDir).filter((f) => !fragments.includes(f));
  if (orphans.length) throw new Error(`fragments not included by ${SHELL}: ${orphans.join(', ')}`);
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

module.exports = { API_DIR, assemble, assembleParts, cockpitScripts, spliceTarget, fragmentFiles };

if (require.main === module) {
  const page = assemble();
  if (process.argv[2]) fs.writeFileSync(process.argv[2], page);
  else process.stdout.write(page);
}
