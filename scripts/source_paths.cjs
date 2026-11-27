'use strict';
// Central source and cockpit path helper (CommonJS) reading `.config/source_paths.tsv`.
// Used directly by CommonJS tests (`tests/*.js`) and re-exported by `scripts/source_paths.mjs`.
// Row grammar (shared with scripts/source_paths.sh and tests/support/source_paths.rs):
//   - Blank lines and `#`-prefixed comments are ignored; trailing CR (\r) is stripped.
//   - Each data row has exactly two tab-separated fields: <key>\t<repo-relative-path>.
//   - Paths must be non-empty, relative (no leading /), and contain no `..` segment.

const fs = require('node:fs');
const path = require('node:path');

const REPO_ROOT = path.resolve(__dirname, '..');
const REGISTRY_PATH = path.join(REPO_ROOT, '.config', 'source_paths.tsv');

function validateRelPath(rawVal, key, rowNum, registryPath) {
  const rel = rawVal.trim().replace(/\\/g, '/');
  if (!rel || rel.startsWith('/') || rel.split('/').includes('..')) {
    throw new Error(
      `invalid repo-relative path '${rel}' for key '${key}' on row ${rowNum} in ${registryPath}`,
    );
  }
  return rel;
}

function loadRegistry(registryPath = REGISTRY_PATH) {
  const raw = fs.readFileSync(registryPath, 'utf8');
  const map = new Map();
  const lines = raw.split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].replace(/\r$/, '');
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith('#')) continue;
    const fields = line.split('\t');
    if (fields.length !== 2) {
      throw new Error(`invalid row ${i + 1} in ${registryPath}: expected <key>\\t<path>`);
    }
    const key = fields[0].trim();
    if (!key) {
      throw new Error(`empty key on row ${i + 1} in ${registryPath}`);
    }
    const val = validateRelPath(fields[1], key, i + 1, registryPath);
    if (!map.has(key)) map.set(key, []);
    map.get(key).push(val);
  }
  return map;
}

function sourceEntries(key, registryPath = REGISTRY_PATH) {
  const map = loadRegistry(registryPath);
  const entries = map.get(key);
  if (!entries || entries.length === 0) {
    throw new Error(`unknown source_paths key '${key}' in ${registryPath}`);
  }
  return entries.slice();
}

function sourcePath(key, root = REPO_ROOT) {
  const registryPath = path.join(root, '.config', 'source_paths.tsv');
  return path.join(root, sourceEntries(key, registryPath)[0]);
}

function cockpitHtmlPath(override) {
  if (override) return path.resolve(override);
  if (process.env.ARMD_COCKPIT_HTML) return path.resolve(process.env.ARMD_COCKPIT_HTML);
  return sourcePath('cockpit_html');
}

// Explicit SRC directory resolution: when `dir` is provided, always resolve `index.html` inside
// that directory so HTML and sibling assets (e.g. calib_wizard.js) can never mix versions.
function resolveCockpitInDir(dir) {
  if (dir) return path.join(path.resolve(dir), 'index.html');
  return cockpitHtmlPath();
}

function readCockpitHtml(override) {
  return fs.readFileSync(cockpitHtmlPath(override), 'utf8');
}

function readCockpitHtmlBuffer(override) {
  return fs.readFileSync(cockpitHtmlPath(override));
}

function cockpitAssetPath(rel) {
  return path.join(sourcePath('cockpit_assets_dir'), rel);
}

module.exports = {
  REPO_ROOT,
  REGISTRY_PATH,
  loadRegistry,
  sourceEntries,
  sourcePath,
  cockpitHtmlPath,
  resolveCockpitInDir,
  readCockpitHtml,
  readCockpitHtmlBuffer,
  cockpitAssetPath,
};
