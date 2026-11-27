'use strict';
// Central source and cockpit path helper (CommonJS) reading `.config/source_paths.tsv`.
// Used directly by CommonJS tests (`tests/*.js`) and re-exported by `scripts/source_paths.mjs`.

const fs = require('node:fs');
const path = require('node:path');

const REPO_ROOT = path.resolve(__dirname, '..');
const REGISTRY_PATH = path.join(REPO_ROOT, '.config', 'source_paths.tsv');

function loadRegistry(registryPath = REGISTRY_PATH) {
  const raw = fs.readFileSync(registryPath, 'utf8');
  const map = new Map();
  const lines = raw.split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    const trimmed = lines[i].trim();
    if (!trimmed || trimmed.startsWith('#')) continue;
    const tab = trimmed.indexOf('\t');
    if (tab < 0) {
      throw new Error(`invalid row ${i + 1} in ${registryPath}: expected <key>\\t<path>`);
    }
    const key = trimmed.slice(0, tab).trim();
    const val = trimmed.slice(tab + 1).trim();
    if (!key || !val) {
      throw new Error(`empty key or path on row ${i + 1} in ${registryPath}`);
    }
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

function resolveCockpitInDir(dir) {
  if (dir) {
    const candidate = path.join(path.resolve(dir), 'index.html');
    if (fs.existsSync(candidate)) return candidate;
  }
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
