// Central source and cockpit path helper (ESM), backed by scripts/source_paths.cjs
// and `.config/source_paths.tsv`.
import cjs from './source_paths.cjs';

export const {
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
} = cjs;
