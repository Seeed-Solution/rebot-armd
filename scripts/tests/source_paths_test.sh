#!/usr/bin/env bash
# Self-tests for the source-path indirection layer (.config/source_paths.tsv,
# scripts/source_paths.{sh,cjs,mjs}, tests/support/source_paths.rs, and build_deb.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "${T}"' EXIT

# 1. Shell helper: default resolution, env override, unknown key failure.
got="$("${REPO_ROOT}/scripts/source_paths.sh" cockpit_html)"
[ "${got}" = "${REPO_ROOT}/src/api/index.html" ] || { echo "FAIL sh default: ${got}" >&2; exit 1; }
[ -f "${got}" ] || { echo "FAIL sh cockpit_html missing: ${got}" >&2; exit 1; }

override="$(ARMD_COCKPIT_HTML="/tmp/override_cockpit.html" "${REPO_ROOT}/scripts/source_paths.sh" cockpit_html)"
[ "${override}" = "/tmp/override_cockpit.html" ] || { echo "FAIL sh override: ${override}" >&2; exit 1; }

if "${REPO_ROOT}/scripts/source_paths.sh" nonexistent_key_xyz >/dev/null 2>&1; then
    echo "FAIL sh unknown key should exit non-zero" >&2
    exit 1
fi

# 2. Node CJS + ESM helpers.
node -e '
const assert = require("node:assert");
const fs = require("node:fs");
const path = require("node:path");
const sp = require(process.argv[1]);
assert.strictEqual(sp.cockpitHtmlPath(), path.join(sp.REPO_ROOT, "src/api/index.html"));
assert.ok(sp.readCockpitHtml().includes("<!DOCTYPE html>"));
assert.ok(Buffer.isBuffer(sp.readCockpitHtmlBuffer()));
assert.strictEqual(sp.cockpitHtmlPath("/tmp/x.html"), path.resolve("/tmp/x.html"));
assert.strictEqual(sp.resolveCockpitInDir(path.join(sp.REPO_ROOT, "src/api")), path.join(sp.REPO_ROOT, "src/api/index.html"));
assert.strictEqual(sp.resolveCockpitInDir("/tmp/definitely_missing_dir_xyz"), path.join(sp.REPO_ROOT, "src/api/index.html"));
assert.throws(() => sp.sourceEntries("nonexistent_key_xyz"), /unknown source_paths key/);
' "${REPO_ROOT}/scripts/source_paths.cjs"

node --input-type=module -e '
import assert from "node:assert";
import { cockpitHtmlPath, readCockpitHtml, sourceEntries, REPO_ROOT } from "./scripts/source_paths.mjs";
import path from "node:path";
assert.strictEqual(cockpitHtmlPath(), path.join(REPO_ROOT, "src/api/index.html"));
assert.ok(readCockpitHtml().length > 1000);
assert.deepStrictEqual(sourceEntries("grasp"), ["src/grasp.rs"]);
'

# 3. Rust helper compiles cleanly with -D warnings and passes all unit tests.
rustc --edition 2021 --test -D warnings \
    "${REPO_ROOT}/tests/support/source_paths.rs" \
    -o "${T}/source_paths_test"
CARGO_MANIFEST_DIR="${REPO_ROOT}" "${T}/source_paths_test"

# 4. build_deb.sh syntax and dry-run still succeed.
bash -n "${REPO_ROOT}/scripts/build_deb.sh"
bash "${REPO_ROOT}/scripts/build_deb.sh" --dry-run >/dev/null

# 5. No remaining scripts/*.mjs hardcode src/api/index.html in fs.readFileSync.
if grep -rn "readFileSync.*src/api/index\.html" "${REPO_ROOT}/scripts/"*.mjs; then
    echo "FAIL: scripts/ still contains direct readFileSync of src/api/index.html" >&2
    exit 1
fi

echo "source_paths_test: all passed"
