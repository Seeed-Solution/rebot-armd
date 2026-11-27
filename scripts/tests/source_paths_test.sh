#!/usr/bin/env bash
# Self-tests for the source-path indirection layer (.config/source_paths.tsv,
# scripts/source_paths.{sh,cjs,mjs}, tests/support/source_paths.rs, and build_deb.sh).
# Live-checkout assertions derive expectations from the registry rather than pinning
# pre-split paths; fixed paths and malformed-row cases run against temporary fixtures.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "${T}"' EXIT

# 1. Fixture tests: CRLF parity, --repo flag, path traversal rejection, extra-tab rejection.
FIXTURE="${T}/fixture_repo"
mkdir -p "${FIXTURE}/.config" "${FIXTURE}/src/api"
printf '<!DOCTYPE html><title>fixture</title>\n' > "${FIXTURE}/src/api/assembled.html"
printf '# comment\r\ncockpit_html\tsrc/api/assembled.html\r\ngrasp\tsrc/grasp\r\n' > "${FIXTURE}/.config/source_paths.tsv"

got_fix="$("${REPO_ROOT}/scripts/source_paths.sh" --root "${FIXTURE}" cockpit_html)"
[ "${got_fix}" = "${FIXTURE}/src/api/assembled.html" ] || { echo "FAIL sh CRLF fixture: ${got_fix}" >&2; exit 1; }

got_ovr="$(ARMD_COCKPIT_HTML="/tmp/override_cockpit.html" "${REPO_ROOT}/scripts/source_paths.sh" --root "${FIXTURE}" cockpit_html)"
[ "${got_ovr}" = "/tmp/override_cockpit.html" ] || { echo "FAIL sh env override: ${got_ovr}" >&2; exit 1; }

got_repo="$(ARMD_COCKPIT_HTML="/tmp/override_cockpit.html" "${REPO_ROOT}/scripts/source_paths.sh" --repo --root "${FIXTURE}" cockpit_html)"
[ "${got_repo}" = "${FIXTURE}/src/api/assembled.html" ] || { echo "FAIL sh --repo ignored env: ${got_repo}" >&2; exit 1; }

if "${REPO_ROOT}/scripts/source_paths.sh" --root "${FIXTURE}" nonexistent_key_xyz >/dev/null 2>&1; then
    echo "FAIL sh unknown key should exit non-zero" >&2
    exit 1
fi

BAD_FIX="${T}/bad_repo"
mkdir -p "${BAD_FIX}/.config"
printf 'cockpit_html\t../escape.html\n' > "${BAD_FIX}/.config/source_paths.tsv"
if "${REPO_ROOT}/scripts/source_paths.sh" --root "${BAD_FIX}" cockpit_html >/dev/null 2>&1; then
    echo "FAIL sh path traversal should exit non-zero" >&2
    exit 1
fi
printf 'cockpit_html\tsrc/a.html\textra_col\n' > "${BAD_FIX}/.config/source_paths.tsv"
if "${REPO_ROOT}/scripts/source_paths.sh" --root "${BAD_FIX}" cockpit_html >/dev/null 2>&1; then
    echo "FAIL sh extra tab column should exit non-zero" >&2
    exit 1
fi

# 2. Live-checkout parity across shell, CJS, and ESM (invoked from /tmp to verify CWD independence).
live_sh="$(env -u ARMD_COCKPIT_HTML "${REPO_ROOT}/scripts/source_paths.sh" cockpit_html)"
[ -f "${live_sh}" ] || { echo "FAIL live cockpit_html missing: ${live_sh}" >&2; exit 1; }

(
    cd "${T}"
    env -u ARMD_COCKPIT_HTML node -e '
const assert = require("node:assert");
const fs = require("node:fs");
const path = require("node:path");
const sp = require(process.argv[1]);
const liveSh = process.argv[2];
const fixtureRoot = process.argv[3];
const badRoot = process.argv[4];

// Live checkout parity with shell helper (no pinned path).
assert.strictEqual(sp.cockpitHtmlPath(), liveSh);
assert.ok(sp.readCockpitHtml().includes("<!DOCTYPE html>"));
assert.ok(Buffer.isBuffer(sp.readCockpitHtmlBuffer()));
assert.strictEqual(sp.cockpitHtmlPath("/tmp/x.html"), path.resolve("/tmp/x.html"));

// Explicit SRC directory never falls back to registry HTML.
assert.strictEqual(
  sp.resolveCockpitInDir("/tmp/explicit_src_dir"),
  path.join(path.resolve("/tmp/explicit_src_dir"), "index.html"),
);
assert.strictEqual(sp.resolveCockpitInDir(null), liveSh);

// Fixture CRLF + bad row rejection parity.
assert.strictEqual(
  sp.sourcePath("cockpit_html", fixtureRoot),
  path.join(fixtureRoot, "src/api/assembled.html"),
);
assert.deepStrictEqual(
  sp.sourceEntries("grasp", path.join(fixtureRoot, ".config/source_paths.tsv")),
  ["src/grasp"],
);
assert.throws(
  () => sp.loadRegistry(path.join(badRoot, ".config/source_paths.tsv")),
  /invalid row/,
);
assert.throws(() => sp.sourceEntries("nonexistent_key_xyz"), /unknown source_paths key/);
' "${REPO_ROOT}/scripts/source_paths.cjs" "${live_sh}" "${FIXTURE}" "${BAD_FIX}"

    env -u ARMD_COCKPIT_HTML node --input-type=module -e '
import assert from "node:assert";
import { pathToFileURL } from "node:url";
const mod = await import(pathToFileURL(process.argv[1]).href);
assert.strictEqual(mod.cockpitHtmlPath(), process.argv[2]);
assert.ok(mod.readCockpitHtml().length > 1000);
assert.ok(mod.sourceEntries("grasp").length >= 1);
' "${REPO_ROOT}/scripts/source_paths.mjs" "${live_sh}"
)

# 3. Rust helper compiles cleanly with -D warnings and passes all unit tests.
rustc --edition 2021 --test -D warnings \
    "${REPO_ROOT}/tests/support/source_paths.rs" \
    -o "${T}/source_paths_test"
CARGO_MANIFEST_DIR="${REPO_ROOT}" "${T}/source_paths_test"

# 4. build_deb.sh syntax and dry-run still succeed.
bash -n "${REPO_ROOT}/scripts/build_deb.sh"
bash -n "${REPO_ROOT}/scripts/source_paths.sh"
bash "${REPO_ROOT}/scripts/build_deb.sh" --dry-run >/dev/null

# 5. No remaining scripts/*.mjs hardcode src/api/index.html in fs.readFileSync.
if grep -rn "readFileSync.*src/api/index\.html" "${REPO_ROOT}/scripts/"*.mjs; then
    echo "FAIL: scripts/ still contains direct readFileSync of src/api/index.html" >&2
    exit 1
fi

echo "source_paths_test: all passed"
