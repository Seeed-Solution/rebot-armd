#!/usr/bin/env bash
# Resolve logical source paths from `.config/source_paths.tsv`.
# Usage: scripts/source_paths.sh [--repo] [--root <dir>] <key>
# Row grammar (shared with scripts/source_paths.cjs and tests/support/source_paths.rs):
#   - Blank lines and `#`-prefixed comments are ignored; trailing CR (\r) is stripped.
#   - Each data row has exactly two tab-separated fields: <key><TAB><repo-relative-path>.
#   - Paths must be non-empty, relative (no leading /), and contain no `..` segment.
# For `cockpit_html`, ARMD_COCKPIT_HTML overrides unless `--repo` is passed.
set -euo pipefail

IGNORE_ENV=0
REPO_ROOT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --repo) IGNORE_ENV=1; shift ;;
        --root) REPO_ROOT="${2:?missing --root value}"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--repo] [--root <dir>] <key>" >&2
            exit 0
            ;;
        -*)
            echo "Error: unknown option $1" >&2
            exit 2
            ;;
        *) break ;;
    esac
done

if [ $# -ne 1 ]; then
    echo "Usage: $0 [--repo] [--root <dir>] <key>" >&2
    exit 2
fi

KEY="$1"
if [ -z "${REPO_ROOT}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
fi
REGISTRY="${REPO_ROOT}/.config/source_paths.tsv"

if [ "${IGNORE_ENV}" -eq 0 ] && [ "${KEY}" = "cockpit_html" ] && [ -n "${ARMD_COCKPIT_HTML:-}" ]; then
    printf '%s\n' "${ARMD_COCKPIT_HTML}"
    exit 0
fi

if [ ! -f "${REGISTRY}" ]; then
    echo "Error: source path registry not found: ${REGISTRY}" >&2
    exit 1
fi

REL="$(awk -F'\t' -v k="${KEY}" -v reg="${REGISTRY}" '
{
    sub(/\r$/, "", $0)
    if ($0 ~ /^[[:space:]]*$/ || $0 ~ /^[[:space:]]*#/) next
    if (NF != 2) {
        printf("Error: invalid row %d in %s: expected <key>\\t<path>\n", NR, reg) > "/dev/stderr"
        exit 2
    }
    key = $1; gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
    val = $2; gsub(/^[[:space:]]+|[[:space:]]+$/, "", val)
    gsub(/\\/, "/", val)
    if (key == "" || val == "" || substr(val, 1, 1) == "/" || val == ".." || index("/" val "/", "/../") > 0) {
        printf("Error: invalid repo-relative path \"%s\" on row %d in %s\n", val, NR, reg) > "/dev/stderr"
        exit 2
    }
    if (key == k && !found) {
        found = 1
        out = val
    }
}
END {
    if (found) print out
}
' "${REGISTRY}")"

if [ -z "${REL}" ]; then
    echo "Error: unknown source_paths key '${KEY}' in ${REGISTRY}" >&2
    exit 1
fi

printf '%s/%s\n' "${REPO_ROOT}" "${REL}"
