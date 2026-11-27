#!/usr/bin/env bash
# Resolve logical source paths from `.config/source_paths.tsv`.
# Usage: scripts/source_paths.sh <key>
# For `cockpit_html`, ARMD_COCKPIT_HTML takes precedence when set and non-empty.
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <key>" >&2
    exit 2
fi

KEY="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REGISTRY="${REPO_ROOT}/.config/source_paths.tsv"

if [ "${KEY}" = "cockpit_html" ] && [ -n "${ARMD_COCKPIT_HTML:-}" ]; then
    printf '%s\n' "${ARMD_COCKPIT_HTML}"
    exit 0
fi

if [ ! -f "${REGISTRY}" ]; then
    echo "Error: source path registry not found: ${REGISTRY}" >&2
    exit 1
fi

REL="$(awk -F'\t' -v k="${KEY}" '!/^[[:space:]]*(#|$)/ && $1 == k { print $2; exit }' "${REGISTRY}")"
if [ -z "${REL}" ]; then
    echo "Error: unknown source_paths key '${KEY}' in ${REGISTRY}" >&2
    exit 1
fi

printf '%s/%s\n' "${REPO_ROOT}" "${REL}"
