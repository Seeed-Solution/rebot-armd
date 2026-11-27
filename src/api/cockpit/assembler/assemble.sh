#!/bin/sh
# sh/awk assembler of the cockpit page (for packaging without cargo's OUT_DIR), same splice rule
# as assemble.rs: a shell line that is exactly `<!-- @include cockpit/<path> -->` is replaced by
# that fragment. Usage: assemble.sh [out.html] (default stdout).
# tests/test_cockpit_assembly.rs checks that this output equals the page armd serves.
set -eu
API_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${1:-/dev/stdout}"
awk -v dir="${API_DIR}" '
/^<!-- @include [^ ]+ -->$/ {
    rel = $3
    if (rel !~ /^cockpit(\/[A-Za-z0-9_.-]+)+$/ || rel ~ /\/\.\.?(\/|$)/) {
        printf("assemble.sh: invalid fragment path %s\n", rel) > "/dev/stderr"; exit 1
    }
    f = dir "/" rel
    while ((r = (getline l < f)) > 0) print l
    if (r < 0) { printf("assemble.sh: cannot read %s\n", f) > "/dev/stderr"; exit 1 }
    close(f)
    next
}
{ print }
' "${API_DIR}/index.html" > "${OUT}"
