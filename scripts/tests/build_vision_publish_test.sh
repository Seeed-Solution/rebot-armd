#!/usr/bin/env bash
# Publication tests of scripts/build_vision.sh without a toolchain: copies the script into a
# checkout path that contains a space and publishes stand-in binaries with
# REBOT_VISION_PUBLISH_PREBUILT.  Usage: scripts/tests/build_vision_publish_test.sh
set -euo pipefail

SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/build_vision.sh"
T="$(mktemp -d)"
trap 'rm -rf "${T}"' EXIT
ROOT="${T}/My Project"
mkdir -p "${ROOT}/scripts" "${T}/My"
echo keep > "${T}/My/marker"
cp "${SCRIPT}" "${ROOT}/scripts/build_vision.sh"
OUT="${ROOT}/target/vision-riscv"
FAIL=0
check() { if ! eval "$1"; then echo "FAIL: $1" >&2; FAIL=1; fi; }

publish() {  # $1 content, further args: env assignments
    local content="$1"; shift
    local bin="${T}/bin.$$.${RANDOM}"
    printf '%s' "${content}" > "${bin}"
    (cd "${T}" && env "$@" REBOT_VISION_PUBLISH_PREBUILT="${bin}" bash "${ROOT}/scripts/build_vision.sh" >/dev/null)
}

# 1. Garbage collection in a checkout path with a space: five publishes keep three generations
#    and touch nothing outside the generations directory.
for i in 1 2 3 4 5; do publish "gen-${i}"; sleep 1; done
check '[ -f "${T}/My/marker" ]'
gens=("${OUT}/generations"/g.*)
check '[ "${#gens[@]}" -eq 3 ]'
check '[ "$(cat "${OUT}/rebot-vision")" = "gen-5" ]'
check '[ "$(cat "${OUT}/current/rebot-vision.sha256" | cut -d" " -f1)" = "$(printf gen-5 | sha256sum | cut -d" " -f1)" ]'

if [ "${FAIL}" != 0 ]; then echo "build_vision publish tests: FAILED" >&2; exit 1; fi
echo "build_vision publish tests: all passed"
