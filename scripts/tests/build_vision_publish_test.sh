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

# 2. Lock fallback without flock (REBOT_VISION_NO_FLOCK=1).
LOCK="${OUT}/.publish.lock"
#    a. Leftovers of an owner that died while taking the lock (an empty lock directory of the
#       mkdir scheme, a lock naming a dead PID) do not block the next publisher.
mkdir -p "${LOCK}.d"
dead=999999; while kill -0 "${dead}" 2>/dev/null; do dead=$((dead - 1)); done
ln -s "${dead}" "${LOCK}.owner"
check 'timeout 20 bash -c "$(declare -f publish); T=\"${T}\" ROOT=\"${ROOT}\" publish stale-1 REBOT_VISION_NO_FLOCK=1"'
check '[ "$(cat "${OUT}/rebot-vision")" = "stale-1" ] && [ ! -e "${LOCK}.owner" ] && [ ! -L "${LOCK}.owner" ]'
rmdir "${LOCK}.d" 2>/dev/null || true
#    b. A lock held by a live process is not taken over; the publisher waits for its release.
sleep 30 & holder=$!
ln -s "${holder}" "${LOCK}.owner"
( publish held-1 REBOT_VISION_NO_FLOCK=1 ) & waiter=$!
sleep 2
check 'kill -0 "${waiter}" 2>/dev/null && [ "$(readlink "${LOCK}.owner")" = "${holder}" ]'
kill "${holder}"; wait "${holder}" 2>/dev/null || true
check 'timeout 20 bash -c "while kill -0 ${waiter} 2>/dev/null; do sleep 0.2; done"'
wait "${waiter}"; check '[ $? -eq 0 ] && [ "$(cat "${OUT}/rebot-vision")" = "held-1" ] && [ ! -L "${LOCK}.owner" ]'
#    c. Concurrent publishers all finish, the result is one complete generation, no lock remains.
pids=()
for i in 1 2 3 4; do ( publish "par-${i}" REBOT_VISION_NO_FLOCK=1 ) & pids+=($!); done
for p in "${pids[@]}"; do check "wait ${p}"; done
check '[ ! -L "${LOCK}.owner" ]'
cur="$(cat "${OUT}/rebot-vision")"
check 'case "${cur}" in par-*) true ;; *) false ;; esac'
check '[ "$(cut -d" " -f1 "${OUT}/current/rebot-vision.sha256")" = "$(printf %s "${cur}" | sha256sum | cut -d" " -f1)" ]'

# 3. REBOT_VISION_COPY_TO hands each caller its own generation, even while other publishers
#    switch `current` and collect old generations.
( for i in 1 2 3 4 5 6; do publish "noise-${i}"; done ) & noise=$!
pids=()
for i in 1 2 3; do
    mkdir -p "${T}/copy-${i}"
    ( publish "mine-${i}" REBOT_VISION_COPY_TO="${T}/copy-${i}" ) & pids+=($!)
done
for p in "${pids[@]}"; do check "wait ${p}"; done
wait "${noise}"
for i in 1 2 3; do
    check '[ "$(cat "${T}/copy-'"${i}"'/rebot-vision")" = "mine-'"${i}"'" ]'
    check '[ "$(cut -d" " -f1 "${T}/copy-'"${i}"'/rebot-vision.sha256")" = "$(printf mine-'"${i}"' | sha256sum | cut -d" " -f1)" ]'
    check '[ -x "${T}/copy-'"${i}"'/rebot-vision" ]'
done

if [ "${FAIL}" != 0 ]; then echo "build_vision publish tests: FAILED" >&2; exit 1; fi
echo "build_vision publish tests: all passed"
