#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# build_vision.sh - cross-compile rebot-vision (kits/recamera-b601dm/vision) for
# the reCamera SG2002 -> target/vision-riscv/rebot-vision
#
# Inputs: sophgo host-tools (gcc/riscv64-linux-musl-x86_64), the reCamera OS SDK
# (SG200X_SDK_PATH) and sscma-example-sg200x at the commit in
# kits/recamera-b601dm/vision/sscma.lock (SSCMA_ROOT), with
# components/libwebsockets/prebuilt built once by its fetch_and_build.sh.
#
# RECAMERA_DOCKER=<container>  docker exec into a running sophgo/tpuc_dev container
#                              (e.g. tpuc_recamera). The sources are copied into an
#                              isolated temporary directory in the container, built
#                              there, verified, and the stripped binary is copied out.
#                              Paths below are inside the container:
#   RECAMERA_ROOT     default /recamera
#   RECAMERA_GCC      default $RECAMERA_ROOT/host-tools/gcc/riscv64-linux-musl-x86_64/bin
#   SG200X_SDK_PATH   default $RECAMERA_ROOT/sg2002_recamera_emmc
#   SSCMA_ROOT        default $RECAMERA_ROOT/sscma-example-sg200x
# Without RECAMERA_DOCKER the build runs on this host: the toolchain must be in
# PATH and SG200X_SDK_PATH / SSCMA_ROOT must be set.
#
# REBOT_VISION_ALLOW_UNPINNED=1 builds against an SSCMA_ROOT off the lock commit.
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${REPO_ROOT}/kits/recamera-b601dm/vision"
OUT_DIR="${REPO_ROOT}/target/vision-riscv"
UNPINNED="-DREBOT_VISION_ALLOW_UNPINNED=$([ "${REBOT_VISION_ALLOW_UNPINNED:-0}" = 1 ] && echo ON || echo OFF)"

# Output layout: every build fills its own generation directory, and one rename of the
# `current` symlink publishes binary, map and checksum together:
#   generations/<id>/{rebot-vision,rebot-vision.map,rebot-vision.sha256}   (never rewritten)
#   current -> generations/<id>
#   rebot-vision, rebot-vision.map, rebot-vision.sha256 -> current/...    (compatibility)
# A reader that needs a matching set resolves `current` once (readlink -f) and reads from that
# directory. A failed or interrupted build changes nothing visible except: a failure while
# `current` still names the generation seen at start removes `current`, so no older binary
# reads as this build's output. The newest KEEP_GENERATIONS generations are kept.
GEN_ROOT="${OUT_DIR}/generations"
CURRENT="${OUT_DIR}/current"
LOCK_FILE="${OUT_DIR}/.publish.lock"
KEEP_GENERATIONS=3
mkdir -p "${GEN_ROOT}"

GEN_ID="$(date -u +%Y%m%dT%H%M%SZ).$$"
STAGE_DIR="$(mktemp -d "${GEN_ROOT}/.staging.$$.XXXXXX")"
STAGE_BIN="${STAGE_DIR}/rebot-vision"
STAGE_MAP="${STAGE_DIR}/rebot-vision.map"
STAGE_SHA="${STAGE_DIR}/rebot-vision.sha256"
HOST_BUILD=""
CONTAINER_WORK=""
PUBLISHED=0
LOCK_DIR="${LOCK_FILE}.d"
LOCK_HELD_DIR=0

# Serialise publishers. flock(1) when present: the kernel drops the lock when the holder dies.
# Otherwise a mkdir lock that records its owner's PID; a lock whose owner is gone is taken over,
# and the EXIT trap releases a lock this shell holds.
acquire_lock() {
    if [ "${LOCK_HELD_DIR}" = 1 ]; then
        return 0
    fi
    if command -v flock >/dev/null 2>&1; then
        exec 9>"${LOCK_FILE}"
        flock -x 9
        return 0
    fi
    while ! mkdir "${LOCK_DIR}" 2>/dev/null; do
        local owner
        owner="$(cat "${LOCK_DIR}/pid" 2>/dev/null || true)"
        if [ -n "${owner}" ] && ! kill -0 "${owner}" 2>/dev/null; then
            rm -rf "${LOCK_DIR}"
            continue
        fi
        sleep 0.1
    done
    echo $$ > "${LOCK_DIR}/pid"
    LOCK_HELD_DIR=1
}

release_lock() {
    if command -v flock >/dev/null 2>&1; then
        exec 9>&-
    elif [ "${LOCK_HELD_DIR}" = 1 ]; then
        rm -rf "${LOCK_DIR}"
        LOCK_HELD_DIR=0
    fi
}

# Atomically point `link` at `target` (rename(2) of a fresh symlink over the old entry).
swap_link() {
    local target="$1" link="$2" tmp
    tmp="${link}.tmp.$$"
    rm -f "${tmp}"
    ln -s "${target}" "${tmp}"
    mv -T "${tmp}" "${link}" 2>/dev/null || mv -fh "${tmp}" "${link}"
}

START_GEN="$(readlink "${CURRENT}" 2>/dev/null || true)"

# Old generations beyond KEEP_GENERATIONS (never the current one) and staging directories of
# builds that are no longer running. Called with the lock held.
collect_garbage() {
    local cur keep=0 d pid
    cur="$(readlink "${CURRENT}" 2>/dev/null || true)"
    for d in $(ls -1d "${GEN_ROOT}"/g.* 2>/dev/null | sort -r); do
        if [ "generations/$(basename "${d}")" = "${cur}" ]; then
            continue
        fi
        keep=$((keep + 1))
        if [ "${keep}" -ge "${KEEP_GENERATIONS}" ]; then
            rm -rf "${d}"
        fi
    done
    for d in "${GEN_ROOT}"/.staging.*; do
        [ -d "${d}" ] || continue
        pid="$(basename "${d}" | cut -d. -f3)"
        if [ "${d}" != "${STAGE_DIR}" ] && [ -n "${pid}" ] && ! kill -0 "${pid}" 2>/dev/null; then
            rm -rf "${d}"
        fi
    done
}

cleanup() {
    if [ "${PUBLISHED}" != 1 ]; then
        acquire_lock || true
        if [ -n "${START_GEN}" ] && [ "$(readlink "${CURRENT}" 2>/dev/null || true)" = "${START_GEN}" ]; then
            rm -f "${CURRENT}"
        elif [ -z "${START_GEN}" ] && [ ! -e "${CURRENT}" ]; then
            # outputs of the layout before generations (regular files) are stale as well
            for f in rebot-vision rebot-vision.map rebot-vision.sha256; do
                if [ -f "${OUT_DIR}/${f}" ] && [ ! -L "${OUT_DIR}/${f}" ]; then rm -f "${OUT_DIR}/${f}"; fi
            done
        fi
        release_lock || true
    fi
    rm -rf "${STAGE_DIR}"
    if [ -n "${HOST_BUILD}" ]; then
        rm -rf "${HOST_BUILD}"
    fi
    if [ -n "${CONTAINER_WORK}" ] && [ -n "${RECAMERA_DOCKER:-}" ]; then
        docker exec "${RECAMERA_DOCKER}" rm -rf "${CONTAINER_WORK}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [ -n "${RECAMERA_DOCKER:-}" ]; then
    C="${RECAMERA_DOCKER}"
    R="${RECAMERA_ROOT:-/recamera}"
    GCC_BIN="${RECAMERA_GCC:-$R/host-tools/gcc/riscv64-linux-musl-x86_64/bin}"
    SDK="${SG200X_SDK_PATH:-$R/sg2002_recamera_emmc}"
    SSCMA="${SSCMA_ROOT:-$R/sscma-example-sg200x}"
    CONTAINER_WORK="$(docker exec "$C" mktemp -d /tmp/rebot-vision-build.XXXXXX)"
    docker exec "$C" mkdir -p "${CONTAINER_WORK}/src" "${CONTAINER_WORK}/build"
    docker cp "${SRC_DIR}/." "$C:${CONTAINER_WORK}/src"
    docker exec -i "$C" bash -s -- "${CONTAINER_WORK}" "${GCC_BIN}" "${SDK}" "${SSCMA}" "${UNPINNED}" <<'EOF'
set -euo pipefail
W="$1"
GCC_BIN="$2"
SDK="$3"
SSCMA="$4"
UNPINNED="$5"

export PATH="${GCC_BIN}:${PATH}"
export SG200X_SDK_PATH="${SDK}"
export SSCMA_ROOT="${SSCMA}"

test -d "${SSCMA_ROOT}/components/libwebsockets/prebuilt" || {
    echo "run components/libwebsockets/fetch_and_build.sh in SSCMA_ROOT first" >&2
    exit 1
}

if ! cmake -S "${W}/src" -B "${W}/build" -DCMAKE_BUILD_TYPE=Release "${UNPINNED}" >"${W}/configure.log" 2>&1; then
    cat "${W}/configure.log" >&2
    exit 1
fi
tail -n 3 "${W}/configure.log"

if ! cmake --build "${W}/build" -j"$(nproc)" -- --no-print-directory >"${W}/build.log" 2>&1; then
    cat "${W}/build.log" >&2
    exit 1
fi
grep -E 'warning: unused|Linking|Built target rebot-vision' "${W}/build.log" || true

test -x "${W}/build/rebot-vision"
riscv64-unknown-linux-musl-strip -o "${W}/build/rebot-vision.stripped" "${W}/build/rebot-vision"
riscv64-unknown-linux-musl-readelf -h "${W}/build/rebot-vision.stripped" | grep -q 'RISC-V' || {
    echo "rebot-vision artifact is not a RISC-V ELF binary" >&2
    exit 1
}
EOF
    docker cp "$C:${CONTAINER_WORK}/build/rebot-vision.stripped" "${STAGE_BIN}"
    docker cp "$C:${CONTAINER_WORK}/build/rebot-vision.map" "${STAGE_MAP}" 2>/dev/null || true
else
    : "${SG200X_SDK_PATH:?set SG200X_SDK_PATH or RECAMERA_DOCKER}"
    : "${SSCMA_ROOT:?set SSCMA_ROOT or RECAMERA_DOCKER}"
    command -v riscv64-unknown-linux-musl-g++ >/dev/null || { echo "riscv64-unknown-linux-musl-g++ not in PATH" >&2; exit 1; }
    HOST_BUILD="$(mktemp -d "${OUT_DIR}/build.XXXXXX")"
    cmake -S "${SRC_DIR}" -B "${HOST_BUILD}" -DCMAKE_BUILD_TYPE=Release "${UNPINNED}"
    cmake --build "${HOST_BUILD}" -j"$(nproc)"
    test -x "${HOST_BUILD}/rebot-vision"
    riscv64-unknown-linux-musl-strip -o "${STAGE_BIN}" "${HOST_BUILD}/rebot-vision"
    riscv64-unknown-linux-musl-readelf -h "${STAGE_BIN}" | grep -q 'RISC-V' || {
        echo "rebot-vision artifact is not a RISC-V ELF binary" >&2
        exit 1
    }
    if [ -f "${HOST_BUILD}/rebot-vision.map" ]; then
        cp "${HOST_BUILD}/rebot-vision.map" "${STAGE_MAP}"
    fi
fi

chmod 0755 "${STAGE_BIN}" "${STAGE_DIR}"
SHA256="$(sha256sum "${STAGE_BIN}" | awk '{print $1}')"
SIZE="$(ls -lh "${STAGE_BIN}" | awk '{print $5}')"
printf '%s  rebot-vision\n' "${SHA256}" > "${STAGE_SHA}"

GEN_DIR="${GEN_ROOT}/g.${GEN_ID}"
acquire_lock
mv -T "${STAGE_DIR}" "${GEN_DIR}" 2>/dev/null || mv "${STAGE_DIR}" "${GEN_DIR}"
swap_link "generations/g.${GEN_ID}" "${CURRENT}"
PUBLISHED=1
for f in rebot-vision rebot-vision.map rebot-vision.sha256; do
    if [ "$(readlink "${OUT_DIR}/${f}" 2>/dev/null || true)" != "current/${f}" ]; then
        swap_link "current/${f}" "${OUT_DIR}/${f}"
    fi
done
collect_garbage
release_lock

echo "rebot-vision: ${GEN_DIR}/rebot-vision (${SIZE}, sha256=${SHA256})"
