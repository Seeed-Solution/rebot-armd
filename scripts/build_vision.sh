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
mkdir -p "${OUT_DIR}"

OUT_BIN="${OUT_DIR}/rebot-vision"
OUT_MAP="${OUT_DIR}/rebot-vision.map"
OUT_SHA="${OUT_DIR}/rebot-vision.sha256"
LOCK_FILE="${OUT_DIR}/.publish.lock"
STAGE_DIR="$(mktemp -d "${OUT_DIR}/.stage.XXXXXX")"
STAGE_BIN="${STAGE_DIR}/rebot-vision"
STAGE_MAP="${STAGE_DIR}/rebot-vision.map"
STAGE_SHA="${STAGE_DIR}/rebot-vision.sha256"
HOST_BUILD=""
CONTAINER_WORK=""
PUBLISHED=0

file_identity() {
    if [ ! -e "$1" ]; then
        return 0
    fi
    stat -c '%d:%i' "$1" 2>/dev/null || stat -f '%d:%i' "$1" 2>/dev/null || true
}

with_publish_lock() {
    if command -v flock >/dev/null 2>&1; then
        (
            flock -x 9
            "$@"
        ) 9>"${LOCK_FILE}"
    else
        local lockdir="${LOCK_FILE}.d"
        while ! mkdir "${lockdir}" 2>/dev/null; do sleep 0.05; done
        "$@"
        local rc=$?
        rmdir "${lockdir}" 2>/dev/null || true
        return "${rc}"
    fi
}

PREV_INODE="$(file_identity "${OUT_BIN}")"

clear_stale_if_unchanged() {
    if [ -n "${PREV_INODE}" ] && [ "$(file_identity "${OUT_BIN}")" = "${PREV_INODE}" ]; then
        rm -f "${OUT_BIN}" "${OUT_MAP}" "${OUT_SHA}"
    fi
}

cleanup() {
    if [ "${PUBLISHED}" != 1 ]; then
        with_publish_lock clear_stale_if_unchanged || true
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

chmod 0755 "${STAGE_BIN}"
SHA256="$(sha256sum "${STAGE_BIN}" | awk '{print $1}')"
SIZE="$(ls -lh "${STAGE_BIN}" | awk '{print $5}')"
printf '%s  rebot-vision\n' "${SHA256}" > "${STAGE_SHA}"

publish_artifacts() {
    mv -f "${STAGE_BIN}" "${OUT_BIN}"
    if [ -f "${STAGE_MAP}" ]; then
        mv -f "${STAGE_MAP}" "${OUT_MAP}"
    else
        rm -f "${OUT_MAP}"
    fi
    mv -f "${STAGE_SHA}" "${OUT_SHA}"
}

with_publish_lock publish_artifacts
PUBLISHED=1

echo "rebot-vision: ${OUT_BIN} (${SIZE}, sha256=${SHA256})"
