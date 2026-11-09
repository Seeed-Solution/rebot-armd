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
#                              (e.g. tpuc_recamera). The sources are copied in with
#                              docker cp, built under /tmp/rebot-vision-build in the
#                              container, and the binary is copied out. Paths below
#                              are inside the container:
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

if [ -n "${RECAMERA_DOCKER:-}" ]; then
    C="${RECAMERA_DOCKER}"
    R="${RECAMERA_ROOT:-/recamera}"
    GCC_BIN="${RECAMERA_GCC:-$R/host-tools/gcc/riscv64-linux-musl-x86_64/bin}"
    SDK="${SG200X_SDK_PATH:-$R/sg2002_recamera_emmc}"
    SSCMA="${SSCMA_ROOT:-$R/sscma-example-sg200x}"
    W=/tmp/rebot-vision-build
    docker exec "$C" sh -c "rm -rf $W/src && mkdir -p $W/src $W/build"
    docker cp "${SRC_DIR}/." "$C:$W/src"
    docker exec "$C" bash -c "set -e
        export PATH=${GCC_BIN}:\$PATH SG200X_SDK_PATH=${SDK} SSCMA_ROOT=${SSCMA}
        test -d \$SSCMA_ROOT/components/libwebsockets/prebuilt || { echo 'run components/libwebsockets/fetch_and_build.sh in SSCMA_ROOT first' >&2; exit 1; }
        cmake -S $W/src -B $W/build -DCMAKE_BUILD_TYPE=Release ${UNPINNED} >$W/configure.log || { cat $W/configure.log; exit 1; }
        tail -n 3 $W/configure.log
        cmake --build $W/build -j\$(nproc) -- --no-print-directory 2>&1 | grep -E 'error|warning: unused|Linking|Built target rebot-vision' || true
        test -x $W/build/rebot-vision
        riscv64-unknown-linux-musl-strip -o $W/build/rebot-vision.stripped $W/build/rebot-vision"
    docker cp "$C:$W/build/rebot-vision.stripped" "${OUT_DIR}/rebot-vision"
    docker cp "$C:$W/build/rebot-vision.map" "${OUT_DIR}/rebot-vision.map" 2>/dev/null || true
else
    : "${SG200X_SDK_PATH:?set SG200X_SDK_PATH or RECAMERA_DOCKER}"
    : "${SSCMA_ROOT:?set SSCMA_ROOT or RECAMERA_DOCKER}"
    command -v riscv64-unknown-linux-musl-g++ >/dev/null || { echo "riscv64-unknown-linux-musl-g++ not in PATH" >&2; exit 1; }
    B="${REPO_ROOT}/target/vision-riscv/build"
    cmake -S "${SRC_DIR}" -B "$B" -DCMAKE_BUILD_TYPE=Release "${UNPINNED}"
    cmake --build "$B" -j"$(nproc)"
    riscv64-unknown-linux-musl-strip -o "${OUT_DIR}/rebot-vision" "$B/rebot-vision"
fi

chmod 0755 "${OUT_DIR}/rebot-vision"
echo "rebot-vision: ${OUT_DIR}/rebot-vision ($(ls -lh "${OUT_DIR}/rebot-vision" | awk '{print $5}'))"
