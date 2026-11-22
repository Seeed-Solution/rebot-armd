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
# REBOT_VISION_COPY_TO=<dir> copies this build's own generation (binary, map, checksum) into
# <dir> while the publish lock is held, before old generations are collected: a caller such
# as build_deb.sh reads from there, not from the shared `current` pointer, which another
# publisher may switch. The last line printed names this build's generation.
# REBOT_VISION_PUBLISH_PREBUILT=<binary> skips the build and publishes that file (and
# <binary>.map when present) through the same output path; scripts/tests use it.
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
LOCK_LINK="${LOCK_FILE}.owner"
LOCK_HELD=0

pid_alive() {
    kill -0 "$1" 2>/dev/null || ps -p "$1" >/dev/null 2>&1
}

# Serialise publishers. flock(1) when present: the kernel drops the lock when the holder dies.
# Otherwise LOCK_LINK, a symlink whose target is the owner's PID: ln -s creates lock and PID in
# one atomic step, so no lock exists without an owner. A lock whose owner is gone is renamed
# aside and removed only when the aside still names that owner; a successor's lock moved by
# mistake is put back. The EXIT trap releases a lock this shell holds, and release removes the
# link only while it names this shell. (REBOT_VISION_NO_FLOCK=1 forces the fallback; tests.)
acquire_lock() {
    if [ "${LOCK_HELD}" = 1 ]; then
        return 0
    fi
    if [ "${REBOT_VISION_NO_FLOCK:-0}" != 1 ] && command -v flock >/dev/null 2>&1; then
        exec 9>"${LOCK_FILE}"
        flock -x 9
        LOCK_HELD=1
        return 0
    fi
    local owner aside moved
    aside="${LOCK_LINK}.stale.$$"
    while ! ln -s "$$" "${LOCK_LINK}" 2>/dev/null; do
        owner="$(readlink "${LOCK_LINK}" 2>/dev/null || true)"
        if [ -n "${owner}" ] && ! pid_alive "${owner}"; then
            if mv "${LOCK_LINK}" "${aside}" 2>/dev/null; then
                moved="$(readlink "${aside}" 2>/dev/null || true)"
                if [ "${moved}" != "${owner}" ] && [ -n "${moved}" ]; then
                    ln -s "${moved}" "${LOCK_LINK}" 2>/dev/null \
                        || echo "build_vision.sh: lock of ${moved} moved during reclaim" >&2
                fi
                rm -f "${aside}"
            fi
            continue
        fi
        sleep 0.1
    done
    LOCK_HELD=1
}

release_lock() {
    if [ "${LOCK_HELD}" != 1 ]; then
        return 0
    fi
    if [ "${REBOT_VISION_NO_FLOCK:-0}" != 1 ] && command -v flock >/dev/null 2>&1; then
        exec 9>&-
    elif [ "$(readlink "${LOCK_LINK}" 2>/dev/null || true)" = "$$" ]; then
        rm -f "${LOCK_LINK}"
    fi
    LOCK_HELD=0
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

# Remove $1 only when it is a real directory directly inside GEN_ROOT.
remove_gen_child() {
    local p="$1" rest
    case "${p}" in
        "${GEN_ROOT}"/*) rest="${p#"${GEN_ROOT}"/}" ;;
        *) echo "build_vision.sh: refusing to remove ${p}" >&2; return 1 ;;
    esac
    case "${rest}" in
        "" | . | .. | */*) echo "build_vision.sh: refusing to remove ${p}" >&2; return 1 ;;
    esac
    if [ -d "${p}" ] && [ ! -L "${p}" ]; then
        rm -rf -- "${p}"
    fi
}

# Old generations beyond KEEP_GENERATIONS (never the current one) and staging directories of
# builds that are no longer running. Called with the lock held. Paths stay quoted (glob
# arrays), so a checkout path with spaces cannot split into deletion candidates.
collect_garbage() {
    local cur keep=0 d name pid i
    local LC_ALL=C
    local gens=() stages=()
    cur="$(readlink "${CURRENT}" 2>/dev/null || true)"
    shopt -s nullglob
    gens=("${GEN_ROOT}"/g.*)
    stages=("${GEN_ROOT}"/.staging.*)
    shopt -u nullglob
    # g.<UTC timestamp>.<pid>: the glob sorts oldest first, walk newest first
    for ((i = ${#gens[@]} - 1; i >= 0; i--)); do
        d="${gens[i]}"
        if [ "generations/${d##*/}" = "${cur}" ]; then
            continue
        fi
        keep=$((keep + 1))
        if [ "${keep}" -ge "${KEEP_GENERATIONS}" ]; then
            remove_gen_child "${d}"
        fi
    done
    for d in ${stages[@]+"${stages[@]}"}; do
        name="${d##*/}"
        pid="$(printf '%s' "${name}" | cut -d. -f3)"
        if [ "${d}" != "${STAGE_DIR}" ] && [ -n "${pid}" ] && ! pid_alive "${pid}"; then
            remove_gen_child "${d}"
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
    fi
    release_lock || true
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

if [ -n "${REBOT_VISION_PUBLISH_PREBUILT:-}" ]; then
    cp "${REBOT_VISION_PUBLISH_PREBUILT}" "${STAGE_BIN}"
    if [ -f "${REBOT_VISION_PUBLISH_PREBUILT}.map" ]; then
        cp "${REBOT_VISION_PUBLISH_PREBUILT}.map" "${STAGE_MAP}"
    fi
elif [ -n "${RECAMERA_DOCKER:-}" ]; then
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
if [ -n "${REBOT_VISION_COPY_TO:-}" ]; then
    mkdir -p "${REBOT_VISION_COPY_TO}"
    cp -p "${GEN_DIR}/rebot-vision" "${GEN_DIR}/rebot-vision.sha256" "${REBOT_VISION_COPY_TO}/"
    if [ -f "${GEN_DIR}/rebot-vision.map" ]; then
        cp -p "${GEN_DIR}/rebot-vision.map" "${REBOT_VISION_COPY_TO}/"
    else
        rm -f "${REBOT_VISION_COPY_TO}/rebot-vision.map"  # not this build's
    fi
fi
collect_garbage
release_lock

echo "rebot-vision: ${GEN_DIR}/rebot-vision (${SIZE}, sha256=${SHA256})"
