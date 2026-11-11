#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# build_deb.sh - RISC-V (reCamera SG2002) build of armd
#
# Usage: build_deb.sh [--flavor recamera|lite] [VERSION]
#
#   --flavor recamera  (default) core + kits/recamera-b601dm: the rebot-teleop
#                      deb (armd, rebot-vision, init script, Wi-Fi AP, CAN setup,
#                      cockpit, URDF, gallery manifest)
#                      -> dist/rebot-teleop_<VERSION>_riscv64.deb
#                      VERSION default: kits/recamera-b601dm/packaging/debian/control
#                      rebot-vision: REBOT_VISION_BIN=<prebuilt binary>, or
#                      RECAMERA_DOCKER=<tpuc_dev container> to build it with
#                      scripts/build_vision.sh (see that script for the paths)
#                      REBOT_SDK_DIR=<rebot-companion checkout>: its rebot/ Python
#                      package goes to /usr/share/rebot-teleop/python (required)
#   --flavor lite      core only: the armd binary with LICENSE and
#                      THIRD_PARTY_NOTICES.md, no init script, no kit files
#                      -> dist/armd-lite_<VERSION>_riscv64.tar.gz
#                      VERSION default: Cargo.toml
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGING_DIR="${REPO_ROOT}/kits/recamera-b601dm/packaging"
TARGET_TRIPLE="riscv64gc-unknown-linux-musl"

FLAVOR="recamera"
CUSTOM_VERSION=""
while [ $# -gt 0 ]; do
    case "$1" in
        --flavor) FLAVOR="${2:-}"; shift 2 ;;
        --flavor=*) FLAVOR="${1#--flavor=}"; shift ;;
        -h|--help) sed -n '4,23p' "${BASH_SOURCE[0]}"; exit 0 ;;
        -*) echo "Error: unknown option $1" >&2; exit 1 ;;
        *) CUSTOM_VERSION="$1"; shift ;;
    esac
done
case "$FLAVOR" in
    recamera|lite) ;;
    *) echo "Error: --flavor must be recamera or lite (got '${FLAVOR}')" >&2; exit 1 ;;
esac
# The recamera deb runs user programs that import the rebot SDK; lite has no SDK.
if [ "$FLAVOR" = "recamera" ] && { [ -z "${REBOT_SDK_DIR:-}" ] || [ ! -d "${REBOT_SDK_DIR}/rebot" ]; }; then
    echo "Error: the recamera flavor needs the rebot Python SDK: set REBOT_SDK_DIR=<rebot-companion checkout>" >&2
    echo "       (a directory with rebot/; got '${REBOT_SDK_DIR:-}')." >&2
    exit 1
fi

# SG2002 RISC-V musl toolchain (sophgo host-tools: gcc/riscv64-linux-musl-x86_64).
# Only its T-Head (lib64v0p7_xthead/lp64d) libraries are used; rust-lld links.
# REC_TOOLCHAIN and RISCV_SYSROOT are accepted as older names.
TOOLCHAIN_ROOT="${RECAMERA_TOOLCHAIN:-${REC_TOOLCHAIN:-${RISCV_SYSROOT:-}}}"

# Target version (from CLI arg, or debian/control for recamera, or Cargo.toml)
if [ -n "$CUSTOM_VERSION" ]; then
    VERSION="$CUSTOM_VERSION"
elif [ "$FLAVOR" = "recamera" ] && [ -f "${PACKAGING_DIR}/debian/control" ]; then
    VERSION="$(grep -m1 '^Version:' "${PACKAGING_DIR}/debian/control" | awk '{print $2}')"
else
    VERSION="$(grep -m1 '^version = ' "${REPO_ROOT}/Cargo.toml" | cut -d '"' -f 2)"
fi

echo "====================================================================="
if [ "$FLAVOR" = "lite" ]; then
    echo "Building armd lite tarball v${VERSION} (${TARGET_TRIPLE})"
else
    echo "Building rebot-teleop debian package v${VERSION} (${TARGET_TRIPLE})"
fi
echo "====================================================================="

# Check toolchain availability
if ! command -v cargo >/dev/null 2>&1; then
    echo "Error: cargo is not installed or not in PATH" >&2
    exit 1
fi

if [ "$FLAVOR" = "recamera" ] && ! command -v dpkg-deb >/dev/null 2>&1; then
    echo "Error: dpkg-deb is not installed or not in PATH" >&2
    exit 1
fi

if [ "$FLAVOR" = "recamera" ] && [ -z "${REBOT_VISION_BIN:-}" ] && [ -z "${RECAMERA_DOCKER:-}" ]; then
    echo "Error: the recamera flavor needs rebot-vision: set RECAMERA_DOCKER=<tpuc_dev container>" >&2
    echo "       to build it (scripts/build_vision.sh), or REBOT_VISION_BIN=<prebuilt binary>." >&2
    exit 1
fi

# Linker flags: either RUSTFLAGS is set by the caller, or they are built from
# RECAMERA_TOOLCHAIN. RUSTFLAGS replaces the rustflags in .cargo/config.toml,
# so the full list is given here.
if [ -z "${RUSTFLAGS:-}" ]; then
    if [ -z "$TOOLCHAIN_ROOT" ] || [ ! -d "$TOOLCHAIN_ROOT" ]; then
        echo "Error: set RECAMERA_TOOLCHAIN to the riscv64-linux-musl-x86_64 toolchain directory" >&2
        echo "       (from https://github.com/sophgo/host-tools, gcc/riscv64-linux-musl-x86_64)," >&2
        echo "       or set RUSTFLAGS with the linker search paths yourself." >&2
        exit 1
    fi
    GCC_LIB_DIR="$(ls -d "${TOOLCHAIN_ROOT}"/lib/gcc/riscv64-unknown-linux-musl/*/lib64v0p7_xthead/lp64d 2>/dev/null | head -n1)"
    if [ -z "$GCC_LIB_DIR" ]; then
        echo "Error: no lib/gcc/riscv64-unknown-linux-musl/<version>/lib64v0p7_xthead/lp64d under ${TOOLCHAIN_ROOT}" >&2
        exit 1
    fi
    export RUSTFLAGS="-C linker=rust-lld -C target-feature=+crt-static \
-L ${GCC_LIB_DIR} \
-L ${TOOLCHAIN_ROOT}/sysroot/lib64v0p7_xthead/lp64d \
-L ${TOOLCHAIN_ROOT}/sysroot/usr/lib64v0p7_xthead/lp64d"
fi

# 1. Compile armd binary for RISC-V musl
if [ "$FLAVOR" = "lite" ]; then echo "[1/3] Compiling armd for ${TARGET_TRIPLE} (release mode)..."; else echo "[1/5] Compiling armd for ${TARGET_TRIPLE} (release mode)..."; fi
cd "${REPO_ROOT}"
cargo build --release --target "${TARGET_TRIPLE}"

BIN_PATH="${REPO_ROOT}/target/${TARGET_TRIPLE}/release/armd"
if [ ! -f "${BIN_PATH}" ]; then
    echo "Error: compiled binary not found at ${BIN_PATH}" >&2
    exit 1
fi

BIN_SIZE=$(ls -lh "${BIN_PATH}" | awk '{print $5}')
echo "      Binary successfully compiled: ${BIN_PATH} (${BIN_SIZE})"

DIST_DIR="${REPO_ROOT}/dist"
mkdir -p "${DIST_DIR}"

# Lite flavor: core only. Deterministic tarball (sorted, root-owned, fixed mtime).
if [ "$FLAVOR" = "lite" ]; then
    LITE_NAME="armd-lite_${VERSION}_riscv64"
    LITE_STAGING="${REPO_ROOT}/target/lite_staging/${LITE_NAME}"
    rm -rf "${LITE_STAGING}"
    mkdir -p "${LITE_STAGING}"
    echo "[2/3] Assembling ${LITE_NAME}..."
    cp -f "${BIN_PATH}" "${LITE_STAGING}/armd"
    chmod 0755 "${LITE_STAGING}/armd"
    cp -f "${REPO_ROOT}/LICENSE" "${REPO_ROOT}/THIRD_PARTY_NOTICES.md" "${LITE_STAGING}/"
    chmod 0644 "${LITE_STAGING}/LICENSE" "${LITE_STAGING}/THIRD_PARTY_NOTICES.md"
    OUTPUT_TAR="${DIST_DIR}/${LITE_NAME}.tar.gz"
    echo "[3/3] Packing ${OUTPUT_TAR}..."
    tar --sort=name --owner=0 --group=0 --numeric-owner --mtime='@0' \
        -C "${LITE_STAGING}/.." -cf - "${LITE_NAME}" | gzip -n -9 > "${OUTPUT_TAR}"
    tar -tvzf "${OUTPUT_TAR}"
    echo "====================================================================="
    echo "Build complete!"
    echo "Lite artifact: ${OUTPUT_TAR}"
    echo "Size:          $(ls -lh "${OUTPUT_TAR}" | awk '{print $5}')"
    echo "====================================================================="
    exit 0
fi

# 2. rebot-vision (C++, SG2002 SDK)
if [ -n "${REBOT_VISION_BIN:-}" ]; then
    VISION_BIN="${REBOT_VISION_BIN}"
    echo "[2/5] Using prebuilt rebot-vision: ${VISION_BIN}"
else
    echo "[2/5] Building rebot-vision in container ${RECAMERA_DOCKER}..."
    "${SCRIPT_DIR}/build_vision.sh"
    VISION_BIN="${REPO_ROOT}/target/vision-riscv/rebot-vision"
fi
if [ ! -f "${VISION_BIN}" ]; then
    echo "Error: rebot-vision binary not found at ${VISION_BIN}" >&2
    exit 1
fi

# 3. Prepare staging directory
STAGING_DIR="${REPO_ROOT}/target/deb_staging/rebot-teleop"
rm -rf "${STAGING_DIR}"
mkdir -p "${STAGING_DIR}/DEBIAN"
mkdir -p "${STAGING_DIR}/etc/init.d"
mkdir -p "${STAGING_DIR}/usr/bin"
mkdir -p "${STAGING_DIR}/usr/local/bin"
mkdir -p "${STAGING_DIR}/usr/share/rebot-teleop"
mkdir -p "${STAGING_DIR}/usr/share/doc/rebot-teleop"

echo "[3/5] Assembling rootfs staging structure..."

# Copy runtime files
cp -f "${PACKAGING_DIR}/rootfs/etc/init.d/K92rebot-teleop" "${STAGING_DIR}/etc/init.d/K92rebot-teleop"
chmod 0755 "${STAGING_DIR}/etc/init.d/K92rebot-teleop"
# No S92 link in the package: postinst creates it only on devices without the reCamera console.

cp -r "${PACKAGING_DIR}/rootfs/usr/share/rebot-teleop/"* "${STAGING_DIR}/usr/share/rebot-teleop/"

# rebot Python SDK (stdlib only) for user programs run by rebot-app-run: the rebot/ package
# of a rebot-companion checkout (REBOT_SDK_DIR), without __pycache__
{
    SDK_DEST="${STAGING_DIR}/usr/share/rebot-teleop/python"
    mkdir -p "${SDK_DEST}"
    (cd "${REBOT_SDK_DIR}" && find rebot -name __pycache__ -prune -o -type f ! -name '*.pyc' -print) |
        while IFS= read -r f; do
            mkdir -p "${SDK_DEST}/$(dirname "$f")"
            cp -f "${REBOT_SDK_DIR}/$f" "${SDK_DEST}/$f"
        done
    echo "      rebot Python SDK copied from ${REBOT_SDK_DIR}/rebot"
}

# Arm profile of the kit (loaded by the init script with --config)
mkdir -p "${STAGING_DIR}/usr/share/rebot-teleop/profile"
cp -f "${REPO_ROOT}/kits/recamera-b601dm/profile/rebot-b601dm.json" "${STAGING_DIR}/usr/share/rebot-teleop/profile/rebot-b601dm.json"

# Ensure web UI index.html is present in usr/share/rebot-teleop
if [ -f "${REPO_ROOT}/src/api/index.html" ]; then
    cp -f "${REPO_ROOT}/src/api/index.html" "${STAGING_DIR}/usr/share/rebot-teleop/index.html"
fi
# Gallery manifest: its "version" is the package version (the console reports it as the
# installed version), written here so the two cannot differ.
MANIFEST="${STAGING_DIR}/usr/share/rebot-teleop/rebot-teleop.json"
sed -i -E 's/^(  "version": )"[^"]*"/\1"'"${VERSION}"'"/' "${MANIFEST}"
if ! grep -q "^  \"version\": \"${VERSION}\"," "${MANIFEST}"; then
    echo "Error: could not set \"version\": \"${VERSION}\" in ${MANIFEST}" >&2
    exit 1
fi
find "${STAGING_DIR}/usr/share/rebot-teleop" -type f -exec chmod 0644 {} +
chmod 0755 "${STAGING_DIR}/usr/share/rebot-teleop/rebot-vision-run"
chmod 0755 "${STAGING_DIR}/usr/share/rebot-teleop/rebot-app-run"

# rebot-vision and the licence notices for the code it contains
cp -f "${VISION_BIN}" "${STAGING_DIR}/usr/bin/rebot-vision"
chmod 0755 "${STAGING_DIR}/usr/bin/rebot-vision"
cp -f "${REPO_ROOT}/LICENSE" "${REPO_ROOT}/THIRD_PARTY_NOTICES.md" \
    "${REPO_ROOT}/kits/recamera-b601dm/vision/LICENSE-APACHE" "${STAGING_DIR}/usr/share/doc/rebot-teleop/"
chmod 0644 "${STAGING_DIR}/usr/share/doc/rebot-teleop/"*

# Copy compiled binary to usr/bin/armd and provide backward-compatible symlink at usr/local/bin/armd
cp -f "${BIN_PATH}" "${STAGING_DIR}/usr/bin/armd"
chmod 0755 "${STAGING_DIR}/usr/bin/armd"
ln -sf /usr/bin/armd "${STAGING_DIR}/usr/local/bin/armd"

# Copy debian control scripts
cp -f "${PACKAGING_DIR}/debian/postinst" "${STAGING_DIR}/DEBIAN/postinst"
cp -f "${PACKAGING_DIR}/debian/prerm" "${STAGING_DIR}/DEBIAN/prerm"
chmod 0755 "${STAGING_DIR}/DEBIAN/postinst" "${STAGING_DIR}/DEBIAN/prerm"

# Compute installed size in KB
INSTALLED_SIZE=$(du -ks "${STAGING_DIR}" | awk '{print $1}')

# Process control file
sed -e "s/^Version:.*/Version: ${VERSION}/" \
    -e "s/^Installed-Size:.*/Installed-Size: ${INSTALLED_SIZE}/" \
    "${PACKAGING_DIR}/debian/control" > "${STAGING_DIR}/DEBIAN/control"
chmod 0644 "${STAGING_DIR}/DEBIAN/control"

# 3. Build deb package with gzip compression & root ownership (opkg/dpkg compliant)
echo "[4/5] Packaging deb archive with dpkg-deb..."
DEB_NAME="rebot-teleop_${VERSION}_riscv64.deb"
OUTPUT_DEB="${DIST_DIR}/${DEB_NAME}"

dpkg-deb -Zgzip --root-owner-group --build "${STAGING_DIR}" "${OUTPUT_DEB}"

# 4. Verify output package
echo "[5/5] Verifying generated deb package..."
dpkg-deb -I "${OUTPUT_DEB}"
echo "---------------------------------------------------------------------"
dpkg-deb -c "${OUTPUT_DEB}"

echo "====================================================================="
echo "Build complete!"
echo "Package artifact: ${OUTPUT_DEB}"
echo "Package size:     $(ls -lh "${OUTPUT_DEB}" | awk '{print $5}')"
echo "====================================================================="
