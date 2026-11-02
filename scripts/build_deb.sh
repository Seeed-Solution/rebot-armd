#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# build_deb.sh - Automated RISC-V Debian package builder for rebot-teleop (armd)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGING_DIR="${REPO_ROOT}/packaging/rebot-teleop"
TARGET_TRIPLE="riscv64gc-unknown-linux-musl"

# Default toolchain sysroot search paths for SG2002 RISC-V musl
DEFAULT_SYSROOT="${RECAMERA_TOOLCHAIN}"
TOOLCHAIN_ROOT="${REC_TOOLCHAIN:-${RISCV_SYSROOT:-$DEFAULT_SYSROOT}}"

# Target version (from CLI arg, or debian/control, or Cargo.toml)
CUSTOM_VERSION="${1:-}"
if [ -n "$CUSTOM_VERSION" ]; then
    VERSION="$CUSTOM_VERSION"
elif [ -f "${PACKAGING_DIR}/debian/control" ]; then
    VERSION="$(grep -m1 '^Version:' "${PACKAGING_DIR}/debian/control" | awk '{print $2}')"
else
    VERSION="$(grep -m1 '^version = ' "${REPO_ROOT}/Cargo.toml" | cut -d '"' -f 2)"
fi

echo "====================================================================="
echo "Building rebot-teleop debian package v${VERSION} (${TARGET_TRIPLE})"
echo "====================================================================="

# Check toolchain availability
if ! command -v cargo >/dev/null 2>&1; then
    echo "Error: cargo is not installed or not in PATH" >&2
    exit 1
fi

if ! command -v dpkg-deb >/dev/null 2>&1; then
    echo "Error: dpkg-deb is not installed or not in PATH" >&2
    exit 1
fi

# Configure linker flags if not already provided via RUSTFLAGS or .cargo/config.toml
if [ -z "${RUSTFLAGS:-}" ] && [ -d "$TOOLCHAIN_ROOT" ]; then
    export RUSTFLAGS="-C linker=rust-lld -C target-feature=+crt-static \
-L ${TOOLCHAIN_ROOT}/lib/gcc/riscv64-unknown-linux-musl/10.2.0/lib64v0p7_xthead/lp64d \
-L ${TOOLCHAIN_ROOT}/sysroot/lib64v0p7_xthead/lp64d \
-L ${TOOLCHAIN_ROOT}/sysroot/usr/lib64v0p7_xthead/lp64d"
fi

# 1. Compile armd binary for RISC-V musl
echo "[1/4] Compiling armd for ${TARGET_TRIPLE} (release mode)..."
cd "${REPO_ROOT}"
cargo build --release --target "${TARGET_TRIPLE}"

BIN_PATH="${REPO_ROOT}/target/${TARGET_TRIPLE}/release/armd"
if [ ! -f "${BIN_PATH}" ]; then
    echo "Error: compiled binary not found at ${BIN_PATH}" >&2
    exit 1
fi

BIN_SIZE=$(ls -lh "${BIN_PATH}" | awk '{print $5}')
echo "      Binary successfully compiled: ${BIN_PATH} (${BIN_SIZE})"

# 2. Prepare staging directory
STAGING_DIR="${REPO_ROOT}/target/deb_staging/rebot-teleop"
rm -rf "${STAGING_DIR}"
mkdir -p "${STAGING_DIR}/DEBIAN"
mkdir -p "${STAGING_DIR}/etc/init.d"
mkdir -p "${STAGING_DIR}/usr/local/bin"
mkdir -p "${STAGING_DIR}/usr/share/rebot-teleop"

echo "[2/4] Assembling rootfs staging structure..."

# Copy runtime files
cp -f "${PACKAGING_DIR}/rootfs/etc/init.d/K92rebot-teleop" "${STAGING_DIR}/etc/init.d/K92rebot-teleop"
chmod 0755 "${STAGING_DIR}/etc/init.d/K92rebot-teleop"

cp -r "${PACKAGING_DIR}/rootfs/usr/share/rebot-teleop/"* "${STAGING_DIR}/usr/share/rebot-teleop/"
find "${STAGING_DIR}/usr/share/rebot-teleop" -type f -exec chmod 0644 {} +

# Copy compiled binary
cp -f "${BIN_PATH}" "${STAGING_DIR}/usr/local/bin/armd"
chmod 0755 "${STAGING_DIR}/usr/local/bin/armd"

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
echo "[3/4] Packaging deb archive with dpkg-deb..."
DIST_DIR="${REPO_ROOT}/dist"
mkdir -p "${DIST_DIR}"
DEB_NAME="rebot-teleop_${VERSION}_riscv64.deb"
OUTPUT_DEB="${DIST_DIR}/${DEB_NAME}"

dpkg-deb -Zgzip --root-owner-group --build "${STAGING_DIR}" "${OUTPUT_DEB}"

# 4. Verify output package
echo "[4/4] Verifying generated deb package..."
dpkg-deb -I "${OUTPUT_DEB}"
echo "---------------------------------------------------------------------"
dpkg-deb -c "${OUTPUT_DEB}"

echo "====================================================================="
echo "Build complete!"
echo "Package artifact: ${OUTPUT_DEB}"
echo "Package size:     $(ls -lh "${OUTPUT_DEB}" | awk '{print $5}')"
echo "====================================================================="
