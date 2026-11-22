#!/usr/bin/env sh
set -eu
export LC_ALL=C

caller_cwd=$PWD
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ $# -ge 1 ] && [ -n "$1" ]; then
    case "$1" in
        /*) dest=$1 ;;
        *) dest="$caller_cwd/$1" ;;
    esac
else
    dest=$src
fi
B=${XUANTIE_BIN:-/home/harve/waste-cvi/host-tools/gcc/riscv64-linux-musl-x86_64/bin}
P=$B/riscv64-unknown-linux-musl

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

cd "$out"
"$P-gcc" --version
"$P-gcc" -march=rv64gcv0p7 -mabi=lp64d -O2 -ffp-contract=off \
    -Wall -Wextra -Werror \
    -I"$src" -c "$src/ncc.c" -o "$out/ncc.raw.o"
"$P-readelf" -A "$out/ncc.raw.o"
"$P-nm" -u "$out/ncc.raw.o"
"$P-objdump" -dr "$out/ncc.raw.o"

"$P-objcopy" --remove-section=.riscv.attributes "$out/ncc.raw.o" "$out/ncc.o"
"$P-objcopy" -O binary --only-section=.text "$out/ncc.raw.o" "$out/raw.text"
"$P-objcopy" -O binary --only-section=.text "$out/ncc.o" "$out/stripped.text"
cmp -s "$out/raw.text" "$out/stripped.text"

attr_after=$("$P-readelf" -A "$out/ncc.o")
if [ -n "$attr_after" ]; then
    echo "error: expected empty readelf -A output for ncc.o, got:" >&2
    printf '%s\n' "$attr_after" >&2
    exit 1
fi

"$P-ar" rcsD "$out/libmvs_xthead071.a" "$out/ncc.o"

mkdir -p "$dest"
cp "$out/libmvs_xthead071.a" "$dest/libmvs_xthead071.a"
if [ "$dest" != "$src" ]; then
    cp "$out/ncc.raw.o" "$dest/ncc.raw.o"
    cp "$out/ncc.o" "$dest/ncc.o"
    cp "$out/raw.text" "$dest/raw.text"
fi

sha256sum ncc.raw.o ncc.o raw.text libmvs_xthead071.a
(cd "$src" && sha256sum ncc.c ncc.h rebuild.sh)
