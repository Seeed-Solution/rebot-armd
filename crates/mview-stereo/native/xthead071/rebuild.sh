#!/usr/bin/env sh
set -eu
export LC_ALL=C

src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
dest=${1:-$src}
B=${XUANTIE_BIN:-/home/harve/waste-cvi/host-tools/gcc/riscv64-linux-musl-x86_64/bin}
P=$B/riscv64-unknown-linux-musl

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

cd "$out"
"$P-gcc" --version
"$P-gcc" -march=rv64gcv0p7 -mabi=lp64d -O2 -ffp-contract=off \
    -Wall -Wextra -Werror \
    -I"$src" -c "$src/ncc.c" -o "$out/ncc.o"
"$P-readelf" -A "$out/ncc.o"
"$P-nm" -u "$out/ncc.o"
"$P-objdump" -dr "$out/ncc.o"
"$P-ar" rcsD "$out/libmvs_xthead071.a" "$out/ncc.o"

mkdir -p "$dest"
cp "$out/libmvs_xthead071.a" "$dest/libmvs_xthead071.a"
if [ "$dest" != "$src" ]; then
    cp "$out/ncc.o" "$dest/ncc.o"
fi

sha256sum ncc.o libmvs_xthead071.a
(cd "$src" && sha256sum ncc.c ncc.h rebuild.sh)
