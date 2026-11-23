#!/bin/sh
# Linker for host builds on aarch64 Linux (see .cargo/config.toml): the system C compiler driver
# with rustup's bundled rust-lld instead of GNU ld. Linking one test binary takes 1.4 s instead of
# 9.8 s (spark, 2026-11-23). Falls back to plain `cc` when the toolchain has no rust-lld.
# x86_64 Linux already links with rust-lld by default (Rust 1.90 and later).
gcc_ld="$(rustc --print sysroot 2>/dev/null)/lib/rustlib/aarch64-unknown-linux-gnu/bin/gcc-ld"
if [ -x "$gcc_ld/ld.lld" ]; then
    exec "${CC:-cc}" -B"$gcc_ld" -fuse-ld=lld "$@"
fi
exec "${CC:-cc}" "$@"
