# RVV 0.7.1 Native Kernels (`libmvs_xthead071.a`)

Prebuilt T-Head Xuantie RVV 0.7.1 static archive for `mview-stereo` on the reCamera SG2002
(`riscv64gc-unknown-linux-musl`, LP64D ABI). Host and other targets build scalar-only without
linking this archive; normal RISC-V builds consume the committed `libmvs_xthead071.a` without
invoking GCC.

## Toolchain Pin

- **Compiler**: `riscv64-unknown-linux-musl-gcc (Xuantie-900 linux-5.10.4 musl gcc Toolchain V2.6.1 B-20220906) 10.2.0`
- **Build host**: `wsl2-local` (x86_64 native): `/home/harve/waste-cvi/host-tools/gcc/riscv64-linux-musl-x86_64/bin`
- **Target ABI**: `lp64d`
- **Compile flags**: `-march=rv64gcv0p7 -mabi=lp64d -O2 -ffp-contract=off -Wall -Wextra -Werror`
- **Archive command**: `riscv64-unknown-linux-musl-ar rcsD libmvs_xthead071.a ncc.o`

## Provenance (Commit 3: `mvs071_coarse_dots`)

- **`readelf -A ncc.o`**:
  `Tag_RISCV_arch: "rv64i2p0_m2p0_a2p0_f2p0_d2p0_c2p0_v0p7_zvamo0p7_zvlsseg0p7"`
- **`nm -u ncc.o`**: empty (no external symbol dependencies)
- **`objdump -dr ncc.o`**: verified 0 FMA (`vfmacc`/`vfmadd`/`vfnmacc`/`vfnmadd`) instructions and no unexpected helper calls; uses `vsetvli`, `vfmv.v.f`, `vle.v`, `vwcvtu.x.x.v`, `vfwcvt.f.xu.v`, `vfmul.vf`, `vfadd.vv`, `vse.v`.
- **Raw `ncc.o` SHA-256** (identical across two independent fresh builds):
  `908c5e2098af81ecbb280a1a6949b5fff52f7b19c561dc51ba336fc27c7f4cf0`
- **`libmvs_xthead071.a` SHA-256** (identical across two independent fresh builds):
  `1a1d7a12f50b2575cd9790de97971857298e1bed34c35b0eb75600912ce2002b`

## Verification & Rebuild

Verify committed sources and archive locally:

```sh
(cd crates/mview-stereo/native/xthead071 && sha256sum -c SHA256SUMS)
```

Rebuild on `wsl2-local` from a copy of this directory (produces deterministic `ncc.o` and `libmvs_xthead071.a`):

```sh
./rebuild.sh
```
