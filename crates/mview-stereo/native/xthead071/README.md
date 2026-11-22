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

## Provenance (Commit 4: `mvs071_coarse_dots` + `mvs071_half_moments`)

- **`readelf -A ncc.o`**:
  `Tag_RISCV_arch: "rv64i2p0_m2p0_a2p0_f2p0_d2p0_c2p0_v0p7_zvamo0p7_zvlsseg0p7"`
- **`nm -u ncc.o`**: empty (no external symbol dependencies)
- **`nm -g ncc.o`**: `mvs071_coarse_dots`, `mvs071_half_moments`
- **`objdump -dr ncc.o`**: verified 0 FMA (`vfmacc`/`vfmadd`/`vfnmacc`/`vfnmadd`) instructions, 0 unordered `vfredsum.vs` instructions, and no external helper calls:
  - `mvs071_coarse_dots`: uses `vsetvli`, `vfmv.v.f`, `vle.v`, `vwcvtu.x.x.v`, `vfwcvt.f.xu.v`, `vfmul.vf`, `vfadd.vv`, `vse.v`.
  - `mvs071_half_moments`: specializes `n == 9` (when `vsetvl_e32m4(9) == 9`) and all four phases (`0..=3`) outside the row loop, plus a strip-mined general path for any other odd `n`; uses `vle.v`, `vwcvtu.x.x.v`, `vwaddu.vv`, `vadd.vv`, `vwaddu.wv`, `vwmulu.vv`, `vredsum.vs`, `vfwcvt.f.xu.v`, `vfmul.vf`, `vfmul.vv`, and ordered `vfredosum.vs` (8 call sites across the 4+4 specialized phase loops) for strict row-major `dot`.
- **Raw `ncc.o` SHA-256** (identical across two independent fresh builds):
  `a9839334c4fc0d841a9fc8c85883a873799fda0a854cfab2bb597de5caacd83c`
- **`libmvs_xthead071.a` SHA-256** (identical across two independent fresh builds):
  `f8f427eda83de1b067bc431052143afbd3ba537db3c262fdd099c1714e1e6412`

## Verification & Rebuild

Verify committed sources and archive locally:

```sh
(cd crates/mview-stereo/native/xthead071 && sha256sum -c SHA256SUMS)
```

Rebuild on `wsl2-local` from a copy of this directory (produces deterministic `ncc.o` and `libmvs_xthead071.a`):

```sh
./rebuild.sh
```
