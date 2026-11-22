use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(mview_has_xthead071)");
    println!("cargo:rerun-if-changed=build.rs");

    if env::var_os("CARGO_FEATURE_XTHEAD_NCC").is_none() {
        return;
    }

    let target = env::var("TARGET").unwrap_or_default();
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    if !(target == "riscv64gc-unknown-linux-musl" && arch == "riscv64" && os == "linux") {
        panic!(
            "feature `xthead-ncc` links the T-Head C906 RVV 0.7.1 LP64D archive and is only supported for target `riscv64gc-unknown-linux-musl` (got TARGET={target}, arch={arch}, os={os})"
        );
    }

    let manifest_dir = PathBuf::from(
        env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR must be set"),
    );
    let native_dir = manifest_dir.join("native").join("xthead071");
    let archive = native_dir.join("libmvs_xthead071.a");
    if !archive.is_file() {
        panic!(
            "missing prebuilt RVV 0.7.1 archive at {}; see native/xthead071/README.md",
            archive.display()
        );
    }
    println!("cargo:rerun-if-changed=native/xthead071/libmvs_xthead071.a");
    println!("cargo:rustc-cfg=mview_has_xthead071");
    println!("cargo:rustc-link-search=native={}", native_dir.display());
    println!("cargo:rustc-link-lib=static=mvs_xthead071");
}
