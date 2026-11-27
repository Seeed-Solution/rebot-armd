//! Assembles the cockpit page (`src/api/index.html` + fragments under `src/api/cockpit/`) into
//! `$OUT_DIR/cockpit/index.html`, which `src/api/mod.rs` serves through `include_str!`.
//! The splice rules are in `src/api/cockpit/assembler/assemble.rs`.

#[path = "src/api/cockpit/assembler/assemble.rs"]
mod assemble;

use std::path::PathBuf;

fn main() {
    let api_dir = PathBuf::from("src/api");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/api/index.html");
    println!("cargo:rerun-if-changed=src/api/cockpit");
    let assembled = assemble::assemble(&api_dir).unwrap_or_else(|e| panic!("cockpit assembly failed: {e}"));
    for rel in &assembled.fragments {
        println!("cargo:rerun-if-changed=src/api/{rel}");
    }
    let out_dir = PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR")).join("cockpit");
    std::fs::create_dir_all(&out_dir).expect("create $OUT_DIR/cockpit");
    std::fs::write(out_dir.join("index.html"), assembled.page).expect("write $OUT_DIR/cockpit/index.html");
}
