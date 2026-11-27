//! Cockpit page assembly (build.rs + src/api/cockpit/assembler): the page armd serves is the
//! assembly of the current sources, the Node and sh assemblers produce the same bytes, and the
//! splice rules refuse malformed fragment sets.

#[path = "../src/api/cockpit/assembler/assemble.rs"]
mod assemble;

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;

use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth};
use tokio::net::TcpListener;

const SERVED: &str = include_str!(concat!(env!("OUT_DIR"), "/cockpit/index.html"));

/// sha256 of src/api/index.html before the split (2626d1fa). Move-only commits keep it.
const PRE_SPLIT_SHA256: &str = "10315408c7d4c85f96356d0448857998779c7d16edceece8f113cdfc6a454f17";

fn api_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src/api")
}

fn source_page() -> String {
    assemble::assemble(&api_dir()).unwrap_or_else(|e| panic!("{e}")).page
}

#[test]
fn assembled_page_is_byte_identical_to_the_pre_split_page() {
    assert_eq!(armd::gravity::compute_sha256(SERVED.as_bytes()), PRE_SPLIT_SHA256);
}

#[test]
fn embedded_page_is_the_assembly_of_the_current_sources() {
    assert!(SERVED == source_page(), "build.rs output differs from the sources (stale build?)");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn served_index_is_the_assembled_page() {
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::default()));
    let router = create_router(supervisor, OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
    let body = reqwest::get(format!("http://{addr}/")).await.unwrap().text().await.unwrap();
    assert!(body == source_page(), "GET / is not the assembled page");
}

#[test]
fn sh_assembler_matches_the_served_page() {
    let out = Command::new("sh")
        .arg(api_dir().join("cockpit/assembler/assemble.sh"))
        .output()
        .expect("run sh assemble.sh");
    assert!(out.status.success(), "assemble.sh failed: {}", String::from_utf8_lossy(&out.stderr));
    assert!(out.stdout == SERVED.as_bytes(), "assemble.sh output differs from the served page");
}

#[test]
fn node_assembler_matches_the_served_page() {
    let script = api_dir().join("cockpit/assembler/assemble.cjs");
    let Ok(out) = Command::new("node").arg(&script).output() else {
        eprintln!("node not found; skipping the Node assembler check");
        return;
    };
    assert!(out.status.success(), "assemble.cjs failed: {}", String::from_utf8_lossy(&out.stderr));
    assert!(out.stdout == SERVED.as_bytes(), "assemble.cjs output differs from the served page");
}

fn scratch(files: &[(&str, &str)]) -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    for (rel, text) in files {
        let path = dir.path().join(rel);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }
    dir
}

fn assemble_err(files: &[(&str, &str)]) -> String {
    match assemble::assemble(scratch(files).path()) {
        Ok(_) => panic!("assembly of {files:?} should fail"),
        Err(e) => e,
    }
}

#[test]
fn splice_replaces_the_whole_line_in_order() {
    let dir = scratch(&[
        ("index.html", "<a>\n<!-- @include cockpit/x/a.css -->\n<b>\n<!-- @include cockpit/b.js -->\n</c>\n"),
        ("cockpit/x/a.css", "A1\nA2\n"),
        ("cockpit/b.js", "B\n"),
        ("cockpit/assembler/notes.js", "not a fragment\n"),
    ]);
    let out = assemble::assemble(dir.path()).unwrap();
    assert_eq!(out.page, "<a>\nA1\nA2\n<b>\nB\n</c>\n");
    assert_eq!(out.fragments, vec!["cockpit/x/a.css", "cockpit/b.js"]);
    assert_eq!(assemble::splice_target(" <!-- @include cockpit/b.js -->"), None, "indented lines are not splice points");
}

#[test]
fn malformed_fragment_sets_are_refused() {
    let shell = |rel: &str| format!("x\n<!-- @include {rel} -->\n");
    let e = assemble_err(&[("index.html", &shell("cockpit/a.js")), ("cockpit/a.js", "no newline")]);
    assert!(e.contains("end with a newline"), "{e}");
    let e = assemble_err(&[("index.html", &shell("cockpit/a.js")), ("cockpit/a.js", "<!-- @include cockpit/b.js -->\n"), ("cockpit/b.js", "b\n")]);
    assert!(e.contains("no nested includes"), "{e}");
    let e = assemble_err(&[("index.html", &shell("cockpit/../a.js")), ("a.js", "a\n")]);
    assert!(e.contains("invalid fragment path"), "{e}");
    let e = assemble_err(&[("index.html", &shell("other/a.js")), ("other/a.js", "a\n")]);
    assert!(e.contains("invalid fragment path"), "{e}");
    let e = assemble_err(&[("index.html", &shell("cockpit/missing.js"))]);
    assert!(e.contains("cannot read fragment"), "{e}");
    let twice = "<!-- @include cockpit/a.js -->\n<!-- @include cockpit/a.js -->\n";
    let e = assemble_err(&[("index.html", twice), ("cockpit/a.js", "a\n")]);
    assert!(e.contains("included twice"), "{e}");
    let e = assemble_err(&[("index.html", &shell("cockpit/a.js")), ("cockpit/a.js", "a\n"), ("cockpit/orphan.css", "o\n")]);
    assert!(e.contains("not included") && e.contains("cockpit/orphan.css"), "{e}");
}
