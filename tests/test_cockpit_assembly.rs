//! Cockpit page assembly (build.rs + src/api/cockpit/assembler): the page armd serves is the
//! assembly of the current sources, the Node assembler produces the same bytes, both assemblers
//! give the same result on the shared cases in tests/fixtures/cockpit_assembly/cases.json, and the
//! deb copies the page from the build-script OUT_DIR of its own cargo build.

#[path = "../src/api/cockpit/assembler/assemble.rs"]
mod assemble;

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;

use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth};
use tokio::net::TcpListener;

const SERVED: &str = include_str!(concat!(env!("OUT_DIR"), "/cockpit/index.html"));

fn api_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src/api")
}

fn source_page() -> String {
    assemble::assemble(&api_dir()).unwrap_or_else(|e| panic!("{e}")).page
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
fn node_assembler_matches_the_served_page() {
    let script = api_dir().join("cockpit/assembler/assemble.cjs");
    let Ok(out) = Command::new("node").arg(&script).output() else {
        eprintln!("node not found; skipping the Node assembler check");
        return;
    };
    assert!(out.status.success(), "assemble.cjs failed: {}", String::from_utf8_lossy(&out.stderr));
    assert!(out.stdout == SERVED.as_bytes(), "assemble.cjs output differs from the served page");
}

fn node_available() -> bool {
    Command::new("node").arg("--version").output().is_ok_and(|o| o.status.success())
}

fn write_case(files: &serde_json::Map<String, serde_json::Value>) -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    for (rel, body) in files {
        let path = dir.path().join(rel);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let bytes = match body {
            serde_json::Value::String(text) => text.as_bytes().to_vec(),
            other => {
                let hex = other["hex"].as_str().expect("file body: string or {hex}");
                (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect()
            }
        };
        std::fs::write(path, bytes).unwrap();
    }
    dir
}

#[test]
fn shared_cases_with_the_rust_assembler() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/cockpit_assembly/cases.json");
    let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let cases = doc["cases"].as_array().unwrap();
    assert!(cases.len() >= 20, "{} cases", cases.len());
    for case in cases {
        let name = case["name"].as_str().unwrap();
        let dir = write_case(case["files"].as_object().unwrap());
        let got = assemble::assemble(dir.path());
        match (case.get("page").and_then(|p| p.as_str()), case.get("error").and_then(|e| e.as_str()), got) {
            (Some(want), None, Ok(out)) => {
                assert!(out.page.as_bytes() == want.as_bytes(), "{name}: page {:?}, want {want:?}", out.page);
                assert!(out.fragments.iter().all(|f| f.starts_with("cockpit/")), "{name}: {:?}", out.fragments);
            }
            (None, Some(want), Err(e)) => assert!(e.contains(want), "{name}: error {e:?}, want {want:?}"),
            (_, _, got) => panic!("{name}: got {:?}", got.map(|a| a.page)),
        }
    }
}

#[test]
fn shared_cases_with_the_node_assembler() {
    if !node_available() {
        eprintln!("node not found; skipping the Node assembler cases");
        return;
    }
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/test_cockpit_assembly_cases.js");
    let out = Command::new("node").arg(&script).output().expect("run node");
    assert!(out.status.success(), "{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
}

#[test]
fn splice_order_and_indented_lines() {
    assert_eq!(assemble::splice_target("<!-- @include cockpit/b.js -->"), Some("cockpit/b.js"));
    assert_eq!(assemble::splice_target(" <!-- @include cockpit/b.js -->"), None, "indented lines are not splice points");
}

#[test]
fn deb_copies_the_page_from_the_armd_build_script_out_dir() {
    let script = include_str!("../scripts/build_deb.sh");
    assert!(script.contains("--message-format=json-render-diagnostics"), "build_deb.sh reads cargo's build-script messages");
    assert!(script.contains("COCKPIT_PAGE=\"${ARMD_OUT_DIRS}/cockpit/index.html\""), "page from the armd OUT_DIR");
    assert!(script.contains("cp \"${COCKPIT_PAGE}\" \"${STAGING_DIR}/usr/share/rebot-teleop/index.html\""), "deb copies that page");
    assert!(!script.contains("assemble.sh") && !script.contains("cockpit_assembler"), "no second assembler on the deb path");
    assert!(!api_dir().join("cockpit/assembler/assemble.sh").exists(), "assemble.sh is gone");
}
