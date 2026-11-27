//! Cockpit page assembly (build.rs + src/api/cockpit/assembler): the page armd serves is the
//! assembly of the current sources, the Node assembler produces the same bytes, both assemblers
//! give the same result on the shared cases in tests/fixtures/cockpit_assembly/cases.json, and
//! scripts/build_deb.sh packages armd cleanly without shipping a standalone index.html.

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
        if case.get("unix_only").and_then(|v| v.as_bool()).unwrap_or(false) && !cfg!(unix) {
            continue;
        }
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

#[cfg(unix)]
fn copy_tree(src: &Path, dst: &Path) {
    std::fs::create_dir_all(dst).unwrap();
    for entry in std::fs::read_dir(src).unwrap() {
        let entry = entry.unwrap();
        let from = entry.path();
        let to = dst.join(entry.file_name());
        if from.is_dir() {
            copy_tree(&from, &to);
        } else {
            std::fs::copy(&from, &to).unwrap();
        }
    }
}

#[cfg(unix)]
fn write_executable(path: &Path, content: &str) {
    use std::os::unix::fs::PermissionsExt;
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path, content).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

#[cfg(unix)]
struct DebFixture {
    _root: tempfile::TempDir,
    _conflicting_target: tempfile::TempDir,
    repo: PathBuf,
    conflicting_target: PathBuf,
    fake_bin: PathBuf,
    sdk_dir: PathBuf,
    vision_bin: PathBuf,
    models_dir: PathBuf,
}

#[cfg(unix)]
fn make_deb_fixture() -> DebFixture {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let root = tempfile::tempdir().unwrap();
    let repo = root.path().join("repo");
    std::fs::create_dir_all(repo.join("scripts")).unwrap();
    std::fs::copy(manifest.join("scripts/build_deb.sh"), repo.join("scripts/build_deb.sh")).unwrap();
    std::fs::copy(manifest.join("LICENSE"), repo.join("LICENSE")).unwrap();
    std::fs::copy(manifest.join("THIRD_PARTY_NOTICES.md"), repo.join("THIRD_PARTY_NOTICES.md")).unwrap();
    std::fs::write(repo.join("Cargo.toml"), "[package]\nname = \"armd\"\nversion = \"0.0.1-test\"\n").unwrap();
    copy_tree(&manifest.join("kits/recamera-b601dm/packaging"), &repo.join("kits/recamera-b601dm/packaging"));
    std::fs::create_dir_all(repo.join("kits/recamera-b601dm/profile")).unwrap();
    std::fs::copy(
        manifest.join("kits/recamera-b601dm/profile/rebot-b601dm.json"),
        repo.join("kits/recamera-b601dm/profile/rebot-b601dm.json"),
    )
    .unwrap();
    std::fs::create_dir_all(repo.join("kits/recamera-b601dm/vision/models")).unwrap();
    std::fs::copy(
        manifest.join("kits/recamera-b601dm/vision/LICENSE-APACHE"),
        repo.join("kits/recamera-b601dm/vision/LICENSE-APACHE"),
    )
    .unwrap();
    // md5("test\n") == d8e8fca2dc0f896fd7cb4cb0031ba249, 5 bytes.
    std::fs::write(
        repo.join("kits/recamera-b601dm/vision/models/models.tsv"),
        "detect\tdummy.cvimodel\td8e8fca2dc0f896fd7cb4cb0031ba249\t5\t0\t-\tbundled\n",
    )
    .unwrap();
    let models_dir = root.path().join("models-cache");
    std::fs::create_dir_all(&models_dir).unwrap();
    std::fs::write(models_dir.join("dummy.cvimodel"), b"test\n").unwrap();
    let sdk_dir = root.path().join("sdk");
    std::fs::create_dir_all(sdk_dir.join("rebot")).unwrap();
    std::fs::write(sdk_dir.join("rebot/__init__.py"), "# stub\n").unwrap();
    let vision_bin = root.path().join("rebot-vision");
    write_executable(&vision_bin, "#!/bin/sh\nexit 0\n");
    let fake_bin = root.path().join("bin");
    // Cargo shim: respects --target-dir over CARGO_TARGET_DIR, mirroring real cargo.
    write_executable(
        &fake_bin.join("cargo"),
        "#!/usr/bin/env bash\nset -euo pipefail\ntarget=\"\"\ntarget_dir=\"${CARGO_TARGET_DIR:-target}\"\nwhile [ $# -gt 0 ]; do\n  case \"$1\" in\n    --target) target=\"$2\"; shift 2 ;;\n    --target=*) target=\"${1#--target=}\"; shift ;;\n    --target-dir) target_dir=\"$2\"; shift 2 ;;\n    --target-dir=*) target_dir=\"${1#--target-dir=}\"; shift ;;\n    *) shift ;;\n  esac\ndone\nout=\"${target_dir}/${target}/release/armd\"\nmkdir -p \"$(dirname \"${out}\")\"\nprintf '#!/bin/sh\\necho fresh-armd-sentinel\\n' > \"${out}\"\nchmod 0755 \"${out}\"\n",
    );
    let conflicting = tempfile::tempdir().unwrap();
    let conflicting_target = conflicting.path().to_path_buf();
    DebFixture { _root: root, _conflicting_target: conflicting, repo, conflicting_target, fake_bin, sdk_dir, vision_bin, models_dir }
}

#[cfg(unix)]
fn run_build_deb(fx: &DebFixture, args: &[&str]) -> std::process::Output {
    let orig_path = std::env::var("PATH").unwrap_or_default();
    Command::new("bash")
        .arg(fx.repo.join("scripts/build_deb.sh"))
        .args(args)
        .env("PATH", format!("{}:{orig_path}", fx.fake_bin.display()))
        .env("CARGO_TARGET_DIR", &fx.conflicting_target)
        .env("RUSTFLAGS", "-C linker=rust-lld")
        .env("REBOT_SDK_DIR", &fx.sdk_dir)
        .env("REBOT_VISION_BIN", &fx.vision_bin)
        .env("REBOT_MODELS_DIR", &fx.models_dir)
        .output()
        .expect("run build_deb.sh")
}

#[cfg(unix)]
#[test]
fn build_deb_packages_armd_without_standalone_index_html() {
    assert!(!api_dir().join("cockpit/assembler/assemble.sh").exists(), "assemble.sh is gone");
    let fx = make_deb_fixture();
    assert!(!fx.repo.join("target").exists(), "clean checkout starts without target/");

    // 1. Dry-run prints explicit --target-dir for both flavors.
    for args in [&["--dry-run"][..], &["--flavor", "lite", "--dry-run"][..]] {
        let out = run_build_deb(&fx, args);
        assert!(out.status.success(), "dry-run {args:?} failed: {}", String::from_utf8_lossy(&out.stderr));
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(stdout.contains("--target-dir"), "dry-run {args:?} missing --target-dir:\n{stdout}");
    }

    // 2. Lite flavor from clean checkout without target/ and with conflicting CARGO_TARGET_DIR.
    let lite_out = run_build_deb(&fx, &["--flavor", "lite", "0.0.1-test"]);
    assert!(
        lite_out.status.success(),
        "lite build failed:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&lite_out.stdout),
        String::from_utf8_lossy(&lite_out.stderr)
    );
    let tar_path = fx.repo.join("dist/armd-lite_0.0.1-test_riscv64.tar.gz");
    let tar_list = Command::new("tar").args(["-tzf"]).arg(&tar_path).output().expect("tar -tzf");
    assert!(tar_list.status.success());
    let entries: Vec<&str> = std::str::from_utf8(&tar_list.stdout).unwrap().lines().collect();
    assert!(entries.contains(&"armd-lite_0.0.1-test_riscv64/armd"), "lite missing armd: {entries:?}");
    assert!(entries.contains(&"armd-lite_0.0.1-test_riscv64/LICENSE"), "lite missing LICENSE: {entries:?}");
    assert!(
        entries.contains(&"armd-lite_0.0.1-test_riscv64/THIRD_PARTY_NOTICES.md"),
        "lite missing THIRD_PARTY_NOTICES.md: {entries:?}"
    );
    assert!(!entries.iter().any(|e| e.contains("index.html")), "lite must not contain index.html: {entries:?}");
    let lite_bin = Command::new("tar")
        .args(["-xOzf"])
        .arg(&tar_path)
        .arg("armd-lite_0.0.1-test_riscv64/armd")
        .output()
        .expect("extract lite armd");
    assert!(String::from_utf8_lossy(&lite_bin.stdout).contains("fresh-armd-sentinel"), "lite armd is not fresh");
    assert!(std::fs::read_dir(&fx.conflicting_target).unwrap().next().is_none(), "CARGO_TARGET_DIR must not be used");

    // 3. Recamera deb flavor from clean checkout without target/ and with conflicting CARGO_TARGET_DIR.
    if Command::new("dpkg-deb").arg("--version").output().is_ok_and(|o| o.status.success()) {
        std::fs::remove_dir_all(fx.repo.join("target")).unwrap();
        assert!(!fx.repo.join("target").exists());
        let deb_out = run_build_deb(&fx, &["--flavor", "recamera", "0.0.1-test"]);
        assert!(
            deb_out.status.success(),
            "recamera deb build failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&deb_out.stdout),
            String::from_utf8_lossy(&deb_out.stderr)
        );
        let deb_path = fx.repo.join("dist/rebot-teleop_0.0.1-test_riscv64.deb");
        let listing = Command::new("dpkg-deb").args(["-c"]).arg(&deb_path).output().expect("dpkg-deb -c");
        assert!(listing.status.success());
        let text = String::from_utf8_lossy(&listing.stdout);
        assert!(text.contains("./usr/bin/armd"), "deb missing ./usr/bin/armd:\n{text}");
        assert!(text.contains("./usr/local/bin/armd -> /usr/bin/armd"), "deb missing symlink:\n{text}");
        assert!(!text.contains("index.html"), "deb must not ship standalone index.html:\n{text}");
        let unpacked = tempfile::tempdir().unwrap();
        let ex = Command::new("dpkg-deb").arg("-x").arg(&deb_path).arg(unpacked.path()).output().expect("dpkg-deb -x");
        assert!(ex.status.success());
        let armd_bytes = std::fs::read_to_string(unpacked.path().join("usr/bin/armd")).unwrap();
        assert!(armd_bytes.contains("fresh-armd-sentinel"), "deb usr/bin/armd is not fresh");
        assert!(!unpacked.path().join("usr/share/rebot-teleop/index.html").exists(), "standalone index.html must not exist");
        assert!(std::fs::read_dir(&fx.conflicting_target).unwrap().next().is_none(), "CARGO_TARGET_DIR must not be used");
    }
}
