//! Read-only static files for the cockpit 3D view: `GET /assets/model/{*path}` serves the URDF and
//! the decimated meshes from the kit asset directory (not embedded in the binary), and
//! `GET /assets/three-urdf.min.js` serves the embedded three.js + urdf-loader bundle.
use std::path::{Path, PathBuf};

use axum::{
    body::Body,
    extract::Path as UrlPath,
    http::{header, StatusCode},
    response::{IntoResponse, Response},
};

/// Directories searched for the model, in order: the same locations as the URDF check in
/// `SupervisorInner::new` (`assets/`, then the kit's `/usr/share/rebot-teleop/assets/`).
/// `ARMD_ASSET_DIR` replaces the list.
pub const ASSET_DIRS: [&str; 2] = ["assets", "/usr/share/rebot-teleop/assets"];
const URDF_NAME: &str = "reBot-DevArm_gripper.urdf";
const MAX_FILE_BYTES: u64 = 8 * 1024 * 1024;

/// The first directory that holds the URDF, or `ARMD_ASSET_DIR` when set.
pub fn asset_dir() -> Option<PathBuf> {
    if let Some(d) = std::env::var_os("ARMD_ASSET_DIR") {
        return Some(PathBuf::from(d));
    }
    ASSET_DIRS.iter().map(PathBuf::from).find(|d| d.join(URDF_NAME).is_file())
}

fn content_type(path: &str) -> Option<&'static str> {
    let ext = path.rsplit_once('.')?.1.to_ascii_lowercase();
    Some(match ext.as_str() {
        "urdf" => "application/xml; charset=utf-8",
        "stl" => "model/stl",
        "json" => "application/json",
        "md" | "txt" => "text/plain; charset=utf-8",
        _ => return None,
    })
}

/// Resolve a request path under `dir`: segments of `[A-Za-z0-9._-]`, none starting with '.', at
/// most 4 deep, a known file type, and (after resolving symlinks) still inside `dir`.
pub fn resolve(dir: &Path, rel: &str) -> Option<(PathBuf, &'static str)> {
    let segs: Vec<&str> = rel.split('/').collect();
    if rel.is_empty() || segs.len() > 4 {
        return None;
    }
    let ok = |s: &str| !s.is_empty() && !s.starts_with('.') && s.len() <= 128
        && s.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'));
    if !segs.iter().all(|s| ok(s)) {
        return None;
    }
    let ct = content_type(rel)?;
    let root = dir.canonicalize().ok()?;
    let file = root.join(rel).canonicalize().ok()?;
    (file.starts_with(&root) && file.is_file()).then_some((file, ct))
}

pub async fn serve_model(UrlPath(rel): UrlPath<String>) -> Response {
    match asset_dir() {
        Some(dir) => serve_from(&dir, &rel).await,
        None => (StatusCode::NOT_FOUND, "not found").into_response(),
    }
}

async fn serve_from(dir: &Path, rel: &str) -> Response {
    let not_found = || (StatusCode::NOT_FOUND, "not found").into_response();
    let Some((file, ct)) = resolve(dir, rel) else { return not_found() };
    match tokio::fs::metadata(&file).await {
        Ok(m) if m.len() <= MAX_FILE_BYTES => {}
        _ => return not_found(),
    }
    match tokio::fs::read(&file).await {
        Ok(bytes) => (
            [(header::CONTENT_TYPE, ct), (header::CACHE_CONTROL, "public, max-age=86400")],
            Body::from(bytes),
        )
            .into_response(),
        Err(_) => not_found(),
    }
}

// three.js 0.186.1 (MIT) + urdf-loader 0.13.1 (Apache-2.0), bundled and gzipped by
// scripts/vendor_three.mjs; fetched only when the 3D view is opened.
static THREE_URDF_GZ: &[u8] = include_bytes!("vendor/three-urdf.min.js.gz");

pub async fn serve_three() -> impl IntoResponse {
    (
        [
            (header::CONTENT_TYPE, "application/javascript; charset=utf-8"),
            (header::CONTENT_ENCODING, "gzip"),
            (header::CACHE_CONTROL, "public, max-age=31536000, immutable"),
        ],
        THREE_URDF_GZ,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> tempfile::TempDir {
        let t = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(t.path().join("assets/meshes")).unwrap();
        std::fs::write(t.path().join("assets").join(URDF_NAME), "<robot/>").unwrap();
        std::fs::write(t.path().join("assets/meshes/link1.stl"), "solid").unwrap();
        std::fs::write(t.path().join("assets/meshes/manifest.json"), "{}").unwrap();
        std::fs::write(t.path().join("assets/meshes/run.sh"), "x").unwrap();
        std::fs::write(t.path().join("secret.json"), "{}").unwrap();
        t
    }

    #[test]
    fn serves_known_files_under_the_dir() {
        let t = tree();
        let d = t.path().join("assets");
        assert_eq!(resolve(&d, URDF_NAME).unwrap().1, "application/xml; charset=utf-8");
        assert_eq!(resolve(&d, "meshes/link1.stl").unwrap().1, "model/stl");
        assert_eq!(resolve(&d, "meshes/manifest.json").unwrap().1, "application/json");
    }

    #[test]
    fn rejects_traversal_hidden_unknown_and_missing() {
        let t = tree();
        let d = t.path().join("assets");
        for bad in ["../secret.json", "meshes/../../secret.json", "..", "/etc/passwd", "meshes//link1.stl",
            ".hidden.json", "meshes/run.sh", "meshes/none.stl", "", "meshes", "a/b/c/d/e.stl", "meshes\\link1.stl",
            "meshes/link1.stl%00"] {
            assert!(resolve(&d, bad).is_none(), "{bad} must be rejected");
        }
    }

    #[cfg(unix)]
    #[test]
    fn rejects_symlink_out_of_the_dir() {
        let t = tree();
        let d = t.path().join("assets");
        std::os::unix::fs::symlink(t.path().join("secret.json"), d.join("meshes/link.json")).unwrap();
        assert!(resolve(&d, "meshes/link.json").is_none());
    }

    #[tokio::test]
    async fn route_serves_and_rejects() {
        use tower::ServiceExt;
        let t = tree();
        let d = t.path().join("assets");
        let app = axum::Router::new()
            .route("/assets/model/{*path}", axum::routing::get(move |UrlPath(rel): UrlPath<String>| async move { serve_from(&d, &rel).await }))
            .route("/assets/three-urdf.min.js", axum::routing::get(serve_three));
        let get = |u: &str| axum::http::Request::get(u).body(Body::empty()).unwrap();
        let r = app.clone().oneshot(get("/assets/model/meshes/link1.stl")).await.unwrap();
        assert_eq!(r.status(), StatusCode::OK);
        assert_eq!(r.headers()[header::CONTENT_TYPE], "model/stl");
        for bad in ["/assets/model/../secret.json", "/assets/model/meshes/%2e%2e/%2e%2e/secret.json", "/assets/model/meshes/run.sh"] {
            let r = app.clone().oneshot(get(bad)).await.unwrap();
            assert_eq!(r.status(), StatusCode::NOT_FOUND, "{bad}");
        }
        let r = app.oneshot(get("/assets/three-urdf.min.js")).await.unwrap();
        assert_eq!(r.headers()[header::CONTENT_ENCODING], "gzip");
        let body = axum::body::to_bytes(r.into_body(), usize::MAX).await.unwrap();
        assert_eq!(&body[..2], &[0x1f, 0x8b]);
    }
}
