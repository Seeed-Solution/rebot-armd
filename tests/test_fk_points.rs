//! Forward-kinematics reference points shared with the cockpit 3D view check
//! (scripts/view3d_shots.mjs compares the rendered end_link with these within 5 mm).
//! Regenerate the fixture with `FK_FIXTURE_REGEN=1 cargo test --test test_fk_points`.
use armd::kinematics::fk;

const Q: [[f64; 6]; 5] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, -1.167, -1.565, 1.435, 0.0, 1.5],
    [0.6, -1.0, -1.4, 1.3, 0.0, 1.57],
    [-1.2, -0.5, -2.2, 0.4, 0.8, -1.0],
    [2.0, -2.5, -0.6, -1.0, -1.2, 2.5],
];
const PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/fk_points.json");

#[test]
fn fk_fixture_matches_kinematics() {
    let points: Vec<serde_json::Value> = Q.iter().map(|q| {
        let p = fk(q).p;
        serde_json::json!({ "q": q, "end_m": p.map(|v| (v * 1e6).round() / 1e6) })
    }).collect();
    if std::env::var_os("FK_FIXTURE_REGEN").is_some() {
        let doc = serde_json::json!({ "frame": "base_link -> end_link, URDF angles J1..J6 (rad), metres", "points": points });
        std::fs::write(PATH, serde_json::to_string_pretty(&doc).unwrap() + "\n").unwrap();
    }
    let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(PATH).unwrap()).unwrap();
    let stored = doc["points"].as_array().unwrap();
    assert_eq!(stored.len(), Q.len());
    for (s, q) in stored.iter().zip(Q.iter()) {
        let p = fk(q).p;
        for k in 0..3 {
            let v = s["end_m"][k].as_f64().unwrap();
            assert!((v - p[k]).abs() < 1e-5, "q {q:?} axis {k}: fixture {v} vs fk {}", p[k]);
        }
    }
}
