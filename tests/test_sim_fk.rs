//! The MuJoCo scene used by `--follower sim` (sim/mujoco/models/rebot_b601dm_scene.xml)
//! must have the same forward kinematics as armd. The fixture is produced by
//! `uv run rebot-sim-fk` in sim/mujoco; sim/mujoco/tests/test_fk_fixture.py checks it
//! is current.
use armd::kinematics::{angle_between, fk};

#[test]
fn mujoco_scene_fk_matches_armd_kinematics() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/mujoco_fk_samples.json")).unwrap();
    let fixture: serde_json::Value = serde_json::from_str(&text).unwrap();
    let samples = fixture["samples"].as_array().unwrap();
    assert!(samples.len() >= 5);
    let mut worst_mm: f64 = 0.0;
    let mut worst_deg: f64 = 0.0;
    for s in samples {
        let q: Vec<f64> = s["q"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        let p: Vec<f64> = s["p"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        let mut r = [[0.0; 3]; 3];
        for (i, row) in s["r"].as_array().unwrap().iter().enumerate() {
            for (j, v) in row.as_array().unwrap().iter().enumerate() { r[i][j] = v.as_f64().unwrap(); }
        }
        let pose = fk(&q.clone().try_into().unwrap());
        let err_mm = ((pose.p[0] - p[0]).powi(2) + (pose.p[1] - p[1]).powi(2) + (pose.p[2] - p[2]).powi(2)).sqrt() * 1000.0;
        let err_deg = angle_between(&pose.r, &r).to_degrees();
        println!("q={q:?} armd p={:?} mujoco p={p:?} |dp|={err_mm:.4} mm |dR|={err_deg:.4} deg", pose.p);
        assert!(err_mm < 2.0, "position differs by {err_mm:.3} mm at q={q:?}");
        assert!(err_deg < 0.5, "orientation differs by {err_deg:.3} deg at q={q:?}");
        worst_mm = worst_mm.max(err_mm);
        worst_deg = worst_deg.max(err_deg);
    }
    println!("worst: {worst_mm:.4} mm, {worst_deg:.4} deg over {} samples", samples.len());
}
