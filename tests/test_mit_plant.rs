//! Joint tracking Phase 2: lag + friction plant model of one Damiao joint, POS_VEL against MIT
//! PD, with the section 4 acceptance metrics of the Phase 2 design. A MODEL, not a measurement:
//! the friction mechanism of the observed wrist hold cycle (creep3: J4/J5/J6 4.6/3.1/4.2 mrad)
//! is a hypothesis, and so are the firmware loop structure and gains below.
//!
//! Plant (10 kHz): joint inertia, gravity load, Coulomb + viscous friction with stiction
//! (stick while |net torque| <= Fs), motor torque saturation at TMAX. Firmware (1 kHz, sampled):
//! 14-bit motor encoder through the gear (joint resolution 2π / 16384 / Gr), velocity estimate
//! from encoder differences.
//! - POS_VEL: v_cmd = (54 / Gr)(q_cmd - q_enc) (KP_APR 54 referred to the joint gives the
//!   measured lag Gr / 54: 0.741 s for J1-J3, 0.185 s for J4-J7), clamped to VLIM, then a PI
//!   velocity loop (proportional band `v_bw` Hz, integral corner 5 Hz).
//! - MIT: τ = kp(q_d - q_enc) + kd(v_d - v_est) + τ_ff, no integrator.
//!
//! Host (50 Hz): quintic reference; MIT τ_ff from the supervisor law (gravity + Coulomb ramp,
//! clamped to [-L, L] ∩ [-L - pd, L - pd] ∩ 15 N·m/s slew, dynamic terms 0 in a hold);
//! measured position through the 16-bit wire quantization (±12.5 rad).
//!
//! Metrics (design section 4): hold = max |Δq| over host sample pairs at most 1 s apart within
//! 2.0-4.5 s after the move (gate <= 1.0 mrad); moving RMS nominal - measured (MIT <= 50 % of
//! the matched POS_VEL run); endpoint |error| at +1 s (<= 2 mrad).

#[derive(Clone, Copy, Debug)]
struct Plant {
    gr: f64,
    inertia: f64,
    coulomb: f64,
    stiction: f64,
    viscous: f64,
    gravity: f64,
    /// POS_VEL velocity-loop proportional bandwidth (Hz).
    v_bw: f64,
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    PosVel,
    /// kp, kd (also the hold gains), Coulomb compensation (N·m), gravity model error (fraction).
    Mit { kp: f64, kd: f64, coulomb: f64, gravity_error: f64 },
}

#[derive(Clone, Copy, Debug)]
struct Trial { start: f64, distance: f64, peak_v: f64 }

struct Run { duration: f64, samples: Vec<(f64, f64, f64)> }

fn quintic(t: f64, dur: f64, d: f64) -> (f64, f64, f64) {
    if t <= 0.0 { return (0.0, 0.0, 0.0); }
    if t >= dur { return (d, 0.0, 0.0); }
    let s = t / dur;
    (d * (10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5)),
     d * (30.0 * s * s - 60.0 * s.powi(3) + 30.0 * s.powi(4)) / dur,
     d * (60.0 * s - 180.0 * s * s + 120.0 * s.powi(3)) / (dur * dur))
}

fn wire(q: f64) -> f64 {
    let raw = armd::protocol::damiao::float_to_uint(q as f32, -12.5, 12.5, 16);
    f64::from(armd::protocol::damiao::uint_to_float(raw, -12.5, 12.5, 16))
}

fn simulate(p: &Plant, mode: Mode, tr: &Trial, hold_s: f64) -> Run {
    const DT: f64 = 1e-4;
    const CTRL: usize = 10; // 1 kHz firmware
    const HOST: usize = 200; // 50 Hz host
    const TMAX: f64 = 10.0;
    const L: f64 = 3.0;
    let duration = 15.0 / 8.0 * tr.distance.abs() / tr.peak_v;
    let enc = 2.0 * std::f64::consts::PI / 16384.0 / p.gr;
    let kpos = 54.0 / p.gr;
    let kv = p.inertia * 2.0 * std::f64::consts::PI * p.v_bw;
    let ki = kv * 2.0 * std::f64::consts::PI * 5.0;
    let (mut q, mut v) = (tr.start, 0.0_f64);
    let (mut integ, mut v_est, mut q_enc_prev, mut tau_cmd, mut last_ff) = (0.0_f64, 0.0_f64, tr.start, 0.0_f64, 0.0_f64);
    let mut cmd = (tr.start, 0.0, 0.0, 0.0, 0.0);
    let mut samples = Vec::new();
    let n = ((duration + hold_s) / DT) as usize;
    for k in 0..n {
        let t = k as f64 * DT;
        if k % HOST == 0 {
            let (dq, vd, ad) = quintic(t, duration, tr.distance);
            let qd = tr.start + dq;
            let qm = wire(q);
            samples.push((t, qd, qm));
            cmd = match mode {
                Mode::PosVel => (qd, 0.0, 0.0, 0.0, 0.0),
                Mode::Mit { kp, kd, coulomb, gravity_error } => {
                    let hold = vd == 0.0 && ad == 0.0;
                    let dynamic = if hold { 0.0 } else { coulomb * (vd / 0.02).clamp(-1.0, 1.0) };
                    let raw = p.gravity * (1.0 + gravity_error) + dynamic;
                    let pd = kp * (qd - qm) + kd * (vd - v_est);
                    let lo = (-L).max(-L - pd).max(last_ff - 15.0 * 0.02);
                    let hi = L.min(L - pd).min(last_ff + 15.0 * 0.02);
                    let ff = raw.clamp(lo, hi);
                    last_ff = ff;
                    (qd, vd, kp, kd, ff)
                }
            };
        }
        if k % CTRL == 0 {
            let q_enc = (q / enc).round() * enc;
            v_est = (q_enc - q_enc_prev) / (DT * CTRL as f64);
            q_enc_prev = q_enc;
            tau_cmd = match mode {
                Mode::PosVel => {
                    let err = (kpos * (cmd.0 - q_enc)).clamp(-2.0, 2.0) - v_est;
                    integ = (integ + ki * err * DT * CTRL as f64).clamp(-TMAX, TMAX);
                    (kv * err + integ).clamp(-TMAX, TMAX)
                }
                Mode::Mit { .. } => (cmd.2 * (cmd.0 - q_enc) + cmd.3 * (cmd.1 - v_est) + cmd.4).clamp(-TMAX, TMAX),
            };
        }
        let net = tau_cmd - p.gravity;
        let friction = if v.abs() < 1e-6 {
            if net.abs() <= p.stiction { v = 0.0; continue; }
            p.coulomb * net.signum()
        } else {
            p.coulomb * v.signum() + p.viscous * v
        };
        let mut v_new = v + (net - friction) / p.inertia * DT;
        if v != 0.0 && v_new * v < 0.0 { v_new = 0.0; }
        q += 0.5 * (v + v_new) * DT;
        v = v_new;
    }
    Run { duration, samples }
}

/// (moving RMS, endpoint error at +1 s, hold metric), rad.
fn metrics(r: &Run) -> (f64, f64, f64) {
    let moving: Vec<f64> = r.samples.iter().filter(|s| s.0 > 0.0 && s.0 < r.duration).map(|s| s.1 - s.2).collect();
    assert!(moving.len() > 20, "too few moving samples");
    let rms = (moving.iter().map(|e| e * e).sum::<f64>() / moving.len() as f64).sqrt();
    let end = r.samples.iter().find(|s| (s.0 - (r.duration + 1.0)).abs() < 0.011).map(|s| (s.1 - s.2).abs()).expect("sample at +1 s");
    let window: Vec<(f64, f64)> = r.samples.iter().filter(|s| s.0 >= r.duration + 2.0 && s.0 <= r.duration + 4.5).map(|s| (s.0, s.2)).collect();
    assert!(window.len() >= 120, "incomplete hold telemetry: {} samples", window.len());
    let mut hold = 0.0_f64;
    for (i, a) in window.iter().enumerate() {
        for b in &window[i + 1..] {
            if b.0 - a.0 <= 1.0 + 1e-9 { hold = hold.max((b.1 - a.1).abs()); }
        }
    }
    (rms, end, hold)
}

/// Seven hold poses (gravity loads, start positions) × two repetitions (both directions).
fn hold_trials() -> Vec<(f64, Trial)> {
    let loads = [0.25, 0.15, 0.05, 0.0, -0.05, -0.15, -0.25];
    let mut v = Vec::new();
    for (k, g) in loads.iter().enumerate() {
        for rep in 0..2 {
            v.push((*g, Trial { start: 0.3 + 1e-4 * (7 * k + 3 * rep) as f64, distance: if rep == 0 { 0.1 } else { -0.1 }, peak_v: 0.06 }));
        }
    }
    v
}

const WRIST_GR: f64 = 10.0;

fn wrist(coulomb: f64, stiction: f64, gravity: f64, v_bw: f64) -> Plant {
    Plant { gr: WRIST_GR, inertia: 0.006, coulomb, stiction, viscous: 0.02, gravity, v_bw }
}

/// J5 hold / tracking gains at the hand-guide ceilings (kp 10, kd 0.3), Coulomb compensation
/// identified on the development plant (0.015 N·m), exact gravity model.
const MIT_J5: Mode = Mode::Mit { kp: 10.0, kd: 0.3, coulomb: 0.015, gravity_error: 0.0 };

#[test]
fn mit_lag_and_wrist_hold() {
    // 1. POS_VEL lag Gr / 54 (ramp following error / speed), J1-J3 and wrist
    for (gr, tau) in [(40.0, 40.0 / 54.0), (10.0, 10.0 / 54.0)] {
        let p = Plant { gr, inertia: 0.006 * gr / 10.0, coulomb: 0.0, stiction: 0.0, viscous: 0.0, gravity: 0.0, v_bw: 40.0 };
        let r = simulate(&p, Mode::PosVel, &Trial { start: 0.0, distance: 0.3, peak_v: 0.06 }, 3.0);
        let mid: Vec<&(f64, f64, f64)> = r.samples.iter().filter(|s| (s.0 - r.duration / 2.0).abs() < 0.011).collect();
        let (_, v_mid, _) = quintic(mid[0].0, r.duration, 0.3);
        let lag = (mid[0].1 - mid[0].2) / v_mid;
        eprintln!("POS_VEL lag Gr {gr}: {lag:.3} s (Gr/54 = {tau:.3} s)");
        assert!((lag - tau).abs() / tau < 0.10, "lag {lag} vs {tau}");
    }

    // 2. development plant: the baseline wrist cycle above 1 mrad on every pose and repetition
    let dev = |g: f64| wrist(0.015, 0.045, g, 12.0);
    let mut base = Vec::new();
    for (g, tr) in hold_trials() {
        let h = metrics(&simulate(&dev(g), Mode::PosVel, &tr, 5.0)).2;
        assert!(h > 1.0e-3, "POS_VEL baseline cycle {:.2} mrad <= 1 mrad (g {g}, {tr:?})", h * 1e3);
        base.push(h);
    }
    eprintln!("dev plant POS_VEL hold (mrad): min {:.2} max {:.2}", base.iter().cloned().fold(f64::MAX, f64::min) * 1e3,
        base.iter().cloned().fold(0.0, f64::max) * 1e3);

    // 3. held-out plants (friction / stiction not used to choose the gains): section 4 gates
    let mut worst = (0.0_f64, 0.0_f64, 0.0_f64, f64::MAX);
    for (coulomb, ratio) in [(0.012, 2.5), (0.012, 3.0), (0.016, 2.5), (0.016, 3.0)] {
        let plant = |g: f64| wrist(coulomb, coulomb * ratio, g, 12.0);
        for (g, tr) in hold_trials() {
            let base_h = metrics(&simulate(&plant(g), Mode::PosVel, &tr, 5.0)).2;
            let mit_h = metrics(&simulate(&plant(g), MIT_J5, &tr, 5.0)).2;
            assert!(mit_h <= 1.0e-3, "MIT hold {:.2} mrad > 1 mrad (Fc {coulomb}, g {g}, {tr:?})", mit_h * 1e3);
            worst.2 = worst.2.max(mit_h);
            worst.3 = worst.3.min(base_h);
        }
        for peak_v in [0.03, 0.06, 0.10] {
            for distance in [0.1, -0.1] {
                for g in [0.25, -0.05] {
                    let tr = Trial { start: 0.0, distance, peak_v };
                    let (rms_p, _, _) = metrics(&simulate(&plant(g), Mode::PosVel, &tr, 5.0));
                    let (rms_m, end_m, _) = metrics(&simulate(&plant(g), MIT_J5, &tr, 5.0));
                    let ratio_m = rms_m / rms_p;
                    assert!(ratio_m <= 0.5, "MIT moving RMS {:.2} mrad is {:.0} % of POS_VEL {:.2} mrad ({tr:?}, Fc {coulomb})",
                        rms_m * 1e3, ratio_m * 100.0, rms_p * 1e3);
                    assert!(end_m <= 2.0e-3, "MIT endpoint error {:.2} mrad > 2 mrad ({tr:?}, Fc {coulomb})", end_m * 1e3);
                    worst.0 = worst.0.max(ratio_m);
                    worst.1 = worst.1.max(end_m);
                }
            }
        }
    }
    eprintln!("held-out MIT: worst RMS ratio {:.2}, worst endpoint {:.2} mrad, worst hold {:.2} mrad; POS_VEL hold min {:.2} mrad",
        worst.0, worst.1 * 1e3, worst.2 * 1e3, worst.3 * 1e3);
}

/// The same model with a 40 Hz velocity loop (the V_BW register read as Hz): a >1 mrad POS_VEL
/// cycle then needs Coulomb 0.08 / stiction 0.20 N·m, and with that friction the integral-free
/// MIT PD (kp 10) still holds without a cycle but misses the 2 mrad endpoint gate and the 50 %
/// tracking gate: the static
/// error is bounded by (friction + gravity-model error) / kp, not by the controller. Pinned here
/// so a model change that alters the conclusion is visible; the device run decides.
#[test]
fn mit_plant_model_limitation_high_friction() {
    let plant = |g: f64| wrist(0.08, 0.20, g, 40.0);
    let tr = Trial { start: 0.3, distance: 0.1, peak_v: 0.06 };
    let base = metrics(&simulate(&plant(0.15), Mode::PosVel, &tr, 5.0));
    let mit = metrics(&simulate(&plant(0.15), MIT_J5, &tr, 5.0));
    eprintln!("high-friction plant: POS_VEL hold {:.2} mrad rms {:.2}; MIT hold {:.2} mrad rms {:.2} endpoint {:.2} mrad",
        base.2 * 1e3, base.0 * 1e3, mit.2 * 1e3, mit.0 * 1e3, mit.1 * 1e3);
    assert!(base.2 > 1.0e-3);
    assert!(mit.2 <= 1.0e-3);
    assert!(mit.1 > 2.0e-3, "the model predicts the endpoint gate fails at this friction");
    assert!(mit.0 / base.0 > 0.5, "and the tracking RMS gate");
    // a 0.02 N·m gravity-model error (either sign) shifts the static balance by about
    // 0.02 / kp = 2 mrad; one sign adds to the friction error
    let lo = wrist(0.012, 0.03, 0.15, 12.0);
    let exact = metrics(&simulate(&lo, MIT_J5, &tr, 5.0)).1;
    let with_error = |e: f64| metrics(&simulate(&lo, Mode::Mit { kp: 10.0, kd: 0.3, coulomb: 0.015, gravity_error: e / 0.15 }, &tr, 5.0)).1;
    let (plus, minus) = (with_error(0.02), with_error(-0.02));
    eprintln!("gravity model error ±0.02 N·m: endpoint {:.2} mrad exact, {:.2} / {:.2} mrad", exact * 1e3, plus * 1e3, minus * 1e3);
    assert!(plus.max(minus) > exact + 1.0e-3, "a 0.02 N·m model error moves the endpoint by more than 1 mrad");
}
