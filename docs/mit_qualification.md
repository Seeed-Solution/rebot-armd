# MIT tracking qualification (B601-DM)

MIT impedance tracking (joint tracking Phase 2, `src/mit.rs`) is disabled by default and is
refused at arm time until the joint is qualified on the device. This page lists what armd checks,
what the operator measures, and the record that ties the two together. Nothing here has been run
on the arm yet; every value marked "needs device verification" is an open item.

## What armd enforces

| Check | Where | Result when it fails |
|---|---|---|
| A joint uses `"strategy": "mit"` with `hold_kp` / `hold_kd`, positive `kp`, `kd`, `effort_limit_nm`; gains at or below the profile's `gravity_comp.hold_kp` / `hold_kd`; `coulomb_nm`, `viscous_nm_s_rad · max_velocity`, `inertia_kg_m2 · max_acceleration` each within `effort_limit_nm` | profile load | enabled block rejected; a disabled block is ignored as in Phase 1 |
| `calibration.json` `mit_qualification` holds the joint, bound to profile, joint name, joint zero, motor ID, reply ID, ESC_ID, MST_ID, sw_ver, PMAX / VMAX / TMAX, TIMEOUT; not invalidated by a zero; `timeout_unit_verified` true; `measured_expiry_ms` > 0 and `tested_response` given; control period + `worst_latency_ms` + 5 ms < 45 ms | every arm and rearm, against a fresh register readback | the arm does not arm (`MIT_UNQUALIFIED`), no enable frame is sent |
| TIMEOUT readback nonzero and equal to the record | same | same |
| Profile gains and effort limit at or below the record's `max_kp`, `max_kd`, `effort_ceiling_nm` (and TMAX) | same | same |
| Handover only from fresh (< 45 ms), stationary, confirmed position control; mode ACK, hold transmit, flush and feedback newer than the ACK | first tick at rest after arming | MIT stays inactive, motions are refused; an unconfirmed rollback latches a fault and blocks clearance |
| Feedback age or gap between dynamic batches ≥ 45 ms, RX backlog, partial or failed batch | every tick (supervisor) and before every transmit (driver) | group cancelled, BUS_LOSS latched wherever the loss is detected (also over another fault), no further frame; read-only status polls only after the operator's fault clearance and 2 × `measured_expiry_ms` (at least 250 ms); reconnect needs a rearm (fresh readback, base modes restored); ages or gaps from 36 ms on are counted and logged as near misses |

TIMEOUT is RAM only. A power cycle resets it to 0 and the next arm refuses until it is
commissioned again.

## Recovery after a motor command timeout

When MIT tracking is lost and a motor's own TIMEOUT then expires, that motor disables itself and
reports error 0xD (CAN communication loss) while the other motors stay enabled. In that state
`/api/v1/safety/clear_fault` (operator session) recovers under control: it sends the Damiao
clear-error command (`FF FF FF FF FF FF FF FB` on the motor's CAN ID) to every motor in an error
state, disables every motor and requires fresh state-0 feedback from each, and only then clears to
Idle. **Torque is then off on every joint**, like a park torque-off: the arm drops onto whatever
supports it. No MIT frame and no POS_VEL target is sent. A motor that does not confirm keeps the
fault latched and is named in the reply. The MIT group is reset; the qualification record is kept
and the next arm reads the TIMEOUT again.

Before an expiry test (step 1 of the device qualification), put the arm at a supported or rest
pose: the recovery turns torque off on all joints.

## Commissioning the motor command timeout

Operator session, torque off, load mechanically supported, operator at the power cutoff.

1. Read only: WS `/api/v1/calibration/mit_timeout` `{"joints": [4], "dry_run": true}`. Record the
   readback (IDs, sw_ver, CTRL_MODE, ranges, original TIMEOUT; 0 on the B601-DM on 2026-11-22).
2. Write: `{"joints": [4], "timeout_counts": 2000}`. armd writes RID 9 with the RAM opcode 0x55 (never
   0xAA), checks the acknowledgement value and reply ID, and confirms with a separate 0x33 read.
3. The count is configurable (1..20000). 2000 counts = 100 ms only if the documented unit of
   50 µs per count holds on this firmware; this needs device verification (step 2 below).

## Device qualification (one joint at a time)

Workspace clear, load supported, operator at the power cutoff. J4 alone first, then J1, then each
later joint individually. All register writes are RAM only.

1. **Timeout response.** On an isolated fixture, observe TX-only loss (CAN TX cut), RX-only loss
   and host loss (armd killed). Record for each: time from the last command to the motor response,
   the response (`disabled`, `energized_hold`, ...), torque, motion and restart behaviour, and which
   frames refresh the timer (MIT frame, POS_VEL frame, 0xCC status poll, register read). The
   measured expiry decides whether 50 µs per count is right (`timeout_unit_verified`).
2. **Load envelope.** Repeat the timeout test at the heaviest payload and the posture with the
   largest gravity load of the joint. A supported bench test alone does not show an acceptable
   unsupported drop; production needs a safe timeout response across the whole envelope.
3. **Comparison runs.** Identical bidirectional excursions with POS_VEL, `position_lead` and MIT at
   0.03, 0.06 and 0.10 rad/s, then a hold. Record nominal and wire output, measured state and
   feedback ages, clipping, current and temperature, and the worst-case loop latency
   (`worst_latency_ms`: scheduling, RX queue and TX to the wire; SocketCAN `flush` does not bound
   wire time, so measure it on the bus).
4. **Gates** (design section 4, the same metrics as `tests/test_mit_plant.rs`):
   - hold: within 2.0-4.5 s after each move, the maximum |Δq| over sample pairs at most 1 s apart
     ≤ 1.0 mrad for every creep3 pose and both repetitions; incomplete telemetry is a failure;
   - tracking: moving RMS nominal-vs-measured ≤ 50 % of the matched POS_VEL run;
   - endpoint: |error| ≤ 2 mrad at +1 s.
5. **Abort** on unexpected displacement, growing oscillation, clipping, a guard trip, a timing
   miss, an uncertain mode, or current / temperature beyond the limits. A failed gate leaves the
   joint disabled.

## Storing the record

WS `/api/v1/calibration/mit_qualification` `{"record": {...}}` (operator, torque off). armd reads
the motors again and stores the record only when it matches them; the file is written atomically,
then published. Fields per joint: `profile_id`, `joint`, `joint_name`, `zero_fingerprint`
(`zero:<hex bits of the joint zero offset>`), `motor_id`, `reply_id`, `esc_id`, `mst_id`, `sw_ver`,
`pmax`, `vmax`, `tmax`, `timeout_raw`, `timeout_unit_us`, `timeout_unit_verified`, `tested_response`,
`measured_expiry_ms`, `worst_latency_ms` (measured worst-case scheduling + RX queue + TX latency of
the control loop under load; step 3 records it), `load_envelope`, `max_kp`, `max_kd`,
`effort_ceiling_nm`, `date`, `trace_id`. A malformed or invalid record is ignored and flagged at load
(the rest of `calibration.json` is used) and refuses MIT. A zero of a qualified joint writes the
invalidation to disk before the zero; if that write fails, the zero is refused.
A motor or software zero of the joint marks the record invalidated; requalify after it.

## What the plant model says (not a measurement)

`tests/test_mit_plant.rs` models one wrist joint (gear 10, POS_VEL lag Gr/54, PI velocity loop,
14-bit encoder, stiction, Coulomb and viscous friction) and compares POS_VEL with MIT PD at the
J5 ceilings (kp 10, kd 0.3). Results depend on the assumed velocity-loop bandwidth and friction:

| Assumption | POS_VEL hold cycle | MIT hold | MIT RMS / POS_VEL | MIT endpoint |
|---|---|---|---|---|
| velocity loop 12 Hz, Coulomb 0.015 / stiction 0.045 N·m (development plant) | 1.91-2.29 mrad on all 14 trials | - | - | - |
| same loop, held out: Coulomb 0.012 / 0.016 N·m, stiction 2.5 / 3 × | ≥ 1.14 mrad | 0.00 mrad | ≤ 0.28 | ≤ 1.77 mrad |
| velocity loop 40 Hz, Coulomb 0.08 / stiction 0.20 N·m | 1.91 mrad | 0.00 mrad | 0.98 (fails) | 8.03 mrad (fails) |

The POS_VEL lag of the model is 0.684 s (gear 40) and 0.194 s (gear 10) against Gr/54 =
0.741 / 0.185 s; the creep2 fits were 0.68 (J1) and 0.17 (J4). A ±0.02 N·m gravity-model error
moves the held-out MIT endpoint from 1.17 mrad to 0.74 / 3.08 mrad.

Without an integrator the MIT endpoint error is bounded by (static friction + gravity-model
error) / kp: at kp 10, 0.02 N·m of either is 2 mrad. The device runs in step 3 decide whether the
endpoint gate holds; the model does not.
