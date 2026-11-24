//! Versioned configuration (armd 1.6 RT core, track B, contracts v2): reserve at the
//! candidate's revision, persist, apply + activate, release on failure, `Reconciling` after a
//! failure past the rename and at start. Each outcome asserts the file on disk, the active
//! revision and motion eligibility (no hold left that inhibits motion; a joint motion is
//! admitted afterwards). RT-side tests use one monotonic base instant or the fixture's
//! virtual clock, never sleeps.

mod common;

use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};

use armd::model::KeepoutBox;
use armd::rt_core::broker::{BootId, Capacities, ClientId, Submit};
use armd::rt_core::config_iface::{Activated, ConfigAuthority, ConfigDomain, Durability, HoldState, Released, Reserved};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture};
use armd::supervisor::ArmError;
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use common::INSPECT;

/// The WS tests set the process-wide stage delay: run them one at a time.
static WRITE_DELAY: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

fn cube(name: &str, c: f32) -> Value { json!({"name": name, "min_m": [c, c, c], "max_m": [c + 0.05, c + 0.05, c + 0.05]}) }

fn boxed(name: &str, c: f32) -> KeepoutBox { KeepoutBox { name: name.into(), min_m: [c; 3], max_m: [c + 0.05; 3] } }

fn files(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().file_name().to_string_lossy().into_owned()).collect();
    v.sort();
    v
}

fn disk_boxes(path: &Path) -> Option<Vec<String>> {
    let text = std::fs::read_to_string(path).ok()?;
    let v: Value = serde_json::from_str(&text).unwrap();
    Some(v["boxes"].as_array().unwrap().iter().map(|b| b["name"].as_str().unwrap().to_string()).collect())
}

fn runtime_boxes(inner: &SupervisorInner) -> Vec<String> { inner.keepout.runtime_boxes.iter().map(|b| b.name.clone()).collect() }

fn state_of(inner: &SupervisorInner) -> Option<HoldState> { inner.core.config_hold().map(|h| h.state) }

// -- the RT side of the contract (production adapter) ------------------------------------

#[test]
fn reserve_refuses_stale_expired_stopped_foreign_and_competing_proposals() {
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let t0 = Instant::now();
    let ctx = inner.config_ctx(t0);
    assert_eq!(inner.revision(ConfigDomain::Keepout), 1);

    let p = inner.config_proposal(ConfigDomain::Keepout, 1, 0, &ctx);
    let late = inner.config_ctx(p.stamp.deadline + Duration::from_millis(1));
    assert_eq!(inner.reserve(&p, &late), Reserved::Refused { code: "DEADLINE_EXPIRED", current: 1 });
    let stopped = armd::rt_core::ops::CommitCtx { stop_generation: 1, ..ctx };
    assert_eq!(inner.reserve(&p, &stopped), Reserved::Refused { code: "STOPPED", current: 1 });
    let mut q = p;
    q.stamp.authority.epoch += 1;
    assert_eq!(inner.reserve(&q, &ctx), Reserved::Refused { code: "EPOCH_MISMATCH", current: 1 });
    let q = inner.config_proposal(ConfigDomain::Keepout, 1, 77, &ctx);
    assert_eq!(inner.reserve(&q, &ctx), Reserved::Refused { code: "LEASE_LOST", current: 1 });
    let q = inner.config_proposal(ConfigDomain::Keepout, 0, 0, &ctx);
    assert_eq!(inner.reserve(&q, &ctx), Reserved::Refused { code: "REVISION_MISMATCH", current: 1 });
    assert!(inner.core.config_hold().is_none() && !inner.core.motion_inhibited_by_config(), "refusals hold nothing");

    // a granted reservation inhibits motion; a competing one (any domain) is busy
    let Reserved::Granted(r) = inner.reserve(&p, &ctx) else { panic!("reserve") };
    assert_eq!((r.from(), r.inhibits_motion()), (1, true));
    assert!(inner.core.motion_inhibited_by_config());
    let c = inner.config_proposal(ConfigDomain::Collision, 1, 0, &ctx);
    assert_eq!(inner.reserve(&c, &ctx), Reserved::Refused { code: "CONFIG_BUSY", current: 1 });

    // nothing applied yet: activation hands the token back and the hold stays
    let Activated::Refused { code: "NOT_APPLIED", reservation: r } = inner.activate(r) else { panic!("not applied") };
    assert_eq!(state_of(&inner), Some(HoldState::Reserved));
    assert_eq!(inner.release(r), Released::Cleared);
    assert!(!inner.core.motion_inhibited_by_config());
    assert_eq!(inner.revision(ConfigDomain::Keepout), 1);

    // poses do not affect motion
    let p = inner.config_proposal(ConfigDomain::Poses, inner.revision(ConfigDomain::Poses), 0, &ctx);
    let Reserved::Granted(r) = inner.reserve(&p, &ctx) else { panic!("reserve poses") };
    assert!(!r.inhibits_motion() && !inner.core.motion_inhibited_by_config());
    assert_eq!(inner.release(r), Released::Cleared);
}

#[test]
fn persisted_then_activated_reports_the_durability_and_frees_motion() {
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let r = inner.reserve_config(ConfigDomain::Keepout, 1, 0).unwrap();
    let r = inner.config_persisted(r, Durability::DurableUnconfirmed).unwrap();
    assert_eq!(state_of(&inner), Some(HoldState::PersistedAwaitingActivation { durability: Durability::DurableUnconfirmed }));
    inner.set_keepout_boxes(vec![boxed("a", 0.3)]).unwrap();
    assert_eq!(inner.activate(r), Activated::Active { revision: 2, durability: Durability::DurableUnconfirmed });
    assert!(inner.core.config_hold().is_none() && !inner.core.motion_inhibited_by_config());
    // volatile: no persisted report
    let ((), rev) = inner.commit_config(ConfigDomain::Keepout, 2, 0, |i| i.set_keepout_boxes(vec![boxed("b", 0.3)])).unwrap();
    assert_eq!(rev, 3);
}

#[test]
fn activation_moves_the_revision_by_one_and_a_failed_apply_changes_nothing() {
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let ((), rev) = inner.commit_config(ConfigDomain::Keepout, 1, 0, |i| i.set_keepout_boxes(vec![boxed("a", 0.3)])).unwrap();
    assert_eq!((rev, inner.keepout.revision), (2, 2));
    assert_eq!(runtime_boxes(&inner), ["a"]);
    assert!(!inner.core.motion_inhibited_by_config());

    // a competing candidate built from revision 1 is refused; the active state is kept
    let err = inner.commit_config(ConfigDomain::Keepout, 1, 0, |i| i.set_keepout_boxes(vec![boxed("b", 0.3)])).unwrap_err();
    assert!(matches!(err, ArmError::PolicyRevisionMismatch { expected: 1, current: 2 }), "{err:?}");
    assert_eq!(runtime_boxes(&inner), ["a"]);

    // an invalid candidate fails to apply before any rename: revision, boxes and motion eligibility unchanged
    let bad = KeepoutBox { name: "bad".into(), min_m: [0.4; 3], max_m: [0.3; 3] };
    inner.commit_config(ConfigDomain::Keepout, 2, 0, |i| i.set_keepout_boxes(vec![bad])).unwrap_err();
    assert_eq!(inner.keepout.revision, 2);
    assert_eq!(runtime_boxes(&inner), ["a"]);
    assert!(inner.core.config_hold().is_none() && !inner.core.motion_inhibited_by_config());
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, { let mut q = INSPECT.to_vec(); q[0] += 0.05; q }, Some(0.3), None, "joint_move")) } }.expect("motion admitted");
}

/// The candidate was renamed into place, then failed to apply: active (revision 1, no
/// boxes) and durable differ, so the hold is `Reconciling`: motion inhibited, further
/// changes busy, until reconciliation finishes.
#[test]
fn apply_failure_after_the_rename_reconciles() {
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let r = inner.reserve_config(ConfigDomain::Keepout, 1, 0).unwrap();
    let r = inner.config_persisted(r, Durability::Durable).unwrap();
    let bad = KeepoutBox { name: "bad".into(), min_m: [0.4; 3], max_m: [0.3; 3] };
    inner.activate_config_candidate(r, |i| i.set_keepout_boxes(vec![bad])).unwrap_err();
    assert_eq!(state_of(&inner), Some(HoldState::Reconciling));
    assert!(inner.core.motion_inhibited_by_config());
    assert_eq!(inner.keepout.revision, 1);
    let err = inner.reserve_config(ConfigDomain::Poses, inner.poses.revision, 0).unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { code, .. } if code == "CONFIG_BUSY"), "{err:?}");
    assert!(inner.finish_reconcile(ConfigDomain::Keepout));
    assert!(!inner.core.motion_inhibited_by_config());

    // activation of a persisted candidate that was never applied reconciles as well
    let r = inner.reserve_config(ConfigDomain::Keepout, 1, 0).unwrap();
    let r = inner.config_persisted(r, Durability::Durable).unwrap();
    assert_eq!(inner.activate(r), Activated::Reconciling { code: "NOT_APPLIED" });
    assert!(inner.core.motion_inhibited_by_config());
}

#[test]
fn a_held_reservation_refuses_rt_installs_and_leaves_motion_untouched() {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = false;
    inner.tick(0.02);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(inner, BootId(7), Capacities { commands: 4, plans: 2, replies: 6 });
    let ctx = fx.ctx(fx.now());
    let p = fx.core.config_proposal(ConfigDomain::Keepout, 1, 0, &ctx);
    let Reserved::Granted(r) = fx.core.reserve(&p, &ctx) else { panic!("reserve") };

    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] += 0.05;
    let later = fx.now() + Duration::from_secs(10);
    let mut plan_snap = armd::rt_core::requests::motion::MotionPlanSnapshot::from_supervisor(&fx.core);
    plan_snap.authority.deadline = later;
    let plan = armd::rt_core::requests::motion::prepare_motion(
        &plan_snap,
        &armd::rt_core::requests::motion::MotionRequest::jog(q, Some(0.2), None),
    ).unwrap();
    let a = fx.core.authority(&fx.ctx(fx.now()));
    let req = b.submit(ClientId(1), 1, a, later, Submit::Install(&move |s: &mut PlanSlot| armd::rt_core::requests::motion::stage_jog(s, &plan.slot))).unwrap();
    let rep = fx.step();
    assert_eq!(rep.command, Some((req, Err("CONFIG_BUSY"))));
    assert!(fx.core.active_trajectory.is_none(), "no motion installed while the reservation is held");

    assert_eq!(fx.core.release(r), Released::Cleared);
    let a = fx.core.authority(&fx.ctx(fx.now()));
    let req2 = b.submit(ClientId(1), 2, a, later, Submit::Install(&move |s: &mut PlanSlot| armd::rt_core::requests::motion::stage_jog(s, &plan.slot))).unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r2, Ok(_))) if r2 == req2), "{rep:?}");
    assert!(fx.core.active_trajectory.is_some());
    let mut got = Vec::new();
    b.poll(|_, r| { got.push(r); true });
    assert!(format!("{:?}", got[0]).contains("CONFIG_BUSY"), "{got:?}");
}

// -- restart ------------------------------------------------------------------------------

/// The process ended after the candidate was renamed into place but before it was applied
/// and activated (its hold `PersistedAwaitingActivation`), with another candidate's temp
/// file still next to it. On the next start the loader reconciles: it activates the file
/// (active == durable), removes the never-published temp file and ends the reconciliation.
#[tokio::test]
async fn restart_after_rename_before_activation_activates_the_file() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&sup, &path).await.unwrap();
    {
        let mut inner = sup.inner().write().await;
        let mut prepared = armd::keepout::prepare_update(&inner, &json!({"boxes": [cube("new", 0.3)]})).unwrap();
        let w = prepared.write.take().unwrap();
        let staged = armd::persist::StagedWrite::stage(w.path, w.text).unwrap();
        let r = inner.reserve_config(ConfigDomain::Keepout, prepared.base_revision, 0).unwrap();
        drop(staged.rename_only().unwrap());
        let _r = inner.config_persisted(r, Durability::DurableUnconfirmed).unwrap();
        // crash here: never applied, never activated
        assert_eq!(inner.keepout.revision, 1);
        assert!(runtime_boxes(&inner).is_empty());
        assert!(matches!(state_of(&inner), Some(HoldState::PersistedAwaitingActivation { .. })));
        assert!(inner.core.motion_inhibited_by_config());
    }
    drop(sup);
    std::fs::write(dir.path().join(".keepout.json.tmp-1-9"), "{\"boxes\": [").unwrap();
    assert_eq!(disk_boxes(&path).unwrap(), ["new"]);

    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&fresh, &path).await.unwrap();
    let inner = fresh.inner().read().await;
    assert_eq!(runtime_boxes(&inner), ["new"]);
    assert_eq!(inner.keepout.revision, 1);
    assert!(inner.core.config_hold().is_none() && !inner.core.motion_inhibited_by_config());
    assert_eq!(files(dir.path()), ["keepout.json"]);
}

/// A start whose keep-out file cannot be installed stays `Reconciling`: the file stays on
/// disk, the profile boxes stay active and motion is inhibited.
#[tokio::test]
async fn restart_with_an_unloadable_file_stays_reconciling() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    std::fs::write(&path, "{\"boxes\": [ broken").unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&sup, &path).await.unwrap_err();
    let mut inner = sup.inner().write().await;
    assert_eq!(state_of(&inner), Some(HoldState::Reconciling));
    assert!(inner.core.motion_inhibited_by_config());
    let err = inner.reserve_config(ConfigDomain::Keepout, 1, 0).unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { code, .. } if code == "CONFIG_BUSY"), "{err:?}");
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "{\"boxes\": [ broken");
}

// -- the WS commit boundary --------------------------------------------------------------

async fn rig(path: Option<&Path>) -> (common::TestSocket, u64, u64, Arc<ArmSupervisor>) {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    if let Some(p) = path { sup.inner().write().await.keepout.path = Some(p.to_path_buf()); }
    let (ws, g, epoch, sup, _) = common::torqued_rig(sup, &INSPECT).await;
    (ws, g, epoch, sup)
}

/// Request ids must increase per connection; these start above `common::rpc`'s.
static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1_000_000);
fn next_id() -> u64 { NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed) }

async fn call(ws: &mut common::TestSocket, g: u64, epoch: u64, path: &str, body: Value) -> Value {
    let id = next_id();
    common::send(ws, json!({"type": "command", "request_id": id, "generation": g, "epoch_id": epoch, "path": path, "body": body})).await;
    reply_for(ws, id).await
}

async fn reply_for(ws: &mut common::TestSocket, request_id: u64) -> Value {
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v["request_id"] == json!(request_id) && (v["type"] == "command_ack" || v["type"] == "command_reject") { break v; }
        }
    })
    .await
    .expect("reply")
}

/// The replies to `ids`, in whatever order they arrive.
async fn replies_for(ws: &mut common::TestSocket, ids: &[u64]) -> std::collections::HashMap<u64, Value> {
    tokio::time::timeout(Duration::from_secs(10), async {
        let mut got = std::collections::HashMap::new();
        while got.len() < ids.len() {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            let is_reply = v["type"] == "command_ack" || v["type"] == "command_reject";
            if let Some(id) = v["request_id"].as_u64().filter(|id| is_reply && ids.contains(id)) { got.insert(id, v); }
        }
        got
    })
    .await
    .expect("replies")
}

fn keepout_cmd(request_id: u64, g: u64, epoch: u64, boxes: Vec<Value>, expected: u64) -> Value {
    json!({"type": "command", "request_id": request_id, "generation": g, "epoch_id": epoch,
        "path": "/api/v1/workspace/keepout", "body": {"boxes": boxes, "expected_revision": expected}})
}

/// Motion is admitted (and stopped again) on this connection.
async fn assert_motion_eligible(ws: &mut common::TestSocket, g: u64, epoch: u64, sup: &ArmSupervisor) {
    assert!(!sup.inner().read().await.core.motion_inhibited_by_config());
    let mut q = INSPECT.to_vec();
    q[0] += 0.05;
    let jog = call(ws, g, epoch, "/api/v1/motion/jog", json!({"target_positions": q, "speed_scale": 0.3})).await;
    assert_eq!(jog["type"], "command_ack", "{jog}");
    let stop = call(ws, g, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop["type"], "command_ack", "{stop}");
}

struct Delay;
impl Delay {
    fn set(ms: u64) -> Self { armd::persist::TEST_WRITE_DELAY_MS.store(ms, std::sync::atomic::Ordering::Relaxed); Delay }
}
impl Drop for Delay {
    fn drop(&mut self) { armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed); }
}

/// Two candidates built from revision 1, sent back to back while persistence is slow: one
/// activates (revision 2, on disk), the other is refused with the revision it lost to.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn competing_updates_activate_once() {
    let _serial = WRITE_DELAY.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let (mut ws, g, epoch, sup) = rig(Some(&path)).await;
    {
        let _d = Delay::set(200);
        let id_9101 = next_id();
        common::send(&mut ws, keepout_cmd(id_9101, g, epoch, vec![cube("first", 0.3)], 1)).await;
        let id_9102 = next_id();
        common::send(&mut ws, keepout_cmd(id_9102, g, epoch, vec![cube("second", 0.3)], 1)).await;
        let mut got = replies_for(&mut ws, &[id_9101, id_9102]).await;
        let (first, second) = (got.remove(&id_9101).unwrap(), got.remove(&id_9102).unwrap());
        // both persist tasks race for the reservation: either one wins, exactly one activates
        let first_won = first["type"] == "command_ack";
        let (a, b, name) = if first_won { (first, second, "first") } else { (second, first, "second") };
        assert_eq!((a["type"].as_str(), a["revision"].as_u64()), (Some("command_ack"), Some(2)), "{a}");
        assert_eq!(a["durability_confirmed"], true, "{a}");
        assert_eq!(b["type"], "command_reject", "{b}");
        assert_eq!(b["code"], "POLICY_REVISION_MISMATCH", "{b}");
        assert_eq!(disk_boxes(&path).unwrap(), [name]);
        let inner = sup.inner().read().await;
        assert_eq!((inner.keepout.revision, runtime_boxes(&inner)), (2, vec![name.to_string()]));
    }
    assert_eq!(files(dir.path()), ["keepout.json"]);
    assert_motion_eligible(&mut ws, g, epoch, &sup).await;
}

/// A motion stop while the candidate persists does not cancel the change.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_during_persistence_still_activates() {
    let _serial = WRITE_DELAY.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let (mut ws, g, epoch, sup) = rig(Some(&path)).await;
    let ack = {
        let _d = Delay::set(250);
        let id_9201 = next_id();
        common::send(&mut ws, keepout_cmd(id_9201, g, epoch, vec![cube("kept", 0.3)], 1)).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let stop = call(&mut ws, g, epoch, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop["type"], "command_ack", "{stop}");
        reply_for(&mut ws, id_9201).await
    };
    assert_eq!((ack["type"].as_str(), ack["revision"].as_u64()), (Some("command_ack"), Some(2)), "{ack}");
    assert_eq!(disk_boxes(&path).unwrap(), ["kept"]);
    assert_eq!(sup.inner().read().await.keepout.revision, 2);
    assert_eq!(files(dir.path()), ["keepout.json"]);
    assert_motion_eligible(&mut ws, g, epoch, &sup).await;
}

/// The lease is given up while the candidate persists: the staged file is discarded, the
/// old file and revision stay, nothing is reserved, and a new lease holder can move.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn lease_revocation_during_persistence_discards_the_candidate() {
    let _serial = WRITE_DELAY.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    std::fs::write(&path, serde_json::to_string(&json!({"boxes": [cube("old", 0.3)]})).unwrap()).unwrap();
    let sup0 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::keepout::load_into(&sup0, &path).await.unwrap();
    let (mut ws, g, epoch, sup, _) = common::torqued_rig(sup0, &INSPECT).await;
    let reject = {
        let _d = Delay::set(250);
        let id_9301 = next_id();
        common::send(&mut ws, keepout_cmd(id_9301, g, epoch, vec![cube("new", 0.3)], 1)).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let rel = common::ws_reply(&mut ws, json!({"type": "release_lease", "generation": g}), &["lease_released"]).await;
        assert_eq!(rel["success"], true, "{rel}");
        reply_for(&mut ws, id_9301).await
    };
    assert_eq!(reject["type"], "command_reject", "{reject}");
    assert_eq!(reject["code"], "LEASE_LOST", "{reject}");
    assert_eq!(disk_boxes(&path).unwrap(), ["old"]);
    {
        let inner = sup.inner().read().await;
        assert_eq!((inner.keepout.revision, runtime_boxes(&inner)), (1, vec!["old".to_string()]));
    }
    assert_eq!(files(dir.path()), ["keepout.json"]);
    let granted = common::ws_reply(&mut ws, json!({"type": "acquire_lease", "client_id": "test_controller"}), &["lease_granted"]).await;
    let g2 = granted["generation"].as_u64().unwrap();
    let torque = call(&mut ws, g2, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(torque["type"], "command_ack", "{torque}");
    assert_motion_eligible(&mut ws, g2, epoch, &sup).await;
}

/// Staging fails (the file's directory cannot be created): `PERSIST_FAILED`, nothing on
/// disk, the revision stays, nothing reserved.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn failed_staging_leaves_disk_revision_and_motion_unchanged() {
    let _serial = WRITE_DELAY.lock().await;
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("not_a_dir"), "x").unwrap();
    let path = dir.path().join("not_a_dir").join("keepout.json");
    let (mut ws, g, epoch, sup) = rig(Some(&path)).await;
    let id_9401 = next_id();
    common::send(&mut ws, keepout_cmd(id_9401, g, epoch, vec![cube("new", 0.3)], 1)).await;
    let reject = reply_for(&mut ws, id_9401).await;
    assert_eq!(reject["type"], "command_reject", "{reject}");
    assert_eq!(reject["code"], "PERSIST_FAILED", "{reject}");
    assert!(!path.exists());
    {
        let inner = sup.inner().read().await;
        assert_eq!((inner.keepout.revision, runtime_boxes(&inner).len()), (1, 0));
    }
    assert_eq!(files(dir.path()), ["not_a_dir"]);
    assert_motion_eligible(&mut ws, g, epoch, &sup).await;
}

/// A change with nothing to persist goes through the same reservation (volatile).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn volatile_change_is_reserved_and_activated() {
    let _serial = WRITE_DELAY.lock().await;
    let (mut ws, g, epoch, sup) = rig(None).await;
    let id_9501 = next_id();
    common::send(&mut ws, keepout_cmd(id_9501, g, epoch, vec![cube("mem", 0.3)], 1)).await;
    let ack = reply_for(&mut ws, id_9501).await;
    assert_eq!((ack["type"].as_str(), ack["revision"].as_u64()), (Some("command_ack"), Some(2)), "{ack}");
    assert!(ack.get("persisted").is_none(), "{ack}");
    let id_9502 = next_id();
    common::send(&mut ws, keepout_cmd(id_9502, g, epoch, vec![cube("stale", 0.3)], 1)).await;
    let rej = reply_for(&mut ws, id_9502).await;
    assert_eq!(rej["code"], "POLICY_REVISION_MISMATCH", "{rej}");
    assert_eq!(runtime_boxes(&*sup.inner().read().await), ["mem"]);
    assert_motion_eligible(&mut ws, g, epoch, &sup).await;
}

/// B2: `config_ctx` supplies a real non-zero boot identity, `config_ctx_with` / `ReleaseFixture::reserve_config`
/// thread real executor boot identity and live urgent-stop generation, and stale/zero boot ids
/// or overtaken stops are refused without inhibiting motion or altering active configuration.
#[test]
fn config_identity_refuses_stale_boot_zero_boot_and_overtaken_stop() {
    use armd::rt_core::urgent::StopRequests;

    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let t0 = Instant::now();
    let legacy_ctx = inner.config_ctx(t0);
    assert!(!legacy_ctx.boot.is_zero(), "config_ctx must not substitute BootId(0)");
    assert_eq!(legacy_ctx.boot, BootId::process());

    // Live executor boot + live StopRequests threaded through config_ctx_with & reserve_config_proposal.
    let boot = BootId::fresh();
    let stale_boot = BootId::fresh();
    assert_ne!(boot, stale_boot);
    let stops = StopRequests::new();
    let ctx = inner.config_ctx_with(boot, &stops, t0);
    assert_eq!((ctx.boot, ctx.stop_generation), (boot, 0));

    let p = inner.config_proposal(ConfigDomain::Keepout, inner.revision(ConfigDomain::Keepout), 0, &ctx);

    // 1. Stale boot id refused with WRONG_TARGET.
    let mut p_stale = p;
    p_stale.stamp.boot = stale_boot;
    let err = inner.reserve_config_proposal(&p_stale, &ctx).unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { code, .. } if code == "WRONG_TARGET"), "{err:?}");

    // 2. Zero boot id refused fail-closed even when both proposal and ctx pass BootId(0).
    let mut p_zero = p;
    p_zero.stamp.boot = BootId(0);
    let zero_ctx = armd::rt_core::ops::CommitCtx { boot: BootId(0), ..ctx };
    assert_eq!(inner.reserve(&p_zero, &zero_ctx), Reserved::Refused { code: "WRONG_TARGET", current: 1 });

    // 3. Overtaken stop refused with STOPPED when live stop generation advances before reserve.
    stops.request();
    let after_stop = inner.config_ctx_with(boot, &stops, t0);
    assert_eq!(after_stop.stop_generation, 1);
    let err = inner.reserve_config_proposal(&p, &after_stop).unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { code, .. } if code == "STOPPED"), "{err:?}");
    assert!(!inner.core.motion_inhibited_by_config());
    assert_eq!(inner.keepout.revision, 1);

    // 4. Proposal stamped at the live stop generation succeeds and commits.
    let fresh_p = inner.config_proposal(ConfigDomain::Keepout, 1, 0, &after_stop);
    let ((), rev) = inner.commit_config_proposal(&fresh_p, &after_stop, |i| i.set_keepout_boxes(vec![boxed("live", 0.3)])).unwrap();
    assert_eq!((rev, runtime_boxes(&inner)), (2, vec!["live".to_string()]));
    assert!(!inner.core.motion_inhibited_by_config());

    // 5. Same contract through ReleaseFixture::new_fresh + reserve_config.
    let (mut fx, _b, _tw, fx_stops) = ReleaseFixture::new_fresh(inner, Capacities::default());
    assert!(!fx.boot().is_zero());
    let fx_ctx = fx.ctx(fx.now());
    let fp = fx.core.config_proposal(ConfigDomain::Keepout, 2, 0, &fx_ctx);
    let mut wrong_fx = fp;
    wrong_fx.stamp.boot = stale_boot;
    assert_eq!(fx.reserve_config(&wrong_fx), Reserved::Refused { code: "WRONG_TARGET", current: 2 });
    fx_stops.request();
    assert_eq!(fx.reserve_config(&fp), Reserved::Refused { code: "STOPPED", current: 2 });
    let fp2 = fx.core.config_proposal(ConfigDomain::Keepout, 2, 0, &fx.ctx(fx.now()));
    let Reserved::Granted(r) = fx.reserve_config(&fp2) else { panic!("fresh proposal after stop should grant") };
    assert_eq!(fx.core.release(r), Released::Cleared);
}
