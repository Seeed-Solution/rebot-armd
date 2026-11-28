//! Unit tests for edge-triggered terminal-action capture and publish-barrier synchronization.

use super::*;
    use crate::model::ArmConfig;
    use crate::motion::{EstimateEntry, MacroMotionDiag};
    use crate::supervisor::ArmSupervisor;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn edge_trigger_captures_once_zero_on_dwell_and_preserves_2s_previous_action() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<ArmObservation>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_c.lock().unwrap().push(obs.clone());
            true
        });

        let id1 = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };

        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // 50 dwell ticks must perform 0 additional captures.
        for _ in 0..50 {
            sup.control_tick(0.02).await;
        }
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);
        assert_eq!(sup.test_terminal_capture_stats().saturated, 0);

        // Starting action 2 moves already-terminal action 1 to `previous_action` (2 s window):
        // must NOT re-capture action 1, while telemetry still exposes `previous_action` within 2 s.
        let id2 = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("park");
            // Keep `park` running during the tick so reconcile_actions does not auto-cancel it.
            inner.pending_normal_unload = true;
            inner.defer_terminal_capture = false;
            id
        };
        let obs_with_prev = sup.tick(0.02).await;
        assert_eq!(obs_with_prev.action.as_ref().map(|a| a.id), Some(id2));
        assert_eq!(obs_with_prev.previous_action.as_ref().map(|a| a.id), Some(id1));
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Repeat finish_action on already-terminal id1 must be a strict no-op.
        {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            inner.finish_action(id1, "done", None, None, None, None, None);
            inner.defer_terminal_capture = false;
        }
        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Expire previous_action past 2 s window and confirm obs omits it without extra capture.
        {
            let mut inner = sup.inner().write().await;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
        }
        let obs_after_2s = sup.tick(0.02).await;
        assert!(obs_after_2s.previous_action.is_none());
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Wait for dispatcher delivery: action 1 delivered strictly once.
        let dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < dl {
            if delivered
                .lock()
                .unwrap()
                .iter()
                .any(|o| o.action.as_ref().is_some_and(|a| a.id == id1))
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        tokio::time::sleep(Duration::from_millis(25)).await;
        let ids: Vec<u64> = delivered
            .lock()
            .unwrap()
            .iter()
            .filter_map(|o| o.action.as_ref().map(|a| a.id))
            .collect();
        assert_eq!(ids, vec![id1]);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn edge_trigger_off_tick_ordering_failed_step_and_zero_to_one_cutoff() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;

        // Pre-subscription terminal while subscriber_count == 0 and drain paused.
        sup.test_set_terminal_capture_drain_paused(true);
        let pre_id = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);
        assert_eq!(sup.test_terminal_capture_stats().queued, 1);

        // Unpause capture drain and register first subscriber: pre-subscription record must be
        // cleared from the ring and NOT delivered to the new subscriber.
        sup.test_set_terminal_capture_drain_paused(false);
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });
        assert_eq!(sup.test_terminal_capture_stats().queued, 0);

        // Action A finishes in-tick (queued in SPSC ring), then Action B finishes off-tick,
        // and `update_recorded_terminal_failed_step` patches Action A before dispatch flushes.
        sup.test_set_terminal_dispatch_paused(true);
        let id_a = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("sequence");
            inner.finish_action(id, "failed", Some("STEP_ERR"), None, None, None, None);
            inner.defer_terminal_capture = false;
            id
        };
        sup.control_tick(0.02).await;
        {
            let inner = sup.inner().read().await;
            inner.update_recorded_terminal_failed_step(id_a, Some(3));
        }
        let id_b = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("jog", None);
            inner.finish_action(id, "done", None, None, None, None, None);
            id
        };
        sup.test_set_terminal_dispatch_paused(false);
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        let ids: Vec<u64> = got.iter().map(|a| a.id).collect();
        assert_eq!(
            ids,
            vec![id_a, id_b],
            "must skip pre-sub {pre_id} and preserve strict FIFO [A, B]"
        );
        assert_eq!(
            got[0].failed_step,
            Some(3),
            "failed_step patch must apply to in-tick captured action"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn gated_telemetry_delivers_once_terminals_clear_without_new_publication() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        let mut tel_rx = sup.subscribe_telemetry().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<u64>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.id);
            }
            true
        });

        // Pause broadcaster drain so telemetry is gated on pending terminal.
        sup.test_set_terminal_dispatch_paused(true);
        let id = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            tel_rx.try_recv().is_err(),
            "telemetry must remain gated while terminal is pending"
        );

        // Unpause WITHOUT running another tick: both terminal AND gated telemetry must arrive.
        sup.test_set_terminal_dispatch_paused(false);
        let tel_obs = tokio::time::timeout(Duration::from_secs(2), tel_rx.recv())
            .await
            .expect("gated telemetry must flush once terminal clears")
            .expect("broadcast ok");
        assert_eq!(tel_obs.action.as_ref().map(|a| a.id), Some(id));
        assert_eq!(*delivered.lock().unwrap(), vec![id]);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_macro_full_rich_payload_preserved_past_2s_window_under_lock_contention() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.test_set_terminal_fifo_capacity(2);
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        // Create a rich macro off-tick with `episodes`, `motions`, `height_estimates`, `phase_ms`,
        // AND grasp diagnostic fields (`held_width_m`, `close_height_m`), then update `phase` to
        // `"done"` under guarded deferral, finish it in-tick + start park + expire 2 s window.
        let (id_macro, id_park, expected_action) = {
            let mut inner = sup.inner().write().await;
            let id1 = inner.begin_action("place", Some("tag-1".into()));
            inner.set_action_phase(id1, "close", 0.75, Some(2), Some(3.5));
            inner.update_action(id1, |a| {
                a.episodes.push("ep-001".into());
                a.motions.push(MacroMotionDiag {
                    phase: "descend".into(),
                    ms: 145,
                    track_mm: 1.2,
                    track_deg: 0.4,
                    goal_mm: 0.5,
                    goal_deg: 0.2,
                    contact: Some(true),
                });
                a.height_estimates.push(EstimateEntry {
                    source: "depth:metric_table".into(),
                    value: Some(0.085),
                    used: true,
                    record_only: false,
                    reason: None,
                });
                a.held_width_m = Some(0.032);
                a.close_height_m = Some(0.041);
                a.phase_ms.insert("descend".into(), 145);
            });
            inner.defer_terminal_capture = true;
            // Simulate a guarded Legacy completion updating `phase` while mirroring is disabled.
            inner.set_action_phase(id1, "done", 1.0, None, None);
            if let Some(a) = inner.last_action.as_mut() {
                a.phase_ms.insert("close".into(), 80);
            }
            inner.finish_action(id1, "done", None, None, Some(true), None, None);
            let expected = inner.last_action.clone().expect("finished macro slot");
            // Staged terminal (1) + starting park (1) == 2 (allowed).
            assert!(inner.check_terminal_fifo_for_new_action().is_ok());
            let id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            // Now staged (1) + running park (needed=2) == 3 > 2 -> MUST return TerminalsBusy!
            assert!(matches!(
                inner.check_terminal_fifo_for_new_action(),
                Err(crate::safety::SafetyError::TerminalsBusy)
            ));
            // Expire previous_action past 2 s window BEFORE publish + drain.
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id1, id2, expected)
        };

        sup.control_tick(0.02).await;
        // Hold SupervisorLock write guard during drain to prove rich payload recovery does NOT
        // depend on `SupervisorLock::try_read()`.
        let _write_hold = sup.inner().write().await;
        let _ = sup.test_drain_terminal_captures();
        drop(_write_hold);

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(
            got[0], expected_action,
            "delayed rich terminal ActionStatus must match full terminal payload field-for-field"
        );
        assert_eq!(got[0].id, id_macro);
        assert_eq!(got[0].phase.as_deref(), Some("done"));
        assert_eq!(got[0].held_width_m, Some(0.032));
        assert_eq!(got[0].close_height_m, Some(0.041));
        assert_ne!(id_macro, id_park);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn deferred_staging_materializes_before_slot_overwrite_preserving_all_ids_in_order() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        // Finish A under deferral, start B (A -> previous_action), then start C (B supersedes and
        // overwrites A in previous_action) BEFORE `capture_edge_terminals` runs.
        let (id_a, id_b, id_c) = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let a = inner.begin_action_rt("jog");
            inner.finish_action(a, "done", None, None, None, None, None);
            let b = inner.begin_action_rt("jog");
            let c = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            inner.defer_terminal_capture = false;
            (a, b, c)
        };

        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        let ids: Vec<u64> = got.iter().map(|a| a.id).collect();
        assert_eq!(
            ids,
            vec![id_a, id_b],
            "materialized deferred A and superseded B must both be delivered in strict FIFO order (C={id_c} still running)"
        );
        assert_eq!(got[0].state.as_str(), "done");
        assert_eq!(got[1].state.as_str(), "cancelled");
        assert_eq!(got[1].code.as_deref(), Some("SUPERSEDED"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn publish_seq_barrier_carried_through_broadcaster_prevents_stale_snapshot_pairing() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<(u64, f64)>::new()));
        let delivered_c = Arc::clone(&delivered);
        // Subscribe directly on `TerminalBroadcaster` without spawning `armd-dispatch` so the
        // manual drains below are 100% deterministic and cannot race with a background thread.
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push((a.id, obs.timestamp));
            }
            true
        }));

        // Snapshot N published before terminal A.
        sup.control_tick(0.02).await;
        let snap_n = sup.hub().latest().expect("snapshot N");

        // Legacy tick publishes terminal A at snapshot N+1 while dispatcher is paused on N.
        let id_a = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        let snap_n1 = sup.hub().latest().expect("snapshot N+1");
        assert!(snap_n1.meta.seq > snap_n.meta.seq);

        // Off-tick Legacy action start drains A from the SPSC ring into `TerminalBroadcaster`
        // (bypassing the ring barrier with `respect_publish_barrier = false`).
        let _id_b = {
            let mut inner = sup.inner().write().await;
            inner.begin_action("jog", None)
        };
        assert_eq!(sup.test_terminal_capture_stats().queued, 0);

        // Attempting to flush `TerminalBroadcaster` with stale snapshot N MUST NOT emit A!
        let blocked = sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq);
        assert!(
            blocked,
            "drain_all against stale snapshot N must hold terminal A gated on required_publish_seq"
        );
        assert!(
            delivered.lock().unwrap().is_empty(),
            "terminal A must not be paired with stale snapshot N"
        );

        // Once snapshot N+1 is used, terminal A flushes paired with snapshot N+1.
        let blocked_after = sup.terminals.drain_all(&snap_n1.obs, snap_n1.meta.seq);
        assert!(!blocked_after);
        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0, id_a);
        assert_eq!(got[0].1, snap_n1.obs.timestamp);
        sup.terminals.unsubscribe(sub_id);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_cancellation_clears_mirrored_workspace_violation_diagnostics() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        let (id_a, expected) = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("grasp", None);
            inner.set_action_workspace_violation(
                id,
                crate::kinematics::WorkspaceViolationDetail {
                    constraint_id: Some("table_plane".into()),
                    geometry_id: Some("jaw_tip".into()),
                    violation_phase: Some("descend".into()),
                    reason: Some("z below table".into()),
                },
            );
            inner.update_action(id, |a| {
                a.approach = Some("same_side".into());
            });
            // Contact interrupts same-side approach -> must update mirror so delayed cancellation
            // reports `direct` + `contact` instead of stale `same_side`.
            inner.approach_interrupted(id, "contact");
            // Guarded cancellation clears workspace violation fields (`finish_action`).
            inner.defer_terminal_capture = true;
            inner.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
            let exp = inner.last_action.clone().expect("cancelled slot");
            assert!(exp.constraint_id.is_none());
            assert!(exp.geometry_id.is_none());
            assert!(exp.violation_phase.is_none());
            assert!(exp.reason.is_none());
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id, exp)
        };

        sup.control_tick(0.02).await;
        let _write_hold = sup.inner().write().await;
        let _ = sup.test_drain_terminal_captures();
        drop(_write_hold);

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, id_a);
        assert_eq!(
            got[0], expected,
            "delayed cancellation must clear previously mirrored workspace violation fields"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn post_publish_and_shutdown_captures_synthesize_self_contained_observation() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ArmObservation>::new()));
        let delivered_c = Arc::clone(&delivered);
        // Subscribe directly on `TerminalBroadcaster` (no background dispatcher) for deterministic
        // manual drain verification.
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            delivered_c.lock().unwrap().push(obs.clone());
            true
        }));

        // Publish baseline snapshot N with initial epoch and timestamp.
        sup.control_tick(0.02).await;
        let snap_n = sup.hub().latest().expect("snapshot N");

        // Simulate post-publish capture with `publish_seq: None` (like persistence saturation or
        // control_stop) after moving the gripper joint and advancing timestamp without another
        // publish_at. Verify self-contained field consistency (`positions[gripper.joint] == gripper.rad`,
        // `feedback_age_ms`) AND `DrainOutcome.last_synthetic_action_id`.
        let gripper_idx = {
            let inner = sup.inner().read().await;
            inner.config.gripper_index().expect("rebot_dm has gripper")
        };
        let moved_gripper_rad = 0.42f32;
        let id_sat = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("gripper_zero_check");
            inner.measured_positions[gripper_idx] = moved_gripper_rad;
            inner.finish_action(id, "failed", Some("PERSIST_FAILED"), None, None, None, None);
            let ts_after = snap_n.obs.timestamp + 1.25;
            let _ = inner.capture_edge_terminals(ts_after, None);
            inner.defer_terminal_capture = false;
            id
        };
        let drained_sat = sup.terminal_capture_channel.drain_into(
            &sup.terminals,
            &sup.hub,
            Some(&snap_n),
            &[],
            &sup.template_obs,
            true,
        );
        assert_eq!(drained_sat, 1);
        assert!(
            sup.terminal_capture_channel
                .was_synthetically_terminated(id_sat),
            "synthetic post-publish drain must record action_id across all consumers so stale running pending_telemetry is suppressed"
        );
        assert!(!sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq));
        {
            let got = delivered.lock().unwrap();
            assert_eq!(got.len(), 1);
            assert_eq!(got[0].action.as_ref().map(|a| a.id), Some(id_sat));
            assert!(
                (got[0].timestamp - (snap_n.obs.timestamp + 1.25)).abs() < 1e-6,
                "publish_seq: None capture must synthesize self-contained observation at capture timestamp"
            );
            let grip = got[0].gripper.as_ref().expect("synthesized obs must include gripper");
            assert_eq!(grip.joint, gripper_idx);
            let gz = got[0]
                .gripper_zero
                .as_ref()
                .expect("synthesized obs must include gripper_zero");
            assert!(
                (got[0].positions[gripper_idx] - moved_gripper_rad).abs() < 1e-6
                    && (grip.rad - moved_gripper_rad).abs() < 1e-6
                    && (gz.j7 - moved_gripper_rad).abs() < 1e-6,
                "synthesized positions[gripper.joint] ({}), gripper.rad ({}), and gripper_zero.j7 ({}) must all agree",
                got[0].positions[gripper_idx],
                grip.rad,
                gz.j7
            );
            assert!(
                got[0].tcp.is_some(),
                "synthesized observation must compute tcp from captured joint positions"
            );
            assert_eq!(got[0].feedback_age_ms.len(), got[0].positions.len());
            assert_eq!(got[0].gravity_comp.phase, None);
            assert_eq!(got[0].gravity_comp.ramp, 0.0);
            assert!(got[0].gravity_comp.tau_ff.is_empty());
        }

        // Simulate hub closing between loop check and normal `drain_into(..., respect_publish_barrier = true)`
        // when a queued record has `publish_seq > final_snap.meta.seq`: unified `enforce_barrier`
        // MUST synthesize `obs_override` (NOT carry an unmet `required_publish_seq`), so `drain_all`
        // delivers it against `snap_n.meta.seq` without stranding.
        let id_shut = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("jog");
            inner.finish_action(id, "done", None, None, None, None, None);
            let ts_shut = snap_n.obs.timestamp + 2.50;
            let future_seq = snap_n.meta.seq + 10;
            let _ = inner.capture_edge_terminals(ts_shut, Some(future_seq));
            inner.defer_terminal_capture = false;
            id
        };
        sup.hub().close();
        let drained_shut = sup.terminal_capture_channel.drain_into(
            &sup.terminals,
            &sup.hub,
            Some(&snap_n),
            &[],
            &sup.template_obs,
            true,
        );
        assert_eq!(drained_shut, 1);
        assert!(sup.terminal_capture_channel.was_synthetically_terminated(id_sat));
        assert!(sup.terminal_capture_channel.was_synthetically_terminated(id_shut));
        let blocked = sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq);
        assert!(
            !blocked,
            "hub_closed drain must synthesize self-contained obs so drain_all does not strand"
        );
        let got = delivered.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[1].action.as_ref().map(|a| a.id), Some(id_shut));
        assert!(
            (got[1].timestamp - (snap_n.obs.timestamp + 2.50)).abs() < 1e-6,
            "hub_closed drain ahead of final_snap.seq must emit captured observation"
        );
        sup.terminals.unsubscribe(sub_id);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_recovery_preserves_long_error_message_and_utf8_boundary_label() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        }));

        // 1) 127-byte ASCII prefix + multibyte UTF-8 char (`'é'`, 2 bytes -> total 129 bytes)
        //    in BOTH user-supplied `tag` (from off-RT `begin_action`) and `reason` MUST NOT be
        //    truncated on delayed recovery past the 2 s window.
        let utf8_reason = format!("{}é-suffix", "a".repeat(127));
        let long_tag = format!("{}é-user-tag", "t".repeat(127));
        assert!(utf8_reason.len() > HolderLabel::CAPACITY);
        assert_eq!(HolderLabel::new(&utf8_reason).len(), 127);
        assert!(long_tag.len() > HolderLabel::CAPACITY);

        let (id_utf8, expected_utf8) = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("gripper", Some(long_tag.clone()));
            inner.set_action_workspace_violation(
                id,
                crate::kinematics::WorkspaceViolationDetail {
                    constraint_id: Some("c1".into()),
                    geometry_id: Some("g1".into()),
                    violation_phase: Some("descend".into()),
                    reason: Some(utf8_reason.clone()),
                },
            );
            inner.defer_terminal_capture = true;
            inner.finish_action(
                id,
                "failed",
                Some("WORKSPACE_VIOLATION"),
                None,
                None,
                None,
                None,
            );
            let exp = inner.last_action.clone().expect("workspace violation slot");
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id, exp)
        };
        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        // 2) >256-byte persistence worker error message mirrored via `mirror_action_message`
        //    must survive 2 s expiry without 256-byte truncation.
        let long_msg = format!("Failed to sync gripper calibration: {}", "x".repeat(360));
        assert!(long_msg.len() > 256);
        let id_long = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("gripper_zero_check");
            sup.terminal_capture_channel
                .mirror_action_message(id, &long_msg);
            inner.finish_action(
                id,
                "failed",
                Some("PERSIST_FAILED"),
                Some(long_msg.clone()),
                None,
                None,
                None,
            );
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            id
        };
        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 3, "expected [id_utf8, superseded park, id_long]");
        assert_eq!(got[0].id, id_utf8);
        assert_eq!(got[0], expected_utf8, "UTF-8 boundary truncated label/tag must preserve full cached values");
        assert_eq!(got[0].tag.as_deref(), Some(long_tag.as_str()));
        assert_eq!(got[0].reason.as_deref(), Some(utf8_reason.as_str()));
        assert_eq!(got[1].code.as_deref(), Some("SUPERSEDED"));
        assert_eq!(got[2].id, id_long);
        assert_eq!(
            got[2].message.as_deref(),
            Some(long_msg.as_str()),
            "delayed >256-byte persistence error message must be recovered losslessly"
        );
        sup.terminals.unsubscribe(sub_id);
    }

    #[test]
    fn cached_feedback_freshness_respects_numeric_validity_poll_valid_instants_and_250ms_boundary() {
        let t0 = Instant::now();
        let max_age = Duration::from_millis(250);
        // Driverless: solely gated by feedback_numeric_valid.
        assert!(evaluate_cached_feedback_fresh(
            true, false, false, 7, &[], &[], None, t0, max_age
        ));
        assert!(!evaluate_cached_feedback_fresh(
            false, true, false, 7, &[], &[], None, t0, max_age
        ));

        // Driver with per-joint instants: fresh at +240ms, stale at +260ms, false when numeric
        // invalid or last_poll_feedback_valid is false (without mutating instants!).
        let instants = vec![Some(t0); 7];
        assert!(evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &instants,
            &[],
            None,
            t0 + Duration::from_millis(240),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &instants,
            &[],
            None,
            t0 + Duration::from_millis(260),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            false,
            true,
            true,
            7,
            &instants,
            &[],
            None,
            t0 + Duration::from_millis(10),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            true,
            false,
            true,
            7,
            &instants,
            &[],
            None,
            t0 + Duration::from_millis(10),
            max_age,
        ));

        // Distinct per-joint ages without instants: preserves per-joint values and agrees with freshness!
        let distinct_ages = [
            Some(Duration::from_millis(10)),
            Some(Duration::from_millis(20)),
            Some(Duration::from_millis(30)),
            Some(Duration::from_millis(40)),
            Some(Duration::from_millis(50)),
            Some(Duration::from_millis(60)),
            Some(Duration::from_millis(200)),
        ];
        let (ages_40, fresh_40) = compute_cached_feedback_state(
            true,
            true,
            true,
            7,
            &[],
            &distinct_ages,
            Some(t0),
            t0 + Duration::from_millis(40),
            max_age,
        );
        assert!(fresh_40);
        assert_eq!(ages_40[0], Some(50));
        assert_eq!(ages_40[6], Some(240));

        // Partial instants + complete distinct ages: freshness and per-joint age_ms derive from
        // the exact same sample (no missing ages while fresh == true).
        let partial_instants = [Some(t0), None, None, None, None, None, None];
        let (ages_partial, fresh_partial) = compute_cached_feedback_state(
            true,
            true,
            true,
            7,
            &partial_instants,
            &distinct_ages,
            Some(t0),
            t0 + Duration::from_millis(40),
            max_age,
        );
        assert!(fresh_partial);
        assert!(ages_partial[..7].iter().all(Option::is_some));
        assert_eq!(ages_partial[1], Some(60));

        assert!(!evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &[],
            &distinct_ages,
            Some(t0),
            t0 + Duration::from_millis(60),
            max_age,
        ));
        // Cleared poll cache (after read/lock error) fails closed.
        assert!(!evaluate_cached_feedback_fresh(
            true, false, true, 7, &[], &[], None, t0, max_age
        ));
    }

    #[test]
    fn install_driver_does_not_advance_feedback_history_before_first_poll() {
        use std::sync::atomic::{AtomicBool, Ordering as AOrd};

        struct InstantPollDriver {
            dof: usize,
            valid: Arc<AtomicBool>,
            read_ok: Arc<AtomicBool>,
            instant: Instant,
        }

        impl crate::driver::ArmDriver for InstantPollDriver {
            fn read_joint_states(
                &mut self,
            ) -> Result<Vec<crate::model::JointState>, crate::driver::ArmDriverError> {
                if !self.read_ok.load(AOrd::Acquire) {
                    return Err(crate::driver::ArmDriverError::Protocol(
                        "injected poll fail".into(),
                    ));
                }
                Ok(vec![crate::model::JointState::default(); self.dof])
            }
            fn write_joint_targets(
                &mut self,
                _targets: &[crate::JointTarget],
            ) -> Result<(), crate::driver::ArmDriverError> {
                Ok(())
            }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), crate::driver::ArmDriverError> {
                Ok(())
            }
            fn is_feedback_valid(&self) -> bool {
                self.valid.load(AOrd::Acquire)
            }
            fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
                out.clear();
                out.resize(self.dof, Some(self.instant));
            }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(5)));
            }
        }

        let cfg = ArmConfig::rebot_dm();
        let dof = cfg.dof;
        let mut inner = SupervisorInner::new(cfg);
        let valid = Arc::new(AtomicBool::new(true));
        let read_ok = Arc::new(AtomicBool::new(false));
        let drv = InstantPollDriver {
            dof,
            valid: Arc::clone(&valid),
            read_ok: Arc::clone(&read_ok),
            instant: Instant::now(),
        };

        let seqs_before = inner.core.feedback().seqs().to_vec();
        let interlock_before = (
            inner.last_poll_enable_state,
            inner.last_poll_partial_enable,
            inner.last_poll_feedback_valid,
            inner.last_poll_oldest_age,
        );
        inner.install_driver(Box::new(drv));
        assert_eq!(
            (
                inner.last_poll_enable_state,
                inner.last_poll_partial_enable,
                inner.last_poll_feedback_valid,
                inner.last_poll_oldest_age,
            ),
            interlock_before,
            "install_driver must not mutate hardware park/torque-off interlock last_poll_* fields"
        );
        assert_eq!(
            inner.core.feedback().seqs(),
            seqs_before.as_slice(),
            "install_driver must not advance ArmCore feedback sequences before the first control poll"
        );
        assert!(
            inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "seeded poll-age fallback keeps freshness valid immediately after attachment"
        );

        // First poll fails -> cached freshness fails closed while sequences and interlock stay unchanged.
        inner.poll_hardware_rx(None, None, None);
        assert!(
            !inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "failed poll must make cached freshness fail closed"
        );
        assert_eq!(inner.core.feedback().seqs(), seqs_before.as_slice());
        assert_eq!(
            (
                inner.last_poll_enable_state,
                inner.last_poll_partial_enable,
                inner.last_poll_feedback_valid,
                inner.last_poll_oldest_age,
            ),
            interlock_before,
            "failed poll must not mutate hardware interlock last_poll_* fields"
        );

        // Subsequent valid poll -> freshness becomes true and sequences advance by exactly 1.
        read_ok.store(true, AOrd::Release);
        inner.poll_hardware_rx(None, None, None);
        assert!(
            inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "successful poll must restore cached freshness"
        );
        for (i, (&after, &before)) in inner
            .core
            .feedback()
            .seqs()
            .iter()
            .zip(seqs_before.iter())
            .enumerate()
        {
            assert_eq!(
                after,
                before + 1,
                "joint {i} feedback sequence must advance exactly once on first valid poll"
            );
        }
        // Subsequent read error after a valid poll matches baseline `last_poll_*` behavior
        // (`feedback_valid=false, oldest_age=None`, enable state unchanged) while failing closed
        // in `capture_driver_cache`.
        let enable_after_valid = (inner.last_poll_enable_state, inner.last_poll_partial_enable);
        read_ok.store(false, AOrd::Release);
        inner.poll_hardware_rx(None, None, None);
        assert!(
            !inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "read error after valid poll must fail closed in capture_driver_cache"
        );
        assert_eq!(
            (inner.last_poll_enable_state, inner.last_poll_partial_enable),
            enable_after_valid,
            "read error must preserve baseline enable interlock fields"
        );
        assert!(!inner.last_poll_feedback_valid);
        assert!(inner.last_poll_oldest_age.is_none());
    }

