//! Unit tests for observation snapshot publication, terminal broadcaster, and stamp namespaces.

use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_holding_the_hub_does_not_delay_the_tick() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let sup = sup.clone();
            // a preempted reader: holds the readers' mutex until released
            std::thread::spawn(move || {
                let _g = sup.hub.reader_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let t = Instant::now();
        let mut last = 0.0;
        for _ in 0..3 {
            last = tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("tick must not wait for readers").timestamp;
        }
        assert!(t.elapsed() < Duration::from_millis(100), "{:?}", t.elapsed());
        release_tx.send(()).unwrap();
        holder.join().unwrap();
        // nothing was skipped: the newest sample is the last tick's
        let served = sup.get_observation().await;
        assert_eq!(served.timestamp, last);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses, c.publish_skipped, c.published), (1, 0, 0, 4));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_that_waited_for_the_lock_serves_the_sample_published_meanwhile() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let mut writer = sup.inner().write().await;
        let reader = {
            let sup = sup.clone();
            tokio::spawn(async move { sup.get_observation().await })
        };
        tokio::time::sleep(Duration::from_millis(10)).await;
        // what a tick does under its guard
        let mut published = writer.current_observation();
        published.timestamp = 123.0;
        let generation = sup.inner().generation();
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *writer;
        let _ = core.publish_displacing(generation, Instant::now(), Instant::now() + Duration::from_secs(1), published, measured_positions, measured_velocities);
        drop(writer);
        assert_eq!(reader.await.unwrap().timestamp, 123.0);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses), (1, 0));
    }

    /// Read-your-writes: a write after the tick is answered by the next tick's sample, not
    /// by the older one and not under the lock.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_write_waits_for_the_next_sample_while_the_loop_runs() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        sup.set_temperatures(vec![55.0; 7]).await;
        let ticker = {
            let sup = sup.clone();
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(5)).await;
                sup.tick(0.02).await
            })
        };
        let obs = sup.get_observation().await;
        let ticked = ticker.await.unwrap();
        assert_eq!(obs.temperatures, vec![55.0; 7]);
        assert_eq!(obs.timestamp, ticked.timestamp);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses, c.waited), (1, 0, 1));
    }

    /// A tick that stalls inside its write guard: GET /observation answers 503 within its
    /// bounds instead of waiting for the lock.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_stalled_tick_gives_observation_unavailable_in_bounded_time() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let stalled = sup.inner().write().await; // the tick never releases its guard
        let t = Instant::now();
        let r = tokio::time::timeout(Duration::from_secs(3), sup.observation()).await.expect("must not hang");
        let e = r.expect_err("no fresh state");
        assert!(matches!(&e, ArmError::Rejected { status: 503, code, .. } if code == "OBSERVATION_UNAVAILABLE"), "{e:?}");
        assert!(t.elapsed() < OBS_LOCK_WAIT + Duration::from_millis(300), "{:?}", t.elapsed());
        assert_eq!(sup.tick_stats().observation_cache.unavailable, 1);
        drop(stalled);
        assert!(sup.observation().await.is_ok());
    }

    /// The second lock wait (a sample was published while the first one waited, but it did
    /// not answer) shares the first wait's deadline, and a sample published by the writer
    /// that keeps it from the lock answers at that deadline instead of a 503.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_second_lock_wait_shares_one_deadline_and_ends_with_a_cache_check() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let mut w1 = sup.inner().write().await;
        let t = Instant::now();
        let reader = { let sup = sup.clone(); tokio::spawn(async move { sup.observation().await }) };
        tokio::time::sleep(Duration::from_millis(20)).await;
        // a second writer queues behind the reader (tokio grants in queue order)
        let w2 = {
            let sup = sup.clone();
            tokio::spawn(async move {
                let mut w = sup.inner().write().await;
                let generation = sup.inner().generation();
                let mut obs = w.current_observation();
                obs.timestamp = 456.0;
                let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *w;
                let now = Instant::now();
                let _ = core.publish_displacing(generation, now, now + Duration::from_secs(5), obs, measured_positions, measured_velocities);
                // keeps the reader from the lock past its deadline
                tokio::time::sleep(OBS_LOCK_WAIT + Duration::from_millis(300)).await;
                drop(w);
            })
        };
        tokio::time::sleep(Duration::from_millis(280)).await;
        // a sample published while the reader waits, too old to answer: the reader retries
        let generation = sup.inner().generation();
        let mut stale = w1.current_observation();
        stale.timestamp = 123.0;
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *w1;
        let past = Instant::now() - Duration::from_millis(1);
        let _ = core.publish_displacing(generation, past, past, stale, measured_positions, measured_velocities);
        drop(w1);
        let got = tokio::time::timeout(Duration::from_secs(3), reader).await.expect("must not hang").unwrap();
        let elapsed = t.elapsed();
        assert_eq!(got.expect("the final cache check answers").timestamp, 456.0);
        assert!(elapsed < OBS_LOCK_WAIT + Duration::from_millis(150), "the budget restarted: {elapsed:?}");
        w2.await.unwrap();
    }

    /// A preempted reader holding the hub mutex: a request after a write neither waits for
    /// it nor holds the read guard while waiting for it, so ticks keep running.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_stuck_on_the_hub_does_not_block_the_fallback_or_the_tick() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        sup.set_temperatures(vec![50.0; 7]).await;
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let sup = sup.clone();
            std::thread::spawn(move || {
                let _g = sup.hub.reader_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let obs = tokio::time::timeout(Duration::from_secs(2), sup.observation()).await.expect("bounded").unwrap();
        assert_eq!(obs.temperatures, vec![50.0; 7]);
        tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("tick runs");
        release_tx.send(()).unwrap();
        holder.join().unwrap();
    }

    /// Recording boundaries: the saved frames are exactly the ticks between start and stop,
    /// with the values the 1.5 in-tick append recorded (the tick's observation timestamp,
    /// measured and commanded positions), also across ring drains.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn recording_holds_exactly_the_ticks_between_start_and_stop() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.tick(0.02).await;
        sup.start_recording(None, None).await.unwrap();
        let mut expected = Vec::new();
        for i in 0..300 {
            // move the commanded state so every frame differs
            sup.inner().write().await.commanded_positions[0] = i as f32 * 1e-3;
            let obs = sup.tick(0.02).await;
            expected.push(RecordFrame { timestamp: obs.timestamp, observation_state: obs.positions.clone(), action: obs.commanded_positions.clone() });
            if i % 100 == 0 { tokio::time::sleep(Duration::from_millis(5)).await; }
        }
        assert_eq!(sup.get_recording_state().await.1, 300);
        let (tx, rx) = std::sync::mpsc::channel();
        sup.save_recording_with(move |path, frames| {
            tx.send(frames.to_vec()).unwrap();
            crate::recording::write_episode_parquet(path, frames, None, &[])
        })
        .await
        .unwrap();
        // a tick after the stop fence records nothing
        sup.tick(0.02).await;
        let frames = rx.recv().unwrap();
        assert_eq!(frames.len(), expected.len());
        assert_eq!(frames, expected);
        assert_eq!(sup.inner().read().await.core.record_dropped(), 0);
    }

    /// The stop path takes the stop fence under the writer and collects the frames after
    /// releasing it: a dispatcher preempted inside the ring's consumer mutex delays the save,
    /// not the tick.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_stalled_ring_consumer_does_not_block_the_tick_during_a_stop() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.start_recording(None, None).await.unwrap();
        for _ in 0..5 { sup.tick(0.02).await; }
        let ring = sup.inner().read().await.core.record_ring().unwrap().clone();
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let ring = ring.clone();
            std::thread::spawn(move || {
                let _g = ring.consumer_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let saver = { let sup = sup.clone(); tokio::spawn(async move { sup.save_recording().await }) };
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(!saver.is_finished(), "the save waits for the consumer");
        tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("the tick must not wait for the stop");
        release_tx.send(()).unwrap();
        holder.join().unwrap();
        saver.await.unwrap().unwrap();
        let st = sup.episode_store().await;
        assert_eq!(st.list_episodes(None, Some(1), None, &[])[0].sidecar.frames, 5);
    }

    /// A save before the first recorded tick is refused and the session keeps recording
    /// into the ring (not into the recorder).
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_save_before_the_first_tick_keeps_the_session() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.start_recording(None, None).await.unwrap();
        assert!(sup.save_recording().await.is_err());
        {
            let inner = sup.inner().read().await;
            assert!(inner.core.recording_active() && inner.recording.is_some());
        }
        for _ in 0..3 { sup.tick(0.02).await; }
        {
            let inner = sup.inner().read().await;
            assert_eq!((inner.recorder.frame_count(), inner.core.record_pushed()), (0, 3));
        }
        sup.save_recording().await.unwrap();
    }

    /// A full ring drops samples without waiting and the session reports them.
    #[test]
    fn recording_ring_overflow_is_explicit() {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.recorder.start(Some("r".into()), Some(std::env::temp_dir().join("rt1_overflow.parquet"))).unwrap();
        inner.core.begin_recording();
        let n = crate::rt_core::record_ring::RECORD_RING_CAPACITY + 44;
        for _ in 0..n { inner.tick(0.02); }
        assert_eq!(inner.recording_frames(), crate::rt_core::record_ring::RECORD_RING_CAPACITY);
        let done = inner.core.finish_recording();
        assert_eq!((done.frames.len(), done.dropped), (crate::rt_core::record_ring::RECORD_RING_CAPACITY, 44));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn snapshot_expires_before_the_lease_or_feedback_freshness_flips() {
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        {
            let mut inner = sup.inner().write().await;
            let timeout = inner.lease_timeout;
            inner.active_lease = Some(ActiveLease {
                client_id: "c".into(),
                generation: 1,
                label: "x".into(),
                acquired_at: Instant::now() - timeout + Duration::from_millis(5),
                last_valid_motion_instant: None,
            });
        }
        let obs = sup.tick(0.02).await;
        assert_eq!(obs.lease_holder.as_deref(), Some("x"));
        tokio::time::sleep(Duration::from_millis(15)).await;
        // within two periods of the tick, but the lease has expired: recomputed
        assert_eq!(sup.get_observation().await.lease_holder, None);
        assert_eq!(sup.tick_stats().observation_cache.misses, 1);

        // An already-expired lease record does not collapse `valid_until` to `sampled` forever.
        {
            let inner = sup.inner().read().await;
            let mut obs = inner.current_observation();
            assert_eq!(obs.lease_holder, None);
            obs.feedback_age_ms.clear();
            let cap = Duration::from_secs(1);
            let (at, until) = inner.observation_valid_until(&obs, cap);
            assert_eq!(until - at, cap);
        }

        let mut inner = sup.inner().write().await;
        inner.active_lease = None;
        let mut obs = inner.current_observation();
        obs.feedback_age_ms = vec![Some(10), Some(240), None];
        let cap = Duration::from_secs(1);
        let (at, until) = inner.observation_valid_until(&obs, cap);
        assert_eq!(until - at, Duration::from_millis(9));
        assert_eq!(at, inner.start_time + Duration::from_secs_f64(obs.timestamp));
        obs.feedback_age_ms = vec![Some(300)];
        let (at, until) = inner.observation_valid_until(&obs, cap);
        assert_eq!(until - at, cap);
        // the deadline comes from the sample, not from when it is evaluated
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(inner.observation_valid_until(&obs, cap), (at, until));
    }

    #[tokio::test]
    async fn legacy_urgent_stop_advances_policy_gen_inside_write_lock_only_on_apply() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let conn = "connection:policy-race";
        let (gen, ep) = {
            let mut inner = sup.inner().write().await;
            let g = inner.acquire_lease(conn).unwrap();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            (g, inner.safety.epoch_id())
        };
        let policy_stop_gen = Arc::new(AtomicU64::new(0));
        let mk = |lg, e| crate::rt_core::urgent::UrgentStop {
            connection: crate::model::HolderLabel::new(conn),
            lease_generation: lg,
            epoch: Some(e),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        };
        // Refused stop (wrong epoch / wrong lease) must NOT advance stop_gen or invoke the hook.
        let r_refused = sup.urgent_stop_with_legacy_hook(mk(gen, ep + 1), |_, _| { policy_stop_gen.fetch_add(1, Ordering::SeqCst); }).await.unwrap();
        assert!(matches!(r_refused, crate::rt_core::urgent::UrgentStopOutcome::EpochMismatch { .. }));
        assert_eq!(policy_stop_gen.load(Ordering::SeqCst), 0);
        assert_eq!(sup.stop_generation(), 0);

        // Authorized stop must advance stop_gen BEFORE releasing `inner.write()`, so a
        // concurrent motion-start installer queued on `inner.write()` observes the bump immediately.
        let entered = Arc::new(tokio::sync::Notify::new());
        let release = Arc::new(tokio::sync::Notify::new());
        let (s1, p1, e1, r1) = (sup.clone(), policy_stop_gen.clone(), entered.clone(), release.clone());
        let stop_task = tokio::spawn(async move {
            s1.urgent_stop_with_legacy_hook(mk(gen, ep), |_, stop_gen| {
                p1.store(stop_gen, Ordering::SeqCst);
                e1.notify_one();
                // Yield briefly while still holding `inner.write()` so the installer task queues on the lock.
                std::thread::sleep(Duration::from_millis(15));
                r1.notify_one();
            }).await.unwrap()
        });
        entered.notified().await;
        // Now acquire `inner.write()` just like policy-load completion does: as soon as the lock
        // is granted, `stop_gen` MUST already be 1 (zero race window after Stop applies).
        let guard = sup.inner().write().await;
        assert_eq!(policy_stop_gen.load(Ordering::SeqCst), 1);
        assert_eq!(guard.current_stop_gen(), 1);
        drop(guard);
        let outcome = stop_task.await.unwrap();
        assert!(matches!(outcome, crate::rt_core::urgent::UrgentStopOutcome::Applied { stop_generation: 1, .. }));
    }

    #[test]
    fn terminal_broadcaster_never_skips_or_pops_on_full_fifo_or_failed_enqueue() {
        let tb = TerminalBroadcaster::default();
        tb.set_capacity(2);
        let accept = Arc::new(AtomicBool::new(false));
        let delivered = Arc::new(std::sync::Mutex::new(Vec::<u64>::new()));
        let accept_c = Arc::clone(&accept);
        let delivered_c = Arc::clone(&delivered);
        let sub_id = tb.subscribe(Box::new(move |obs| {
            if !accept_c.load(Ordering::SeqCst) {
                return false;
            }
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.id);
            }
            true
        }));
        let mut factory = SupervisorInner::new(ArmConfig::rebot_dm());
        let mut mk_done = || {
            let id = factory.begin_action("jog", None);
            factory.finish_action(id, "done", None, None, None, None, None);
            factory.last_action.clone().unwrap()
        };
        let a1 = mk_done();
        let a2 = mk_done();
        let a3 = mk_done();
        assert!(tb.can_start_action(false, false));
        assert!(tb.record_finished(&a1));
        assert!(tb.record_finished(&a2));
        // FIFO is now at capacity (2/2): NEW starts must be refused (`TERMINALS_BUSY`).
        assert!(!tb.can_start_action(false, false));
        assert!(!tb.can_start_action(false, true));
        // An already-running action finishing while FIFO is full MUST NOT be skipped/dropped.
        assert!(tb.record_finished(&a3));
        assert_eq!(tb.pending_total(), 3);
        assert!(!tb.can_start_action(false, false));

        // Sink rejects (outbound queue full): drain_all MUST NOT pop action 1 and MUST NOT remove
        // the subscriber.
        let base_obs = factory.current_observation();
        let _ = tb.drain_all(&base_obs, 0);
        assert_eq!(tb.pending_total(), 3, "failed enqueue must not pop terminal or drop subscriber");
        assert!(delivered.lock().unwrap().is_empty());

        // Once sink accepts again, all 3 terminals pop and deliver in exact FIFO order.
        accept.store(true, Ordering::SeqCst);
        let _ = tb.drain_all(&base_obs, 0);
        assert_eq!(tb.pending_total(), 0);
        assert_eq!(*delivered.lock().unwrap(), vec![a1.id, a2.id, a3.id]);
        assert!(tb.can_start_action(false, false));
        tb.unsubscribe(sub_id);
    }

    #[test]
    fn parse_linux_process_start_unix_ns_handles_valid_weird_comm_and_errors() {
        // 19 fields after `)` before field 22 (`starttime`), e.g. state='S', ppid..=num_threads(18) -> 19 tokens, index 19 is starttime=250.
        let proc_stat = "cpu 1 2 3\nbtime 1700000000\nprocesses 42\n";
        let self_stat = "1234 (armd (rt) v1.6) S 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 250 999 1000";
        // 1700000000 s + 250 / 100 s = 1700000002.5 s = 1700000002_500_000_000 ns
        assert_eq!(
            parse_linux_process_start_unix_ns(proc_stat, self_stat, 100),
            Some(1_700_000_002_500_000_000)
        );
        // Invalid clk_tck <= 0 -> None
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, self_stat, 0), None);
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, self_stat, -1), None);
        // Missing btime -> None
        assert_eq!(parse_linux_process_start_unix_ns("cpu 1 2 3\n", self_stat, 100), None);
        // Malformed / truncated /proc/self/stat -> None
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, "1234 (armd) S 1 2", 100), None);
        // Overflow -> None
        let overflow_stat = format!("btime {}\n", u64::MAX);
        assert_eq!(parse_linux_process_start_unix_ns(&overflow_stat, self_stat, 100), None);

        #[cfg(target_os = "linux")]
        {
            let id1 = init_process_identity();
            let id2 = init_process_identity();
            assert_eq!(id1, id2);
            assert!(id1.boot_id.as_deref().is_some_and(|s| !s.is_empty()));
            assert!(id1.process_start_unix_ns.is_some_and(|ns| ns > 0));
        }
    }


// scaffold-test-mod
mod stamp_namespace_tests {
    use super::*;

    /// 2026-11-27 real arm (rc7, executor): after cockpit jogging, grasp/Home were refused with
    /// STAMP_REUSED. Interleaved RT-internal (servo frame) and ordinary IDs from the shared
    /// counter fragmented one tracker; the capacity fold moved the watermark past every
    /// ordinary ID. Each namespace now has its own tracker.
    #[test]
    fn jog_frames_do_not_consume_future_ordinary_ids() {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        let mut ordinary_used = Vec::new();
        for i in 0..4000u32 {
            let internal = next_admission_request_id() | RT_INTERNAL_REQUEST_ID_BIT;
            assert!(!inner.is_stamp_consumed(internal));
            inner.record_consumed_stamp(internal);
            // an ordinary ID allocated between frames (most are never consumed here)
            let ordinary = next_admission_request_id();
            if i % 7 == 0 {
                inner.record_consumed_stamp(ordinary);
                ordinary_used.push(ordinary);
            }
        }
        let fresh = next_admission_request_id();
        assert!(!inner.is_stamp_consumed(fresh), "a fresh ordinary ID must stay admissible after jogging");
        let fresh_internal = next_admission_request_id() | RT_INTERNAL_REQUEST_ID_BIT;
        assert!(!inner.is_stamp_consumed(fresh_internal));
        // replay protection is kept in both namespaces
        assert!(ordinary_used.iter().all(|&id| inner.is_stamp_consumed(id)));
        inner.record_consumed_stamp(fresh_internal);
        assert!(inner.is_stamp_consumed(fresh_internal));
        inner.record_consumed_stamp(fresh);
        assert!(inner.is_stamp_consumed(fresh));
    }
}
