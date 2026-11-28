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


    #[test]
    fn drv_lock_release_helpers_positive_control_legacy_panics_exclusive_succeeds() {
        use crate::driver::{
            ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget,
            PosForceEmitOutcome, PosForceIntent, PosForceIntentKind, PosForceIntentStamp,
        };
        use crate::model::JointState;
        use crate::rt_core::executor::{ControlThreadGuard, ReleaseGuard};
        use crate::rt_core::ops::ServiceBudget;
        use crate::supervisor_lock::DriverSlot;
        use std::sync::atomic::AtomicUsize;

        struct ObservableReleaseDriver {
            dof: usize,
            rx_pos: f32,
            writes: Arc<AtomicUsize>,
            last_j0_cmd: Arc<std::sync::atomic::AtomicU32>,
            j7_epilogue_calls: Arc<AtomicUsize>,
        }

        impl ArmDriver for ObservableReleaseDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    enable_feedback: true,
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    ..DriverCapabilities::default()
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                let mut v = vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof];
                v[0].position = self.rx_pos;
                Ok(v)
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                _max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                out.clear();
                out.resize(self.dof, JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                out[0].position = self.rx_pos;
                Ok((self.dof as u16, false))
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.writes.fetch_add(1, Ordering::SeqCst);
                if let Some(t0) = targets.first() {
                    self.last_j0_cmd.store(t0.position.to_bits(), Ordering::SeqCst);
                }
                Ok(())
            }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> {
                Ok(())
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(pos_force);
        }
        impl crate::driver::PosForceExt for ObservableReleaseDriver {
            fn emit_pos_force_j7(
                &mut self,
                _intent: PosForceIntent,
                _current_stamp: PosForceIntentStamp,
            ) -> Result<PosForceEmitOutcome, ArmDriverError> {
                self.j7_epilogue_calls.fetch_add(1, Ordering::SeqCst);
                Ok(PosForceEmitOutcome::NotApplicable)
            }
        }

        let make_inner = |exclusive: bool| {
            let writes = Arc::new(AtomicUsize::new(0));
            let last_j0_cmd = Arc::new(std::sync::atomic::AtomicU32::new(0));
            let j7_epilogue_calls = Arc::new(AtomicUsize::new(0));
            let drv = ObservableReleaseDriver {
                dof: 7,
                rx_pos: 0.42,
                writes: Arc::clone(&writes),
                last_j0_cmd: Arc::clone(&last_j0_cmd),
                j7_epilogue_calls: Arc::clone(&j7_epilogue_calls),
            };
            let counter = Arc::new(AtomicU64::new(0));
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.rt_executor_owned = true;
            inner.safety.set_torque(true).unwrap();
            inner.driver_lock_attempts = Arc::clone(&counter);
            inner.driver = Some(if exclusive {
                DriverSlot::exclusive(Box::new(drv))
            } else {
                DriverSlot::legacy_with_counter(Box::new(drv), Arc::clone(&counter))
            });
            (inner, counter, writes, last_j0_cmd, j7_epilogue_calls)
        };

        // 1. Positive control: Legacy DriverSlot panics + increments `driver_lock_attempts` on
        //    EVERY one of the 4 release-path helpers under ControlThreadGuard + ReleaseGuard.
        {
            let (mut inner, counter, writes, _last_j0, j7_calls) = make_inner(false);
            let mut obs = inner.current_observation();
            let _cg = ControlThreadGuard::enter();
            let _rg = ReleaseGuard::enter();
            let now = Instant::now();
            let budget = ServiceBudget::for_release(now, now + Duration::from_millis(5));

            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = inner.poll_hardware_rx(Some(&budget), None, Some(now + Duration::from_millis(5)));
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 1, "poll_hardware_rx must count +1");

            let mut targets = vec![JointTarget::position(0.15); 7];
            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = inner.write_output_slice(&mut targets, 0.02, None);
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 2, "write_output_slice must count +1");
            assert_eq!(writes.load(Ordering::SeqCst), 0);

            let stamp = PosForceIntentStamp { stop_gen: 1, estop_epoch: 0, lease_gen: 0 };
            let mut tick_out = TickOutput::None;
            let mut allow_acc = true;
            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                inner.emit_j7_tick_epilogue(
                    false,
                    PosForceIntent { kind: PosForceIntentKind::Hold, stamp },
                    stamp,
                    &mut tick_out,
                    &mut allow_acc,
                );
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 3, "emit_j7_tick_epilogue must count +1");
            assert_eq!(j7_calls.load(Ordering::SeqCst), 0);

            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                inner.fill_observation(1.0, &mut obs);
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 4, "fill_observation must count +1");
        }

        // 2. Exclusive DriverSlot succeeds across ALL 4 release-path helpers under
        //    ControlThreadGuard + ReleaseGuard with 0 attempts AND observable side effects.
        {
            let (mut inner, counter, writes, last_j0, j7_calls) = make_inner(true);
            let _cg = ControlThreadGuard::enter();
            let _rg = ReleaseGuard::enter();
            let now = Instant::now();
            let budget = ServiceBudget::for_release(now, now + Duration::from_millis(5));

            let rx = inner.poll_hardware_rx(Some(&budget), None, Some(now + Duration::from_millis(5)));
            assert!(rx.active);
            assert!((inner.measured_positions[0] - 0.42).abs() < 1e-5, "poll_hardware_rx must update measured_positions");

            let mut targets = vec![JointTarget::position(0.15); 7];
            let out = inner.write_output_slice(&mut targets, 0.02, None);
            assert!(matches!(out, TickOutput::Complete { .. }), "write_output_slice must complete: {out:?}");
            assert_eq!(writes.load(Ordering::SeqCst), 1);
            assert!((f32::from_bits(last_j0.load(Ordering::SeqCst)) - 0.15).abs() < 1e-5);

            let stamp = PosForceIntentStamp { stop_gen: 1, estop_epoch: 0, lease_gen: 0 };
            let mut tick_out = TickOutput::None;
            let mut allow_acc = true;
            inner.emit_j7_tick_epilogue(
                false,
                PosForceIntent { kind: PosForceIntentKind::Hold, stamp },
                stamp,
                &mut tick_out,
                &mut allow_acc,
            );
            assert_eq!(j7_calls.load(Ordering::SeqCst), 1, "emit_j7_tick_epilogue must invoke driver");

            let mut obs = inner.current_observation();
            inner.fill_observation(1.0, &mut obs);
            assert!(obs.driver_capabilities.enable_feedback);
            assert!((obs.positions[0] - 0.42).abs() < 1e-5);
            assert_eq!(counter.load(Ordering::Acquire), 0, "Exclusive slot must record 0 lock attempts");
        }
    }

    #[test]
    fn drv_lock_major6_tx_cutoff_samples_owner_clock_at_driver_boundary() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::model::JointState;
        use crate::supervisor_lock::DriverSlot;
        use crate::transport::MockClock;

        struct DeadlineProbeDriver {
            dof: usize,
            clock: MockClock,
            installed_deadlines: Arc<std::sync::Mutex<Vec<Option<Instant>>>>,
        }
        impl ArmDriver for DeadlineProbeDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn clock_now(&self) -> Instant { self.clock.now() }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                self.installed_deadlines.lock().unwrap().push(deadline);
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(self.clock.now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
        }

        let clock = MockClock::new();
        let deadlines = Arc::new(std::sync::Mutex::new(Vec::new()));
        let drv = DeadlineProbeDriver {
            dof: 7,
            clock: clock.clone(),
            installed_deadlines: Arc::clone(&deadlines),
        };
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));

        let t0 = Instant::now();
        let rd = t0 + Duration::from_millis(10);
        // Simulate clock advancing past `rd` right at the driver-boundary sample.
        let mut calls = 0usize;
        let mut now_fn = || {
            calls += 1;
            if calls == 1 { t0 } else { rd + Duration::from_millis(5) }
        };
        let _ = inner.tick_control_inner(0.02, Some(rd), &mut now_fn);
        let captured = deadlines.lock().unwrap().clone();
        // DRV-F1 conservative driver-first cutoff translation: since call #2 (`owner_now`) is
        // already past `rd - margin`, expired cutoffs are rejected BEFORE staging any driver
        // deadline (zero `Some(_)` deadlines staged).
        assert!(
            captured.iter().all(Option::is_none),
            "expired RX/output cutoffs must reject before staging any driver deadline, got {captured:?}"
        );

        // Also verify `install_effective_tx_cutoff` directly when `owner_now` is past `rd`:
        // must return `DriverTxCapOutcome::Expired` with zero staged deadlines.
        deadlines.lock().unwrap().clear();
        let mut expired_now = || rd + Duration::from_millis(10);
        let (outcome, cutoff) = inner.install_effective_tx_cutoff(Some(rd), &mut expired_now);
        assert_eq!(outcome, DriverTxCapOutcome::Expired);
        assert!(cutoff.is_none());
        assert!(deadlines.lock().unwrap().is_empty());

        // When `owner_now` is still strictly before `rd - margin`, a live deadline IS staged.
        let mut live_now = || t0;
        let (live_outcome, live_cutoff) = inner.install_effective_tx_cutoff(Some(rd), &mut live_now);
        assert_eq!(live_outcome, DriverTxCapOutcome::Installed);
        let live_cutoff = live_cutoff.expect("live cutoff returned");
        assert!(live_cutoff.drv_release_cap > clock.now());
        assert_eq!(live_cutoff.owner_anchor, t0);
    }

    #[test]
    fn drv_lock_major7_gravity_comp_rechecks_disconnect_fence_after_enable_state() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::gravity_comp::{GravityCompPhase, GravityCompRun};
        use crate::model::{GravityCompPolicy, JointState};
        use crate::supervisor_lock::DriverSlot;

        #[derive(Clone, Copy)]
        enum InjectPoint {
            EnableState,
            SetOutputLeaseGen,
            None,
        }

        struct DisconnectInjectDriver {
            dof: usize,
            inject_at: InjectPoint,
            pending_disconnect_hold: Arc<AtomicU64>,
            writes: Arc<AtomicUsize>,
            hold_writes: Arc<AtomicUsize>,
            last_lease_gen: Arc<AtomicU64>,
        }
        impl ArmDriver for DisconnectInjectDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.writes.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn write_estop_hold_bounded(
                &mut self,
                targets: &[JointTarget],
            ) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
                self.hold_writes.fetch_add(1, Ordering::SeqCst);
                self.write_joint_targets_bounded(targets)
            }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState {
                if matches!(self.inject_at, InjectPoint::EnableState) {
                    self.pending_disconnect_hold.fetch_add(1, Ordering::SeqCst);
                }
                EnableState::Enabled
            }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(tx);
        }
        impl crate::driver::BoundedTxExt for DisconnectInjectDriver {
            fn set_output_lease_gen(&mut self, gen: u64) {
                self.last_lease_gen.store(gen, Ordering::SeqCst);
                if matches!(self.inject_at, InjectPoint::SetOutputLeaseGen) && gen != 0 {
                    self.pending_disconnect_hold.fetch_add(1, Ordering::SeqCst);
                }
            }
        }

        let make_run = |phase: GravityCompPhase, lease_generation: Option<u64>| GravityCompRun {
            phase,
            policy: GravityCompPolicy::rebot_dm(),
            elapsed_s: 1.0,
            scale: 1.0,
            damping_kd: [0.0; 6],
            entry_pose: [0.0; 6],
            entry_torque: [0.0; 6],
            tau_ff: [0.0; 6],
            kp: [0.0; 6],
            kd: [0.0; 6],
            position: [0.0; 6],
            overspeed_count: 0,
            prev_measured: Some([0.0; 6]),
            lease_generation,
            workspace_status: crate::gravity_comp::GravityCompWorkspaceStatus::Inside,
            workspace_cache_key: None,
        };

        // (a) Periodic output suppresses non-hold write when disconnect arrives in `enable_state()`
        //     OR in `set_output_lease_gen()`, and resets staged `output_lease_gen` to 0.
        for inject_at in [InjectPoint::EnableState, InjectPoint::SetOutputLeaseGen] {
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = true;
            let gen = inner.acquire_lease("c1").unwrap();
            assert!(gen > 0);
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = DisconnectInjectDriver {
                dof: 7,
                inject_at,
                pending_disconnect_hold: Arc::clone(&inner.pending_disconnect_hold),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(GravityCompPhase::Active, Some(gen)));

            let _ = inner.tick_control(0.02);
            assert_eq!(
                writes.load(Ordering::SeqCst),
                0,
                "periodic non-hold write must be suppressed"
            );
            assert!(
                inner.pending_disconnect_hold.load(Ordering::SeqCst) >= 1,
                "disconnect injection must have fired"
            );
            assert_eq!(
                last_lease_gen.load(Ordering::SeqCst),
                0,
                "staged output_lease_gen must be cleared to 0 on suppression"
            );
        }

        // (b) Immediate gravity-comp output suppresses non-hold writes on live disconnect in BOTH
        //     Active and Exiting (output_lease_gen == 0) phases, while preserving E-stop hold!
        for exiting in [false, true] {
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            inner.pending_disconnect_hold.store(1, Ordering::SeqCst);
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = DisconnectInjectDriver {
                dof: 7,
                inject_at: InjectPoint::None,
                pending_disconnect_hold: Arc::clone(&inner.pending_disconnect_hold),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(
                if exiting {
                    GravityCompPhase::Exiting {
                        hold: [0.0; 6],
                        elapsed_s: 0.0,
                    }
                } else {
                    GravityCompPhase::Active
                },
                None,
            ));

            let out_non_hold = inner.write_gravity_comp_immediate_inner(None, false, false);
            assert!(matches!(out_non_hold, TickOutput::None), "immediate non-hold write must be suppressed (exiting={exiting})");
            assert_eq!(writes.load(Ordering::SeqCst), 0);
            assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);

            let out_hold = inner.write_gravity_comp_immediate_inner(None, true, false);
            assert!(matches!(out_hold, TickOutput::Complete { .. }), "immediate E-stop hold must still transmit");
            assert_eq!(hold_writes.load(Ordering::SeqCst), 1);
        }

        // (c) Live E-stop latching during `enable_state()` OR during `set_output_lease_gen()` on
        //     BOTH immediate AND periodic gravity-comp switches to `write_estop_hold_bounded`,
        //     clears staged lease gen to 0, and sets immediate J7 intent to `Stop`.
        for (inject_in_setter, immediate) in [(false, true), (true, true), (true, false)] {
            struct EstopInjectDriver {
                dof: usize,
                inject_in_setter: bool,
                estop_state: Arc<crate::rt_core::urgent::EstopState>,
                writes: Arc<AtomicUsize>,
                hold_writes: Arc<AtomicUsize>,
                last_lease_gen: Arc<AtomicU64>,
            }
            impl ArmDriver for EstopInjectDriver {
                fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
                fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                    Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
                }
                fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                    self.writes.fetch_add(1, Ordering::SeqCst);
                    Ok(())
                }
                fn write_estop_hold_bounded(
                    &mut self,
                    _targets: &[JointTarget],
                ) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
                    self.hold_writes.fetch_add(1, Ordering::SeqCst);
                    Ok(crate::driver::TxBatchOutcome::Complete)
                }
                fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
                fn is_feedback_valid(&self) -> bool { true }
                fn is_enabled(&self) -> bool { true }
                fn enable_state(&self) -> EnableState {
                    if !self.inject_in_setter {
                        self.estop_state.request_and_latch();
                    }
                    EnableState::Enabled
                }
                fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
                fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
                fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
                crate::ext_accessors!(tx);
            }
            impl crate::driver::BoundedTxExt for EstopInjectDriver {
                fn set_output_lease_gen(&mut self, gen: u64) {
                    self.last_lease_gen.store(gen, Ordering::SeqCst);
                    if self.inject_in_setter && gen != 0 {
                        self.estop_state.request_and_latch();
                    }
                }
            }

            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = true;
            let gen = inner.acquire_lease("c1").unwrap();
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = EstopInjectDriver {
                dof: 7,
                inject_in_setter,
                estop_state: Arc::clone(&inner.estop_state),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(GravityCompPhase::Active, Some(gen)));

            if immediate {
                let out = inner.write_gravity_comp_immediate_inner(None, false, false);
                assert!(
                    matches!(out, TickOutput::Complete { is_hold: true, .. }),
                    "immediate write must switch to hold on live E-stop (inject_in_setter={inject_in_setter}): {out:?}"
                );
                assert_eq!(writes.load(Ordering::SeqCst), 0);
                assert_eq!(hold_writes.load(Ordering::SeqCst), 1);
                assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);
                assert!(matches!(
                    inner.j7_intent.map(|i| i.kind),
                    Some(crate::driver::PosForceIntentKind::Stop)
                ));
            } else {
                let _ = inner.tick_control(0.02);
                assert_eq!(writes.load(Ordering::SeqCst), 0);
                assert!(hold_writes.load(Ordering::SeqCst) >= 1);
                assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);
            }
        }
    }

    #[test]
    fn drv_lock_budgeted_clear_fault_clock_expiry_under_guard_refuses_without_tx() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::model::JointState;
        use crate::safety::{ClearFaultError, ClearRefusal, FaultCode, FaultReason};
        use crate::supervisor_lock::DriverSlot;
        use crate::transport::MockClock;

        struct ExpiryProbeDriver {
            dof: usize,
            clock: MockClock,
            hw_fault: bool,
            tx_probe_pending: bool,
            deadlines_staged: Arc<AtomicUsize>,
            tx_calls: Arc<AtomicUsize>,
        }
        impl ArmDriver for ExpiryProbeDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn clock_now(&self) -> Instant { self.clock.now() }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                if deadline.is_some() {
                    self.deadlines_staged.fetch_add(1, Ordering::SeqCst);
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
                self.tx_calls.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn has_hardware_fault(&self) -> bool { self.hw_fault }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { false }
            fn enable_state(&self) -> EnableState { EnableState::Disabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(self.clock.now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(tx);
        }
        impl crate::driver::BoundedTxExt for ExpiryProbeDriver {
            fn probe_tx_ready_budgeted(&mut self, _deadline: Instant) -> Result<bool, ArmDriverError> {
                self.tx_calls.fetch_add(1, Ordering::SeqCst);
                Ok(true)
            }
            fn tx_probe_pending(&self) -> bool { false }
            fn tx_probe_confirmed(&self) -> bool { !self.tx_probe_pending }
        }

        // Test BOTH budgeted TX probe and budgeted hardware reset when owner clock reaches
        // `tx_deadline` right after `drv.clock_now()` under the guard: must return `BudgetExpired`
        // with 0 staged deadlines, 0 TX calls, and preserved `scratch.ages` capacity.
        for (test_reset, overshoot_ms) in [(false, 0u64), (false, 5u64), (true, 0u64), (true, 5u64)] {
            let clock = MockClock::new();
            let staged = Arc::new(AtomicUsize::new(0));
            let tx_calls = Arc::new(AtomicUsize::new(0));
            let drv = ExpiryProbeDriver {
                dof: 7,
                clock: clock.clone(),
                hw_fault: test_reset,
                tx_probe_pending: !test_reset,
                deadlines_staged: Arc::clone(&staged),
                tx_calls: Arc::clone(&tx_calls),
            };
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.scratch.ages = Vec::with_capacity(16);
            let orig_cap = inner.scratch.ages.capacity();
            if test_reset {
                let _ = inner
                    .safety
                    .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hw fault".into()));
            } else {
                let _ = inner
                    .safety
                    .trigger_fault_coded(FaultCode::CanTxStalled, FaultReason::CanTxStalled);
            }

            let t0 = Instant::now();
            let release_deadline = t0 + Duration::from_millis(20);
            let tx_deadline = release_deadline - inner.tx_release_publish_margin();
            // First `now_fn()` call (pre-guard) is before `tx_deadline`; second `now_fn()` call
            // (inside guard after `drv.clock_now()`) is AT or PAST `tx_deadline`!
            let mut calls = 0usize;
            let mut live_now = || {
                calls += 1;
                if calls == 1 {
                    t0
                } else {
                    tx_deadline + Duration::from_millis(overshoot_ms)
                }
            };
            let res = inner.clear_fault_executor_bounded_for_release_with_clock(
                t0,
                release_deadline,
                None,
                &mut live_now,
            );
            assert!(
                matches!(res, Err(ClearFaultError::Rejected(ClearRefusal::BudgetExpired))),
                "expected BudgetExpired (test_reset={test_reset}, overshoot_ms={overshoot_ms}), got {res:?}"
            );
            assert_eq!(staged.load(Ordering::SeqCst), 0, "must not stage TX deadline after expiry");
            assert_eq!(tx_calls.load(Ordering::SeqCst), 0, "must not transmit after expiry");
            assert_eq!(inner.scratch.ages.capacity(), orig_cap, "scratch.ages must be restored on BudgetExpired");
        }
    }

    #[test]
    fn drv_lock_major8_park_disable_stop_gen_max_cancels_cleanly_on_estop_inhibited() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueProgress};
        use crate::model::JointState;
        use crate::rt_core::urgent::StopRequests;
        use crate::supervisor_lock::DriverSlot;

        struct StopGenLeadParkDriver {
            dof: usize,
            owner_stop_gen: Arc<AtomicU64>,
        }
        impl ArmDriver for StopGenLeadParkDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn begin_client_park_disable(
                &mut self,
                _max_rx_frames: Option<u16>,
                _expected_stop_gen: Option<u64>,
            ) -> Result<TorqueProgress, ArmDriverError> {
                // Advance owner-local `stop_gen` ahead of `StopRequests::current()` and fail
                // with `EstopInhibited` (modeling a local stop before any disable frame hit the wire).
                self.owner_stop_gen.fetch_add(1, Ordering::SeqCst);
                Err(ArmDriverError::EstopInhibited)
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
        }

        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        inner.commanded_velocities.fill(0.0);
        inner.attach_stop_requests(StopRequests::new());
        let drv = StopGenLeadParkDriver {
            dof: 7,
            owner_stop_gen: Arc::clone(&inner.stop_gen),
        };
        inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));

        inner.begin_park_disable_transition(false, None);
        assert!(
            inner.torque_transition.is_none(),
            "client park stopped via owner-local stop_gen must cancel cleanly without torque_transition"
        );
        assert!(
            !inner.safety.state().is_stopped(),
            "clean park stop-gate cancellation must not latch E-stop/stopped fault"
        );
    }

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
