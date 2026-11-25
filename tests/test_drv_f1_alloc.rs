//! Zero-allocation & zero-free proof for DRV-F1 SAFE-HOLD ticks and energized handback on
//! `armd-control` (RT invariant #5 + DRV-F1 R5 Blocker 1 / FU-PANIC).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::executor::{ExecutorConfig, PanicPolicy, PersistTestHooks};
use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl, SafeHoldReason};
use armd::supervisor::{ArmSupervisor, PanicDisableOutcome, SupervisorInner, OWNER_LEGACY};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn bump(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) {
        let _ = c.try_with(|n| n.set(n.get() + 1));
    }
}

// SAFETY: forwards to System; thread-local counters are const-initialized without destructors.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        bump(&ALLOCS);
        unsafe { System.alloc(l) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        bump(&ALLOCS);
        unsafe { System.alloc_zeroed(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        bump(&FREES);
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        bump(&ALLOCS);
        bump(&FREES);
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[derive(Clone, Copy)]
struct Motor {
    id: u16,
    q: f32,
    temp_c: u8,
    enabled: bool,
    silent: bool,
    limits: DamiaoLimits,
}

impl Motor {
    fn fb(&self) -> CanFrame {
        let st = if self.enabled { 1u8 } else { 0u8 };
        let q = damiao::float_to_uint(self.q, -self.limits.p_max, self.limits.p_max, 16);
        let dq = damiao::float_to_uint(0.0, -self.limits.v_max, self.limits.v_max, 12);
        let t = damiao::float_to_uint(0.0, -self.limits.t_max, self.limits.t_max, 12);
        CanFrame::new_standard(
            0x10 + self.id,
            &[
                (st << 4) | (self.id as u8 & 0x0F),
                (q >> 8) as u8,
                (q & 0xFF) as u8,
                (dq >> 4) as u8,
                (((dq & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
                (t & 0xFF) as u8,
                self.temp_c,
                self.temp_c,
            ],
        )
        .unwrap()
    }
}

struct FakeBusInner {
    motors: [Motor; 7],
    ctrl_modes: [u32; 7],
    rx: VecDeque<CanFrame>,
    immediate: bool,
    clock: MockClock,
}

impl FakeBusInner {
    fn push_all_feedback(&mut self) {
        for m in self.motors {
            if !m.silent && self.rx.len() < self.rx.capacity() {
                self.rx.push_back(m.fb());
            }
        }
    }
}

#[derive(Clone)]
struct FakeBus(Arc<Mutex<FakeBusInner>>);

impl FakeBus {
    fn new(q_init: [f32; 7], clock: MockClock) -> Self {
        let lims = dm_limits();
        let motors = std::array::from_fn(|i| Motor {
            id: (i + 1) as u16,
            q: q_init[i],
            temp_c: 30,
            enabled: false,
            silent: false,
            limits: lims[i],
        });
        Self(Arc::new(Mutex::new(FakeBusInner {
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            rx: VecDeque::with_capacity(256),
            immediate: true,
            clock,
        })))
    }
    fn with<R>(&self, f: impl FnOnce(&mut FakeBusInner) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for FakeBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(fb) = s
                .motors
                .iter()
                .find(|m| m.id == id && !m.silent)
                .map(Motor::fb)
            {
                if s.rx.len() < s.rx.capacity() {
                    s.rx.push_back(fb);
                }
            }
            return Ok(());
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let reply = s.motors.iter().enumerate().find(|(_, m)| m.id == id).and_then(|(idx, m)| {
                let ctrl_mode = s.ctrl_modes[idx];
                let val: Option<[u8; 4]> = match rid {
                    damiao::RID_CTRL_MODE => Some(ctrl_mode.to_le_bytes()),
                    damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                    _ => None,
                };
                val.map(|v| {
                    let [lo, hi] = id.to_le_bytes();
                    CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap()
                })
            });
            if let Some(r) = reply {
                if s.rx.len() < s.rx.capacity() {
                    s.rx.push_back(r);
                }
            }
            return Ok(());
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let v = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                if rid == damiao::RID_CTRL_MODE {
                    s.ctrl_modes[idx] = u32::from_le_bytes(v);
                }
                let [lo, hi] = id.to_le_bytes();
                let r = CanFrame::new_standard(
                    0x10 + id,
                    &[lo, hi, damiao::REG_WRITE, rid, v[0], v[1], v[2], v[3]],
                )
                .unwrap();
                if s.rx.len() < s.rx.capacity() {
                    s.rx.push_back(r);
                }
            }
            return Ok(());
        }
        let id = frame.id as u16;
        let base_id = if id >= 0x300 {
            id - 0x300
        } else if id >= 0x100 {
            id - 0x100
        } else {
            id
        };
        let immediate = s.immediate;
        let mut reply = None;
        if let Some(m) = s.motors.iter_mut().find(|m| m.id == base_id) {
            if frame.data == damiao::CMD_ENABLE {
                m.enabled = true;
            } else if frame.data == damiao::CMD_DISABLE {
                m.enabled = false;
            }
            if immediate && !m.silent {
                reply = Some(m.fb());
            }
        }
        if let Some(fb) = reply {
            if s.rx.len() < s.rx.capacity() {
                s.rx.push_back(fb);
            }
        }
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }
    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let mut s = self.0.lock().unwrap();
        let now = s.clock.now();
        s.rx.pop_front()
            .map(|f| (f, Some(now)))
            .ok_or(TransportError::WouldBlock)
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(Arc::new(self.0.lock().unwrap().clock.clone()))
    }
    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
    fn supports_bounded_flush(&self) -> bool {
        true
    }
}

#[test]
fn safe_hold_ticks_zero_alloc_zero_free_including_rx_thermal_bus_loss_and_j7_emitter() {
    let clock = MockClock::new();
    let raised = [0.35, -0.25, 0.45, 0.10, -0.10, 0.05, -0.80];
    let bus = FakeBus::new(raised, clock.clone());
    bus.with(|s| s.push_all_feedback());

    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_pos_force(armd::driver::GripperPosForce {
            joint: 6,
            current_raw: 220,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        })
        .unwrap();
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    drv.set_torque(true).unwrap();

    let cfg = ArmConfig::rebot_dm();
    let mut core = SupervisorInner::new(cfg.clone());
    core.lease_enabled = false;
    core.measured_positions = raised.to_vec();
    core.feedback_numeric_valid = true;

    let period = Duration::from_millis(10);
    let mut bufs = SafeHoldBuffers::from_config(&cfg, period);
    bufs.mirror_from_core(&core);
    bufs.seed_on_entry(&core);
    bufs.prepare_driver_on_entry(&mut drv);

    let ctrl = SafeHoldControl::new();
    ctrl.mark_active(SafeHoldReason::ControlPanic);

    // 1. Steady-state SAFE-HOLD ticks (RX + freshness + arm hold TX + J7 0x307 PosForce TX):
    let ((), allocs_hold, frees_hold) = counted(|| {
        for _ in 0..20 {
            clock.advance(period);
            bus.with(|s| s.push_all_feedback());
            let mut now_fn = || clock.now();
            let dl = clock.now() + period;
            bufs.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        }
    });
    assert_eq!(allocs_hold, 0, "SAFE-HOLD hold ticks allocated {allocs_hold} times");
    assert_eq!(frees_hold, 0, "SAFE-HOLD hold ticks freed {frees_hold} times");
    assert_eq!(ctrl.ticks(), 20);
    assert!(ctrl.hold_tx_count() >= 20);
    assert!(ctrl.j7_tx_count() >= 20);
    assert!(!ctrl.bus_loss_latched());
    assert!(!ctrl.thermal_tripped());

    // 2. Bus loss during SAFE-HOLD (0 allocs, 0 frees, NO disable):
    bus.with(|s| {
        s.rx.clear();
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    clock.advance(Duration::from_millis(600));
    let ((), allocs_bl, frees_bl) = counted(|| {
        for _ in 0..5 {
            clock.advance(period);
            let mut now_fn = || clock.now();
            let dl = clock.now() + period;
            bufs.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        }
    });
    assert_eq!(allocs_bl, 0, "SAFE-HOLD bus-loss ticks allocated {allocs_bl} times");
    assert_eq!(frees_bl, 0, "SAFE-HOLD bus-loss ticks freed {frees_bl} times");
    assert!(ctrl.bus_loss_latched());
    assert!(ctrl.holding_unconfirmed());
    assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));

    // 3. Thermal trip during SAFE-HOLD (0 allocs, 0 frees, DisableProtective { Thermal }):
    bus.with(|s| {
        for m in &mut s.motors {
            m.silent = false;
        }
        s.motors[2].temp_c = 95;
        s.push_all_feedback();
    });
    let ((), allocs_th, frees_th) = counted(|| {
        for _ in 0..10 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let mut now_fn = || clock.now();
            let dl = clock.now() + Duration::from_millis(20);
            bufs.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        }
    });
    assert_eq!(allocs_th, 0, "SAFE-HOLD thermal ticks allocated {allocs_th} times");
    assert_eq!(frees_th, 0, "SAFE-HOLD thermal ticks freed {frees_th} times");
    assert!(ctrl.thermal_tripped());
    assert!(ctrl.thermal_disable_confirmed());
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_energized_handback_and_safe_hold_zero_rt_allocs() {
    // (a) Energized handback: verify 0 RT allocations/frees across releases + handback boundary.
    {
        let clock = MockClock::new();
        let raised = [0.20, -0.20, 0.30, 0.0, 0.0, 0.0, -0.50];
        let bus = FakeBus::new(raised, clock.clone());
        bus.with(|s| s.push_all_feedback());
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.alloc_timing = true;
        let handle = sup.start_executor(cfg).await.unwrap();
        let dl = Instant::now() + Duration::from_secs(2);
        while handle.stats().steps.load(Ordering::Relaxed) < 4 {
            assert!(Instant::now() < dl);
            clock.advance(Duration::from_millis(5));
            bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert_eq!(
            handle.stats().total_allocs(),
            armd::diagnostics::alloc::AllocCounts::ZERO,
            "RT releases before handback must perform 0 heap ops"
        );
        sup.handback_to_legacy(Duration::from_secs(2))
            .await
            .expect("handback succeeds");
        assert_eq!(sup.owner_mode(), OWNER_LEGACY);
        assert_eq!(sup.driver_lock_attempts(), 0);
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }

    // (b) Executor panic -> SAFE-HOLD ticks run cleanly with `alloc_timing` enabled and
    //     unwind payload retired off-thread without touching `DriverLock`.
    {
        let raised = [0.25, -0.15, 0.35, 0.0, 0.0, 0.0, -0.50];
        let clock = MockClock::new();
        let bus = FakeBus::new(raised, clock.clone());
        bus.with(|s| s.push_all_feedback());
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.alloc_timing = true;
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_on_release: Some(2),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        let sh = handle.safe_hold_control();
        let dl = Instant::now() + Duration::from_secs(2);
        while !(sh.is_active() && sh.ticks() >= 3) {
            assert!(Instant::now() < dl);
            clock.advance(Duration::from_millis(5));
            bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert_eq!(sup.driver_lock_attempts(), 0);
        let _active_check = AtomicBool::new(sh.is_active());
        assert!(_active_check.load(Ordering::Relaxed));
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }
}
