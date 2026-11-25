//! Zero-allocation proof for legacy control releases (`SupervisorInner::tick_control`) while
//! servicing ticked legacy WS torque on/off transitions, `RangeCheck` recovery, Stop
//! cancellation, refusal, and failure settling, plus `HoldIoScope` instrumentation.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::diagnostics::tick::{
    record_hold_rx_ns, record_hold_sleep_ns, record_hold_tx_ns, HoldIoScope,
};
use armd::driver::{ArmDriver, DamiaoArmDriver, RangeCheckStatus};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, ArmSupervisor};

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

// SAFETY: delegates directly to System; thread-local counters are const-initialized.
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

#[derive(Clone, Copy)]
struct Motor {
    id: u16,
    q: f32,
    enabled: bool,
    silent: bool,
    drop_enable: bool,
    silent_disable: bool,
    limits: DamiaoLimits,
}

impl Motor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
        let data = [
            (state << 4) | (self.id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct BusState {
    motors: [Motor; 7],
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    clock: Arc<MockClock>,
}

impl BusState {
    fn push_all_feedback(&mut self) {
        let now = self.clock.now();
        for m in &self.motors {
            if !m.silent {
                self.rx.push_back((m.feedback_frame(), Some(now)));
            }
        }
    }
}

#[derive(Clone)]
struct ZeroAllocLegacyBus(Arc<Mutex<BusState>>);

impl ZeroAllocLegacyBus {
    fn new(clock: Arc<MockClock>) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| Motor {
            id: (i + 1) as u16,
            q: 0.0,
            enabled: false,
            silent: false,
            drop_enable: false,
            silent_disable: false,
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(BusState {
            motors,
            rx: VecDeque::with_capacity(256),
            clock,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut BusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for ZeroAllocLegacyBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone() as Arc<dyn DriverClock>)
    }
    fn supports_bounded_flush(&self) -> bool {
        true
    }
    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        let now = s.clock.now();
        if frame.id == 0x7FF {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            let Some(idx) = s.motors.iter().position(|m| m.id == id) else {
                return Ok(());
            };
            if s.motors[idx].silent {
                return Ok(());
            }
            match op {
                0xCC => {
                    if !s.motors[idx].silent_disable {
                        let fb = s.motors[idx].feedback_frame();
                        s.rx.push_back((fb, Some(now)));
                    }
                }
                damiao::REG_READ => {
                    let m = s.motors[idx];
                    let val: Option<[u8; 4]> = match rid {
                        damiao::RID_CTRL_MODE => Some(damiao::CTRL_MODE_POS_VEL.to_le_bytes()),
                        damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                        damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                        damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                        _ => None,
                    };
                    if let Some(v) = val {
                        let [lo, hi] = id.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + id,
                            &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                        )
                        .unwrap();
                        s.rx.push_back((reply, Some(now)));
                    }
                }
                _ => {}
            }
            return Ok(());
        }
        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if s.motors[idx].silent {
                return Ok(());
            }
            if frame.data == damiao::CMD_ENABLE {
                if !s.motors[idx].drop_enable {
                    s.motors[idx].enabled = true;
                    let fb = s.motors[idx].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
            } else if frame.data == damiao::CMD_DISABLE {
                if !s.motors[idx].silent_disable {
                    s.motors[idx].enabled = false;
                    let fb = s.motors[idx].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
            } else {
                let fb = s.motors[idx].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
            return Ok(());
        }
        if let Some(idx) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            if !s.motors[idx].silent {
                let fb = s.motors[idx].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        self.0
            .lock()
            .unwrap()
            .rx
            .pop_front()
            .ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

async fn setup_sup(
    unverified_j2: bool,
) -> (
    Arc<ArmSupervisor>,
    ZeroAllocLegacyBus,
    Arc<MockClock>,
    armd::rt_core::requests::lease::ConnectionHandle,
    u64,
) {
    let clock = Arc::new(MockClock::new());
    let bus = ZeroAllocLegacyBus::new(clock.clone());
    if unverified_j2 {
        bus.with(|s| s.motors[1].silent = true);
    }
    bus.with(|s| s.push_all_feedback());
    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let _ = dm.read_joint_states();
    let rc = dm.check_mapping_ranges();
    if unverified_j2 {
        assert_eq!(rc.status, RangeCheckStatus::Unverified);
    } else {
        assert_eq!(rc.status, RangeCheckStatus::Passed);
    }
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm)),
    );
    let conn = sup.try_claim_ws_slot().unwrap();
    sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
    let gen = {
        let mut inner = sup.inner().write().await;
        let gen = inner
            .acquire_lease(&format!("connection:{}", conn.id()))
            .unwrap();
        // Warm up steady-state scratch buffers before measuring zero-alloc releases.
        inner.tick_control(0.02);
        inner.tick_control(0.02);
        gen
    };
    (sup, bus, clock, conn, gen)
}

fn spawn_torque_ws(
    sup: &Arc<ArmSupervisor>,
    conn: armd::rt_core::requests::lease::ConnectionHandle,
    gen: u64,
    epoch: u64,
    req_id: u64,
    enable: bool,
) -> tokio::task::JoinHandle<Result<TorqueWsOutcome, armd::ArmError>> {
    let sup = sup.clone();
    tokio::spawn(async move {
        let stamp = sup.inner().read().await.current_admission_stamp();
        sup.torque_ws(
            conn,
            gen,
            Some(epoch),
            true,
            req_id,
            Ok(enable),
            OwnerAuthPolicy::None,
            OperatorGrant::unenforced(),
            stamp,
        )
        .await
    })
}

async fn wait_transition_pending(sup: &Arc<ArmSupervisor>) {
    let dl = Instant::now() + Duration::from_secs(2);
    while Instant::now() < dl {
        if sup.inner().read().await.torque_transition_pending() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    panic!("legacy torque transition did not become pending");
}

#[test]
fn hold_io_scope_is_strictly_zero_alloc() {
    let (timing, allocs, frees) = counted(|| {
        let scope = HoldIoScope::enter();
        record_hold_tx_ns(12_000);
        record_hold_rx_ns(8_000);
        record_hold_sleep_ns(500_000);
        scope.finish()
    });
    assert_eq!((allocs, frees), (0, 0), "HoldIoScope must not allocate or free");
    assert_eq!(timing.tx_us, 12);
    assert_eq!(timing.rx_us, 8);
    assert_eq!(timing.sleep_us, 500);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_control_release_zero_alloc_across_enable_disable_rangecheck_stop_and_failure() {
    // 1. RangeCheck recovery + enable pending + enable completion + disable pending + disable completion.
    {
        let (sup, bus, clock, conn, gen) = setup_sup(true).await;
        bus.with(|s| {
            s.motors[1].silent = false;
            s.motors[2].drop_enable = true;
            s.push_all_feedback();
        });
        // Warm up after motor 2 returns.
        {
            let mut inner = sup.inner().write().await;
            clock.advance(Duration::from_millis(20));
            inner.tick_control(0.02);
        }
        let ep = sup.inner().read().await.safety.epoch_id();
        let enable_task = spawn_torque_ws(&sup, conn, gen, ep, 1, true);
        wait_transition_pending(&sup).await;

        // Tick through RangeCheck + pending enable while motor 3 drops enable.
        for _ in 0..4 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "legacy enable/rangecheck pending control release allocated ({allocs}, {frees})"
            );
        }

        // Let motor 3 confirm; next ticks complete enable and settle LegacyTorqueSlot.
        bus.with(|s| s.motors[2].drop_enable = false);
        for _ in 0..8 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let pending_before = inner.torque_transition_pending();
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "legacy enable completion control release allocated ({allocs}, {frees})"
            );
            if pending_before && !inner.torque_transition_pending() {
                break;
            }
        }
        let res = enable_task.await.unwrap().unwrap();
        assert!(
            matches!(res, TorqueWsOutcome::Accepted { is_torqued: true, .. }),
            "expected Accepted enable: {res:?}"
        );

        // Steady-state rest ticks + install heap-backed `servo_ws_min_depths` and verify rearm
        // does NOT free the heap-backed workspace buffer on the control release (Major 5).
        for _ in 0..4 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!((allocs, frees), (0, 0));
        }
        {
            let mut inner = sup.inner().write().await;
            inner.servo_ws_min_depths = Some((1, armd::kinematics::SlotBuf::Heap(vec![0.1; 300])));
        }
        let ep_rearm = sup.inner().read().await.safety.epoch_id();
        let rearm_task = spawn_torque_ws(&sup, conn, gen, ep_rearm, 11, true);
        // If rearm went Pending, tick to completion with 0 allocs/frees.
        tokio::time::sleep(Duration::from_millis(5)).await;
        for _ in 0..4 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "already-torqued rearm with heap-backed servo_ws_min_depths allocated/freed ({allocs}, {frees})"
            );
            if !inner.torque_transition_pending() {
                break;
            }
        }
        let res_rearm = rearm_task.await.unwrap().unwrap();
        assert!(
            matches!(res_rearm, TorqueWsOutcome::Accepted { is_torqued: true, .. }),
            "expected Accepted rearm: {res_rearm:?}"
        );

        // Disable while motor 5 is briefly silent, then confirms -> zero alloc on pending & completion.
        bus.with(|s| s.motors[4].silent_disable = true);
        let ep2 = sup.inner().read().await.safety.epoch_id();
        let disable_task = spawn_torque_ws(&sup, conn, gen, ep2, 2, false);
        wait_transition_pending(&sup).await;

        for _ in 0..2 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "legacy disable pending control release allocated ({allocs}, {frees})"
            );
        }
        bus.with(|s| s.motors[4].silent_disable = false);
        for _ in 0..8 {
            clock.advance(Duration::from_millis(20));
            let mut inner = sup.inner().write().await;
            let pending_before = inner.torque_transition_pending();
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "legacy disable completion control release allocated ({allocs}, {frees})"
            );
            if pending_before && !inner.torque_transition_pending() {
                break;
            }
        }
        let res_off = disable_task.await.unwrap().unwrap();
        assert!(
            matches!(res_off, TorqueWsOutcome::Accepted { is_torqued: false, .. }),
            "expected Accepted disable: {res_off:?}"
        );
    }

    // 2. Stop cancellation during pending legacy enable -> zero alloc on control release.
    {
        let (sup, bus, clock, conn, gen) = setup_sup(false).await;
        bus.with(|s| s.motors[3].drop_enable = true);
        let ep = sup.inner().read().await.safety.epoch_id();
        let enable_task = spawn_torque_ws(&sup, conn, gen, ep, 3, true);
        wait_transition_pending(&sup).await;

        {
            let mut inner = sup.inner().write().await;
            let (_, allocs, frees) = counted(|| inner.authorized_stop());
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "authorized_stop cancelling legacy enable allocated ({allocs}, {frees})"
            );
            clock.advance(Duration::from_millis(20));
            let (_, allocs2, frees2) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs2, frees2),
                (0, 0),
                "post-stop control release allocated ({allocs2}, {frees2})"
            );
        }
        let res = enable_task.await.unwrap().unwrap();
        assert!(
            matches!(res, TorqueWsOutcome::Rejected { ref error, .. } if error.code.as_deref() == Some("STOPPED")),
            "expected STOPPED rejection: {res:?}"
        );
    }

    // 3. Enable failure timeout (`EnableFailed` -> `FailedHold`) -> zero alloc on control release.
    {
        let (sup, bus, clock, conn, gen) = setup_sup(false).await;
        bus.with(|s| s.motors[3].drop_enable = true);
        let ep = sup.inner().read().await.safety.epoch_id();
        let enable_task = spawn_torque_ws(&sup, conn, gen, ep, 4, true);
        wait_transition_pending(&sup).await;

        let mut finished = false;
        for _ in 0..30 {
            clock.advance(Duration::from_millis(25));
            let mut inner = sup.inner().write().await;
            let pending_before = inner.torque_transition_pending();
            let (_, allocs, frees) = counted(|| inner.tick_control(0.02));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "legacy enable timeout/failure control release allocated ({allocs}, {frees})"
            );
            if pending_before && !inner.torque_transition_pending() {
                finished = true;
                break;
            }
        }
        assert!(finished, "expected enable timeout to settle within 30 ticks");
        let res = enable_task.await.unwrap().unwrap();
        assert!(
            matches!(res, TorqueWsOutcome::Rejected { ref error, .. } if error.code.as_deref() == Some("CLEAR_FAULT_REJECTED")),
            "expected legacy CLEAR_FAULT_REJECTED wire rejection (with EnableFailed fault latched): {res:?}"
        );
        assert!(
            sup.inner().read().await.safety.has_latched_fault_code(armd::FaultCode::EnableFailed),
            "EnableFailed fault code must remain latched"
        );
    }

    // 4. Production `tick_shared` (`control_tick`, `keep = false`) executes with strictly
    //    (0 allocs, 0 frees) measured by `CountingAlloc` across steady-state AND legacy WS
    //    torque enable ticks (R5 Major 7).
    {
        let (sup, bus, clock, conn, gen) = setup_sup(false).await;
        // Prewarm all triple-buffer publication slots (matching `test_rt_tick_alloc::warm`).
        for _ in 0..4 {
            clock.advance(Duration::from_millis(20));
            sup.control_tick(0.02).await;
            let _ = sup.get_observation().await;
        }
        bus.with(|s| s.motors[2].drop_enable = true);
        let ep = sup.inner().read().await.safety.epoch_id();
        let enable_task = spawn_torque_ws(&sup, conn, gen, ep, 5, true);
        wait_transition_pending(&sup).await;
        for step in 0..8 {
            if step == 2 {
                bus.with(|s| s.motors[2].drop_enable = false);
            }
            clock.advance(Duration::from_millis(25));
            ALLOCS.with(|c| c.set(0));
            FREES.with(|c| c.set(0));
            WATCH.with(|w| w.set(true));
            sup.control_tick(0.02).await;
            WATCH.with(|w| w.set(false));
            let (allocs, frees) = (ALLOCS.with(Cell::get), FREES.with(Cell::get));
            assert_eq!(
                (allocs, frees),
                (0, 0),
                "production tick_shared step {step} allocated ({allocs}, {frees})"
            );
            if !sup.inner().read().await.torque_transition_pending() {
                break;
            }
        }
        let res = enable_task.await.unwrap().unwrap();
        assert!(
            matches!(res, TorqueWsOutcome::Accepted { is_torqued: true, .. }),
            "expected Accepted enable via tick_shared: {res:?}"
        );
    }
}
