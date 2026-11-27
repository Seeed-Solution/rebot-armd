//! `ZeroOffsetDriver` joint-mode handover with non-zero offsets allocates nothing (S4 PR 2,
//! Fable spec §4.4): `set_joint_modes_in_tick` runs in a release's TX pass (gravity-comp
//! POS_VEL handback), so the motor-space hold targets reuse the wrapper's buffer.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::{Arc, Mutex};

use armd::driver::{ArmDriver, ArmDriverError, JointMode, JointTarget};
use armd::joint_zero::ZeroOffsetDriver;
use armd::model::JointState;

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
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { bump(&ALLOCS); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { bump(&ALLOCS); unsafe { System.alloc_zeroed(l) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { bump(&FREES); unsafe { System.dealloc(p, l) } }
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

/// Records the motor-space hold positions it receives in a fixed array (no allocation).
struct HoldRecorder(Arc<Mutex<[f32; 7]>>);

impl ArmDriver for HoldRecorder {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { Ok(Vec::new()) }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    armd::ext_accessors!(joint_mode);
}

impl HoldRecorder {
    fn record(&self, hold: &[JointTarget]) {
        let mut g = self.0.lock().unwrap();
        for (slot, t) in g.iter_mut().zip(hold) {
            *slot = t.position;
        }
    }
}

impl armd::driver::JointModeExt for HoldRecorder {
    fn supports_mit_mode_switch(&self) -> bool { true }
    fn set_joint_modes(&mut self, _: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.record(hold);
        Ok(())
    }
    fn set_joint_modes_in_tick(&mut self, _: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.record(hold);
        Ok(())
    }
    fn set_joint_modes_confirmed(&mut self, _: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.record(hold);
        Ok(())
    }
}

const OFFSETS: [f32; 7] = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];

#[test]
fn set_joint_modes_with_nonzero_offsets_allocates_nothing() {
    let seen = Arc::new(Mutex::new([0.0f32; 7]));
    let mut wrapped: Box<dyn ArmDriver> =
        Box::new(ZeroOffsetDriver::new(Box::new(HoldRecorder(seen.clone())), &OFFSETS));
    let hold: [JointTarget; 7] = std::array::from_fn(|i| JointTarget::position(i as f32 * 0.01));
    let modes = [JointMode::Position; 7];
    let want: [f32; 7] = std::array::from_fn(|i| i as f32 * 0.01 + OFFSETS[i]);

    type Call = fn(&mut dyn ArmDriver, &[JointMode], &[JointTarget]) -> Result<(), ArmDriverError>;
    let calls: [(&str, Call); 3] = [
        ("set_joint_modes_in_tick", |d, m, h| d.joint_mode_mut().set_joint_modes_in_tick(m, h)),
        ("set_joint_modes", |d, m, h| d.joint_mode_mut().set_joint_modes(m, h)),
        ("set_joint_modes_confirmed", |d, m, h| d.joint_mode_mut().set_joint_modes_confirmed(m, h)),
    ];
    for (name, call) in calls {
        call(&mut *wrapped, &modes, &hold).unwrap(); // warm-up
        *seen.lock().unwrap() = [0.0; 7];
        let (r, allocs, frees) = counted(|| call(&mut *wrapped, &modes, &hold));
        r.unwrap();
        assert_eq!((allocs, frees), (0, 0), "{name} with non-zero offsets touched the allocator");
        assert_eq!(*seen.lock().unwrap(), want, "{name}: inner driver must see motor = joint + offset");
    }
}
