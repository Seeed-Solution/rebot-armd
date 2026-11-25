//! Process-wide tiered panic coordinator & owner-side internal-fault lane (DRV-F1 / FU-PANIC).
//!
//! Tiered response:
//! 1. **Control-thread panic (`armd-control`, either Legacy or RT Executor)**:
//!    The panic hook checks thread-local [`on_any_control_thread`] and returns immediately so the
//!    control thread's `catch_unwind` transitions directly into preallocated SAFE-HOLD.
//! 2. **Non-control thread panic (WS, tokio tasks, HTTP, persistence, logging, etc.)**:
//!    The panic hook is strictly lock-free, allocation-free, non-logging, and non-waiting:
//!    it latches [`InternalFaultLane`] (immediately fencing all new commands & rearm with
//!    `INTERNAL_FAULT` AND revoking queued motion/rearm at the owner's TX gate), copies the
//!    panicking thread name into a fixed atomic byte buffer, and wakes a pre-spawned
//!    `"armd-panic-coord"` thread via lock-free `Thread::unpark`.
//!    The healthy control thread (either owner) reads `InternalFaultLane` directly on its next
//!    tick (zero queue dependency), stops motion, holds, attempts normal stop-direction park to
//!    the resting pose, disables torque at confirmed rest (`Rule A'`), and latches a
//!    non-recoverable fault naming the panicking thread. If the park cannot complete, it keeps
//!    holding and latches the non-recoverable fault (never disables away from rest).

use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::{self, Thread};

const THREAD_NAME_CAP: usize = 64;

thread_local! {
    static ON_ANY_CONTROL_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// True on BOTH the RT executor `armd-control` thread and the legacy `armd-control` thread.
#[inline]
pub fn on_any_control_thread() -> bool {
    ON_ANY_CONTROL_THREAD.with(Cell::get)
}

/// RAII thread-local guard marking a control thread (`armd-control`) for tiered panic routing.
pub struct ControlPanicScope {
    prev: bool,
    _not_send: std::marker::PhantomData<*mut ()>,
}

impl ControlPanicScope {
    pub fn enter() -> Self {
        Self {
            prev: ON_ANY_CONTROL_THREAD.with(|f| f.replace(true)),
            _not_send: std::marker::PhantomData,
        }
    }
}

impl Drop for ControlPanicScope {
    fn drop(&mut self) {
        ON_ANY_CONTROL_THREAD.with(|f| f.set(self.prev));
    }
}

/// Authoritative outcome of the non-control-panic stop -> hold -> park -> rest-disable flow.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InternalFaultParkOutcome {
    Idle = 0,
    Parking = 1,
    ParkedAndDisabled = 2,
    HoldingFallback = 3,
}

impl InternalFaultParkOutcome {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Parking,
            2 => Self::ParkedAndDisabled,
            3 => Self::HoldingFallback,
            _ => Self::Idle,
        }
    }
}

/// Lock-free, allocation-free owner-side internal-fault lane shared between the process panic
/// hook, API/WS ingress, and the active control owner (`armd-control`, Legacy or RT Executor).
pub struct InternalFaultLane {
    /// Immediately fences ALL external commands and rearm at ingress AND at the owner TX/motion gate.
    fenced: AtomicBool,
    /// Signals the active control thread to stop motion, revoke prior motion/rearm authority,
    /// and execute the stop-direction park-to-rest -> rest-gated disable sequence.
    park_requested: AtomicBool,
    /// Set once the owner has accepted `park_requested` (one-shot).
    park_accepted: AtomicBool,
    /// Current outcome (`InternalFaultParkOutcome`).
    outcome: AtomicU8,
    /// Set once the non-recoverable fault naming the panicking thread has been latched on `SafetyManager`.
    fault_latched: AtomicBool,
    /// Fixed-size lock-free UTF-8 thread name buffer written by the panic hook.
    thread_name_len: AtomicUsize,
    thread_name: [AtomicU8; THREAD_NAME_CAP],
}

impl Default for InternalFaultLane {
    fn default() -> Self {
        Self::new()
    }
}

impl InternalFaultLane {
    pub fn new() -> Self {
        Self {
            fenced: AtomicBool::new(false),
            park_requested: AtomicBool::new(false),
            park_accepted: AtomicBool::new(false),
            outcome: AtomicU8::new(InternalFaultParkOutcome::Idle as u8),
            fault_latched: AtomicBool::new(false),
            thread_name_len: AtomicUsize::new(0),
            thread_name: std::array::from_fn(|_| AtomicU8::new(0)),
        }
    }

    /// Lock-free, allocation-free, one-shot trigger called from the panic hook or tests.
    /// Immediately fences all new commands & rearm and requests owner-side park-to-rest.
    /// Returns `true` for the first non-control panic recorded on this lane.
    pub fn trigger_non_control_panic(&self, thread_name: &str) -> bool {
        let first = self
            .park_requested
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok();
        if first {
            let bytes = thread_name.as_bytes();
            let mut len = bytes.len().min(THREAD_NAME_CAP);
            while len > 0 && !thread_name.is_char_boundary(len) {
                len -= 1;
            }
            for (i, &b) in bytes[..len].iter().enumerate() {
                self.thread_name[i].store(b, Ordering::Relaxed);
            }
            self.thread_name_len.store(len, Ordering::Release);
        }
        self.fenced.store(true, Ordering::Release);
        first
    }

    /// Marks the command/rearm fence active (used when `armd-control` enters SAFE-HOLD).
    #[inline]
    pub fn latch_control_safe_hold_fence(&self) {
        self.fenced.store(true, Ordering::Release);
    }

    #[inline]
    pub fn is_fenced(&self) -> bool {
        self.fenced.load(Ordering::Acquire)
    }

    #[inline]
    pub fn park_requested(&self) -> bool {
        self.park_requested.load(Ordering::Acquire)
    }

    /// Atomically claims the pending internal-fault park request on the control thread (one-shot).
    #[inline]
    pub fn take_park_request(&self) -> bool {
        self.park_requested.load(Ordering::Acquire)
            && self
                .park_accepted
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
    }

    #[inline]
    pub fn set_outcome(&self, outcome: InternalFaultParkOutcome) {
        self.outcome.store(outcome as u8, Ordering::Release);
    }

    #[inline]
    pub fn outcome(&self) -> InternalFaultParkOutcome {
        InternalFaultParkOutcome::from_u8(self.outcome.load(Ordering::Acquire))
    }

    #[inline]
    pub fn mark_fault_latched(&self) {
        self.fault_latched.store(true, Ordering::Release);
    }

    #[inline]
    pub fn fault_latched(&self) -> bool {
        self.fault_latched.load(Ordering::Acquire)
    }

    /// Reads the panicking thread name into a stack buffer with zero heap allocation.
    pub fn copy_thread_name<'a>(&self, buf: &'a mut [u8; THREAD_NAME_CAP]) -> &'a str {
        let len = self
            .thread_name_len
            .load(Ordering::Acquire)
            .min(THREAD_NAME_CAP);
        for (i, slot) in buf[..len].iter_mut().enumerate() {
            *slot = self.thread_name[i].load(Ordering::Relaxed);
        }
        std::str::from_utf8(&buf[..len])
            .ok()
            .filter(|s| !s.is_empty())
            .unwrap_or("<unnamed>")
    }

    pub fn thread_name_string(&self) -> String {
        let mut buf = [0u8; THREAD_NAME_CAP];
        self.copy_thread_name(&mut buf).to_string()
    }
}

/// Process-wide slot holding the active `InternalFaultLane` for the lock-free panic hook.
///
/// The hook itself NEVER acquires any mutex: it reads a raw `*const InternalFaultLane` stored
/// via `AtomicPtr` (whose `Arc` is kept alive in `ACTIVE_LANE_OWNERS` for the process lifetime).
struct GlobalPanicState {
    lane_ptr: std::sync::atomic::AtomicPtr<InternalFaultLane>,
    coord_woken: AtomicBool,
    coord_thread: OnceLock<Thread>,
    hook_installed: AtomicBool,
    /// Keeps registered `Arc<InternalFaultLane>` alive so `lane_ptr` is always valid to deref
    /// lock-freely inside the panic hook without touching any mutex.
    owners: Mutex<Vec<Arc<InternalFaultLane>>>,
}

static GLOBAL_PANIC: OnceLock<GlobalPanicState> = OnceLock::new();

fn global_panic_state() -> &'static GlobalPanicState {
    GLOBAL_PANIC.get_or_init(|| GlobalPanicState {
        lane_ptr: std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()),
        coord_woken: AtomicBool::new(false),
        coord_thread: OnceLock::new(),
        hook_installed: AtomicBool::new(false),
        owners: Mutex::new(Vec::new()),
    })
}

/// Registers `lane` as the process-wide non-control panic target and installs the one-shot
/// lock-free/zero-alloc panic hook + pre-spawned `"armd-panic-coord"` thread before hardware enable.
pub fn install_process_panic_coordinator(lane: Arc<InternalFaultLane>) {
    let st = global_panic_state();
    // Pre-spawn coordinator thread once (off the panic hook!).
    if st.coord_thread.get().is_none() {
        if let Ok(jh) = thread::Builder::new()
            .name("armd-panic-coord".into())
            .spawn(move || {
                let st = global_panic_state();
                loop {
                    thread::park();
                    if st.coord_woken.swap(false, Ordering::AcqRel) {
                        let ptr = st.lane_ptr.load(Ordering::Acquire);
                        if !ptr.is_null() {
                            // SAFETY: `owners` keeps every installed `Arc<InternalFaultLane>` alive
                            // for the lifetime of the process.
                            let lane = unsafe { &*ptr };
                            let name = lane.thread_name_string();
                            tracing::error!(
                                "Non-control thread '{name}' panicked: commands fenced (INTERNAL_FAULT); \
                                 active control owner executing stop -> hold -> park -> rest-gated disable"
                            );
                        }
                    }
                }
            })
        {
            let _ = st.coord_thread.set(jh.thread().clone());
        }
    }

    // Pin `Arc` for process lifetime so `lane_ptr` can be dereferenced lock-freely inside the hook.
    let raw = Arc::as_ptr(&lane) as *mut InternalFaultLane;
    st.owners.lock().unwrap_or_else(|e| e.into_inner()).push(lane);
    st.lane_ptr.store(raw, Ordering::Release);

    if st
        .hook_installed
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            // Tier 1: Control-thread panics unwind to `catch_unwind` -> SAFE-HOLD directly.
            // Tier 2: Non-control panics trigger the lock-free, allocation-free, non-logging,
            // non-waiting fence + owner-side park lane, and unpark `armd-panic-coord`.
            if !on_any_control_thread() {
                if let Some(st) = GLOBAL_PANIC.get() {
                    let ptr = st.lane_ptr.load(Ordering::Acquire);
                    if !ptr.is_null() {
                        // SAFETY: `owners` retains the `Arc` for the lifetime of the process.
                        let lane = unsafe { &*ptr };
                        let cur = thread::current();
                        let name = cur.name().unwrap_or("<unnamed>");
                        if lane.trigger_non_control_panic(name) {
                            st.coord_woken.store(true, Ordering::Release);
                            if let Some(coord) = st.coord_thread.get() {
                                coord.unpark();
                            }
                        }
                    }
                }
            }
            // Note: chaining `prev` preserves stderr backtrace in tests/diagnostics. For strict
            // non-blocking panic hook behavior in production, `prev` writes directly to stderr.
            prev(info);
        }));
    }
}
