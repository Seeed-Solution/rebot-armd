//! Scheduling of the RT executor (design section 5): the platform profile of the control
//! thread and the process-wide memory lock.
//!
//! - Priority: SCHED_FIFO 79 by default on SG2002. The vendor kernel
//!   (`linux_5.10/kernel/sched/cvi_sched.h`) caps non-`cvitask` tasks at 80 and pins
//!   `cvitask_isp_pre`, `cvitask_vpss_0/1` (90) and `cvitask_tpu_wor` (85). The kit init script
//!   raises `spi0` and SPI/CAN IRQ threads to SCHED_FIFO 80 and lowers the 4 unpinned `cvitask`
//!   threads and `rebot-vision`'s RT SDK threads to 70, placing `armd-control` at FIFO 79:
//!   strictly below SPI/CAN bus interrupts and strictly above vision/unpinned `cvitask` threads.
//!   `ARMD_RT_PRIORITY` (1..=99) overrides the default for other platforms or A/B tests.
//! - CPU: `ARMD_RT_CPU` pins the executor to one CPU (an isolated core on multi-core
//!   boards); unset, the thread keeps the inherited affinity.
//! - Memory: `mlockall(MCL_CURRENT | MCL_FUTURE)` at startup, so no page fault of armd's
//!   memory can stall the executor. It is attempted only where the memory-lock limit cannot
//!   make a later allocation fail (root, or `RLIMIT_MEMLOCK` unlimited); `ARMD_MLOCKALL=0`
//!   turns it off. The result is logged and published in `GET /api/v1/diagnostics/tick`
//!   (`control_thread`).
//!
//! [`RtReadiness`] is the fail-closed gate: MIT tracking is refused unless the executor runs
//! at SCHED_FIFO and the memory lock succeeded.

use crate::diagnostics::tick::ControlThreadSched;

/// Default executor priority on SG2002 (`SCHED_FIFO 79`): strictly below the SPI / CAN interrupt
/// threads raised to FIFO 80 (the vendor `cvi_sched.h` cap for non-`cvitask` tasks), and strictly
/// above `rebot-vision` RT SDK threads and the 4 unpinned `cvitask_*` threads lowered to 70 by the
/// kit init script. Overridable via `ARMD_RT_PRIORITY`.
pub const DEFAULT_PRIORITY: i32 = 79;

/// Scheduling request of the executor thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtSchedConfig {
    /// SCHED_FIFO priority, 1..=99.
    pub priority: i32,
    /// CPU to pin the executor to.
    pub cpu: Option<usize>,
    /// Attempt `mlockall(MCL_CURRENT | MCL_FUTURE)` at startup.
    pub lock_memory: bool,
}

impl Default for RtSchedConfig {
    fn default() -> Self { Self { priority: DEFAULT_PRIORITY, cpu: None, lock_memory: true } }
}

impl RtSchedConfig {
    /// From `ARMD_RT_PRIORITY`, `ARMD_RT_CPU` and `ARMD_MLOCKALL`; unset variables keep the
    /// defaults. An invalid value is an error (armd refuses to start rather than run with
    /// a priority nobody asked for).
    pub fn from_env() -> Result<Self, String> {
        Self::from_vars(|k| std::env::var(k).ok())
    }

    pub fn from_vars(var: impl Fn(&str) -> Option<String>) -> Result<Self, String> {
        let mut cfg = Self::default();
        if let Some(v) = var("ARMD_RT_PRIORITY") {
            cfg.priority = v.trim().parse::<i32>().ok().filter(|p| (1..=99).contains(p))
                .ok_or_else(|| format!("ARMD_RT_PRIORITY must be 1..=99, got {v:?}"))?;
        }
        if let Some(v) = var("ARMD_RT_CPU") {
            cfg.cpu = Some(v.trim().parse::<usize>().map_err(|_| format!("ARMD_RT_CPU must be a CPU number, got {v:?}"))?);
        }
        if let Some(v) = var("ARMD_MLOCKALL") {
            cfg.lock_memory = match v.trim() {
                "1" | "on" | "true" => true,
                "0" | "off" | "false" => false,
                _ => return Err(format!("ARMD_MLOCKALL must be 0 or 1, got {v:?}")),
            };
        }
        Ok(cfg)
    }
}

/// Outcome of the startup memory lock.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MemoryLock {
    Locked,
    /// Not attempted (the reason).
    Skipped(String),
    /// `mlockall` failed (the OS error).
    Failed(String),
}

impl MemoryLock {
    pub fn is_locked(&self) -> bool { matches!(self, Self::Locked) }

    /// `"locked"`, `"skipped: ..."` or `"failed: ..."`.
    pub fn describe(&self) -> String {
        match self {
            Self::Locked => "locked".into(),
            Self::Skipped(r) => format!("skipped: {r}"),
            Self::Failed(e) => format!("failed: {e}"),
        }
    }
}

/// `mlockall(MCL_CURRENT | MCL_FUTURE)` when `cfg.lock_memory` and the memory-lock limit
/// allows it for the rest of the process's life (see the module docs).
pub fn lock_memory(cfg: &RtSchedConfig) -> MemoryLock {
    if !cfg.lock_memory { return MemoryLock::Skipped("ARMD_MLOCKALL=0".into()); }
    #[cfg(target_os = "linux")]
    {
        // SAFETY: plain syscalls; `lim` is a valid out-pointer.
        let (root, unlimited) = unsafe {
            let mut lim: libc::rlimit = std::mem::zeroed();
            let ok = libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut lim) == 0;
            (libc::geteuid() == 0, ok && lim.rlim_cur == libc::RLIM_INFINITY)
        };
        if !root && !unlimited {
            // MCL_FUTURE with a finite limit makes later mappings fail once the limit is
            // reached, and a failed allocation aborts the process.
            return MemoryLock::Skipped("not root and RLIMIT_MEMLOCK is finite".into());
        }
        // SAFETY: plain syscall without pointers.
        if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } == 0 {
            MemoryLock::Locked
        } else {
            MemoryLock::Failed(std::io::Error::last_os_error().to_string())
        }
    }
    #[cfg(not(target_os = "linux"))]
    MemoryLock::Skipped("not Linux".into())
}

/// Applies `cfg` to the calling thread (SCHED_FIFO priority, CPU affinity) and reads the
/// scheduling back. `memory` is the startup memory lock, recorded with it.
pub fn apply_to_current_thread(cfg: &RtSchedConfig, memory: &MemoryLock) -> ControlThreadSched {
    #[cfg(target_os = "linux")]
    {
        let mut errors = Vec::new();
        // musl adds sporadic-server fields; zero them and set only the priority.
        // SAFETY: sched_param is plain old data, all-zero is a valid value.
        let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
        param.sched_priority = cfg.priority;
        // SAFETY: plain call on the calling thread with a valid param struct. musl stubs
        // sched_setscheduler() with ENOSYS; the pthread call is implemented.
        let rc = unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
        if rc != 0 { errors.push(format!("SCHED_FIFO {}: {}", cfg.priority, std::io::Error::from_raw_os_error(rc))); }
        let mut cpu_ok = None;
        if let Some(cpu) = cfg.cpu {
            // SAFETY: cpu_set_t is plain old data; the call only touches the calling thread.
            let ok = unsafe {
                let mut set: libc::cpu_set_t = std::mem::zeroed();
                cpu < libc::CPU_SETSIZE as usize && {
                    libc::CPU_SET(cpu, &mut set);
                    libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) == 0
                }
            };
            if !ok { errors.push(format!("CPU {cpu}: {}", std::io::Error::last_os_error())); }
            cpu_ok = Some(ok);
        }
        let mut policy: libc::c_int = 0;
        // SAFETY: as above; both out-pointers are valid for the call.
        let mut now: libc::sched_param = unsafe { std::mem::zeroed() };
        let rc_get = unsafe { libc::pthread_getschedparam(libc::pthread_self(), &mut policy, &mut now) };
        let name = if rc_get != 0 { "unknown".to_string() } else {
            match policy {
                libc::SCHED_FIFO => "fifo".to_string(),
                libc::SCHED_RR => "rr".to_string(),
                libc::SCHED_OTHER => "other".to_string(),
                p => p.to_string(),
            }
        };
        ControlThreadSched {
            policy: name,
            priority: if rc_get == 0 { now.sched_priority } else { 0 },
            realtime: rc == 0 && rc_get == 0 && policy == libc::SCHED_FIFO,
            requested_priority: Some(cfg.priority),
            cpu: cfg.cpu.filter(|_| cpu_ok == Some(true)),
            memory_locked: Some(memory.is_locked()),
            memory_lock: Some(memory.describe()),
            errors,
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cfg;
        ControlThreadSched {
            policy: "other".into(),
            priority: 0,
            realtime: false,
            requested_priority: Some(cfg.priority),
            cpu: None,
            memory_locked: Some(memory.is_locked()),
            memory_lock: Some(memory.describe()),
            errors: vec!["real-time scheduling needs Linux".into()],
        }
    }
}

/// Reads the calling thread's current scheduling policy and priority without modifying it or
/// updating process-wide [`RtReadiness`].
pub fn observe_current_thread(memory: &MemoryLock) -> ControlThreadSched {
    #[cfg(target_os = "linux")]
    {
        let mut policy: libc::c_int = 0;
        // SAFETY: `sched_param` is plain old data; both out-pointers are valid for the call.
        let mut now: libc::sched_param = unsafe { std::mem::zeroed() };
        let rc_get = unsafe { libc::pthread_getschedparam(libc::pthread_self(), &mut policy, &mut now) };
        let name = if rc_get != 0 {
            "unknown".to_string()
        } else {
            match policy {
                libc::SCHED_FIFO => "fifo".to_string(),
                libc::SCHED_RR => "rr".to_string(),
                libc::SCHED_OTHER => "other".to_string(),
                p => p.to_string(),
            }
        };
        ControlThreadSched {
            policy: name,
            priority: if rc_get == 0 { now.sched_priority } else { 0 },
            realtime: rc_get == 0 && policy == libc::SCHED_FIFO,
            requested_priority: None,
            cpu: None,
            memory_locked: Some(memory.is_locked()),
            memory_lock: Some(memory.describe()),
            errors: Vec::new(),
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        ControlThreadSched {
            policy: "other".into(),
            priority: 0,
            realtime: false,
            requested_priority: None,
            cpu: None,
            memory_locked: Some(memory.is_locked()),
            memory_lock: Some(memory.describe()),
            errors: Vec::new(),
        }
    }
}

/// Whether the executor runs as the RT design requires (SCHED_FIFO, memory locked); MIT
/// tracking is refused otherwise (fail-closed).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RtReadiness {
    pub realtime: bool,
    pub memory_locked: bool,
    pub detail: String,
}

impl RtReadiness {
    pub fn from_sched(sched: &ControlThreadSched) -> Self {
        let memory_locked = sched.memory_locked == Some(true);
        let mut detail = format!("control thread {} priority {}", sched.policy, sched.priority);
        if let Some(m) = &sched.memory_lock { detail.push_str(&format!(", memory {m}")); }
        for e in &sched.errors { detail.push_str(&format!(", {e}")); }
        Self { realtime: sched.realtime, memory_locked, detail }
    }

    pub fn ready(&self) -> bool { self.realtime && self.memory_locked }

    /// None when ready, else why MIT is refused.
    pub fn refusal(&self) -> Option<String> {
        (!self.ready()).then(|| format!("the RT executor is not real-time ({}); MIT needs SCHED_FIFO and locked memory", self.detail))
    }
}

static READINESS: std::sync::OnceLock<RtReadiness> = std::sync::OnceLock::new();

/// Records the executor's readiness (the armd binary, once the executor thread runs). Only
/// the first call counts.
pub fn set_readiness(r: RtReadiness) { let _ = READINESS.set(r); }

/// Why MIT tracking is refused by the RT gate: None when the executor is ready, or when no
/// executor recorded its readiness (library use and tests drive the tick themselves).
pub fn mit_refusal() -> Option<String> { READINESS.get().and_then(RtReadiness::refusal) }

#[cfg(test)]
mod tests {
    use super::*;

    fn vars<'a>(pairs: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<String> + 'a {
        move |k| pairs.iter().find(|(n, _)| *n == k).map(|(_, v)| v.to_string())
    }

    #[test]
    fn config_defaults_and_env() {
        assert_eq!(RtSchedConfig::from_vars(vars(&[])).unwrap(), RtSchedConfig { priority: 79, cpu: None, lock_memory: true });
        let c = RtSchedConfig::from_vars(vars(&[("ARMD_RT_PRIORITY", "60"), ("ARMD_RT_CPU", "3"), ("ARMD_MLOCKALL", "0")])).unwrap();
        assert_eq!(c, RtSchedConfig { priority: 60, cpu: Some(3), lock_memory: false });
        assert!(RtSchedConfig::from_vars(vars(&[("ARMD_RT_PRIORITY", "0")])).is_err());
        assert!(RtSchedConfig::from_vars(vars(&[("ARMD_RT_PRIORITY", "100")])).is_err());
        assert!(RtSchedConfig::from_vars(vars(&[("ARMD_RT_CPU", "x")])).is_err());
        assert!(RtSchedConfig::from_vars(vars(&[("ARMD_MLOCKALL", "maybe")])).is_err());
        assert_eq!(lock_memory(&c), MemoryLock::Skipped("ARMD_MLOCKALL=0".into()));
    }

    #[test]
    fn readiness_is_fail_closed() {
        let mut s = ControlThreadSched {
            policy: "fifo".into(), priority: 79, realtime: true, requested_priority: Some(79), cpu: None,
            memory_locked: Some(true), memory_lock: Some("locked".into()), errors: vec![],
        };
        assert!(RtReadiness::from_sched(&s).refusal().is_none());
        s.memory_locked = Some(false);
        assert!(RtReadiness::from_sched(&s).refusal().unwrap().contains("locked memory"));
        s.memory_locked = None;
        assert!(!RtReadiness::from_sched(&s).ready());
        s.memory_locked = Some(true);
        s.realtime = false;
        assert!(!RtReadiness::from_sched(&s).ready());
    }
}
