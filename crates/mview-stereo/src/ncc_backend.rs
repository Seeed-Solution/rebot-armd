//! NCC backend selection and per-kernel call diagnostics.

use std::cell::Cell;

/// Active implementation kind for the NCC kernels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum NccBackendKind {
    Scalar,
    Xthead071,
}

impl NccBackendKind {
    /// Lowercase identifier used in diagnostics and environment selection.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Scalar => "scalar",
            Self::Xthead071 => "xthead071",
        }
    }
}

/// Per-thread counters of kernel invocations and cells processed per backend.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct KernelCallCounts {
    pub coarse_dots_scalar: u64,
    pub coarse_dots_xthead071: u64,
    pub coarse_cells_scalar: u64,
    pub coarse_cells_xthead071: u64,
    pub half_moments_scalar: u64,
    pub half_moments_xthead071: u64,
    pub patch_moments_scalar: u64,
    pub patch_moments_xthead071: u64,
}

/// Effective backend, selection reason, and current thread's kernel call counts.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct BackendDiagnostics {
    pub backend: &'static str,
    pub reason: String,
    pub counts: KernelCallCounts,
}

thread_local! {
    static KERNEL_COUNTS: Cell<KernelCallCounts> = const {
        Cell::new(KernelCallCounts {
            coarse_dots_scalar: 0,
            coarse_dots_xthead071: 0,
            coarse_cells_scalar: 0,
            coarse_cells_xthead071: 0,
            half_moments_scalar: 0,
            half_moments_xthead071: 0,
            patch_moments_scalar: 0,
            patch_moments_xthead071: 0,
        })
    };
}

/// Reset the calling thread's kernel call counters to zero.
pub fn reset_kernel_counts() {
    KERNEL_COUNTS.with(|c| c.set(KernelCallCounts::default()));
}

/// Read the calling thread's kernel call counters.
pub fn kernel_counts() -> KernelCallCounts {
    KERNEL_COUNTS.with(Cell::get)
}

/// Report the active backend, selection reason, and current thread's kernel call counts.
pub fn backend_diagnostics() -> BackendDiagnostics {
    BackendDiagnostics {
        backend: NccBackendKind::Scalar.as_str(),
        reason: "scalar-only".to_string(),
        counts: kernel_counts(),
    }
}

#[inline]
#[allow(dead_code)]
pub(crate) fn record_coarse_dots(kind: NccBackendKind, cells: u64) {
    record_coarse_dots_batch(kind, 1, cells);
}

#[inline]
pub(crate) fn record_coarse_dots_batch(kind: NccBackendKind, calls: u64, cells: u64) {
    if calls == 0 && cells == 0 {
        return;
    }
    KERNEL_COUNTS.with(|c| {
        let mut cur = c.get();
        match kind {
            NccBackendKind::Scalar => {
                cur.coarse_dots_scalar += calls;
                cur.coarse_cells_scalar += cells;
            }
            NccBackendKind::Xthead071 => {
                cur.coarse_dots_xthead071 += calls;
                cur.coarse_cells_xthead071 += cells;
            }
        }
        c.set(cur);
    });
}

#[inline]
pub(crate) fn record_half_moments(kind: NccBackendKind) {
    KERNEL_COUNTS.with(|c| {
        let mut cur = c.get();
        match kind {
            NccBackendKind::Scalar => cur.half_moments_scalar += 1,
            NccBackendKind::Xthead071 => cur.half_moments_xthead071 += 1,
        }
        c.set(cur);
    });
}

#[inline]
pub(crate) fn record_patch_moments(kind: NccBackendKind) {
    KERNEL_COUNTS.with(|c| {
        let mut cur = c.get();
        match kind {
            NccBackendKind::Scalar => cur.patch_moments_scalar += 1,
            NccBackendKind::Xthead071 => cur.patch_moments_xthead071 += 1,
        }
        c.set(cur);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backend_diagnostics_reports_scalar_and_resets_counts() {
        reset_kernel_counts();
        let initial = backend_diagnostics();
        assert_eq!(initial.backend, "scalar");
        assert_eq!(initial.reason, "scalar-only");
        assert_eq!(initial.counts, KernelCallCounts::default());
        assert_eq!(NccBackendKind::Scalar.as_str(), "scalar");
        assert_eq!(NccBackendKind::Xthead071.as_str(), "xthead071");

        record_coarse_dots(NccBackendKind::Scalar, 5);
        record_coarse_dots_batch(NccBackendKind::Scalar, 3, 12);
        record_half_moments(NccBackendKind::Scalar);
        record_half_moments(NccBackendKind::Scalar);
        record_patch_moments(NccBackendKind::Scalar);

        let mid = backend_diagnostics();
        assert_eq!(mid.backend, "scalar");
        assert_eq!(mid.counts.coarse_dots_scalar, 4);
        assert_eq!(mid.counts.coarse_cells_scalar, 17);
        assert_eq!(mid.counts.half_moments_scalar, 2);
        assert_eq!(mid.counts.patch_moments_scalar, 1);
        assert_eq!(mid.counts.coarse_dots_xthead071, 0);
        assert_eq!(mid.counts.coarse_cells_xthead071, 0);
        assert_eq!(mid.counts.half_moments_xthead071, 0);
        assert_eq!(mid.counts.patch_moments_xthead071, 0);

        reset_kernel_counts();
        assert_eq!(kernel_counts(), KernelCallCounts::default());
    }
}
