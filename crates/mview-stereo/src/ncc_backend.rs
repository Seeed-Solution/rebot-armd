//! NCC backend selection, checked patch views, ISA detection, scalar oracle, and diagnostics.

use std::cell::Cell;
use std::fmt;
use std::sync::OnceLock;

use crate::{Gray, PATCH_R};

/// Maximum patch radius supported by the half-grid integer-moment kernel (`33x33` patch).
pub(crate) const HALF_GRID_MAX_R: usize = 16;

/// Compiled-in RVV 0.7.1 archive availability (enabled by `build.rs` on `riscv64gc-unknown-linux-musl`).
const HAS_COMPILED_XTHEAD071: bool = cfg!(mview_has_xthead071);

#[cfg(mview_has_xthead071)]
mod ffi {
    extern "C" {
        pub(crate) fn mvs071_coarse_dots(
            p: *const u8,
            stride: usize,
            a: *const f32,
            n: usize,
            count: usize,
            out: *mut f32,
        );

        pub(crate) fn mvs071_half_moments(
            p: *const u8,
            stride: usize,
            a: *const f32,
            n: usize,
            phase: u32,
            sum_raw: *mut u32,
            sumsq_raw: *mut u32,
            dot: *mut f32,
        );
    }
}

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
    let sel = active_selection();
    BackendDiagnostics {
        backend: sel.backend.as_str(),
        reason: sel.reason.clone(),
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

// ---- Checked patch views ----

/// Validation errors for patch geometry, backing buffer bounds, or kernel arguments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PatchError {
    EmptyDimensions,
    OutOfBounds,
    TruncatedBuffer,
    Overflow,
    InvalidTemplate,
    InvalidCount,
    InvalidPhase,
    RadiusTooLarge,
}

impl fmt::Display for PatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyDimensions => f.write_str("patch or image dimensions must be nonzero"),
            Self::OutOfBounds => f.write_str("patch rectangle exceeds image dimensions"),
            Self::TruncatedBuffer => f.write_str("image pixel buffer is smaller than patch span"),
            Self::Overflow => f.write_str("integer overflow computing patch bounds"),
            Self::InvalidTemplate => f.write_str("invalid template dimensions or buffer length"),
            Self::InvalidCount => f.write_str("invalid output count or slice length"),
            Self::InvalidPhase => f.write_str("half-grid phase must be in 0..=3"),
            Self::RadiusTooLarge => f.write_str("half-grid patch radius exceeds HALF_GRID_MAX_R"),
        }
    }
}

impl std::error::Error for PatchError {}

mod patch_view {
    use super::{Gray, PatchError};

    /// Bounds-checked immutable view over a rectangular region of a [`Gray`] image.
    ///
    /// Constructible exclusively via [`PatchView::new`] so every stride, row wrap, and backing
    /// allocation check is enforced before any slice access or FFI pointer formation.
    #[derive(Debug, Clone, Copy)]
    pub(crate) struct PatchView<'a> {
        px: &'a [u8],
        base: usize,
        stride: usize,
        width: usize,
        height: usize,
    }

    impl<'a> PatchView<'a> {
        /// Validate `(x0, y0, width, height)` against `img` and its backing `px` buffer.
        pub(crate) fn new(
            img: &'a Gray,
            x0: usize,
            y0: usize,
            width: usize,
            height: usize,
        ) -> Result<Self, PatchError> {
            if width == 0 || height == 0 || img.w == 0 || img.h == 0 {
                return Err(PatchError::EmptyDimensions);
            }
            let x_end = x0.checked_add(width).ok_or(PatchError::Overflow)?;
            if x_end > img.w {
                return Err(PatchError::OutOfBounds);
            }
            let y_end = y0.checked_add(height).ok_or(PatchError::Overflow)?;
            if y_end > img.h {
                return Err(PatchError::OutOfBounds);
            }
            let base = y0
                .checked_mul(img.w)
                .and_then(|row| row.checked_add(x0))
                .ok_or(PatchError::Overflow)?;
            let last_row_off = height
                .checked_sub(1)
                .and_then(|h| h.checked_mul(img.w))
                .ok_or(PatchError::Overflow)?;
            let end = base
                .checked_add(last_row_off)
                .and_then(|v| v.checked_add(width))
                .ok_or(PatchError::Overflow)?;
            if end > img.px.len() {
                return Err(PatchError::TruncatedBuffer);
            }
            Ok(Self {
                px: &img.px,
                base,
                stride: img.w,
                width,
                height,
            })
        }

        #[inline]
        #[allow(dead_code)]
        pub(crate) fn as_ptr(&self) -> *const u8 {
            self.px[self.base..].as_ptr()
        }

        #[inline]
        #[allow(dead_code)]
        pub(crate) fn stride(&self) -> usize {
            self.stride
        }

        #[inline]
        #[allow(dead_code)]
        pub(crate) fn width(&self) -> usize {
            self.width
        }

        #[inline]
        #[allow(dead_code)]
        pub(crate) fn height(&self) -> usize {
            self.height
        }

        #[inline]
        pub(crate) fn row(&self, j: usize) -> &'a [u8] {
            assert!(j < self.height, "patch row {j} out of bounds {}", self.height);
            let start = self.base + j * self.stride;
            &self.px[start..start + self.width]
        }
    }
}

pub(crate) use patch_view::PatchView;

fn check_odd_template(a: &[f32], n: usize) -> Result<usize, PatchError> {
    if n == 0 || n.is_multiple_of(2) {
        return Err(PatchError::InvalidTemplate);
    }
    let nn = n.checked_mul(n).ok_or(PatchError::Overflow)?;
    if a.len() != nn {
        return Err(PatchError::InvalidTemplate);
    }
    Ok(nn)
}

#[allow(dead_code)]
pub(crate) fn validate_coarse_dots<'a>(
    img: &'a Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    n: usize,
    count: usize,
    out: &[f32],
) -> Result<PatchView<'a>, PatchError> {
    check_odd_template(a, n)?;
    if count == 0 || out.len() != count {
        return Err(PatchError::InvalidCount);
    }
    let width = n.checked_add(count - 1).ok_or(PatchError::Overflow)?;
    PatchView::new(img, x0, y0, width, n)
}

pub(crate) fn validate_half_moments<'a>(
    img: &'a Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    n: usize,
    phase: u8,
) -> Result<PatchView<'a>, PatchError> {
    check_odd_template(a, n)?;
    if n / 2 > HALF_GRID_MAX_R {
        return Err(PatchError::RadiusTooLarge);
    }
    if phase > 3 {
        return Err(PatchError::InvalidPhase);
    }
    let width = n
        .checked_add(usize::from(phase == 1 || phase == 3))
        .ok_or(PatchError::Overflow)?;
    let height = n
        .checked_add(usize::from(phase == 2 || phase == 3))
        .ok_or(PatchError::Overflow)?;
    PatchView::new(img, x0, y0, width, height)
}

pub(crate) fn validate_patch_moments<'a>(
    img: &'a Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    weights: Option<&[f32]>,
    n: usize,
    bilerp: Option<[f32; 4]>,
) -> Result<PatchView<'a>, PatchError> {
    let nn = check_odd_template(a, n)?;
    if let Some(w) = weights {
        if w.len() != nn {
            return Err(PatchError::InvalidTemplate);
        }
    }
    let extra = usize::from(bilerp.is_some());
    let width = n.checked_add(extra).ok_or(PatchError::Overflow)?;
    let height = n.checked_add(extra).ok_or(PatchError::Overflow)?;
    PatchView::new(img, x0, y0, width, height)
}

// ---- Once-per-process dispatch & RVV 0.7.1 ISA detection ----

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct BackendSelection {
    pub(crate) backend: NccBackendKind,
    pub(crate) reason: String,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct IsaProbe {
    pub auxv: Option<Vec<u8>>,
    pub cpuinfo: Option<String>,
    pub dt_compatible: Option<Vec<u8>>,
    pub kernel_release: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct UnversionedVAllowlistEntry {
    pub dt_compatible: &'static str,
    pub uarch: &'static str,
    pub kernel_release: &'static str,
}

/// Empty until real reCamera SG2002 `/proc/device-tree/compatible`, `uarch`, and
/// `/proc/sys/kernel/osrelease` strings are captured on device.
const UNVERSIONED_V_ALLOWLIST: &[UnversionedVAllowlistEntry] = &[];

static ACTIVE_SELECTION: OnceLock<BackendSelection> = OnceLock::new();

#[cfg(all(target_os = "linux", target_arch = "riscv64"))]
fn read_os_probe() -> IsaProbe {
    IsaProbe {
        auxv: std::fs::read("/proc/self/auxv").ok(),
        cpuinfo: std::fs::read_to_string("/proc/cpuinfo").ok(),
        dt_compatible: std::fs::read("/proc/device-tree/compatible").ok(),
        kernel_release: std::fs::read_to_string("/proc/sys/kernel/osrelease").ok(),
    }
}

#[cfg(not(all(target_os = "linux", target_arch = "riscv64")))]
fn read_os_probe() -> IsaProbe {
    IsaProbe::default()
}

fn active_selection() -> &'static BackendSelection {
    ACTIVE_SELECTION.get_or_init(|| {
        let env = match std::env::var("MVIEW_NCC_BACKEND") {
            Ok(s) => Some(Ok(s)),
            Err(std::env::VarError::NotPresent) => None,
            Err(std::env::VarError::NotUnicode(_)) => Some(Err(())),
        };
        let env_ref = env.as_ref().map(|r| match r {
            Ok(s) => Ok(s.as_str()),
            Err(()) => Err(()),
        });
        let probe = if HAS_COMPILED_XTHEAD071
            && matches!(env_ref, None | Some(Ok("auto")))
        {
            read_os_probe()
        } else {
            IsaProbe::default()
        };
        detect_backend_with_allowlist(env_ref, HAS_COMPILED_XTHEAD071, &probe, UNVERSIONED_V_ALLOWLIST)
    })
}

/// Parse Linux `/proc/self/auxv` 64-bit LE `(key, val)` pairs up to `AT_NULL` (`k == 0`)
/// and return whether `AT_HWCAP` (`k == 16`) has bit 21 (`'V' - 'A'`) set.
pub(crate) fn parse_hwcap_v(auxv: &[u8]) -> Result<bool, &'static str> {
    if auxv.is_empty() {
        return Err("auxv:empty");
    }
    if !auxv.len().is_multiple_of(16) {
        return Err("auxv:truncated");
    }
    let mut hwcap_v = None;
    for chunk in auxv.chunks_exact(16) {
        let k = u64::from_le_bytes(chunk[0..8].try_into().unwrap());
        let v = u64::from_le_bytes(chunk[8..16].try_into().unwrap());
        if k == 0 {
            return Ok(hwcap_v.unwrap_or(false));
        }
        if k == 16 {
            hwcap_v = Some((v & (1u64 << 21)) != 0);
        }
    }
    Err("auxv:missing_at_null")
}

/// Split NUL-delimited `/proc/device-tree/compatible` bytes into UTF-8 entries, dropping empty
/// trailing entries. Returns an empty vector if any non-empty entry is not valid UTF-8.
pub(crate) fn parse_dt_compatibles(raw: &[u8]) -> Vec<&str> {
    let mut out = Vec::new();
    for entry in raw.split(|&b| b == 0) {
        if entry.is_empty() {
            continue;
        }
        let Ok(s) = std::str::from_utf8(entry) else {
            return Vec::new();
        };
        out.push(s);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RiscvVExt {
    None,
    Draft0p7,
    Unversioned,
    OtherVersion(u32, Option<u32>),
    Rvv1Subext,
    Conflict,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IsaVersion {
    None,
    Parts(u32, Option<u32>, Option<u32>),
}

/// Consume `<digits>(p<digits>)*` starting at `bytes[pos]`. Only treats `'p'` as a version
/// separator when immediately followed by an ASCII digit so `i2p0p` still parses `p` later.
fn parse_version_suffix(bytes: &[u8], pos: &mut usize) -> IsaVersion {
    if *pos >= bytes.len() || !bytes[*pos].is_ascii_digit() {
        return IsaVersion::None;
    }
    let read_digits = |p: &mut usize| -> Option<u32> {
        let start = *p;
        while *p < bytes.len() && bytes[*p].is_ascii_digit() {
            *p += 1;
        }
        std::str::from_utf8(&bytes[start..*p]).ok()?.parse::<u32>().ok()
    };
    let Some(major) = read_digits(pos) else {
        return IsaVersion::None;
    };
    let mut minor = None;
    let mut patch = None;
    if *pos + 1 < bytes.len() && bytes[*pos] == b'p' && bytes[*pos + 1].is_ascii_digit() {
        *pos += 1;
        minor = read_digits(pos);
        if *pos + 1 < bytes.len() && bytes[*pos] == b'p' && bytes[*pos + 1].is_ascii_digit() {
            *pos += 1;
            patch = read_digits(pos);
        }
    }
    // Reject extra `p<digits>` chains beyond major.minor.patch cleanly in callers via leftover check.
    while *pos + 1 < bytes.len() && bytes[*pos] == b'p' && bytes[*pos + 1].is_ascii_digit() {
        *pos += 1;
        let _ = read_digits(pos);
    }
    IsaVersion::Parts(major, minor, patch)
}

fn is_single_letter_ext(b: u8) -> bool {
    matches!(
        b,
        b'm' | b'a'
            | b'f'
            | b'd'
            | b'c'
            | b'b'
            | b'k'
            | b'j'
            | b'p'
            | b'v'
            | b'h'
            | b'n'
            | b'q'
            | b'l'
            | b't'
            | b's'
            | b'u'
    )
}

fn is_rvv1_subext_name(seg: &str) -> bool {
    let is_zve = seg
        .strip_prefix("zve")
        .is_some_and(|r| r.as_bytes().first().is_some_and(u8::is_ascii_digit));
    let is_zvl = seg
        .strip_prefix("zvl")
        .is_some_and(|r| r.as_bytes().first().is_some_and(u8::is_ascii_digit));
    is_zve || is_zvl
}

fn classify_v_version(ver: IsaVersion) -> RiscvVExt {
    match ver {
        IsaVersion::None => RiscvVExt::Unversioned,
        IsaVersion::Parts(0, Some(7), None | Some(1)) => RiscvVExt::Draft0p7,
        IsaVersion::Parts(maj, min, _) => RiscvVExt::OtherVersion(maj, min),
    }
}

fn record_v_ext(cur: &mut RiscvVExt, next: RiscvVExt) {
    match (*cur, next) {
        (RiscvVExt::None, n) => *cur = n,
        (RiscvVExt::Conflict, _) | (_, RiscvVExt::Conflict) => *cur = RiscvVExt::Conflict,
        _ => *cur = RiscvVExt::Conflict,
    }
}

/// Strictly parse a RISC-V `isa` string and classify its vector extension identity.
pub(crate) fn parse_riscv_isa(isa: &str) -> Result<RiscvVExt, &'static str> {
    let lower = isa.trim().to_ascii_lowercase();
    let Some(rest) = lower.strip_prefix("rv64") else {
        return Err("isa:not_rv64");
    };
    if rest.is_empty() {
        return Err("isa:empty_after_rv64");
    }

    let mut segments = rest.split('_');
    let seg0 = segments.next().ok_or("isa:empty")?;
    if seg0.is_empty() {
        return Err("isa:empty_base_segment");
    }

    let mut v_ext = RiscvVExt::None;
    let mut rvv1_subext = false;

    // Parse seg0: base letter ('i' | 'e' | 'g') + single-letter extensions (optionally versioned),
    // stopping cleanly if a multi-letter z*/x*/s* suffix appears without '_' at the end of seg0.
    let b0 = seg0.as_bytes();
    if !matches!(b0[0], b'i' | b'e' | b'g') {
        return Err("isa:invalid_base");
    }
    let mut pos = 1usize;
    let _base_ver = parse_version_suffix(b0, &mut pos);
    while pos < b0.len() {
        let ch = b0[pos];
        if matches!(ch, b'z' | b'x') {
            let tail = &seg0[pos..];
            if tail.len() < 2 || !tail.bytes().all(|b| b.is_ascii_alphanumeric()) {
                return Err("isa:invalid_multi_ext");
            }
            if is_rvv1_subext_name(tail) {
                rvv1_subext = true;
            }
            break;
        }
        if ch == b's' {
            let single_s = pos + 1 == b0.len()
                || b0[pos + 1].is_ascii_digit()
                || (b0[pos + 1] == b'u' && (pos + 2 == b0.len() || b0[pos + 2].is_ascii_digit()));
            if !single_s {
                let tail = &seg0[pos..];
                if tail.len() < 2 || !tail.bytes().all(|b| b.is_ascii_alphanumeric()) {
                    return Err("isa:invalid_multi_ext");
                }
                break;
            }
        }
        if !is_single_letter_ext(ch) {
            return Err("isa:invalid_single_ext");
        }
        pos += 1;
        let ver = parse_version_suffix(b0, &mut pos);
        if ch == b'v' {
            record_v_ext(&mut v_ext, classify_v_version(ver));
        }
    }

    // Parse remaining underscore-separated segments.
    for seg in segments {
        if seg.is_empty() {
            return Err("isa:empty_segment");
        }
        let sb = seg.as_bytes();
        if !sb[0].is_ascii_lowercase() || !seg.bytes().all(|b| b.is_ascii_alphanumeric()) {
            return Err("isa:invalid_segment_chars");
        }
        if matches!(sb[0], b'z' | b'x') {
            if seg.len() < 2 {
                return Err("isa:short_multi_ext");
            }
            if is_rvv1_subext_name(seg) {
                rvv1_subext = true;
            }
        } else if sb[0] == b's' && seg.len() > 1 && !sb[1].is_ascii_digit() {
            // Multi-letter supervisor extension such as sstc, svpbmt.
        } else {
            if !is_single_letter_ext(sb[0]) {
                return Err("isa:invalid_single_ext");
            }
            let mut p = 1usize;
            let ver = parse_version_suffix(sb, &mut p);
            if p != sb.len() {
                return Err("isa:trailing_garbage");
            }
            if sb[0] == b'v' {
                record_v_ext(&mut v_ext, classify_v_version(ver));
            }
        }
    }

    if rvv1_subext {
        match v_ext {
            RiscvVExt::None => return Ok(RiscvVExt::Rvv1Subext),
            RiscvVExt::OtherVersion(_, _) => return Ok(v_ext),
            _ => return Ok(RiscvVExt::Conflict),
        }
    }
    Ok(v_ext)
}

#[derive(Debug, Clone, Default)]
struct HartInfo<'a> {
    saw_processor: bool,
    isa: Option<&'a str>,
    uarch: Option<&'a str>,
    duplicate_conflict: bool,
}

fn parse_cpuinfo_harts(cpuinfo: &str) -> Vec<HartInfo<'_>> {
    let mut harts = Vec::new();
    let mut cur = HartInfo::default();
    let mut has_fields = false;

    for raw_line in cpuinfo.lines() {
        let line = raw_line.trim();
        if line.is_empty() {
            if has_fields {
                harts.push(cur);
                cur = HartInfo::default();
                has_fields = false;
            }
            continue;
        }
        let Some((k, v)) = line.split_once(':') else {
            continue;
        };
        let key = k.trim();
        let val = v.trim();
        if key.eq_ignore_ascii_case("processor") {
            if cur.saw_processor {
                harts.push(cur);
                cur = HartInfo::default();
            }
            cur.saw_processor = true;
            has_fields = true;
        } else if key.eq_ignore_ascii_case("isa") {
            has_fields = true;
            if let Some(prev) = cur.isa {
                if prev != val {
                    cur.duplicate_conflict = true;
                }
            } else {
                cur.isa = Some(val);
            }
        } else if key.eq_ignore_ascii_case("uarch") {
            has_fields = true;
            if let Some(prev) = cur.uarch {
                if prev != val {
                    cur.duplicate_conflict = true;
                }
            } else {
                cur.uarch = Some(val);
            }
        } else {
            has_fields = true;
        }
    }
    if has_fields {
        harts.push(cur);
    }
    harts
}

fn sanitize_field(s: &str, max_bytes: usize) -> String {
    let cleaned: String = s
        .trim()
        .chars()
        .map(|c| {
            if c.is_ascii_graphic() && !matches!(c, ';' | '(' | ')') {
                c
            } else {
                '_'
            }
        })
        .collect();
    if cleaned.is_empty() {
        return "-".to_string();
    }
    if cleaned.len() <= max_bytes {
        cleaned
    } else {
        cleaned[..max_bytes].to_string()
    }
}

fn is_supported_sg2002_dt(compat: &[&str]) -> bool {
    compat.iter().any(|&c| {
        matches!(
            c,
            "sophgo,sg2002" | "cvitek,sg2002" | "cvitek,cv181x" | "thead,c906"
        )
    })
}

pub(crate) fn detect_backend_with_allowlist(
    env: Option<Result<&str, ()>>,
    has_compiled_kernel: bool,
    probe: &IsaProbe,
    allowlist: &[UnversionedVAllowlistEntry],
) -> BackendSelection {
    match env {
        Some(Ok("scalar")) => {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: "env:scalar".to_string(),
            };
        }
        Some(Ok("auto")) | None => {}
        Some(Ok(other)) => {
            let tag = if other.is_empty() {
                "<empty>".to_string()
            } else {
                sanitize_field(other, 48)
            };
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: format!("env:unknown:{tag}"),
            };
        }
        Some(Err(())) => {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: "env:unknown:non_utf8".to_string(),
            };
        }
    }

    if !has_compiled_kernel {
        return BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "scalar-only".to_string(),
        };
    }

    let Some(auxv) = probe.auxv.as_deref() else {
        return BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "auxv:missing".to_string(),
        };
    };
    match parse_hwcap_v(auxv) {
        Ok(true) => {}
        Ok(false) => {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: "auxv:no_v_hwcap".to_string(),
            };
        }
        Err(err) => {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: err.to_string(),
            };
        }
    }

    let Some(cpuinfo) = probe.cpuinfo.as_deref() else {
        return BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "cpuinfo:missing".to_string(),
        };
    };
    let harts = parse_cpuinfo_harts(cpuinfo);
    if harts.is_empty() {
        return BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "cpuinfo:no_harts".to_string(),
        };
    }

    let mut first_v: Option<RiscvVExt> = None;
    let mut first_uarch: Option<Option<&str>> = None;
    let mut first_isa_raw: &str = "";

    for hart in &harts {
        if hart.duplicate_conflict {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: "cpuinfo:conflicting_hart".to_string(),
            };
        }
        let Some(isa_str) = hart.isa else {
            return BackendSelection {
                backend: NccBackendKind::Scalar,
                reason: "cpuinfo:missing_isa".to_string(),
            };
        };
        let parsed = match parse_riscv_isa(isa_str) {
            Ok(v) => v,
            Err(err) => {
                return BackendSelection {
                    backend: NccBackendKind::Scalar,
                    reason: err.to_string(),
                };
            }
        };
        match first_v {
            None => {
                first_v = Some(parsed);
                first_uarch = Some(hart.uarch);
                first_isa_raw = isa_str;
            }
            Some(ref v0) if *v0 != parsed || first_uarch != Some(hart.uarch) => {
                return BackendSelection {
                    backend: NccBackendKind::Scalar,
                    reason: "cpuinfo:mixed_harts".to_string(),
                };
            }
            Some(_) => {}
        }
    }

    let v_ext = first_v.expect("harts is non-empty");
    let uarch = first_uarch.flatten();
    let dt_compat = probe
        .dt_compatible
        .as_deref()
        .map(parse_dt_compatibles)
        .unwrap_or_default();

    match v_ext {
        RiscvVExt::Draft0p7 => {
            if let Some(u) = uarch {
                if u != "thead,c906" {
                    return BackendSelection {
                        backend: NccBackendKind::Scalar,
                        reason: "uarch:unsupported".to_string(),
                    };
                }
            }
            if uarch == Some("thead,c906") || is_supported_sg2002_dt(&dt_compat) {
                BackendSelection {
                    backend: NccBackendKind::Xthead071,
                    reason: "detect:hwcap_v+isa_v0p7".to_string(),
                }
            } else {
                BackendSelection {
                    backend: NccBackendKind::Scalar,
                    reason: "hw:unrecognized_for_v0p7".to_string(),
                }
            }
        }
        RiscvVExt::Unversioned => {
            let kr_trimmed = probe.kernel_release.as_deref().map(str::trim);
            let allowed = uarch.is_some()
                && kr_trimmed.is_some()
                && allowlist.iter().any(|entry| {
                    Some(entry.uarch) == uarch
                        && kr_trimmed == Some(entry.kernel_release)
                        && dt_compat.contains(&entry.dt_compatible)
                });
            if allowed {
                BackendSelection {
                    backend: NccBackendKind::Xthead071,
                    reason: "detect:hwcap_v+unversioned_v_allowlist".to_string(),
                }
            } else {
                let isa_s = sanitize_field(first_isa_raw, 64);
                let uarch_s = uarch.map_or_else(|| "-".to_string(), |u| sanitize_field(u, 48));
                let compat_joined = if dt_compat.is_empty() {
                    "-".to_string()
                } else {
                    sanitize_field(&dt_compat.join("|"), 64)
                };
                let os_s = kr_trimmed.map_or_else(|| "-".to_string(), |k| sanitize_field(k, 48));
                BackendSelection {
                    backend: NccBackendKind::Scalar,
                    reason: format!(
                        "isa:unversioned_v(isa={isa_s};uarch={uarch_s};compat={compat_joined};osrelease={os_s})"
                    ),
                }
            }
        }
        RiscvVExt::OtherVersion(_, _) | RiscvVExt::Rvv1Subext => BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "isa:non_0p7_v".to_string(),
        },
        RiscvVExt::Conflict => BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "isa:conflicting_v".to_string(),
        },
        RiscvVExt::None => BackendSelection {
            backend: NccBackendKind::Scalar,
            reason: "isa:no_v".to_string(),
        },
    }
}

// ---- Injectable NccBackend & scalar implementations ----

/// Selectable NCC kernel backend (injectable in unit tests; cached per process via `active()`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NccBackend {
    Scalar,
    Xthead071,
}

impl NccBackend {
    #[inline]
    pub(crate) fn active() -> Self {
        match active_selection().backend {
            NccBackendKind::Scalar => Self::Scalar,
            NccBackendKind::Xthead071 => Self::Xthead071,
        }
    }

    #[inline]
    #[allow(dead_code)]
    pub(crate) const fn scalar() -> Self {
        Self::Scalar
    }

    #[inline]
    #[allow(dead_code)]
    pub(crate) const fn kind(self) -> NccBackendKind {
        match self {
            Self::Scalar => NccBackendKind::Scalar,
            Self::Xthead071 => NccBackendKind::Xthead071,
        }
    }

    pub(crate) fn coarse_dots(
        self,
        img: &Gray,
        x0: usize,
        y0: usize,
        a: &[f32],
        n: usize,
        out: &mut [f32],
    ) -> Result<(), PatchError> {
        let count = out.len();
        let view = validate_coarse_dots(img, x0, y0, a, n, count, out)?;
        match self {
            #[cfg(mview_has_xthead071)]
            Self::Xthead071 => {
                // SAFETY: `validate_coarse_dots` verified `a.len() == n * n`,
                // `out.len() == count > 0`, and `PatchView` bounds for `(n + count - 1) x n`
                // bytes starting at `view.as_ptr()` with byte stride `view.stride()`,
                // disjoint from `out`.
                unsafe {
                    ffi::mvs071_coarse_dots(
                        view.as_ptr(),
                        view.stride(),
                        a.as_ptr(),
                        n,
                        count,
                        out.as_mut_ptr(),
                    );
                }
                record_coarse_dots(NccBackendKind::Xthead071, count as u64);
                Ok(())
            }
            #[cfg(not(mview_has_xthead071))]
            Self::Xthead071 => {
                if n == 5 {
                    coarse_dots_scalar::<5>(&view, a, n, out);
                } else {
                    coarse_dots_scalar::<0>(&view, a, n, out);
                }
                record_coarse_dots(NccBackendKind::Scalar, count as u64);
                Ok(())
            }
            Self::Scalar => {
                if n == 5 {
                    coarse_dots_scalar::<5>(&view, a, n, out);
                } else {
                    coarse_dots_scalar::<0>(&view, a, n, out);
                }
                record_coarse_dots(NccBackendKind::Scalar, count as u64);
                Ok(())
            }
        }
    }

    pub(crate) fn half_moments(
        self,
        img: &Gray,
        x0: usize,
        y0: usize,
        a: &[f32],
        n: usize,
        phase: u8,
    ) -> Result<(u32, u32, f32), PatchError> {
        let view = validate_half_moments(img, x0, y0, a, n, phase)?;
        match self {
            #[cfg(mview_has_xthead071)]
            Self::Xthead071 => {
                let mut sum_raw = 0u32;
                let mut sumsq_raw = 0u32;
                let mut dot = 0.0f32;
                // SAFETY: `validate_half_moments` verified `a.len() == n * n`,
                // `n` is odd with `n / 2 <= HALF_GRID_MAX_R`, `phase <= 3`, and `PatchView`
                // bounds for `(n + extra_x) x (n + extra_y)` bytes starting at `view.as_ptr()`
                // with byte stride `view.stride()`. Output pointers are distinct locals.
                unsafe {
                    ffi::mvs071_half_moments(
                        view.as_ptr(),
                        view.stride(),
                        a.as_ptr(),
                        n,
                        u32::from(phase),
                        &mut sum_raw,
                        &mut sumsq_raw,
                        &mut dot,
                    );
                }
                record_half_moments(NccBackendKind::Xthead071);
                Ok((sum_raw, sumsq_raw, dot))
            }
            #[cfg(not(mview_has_xthead071))]
            Self::Xthead071 => {
                let res = half_moments_scalar_dispatch(&view, a, n, phase);
                record_half_moments(NccBackendKind::Scalar);
                Ok(res)
            }
            Self::Scalar => {
                let res = half_moments_scalar_dispatch(&view, a, n, phase);
                record_half_moments(NccBackendKind::Scalar);
                Ok(res)
            }
        }
    }

    pub(crate) fn patch_moments(
        self,
        img: &Gray,
        x0: usize,
        y0: usize,
        a: &[f32],
        weights: Option<&[f32]>,
        n: usize,
        bilerp: Option<[f32; 4]>,
    ) -> Result<(f64, f64, f32), PatchError> {
        let view = validate_patch_moments(img, x0, y0, a, weights, n, bilerp)?;
        match self {
            Self::Scalar | Self::Xthead071 => {
                let res = patch_moments_scalar(&view, a, weights, n, bilerp);
                record_patch_moments(NccBackendKind::Scalar);
                Ok(res)
            }
        }
    }
}

#[inline(always)]
#[allow(dead_code)]
fn coarse_dots_scalar<const N: usize>(view: &PatchView<'_>, a: &[f32], n_dyn: usize, out: &mut [f32]) {
    let n = if N > 0 { N } else { n_dyn };
    for (c, slot) in out.iter_mut().enumerate() {
        let mut dot = 0.0f32;
        for pj in 0..n {
            let row = &view.row(pj)[c..c + n];
            let ar = &a[pj * n..][..n];
            for pi in 0..n {
                dot += ar[pi] * f32::from(row[pi]);
            }
        }
        *slot = dot;
    }
}

#[inline(always)]
fn half_moments_scalar<const N: usize, const PHASE: u8>(
    view: &PatchView<'_>,
    a: &[f32],
    n_dyn: usize,
) -> (u32, u32, f32) {
    let n = if N > 0 { N } else { n_dyn };
    let (mut is1, mut is2, mut dot) = (0u32, 0u32, 0.0f32);
    for j in 0..n {
        let r0 = view.row(j);
        let r1 = if PHASE >= 2 { view.row(j + 1) } else { r0 };
        let ar = &a[j * n..][..n];
        for i in 0..n {
            let (s, sample) = match PHASE {
                0 => {
                    let s = u32::from(r0[i]);
                    (s, s as f32)
                }
                1 => {
                    let s = u32::from(r0[i]) + u32::from(r0[i + 1]);
                    (s, 0.5 * s as f32)
                }
                2 => {
                    let s = u32::from(r0[i]) + u32::from(r1[i]);
                    (s, 0.5 * s as f32)
                }
                _ => {
                    let s = u32::from(r0[i])
                        + u32::from(r0[i + 1])
                        + u32::from(r1[i])
                        + u32::from(r1[i + 1]);
                    (s, 0.25 * s as f32)
                }
            };
            is1 += s;
            is2 += s * s;
            dot += ar[i] * sample;
        }
    }
    (is1, is2, dot)
}

#[inline]
fn half_moments_scalar_dispatch(
    view: &PatchView<'_>,
    a: &[f32],
    n: usize,
    phase: u8,
) -> (u32, u32, f32) {
    const PATCH_N: usize = 2 * PATCH_R + 1;
    if n == PATCH_N {
        match phase {
            0 => half_moments_scalar::<PATCH_N, 0>(view, a, n),
            1 => half_moments_scalar::<PATCH_N, 1>(view, a, n),
            2 => half_moments_scalar::<PATCH_N, 2>(view, a, n),
            _ => half_moments_scalar::<PATCH_N, 3>(view, a, n),
        }
    } else {
        match phase {
            0 => half_moments_scalar::<0, 0>(view, a, n),
            1 => half_moments_scalar::<0, 1>(view, a, n),
            2 => half_moments_scalar::<0, 2>(view, a, n),
            _ => half_moments_scalar::<0, 3>(view, a, n),
        }
    }
}

#[inline]
fn patch_moments_scalar(
    view: &PatchView<'_>,
    a: &[f32],
    weights: Option<&[f32]>,
    n: usize,
    bilerp: Option<[f32; 4]>,
) -> (f64, f64, f32) {
    let (mut s1, mut s2, mut dot) = (0.0f64, 0.0f64, 0.0f32);
    if let Some([w00, w10, w01, w11]) = bilerp {
        for j in 0..n {
            let r0 = view.row(j);
            let r1 = view.row(j + 1);
            let ar = &a[j * n..(j + 1) * n];
            match weights {
                None => {
                    for i in 0..n {
                        let v = w00 * f32::from(r0[i])
                            + w10 * f32::from(r0[i + 1])
                            + w01 * f32::from(r1[i])
                            + w11 * f32::from(r1[i + 1]);
                        s1 += f64::from(v);
                        s2 += f64::from(v * v);
                        dot += ar[i] * v;
                    }
                }
                Some(w) => {
                    let wr = &w[j * n..(j + 1) * n];
                    for i in 0..n {
                        let v = w00 * f32::from(r0[i])
                            + w10 * f32::from(r0[i + 1])
                            + w01 * f32::from(r1[i])
                            + w11 * f32::from(r1[i + 1]);
                        let wv = wr[i] * v;
                        s1 += f64::from(wv);
                        s2 += f64::from(wv * v);
                        dot += ar[i] * v;
                    }
                }
            }
        }
    } else {
        for j in 0..n {
            let row = view.row(j);
            let ar = &a[j * n..(j + 1) * n];
            match weights {
                None => {
                    for (&p, &ak) in row.iter().zip(ar) {
                        let v = f32::from(p);
                        s1 += f64::from(v);
                        s2 += f64::from(v * v);
                        dot += ak * v;
                    }
                }
                Some(w) => {
                    let wr = &w[j * n..(j + 1) * n];
                    for ((&p, &ak), &wk) in row.iter().zip(ar).zip(wr) {
                        let v = f32::from(p);
                        let wv = wk * v;
                        s1 += f64::from(wv);
                        s2 += f64::from(wv * v);
                        dot += ak * v;
                    }
                }
            }
        }
    }
    (s1, s2, dot)
}

#[cfg(test)]
pub(crate) fn assert_coarse_dots_match_scalar(
    backend: NccBackend,
    img: &Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    n: usize,
    count: usize,
) {
    let mut exp = vec![f32::NAN; count];
    let mut got = vec![f32::NAN; count];
    NccBackend::scalar()
        .coarse_dots(img, x0, y0, a, n, &mut exp)
        .expect("scalar coarse_dots should succeed");
    backend
        .coarse_dots(img, x0, y0, a, n, &mut got)
        .expect("candidate coarse_dots should succeed");
    for (idx, (&g, &e)) in got.iter().zip(&exp).enumerate() {
        assert_eq!(
            g.to_bits(),
            e.to_bits(),
            "coarse_dots[{idx}] mismatch: got={g:?} exp={e:?}"
        );
    }
}

#[cfg(test)]
pub(crate) fn assert_half_moments_match_scalar(
    backend: NccBackend,
    img: &Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    n: usize,
    phase: u8,
) {
    let exp = NccBackend::scalar()
        .half_moments(img, x0, y0, a, n, phase)
        .expect("scalar half_moments should succeed");
    let got = backend
        .half_moments(img, x0, y0, a, n, phase)
        .expect("candidate half_moments should succeed");
    assert_eq!(got.0, exp.0, "half_moments sum_raw mismatch");
    assert_eq!(got.1, exp.1, "half_moments sumsq_raw mismatch");
    assert_eq!(
        got.2.to_bits(),
        exp.2.to_bits(),
        "half_moments dot mismatch: got={:?} exp={:?}",
        got.2,
        exp.2
    );
}

#[cfg(test)]
pub(crate) fn assert_patch_moments_match_scalar(
    backend: NccBackend,
    img: &Gray,
    x0: usize,
    y0: usize,
    a: &[f32],
    weights: Option<&[f32]>,
    n: usize,
    bilerp: Option<[f32; 4]>,
) {
    let exp = NccBackend::scalar()
        .patch_moments(img, x0, y0, a, weights, n, bilerp)
        .expect("scalar patch_moments should succeed");
    let got = backend
        .patch_moments(img, x0, y0, a, weights, n, bilerp)
        .expect("candidate patch_moments should succeed");
    assert_eq!(
        got.0.to_bits(),
        exp.0.to_bits(),
        "patch_moments sum mismatch: got={} exp={}",
        got.0,
        exp.0
    );
    assert_eq!(
        got.1.to_bits(),
        exp.1.to_bits(),
        "patch_moments sumsq mismatch: got={} exp={}",
        got.1,
        exp.1
    );
    assert_eq!(
        got.2.to_bits(),
        exp.2.to_bits(),
        "patch_moments dot mismatch: got={} exp={}",
        got.2,
        exp.2
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backend_diagnostics_reports_scalar_and_resets_counts() {
        reset_kernel_counts();
        let active = NccBackend::active();
        let initial = backend_diagnostics();
        assert_eq!(initial.backend, active.kind().as_str());
        if !HAS_COMPILED_XTHEAD071 {
            assert_eq!(initial.backend, "scalar");
            assert_eq!(initial.reason, "scalar-only");
        }
        assert_eq!(initial.counts, KernelCallCounts::default());
        assert_eq!(NccBackendKind::Scalar.as_str(), "scalar");
        assert_eq!(NccBackendKind::Xthead071.as_str(), "xthead071");

        record_coarse_dots(NccBackendKind::Scalar, 5);
        record_coarse_dots_batch(NccBackendKind::Scalar, 3, 12);
        record_half_moments(NccBackendKind::Scalar);
        record_half_moments(NccBackendKind::Scalar);
        record_patch_moments(NccBackendKind::Scalar);

        let mid = backend_diagnostics();
        assert_eq!(mid.backend, active.kind().as_str());
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

    fn encode_auxv(pairs: &[(u64, u64)]) -> Vec<u8> {
        let mut out = Vec::with_capacity(pairs.len() * 16);
        for &(k, v) in pairs {
            out.extend_from_slice(&k.to_le_bytes());
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    fn valid_sg2002_probe(isa: &str) -> IsaProbe {
        IsaProbe {
            auxv: Some(encode_auxv(&[(16, 1u64 << 21), (0, 0)])),
            cpuinfo: Some(format!("processor\t: 0\nhart\t\t: 0\nisa\t\t: {isa}\nuarch\t\t: thead,c906\n")),
            dt_compatible: Some(b"sipeed,licheerv-nano\0sophgo,sg2002\0".to_vec()),
            kernel_release: Some("5.10.4-tag-\n".to_string()),
        }
    }

    #[test]
    fn patch_view_rejects_truncated_row_wrap_overflow_and_accepts_exact_margins() {
        let (w, h) = (10usize, 8usize);
        let px: Vec<u8> = (0..w * h).map(|k| k as u8).collect();
        let img = Gray::new(w, h, px.clone()).unwrap();

        // Exact margins: (0, 0) and (w - W, h - H).
        let tl = PatchView::new(&img, 0, 0, 5, 5).unwrap();
        assert_eq!(tl.width(), 5);
        assert_eq!(tl.height(), 5);
        assert_eq!(tl.stride(), w);
        assert_eq!(tl.as_ptr(), img.px.as_ptr());
        assert_eq!(tl.row(0), &[0, 1, 2, 3, 4]);
        assert_eq!(tl.row(4), &[40, 41, 42, 43, 44]);

        let br = PatchView::new(&img, w - 5, h - 5, 5, 5).unwrap();
        assert_eq!(br.row(0), &[35, 36, 37, 38, 39]);
        assert_eq!(br.row(4), &[75, 76, 77, 78, 79]);

        // 1-past-margin rejection.
        assert_eq!(PatchView::new(&img, w - 4, h - 5, 5, 5).unwrap_err(), PatchError::OutOfBounds);
        assert_eq!(PatchView::new(&img, w - 5, h - 4, 5, 5).unwrap_err(), PatchError::OutOfBounds);

        // Empty dimensions.
        assert_eq!(PatchView::new(&img, 0, 0, 0, 5).unwrap_err(), PatchError::EmptyDimensions);
        assert_eq!(PatchView::new(&img, 0, 0, 5, 0).unwrap_err(), PatchError::EmptyDimensions);
        let zero_img = Gray { w: 0, h: 8, px: vec![] };
        assert_eq!(PatchView::new(&zero_img, 0, 0, 1, 1).unwrap_err(), PatchError::EmptyDimensions);

        // Row wrap despite sufficient total bytes in px: x0 + W > img.w while base + W <= px.len().
        assert_eq!(PatchView::new(&img, 7, 1, 5, 2).unwrap_err(), PatchError::OutOfBounds);

        // Truncated Gray.px buffer: geometric coords fit w x h, and even base + W <= px.len(),
        // but base + (H - 1) * stride + W > px.len().
        let trunc = Gray { w: 10, h: 8, px: vec![42u8; 20] };
        assert_eq!(PatchView::new(&trunc, 0, 0, 5, 3).unwrap_err(), PatchError::TruncatedBuffer);
        // Fits within the first 15 bytes (0..10 + 0..5 = 15 <= 20), even if rest of image is truncated.
        assert!(PatchView::new(&trunc, 0, 0, 5, 2).is_ok());

        // Arithmetic overflow & out-of-bounds.
        assert_eq!(PatchView::new(&img, usize::MAX, 0, 1, 1).unwrap_err(), PatchError::Overflow);
        assert_eq!(PatchView::new(&img, 0, usize::MAX, 1, 1).unwrap_err(), PatchError::Overflow);
        assert_eq!(PatchView::new(&img, 1, 0, usize::MAX, 1).unwrap_err(), PatchError::Overflow);
        assert_eq!(PatchView::new(&img, 0, 1, 1, usize::MAX).unwrap_err(), PatchError::Overflow);
        assert_eq!(PatchView::new(&img, 0, 0, usize::MAX, 1).unwrap_err(), PatchError::OutOfBounds);
        let huge_img = Gray { w: usize::MAX, h: usize::MAX, px: vec![] };
        assert_eq!(PatchView::new(&huge_img, 0, 2, 1, 1).unwrap_err(), PatchError::Overflow);
    }

    #[test]
    fn kernel_validators_reject_malformed_templates_phases_counts_and_overflows() {
        let img = Gray::new(40, 40, vec![100u8; 40 * 40]).unwrap();
        let a5 = vec![0.1f32; 25];
        let a9 = vec![0.1f32; 81];

        // coarse_dots
        let out0 = Vec::<f32>::new();
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &a5, 5, 0, &out0).unwrap_err(),
            PatchError::InvalidCount
        );
        let out3 = vec![0.0f32; 3];
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &a5, 5, 2, &out3).unwrap_err(),
            PatchError::InvalidCount
        );
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &[], 0, 1, &out3[..1]).unwrap_err(),
            PatchError::InvalidTemplate
        );
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &[0.0; 16], 4, 1, &out3[..1]).unwrap_err(),
            PatchError::InvalidTemplate
        );
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &a5[..24], 5, 1, &out3[..1]).unwrap_err(),
            PatchError::InvalidTemplate
        );
        // Overflow in n + count - 1
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &a5, 5, usize::MAX, &out3).unwrap_err(),
            PatchError::InvalidCount
        );
        assert_eq!(
            validate_coarse_dots(&img, 0, 0, &a5, usize::MAX, 1, &out0).unwrap_err(),
            PatchError::Overflow
        );
        // Exact right margin for count=3, n=5 => width=7, x0=33 fits, x0=34 fails.
        assert!(validate_coarse_dots(&img, 33, 35, &a5, 5, 3, &out3).is_ok());
        assert_eq!(
            validate_coarse_dots(&img, 34, 35, &a5, 5, 3, &out3).unwrap_err(),
            PatchError::OutOfBounds
        );

        // half_moments
        assert_eq!(
            validate_half_moments(&img, 0, 0, &a9, 9, 4).unwrap_err(),
            PatchError::InvalidPhase
        );
        let n17 = 2 * (HALF_GRID_MAX_R + 1) + 1; // 35, r = 17 > 16
        let a35 = vec![0.0f32; n17 * n17];
        assert_eq!(
            validate_half_moments(&img, 0, 0, &a35, n17, 0).unwrap_err(),
            PatchError::RadiusTooLarge
        );
        // Phase margins: for n=9 in 40x40, phase 0 allows x0=31, y0=31; phase 1/3 needs width=10 (x0<=30).
        assert!(validate_half_moments(&img, 31, 31, &a9, 9, 0).is_ok());
        assert_eq!(
            validate_half_moments(&img, 31, 31, &a9, 9, 1).unwrap_err(),
            PatchError::OutOfBounds
        );
        assert_eq!(
            validate_half_moments(&img, 31, 31, &a9, 9, 2).unwrap_err(),
            PatchError::OutOfBounds
        );
        assert!(validate_half_moments(&img, 30, 30, &a9, 9, 3).is_ok());

        // patch_moments
        assert_eq!(
            validate_patch_moments(&img, 0, 0, &a9, Some(&[1.0; 80]), 9, None).unwrap_err(),
            PatchError::InvalidTemplate
        );
        assert!(validate_patch_moments(&img, 31, 31, &a9, Some(&a9), 9, None).is_ok());
        assert_eq!(
            validate_patch_moments(&img, 31, 31, &a9, None, 9, Some([1.0, 0.0, 0.0, 0.0])).unwrap_err(),
            PatchError::OutOfBounds
        );
        assert!(validate_patch_moments(&img, 30, 30, &a9, Some(&a9), 9, Some([1.0, 0.0, 0.0, 0.0])).is_ok());

        let _ = out0;
    }

    #[test]
    fn isa_detector_accepts_valid_sg2002_v0p7_and_handles_env_overrides() {
        for isa in [
            "rv64imafdcv0p7",
            "rv64gcv0p7",
            "rv64i2p0m2p0a2p0f2p0d2p0c2p0v0p7",
            "rv64i2p0_m2p0_a2p0_f2p0_d2p0_c2p0_v0p7_zvamo0p7_zvlsseg0p7",
            "rv64imafdc_v0p7_xtheadba",
            "rv64imafdcv0p7p1",
        ] {
            let probe = valid_sg2002_probe(isa);
            let sel_none = detect_backend_with_allowlist(None, true, &probe, &[]);
            assert_eq!(sel_none.backend, NccBackendKind::Xthead071, "isa={isa}: {sel_none:?}");
            assert_eq!(sel_none.reason, "detect:hwcap_v+isa_v0p7");

            let sel_auto = detect_backend_with_allowlist(Some(Ok("auto")), true, &probe, &[]);
            assert_eq!(sel_auto.backend, NccBackendKind::Xthead071);

            let sel_scalar = detect_backend_with_allowlist(Some(Ok("scalar")), true, &probe, &[]);
            assert_eq!(sel_scalar.backend, NccBackendKind::Scalar);
            assert_eq!(sel_scalar.reason, "env:scalar");

            // Unknown env values select scalar and record clear reason.
            for (val, expected) in [
                ("xthead071", "env:unknown:xthead071"),
                ("foo", "env:unknown:foo"),
                ("", "env:unknown:<empty>"),
            ] {
                let sel_unk = detect_backend_with_allowlist(Some(Ok(val)), true, &probe, &[]);
                assert_eq!(sel_unk.backend, NccBackendKind::Scalar);
                assert_eq!(sel_unk.reason, expected);
            }
            let sel_non_utf8 = detect_backend_with_allowlist(Some(Err(())), true, &probe, &[]);
            assert_eq!(sel_non_utf8.backend, NccBackendKind::Scalar);
            assert_eq!(sel_non_utf8.reason, "env:unknown:non_utf8");
        }
    }

    #[test]
    fn isa_detector_rejects_missing_hwcap_plain_v_vendor_only_rvv1_and_mixed_harts() {
        let base = valid_sg2002_probe("rv64imafdcv0p7");

        // Missing / bad / no-V auxv / AT_NULL before AT_HWCAP.
        let mut p = base.clone();
        p.auxv = None;
        assert_eq!(detect_backend_with_allowlist(None, true, &p, &[]).backend, NccBackendKind::Scalar);

        p.auxv = Some(vec![0u8; 10]); // truncated
        assert_eq!(detect_backend_with_allowlist(None, true, &p, &[]).backend, NccBackendKind::Scalar);

        p.auxv = Some(encode_auxv(&[(16, 0), (0, 0)])); // V bit 21 unset
        assert_eq!(detect_backend_with_allowlist(None, true, &p, &[]).backend, NccBackendKind::Scalar);

        p.auxv = Some(encode_auxv(&[(0, 0), (16, 1u64 << 21)])); // AT_NULL before AT_HWCAP
        assert_eq!(detect_backend_with_allowlist(None, true, &p, &[]).backend, NccBackendKind::Scalar);

        // Plain unversioned `v` rejected without allowlist, accepted only on exact allowlist match.
        for plain_isa in ["rv64imafdcv", "rv64imafdcvsu"] {
            let p_plain = valid_sg2002_probe(plain_isa);
            let sel = detect_backend_with_allowlist(None, true, &p_plain, &[]);
            assert_eq!(sel.backend, NccBackendKind::Scalar);
            assert!(sel.reason.starts_with("isa:unversioned_v("), "{sel:?}");
            assert!(sel.reason.contains("sophgo,sg2002"), "{sel:?}");
            assert!(sel.reason.contains("thead,c906"), "{sel:?}");
            assert!(sel.reason.contains("5.10.4-tag-"), "{sel:?}");

            let allow = [UnversionedVAllowlistEntry {
                dt_compatible: "sophgo,sg2002",
                uarch: "thead,c906",
                kernel_release: "5.10.4-tag-",
            }];
            assert_eq!(
                detect_backend_with_allowlist(None, true, &p_plain, &allow).backend,
                NccBackendKind::Xthead071
            );

            // Mismatched kernel / compat / uarch rejected even with allowlist.
            let mut bad_kern = p_plain.clone();
            bad_kern.kernel_release = Some("6.6.0".to_string());
            assert_eq!(
                detect_backend_with_allowlist(None, true, &bad_kern, &allow).backend,
                NccBackendKind::Scalar
            );

            let mut bad_compat = p_plain.clone();
            bad_compat.dt_compatible = Some(b"sophgo,sg2002-rev2\0".to_vec());
            assert_eq!(
                detect_backend_with_allowlist(None, true, &bad_compat, &allow).backend,
                NccBackendKind::Scalar
            );

            let mut bad_uarch = p_plain.clone();
            bad_uarch.cpuinfo = Some(format!("processor : 0\nisa : {plain_isa}\nuarch : thead,c910\n"));
            assert_eq!(
                detect_backend_with_allowlist(None, true, &bad_uarch, &allow).backend,
                NccBackendKind::Scalar
            );
        }

        // T-Head vendor alone without v0p7.
        let p_vendor = valid_sg2002_probe("rv64imafdc_xtheadcmo");
        assert_eq!(detect_backend_with_allowlist(None, true, &p_vendor, &[]).backend, NccBackendKind::Scalar);

        // RVV 1.0 and conflicting strings.
        for rvv1_isa in [
            "rv64imafdcv1p0",
            "rv64imafdc_v1p0",
            "rv64imafdc_zve64d",
            "rv64imafdc_zvl128b",
            "rv64imafdcv0p7_zve64d",
            "rv64imafdcv0p7_v1p0",
            "rv32imafdcv0p7",
        ] {
            let p_rvv1 = valid_sg2002_probe(rvv1_isa);
            assert_eq!(
                detect_backend_with_allowlist(None, true, &p_rvv1, &[]).backend,
                NccBackendKind::Scalar,
                "must reject {rvv1_isa}"
            );
        }

        // Mixed harts: ISA disagreement and uarch disagreement.
        for second_hart in [
            "processor : 1\nisa : rv64imafdc\nuarch : thead,c906\n",
            "processor : 1\nisa : rv64imafdcv1p0\nuarch : thead,c906\n",
            "processor : 1\nisa : rv64imafdcv\nuarch : thead,c906\n",
            "processor : 1\nisa : rv64imafdcv0p7\nuarch : sifive,u74\n",
        ] {
            let mut mixed = base.clone();
            mixed.cpuinfo = Some(format!(
                "processor : 0\nisa : rv64imafdcv0p7\nuarch : thead,c906\n\n{second_hart}"
            ));
            let sel = detect_backend_with_allowlist(None, true, &mixed, &[]);
            assert_eq!(sel.backend, NccBackendKind::Scalar);
            assert_eq!(sel.reason, "cpuinfo:mixed_harts");
        }

        // Non-SG2002/C906 uarch even with v0p7 and SG2002 DT compatible.
        let mut wrong_uarch = base.clone();
        wrong_uarch.cpuinfo = Some("processor : 0\nisa : rv64imafdcv0p7\nuarch : sifive,u74\n".to_string());
        assert_eq!(
            detect_backend_with_allowlist(None, true, &wrong_uarch, &[]).backend,
            NccBackendKind::Scalar
        );

        // NUL-separated DT compatibles parsing & exact match (not substring).
        assert_eq!(
            parse_dt_compatibles(b"vendor,board\0sophgo,sg2002-rev2\0\0"),
            vec!["vendor,board", "sophgo,sg2002-rev2"]
        );
        let mut no_uarch_bad_dt = base.clone();
        no_uarch_bad_dt.cpuinfo = Some("processor : 0\nisa : rv64imafdcv0p7\n".to_string());
        no_uarch_bad_dt.dt_compatible = Some(b"sophgo,sg2002-rev2\0".to_vec());
        assert_eq!(
            detect_backend_with_allowlist(None, true, &no_uarch_bad_dt, &[]).backend,
            NccBackendKind::Scalar
        );
        no_uarch_bad_dt.dt_compatible = Some(b"vendor,board\0cvitek,cv181x\0".to_vec());
        assert_eq!(
            detect_backend_with_allowlist(None, true, &no_uarch_bad_dt, &[]).backend,
            NccBackendKind::Xthead071
        );
    }

    #[test]
    fn direct_oracle_comparison_helpers_match_scalar_bit_exact() {
        let (w, h) = (32usize, 32usize);
        let px: Vec<u8> = (0..w * h)
            .map(|k| ((k * 37 + (k / w) * 53) % 210 + 20) as u8)
            .collect();
        let img = Gray::new(w, h, px).unwrap();
        let active = NccBackend::active();

        let a5: Vec<f32> = (0..25).map(|i| (i as f32 - 12.0) * 0.05).collect();
        assert_coarse_dots_match_scalar(NccBackend::scalar(), &img, 3, 4, &a5, 5, 11);
        assert_coarse_dots_match_scalar(active, &img, 3, 4, &a5, 5, 11);
        if !HAS_COMPILED_XTHEAD071 {
            assert_coarse_dots_match_scalar(NccBackend::Xthead071, &img, 3, 4, &a5, 5, 11);
        }

        let a9: Vec<f32> = (0..81).map(|i| (i as f32 - 40.0) * 0.02).collect();
        let w9: Vec<f32> = (0..81).map(|i| 0.1 + (i % 9) as f32 * 0.1).collect();
        for phase in 0..=3 {
            assert_half_moments_match_scalar(NccBackend::scalar(), &img, 5, 6, &a9, 9, phase);
            assert_half_moments_match_scalar(active, &img, 5, 6, &a9, 9, phase);
            if !HAS_COMPILED_XTHEAD071 {
                assert_half_moments_match_scalar(NccBackend::Xthead071, &img, 5, 6, &a9, 9, phase);
            }
        }
        for (weights, bilerp) in [
            (None, None),
            (Some(w9.as_slice()), None),
            (None, Some([0.21f32, 0.49, 0.09, 0.21])),
            (Some(w9.as_slice()), Some([0.21f32, 0.49, 0.09, 0.21])),
        ] {
            assert_patch_moments_match_scalar(
                NccBackend::scalar(),
                &img,
                5,
                6,
                &a9,
                weights,
                9,
                bilerp,
            );
            assert_patch_moments_match_scalar(active, &img, 5, 6, &a9, weights, 9, bilerp);
            if !HAS_COMPILED_XTHEAD071 {
                assert_patch_moments_match_scalar(
                    NccBackend::Xthead071,
                    &img,
                    5,
                    6,
                    &a9,
                    weights,
                    9,
                    bilerp,
                );
            }
        }
    }

    #[test]
    fn coarse_dots_matches_scalar_bit_exact_across_sizes_and_vector_lengths() {
        let (w, h) = (97usize, 23usize);
        let mut px: Vec<u8> = (0..w * h)
            .map(|k| ((k * 73 + (k / w) * 139 + 17) % 256) as u8)
            .collect();
        // Include extreme 0 and 255 stripes across multiple rows.
        for x in 1..40 {
            px[5 * w + x] = 255;
            px[6 * w + x] = 0;
        }
        let img = Gray::new(w, h, px).unwrap();
        let active = NccBackend::active();

        for n in [3usize, 5, 7] {
            let mut a = Vec::with_capacity(n * n);
            let mean = ((n * n - 1) as f32) * 0.5;
            for i in 0..(n * n) {
                let centered = i as f32 - mean;
                a.push(centered * 0.0375 + if i % 2 == 0 { 0.01 } else { -0.015 });
            }

            for count in [1usize, 2, 4, 5, 15, 16, 17, 31, 32, 33, 48, 65] {
                let x0 = (count % 5) + 1; // exercises unaligned byte offsets
                let y0 = 3 + (n % 3);
                assert_coarse_dots_match_scalar(active, &img, x0, y0, &a, n, count);
                if !HAS_COMPILED_XTHEAD071 {
                    assert_coarse_dots_match_scalar(
                        NccBackend::Xthead071,
                        &img,
                        x0,
                        y0,
                        &a,
                        n,
                        count,
                    );
                }
            }

            // Right-margin boundary check for a multi-VL run.
            let count = 33usize;
            let right_x0 = w - (n + count - 1);
            let bot_y0 = h - n;
            assert_coarse_dots_match_scalar(active, &img, right_x0, bot_y0, &a, n, count);
            if !HAS_COMPILED_XTHEAD071 {
                assert_coarse_dots_match_scalar(
                    NccBackend::Xthead071,
                    &img,
                    right_x0,
                    bot_y0,
                    &a,
                    n,
                    count,
                );
            }
        }

        // Verify per-kernel counter attribution for active backend.
        reset_kernel_counts();
        let a5 = vec![0.04f32; 25];
        let mut out = vec![0.0f32; 17];
        active.coarse_dots(&img, 2, 4, &a5, 5, &mut out).unwrap();
        let counts = kernel_counts();
        match active {
            NccBackend::Scalar => {
                assert_eq!(counts.coarse_dots_scalar, 1);
                assert_eq!(counts.coarse_cells_scalar, 17);
                assert_eq!(counts.coarse_dots_xthead071, 0);
                assert_eq!(counts.coarse_cells_xthead071, 0);
            }
            NccBackend::Xthead071 => {
                if HAS_COMPILED_XTHEAD071 {
                    assert_eq!(counts.coarse_dots_xthead071, 1);
                    assert_eq!(counts.coarse_cells_xthead071, 17);
                    assert_eq!(counts.coarse_dots_scalar, 0);
                    assert_eq!(counts.coarse_cells_scalar, 0);
                } else {
                    assert_eq!(counts.coarse_dots_scalar, 1);
                    assert_eq!(counts.coarse_cells_scalar, 17);
                }
            }
        }
    }

    #[test]
    fn half_moments_matches_scalar_across_phases_radii_and_extremes() {
        let (w, h) = (73usize, 73usize);
        let mut px: Vec<u8> = (0..w * h)
            .map(|k| ((k * 59 + (k / w) * 113 + 31) % 256) as u8)
            .collect();
        // Bright nearly-flat region (240..=255) and flat-255 region.
        for y in 4..39 {
            for x in 4..39 {
                px[y * w + x] = 240 + (((x * 11 + y * 17) % 16) as u8);
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let flat_255 = Gray::new(w, h, vec![255u8; w * h]).unwrap();
        let alternating_max = Gray::new(
            w,
            h,
            (0..w * h)
                .map(|k| if ((k % w) + (k / w)).is_multiple_of(2) { 255 } else { 254 })
                .collect(),
        )
        .unwrap();
        let active = NccBackend::active();

        for r in [2usize, 4, 6, 9, HALF_GRID_MAX_R] {
            let n = 2 * r + 1;
            let mut a = Vec::with_capacity(n * n);
            let mean = ((n * n - 1) as f32) * 0.5;
            for i in 0..(n * n) {
                let centered = i as f32 - mean;
                a.push(centered * 0.0125 + if i % 3 == 0 { 0.007 } else { -0.009 });
            }

            for phase in 0..=3u8 {
                let extra_x = usize::from(phase == 1 || phase == 3);
                let extra_y = usize::from(phase == 2 || phase == 3);
                for (x0, y0) in [
                    (0usize, 0usize),
                    (5, 5), // bright nearly-flat
                    (3 + usize::from(phase), 7),
                    (w - n - extra_x, h - n - extra_y), // exact bottom-right margin
                ] {
                    assert_half_moments_match_scalar(active, &img, x0, y0, &a, n, phase);
                    assert_half_moments_match_scalar(active, &flat_255, x0, y0, &a, n, phase);
                    assert_half_moments_match_scalar(active, &alternating_max, x0, y0, &a, n, phase);
                    if !HAS_COMPILED_XTHEAD071 {
                        assert_half_moments_match_scalar(
                            NccBackend::Xthead071,
                            &img,
                            x0,
                            y0,
                            &a,
                            n,
                            phase,
                        );
                    }
                }
            }
        }

        // Verify per-kernel counter attribution for active backend.
        reset_kernel_counts();
        let a9: Vec<f32> = (0..81).map(|i| (i as f32 - 40.0) * 0.02).collect();
        for phase in 0..=3u8 {
            active.half_moments(&img, 2, 3, &a9, 9, phase).unwrap();
        }
        let counts = kernel_counts();
        match active {
            NccBackend::Scalar => {
                assert_eq!(counts.half_moments_scalar, 4);
                assert_eq!(counts.half_moments_xthead071, 0);
            }
            NccBackend::Xthead071 => {
                if HAS_COMPILED_XTHEAD071 {
                    assert_eq!(counts.half_moments_xthead071, 4);
                    assert_eq!(counts.half_moments_scalar, 0);
                } else {
                    assert_eq!(counts.half_moments_scalar, 4);
                    assert_eq!(counts.half_moments_xthead071, 0);
                }
            }
        }
    }
}
