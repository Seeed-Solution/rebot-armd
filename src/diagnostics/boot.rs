//! Boot record: one line per armd start in `boots.log`, with the last black-box line of
//! the previous start, so a reset shows what the arm was doing right before it.

use std::fs::OpenOptions;
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::Path;

use serde_json::{json, Value};

use super::blackbox::{BB_FILE, BB_KEEP};
use super::rotating::{files_oldest_first, RotatingFile};

pub const BOOTS_FILE: &str = "boots.log";
pub const BOOTS_MAX_BYTES: u64 = 256 * 1024;
pub const BOOTS_KEEP: usize = 1;
const COUNTER_FILE: &str = "boot_count";

/// What [`record_boot`] wrote.
#[derive(Debug, Clone)]
pub struct BootRecord {
    /// This start's counter (`boot` in every black-box line).
    pub boot: u64,
    /// The boots.log line.
    pub entry: Value,
}

/// System facts at armd start; [`SystemInfo::read`] fills them on Linux.
#[derive(Debug, Clone, Default)]
pub struct SystemInfo {
    /// `/proc/uptime`, seconds.
    pub uptime_s: Option<f64>,
    /// `/proc/sys/kernel/random/boot_id`: changes on every kernel boot.
    pub kernel_boot_id: Option<String>,
    /// SoC power-on reason, when the platform exposes it.
    pub reset_reason: Option<String>,
}

impl SystemInfo {
    pub fn read() -> Self {
        let uptime_s = std::fs::read_to_string("/proc/uptime")
            .ok()
            .and_then(|s| s.split_whitespace().next().and_then(|v| v.parse().ok()));
        let kernel_boot_id = std::fs::read_to_string("/proc/sys/kernel/random/boot_id").ok().map(|s| s.trim().to_string());
        Self { uptime_s, kernel_boot_id, reset_reason: read_reset_reason() }
    }
}

/// SG2002 / CV181x: the `cvi-base` driver's `IOCTL_READ_CHIP_PWR_ON_REASON`
/// (`_IOR('s', 7, unsigned int)`), decoded from the RTC `ST_ON_REASON` register
/// (`osdrv/interdrv/base/base.c`, `cvi_base_read_chip_pwr_on_reason`). The driver maps
/// unknown register values to cold boot.
#[cfg(target_os = "linux")]
pub fn read_reset_reason() -> Option<String> {
    use std::os::fd::AsRawFd;
    const IOCTL_READ_CHIP_PWR_ON_REASON: u32 = (2 << 30) | (4 << 16) | ((b's' as u32) << 8) | 7;
    let file = std::fs::File::open("/dev/cvi-base").ok()?;
    let mut reason: libc::c_uint = 0;
    // SAFETY: the ioctl writes one unsigned int into `reason`, which outlives the call.
    let rc = unsafe { libc::ioctl(file.as_raw_fd(), IOCTL_READ_CHIP_PWR_ON_REASON as _, &mut reason as *mut libc::c_uint) };
    if rc != 0 {
        return None;
    }
    Some(match reason {
        1 => "cold_boot".to_string(),
        2 => "watchdog".to_string(),
        3 => "suspend".to_string(),
        4 => "warm_reset".to_string(),
        other => format!("unknown_{other}"),
    })
}

#[cfg(not(target_os = "linux"))]
pub fn read_reset_reason() -> Option<String> {
    None
}

/// Last complete JSON line of a file (reads at most the last 8 KiB).
pub fn last_json_line(path: &Path) -> Option<Value> {
    let mut f = std::fs::File::open(path).ok()?;
    let len = f.metadata().ok()?.len();
    let start = len.saturating_sub(8 * 1024);
    f.seek(SeekFrom::Start(start)).ok()?;
    let mut buf = Vec::new();
    f.read_to_end(&mut buf).ok()?;
    // A power cut can leave a partial last line: take the last one that parses.
    String::from_utf8_lossy(&buf).lines().rev().find_map(|l| serde_json::from_str::<Value>(l).ok().filter(Value::is_object))
}

/// Last black-box record in `dir` (newest file first).
pub fn last_blackbox_record(dir: &Path) -> Option<Value> {
    files_oldest_first(&dir.join(BB_FILE), BB_KEEP).iter().rev().find_map(|p| last_json_line(p))
}

fn write_counter(dir: &Path, n: u64) -> io::Result<()> {
    let tmp = dir.join(format!("{COUNTER_FILE}.tmp"));
    let mut f = std::fs::File::create(&tmp)?;
    writeln!(f, "{n}")?;
    f.sync_all()?;
    std::fs::rename(&tmp, dir.join(COUNTER_FILE))
}

/// Increments the persisted start counter, and appends (and syncs) this start's line to
/// `boots.log`. Call before the black box writes its first line.
pub fn record_boot(dir: &Path, sys: &SystemInfo) -> io::Result<BootRecord> {
    let stored = std::fs::read_to_string(dir.join(COUNTER_FILE)).ok().and_then(|s| s.trim().parse::<u64>().ok());
    let boots_path = dir.join(BOOTS_FILE);
    let prev_entry = files_oldest_first(&boots_path, BOOTS_KEEP).iter().rev().find_map(|p| last_json_line(p));
    // A lost counter file continues from boots.log.
    let prev_boot = stored.or_else(|| prev_entry.as_ref().and_then(|e| e["boot"].as_u64())).unwrap_or(0);
    let boot = prev_boot + 1;
    write_counter(dir, boot)?;

    let prev_last = last_blackbox_record(dir);
    let prev_kernel = prev_entry.as_ref().and_then(|e| e["kernel_boot_id"].as_str().map(str::to_string));
    let same_kernel_boot = match (&prev_kernel, &sys.kernel_boot_id) {
        (Some(a), Some(b)) => Some(a == b),
        _ => None,
    };
    let entry = json!({
        "boot": boot,
        "uptime_s": sys.uptime_s,
        "kernel_boot_id": sys.kernel_boot_id,
        // false: the kernel booted since the previous armd start (a reset or reboot);
        // true: armd restarted without a reboot; null: unknown.
        "same_kernel_boot": same_kernel_boot,
        "reset_reason": sys.reset_reason,
        "armd_version": env!("CARGO_PKG_VERSION"),
        "prev_last": prev_last,
    });
    let mut log = RotatingFile::open(&boots_path, BOOTS_MAX_BYTES, BOOTS_KEEP)?;
    let mut line = entry.to_string();
    line.push('\n');
    log.write_record(line.as_bytes())?;
    log.sync_data()?;
    // The directory entry of a new file.
    if let Ok(d) = OpenOptions::new().read(true).open(dir) {
        let _ = d.sync_all();
    }
    Ok(BootRecord { boot, entry })
}

/// One-line summary of the previous start's last record for the startup log.
pub fn summary(rec: &BootRecord) -> String {
    let e = &rec.entry;
    let head = format!(
        "boot {} (uptime {} s, reset reason {}, same kernel boot {})",
        rec.boot,
        e["uptime_s"].as_f64().map(|v| format!("{v:.1}")).unwrap_or_else(|| "?".into()),
        e["reset_reason"].as_str().unwrap_or("n/a"),
        e["same_kernel_boot"]
    );
    match e.get("prev_last").filter(|v| v.is_object()) {
        None => format!("{head}; no black-box record from the previous start"),
        Some(p) => format!(
            "{head}; previous start {} ended at t={} ms: safety_state={} is_torqued={} lease_holder={} action={} motion_limit={} motor_errors={}",
            p["boot"], p["t_ms"], p["safety_state"], p["is_torqued"], p["lease_holder"], p["action"], p["motion_limit"], p["motor_errors"]
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sys(id: &str) -> SystemInfo {
        SystemInfo { uptime_s: Some(12.5), kernel_boot_id: Some(id.into()), reset_reason: None }
    }

    fn lines(path: &Path) -> Vec<Value> {
        std::fs::read_to_string(path).unwrap().lines().map(|l| serde_json::from_str(l).unwrap()).collect()
    }

    #[test]
    fn counts_starts_and_reads_the_previous_last_record() {
        let dir = tempfile::tempdir().unwrap();
        let first = record_boot(dir.path(), &sys("k1")).unwrap();
        assert_eq!(first.boot, 1);
        assert!(first.entry["prev_last"].is_null());
        assert!(summary(&first).contains("no black-box record"));

        // Previous start's black box: the last line was cut by a power loss.
        let bb = dir.path().join(BB_FILE);
        std::fs::write(&bb, "{\"boot\":1,\"t_ms\":100,\"is_torqued\":false}\n{\"boot\":1,\"t_ms\":5200,\"is_torqued\":true,\"safety_state\":\"Moving\"}\n{\"boot\":1,\"t_ms\":54").unwrap();
        let second = record_boot(dir.path(), &sys("k2")).unwrap();
        assert_eq!(second.boot, 2);
        assert_eq!(second.entry["prev_last"]["t_ms"], 5200);
        assert_eq!(second.entry["prev_last"]["is_torqued"], true);
        assert_eq!(second.entry["same_kernel_boot"], false, "kernel boot id changed: a reset");
        let s = summary(&second);
        assert!(s.contains("t=5200 ms") && s.contains("\"Moving\""), "{s}");

        let third = record_boot(dir.path(), &sys("k2")).unwrap();
        assert_eq!(third.entry["same_kernel_boot"], true, "armd restart without a reboot");
        let all = lines(&dir.path().join(BOOTS_FILE));
        assert_eq!(all.iter().map(|e| e["boot"].as_u64().unwrap()).collect::<Vec<_>>(), vec![1, 2, 3]);
        assert_eq!(all[0]["uptime_s"], 12.5);
    }

    #[test]
    fn previous_record_falls_back_to_the_rotated_file() {
        let dir = tempfile::tempdir().unwrap();
        let bb = dir.path().join(BB_FILE);
        std::fs::write(RotatingFile::numbered(&bb, 1), "{\"boot\":4,\"t_ms\":9}\n").unwrap();
        std::fs::write(&bb, "").unwrap();
        assert_eq!(last_blackbox_record(dir.path()).unwrap()["t_ms"], 9);
    }

    #[test]
    fn counter_continues_from_boots_log_when_the_counter_file_is_lost() {
        let dir = tempfile::tempdir().unwrap();
        record_boot(dir.path(), &sys("a")).unwrap();
        record_boot(dir.path(), &sys("a")).unwrap();
        std::fs::remove_file(dir.path().join(COUNTER_FILE)).unwrap();
        assert_eq!(record_boot(dir.path(), &sys("a")).unwrap().boot, 3);
    }
}
