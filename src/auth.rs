//! Operator authentication and session management (`OperatorAuth`).
//!
//! When enforced, privileged operations (force lease preemption, non-recoverable
//! fault/E-stop clear, workspace keepout, collision guard, gravity comp, calibration,
//! grasp setup) require an active operator session unlocked via `POST /api/v1/auth/unlock`
//! with the configured or generated PIN.

use std::collections::HashMap;
use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::http::HeaderMap;
use serde::Serialize;

use crate::persist;

pub const DEFAULT_TTL_S: u64 = 1800;
pub const DEFAULT_OPERATOR_PIN_FILE: &str = "/userdata/rebot-teleop/operator.pin";
pub const SESSION_COOKIE_NAME: &str = "rebot_op";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Role {
    Program,
    Operator,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AuthStatus {
    pub auth_enforced: bool,
    pub role: Role,
    pub expires_in_s: Option<u64>,
}

struct UnlockRateLimit {
    consecutive_failures: u32,
    locked_until: Option<Instant>,
}

struct OperatorAuthInner {
    enforced: bool,
    pin: String,
    ttl: Duration,
    sessions: Mutex<HashMap<String, Instant>>,
    rate_limit: Mutex<UnlockRateLimit>,
}

/// Shared operator authentication handle used by HTTP routes, WS connections, and `safety/describe`.
#[derive(Clone)]
pub struct OperatorAuth(Arc<OperatorAuthInner>);

impl std::fmt::Debug for OperatorAuth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OperatorAuth")
            .field("enforced", &self.0.enforced)
            .field("ttl_s", &self.0.ttl.as_secs())
            .finish()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnlockError {
    InvalidPin,
    RateLimited { retry_after_s: u64 },
    RngFailed,
}

impl OperatorAuth {
    /// Disabled mode: all operations are allowed without unlocking.
    pub fn disabled() -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: false,
            pin: String::new(),
            ttl: Duration::from_secs(DEFAULT_TTL_S),
            sessions: Mutex::new(HashMap::new()),
            rate_limit: Mutex::new(UnlockRateLimit {
                consecutive_failures: 0,
                locked_until: None,
            }),
        }))
    }

    /// Enforced mode: privileged operations require a valid session token unlocked with `pin`.
    pub fn enforced(pin: impl Into<String>, ttl_s: u64) -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: true,
            pin: pin.into(),
            ttl: Duration::from_secs(ttl_s),
            sessions: Mutex::new(HashMap::new()),
            rate_limit: Mutex::new(UnlockRateLimit {
                consecutive_failures: 0,
                locked_until: None,
            }),
        }))
    }

    pub fn is_enforced(&self) -> bool {
        self.0.enforced
    }

    pub fn ttl_s(&self) -> u64 {
        self.0.ttl.as_secs()
    }

    /// Live status check for `token` (prunes expired sessions).
    pub fn status(&self, token: Option<&str>) -> AuthStatus {
        if !self.0.enforced {
            return AuthStatus {
                auth_enforced: false,
                role: Role::Operator,
                expires_in_s: None,
            };
        }
        let now = Instant::now();
        let mut sessions = self.0.sessions.lock().unwrap_or_else(|e| e.into_inner());
        sessions.retain(|_, exp| *exp > now);
        if let Some(tok) = token {
            if let Some(exp) = sessions.get(tok) {
                let rem_ms = exp.saturating_duration_since(now).as_millis() as u64;
                let expires_in_s = rem_ms.div_ceil(1000).max(1);
                return AuthStatus {
                    auth_enforced: true,
                    role: Role::Operator,
                    expires_in_s: Some(expires_in_s),
                };
            }
        }
        AuthStatus {
            auth_enforced: true,
            role: Role::Program,
            expires_in_s: None,
        }
    }

    /// Returns true if auth is disabled OR `token` currently holds an unexpired operator session.
    pub fn is_operator(&self, token: Option<&str>) -> bool {
        if !self.0.enforced {
            return true;
        }
        self.status(token).role == Role::Operator
    }

    /// Verify `pin` and, on success, create a random 32-byte hex session token.
    /// Enforces exponential backoff after 5 consecutive failures (`1s` -> `2s` -> ... -> `30s` max).
    pub fn unlock(&self, pin: &str) -> Result<(String, u64), UnlockError> {
        if !self.0.enforced {
            return Ok((String::new(), self.ttl_s()));
        }
        let now = Instant::now();
        let mut rl = self.0.rate_limit.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(until) = rl.locked_until {
            if now < until {
                let rem_ms = until.saturating_duration_since(now).as_millis() as u64;
                let retry_after_s = rem_ms.div_ceil(1000).max(1);
                return Err(UnlockError::RateLimited { retry_after_s });
            }
        }
        if !constant_time_eq(pin.as_bytes(), self.0.pin.as_bytes()) {
            rl.consecutive_failures = rl.consecutive_failures.saturating_add(1);
            if rl.consecutive_failures >= 5 {
                let shift = (rl.consecutive_failures - 5).min(5);
                let secs = (1u64 << shift).min(30);
                rl.locked_until = Some(Instant::now() + Duration::from_secs(secs));
            }
            return Err(UnlockError::InvalidPin);
        }
        let bytes = random_bytes::<32>().map_err(|_| UnlockError::RngFailed)?;
        let token = hex_encode(&bytes);
        let ttl_s = self.0.ttl.as_secs();
        let now = Instant::now();
        let expires_at = now + self.0.ttl;
        rl.consecutive_failures = 0;
        rl.locked_until = None;
        drop(rl);
        let mut sessions = self.0.sessions.lock().unwrap_or_else(|e| e.into_inner());
        sessions.retain(|_, exp| *exp > now);
        sessions.insert(token.clone(), expires_at);
        Ok((token, ttl_s))
    }

    /// Immediately revoke `token` (if present).
    pub fn lock(&self, token: Option<&str>) {
        if let Some(tok) = token {
            let mut sessions = self.0.sessions.lock().unwrap_or_else(|e| e.into_inner());
            sessions.remove(tok);
        }
    }

    /// Extract token from `Authorization: Bearer <token>` first, then `Cookie: ...; rebot_op=<token>`.
    pub fn extract_token(headers: &HeaderMap) -> Option<String> {
        for value in headers.get_all(axum::http::header::AUTHORIZATION) {
            if let Ok(s) = value.to_str() {
                if s.len() > 7 && s[..7].eq_ignore_ascii_case("bearer ") {
                    let tok = s[7..].trim();
                    if !tok.is_empty() {
                        return Some(tok.to_string());
                    }
                }
            }
        }
        for value in headers.get_all(axum::http::header::COOKIE) {
            if let Ok(s) = value.to_str() {
                for part in s.split(';') {
                    let part = part.trim();
                    if let Some(val) = part.strip_prefix(&format!("{SESSION_COOKIE_NAME}=")) {
                        let tok = val.trim();
                        if !tok.is_empty() {
                            return Some(tok.to_string());
                        }
                    }
                }
            }
        }
        None
    }
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    let max_len = a.len().max(b.len());
    let mut diff: u8 = if a.len() == b.len() { 0 } else { 1 };
    for i in 0..max_len {
        let ba = a.get(i).copied().unwrap_or(0);
        let bb = b.get(i).copied().unwrap_or(0);
        diff |= ba ^ bb;
    }
    diff == 0
}

fn random_bytes<const N: usize>() -> std::io::Result<[u8; N]> {
    let mut buf = [0u8; N];
    let mut file = std::fs::File::open("/dev/urandom")?;
    file.read_exact(&mut buf)?;
    Ok(buf)
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        out.push(HEX[(b >> 4) as usize] as char);
        out.push(HEX[(b & 0x0f) as usize] as char);
    }
    out
}

/// Generate a cryptographically random 6-digit decimal PIN (`000000..=999999`).
pub fn generate_pin_6_digits() -> std::io::Result<String> {
    let limit = u32::MAX - (u32::MAX % 1_000_000);
    loop {
        let b = random_bytes::<4>()?;
        let val = u32::from_le_bytes(b);
        if val < limit {
            return Ok(format!("{:06}", val % 1_000_000));
        }
    }
}

/// Resolve the operator PIN file path from CLI flag, `ARMD_OPERATOR_PIN_FILE`, or default.
pub fn resolve_pin_file_path(cli: Option<&str>) -> PathBuf {
    if let Some(p) = cli {
        return PathBuf::from(p);
    }
    persist::path_from_env("ARMD_OPERATOR_PIN_FILE", DEFAULT_OPERATOR_PIN_FILE)
}

/// Write the generated PIN atomically with `0600` permissions (never logs the plaintext PIN).
pub fn write_pin_file_atomic(path: &Path, pin: &str) -> std::io::Result<()> {
    let dir = path
        .parent()
        .filter(|d| !d.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "operator.pin".into());
    let tmp = dir.join(format!(
        ".{name}.tmp-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos())
    ));
    let result = (|| {
        let mut opts = OpenOptions::new();
        opts.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            opts.mode(0o600);
        }
        let mut file = opts.open(&tmp)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            file.set_permissions(std::fs::Permissions::from_mode(0o600))?;
        }
        writeln!(file, "{pin}")?;
        file.sync_all()?;
        drop(file);
        std::fs::rename(&tmp, path)?;
        std::fs::File::open(dir)?.sync_all()
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    result
}
