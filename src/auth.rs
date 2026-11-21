//! Operator authentication and session management (`OperatorAuth`).
//!
//! When enforced, privileged operations (force lease preemption, non-recoverable
//! fault/E-stop clear, workspace keepout, collision guard, gravity comp, calibration,
//! grasp setup, sandbox uploads / activation, `/api/v1/action/policy/start` on a real or
//! unknown driver in any mode) require an active operator session unlocked via `POST /api/v1/auth/unlock`
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
    public_origin: Option<String>,
    trust_forwarded_proto: bool,
    sessions: Arc<Mutex<HashMap<String, Instant>>>,
    rate_limit: Arc<Mutex<UnlockRateLimit>>,
}

/// Extracted authentication credential preserving whether `Authorization: Bearer` was present.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthCredential {
    /// Request carried an `Authorization` header (e.g. `Bearer <token>`). Never falls back to Cookie.
    Bearer(Option<String>),
    /// No `Authorization` header; request carried a `rebot_op` cookie.
    Cookie(String),
    /// Neither `Authorization` nor `rebot_op` cookie was present.
    None,
}

/// Normalized `(scheme, host, effective_port)` origin tuple.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OriginTuple {
    pub scheme: String,
    pub host: String,
    pub port: u16,
}

fn default_port(scheme: &str) -> Option<u16> {
    match scheme {
        "http" => Some(80),
        "https" => Some(443),
        _ => None,
    }
}

fn parse_authority(raw: &str, default_scheme: &str) -> Option<(String, u16)> {
    let default_p = default_port(default_scheme)?;
    if raw.is_empty()
        || raw.bytes().any(|b| b.is_ascii_whitespace() || b.is_ascii_control())
        || raw.contains(['/', '?', '#', '@', '\\'])
    {
        return None;
    }
    if let Some(after_open) = raw.strip_prefix('[') {
        let close_idx = after_open.find(']')?;
        let host_raw = &after_open[..close_idx];
        let ip: std::net::Ipv6Addr = host_raw.parse().ok()?;
        let rest = &after_open[close_idx + 1..];
        let port = if rest.is_empty() {
            default_p
        } else {
            let port_str = rest.strip_prefix(':')?;
            if port_str.is_empty() || !port_str.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            let p: u16 = port_str.parse().ok()?;
            if p == 0 {
                return None;
            }
            p
        };
        return Some((ip.to_string(), port));
    }
    if raw.contains(['[', ']']) {
        return None;
    }
    if let Some((host_raw, port_str)) = raw.split_once(':') {
        // Reject unbracketed IPv6 or extra colons
        if host_raw.is_empty()
            || port_str.is_empty()
            || port_str.contains(':')
            || !port_str.bytes().all(|b| b.is_ascii_digit())
        {
            return None;
        }
        let p: u16 = port_str.parse().ok()?;
        if p == 0 {
            return None;
        }
        Some((host_raw.to_ascii_lowercase(), p))
    } else {
        Some((raw.to_ascii_lowercase(), default_p))
    }
}

/// Parse an HTTP `Origin` header (`scheme://host[:port]`) into a normalized `(scheme, host, effective_port)`.
pub fn parse_origin(raw: &str) -> Option<OriginTuple> {
    let s = raw.trim();
    if s.is_empty() || s.eq_ignore_ascii_case("null") {
        return None;
    }
    let (scheme_raw, rest) = s.split_once("://")?;
    let scheme = scheme_raw.to_ascii_lowercase();
    if !matches!(scheme.as_str(), "http" | "https") {
        return None;
    }
    let (host, port) = parse_authority(rest, &scheme)?;
    Some(OriginTuple { scheme, host, port })
}

/// Parse an HTTP `Host` header into `(scheme, host, effective_port)` using `scheme`.
pub fn parse_host_header(raw: &str, scheme: &str) -> Option<OriginTuple> {
    let scheme = scheme.to_ascii_lowercase();
    if !matches!(scheme.as_str(), "http" | "https") {
        return None;
    }
    let (host, port) = parse_authority(raw.trim(), &scheme)?;
    Some(OriginTuple { scheme, host, port })
}

/// Validate a request's `Origin` header against `Host` and optional `REBOT_PUBLIC_ORIGIN` override.
pub fn validate_origin_headers(headers: &HeaderMap, configured_public_origin: Option<&str>) -> bool {
    let mut origins = headers.get_all(axum::http::header::ORIGIN).iter();
    let Some(origin_hdr) = origins.next() else {
        return true;
    };
    if origins.next().is_some() {
        return false;
    }
    let Some(origin) = origin_hdr.to_str().ok().and_then(parse_origin) else {
        return false;
    };

    let host_hdr = headers
        .get(axum::http::header::HOST)
        .and_then(|h| h.to_str().ok());

    let env_origin;
    let public_raw = match configured_public_origin.map(str::trim).filter(|s| !s.is_empty()) {
        Some(v) => Some(v),
        None => {
            env_origin = std::env::var("REBOT_PUBLIC_ORIGIN").ok();
            env_origin.as_deref().map(str::trim).filter(|s| !s.is_empty())
        }
    };

    if let Some(pub_raw) = public_raw {
        if pub_raw.eq_ignore_ascii_case("http") || pub_raw.eq_ignore_ascii_case("https") {
            let expected_scheme = pub_raw.to_ascii_lowercase();
            return host_hdr
                .and_then(|h| parse_host_header(h, &expected_scheme))
                .is_some_and(|expected| expected == origin);
        }
        let Some(pub_origin) = parse_origin(pub_raw.trim_end_matches('/')) else {
            return false;
        };
        if origin == pub_origin {
            return true;
        }
        // Allow direct local HTTP Cockpit requests only when Host does not target the configured public origin.
        return host_hdr
            .and_then(|h| parse_host_header(h, "http"))
            .is_some_and(|expected| {
                let shadows_public = expected.host == pub_origin.host
                    && (expected.port == pub_origin.port
                        || (expected.port == 80 && pub_origin.port == 443));
                expected == origin && !shadows_public
            });
    }

    host_hdr
        .and_then(|h| parse_host_header(h, "http"))
        .is_some_and(|expected| expected == origin)
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
            public_origin: None,
            trust_forwarded_proto: false,
            sessions: Arc::new(Mutex::new(HashMap::new())),
            rate_limit: Arc::new(Mutex::new(UnlockRateLimit {
                consecutive_failures: 0,
                locked_until: None,
            })),
        }))
    }

    /// Enforced mode: privileged operations require a valid session token unlocked with `pin`.
    pub fn enforced(pin: impl Into<String>, ttl_s: u64) -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: true,
            pin: pin.into(),
            ttl: Duration::from_secs(ttl_s),
            public_origin: None,
            trust_forwarded_proto: false,
            sessions: Arc::new(Mutex::new(HashMap::new())),
            rate_limit: Arc::new(Mutex::new(UnlockRateLimit {
                consecutive_failures: 0,
                locked_until: None,
            })),
        }))
    }

    /// Override the trusted public origin (otherwise read from `REBOT_PUBLIC_ORIGIN`, defaulting to `http` + `Host`).
    pub fn with_public_origin(self, origin: Option<String>) -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: self.0.enforced,
            pin: self.0.pin.clone(),
            ttl: self.0.ttl,
            public_origin: origin,
            trust_forwarded_proto: self.0.trust_forwarded_proto,
            sessions: Arc::clone(&self.0.sessions),
            rate_limit: Arc::clone(&self.0.rate_limit),
        }))
    }

    /// Trust `X-Forwarded-Proto` / `Forwarded` headers from a reverse proxy when deriving `; Secure` cookies.
    pub fn with_trust_forwarded_proto(self, trust: bool) -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: self.0.enforced,
            pin: self.0.pin.clone(),
            ttl: self.0.ttl,
            public_origin: self.0.public_origin.clone(),
            trust_forwarded_proto: trust,
            sessions: Arc::clone(&self.0.sessions),
            rate_limit: Arc::clone(&self.0.rate_limit),
        }))
    }

    pub fn validate_origin(&self, headers: &HeaderMap) -> bool {
        validate_origin_headers(headers, self.0.public_origin.as_deref())
    }

    /// Whether `Set-Cookie` should include `; Secure`.
    /// Only enabled when `public_origin` / `REBOT_PUBLIC_ORIGIN` is HTTPS (and request `Origin`, if present, is not plain `http`)
    /// OR when trusted-proxy forwarding (`with_trust_forwarded_proto(true)` / `REBOT_TRUST_FORWARDED_PROTO`) is enabled
    /// and the forwarded headers indicate `https`.
    pub fn should_set_secure_cookie(&self, headers: &HeaderMap) -> bool {
        let env_origin;
        let public_raw = match self.0.public_origin.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
            Some(v) => Some(v),
            None => {
                env_origin = std::env::var("REBOT_PUBLIC_ORIGIN").ok();
                env_origin.as_deref().map(str::trim).filter(|s| !s.is_empty())
            }
        };
        if let Some(pub_raw) = public_raw {
            let pub_is_https = pub_raw.eq_ignore_ascii_case("https")
                || parse_origin(pub_raw.trim_end_matches('/')).is_some_and(|o| o.scheme == "https");
            if pub_is_https {
                let plain_http_origin = headers
                    .get(axum::http::header::ORIGIN)
                    .and_then(|h| h.to_str().ok())
                    .and_then(parse_origin)
                    .is_some_and(|o| o.scheme == "http");
                if !plain_http_origin {
                    return true;
                }
            }
        }
        let trust_forwarded = self.0.trust_forwarded_proto
            || std::env::var("REBOT_TRUST_FORWARDED_PROTO")
                .ok()
                .is_some_and(|v| matches!(v.trim().to_ascii_lowercase().as_str(), "1" | "true" | "yes"));
        trust_forwarded && is_https_forwarded(headers)
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

    /// Extract credential preserving Bearer vs Cookie isolation:
    /// when any `Authorization` header is present, NEVER fall back to `Cookie`.
    pub fn extract_credential(headers: &HeaderMap) -> AuthCredential {
        if headers.contains_key(axum::http::header::AUTHORIZATION) {
            for value in headers.get_all(axum::http::header::AUTHORIZATION) {
                if let Ok(s) = value.to_str() {
                    if s.len() >= 6 && s[..6].eq_ignore_ascii_case("bearer") {
                        let rest = &s[6..];
                        if rest.is_empty() || rest.starts_with(char::is_whitespace) {
                            let tok = rest.trim();
                            if !tok.is_empty() {
                                return AuthCredential::Bearer(Some(tok.to_string()));
                            }
                            return AuthCredential::Bearer(None);
                        }
                    }
                }
            }
            return AuthCredential::Bearer(None);
        }
        for value in headers.get_all(axum::http::header::COOKIE) {
            if let Ok(s) = value.to_str() {
                for part in s.split(';') {
                    let part = part.trim();
                    if let Some(val) = part.strip_prefix(&format!("{SESSION_COOKIE_NAME}=")) {
                        let tok = val.trim();
                        if !tok.is_empty() {
                            return AuthCredential::Cookie(tok.to_string());
                        }
                    }
                }
            }
        }
        AuthCredential::None
    }

    /// Extract token from `Authorization: Bearer <token>` if an `Authorization` header is present
    /// (without falling back to `Cookie`), otherwise from `Cookie: ...; rebot_op=<token>`.
    pub fn extract_token(headers: &HeaderMap) -> Option<String> {
        match Self::extract_credential(headers) {
            AuthCredential::Bearer(tok) => tok,
            AuthCredential::Cookie(tok) => Some(tok),
            AuthCredential::None => None,
        }
    }
}

fn is_https_forwarded(headers: &HeaderMap) -> bool {
    for val in headers.get_all("x-forwarded-proto") {
        if let Ok(s) = val.to_str() {
            if s.split(',').next().is_some_and(|p| p.trim().eq_ignore_ascii_case("https")) {
                return true;
            }
        }
    }
    for val in headers.get_all(axum::http::header::FORWARDED) {
        if let Ok(s) = val.to_str() {
            for elem in s.split(',') {
                for dir in elem.split(';') {
                    let dir = dir.trim();
                    if let Some((k, v)) = dir.split_once('=') {
                        if k.trim().eq_ignore_ascii_case("proto") {
                            let v = v.trim().trim_matches('"');
                            if v.eq_ignore_ascii_case("https") {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    false
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
