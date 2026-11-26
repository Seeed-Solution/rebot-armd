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
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::http::HeaderMap;
use serde::Serialize;

use crate::persist;

pub const DEFAULT_TTL_S: u64 = 1800;
pub const DEFAULT_OPERATOR_PIN_FILE: &str = "/userdata/rebot-teleop/operator.pin";
pub const SESSION_COOKIE_NAME: &str = "rebot_op";

/// Maximum concurrent live operator sessions per [`OperatorAuth`] instance (fail-closed when full).
pub const MAX_OPERATOR_SESSIONS: usize = 64;

/// Process-wide pool of lock-free per-session revocation generation slots.
/// Each active operator session claims one slot with a monotonic non-zero generation so the RT
/// owner validates [`OperatorGrant`] with a single `Acquire` load and zero allocations/locks,
/// without cross-invalidating concurrent sessions on single-session logout.
const GLOBAL_SESSION_SLOTS: usize = 2048;

static SESSION_SLOT_GEN: [AtomicU64; GLOBAL_SESSION_SLOTS] =
    [const { AtomicU64::new(0) }; GLOBAL_SESSION_SLOTS];
static SESSION_SLOT_IN_USE: [AtomicBool; GLOBAL_SESSION_SLOTS] =
    [const { AtomicBool::new(false) }; GLOBAL_SESSION_SLOTS];
static NEXT_SESSION_SLOT_HINT: AtomicUsize = AtomicUsize::new(0);

fn claim_global_slot() -> Option<(u16, u64)> {
    let start = NEXT_SESSION_SLOT_HINT.fetch_add(1, Ordering::Relaxed) % GLOBAL_SESSION_SLOTS;
    for offset in 0..GLOBAL_SESSION_SLOTS {
        let idx = (start + offset) % GLOBAL_SESSION_SLOTS;
        if SESSION_SLOT_IN_USE[idx]
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok()
        {
            let mut gen = SESSION_SLOT_GEN[idx].fetch_add(1, Ordering::AcqRel).wrapping_add(1);
            if gen == 0 {
                gen = SESSION_SLOT_GEN[idx].fetch_add(1, Ordering::AcqRel).wrapping_add(1);
            }
            return Some((idx as u16, gen));
        }
    }
    None
}

fn release_global_slot(slot: u16) {
    let idx = usize::from(slot);
    if idx < GLOBAL_SESSION_SLOTS {
        // Advance generation with Release ordering BEFORE marking the slot free so any live
        // `OperatorGrant` referencing this session observes revocation immediately and no
        // subsequent occupant can match an older generation.
        let next = SESSION_SLOT_GEN[idx].fetch_add(1, Ordering::Release).wrapping_add(1);
        if next == 0 {
            SESSION_SLOT_GEN[idx].fetch_add(1, Ordering::Release);
        }
        SESSION_SLOT_IN_USE[idx].store(false, Ordering::Release);
    }
}

/// Lock-free, `Copy`, zero-allocation operator authorization token validated on the RT owner.
///
/// Issued atomically under [`OperatorAuth`]'s session mutex via [`OperatorAuth::operator_grant`]
/// (zero TOCTOU between session validation and generation capture) and re-verified at owner commit
/// via [`Self::is_valid_at`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperatorGrant {
    authorized: bool,
    unenforced: bool,
    expires_at: Option<Instant>,
    slot: u16,
    gen: u64,
}

impl OperatorGrant {
    /// Denied grant (always rejected by [`Self::is_valid_at`]).
    pub const DENIED: Self = Self {
        authorized: false,
        unenforced: false,
        expires_at: None,
        slot: 0,
        gen: 0,
    };

    /// Grant issued when [`OperatorAuth::is_enforced`] is `false`.
    /// Cross-checked against the owner's `operator_auth_enforced` flag fail-closed.
    pub const fn unenforced() -> Self {
        Self {
            authorized: true,
            unenforced: true,
            expires_at: None,
            slot: 0,
            gen: 0,
        }
    }

    fn session(slot: u16, gen: u64, expires_at: Instant) -> Self {
        Self {
            authorized: true,
            unenforced: false,
            expires_at: Some(expires_at),
            slot,
            gen,
        }
    }

    /// Lock-free, zero-allocation live validity check at `now` against `owner_auth_enforced`.
    #[inline]
    pub fn is_valid_at(self, now: Instant, owner_auth_enforced: bool) -> bool {
        if !self.authorized {
            return false;
        }
        if self.unenforced {
            return !owner_auth_enforced;
        }
        let Some(exp) = self.expires_at else {
            return false;
        };
        if now >= exp {
            return false;
        }
        let idx = usize::from(self.slot);
        idx < GLOBAL_SESSION_SLOTS
            && self.gen != 0
            && SESSION_SLOT_GEN[idx].load(Ordering::Acquire) == self.gen
    }
}

/// Explicit operator authorization requirement carried on prepared commands and sessions.
///
/// Protected operations specify [`Self::Required`]; unprivileged operations specify
/// [`Self::NotRequired`]. Checked lock-free and zero-alloc on the owner against `now` and
/// `owner_auth_enforced` via [`Self::is_satisfied_at`], reusing [`OperatorGrant::is_valid_at`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorRequirement {
    Required(OperatorGrant),
    NotRequired,
}

impl OperatorRequirement {
    #[inline]
    pub const fn required(grant: OperatorGrant) -> Self {
        Self::Required(grant)
    }

    #[inline]
    pub const fn when(required: bool, grant: OperatorGrant) -> Self {
        if required {
            Self::Required(grant)
        } else {
            Self::NotRequired
        }
    }

    #[inline]
    pub const fn is_required(self) -> bool {
        matches!(self, Self::Required(_))
    }

    /// Lock-free, zero-allocation live check against `now` and `owner_auth_enforced`.
    #[inline]
    pub fn is_satisfied_at(self, now: Instant, owner_auth_enforced: bool) -> bool {
        match self {
            Self::NotRequired => true,
            Self::Required(grant) => grant.is_valid_at(now, owner_auth_enforced),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SessionEntry {
    expires_at: Instant,
    slot: u16,
    gen: u64,
}

struct SessionStore {
    map: Mutex<HashMap<String, SessionEntry>>,
}

impl SessionStore {
    fn new() -> Self {
        Self {
            map: Mutex::new(HashMap::new()),
        }
    }

    fn prune_expired(sessions: &mut HashMap<String, SessionEntry>, now: Instant) {
        sessions.retain(|_, entry| {
            if entry.expires_at > now {
                true
            } else {
                release_global_slot(entry.slot);
                false
            }
        });
    }
}

impl Drop for SessionStore {
    fn drop(&mut self) {
        let mut sessions = self.map.lock().unwrap_or_else(|e| e.into_inner());
        for (_, entry) in sessions.drain() {
            release_global_slot(entry.slot);
        }
    }
}

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
    allowed_origins: Vec<OriginTuple>,
    trust_forwarded_proto: bool,
    sessions: Arc<SessionStore>,
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

fn parse_nonzero_port(port_str: &str) -> Option<u16> {
    if port_str.is_empty() || !port_str.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let p: u16 = port_str.parse().ok()?;
    (p != 0).then_some(p)
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
        let ip: std::net::Ipv6Addr = after_open[..close_idx].parse().ok()?;
        let rest = &after_open[close_idx + 1..];
        let port = if rest.is_empty() {
            default_p
        } else {
            parse_nonzero_port(rest.strip_prefix(':')?)?
        };
        return Some((ip.to_string(), port));
    }
    if raw.contains(['[', ']']) {
        return None;
    }
    if let Some((host_raw, port_str)) = raw.split_once(':') {
        // Reject unbracketed IPv6 or extra colons
        if host_raw.is_empty() || port_str.contains(':') {
            return None;
        }
        let port = parse_nonzero_port(port_str)?;
        Some((host_raw.to_ascii_lowercase(), port))
    } else {
        Some((raw.to_ascii_lowercase(), default_p))
    }
}

fn is_loopback_host(host: &str) -> bool {
    matches!(host, "localhost" | "127.0.0.1" | "::1")
}

fn resolve_public_origin(configured: Option<&str>) -> Option<String> {
    if let Some(v) = configured.map(str::trim).filter(|s| !s.is_empty()) {
        return Some(v.to_string());
    }
    std::env::var("REBOT_PUBLIC_ORIGIN")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
}

fn resolve_allowed_origins(configured: &[OriginTuple]) -> Vec<OriginTuple> {
    if !configured.is_empty() {
        return configured.to_vec();
    }
    std::env::var("REBOT_ALLOWED_ORIGINS")
        .ok()
        .map(|raw| {
            raw.split(',')
                .filter_map(|s| parse_origin(s.trim().trim_end_matches('/')))
                .collect()
        })
        .unwrap_or_default()
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

/// Validate a request's `Origin` header against `Host`, optional `REBOT_PUBLIC_ORIGIN` override,
/// and explicitly configured/env `allowed_origins`.
pub fn validate_origin_headers_with_allowed(
    headers: &HeaderMap,
    configured_public_origin: Option<&str>,
    configured_allowed_origins: &[OriginTuple],
) -> bool {
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
    let allowed_origins = resolve_allowed_origins(configured_allowed_origins);

    if let Some(pub_raw) = resolve_public_origin(configured_public_origin) {
        let Some(pub_origin) = parse_origin(pub_raw.trim_end_matches('/')) else {
            return false;
        };
        if origin == pub_origin {
            return true;
        }
        let shadows_public = origin.host == pub_origin.host
            && (origin.port == pub_origin.port
                || (origin.port == 80 && pub_origin.port == 443));
        if allowed_origins.contains(&origin)
            && !(shadows_public && origin.scheme != pub_origin.scheme)
        {
            return true;
        }
        return origin.scheme == "http"
            && is_loopback_host(&origin.host)
            && host_hdr
                .and_then(|h| parse_host_header(h, "http"))
                .is_some_and(|expected| expected == origin)
            && !shadows_public;
    }

    if allowed_origins.contains(&origin) {
        return true;
    }

    host_hdr
        .and_then(|h| parse_host_header(h, "http"))
        .is_some_and(|expected| expected == origin)
}

/// Validate a request's `Origin` header against `Host` and optional `REBOT_PUBLIC_ORIGIN` override.
pub fn validate_origin_headers(headers: &HeaderMap, configured_public_origin: Option<&str>) -> bool {
    validate_origin_headers_with_allowed(headers, configured_public_origin, &[])
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
    SessionsFull,
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
            allowed_origins: Vec::new(),
            trust_forwarded_proto: false,
            sessions: Arc::new(SessionStore::new()),
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
            allowed_origins: Vec::new(),
            trust_forwarded_proto: false,
            sessions: Arc::new(SessionStore::new()),
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
            allowed_origins: self.0.allowed_origins.clone(),
            trust_forwarded_proto: self.0.trust_forwarded_proto,
            sessions: Arc::clone(&self.0.sessions),
            rate_limit: Arc::clone(&self.0.rate_limit),
        }))
    }

    /// Explicitly approve additional origins (otherwise read from comma-separated `REBOT_ALLOWED_ORIGINS`).
    pub fn with_allowed_origins(self, origins: Vec<OriginTuple>) -> Self {
        Self(Arc::new(OperatorAuthInner {
            enforced: self.0.enforced,
            pin: self.0.pin.clone(),
            ttl: self.0.ttl,
            public_origin: self.0.public_origin.clone(),
            allowed_origins: origins,
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
            allowed_origins: self.0.allowed_origins.clone(),
            trust_forwarded_proto: trust,
            sessions: Arc::clone(&self.0.sessions),
            rate_limit: Arc::clone(&self.0.rate_limit),
        }))
    }

    pub fn validate_origin(&self, headers: &HeaderMap) -> bool {
        validate_origin_headers_with_allowed(
            headers,
            self.0.public_origin.as_deref(),
            &self.0.allowed_origins,
        )
    }

    /// Whether `Set-Cookie` should include `; Secure`.
    /// Only enabled when `public_origin` / `REBOT_PUBLIC_ORIGIN` is HTTPS (and request `Origin`, if present, is not plain `http`)
    /// OR when trusted-proxy forwarding (`with_trust_forwarded_proto(true)` / `REBOT_TRUST_FORWARDED_PROTO`) is enabled
    /// and the forwarded headers indicate `https`.
    pub fn should_set_secure_cookie(&self, headers: &HeaderMap) -> bool {
        if let Some(pub_raw) = resolve_public_origin(self.0.public_origin.as_deref()) {
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
        let mut sessions = self.0.sessions.map.lock().unwrap_or_else(|e| e.into_inner());
        SessionStore::prune_expired(&mut sessions, now);
        if let Some(tok) = token {
            if let Some(entry) = sessions.get(tok) {
                let rem_ms = entry.expires_at.saturating_duration_since(now).as_millis() as u64;
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

    /// Atomically validates `token` under the session mutex and captures a lock-free
    /// [`OperatorGrant`] bound to that exact session's revocation slot and expiry instant.
    pub fn operator_grant(&self, token: Option<&str>) -> OperatorGrant {
        if !self.0.enforced {
            return OperatorGrant::unenforced();
        }
        let now = Instant::now();
        let mut sessions = self.0.sessions.map.lock().unwrap_or_else(|e| e.into_inner());
        SessionStore::prune_expired(&mut sessions, now);
        let Some(tok) = token else {
            return OperatorGrant::DENIED;
        };
        match sessions.get(tok) {
            Some(entry) if entry.expires_at > now => {
                OperatorGrant::session(entry.slot, entry.gen, entry.expires_at)
            }
            _ => OperatorGrant::DENIED,
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
    /// Enforces exponential backoff after 5 consecutive failures (`1s` -> `2s` -> ... -> `30s` max)
    /// and a fail-closed cap of [`MAX_OPERATOR_SESSIONS`] live sessions per instance.
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
        let mut sessions = self.0.sessions.map.lock().unwrap_or_else(|e| e.into_inner());
        SessionStore::prune_expired(&mut sessions, now);
        if sessions.len() >= MAX_OPERATOR_SESSIONS {
            return Err(UnlockError::SessionsFull);
        }
        let Some((slot, gen)) = claim_global_slot() else {
            return Err(UnlockError::SessionsFull);
        };
        sessions.insert(
            token.clone(),
            SessionEntry {
                expires_at,
                slot,
                gen,
            },
        );
        Ok((token, ttl_s))
    }

    /// Immediately revoke `token` (if present) without invalidating other active sessions.
    pub fn lock(&self, token: Option<&str>) {
        if let Some(tok) = token {
            let mut sessions = self.0.sessions.map.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(entry) = sessions.remove(tok) {
                release_global_slot(entry.slot);
            }
        }
    }

    /// Extract credential preserving Bearer vs Cookie isolation:
    /// when any `Authorization` header is present, NEVER fall back to `Cookie`.
    pub fn extract_credential(headers: &HeaderMap) -> AuthCredential {
        if headers.contains_key(axum::http::header::AUTHORIZATION) {
            for value in headers.get_all(axum::http::header::AUTHORIZATION) {
                let Ok(s) = value.to_str() else { continue };
                if s.len() < 6 || !s[..6].eq_ignore_ascii_case("bearer") {
                    continue;
                }
                let rest = &s[6..];
                if rest.is_empty() || rest.starts_with(char::is_whitespace) {
                    let tok = rest.trim();
                    return AuthCredential::Bearer((!tok.is_empty()).then(|| tok.to_string()));
                }
            }
            return AuthCredential::Bearer(None);
        }
        let cookie_prefix = format!("{SESSION_COOKIE_NAME}=");
        for value in headers.get_all(axum::http::header::COOKIE) {
            let Ok(s) = value.to_str() else { continue };
            for part in s.split(';').map(str::trim) {
                if let Some(val) = part.strip_prefix(&cookie_prefix) {
                    let tok = val.trim();
                    if !tok.is_empty() {
                        return AuthCredential::Cookie(tok.to_string());
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
        let Ok(s) = val.to_str() else { continue };
        if s.split(',').next().is_some_and(|p| p.trim().eq_ignore_ascii_case("https")) {
            return true;
        }
    }
    for val in headers.get_all(axum::http::header::FORWARDED) {
        let Ok(s) = val.to_str() else { continue };
        for dir in s.split(',').flat_map(|elem| elem.split(';')) {
            let Some((k, v)) = dir.trim().split_once('=') else { continue };
            if k.trim().eq_ignore_ascii_case("proto")
                && v.trim().trim_matches('"').eq_ignore_ascii_case("https")
            {
                return true;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn operator_grant_per_session_revocation_and_expiry() {
        let auth = OperatorAuth::enforced("123456", 10);
        let (tok_a, _) = auth.unlock("123456").unwrap();
        let (tok_b, _) = auth.unlock("123456").unwrap();

        let grant_a = auth.operator_grant(Some(&tok_a));
        let grant_b = auth.operator_grant(Some(&tok_b));
        let now = Instant::now();
        assert!(grant_a.is_valid_at(now, true));
        assert!(grant_b.is_valid_at(now, true));
        assert!(!OperatorGrant::DENIED.is_valid_at(now, false));
        assert!(!OperatorGrant::unenforced().is_valid_at(now, true));
        assert!(OperatorGrant::unenforced().is_valid_at(now, false));

        // Locking session A revokes only A's queued grant; session B stays valid.
        auth.lock(Some(&tok_a));
        assert!(!grant_a.is_valid_at(now, true));
        assert!(grant_b.is_valid_at(now, true));

        // Expiry while queued invalidates B without acquiring any lock.
        let after_ttl = now + Duration::from_secs(11);
        assert!(!grant_b.is_valid_at(after_ttl, true));
    }

    #[test]
    fn operator_sessions_cap_fails_closed_and_reclaims_on_lock() {
        let auth = OperatorAuth::enforced("654321", 60);
        let mut tokens = Vec::with_capacity(MAX_OPERATOR_SESSIONS);
        for _ in 0..MAX_OPERATOR_SESSIONS {
            let (tok, _) = auth.unlock("654321").expect("slot within cap");
            tokens.push(tok);
        }
        assert_eq!(auth.unlock("654321"), Err(UnlockError::SessionsFull));

        // Revoking one session frees a slot; the revoked grant stays permanently invalid
        // even after the slot is reused by a new session.
        let evicted_tok = tokens.pop().unwrap();
        let old_grant = auth.operator_grant(Some(&evicted_tok));
        auth.lock(Some(&evicted_tok));
        let (replacement_tok, _) = auth.unlock("654321").expect("slot reclaimed after lock");
        let new_grant = auth.operator_grant(Some(&replacement_tok));
        let now = Instant::now();
        assert!(!old_grant.is_valid_at(now, true));
        assert!(new_grant.is_valid_at(now, true));
    }

    #[test]
    fn dropping_auth_revokes_outstanding_grants_and_preserves_clones() {
        let auth = OperatorAuth::enforced("999999", 60);
        let auth2 = auth.clone().with_public_origin(Some("https://example.test".into()));
        let (tok, _) = auth.unlock("999999").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        drop(auth);
        // `auth2` still shares the same `Arc<SessionStore>`.
        assert!(grant.is_valid_at(Instant::now(), true));
        drop(auth2);
        // Last owner dropped -> slot released and generation bumped.
        assert!(!grant.is_valid_at(Instant::now(), true));
    }
}
