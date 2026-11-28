//! WebSocket connection slot pool, disconnect fencing, and WS lease RPC handlers.

#[allow(unused_imports)]
use super::*;

impl WsConnectionSlotPool {
    fn new() -> Self {
        Self {
            entries: std::sync::Mutex::new(std::collections::HashMap::with_capacity(
                MAX_LIVE_CONNECTIONS,
            )),
            notify: tokio::sync::Notify::new(),
        }
    }

    fn try_claim(&self) -> Result<crate::rt_core::requests::lease::ConnectionHandle, ArmError> {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        if g.len() >= MAX_LIVE_CONNECTIONS {
            return Err(ArmError::Rejected {
                status: 503,
                code: "TOO_MANY_CONNECTIONS".into(),
                message: "Maximum live WebSocket connections reached".into(),
            });
        }
        let id = NEXT_WS_CONN_ID.fetch_add(1, Ordering::Relaxed);
        let handle = crate::rt_core::requests::lease::ConnectionHandle::from_ws_id(id)
            .expect("monotonic WS conn_id >= 1");
        g.insert(
            id,
            WsSlotEntry {
                phase: WsSlotPhase::Admitted,
                opening: false,
                #[cfg(not(target_arch = "wasm32"))]
                reply_sink: None,
            },
        );
        Ok(handle)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn attach_reply_sink(
        &self,
        conn_id: u64,
        sink: Arc<crate::rt_core::reply_router::ConnectionReplySink>,
    ) -> bool {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let Some(entry) = g.get_mut(&conn_id) else {
            return false;
        };
        if entry.phase == WsSlotPhase::Closing {
            return false;
        }
        entry.reply_sink = Some(sink);
        true
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn reply_sink(
        &self,
        conn_id: u64,
    ) -> Option<Arc<crate::rt_core::reply_router::ConnectionReplySink>> {
        let g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let entry = g.get(&conn_id)?;
        if entry.phase == WsSlotPhase::Closing {
            return None;
        }
        entry.reply_sink.clone()
    }

    /// Joins transferred `live_ids` against non-closing `WsSlotEntry` reply sinks for atomic
    /// executor handover seeding (omits unregistered claimed slots and already-closing slots).
    #[cfg(not(target_arch = "wasm32"))]
    fn sinks_for_live_ids(
        &self,
        live_ids: &[u64],
    ) -> Vec<(u64, Arc<crate::rt_core::reply_router::ConnectionReplySink>)> {
        let g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = Vec::with_capacity(live_ids.len());
        for &id in live_ids {
            if let Some(entry) = g.get(&id) {
                if entry.phase != WsSlotPhase::Closing {
                    if let Some(sink) = entry.reply_sink.as_ref() {
                        out.push((id, Arc::clone(sink)));
                    }
                }
            }
        }
        out
    }

    fn begin_open(&self, conn_id: u64) -> bool {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let Some(entry) = g.get_mut(&conn_id) else {
            return false;
        };
        if entry.phase != WsSlotPhase::Admitted || entry.opening {
            return false;
        }
        entry.opening = true;
        true
    }

    fn finish_open(&self, conn_id: u64, succeeded: bool) {
        {
            let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(entry) = g.get_mut(&conn_id) {
                entry.opening = false;
                if succeeded && entry.phase == WsSlotPhase::Admitted {
                    entry.phase = WsSlotPhase::Open;
                }
            }
        }
        self.notify.notify_waiters();
    }

    fn mark_closing(&self, conn_id: u64) -> Option<bool> {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let entry = g.get_mut(&conn_id)?;
        entry.phase = WsSlotPhase::Closing;
        Some(entry.opening)
    }

    fn is_opening(&self, conn_id: u64) -> bool {
        self.entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&conn_id)
            .is_some_and(|e| e.opening)
    }

    fn remove(&self, conn_id: u64) {
        {
            let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
            g.remove(&conn_id);
        }
        self.notify.notify_waiters();
    }

    fn len(&self) -> usize {
        self.entries.lock().unwrap_or_else(|e| e.into_inner()).len()
    }
}

impl ArmSupervisor {
    /// Claims one off-thread WS connection slot (`<= MAX_LIVE_CONNECTIONS`) and allocates a
    /// strictly monotonic connection ID without touching the RT executor. Rejected upgrades
    /// never increment the connection ID counter (preventing permanent holes in `ClosedConnections`)
    /// and the budget applies uniformly to every retained socket, including degraded E-stop sockets.
    pub fn try_claim_ws_slot(
        &self,
    ) -> Result<crate::rt_core::requests::lease::ConnectionHandle, ArmError> {
        self.ws_slots.try_claim()
    }

    /// Attaches a per-connection reply sink to an already-claimed WS slot BEFORE owner
    /// registration begins, so both live executor registration and Legacy->Executor handover
    /// seed the exact same sink boundary.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn attach_ws_reply_sink(
        &self,
        conn_id: u64,
        sink: Arc<crate::rt_core::reply_router::ConnectionReplySink>,
    ) -> bool {
        self.ws_slots.attach_reply_sink(conn_id, sink)
    }

    /// Number of currently admitted WS connection slots (including closing slots awaiting owner
    /// disconnect confirmation).
    pub fn live_ws_slot_count(&self) -> usize {
        self.ws_slots.len()
    }

    /// Completes owner-side disconnect teardown for an admitted WS connection and releases its
    /// off-thread slot ONLY AFTER owner cleanup finishes (or the executor has permanently stopped).
    pub async fn finish_ws_connection(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        let Some(was_opening) = self.ws_slots.mark_closing(conn.id()) else {
            // Slot already removed (idempotent).
            return;
        };
        if was_opening {
            // Wait briefly for an in-flight `register_ws_connection_for_slot` to drop/finish so
            // `Open` and `Disconnect` stay linearized without hanging if the executor is stalled.
            let deadline = tokio::time::Instant::now() + Duration::from_millis(250);
            while self.ws_slots.is_opening(conn.id()) {
                let notified = self.ws_slots.notify.notified();
                if !self.ws_slots.is_opening(conn.id()) {
                    break;
                }
                if tokio::time::timeout_at(deadline, notified).await.is_err() {
                    break;
                }
            }
        }
        let _ = self.disconnect_ws_connection(conn).await;
        self.ws_slots.remove(conn.id());
    }

    /// Durable cancellation-safe Drop fallback for externally aborted WS socket tasks:
    /// spawns async `finish_ws_connection` on the Tokio runtime when available so contended
    /// legacy write locks, in-flight `OWNER_TRANSFERRING`, or pending `Open`s never strand a slot.
    pub fn finish_ws_connection_detached(
        self: &Arc<Self>,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        if let Ok(rt) = tokio::runtime::Handle::try_current() {
            let sup = Arc::clone(self);
            rt.spawn(async move {
                sup.finish_ws_connection(conn).await;
            });
            return;
        }
        self.disconnect_ws_connection_detached(conn);
        self.ws_slots.remove(conn.id());
    }

    /// Registers a live WS connection ID (`>= 1`) through the active owner (fail-closed at
    /// [`MAX_LIVE_CONNECTIONS`]). If ownership transfer (`OWNER_TRANSFERRING`) is momentarily
    /// in flight, waits up to a bounded 200 ms window for handoff to settle before registering.
    pub async fn register_ws_connection(&self, conn_id: u64) -> Result<(), ArmError> {
        use crate::rt_core::requests::lease::ConnLifecycleError;
        let deadline = Instant::now() + Duration::from_millis(200);
        let mode = loop {
            let m = self.owner_mode();
            if m != OWNER_TRANSFERRING || Instant::now() >= deadline {
                break m;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        };
        match mode {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                if self.check_legacy_under_guard(&inner).is_err() {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        if let Some(handle) = self.executor_handle() {
                            let open_res = if let Some(sink) = self.ws_slots.reply_sink(conn_id) {
                                handle.open_connection_with_reply_sink_async(conn_id, sink).await
                            } else {
                                handle.open_connection_async(conn_id).await
                            };
                            return match open_res {
                                Ok(Ok(())) => Ok(()),
                                Ok(Err(code)) => {
                                    let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                                    Err(ArmError::Rejected {
                                        status,
                                        code: code.into(),
                                        message: format!("Connection registration rejected: {code}"),
                                    })
                                }
                                Err(ConnLifecycleError::TooManyConnections) => Err(ArmError::Rejected {
                                    status: 503,
                                    code: "TOO_MANY_CONNECTIONS".into(),
                                    message: "Maximum live WebSocket connections reached".into(),
                                }),
                                Err(ConnLifecycleError::AlreadyClosed | ConnLifecycleError::InvalidConnection) => {
                                    Err(ArmError::Rejected {
                                        status: 409,
                                        code: "DISCONNECTED".into(),
                                        message: "Connection identity is closed or invalid".into(),
                                    })
                                }
                                Err(ConnLifecycleError::Stopped | ConnLifecycleError::ReentrantCallback) => {
                                    Err(executor_unavailable())
                                }
                            };
                        }
                    }
                    return Err(executor_unavailable());
                }
                inner.register_live_connection(conn_id).map_err(|code| {
                    let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                    ArmError::Rejected {
                        status,
                        code: code.into(),
                        message: format!("Connection registration rejected: {code}"),
                    }
                })
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                let open_res = if let Some(sink) = self.ws_slots.reply_sink(conn_id) {
                    handle.open_connection_with_reply_sink_async(conn_id, sink).await
                } else {
                    handle.open_connection_async(conn_id).await
                };
                match open_res {
                    Ok(Ok(())) => Ok(()),
                    Ok(Err(code)) => {
                        let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                        Err(ArmError::Rejected {
                            status,
                            code: code.into(),
                            message: format!("Connection registration rejected: {code}"),
                        })
                    }
                    Err(ConnLifecycleError::TooManyConnections) => Err(ArmError::Rejected {
                        status: 503,
                        code: "TOO_MANY_CONNECTIONS".into(),
                        message: "Maximum live WebSocket connections reached".into(),
                    }),
                    Err(ConnLifecycleError::AlreadyClosed | ConnLifecycleError::InvalidConnection) => {
                        Err(ArmError::Rejected {
                            status: 409,
                            code: "DISCONNECTED".into(),
                            message: "Connection identity is closed or invalid".into(),
                        })
                    }
                    Err(ConnLifecycleError::Stopped | ConnLifecycleError::ReentrantCallback) => {
                        Err(executor_unavailable())
                    }
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// True when one or more socket-close disconnect fences are queued awaiting owner drain
    /// (fail-closed output hold active).
    pub fn has_pending_disconnect_hold(&self) -> bool {
        self.pending_disconnect_hold.load(Ordering::SeqCst) > 0
    }

    /// True if `conn_id` (`>= 1`) has been marked closed in the lock-free immediate disconnect fence.
    pub fn is_connection_fenced_closed(&self, conn_id: u64) -> bool {
        self.closed_bitset.is_closed(conn_id)
    }

    /// Highest lease generation fenced via the lock-free immediate socket-close fence.
    pub fn atomic_fenced_lease_gen(&self) -> u64 {
        self.atomic_fenced_lease_gen.load(Ordering::SeqCst)
    }

    pub(super) fn enqueue_pending_disconnect(&self, conn: crate::rt_core::requests::lease::ConnectionHandle) {
        let mut q = self
            .pending_disconnects
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if !q.iter().any(|c| c.id() == conn.id()) {
            q.push_back(conn);
        }
        self.pending_disconnect_hold
            .store(q.len() as u64, Ordering::SeqCst);
    }

    pub(super) fn drain_pending_disconnects_under_guard(
        &self,
        inner: &mut SupervisorInner,
        target_conn_id: Option<u64>,
    ) -> Option<crate::rt_core::requests::lease::LeaseOutcome> {
        use crate::rt_core::requests::lease::{install_lease_disconnect, prepare_lease_disconnect};
        let mut target_outcome = None;
        loop {
            let next = {
                let q = self
                    .pending_disconnects
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                q.front().copied()
            };
            let Some(conn) = next else {
                break;
            };
            let prep = prepare_lease_disconnect(conn);
            let outcome = install_lease_disconnect(inner, &prep.request, Instant::now());
            if target_conn_id == Some(conn.id()) {
                target_outcome = Some(outcome);
            }
            let mut q = self
                .pending_disconnects
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            if q.front().is_some_and(|c| c.id() == conn.id()) {
                q.pop_front();
            }
            self.pending_disconnect_hold
                .store(q.len() as u64, Ordering::SeqCst);
        }
        target_outcome
    }

    pub(super) fn forward_pending_disconnects_to_executor(
        &self,
        handle: &crate::rt_core::executor::RtExecutorHandle,
    ) {
        loop {
            let next = {
                let q = self
                    .pending_disconnects
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                q.front().copied()
            };
            let Some(conn) = next else {
                break;
            };
            handle.enqueue_disconnect_detached(conn);
            let mut q = self
                .pending_disconnects
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            if q.front().is_some_and(|c| c.id() == conn.id()) {
                q.pop_front();
            }
            self.pending_disconnect_hold
                .store(q.len() as u64, Ordering::SeqCst);
        }
    }

    pub(super) fn try_drain_pending_disconnects_legacy(&self) {
        if self.pending_disconnect_hold.load(Ordering::Acquire) == 0 {
            return;
        }
        if let Ok(mut inner) = self.inner.try_write() {
            if !inner.detached && self.owner_mode() == OWNER_LEGACY {
                let _ = self.drain_pending_disconnects_under_guard(&mut inner, None);
            }
        }
    }

    /// Single non-blocking socket-close fence entry point (M1): marks the connection closed in the
    /// lock-free bitset, monotonically fences `local_gen` if non-zero, and dispatches a
    /// non-blocking owner-side fence BEFORE any task abort, join, or cleanup runs.
    ///
    /// - Under `OWNER_EXECUTOR`: immediately disconnects the broker client and pushes priority
    ///   `LifecycleCmd::Disconnect` onto the RT priority intake without awaiting.
    /// - Under `OWNER_LEGACY`: attempts `try_write()` to run `install_lease_disconnect` inline;
    ///   when the write lock is contended, queues the fence and activates fail-closed hold.
    /// - Under `OWNER_TRANSFERRING` / `OWNER_FAILED`: queues the fence to the winning owner and
    ///   activates fail-closed hold so teardown never returns unfenced and RT never inherits a
    ///   stale connection or lease.
    pub fn fence_ws_connection_immediate(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_gen: u64,
    ) {
        let _ = self.fence_ws_connection_immediate_outcome(conn, local_gen);
    }

    pub(super) fn fence_ws_connection_immediate_outcome(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_gen: u64,
    ) -> Option<crate::rt_core::requests::lease::LeaseOutcome> {
        use crate::rt_core::requests::lease::{install_lease_disconnect, prepare_lease_disconnect};
        if local_gen > 0 {
            self.atomic_fenced_lease_gen
                .fetch_max(local_gen, Ordering::SeqCst);
            let _ = self.lease_gen_mirror.compare_exchange(
                local_gen,
                0,
                Ordering::SeqCst,
                Ordering::SeqCst,
            );
        }
        if conn.id() != 0 {
            self.closed_bitset.mark_closed(conn.id());
        }
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                if let Some(handle) = self.executor_handle() {
                    if local_gen > 0 {
                        // Non-blocking: `atomic_fenced_lease_gen` is already set above, so if
                        // `servo_dispatcher`'s mutex is currently held, never block teardown; the
                        // owner and dispatcher sync the authoritative fence asynchronously.
                        let _ = handle.try_close_servo(local_gen);
                    }
                    handle.enqueue_disconnect_detached(conn);
                } else {
                    self.enqueue_pending_disconnect(conn);
                }
                None
            }
            OWNER_LEGACY => {
                if let Ok(mut inner) = self.inner.try_write() {
                    if !inner.detached && self.owner_mode() == OWNER_LEGACY {
                        let queued_out =
                            self.drain_pending_disconnects_under_guard(&mut inner, Some(conn.id()));
                        let prep = prepare_lease_disconnect(conn);
                        let direct_out =
                            install_lease_disconnect(&mut inner, &prep.request, Instant::now());
                        return Some(queued_out.unwrap_or(direct_out));
                    }
                }
                self.enqueue_pending_disconnect(conn);
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(handle) = self.executor_handle() {
                        self.forward_pending_disconnects_to_executor(&handle);
                    }
                }
                None
            }
            _ => {
                self.enqueue_pending_disconnect(conn);
                match self.owner_mode() {
                    OWNER_EXECUTOR => {
                        if let Some(handle) = self.executor_handle() {
                            self.forward_pending_disconnects_to_executor(&handle);
                        }
                    }
                    OWNER_LEGACY => {
                        self.try_drain_pending_disconnects_legacy();
                    }
                    _ => {}
                }
                None
            }
        }
    }

    /// Unified owner-routed WS connection disconnect (same `prepare_lease_disconnect` +
    /// `install_lease_disconnect` split on both legacy and executor; priority lossless channel
    /// on executor, never dropped when the ordinary broker ring is full).
    ///
    /// Runs the non-blocking immediate fence FIRST, then waits up to 500 ms for `OWNER_TRANSFERRING`
    /// or a contended legacy write lock to settle. If the lock or transfer is still held past the
    /// bound, the connection remains fenced in `closed_bitset`, queued in `pending_disconnects`,
    /// and held fail-closed (`pending_disconnect_hold > 0`) until the winning owner drains it.
    pub async fn disconnect_ws_connection(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) -> crate::rt_core::requests::lease::LeaseOutcome {
        use crate::rt_core::requests::lease::{
            install_lease_disconnect, prepare_lease_disconnect, LeaseOutcome,
        };
        if let Some(outcome) = self.fence_ws_connection_immediate_outcome(conn, 0) {
            return outcome;
        }
        let prep = prepare_lease_disconnect(conn);
        let settle_deadline = Instant::now() + Duration::from_millis(500);
        let mode = loop {
            let m = self.owner_mode();
            if m != OWNER_TRANSFERRING || Instant::now() >= settle_deadline {
                break m;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        };
        match mode {
            OWNER_LEGACY => {
                let rem = settle_deadline
                    .saturating_duration_since(Instant::now())
                    .max(Duration::from_millis(50));
                match tokio::time::timeout(rem, self.inner.write()).await {
                    Ok(mut inner) => {
                        if self.check_legacy_under_guard(&inner).is_ok() {
                            let queued_out = self
                                .drain_pending_disconnects_under_guard(&mut inner, Some(conn.id()));
                            let direct_out =
                                install_lease_disconnect(&mut inner, &prep.request, Instant::now());
                            return queued_out.unwrap_or(direct_out);
                        }
                        drop(inner);
                        if let Some(handle) = self.executor_handle() {
                            self.forward_pending_disconnects_to_executor(&handle);
                            if let Ok(outcome) = handle.disconnect_connection_async(conn).await {
                                return outcome;
                            }
                        }
                        LeaseOutcome::Disconnected {
                            released: false,
                            displaced_generation: 0,
                        }
                    }
                    Err(_) => {
                        // Write lock held past bound: keep queued & fail-closed (hold); never unfenced.
                        self.enqueue_pending_disconnect(conn);
                        if self.owner_mode() == OWNER_EXECUTOR {
                            if let Some(handle) = self.executor_handle() {
                                self.forward_pending_disconnects_to_executor(&handle);
                            }
                        }
                        LeaseOutcome::Disconnected {
                            released: false,
                            displaced_generation: 0,
                        }
                    }
                }
            }
            OWNER_EXECUTOR => {
                if let Some(handle) = self.executor_handle() {
                    self.forward_pending_disconnects_to_executor(&handle);
                    if let Ok(outcome) = handle.disconnect_connection_async(conn).await {
                        if let LeaseOutcome::Disconnected {
                            displaced_generation,
                            ..
                        } = outcome
                        {
                            if displaced_generation > 0 {
                                handle.close_servo(displaced_generation);
                            }
                        }
                        return outcome;
                    }
                }
                LeaseOutcome::Disconnected {
                    released: false,
                    displaced_generation: 0,
                }
            }
            _ => {
                // Still `OWNER_TRANSFERRING` past bound (or `OWNER_FAILED`): ensure queued & held
                // fail-closed so the winning owner drains it before any RT handoff or bus output.
                self.enqueue_pending_disconnect(conn);
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(handle) = self.executor_handle() {
                        self.forward_pending_disconnects_to_executor(&handle);
                    }
                }
                LeaseOutcome::Disconnected {
                    released: false,
                    displaced_generation: 0,
                }
            }
        }
    }

    /// Drop-time non-blocking disconnect safety net for aborted WS handler tasks.
    pub fn disconnect_ws_connection_detached(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        self.fence_ws_connection_immediate(conn, 0);
    }

    pub(super) fn map_lease_install_err(code: &'static str) -> ArmError {
        let status = match code {
            "OWNER_TRANSFERRED" | "DISCONNECTED" | "LEASE_LOST" | "NO_LEASE" | "STAMP_REUSED" => 409,
            "DEADLINE_EXPIRED" => 408,
            "MOTION_BUSY" => 503,
            _ => 400,
        };
        ArmError::Rejected {
            status,
            code: code.into(),
            message: format!("Lease operation refused: {code}"),
        }
    }

    pub(super) fn map_broker_dispatch_err(
        err: crate::rt_core::executor::BrokerDispatchError,
    ) -> ArmError {
        use crate::rt_core::executor::BrokerDispatchError;
        match err {
            BrokerDispatchError::Admit(e) => {
                let code = e.code();
                let status = if code == "DISCONNECTED" { 409 } else { 503 };
                ArmError::Rejected {
                    status,
                    code: code.into(),
                    message: format!("Broker admission refused: {code}"),
                }
            }
            BrokerDispatchError::Stopped => executor_unavailable(),
        }
    }

    /// Unified owner-routed WS `acquire_lease` (`force == false`): off-lock preparation from
    /// published snapshot + single owner-side [`install_lease_acquire`].
    pub async fn acquire_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        label: &str,
        client_seq: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_acquire, prepare_lease_acquire, LeaseInstallCtx, LeasePrepareError,
            LeaseRequest, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep = prepare_lease_acquire(&snap.owner, conn, label, client_seq, now, deadline)
            .map_err(|e| match e {
                LeasePrepareError::InvalidClientId => ArmError::Rejected {
                    status: 400,
                    code: "INVALID_CLIENT_ID".into(),
                    message: "client_id must be 1..=128 bytes".into(),
                },
                LeasePrepareError::InvalidConnection => ArmError::Rejected {
                    status: 400,
                    code: "INVALID_CONNECTION".into(),
                    message: "Invalid connection handle".into(),
                },
                LeasePrepareError::SnapshotUnavailable => observation_unavailable(),
            })?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("acquire_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_acquire(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        prep.client_seq,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Acquire(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => Ok(outcome),
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeaseAcquire".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `acquire_lease` (`force == true`): off-lock preparation from
    /// published snapshot + single owner-side [`install_lease_preempt`] with live [`OperatorGrant`].
    pub async fn preempt_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        label: &str,
        operator: crate::auth::OperatorGrant,
        client_seq: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_preempt, prepare_lease_preempt, LeaseInstallCtx, LeasePrepareError,
            LeaseRequest, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep =
            prepare_lease_preempt(&snap.owner, conn, label, operator, client_seq, now, deadline)
                .map_err(|e| match e {
                    LeasePrepareError::InvalidClientId => ArmError::Rejected {
                        status: 400,
                        code: "INVALID_CLIENT_ID".into(),
                        message: "client_id must be 1..=128 bytes".into(),
                    },
                    LeasePrepareError::InvalidConnection => ArmError::Rejected {
                        status: 400,
                        code: "INVALID_CONNECTION".into(),
                        message: "Invalid connection handle".into(),
                    },
                    LeasePrepareError::SnapshotUnavailable => observation_unavailable(),
                })?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("acquire_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_preempt(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        prep.client_seq,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Preempt(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => {
                        if let crate::rt_core::requests::lease::LeaseOutcome::Granted {
                            evicted_generation,
                            ..
                        } = outcome
                        {
                            if evicted_generation > 0 {
                                handle.close_servo(evicted_generation);
                            }
                        }
                        Ok(outcome)
                    }
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeasePreempt".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `release_lease`: off-lock preparation from published snapshot +
    /// single owner-side [`install_lease_release`].
    pub async fn release_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        generation: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_release, prepare_lease_release, LeaseInstallCtx, LeaseRequest,
            DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep = prepare_lease_release(&snap.owner, conn, generation, now, deadline)
            .map_err(|_| observation_unavailable())?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("release_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_release(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        generation,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Release(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => {
                        if let crate::rt_core::requests::lease::LeaseOutcome::Released {
                            displaced_generation,
                            ..
                        } = outcome
                        {
                            if displaced_generation > 0 {
                                handle.close_servo(displaced_generation);
                            }
                        }
                        Ok(outcome)
                    }
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeaseRelease".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `heartbeat`: off-lock preparation from published snapshot +
    /// `ImmediateOp::Heartbeat` in executor mode or `install_legacy_heartbeat` in legacy mode.
    #[allow(clippy::too_many_arguments)]
    pub async fn heartbeat_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_generation: u64,
        local_label: Option<&str>,
        supplied_client_id: Option<Option<&str>>,
        supplied_epoch: Option<u64>,
    ) -> Result<crate::rt_core::requests::lease::HeartbeatInstallOutcome, ArmError> {
        use crate::rt_core::broker::{ImmediateOp, Reply};
        use crate::rt_core::requests::lease::{
            install_legacy_heartbeat, prepare_heartbeat, HeartbeatInstallOutcome,
            PreparedHeartbeat, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let mode = self.owner_mode();
        if mode == OWNER_TRANSFERRING
            || mode == OWNER_FAILED
            || (mode == OWNER_EXECUTOR && self.executor_handle().is_none_or(|h| h.is_exited()))
        {
            return Err(observation_unavailable());
        }
        if local_generation == 0 {
            return Ok(HeartbeatInstallOutcome::LeaseLost {
                generation: 0,
                reason: None,
            });
        }
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        match prepare_heartbeat(
            &snap.owner,
            conn,
            local_generation,
            supplied_generation,
            local_label,
            supplied_client_id,
            supplied_epoch,
            now,
            deadline,
        ) {
            PreparedHeartbeat::LeaseLost { generation, reason } => {
                Ok(HeartbeatInstallOutcome::LeaseLost { generation, reason })
            }
            PreparedHeartbeat::Submit {
                connection,
                local_generation,
                supplied_epoch,
                mut authority,
                client,
                deadline,
                ..
            } => match self.owner_mode() {
                OWNER_LEGACY => {
                    let mut inner = self.inner.write().await;
                    inner.set_command("heartbeat");
                    self.check_legacy_under_guard(&inner)?;
                    install_legacy_heartbeat(
                        &mut inner,
                        connection,
                        local_generation,
                        supplied_epoch,
                        Instant::now(),
                    )
                    .map_err(Self::map_lease_install_err)
                }
                OWNER_EXECUTOR => {
                    let Some(handle) = self.executor_handle() else {
                        return Err(executor_unavailable());
                    };
                    // Bounded retry (at most 1) only when `stop_generation` or `config_revision`
                    // moved between snapshot preparation and commit: an ordinary Stop or config
                    // change must not cause a spurious `lease_lost` on heartbeat.
                    for attempt in 0..2 {
                        let reply = handle
                            .submit_immediate_async(
                                client,
                                0,
                                authority,
                                deadline,
                                ImmediateOp::Heartbeat,
                            )
                            .await
                            .map_err(Self::map_broker_dispatch_err)?;
                        match reply {
                            Reply::Ack { .. } => return Ok(HeartbeatInstallOutcome::Renewed),
                            Reply::Refused { code: "STOPPED" | "PLAN_STALE", .. } if attempt == 0 => {
                                let fresh = self.fresh_snapshot(None).await?;
                                let mut next_auth = fresh.owner.authority_at(Instant::now());
                                next_auth.lease_generation = local_generation;
                                next_auth.epoch = supplied_epoch.unwrap_or(u64::MAX);
                                authority = next_auth;
                            }
                            Reply::Refused { code: "EPOCH_MISMATCH", .. } => {
                                let latest = self.fresh_snapshot(None).await?;
                                return Ok(HeartbeatInstallOutcome::EpochSync {
                                    epoch: latest.obs.epoch_id,
                                    safety_state: latest.obs.safety_state.clone(),
                                });
                            }
                            Reply::Refused {
                                code: "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED",
                                ..
                            } => {
                                let reason = self
                                    .hub
                                    .latest()
                                    .map_or("expired", |s| {
                                        s.owner.lease_loss_reason(conn.as_str(), local_generation)
                                    });
                                return Ok(HeartbeatInstallOutcome::LeaseLost {
                                    generation: local_generation,
                                    reason: Some(reason),
                                });
                            }
                            Reply::Refused { code, .. } => {
                                return Err(Self::map_lease_install_err(code));
                            }
                            _ => {
                                return Err(ArmError::Internal(
                                    "Unexpected broker reply for Heartbeat".into(),
                                ));
                            }
                        }
                    }
                    Err(ArmError::Rejected {
                        status: 409,
                        code: "STOPPED".into(),
                        message: "Heartbeat superseded by concurrent stop".into(),
                    })
                }
                _ => Err(executor_unavailable()),
            },
        }
    }

    /// Registers an already-claimed off-thread WS slot with the active owner (legacy or RT executor).
    /// Returns `Err(DISCONNECTED)` without touching the owner if teardown already marked the slot closing.
    pub async fn register_ws_connection_for_slot(&self, conn_id: u64) -> Result<(), ArmError> {
        if !self.ws_slots.begin_open(conn_id) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "DISCONNECTED".into(),
                message: "Connection slot is already closing".into(),
            });
        }
        struct OpenGuard<'a> {
            pool: &'a WsConnectionSlotPool,
            conn_id: u64,
            succeeded: bool,
        }
        impl Drop for OpenGuard<'_> {
            fn drop(&mut self) {
                self.pool.finish_open(self.conn_id, self.succeeded);
            }
        }
        let mut guard = OpenGuard {
            pool: &self.ws_slots,
            conn_id,
            succeeded: false,
        };
        let res = self.register_ws_connection(conn_id).await;
        if res.is_ok() {
            guard.succeeded = true;
        }
        res
    }
}
