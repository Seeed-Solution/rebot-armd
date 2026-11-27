//! Wire epoch guard of the writer task.

use serde_json::{json, Value};

/// Single-convergence-point wire epoch guard in the writer task: ensures outbound control
/// frames (`command_ack`, `command_reject`, delayed acks, etc.) never regress `epoch_id` or
/// `safety_state` behind an already-emitted newer `epoch_sync` or control frame.
pub(super) fn enforce_monotonic_epoch(last_wire_epoch: &mut Option<(u64, Option<Value>)>, reply: &mut Value) {
    let Some(obj) = reply.as_object_mut() else { return; };
    let is_sync = obj.get("type").and_then(Value::as_str) == Some("epoch_sync");
    if let Some(ep) = obj.get("epoch_id").and_then(Value::as_u64) {
        let st = obj.get("safety_state").cloned();
        if let Some((prev_ep, prev_st)) = last_wire_epoch.as_ref() {
            if ep < *prev_ep {
                obj.insert("epoch_id".into(), json!(prev_ep));
                if obj.contains_key("safety_state") {
                    if let Some(s) = prev_st {
                        obj.insert("safety_state".into(), s.clone());
                    }
                }
                return;
            }
            if ep == *prev_ep && is_sync && st.is_some() {
                *last_wire_epoch = Some((ep, st));
                return;
            }
        }
        if is_sync || last_wire_epoch.as_ref().is_none_or(|(p, _)| ep > *p) {
            let next_st = st.or_else(|| last_wire_epoch.as_ref().and_then(|(_, s)| s.clone()));
            *last_wire_epoch = Some((ep, next_st));
        }
    }
}
