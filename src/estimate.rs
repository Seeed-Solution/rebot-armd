//! Estimate chain for one grasp quantity (step 1: the object height, `height_sources`,
//! `height_record`, docs/agent_action_api.md).
//!
//! Sources are offered in evaluation order. The first deciding source that accepts is chosen;
//! record-only sources are reported but never chosen and never veto. A veto (OBJECT_TOO_TALL)
//! from a deciding source is kept (the first one) and returned only when no later source accepts.
use serde::{Deserialize, Serialize};

use crate::macro_action::Stop;

/// Result of one source.
pub enum SourceResult {
    Accept(f64),
    Reject(String),
    Veto(Stop),
}

/// One evaluated source, reported in `ActionStatus.height_estimates`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EstimateEntry {
    /// `depth:<anchor>`, `depth`, `scale`, `parallax`, `probe`, `prior`.
    pub source: String,
    /// Height above the table (m) the source measured, also when rejected or vetoed.
    pub value: Option<f32>,
    pub used: bool,
    pub record_only: bool,
    /// Reject reason or veto code.
    pub reason: Option<String>,
}

#[derive(Default)]
pub struct HeightChain {
    entries: Vec<EstimateEntry>,
    veto: Option<Stop>,
    chosen: Option<(f64, String)>,
}

impl HeightChain {
    pub fn new() -> Self { Self::default() }

    /// Record the result of `source` (`value`: the measured height, if any); true when it is
    /// chosen now.
    pub fn offer(&mut self, source: &str, value: Option<f64>, r: SourceResult, record_only: bool) -> bool {
        let mut entry = EstimateEntry { source: source.into(), value: value.map(|v| v as f32), used: false, record_only, reason: None };
        let mut chosen = false;
        match r {
            SourceResult::Accept(h) => {
                entry.value = Some(h as f32);
                if !record_only && self.chosen.is_none() {
                    self.chosen = Some((h, source.into()));
                    entry.used = true;
                    chosen = true;
                }
            }
            SourceResult::Reject(reason) => entry.reason = Some(reason),
            SourceResult::Veto(stop) => {
                entry.reason = Some(match &stop { Stop::Fail(code, _) => code.clone(), Stop::Cancelled => "cancelled".into() });
                if !record_only && self.veto.is_none() { self.veto = Some(stop); }
            }
        }
        self.entries.push(entry);
        chosen
    }

    pub fn done(&self) -> bool { self.chosen.is_some() }

    pub fn vetoed(&self) -> bool { self.veto.is_some() }

    pub fn entries(&self) -> &[EstimateEntry] { &self.entries }

    /// Chosen (height, source) -> Ok(Some); nothing chosen and a veto -> Err(veto); neither -> Ok(None).
    pub fn finish(self) -> Result<(Option<(f64, String)>, Vec<EstimateEntry>), (Stop, Vec<EstimateEntry>)> {
        match (self.chosen, self.veto) {
            (Some(c), _) => Ok((Some(c), self.entries)),
            (None, Some(v)) => Err((v, self.entries)),
            (None, None) => Ok((None, self.entries)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::macro_action::fail;

    fn tall() -> SourceResult { SourceResult::Veto(fail("OBJECT_TOO_TALL", "tall")) }

    #[test]
    fn first_accept_wins_and_entries_keep_order() {
        let mut c = HeightChain::new();
        assert!(!c.offer("depth", None, SourceResult::Reject("no_measurement".into()), false));
        assert!(c.offer("scale", None, SourceResult::Accept(0.04), false));
        assert!(c.done());
        assert!(!c.offer("prior", None, SourceResult::Accept(0.05), false));
        let (chosen, entries) = c.finish().ok().unwrap();
        assert_eq!(chosen, Some((0.04, "scale".to_string())));
        let used: Vec<_> = entries.iter().map(|e| (e.source.as_str(), e.used)).collect();
        assert_eq!(used, [("depth", false), ("scale", true), ("prior", false)]);
        assert_eq!(entries[0].reason.as_deref(), Some("no_measurement"));
    }

    #[test]
    fn record_only_is_never_chosen_and_never_vetoes() {
        let mut c = HeightChain::new();
        assert!(!c.offer("depth:metric_table", None, SourceResult::Accept(0.03), true));
        assert!(!c.offer("prior", Some(0.2), tall(), true));
        assert!(!c.done());
        let (chosen, entries) = c.finish().ok().unwrap();
        assert_eq!(chosen, None);
        assert_eq!(entries[0].value, Some(0.03));
        assert!(entries[0].record_only && !entries[0].used);
        assert_eq!(entries[1].reason.as_deref(), Some("OBJECT_TOO_TALL"));
        assert_eq!(entries[1].value, Some(0.2));
    }

    #[test]
    fn veto_only_when_nothing_accepts() {
        let mut c = HeightChain::new();
        c.offer("depth", Some(0.2), tall(), false);
        c.offer("probe", None, SourceResult::Reject("border".into()), false);
        let (stop, entries) = c.finish().err().unwrap();
        assert!(matches!(stop, Stop::Fail(ref code, _) if code == "OBJECT_TOO_TALL"));
        assert_eq!(entries.len(), 2);

        let mut c = HeightChain::new();
        c.offer("depth", Some(0.2), tall(), false);
        c.offer("scale", None, SourceResult::Accept(0.05), false);
        assert_eq!(c.finish().ok().unwrap().0, Some((0.05, "scale".to_string())));
    }

    #[test]
    fn the_first_veto_is_kept() {
        let mut c = HeightChain::new();
        c.offer("depth", None, SourceResult::Veto(fail("OBJECT_TOO_TALL", "first")), false);
        c.offer("probe", None, SourceResult::Veto(fail("OBJECT_TOO_TALL", "second")), false);
        let (stop, _) = c.finish().err().unwrap();
        assert!(matches!(stop, Stop::Fail(_, ref m) if m == "first"));
    }
}
