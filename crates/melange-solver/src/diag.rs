//! Build-scoped warnings.
//!
//! A build resolves device parameters and solves operating points at several
//! steps (junction caps, reductions, the preflight, routing, each IR build),
//! and a warning raised there was logged at every step: a one-triode deck
//! printed the same grid-law warning five times. While a [`BuildScope`] is
//! active on the thread, [`warn`] collects instead of logging, and when the
//! scope ends each distinct warning is logged once, in the order first raised.
//! Outside a build, [`warn`] logs at once.

use std::cell::RefCell;

thread_local! {
    static SINK: RefCell<Option<Vec<(&'static str, String)>>> = const { RefCell::new(None) };
}

/// Log `message` as a warning from `target`, once per build (see the module
/// doc). Use through [`diag_warn!`](crate::diag_warn).
pub fn warn(target: &'static str, message: String) {
    let logged_now = SINK.with(|sink| match sink.borrow_mut().as_mut() {
        Some(seen) => {
            if !seen.iter().any(|(_, m)| *m == message) {
                seen.push((target, message.clone()));
            }
            false
        }
        None => true,
    });
    if logged_now {
        log::warn!(target: target, "{message}");
    }
}

/// `log::warn!`, collected per build (see [`crate::diag`]).
#[macro_export]
macro_rules! diag_warn {
    ($($arg:tt)*) => {
        $crate::diag::warn(module_path!(), format!($($arg)*))
    };
}

/// Collects [`warn`]ings on this thread until dropped, then logs each once.
/// A scope opened inside another adds to the outer one.
pub struct BuildScope {
    owner: bool,
}

impl BuildScope {
    /// Start collecting.
    pub fn begin() -> Self {
        let owner = SINK.with(|sink| {
            let mut sink = sink.borrow_mut();
            if sink.is_none() {
                *sink = Some(Vec::new());
                true
            } else {
                false
            }
        });
        BuildScope { owner }
    }
}

impl Drop for BuildScope {
    fn drop(&mut self) {
        if !self.owner {
            return;
        }
        let collected = SINK
            .with(|sink| sink.borrow_mut().take())
            .unwrap_or_default();
        for (target, message) in collected {
            log::warn!(target: target, "{message}");
        }
    }
}
