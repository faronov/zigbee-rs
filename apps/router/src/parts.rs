//! Explicit resource bundle supplied by the composition root.

/// Concrete status, supervision, and diagnostics capabilities.
///
/// This is only an ownership bundle. It does not acquire peripherals or
/// provide MAC, clock, storage, profile, or platform lookup methods.
pub struct RouterParts<St, Sv, D> {
    pub status: St,
    pub supervisor: Sv,
    pub diagnostics: D,
}

impl<St, Sv, D> RouterParts<St, Sv, D> {
    pub const fn new(status: St, supervisor: Sv, diagnostics: D) -> Self {
        Self {
            status,
            supervisor,
            diagnostics,
        }
    }
}
