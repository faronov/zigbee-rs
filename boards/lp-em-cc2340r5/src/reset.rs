//! CC2340R52 system-reset resource.

/// Exclusive access to the Cortex-M system-reset mechanism.
pub struct SystemReset {
    _private: (),
}

impl SystemReset {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Issue the architectural Cortex-M system-reset request.
    pub fn reset(&self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}
