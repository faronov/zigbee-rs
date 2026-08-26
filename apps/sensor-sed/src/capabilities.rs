//! Compile-time platform capabilities used by the shared SED lifecycle.

use zigbee_mac::MacDriver;

use crate::policy::SleepDepth;

/// Why an atomic platform wait returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeReason {
    Button,
    Timer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WaitRequest {
    pub timeout_ms: u32,
    pub sleep_depth: SleepDepth,
}

/// Monotonic time, user wake input, and the atomic MAC wait transition.
///
/// A non-active wait must be atomic from the application's point of view:
/// quiesce the MAC/radio, enter the selected sleep, then make the MAC ready for
/// its next normal operation before returning `Ok`.
///
/// A driver that guarantees lazy restoration from its quiescent state may
/// return with the radio disabled when its next normal RX/TX operation performs
/// the complete transition itself. Platforms whose clocks, radio, or MAC state
/// require eager post-wake restoration must perform that restoration inside
/// [`wait`](Self::wait), before returning `Ok`. Preparation failure returns
/// `Err` without entering the wait; required restoration failure also returns
/// `Err`.
#[allow(async_fn_in_trait)]
pub trait WakeController<M: MacDriver> {
    /// Opaque platform-native monotonic timestamp.
    type Mark: Copy;
    type Error;

    fn mark(&self) -> Self::Mark;

    /// Add a bounded millisecond duration using the mark's native rollover
    /// rules.
    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark;

    /// Return bounded elapsed milliseconds with native rollover handled by the
    /// implementation.
    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32;

    async fn wait(&mut self, mac: &mut M, request: WaitRequest) -> Result<WakeReason, Self::Error>;

    async fn button_held_for(&mut self, duration_ms: u32) -> bool;
    async fn delay_ms(&mut self, duration_ms: u32);
}

/// Semantic product status, independent of LED count, color, or polarity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorStatus {
    Off,
    Joining { on: bool },
    Joined { active: bool },
    Identifying { on: bool },
    Reporting { on: bool },
    Resetting { on: bool },
    Ota,
    Fault,
}

pub trait StatusSink {
    /// Whether this product has a fitted status indicator.
    ///
    /// The shared lifecycle uses this compile-time capability to remove
    /// status-only deadlines and delay waits entirely when no indicator is
    /// fitted. Implementations with real hardware inherit `true`.
    const PRESENT: bool = true;

    fn set(&mut self, status: SensorStatus);
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoStatus;

impl StatusSink for NoStatus {
    const PRESENT: bool = false;

    fn set(&mut self, _status: SensorStatus) {}
}

/// Reset and watchdog supervision selected by the composition root.
pub trait Supervisor {
    /// Feed or service the watchdog after useful application progress.
    fn heartbeat(&mut self);

    /// Maximum safe wait before the watchdog must be serviced.
    fn max_wait_ms(&self) -> Option<u32>;

    fn reset(&mut self) -> !;
}
