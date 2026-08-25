//! Compile-time platform capabilities used by the shared SED lifecycle.

use zigbee_mac::{MacDriver, MacError};

/// Why the application left its bounded poll wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeReason {
    Button,
    Timer,
}

/// Clock, user-interface, delay, and reset services needed by the lifecycle.
///
/// The associated instant keeps platform-native clock precision and rollover
/// behavior. Implementations own all GPIO polarity and executor/timer details;
/// the shared application only uses semantic LED operations and bounded waits.
#[allow(async_fn_in_trait)]
pub trait LifecyclePlatform {
    type Instant: Copy + Ord;

    fn now(&self) -> Self::Instant;
    fn add_millis(instant: Self::Instant, duration_ms: u64) -> Self::Instant;
    fn elapsed_millis(later: Self::Instant, earlier: Self::Instant) -> u64;

    async fn wait_for_wake(&mut self, timeout_ms: u64) -> WakeReason;
    async fn button_held_for(&mut self, duration_ms: u64) -> bool;
    async fn delay_ms(&mut self, duration_ms: u64);

    fn led_on(&mut self);
    fn led_off(&mut self);
    fn led_toggle(&mut self);
    fn reset(&mut self) -> !;
}

/// MAC-specific power transition performed immediately before a poll wait.
///
/// Keeping this separate from [`LifecyclePlatform`] allows the same board I/O
/// implementation to pair with different silicon MAC backends without
/// downcasts or runtime capability checks.
pub trait RadioPower<M: MacDriver> {
    fn prepare_for_sleep(&mut self, mac: &mut M) -> Result<(), MacError>;
}
