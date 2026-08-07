//! Pure poll-delay arbitration for [`crate::app::SensorApp`], factored out so
//! it can be tested on the host.
//!
//! This module touches nothing but `core` and `zigbee_runtime::event_loop`
//! (both host-buildable — see `zigbee-runtime`'s own `cargo test`), unlike
//! the rest of this crate which needs the real `thumbv7em-none-eabihf`
//! target and Nordic hardware. `tests/src/nrf52840_policy_tests.rs`
//! `#[path]`-includes this exact file into the workspace's host test crate,
//! the same way `tests/src/efr32mg1_pm_tests.rs` mirrors `efr32mg1-hal`'s
//! `pm.rs`, so the arbitration logic exercised on the host is byte-for-byte
//! what runs on hardware.

use zigbee_runtime::event_loop::TickResult;

/// Extract the runtime's requested "run again within this many ms" from a
/// tick result, if any.
///
/// [`TickResult::RunAgain`] is the stack asking to be ticked again sooner
/// than the caller's own periodic schedule — for example while a Trust
/// Center link-key exchange or a light-sleep decision is in progress.
/// `Idle` and `Event` results carry no such request.
pub fn run_again_delay_ms(result: &TickResult) -> Option<u32> {
    match result {
        TickResult::Idle | TickResult::Event(_) => None,
        TickResult::RunAgain(delay_ms) => Some(*delay_ms),
    }
}

/// Resolve the next poll/sleep delay, honoring an earlier `RunAgain`
/// deadline over the fixed fast/slow poll window.
///
/// A `RunAgain` request only ever *shortens* the wait — it never lengthens
/// past whatever fast/slow poll window is already in effect — and a
/// zero-millisecond request is treated as "as soon as possible" (1 ms)
/// rather than a busy loop.
pub fn resolve_poll_delay_ms(base_poll_ms: u64, run_again_ms: Option<u64>) -> u64 {
    match run_again_ms {
        Some(delay_ms) => base_poll_ms.min(delay_ms.max(1)),
        None => base_poll_ms,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::event_loop::StackEvent;

    #[test]
    fn idle_and_event_carry_no_run_again_request() {
        assert_eq!(run_again_delay_ms(&TickResult::Idle), None);
        assert_eq!(
            run_again_delay_ms(&TickResult::Event(StackEvent::Left)),
            None
        );
    }

    #[test]
    fn run_again_is_extracted_verbatim() {
        assert_eq!(run_again_delay_ms(&TickResult::RunAgain(42)), Some(42));
    }

    #[test]
    fn run_again_only_shortens_the_wait() {
        assert_eq!(resolve_poll_delay_ms(30_000, Some(1_500)), 1_500);
        assert_eq!(resolve_poll_delay_ms(250, Some(1_500)), 250);
        assert_eq!(resolve_poll_delay_ms(30_000, None), 30_000);
    }

    #[test]
    fn a_zero_run_again_delay_does_not_busy_loop() {
        assert_eq!(resolve_poll_delay_ms(30_000, Some(0)), 1);
    }
}
