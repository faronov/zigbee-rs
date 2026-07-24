//! Event-driven unique Trust Center link-key (TCLK) exchange state machine.
//!
//! This mirrors the Silicon Labs GSDK 4.5 split between the *network-steering*
//! plugin (scan → join → initial Transport-Key) and the *update-tc-link-key*
//! plugin (Node_Desc → APS Request-Key → Verify-Key → Confirm-Key), which the
//! stack advances through scheduled events **after** the network is up.
//!
//! In this crate the pre-network work stays awaited inside
//! [`crate::BdbLayer::network_steering`]. Once the device has the network key,
//! has reserved network security, and has sent `Device_annce`, the post-network
//! unique-TCLK handshake is captured here as an explicit bounded state machine
//! that the application/runtime advances one step per tick/poll (see
//! [`crate::BdbLayer::advance_tclk_exchange`]).
//!
//! The machine performs **at most one non-blocking action per step** — either a
//! single bounded transmit, or a non-blocking check of already-received ZDO /
//! APS security state — so normal ZDO/ZCL processing and sleepy-end-device
//! polling continue between steps instead of being monopolised by one long
//! future.

use zigbee_types::{IeeeAddress, ShortAddress};

/// Number of complete Node_Desc → Verify/Confirm attempts before failure.
///
/// Matches the official Telink/GSDK budget of one initial request plus three
/// retries.
pub(crate) const TCLK_EXCHANGE_ATTEMPTS: u8 = 4;
/// Delay after `Device_annce` before the first Node_Desc request.
pub(crate) const TCLK_EXCHANGE_START_DELAY_US: u32 = 1_200_000;
/// Per-message response window for Node_Desc, Request-Key, and Verify-Key.
pub(crate) const TCLK_EXCHANGE_TIMEOUT_US: u32 = 5_000_000;

/// Stage of the bounded unique-TCLK handshake.
///
/// Each stage advances by a single bounded action per
/// [`crate::BdbLayer::advance_tclk_exchange`] call. `Send*` stages perform one
/// transmit; `Await*` stages check non-blocking state and enforce the
/// per-attempt timeout; `AttemptCooldown` drains the remainder of a failed
/// attempt's window before retrying.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TclkStage {
    /// Waiting out the post-announce start delay before the first attempt.
    StartDelay,
    /// Send Node_Desc_req to the Trust Center (start of an attempt).
    SendNodeDesc,
    /// Await the Node_Desc_rsp to determine the Trust Center stack revision.
    AwaitNodeDesc,
    /// Send the APS Request-Key for a unique Trust Center link key.
    SendRequestKey,
    /// Await installation of the unique Trust Center link key.
    AwaitTclk,
    /// Send the APS Verify-Key proving possession of the unique key.
    SendVerifyKey,
    /// Await a successful Confirm-Key from the Trust Center.
    AwaitConfirmKey,
    /// Drain the rest of a failed attempt's window before retrying.
    AttemptCooldown,
    /// Terminal: exchange completed (pre-R21 or confirmed unique key).
    Complete,
    /// Terminal: exchange failed after exhausting the attempt budget.
    Failed,
}

/// Result of advancing the exchange by one bounded step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TclkProgress {
    /// The exchange is still running; call again next tick/poll.
    InProgress,
    /// The unique-TCLK exchange finished successfully (or was not required).
    Complete,
    /// The exchange failed; the network has been reset and left consistently.
    Failed(crate::BdbStatus),
}

/// Bounded storage for an in-flight unique-TCLK exchange.
///
/// Stored in [`crate::BdbLayer`] between ticks. Contains no heap allocations
/// and no borrows — the driver takes it out, advances one step, and stores it
/// back while `InProgress`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TclkExchange {
    pub stage: TclkStage,
    pub(crate) tc_addr: ShortAddress,
    pub(crate) tc_ieee: IeeeAddress,
    pub(crate) attempts_remaining: u8,
    pub(crate) armed_at_us: u32,
    pub(crate) attempt_started_us: u32,
    pub(crate) node_desc_slot: Option<usize>,
    pub(crate) confirm_success_baseline: u32,
    pub(crate) confirm_reject_baseline: u32,
}

impl TclkExchange {
    /// Arm a fresh exchange immediately after `Device_annce`.
    ///
    /// `now` is the current monotonic time in microseconds.
    pub(crate) fn new(tc_addr: ShortAddress, tc_ieee: IeeeAddress, now: u32) -> Self {
        Self {
            stage: TclkStage::StartDelay,
            tc_addr,
            tc_ieee,
            attempts_remaining: TCLK_EXCHANGE_ATTEMPTS,
            armed_at_us: now,
            attempt_started_us: now,
            node_desc_slot: None,
            confirm_success_baseline: 0,
            confirm_reject_baseline: 0,
        }
    }

    /// Whether the post-announce start delay has elapsed.
    pub(crate) fn start_delay_elapsed(&self, now: u32) -> bool {
        now.wrapping_sub(self.armed_at_us) >= TCLK_EXCHANGE_START_DELAY_US
    }

    /// Whether the current protocol stage has exhausted its 5 s window.
    pub(crate) fn attempt_timed_out(&self, now: u32) -> bool {
        now.wrapping_sub(self.attempt_started_us) >= TCLK_EXCHANGE_TIMEOUT_US
    }

    /// Start a fresh response window within the current exchange attempt.
    pub(crate) fn restart_stage_timeout(&mut self, now: u32) {
        self.attempt_started_us = now;
    }

    /// Begin a (re)attempt: reset the per-attempt clock and slot, and move to
    /// the initial `SendNodeDesc` stage.
    pub(crate) fn begin_attempt(&mut self, now: u32) {
        self.stage = TclkStage::SendNodeDesc;
        self.attempt_started_us = now;
        self.node_desc_slot = None;
    }

    /// Record a failed attempt.
    ///
    /// Returns `true` when the attempt budget is exhausted and the exchange
    /// must fail; `false` when at least one attempt remains.
    pub(crate) fn record_attempt_failure(&mut self) -> bool {
        self.attempts_remaining = self.attempts_remaining.saturating_sub(1);
        self.attempts_remaining == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TC_ADDR: ShortAddress = ShortAddress(0x0000);
    const TC_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];

    fn armed(now: u32) -> TclkExchange {
        TclkExchange::new(TC_ADDR, TC_IEEE, now)
    }

    #[test]
    fn new_arms_in_start_delay_with_full_budget() {
        let ex = armed(1_000);
        assert_eq!(ex.stage, TclkStage::StartDelay);
        assert_eq!(ex.attempts_remaining, TCLK_EXCHANGE_ATTEMPTS);
        assert_eq!(ex.node_desc_slot, None);
    }

    #[test]
    fn start_delay_elapses_after_the_configured_window() {
        let ex = armed(1_000);
        assert!(!ex.start_delay_elapsed(1_000));
        assert!(!ex.start_delay_elapsed(1_000 + TCLK_EXCHANGE_START_DELAY_US - 1));
        assert!(ex.start_delay_elapsed(1_000 + TCLK_EXCHANGE_START_DELAY_US));
    }

    #[test]
    fn begin_attempt_moves_to_send_and_resets_attempt_clock() {
        let mut ex = armed(0);
        ex.node_desc_slot = Some(3);
        ex.begin_attempt(10_000);
        assert_eq!(ex.stage, TclkStage::SendNodeDesc);
        assert_eq!(ex.attempt_started_us, 10_000);
        assert_eq!(ex.node_desc_slot, None);
        assert!(!ex.attempt_timed_out(10_000));
        assert!(ex.attempt_timed_out(10_000 + TCLK_EXCHANGE_TIMEOUT_US));
    }

    #[test]
    fn attempt_timeout_uses_wrapping_arithmetic() {
        // Arm near the u32 wraparound boundary.
        let start = u32::MAX - 100;
        let mut ex = armed(start);
        ex.begin_attempt(start);
        let after_wrap = start.wrapping_add(TCLK_EXCHANGE_TIMEOUT_US);
        assert!(ex.attempt_timed_out(after_wrap));
        assert!(!ex.attempt_timed_out(start.wrapping_add(TCLK_EXCHANGE_TIMEOUT_US - 1)));
    }

    #[test]
    fn response_window_can_restart_within_an_attempt() {
        let mut ex = armed(0);
        ex.begin_attempt(1_000);
        ex.restart_stage_timeout(2_000);
        assert!(!ex.attempt_timed_out(2_000 + TCLK_EXCHANGE_TIMEOUT_US - 1));
        assert!(ex.attempt_timed_out(2_000 + TCLK_EXCHANGE_TIMEOUT_US));
    }

    #[test]
    fn attempt_budget_exhausts_after_configured_attempts() {
        let mut ex = armed(0);
        // Three failures still leave a final attempt.
        for _ in 0..(TCLK_EXCHANGE_ATTEMPTS - 1) {
            assert!(!ex.record_attempt_failure());
        }
        // The last failure exhausts the budget.
        assert!(ex.record_attempt_failure());
        // Further failures stay exhausted without underflow.
        assert!(ex.record_attempt_failure());
        assert_eq!(ex.attempts_remaining, 0);
    }
}
