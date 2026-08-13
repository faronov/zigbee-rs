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
//!
//! # Retry model (GSDK `emberUpdateTcLinkKey(maxAttempts)`)
//!
//! GSDK budgets attempts **per message type**, not per whole procedure: the
//! `maxAttempts` value is applied independently to the Node Descriptor probe,
//! to the APS Request-Key, and to the Verify-Key, and it is reset when the
//! next message type starts. A lost Confirm-Key therefore retransmits the
//! Verify-Key — it never restarts the Node Descriptor probe, and it never
//! discards the unique key the Trust Center already installed.
//!
//! This module implements exactly that: three independent transmission budgets
//! of [`TCLK_MESSAGE_ATTEMPTS`] each, a short explicit retry backoff,
//! per-message response windows, and one wrapping-safe overall deadline that
//! strictly fails the exchange.

use zigbee_types::{IeeeAddress, ShortAddress};

/// Transmissions allowed per message type before that stage's budget is
/// exhausted (GSDK `EMBER_AF_PLUGIN_UPDATE_TC_LINK_KEY_MAX_ATTEMPTS`).
///
/// The budget is *per message type* and is reset when a new message type is
/// started, so Node_Desc, Request-Key and Verify-Key each get their own three
/// transmissions.
pub(crate) const TCLK_MESSAGE_ATTEMPTS: u8 = 3;

/// Delay before retransmitting a message after a synchronous transmit failure
/// or a rejected Confirm-Key.
///
/// GSDK advances the update-TCLK procedure from scheduled events rather than
/// retrying on the next application tick. A 250 ms delay is long enough to
/// avoid burning all three budgets on consecutive 50 ms runtime ticks, while
/// remaining far shorter than the old whole-procedure 5 s cooldown.
pub(crate) const TCLK_RETRY_BACKOFF_US: u32 = 250_000;

/// Delay after `Device_annce` before the first Node_Desc request.
///
/// GSDK schedules the update-tc-link-key event with a short jitter right after
/// the announce; a fixed short delay keeps the whole first authentication pass
/// inside [`TCLK_EXCHANGE_DEADLINE_US`].
pub(crate) const TCLK_EXCHANGE_START_DELAY_US: u32 = 300_000;

/// Response window for the Trust Center Node Descriptor probe.
pub(crate) const TCLK_NODE_DESC_TIMEOUT_US: u32 = 1_500_000;

/// Response window for APS Request-Key → Transport-Key (unique TCLK install).
pub(crate) const TCLK_REQUEST_KEY_TIMEOUT_US: u32 = 3_000_000;

/// Response window for APS Verify-Key → Confirm-Key.
///
/// Matches GSDK `VERIFY_KEY_TIMEOUT_MS` (5 s) in
/// `app/framework/plugin/network-steering/network-steering.c`.
pub(crate) const TCLK_VERIFY_KEY_TIMEOUT_US: u32 = 5_000_000;

/// Overall deadline for the whole post-announce handshake.
///
/// Measured from the moment the exchange is armed. Expiry is a strict failure:
/// the exchange never silently keeps running or converts a transport
/// acknowledgement into key-verification success.
pub(crate) const TCLK_EXCHANGE_DEADLINE_US: u32 = 15_000_000;

/// Worst-case duration of the *first* pass through every stage.
///
/// One start delay plus one full response window per message type. This must
/// stay strictly below [`TCLK_EXCHANGE_DEADLINE_US`] so a Trust Center that
/// answers slowly — but answers — is never cut off mid-handshake by the
/// overall deadline.
pub(crate) const TCLK_FIRST_PASS_BUDGET_US: u32 = TCLK_EXCHANGE_START_DELAY_US
    + TCLK_NODE_DESC_TIMEOUT_US
    + TCLK_REQUEST_KEY_TIMEOUT_US
    + TCLK_VERIFY_KEY_TIMEOUT_US;

/// Maximum inter-attempt delay when each message type needs all three
/// transmissions before one full response window succeeds.
pub(crate) const TCLK_MAX_RETRY_BACKOFF_BUDGET_US: u32 =
    (TCLK_MESSAGE_ATTEMPTS as u32 - 1) * 3 * TCLK_RETRY_BACKOFF_US;

const _: () = assert!(TCLK_FIRST_PASS_BUDGET_US < TCLK_EXCHANGE_DEADLINE_US);
const _: () = assert!(
    TCLK_FIRST_PASS_BUDGET_US + TCLK_MAX_RETRY_BACKOFF_BUDGET_US < TCLK_EXCHANGE_DEADLINE_US
);

/// Stage of the bounded unique-TCLK handshake.
///
/// Each stage advances by a single bounded action per
/// [`crate::BdbLayer::advance_tclk_exchange`] call. `Send*` stages consume one
/// transmission from their message type's budget and perform one transmit;
/// `Retry*` stages wait out the short scheduled retransmission backoff;
/// `Await*` stages check non-blocking state and enforce that message type's
/// response window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TclkStage {
    /// Waiting out the post-announce start delay before the first probe.
    StartDelay,
    /// Send Node_Desc_req to the Trust Center.
    SendNodeDesc,
    /// Pace a Node_Desc_req retransmission after a synchronous send failure.
    RetryNodeDesc,
    /// Await the Node_Desc_rsp to determine the Trust Center stack revision.
    AwaitNodeDesc,
    /// Send the APS Request-Key for a unique Trust Center link key.
    SendRequestKey,
    /// Pace a Request-Key retransmission after a synchronous send failure.
    RetryRequestKey,
    /// Await installation of the unique Trust Center link key.
    AwaitTclk,
    /// Send the APS Verify-Key proving possession of the unique key.
    SendVerifyKey,
    /// Pace Verify-Key after a send failure or rejected Confirm-Key.
    RetryVerifyKey,
    /// Await a successful Confirm-Key from the Trust Center.
    AwaitConfirmKey,
    /// Terminal: exchange completed (pre-R21 or confirmed unique key).
    Complete,
    /// Terminal: exchange failed (deadline or an exhausted message budget).
    Failed,
}

impl TclkStage {
    /// Timing window that applies while this stage is current.
    pub(crate) const fn window_us(self) -> u32 {
        match self {
            Self::StartDelay => TCLK_EXCHANGE_START_DELAY_US,
            Self::RetryNodeDesc | Self::RetryRequestKey | Self::RetryVerifyKey => {
                TCLK_RETRY_BACKOFF_US
            }
            Self::SendNodeDesc | Self::AwaitNodeDesc => TCLK_NODE_DESC_TIMEOUT_US,
            Self::SendRequestKey | Self::AwaitTclk => TCLK_REQUEST_KEY_TIMEOUT_US,
            Self::SendVerifyKey | Self::AwaitConfirmKey => TCLK_VERIFY_KEY_TIMEOUT_US,
            Self::Complete | Self::Failed => TCLK_EXCHANGE_DEADLINE_US,
        }
    }

    /// Whether this stage is terminal.
    pub(crate) const fn is_terminal(self) -> bool {
        matches!(self, Self::Complete | Self::Failed)
    }
}

/// Result of advancing the exchange by one bounded step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TclkProgress {
    /// The exchange is still running; call again next tick/poll.
    InProgress,
    /// The unique-TCLK exchange finished successfully (or was not required).
    Complete,
    /// The exchange failed; the device left the network and cleaned up.
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
    /// Node_Desc_req transmissions left.
    pub(crate) node_desc_budget: u8,
    /// APS Request-Key transmissions left.
    pub(crate) request_key_budget: u8,
    /// APS Verify-Key transmissions left.
    pub(crate) verify_key_budget: u8,
    /// Monotonic time at which the exchange was armed (overall deadline base).
    pub(crate) armed_at_us: u32,
    /// Monotonic time at which the current stage's window started.
    pub(crate) stage_started_us: u32,
    pub(crate) node_desc_slot: Option<usize>,
    pub(crate) confirm_success_baseline: u32,
    pub(crate) confirm_reject_baseline: u32,
    pub(crate) verify_key_was_sent: bool,
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
            node_desc_budget: TCLK_MESSAGE_ATTEMPTS,
            request_key_budget: TCLK_MESSAGE_ATTEMPTS,
            verify_key_budget: TCLK_MESSAGE_ATTEMPTS,
            armed_at_us: now,
            stage_started_us: now,
            node_desc_slot: None,
            confirm_success_baseline: 0,
            confirm_reject_baseline: 0,
            verify_key_was_sent: false,
        }
    }

    /// Capture the APS security-handshake counters this exchange is measured
    /// against.
    ///
    /// Those counters are layer-wide and survive a previous exchange, so every
    /// "did something new happen?" test in this state machine is a comparison
    /// against the value observed when *this* exchange was armed.
    pub(crate) fn baseline_handshake_counters(
        &mut self,
        stats: &zigbee_aps::ApsSecurityHandshakeStats,
    ) {
        self.confirm_success_baseline = stats.confirm_key_successes;
        self.confirm_reject_baseline = stats.confirm_key_rejections;
    }

    /// Whether the post-announce start delay has elapsed.
    pub(crate) fn start_delay_elapsed(&self, now: u32) -> bool {
        now.wrapping_sub(self.armed_at_us) >= TCLK_EXCHANGE_START_DELAY_US
    }

    /// Whether the overall handshake deadline has expired (wrapping-safe).
    pub(crate) fn deadline_expired(&self, now: u32) -> bool {
        now.wrapping_sub(self.armed_at_us) >= TCLK_EXCHANGE_DEADLINE_US
    }

    /// Whether the current stage has exhausted its timing window
    /// (wrapping-safe).
    pub(crate) fn stage_timed_out(&self, now: u32) -> bool {
        now.wrapping_sub(self.stage_started_us) >= self.stage.window_us()
    }

    /// Enter `stage` and start its response window at `now`.
    pub(crate) fn enter(&mut self, stage: TclkStage, now: u32) {
        self.stage = stage;
        self.stage_started_us = now;
    }

    /// Consume one Node_Desc_req transmission. `false` when exhausted.
    pub(crate) fn take_node_desc_attempt(&mut self) -> bool {
        Self::take(&mut self.node_desc_budget)
    }

    /// Whether another Node_Desc_req transmission is still allowed.
    pub(crate) fn has_node_desc_attempt(&self) -> bool {
        self.node_desc_budget > 0
    }

    /// Consume one APS Request-Key transmission. `false` when exhausted.
    pub(crate) fn take_request_key_attempt(&mut self) -> bool {
        Self::take(&mut self.request_key_budget)
    }

    /// Consume one APS Verify-Key transmission. `false` when exhausted.
    pub(crate) fn take_verify_key_attempt(&mut self) -> bool {
        Self::take(&mut self.verify_key_budget)
    }

    /// Whether another Verify-Key transmission is still allowed.
    pub(crate) fn has_verify_key_attempt(&self) -> bool {
        self.verify_key_budget > 0
    }

    /// Whether at least one Verify-Key transmission succeeded for this key.
    pub(crate) fn verify_key_was_sent(&self) -> bool {
        self.verify_key_was_sent
    }

    /// Record a successful Verify-Key transmission.
    pub(crate) fn mark_verify_key_sent(&mut self) {
        self.verify_key_was_sent = true;
    }

    /// Whether another Request-Key transmission is still allowed.
    pub(crate) fn has_request_key_attempt(&self) -> bool {
        self.request_key_budget > 0
    }

    /// Restore the Verify-Key budget when a *new* key establishment starts.
    ///
    /// GSDK applies `maxAttempts` per message type and resets it when the next
    /// message type begins, so a fresh Request-Key gets a fresh Verify-Key
    /// budget for the replacement key.
    pub(crate) fn reset_verify_key_budget(&mut self) {
        self.verify_key_budget = TCLK_MESSAGE_ATTEMPTS;
        self.verify_key_was_sent = false;
    }

    fn take(budget: &mut u8) -> bool {
        if *budget == 0 {
            return false;
        }
        *budget -= 1;
        true
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
    fn new_arms_in_start_delay_with_a_full_budget_per_message_type() {
        let ex = armed(1_000);
        assert_eq!(ex.stage, TclkStage::StartDelay);
        assert_eq!(ex.node_desc_budget, TCLK_MESSAGE_ATTEMPTS);
        assert_eq!(ex.request_key_budget, TCLK_MESSAGE_ATTEMPTS);
        assert_eq!(ex.verify_key_budget, TCLK_MESSAGE_ATTEMPTS);
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
    fn each_stage_enforces_its_own_response_window() {
        let mut ex = armed(0);
        ex.enter(TclkStage::AwaitNodeDesc, 10_000);
        assert!(!ex.stage_timed_out(10_000 + TCLK_NODE_DESC_TIMEOUT_US - 1));
        assert!(ex.stage_timed_out(10_000 + TCLK_NODE_DESC_TIMEOUT_US));

        ex.enter(TclkStage::AwaitTclk, 10_000);
        assert!(!ex.stage_timed_out(10_000 + TCLK_NODE_DESC_TIMEOUT_US));
        assert!(ex.stage_timed_out(10_000 + TCLK_REQUEST_KEY_TIMEOUT_US));

        ex.enter(TclkStage::AwaitConfirmKey, 10_000);
        assert!(!ex.stage_timed_out(10_000 + TCLK_REQUEST_KEY_TIMEOUT_US));
        assert!(ex.stage_timed_out(10_000 + TCLK_VERIFY_KEY_TIMEOUT_US));
    }

    #[test]
    fn stage_and_deadline_timeouts_use_wrapping_arithmetic() {
        // Arm near the u32 wraparound boundary.
        let start = u32::MAX - 100;
        let mut ex = armed(start);
        ex.enter(TclkStage::AwaitConfirmKey, start);
        assert!(!ex.stage_timed_out(start.wrapping_add(TCLK_VERIFY_KEY_TIMEOUT_US - 1)));
        assert!(ex.stage_timed_out(start.wrapping_add(TCLK_VERIFY_KEY_TIMEOUT_US)));
        assert!(!ex.deadline_expired(start.wrapping_add(TCLK_EXCHANGE_DEADLINE_US - 1)));
        assert!(ex.deadline_expired(start.wrapping_add(TCLK_EXCHANGE_DEADLINE_US)));
    }

    #[test]
    fn retry_backoff_is_short_bounded_and_wrapping_safe() {
        let mut ex = armed(0);
        let start = u32::MAX - 100;
        ex.enter(TclkStage::RetryVerifyKey, start);
        assert!(!ex.stage_timed_out(start.wrapping_add(TCLK_RETRY_BACKOFF_US - 1)));
        assert!(ex.stage_timed_out(start.wrapping_add(TCLK_RETRY_BACKOFF_US)));
        const { assert!(TCLK_RETRY_BACKOFF_US < TCLK_VERIFY_KEY_TIMEOUT_US) };
    }

    #[test]
    fn message_budgets_are_independent_and_saturate() {
        let mut ex = armed(0);
        for _ in 0..TCLK_MESSAGE_ATTEMPTS {
            assert!(ex.take_verify_key_attempt());
        }
        assert!(!ex.take_verify_key_attempt());
        assert_eq!(ex.verify_key_budget, 0);

        // Retrying Verify-Key must not have consumed the other budgets.
        assert_eq!(ex.node_desc_budget, TCLK_MESSAGE_ATTEMPTS);
        assert!(ex.has_request_key_attempt());
        assert!(ex.take_request_key_attempt());
        assert_eq!(ex.request_key_budget, TCLK_MESSAGE_ATTEMPTS - 1);

        // A fresh key establishment restores the Verify-Key budget only.
        ex.reset_verify_key_budget();
        assert_eq!(ex.verify_key_budget, TCLK_MESSAGE_ATTEMPTS);
        assert_eq!(ex.request_key_budget, TCLK_MESSAGE_ATTEMPTS - 1);
    }

    #[test]
    fn one_full_pass_of_every_stage_fits_inside_the_overall_deadline() {
        // A Trust Center that answers at the edge of every window must still
        // be able to finish the first authentication pass, even if every
        // message type needs both permitted inter-attempt backoffs first.
        const { assert!(TCLK_FIRST_PASS_BUDGET_US < TCLK_EXCHANGE_DEADLINE_US) };
        const {
            assert!(
                TCLK_FIRST_PASS_BUDGET_US + TCLK_MAX_RETRY_BACKOFF_BUDGET_US
                    < TCLK_EXCHANGE_DEADLINE_US
            )
        };
        assert_eq!(TCLK_FIRST_PASS_BUDGET_US, 9_800_000);
        assert_eq!(TCLK_MAX_RETRY_BACKOFF_BUDGET_US, 1_500_000);
        assert_eq!(TCLK_EXCHANGE_DEADLINE_US, 15_000_000);
    }
}
