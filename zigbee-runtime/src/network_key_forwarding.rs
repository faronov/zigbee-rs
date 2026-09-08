//! R22 parent Network-Key update distribution.
//!
//! The security snapshot's explicit zero-destination bit is the write-ahead
//! intent: a merely staged or router-addressed key never implies fanout.
//! Child membership is recovered from the child journal.
//! Progress needs no extra flash writes: after reset we conservatively repeat
//! the bounded window with fresh reserved NWK counters. A retained previous
//! key is never selected for transmission.

use zigbee_mac::MacDriver;
use zigbee_nwk::IndirectFrameKind;
use zigbee_types::ShortAddress;

use crate::role::ParentRole;
use crate::security_store::{PersistentSecurityState, SecurityStateStore, SecurityStoreError};
use crate::{TrustCenterMode, ZigbeeDevice};

const _: () = assert!(zigbee_nwk::neighbor::MAX_NEIGHBORS <= u32::BITS as usize);

/// Runtime policy default, matching the existing eight-second indirect queue
/// lifetime. This is NOT an R22 numeric recommendation: the recommended maximum
/// polling interval comes from the application profile. Products must configure
/// their profile's interval before restoring/arming forwarding.
pub const DEFAULT_NETWORK_KEY_MAX_POLL_INTERVAL_US: u32 = 8_000_000;

/// Default-policy fanout window (twice the default maximum poll interval).
///
/// Not a mandatory Trust Center pre-switch delay. A TC may choose an independent
/// propagation grace; first-switch grace is a strategy, not this R22 obligation.
/// Use [`ZigbeeDevice::network_key_forwarding_window_us`] for configured policy.
pub const NETWORK_KEY_DISTRIBUTION_WINDOW_US: u32 = 2 * DEFAULT_NETWORK_KEY_MAX_POLL_INTERVAL_US;

pub(crate) struct ParentNetworkKeyForwarding {
    key: [u8; 16],
    started_at: u32,
    window_us: u32,
    child_fingerprint: u32,
    delivered: u32,
    sequence: u8,
    cursor: u8,
    armed: bool,
    expired: bool,
}

impl ParentNetworkKeyForwarding {
    pub(crate) const fn new() -> Self {
        Self {
            key: [0; 16],
            started_at: 0,
            window_us: NETWORK_KEY_DISTRIBUTION_WINDOW_US,
            child_fingerprint: 0,
            delivered: 0,
            sequence: 0,
            cursor: 0,
            armed: false,
            expired: false,
        }
    }
}

impl<M: MacDriver, R: ParentRole> ZigbeeDevice<M, R> {
    /// Configure the profile's recommended maximum poll interval before
    /// restore/arming. Reapply the same product policy on every boot.
    ///
    /// Zero and intervals whose doubled duration reaches the wrapping clock's
    /// half range are rejected. Changing an already armed window is rejected.
    pub fn set_network_key_forwarding_max_poll_interval_us(
        &mut self,
        max_poll_us: u32,
    ) -> Result<(), SecurityStoreError> {
        let state = &mut R::parent_state_mut(&mut self.role_state).network_key_forwarding;
        if state.armed || max_poll_us == 0 || max_poll_us > (i32::MAX as u32) / 2 {
            return Err(SecurityStoreError::Corrupt);
        }
        state.window_us = 2 * max_poll_us;
        Ok(())
    }

    /// Twice this parent's configured profile maximum polling interval.
    pub fn network_key_forwarding_window_us(&self) -> u32 {
        R::parent_state(&self.role_state)
            .network_key_forwarding
            .window_us
    }

    /// Checkpoint the staged update and initiate this parent's Rx-off fanout.
    ///
    /// The TC calls this for an originating all-zero-destination descriptor,
    /// normally broadcast distribution. Router-addressed unicast distribution
    /// does NOT itself imply fanout. This call does not need to send a
    /// Transport-Key command to itself. An already-checkpointed key is safe:
    /// snapshot already carrying intent also arms without another write.
    /// Receiving parents and restored parents arm the same state automatically
    /// after snapshot commit/restore.
    /// Returns false if there is no staged key. This does not transmit or
    /// claim delivery; [`Self::service_parent_commands`] performs bounded work.
    /// Repeated calls for the same key do not restart its forwarding window.
    pub fn begin_network_key_forwarding<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, SecurityStoreError> {
        if self.trust_center_mode() != TrustCenterMode::Centralized {
            return Err(SecurityStoreError::Corrupt);
        }
        let Some(sequence) = self
            .bdb
            .zdo()
            .nwk()
            .security()
            .staged_key()
            .map(|key| key.seq_number)
        else {
            return Ok(false);
        };
        let state = store.load()?.ok_or(SecurityStoreError::NotFound)?;
        if !state.commissioned {
            return Err(SecurityStoreError::NotFound);
        }
        self.bdb
            .zdo_mut()
            .aps_mut()
            .request_network_key_forwarding(sequence)
            .map_err(|_| SecurityStoreError::Corrupt)?;
        self.refresh_security_state(store)?;
        Ok(R::parent_state(&self.role_state)
            .network_key_forwarding
            .armed)
    }

    /// Whether the full distribution window for this committed key elapsed.
    ///
    /// This is only the *local fanout* deadline, starting at snapshot commit;
    /// it is not evidence of delivery to remote children or a mandatory
    /// pre-switch grace. Reset conservatively restarts the configured window.
    pub fn network_key_distribution_window_complete(&self, sequence: u8) -> bool {
        let state = &R::parent_state(&self.role_state).network_key_forwarding;
        state.armed
            && state.sequence == sequence
            && (state.expired
                || self
                    .bdb
                    .zdo()
                    .nwk()
                    .mac()
                    .monotonic_micros()
                    .wrapping_sub(state.started_at)
                    >= state.window_us)
    }

    pub(crate) fn record_parent_network_key_snapshot(
        &mut self,
        snapshot: &PersistentSecurityState,
    ) {
        let target = if snapshot.commissioned
            && snapshot.staged_network_key_present
            && snapshot.network_key_forwarding_pending
            && self.trust_center_mode() == TrustCenterMode::Centralized
        {
            if snapshot.secondary_network_key_is_previous {
                Some((snapshot.network_key, snapshot.key_sequence))
            } else {
                Some((snapshot.staged_network_key, snapshot.staged_key_sequence))
            }
        } else {
            None
        };
        let current = &R::parent_state(&self.role_state).network_key_forwarding;
        if target.is_some_and(|(key, sequence)| {
            current.armed && current.key == key && current.sequence == sequence
        }) {
            return;
        }
        self.cancel_network_key_forwarding();
        let now = self.bdb.zdo().nwk().mac().monotonic_micros();
        let state = &mut R::parent_state_mut(&mut self.role_state).network_key_forwarding;
        let window_us = state.window_us;
        *state = ParentNetworkKeyForwarding::new();
        state.window_us = window_us;
        if let Some((key, sequence)) = target {
            state.key = key;
            state.sequence = sequence;
            state.started_at = now;
            state.armed = true;
        }
    }

    fn cancel_network_key_forwarding(&mut self) {
        let state = &R::parent_state(&self.role_state).network_key_forwarding;
        if !state.armed {
            return;
        }
        let kind = IndirectFrameKind::NetworkKeyUpdate(state.sequence);
        // Queue membership, not current neighbors: eviction/address changes
        // must not strand an obsolete key transport.
        loop {
            let child = self
                .bdb
                .zdo()
                .nwk()
                .indirect_queue()
                .pending_children()
                .find(|child| self.bdb.zdo().nwk().has_pending_indirect_kind(*child, kind));
            let Some(child) = child else { break };
            if self
                .bdb
                .zdo_mut()
                .nwk_mut()
                .cancel_pending_indirect_kind(child, kind)
                .is_err()
            {
                // NWK removes the frame before clearing Frame Pending.
                log::warn!("[Runtime] Network-Key pending-bit clear failed");
            }
        }
    }

    pub(crate) fn complete_child_network_key_forwarding(
        &mut self,
        child: ShortAddress,
        sequence: u8,
    ) {
        self.refresh_forwarding_child_order();
        let index = self
            .bdb
            .zdo()
            .nwk()
            .neighbor_table()
            .children()
            .position(|entry| entry.network_address == child);
        let state = &mut R::parent_state_mut(&mut self.role_state).network_key_forwarding;
        if state.armed
            && state.sequence == sequence
            && let Some(index) = index
        {
            state.delivered |= 1u32 << index;
        }
    }

    /// Retire expired/superseded work before any queued MAC data poll.
    pub(crate) fn prune_network_key_forwarding(&mut self) -> bool {
        let state = &R::parent_state(&self.role_state).network_key_forwarding;
        if !state.armed {
            return false;
        }
        let sequence = state.sequence;
        let security = self.bdb.zdo().nwk().security();
        let current = security.staged_key().or_else(|| security.active_key());
        let expired = self.network_key_distribution_window_complete(sequence);
        if !current.is_some_and(|entry| entry.seq_number == sequence && entry.key == state.key)
            || expired
            || self
                .bdb
                .zdo()
                .aps()
                .network_key_forwarding_intent()
                .is_some()
        {
            // In particular, an uncommitted superseding key cannot cause
            // further transmission of a previously queued update.
            self.cancel_network_key_forwarding();
            R::parent_state_mut(&mut self.role_state)
                .network_key_forwarding
                .expired |= expired;
            return false;
        }
        true
    }

    fn refresh_forwarding_child_order(&mut self) {
        // Delivery bits index child iteration order, unlike the order-independent
        // durable child-table fingerprint. A changed order must clear the bits.
        let mut fingerprint = 0x811C_9DC5u32;
        for child in self.bdb.zdo().nwk().neighbor_table().children() {
            for byte in child
                .ieee_address
                .into_iter()
                .chain(child.network_address.0.to_le_bytes())
                .chain([u8::from(child.rx_on_when_idle)])
            {
                fingerprint = (fingerprint ^ u32::from(byte)).wrapping_mul(0x0100_0193);
            }
        }
        let state = &mut R::parent_state_mut(&mut self.role_state).network_key_forwarding;
        if state.child_fingerprint != fingerprint {
            state.child_fingerprint = fingerprint;
            state.delivered = 0;
        }
    }

    pub(crate) async fn service_network_key_forwarding(&mut self) {
        if !self.prune_network_key_forwarding() {
            return;
        }
        self.refresh_forwarding_child_order();
        let state = &R::parent_state(&self.role_state).network_key_forwarding;
        let sequence = state.sequence;
        let delivered = state.delivered;
        let cursor = usize::from(state.cursor);
        let kind = IndirectFrameKind::NetworkKeyUpdate(sequence);
        // One queue attempt per call bounds future size and radio work. A
        // full queue or expired indirect transaction is retried on a later
        // service call, never mistaken for successful delivery.
        let child = self
            .bdb
            .zdo()
            .nwk()
            .neighbor_table()
            .children()
            .enumerate()
            .filter(|(index, entry)| {
                !entry.rx_on_when_idle
                    && delivered & (1u32 << index) == 0
                    && !self
                        .bdb
                        .zdo()
                        .nwk()
                        .has_pending_indirect_kind(entry.network_address, kind)
            })
            .min_by_key(|(index, _)| (index + 32 - cursor) % 32)
            .map(|(index, entry)| (index, entry.network_address, entry.ieee_address));
        if let Some((index, short, ieee)) = child {
            R::parent_state_mut(&mut self.role_state)
                .network_key_forwarding
                .cursor = ((index + 1) % 32) as u8;
            if let Err(error) = self
                .bdb
                .zdo_mut()
                .aps_mut()
                .forward_network_key_update(short, &ieee, sequence)
                .await
            {
                log::warn!("[Runtime] Network-Key forwarding to {short:?} deferred: {error:?}");
            }
        }
    }
}
