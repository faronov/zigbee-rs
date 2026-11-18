//! The single reserved outgoing APS frame counter (`global-aps-counter`,
//! ZRHP decision 2, spec 8.4).
//!
//! With the feature, every APS-secured frame takes its counter from one
//! reservation that the owner (the coordinator's Trust Center runtime)
//! installs after a durable commit: [`ApsSecurity::next_frame_counter`]
//! still requires the key-pair entry to exist, but the entry's own counter
//! is unused, and the preconfigured global keys stop sharing the NWK
//! counter. Until a reservation is installed, at its limit, and after
//! [`ApsSecurity::clear_keys`], every APS-secured send is refused.

use super::{ApsKeyType, ApsSecurity};
use zigbee_types::IeeeAddress;

/// The reserved global outgoing APS frame counter.
///
/// `current` is the next value to use; `limit` is the exclusive, durably
/// reserved upper bound. Values at or beyond `limit` are never used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GlobalApsCounter {
    pub current: u32,
    pub limit: u32,
}

impl ApsSecurity {
    /// Install (or advance) the global outgoing counter reservation.
    ///
    /// Only moves forward: `current` and `limit` may not fall below the
    /// installed values, and `current <= limit`. Returns `false` (nothing
    /// changed) otherwise.
    pub fn set_global_outgoing_counter(&mut self, current: u32, limit: u32) -> bool {
        if current > limit {
            return false;
        }
        if let Some(installed) = self.global_outgoing
            && (current < installed.current || limit < installed.limit)
        {
            return false;
        }
        self.global_outgoing = Some(GlobalApsCounter { current, limit });
        true
    }

    /// The installed global outgoing counter, if any.
    pub const fn global_outgoing_counter(&self) -> Option<GlobalApsCounter> {
        self.global_outgoing
    }

    /// Drop the reservation: APS-secured sends are refused until a new one
    /// is installed (network left, Trust Center state cleared).
    pub fn clear_global_outgoing_counter(&mut self) {
        self.global_outgoing = None;
    }

    /// Next value of the global outgoing counter; `None` (send refused) when
    /// none is installed or the reservation is exhausted.
    pub fn next_global_frame_counter(&mut self) -> Option<u32> {
        let Some(counter) = self.global_outgoing.as_mut() else {
            log::error!("[APS] No global outgoing counter reservation");
            return None;
        };
        if counter.current >= counter.limit {
            log::error!("[APS] Global frame counter reservation exhausted");
            return None;
        }
        let value = counter.current;
        counter.current += 1;
        Some(value)
    }

    /// Outgoing counter for a key-pair entry: the entry must exist; the value
    /// comes from the global counter.
    pub fn next_frame_counter(
        &mut self,
        partner: &IeeeAddress,
        key_type: ApsKeyType,
    ) -> Option<u32> {
        self.find_key(partner, key_type)?;
        self.next_global_frame_counter()
    }

    /// Remove all negotiated link keys while retaining the configured default
    /// Trust Center key, and drop the global reservation. Key material is
    /// wiped before the slots are released.
    pub fn clear_keys(&mut self) {
        for entry in self.key_table.iter_mut() {
            zigbee_crypto::zeroize_key(&mut entry.key);
        }
        self.key_table.clear();
        self.global_replay_table.clear();
        self.stale_incoming_floors.clear();
        self.global_outgoing = None;
    }
}

impl<M: zigbee_mac::MacDriver> crate::ApsLayer<M> {
    /// With `global-aps-counter` the preconfigured global keys take the one
    /// reserved APS counter instead of sharing the NWK counter.
    pub(crate) fn next_default_tc_link_key_frame_counter(&mut self) -> Option<u32> {
        self.security.next_global_frame_counter()
    }
}
