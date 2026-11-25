//! NWK address map (`nwkAddressMap`, R22 Table 3-60 / §3.6.1.9).
//!
//! The address map caches 64-bit IEEE ↔ 16-bit network address pairs for
//! devices *anywhere* in the network — learned from authenticated
//! `Device_annce` frames and from the source IEEE address carried end to end
//! in relayed NWK headers.
//!
//! It is deliberately separate from the neighbor table (R22 §3.6.1.5), which
//! only ever describes devices this one hears directly. Knowing a device's
//! address pair says nothing about whether it is one hop away; routing on
//! such an entry would unicast straight to a multi-hop device, fail with
//! NO_ACK and tear down the route instead of discovering one.
//!
//! Entries are kept in least-recently-updated order; a full map evicts the
//! oldest pair.

use zigbee_types::{IeeeAddress, ShortAddress};

/// Capacity of the address map.
///
/// A router resolves IEEE-addressed traffic for the whole network. A build
/// without the `router` feature never routes on its neighbor table (every
/// unicast goes to the parent), so it keeps identity in its small neighbor
/// cache exactly as before and the address map has zero capacity — keeping
/// the sleepy end-device image lean.
#[cfg(feature = "router")]
pub const MAX_ADDRESS_MAP_ENTRIES: usize = crate::ncp_capacity_tier(32, 512, 256, 64);
#[cfg(not(feature = "router"))]
pub const MAX_ADDRESS_MAP_ENTRIES: usize = 0;

/// One IEEE ↔ short address pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddressMapEntry {
    pub network_address: ShortAddress,
    pub ieee_address: IeeeAddress,
}

/// The NWK address map.
#[derive(Debug, Default)]
pub struct AddressMap {
    #[cfg(feature = "router")]
    entries: heapless::Vec<AddressMapEntry, MAX_ADDRESS_MAP_ENTRIES>,
}

#[cfg(feature = "router")]
impl AddressMap {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    /// IEEE address recorded for `short`.
    pub fn ieee_of(&self, short: ShortAddress) -> Option<IeeeAddress> {
        self.entries
            .iter()
            .find(|entry| entry.network_address == short)
            .map(|entry| entry.ieee_address)
    }

    /// Network address recorded for `ieee`.
    pub fn short_of(&self, ieee: &IeeeAddress) -> Option<ShortAddress> {
        self.entries
            .iter()
            .find(|entry| entry.ieee_address == *ieee)
            .map(|entry| entry.network_address)
    }

    /// Record `short` ↔ `ieee` as the current pairing.
    ///
    /// Any older entry naming either address is replaced, so the two lookups
    /// can never disagree. A full map evicts its least-recently-updated pair.
    pub fn update(&mut self, short: ShortAddress, ieee: IeeeAddress) {
        self.entries
            .retain(|entry| entry.network_address != short && entry.ieee_address != ieee);
        if self.entries.is_full() {
            self.entries.remove(0);
        }
        let _ = self.entries.push(AddressMapEntry {
            network_address: short,
            ieee_address: ieee,
        });
    }

    /// Forget the pairing recorded for `short`.
    pub fn remove(&mut self, short: ShortAddress) {
        self.entries.retain(|entry| entry.network_address != short);
    }

    /// Forget every pairing.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// All recorded pairs, least recently updated first.
    pub fn iter(&self) -> impl Iterator<Item = &AddressMapEntry> {
        self.entries.iter()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Without the `router` feature the map has no storage and every operation
/// is a no-op, so nothing of it is linked into an end-device image.
#[cfg(not(feature = "router"))]
impl AddressMap {
    pub const fn new() -> Self {
        Self {}
    }

    #[inline(always)]
    pub fn ieee_of(&self, _short: ShortAddress) -> Option<IeeeAddress> {
        None
    }

    #[inline(always)]
    pub fn short_of(&self, _ieee: &IeeeAddress) -> Option<ShortAddress> {
        None
    }

    #[inline(always)]
    pub fn update(&mut self, _short: ShortAddress, _ieee: IeeeAddress) {}

    #[inline(always)]
    pub fn remove(&mut self, _short: ShortAddress) {}

    #[inline(always)]
    pub fn clear(&mut self) {}

    pub fn iter(&self) -> impl Iterator<Item = &AddressMapEntry> {
        core::iter::empty()
    }

    pub fn len(&self) -> usize {
        0
    }

    pub fn is_empty(&self) -> bool {
        true
    }
}

#[cfg(all(test, feature = "router"))]
mod tests {
    use super::*;

    #[test]
    fn a_pairing_replaces_older_entries_for_either_address() {
        let mut map = AddressMap::new();
        map.update(ShortAddress(1), [1; 8]);
        map.update(ShortAddress(2), [2; 8]);
        // Device 1 moved to address 3; address 2 was reused by device 4.
        map.update(ShortAddress(3), [1; 8]);
        map.update(ShortAddress(2), [4; 8]);
        assert_eq!(map.len(), 2);
        assert_eq!(map.short_of(&[1; 8]), Some(ShortAddress(3)));
        assert_eq!(map.ieee_of(ShortAddress(1)), None);
        assert_eq!(map.ieee_of(ShortAddress(2)), Some([4; 8]));
        assert_eq!(map.short_of(&[2; 8]), None);
        map.remove(ShortAddress(3));
        assert_eq!(map.short_of(&[1; 8]), None);
    }

    #[test]
    fn a_full_map_evicts_the_least_recently_updated_pair() {
        let mut map = AddressMap::new();
        // Distinct IEEE per entry even when the map holds more than 255.
        let ieee = |i: u16| {
            let [lo, hi] = (i + 1).to_le_bytes();
            [lo, hi, 1, 1, 1, 1, 1, 1]
        };
        for i in 0..MAX_ADDRESS_MAP_ENTRIES as u16 {
            map.update(ShortAddress(i + 1), ieee(i));
        }
        assert_eq!(map.len(), MAX_ADDRESS_MAP_ENTRIES);
        // Refresh the oldest pair so the second one becomes the oldest.
        map.update(ShortAddress(1), ieee(0));
        map.update(ShortAddress(0x7777), [0x77; 8]);
        assert_eq!(map.len(), MAX_ADDRESS_MAP_ENTRIES);
        assert_eq!(map.ieee_of(ShortAddress(1)), Some(ieee(0)));
        assert_eq!(map.ieee_of(ShortAddress(2)), None);
        assert_eq!(map.ieee_of(ShortAddress(0x7777)), Some([0x77; 8]));
    }
}
