//! R22 conflict management: address conflicts (§3.6.1.9) and PAN identifier
//! conflicts (§3.6.1.13).
//!
//! Two different identifiers can collide on a Zigbee network:
//!
//! * two devices can end up holding the same 16-bit `nwkNetworkAddress`,
//!   because stochastic address assignment does not coordinate;
//! * two *networks* can end up using the same 16-bit PAN identifier, because
//!   it is not globally unique.
//!
//! Both are resolved by NWK commands rather than by the application: an
//! address conflict by a Network Status command with status code `0x0D`, a PAN
//! identifier conflict by a Network Report (0x09) to the network manager and a
//! Network Update (0x0A) from it.
//!
//! # Trust
//!
//! Every conflict signal implemented here mutates network-defining state — the
//! device's own short address, or the PAN identifier it operates on. On a
//! secured network this module therefore only ever acts on frames that already
//! passed NWK CCM* authentication; the receive path applies that gate before
//! calling in. An unauthenticated "conflict" would otherwise be a one-frame
//! denial of service against any device in radio range.
//!
//! # Roles
//!
//! Detection of a conflict on the *local* address, and the resulting rejoin,
//! apply to every role including a sleepy end device. Everything that requires
//! forwarding — broadcasting a conflict on another device's behalf, reassigning
//! a child's address, receiving Network Report / Network Update — is a router
//! and coordinator behavior and is gated on [`NwkLayer::can_route`], so a
//! non-routing build neither links nor performs it.

use crate::frames::{PanIdConflictReport, PanIdUpdate};
use crate::nlde::{NwkCommandOutcome, is_unicast_address};
use crate::nlme::nwk_update_id_is_newer;
use crate::{DeviceType, NwkLayer, NwkStatus, PendingAddressConflict, PendingPanIdUpdate};
use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, PanId, ShortAddress};

/// `nwkcMaxBroadcastJitter` — R22 Table 3-57 gives `0x7D0` octet durations,
/// which is 64 ms at the 2.4 GHz octet duration of 32 µs.
pub const MAX_BROADCAST_JITTER_US: u32 = 64_000;

/// `nwkNetworkBroadcastDeliveryTime` in microseconds.
///
/// R22 Table 3-58 derives it from `nwkPassiveAckTimeout` and
/// `nwkMaxBroadcastRetries`; the NIB default in this stack is 9 s, which is the
/// value a broadcast is allowed to take to reach the whole network. A PAN
/// identifier update may only take effect after that time so the broadcast that
/// announced it can still travel on the old PAN identifier (R22 §3.6.1.13.3).
pub const NETWORK_BROADCAST_DELIVERY_TIME_US: u32 = 9_000_000;

/// The IEEE address that means "unknown" in an address map entry.
const NULL_IEEE: IeeeAddress = [0u8; 8];
/// The IEEE address a device uses when it has to name an unknown destination.
const BROADCAST_IEEE: IeeeAddress = [0xFFu8; 8];

/// The result of checking a statement of identity against the address map.
///
/// A conflicting statement must not be recorded — the mapping this device
/// already holds stays until the conflict is resolved — so the caller needs to
/// tell "nothing to do" from "conflict handled here" even when the conflict
/// produced no work for the layer above.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AddressCheck {
    /// The pairing agrees with the address map, or was recorded into it.
    Consistent,
    /// The pairing conflicts with a mapping this device already holds.
    Conflict {
        /// Lifecycle work this conflict creates for the layer above.
        outcome: Option<NwkCommandOutcome>,
    },
}

/// What resolving a conflict on this device's own address requires.
///
/// R22 §3.6.1.9.3: an end device — or any device whose address was assigned by
/// its parent rather than picked stochastically — rejoins to obtain a new
/// address; a stochastically addressed router picks a new one itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressConflictResolution {
    /// Rejoin the network to be given a new address.
    Rejoin,
    /// Pick a new address locally with
    /// [`NwkLayer::assign_new_local_address`].
    NewLocalAddress,
}

impl<M: MacDriver> NwkLayer<M> {
    /// Whether this device detects and corrects address conflicts.
    ///
    /// R22 §3.6.1.9: conflict detection is enabled when `nwkUniqueAddr` is
    /// FALSE, which is the case whenever addresses are assigned stochastically
    /// rather than by tree assignment.
    pub fn address_conflict_detection_enabled(&self) -> bool {
        self.nib.address_assign == crate::nib::AddressAssignMethod::Stochastic
    }

    /// Record an authenticated (short address, IEEE address) pairing and report
    /// any address conflict it reveals (R22 §3.6.1.9.1, §3.6.1.9.2).
    ///
    /// The pairing may come from the source IEEE address field of a NWK header,
    /// from a `Device_annce`, or from any other authenticated statement of
    /// identity. Three outcomes are possible:
    ///
    /// * the address map holds no entry, or a null IEEE address — the mapping
    ///   is recorded, no conflict;
    /// * the address map holds the same IEEE address — nothing to do;
    /// * the address map holds a *different* non-null IEEE address — a conflict
    ///   exists elsewhere in the network, and a router or coordinator informs
    ///   the network about it.
    ///
    /// A pairing that names this device's own address with somebody else's IEEE
    /// address is a conflict on the *local* address and is reported to the
    /// caller instead, because resolving it changes this device's own identity.
    #[inline(never)]
    pub fn note_address_information(
        &mut self,
        address: ShortAddress,
        ieee: IeeeAddress,
    ) -> AddressCheck {
        if !self.joined
            || !self.address_conflict_detection_enabled()
            || !is_unicast_address(address)
            || ieee == NULL_IEEE
            || ieee == BROADCAST_IEEE
        {
            return AddressCheck::Consistent;
        }

        if address == self.nib.network_address {
            if ieee == self.nib.ieee_address {
                return AddressCheck::Consistent;
            }
            let outcome = self.detect_local_address_conflict();
            return AddressCheck::Conflict {
                outcome: Some(outcome),
            };
        }
        // Somebody else's IEEE address may not masquerade under our own IEEE
        // address either: that is the same conflict seen from the other side,
        // and the offender is the *other* short address.
        if ieee == self.nib.ieee_address {
            return AddressCheck::Consistent;
        }
        if !cfg!(feature = "router") {
            // A non-routing build keeps no network-wide address map and never
            // announces another device's conflict — R22 §3.6.1.9.3 gives that
            // duty to a router or the coordinator. It still detects and
            // resolves a conflict on its *own* address, above, which is the
            // only resolution an end device can perform.
            return AddressCheck::Consistent;
        }

        let known = self
            .neighbors
            .find_by_short(address)
            .map(|entry| entry.ieee_address);
        match known {
            Some(NULL_IEEE) | None => {
                // Nothing recorded yet — learn the mapping. This is what makes
                // the *next* statement of identity able to detect a conflict.
                self.update_neighbor_address(address, ieee);
                AddressCheck::Consistent
            }
            Some(existing) if existing == ieee => AddressCheck::Consistent,
            Some(_) => {
                log::warn!("[NWK] Address conflict on 0x{:04X}", address.0);
                AddressCheck::Conflict {
                    outcome: self.report_foreign_address_conflict(address, ieee),
                }
            }
        }
    }

    /// Inform the network of a conflict on an address that is not our own
    /// (R22 §3.6.1.9.3), and reassign a conflicting child if we parent one.
    ///
    /// End devices never originate this broadcast: R22 gives the obligation to
    /// a "ZigBee coordinator or Router".
    fn report_foreign_address_conflict(
        &mut self,
        address: ShortAddress,
        ieee: IeeeAddress,
    ) -> Option<NwkCommandOutcome> {
        if !self.can_route() {
            return None;
        }
        self.queue_address_conflict_broadcast(address);

        // R22 §3.6.1.9.3: a parent that detects a conflict with the address of
        // one of its end device children picks a new address for that child and
        // tells it with an unsolicited rejoin response. The child keeps its own
        // IEEE address, so the entry that must move is the one whose IEEE
        // address is *not* the newly observed one.
        let child = self.neighbors.find_by_short(address).filter(|entry| {
            entry.ieee_address != ieee
                && entry.device_type == crate::neighbor::NeighborDeviceType::EndDevice
                && matches!(
                    entry.relationship,
                    crate::neighbor::Relationship::Child
                        | crate::neighbor::Relationship::UnauthenticatedChild
                )
        })?;
        Some(NwkCommandOutcome::ChildAddressConflict {
            child: address,
            ieee: child.ieee_address,
        })
    }

    /// Note a conflict on this device's own address and report how it must be
    /// resolved (R22 §3.6.1.9.3).
    ///
    /// A router or coordinator additionally informs the network, naming its
    /// *previous* address, unless it learned of the conflict from a Network
    /// Status command that already carries exactly that payload — R22 only asks
    /// for the broadcast when the conflict was learned some other way.
    pub(crate) fn detect_local_address_conflict(&mut self) -> NwkCommandOutcome {
        let previous = self.nib.network_address;
        log::warn!("[NWK] Local address conflict on 0x{:04X}", previous.0);
        if self.can_route() {
            self.queue_address_conflict_broadcast(previous);
        }
        NwkCommandOutcome::AddressConflict {
            previous,
            resolution: self.local_conflict_resolution(),
        }
    }

    /// R22 §3.6.1.9.3 resolution branch for a conflict on the local address.
    fn local_conflict_resolution(&self) -> AddressConflictResolution {
        if self.device_type == DeviceType::EndDevice
            || self.nib.address_assign != crate::nib::AddressAssignMethod::Stochastic
        {
            AddressConflictResolution::Rejoin
        } else if self.device_type == DeviceType::Coordinator {
            // The coordinator has no parent to rejoin through and 0x0000 is its
            // address by definition (R22 §3.6.1.7): the offending device is the
            // one that must move, which the broadcast above tells it to do.
            AddressConflictResolution::Rejoin
        } else {
            AddressConflictResolution::NewLocalAddress
        }
    }

    /// Queue an address-conflict broadcast after a random jitter bounded by
    /// `nwkcMaxBroadcastJitter` (R22 §3.6.1.9.3).
    ///
    /// A conflict is usually seen by several routers at once. The jitter — plus
    /// cancellation when an identical broadcast arrives first, handled in
    /// [`Self::handle_network_status_address_conflict`] — keeps that from
    /// turning into a broadcast storm.
    fn queue_address_conflict_broadcast(&mut self, address: ShortAddress) {
        if !self.can_route() || !is_unicast_address(address) {
            return;
        }
        if self
            .pending_conflicts
            .iter()
            .any(|pending| pending.address == address)
        {
            return;
        }
        let now = self.mac.monotonic_micros();
        let jitter = crate::routing::routing_random_sample(
            now ^ (u32::from(self.nib.network_address.0) << 16) ^ u32::from(address.0),
        ) % MAX_BROADCAST_JITTER_US;
        if self
            .pending_conflicts
            .push(PendingAddressConflict {
                address,
                send_after_us: now.wrapping_add(jitter),
            })
            .is_err()
        {
            log::warn!(
                "[NWK] Address conflict queue full; 0x{:04X} not announced",
                address.0
            );
        }
    }

    /// Handle a received Network Status command with status code `0x0D`
    /// (R22 §3.6.1.9.3).
    ///
    /// Three cases, in order:
    ///
    /// * the offending address is ours — we must change it, and we do *not*
    ///   rebroadcast, because the network has already been told;
    /// * the offending address belongs to an end device child of ours — we pick
    ///   a new address for it and tell it with an unsolicited rejoin response;
    /// * otherwise — somebody else already announced this exact conflict, so any
    ///   identical broadcast of our own is cancelled.
    pub(crate) fn handle_network_status_address_conflict(
        &mut self,
        offending: ShortAddress,
    ) -> Option<NwkCommandOutcome> {
        if !self.address_conflict_detection_enabled() || !is_unicast_address(offending) {
            return None;
        }

        if offending == self.nib.network_address {
            let previous = self.nib.network_address;
            log::warn!(
                "[NWK] Address conflict announced for our own address 0x{:04X}",
                previous.0
            );
            // Learned *from* a network status command: R22 asks for a broadcast
            // only when the conflict was learned another way, so nothing is
            // queued here.
            self.cancel_address_conflict_broadcast(offending);
            return Some(NwkCommandOutcome::AddressConflict {
                previous,
                resolution: self.local_conflict_resolution(),
            });
        }

        // Somebody else announced it — our own identical broadcast is redundant.
        self.cancel_address_conflict_broadcast(offending);

        if !self.can_route() {
            return None;
        }
        let child = self.neighbors.find_by_short(offending).filter(|entry| {
            entry.device_type == crate::neighbor::NeighborDeviceType::EndDevice
                && matches!(
                    entry.relationship,
                    crate::neighbor::Relationship::Child
                        | crate::neighbor::Relationship::UnauthenticatedChild
                )
        })?;
        Some(NwkCommandOutcome::ChildAddressConflict {
            child: offending,
            ieee: child.ieee_address,
        })
    }

    /// Drop a queued conflict broadcast for `address` (R22 §3.6.1.9.3 asks a
    /// device to cancel its own broadcast when an identical one arrives during
    /// the jitter window).
    fn cancel_address_conflict_broadcast(&mut self, address: ShortAddress) {
        if let Some(index) = self
            .pending_conflicts
            .iter()
            .position(|pending| pending.address == address)
        {
            self.pending_conflicts.swap_remove(index);
        }
    }

    /// Pick and apply a new short address after a local address conflict
    /// (R22 §3.6.1.9.3 stochastic branch).
    ///
    /// The replacement is chosen randomly while avoiding every address this
    /// device holds in NIB state — its own, its parent's, and every neighbor —
    /// as well as the reserved and broadcast ranges. The MAC PIB is retuned
    /// before the NIB so the radio can never be listening for the old address
    /// while the stack believes it owns the new one.
    ///
    /// The caller owns what comes next: persisting the new address and
    /// announcing it with a `Device_annce` (R22 §3.6.1.9.2 requires a
    /// `Device_annce` or a route discovery after an address change).
    pub async fn assign_new_local_address(&mut self) -> Result<ShortAddress, NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }
        let previous = self.nib.network_address;
        let mut candidate = previous;
        // Bounded search: a neighbor table can only hold so many addresses, so
        // a handful of draws is always enough to miss all of them.
        for attempt in 0..16u32 {
            let sample = crate::routing::routing_random_sample(
                self.mac
                    .monotonic_micros()
                    .wrapping_add(attempt.wrapping_mul(0x9E37_79B9))
                    ^ (u32::from(previous.0) << 16),
            );
            let proposed = ShortAddress((sample % 0xFFF7) as u16);
            if proposed == previous
                || proposed == ShortAddress::COORDINATOR
                || !is_unicast_address(proposed)
                || proposed == self.nib.parent_address
                || self.neighbors.find_by_short(proposed).is_some()
                || self.routing.next_hop(proposed).is_some()
            {
                continue;
            }
            candidate = proposed;
            break;
        }
        if candidate == previous {
            log::error!("[NWK] No free short address found for conflict resolution");
            return Err(NwkStatus::InvalidRequest);
        }

        self.mac
            .mlme_set(
                zigbee_mac::PibAttribute::MacShortAddress,
                zigbee_mac::PibValue::ShortAddress(candidate),
            )
            .await
            .map_err(|_| NwkStatus::InvalidRequest)?;
        self.nib.network_address = candidate;
        // The old address is no longer ours: drop routing state that named it
        // so a stale entry cannot send our traffic to the offending device.
        self.routing.remove(previous);
        self.neighbors.remove(previous);
        log::warn!(
            "[NWK] Short address changed 0x{:04X} -> 0x{:04X} after conflict",
            previous.0,
            candidate.0,
        );
        Ok(candidate)
    }

    // ── PAN identifier conflicts (R22 §3.6.1.13) ────────────────

    /// Whether a beacon reveals a PAN identifier conflict (R22 §3.6.1.13.1).
    ///
    /// A conflict exists when a beacon carries this network's short PAN
    /// identifier but a different — or absent — extended PAN identifier.
    pub fn beacon_reveals_pan_id_conflict(&self, pan_id: PanId, epid: Option<IeeeAddress>) -> bool {
        self.joined
            && pan_id == self.nib.pan_id
            && epid.is_none_or(|epid| epid != self.nib.extended_pan_id)
    }

    /// Handle a received Network Report command (R22 §3.6.1.13.2).
    ///
    /// Only the device named by `nwkManagerAddr` acts on it: it selects a
    /// replacement PAN identifier that appears neither in the report nor in its
    /// own neighborhood, increments `nwkUpdateId`, and broadcasts a Network
    /// Update. The switch itself is deferred by
    /// `nwkNetworkBroadcastDeliveryTime` for the manager as well, so that the
    /// update broadcast can still reach the network on the old PAN identifier.
    ///
    /// Returns the update to broadcast, which the async command path sends.
    pub(crate) fn handle_network_report(&mut self, payload: &[u8]) -> Option<PanIdUpdate> {
        if !self.can_route() {
            return None;
        }
        let Some(report) = PanIdConflictReport::parse(payload) else {
            log::warn!("[NWK] Malformed Network Report");
            return None;
        };
        if report.epid != self.nib.extended_pan_id {
            log::debug!("[NWK] Network Report for another network ignored");
            return None;
        }
        if self.nib.network_address != self.nib.nwk_manager_addr {
            // Not the network manager: R22 §3.4.9.1 routes the report to the
            // manager, and only the manager selects a new PAN identifier.
            log::debug!("[NWK] Network Report ignored: this device is not the network manager");
            return None;
        }
        if self.pending_pan_id_update.is_some() {
            log::debug!("[NWK] PAN ID update already in flight; report ignored");
            return None;
        }

        let new_pan_id = self.select_replacement_pan_id(&report.pan_ids)?;
        let update_id = self
            .nib
            .nwk_update_id()
            .unwrap_or(self.nib.update_id)
            .wrapping_add(1);
        self.nib.set_nwk_update_id(update_id);
        self.arm_pan_id_update(new_pan_id);
        log::warn!(
            "[NWK] PAN ID conflict reported; moving 0x{:04X} -> 0x{:04X} (update_id={})",
            self.nib.pan_id.0,
            new_pan_id.0,
            update_id,
        );
        Some(PanIdUpdate {
            epid: self.nib.extended_pan_id,
            update_id,
            new_pan_id,
        })
    }

    /// Choose a PAN identifier that is used neither by the reporter's
    /// neighborhood nor by ours (R22 §3.6.1.13.2).
    fn select_replacement_pan_id(&mut self, reported: &[PanId]) -> Option<PanId> {
        for attempt in 0..32u32 {
            let sample = crate::routing::routing_random_sample(
                self.mac
                    .monotonic_micros()
                    .wrapping_add(attempt.wrapping_mul(0x85EB_CA6B))
                    ^ (u32::from(self.nib.pan_id.0) << 8),
            );
            // 0xFFFF is the broadcast PAN identifier and 0x0000 is avoided so a
            // zeroed PIB can never look like a legitimate network.
            let candidate = PanId(1 + (sample % 0xFFFE) as u16);
            if candidate == self.nib.pan_id || reported.contains(&candidate) {
                continue;
            }
            return Some(candidate);
        }
        log::error!("[NWK] No replacement PAN ID found");
        None
    }

    /// Handle a received Network Update command (R22 §3.6.1.13.3).
    ///
    /// The update is accepted when it names this network and carries a strictly
    /// newer `nwkUpdateId` than the one held locally — the manager increments it
    /// before sending, so a repeat of an update already in flight is idempotent
    /// and an older one is a stale replay. The new PAN identifier is adopted
    /// only after `nwkNetworkBroadcastDeliveryTime`, so the broadcast that
    /// carries it can still cross the network on the old PAN identifier.
    pub(crate) fn handle_network_update(&mut self, src: ShortAddress, payload: &[u8]) {
        if !self.can_route() {
            return;
        }
        let Some(update) = PanIdUpdate::parse(payload) else {
            log::warn!("[NWK] Malformed Network Update from 0x{:04X}", src.0);
            return;
        };
        if update.epid != self.nib.extended_pan_id {
            log::debug!("[NWK] Network Update for another network ignored");
            return;
        }
        if update.new_pan_id == self.nib.pan_id {
            return;
        }
        if let Some(pending) = self.pending_pan_id_update
            && pending.new_pan_id == update.new_pan_id
        {
            // Retransmission of an update already being applied.
            return;
        }
        match self.nib.nwk_update_id() {
            Some(local) if !nwk_update_id_is_newer(update.update_id, local) => {
                log::warn!(
                    "[NWK] Stale Network Update from 0x{:04X}: update_id={} (local {})",
                    src.0,
                    update.update_id,
                    local,
                );
                return;
            }
            _ => {}
        }

        log::warn!(
            "[NWK] Network Update from 0x{:04X}: PAN 0x{:04X} -> 0x{:04X} (update_id={})",
            src.0,
            self.nib.pan_id.0,
            update.new_pan_id.0,
            update.update_id,
        );
        // R22 §3.6.1.13.3: the update id is stored on receipt; only the PAN
        // identifier itself waits for the delivery timer.
        self.nib.set_nwk_update_id(update.update_id);
        self.arm_pan_id_update(update.new_pan_id);
    }

    /// Arm the deferred PAN identifier switch.
    fn arm_pan_id_update(&mut self, new_pan_id: PanId) {
        let apply_at_us = self
            .mac
            .monotonic_micros()
            .wrapping_add(NETWORK_BROADCAST_DELIVERY_TIME_US);
        self.pending_pan_id_update = Some(PendingPanIdUpdate {
            new_pan_id,
            apply_at_us,
        });
    }

    /// The PAN identifier this device is about to move to, if any.
    pub fn pending_pan_id(&self) -> Option<PanId> {
        self.pending_pan_id_update.map(|pending| pending.new_pan_id)
    }

    /// Report a PAN identifier conflict to the network manager, or resolve it
    /// directly when this device *is* the manager (R22 §3.6.1.13.1).
    ///
    /// `observed` is the list of PAN identifiers seen in the neighborhood, which
    /// R22 recommends building from an active scan.
    pub async fn report_pan_id_conflict(&mut self, observed: &[PanId]) -> Result<(), NwkStatus> {
        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }
        if self.nib.network_address == self.nib.nwk_manager_addr {
            let mut report = PanIdConflictReport {
                epid: self.nib.extended_pan_id,
                pan_ids: heapless::Vec::new(),
            };
            for pan_id in observed {
                let _ = report.pan_ids.push(*pan_id);
            }
            let mut payload = [0u8; 9 + 2 * crate::frames::MAX_PAN_ID_CONFLICT_REPORT];
            let len = report
                .serialize(&mut payload)
                .ok_or(NwkStatus::FrameTooLong)?;
            let Some(update) = self.handle_network_report(&payload[..len]) else {
                return Err(NwkStatus::InvalidRequest);
            };
            return self.send_pan_id_update(update).await;
        }
        self.send_pan_id_conflict_report(observed).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DeviceType;
    use crate::frames::{
        LINK_STATUS_FIRST_FRAME, LINK_STATUS_LAST_FRAME, NetworkStatusCommand, NwkCommandId,
        NwkFrameControl, NwkFrameType, NwkHeader,
    };
    use crate::neighbor::{NeighborDeviceType, NeighborEntry, ROUTER_AGE_LIMIT, Relationship};
    use crate::nlde::NwkCommandOutcome;
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    use zigbee_types::PanId;

    const OUR_ADDR: ShortAddress = ShortAddress(0x1111);
    const OUR_IEEE: IeeeAddress = [0x11; 8];
    const PEER: ShortAddress = ShortAddress(0x2222);
    const PEER_IEEE: IeeeAddress = [0x22; 8];
    const IMPOSTOR_IEEE: IeeeAddress = [0x99; 8];
    const CHILD: ShortAddress = ShortAddress(0x3333);
    const CHILD_IEEE: IeeeAddress = [0x33; 8];
    const PAN: PanId = PanId(0x1234);
    const EPID: IeeeAddress = [0xE0; 8];

    struct NoopWake;
    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    fn node(device_type: DeviceType) -> NwkLayer<MockMac> {
        let mut nwk = NwkLayer::new(MockMac::new(OUR_IEEE), device_type);
        nwk.set_joined(true);
        let nib = nwk.nib_mut();
        nib.network_address = OUR_ADDR;
        nib.ieee_address = OUR_IEEE;
        nib.pan_id = PAN;
        nib.extended_pan_id = EPID;
        nib.nwk_manager_addr = ShortAddress::COORDINATOR;
        nib.security_enabled = false;
        nwk
    }

    fn router_neighbour(nwk: &mut NwkLayer<MockMac>, address: ShortAddress, ieee: IeeeAddress) {
        let mut entry = NeighborEntry::new_from_annce(address, ieee);
        entry.device_type = NeighborDeviceType::Router;
        entry.incoming_cost = 2;
        nwk.neighbors.add_or_update(entry).unwrap();
    }

    fn end_device_child(nwk: &mut NwkLayer<MockMac>, address: ShortAddress, ieee: IeeeAddress) {
        let mut entry = NeighborEntry::new_from_annce(address, ieee);
        entry.device_type = NeighborDeviceType::EndDevice;
        entry.relationship = Relationship::Child;
        nwk.neighbors.add_or_update(entry).unwrap();
    }

    /// Feed a NWK command frame in as if it arrived from `src`.
    fn deliver(nwk: &mut NwkLayer<MockMac>, src: ShortAddress, id: NwkCommandId, body: &[u8]) {
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: false,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            src_addr: src,
            radius: 1,
            seq_number: 9,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let mut buf = [0u8; 128];
        let header_len = header.serialize(&mut buf);
        buf[header_len] = id as u8;
        buf[header_len + 1..header_len + 1 + body.len()].copy_from_slice(body);
        let len = header_len + 1 + body.len();
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 200)).is_none());
    }

    fn address_conflict_body(offending: ShortAddress) -> [u8; 3] {
        [
            NetworkStatusCommand::ADDRESS_CONFLICT,
            (offending.0 & 0xFF) as u8,
            (offending.0 >> 8) as u8,
        ]
    }

    // ── R22 §3.6.3.4.2 Link Status receive processing ───────────

    #[test]
    #[cfg(feature = "router")]
    fn a_listed_receiver_takes_its_outgoing_cost_from_the_entry() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        let body = [
            LINK_STATUS_FIRST_FRAME | LINK_STATUS_LAST_FRAME | 1,
            (OUR_ADDR.0 & 0xFF) as u8,
            (OUR_ADDR.0 >> 8) as u8,
            0x04,
        ];
        deliver(&mut router, PEER, NwkCommandId::LinkStatus, &body);

        let neighbour = router.neighbors.find_by_short(PEER).unwrap();
        assert_eq!(neighbour.outgoing_cost, 4);
        assert_eq!(neighbour.link_status_age, 0);
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_unlisted_receiver_inside_the_range_loses_its_outgoing_cost() {
        // R22 §3.6.3.4.2: "If the receiver's address is not found, the outgoing
        // cost field is set to 0."
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        router
            .neighbors
            .find_by_short_mut(PEER)
            .unwrap()
            .outgoing_cost = 3;

        let body = [
            LINK_STATUS_FIRST_FRAME | LINK_STATUS_LAST_FRAME | 1,
            0x01,
            0x00,
            0x02,
        ];
        deliver(&mut router, PEER, NwkCommandId::LinkStatus, &body);
        assert_eq!(
            router.neighbors.find_by_short(PEER).unwrap().outgoing_cost,
            0
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_fragment_that_does_not_cover_us_leaves_the_cost_alone() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        router
            .neighbors
            .find_by_short_mut(PEER)
            .unwrap()
            .outgoing_cost = 3;
        router
            .neighbors
            .find_by_short_mut(PEER)
            .unwrap()
            .link_status_age = 2;

        // An intermediate fragment covering 0x0001..=0x0002 only. Our address
        // (0x1111) is outside it, so the frame says nothing about our link.
        let body = [2u8, 0x01, 0x00, 0x02, 0x02, 0x00, 0x02];
        deliver(&mut router, PEER, NwkCommandId::LinkStatus, &body);

        let neighbour = router.neighbors.find_by_short(PEER).unwrap();
        assert_eq!(neighbour.outgoing_cost, 3, "the cost is untouched");
        assert_eq!(
            neighbour.link_status_age, 0,
            "the frame itself is still proof the router is alive",
        );
    }

    #[test]
    fn an_end_device_ignores_link_status_entirely() {
        // R22 §3.6.3.4.2: "End devices do not process link status command
        // frames."
        let mut end_device = node(DeviceType::EndDevice);
        router_neighbour(&mut end_device, PEER, PEER_IEEE);
        let body = [
            LINK_STATUS_FIRST_FRAME | LINK_STATUS_LAST_FRAME | 1,
            (OUR_ADDR.0 & 0xFF) as u8,
            (OUR_ADDR.0 >> 8) as u8,
            0x04,
        ];
        deliver(&mut end_device, PEER, NwkCommandId::LinkStatus, &body);
        assert_eq!(
            end_device
                .neighbors
                .find_by_short(PEER)
                .unwrap()
                .outgoing_cost,
            0
        );
    }

    #[test]
    fn a_router_neighbour_goes_stale_after_the_router_age_limit() {
        // R22 §3.6.3.4.3: after nwkRouterAgeLimit missed link status periods
        // the outgoing cost is discarded.
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        router
            .neighbors
            .find_by_short_mut(PEER)
            .unwrap()
            .outgoing_cost = 3;
        end_device_child(&mut router, CHILD, CHILD_IEEE);
        router
            .neighbors
            .find_by_short_mut(CHILD)
            .unwrap()
            .outgoing_cost = 1;

        for _ in 0..ROUTER_AGE_LIMIT {
            router.tick_router_maintenance(crate::neighbor::LINK_STATUS_PERIOD_SECS);
            assert_eq!(
                router.neighbors.find_by_short(PEER).unwrap().outgoing_cost,
                3,
                "the cost survives until the limit is exceeded",
            );
        }
        router.tick_router_maintenance(crate::neighbor::LINK_STATUS_PERIOD_SECS);
        assert_eq!(
            router.neighbors.find_by_short(PEER).unwrap().outgoing_cost,
            0
        );
        assert_eq!(
            router.neighbors.find_by_short(CHILD).unwrap().outgoing_cost,
            1,
            "an end device never issues link status and is never aged this way",
        );
    }

    // ── R22 §3.6.1.9 address conflicts ──────────────────────────

    /// An unsecured unicast is still accepted on a secured network so pre-key
    /// APS commissioning traffic can arrive, and its NWK header is plaintext
    /// that any device in range can write. It must never be able to take a
    /// device off its address.
    #[test]
    fn an_unauthenticated_header_cannot_claim_an_address_conflict() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().security_enabled = true;
        router_neighbour(&mut router, PEER, PEER_IEEE);

        // A plaintext data frame addressed to our short address but to somebody
        // else's IEEE address — the strongest R22 §3.6.1.9.2 evidence there is,
        // forged by a device with no keys at all.
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: false,
                source_route: false,
                dst_ieee_present: true,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr: OUR_ADDR,
            src_addr: PEER,
            radius: 5,
            seq_number: 3,
            dst_ieee: Some(IMPOSTOR_IEEE),
            src_ieee: Some(IMPOSTOR_IEEE),
            multicast_control: None,
            source_route: None,
        };
        let mut buf = [0u8; 128];
        let header_len = header.serialize(&mut buf);
        buf[header_len] = 0xAA;
        let len = header_len + 1;

        let _ = block_on(router.process_incoming_nwk_frame(&buf[..len], 200));
        assert_eq!(
            router.take_command_outcome(),
            None,
            "an unauthenticated header is not evidence of a conflict",
        );
        assert_eq!(router.nib().network_address, OUR_ADDR);

        // The same is true of an announcement carried by that frame.
        router.note_announced_address(OUR_ADDR, IMPOSTOR_IEEE);
        assert_eq!(router.take_command_outcome(), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_network_status_naming_our_address_asks_a_router_to_pick_a_new_one() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        deliver(
            &mut router,
            PEER,
            NwkCommandId::NetworkStatus,
            &address_conflict_body(OUR_ADDR),
        );

        assert_eq!(
            router.take_command_outcome(),
            Some(NwkCommandOutcome::AddressConflict {
                previous: OUR_ADDR,
                resolution: AddressConflictResolution::NewLocalAddress,
            }),
        );
        // Learned *from* a network status command: R22 §3.6.1.9.3 only asks for
        // a broadcast when the conflict was learned another way.
        block_on(router.process_pending_routing());
        assert!(
            router.mac().tx_history().is_empty(),
            "the announcement is not echoed back at the network",
        );
    }

    #[test]
    fn an_end_device_resolves_its_own_conflict_by_rejoining() {
        let mut end_device = node(DeviceType::EndDevice);
        router_neighbour(&mut end_device, PEER, PEER_IEEE);
        deliver(
            &mut end_device,
            PEER,
            NwkCommandId::NetworkStatus,
            &address_conflict_body(OUR_ADDR),
        );
        assert_eq!(
            end_device.take_command_outcome(),
            Some(NwkCommandOutcome::AddressConflict {
                previous: OUR_ADDR,
                resolution: AddressConflictResolution::Rejoin,
            }),
        );
    }

    #[test]
    fn a_tree_addressed_device_rejoins_instead_of_picking_an_address() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().address_assign = crate::nib::AddressAssignMethod::TreeBased;
        // R22 §3.6.1.9: detection is only enabled when nwkUniqueAddr is FALSE,
        // which tree addressing turns off, so nothing is reported at all.
        assert_eq!(
            router.handle_network_status_address_conflict(OUR_ADDR),
            None,
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_announced_address_held_by_another_ieee_is_announced_as_a_conflict() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);

        // A Device_annce claiming PEER's address under a different identity.
        router.note_announced_address(PEER, IMPOSTOR_IEEE);
        assert_eq!(
            router.neighbors.find_by_short(PEER).unwrap().ieee_address,
            PEER_IEEE,
            "a conflicting announcement does not overwrite the known mapping",
        );

        // R22 §3.6.1.9.3 delays the broadcast by a jitter bounded by
        // nwkcMaxBroadcastJitter.
        block_on(router.process_pending_routing());
        assert!(
            router.mac().tx_history().is_empty(),
            "the jitter is honoured"
        );
        block_on(router.mac_mut().delay_micros(MAX_BROADCAST_JITTER_US));
        block_on(router.process_pending_routing());

        let record = &router.mac().tx_history()[0];
        let bytes = record.payload.as_slice();
        let (header, consumed) = NwkHeader::parse(bytes).expect("the frame parses");
        assert_eq!(
            header.dst_addr,
            ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            "R22 §3.6.1.9.3: broadcast to 0xFFFD",
        );
        assert_eq!(bytes[consumed], NwkCommandId::NetworkStatus as u8);
        assert_eq!(
            &bytes[consumed + 1..consumed + 4],
            &address_conflict_body(PEER)
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_identical_announcement_cancels_our_own_pending_broadcast() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        router_neighbour(&mut router, ShortAddress(0x4444), [0x44; 8]);
        router.note_announced_address(PEER, IMPOSTOR_IEEE);

        // Somebody else announced the same conflict first.
        deliver(
            &mut router,
            ShortAddress(0x4444),
            NwkCommandId::NetworkStatus,
            &address_conflict_body(PEER),
        );
        block_on(router.mac_mut().delay_micros(MAX_BROADCAST_JITTER_US));
        block_on(router.process_pending_routing());
        assert!(
            router.mac().tx_history().is_empty(),
            "R22 §3.6.1.9.3: an identical broadcast cancels ours",
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_parent_is_told_to_reassign_a_conflicting_end_device_child() {
        let mut parent = node(DeviceType::Router);
        router_neighbour(&mut parent, PEER, PEER_IEEE);
        end_device_child(&mut parent, CHILD, CHILD_IEEE);

        deliver(
            &mut parent,
            PEER,
            NwkCommandId::NetworkStatus,
            &address_conflict_body(CHILD),
        );
        assert_eq!(
            parent.take_command_outcome(),
            Some(NwkCommandOutcome::ChildAddressConflict {
                child: CHILD,
                ieee: CHILD_IEEE,
            }),
        );
    }

    #[test]
    fn a_new_local_address_avoids_every_address_the_nib_already_holds() {
        let mut router = node(DeviceType::Router);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        let assigned = block_on(router.assign_new_local_address()).expect("an address is free");

        assert_ne!(assigned, OUR_ADDR);
        assert_ne!(assigned, PEER);
        assert_ne!(assigned, ShortAddress::COORDINATOR);
        assert!(assigned.0 < 0xFFF8);
        assert_eq!(router.nib().network_address, assigned);
    }

    // ── R22 §3.6.1.13 PAN identifier conflicts ──────────────────

    fn pan_id_report(epid: IeeeAddress, pan_ids: &[PanId]) -> heapless::Vec<u8, 32> {
        let mut report = PanIdConflictReport {
            epid,
            pan_ids: heapless::Vec::new(),
        };
        for pan_id in pan_ids {
            report.pan_ids.push(*pan_id).unwrap();
        }
        let mut buf = [0u8; 32];
        let len = report.serialize(&mut buf).unwrap();
        heapless::Vec::from_slice(&buf[..len]).unwrap()
    }

    fn pan_id_update(epid: IeeeAddress, update_id: u8, new_pan_id: PanId) -> [u8; 12] {
        let mut buf = [0u8; 12];
        PanIdUpdate {
            epid,
            update_id,
            new_pan_id,
        }
        .serialize(&mut buf)
        .unwrap();
        buf
    }

    #[test]
    #[cfg(feature = "router")]
    fn the_network_manager_answers_a_report_with_a_fresh_pan_id_and_update_id() {
        let mut manager = node(DeviceType::Coordinator);
        manager.nib_mut().network_address = ShortAddress::COORDINATOR;
        manager.nib_mut().set_nwk_update_id(4);
        router_neighbour(&mut manager, PEER, PEER_IEEE);

        let report = pan_id_report(EPID, &[PAN, PanId(0x4321)]);
        deliver(&mut manager, PEER, NwkCommandId::NetworkReport, &report);
        block_on(manager.process_pending_routing());

        assert_eq!(
            manager.nib().nwk_update_id(),
            Some(5),
            "R22 §3.6.1.13.2: the manager increments nwkUpdateId",
        );
        let new_pan_id = manager.pending_pan_id().expect("a switch is armed");
        assert_ne!(new_pan_id, PAN);
        assert_ne!(new_pan_id, PanId(0x4321));

        let record = &manager.mac().tx_history()[0];
        let bytes = record.payload.as_slice();
        let (header, consumed) = NwkHeader::parse(bytes).expect("the frame parses");
        assert_eq!(header.dst_addr, ShortAddress::BROADCAST);
        assert_eq!(bytes[consumed], NwkCommandId::NetworkUpdate as u8);
        let update = PanIdUpdate::parse(&bytes[consumed + 1..]).expect("the update parses");
        assert_eq!(update.update_id, 5);
        assert_eq!(update.new_pan_id, new_pan_id);
        assert_eq!(update.epid, EPID);

        // R22 §3.6.1.13.2: the manager itself only moves after the delivery
        // time, so the announcement can still cross the old PAN.
        assert_eq!(manager.nib().pan_id, PAN);
        block_on(manager.process_pending_routing());
        assert_eq!(manager.nib().pan_id, PAN);
        block_on(
            manager
                .mac_mut()
                .delay_micros(NETWORK_BROADCAST_DELIVERY_TIME_US),
        );
        block_on(manager.process_pending_routing());
        assert_eq!(manager.nib().pan_id, new_pan_id);
        assert!(manager.pending_pan_id().is_none());
    }

    /// R22 §3.6.1.13.1 has *every* router that sees the conflict report it, so
    /// several reports arrive between maintenance passes. A later duplicate
    /// must not cancel the update the manager already owes the network — it
    /// would otherwise change PAN ID alone and strand itself.
    /// A deferred PAN identifier switch and a queued conflict announcement both
    /// belong to the network being left; carrying them across a leave would
    /// retune the radio away from the next network, or announce an address this
    /// device no longer holds.
    #[test]
    #[cfg(feature = "router")]
    fn leaving_the_network_drops_deferred_conflict_work() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().set_nwk_update_id(1);
        router_neighbour(&mut router, PEER, PEER_IEEE);
        router.note_announced_address(PEER, IMPOSTOR_IEEE);
        deliver(
            &mut router,
            PEER,
            NwkCommandId::NetworkUpdate,
            &pan_id_update(EPID, 2, PanId(0xBEEF)),
        );
        assert!(router.pending_pan_id().is_some());

        block_on(router.nlme_leave(false)).expect("the leave is sent");
        assert!(router.pending_pan_id().is_none());

        block_on(
            router
                .mac_mut()
                .delay_micros(NETWORK_BROADCAST_DELIVERY_TIME_US),
        );
        router.set_joined(true);
        let before = router.mac().tx_history().len();
        block_on(router.process_pending_routing());
        assert_eq!(
            router.mac().tx_history().len(),
            before,
            "no conflict announcement and no PAN ID switch survive the leave",
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_second_report_does_not_cancel_the_update_the_manager_owes() {
        let mut manager = node(DeviceType::Coordinator);
        manager.nib_mut().network_address = ShortAddress::COORDINATOR;
        manager.nib_mut().set_nwk_update_id(1);
        router_neighbour(&mut manager, PEER, PEER_IEEE);

        let report = pan_id_report(EPID, &[PAN]);
        deliver(&mut manager, PEER, NwkCommandId::NetworkReport, &report);
        let armed = manager.pending_pan_id().expect("the switch is armed");
        // A second report, before the first update went out.
        deliver(&mut manager, PEER, NwkCommandId::NetworkReport, &report);
        assert_eq!(manager.pending_pan_id(), Some(armed));

        block_on(manager.process_pending_routing());
        let record = manager
            .mac()
            .tx_history()
            .first()
            .expect("the network update is still broadcast");
        let bytes = record.payload.as_slice();
        let (_, consumed) = NwkHeader::parse(bytes).expect("the frame parses");
        assert_eq!(bytes[consumed], NwkCommandId::NetworkUpdate as u8);
        let update = PanIdUpdate::parse(&bytes[consumed + 1..]).expect("the update parses");
        assert_eq!(update.new_pan_id, armed);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_router_adopts_a_newer_pan_id_update_after_the_delivery_time() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().set_nwk_update_id(7);
        router_neighbour(&mut router, PEER, PEER_IEEE);

        deliver(
            &mut router,
            PEER,
            NwkCommandId::NetworkUpdate,
            &pan_id_update(EPID, 8, PanId(0xBEEF)),
        );
        // R22 §3.6.1.13.3: the update id is stored on receipt, the PAN ID waits.
        assert_eq!(router.nib().nwk_update_id(), Some(8));
        assert_eq!(router.nib().pan_id, PAN);

        block_on(router.process_pending_routing());
        assert_eq!(router.nib().pan_id, PAN);
        block_on(
            router
                .mac_mut()
                .delay_micros(NETWORK_BROADCAST_DELIVERY_TIME_US),
        );
        block_on(router.process_pending_routing());
        assert_eq!(router.nib().pan_id, PanId(0xBEEF));
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_stale_or_foreign_pan_id_update_is_refused() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().set_nwk_update_id(7);
        router_neighbour(&mut router, PEER, PEER_IEEE);

        // Older update state.
        deliver(
            &mut router,
            PEER,
            NwkCommandId::NetworkUpdate,
            &pan_id_update(EPID, 6, PanId(0xBEEF)),
        );
        assert!(router.pending_pan_id().is_none());
        assert_eq!(router.nib().nwk_update_id(), Some(7));

        // Another network's EPID.
        deliver(
            &mut router,
            PEER,
            NwkCommandId::NetworkUpdate,
            &pan_id_update([0xAB; 8], 9, PanId(0xBEEF)),
        );
        assert!(router.pending_pan_id().is_none());
        assert_eq!(router.nib().nwk_update_id(), Some(7));
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_report_is_ignored_by_a_device_that_is_not_the_network_manager() {
        let mut router = node(DeviceType::Router);
        router.nib_mut().set_nwk_update_id(3);
        router_neighbour(&mut router, PEER, PEER_IEEE);

        let report = pan_id_report(EPID, &[PAN]);
        deliver(&mut router, PEER, NwkCommandId::NetworkReport, &report);
        block_on(router.process_pending_routing());

        assert!(router.pending_pan_id().is_none());
        assert_eq!(router.nib().nwk_update_id(), Some(3));
        assert!(router.mac().tx_history().is_empty());
    }

    #[test]
    fn beacon_conflict_detection_matches_the_r22_rule() {
        let router = node(DeviceType::Router);
        // Same PAN ID, different EPID — a conflict.
        assert!(router.beacon_reveals_pan_id_conflict(PAN, Some([0xAB; 8])));
        // Same PAN ID, no EPID in the beacon payload — also a conflict.
        assert!(router.beacon_reveals_pan_id_conflict(PAN, None));
        // Our own network.
        assert!(!router.beacon_reveals_pan_id_conflict(PAN, Some(EPID)));
        // A different network on a different PAN ID.
        assert!(!router.beacon_reveals_pan_id_conflict(PanId(0x4321), Some([0xAB; 8])));
    }
}
