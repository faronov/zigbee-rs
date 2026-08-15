//! NLME — NWK Layer Management Entity.
//!
//! Implements all NWK management primitives:
//! - NLME-NETWORK-DISCOVERY — find available networks
//! - NLME-NETWORK-FORMATION — create a new PAN (coordinator)
//! - NLME-JOIN — join a network (association or rejoin)
//! - NLME-LEAVE — leave the network
//! - NLME-PERMIT-JOINING — open/close joining
//! - NLME-START-ROUTER — start routing (router)
//! - NLME-ED-SCAN — energy detection scan
//! - NLME-RESET — reset NWK layer

use crate::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
use crate::neighbor::{NeighborDeviceType, NeighborEntry, Relationship};
use crate::nib::Nib;
use crate::{DeviceType, NwkLayer, NwkStatus};
use zigbee_mac::pib::{PibAttribute, PibValue};
use zigbee_mac::primitives::*;
use zigbee_mac::{MacDriver, MacError};
use zigbee_types::*;

#[cfg(feature = "trace")]
macro_rules! nwk_diag {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}
#[cfg(not(feature = "trace"))]
macro_rules! nwk_diag {
    ($($arg:tt)*) => {};
}

const REJOIN_RESPONSE_WAIT_US: u32 = 491_520;

/// Network descriptor — result of network discovery.
#[derive(Debug, Clone)]
pub struct NetworkDescriptor {
    pub extended_pan_id: IeeeAddress,
    pub pan_id: PanId,
    pub logical_channel: u8,
    pub stack_profile: u8,
    pub zigbee_version: u8,
    pub beacon_order: u8,
    pub superframe_order: u8,
    pub permit_joining: bool,
    pub router_capacity: bool,
    pub end_device_capacity: bool,
    pub update_id: u8,
    /// LQI to the coordinator/router
    pub lqi: u8,
    /// Short address of the beacon sender (coordinator or router)
    pub router_address: ShortAddress,
    /// Network depth of the beacon sender (from Zigbee beacon payload)
    pub depth: u8,
}

impl From<&PanDescriptor> for NetworkDescriptor {
    fn from(pd: &PanDescriptor) -> Self {
        let router_address = match pd.coord_address {
            MacAddress::Short(_, addr) => addr,
            MacAddress::Extended(_, _) => ShortAddress(0xFFFF),
        };
        Self {
            extended_pan_id: pd.zigbee_beacon.extended_pan_id,
            pan_id: pd.coord_address.pan_id(),
            logical_channel: pd.channel,
            stack_profile: pd.zigbee_beacon.stack_profile,
            zigbee_version: pd.zigbee_beacon.protocol_version,
            beacon_order: pd.superframe_spec.beacon_order,
            superframe_order: pd.superframe_spec.superframe_order,
            permit_joining: pd.superframe_spec.association_permit,
            router_capacity: pd.zigbee_beacon.router_capacity,
            end_device_capacity: pd.zigbee_beacon.end_device_capacity,
            update_id: pd.zigbee_beacon.update_id,
            lqi: pd.lqi,
            router_address,
            depth: pd.zigbee_beacon.device_depth,
        }
    }
}

/// Sort the bounded network-discovery list with a compact stable insertion sort.
pub fn sort_network_descriptors_by(
    networks: &mut [NetworkDescriptor],
    mut precedes: impl FnMut(&NetworkDescriptor, &NetworkDescriptor) -> bool,
) {
    for index in 1..networks.len() {
        let mut current = index;
        while current > 0 && precedes(&networks[current], &networks[current - 1]) {
            networks.swap(current - 1, current);
            current -= 1;
        }
    }
}

/// Highest link cost that R22 §3.6.1.4.1/§3.6.1.4.2 accept for a prospective
/// parent.
///
/// Link costs are derived from beacon LQI via
/// [`link_cost_from_lqi`](crate::neighbor::link_cost_from_lqi) and run from `1`
/// (best) to `7` (worst); anything above this bound is not usable as a parent.
pub const MAX_PARENT_LINK_COST: u8 = 3;

/// Wrap-aware "strictly newer" test for the 8-bit `nwkUpdateId`.
///
/// `nwkUpdateId` is a serial number, not an integer: it wraps from `0xFF` to
/// `0x00` while the network keeps moving forward. `candidate` is newer than
/// `reference` when it lies in the following half of the 8-bit circle, i.e.
/// `(candidate - reference) mod 256` is in `1..=127`.
///
/// The exact half-window distance (`128`) is ambiguous under serial-number
/// arithmetic and is deliberately reported as "not newer" in both directions.
///
/// Parent selection needs the "equal or newer" form
/// ([`nwk_update_id_is_current`]); this strict form is the comparison a NWK
/// Update / channel-change handler needs before adopting a new update state.
pub const fn nwk_update_id_is_newer(candidate: u8, reference: u8) -> bool {
    let delta = candidate.wrapping_sub(reference);
    delta != 0 && delta < 0x80
}

/// Wrap-aware "not stale" test for the 8-bit `nwkUpdateId`.
///
/// Returns `true` when `candidate` is equal to or newer than `reference`, the
/// comparison R22 §3.6.1.4.1 states for beacon `nwkUpdateId`. The ambiguous
/// half-window distance (`128`) is treated as stale, so a candidate whose
/// network update state cannot be ordered against ours is never preferred over
/// a candidate that can.
pub const fn nwk_update_id_is_current(candidate: u8, reference: u8) -> bool {
    candidate.wrapping_sub(reference) < 0x80
}

/// R22 rejoin parent-candidate eligibility and ordering (§3.6.1.4.2, which
/// reuses the association procedure of §3.6.1.4.1 with MAC association
/// replaced by the NWK Rejoin Request/Response exchange).
///
/// The normative filter after the scan keeps a candidate only when it
///
/// - belongs to the network we are rejoining (`nwkExtendedPANID`);
/// - advertises capacity **for the device type being requested** —
///   `router_capacity` when rejoining as a router, `end_device_capacity` when
///   rejoining as an end device;
/// - carries an `nwkUpdateId` that is not older than the one we hold
///   (wrap-aware serial-number comparison, §3.6.1.4.1) and, among the
///   candidates that survive the above, is the **most recent** one seen in
///   this scan;
/// - has a link cost of at most [`MAX_PARENT_LINK_COST`].
///
/// Of the resulting suitable parents the normative choice is the one at
/// **minimum depth**. Everything below depth in [`Self::precedes`] is an
/// implementation tie-break, chosen only to make selection deterministic.
///
/// Unlike association, rejoin deliberately does **not** require
/// `permit_joining`: §3.6.1.4.2 explicitly allows a rejoin into a closed
/// network. Capacity, however, is still required by the same text.
///
/// # Unknown local update state
///
/// The staleness comparison is only defined against a *known-good* local
/// `nwkUpdateId` (see [`Nib::nwk_update_id`](crate::nib::Nib::nwk_update_id)).
/// A device that has never held authoritative update state — factory-new, or
/// restored from a record that predates the item — carries
/// [`None`] here. In that case no candidate is rejected as stale; the scan's
/// most recent update ID is still narrowed down to a single deterministic
/// value, and a successful rejoin makes it authoritative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RejoinParentCriteria {
    /// Extended PAN ID of the network we are rejoining (`nwkExtendedPANID`).
    pub extended_pan_id: IeeeAddress,
    /// Our own `nwkUpdateId` when it is known-good; candidates older than it
    /// are stale. `None` when this device holds no authoritative update state,
    /// which disables the staleness gate rather than defaulting it to `0`.
    pub nwk_update_id: Option<u8>,
    /// Device type we are requesting to rejoin as. Selects which beacon
    /// capacity bit a candidate must advertise.
    pub device_type: DeviceType,
    /// Parent we were attached to. Implementation tie-break only.
    pub previous_parent: Option<ShortAddress>,
}

impl RejoinParentCriteria {
    /// Criteria for rejoining `extended_pan_id` as `device_type` while holding
    /// the network update state `nwk_update_id`.
    ///
    /// Pass `None` for `nwk_update_id` when the local update state is not
    /// known-good; pass the value from
    /// [`Nib::nwk_update_id`](crate::nib::Nib::nwk_update_id) otherwise.
    pub const fn new(
        extended_pan_id: IeeeAddress,
        nwk_update_id: Option<u8>,
        device_type: DeviceType,
    ) -> Self {
        Self {
            extended_pan_id,
            nwk_update_id,
            device_type,
            previous_parent: None,
        }
    }

    /// Prefer `parent` when every other criterion ties.
    ///
    /// This is an implementation tie-break, not a normative priority.
    pub const fn with_previous_parent(mut self, parent: ShortAddress) -> Self {
        self.previous_parent = Some(parent);
        self
    }

    /// Whether `candidate` advertises capacity for the requested device type.
    ///
    /// A coordinator does not rejoin; should one ever be configured this way
    /// it is an FFD and is checked against the router capacity bit.
    pub const fn has_required_capacity(&self, candidate: &NetworkDescriptor) -> bool {
        match self.device_type {
            DeviceType::EndDevice => candidate.end_device_capacity,
            DeviceType::Router | DeviceType::Coordinator => candidate.router_capacity,
        }
    }

    /// Link cost of `candidate`, or `None` when it is invalid or unusable.
    ///
    /// A cost of `0` is invalid (link costs start at `1`) and a cost above
    /// [`MAX_PARENT_LINK_COST`] is too poor to parent us.
    pub fn link_cost(&self, candidate: &NetworkDescriptor) -> Option<u8> {
        let cost = crate::neighbor::link_cost_from_lqi(candidate.lqi);
        if cost == 0 || cost > MAX_PARENT_LINK_COST {
            return None;
        }
        Some(cost)
    }

    /// Whether `candidate`'s advertised `nwkUpdateId` is not older than ours.
    ///
    /// Always `true` when the local update state is unknown: without a
    /// reference there is no defensible way to call a candidate stale, and
    /// rejecting on a fabricated reference of `0` would strand the device.
    pub const fn update_id_is_acceptable(&self, candidate: &NetworkDescriptor) -> bool {
        match self.nwk_update_id {
            Some(local) => nwk_update_id_is_current(candidate.update_id, local),
            None => true,
        }
    }

    /// Distance of `candidate`'s network update state ahead of ours, when ours
    /// is known.
    ///
    /// Only meaningful for base-eligible candidates, where it is bounded by
    /// `127` and therefore a totally ordered freshness key (larger is
    /// fresher) even across a wrap of the 8-bit serial number. `None` when the
    /// local update state is unknown, where no such total order exists; see
    /// [`Self::most_recent_update_id`].
    pub const fn freshness(&self, candidate: &NetworkDescriptor) -> Option<u8> {
        match self.nwk_update_id {
            Some(local) => Some(candidate.update_id.wrapping_sub(local)),
            None => None,
        }
    }

    /// Per-candidate ("base") eligibility: everything R22 requires that can be
    /// decided from a single beacon without seeing the rest of the scan.
    ///
    /// Network identity, capacity for the requested device type, a
    /// non-stale `nwkUpdateId` (only enforced when ours is known) and a usable
    /// link cost. The "most recent update ID in the scan" rule is *not* part
    /// of this; see [`Self::is_suitable`].
    pub fn is_base_eligible(&self, candidate: &NetworkDescriptor) -> bool {
        candidate.extended_pan_id == self.extended_pan_id
            && self.has_required_capacity(candidate)
            && self.update_id_is_acceptable(candidate)
            && self.link_cost(candidate).is_some()
    }

    /// The single most recent `nwkUpdateId` advertised by a base-eligible
    /// candidate in `networks`, or `None` when there is no such candidate.
    ///
    /// With a known local update state, freshness is the wrap-aware distance
    /// ahead of it, which is bounded by `127` for base-eligible candidates, so
    /// the maximum is well defined and deterministic even across a wrap.
    ///
    /// With an unknown local update state there is no reference point and
    /// therefore no total order — serial-number comparison is not transitive
    /// over an arbitrary set. The result is instead the fixed point of a
    /// left-to-right fold that only ever moves forward: the first base-eligible
    /// candidate in discovery order seeds the answer, and a later candidate
    /// replaces it only when it is strictly newer under
    /// [`nwk_update_id_is_newer`]. Older and ambiguous (half-window) values
    /// leave the answer untouched, so discovery order breaks every tie and the
    /// outcome is deterministic for a given scan.
    pub fn most_recent_update_id(&self, networks: &[NetworkDescriptor]) -> Option<u8> {
        let mut eligible = networks
            .iter()
            .filter(|candidate| self.is_base_eligible(candidate));

        match self.nwk_update_id {
            Some(local) => eligible
                .filter_map(|candidate| self.freshness(candidate))
                .max()
                .map(|freshness| local.wrapping_add(freshness)),
            None => {
                let mut most_recent = eligible.next()?.update_id;
                for candidate in eligible {
                    if nwk_update_id_is_newer(candidate.update_id, most_recent) {
                        most_recent = candidate.update_id;
                    }
                }
                Some(most_recent)
            }
        }
    }

    /// Whether `candidate` is a suitable rejoin parent, given the most recent
    /// update ID discovered in the same scan.
    ///
    /// Base-eligible candidates that carry an older (but still non-stale)
    /// update ID are excluded: R22 keeps only the most recent one.
    pub fn is_suitable(&self, candidate: &NetworkDescriptor, most_recent_update_id: u8) -> bool {
        self.is_base_eligible(candidate) && candidate.update_id == most_recent_update_id
    }

    /// Strict ordering used to rank candidates: `true` when `candidate`
    /// outranks `other`.
    ///
    /// Normative part:
    /// 1. suitable candidates before unsuitable ones;
    /// 2. **minimum depth** — the only preference R22 states among suitable
    ///    parents.
    ///
    /// Implementation tie-breaks, applied only at equal depth to keep the
    /// attempt order deterministic:
    /// 3. lower link cost;
    /// 4. the previous parent, when configured;
    /// 5. discovery order (the sort is stable).
    pub fn precedes(
        &self,
        candidate: &NetworkDescriptor,
        other: &NetworkDescriptor,
        most_recent_update_id: u8,
    ) -> bool {
        let candidate_suitable = self.is_suitable(candidate, most_recent_update_id);
        if candidate_suitable != self.is_suitable(other, most_recent_update_id) {
            return candidate_suitable;
        }
        if !candidate_suitable {
            return false;
        }

        // Normative: minimum depth wins.
        if candidate.depth != other.depth {
            return candidate.depth < other.depth;
        }

        // Implementation tie-breaks below this line.
        let candidate_cost = self.link_cost(candidate).unwrap_or(u8::MAX);
        let other_cost = self.link_cost(other).unwrap_or(u8::MAX);
        if candidate_cost != other_cost {
            return candidate_cost < other_cost;
        }

        match self.previous_parent {
            Some(previous) => {
                candidate.router_address == previous && other.router_address != previous
            }
            None => false,
        }
    }

    /// Order `networks` in place and return how many leading entries are
    /// suitable rejoin parents.
    ///
    /// Unsuitable entries are kept at the tail in discovery order rather than
    /// dropped, so callers can still report them as diagnostics.
    pub fn select(&self, networks: &mut [NetworkDescriptor]) -> usize {
        let Some(most_recent_update_id) = self.most_recent_update_id(networks) else {
            return 0;
        };
        sort_network_descriptors_by(networks, |candidate, other| {
            self.precedes(candidate, other, most_recent_update_id)
        });
        networks
            .iter()
            .position(|network| !self.is_suitable(network, most_recent_update_id))
            .unwrap_or(networks.len())
    }
}

/// Join method
#[derive(Debug, Clone, Copy)]
pub enum JoinMethod {
    /// MAC-level association (normal first join)
    Association,
    /// NWK rejoin using network key (after losing parent)
    Rejoin,
    /// Direct join (coordinator adds device without association)
    Direct,
}

fn zigbee_capability_info(device_type: DeviceType, rx_on_when_idle: bool) -> CapabilityInfo {
    CapabilityInfo {
        device_type_ffd: device_type != DeviceType::EndDevice,
        mains_powered: device_type != DeviceType::EndDevice,
        rx_on_when_idle,
        // Zigbee PRO uses NWK/APS security, not IEEE 802.15.4 MAC security.
        // This matches the official Telink stack and the working nRF backend.
        security_capable: false,
        allocate_address: true,
    }
}

/// NLME management primitive implementations.
impl<M: MacDriver> NwkLayer<M> {
    // ── Rejoin parent selection ─────────────────────────────

    /// R22 rejoin parent criteria derived from the current NIB.
    ///
    /// Uses `nwkExtendedPANID`, the locally held `nwkUpdateId` — only when it
    /// is known-good, so an unknown state never rejects candidates as stale —
    /// and the device type we rejoin as (which selects the required beacon
    /// capacity bit), and keeps the previous parent as an implementation
    /// tie-break.
    pub fn rejoin_parent_criteria(&self) -> RejoinParentCriteria {
        let criteria = RejoinParentCriteria::new(
            self.nib.extended_pan_id,
            self.nib.nwk_update_id(),
            self.device_type,
        );
        if self.nib.parent_address == ShortAddress(0xFFFF) {
            criteria
        } else {
            criteria.with_previous_parent(self.nib.parent_address)
        }
    }

    /// Order a discovery result for rejoin and return the suitable count.
    ///
    /// The first `n` returned entries are, in order, the rejoin parents this
    /// device may attach to; the remainder are retained for diagnostics only
    /// and must not be used as parents.
    pub fn select_rejoin_parents(&self, networks: &mut [NetworkDescriptor]) -> usize {
        self.rejoin_parent_criteria().select(networks)
    }

    // ── NLME-NETWORK-DISCOVERY ──────────────────────────────

    /// Discover available Zigbee networks on the given channels.
    ///
    /// Performs an active scan via MAC, then filters and converts beacon
    /// responses into network descriptors.
    pub async fn nlme_network_discovery(
        &mut self,
        channel_mask: ChannelMask,
        scan_duration: u8,
    ) -> Result<heapless::Vec<NetworkDescriptor, 16>, NwkStatus> {
        nwk_diag!("[NWK] discovery mask=0x{:08X}", channel_mask.0);

        // Set macAutoRequest = false during scan
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacAutoRequest, PibValue::Bool(false))
            .await;

        let scan_result = self
            .mac
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask,
                scan_duration,
            })
            .await
            .map_err(|_| NwkStatus::NoNetworks)?;

        // Restore macAutoRequest
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacAutoRequest, PibValue::Bool(true))
            .await;

        nwk_diag!(
            "[NWK] discovery found {} PAN descriptors",
            scan_result.pan_descriptors.len()
        );

        let mut networks: heapless::Vec<NetworkDescriptor, 16> = heapless::Vec::new();
        for pd in &scan_result.pan_descriptors {
            nwk_diag!(
                "[NWK] PD ch={} proto={} stack={} depth={} permit={}",
                pd.channel,
                pd.zigbee_beacon.protocol_id,
                pd.zigbee_beacon.stack_profile,
                pd.zigbee_beacon.device_depth,
                pd.superframe_spec.association_permit,
            );
            // Filter: only Zigbee PRO beacons (protocol_id = 0, stack_profile = 2)
            if pd.zigbee_beacon.protocol_id != 0 {
                continue;
            }
            if pd.zigbee_beacon.stack_profile != 2 {
                log::info!(
                    "[NWK] Skipping non-PRO beacon (stack_profile={})",
                    pd.zigbee_beacon.stack_profile
                );
                continue;
            }
            let mut descriptor = NetworkDescriptor::from(pd);
            if let Some(existing) = networks.iter_mut().find(|existing| {
                existing.logical_channel == descriptor.logical_channel
                    && existing.pan_id == descriptor.pan_id
                    && existing.router_address == descriptor.router_address
            }) {
                descriptor.lqi = descriptor.lqi.max(existing.lqi);
                *existing = descriptor;
            } else {
                let _ = networks.push(descriptor);
            }
        }

        if networks.is_empty() {
            return Err(NwkStatus::NoNetworks);
        }

        sort_network_descriptors_by(&mut networks, |candidate, current| {
            candidate.lqi > current.lqi
        });

        Ok(networks)
    }

    // ── NLME-NETWORK-FORMATION ──────────────────────────────

    /// Form a new Zigbee network (coordinator only).
    ///
    /// 1. ED scan to find quietest channel
    /// 2. Choose PAN ID (random, avoid conflicts)
    /// 3. Set MAC PIB and start PAN
    pub async fn nlme_network_formation(
        &mut self,
        channel_mask: ChannelMask,
        scan_duration: u8,
    ) -> Result<(), NwkStatus> {
        if self.device_type != DeviceType::Coordinator {
            return Err(NwkStatus::InvalidRequest);
        }

        // ED scan to find quietest channel
        let ed_result = self
            .mac
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Ed,
                channel_mask,
                scan_duration,
            })
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;

        // Pick channel with lowest energy
        let best_channel = ed_result
            .energy_list
            .iter()
            .min_by_key(|ed| ed.energy)
            .map(|ed| ed.channel)
            .unwrap_or(15); // Default to ch 15

        // Generate a PAN ID from the platform entropy source.
        let mut pan_id_bytes = [0u8; 2];
        self.mac
            .fill_random(&mut pan_id_bytes)
            .map_err(|_| NwkStatus::StartupFailure)?;
        let pan_id = PanId(u16::from_le_bytes(pan_id_bytes) & 0x3FFF);

        // Configure MAC
        self.mac
            .mlme_set(
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(ShortAddress::COORDINATOR),
            )
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;
        self.mac
            .mlme_set(PibAttribute::MacPanId, PibValue::PanId(pan_id))
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;
        self.mac
            .mlme_set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;

        // Start PAN
        self.mac
            .mlme_start(MlmeStartRequest {
                pan_id,
                channel: best_channel,
                beacon_order: 15,     // Non-beacon mode
                superframe_order: 15, // Non-beacon mode
                pan_coordinator: true,
                battery_life_ext: false,
            })
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;

        // Update NIB
        self.nib.pan_id = pan_id;
        self.nib.logical_channel = best_channel;
        self.nib.network_address = ShortAddress::COORDINATOR;
        self.nib.depth = 0;
        // The coordinator defines the network's update state, so forming one
        // makes the local `nwkUpdateId` authoritative from `0` onwards. The
        // beacon payload we start advertising carries this value.
        self.nib.set_nwk_update_id(0);

        // Read our IEEE address from MAC
        if let Ok(PibValue::ExtendedAddress(addr)) =
            self.mac.mlme_get(PibAttribute::MacExtendedAddress).await
        {
            self.nib.ieee_address = addr;
            self.nib.extended_pan_id = addr; // Use own IEEE as extended PAN ID
        }

        self.joined = true;
        log::info!(
            "[NWK] Network formed: PAN 0x{:04X} ch {} addr 0x{:04X}",
            pan_id.0,
            best_channel,
            0x0000u16
        );

        Ok(())
    }

    // ── NLME-JOIN ───────────────────────────────────────────

    /// Join a discovered network.
    ///
    /// Uses MAC association to join the network described by `network`.
    /// On success, we receive a short address and become part of the PAN.
    pub async fn nlme_join(
        &mut self,
        network: &NetworkDescriptor,
        method: JoinMethod,
    ) -> Result<ShortAddress, NwkStatus> {
        match method {
            JoinMethod::Association => self.join_via_association(network).await,
            JoinMethod::Rejoin => self.join_via_rejoin(network).await,
            JoinMethod::Direct => Err(NwkStatus::InvalidRequest),
        }
    }

    async fn join_via_association(
        &mut self,
        network: &NetworkDescriptor,
    ) -> Result<ShortAddress, NwkStatus> {
        nwk_diag!(
            "[NWK] join_assoc: pan=0x{:04X} ch={} via=0x{:04X} permit={} ed_cap={} rtr_cap={}",
            network.pan_id.0,
            network.logical_channel,
            network.router_address.0,
            network.permit_joining,
            network.end_device_capacity,
            network.router_capacity,
        );

        // Check capacity
        match self.device_type {
            DeviceType::Router if !network.router_capacity => {
                nwk_diag!("[NWK] join rejected: no router capacity");
                return Err(NwkStatus::NotPermitted);
            }
            DeviceType::EndDevice if !network.end_device_capacity => {
                nwk_diag!("[NWK] join rejected: no end-device capacity");
                return Err(NwkStatus::NotPermitted);
            }
            _ => {}
        }
        if !network.permit_joining {
            nwk_diag!("[NWK] join rejected: association permit is closed");
            return Err(NwkStatus::NotPermitted);
        }

        // Build capability info. The requested receiver mode must not depend
        // on diagnostic features; sleepy devices can still obtain indirect
        // Transport-Key frames through MAC polling.
        let cap = zigbee_capability_info(self.device_type, self.rx_on_when_idle);

        nwk_diag!(
            "[NWK] assoc: ffd={} rx_on={} dev_type={:?}",
            cap.device_type_ffd,
            cap.rx_on_when_idle,
            self.device_type,
        );

        // Perform MAC association
        let join_target = if network.router_address.0 != 0xFFFF {
            network.router_address
        } else {
            ShortAddress::COORDINATOR
        };
        let coord_addr = MacAddress::Short(network.pan_id, join_target);
        let result = self
            .mac
            .mlme_associate(MlmeAssociateRequest {
                channel: network.logical_channel,
                coord_address: coord_addr,
                capability_info: cap,
            })
            .await
            .map_err(|e| {
                nwk_diag!("[NWK] association failed: {:?}", e);
                match e {
                    MacError::NoAck => NwkStatus::NoNetworks,
                    _ => NwkStatus::StartupFailure,
                }
            })?;

        nwk_diag!(
            "[NWK] assoc result: status={:?} addr=0x{:04X}",
            result.status,
            result.short_address.0,
        );

        if result.status != AssociationStatus::Success {
            return Err(NwkStatus::NotPermitted);
        }

        // Update NIB with assigned address
        self.nib.network_address = result.short_address;
        self.nib.pan_id = network.pan_id;
        self.nib.logical_channel = network.logical_channel;
        self.nib.extended_pan_id = network.extended_pan_id;
        // A successful association is an authoritative statement of the
        // network's update state: adopt the parent's beacon value and mark it
        // known-good.
        self.nib.set_nwk_update_id(network.update_id);
        self.nib.stack_profile = network.stack_profile;
        self.nib.parent_address = join_target;
        // Authoritative parent assignment: the R22 End Device Timeout has to
        // be renegotiated with this parent before any of its keepalive
        // advertisements may be trusted again.
        self.nib.reset_end_device_timeout_negotiation();

        // Set macCoordShortAddress so MAC layer knows the parent for mlme_poll
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::MacCoordShortAddress,
                PibValue::ShortAddress(join_target),
            )
            .await;

        // Set MAC PAN ID — critical for outgoing frames to have correct PAN
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacPanId, PibValue::PanId(network.pan_id))
            .await;

        // Set our MAC short address — needed for source addressing in TX frames
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(result.short_address),
            )
            .await;

        // Read our IEEE address
        if let Ok(PibValue::ExtendedAddress(addr)) =
            self.mac.mlme_get(PibAttribute::MacExtendedAddress).await
        {
            self.nib.ieee_address = addr;
        }

        // Set depth from beacon's device_depth + 1 (our depth is one hop deeper than parent)
        self.nib.depth = network.depth.saturating_add(1);

        // Update MAC PIB
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::MacRxOnWhenIdle,
                PibValue::Bool(self.rx_on_when_idle),
            )
            .await;

        // Add parent to neighbor table — use actual join target info
        // Try to get coordinator IEEE from MAC PIB (cached from association)
        let parent_ieee = if let Ok(PibValue::ExtendedAddress(addr)) = self
            .mac
            .mlme_get(PibAttribute::MacCoordExtendedAddress)
            .await
        {
            addr
        } else {
            [0; 8] // Will be updated when we receive a frame with source IEEE
        };
        // Use actual join target address and determine device type from address
        let parent_device_type = if join_target == ShortAddress::COORDINATOR {
            NeighborDeviceType::Coordinator
        } else {
            NeighborDeviceType::Router
        };
        let parent = NeighborEntry {
            ieee_address: parent_ieee,
            network_address: join_target,
            device_type: parent_device_type,
            rx_on_when_idle: true,
            security_capable: true,
            relationship: Relationship::Parent,
            lqi: network.lqi,
            // R22 §3.6.1.5: the outgoing cost is the neighbor's own
            // measurement and stays unknown until a Link Status naming
            // this device arrives (§3.6.3.4.2).
            incoming_cost: crate::neighbor::link_cost_from_lqi(network.lqi),
            outgoing_cost: 0,
            link_status_age: 0,
            depth: network.depth,
            permit_joining: network.permit_joining,
            age: 0,
            end_device_timeout: crate::frames::ED_TIMEOUT_ENUM_DEFAULT,
            keepalive_remaining_secs: 0,
            keepalive_confirmed: false,
            #[cfg(feature = "router")]
            parent_annce_pending: false,
            extended_pan_id: network.extended_pan_id,
            active: true,
        };
        let _ = self.neighbors.add_or_update(parent);

        self.joined = true;

        log::info!(
            "[NWK] Joined PAN 0x{:04X} ch {} as 0x{:04X}",
            network.pan_id.0,
            network.logical_channel,
            result.short_address.0
        );

        Ok(result.short_address)
    }

    async fn join_via_rejoin(
        &mut self,
        network: &NetworkDescriptor,
    ) -> Result<ShortAddress, NwkStatus> {
        self.rejoin_diagnostics.stage = 1;
        self.rejoin_diagnostics.candidate_attempts =
            self.rejoin_diagnostics.candidate_attempts.saturating_add(1);
        self.rejoin_diagnostics.last_parent = network.router_address.0;

        // R22 §3.6.1.4.1/§3.6.1.4.2 — never rejoin through a candidate that is
        // on another network, does not advertise capacity for our device type,
        // advertises a stale nwkUpdateId, or has an unusable link cost.
        //
        // The staleness part of that gate only applies when our own update
        // state is known-good; `RejoinParentCriteria` carries that validity,
        // so an unknown local state skips the comparison here exactly as it
        // does in whole-scan selection.
        //
        // This is the mechanical, per-candidate part of the rule only. The
        // "most recent update ID in the scan" and the minimum-depth preference
        // are policy over the whole discovery result and cannot be decided
        // here; callers rank candidates with `select_rejoin_parents` first.
        let criteria = self.rejoin_parent_criteria();
        if !criteria.is_base_eligible(network) {
            match criteria.nwk_update_id {
                Some(local) => log::warn!(
                    "[NWK] Rejoin candidate 0x{:04X} rejected: update_id={} (local {}) lqi={} \
                     router_cap={} ed_cap={}",
                    network.router_address.0,
                    network.update_id,
                    local,
                    network.lqi,
                    network.router_capacity,
                    network.end_device_capacity,
                ),
                None => log::warn!(
                    "[NWK] Rejoin candidate 0x{:04X} rejected: update_id={} (local unknown) \
                     lqi={} router_cap={} ed_cap={}",
                    network.router_address.0,
                    network.update_id,
                    network.lqi,
                    network.router_capacity,
                    network.end_device_capacity,
                ),
            }
            self.rejoin_diagnostics.last_status = NwkStatus::InvalidRequest as u8;
            return Err(NwkStatus::InvalidRequest);
        }

        // Rejoin uses NWK-level Rejoin Request command (encrypted with network key)
        // This is used when a device has been disconnected but still knows the network key

        // Switch to the target channel
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::PhyCurrentChannel,
                PibValue::U8(network.logical_channel),
            )
            .await;
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacPanId, PibValue::PanId(network.pan_id))
            .await;
        // Set MAC short address so the MAC address filter accepts the
        // unicast Rejoin Response addressed to our restored NWK address.
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(self.nib.network_address),
            )
            .await;
        let _ = self
            .mac
            .mlme_set(
                PibAttribute::MacCoordShortAddress,
                PibValue::ShortAddress(network.router_address),
            )
            .await;
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true))
            .await;

        // Build NWK Rejoin Request frame
        let cap_byte = zigbee_capability_info(self.device_type, self.rx_on_when_idle);

        let seq = self.nib.next_seq();
        let mut nwk_frame_buf = [0u8; 64];
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: self.nib.security_enabled,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                // EDI indicates forwarding on behalf of a child. A device
                // selecting a new prospective parent clears it during rejoin.
                end_device_initiator: false,
            },
            dst_addr: network.router_address,
            src_addr: self.nib.network_address,
            radius: 1,
            seq_number: seq,
            dst_ieee: None,
            src_ieee: Some(self.nib.ieee_address),
            multicast_control: None,
            source_route: None,
        };

        let hdr_len = header.serialize(&mut nwk_frame_buf);

        // Rejoin Request command payload: command_id(1) + capability_info(1)
        let cmd_payload = [0x06u8, cap_byte.to_byte()];
        let total_len;

        if self.nib.security_enabled {
            // Encrypt rejoin request with network key
            let sec_hdr = crate::security::NwkSecurityHeader {
                security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
                frame_counter: self
                    .nib
                    .next_frame_counter()
                    .ok_or(NwkStatus::InvalidRequest)?,
                source_address: self.nib.ieee_address,
                key_seq_number: self.nib.active_key_seq_number,
            };
            let sec_hdr_len = sec_hdr.serialize(&mut nwk_frame_buf[hdr_len..]);
            let aad_len = hdr_len + sec_hdr_len;

            if let Some(key_entry) = self.security.active_key() {
                if let Some(encrypted) = self.security.encrypt_with(
                    &mut self.mac,
                    &nwk_frame_buf[..aad_len],
                    &cmd_payload,
                    &key_entry.key,
                    &sec_hdr,
                ) {
                    if aad_len + encrypted.len() > nwk_frame_buf.len() {
                        return Err(NwkStatus::FrameTooLong);
                    }
                    nwk_frame_buf[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
                    total_len = aad_len + encrypted.len();
                    // Zigbee transmits security level 0 in the auxiliary
                    // header while authenticating with the actual level 5.
                    nwk_frame_buf[hdr_len] &= !0x07;
                } else {
                    return Err(NwkStatus::InvalidRequest);
                }
            } else {
                return Err(NwkStatus::InvalidRequest);
            }
        } else {
            nwk_frame_buf[hdr_len..hdr_len + 2].copy_from_slice(&cmd_payload);
            total_len = hdr_len + 2;
        }

        // Send via MAC
        self.rejoin_diagnostics.tx_attempts = self.rejoin_diagnostics.tx_attempts.saturating_add(1);
        let tx_result = self
            .mac
            .mcps_data(zigbee_mac::McpsDataRequest {
                src_addr_mode: zigbee_mac::AddressMode::Short,
                dst_address: MacAddress::Short(network.pan_id, network.router_address),
                payload: &nwk_frame_buf[..total_len],
                msdu_handle: seq,
                tx_options: zigbee_mac::TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await;
        if let Err(error) = tx_result {
            let status = match error {
                MacError::NoAck => {
                    self.rejoin_diagnostics.no_ack_failures =
                        self.rejoin_diagnostics.no_ack_failures.saturating_add(1);
                    NwkStatus::NoNetworks
                }
                MacError::ChannelAccessFailure => {
                    self.rejoin_diagnostics.channel_access_failures = self
                        .rejoin_diagnostics
                        .channel_access_failures
                        .saturating_add(1);
                    NwkStatus::NoNetworks
                }
                _ => {
                    self.rejoin_diagnostics.other_tx_failures =
                        self.rejoin_diagnostics.other_tx_failures.saturating_add(1);
                    NwkStatus::StartupFailure
                }
            };
            self.rejoin_diagnostics.stage = 2;
            self.rejoin_diagnostics.last_status = status as u8;
            return Err(status);
        }
        self.rejoin_diagnostics.stage = 3;

        // A sleepy end device receives the Rejoin Response indirectly and
        // must poll the prospective parent during aResponseWaitTime
        // (approximately 492 ms at 2.4 GHz).
        const SLEEPY_POLL_INTERVAL_US: u32 = 50_000;
        const MAX_RX_ATTEMPTS: usize = 64;
        let sleepy = self.device_type == DeviceType::EndDevice && !self.rx_on_when_idle;
        let response_wait_started = self.mac.monotonic_micros();
        let mut attempt = 0usize;

        loop {
            if attempt >= MAX_RX_ATTEMPTS {
                break;
            }
            let elapsed = self
                .mac
                .monotonic_micros()
                .wrapping_sub(response_wait_started);
            if elapsed >= REJOIN_RESPONSE_WAIT_US {
                break;
            }
            let response_time_remaining = REJOIN_RESPONSE_WAIT_US.saturating_sub(elapsed);
            attempt += 1;
            self.rejoin_diagnostics.poll_attempts =
                self.rejoin_diagnostics.poll_attempts.saturating_add(1);

            let frame = if sleepy {
                match self.mac.mlme_poll_timeout(response_time_remaining).await {
                    Ok(Some(frame)) => frame,
                    Ok(None) | Err(_) => {
                        let elapsed = self
                            .mac
                            .monotonic_micros()
                            .wrapping_sub(response_wait_started);
                        let remaining = REJOIN_RESPONSE_WAIT_US.saturating_sub(elapsed);
                        self.mac
                            .delay_micros(SLEEPY_POLL_INTERVAL_US.min(remaining))
                            .await;
                        continue;
                    }
                }
            } else {
                match self
                    .mac
                    .mcps_data_indication_timeout(response_time_remaining)
                    .await
                {
                    Ok(indication) => indication.payload,
                    Err(_) => break,
                }
            };
            self.rejoin_diagnostics.rx_frames = self.rejoin_diagnostics.rx_frames.saturating_add(1);

            let data = frame.as_slice();
            let (hdr, consumed) = match NwkHeader::parse(data) {
                Some(v) => v,
                None => {
                    log::info!(
                        "[NWK] Rejoin RX #{}: not NWK ({} bytes)",
                        attempt,
                        data.len()
                    );
                    continue;
                }
            };

            let ft = hdr.frame_control.frame_type;
            log::info!(
                "[NWK] Rejoin RX #{}: ft={} src=0x{:04X} dst=0x{:04X} sec={}",
                attempt,
                ft,
                hdr.src_addr.0,
                hdr.dst_addr.0,
                hdr.frame_control.security
            );

            // Must be a NWK Command frame
            if ft != NwkFrameType::Command as u8 {
                continue;
            }
            let Some(parent_ieee) = hdr.src_ieee else {
                continue;
            };
            if hdr.src_addr != network.router_address
                || hdr.dst_addr != self.nib.network_address
                || hdr.dst_ieee != Some(self.nib.ieee_address)
                || (self.nib.security_enabled && !hdr.frame_control.security)
            {
                log::warn!("[NWK] Rejoin RX #{}: unrelated response", attempt);
                continue;
            }

            // Get command payload — may need NWK decryption
            let cmd_data = if hdr.frame_control.security {
                let after_hdr = &data[consumed..];
                let (sec_hdr, sec_consumed) =
                    match crate::security::NwkSecurityHeader::parse(after_hdr) {
                        Some(v) => v,
                        None => {
                            log::warn!("[NWK] Rejoin RX #{}: bad security header", attempt);
                            continue;
                        }
                    };
                if sec_hdr.source_address != parent_ieee
                    || !self.security.check_frame_counter_for_key(
                        &sec_hdr.source_address,
                        sec_hdr.key_seq_number,
                        sec_hdr.frame_counter,
                    )
                {
                    log::warn!(
                        "[NWK] Rejoin RX #{}: replay or source mismatch (fc={})",
                        attempt,
                        sec_hdr.frame_counter
                    );
                    continue;
                }
                let key = match self.security.key_by_seq(sec_hdr.key_seq_number) {
                    Some(k) => k.key,
                    None => {
                        log::warn!(
                            "[NWK] Rejoin RX #{}: unknown key seq {}",
                            attempt,
                            sec_hdr.key_seq_number
                        );
                        continue;
                    }
                };
                let aad_len = consumed + sec_consumed;
                // AAD must use ACTUAL security level (5), not OTA value (0).
                // Patch the security control byte (first byte after NWK header).
                let mut aad_buf = [0u8; 64];
                let copy_len = aad_len.min(aad_buf.len());
                aad_buf[..copy_len].copy_from_slice(&data[..copy_len]);
                aad_buf[consumed] = (aad_buf[consumed] & !0x07) | 0x05;
                match self.security.decrypt_with(
                    &mut self.mac,
                    &aad_buf[..copy_len],
                    &after_hdr[sec_consumed..],
                    &key,
                    &sec_hdr,
                ) {
                    Some(v) => {
                        self.security.commit_frame_counter_for_key(
                            &sec_hdr.source_address,
                            sec_hdr.key_seq_number,
                            sec_hdr.frame_counter,
                        );
                        v
                    }
                    None => {
                        log::warn!(
                            "[NWK] Rejoin RX #{}: decrypt failed (fc={})",
                            attempt,
                            sec_hdr.frame_counter
                        );
                        continue;
                    }
                }
            } else {
                if self.nib.security_enabled {
                    continue;
                }
                let payload = &data[consumed..];
                let mut v = heapless::Vec::<u8, 128>::new();
                let _ = v.extend_from_slice(payload);
                v
            };

            // Rejoin Response: cmd_id(0x07) + new_short_addr(2) + rejoin_status(1)
            log::info!(
                "[NWK] Rejoin RX #{}: decrypted cmd_id=0x{:02X} len={}",
                attempt,
                cmd_data.first().copied().unwrap_or(0xFF),
                cmd_data.len()
            );
            if cmd_data.len() >= 4 && cmd_data[0] == 0x07 {
                let new_addr = u16::from_le_bytes([cmd_data[1], cmd_data[2]]);
                let rejoin_status = cmd_data[3];

                if rejoin_status == 0x00 && (0x0001..=0xFFF7).contains(&new_addr) {
                    self.rejoin_diagnostics.stage = 5;
                    self.rejoin_diagnostics.last_status = 0;
                    log::info!("[NWK] Rejoin accepted, new addr=0x{:04X}", new_addr);
                    self.nib.network_address = ShortAddress(new_addr);
                    // Refresh parent address to the sender of the rejoin response
                    self.nib.parent_address = hdr.src_addr;
                    // Authoritative parent assignment — renegotiate the R22
                    // End Device Timeout with the parent that accepted us.
                    self.nib.reset_end_device_timeout_negotiation();
                    self.nib.extended_pan_id = network.extended_pan_id;
                    self.nib.pan_id = network.pan_id;
                    self.nib.logical_channel = network.logical_channel;
                    // The parent accepted the rejoin, so the update state we
                    // rejoined against is now authoritative — including the
                    // case where we started from an unknown local state and
                    // picked this candidate's ID by discovery order.
                    self.nib.set_nwk_update_id(network.update_id);
                    // Update depth from beacon (parent depth + 1)
                    self.nib.depth = network.depth.saturating_add(1);
                    let _ = self
                        .mac
                        .mlme_set(
                            PibAttribute::MacShortAddress,
                            PibValue::ShortAddress(ShortAddress(new_addr)),
                        )
                        .await;
                    let _ = self
                        .mac
                        .mlme_set(
                            PibAttribute::MacCoordShortAddress,
                            PibValue::ShortAddress(hdr.src_addr),
                        )
                        .await;
                    let _ = self
                        .mac
                        .mlme_set(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true))
                        .await;
                    // Update parent neighbor entry
                    let parent_device_type = if hdr.src_addr == ShortAddress::COORDINATOR {
                        NeighborDeviceType::Coordinator
                    } else {
                        NeighborDeviceType::Router
                    };
                    let parent_ieee = hdr.src_ieee.unwrap_or([0; 8]);
                    let parent = NeighborEntry {
                        ieee_address: parent_ieee,
                        network_address: hdr.src_addr,
                        device_type: parent_device_type,
                        rx_on_when_idle: true,
                        security_capable: true,
                        relationship: Relationship::Parent,
                        lqi: network.lqi,
                        // R22 §3.6.1.5: the outgoing cost is the neighbor's own
                        // measurement and stays unknown until a Link Status naming
                        // this device arrives (§3.6.3.4.2).
                        incoming_cost: crate::neighbor::link_cost_from_lqi(network.lqi),
                        outgoing_cost: 0,
                        link_status_age: 0,
                        depth: network.depth,
                        permit_joining: network.permit_joining,
                        age: 0,
                        end_device_timeout: crate::frames::ED_TIMEOUT_ENUM_DEFAULT,
                        keepalive_remaining_secs: 0,
                        keepalive_confirmed: false,
                        #[cfg(feature = "router")]
                        parent_annce_pending: false,
                        extended_pan_id: network.extended_pan_id,
                        active: true,
                    };
                    let _ = self.neighbors.add_or_update(parent);
                    self.joined = true;
                    return Ok(ShortAddress(new_addr));
                } else {
                    self.rejoin_diagnostics.stage = 6;
                    self.rejoin_diagnostics.last_status = rejoin_status;
                    log::warn!("[NWK] Rejoin rejected (status=0x{:02X})", rejoin_status);
                    return Err(NwkStatus::NotPermitted);
                }
            }
        }

        log::warn!(
            "[NWK] Rejoin response not received after {} attempts",
            attempt
        );
        self.rejoin_diagnostics.stage = 4;
        self.rejoin_diagnostics.last_status = NwkStatus::NoNetworks as u8;
        Err(NwkStatus::NoNetworks)
    }

    // ── NLME-LEAVE ──────────────────────────────────────────

    /// Leave the current network.
    pub async fn nlme_leave(&mut self, rejoin: bool) -> Result<(), NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }

        // Send NWK Leave command
        let seq = self.nib.next_seq();
        let mut buf = [0u8; 128];
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: self.nib.security_enabled,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr: ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            src_addr: self.nib.network_address,
            radius: 1,
            seq_number: seq,
            dst_ieee: None,
            src_ieee: Some(self.nib.ieee_address),
            multicast_control: None,
            source_route: None,
        };
        let hdr_len = header.serialize(&mut buf);

        // Leave command payload: command ID + options byte
        let leave_cmd = crate::frames::LeaveCommand {
            remove_children: false,
            request: false,
            rejoin,
        };
        let payload = [0x04u8, leave_cmd.serialize()]; // cmd_id=Leave, options

        let total_len;
        if self.nib.security_enabled {
            // Apply NWK security — same path as rejoin and data frames
            let sec_hdr = crate::security::NwkSecurityHeader {
                security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
                frame_counter: self
                    .nib
                    .next_frame_counter()
                    .ok_or(NwkStatus::InvalidRequest)?,
                source_address: self.nib.ieee_address,
                key_seq_number: self.nib.active_key_seq_number,
            };
            let sec_hdr_len = sec_hdr.serialize(&mut buf[hdr_len..]);
            let aad_len = hdr_len + sec_hdr_len;

            if let Some(key_entry) = self.security.active_key() {
                if let Some(encrypted) = self.security.encrypt_with(
                    &mut self.mac,
                    &buf[..aad_len],
                    &payload,
                    &key_entry.key,
                    &sec_hdr,
                ) {
                    if aad_len + encrypted.len() > buf.len() {
                        return Err(NwkStatus::FrameTooLong);
                    }
                    buf[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
                    total_len = aad_len + encrypted.len();
                    buf[hdr_len] &= !0x07;
                } else {
                    return Err(NwkStatus::BadCcmOutput);
                }
            } else {
                return Err(NwkStatus::NoKey);
            }
        } else {
            // No security — send plaintext
            buf[hdr_len..hdr_len + 2].copy_from_slice(&payload);
            total_len = hdr_len + 2;
        }

        let _ = self
            .mac
            .mcps_data(zigbee_mac::McpsDataRequest {
                src_addr_mode: zigbee_mac::AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, ShortAddress::BROADCAST),
                payload: &buf[..total_len],
                msdu_handle: seq,
                tx_options: zigbee_mac::TxOptions {
                    ack_tx: false,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::SyncFailure)?;

        // MAC disassociation
        let _ = self
            .mac
            .mlme_disassociate(MlmeDisassociateRequest {
                device_address: MacAddress::Short(self.nib.pan_id, self.nib.parent_address),
                reason: DisassociateReason::DeviceLeave,
                tx_indirect: false,
            })
            .await;

        self.finish_leave(rejoin);

        log::info!("[NWK] Left network, rejoin={rejoin}");

        Ok(())
    }

    fn finish_leave(&mut self, rejoin: bool) {
        self.joined = false;
        self.neighbors = crate::neighbor::NeighborTable::new();
        self.routing = crate::routing::RoutingTable::new();
        // Conflict work belongs to the network being left: an address-conflict
        // announcement names an address this device no longer holds, and a
        // deferred PAN identifier switch would retune the radio away from
        // whatever network it joins next (R22 §3.6.1.9.3, §3.6.1.13.3).
        self.pending_conflicts.clear();
        self.pending_pan_id_update = None;
        self.pending_pan_id_broadcast = None;
        if !rejoin {
            self.nib.network_address = ShortAddress(0xFFFF);
            self.nib.pan_id = PanId(0xFFFF);
            self.nib.parent_address = ShortAddress(0xFFFF);
            self.nib.logical_channel = 0;
            self.nib.depth = 0;
            self.nib.extended_pan_id = [0u8; 8];
            // We no longer belong to any network, so the update state we held
            // describes nothing. Clearing it (rather than leaving a stale
            // byte behind) keeps a later rejoin from filtering candidates
            // against a network we have left.
            self.nib.clear_nwk_update_id();
            self.security = crate::security::NwkSecurity::new();
            // The parent relationship is gone: no advertised keepalive method
            // survives a full leave.
            self.nib.reset_end_device_timeout_negotiation();
        }
    }

    // ── NLME-ORPHAN-RECOVERY ────────────────────────────────

    /// Check whether the parent is still reachable.
    ///
    /// Returns `false` if the parent entry is missing from the neighbor
    /// table or has an age value indicating staleness.
    pub fn nlme_check_parent_alive(&self) -> bool {
        if !self.joined {
            return false;
        }
        match self.neighbors.parent() {
            Some(entry) => entry.age < 255,
            None => false,
        }
    }

    /// Attempt to recover from parent loss via orphan rejoin.
    ///
    /// Scans for the original network (matched by extended PAN ID) and
    /// attempts an NWK-level rejoin.  Channels are tried in order:
    /// current → primary (11, 15, 20, 25) → all 2.4 GHz.
    pub async fn nlme_orphan_recovery(&mut self) -> Result<ShortAddress, NwkStatus> {
        self.joined = false;
        log::info!("[NWK] Parent lost — starting orphan recovery");

        let saved_ext_pan = self.nib.extended_pan_id;
        let saved_channel = self.nib.logical_channel;

        // Helper closure-like search: try discovery on a channel mask and
        // rejoin the first network whose extended PAN ID matches.
        // Phase 1 — current channel only
        let current_mask = ChannelMask(1u32 << saved_channel);
        if let Ok(addr) = self.try_rejoin_on_mask(current_mask, &saved_ext_pan).await {
            return Ok(addr);
        }

        // Phase 2 — primary Touchlink channels (11, 15, 20, 25)
        let primary_mask = ChannelMask((1u32 << 11) | (1u32 << 15) | (1u32 << 20) | (1u32 << 25));
        if let Ok(addr) = self.try_rejoin_on_mask(primary_mask, &saved_ext_pan).await {
            return Ok(addr);
        }

        // Phase 3 — all 2.4 GHz channels
        if let Ok(addr) = self
            .try_rejoin_on_mask(ChannelMask::ALL_2_4GHZ, &saved_ext_pan)
            .await
        {
            return Ok(addr);
        }

        log::warn!("[NWK] Orphan recovery failed — network not found");
        Err(NwkStatus::NoNetworks)
    }

    /// Scan on `mask` and attempt rejoin on the suitable parents of `ext_pan`,
    /// minimum depth first (R22 §3.6.1.4.2 ordering).
    async fn try_rejoin_on_mask(
        &mut self,
        mask: ChannelMask,
        ext_pan: &IeeeAddress,
    ) -> Result<ShortAddress, NwkStatus> {
        let mut networks = self.nlme_network_discovery(mask, 3).await?;
        let criteria =
            RejoinParentCriteria::new(*ext_pan, self.nib.nwk_update_id(), self.device_type());
        let criteria = if self.nib.parent_address == ShortAddress(0xFFFF) {
            criteria
        } else {
            criteria.with_previous_parent(self.nib.parent_address)
        };
        let suitable = criteria.select(&mut networks);
        if suitable == 0 {
            log::warn!(
                "[NWK] Orphan recovery: {} beacon(s), none suitable as rejoin parent",
                networks.len()
            );
            return Err(NwkStatus::NoNetworks);
        }
        for net in &networks[..suitable] {
            match self.nlme_join(net, JoinMethod::Rejoin).await {
                Ok(addr) => {
                    log::info!("[NWK] Orphan recovery succeeded — addr=0x{:04X}", addr.0);
                    return Ok(addr);
                }
                Err(e) => {
                    log::debug!("[NWK] Rejoin attempt failed: {:?}", e);
                }
            }
        }
        Err(NwkStatus::NoNetworks)
    }

    // ── NLME-PERMIT-JOINING ─────────────────────────────────

    /// Open or close the network for joining.
    ///
    /// Duration: 0 = close, 0xFF = open permanently, 1-254 = open for N seconds.
    pub async fn nlme_permit_joining(&mut self, duration: u8) -> Result<(), NwkStatus> {
        if self.device_type == DeviceType::EndDevice {
            return Err(NwkStatus::InvalidRequest);
        }

        // Commit the NIB only after the MAC accepts the matching PIB state so
        // policy and over-the-air association handling cannot diverge.
        self.mac
            .mlme_set(
                PibAttribute::MacAssociationPermit,
                PibValue::Bool(duration != 0),
            )
            .await
            .map_err(|_| NwkStatus::InvalidRequest)?;
        self.nib.permit_joining = duration != 0;
        self.nib.permit_joining_duration = duration;

        log::info!("[NWK] Permit joining: duration={duration}");
        Ok(())
    }

    /// Age a finite permit-joining window and synchronize the MAC PIB when it
    /// closes. `0xFF` remains open indefinitely and `0` remains closed.
    ///
    /// Returns `true` only when this tick transitions the window to closed.
    pub async fn tick_permit_joining(&mut self, elapsed_secs: u16) -> Result<bool, NwkStatus> {
        if !self.nib.permit_joining
            || self.nib.permit_joining_duration == 0
            || self.nib.permit_joining_duration == 0xFF
            || elapsed_secs == 0
        {
            return Ok(false);
        }
        let remaining = u16::from(self.nib.permit_joining_duration);
        if elapsed_secs < remaining {
            self.nib.permit_joining_duration = (remaining - elapsed_secs) as u8;
            return Ok(false);
        }
        self.nlme_permit_joining(0).await?;
        Ok(true)
    }

    // ── NLME-START-ROUTER ───────────────────────────────────

    /// Start operating as a router (after joining as router).
    pub async fn nlme_start_router(&mut self) -> Result<(), NwkStatus> {
        if self.device_type != DeviceType::Router {
            return Err(NwkStatus::InvalidRequest);
        }
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }

        // Start MAC (non-beacon mode)
        self.mac
            .mlme_start(MlmeStartRequest {
                pan_id: self.nib.pan_id,
                channel: self.nib.logical_channel,
                beacon_order: 15,
                superframe_order: 15,
                pan_coordinator: false,
                battery_life_ext: false,
            })
            .await
            .map_err(|_| NwkStatus::StartupFailure)?;

        // Ensure RX on when idle
        let _ = self
            .mac
            .mlme_set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))
            .await;

        log::info!(
            "[NWK] Router started on PAN 0x{:04X} ch {}",
            self.nib.pan_id.0,
            self.nib.logical_channel
        );
        Ok(())
    }

    // ── NLME-ED-SCAN ───────────────────────────────────────────

    /// Perform an energy-detection scan on the specified channels.
    ///
    /// Returns the scan result with energy readings per channel.
    pub async fn nlme_ed_scan(
        &mut self,
        channel_mask: ChannelMask,
        scan_duration: u8,
    ) -> Result<MlmeScanConfirm, NwkStatus> {
        self.mac
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Ed,
                channel_mask,
                scan_duration,
            })
            .await
            .map_err(|_| NwkStatus::InvalidRequest)
    }

    // ── NLME-SET-CHANNEL ──────────────────────────────────────

    /// Change the operating channel.
    pub async fn nlme_set_channel(&mut self, channel: u8) -> Result<(), NwkStatus> {
        self.mac
            .mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(channel))
            .await
            .map_err(|_| NwkStatus::InvalidRequest)?;
        self.nib.logical_channel = channel;
        log::info!("[NWK] Channel changed to {channel}");
        Ok(())
    }

    // ── NLME-RESET ──────────────────────────────────────────

    /// Reset the NWK layer to initial state.
    pub fn nlme_reset(&mut self, warm_start: bool) -> Result<(), NwkStatus> {
        if !warm_start {
            self.nib = Nib::new();
            self.neighbors = crate::neighbor::NeighborTable::new();
            self.routing = crate::routing::RoutingTable::new();
            self.security = crate::security::NwkSecurity::new();
            self.joined = false;
            self.pending_conflicts.clear();
            self.pending_pan_id_update = None;
            self.pending_pan_id_broadcast = None;
        }

        self.mac
            .mlme_reset(!warm_start)
            .map_err(|_| NwkStatus::InvalidRequest)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_mac::mock::MockMac;

    fn descriptor(router_address: u16, lqi: u8) -> NetworkDescriptor {
        NetworkDescriptor {
            extended_pan_id: [0; 8],
            pan_id: PanId(0x1234),
            logical_channel: 15,
            stack_profile: 2,
            zigbee_version: 2,
            beacon_order: 15,
            superframe_order: 15,
            permit_joining: true,
            router_capacity: true,
            end_device_capacity: true,
            update_id: 0,
            lqi,
            router_address: ShortAddress(router_address),
            depth: 1,
        }
    }

    #[test]
    fn bounded_descriptor_sort_orders_by_lqi_and_preserves_ties() {
        let mut networks = [
            descriptor(0x0001, 100),
            descriptor(0x0002, 200),
            descriptor(0x0003, 200),
            descriptor(0x0004, 50),
        ];

        sort_network_descriptors_by(&mut networks, |candidate, current| {
            candidate.lqi > current.lqi
        });

        assert_eq!(
            networks.map(|network| network.router_address),
            [
                ShortAddress(0x0002),
                ShortAddress(0x0003),
                ShortAddress(0x0001),
                ShortAddress(0x0004),
            ]
        );
    }

    #[test]
    fn bounded_descriptor_sort_supports_parent_preference() {
        let previous_parent = ShortAddress(0x0001);
        let mut networks = [
            descriptor(0x0002, 220),
            descriptor(0x0001, 80),
            descriptor(0x0003, 180),
        ];

        sort_network_descriptors_by(&mut networks, |candidate, current| {
            match (
                candidate.router_address == previous_parent,
                current.router_address == previous_parent,
            ) {
                (true, false) => true,
                (false, true) => false,
                _ => candidate.lqi > current.lqi,
            }
        });

        assert_eq!(
            networks.map(|network| network.router_address),
            [
                ShortAddress(0x0001),
                ShortAddress(0x0002),
                ShortAddress(0x0003),
            ]
        );
    }

    // ── R22 rejoin parent selection (§3.6.1.4.2) ────────────

    const REJOIN_EPID: IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
    const OTHER_EPID: IeeeAddress = [0x99; 8];

    fn candidate(
        router_address: u16,
        lqi: u8,
        update_id: u8,
        depth: u8,
        extended_pan_id: IeeeAddress,
    ) -> NetworkDescriptor {
        NetworkDescriptor {
            extended_pan_id,
            pan_id: PanId(0x1234),
            logical_channel: 15,
            stack_profile: 2,
            zigbee_version: 2,
            beacon_order: 15,
            superframe_order: 15,
            // Rejoin is allowed into a closed network, so every candidate in
            // these tests has joining closed.
            permit_joining: false,
            router_capacity: true,
            end_device_capacity: true,
            update_id,
            lqi,
            router_address: ShortAddress(router_address),
            depth,
        }
    }

    /// `candidate` with the advertised beacon capacity bits overridden.
    fn candidate_with_capacity(
        router_address: u16,
        lqi: u8,
        update_id: u8,
        depth: u8,
        router_capacity: bool,
        end_device_capacity: bool,
    ) -> NetworkDescriptor {
        NetworkDescriptor {
            router_capacity,
            end_device_capacity,
            ..candidate(router_address, lqi, update_id, depth, REJOIN_EPID)
        }
    }

    /// LQI values whose derived link cost is unambiguous for these tests.
    const GOOD_LQI: u8 = 220; // cost 1
    const FAIR_LQI: u8 = 160; // cost 2
    const WEAK_LQI: u8 = 120; // cost 3
    const UNUSABLE_LQI: u8 = 100; // cost 5 — above MAX_PARENT_LINK_COST

    /// Criteria for an end device holding a *known-good* local update state.
    fn end_device_criteria(nwk_update_id: u8) -> RejoinParentCriteria {
        RejoinParentCriteria::new(REJOIN_EPID, Some(nwk_update_id), DeviceType::EndDevice)
    }

    /// Criteria for an end device that holds no authoritative update state.
    fn end_device_criteria_unknown() -> RejoinParentCriteria {
        RejoinParentCriteria::new(REJOIN_EPID, None, DeviceType::EndDevice)
    }

    fn addresses<const N: usize>(networks: &[NetworkDescriptor; N]) -> [u16; N] {
        core::array::from_fn(|index| networks[index].router_address.0)
    }

    #[test]
    fn update_id_comparison_wraps_in_both_directions() {
        // Forward wrap: 0x00 immediately follows 0xFF.
        assert!(nwk_update_id_is_newer(0x00, 0xFF));
        assert!(nwk_update_id_is_newer(0x02, 0xFE));
        // The same pair must not compare newer in the reverse direction.
        assert!(!nwk_update_id_is_newer(0xFF, 0x00));
        assert!(!nwk_update_id_is_newer(0xFE, 0x02));
        // Plain integer ordering would get both of these wrong.
        assert!(nwk_update_id_is_newer(0x80, 0x01));
        assert!(!nwk_update_id_is_newer(0x01, 0x80));
        // Equality is never "newer", in either direction.
        assert!(!nwk_update_id_is_newer(0x42, 0x42));
        // The ambiguous half-window (delta 128) is unordered: not newer and
        // stale in both directions, so it can never be selected.
        assert!(!nwk_update_id_is_newer(0x80, 0x00));
        assert!(!nwk_update_id_is_newer(0x00, 0x80));
        assert!(!nwk_update_id_is_current(0x80, 0x00));
        assert!(!nwk_update_id_is_current(0x00, 0x80));
        // "Current" accepts equal or newer.
        assert!(nwk_update_id_is_current(0x42, 0x42));
        assert!(nwk_update_id_is_current(0x00, 0xFF));
        assert!(!nwk_update_id_is_current(0xFF, 0x00));
    }

    #[test]
    fn rejoin_requires_capacity_for_the_requested_device_type() {
        let end_device = end_device_criteria(5);
        let router = RejoinParentCriteria::new(REJOIN_EPID, Some(5), DeviceType::Router);

        let both = candidate_with_capacity(0x0001, GOOD_LQI, 5, 1, true, true);
        let routers_only = candidate_with_capacity(0x0002, GOOD_LQI, 5, 1, true, false);
        let end_devices_only = candidate_with_capacity(0x0003, GOOD_LQI, 5, 1, false, true);
        let full = candidate_with_capacity(0x0004, GOOD_LQI, 5, 1, false, false);

        assert!(end_device.is_base_eligible(&both));
        assert!(!end_device.is_base_eligible(&routers_only));
        assert!(end_device.is_base_eligible(&end_devices_only));
        assert!(!end_device.is_base_eligible(&full));

        assert!(router.is_base_eligible(&both));
        assert!(router.is_base_eligible(&routers_only));
        assert!(!router.is_base_eligible(&end_devices_only));
        assert!(!router.is_base_eligible(&full));
    }

    #[test]
    fn rejoin_does_not_require_permit_joining() {
        // Every candidate built here has `permit_joining == false`; a closed
        // network must still be rejoinable (R22 §3.6.1.4.2).
        let criteria = end_device_criteria(5);
        let closed = candidate(0x0001, GOOD_LQI, 5, 1, REJOIN_EPID);
        assert!(!closed.permit_joining);
        assert!(criteria.is_base_eligible(&closed));

        let mut networks = [closed];
        assert_eq!(criteria.select(&mut networks), 1);
    }

    #[test]
    fn rejoin_rejects_stale_update_id_across_wrap() {
        // Local update id just after the wrap: 0xFF is one update behind us.
        let criteria = end_device_criteria(0x00);
        assert!(!criteria.is_base_eligible(&candidate(0x0001, GOOD_LQI, 0xFF, 1, REJOIN_EPID)));
        assert!(criteria.is_base_eligible(&candidate(0x0002, GOOD_LQI, 0x00, 1, REJOIN_EPID)));
        assert!(criteria.is_base_eligible(&candidate(0x0003, GOOD_LQI, 0x01, 1, REJOIN_EPID)));

        // Local update id just before the wrap: 0x00 is one update ahead.
        let criteria = end_device_criteria(0xFF);
        assert!(!criteria.is_base_eligible(&candidate(0x0001, GOOD_LQI, 0xFE, 1, REJOIN_EPID)));
        assert!(criteria.is_base_eligible(&candidate(0x0002, GOOD_LQI, 0xFF, 1, REJOIN_EPID)));
        assert!(criteria.is_base_eligible(&candidate(0x0003, GOOD_LQI, 0x00, 1, REJOIN_EPID)));
    }

    #[test]
    fn rejoin_rejects_other_networks_and_unusable_link_costs() {
        let criteria = end_device_criteria(0x05);

        // Another network is never a rejoin candidate.
        assert!(!criteria.is_base_eligible(&candidate(0x0001, GOOD_LQI, 0x05, 1, OTHER_EPID)));

        // Link cost above MAX_PARENT_LINK_COST is unusable — a hard gate, not
        // a ranking penalty.
        assert_eq!(
            criteria.link_cost(&candidate(0x0002, UNUSABLE_LQI, 0x05, 1, REJOIN_EPID)),
            None
        );
        assert!(!criteria.is_base_eligible(&candidate(0x0002, UNUSABLE_LQI, 0x05, 1, REJOIN_EPID)));
        // An unreported/zero LQI is not treated as a perfect link.
        assert!(!criteria.is_base_eligible(&candidate(0x0003, 0, 0x05, 1, REJOIN_EPID)));
        // The worst still-usable cost is accepted.
        assert_eq!(
            criteria.link_cost(&candidate(0x0004, WEAK_LQI, 0x05, 1, REJOIN_EPID)),
            Some(MAX_PARENT_LINK_COST)
        );
        assert!(criteria.is_base_eligible(&candidate(0x0004, WEAK_LQI, 0x05, 1, REJOIN_EPID)));
    }

    #[test]
    fn rejoin_keeps_only_the_most_recent_update_id() {
        let criteria = end_device_criteria(4);
        let mut networks = [
            // Base-eligible but one update behind the freshest beacon: still
            // "not stale", yet R22 keeps only the most recent update id.
            candidate(0x0001, GOOD_LQI, 4, 0, REJOIN_EPID),
            candidate(0x0002, WEAK_LQI, 5, 3, REJOIN_EPID),
            candidate(0x0003, FAIR_LQI, 5, 7, REJOIN_EPID),
        ];

        assert_eq!(criteria.most_recent_update_id(&networks), Some(5));
        assert!(criteria.is_base_eligible(&networks[0]));
        assert!(!criteria.is_suitable(&networks[0], 5));

        let suitable = criteria.select(&mut networks);

        assert_eq!(suitable, 2);
        assert_eq!(addresses(&networks), [0x0002, 0x0003, 0x0001]);
    }

    #[test]
    fn most_recent_update_id_is_deterministic_across_wrap() {
        let criteria = end_device_criteria(0xFE);
        let networks = [
            candidate(0x0001, GOOD_LQI, 0xFE, 0, REJOIN_EPID), // delta 0
            candidate(0x0002, GOOD_LQI, 0x01, 0, REJOIN_EPID), // delta 3 — freshest
            candidate(0x0003, GOOD_LQI, 0xFF, 0, REJOIN_EPID), // delta 1
            candidate(0x0004, GOOD_LQI, 0xFD, 0, REJOIN_EPID), // stale, ignored
            candidate(0x0005, GOOD_LQI, 0x02, 0, OTHER_EPID),  // other network
        ];

        assert_eq!(criteria.most_recent_update_id(&networks), Some(0x01));

        // Only stale / foreign / unusable candidates: nothing to select.
        let none = [
            candidate(0x0001, GOOD_LQI, 0xFD, 0, REJOIN_EPID),
            candidate(0x0002, UNUSABLE_LQI, 0x01, 0, REJOIN_EPID),
            candidate(0x0003, GOOD_LQI, 0x01, 0, OTHER_EPID),
        ];
        assert_eq!(criteria.most_recent_update_id(&none), None);
        let mut none = none;
        assert_eq!(criteria.select(&mut none), 0);
    }

    // ── Unknown local nwkUpdateId ───────────────────────────

    /// A device that holds no authoritative update state must not reject
    /// candidates as stale. With a fabricated reference of `0`, every ID in
    /// `0x81..=0xFF` would look stale and the device could never get back on
    /// its own network.
    #[test]
    fn unknown_local_update_id_never_rejects_a_candidate_as_stale() {
        let criteria = end_device_criteria_unknown();
        assert_eq!(criteria.nwk_update_id, None);

        for update_id in [0x00u8, 0x01, 0x7F, 0x80, 0x81, 0xFE, 0xFF] {
            let network = candidate(0x0001, GOOD_LQI, update_id, 1, REJOIN_EPID);
            assert!(
                criteria.update_id_is_acceptable(&network),
                "update id {update_id:#04X} must not be stale against an unknown local state"
            );
            assert!(criteria.is_base_eligible(&network));
            assert_eq!(criteria.freshness(&network), None);
        }

        // Every other gate still applies unchanged.
        assert!(!criteria.is_base_eligible(&candidate(0x0002, GOOD_LQI, 0x42, 1, OTHER_EPID)));
        assert!(!criteria.is_base_eligible(&candidate(0x0003, UNUSABLE_LQI, 0x42, 1, REJOIN_EPID)));
        assert!(!criteria.is_base_eligible(&candidate_with_capacity(
            0x0004, GOOD_LQI, 0x42, 1, true, false
        )));

        // The known-state gate is unchanged by all this.
        let known = end_device_criteria(0x00);
        assert!(!known.is_base_eligible(&candidate(0x0001, GOOD_LQI, 0xFF, 1, REJOIN_EPID)));
    }

    /// With no reference point the most recent ID is the fixed point of a
    /// forward-only pairwise fold in discovery order, and the selection still
    /// narrows to exactly one update ID.
    #[test]
    fn unknown_local_update_id_picks_a_deterministic_most_recent_across_wrap() {
        let criteria = end_device_criteria_unknown();

        // A chain that wraps: 0xFD -> 0xFE -> 0x02 are each strictly newer
        // than the one before, so the fold walks forward to 0x02.
        let networks = [
            candidate(0x0001, GOOD_LQI, 0xFD, 0, REJOIN_EPID),
            candidate(0x0002, GOOD_LQI, 0xFE, 0, REJOIN_EPID),
            candidate(0x0003, GOOD_LQI, 0x02, 0, REJOIN_EPID),
            // Older than the running answer: must not pull it back.
            candidate(0x0004, GOOD_LQI, 0xFF, 0, REJOIN_EPID),
        ];
        assert_eq!(criteria.most_recent_update_id(&networks), Some(0x02));

        // Discovery order does not change which ID wins when the set is
        // orderable end to end.
        let reordered = [
            candidate(0x0004, GOOD_LQI, 0xFF, 0, REJOIN_EPID),
            candidate(0x0003, GOOD_LQI, 0x02, 0, REJOIN_EPID),
            candidate(0x0001, GOOD_LQI, 0xFD, 0, REJOIN_EPID),
            candidate(0x0002, GOOD_LQI, 0xFE, 0, REJOIN_EPID),
        ];
        assert_eq!(criteria.most_recent_update_id(&reordered), Some(0x02));

        // Ambiguous half-window pair: neither is newer, so discovery order
        // decides — deterministically, and without inventing an ordering.
        let ambiguous = [
            candidate(0x0001, GOOD_LQI, 0x00, 0, REJOIN_EPID),
            candidate(0x0002, GOOD_LQI, 0x80, 0, REJOIN_EPID),
        ];
        assert_eq!(criteria.most_recent_update_id(&ambiguous), Some(0x00));
        let ambiguous_reversed = [
            candidate(0x0002, GOOD_LQI, 0x80, 0, REJOIN_EPID),
            candidate(0x0001, GOOD_LQI, 0x00, 0, REJOIN_EPID),
        ];
        assert_eq!(
            criteria.most_recent_update_id(&ambiguous_reversed),
            Some(0x80)
        );

        // Ineligible candidates never seed or move the answer.
        let with_noise = [
            candidate(0x0009, GOOD_LQI, 0x7F, 0, OTHER_EPID),
            candidate(0x000A, UNUSABLE_LQI, 0x7E, 0, REJOIN_EPID),
            candidate(0x0001, GOOD_LQI, 0xFD, 0, REJOIN_EPID),
            candidate(0x0003, GOOD_LQI, 0x02, 0, REJOIN_EPID),
        ];
        assert_eq!(criteria.most_recent_update_id(&with_noise), Some(0x02));

        // No eligible candidate at all is still "nothing to select".
        assert_eq!(
            criteria.most_recent_update_id(&[candidate(0x0001, GOOD_LQI, 0x02, 0, OTHER_EPID)]),
            None
        );
    }

    /// Unknown local state still narrows the scan to one update ID and then
    /// ranks by the normative minimum-depth rule.
    #[test]
    fn unknown_local_update_id_still_keeps_only_one_update_id() {
        let criteria = end_device_criteria_unknown();
        let mut networks = [
            candidate(0x0001, GOOD_LQI, 0xFE, 0, REJOIN_EPID), // older
            candidate(0x0002, WEAK_LQI, 0x01, 3, REJOIN_EPID), // newest
            candidate(0x0003, GOOD_LQI, 0x01, 1, REJOIN_EPID), // newest, shallower
            candidate(0x0004, GOOD_LQI, 0x00, 0, REJOIN_EPID), // older
        ];

        let suitable = criteria.select(&mut networks);

        assert_eq!(suitable, 2);
        assert_eq!(addresses(&networks), [0x0003, 0x0002, 0x0001, 0x0004]);
    }

    /// The criteria derived from the NIB carry the validity flag, so an
    /// un-commissioned NIB never claims to hold update state `0`.
    #[test]
    fn rejoin_criteria_report_unknown_update_id_from_a_factory_new_nib() {
        let mut nwk = NwkLayer::new(MockMac::new([3; 8]), DeviceType::EndDevice);
        nwk.nib.extended_pan_id = REJOIN_EPID;

        assert_eq!(nwk.nib.nwk_update_id(), None);
        assert_eq!(nwk.rejoin_parent_criteria().nwk_update_id, None);

        // A beacon that a fabricated local `0` would have called stale.
        let stale_looking = candidate(0x0001, GOOD_LQI, 0xF0, 1, REJOIN_EPID);
        assert!(
            nwk.rejoin_parent_criteria()
                .is_base_eligible(&stale_looking)
        );

        // Adopting an update state switches the gate on.
        nwk.nib.set_nwk_update_id(0x00);
        assert_eq!(nwk.rejoin_parent_criteria().nwk_update_id, Some(0x00));
        assert!(
            !nwk.rejoin_parent_criteria()
                .is_base_eligible(&stale_looking)
        );

        // And clearing it takes the gate back off.
        nwk.nib.clear_nwk_update_id();
        assert_eq!(nwk.rejoin_parent_criteria().nwk_update_id, None);
    }

    /// A restore that carries no update state must land in the unknown state,
    /// not in a known `0`.
    #[test]
    fn restoring_an_absent_update_id_leaves_it_unknown() {
        let mut nib = crate::nib::Nib::new();

        nib.restore_nwk_update_id(Some(0x2A));
        assert_eq!(nib.nwk_update_id(), Some(0x2A));
        assert!(nib.update_id_valid);

        nib.restore_nwk_update_id(None);
        assert_eq!(nib.nwk_update_id(), None);
        assert!(!nib.update_id_valid);
        // The raw byte is reset too, so nothing can read a stale value.
        assert_eq!(nib.update_id, 0);

        // A known 0 and an unknown state are distinguishable.
        nib.set_nwk_update_id(0);
        assert_eq!(nib.nwk_update_id(), Some(0));
    }

    #[test]
    fn rejoin_orders_suitable_candidates_by_minimum_depth_first() {
        let criteria = end_device_criteria(7);
        let mut networks = [
            candidate(0x0001, GOOD_LQI, 7, 4, REJOIN_EPID),
            candidate(0x0002, WEAK_LQI, 7, 0, REJOIN_EPID),
            candidate(0x0003, GOOD_LQI, 7, 2, REJOIN_EPID),
            candidate(0x0004, FAIR_LQI, 7, 1, REJOIN_EPID),
        ];

        let suitable = criteria.select(&mut networks);

        // Depth is normative and outranks link cost: the deepest candidate
        // loses even with the best link, and the shallowest wins with the
        // worst still-usable link.
        assert_eq!(suitable, 4);
        assert_eq!(addresses(&networks), [0x0002, 0x0004, 0x0003, 0x0001]);
    }

    #[test]
    fn rejoin_ties_at_equal_depth_are_deterministic() {
        let criteria = end_device_criteria(3);
        // Implementation tie-break 1: lower link cost at equal depth.
        let mut networks = [
            candidate(0x0001, WEAK_LQI, 3, 2, REJOIN_EPID),
            candidate(0x0002, GOOD_LQI, 3, 2, REJOIN_EPID),
            candidate(0x0003, FAIR_LQI, 3, 2, REJOIN_EPID),
        ];
        assert_eq!(criteria.select(&mut networks), 3);
        assert_eq!(addresses(&networks), [0x0002, 0x0003, 0x0001]);

        // Implementation tie-break 2: scan order when nothing else differs.
        let identical = |address: u16| candidate(address, GOOD_LQI, 3, 2, REJOIN_EPID);
        let mut networks = [identical(0x0003), identical(0x0001), identical(0x0002)];
        assert_eq!(criteria.select(&mut networks), 3);
        assert_eq!(addresses(&networks), [0x0003, 0x0001, 0x0002]);

        // Implementation tie-break 3: previous parent, ahead of scan order but
        // behind depth and cost.
        let criteria = criteria.with_previous_parent(ShortAddress(0x0002));
        let mut networks = [identical(0x0003), identical(0x0001), identical(0x0002)];
        assert_eq!(criteria.select(&mut networks), 3);
        assert_eq!(addresses(&networks), [0x0002, 0x0003, 0x0001]);

        // The previous parent never beats the normative depth rule.
        let mut networks = [
            candidate(0x0002, GOOD_LQI, 3, 3, REJOIN_EPID),
            candidate(0x0001, WEAK_LQI, 3, 1, REJOIN_EPID),
        ];
        assert_eq!(criteria.select(&mut networks), 2);
        assert_eq!(addresses(&networks), [0x0001, 0x0002]);
    }

    #[test]
    fn rejoin_selection_keeps_unsuitable_candidates_at_the_tail() {
        let criteria = end_device_criteria(4);
        let mut networks = [
            candidate(0x0001, UNUSABLE_LQI, 4, 1, REJOIN_EPID), // link cost
            candidate(0x0002, GOOD_LQI, 3, 1, REJOIN_EPID),     // stale
            candidate(0x0003, GOOD_LQI, 4, 3, REJOIN_EPID),     // suitable
            candidate(0x0004, GOOD_LQI, 4, 1, OTHER_EPID),      // other network
            candidate(0x0005, FAIR_LQI, 4, 1, REJOIN_EPID),     // suitable
            candidate_with_capacity(0x0006, GOOD_LQI, 4, 0, true, false), // no ED capacity
        ];

        let suitable = criteria.select(&mut networks);

        // 0x0005 is shallower than 0x0003, so depth puts it first.
        assert_eq!(suitable, 2);
        assert_eq!(
            addresses(&networks),
            [0x0005, 0x0003, 0x0001, 0x0002, 0x0004, 0x0006]
        );
        let most_recent = criteria
            .most_recent_update_id(&networks)
            .expect("a suitable candidate exists");
        assert!(
            networks[..suitable]
                .iter()
                .all(|network| criteria.is_suitable(network, most_recent))
        );
        assert!(
            networks[suitable..]
                .iter()
                .all(|network| !criteria.is_suitable(network, most_recent))
        );
    }

    #[test]
    fn rejoin_criteria_track_the_nib() {
        let mut nwk = NwkLayer::new(
            MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]),
            DeviceType::EndDevice,
        );
        nwk.nib.extended_pan_id = REJOIN_EPID;
        nwk.nib.set_nwk_update_id(9);

        // No parent yet — nothing to prefer on a tie.
        assert_eq!(nwk.rejoin_parent_criteria(), end_device_criteria(9));
        assert_eq!(
            nwk.rejoin_parent_criteria().device_type,
            DeviceType::EndDevice
        );

        nwk.nib.parent_address = ShortAddress(0x1234);
        assert_eq!(
            nwk.rejoin_parent_criteria(),
            end_device_criteria(9).with_previous_parent(ShortAddress(0x1234))
        );

        // A router rejoins against the router capacity bit instead.
        let mut router = NwkLayer::new(MockMac::new([2; 8]), DeviceType::Router);
        router.nib.extended_pan_id = REJOIN_EPID;
        router.nib.set_nwk_update_id(9);
        assert_eq!(
            router.rejoin_parent_criteria(),
            RejoinParentCriteria::new(REJOIN_EPID, Some(9), DeviceType::Router)
        );
    }

    #[test]
    fn zigbee_capability_bytes_match_reference_stack() {
        assert_eq!(
            zigbee_capability_info(DeviceType::EndDevice, false).to_byte(),
            0x80
        );
        assert_eq!(
            zigbee_capability_info(DeviceType::EndDevice, true).to_byte(),
            0x88
        );
        assert_eq!(
            zigbee_capability_info(DeviceType::Router, true).to_byte(),
            0x8E
        );
    }

    #[test]
    fn rejoin_leave_preserves_previous_network_identity() {
        let mut nwk = NwkLayer::new(
            MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]),
            DeviceType::EndDevice,
        );
        nwk.joined = true;
        nwk.nib.network_address = ShortAddress(0x1234);
        nwk.nib.pan_id = PanId(0xABCD);
        nwk.nib.parent_address = ShortAddress(0x0000);
        nwk.nib.logical_channel = 15;
        nwk.nib.extended_pan_id = [8, 7, 6, 5, 4, 3, 2, 1];

        nwk.finish_leave(true);

        assert!(!nwk.joined);
        assert_eq!(nwk.nib.network_address, ShortAddress(0x1234));
        assert_eq!(nwk.nib.pan_id, PanId(0xABCD));
        assert_eq!(nwk.nib.parent_address, ShortAddress(0x0000));
        assert_eq!(nwk.nib.logical_channel, 15);
        assert_eq!(nwk.nib.extended_pan_id, [8, 7, 6, 5, 4, 3, 2, 1]);
    }

    #[test]
    fn final_leave_clears_previous_network_identity() {
        let mut nwk = NwkLayer::new(
            MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]),
            DeviceType::EndDevice,
        );
        nwk.joined = true;
        nwk.nib.network_address = ShortAddress(0x1234);
        nwk.nib.pan_id = PanId(0xABCD);
        nwk.nib.parent_address = ShortAddress(0x0000);
        nwk.nib.logical_channel = 15;
        nwk.nib.extended_pan_id = [8, 7, 6, 5, 4, 3, 2, 1];

        nwk.finish_leave(false);

        assert!(!nwk.joined);
        assert_eq!(nwk.nib.network_address, ShortAddress(0xFFFF));
        assert_eq!(nwk.nib.pan_id, PanId(0xFFFF));
        assert_eq!(nwk.nib.parent_address, ShortAddress(0xFFFF));
        assert_eq!(nwk.nib.logical_channel, 0);
        assert_eq!(nwk.nib.extended_pan_id, [0; 8]);
    }

    #[test]
    fn sleepy_secure_rejoin_unicasts_and_polls_selected_parent() {
        const DEVICE_IEEE: IeeeAddress = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
        const PARENT_IEEE: IeeeAddress = [0x00, 0x12, 0x4B, 0x00, 0x01, 0xAA, 0xBB, 0xCC];
        const KEY: [u8; 16] = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD,
            0xEE, 0xFF,
        ];
        const PAN_ID: PanId = PanId(0xDFE9);
        const OLD_ADDRESS: ShortAddress = ShortAddress(0x07D6);
        const NEW_ADDRESS: ShortAddress = ShortAddress(0x1234);
        const DIRECT_ADDRESS: ShortAddress = ShortAddress(0x2345);
        const LATE_ADDRESS: ShortAddress = ShortAddress(0x3456);
        const PARENT_ADDRESS: ShortAddress = ShortAddress(0xBA0F);

        fn block_on<F: core::future::Future>(future: F) -> F::Output {
            use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
            use std::boxed::Box;

            fn no_op(_: *const ()) {}
            fn clone(_: *const ()) -> RawWaker {
                RawWaker::new(core::ptr::null(), &VTABLE)
            }
            static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);

            let waker = unsafe { Waker::from_raw(clone(core::ptr::null())) };
            let mut context = Context::from_waker(&waker);
            let mut future = Box::pin(future);
            loop {
                if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                    return output;
                }
                std::thread::yield_now();
            }
        }

        let network = NetworkDescriptor {
            extended_pan_id: [0xAA; 8],
            pan_id: PAN_ID,
            logical_channel: 15,
            stack_profile: 2,
            zigbee_version: 2,
            beacon_order: 15,
            superframe_order: 15,
            permit_joining: false,
            router_capacity: true,
            end_device_capacity: true,
            update_id: 0,
            lqi: 200,
            router_address: PARENT_ADDRESS,
            depth: 1,
        };

        let build_response =
            |frame_counter: u32, destination: ShortAddress, address: ShortAddress, status: u8| {
                let response_header = NwkHeader {
                    frame_control: NwkFrameControl {
                        frame_type: NwkFrameType::Command as u8,
                        protocol_version: 2,
                        discover_route: 0,
                        multicast: false,
                        security: true,
                        source_route: false,
                        dst_ieee_present: true,
                        src_ieee_present: true,
                        end_device_initiator: false,
                    },
                    dst_addr: destination,
                    src_addr: PARENT_ADDRESS,
                    radius: 1,
                    seq_number: 0x42,
                    dst_ieee: Some(DEVICE_IEEE),
                    src_ieee: Some(PARENT_IEEE),
                    multicast_control: None,
                    source_route: None,
                };
                let response_security = crate::security::NwkSecurityHeader {
                    security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
                    frame_counter,
                    source_address: PARENT_IEEE,
                    key_seq_number: 0,
                };
                let mut response_buf = [0u8; 128];
                let response_header_len = response_header.serialize(&mut response_buf);
                let response_security_len =
                    response_security.serialize(&mut response_buf[response_header_len..]);
                let response_aad_len = response_header_len + response_security_len;
                let response_plaintext = [0x07, address.0 as u8, (address.0 >> 8) as u8, status];
                let crypto = crate::security::NwkSecurity::new();
                let encrypted = crypto
                    .encrypt(
                        &response_buf[..response_aad_len],
                        &response_plaintext,
                        &KEY,
                        &response_security,
                    )
                    .unwrap();
                response_buf[response_aad_len..response_aad_len + encrypted.len()]
                    .copy_from_slice(&encrypted);
                response_buf[response_header_len] &= !0x07;
                MacFrame::from_slice(&response_buf[..response_aad_len + encrypted.len()]).unwrap()
            };

        let mut mac = MockMac::new(DEVICE_IEEE);
        mac.enqueue_poll_response(build_response(4, OLD_ADDRESS, ShortAddress(0x2222), 0x00));
        mac.enqueue_poll_response(build_response(6, OLD_ADDRESS, NEW_ADDRESS, 0x01));
        let mut nwk = crate::NwkLayer::new(mac, DeviceType::EndDevice);
        nwk.set_rx_on_when_idle(false);
        {
            let nib = nwk.nib_mut();
            nib.extended_pan_id = network.extended_pan_id;
            nib.pan_id = PanId(0x1234);
            nib.network_address = OLD_ADDRESS;
            nib.logical_channel = network.logical_channel;
            nib.ieee_address = DEVICE_IEEE;
            nib.security_enabled = true;
            nib.active_key_seq_number = 0;
            nib.outgoing_frame_counter = 0x100;
            nib.outgoing_frame_counter_limit = 0x200;
        }
        nwk.security_mut().set_network_key(KEY, 0);
        nwk.security_mut().commit_frame_counter(&PARENT_IEEE, 5);

        // This NIB was never given an authoritative update state, so the
        // staleness gate is off and the candidate is attempted on its merits.
        assert_eq!(nwk.nib().nwk_update_id(), None);

        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)),
            Err(NwkStatus::NotPermitted)
        );
        assert_eq!(
            nwk.nib().nwk_update_id(),
            None,
            "a refused rejoin must not make the candidate's update id authoritative"
        );
        assert_eq!(nwk.mac().poll_count(), 2);
        assert!(!nwk.security().check_frame_counter(&PARENT_IEEE, 6));
        assert!(nwk.security().check_frame_counter(&PARENT_IEEE, 7));

        nwk.mac_mut()
            .enqueue_poll_response(build_response(7, OLD_ADDRESS, NEW_ADDRESS, 0x00));
        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)).unwrap(),
            NEW_ADDRESS
        );
        assert_eq!(nwk.mac().poll_count(), 3);
        assert_eq!(nwk.nib().pan_id, PAN_ID);
        assert_eq!(
            nwk.nib().nwk_update_id(),
            Some(network.update_id),
            "the parent accepted us, so the update state we rejoined against \
             is now authoritative"
        );
        assert!(!nwk.security().check_frame_counter(&PARENT_IEEE, 7));
        assert!(nwk.security().check_frame_counter(&PARENT_IEEE, 8));

        let tx = &nwk.mac().tx_history()[0];
        assert_eq!(
            tx.dst,
            MacAddress::Short(PAN_ID, PARENT_ADDRESS),
            "rejoin must target the selected prospective parent"
        );
        assert!(tx.ack_requested);

        let (request_header, consumed) = NwkHeader::parse(tx.payload.as_slice()).unwrap();
        assert_eq!(request_header.dst_addr, PARENT_ADDRESS);
        assert_eq!(request_header.src_addr, OLD_ADDRESS);
        assert!(!request_header.frame_control.end_device_initiator);
        assert!(request_header.frame_control.security);
        assert_eq!(request_header.src_ieee, Some(DEVICE_IEEE));
        assert_eq!(tx.payload.as_slice()[consumed] & 0x07, 0);

        nwk.set_rx_on_when_idle(true);
        nwk.mac_mut().enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PAN_ID, PARENT_ADDRESS),
            dst_address: MacAddress::Short(PAN_ID, NEW_ADDRESS),
            lqi: 100,
            payload: MacFrame::from_slice(&[0x00]).unwrap(),
            security_use: false,
        });
        nwk.mac_mut().enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PAN_ID, PARENT_ADDRESS),
            dst_address: MacAddress::Short(PAN_ID, NEW_ADDRESS),
            lqi: 200,
            payload: build_response(8, NEW_ADDRESS, DIRECT_ADDRESS, 0x00),
            security_use: true,
        });
        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)).unwrap(),
            DIRECT_ADDRESS
        );
        assert_eq!(
            nwk.mac().poll_count(),
            3,
            "RX-on devices must wait directly without polling"
        );
        assert!(!nwk.security().check_frame_counter(&PARENT_IEEE, 8));
        assert!(nwk.security().check_frame_counter(&PARENT_IEEE, 9));

        nwk.mac_mut().enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PAN_ID, PARENT_ADDRESS),
            dst_address: MacAddress::Short(PAN_ID, DIRECT_ADDRESS),
            lqi: 200,
            payload: build_response(9, DIRECT_ADDRESS, LATE_ADDRESS, 0x00),
            security_use: true,
        });
        nwk.mac_mut().set_rx_delay_us(REJOIN_RESPONSE_WAIT_US);
        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)),
            Err(NwkStatus::NoNetworks)
        );
        assert_eq!(nwk.nib().network_address, DIRECT_ADDRESS);
        assert!(
            nwk.security().check_frame_counter(&PARENT_IEEE, 9),
            "a response at or after the deadline must not be authenticated"
        );

        nwk.set_rx_on_when_idle(false);
        nwk.mac_mut()
            .enqueue_poll_response(build_response(9, DIRECT_ADDRESS, LATE_ADDRESS, 0x00));
        nwk.mac_mut().set_poll_delay_us(REJOIN_RESPONSE_WAIT_US);
        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)),
            Err(NwkStatus::NoNetworks)
        );
        assert_eq!(nwk.nib().network_address, DIRECT_ADDRESS);
        assert!(
            nwk.security().check_frame_counter(&PARENT_IEEE, 9),
            "a late indirect response must not be authenticated"
        );
    }
}
