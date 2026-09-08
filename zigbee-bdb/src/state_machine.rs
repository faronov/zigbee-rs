//! BDB initialization and commissioning state machine (BDB v3.0.1 §§7–8).
//!
//! The state machine orchestrates the four commissioning methods in priority
//! order: Touchlink (when explicitly enabled) → Steering → Formation →
//! Finding & Binding.
//!
//! ```text
//!                         ┌──────────┐
//!              ┌─────────►│   Idle   │◄────────────────┐
//!              │          └────┬─────┘                  │
//!              │               │ commission()           │
//!              │          ┌────▼──────────┐             │
//!              │          │ Initializing  │             │
//!              │          └────┬──────────┘             │
//!              │               │                        │
//!              │       ┌───────▼────────┐               │
//!              │  TL?  │   Touchlink    │──► fail ──┐   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  NS?  │ NetworkSteering│──► fail ──┤   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  NF?  │NetworkFormation│──► fail ──┤   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  FB?  │FindingBinding  │──► fail ──┘   │
//!              │       └───────┬────────┘               │
//!              │               │                        │
//!              └───────────────┴────────────────────────┘
//! ```

use zigbee_mac::MacDriver;
#[cfg(any(not(feature = "end-device"), feature = "router"))]
use zigbee_nwk::DeviceType;

use crate::{BdbLayer, BdbStatus};

// ── Commissioning mode bitmask ──────────────────────────────

/// Bitmask of enabled commissioning methods (BDB spec Table 5).
///
/// The application sets this before calling [`BdbLayer::commission`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissioningMode(pub u8);

impl CommissioningMode {
    /// Network Steering (BDB §§8.1–8.2) — join or open permit joining.
    pub const STEERING: Self = Self(1 << 0);
    /// Network Formation (BDB §8.3) — create a new PAN.
    pub const FORMATION: Self = Self(1 << 1);
    /// Finding & Binding (BDB §§8.4–8.5).
    pub const FINDING_BINDING: Self = Self(1 << 2);
    /// Touchlink commissioning (BDB §§8.6–8.7).
    pub const TOUCHLINK: Self = Self(1 << 3);
    /// All methods enabled
    pub const ALL: Self = Self(0x0F);

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) != 0
    }

    pub const fn or(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    pub const fn empty() -> Self {
        Self(0)
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

// ── BDB state ───────────────────────────────────────────────

/// Current state of the BDB commissioning state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BdbState {
    /// No commissioning in progress.
    Idle,
    /// Running BDB initialisation (spec §7.1).
    Initializing,
    /// Network Steering is in progress (spec §§8.1–8.2).
    NetworkSteering,
    /// Network Formation is in progress (spec §8.3).
    NetworkFormation,
    /// Finding & Binding is in progress (spec §§8.4–8.5).
    FindingBinding,
    /// Touchlink commissioning is in progress (spec §§8.6–8.7).
    Touchlink,
}

// ── State machine implementation ────────────────────────────

/// An end-device image does not link coordinator network formation.
///
/// Keep the public BDB entry point so generic downstream application code gets
/// an explicit `NotPermitted` result rather than a configuration-dependent
/// missing method.
#[cfg(all(
    feature = "end-device",
    not(feature = "formation"),
    not(feature = "router"),
    not(test)
))]
impl<M: MacDriver> BdbLayer<M> {
    pub async fn network_formation(&mut self) -> Result<(), BdbStatus> {
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::NotPermitted;
        Err(BdbStatus::NotPermitted)
    }
}

impl<M: MacDriver> BdbLayer<M> {
    /// BDB initialisation procedure (BDB spec §7.1).
    ///
    /// Must be called once after power-on/reset before any commissioning.
    /// Sets up the device-type–dependent commissioning capabilities and
    /// optionally restores network state from NV storage.
    pub fn initialize(&mut self) -> Result<(), BdbStatus> {
        self.state = BdbState::Initializing;
        log::info!("[BDB] Initializing…");

        // Reset lower layers
        if self.zdo.nlme_reset(false).is_err() {
            self.state = BdbState::Idle;
            return Err(BdbStatus::NotPermitted);
        }

        // Network steering is mandatory. Formation depends on the logical
        // type; optional F&B and Touchlink capabilities must be explicitly
        // enabled by the application before initialization.
        let device_type = self.zdo.nwk().device_type();
        #[cfg(any(feature = "finding-binding", feature = "touchlink", test))]
        let requested_capabilities = self.attributes.node_commissioning_capability;
        #[cfg(any(
            not(feature = "end-device"),
            feature = "finding-binding",
            feature = "touchlink",
            test
        ))]
        let mut cap = CommissioningMode::STEERING;
        #[cfg(not(any(
            not(feature = "end-device"),
            feature = "finding-binding",
            feature = "touchlink",
            test
        )))]
        let cap = CommissioningMode::STEERING;
        #[cfg(any(not(feature = "end-device"), feature = "router"))]
        if device_type == DeviceType::Coordinator {
            cap = cap.or(CommissioningMode::FORMATION);
        }
        #[cfg(any(feature = "finding-binding", test))]
        if requested_capabilities.contains(CommissioningMode::FINDING_BINDING) {
            cap = cap.or(CommissioningMode::FINDING_BINDING);
        }
        #[cfg(feature = "touchlink")]
        if requested_capabilities.contains(CommissioningMode::TOUCHLINK) {
            cap = cap.or(CommissioningMode::TOUCHLINK);
        }
        self.attributes.node_commissioning_capability = cap;

        // Sync on-network state with NWK layer
        self.attributes.node_is_on_a_network = self.zdo.nwk().is_joined();

        self.state = BdbState::Idle;
        log::info!(
            "[BDB] Initialized (type={:?}, cap=0x{:02X})",
            device_type,
            cap.0
        );
        Ok(())
    }

    /// Top-level commissioning dispatcher (BDB spec §8).
    ///
    /// Runs each enabled commissioning method in the spec-defined order:
    /// 1. Touchlink (if enabled)
    /// 2. Network Steering (if enabled)
    /// 3. Network Formation (if enabled and coordinator-capable)
    /// 4. Finding & Binding (if enabled and on a network)
    ///
    /// Returns `Ok` once at least one method has started successfully. Network
    /// Steering returns at network-up; when it arms the unique-TCLK exchange,
    /// the caller must drive [`crate::BdbLayer::advance_tclk_exchange`] before
    /// starting later commissioning methods.
    pub async fn commission(&mut self) -> Result<(), BdbStatus> {
        let mode = self.attributes.commissioning_mode;
        let cap = self.attributes.node_commissioning_capability;
        // Gate requested mode by the BDB Table 5 capability bitmap.
        let effective = CommissioningMode(mode.0 & cap.0);
        log::info!(
            "[BDB] Commissioning start (requested=0x{:02X}, cap=0x{:02X}, effective=0x{:02X})",
            mode.0,
            cap.0,
            effective.0,
        );

        if effective.is_empty() {
            log::warn!("[BDB] No commissioning methods available for this device type");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        }

        let mut any_success = false;
        let mut last_error = None;

        // ── 1. Touchlink ────────────────────────────────────
        #[cfg(feature = "touchlink")]
        if effective.contains(CommissioningMode::TOUCHLINK) {
            self.state = BdbState::Touchlink;
            match self.touchlink_commissioning().await {
                Ok(()) => {
                    log::info!("[BDB] Touchlink succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Touchlink failed: {:?}", e);
                    last_error = Some(e);
                }
            }
        }

        // ── 2. Network Steering ─────────────────────────────
        if effective.contains(CommissioningMode::STEERING) {
            self.state = BdbState::NetworkSteering;
            match self.network_steering().await {
                Ok(()) => {
                    log::info!("[BDB] Network Steering reached network-up");
                    any_success = true;
                    #[cfg(feature = "centralized-tclk")]
                    if self.tclk_exchange_active() {
                        self.state = BdbState::Idle;
                        return Ok(());
                    }
                }
                Err(e) => {
                    log::warn!("[BDB] Network Steering failed: {:?}", e);
                    last_error = Some(e);
                }
            }
        }

        // ── 3. Network Formation ────────────────────────────
        #[cfg(any(not(feature = "end-device"), feature = "router"))]
        if effective.contains(CommissioningMode::FORMATION) {
            self.state = BdbState::NetworkFormation;
            match self.network_formation().await {
                Ok(()) => {
                    log::info!("[BDB] Network Formation succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Network Formation failed: {:?}", e);
                    last_error = Some(e);
                }
            }
        }

        // ── 4. Finding & Binding ────────────────────────────
        #[cfg(any(feature = "finding-binding", test))]
        if effective.contains(CommissioningMode::FINDING_BINDING) {
            self.state = BdbState::FindingBinding;
            match self.finding_binding_initiator(1).await {
                Ok(()) => {
                    log::info!("[BDB] Finding & Binding succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Finding & Binding failed: {:?}", e);
                    last_error = Some(e);
                }
            }
        }

        self.state = BdbState::Idle;

        if any_success {
            Ok(())
        } else {
            Err(last_error.unwrap_or(BdbStatus::SteeringFailure))
        }
    }

    /// BDB factory reset procedure (BDB spec §9.5).
    ///
    /// Performs a full factory reset:
    /// 1. Leave the current network (if joined)
    /// 2. Clear all NWK, APS, and BDB state
    /// 3. Reset all BDB attributes to defaults
    ///
    /// After factory reset the device is in a "fresh out of box" state
    /// and must be commissioned again.
    pub async fn factory_reset(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB] Factory reset…");
        self.state = BdbState::Initializing;

        // Step 1: Leave the network if we are on one
        if self.attributes.node_is_on_a_network {
            let _ = self.zdo.nwk_mut().nlme_leave(false).await;
        }

        // Step 2: Reset lower layers (NWK + MAC) — clears neighbor table,
        // security material, routing table, frame counters
        let _ = self.zdo.nlme_reset(false);

        // Step 3: Clear APS state — binding table, group table, key table
        self.zdo.aps_mut().binding_table_mut().clear();
        self.zdo.aps_mut().group_table_mut().clear();
        self.zdo.aps_mut().security_mut().clear_keys();
        self.zdo.aps_mut().cancel_all_ack_tracking();

        // Step 4: Reset all BDB attributes to defaults
        self.reset_attributes();

        log::info!("[BDB] Factory reset complete — device is in fresh state");
        Ok(())
    }

    /// Leave the current network and immediately attempt to rejoin.
    ///
    /// Useful for recovering from communication problems — performs
    /// a clean leave followed by rejoin with the stored NWK key.
    pub async fn leave_and_rejoin(&mut self) -> Result<(), BdbStatus> {
        if !self.attributes.node_is_on_a_network {
            return Err(BdbStatus::NotOnNetwork);
        }

        log::info!("[BDB] Leave-and-rejoin…");

        // Remember key material before leaving (leave clears NWK state)
        let channel = self.zdo.nwk().nib().logical_channel;
        let _epid = self.zdo.nwk().nib().extended_pan_id;

        // Leave (but keep BDB on-network flag so rejoin knows we had a network)
        let _ = self.zdo.nwk_mut().nlme_leave(false).await;

        // The device still considers itself "on a network" for rejoin purposes
        // (node_is_on_a_network stays true so rejoin() works)

        // Attempt rejoin on the last-known channel first
        let result = self.rejoin().await;

        if result.is_err() {
            // Full rejoin failed — try steering from scratch
            log::warn!("[BDB] Leave-and-rejoin: rejoin failed, trying full steering");
            self.attributes.node_is_on_a_network = false;
            // Restore last-known channel in primary set for targeted scan
            self.attributes.primary_channel_set = zigbee_types::ChannelMask(1u32 << channel);
            return self.network_steering().await;
        }

        result
    }

    /// BDB rejoin procedure — attempt to rejoin the previous network using
    /// the stored NWK key (BDB spec §7.1 steps 4–5).
    ///
    /// Call this when the device loses its parent or detects network loss.
    /// It performs:
    /// 1. NWK discovery on the last-known channel
    /// 2. NLME-JOIN with Rejoin method (uses stored NWK key)
    /// 3. Device announce
    ///
    /// Falls back to full steering if rejoin fails.
    pub async fn rejoin(&mut self) -> Result<(), BdbStatus> {
        if self.rejoin_previous_network().await.is_ok() {
            return Ok(());
        }

        log::warn!("[BDB] Secured rejoin failed — falling back to steering");
        self.attributes.node_is_on_a_network = false;
        self.state = BdbState::Idle;
        self.network_steering().await
    }

    /// Rejoin only the previously commissioned network using the stored NWK
    /// key. Failure preserves the stored credentials but rolls the live NWK
    /// and BDB joined flags back to off-network, and never falls back to
    /// factory-new steering.
    pub async fn rejoin_previous_network(&mut self) -> Result<(), BdbStatus> {
        let mut volatile_commit = |_| true;
        self.rejoin_previous_network_with_replay_commit(&mut volatile_commit)
            .await?;
        let nwk_addr = self.zdo.nwk().nib().network_address;
        let ieee = self.zdo.nwk().nib().ieee_address;
        let _ = self.zdo.device_annce(nwk_addr, ieee).await;
        Ok(())
    }

    /// Rejoin while committing the secured response replay floor before the
    /// new parent relationship is accepted.
    pub async fn rejoin_previous_network_with_replay_commit<F>(
        &mut self,
        replay_commit: &mut F,
    ) -> Result<(), BdbStatus>
    where
        F: FnMut(zigbee_nwk::security::NwkReplayCounter) -> bool,
    {
        self.rejoin_previous_network_mode(replay_commit, false)
            .await
    }

    /// Perform an unsecured NWK rejoin on a centralized network, then wait
    /// for the current network key under APS Trust Center link-key security.
    ///
    /// The caller must durably reserve the received key/counters before
    /// broadcasting Device_annce or allowing normal traffic.
    pub async fn trust_center_rejoin_previous_network(&mut self) -> Result<(), BdbStatus> {
        if self.attributes.node_join_link_key_type.is_distributed() {
            return Err(BdbStatus::NotPermitted);
        }
        let mut volatile_commit = |_| true;
        self.rejoin_previous_network_mode(&mut volatile_commit, true)
            .await
    }

    async fn rejoin_previous_network_mode<F>(
        &mut self,
        replay_commit: &mut F,
        trust_center_rejoin: bool,
    ) -> Result<(), BdbStatus>
    where
        F: FnMut(zigbee_nwk::security::NwkReplayCounter) -> bool,
    {
        if !self.attributes.node_is_on_a_network {
            return Err(BdbStatus::NotOnNetwork);
        }

        self.state = BdbState::NetworkSteering;
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::InProgress;
        // A restored network record is only authorization to attempt the
        // over-the-air rejoin. It must not remain visible as a live joined
        // relationship while the candidate parent and (for TC rejoin) current
        // network key are still provisional.
        self.zdo.nwk_mut().set_joined(false);
        self.attributes.node_is_on_a_network = false;
        log::info!("[BDB] Attempting rejoin on previous network…");

        let nib = self.zdo.nwk().nib();
        let channel = nib.logical_channel;
        let channel_mask = zigbee_types::ChannelMask(1u32 << channel);

        self.zdo.nwk_mut().reset_rejoin_diagnostics();
        let mut networks = match self.zdo.nlme_network_discovery(channel_mask, 3).await {
            Ok(n) => n,
            Err(_) => {
                log::warn!("[BDB] Rejoin: no networks found on channel {}", channel);
                self.rollback_failed_rejoin(
                    crate::attributes::BdbCommissioningStatus::NoScanResponse,
                );
                return Err(BdbStatus::NoScanResponse);
            }
        };
        // R22 §3.6.1.4.2 — keep only candidates on our network that advertise
        // capacity for our device type, are not stale, carry the most recent
        // nwkUpdateId seen in this scan and have a link cost of at most 3.
        // Suitable parents are then tried at minimum depth first.
        let suitable = self.zdo.nwk().select_rejoin_parents(&mut networks);
        if suitable == 0 {
            log::warn!(
                "[BDB] Rejoin: {} beacon(s) on channel {}, none suitable as parent",
                networks.len(),
                channel,
            );
            self.rollback_failed_rejoin(crate::attributes::BdbCommissioningStatus::NoNetwork);
            return Err(BdbStatus::SteeringFailure);
        }

        for network in &networks[..suitable] {
            log::info!(
                "[BDB] Rejoin: trying PAN 0x{:04X} ch {} via 0x{:04X} (update_id={} LQI={} depth={})",
                network.pan_id.0,
                network.logical_channel,
                network.router_address.0,
                network.update_id,
                network.lqi,
                network.depth,
            );

            let rejoin = if trust_center_rejoin {
                self.zdo
                    .nlme_trust_center_rejoin_with_replay_commit(network, replay_commit)
                    .await
            } else {
                self.zdo
                    .nlme_rejoin_with_replay_commit(network, replay_commit)
                    .await
            };
            match rejoin {
                Ok(nwk_addr) => {
                    if trust_center_rejoin {
                        // The old network key may be stale. Keep the APS link
                        // key, but remove live NWK material so an unsecured
                        // Transport-Key is treated as authorization rather
                        // than as an unauthenticated key update.
                        let nwk = self.zdo.aps_mut().nwk_mut();
                        nwk.security_mut().clear_network_keys();
                        nwk.nib_mut().security_enabled = false;

                        if !self.wait_for_transport_key().await {
                            log::warn!(
                                "[BDB] Trust Center rejoin accepted but no current network key arrived"
                            );
                            self.rollback_failed_rejoin(
                                crate::attributes::BdbCommissioningStatus::NoNetwork,
                            );
                            return Err(BdbStatus::SteeringFailure);
                        }
                        // Keep the persisted Table 6 regime authoritative.
                        // A rejoin may use a later generated unique TCLK, for
                        // which Table 6 has no separate value.
                        let _ = self.zdo.aps_mut().take_network_key_join_method();
                    }

                    log::info!("[BDB] Rejoin successful as 0x{:04X}", nwk_addr.0);
                    self.attributes.node_is_on_a_network = true;
                    self.attributes.commissioning_status =
                        crate::attributes::BdbCommissioningStatus::Success;
                    self.state = BdbState::Idle;
                    return Ok(());
                }
                Err(e) => {
                    log::warn!(
                        "[BDB] Rejoin failed on PAN 0x{:04X}: {:?}",
                        network.pan_id.0,
                        e
                    );
                }
            }
        }

        log::warn!("[BDB] Previous network did not accept secured rejoin");
        self.rollback_failed_rejoin(crate::attributes::BdbCommissioningStatus::NoNetwork);
        Err(BdbStatus::SteeringFailure)
    }

    fn rollback_failed_rejoin(
        &mut self,
        commissioning_status: crate::attributes::BdbCommissioningStatus,
    ) {
        self.zdo.nwk_mut().set_joined(false);
        self.attributes.node_is_on_a_network = false;
        self.attributes.commissioning_status = commissioning_status;
        self.state = BdbState::Idle;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_mac::mock::MockMac;
    #[cfg(feature = "centralized-tclk")]
    use zigbee_mac::primitives::McpsDataIndication;
    use zigbee_mac::primitives::{PanDescriptor, SuperframeSpec, ZigbeeBeaconPayload};
    use zigbee_nwk::NwkLayer;
    #[cfg(feature = "centralized-tclk")]
    use zigbee_nwk::frames::{NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
    use zigbee_types::{MacAddress, PanId, ShortAddress};
    use zigbee_zdo::ZdoLayer;

    const DEVICE_IEEE: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const TC_IEEE: [u8; 8] = [0xAA; 8];
    const EPID: [u8; 8] = [0xBB; 8];
    const PAN_ID: PanId = PanId(0x1234);
    const OLD_ADDRESS: ShortAddress = ShortAddress(0x2345);
    #[cfg(feature = "centralized-tclk")]
    const NEW_ADDRESS: ShortAddress = ShortAddress(0x3456);

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, Waker};

        let mut context = Context::from_waker(Waker::noop());
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    fn rejoin_beacon() -> PanDescriptor {
        PanDescriptor {
            channel: 15,
            coord_address: MacAddress::Short(PAN_ID, ShortAddress::COORDINATOR),
            superframe_spec: SuperframeSpec {
                association_permit: true,
                pan_coordinator: true,
                ..Default::default()
            },
            lqi: 250,
            security_use: false,
            zigbee_beacon: ZigbeeBeaconPayload {
                protocol_id: 0,
                stack_profile: 2,
                protocol_version: 2,
                router_capacity: true,
                device_depth: 0,
                end_device_capacity: true,
                extended_pan_id: EPID,
                tx_offset: [0xFF; 3],
                update_id: 0,
            },
        }
    }

    #[cfg(feature = "centralized-tclk")]
    fn unsecured_rejoin_response() -> zigbee_mac::MacFrame {
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 2,
                discover_route: 0,
                multicast: false,
                security: false,
                source_route: false,
                dst_ieee_present: true,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr: OLD_ADDRESS,
            src_addr: ShortAddress::COORDINATOR,
            radius: 1,
            seq_number: 0x43,
            dst_ieee: Some(DEVICE_IEEE),
            src_ieee: Some(TC_IEEE),
            multicast_control: None,
            source_route: None,
        };
        let payload = [
            NwkCommandId::RejoinResponse as u8,
            NEW_ADDRESS.0 as u8,
            (NEW_ADDRESS.0 >> 8) as u8,
            0,
        ];
        let mut frame = [0u8; 64];
        let header_len = header.serialize(&mut frame);
        frame[header_len..header_len + payload.len()].copy_from_slice(&payload);
        zigbee_mac::MacFrame::from_slice(&frame[..header_len + payload.len()]).unwrap()
    }

    fn restored_bdb(with_beacon: bool) -> BdbLayer<MockMac> {
        let mut mac = MockMac::new(DEVICE_IEEE);
        if with_beacon {
            mac.add_beacon(rejoin_beacon());
        }
        let mut nwk = NwkLayer::new(mac, zigbee_nwk::DeviceType::EndDevice);
        nwk.set_joined(true);
        nwk.security_mut().set_network_key([0x5A; 16], 0);
        {
            let nib = nwk.nib_mut();
            nib.extended_pan_id = EPID;
            nib.pan_id = PAN_ID;
            nib.network_address = OLD_ADDRESS;
            nib.ieee_address = DEVICE_IEEE;
            nib.logical_channel = 15;
            nib.parent_address = ShortAddress::COORDINATOR;
            nib.set_nwk_update_id(0);
            nib.security_enabled = true;
            nib.outgoing_frame_counter_limit = 0x400;
        }
        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);
        zdo.set_local_nwk_addr(OLD_ADDRESS);
        zdo.set_local_ieee_addr(DEVICE_IEEE);
        zdo.aps_mut().aib_mut().aps_trust_center_address = TC_IEEE;
        let mut bdb = BdbLayer::new(zdo);
        bdb.attributes_mut().node_is_on_a_network = true;
        bdb
    }

    #[test]
    fn failed_rejoin_scan_rolls_back_live_joined_flags() {
        let mut bdb = restored_bdb(false);

        assert_eq!(
            block_on(bdb.rejoin_previous_network()),
            Err(BdbStatus::NoScanResponse)
        );
        assert!(!bdb.zdo().nwk().is_joined());
        assert!(!bdb.is_on_network());
        assert_eq!(bdb.state(), &BdbState::Idle);
    }

    #[test]
    fn initialization_preserves_requested_finding_binding_test_capability() {
        let mut bdb = restored_bdb(false);
        bdb.attributes_mut().node_commissioning_capability = CommissioningMode::FINDING_BINDING;

        assert_eq!(bdb.initialize(), Ok(()));
        assert!(
            bdb.attributes()
                .node_commissioning_capability
                .contains(CommissioningMode::FINDING_BINDING)
        );
    }

    #[cfg(feature = "centralized-tclk")]
    #[test]
    fn trust_center_rejoin_without_transport_key_rolls_back_provisional_join() {
        let mut bdb = restored_bdb(true);
        bdb.zdo_mut()
            .nwk_mut()
            .mac_mut()
            .enqueue_rx(McpsDataIndication {
                src_address: MacAddress::Short(PAN_ID, ShortAddress::COORDINATOR),
                dst_address: MacAddress::Short(PAN_ID, OLD_ADDRESS),
                lqi: 250,
                payload: unsecured_rejoin_response(),
                security_use: false,
            });

        assert_eq!(
            block_on(bdb.trust_center_rejoin_previous_network()),
            Err(BdbStatus::SteeringFailure)
        );
        assert!(!bdb.zdo().nwk().is_joined());
        assert!(!bdb.is_on_network());
        assert_eq!(
            bdb.attributes().commissioning_status,
            crate::attributes::BdbCommissioningStatus::NoNetwork
        );
        assert_eq!(bdb.state(), &BdbState::Idle);
    }
}
