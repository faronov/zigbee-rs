//! Network Steering commissioning (BDB v3.0.1 spec §8.3).
//!
//! Network Steering has two operating modes depending on whether the
//! device is already on a network:
//!
//! ## Not on a network
//! 1. Scan primary channels for open networks (`NLME-NETWORK-DISCOVERY`)
//! 2. Filter by extended PAN ID if `bdbUseExtendedPanId` is configured
//! 3. Attempt to join the best-LQI network (`NLME-JOIN`)
//! 4. On join success: broadcast `Device_annce`
//! 5. Request Trust Center link key (APSME-REQUEST-KEY)
//! 6. If primary channels fail, retry on secondary channels
//!
//! ## Already on a network
//! 1. Open local permit joining for `bdbcMinCommissioningTime`
//! 2. Broadcast `Mgmt_Permit_Joining_req` to the network

use zigbee_aps::security::{ApsKeyType, ApsLinkKeyEntry};
use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::{ChannelMask, ShortAddress};
use zigbee_zdo::ZdpStatus;
use zigbee_zdo::discovery::NodeDescRsp;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::tclk_exchange::{TCLK_EXCHANGE_TIMEOUT_US, TclkExchange, TclkProgress, TclkStage};
use crate::{
    BdbLayer, BdbStatus, KeyFrameResult, NetworkSecurityState, SecurityPersistence,
    SteeringDiagnostics, SteeringStage, TrustCenterLinkKeyState,
};

#[cfg(feature = "trace")]
macro_rules! bdb_diag {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}
#[cfg(not(feature = "trace"))]
macro_rules! bdb_diag {
    ($($arg:tt)*) => {};
}

/// Default scan duration exponent for active scan (2^n + 1 superframes).
/// Exponent 3 ≈ 138 ms per channel — good balance of speed vs. reliability.
const SCAN_DURATION: u8 = 3;
// The unique Trust Center link-key handshake timing/budget lives in the
// event-driven state machine (`crate::tclk_exchange`).
const TCLK_MIN_STACK_REVISION: u8 = 21;
const FIRST_SCAN_CHANNEL: u8 = 15;

fn ordered_steering_channel_sets(primary: ChannelMask, secondary: ChannelMask) -> [ChannelMask; 3] {
    let first_channel_bit = 1u32 << FIRST_SCAN_CHANNEL;
    let first = ChannelMask(primary.0 & first_channel_bit);
    let preferred = ChannelMask(primary.0 & !first_channel_bit);
    let fallback = ChannelMask(secondary.0 & !primary.0);
    [first, preferred, fallback]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_scans_channel_15_then_primary_then_secondary() {
        let primary = ChannelMask((1 << 11) | (1 << 15) | (1 << 20));
        let secondary = ChannelMask((1 << 12) | (1 << 20) | (1 << 25));

        assert_eq!(
            ordered_steering_channel_sets(primary, secondary),
            [
                ChannelMask(1 << 15),
                ChannelMask((1 << 11) | (1 << 20)),
                ChannelMask((1 << 12) | (1 << 25)),
            ]
        );
    }

    #[test]
    fn steering_preserves_primary_order_when_channel_15_is_not_primary() {
        let primary = ChannelMask((1 << 20) | (1 << 25));
        let secondary = ChannelMask((1 << 15) | (1 << 26));

        assert_eq!(
            ordered_steering_channel_sets(primary, secondary),
            [ChannelMask(0), primary, secondary]
        );
    }

    // ── Event-driven unique-TCLK exchange integration ───────

    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_zdo::ZdoLayer;

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    fn test_bdb() -> BdbLayer<MockMac> {
        let mac = MockMac::new([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        let nwk = NwkLayer::new(mac, DeviceType::EndDevice);
        let aps = ApsLayer::new(nwk);
        let zdo = ZdoLayer::new(aps);
        BdbLayer::new(zdo)
    }

    fn advance_time(bdb: &mut BdbLayer<MockMac>, micros: u32) {
        block_on(
            bdb.zdo_mut()
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .delay_micros(micros),
        );
    }

    #[test]
    fn tclk_exchange_fails_and_resets_after_attempt_budget() {
        let tc_ieee = [0xAA; 8];
        let mut bdb = test_bdb();
        bdb.arm_tclk_exchange_for_test(ShortAddress::COORDINATOR, tc_ieee);
        assert!(bdb.tclk_exchange_active());
        assert!(bdb.is_on_network());

        // No Node_Desc_rsp is ever injected, so each attempt must time out.
        // Advancing mock time between bounded steps drives the machine to its
        // terminal failure without monopolising a single long future.
        let mut result = None;
        for _ in 0..512 {
            match block_on(bdb.advance_tclk_exchange(None)) {
                TclkProgress::InProgress => advance_time(&mut bdb, 2_000_000),
                terminal => {
                    result = Some(terminal);
                    break;
                }
            }
        }

        assert_eq!(
            result,
            Some(TclkProgress::Failed(
                BdbStatus::TrustCenterLinkKeyExchangeFailure
            )),
            "exhausting the attempt budget must fail the exchange"
        );
        assert!(
            !bdb.tclk_exchange_active(),
            "a terminal exchange must be cleared"
        );
        assert!(
            !bdb.is_on_network(),
            "failure must reset the on-network flag consistently"
        );
        assert!(bdb.steering_diagnostics().node_desc_requests >= 1);
    }

    #[test]
    fn advance_without_armed_exchange_reports_complete() {
        let mut bdb = test_bdb();
        assert!(!bdb.tclk_exchange_active());
        assert_eq!(
            block_on(bdb.advance_tclk_exchange(None)),
            TclkProgress::Complete
        );
    }

    #[derive(Default)]
    struct TestPersistence {
        reserved: Option<TrustCenterLinkKeyState>,
        committed: Option<TrustCenterLinkKeyState>,
    }

    impl SecurityPersistence for TestPersistence {
        fn reserve_network_security(
            &mut self,
            _state: &NetworkSecurityState,
        ) -> Result<crate::CounterReservation, crate::SecurityPersistenceError> {
            unreachable!()
        }

        fn reserve_trust_center_link_key(
            &mut self,
            state: &TrustCenterLinkKeyState,
        ) -> Result<crate::CounterReservation, crate::SecurityPersistenceError> {
            self.reserved = Some(*state);
            Ok(crate::CounterReservation {
                current: 0x400,
                limit: 0x800,
            })
        }

        fn commit_network(
            &mut self,
            state: &TrustCenterLinkKeyState,
        ) -> Result<(), crate::SecurityPersistenceError> {
            self.committed = Some(*state);
            Ok(())
        }
    }

    #[test]
    fn pre_r21_commits_configured_default_trust_center_key() {
        let tc_ieee = [0xAA; 8];
        let mut bdb = test_bdb();
        let expected_key = *bdb.zdo().aps().security().default_tc_link_key();
        let mut exchange = TclkExchange::new(ShortAddress::COORDINATOR, tc_ieee, 0);
        let mut persistence = TestPersistence::default();

        assert_eq!(
            bdb.finalize_pre_r21(&mut exchange, Some(&mut persistence)),
            TclkProgress::Complete
        );
        assert_eq!(
            persistence.reserved,
            Some(TrustCenterLinkKeyState {
                partner_address: tc_ieee,
                key: expected_key,
                key_type: ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
        );
        assert_eq!(persistence.committed.unwrap().outgoing_frame_counter, 0x400);
        let stored = bdb
            .zdo()
            .aps()
            .security()
            .find_key(&tc_ieee, ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert_eq!(stored.key, expected_key);
        assert_eq!(stored.outgoing_frame_counter, 0x400);
        assert_eq!(stored.outgoing_frame_counter_limit, 0x800);
    }
}

impl<M: MacDriver> BdbLayer<M> {
    fn security_exchange_timed_out(&self, started: u32) -> bool {
        self.zdo
            .aps()
            .nwk()
            .mac()
            .monotonic_micros()
            .wrapping_sub(started)
            >= TCLK_EXCHANGE_TIMEOUT_US
    }

    /// Execute the Network Steering procedure (BDB spec §8.3).
    ///
    /// Behaviour depends on `bdbNodeIsOnANetwork`:
    /// - **Not on network**: scan → join → announce → TC key exchange
    /// - **On network**: open permit joining → broadcast Mgmt_Permit_Joining_req
    ///
    /// Runs the pre-network work (scan → join → Transport-Key →
    /// `Device_annce`) and arms the post-network unique-TCLK exchange. It
    /// returns once the network is up; the caller must continue normal stack
    /// processing and call [`Self::advance_tclk_exchange`] until completion.
    pub async fn network_steering(&mut self) -> Result<(), BdbStatus> {
        self.network_steering_inner(None).await
    }

    /// Event-driven Network Steering with synchronous security persistence.
    ///
    /// Network security is reserved before `Device_annce`; the unique TCLK and
    /// its counter are reserved before Verify-Key and the network is committed
    /// only after Confirm-Key while advancing the exchange.
    pub async fn network_steering_with_persistence(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
    ) -> Result<(), BdbStatus> {
        self.network_steering_inner(Some(persistence)).await
    }

    async fn network_steering_inner(
        &mut self,
        persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> Result<(), BdbStatus> {
        self.steering_diagnostics = SteeringDiagnostics::default();
        self.tclk_exchange = None;
        if self.attributes.node_is_on_a_network {
            self.steer_on_network().await
        } else {
            self.steer_off_network(persistence).await
        }
    }

    /// Advance the armed unique Trust Center link-key exchange by one bounded
    /// step (GSDK update-tc-link-key, event-driven).
    ///
    /// Performs at most one non-blocking action per call — a single transmit,
    /// or a check of already-received ZDO/APS security state plus the
    /// per-attempt timeout — so the application/runtime keeps servicing normal
    /// traffic between calls. Returns [`TclkProgress::Complete`] only after a
    /// pre-R21 determination or a successful unique-key Verify/Confirm, and
    /// [`TclkProgress::Failed`] after resetting/leaving the network
    /// consistently once the attempt budget is exhausted (or on a persistence
    /// error). When no exchange is armed it returns [`TclkProgress::Complete`].
    ///
    /// `persistence`, when supplied, reserves the unique TCLK/counter before
    /// Verify-Key and commits the commissioned network only after Confirm-Key.
    pub async fn advance_tclk_exchange(
        &mut self,
        persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> TclkProgress {
        let Some(mut exchange) = self.tclk_exchange.take() else {
            return TclkProgress::Complete;
        };
        let progress = self.step_tclk_exchange(&mut exchange, persistence).await;
        if matches!(progress, TclkProgress::InProgress) {
            self.tclk_exchange = Some(exchange);
        }
        progress
    }

    /// Steering when the device is NOT on a network — join an existing PAN.
    async fn steer_off_network(
        &mut self,
        mut persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> Result<(), BdbStatus> {
        self.steering_diagnostics.stage = SteeringStage::Scanning;
        let mut discovered_any = false;
        let mut discovered_networks_total: u16 = 0;
        let mut attempted_joins: u16 = 0;

        // Reset retry budget at the start of each commissioning attempt
        if self.attributes.steering_attempts_remaining == 0 {
            self.attributes.steering_attempts_remaining = 5;
        }
        self.attributes.steering_attempts_remaining = self
            .attributes
            .steering_attempts_remaining
            .saturating_sub(1);

        log::info!(
            "[BDB:Steering] Scanning for open networks… (attempts left: {})",
            self.attributes.steering_attempts_remaining,
        );

        // Give channel 15 a short dedicated first chance, then scan the rest
        // of the primary set and finally the non-overlapping secondary set.
        let channel_sets = ordered_steering_channel_sets(
            self.attributes.primary_channel_set,
            self.attributes.secondary_channel_set,
        );

        for (idx, &channel_mask) in channel_sets.iter().enumerate() {
            if channel_mask.0 == 0 {
                continue;
            }

            let set_name = match idx {
                0 => "channel 15",
                1 => "preferred",
                _ => "fallback",
            };
            log::debug!(
                "[BDB:Steering] Scanning {} channels: 0x{:08X}",
                set_name,
                channel_mask.0
            );

            // Step 1: Network discovery
            self.steering_diagnostics.scan_requests =
                self.steering_diagnostics.scan_requests.saturating_add(1);
            let networks = match self
                .zdo
                .nlme_network_discovery(channel_mask, SCAN_DURATION)
                .await
            {
                Ok(n) => n,
                Err(_) => {
                    log::debug!("[BDB:Steering] No networks on {} channels", set_name);
                    continue;
                }
            };

            log::info!("[BDB:Steering] Found {} network(s)", networks.len());
            discovered_any = discovered_any || !networks.is_empty();
            discovered_networks_total = discovered_networks_total
                .saturating_add(networks.len().min(u16::MAX as usize) as u16);
            self.steering_diagnostics.networks_discovered = self
                .steering_diagnostics
                .networks_discovered
                .saturating_add(networks.len().min(u16::MAX as usize) as u16);

            // Step 2: Filter by extended PAN ID if configured
            let use_epid = self.zdo.aps().aib().aps_use_extended_pan_id;
            let has_epid_filter = use_epid != [0u8; 8];
            let mut epid_rejects: u16 = 0;
            let mut permit_closed_rejects: u16 = 0;
            let mut pass_skips: u16 = 0;
            let mut set_attempted_joins: u16 = 0;

            // Debug: show all discovered networks
            for (i, network) in networks.iter().enumerate() {
                log::info!(
                    "[BDB:Steering] net[{}] PAN=0x{:04X} ch={} d={} permit={} LQI={} via 0x{:04X}",
                    i,
                    network.pan_id.0,
                    network.logical_channel,
                    network.depth,
                    network.permit_joining,
                    network.lqi,
                    network.router_address.0,
                );
            }

            // Step 3: Try routers before the coordinator. Coordinators often
            // have a small or saturated child table, while nearby routers are
            // the normal parents for sleepy devices. Keep the coordinator as
            // a fallback for sparse networks without an eligible router.
            for prefer_coordinator in [false, true] {
                for network in &networks {
                    // Apply extended PAN ID filter
                    if has_epid_filter && network.extended_pan_id != use_epid {
                        epid_rejects = epid_rejects.saturating_add(1);
                        log::debug!(
                            "[BDB:Steering] Skipping PAN 0x{:04X} — EPID mismatch",
                            network.pan_id.0,
                        );
                        continue;
                    }

                    // Must have permit joining enabled
                    if !network.permit_joining {
                        permit_closed_rejects = permit_closed_rejects.saturating_add(1);
                        self.steering_diagnostics.permit_closed_rejects = self
                            .steering_diagnostics
                            .permit_closed_rejects
                            .saturating_add(1);
                        continue;
                    }

                    // Two-pass: routers first, then coordinator fallback.
                    let is_coordinator = network.depth == 0;
                    if prefer_coordinator && !is_coordinator {
                        pass_skips = pass_skips.saturating_add(1);
                        continue;
                    }
                    if !prefer_coordinator && is_coordinator {
                        pass_skips = pass_skips.saturating_add(1);
                        continue; // already tried
                    }

                    set_attempted_joins = set_attempted_joins.saturating_add(1);
                    attempted_joins = attempted_joins.saturating_add(1);
                    self.steering_diagnostics.stage = SteeringStage::Joining;
                    self.steering_diagnostics.join_attempts =
                        self.steering_diagnostics.join_attempts.saturating_add(1);
                    self.steering_diagnostics.channel = network.logical_channel;
                    self.steering_diagnostics.pan_id = network.pan_id.0;
                    self.steering_diagnostics.parent_address = network.router_address.0;
                    self.steering_diagnostics.parent_lqi = network.lqi;
                    self.steering_diagnostics.parent_depth = network.depth;
                    // One attempt per parent — avoid polluting TC state with repeated join/leave
                    let max_tries = 1u8;
                    let mut joined_addr = None;
                    for try_num in 0..max_tries {
                        if try_num > 0 {
                            log::info!(
                                "[BDB:Steering] Retrying coordinator join (attempt {}/{})",
                                try_num + 1,
                                max_tries,
                            );
                        }

                        log::info!(
                            "[BDB:Steering] Joining PAN 0x{:04X} ch {} LQI {} depth {} via 0x{:04X}",
                            network.pan_id.0,
                            network.logical_channel,
                            network.lqi,
                            network.depth,
                            network.router_address.0,
                        );

                        // Step 3: Attempt join
                        match self.zdo.nlme_join(network).await {
                            Ok(addr) => {
                                bdb_diag!("[BDB] nlme_join=ok addr=0x{:04X}", addr.0);
                                self.steering_diagnostics.join_successes =
                                    self.steering_diagnostics.join_successes.saturating_add(1);
                                self.steering_diagnostics.last_join_status = 0;
                                self.steering_diagnostics.assigned_address = addr.0;
                                joined_addr = Some(addr);
                                break;
                            }
                            Err(e) => {
                                self.steering_diagnostics.last_join_status = e as u8;
                                bdb_diag!("[BDB] nlme_join=err {:?}", e);
                                log::warn!("[BDB:Steering] Join failed: {:?}", e);
                                continue;
                            }
                        }
                    }
                    let nwk_addr = match joined_addr {
                        Some(a) => a,
                        None => continue,
                    };

                    let ieee = self.zdo.nwk().nib().ieee_address;

                    // Step 5: Start router if we are a router
                    if self.zdo.nwk().device_type() == DeviceType::Router {
                        let _ = self.zdo.nlme_start_router().await;
                    }

                    // Step 5b: TC link key exchange
                    // After joining, the coordinator sends Transport-Key (with NWK key)
                    // encrypted with the well-known TC link key (ZigBeeAlliance09).
                    // We must receive and process it before declaring success.
                    // Then send APSME-REQUEST-KEY(0x04) so Z2M establishes a unique TC link key.
                    log::info!("[BDB:Steering] Waiting for Transport-Key from TC...");
                    self.steering_diagnostics.stage = SteeringStage::WaitingForTransportKey;

                    let mut key_received = false;
                    let rx_on = self.zdo.nwk().rx_on_when_idle();

                    // Phase 0: Passive RX listen — only useful when rx_on_when_idle=true
                    // because the TC sends Transport-Key as a DIRECT unicast. When the
                    // device is sleepy (rx_on_when_idle=false), the TC buffers the TK at
                    // the parent as an indirect frame — passive RX will never see it and
                    // the ~3 s timeout delays the first poll, risking indirect-frame expiry.
                    if rx_on {
                        log::info!(
                            "[BDB:Steering] Phase 0: passive RX for direct Transport-Key..."
                        );
                        for rx_attempt in 0..4u8 {
                            #[allow(clippy::single_match)]
                            match self
                                .zdo
                                .aps_mut()
                                .nwk_mut()
                                .mac_mut()
                                .mcps_data_indication()
                                .await
                            {
                                Ok(mac_frame) => {
                                    self.steering_diagnostics.passive_rx_frames = self
                                        .steering_diagnostics
                                        .passive_rx_frames
                                        .saturating_add(1);
                                    self.steering_diagnostics.last_frame_len =
                                        mac_frame.payload.len().min(u8::MAX as usize) as u8;
                                    let prefix_len = mac_frame
                                        .payload
                                        .len()
                                        .min(self.steering_diagnostics.last_frame_prefix.len());
                                    self.steering_diagnostics.last_frame_prefix[..prefix_len]
                                        .copy_from_slice(
                                            &mac_frame.payload.as_slice()[..prefix_len],
                                        );
                                    let mac_payload = mac_frame.payload.as_slice();
                                    bdb_diag!(
                                        "[BDB] passive_rx[{}] {} bytes",
                                        rx_attempt,
                                        mac_payload.len()
                                    );
                                    log::info!(
                                        "[BDB:Steering] RX {}: {} bytes",
                                        rx_attempt,
                                        mac_payload.len(),
                                    );
                                    if let Some(true) = self.try_process_frame(mac_payload) {
                                        key_received = true;
                                        break;
                                    }
                                }

                                Err(_) => {
                                    bdb_diag!("[BDB] passive_rx[{}] none", rx_attempt);
                                }
                            }
                        }
                        if key_received {
                            log::info!("[BDB:Steering] Transport-Key received during passive RX!");
                        }
                    } else {
                        log::info!(
                            "[BDB:Steering] Phase 0: skipped (sleepy device, TK via indirect poll)"
                        );
                    }
                    // Poll long enough for the Trust Center to send the key after
                    // the parent relays Update-Device. Slow coordinators and
                    // multi-hop relays may require many rounds.
                    const MAX_TOTAL_ROUNDS: usize = 128;
                    const MAX_EMPTY_ROUNDS: u16 = 128;
                    const POLL_TIMEOUT_US: u32 = 500_000;
                    let mut empty_count: u16 = 0;
                    let mut total_rounds: usize = 0;
                    let mut data_frames: usize = 0;
                    let transport_key_wait_started = self.zdo.aps().nwk().mac().monotonic_micros();

                    while !key_received
                        && total_rounds < MAX_TOTAL_ROUNDS
                        && empty_count < MAX_EMPTY_ROUNDS
                        && !self.security_exchange_timed_out(transport_key_wait_started)
                    {
                        total_rounds += 1;
                        let mut got_data_this_round = false;
                        let elapsed = self
                            .zdo
                            .aps()
                            .nwk()
                            .mac()
                            .monotonic_micros()
                            .wrapping_sub(transport_key_wait_started);
                        let remaining = TCLK_EXCHANGE_TIMEOUT_US.saturating_sub(elapsed);

                        // Poll parent for indirect frames
                        self.steering_diagnostics.poll_attempts =
                            self.steering_diagnostics.poll_attempts.saturating_add(1);
                        match self
                            .zdo
                            .aps_mut()
                            .nwk_mut()
                            .mac_mut()
                            .mlme_poll_timeout(POLL_TIMEOUT_US.min(remaining))
                            .await
                        {
                            Ok(Some(mac_frame)) => {
                                self.steering_diagnostics.poll_data_frames =
                                    self.steering_diagnostics.poll_data_frames.saturating_add(1);
                                self.steering_diagnostics.last_frame_len =
                                    mac_frame.len().min(u8::MAX as usize) as u8;
                                let prefix_len = mac_frame
                                    .len()
                                    .min(self.steering_diagnostics.last_frame_prefix.len());
                                self.steering_diagnostics.last_frame_prefix[..prefix_len]
                                    .copy_from_slice(&mac_frame.as_slice()[..prefix_len]);
                                got_data_this_round = true;
                                data_frames += 1;
                                let mac_payload = mac_frame.as_slice();
                                bdb_diag!(
                                    "[BDB] parent_poll[{}] {} bytes total={}",
                                    total_rounds,
                                    mac_payload.len(),
                                    data_frames
                                );
                                log::info!(
                                    "[BDB:Steering] P-Poll {}: {} bytes (total={})",
                                    total_rounds,
                                    mac_payload.len(),
                                    data_frames,
                                );
                                if let Some(true) = self.try_process_frame(mac_payload) {
                                    bdb_diag!("[BDB] transport_key=ok via parent_poll");
                                    key_received = true;
                                    break;
                                }
                            }
                            Ok(None) => {
                                bdb_diag!("[BDB] parent_poll[{}] none", total_rounds);
                            }
                            Err(e) => {
                                self.steering_diagnostics.poll_errors =
                                    self.steering_diagnostics.poll_errors.saturating_add(1);
                                bdb_diag!("[BDB] parent_poll[{}] err {:?}", total_rounds, e);
                                log::warn!("[BDB:Steering] P-Poll {}: err {:?}", total_rounds, e);
                            }
                        }

                        if key_received {
                            break;
                        }

                        if key_received {
                            break;
                        }

                        if got_data_this_round {
                            empty_count = 0;
                        } else {
                            empty_count += 1;
                            log::debug!(
                                "[BDB:Steering] Round {}: no data ({}/{})",
                                total_rounds,
                                empty_count,
                                MAX_EMPTY_ROUNDS,
                            );
                        }
                    }

                    log::info!(
                        "[BDB:Steering] Transport-Key wait done: passive_rx={} rounds={} frames={} empty={}",
                        if key_received { "hit" } else { "miss" },
                        total_rounds,
                        data_frames,
                        empty_count
                    );

                    if !key_received {
                        self.steering_diagnostics.stage = SteeringStage::TransportKeyMissing;
                        bdb_diag!(
                            "[BDB] transport_key=missing rounds={} frames={} empty={}",
                            total_rounds,
                            data_frames,
                            empty_count
                        );
                        log::warn!(
                            "[BDB:Steering] Transport-Key NOT received after {} rounds ({} data frames, {} consecutive empty)",
                            total_rounds,
                            data_frames,
                            empty_count,
                        );
                    }

                    if !key_received {
                        bdb_diag!(
                            "[BDB] reset pan=0x{:04X} reason=no_transport_key",
                            network.pan_id.0
                        );
                        log::warn!(
                            "[BDB:Steering] Transport-Key not received — resetting and trying next parent on PAN 0x{:04X}",
                            network.pan_id.0,
                        );
                        // We cannot send a proper encrypted leave without the
                        // network key. Clear local NWK/MAC state and try the
                        // next beacon candidate; declaring success here leaves
                        // us unable to decrypt ZHA interview traffic.
                        let _ = self.zdo.nlme_reset(false);
                        continue;
                    }

                    self.steering_diagnostics.stage = SteeringStage::TransportKeyReceived;
                    self.steering_diagnostics.transport_key_received = true;

                    if let Some(persistence) = persistence.as_deref_mut()
                        && let Err(error) = self.reserve_network_security(persistence)
                    {
                        self.steering_diagnostics.stage = SteeringStage::PersistenceFailed;
                        log::error!(
                            "[BDB:Steering] Failed to persist network security: {:?}",
                            error
                        );
                        let _ = self.zdo.nlme_reset(false);
                        return Err(BdbStatus::PersistenceFailure);
                    }

                    // Step 5c: Send Device_annce now that we have the NWK key
                    self.steering_diagnostics.stage = SteeringStage::Announcing;
                    self.zdo.set_local_nwk_addr(nwk_addr);
                    self.zdo.set_local_ieee_addr(ieee);
                    bdb_diag!(
                        "[BDB] zdo_local addr=0x{:04X} ieee={:02X?}",
                        nwk_addr.0,
                        ieee
                    );
                    if let Err(e) = self.zdo.device_annce(nwk_addr, ieee).await {
                        log::warn!("[BDB:Steering] Device_annce failed: {:?}", e);
                        let _ = self.zdo.nlme_reset(false);
                        continue;
                    }

                    // Step 5d: retrieve a unique Trust Center link key, prove
                    // possession, and wait for a successful Confirm-Key. In the
                    // GSDK model this runs *after* the network is up: we arm an
                    // explicit bounded state machine here and let the runtime
                    // advance it one step per tick/poll while normal ZDO/ZCL
                    // processing and sleepy polling continue.
                    let tc_addr = ShortAddress::COORDINATOR;
                    let tc_ieee = self.zdo.aps().aib().aps_trust_center_address;
                    if tc_ieee == [0u8; 8] {
                        self.steering_diagnostics.stage =
                            SteeringStage::TrustCenterLinkKeyExchangeFailed;
                        self.attributes.commissioning_status =
                            crate::attributes::BdbCommissioningStatus::TcLinkKeyExchangeFailure;
                        let _ = self.zdo.nlme_reset(false);
                        return Err(BdbStatus::TrustCenterLinkKeyExchangeFailure);
                    }

                    // The network is up. Mark the node on-network so the runtime
                    // resumes normal servicing immediately (GSDK EMBER_NETWORK_UP);
                    // the durable "commissioned" flag is only committed after
                    // Confirm-Key via the persistence hook.
                    self.attributes.node_is_on_a_network = true;
                    let now = self.zdo.aps().nwk().mac().monotonic_micros();
                    self.tclk_exchange = Some(TclkExchange::new(tc_addr, tc_ieee, now));
                    bdb_diag!("[BDB] steering=network_up addr=0x{:04X}", nwk_addr.0);
                    log::info!(
                        "[BDB:Steering] Network up as 0x{:04X} — unique TCLK exchange armed",
                        nwk_addr.0,
                    );
                    return Ok(());
                }
            } // end prefer_coordinator pass

            if !networks.is_empty() {
                log::info!(
                    "[BDB:Steering] {} summary: total={} attempted={} reject_epid={} reject_permit_closed={} pass_skips={}",
                    set_name,
                    networks.len(),
                    set_attempted_joins,
                    epid_rejects,
                    permit_closed_rejects,
                    pass_skips,
                );
                if set_attempted_joins == 0 {
                    log::warn!(
                        "[BDB:Steering] {}: discovered networks but none were join candidates (all filtered)",
                        set_name
                    );
                }
            }
        }

        // All attempts exhausted
        if discovered_any {
            log::warn!(
                "[BDB:Steering] Exhausted steering with {} discovered network(s) but {} join attempt(s)",
                discovered_networks_total,
                attempted_joins
            );
        }
        if self.steering_diagnostics.join_successes != 0 {
            self.steering_diagnostics.stage = SteeringStage::TransportKeyMissing;
        } else if attempted_joins != 0 {
            self.steering_diagnostics.stage = SteeringStage::JoinFailed;
        } else if discovered_any {
            self.steering_diagnostics.stage = SteeringStage::NoJoinCandidate;
        } else {
            self.steering_diagnostics.stage = SteeringStage::NoNetworks;
        }
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::NoScanResponse;
        Err(BdbStatus::NoScanResponse)
    }

    fn reserve_network_security(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let (network_key, key_sequence) = self
            .zdo
            .nwk()
            .security()
            .active_key()
            .map(|entry| (entry.key, entry.seq_number))
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        let nib = self.zdo.nwk().nib();
        let state = NetworkSecurityState {
            extended_pan_id: nib.extended_pan_id,
            pan_id: nib.pan_id.0,
            short_address: nib.network_address.0,
            ieee_address: nib.ieee_address,
            channel: nib.logical_channel,
            depth: nib.depth,
            parent_address: nib.parent_address.0,
            update_id: nib.update_id,
            network_key,
            key_sequence,
            outgoing_frame_counter: nib.outgoing_frame_counter,
        };
        let reservation = persistence.reserve_network_security(&state)?;
        if !reservation.is_valid() || reservation.current < state.outgoing_frame_counter {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        if !self
            .zdo
            .nwk_mut()
            .nib_mut()
            .set_frame_counter_reservation(reservation.current, reservation.limit)
        {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        Ok(())
    }

    fn reserve_trust_center_link_key(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
        trust_center: &zigbee_types::IeeeAddress,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let state = self
            .zdo
            .aps()
            .security()
            .find_key(trust_center, ApsKeyType::TrustCenterLinkKey)
            .map(|entry| TrustCenterLinkKeyState {
                partner_address: entry.partner_address,
                key: entry.key,
                key_type: entry.key_type,
                outgoing_frame_counter: entry.outgoing_frame_counter,
                incoming_frame_counter: entry.incoming_frame_counter,
                incoming_frame_counter_valid: entry.incoming_frame_counter_valid,
            })
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        let reservation = persistence.reserve_trust_center_link_key(&state)?;
        if !reservation.is_valid() || reservation.current < state.outgoing_frame_counter {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        let entry = self
            .zdo
            .aps_mut()
            .security_mut()
            .find_key_mut(trust_center, ApsKeyType::TrustCenterLinkKey)
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        entry.outgoing_frame_counter = reservation.current;
        entry.outgoing_frame_counter_limit = reservation.limit;
        Ok(())
    }

    fn commit_persisted_network(
        &self,
        persistence: &mut dyn SecurityPersistence,
        trust_center: &zigbee_types::IeeeAddress,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let state = self
            .zdo
            .aps()
            .security()
            .find_key(trust_center, ApsKeyType::TrustCenterLinkKey)
            .map(|entry| TrustCenterLinkKeyState {
                partner_address: entry.partner_address,
                key: entry.key,
                key_type: entry.key_type,
                outgoing_frame_counter: entry.outgoing_frame_counter,
                incoming_frame_counter: entry.incoming_frame_counter,
                incoming_frame_counter_valid: entry.incoming_frame_counter_valid,
            })
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        persistence.commit_network(&state)
    }

    // ── Event-driven unique TCLK exchange ───────────────────

    /// Advance the exchange by one bounded step. See
    /// [`Self::advance_tclk_exchange`] for the public contract.
    async fn step_tclk_exchange(
        &mut self,
        ex: &mut TclkExchange,
        mut persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> TclkProgress {
        let now = self.zdo.aps().nwk().mac().monotonic_micros();
        match ex.stage {
            TclkStage::StartDelay => {
                if ex.start_delay_elapsed(now) {
                    ex.begin_attempt(now);
                }
                TclkProgress::InProgress
            }

            TclkStage::SendNodeDesc => {
                // Fresh attempt — drop any stale unique key so a retry cannot
                // reuse partial security material.
                self.zdo
                    .aps_mut()
                    .security_mut()
                    .remove_key(&ex.tc_ieee, ApsKeyType::TrustCenterLinkKey);
                self.steering_diagnostics.stage = SteeringStage::QueryingTrustCenterNodeDescriptor;
                self.steering_diagnostics.node_desc_requests = self
                    .steering_diagnostics
                    .node_desc_requests
                    .saturating_add(1);
                match self.zdo.start_node_desc_req(ex.tc_addr).await {
                    Ok(slot) => {
                        ex.node_desc_slot = Some(slot);
                        ex.stage = TclkStage::AwaitNodeDesc;
                    }
                    Err(e) => {
                        self.steering_diagnostics.node_desc_send_failures = self
                            .steering_diagnostics
                            .node_desc_send_failures
                            .saturating_add(1);
                        self.steering_diagnostics.last_node_desc_status = e as u8;
                        log::warn!("[BDB:Steering] Node_Desc_req failed: {:?}", e);
                        ex.stage = TclkStage::AttemptCooldown;
                    }
                }
                TclkProgress::InProgress
            }

            TclkStage::AwaitNodeDesc => {
                let slot = match ex.node_desc_slot {
                    Some(slot) => slot,
                    None => {
                        ex.stage = TclkStage::AttemptCooldown;
                        return TclkProgress::InProgress;
                    }
                };
                if let Some(payload) = self.zdo.take_response(slot) {
                    ex.node_desc_slot = None;
                    ex.restart_stage_timeout(now);
                    self.steering_diagnostics.node_desc_responses = self
                        .steering_diagnostics
                        .node_desc_responses
                        .saturating_add(1);
                    self.handle_node_desc_payload(ex, &payload, persistence.as_deref_mut())
                } else if ex.attempt_timed_out(now) {
                    self.zdo.cancel_pending(slot);
                    ex.node_desc_slot = None;
                    self.steering_diagnostics.node_desc_timeouts = self
                        .steering_diagnostics
                        .node_desc_timeouts
                        .saturating_add(1);
                    log::warn!("[BDB:Steering] Node_Desc_rsp timed out");
                    self.retry_or_fail(ex, now)
                } else {
                    TclkProgress::InProgress
                }
            }

            TclkStage::SendRequestKey => {
                self.steering_diagnostics.stage = SteeringStage::RequestingTrustCenterLinkKey;
                self.steering_diagnostics.request_key_attempts = self
                    .steering_diagnostics
                    .request_key_attempts
                    .saturating_add(1);
                match self.zdo.aps_mut().send_request_key(ex.tc_addr).await {
                    Ok(()) => {
                        ex.restart_stage_timeout(self.zdo.aps().nwk().mac().monotonic_micros());
                        self.steering_diagnostics.request_key_send_successes = self
                            .steering_diagnostics
                            .request_key_send_successes
                            .saturating_add(1);
                        self.steering_diagnostics.request_key_error = 0;
                        ex.stage = TclkStage::AwaitTclk;
                    }
                    Err(e) => {
                        self.steering_diagnostics.request_key_send_failures = self
                            .steering_diagnostics
                            .request_key_send_failures
                            .saturating_add(1);
                        self.steering_diagnostics.request_key_error = e as u8;
                        log::warn!("[BDB:Steering] Request-Key failed: {:?}", e);
                        ex.stage = TclkStage::AttemptCooldown;
                    }
                }
                TclkProgress::InProgress
            }

            TclkStage::AwaitTclk => {
                self.steering_diagnostics.stage = SteeringStage::WaitingForTrustCenterLinkKey;
                let installed = self
                    .zdo
                    .aps()
                    .security()
                    .find_key(&ex.tc_ieee, ApsKeyType::TrustCenterLinkKey)
                    .is_some();
                if installed {
                    self.steering_diagnostics.tclk_installations = self
                        .steering_diagnostics
                        .tclk_installations
                        .saturating_add(1);
                    // Reserve the unique TCLK/counter *before* Verify-Key.
                    if let Some(persistence) = persistence.as_deref_mut()
                        && let Err(error) =
                            self.reserve_trust_center_link_key(persistence, &ex.tc_ieee)
                    {
                        log::error!(
                            "[BDB:Steering] Failed to persist Trust Center link key: {:?}",
                            error
                        );
                        return self.finalize_persistence_failure(ex);
                    }
                    let stats = self.zdo.aps().security_handshake_stats();
                    ex.confirm_success_baseline = stats.confirm_key_successes;
                    ex.confirm_reject_baseline = stats.confirm_key_rejections;
                    ex.restart_stage_timeout(now);
                    ex.stage = TclkStage::SendVerifyKey;
                    TclkProgress::InProgress
                } else if ex.attempt_timed_out(now) {
                    log::warn!("[BDB:Steering] Unique TC link key was not received");
                    self.retry_or_fail(ex, now)
                } else {
                    TclkProgress::InProgress
                }
            }

            TclkStage::SendVerifyKey => {
                self.steering_diagnostics.stage = SteeringStage::VerifyingLinkKey;
                self.steering_diagnostics.verify_key_attempts = self
                    .steering_diagnostics
                    .verify_key_attempts
                    .saturating_add(1);
                match self.zdo.aps_mut().send_tc_verify_key(ex.tc_addr).await {
                    Ok(()) => {
                        ex.restart_stage_timeout(self.zdo.aps().nwk().mac().monotonic_micros());
                        self.steering_diagnostics.verify_key_successes = self
                            .steering_diagnostics
                            .verify_key_successes
                            .saturating_add(1);
                        self.steering_diagnostics.verify_key_error = 0;
                        ex.stage = TclkStage::AwaitConfirmKey;
                    }
                    Err(e) => {
                        self.steering_diagnostics.verify_key_error = e as u8;
                        log::warn!("[BDB:Steering] Verify-Key failed: {:?}", e);
                        ex.stage = TclkStage::AttemptCooldown;
                    }
                }
                TclkProgress::InProgress
            }

            TclkStage::AwaitConfirmKey => {
                self.steering_diagnostics.stage = SteeringStage::WaitingForConfirmKey;
                let stats = self.zdo.aps().security_handshake_stats();
                self.steering_diagnostics.confirm_key_frames = stats.confirm_key_received;
                self.steering_diagnostics.confirm_key_successes = stats.confirm_key_successes;
                self.steering_diagnostics.confirm_key_rejections = stats.confirm_key_rejections;
                self.steering_diagnostics.last_confirm_key_status = stats.last_confirm_key_status;

                if stats.confirm_key_successes > ex.confirm_success_baseline {
                    self.finalize_tclk_success(ex, persistence.as_deref_mut())
                } else if stats.confirm_key_rejections > ex.confirm_reject_baseline {
                    log::warn!("[BDB:Steering] Confirm-Key rejected by Trust Center");
                    self.retry_or_fail(ex, now)
                } else if ex.attempt_timed_out(now) {
                    log::warn!("[BDB:Steering] Confirm-Key not received");
                    self.retry_or_fail(ex, now)
                } else {
                    TclkProgress::InProgress
                }
            }

            TclkStage::AttemptCooldown => {
                // Drain the remainder of the failed attempt's window before
                // retrying, matching the pacing of the original blocking loop.
                if ex.attempt_timed_out(now) {
                    self.retry_or_fail(ex, now)
                } else {
                    TclkProgress::InProgress
                }
            }

            TclkStage::Complete => TclkProgress::Complete,
            TclkStage::Failed => TclkProgress::Failed(BdbStatus::TrustCenterLinkKeyExchangeFailure),
        }
    }

    /// Parse a Node_Desc_rsp and decide the next stage: pre-R21 completes the
    /// exchange, R21+ proceeds to the unique-key request; a rejected or
    /// malformed response cools down and retries.
    fn handle_node_desc_payload(
        &mut self,
        ex: &mut TclkExchange,
        payload: &[u8],
        persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> TclkProgress {
        let node_desc = match NodeDescRsp::parse(payload) {
            Ok(response) => response,
            Err(e) => {
                self.steering_diagnostics.node_desc_parse_failures = self
                    .steering_diagnostics
                    .node_desc_parse_failures
                    .saturating_add(1);
                log::warn!("[BDB:Steering] Invalid Node_Desc_rsp: {:?}", e);
                ex.stage = TclkStage::AttemptCooldown;
                return TclkProgress::InProgress;
            }
        };
        self.steering_diagnostics.last_node_desc_status = node_desc.status as u8;
        if node_desc.status != ZdpStatus::Success || node_desc.nwk_addr_of_interest != ex.tc_addr {
            log::warn!(
                "[BDB:Steering] Node_Desc_rsp rejected: status={:?} addr=0x{:04X}",
                node_desc.status,
                node_desc.nwk_addr_of_interest.0,
            );
            ex.stage = TclkStage::AttemptCooldown;
            return TclkProgress::InProgress;
        }
        let Some(node_descriptor) = node_desc.node_descriptor else {
            self.steering_diagnostics.node_desc_parse_failures = self
                .steering_diagnostics
                .node_desc_parse_failures
                .saturating_add(1);
            ex.stage = TclkStage::AttemptCooldown;
            return TclkProgress::InProgress;
        };
        let stack_revision = node_descriptor.stack_revision();
        self.steering_diagnostics.trust_center_server_mask = node_descriptor.server_mask;
        self.steering_diagnostics.trust_center_stack_revision = stack_revision;
        log::info!(
            "[BDB:Steering] Trust Center stack revision {} (server mask 0x{:04X})",
            stack_revision,
            node_descriptor.server_mask,
        );

        if stack_revision < TCLK_MIN_STACK_REVISION {
            log::info!(
                "[BDB:Steering] Pre-R21 Trust Center; unique link-key exchange not required"
            );
            return self.finalize_pre_r21(ex, persistence);
        }

        ex.stage = TclkStage::SendRequestKey;
        TclkProgress::InProgress
    }

    /// Common success finalisation shared by the pre-R21 and confirmed paths.
    fn mark_commissioned_success(&mut self, ex: &mut TclkExchange) -> TclkProgress {
        self.attributes.node_is_on_a_network = true;
        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;
        self.steering_diagnostics.stage = SteeringStage::Complete;
        ex.stage = TclkStage::Complete;
        bdb_diag!("[BDB] tclk_exchange=complete");
        log::info!("[BDB:Steering] Commissioning security complete");
        TclkProgress::Complete
    }

    fn finalize_pre_r21(
        &mut self,
        ex: &mut TclkExchange,
        persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> TclkProgress {
        if let Some(persistence) = persistence {
            let key = *self.zdo.aps().security().default_tc_link_key();
            let mut state = TrustCenterLinkKeyState {
                partner_address: ex.tc_ieee,
                key,
                key_type: ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            };
            let reservation = match persistence.reserve_trust_center_link_key(&state) {
                Ok(reservation) if reservation.is_valid() => reservation,
                Ok(_) => {
                    log::error!("[BDB:Steering] Invalid pre-R21 TCLK counter reservation");
                    return self.finalize_persistence_failure(ex);
                }
                Err(error) => {
                    log::error!(
                        "[BDB:Steering] Failed to reserve pre-R21 Trust Center key: {:?}",
                        error
                    );
                    return self.finalize_persistence_failure(ex);
                }
            };
            state.outgoing_frame_counter = reservation.current;
            if let Err(_entry) = self.zdo.aps_mut().security_mut().add_key(ApsLinkKeyEntry {
                partner_address: ex.tc_ieee,
                key,
                key_type: ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: reservation.current,
                outgoing_frame_counter_limit: reservation.limit,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            }) {
                log::error!("[BDB:Steering] Failed to install pre-R21 Trust Center key");
                return self.finalize_persistence_failure(ex);
            }
            if let Err(error) = persistence.commit_network(&state) {
                log::error!(
                    "[BDB:Steering] Failed to commit pre-R21 commissioned network: {:?}",
                    error
                );
                return self.finalize_persistence_failure(ex);
            }
        }
        self.mark_commissioned_success(ex)
    }

    /// Commit the commissioned network after a successful Confirm-Key.
    fn finalize_tclk_success(
        &mut self,
        ex: &mut TclkExchange,
        persistence: Option<&mut (dyn SecurityPersistence + '_)>,
    ) -> TclkProgress {
        if let Some(persistence) = persistence
            && let Err(error) = self.commit_persisted_network(persistence, &ex.tc_ieee)
        {
            log::error!(
                "[BDB:Steering] Failed to commit commissioned network: {:?}",
                error
            );
            return self.finalize_persistence_failure(ex);
        }
        self.mark_commissioned_success(ex)
    }

    /// Decrement the attempt budget and either start a fresh attempt or fail
    /// the exchange, resetting/leaving the network consistently.
    fn retry_or_fail(&mut self, ex: &mut TclkExchange, now: u32) -> TclkProgress {
        self.zdo
            .aps_mut()
            .security_mut()
            .remove_key(&ex.tc_ieee, ApsKeyType::TrustCenterLinkKey);
        if let Some(slot) = ex.node_desc_slot.take() {
            self.zdo.cancel_pending(slot);
        }
        if ex.record_attempt_failure() {
            self.finalize_exchange_failure(ex)
        } else {
            ex.begin_attempt(now);
            TclkProgress::InProgress
        }
    }

    /// Terminal failure after exhausting the attempt budget.
    fn finalize_exchange_failure(&mut self, ex: &mut TclkExchange) -> TclkProgress {
        ex.stage = TclkStage::Failed;
        self.steering_diagnostics.stage = SteeringStage::TrustCenterLinkKeyExchangeFailed;
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::TcLinkKeyExchangeFailure;
        self.reset_after_tclk_failure(&ex.tc_ieee);
        TclkProgress::Failed(BdbStatus::TrustCenterLinkKeyExchangeFailure)
    }

    /// Terminal failure caused by a durable-persistence error.
    fn finalize_persistence_failure(&mut self, ex: &mut TclkExchange) -> TclkProgress {
        ex.stage = TclkStage::Failed;
        self.steering_diagnostics.stage = SteeringStage::PersistenceFailed;
        self.reset_after_tclk_failure(&ex.tc_ieee);
        TclkProgress::Failed(BdbStatus::PersistenceFailure)
    }

    /// Reset security/network state consistently after a failed exchange so
    /// the device does not linger half-commissioned.
    fn reset_after_tclk_failure(&mut self, tc_ieee: &zigbee_types::IeeeAddress) {
        self.zdo
            .aps_mut()
            .security_mut()
            .remove_key(tc_ieee, ApsKeyType::TrustCenterLinkKey);
        let _ = self.zdo.nlme_reset(false);
        self.attributes.node_is_on_a_network = false;
    }

    /// Steering when the device IS already on a network.
    ///
    /// Opens the network for joining and broadcasts Mgmt_Permit_Joining_req
    /// so that routers in the network also open their permit joining.
    async fn steer_on_network(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Steering] Already on network — opening permit joining");

        // Can only permit joining on coordinator / router
        if self.zdo.nwk().device_type() == DeviceType::EndDevice {
            log::debug!("[BDB:Steering] End device — skipping permit joining");
            // End devices can only trigger steering-on-network by sending
            // Mgmt_Permit_Joining_req to their parent / coordinator.
            let _ = self
                .zdo
                .mgmt_permit_joining_req(
                    ShortAddress::COORDINATOR,
                    BDB_MIN_COMMISSIONING_TIME as u8,
                    true,
                )
                .await;
            return Ok(());
        }

        // Open local permit joining (duration = bdbcMinCommissioningTime)
        // Duration is capped at 254 (0xFE) seconds per Zigbee spec.
        let duration = core::cmp::min(BDB_MIN_COMMISSIONING_TIME, 254) as u8;

        self.zdo
            .nlme_permit_joining(duration)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        // Broadcast Mgmt_Permit_Joining_req to all routers
        self.zdo
            .mgmt_permit_joining_req(ShortAddress::BROADCAST, duration, true)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;

        log::info!("[BDB:Steering] Permit joining opened for {}s", duration,);
        Ok(())
    }

    /// Parse a MAC payload, log diagnostics, and attempt Transport-Key extraction.
    /// Returns `Some(true)` if the NWK key was installed.
    fn try_process_frame(&mut self, mac_payload: &[u8]) -> Option<bool> {
        if let Some((nwk_hdr, nwk_consumed)) = zigbee_nwk::frames::NwkHeader::parse(mac_payload) {
            self.steering_diagnostics.nwk_header_len = nwk_consumed.min(u8::MAX as usize) as u8;
            self.steering_diagnostics.nwk_security = nwk_hdr.frame_control.security;
            bdb_diag!(
                "[BDB] nwk type={} src=0x{:04X} dst=0x{:04X} sec={} used={}",
                nwk_hdr.frame_control.frame_type,
                nwk_hdr.src_addr.0,
                nwk_hdr.dst_addr.0,
                nwk_hdr.frame_control.security as u8,
                nwk_consumed
            );
            log::info!(
                "[BDB:Steering] NWK: type={} src=0x{:04X} dst=0x{:04X} sec={}",
                nwk_hdr.frame_control.frame_type,
                nwk_hdr.src_addr.0,
                nwk_hdr.dst_addr.0,
                nwk_hdr.frame_control.security,
            );
            // Hex dump coordinator frames for debugging
            if nwk_hdr.src_addr.0 == 0x0000 {
                let dump_len = mac_payload.len().min(32);
                let hex: heapless::String<96> =
                    mac_payload[..dump_len]
                        .iter()
                        .fold(heapless::String::new(), |mut s, b| {
                            let _ = core::fmt::Write::write_fmt(&mut s, format_args!("{:02X}", b));
                            s
                        });
                log::info!("[BDB:Steering] COORD hex: {}", hex);
            }
            self.process_key_wait_frame(mac_payload, &nwk_hdr, nwk_consumed, 0)
        } else if mac_payload.len() > 2 {
            self.steering_diagnostics.key_frame_result = KeyFrameResult::NwkParseFailed;
            bdb_diag!("[BDB] nwk_parse=fail len={}", mac_payload.len());
            let dump_len = mac_payload.len().min(20);
            let hex: heapless::String<60> =
                mac_payload[..dump_len]
                    .iter()
                    .fold(heapless::String::new(), |mut s, b| {
                        let _ = core::fmt::Write::write_fmt(&mut s, format_args!("{:02X}", b));
                        s
                    });
            log::warn!(
                "[BDB:Steering] NWK parse FAIL: len={} {}",
                mac_payload.len(),
                hex
            );
            None
        } else {
            None
        }
    }

    /// Process a received MAC frame during Transport-Key wait.
    ///
    /// Parses NWK header and security, attempts decrypt if needed, then processes
    /// via APS layer. Returns `Some(true)` if NWK key was installed (Transport-Key
    /// received), `Some(false)` if frame was processed but no key, `None` if
    /// parsing/decrypt failed.
    fn process_key_wait_frame(
        &mut self,
        mac_payload: &[u8],
        nwk_hdr: &zigbee_nwk::frames::NwkHeader,
        nwk_consumed: usize,
        lqi: u8,
    ) -> Option<bool> {
        let after_nwk = &mac_payload[nwk_consumed..];
        let mut buf = [0u8; 128];
        let payload_data: Option<([u8; 128], usize)>;

        if nwk_hdr.frame_control.security {
            let parse_result = zigbee_nwk::security::NwkSecurityHeader::parse(after_nwk);
            if let Some((sec_hdr, sec_consumed)) = parse_result {
                bdb_diag!(
                    "[BDB] nwk_sec key_seq={} sec_used={} cipher_len={}",
                    sec_hdr.key_seq_number,
                    sec_consumed,
                    after_nwk.len().saturating_sub(sec_consumed)
                );
                if let Some(key_entry) = self
                    .zdo
                    .aps()
                    .nwk()
                    .security()
                    .key_by_seq(sec_hdr.key_seq_number)
                {
                    let key = key_entry.key;
                    let aad_len = nwk_consumed + sec_consumed;
                    // AAD must use ACTUAL security level (5), not OTA value (0).
                    let mut aad_buf = [0u8; 64];
                    let aad_copy_len = aad_len.min(aad_buf.len());
                    aad_buf[..aad_copy_len].copy_from_slice(&mac_payload[..aad_copy_len]);
                    aad_buf[nwk_consumed] = (aad_buf[nwk_consumed] & !0x07) | 0x05;
                    let active_pt = self.zdo.aps().nwk().security().decrypt(
                        &aad_buf[..aad_copy_len],
                        &after_nwk[sec_consumed..],
                        &key,
                        &sec_hdr,
                    );
                    if let Some(pt) = active_pt {
                        bdb_diag!("[BDB] nwk_decrypt=ok active_key len={}", pt.len());
                        let len = pt.len().min(128);
                        buf[..len].copy_from_slice(&pt[..len]);
                        payload_data = Some((buf, len));
                    } else {
                        self.steering_diagnostics.key_frame_result =
                            KeyFrameResult::ActiveKeyDecryptFailed;
                        bdb_diag!("[BDB] nwk_decrypt=fail active_key");
                        log::warn!("[BDB:Steering] NWK decrypt failed");
                        payload_data = None;
                    }
                } else {
                    // No active NWK key yet. Per Zigbee Pro spec §4.5.3 the
                    // Transport-Key arrives as a **sec=0 NWK** frame carrying
                    // an APS Transport-Key command encrypted with the *APS-
                    // layer* KT key (HMAC-derived from the TC link key). The
                    // KT key is NOT a NWK-layer key — attempting to use it
                    // here against sec=1 broadcasts only burns cycles and,
                    // worse, can hang RustCrypto `ccm` on certain inputs
                    // (observed on TLSR8258 after 3 successful failures on
                    // the 4th call → stalls the steering loop indefinitely).
                    //
                    // The correct sec=0 path lives in the `else` branch
                    // below (`if nwk_hdr.frame_control.security` false →
                    // pass after_nwk to `process_incoming_aps_frame`, which
                    // applies the KT key at the APS layer where it belongs).
                    //
                    // So: when sec=1 and we have no NWK key, simply drop.
                    self.steering_diagnostics.key_frame_result = KeyFrameResult::SecuredNoActiveKey;
                    bdb_diag!("[BDB] sec=1 no_active_key — drop (KT is APS-layer, not NWK)");
                    payload_data = None;
                }
            } else {
                self.steering_diagnostics.key_frame_result =
                    KeyFrameResult::SecurityHeaderParseFailed;
                bdb_diag!("[BDB] nwk_sec=parse_fail len={}", after_nwk.len());
                log::warn!("[BDB:Steering] NWK security header parse failed");
                payload_data = None;
            }
        } else {
            // NWK security OFF — this is what Transport-Key looks like
            bdb_diag!("[BDB] nwk_unsecured after_nwk={}", after_nwk.len());
            log::info!(
                "[BDB:Steering] NWK unsecured frame! {} bytes — possible Transport-Key",
                after_nwk.len()
            );
            let len = after_nwk.len().min(128);
            buf[..len].copy_from_slice(&after_nwk[..len]);
            payload_data = Some((buf, len));
            self.steering_diagnostics.key_frame_result = KeyFrameResult::UnsecuredAps;
        }

        if let Some((data, len)) = payload_data {
            let mut aps_buf = zigbee_aps::apsde::ApsFrameBuffer::new();
            // Log first 20 bytes hex for debugging APS parsing
            if len >= 4 {
                bdb_diag!(
                    "[BDB] aps first={:02X} {:02X} {:02X} {:02X} len={}",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    len
                );
                log::info!(
                    "[BDB:Steering] APS payload hex: {:02X} {:02X} {:02X} {:02X} (len={})",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    len,
                );
            }

            let indication = {
                self.zdo.aps_mut().process_incoming_aps_frame(
                    &data[..len],
                    nwk_hdr.src_addr,
                    nwk_hdr.dst_addr,
                    lqi,
                    nwk_hdr.frame_control.security,
                    &mut aps_buf,
                )
            };
            if let Some(indication) = indication {
                let _ = self.zdo.deliver_client_response(&indication);
            }

            if self.zdo.aps().nwk().security().active_key().is_some() {
                self.steering_diagnostics.key_frame_result = KeyFrameResult::KeyInstalled;
                bdb_diag!("[BDB] aps_process=key_installed");
                log::info!("[BDB:Steering] NWK key received from TC!");
                return Some(true);
            }
            self.steering_diagnostics.key_frame_result = KeyFrameResult::ApsProcessedNoKey;
            bdb_diag!("[BDB] aps_process=no_key");
            log::info!("[BDB:Steering] APS processed but no key installed yet");
            Some(false)
        } else {
            bdb_diag!("[BDB] payload_data=none");
            None
        }
    }
}
