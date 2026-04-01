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

use zigbee_mac::pib::{PibAttribute, PibValue};
use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::ShortAddress;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{BdbLayer, BdbStatus};

/// Default scan duration exponent for active scan (2^n + 1 superframes).
/// Exponent 3 ≈ 138 ms per channel — good balance of speed vs. reliability.
const SCAN_DURATION: u8 = 3;

impl<M: MacDriver> BdbLayer<M> {
    /// Execute the Network Steering procedure (BDB spec §8.3).
    ///
    /// Behaviour depends on `bdbNodeIsOnANetwork`:
    /// - **Not on network**: scan → join → announce → TC key exchange
    /// - **On network**: open permit joining → broadcast Mgmt_Permit_Joining_req
    pub async fn network_steering(&mut self) -> Result<(), BdbStatus> {
        if self.attributes.node_is_on_a_network {
            self.steer_on_network().await
        } else {
            self.steer_off_network().await
        }
    }

    /// Steering when the device is NOT on a network — join an existing PAN.
    async fn steer_off_network(&mut self) -> Result<(), BdbStatus> {
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

        // Try primary channels first, then secondary
        let channel_sets = [
            self.attributes.primary_channel_set,
            self.attributes.secondary_channel_set,
        ];

        for (idx, &channel_mask) in channel_sets.iter().enumerate() {
            if channel_mask.0 == 0 {
                continue;
            }

            let set_name = if idx == 0 { "primary" } else { "secondary" };
            log::debug!(
                "[BDB:Steering] Scanning {} channels: 0x{:08X}",
                set_name,
                channel_mask.0
            );

            // Step 1: Network discovery
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

            // Step 2: Filter by extended PAN ID if configured
            let use_epid = self.zdo.aps().aib().aps_use_extended_pan_id;
            let has_epid_filter = use_epid != [0u8; 8];

            // Debug: show all discovered networks
            for (i, network) in networks.iter().enumerate() {
                log::info!(
                    "[BDB:Steering] net[{}] PAN=0x{:04X} ch={} d={} permit={} LQI={} via 0x{:04X}",
                    i, network.pan_id.0, network.logical_channel, network.depth,
                    network.permit_joining, network.lqi, network.router_address.0,
                );
            }

            // Step 3: Try joining networks — prefer coordinator (depth=0) for
            // reliable Transport-Key delivery. Some routers don't properly relay
            // the "Update Device" APS command to the TC, so direct coordinator
            // association is more reliable.
            // First pass: coordinator beacons only (depth == 0)
            // Second pass: all other routers
            for prefer_coordinator in [true, false] {
            for network in &networks {
                // Apply extended PAN ID filter
                if has_epid_filter && network.extended_pan_id != use_epid {
                    log::debug!(
                        "[BDB:Steering] Skipping PAN 0x{:04X} — EPID mismatch",
                        network.pan_id.0,
                    );
                    continue;
                }

                // Must have permit joining enabled
                if !network.permit_joining {
                    continue;
                }

                // Two-pass: coordinators first, then routers
                let is_coordinator = network.depth == 0;
                if prefer_coordinator && !is_coordinator {
                    continue;
                }
                if !prefer_coordinator && is_coordinator {
                    continue; // already tried
                }

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
                        joined_addr = Some(addr);
                        break;
                    }
                    Err(e) => {
                        log::warn!("[BDB:Steering] Join failed: {:?}", e);
                        continue;
                    }
                }
                }
                let nwk_addr = match joined_addr {
                    Some(a) => a,
                    None => continue,
                };

                // Step 4: Announce our presence
                let ieee = self.zdo.nwk().nib().ieee_address;
                if let Err(e) = self.zdo.device_annce(nwk_addr, ieee).await {
                    log::warn!("[BDB:Steering] Device_annce failed: {:?}", e);
                    // Non-fatal — continue commissioning
                }

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

                let mut key_received = false;
                // With rx_on_when_idle=true in CapabilityInfo, the coordinator sends
                // Transport-Key as a DIRECT unicast (not indirect). We must be in RX
                // mode to catch it. Do a passive listen first, then fall back to polling.

                // Phase 0: Passive RX listen — catch direct unicast Transport-Key
                log::info!("[BDB:Steering] Phase 0: passive RX for direct Transport-Key...");
                for rx_attempt in 0..10u8 {
                    match self.zdo.aps_mut().nwk_mut().mac_mut().mcps_data_indication().await {
                        Ok(mac_frame) => {
                            let mac_payload = mac_frame.payload.as_slice();
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
                            // Timeout — no frame received
                        }
                    }
                }

                if key_received {
                    log::info!("[BDB:Steering] Transport-Key received during passive RX!");
                    // fall through to success path below
                }

                // Phase 1+2: Poll parent and coordinator if passive RX didn't work
                let parent_addr = self.zdo.nwk().nib().parent_address;

                const MAX_TOTAL_ROUNDS: usize = 40;
                const MAX_EMPTY_ROUNDS: u8 = 10;
                let mut empty_count: u8 = 0;
                let mut total_rounds: usize = 0;
                let mut data_frames: usize = 0;

                while !key_received && total_rounds < MAX_TOTAL_ROUNDS && empty_count < MAX_EMPTY_ROUNDS {
                    total_rounds += 1;
                    let mut got_data_this_round = false;

                    // Poll parent for indirect frames
                    match self.zdo.aps_mut().nwk_mut().mac_mut().mlme_poll().await {
                        Ok(Some(mac_frame)) => {
                            got_data_this_round = true;
                            data_frames += 1;
                            let mac_payload = mac_frame.as_slice();
                            log::info!(
                                "[BDB:Steering] P-Poll {}: {} bytes (total={})",
                                total_rounds,
                                mac_payload.len(),
                                data_frames,
                            );
                            if let Some(true) = self.try_process_frame(mac_payload) {
                                key_received = true;
                                break;
                            }
                        }
                        Ok(None) => {}
                        Err(e) => {
                            log::warn!("[BDB:Steering] P-Poll {}: err {:?}", total_rounds, e);
                        }
                    }

                    if key_received {
                        break;
                    }

                    // Phase 2: Poll coordinator (0x0000) for indirect frames.
                    // The EZSP NCP may queue Transport-Key in the coordinator's own
                    // indirect buffer (not routed through parent), especially when
                    // parentOfNewNodeId in trustCenterJoinHandler points to the coordinator.
                    {
                        let mac = self.zdo.aps_mut().nwk_mut().mac_mut();
                        let _ = mac
                            .mlme_set(
                                PibAttribute::MacCoordShortAddress,
                                PibValue::ShortAddress(ShortAddress(0x0000)),
                            )
                            .await;
                        match mac.mlme_poll().await {
                            Ok(Some(mac_frame)) => {
                                got_data_this_round = true;
                                data_frames += 1;
                                let mac_payload = mac_frame.as_slice();
                                log::info!(
                                    "[BDB:Steering] C-Poll {}: {} bytes (total={})",
                                    total_rounds,
                                    mac_payload.len(),
                                    data_frames,
                                );
                                if let Some(true) = self.try_process_frame(mac_payload) {
                                    key_received = true;
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                log::debug!("[BDB:Steering] C-Poll {}: err {:?}", total_rounds, e);
                            }
                        }
                        // Restore parent address
                        let mac = self.zdo.aps_mut().nwk_mut().mac_mut();
                        let _ = mac
                            .mlme_set(
                                PibAttribute::MacCoordShortAddress,
                                PibValue::ShortAddress(parent_addr),
                            )
                            .await;
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

                if !key_received {
                    log::warn!(
                        "[BDB:Steering] Transport-Key NOT received after {} rounds ({} data frames, {} consecutive empty)",
                        total_rounds, data_frames, empty_count,
                    );
                }

                if !key_received {
                    log::warn!(
                        "[BDB:Steering] Transport-Key not received — leaving PAN 0x{:04X}",
                        network.pan_id.0,
                    );
                    // Leave cleanly so the TC can clean up its state.
                    // Then continue to try the next PAN in the scan list.
                    let _ = self.zdo.nwk_mut().nlme_leave(false).await;
                    continue;
                }

                // Step 5c: Re-send Device_annce now that we have the NWK key
                // (first attempt was pre-key and likely failed with NoAck)
                if let Err(e) = self.zdo.device_annce(nwk_addr, ieee).await {
                    log::warn!("[BDB:Steering] Device_annce (post-key) failed: {:?}", e);
                }

                // Step 5d: Send APSME-REQUEST-KEY to TC for unique link key
                // Z2M requires this within ~10s of joining
                let tc_addr = zigbee_types::ShortAddress::COORDINATOR;
                if let Err(e) = self.zdo.aps_mut().send_request_key(tc_addr).await {
                    log::warn!("[BDB:Steering] Request-Key failed: {:?}", e);
                    // Non-fatal — some coordinators don't require this
                }

                // Success!
                self.attributes.node_is_on_a_network = true;
                self.attributes.commissioning_status =
                    crate::attributes::BdbCommissioningStatus::Success;

                log::info!("[BDB:Steering] Joined successfully as 0x{:04X}", nwk_addr.0,);
                return Ok(());
            }
            } // end prefer_coordinator pass
        }

        // All attempts exhausted
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::NoScanResponse;
        Err(BdbStatus::NoScanResponse)
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
        if let Some((nwk_hdr, nwk_consumed)) =
            zigbee_nwk::frames::NwkHeader::parse(mac_payload)
        {
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
                let hex: heapless::String<96> = mac_payload[..dump_len]
                    .iter()
                    .fold(heapless::String::new(), |mut s, b| {
                        let _ = core::fmt::Write::write_fmt(
                            &mut s,
                            format_args!("{:02X}", b),
                        );
                        s
                    });
                log::info!("[BDB:Steering] COORD hex: {}", hex);
            }
            self.process_key_wait_frame(mac_payload, &nwk_hdr, nwk_consumed, 0)
        } else if mac_payload.len() > 2 {
            let dump_len = mac_payload.len().min(20);
            let hex: heapless::String<60> = mac_payload[..dump_len]
                .iter()
                .fold(heapless::String::new(), |mut s, b| {
                    let _ = core::fmt::Write::write_fmt(
                        &mut s,
                        format_args!("{:02X}", b),
                    );
                    s
                });
            log::warn!("[BDB:Steering] NWK parse FAIL: len={} {}", mac_payload.len(), hex);
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
        let mut payload_data: Option<([u8; 128], usize)> = None;

        if nwk_hdr.frame_control.security {
            if let Some((sec_hdr, sec_consumed)) =
                zigbee_nwk::security::NwkSecurityHeader::parse(after_nwk)
            {
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
                    if let Some(pt) = self.zdo.aps().nwk().security().decrypt(
                        &aad_buf[..aad_copy_len],
                        &after_nwk[sec_consumed..],
                        &key,
                        &sec_hdr,
                    ) {
                        let len = pt.len().min(128);
                        buf[..len].copy_from_slice(&pt[..len]);
                        payload_data = Some((buf, len));
                    } else {
                        log::warn!("[BDB:Steering] NWK decrypt failed");
                        payload_data = None;
                    }
                } else {
                    // No NWK key yet — try decrypting with the well-known TC link key.
                    // Some EZSP coordinators wrap Transport-Key in NWK security using
                    // a key the joining device is expected to derive/know.
                    let tc_link_key: [u8; 16] = *b"ZigBeeAlliance09";
                    let ciphertext = &after_nwk[sec_consumed..];
                    let aad_len = nwk_consumed + sec_consumed;
                    let mut aad_buf = [0u8; 64];
                    let aad_copy_len = aad_len.min(aad_buf.len());
                    aad_buf[..aad_copy_len].copy_from_slice(&mac_payload[..aad_copy_len]);

                    // Try 1: TC link key with patched AAD (level 0→5, standard)
                    let mut tc_aad = aad_buf;
                    tc_aad[nwk_consumed] = (tc_aad[nwk_consumed] & !0x07) | 0x05;
                    let mut decrypted = false;

                    if let Some(pt) = self.zdo.aps().nwk().security().decrypt(
                        &tc_aad[..aad_copy_len],
                        ciphertext,
                        &tc_link_key,
                        &sec_hdr,
                    ) {
                        log::info!(
                            "[BDB:Steering] TC link key decrypt SUCCESS (patched)! {} bytes",
                            pt.len()
                        );
                        let len = pt.len().min(128);
                        buf[..len].copy_from_slice(&pt[..len]);
                        payload_data = Some((buf, len));
                        decrypted = true;
                    }

                    // Try 2: TC link key with original AAD (no level patching)
                    if !decrypted {
                        if let Some(pt) = self.zdo.aps().nwk().security().decrypt(
                            &aad_buf[..aad_copy_len],
                            ciphertext,
                            &tc_link_key,
                            &sec_hdr,
                        ) {
                            log::info!(
                                "[BDB:Steering] TC link key decrypt SUCCESS (raw)! {} bytes",
                                pt.len()
                            );
                            let len = pt.len().min(128);
                            buf[..len].copy_from_slice(&pt[..len]);
                            payload_data = Some((buf, len));
                            decrypted = true;
                        }
                    }

                    // Try 3: Key-Transport key derived from TC link key
                    if !decrypted {
                        let kt_key = zigbee_aps::security::derive_key_transport_key(&tc_link_key);
                        if let Some(pt) = self.zdo.aps().nwk().security().decrypt(
                            &tc_aad[..aad_copy_len],
                            ciphertext,
                            &kt_key,
                            &sec_hdr,
                        ) {
                            log::info!(
                                "[BDB:Steering] Key-transport key decrypt SUCCESS! {} bytes",
                                pt.len()
                            );
                            let len = pt.len().min(128);
                            buf[..len].copy_from_slice(&pt[..len]);
                            payload_data = Some((buf, len));
                            decrypted = true;
                        }
                    }

                    if !decrypted {
                        // Log only unicast frames (likely meant for us)
                        if nwk_hdr.dst_addr.0 != 0xFFFF
                            && nwk_hdr.dst_addr.0 != 0xFFFC
                            && nwk_hdr.dst_addr.0 != 0xFFFD
                        {
                            let total = after_nwk.len().min(16);
                            if total >= 6 {
                                log::info!(
                                    "[BDB:Steering] Undecryptable unicast: sec_ctrl={:02X} fc={:02X}{:02X}{:02X}{:02X} ks={} len={}",
                                    after_nwk[0],
                                    after_nwk[1], after_nwk[2], after_nwk[3], after_nwk[4],
                                    sec_hdr.key_seq_number,
                                    ciphertext.len(),
                                );
                            }
                        }
                        payload_data = None;
                    }
                }
            } else {
                log::warn!("[BDB:Steering] NWK security header parse failed");
                payload_data = None;
            }
        } else {
            // NWK security OFF — this is what Transport-Key looks like
            log::info!(
                "[BDB:Steering] NWK unsecured frame! {} bytes — possible Transport-Key",
                after_nwk.len()
            );
            let len = after_nwk.len().min(128);
            buf[..len].copy_from_slice(&after_nwk[..len]);
            payload_data = Some((buf, len));
        }

        if let Some((data, len)) = payload_data {
            let mut aps_buf = zigbee_aps::apsde::ApsFrameBuffer::new();
            // Log first 20 bytes hex for debugging APS parsing
            if len >= 4 {
                log::info!(
                    "[BDB:Steering] APS payload hex: {:02X} {:02X} {:02X} {:02X} (len={})",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    len,
                );
            }

            let _ = self.zdo.aps_mut().process_incoming_aps_frame(
                &data[..len],
                nwk_hdr.src_addr,
                nwk_hdr.dst_addr,
                lqi,
                nwk_hdr.frame_control.security,
                &mut aps_buf,
            );

            if self.zdo.aps().nwk().security().active_key().is_some() {
                log::info!("[BDB:Steering] NWK key received from TC!");
                return Some(true);
            }
            log::info!("[BDB:Steering] APS processed but no key installed yet");
            Some(false)
        } else {
            None
        }
    }
}
