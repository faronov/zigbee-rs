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
        // Check retry budget
        if self.attributes.steering_attempts_remaining == 0 {
            log::warn!("[BDB:Steering] No steering attempts remaining");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::SteeringFormationFailure;
            return Err(BdbStatus::SteeringFailure);
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

            log::debug!("[BDB:Steering] Found {} network(s)", networks.len());

            // Step 2: Filter by extended PAN ID if configured
            let use_epid = self.zdo.aps().aib().aps_use_extended_pan_id;
            let has_epid_filter = use_epid != [0u8; 8];

            // Step 3: Try joining each network (already sorted by LQI)
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

                log::info!(
                    "[BDB:Steering] Joining PAN 0x{:04X} ch {} LQI {}",
                    network.pan_id.0,
                    network.logical_channel,
                    network.lqi,
                );

                // Step 3: Attempt join
                let nwk_addr = match self.zdo.nlme_join(network).await {
                    Ok(addr) => addr,
                    Err(e) => {
                        log::warn!("[BDB:Steering] Join failed: {:?}", e);
                        continue;
                    }
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
                // Poll parent for pending frames — wait up to ~30s for Transport-Key.
                // Because we declared rx_on_when_idle=false, the parent uses indirect
                // delivery: it stores frames for us and only sends them when we poll
                // with a MAC Data Request command.
                const MAX_KEY_WAIT_ATTEMPTS: usize = 15;
                for attempt in 0..MAX_KEY_WAIT_ATTEMPTS {
                    log::info!(
                        "[BDB:Steering] Key wait attempt {}/{} — polling parent...",
                        attempt + 1,
                        MAX_KEY_WAIT_ATTEMPTS
                    );

                    // Send MAC Data Request to parent to retrieve pending indirect frames.
                    // The parent will respond with any buffered Transport-Key frame.
                    match self
                        .zdo
                        .aps_mut()
                        .nwk_mut()
                        .mac_mut()
                        .mlme_poll()
                        .await
                    {
                        Ok(Some(mac_frame)) => {
                            let mac_payload = mac_frame.as_slice();
                            log::info!(
                                "[BDB:Steering] Poll returned frame, {} bytes",
                                mac_payload.len(),
                            );
                            if let Some((nwk_hdr, nwk_consumed)) =
                                zigbee_nwk::frames::NwkHeader::parse(mac_payload)
                            {
                                log::info!(
                                    "[BDB:Steering] Poll NWK: src=0x{:04X} dst=0x{:04X} sec={}",
                                    nwk_hdr.src_addr.0,
                                    nwk_hdr.dst_addr.0,
                                    nwk_hdr.frame_control.security,
                                );
                                if let Some(true) = self.process_key_wait_frame(
                                    mac_payload, &nwk_hdr, nwk_consumed, 0,
                                ) {
                                    key_received = true;
                                    break;
                                }
                            }
                        }
                        Ok(None) => {
                            log::info!("[BDB:Steering] Poll: no pending data from parent");
                        }
                        Err(e) => {
                            log::warn!("[BDB:Steering] Poll failed: {:?}", e);
                        }
                    }

                    if key_received {
                        break;
                    }

                    // Also listen passively for a short time — the Transport-Key might
                    // arrive as a direct transmission (if parent treats us as RxOnWhenIdle)
                    match self
                        .zdo
                        .aps_mut()
                        .nwk_mut()
                        .mac_mut()
                        .mcps_data_indication()
                        .await
                    {
                        Ok(mac_ind) => {
                            log::info!(
                                "[BDB:Steering] Got MAC frame, payload {} bytes, LQI {}",
                                mac_ind.payload.as_slice().len(),
                                mac_ind.lqi,
                            );
                            let mac_payload = mac_ind.payload.as_slice();
                            if let Some((nwk_hdr, nwk_consumed)) =
                                zigbee_nwk::frames::NwkHeader::parse(mac_payload)
                            {
                                log::info!(
                                    "[BDB:Steering] NWK frame: src=0x{:04X} dst=0x{:04X} sec={}",
                                    nwk_hdr.src_addr.0,
                                    nwk_hdr.dst_addr.0,
                                    nwk_hdr.frame_control.security,
                                );
                                if let Some(true) = self.process_key_wait_frame(
                                    mac_payload, &nwk_hdr, nwk_consumed, mac_ind.lqi,
                                ) {
                                    key_received = true;
                                    break;
                                }
                            } else {
                                log::warn!("[BDB:Steering] NWK header parse failed");
                            }
                        }
                        Err(e) => {
                            log::debug!("[BDB:Steering] Key wait: rx error {:?}", e);
                        }
                    }
                }

                if !key_received {
                    log::warn!("[BDB:Steering] Transport-Key not received within timeout");
                    // Continue anyway — the key might arrive later
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
        let payload_data;

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
                    if let Some(pt) = self.zdo.aps().nwk().security().decrypt(
                        &mac_payload[..aad_len],
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
                    log::warn!(
                        "[BDB:Steering] No key for seq {}",
                        sec_hdr.key_seq_number
                    );
                    payload_data = None;
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
                    data[0], data[1], data[2], data[3], len,
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
