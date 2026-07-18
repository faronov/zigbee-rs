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

        networks.sort_unstable_by_key(|n| core::cmp::Reverse(n.lqi));

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
        self.nib.update_id = network.update_id;
        self.nib.stack_profile = network.stack_profile;
        self.nib.parent_address = join_target;

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
            relationship: Relationship::Parent,
            lqi: network.lqi,
            outgoing_cost: 1,
            depth: network.depth,
            permit_joining: network.permit_joining,
            age: 0,
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
                if let Some(encrypted) = self.security.encrypt(
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
        self.mac
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
            .await
            .map_err(|error| match error {
                MacError::NoAck | MacError::ChannelAccessFailure => NwkStatus::NoNetworks,
                _ => NwkStatus::StartupFailure,
            })?;

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
                    || !self
                        .security
                        .check_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter)
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
                match self.security.decrypt(
                    &aad_buf[..copy_len],
                    &after_hdr[sec_consumed..],
                    &key,
                    &sec_hdr,
                ) {
                    Some(v) => {
                        self.security
                            .commit_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter);
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
                    log::info!("[NWK] Rejoin accepted, new addr=0x{:04X}", new_addr);
                    self.nib.network_address = ShortAddress(new_addr);
                    // Refresh parent address to the sender of the rejoin response
                    self.nib.parent_address = hdr.src_addr;
                    self.nib.extended_pan_id = network.extended_pan_id;
                    self.nib.pan_id = network.pan_id;
                    self.nib.logical_channel = network.logical_channel;
                    self.nib.update_id = network.update_id;
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
                        relationship: Relationship::Parent,
                        lqi: network.lqi,
                        outgoing_cost: 1,
                        depth: network.depth,
                        permit_joining: network.permit_joining,
                        age: 0,
                        extended_pan_id: network.extended_pan_id,
                        active: true,
                    };
                    let _ = self.neighbors.add_or_update(parent);
                    self.joined = true;
                    return Ok(ShortAddress(new_addr));
                } else {
                    log::warn!("[NWK] Rejoin rejected (status=0x{:02X})", rejoin_status);
                    return Err(NwkStatus::NotPermitted);
                }
            }
        }

        log::warn!(
            "[NWK] Rejoin response not received after {} attempts",
            attempt
        );
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
            dst_addr: self.nib.parent_address,
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
                if let Some(encrypted) =
                    self.security
                        .encrypt(&buf[..aad_len], &payload, &key_entry.key, &sec_hdr)
                {
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
                dst_address: MacAddress::Short(self.nib.pan_id, self.nib.parent_address),
                payload: &buf[..total_len],
                msdu_handle: seq,
                tx_options: zigbee_mac::TxOptions {
                    ack_tx: true,
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

        // Reset state — clear all network-scoped state
        self.joined = false;
        self.nib.network_address = ShortAddress(0xFFFF);
        self.nib.pan_id = PanId(0xFFFF);
        self.nib.parent_address = ShortAddress(0xFFFF);
        self.nib.logical_channel = 0;
        self.nib.depth = 0;
        self.nib.extended_pan_id = [0u8; 8];
        self.neighbors = crate::neighbor::NeighborTable::new();
        self.routing = crate::routing::RoutingTable::new();
        if !rejoin {
            // Full leave — also clear security state
            self.security = crate::security::NwkSecurity::new();
        }

        log::info!("[NWK] Left network, rejoin={rejoin}");

        Ok(())
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

    /// Scan on `mask` and attempt rejoin on the first network matching
    /// `ext_pan`.
    async fn try_rejoin_on_mask(
        &mut self,
        mask: ChannelMask,
        ext_pan: &IeeeAddress,
    ) -> Result<ShortAddress, NwkStatus> {
        let networks = self.nlme_network_discovery(mask, 3).await?;
        for net in &networks {
            if net.extended_pan_id == *ext_pan {
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

        self.nib.permit_joining = duration != 0;
        self.nib.permit_joining_duration = duration;

        // Update MAC
        self.mac
            .mlme_set(
                PibAttribute::MacAssociationPermit,
                PibValue::Bool(duration != 0),
            )
            .await
            .map_err(|_| NwkStatus::InvalidRequest)?;

        log::info!("[NWK] Permit joining: duration={duration}");
        Ok(())
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
    pub async fn nlme_reset(&mut self, warm_start: bool) -> Result<(), NwkStatus> {
        if !warm_start {
            self.nib = Nib::new();
            self.neighbors = crate::neighbor::NeighborTable::new();
            self.routing = crate::routing::RoutingTable::new();
            self.security = crate::security::NwkSecurity::new();
            self.joined = false;
        }

        self.mac
            .mlme_reset(!warm_start)
            .await
            .map_err(|_| NwkStatus::InvalidRequest)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_mac::mock::MockMac;

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

        assert_eq!(
            block_on(nwk.nlme_join(&network, JoinMethod::Rejoin)),
            Err(NwkStatus::NotPermitted)
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
