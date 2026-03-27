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

        let mut networks = heapless::Vec::new();
        for pd in &scan_result.pan_descriptors {
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
            let nd = NetworkDescriptor::from(pd);
            let _ = networks.push(nd);
        }

        if networks.is_empty() {
            return Err(NwkStatus::NoNetworks);
        }

        // Sort by LQI (best signal first)
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

        // Generate random PAN ID (avoid 0xFFFF)
        let pan_id = PanId(generate_pan_id());

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
        // Check capacity
        match self.device_type {
            DeviceType::Router if !network.router_capacity => {
                return Err(NwkStatus::NotPermitted);
            }
            DeviceType::EndDevice if !network.end_device_capacity => {
                return Err(NwkStatus::NotPermitted);
            }
            _ => {}
        }

        if !network.permit_joining {
            return Err(NwkStatus::NotPermitted);
        }

        // Build capability info
        let cap = CapabilityInfo {
            device_type_ffd: self.device_type != DeviceType::EndDevice,
            mains_powered: self.device_type != DeviceType::EndDevice,
            rx_on_when_idle: self.rx_on_when_idle,
            security_capable: false,
            allocate_address: true,
        };

        // Perform MAC association — use discovered router address, not hardcoded coordinator
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
            .map_err(|e| match e {
                MacError::NoAck => NwkStatus::NoNetworks,
                _ => NwkStatus::StartupFailure,
            })?;

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
                PibValue::Bool(self.device_type != DeviceType::EndDevice),
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

        // Build NWK Rejoin Request frame
        let cap_byte = CapabilityInfo {
            device_type_ffd: self.device_type != DeviceType::EndDevice,
            mains_powered: self.device_type != DeviceType::EndDevice,
            rx_on_when_idle: self.device_type != DeviceType::EndDevice,
            security_capable: false,
            allocate_address: true,
        };

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
                end_device_initiator: self.device_type == DeviceType::EndDevice,
            },
            dst_addr: ShortAddress::BROADCAST,
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
                frame_counter: self.nib.next_frame_counter().unwrap_or(0),
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
        let _ = self
            .mac
            .mcps_data(zigbee_mac::McpsDataRequest {
                src_addr_mode: zigbee_mac::AddressMode::Short,
                dst_address: MacAddress::Short(network.pan_id, ShortAddress::BROADCAST),
                payload: &nwk_frame_buf[..total_len],
                msdu_handle: seq,
                tx_options: zigbee_mac::TxOptions::default(),
            })
            .await;

        // Wait for Rejoin Response (NWK command 0x07).
        // Try receiving up to MAX_RX_ATTEMPTS frames; give up if none is
        // the expected rejoin response. This avoids embassy_time dependency.
        const MAX_RX_ATTEMPTS: usize = 16;

        for _ in 0..MAX_RX_ATTEMPTS {
            let indication = match self.mac.mcps_data_indication().await {
                Ok(ind) => ind,
                Err(_) => continue,
            };

            let data = indication.payload.as_slice();
            let (hdr, consumed) = match NwkHeader::parse(data) {
                Some(v) => v,
                None => continue,
            };

            // Must be a NWK Command frame
            if hdr.frame_control.frame_type != NwkFrameType::Command as u8 {
                continue;
            }

            // Get command payload — may need NWK decryption
            let cmd_data = if hdr.frame_control.security {
                let after_hdr = &data[consumed..];
                let (sec_hdr, sec_consumed) =
                    match crate::security::NwkSecurityHeader::parse(after_hdr) {
                        Some(v) => v,
                        None => continue,
                    };
                let key = match self.security.key_by_seq(sec_hdr.key_seq_number) {
                    Some(k) => k.key,
                    None => continue,
                };
                let aad_len = consumed + sec_consumed;
                match self.security.decrypt(
                    &data[..aad_len],
                    &after_hdr[sec_consumed..],
                    &key,
                    &sec_hdr,
                ) {
                    Some(v) => v,
                    None => continue,
                }
            } else {
                let payload = &data[consumed..];
                let mut v = heapless::Vec::<u8, 128>::new();
                let _ = v.extend_from_slice(payload);
                v
            };

            // Rejoin Response: cmd_id(0x07) + new_short_addr(2) + rejoin_status(1)
            if cmd_data.len() >= 4 && cmd_data[0] == 0x07 {
                let new_addr = u16::from_le_bytes([cmd_data[1], cmd_data[2]]);
                let rejoin_status = cmd_data[3];

                if rejoin_status == 0x00 {
                    log::info!("[NWK] Rejoin accepted, new addr=0x{:04X}", new_addr);
                    self.nib.network_address = ShortAddress(new_addr);
                    // Refresh parent address to the sender of the rejoin response
                    self.nib.parent_address = hdr.src_addr;
                    // Update depth from beacon (parent depth + 1)
                    self.nib.depth = network.depth.saturating_add(1);
                    let _ = self
                        .mac
                        .mlme_set(
                            PibAttribute::MacShortAddress,
                            PibValue::ShortAddress(ShortAddress(new_addr)),
                        )
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
            MAX_RX_ATTEMPTS
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
                frame_counter: self.nib.next_frame_counter().unwrap_or(0),
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

/// Simple PAN ID generation (should use proper RNG in production).
fn generate_pan_id() -> u16 {
    // Use a simple PRNG seed — real implementation should use hardware RNG
    static mut SEED: u32 = 0xDEAD_BEEF;
    unsafe {
        SEED ^= SEED << 13;
        SEED ^= SEED >> 17;
        SEED ^= SEED << 5;
        let pan = (SEED & 0x3FFF) as u16; // Avoid reserved range
        if pan == 0xFFFF { 0x1234 } else { pan }
    }
}
