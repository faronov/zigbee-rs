//! NLDE — NWK Layer Data Entity.
//!
//! Handles sending and receiving NWK data frames via the MAC layer.
//! - NLDE-DATA.request: send NWK data to a destination
//! - NLDE-DATA.indication: receive NWK data from the network
//! - Frame relay for routers/coordinators

use crate::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
use crate::{DeviceType, NwkLayer, NwkStatus};
use zigbee_mac::{AddressMode, MacDriver, McpsDataRequest, TxOptions};
use zigbee_types::*;

/// NWK data indication — received NWK-level data.
#[derive(Debug)]
pub struct NldeDataIndication<'a> {
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub payload: &'a [u8],
    pub lqi: u8,
    pub security_use: bool,
}

/// NWK data confirm — result of NLDE-DATA.request.
#[derive(Debug)]
pub struct NldeDataConfirm {
    pub status: NwkStatus,
    pub nsdu_handle: u8,
}

impl<M: MacDriver> NwkLayer<M> {
    /// Send NWK data to a destination address.
    ///
    /// This is the primary data service used by the APS layer above.
    /// It builds a NWK frame, determines the MAC next-hop, and sends via MAC.
    pub async fn nlde_data_request(
        &mut self,
        dst_addr: ShortAddress,
        radius: u8,
        payload: &[u8],
        security_enable: bool,
        discover_route: bool,
    ) -> Result<NldeDataConfirm, NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }

        let seq = self.nib.next_seq();

        // Build NWK header
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: if discover_route { 1 } else { 0 },
                multicast: false,
                security: security_enable && self.nib.security_enabled,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: self.device_type == DeviceType::EndDevice,
            },
            dst_addr,
            src_addr: self.nib.network_address,
            radius,
            seq_number: seq,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };

        // Serialize NWK frame
        let mut nwk_buf = [0u8; 128];
        let hdr_len = header.serialize(&mut nwk_buf);

        // Copy payload
        if hdr_len + payload.len() > nwk_buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        nwk_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        let total_len = hdr_len + payload.len();

        // TODO: If security_enable, encrypt with NWK key here
        // let encrypted = self.security.encrypt(...);

        // Determine MAC-level next hop
        let next_hop = self.resolve_next_hop(dst_addr)?;

        // Send via MAC
        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &nwk_buf[..total_len],
                msdu_handle: seq,
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(NldeDataConfirm {
            status: NwkStatus::Success,
            nsdu_handle: seq,
        })
    }

    /// Process incoming MAC data indication as a NWK frame.
    ///
    /// Parses the NWK header and either:
    /// - Delivers to upper layer (if destined for us)
    /// - Relays the frame (if we're a router/coordinator)
    pub async fn process_incoming_nwk_frame<'a>(
        &mut self,
        mac_payload: &'a [u8],
        lqi: u8,
    ) -> Option<NldeDataIndication<'a>> {
        // Parse NWK header
        let (header, consumed) = NwkHeader::parse(mac_payload)?;

        // TODO: decrypt if security bit is set

        let dst = header.dst_addr;
        let src = header.src_addr;
        let is_for_us = dst == self.nib.network_address
            || dst == ShortAddress::BROADCAST
            || dst == ShortAddress(0xFFFF);

        if is_for_us {
            // Deliver to upper layer
            let payload = &mac_payload[consumed..];
            return Some(NldeDataIndication {
                dst_addr: dst,
                src_addr: src,
                payload,
                lqi,
                security_use: header.frame_control.security,
            });
        }

        // Not for us — relay if router/coordinator
        if self.device_type != DeviceType::EndDevice && header.radius > 1 {
            let _ = self.relay_frame(mac_payload, &header).await;
        }

        None
    }

    /// Relay a NWK frame (router/coordinator duty).
    async fn relay_frame(&mut self, original: &[u8], header: &NwkHeader) -> Result<(), NwkStatus> {
        // Decrement radius
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(()); // TTL expired
        }

        // Determine next hop for the final destination
        let next_hop = self.resolve_next_hop(header.dst_addr)?;

        // Rebuild frame with decremented radius
        let mut relay_buf = [0u8; 128];
        let mut new_header = header.clone();
        new_header.radius = new_radius;
        let hdr_len = new_header.serialize(&mut relay_buf);

        // Copy original payload (everything after header)
        let (_, orig_hdr_len) = NwkHeader::parse(original).unwrap();
        let payload = &original[orig_hdr_len..];
        if hdr_len + payload.len() > relay_buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        relay_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        let total = hdr_len + payload.len();

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(())
    }

    /// Resolve the MAC next hop for a given NWK destination.
    ///
    /// Strategy:
    /// 1. If destination is a neighbor → send directly
    /// 2. If destination is in routing table → use next_hop
    /// 3. If we're an end device → send to parent
    /// 4. For broadcast → send to all neighbors (simplified: send to parent)
    fn resolve_next_hop(&self, destination: ShortAddress) -> Result<ShortAddress, NwkStatus> {
        // Broadcast: send to parent (end device) or all neighbors (router)
        if destination.0 >= 0xFFF8 {
            if self.device_type == DeviceType::EndDevice {
                return Ok(self.nib.parent_address);
            }
            // Routers broadcast via MAC broadcast
            return Ok(ShortAddress::BROADCAST);
        }

        // Direct neighbor?
        if self.neighbors.find_by_short(destination).is_some() {
            return Ok(destination);
        }

        // Routing table lookup
        if let Some(next) = self.routing.next_hop(destination) {
            return Ok(next);
        }

        // End device fallback: always route through parent
        if self.device_type == DeviceType::EndDevice {
            return Ok(self.nib.parent_address);
        }

        // Tree routing fallback
        if let Some(next) = self.routing.tree_route(
            self.nib.network_address,
            destination,
            self.nib.depth,
            self.nib.max_routers,
            self.nib.max_depth,
        ) {
            return Ok(next);
        }

        // Route to parent as last resort
        if self.nib.parent_address.0 != 0xFFFF {
            Ok(self.nib.parent_address)
        } else {
            Err(NwkStatus::RouteError)
        }
    }
}
