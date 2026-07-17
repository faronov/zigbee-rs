//! Shared IEEE 802.15.4 MAC frame builders and parsers.
//!
//! These functions are platform-independent and used by all MAC backends.
//! Extracted to avoid duplication across nRF, ESP32, PHY6222, BL702, CC2340, and Telink.

use crate::primitives::*;
use zigbee_types::*;

// ── Frame builders ──────────────────────────────────────────────

/// Build a Beacon Request MAC command (broadcast, no ACK).
pub fn build_beacon_request(seq: u8) -> [u8; 8] {
    // FC: command(0x03), dst=short, no src, no ACK, no PAN compress
    // 0x0803: type=011, dst_mode=10, src_mode=00
    let fc: u16 = 0x0803;
    [
        fc as u8,
        (fc >> 8) as u8,
        seq,
        0xFF,
        0xFF, // dst PAN = broadcast
        0xFF,
        0xFF, // dst addr = broadcast
        0x07, // Beacon Request command ID
    ]
}

/// Build an Association Request MAC command.
///
/// An unassociated device uses source PAN ID `0xFFFF`, so PAN ID compression
/// must be clear. This matches IEEE 802.15.4 association framing and Telink's
/// reference stack (`0xC823` for a short-addressed coordinator).
pub fn build_association_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
    cap: &CapabilityInfo,
) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();
    let frame_control: u16 = match coord {
        MacAddress::Short(_, _) => 0xC823,
        MacAddress::Extended(_, _) => 0xCC23,
    };
    let _ = frame.extend_from_slice(&frame_control.to_le_bytes());
    let _ = frame.push(seq);
    let dst_pan = coord.pan_id();
    let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(&PanId::BROADCAST.0.to_le_bytes());
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(0x01); // Association Request command ID
    let _ = frame.push(cap.to_byte());
    frame
}

/// Build a Data Request MAC command with IEEE (extended) source address.
///
/// Used for indirect frame retrieval (polling parent).
/// FC = 0xC863: command, ACK request, PAN compress, dst=short, src=extended.
pub fn build_data_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
    let dst_pan = coord.pan_id();
    let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(0x04); // Data Request command ID
    frame
}

/// Build a Data Request MAC command with SHORT source address.
///
/// Used after association when we have a short address assigned.
/// FC = 0x8863: command, ACK request, PAN compress, dst=short, src=short.
pub fn build_data_request_short(
    seq: u8,
    coord: &MacAddress,
    own_short: ShortAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    let _ = frame.extend_from_slice(&[0x63, 0x88, seq]);
    let dst_pan = coord.pan_id();
    let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(&own_short.0.to_le_bytes());
    let _ = frame.push(0x04); // Data Request command ID
    frame
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameBuildError {
    FrameTooLong,
}

/// Build a data frame without an FCS. The radio backend appends the FCS.
pub fn build_data_frame(
    seq: u8,
    src_addr_mode: AddressMode,
    own_short: ShortAddress,
    own_extended: &IeeeAddress,
    dst_address: &MacAddress,
    payload: &[u8],
    ack_request: bool,
) -> Result<heapless::Vec<u8, 125>, FrameBuildError> {
    let dst_len = match dst_address {
        MacAddress::Short(_, _) => 2,
        MacAddress::Extended(_, _) => 8,
    };
    let src_len = match src_addr_mode {
        AddressMode::None => 0,
        AddressMode::Short => 2,
        AddressMode::Extended => 8,
    };
    let frame_len = 3 + 2 + dst_len + src_len + payload.len();
    if frame_len > 125 {
        return Err(FrameBuildError::FrameTooLong);
    }

    let mut fc = 0x0001u16;
    if ack_request {
        fc |= 1 << 5;
    }
    if src_addr_mode != AddressMode::None {
        fc |= 1 << 6;
    }
    fc |= match dst_address {
        MacAddress::Short(_, _) => 0b10 << 10,
        MacAddress::Extended(_, _) => 0b11 << 10,
    };
    fc |= (src_addr_mode as u16) << 14;

    let mut frame = heapless::Vec::new();
    frame
        .extend_from_slice(&fc.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame.push(seq).map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(&dst_address.pan_id().0.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    match dst_address {
        MacAddress::Short(_, address) => frame
            .extend_from_slice(&address.0.to_le_bytes())
            .map_err(|_| FrameBuildError::FrameTooLong)?,
        MacAddress::Extended(_, address) => frame
            .extend_from_slice(address)
            .map_err(|_| FrameBuildError::FrameTooLong)?,
    }
    match src_addr_mode {
        AddressMode::None => {}
        AddressMode::Short => frame
            .extend_from_slice(&own_short.0.to_le_bytes())
            .map_err(|_| FrameBuildError::FrameTooLong)?,
        AddressMode::Extended => frame
            .extend_from_slice(own_extended)
            .map_err(|_| FrameBuildError::FrameTooLong)?,
    }
    frame
        .extend_from_slice(payload)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    Ok(frame)
}

// ── Frame parsers ───────────────────────────────────────────────

/// Calculate total addressing field size from frame control.
pub fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut size = 0;
    match dst_mode {
        0x02 => size += 2 + 2, // PAN(2) + Short(2)
        0x03 => size += 2 + 8, // PAN(2) + Extended(8)
        _ => {}
    }
    match src_mode {
        0x02 => size += if pan_compress { 2 } else { 4 },
        0x03 => size += if pan_compress { 8 } else { 10 },
        _ => {}
    }
    size
}

/// Parse source address from raw MAC frame.
pub fn parse_source_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut offset = 3;
    let dst_pan = if dst_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        Some(pan)
    } else {
        None
    };
    match dst_mode {
        0x02 => offset += 2,
        0x03 => offset += 8,
        _ => {}
    }

    let src_pan = if !pan_compress && src_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        pan
    } else {
        dst_pan.unwrap_or(0xFFFF)
    };

    match src_mode {
        0x02 if data.len() >= offset + 2 => {
            let addr = u16::from_le_bytes([data[offset], data[offset + 1]]);
            Some(MacAddress::Short(PanId(src_pan), ShortAddress(addr)))
        }
        0x03 if data.len() >= offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            Some(MacAddress::Extended(PanId(src_pan), ext))
        }
        _ => None,
    }
}

/// Parse destination address from raw MAC frame.
pub fn parse_dest_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let offset = 3;

    if data.len() < offset + 2 {
        return None;
    }
    let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
    let addr_offset = offset + 2;

    match dst_mode {
        0x02 if data.len() >= addr_offset + 2 => {
            let addr = u16::from_le_bytes([data[addr_offset], data[addr_offset + 1]]);
            Some(MacAddress::Short(PanId(pan), ShortAddress(addr)))
        }
        0x03 if data.len() >= addr_offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[addr_offset..addr_offset + 8]);
            Some(MacAddress::Extended(PanId(pan), ext))
        }
        _ => None,
    }
}

/// Parse Zigbee beacon payload (at least 15 bytes expected).
pub fn parse_zigbee_beacon(data: &[u8]) -> ZigbeeBeaconPayload {
    let protocol_id = data[0];
    let nwk_info = u16::from_le_bytes([data[1], data[2]]);

    let mut extended_pan_id = [0u8; 8];
    extended_pan_id.copy_from_slice(&data[3..11]);
    let mut tx_offset = [0u8; 3];
    tx_offset.copy_from_slice(&data[11..14]);

    ZigbeeBeaconPayload {
        protocol_id,
        stack_profile: (nwk_info & 0x0F) as u8,
        protocol_version: ((nwk_info >> 4) & 0x0F) as u8,
        router_capacity: (nwk_info >> 10) & 1 != 0,
        device_depth: ((nwk_info >> 11) & 0x0F) as u8,
        end_device_capacity: (nwk_info >> 15) & 1 != 0,
        extended_pan_id,
        tx_offset,
        update_id: data[14],
    }
}

/// Parse a beacon frame into a PanDescriptor.
///
/// Handles both MAC-only beacons and full Zigbee beacon payloads.
pub fn parse_beacon(channel: u8, data: &[u8], lqi: u8) -> Option<PanDescriptor> {
    if data.len() < 11 {
        return None;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    if fc & 0x07 != 0x00 {
        return None; // Not a beacon frame
    }
    let coord_address = parse_source_address(data, fc)?;
    let superframe_offset = 3 + addressing_size(fc);
    if data.len() < superframe_offset + 4 {
        return None;
    }
    let superframe_raw = u16::from_le_bytes([data[superframe_offset], data[superframe_offset + 1]]);

    let mut payload_offset = superframe_offset + 2;
    let gts_spec = *data.get(payload_offset)?;
    payload_offset += 1;
    let gts_count = (gts_spec & 0x07) as usize;
    if gts_count != 0 {
        payload_offset = payload_offset.checked_add(1 + gts_count * 3)?;
    }

    let pending_spec = *data.get(payload_offset)?;
    payload_offset += 1;
    let short_pending = (pending_spec & 0x07) as usize;
    let extended_pending = ((pending_spec >> 4) & 0x07) as usize;
    payload_offset = payload_offset.checked_add(short_pending * 2 + extended_pending * 8)?;
    let zigbee_data = data.get(payload_offset..)?;
    if zigbee_data.len() < 15 {
        return None;
    }
    let zigbee_beacon = parse_zigbee_beacon(zigbee_data);

    Some(PanDescriptor {
        coord_address,
        channel,
        superframe_spec: SuperframeSpec::from_raw(superframe_raw),
        lqi,
        security_use: false,
        zigbee_beacon,
    })
}

/// Parse full MAC addresses from a raw frame.
///
/// Returns (src_address, dst_address, payload_offset, security_bit).
pub fn parse_mac_addresses(data: &[u8]) -> (MacAddress, MacAddress, usize, bool) {
    let default_addr = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));
    if data.len() < 3 {
        return (default_addr, default_addr, 0, false);
    }

    let fc = u16::from_le_bytes([data[0], data[1]]);
    let security = (fc >> 3) & 1 != 0;
    let pan_compress = (fc >> 6) & 1 != 0;
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;

    let mut offset = 3;

    let dst_pan = if dst_mode > 0 && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        PanId(0xFFFF)
    };

    let dst_address = match dst_mode {
        2 if offset + 2 <= data.len() => {
            let a = u16::from_le_bytes([data[offset], data[offset + 1]]);
            offset += 2;
            MacAddress::Short(dst_pan, ShortAddress(a))
        }
        3 if offset + 8 <= data.len() => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            MacAddress::Extended(dst_pan, ext)
        }
        _ => default_addr,
    };

    let src_pan = if src_mode > 0 && !pan_compress && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        dst_pan
    };

    let src_address = match src_mode {
        2 if offset + 2 <= data.len() => {
            let a = u16::from_le_bytes([data[offset], data[offset + 1]]);
            offset += 2;
            MacAddress::Short(src_pan, ShortAddress(a))
        }
        3 if offset + 8 <= data.len() => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            MacAddress::Extended(src_pan, ext)
        }
        _ => MacAddress::Short(src_pan, ShortAddress(0xFFFF)),
    };

    (src_address, dst_address, offset, security)
}

/// Parse an Association Response from a MAC command frame.
///
/// Returns (assigned_short_address, status_byte) if valid.
pub fn parse_association_response(data: &[u8]) -> Option<(ShortAddress, u8)> {
    if data.len() < 5 {
        return None;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    if fc & 0x07 != 0x03 {
        return None; // Not a command frame
    }

    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 0x01;

    let mut offset = 3;
    if dst_mode > 0 {
        offset += 2;
    } // PAN
    match dst_mode {
        2 => offset += 2,
        3 => offset += 8,
        _ => {}
    }
    if src_mode > 0 && pan_compress == 0 {
        offset += 2;
    } // Src PAN
    match src_mode {
        2 => offset += 2,
        3 => offset += 8,
        _ => {}
    }

    if offset + 4 > data.len() {
        return None;
    }
    if data[offset] != 0x02 {
        return None; // Not Association Response command
    }

    let short = u16::from_le_bytes([data[offset + 1], data[offset + 2]]);
    let status = data[offset + 3];
    Some((ShortAddress(short), status))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn association_request_uses_unassociated_source_pan() {
        let frame = build_association_request(
            0x42,
            &MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0000)),
            &[0x29, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02],
            &CapabilityInfo {
                rx_on_when_idle: false,
                allocate_address: true,
                ..CapabilityInfo::default()
            },
        );

        assert_eq!(
            frame.as_slice(),
            [
                0x23, 0xC8, 0x42, 0xE9, 0xDF, 0x00, 0x00, 0xFF, 0xFF, 0x29, 0x34, 0x36, 0x39, 0x33,
                0x4E, 0x55, 0x02, 0x01, 0x80,
            ]
        );
    }

    #[test]
    fn builds_short_to_short_data_frame() {
        let frame = build_data_frame(
            0x42,
            AddressMode::Short,
            ShortAddress(0x3344),
            &[0; 8],
            &MacAddress::Short(PanId(0xABCD), ShortAddress(0x1122)),
            &[0xAA, 0xBB],
            true,
        )
        .unwrap();

        assert_eq!(
            frame.as_slice(),
            [
                0x61, 0x88, 0x42, 0xCD, 0xAB, 0x22, 0x11, 0x44, 0x33, 0xAA, 0xBB
            ]
        );
    }

    #[test]
    fn rejects_data_frame_larger_than_psdu_without_fcs() {
        let payload = [0u8; 117];
        assert_eq!(
            build_data_frame(
                0,
                AddressMode::Short,
                ShortAddress(1),
                &[0; 8],
                &MacAddress::Short(PanId(1), ShortAddress(2)),
                &payload,
                false,
            ),
            Err(FrameBuildError::FrameTooLong)
        );
    }

    #[test]
    fn rejects_truncated_association_response() {
        let truncated = [
            0x63, 0x88, 0x01, 0x34, 0x12, 0x78, 0x56, 0x9A, 0xBC, 0x02, 0x44, 0x33,
        ];
        assert_eq!(parse_association_response(&truncated), None);
    }

    #[test]
    fn parses_full_zigbee_beacon_at_correct_payload_offset() {
        let frame = [
            0x00, 0x80, 0x55, // beacon + sequence
            0xE9, 0xDF, 0x2D, 0x7D, // source PAN and coordinator
            0xFF, 0xCF, // superframe
            0x00, // no GTS
            0x00, // no pending addresses
            0x00, // Zigbee protocol ID
            0x22, 0x84, // stack/profile/version/capacities
            1, 2, 3, 4, 5, 6, 7, 8, // extended PAN ID
            0, 0, 0,    // TX offset
            0x09, // update ID
        ];
        let descriptor = parse_beacon(15, &frame, 200).unwrap();
        assert_eq!(
            descriptor.coord_address,
            MacAddress::Short(PanId(0xDFE9), ShortAddress(0x7D2D))
        );
        assert_eq!(descriptor.zigbee_beacon.protocol_id, 0);
        assert_eq!(descriptor.zigbee_beacon.stack_profile, 2);
        assert_eq!(descriptor.zigbee_beacon.protocol_version, 2);
        assert_eq!(
            descriptor.zigbee_beacon.extended_pan_id,
            [1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(descriptor.zigbee_beacon.update_id, 9);
    }
}
