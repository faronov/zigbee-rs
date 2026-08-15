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
/// Uses PAN compression because the associated child and parent share a PAN.
pub fn build_data_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    let frame_control: u16 = match coord {
        MacAddress::Short(_, _) => 0xC863,
        MacAddress::Extended(_, _) => 0xCC63,
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
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(0x04); // Data Request command ID
    frame
}

/// Build a Data Request MAC command with SHORT source address.
///
/// Used after association when we have a short address assigned.
/// Uses PAN compression because the associated child and parent share a PAN.
pub fn build_data_request_short(
    seq: u8,
    coord: &MacAddress,
    own_short: ShortAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    let frame_control: u16 = match coord {
        MacAddress::Short(_, _) => 0x8863,
        MacAddress::Extended(_, _) => 0x8C63,
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
    let _ = frame.extend_from_slice(&own_short.0.to_le_bytes());
    let _ = frame.push(0x04); // Data Request command ID
    frame
}

/// Build an on-demand IEEE 802.15.4 beacon for a Zigbee non-beacon network.
///
/// The superframe uses BO=15, SO=15 and final CAP slot=15. Zigbee does not
/// allocate GTS slots, so the GTS Specification is zero. Pending short and
/// extended addresses are encoded in the bounded Pending Address fields.
pub fn build_nonbeacon_beacon(
    sequence: u8,
    source: &MacAddress,
    pan_coordinator: bool,
    association_permit: bool,
    pending_short_addresses: &[ShortAddress],
    pending_extended_addresses: &[IeeeAddress],
    beacon_payload: &[u8],
) -> Result<heapless::Vec<u8, 125>, FrameBuildError> {
    if pending_short_addresses.len() > MAX_BEACON_PENDING_ADDRESSES
        || pending_extended_addresses.len() > MAX_BEACON_PENDING_ADDRESSES
        || source.pan_id() == PanId::BROADCAST
    {
        return Err(FrameBuildError::InvalidParameter);
    }

    let mut frame_control = 0u16; // Beacon, no ACK/security/PAN compression.
    frame_control |= match source {
        MacAddress::Short(_, address) if address.0 < 0xFFF8 => (AddressMode::Short as u16) << 14,
        MacAddress::Extended(_, address) if *address != [0xFF; 8] => {
            (AddressMode::Extended as u16) << 14
        }
        _ => return Err(FrameBuildError::InvalidParameter),
    };

    let mut frame = heapless::Vec::new();
    frame
        .extend_from_slice(&frame_control.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .push(sequence)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(&source.pan_id().0.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    match source {
        MacAddress::Short(_, address) => frame
            .extend_from_slice(&address.0.to_le_bytes())
            .map_err(|_| FrameBuildError::FrameTooLong)?,
        MacAddress::Extended(_, address) => frame
            .extend_from_slice(address)
            .map_err(|_| FrameBuildError::FrameTooLong)?,
    }

    let mut superframe_spec = 0x0FFFu16;
    if pan_coordinator {
        superframe_spec |= 1 << 14;
    }
    if association_permit {
        superframe_spec |= 1 << 15;
    }
    frame
        .extend_from_slice(&superframe_spec.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame.push(0).map_err(|_| FrameBuildError::FrameTooLong)?; // GTS Specification
    frame
        .push(pending_short_addresses.len() as u8 | ((pending_extended_addresses.len() as u8) << 4))
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    for address in pending_short_addresses {
        if address.0 >= 0xFFF8 {
            return Err(FrameBuildError::InvalidParameter);
        }
        frame
            .extend_from_slice(&address.0.to_le_bytes())
            .map_err(|_| FrameBuildError::FrameTooLong)?;
    }
    for address in pending_extended_addresses {
        if *address == [0xFF; 8] {
            return Err(FrameBuildError::InvalidParameter);
        }
        frame
            .extend_from_slice(address)
            .map_err(|_| FrameBuildError::FrameTooLong)?;
    }
    frame
        .extend_from_slice(beacon_payload)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    Ok(frame)
}

/// Build an Association Response MAC command.
///
/// IEEE 802.15.4 association responses use extended source and destination
/// addresses with PAN ID compression. The child therefore remains
/// identifiable before it begins using its newly allocated short address.
pub fn build_association_response(
    sequence: u8,
    pan_id: PanId,
    coordinator_extended: &IeeeAddress,
    response: &MlmeAssociateResponse,
) -> Result<heapless::Vec<u8, 32>, FrameBuildError> {
    if pan_id == PanId::BROADCAST
        || *coordinator_extended == [0xFF; 8]
        || response.device_address == [0xFF; 8]
        || (response.status == AssociationStatus::Success && response.short_address.0 >= 0xFFF8)
    {
        return Err(FrameBuildError::InvalidParameter);
    }

    // Command, ACK request, PAN compression, extended dst + extended src.
    let frame_control = 0xCC63u16;
    let mut frame = heapless::Vec::new();
    frame
        .extend_from_slice(&frame_control.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .push(sequence)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(&pan_id.0.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(&response.device_address)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(coordinator_extended)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .push(0x02)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .extend_from_slice(&response.short_address.0.to_le_bytes())
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    frame
        .push(response.status as u8)
        .map_err(|_| FrameBuildError::FrameTooLong)?;
    Ok(frame)
}

/// Build a Disassociation Notification MAC command.
pub fn build_disassociation_notification(
    seq: u8,
    destination: &MacAddress,
    own_short: ShortAddress,
    own_extended: &IeeeAddress,
    reason: DisassociateReason,
) -> heapless::Vec<u8, 32> {
    let source_mode = if matches!(own_short.0, 0xFFFE | 0xFFFF) {
        AddressMode::Extended
    } else {
        AddressMode::Short
    };
    let mut frame_control = 0x0003u16 | (1 << 5) | (1 << 6);
    frame_control |= match destination {
        MacAddress::Short(_, _) => 0b10 << 10,
        MacAddress::Extended(_, _) => 0b11 << 10,
    };
    frame_control |= (source_mode as u16) << 14;

    let mut frame = heapless::Vec::new();
    let _ = frame.extend_from_slice(&frame_control.to_le_bytes());
    let _ = frame.push(seq);
    let _ = frame.extend_from_slice(&destination.pan_id().0.to_le_bytes());
    match destination {
        MacAddress::Short(_, address) => {
            let _ = frame.extend_from_slice(&address.0.to_le_bytes());
        }
        MacAddress::Extended(_, address) => {
            let _ = frame.extend_from_slice(address);
        }
    }
    match source_mode {
        AddressMode::Short => {
            let _ = frame.extend_from_slice(&own_short.0.to_le_bytes());
        }
        AddressMode::Extended => {
            let _ = frame.extend_from_slice(own_extended);
        }
        AddressMode::None => {}
    }
    let _ = frame.push(0x03);
    let _ = frame.push(reason as u8);
    frame
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameBuildError {
    FrameTooLong,
    InvalidParameter,
}

// ── Orphan notification / coordinator realignment ───────────────

/// IEEE 802.15.4 Orphan Notification MAC command identifier.
pub const MAC_CMD_ORPHAN_NOTIFICATION: u8 = 0x06;
/// IEEE 802.15.4 Coordinator Realignment MAC command identifier.
pub const MAC_CMD_COORDINATOR_REALIGNMENT: u8 = 0x08;

/// Build an Orphan Notification MAC command (IEEE 802.15.4 §7.3.6).
///
/// Broadcast on the orphan scan channel with both PAN IDs set to `0xFFFF`
/// (so PAN ID compression is set), a short broadcast destination and the
/// orphan's extended address as source. No ACK is requested — the parent
/// answers with a Coordinator Realignment.
pub fn build_orphan_notification(seq: u8, own_extended: &IeeeAddress) -> heapless::Vec<u8, 24> {
    // Command (0b011), PAN ID compression (bit 6), dst short (0b10 << 10),
    // src extended (0b11 << 14). No ACK request: the realignment is the reply.
    let frame_control: u16 = 0x0003 | (1 << 6) | (0b10 << 10) | (0b11 << 14);
    let mut frame = heapless::Vec::new();
    let _ = frame.extend_from_slice(&frame_control.to_le_bytes());
    let _ = frame.push(seq);
    let _ = frame.extend_from_slice(&PanId::BROADCAST.0.to_le_bytes());
    let _ = frame.extend_from_slice(&ShortAddress::BROADCAST.0.to_le_bytes());
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(MAC_CMD_ORPHAN_NOTIFICATION);
    frame
}

/// Parse an Orphan Notification, returning the orphan's extended address.
///
/// Accepts only the normative framing: a MAC command frame with PAN ID
/// compression, a broadcast short destination in the broadcast PAN and an
/// extended source. Anything else (including a short-source "orphan", which
/// would let an unidentifiable device solicit a realignment) is rejected.
pub fn parse_orphan_notification(data: &[u8]) -> Option<IeeeAddress> {
    if data.len() != 16 {
        return None;
    }
    let frame_control = u16::from_le_bytes([*data.first()?, *data.get(1)?]);
    if frame_control != 0xC843 {
        return None;
    }
    let dst_pan = u16::from_le_bytes([*data.get(3)?, *data.get(4)?]);
    let dst_addr = u16::from_le_bytes([*data.get(5)?, *data.get(6)?]);
    if dst_pan != PanId::BROADCAST.0 || dst_addr != ShortAddress::BROADCAST.0 {
        return None;
    }
    let mut orphan = [0u8; 8];
    orphan.copy_from_slice(data.get(7..15)?);
    if *data.get(15)? != MAC_CMD_ORPHAN_NOTIFICATION {
        return None;
    }
    if orphan == [0u8; 8] || orphan == [0xFF; 8] {
        return None;
    }
    Some(orphan)
}

/// Build a Coordinator Realignment MAC command sent in response to an Orphan
/// Notification (IEEE 802.15.4 §7.3.8).
///
/// R22 Annex D requires the Frame Version field to be zero in a ZigBee
/// coordinator realignment, which this framing satisfies.
///
/// Orphan-response framing: destination PAN `0xFFFF` with the orphan's
/// extended address, source PAN `macPANId` with this device's extended
/// address, ACK requested, no PAN ID compression. The `Short Address` field
/// carries the address the parent already holds for the orphan, which is what
/// re-attaches a restored child without a fresh association.
pub fn build_coordinator_realignment_orphan_response(
    seq: u8,
    pan_id: PanId,
    coordinator_short: ShortAddress,
    coordinator_extended: &IeeeAddress,
    channel: u8,
    orphan_extended: &IeeeAddress,
    orphan_short: ShortAddress,
) -> Result<heapless::Vec<u8, 32>, FrameBuildError> {
    if pan_id == PanId::BROADCAST
        || *coordinator_extended == [0xFF; 8]
        || *orphan_extended == [0xFF; 8]
        || *orphan_extended == [0x00; 8]
        || !(0x0001..=0xFFF7).contains(&orphan_short.0)
        || coordinator_short.0 >= 0xFFF8
        || orphan_short == coordinator_short
        || !(11..=26).contains(&channel)
    {
        return Err(FrameBuildError::InvalidParameter);
    }

    // Command (0b011), ACK request (bit 5), extended dst (0b11 << 10),
    // extended src (0b11 << 14). Frame version stays 0.
    let frame_control: u16 = 0x0003 | (1 << 5) | (0b11 << 10) | (0b11 << 14);
    let mut frame = heapless::Vec::new();
    let mut put = |bytes: &[u8]| -> Result<(), FrameBuildError> {
        frame
            .extend_from_slice(bytes)
            .map_err(|_| FrameBuildError::FrameTooLong)
    };
    put(&frame_control.to_le_bytes())?;
    put(&[seq])?;
    put(&PanId::BROADCAST.0.to_le_bytes())?;
    put(orphan_extended)?;
    put(&pan_id.0.to_le_bytes())?;
    put(coordinator_extended)?;
    put(&[MAC_CMD_COORDINATOR_REALIGNMENT])?;
    put(&pan_id.0.to_le_bytes())?;
    put(&coordinator_short.0.to_le_bytes())?;
    put(&[channel])?;
    put(&orphan_short.0.to_le_bytes())?;
    Ok(frame)
}

/// A decoded Coordinator Realignment payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoordinatorRealignment {
    pub pan_id: PanId,
    pub coordinator_short: ShortAddress,
    pub channel: u8,
    /// Address assigned to the realigned device (`0xFFFF` in a broadcast
    /// realignment, which does not re-address anyone).
    pub short_address: ShortAddress,
}

/// Parse a Coordinator Realignment received as an orphan response.
///
/// Returns the realignment parameters together with the extended addresses of
/// the responding coordinator/router and of the realigned device, so a caller
/// can verify the frame was addressed to it before adopting the parameters.
pub fn parse_coordinator_realignment_orphan_response(
    data: &[u8],
) -> Option<(CoordinatorRealignment, IeeeAddress, IeeeAddress)> {
    if data.len() != 31 {
        return None;
    }
    let frame_control = u16::from_le_bytes([*data.first()?, *data.get(1)?]);
    if frame_control != 0xCC23 {
        return None;
    }
    let destination_pan = u16::from_le_bytes([*data.get(3)?, *data.get(4)?]);
    if destination_pan != PanId::BROADCAST.0 {
        return None;
    }
    let mut destination = [0u8; 8];
    destination.copy_from_slice(data.get(5..13)?);
    let source_pan = u16::from_le_bytes([*data.get(13)?, *data.get(14)?]);
    let mut source = [0u8; 8];
    source.copy_from_slice(data.get(15..23)?);
    if *data.get(23)? != MAC_CMD_COORDINATOR_REALIGNMENT {
        return None;
    }
    let pan_id = PanId(u16::from_le_bytes([*data.get(24)?, *data.get(25)?]));
    let coordinator_short = ShortAddress(u16::from_le_bytes([*data.get(26)?, *data.get(27)?]));
    let channel = *data.get(28)?;
    let short_address = ShortAddress(u16::from_le_bytes([*data.get(29)?, *data.get(30)?]));
    if pan_id.0 != source_pan
        || pan_id == PanId::BROADCAST
        || source == [0u8; 8]
        || source == [0xFF; 8]
        || destination == [0u8; 8]
        || destination == [0xFF; 8]
        || coordinator_short.0 >= 0xFFF8
        || !(0x0001..=0xFFF7).contains(&short_address.0)
        || short_address == coordinator_short
        || !(11..=26).contains(&channel)
    {
        return None;
    }
    Some((
        CoordinatorRealignment {
            pan_id,
            coordinator_short,
            channel,
            short_address,
        },
        source,
        destination,
    ))
}

/// Build a data frame without an FCS. The radio backend appends the FCS.
#[allow(clippy::too_many_arguments)]
pub fn build_data_frame(
    seq: u8,
    src_addr_mode: AddressMode,
    own_short: ShortAddress,
    own_extended: &IeeeAddress,
    dst_address: &MacAddress,
    payload: &[u8],
    ack_request: bool,
    frame_pending: bool,
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
    if frame_pending {
        fc |= 1 << 4;
    }
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

/// Return the sequence number if `data` is an IEEE 802.15.4 ACK frame.
pub fn ack_sequence(data: &[u8]) -> Option<u8> {
    if data.len() < 3 {
        return None;
    }
    let frame_type = data[0] & 0x07;
    (frame_type == 0x02).then_some(data[2])
}

/// Return `(sequence, frame_pending)` if `data` is an IEEE 802.15.4 ACK frame.
///
/// An ACK carries no addresses, so the sequence number is the only way to tell
/// our acknowledgement apart from one that another pair of nodes exchanged on
/// the same channel. The frame-pending bit is what a sleepy end device reads
/// after a Data Request to learn whether the parent has traffic buffered for
/// it.
pub fn ack_info(data: &[u8]) -> Option<(u8, bool)> {
    let sequence = ack_sequence(data)?;
    Some((sequence, data[0] & 0x10 != 0))
}

/// Decide whether a received frame is addressed to this node.
///
/// This mirrors IEEE 802.15.4-2015 §6.7.2 third-level filtering and must agree
/// with whatever the radio's hardware address filter is programmed with —
/// otherwise the hardware acknowledges frames software then throws away (or,
/// worse, software accepts frames the hardware never acknowledged).
///
/// `our_short` is `0xFFFF` while unassociated. The only IEEE 802.15.4 MAC
/// short broadcast destination is `0xFFFF`; Zigbee values `0xFFFC` and
/// `0xFFFD` belong in the decoded NWK header, not the MAC destination field.
/// An unassociated node (`our_pan == 0xFFFF`) accepts any PAN so that the
/// extended-address Association Response and Transport-Key can be received.
pub fn frame_is_for_us(
    dst: &MacAddress,
    our_pan: PanId,
    our_short: ShortAddress,
    our_extended: &IeeeAddress,
) -> bool {
    let pan_ok = |pan: PanId| pan.0 == our_pan.0 || pan.0 == 0xFFFF || our_pan.0 == 0xFFFF;
    match dst {
        MacAddress::Short(pan, addr) => {
            pan_ok(*pan) && (addr.0 == 0xFFFF || (our_short.0 != 0xFFFF && addr.0 == our_short.0))
        }
        MacAddress::Extended(pan, addr) => pan_ok(*pan) && addr == our_extended,
    }
}

/// Convert an EUI-64 into the big-endian word expected by radio drivers that
/// take the extended address as a `u64`.
///
/// [`IeeeAddress`] holds the bytes in the order the MAC writes them into the
/// addressing fields (IEEE 802.15.4 transmits extended addresses
/// least-significant octet first). `esp-radio`'s `Config::ext_addr` is fed to
/// the hardware through `u64::to_be_bytes()`, so the round-trip only preserves
/// the on-air ordering if the word is built with `from_be_bytes`.
pub fn ieee_address_as_be_word(address: &IeeeAddress) -> u64 {
    u64::from_be_bytes(*address)
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
    fn nonbeacon_beacon_encodes_superframe_gts_and_pending_addresses() {
        let frame = build_nonbeacon_beacon(
            0x21,
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x0001)),
            false,
            true,
            &[ShortAddress(0x3344)],
            &[[1, 2, 3, 4, 5, 6, 7, 8]],
            &[0xAA, 0xBB],
        )
        .unwrap();

        assert_eq!(
            frame.as_slice(),
            [
                0x00, 0x80, 0x21, 0x34, 0x12, 0x01, 0x00, // MHR
                0xFF, 0x8F, // BO=SO=CAP=15, association permitted
                0x00, // no GTS
                0x11, // one short + one extended pending address
                0x44, 0x33, 1, 2, 3, 4, 5, 6, 7, 8, 0xAA, 0xBB,
            ]
        );
    }

    #[test]
    fn nonbeacon_beacon_rejects_oversize_pending_payload() {
        let pending = [ShortAddress(1); MAX_BEACON_PENDING_ADDRESSES];
        let payload = [0u8; 110];
        assert_eq!(
            build_nonbeacon_beacon(
                0,
                &MacAddress::Short(PanId(1), ShortAddress(1)),
                false,
                false,
                &pending,
                &[],
                &payload,
            ),
            Err(FrameBuildError::FrameTooLong)
        );
    }

    #[test]
    fn association_response_uses_extended_child_and_parent_addresses() {
        let response = MlmeAssociateResponse {
            device_address: [1, 2, 3, 4, 5, 6, 7, 8],
            short_address: ShortAddress(0x3344),
            status: AssociationStatus::Success,
        };
        let frame =
            build_association_response(0x22, PanId(0x1234), &[8, 7, 6, 5, 4, 3, 2, 1], &response)
                .unwrap();

        assert_eq!(&frame[..5], &[0x63, 0xCC, 0x22, 0x34, 0x12]);
        assert_eq!(&frame[5..13], &response.device_address);
        assert_eq!(&frame[13..21], &[8, 7, 6, 5, 4, 3, 2, 1]);
        assert_eq!(&frame[21..], &[0x02, 0x44, 0x33, 0x00]);
        assert_eq!(
            parse_association_response(&frame),
            Some((ShortAddress(0x3344), 0))
        );
    }

    #[test]
    fn data_request_uses_coordinator_address_mode() {
        let own_ieee = [1, 2, 3, 4, 5, 6, 7, 8];
        let short = build_data_request(
            0x22,
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &own_ieee,
        );
        assert_eq!(&short[..3], &[0x63, 0xC8, 0x22]);

        let extended = build_data_request(
            0x23,
            &MacAddress::Extended(PanId(0x1234), [9; 8]),
            &own_ieee,
        );
        assert_eq!(&extended[..3], &[0x63, 0xCC, 0x23]);
    }

    #[test]
    fn disassociation_notification_uses_assigned_short_source() {
        let frame = build_disassociation_notification(
            0x24,
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            ShortAddress(0x5678),
            &[1; 8],
            DisassociateReason::DeviceLeave,
        );
        assert_eq!(&frame[..3], &[0x63, 0x88, 0x24]);
        assert_eq!(
            &frame[3..],
            &[0x34, 0x12, 0x00, 0x00, 0x78, 0x56, 0x03, 0x02]
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
            false,
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
    fn data_frame_can_advertise_more_pending_data() {
        let frame = build_data_frame(
            0x43,
            AddressMode::Short,
            ShortAddress(0x3344),
            &[0; 8],
            &MacAddress::Short(PanId(0xABCD), ShortAddress(0x1122)),
            &[0xAA],
            true,
            true,
        )
        .unwrap();

        assert_ne!(u16::from_le_bytes([frame[0], frame[1]]) & (1 << 4), 0);
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

    #[test]
    fn ack_frames_are_detected_by_sequence() {
        // FCF 0x0200 = ACK, sequence 0x42
        assert_eq!(ack_sequence(&[0x02, 0x00, 0x42]), Some(0x42));
        // Data frame must never be mistaken for an ACK
        assert_eq!(ack_sequence(&[0x61, 0x88, 0x42, 0x34, 0x12]), None);
        assert_eq!(ack_sequence(&[0x02, 0x00]), None);
    }

    #[test]
    fn ack_frame_pending_bit_is_read_with_the_sequence_number() {
        // FCF 0x0200: ACK, no frame pending.
        assert_eq!(ack_info(&[0x02, 0x00, 0x42]), Some((0x42, false)));
        // FCF 0x0212: ACK with the frame-pending bit set — the parent has data
        // buffered for us and we must keep listening.
        assert_eq!(ack_info(&[0x12, 0x00, 0x42]), Some((0x42, true)));
        // A data frame with the same bit set is not an ACK.
        assert_eq!(ack_info(&[0x61, 0x88, 0x42, 0x34, 0x12]), None);
    }

    #[test]
    fn address_filter_accepts_our_unicast_and_broadcasts() {
        let our_pan = PanId(0xDFE9);
        let our_short = ShortAddress(0x167E);
        let our_ieee: IeeeAddress = [0x62, 0x55, 0xF9, 0xFF, 0xFE, 0xF6, 0x96, 0x88];

        for accepted in [
            MacAddress::Short(our_pan, our_short),
            MacAddress::Short(our_pan, ShortAddress(0xFFFF)),
            MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
            // Transport-Key and Association Response are IEEE addressed —
            // these are the frames that used to require promiscuous mode.
            MacAddress::Extended(our_pan, our_ieee),
        ] {
            assert!(
                frame_is_for_us(&accepted, our_pan, our_short, &our_ieee),
                "{accepted:?} must be accepted"
            );
        }

        for rejected in [
            MacAddress::Short(our_pan, ShortAddress(0x1F0F)),
            // These are Zigbee NWK broadcasts, not MAC broadcasts.
            MacAddress::Short(our_pan, ShortAddress(0xFFFD)),
            MacAddress::Short(our_pan, ShortAddress(0xFFFC)),
            MacAddress::Short(PanId(0x1234), our_short),
            MacAddress::Extended(our_pan, [0; 8]),
        ] {
            assert!(
                !frame_is_for_us(&rejected, our_pan, our_short, &our_ieee),
                "{rejected:?} must be rejected"
            );
        }
    }

    #[test]
    fn unassociated_node_does_not_claim_short_broadcast_as_unicast() {
        let our_ieee: IeeeAddress = [0x62, 0x55, 0xF9, 0xFF, 0xFE, 0xF6, 0x96, 0x88];
        // While unassociated macShortAddress is 0xFFFF; that must not turn
        // every 0xFFFF-addressed frame into a "unicast for us" match, and the
        // IEEE-addressed association response must still be accepted.
        assert!(frame_is_for_us(
            &MacAddress::Extended(PanId(0xDFE9), our_ieee),
            PanId(0xFFFF),
            ShortAddress(0xFFFF),
            &our_ieee,
        ));
        assert!(!frame_is_for_us(
            &MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0001)),
            PanId(0xFFFF),
            ShortAddress(0xFFFF),
            &our_ieee,
        ));
    }

    #[test]
    fn hardware_filter_word_round_trips_to_on_air_order() {
        // The MAC writes IeeeAddress verbatim into the addressing fields, and
        // esp-radio hands Config::ext_addr to the radio via to_be_bytes(), so
        // the two must be identical byte sequences.
        let ieee: IeeeAddress = [0x62, 0x55, 0xF9, 0xFF, 0xFE, 0xF6, 0x96, 0x88];
        assert_eq!(ieee_address_as_be_word(&ieee).to_be_bytes(), ieee);

        let request = build_association_request(
            0x01,
            &MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0000)),
            &ieee,
            &CapabilityInfo::default(),
        );
        let on_air_source = &request[9..17];
        assert_eq!(
            on_air_source,
            ieee_address_as_be_word(&ieee).to_be_bytes(),
            "hardware filter must be programmed with the bytes we transmit"
        );
    }

    // ── Orphan notification / coordinator realignment ───────────

    const ORPHAN_IEEE: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const PARENT_IEEE: IeeeAddress = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7];

    #[test]
    fn orphan_notification_matches_the_ieee_802_15_4_golden_frame() {
        // IEEE 802.15.4 §7.3.6: FC = command | PAN ID compression | short dst
        // | extended src = 0xC803, broadcast PAN and short destination, then
        // the orphan's extended address and command id 0x06.
        let frame = build_orphan_notification(0x5A, &ORPHAN_IEEE);
        assert_eq!(
            frame.as_slice(),
            &[
                0x43, 0xC8, // frame control (0xC843)
                0x5A, // sequence number
                0xFF, 0xFF, // destination PAN = 0xFFFF
                0xFF, 0xFF, // destination address = 0xFFFF
                0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // source IEEE
                0x06, // Orphan Notification
            ]
        );
        assert_eq!(parse_orphan_notification(&frame), Some(ORPHAN_IEEE));
    }

    #[test]
    fn orphan_notification_parser_rejects_non_normative_framing() {
        let good = build_orphan_notification(1, &ORPHAN_IEEE);

        // A short source cannot identify an orphan.
        let mut short_source = good.clone();
        short_source[1] = 0x88; // src mode 0b10
        assert_eq!(parse_orphan_notification(&short_source), None);

        // Unicast destination is not an orphan notification.
        let mut unicast = good.clone();
        unicast[5] = 0x34;
        unicast[6] = 0x12;
        assert_eq!(parse_orphan_notification(&unicast), None);

        // Wrong command identifier.
        let mut wrong_command = good.clone();
        let last = wrong_command.len() - 1;
        wrong_command[last] = 0x07;
        assert_eq!(parse_orphan_notification(&wrong_command), None);

        // Without PAN ID compression the addressing layout differs.
        let mut no_compression = good.clone();
        no_compression[0] = 0x03;
        assert_eq!(parse_orphan_notification(&no_compression), None);

        // A data frame is not a command frame, and a truncated frame is not a
        // frame at all.
        let mut data_frame = good.clone();
        data_frame[0] = 0x01;
        assert_eq!(parse_orphan_notification(&data_frame), None);
        assert_eq!(parse_orphan_notification(&good[..good.len() - 1]), None);

        // An all-zero / broadcast "orphan" identity is not a device.
        let mut zero_orphan = good.clone();
        zero_orphan[7..15].copy_from_slice(&[0u8; 8]);
        assert_eq!(parse_orphan_notification(&zero_orphan), None);
    }

    #[test]
    fn coordinator_realignment_matches_the_ieee_802_15_4_golden_frame() {
        // IEEE 802.15.4 §7.3.8 orphan response: FC = command | ACK request |
        // extended dst | extended src = 0xCC23 (frame version 0, as R22
        // Annex D requires), destination PAN 0xFFFF with the orphan's IEEE,
        // source PAN = macPANId with the parent's IEEE, then the payload.
        let frame = build_coordinator_realignment_orphan_response(
            0x7E,
            PanId(0x1A62),
            ShortAddress(0x0000),
            &PARENT_IEEE,
            15,
            &ORPHAN_IEEE,
            ShortAddress(0x89AB),
        )
        .unwrap();
        assert_eq!(
            frame.as_slice(),
            &[
                0x23, 0xCC, // frame control
                0x7E, // sequence number
                0xFF, 0xFF, // destination PAN = 0xFFFF
                0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // orphan IEEE
                0x62, 0x1A, // source PAN = macPANId
                0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, // parent IEEE
                0x08, // Coordinator Realignment
                0x62, 0x1A, // PAN identifier
                0x00, 0x00, // coordinator short address
                0x0F, // logical channel
                0xAB, 0x89, // realigned device short address
            ]
        );
        let frame_control = u16::from_le_bytes([frame[0], frame[1]]);
        assert_eq!((frame_control >> 12) & 0x03, 0, "frame version must be 0");

        let (realignment, source, destination) =
            parse_coordinator_realignment_orphan_response(&frame).unwrap();
        assert_eq!(realignment.pan_id, PanId(0x1A62));
        assert_eq!(realignment.coordinator_short, ShortAddress(0x0000));
        assert_eq!(realignment.channel, 15);
        assert_eq!(realignment.short_address, ShortAddress(0x89AB));
        assert_eq!(source, PARENT_IEEE);
        assert_eq!(destination, ORPHAN_IEEE);
    }

    #[test]
    fn coordinator_realignment_rejects_unusable_parameters() {
        let build = |pan: PanId, coord: ShortAddress, channel: u8, short: ShortAddress| {
            build_coordinator_realignment_orphan_response(
                0,
                pan,
                coord,
                &PARENT_IEEE,
                channel,
                &ORPHAN_IEEE,
                short,
            )
        };
        // Broadcast PAN is never a real network.
        assert_eq!(
            build(PanId::BROADCAST, ShortAddress(0), 15, ShortAddress(2)),
            Err(FrameBuildError::InvalidParameter)
        );
        // A realigned device must get a usable unicast address.
        for short in [0x0000u16, 0xFFFE, 0xFFFF] {
            assert_eq!(
                build(PanId(1), ShortAddress(0), 15, ShortAddress(short)),
                Err(FrameBuildError::InvalidParameter)
            );
        }
        // The parent may not hand the orphan its own address.
        assert_eq!(
            build(PanId(1), ShortAddress(0x1234), 15, ShortAddress(0x1234)),
            Err(FrameBuildError::InvalidParameter)
        );
        // Channels outside the 2.4 GHz page-0 range are not operable.
        for channel in [0u8, 10, 27] {
            assert_eq!(
                build(PanId(1), ShortAddress(0), channel, ShortAddress(2)),
                Err(FrameBuildError::InvalidParameter)
            );
        }
    }

    #[test]
    fn coordinator_realignment_parser_rejects_a_pan_id_mismatch() {
        let mut frame = build_coordinator_realignment_orphan_response(
            0,
            PanId(0x1A62),
            ShortAddress(0x0000),
            &PARENT_IEEE,
            15,
            &ORPHAN_IEEE,
            ShortAddress(0x89AB),
        )
        .unwrap();
        // Payload PAN identifier no longer matches the MHR source PAN.
        frame[24] = 0x63;
        assert_eq!(parse_coordinator_realignment_orphan_response(&frame), None);
    }

    #[test]
    fn orphan_and_realignment_parsers_reject_non_r22_frame_control_and_trailing_data() {
        let orphan = build_orphan_notification(1, &ORPHAN_IEEE);
        let mut secured_orphan = orphan.clone();
        secured_orphan[0] |= 1 << 3;
        assert_eq!(parse_orphan_notification(&secured_orphan), None);
        let mut versioned_orphan = orphan.clone();
        versioned_orphan[1] |= 1 << 4;
        assert_eq!(parse_orphan_notification(&versioned_orphan), None);
        let mut extended_orphan = orphan.clone();
        extended_orphan.push(0).unwrap();
        assert_eq!(parse_orphan_notification(&extended_orphan), None);

        let realignment = build_coordinator_realignment_orphan_response(
            0,
            PanId(0x1A62),
            ShortAddress(0),
            &PARENT_IEEE,
            15,
            &ORPHAN_IEEE,
            ShortAddress(0x89AB),
        )
        .unwrap();
        let mut versioned_realignment = realignment.clone();
        versioned_realignment[1] |= 1 << 4;
        assert_eq!(
            parse_coordinator_realignment_orphan_response(&versioned_realignment),
            None
        );
        let mut wrong_destination_pan = realignment.clone();
        wrong_destination_pan[3] = 0x34;
        wrong_destination_pan[4] = 0x12;
        assert_eq!(
            parse_coordinator_realignment_orphan_response(&wrong_destination_pan),
            None
        );
        let mut extended_realignment = realignment.clone();
        extended_realignment.push(0).unwrap();
        assert_eq!(
            parse_coordinator_realignment_orphan_response(&extended_realignment),
            None
        );
    }
}
