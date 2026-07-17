//! Pure, host-testable IEEE 802.15.4 MAC framing and DMA helpers:
//! the TLSR8258 5-byte DMA header format, and length/CRC/Beacon parsing.
//!
//! Every function here operates on plain `&[u8]`/`&mut [u8]` slices, not
//! raw hardware pointers, so it compiles and runs identically on the tc32
//! target and on the host (`cargo test`). `radio::hw` is the thin unsafe
//! wrapper around real DMA buffers that calls into this module.

/// MAC Command frame: "Beacon Request" (command ID 0x07).
///
/// Layout, matching `examples/telink-tlsr8258-sensor::build_beacon_request` /
/// `send_beacon_request` (proven on hardware):
///
/// | Offset | Field                | Value                              |
/// |--------|----------------------|-------------------------------------|
/// | 0..2   | Frame Control        | `0x0803` (MAC cmd, short dst, no src, no ack) |
/// | 2      | Sequence number      | caller-supplied                    |
/// | 3..5   | Destination PAN ID   | `0xFFFF` (broadcast)                |
/// | 5..7   | Destination address  | `0xFFFF` (broadcast)                |
/// | 7      | Command ID           | `0x07` (Beacon Request)             |
pub const BEACON_REQUEST_LEN: usize = 8;

pub const FC_LO_BEACON_REQUEST: u8 = 0x03;
pub const FC_HI_BEACON_REQUEST: u8 = 0x08;
pub const CMD_ID_BEACON_REQUEST: u8 = 0x07;
pub const CMD_ID_ASSOCIATION_REQUEST: u8 = 0x01;
pub const CMD_ID_ASSOCIATION_RESPONSE: u8 = 0x02;
pub const CMD_ID_DATA_REQUEST: u8 = 0x04;

pub fn beacon_request_mac_frame(seq: u8) -> [u8; BEACON_REQUEST_LEN] {
    [
        FC_LO_BEACON_REQUEST,
        FC_HI_BEACON_REQUEST,
        seq,
        0xFF,
        0xFF, // dst PAN = broadcast
        0xFF,
        0xFF, // dst addr = broadcast
        CMD_ID_BEACON_REQUEST,
    ]
}

pub fn association_request_short(
    seq: u8,
    pan_id: u16,
    coordinator: u16,
    source_ieee: [u8; 8],
    capability: u8,
) -> [u8; 17] {
    let mut frame = [0u8; 17];
    frame[0] = 0x63;
    frame[1] = 0xC8;
    frame[2] = seq;
    frame[3] = pan_id as u8;
    frame[4] = (pan_id >> 8) as u8;
    frame[5] = coordinator as u8;
    frame[6] = (coordinator >> 8) as u8;
    frame[7] = source_ieee[0];
    frame[8] = source_ieee[1];
    frame[9] = source_ieee[2];
    frame[10] = source_ieee[3];
    frame[11] = source_ieee[4];
    frame[12] = source_ieee[5];
    frame[13] = source_ieee[6];
    frame[14] = source_ieee[7];
    frame[15] = CMD_ID_ASSOCIATION_REQUEST;
    frame[16] = capability;
    frame
}

pub fn data_request_short(
    seq: u8,
    pan_id: u16,
    coordinator: u16,
    source_ieee: [u8; 8],
) -> [u8; 16] {
    let mut frame = [0u8; 16];
    frame[0] = 0x63;
    frame[1] = 0xC8;
    frame[2] = seq;
    frame[3] = pan_id as u8;
    frame[4] = (pan_id >> 8) as u8;
    frame[5] = coordinator as u8;
    frame[6] = (coordinator >> 8) as u8;
    frame[7] = source_ieee[0];
    frame[8] = source_ieee[1];
    frame[9] = source_ieee[2];
    frame[10] = source_ieee[3];
    frame[11] = source_ieee[4];
    frame[12] = source_ieee[5];
    frame[13] = source_ieee[6];
    frame[14] = source_ieee[7];
    frame[15] = CMD_ID_DATA_REQUEST;
    frame
}

pub fn data_request_associated_short(
    seq: u8,
    pan_id: u16,
    coordinator: u16,
    source: u16,
) -> [u8; 10] {
    [
        0x63,
        0x88,
        seq,
        pan_id as u8,
        (pan_id >> 8) as u8,
        coordinator as u8,
        (coordinator >> 8) as u8,
        source as u8,
        (source >> 8) as u8,
        CMD_ID_DATA_REQUEST,
    ]
}

pub fn data_frame_short(seq: u8, pan_id: u16, destination: u16, source: u16) -> [u8; 9] {
    [
        0x61,
        0x88,
        seq,
        pan_id as u8,
        (pan_id >> 8) as u8,
        destination as u8,
        (destination >> 8) as u8,
        source as u8,
        (source >> 8) as u8,
    ]
}

pub fn ack_info(psdu: &[u8]) -> Option<(u8, bool)> {
    if psdu.len() < 3 {
        return None;
    }
    let frame_control = u16::from_le_bytes([psdu[0], psdu[1]]);
    if frame_control & 0x07 != 0x02 {
        return None;
    }
    Some((psdu[2], frame_control & (1 << 4) != 0))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AssociationResponse {
    pub sequence: u8,
    pub short_address: u16,
    pub status: u8,
}

pub fn parse_association_response(psdu: &[u8]) -> Option<AssociationResponse> {
    if psdu.len() < 7 {
        return None;
    }
    let frame_control = u16::from_le_bytes([psdu[0], psdu[1]]);
    if frame_control & 0x07 != 0x03 {
        return None;
    }
    let command_offset = 3usize.checked_add(addressing_size(frame_control))?;
    if psdu.len() < command_offset + 4 || psdu[command_offset] != CMD_ID_ASSOCIATION_RESPONSE {
        return None;
    }
    Some(AssociationResponse {
        sequence: psdu[2],
        short_address: u16::from_le_bytes([psdu[command_offset + 1], psdu[command_offset + 2]]),
        status: psdu[command_offset + 3],
    })
}

/// Errors from [`encode_tx_dma`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// `mac_frame` plus the 5-byte DMA header does not fit in `buf`.
    BufferTooSmall,
    /// `mac_frame` is too long to fit in the DMA header's 8-bit length field.
    FrameTooLong,
}

/// Write `mac_frame` into `buf` using the TLSR8258 5-byte DMA TX header:
/// `buf[0] = len+1`, `buf[1..4] = 0`, `buf[4] = len+2` (hardware appends the
/// 2-byte CRC that `len+2` accounts for), `buf[5..5+len] = mac_frame`.
/// Returns the total number of bytes written (`5 + mac_frame.len()`).
pub fn encode_tx_dma(buf: &mut [u8], mac_frame: &[u8]) -> Result<usize, EncodeError> {
    let len = mac_frame.len();
    if len > 253 {
        // len+2 must fit in a u8 DMA header field.
        return Err(EncodeError::FrameTooLong);
    }
    let total = 5 + len;
    if buf.len() < total {
        return Err(EncodeError::BufferTooSmall);
    }
    buf[0] = (len as u8).wrapping_add(1);
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = (len as u8).wrapping_add(2);
    buf[5..total].copy_from_slice(mac_frame);
    Ok(total)
}

/// `RF_ZIGBEE_PACKET_LENGTH_OK(p)`: `p[0] == p[4] + 9` (TLSR825x SDK macro).
/// `buf` is the raw RX DMA buffer (5-byte header + PSDU + HW trailer).
pub fn packet_length_ok(buf: &[u8]) -> bool {
    if buf.len() < 5 {
        return false;
    }
    buf[0] as u16 == buf[4] as u16 + 9
}

/// `RF_ZIGBEE_PACKET_CRC_OK(p)`: `(p[p[0]+3] & 0x51) == 0x10`.
pub fn packet_crc_ok(buf: &[u8]) -> bool {
    let total_len = buf.first().copied().unwrap_or(0) as usize;
    if total_len == 0 || total_len > 136 {
        return false;
    }
    match buf.get(total_len + 3) {
        Some(&status) => (status & 0x51) == 0x10,
        None => false,
    }
}

/// `p[p[0]+2] - 110`: RSSI in dBm, per the TLSR825x SDK RX buffer layout.
pub fn packet_rssi(buf: &[u8]) -> i8 {
    let total_len = buf.first().copied().unwrap_or(0) as usize;
    if total_len == 0 || total_len > 136 {
        return -110;
    }
    match buf.get(total_len + 2) {
        Some(&b) => (b as i8).wrapping_sub(110),
        None => -110,
    }
}

/// MAC PSDU payload length, i.e. `buf[4]`.
pub fn payload_len(buf: &[u8]) -> u8 {
    buf.get(4).copied().unwrap_or(0)
}

/// The MAC PSDU itself, i.e. `buf[5..5+payload_len]`, or `None` if `buf` is
/// shorter than that (which `packet_length_ok`/`packet_crc_ok` should have
/// already ruled out — this is a defensive second check).
pub fn mac_psdu(buf: &[u8]) -> Option<&[u8]> {
    let len = payload_len(buf) as usize;
    buf.get(5..5 + len)
}

/// RSSI (dBm) -> LQI (0..255), matching
/// `examples/telink-tlsr8258-sensor::rssi_to_lqi`.
pub fn rssi_to_lqi(rssi: i8) -> u8 {
    let v = (rssi as i16 + 106).clamp(0, 100);
    ((v * 255) / 100) as u8
}

/// Coordinator address extracted from a parsed Beacon frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordAddress {
    Short(u16),
    Extended([u8; 8]),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SuperframeInfo {
    pub beacon_order: u8,
    pub superframe_order: u8,
    pub final_cap_slot: u8,
    pub battery_life_extension: bool,
    pub pan_coordinator: bool,
    pub association_permit: bool,
}

impl SuperframeInfo {
    pub fn from_raw(raw: u16) -> Self {
        Self {
            beacon_order: (raw & 0x0F) as u8,
            superframe_order: ((raw >> 4) & 0x0F) as u8,
            final_cap_slot: ((raw >> 8) & 0x0F) as u8,
            battery_life_extension: (raw >> 12) & 1 != 0,
            pan_coordinator: (raw >> 14) & 1 != 0,
            association_permit: (raw >> 15) & 1 != 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZigbeeBeaconInfo {
    pub protocol_id: u8,
    pub stack_profile: u8,
    pub protocol_version: u8,
    pub router_capacity: bool,
    pub device_depth: u8,
    pub end_device_capacity: bool,
    pub extended_pan_id: [u8; 8],
    pub tx_offset: [u8; 3],
    pub update_id: u8,
}

/// Fields extracted from a length/CRC-valid Beacon frame's MAC PSDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BeaconInfo {
    pub pan_id: u16,
    pub coord_address: CoordAddress,
    pub association_permit: bool,
    pub sequence: u8,
    pub superframe: SuperframeInfo,
    pub zigbee: Option<ZigbeeBeaconInfo>,
}

/// Size, in bytes, of the addressing fields (dst PAN/addr + src PAN/addr)
/// implied by a Frame Control field, per IEEE 802.15.4-2006 §7.2.1.
/// Transcribed from `examples/telink-tlsr8258-sensor::addressing_size`.
pub fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut size = 0;
    match dst_mode {
        0x02 => size += 4,
        0x03 => size += 10,
        _ => {}
    }
    match src_mode {
        0x02 => size += if pan_compress { 2 } else { 4 },
        0x03 => size += if pan_compress { 8 } else { 10 },
        _ => {}
    }
    size
}

/// Parse `psdu` (the MAC PSDU, i.e. `mac_psdu(dma_buf)`, *not* the raw DMA
/// buffer) as a Beacon frame. Returns `None` if it is not a well-formed
/// Beacon (wrong frame type, no usable source address, or too short for its
/// own addressing-mode-implied length).
///
/// This intentionally stops at the MAC/superframe layer — it does not parse
/// the NWK-layer Zigbee beacon payload (protocol id / stack profile /
/// extended PAN id), which `mac_test` does not need for a raw PHY/MAC
/// bring-up scan.
pub fn parse_beacon(psdu: &[u8]) -> Option<BeaconInfo> {
    if psdu.len() < 5 {
        return None;
    }
    let fc = u16::from_le_bytes([psdu[0], psdu[1]]);
    if fc & 0x07 != 0 {
        return None; // not a Beacon frame (frame type must be 0b000)
    }
    let sequence = psdu[2];

    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let superframe_offset = 3 + addressing_size(fc);
    if psdu.len() < superframe_offset + 2 {
        return None;
    }
    let sf_raw = u16::from_le_bytes([psdu[superframe_offset], psdu[superframe_offset + 1]]);
    let superframe = SuperframeInfo::from_raw(sf_raw);

    // Source address parsing (dst address for a Beacon is always "not
    // present", i.e. dst_mode should be 0, but tolerate malformed/spoofed
    // frames the same way the sensor lab's parser does: skip whatever the
    // FC claims is there).
    let mut offset = 3usize;
    let dst_pan = if dst_mode >= 2 && psdu.len() > offset + 1 {
        let pan = u16::from_le_bytes([psdu[offset], psdu[offset + 1]]);
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
    let src_pan = if !pan_compress && src_mode >= 2 && psdu.len() > offset + 1 {
        let pan = u16::from_le_bytes([psdu[offset], psdu[offset + 1]]);
        offset += 2;
        pan
    } else {
        dst_pan.unwrap_or(0xFFFF)
    };

    let coord_address = match src_mode {
        0x02 if psdu.len() >= offset + 2 => {
            CoordAddress::Short(u16::from_le_bytes([psdu[offset], psdu[offset + 1]]))
        }
        0x03 if psdu.len() >= offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&psdu[offset..offset + 8]);
            CoordAddress::Extended(ext)
        }
        _ => return None, // no usable source address: not a valid Beacon
    };

    let mut payload_offset = superframe_offset + 2;
    let gts_spec = *psdu.get(payload_offset)?;
    payload_offset += 1;
    let gts_count = (gts_spec & 0x07) as usize;
    if gts_count != 0 {
        payload_offset = payload_offset.checked_add(1 + gts_count * 3)?;
    }

    let pending_spec = *psdu.get(payload_offset)?;
    payload_offset += 1;
    let short_pending = (pending_spec & 0x07) as usize;
    let extended_pending = ((pending_spec >> 4) & 0x07) as usize;
    payload_offset = payload_offset.checked_add(short_pending * 2 + extended_pending * 8)?;
    if payload_offset > psdu.len() {
        return None;
    }

    let zigbee = parse_zigbee_beacon(psdu.get(payload_offset..).unwrap_or(&[]));

    Some(BeaconInfo {
        pan_id: src_pan,
        coord_address,
        association_permit: superframe.association_permit,
        sequence,
        superframe,
        zigbee,
    })
}

fn parse_zigbee_beacon(data: &[u8]) -> Option<ZigbeeBeaconInfo> {
    if data.len() < 15 {
        return None;
    }
    let nwk_info = u16::from_le_bytes([data[1], data[2]]);
    let mut extended_pan_id = [0u8; 8];
    extended_pan_id.copy_from_slice(&data[3..11]);
    let mut tx_offset = [0u8; 3];
    tx_offset.copy_from_slice(&data[11..14]);
    Some(ZigbeeBeaconInfo {
        protocol_id: data[0],
        stack_profile: (nwk_info & 0x0F) as u8,
        protocol_version: ((nwk_info >> 4) & 0x0F) as u8,
        router_capacity: (nwk_info >> 10) & 1 != 0,
        device_depth: ((nwk_info >> 11) & 0x0F) as u8,
        end_device_capacity: (nwk_info >> 15) & 1 != 0,
        extended_pan_id,
        tx_offset,
        update_id: data[14],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beacon_request_matches_known_good_bytes() {
        // Exact bytes proven on hardware by
        // examples/telink-tlsr8258-sensor::build_beacon_request(seq).
        assert_eq!(
            beacon_request_mac_frame(0x42),
            [0x03, 0x08, 0x42, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
        );
    }

    #[test]
    fn encode_tx_dma_matches_known_good_header() {
        let frame = beacon_request_mac_frame(7);
        let mut buf = [0u8; 32];
        let n = encode_tx_dma(&mut buf, &frame).unwrap();
        assert_eq!(n, 13);
        assert_eq!(buf[0], 9); // mac_len(8) + 1
        assert_eq!(buf[1], 0);
        assert_eq!(buf[2], 0);
        assert_eq!(buf[3], 0);
        assert_eq!(buf[4], 10); // mac_len(8) + 2
        assert_eq!(&buf[5..13], &frame);
    }

    #[test]
    fn association_request_matches_known_good_layout() {
        let frame = association_request_short(0x22, 0xDFE9, 0x7D2D, [1, 2, 3, 4, 5, 6, 7, 8], 0xC0);
        assert_eq!(
            frame,
            [
                0x63, 0xC8, 0x22, 0xE9, 0xDF, 0x2D, 0x7D, 1, 2, 3, 4, 5, 6, 7, 8, 0x01, 0xC0,
            ]
        );
    }

    #[test]
    fn associated_poll_and_data_frames_match_golden_layout() {
        assert_eq!(
            data_request_associated_short(0x34, 0xDFE9, 0xB5A2, 0x4A0D),
            [0x63, 0x88, 0x34, 0xE9, 0xDF, 0xA2, 0xB5, 0x0D, 0x4A, 0x04]
        );
        assert_eq!(
            data_frame_short(0x35, 0xDFE9, 0xB5A2, 0x4A0D),
            [0x61, 0x88, 0x35, 0xE9, 0xDF, 0xA2, 0xB5, 0x0D, 0x4A]
        );
    }

    #[test]
    fn ack_and_association_response_parse() {
        assert_eq!(ack_info(&[0x12, 0x00, 0x33]), Some((0x33, true)));
        let response = [
            0x43, 0x88, 0x44, // command, PAN compression, short dst/src
            0xE9, 0xDF, 0x34, 0x12, // destination PAN/address
            0x00, 0x00, // source coordinator
            0x02, 0x78, 0x56, 0x00, // response, assigned address, success
        ];
        assert_eq!(
            parse_association_response(&response),
            Some(AssociationResponse {
                sequence: 0x44,
                short_address: 0x5678,
                status: 0,
            })
        );
    }

    #[test]
    fn encode_tx_dma_rejects_undersized_buffer() {
        let frame = beacon_request_mac_frame(1);
        let mut buf = [0u8; 4];
        assert_eq!(
            encode_tx_dma(&mut buf, &frame),
            Err(EncodeError::BufferTooSmall)
        );
    }

    #[test]
    fn length_ok_matches_sdk_macro() {
        // p[0] == p[4] + 9
        let buf = [15u8, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        assert!(packet_length_ok(&buf));
        let mut bad = buf;
        bad[0] = 14;
        assert!(!packet_length_ok(&bad));
    }

    #[test]
    fn length_ok_rejects_short_buffer() {
        assert!(!packet_length_ok(&[1, 2, 3]));
    }

    #[test]
    fn crc_ok_matches_sdk_macro() {
        let mut buf = [0u8; 32];
        buf[0] = 20; // total_len
        buf[20 + 3] = 0x10; // status byte, masked bits match
        assert!(packet_crc_ok(&buf));
        buf[20 + 3] = 0x00;
        assert!(!packet_crc_ok(&buf));
        buf[20 + 3] = 0x51; // extra bits set outside the mask are fine
        assert!(!packet_crc_ok(&buf)); // 0x51 & 0x51 == 0x51 != 0x10
    }

    #[test]
    fn crc_ok_rejects_absurd_length() {
        let mut buf = [0u8; 8];
        buf[0] = 200; // way past 130 and past buf bounds
        assert!(!packet_crc_ok(&buf));
    }

    #[test]
    fn rssi_and_lqi_formulas() {
        let mut buf = [0u8; 32];
        buf[0] = 20;
        buf[20 + 2] = 130; // rssi raw byte
        assert_eq!(packet_rssi(&buf), (130i16 - 110) as i8);
        assert_eq!(rssi_to_lqi(-40), 168); // (-40+106)=66 -> 66*255/100=168.3 -> 168 (truncated)
    }

    #[test]
    fn rssi_to_lqi_clamps_to_valid_range() {
        assert_eq!(rssi_to_lqi(-120), 0);
        assert_eq!(rssi_to_lqi(10), 255);
    }

    /// Golden vector: a realistic Zigbee coordinator Beacon frame with short
    /// source addressing, PAN compression, and association permitted.
    /// FC = 0x8000 (src_mode=short) with association_permit bit set in the
    /// superframe spec's bit 15.
    #[test]
    fn parse_beacon_golden_vector_short_addr() {
        let psdu: [u8; 11] = [
            0x00, 0x80, // FC: src addr mode = short (0b10 << 14), everything else 0
            0x2A, // sequence
            0x34, 0x12, // src PAN = 0x1234 (dst_mode=0 so this is parsed as src PAN)
            0x78, 0x56, // src short addr = 0x5678
            0x00, 0x80, // superframe spec: bit15 (association permit) set
            0x00, 0x00, // GTS + pending fields not parsed, padding
        ];
        let info = parse_beacon(&psdu).expect("golden beacon must parse");
        assert_eq!(info.pan_id, 0x1234);
        assert_eq!(info.coord_address, CoordAddress::Short(0x5678));
        assert!(info.association_permit);
        assert!(info.superframe.association_permit);
        assert_eq!(info.zigbee, None);
        assert_eq!(info.sequence, 0x2A);
    }

    #[test]
    fn parse_beacon_rejects_non_beacon_frame_type() {
        let mut psdu: [u8; 12] = [
            0x00, 0x80, 0x2A, 0x34, 0x12, 0x78, 0x56, 0x00, 0x80, 0x00, 0x00, 0x00,
        ];
        psdu[0] |= 0x01; // frame type = 1 (data), no longer a beacon
        assert_eq!(parse_beacon(&psdu), None);
    }

    #[test]
    fn parse_beacon_rejects_truncated_frame() {
        let psdu: [u8; 4] = [0x00, 0x80, 0x2A, 0x34];
        assert_eq!(parse_beacon(&psdu), None);
    }

    #[test]
    fn parse_beacon_extended_source_address() {
        let mut psdu = [0u8; 20];
        psdu[0] = 0x00;
        psdu[1] = 0xC0; // src addr mode = extended (0b11 << 14)
        psdu[2] = 0x07; // sequence
        psdu[3] = 0xCD;
        psdu[4] = 0xAB; // src PAN = 0xABCD
        let ext = [1u8, 2, 3, 4, 5, 6, 7, 8];
        psdu[5..13].copy_from_slice(&ext);
        psdu[13] = 0x00;
        psdu[14] = 0x00; // superframe spec, association not permitted
        let info = parse_beacon(&psdu).expect("extended-addr beacon must parse");
        assert_eq!(info.pan_id, 0xABCD);
        assert_eq!(info.coord_address, CoordAddress::Extended(ext));
        assert!(!info.association_permit);
    }

    #[test]
    fn parse_full_zigbee_beacon_descriptor() {
        let psdu: [u8; 26] = [
            0x00, 0x80, 0x55, // Beacon FC + sequence
            0xE9, 0xDF, 0x2D, 0x7D, // PAN 0xDFE9, coordinator 0x7D2D
            0xFF, 0xCF, // superframe: coordinator + association permit
            0x00, // no GTS descriptors
            0x00, // no pending addresses
            0x00, // Zigbee protocol ID
            0x22, 0x84, // stack profile 2, protocol version 2, capacities/depth
            1, 2, 3, 4, 5, 6, 7, 8, // extended PAN ID
            0, 0, 0,    // TX offset
            0x09, // update ID
        ];
        let info = parse_beacon(&psdu).expect("full Zigbee beacon must parse");
        let zigbee = info.zigbee.expect("Zigbee payload must parse");
        assert_eq!(zigbee.protocol_id, 0);
        assert_eq!(zigbee.stack_profile, 2);
        assert_eq!(zigbee.protocol_version, 2);
        assert_eq!(zigbee.extended_pan_id, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(zigbee.update_id, 9);
    }

    #[test]
    fn beacon_payload_offset_skips_gts_and_pending_addresses() {
        let mut psdu = [0u8; 42];
        psdu[..7].copy_from_slice(&[0x00, 0x80, 1, 0x34, 0x12, 0x78, 0x56]);
        psdu[7..9].copy_from_slice(&0x8000u16.to_le_bytes());
        psdu[9] = 1; // one GTS descriptor
        psdu[10] = 0; // GTS directions
        psdu[11..14].copy_from_slice(&[1, 2, 3]);
        psdu[14] = 0x11; // one short + one extended pending address
        psdu[15..17].copy_from_slice(&[4, 5]);
        psdu[17..25].copy_from_slice(&[6, 7, 8, 9, 10, 11, 12, 13]);
        psdu[25] = 0;
        psdu[26..28].copy_from_slice(&0x0022u16.to_le_bytes());
        psdu[28..36].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        psdu[36..39].copy_from_slice(&[0, 0, 0]);
        psdu[39] = 3;
        let zigbee = parse_beacon(&psdu)
            .and_then(|info| info.zigbee)
            .expect("payload after variable beacon fields must parse");
        assert_eq!(zigbee.update_id, 3);
    }
}
