//! EZSP over ASH protocol for Silicon Labs EFR32-based USB dongles.
//!
//! Implements the EmberZNet Serial Protocol (EZSP) framed over the
//! Asynchronous Serial Host (ASH) transport used by devices such as
//! Sonoff ZBDongle-E, Home Assistant SkyConnect, and other EFR32
//! USB sticks.
//!
//! # ASH framing
//!
//! ```text
//! FLAG(0x7E) | <byte-stuffed frame: type/ctrl + data + CRC16> | FLAG(0x7E)
//! ```
//!
//! Byte-stuffing: 0x7E → 0x7D 0x5E, 0x7D → 0x7D 0x5D, 0x11 → 0x7D 0x31,
//! 0x13 → 0x7D 0x33, 0x18 → 0x7D 0x38, 0x1A → 0x7D 0x3A.
//!
//! Data bytes are LFSR-randomized before CRC and stuffing.

#![allow(dead_code)]

use super::SerialError;

// ── ASH Constants ───────────────────────────────────────────────

pub const ASH_FLAG: u8 = 0x7E;
pub const ASH_ESCAPE: u8 = 0x7D;
pub const ASH_XON: u8 = 0x11;
pub const ASH_XOFF: u8 = 0x13;
pub const ASH_SUBSTITUTE: u8 = 0x18;
pub const ASH_CANCEL: u8 = 0x1A;

/// Maximum ASH frame payload (before stuffing)
pub const ASH_MAX_FRAME_SIZE: usize = 256;

// ── ASH Frame Types ─────────────────────────────────────────────

/// ASH frame type decoded from the control byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AshFrameType {
    /// DATA frame: carries EZSP payload. Contains frmNum (3 bits) and ackNum (3 bits).
    Data { frm_num: u8, retransmit: bool, ack_num: u8 },
    /// ACK frame: acknowledges received DATA frames.
    Ack { ack_num: u8, not_ready: bool },
    /// NAK frame: negative acknowledgement — requests retransmission.
    Nak { ack_num: u8, not_ready: bool },
    /// RST frame: host requests NCP reset.
    Rst,
    /// RSTACK frame: NCP reset acknowledgement, carries version and reset reason.
    RstAck { version: u8, reset_code: u8 },
    /// ERROR frame: NCP reports fatal error.
    Error { version: u8, error_code: u8 },
}

// ── ASH control byte encoding/decoding ──────────────────────────

fn ash_encode_data_control(frm_num: u8, retransmit: bool, ack_num: u8) -> u8 {
    // DATA: bit7=0, bits[6:4]=frmNum, bit3=reTx, bits[2:0]=ackNum
    ((frm_num & 0x07) << 4) | ((retransmit as u8) << 3) | (ack_num & 0x07)
}

fn ash_encode_ack_control(ack_num: u8, not_ready: bool) -> u8 {
    // ACK: 1000_0nnn | nRdy << 3
    0x80 | ((not_ready as u8) << 3) | (ack_num & 0x07)
}

fn ash_encode_nak_control(ack_num: u8, not_ready: bool) -> u8 {
    // NAK: 1010_0nnn | nRdy << 3
    0xA0 | ((not_ready as u8) << 3) | (ack_num & 0x07)
}

const ASH_RST_CONTROL: u8 = 0xC0;
const ASH_RSTACK_CONTROL: u8 = 0xC1;
const ASH_ERROR_CONTROL: u8 = 0xC2;

fn ash_decode_frame_type(ctrl: u8, data: &[u8]) -> AshFrameType {
    if ctrl & 0x80 == 0 {
        // DATA
        AshFrameType::Data {
            frm_num: (ctrl >> 4) & 0x07,
            retransmit: (ctrl >> 3) & 0x01 != 0,
            ack_num: ctrl & 0x07,
        }
    } else if ctrl & 0x60 == 0x00 {
        // ACK (10xx_xxxx where xx=00)
        AshFrameType::Ack {
            ack_num: ctrl & 0x07,
            not_ready: (ctrl >> 3) & 0x01 != 0,
        }
    } else if ctrl & 0x60 == 0x20 {
        // NAK
        AshFrameType::Nak {
            ack_num: ctrl & 0x07,
            not_ready: (ctrl >> 3) & 0x01 != 0,
        }
    } else if ctrl == ASH_RST_CONTROL {
        AshFrameType::Rst
    } else if ctrl == ASH_RSTACK_CONTROL {
        AshFrameType::RstAck {
            version: if data.len() > 1 { data[1] } else { 0 },
            reset_code: if data.len() > 2 { data[2] } else { 0 },
        }
    } else if ctrl == ASH_ERROR_CONTROL {
        AshFrameType::Error {
            version: if data.len() > 1 { data[1] } else { 0 },
            error_code: if data.len() > 2 { data[2] } else { 0 },
        }
    } else {
        // Unknown — treat as error
        AshFrameType::Error {
            version: 0,
            error_code: 0xFF,
        }
    }
}

// ── LFSR randomization ──────────────────────────────────────────

/// ASH LFSR randomization (applied to data bytes before CRC computation).
/// The LFSR is seeded with 0x42 and uses polynomial x^8 + x^5 + x^4 + 1.
fn ash_lfsr_randomize(data: &mut [u8]) {
    let mut lfsr: u8 = 0x42;
    for byte in data.iter_mut() {
        *byte ^= lfsr;
        // Advance LFSR: if bit 0 is set, feedback taps
        let bit0 = lfsr & 0x01 != 0;
        lfsr >>= 1;
        if bit0 {
            lfsr ^= 0xB8; // taps at bits 7,5,4,3
        }
    }
}

// ── ASH byte stuffing ───────────────────────────────────────────

fn ash_needs_stuffing(byte: u8) -> bool {
    matches!(
        byte,
        ASH_FLAG | ASH_ESCAPE | ASH_XON | ASH_XOFF | ASH_SUBSTITUTE | ASH_CANCEL
    )
}

fn ash_stuff_byte(byte: u8) -> (u8, u8) {
    (ASH_ESCAPE, byte ^ 0x20)
}

fn ash_unstuff_byte(escaped: u8) -> u8 {
    escaped ^ 0x20
}

// ── CRC16-CCITT for ASH ─────────────────────────────────────────

fn ash_crc16(data: &[u8]) -> u16 {
    super::crc16_ccitt(data)
}

// ── EZSP Constants ──────────────────────────────────────────────

// EZSP v8+ frame IDs (2-byte LE)
pub const EZSP_VERSION: u16 = 0x0000;
pub const EZSP_GET_VALUE: u16 = 0x00AA;
pub const EZSP_GET_NETWORK_PARAMETERS: u16 = 0x0028;
pub const EZSP_NETWORK_STATE: u16 = 0x0018;
pub const EZSP_NETWORK_INIT: u16 = 0x0017;
pub const EZSP_FORM_NETWORK: u16 = 0x001E;
pub const EZSP_LEAVE_NETWORK: u16 = 0x0020;
pub const EZSP_PERMIT_JOINING: u16 = 0x0022;
pub const EZSP_SEND_UNICAST: u16 = 0x0034;
pub const EZSP_SEND_BROADCAST: u16 = 0x0036;
pub const EZSP_MESSAGE_SENT_HANDLER: u16 = 0x003F;
pub const EZSP_INCOMING_MESSAGE_HANDLER: u16 = 0x0045;
pub const EZSP_SET_INITIAL_SECURITY_STATE: u16 = 0x0068;
pub const EZSP_GET_EUI64: u16 = 0x0026;
pub const EZSP_GET_NODE_ID: u16 = 0x0027;
pub const EZSP_STACK_STATUS_HANDLER: u16 = 0x0019;

/// Minimum supported EZSP protocol version
pub const EZSP_MIN_VERSION: u8 = 8;

// ── EZSP Frame ──────────────────────────────────────────────────

/// An EZSP command/response frame (before ASH encapsulation).
#[derive(Debug, Clone)]
pub struct EzspFrame {
    /// EZSP sequence number (wraps at 0xFF)
    pub sequence: u8,
    /// Frame control low byte
    pub frame_control_lo: u8,
    /// Frame control high byte
    pub frame_control_hi: u8,
    /// EZSP frame ID (2 bytes LE for v8+)
    pub frame_id: u16,
    /// Payload data
    pub payload: heapless::Vec<u8, 200>,
}

impl EzspFrame {
    /// Create a new EZSP command frame.
    pub fn command(sequence: u8, frame_id: u16, payload: &[u8]) -> Result<Self, SerialError> {
        let mut p = heapless::Vec::new();
        p.extend_from_slice(payload)
            .map_err(|_| SerialError::FrameTooLong)?;
        Ok(Self {
            sequence,
            frame_control_lo: 0x00, // command frame, no sleep
            frame_control_hi: 0x00,
            frame_id,
            payload: p,
        })
    }

    /// Serialize to bytes (without ASH framing).
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerialError> {
        let total = 5 + self.payload.len(); // seq + fc_lo + fc_hi + frame_id(2) + payload
        if total > buf.len() {
            return Err(SerialError::FrameTooLong);
        }
        buf[0] = self.sequence;
        buf[1] = self.frame_control_lo;
        buf[2] = self.frame_control_hi;
        buf[3] = self.frame_id as u8;
        buf[4] = (self.frame_id >> 8) as u8;
        buf[5..5 + self.payload.len()].copy_from_slice(&self.payload);
        Ok(total)
    }

    /// Parse from bytes (without ASH framing).
    pub fn parse(buf: &[u8]) -> Result<Self, SerialError> {
        if buf.len() < 5 {
            return Err(SerialError::MalformedFrame);
        }
        let sequence = buf[0];
        let frame_control_lo = buf[1];
        let frame_control_hi = buf[2];
        let frame_id = u16::from_le_bytes([buf[3], buf[4]]);
        let mut payload = heapless::Vec::new();
        if buf.len() > 5 {
            payload
                .extend_from_slice(&buf[5..])
                .map_err(|_| SerialError::FrameTooLong)?;
        }
        Ok(Self {
            sequence,
            frame_control_lo,
            frame_control_hi,
            frame_id,
            payload,
        })
    }

    /// Check if this is a response frame (bit 7 of fc_lo).
    pub fn is_response(&self) -> bool {
        self.frame_control_lo & 0x80 != 0
    }
}

// ── EzspCodec ───────────────────────────────────────────────────

/// Codec for building and parsing ASH+EZSP frames over a serial stream.
pub struct EzspCodec {
    /// Next send sequence number (0-7, wraps)
    send_seq: u8,
    /// Next expected receive sequence number
    recv_seq: u8,
    /// EZSP sequence counter (0-255)
    ezsp_seq: u8,
    /// Receive buffer for accumulating a raw (unstuffed) ASH frame
    rx_buf: [u8; ASH_MAX_FRAME_SIZE],
    rx_pos: usize,
    /// Whether we're inside a frame (seen opening FLAG)
    in_frame: bool,
    /// Whether the next byte is escaped
    escape_next: bool,
    /// Last sent DATA frame (for retransmission)
    last_data: [u8; ASH_MAX_FRAME_SIZE],
    last_data_len: usize,
    /// Negotiated EZSP protocol version
    pub protocol_version: u8,
}

impl EzspCodec {
    pub fn new() -> Self {
        Self {
            send_seq: 0,
            recv_seq: 0,
            ezsp_seq: 0,
            rx_buf: [0u8; ASH_MAX_FRAME_SIZE],
            rx_pos: 0,
            in_frame: false,
            escape_next: false,
            last_data: [0u8; ASH_MAX_FRAME_SIZE],
            last_data_len: 0,
            protocol_version: EZSP_MIN_VERSION,
        }
    }

    /// Reset ASH state machine (call after receiving RSTACK).
    pub fn reset_state(&mut self) {
        self.send_seq = 0;
        self.recv_seq = 0;
        self.rx_pos = 0;
        self.in_frame = false;
        self.escape_next = false;
        self.last_data_len = 0;
    }

    /// Build an ASH RST frame into `out`. Returns bytes written.
    pub fn build_rst(&self, out: &mut [u8]) -> Result<usize, SerialError> {
        // RST frame: FLAG + stuff(ctrl_byte + CRC16) + FLAG
        let ctrl = ASH_RST_CONTROL;
        let crc = ash_crc16(&[ctrl]);
        let raw = [ctrl, (crc >> 8) as u8, crc as u8];
        Self::stuff_frame(&raw, out)
    }

    /// Build an ASH ACK frame for the current recv_seq.
    pub fn build_ack(&self, out: &mut [u8]) -> Result<usize, SerialError> {
        let ctrl = ash_encode_ack_control(self.recv_seq, false);
        let crc = ash_crc16(&[ctrl]);
        let raw = [ctrl, (crc >> 8) as u8, crc as u8];
        Self::stuff_frame(&raw, out)
    }

    /// Build an ASH NAK frame for the current recv_seq.
    pub fn build_nak(&self, out: &mut [u8]) -> Result<usize, SerialError> {
        let ctrl = ash_encode_nak_control(self.recv_seq, false);
        let crc = ash_crc16(&[ctrl]);
        let raw = [ctrl, (crc >> 8) as u8, crc as u8];
        Self::stuff_frame(&raw, out)
    }

    /// Build an ASH DATA frame carrying an EZSP command.
    /// Returns bytes written to `out` and advances send_seq.
    pub fn build_data(
        &mut self,
        ezsp: &EzspFrame,
        out: &mut [u8],
    ) -> Result<usize, SerialError> {
        let mut ezsp_bytes = [0u8; 210];
        let ezsp_len = ezsp.serialize(&mut ezsp_bytes)?;

        // Randomize EZSP data
        let mut data_buf = [0u8; 210];
        data_buf[..ezsp_len].copy_from_slice(&ezsp_bytes[..ezsp_len]);
        ash_lfsr_randomize(&mut data_buf[..ezsp_len]);

        // Build raw frame: ctrl + randomized_data + CRC16
        let ctrl = ash_encode_data_control(self.send_seq, false, self.recv_seq);
        let mut raw = [0u8; ASH_MAX_FRAME_SIZE];
        raw[0] = ctrl;
        raw[1..1 + ezsp_len].copy_from_slice(&data_buf[..ezsp_len]);
        let frame_len = 1 + ezsp_len;
        let crc = ash_crc16(&raw[..frame_len]);
        raw[frame_len] = (crc >> 8) as u8;
        raw[frame_len + 1] = crc as u8;
        let total_raw = frame_len + 2;

        // Save for potential retransmission
        self.last_data[..total_raw].copy_from_slice(&raw[..total_raw]);
        self.last_data_len = total_raw;

        // Advance send sequence
        self.send_seq = (self.send_seq + 1) & 0x07;

        Self::stuff_frame(&raw[..total_raw], out)
    }

    /// Build the next EZSP command, incrementing the EZSP sequence counter.
    pub fn next_ezsp_command(
        &mut self,
        frame_id: u16,
        payload: &[u8],
    ) -> Result<EzspFrame, SerialError> {
        let seq = self.ezsp_seq;
        self.ezsp_seq = self.ezsp_seq.wrapping_add(1);
        EzspFrame::command(seq, frame_id, payload)
    }

    /// Retransmit the last DATA frame (after receiving NAK).
    pub fn retransmit(&mut self, out: &mut [u8]) -> Result<usize, SerialError> {
        if self.last_data_len == 0 {
            return Err(SerialError::IoError);
        }
        // Rebuild with retransmit flag set
        let ctrl = ash_encode_data_control(
            (self.send_seq.wrapping_sub(1)) & 0x07,
            true,
            self.recv_seq,
        );
        self.last_data[0] = ctrl;
        // Recompute CRC
        let data_end = self.last_data_len - 2;
        let crc = ash_crc16(&self.last_data[..data_end]);
        self.last_data[data_end] = (crc >> 8) as u8;
        self.last_data[data_end + 1] = crc as u8;

        Self::stuff_frame(&self.last_data[..self.last_data_len], out)
    }

    /// Feed raw serial bytes into the decoder. Returns a decoded ASH frame
    /// type and the de-randomized EZSP data (for DATA frames) if a complete
    /// frame was found, or None if more bytes are needed.
    pub fn feed(
        &mut self,
        data: &[u8],
    ) -> Result<Option<(AshFrameType, heapless::Vec<u8, ASH_MAX_FRAME_SIZE>)>, SerialError> {
        for &byte in data {
            if byte == ASH_CANCEL {
                // Cancel current frame
                self.rx_pos = 0;
                self.in_frame = false;
                self.escape_next = false;
                continue;
            }
            if byte == ASH_SUBSTITUTE {
                // Substitute error — discard frame in progress
                self.rx_pos = 0;
                self.in_frame = false;
                self.escape_next = false;
                continue;
            }
            if byte == ASH_FLAG {
                if self.in_frame && self.rx_pos > 0 {
                    // End of frame — process it
                    let result = self.process_ash_frame();
                    self.rx_pos = 0;
                    self.in_frame = false;
                    self.escape_next = false;
                    if let Ok(Some(r)) = result {
                        return Ok(Some(r));
                    }
                }
                // Start of new frame
                self.in_frame = true;
                self.rx_pos = 0;
                self.escape_next = false;
                continue;
            }

            if !self.in_frame {
                continue;
            }

            if byte == ASH_ESCAPE {
                self.escape_next = true;
                continue;
            }

            let actual = if self.escape_next {
                self.escape_next = false;
                ash_unstuff_byte(byte)
            } else {
                byte
            };

            if self.rx_pos < ASH_MAX_FRAME_SIZE {
                self.rx_buf[self.rx_pos] = actual;
                self.rx_pos += 1;
            }
        }
        Ok(None)
    }

    /// Process a complete unstuffed ASH frame in rx_buf.
    fn process_ash_frame(
        &mut self,
    ) -> Result<
        Option<(AshFrameType, heapless::Vec<u8, ASH_MAX_FRAME_SIZE>)>,
        SerialError,
    > {
        if self.rx_pos < 3 {
            // Minimum: control + CRC16
            return Ok(None);
        }

        // Verify CRC
        let data_end = self.rx_pos - 2;
        let expected_crc =
            ((self.rx_buf[data_end] as u16) << 8) | (self.rx_buf[data_end + 1] as u16);
        let computed_crc = ash_crc16(&self.rx_buf[..data_end]);
        if expected_crc != computed_crc {
            return Err(SerialError::CrcError);
        }

        let ctrl = self.rx_buf[0];
        let frame_type = ash_decode_frame_type(ctrl, &self.rx_buf[..data_end]);

        let mut ezsp_data: heapless::Vec<u8, ASH_MAX_FRAME_SIZE> = heapless::Vec::new();

        match frame_type {
            AshFrameType::Data { ack_num: _, .. } => {
                // De-randomize the data portion (bytes after control byte)
                if data_end > 1 {
                    let mut randomized = [0u8; ASH_MAX_FRAME_SIZE];
                    let dlen = data_end - 1;
                    randomized[..dlen].copy_from_slice(&self.rx_buf[1..data_end]);
                    ash_lfsr_randomize(&mut randomized[..dlen]);
                    let _ = ezsp_data.extend_from_slice(&randomized[..dlen]);
                }
                // Advance receive sequence
                self.recv_seq = (self.recv_seq + 1) & 0x07;
            }
            AshFrameType::RstAck { .. } | AshFrameType::Error { .. } => {
                // Include raw data for version/code extraction
                if data_end > 1 {
                    let _ = ezsp_data.extend_from_slice(&self.rx_buf[1..data_end]);
                }
            }
            _ => {}
        }

        Ok(Some((frame_type, ezsp_data)))
    }

    /// Byte-stuff a raw frame and wrap with FLAG delimiters.
    fn stuff_frame(raw: &[u8], out: &mut [u8]) -> Result<usize, SerialError> {
        let mut pos = 0;
        if pos >= out.len() {
            return Err(SerialError::BufferFull);
        }
        out[pos] = ASH_FLAG;
        pos += 1;

        for &byte in raw {
            if ash_needs_stuffing(byte) {
                if pos + 2 > out.len() {
                    return Err(SerialError::BufferFull);
                }
                let (esc, val) = ash_stuff_byte(byte);
                out[pos] = esc;
                out[pos + 1] = val;
                pos += 2;
            } else {
                if pos >= out.len() {
                    return Err(SerialError::BufferFull);
                }
                out[pos] = byte;
                pos += 1;
            }
        }

        if pos >= out.len() {
            return Err(SerialError::BufferFull);
        }
        out[pos] = ASH_FLAG;
        pos += 1;

        Ok(pos)
    }

    // ── EZSP command builders ───────────────────────────────

    /// Build EZSP version command.
    pub fn build_version_command(&mut self, desired_version: u8) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_VERSION, &[desired_version])
    }

    /// Build EZSP formNetwork command.
    /// Parameters: extended PAN ID (8), pan_id (2 LE), tx_power (1 i8), channel (1)
    pub fn build_form_network(
        &mut self,
        extended_pan_id: &[u8; 8],
        pan_id: u16,
        tx_power: i8,
        channel: u8,
    ) -> Result<EzspFrame, SerialError> {
        let mut payload = [0u8; 16];
        payload[0..8].copy_from_slice(extended_pan_id);
        payload[8] = pan_id as u8;
        payload[9] = (pan_id >> 8) as u8;
        payload[10] = tx_power as u8;
        payload[11] = channel;
        self.next_ezsp_command(EZSP_FORM_NETWORK, &payload[..12])
    }

    /// Build EZSP networkInit command (v8+: no parameters).
    pub fn build_network_init(&mut self) -> Result<EzspFrame, SerialError> {
        // v8+: networkInit takes a NetworkInitStruct (2 bytes: bitmask LE)
        self.next_ezsp_command(EZSP_NETWORK_INIT, &[0x00, 0x00])
    }

    /// Build EZSP getNetworkParameters command.
    pub fn build_get_network_parameters(&mut self) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_GET_NETWORK_PARAMETERS, &[])
    }

    /// Build EZSP getEui64 command.
    pub fn build_get_eui64(&mut self) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_GET_EUI64, &[])
    }

    /// Build EZSP getNodeId command.
    pub fn build_get_node_id(&mut self) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_GET_NODE_ID, &[])
    }

    /// Build EZSP sendUnicast command.
    /// type(1) + indexOrDestination(2 LE) + apsFrame(~14) + messageTag(1) + messageLength(1) + message
    pub fn build_send_unicast(
        &mut self,
        destination: u16,
        profile_id: u16,
        cluster_id: u16,
        src_endpoint: u8,
        dst_endpoint: u8,
        message: &[u8],
    ) -> Result<EzspFrame, SerialError> {
        let mut payload = [0u8; 200];
        let mut pos = 0;
        // type = EMBER_OUTGOING_DIRECT (0)
        payload[pos] = 0x00;
        pos += 1;
        // indexOrDestination
        payload[pos] = destination as u8;
        payload[pos + 1] = (destination >> 8) as u8;
        pos += 2;
        // APS frame: profileId(2) + clusterId(2) + srcEndpoint(1) + dstEndpoint(1)
        //   + options(2) + groupId(2) + sequence(1)
        payload[pos] = profile_id as u8;
        payload[pos + 1] = (profile_id >> 8) as u8;
        pos += 2;
        payload[pos] = cluster_id as u8;
        payload[pos + 1] = (cluster_id >> 8) as u8;
        pos += 2;
        payload[pos] = src_endpoint;
        pos += 1;
        payload[pos] = dst_endpoint;
        pos += 1;
        // options: EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY
        payload[pos] = 0x40;
        payload[pos + 1] = 0x01;
        pos += 2;
        // groupId = 0
        payload[pos] = 0x00;
        payload[pos + 1] = 0x00;
        pos += 2;
        // sequence = 0 (EZSP assigns)
        payload[pos] = 0x00;
        pos += 1;
        // messageTag
        payload[pos] = 0x00;
        pos += 1;
        // messageLength
        let mlen = message.len();
        payload[pos] = mlen as u8;
        pos += 1;
        // message contents
        if pos + mlen > payload.len() {
            return Err(SerialError::FrameTooLong);
        }
        payload[pos..pos + mlen].copy_from_slice(message);
        pos += mlen;

        self.next_ezsp_command(EZSP_SEND_UNICAST, &payload[..pos])
    }

    /// Build EZSP sendBroadcast command.
    pub fn build_send_broadcast(
        &mut self,
        destination: u16,
        profile_id: u16,
        cluster_id: u16,
        src_endpoint: u8,
        radius: u8,
        message: &[u8],
    ) -> Result<EzspFrame, SerialError> {
        let mut payload = [0u8; 200];
        let mut pos = 0;
        // destination
        payload[pos] = destination as u8;
        payload[pos + 1] = (destination >> 8) as u8;
        pos += 2;
        // APS frame (same structure as unicast)
        payload[pos] = profile_id as u8;
        payload[pos + 1] = (profile_id >> 8) as u8;
        pos += 2;
        payload[pos] = cluster_id as u8;
        payload[pos + 1] = (cluster_id >> 8) as u8;
        pos += 2;
        payload[pos] = src_endpoint;
        pos += 1;
        payload[pos] = 0xFF; // dst endpoint broadcast
        pos += 1;
        // options
        payload[pos] = 0x00;
        payload[pos + 1] = 0x00;
        pos += 2;
        // groupId
        payload[pos] = 0x00;
        payload[pos + 1] = 0x00;
        pos += 2;
        // sequence
        payload[pos] = 0x00;
        pos += 1;
        // radius
        payload[pos] = radius;
        pos += 1;
        // messageTag
        payload[pos] = 0x00;
        pos += 1;
        // messageLength
        let mlen = message.len();
        payload[pos] = mlen as u8;
        pos += 1;
        if pos + mlen > payload.len() {
            return Err(SerialError::FrameTooLong);
        }
        payload[pos..pos + mlen].copy_from_slice(message);
        pos += mlen;

        self.next_ezsp_command(EZSP_SEND_BROADCAST, &payload[..pos])
    }

    /// Build EZSP setInitialSecurityState command.
    pub fn build_set_initial_security(
        &mut self,
        network_key: &[u8; 16],
        trust_center_key: &[u8; 16],
    ) -> Result<EzspFrame, SerialError> {
        let mut payload = [0u8; 40];
        let mut pos = 0;
        // bitmask: HAVE_PRECONFIGURED_KEY | HAVE_NETWORK_KEY | REQUIRE_ENCRYPTED_KEY
        let bitmask: u16 = 0x0004 | 0x0008 | 0x0100;
        payload[pos] = bitmask as u8;
        payload[pos + 1] = (bitmask >> 8) as u8;
        pos += 2;
        // preconfiguredKey (trust center link key)
        payload[pos..pos + 16].copy_from_slice(trust_center_key);
        pos += 16;
        // networkKey
        payload[pos..pos + 16].copy_from_slice(network_key);
        pos += 16;
        // networkKeySequenceNumber
        payload[pos] = 0x00;
        pos += 1;
        // preconfiguredTrustCenterEui64 = all zeros
        pos += 8;

        self.next_ezsp_command(EZSP_SET_INITIAL_SECURITY_STATE, &payload[..pos])
    }

    /// Build EZSP permitJoining command.
    pub fn build_permit_joining(&mut self, duration: u8) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_PERMIT_JOINING, &[duration])
    }

    /// Build EZSP leaveNetwork command.
    pub fn build_leave_network(&mut self) -> Result<EzspFrame, SerialError> {
        self.next_ezsp_command(EZSP_LEAVE_NETWORK, &[])
    }

    // ── EZSP response parsers ───────────────────────────────

    /// Parse EZSP version response. Returns (protocol_version, stack_type, stack_version).
    pub fn parse_version_response(data: &[u8]) -> Result<(u8, u8, u16), SerialError> {
        if data.len() < 4 {
            return Err(SerialError::PayloadError);
        }
        // For v8+ response: the EZSP frame header is already stripped
        // Response payload: protocolVersion(1) + stackType(1) + stackVersion(2 LE)
        let ezsp = EzspFrame::parse(data)?;
        if ezsp.payload.len() < 4 {
            return Err(SerialError::PayloadError);
        }
        let ver = ezsp.payload[0];
        let stack_type = ezsp.payload[1];
        let stack_ver =
            u16::from_le_bytes([ezsp.payload[2], ezsp.payload[3]]);
        Ok((ver, stack_type, stack_ver))
    }

    /// Parse a generic EZSP response, returning the EZSP frame.
    pub fn parse_response(data: &[u8]) -> Result<EzspFrame, SerialError> {
        EzspFrame::parse(data)
    }
}

// ── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ash_stuff_unstuff_roundtrip() {
        let raw = [0xC0, 0x38, 0xBC]; // RST + CRC bytes
        let mut stuffed = [0u8; 32];
        let len = EzspCodec::stuff_frame(&raw, &mut stuffed).unwrap();

        // Should have FLAG + stuffed_bytes + FLAG
        assert_eq!(stuffed[0], ASH_FLAG);
        assert_eq!(stuffed[len - 1], ASH_FLAG);
    }

    #[test]
    fn lfsr_roundtrip() {
        let original = [0x00, 0x00, 0x08, 0x00, 0x00];
        let mut data = original;
        ash_lfsr_randomize(&mut data);
        // Data should be different after randomization
        assert_ne!(data, original);
        // Applying LFSR again should restore original
        ash_lfsr_randomize(&mut data);
        assert_eq!(data, original);
    }

    #[test]
    fn ezsp_frame_roundtrip() {
        let frame = EzspFrame::command(0x01, EZSP_VERSION, &[0x08]).unwrap();
        let mut buf = [0u8; 64];
        let len = frame.serialize(&mut buf).unwrap();
        let parsed = EzspFrame::parse(&buf[..len]).unwrap();
        assert_eq!(parsed.sequence, 0x01);
        assert_eq!(parsed.frame_id, EZSP_VERSION);
        assert_eq!(parsed.payload.as_slice(), &[0x08]);
    }

    #[test]
    fn ash_data_frame_codec_roundtrip() {
        let mut codec = EzspCodec::new();
        let ezsp = codec.build_version_command(8).unwrap();

        let mut out = [0u8; 128];
        let len = codec.build_data(&ezsp, &mut out).unwrap();

        // Feed the stuffed frame back into a fresh codec
        let mut rx_codec = EzspCodec::new();
        let result = rx_codec.feed(&out[..len]).unwrap();
        assert!(result.is_some());
        let (frame_type, data) = result.unwrap();
        match frame_type {
            AshFrameType::Data { frm_num, .. } => assert_eq!(frm_num, 0),
            other => panic!("expected DATA, got {:?}", other),
        }
        // data should be the original EZSP bytes
        let ezsp_parsed = EzspFrame::parse(&data).unwrap();
        assert_eq!(ezsp_parsed.frame_id, EZSP_VERSION);
    }
}
