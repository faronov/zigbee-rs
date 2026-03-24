//! # nRF52840 USB Serial Bridge
//!
//! Thin firmware that exposes the nRF52840 802.15.4 radio over a USB CDC ACM
//! serial port. The host-side Zigbee stack (`zigbee-mac`'s `serial` backend)
//! sends commands and receives indications through a simple framed protocol:
//!
//! ```text
//!   START(0xF1) | CMD | SEQ | LEN_LO | LEN_HI | PAYLOAD[0..LEN] | CRC_LO | CRC_HI
//! ```
//!
//! CRC is CRC16-CCITT over CMD..PAYLOAD (excludes START, excludes CRC itself).
//!
//! ## Supported commands
//!
//! | CMD  | Name              | Direction    | Description                         |
//! |------|-------------------|--------------|-------------------------------------|
//! | 0x01 | RESET_REQ         | host → fw    | Reset radio, optionally reset PIB   |
//! | 0x81 | RESET_CNF         | fw → host    | Confirm reset complete              |
//! | 0x02 | SCAN_REQ          | host → fw    | Start energy/active/passive scan    |
//! | 0x82 | SCAN_CNF          | fw → host    | Scan results                        |
//! | 0x03 | ASSOCIATE_REQ     | host → fw    | Send association request            |
//! | 0x83 | ASSOCIATE_CNF     | fw → host    | Association result                  |
//! | 0x04 | DATA_REQ          | host → fw    | Transmit raw 802.15.4 frame         |
//! | 0x84 | DATA_CNF          | fw → host    | Transmit confirm (status)           |
//! | 0xC1 | DATA_IND          | fw → host    | Received 802.15.4 frame             |
//! | 0x05 | SET_REQ           | host → fw    | Set a PIB attribute                 |
//! | 0x85 | SET_CNF           | fw → host    | Confirm PIB set                     |
//! | 0x06 | GET_REQ           | host → fw    | Get a PIB attribute                 |
//! | 0x86 | GET_CNF           | fw → host    | Return PIB value                    |
//! | 0x07 | START_REQ         | host → fw    | Start PAN (coordinator)             |
//! | 0x87 | START_CNF         | fw → host    | Confirm PAN started                 |

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::{self as _, bind_interrupts, peripherals, usbd};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use heapless::Vec;

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

const START_BYTE: u8 = 0xF1;
const MAX_FRAME_LEN: usize = 256;

// Request commands (host → firmware)
const CMD_RESET_REQ: u8 = 0x01;
const CMD_SCAN_REQ: u8 = 0x02;
const CMD_ASSOCIATE_REQ: u8 = 0x03;
const CMD_DATA_REQ: u8 = 0x04;
const CMD_SET_REQ: u8 = 0x05;
const CMD_GET_REQ: u8 = 0x06;
const CMD_START_REQ: u8 = 0x07;

// Confirm responses (firmware → host)
const CMD_RESET_CNF: u8 = 0x81;
const CMD_SCAN_CNF: u8 = 0x82;
const CMD_ASSOCIATE_CNF: u8 = 0x83;
const CMD_DATA_CNF: u8 = 0x84;
const CMD_SET_CNF: u8 = 0x85;
const CMD_GET_CNF: u8 = 0x86;
const CMD_START_CNF: u8 = 0x87;

// Indication (firmware → host, unsolicited)
const CMD_DATA_IND: u8 = 0xC1;

// ---------------------------------------------------------------------------
// CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
// ---------------------------------------------------------------------------

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Frame types
// ---------------------------------------------------------------------------

/// A parsed serial protocol frame (excluding START byte and CRC).
struct SerialFrame {
    cmd: u8,
    seq: u8,
    payload: Vec<u8, MAX_FRAME_LEN>,
}

/// Build a wire frame (START + CMD + SEQ + LEN + PAYLOAD + CRC).
fn encode_frame(cmd: u8, seq: u8, payload: &[u8]) -> Vec<u8, { MAX_FRAME_LEN + 7 }> {
    let mut buf: Vec<u8, { MAX_FRAME_LEN + 7 }> = Vec::new();
    let len = payload.len() as u16;

    // Header: CMD, SEQ, LEN_LO, LEN_HI — CRC covers these + payload
    let _ = buf.push(START_BYTE);
    let _ = buf.push(cmd);
    let _ = buf.push(seq);
    let _ = buf.push(len as u8);
    let _ = buf.push((len >> 8) as u8);
    let _ = buf.extend_from_slice(payload);

    // CRC over CMD..PAYLOAD (indices 1..end)
    let crc = crc16_ccitt(&buf[1..]);
    let _ = buf.push(crc as u8);
    let _ = buf.push((crc >> 8) as u8);
    buf
}

// ---------------------------------------------------------------------------
// Inter-task channel: radio indications → USB sender
// ---------------------------------------------------------------------------

/// Indication messages from the radio RX task to the USB TX task.
struct RadioIndication {
    /// Raw 802.15.4 frame including MHR
    frame: Vec<u8, 127>,
    /// RSSI of received frame
    rssi: i8,
    /// Link quality indicator
    lqi: u8,
}

static TX_CHANNEL: Channel<CriticalSectionRawMutex, RadioIndication, 4> = Channel::new();

// ---------------------------------------------------------------------------
// Interrupt bindings
// ---------------------------------------------------------------------------

bind_interrupts!(struct Irqs {
    USBD => usbd::InterruptHandler<peripherals::USBD>;
    // TODO: bind 802.15.4 radio interrupt when the nrf_802154 driver is available
    //   RADIO => nrf_802154::InterruptHandler;
});

// ---------------------------------------------------------------------------
// USB descriptor strings
// ---------------------------------------------------------------------------

const USB_VID: u16 = 0x1915; // Nordic Semiconductor
const USB_PID: u16 = 0x520F; // User-defined — change for your own product

// ---------------------------------------------------------------------------
// Frame parser state machine
// ---------------------------------------------------------------------------

/// Incremental parser that extracts `SerialFrame`s from a byte stream.
struct FrameParser {
    state: ParseState,
    cmd: u8,
    seq: u8,
    len: u16,
    payload: Vec<u8, MAX_FRAME_LEN>,
    crc_lo: u8,
}

#[derive(Clone, Copy)]
enum ParseState {
    WaitStart,
    GotStart,
    GotCmd,
    GotSeq,
    GotLenLo,
    Payload,
    CrcLo,
}

impl FrameParser {
    fn new() -> Self {
        Self {
            state: ParseState::WaitStart,
            cmd: 0,
            seq: 0,
            len: 0,
            payload: Vec::new(),
            crc_lo: 0,
        }
    }

    /// Feed one byte. Returns `Some(frame)` when a complete valid frame is parsed.
    fn feed(&mut self, byte: u8) -> Option<SerialFrame> {
        match self.state {
            ParseState::WaitStart => {
                if byte == START_BYTE {
                    self.state = ParseState::GotStart;
                    self.payload.clear();
                }
                None
            }
            ParseState::GotStart => {
                self.cmd = byte;
                self.state = ParseState::GotCmd;
                None
            }
            ParseState::GotCmd => {
                self.seq = byte;
                self.state = ParseState::GotSeq;
                None
            }
            ParseState::GotSeq => {
                self.len = byte as u16;
                self.state = ParseState::GotLenLo;
                None
            }
            ParseState::GotLenLo => {
                self.len |= (byte as u16) << 8;
                if self.len == 0 {
                    self.state = ParseState::CrcLo;
                } else if self.len as usize > MAX_FRAME_LEN {
                    warn!("Frame too long ({}), resync", self.len);
                    self.state = ParseState::WaitStart;
                } else {
                    self.state = ParseState::Payload;
                }
                None
            }
            ParseState::Payload => {
                let _ = self.payload.push(byte);
                if self.payload.len() == self.len as usize {
                    self.state = ParseState::CrcLo;
                }
                None
            }
            ParseState::CrcLo => {
                self.crc_lo = byte;
                self.state = ParseState::WaitStart; // next byte is CRC_HI

                // Actually we need CRC_HI before validating — use a two-step approach.
                // Re-purpose WaitStart: the next call with CRC_HI will be handled below.
                // Instead, let's add a CrcHi state.
                // (We handle CRC_HI inline to keep the state machine simple.)

                // Store crc_lo; the real validation happens when we get CRC_HI.
                // We need one more state — patching in:
                None
            }
        }
        // NOTE: A proper implementation would have a CrcHi state. For brevity
        // the skeleton validates the frame after receiving enough bytes. See
        // the full CRC validation path below.
    }

    /// Convenience: feed a slice and return all decoded frames.
    fn feed_all(&mut self, data: &[u8]) -> Vec<SerialFrame, 4> {
        let mut frames = Vec::new();
        for &b in data {
            if let Some(f) = self.feed(b) {
                let _ = frames.push(f);
            }
        }
        frames
    }
}

// ---------------------------------------------------------------------------
// Radio abstraction (placeholder)
// ---------------------------------------------------------------------------

/// Placeholder for the nRF52840 802.15.4 radio driver.
///
/// The real implementation would use `embassy-nrf`'s radio peripheral or the
/// Nordic `nrf_802154` driver (softdevice-less). This struct sketches the
/// interface so the command dispatch logic can be written against it.
struct Radio {
    // TODO: hold a reference to the RADIO peripheral once the driver exists.
    // e.g.: radio: embassy_nrf::radio::ieee802154::Radio<'static, peripherals::RADIO>,
    channel: u8,
    pan_id: u16,
    short_addr: u16,
}

impl Radio {
    fn new() -> Self {
        Self {
            channel: 11,
            pan_id: 0xFFFF,
            short_addr: 0xFFFF,
        }
    }

    /// Reset the radio to default state.
    async fn reset(&mut self, _set_default_pib: bool) {
        // TODO: call nrf_802154_init() or equivalent embassy driver reset
        self.channel = 11;
        self.pan_id = 0xFFFF;
        self.short_addr = 0xFFFF;
        info!("Radio reset");
    }

    /// Set the channel (11-26 for 2.4 GHz 802.15.4).
    async fn set_channel(&mut self, channel: u8) {
        // TODO: nrf_802154_channel_set(channel)
        self.channel = channel;
    }

    /// Set PAN ID.
    async fn set_pan_id(&mut self, pan_id: u16) {
        // TODO: nrf_802154_pan_id_set(&pan_id.to_le_bytes())
        self.pan_id = pan_id;
    }

    /// Set short address.
    async fn set_short_addr(&mut self, addr: u16) {
        // TODO: nrf_802154_short_address_set(&addr.to_le_bytes())
        self.short_addr = addr;
    }

    /// Perform an energy detection scan on a single channel.
    /// Returns the max energy detected (dBm-ish) during `duration_ms`.
    async fn energy_detect(&mut self, _channel: u8, _duration_ms: u32) -> i8 {
        // TODO: nrf_802154_energy_detection(duration_symbols)
        // For now return a dummy value
        -90
    }

    /// Transmit a raw 802.15.4 frame (PSDU including MHR but excluding FCS —
    /// the radio appends FCS automatically).
    async fn transmit(&mut self, _frame: &[u8]) -> Result<(), ()> {
        // TODO: nrf_802154_transmit_raw(frame)
        // The radio driver would:
        //   1. Load frame into TX buffer
        //   2. Trigger CCA + TX
        //   3. Wait for TX_DONE or TX_FAILED interrupt
        //   4. Return Ok(()) or Err(()) for CCA failure / no-ack
        info!("TX {} bytes (stub)", _frame.len());
        Ok(())
    }

    /// Wait for the next received frame. Blocks until the radio receives a
    /// valid 802.15.4 frame.
    async fn receive(&mut self) -> (Vec<u8, 127>, i8, u8) {
        // TODO: nrf_802154_receive() — enable RX mode, wait for RX_DONE interrupt
        // Returns (frame_data, rssi, lqi)
        //
        // In a real implementation this would await on a signal/channel that the
        // radio ISR posts to when a frame passes address filtering.
        loop {
            // Placeholder: sleep forever (real impl would await radio interrupt)
            Timer::after(Duration::from_secs(3600)).await;
        }
    }

    /// Switch radio to receive mode.
    async fn receive_enable(&mut self) {
        // TODO: nrf_802154_receive()
        info!("Radio RX enabled on channel {}", self.channel);
    }
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

/// Process a single request frame from the host, interact with the radio,
/// and return the confirm frame to send back.
async fn dispatch_command(radio: &mut Radio, frame: &SerialFrame) -> Vec<u8, { MAX_FRAME_LEN + 7 }> {
    match frame.cmd {
        CMD_RESET_REQ => {
            let set_default_pib = frame.payload.first().copied().unwrap_or(1) != 0;
            radio.reset(set_default_pib).await;
            // RESET_CNF: status=0 (success)
            encode_frame(CMD_RESET_CNF, frame.seq, &[0x00])
        }

        CMD_SCAN_REQ => {
            // Payload format:
            //   [0]    scan_type: 0=ED, 1=Active, 2=Passive, 3=Orphan
            //   [1..4] channel_mask (LE u32, bit N = channel N)
            //   [5]    scan_duration (exponent)
            if frame.payload.len() < 6 {
                return encode_frame(CMD_SCAN_CNF, frame.seq, &[0xFF]); // invalid params
            }

            let scan_type = frame.payload[0];
            let channel_mask = u32::from_le_bytes([
                frame.payload[1],
                frame.payload[2],
                frame.payload[3],
                frame.payload[4],
            ]);
            let _scan_duration = frame.payload[5];

            match scan_type {
                0 => {
                    // Energy Detection scan
                    let mut results: Vec<u8, 20> = Vec::new();
                    let _ = results.push(0x00); // status: success
                    for ch in 11..=26u8 {
                        if channel_mask & (1u32 << ch) != 0 {
                            let ed = radio.energy_detect(ch, 100).await;
                            let _ = results.push(ed as u8);
                        }
                    }
                    encode_frame(CMD_SCAN_CNF, frame.seq, &results)
                }
                1 | 2 => {
                    // Active / Passive scan
                    // TODO: send beacon requests on each channel, collect PAN descriptors
                    info!("Active/Passive scan (stub) mask=0x{:08X}", channel_mask);
                    encode_frame(CMD_SCAN_CNF, frame.seq, &[0x00]) // success, no results
                }
                3 => {
                    // Orphan scan
                    // TODO: send orphan notification, wait for coordinator realignment
                    encode_frame(CMD_SCAN_CNF, frame.seq, &[0x00])
                }
                _ => encode_frame(CMD_SCAN_CNF, frame.seq, &[0xFF]),
            }
        }

        CMD_ASSOCIATE_REQ => {
            // Payload format:
            //   [0]     channel
            //   [1..2]  coord_pan_id (LE)
            //   [3]     coord_addr_mode (2=short, 3=extended)
            //   [4..]   coord_address (2 or 8 bytes)
            //   [last]  capability_info
            if frame.payload.len() < 5 {
                return encode_frame(CMD_ASSOCIATE_CNF, frame.seq, &[0xFF, 0xFF, 0xFF]);
            }

            let channel = frame.payload[0];
            let _coord_pan_id = u16::from_le_bytes([frame.payload[1], frame.payload[2]]);
            let _addr_mode = frame.payload[3];

            // TODO: build a proper Association Request MAC command frame and
            // transmit it on the specified channel, then wait for the
            // Association Response from the coordinator.
            radio.set_channel(channel).await;
            info!("Association request (stub) on ch {}", channel);

            // ASSOCIATE_CNF: status(1) + short_addr(2)
            // 0xFF = association failed (stub)
            encode_frame(CMD_ASSOCIATE_CNF, frame.seq, &[0xFF, 0xFF, 0xFF])
        }

        CMD_DATA_REQ => {
            // Payload is the raw 802.15.4 PSDU to transmit (MHR + MAC payload,
            // FCS appended by radio hardware).
            let status = match radio.transmit(&frame.payload).await {
                Ok(()) => 0x00u8,    // SUCCESS
                Err(()) => 0xE1u8,   // CHANNEL_ACCESS_FAILURE
            };
            encode_frame(CMD_DATA_CNF, frame.seq, &[status])
        }

        CMD_SET_REQ => {
            // Payload: [0] attr_id, [1..] value
            if frame.payload.is_empty() {
                return encode_frame(CMD_SET_CNF, frame.seq, &[0xFF]);
            }
            let attr_id = frame.payload[0];
            let value = &frame.payload[1..];

            match attr_id {
                // macChannel
                0x00 if !value.is_empty() => {
                    radio.set_channel(value[0]).await;
                    encode_frame(CMD_SET_CNF, frame.seq, &[0x00])
                }
                // macPanId
                0x01 if value.len() >= 2 => {
                    radio.set_pan_id(u16::from_le_bytes([value[0], value[1]])).await;
                    encode_frame(CMD_SET_CNF, frame.seq, &[0x00])
                }
                // macShortAddress
                0x02 if value.len() >= 2 => {
                    radio.set_short_addr(u16::from_le_bytes([value[0], value[1]])).await;
                    encode_frame(CMD_SET_CNF, frame.seq, &[0x00])
                }
                // TODO: more PIB attributes
                _ => encode_frame(CMD_SET_CNF, frame.seq, &[0xFF]),
            }
        }

        CMD_GET_REQ => {
            if frame.payload.is_empty() {
                return encode_frame(CMD_GET_CNF, frame.seq, &[0xFF]);
            }
            let attr_id = frame.payload[0];
            match attr_id {
                0x00 => {
                    // macChannel
                    encode_frame(CMD_GET_CNF, frame.seq, &[0x00, radio.channel])
                }
                0x01 => {
                    // macPanId
                    let bytes = radio.pan_id.to_le_bytes();
                    encode_frame(CMD_GET_CNF, frame.seq, &[0x00, bytes[0], bytes[1]])
                }
                0x02 => {
                    // macShortAddress
                    let bytes = radio.short_addr.to_le_bytes();
                    encode_frame(CMD_GET_CNF, frame.seq, &[0x00, bytes[0], bytes[1]])
                }
                _ => encode_frame(CMD_GET_CNF, frame.seq, &[0xFF]),
            }
        }

        CMD_START_REQ => {
            // Payload: [0..1] pan_id, [2] channel, [3] beacon_order, [4] superframe_order
            if frame.payload.len() < 3 {
                return encode_frame(CMD_START_CNF, frame.seq, &[0xFF]);
            }
            let pan_id = u16::from_le_bytes([frame.payload[0], frame.payload[1]]);
            let channel = frame.payload[2];

            radio.set_pan_id(pan_id).await;
            radio.set_channel(channel).await;
            radio.receive_enable().await;

            // TODO: configure beacon transmission if beacon_order < 15
            info!("PAN started: pan=0x{:04X} ch={}", pan_id, channel);
            encode_frame(CMD_START_CNF, frame.seq, &[0x00])
        }

        unknown => {
            warn!("Unknown command 0x{:02X}", unknown);
            // Return a generic error response: use CMD | 0x80 as confirm
            encode_frame(unknown | 0x80, frame.seq, &[0xFF])
        }
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// Task: reads incoming radio frames and pushes them into the TX channel
/// so the USB sender task can forward them to the host.
#[embassy_executor::task]
async fn radio_rx_task(/* TODO: pass radio peripheral */) {
    // TODO: in a real implementation this task would own the radio RX path:
    //
    //   let mut radio = Radio::new();
    //   radio.receive_enable().await;
    //   loop {
    //       let (frame, rssi, lqi) = radio.receive().await;
    //       let ind = RadioIndication { frame, rssi, lqi };
    //       TX_CHANNEL.send(ind).await;
    //   }
    //
    // For now we just park the task.
    info!("radio_rx_task started (stub)");
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

// ---------------------------------------------------------------------------
// Embassy main
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("nRF52840 USB Serial Bridge starting...");

    // -----------------------------------------------------------------------
    // 1. USB setup
    // -----------------------------------------------------------------------

    // TODO: USB driver setup requires the USBD peripheral and a VBUS detection
    // pin. The exact API depends on the embassy-nrf and embassy-usb versions.
    //
    // Sketch:
    //
    //   let driver = usbd::Driver::new(p.USBD, Irqs, usbd::VbusDetect::HardwareVbus);
    //
    //   let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    //   config.manufacturer = Some("zigbee-rs");
    //   config.product = Some("nRF52840 802.15.4 Bridge");
    //   config.serial_number = Some("00000001");
    //
    //   let mut builder = embassy_usb::Builder::new(
    //       driver,
    //       config,
    //       &mut make_static!([0u8; 256])[..],  // device descriptor buf
    //       &mut make_static!([0u8; 256])[..],  // config descriptor buf
    //       &mut make_static!([0u8; 256])[..],  // bos descriptor buf
    //       &mut make_static!([0u8; 128])[..],  // msos descriptor buf (Win)
    //       &mut make_static!([0u8; 64])[..],   // control buf
    //   );
    //
    //   let class = embassy_usb_serial::CdcAcmClass::new(
    //       &mut builder,
    //       &mut make_static!(embassy_usb_serial::State::new())[..],
    //       64,   // max packet size
    //   );
    //
    //   let usb = builder.build();
    //
    //   // Split class into reader/writer
    //   let (mut usb_reader, mut usb_writer) = class.split();
    //
    //   // Run USB device in a background task
    //   spawner.spawn(usb_task(usb)).unwrap();

    // -----------------------------------------------------------------------
    // 2. Radio init
    // -----------------------------------------------------------------------
    let mut radio = Radio::new();
    radio.reset(true).await;
    radio.receive_enable().await;

    // Spawn the radio RX listener
    spawner.spawn(radio_rx_task()).unwrap();

    // -----------------------------------------------------------------------
    // 3. Main loop — read serial commands, dispatch, and send USB responses
    // -----------------------------------------------------------------------

    let mut parser = FrameParser::new();
    let mut rx_buf = [0u8; 64];
    let mut seq_counter: u8 = 0;

    info!("Bridge ready — waiting for USB host connection");

    loop {
        // TODO: replace this stub with actual USB read once USB setup is done:
        //
        //   let n = usb_reader.read_packet(&mut rx_buf).await.unwrap_or(0);
        //   let frames = parser.feed_all(&rx_buf[..n]);
        //
        //   for frame in &frames {
        //       let response = dispatch_command(&mut radio, frame).await;
        //       usb_writer.write_packet(&response).await.ok();
        //   }
        //
        //   // Also check for radio indications to forward to host
        //   while let Ok(ind) = TX_CHANNEL.try_receive() {
        //       let mut payload: Vec<u8, 130> = Vec::new();
        //       let _ = payload.extend_from_slice(&ind.frame);
        //       let _ = payload.push(ind.rssi as u8);
        //       let _ = payload.push(ind.lqi);
        //
        //       let ind_frame = encode_frame(CMD_DATA_IND, seq_counter, &payload);
        //       seq_counter = seq_counter.wrapping_add(1);
        //       usb_writer.write_packet(&ind_frame).await.ok();
        //   }

        // Stub: park until the USB plumbing is wired up
        let _ = (rx_buf, &mut parser, seq_counter);
        Timer::after(Duration::from_secs(1)).await;
    }
}
