//! Telink MAC backend.
//!
//! Implements `MacDriver` for Telink TLSR825x / B91 / TL721x / TL321x SoCs.
//! These chips have a built-in IEEE 802.15.4 radio suitable for Zigbee.
//!
//! This backend uses the Telink radio driver for radio access, with async
//! TX/RX through Embassy signals.
//!
//! # Architecture
//! ```text
//! MacDriver trait methods
//!        │
//!        ▼
//! TelinkMac (this module)
//!   ├── PIB state (addresses, channel, config)
//!   ├── Frame construction (beacon req, assoc req, data)
//!   └── TelinkDriver (driver.rs)
//!          ├── Radio register access (Telink HAL)
//!          ├── TX via Signal (interrupt-driven)
//!          └── RX via Signal (interrupt-driven)
//! ```
//!
//! # Hardware
//! - TLSR8258 (2.4 GHz IEEE 802.15.4 + BLE)
//! - B91 / TLSR9218 (multi-protocol)
//! - TL721x / TL321x (next-gen Telink Zigbee SoCs)

pub mod driver;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::{RadioConfig, TelinkDriver};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

// ── IEEE 802.15.4 MAC timing constants (2.4 GHz) ──────────────

/// Symbol period in microseconds (62.5 ksym/s at 2.4 GHz).
const SYMBOL_PERIOD_US: u64 = 16;

/// Unit backoff period in symbols (aUnitBackoffPeriod).
const UNIT_BACKOFF_SYMBOLS: u64 = 20;

/// ACK wait timeout in microseconds.
/// aTurnaroundTime (192µs) + ACK duration (~352µs) + software margin.
const ACK_WAIT_US: u64 = 1500;

/// Association response wait timeout in milliseconds.
const ASSOC_RESPONSE_WAIT_MS: u64 = 3000;

/// RX indication timeout in milliseconds.
const RX_INDICATION_TIMEOUT_MS: u64 = 5000;

/// Poll response wait timeout in milliseconds.
const POLL_RESPONSE_WAIT_MS: u64 = 500;

/// Maximum number of frames in the indirect TX queue (for coordinator role).
const INDIRECT_QUEUE_SIZE: usize = 8;

/// Persistence timeout for indirect frames (in ticks / calls to age_indirect_queue).
/// IEEE 802.15.4: macTransactionPersistenceTime × aBaseSuperframeDuration.
const INDIRECT_PERSISTENCE_TICKS: u8 = 30;

/// Maximum MAC overhead for a 2.4 GHz frame:
/// FC(2) + Seq(1) + DstPAN(2) + DstAddr(2|8) + SrcPAN(0|2) + SrcAddr(2|8) + FCS(2)
/// Worst case with extended addressing: 2+1+2+8+2+8+2 = 25
const MAX_MAC_OVERHEAD: usize = 25;

/// Entry in the indirect TX queue (buffered frame for sleepy child).
struct IndirectEntry {
    dst: MacAddress,
    frame: heapless::Vec<u8, 128>,
    remaining_ticks: u8,
}

/// Telink 802.15.4 MAC driver.
///
/// Built on Telink radio HAL — uses the Telink SoC's hardware radio with
/// interrupt-driven TX/RX via Embassy signals.
///
/// # Usage
/// ```rust,no_run
/// use zigbee_mac::telink::TelinkMac;
///
/// // After initializing Telink clocks and enabling radio peripheral:
/// let mac = TelinkMac::new();
/// let device = zigbee_runtime::builder(mac)
///     .device_type(zigbee_nwk::DeviceType::EndDevice)
///     .build();
/// ```
pub struct TelinkMac {
    driver: TelinkDriver,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    coord_short_address: ShortAddress,
    coord_extended_address: IeeeAddress,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    pan_coordinator: bool,
    beacon_order: u8,
    superframe_order: u8,
    response_wait_time: u8,
    dsn: u8,
    bsn: u8,
    beacon_payload: PibPayload,
    max_csma_backoffs: u8,
    min_be: u8,
    max_be: u8,
    max_frame_retries: u8,
    promiscuous: bool,
    tx_power: i8,
    // Indirect TX queue (coordinator buffers frames for sleepy children)
    indirect_queue: heapless::Vec<IndirectEntry, INDIRECT_QUEUE_SIZE>,
}

impl TelinkMac {
    /// Create a new Telink MAC driver with default PIB values.
    ///
    /// The caller must have already:
    /// 1. Enabled Telink radio peripheral clocks
    /// 2. Configured the radio interrupt to call the driver callbacks
    pub fn new() -> Self {
        Self {
            driver: TelinkDriver::new(RadioConfig::default()),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
            coord_short_address: ShortAddress(0x0000),
            coord_extended_address: [0u8; 8],
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            pan_coordinator: false,
            beacon_order: 15, // 15 = non-beacon mode
            superframe_order: 15,
            response_wait_time: 32, // 32 × aBaseSuperframeDuration ≈ 491ms
            dsn: 0,
            bsn: 0,
            beacon_payload: PibPayload::new(),
            max_csma_backoffs: 4,
            min_be: 3,
            max_be: 5,
            max_frame_retries: 3,
            promiscuous: false,
            tx_power: 0,
            indirect_queue: heapless::Vec::new(),
        }
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    /// Power down the radio to save battery between poll cycles.
    /// Saves ~5-8 mA. Call `radio_wake()` before next TX/RX.
    pub fn radio_sleep(&self) {
        self.driver.radio_sleep();
    }

    /// Re-enable the radio after `radio_sleep()`.
    pub fn radio_wake(&mut self) {
        self.driver.radio_wake();
    }

    /// Enter CPU suspend mode for the given duration.
    ///
    /// Halts the CPU with SRAM retained (~3 µA). Resumes after timer fires.
    /// Call `radio_sleep()` before this and `radio_wake()` after.
    pub fn cpu_suspend_ms(duration_ms: u32) {
        TelinkDriver::cpu_suspend_ms(duration_ms);
    }

    #[allow(dead_code)]
    fn next_bsn(&mut self) -> u8 {
        let seq = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        seq
    }

    /// Construct an IEEE 802.15.4 Beacon Request MAC command frame.
    fn beacon_request_frame(&mut self) -> [u8; 8] {
        let seq = self.next_dsn();
        // Frame Control: MAC command (0x03), no security, no frame pending,
        // no ack request, no PAN ID compression, dst=short, src=none
        [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
    }

    /// Construct an IEEE 802.15.4 Association Request MAC command frame.
    fn association_request_frame(
        &mut self,
        coord_address: &MacAddress,
        capability_info: &CapabilityInfo,
    ) -> heapless::Vec<u8, 32> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: MAC command (type=3), no security, no pending,
        // ack requested (bit 5), PAN ID compression (bit 6), dst=short, src=extended
        // 0x63 = 0b_0110_0011 (low byte), 0xC8 = 0b_1100_1000 (high byte)
        let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);

        // Destination PAN + address
        let dst_pan = coord_address.pan_id();
        let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());

        match coord_address {
            MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }

        // Source: extended address (our IEEE address)
        let _ = frame.extend_from_slice(&self.extended_address);

        // MAC command: Association Request (0x01)
        let _ = frame.push(0x01);

        // Capability Information byte
        let _ = frame.push(capability_info.to_byte());

        frame
    }

    /// Scan a single channel for beacons (active scan).
    async fn scan_channel_active(
        &mut self,
        channel: u8,
        duration_ms: u32,
    ) -> heapless::Vec<PanDescriptor, 8> {
        let mut results = heapless::Vec::new();

        // Switch to target channel
        self.driver.update_config(|cfg| cfg.channel = channel);

        // Send beacon request
        let beacon_req = self.beacon_request_frame();
        if self.driver.transmit(&beacon_req).await.is_err() {
            return results;
        }

        // Collect beacons for scan duration
        let deadline = Timer::after(embassy_time::Duration::from_millis(duration_ms as u64));

        let collect = async {
            loop {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        if let Some(desc) = Self::parse_beacon(
                            &rx_frame.data[..rx_frame.len],
                            rx_frame.lqi,
                            channel,
                        ) {
                            let _ = results.push(desc);
                        }
                    }
                    Err(_) => continue, // ignore RX errors, keep scanning
                }
            }
        };

        let _ = select::select(deadline, collect).await;
        results
    }

    /// Scan a single channel for beacons (passive scan — listen only, no TX).
    async fn scan_channel_passive(
        &mut self,
        channel: u8,
        duration_ms: u32,
    ) -> heapless::Vec<PanDescriptor, 8> {
        let mut results = heapless::Vec::new();

        self.driver.update_config(|cfg| cfg.channel = channel);
        self.driver.enable_rx();

        let deadline = Timer::after(embassy_time::Duration::from_millis(duration_ms as u64));

        let collect = async {
            loop {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        if let Some(desc) = Self::parse_beacon(
                            &rx_frame.data[..rx_frame.len],
                            rx_frame.lqi,
                            channel,
                        ) {
                            let _ = results.push(desc);
                        }
                    }
                    Err(_) => continue, // ignore RX errors, keep listening
                }
            }
        };

        let _ = select::select(deadline, collect).await;
        results
    }

    /// Scan a single channel for energy (ED scan).
    async fn scan_channel_ed(&mut self, channel: u8) -> u8 {
        self.driver.update_config(|cfg| cfg.channel = channel);

        match self.driver.energy_detect() {
            Ok(ed) => ed,
            Err(_) => 0,
        }
    }

    /// Try to parse a received frame as a beacon and extract a PAN descriptor.
    fn parse_beacon(frame_data: &[u8], lqi: u8, channel: u8) -> Option<PanDescriptor> {
        if frame_data.len() < 5 {
            return None;
        }

        let fc = u16::from_le_bytes([frame_data[0], frame_data[1]]);
        let frame_type = fc & 0x07;

        // Must be a beacon frame (type 0)
        if frame_type != 0 {
            return None;
        }

        // Superframe spec position depends on addressing
        let superframe_offset = 3 + addressing_size(fc);
        if frame_data.len() < superframe_offset + 2 {
            return None;
        }

        let sf_raw = u16::from_le_bytes([
            frame_data[superframe_offset],
            frame_data[superframe_offset + 1],
        ]);
        let superframe_spec = SuperframeSpec::from_raw(sf_raw);

        // Zigbee beacon payload follows superframe + GTS(1) + pending(1)
        let beacon_payload_offset = superframe_offset + 4;
        if frame_data.len() < beacon_payload_offset + 15 {
            return None;
        }

        let zigbee_beacon = parse_zigbee_beacon(&frame_data[beacon_payload_offset..]);
        let coord_address = parse_source_address(frame_data, fc)?;

        Some(PanDescriptor {
            channel,
            coord_address,
            superframe_spec,
            lqi,
            security_use: (fc >> 3) & 1 != 0,
            zigbee_beacon,
        })
    }

    /// Build a raw 802.15.4 data frame for transmission.
    fn build_data_frame(
        &mut self,
        dst_address: &MacAddress,
        payload: &[u8],
        ack_request: bool,
    ) -> heapless::Vec<u8, 127> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: Data frame
        let mut fc: u16 = 0x0001; // Frame type = data
        if ack_request {
            fc |= 0x0020; // ACK request
        }

        // PAN ID compression
        fc |= 0x0040;

        // Destination addressing mode
        match dst_address {
            MacAddress::Short(_, _) => fc |= 0x0800,    // Short address
            MacAddress::Extended(_, _) => fc |= 0x0C00, // Extended address
        }

        // Source addressing mode: short (if we have one) or extended
        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            fc |= 0x8000; // Short address
        } else {
            fc |= 0xC000; // Extended address
        }

        let _ = frame.extend_from_slice(&fc.to_le_bytes());
        let _ = frame.push(seq);

        // Destination PAN + address
        let dst_pan = dst_address.pan_id();
        let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());

        match dst_address {
            MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }

        // Source address
        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            let _ = frame.extend_from_slice(&self.short_address.0.to_le_bytes());
        } else {
            let _ = frame.extend_from_slice(&self.extended_address);
        }

        // Payload
        let _ = frame.extend_from_slice(payload);

        frame
    }

    /// Simple PRNG for CSMA-CA random backoff (LCG).
    fn prng(seed: u8) -> u32 {
        (seed as u32).wrapping_mul(1103515245).wrapping_add(12345)
    }

    /// Unslotted CSMA-CA + TX + ACK wait + retries.
    ///
    /// Implements IEEE 802.15.4-2011 §5.1.1.4 (unslotted CSMA-CA) with
    /// optional ACK reception and retry loop per `macMaxFrameRetries`.
    /// Reference: Telink ESL SDK `mac_csmaDelayCb` → `rf_performCCA` → `mac_doTx`.
    async fn csma_ca_transmit(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
    ) -> Result<(), MacError> {
        let max_retries = if ack_requested {
            self.max_frame_retries
        } else {
            0
        };

        for attempt in 0..=max_retries {
            // ── Unslotted CSMA-CA ──
            let mut nb: u8 = 0;
            let mut be = self.min_be;

            let channel_clear = loop {
                // Random backoff: 0 to (2^BE - 1) unit backoff periods
                let max_val = (1u32 << be) - 1;
                let seed = self.dsn.wrapping_add(nb).wrapping_add(attempt);
                let random = Self::prng(seed);
                let backoff = (random % (max_val + 1)) as u64;
                let delay_us = backoff * UNIT_BACKOFF_SYMBOLS * SYMBOL_PERIOD_US;
                if delay_us > 0 {
                    Timer::after_micros(delay_us).await;
                }

                // CCA — use rf_performCCA via driver
                match self.driver.cca() {
                    Ok(idle) if idle => break true,
                    Ok(_) => {
                        // Channel busy — backoff and retry
                        nb += 1;
                        be = core::cmp::min(be + 1, self.max_be);
                        if nb > self.max_csma_backoffs {
                            break false;
                        }
                    }
                    Err(_) => break false,
                }
            };

            if !channel_clear {
                if attempt == max_retries {
                    return Err(MacError::ChannelAccessFailure);
                }
                continue;
            }

            // ── TX ──
            self.driver
                .transmit(frame)
                .await
                .map_err(|_| MacError::RadioError)?;

            if !ack_requested {
                return Ok(());
            }

            // ── ACK wait ──
            // Spec: aTurnaroundTime (192µs) + ACK frame duration (~352µs).
            // Allow 1.5ms total for software overhead.
            let seq = frame[2];
            let ack_result =
                select::select(self.driver.receive(), Timer::after_micros(ACK_WAIT_US)).await;

            if let select::Either::First(Ok(rx)) = ack_result {
                if rx.len >= 3 {
                    let fc = u16::from_le_bytes([rx.data[0], rx.data[1]]);
                    let frame_type = fc & 0x07;
                    let ack_seq = rx.data[2];
                    if frame_type == 0x02 && ack_seq == seq {
                        return Ok(());
                    }
                }
            }

            // No valid ACK — retry if attempts remain
            if attempt == max_retries {
                return Err(MacError::NoAck);
            }
            log::debug!(
                "telink: no ACK for seq={}, retry {}/{}",
                seq,
                attempt + 1,
                max_retries
            );
        }

        Err(MacError::NoAck)
    }

    /// Send a 3-byte IEEE 802.15.4 ACK frame for the given sequence number.
    /// If `frame_pending` is true, set the frame pending bit (bit 4 of FC).
    async fn send_ack(&mut self, seq: u8, frame_pending: bool) {
        let fc_low = if frame_pending { 0x12u8 } else { 0x02u8 };
        let ack = [fc_low, 0x00, seq];
        let _ = self.driver.transmit(&ack).await;
    }

    /// Sync radio hardware config with current PIB state.
    fn sync_radio_config(&mut self) {
        self.driver.update_config(|cfg| {
            cfg.channel = self.channel;
            cfg.pan_id = self.pan_id.0;
            cfg.short_address = self.short_address.0;
            cfg.extended_address = self.extended_address;
            cfg.tx_power = self.tx_power;
            cfg.promiscuous = self.promiscuous;
        });
    }
}

// ── MacDriver implementation ────────────────────────────────────

impl MacDriver for TelinkMac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let scan_duration_ms = ((1u32 << req.scan_duration) + 1) * 15; // ~aBaseSuperframeDuration

        match req.scan_type {
            ScanType::Active => {
                let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = zigbee_types::Channel::from_number(ch) {
                        if req.channel_mask.contains(channel) {
                            let beacons = self.scan_channel_active(ch, scan_duration_ms).await;
                            for desc in beacons {
                                let _ = pan_descriptors.push(desc);
                            }
                        }
                    }
                }

                self.sync_radio_config();

                if pan_descriptors.is_empty() {
                    Err(MacError::NoBeacon)
                } else {
                    Ok(MlmeScanConfirm {
                        scan_type: req.scan_type,
                        pan_descriptors,
                        energy_list: heapless::Vec::new(),
                    })
                }
            }
            ScanType::Passive => {
                let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = zigbee_types::Channel::from_number(ch) {
                        if req.channel_mask.contains(channel) {
                            let beacons = self.scan_channel_passive(ch, scan_duration_ms).await;
                            for desc in beacons {
                                let _ = pan_descriptors.push(desc);
                            }
                        }
                    }
                }

                self.sync_radio_config();

                if pan_descriptors.is_empty() {
                    Err(MacError::NoBeacon)
                } else {
                    Ok(MlmeScanConfirm {
                        scan_type: req.scan_type,
                        pan_descriptors,
                        energy_list: heapless::Vec::new(),
                    })
                }
            }
            ScanType::Ed => {
                let mut energy_list: EdList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = zigbee_types::Channel::from_number(ch) {
                        if req.channel_mask.contains(channel) {
                            let energy = self.scan_channel_ed(ch).await;
                            let _ = energy_list.push(EdValue {
                                channel: ch,
                                energy,
                            });
                        }
                    }
                }

                self.sync_radio_config();

                Ok(MlmeScanConfirm {
                    scan_type: req.scan_type,
                    pan_descriptors: heapless::Vec::new(),
                    energy_list,
                })
            }
            ScanType::Orphan => {
                // Orphan scan: no specific action at MAC level for end devices.
                // The coordinator role would send orphan notification — not yet supported.
                self.sync_radio_config();

                Ok(MlmeScanConfirm {
                    scan_type: req.scan_type,
                    pan_descriptors: heapless::Vec::new(),
                    energy_list: heapless::Vec::new(),
                })
            }
        }
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        // Switch to coordinator's channel and PAN
        self.channel = req.channel;
        self.pan_id = req.coord_address.pan_id();
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = req.coord_address.pan_id().0;
        });

        log::info!(
            "[Telink MLME-ASSOC] Associating on ch {} with {:?}",
            req.channel,
            req.coord_address
        );

        // Build and send association request (ACK requested)
        let frame = self.association_request_frame(&req.coord_address, &req.capability_info);
        self.csma_ca_transmit(&frame, true).await?;

        // Per IEEE 802.15.4 §5.3.2.1: wait, then poll with Data Request
        Timer::after_millis(100).await;

        // Send Data Request to poll for indirect Association Response (ACK requested)
        let data_req =
            build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
        self.csma_ca_transmit(&data_req, true).await?;

        // Wait for Association Response
        let timeout = Timer::after(embassy_time::Duration::from_millis(ASSOC_RESPONSE_WAIT_MS));

        let wait_response = async {
            for _ in 0..10 {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        let data = &rx_frame.data[..rx_frame.len];
                        if data.len() < 5 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        // Must be a MAC command frame (type 3)
                        if fc & 0x07 != 3 {
                            continue;
                        }
                        let cmd_offset = 3 + addressing_size(fc);
                        if data.len() < cmd_offset + 4 {
                            continue;
                        }
                        // Association Response = command ID 0x02
                        if data[cmd_offset] == 0x02 {
                            // Send ACK if requested
                            if (fc & 0x0020) != 0 {
                                self.send_ack(data[2], false).await;
                            }
                            let short_addr =
                                u16::from_le_bytes([data[cmd_offset + 1], data[cmd_offset + 2]]);
                            let status = match data[cmd_offset + 3] {
                                0x00 => AssociationStatus::Success,
                                0x01 => AssociationStatus::PanAtCapacity,
                                _ => AssociationStatus::PanAccessDenied,
                            };
                            if status == AssociationStatus::Success {
                                self.short_address = ShortAddress(short_addr);
                                self.sync_radio_config();
                            }
                            return Ok(MlmeAssociateConfirm {
                                short_address: ShortAddress(short_addr),
                                status,
                            });
                        }
                    }
                    Err(_) => return Err(MacError::RadioError),
                }
            }
            Err(MacError::NoAck)
        };

        match select::select(timeout, wait_response).await {
            select::Either::First(_) => Err(MacError::NoAck),
            select::Either::Second(result) => result,
        }
    }

    async fn mlme_associate_response(
        &mut self,
        rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        // Build Association Response MAC command frame (IEEE 802.15.4 §5.3.2)
        let mut frame: heapless::Vec<u8, 32> = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: MAC command, ACK req, PAN ID compression, dst=extended, src=extended
        let _ = frame.extend_from_slice(&[0x63, 0xCC, seq]);

        // Destination PAN + extended address of requesting device
        let _ = frame.extend_from_slice(&self.pan_id.0.to_le_bytes());
        let _ = frame.extend_from_slice(&rsp.device_address);

        // Source: our extended address (PAN ID compressed)
        let _ = frame.extend_from_slice(&self.extended_address);

        // MAC command: Association Response (0x02)
        let _ = frame.push(0x02);
        // Assigned short address (LE)
        let _ = frame.extend_from_slice(&rsp.short_address.0.to_le_bytes());
        // Association status
        let _ = frame.push(rsp.status as u8);

        self.csma_ca_transmit(&frame, true).await?;
        Ok(())
    }

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        // Build Disassociation Notification MAC command frame (IEEE 802.15.4 §5.3.3)
        let mut frame: heapless::Vec<u8, 32> = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: MAC command, ACK request, PAN ID compression, dst=short/ext, src=extended
        match &req.device_address {
            MacAddress::Short(_, _) => {
                let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
            }
            MacAddress::Extended(_, _) => {
                let _ = frame.extend_from_slice(&[0x63, 0xCC, seq]);
            }
        }

        // Destination PAN + address
        let dst_pan = req.device_address.pan_id();
        let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
        match &req.device_address {
            MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }

        // Source: our extended address
        let _ = frame.extend_from_slice(&self.extended_address);

        // MAC command: Disassociation Notification (0x03) + reason
        let _ = frame.push(0x03);
        let _ = frame.push(req.reason as u8);

        // Send via CSMA-CA (ACK requested)
        let _ = self.csma_ca_transmit(&frame, true).await;

        // Clear local addressing state regardless of TX result
        self.short_address = ShortAddress(0xFFFF);
        self.pan_id = PanId(0xFFFF);
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.rx_on_when_idle = false;
            self.association_permit = false;
            self.auto_request = true;
            self.dsn = 0;
            self.bsn = 0;
            self.beacon_payload = PibPayload::new();
            self.max_csma_backoffs = 4;
            self.min_be = 3;
            self.max_be = 5;
            self.max_frame_retries = 3;
            self.promiscuous = false;
            self.tx_power = 0;
            self.pan_coordinator = false;
        }

        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        self.pan_coordinator = req.pan_coordinator;

        // Coordinators must be rx-on-when-idle to receive association requests
        if req.pan_coordinator {
            self.rx_on_when_idle = true;
            self.association_permit = true;
        }

        self.sync_radio_config();

        // Enable receiver if rx_on_when_idle
        if self.rx_on_when_idle {
            self.driver.enable_rx();
        }

        log::info!(
            "telink: started PAN 0x{:04X} on channel {} (coordinator={})",
            self.pan_id.0,
            self.channel,
            self.pan_coordinator,
        );
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        use PibAttribute::*;

        match attr {
            MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            MacPanId => Ok(PibValue::PanId(self.pan_id)),
            MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.extended_address)),
            MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            MacDsn => Ok(PibValue::U8(self.dsn)),
            MacBsn => Ok(PibValue::U8(self.bsn)),
            MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            MacMinBe => Ok(PibValue::U8(self.min_be)),
            MacMaxBe => Ok(PibValue::U8(self.max_be)),
            MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PhyTransmitPower => Ok(PibValue::U8(self.tx_power as u8)),
            PhyChannelsSupported => Ok(PibValue::U32(zigbee_types::ChannelMask::ALL_2_4GHZ.0)),
            PhyCurrentPage => Ok(PibValue::U8(0)), // Page 0 = 2.4 GHz
            MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            MacCoordShortAddress => Ok(PibValue::ShortAddress(self.coord_short_address)),
            MacCoordExtendedAddress => Ok(PibValue::ExtendedAddress(self.coord_extended_address)),
            MacBeaconOrder => Ok(PibValue::U8(self.beacon_order)),
            MacSuperframeOrder => Ok(PibValue::U8(self.superframe_order)),
            MacResponseWaitTime => Ok(PibValue::U8(self.response_wait_time)),
            _ => Err(MacError::Unsupported),
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        use PibAttribute::*;

        match (attr, value) {
            (MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = v;
                self.sync_radio_config();
            }
            (MacPanId, PibValue::PanId(v)) => {
                self.pan_id = v;
                self.sync_radio_config();
            }
            (MacExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.extended_address = v;
                self.sync_radio_config();
            }
            (MacRxOnWhenIdle, PibValue::Bool(v)) => {
                self.rx_on_when_idle = v;
                if v {
                    self.driver.enable_rx();
                } else {
                    self.driver.disable_rx();
                }
            }
            (MacAssociationPermit, PibValue::Bool(v)) => self.association_permit = v,
            (MacAutoRequest, PibValue::Bool(v)) => self.auto_request = v,
            (MacDsn, PibValue::U8(v)) => self.dsn = v,
            (MacBsn, PibValue::U8(v)) => self.bsn = v,
            (MacMaxCsmaBackoffs, PibValue::U8(v)) => self.max_csma_backoffs = v,
            (MacMinBe, PibValue::U8(v)) => self.min_be = v,
            (MacMaxBe, PibValue::U8(v)) => self.max_be = v,
            (MacMaxFrameRetries, PibValue::U8(v)) => self.max_frame_retries = v,
            (MacPromiscuousMode, PibValue::Bool(v)) => {
                self.promiscuous = v;
                self.sync_radio_config();
            }
            (PhyCurrentChannel, PibValue::U8(v)) => {
                self.channel = v;
                self.sync_radio_config();
            }
            (PhyTransmitPower, PibValue::U8(v)) => {
                self.tx_power = v as i8;
                self.sync_radio_config();
            }
            (MacBeaconPayload, PibValue::Payload(v)) => self.beacon_payload = v,
            (MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = v;
            }
            (MacCoordExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.coord_extended_address = v;
            }
            (MacBeaconOrder, PibValue::U8(v)) => self.beacon_order = v,
            (MacSuperframeOrder, PibValue::U8(v)) => self.superframe_order = v,
            (MacResponseWaitTime, PibValue::U8(v)) => self.response_wait_time = v,
            _ => return Err(MacError::Unsupported),
        }

        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Build MAC Data Request command to parent (coordinator)
        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = build_data_request(self.next_dsn(), &parent, &self.extended_address);

        // Enable RX briefly for the poll exchange
        self.driver.enable_rx();

        // Transmit Data Request via CSMA-CA (ACK requested)
        self.csma_ca_transmit(&data_req, true).await?;

        // Wait for response — parent may reply with data or empty ACK
        let result = select::select(
            Timer::after_millis(POLL_RESPONSE_WAIT_MS),
            self.driver.receive(),
        )
        .await;

        // Disable RX after poll for sleepy devices
        if !self.rx_on_when_idle {
            self.driver.disable_rx();
        }

        match result {
            select::Either::Second(Ok(received)) => {
                if received.len < 5 {
                    return Ok(None);
                }
                let data = &received.data[..received.len];
                let fc = u16::from_le_bytes([data[0], data[1]]);
                let frame_type = fc & 0x07;

                // Only deliver data frames (type 1)
                if frame_type != 1 {
                    return Ok(None);
                }

                // Send ACK if the polled response requests one
                if (fc & 0x0020) != 0 && data.len() >= 3 {
                    self.send_ack(data[2], false).await;
                }

                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    return Ok(None);
                }

                Ok(MacFrame::from_slice(&data[header_len..]))
            }
            _ => Ok(None),
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        if req.tx_options.indirect {
            // Indirect transmission: buffer frame for sleepy child to poll.
            if self.pan_coordinator {
                let mut frame_buf = heapless::Vec::new();
                let ack_requested = req.tx_options.ack_tx;
                let built = self.build_data_frame(&req.dst_address, req.payload, ack_requested);
                if frame_buf.extend_from_slice(&built).is_err() {
                    log::warn!("telink: indirect frame too large");
                    return Err(MacError::FrameTooLong);
                }
                if self.indirect_queue.is_full() {
                    // Evict oldest entry
                    self.indirect_queue.remove(0);
                }
                let _ = self.indirect_queue.push(IndirectEntry {
                    dst: req.dst_address.clone(),
                    frame: frame_buf,
                    remaining_ticks: INDIRECT_PERSISTENCE_TICKS,
                });
                log::debug!(
                    "telink: buffered indirect frame (queue={})",
                    self.indirect_queue.len()
                );
                return Ok(McpsDataConfirm {
                    msdu_handle: req.msdu_handle,
                    timestamp: None,
                });
            }
            log::warn!("telink: indirect TX requested but not coordinator, sending directly");
        }

        let ack_requested = req.tx_options.ack_tx;
        let frame = self.build_data_frame(&req.dst_address, req.payload, ack_requested);

        // CSMA-CA + TX + ACK wait + retries
        self.csma_ca_transmit(&frame, ack_requested).await?;

        // Disable RX after TX for sleepy devices (saves power)
        if !self.rx_on_when_idle {
            self.driver.disable_rx();
        }

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        let deadline = embassy_time::Instant::now()
            + embassy_time::Duration::from_millis(RX_INDICATION_TIMEOUT_MS);

        loop {
            let now = embassy_time::Instant::now();
            if now >= deadline {
                return Err(MacError::NoData);
            }
            let remaining = deadline - now;

            let rx_result = select::select(self.driver.receive(), Timer::after(remaining)).await;

            match rx_result {
                select::Either::Second(_) => {
                    return Err(MacError::NoData);
                }
                select::Either::First(Err(_)) => {
                    continue;
                }
                select::Either::First(Ok(rx_frame)) => {
                    let data = &rx_frame.data[..rx_frame.len];

                    if data.len() < 5 {
                        continue;
                    }

                    let fc = u16::from_le_bytes([data[0], data[1]]);
                    let frame_type = fc & 0x07;

                    // Accept data frames (1) and MAC command frames (3) for processing
                    if frame_type != 1 && frame_type != 3 {
                        continue;
                    }

                    let header_len = 3 + addressing_size(fc);
                    if data.len() <= header_len {
                        continue;
                    }

                    let src = match parse_source_address(data, fc) {
                        Some(addr) => addr,
                        None => continue, // discard malformed frame
                    };
                    let dst = match parse_dest_address(data, fc) {
                        Some(addr) => addr,
                        None => continue, // discard malformed frame
                    };

                    // Software address filtering
                    if !self.promiscuous {
                        let accepted = match &dst {
                            MacAddress::Short(pan, addr) => {
                                (pan.0 == self.pan_id.0 || pan.0 == 0xFFFF)
                                    && (addr.0 == self.short_address.0 || addr.0 == 0xFFFF)
                            }
                            MacAddress::Extended(pan, addr) => {
                                (pan.0 == self.pan_id.0 || pan.0 == 0xFFFF)
                                    && *addr == self.extended_address
                            }
                        };
                        if !accepted {
                            continue;
                        }
                    }

                    log::info!(
                        "[Telink RX] Accepted frame {} bytes, LQI {}",
                        data.len(),
                        rx_frame.lqi
                    );

                    // Send ACK if the received frame requests one
                    if (fc & 0x0020) != 0 && data.len() >= 3 {
                        // Check if we have indirect frames for this source
                        let has_pending = self.has_indirect_frame_for(&src);
                        self.send_ack(data[2], has_pending).await;
                    }

                    // If coordinator and this is a MAC command frame (type 3),
                    // check for Data Request (poll) and drain indirect queue.
                    if frame_type == 3 && self.pan_coordinator {
                        // MAC command ID is the first byte after the header
                        if let Some(&cmd_id) = data.get(header_len) {
                            if cmd_id == 0x04 {
                                // Data Request — send buffered indirect frame
                                self.send_indirect_to(&src).await;
                            }
                        }
                    }

                    // Only deliver data frames (type 1) to upper layers
                    if frame_type != 1 {
                        continue;
                    }

                    let payload_data = &data[header_len..];
                    if let Some(mac_frame) = MacFrame::from_slice(payload_data) {
                        return Ok(McpsDataIndication {
                            src_address: src,
                            dst_address: dst,
                            lqi: rx_frame.lqi,
                            payload: mac_frame,
                            security_use: (fc >> 3) & 1 != 0,
                        });
                    }
                }
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: true, // Telink chips have AES-128 hardware
            // 127 (max PSDU) - 2 (FCS) - MAX_MAC_OVERHEAD = usable payload
            max_payload: (127 - 2 - MAX_MAC_OVERHEAD) as u16,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(10),
        }
    }
}

// ── Indirect TX queue helpers ──────────────────────────────────

impl TelinkMac {
    /// Check whether we have a buffered indirect frame for the given address.
    fn has_indirect_frame_for(&self, addr: &MacAddress) -> bool {
        self.indirect_queue
            .iter()
            .any(|e| addresses_match(&e.dst, addr))
    }

    /// Transmit the first matching indirect frame to the polling device.
    async fn send_indirect_to(&mut self, addr: &MacAddress) {
        if let Some(idx) = self
            .indirect_queue
            .iter()
            .position(|e| addresses_match(&e.dst, addr))
        {
            let entry = self.indirect_queue.remove(idx);
            log::debug!(
                "telink: sending indirect frame to poller (queue={})",
                self.indirect_queue.len()
            );
            let _ = self.csma_ca_transmit(&entry.frame, true).await;
        }
    }

    /// Age indirect queue entries. Call periodically (e.g., from tick).
    /// Removes entries that have expired.
    pub fn age_indirect_queue(&mut self) {
        self.indirect_queue.retain_mut(|entry| {
            entry.remaining_ticks = entry.remaining_ticks.saturating_sub(1);
            if entry.remaining_ticks == 0 {
                log::debug!("telink: indirect frame expired");
                false
            } else {
                true
            }
        });
    }
}

/// Check if two MAC addresses refer to the same device (ignoring PAN ID).
fn addresses_match(a: &MacAddress, b: &MacAddress) -> bool {
    match (a, b) {
        (MacAddress::Short(_, a_addr), MacAddress::Short(_, b_addr)) => a_addr.0 == b_addr.0,
        (MacAddress::Extended(_, a_addr), MacAddress::Extended(_, b_addr)) => a_addr == b_addr,
        _ => false,
    }
}

/// Build a MAC Data Request command frame (used for polling parent).
fn build_data_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command, ack req, PAN compress, dst=short, src=extended
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

// --- IEEE 802.15.4 frame parsing utilities ---

/// Calculate addressing field size from Frame Control word.
fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut size = 0;
    // Destination
    match dst_mode {
        0x02 => size += 2 + 2, // PAN(2) + Short(2)
        0x03 => size += 2 + 8, // PAN(2) + Extended(8)
        _ => {}
    }
    // Source
    match src_mode {
        0x02 => size += if pan_compress { 2 } else { 4 }, // Short ± PAN
        0x03 => size += if pan_compress { 8 } else { 10 }, // Extended ± PAN
        _ => {}
    }
    size
}

/// Parse source address from raw MAC frame.
fn parse_source_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    // Skip past FC(2) + Seq(1) + dst addressing
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

    // Source PAN
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
fn parse_dest_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let offset = 3; // After FC(2) + Seq(1)

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

/// Parse Zigbee beacon payload from raw bytes (at least 15 bytes).
fn parse_zigbee_beacon(data: &[u8]) -> ZigbeeBeaconPayload {
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
