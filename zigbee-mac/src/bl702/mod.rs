//! BL702 MAC backend.
//!
//! Implements `MacDriver` for the Bouffalo Lab BL702 RISC-V SoC. The BL702
//! has a built-in multi-protocol radio supporting BLE 5.0 and IEEE 802.15.4.
//!
//! This backend uses FFI bindings to Bouffalo's `lmac154` C library for
//! radio access, with async TX/RX through Embassy signals.
//!
//! # Architecture
//! ```text
//! MacDriver trait methods
//!        │
//!        ▼
//! Bl702Mac (this module)
//!   ├── PIB state (addresses, channel, config)
//!   ├── Frame construction (beacon req, assoc req, data)
//!   └── Bl702Driver (driver.rs)
//!          ├── FFI → liblmac154.a (Bouffalo C library)
//!          ├── TX via Signal (interrupt-driven)
//!          └── RX via Signal (interrupt-driven)
//! ```
//!
//! # Dependencies
//! - `liblmac154.a` — Bouffalo's pre-compiled 802.15.4 MAC/PHY library (linked via FFI)
//! - `bl702-hal` — clock and GPIO configuration
//! - Embassy async primitives (`embassy-sync`, `embassy-time`, `embassy-futures`)
//!
//! # Hardware
//! - BL702 module boards (XT-ZB1, DT-BL10, Pine64 Pinenut)
//! - BL706 IoT Development Board (Sipeed)
//!
//! # Build requirements
//! The firmware crate must link `liblmac154.a` from Bouffalo's IoT SDK.
//! See `driver.rs` module documentation for details.

pub mod driver;

use crate::pib::{self, PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::{Bl702Driver, RadioConfig, RadioError};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

/// BL702 802.15.4 MAC driver.
///
/// Built on direct `bl702-pac` register access — uses the BL702's hardware
/// radio with interrupt-driven TX/RX via Embassy signals.
///
/// # Usage
/// ```rust,no_run
/// use zigbee_mac::bl702::Bl702Mac;
///
/// // After initializing BL702 clocks and enabling radio peripheral:
/// let mac = Bl702Mac::new();
/// let device = zigbee_runtime::builder(mac)
///     .device_type(zigbee_nwk::DeviceType::EndDevice)
///     .build();
/// ```
pub struct Bl702Mac {
    driver: Bl702Driver,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    dsn: u8,
    bsn: u8,
    beacon_payload: PibPayload,
    max_csma_backoffs: u8,
    min_be: u8,
    max_be: u8,
    max_frame_retries: u8,
    promiscuous: bool,
    tx_power: i8,
}

impl Bl702Mac {
    /// Create a new BL702 MAC driver with default PIB values.
    ///
    /// The caller must have already:
    /// 1. Enabled BL702 radio peripheral clocks
    /// 2. Configured the radio interrupt to call `driver::rx_callback`
    ///    and `driver::tx_callback`
    pub fn new() -> Self {
        Self {
            driver: Bl702Driver::new(RadioConfig::default()),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            bsn: 0,
            beacon_payload: PibPayload::new(),
            max_csma_backoffs: 4,
            min_be: 3,
            max_be: 5,
            max_frame_retries: 3,
            promiscuous: false,
            tx_power: 0,
        }
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    fn next_bsn(&mut self) -> u8 {
        let seq = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        seq
    }

    /// Construct an IEEE 802.15.4 Beacon Request MAC command frame.
    fn beacon_request_frame(&mut self) -> [u8; 10] {
        let seq = self.next_dsn();
        // Frame Control: MAC command, no security, no frame pending,
        // no ack request, no PAN ID compression, dst=short, src=none
        [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00]
    }

    /// Construct an IEEE 802.15.4 Association Request MAC command frame.
    fn association_request_frame(
        &mut self,
        coord_address: &MacAddress,
        capability_info: &CapabilityInfo,
    ) -> heapless::Vec<u8, 32> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: MAC command, no security, no pending,
        // ack requested, PAN ID compression, dst=short, src=extended
        let _ = frame.extend_from_slice(&[0x23, 0xC8, seq]);

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
                        if let Some(desc) = self.parse_beacon(&rx_frame.data[..rx_frame.len]) {
                            let _ = results.push(desc);
                        }
                    }
                    Err(_) => break,
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
            Ok((rssi, _busy)) => rssi.wrapping_add(128) as u8, // Convert to unsigned energy level
            Err(_) => 0,
        }
    }

    /// Try to parse a received frame as a beacon and extract a PAN descriptor.
    fn parse_beacon(&self, _frame_data: &[u8]) -> Option<PanDescriptor> {
        // TODO: Parse IEEE 802.15.4 beacon frame format:
        //   - Frame Control (2 bytes)
        //   - Sequence Number (1 byte)
        //   - Address fields (variable)
        //   - Superframe Specification (2 bytes)
        //   - GTS fields (variable)
        //   - Pending Address fields (variable)
        //   - Beacon Payload (variable)
        //
        // Extract: coordinator address, PAN ID, channel, LQI, superframe spec
        // Return PanDescriptor on success.
        //
        // For now, this is a placeholder — full beacon parsing will be
        // implemented when the radio driver is functional.
        None
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

impl MacDriver for Bl702Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let scan_duration_ms = ((1u32 << req.scan_duration) + 1) * 15; // ~aBaseSuperframeDuration

        match req.scan_type {
            ScanType::Active | ScanType::Passive => {
                let mut descriptors: heapless::Vec<PanDescriptor, 16> = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if req.channel_mask.contains(ch) {
                        let beacons = self.scan_channel_active(ch, scan_duration_ms).await;
                        for desc in beacons {
                            let _ = descriptors.push(desc);
                        }
                    }
                }

                // Restore original channel
                self.sync_radio_config();

                if descriptors.is_empty() {
                    Err(MacError::NoBeacon)
                } else {
                    Ok(MlmeScanConfirm::ActivePassive { descriptors })
                }
            }
            ScanType::Ed => {
                let mut energy_levels: heapless::Vec<u8, 16> = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if req.channel_mask.contains(ch) {
                        let energy = self.scan_channel_ed(ch).await;
                        let _ = energy_levels.push(energy);
                    }
                }

                self.sync_radio_config();

                Ok(MlmeScanConfirm::Ed { energy_levels })
            }
            ScanType::Orphan => Err(MacError::Unsupported),
        }
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        // Switch to coordinator's channel
        self.driver.update_config(|cfg| cfg.channel = req.channel);

        // Build and send association request
        let frame = self.association_request_frame(&req.coord_address, &req.capability_info);
        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| MacError::RadioError)?;

        // Wait for association response with timeout
        let timeout = Timer::after(embassy_time::Duration::from_millis(500));

        let wait_response = async {
            loop {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        // TODO: Parse association response MAC command (0x02)
                        // Extract: assigned short address, association status
                        //
                        // For now check if we got any frame back
                        if rx_frame.len > 0 {
                            // Placeholder: would extract actual address from frame
                            return Ok(MlmeAssociateConfirm {
                                short_address: ShortAddress(0xFFFE),
                            });
                        }
                    }
                    Err(_) => return Err(MacError::RadioError),
                }
            }
        };

        match select::select(timeout, wait_response).await {
            select::Either::First(_) => Err(MacError::NoAck), // Timeout
            select::Either::Second(result) => result,
        }
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        // TODO: Build and send Association Response MAC command frame.
        // Required for coordinator/router role to accept joining devices.
        //
        // Steps:
        // 1. Build MAC command frame (type 0x02) with:
        //    - Assigned short address (from rsp.short_address)
        //    - Association status (success/denied)
        // 2. Queue as indirect transmission (for the joining device to poll)
        //    OR transmit directly if device is rx-on-when-idle
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        // TODO: Build and send Disassociation Notification MAC command.
        //
        // Steps:
        // 1. Build MAC command frame (type 0x03) with disassociate reason
        // 2. Transmit to coordinator/device
        // 3. Clear local addressing state

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
        }

        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.logical_channel;

        self.sync_radio_config();
        log::info!(
            "bl702: started PAN 0x{:04X} on channel {}",
            self.pan_id.0,
            self.channel
        );
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        use PibAttribute::*;
        use PibValue::*;

        match attr {
            MacShortAddress => Ok(Short(self.short_address.0)),
            MacPanId => Ok(Short(self.pan_id.0)),
            MacExtendedAddress => Ok(ExtAddress(self.extended_address)),
            MacRxOnWhenIdle => Ok(Bool(self.rx_on_when_idle)),
            MacAssociationPermit => Ok(Bool(self.association_permit)),
            MacAutoRequest => Ok(Bool(self.auto_request)),
            MacDsn => Ok(Byte(self.dsn)),
            MacBsn => Ok(Byte(self.bsn)),
            MacMaxCsmaBackoffs => Ok(Byte(self.max_csma_backoffs)),
            MacMinBe => Ok(Byte(self.min_be)),
            MacMaxBe => Ok(Byte(self.max_be)),
            MacMaxFrameRetries => Ok(Byte(self.max_frame_retries)),
            MacPromiscuousMode => Ok(Bool(self.promiscuous)),
            PhyCurrentChannel => Ok(Byte(self.channel)),
            PhyTransmitPower => Ok(Byte(self.tx_power as u8)),
            PhyChannelsSupported => Ok(Word(pib::CHANNELS_2_4GHZ)),
            PhyCurrentPage => Ok(Byte(0)), // Page 0 = 2.4 GHz
            MacBeaconPayload => Ok(Payload(self.beacon_payload.clone())),
            _ => Err(MacError::Unsupported),
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        use PibAttribute::*;
        use PibValue::*;

        match (attr, value) {
            (MacShortAddress, Short(v)) => {
                self.short_address = ShortAddress(v);
                self.sync_radio_config();
            }
            (MacPanId, Short(v)) => {
                self.pan_id = PanId(v);
                self.sync_radio_config();
            }
            (MacExtendedAddress, ExtAddress(v)) => {
                self.extended_address = v;
                self.sync_radio_config();
            }
            (MacRxOnWhenIdle, Bool(v)) => self.rx_on_when_idle = v,
            (MacAssociationPermit, Bool(v)) => self.association_permit = v,
            (MacAutoRequest, Bool(v)) => self.auto_request = v,
            (MacDsn, Byte(v)) => self.dsn = v,
            (MacBsn, Byte(v)) => self.bsn = v,
            (MacMaxCsmaBackoffs, Byte(v)) => self.max_csma_backoffs = v,
            (MacMinBe, Byte(v)) => self.min_be = v,
            (MacMaxBe, Byte(v)) => self.max_be = v,
            (MacMaxFrameRetries, Byte(v)) => self.max_frame_retries = v,
            (MacPromiscuousMode, Bool(v)) => {
                self.promiscuous = v;
                self.sync_radio_config();
            }
            (PhyCurrentChannel, Byte(v)) => {
                self.channel = v;
                self.sync_radio_config();
            }
            (PhyTransmitPower, Byte(v)) => {
                self.tx_power = v as i8;
                self.sync_radio_config();
            }
            (MacBeaconPayload, Payload(v)) => self.beacon_payload = v,
            _ => return Err(MacError::Unsupported),
        }

        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // TODO: Send Data Request MAC command to coordinator and wait
        // for an indirect frame or empty ACK.
        //
        // Steps:
        // 1. Build Data Request MAC command (type 0x04)
        // 2. Transmit to coordinator address
        // 3. If ACK has frame-pending bit set, wait for the data frame
        // 4. Return the received frame or None
        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let ack_request = req.tx_options.ack_request();

        let frame = self.build_data_frame(&req.dst_address, req.payload, ack_request);

        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| MacError::RadioError)?;

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        loop {
            let rx_frame = self
                .driver
                .receive()
                .await
                .map_err(|_| MacError::RadioError)?;

            if rx_frame.len == 0 {
                continue;
            }

            // TODO: Full 802.15.4 frame parsing to extract:
            //   - Frame type (filter for data frames)
            //   - Source/destination addresses
            //   - Payload
            //   - Security fields
            //
            // For now, return a minimal indication with raw data.
            // This will be replaced with proper frame parsing.

            let data = &rx_frame.data[..rx_frame.len];

            // Minimal frame parsing: need at least frame control (2) + seq (1)
            if data.len() < 3 {
                continue;
            }

            let frame_type = data[0] & 0x07;
            if frame_type != 0x01 {
                // Not a data frame — skip
                continue;
            }

            // Extract payload (skip header — simplified, assumes short addressing)
            // A proper implementation would parse the full MAC header.
            let header_len = 9; // FC(2) + Seq(1) + DstPAN(2) + DstAddr(2) + SrcAddr(2) minimum
            if data.len() <= header_len {
                continue;
            }

            let mut payload = MacPayload::new();
            let _ = payload.extend_from_slice(&data[header_len..]);

            return Ok(McpsDataIndication {
                src_address: MacAddress::Short(self.pan_id, ShortAddress(0x0000)),
                dst_address: MacAddress::Short(self.pan_id, self.short_address),
                lqi: rx_frame.lqi,
                payload,
                security_use: false,
            });
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: true, // BL702 has AES-128 hardware
            max_payload: 102,        // 127 - 25 (max MAC overhead)
            tx_power_min: TxPower(-21),
            tx_power_max: TxPower(14),
        }
    }
}
