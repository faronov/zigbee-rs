//! Mock MAC backend for unit testing.
//!
//! Implements `MacDriver` with configurable responses — no hardware required.
//! Use this to test NWK, APS, ZCL, and BDB logic without a radio.

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use zigbee_types::*;

use heapless::Vec;

/// Configurable mock MAC for stack testing.
///
/// # Example
/// ```rust,no_run,ignore
/// let mut mock = MockMac::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
///
/// // Pre-configure a beacon to be returned during scan
/// mock.add_beacon(PanDescriptor {
///     channel: 15,
///     coord_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
///     superframe_spec: SuperframeSpec::default(),
///     lqi: 200,
///     security_use: false,
///     zigbee_beacon: ZigbeeBeaconPayload { /* ... */ },
/// });
///
/// // Use mock with NWK layer
/// let nlme = Nlme::new(storage, mock);
/// ```
pub struct MockMac {
    // PIB state
    ieee_address: IeeeAddress,
    short_address: ShortAddress,
    pan_id: PanId,
    coord_short_address: ShortAddress,
    associated_pan_coord: bool,
    channel: u8,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    dsn: u8,
    beacon_payload: PibPayload,
    tx_power: i8,
    promiscuous: bool,

    // Pre-configured responses
    scan_beacons: Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>,
    scan_energy: Vec<EdValue, MAX_ED_VALUES>,
    associate_response: Option<MlmeAssociateConfirm>,
    poll_queue: Vec<MacFrame, 8>,

    // Recorded calls for assertions
    tx_history: Vec<TxRecord, 16>,
    rx_queue: Vec<McpsDataIndication, 8>,
    poll_count: u32,
    poll_delay_us: u32,
    time_micros: u32,
    rx_delay_us: u32,
    random_state: u32,
}

/// Record of a transmitted frame (for test assertions)
#[derive(Debug)]
pub struct TxRecord {
    pub dst: MacAddress,
    pub payload_len: usize,
    pub payload: MacFrame,
    pub handle: u8,
    pub ack_requested: bool,
}

impl MockMac {
    /// Create a new mock MAC with the given IEEE address.
    pub fn new(ieee_address: IeeeAddress) -> Self {
        let random_state = u32::from_le_bytes([
            ieee_address[0],
            ieee_address[1],
            ieee_address[2],
            ieee_address[3],
        ]) ^ u32::from_le_bytes([
            ieee_address[4],
            ieee_address[5],
            ieee_address[6],
            ieee_address[7],
        ]) ^ 0xA536_6B4D;
        Self {
            ieee_address,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            coord_short_address: ShortAddress(0xFFFF),
            associated_pan_coord: false,
            channel: 11,
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            beacon_payload: PibPayload::new(),
            tx_power: 0,
            promiscuous: false,
            scan_beacons: Vec::new(),
            scan_energy: Vec::new(),
            associate_response: None,
            poll_queue: Vec::new(),
            tx_history: Vec::new(),
            rx_queue: Vec::new(),
            poll_count: 0,
            poll_delay_us: 0,
            time_micros: 0,
            rx_delay_us: 0,
            random_state: random_state.max(1),
        }
    }

    // ── Test setup methods ──────────────────────────────────

    /// Add a beacon that will be returned during active/passive scan.
    pub fn add_beacon(&mut self, pd: PanDescriptor) {
        let _ = self.scan_beacons.push(pd);
    }

    /// Add an energy measurement for ED scan.
    pub fn add_energy(&mut self, ed: EdValue) {
        let _ = self.scan_energy.push(ed);
    }

    /// Set the response that will be returned for MLME-ASSOCIATE.
    pub fn set_associate_response(&mut self, rsp: MlmeAssociateConfirm) {
        self.associate_response = Some(rsp);
    }

    /// Enqueue a frame that will be delivered via mcps_data_indication.
    pub fn enqueue_rx(&mut self, ind: McpsDataIndication) {
        let _ = self.rx_queue.push(ind);
    }

    /// Set the simulated delay before the next direct data indication.
    pub fn set_rx_delay_us(&mut self, delay_us: u32) {
        self.rx_delay_us = delay_us;
    }

    /// Enqueue a frame that will be returned by the next MLME-POLL.
    pub fn enqueue_poll_response(&mut self, frame: MacFrame) {
        let _ = self.poll_queue.push(frame);
    }

    /// Set the simulated delay before the next poll result.
    pub fn set_poll_delay_us(&mut self, delay_us: u32) {
        self.poll_delay_us = delay_us;
    }

    // ── Test assertion methods ──────────────────────────────

    /// Get all transmitted frames for verification.
    pub fn tx_history(&self) -> &[TxRecord] {
        &self.tx_history
    }

    /// Clear TX history.
    pub fn clear_tx_history(&mut self) {
        self.tx_history.clear();
    }

    /// Number of MLME-POLL requests issued.
    pub fn poll_count(&self) -> u32 {
        self.poll_count
    }
}

impl PlatformServices for MockMac {
    fn monotonic_micros(&self) -> u32 {
        self.time_micros
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.time_micros = self.time_micros.wrapping_add(duration_us);
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        for chunk in output.chunks_mut(4) {
            let mut value = self.random_state;
            value ^= value << 13;
            value ^= value >> 17;
            value ^= value << 5;
            self.random_state = value.max(1);
            chunk.copy_from_slice(&value.to_le_bytes()[..chunk.len()]);
        }
        Ok(())
    }
}

impl MacDriver for MockMac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        match req.scan_type {
            ScanType::Ed => Ok(MlmeScanConfirm {
                scan_type: ScanType::Ed,
                pan_descriptors: Vec::new(),
                energy_list: self.scan_energy.clone(),
            }),
            ScanType::Active | ScanType::Passive => {
                // Filter beacons by channel mask
                let mut pds = Vec::new();
                for pd in &self.scan_beacons {
                    if req
                        .channel_mask
                        .contains(Channel::from_number(pd.channel).unwrap_or(Channel::Ch11))
                    {
                        let _ = pds.push(pd.clone());
                    }
                }
                if pds.is_empty() {
                    return Err(MacError::NoBeacon);
                }
                Ok(MlmeScanConfirm {
                    scan_type: req.scan_type,
                    pan_descriptors: pds,
                    energy_list: Vec::new(),
                })
            }
            ScanType::Orphan => {
                // Orphan scan: return first beacon if any, else error
                if self.scan_beacons.is_empty() {
                    Err(MacError::NoBeacon)
                } else {
                    let mut pds = Vec::new();
                    let _ = pds.push(self.scan_beacons[0].clone());
                    Ok(MlmeScanConfirm {
                        scan_type: ScanType::Orphan,
                        pan_descriptors: pds,
                        energy_list: Vec::new(),
                    })
                }
            }
        }
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.channel = req.channel;
        self.pan_id = req.coord_address.pan_id();
        if let MacAddress::Short(_, coordinator) = req.coord_address {
            self.coord_short_address = coordinator;
        }

        match &self.associate_response {
            Some(rsp) => {
                let confirm = rsp.clone();
                if confirm.status == AssociationStatus::Success {
                    self.short_address = confirm.short_address;
                    self.associated_pan_coord = true;
                }
                Ok(confirm)
            }
            None => Err(MacError::NoAck),
        }
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        // Mock: just accept
        Ok(())
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        self.short_address = ShortAddress(0xFFFF);
        self.pan_id = PanId(0xFFFF);
        self.coord_short_address = ShortAddress(0xFFFF);
        self.associated_pan_coord = false;
        Ok(())
    }

    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.coord_short_address = ShortAddress(0xFFFF);
            self.associated_pan_coord = false;
            self.channel = 11;
            self.rx_on_when_idle = false;
            self.association_permit = false;
            self.auto_request = true;
            self.dsn = 0;
        }
        self.tx_history.clear();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        match attr {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.ieee_address)),
            PibAttribute::MacCoordShortAddress => {
                Ok(PibValue::ShortAddress(self.coord_short_address))
            }
            PibAttribute::MacAssociatedPanCoord => Ok(PibValue::Bool(self.associated_pan_coord)),
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            PibAttribute::PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            _ => Ok(PibValue::U8(0)), // Default for unhandled attributes
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        match attr {
            PibAttribute::MacShortAddress => {
                self.short_address = value.as_short_address().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacPanId => {
                self.pan_id = value.as_pan_id().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacCoordShortAddress => {
                self.coord_short_address =
                    value.as_short_address().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAssociatedPanCoord => {
                self.associated_pan_coord = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacRxOnWhenIdle => {
                self.rx_on_when_idle = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAssociationPermit => {
                self.association_permit = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAutoRequest => {
                self.auto_request = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::PhyCurrentChannel => {
                self.channel = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::PhyTransmitPower => {
                if let PibValue::I8(p) = value {
                    self.tx_power = p;
                }
            }
            PibAttribute::MacPromiscuousMode => {
                self.promiscuous = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacBeaconPayload => {
                if let PibValue::Payload(p) = value {
                    self.beacon_payload = p;
                }
            }
            _ => {} // Silently accept other attributes
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        self.poll_count = self.poll_count.wrapping_add(1);
        if self.poll_queue.is_empty() {
            Ok(None)
        } else {
            Ok(Some(self.poll_queue.remove(0)))
        }
    }

    async fn mlme_poll_timeout(&mut self, timeout_us: u32) -> Result<Option<MacFrame>, MacError> {
        if self.poll_delay_us >= timeout_us {
            self.poll_count = self.poll_count.wrapping_add(1);
            self.time_micros = self.time_micros.wrapping_add(timeout_us);
            return Ok(None);
        }
        self.time_micros = self.time_micros.wrapping_add(self.poll_delay_us);
        self.poll_delay_us = 0;
        self.mlme_poll().await
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let payload = MacFrame::from_slice(req.payload).ok_or(MacError::FrameTooLong)?;
        let record = TxRecord {
            dst: req.dst_address,
            payload_len: req.payload.len(),
            payload,
            handle: req.msdu_handle,
            ack_requested: req.tx_options.ack_tx,
        };
        let _ = self.tx_history.push(record);

        self.dsn = self.dsn.wrapping_add(1);

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        if self.rx_queue.is_empty() {
            // In a real impl this would block. Mock just returns error.
            return Err(MacError::Other);
        }
        // Pop from front (swap_remove from index 0)
        Ok(self.rx_queue.swap_remove(0))
    }

    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        if self.rx_delay_us >= timeout_us {
            self.time_micros = self.time_micros.wrapping_add(timeout_us);
            return Err(MacError::NoData);
        }
        self.time_micros = self.time_micros.wrapping_add(self.rx_delay_us);
        self.rx_delay_us = 0;
        self.mcps_data_indication().await
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(20),
        }
    }
}
