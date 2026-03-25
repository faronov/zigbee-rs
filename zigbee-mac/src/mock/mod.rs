//! Mock MAC backend for unit testing.
//!
//! Implements `MacDriver` with configurable responses — no hardware required.
//! Use this to test NWK, APS, ZCL, and BDB logic without a radio.

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
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

    // Recorded calls for assertions
    tx_history: Vec<TxRecord, 16>,
    rx_queue: Vec<McpsDataIndication, 8>,
}

/// Record of a transmitted frame (for test assertions)
#[derive(Debug)]
pub struct TxRecord {
    pub dst: MacAddress,
    pub payload_len: usize,
    pub handle: u8,
    pub ack_requested: bool,
}

impl MockMac {
    /// Create a new mock MAC with the given IEEE address.
    pub fn new(ieee_address: IeeeAddress) -> Self {
        Self {
            ieee_address,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
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
            tx_history: Vec::new(),
            rx_queue: Vec::new(),
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

    // ── Test assertion methods ──────────────────────────────

    /// Get all transmitted frames for verification.
    pub fn tx_history(&self) -> &[TxRecord] {
        &self.tx_history
    }

    /// Clear TX history.
    pub fn clear_tx_history(&mut self) {
        self.tx_history.clear();
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

        match &self.associate_response {
            Some(rsp) => {
                let confirm = rsp.clone();
                if confirm.status == AssociationStatus::Success {
                    self.short_address = confirm.short_address;
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
        // Mock: return nothing (no pending indirect frames)
        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let record = TxRecord {
            dst: req.dst_address,
            payload_len: req.payload.len(),
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
