//! ESP32-C6 MAC backend.
//!
//! Wraps `esp-radio::ieee802154` radio driver to implement `MacDriver`.

mod driver;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::Ieee802154Driver;
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;
use esp_radio::ieee802154::{Config, Ieee802154};

/// ESP32-C6 802.15.4 MAC driver.
pub struct EspMac<'a> {
    driver: Ieee802154Driver<'a>,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    coord_short_address: ShortAddress,
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

impl<'a> EspMac<'a> {
    pub fn new(ieee802154: Ieee802154<'a>, config: Config) -> Self {
        Self {
            driver: Ieee802154Driver::new(ieee802154, config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
            coord_short_address: ShortAddress(0x0000),
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

    #[allow(dead_code)]
    fn next_bsn(&mut self) -> u8 {
        let seq = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        seq
    }
}

impl MacDriver for EspMac<'_> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();

        // Scan duration: aBaseSuperframeDuration * (2^n + 1) symbols
        let scan_us: u64 = 15360 * ((1u64 << req.scan_duration) + 1);

        for ch in req.channel_mask.iter() {
            let ch_num = ch.number();
            self.channel = ch_num;
            self.driver.update_config(|cfg| cfg.channel = ch_num);

            match req.scan_type {
                ScanType::Active => {
                    let beacon_req = build_beacon_request(self.next_dsn());
                    let _ = self.driver.transmit(&beacon_req).await;

                    let _ = select::select(Timer::after_micros(scan_us), async {
                        if let Ok(_rx) = self.driver.receive().await {
                            // TODO: parse beacon and push to pan_descriptors
                        }
                    })
                    .await;
                }
                ScanType::Passive => {
                    let _ = select::select(Timer::after_micros(scan_us), async {
                        if let Ok(_rx) = self.driver.receive().await {
                            // TODO: parse beacon
                        }
                    })
                    .await;
                }
                ScanType::Ed => {
                    // TODO: real energy detection
                    let _ = energy_list.push(EdValue {
                        channel: ch_num,
                        energy: 0,
                    });
                }
                ScanType::Orphan => {}
            }
        }

        Ok(MlmeScanConfirm {
            scan_type: req.scan_type,
            pan_descriptors,
            energy_list,
        })
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.channel = req.channel;
        self.pan_id = req.coord_address.pan_id();
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = Some(req.coord_address.pan_id().0);
        });

        // TODO: build & send Association Request, wait for response
        Ok(MlmeAssociateConfirm {
            short_address: ShortAddress(0xFFFE),
            status: AssociationStatus::Success,
        })
    }

    async fn mlme_associate_response(
        &mut self,
        _resp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        Ok(())
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
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
        self.driver.update_config(|cfg| {
            cfg.channel = self.channel;
            cfg.pan_id = Some(self.pan_id.0);
            cfg.short_addr = Some(self.short_address.0);
        });
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = Some(req.pan_id.0);
        });
        Ok(())
    }

    async fn mlme_get(&self, attribute: PibAttribute) -> Result<PibValue, MacError> {
        match attribute {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::MacExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.extended_address))
            }
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::MacBsn => Ok(PibValue::U8(self.bsn)),
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            PibAttribute::MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            PibAttribute::MacMinBe => Ok(PibValue::U8(self.min_be)),
            PibAttribute::MacMaxBe => Ok(PibValue::U8(self.max_be)),
            PibAttribute::MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            _ => Err(MacError::Unsupported),
        }
    }

    async fn mlme_set(&mut self, attribute: PibAttribute, value: PibValue) -> Result<(), MacError> {
        match (attribute, value) {
            (PibAttribute::MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = v;
                self.driver.update_config(|cfg| cfg.short_addr = Some(v.0));
            }
            (PibAttribute::MacPanId, PibValue::PanId(v)) => {
                self.pan_id = v;
                self.driver.update_config(|cfg| cfg.pan_id = Some(v.0));
            }
            (PibAttribute::PhyCurrentChannel, PibValue::U8(v)) => {
                self.channel = v;
                self.driver.update_config(|cfg| cfg.channel = v);
            }
            (PibAttribute::MacExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.extended_address = v;
            }
            (PibAttribute::MacRxOnWhenIdle, PibValue::Bool(v)) => self.rx_on_when_idle = v,
            (PibAttribute::MacAssociationPermit, PibValue::Bool(v)) => {
                self.association_permit = v;
            }
            (PibAttribute::MacAutoRequest, PibValue::Bool(v)) => self.auto_request = v,
            (PibAttribute::MacBeaconPayload, PibValue::Payload(v)) => self.beacon_payload = v,
            (PibAttribute::MacMaxCsmaBackoffs, PibValue::U8(v)) => self.max_csma_backoffs = v,
            (PibAttribute::MacMinBe, PibValue::U8(v)) => self.min_be = v,
            (PibAttribute::MacMaxBe, PibValue::U8(v)) => self.max_be = v,
            (PibAttribute::MacMaxFrameRetries, PibValue::U8(v)) => self.max_frame_retries = v,
            (PibAttribute::MacPromiscuousMode, PibValue::Bool(v)) => self.promiscuous = v,
            (PibAttribute::PhyTransmitPower, PibValue::I8(v)) => self.tx_power = v,
            (PibAttribute::MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = v;
            }
            _ => return Err(MacError::Unsupported),
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Build MAC Data Request command to parent (coordinator)
        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = build_data_request(self.next_dsn(), &parent, &self.extended_address);

        // Transmit Data Request
        self.driver
            .transmit(&data_req)
            .await
            .map_err(|_| MacError::RadioError)?;

        // Wait for response — parent may reply with data or empty ACK
        let result = select::select(Timer::after_millis(500), self.driver.receive()).await;

        match result {
            select::Either::Second(Ok(received)) => {
                if received.len < 3 {
                    return Ok(None);
                }
                let frame_type = received.data[0] & 0x07;
                if frame_type != 0x01 {
                    return Ok(None); // Not a data frame
                }
                let payload =
                    MacFrame::from_slice(&received.data[..received.len]).unwrap_or_default();
                Ok(Some(payload))
            }
            _ => Ok(None), // Timeout or error
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let msdu_handle = req.msdu_handle;
        let mut frame_buf = [0u8; 127];
        let len = build_data_frame(
            &mut frame_buf,
            self.next_dsn(),
            self.short_address,
            self.pan_id,
            &self.extended_address,
            &req,
        )?;

        // Fire-and-forget: try up to 3 times (no ACK verification)
        for attempt in 0..3u8 {
            match self.driver.transmit(&frame_buf[..len]).await {
                Ok(_) => break,
                Err(_) if attempt < 2 => {
                    Timer::after_millis(2).await;
                    continue;
                }
                Err(_) => return Err(MacError::RadioError),
            }
        }

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        loop {
            let received = self
                .driver
                .receive()
                .await
                .map_err(|_| MacError::RadioError)?;

            if received.len < 3 {
                continue;
            }

            let frame_type = received.data[0] & 0x07;
            if frame_type != 0x01 {
                continue; // Not a data frame
            }

            let payload = MacFrame::from_slice(&received.data[..received.len]).unwrap_or_default();

            return Ok(McpsDataIndication {
                src_address: MacAddress::Short(self.pan_id, ShortAddress(0x0000)),
                dst_address: MacAddress::Short(self.pan_id, self.short_address),
                lqi: received.lqi,
                payload,
                security_use: false,
            });
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-24),
            tx_power_max: TxPower(21),
        }
    }
}

/// Build a Beacon Request MAC command frame.
fn build_beacon_request(seq: u8) -> [u8; 8] {
    [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
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

/// Build an IEEE 802.15.4 data frame into `buf`, return length.
fn build_data_frame(
    buf: &mut [u8; 127],
    seq: u8,
    short_addr: ShortAddress,
    _pan_id: PanId,
    extended_addr: &IeeeAddress,
    req: &McpsDataRequest<'_>,
) -> Result<usize, MacError> {
    let mut fc: u16 = 0x0001; // Data frame
    if req.tx_options.ack_tx {
        fc |= 0x0020;
    }
    fc |= 0x0040; // PAN ID compression

    match req.dst_address {
        MacAddress::Short(_, _) => fc |= 0x0800,
        MacAddress::Extended(_, _) => fc |= 0x0C00,
    }
    if short_addr.0 != 0xFFFF {
        fc |= 0x8000;
    } else {
        fc |= 0xC000;
    }

    let mut pos = 0;
    buf[pos] = (fc & 0xFF) as u8;
    pos += 1;
    buf[pos] = ((fc >> 8) & 0xFF) as u8;
    pos += 1;
    buf[pos] = seq;
    pos += 1;

    let dst_pan = req.dst_address.pan_id();
    buf[pos..pos + 2].copy_from_slice(&dst_pan.0.to_le_bytes());
    pos += 2;

    match req.dst_address {
        MacAddress::Short(_, addr) => {
            buf[pos..pos + 2].copy_from_slice(&addr.0.to_le_bytes());
            pos += 2;
        }
        MacAddress::Extended(_, addr) => {
            buf[pos..pos + 8].copy_from_slice(&addr);
            pos += 8;
        }
    }

    if short_addr.0 != 0xFFFF {
        buf[pos..pos + 2].copy_from_slice(&short_addr.0.to_le_bytes());
        pos += 2;
    } else {
        buf[pos..pos + 8].copy_from_slice(extended_addr);
        pos += 8;
    }

    if pos + req.payload.len() > 125 {
        return Err(MacError::FrameTooLong);
    }
    buf[pos..pos + req.payload.len()].copy_from_slice(req.payload);
    pos += req.payload.len();

    Ok(pos)
}
