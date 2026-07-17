//! MAC PAN Information Base (PIB) attributes and values.
//!
//! PIB attributes are the configuration interface between the Zigbee NWK layer
//! and the MAC. The NWK layer uses MLME-GET/SET to read and write these.

use zigbee_types::{ChannelMask, IeeeAddress, PanId, ShortAddress};

/// MAC PIB attribute identifiers (IEEE 802.15.4 Table 8-82)
///
/// Only attributes actually used by Zigbee PRO R22 are included.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PibAttribute {
    // ── Addressing (critical — set during join) ─────────────
    /// Own 16-bit short address. Default: 0xFFFF (unassigned)
    MacShortAddress = 0x53,
    /// PAN ID of the network we're in. Default: 0xFFFF (not associated)
    MacPanId = 0x50,
    /// Own 64-bit IEEE address (read from hardware, usually read-only)
    MacExtendedAddress = 0x6F,
    /// Short address of our parent coordinator/router
    MacCoordShortAddress = 0x4B,
    /// Extended address of our parent coordinator/router
    MacCoordExtendedAddress = 0x4A,

    // ── Network configuration ───────────────────────────────
    /// True if this device is the PAN coordinator
    MacAssociatedPanCoord = 0x56,
    /// RX enabled during idle (true for ZC/ZR, false for sleepy ZED)
    MacRxOnWhenIdle = 0x52,
    /// True = accepting association requests (join permit open)
    MacAssociationPermit = 0x41,

    // ── Beacon (always 15/15 for Zigbee non-beacon mode) ────
    /// Beacon order. ALWAYS 15 for Zigbee PRO (non-beacon mode)
    MacBeaconOrder = 0x47,
    /// Superframe order. ALWAYS 15 for Zigbee PRO
    MacSuperframeOrder = 0x54,
    /// Beacon payload bytes (NWK beacon content for ZC/ZR)
    MacBeaconPayload = 0x45,
    /// Length of beacon payload
    MacBeaconPayloadLength = 0x46,

    // ── TX/RX tuning ────────────────────────────────────────
    /// Auto data-request after beacon with pending bit (ZED)
    MacAutoRequest = 0x42,
    /// Max CSMA-CA backoffs (default 4)
    MacMaxCsmaBackoffs = 0x4E,
    /// Min backoff exponent (default 3 for 2.4 GHz)
    MacMinBe = 0x4F,
    /// Max backoff exponent (default 5)
    MacMaxBe = 0x57,
    /// Max frame retries after ACK failure (default 3)
    MacMaxFrameRetries = 0x59,
    /// Max wait for indirect TX frame (symbols)
    MacMaxFrameTotalWaitTime = 0x58,
    /// Response wait time for association etc
    MacResponseWaitTime = 0x5A,

    // ── Sequence numbers ────────────────────────────────────
    /// Data/command frame sequence number
    MacDsn = 0x4C,
    /// Beacon sequence number
    MacBsn = 0x49,

    // ── Indirect TX (ZC/ZR) ─────────────────────────────────
    /// How long coordinator stores indirect frames (symbols)
    MacTransactionPersistenceTime = 0x55,

    // ── Debug / special ─────────────────────────────────────
    /// Promiscuous mode (sniffer use)
    MacPromiscuousMode = 0x51,

    // ── PHY attributes (accessed via MAC GET/SET) ───────────
    /// Current channel (11-26 for 2.4 GHz)
    PhyCurrentChannel = 0x00,
    /// Supported channels bitmask (0x07FFF800 for 2.4 GHz)
    PhyChannelsSupported = 0x01,
    /// TX power in dBm
    PhyTransmitPower = 0x02,
    /// CCA mode
    PhyCcaMode = 0x03,
    /// Channel page (always 0 for 2.4 GHz Zigbee)
    PhyCurrentPage = 0x04,
}

/// Value container for PIB GET/SET operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PibValue {
    Bool(bool),
    U8(u8),
    U16(u16),
    U32(u32),
    I8(i8),
    ShortAddress(ShortAddress),
    PanId(PanId),
    ExtendedAddress(IeeeAddress),
    /// Variable-length beacon payload (max 52 bytes for Zigbee)
    Payload(PibPayload),
}

/// Fixed-capacity beacon payload buffer
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PibPayload {
    buf: [u8; 52],
    len: usize,
}

impl PibPayload {
    pub fn new() -> Self {
        Self {
            buf: [0u8; 52],
            len: 0,
        }
    }

    pub fn from_slice(data: &[u8]) -> Option<Self> {
        if data.len() > 52 {
            return None;
        }
        let mut p = Self::new();
        p.buf[..data.len()].copy_from_slice(data);
        p.len = data.len();
        Some(p)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl Default for PibPayload {
    fn default() -> Self {
        Self::new()
    }
}

/// Invalid PIB update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PibError {
    InvalidValue,
    ReadOnly,
}

/// Portable IEEE 802.15.4 MAC and PHY PIB state.
///
/// Hardware side effects such as channel selection, TX power, and address
/// filter programming are deliberately left to the software-MAC wrapper.
#[derive(Debug, Clone)]
pub struct MacPib {
    short_address: ShortAddress,
    pan_id: PanId,
    extended_address: IeeeAddress,
    coord_short_address: ShortAddress,
    coord_extended_address: IeeeAddress,
    associated_pan_coord: bool,
    rx_on_when_idle: bool,
    association_permit: bool,
    beacon_order: u8,
    superframe_order: u8,
    beacon_payload: PibPayload,
    auto_request: bool,
    max_csma_backoffs: u8,
    min_be: u8,
    max_be: u8,
    max_frame_retries: u8,
    max_frame_total_wait_time: u32,
    response_wait_time: u8,
    dsn: u8,
    bsn: u8,
    transaction_persistence_time: u16,
    promiscuous: bool,
    current_channel: u8,
    transmit_power: i8,
}

impl MacPib {
    pub fn new(extended_address: IeeeAddress, dsn: u8, bsn: u8) -> Self {
        Self {
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            extended_address,
            coord_short_address: ShortAddress(0xFFFF),
            coord_extended_address: [0; 8],
            associated_pan_coord: false,
            rx_on_when_idle: false,
            association_permit: false,
            beacon_order: 15,
            superframe_order: 15,
            beacon_payload: PibPayload::new(),
            auto_request: true,
            max_csma_backoffs: 4,
            min_be: 3,
            max_be: 5,
            max_frame_retries: 3,
            max_frame_total_wait_time: 0,
            response_wait_time: 32,
            dsn,
            bsn,
            transaction_persistence_time: 0x01F4,
            promiscuous: false,
            current_channel: 11,
            transmit_power: 0,
        }
    }

    pub fn reset(&mut self, dsn: u8, bsn: u8) {
        *self = Self::new(self.extended_address, dsn, bsn);
    }

    pub fn short_address(&self) -> ShortAddress {
        self.short_address
    }

    pub fn pan_id(&self) -> PanId {
        self.pan_id
    }

    pub fn extended_address(&self) -> IeeeAddress {
        self.extended_address
    }

    pub fn coord_short_address(&self) -> ShortAddress {
        self.coord_short_address
    }

    pub fn coord_extended_address(&self) -> IeeeAddress {
        self.coord_extended_address
    }

    pub fn promiscuous(&self) -> bool {
        self.promiscuous
    }

    pub fn current_channel(&self) -> u8 {
        self.current_channel
    }

    pub fn transmit_power(&self) -> i8 {
        self.transmit_power
    }

    pub fn max_csma_backoffs(&self) -> u8 {
        self.max_csma_backoffs
    }

    pub fn min_be(&self) -> u8 {
        self.min_be
    }

    pub fn max_be(&self) -> u8 {
        self.max_be
    }

    pub fn max_frame_retries(&self) -> u8 {
        self.max_frame_retries
    }

    pub fn dsn(&self) -> u8 {
        self.dsn
    }

    pub fn bsn(&self) -> u8 {
        self.bsn
    }

    pub fn get(&self, attr: PibAttribute) -> PibValue {
        use PibAttribute::*;
        match attr {
            MacShortAddress => PibValue::ShortAddress(self.short_address),
            MacPanId => PibValue::PanId(self.pan_id),
            MacExtendedAddress => PibValue::ExtendedAddress(self.extended_address),
            MacCoordShortAddress => PibValue::ShortAddress(self.coord_short_address),
            MacCoordExtendedAddress => PibValue::ExtendedAddress(self.coord_extended_address),
            MacAssociatedPanCoord => PibValue::Bool(self.associated_pan_coord),
            MacRxOnWhenIdle => PibValue::Bool(self.rx_on_when_idle),
            MacAssociationPermit => PibValue::Bool(self.association_permit),
            MacBeaconOrder => PibValue::U8(self.beacon_order),
            MacSuperframeOrder => PibValue::U8(self.superframe_order),
            MacBeaconPayload => PibValue::Payload(self.beacon_payload.clone()),
            MacBeaconPayloadLength => PibValue::U8(self.beacon_payload.len as u8),
            MacAutoRequest => PibValue::Bool(self.auto_request),
            MacMaxCsmaBackoffs => PibValue::U8(self.max_csma_backoffs),
            MacMinBe => PibValue::U8(self.min_be),
            MacMaxBe => PibValue::U8(self.max_be),
            MacMaxFrameRetries => PibValue::U8(self.max_frame_retries),
            MacMaxFrameTotalWaitTime => PibValue::U32(self.max_frame_total_wait_time),
            MacResponseWaitTime => PibValue::U8(self.response_wait_time),
            MacDsn => PibValue::U8(self.dsn),
            MacBsn => PibValue::U8(self.bsn),
            MacTransactionPersistenceTime => PibValue::U16(self.transaction_persistence_time),
            MacPromiscuousMode => PibValue::Bool(self.promiscuous),
            PhyCurrentChannel => PibValue::U8(self.current_channel),
            PhyChannelsSupported => PibValue::U32(ChannelMask::ALL_2_4GHZ.0),
            PhyTransmitPower => PibValue::I8(self.transmit_power),
            PhyCcaMode => PibValue::U8(1),
            PhyCurrentPage => PibValue::U8(0),
        }
    }

    pub fn set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), PibError> {
        use PibAttribute::*;
        match (attr, value) {
            (MacShortAddress, PibValue::ShortAddress(value)) => self.short_address = value,
            (MacPanId, PibValue::PanId(value)) => self.pan_id = value,
            (MacExtendedAddress, PibValue::ExtendedAddress(value)) => {
                self.extended_address = value;
            }
            (MacCoordShortAddress, PibValue::ShortAddress(value)) => {
                self.coord_short_address = value;
            }
            (MacCoordExtendedAddress, PibValue::ExtendedAddress(value)) => {
                self.coord_extended_address = value;
            }
            (MacAssociatedPanCoord, PibValue::Bool(value)) => {
                self.associated_pan_coord = value;
            }
            (MacRxOnWhenIdle, PibValue::Bool(value)) => self.rx_on_when_idle = value,
            (MacAssociationPermit, PibValue::Bool(value)) => {
                self.association_permit = value;
            }
            (MacBeaconOrder, PibValue::U8(value)) if value <= 15 => self.beacon_order = value,
            (MacSuperframeOrder, PibValue::U8(value)) if value <= 15 => {
                self.superframe_order = value;
            }
            (MacBeaconPayload, PibValue::Payload(value)) => self.beacon_payload = value,
            (MacBeaconPayloadLength, PibValue::U8(value))
                if usize::from(value) == self.beacon_payload.len => {}
            (MacAutoRequest, PibValue::Bool(value)) => self.auto_request = value,
            (MacMaxCsmaBackoffs, PibValue::U8(value)) if value <= 5 => {
                self.max_csma_backoffs = value;
            }
            (MacMinBe, PibValue::U8(value)) if value <= self.max_be && value <= 8 => {
                self.min_be = value;
            }
            (MacMaxBe, PibValue::U8(value)) if value >= self.min_be && value <= 8 => {
                self.max_be = value;
            }
            (MacMaxFrameRetries, PibValue::U8(value)) if value <= 7 => {
                self.max_frame_retries = value;
            }
            (MacMaxFrameTotalWaitTime, PibValue::U32(value)) => {
                self.max_frame_total_wait_time = value;
            }
            (MacResponseWaitTime, PibValue::U8(value)) => self.response_wait_time = value,
            (MacDsn, PibValue::U8(value)) => self.dsn = value,
            (MacBsn, PibValue::U8(value)) => self.bsn = value,
            (MacTransactionPersistenceTime, PibValue::U16(value)) => {
                self.transaction_persistence_time = value;
            }
            (MacPromiscuousMode, PibValue::Bool(value)) => self.promiscuous = value,
            (PhyCurrentChannel, PibValue::U8(value)) if (11..=26).contains(&value) => {
                self.current_channel = value;
            }
            (PhyTransmitPower, PibValue::I8(value)) => self.transmit_power = value,
            (PhyCcaMode, PibValue::U8(1)) | (PhyCurrentPage, PibValue::U8(0)) => {}
            (PhyChannelsSupported, _) => return Err(PibError::ReadOnly),
            _ => return Err(PibError::InvalidValue),
        }
        Ok(())
    }

    pub fn next_dsn(&mut self) -> u8 {
        let sequence = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        sequence
    }

    pub fn next_bsn(&mut self) -> u8 {
        let sequence = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        sequence
    }
}

// ── Convenience conversions ─────────────────────────────────────

impl PibValue {
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Self::Bool(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_u8(&self) -> Option<u8> {
        match self {
            Self::U8(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_u16(&self) -> Option<u16> {
        match self {
            Self::U16(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_u32(&self) -> Option<u32> {
        match self {
            Self::U32(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_i8(&self) -> Option<i8> {
        match self {
            Self::I8(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_short_address(&self) -> Option<ShortAddress> {
        match self {
            Self::ShortAddress(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_pan_id(&self) -> Option<PanId> {
        match self {
            Self::PanId(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_extended_address(&self) -> Option<IeeeAddress> {
        match self {
            Self::ExtendedAddress(v) => Some(*v),
            _ => None,
        }
    }
}

// ── PHY constants ───────────────────────────────────────────────

/// Base superframe duration in symbols (960)
pub const A_BASE_SUPERFRAME_DURATION: u32 = 960;

/// Symbol rate at 2.4 GHz in symbols/second
pub const SYMBOL_RATE_2_4GHZ: u32 = 62_500;

/// Calculate scan duration per channel in symbols
pub fn scan_duration_symbols(exponent: u8) -> u32 {
    A_BASE_SUPERFRAME_DURATION * ((1u32 << exponent) + 1)
}

/// Calculate scan duration per channel in microseconds
pub fn scan_duration_us(exponent: u8) -> u64 {
    let symbols = scan_duration_symbols(exponent) as u64;
    symbols * 1_000_000 / SYMBOL_RATE_2_4GHZ as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    const IEEE: IeeeAddress = [0x02, 1, 2, 3, 4, 5, 6, 7];

    #[test]
    fn mac_pib_uses_zigbee_defaults() {
        let pib = MacPib::new(IEEE, 0x12, 0x34);

        assert_eq!(
            pib.get(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            pib.get(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            pib.get(PibAttribute::MacExtendedAddress),
            PibValue::ExtendedAddress(IEEE)
        );
        assert_eq!(pib.get(PibAttribute::MacBeaconOrder), PibValue::U8(15));
        assert_eq!(pib.get(PibAttribute::MacSuperframeOrder), PibValue::U8(15));
        assert_eq!(pib.get(PibAttribute::MacMaxCsmaBackoffs), PibValue::U8(4));
        assert_eq!(pib.get(PibAttribute::MacMinBe), PibValue::U8(3));
        assert_eq!(pib.get(PibAttribute::MacMaxBe), PibValue::U8(5));
        assert_eq!(pib.get(PibAttribute::MacMaxFrameRetries), PibValue::U8(3));
        assert_eq!(pib.get(PibAttribute::MacResponseWaitTime), PibValue::U8(32));
        assert_eq!(
            pib.get(PibAttribute::MacTransactionPersistenceTime),
            PibValue::U16(0x01F4)
        );
        assert_eq!(pib.get(PibAttribute::PhyCurrentChannel), PibValue::U8(11));
        assert_eq!(pib.get(PibAttribute::MacDsn), PibValue::U8(0x12));
        assert_eq!(pib.get(PibAttribute::MacBsn), PibValue::U8(0x34));
    }

    #[test]
    fn mac_pib_round_trips_writable_values() {
        let mut pib = MacPib::new(IEEE, 0, 0);
        let updates = [
            (
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(ShortAddress(0x1234)),
            ),
            (PibAttribute::MacPanId, PibValue::PanId(PanId(0x5678))),
            (PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true)),
            (PibAttribute::MacMaxCsmaBackoffs, PibValue::U8(5)),
            (PibAttribute::MacMinBe, PibValue::U8(2)),
            (PibAttribute::MacMaxBe, PibValue::U8(7)),
            (PibAttribute::MacMaxFrameRetries, PibValue::U8(6)),
            (PibAttribute::PhyCurrentChannel, PibValue::U8(26)),
            (PibAttribute::PhyTransmitPower, PibValue::I8(-8)),
        ];

        for (attribute, value) in updates {
            pib.set(attribute, value.clone()).unwrap();
            assert_eq!(pib.get(attribute), value);
        }
    }

    #[test]
    fn mac_pib_rejects_invalid_or_read_only_values() {
        let mut pib = MacPib::new(IEEE, 0, 0);
        assert_eq!(
            pib.set(PibAttribute::PhyCurrentChannel, PibValue::U8(10)),
            Err(PibError::InvalidValue)
        );
        assert_eq!(
            pib.set(PibAttribute::MacMinBe, PibValue::U8(6)),
            Err(PibError::InvalidValue)
        );
        assert_eq!(
            pib.set(
                PibAttribute::PhyChannelsSupported,
                PibValue::U32(ChannelMask::ALL_2_4GHZ.0)
            ),
            Err(PibError::ReadOnly)
        );
    }

    #[test]
    fn mac_pib_reset_preserves_ieee_and_reseeds_sequences() {
        let mut pib = MacPib::new(IEEE, u8::MAX, u8::MAX);
        pib.set(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x1234)),
        )
        .unwrap();
        assert_eq!(pib.next_dsn(), u8::MAX);
        assert_eq!(pib.next_dsn(), 0);

        pib.reset(7, 9);

        assert_eq!(
            pib.get(PibAttribute::MacExtendedAddress),
            PibValue::ExtendedAddress(IEEE)
        );
        assert_eq!(
            pib.get(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(pib.next_dsn(), 7);
        assert_eq!(pib.next_bsn(), 9);
    }
}
