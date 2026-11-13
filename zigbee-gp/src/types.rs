//! Green Power identifiers, enumerations and alias derivation.

use crate::{Error, Result};
pub use zigbee_types::IeeeAddress;

/// GPD ApplicationID (Extended NWK Frame Control bits 0-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApplicationId {
    /// GPD identified by a 32-bit SrcID.
    SrcId = 0b000,
    /// Low-power end device (LPED); not processed by this crate.
    Lped = 0b001,
    /// GPD identified by its IEEE address and an endpoint.
    Ieee = 0b010,
}

impl ApplicationId {
    /// Decode a 3-bit ApplicationID.
    pub fn from_bits(bits: u8) -> Result<Self> {
        match bits & 0x07 {
            0b000 => Ok(Self::SrcId),
            0b001 => Ok(Self::Lped),
            0b010 => Ok(Self::Ieee),
            _ => Err(Error::Unsupported),
        }
    }
}

/// Identity of a Green Power Device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpdId {
    /// ApplicationID 0b000.
    SrcId(u32),
    /// ApplicationID 0b010.
    Ieee {
        /// GPD IEEE address (little-endian as on air).
        ieee: IeeeAddress,
        /// GPD endpoint.
        endpoint: u8,
    },
}

/// SrcID value meaning "unspecified" (used e.g. by Channel Request).
pub const SRC_ID_UNSPECIFIED: u32 = 0x0000_0000;
/// SrcID value meaning "all".
pub const SRC_ID_ALL: u32 = 0xFFFF_FFFF;

impl GpdId {
    /// ApplicationID of this identifier.
    pub fn application_id(&self) -> ApplicationId {
        match self {
            Self::SrcId(_) => ApplicationId::SrcId,
            Self::Ieee { .. } => ApplicationId::Ieee,
        }
    }

    /// True for SrcIDs that are reserved and must not be commissioned
    /// (0, 0xFFFFFFF9-0xFFFFFFFF).
    pub fn is_reserved(&self) -> bool {
        match *self {
            Self::SrcId(id) => id == SRC_ID_UNSPECIFIED || id >= 0xFFFF_FFF9,
            Self::Ieee { ieee, .. } => ieee == [0; 8] || ieee == [0xFF; 8],
        }
    }

    /// Lower 32 bits of the identifier (SrcID or the 4 LSB of the IEEE
    /// address), used for alias derivation and TC-LK key wrapping.
    pub fn low32(&self) -> u32 {
        match *self {
            Self::SrcId(id) => id,
            Self::Ieee { ieee, .. } => u32::from_le_bytes([ieee[0], ieee[1], ieee[2], ieee[3]]),
        }
    }

    /// Identity match per GP spec: IEEE endpoints 0x00 and 0xFF are
    /// wildcards.
    pub fn matches(&self, other: &GpdId) -> bool {
        match (self, other) {
            (Self::SrcId(a), Self::SrcId(b)) => a == b,
            (
                Self::Ieee {
                    ieee: a,
                    endpoint: ea,
                },
                Self::Ieee {
                    ieee: b,
                    endpoint: eb,
                },
            ) => a == b && (ea == eb || matches!(*ea, 0x00 | 0xFF) || matches!(*eb, 0x00 | 0xFF)),
            _ => false,
        }
    }

    /// Encoded length of the GPD ID field in GP cluster commands
    /// (4 for SrcID, 9 for IEEE + endpoint).
    pub fn wire_len(&self) -> usize {
        match self {
            Self::SrcId(_) => 4,
            Self::Ieee { .. } => 9,
        }
    }
}

/// GP security level (Extended NWK Frame Control bits 3-4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SecurityLevel {
    /// No security.
    None = 0b00,
    /// Deprecated short-MIC level; reserved since GP 1.1 and rejected.
    Reserved = 0b01,
    /// Full 4-byte frame counter and 4-byte MIC.
    FullCounterMic = 0b10,
    /// Encryption plus full frame counter and MIC.
    Encrypted = 0b11,
}

impl SecurityLevel {
    /// Decode the 2-bit field.
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::None,
            1 => Self::Reserved,
            2 => Self::FullCounterMic,
            _ => Self::Encrypted,
        }
    }

    /// Whether frames at this level carry a 4-byte counter and MIC.
    pub fn is_protected(self) -> bool {
        matches!(self, Self::FullCounterMic | Self::Encrypted)
    }
}

/// GP security key type (3 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KeyType {
    /// No key.
    NoKey = 0b000,
    /// Zigbee network key.
    NwkKey = 0b001,
    /// GPD group key.
    GpdGroupKey = 0b010,
    /// NWK-key derived GPD group key.
    NwkKeyDerivedGroupKey = 0b011,
    /// Out-of-the-box individual GPD key.
    IndividualKey = 0b100,
    /// Individual key derived from the GPD group key.
    DerivedIndividualKey = 0b111,
}

impl KeyType {
    /// Decode the 3-bit field; reserved values (0b101, 0b110) are rejected.
    pub fn from_bits(bits: u8) -> Result<Self> {
        match bits & 0x07 {
            0b000 => Ok(Self::NoKey),
            0b001 => Ok(Self::NwkKey),
            0b010 => Ok(Self::GpdGroupKey),
            0b011 => Ok(Self::NwkKeyDerivedGroupKey),
            0b100 => Ok(Self::IndividualKey),
            0b111 => Ok(Self::DerivedIndividualKey),
            _ => Err(Error::Unsupported),
        }
    }

    /// Value of the 1-bit SecurityKey sub-field of the Extended NWK FC that
    /// corresponds to this key type (1 = individual key).
    pub fn is_individual(self) -> bool {
        matches!(self, Self::IndividualKey | Self::DerivedIndividualKey)
    }
}

/// GP communication mode (Pairing options / sink tables).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommunicationMode {
    /// Full unicast with GP Notification Response.
    FullUnicast = 0b00,
    /// Groupcast to the GPD-ID derived group.
    DerivedGroupcast = 0b01,
    /// Groupcast to a pre-commissioned group.
    CommissionedGroupcast = 0b10,
    /// Lightweight unicast (no response, no tunnelling delay).
    LightweightUnicast = 0b11,
}

impl CommunicationMode {
    /// Decode the 2-bit field.
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::FullUnicast,
            1 => Self::DerivedGroupcast,
            2 => Self::CommissionedGroupcast,
            _ => Self::LightweightUnicast,
        }
    }

    /// Whether this is a unicast mode (carries sink IEEE + NWK).
    pub fn is_unicast(self) -> bool {
        matches!(self, Self::FullUnicast | Self::LightweightUnicast)
    }
}

/// GPD command identifiers (GP spec Table 48/49).
pub mod cmd {
    /// Identify.
    pub const IDENTIFY: u8 = 0x00;
    /// Off.
    pub const OFF: u8 = 0x20;
    /// On.
    pub const ON: u8 = 0x21;
    /// Toggle.
    pub const TOGGLE: u8 = 0x22;
    /// Release.
    pub const RELEASE: u8 = 0x23;
    /// Press 1 of 1.
    pub const PRESS_1_OF_1: u8 = 0x60;
    /// Release 1 of 1.
    pub const RELEASE_1_OF_1: u8 = 0x61;
    /// Attribute reporting.
    pub const ATTRIBUTE_REPORTING: u8 = 0xA0;
    /// Manufacturer-specific attribute reporting.
    pub const MFR_ATTRIBUTE_REPORTING: u8 = 0xA1;
    /// Multi-cluster reporting.
    pub const MULTI_CLUSTER_REPORTING: u8 = 0xA2;
    /// Manufacturer-specific multi-cluster reporting.
    pub const MFR_MULTI_CLUSTER_REPORTING: u8 = 0xA3;
    /// Commissioning (GPD → GPP).
    pub const COMMISSIONING: u8 = 0xE0;
    /// Decommissioning (GPD → GPP).
    pub const DECOMMISSIONING: u8 = 0xE1;
    /// Success (GPD → GPP).
    pub const SUCCESS: u8 = 0xE2;
    /// Channel Request (GPD → GPP).
    pub const CHANNEL_REQUEST: u8 = 0xE3;
    /// Application Description (GPD → GPP).
    pub const APPLICATION_DESCRIPTION: u8 = 0xE4;
    /// Commissioning Reply (GPP → GPD).
    pub const COMMISSIONING_REPLY: u8 = 0xF0;
    /// Write Attributes (GPP → GPD).
    pub const WRITE_ATTRIBUTES: u8 = 0xF1;
    /// Read Attributes (GPP → GPD).
    pub const READ_ATTRIBUTES: u8 = 0xF2;
    /// Channel Configuration (GPP → GPD).
    pub const CHANNEL_CONFIGURATION: u8 = 0xF3;
    /// ZCL tunnelling to the GPD.
    pub const ZCL_TUNNELING_TO_GPD: u8 = 0xF6;

    /// Whether `id` is a commissioning-related command handled only in
    /// commissioning mode (0xE0-0xE3).
    pub fn is_commissioning(id: u8) -> bool {
        matches!(
            id,
            COMMISSIONING | DECOMMISSIONING | SUCCESS | CHANNEL_REQUEST
        )
    }

    /// Whether `id` belongs to the GPP → GPD range (0xF0-0xFF).
    pub fn is_to_gpd(id: u8) -> bool {
        id >= 0xF0
    }
}

/// GPD device identifiers (GP spec Table 51).
pub mod device_id {
    /// Simple generic 1-state switch.
    pub const SIMPLE_GENERIC_1_STATE_SWITCH: u8 = 0x00;
    /// On/off switch.
    pub const ON_OFF_SWITCH: u8 = 0x02;
    /// Level control switch.
    pub const LEVEL_CONTROL_SWITCH: u8 = 0x03;
    /// Simple sensor.
    pub const SIMPLE_SENSOR: u8 = 0x04;
    /// Temperature sensor.
    pub const TEMPERATURE_SENSOR: u8 = 0x30;
    /// Indoor environment sensor (temperature + humidity …).
    pub const INDOOR_ENVIRONMENT_SENSOR: u8 = 0x33;
    /// Manufacturer specific / undefined.
    pub const MANUFACTURER_SPECIFIC: u8 = 0xFE;
}

/// Derive the NWK alias source address for a GPD (GP spec A.3.6.3.3.1).
pub fn derive_alias(gpd: &GpdId) -> u16 {
    let id = gpd.low32();
    let lo = (id & 0xFFFF) as u16;
    if lo != 0 && lo <= 0xFFF7 {
        return lo;
    }
    let xored = lo ^ ((id >> 16) as u16);
    if xored != 0 && xored <= 0xFFF7 {
        return xored;
    }
    if lo == 0 { 0x0007 } else { lo.wrapping_sub(8) }
}

/// Synthetic IEEE address used by sinks (zigpy) for SrcID GPDs: the SrcID
/// little-endian followed by four zero bytes.
pub fn src_id_ieee(src_id: u32) -> IeeeAddress {
    let b = src_id.to_le_bytes();
    [b[0], b[1], b[2], b[3], 0, 0, 0, 0]
}

/// Alias sequence-number offsets, subtracted (mod 256) from the triggering
/// GPDF MAC sequence number. The result is used both as the alias NWK
/// sequence number and as the APS counter.
///
/// Source: GP Basic 1.1.2 (CSA 14-0563-19) A.3.6.3.3.2. GP 1.0a (095499r26)
/// had 13 for the commissioned groupcast GP Notification; 1.1 changed it to
/// 9, which EmberZNet (`EMBER_GP_NOTIFICATION_COMMISSIONED_GROUPCAST_
/// SEQUENCE_NUMBER_OFFSET`) also uses.
pub mod alias_seq {
    /// Derived groupcast GP Notification.
    pub const DERIVED_GROUPCAST: u8 = 0;
    /// GP Pairing Search.
    pub const PAIRING_SEARCH: u8 = 10;
    /// GP Tunneling Stop.
    pub const TUNNELING_STOP: u8 = 11;
    /// GP Commissioning Notification.
    pub const COMMISSIONING_NOTIFICATION: u8 = 12;
    /// Commissioned groupcast GP Notification.
    pub const COMMISSIONED_GROUPCAST: u8 = 9;
    /// Broadcast GP Notification.
    pub const BROADCAST_NOTIFICATION: u8 = 14;

    /// Alias sequence number for `offset` given the GPDF MAC sequence.
    pub fn derive(mac_seq: u8, offset: u8) -> u8 {
        mac_seq.wrapping_sub(offset)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// GP Basic 1.1.2 A.3.6.3.3.2, one line per forwarded command type.
    #[test]
    fn alias_sequence_offsets_match_gp_1_1_2() {
        use alias_seq::*;
        let mac_seq = 0x05;
        assert_eq!(derive(mac_seq, DERIVED_GROUPCAST), 0x05);
        assert_eq!(derive(mac_seq, PAIRING_SEARCH), 0xFB);
        assert_eq!(derive(mac_seq, TUNNELING_STOP), 0xFA);
        assert_eq!(derive(mac_seq, COMMISSIONING_NOTIFICATION), 0xF9);
        assert_eq!(derive(mac_seq, COMMISSIONED_GROUPCAST), 0xFC);
        assert_eq!(derive(mac_seq, BROADCAST_NOTIFICATION), 0xF7);
    }

    #[test]
    fn alias_derivation_matches_spec_rules() {
        assert_eq!(derive_alias(&GpdId::SrcId(0x1234_5678)), 0x5678);
        // Low half 0 -> XOR with upper half.
        assert_eq!(derive_alias(&GpdId::SrcId(0x1234_0000)), 0x1234);
        // Low half reserved, XOR also reserved/zero.
        assert_eq!(derive_alias(&GpdId::SrcId(0x0000_0000)), 0x0007);
        // 0xFFF8 ^ 0xFFFF = 0x0007 (valid).
        assert_eq!(derive_alias(&GpdId::SrcId(0xFFFF_FFF8)), 0x0007);
        // 0xFFFF ^ 0x0000 = 0xFFFF (reserved) -> 0xFFFF - 8.
        assert_eq!(derive_alias(&GpdId::SrcId(0x0000_FFFF)), 0xFFF7);
        // 0xFFFA ^ 0xFFFA = 0 -> 0xFFFA - 8.
        assert_eq!(derive_alias(&GpdId::SrcId(0xFFFA_FFFA)), 0xFFF2);
    }

    #[test]
    fn ieee_endpoint_wildcards() {
        let a = GpdId::Ieee {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            endpoint: 1,
        };
        let any = GpdId::Ieee {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            endpoint: 0xFF,
        };
        let other = GpdId::Ieee {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            endpoint: 2,
        };
        assert!(a.matches(&any));
        assert!(!a.matches(&other));
        assert!(!a.matches(&GpdId::SrcId(1)));
    }
}
