//! Shared enumerations: link endpoints, message status codes and IEEE
//! addresses.

use crate::codec::{CodecError, Reader, Wire, Writer};

/// Link endpoint (spec 5.10). Not a Zigbee endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Endpoint {
    /// Never used for DATA.
    Link = 0,
    /// Session, negotiation, config, diagnostics (priority 1).
    System = 1,
    /// Network, data, ZDO, security, GP (priority 3).
    Zigbee = 2,
    /// Hybrid table paging (priority 2).
    Tables = 3,
    /// Backup and NCP firmware update (priority 4).
    Bulk = 4,
    /// Log records (priority 5, lowest).
    Log = 5,
}

impl Endpoint {
    /// Every DATA endpoint, in ID order.
    pub const DATA: [Endpoint; 5] = [
        Self::System,
        Self::Zigbee,
        Self::Tables,
        Self::Bulk,
        Self::Log,
    ];

    /// Number of endpoint IDs tracked by the link (0..=5).
    pub const COUNT: usize = 6;

    /// Decode an endpoint ID; reserved and experimental IDs map to `None`.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Link,
            1 => Self::System,
            2 => Self::Zigbee,
            3 => Self::Tables,
            4 => Self::Bulk,
            5 => Self::Log,
            _ => return None,
        })
    }

    /// Scheduler priority (1 = highest); `Link` has none (0).
    pub const fn priority(self) -> u8 {
        match self {
            Self::Link => 0,
            Self::System => 1,
            Self::Tables => 2,
            Self::Zigbee => 3,
            Self::Bulk => 4,
            Self::Log => 5,
        }
    }

    /// `true` for the endpoints that share the `max(1, W/2)` slot quota.
    pub const fn is_bulk_class(self) -> bool {
        matches!(self, Self::Bulk | Self::Log)
    }
}

/// Message-level status (spec 6.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Status {
    /// `0x00` Done, or accepted for a two-phase command.
    Success,
    /// `0x01` Generic failure.
    Failure,
    /// `0x02` Queue or resource full; nothing was done.
    Busy,
    /// `0x03` Malformed or out-of-range payload.
    InvalidParameter,
    /// `0x04` Unknown `msg_id` or capability not present.
    UnsupportedCommand,
    /// `0x05` Not allowed now.
    InvalidState,
    /// `0x06` Unknown entry.
    NotFound,
    /// `0x07` Table full.
    NoSpace,
    /// `0x08` An internal deadline expired.
    Timeout,
    /// `0x09` Host-local: the session reset before the response arrived.
    LinkReset,
    /// `0x0A` Policy forbids it.
    AccessDenied,
    /// `0x0B` No network in NV.
    NotFormed,
    /// `0x0C` Duplicate entry.
    AlreadyExists,
    /// `0x0D` NV write or commit failed.
    StorageError,
    /// `0x0E` Key, counter or authentication rule violated.
    SecurityError,
    /// `0x0F` No common protocol major version.
    VersionMismatch,
    /// `0x10` Another exclusive operation is running.
    InProgress,
    /// `0x11` Payload exceeds the NCP limit.
    TooLarge,
    /// `0x12` Command sent on the wrong link endpoint.
    WrongEndpoint,
    /// `0x13` Host push conflicts with a newer dirty NCP entry.
    Conflict,
    /// `0x14` Cancelled by reset, leave or restore.
    Aborted,
    /// `0x15` Host-local: the carrying frame was REJECTed.
    LinkRejected,
    /// Any other value (kept for forward compatibility).
    Other(u8),
}

impl Status {
    /// Decode a status byte.
    pub const fn from_u8(v: u8) -> Self {
        match v {
            0x00 => Self::Success,
            0x01 => Self::Failure,
            0x02 => Self::Busy,
            0x03 => Self::InvalidParameter,
            0x04 => Self::UnsupportedCommand,
            0x05 => Self::InvalidState,
            0x06 => Self::NotFound,
            0x07 => Self::NoSpace,
            0x08 => Self::Timeout,
            0x09 => Self::LinkReset,
            0x0A => Self::AccessDenied,
            0x0B => Self::NotFormed,
            0x0C => Self::AlreadyExists,
            0x0D => Self::StorageError,
            0x0E => Self::SecurityError,
            0x0F => Self::VersionMismatch,
            0x10 => Self::InProgress,
            0x11 => Self::TooLarge,
            0x12 => Self::WrongEndpoint,
            0x13 => Self::Conflict,
            0x14 => Self::Aborted,
            0x15 => Self::LinkRejected,
            v => Self::Other(v),
        }
    }

    /// The status byte.
    pub const fn to_u8(self) -> u8 {
        match self {
            Self::Success => 0x00,
            Self::Failure => 0x01,
            Self::Busy => 0x02,
            Self::InvalidParameter => 0x03,
            Self::UnsupportedCommand => 0x04,
            Self::InvalidState => 0x05,
            Self::NotFound => 0x06,
            Self::NoSpace => 0x07,
            Self::Timeout => 0x08,
            Self::LinkReset => 0x09,
            Self::AccessDenied => 0x0A,
            Self::NotFormed => 0x0B,
            Self::AlreadyExists => 0x0C,
            Self::StorageError => 0x0D,
            Self::SecurityError => 0x0E,
            Self::VersionMismatch => 0x0F,
            Self::InProgress => 0x10,
            Self::TooLarge => 0x11,
            Self::WrongEndpoint => 0x12,
            Self::Conflict => 0x13,
            Self::Aborted => 0x14,
            Self::LinkRejected => 0x15,
            Self::Other(v) => v,
        }
    }

    /// `true` for [`Status::Success`].
    pub const fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }
}

impl<'a> Wire<'a> for Status {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(Self::from_u8(r.u8()?))
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.to_u8())
    }
}

/// IEEE (EUI-64) address in on-air order (least significant byte first),
/// exactly as in a MAC header.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default, PartialOrd, Ord)]
pub struct Ieee(pub [u8; 8]);

impl Ieee {
    /// All `0xFF` (used by `SYS_SET_IEEE` to revert to the factory address).
    pub const BROADCAST: Self = Self([0xFF; 8]);

    /// From the conventional big-endian `u64` (`0x00158d0004c5d6e7`).
    pub const fn from_u64(v: u64) -> Self {
        Self(v.to_le_bytes())
    }

    /// As the conventional big-endian `u64`.
    pub const fn to_u64(self) -> u64 {
        u64::from_le_bytes(self.0)
    }
}

impl core::fmt::Debug for Ieee {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Ieee({:016x})", self.to_u64())
    }
}

impl<'a> Wire<'a> for Ieee {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(Self(r.array()?))
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.bytes(&self.0)
    }
}

/// 128-bit key in AES input order.
pub type Key = [u8; 16];
