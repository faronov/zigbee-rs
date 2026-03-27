//! Zigbee Cluster Library (ZCL) implementation for Zigbee PRO R22 / ZCL Rev 8.
//!
//! This crate provides a `#![no_std]` ZCL layer with frame parsing, foundation
//! commands, data types, attribute storage, reporting, and standard cluster
//! implementations. All collections use `heapless` — no heap allocation.

#![no_std]

pub mod attribute;
pub mod clusters;
pub mod data_types;
pub mod foundation;
pub mod frame;
pub mod reporting;
pub mod transition;

/// A 16-bit Zigbee Cluster identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ClusterId(pub u16);

impl ClusterId {
    // General
    pub const BASIC: Self = Self(0x0000);
    pub const POWER_CONFIG: Self = Self(0x0001);
    pub const DEVICE_TEMP_CONFIG: Self = Self(0x0002);
    pub const IDENTIFY: Self = Self(0x0003);
    pub const GROUPS: Self = Self(0x0004);
    pub const SCENES: Self = Self(0x0005);
    pub const ON_OFF: Self = Self(0x0006);
    pub const ON_OFF_SWITCH_CONFIG: Self = Self(0x0007);
    pub const LEVEL_CONTROL: Self = Self(0x0008);
    pub const ALARMS: Self = Self(0x0009);
    pub const TIME: Self = Self(0x000A);
    pub const ANALOG_INPUT: Self = Self(0x000C);
    pub const ANALOG_OUTPUT: Self = Self(0x000D);
    pub const ANALOG_VALUE: Self = Self(0x000E);
    pub const BINARY_INPUT: Self = Self(0x000F);
    pub const BINARY_OUTPUT: Self = Self(0x0010);
    pub const BINARY_VALUE: Self = Self(0x0011);
    pub const MULTISTATE_INPUT: Self = Self(0x0012);
    pub const OTA_UPGRADE: Self = Self(0x0019);
    pub const POLL_CONTROL: Self = Self(0x0020);
    pub const GREEN_POWER: Self = Self(0x0021);
    // Closures
    pub const DOOR_LOCK: Self = Self(0x0101);
    pub const WINDOW_COVERING: Self = Self(0x0102);
    // HVAC
    pub const THERMOSTAT: Self = Self(0x0201);
    pub const FAN_CONTROL: Self = Self(0x0202);
    pub const THERMOSTAT_UI: Self = Self(0x0204);
    // Lighting
    pub const COLOR_CONTROL: Self = Self(0x0300);
    pub const BALLAST_CONFIG: Self = Self(0x0301);
    // Measurement & Sensing
    pub const ILLUMINANCE: Self = Self(0x0400);
    pub const ILLUMINANCE_LEVEL_SENSING: Self = Self(0x0401);
    pub const TEMPERATURE: Self = Self(0x0402);
    pub const PRESSURE: Self = Self(0x0403);
    pub const FLOW_MEASUREMENT: Self = Self(0x0404);
    pub const HUMIDITY: Self = Self(0x0405);
    pub const OCCUPANCY: Self = Self(0x0406);
    pub const SOIL_MOISTURE: Self = Self(0x0408);
    pub const CARBON_DIOXIDE: Self = Self(0x040D);
    pub const PM25_MEASUREMENT: Self = Self(0x042A);
    // Security
    pub const IAS_ZONE: Self = Self(0x0500);
    pub const IAS_ACE: Self = Self(0x0501);
    pub const IAS_WD: Self = Self(0x0502);
    // Smart Energy
    pub const METERING: Self = Self(0x0702);
    pub const ELECTRICAL_MEASUREMENT: Self = Self(0x0B04);
    pub const DIAGNOSTICS: Self = Self(0x0B05);
    pub const TOUCHLINK: Self = Self(0x1000);
}

/// A 16-bit ZCL attribute identifier, scoped to a cluster.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AttributeId(pub u16);

/// An 8-bit ZCL command identifier, scoped to a cluster.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandId(pub u8);

/// Direction of a cluster command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClusterDirection {
    /// Client-to-server (e.g. a remote sending On/Off to a light).
    ClientToServer,
    /// Server-to-client (e.g. a light reporting its state).
    ServerToClient,
}

/// Standard ZCL status codes (ZCL Rev 8, Table 2-12).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ZclStatus {
    Success = 0x00,
    Failure = 0x01,
    NotAuthorized = 0x7E,
    ReservedFieldNotZero = 0x7F,
    MalformedCommand = 0x80,
    UnsupClusterCommand = 0x81,
    UnsupGeneralCommand = 0x82,
    UnsupManufacturerClusterCommand = 0x83,
    UnsupManufacturerGeneralCommand = 0x84,
    InvalidField = 0x85,
    UnsupportedAttribute = 0x86,
    InvalidValue = 0x87,
    ReadOnly = 0x88,
    InsufficientSpace = 0x89,
    DuplicateExists = 0x8A,
    NotFound = 0x8B,
    UnreportableAttribute = 0x8C,
    InvalidDataType = 0x8D,
    InvalidSelector = 0x8E,
    WriteOnly = 0x8F,
    InconsistentStartupState = 0x90,
    DefinedOutOfBand = 0x91,
    Inconsistent = 0x92,
    ActionDenied = 0x93,
    Timeout = 0x94,
    Abort = 0x95,
    InvalidImage = 0x96,
    WaitForData = 0x97,
    NoImageAvailable = 0x98,
    RequireMoreImage = 0x99,
    NotificationPending = 0x9A,
    HardwareFailure = 0xC0,
    SoftwareFailure = 0xC1,
    CalibrationError = 0xC2,
    UnsupportedCluster = 0xC3,
}

impl ZclStatus {
    /// Create a `ZclStatus` from its wire value.
    pub fn from_u8(val: u8) -> Self {
        match val {
            0x00 => Self::Success,
            0x01 => Self::Failure,
            0x7E => Self::NotAuthorized,
            0x7F => Self::ReservedFieldNotZero,
            0x80 => Self::MalformedCommand,
            0x81 => Self::UnsupClusterCommand,
            0x82 => Self::UnsupGeneralCommand,
            0x83 => Self::UnsupManufacturerClusterCommand,
            0x84 => Self::UnsupManufacturerGeneralCommand,
            0x85 => Self::InvalidField,
            0x86 => Self::UnsupportedAttribute,
            0x87 => Self::InvalidValue,
            0x88 => Self::ReadOnly,
            0x89 => Self::InsufficientSpace,
            0x8A => Self::DuplicateExists,
            0x8B => Self::NotFound,
            0x8C => Self::UnreportableAttribute,
            0x8D => Self::InvalidDataType,
            0x8E => Self::InvalidSelector,
            0x8F => Self::WriteOnly,
            0x90 => Self::InconsistentStartupState,
            0x91 => Self::DefinedOutOfBand,
            0x92 => Self::Inconsistent,
            0x93 => Self::ActionDenied,
            0x94 => Self::Timeout,
            0x95 => Self::Abort,
            0x96 => Self::InvalidImage,
            0x97 => Self::WaitForData,
            0x98 => Self::NoImageAvailable,
            0x99 => Self::RequireMoreImage,
            0x9A => Self::NotificationPending,
            0xC0 => Self::HardwareFailure,
            0xC1 => Self::SoftwareFailure,
            0xC2 => Self::CalibrationError,
            0xC3 => Self::UnsupportedCluster,
            _ => Self::Failure,
        }
    }
}
