//! Green Power cluster (0x0021).
//!
//! Enables energy-harvesting devices (e.g. EnOcean switches) that send single-frame
//! Green Power Data Frames (GPDFs) without joining the network. Supports Proxy
//! (router), Sink (coordinator), and Combined roles.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// ---------------------------------------------------------------------------
// Proxy-side attribute IDs
// ---------------------------------------------------------------------------
pub const ATTR_GPP_MAX_PROXY_TABLE_ENTRIES: AttributeId = AttributeId(0x0010);
pub const ATTR_PROXY_TABLE: AttributeId = AttributeId(0x0011);
pub const ATTR_GPP_NOTIFICATION_RETRY_NUMBER: AttributeId = AttributeId(0x0012);
pub const ATTR_GPP_NOTIFICATION_RETRY_TIMER: AttributeId = AttributeId(0x0013);
pub const ATTR_GPP_MAX_SEARCH_COUNTER: AttributeId = AttributeId(0x0014);
pub const ATTR_GPP_BLOCKED_GPD_ID: AttributeId = AttributeId(0x0015);
pub const ATTR_GPP_FUNCTIONALITY: AttributeId = AttributeId(0x0016);
pub const ATTR_GPP_ACTIVE_FUNCTIONALITY: AttributeId = AttributeId(0x0017);

// ---------------------------------------------------------------------------
// Sink-side attribute IDs
// ---------------------------------------------------------------------------
pub const ATTR_GPS_SINK_TABLE: AttributeId = AttributeId(0x0000);
pub const ATTR_GPS_COMMUNICATION_MODE: AttributeId = AttributeId(0x0001);
pub const ATTR_GPS_COMMISSIONING_EXIT_MODE: AttributeId = AttributeId(0x0002);
pub const ATTR_GPS_COMMISSIONING_WINDOW: AttributeId = AttributeId(0x0003);
pub const ATTR_GPS_SECURITY_LEVEL: AttributeId = AttributeId(0x0004);
pub const ATTR_GPS_FUNCTIONALITY: AttributeId = AttributeId(0x0005);
pub const ATTR_GPS_ACTIVE_FUNCTIONALITY: AttributeId = AttributeId(0x0006);

// ---------------------------------------------------------------------------
// Client → Server commands (GP Proxy → Sink)
// ---------------------------------------------------------------------------
pub const CMD_GP_NOTIFICATION: CommandId = CommandId(0x00);
pub const CMD_GP_PAIRING_SEARCH: CommandId = CommandId(0x01);
pub const CMD_GP_TUNNELING_STOP: CommandId = CommandId(0x03);
pub const CMD_GP_COMMISSIONING_NOTIFICATION: CommandId = CommandId(0x04);
pub const CMD_GP_TRANSLATION_TABLE_UPDATE: CommandId = CommandId(0x07);
pub const CMD_GP_TRANSLATION_TABLE_REQUEST: CommandId = CommandId(0x08);
pub const CMD_GP_PAIRING_CONFIGURATION: CommandId = CommandId(0x09);
pub const CMD_GP_SINK_TABLE_REQUEST: CommandId = CommandId(0x0A);
pub const CMD_GP_PROXY_TABLE_RESPONSE: CommandId = CommandId(0x0B);

// ---------------------------------------------------------------------------
// Server → Client commands (GP Sink → Proxy)
// ---------------------------------------------------------------------------
pub const CMD_GP_PAIRING: CommandId = CommandId(0x01);
pub const CMD_GP_PROXY_COMMISSIONING_MODE: CommandId = CommandId(0x02);
pub const CMD_GP_RESPONSE: CommandId = CommandId(0x06);
pub const CMD_GP_SINK_TABLE_RESPONSE: CommandId = CommandId(0x0A);
pub const CMD_GP_PROXY_TABLE_REQUEST: CommandId = CommandId(0x0B);

// ---------------------------------------------------------------------------
// Green Power Device ID types
// ---------------------------------------------------------------------------
pub const GPD_DEVICE_ID_GP_ON_OFF_SWITCH: u8 = 0x02;
pub const GPD_DEVICE_ID_GP_LEVEL_CONTROL_SWITCH: u8 = 0x03;
pub const GPD_DEVICE_ID_GP_ADVANCED_GENERIC_SWITCH: u8 = 0x04;
pub const GPD_DEVICE_ID_GP_COLOR_DIMMER_SWITCH: u8 = 0x05;
pub const GPD_DEVICE_ID_GP_LIGHT_SENSOR: u8 = 0x06;
pub const GPD_DEVICE_ID_GP_SIMPLE_SENSOR: u8 = 0x07;
pub const GPD_DEVICE_ID_GP_OCCUPANCY_SENSOR: u8 = 0x08;
pub const GPD_DEVICE_ID_GP_DOOR_LOCK_CONTROLLER: u8 = 0x09;
pub const GPD_DEVICE_ID_GP_TEMPERATURE_SENSOR: u8 = 0x30;
pub const GPD_DEVICE_ID_GP_HUMIDITY_SENSOR: u8 = 0x31;

// ---------------------------------------------------------------------------
// Security levels
// ---------------------------------------------------------------------------
pub const GP_SECURITY_LEVEL_NONE: u8 = 0x00;
pub const GP_SECURITY_LEVEL_RESERVED: u8 = 0x01;
pub const GP_SECURITY_LEVEL_FC_MIC_32: u8 = 0x02;
pub const GP_SECURITY_LEVEL_FC_MIC_ENC_32: u8 = 0x03;

// ---------------------------------------------------------------------------
// GP Device Entry
// ---------------------------------------------------------------------------

/// Describes a paired Green Power Device.
pub struct GpDeviceEntry {
    pub gpd_id: u32,
    pub gpd_ieee: [u8; 8],
    pub endpoint: u8,
    pub security_level: u8,
    pub security_key: [u8; 16],
    pub device_id: u8,
    pub assigned_alias: u16,
}

// ---------------------------------------------------------------------------
// GP Role
// ---------------------------------------------------------------------------

/// Operational role of the Green Power cluster instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpRole {
    /// Router acting as a GP Proxy.
    Proxy,
    /// Coordinator acting as a GP Sink.
    Sink,
    /// Device implementing both Proxy and Sink.
    Combined,
}

// ---------------------------------------------------------------------------
// Cluster struct
// ---------------------------------------------------------------------------

/// Green Power cluster implementation.
///
/// The attribute set depends on the selected [`GpRole`]:
/// - `Proxy` registers only proxy-side attributes.
/// - `Sink` registers only sink-side attributes.
/// - `Combined` registers both.
pub struct GreenPowerCluster {
    store: AttributeStore<16>,
    role: GpRole,
}

impl GreenPowerCluster {
    pub fn new(role: GpRole) -> Self {
        let mut store = AttributeStore::new();

        // Proxy attributes
        if matches!(role, GpRole::Proxy | GpRole::Combined) {
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_MAX_PROXY_TABLE_ENTRIES,
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "GppMaxProxyTableEntries",
                },
                ZclValue::U8(5),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_PROXY_TABLE,
                    data_type: ZclDataType::OctetString,
                    access: AttributeAccess::ReadOnly,
                    name: "ProxyTable",
                },
                ZclValue::OctetString(heapless::Vec::new()),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_NOTIFICATION_RETRY_NUMBER,
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "GppNotificationRetryNumber",
                },
                ZclValue::U8(2),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_NOTIFICATION_RETRY_TIMER,
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "GppNotificationRetryTimer",
                },
                ZclValue::U8(100),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_MAX_SEARCH_COUNTER,
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "GppMaxSearchCounter",
                },
                ZclValue::U8(10),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_BLOCKED_GPD_ID,
                    data_type: ZclDataType::OctetString,
                    access: AttributeAccess::ReadOnly,
                    name: "GppBlockedGPDID",
                },
                ZclValue::OctetString(heapless::Vec::new()),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_FUNCTIONALITY,
                    data_type: ZclDataType::Bitmap32,
                    access: AttributeAccess::ReadOnly,
                    name: "GppFunctionality",
                },
                ZclValue::Bitmap32(0x00FF_FFFF),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPP_ACTIVE_FUNCTIONALITY,
                    data_type: ZclDataType::Bitmap32,
                    access: AttributeAccess::ReadOnly,
                    name: "GppActiveFunctionality",
                },
                ZclValue::Bitmap32(0x00FF_FFFF),
            );
        }

        // Sink attributes
        if matches!(role, GpRole::Sink | GpRole::Combined) {
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_SINK_TABLE,
                    data_type: ZclDataType::OctetString,
                    access: AttributeAccess::ReadOnly,
                    name: "GpsSinkTable",
                },
                ZclValue::OctetString(heapless::Vec::new()),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_COMMUNICATION_MODE,
                    data_type: ZclDataType::Bitmap8,
                    access: AttributeAccess::ReadWrite,
                    name: "GpsCommunicationMode",
                },
                ZclValue::Bitmap8(0x01),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_COMMISSIONING_EXIT_MODE,
                    data_type: ZclDataType::Bitmap8,
                    access: AttributeAccess::ReadWrite,
                    name: "GpsCommissioningExitMode",
                },
                ZclValue::Bitmap8(0x01),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_COMMISSIONING_WINDOW,
                    data_type: ZclDataType::U16,
                    access: AttributeAccess::ReadWrite,
                    name: "GpsCommissioningWindow",
                },
                ZclValue::U16(180),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_SECURITY_LEVEL,
                    data_type: ZclDataType::Bitmap8,
                    access: AttributeAccess::ReadWrite,
                    name: "GpsSecurityLevel",
                },
                ZclValue::Bitmap8(0x06),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_FUNCTIONALITY,
                    data_type: ZclDataType::Bitmap32,
                    access: AttributeAccess::ReadOnly,
                    name: "GpsFunctionality",
                },
                ZclValue::Bitmap32(0x00FF_FFFF),
            );
            let _ = store.register(
                AttributeDefinition {
                    id: ATTR_GPS_ACTIVE_FUNCTIONALITY,
                    data_type: ZclDataType::Bitmap32,
                    access: AttributeAccess::ReadOnly,
                    name: "GpsActiveFunctionality",
                },
                ZclValue::Bitmap32(0x00FF_FFFF),
            );
        }

        Self { store, role }
    }

    /// The role this instance was created with.
    pub fn role(&self) -> GpRole {
        self.role
    }
}

impl Cluster for GreenPowerCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::GREEN_POWER
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_GP_NOTIFICATION => {
                // Minimum payload: options (u16) + GPD SrcID (u32) = 6 bytes
                if payload.len() < 6 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _options = u16::from_le_bytes([payload[0], payload[1]]);
                let _gpd_src_id =
                    u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                // Forwarded to sink processing (application layer).
                Ok(heapless::Vec::new())
            }
            CMD_GP_COMMISSIONING_NOTIFICATION => {
                // Minimum payload: options (u16) + GPD SrcID (u32) + security frame counter (u32) = 10
                if payload.len() < 10 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _options = u16::from_le_bytes([payload[0], payload[1]]);
                let _gpd_src_id =
                    u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                let _security_frame_counter =
                    u32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]);
                Ok(heapless::Vec::new())
            }
            CMD_GP_PAIRING_SEARCH
            | CMD_GP_TUNNELING_STOP
            | CMD_GP_TRANSLATION_TABLE_UPDATE
            | CMD_GP_TRANSLATION_TABLE_REQUEST
            | CMD_GP_PAIRING_CONFIGURATION
            | CMD_GP_SINK_TABLE_REQUEST
            | CMD_GP_PROXY_TABLE_RESPONSE => {
                // These commands require full GP infrastructure; accept but no-op.
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
