//! Diagnostics cluster (0x0B05).
//!
//! Exposes network-layer and MAC-layer counters for monitoring device health.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_NUMBER_OF_RESETS: AttributeId = AttributeId(0x0000);
pub const ATTR_MAC_RX_BCAST: AttributeId = AttributeId(0x0100);
pub const ATTR_MAC_TX_BCAST: AttributeId = AttributeId(0x0101);
pub const ATTR_MAC_RX_UCAST: AttributeId = AttributeId(0x0102);
pub const ATTR_MAC_TX_UCAST: AttributeId = AttributeId(0x0103);
pub const ATTR_MAC_TX_UCAST_RETRY: AttributeId = AttributeId(0x0104);
pub const ATTR_MAC_TX_UCAST_FAIL: AttributeId = AttributeId(0x0105);
pub const ATTR_APS_RX_BCAST: AttributeId = AttributeId(0x0106);
pub const ATTR_APS_TX_BCAST: AttributeId = AttributeId(0x0107);
pub const ATTR_APS_RX_UCAST: AttributeId = AttributeId(0x0108);
pub const ATTR_APS_TX_UCAST_SUCCESS: AttributeId = AttributeId(0x0109);
pub const ATTR_APS_TX_UCAST_RETRY: AttributeId = AttributeId(0x010A);
pub const ATTR_APS_TX_UCAST_FAIL: AttributeId = AttributeId(0x010B);
pub const ATTR_LAST_MESSAGE_LQI: AttributeId = AttributeId(0x011C);
pub const ATTR_LAST_MESSAGE_RSSI: AttributeId = AttributeId(0x011D);

/// Diagnostics cluster implementation.
pub struct DiagnosticsCluster {
    store: AttributeStore<15>,
}

impl Default for DiagnosticsCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl DiagnosticsCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUMBER_OF_RESETS,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfResets",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_RX_BCAST,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "MacRxBcast",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_TX_BCAST,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "MacTxBcast",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_RX_UCAST,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "MacRxUcast",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_TX_UCAST,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "MacTxUcast",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_TX_UCAST_RETRY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MacTxUcastRetry",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAC_TX_UCAST_FAIL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MacTxUcastFail",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_RX_BCAST,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSRxBcast",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_TX_BCAST,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSTxBcast",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_RX_UCAST,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSRxUcast",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_TX_UCAST_SUCCESS,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSTxUcastSuccess",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_TX_UCAST_RETRY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSTxUcastRetry",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APS_TX_UCAST_FAIL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "APSTxUcastFail",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAST_MESSAGE_LQI,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "LastMessageLQI",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAST_MESSAGE_RSSI,
                data_type: ZclDataType::I8,
                access: AttributeAccess::ReadOnly,
                name: "LastMessageRSSI",
            },
            ZclValue::I8(0),
        );
        Self { store }
    }

    /// Increment a U16 or U32 counter attribute by one (saturating).
    pub fn increment_counter(&mut self, attr: AttributeId) {
        match self.store.get(attr) {
            Some(ZclValue::U16(v)) => {
                let _ = self.store.set_raw(attr, ZclValue::U16(v.saturating_add(1)));
            }
            Some(ZclValue::U32(v)) => {
                let _ = self.store.set_raw(attr, ZclValue::U32(v.saturating_add(1)));
            }
            _ => {}
        }
    }
}

impl Cluster for DiagnosticsCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0B05)
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // No cluster-specific commands defined for Diagnostics cluster.
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
