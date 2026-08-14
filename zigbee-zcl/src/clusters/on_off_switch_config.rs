//! On/Off Switch Configuration cluster (0x0007).
//!
//! Configures the behaviour of an on/off switch device.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_SWITCH_TYPE: AttributeId = AttributeId(0x0000);
pub const ATTR_SWITCH_ACTIONS: AttributeId = AttributeId(0x0010);

// SwitchType enumeration values
pub const SWITCH_TYPE_TOGGLE: u8 = 0x00;
pub const SWITCH_TYPE_MOMENTARY: u8 = 0x01;
pub const SWITCH_TYPE_MULTIFUNCTION: u8 = 0x02;

// SwitchActions enumeration values
pub const SWITCH_ACTION_ON_OFF: u8 = 0x00;
pub const SWITCH_ACTION_OFF_ON: u8 = 0x01;
pub const SWITCH_ACTION_TOGGLE: u8 = 0x02;

/// On/Off Switch Configuration cluster implementation.
pub struct OnOffSwitchConfigCluster {
    store: AttributeStore<2>,
}

impl OnOffSwitchConfigCluster {
    pub fn new(switch_type: u8) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SWITCH_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "SwitchType",
            },
            ZclValue::Enum8(switch_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SWITCH_ACTIONS,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "SwitchActions",
            },
            ZclValue::Enum8(SWITCH_ACTION_ON_OFF),
        );
        Self { store }
    }
}

impl Cluster for OnOffSwitchConfigCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0007)
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // No cluster-specific commands defined for On/Off Switch Configuration.
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    fn reset_to_factory_defaults(&mut self) {
        // SwitchType is a physical-capability constructor parameter and is
        // preserved; SwitchActions is the only writable attribute.
        let _ = self
            .store
            .set_raw(ATTR_SWITCH_ACTIONS, ZclValue::Enum8(SWITCH_ACTION_ON_OFF));
    }
}
