//! Identify cluster (0x0003).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_IDENTIFY_TIME: AttributeId = AttributeId(0x0000);

// Command IDs
pub const CMD_IDENTIFY: CommandId = CommandId(0x00);
pub const CMD_IDENTIFY_QUERY: CommandId = CommandId(0x01);
pub const CMD_TRIGGER_EFFECT: CommandId = CommandId(0x40);

// Response command IDs (server to client)
pub const CMD_IDENTIFY_QUERY_RESPONSE: CommandId = CommandId(0x00);

/// Identify cluster implementation.
pub struct IdentifyCluster {
    store: AttributeStore<4>,
    /// Pending trigger effect: (effect_id, effect_variant).
    /// Set by TriggerEffect command, cleared by application via `take_pending_effect()`.
    pending_effect: Option<(u8, u8)>,
}

impl Default for IdentifyCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl IdentifyCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IDENTIFY_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "IdentifyTime",
            },
            ZclValue::U16(0),
        );
        Self {
            store,
            pending_effect: None,
        }
    }

    /// Tick down identify timer by `elapsed_secs`.
    pub fn tick(&mut self, elapsed_secs: u16) {
        if let Some(ZclValue::U16(t)) = self.store.get(ATTR_IDENTIFY_TIME) {
            let remaining = t.saturating_sub(elapsed_secs);
            let _ = self
                .store
                .set_raw(ATTR_IDENTIFY_TIME, ZclValue::U16(remaining));
        }
    }

    /// Whether the device is currently identifying.
    pub fn is_identifying(&self) -> bool {
        matches!(self.store.get(ATTR_IDENTIFY_TIME), Some(ZclValue::U16(t)) if *t > 0)
    }

    /// Returns the pending trigger effect if one was received, or `None`.
    /// Calling this clears the pending effect.
    pub fn take_pending_effect(&mut self) -> Option<(u8, u8)> {
        self.pending_effect.take()
    }

    /// Check if there is a pending trigger effect (without consuming it).
    pub fn has_pending_effect(&self) -> bool {
        self.pending_effect.is_some()
    }
}

impl Cluster for IdentifyCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::IDENTIFY
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_IDENTIFY => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let time = u16::from_le_bytes([payload[0], payload[1]]);
                let _ = self.store.set_raw(ATTR_IDENTIFY_TIME, ZclValue::U16(time));
                Ok(heapless::Vec::new())
            }
            CMD_IDENTIFY_QUERY => {
                // Respond with IdentifyQueryResponse if identifying.
                let mut resp = heapless::Vec::new();
                if let Some(ZclValue::U16(t)) = self.store.get(ATTR_IDENTIFY_TIME)
                    && *t > 0
                {
                    let b = t.to_le_bytes();
                    let _ = resp.push(b[0]);
                    let _ = resp.push(b[1]);
                }
                Ok(resp)
            }
            CMD_TRIGGER_EFFECT => {
                // Payload: effect_id (u8) + effect_variant (u8)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                self.pending_effect = Some((payload[0], payload[1]));
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
