//! Poll Control cluster (0x0020).
//!
//! Server = sleepy end device; Client = coordinator/gateway.
//! Allows a sleepy device to check in periodically and be told to enter fast-polling mode.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_CHECK_IN_INTERVAL: AttributeId = AttributeId(0x0000);
pub const ATTR_LONG_POLL_INTERVAL: AttributeId = AttributeId(0x0001);
pub const ATTR_SHORT_POLL_INTERVAL: AttributeId = AttributeId(0x0002);
pub const ATTR_FAST_POLL_TIMEOUT: AttributeId = AttributeId(0x0003);
pub const ATTR_CHECK_IN_INTERVAL_MIN: AttributeId = AttributeId(0x0004);
pub const ATTR_LONG_POLL_INTERVAL_MIN: AttributeId = AttributeId(0x0005);
pub const ATTR_FAST_POLL_TIMEOUT_MAX: AttributeId = AttributeId(0x0006);

// Server→Client command IDs
pub const CMD_CHECK_IN: CommandId = CommandId(0x00);

// Client→Server command IDs
pub const CMD_CHECK_IN_RESPONSE: CommandId = CommandId(0x00);
pub const CMD_FAST_POLL_STOP: CommandId = CommandId(0x01);
pub const CMD_SET_LONG_POLL_INTERVAL: CommandId = CommandId(0x02);
pub const CMD_SET_SHORT_POLL_INTERVAL: CommandId = CommandId(0x03);

/// Poll Control cluster implementation.
pub struct PollControlCluster {
    store: AttributeStore<7>,
    fast_polling: bool,
    /// Ticks since last check-in (1 tick = 1 quarter-second = 250ms).
    ticks_since_checkin: u32,
    /// Remaining fast-poll ticks (quarter-seconds).
    fast_poll_remaining: u16,
}

impl Default for PollControlCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl PollControlCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CHECK_IN_INTERVAL,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "CheckInInterval",
            },
            ZclValue::U32(14400), // 60 min in quarter-seconds
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LONG_POLL_INTERVAL,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "LongPollInterval",
            },
            ZclValue::U32(24), // 6 sec in quarter-seconds
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SHORT_POLL_INTERVAL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ShortPollInterval",
            },
            ZclValue::U16(4), // 1 sec in quarter-seconds
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAST_POLL_TIMEOUT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "FastPollTimeout",
            },
            ZclValue::U16(40), // 10 sec in quarter-seconds
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CHECK_IN_INTERVAL_MIN,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "CheckInIntervalMin",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LONG_POLL_INTERVAL_MIN,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "LongPollIntervalMin",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAST_POLL_TIMEOUT_MAX,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "FastPollTimeoutMax",
            },
            ZclValue::U16(0),
        );
        Self {
            store,
            fast_polling: false,
            ticks_since_checkin: 0,
            fast_poll_remaining: 0,
        }
    }

    /// Build a CheckIn command payload (server→client, empty body).
    pub fn trigger_checkin(&self) -> heapless::Vec<u8, 64> {
        heapless::Vec::new()
    }

    /// Tick the poll control cluster (call every quarter-second = 250ms).
    ///
    /// Returns `true` when a CheckIn command should be sent to the bound client.
    pub fn tick(&mut self) -> bool {
        // Fast-poll timeout countdown
        if self.fast_polling && self.fast_poll_remaining > 0 {
            self.fast_poll_remaining = self.fast_poll_remaining.saturating_sub(1);
            if self.fast_poll_remaining == 0 {
                self.fast_polling = false;
            }
        }

        // Check-in interval countdown
        let check_in_interval = match self.store.get(ATTR_CHECK_IN_INTERVAL) {
            Some(ZclValue::U32(v)) => *v,
            _ => 14400,
        };
        if check_in_interval == 0 {
            return false; // disabled
        }

        self.ticks_since_checkin += 1;
        if self.ticks_since_checkin >= check_in_interval {
            self.ticks_since_checkin = 0;
            true // time to send CheckIn
        } else {
            false
        }
    }

    /// Enter fast-polling mode with the given timeout (in quarter-seconds).
    pub fn set_fast_polling(&mut self, timeout: u16) {
        let _ = self
            .store
            .set_raw(ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(timeout));
        self.fast_poll_remaining = timeout;
        self.fast_polling = true;
    }

    /// Whether the device is currently in fast-polling mode.
    pub fn is_fast_polling(&self) -> bool {
        self.fast_polling
    }
}

impl Cluster for PollControlCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0020)
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_CHECK_IN_RESPONSE => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let start_fast_polling = payload[0] != 0;
                let fast_poll_timeout = u16::from_le_bytes([payload[1], payload[2]]);
                if start_fast_polling {
                    self.set_fast_polling(fast_poll_timeout);
                } else {
                    self.fast_polling = false;
                }
                Ok(heapless::Vec::new())
            }
            CMD_FAST_POLL_STOP => {
                self.fast_polling = false;
                Ok(heapless::Vec::new())
            }
            CMD_SET_LONG_POLL_INTERVAL => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let interval = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                let _ = self
                    .store
                    .set_raw(ATTR_LONG_POLL_INTERVAL, ZclValue::U32(interval));
                Ok(heapless::Vec::new())
            }
            CMD_SET_SHORT_POLL_INTERVAL => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let interval = u16::from_le_bytes([payload[0], payload[1]]);
                let _ = self
                    .store
                    .set_raw(ATTR_SHORT_POLL_INTERVAL, ZclValue::U16(interval));
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        let mut v = heapless::Vec::new();
        let _ = v.push(CMD_CHECK_IN_RESPONSE.0);
        let _ = v.push(CMD_FAST_POLL_STOP.0);
        let _ = v.push(CMD_SET_LONG_POLL_INTERVAL.0);
        let _ = v.push(CMD_SET_SHORT_POLL_INTERVAL.0);
        v
    }

    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        let mut v = heapless::Vec::new();
        let _ = v.push(CMD_CHECK_IN.0);
        v
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
