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

/// Product power-on values and limits, all in quarter-seconds.
///
/// The attribute fields are the ZCL attributes' power-on (and factory-reset)
/// values. [`long_poll_interval_max`](Self::long_poll_interval_max) is not an
/// attribute: it is the longest LongPollInterval the product can honour
/// (for example its parent child-timeout or bounded sleep), and a
/// SetLongPollInterval above it is refused with INVALID_VALUE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PollControlConfig {
    pub check_in_interval: u32,
    pub long_poll_interval: u32,
    pub short_poll_interval: u16,
    pub fast_poll_timeout: u16,
    pub check_in_interval_min: u32,
    pub long_poll_interval_min: u32,
    pub fast_poll_timeout_max: u16,
    /// 0 = only the ZCL range limit applies.
    pub long_poll_interval_max: u32,
    /// How long the server fast-polls after its own Check-in while it waits
    /// for the Check-in Response (not an attribute).
    pub check_in_response_timeout: u16,
}

impl PollControlConfig {
    /// The cluster's historical defaults: 1 h check-in, 6 s long poll,
    /// 1 s short poll, 10 s fast-poll timeout, no product minimums.
    pub const DEFAULT: Self = Self {
        check_in_interval: 14400,
        long_poll_interval: 24,
        short_poll_interval: 4,
        fast_poll_timeout: 40,
        check_in_interval_min: 0,
        long_poll_interval_min: 0,
        fast_poll_timeout_max: 0,
        long_poll_interval_max: 0,
        check_in_response_timeout: 32,
    };

    /// Whether the values satisfy the ZCL r8 §3.16.4 ranges and orderings
    /// the cluster itself enforces on remote changes.
    pub const fn is_valid(&self) -> bool {
        let long_max = if self.long_poll_interval_max == 0 {
            MAX_POLL_INTERVAL
        } else {
            self.long_poll_interval_max
        };
        self.long_poll_interval >= 4
            && self.long_poll_interval <= long_max
            && long_max <= MAX_POLL_INTERVAL
            && self.long_poll_interval >= self.long_poll_interval_min
            && self.short_poll_interval != 0
            && self.short_poll_interval as u32 <= self.long_poll_interval
            && (self.check_in_interval == 0
                || (self.check_in_interval <= MAX_POLL_INTERVAL
                    && self.check_in_interval >= self.check_in_interval_min
                    && self.check_in_interval >= self.long_poll_interval))
            && self.fast_poll_timeout != 0
            && (self.fast_poll_timeout_max == 0
                || self.fast_poll_timeout <= self.fast_poll_timeout_max)
            && self.check_in_response_timeout != 0
    }
}

impl Default for PollControlConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// The Poll Control values a client can change remotely (CheckInInterval and
/// FastPollTimeout by Write Attributes, LongPollInterval and
/// ShortPollInterval by command), in quarter-seconds, for products that
/// persist them across a reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PollControlState {
    pub check_in_interval: u32,
    pub long_poll_interval: u32,
    pub short_poll_interval: u16,
    pub fast_poll_timeout: u16,
}

impl PollControlState {
    /// Encoded length: two little-endian `u32` and two `u16`.
    pub const ENCODED_LEN: usize = 12;

    pub fn encode(&self) -> [u8; Self::ENCODED_LEN] {
        let mut out = [0u8; Self::ENCODED_LEN];
        out[0..4].copy_from_slice(&self.check_in_interval.to_le_bytes());
        out[4..8].copy_from_slice(&self.long_poll_interval.to_le_bytes());
        out[8..10].copy_from_slice(&self.short_poll_interval.to_le_bytes());
        out[10..12].copy_from_slice(&self.fast_poll_timeout.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let bytes: &[u8; Self::ENCODED_LEN] = bytes.try_into().ok()?;
        Some(Self {
            check_in_interval: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            long_poll_interval: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            short_poll_interval: u16::from_le_bytes([bytes[8], bytes[9]]),
            fast_poll_timeout: u16::from_le_bytes([bytes[10], bytes[11]]),
        })
    }
}

/// A client's fast-poll decision, for products that run their own poll
/// timing instead of [`PollControlCluster::tick`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastPollEvent {
    /// Check-in Response asked for fast polling for this many
    /// quarter-seconds (already resolved against FastPollTimeout and
    /// FastPollTimeoutMax).
    Start { timeout_qs: u16 },
    /// Fast Poll Stop, or a Check-in Response declining fast polling.
    Stop,
}

/// Poll Control cluster implementation.
pub struct PollControlCluster {
    store: AttributeStore<7>,
    config: PollControlConfig,
    /// Latest unconsumed client fast-poll decision.
    fast_poll_event: Option<FastPollEvent>,
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
        Self::with_config(PollControlConfig::DEFAULT)
    }

    /// A server with product power-on values and limits. An invalid
    /// configuration (see [`PollControlConfig::is_valid`]) is a product bug;
    /// callers should check it at compile time.
    pub fn with_config(config: PollControlConfig) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CHECK_IN_INTERVAL,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "CheckInInterval",
            },
            ZclValue::U32(config.check_in_interval),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LONG_POLL_INTERVAL,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "LongPollInterval",
            },
            ZclValue::U32(config.long_poll_interval),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SHORT_POLL_INTERVAL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ShortPollInterval",
            },
            ZclValue::U16(config.short_poll_interval),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAST_POLL_TIMEOUT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "FastPollTimeout",
            },
            ZclValue::U16(config.fast_poll_timeout),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CHECK_IN_INTERVAL_MIN,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "CheckInIntervalMin",
            },
            ZclValue::U32(config.check_in_interval_min),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LONG_POLL_INTERVAL_MIN,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "LongPollIntervalMin",
            },
            ZclValue::U32(config.long_poll_interval_min),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAST_POLL_TIMEOUT_MAX,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "FastPollTimeoutMax",
            },
            ZclValue::U16(config.fast_poll_timeout_max),
        );
        Self {
            store,
            config,
            fast_poll_event: None,
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
    ///
    /// The timeout applies to this fast-poll period only; the FastPollTimeout
    /// attribute (the default period) is left unchanged.
    pub fn set_fast_polling(&mut self, timeout: u16) {
        self.fast_poll_remaining = timeout;
        self.fast_polling = timeout != 0;
    }

    /// Whether the device is currently in fast-polling mode.
    pub fn is_fast_polling(&self) -> bool {
        self.fast_polling
    }

    /// Take the latest client fast-poll decision since the previous call.
    pub fn take_fast_poll_event(&mut self) -> Option<FastPollEvent> {
        self.fast_poll_event.take()
    }

    /// CheckInInterval in quarter-seconds; 0 = check-ins disabled.
    pub fn check_in_interval_qs(&self) -> u32 {
        self.u32_attr(ATTR_CHECK_IN_INTERVAL)
    }

    /// LongPollInterval in quarter-seconds.
    pub fn long_poll_interval_qs(&self) -> u32 {
        self.u32_attr(ATTR_LONG_POLL_INTERVAL)
    }

    /// ShortPollInterval in quarter-seconds.
    pub fn short_poll_interval_qs(&self) -> u16 {
        self.u32_attr(ATTR_SHORT_POLL_INTERVAL) as u16
    }

    /// Current client-changeable values; see [`PollControlState`].
    pub fn state(&self) -> PollControlState {
        PollControlState {
            check_in_interval: self.u32_attr(ATTR_CHECK_IN_INTERVAL),
            long_poll_interval: self.u32_attr(ATTR_LONG_POLL_INTERVAL),
            short_poll_interval: self.u32_attr(ATTR_SHORT_POLL_INTERVAL) as u16,
            fast_poll_timeout: self.u32_attr(ATTR_FAST_POLL_TIMEOUT) as u16,
        }
    }

    /// Apply persisted client-changeable values.
    ///
    /// The values are validated together against this product's limits
    /// (the same ranges and orderings [`PollControlConfig::is_valid`]
    /// enforces); an inconsistent or out-of-range state is refused and the
    /// cluster keeps its current values. Returns whether it was applied.
    pub fn restore_state(&mut self, state: PollControlState) -> bool {
        let candidate = PollControlConfig {
            check_in_interval: state.check_in_interval,
            long_poll_interval: state.long_poll_interval,
            short_poll_interval: state.short_poll_interval,
            fast_poll_timeout: state.fast_poll_timeout,
            ..self.config
        };
        if !candidate.is_valid() {
            return false;
        }
        let _ = self.store.set_raw(
            ATTR_CHECK_IN_INTERVAL,
            ZclValue::U32(state.check_in_interval),
        );
        let _ = self.store.set_raw(
            ATTR_LONG_POLL_INTERVAL,
            ZclValue::U32(state.long_poll_interval),
        );
        let _ = self.store.set_raw(
            ATTR_SHORT_POLL_INTERVAL,
            ZclValue::U16(state.short_poll_interval),
        );
        let _ = self.store.set_raw(
            ATTR_FAST_POLL_TIMEOUT,
            ZclValue::U16(state.fast_poll_timeout),
        );
        true
    }

    /// Fast-poll period after a Check-in, waiting for its response.
    pub fn check_in_response_timeout_qs(&self) -> u16 {
        self.config.check_in_response_timeout
    }

    fn long_poll_interval_max(&self) -> u32 {
        if self.config.long_poll_interval_max == 0 {
            MAX_POLL_INTERVAL
        } else {
            self.config.long_poll_interval_max
        }
    }

    fn u32_attr(&self, id: AttributeId) -> u32 {
        match self.store.get(id) {
            Some(ZclValue::U32(v)) => *v,
            Some(ZclValue::U16(v)) => *v as u32,
            _ => 0,
        }
    }

    /// Range/ordering rules for the poll intervals (ZCL r8 §3.16.4).
    fn check_value(&self, id: AttributeId, value: &ZclValue) -> Result<(), ZclStatus> {
        let ok = match (id, value) {
            // 0 disables check-ins; otherwise within range, not below the
            // product minimum and not shorter than the long poll interval.
            (ATTR_CHECK_IN_INTERVAL, ZclValue::U32(v)) => {
                *v == 0
                    || (*v <= MAX_POLL_INTERVAL
                        && *v >= self.u32_attr(ATTR_CHECK_IN_INTERVAL_MIN)
                        && *v >= self.u32_attr(ATTR_LONG_POLL_INTERVAL))
            }
            (ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(v)) => self.fast_poll_timeout_ok(*v),
            _ => true,
        };
        if ok {
            Ok(())
        } else {
            Err(ZclStatus::InvalidValue)
        }
    }

    /// FastPollTimeout values must be non-zero and, when the product
    /// advertises a FastPollTimeoutMax, not exceed it.
    fn fast_poll_timeout_ok(&self, v: u16) -> bool {
        let max = self.u32_attr(ATTR_FAST_POLL_TIMEOUT_MAX);
        v != 0 && (max == 0 || v as u32 <= max)
    }
}

/// Largest LongPollInterval / CheckInInterval (quarter-seconds, ZCL r8 §3.16.4).
const MAX_POLL_INTERVAL: u32 = 0x006E_0000;

/// Attribute writes go through the cluster so range/ordering rules are
/// enforced (INVALID_VALUE), not just the data type.
impl AttributeStoreMutAccess for PollControlCluster {
    fn set(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus> {
        AttributeStoreMutAccess::validate_set(self, id, &value)?;
        self.store.set(id, value)
    }
    fn set_raw(&mut self, id: AttributeId, value: ZclValue) -> Result<(), ZclStatus> {
        self.store.set_raw(id, value)
    }
    fn find(&self, id: AttributeId) -> Option<&AttributeDefinition> {
        self.store.find(id)
    }
    fn validate_set(&self, id: AttributeId, value: &ZclValue) -> Result<(), ZclStatus> {
        self.store.validate_set(id, value)?;
        self.check_value(id, value)
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
                let requested = u16::from_le_bytes([payload[1], payload[2]]);
                if start_fast_polling {
                    // §3.16.5.1: a zero timeout means "use the FastPollTimeout
                    // attribute"; a non-zero one must respect FastPollTimeoutMax.
                    let timeout = if requested == 0 {
                        self.u32_attr(ATTR_FAST_POLL_TIMEOUT) as u16
                    } else if self.fast_poll_timeout_ok(requested) {
                        requested
                    } else {
                        return Err(ZclStatus::InvalidField);
                    };
                    self.set_fast_polling(timeout);
                    self.fast_poll_event = Some(FastPollEvent::Start {
                        timeout_qs: timeout,
                    });
                } else {
                    self.fast_polling = false;
                    self.fast_poll_event = Some(FastPollEvent::Stop);
                }
                Ok(heapless::Vec::new())
            }
            CMD_FAST_POLL_STOP => {
                self.fast_polling = false;
                self.fast_poll_event = Some(FastPollEvent::Stop);
                Ok(heapless::Vec::new())
            }
            CMD_SET_LONG_POLL_INTERVAL => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let interval = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                // §3.16.5.3: within 0x04..=0x6E0000, not below
                // LongPollIntervalMin or ShortPollInterval, and not above
                // a non-zero CheckInInterval. A product that cannot sleep
                // longer also refuses values above its own maximum.
                let check_in = self.u32_attr(ATTR_CHECK_IN_INTERVAL);
                if !(4..=self.long_poll_interval_max()).contains(&interval)
                    || interval < self.u32_attr(ATTR_LONG_POLL_INTERVAL_MIN)
                    || interval < self.u32_attr(ATTR_SHORT_POLL_INTERVAL)
                    || (check_in != 0 && interval > check_in)
                {
                    return Err(ZclStatus::InvalidValue);
                }
                self.store
                    .set_raw(ATTR_LONG_POLL_INTERVAL, ZclValue::U32(interval))?;
                Ok(heapless::Vec::new())
            }
            CMD_SET_SHORT_POLL_INTERVAL => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let interval = u16::from_le_bytes([payload[0], payload[1]]);
                // §3.16.5.4: non-zero and not above LongPollInterval.
                if interval == 0 || interval as u32 > self.u32_attr(ATTR_LONG_POLL_INTERVAL) {
                    return Err(ZclStatus::InvalidValue);
                }
                self.store
                    .set_raw(ATTR_SHORT_POLL_INTERVAL, ZclValue::U16(interval))?;
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
        self
    }

    fn reset_to_factory_defaults(&mut self) {
        let config = self.config;
        let _ = self.store.set_raw(
            ATTR_CHECK_IN_INTERVAL,
            ZclValue::U32(config.check_in_interval),
        );
        let _ = self.store.set_raw(
            ATTR_FAST_POLL_TIMEOUT,
            ZclValue::U16(config.fast_poll_timeout),
        );
        // LongPollInterval/ShortPollInterval are technically ReadOnly per
        // their access mode but are mutated by SetLongPollInterval/
        // SetShortPollInterval — restore them to their power-on defaults
        // too so a stale negotiated interval cannot survive a reset.
        let _ = self.store.set_raw(
            ATTR_LONG_POLL_INTERVAL,
            ZclValue::U32(config.long_poll_interval),
        );
        let _ = self.store.set_raw(
            ATTR_SHORT_POLL_INTERVAL,
            ZclValue::U16(config.short_poll_interval),
        );
        self.fast_poll_event = None;
        self.fast_polling = false;
        self.ticks_since_checkin = 0;
        self.fast_poll_remaining = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get(c: &PollControlCluster, id: AttributeId) -> ZclValue {
        c.attributes().get(id).unwrap().clone()
    }

    #[test]
    fn check_in_response_zero_timeout_uses_fast_poll_timeout_attribute() {
        let mut c = PollControlCluster::new();
        c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 0, 0]).unwrap();
        assert!(c.is_fast_polling());
        // Default FastPollTimeout is 40 quarter-seconds: ends after 40 ticks.
        for _ in 0..39 {
            c.tick();
        }
        assert!(c.is_fast_polling());
        c.tick();
        assert!(!c.is_fast_polling());
        // A per-check-in timeout does not overwrite the attribute.
        c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 8, 0]).unwrap();
        assert_eq!(get(&c, ATTR_FAST_POLL_TIMEOUT), ZclValue::U16(40));
    }

    #[test]
    fn fast_poll_timeout_max_is_enforced() {
        let mut c = PollControlCluster::new();
        c.store
            .set_raw(ATTR_FAST_POLL_TIMEOUT_MAX, ZclValue::U16(100))
            .unwrap();
        assert_eq!(
            c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 101, 0]),
            Err(ZclStatus::InvalidField)
        );
        assert!(!c.is_fast_polling());
        assert_eq!(
            c.attributes_mut()
                .set(ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(101)),
            Err(ZclStatus::InvalidValue)
        );
        assert_eq!(
            c.attributes_mut()
                .set(ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(0)),
            Err(ZclStatus::InvalidValue)
        );
        c.attributes_mut()
            .set(ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(100))
            .unwrap();
    }

    #[test]
    fn poll_interval_commands_validate_range_and_ordering() {
        let mut c = PollControlCluster::new();
        let long = |v: u32| v.to_le_bytes();
        for bad in [
            0u32, 3, 2,     /* < short (4) */
            14401, /* > check-in */
        ] {
            assert_eq!(
                c.handle_command(CMD_SET_LONG_POLL_INTERVAL, &long(bad)),
                Err(ZclStatus::InvalidValue),
                "{bad}"
            );
        }
        c.handle_command(CMD_SET_LONG_POLL_INTERVAL, &long(8))
            .unwrap();
        assert_eq!(get(&c, ATTR_LONG_POLL_INTERVAL), ZclValue::U32(8));
        for bad in [0u16, 9] {
            assert_eq!(
                c.handle_command(CMD_SET_SHORT_POLL_INTERVAL, &bad.to_le_bytes()),
                Err(ZclStatus::InvalidValue)
            );
        }
        c.handle_command(CMD_SET_SHORT_POLL_INTERVAL, &8u16.to_le_bytes())
            .unwrap();
        // CheckInInterval below LongPollInterval is rejected; 0 disables.
        assert_eq!(
            c.attributes_mut()
                .set(ATTR_CHECK_IN_INTERVAL, ZclValue::U32(7)),
            Err(ZclStatus::InvalidValue)
        );
        c.attributes_mut()
            .set(ATTR_CHECK_IN_INTERVAL, ZclValue::U32(0))
            .unwrap();
    }

    const PRODUCT: PollControlConfig = PollControlConfig {
        check_in_interval: 14400,
        long_poll_interval: 120,
        short_poll_interval: 1,
        fast_poll_timeout: 40,
        check_in_interval_min: 480,
        long_poll_interval_min: 120,
        fast_poll_timeout_max: 240,
        long_poll_interval_max: 240,
        check_in_response_timeout: 32,
    };

    #[test]
    fn product_config_sets_power_on_values_limits_and_factory_defaults() {
        assert!(PRODUCT.is_valid());
        assert!(PollControlConfig::DEFAULT.is_valid());
        let mut c = PollControlCluster::with_config(PRODUCT);
        assert_eq!(c.check_in_interval_qs(), 14400);
        assert_eq!(c.long_poll_interval_qs(), 120);
        assert_eq!(c.short_poll_interval_qs(), 1);
        assert_eq!(get(&c, ATTR_CHECK_IN_INTERVAL_MIN), ZclValue::U32(480));
        assert_eq!(get(&c, ATTR_LONG_POLL_INTERVAL_MIN), ZclValue::U32(120));
        assert_eq!(get(&c, ATTR_FAST_POLL_TIMEOUT_MAX), ZclValue::U16(240));
        // ZHA's 6 s long poll is below the product minimum, 61 s above the
        // product maximum; 60 s is accepted.
        for bad in [24u32, 244] {
            assert_eq!(
                c.handle_command(CMD_SET_LONG_POLL_INTERVAL, &bad.to_le_bytes()),
                Err(ZclStatus::InvalidValue),
                "{bad}"
            );
        }
        c.handle_command(CMD_SET_LONG_POLL_INTERVAL, &240u32.to_le_bytes())
            .unwrap();
        assert_eq!(c.long_poll_interval_qs(), 240);
        // ZHA's 55 min check-in is accepted; below the minimum it is not.
        c.attributes_mut()
            .set(ATTR_CHECK_IN_INTERVAL, ZclValue::U32(13200))
            .unwrap();
        assert_eq!(
            c.attributes_mut()
                .set(ATTR_CHECK_IN_INTERVAL, ZclValue::U32(479)),
            Err(ZclStatus::InvalidValue)
        );
        c.reset_to_factory_defaults();
        assert_eq!(c.check_in_interval_qs(), 14400);
        assert_eq!(c.long_poll_interval_qs(), 120);
        assert_eq!(c.short_poll_interval_qs(), 1);
    }

    #[test]
    fn invalid_product_configs_are_detected() {
        for bad in [
            PollControlConfig {
                long_poll_interval: 3,
                ..PRODUCT
            },
            PollControlConfig {
                long_poll_interval: 241,
                ..PRODUCT
            },
            PollControlConfig {
                long_poll_interval_min: 121,
                ..PRODUCT
            },
            PollControlConfig {
                short_poll_interval: 121,
                ..PRODUCT
            },
            PollControlConfig {
                check_in_interval: 100,
                check_in_interval_min: 0,
                ..PRODUCT
            },
            PollControlConfig {
                check_in_interval: 479,
                ..PRODUCT
            },
            PollControlConfig {
                fast_poll_timeout: 241,
                ..PRODUCT
            },
            PollControlConfig {
                fast_poll_timeout: 0,
                ..PRODUCT
            },
        ] {
            assert!(!bad.is_valid(), "{bad:?}");
        }
        assert!(
            PollControlConfig {
                check_in_interval: 0,
                ..PRODUCT
            }
            .is_valid()
        );
    }

    #[test]
    fn client_fast_poll_decisions_are_reported_once() {
        let mut c = PollControlCluster::with_config(PRODUCT);
        assert_eq!(c.take_fast_poll_event(), None);
        // ZHA: Check-in Response (start, 8 qs), then Fast Poll Stop.
        c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 8, 0]).unwrap();
        assert_eq!(
            c.take_fast_poll_event(),
            Some(FastPollEvent::Start { timeout_qs: 8 })
        );
        assert_eq!(c.take_fast_poll_event(), None);
        c.handle_command(CMD_FAST_POLL_STOP, &[]).unwrap();
        assert_eq!(c.take_fast_poll_event(), Some(FastPollEvent::Stop));
        // Zero timeout resolves to FastPollTimeout; a decline is a stop.
        c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 0, 0]).unwrap();
        assert_eq!(
            c.take_fast_poll_event(),
            Some(FastPollEvent::Start { timeout_qs: 40 })
        );
        c.handle_command(CMD_CHECK_IN_RESPONSE, &[0, 0, 0]).unwrap();
        assert_eq!(c.take_fast_poll_event(), Some(FastPollEvent::Stop));
        // A refused timeout produces no event.
        assert_eq!(
            c.handle_command(CMD_CHECK_IN_RESPONSE, &[1, 0xF1, 0]),
            Err(ZclStatus::InvalidField)
        );
        assert_eq!(c.take_fast_poll_event(), None);
    }

    #[test]
    fn client_state_round_trips_and_restores_within_product_limits() {
        let mut c = PollControlCluster::with_config(PRODUCT);
        // A coordinator's writes and commands.
        c.set(ATTR_CHECK_IN_INTERVAL, ZclValue::U32(13200)).unwrap();
        c.set(ATTR_FAST_POLL_TIMEOUT, ZclValue::U16(80)).unwrap();
        c.handle_command(CMD_SET_LONG_POLL_INTERVAL, &240u32.to_le_bytes())
            .unwrap();
        c.handle_command(CMD_SET_SHORT_POLL_INTERVAL, &2u16.to_le_bytes())
            .unwrap();
        let state = c.state();
        assert_eq!(
            state,
            PollControlState {
                check_in_interval: 13200,
                long_poll_interval: 240,
                short_poll_interval: 2,
                fast_poll_timeout: 80,
            }
        );
        let encoded = state.encode();
        assert_eq!(PollControlState::decode(&encoded), Some(state));
        assert_eq!(PollControlState::decode(&encoded[..11]), None);

        let mut fresh = PollControlCluster::with_config(PRODUCT);
        assert!(fresh.restore_state(state));
        assert_eq!(fresh.state(), state);
        assert_eq!(fresh.long_poll_interval_qs(), 240);

        // Out of this product's limits (long poll above its maximum, check-in
        // below its minimum, short above long): refused, values unchanged.
        for bad in [
            PollControlState {
                long_poll_interval: 241,
                ..state
            },
            PollControlState {
                check_in_interval: 479,
                ..state
            },
            PollControlState {
                short_poll_interval: 241,
                ..state
            },
            PollControlState {
                fast_poll_timeout: 0,
                ..state
            },
        ] {
            let mut c = PollControlCluster::with_config(PRODUCT);
            let before = c.state();
            assert!(!c.restore_state(bad));
            assert_eq!(c.state(), before);
        }
    }
}
