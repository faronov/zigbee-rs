//! LOG group (`0x09xx`, endpoint LOG), spec 7.10.

use crate::codec::{Rest, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;

/// Maximum log text length (decision 12).
pub const MAX_LOG_TEXT: usize = 200;

wire_struct! {
    /// `LOG_CONFIG` request.
    pub struct LogConfig {
        /// 0 off … 5 trace.
        pub level: u8,
        /// Module mask.
        pub module_mask: u32,
    }
}

wire_struct! {
    /// `EVT_LOG_RECORD`: UTF-8 text, ≤ 200 B, no terminator.
    pub struct EvtLogRecord<'a> {
        /// NCP time.
        pub time_ms: u32,
        /// 1 error … 5 trace.
        pub level: u8,
        /// Module.
        pub module: u8,
        /// Record sequence.
        pub seq: u16,
        /// Text (use [`Rest::as_str`]).
        pub text: Rest<'a>,
    }
}

wire_struct! {
    /// `EVT_LOG_DROPPED`.
    pub struct EvtLogDropped {
        /// Dropped records.
        pub count: u32,
        /// First dropped sequence.
        pub first_seq: u16,
    }
}

message_def! {
    LogConfig = ids::LOG_CONFIG, Request;
    EvtLogRecord<'a> = ids::EVT_LOG_RECORD, Event;
    EvtLogDropped = ids::EVT_LOG_DROPPED, Event;
}
