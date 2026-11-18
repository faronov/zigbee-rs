//! FIRMWARE group (`0x08xx`, endpoint BULK), spec 7.9: NCP firmware update.
//! `FW_FINISH`, `FW_CONFIRM`, `FW_ABORT` and `FW_GET_SLOTS` requests have
//! no payload.

use crate::codec::{List, Rest, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;

wire_struct! {
    /// `FW_BEGIN` request.
    pub struct FwBegin {
        /// Image size.
        pub image_size: u32,
        /// 1 ESP app image + MCUboot TLVs, 2 MCUboot.
        pub format: u8,
        /// SHA-256 of the image.
        pub sha256: [u8; 32],
        /// Image version.
        pub version: u32,
        /// Bit0 allow downgrade (dev builds only).
        pub flags: u8,
    }
}

wire_struct! {
    /// `FW_BEGIN` response body.
    pub struct FwBeginRsp {
        /// Largest `FW_WRITE` data.
        pub max_chunk: u16,
        /// Target slot.
        pub slot: u8,
    }
}

wire_struct! {
    /// `FW_WRITE` request.
    pub struct FwWrite<'a> {
        /// Offset.
        pub offset: u32,
        /// Data.
        pub data: Rest<'a>,
    }
}

wire_struct! {
    /// `FW_WRITE` response body.
    pub struct FwWriteRsp {
        /// Next expected offset.
        pub next_offset: u32,
    }
}

wire_struct! {
    /// `FW_FINISH` response body.
    pub struct FwFinishRsp {
        /// Verification result.
        pub verify_result: u8,
    }
}

wire_struct! {
    /// `FW_ACTIVATE` request.
    pub struct FwActivate {
        /// Delay before reboot.
        pub delay_ms: u16,
    }
}

wire_struct! {
    /// One firmware slot.
    pub struct FwSlot {
        /// Slot.
        pub slot: u8,
        /// State.
        pub state: u8,
        /// Version.
        pub version: u32,
        /// SHA-256.
        pub sha256: [u8; 32],
    }
}

wire_struct! {
    /// `FW_GET_SLOTS` response body.
    pub struct FwGetSlotsRsp<'a> {
        /// Slots.
        pub slots: List<'a, FwSlot>,
    }
}

message_def! {
    FwBegin = ids::FW_BEGIN, Request;
    FwBeginRsp = ids::FW_BEGIN, Response;
    FwWrite<'a> = ids::FW_WRITE, Request;
    FwWriteRsp = ids::FW_WRITE, Response;
    FwFinishRsp = ids::FW_FINISH, Response;
    FwActivate = ids::FW_ACTIVATE, Request;
    FwGetSlotsRsp<'a> = ids::FW_GET_SLOTS, Response;
}
