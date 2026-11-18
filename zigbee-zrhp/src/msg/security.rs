//! SECURITY group (`0x04xx`, endpoint ZIGBEE), spec 7.5.

use crate::codec::{Bytes8, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::types::{Ieee, Key};

/// `SEC_LOCK_KEY_EXPORT` confirm word ("LOCK").
pub const LOCK_CONFIRM: u32 = 0x4B43_4F4C;

wire_struct! {
    /// `SEC_GET_NETWORK_KEY` request.
    pub struct SecGetNetworkKey {
        /// 0 active, 1 alternate.
        pub which: u8,
    }
}

wire_struct! {
    /// `SEC_GET_NETWORK_KEY` response body.
    pub struct SecGetNetworkKeyRsp {
        /// Key.
        pub key: Key,
        /// Sequence.
        pub seq: u8,
        /// Durable NWK frame-counter limit.
        pub nwk_fc_limit: u32,
    }
}

wire_struct! {
    /// `SEC_SET_TC_POLICY` request.
    pub struct SecSetTcPolicy {
        /// 0 deny, 1 global TCLK, 2 install code only.
        pub allow_join: u8,
        /// Require TCLK update.
        pub require_tclk_update: u8,
        /// Allow TC rejoin.
        pub allow_tc_rejoin: u8,
        /// Allow application link keys.
        pub allow_app_link_keys: u8,
        /// Join timeout.
        pub join_timeout_s: u16,
    }
}

wire_struct! {
    /// `SEC_ADD_INSTALL_CODE` request (code includes its CRC, LE).
    pub struct SecAddInstallCode<'a> {
        /// Device IEEE.
        pub ieee: Ieee,
        /// Install code (8/10/14/18 bytes).
        pub code: Bytes8<'a>,
    }
}

wire_struct! {
    /// `SEC_ADD_INSTALL_CODE` response body.
    pub struct SecAddInstallCodeRsp {
        /// Derived link key.
        pub derived_key: Key,
    }
}

wire_struct! {
    /// `SEC_ADD_LINK_KEY` request.
    pub struct SecAddLinkKey {
        /// Device IEEE.
        pub ieee: Ieee,
        /// Key.
        pub key: Key,
        /// 1 TCLK unique, 3 application.
        pub key_type: u8,
        /// Attributes.
        pub attrs: u8,
        /// Incoming frame-counter floor.
        pub rx_floor: u32,
        /// Bit0 temporary until join.
        pub flags: u8,
    }
}

wire_struct! {
    /// `SEC_REMOVE_LINK_KEY` request.
    pub struct SecRemoveLinkKey {
        /// Device IEEE.
        pub ieee: Ieee,
    }
}

wire_struct! {
    /// `SEC_GET_LINK_KEY` request.
    pub struct SecGetLinkKey {
        /// Device IEEE.
        pub ieee: Ieee,
    }
}

wire_struct! {
    /// `SEC_DERIVE_EZSP_KEY` request.
    pub struct SecDeriveEzspKey {
        /// Device IEEE.
        pub ieee: Ieee,
    }
}

wire_struct! {
    /// `SEC_ROTATE_NETWORK_KEY` request.
    pub struct SecRotateNetworkKey {
        /// New key (0 = random).
        pub key: Key,
        /// 0 broadcast, 1 unicast to routers.
        pub mode: u8,
        /// Switch delay.
        pub switch_delay_s: u16,
    }
}

wire_struct! {
    /// `SEC_GET_LINK_KEY` response body.
    pub struct SecGetLinkKeyRsp {
        /// Key.
        pub key: Key,
        /// Type.
        pub key_type: u8,
        /// Attributes.
        pub attrs: u8,
        /// Global APS limit.
        pub tx: u32,
        /// Incoming floor.
        pub rx_floor: u32,
    }
}

wire_struct! {
    /// `SEC_GET_FRAME_COUNTERS` response body.
    pub struct SecGetFrameCountersRsp {
        /// NWK current.
        pub nwk_current: u32,
        /// NWK durable limit.
        pub nwk_limit: u32,
        /// APS current.
        pub aps_current: u32,
        /// APS durable limit.
        pub aps_limit: u32,
        /// Reservation stride.
        pub stride: u32,
    }
}

wire_struct! {
    /// `SEC_DERIVE_EZSP_KEY` response body.
    pub struct SecDeriveEzspKeyRsp {
        /// Derived key.
        pub key: Key,
    }
}

wire_struct! {
    /// `SEC_LOCK_KEY_EXPORT` request.
    pub struct SecLockKeyExport {
        /// [`LOCK_CONFIRM`].
        pub confirm: u32,
    }
}

wire_struct! {
    /// `EVT_LINK_KEY_UPDATED`.
    pub struct EvtLinkKeyUpdated {
        /// IEEE.
        pub ieee: Ieee,
        /// Short address.
        pub nwk: u16,
        /// Key type.
        pub key_type: u8,
        /// 0 provisional, 1 unverified, 2 verified.
        pub attrs: u8,
        /// Origin.
        pub origin: u8,
    }
}

wire_struct! {
    /// `EVT_KEY_FAILURE`.
    pub struct EvtKeyFailure {
        /// IEEE.
        pub ieee: Ieee,
        /// Short address.
        pub nwk: u16,
        /// 1 verify failed, 2 request denied, 3 no key, 4 MIC failure burst.
        pub reason: u8,
    }
}

wire_struct! {
    /// `EVT_NETWORK_KEY_SWITCHED`.
    pub struct EvtNetworkKeySwitched {
        /// New sequence.
        pub new_seq: u8,
        /// Request TID.
        pub req_tid: u16,
    }
}

wire_struct! {
    /// `EVT_FC_CHECKPOINT`.
    pub struct EvtFcCheckpoint {
        /// 1 NWK, 2 APS.
        pub domain: u8,
        /// Key sequence.
        pub key_seq: u8,
        /// New durable limit.
        pub durable_limit: u32,
        /// Journal generation.
        pub generation: u32,
    }
}

message_def! {
    SecGetNetworkKey = ids::SEC_GET_NETWORK_KEY, Request;
    SecGetNetworkKeyRsp = ids::SEC_GET_NETWORK_KEY, Response;
    SecSetTcPolicy = ids::SEC_SET_TC_POLICY, Request;
    SecAddInstallCode<'a> = ids::SEC_ADD_INSTALL_CODE, Request;
    SecAddInstallCodeRsp = ids::SEC_ADD_INSTALL_CODE, Response;
    SecAddLinkKey = ids::SEC_ADD_LINK_KEY, Request;
    SecRemoveLinkKey = ids::SEC_REMOVE_LINK_KEY, Request;
    SecRotateNetworkKey = ids::SEC_ROTATE_NETWORK_KEY, Request;
    SecGetLinkKey = ids::SEC_GET_LINK_KEY, Request;
    SecDeriveEzspKey = ids::SEC_DERIVE_EZSP_KEY, Request;
    SecGetLinkKeyRsp = ids::SEC_GET_LINK_KEY, Response;
    SecGetFrameCountersRsp = ids::SEC_GET_FRAME_COUNTERS, Response;
    SecDeriveEzspKeyRsp = ids::SEC_DERIVE_EZSP_KEY, Response;
    SecLockKeyExport = ids::SEC_LOCK_KEY_EXPORT, Request;
    EvtLinkKeyUpdated = ids::EVT_LINK_KEY_UPDATED, Event;
    EvtKeyFailure = ids::EVT_KEY_FAILURE, Event;
    EvtNetworkKeySwitched = ids::EVT_NETWORK_KEY_SWITCHED, Event;
    EvtFcCheckpoint = ids::EVT_FC_CHECKPOINT, Event;
}
