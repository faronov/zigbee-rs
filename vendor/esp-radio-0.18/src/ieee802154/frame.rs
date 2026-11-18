use alloc::vec::Vec;

use ieee802154::mac::{FrameContent, Header};

pub(crate) const FRAME_SIZE: usize = 129;
// Local patch `ieee802154-fcf-offsets`: the FCF readers live in `fcf.rs`.
pub(crate) use super::fcf::{
    FRAME_VERSION_1, FRAME_VERSION_2, frame_get_version, frame_is_ack_required,
};

/// IEEE 802.15.4 MAC frame
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Header
    pub header: Header,
    /// Content
    pub content: FrameContent,
    /// Payload
    pub payload: Vec<u8>,
    /// This is a 2-byte CRC checksum
    pub footer: [u8; 2],
}

/// IEEE 802.15.4 MAC frame which has been received
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedFrame {
    /// Frame
    pub frame: Frame,
    /// Receiver channel
    pub channel: u8,
    /// Received Signal Strength Indicator (RSSI)
    pub rssi: i8,
    /// Link Quality Indication (LQI)
    pub lqi: u8,
}
