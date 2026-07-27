//! TI TOPsm firmware images imported as build-time data.
//!
//! The host-side radio driver is Rust. The PBE, MCE, and RFE processors inside
//! the CC2340 still require TI's IEEE 802.15.4 firmware images. `build.rs`
//! imports the BSD-licensed arrays from an installed SimpleLink Low Power F3
//! SDK when `CC2340_SDK_DIR` is set.

include!(concat!(env!("OUT_DIR"), "/cc2340_firmware.rs"));

pub(crate) struct FirmwareImages {
    pub pbe: &'static [u32],
    pub mce: &'static [u32],
    pub rfe: &'static [u32],
}

pub(crate) fn images() -> Option<FirmwareImages> {
    FIRMWARE_AVAILABLE.then_some(FirmwareImages {
        pbe: PBE_IMAGE,
        mce: MCE_IMAGE,
        rfe: RFE_IMAGE,
    })
}

pub(crate) fn source() -> &'static str {
    FIRMWARE_SOURCE
}
