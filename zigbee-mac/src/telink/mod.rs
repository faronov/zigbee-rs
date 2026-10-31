//! Pure-Rust TLSR8258 MAC backend.
//!
//! [`TelinkRadioPhy`] adapts the hardware-proven `tlsr8258-hal` radio to the
//! portable [`crate::RadioPhy`] contract. Scanning, association, polling,
//! CSMA-CA parameters, frame retries, ACK matching and the parent role run in
//! the shared [`crate::SoftMacCore`]; the HAL performs the CSMA backoff,
//! transmission and Imm-Ack wait in one radio operation (`type Tx = FullTx`).
//!
//! - [`TelinkMac`] — `SoftMacCore<TelinkRadioPhy, EndDevice>`, used by every
//!   sensor image.
//! - [`TelinkRouterMac`] — `SoftMacCore<TelinkRadioPhy, Parent<8, 8, 0>>`,
//!   used by the always-on router image.
//!
//! The turnaround-critical Imm-Ack stays in the HAL RX-completion path
//! (`ram_code`, also run from the RF IRQ when `macRxOnWhenIdle` is set), so
//! the PHY reports `hardware_auto_ack` and the core never sends a
//! task-context ACK. A router arms Frame Pending in the HAL's source-match
//! table through [`crate::RadioPhy::set_ack_frame_pending`].

#[cfg(target_arch = "tc32")]
mod radio_phy;

#[cfg(all(
    target_arch = "tc32",
    feature = "hardware-aes",
    feature = "telink-retention-proof"
))]
pub use radio_phy::TelinkRetentionResumeError;
#[cfg(target_arch = "tc32")]
pub use radio_phy::{
    TELINK_PENDING_RX, TELINK_ROUTER_PENDING_RX, TelinkMac, TelinkRadioPhy, TelinkRouterMac,
    TelinkRouterRole,
};
