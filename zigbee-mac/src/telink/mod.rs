//! Pure-Rust TLSR8258 MAC backends.
//!
//! [`TelinkRadioPhy`] adapts the hardware-proven `tlsr8258-hal` radio to the
//! portable [`crate::RadioPhy`] contract. Scanning, association, polling,
//! CSMA-CA parameters, frame retries and ACK matching run in the shared
//! [`crate::SoftMacCore`]; the HAL performs the CSMA backoff, transmission
//! and Imm-Ack wait in one radio operation (`type Tx = FullTx`).
//!
//! - [`TelinkMac`] — `SoftMacCore<TelinkRadioPhy, EndDevice>`, used by every
//!   sensor image.
//! - `TelinkRouterMac` — the legacy hand-written parent MAC, compiled only
//!   with the `telink-legacy-router` feature for the always-on router image.
//!   It stays until the shared core's parent role fits the router's flash
//!   budget; the router is then re-ported onto
//!   `SoftMacCore<TelinkRadioPhy, Parent<..>>` (the PHY already implements
//!   [`crate::RadioPhy::set_ack_frame_pending`] for that).
//!
//! The turnaround-critical Imm-Ack stays in the HAL RX-completion path
//! (`ram_code`, also run from the RF IRQ when `macRxOnWhenIdle` is set), so
//! the PHY reports `hardware_auto_ack` and the core never sends a
//! task-context ACK.

// Host builds only exercise the frame/queue helpers in unit tests; the
// tc32-only `imp` module is their sole production user.
#[cfg(any(feature = "telink-legacy-router", test))]
#[cfg_attr(not(target_arch = "tc32"), allow(dead_code))]
mod legacy;
#[cfg(target_arch = "tc32")]
mod radio_phy;

#[cfg(all(target_arch = "tc32", feature = "telink-legacy-router"))]
pub use legacy::{AckDiagnostics, MacQueueDiagnostics, TelinkRouterMac};
#[cfg(all(
    target_arch = "tc32",
    feature = "hardware-aes",
    feature = "telink-retention-proof"
))]
pub use radio_phy::TelinkRetentionResumeError;
#[cfg(target_arch = "tc32")]
pub use radio_phy::{TELINK_PENDING_RX, TelinkMac, TelinkRadioPhy};
