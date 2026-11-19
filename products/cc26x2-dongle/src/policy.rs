//! Router lifecycle policy.

/// The dongle has no button to request commissioning after a Leave (or a
/// coordinator's Remove Device), so Network Steering is re-armed after this
/// hold-off. It outlasts ZHA's default 254 s permit-join window, so a removed
/// router does not immediately rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;

const DEFAULT_PERMIT_JOIN_WINDOW_MS: u64 = 254 * 1000;
const _: () = assert!(RECOMMISSION_AFTER_LEAVE_MS > DEFAULT_PERMIT_JOIN_WINDOW_MS);
