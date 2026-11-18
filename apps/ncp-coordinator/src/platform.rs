//! Product services behind the NCP that are not the Zigbee stack or the
//! host byte stream: firmware update (`FW_*`, spec 7.9), the persisted IEEE
//! override (`SYS_SET_IEEE`, 9.4 rule 7) and PTA coexistence (`coex_*`
//! config, 11.5).
//!
//! Every method has an explicit "unsupported" default so a product only
//! implements what it really has; the NCP then answers
//! `UNSUPPORTED_COMMAND` and leaves the capability bit clear.

/// Why a platform service refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformError {
    /// Not built into this product (`UNSUPPORTED_COMMAND`).
    Unsupported,
    /// Not allowed in the current state (`INVALID_STATE`).
    InvalidState,
    /// Malformed or out-of-range argument (`INVALID_PARAMETER`).
    InvalidParameter,
    /// Larger than the slot (`TOO_LARGE`).
    TooLarge,
    /// Different bytes at an already written offset (`CONFLICT`).
    Conflict,
    /// Flash/NV failure (`STORAGE_ERROR`).
    Storage,
    /// Unsigned or wrongly signed image, or a refused downgrade
    /// (`ACCESS_DENIED`).
    AccessDenied,
    /// Length/hash/image check failed (`SECURITY_ERROR`).
    VerifyFailed,
}

/// `FW_BEGIN` result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwBegin {
    pub max_chunk: u16,
    /// Slot being written (0/1 = `ota_0`/`ota_1`).
    pub slot: u8,
}

/// `FW_BEGIN` request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwImage {
    pub image_size: u32,
    /// 1 ESP app image (+ MCUboot TLVs), 2 MCUboot.
    pub format: u8,
    pub sha256: [u8; 32],
    pub version: u32,
    /// bit0 allow downgrade (development builds only).
    pub flags: u8,
}

/// `FW_GET_SLOTS` entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwSlot {
    pub slot: u8,
    /// 0 empty/invalid, 1 running confirmed, 2 running on trial, 3 staged
    /// (verified, not active), 4 valid inactive.
    pub state: u8,
    pub version: u32,
    pub sha256: [u8; 32],
}

/// `FwSlot::state` values.
pub mod slot_state {
    pub const EMPTY: u8 = 0;
    pub const RUNNING: u8 = 1;
    pub const RUNNING_TRIAL: u8 = 2;
    pub const STAGED: u8 = 3;
    pub const INACTIVE: u8 = 4;
}

/// Product services; see the module documentation.
pub trait NcpPlatform {
    // ── Firmware (spec 7.9) ─────────────────────────────────────────

    /// Open a staging transaction for the inactive slot. Erases lazily.
    fn fw_begin(&mut self, _image: &FwImage) -> Result<FwBegin, PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Sequential write; an identical rewrite below `next_offset` succeeds
    /// without writing, different bytes are `Conflict`. Returns the next
    /// expected offset.
    fn fw_write(&mut self, _offset: u32, _data: &[u8]) -> Result<u32, PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Verify length, SHA-256, image format and the signature policy.
    /// `Ok(0)` = verified.
    fn fw_finish(&mut self) -> Result<u8, PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Mark the verified image as a trial boot; the NCP reboots afterwards.
    fn fw_activate(&mut self) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Confirm the running trial image (no-op when it is already confirmed).
    fn fw_confirm(&mut self) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Discard the staging transaction.
    fn fw_abort(&mut self) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Describe up to `out.len()` slots; returns how many were filled.
    fn fw_slots(&mut self, _out: &mut [FwSlot]) -> Result<usize, PlatformError> {
        Err(PlatformError::Unsupported)
    }
    /// Whether `FW_*` is built in (`FW_UPDATE` capability).
    fn fw_supported(&self) -> bool {
        false
    }
    /// The running image is a trial that still needs `FW_CONFIRM`.
    fn fw_trial_pending(&self) -> bool {
        false
    }

    // ── IEEE override (spec 7.1 SYS_SET_IEEE, 9.4 rule 7) ───────────

    /// Whether a persisted override is supported (`IEEE_REWRITABLE`).
    fn ieee_rewritable(&self) -> bool {
        false
    }
    /// Persist (`Some`) or remove (`None`) the override. It takes effect
    /// at the next boot (the composition root applies it to the MAC before
    /// the stack starts).
    fn set_ieee_override(&mut self, _ieee: Option<[u8; 8]>) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported)
    }

    // ── PTA coexistence (config 0x0009/0x000A, spec 11.5) ───────────

    /// Whether the board has PTA lines and the product drives them
    /// (`PTA_COEX` capability).
    fn pta_supported(&self) -> bool {
        false
    }
    /// Enable/disable PTA arbitration (volatile) with the
    /// `coex_tx_priority_mask` bits.
    fn set_pta(&mut self, _enable: bool, _priority_mask: u8) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported)
    }
}

/// A product with none of the optional services.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoPlatform;

impl NcpPlatform for NoPlatform {}
