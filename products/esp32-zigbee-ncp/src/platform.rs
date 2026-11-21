//! The ESP32 NCP's [`NcpPlatform`]: firmware update (`FW_*`, spec 7.9),
//! the persisted IEEE override (`SYS_SET_IEEE`, 9.4 rule 7) and the trial
//! boot window, the PTA coexistence follower a board attaches
//! ([`Platform::attach_coex`], spec 11.5) and the radio's MAC counters.
//!
//! # Boot sequence (composition root)
//!
//! 1. [`Platform::boot`] before the stack is composed: it reads the identity
//!    item ([`Platform::boot_ieee_override`], to be written to
//!    `MacExtendedAddress` and used as the active IEEE everywhere) and runs
//!    the `otadata` boot check ([`fw::check_boot`]).
//! 2. [`BootOutcome::RolledBack`]: the previous image is selected again —
//!    reset now.
//! 3. [`BootOutcome::Trial`]: this image runs on trial. The host must send
//!    `FW_CONFIRM` within [`TRIAL_WINDOW_MS`] (spec default 10 minutes);
//!    [`Platform::poll_trial`] rolls back when the window expires and the
//!    root resets. A reset of any kind before `FW_CONFIRM` also rolls back
//!    (the next boot finds the boot-attempt mark without a confirmation).
//!
//! # Signature policy
//!
//! Release builds accept only format-1 images in a signed MCUboot envelope
//! that verifies with one of the devkit product's `signed-ota` keys
//! (`products/esp32-zigbee-devkit/boot/app-verify.conf`, overridable with
//! `ESP32_DEVKIT_OTA_KEY{,2,3}_XY` at build time), whose version equals
//! `FW_BEGIN.version` and is not older than the running image. Unsigned
//! (plain ESP) images are `ACCESS_DENIED` unless the image is built with
//! the `dev-unsigned-fw` feature, which is also what "development build"
//! means for the downgrade flag. This authenticates what the NCP accepts
//! over ZRHP; the ESP-IDF bootloader itself boots whatever is in the slot
//! (no secure boot), so UART/ROM-bootloader flashing bypasses it.

use esp32_zigbee_devkit_product::ota::{BootCheck, BootCheckError, OtaFlash};
use ncp_coordinator_app::platform::{FwBegin, FwImage, FwSlot, NcpPlatform, PlatformError};
use zigbee_runtime::settings_journal::SettingsStore;

use crate::fw::{self, FwPolicy, FwUpdate};
use crate::identity::{IdentityError, IdentityItem};
use crate::radio::MacCounters;

/// Spec 7.9 default trial window.
pub const TRIAL_WINDOW_MS: u64 = 10 * 60 * 1_000;

/// Whether this image is a development build (`dev-unsigned-fw`).
pub const DEV_BUILD: bool = cfg!(feature = "dev-unsigned-fw");

/// What the boot check found; see the module documentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootOutcome {
    /// No unconfirmed image is running.
    Confirmed,
    /// This image runs on trial and needs `FW_CONFIRM`.
    Trial { slot: u8 },
    /// The bootloader fell back from an unconfirmed image to `slot`; the
    /// metadata now records it.
    FallbackRecorded { slot: u8 },
    /// The previous attempt of the trial image never confirmed: `slot` is
    /// selected again. The caller must reset.
    RolledBack { slot: u8 },
    /// No executing-slot evidence or a foreign partition table: `FW_*` is
    /// not offered.
    FwUnavailable(BootCheckError),
}

/// Result of [`Platform::poll_trial`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrialPoll {
    /// Nothing on trial.
    Idle,
    /// On trial, inside the window.
    Pending,
    /// The window expired without `FW_CONFIRM`. The previous slot is
    /// selected (`Some`), or the in-place rollback failed (`None`) and the
    /// next boot's check rolls back. Either way: reset now.
    Expired { rolled_back: Option<u8> },
}

/// PTA arbitration a board can attach ([`Platform::attach_coex`]; spec
/// 11.5, `crate::pta::Pta` on the Thread BR board).
pub trait CoexControl {
    /// Enable/disable with `coex_tx_priority_mask`; validates the mask even
    /// when disabling.
    fn set(&mut self, enable: bool, priority_mask: u8) -> Result<(), PlatformError>;
    /// Radio operations the arbiter broke off since boot or the last reset.
    fn counters(&self) -> CoexCounters;
    /// Zero [`Self::counters`].
    fn reset_counters(&mut self);
}

/// Radio operations the PTA arbiter broke off (`DIAG_GET_COUNTERS` MAC
/// group, spec 7.8 `0x0220..=0x0222`).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct CoexCounters {
    /// `coex_denied`: transmissions refused for lack of grant (each failed
    /// as a busy channel and retried by CSMA-CA, or a timed reply missed).
    pub tx_denied: u32,
    /// `coex_ack_rx_denied`: ACK receptions broken off after a transmission.
    pub ack_rx_denied: u32,
    /// `coex_ack_tx_denied`: automatic ACKs broken off.
    pub ack_tx_denied: u32,
}

/// The product platform over an OTA flash `F` and the identity journal `S`.
pub struct Platform<F: OtaFlash, S: SettingsStore> {
    fw: Option<FwUpdate<F>>,
    identity: IdentityItem<S>,
    boot_ieee: Option<[u8; 8]>,
    identity_error: Option<IdentityError>,
    trial_deadline_ms: Option<u64>,
    factory_ieee: Option<[u8; 8]>,
    coex: Option<&'static mut dyn CoexControl>,
    /// Free-running MAC counters of the radio backend and their values at
    /// the last `DIAG_RESET_COUNTERS`.
    mac_counters: Option<(fn() -> MacCounters, MacCounters)>,
}

impl<F: OtaFlash, S: SettingsStore> Platform<F, S> {
    /// Read the identity item and run the once-per-boot `otadata` check;
    /// `now_ms` starts the trial window.
    pub fn boot(
        mut flash: F,
        identity: S,
        policy: FwPolicy,
        trial_window_ms: u64,
        now_ms: u64,
    ) -> (Self, BootOutcome) {
        let mut identity = IdentityItem::new(identity);
        let (boot_ieee, identity_error) = match identity.ieee_override() {
            Ok(ieee) => (ieee, None),
            Err(error) => {
                log::error!("[ID] identity item unreadable: {:?}", error);
                (None, Some(error))
            }
        };
        let mut outcome = match fw::check_boot(&mut flash) {
            Ok(BootCheck::Confirmed) => BootOutcome::Confirmed,
            Ok(BootCheck::PendingVerify { slot }) => BootOutcome::Trial { slot },
            Ok(BootCheck::FallbackRecorded { slot }) => BootOutcome::FallbackRecorded { slot },
            Ok(BootCheck::RolledBack { slot }) => BootOutcome::RolledBack { slot },
            Err(error) => BootOutcome::FwUnavailable(error),
        };
        let fw = if matches!(outcome, BootOutcome::FwUnavailable(_)) {
            None
        } else {
            match FwUpdate::new(flash, policy) {
                Ok(fw) => Some(fw),
                Err((_, error)) => {
                    outcome = BootOutcome::FwUnavailable(error);
                    None
                }
            }
        };
        if let BootOutcome::FwUnavailable(error) = outcome {
            log::error!("[FW] firmware update unavailable: {:?}", error);
        }
        let trial_deadline_ms = matches!(outcome, BootOutcome::Trial { .. })
            .then(|| now_ms.saturating_add(trial_window_ms));
        (
            Self {
                fw,
                identity,
                boot_ieee,
                identity_error,
                trial_deadline_ms,
                factory_ieee: None,
                coex: None,
                mac_counters: None,
            },
            outcome,
        )
    }

    /// The IEEE override read at boot (`None` = factory EUI-64). A later
    /// `SYS_SET_IEEE` takes effect at the next boot.
    pub fn boot_ieee_override(&self) -> Option<[u8; 8]> {
        self.boot_ieee
    }

    /// The factory EUI-64 (eFuse), read by the composition root before the
    /// override is applied (`SYS_GET_INFO.ieee_factory`, `SYS_SET_IEEE FF…`).
    pub fn set_factory_ieee(&mut self, ieee: [u8; 8]) {
        self.factory_ieee = Some(ieee);
    }

    /// The board's PTA lines (`PTA_COEX`).
    pub fn attach_coex(&mut self, coex: &'static mut dyn CoexControl) {
        self.coex = Some(coex);
    }

    /// The radio backend's MAC counters (`DIAG_GET_COUNTERS` MAC group).
    pub fn attach_mac_counters(&mut self, read: fn() -> MacCounters) {
        self.mac_counters = Some((read, [0; 8]));
    }

    /// Why the identity item could not be read at boot, if it could not.
    pub fn identity_error(&self) -> Option<IdentityError> {
        self.identity_error
    }

    /// The firmware engine (tests, diagnostics).
    pub fn fw_mut(&mut self) -> Option<&mut FwUpdate<F>> {
        self.fw.as_mut()
    }

    pub fn identity_mut(&mut self) -> &mut IdentityItem<S> {
        &mut self.identity
    }

    /// Enforce the trial window; call from the main loop.
    pub fn poll_trial(&mut self, now_ms: u64) -> TrialPoll {
        let Some(deadline) = self.trial_deadline_ms else {
            return TrialPoll::Idle;
        };
        if now_ms < deadline {
            return TrialPoll::Pending;
        }
        let Some(fw) = self.fw.as_mut() else {
            return TrialPoll::Expired { rolled_back: None };
        };
        match fw::roll_back(fw.flash_mut()) {
            Ok(Some(slot)) => {
                log::warn!("[FW] trial window expired; slot {} selected", slot);
                TrialPoll::Expired {
                    rolled_back: Some(slot),
                }
            }
            Ok(None) => {
                // Confirmed in the meantime (nothing left to roll back).
                self.trial_deadline_ms = None;
                TrialPoll::Idle
            }
            Err(error) => {
                log::error!("[FW] trial rollback failed: {:?}", error);
                TrialPoll::Expired { rolled_back: None }
            }
        }
    }

    fn fw(&mut self) -> Result<&mut FwUpdate<F>, PlatformError> {
        self.fw.as_mut().ok_or(PlatformError::Unsupported)
    }
}

impl<F: OtaFlash, S: SettingsStore> NcpPlatform for Platform<F, S> {
    fn fw_begin(&mut self, image: &FwImage) -> Result<FwBegin, PlatformError> {
        if self.trial_deadline_ms.is_some() {
            return Err(PlatformError::InvalidState);
        }
        self.fw()?.begin(image)
    }

    fn fw_write(&mut self, offset: u32, data: &[u8]) -> Result<u32, PlatformError> {
        self.fw()?.write(offset, data)
    }

    fn fw_finish(&mut self) -> Result<u8, PlatformError> {
        self.fw()?.finish()
    }

    fn fw_activate(&mut self) -> Result<(), PlatformError> {
        self.fw()?.activate()
    }

    fn fw_confirm(&mut self) -> Result<(), PlatformError> {
        let fw = self.fw()?;
        match fw::confirm_boot(fw.flash_mut()) {
            Ok(confirmed) => {
                if confirmed {
                    log::info!("[FW] running image confirmed");
                }
                self.trial_deadline_ms = None;
                Ok(())
            }
            Err(error) => {
                log::error!("[FW] confirmation failed: {:?}", error);
                Err(PlatformError::Storage)
            }
        }
    }

    fn fw_abort(&mut self) -> Result<(), PlatformError> {
        self.fw()?.abort()
    }

    fn fw_slots(&mut self, out: &mut [FwSlot]) -> Result<usize, PlatformError> {
        let trial = self.trial_deadline_ms.is_some();
        Ok(self.fw()?.slots(trial, out))
    }

    fn fw_supported(&self) -> bool {
        self.fw.is_some()
    }

    fn fw_trial_pending(&self) -> bool {
        self.trial_deadline_ms.is_some()
    }

    fn factory_ieee(&self) -> Option<[u8; 8]> {
        self.factory_ieee
    }

    fn pta_supported(&self) -> bool {
        self.coex.is_some()
    }

    fn set_pta(&mut self, enable: bool, priority_mask: u8) -> Result<(), PlatformError> {
        match self.coex.as_mut() {
            Some(coex) => coex.set(enable, priority_mask),
            None => Err(PlatformError::Unsupported),
        }
    }

    fn tx_power_step(&self, dbm: i8) -> Option<i8> {
        Some(crate::radio::tx_power_step(dbm))
    }

    fn energy_to_dbm(&self, energy: u8) -> i8 {
        crate::radio::energy_to_dbm(energy)
    }

    fn lqi_to_dbm(&self, lqi: u8) -> i8 {
        crate::radio::lqi_to_dbm(lqi)
    }

    fn visit_counters(&self, group_mask: u8, sink: &mut dyn FnMut(u16, u32)) {
        if let (true, Some((read, base))) = (group_mask & 0x02 != 0, self.mac_counters) {
            for ((id, now), base) in (0x0201..).zip(read()).zip(base) {
                sink(id, now.wrapping_sub(base));
            }
        }
        if let (true, Some(coex)) = (group_mask & 0x02 != 0, self.coex.as_ref()) {
            let coex = coex.counters();
            for (id, value) in [
                (0x0220, coex.tx_denied),
                (0x0221, coex.ack_rx_denied),
                (0x0222, coex.ack_tx_denied),
            ] {
                sink(id, value);
            }
        }
    }

    fn reset_counters(&mut self, group_mask: u8) {
        if group_mask & 0x02 == 0 {
            return;
        }
        if let Some((read, base)) = self.mac_counters.as_mut() {
            *base = read();
        }
        if let Some(coex) = self.coex.as_mut() {
            coex.reset_counters();
        }
    }

    fn ieee_rewritable(&self) -> bool {
        !matches!(self.identity_error, Some(IdentityError::UnknownFormat(_)))
    }

    fn set_ieee_override(&mut self, ieee: Option<[u8; 8]>) -> Result<(), PlatformError> {
        match self.identity.set_ieee_override(ieee) {
            Ok(written) => {
                if written {
                    log::info!("[ID] IEEE override stored; applies at the next boot");
                }
                Ok(())
            }
            Err(IdentityError::InvalidIeee) => Err(PlatformError::InvalidParameter),
            Err(error) => {
                log::error!("[ID] IEEE override not stored: {:?}", error);
                Err(PlatformError::Storage)
            }
        }
    }
}

/// The hardware platform: whole-chip OTA flash and the `zbnv` identity
/// journal.
#[cfg(target_os = "none")]
pub type EspPlatform = Platform<crate::fw_flash::NcpOtaFlash, crate::stores::IdentityStore>;

/// The product policy: the devkit's `signed-ota` keys and
/// [`DEV_BUILD`].
pub fn product_policy(running_version: u32) -> FwPolicy {
    FwPolicy {
        keys: mcuboot_image::KeySet::new(&esp32_zigbee_devkit_product::signed_ota::TRUSTED_KEYS),
        running_version,
        dev_build: DEV_BUILD,
    }
}

/// [`Platform::boot`] on the hardware with [`product_policy`] and
/// [`TRIAL_WINDOW_MS`].
#[cfg(target_os = "none")]
pub fn boot(running_version: u32, now_ms: u64) -> (EspPlatform, BootOutcome) {
    Platform::boot(
        crate::fw_flash::NcpOtaFlash::new(),
        crate::stores::identity_store(),
        product_policy(running_version),
        TRIAL_WINDOW_MS,
        now_ms,
    )
}

#[cfg(test)]
#[path = "platform_tests.rs"]
mod tests;
