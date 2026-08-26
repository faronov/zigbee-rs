//! Product OTA staging through the resident Gecko Bootloader slot.

use efr32mg1_hal::bootloader::Bootloader;
use efr32mg1_tradfri::resources::BootloaderFlashAccess;
use sensor_sed_app::{
    OtaActivationOutcome as AppOtaActivationOutcome, OtaEventOutcome as AppOtaEventOutcome,
    OtaLifecycle, OtaServiceOutcome, is_ota_event,
};
use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::ota_transport::{OtaEventOutcome as RuntimeOtaEventOutcome, OtaSession};
use zigbee_runtime::profile::{ApplicationProfile, WithOta};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::ENDPOINT;
use crate::policy::OTA_KEEP_AWAKE_MS;

const OTA_SLOT: u32 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Erased,
    Verified,
}

/// Firmware writer for the TRADFRI module's resident Gecko Bootloader slot.
///
/// The writer keeps the bootloader initialized only while an OTA transaction
/// is active. `activate()` does not return: it requests a bootloader reset and
/// installs the verified GBL from slot 0.
pub struct Efr32FirmwareWriter {
    _flash_access: BootloaderFlashAccess,
    bootloader: Option<Bootloader>,
    slot_size: u32,
    written: u32,
    state: State,
}

impl Efr32FirmwareWriter {
    /// Discover the resident bootloader while retaining exclusive ownership of
    /// the external-flash path for the lifetime of this writer.
    pub fn new(flash_access: BootloaderFlashAccess) -> Result<Self, FirmwareError> {
        Ok(Self {
            _flash_access: flash_access,
            bootloader: Some(Bootloader::discover().map_err(|_| FirmwareError::HardwareError)?),
            slot_size: 0,
            written: 0,
            state: State::Idle,
        })
    }

    pub const fn bytes_written(&self) -> u32 {
        self.written
    }

    /// Read staged bytes back from slot 0 for diagnostics.
    pub fn read_block(&mut self, offset: u32, data: &mut [u8]) -> Result<(), FirmwareError> {
        self.initialize()?;
        let length = u32::try_from(data.len()).map_err(|_| FirmwareError::OutOfRange)?;
        if offset
            .checked_add(length)
            .is_none_or(|end| end > self.slot_size)
        {
            return Err(FirmwareError::OutOfRange);
        }
        self.bootloader()?
            .read_slot(OTA_SLOT, offset, data)
            .map_err(|_| FirmwareError::HardwareError)
    }

    fn bootloader(&mut self) -> Result<&mut Bootloader, FirmwareError> {
        self.bootloader.as_mut().ok_or(FirmwareError::HardwareError)
    }

    fn initialize(&mut self) -> Result<(), FirmwareError> {
        let slot_size = {
            let bootloader = self.bootloader()?;
            bootloader
                .init()
                .map_err(|_| FirmwareError::HardwareError)?;
            bootloader
                .storage_slot(OTA_SLOT)
                .map_err(|_| FirmwareError::HardwareError)?
                .length
        };
        if slot_size == 0 {
            return Err(FirmwareError::HardwareError);
        }
        self.slot_size = slot_size;
        Ok(())
    }
}

impl FirmwareWriter for Efr32FirmwareWriter {
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        self.initialize()?;
        let bootloader = self.bootloader()?;
        bootloader
            .clear_bootload_list()
            .map_err(|_| FirmwareError::EraseFailed)?;
        bootloader
            .erase_slot(OTA_SLOT)
            .map_err(|_| FirmwareError::EraseFailed)?;
        self.written = 0;
        self.state = State::Erased;
        Ok(())
    }

    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
        let end = validate_write(self.state, self.slot_size, self.written, offset, data.len())?;
        self.bootloader()?
            .write_slot(OTA_SLOT, offset, data)
            .map_err(|_| FirmwareError::WriteFailed)?;
        self.written = end;
        Ok(())
    }

    fn verify(
        &mut self,
        expected_size: u32,
        expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError> {
        if self.state != State::Erased || self.written != expected_size || expected_hash.is_some() {
            return Err(FirmwareError::VerifyFailed);
        }
        self.bootloader()?
            .verify_gbl_slot(OTA_SLOT)
            .map_err(|_| FirmwareError::VerifyFailed)?;
        self.state = State::Verified;
        Ok(())
    }

    fn activate(&mut self) -> Result<(), FirmwareError> {
        if self.state != State::Verified {
            return Err(FirmwareError::ActivateFailed);
        }
        let bootloader = self
            .bootloader
            .as_mut()
            .ok_or(FirmwareError::ActivateFailed)?;
        let mut slots = [OTA_SLOT as i32];
        bootloader
            .set_bootload_list(&mut slots)
            .map_err(|_| FirmwareError::ActivateFailed)?;

        let bootloader = self
            .bootloader
            .take()
            .ok_or(FirmwareError::ActivateFailed)?;
        bootloader.reboot_and_install()
    }

    fn slot_size(&self) -> u32 {
        self.slot_size
    }

    fn abort(&mut self) -> Result<(), FirmwareError> {
        if self.state == State::Idle {
            return Ok(());
        }
        if let Some(bootloader) = self.bootloader.as_mut() {
            bootloader
                .clear_bootload_list()
                .map_err(|_| FirmwareError::HardwareError)?;
            bootloader
                .deinit()
                .map_err(|_| FirmwareError::HardwareError)?;
        }
        self.written = 0;
        self.state = State::Idle;
        Ok(())
    }
}

/// OTA transport paired with this product's mandatory `WithOta` profile.
///
/// `handle_event` and `service` only report that activation is pending.
/// `SensorApp` checkpoints the security journal before it calls
/// [`OtaLifecycle::activate`], which may reset into the resident bootloader.
pub struct Efr32OtaLifecycle {
    session: OtaSession,
}

impl Efr32OtaLifecycle {
    pub const fn new() -> Self {
        Self {
            session: OtaSession::new(),
        }
    }

    fn log_status(status: Option<&StackEvent>) {
        match status {
            Some(StackEvent::OtaImageAvailable { version, size }) => {
                rtt_target::rprintln!("[EFR32][ota] IMAGE version=0x{:08X} size={}", version, size);
            }
            Some(StackEvent::OtaProgress { percent }) => {
                rtt_target::rprintln!("[EFR32][ota] PROGRESS {}%", percent);
            }
            Some(StackEvent::OtaDelayedActivation { delay_secs }) => {
                rtt_target::rprintln!("[EFR32][ota] ACTIVATE_DELAY {}s", delay_secs);
            }
            Some(StackEvent::OtaComplete) => {
                rtt_target::rprintln!("[EFR32][ota] VERIFIED_CHECKPOINT_PENDING");
            }
            Some(StackEvent::OtaFailed) => {
                rtt_target::rprintln!("[EFR32][ota] FAILED");
            }
            _ => {}
        }
    }
}

impl Default for Efr32OtaLifecycle {
    fn default() -> Self {
        Self::new()
    }
}

impl<M, S, P> OtaLifecycle<M, S, WithOta<P, Efr32FirmwareWriter>> for Efr32OtaLifecycle
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    const ENABLED: bool = true;

    fn is_active(&self, profile: &WithOta<P, Efr32FirmwareWriter>) -> bool {
        OtaSession::is_active(profile.ota())
    }

    fn next_deadline_ms(&self, _profile: &WithOta<P, Efr32FirmwareWriter>) -> Option<u32> {
        // OtaManager advances in whole seconds. While it is active,
        // SensorApp already selects the product's 250 ms fast-poll cadence.
        None
    }

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, Efr32FirmwareWriter>>,
        event: &StackEvent,
    ) -> AppOtaEventOutcome {
        let outcome = {
            let (device, profile) = node.device_and_profile_mut();
            self.session
                .handle_event(device, profile.ota_mut(), ENDPOINT, event)
                .await
        };

        match outcome {
            RuntimeOtaEventOutcome::NotOta if !is_ota_event(event) => {
                AppOtaEventOutcome::NotHandled
            }
            RuntimeOtaEventOutcome::NotOta => {
                Self::log_status(Some(event));
                AppOtaEventOutcome::Handled {
                    keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                    activation_pending: self.session.activation_pending()
                        || matches!(event, StackEvent::OtaComplete),
                }
            }
            RuntimeOtaEventOutcome::Ignored => AppOtaEventOutcome::Handled {
                keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                activation_pending: self.session.activation_pending(),
            },
            RuntimeOtaEventOutcome::Consumed(status) => {
                Self::log_status(status.as_ref());
                AppOtaEventOutcome::Handled {
                    keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                    activation_pending: self.session.activation_pending(),
                }
            }
        }
    }

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, Efr32FirmwareWriter>>,
        elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        let status = {
            let (device, profile) = node.device_and_profile_mut();
            self.session
                .service(device, profile.ota_mut(), elapsed_secs)
                .await
        };
        Self::log_status(status.as_ref());

        OtaServiceOutcome {
            keep_awake_ms: status.as_ref().map(|_| OTA_KEEP_AWAKE_MS),
            activation_pending: self.session.activation_pending(),
        }
    }

    fn activate(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, Efr32FirmwareWriter>>,
    ) -> AppOtaActivationOutcome {
        // SensorApp has just durably checkpointed security state. Success
        // requests a bootloader reset and therefore does not return.
        match self.session.activate(node.profile_mut().ota_mut()) {
            Ok(()) => AppOtaActivationOutcome::Activated,
            Err(_) => {
                rtt_target::rprintln!("[EFR32][ota] ACTIVATE_FAILED");
                AppOtaActivationOutcome::Failed
            }
        }
    }
}

fn validate_write(
    state: State,
    slot_size: u32,
    written: u32,
    offset: u32,
    length: usize,
) -> Result<u32, FirmwareError> {
    if state != State::Erased || offset != written {
        return Err(FirmwareError::WriteFailed);
    }
    let length = u32::try_from(length).map_err(|_| FirmwareError::OutOfRange)?;
    let end = offset
        .checked_add(length)
        .ok_or(FirmwareError::OutOfRange)?;
    if end > slot_size {
        return Err(FirmwareError::OutOfRange);
    }
    Ok(end)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_byte_granular_sequential_writes() {
        assert_eq!(validate_write(State::Erased, 256, 0, 0, 61), Ok(61));
        assert_eq!(validate_write(State::Erased, 256, 61, 61, 3), Ok(64));
    }

    #[test]
    fn rejects_writes_before_erase_or_out_of_order() {
        assert_eq!(
            validate_write(State::Idle, 256, 0, 0, 1),
            Err(FirmwareError::WriteFailed)
        );
        assert_eq!(
            validate_write(State::Erased, 256, 64, 63, 1),
            Err(FirmwareError::WriteFailed)
        );
    }

    #[test]
    fn rejects_slot_overflow() {
        assert_eq!(
            validate_write(State::Erased, 64, 63, 63, 2),
            Err(FirmwareError::OutOfRange)
        );
    }

    #[test]
    fn ota_lifecycle_starts_without_pending_activation() {
        let lifecycle = Efr32OtaLifecycle::new();
        assert!(!lifecycle.session.activation_pending());
    }
}
