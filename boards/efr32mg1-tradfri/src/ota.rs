//! OTA staging through the resident Gecko Bootloader and external SPI flash.

use efr32mg1_hal::bootloader::Bootloader;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

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
    bootloader: Option<Bootloader>,
    slot_size: u32,
    written: u32,
    state: State,
}

impl Efr32FirmwareWriter {
    pub fn new() -> Result<Self, FirmwareError> {
        Ok(Self {
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
}
