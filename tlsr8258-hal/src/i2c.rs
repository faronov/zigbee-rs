//! Blocking TLSR8258 I2C master with validated pin groups and bounded waits.
//!
//! The command sequence follows Telink's 8258 driver: address phases use
//! `START | ID`, writes use the shared data register plus `DI`, reads use
//! `DI | READ_ID`, and the final read additionally sets the hardware's
//! (misnamed) `ACK` bit to transmit NACK. No wait in this module is unbounded.
//!
//! TLSR8258 exposes `CMD_BUSY`, `BUS_BUSY`, and `NAK`, but no arbitration-loss
//! status. `BUS_BUSY` is therefore used only for idle/STOP completion; it is
//! not misreported as `ErrorKind::ArbitrationLoss`.

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
#[cfg(target_arch = "tc32")]
use embedded_hal::i2c::{ErrorType, I2c, Operation, SevenBitAddress};

use crate::gpio::{Pin, Port};

const I2C_MAX_HZ: u32 = 400_000;

const REG_I2C_SPEED: u32 = crate::mmio::REG_BASE;
const REG_I2C_ID: u32 = crate::mmio::REG_BASE + 0x01;
const REG_I2C_STATUS: u32 = crate::mmio::REG_BASE + 0x02;
const REG_I2C_MODE: u32 = crate::mmio::REG_BASE + 0x03;
// The vendor 8258 object uses 0x06 for both transmitted and received bytes.
const REG_I2C_DATA: u32 = crate::mmio::REG_BASE + 0x06;
const REG_I2C_CTRL: u32 = crate::mmio::REG_BASE + 0x07;
const REG_SPI_SP: u32 = crate::mmio::REG_BASE + 0x0A;
const REG_PIN_I2C_SPI_EN: u32 = crate::mmio::REG_BASE + 0x5B7;

const STATUS_COMMAND_BUSY: u8 = 1 << 0;
const STATUS_BUS_BUSY: u8 = 1 << 1;
const STATUS_NACK: u8 = 1 << 2;
const MODE_MASTER_ENABLE: u8 = 1 << 1;
const CMD_ID: u8 = 1 << 0;
const CMD_DATA: u8 = 1 << 3;
const CMD_START: u8 = 1 << 4;
const CMD_STOP: u8 = 1 << 5;
const CMD_READ_ID: u8 = 1 << 6;
const CMD_NACK: u8 = 1 << 7;
const SPI_ENABLE: u8 = 1 << 7;
const RESET_I2C: u8 = 1 << 1;
const CLOCK_I2C: u8 = 1 << 1;

/// The TLSR8258 status register has no documented arbitration-loss bit.
pub const ARBITRATION_LOSS_DETECTION_SUPPORTED: bool = false;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullUp {
    External,
    Internal10K,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinGroup {
    A3A4,
    B6D7,
    C0C1,
    C2C3,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Config {
    pub reference_hz: u32,
    pub bus_hz: u32,
    pub sda: Pin,
    pub scl: Pin,
    pub pull_up: PullUp,
    pub timeout_iterations: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    InvalidClock,
    InvalidPins,
    InvalidTimeout,
    InvalidAddress,
    TransferTooLong,
    CommandTimeout,
    BusTimeout,
    NoAcknowledgeAddress,
    NoAcknowledgeData,
    BusStuck,
    PinConfiguration,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::BusTimeout | Self::BusStuck => ErrorKind::Bus,
            Self::NoAcknowledgeAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            _ => ErrorKind::Other,
        }
    }
}

pub struct I2cMaster {
    config: Config,
    group: PinGroup,
    divider: u8,
    actual_bus_hz: u32,
}

impl I2cMaster {
    #[cfg(target_arch = "tc32")]
    pub fn new(
        _peripheral: crate::peripherals::SerialController,
        config: Config,
    ) -> Result<Self, I2cError> {
        let (divider, actual_bus_hz) = clock_divider(config.reference_hz, config.bus_hz)?;
        let group = pin_group(&config.sda, &config.scl).ok_or(I2cError::InvalidPins)?;
        if config.timeout_iterations == 0 {
            return Err(I2cError::InvalidTimeout);
        }

        let mut controller = Self {
            config,
            group,
            divider,
            actual_bus_hz,
        };
        controller.configure_pins()?;
        controller.configure_peripheral();
        if !crate::gpio::read(&controller.config.sda) || !crate::gpio::read(&controller.config.scl)
        {
            controller.recover_bus()?;
        }
        controller.wait_bus_idle()?;
        Ok(controller)
    }

    pub const fn actual_bus_hz(&self) -> u32 {
        self.actual_bus_hz
    }

    pub const fn pin_group(&self) -> PinGroup {
        self.group
    }

    #[cfg(target_arch = "tc32")]
    pub fn recover_bus(&mut self) -> Result<(), I2cError> {
        self.reset_peripheral();
        self.configure_recovery_pin(&self.config.sda)?;
        self.configure_recovery_pin(&self.config.scl)?;
        recovery_delay();

        for _ in 0..9 {
            if crate::gpio::read(&self.config.sda) {
                break;
            }
            self.drive_low(&self.config.scl);
            recovery_delay();
            self.release(&self.config.scl);
            recovery_delay();
        }

        // GPIO STOP: SDA low while SCL is released, then release SDA.
        self.drive_low(&self.config.sda);
        recovery_delay();
        self.release(&self.config.scl);
        recovery_delay();
        self.release(&self.config.sda);
        recovery_delay();

        if !crate::gpio::read(&self.config.sda) || !crate::gpio::read(&self.config.scl) {
            return Err(I2cError::BusStuck);
        }
        self.configure_pins()?;
        self.configure_peripheral();
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    fn configure_recovery_pin(&self, pin: &Pin) -> Result<(), I2cError> {
        crate::gpio::set_function_gpio(pin);
        crate::gpio::write(pin, false);
        crate::gpio::set_output_enable(pin, false);
        crate::gpio::set_input_enable(pin, true).map_err(|_| I2cError::PinConfiguration)?;
        let pull = match self.config.pull_up {
            PullUp::External => crate::gpio::Pull::Float,
            PullUp::Internal10K => crate::gpio::Pull::PullUp10K,
        };
        crate::gpio::set_pull(pin, pull).map_err(|_| I2cError::PinConfiguration)
    }

    #[cfg(target_arch = "tc32")]
    fn drive_low(&self, pin: &Pin) {
        crate::gpio::write(pin, false);
        crate::gpio::set_output_enable(pin, true);
    }

    #[cfg(target_arch = "tc32")]
    fn release(&self, pin: &Pin) {
        crate::gpio::set_output_enable(pin, false);
    }

    #[cfg(target_arch = "tc32")]
    fn configure_pins(&self) -> Result<(), I2cError> {
        let pull = match self.config.pull_up {
            PullUp::External => crate::gpio::Pull::Float,
            PullUp::Internal10K => crate::gpio::Pull::PullUp10K,
        };
        for pin in [&self.config.sda, &self.config.scl] {
            crate::gpio::set_output_enable(pin, false);
            crate::gpio::set_input_enable(pin, true).map_err(|_| I2cError::PinConfiguration)?;
            crate::gpio::set_pull(pin, pull).map_err(|_| I2cError::PinConfiguration)?;
            crate::gpio::set_function(pin, crate::gpio::PinFunction::I2c)
                .map_err(|_| I2cError::InvalidPins)?;
        }

        // PA/PB inputs share 0x5b7 with SPI. These masks are the exact
        // i2c_gpio_set() read-modify-writes; PC groups need no extra selector.
        unsafe {
            let current = crate::mmio::r8(REG_PIN_I2C_SPI_EN);
            let selected = match self.group {
                PinGroup::A3A4 => (current | 0x30) & !0x03,
                PinGroup::B6D7 => (current | 0xC0) & !0x0C,
                PinGroup::C0C1 | PinGroup::C2C3 => current,
            };
            crate::mmio::w8(REG_PIN_I2C_SPI_EN, selected);
        }
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    fn configure_peripheral(&self) {
        unsafe {
            let clocks = crate::mmio::r8(crate::mmio::REG_CLK_EN0);
            crate::mmio::w8(crate::mmio::REG_CLK_EN0, clocks | CLOCK_I2C);
            self.reset_peripheral();
            crate::mmio::w8(REG_I2C_SPEED, self.divider);
            crate::mmio::w8(REG_I2C_MODE, MODE_MASTER_ENABLE);
            let spi = crate::mmio::r8(REG_SPI_SP);
            crate::mmio::w8(REG_SPI_SP, spi & !SPI_ENABLE);
        }
    }

    #[cfg(target_arch = "tc32")]
    fn reset_peripheral(&self) {
        unsafe {
            let reset = crate::mmio::r8(crate::mmio::REG_RST0);
            crate::mmio::w8(crate::mmio::REG_RST0, reset | RESET_I2C);
            let reset = crate::mmio::r8(crate::mmio::REG_RST0);
            crate::mmio::w8(crate::mmio::REG_RST0, reset & !RESET_I2C);
        }
    }

    #[cfg(target_arch = "tc32")]
    fn wait_bus_idle(&self) -> Result<(), I2cError> {
        let mut command_busy = false;
        for _ in 0..self.config.timeout_iterations {
            let status = unsafe { crate::mmio::r8(REG_I2C_STATUS) };
            command_busy = status & STATUS_COMMAND_BUSY != 0;
            if !command_busy && status & STATUS_BUS_BUSY == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        if command_busy {
            Err(I2cError::CommandTimeout)
        } else {
            Err(I2cError::BusTimeout)
        }
    }

    #[cfg(target_arch = "tc32")]
    fn wait_active(&self, address_phase: bool, check_nack: bool) -> Result<(), I2cError> {
        for _ in 0..self.config.timeout_iterations {
            let status = unsafe { crate::mmio::r8(REG_I2C_STATUS) };
            if status & STATUS_COMMAND_BUSY != 0 {
                core::hint::spin_loop();
                continue;
            }
            if let Some(error) = command_error(status, address_phase, check_nack) {
                return Err(error);
            }
            return Ok(());
        }
        Err(I2cError::CommandTimeout)
    }

    #[cfg(target_arch = "tc32")]
    fn start(&self, address: SevenBitAddress, read: bool) -> Result<(), I2cError> {
        let address_byte = (address << 1) | u8::from(read);
        unsafe {
            crate::mmio::w8(REG_I2C_ID, address_byte);
            crate::mmio::w8(REG_I2C_CTRL, CMD_START | CMD_ID);
        }
        self.wait_active(true, true)
    }

    #[cfg(target_arch = "tc32")]
    fn write_byte(&self, byte: u8) -> Result<(), I2cError> {
        unsafe {
            crate::mmio::w8(REG_I2C_DATA, byte);
            crate::mmio::w8(REG_I2C_CTRL, CMD_DATA);
        }
        self.wait_active(false, true)
    }

    #[cfg(target_arch = "tc32")]
    fn read_byte(&self, last: bool) -> Result<u8, I2cError> {
        unsafe {
            crate::mmio::w8(
                REG_I2C_CTRL,
                CMD_DATA | CMD_READ_ID | if last { CMD_NACK } else { 0 },
            );
        }
        self.wait_active(false, false)?;
        Ok(unsafe { crate::mmio::r8(REG_I2C_DATA) })
    }

    #[cfg(target_arch = "tc32")]
    fn stop(&self) -> Result<(), I2cError> {
        unsafe { crate::mmio::w8(REG_I2C_CTRL, CMD_STOP) };
        let mut command_busy = false;
        for _ in 0..self.config.timeout_iterations {
            let status = unsafe { crate::mmio::r8(REG_I2C_STATUS) };
            command_busy = status & STATUS_COMMAND_BUSY != 0;
            if status & (STATUS_COMMAND_BUSY | STATUS_BUS_BUSY) == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        if command_busy {
            Err(I2cError::CommandTimeout)
        } else {
            Err(I2cError::BusTimeout)
        }
    }

    #[cfg(target_arch = "tc32")]
    fn cleanup_error(&mut self, original: I2cError) -> I2cError {
        let recovery_reason = if matches!(
            original,
            I2cError::NoAcknowledgeAddress | I2cError::NoAcknowledgeData
        ) {
            match self.stop() {
                Ok(()) => return original,
                Err(stop_error) => stop_error,
            }
        } else {
            original
        };
        match self.recover_bus() {
            Ok(()) => recovery_reason,
            Err(error) => error,
        }
    }
}

#[cfg(target_arch = "tc32")]
impl ErrorType for I2cMaster {
    type Error = I2cError;
}

#[cfg(target_arch = "tc32")]
impl I2c<SevenBitAddress> for I2cMaster {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if address > 0x7F {
            return Err(I2cError::InvalidAddress);
        }
        if operations.is_empty() {
            return Ok(());
        }
        if let Err(error) = self.wait_bus_idle() {
            // No command was launched, so another master's ownership must
            // not trigger GPIO recovery pulses on a multi-master bus.
            return Err(error);
        }

        let result = (|| {
            let mut index = 0;
            while index < operations.len() {
                let reading = matches!(operations[index], Operation::Read(_));
                let mut run_end = index + 1;
                while run_end < operations.len()
                    && matches!(operations[run_end], Operation::Read(_)) == reading
                {
                    run_end += 1;
                }

                self.start(address, reading)?;
                if reading {
                    let read_count = operations[index..run_end]
                        .iter()
                        .try_fold(0usize, |count, operation| {
                            let len = match operation {
                                Operation::Read(bytes) => bytes.len(),
                                Operation::Write(_) => 0,
                            };
                            count.checked_add(len)
                        })
                        .ok_or(I2cError::TransferTooLong)?;
                    let mut remaining = read_count;
                    for operation in &mut operations[index..run_end] {
                        if let Operation::Read(bytes) = operation {
                            for byte in bytes.iter_mut() {
                                remaining -= 1;
                                *byte = self.read_byte(remaining == 0)?;
                            }
                        }
                    }
                } else {
                    for operation in &operations[index..run_end] {
                        if let Operation::Write(bytes) = operation {
                            for &byte in *bytes {
                                self.write_byte(byte)?;
                            }
                        }
                    }
                }
                index = run_end;
            }
            self.stop()
        })();

        result.map_err(|error| self.cleanup_error(error))
    }
}

fn pin_group(sda: &Pin, scl: &Pin) -> Option<PinGroup> {
    match (sda.port_and_bit(), scl.port_and_bit()) {
        ((Port::A, 3), (Port::A, 4)) => Some(PinGroup::A3A4),
        ((Port::B, 6), (Port::D, 7)) => Some(PinGroup::B6D7),
        ((Port::C, 0), (Port::C, 1)) => Some(PinGroup::C0C1),
        ((Port::C, 2), (Port::C, 3)) => Some(PinGroup::C2C3),
        _ => None,
    }
}

const fn command_error(status: u8, address_phase: bool, check_nack: bool) -> Option<I2cError> {
    if check_nack && status & STATUS_NACK != 0 {
        Some(if address_phase {
            I2cError::NoAcknowledgeAddress
        } else {
            I2cError::NoAcknowledgeData
        })
    } else {
        None
    }
}

fn clock_divider(reference_hz: u32, bus_hz: u32) -> Result<(u8, u32), I2cError> {
    if reference_hz == 0 || bus_hz == 0 || bus_hz > I2C_MAX_HZ {
        return Err(I2cError::InvalidClock);
    }
    let denominator = bus_hz.checked_mul(4).ok_or(I2cError::InvalidClock)?;
    let divider = reference_hz
        .checked_add(denominator - 1)
        .ok_or(I2cError::InvalidClock)?
        / denominator;
    if divider == 0 || divider > u8::MAX as u32 {
        return Err(I2cError::InvalidClock);
    }
    let actual = reference_hz / (4 * divider);
    Ok((divider as u8, actual))
}

#[cfg(target_arch = "tc32")]
#[inline(never)]
fn recovery_delay() {
    for _ in 0..256 {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_map_and_command_bits_match_8258_header() {
        assert_eq!(REG_I2C_SPEED, 0x800000);
        assert_eq!(REG_I2C_ID, 0x800001);
        assert_eq!(REG_I2C_STATUS, 0x800002);
        assert_eq!(REG_I2C_MODE, 0x800003);
        assert_eq!(REG_I2C_DATA, 0x800006);
        assert_eq!(REG_I2C_CTRL, 0x800007);
        assert_eq!(CMD_START | CMD_ID, 0x11);
        assert_eq!(CMD_DATA, 0x08);
        assert_eq!(CMD_DATA | CMD_READ_ID, 0x48);
        assert_eq!(CMD_DATA | CMD_READ_ID | CMD_NACK, 0xC8);
        assert_eq!(CMD_STOP, 0x20);
        assert_eq!(STATUS_COMMAND_BUSY | STATUS_BUS_BUSY | STATUS_NACK, 0x07);
    }

    #[test]
    fn divider_matches_tlsr8258_formula_without_overspeed() {
        assert_eq!(clock_divider(24_000_000, 100_000), Ok((60, 100_000)));
        assert_eq!(clock_divider(24_000_000, 400_000), Ok((15, 400_000)));
        assert_eq!(clock_divider(24_000_000, 110_000), Ok((55, 109_090)));
    }

    #[test]
    fn invalid_i2c_clocks_are_rejected() {
        assert_eq!(clock_divider(0, 100_000), Err(I2cError::InvalidClock));
        assert_eq!(clock_divider(24_000_000, 0), Err(I2cError::InvalidClock));
        assert_eq!(
            clock_divider(24_000_000, 400_001),
            Err(I2cError::InvalidClock)
        );
        assert_eq!(
            clock_divider(24_000_000, 1_000),
            Err(I2cError::InvalidClock)
        );
    }

    #[test]
    fn all_documented_pin_groups_validate() {
        assert_eq!(
            pin_group(&Pin::new(Port::A, 3), &Pin::new(Port::A, 4)),
            Some(PinGroup::A3A4)
        );
        assert_eq!(
            pin_group(&Pin::new(Port::B, 6), &Pin::new(Port::D, 7)),
            Some(PinGroup::B6D7)
        );
        assert_eq!(
            pin_group(&Pin::new(Port::C, 0), &Pin::new(Port::C, 1)),
            Some(PinGroup::C0C1)
        );
        assert_eq!(
            pin_group(&Pin::new(Port::C, 2), &Pin::new(Port::C, 3)),
            Some(PinGroup::C2C3)
        );
    }

    #[test]
    fn swapped_or_mixed_i2c_pins_are_rejected() {
        assert_eq!(
            pin_group(&Pin::new(Port::C, 1), &Pin::new(Port::C, 0)),
            None
        );
        assert_eq!(
            pin_group(&Pin::new(Port::A, 3), &Pin::new(Port::C, 1)),
            None
        );
    }

    #[test]
    fn bus_busy_is_not_guessed_to_be_arbitration_loss() {
        assert!(!ARBITRATION_LOSS_DETECTION_SUPPORTED);
        assert_eq!(command_error(0, true, true), None);
        assert_eq!(
            command_error(STATUS_NACK, true, true),
            Some(I2cError::NoAcknowledgeAddress)
        );
        assert_eq!(
            command_error(STATUS_NACK, false, true),
            Some(I2cError::NoAcknowledgeData)
        );
    }
}
