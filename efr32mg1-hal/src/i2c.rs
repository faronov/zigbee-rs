//! Blocking EFR32MG1 I2C0 master with bounded polling and bus recovery.

use embedded_hal::i2c::{
    ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};

use crate::{
    clock,
    gpio::{Mode, Pin},
};

const I2C0_BASE: u32 = 0x4000_C000;
const I2C_CTRL: u32 = I2C0_BASE;
const I2C_CMD: u32 = I2C0_BASE + 0x004;
const I2C_STATE: u32 = I2C0_BASE + 0x008;
const I2C_STATUS: u32 = I2C0_BASE + 0x00C;
const I2C_CLKDIV: u32 = I2C0_BASE + 0x010;
const I2C_RXDATA: u32 = I2C0_BASE + 0x01C;
const I2C_TXDATA: u32 = I2C0_BASE + 0x02C;
const I2C_IF: u32 = I2C0_BASE + 0x034;
const I2C_IFC: u32 = I2C0_BASE + 0x03C;
const I2C_ROUTEPEN: u32 = I2C0_BASE + 0x044;
const I2C_ROUTELOC0: u32 = I2C0_BASE + 0x048;

const CTRL_EN: u32 = 1 << 0;
const CMD_START: u32 = 1 << 0;
const CMD_STOP: u32 = 1 << 1;
const CMD_ACK: u32 = 1 << 2;
const CMD_NACK: u32 = 1 << 3;
const CMD_ABORT: u32 = 1 << 5;
const CMD_CLEARTX: u32 = 1 << 6;
const CMD_CLEARPC: u32 = 1 << 7;

const STATUS_RXDATAV: u32 = 1 << 8;
const STATE_BUSY: u32 = 1 << 0;

const IF_RXDATAV: u32 = 1 << 5;
const IF_ACK: u32 = 1 << 6;
const IF_NACK: u32 = 1 << 7;
const IF_MSTOP: u32 = 1 << 8;
const IF_ARBLOST: u32 = 1 << 9;
const IF_BUSERR: u32 = 1 << 10;
const IF_ALL: u32 = 0x0007_FFFF;

const ROUTEPEN_SDAPEN: u32 = 1 << 0;
const ROUTEPEN_SCLPEN: u32 = 1 << 1;
const ROUTELOC0_SDALOC_MASK: u32 = 0x1F;
const ROUTELOC0_SCLLOC_MASK: u32 = 0x1F << 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullUp {
    External,
    Internal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub reference_hz: u32,
    pub bus_hz: u32,
    pub sda: Pin,
    pub scl: Pin,
    pub location: u8,
    pub pull_up: PullUp,
    pub timeout_iterations: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    InvalidConfig,
    Timeout,
    Bus,
    ArbitrationLoss,
    NoAcknowledgeAddress,
    NoAcknowledgeData,
    BusStuck,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Bus | Self::BusStuck => ErrorKind::Bus,
            Self::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            Self::NoAcknowledgeAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Self::InvalidConfig | Self::Timeout => ErrorKind::Other,
        }
    }
}

pub struct I2c0 {
    config: Config,
    divider: u16,
}

impl I2c0 {
    pub fn new(config: Config) -> Result<Self, I2cError> {
        let divider = divider(config.reference_hz, config.bus_hz)?;
        if config.location > 31 || config.timeout_iterations == 0 {
            return Err(I2cError::InvalidConfig);
        }

        let mut controller = Self { config, divider };
        clock::enable_gpio_clock();
        clock::enable_i2c0_clock();
        controller.configure_pins();
        controller.configure_peripheral();

        if !controller.config.sda.is_high() || !controller.config.scl.is_high() {
            controller.recover_bus()?;
        }
        Ok(controller)
    }

    pub fn recover_bus(&mut self) -> Result<(), I2cError> {
        unsafe {
            write(I2C_CMD, CMD_ABORT | CMD_CLEARTX | CMD_CLEARPC);
            write(I2C_ROUTEPEN, 0);
            write(I2C_CTRL, 0);
        }

        let mode = self.pin_mode();
        self.config.sda.configure(mode, true);
        self.config.scl.configure(mode, true);
        recovery_delay();

        // Release a slave that was interrupted while transmitting a zero bit.
        for _ in 0..9 {
            if self.config.sda.is_high() {
                break;
            }
            self.config.scl.set_low();
            recovery_delay();
            self.config.scl.set_high();
            recovery_delay();
        }

        // Generate a GPIO STOP while both pins remain open drain.
        self.config.sda.set_low();
        recovery_delay();
        self.config.scl.set_high();
        recovery_delay();
        self.config.sda.set_high();
        recovery_delay();

        self.configure_pins();
        self.configure_peripheral();

        if self.config.sda.is_high() && self.config.scl.is_high() {
            Ok(())
        } else {
            Err(I2cError::BusStuck)
        }
    }

    fn configure_pins(&self) {
        let mode = self.pin_mode();
        self.config.sda.configure(mode, true);
        self.config.scl.configure(mode, true);
    }

    fn pin_mode(&self) -> Mode {
        match self.config.pull_up {
            PullUp::External => Mode::WiredAndFilter,
            PullUp::Internal => Mode::WiredAndPullUpFilter,
        }
    }

    fn configure_peripheral(&self) {
        unsafe {
            write(I2C_CMD, CMD_ABORT | CMD_CLEARTX | CMD_CLEARPC);
            write(I2C_CTRL, 0);
            flush_receive_buffer();
            write(I2C_IFC, IF_ALL);
            write(I2C_CLKDIV, self.divider as u32);
            modify(
                I2C_ROUTELOC0,
                ROUTELOC0_SDALOC_MASK | ROUTELOC0_SCLLOC_MASK,
                (self.config.location as u32) | ((self.config.location as u32) << 8),
            );
            write(I2C_ROUTEPEN, ROUTEPEN_SDAPEN | ROUTEPEN_SCLPEN);
            write(I2C_CTRL, CTRL_EN);
        }
    }

    fn start(
        &mut self,
        address: SevenBitAddress,
        read_direction: bool,
        repeated: bool,
    ) -> Result<(), I2cError> {
        let address_byte = (address << 1) | u8::from(read_direction);
        unsafe {
            write(I2C_IFC, IF_ACK | IF_NACK);
            if repeated {
                // Series 1 requires START before TXDATA for a repeated START;
                // the opposite order transmits the address as write data.
                write(I2C_CMD, CMD_START);
                write(I2C_TXDATA, address_byte as u32);
            } else {
                write(I2C_TXDATA, address_byte as u32);
                write(I2C_CMD, CMD_START);
            }
        }
        self.wait_ack(true)
    }

    fn write_byte(&mut self, byte: u8) -> Result<(), I2cError> {
        unsafe {
            write(I2C_IFC, IF_ACK | IF_NACK);
            write(I2C_TXDATA, byte as u32);
        }
        self.wait_ack(false)
    }

    fn wait_ack(&mut self, address_phase: bool) -> Result<(), I2cError> {
        for _ in 0..self.config.timeout_iterations {
            let flags = unsafe { read(I2C_IF) };
            if flags & IF_BUSERR != 0 {
                return self.fail(I2cError::Bus);
            }
            if flags & IF_ARBLOST != 0 {
                return self.fail(I2cError::ArbitrationLoss);
            }
            if flags & IF_NACK != 0 {
                let nack = if address_phase {
                    I2cError::NoAcknowledgeAddress
                } else {
                    I2cError::NoAcknowledgeData
                };
                return match self.stop() {
                    Ok(()) => Err(nack),
                    Err(error) => Err(error),
                };
            }
            if flags & IF_ACK != 0 {
                unsafe { write(I2C_IFC, IF_ACK) };
                return Ok(());
            }
            core::hint::spin_loop();
        }
        self.fail(I2cError::Timeout)
    }

    fn wait_receive(&mut self) -> Result<u8, I2cError> {
        for _ in 0..self.config.timeout_iterations {
            let flags = unsafe { read(I2C_IF) };
            if flags & IF_BUSERR != 0 {
                return self.fail(I2cError::Bus);
            }
            if flags & IF_ARBLOST != 0 {
                return self.fail(I2cError::ArbitrationLoss);
            }
            if flags & IF_RXDATAV != 0 {
                return Ok(unsafe { read(I2C_RXDATA) as u8 });
            }
            core::hint::spin_loop();
        }
        self.fail(I2cError::Timeout)
    }

    fn stop(&mut self) -> Result<(), I2cError> {
        unsafe {
            write(I2C_IFC, IF_MSTOP);
            write(I2C_CMD, CMD_STOP);
        }
        for _ in 0..self.config.timeout_iterations {
            let flags = unsafe { read(I2C_IF) };
            if flags & IF_BUSERR != 0 {
                return self.fail(I2cError::Bus);
            }
            if flags & IF_ARBLOST != 0 {
                return self.fail(I2cError::ArbitrationLoss);
            }
            if flags & IF_MSTOP != 0 {
                unsafe { write(I2C_IFC, IF_MSTOP) };
                return Ok(());
            }
            core::hint::spin_loop();
        }
        self.fail(I2cError::Timeout)
    }

    fn fail<T>(&mut self, error: I2cError) -> Result<T, I2cError> {
        let recovery = self.recover_bus();
        if let Err(recovery_error) = recovery {
            Err(recovery_error)
        } else {
            Err(error)
        }
    }

    fn prepare_transaction(&mut self) {
        unsafe {
            if read(I2C_STATE) & STATE_BUSY != 0 {
                write(I2C_CMD, CMD_ABORT);
            }
            // Match GSDK I2C_TransferInit: discard pending commands and stale
            // receive bytes, then clear every sticky interrupt flag.
            write(I2C_CMD, CMD_CLEARTX | CMD_CLEARPC);
            flush_receive_buffer();
            write(I2C_IFC, IF_ALL);
        }
    }
}

impl ErrorType for I2c0 {
    type Error = I2cError;
}

impl I2c<SevenBitAddress> for I2c0 {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if address > 0x7F {
            return Err(I2cError::InvalidConfig);
        }
        self.prepare_transaction();

        let mut index = 0;
        let mut transferred = false;
        while index < operations.len() {
            while index < operations.len() && operation_len(&operations[index]) == 0 {
                index += 1;
            }
            if index == operations.len() {
                break;
            }

            let reading = matches!(operations[index], Operation::Read(_));
            let run_end = next_direction_change(operations, index, reading);
            let read_count = if reading {
                operations[index..run_end].iter().map(operation_len).sum()
            } else {
                0
            };

            if read_count == 1 {
                // Series 1 preloads NACK before START for a one-byte read.
                unsafe { write(I2C_CMD, CMD_NACK) };
            }
            self.start(address, reading, transferred)?;
            transferred = true;

            if reading {
                let mut remaining = read_count;
                for operation in &mut operations[index..run_end] {
                    if let Operation::Read(buffer) = operation {
                        for byte in buffer.iter_mut() {
                            *byte = self.wait_receive()?;
                            remaining -= 1;
                            if remaining != 0 {
                                unsafe { write(I2C_CMD, CMD_ACK) };
                                if remaining == 1 {
                                    unsafe { write(I2C_CMD, CMD_NACK) };
                                }
                            }
                        }
                    }
                }
            } else {
                for operation in &operations[index..run_end] {
                    if let Operation::Write(buffer) = operation {
                        for &byte in *buffer {
                            self.write_byte(byte)?;
                        }
                    }
                }
            }
            index = run_end;
        }

        if transferred { self.stop() } else { Ok(()) }
    }
}

fn next_direction_change(operations: &[Operation<'_>], start: usize, reading: bool) -> usize {
    let mut index = start;
    while index < operations.len() {
        if operation_len(&operations[index]) != 0
            && matches!(operations[index], Operation::Read(_)) != reading
        {
            break;
        }
        index += 1;
    }
    index
}

fn operation_len(operation: &Operation<'_>) -> usize {
    match operation {
        Operation::Read(buffer) => buffer.len(),
        Operation::Write(buffer) => buffer.len(),
    }
}

fn divider(reference_hz: u32, bus_hz: u32) -> Result<u16, I2cError> {
    if reference_hz == 0 || bus_hz == 0 || bus_hz > 100_000 {
        return Err(I2cError::InvalidConfig);
    }
    let denominator = bus_hz.checked_mul(8).ok_or(I2cError::InvalidConfig)?;
    let quotient = reference_hz
        .checked_add(denominator - 1)
        .ok_or(I2cError::InvalidConfig)?
        / denominator;
    let divider = quotient
        .checked_sub(2)
        .filter(|value| *value <= 0x1FF)
        .ok_or(I2cError::InvalidConfig)?;
    Ok(divider as u16)
}

#[inline(never)]
fn recovery_delay() {
    for _ in 0..256 {
        core::hint::spin_loop();
    }
}

#[inline]
unsafe fn read(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline]
unsafe fn modify(address: u32, mask: u32, value: u32) {
    let current = unsafe { read(address) };
    unsafe { write(address, (current & !mask) | (value & mask)) };
}

unsafe fn flush_receive_buffer() {
    for _ in 0..4 {
        if unsafe { read(I2C_STATUS) } & STATUS_RXDATAV == 0 {
            break;
        }
        let _ = unsafe { read(I2C_RXDATA) };
    }
}

#[cfg(test)]
mod tests {
    use super::divider;

    #[test]
    fn gsdk_divider_for_38m4_100k_is_46() {
        assert_eq!(divider(38_400_000, 100_000), Ok(46));
    }
}
