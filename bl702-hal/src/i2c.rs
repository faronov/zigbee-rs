//! Blocking BL702 I2C0 controller.
//!
//! The hardware describes one packet at a time. Single reads/writes and
//! one-to-four-byte register-address write/read operations use that fast path.
//! Other valid `embedded-hal` transaction shapes use a bounded GPIO
//! open-drain fallback so adjacent operations retain the required repeated
//! START semantics instead of being split by unintended STOP conditions.

use core::hint::spin_loop;

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};

use crate::clock::{Clocks, Peripheral, configure_i2c_clock, enable_and_reset};
use crate::gpio::{Alternate, Disabled, Drive, FUNCTION_I2C, Pin, Pull};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::I2c0;

const I2C_BASE: u32 = 0x4000_a300;
const CONFIG: u32 = I2C_BASE;
const INT_STS: u32 = I2C_BASE + 0x004;
const SUB_ADDR: u32 = I2C_BASE + 0x008;
const BUS_BUSY: u32 = I2C_BASE + 0x00c;
const PRD_START: u32 = I2C_BASE + 0x010;
const PRD_STOP: u32 = I2C_BASE + 0x014;
const PRD_DATA: u32 = I2C_BASE + 0x018;
const FIFO_CONFIG_0: u32 = I2C_BASE + 0x080;
const FIFO_CONFIG_1: u32 = I2C_BASE + 0x084;
const FIFO_WDATA: u32 = I2C_BASE + 0x088;
const FIFO_RDATA: u32 = I2C_BASE + 0x08c;

const ENABLE: u32 = 1 << 0;
const DIRECTION_READ: u32 = 1 << 1;
const SCL_SYNC: u32 = 1 << 3;
const SUB_ADDRESS_ENABLE: u32 = 1 << 4;
const SUB_ADDRESS_COUNT_MASK: u32 = 0x3 << 5;
const ADDRESS_MASK: u32 = 0x7f << 8;
const LENGTH_MASK: u32 = 0xff << 16;

const END: u32 = 1 << 0;
const NACK: u32 = 1 << 3;
const ARBITRATION_LOST: u32 = 1 << 4;
const FIFO_ERROR: u32 = 1 << 5;
const CLEAR_STATUS: u32 = (1 << 16) | (1 << 19) | (1 << 20);
const FIFO_CLEAR: u32 = (1 << 2) | (1 << 3);
const TIMEOUT_ITERATIONS: u32 = 1_000_000;

/// I2C clock divider and phase lengths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    pub divider: u8,
    pub phase: u8,
    pub actual_hz: u32,
}

impl ClockConfig {
    /// Select a clock no faster than `requested_hz`.
    pub const fn calculate(source_hz: u32, requested_hz: u32) -> Option<Self> {
        if source_hz == 0 || requested_hz == 0 {
            return None;
        }
        let mut divider = 1u32;
        while divider <= 256 {
            let denominator = match requested_hz.checked_mul(4) {
                Some(value) => match value.checked_mul(divider) {
                    Some(value) => value,
                    None => return None,
                },
                None => return None,
            };
            let cycles = source_hz.div_ceil(denominator);
            if cycles >= 1 && cycles <= 256 {
                let actual_hz = source_hz / (divider * 4 * cycles);
                return Some(Self {
                    divider: (divider - 1) as u8,
                    phase: (cycles - 1) as u8,
                    actual_hz,
                });
            }
            divider += 1;
        }
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    InvalidAddress,
    InvalidLength,
    NoAcknowledge,
    ArbitrationLost,
    Fifo,
    Bus,
    Timeout,
    Pin(crate::gpio::ConfigError),
    Clock,
    Route,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::NoAcknowledge => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::Bus => ErrorKind::Bus,
            _ => ErrorKind::Other,
        }
    }
}

pub struct I2c0Bus<const SCL: u8, const SDA: u8> {
    _token: I2c0,
    _scl: Pin<SCL, Alternate>,
    _sda: Pin<SDA, Alternate>,
    bitbang_half_period_us: u32,
}

impl<const SCL: u8, const SDA: u8> I2c0Bus<SCL, SDA> {
    pub fn new(
        token: I2c0,
        scl: Pin<SCL, Disabled>,
        sda: Pin<SDA, Disabled>,
        clocks: Clocks,
        bus_hz: u32,
    ) -> Result<Self, I2cError> {
        if !valid_route(SCL, SDA) {
            return Err(I2cError::Route);
        }
        let clock = ClockConfig::calculate(clocks.bclk_hz(), bus_hz).ok_or(I2cError::Clock)?;
        let scl = scl
            .into_alternate(FUNCTION_I2C, Pull::Up, Drive::Milliamp9_6)
            .map_err(I2cError::Pin)?;
        let sda = sda
            .into_alternate(FUNCTION_I2C, Pull::Up, Drive::Milliamp9_6)
            .map_err(I2cError::Pin)?;

        enable_and_reset(Peripheral::I2c);
        configure_i2c_clock(clock.divider);
        let phases = u32::from(clock.phase) * 0x0101_0101;
        write32(PRD_START, phases);
        write32(PRD_STOP, phases);
        write32(PRD_DATA, phases);
        rmw(CONFIG, SCL_SYNC, SCL_SYNC);

        Ok(Self {
            _token: token,
            _scl: scl,
            _sda: sda,
            bitbang_half_period_us: 500_000u32.div_ceil(clock.actual_hz).max(1),
        })
    }

    fn transfer_packet(
        &mut self,
        address: u8,
        subaddress: &[u8],
        data: PacketData<'_>,
    ) -> Result<(), I2cError> {
        if address > 0x7f {
            return Err(I2cError::InvalidAddress);
        }
        let length = data.len();
        if length == 0 || length > 256 || subaddress.len() > 4 {
            return Err(I2cError::InvalidLength);
        }

        self.disable_and_clear();
        let mut config = (u32::from(address) << 8) | ((length as u32 - 1) << 16);
        if data.is_read() {
            config |= DIRECTION_READ;
        }
        if !subaddress.is_empty() {
            config |= SUB_ADDRESS_ENABLE | ((subaddress.len() as u32 - 1) << 5);
            write32(SUB_ADDR, pack_subaddress(subaddress));
        }
        rmw(
            CONFIG,
            DIRECTION_READ
                | SUB_ADDRESS_ENABLE
                | SUB_ADDRESS_COUNT_MASK
                | ADDRESS_MASK
                | LENGTH_MASK,
            config,
        );
        rmw(CONFIG, ENABLE, ENABLE);

        let result = match data {
            PacketData::Write(bytes) => self.write_fifo(bytes),
            PacketData::Read(bytes) => self.read_fifo(bytes),
        }
        .and_then(|_| self.wait_complete());
        self.disable_and_clear();
        result
    }

    fn write_fifo(&mut self, bytes: &[u8]) -> Result<(), I2cError> {
        for chunk in bytes.chunks(4) {
            self.wait_tx_space()?;
            let mut word = [0u8; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            write32(FIFO_WDATA, u32::from_le_bytes(word));
        }
        Ok(())
    }

    fn read_fifo(&mut self, bytes: &mut [u8]) -> Result<(), I2cError> {
        for chunk in bytes.chunks_mut(4) {
            self.wait_rx_data()?;
            let word = read32(FIFO_RDATA).to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        Ok(())
    }

    fn wait_tx_space(&self) -> Result<(), I2cError> {
        self.wait_for(|| read32(FIFO_CONFIG_1) & 0x3 != 0)
    }

    fn wait_rx_data(&self) -> Result<(), I2cError> {
        self.wait_for(|| (read32(FIFO_CONFIG_1) >> 8) & 0x3 != 0)
    }

    fn wait_complete(&self) -> Result<(), I2cError> {
        self.wait_for(|| read32(BUS_BUSY) & 1 == 0 && read32(INT_STS) & END != 0)
    }

    fn wait_for(&self, predicate: impl Fn() -> bool) -> Result<(), I2cError> {
        for _ in 0..TIMEOUT_ITERATIONS {
            self.check_status()?;
            if predicate() {
                return Ok(());
            }
            spin_loop();
        }
        Err(I2cError::Timeout)
    }

    fn check_status(&self) -> Result<(), I2cError> {
        let status = read32(INT_STS);
        if status & NACK != 0 {
            Err(I2cError::NoAcknowledge)
        } else if status & ARBITRATION_LOST != 0 {
            Err(I2cError::ArbitrationLost)
        } else if status & FIFO_ERROR != 0 {
            Err(I2cError::Fifo)
        } else {
            Ok(())
        }
    }

    fn disable_and_clear(&self) {
        rmw(CONFIG, ENABLE, 0);
        rmw(FIFO_CONFIG_0, FIFO_CLEAR, FIFO_CLEAR);
        rmw(INT_STS, CLEAR_STATUS, CLEAR_STATUS);
    }

    fn bitbang_transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), I2cError> {
        if address > 0x7f {
            return Err(I2cError::InvalidAddress);
        }
        if operations.is_empty() {
            return Ok(());
        }

        self.disable_and_clear();
        crate::gpio::configure_open_drain::<SCL>().map_err(I2cError::Pin)?;
        crate::gpio::configure_open_drain::<SDA>().map_err(I2cError::Pin)?;

        let result = execute_transaction(self, address, operations);
        match result {
            Err(I2cError::ArbitrationLost) => {
                crate::gpio::release::<SCL>();
                crate::gpio::release::<SDA>();
            }
            Err(_) => {
                let _ = self.bitbang_stop();
            }
            Ok(()) => {}
        }
        let restore_scl = crate::gpio::configure_i2c::<SCL>().map_err(I2cError::Pin);
        let restore_sda = crate::gpio::configure_i2c::<SDA>().map_err(I2cError::Pin);
        restore_scl?;
        restore_sda?;
        result
    }

    fn bitbang_start(&self) -> Result<(), I2cError> {
        crate::gpio::release::<SDA>();
        crate::gpio::release::<SCL>();
        self.wait_scl_high()?;
        self.bitbang_delay();
        crate::gpio::drive_low::<SDA>();
        self.bitbang_delay();
        crate::gpio::drive_low::<SCL>();
        Ok(())
    }

    fn bitbang_repeated_start(&self) -> Result<(), I2cError> {
        crate::gpio::release::<SDA>();
        self.bitbang_delay();
        crate::gpio::release::<SCL>();
        self.wait_scl_high()?;
        self.bitbang_delay();
        crate::gpio::drive_low::<SDA>();
        self.bitbang_delay();
        crate::gpio::drive_low::<SCL>();
        Ok(())
    }

    fn bitbang_stop(&self) -> Result<(), I2cError> {
        crate::gpio::drive_low::<SDA>();
        self.bitbang_delay();
        crate::gpio::release::<SCL>();
        self.wait_scl_high()?;
        self.bitbang_delay();
        crate::gpio::release::<SDA>();
        self.bitbang_delay();
        Ok(())
    }

    fn bitbang_write_byte(&self, byte: u8) -> Result<(), I2cError> {
        for bit in (0..8).rev() {
            let high = byte & (1 << bit) != 0;
            if high {
                crate::gpio::release::<SDA>();
            } else {
                crate::gpio::drive_low::<SDA>();
            }
            self.bitbang_delay();
            crate::gpio::release::<SCL>();
            self.wait_scl_high()?;
            if high && !crate::gpio::read_level::<SDA>() {
                crate::gpio::drive_low::<SCL>();
                return Err(I2cError::ArbitrationLost);
            }
            self.bitbang_delay();
            crate::gpio::drive_low::<SCL>();
        }

        crate::gpio::release::<SDA>();
        self.bitbang_delay();
        crate::gpio::release::<SCL>();
        self.wait_scl_high()?;
        let acknowledged = !crate::gpio::read_level::<SDA>();
        self.bitbang_delay();
        crate::gpio::drive_low::<SCL>();
        if acknowledged {
            Ok(())
        } else {
            Err(I2cError::NoAcknowledge)
        }
    }

    fn bitbang_read_byte(&self, acknowledge: bool) -> Result<u8, I2cError> {
        crate::gpio::release::<SDA>();
        let mut byte = 0u8;
        for _ in 0..8 {
            self.bitbang_delay();
            crate::gpio::release::<SCL>();
            self.wait_scl_high()?;
            byte = (byte << 1) | u8::from(crate::gpio::read_level::<SDA>());
            self.bitbang_delay();
            crate::gpio::drive_low::<SCL>();
        }

        if acknowledge {
            crate::gpio::drive_low::<SDA>();
        } else {
            crate::gpio::release::<SDA>();
        }
        self.bitbang_delay();
        crate::gpio::release::<SCL>();
        self.wait_scl_high()?;
        self.bitbang_delay();
        crate::gpio::drive_low::<SCL>();
        crate::gpio::release::<SDA>();
        Ok(byte)
    }

    fn wait_scl_high(&self) -> Result<(), I2cError> {
        for _ in 0..TIMEOUT_ITERATIONS {
            if crate::gpio::read_level::<SCL>() {
                return Ok(());
            }
            spin_loop();
        }
        Err(I2cError::Timeout)
    }

    fn bitbang_delay(&self) {
        crate::timer::delay_us(self.bitbang_half_period_us);
    }
}

trait TransactionIo {
    type Error;

    fn start(&mut self, repeated: bool) -> Result<(), Self::Error>;
    fn write_byte(&mut self, byte: u8) -> Result<(), Self::Error>;
    fn read_byte(&mut self, acknowledge: bool) -> Result<u8, Self::Error>;
    fn stop(&mut self) -> Result<(), Self::Error>;
}

impl<const SCL: u8, const SDA: u8> TransactionIo for I2c0Bus<SCL, SDA> {
    type Error = I2cError;

    fn start(&mut self, repeated: bool) -> Result<(), Self::Error> {
        if repeated {
            self.bitbang_repeated_start()
        } else {
            self.bitbang_start()
        }
    }

    fn write_byte(&mut self, byte: u8) -> Result<(), Self::Error> {
        self.bitbang_write_byte(byte)
    }

    fn read_byte(&mut self, acknowledge: bool) -> Result<u8, Self::Error> {
        self.bitbang_read_byte(acknowledge)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.bitbang_stop()
    }
}

fn execute_transaction<T: TransactionIo>(
    io: &mut T,
    address: u8,
    operations: &mut [Operation<'_>],
) -> Result<(), T::Error> {
    let mut index = 0;
    let mut first = true;
    while index < operations.len() {
        let reading = matches!(operations[index], Operation::Read(_));
        let mut run_end = index + 1;
        while run_end < operations.len()
            && matches!(operations[run_end], Operation::Read(_)) == reading
        {
            run_end += 1;
        }

        io.start(!first)?;
        first = false;
        io.write_byte((address << 1) | u8::from(reading))?;

        if reading {
            let total = operations[index..run_end]
                .iter()
                .map(|operation| match operation {
                    Operation::Read(bytes) => bytes.len(),
                    Operation::Write(_) => 0,
                })
                .sum::<usize>();
            let mut read_index = 0usize;
            for operation in &mut operations[index..run_end] {
                let Operation::Read(bytes) = operation else {
                    unreachable!()
                };
                for byte in bytes.iter_mut() {
                    read_index += 1;
                    *byte = io.read_byte(read_index < total)?;
                }
            }
        } else {
            for operation in &operations[index..run_end] {
                let Operation::Write(bytes) = operation else {
                    unreachable!()
                };
                for &byte in *bytes {
                    io.write_byte(byte)?;
                }
            }
        }
        index = run_end;
    }
    io.stop()
}

const fn valid_route(scl: u8, sda: u8) -> bool {
    scl & 1 == 0 && sda & 1 == 1
}

fn pack_subaddress(subaddress: &[u8]) -> u32 {
    let mut bytes = [0u8; 4];
    bytes[..subaddress.len()].copy_from_slice(subaddress);
    u32::from_le_bytes(bytes)
}

enum PacketData<'a> {
    Write(&'a [u8]),
    Read(&'a mut [u8]),
}

impl PacketData<'_> {
    fn len(&self) -> usize {
        match self {
            Self::Write(bytes) => bytes.len(),
            Self::Read(bytes) => bytes.len(),
        }
    }

    fn is_read(&self) -> bool {
        matches!(self, Self::Read(_))
    }
}

impl<const SCL: u8, const SDA: u8> ErrorType for I2c0Bus<SCL, SDA> {
    type Error = I2cError;
}

impl<const SCL: u8, const SDA: u8> I2c for I2c0Bus<SCL, SDA> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        match operations {
            [] => Ok(()),
            [Operation::Write(bytes)] if !bytes.is_empty() && bytes.len() <= 256 => {
                self.transfer_packet(address, &[], PacketData::Write(bytes))
            }
            [Operation::Read(bytes)] if !bytes.is_empty() && bytes.len() <= 256 => {
                self.transfer_packet(address, &[], PacketData::Read(bytes))
            }
            [Operation::Write(subaddress), Operation::Read(bytes)]
                if (1..=4).contains(&subaddress.len())
                    && !bytes.is_empty()
                    && bytes.len() <= 256 =>
            {
                self.transfer_packet(address, subaddress, PacketData::Read(bytes))
            }
            _ => self.bitbang_transaction(address, operations),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Start,
        RepeatedStart,
        Write(u8),
        Read { acknowledge: bool },
        Stop,
    }

    #[derive(Default)]
    struct TraceIo {
        events: Vec<Event>,
        next_read: u8,
    }

    impl TransactionIo for TraceIo {
        type Error = core::convert::Infallible;

        fn start(&mut self, repeated: bool) -> Result<(), Self::Error> {
            self.events.push(if repeated {
                Event::RepeatedStart
            } else {
                Event::Start
            });
            Ok(())
        }

        fn write_byte(&mut self, byte: u8) -> Result<(), Self::Error> {
            self.events.push(Event::Write(byte));
            Ok(())
        }

        fn read_byte(&mut self, acknowledge: bool) -> Result<u8, Self::Error> {
            self.events.push(Event::Read { acknowledge });
            self.next_read = self.next_read.wrapping_add(1);
            Ok(self.next_read)
        }

        fn stop(&mut self) -> Result<(), Self::Error> {
            self.events.push(Event::Stop);
            Ok(())
        }
    }

    #[test]
    fn common_i2c_rates_are_exact_at_32mhz() {
        assert_eq!(
            ClockConfig::calculate(32_000_000, 100_000),
            Some(ClockConfig {
                divider: 0,
                phase: 79,
                actual_hz: 100_000,
            })
        );
        assert_eq!(
            ClockConfig::calculate(32_000_000, 400_000),
            Some(ClockConfig {
                divider: 0,
                phase: 19,
                actual_hz: 400_000,
            })
        );
    }

    #[test]
    fn zero_rate_is_rejected() {
        assert_eq!(ClockConfig::calculate(32_000_000, 0), None);
    }

    #[test]
    fn board_route_and_subaddress_encoding_match_hardware() {
        assert!(valid_route(4, 3));
        assert!(!valid_route(3, 4));
        assert_eq!(pack_subaddress(&[0x12]), 0x0000_0012);
        assert_eq!(pack_subaddress(&[0x12, 0x34, 0x56, 0x78]), 0x7856_3412);
    }

    #[test]
    fn adjacent_writes_share_one_address_phase() {
        let mut operations = [Operation::Write(&[0x10]), Operation::Write(&[0x20, 0x30])];
        let mut io = TraceIo::default();
        execute_transaction(&mut io, 0x44, &mut operations).unwrap();
        assert_eq!(
            io.events,
            vec![
                Event::Start,
                Event::Write(0x88),
                Event::Write(0x10),
                Event::Write(0x20),
                Event::Write(0x30),
                Event::Stop,
            ]
        );
    }

    #[test]
    fn adjacent_reads_ack_all_but_the_final_byte() {
        let mut first = [0u8; 1];
        let mut second = [0u8; 2];
        let mut operations = [Operation::Read(&mut first), Operation::Read(&mut second)];
        let mut io = TraceIo::default();
        execute_transaction(&mut io, 0x44, &mut operations).unwrap();
        assert_eq!(first, [1]);
        assert_eq!(second, [2, 3]);
        assert_eq!(
            io.events,
            vec![
                Event::Start,
                Event::Write(0x89),
                Event::Read { acknowledge: true },
                Event::Read { acknowledge: true },
                Event::Read { acknowledge: false },
                Event::Stop,
            ]
        );
    }

    #[test]
    fn long_write_preamble_uses_a_repeated_start_before_reading() {
        let preamble = [1, 2, 3, 4, 5];
        let mut read = [0u8; 1];
        let mut operations = [Operation::Write(&preamble), Operation::Read(&mut read)];
        let mut io = TraceIo::default();
        execute_transaction(&mut io, 0x2a, &mut operations).unwrap();
        assert_eq!(
            io.events,
            vec![
                Event::Start,
                Event::Write(0x54),
                Event::Write(1),
                Event::Write(2),
                Event::Write(3),
                Event::Write(4),
                Event::Write(5),
                Event::RepeatedStart,
                Event::Write(0x55),
                Event::Read { acknowledge: false },
                Event::Stop,
            ]
        );
    }

    #[test]
    fn alternating_directions_repeat_start_without_stopping() {
        let mut first_read = [0u8; 1];
        let mut second_read = [0u8; 1];
        let mut operations = [
            Operation::Write(&[0x01]),
            Operation::Read(&mut first_read),
            Operation::Write(&[0x02]),
            Operation::Read(&mut second_read),
        ];
        let mut io = TraceIo::default();
        execute_transaction(&mut io, 0x33, &mut operations).unwrap();
        assert_eq!(
            io.events,
            vec![
                Event::Start,
                Event::Write(0x66),
                Event::Write(0x01),
                Event::RepeatedStart,
                Event::Write(0x67),
                Event::Read { acknowledge: false },
                Event::RepeatedStart,
                Event::Write(0x66),
                Event::Write(0x02),
                Event::RepeatedStart,
                Event::Write(0x67),
                Event::Read { acknowledge: false },
                Event::Stop,
            ]
        );
    }
}
