//! Polled I2C0 controller (`embedded-hal` 1.0 [`I2c`](embedded_hal::i2c::I2c)).
//!
//! Register layout is identical on xG21 and xG22 (GSDK `efr32mg2x_i2c.h`);
//! only the GPIO route offset differs. The transfer sequence follows GSDK
//! `I2C_Transfer`: START + address, ACK/NACK per byte, repeated START on a
//! direction change, STOP after the last operation. Every wait is bounded.

use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation, SevenBitAddress};

use crate::chip::{BusClock, base, route};
use crate::delay::Deadline;
use crate::gpio::{OpenDrain, Pin};

const EN: u32 = 0x04;
const CTRL: u32 = 0x08;
const CMD: u32 = 0x0C;
const STATE: u32 = 0x10;
const CLKDIV: u32 = 0x18;
const RXDATA: u32 = 0x24;
const TXDATA: u32 = 0x34;
const IF: u32 = 0x3C;

const CMD_START: u32 = 1 << 0;
const CMD_STOP: u32 = 1 << 1;
const CMD_ACK: u32 = 1 << 2;
const CMD_NACK: u32 = 1 << 3;
const CMD_ABORT: u32 = 1 << 5;
const CMD_CLEARTX: u32 = 1 << 6;
const CMD_CLEARPC: u32 = 1 << 7;
const STATE_BUSY: u32 = 1 << 0;

const IF_RXDATAV: u32 = 1 << 5;
const IF_ACK: u32 = 1 << 6;
const IF_NACK: u32 = 1 << 7;
const IF_MSTOP: u32 = 1 << 8;
const IF_ARBLOST: u32 = 1 << 9;
const IF_BUSERR: u32 = 1 << 10;
const IF_ERRORS: u32 = IF_ARBLOST | IF_BUSERR;

const ROUTEEN_SCLPEN: u32 = 1 << 0;
const ROUTEEN_SDAPEN: u32 = 1 << 1;
const SCLROUTE: u32 = 0x04;
const SDAROUTE: u32 = 0x08;

const CLEAR_ALIAS_OFFSET: u32 = 0x2000;

/// I2C0 runs from LSPCLK = PCLK / 2 on both dies.
pub const LSPCLK_HZ: u32 = 19_200_000;
/// Per-byte timeout (100 kHz byte = 90 us; allow clock stretching).
const BYTE_TIMEOUT_US: u32 = 5_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Nack(NoAcknowledgeSource),
    ArbitrationLost,
    Bus,
    Timeout,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match *self {
            Self::Nack(source) => ErrorKind::NoAcknowledge(source),
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::Bus => ErrorKind::Bus,
            Self::Timeout => ErrorKind::Other,
        }
    }
}

/// Exclusive ownership of I2C0.
pub struct I2c0Token {
    _private: (),
}

impl I2c0Token {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Route I2C0 to `scl`/`sda` and run at `frequency_hz` (standard-mode
    /// 4:4 duty, at most 400 kHz).
    pub fn into_master(self, scl: Pin<OpenDrain>, sda: Pin<OpenDrain>, frequency_hz: u32) -> I2c {
        crate::clock::enable_bus_clock(BusClock::I2C0);
        let routeen = base::GPIO + route::I2C0_ROUTEEN;
        unsafe {
            write(routeen + SCLROUTE, crate::usart::route_value(&scl));
            write(routeen + SDAROUTE, crate::usart::route_value(&sda));
            write(routeen, read(routeen) | ROUTEEN_SCLPEN | ROUTEEN_SDAPEN);
            write(base::I2C0 + EN, 1);
            write(base::I2C0 + CTRL, 0);
            write(base::I2C0 + CLKDIV, clkdiv(LSPCLK_HZ, frequency_hz));
            if read(base::I2C0 + STATE) & STATE_BUSY != 0 {
                write(base::I2C0 + CMD, CMD_ABORT);
            }
        }
        I2c {
            _token: self,
            _scl: scl,
            _sda: sda,
        }
    }
}

/// GSDK `I2C_BusFreqSet` for `i2cClockHLRStandard` (Nlow = Nhigh = 4):
/// `DIV = (fref - 8 * fscl) / (8 * fscl) - 1`, rounded up so the bus is never
/// faster than requested.
pub const fn clkdiv(fref: u32, fscl: u32) -> u32 {
    let n = 8;
    let div = (fref - 8 * fscl).div_ceil(n * fscl);
    if div == 0 { 0 } else { (div - 1) & 0x1FF }
}

/// Polled I2C0 master.
pub struct I2c {
    _token: I2c0Token,
    _scl: Pin<OpenDrain>,
    _sda: Pin<OpenDrain>,
}

impl I2c {
    fn wait_flags(mask: u32) -> Result<u32, Error> {
        let deadline = Deadline::after_us(BYTE_TIMEOUT_US);
        loop {
            let flags = unsafe { read(base::I2C0 + IF) };
            if flags & IF_ERRORS != 0 {
                return Err(if flags & IF_ARBLOST != 0 {
                    Error::ArbitrationLost
                } else {
                    Error::Bus
                });
            }
            if flags & mask != 0 {
                return Ok(flags);
            }
            if deadline.expired() {
                return Err(Error::Timeout);
            }
        }
    }

    fn clear_flags(mask: u32) {
        unsafe { write(base::I2C0 + CLEAR_ALIAS_OFFSET + IF, mask) };
    }

    fn start(address: u8, read_mode: bool) -> Result<(), Error> {
        Self::clear_flags(IF_ACK | IF_NACK);
        unsafe {
            write(base::I2C0 + CMD, CMD_START);
            write(
                base::I2C0 + TXDATA,
                ((address as u32) << 1) | read_mode as u32,
            );
        }
        let flags = Self::wait_flags(IF_ACK | IF_NACK)?;
        Self::clear_flags(IF_ACK | IF_NACK);
        if flags & IF_NACK != 0 {
            return Err(Error::Nack(NoAcknowledgeSource::Address));
        }
        Ok(())
    }

    fn write_bytes(bytes: &[u8]) -> Result<(), Error> {
        for &byte in bytes {
            unsafe { write(base::I2C0 + TXDATA, byte as u32) };
            let flags = Self::wait_flags(IF_ACK | IF_NACK)?;
            Self::clear_flags(IF_ACK | IF_NACK);
            if flags & IF_NACK != 0 {
                return Err(Error::Nack(NoAcknowledgeSource::Data));
            }
        }
        Ok(())
    }

    /// Read `buffer`, ACKing every byte but the last. The last byte is
    /// always NACKed: either STOP or a repeated START follows.
    fn read_bytes(buffer: &mut [u8]) -> Result<(), Error> {
        let len = buffer.len();
        for (index, slot) in buffer.iter_mut().enumerate() {
            Self::wait_flags(IF_RXDATAV)?;
            *slot = unsafe { read(base::I2C0 + RXDATA) } as u8;
            Self::clear_flags(IF_RXDATAV);
            let command = if index + 1 == len { CMD_NACK } else { CMD_ACK };
            unsafe { write(base::I2C0 + CMD, command) };
        }
        Ok(())
    }

    fn stop() -> Result<(), Error> {
        Self::clear_flags(IF_MSTOP);
        unsafe { write(base::I2C0 + CMD, CMD_STOP) };
        Self::wait_flags(IF_MSTOP).map(|_| ())
    }

    fn recover() {
        unsafe {
            write(base::I2C0 + CMD, CMD_ABORT | CMD_CLEARTX | CMD_CLEARPC);
        }
        Self::clear_flags(u32::MAX);
    }
}

impl ErrorType for I2c {
    type Error = Error;
}

impl embedded_hal::i2c::I2c<SevenBitAddress> for I2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        unsafe { write(base::I2C0 + CMD, CMD_CLEARTX | CMD_CLEARPC) };
        Self::clear_flags(u32::MAX);
        let mut previous_read: Option<bool> = None;
        let result = (|| {
            for operation in operations.iter_mut() {
                let is_read = matches!(operation, Operation::Read(_));
                if previous_read != Some(is_read) {
                    Self::start(address, is_read)?;
                    previous_read = Some(is_read);
                }
                match operation {
                    Operation::Write(bytes) => Self::write_bytes(bytes)?,
                    Operation::Read(buffer) => Self::read_bytes(buffer)?,
                }
            }
            Ok(())
        })();
        match result {
            Ok(()) => Self::stop(),
            Err(error) => {
                // Release the bus even after a NACK so the next transfer
                // starts from idle.
                let _ = Self::stop();
                Self::recover();
                Err(error)
            }
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_mode_divider() {
        // 19.2 MHz / (8 * (22 + 1) + 8) = 100 kHz exactly.
        assert_eq!(clkdiv(19_200_000, 100_000), 22);
        // 400 kHz: (19.2e6 - 3.2e6) / 3.2e6 = 5 -> DIV 4 (400 kHz).
        assert_eq!(clkdiv(19_200_000, 400_000), 4);
    }

    #[test]
    #[cfg(feature = "efr32mg22")]
    fn xg22_route_offset() {
        assert_eq!(base::GPIO + route::I2C0_ROUTEEN, 0x4003_C490);
    }
}
