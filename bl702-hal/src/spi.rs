//! Blocking eight-bit BL702 SPI0 master.

use core::hint::spin_loop;

use embedded_hal::spi::{ErrorKind, ErrorType, Mode, Phase, Polarity, SpiBus};

use crate::clock::{Clocks, Peripheral, configure_spi_clock, enable_and_reset};
use crate::gpio::{Alternate, Disabled, Drive, FUNCTION_SPI, Pin, Pull};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Spi0;

const GLB_BASE: u32 = 0x4000_0000;
const GLB_PARM: u32 = GLB_BASE + 0x080;
const SPI_BASE: u32 = 0x4000_a200;
const CONFIG: u32 = SPI_BASE;
const BUS_BUSY: u32 = SPI_BASE + 0x008;
const PRD_0: u32 = SPI_BASE + 0x010;
const PRD_1: u32 = SPI_BASE + 0x014;
const FIFO_CONFIG_0: u32 = SPI_BASE + 0x080;
const FIFO_CONFIG_1: u32 = SPI_BASE + 0x084;
const FIFO_WDATA: u32 = SPI_BASE + 0x088;
const FIFO_RDATA: u32 = SPI_BASE + 0x08c;

const MASTER_ENABLE: u32 = 1 << 0;
const SLAVE_ENABLE: u32 = 1 << 1;
const FRAME_SIZE_MASK: u32 = 0x3 << 2;
const POLARITY: u32 = 1 << 4;
const PHASE: u32 = 1 << 5;
const BIT_INVERSE: u32 = 1 << 6;
const BYTE_INVERSE: u32 = 1 << 7;
const RX_IGNORE: u32 = 1 << 8;
const MASTER_CONTINUOUS: u32 = 1 << 9;
const FIFO_CLEAR: u32 = (1 << 2) | (1 << 3);
const FIFO_FAULTS: u32 = 0xf << 4;
const TIMEOUT_ITERATIONS: u32 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    pub divider: u8,
    pub phase0_cycles: u16,
    pub phase1_cycles: u16,
    pub actual_hz: u32,
}

impl ClockConfig {
    /// Select a GLB divider and two SCLK phase lengths no faster than
    /// requested.
    pub const fn calculate(source_hz: u32, requested_hz: u32) -> Option<Self> {
        if source_hz == 0 || requested_hz == 0 {
            return None;
        }
        let mut divider = 1u32;
        while divider <= 32 {
            let denominator = match requested_hz.checked_mul(divider) {
                Some(value) => value,
                None => return None,
            };
            let total_cycles = source_hz.div_ceil(denominator);
            if total_cycles >= 2 && total_cycles <= 512 {
                let phase0_cycles = total_cycles / 2;
                let phase1_cycles = total_cycles - phase0_cycles;
                return Some(Self {
                    divider: (divider - 1) as u8,
                    phase0_cycles: phase0_cycles as u16,
                    phase1_cycles: phase1_cycles as u16,
                    actual_hz: source_hz / (divider * total_cycles),
                });
            }
            divider += 1;
        }
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    Timeout,
    Fifo,
    Clock,
    Route,
    Pin(crate::gpio::ConfigError),
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Fifo => ErrorKind::Overrun,
            _ => ErrorKind::Other,
        }
    }
}

pub struct Spi0Bus<const SCLK: u8, const MOSI: u8, const MISO: u8> {
    _token: Spi0,
    _sclk: Pin<SCLK, Alternate>,
    _mosi: Pin<MOSI, Alternate>,
    _miso: Pin<MISO, Alternate>,
}

impl<const SCLK: u8, const MOSI: u8, const MISO: u8> Spi0Bus<SCLK, MOSI, MISO> {
    pub fn new(
        token: Spi0,
        sclk: Pin<SCLK, Disabled>,
        mosi: Pin<MOSI, Disabled>,
        miso: Pin<MISO, Disabled>,
        clocks: Clocks,
        frequency_hz: u32,
        mode: Mode,
    ) -> Result<Self, SpiError> {
        if !valid_route(SCLK, MOSI, MISO) {
            return Err(SpiError::Route);
        }
        let clock =
            ClockConfig::calculate(clocks.bclk_hz(), frequency_hz).ok_or(SpiError::Clock)?;
        let sclk = sclk
            .into_alternate(FUNCTION_SPI, Pull::None, Drive::Milliamp9_6)
            .map_err(SpiError::Pin)?;
        let mosi = mosi
            .into_alternate(FUNCTION_SPI, Pull::None, Drive::Milliamp9_6)
            .map_err(SpiError::Pin)?;
        let miso = miso
            .into_alternate(FUNCTION_SPI, Pull::Up, Drive::Milliamp9_6)
            .map_err(SpiError::Pin)?;

        enable_and_reset(Peripheral::Spi);
        configure_spi_clock(clock.divider);
        // Pad acts as master; normal even-pin MOSI / odd-pin MISO mapping.
        configure_master_route();

        let phase0 = u32::from(clock.phase0_cycles - 1);
        let phase1 = u32::from(clock.phase1_cycles - 1);
        write32(
            PRD_0,
            phase0 | (phase1 << 8) | (phase0 << 16) | (phase1 << 24),
        );
        write32(PRD_1, phase1);

        let mode_bits = match mode.polarity {
            Polarity::IdleLow => 0,
            Polarity::IdleHigh => POLARITY,
        } | match mode.phase {
            // Bouffalo's SCLK_PH field is the inverse of the SDK phase enum.
            Phase::CaptureOnFirstTransition => PHASE,
            Phase::CaptureOnSecondTransition => 0,
        };
        rmw(
            CONFIG,
            MASTER_ENABLE
                | SLAVE_ENABLE
                | FRAME_SIZE_MASK
                | POLARITY
                | PHASE
                | BIT_INVERSE
                | BYTE_INVERSE
                | RX_IGNORE
                | MASTER_CONTINUOUS,
            MASTER_ENABLE | MASTER_CONTINUOUS | mode_bits,
        );
        rmw(FIFO_CONFIG_0, FIFO_CLEAR, FIFO_CLEAR);

        Ok(Self {
            _token: token,
            _sclk: sclk,
            _mosi: mosi,
            _miso: miso,
        })
    }

    fn exchange(&mut self, write: &[u8], mut read: Option<&mut [u8]>) -> Result<(), SpiError> {
        rmw(FIFO_CONFIG_0, FIFO_CLEAR, FIFO_CLEAR);
        let read_len = read.as_ref().map_or(0, |bytes| bytes.len());
        let count = core::cmp::max(write.len(), read_len);
        for index in 0..count {
            self.wait_tx_space()?;
            write32(
                FIFO_WDATA,
                u32::from(write.get(index).copied().unwrap_or(0)),
            );
            self.wait_rx_data()?;
            let byte = read32(FIFO_RDATA) as u8;
            if let Some(bytes) = read.as_deref_mut()
                && let Some(slot) = bytes.get_mut(index)
            {
                *slot = byte;
            }
        }
        self.wait_idle()
    }

    fn wait_tx_space(&self) -> Result<(), SpiError> {
        self.wait_for(|| read32(FIFO_CONFIG_1) & 0x7 != 0)
    }

    fn wait_rx_data(&self) -> Result<(), SpiError> {
        self.wait_for(|| (read32(FIFO_CONFIG_1) >> 8) & 0x7 != 0)
    }

    fn wait_idle(&self) -> Result<(), SpiError> {
        self.wait_for(|| read32(BUS_BUSY) & 1 == 0)
    }

    fn wait_for(&self, predicate: impl Fn() -> bool) -> Result<(), SpiError> {
        for _ in 0..TIMEOUT_ITERATIONS {
            if read32(FIFO_CONFIG_0) & FIFO_FAULTS != 0 {
                return Err(SpiError::Fifo);
            }
            if predicate() {
                return Ok(());
            }
            spin_loop();
        }
        Err(SpiError::Timeout)
    }
}

fn configure_master_route() {
    #[cfg(target_arch = "riscv32")]
    riscv::interrupt::free(|| {
        rmw(GLB_PARM, (1 << 12) | (1 << 13), 1 << 12);
    });

    #[cfg(not(target_arch = "riscv32"))]
    rmw(GLB_PARM, (1 << 12) | (1 << 13), 1 << 12);
}

const fn valid_route(sclk: u8, mosi: u8, miso: u8) -> bool {
    sclk & 3 == 3 && mosi & 3 == 0 && miso & 3 == 1
}

impl<const SCLK: u8, const MOSI: u8, const MISO: u8> ErrorType for Spi0Bus<SCLK, MOSI, MISO> {
    type Error = SpiError;
}

impl<const SCLK: u8, const MOSI: u8, const MISO: u8> SpiBus for Spi0Bus<SCLK, MOSI, MISO> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.exchange(&[], Some(words))
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.exchange(words, None)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.exchange(write, Some(read))
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        rmw(FIFO_CONFIG_0, FIFO_CLEAR, FIFO_CLEAR);
        for word in words {
            self.wait_tx_space()?;
            write32(FIFO_WDATA, u32::from(*word));
            self.wait_rx_data()?;
            *word = read32(FIFO_RDATA) as u8;
        }
        self.wait_idle()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_math_is_bounded_and_not_faster() {
        let config = ClockConfig::calculate(32_000_000, 8_000_000).unwrap();
        assert_eq!(config.actual_hz, 8_000_000);
        assert_eq!(config.divider, 0);
        assert_eq!(config.phase0_cycles, 2);
        assert_eq!(config.phase1_cycles, 2);

        let slow = ClockConfig::calculate(32_000_000, 4_000).unwrap();
        assert!(slow.actual_hz <= 4_000);
        assert!(slow.divider <= 31);
        assert_eq!(ClockConfig::calculate(32_000_000, 1_000), None);
    }

    #[test]
    fn zero_rate_is_rejected() {
        assert_eq!(ClockConfig::calculate(32_000_000, 0), None);
    }

    #[test]
    fn xt_zb1_diagnostic_route_matches_spi_signal_slots() {
        assert!(valid_route(7, 8, 9));
        assert!(valid_route(3, 0, 1));
        assert!(!valid_route(8, 7, 9));
    }
}
