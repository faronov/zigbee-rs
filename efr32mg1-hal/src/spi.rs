//! Blocking USART0 synchronous SPI master with bounded polling.

use embedded_hal::spi::{ErrorKind, ErrorType, Mode as SpiMode, Phase, Polarity, SpiBus};

use crate::{
    clock,
    gpio::{Mode as GpioMode, Pin},
    routing::{RouteSignal, signal_pin},
};

const USART0_BASE: u32 = 0x4001_0000;
const USART_CTRL: u32 = USART0_BASE;
const USART_FRAME: u32 = USART0_BASE + 0x004;
const USART_CMD: u32 = USART0_BASE + 0x00C;
const USART_STATUS: u32 = USART0_BASE + 0x010;
const USART_CLKDIV: u32 = USART0_BASE + 0x014;
const USART_RXDATA: u32 = USART0_BASE + 0x01C;
const USART_TXDATA: u32 = USART0_BASE + 0x034;
const USART_IFC: u32 = USART0_BASE + 0x048;
const USART_IEN: u32 = USART0_BASE + 0x04C;
const USART_ROUTEPEN: u32 = USART0_BASE + 0x074;
const USART_ROUTELOC0: u32 = USART0_BASE + 0x078;

const CTRL_SYNC: u32 = 1 << 0;
const CTRL_CLKPOL: u32 = 1 << 8;
const CTRL_CLKPHA: u32 = 1 << 9;
const CTRL_MSBF: u32 = 1 << 10;
const FRAME_DATABITS_EIGHT: u32 = 5;
const CMD_RXEN: u32 = 1 << 0;
const CMD_RXDIS: u32 = 1 << 1;
const CMD_TXEN: u32 = 1 << 2;
const CMD_TXDIS: u32 = 1 << 3;
const CMD_MASTEREN: u32 = 1 << 4;
const CMD_MASTERDIS: u32 = 1 << 5;
const CMD_CLEARTX: u32 = 1 << 10;
const CMD_CLEARRX: u32 = 1 << 11;
const STATUS_TXC: u32 = 1 << 5;
const STATUS_TXBL: u32 = 1 << 6;
const STATUS_RXDATAV: u32 = 1 << 7;
const IFC_ALL: u32 = 0x0001_FFF9;
const CLKDIV_DIV_MASK: u32 = 0x007F_FFF8;
const ROUTEPEN_RXPEN: u32 = 1 << 0;
const ROUTEPEN_TXPEN: u32 = 1 << 1;
const ROUTEPEN_CLKPEN: u32 = 1 << 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    MostSignificantFirst,
    LeastSignificantFirst,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub reference_hz: u32,
    pub bus_hz: u32,
    pub mode: SpiMode,
    pub bit_order: BitOrder,
    pub mosi: Pin,
    pub miso: Pin,
    pub clock: Pin,
    pub mosi_location: u8,
    pub miso_location: u8,
    pub clock_location: u8,
    pub timeout_iterations: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    InvalidConfig,
    InvalidRoute,
    Timeout,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct Usart0Spi {
    config: Config,
    actual_bus_hz: u32,
}

impl Usart0Spi {
    pub fn new(config: Config) -> Result<Self, SpiError> {
        let (divider, actual_bus_hz) = clock_divider(config.reference_hz, config.bus_hz)?;
        validate_route(config)?;
        if config.timeout_iterations == 0 {
            return Err(SpiError::InvalidConfig);
        }

        clock::enable_gpio_clock();
        clock::enable_usart0_clock();
        config.mosi.configure(GpioMode::PushPull, false);
        config.miso.configure(GpioMode::Input, false);
        config.clock.configure(
            GpioMode::PushPull,
            matches!(config.mode.polarity, Polarity::IdleHigh),
        );

        let mut control = CTRL_SYNC;
        if matches!(config.mode.polarity, Polarity::IdleHigh) {
            control |= CTRL_CLKPOL;
        }
        if matches!(config.mode.phase, Phase::CaptureOnSecondTransition) {
            control |= CTRL_CLKPHA;
        }
        if matches!(config.bit_order, BitOrder::MostSignificantFirst) {
            control |= CTRL_MSBF;
        }

        unsafe {
            write(USART_ROUTEPEN, 0);
            write(
                USART_CMD,
                CMD_RXDIS | CMD_TXDIS | CMD_MASTERDIS | CMD_CLEARTX | CMD_CLEARRX,
            );
            write(USART_CTRL, 0);
            write(USART_FRAME, FRAME_DATABITS_EIGHT);
            write(USART_CLKDIV, divider);
            write(USART_IEN, 0);
            write(USART_IFC, IFC_ALL);
            write(
                USART_ROUTELOC0,
                (config.miso_location as u32)
                    | ((config.mosi_location as u32) << 8)
                    | ((config.clock_location as u32) << 24),
            );
            write(USART_CTRL, control);
            write(USART_CMD, CMD_MASTEREN);
            write(USART_CMD, CMD_RXEN | CMD_TXEN);
            write(
                USART_ROUTEPEN,
                ROUTEPEN_RXPEN | ROUTEPEN_TXPEN | ROUTEPEN_CLKPEN,
            );
        }

        Ok(Self {
            config,
            actual_bus_hz,
        })
    }

    pub const fn actual_bus_hz(&self) -> u32 {
        self.actual_bus_hz
    }

    pub fn disable(&mut self) {
        unsafe {
            write(USART_ROUTEPEN, 0);
            write(
                USART_CMD,
                CMD_RXDIS | CMD_TXDIS | CMD_MASTERDIS | CMD_CLEARTX | CMD_CLEARRX,
            );
        }
    }

    fn transfer_word(&mut self, word: u8) -> Result<u8, SpiError> {
        self.wait_status(STATUS_TXBL)?;
        unsafe { write(USART_TXDATA, word as u32) };
        self.wait_status(STATUS_RXDATAV)?;
        self.wait_status(STATUS_TXC)?;
        Ok(unsafe { read(USART_RXDATA) as u8 })
    }

    fn wait_status(&self, mask: u32) -> Result<(), SpiError> {
        for _ in 0..self.config.timeout_iterations {
            if unsafe { read(USART_STATUS) } & mask != 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SpiError::Timeout)
    }
}

impl ErrorType for Usart0Spi {
    type Error = SpiError;
}

impl SpiBus<u8> for Usart0Spi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word(0xFF)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            let _ = self.transfer_word(word)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read_words: &mut [u8], write_words: &[u8]) -> Result<(), Self::Error> {
        let count = read_words.len().max(write_words.len());
        for index in 0..count {
            let received = self.transfer_word(write_words.get(index).copied().unwrap_or(0xFF))?;
            if let Some(slot) = read_words.get_mut(index) {
                *slot = received;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word(*word)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_status(STATUS_TXC)
    }
}

fn validate_route(config: Config) -> Result<(), SpiError> {
    let valid = signal_pin(RouteSignal::Primary, config.mosi_location) == Some(config.mosi)
        && signal_pin(RouteSignal::Secondary, config.miso_location) == Some(config.miso)
        && signal_pin(RouteSignal::Tertiary, config.clock_location) == Some(config.clock);
    if valid {
        Ok(())
    } else {
        Err(SpiError::InvalidRoute)
    }
}

fn clock_divider(reference_hz: u32, bus_hz: u32) -> Result<(u32, u32), SpiError> {
    if reference_hz == 0 || bus_hz == 0 || bus_hz > reference_hz / 2 {
        return Err(SpiError::InvalidConfig);
    }
    let denominator = bus_hz.checked_mul(2).ok_or(SpiError::InvalidConfig)?;
    let integral = (reference_hz - 1) / denominator;
    let encoded = integral
        .checked_shl(8)
        .filter(|value| *value & !CLKDIV_DIV_MASK == 0)
        .ok_or(SpiError::InvalidConfig)?;
    let actual = reference_hz / (2 * (integral + 1));
    Ok((encoded, actual))
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
    use embedded_hal::spi::MODE_0;

    use super::{BitOrder, Config, SpiError, clock_divider, validate_route};
    use crate::gpio::{Pin, Port};

    fn tradfri_config() -> Config {
        Config {
            reference_hz: 38_400_000,
            bus_hz: 4_000_000,
            mode: MODE_0,
            bit_order: BitOrder::MostSignificantFirst,
            mosi: Pin::new(Port::D, 15),
            miso: Pin::new(Port::D, 14),
            clock: Pin::new(Port::D, 13),
            mosi_location: 23,
            miso_location: 21,
            clock_location: 19,
            timeout_iterations: 1,
        }
    }

    #[test]
    fn divider_matches_emlib_synchronous_formula() {
        assert_eq!(
            clock_divider(38_400_000, 4_000_000),
            Ok((4 << 8, 3_840_000))
        );
        assert_eq!(clock_divider(38_400_000, 1_000_000), Ok((19 << 8, 960_000)));
    }

    #[test]
    fn tradfri_flash_route_is_valid() {
        assert_eq!(validate_route(tradfri_config()), Ok(()));
    }

    #[test]
    fn mismatched_location_is_rejected() {
        let mut config = tradfri_config();
        config.clock_location = 18;
        assert_eq!(validate_route(config), Err(SpiError::InvalidRoute));
    }
}
