//! SPI1 and the two devices sharing its MOSI line (UM2825, ST BSP):
//!
//! * the SSD1315 128x64 OLED: SPI1 SCK PA1 / MOSI PA7, CS PH0, D/C PC9,
//!   RES PC8 (feature `lcd`);
//! * the TLC59731 RGB LED: its EasySet SDI is PA7 gated by the "LED select"
//!   line PH1 (feature `rgb-led`).
//!
//! [`Spi1Bus`] owns the pins so that the two never overlap: PA7 is an SPI
//! alternate function only for the length of one LCD transaction (CS low,
//! LED select low) and a push-pull GPIO only for one LED write (CS high,
//! LED select high). SPI1 is enabled per transaction and its clock gated
//! off again afterwards.

#[cfg(feature = "rgb-led")]
use embassy_time::{Duration, Instant};
#[cfg(feature = "lcd")]
use embedded_hal::delay::DelayNs;
use stm32wb_hal::embassy_stm32::gpio::{Level, Output, Speed};
#[cfg(feature = "lcd")]
use stm32wb_hal::embassy_stm32::spi::{self, Spi};
#[cfg(feature = "lcd")]
use stm32wb_hal::embassy_stm32::time::Hertz;
use stm32wb_hal::embassy_stm32::{Peri, peripherals};

#[cfg(feature = "lcd")]
use crate::CycleDelay;

/// SPI1 SCK of the OLED (SSD1315 tCYCLE ≥ 100 ns; the BSP uses 8 MHz).
#[cfg(feature = "lcd")]
pub const LCD_SPI_HZ: u32 = 8_000_000;
/// Mirror both axes in the controller (orientation as seen with the
/// buttons below the display).
#[cfg(feature = "lcd")]
pub const LCD_ROTATE180: bool = false;
/// LED select must be low this long before a write (ST BSP
/// `PWM_LED_WriteData`: 10 ms) so that the TLC59731 starts from idle.
#[cfg(feature = "rgb-led")]
pub const RGB_SELECT_IDLE: Duration = Duration::from_millis(10);

/// The pins of SPI1 and its two devices.
pub struct Spi1Pins {
    pub spi1: Peri<'static, peripherals::SPI1>,
    pub sck: Peri<'static, peripherals::PA1>,
    pub mosi: Peri<'static, peripherals::PA7>,
    pub lcd_cs: Peri<'static, peripherals::PH0>,
    pub lcd_dc: Peri<'static, peripherals::PC9>,
    pub lcd_rst: Peri<'static, peripherals::PC8>,
    pub led_select: Peri<'static, peripherals::PH1>,
}

/// Owner of SPI1, the OLED control lines and the RGB LED select.
pub struct Spi1Bus {
    #[cfg_attr(not(feature = "lcd"), allow(dead_code))]
    spi1: Peri<'static, peripherals::SPI1>,
    #[cfg_attr(not(feature = "lcd"), allow(dead_code))]
    sck: Peri<'static, peripherals::PA1>,
    mosi: Peri<'static, peripherals::PA7>,
    /// Idle high: the OLED ignores PA7 while the LED is written.
    lcd_cs: Output<'static>,
    #[cfg_attr(not(feature = "lcd"), allow(dead_code))]
    lcd_dc: Output<'static>,
    /// Idle high (RES# released).
    #[cfg_attr(not(feature = "lcd"), allow(dead_code))]
    lcd_rst: Output<'static>,
    /// Idle low: the LED ignores PA7 while the OLED is written.
    #[cfg_attr(not(feature = "rgb-led"), allow(dead_code))]
    led_select: Output<'static>,
    #[cfg(feature = "rgb-led")]
    select_low_since: Instant,
}

impl Spi1Bus {
    /// Put both devices in their idle state. No bus traffic.
    pub fn new(pins: Spi1Pins) -> Self {
        Self {
            spi1: pins.spi1,
            sck: pins.sck,
            mosi: pins.mosi,
            lcd_cs: Output::new(pins.lcd_cs, Level::High, Speed::Medium),
            lcd_dc: Output::new(pins.lcd_dc, Level::Low, Speed::Medium),
            lcd_rst: Output::new(pins.lcd_rst, Level::High, Speed::Low),
            led_select: Output::new(pins.led_select, Level::Low, Speed::Medium),
            #[cfg(feature = "rgb-led")]
            select_low_since: Instant::now(),
        }
    }
}

#[cfg(feature = "rgb-led")]
mod rgb {
    use super::*;
    use crate::CycleDelay;
    use zigbee_tlc59731::{Error, Tlc59731};

    impl Spi1Bus {
        /// Set the RGB LED grayscale (`[red, green, blue]`, 0..=255).
        ///
        /// Waits (busy) until LED select has been low for
        /// [`RGB_SELECT_IDLE`] since the previous write, then sends the
        /// 32-bit EasySet packet with interrupts masked (about 0.4 ms).
        pub fn set_rgb(&mut self, rgb: [u8; 3]) -> Result<(), Error<core::convert::Infallible>> {
            let ready = self.select_low_since + RGB_SELECT_IDLE;
            while Instant::now() < ready {}
            self.lcd_cs.set_high();
            let sdi = Output::new(self.mosi.reborrow(), Level::Low, Speed::VeryHigh);
            self.led_select.set_high();
            let mut led = Tlc59731::new(sdi, CycleDelay);
            let result = critical_section::with(|_| led.write(&[rgb]));
            self.led_select.set_low();
            self.select_low_since = Instant::now();
            // Dropping the pin returns PA7 to analog (disconnected).
            drop(led);
            result
        }
    }
}

#[cfg(feature = "lcd")]
mod lcd {
    use super::*;
    use embedded_hal::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
    use zigbee_display::SpiInterface;
    use zigbee_ssd1306::{Panel, Ssd1306};

    /// The OLED driver over SPI1, borrowed from the bus. Construct it with
    /// [`Spi1Bus::lcd`] whenever the display is updated; call
    /// [`Ssd1306::init`] once after power-up (it pulses RES#).
    pub type Lcd<'a> =
        Ssd1306<SpiInterface<LcdSpi<'a>, &'a mut Output<'static>>, &'a mut Output<'static>>;

    /// Panel of the board: SSD1315 is register-compatible with SSD1306.
    pub const LCD_PANEL: Panel = Panel::SSD1306_128X64;

    /// SPI1 configuration for the OLED: mode 0, MSB first.
    fn lcd_config() -> spi::Config {
        let mut config = spi::Config::default();
        config.frequency = Hertz(LCD_SPI_HZ);
        config
    }

    /// [`SpiDevice`] for the OLED. SPI1 is enabled for the transaction only;
    /// only writes and delays are supported (the panel is write-only).
    pub struct LcdSpi<'a> {
        spi1: &'a mut Peri<'static, peripherals::SPI1>,
        sck: &'a mut Peri<'static, peripherals::PA1>,
        mosi: &'a mut Peri<'static, peripherals::PA7>,
        cs: &'a mut Output<'static>,
    }

    /// OLED bus error.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum LcdSpiError {
        /// SPI1 reported an error (mode fault, overrun).
        Bus,
        /// Read or transfer requested on the write-only bus.
        Unsupported,
    }

    impl embedded_hal::spi::Error for LcdSpiError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl ErrorType for LcdSpi<'_> {
        type Error = LcdSpiError;
    }

    impl SpiDevice for LcdSpi<'_> {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), LcdSpiError> {
            let mut spi = Spi::new_blocking_txonly(
                self.spi1.reborrow(),
                self.sck.reborrow(),
                self.mosi.reborrow(),
                lcd_config(),
            );
            self.cs.set_low();
            let mut result = Ok(());
            for operation in operations.iter_mut() {
                result = match operation {
                    Operation::Write(bytes) => {
                        spi.blocking_write(bytes).map_err(|_| LcdSpiError::Bus)
                    }
                    Operation::DelayNs(ns) => {
                        CycleDelay.delay_ns(*ns);
                        Ok(())
                    }
                    _ => Err(LcdSpiError::Unsupported),
                };
                if result.is_err() {
                    break;
                }
            }
            self.cs.set_high();
            // Dropping the driver gates SPI1 off and releases PA1/PA7.
            drop(spi);
            result
        }
    }

    impl Spi1Bus {
        /// The OLED driver for one update (no bus traffic until used).
        pub fn lcd(&mut self) -> Lcd<'_> {
            // LED select is low between LED writes, so PA7 traffic is
            // never seen by the TLC59731.
            let spi = LcdSpi {
                spi1: &mut self.spi1,
                sck: &mut self.sck,
                mosi: &mut self.mosi,
                cs: &mut self.lcd_cs,
            };
            Ssd1306::new(SpiInterface::new(spi, &mut self.lcd_dc), LCD_PANEL)
                .with_reset(&mut self.lcd_rst)
                .with_rotate180(LCD_ROTATE180)
        }
    }
}

#[cfg(feature = "lcd")]
pub use lcd::{LCD_PANEL, Lcd, LcdSpi, LcdSpiError};
