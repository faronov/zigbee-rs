//! DOIT/XT-ZB1 BL702 board wiring.
//!
//! Proven wiring:
//! - UART0 monitor: GPIO14 TX, GPIO15 RX, 2 Mbaud in the sensor product.
//!
//! Reference sensor wiring:
//! - I2C0: GPIO4 SCL, GPIO3 SDA at 100 kHz.
//!
//! Diagnostic header routing (not yet lab-validated by this crate):
//! - SPI0: GPIO7 SCLK, GPIO8 MOSI, GPIO9 MISO.
//!
//! GPIO7/GPIO8 are alternate USB pins on module-level descriptions. The
//! `spi_or_usb` resource is therefore exclusive and does not claim that both
//! functions may be active simultaneously. No fitted user LED is claimed
//! because the available evidence does not establish one.

#![no_std]

use bl702_hal::Peripherals;
use bl702_hal::clock::Clocks;
use bl702_hal::gpio::Pin;
use bl702_hal::peripherals::{Adc, Efuse, Flash, I2c0, Power, Pwm, Spi0, Timer0, Uart0, Uart1};
use bl702_hal::uart::{ConfigError as UartConfigError, Uart0Tx};

/// Flash capacity configured by the local XT-ZB1 reference firmware.
pub const ONBOARD_FLASH_CAPACITY: usize = 1024 * 1024;
pub const MONITOR_BAUD: u32 = 2_000_000;

pub struct MonitorResources {
    uart: Uart0,
    tx: Pin<14>,
    rx: Pin<15>,
}

impl MonitorResources {
    pub fn into_uart(self, clocks: Clocks) -> Result<Uart0Tx<14, 15>, UartConfigError> {
        Uart0Tx::new(self.uart, self.tx, self.rx, clocks, MONITOR_BAUD)
    }
}

pub struct I2cResources {
    pub peripheral: I2c0,
    pub scl: Pin<4>,
    pub sda: Pin<3>,
}

pub struct SpiOrUsbResources {
    pub peripheral: Spi0,
    pub sclk: Pin<7>,
    pub mosi: Pin<8>,
    pub miso: Pin<9>,
}

pub struct OtherPins {
    pub p0: Pin<0>,
    pub p1: Pin<1>,
    pub p2: Pin<2>,
    pub p5: Pin<5>,
    pub p6: Pin<6>,
    pub p10: Pin<10>,
    pub p11: Pin<11>,
    pub p12: Pin<12>,
    pub p13: Pin<13>,
    pub p16: Pin<16>,
    pub p17: Pin<17>,
    pub p18: Pin<18>,
    pub p19: Pin<19>,
    pub p20: Pin<20>,
    pub p21: Pin<21>,
    pub p22: Pin<22>,
    pub p23: Pin<23>,
    pub p24: Pin<24>,
    pub p25: Pin<25>,
    pub p26: Pin<26>,
    pub p27: Pin<27>,
    pub p28: Pin<28>,
    pub p29: Pin<29>,
    pub p30: Pin<30>,
    pub p31: Pin<31>,
}

/// Uniquely owned XT-ZB1 resources.
pub struct Resources {
    pub clocks: Clocks,
    pub monitor: MonitorResources,
    pub i2c0: I2cResources,
    pub spi_or_usb: SpiOrUsbResources,
    pub timer0: Timer0,
    pub adc: Adc,
    pub efuse: Efuse,
    pub flash: Flash,
    pub power: Power,
    pub pwm: Pwm,
    pub uart1: Uart1,
    pub other_pins: OtherPins,
}

/// Resources consumed by platform startup and retained by the monitor/time
/// adapter.
pub struct RuntimeResources {
    pub clocks: Clocks,
    pub monitor: MonitorResources,
    pub timer0: Timer0,
    pub efuse: Efuse,
}

/// Board resources returned to the application after runtime initialization.
///
/// This value is intentionally owned, not globally aliased. Applications can
/// move individual fields into sensor, storage, PWM, or power backends.
#[must_use = "retain or pass these exclusive application peripheral resources"]
pub struct ApplicationResources {
    pub i2c0: I2cResources,
    pub spi_or_usb: SpiOrUsbResources,
    pub adc: Adc,
    pub flash: Flash,
    pub power: Power,
    pub pwm: Pwm,
    pub uart1: Uart1,
    pub other_pins: OtherPins,
}

impl Resources {
    pub fn take() -> Option<Self> {
        Peripherals::take().map(Self::from_peripherals)
    }

    /// Separate runtime-owned resources from peripherals available to the
    /// application.
    pub fn split(self) -> (RuntimeResources, ApplicationResources) {
        (
            RuntimeResources {
                clocks: self.clocks,
                monitor: self.monitor,
                timer0: self.timer0,
                efuse: self.efuse,
            },
            ApplicationResources {
                i2c0: self.i2c0,
                spi_or_usb: self.spi_or_usb,
                adc: self.adc,
                flash: self.flash,
                power: self.power,
                pwm: self.pwm,
                uart1: self.uart1,
                other_pins: self.other_pins,
            },
        )
    }

    fn from_peripherals(peripherals: Peripherals) -> Self {
        let bl702_hal::gpio::Pins {
            p0,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
            p8,
            p9,
            p10,
            p11,
            p12,
            p13,
            p14,
            p15,
            p16,
            p17,
            p18,
            p19,
            p20,
            p21,
            p22,
            p23,
            p24,
            p25,
            p26,
            p27,
            p28,
            p29,
            p30,
            p31,
        } = peripherals.pins;
        Self {
            clocks: Clocks::rom_boot_32mhz(),
            monitor: MonitorResources {
                uart: peripherals.uart0,
                tx: p14,
                rx: p15,
            },
            i2c0: I2cResources {
                peripheral: peripherals.i2c0,
                scl: p4,
                sda: p3,
            },
            spi_or_usb: SpiOrUsbResources {
                peripheral: peripherals.spi0,
                sclk: p7,
                mosi: p8,
                miso: p9,
            },
            timer0: peripherals.timer0,
            adc: peripherals.adc,
            efuse: peripherals.efuse,
            flash: peripherals.flash,
            power: peripherals.power,
            pwm: peripherals.pwm,
            uart1: peripherals.uart1,
            other_pins: OtherPins {
                p0,
                p1,
                p2,
                p5,
                p6,
                p10,
                p11,
                p12,
                p13,
                p16,
                p17,
                p18,
                p19,
                p20,
                p21,
                p22,
                p23,
                p24,
                p25,
                p26,
                p27,
                p28,
                p29,
                p30,
                p31,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn application_owns_every_non_runtime_resource(resources: ApplicationResources) {
        let ApplicationResources {
            i2c0,
            spi_or_usb,
            adc,
            flash,
            power,
            pwm,
            uart1,
            other_pins,
        } = resources;

        let I2cResources {
            peripheral: _,
            scl: _,
            sda: _,
        } = i2c0;
        let SpiOrUsbResources {
            peripheral: _,
            sclk: _,
            mosi: _,
            miso: _,
        } = spi_or_usb;
        let _ = (adc, flash, power, pwm, uart1, other_pins);
    }

    #[test]
    fn split_type_returns_application_peripheral_ownership() {
        let _: fn(Resources) -> (RuntimeResources, ApplicationResources) = Resources::split;
        let _: fn(ApplicationResources) = application_owns_every_non_runtime_resource;
    }
}
