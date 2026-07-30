//! BL702 PWM channels with frequency and duty-cycle configuration.

use core::hint::spin_loop;
use core::marker::PhantomData;

use crate::clock::{Clocks, Peripheral, enable_and_reset};
use crate::gpio::{Alternate, Disabled, Drive, FUNCTION_PWM, Pin, Pull};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Pwm;

const PWM_BASE: u32 = 0x4000_a400;
const CHANNEL_OFFSET: u32 = 0x20;
const CHANNEL_STRIDE: u32 = 0x20;
const CLKDIV: u32 = 0x00;
const THRESHOLD1: u32 = 0x04;
const THRESHOLD2: u32 = 0x08;
const PERIOD: u32 = 0x0c;
const CONFIG: u32 = 0x10;
const CLOCK_BCLK: u32 = 1;
const POLARITY_INVERT: u32 = 1 << 2;
const STOP_GRACEFUL: u32 = 1 << 3;
const STOP_ENABLE: u32 = 1 << 6;
const STOPPED_AT_TOP: u32 = 1 << 7;
const TIMEOUT_ITERATIONS: u32 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrequencyConfig {
    /// Input-clock divisor as stored directly by the BL702 PWM register.
    pub divider: u16,
    pub period: u16,
    pub actual_hz: u32,
}

impl FrequencyConfig {
    /// Calculate a BCLK configuration no faster than the requested frequency.
    pub const fn calculate(source_hz: u32, requested_hz: u32) -> Option<Self> {
        if source_hz == 0 || requested_hz == 0 || requested_hz > source_hz {
            return None;
        }
        let total_ticks = source_hz.div_ceil(requested_hz);
        let divider = total_ticks.div_ceil(u16::MAX as u32);
        if divider == 0 || divider > u16::MAX as u32 {
            return None;
        }
        let denominator = match requested_hz.checked_mul(divider) {
            Some(value) => value,
            None => return None,
        };
        let period = source_hz.div_ceil(denominator);
        if period == 0 || period > u16::MAX as u32 {
            return None;
        }
        Some(Self {
            divider: divider as u16,
            period: period as u16,
            actual_hz: source_hz / (divider * period),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    InvalidFrequency,
    InvalidDuty,
    Route,
    Timeout,
    Pin(crate::gpio::ConfigError),
}

pub struct ChannelToken<const CH: u8> {
    _private: (),
}

pub struct Channels {
    pub ch0: ChannelToken<0>,
    pub ch1: ChannelToken<1>,
    pub ch2: ChannelToken<2>,
    pub ch3: ChannelToken<3>,
    pub ch4: ChannelToken<4>,
}

impl Pwm {
    pub fn split(self) -> Channels {
        enable_and_reset(Peripheral::Pwm);
        Channels {
            ch0: ChannelToken { _private: () },
            ch1: ChannelToken { _private: () },
            ch2: ChannelToken { _private: () },
            ch3: ChannelToken { _private: () },
            ch4: ChannelToken { _private: () },
        }
    }
}

pub struct PwmChannel<const CH: u8, const PIN: u8> {
    period: u16,
    _token: ChannelToken<CH>,
    _pin: Pin<PIN, Alternate>,
    _marker: PhantomData<()>,
}

impl<const CH: u8, const PIN: u8> PwmChannel<CH, PIN> {
    pub fn new(
        token: ChannelToken<CH>,
        pin: Pin<PIN, Disabled>,
        clocks: Clocks,
        frequency_hz: u32,
        inverted: bool,
    ) -> Result<Self, PwmError> {
        if CH >= 5 || PIN % 5 != CH {
            return Err(PwmError::Route);
        }
        let frequency = FrequencyConfig::calculate(clocks.bclk_hz(), frequency_hz)
            .ok_or(PwmError::InvalidFrequency)?;
        let pin = pin
            .into_alternate(FUNCTION_PWM, Pull::None, Drive::Milliamp9_6)
            .map_err(PwmError::Pin)?;
        let base = channel_base(CH);
        stop(base)?;
        write32(base + CLKDIV, u32::from(frequency.divider));
        write32(base + THRESHOLD1, 0);
        write32(base + THRESHOLD2, 0);
        write32(base + PERIOD, u32::from(frequency.period));
        rmw(
            base + CONFIG,
            0x3 | POLARITY_INVERT | STOP_GRACEFUL,
            CLOCK_BCLK | STOP_GRACEFUL | if inverted { POLARITY_INVERT } else { 0 },
        );
        rmw(base + CONFIG, STOP_ENABLE, 0);
        Ok(Self {
            period: frequency.period,
            _token: token,
            _pin: pin,
            _marker: PhantomData,
        })
    }

    pub const fn max_duty(&self) -> u16 {
        self.period
    }

    pub fn set_duty(&mut self, duty: u16) -> Result<(), PwmError> {
        if duty > self.period {
            return Err(PwmError::InvalidDuty);
        }
        write32(channel_base(CH) + THRESHOLD1, 0);
        write32(channel_base(CH) + THRESHOLD2, u32::from(duty));
        Ok(())
    }

    pub fn set_duty_fraction(&mut self, numerator: u32, denominator: u32) -> Result<(), PwmError> {
        if denominator == 0 || numerator > denominator {
            return Err(PwmError::InvalidDuty);
        }
        let duty = (u32::from(self.period) * numerator + denominator / 2) / denominator;
        self.set_duty(duty as u16)
    }

    pub fn enable(&mut self) {
        rmw(channel_base(CH) + CONFIG, STOP_ENABLE, 0);
    }

    pub fn disable(&mut self) -> Result<(), PwmError> {
        stop(channel_base(CH))
    }
}

fn channel_base(channel: u8) -> u32 {
    PWM_BASE + CHANNEL_OFFSET + u32::from(channel) * CHANNEL_STRIDE
}

fn stop(base: u32) -> Result<(), PwmError> {
    rmw(base + CONFIG, STOP_ENABLE, STOP_ENABLE);
    for _ in 0..TIMEOUT_ITERATIONS {
        if read32(base + CONFIG) & STOPPED_AT_TOP != 0 {
            return Ok(());
        }
        spin_loop();
    }
    Err(PwmError::Timeout)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn common_frequency_uses_full_resolution() {
        let config = FrequencyConfig::calculate(32_000_000, 1_000).unwrap();
        assert_eq!(config.divider, 1);
        assert_eq!(config.period, 32_000);
        assert_eq!(config.actual_hz, 1_000);
    }

    #[test]
    fn low_frequency_remains_bounded() {
        let config = FrequencyConfig::calculate(32_000_000, 1).unwrap();
        assert!(config.divider > 1);
        assert!(config.period > 0);
        assert!(config.actual_hz <= 1);
    }

    #[test]
    fn invalid_frequency_is_rejected() {
        assert_eq!(FrequencyConfig::calculate(32_000_000, 0), None);
        assert_eq!(FrequencyConfig::calculate(32_000_000, 64_000_000), None);
    }

    #[test]
    fn channel_registers_start_at_documented_offset_and_stride() {
        assert_eq!(channel_base(0), PWM_BASE + 0x20);
        assert_eq!(channel_base(4), PWM_BASE + 0xa0);
    }
}
