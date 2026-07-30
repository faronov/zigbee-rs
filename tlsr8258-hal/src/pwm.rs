//! TLSR8258 six-channel PWM with a shared validated clock and period.

use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};

#[cfg(target_arch = "tc32")]
use crate::gpio::Pin;

const REG_PWM_ENABLE: u32 = crate::mmio::REG_BASE + 0x780;
const REG_PWM0_ENABLE: u32 = crate::mmio::REG_BASE + 0x781;
const REG_PWM_CLOCK: u32 = crate::mmio::REG_BASE + 0x782;
const REG_PWM0_MODE: u32 = crate::mmio::REG_BASE + 0x783;
const REG_PWM_INVERT: u32 = crate::mmio::REG_BASE + 0x784;
const REG_PWM_CYCLE_BASE: u32 = crate::mmio::REG_BASE + 0x794;

const CLOCK_PWM: u8 = 1 << 4;
const RESET_PWM: u8 = 1 << 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    Pwm0 = 0,
    Pwm1 = 1,
    Pwm2 = 2,
    Pwm3 = 3,
    Pwm4 = 4,
    Pwm5 = 5,
}

impl Channel {
    const fn bit(self) -> u8 {
        1 << self as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub reference_hz: u32,
    pub frequency_hz: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    InvalidFrequency,
    InvalidPin,
    ChannelNotConfigured,
    DutyOutOfRange,
    InvalidDutyRatio,
}

impl embedded_hal::pwm::Error for PwmError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct Pwm {
    divider: u8,
    period_ticks: u16,
    actual_frequency_hz: u32,
    configured: u8,
    #[cfg(target_arch = "tc32")]
    pins: [Option<Pin>; 6],
}

impl Pwm {
    #[cfg(target_arch = "tc32")]
    pub fn new(_peripheral: crate::peripherals::Pwm, config: Config) -> Result<Self, PwmError> {
        let (divider, period_ticks, actual_frequency_hz) =
            frequency_config(config.reference_hz, config.frequency_hz)?;
        let controller = Self {
            divider,
            period_ticks,
            actual_frequency_hz,
            configured: 0,
            pins: [None, None, None, None, None, None],
        };
        controller.configure_peripheral();
        Ok(controller)
    }

    pub const fn actual_frequency_hz(&self) -> u32 {
        self.actual_frequency_hz
    }

    pub const fn period_ticks(&self) -> u16 {
        self.period_ticks
    }

    pub const fn max_duty_cycle(&self) -> u16 {
        self.period_ticks
    }

    pub const fn is_configured(&self, channel: Channel) -> bool {
        self.configured & channel.bit() != 0
    }

    #[cfg(target_arch = "tc32")]
    pub fn configure_channel(
        &mut self,
        channel: Channel,
        pin: Pin,
        polarity: Polarity,
    ) -> Result<(), PwmError> {
        crate::gpio::set_function(&pin, pin_function(channel)).map_err(|_| PwmError::InvalidPin)?;

        unsafe {
            let inverted = crate::mmio::r8(REG_PWM_INVERT);
            crate::mmio::w8(
                REG_PWM_INVERT,
                if matches!(polarity, Polarity::ActiveLow) {
                    inverted | channel.bit()
                } else {
                    inverted & !channel.bit()
                },
            );
            if matches!(channel, Channel::Pwm0) {
                // Only PWM0 has alternate count/IR modes; normal PWM is zero.
                crate::mmio::w8(REG_PWM0_MODE, 0);
            }
        }
        self.pins[channel as usize] = Some(pin);
        self.configured |= channel.bit();
        self.set_duty_cycle(channel, 0)
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_duty_cycle(&mut self, channel: Channel, duty: u16) -> Result<(), PwmError> {
        if !self.is_configured(channel) {
            return Err(PwmError::ChannelNotConfigured);
        }
        if duty > self.period_ticks {
            return Err(PwmError::DutyOutOfRange);
        }
        unsafe {
            crate::mmio::w32(
                cycle_register(channel),
                encode_cycle(duty, self.period_ticks),
            );
        }
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_duty_fraction(
        &mut self,
        channel: Channel,
        numerator: u16,
        denominator: u16,
    ) -> Result<(), PwmError> {
        let duty = duty_from_ratio(self.period_ticks, numerator, denominator)?;
        self.set_duty_cycle(channel, duty)
    }

    #[cfg(target_arch = "tc32")]
    pub fn enable(&mut self, channel: Channel) -> Result<(), PwmError> {
        if !self.is_configured(channel) {
            return Err(PwmError::ChannelNotConfigured);
        }
        unsafe {
            let register = if matches!(channel, Channel::Pwm0) {
                REG_PWM0_ENABLE
            } else {
                REG_PWM_ENABLE
            };
            crate::mmio::w8(register, crate::mmio::r8(register) | channel.bit());
        }
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    pub fn disable(&mut self, channel: Channel) {
        unsafe {
            let register = if matches!(channel, Channel::Pwm0) {
                REG_PWM0_ENABLE
            } else {
                REG_PWM_ENABLE
            };
            crate::mmio::w8(register, crate::mmio::r8(register) & !channel.bit());
        }
    }

    pub fn channel(&mut self, channel: Channel) -> Result<PwmOutput<'_>, PwmError> {
        if !self.is_configured(channel) {
            return Err(PwmError::ChannelNotConfigured);
        }
        Ok(PwmOutput {
            controller: self,
            channel,
        })
    }

    #[cfg(target_arch = "tc32")]
    fn configure_peripheral(&self) {
        unsafe {
            let clocks = crate::mmio::r8(crate::mmio::REG_CLK_EN0);
            crate::mmio::w8(crate::mmio::REG_CLK_EN0, clocks | CLOCK_PWM);
            let reset = crate::mmio::r8(crate::mmio::REG_RST0);
            crate::mmio::w8(crate::mmio::REG_RST0, reset | RESET_PWM);
            let reset = crate::mmio::r8(crate::mmio::REG_RST0);
            crate::mmio::w8(crate::mmio::REG_RST0, reset & !RESET_PWM);
            crate::mmio::w8(REG_PWM_CLOCK, self.divider);
            crate::mmio::w8(REG_PWM_ENABLE, 0);
            crate::mmio::w8(REG_PWM0_ENABLE, 0);
        }
    }
}

pub struct PwmOutput<'a> {
    controller: &'a mut Pwm,
    channel: Channel,
}

impl PwmOutput<'_> {
    #[cfg(target_arch = "tc32")]
    pub fn enable(&mut self) -> Result<(), PwmError> {
        self.controller.enable(self.channel)
    }

    #[cfg(target_arch = "tc32")]
    pub fn disable(&mut self) {
        self.controller.disable(self.channel);
    }
}

impl ErrorType for PwmOutput<'_> {
    type Error = PwmError;
}

impl SetDutyCycle for PwmOutput<'_> {
    fn max_duty_cycle(&self) -> u16 {
        self.controller.max_duty_cycle()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        #[cfg(target_arch = "tc32")]
        {
            return self.controller.set_duty_cycle(self.channel, duty);
        }
        #[cfg(not(target_arch = "tc32"))]
        {
            let _ = duty;
            Err(PwmError::ChannelNotConfigured)
        }
    }
}

const fn pin_function(channel: Channel) -> crate::gpio::PinFunction {
    match channel {
        Channel::Pwm0 => crate::gpio::PinFunction::Pwm0,
        Channel::Pwm1 => crate::gpio::PinFunction::Pwm1,
        Channel::Pwm2 => crate::gpio::PinFunction::Pwm2,
        Channel::Pwm3 => crate::gpio::PinFunction::Pwm3,
        Channel::Pwm4 => crate::gpio::PinFunction::Pwm4,
        Channel::Pwm5 => crate::gpio::PinFunction::Pwm5,
    }
}

const fn cycle_register(channel: Channel) -> u32 {
    REG_PWM_CYCLE_BASE + channel as u32 * 4
}

const fn encode_cycle(duty: u16, period: u16) -> u32 {
    duty as u32 | ((period as u32) << 16)
}

fn duty_from_ratio(period: u16, numerator: u16, denominator: u16) -> Result<u16, PwmError> {
    if denominator == 0 || numerator > denominator {
        return Err(PwmError::InvalidDutyRatio);
    }
    Ok(((u32::from(period) * u32::from(numerator)) / u32::from(denominator)) as u16)
}

fn frequency_config(reference_hz: u32, frequency_hz: u32) -> Result<(u8, u16, u32), PwmError> {
    if reference_hz == 0 || frequency_hz == 0 {
        return Err(PwmError::InvalidFrequency);
    }

    let resolution_denominator = u64::from(frequency_hz) * u64::from(u16::MAX);
    let factor = u64::from(reference_hz)
        .div_ceil(resolution_denominator)
        .max(1);
    if factor > 256 {
        return Err(PwmError::InvalidFrequency);
    }
    let period = u64::from(reference_hz).div_ceil(factor * u64::from(frequency_hz));
    if !(2..=u64::from(u16::MAX)).contains(&period) {
        return Err(PwmError::InvalidFrequency);
    }
    let actual = u64::from(reference_hz) / (factor * period);
    Ok(((factor - 1) as u8, period as u16, actual as u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_map_and_channel_enable_bits_match_8258_header() {
        assert_eq!(REG_PWM_ENABLE, 0x800780);
        assert_eq!(REG_PWM0_ENABLE, 0x800781);
        assert_eq!(REG_PWM_CLOCK, 0x800782);
        assert_eq!(REG_PWM0_MODE, 0x800783);
        assert_eq!(REG_PWM_INVERT, 0x800784);
        assert_eq!(REG_PWM_CYCLE_BASE, 0x800794);
        assert_eq!(Channel::Pwm0.bit(), 0x01);
        assert_eq!(Channel::Pwm1.bit(), 0x02);
        assert_eq!(Channel::Pwm5.bit(), 0x20);
    }

    #[test]
    fn one_kilohertz_uses_full_resolution_at_24mhz() {
        assert_eq!(frequency_config(24_000_000, 1_000), Ok((0, 24_000, 1_000)));
    }

    #[test]
    fn low_frequency_selects_a_representable_shared_divider() {
        assert_eq!(frequency_config(24_000_000, 10), Ok((36, 64_865, 9)));
    }

    #[test]
    fn invalid_frequencies_are_rejected() {
        assert_eq!(frequency_config(0, 1_000), Err(PwmError::InvalidFrequency));
        assert_eq!(
            frequency_config(24_000_000, 0),
            Err(PwmError::InvalidFrequency)
        );
        assert_eq!(
            frequency_config(24_000_000, 24_000_000),
            Err(PwmError::InvalidFrequency)
        );
        assert_eq!(
            frequency_config(24_000_000, 1),
            Err(PwmError::InvalidFrequency)
        );
    }

    #[test]
    fn duty_conversion_covers_off_half_and_full() {
        assert_eq!(duty_from_ratio(24_000, 0, 100), Ok(0));
        assert_eq!(duty_from_ratio(24_000, 50, 100), Ok(12_000));
        assert_eq!(duty_from_ratio(24_000, 100, 100), Ok(24_000));
    }

    #[test]
    fn invalid_duty_ratios_are_rejected() {
        assert_eq!(
            duty_from_ratio(24_000, 1, 0),
            Err(PwmError::InvalidDutyRatio)
        );
        assert_eq!(
            duty_from_ratio(24_000, 101, 100),
            Err(PwmError::InvalidDutyRatio)
        );
    }

    #[test]
    fn cycle_register_packs_cmp_then_max() {
        assert_eq!(encode_cycle(12_000, 24_000), 0x5DC0_2EE0);
        assert_eq!(cycle_register(Channel::Pwm5), 0x8007A8);
    }
}
