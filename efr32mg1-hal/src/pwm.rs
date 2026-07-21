//! TIMER0 compare-channel PWM with validated Series 1 LOC routing.

use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};

use crate::{
    clock,
    gpio::{Mode, Pin},
    routing::{RouteSignal, signal_pin},
};

const TIMER0_BASE: u32 = 0x4001_8000;
const TIMER_CTRL: u32 = TIMER0_BASE;
const TIMER_CMD: u32 = TIMER0_BASE + 0x004;
const TIMER_TOP: u32 = TIMER0_BASE + 0x01C;
const TIMER_TOPB: u32 = TIMER0_BASE + 0x020;
const TIMER_CNT: u32 = TIMER0_BASE + 0x024;
const TIMER_ROUTEPEN: u32 = TIMER0_BASE + 0x030;
const TIMER_ROUTELOC0: u32 = TIMER0_BASE + 0x034;
const TIMER_CC_BASE: u32 = TIMER0_BASE + 0x060;
const TIMER_CC_STRIDE: u32 = 0x010;
const TIMER_CC_CTRL_OFFSET: u32 = 0;
const TIMER_CC_CCV_OFFSET: u32 = 0x004;
const TIMER_CC_CCVB_OFFSET: u32 = 0x00C;

const TIMER_CMD_START: u32 = 1 << 0;
const TIMER_CMD_STOP: u32 = 1 << 1;
const TIMER_CC_CTRL_MODE_PWM: u32 = 3;
const TIMER_CC_CTRL_OUTINV: u32 = 1 << 2;
const TIMER_CC_CTRL_CMOA_TOGGLE: u32 = 1 << 8;
const TIMER_CC_CTRL_ICEDGE_BOTH: u32 = 2 << 24;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    Cc0 = 0,
    Cc1 = 1,
    Cc2 = 2,
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
    pub pin: Pin,
    pub channel: Channel,
    pub location: u8,
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    InvalidConfig,
    InvalidRoute,
    DutyOutOfRange,
}

impl embedded_hal::pwm::Error for PwmError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct Timer0Pwm {
    pin: Pin,
    channel: Channel,
    top: u16,
    max_duty: u16,
    polarity: Polarity,
    forced_active: bool,
    actual_frequency_hz: u32,
}

impl Timer0Pwm {
    pub fn new(config: Config) -> Result<Self, PwmError> {
        let (prescaler, top, actual_frequency_hz) =
            frequency_config(config.reference_hz, config.frequency_hz)?;
        if route_pin(config.channel, config.location) != Some(config.pin) {
            return Err(PwmError::InvalidRoute);
        }

        clock::enable_gpio_clock();
        clock::enable_timer0_clock();
        config.pin.configure(
            Mode::PushPull,
            matches!(config.polarity, Polarity::ActiveLow),
        );

        let channel = config.channel as u32;
        let route_shift = channel * 8;
        let route_mask = 0x1F << route_shift;
        let route_enable = 1 << channel;
        let cc_base = TIMER_CC_BASE + channel * TIMER_CC_STRIDE;
        let mut cc_control =
            TIMER_CC_CTRL_MODE_PWM | TIMER_CC_CTRL_CMOA_TOGGLE | TIMER_CC_CTRL_ICEDGE_BOTH;
        if matches!(config.polarity, Polarity::ActiveLow) {
            cc_control |= TIMER_CC_CTRL_OUTINV;
        }

        unsafe {
            write(TIMER_CMD, TIMER_CMD_STOP);
            modify(TIMER_ROUTEPEN, route_enable, 0);
            modify(
                TIMER_ROUTELOC0,
                route_mask,
                (config.location as u32) << route_shift,
            );
            write(TIMER_CTRL, (prescaler as u32) << 24);
            write(TIMER_CNT, 0);
            write(TIMER_TOP, top as u32);
            write(TIMER_TOPB, top as u32);
            write(cc_base + TIMER_CC_CTRL_OFFSET, cc_control);
            write(cc_base + TIMER_CC_CCV_OFFSET, 0);
            write(cc_base + TIMER_CC_CCVB_OFFSET, 0);
            modify(TIMER_ROUTEPEN, route_enable, route_enable);
            write(TIMER_CMD, TIMER_CMD_START);
        }

        Ok(Self {
            pin: config.pin,
            channel: config.channel,
            top,
            max_duty: max_duty(top),
            polarity: config.polarity,
            forced_active: false,
            actual_frequency_hz,
        })
    }

    pub const fn actual_frequency_hz(&self) -> u32 {
        self.actual_frequency_hz
    }

    pub fn enable_output(&mut self) {
        if self.forced_active {
            return;
        }
        unsafe {
            modify(
                TIMER_ROUTEPEN,
                1 << self.channel as u32,
                1 << self.channel as u32,
            );
        }
    }

    pub fn disable_output(&mut self) {
        unsafe {
            modify(TIMER_ROUTEPEN, 1 << self.channel as u32, 0);
        }
        if self.forced_active {
            self.pin
                .configure(Mode::PushPull, matches!(self.polarity, Polarity::ActiveLow));
            self.forced_active = false;
        }
    }

    pub fn stop(&mut self) {
        unsafe {
            write(TIMER_CMD, TIMER_CMD_STOP);
        }
    }

    pub fn start(&mut self) {
        unsafe {
            write(TIMER_CMD, TIMER_CMD_START);
        }
    }

    pub fn release(mut self) -> Pin {
        self.disable_output();
        self.stop();
        self.pin.configure(Mode::Disabled, false);
        self.pin
    }
}

impl ErrorType for Timer0Pwm {
    type Error = PwmError;
}

impl SetDutyCycle for Timer0Pwm {
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        if duty > self.max_duty {
            return Err(PwmError::DutyOutOfRange);
        }

        // A 16-bit timer with TOP=0xFFFF cannot encode TOP+1 in CCVB.
        // Drive the GPIO directly for the one unrepresentable full-on state.
        if self.top == u16::MAX && duty == self.max_duty {
            self.disable_output();
            self.pin.configure(
                Mode::PushPull,
                matches!(self.polarity, Polarity::ActiveHigh),
            );
            self.forced_active = true;
            return Ok(());
        }
        if self.forced_active {
            self.pin
                .configure(Mode::PushPull, matches!(self.polarity, Polarity::ActiveLow));
            self.forced_active = false;
            self.enable_output();
        }

        let cc_base = TIMER_CC_BASE + self.channel as u32 * TIMER_CC_STRIDE;
        unsafe {
            write(cc_base + TIMER_CC_CCVB_OFFSET, duty as u32);
        }
        Ok(())
    }
}

fn route_pin(channel: Channel, location: u8) -> Option<Pin> {
    let signal = match channel {
        Channel::Cc0 => RouteSignal::Primary,
        Channel::Cc1 => RouteSignal::Secondary,
        Channel::Cc2 => RouteSignal::Tertiary,
    };
    signal_pin(signal, location)
}

const fn max_duty(top: u16) -> u16 {
    match top.checked_add(1) {
        Some(value) => value,
        None => u16::MAX,
    }
}

fn frequency_config(reference_hz: u32, frequency_hz: u32) -> Result<(u8, u16, u32), PwmError> {
    if reference_hz == 0 || frequency_hz == 0 {
        return Err(PwmError::InvalidConfig);
    }

    for prescaler in 0..=10u8 {
        let divisor = 1u64 << prescaler;
        let denominator = divisor * u64::from(frequency_hz);
        let period_ticks = u64::from(reference_hz).div_ceil(denominator);
        if (2..=65_536).contains(&period_ticks) {
            let actual = u64::from(reference_hz) / (divisor * period_ticks);
            return Ok((prescaler, (period_ticks - 1) as u16, actual as u32));
        }
    }
    Err(PwmError::InvalidConfig)
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

#[cfg(test)]
mod tests {
    use super::{Channel, PwmError, frequency_config, max_duty, route_pin};
    use crate::gpio::{Pin, Port};

    #[test]
    fn one_kilohertz_uses_exact_divide_by_one_period() {
        assert_eq!(frequency_config(38_400_000, 1_000), Ok((0, 38_399, 1_000)));
    }

    #[test]
    fn one_hertz_selects_representable_prescaler() {
        assert_eq!(frequency_config(38_400_000, 1), Ok((10, 37_499, 1)));
    }

    #[test]
    fn rejects_frequency_above_timer_resolution() {
        assert_eq!(
            frequency_config(38_400_000, 38_400_000),
            Err(PwmError::InvalidConfig)
        );
    }

    #[test]
    fn tradfri_led_has_a_valid_timer_route() {
        assert_eq!(route_pin(Channel::Cc0, 0), Some(Pin::new(Port::A, 0)));
    }

    #[test]
    fn full_scale_uses_top_plus_one_when_representable() {
        assert_eq!(max_duty(38_399), 38_400);
        assert_eq!(max_duty(u16::MAX), u16::MAX);
    }
}
