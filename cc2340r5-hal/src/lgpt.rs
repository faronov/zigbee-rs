//! LGPT1 single-channel PWM output.
//!
//! Mirrors `PWMTimerLPF3`/`LGPTimerLPF3` (TI Drivers): up-periodic counter,
//! channel 0 `SET_ON_0_TGL_ON_CMP` with `OUT0_EN`, idle level forced low via
//! `OUTCTL.CLROUT0`. Period/duty counts follow `PWMTimerLPF3_getPeriodCounts()`
//! (`f_clk / f_pwm - 1`) and `PWMTimerLPF3_getDutyCounts()` (fraction of the
//! period count).
//!
//! The LGPT1 clock is enabled and the pad muxed to the timer only while a
//! [`PwmActive`] guard is alive; otherwise the pad is a GPIO driven low and
//! the clock is gated.

use crate::gpio::{Dio, PinFunction};

/// `LGPT1_BASE` (`hw_memmap.h`).
pub const LGPT1_BASE: u32 = 0x4006_1000;
const CTL: u32 = 0x0C;
const OUTCTL: u32 = 0x10;
const CNTR: u32 = 0x14;
const PRECFG: u32 = 0x18;
const IMASK: u32 = 0x68;
const ICLR: u32 = 0x78;
const EMU: u32 = 0x84;
const C0CFG: u32 = 0xC0;
const TGT: u32 = 0x13C;
const C0CC: u32 = 0x140;

const CTL_C0RST: u32 = 0x100;
const CTL_C1RST: u32 = 0x200;
const CTL_C2RST: u32 = 0x400;
const CTL_INTP_LATE: u32 = 0x20;
const CTL_MODE_M: u32 = 0x7;
const CTL_MODE_UP_PER: u32 = 0x2;
const OUTCTL_CLROUT0: u32 = 0x1;
const C0CFG_CCACT_SET_ON_0_TGL_ON_CMP: u32 = 0xB;
const C0CFG_OUT0_EN: u32 = 0x100;

/// LGPT input clock (CLKSVT, no prescaler).
pub const LGPT_CLOCK_HZ: u32 = 48_000_000;
/// LGPT1 is a 16-bit timer.
pub const MAX_COUNT: u32 = 0xFFFF;

/// Timer counts for a PWM frequency and duty.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmCounts {
    /// `TGT`: period - 1.
    pub target: u16,
    /// `C0CC`: compare value (high time with `SET_ON_0_TGL_ON_CMP`).
    pub compare: u16,
}

/// Invalid PWM settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    /// The period does not fit the 16-bit counter or is below two counts.
    FrequencyOutOfRange,
    /// Duty above 100 %.
    DutyOutOfRange,
}

/// TI period/duty math for `frequency_hz` and `duty_percent`.
pub const fn pwm_counts(frequency_hz: u32, duty_percent: u8) -> Result<PwmCounts, PwmError> {
    if duty_percent > 100 {
        return Err(PwmError::DutyOutOfRange);
    }
    if frequency_hz == 0 || frequency_hz > LGPT_CLOCK_HZ / 2 {
        return Err(PwmError::FrequencyOutOfRange);
    }
    let target = LGPT_CLOCK_HZ / frequency_hz - 1;
    if target > MAX_COUNT {
        return Err(PwmError::FrequencyOutOfRange);
    }
    let mut compare = target * duty_percent as u32 / 100;
    // PWMTimerLPF3_getDutyCounts(): 100 % becomes period + 1.
    if duty_percent == 100 {
        compare = target + 1;
    }
    Ok(PwmCounts {
        target: target as u16,
        compare: compare as u16,
    })
}

/// Ownership token for `LGPT1` (from [`crate::Core`]).
pub struct Lgpt1 {
    _private: (),
}

impl Lgpt1 {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// PWM on LGPT1 channel 0.
pub struct Pwm {
    _token: Lgpt1,
    pin: Dio,
    function: PinFunction,
    counts: PwmCounts,
}

impl Pwm {
    /// Take LGPT1 and the channel-0 pad (`function` is the pad's LGPT1 C0
    /// mux from the device pin table). The pad idles as a low GPIO output.
    pub fn new(
        token: Lgpt1,
        pin: Dio,
        function: PinFunction,
        frequency_hz: u32,
        duty_percent: u8,
    ) -> Result<Self, PwmError> {
        let counts = pwm_counts(frequency_hz, duty_percent)?;
        let pwm = Self {
            _token: token,
            pin,
            function,
            counts,
        };
        #[cfg(target_os = "none")]
        pwm.idle_pin();
        Ok(pwm)
    }

    pub const fn counts(&self) -> PwmCounts {
        self.counts
    }

    pub fn release(self) -> (Lgpt1, Dio) {
        (self._token, self.pin)
    }

    /// Clock LGPT1 and start the output until the guard is dropped.
    #[cfg(target_os = "none")]
    pub fn start(&mut self) -> PwmActive<'_> {
        crate::clkctl::enable(crate::clkctl::PeripheralClock::Lgpt1);
        // LGPTimerLPF3_open(): channels reset, late interrupts, timer off.
        wr(CTL, CTL_C0RST | CTL_C1RST | CTL_C2RST | CTL_INTP_LATE);
        wr(IMASK, 0);
        wr(PRECFG, 0);
        wr(EMU, 0);
        // PWMTimerLPF3_open(): channel action and idle-low output.
        wr(C0CFG, C0CFG_CCACT_SET_ON_0_TGL_ON_CMP | C0CFG_OUT0_EN);
        wr(OUTCTL, OUTCTL_CLROUT0);
        // PWMTimerLPF3_start(): initial target/compare, then up-periodic.
        wr(TGT, u32::from(self.counts.target));
        wr(C0CC, u32::from(self.counts.compare));
        self.pin.set_output_enable(false);
        self.pin.set_ioc(self.function.portcfg());
        wr(CTL, (rd(CTL) & !CTL_MODE_M) | CTL_MODE_UP_PER);
        PwmActive { pwm: self }
    }

    #[cfg(target_os = "none")]
    fn idle_pin(&self) {
        self.pin.set_output(false);
        self.pin.set_ioc(PinFunction::Gpio.portcfg());
        self.pin.set_output_enable(true);
    }
}

/// A running PWM output; stops, idles the pad low and gates the clock on
/// drop.
#[cfg(target_os = "none")]
pub struct PwmActive<'a> {
    pwm: &'a mut Pwm,
}

#[cfg(target_os = "none")]
impl Drop for PwmActive<'_> {
    fn drop(&mut self) {
        // PWMTimerLPF3_stop(): timer off, force idle level.
        wr(CTL, rd(CTL) & !CTL_MODE_M);
        wr(OUTCTL, OUTCTL_CLROUT0);
        self.pwm.idle_pin();
        // LGPTimerLPF3_close(): reset state before gating the clock.
        wr(CTL, CTL_C0RST | CTL_C1RST | CTL_C2RST);
        wr(ICLR, 0xFFFF_FFFF);
        wr(CNTR, 0);
        crate::clkctl::disable(crate::clkctl::PeripheralClock::Lgpt1);
    }
}

#[cfg(target_os = "none")]
fn rd(offset: u32) -> u32 {
    // SAFETY: LGPT1 registers; clocked while a `PwmActive` exists.
    unsafe { ((LGPT1_BASE + offset) as *const u32).read_volatile() }
}

#[cfg(target_os = "none")]
fn wr(offset: u32, value: u32) {
    // SAFETY: LGPT1 registers; clocked while a `PwmActive` exists.
    unsafe { ((LGPT1_BASE + offset) as *mut u32).write_volatile(value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soil_probe_drive_matches_pwm_timer_lpf3() {
        // 2 MHz, 50 %: periodCounts = 24 - 1, dutyCounts = 23 * 50 %.
        assert_eq!(
            pwm_counts(2_000_000, 50),
            Ok(PwmCounts {
                target: 23,
                compare: 11
            })
        );
    }

    #[test]
    fn duty_edges_and_range_checks() {
        assert_eq!(pwm_counts(1_000_000, 0).unwrap().compare, 0);
        assert_eq!(pwm_counts(1_000_000, 100).unwrap().compare, 48);
        assert_eq!(pwm_counts(500, 50), Err(PwmError::FrequencyOutOfRange));
        assert_eq!(
            pwm_counts(733, 50),
            Ok(PwmCounts {
                target: 65_483,
                compare: 32_741
            })
        );
        assert_eq!(
            pwm_counts(48_000_000, 50),
            Err(PwmError::FrequencyOutOfRange)
        );
        assert_eq!(pwm_counts(0, 50), Err(PwmError::FrequencyOutOfRange));
        assert_eq!(pwm_counts(2_000_000, 101), Err(PwmError::DutyOutOfRange));
    }

    #[test]
    fn register_addresses_match_hw_lgpt() {
        assert_eq!(LGPT1_BASE + CTL, 0x4006_100C);
        assert_eq!(LGPT1_BASE + C0CFG, 0x4006_10C0);
        assert_eq!(LGPT1_BASE + TGT, 0x4006_113C);
        assert_eq!(LGPT1_BASE + C0CC, 0x4006_1140);
    }
}
