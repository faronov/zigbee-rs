//! The capacitive soil probe: `LGPT1` excites the probe's RC/diode network
//! with a 2-MHz square wave and the ADC reads the rectified level.
//!
//! The PWM and the ADC are clocked only inside
//! [`SoilProbe::average_microvolts`]: the PWM guard and the ADC session are
//! dropped (clocks gated, excitation pad driven low) before it returns.

use cc2340r5_hal::adc::{Adc, AdcConfig, ClockDivider};
use cc2340r5_hal::lgpt::Pwm;

/// Excitation frequency (reference: `pwmParams.periodValue = 2000000`).
pub const EXCITATION_HZ: u32 = 2_000_000;
/// Excitation duty cycle in percent.
pub const EXCITATION_DUTY_PERCENT: u8 = 50;

/// ADC settings from the reference SysConfig: VDDS reference (3.3 V
/// nominal), 12 bit, 16 sample cycles, clock divider 8.
pub const ADC_CONFIG: AdcConfig = AdcConfig {
    clock_divider: ClockDivider::Div8,
    sample_cycles: 16,
    reference_uv: 3_300_000,
};

/// Average the successful conversions of one burst, `None` when none
/// succeeded (reference `soil_moisture()` semantics).
pub fn average_successful(conversions: impl IntoIterator<Item = Option<u32>>) -> Option<u32> {
    let mut sum = 0u64;
    let mut count = 0u32;
    for microvolts in conversions.into_iter().flatten() {
        sum += u64::from(microvolts);
        count += 1;
    }
    if count == 0 {
        None
    } else {
        Some((sum / u64::from(count)) as u32)
    }
}

/// Exclusive owner of the probe's excitation PWM and detector ADC.
pub struct SoilProbe {
    pwm: Pwm,
    adc: Adc,
}

impl SoilProbe {
    pub fn new(pwm: Pwm, adc: Adc) -> Self {
        Self { pwm, adc }
    }

    pub fn release(self) -> (Pwm, Adc) {
        (self.pwm, self.adc)
    }

    /// Excite the probe, take `count` conversions and stop. Returns the mean
    /// of the successful conversions in microvolts.
    #[cfg(target_os = "none")]
    pub fn average_microvolts(&mut self, count: u8) -> Option<u32> {
        let _excitation = self.pwm.start();
        let mut session = self.adc.session();
        average_successful((0..count).map(|_| session.convert_microvolts().ok()))
        // `session` then `_excitation` drop here: ADC and LGPT1 clocks gated,
        // excitation pad idles low.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn failed_conversions_are_excluded_from_the_average() {
        assert_eq!(
            average_successful([Some(1_000_000), None, Some(2_000_000)]),
            Some(1_500_000)
        );
        assert_eq!(average_successful([None, None]), None);
        assert_eq!(average_successful([]), None);
    }

    #[test]
    fn excitation_is_representable_by_lgpt1() {
        let counts =
            cc2340r5_hal::lgpt::pwm_counts(EXCITATION_HZ, EXCITATION_DUTY_PERCENT).unwrap();
        assert_eq!(counts.target, 23);
        assert_eq!(counts.compare, 11);
    }
}
