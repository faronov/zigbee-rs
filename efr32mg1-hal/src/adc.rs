//! EFR32MG1 ADC0 single-conversion support for measuring AVDD.
//!
//! This driver uses the synchronous HFPER clock, the internal 5 V reference,
//! the AVDD positive input, VSS negative input, 12-bit resolution, and a
//! 256-cycle acquisition time. Every hardware wait is bounded.

use crate::clock;

const ADC0_BASE: u32 = 0x4000_2000;
const ADC_CTRL: u32 = ADC0_BASE;
const ADC_CMD: u32 = ADC0_BASE + 0x008;
const ADC_STATUS: u32 = ADC0_BASE + 0x00C;
const ADC_SINGLECTRL: u32 = ADC0_BASE + 0x010;
const ADC_CAL: u32 = ADC0_BASE + 0x034;
const ADC_IF: u32 = ADC0_BASE + 0x038;
const ADC_IFC: u32 = ADC0_BASE + 0x040;
const ADC_SINGLEDATA: u32 = ADC0_BASE + 0x048;
const ADC_SINGLEFIFOCLEAR: u32 = ADC0_BASE + 0x08C;

const DEVINFO_BASE: u32 = 0x0FE0_81B0;
const DEVINFO_ADC0CAL1: u32 = DEVINFO_BASE + 0x064;

const ADC_CMD_SINGLESTART: u32 = 1 << 0;
const ADC_CMD_SINGLESTOP: u32 = 1 << 1;
const ADC_CMD_SCANSTOP: u32 = 1 << 3;
const ADC_STATUS_SINGLEACT: u32 = 1 << 0;
const ADC_STATUS_SCANACT: u32 = 1 << 1;
const ADC_IF_SINGLE: u32 = 1 << 0;
const ADC_IF_SINGLEOF: u32 = 1 << 8;
const ADC_IF_VREFOV: u32 = 1 << 24;
const ADC_IF_PROGERR: u32 = 1 << 25;
const ADC_IFC_CLEARABLE_MASK: u32 = 0x0303_0F00;
const ADC_SINGLEFIFOCLEAR_CLEAR: u32 = 1 << 0;

const ADC_CTRL_PRESC_SHIFT: u32 = 8;
const ADC_CTRL_TIMEBASE_SHIFT: u32 = 16;
const ADC_CTRL_FIELD_MAX: u32 = 0x7F;

const ADC_SINGLECTRL_REF_5V: u32 = 3 << 5;
const ADC_SINGLECTRL_POSSEL_AVDD: u32 = 0xE0 << 8;
const ADC_SINGLECTRL_NEGSEL_VSS: u32 = 0xFF << 16;
const ADC_SINGLECTRL_AT_256CYCLES: u32 = 9 << 24;

const ADC_CAL_SINGLE_FIELDS: u32 = 0x0000_7FFF;
const DEVINFO_5V_OFFSET_MASK: u32 = 0x000F_0000;
const DEVINFO_5V_OFFSET_INV_MASK: u32 = 0x00F0_0000;
const DEVINFO_5V_GAIN_MASK: u32 = 0x7F00_0000;

const ADC_MIN_CLOCK_HZ: u32 = 32_000;
const ADC_MAX_CLOCK_HZ: u32 = 16_000_000;
const ADC_REFERENCE_MV: u32 = 5_000;
const ADC_12BIT_MAX: u16 = 0x0FFF;

/// ADC0 clocking and bounded-poll configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Frequency of the synchronous HFPER clock feeding ADC0.
    pub reference_hz: u32,
    /// Requested ADC clock. The selected clock is never faster than this.
    pub adc_hz: u32,
    /// Maximum status-register polls for each hardware wait.
    pub timeout_iterations: u32,
}

/// Errors returned by the ADC0 AVDD driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    InvalidConfig,
    StopTimeout,
    ConversionTimeout,
    FifoOverflow,
    ReferenceOvervoltage,
    ProgrammingError,
    InvalidSample,
}

/// Exclusive ADC0 handle configured for single, polled AVDD conversions.
pub struct Adc0 {
    timeout_iterations: u32,
}

impl Adc0 {
    /// Enable and configure ADC0. The ADC remains shut down between samples.
    pub fn new(config: Config) -> Result<Self, AdcError> {
        let (prescaler, timebase) = clock_fields(config)?;
        clock::enable_adc0_clock();

        let adc = Self {
            timeout_iterations: config.timeout_iterations,
        };
        unsafe {
            write(ADC_CMD, ADC_CMD_SINGLESTOP | ADC_CMD_SCANSTOP);
        }
        adc.wait_inactive()?;

        unsafe {
            // adcWarmupNormal, synchronous HFPER clock, 2x OVS default,
            // no tailgating; PRESC/TIMEBASE match ADC_Init_TypeDef.
            write(
                ADC_CTRL,
                ((prescaler as u32) << ADC_CTRL_PRESC_SHIFT)
                    | ((timebase as u32) << ADC_CTRL_TIMEBASE_SHIFT),
            );
            load_5v_single_calibration();
            write(
                ADC_SINGLECTRL,
                ADC_SINGLECTRL_REF_5V
                    | ADC_SINGLECTRL_POSSEL_AVDD
                    | ADC_SINGLECTRL_NEGSEL_VSS
                    | ADC_SINGLECTRL_AT_256CYCLES,
            );
            clear_single_state();
        }
        Ok(adc)
    }

    /// Perform one 12-bit, single-channel AVDD conversion.
    pub fn read_avdd_raw(&mut self) -> Result<u16, AdcError> {
        if unsafe { read(ADC_STATUS) } & (ADC_STATUS_SINGLEACT | ADC_STATUS_SCANACT) != 0 {
            unsafe {
                write(ADC_CMD, ADC_CMD_SINGLESTOP | ADC_CMD_SCANSTOP);
            }
            self.wait_inactive()?;
        }

        unsafe {
            clear_single_state();
            write(ADC_CMD, ADC_CMD_SINGLESTART);
        }

        for _ in 0..self.timeout_iterations {
            let flags = unsafe { read(ADC_IF) };
            if flags & ADC_IF_PROGERR != 0 {
                return self.fail_conversion(AdcError::ProgrammingError);
            }
            if flags & ADC_IF_VREFOV != 0 {
                return self.fail_conversion(AdcError::ReferenceOvervoltage);
            }
            if flags & ADC_IF_SINGLEOF != 0 {
                return self.fail_conversion(AdcError::FifoOverflow);
            }
            if flags & ADC_IF_SINGLE != 0 {
                let raw = unsafe { read(ADC_SINGLEDATA) };
                return u16::try_from(raw)
                    .ok()
                    .filter(|sample| *sample <= ADC_12BIT_MAX)
                    .ok_or(AdcError::InvalidSample);
            }
            core::hint::spin_loop();
        }

        self.fail_conversion(AdcError::ConversionTimeout)
    }

    /// Perform one AVDD conversion and return supply millivolts.
    pub fn read_avdd_millivolts(&mut self) -> Result<u16, AdcError> {
        avdd_raw_to_millivolts(self.read_avdd_raw()?)
    }

    fn fail_conversion<T>(&self, error: AdcError) -> Result<T, AdcError> {
        unsafe {
            write(ADC_CMD, ADC_CMD_SINGLESTOP);
        }
        self.wait_inactive()?;
        unsafe {
            clear_single_state();
        }
        Err(error)
    }

    fn wait_inactive(&self) -> Result<(), AdcError> {
        for _ in 0..self.timeout_iterations {
            if unsafe { read(ADC_STATUS) } & (ADC_STATUS_SINGLEACT | ADC_STATUS_SCANACT) == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(AdcError::StopTimeout)
    }
}

/// Convert a right-adjusted 12-bit AVDD result made against the 5 V reference.
pub const fn avdd_raw_to_millivolts(raw: u16) -> Result<u16, AdcError> {
    if raw > ADC_12BIT_MAX {
        return Err(AdcError::InvalidSample);
    }
    Ok(((raw as u32 * ADC_REFERENCE_MV) / ADC_12BIT_MAX as u32) as u16)
}

fn clock_fields(config: Config) -> Result<(u8, u8), AdcError> {
    if config.reference_hz == 0
        || config.adc_hz < ADC_MIN_CLOCK_HZ
        || config.adc_hz > ADC_MAX_CLOCK_HZ
        || config.timeout_iterations == 0
    {
        return Err(AdcError::InvalidConfig);
    }

    let divisor = (config.reference_hz as u64).div_ceil(config.adc_hz as u64);
    if divisor == 0 || divisor - 1 > ADC_CTRL_FIELD_MAX as u64 {
        return Err(AdcError::InvalidConfig);
    }
    let actual_adc_hz = config.reference_hz as u64 / divisor;
    if actual_adc_hz < ADC_MIN_CLOCK_HZ as u64 || actual_adc_hz > ADC_MAX_CLOCK_HZ as u64 {
        return Err(AdcError::InvalidConfig);
    }

    // ADC_TimebaseCalc: N + 1 HFPER cycles must span at least 1 us.
    let timebase_cycles = (config.reference_hz as u64).div_ceil(1_000_000);
    if timebase_cycles == 0 || timebase_cycles - 1 > ADC_CTRL_FIELD_MAX as u64 {
        return Err(AdcError::InvalidConfig);
    }

    Ok(((divisor - 1) as u8, (timebase_cycles - 1) as u8))
}

unsafe fn load_5v_single_calibration() {
    let devinfo = unsafe { read(DEVINFO_ADC0CAL1) };
    let calibration = ((devinfo & DEVINFO_5V_OFFSET_MASK) >> 16)
        | ((devinfo & DEVINFO_5V_OFFSET_INV_MASK) >> 16)
        | ((devinfo & DEVINFO_5V_GAIN_MASK) >> 16);
    unsafe {
        modify(
            ADC_CAL,
            ADC_CAL_SINGLE_FIELDS,
            calibration & ADC_CAL_SINGLE_FIELDS,
        );
    }
}

unsafe fn clear_single_state() {
    unsafe {
        write(ADC_SINGLEFIFOCLEAR, ADC_SINGLEFIFOCLEAR_CLEAR);
        write(ADC_IFC, ADC_IFC_CLEARABLE_MASK);
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

#[inline]
unsafe fn modify(address: u32, mask: u32, value: u32) {
    let current = unsafe { read(address) };
    unsafe { write(address, (current & !mask) | (value & mask)) };
}

#[cfg(test)]
mod tests {
    use super::{AdcError, Config, avdd_raw_to_millivolts, clock_fields};

    #[test]
    fn clock_fields_match_emlib_at_38m4() {
        assert_eq!(
            clock_fields(Config {
                reference_hz: 38_400_000,
                adc_hz: 1_000_000,
                timeout_iterations: 1,
            }),
            Ok((38, 38))
        );
    }

    #[test]
    fn five_volt_conversion_has_expected_endpoints() {
        assert_eq!(avdd_raw_to_millivolts(0), Ok(0));
        assert_eq!(avdd_raw_to_millivolts(2_457), Ok(3_000));
        assert_eq!(avdd_raw_to_millivolts(4_095), Ok(5_000));
        assert_eq!(avdd_raw_to_millivolts(4_096), Err(AdcError::InvalidSample));
    }

    #[test]
    fn rejects_unrepresentable_clock_configuration() {
        assert_eq!(
            clock_fields(Config {
                reference_hz: 38_400_000,
                adc_hz: 0,
                timeout_iterations: 1,
            }),
            Err(AdcError::InvalidConfig)
        );
        assert_eq!(
            clock_fields(Config {
                reference_hz: 200_000_000,
                adc_hz: 1_000_000,
                timeout_iterations: 1,
            }),
            Err(AdcError::InvalidConfig)
        );
    }
}
