//! Pure-Rust TLSR8258 ADC driver: single-ended GPIO/"VBAT-style" voltage
//! sampling on the MISC channel, transcribed from the fully open
//! `static inline` functions in `platform/chip_8258/adc.h` and the digital
//! register fields in `platform/chip_8258/register.h`/`dfifo.h`. Unlike
//! GPIO pull-resistors or I2C, every register this module touches ships as
//! open source in the vendor SDK (no disassembly was needed).
//!
//! # Scope
//!
//! Telink's TLSR8258 has no dedicated internal VBAT channel: their own
//! `adc_vbat_init()` just samples a caller-chosen GPIO pin (assumed to be
//! wired to an external resistor divider from the battery) through the same
//! MISC channel used for generic GPIO voltage sampling — see
//! `proj/drivers/drv_hw.c`'s `VOLTAGE_DETECT_ADC_PIN`, which is a
//! per-*board* `#define` (`GPIO_PC5` on the reference EVK/dongle boards,
//! `NOINPUT` on others). This module mirrors that: [`sample_gpio_mv`] reads
//! any of the ten pins the ADC MISC channel can reach in single/differential
//! mode, and the caller supplies which pin is wired to what (battery
//! divider, sensor, etc.) — **this crate cannot know your board's wiring**.
//!
//! # Calibration
//!
//! Factory per-chip calibration (`CFG_ADC_CALIBRATION`,
//! `proj/drivers/drv_nv.h`) lives in flash at a fixed offset from the
//! per-flash-size "factory config" base, which for the 512 KiB flash this
//! crate targets (matching `tlsr8258-hal/src/flash.rs`'s
//! `FACTORY_IEEE_ADDR = 0x76000`) is `0x77000 + 0xC0 = 0x770C0`. This is
//! **outside the linked image** (see `memory.x`'s excluded factory-data
//! range) and is programmed at chip manufacturing time — this module can
//! read and interpret it ([`read_calibration`], transcribed from
//! `proj/drivers/drv_calibration.c`'s `drv_calib_adc_verf()`), but cannot
//! invent it. Boards without valid factory calibration bytes fall back to
//! [`Calibration::UNCALIBRATED`] (Telink's own 1175 mV/0 mV default), which
//! is accurate to only a rough +/-10% per the vendor's own calibration
//! range checks below.

#[cfg(target_arch = "tc32")]
use super::flash;
#[cfg(target_arch = "tc32")]
use super::gpio::Pin;
use super::gpio::Port;
#[cfg(target_arch = "tc32")]
use super::mmio::{analog_read, analog_write, r8, w8, w16};

// --- Analog register addresses (platform/chip_8258/adc.h) ---
const AREG_CLK_SETTING: u8 = 0x82;
const FLD_CLK_24M_TO_SAR_EN: u8 = 1 << 6;
const AREG_ADC_SAMPLING_CLK_DIV: u8 = 0xF4;
const AREG_ADC_VREF: u8 = 0xE7;
const AREG_ADC_AIN_CHN_MISC: u8 = 0xE8;
const AREG_ADC_RES_M: u8 = 0xEC;
const FLD_ADC_RES_M: u8 = 0x03;
const FLD_ADC_EN_DIFF_CHN_M: u8 = 1 << 6;
const AREG_ADC_TSAMPLE_M: u8 = 0xEE;
const AREG_R_MAX_MC: u8 = 0xEF;
const AREG_R_MAX_C: u8 = 0xF0;
const AREG_R_MAX_S: u8 = 0xF1;
const AREG_ADC_CHN_EN: u8 = 0xF2;
// `areg_adc_vref_vbat_div` (bits[3:2]) *and* an undocumented bit4 that
// `adc_set_ain_pre_scaler()` also pokes directly (see `AREG_AIN_SCALE`
// below) share this address in the vendor source.
const AREG_ADC_VREF_VBAT_DIV: u8 = 0xF9;
const FLD_ADC_VREF_VBAT_DIV: u8 = 0x03 << 2;
/// Undocumented companion bit to the pre-scaler (`adc_set_ain_pre_scaler()`
/// in `adc.c` pokes this directly as `0xF9 |= 0x10` / `&= 0xCF` with no
/// named `FLD_*` constant in the header — transcribed verbatim from the
/// open source, not guessed).
const BIT_AIN_SCALE_BIAS_EN: u8 = 1 << 4;
const AREG_AIN_SCALE: u8 = 0xFA;
const FLD_SEL_AIN_SCALE: u8 = 0x03 << 6;
/// Bias/itrim bits[5:0] of `areg_ain_scale`, set by `adc_set_ref_voltage()`
/// whenever the reference is `ADC_VREF_1P2V` (this module's only supported
/// reference) to `0x3D` — transcribed verbatim; the individual
/// `FLD_ADC_ITRIM_*` sub-fields don't need to be broken out since the
/// vendor always writes this exact combined value for the 1.2V case.
const AIN_SCALE_ITRIM_1P2V: u8 = 0x3D;
const AREG_ADC_PGA_CTRL: u8 = 0xFC;
const FLD_PGA_ITRIM_GAIN_L: u8 = 0x03;
const FLD_PGA_ITRIM_GAIN_R: u8 = 0x03 << 2;
const FLD_ADC_MODE: u8 = 1 << 4;
const FLD_SAR_ADC_POWER_DOWN: u8 = 1 << 5;
const FLD_POWER_DOWN_PGA_CHN_L: u8 = 1 << 6;
const FLD_POWER_DOWN_PGA_CHN_R: u8 = 1 << 7;
/// `GAIN_STAGE_BIAS_PER100` (`ADC_Gain_BiasTypeDef`) — the vendor's
/// `adc_init()` sets both PGA channels' gain-stage bias current trim to
/// this value regardless of which channel is actually sampled.
const GAIN_STAGE_BIAS_PER100: u8 = 1;

/// ADC reference voltage selection (`ADC_RefVolTypeDef`). Only `Vref1P2V`
/// is used by this module (matching the vendor's own `adc_base_init`/
/// `adc_vbat_init`, which both hardcode it).
const ADC_VREF_1P2V: u8 = 2;

/// `ADC_MISC_CHN` bit in `ADC_ChTypeDef`.
const ADC_MISC_CHN: u8 = 1 << 2;

/// `ADC_PRESCALER_1F8` (1/8 attenuation) — matches both vendor init paths.
/// The scale factor used in the mV formula is `1 << prescaler_enum`.
const ADC_PRESCALE_1F8: u8 = 3;
const ADC_PRESCALE_FACTOR: u32 = 1 << ADC_PRESCALE_1F8;

/// `SAMPLING_CYCLES_6` (`ADC_SampCycTypeDef`), matching the vendor init.
const ADC_SAMPLE_CYCLES_6: u8 = 1;

/// `GND` in `ADC_InputNchTypeDef` — the negative input for single-ended
/// (vs. ground) sampling.
const ADC_AIN_GND: u8 = 15;

// --- Digital registers (platform/chip_8258/register.h, dfifo.h) ---
const FLD_RST1_ADC: u8 = 1 << 5;
const REG_DFIFO2_ADDR: u32 = super::mmio::REG_BASE + 0xB08;
const REG_DFIFO2_SIZE: u32 = super::mmio::REG_BASE + 0xB0A;
const REG_DFIFO2_WPTR: u32 = super::mmio::REG_BASE + 0xB1E;
const REG_DFIFO_MODE: u32 = super::mmio::REG_BASE + 0xB10;
const FLD_AUD_DFIFO2_IN: u8 = 1 << 2;

/// Number of samples per reading and mid-point trim, matching
/// `adc_sample_and_get_result_op()` (`ADC_SAMPLE_NUM = 8`, keep the middle
/// 4 after an insert-sort).
pub const SAMPLE_COUNT: usize = 8;

/// Sample buffer type. Must live in the TLSR8258's 64 KiB SRAM window (the
/// digital `dfifo2` engine DMAs into it) — see [`SampleBuffer::at`].
#[repr(align(16))]
pub struct SampleBuffer([i32; SAMPLE_COUNT]);

impl SampleBuffer {
    pub const fn new() -> Self {
        Self([0; SAMPLE_COUNT])
    }
}

impl Default for SampleBuffer {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    /// The pin passed to [`configure_gpio_channel`] is not one of the ten
    /// pins the MISC channel can reach (`ADC_GPIO_tab` in `adc.c`: PB0-7,
    /// PC4, PC5).
    UnsupportedPin,
    /// The sample buffer is not inside the TLSR8258 SRAM window, or is not
    /// 16-byte aligned (`dfifo2` requires both).
    BufferNotInRam,
    /// The analog bus timed out reading/writing an ADC configuration
    /// register — see [`super::mmio::AnalogError`]. The ADC is left in
    /// a partially configured state when this occurs; re-run [`init`] (or
    /// whichever call failed) before trusting further samples.
    Analog(super::mmio::AnalogError),
}

impl From<super::mmio::AnalogError> for AdcError {
    fn from(error: super::mmio::AnalogError) -> Self {
        AdcError::Analog(error)
    }
}

/// Map a [`super::gpio::GpioError`] onto [`AdcError`] for the `gpio::*`
/// calls in [`configure_gpio_channel`]. Not a blanket `From` impl: this
/// module never asks `gpio` for a pull-resistor operation, so
/// `GpioError::PullNotSupportedOnPort` folding into `UnsupportedPin` here
/// is a defensive catch-all for an otherwise-unreachable case, not a
/// meaningful semantic mapping worth exposing as a general conversion.
#[cfg(target_arch = "tc32")]
fn gpio_to_adc_error(error: super::gpio::GpioError) -> AdcError {
    match error {
        super::gpio::GpioError::Analog(analog) => AdcError::Analog(analog),
        super::gpio::GpioError::PullNotSupportedOnPort => AdcError::UnsupportedPin,
    }
}

/// Read-modify-write helper: `(analog_read(addr) & !mask) | (value & mask)`.
/// Shared by every setter below that only owns a sub-field of a shared
/// analog register, so the mask/value relationship is written once instead
/// of at each call site.
#[cfg(target_arch = "tc32")]
fn analog_update(addr: u8, mask: u8, value: u8) -> Result<(), super::mmio::AnalogError> {
    let current = analog_read(addr)?;
    analog_write(addr, (current & !mask) | (value & mask))
}

/// Per-chip ADC calibration (`adc_vref`/`adc_vref_offset` in `adc.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    pub vref_mv: u16,
    pub vref_offset_mv: i16,
}

impl Calibration {
    /// Telink's own uncalibrated default (`adc_vref = 1175`,
    /// `adc_vref_offset = 0` in `adc.c`), used when no valid factory
    /// calibration is present.
    pub const UNCALIBRATED: Calibration = Calibration {
        vref_mv: 1175,
        vref_offset_mv: 0,
    };
}

/// Flash offset of the 7-byte factory ADC calibration block
/// (`CFG_ADC_CALIBRATION = FACTORY_CFG_BASE_ADD + 0xC0`) for the 512 KiB
/// flash size this crate targets (`FLASH_ADDR_OF_F_CFG_INFO_512K =
/// 0x77000`, `proj/drivers/drv_nv.h`). This is a *different* address on
/// 1 MiB/2 MiB/4 MiB flash parts — this crate only supports 512 KiB.
const CFG_ADC_CALIBRATION_512K: u32 = 0x0007_7000 + 0xC0;

/// Read and interpret the factory ADC calibration block, transcribed from
/// `drv_calib_adc_verf()`'s GPIO branch in `proj/drivers/drv_calibration.c`
/// (the 8258-only two-point/one-point selection logic; the 8278-only VBAT
/// branch is not applicable to this chip and was not ported).
///
/// Falls back to [`Calibration::UNCALIBRATED`] if the flash read fails or
/// the stored bytes don't pass the vendor's own plausibility checks (e.g.
/// erased flash reading back as `0xFF`).
#[cfg(target_arch = "tc32")]
pub fn read_calibration() -> Calibration {
    let mut raw = [0u8; 7];
    if !flash::read_bytes(CFG_ADC_CALIBRATION_512K, &mut raw) {
        return Calibration::UNCALIBRATED;
    }
    calibration_from_bytes(raw)
}

/// Pure decode of the 7-byte calibration block — split out from
/// [`read_calibration`] so the vendor arithmetic can be host-tested without
/// touching flash hardware.
pub fn calibration_from_bytes(raw: [u8; 7]) -> Calibration {
    let two_point_word = ((raw[6] as u16) << 8) + raw[5] as u16;
    if (raw[4] as i16 <= 127) && (47..=300).contains(&two_point_word) {
        // Vref = [(byte6<<8) + byte5 + 1000] mV; offset = [byte4 - 20] mV.
        let vref = two_point_word + 1000;
        let offset = raw[4] as i16 - 20;
        return Calibration {
            vref_mv: vref,
            vref_offset_mv: offset,
        };
    }
    // One-point fallback: Vref = [920 + byte0 + byte1] mV.
    let one_point = 920u16 + raw[0] as u16 + raw[1] as u16;
    if (1047..=1302).contains(&one_point) {
        return Calibration {
            vref_mv: one_point,
            vref_offset_mv: 0,
        };
    }
    Calibration::UNCALIBRATED
}

/// `ADC_GPIO_tab` from `adc.c`: the ten pins the MISC channel can sample,
/// in the order that defines their positive-input channel code
/// (`index + 1`, matching `B0P..C5P` in `ADC_InputPchTypeDef`).
const ADC_GPIO_TAB: [(Port, u8); 10] = [
    (Port::B, 0),
    (Port::B, 1),
    (Port::B, 2),
    (Port::B, 3),
    (Port::B, 4),
    (Port::B, 5),
    (Port::B, 6),
    (Port::B, 7),
    (Port::C, 4),
    (Port::C, 5),
];

fn ain_positive_channel(port: Port, bit: u8) -> Option<u8> {
    ADC_GPIO_TAB
        .iter()
        .position(|&(p, b)| p == port && b == bit)
        .map(|index| index as u8 + 1)
}

/// Digital reset pulse for the ADC module (`adc_reset_adc_module()`):
/// assert then de-assert `FLD_RST1_ADC` in `reg_rst1`. Shared by [`init`]
/// and [`sample_raw`], which — matching `adc_sample_and_get_result_op()` —
/// resets the module at the start of *every* sample, not just once at
/// startup.
#[cfg(target_arch = "tc32")]
fn reset_adc_module() {
    unsafe {
        w8(
            super::mmio::REG_RST1,
            r8(super::mmio::REG_RST1) | FLD_RST1_ADC,
        );
        w8(
            super::mmio::REG_RST1,
            r8(super::mmio::REG_RST1) & !FLD_RST1_ADC,
        );
    }
}

/// Which PGA channel [`set_pga_power`] controls (`pga_left_chn_power_on`/
/// `pga_right_chn_power_on` in `adc.c`).
#[cfg(target_arch = "tc32")]
enum PgaChannel {
    Left,
    Right,
}

/// Power a PGA channel up (`true`) or down (`false`). Same active-low
/// "power down" bit convention as [`set_powered`].
#[cfg(target_arch = "tc32")]
fn set_pga_power(channel: PgaChannel, on: bool) -> Result<(), AdcError> {
    let bit = match channel {
        PgaChannel::Left => FLD_POWER_DOWN_PGA_CHN_L,
        PgaChannel::Right => FLD_POWER_DOWN_PGA_CHN_R,
    };
    analog_update(AREG_ADC_PGA_CTRL, bit, if on { 0 } else { bit })?;
    Ok(())
}

/// Global ADC bring-up, matching `adc_init()` in `adc.c` field-for-field
/// and in the *same order*: power the SAR core and both PGA channels down
/// first (`adc_power_on_sar_adc(0)`, `pga_{left,right}_chn_power_on(0)`),
/// only *then* perform the digital reset, enable the 24 MHz source clock,
/// set the sampling clock to 4 MHz (`div = 5`), set both PGA channels'
/// gain-stage bias trim to 100%, disable `dfifo2`, and configure the
/// MISC-channel state-machine timing for the `ADC_SAMPLE_RATE_23K`
/// profile. Powering the SAR core back on is a separate, explicit
/// [`set_powered`] call, matching `drv_adc_enable()` in the vendor's own
/// `proj/drivers/drv_adc.c`.
#[cfg(target_arch = "tc32")]
pub fn init() -> Result<(), AdcError> {
    set_powered(false)?;
    set_pga_power(PgaChannel::Left, false)?;
    set_pga_power(PgaChannel::Right, false)?;

    reset_adc_module();

    analog_update(
        AREG_CLK_SETTING,
        FLD_CLK_24M_TO_SAR_EN,
        FLD_CLK_24M_TO_SAR_EN,
    )?;
    analog_write(AREG_ADC_SAMPLING_CLK_DIV, 5 & 0x07)?;

    analog_update(
        AREG_ADC_PGA_CTRL,
        FLD_PGA_ITRIM_GAIN_L,
        GAIN_STAGE_BIAS_PER100,
    )?;
    analog_update(
        AREG_ADC_PGA_CTRL,
        FLD_PGA_ITRIM_GAIN_R,
        GAIN_STAGE_BIAS_PER100 << 2,
    )?;

    unsafe {
        w8(REG_DFIFO_MODE, r8(REG_DFIFO_MODE) & !FLD_AUD_DFIFO2_IN);
    }

    // adc_set_state_length(1023, 0, 15) for ADC_SAMPLE_RATE_23K.
    analog_write(AREG_R_MAX_MC, (1023u16 & 0xFF) as u8)?;
    analog_write(AREG_R_MAX_C, 0)?;
    analog_write(AREG_R_MAX_S, ((1023u16 >> 8) as u8) << 6 | 15)?;
    Ok(())
}

/// Configure the MISC channel to sample `pin` in differential-vs-ground
/// mode (Telink's `adc_base_pin_init`/`adc_set_ain_channel_differential_mode`
/// path — every GPIO sample on this chip goes through "differential vs.
/// GND", there is no separate true single-ended mode for the MISC channel).
///
/// `drive_high` mirrors the one behavioral difference between Telink's
/// `adc_base_init` (drive_high = false: pin left as a high-Z input, for
/// passively sensing an external signal) and `adc_vbat_init` (drive_high =
/// true: pin driven high before sampling — on Telink's reference boards
/// this enables a divider/load switch; whether that is correct for *your*
/// board is exactly the kind of board-specific wiring this crate cannot
/// verify, so the choice is left explicit rather than defaulted).
#[cfg(target_arch = "tc32")]
pub fn configure_gpio_channel(pin: Pin, drive_high: bool) -> Result<(), AdcError> {
    let (port, bit) = pin.port_and_bit();
    let channel = ain_positive_channel(port, bit).ok_or(AdcError::UnsupportedPin)?;

    super::gpio::set_function_gpio(pin);
    super::gpio::set_input_enable(pin, false).map_err(gpio_to_adc_error)?;
    if drive_high {
        super::gpio::set_output_enable(pin, true);
        super::gpio::write(pin, true);
    } else {
        super::gpio::set_output_enable(pin, false);
        super::gpio::write(pin, false);
    }

    // `adc_set_vref_chn_misc(VREF_1P2V)`.
    analog_update(AREG_ADC_VREF, 0x03 << 4, ADC_VREF_1P2V << 4)?;
    // `adc_set_ref_voltage(MISC, VREF_1P2V)`'s bias-current-trim write,
    // preserving the prescaler bits (`FLD_SEL_AIN_SCALE`) at bits[7:6].
    analog_update(AREG_AIN_SCALE, !FLD_SEL_AIN_SCALE, AIN_SCALE_ITRIM_1P2V)?;
    // `adc_set_ain_chn_misc(GND, channel)` — a plain full-byte overwrite
    // in the vendor source, not a read-modify-write.
    analog_write(AREG_ADC_AIN_CHN_MISC, ADC_AIN_GND | (channel << 4))?;
    // `adc_set_input_mode_chn_misc(DIFFERENTIAL_MODE)`.
    analog_update(AREG_ADC_RES_M, FLD_ADC_EN_DIFF_CHN_M, FLD_ADC_EN_DIFF_CHN_M)?;
    // `adc_set_chn_enable_and_max_state_cnt(MISC, 2)` — also a plain
    // full-byte overwrite in the vendor source (not a read-modify-write:
    // an earlier revision of this function incorrectly preserved
    // `areg_adc_chn_en`'s upper nibble before re-ORing in the state-count
    // field, which is wrong whenever a previous state count left other
    // bits in that nibble set).
    analog_write(AREG_ADC_CHN_EN, ADC_MISC_CHN | (2 << 4))?;
    // `adc_set_vref_vbat_divider(OFF)`.
    analog_update(AREG_ADC_VREF_VBAT_DIV, FLD_ADC_VREF_VBAT_DIV, 0)?;
    // `adc_set_resolution_chn_misc(RES14)`; RES14 = 3 (`ADC_ResTypeDef`).
    analog_update(AREG_ADC_RES_M, FLD_ADC_RES_M, 3)?;
    // `adc_set_tsample_cycle_chn_misc(SAMPLING_CYCLES_6)` — plain
    // full-byte overwrite in the vendor source (bits[7:4] "not cared").
    analog_write(AREG_ADC_TSAMPLE_M, ADC_SAMPLE_CYCLES_6)?;
    // `adc_set_ain_pre_scaler(ADC_PRESCALER_1F8)`: scale bits at
    // `areg_ain_scale<7:6>`, plus its undocumented companion bit at
    // `0xF9<4>` (see `BIT_AIN_SCALE_BIAS_EN`'s docs).
    analog_update(AREG_AIN_SCALE, FLD_SEL_AIN_SCALE, ADC_PRESCALE_1F8 << 6)?;
    analog_update(
        AREG_ADC_VREF_VBAT_DIV,
        BIT_AIN_SCALE_BIAS_EN,
        BIT_AIN_SCALE_BIAS_EN,
    )?;
    // `adc_set_mode(ADC_NORMAL_MODE)`.
    analog_update(AREG_ADC_PGA_CTRL, FLD_ADC_MODE, 0)?;
    Ok(())
}

/// Power the SAR ADC core up (`true`) or down (`false`)
/// (`adc_power_on_sar_adc()`).
#[cfg(target_arch = "tc32")]
pub fn set_powered(on: bool) -> Result<(), AdcError> {
    analog_update(
        AREG_ADC_PGA_CTRL,
        FLD_SAR_ADC_POWER_DOWN,
        if on { 0 } else { FLD_SAR_ADC_POWER_DOWN },
    )?;
    Ok(())
}

/// Take [`SAMPLE_COUNT`] raw ADC codes into `buffer` via the `dfifo2` DMA
/// engine, matching the DMA setup + timing waits in
/// `adc_sample_and_get_result_op()` — including that function's own first
/// step, `adc_reset_adc_module()`, which resets the digital ADC module
/// *before* the `dfifo2` buffer is configured/enabled on every sample, not
/// just once at [`init`] time. Returns the sorted-and-trimmed average
/// *code* (not yet converted to mV) plus the full spread between the
/// highest and lowest raw sample (used by [`sample_with_fluctuation_mv`]
/// for the Zbit flash voltage guard).
#[cfg(target_arch = "tc32")]
fn sample_raw(buffer: &mut SampleBuffer) -> Result<(u32, u32), AdcError> {
    let ptr = buffer.0.as_mut_ptr() as usize;
    let byte_len = core::mem::size_of_val(&buffer.0);
    if !super::mmio::sram_contains(ptr, byte_len) || ptr % 16 != 0 {
        return Err(AdcError::BufferNotInRam);
    }

    reset_adc_module();

    for slot in buffer.0.iter_mut() {
        *slot = 0;
    }

    unsafe {
        w16(REG_DFIFO2_ADDR, (ptr & 0xFFFF) as u16);
        w8(REG_DFIFO2_SIZE, ((byte_len >> 4) - 1) as u8);
        w16(REG_DFIFO2_WPTR, 0);
        w8(REG_DFIFO_MODE, r8(REG_DFIFO_MODE) | FLD_AUD_DFIFO2_IN);
    }

    // ADC_SAMPLE_RATE_23K: wait >= 2 sample cycles (~43.4 us) before the
    // first read and between each subsequent one, matching the vendor loop.
    let wait_ticks = super::timer::us(90);
    super::timer::sleep_ticks(wait_ticks);

    let mut samples = [0u16; SAMPLE_COUNT];
    for (index, sample) in samples.iter_mut().enumerate() {
        super::timer::sleep_ticks(wait_ticks);
        let raw = unsafe { core::ptr::read_volatile(buffer.0.as_ptr().add(index)) };
        // 14-bit resolution; bit13 is the differential-mode sign bit — a
        // "negative" reading (below GND) clamps to 0, matching the vendor.
        *sample = if raw & (1 << 13) != 0 {
            0
        } else {
            (raw & 0x1FFF) as u16
        };
    }

    unsafe {
        w8(REG_DFIFO_MODE, r8(REG_DFIFO_MODE) & !FLD_AUD_DFIFO2_IN);
    }

    samples.sort_unstable();
    let average = middle_average(&samples);
    let fluctuation = (samples[SAMPLE_COUNT - 1] - samples[0]) as u32;
    Ok((average, fluctuation))
}

/// Average of the middle 4 of 8 sorted samples (discard the extreme
/// quartiles), matching `adc_sample_and_get_result_op()`. Pure/host-testable.
pub fn middle_average(sorted_samples: &[u16; SAMPLE_COUNT]) -> u32 {
    let mid = &sorted_samples[2..6];
    (mid.iter().map(|&v| v as u32).sum::<u32>()) / 4
}

/// Convert a raw averaged ADC code to millivolts using `calibration`,
/// matching `adc_vol_mv = (code * prescale * vref) >> 13 + offset`
/// (`adc_sample_and_get_result_op()`).
pub fn code_to_millivolts(code: u32, calibration: Calibration) -> u16 {
    if code == 0 {
        return 0;
    }
    let scaled = (code * ADC_PRESCALE_FACTOR * calibration.vref_mv as u32) >> 13;
    (scaled as i32 + calibration.vref_offset_mv as i32).max(0) as u16
}

/// Convert a raw code *difference* (fluctuation between the highest and
/// lowest of the [`SAMPLE_COUNT`] raw readings) to millivolts.
///
/// This is **not** the same formula as [`code_to_millivolts`]: the vendor's
/// `adc_sample_and_get_result_op()` computes the fluctuation figure as
/// `(sample[last] - sample[0]) * prescale * vref >> 13` with no
/// `+ adc_vref_offset` term — the offset only ever applies to the
/// *absolute* voltage reading, never to a delta between two samples of the
/// same offset (it would cancel out algebraically anyway; the vendor
/// source simply never adds it here). Adding the offset to a delta as
/// [`code_to_millivolts`] does would be a distinct, separate bug from just
/// reusing that function, which is why this is its own function with its
/// own test rather than a thin wrapper.
pub fn code_delta_to_millivolts(code_delta: u32, calibration: Calibration) -> u16 {
    let scaled = (code_delta * ADC_PRESCALE_FACTOR * calibration.vref_mv as u32) >> 13;
    scaled.min(u16::MAX as u32) as u16
}

/// Sample the pin configured by the most recent [`configure_gpio_channel`]
/// call and return the result in millivolts.
#[cfg(target_arch = "tc32")]
pub fn sample_gpio_mv(
    buffer: &mut SampleBuffer,
    calibration: Calibration,
) -> Result<u16, AdcError> {
    let (code, _fluctuation) = sample_raw(buffer)?;
    Ok(code_to_millivolts(code, calibration))
}

/// Sample and additionally report the spread between the highest and
/// lowest of the [`SAMPLE_COUNT`] raw readings, converted to millivolts —
/// this is the fluctuation figure Telink's own Zbit-flash voltage guard
/// checks against a threshold (see `flash.rs`'s `ensure_safe_flash`),
/// converted with [`code_delta_to_millivolts`] (*not*
/// [`code_to_millivolts`] — see that function's docs for why the two
/// differ).
#[cfg(target_arch = "tc32")]
pub fn sample_with_fluctuation_mv(
    buffer: &mut SampleBuffer,
    calibration: Calibration,
) -> Result<(u16, u16), AdcError> {
    let (code, fluctuation) = sample_raw(buffer)?;
    Ok((
        code_to_millivolts(code, calibration),
        code_delta_to_millivolts(fluctuation, calibration),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ain_channel_matches_vendor_table_order() {
        assert_eq!(ain_positive_channel(Port::B, 0), Some(1));
        assert_eq!(ain_positive_channel(Port::B, 7), Some(8));
        assert_eq!(ain_positive_channel(Port::C, 4), Some(9));
        assert_eq!(ain_positive_channel(Port::C, 5), Some(10));
    }

    #[test]
    fn ain_channel_rejects_unsupported_pins() {
        assert_eq!(ain_positive_channel(Port::A, 0), None);
        assert_eq!(ain_positive_channel(Port::C, 0), None);
        assert_eq!(ain_positive_channel(Port::D, 0), None);
    }

    #[test]
    fn middle_average_discards_extreme_quartiles() {
        // Sorted samples; extremes 0 and 9000 should be dropped, leaving
        // (100+110+120+130)/4 = 115.
        let samples = [0, 90, 100, 110, 120, 130, 140, 9000];
        assert_eq!(middle_average(&samples), 115);
    }

    #[test]
    fn code_to_millivolts_zero_code_is_zero() {
        assert_eq!(code_to_millivolts(0, Calibration::UNCALIBRATED), 0);
    }

    #[test]
    fn code_to_millivolts_matches_vendor_formula() {
        // code=4096 (0x1000), prescale=8, vref=1175:
        // (4096*8*1175) >> 13 = 4700 mV, offset 0.
        assert_eq!(code_to_millivolts(4096, Calibration::UNCALIBRATED), 4700);
    }

    #[test]
    fn code_to_millivolts_adds_calibration_offset() {
        let calibration = Calibration {
            vref_mv: 1175,
            vref_offset_mv: 50,
        };
        // Same code/vref as the vendor-formula test above, plus the +50 mV
        // offset that only applies to the absolute-voltage conversion.
        assert_eq!(code_to_millivolts(4096, calibration), 4750);
    }

    #[test]
    fn code_delta_to_millivolts_zero_delta_is_zero() {
        assert_eq!(code_delta_to_millivolts(0, Calibration::UNCALIBRATED), 0);
    }

    #[test]
    fn code_delta_to_millivolts_matches_vendor_formula() {
        // Same scale factor as code_to_millivolts (code=4096, prescale=8,
        // vref=1175 -> 4700), confirming the delta path uses the same
        // `* prescale * vref >> 13` scaling.
        assert_eq!(
            code_delta_to_millivolts(4096, Calibration::UNCALIBRATED),
            4700
        );
    }

    #[test]
    fn code_delta_to_millivolts_does_not_add_offset() {
        // This is the crux of the fix: a non-zero `vref_offset_mv` must
        // *not* change the delta conversion, unlike `code_to_millivolts`.
        let calibration = Calibration {
            vref_mv: 1175,
            vref_offset_mv: 50,
        };
        assert_eq!(code_delta_to_millivolts(4096, calibration), 4700);
        assert_ne!(
            code_delta_to_millivolts(4096, calibration),
            code_to_millivolts(4096, calibration)
        );
    }

    #[test]
    fn calibration_two_point_takes_priority() {
        // byte4=10 (offset 10-20=-10), byte5/6 encode 2000 (>=47,<=300... )
        // NOTE: two_point_word must itself be in 47..=300, it is added to
        // 1000 afterwards. Use byte6=0, byte5=200 -> word=200 -> vref=1200.
        let raw = [0, 0, 0, 0, 10, 200, 0];
        let cal = calibration_from_bytes(raw);
        assert_eq!(cal.vref_mv, 1200);
        assert_eq!(cal.vref_offset_mv, -10);
    }

    #[test]
    fn calibration_falls_back_to_one_point() {
        // Two-point word out of range (byte5/6 = 0 -> word=0, not in
        // 47..=300), so fall back to one-point: 920+byte0+byte1, which
        // must itself land in the vendor's 1047..=1302 plausibility check.
        let raw = [100, 50, 0, 0, 0, 0, 0];
        let cal = calibration_from_bytes(raw);
        assert_eq!(cal.vref_mv, 1070);
        assert_eq!(cal.vref_offset_mv, 0);
    }

    #[test]
    fn calibration_falls_back_to_uncalibrated_on_erased_flash() {
        // Erased flash reads back as 0xFF everywhere.
        let raw = [0xFF; 7];
        assert_eq!(calibration_from_bytes(raw), Calibration::UNCALIBRATED);
    }
}
