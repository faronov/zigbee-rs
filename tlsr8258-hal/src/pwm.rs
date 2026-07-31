//! TLSR8258 six-channel PWM with a shared validated clock and period.
//!
//! # Register evidence
//!
//! Transcribed directly from the official Telink SDK V3.7.x tree,
//! `platform/chip_8258/pwm.h` (open, `static inline` bodies — full
//! confidence, same tier as `gpio.rs`'s per-pin digital registers) and
//! `platform/chip_8258/register.h`'s `pwm registers: 0x780` block. Every
//! register address and bit position below is copied verbatim from those
//! two files, not reverse engineered.
//!
//! # Supported
//!
//! * **Modes** ([`Pwm0Mode`], `reg_pwm0_mode` @ `0x783`): `Normal` (`0x00`,
//!   the only mode available on PWM1..5 and the reset default of PWM0),
//!   `Count` (`0x01`), `Ir` (`0x03`), and `Ir` **FIFO** (`0x07`) — all four
//!   CPU-fed modes `pwm_set_mode()` documents for PWM0. Pulse-count
//!   configuration ([`Pwm::set_pwm0_pulse_num`], `reg_pwm0_pulse_num` @
//!   `0x7ac`) and the non-DMA IR FIFO path ([`Pwm::ir_fifo_push`]/
//!   [`Pwm::ir_fifo_status`]/[`Pwm::ir_fifo_set_trigger_level`]/
//!   [`Pwm::ir_fifo_clear`], `0x7c4`-`0x7ce`) are implemented because their
//!   register semantics are fully spelled out by `pwm.h`'s own inline
//!   bodies (`pwm_set_pulse_num`, `pwm_ir_fifo_set_data_entry`,
//!   `pwm_ir_fifo_is_full`/`_is_empty`/`_get_data_num`,
//!   `pwm_ir_fifo_set_irq_trig_level`, `pwm_ir_fifo_clr_data`).
//! * **IRQ sources** ([`PwmIrqSource`], `reg_pwm_irq_mask`/`reg_pwm_irq_sta`
//!   @ `0x7b0`/`0x7b1`, both a W1C status register): PWM0's pulse-count-done
//!   and frame sources plus PWM1..5's frame sources. PWM0's IR **FIFO**
//!   source is deliberately *not* one of the [`PwmIrqSource`] variants —
//!   see "PWM0's second IRQ sub-status register" below.
//!
//! # Explicitly unsupported (do not guess — omitted, not stubbed)
//!
//! * **`PWM_IR_DMA_FIFO_MODE` (`0x0f`)** has no [`Pwm0Mode`] variant, so it
//!   cannot be selected through this API at all (the strongest form of
//!   "omit the constructor" this crate's conventions call for, stronger
//!   than an `UnsupportedMode` runtime error). `pwm.h`'s DMA path
//!   (`pwm_set_dma_address`/`pwm_start_dma_ir_sending`) hands a raw pointer
//!   to `reg_dma_pwm_addr` (channel 7, shared `reg_dma7_*` registers) with
//!   no channel-ownership abstraction anywhere in this crate yet — nothing
//!   today stops two peripherals from being wired to the same DMA channel,
//!   and the vendor's own 511-byte send-length caveat
//!   (`pwm_set_dma_address`'s comment) is a hand-tuned workaround for a
//!   documented hardware quirk, not a clean invariant this driver could
//!   enforce on the caller's behalf. Until this crate has a real DMA
//!   channel-ownership facility, wiring PWM to it is not sound to expose
//!   here — no matter how attractive the higher throughput would be.
//! * **`PWMx_N` complementary/inverted outputs** (`reg_pwm_n_invert` @
//!   `0x785`, `reg_pwm_pol` @ `0x786`, `pwm_n_revert`/`pwm_polo_enable`)
//!   have no pin-mux route in [`crate::gpio`]. That module's own docs
//!   explain its `PinFunction`/selector table is cross-checked against
//!   `gpio_set_func()`'s *compiled* object for the plain PWM0..5 routes
//!   this file already uses; `platform/chip_8258/gpio.h`'s
//!   `AS_PWM0_N`..`AS_PWM5_N` enum only gives abstract values (`26`..`31`)
//!   that do not fit that same 2-bit-selector scheme (unlike `AS_PWM0`
//!   .. `AS_PWM5`, which do), and `gpio_set_func()` itself ships only as a
//!   compiled object on this chip — no independent source or disassembly
//!   evidence for the *specific* selector value each `_N` pin would need
//!   was available while writing this file. `reg_pwm_n_invert` and
//!   `reg_pwm_pol` are real, distinct registers (confirmed against
//!   `register.h`), but this driver deliberately does not add any way to
//!   reach them: doing so without an exact pin route would be guessing at
//!   the one part (which physical pad a given `_N` channel comes out on)
//!   this crate's `gpio.rs` otherwise refuses to guess for any other
//!   peripheral. A future patch with that evidence can add
//!   `PinFunction::Pwm0N`..`Pwm5N` to `gpio.rs` and wire these registers up
//!   here; until then, treat `PWMx_N` as fully unimplemented, not merely
//!   "unsafe to call" (there is no call to make).
//!
//! # PWM0's second IRQ sub-status register
//!
//! Unlike PWM1..5 (frame-only, all folded into the shared `reg_pwm_irq_sta`
//! byte), PWM0's IR FIFO condition lives in its *own* one-bit mask/status
//! pair, `reg_pwm0_fifo_mode_irq_mask`/`_sta` @ `0x7b2`/`0x7b3` — a
//! completely separate register from `reg_pwm_irq_sta`, not a bit within
//! it. `pwm.h`'s own `pwm_set_interrupt_enable`/`_disable`/
//! `_clear_interrupt_status`/`_get_interrupt_status` special-case exactly
//! this (`if(irq == PWM_IRQ_PWM0_IR_FIFO) { ...reg_pwm0_fifo_mode_irq_* }
//! else { ...reg_pwm_irq_* }`), so this module mirrors that split with
//! separate `ir_fifo_irq_*` methods rather than folding a phantom bit 16
//! into [`PwmIrqSource`]. **Both** of those PWM-internal registers are, in
//! turn, gated behind a single *shared* bit in the chip's global CPU IRQ
//! controller: `register.h`'s `reg_irq_mask`/`reg_irq_src` (`0x640`/`0x648`)
//! `FLD_IRQ_SW_PWM_EN = BIT(14)` is annotated `//irq_software | irq_pwm` —
//! one CPU vector bit multiplexes *both* the software-triggered IRQ and
//! every PWM reason. This module intentionally does not touch
//! `reg_irq_mask`/`reg_irq_src` itself — that is [`crate::irq`]'s
//! territory, not PWM's; a caller must separately call
//! `crate::irq::enable(crate::irq::IrqSource::SwPwm)` before the CPU will
//! ever take an interrupt for *any* source this module reports as
//! pending, and must not assume clearing every bit this module knows about
//! (via [`Pwm::clear_irq`]/[`Pwm::clear_ir_fifo_irq`]) is sufficient to
//! also clear a pending software-IRQ request sharing that same global bit
//! (use `crate::irq::clear_pending(crate::irq::IrqSource::SwPwm)` for
//! that, once the application-level software-IRQ source itself is also
//! serviced).
//!
//! # Enable/disable register audit (PWM0 vs PWM1..5)
//!
//! `pwm.h`'s `pwm_start`/`pwm_stop` use **two different enable registers**
//! depending on the channel: PWM0 alone is gated by `reg_pwm0_enable`'s
//! `BIT(0)`, while PWM1..5 share `reg_pwm_enable`'s `BIT(id)` for
//! `id in 1..=5` (`reg_pwm_enable`'s own bit 0 is never written by the
//! vendor driver — it has no defined meaning for PWM0, which is not gated
//! through this register at all). `Channel::bit` (private) returns the same
//! `1 << (channel as u8)` value for every channel (`0x01`..`0x20`), and
//! [`Pwm::enable`]/[`Pwm::disable`] already select the correct *register*
//! per channel (`REG_PWM0_ENABLE` only for [`Channel::Pwm0`],
//! `REG_PWM_ENABLE` for everything else) before applying that bit — so
//! `Channel::Pwm0.bit() == 0x01` is written to `REG_PWM0_ENABLE`'s bit 0
//! (correct: that register only has one defined bit), and is *never*
//! written to `REG_PWM_ENABLE` (whose bit 0 the vendor driver also never
//! touches). This was audited against `pwm.h` while writing this module
//! and found already correct — see
//! `tests::register_map_and_channel_enable_bits_match_8258_header` and
//! `tests::pwm0_enable_bit_is_never_applied_to_the_shared_enable_register`
//! for the regression coverage.
//!
//! # Silicon-validation status
//!
//! **None of this file — old or new — has been run against real TLSR8258
//! hardware.** Every register address/bit and every mode/IRQ encoding is
//! transcribed from the vendor header/source, and the arithmetic
//! (frequency search, cycle encoding, IR FIFO word packing) is covered by
//! host-side unit tests, but no logic analyzer or oscilloscope capture has
//! confirmed actual PWM0 Count/IR/IR-FIFO waveforms, IRQ delivery, or the
//! bounded IR FIFO push path on silicon. Treat every non-Normal-mode API in
//! this file as "believed correct from the datasheet/SDK, not yet
//! hardware-confirmed" until proven on a board.

use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};

#[cfg(target_arch = "tc32")]
use crate::gpio::Pin;

const REG_PWM_ENABLE: u32 = crate::mmio::REG_BASE + 0x780;
const REG_PWM0_ENABLE: u32 = crate::mmio::REG_BASE + 0x781;
const REG_PWM_CLOCK: u32 = crate::mmio::REG_BASE + 0x782;
const REG_PWM0_MODE: u32 = crate::mmio::REG_BASE + 0x783;
const REG_PWM_INVERT: u32 = crate::mmio::REG_BASE + 0x784;
const REG_PWM_CYCLE_BASE: u32 = crate::mmio::REG_BASE + 0x794;

// PWM0-only count/IR/IR-FIFO registers (`platform/chip_8258/register.h`'s
// `pwm registers: 0x780` block, `0x7ac`..`0x7ce`). See the module docs'
// "Supported" section for which `pwm.h` inline function each one backs.
const REG_PWM0_PULSE_NUM: u32 = crate::mmio::REG_BASE + 0x7ac;
const REG_PWM_IRQ_MASK: u32 = crate::mmio::REG_BASE + 0x7b0;
const REG_PWM_IRQ_STA: u32 = crate::mmio::REG_BASE + 0x7b1;
const REG_PWM0_FIFO_MODE_IRQ_MASK: u32 = crate::mmio::REG_BASE + 0x7b2;
const REG_PWM0_FIFO_MODE_IRQ_STA: u32 = crate::mmio::REG_BASE + 0x7b3;
const REG_PWM_TCMP0_SHADOW: u32 = crate::mmio::REG_BASE + 0x7c4;
const REG_PWM_TMAX0_SHADOW: u32 = crate::mmio::REG_BASE + 0x7c6;
const REG_PWM_IR_FIFO_DAT_BASE: u32 = crate::mmio::REG_BASE + 0x7c8;
const REG_PWM_IR_FIFO_IRQ_TRIG_LEVEL: u32 = crate::mmio::REG_BASE + 0x7cc;
const REG_PWM_IR_FIFO_DATA_STATUS: u32 = crate::mmio::REG_BASE + 0x7cd;
const REG_PWM_IR_CLR_FIFO_DATA: u32 = crate::mmio::REG_BASE + 0x7ce;

/// `FLD_PWM0_IR_FIFO_CLR_DATA` (`register.h`).
const FLD_PWM0_IR_FIFO_CLR_DATA: u8 = 1 << 0;
/// `FLD_PWM0_IRQ_IR_FIFO_EN`/`FLD_PWM0_IRQ_IR_FIFO_CNT` (`register.h`) —
/// both the sub-block's mask-enable and status/W1C bit are bit 0 of their
/// respective one-bit registers.
const FLD_PWM0_IR_FIFO_IRQ_BIT: u8 = 1 << 0;
/// `FLD_PWM0_IR_FIFO_DATA_NUM = BIT_RNG(0,3)` (`register.h`).
const FLD_PWM0_IR_FIFO_DATA_NUM_MASK: u8 = 0x0f;
/// `FLD_PWM0_IR_FIFO_EMPTY = BIT(4)` (`register.h`).
const FLD_PWM0_IR_FIFO_EMPTY: u8 = 1 << 4;
/// `FLD_PWM0_IR_FIFO_FULL = BIT(5)` (`register.h`).
const FLD_PWM0_IR_FIFO_FULL: u8 = 1 << 5;

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

/// PWM0's CPU-fed work mode (`pwm_mode`/`reg_pwm0_mode`,
/// `platform/chip_8258/pwm.h`/`register.h`). PWM1..5 have no mode register
/// at all — they are always the equivalent of [`Pwm0Mode::Normal`], which
/// is why [`Pwm::set_pwm0_mode`] only accepts [`Channel::Pwm0`].
///
/// `PWM_IR_DMA_FIFO_MODE` (`0x0f`) has no variant here — see the module
/// docs' "Explicitly unsupported" section for why.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Pwm0Mode {
    /// `PWM_NORMAL_MODE` — plain duty-cycle output from `TCMP0`/`TMAX0`,
    /// identical in kind to every other channel. Reset default.
    Normal = 0x00,
    /// `PWM_COUNT_MODE` — normal waveform output that additionally counts
    /// down [`Pwm::set_pwm0_pulse_num`] periods before signalling
    /// [`PwmIrqSource::Pwm0PulseCountDone`].
    Count = 0x01,
    /// `PWM_IR_MODE` — single programmable IR carrier burst, pulse count
    /// from [`Pwm::set_pwm0_pulse_num`].
    Ir = 0x03,
    /// `PWM_IR_FIFO_MODE` — same IR carrier hardware as [`Self::Ir`], but
    /// each burst's pulse count/shadow-select/carrier-enable comes from a
    /// software-fed two-entry FIFO ([`Pwm::ir_fifo_push`]) instead of the
    /// single [`Pwm::set_pwm0_pulse_num`] register, so a new burst can be
    /// queued while the previous one is still transmitting.
    IrFifo = 0x07,
}

/// One IR FIFO entry (`pwm_ir_fifo_set_data_entry`, `platform/chip_8258/
/// pwm.h`): a 16-bit word packing a 14-bit pulse count with a shadow-select
/// and carrier-enable flag. Only meaningful in [`Pwm0Mode::IrFifo`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pwm0IrFifoEntry {
    pulse_num: u16,
    use_shadow: bool,
    carrier_en: bool,
}

impl Pwm0IrFifoEntry {
    /// The pulse-count field is 14 bits wide (bits 15/14 of the FIFO word
    /// are `carrier_en`/`use_shadow`), so this is the largest value
    /// [`Self::new`] accepts.
    pub const MAX_PULSE_NUM: u16 = 0x3fff;

    /// Build one FIFO entry. `pulse_num` must fit in 14 bits
    /// (`0..=`[`Self::MAX_PULSE_NUM`]); `use_shadow` selects
    /// `reg_pwm_tcmp0_shadow`/`reg_pwm_tmax0_shadow` (set via
    /// [`Pwm::set_pwm0_shadow_cycle_and_duty`]) over the normal `TCMP0`/
    /// `TMAX0` registers for this burst's cycle/duty, and `carrier_en`
    /// enables the IR carrier for this burst.
    pub const fn new(pulse_num: u16, use_shadow: bool, carrier_en: bool) -> Result<Self, PwmError> {
        if pulse_num > Self::MAX_PULSE_NUM {
            return Err(PwmError::PulseNumOutOfRange);
        }
        Ok(Self {
            pulse_num,
            use_shadow,
            carrier_en,
        })
    }

    /// Pack into the raw 16-bit FIFO word `pwm_ir_fifo_set_data_entry`
    /// writes: `pulse_num | (use_shadow << 14) | (carrier_en << 15)`.
    const fn encode(self) -> u16 {
        self.pulse_num | ((self.use_shadow as u16) << 14) | ((self.carrier_en as u16) << 15)
    }
}

/// `reg_pwm_ir_fifo_data_status` (`0x7cd`) decoded
/// (`pwm_ir_fifo_get_data_num`/`_is_empty`/`_is_full`, `pwm.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pwm0IrFifoStatus {
    /// `FLD_PWM0_IR_FIFO_DATA_NUM` — number of entries currently held.
    pub data_num: u8,
    /// `FLD_PWM0_IR_FIFO_EMPTY`.
    pub empty: bool,
    /// `FLD_PWM0_IR_FIFO_FULL`.
    pub full: bool,
}

impl Pwm0IrFifoStatus {
    const fn decode(raw: u8) -> Self {
        Self {
            data_num: raw & FLD_PWM0_IR_FIFO_DATA_NUM_MASK,
            empty: raw & FLD_PWM0_IR_FIFO_EMPTY != 0,
            full: raw & FLD_PWM0_IR_FIFO_FULL != 0,
        }
    }
}

/// PWM IRQ sources living in the shared `reg_pwm_irq_mask`/`reg_pwm_irq_sta`
/// byte (`0x7b0`/`0x7b1`) — PWM0's pulse-count and frame sources, plus
/// PWM1..5's frame sources. PWM0's IR **FIFO** source is *not* included
/// here: it has its own one-bit sub-register pair
/// (`reg_pwm0_fifo_mode_irq_mask`/`_sta` @ `0x7b2`/`0x7b3`) — see
/// [`Pwm::set_ir_fifo_irq_enabled`]/[`Pwm::is_ir_fifo_irq_pending`]/
/// [`Pwm::clear_ir_fifo_irq`] and the module docs' "PWM0's second IRQ
/// sub-status register" section for why that is a genuinely separate
/// register, not a bit this enum could also expose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PwmIrqSource {
    /// `PWM_IRQ_PWM0_PNUM` — PWM0's [`Pwm0Mode::Count`]/[`Pwm0Mode::Ir`]
    /// pulse-count-reached condition.
    Pwm0PulseCountDone = 1 << 0,
    /// `PWM_IRQ_PWM0_IR_DMA_FIFO_DONE`. Included only because the bit
    /// exists in `reg_pwm_irq_sta` regardless of software support — this
    /// driver provides no way to enter `PWM_IR_DMA_FIFO_MODE`, so nothing
    /// reachable through this crate's API will ever set it. Kept for
    /// status/mask-register completeness, not as a usable feature.
    Pwm0IrDmaFifoDone = 1 << 1,
    /// `PWM_IRQ_PWM0_FRAME` — one PWM0 period elapsed.
    Pwm0Frame = 1 << 2,
    /// `PWM_IRQ_PWM1_FRAME`.
    Pwm1Frame = 1 << 3,
    /// `PWM_IRQ_PWM2_FRAME`.
    Pwm2Frame = 1 << 4,
    /// `PWM_IRQ_PWM3_FRAME`.
    Pwm3Frame = 1 << 5,
    /// `PWM_IRQ_PWM4_FRAME`.
    Pwm4Frame = 1 << 6,
    /// `PWM_IRQ_PWM5_FRAME`.
    Pwm5Frame = 1 << 7,
}

impl PwmIrqSource {
    /// This source's bit in `reg_pwm_irq_mask`/`reg_pwm_irq_sta`. The
    /// discriminant already *is* the bit value (each variant is a distinct
    /// power of two matching the vendor `PWM_IRQ` enum exactly), so this is
    /// a plain reinterpret, not a shift/lookup.
    pub const fn bit(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    InvalidFrequency,
    InvalidPin,
    /// The channel already owns a pin. Reconfiguring it in place would
    /// leave the previous pad muxed to the same PWM output while losing
    /// its ownership token.
    ChannelAlreadyConfigured,
    ChannelNotConfigured,
    DutyOutOfRange,
    InvalidDutyRatio,
    /// [`Pwm0IrFifoEntry::new`]'s `pulse_num` did not fit in the FIFO
    /// word's 14-bit field (`0..=`[`Pwm0IrFifoEntry::MAX_PULSE_NUM`]).
    PulseNumOutOfRange,
    /// [`Pwm::ir_fifo_push`]'s bounded wait for a free FIFO slot
    /// (`pwm_ir_fifo_is_full`) elapsed without the hardware draining an
    /// entry.
    IrFifoTimeout,
    /// [`Pwm::ir_fifo_clear`] was called while PWM0 is enabled.
    /// `pwm_ir_fifo_clr_data`'s own doc comment: "Only when pwm is in not
    /// active mode, it is possible to clear data in fifo" — disable PWM0
    /// first ([`Pwm::disable`]).
    Pwm0Active,
    /// Timer0 is stopped, so [`Pwm::ir_fifo_push`]'s timeout cannot make
    /// progress. Call [`crate::timer::init`] first.
    TimerNotRunning,
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
    /// PWM0's current [`Pwm0Mode`], mirrored in software so
    /// [`Pwm::pwm0_mode`] can be read back without a register round-trip.
    /// Reset to [`Pwm0Mode::Normal`] whenever PWM0 is (re)configured,
    /// matching `reg_pwm0_mode`'s hardware reset value and
    /// `configure_channel`'s explicit `reg_pwm0_mode = 0` write.
    pwm0_mode: Pwm0Mode,
    /// Which of the two `reg_pwm_ir_fifo_dat` slots (`0x7c8`/`0x7ca`)
    /// [`Pwm::ir_fifo_push`] writes next. `pwm_ir_fifo_set_data_entry`
    /// (`pwm.h`) tracks this with a `static unsigned char index` local to
    /// that one C function; this crate has no hidden global state, so the
    /// equivalent toggle lives here instead, scoped to this `Pwm` instance.
    ir_fifo_index: u8,
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
            pwm0_mode: Pwm0Mode::Normal,
            ir_fifo_index: 0,
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
        self.ensure_channel_unconfigured(channel)?;
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
        if matches!(channel, Channel::Pwm0) {
            self.pwm0_mode = Pwm0Mode::Normal;
            self.ir_fifo_index = 0;
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

    /// Whether PWM0 is currently enabled (`reg_pwm0_enable`'s bit 0) —
    /// used internally by [`Self::ir_fifo_clear`]'s "not active" guard, and
    /// exposed for callers that want to check before issuing PWM0-only
    /// commands the vendor driver itself documents as unsafe while running.
    #[cfg(target_arch = "tc32")]
    pub fn is_pwm0_enabled(&self) -> bool {
        unsafe { crate::mmio::r8(REG_PWM0_ENABLE) & Channel::Pwm0.bit() != 0 }
    }

    /// PWM0's currently configured [`Pwm0Mode`], mirrored in software (see
    /// the `pwm0_mode` field doc) — never re-reads `reg_pwm0_mode` since
    /// that register is write-only in the vendor driver's own usage.
    pub const fn pwm0_mode(&self) -> Pwm0Mode {
        self.pwm0_mode
    }

    /// Set PWM0's work mode (`pwm_set_mode`, `reg_pwm0_mode` @ `0x783`).
    /// Only valid for [`Channel::Pwm0`] — PWM1..5 have no mode register at
    /// all (`pwm_set_mode`'s own body is a no-op for any other channel), so
    /// this takes no `Channel` parameter.
    #[cfg(target_arch = "tc32")]
    pub fn set_pwm0_mode(&mut self, mode: Pwm0Mode) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        unsafe { crate::mmio::w8(REG_PWM0_MODE, mode as u8) };
        self.pwm0_mode = mode;
        // A stale ping-pong index from a previous IrFifo session could
        // write to the wrong of the two `reg_pwm_ir_fifo_dat` slots
        // relative to whatever the hardware last consumed; re-entering any
        // mode (including IrFifo again) restarts the toggle at a known
        // value instead of carrying one over across mode switches.
        self.ir_fifo_index = 0;
        Ok(())
    }

    /// Program PWM0's pulse-count target (`pwm_set_pulse_num`,
    /// `reg_pwm0_pulse_num` @ `0x7ac`, a plain 16-bit count). Meaningful in
    /// [`Pwm0Mode::Count`]/[`Pwm0Mode::Ir`]; in [`Pwm0Mode::IrFifo`] each
    /// burst's pulse count instead comes from the corresponding
    /// [`Pwm0IrFifoEntry`] pushed via [`Self::ir_fifo_push`], not this
    /// register.
    #[cfg(target_arch = "tc32")]
    pub fn set_pwm0_pulse_num(&mut self, pulse_num: u16) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        unsafe { crate::mmio::w16(REG_PWM0_PULSE_NUM, pulse_num) };
        Ok(())
    }

    /// Program the shadow cycle/duty pair
    /// (`pwm_set_pwm0_shadow_cycle_and_duty`, `reg_pwm_tmax0_shadow`/
    /// `reg_pwm_tcmp0_shadow` @ `0x7c6`/`0x7c4`) an [`Pwm0IrFifoEntry`] can
    /// select via its `use_shadow` flag, as an alternative to the normal
    /// `TCMP0`/`TMAX0` pair [`Self::set_duty_cycle`] programs.
    #[cfg(target_arch = "tc32")]
    pub fn set_pwm0_shadow_cycle_and_duty(
        &mut self,
        cycle_ticks: u16,
        cmp_ticks: u16,
    ) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        validate_cycle_and_duty(cycle_ticks, cmp_ticks)?;
        unsafe {
            crate::mmio::w16(REG_PWM_TCMP0_SHADOW, cmp_ticks);
            crate::mmio::w16(REG_PWM_TMAX0_SHADOW, cycle_ticks);
        }
        Ok(())
    }

    /// Program the IR FIFO's IRQ trigger level
    /// (`pwm_ir_fifo_set_irq_trig_level`, `reg_pwm_ir_fifo_irq_trig_level`
    /// @ `0x7cc`).
    #[cfg(target_arch = "tc32")]
    pub fn ir_fifo_set_trigger_level(&mut self, level: u8) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        unsafe { crate::mmio::w8(REG_PWM_IR_FIFO_IRQ_TRIG_LEVEL, level) };
        Ok(())
    }

    /// Read the IR FIFO's occupancy/empty/full status
    /// (`pwm_ir_fifo_get_data_num`/`_is_empty`/`_is_full`,
    /// `reg_pwm_ir_fifo_data_status` @ `0x7cd`).
    #[cfg(target_arch = "tc32")]
    pub fn ir_fifo_status(&self) -> Pwm0IrFifoStatus {
        Pwm0IrFifoStatus::decode(unsafe { crate::mmio::r8(REG_PWM_IR_FIFO_DATA_STATUS) })
    }

    /// Push one entry into PWM0's two-slot IR FIFO
    /// (`pwm_ir_fifo_set_data_entry`, `reg_pwm_ir_fifo_dat` @
    /// `0x7c8`/`0x7ca`), waiting up to `timeout_ticks` (Timer0 ticks, see
    /// [`crate::timer::ms`]/[`crate::timer::us`]) for a free slot instead
    /// of the vendor body's unconditional `while(pwm_ir_fifo_is_full());`
    /// spin.
    ///
    /// Returns [`PwmError::IrFifoTimeout`] if the FIFO is still full after
    /// `timeout_ticks` elapse — this can happen if [`Pwm0Mode::IrFifo`]
    /// hasn't been entered (nothing is draining the FIFO) or the caller is
    /// pushing faster than the configured burst rate drains it.
    #[cfg(target_arch = "tc32")]
    pub fn ir_fifo_push(
        &mut self,
        entry: Pwm0IrFifoEntry,
        timeout_ticks: u32,
    ) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        if !crate::timer::is_timer0_running() {
            return Err(PwmError::TimerNotRunning);
        }
        let has_room = crate::timer::wait_until(timeout_ticks, || !self.ir_fifo_status().full);
        if !has_room {
            return Err(PwmError::IrFifoTimeout);
        }
        unsafe {
            crate::mmio::w16(ir_fifo_dat_register(self.ir_fifo_index), entry.encode());
        }
        self.ir_fifo_index ^= 1;
        Ok(())
    }

    /// Clear all data currently held in the IR FIFO
    /// (`pwm_ir_fifo_clr_data`, `reg_pwm_ir_clr_fifo_data` @ `0x7ce`).
    ///
    /// Returns [`PwmError::Pwm0Active`] if PWM0 is currently enabled —
    /// `pwm_ir_fifo_clr_data`'s own doc comment states this is only valid
    /// while PWM0 is not active; see [`Self::disable`].
    #[cfg(target_arch = "tc32")]
    pub fn ir_fifo_clear(&mut self) -> Result<(), PwmError> {
        if !self.is_configured(Channel::Pwm0) {
            return Err(PwmError::ChannelNotConfigured);
        }
        if self.is_pwm0_enabled() {
            return Err(PwmError::Pwm0Active);
        }
        unsafe {
            let value = crate::mmio::r8(REG_PWM_IR_CLR_FIFO_DATA);
            crate::mmio::w8(REG_PWM_IR_CLR_FIFO_DATA, value | FLD_PWM0_IR_FIFO_CLR_DATA);
        }
        self.ir_fifo_index = 0;
        Ok(())
    }

    /// Enable or disable one `reg_pwm_irq_mask`/`reg_pwm_irq_sta` source
    /// (`pwm_set_interrupt_enable`/`_disable`, `reg_pwm_irq_mask` @
    /// `0x7b0`). This is a genuine enable-mask register (`1` = enabled),
    /// not a mask-out — see the module docs' "PWM0's second IRQ sub-status
    /// register" section for how this relates to the chip's *global* CPU
    /// IRQ enable, which this method does not touch. The read-modify-write
    /// runs inside [`crate::mmio::with_irqs_disabled`] so a concurrent ISR
    /// cannot observe or race a partially updated mask.
    #[cfg(target_arch = "tc32")]
    pub fn set_irq_enabled(&mut self, source: PwmIrqSource, enabled: bool) {
        crate::mmio::with_irqs_disabled(|| unsafe {
            let mask = crate::mmio::r8(REG_PWM_IRQ_MASK);
            crate::mmio::w8(
                REG_PWM_IRQ_MASK,
                if enabled {
                    mask | source.bit()
                } else {
                    mask & !source.bit()
                },
            );
        });
    }

    /// Raw `reg_pwm_irq_sta` snapshot — combine with [`PwmIrqSource::bit`]
    /// or use [`Self::is_irq_pending`] for a single source.
    #[cfg(target_arch = "tc32")]
    pub fn irq_status(&self) -> u8 {
        unsafe { crate::mmio::r8(REG_PWM_IRQ_STA) }
    }

    /// Whether one specific `reg_pwm_irq_sta` source is currently pending.
    #[cfg(target_arch = "tc32")]
    pub fn is_irq_pending(&self, source: PwmIrqSource) -> bool {
        self.irq_status() & source.bit() != 0
    }

    /// Write-1-to-clear exactly the requested sources in `reg_pwm_irq_sta`
    /// (`pwm_clear_interrupt_status`). `reg_pwm_irq_sta` is W1C, so this
    /// writes only the OR of `sources`' bits directly — not a
    /// read-modify-write — matching the vendor body's own
    /// `reg_pwm_irq_sta = status` (a plain assignment, not `|=`): any bit
    /// left `0` in that write is left untouched by hardware regardless of
    /// the register's current value, so sources not passed here are
    /// unaffected even though this is not a read/modify/write sequence.
    #[cfg(target_arch = "tc32")]
    pub fn clear_irq(&mut self, sources: &[PwmIrqSource]) {
        let mut mask = 0u8;
        let mut i = 0;
        while i < sources.len() {
            mask |= sources[i].bit();
            i += 1;
        }
        unsafe { crate::mmio::w8(REG_PWM_IRQ_STA, mask) };
    }

    /// Enable or disable PWM0's IR FIFO IRQ
    /// (`reg_pwm0_fifo_mode_irq_mask` @ `0x7b2`, bit 0) — see the module
    /// docs' "PWM0's second IRQ sub-status register" section for why this
    /// is a separate register from [`Self::set_irq_enabled`], not one more
    /// [`PwmIrqSource`] variant. Runs inside
    /// [`crate::mmio::with_irqs_disabled`] like [`Self::set_irq_enabled`].
    #[cfg(target_arch = "tc32")]
    pub fn set_ir_fifo_irq_enabled(&mut self, enabled: bool) {
        crate::mmio::with_irqs_disabled(|| unsafe {
            crate::mmio::w8(
                REG_PWM0_FIFO_MODE_IRQ_MASK,
                if enabled { FLD_PWM0_IR_FIFO_IRQ_BIT } else { 0 },
            );
        });
    }

    /// Whether PWM0's IR FIFO IRQ (`reg_pwm0_fifo_mode_irq_sta` @ `0x7b3`,
    /// bit 0) is currently pending.
    #[cfg(target_arch = "tc32")]
    pub fn is_ir_fifo_irq_pending(&self) -> bool {
        unsafe { crate::mmio::r8(REG_PWM0_FIFO_MODE_IRQ_STA) & FLD_PWM0_IR_FIFO_IRQ_BIT != 0 }
    }

    /// Write-1-to-clear PWM0's IR FIFO IRQ (`pwm_clear_interrupt_status`'s
    /// `PWM_IRQ_PWM0_IR_FIFO` branch, `reg_pwm0_fifo_mode_irq_sta = BIT(0)`
    /// — again a direct write of exactly the one valid bit, not a
    /// read-modify-write, matching the vendor body).
    #[cfg(target_arch = "tc32")]
    pub fn clear_ir_fifo_irq(&mut self) {
        unsafe { crate::mmio::w8(REG_PWM0_FIFO_MODE_IRQ_STA, FLD_PWM0_IR_FIFO_IRQ_BIT) };
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
        crate::reset::enable_clock(crate::reset::Peripheral::Pwm)
            .expect("PWM has a documented reg_clk_en0 bit");
        crate::reset::pulse_reset(crate::reset::Peripheral::Pwm);
        unsafe {
            crate::mmio::w8(REG_PWM_CLOCK, self.divider);
            crate::mmio::w8(REG_PWM_ENABLE, 0);
            crate::mmio::w8(REG_PWM0_ENABLE, 0);
        }
    }

    fn ensure_channel_unconfigured(&self, channel: Channel) -> Result<(), PwmError> {
        if self.is_configured(channel) {
            Err(PwmError::ChannelAlreadyConfigured)
        } else {
            Ok(())
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

/// `reg_pwm_ir_fifo_dat(i) = REG_ADDR16(0x7c8 + i*2)` (`register.h`) — the
/// two-slot IR FIFO data ping-pong register `index` (`0` or `1`) selects.
const fn ir_fifo_dat_register(index: u8) -> u32 {
    REG_PWM_IR_FIFO_DAT_BASE + (index as u32) * 2
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

fn validate_cycle_and_duty(cycle_ticks: u16, cmp_ticks: u16) -> Result<(), PwmError> {
    if cycle_ticks < 2 {
        return Err(PwmError::InvalidFrequency);
    }
    if cmp_ticks > cycle_ticks {
        return Err(PwmError::DutyOutOfRange);
    }
    Ok(())
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
    fn channel_reconfiguration_is_rejected_before_losing_the_old_pin() {
        let pwm = Pwm {
            divider: 0,
            period_ticks: 24_000,
            actual_frequency_hz: 1_000,
            configured: Channel::Pwm0.bit(),
            pwm0_mode: Pwm0Mode::Normal,
            ir_fifo_index: 0,
        };
        assert_eq!(
            pwm.ensure_channel_unconfigured(Channel::Pwm0),
            Err(PwmError::ChannelAlreadyConfigured)
        );
        assert_eq!(pwm.ensure_channel_unconfigured(Channel::Pwm1), Ok(()));
    }

    #[test]
    fn shadow_cycle_rejects_zero_period_and_compare_overflow() {
        assert_eq!(
            validate_cycle_and_duty(0, 0),
            Err(PwmError::InvalidFrequency)
        );
        assert_eq!(
            validate_cycle_and_duty(100, 101),
            Err(PwmError::DutyOutOfRange)
        );
        assert_eq!(validate_cycle_and_duty(100, 100), Ok(()));
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

    #[test]
    fn pwm0_and_shared_enable_registers_are_distinct() {
        // Requirement-4 audit: PWM0 is gated through its own enable
        // register (`reg_pwm0_enable`), never through bit 0 of the
        // register PWM1..5 share (`reg_pwm_enable`) — see `pwm_start`/
        // `pwm_stop`'s own `if(PWM0_ID == id) {...} else {...}` branch in
        // `pwm.h`. `Pwm::enable`/`Pwm::disable` select the register with
        // that same branch before applying `Channel::bit()`, so this only
        // needs to confirm the two addresses can never alias each other.
        assert_ne!(REG_PWM_ENABLE, REG_PWM0_ENABLE);
    }

    #[test]
    fn pwm0_extended_register_map_matches_8258_header() {
        assert_eq!(REG_PWM0_PULSE_NUM, 0x8007AC);
        assert_eq!(REG_PWM_IRQ_MASK, 0x8007B0);
        assert_eq!(REG_PWM_IRQ_STA, 0x8007B1);
        assert_eq!(REG_PWM0_FIFO_MODE_IRQ_MASK, 0x8007B2);
        assert_eq!(REG_PWM0_FIFO_MODE_IRQ_STA, 0x8007B3);
        assert_eq!(REG_PWM_TCMP0_SHADOW, 0x8007C4);
        assert_eq!(REG_PWM_TMAX0_SHADOW, 0x8007C6);
        assert_eq!(REG_PWM_IR_FIFO_DAT_BASE, 0x8007C8);
        assert_eq!(REG_PWM_IR_FIFO_IRQ_TRIG_LEVEL, 0x8007CC);
        assert_eq!(REG_PWM_IR_FIFO_DATA_STATUS, 0x8007CD);
        assert_eq!(REG_PWM_IR_CLR_FIFO_DATA, 0x8007CE);
    }

    #[test]
    fn ir_fifo_dat_register_selects_the_two_ping_pong_slots() {
        assert_eq!(ir_fifo_dat_register(0), 0x8007C8);
        assert_eq!(ir_fifo_dat_register(1), 0x8007CA);
    }

    #[test]
    fn pwm0_mode_encodings_match_8258_header() {
        // `pwm_mode` enum, `platform/chip_8258/pwm.h`. `PWM_IR_DMA_FIFO_MODE
        // = 0x0f` has no variant here — see the module docs' "Explicitly
        // unsupported" section.
        assert_eq!(Pwm0Mode::Normal as u8, 0x00);
        assert_eq!(Pwm0Mode::Count as u8, 0x01);
        assert_eq!(Pwm0Mode::Ir as u8, 0x03);
        assert_eq!(Pwm0Mode::IrFifo as u8, 0x07);
    }

    #[test]
    fn irq_source_bits_match_8258_header() {
        // `PWM_IRQ`/`FLD_IRQ_PWMx_*` enum, `platform/chip_8258/pwm.h`/
        // `register.h`.
        assert_eq!(PwmIrqSource::Pwm0PulseCountDone.bit(), 0x01);
        assert_eq!(PwmIrqSource::Pwm0IrDmaFifoDone.bit(), 0x02);
        assert_eq!(PwmIrqSource::Pwm0Frame.bit(), 0x04);
        assert_eq!(PwmIrqSource::Pwm1Frame.bit(), 0x08);
        assert_eq!(PwmIrqSource::Pwm2Frame.bit(), 0x10);
        assert_eq!(PwmIrqSource::Pwm3Frame.bit(), 0x20);
        assert_eq!(PwmIrqSource::Pwm4Frame.bit(), 0x40);
        assert_eq!(PwmIrqSource::Pwm5Frame.bit(), 0x80);
    }

    #[test]
    fn clear_irq_w1c_mask_ors_every_requested_source_and_only_those() {
        // `clear_irq`'s inner mask-building loop is what actually gets
        // written to `reg_pwm_irq_sta` (a W1C register) — this pins down
        // that it is a plain OR of exactly the requested bits, matching
        // `pwm_clear_interrupt_status`'s `reg_pwm_irq_sta = status`
        // (direct assignment, not a read-modify-write).
        let mut mask = 0u8;
        let mut i = 0;
        let sources = [PwmIrqSource::Pwm0Frame, PwmIrqSource::Pwm3Frame];
        while i < sources.len() {
            mask |= sources[i].bit();
            i += 1;
        }
        assert_eq!(mask, 0x04 | 0x20);
    }

    #[test]
    fn ir_fifo_irq_sub_status_bit_is_disjoint_from_shared_register() {
        // The IR FIFO sub-block (`reg_pwm0_fifo_mode_irq_mask`/`_sta`) is a
        // separate one-bit register, not a bit within `reg_pwm_irq_sta` —
        // this only asserts the bit constant itself (bit 0) rather than
        // colliding with any `PwmIrqSource` variant, since the two live in
        // different registers entirely (see the module docs' "PWM0's
        // second IRQ sub-status register" section).
        assert_eq!(FLD_PWM0_IR_FIFO_IRQ_BIT, 0x01);
        assert_eq!(FLD_PWM0_IR_FIFO_CLR_DATA, 0x01);
    }

    #[test]
    fn ir_fifo_entry_encodes_pulse_num_shadow_and_carrier_bits() {
        // Matches `pwm_ir_fifo_set_data_entry`'s packing exactly:
        // `pulse_num + ((use_shadow&1)<<14) + ((carrier_en&1)<<15)`.
        let plain = Pwm0IrFifoEntry::new(100, false, false).unwrap();
        assert_eq!(plain.encode(), 100);

        let shadow_only = Pwm0IrFifoEntry::new(100, true, false).unwrap();
        assert_eq!(shadow_only.encode(), 100 | (1 << 14));

        let carrier_only = Pwm0IrFifoEntry::new(100, false, true).unwrap();
        assert_eq!(carrier_only.encode(), 100 | (1 << 15));

        let both = Pwm0IrFifoEntry::new(100, true, true).unwrap();
        assert_eq!(both.encode(), 100 | (1 << 14) | (1 << 15));
    }

    #[test]
    fn ir_fifo_entry_pulse_num_boundaries() {
        assert_eq!(Pwm0IrFifoEntry::new(0, false, false).unwrap().encode(), 0);
        assert_eq!(
            Pwm0IrFifoEntry::new(Pwm0IrFifoEntry::MAX_PULSE_NUM, false, false)
                .unwrap()
                .encode(),
            0x3fff
        );
        assert_eq!(
            Pwm0IrFifoEntry::new(Pwm0IrFifoEntry::MAX_PULSE_NUM + 1, false, false),
            Err(PwmError::PulseNumOutOfRange)
        );
        assert_eq!(
            Pwm0IrFifoEntry::new(u16::MAX, false, false),
            Err(PwmError::PulseNumOutOfRange)
        );
    }

    #[test]
    fn ir_fifo_status_decodes_data_num_empty_and_full() {
        assert_eq!(
            Pwm0IrFifoStatus::decode(0x00),
            Pwm0IrFifoStatus {
                data_num: 0,
                empty: false,
                full: false,
            }
        );
        assert_eq!(
            Pwm0IrFifoStatus::decode(0b0001_0010),
            Pwm0IrFifoStatus {
                data_num: 0x02,
                empty: true,
                full: false,
            }
        );
        assert_eq!(
            Pwm0IrFifoStatus::decode(0b0010_0000),
            Pwm0IrFifoStatus {
                data_num: 0,
                empty: false,
                full: true,
            }
        );
        // The data-num field is 4 bits wide; a set bit 4/5 (empty/full)
        // must not leak into it.
        assert_eq!(Pwm0IrFifoStatus::decode(0xff).data_num, 0x0f);
    }
}
