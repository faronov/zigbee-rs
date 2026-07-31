//! TLSR8258 software MCU reset (immediate, not the watchdog-timeout path).
//!
//! # Register evidence
//!
//! `platform/chip_8258/register.h`:
//! ```c
//! #define reg_pwdn_ctrl   REG_ADDR8(0x6f)
//! enum{
//!     FLD_PWDN_CTRL_REBOOT = BIT(5),
//!     FLD_PWDN_CTRL_SLEEP  = BIT(7),
//! };
//! ```
//! `platform/chip_8258/bsp.h`'s own `static inline` function (open C source,
//! not a compiled/disassembled object — full confidence, the same tier as
//! `gpio.rs`'s per-pin registers):
//! ```c
//! static inline void mcu_reset(void)
//! {
//!     write_reg8(0x06f,0x20);
//! }
//! ```
//! `0x20 == BIT(5) == FLD_PWDN_CTRL_REBOOT`, confirming the bit name. Note
//! that the vendor's own `mcu_reset()` writes the byte outright rather than
//! doing a read-modify-write (it does not preserve `FLD_PWDN_CTRL_SLEEP` or
//! any other bit that might be set in `reg_pwdn_ctrl`); [`reboot`] below
//! reproduces that exact behaviour rather than inventing a
//! read-modify-write the vendor's own reference implementation does not
//! perform.
//!
//! # Relationship to `watchdog.rs`
//!
//! `watchdog.rs` already provides an MCU reset *mechanism* (arm a short
//! timeout, stop feeding it), but that is a bounded, delayed, deliberately
//! non-immediate reset used for hang recovery. This module instead
//! reproduces the vendor's separate, immediate, unconditional software
//! reboot register — useful for cases like an explicit application-level
//! "reboot now" command (e.g. servicing a Zigbee Basic-cluster reset
//! request or completing an OTA image swap) where waiting out a watchdog
//! window is not appropriate. The two modules intentionally do not share
//! code: they use different registers for different purposes.
//!
//! # Why this never returns a `Result`/error
//!
//! There is nothing to validate: the operation is a single fixed byte
//! write with no caller-supplied parameters, and the hardware is expected
//! to reset before program flow could observe any distinguishable failure.
//! [`reboot`] is marked `-> !`, so it *cannot* return control to its caller
//! even if the reset is delayed or never happens; the trailing spin is
//! therefore an intentionally **unbounded**, fail-closed terminal loop, not
//! a bounded wait with a timeout. This is a deliberate exception to this
//! crate's usual "no infinite waits" rule: it exists specifically to
//! prevent falling through to execute arbitrary unrelated code after a
//! reset has been requested, and it is only reachable by a caller that has
//! already committed to `reboot`'s `-> !` contract (there is no way to
//! recover from this function once called, by construction). On a
//! non-`tc32` host build there is no register to write and no reset to
//! wait for, so this module exposes no callable function there (mirroring
//! `clocks::init`'s and `watchdog`'s own `#[cfg(target_arch = "tc32")]`
//! gating pattern).

#[cfg(target_arch = "tc32")]
use super::mmio::{REG_PWDN_CTRL, w8};

/// `FLD_PWDN_CTRL_REBOOT` (`register.h`) — writing this bit to
/// `reg_pwdn_ctrl` triggers an immediate MCU reset.
pub const FLD_PWDN_CTRL_REBOOT: u8 = 1 << 5;

/// Immediately reset the MCU (software reboot), matching
/// `platform/chip_8258/bsp.h`'s `mcu_reset()` exactly (a plain byte write,
/// not a read-modify-write — see module docs for why).
///
/// Never returns: the hardware reset is expected to take effect before
/// this function's caller resumes. The trailing spin is an intentionally
/// **unbounded** fail-closed terminal loop (not a bounded/timed wait) — see
/// the module-level "Why this never returns a `Result`/error" section for
/// why that is the correct and only sound behaviour for a `-> !` function
/// whose entire purpose is "the CPU must not execute anything else after
/// this point".
#[cfg(target_arch = "tc32")]
pub fn reboot() -> ! {
    unsafe { w8(REG_PWDN_CTRL, FLD_PWDN_CTRL_REBOOT) };
    // Intentionally unbounded fail-closed terminal spin, not a bounded
    // wait: this function is `-> !` and therefore must never return
    // control to its caller, whether or not the reset takes effect
    // immediately. Do not "fix" this into a bounded loop with a timeout —
    // there is no safe fallback action to take if the reset does not
    // occur, so refusing to proceed is the correct fail-closed behaviour.
    loop {
        core::hint::spin_loop();
    }
}

/// Typed per-peripheral reset-pulse and clock-gate control
/// (`reg_rst0`/`reg_rst1` and `reg_clk_en0`/`reg_clk_en1`,
/// `platform/chip_8258/register.h`'s "reset registers: 0x60" and
/// "0x63" blocks).
///
/// Before this facade existed, `i2c.rs`, `spi.rs`, `uart.rs`, and `pwm.rs`
/// each hand-rolled an identical "OR the clock-enable bit in, then pulse
/// the matching reset bit" sequence against these same shared registers
/// (see those modules' own `configure_peripheral`/`enable_peripheral`
/// functions, now migrated to call [`pulse_reset`]/[`enable_clock`]
/// instead of repeating the read-modify-write). [`Peripheral`] enumerates
/// every bit this crate's drivers or a future one plausibly need; see
/// "Scope" below for what is deliberately left out.
///
/// # Register evidence
///
/// `platform/chip_8258/register.h` (open, shipped-as-C-source, not a
/// compiled object — full confidence):
/// ```c
/// #define reg_rst0    REG_ADDR8(0x60)
/// enum{
///     FLD_RST0_SPI   = BIT(0),
///     FLD_RST0_I2C   = BIT(1),
///     FLD_RST0_UART  = BIT(2),
///     FLD_RST0_USB   = BIT(3),   // out of scope, see below
///     FLD_RST0_PWM   = BIT(4),
///     FLD_RST0_QDEC  = BIT(5),   // out of scope, see below
///     FLD_RST0_SWIRE = BIT(7),   // out of scope, see below
/// };
/// #define reg_rst1    REG_ADDR8(0x61)
/// enum{
///     FLD_RST1_ZB         = BIT(0),
///     FLD_RST1_SYS_TIMER  = BIT(1),
///     FLD_RST1_DMA        = BIT(2),
///     FLD_RST1_ALGM       = BIT(3),  // out of scope, see below
///     FLD_RST1_AES        = BIT(4),
///     FLD_RST1_ADC        = BIT(5),
///     FLD_RST1_ALG        = BIT(6),  // out of scope, see below
/// };
/// #define reg_rst2    REG_ADDR8(0x62)  // entirely out of scope, see below
///
/// #define reg_clk_en0 REG_ADDR8(0x63)
/// enum{
///     FLD_CLK0_SPI_EN   = BIT(0),
///     FLD_CLK0_I2C_EN   = BIT(1),
///     FLD_CLK0_UART_EN  = BIT(2),
///     FLD_CLK0_USB_EN   = BIT(3),   // out of scope
///     FLD_CLK0_PWM_EN   = BIT(4),
///     FLD_CLK0_QDEC_EN  = BIT(5),   // out of scope
///     FLD_CLK0_SWIRE_EN = BIT(7),   // out of scope
/// };
/// #define reg_clk_en1 REG_ADDR8(0x64)
/// enum{
///     FLD_CLK1_ZB_EN         = BIT(0),
///     FLD_CLK1_SYS_TIMER_EN  = BIT(1),
///     FLD_CLK1_DMA_EN        = BIT(2),
///     FLD_CLK1_ALGM_EN       = BIT(3),  // out of scope
///     FLD_CLK1_AES_EN        = BIT(4),
///     // no ADC bit here — see Peripheral::Adc's own doc.
/// };
/// #define reg_clk_en2 REG_ADDR8(0x65)   // entirely out of scope, see below
/// ```
/// Cross-checked bit-for-bit against a second, independently maintained
/// mirror of the same vendor header (`pvvx/BZdevice`'s
/// `SDK/platform/chip_8258/register.h`); both agree on every bit above,
/// including `reg_clk_en1`'s missing ADC bit.
///
/// # Scope: which peripherals this enum covers, and why
///
/// [`Peripheral`] covers every `reg_rst0`/`reg_rst1`/`reg_clk_en0`/
/// `reg_clk_en1` bit this crate's drivers or the Zigbee/IoT stack around
/// them plausibly need: [`Peripheral::Spi`], [`Peripheral::I2c`],
/// [`Peripheral::Uart`], [`Peripheral::Pwm`] (all driven by `spi.rs`/
/// `i2c.rs`/`uart.rs`/`pwm.rs`, migrated to this facade), [`Peripheral::Zb`]
/// (the RF/Zigbee baseband — not yet migrated here; see
/// `radio::phy::rf_phy_init_zigbee`'s own doc for why its bulk
/// `0xFF`/`0x00` bring-up sequence stays specialized), [`Peripheral::Dma`],
/// [`Peripheral::SysTimer`] (the free-running system-tick block, distinct
/// from [`crate::timer`]'s `reg_tmr_ctrl`-gated Timer0/1/2 — see below),
/// and [`Peripheral::Aes`]/[`Peripheral::Adc`] (used by `aes.rs` and
/// `adc.rs`; ADC has a reset bit but no documented software clock gate).
///
/// Deliberately **not** modeled, in line with this crate's "typed APIs
/// only for evidenced, in-scope register semantics" convention:
/// - **`reg_rst2`/`reg_clk_en2` in their entirety** (`FLD_RST2_AIF`/`_AUD`/
///   `_DFIFO`/`_RISC`/`_MCIC`/`_RISC1`/`_MCIC1` and their `reg_clk_en2`
///   counterparts) — these gate the audio interface, audio codec, data
///   FIFO, and the second RISC core/its cache, none of which are
///   multimedia/debug blocks a Zigbee/IoT stack drives.
/// - **`FLD_RST0_USB`/`FLD_CLK0_USB_EN`, `FLD_RST0_QDEC`/`FLD_CLK0_QDEC_EN`,
///   `FLD_RST0_SWIRE`/`FLD_CLK0_SWIRE_EN`** — USB, the quadrature decoder,
///   and the single-wire debug/programming interface are not part of this
///   stack's Zigbee/IoT surface (SWIRE in particular is explicitly a debug
///   interface).
/// - **`FLD_RST1_ALGM`/`FLD_CLK1_ALGM_EN` and `FLD_RST1_ALG`** — the
///   vendor header names these but no open source in the SDK (`bsp.h`,
///   any `platform/chip_8258/*.h` driver) documents what "ALG"/"ALGM"
///   drive, so — same reasoning as [`crate::irq`]'s omission of
///   `reg_irq_pri` — this module does not guess.
///
/// # `Peripheral::Adc` has no clock-gate bit
///
/// `reg_clk_en1` (unlike `reg_rst1`, which *does* have `FLD_RST1_ADC`)
/// has no documented ADC clock-enable bit — confirmed by grepping the
/// full `reg_clk_en1` enum in two independently maintained copies of
/// `register.h` (see above): it lists exactly `ZB`/`SYS_TIMER`/`DMA`/
/// `ALGM`/`AES`, nothing else. [`Peripheral::clock_bit`] returns `None`
/// for `Adc` accordingly, and [`enable_clock`]/[`disable_clock`]/
/// [`is_clock_enabled`] return [`ClockError::NotDocumented`] rather than
/// silently treating a missing bit as "always enabled" or writing to an
/// address with no defined meaning. `adc.rs` reproduces the vendor's ADC
/// reset and power-up
/// path via the analog register bus, not a digital clock-gate bit.
///
/// # `Peripheral::SysTimer` vs. `crate::timer`'s Timer0/1/2
///
/// `FLD_RST1_SYS_TIMER`/`FLD_CLK1_SYS_TIMER_EN` gate the free-running
/// `reg_system_tick` block ([`crate::mmio::REG_SYSTEM_TICK`]), which is a
/// different piece of hardware from the `reg_tmr_ctrl`-gated Timer0/
/// Timer1/Timer2 (SYS_CLK-mode tick counters) [`crate::timer`] and
/// [`crate::watchdog`] already fully own via that register's own
/// `FLD_TMR0_EN`/`FLD_TMR1_EN`/`FLD_TMR2_EN` bits — there is no
/// `reg_rst0`/`reg_rst1`/`reg_clk_en0`/`reg_clk_en1` bit for Timer0/1/2
/// specifically (they are gated only by `reg_tmr_ctrl`'s own enable bits,
/// which `timer.rs`/`watchdog.rs` already manage). Do not confuse
/// [`Peripheral::SysTimer`] with "Timer0/1/2 support" — it is a distinct,
/// separately clocked/reset counter this crate does not otherwise expose.
///
/// # Relationship to `clocks::init`
///
/// [`crate::clocks::init`] runs once, very early in boot, from
/// `.ram_code`, and unconditionally sets `reg_clk_en0`/`reg_clk_en1`/
/// `reg_clk_en2` to `0xFF` (every documented and undocumented bit on) as
/// part of a fixed analog/PLL bring-up sequence that must execute in
/// exactly that order before the flash cache/XIP path is known-stable —
/// it is not expressed in terms of this facade, and should not be: at the
/// point it runs, no peripheral driver has initialized anything yet, so
/// there is no "gate peripheral X" decision to make, only "the whole
/// clock tree must come up." This module instead provides *runtime*,
/// per-peripheral gating for code that runs after `clocks::init` has
/// already completed — narrowing (or re-pulsing the reset of) one
/// already-running peripheral's clock, which is what every migrated
/// driver's own bring-up (and any future power-management policy that
/// wants to gate an idle peripheral's clock) actually needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Peripheral {
    /// `FLD_RST0_SPI` / `FLD_CLK0_SPI_EN`, bit 0 of `reg_rst0`/`reg_clk_en0`.
    Spi,
    /// `FLD_RST0_I2C` / `FLD_CLK0_I2C_EN`, bit 1.
    I2c,
    /// `FLD_RST0_UART` / `FLD_CLK0_UART_EN`, bit 2.
    Uart,
    /// `FLD_RST0_PWM` / `FLD_CLK0_PWM_EN`, bit 4.
    Pwm,
    /// `FLD_RST1_ZB` / `FLD_CLK1_ZB_EN`, bit 0 of `reg_rst1`/`reg_clk_en1`
    /// — the RF/Zigbee baseband.
    Zb,
    /// `FLD_RST1_SYS_TIMER` / `FLD_CLK1_SYS_TIMER_EN`, bit 1. See this
    /// enum's own doc section on why this is not "Timer0/1/2".
    SysTimer,
    /// `FLD_RST1_DMA` / `FLD_CLK1_DMA_EN`, bit 2.
    Dma,
    /// `FLD_RST1_AES` / `FLD_CLK1_AES_EN`, bit 4. Used by `aes.rs`.
    Aes,
    /// `FLD_RST1_ADC`, bit 5 of `reg_rst1`. See this enum's own doc
    /// section on why [`Peripheral::clock_bit`] is `None` for this
    /// variant. Used by `adc.rs`.
    Adc,
}

impl Peripheral {
    /// This peripheral's reset register (`reg_rst0` or `reg_rst1`).
    pub(crate) const fn reset_register(self) -> u32 {
        match self {
            Peripheral::Spi | Peripheral::I2c | Peripheral::Uart | Peripheral::Pwm => {
                super::mmio::REG_RST0
            }
            Peripheral::Zb
            | Peripheral::SysTimer
            | Peripheral::Dma
            | Peripheral::Aes
            | Peripheral::Adc => super::mmio::REG_RST1,
        }
    }

    /// This peripheral's bit within its own [`reset_register`].
    pub(crate) const fn reset_bit(self) -> u8 {
        match self {
            Peripheral::Spi => 1 << 0,
            Peripheral::I2c => 1 << 1,
            Peripheral::Uart => 1 << 2,
            Peripheral::Pwm => 1 << 4,
            Peripheral::Zb => 1 << 0,
            Peripheral::SysTimer => 1 << 1,
            Peripheral::Dma => 1 << 2,
            Peripheral::Aes => 1 << 4,
            Peripheral::Adc => 1 << 5,
        }
    }

    /// This peripheral's clock-enable register (`reg_clk_en0` or
    /// `reg_clk_en1`) — always the register at the same offset as
    /// [`reset_register`] within the shared 0x60/0x63 layout.
    pub(crate) const fn clock_register(self) -> u32 {
        match self {
            Peripheral::Spi | Peripheral::I2c | Peripheral::Uart | Peripheral::Pwm => {
                super::mmio::REG_CLK_EN0
            }
            Peripheral::Zb
            | Peripheral::SysTimer
            | Peripheral::Dma
            | Peripheral::Aes
            | Peripheral::Adc => super::mmio::REG_CLK_EN1,
        }
    }

    /// This peripheral's bit within its own [`clock_register`], or `None`
    /// if `register.h` documents no software clock-gate bit for it (only
    /// [`Peripheral::Adc`] today — see this enum's own doc section).
    pub(crate) const fn clock_bit(self) -> Option<u8> {
        match self {
            Peripheral::Spi => Some(1 << 0),
            Peripheral::I2c => Some(1 << 1),
            Peripheral::Uart => Some(1 << 2),
            Peripheral::Pwm => Some(1 << 4),
            Peripheral::Zb => Some(1 << 0),
            Peripheral::SysTimer => Some(1 << 1),
            Peripheral::Dma => Some(1 << 2),
            Peripheral::Aes => Some(1 << 4),
            Peripheral::Adc => None,
        }
    }
}

/// [`enable_clock`]/[`disable_clock`]/[`is_clock_enabled`] cannot act on a
/// [`Peripheral`] whose clock gating is not a documented register bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    /// `register.h`'s `reg_clk_en0`/`reg_clk_en1` has no bit for this
    /// peripheral (currently only [`Peripheral::Adc`] — see
    /// [`Peripheral`]'s own doc section on why).
    NotDocumented,
}

/// Assert then de-assert `peripheral`'s reset bit (`reg_rst0`/`reg_rst1`),
/// matching every migrated driver's own former "set the bit, read back,
/// clear the bit" sequence. Performed as two separate read-modify-write
/// steps under [`crate::mmio::with_irqs_disabled`] (re-reading the
/// register between the assert and de-assert writes, exactly as the
/// pre-facade per-driver code did) so an interrupt handler that happens to
/// also touch this shared register cannot observe — or be lost under — a
/// half-updated byte.
#[cfg(target_arch = "tc32")]
pub fn pulse_reset(peripheral: Peripheral) {
    use super::mmio::{r8, w8, with_irqs_disabled};
    let register = peripheral.reset_register();
    let bit = peripheral.reset_bit();
    with_irqs_disabled(|| unsafe {
        w8(register, r8(register) | bit);
        w8(register, r8(register) & !bit);
    });
}

/// Enable `peripheral`'s clock-gate bit (`reg_clk_en0`/`reg_clk_en1`).
/// Returns [`ClockError::NotDocumented`] instead of silently no-op'ing for
/// a peripheral with no documented clock bit (see [`Peripheral::Adc`]'s
/// doc).
#[cfg(target_arch = "tc32")]
pub fn enable_clock(peripheral: Peripheral) -> Result<(), ClockError> {
    set_clock_enabled(peripheral, true)
}

/// Disable `peripheral`'s clock-gate bit. See [`enable_clock`].
#[cfg(target_arch = "tc32")]
pub fn disable_clock(peripheral: Peripheral) -> Result<(), ClockError> {
    set_clock_enabled(peripheral, false)
}

#[cfg(target_arch = "tc32")]
fn set_clock_enabled(peripheral: Peripheral, enable: bool) -> Result<(), ClockError> {
    use super::mmio::{r8, w8, with_irqs_disabled};
    let bit = peripheral.clock_bit().ok_or(ClockError::NotDocumented)?;
    let register = peripheral.clock_register();
    with_irqs_disabled(|| unsafe {
        let value = r8(register);
        w8(register, if enable { value | bit } else { value & !bit });
    });
    Ok(())
}

/// `true` if `peripheral`'s clock-gate bit is currently set. See
/// [`enable_clock`] for the `Err` case.
#[cfg(target_arch = "tc32")]
pub fn is_clock_enabled(peripheral: Peripheral) -> Result<bool, ClockError> {
    use super::mmio::r8;
    let bit = peripheral.clock_bit().ok_or(ClockError::NotDocumented)?;
    Ok(unsafe { r8(peripheral.clock_register()) } & bit != 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reboot_bit_matches_register_h() {
        // FLD_PWDN_CTRL_REBOOT = BIT(5) = 0x20, and bsp.h's own
        // mcu_reset() writes exactly 0x20 to reg_pwdn_ctrl (0x6f).
        assert_eq!(FLD_PWDN_CTRL_REBOOT, 0x20);
    }

    #[test]
    fn reboot_bit_is_disjoint_from_sleep_bit() {
        // FLD_PWDN_CTRL_SLEEP = BIT(7) = 0x80, from the same register.h
        // enum; confirms the two control bits don't overlap so writing
        // FLD_PWDN_CTRL_REBOOT alone cannot accidentally also assert
        // FLD_PWDN_CTRL_SLEEP.
        const FLD_PWDN_CTRL_SLEEP: u8 = 1 << 7;
        assert_eq!(FLD_PWDN_CTRL_REBOOT & FLD_PWDN_CTRL_SLEEP, 0);
    }

    const ALL_PERIPHERALS: &[Peripheral] = &[
        Peripheral::Spi,
        Peripheral::I2c,
        Peripheral::Uart,
        Peripheral::Pwm,
        Peripheral::Zb,
        Peripheral::SysTimer,
        Peripheral::Dma,
        Peripheral::Aes,
        Peripheral::Adc,
    ];

    #[test]
    fn reset_bits_match_register_h() {
        assert_eq!(Peripheral::Spi.reset_bit(), 0x01);
        assert_eq!(Peripheral::I2c.reset_bit(), 0x02);
        assert_eq!(Peripheral::Uart.reset_bit(), 0x04);
        assert_eq!(Peripheral::Pwm.reset_bit(), 0x10);
        assert_eq!(Peripheral::Zb.reset_bit(), 0x01);
        assert_eq!(Peripheral::SysTimer.reset_bit(), 0x02);
        assert_eq!(Peripheral::Dma.reset_bit(), 0x04);
        assert_eq!(Peripheral::Aes.reset_bit(), 0x10);
        assert_eq!(Peripheral::Adc.reset_bit(), 0x20);
    }

    #[test]
    fn clock_bits_match_register_h_where_documented() {
        assert_eq!(Peripheral::Spi.clock_bit(), Some(0x01));
        assert_eq!(Peripheral::I2c.clock_bit(), Some(0x02));
        assert_eq!(Peripheral::Uart.clock_bit(), Some(0x04));
        assert_eq!(Peripheral::Pwm.clock_bit(), Some(0x10));
        assert_eq!(Peripheral::Zb.clock_bit(), Some(0x01));
        assert_eq!(Peripheral::SysTimer.clock_bit(), Some(0x02));
        assert_eq!(Peripheral::Dma.clock_bit(), Some(0x04));
        assert_eq!(Peripheral::Aes.clock_bit(), Some(0x10));
    }

    #[test]
    fn adc_has_no_documented_clock_bit() {
        // reg_clk_en1 (unlike reg_rst1) has no ADC bit in register.h — see
        // Peripheral's own doc section. Confirm the type-level contract
        // that callers cannot silently "succeed" at gating a
        // non-existent bit.
        assert_eq!(Peripheral::Adc.clock_bit(), None);
    }

    #[test]
    fn reset_register_matches_reg_rst0_or_reg_rst1() {
        for &peripheral in ALL_PERIPHERALS {
            let register = peripheral.reset_register();
            assert!(
                register == super::super::mmio::REG_RST0
                    || register == super::super::mmio::REG_RST1,
                "{peripheral:?} uses an unexpected reset register"
            );
        }
    }

    #[test]
    fn clock_register_matches_reg_clk_en0_or_reg_clk_en1() {
        for &peripheral in ALL_PERIPHERALS {
            let register = peripheral.clock_register();
            assert!(
                register == super::super::mmio::REG_CLK_EN0
                    || register == super::super::mmio::REG_CLK_EN1,
                "{peripheral:?} uses an unexpected clock register"
            );
        }
    }

    #[test]
    fn reset_register_and_clock_register_pair_up_consistently() {
        // Every peripheral must use reg_rst0 with reg_clk_en0, or reg_rst1
        // with reg_clk_en1 — never a mismatched pair (which would gate one
        // peripheral's clock while resetting a different one).
        for &peripheral in ALL_PERIPHERALS {
            let reset_is_0 = peripheral.reset_register() == super::super::mmio::REG_RST0;
            let clock_is_0 = peripheral.clock_register() == super::super::mmio::REG_CLK_EN0;
            assert_eq!(
                reset_is_0, clock_is_0,
                "{peripheral:?} pairs mismatched reset/clock registers"
            );
        }
    }

    #[test]
    fn reg0_peripherals_have_disjoint_reset_bits() {
        let mut seen = 0u8;
        for &peripheral in ALL_PERIPHERALS {
            if peripheral.reset_register() != super::super::mmio::REG_RST0 {
                continue;
            }
            let bit = peripheral.reset_bit();
            assert_eq!(bit.count_ones(), 1);
            assert_eq!(
                seen & bit,
                0,
                "{peripheral:?} collides with an earlier reg_rst0 bit"
            );
            seen |= bit;
        }
    }

    #[test]
    fn reg1_peripherals_have_disjoint_reset_bits() {
        let mut seen = 0u8;
        for &peripheral in ALL_PERIPHERALS {
            if peripheral.reset_register() != super::super::mmio::REG_RST1 {
                continue;
            }
            let bit = peripheral.reset_bit();
            assert_eq!(bit.count_ones(), 1);
            assert_eq!(
                seen & bit,
                0,
                "{peripheral:?} collides with an earlier reg_rst1 bit"
            );
            seen |= bit;
        }
    }

    #[test]
    fn reg0_peripherals_have_disjoint_clock_bits() {
        let mut seen = 0u8;
        for &peripheral in ALL_PERIPHERALS {
            if peripheral.clock_register() != super::super::mmio::REG_CLK_EN0 {
                continue;
            }
            if let Some(bit) = peripheral.clock_bit() {
                assert_eq!(
                    seen & bit,
                    0,
                    "{peripheral:?} collides with an earlier reg_clk_en0 bit"
                );
                seen |= bit;
            }
        }
    }

    #[test]
    fn reg1_peripherals_have_disjoint_clock_bits() {
        let mut seen = 0u8;
        for &peripheral in ALL_PERIPHERALS {
            if peripheral.clock_register() != super::super::mmio::REG_CLK_EN1 {
                continue;
            }
            if let Some(bit) = peripheral.clock_bit() {
                assert_eq!(
                    seen & bit,
                    0,
                    "{peripheral:?} collides with an earlier reg_clk_en1 bit"
                );
                seen |= bit;
            }
        }
    }

    #[test]
    fn reset_bit_and_clock_bit_agree_where_both_are_documented() {
        // Every migrated driver's pre-facade code used the *same* bit
        // position for its reset and clock-enable field (e.g. SPI is bit
        // 0 in both reg_rst0 and reg_clk_en0). Confirm that holds for
        // every peripheral that has both.
        for &peripheral in ALL_PERIPHERALS {
            if let Some(clock_bit) = peripheral.clock_bit() {
                assert_eq!(
                    peripheral.reset_bit(),
                    clock_bit,
                    "{peripheral:?} has mismatched reset/clock bit positions"
                );
            }
        }
    }

    #[test]
    fn all_peripherals_list_is_exhaustive() {
        assert_eq!(ALL_PERIPHERALS.len(), 9);
    }
}
