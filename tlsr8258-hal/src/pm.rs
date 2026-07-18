//! TLSR8258 power-management primitives that are safe to implement without
//! vendor-closed-source register sequences: deep-sleep-retained analog
//! scratch registers and wakeup-source bit constants.
//!
//! **Scope note (read before using):** actual suspend/deep-sleep *entry*
//! (`cpu_sleep_wakeup_32k_rc`/`_32k_xtal`, `sleep_start`) is compiled into
//! `libdrivers_8258.a` (`pm.o`, `pm_32k_rc.o`, `pm_32k_xtal.o`) with **no**
//! open-source register sequence in `platform/chip_8258/pm.h` — only
//! function prototypes. Reimplementing that sequence from a black-box
//! disassembly without hardware-in-the-loop validation risks bricking or
//! hanging real boards (clock/LDO/XTAL sequencing, unlike the simpler
//! register pokes elsewhere in this crate), so it is **not implemented
//! here**. This module only exposes the pieces that *are* documented in the
//! open header:
//!
//!   * the analog scratch registers Telink reserves for carrying state
//!     across deep sleep ([`DEEP_ANA_REG0`] etc.), and
//!   * the wakeup-source bit constants used by the (currently
//!     unimplemented) sleep entry points, so application code that talks to
//!     the vendor's own `cpu_sleep_wakeup_32k_rc` via FFI (or a future,
//!     hardware-validated implementation of this module) has a single
//!     source of truth for them.

/// Analog scratch registers that survive a power cycle (`DEEP_ANA_REG0/1`,
/// `platform/chip_8258/pm.h`). Reset only by a full power cycle.
pub const DEEP_ANA_REG0: u8 = 0x3A;
pub const DEEP_ANA_REG1: u8 = 0x3B;

/// Analog scratch register that survives deep sleep (with or without SRAM
/// retention) but is reset by watchdog, chip reset, or the RESET pin.
/// Telink reserves this one for its own `SYS_NEED_REINIT_EXT32K` /
/// `SYS_DEEP_SLEEP_FLAG` bookkeeping (`SYS_DEEP_ANA_REG` in `pm.h`) — avoid
/// it unless you also own that bookkeeping.
pub const DEEP_ANA_REG2: u8 = 0x3C;

/// Analog scratch registers reset by watchdog, chip reset, or the RESET
/// pin (but *not* by deep sleep) — free for application use, matching
/// `DEEP_ANA_REG6..DEEP_ANA_REG10` in `platform/chip_8258/pm.h`.
pub const DEEP_ANA_REG6: u8 = 0x35;
pub const DEEP_ANA_REG7: u8 = 0x36;
pub const DEEP_ANA_REG8: u8 = 0x37;
pub const DEEP_ANA_REG9: u8 = 0x38;
pub const DEEP_ANA_REG10: u8 = 0x39;

/// Application-safe retention scratch registers — the *only* addresses
/// [`read_retention`]/[`write_retention`] can reach. These map 1:1 to
/// [`DEEP_ANA_REG6`]..[`DEEP_ANA_REG10`] above.
///
/// A previous version of this module accepted an arbitrary `u8` address
/// for those two functions, which made it possible to pass
/// [`DEEP_ANA_REG0`]/[`DEEP_ANA_REG1`] (power-cycle-only reset — surprising
/// persistence semantics for what looks like ordinary scratch storage) or
/// [`DEEP_ANA_REG2`] (Telink's own `SYS_DEEP_ANA_REG` bookkeeping register
/// — colliding with it can corrupt the vendor's own deep-sleep/32k-restart
/// state machine) with nothing but a doc comment saying not to. This enum
/// makes those addresses impossible to name through this API, rather than
/// merely discouraged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RetentionRegister {
    Reg6 = DEEP_ANA_REG6,
    Reg7 = DEEP_ANA_REG7,
    Reg8 = DEEP_ANA_REG8,
    Reg9 = DEEP_ANA_REG9,
    Reg10 = DEEP_ANA_REG10,
}

impl RetentionRegister {
    const fn address(self) -> u8 {
        self as u8
    }
}

/// Read an application-owned retention scratch byte.
#[cfg(target_arch = "tc32")]
pub fn read_retention(reg: RetentionRegister) -> Result<u8, super::mmio::AnalogError> {
    super::mmio::analog_read(reg.address())
}

/// Write an application-owned retention scratch byte.
#[cfg(target_arch = "tc32")]
pub fn write_retention(reg: RetentionRegister, value: u8) -> Result<(), super::mmio::AnalogError> {
    super::mmio::analog_write(reg.address(), value)
}

/// Wakeup source bits, matching `SleepWakeupSrc_TypeDef` in
/// `platform/chip_8258/pm.h`. Provided for callers driving the vendor's own
/// (closed) sleep entry points via FFI; this crate does not implement sleep
/// entry itself (see module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WakeupSource {
    Pad = 1 << 4,
    Core = 1 << 5,
    Timer = 1 << 6,
    Comparator = 1 << 7,
}

/// MCU status after reset, matching `pm_mcu_status` in
/// `platform/chip_8258/pm.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum McuStatus {
    Boot = 0,
    DeepRetentionBack = 1,
    DeepBack = 2,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retention_registers_are_distinct() {
        let regs = [
            DEEP_ANA_REG0,
            DEEP_ANA_REG1,
            DEEP_ANA_REG2,
            DEEP_ANA_REG6,
            DEEP_ANA_REG7,
            DEEP_ANA_REG8,
            DEEP_ANA_REG9,
            DEEP_ANA_REG10,
        ];
        for (i, a) in regs.iter().enumerate() {
            for b in &regs[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn retention_register_addresses_match_documented_scratch_registers() {
        assert_eq!(RetentionRegister::Reg6.address(), DEEP_ANA_REG6);
        assert_eq!(RetentionRegister::Reg7.address(), DEEP_ANA_REG7);
        assert_eq!(RetentionRegister::Reg8.address(), DEEP_ANA_REG8);
        assert_eq!(RetentionRegister::Reg9.address(), DEEP_ANA_REG9);
        assert_eq!(RetentionRegister::Reg10.address(), DEEP_ANA_REG10);
    }

    #[test]
    fn retention_register_cannot_name_power_cycle_or_reserved_registers() {
        // This is a compile-time property, not a runtime one: there is no
        // `RetentionRegister` variant whose address equals
        // DEEP_ANA_REG0/1/2. Assert it here so the constants can't drift
        // apart silently if `pm.rs` is edited later.
        let selectable = [
            RetentionRegister::Reg6.address(),
            RetentionRegister::Reg7.address(),
            RetentionRegister::Reg8.address(),
            RetentionRegister::Reg9.address(),
            RetentionRegister::Reg10.address(),
        ];
        for reserved in [DEEP_ANA_REG0, DEEP_ANA_REG1, DEEP_ANA_REG2] {
            assert!(!selectable.contains(&reserved));
        }
    }

    #[test]
    fn wakeup_source_bits_are_disjoint_powers_of_two() {
        let sources = [
            WakeupSource::Pad,
            WakeupSource::Core,
            WakeupSource::Timer,
            WakeupSource::Comparator,
        ];
        let mut seen = 0u8;
        for source in sources {
            let bit = source as u8;
            assert_eq!(bit & (bit - 1), 0, "not a single bit: {bit:#x}");
            assert_eq!(seen & bit, 0, "overlapping wakeup bit: {bit:#x}");
            seen |= bit;
        }
    }
}
