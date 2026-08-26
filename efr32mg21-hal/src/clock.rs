//! EFR32MG21 Series-2 HFXO and system-clock setup.
//!
//! This implements the subset needed by BRD4181A: crystal mode at 38.4 MHz,
//! divide-by-one HCLK/PCLK, and the GSDK Series-2 config-1 HFXO defaults.

const CMU_BASE: u32 = 0x4000_8000;
const CMU_LOCK: u32 = CMU_BASE + 0x010;
const CMU_SYSCLKCTRL: u32 = CMU_BASE + 0x070;

const HFXO_BASE: u32 = 0x4000_C000;
const HFXO_XTALCFG: u32 = HFXO_BASE + 0x010;
const HFXO_XTALCTRL: u32 = HFXO_BASE + 0x018;
const HFXO_CFG: u32 = HFXO_BASE + 0x020;
const HFXO_CTRL: u32 = HFXO_BASE + 0x028;
const HFXO_STATUS: u32 = HFXO_BASE + 0x058;
const HFXO_LOCK: u32 = HFXO_BASE + 0x080;

const SET_ALIAS_OFFSET: u32 = 0x1000;
const CLEAR_ALIAS_OFFSET: u32 = 0x2000;

const CMU_LOCK_UNLOCK: u32 = 0x93F7;
const CMU_SYSCLKCTRL_CLKSEL_MASK: u32 = 0x7;
const CMU_SYSCLKCTRL_CLKSEL_FSRCO: u32 = 0x1;
const CMU_SYSCLKCTRL_CLKSEL_HFXO: u32 = 0x3;
const CMU_SYSCLKCTRL_PCLKPRESC_MASK: u32 = 1 << 10;
const CMU_SYSCLKCTRL_HCLKPRESC_MASK: u32 = 0x3 << 12;

const HFXO_LOCK_UNLOCK: u32 = 0x580E;
const HFXO_CTRL_FORCEEN: u32 = 1 << 0;
const HFXO_CTRL_DISONDEMAND: u32 = 1 << 1;
const HFXO_CTRL_FORCEXI2GNDANA: u32 = 1 << 4;
const HFXO_CTRL_FORCEXO2GNDANA: u32 = 1 << 5;
const HFXO_STATUS_RDY: u32 = 1 << 0;
const HFXO_STATUS_COREBIASOPTRDY: u32 = 1 << 1;
const HFXO_STATUS_ENS: u32 = 1 << 16;
const HFXO_STATUS_FSMLOCK: u32 = 1 << 30;

const HFXO_XTALCFG_TIMEOUTSTEADY_MASK: u32 = 0xF << 20;
const HFXO_XTALCTRL_CONFIG_MASK: u32 = 0x0FFF_FFFF;
const HFXO_XTALCTRL_SKIPCOREBIASOPT: u32 = 1 << 31;
const HFXO_CFG_CONFIG_MASK: u32 = (1 << 0) | (1 << 2) | (1 << 3);

const STARTUP_TIMEOUT_ITERATIONS: u32 = 2_000_000;

/// Board-provided crystal configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HfxoConfig {
    pub frequency_hz: u32,
    pub ctune: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    UnsupportedFrequency,
    CtuneOutOfRange,
    FsrcoSelectTimeout,
    HfxoStopTimeout,
    HfxoStartTimeout,
    HfxoUnlockTimeout,
    HfxoSelectTimeout,
}

/// Marker proving that the declared system clock was selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemClocks {
    hclk_hz: u32,
}

impl SystemClocks {
    pub const fn hclk_hz(self) -> u32 {
        self.hclk_hz
    }
}

/// Exclusive clock-control token.
pub struct ClockControl {
    _private: (),
}

impl ClockControl {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Configure and select the 38.4 MHz crystal as HCLK and PCLK.
    pub fn configure_hfxo(&mut self, config: HfxoConfig) -> Result<SystemClocks, ClockError> {
        if config.frequency_hz != 38_400_000 {
            return Err(ClockError::UnsupportedFrequency);
        }
        if config.ctune > u8::MAX as u16 {
            return Err(ClockError::CtuneOutOfRange);
        }

        unsafe {
            write(CMU_LOCK, CMU_LOCK_UNLOCK);

            // FSRCO is the reset SYSCLK source and remains available. Move
            // there before touching HFXO analog settings, including when a
            // resident bootloader entered with HFXO selected.
            select_system_clock(CMU_SYSCLKCTRL_CLKSEL_FSRCO);
            wait_value(
                CMU_SYSCLKCTRL,
                CMU_SYSCLKCTRL_CLKSEL_MASK,
                CMU_SYSCLKCTRL_CLKSEL_FSRCO,
            )
            .ok_or(ClockError::FsrcoSelectTimeout)?;

            write(HFXO_LOCK, HFXO_LOCK_UNLOCK);
            write(HFXO_CTRL + SET_ALIAS_OFFSET, HFXO_CTRL_DISONDEMAND);
            write(HFXO_CTRL + CLEAR_ALIAS_OFFSET, HFXO_CTRL_FORCEEN);
            wait_clear(HFXO_STATUS, HFXO_STATUS_ENS).ok_or(ClockError::HfxoStopTimeout)?;

            // GSDK CMU_HFXOINIT_DEFAULT for Series-2 config 1:
            // 416 us CBLSP timeout, 833 us first-lock timeout, startup bias
            // 32/32, no startup CTUNE offset.
            write(HFXO_XTALCFG, initial_xtalcfg());
            modify(
                HFXO_XTALCTRL,
                HFXO_XTALCTRL_CONFIG_MASK,
                steady_xtalctrl(config.ctune as u8),
            );
            modify(HFXO_CFG, HFXO_CFG_CONFIG_MASK, 0); // crystal mode
            modify(
                HFXO_CTRL,
                HFXO_CTRL_FORCEEN
                    | HFXO_CTRL_DISONDEMAND
                    | HFXO_CTRL_FORCEXI2GNDANA
                    | HFXO_CTRL_FORCEXO2GNDANA,
                HFXO_CTRL_FORCEEN,
            );

            let ready = HFXO_STATUS_RDY
                | HFXO_STATUS_COREBIASOPTRDY
                | HFXO_STATUS_ENS
                | HFXO_STATUS_FSMLOCK;
            wait_value(HFXO_STATUS, ready, ready).ok_or(ClockError::HfxoStartTimeout)?;

            // Finish the first-lock optimization, then use the shorter 83 us
            // timeout and retain the optimized core-bias value.
            write(HFXO_CTRL + SET_ALIAS_OFFSET, HFXO_CTRL_DISONDEMAND);
            wait_clear(HFXO_STATUS, HFXO_STATUS_FSMLOCK).ok_or(ClockError::HfxoUnlockTimeout)?;
            modify(HFXO_XTALCFG, HFXO_XTALCFG_TIMEOUTSTEADY_MASK, 2 << 20);
            write(
                HFXO_XTALCTRL + SET_ALIAS_OFFSET,
                HFXO_XTALCTRL_SKIPCOREBIASOPT,
            );
            write(HFXO_CTRL + CLEAR_ALIAS_OFFSET, HFXO_CTRL_DISONDEMAND);

            // 38.4 MHz is below the config-1 40 MHz zero-wait-state limit for
            // flash and below the 50 MHz zero-wait-state SRAM limit.
            select_system_clock(CMU_SYSCLKCTRL_CLKSEL_HFXO);
            wait_value(
                CMU_SYSCLKCTRL,
                CMU_SYSCLKCTRL_CLKSEL_MASK,
                CMU_SYSCLKCTRL_CLKSEL_HFXO,
            )
            .ok_or(ClockError::HfxoSelectTimeout)?;

            // SYSCLK now supplies the hardware request, so software no longer
            // needs to force the oscillator.
            write(HFXO_CTRL + CLEAR_ALIAS_OFFSET, HFXO_CTRL_FORCEEN);
            wait_value(
                HFXO_STATUS,
                HFXO_STATUS_RDY | HFXO_STATUS_ENS,
                HFXO_STATUS_RDY | HFXO_STATUS_ENS,
            )
            .ok_or(ClockError::HfxoSelectTimeout)?;
        }

        Ok(SystemClocks {
            hclk_hz: config.frequency_hz,
        })
    }
}

const fn initial_xtalcfg() -> u32 {
    // TIMEOUTCBLSB=11, TIMEOUTSTEADY(first lock)=11,
    // COREBIASSTARTUP=32, COREBIASSTARTUPI=32.
    (11 << 24) | (11 << 20) | (32 << 6) | 32
}

const fn steady_xtalctrl(ctune: u8) -> u32 {
    // COREDGENANA=none, CTUNEFIXANA=both, CTUNEXOANA/CTUNEXIANA=ctune,
    // COREBIASANA=60. MG21 config 1 has no XI/XO CTUNE delta.
    (3 << 24) | ((ctune as u32) << 16) | ((ctune as u32) << 8) | 60
}

unsafe fn select_system_clock(source: u32) {
    unsafe {
        modify(
            CMU_SYSCLKCTRL,
            CMU_SYSCLKCTRL_CLKSEL_MASK
                | CMU_SYSCLKCTRL_PCLKPRESC_MASK
                | CMU_SYSCLKCTRL_HCLKPRESC_MASK,
            source,
        );
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

unsafe fn wait_clear(address: u32, mask: u32) -> Option<()> {
    unsafe { wait_value(address, mask, 0) }
}

unsafe fn wait_value(address: u32, mask: u32, expected: u32) -> Option<()> {
    for _ in 0..STARTUP_TIMEOUT_ITERATIONS {
        if unsafe { read(address) } & mask == expected {
            return Some(());
        }
        core::hint::spin_loop();
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gsdk_config_one_defaults_are_encoded_exactly() {
        assert_eq!(initial_xtalcfg(), 0x0BB0_0820);
        assert_eq!(steady_xtalctrl(133), 0x0385_853C);
    }

    #[test]
    fn board_clock_contract_is_narrow() {
        let mut clocks = ClockControl::new();
        assert_eq!(
            clocks.configure_hfxo(HfxoConfig {
                frequency_hz: 80_000_000,
                ctune: 133,
            }),
            Err(ClockError::UnsupportedFrequency)
        );
        assert_eq!(
            clocks.configure_hfxo(HfxoConfig {
                frequency_hz: 38_400_000,
                ctune: 256,
            }),
            Err(ClockError::CtuneOutOfRange)
        );
    }
}
