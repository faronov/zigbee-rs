//! EFR32MG1 Series 1 clock setup used before timers or radio startup.

const CMU_BASE: u32 = 0x400E_4000;
const CMU_CTRL: u32 = CMU_BASE;
const CMU_HFXOCTRL: u32 = CMU_BASE + 0x024;
const CMU_HFXOCTRL1: u32 = CMU_BASE + 0x028;
const CMU_HFXOSTARTUPCTRL: u32 = CMU_BASE + 0x02C;
const CMU_HFXOSTEADYSTATECTRL: u32 = CMU_BASE + 0x030;
const CMU_HFXOTIMEOUTCTRL: u32 = CMU_BASE + 0x034;
const CMU_OSCENCMD: u32 = CMU_BASE + 0x060;
const CMU_HFCLKSEL: u32 = CMU_BASE + 0x074;
const CMU_STATUS: u32 = CMU_BASE + 0x090;
const CMU_HFCLKSTATUS: u32 = CMU_BASE + 0x094;
const CMU_HFBUSCLKEN0: u32 = CMU_BASE + 0x0B0;
const CMU_HFPERCLKEN0: u32 = CMU_BASE + 0x0C0;
const CMU_HFPRESC: u32 = CMU_BASE + 0x100;
const CMU_HFCOREPRESC: u32 = CMU_BASE + 0x108;
const CMU_HFPERPRESC: u32 = CMU_BASE + 0x10C;

const MSC_BASE: u32 = 0x400E_0000;
const MSC_READCTRL: u32 = MSC_BASE + 0x004;
const MSC_LOCK: u32 = MSC_BASE + 0x040;

const CMU_CTRL_HFPERCLKEN: u32 = 1 << 20;
const CMU_STATUS_HFRCORDY: u32 = 1 << 1;
const CMU_STATUS_HFXOENS: u32 = 1 << 2;
const CMU_STATUS_HFXORDY: u32 = 1 << 3;
const CMU_OSCENCMD_HFRCOEN: u32 = 1 << 0;
const CMU_OSCENCMD_HFXOEN: u32 = 1 << 2;
const CMU_OSCENCMD_HFXODIS: u32 = 1 << 3;
const CMU_HFCLK_HFRCO: u32 = 1;
const CMU_HFCLK_HFXO: u32 = 2;
const CMU_HFCLKSTATUS_SELECTED_MASK: u32 = 0x7;
const CMU_HFPRESC_MASK: u32 = 0x1F00;
const CMU_HFCOREPER_PRESC_MASK: u32 = 0x1FF00;

const CMU_HFXOCTRL_INIT_MASK: u32 = 0x0000_0101;
const CMU_HFXOCTRL1_PEAKDETTHR_MASK: u32 = 0x0000_0007;
const CMU_HFXOSTEADYSTATECTRL_INIT_MASK: u32 = 0xF00F_FFFF;

const MSC_READCTRL_MODE_MASK: u32 = 0x0300_0000;
const MSC_READCTRL_MODE_WS1: u32 = 0x0100_0000;
const MSC_LOCK_UNLOCK: u32 = 0x1B71;
const MSC_LOCK_LOCK: u32 = 0;

const DEFAULT_TIMEOUT: u32 = 1_000_000;

/// Board-supplied HFXO configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HfxoConfig {
    pub frequency_hz: u32,
    pub ctune: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    UnsupportedFrequency,
    CtuneOutOfRange,
    HfrcoStartTimeout,
    HfrcoSelectTimeout,
    HfxoStopTimeout,
    HfxoStartTimeout,
    HfxoSelectTimeout,
}

/// Configure and select an EFR32xG1 HFXO using the GSDK 4.5 emlib defaults.
///
/// This deliberately leaves DCDC and low-energy clocks untouched. It sets one
/// flash wait state before raising HCLK above 25 MHz.
pub fn init_hfxo(config: HfxoConfig) -> Result<(), ClockError> {
    if config.frequency_hz != 38_400_000 {
        return Err(ClockError::UnsupportedFrequency);
    }
    if config.ctune > 0x1FF {
        return Err(ClockError::CtuneOutOfRange);
    }

    unsafe {
        // Always move to HFRCO before changing HFXO analog configuration.
        write(CMU_OSCENCMD, CMU_OSCENCMD_HFRCOEN);
        wait_set(CMU_STATUS, CMU_STATUS_HFRCORDY)
            .ok_or(ClockError::HfrcoStartTimeout)?;
        write(CMU_HFCLKSEL, CMU_HFCLK_HFRCO);
        wait_value(
            CMU_HFCLKSTATUS,
            CMU_HFCLKSTATUS_SELECTED_MASK,
            CMU_HFCLK_HFRCO,
        )
        .ok_or(ClockError::HfrcoSelectTimeout)?;

        if read(CMU_STATUS) & CMU_STATUS_HFXOENS != 0 {
            write(CMU_OSCENCMD, CMU_OSCENCMD_HFXODIS);
            wait_clear(CMU_STATUS, CMU_STATUS_HFXOENS)
                .ok_or(ClockError::HfxoStopTimeout)?;
        }

        // GSDK 4.5 CMU_HFXOInit defaults for internal SDID 80 (EFR32xG1):
        // crystal/low-noise mode, startup CTUNE 160, steady CTUNE supplied by
        // the board, startup/steady IBTRIMXOCORE 0x20/0x7, REGISH 0xA,
        // REGISHUPPER min(REGISH + 3, 0xF), and 7/4/A/2 timeout fields.
        modify(CMU_HFXOCTRL, CMU_HFXOCTRL_INIT_MASK, 0);
        modify(CMU_HFXOCTRL1, CMU_HFXOCTRL1_PEAKDETTHR_MASK, 0x6);
        write(CMU_HFXOSTARTUPCTRL, (0xA0 << 11) | 0x20);
        modify(
            CMU_HFXOSTEADYSTATECTRL,
            CMU_HFXOSTEADYSTATECTRL_INIT_MASK,
            (0xD << 28) | ((config.ctune as u32) << 11) | (0xA << 7) | 0x7,
        );
        write(
            CMU_HFXOTIMEOUTCTRL,
            (0x2 << 16) | (0xA << 12) | (0x4 << 4) | 0x7,
        );

        // HCLK/HFCORE/HFPER are all divide-by-one for the declared 38.4 MHz.
        modify(CMU_HFPRESC, CMU_HFPRESC_MASK, 0);
        modify(CMU_HFCOREPRESC, CMU_HFCOREPER_PRESC_MASK, 0);
        modify(CMU_HFPERPRESC, CMU_HFCOREPER_PRESC_MASK, 0);
        set_flash_wait_state_38m4();

        write(CMU_OSCENCMD, CMU_OSCENCMD_HFXOEN);
        wait_set(CMU_STATUS, CMU_STATUS_HFXORDY)
            .ok_or(ClockError::HfxoStartTimeout)?;
        write(CMU_HFCLKSEL, CMU_HFCLK_HFXO);
        wait_value(
            CMU_HFCLKSTATUS,
            CMU_HFCLKSTATUS_SELECTED_MASK,
            CMU_HFCLK_HFXO,
        )
        .ok_or(ClockError::HfxoSelectTimeout)?;
    }
    Ok(())
}

#[inline]
pub fn enable_gpio_clock() {
    unsafe {
        modify(CMU_HFBUSCLKEN0, 1 << 2, 1 << 2);
    }
}

#[inline]
pub fn enable_i2c0_clock() {
    unsafe {
        modify(CMU_CTRL, CMU_CTRL_HFPERCLKEN, CMU_CTRL_HFPERCLKEN);
        modify(CMU_HFPERCLKEN0, 1 << 7, 1 << 7);
    }
}

unsafe fn set_flash_wait_state_38m4() {
    // EFR32xG1 permits up to 25 MHz at WS0 and 40 MHz at WS1 at 1.2 V.
    let was_locked = unsafe { read(MSC_LOCK) != 0 };
    unsafe {
        write(MSC_LOCK, MSC_LOCK_UNLOCK);
        modify(
            MSC_READCTRL,
            MSC_READCTRL_MODE_MASK,
            MSC_READCTRL_MODE_WS1,
        );
        if was_locked {
            write(MSC_LOCK, MSC_LOCK_LOCK);
        }
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

unsafe fn wait_set(address: u32, mask: u32) -> Option<()> {
    unsafe { wait_value(address, mask, mask) }
}

unsafe fn wait_clear(address: u32, mask: u32) -> Option<()> {
    unsafe { wait_value(address, mask, 0) }
}

unsafe fn wait_value(address: u32, mask: u32, expected: u32) -> Option<()> {
    for _ in 0..DEFAULT_TIMEOUT {
        if unsafe { read(address) } & mask == expected {
            return Some(());
        }
        core::hint::spin_loop();
    }
    None
}
