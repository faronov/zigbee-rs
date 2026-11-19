//! Customer Configuration (CCFG): the 88 bytes at the end of flash that the
//! boot ROM reads before starting the image (SDK `inc/hw_ccfg.h`,
//! `startup_files/ccfg.c`).
//!
//! The product links one [`Ccfg`] into the `.ccfg` output section at
//! `FLASH_SIZE - 88`. The board chooses the physical options (LF clock
//! source, ROM bootloader backdoor pin); [`Ccfg::check_recoverable`] is
//! evaluated at compile time by the product so an image can never disable
//! the serial ROM bootloader — on a USB dongle without a debug header it is
//! the only way back in.

/// Number of CCFG words.
pub const WORDS: usize = 22;
/// CCFG size in bytes (`SIZE_AND_DIS_FLAGS.SIZE_OF_CCFG`).
pub const SIZE: usize = WORDS * 4;

const EXT_LF_CLK: usize = 0;
const MODE_CONF_1: usize = 1;
const SIZE_AND_DIS_FLAGS: usize = 2;
const MODE_CONF: usize = 3;
const IEEE_MAC_0: usize = 8;
const IEEE_MAC_1: usize = 9;
const BL_CONFIG: usize = 12;
const ERASE_CONF: usize = 13;
const TI_OPTIONS: usize = 14;
const TAP_DAP_0: usize = 15;
const TAP_DAP_1: usize = 16;
const IMAGE_VALID_CONF: usize = 17;

const ENABLE: u32 = 0xC5;

const DIS_XOSC_OVR: u32 = 1 << 0;
const DIS_GPRAM: u32 = 1 << 2;
const DIS_TCXO: u32 = 1 << 3;
const DIS_LINEAR_CAPARRAY_DELTA_WORKAROUND: u32 = 1 << 4;

/// `MODE_CONF.SCLK_LF_OPTION`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LfClock {
    /// 32.768 kHz crystal (XOSC_LF).
    Crystal = 2,
    /// Internal RC oscillator (RCOSC_LF).
    RcOsc = 3,
}

/// ROM bootloader backdoor: holding `pin` at `active_high` during reset
/// enters the serial bootloader instead of the image.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Backdoor {
    pub pin: u8,
    pub active_high: bool,
}

/// A complete CCFG image.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ccfg {
    words: [u32; WORDS],
}

impl Ccfg {
    /// TI `ccfg.c` defaults for a 48 MHz crystal with the DC/DC converter,
    /// cache enabled (GPRAM off), debug (CPU DAP) enabled, image valid at
    /// address 0 and the ROM bootloader **disabled**; boards then select
    /// the LF clock and the bootloader backdoor.
    pub const fn ti_default() -> Self {
        Self {
            words: [
                // EXT_LF_CLK: DIO1, RTC increment 0x800000 (unused).
                0x0180_0000,
                // MODE_CONF_1: TCXO clipped-sine, TCXO_MAX_START 0x7F,
                // ALT_DCDC_VMIN 0xC (2.5 V), no dither, IPEAK 0, no IBIAS
                // deltas, XOSC_MAX_START 0x10 (1600 us).
                0xFFC0_0010,
                // SIZE_AND_DIS_FLAGS: 88 bytes; alternative DC/DC setting
                // enabled; XOSC override, GPRAM, TCXO and the cap-array
                // delta workaround disabled.
                (SIZE as u32) << 16
                    | 0xFFE0
                    | DIS_LINEAR_CAPARRAY_DELTA_WORKAROUND
                    | DIS_TCXO
                    | DIS_GPRAM
                    | DIS_XOSC_OVR,
                // MODE_CONF: VDDR_TRIM_SLEEP_DELTA 0xF, DC/DC in active and
                // recharge, VDDS_BOD_LEVEL 1 (with VDDR_EXT_LOAD 0: VDDR boost,
                // required for the +20 dBm PA), SCLK_LF = crystal, sleep TC
                // off, RTC/HF compensation off, 48 MHz crystal, no cap-array
                // delta, VDDR_CAP 0xFF.
                0xF1BB_FFFF,
                0xFFFF_FFFF, // VOLT_LOAD_0
                0xFFFF_FFFF, // VOLT_LOAD_1
                0xFFFF_FFFF, // RTC_OFFSET
                0xFFFF_FFFF, // FREQ_OFFSET
                0xFFFF_FFFF, // IEEE_MAC_0: use the FCFG1 primary address
                0xFFFF_FFFF, // IEEE_MAC_1
                0xFFFF_FFFF, // IEEE_BLE_0
                0xFFFF_FFFF, // IEEE_BLE_1
                // BL_CONFIG: bootloader disabled, backdoor disabled.
                0x00FF_FFFF,
                // ERASE_CONF: chip and bank erase allowed.
                0xFFFF_FFFF,
                // CCFG_TI_OPTIONS: TI FA unlock disabled.
                0xFFFF_FF00,
                // TAP_DAP_0: CPU DAP and PWRPROF TAP enabled, TEST TAP off.
                0xFFC5_C500,
                // TAP_DAP_1: PBIST2, PBIST1 and AON TAPs off.
                0xFF00_0000,
                // IMAGE_VALID_CONF: vector table at 0.
                0x0000_0000,
                0xFFFF_FFFF, // CCFG_PROT_31_0: no write/erase protection
                0xFFFF_FFFF,
                0xFFFF_FFFF,
                0xFFFF_FFFF,
            ],
        }
    }

    /// Select the LF clock source.
    pub const fn lf_clock(mut self, clock: LfClock) -> Self {
        self.words[MODE_CONF] = (self.words[MODE_CONF] & !(0x3 << 22)) | ((clock as u32) << 22);
        self
    }

    /// Enable the serial ROM bootloader with a backdoor pin.
    pub const fn rom_bootloader(mut self, backdoor: Backdoor) -> Self {
        self.words[BL_CONFIG] = ENABLE << 24
            | 0x7F << 17
            | (backdoor.active_high as u32) << 16
            | (backdoor.pin as u32) << 8
            | ENABLE;
        self
    }

    /// Override the IEEE 802.15.4 address the stack uses (secondary
    /// address; all-ones keeps the factory FCFG1 address).
    pub const fn ieee_mac(mut self, eui64_msb_first: u64) -> Self {
        self.words[IEEE_MAC_0] = eui64_msb_first as u32;
        self.words[IEEE_MAC_1] = (eui64_msb_first >> 32) as u32;
        self
    }

    pub const fn words(&self) -> [u32; WORDS] {
        self.words
    }

    pub const fn bootloader_backdoor(&self) -> Option<Backdoor> {
        let bl = self.words[BL_CONFIG];
        if bl >> 24 != ENABLE || bl & 0xFF != ENABLE {
            return None;
        }
        Some(Backdoor {
            pin: ((bl >> 8) & 0xFF) as u8,
            active_high: bl & (1 << 16) != 0,
        })
    }

    pub const fn sclk_lf(&self) -> u32 {
        (self.words[MODE_CONF] >> 22) & 0x3
    }

    /// Compile-time safety gate for a dongle that can only be recovered over
    /// UART: the ROM bootloader and its backdoor are enabled on a real DIO,
    /// chip/bank erase are allowed, the image is valid at 0, no sector is
    /// write-protected, the cache stays enabled (the linker scripts do not
    /// use GPRAM) and the cap-array workaround this HAL does not implement
    /// is disabled.
    pub const fn check_recoverable(&self) -> Result<(), &'static str> {
        match self.bootloader_backdoor() {
            None => return Err("CCFG must enable the ROM bootloader and its backdoor"),
            Some(b) if b.pin > 30 => return Err("CCFG backdoor pin is not a DIO"),
            Some(_) => {}
        }
        if self.words[ERASE_CONF] & 0x101 != 0x101 {
            return Err("CCFG must allow chip and bank erase");
        }
        if self.words[IMAGE_VALID_CONF] != 0 {
            return Err("CCFG image must be valid at address 0");
        }
        let mut i = 18;
        while i < WORDS {
            if self.words[i] != 0xFFFF_FFFF {
                return Err("CCFG must not write-protect flash sectors");
            }
            i += 1;
        }
        let flags = self.words[SIZE_AND_DIS_FLAGS];
        if flags >> 16 != SIZE as u32 {
            return Err("CCFG size must be 88 bytes");
        }
        if flags & DIS_GPRAM == 0 {
            return Err("CCFG must keep the cache enabled (DIS_GPRAM = 1)");
        }
        if flags & DIS_LINEAR_CAPARRAY_DELTA_WORKAROUND == 0 {
            return Err("CCFG cap-array delta workaround is not implemented by cc26x2-hal");
        }
        if (self.words[TAP_DAP_0] >> 16) & 0xFF != ENABLE {
            return Err("CCFG must keep the CPU DAP enabled");
        }
        let _ = (EXT_LF_CLK, MODE_CONF_1, TI_OPTIONS, TAP_DAP_1);
        Ok(())
    }

    /// Little-endian bytes as linked at the end of flash.
    pub const fn to_bytes(&self) -> [u8; SIZE] {
        let mut out = [0u8; SIZE];
        let mut i = 0;
        while i < WORDS {
            let b = self.words[i].to_le_bytes();
            out[i * 4] = b[0];
            out[i * 4 + 1] = b[1];
            out[i * 4 + 2] = b[2];
            out[i * 4 + 3] = b[3];
            i += 1;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The CCFG read from the lab Sonoff ZBDongle-P (Z-Stack image,
    /// `cc2652p-00124b0020cd1d7d-backup/backup.json`).
    const SONOFF_ZBDONGLE_P: [u32; WORDS] = [
        0x0180_0000,
        0xFFC0_0010,
        0x0058_FFFD,
        0xF1BB_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xC5FE_0FC5,
        0xFFFF_FFFF,
        0xFFFF_FF00,
        0xFFC5_C500,
        0xFF00_0000,
        0x0000_0000,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
        0xFFFF_FFFF,
    ];

    #[test]
    fn dongle_configuration_reproduces_the_shipped_ccfg() {
        let ccfg = Ccfg::ti_default()
            .lf_clock(LfClock::Crystal)
            .rom_bootloader(Backdoor {
                pin: 15,
                active_high: false,
            });
        assert_eq!(ccfg.words(), SONOFF_ZBDONGLE_P);
        assert_eq!(ccfg.check_recoverable(), Ok(()));
        assert_eq!(
            ccfg.bootloader_backdoor(),
            Some(Backdoor {
                pin: 15,
                active_high: false
            })
        );
        assert_eq!(ccfg.sclk_lf(), 2);
        assert_eq!(&ccfg.to_bytes()[0x30..0x34], &[0xC5, 0x0F, 0xFE, 0xC5]);
    }

    #[test]
    fn ti_default_is_not_recoverable_without_a_bootloader() {
        assert!(Ccfg::ti_default().check_recoverable().is_err());
    }

    #[test]
    fn protection_and_gpram_are_rejected() {
        let base = Ccfg::ti_default().rom_bootloader(Backdoor {
            pin: 15,
            active_high: false,
        });
        let mut protected = base;
        protected.words[18] = 0xFFFF_FFFE;
        assert!(protected.check_recoverable().is_err());
        let mut gpram = base;
        gpram.words[SIZE_AND_DIS_FLAGS] &= !DIS_GPRAM;
        assert!(gpram.check_recoverable().is_err());
        let mut no_dap = base;
        no_dap.words[TAP_DAP_0] = 0xFF00_C500;
        assert!(no_dap.check_recoverable().is_err());
    }
}
