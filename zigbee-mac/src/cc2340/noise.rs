//! CC2340R5 radio ADC noise capture, the entropy input of [`super::entropy`].
//!
//! The CC23x0 has no general-purpose TRNG. TI's `RNGLPF3RF` driver seeds its
//! DRBG from IF-ADC noise collected while the radio is otherwise unused:
//! `RCL_AdcNoise_get_samples_blocking()` (`rcl/wrappers/RCL_AdcNoise.c`)
//! submits `RCL_CMD_ADC_NOISE_GET`, handled by
//! `RCL_Handler_ADC_Noise_getNoise()` (`rcl/handlers/adc_noise.c`) under
//! `LRF_configAdcNoise` (`adc_noise_bt5_cc23xx.json`). This module performs
//! the same register sequence directly, without RCL, from data `build.rs`
//! imports from the pinned SDK:
//!
//! 1. RCL `LRF_setupRadio()` for `LRF_configAdcNoise`: HFXT and its
//!    high-performance buffer, the LRFD clocks, the BLE5 **RFE** image only
//!    (`pbeImage = mceImage = NULL`; the PBE and MCE stay disabled), the
//!    142 ADC-noise register fields, `RSSI` invalid, `FIFOCMDADD`, and the
//!    FCFG trims with that PHY's `bwIndex = bwIndexDither = 1`;
//! 2. `LRF_enableSynthRefsys()`;
//! 3. `RCL_Handler_Adc_Noise_powerUp()`: precomputed 2440 MHz synthesizer
//!    words (`RFE_COMMON_RAM.K5`, `LRFDRFE32.CALMMID_CALMCRS`, `PLLM0`,
//!    `PLLM1`, `RXIF = 0`), RFE TOPsm `INIT`/`ENABLE`, wait for the RFE boot
//!    message (`LRFDRFE.MSGBOX == 4`), then RFE command 4 (PLL calibration)
//!    through `LRFDPBE.RFEAPI`;
//! 4. `RCL_Handler_Adc_Noise_configureS2R()`: `LRFDDBELL.CLKCTL.S2R`,
//!    `LRFDMDM.ADCDIGCONF = IBRANCHEN | QBRANCHEN`, `LRFDS2R.CFG = ONESHOT |
//!    SEL_ADCDIG | CTL_EN`, `START`/`STOP` addressing PBE RAM (S2R word
//!    address 2048 on the R5);
//! 5. on `LRFDDBELL.RIS0.RFEDONE` (PLL calibrated): RFE command 3 (RX
//!    start), wait for `LRFDMDM.RFECMDIN` bit 3, clear `LRFDRFE.LNA.EN` and
//!    `LRFDRFE.DIVCTL.RXPH90DIV/RXPH0DIV` (LNA and mixer off, so the ADC
//!    digitises front-end noise rather than an antenna signal), reset and
//!    enable `ADCDIG` (`LRFDMDM.INIT/ENABLE`), arm `LRFDS2R.TRIG`;
//! 6. wait until `LRFDS2R.STAT.RUNNING` clears, then
//!    `RCL_Handler_Adc_Noise_powerDown()`: RFE command 1 (stop RX) and 5
//!    (stop PLL), each acknowledged through `LRFDPBE.RFEMSGBOX`, S2R/ADCDIG
//!    off, RFE TOPsm power-down, `CLKCTL.S2R` off, refsys off;
//! 7. the samples are read from PBE RAM and then wiped, as TI's handler does
//!    (`memcpy` + `memset`).
//!
//! Differences from TI: every wait is bounded by the SYSTIM (a wedged RFE or
//! S2R becomes a [`NoiseError`] instead of a hang), completion of the PLL
//! calibration is polled in `RIS0` instead of taking the LRFD interrupt, the
//! samples are consumed in place instead of being copied to SRAM, and the
//! LRFD clocks and HFXT are stopped again afterwards.
//!
//! # Ownership
//!
//! The capture owns the whole LRFD: it overwrites PBE RAM (the IEEE PBE
//! image), RFE RAM (the IEEE RFE image) and the PHY registers. It may only
//! run while the IEEE radio is not initialised or is asleep, and afterwards
//! the radio driver must reload its images and re-run its bring-up
//! ([`super::driver::Cc2340Driver::invalidate_lrfd`]). The `LAES` core is not
//! touched here.

use super::config::{RegisterWidth, RegisterWrite};
#[cfg(target_os = "none")]
use super::hardware;

include!(concat!(env!("OUT_DIR"), "/cc2340_noise_config.rs"));

/// Most words one capture can hold (`RCL_ADC_NOISE_MAX_NUM_WORDS`: the
/// 4 KiB PBE RAM).
pub const MAX_WORDS: usize = 1024;

/// Failure of one ADC-noise capture. None of them yields samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoiseError {
    /// Built without the TI SDK ADC-noise configuration (fallback build).
    Unavailable,
    /// Zero or more than [`MAX_WORDS`] words requested.
    InvalidLength,
    /// The IEEE radio is initialised and awake (or holds an undelivered
    /// frame); the capture would destroy its state.
    RadioBusy,
    /// HFXT or the LRFD clock did not start.
    ClockTimeout,
    /// The RFE image does not fit the 4 KiB TOPsm RAM.
    ImageTooLarge,
    /// FCFG radio trims are erased.
    TrimUnavailable,
    /// The RFE did not post its boot message.
    RfeBootTimeout,
    /// The RFE did not finish the PLL calibration (`RFEDONE`).
    PllCalibrationTimeout,
    /// The receiver did not start (`RFECMDIN`).
    RxStartTimeout,
    /// The S2R one-shot capture did not finish.
    CaptureTimeout,
    /// The RFE did not acknowledge the RX/PLL stop command.
    StopTimeout,
}

/// Whether this image carries the ADC-noise configuration.
pub const fn available() -> bool {
    NOISE_CONFIG_AVAILABLE
}

pub(crate) const fn validate_words(words: usize) -> Result<usize, NoiseError> {
    if words == 0 || words > MAX_WORDS {
        Err(NoiseError::InvalidLength)
    } else {
        Ok(words)
    }
}

// The generated tables are only referenced by the target capture (and an
// unavailable table never names `RegisterWidth`).
const _: &str = NOISE_CONFIG_SOURCE;
const _: usize = NOISE_RFE_LEN;
const _: u32 = NOISE_RFE_CHECKSUM;
const _: &[(u16, u16)] = NOISE_RFE_RUNS;
const _: &[u32] = NOISE_RFE_WORDS;
const _: (&[RegisterWrite], RegisterWidth) = (ADC_NOISE_PHY_WRITES, RegisterWidth::U32);

/// Rebuild the BLE5 RFE image: the IEEE RFE image (embedded for the radio
/// anyway) with the build-time patch runs applied. `put(index, word)`
/// receives every word that differs from `ieee`.
#[cfg_attr(not(any(test, target_os = "none")), allow(dead_code))]
fn for_each_patch_word(mut put: impl FnMut(usize, u32)) {
    let mut words = NOISE_RFE_WORDS.iter().copied();
    for &(start, len) in NOISE_RFE_RUNS {
        for offset in 0..usize::from(len) {
            if let Some(word) = words.next() {
                put(usize::from(start) + offset, word);
            }
        }
    }
}

#[cfg(target_os = "none")]
mod lrf {
    use super::*;
    use hardware::{read32, wait_ticks, wait_until, write16, write32};

    const LRFDDBELL_BASE: u32 = 0x4008_0000;
    const LRFDPBE_BASE: u32 = 0x4008_1000;
    const LRFDMDM_BASE: u32 = 0x4008_2000;
    const LRFDRFE_BASE: u32 = 0x4008_3000;
    const LRFDRFE32_BASE: u32 = 0x4008_3400;
    const LRFDS2R_BASE: u32 = 0x4008_5000;
    const PBE_RAM_BASE: u32 = 0x4009_0000;
    const BUF_RAM_BASE: u32 = 0x4009_2000;
    const RFE_RAM_BASE: u32 = 0x4009_6000;

    const DBELL_CLKCTL: u32 = LRFDDBELL_BASE + 0x004;
    const DBELL_RIS0: u32 = LRFDDBELL_BASE + 0x048;
    const DBELL_ICLR0: u32 = LRFDDBELL_BASE + 0x054;
    const DBELL_CLKCTL_S2R: u32 = 0x0000_0010;
    const DBELL_RFEDONE: u32 = 0x0040_0000;

    const PBE_RFEAPI: u32 = LRFDPBE_BASE + 0x064;
    const PBE_RFEMSGBOX: u32 = LRFDPBE_BASE + 0x070;

    const MDM_ENABLE: u32 = LRFDMDM_BASE;
    const MDM_INIT: u32 = LRFDMDM_BASE + 0x008;
    const MDM_RFECMDIN: u32 = LRFDMDM_BASE + 0x09C;
    const MDM_ADCDIGCONF: u32 = LRFDMDM_BASE + 0x0C0;
    const MDM_ADCDIG: u32 = 0x0000_0020;
    const MDM_ADCDIGCONF_IQ: u32 = 0x0000_0003;
    const MDM_RFECMDIN_RX_RUNNING: u32 = 0x0000_0008;

    const RFE_ENABLE: u32 = LRFDRFE_BASE;
    const RFE_INIT: u32 = LRFDRFE_BASE + 0x008;
    const RFE_PDREQ: u32 = LRFDRFE_BASE + 0x00C;
    const RFE_MSGBOX: u32 = LRFDRFE_BASE + 0x054;
    const RFE_RSSI: u32 = LRFDRFE_BASE + 0x21C;
    const RFE_LNA: u32 = LRFDRFE_BASE + 0x0B0;
    const RFE_DIVCTL: u32 = LRFDRFE_BASE + 0x20C;
    const RFE_TOPSM: u32 = 0x0000_0001;
    const RFE_LNA_EN: u32 = 0x0000_0001;
    const RFE_DIVCTL_RXPH_DIV: u32 = 0x0000_0008 | 0x0000_0004;
    const RFE_BOOT_DONE: u32 = 4;

    const RFE32_PLLM0: u32 = LRFDRFE32_BASE + 0x0BC;
    const RFE32_PLLM1: u32 = LRFDRFE32_BASE + 0x0C0;
    const RFE32_CALMMID_CALMCRS: u32 = LRFDRFE32_BASE + 0x0C4;
    const RFE_RAM_K5: u32 = RFE_RAM_BASE + 0x81A;
    const RFE_RAM_RXIF: u32 = RFE_RAM_BASE + 0x81C;
    const PBE_FIFOCMDADD: u32 = BUF_RAM_BASE + 0x008;
    const PBE_FCMD_ADDRESS: u16 = (((LRFDPBE_BASE + 0x1A0) & 0x0FFF) >> 2) as u16;
    const INVALID_RSSI: u32 = 127;

    const S2R_CFG: u32 = LRFDS2R_BASE;
    const S2R_START: u32 = LRFDS2R_BASE + 0x004;
    const S2R_STOP: u32 = LRFDS2R_BASE + 0x008;
    const S2R_STAT: u32 = LRFDS2R_BASE + 0x00C;
    const S2R_TRIG: u32 = LRFDS2R_BASE + 0x010;
    /// `TRIGMODE_ONESHOT | SEL_ADCDIG | CTL_EN`.
    const S2R_CFG_ONESHOT_ADCDIG: u32 = 0x0000_0002 | 0x0000_0001;
    const S2R_RUNNING: u32 = 0x0000_0001;
    const S2R_TRIG_ARM: u32 = 0x0000_0001;
    /// Start of PBE RAM in the S2R address space (CC23x0R5).
    const S2R_PBE_RAM_START: u32 = 2048;

    /// RFE API commands used by `adc_noise.c`.
    const RFE_CMD_STOP_RX: u32 = 1;
    const RFE_CMD_START_RX: u32 = 3;
    const RFE_CMD_CALIBRATE_PLL: u32 = 4;
    const RFE_CMD_STOP_PLL: u32 = 5;

    /// `RCL_Handler_Adc_Noise_powerUp()`: precomputed 2440 MHz words.
    const K5_2440_MHZ: u16 = 0x9160;
    const CALMMID_CALMCRS_2440_MHZ: u32 = 0x0986_04C4;
    const PLLM0_2440_MHZ: u32 = 0x130E_0000;
    const PLLM1_2440_MHZ: u32 = 0x163B_0000;

    /// `LRF_configAdcNoise` swConfig: `bwIndex = bwIndexDither = 1`.
    const ADC_NOISE_TRIM_VARIANT: hardware::TrimVariant = hardware::TrimVariant {
        bw_index: 1,
        bw_index_dither: 1,
    };

    /// Bounds in 250 ns SYSTIM ticks. TI's handler waits without bound.
    const RFE_BOOT_TICKS: u32 = 4 * 1_000;
    const PLL_CALIBRATION_TICKS: u32 = 4 * 2_000;
    const RX_START_TICKS: u32 = 4 * 1_000;
    const RFE_STOP_TICKS: u32 = 4 * 1_000;
    /// ~11.5 Msample/s, three samples per word: 1024 words take ~270 us.
    const CAPTURE_TICKS: u32 = 4 * 2_000;

    impl From<hardware::HardwareError> for NoiseError {
        fn from(error: hardware::HardwareError) -> Self {
            match error {
                hardware::HardwareError::ImageTooLarge => Self::ImageTooLarge,
                hardware::HardwareError::FactoryTrimUnavailable => Self::TrimUnavailable,
                _ => Self::ClockTimeout,
            }
        }
    }

    /// The LRFD set up as `LRF_configAdcNoise` with captured samples in PBE
    /// RAM. Dropping it wipes the samples and stops the LRFD clocks and HFXT.
    pub(crate) struct Capture {
        words: usize,
    }

    impl Capture {
        pub(crate) fn word(&self, index: usize) -> u32 {
            if index < self.words {
                read32(PBE_RAM_BASE + 4 * index as u32)
            } else {
                0
            }
        }
    }

    impl Drop for Capture {
        fn drop(&mut self) {
            wipe_samples(self.words);
            hardware::disable_radio_clocks();
            hardware::stop_hfxt();
        }
    }

    fn wipe_samples(words: usize) {
        for index in 0..words {
            write32(PBE_RAM_BASE + 4 * index as u32, 0);
        }
    }

    /// Capture `words` ADC-noise words. The caller guarantees the IEEE radio
    /// is not running (see the module documentation).
    pub(crate) fn capture(words: usize) -> Result<Capture, NoiseError> {
        let words = validate_words(words)?;
        if !NOISE_CONFIG_AVAILABLE {
            return Err(NoiseError::Unavailable);
        }

        hardware::start_lrfd_clocks()?;
        // From here on the guard restores clocks/HFXT on every exit.
        let capture = Capture { words };
        wipe_samples(MAX_WORDS);

        let result = setup().and_then(|()| run(words));
        if result.is_err() {
            hard_stop();
        }
        result.map(|()| capture)
    }

    fn setup() -> Result<(), NoiseError> {
        // `LRF_loadImage(LRF_RFE_binary_ble5)`, stored as a patch over the
        // IEEE RFE image (see `build.rs`).
        let ieee = super::super::firmware::images()
            .ok_or(NoiseError::Unavailable)?
            .rfe;
        if NOISE_RFE_LEN > MAX_WORDS {
            return Err(NoiseError::ImageTooLarge);
        }
        hardware::load_rfe_image(ieee)?;
        for_each_patch_word(|index, word| {
            if index < NOISE_RFE_LEN {
                write32(RFE_RAM_BASE + 4 * index as u32, word);
            }
        });
        hardware::apply_phy_configuration(ADC_NOISE_PHY_WRITES);
        write32(RFE_RSSI, INVALID_RSSI);
        write16(PBE_FIFOCMDADD, PBE_FCMD_ADDRESS);
        hardware::apply_factory_radio_trim(ADC_NOISE_TRIM_VARIANT)?;
        hardware::enable_synth_refsys()?;
        Ok(())
    }

    fn run(words: usize) -> Result<(), NoiseError> {
        // RCL_Handler_Adc_Noise_powerUp()
        write16(RFE_RAM_K5, K5_2440_MHZ);
        write32(RFE32_CALMMID_CALMCRS, CALMMID_CALMCRS_2440_MHZ);
        write32(RFE32_PLLM0, PLLM0_2440_MHZ);
        write32(RFE32_PLLM1, PLLM1_2440_MHZ);
        write16(RFE_RAM_RXIF, 0);
        write32(DBELL_ICLR0, DBELL_RFEDONE);
        write32(RFE_INIT, RFE_TOPSM);
        write32(RFE_ENABLE, RFE_TOPSM);
        if !wait_field(RFE_BOOT_TICKS, RFE_MSGBOX, u32::MAX, RFE_BOOT_DONE, true) {
            return Err(NoiseError::RfeBootTimeout);
        }
        rfe_command(RFE_CMD_CALIBRATE_PLL);

        // RCL_Handler_Adc_Noise_configureS2R()
        write32(DBELL_CLKCTL, read32(DBELL_CLKCTL) | DBELL_CLKCTL_S2R);
        write32(MDM_ADCDIGCONF, MDM_ADCDIGCONF_IQ);
        write32(S2R_CFG, S2R_CFG_ONESHOT_ADCDIG);
        write32(S2R_START, S2R_PBE_RAM_START);
        write32(S2R_STOP, S2R_PBE_RAM_START + words as u32 - 1);

        // lrfEvents.rfedone: the PLL is calibrated.
        if !wait_field(
            PLL_CALIBRATION_TICKS,
            DBELL_RIS0,
            DBELL_RFEDONE,
            DBELL_RFEDONE,
            true,
        ) {
            return Err(NoiseError::PllCalibrationTimeout);
        }
        write32(DBELL_ICLR0, DBELL_RFEDONE);
        rfe_command(RFE_CMD_START_RX);
        if !wait_field(
            RX_START_TICKS,
            MDM_RFECMDIN,
            MDM_RFECMDIN_RX_RUNNING,
            MDM_RFECMDIN_RX_RUNNING,
            true,
        ) {
            return Err(NoiseError::RxStartTimeout);
        }
        write32(RFE_LNA, read32(RFE_LNA) & !RFE_LNA_EN);
        write32(RFE_DIVCTL, read32(RFE_DIVCTL) & !RFE_DIVCTL_RXPH_DIV);
        write32(MDM_INIT, MDM_ADCDIG);
        write32(MDM_ENABLE, MDM_ADCDIG);
        write32(S2R_TRIG, S2R_TRIG_ARM);

        // The capture needs ~words * 87 ns; then poll for the last part.
        wait_ticks(words as u32 / 3)?;
        if !wait_field(CAPTURE_TICKS, S2R_STAT, S2R_RUNNING, 0, true) {
            return Err(NoiseError::CaptureTimeout);
        }
        power_down()
    }

    /// `RCL_Handler_Adc_Noise_powerDown()`.
    fn power_down() -> Result<(), NoiseError> {
        let rx_stopped = rfe_command_acknowledged(RFE_CMD_STOP_RX);
        let pll_stopped = rfe_command_acknowledged(RFE_CMD_STOP_PLL);
        hard_stop();
        if rx_stopped && pll_stopped {
            Ok(())
        } else {
            Err(NoiseError::StopTimeout)
        }
    }

    /// Post an RFE API command (`LRFDRFE.MSGBOX` cleared first, as
    /// `adc_noise.c` does).
    #[inline(never)]
    fn rfe_command(command: u32) {
        write32(RFE_MSGBOX, 0);
        write32(PBE_RFEAPI, command);
    }

    /// [`rfe_command`], then wait for the RFE's reply in `LRFDPBE.RFEMSGBOX`.
    fn rfe_command_acknowledged(command: u32) -> bool {
        rfe_command(command);
        wait_field(RFE_STOP_TICKS, PBE_RFEMSGBOX, u32::MAX, 0, false)
    }

    /// Bounded wait until `(read32(address) & mask == value) == equal`. One
    /// out-of-line loop for every wait of the capture.
    #[inline(never)]
    fn wait_field(timeout_ticks: u32, address: u32, mask: u32, value: u32, equal: bool) -> bool {
        wait_until(timeout_ticks, || (read32(address) & mask == value) == equal)
    }

    /// The unconditional tail of `powerDown()`: S2R, ADCDIG and RFE off.
    /// Shared by the normal and the error path.
    #[inline(never)]
    fn hard_stop() {
        write32(S2R_CFG, 0);
        write32(MDM_INIT, 0xFFFF);
        write32(MDM_ENABLE, 0);
        write32(RFE_PDREQ, RFE_TOPSM);
        write32(RFE_ENABLE, 0);
        write32(RFE_PDREQ, 0);
        write32(DBELL_CLKCTL, read32(DBELL_CLKCTL) & !DBELL_CLKCTL_S2R);
        write32(DBELL_ICLR0, DBELL_RFEDONE);
        hardware::disable_synth_refsys();
    }
}

#[cfg(target_os = "none")]
pub(crate) use lrf::capture;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_empty_and_oversized_captures() {
        assert_eq!(validate_words(0), Err(NoiseError::InvalidLength));
        assert_eq!(
            validate_words(MAX_WORDS + 1),
            Err(NoiseError::InvalidLength)
        );
        assert_eq!(validate_words(MAX_WORDS), Ok(MAX_WORDS));
    }

    #[test]
    fn generated_tables_match_their_availability() {
        if available() {
            // BLE5 RFE image: 880 words in SDK 8.40 and lpf3-9.20.
            let len = core::hint::black_box(NOISE_RFE_LEN);
            assert!(len > 0 && len <= MAX_WORDS);
            let patched: usize = NOISE_RFE_RUNS
                .iter()
                .map(|&(_, len)| usize::from(len))
                .sum();
            assert_eq!(patched, NOISE_RFE_WORDS.len());
            assert!(
                NOISE_RFE_RUNS
                    .iter()
                    .all(|&(start, run)| usize::from(start) + usize::from(run) <= len)
            );
            assert!(!ADC_NOISE_PHY_WRITES.is_empty());
            assert!(ADC_NOISE_PHY_WRITES.iter().all(|write| {
                (0x4008_3000..0x4008_4000).contains(&write.address())
                    || (0x4009_6000..0x4009_7000).contains(&write.address())
            }));
            // adc_noise_bt5_cc23xx.json: 142 fields in 60 register writes.
            assert_eq!(ADC_NOISE_PHY_WRITES.len(), 60);
        } else {
            assert!(NOISE_RFE_RUNS.is_empty() && ADC_NOISE_PHY_WRITES.is_empty());
        }
    }

    /// The packed table decodes to exactly the writes `build.rs` resolved
    /// from TI's `adc_noise_bt5_cc23xx.json` (same order).
    #[test]
    fn packed_noise_phy_table_matches_resolved_writes() {
        assert_eq!(
            ADC_NOISE_PHY_WRITES.len(),
            ADC_NOISE_PHY_WRITES_RESOLVED.len()
        );
        assert!(
            ADC_NOISE_PHY_WRITES
                .iter()
                .map(|write| (write.address(), write.value(), write.width()))
                .eq(ADC_NOISE_PHY_WRITES_RESOLVED.iter().copied())
        );
    }

    /// The IEEE RFE image plus the patch must be TI's BLE5 RFE image
    /// (checksum computed by `build.rs` over the SDK array).
    #[test]
    fn patched_rfe_image_reproduces_the_sdk_ble5_image() {
        let Some(images) = super::super::firmware::images() else {
            return;
        };
        if !available() {
            return;
        }
        let mut rebuilt = [0u32; 1024];
        rebuilt[..images.rfe.len()].copy_from_slice(images.rfe);
        for_each_patch_word(|index, word| rebuilt[index] = word);
        let mut hash = 0x811C_9DC5u32;
        for byte in rebuilt[..NOISE_RFE_LEN]
            .iter()
            .flat_map(|word| word.to_le_bytes())
        {
            hash ^= u32::from(byte);
            hash = hash.wrapping_mul(0x0100_0193);
        }
        assert_eq!(hash, NOISE_RFE_CHECKSUM);
    }
}
