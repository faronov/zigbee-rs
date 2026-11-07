//! Register regions a TOP/CPU/modem power-down light sleep must retain
//! (`SleepMode::TopPowerDown`).
//!
//! The lists follow ESP-IDF v5.5.1's H2 REGDMA tables
//! (`soc/esp32h2/system_retention_periph.c`, `lowpower/port/esp32h2/
//! sleep_clock.c`, `esp_phy/esp32h2/include/btbb_retention_reg.h`,
//! `ieee802154/driver/esp_ieee802154_dev.c`) and CPU retention frames
//! (`lowpower/port/esp32h2/sleep_cpu.c`). H2 powers its REGDMA (PAU) down
//! with TOP, so IDF triggers the copies from software; this crate does the
//! same copies with the CPU.
//!
//! Every list is a macro so one source generates both the host-tested
//! [`Region`] tables and the register copy loops of the RAM-resident wake
//! path (a `const` table read at wake could sit in flash `.rodata`).

/// One contiguous run of 32-bit registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    pub name: &'static str,
    pub base: u32,
    pub words: usize,
}

/// Invoke `$cb!` with the PCR (clock/reset) regions. Restored first, at
/// XTAL, then latched with `PCR_BUS_CLOCK_UPDATE`.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_pcr_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            // PCR_UART0_CONF .. PCR_PWDET_SAR_CLK_CONF.
            (PCR, 0x6009_6000, 85),
            (PCR_RESET_EVENT_BYPASS, 0x6009_6FF0, 1),
        ] }
    };
}

/// Invoke `$cb!` with the flash controller regions. `SPI_MEM_MMU_ITEM_CONTENT`
/// (+0x37C) and `SPI_MEM_MMU_ITEM_INDEX` (+0x380) are never written: they
/// are an indirect window into the (retained) MMU table.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_spi_mem_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            (SPI1_MEM_0, 0x6000_3000, 55),
            (SPI1_MEM_1, 0x6000_3100, 41),
            (SPI1_MEM_CLOCK_GATE, 0x6000_3200, 1),
            (SPI1_MEM_MMU_POWER_CTRL, 0x6000_3384, 31),
            (SPI0_MEM_0, 0x6000_2000, 55),
            (SPI0_MEM_1, 0x6000_2100, 41),
            (SPI0_MEM_CLOCK_GATE, 0x6000_2200, 1),
            (SPI0_MEM_MMU_POWER_CTRL, 0x6000_2384, 31),
        ] }
    };
}

/// Invoke `$cb!` with the remaining TOP-domain system regions.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_system_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            (IO_MUX, 0x6009_0000, 29),
            (GPIO_FUNC_OUT_SEL, 0x6009_1554, 32),
            (GPIO_STATUS_NEXT_IN_SEL, 0x6009_114C, 127),
            (GPIO, 0x6009_1000, 61),
            (INTMTX, 0x6001_0000, 69),
            (HP_SYSTEM, 0x6009_5000, 12),
            (HP_APM, 0x6009_9000, 68),
            (TEE, 0x6009_8000, 33),
        ] }
    };
}

/// Invoke `$cb!` with the modem clock/reset regions (`sleep_clock.c`),
/// restored before the MAC and baseband so their clocks run.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_modem_clock_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            // MODEM_SYSCON_TEST_CONF .. MODEM_SYSCON_MEM_CONF.
            (MODEM_SYSCON, 0x600A_5400, 7),
            // MODEM_LPCON_TEST_CONF .. MODEM_LPCON_MEM_CONF.
            (MODEM_LPCON, 0x600A_D000, 7),
        ] }
    };
}

/// Invoke `$cb!` with the 802.15.4 baseband and MAC regions.
///
/// Conscious difference: IDF's MAC entry starts at `IEEE802154_COMMAND`
/// (+0); writing a saved command word would replay it, so the copy starts at
/// +4. The driver re-arms receive itself on resume.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_radio_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            (BT_BB_0, 0x600A_2000, 93),
            (BT_BB_1, 0x600A_2800, 62),
            (BT_BB_2, 0x600A_2C00, 19),
            // IEEE802154_CTRL_CFG .. IEEE802154_MAC_DATE.
            (IEEE802154_MAC, 0x600A_3004, 99),
        ] }
    };
}

/// SYSTIMER base (`DR_REG_SYSTIMER_BASE`); saved through its load/target
/// sequence, not as a plain copy.
pub const SYSTIMER_BASE: u32 = 0x6000_B000;
/// SYSTIMER words kept: CONF, two unit values, TARGET0_HI..TARGET2_CONF (9)
/// and INT_ENA.
pub const SYSTIMER_WORDS: usize = 1 + 4 + 9 + 1;

/// CPU-domain frames (`sleep_cpu.c`): PLIC MX/UX, CLINT M/U, INTPRI and the
/// cache wrap-around control.
#[macro_export]
#[doc(hidden)]
macro_rules! __h2_cpu_regions {
    ($cb:ident $(, $arg:tt)*) => {
        $cb! { $($arg,)* [
            // PLIC_MXINT_ENABLE .. PLIC_MXINT_CLAIM, PLIC_MXINT_CONF.
            (PLIC_MX, 0x2000_1000, 38),
            (PLIC_MX_CONF, 0x2000_13FC, 1),
            (PLIC_UX, 0x2000_1400, 38),
            (PLIC_UX_CONF, 0x2000_17FC, 1),
            // CLINT_MINT_SIP .. CLINT_MINT_MTIMECMP_H, and the U-mode copy.
            (CLINT_M, 0x2000_1800, 6),
            (CLINT_U, 0x2000_1C00, 6),
            // INTPRI_CORE0_CPU_INT_ENABLE .. INTPRI_RND_ECO_LOW, RND_ECO_HIGH.
            (INTPRI, 0x600C_5000, 45),
            (INTPRI_RND_ECO_HIGH, 0x600C_53FC, 1),
            (CACHE_WRAP_AROUND_CTRL, 0x600C_8020, 1),
        ] }
    };
}

macro_rules! table {
    ([$(($name:ident, $base:expr, $words:expr)),* $(,)?]) => {
        &[$(Region { name: stringify!($name), base: $base, words: $words }),*]
    };
}

pub const PCR: &[Region] = __h2_pcr_regions!(table);
pub const SPI_MEM: &[Region] = __h2_spi_mem_regions!(table);
pub const SYSTEM: &[Region] = __h2_system_regions!(table);
pub const MODEM_CLOCK: &[Region] = __h2_modem_clock_regions!(table);
pub const RADIO: &[Region] = __h2_radio_regions!(table);
pub const CPU: &[Region] = __h2_cpu_regions!(table);

/// Total words of a table, usable for buffer sizes.
pub const fn words(regions: &[Region]) -> usize {
    let mut total = 0;
    let mut i = 0;
    while i < regions.len() {
        total += regions[i].words;
        i += 1;
    }
    total
}

/// Non-critical CPU CSRs (`RvCoreNonCriticalSleepFrame`): mscratch, mideleg,
/// misa, tselect, tdata1, tdata2, tcontrol, 16 pmpaddr, 4 pmpcfg, 16 PMA
/// address, 16 PMA config, utvec, ustatus, uepc, ucause, and the 10 custom
/// CSRs (machine/user performance counters, CPU test bus, user GPIO).
pub const NONCRITICAL_CSR_WORDS: usize = 7 + 16 + 4 + 16 + 16 + 4 + 10;

/// Critical CPU frame (`RvCoreCriticalSleepFrame` without its unused `pc`):
/// the 31 GPRs, mstatus, mtvec, mcause, mtval, mie, mip, mepc and pmufunc.
/// The word offsets are fixed by the assembly in `hw/top.rs`.
pub const CRITICAL_FRAME_WORDS: usize = 31 + 8;

pub const PCR_WORDS: usize = words(PCR);
pub const SPI_MEM_WORDS: usize = words(SPI_MEM);
pub const SYSTEM_WORDS: usize = words(SYSTEM);
pub const MODEM_CLOCK_WORDS: usize = words(MODEM_CLOCK);
pub const RADIO_WORDS: usize = words(RADIO);
pub const CPU_WORDS: usize = words(CPU);

/// Every retained word, including the CPU frames.
pub const TOTAL_WORDS: usize = PCR_WORDS
    + SPI_MEM_WORDS
    + SYSTEM_WORDS
    + MODEM_CLOCK_WORDS
    + RADIO_WORDS
    + SYSTIMER_WORDS
    + CPU_WORDS
    + NONCRITICAL_CSR_WORDS
    + CRITICAL_FRAME_WORDS;

/// Retention RAM in bytes.
pub const TOTAL_BYTES: usize = TOTAL_WORDS * 4;

#[cfg(test)]
mod tests {
    use super::*;

    fn all() -> Vec<Region> {
        [PCR, SPI_MEM, SYSTEM, MODEM_CLOCK, RADIO, CPU]
            .iter()
            .flat_map(|t| t.iter().copied())
            .collect()
    }

    fn end(r: &Region) -> u32 {
        r.base + 4 * r.words as u32
    }

    #[test]
    fn regions_are_word_aligned_and_do_not_overlap() {
        let mut regions = all();
        regions.sort_by_key(|r| r.base);
        for r in &regions {
            assert_eq!(r.base % 4, 0, "{}", r.name);
            assert!(r.words > 0, "{}", r.name);
        }
        for pair in regions.windows(2) {
            assert!(
                end(&pair[0]) <= pair[1].base,
                "{} overlaps {}",
                pair[0].name,
                pair[1].name
            );
        }
    }

    #[test]
    fn idf_end_registers_are_included() {
        let find = |name: &str| all().into_iter().find(|r| r.name == name).unwrap();
        // INTMTX_CORE0_CLOCK_GATE_REG = base + 0x110.
        assert_eq!(end(&find("INTMTX")), 0x6001_0000 + 0x114);
        // PCR_PWDET_SAR_CLK_CONF_REG = base + 0x150.
        assert_eq!(end(&find("PCR")), 0x6009_6000 + 0x154);
        // HP_SYSTEM_MEM_TEST_CONF_REG = base + 0x2C.
        assert_eq!(end(&find("HP_SYSTEM")), 0x6009_5000 + 0x30);
        // HP_APM_CLOCK_GATE_REG = base + 0x10C; TEE_CLOCK_GATE_REG = base + 0x80.
        assert_eq!(end(&find("HP_APM")), 0x6009_9000 + 0x110);
        assert_eq!(end(&find("TEE")), 0x6009_8000 + 0x84);
        // IO_MUX_GPIO27_REG = base + 0x70; GPIO_PIN31_REG = base + 0xF0.
        assert_eq!(end(&find("IO_MUX")), 0x6009_0000 + 0x74);
        assert_eq!(end(&find("GPIO")), 0x6009_1000 + 0xF4);
        // GPIO_FUNC31_OUT_SEL_CFG = 0x5D0; FUNC124_IN_SEL_CFG = 0x344.
        assert_eq!(end(&find("GPIO_FUNC_OUT_SEL")), 0x6009_15D4);
        assert_eq!(end(&find("GPIO_STATUS_NEXT_IN_SEL")), 0x6009_1348);
        // IEEE802154_MAC_DATE_REG = base + 0x18C.
        assert_eq!(end(&find("IEEE802154_MAC")), 0x600A_3000 + 0x190);
    }

    #[test]
    fn mmu_window_and_mac_command_are_never_written() {
        for r in all() {
            for spi in [0x6000_2000u32, 0x6000_3000] {
                for hole in [spi + 0x37C, spi + 0x380] {
                    assert!(
                        !(r.base..end(&r)).contains(&hole),
                        "{} covers {hole:#x}",
                        r.name
                    );
                }
            }
            assert!(!(r.base..end(&r)).contains(&0x600A_3000), "{}", r.name);
        }
    }

    #[test]
    fn systimer_registers_are_not_plain_copies() {
        for r in all() {
            assert!(!(r.base..end(&r)).contains(&SYSTIMER_BASE), "{}", r.name);
        }
    }

    #[test]
    fn retention_ram_budget() {
        assert_eq!(PCR_WORDS, 86);
        assert_eq!(SPI_MEM_WORDS, 256);
        assert_eq!(SYSTEM_WORDS, 431);
        assert_eq!(MODEM_CLOCK_WORDS, 14);
        assert_eq!(RADIO_WORDS, 273);
        assert_eq!(CPU_WORDS, 137);
        assert_eq!(NONCRITICAL_CSR_WORDS, 73);
        assert_eq!(CRITICAL_FRAME_WORDS, 39);
        assert_eq!(TOTAL_WORDS, 1_324);
        assert_eq!(TOTAL_BYTES, 5_296);
    }
}
