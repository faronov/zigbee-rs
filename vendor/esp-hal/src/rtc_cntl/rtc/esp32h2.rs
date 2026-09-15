use strum::FromRepr;

use crate::{
    clock::{
        PllClock,
        RtcClock,
        RtcFastClock,
        RtcSlowClock,
        XtalClock,
        clocks_ll::{
            esp32h2_rtc_bbpll_configure,
            esp32h2_rtc_bbpll_enable,
            esp32h2_rtc_update_to_xtal,
            regi2c_write_mask,
        },
    },
    peripherals::{LP_AON, PCR, PMU},
    rtc_cntl::RtcCalSel,
};

#[path = "h2_sleep_clock.rs"]
mod sleep_clock;

const I2C_PMU: u8 = 0x6d;
const I2C_PMU_HOSTID: u8 = 0;

const I2C_PMU_EN_I2C_RTC_DREG: u8 = 8;
const I2C_PMU_EN_I2C_RTC_DREG_MSB: u8 = 0;
const I2C_PMU_EN_I2C_RTC_DREG_LSB: u8 = 0;

const I2C_PMU_EN_I2C_DIG_DREG: u8 = 8;
const I2C_PMU_EN_I2C_DIG_DREG_MSB: u8 = 1;
const I2C_PMU_EN_I2C_DIG_DREG_LSB: u8 = 1;

const I2C_PMU_EN_I2C_RTC_DREG_SLP: u8 = 8;
const I2C_PMU_EN_I2C_RTC_DREG_SLP_MSB: u8 = 2;
const I2C_PMU_EN_I2C_RTC_DREG_SLP_LSB: u8 = 2;

const I2C_PMU_EN_I2C_DIG_DREG_SLP: u8 = 8;
const I2C_PMU_EN_I2C_DIG_DREG_SLP_MSB: u8 = 3;
const I2C_PMU_EN_I2C_DIG_DREG_SLP_LSB: u8 = 3;

const I2C_PMU_OR_XPD_RTC_REG: u8 = 8;
const I2C_PMU_OR_XPD_RTC_REG_MSB: u8 = 4;
const I2C_PMU_OR_XPD_RTC_REG_LSB: u8 = 4;

const I2C_PMU_OR_XPD_DIG_REG: u8 = 8;
const I2C_PMU_OR_XPD_DIG_REG_MSB: u8 = 5;
const I2C_PMU_OR_XPD_DIG_REG_LSB: u8 = 5;

const I2C_PMU_OR_XPD_TRX: u8 = 15;
const I2C_PMU_OR_XPD_TRX_MSB: u8 = 2;
const I2C_PMU_OR_XPD_TRX_LSB: u8 = 2;

pub(crate) fn init() {
    // * No peripheral reg i2c power up required on the target */
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_EN_I2C_RTC_DREG,
        I2C_PMU_EN_I2C_RTC_DREG_MSB,
        I2C_PMU_EN_I2C_RTC_DREG_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_EN_I2C_DIG_DREG,
        I2C_PMU_EN_I2C_DIG_DREG_MSB,
        I2C_PMU_EN_I2C_DIG_DREG_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_EN_I2C_RTC_DREG_SLP,
        I2C_PMU_EN_I2C_RTC_DREG_SLP_MSB,
        I2C_PMU_EN_I2C_RTC_DREG_SLP_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_EN_I2C_DIG_DREG_SLP,
        I2C_PMU_EN_I2C_DIG_DREG_SLP_MSB,
        I2C_PMU_EN_I2C_DIG_DREG_SLP_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_OR_XPD_RTC_REG,
        I2C_PMU_OR_XPD_RTC_REG_MSB,
        I2C_PMU_OR_XPD_RTC_REG_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_OR_XPD_DIG_REG,
        I2C_PMU_OR_XPD_DIG_REG_MSB,
        I2C_PMU_OR_XPD_DIG_REG_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_PMU,
        I2C_PMU_HOSTID,
        I2C_PMU_OR_XPD_TRX,
        I2C_PMU_OR_XPD_TRX_MSB,
        I2C_PMU_OR_XPD_TRX_LSB,
        0,
    );

    let pmu = PMU::regs();
    unsafe {
        pmu.power_pd_top_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpaon_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpcpu_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpperi_reserve().write(|w| w.bits(0));
        pmu.power_pd_hpwifi_cntl().write(|w| w.bits(0));
        pmu.power_pd_lpperi_cntl().write(|w| w.bits(0));

        pmu.hp_active_hp_regulator0()
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0()
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));

        pmu.hp_active_hp_ck_power().modify(|_, w| {
            w.hp_active_xpd_bbpll()
                .set_bit()
                .hp_active_xpd_bb_i2c()
                .set_bit()
                .hp_active_xpd_bbpll_i2c()
                .set_bit()
        });
        pmu.hp_active_sysclk().modify(|_, w| {
            w.hp_active_icg_sys_clock_en()
                .set_bit()
                .hp_active_sys_clk_slp_sel()
                .clear_bit()
                .hp_active_icg_slp_sel()
                .clear_bit()
        });
        pmu.hp_sleep_sysclk().modify(|_, w| {
            w.hp_sleep_icg_sys_clock_en()
                .clear_bit()
                .hp_sleep_sys_clk_slp_sel()
                .set_bit()
                .hp_sleep_icg_slp_sel()
                .set_bit()
        });
        pmu.hp_active_hp_sys_cntl().modify(|_, w| {
            w.hp_active_dig_cpu_stall()
                .clear_bit()
                .hp_active_dig_pause_wdt()
                .clear_bit()
        });
        pmu.slp_wakeup_cntl5()
            .modify(|_, w| w.lp_ana_wait_target().bits(15));
        pmu.slp_wakeup_cntl7()
            .modify(|_, w| w.ana_wait_target().bits(1700));
    }

    RtcClock::set_fast_freq(RtcFastClock::RcFast);
    RtcClock::set_slow_freq(RtcSlowClock::RcSlow);
}

pub(crate) fn configure_clock() {
    let cal_val = loop {
        let res = RtcClock::calibrate(RtcCalSel::RtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    LP_AON::regs()
        .store1()
        .modify(|_, w| unsafe { w.bits(cal_val) });
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn   = 0x01,
    /// Software resets the digital core by RTC_CNTL_SW_SYS_RST
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU 0
    Cpu0Mwdt0     = 0x0B,
    /// Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST
    Cpu0Sw        = 0x0C,
    /// RTC watch dog resets CPU 0
    Cpu0RtcWdt    = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut   = 0x0F,
    /// RTC watch dog resets digital core and rtc module
    SysRtcWdt     = 0x10,
    /// Main watch dog 1 resets CPU 0
    Cpu0Mwdt1     = 0x11,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt   = 0x12,
    /// Glitch on clock resets the digital core and rtc module
    SysClkGlitch  = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// Glitch on power resets the digital core
    CorePwrGlitch = 0x17,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpDigPower(u32);
    pub bool, vdd_spi_pd_en, set_vdd_spi_pd_en: 21;
    pub bool, mem_dslp, set_mem_dslp: 22;
    pub bool, modem_pd_en, set_modem_pd_en: 27;
    pub bool, cpu_pd_en, set_cpu_pd_en: 29;
    pub bool, top_pd_en, set_top_pd_en: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpClkPower(u32);
    pub bool, xpd_bbpll, set_xpd_bbpll: 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct XtalPower(u32);
    pub bool, xpd_xtal, set_xpd_xtal: 31;
}

#[derive(Clone, Copy, Default)]
pub struct HpSysPower {
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpDigPower(u32);
    pub bool, bod_source_sel, set_bod_source_sel: 27;
    pub u32, vddbat_mode, set_vddbat_mode: 29, 28;
    pub u32, mem_dslp, set_mem_dslp: 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpClkPower(u32);
    pub u32, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub u32, xpd_rc32k, set_xpd_rc32k: 29;
    pub u32, xpd_fosc, set_xpd_fosc: 30;
}

#[derive(Clone, Copy, Default)]
pub struct LpSysPower {
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpSysCntlReg(u32);
    pub bool, uart_wakeup_en, set_uart_wakeup_en: 24;
    pub bool, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub bool, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub bool, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub bool, dig_pause_wdt, set_dig_pause_wdt: 28;
    pub bool, dig_cpu_stall, set_dig_cpu_stall: 29;
}

pub(crate) fn rtc_clk_cpu_freq_set_xtal() {
    // Unlike C6, H2's MSPI can use the separate PLL-derived flash clock.
    // Do not manually disable BBPLL while executing from flash.
    esp32h2_rtc_update_to_xtal(XtalClock::_32M, 1);
}

pub(crate) struct SavedClockConfig(sleep_clock::ClockConfig);

impl SavedClockConfig {
    pub(crate) fn try_save() -> Option<Self> {
        let pcr = PCR::regs();
        sleep_clock::ClockConfig::decode(
            pcr.sysclk_conf().read().soc_clk_sel().bits(),
            pcr.cpu_freq_conf().read().cpu_div_num().bits(),
            pcr.ahb_freq_conf().read().ahb_div_num().bits(),
        )
        .map(Self)
    }

    #[crate::ram]
    pub(crate) fn restore(self) {
        if self.0.source == 1 {
            esp32h2_rtc_bbpll_enable();
            esp32h2_rtc_bbpll_configure(XtalClock::_32M, PllClock::Pll96MHz);
        }
        let pcr = PCR::regs();
        use crate::rtc_cntl::sleep::clock_restore::{self, Register};
        clock_restore::h2(
            self.0.cpu_div,
            self.0.ahb_div,
            self.0.source,
            self.0.cpu_mhz,
            |register, value| match register {
                Register::Cpu => {
                    pcr.cpu_freq_conf()
                        .modify(|_, w| unsafe { w.cpu_div_num().bits(value as u8) });
                }
                Register::Ahb => {
                    pcr.ahb_freq_conf()
                        .modify(|_, w| unsafe { w.ahb_div_num().bits(value as u8) });
                }
                Register::Source => {
                    pcr.sysclk_conf()
                        .modify(|_, w| unsafe { w.soc_clk_sel().bits(value as u8) });
                }
                Register::LatchBus => {
                    pcr.bus_clk_update()
                        .modify(|_, w| w.bus_clock_update().set_bit());
                    while pcr.bus_clk_update().read().bus_clock_update().bit_is_set() {}
                }
                Register::RomTicks => crate::rom::ets_update_cpu_frequency_rom(value),
            },
        );
    }
}

pub(crate) fn restore_sleep_bias() {
    // ESP-IDF v5.5.1 pmu_sleep_finish / soc/regi2c_bias.h: low-temperature fix.
    regi2c_write_mask(0x6a, 0, 0, 7, 4, 8);
}
