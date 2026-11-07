//! Register backend: the ESP-IDF v5.5.1 H2 `esp_light_sleep_start` /
//! `esp_deep_sleep_start` sequence (`sleep_modes.c` -> `pmu_sleep_init` ->
//! `pmu_sleep_start` -> `pmu_sleep_finish` -> `rtc_clk_cpu_freq_set_config`).
//!
//! Step A reproduces, register for register, the hardware-proven vendor
//! esp-hal backport (`rtc_cntl/sleep/esp32h2.rs`) it retires. One conscious
//! difference: the CPU clock is decoded *before* the LP timer is armed, so a
//! refused request leaves no wake armed.
//!
//! RAM residency: everything between the sleep request and the restored CPU
//! clock runs from `.rwtext` and calls only ROM or other `.rwtext` code (HP
//! sleep powers BBPLL down, and on H2 MSPI may be clocked from it). The
//! linked ELF is checked by `tools/esp32h2-sleep-ram-check.py`.
#![allow(unused_unsafe)]

#[cfg(feature = "top-pd")]
mod top;

use core::convert::Infallible;

use esp32h2 as pac;

use crate::analog::{SleepAnalog, h2_deep_sleep_analog, h2_light_sleep_analog};
use crate::clock::{ClockConfig, Register, restore_order};
use crate::error::SleepError;
use crate::mode::{DEEP_PD, WAKE_TIMER, plan_light, validate_deep};
use crate::power::{PowerWords, SlowClock, power_words};
use crate::timing::{
    SleepParams, SleepTime, divided_fast_clock, fast_calibration_cycles, fast_clock_period,
    timer_ticks,
};
use crate::{ChipInfo, Request, TOP_PD_SUPPORTED, Wake};

// ---------------------------------------------------------------------------
// Register blocks and ROM

#[inline(always)]
fn pmu() -> &'static pac::pmu::RegisterBlock {
    unsafe { &*pac::PMU::ptr() }
}
#[inline(always)]
fn lp_aon() -> &'static pac::lp_aon::RegisterBlock {
    unsafe { &*pac::LP_AON::ptr() }
}
#[inline(always)]
fn lp_timer() -> &'static pac::lp_timer::RegisterBlock {
    unsafe { &*pac::LP_TIMER::ptr() }
}
#[inline(always)]
fn lp_clkrst() -> &'static pac::lp_clkrst::RegisterBlock {
    unsafe { &*pac::LP_CLKRST::ptr() }
}
#[inline(always)]
fn pcr() -> &'static pac::pcr::RegisterBlock {
    unsafe { &*pac::PCR::ptr() }
}
#[inline(always)]
fn timg0() -> &'static pac::timg0::RegisterBlock {
    unsafe { &*pac::TIMG0::ptr() }
}
#[inline(always)]
fn apb_saradc() -> &'static pac::apb_saradc::RegisterBlock {
    unsafe { &*pac::APB_SARADC::ptr() }
}
#[inline(always)]
fn modem_lpcon() -> &'static pac::modem_lpcon::RegisterBlock {
    unsafe { &*pac::MODEM_LPCON::ptr() }
}
#[inline(always)]
fn i2c_ana_mst() -> &'static pac::i2c_ana_mst::RegisterBlock {
    unsafe { &*pac::I2C_ANA_MST::ptr() }
}

/// ROM functions by their fixed `esp32h2.rom.ld` addresses, so the crate does
/// not depend on a ROM linker script and calls never resolve into flash.
pub(crate) mod rom {
    #[inline(always)]
    pub(crate) fn delay_us(us: u32) {
        let f: unsafe extern "C" fn(u32) = unsafe { core::mem::transmute::<usize, _>(0x4000_0040) };
        unsafe { f(us) }
    }

    #[inline(always)]
    pub(crate) fn update_cpu_frequency(mhz: u32) {
        let f: unsafe extern "C" fn(u32) = unsafe { core::mem::transmute::<usize, _>(0x4000_0048) };
        unsafe { f(mhz) }
    }
}

// ---------------------------------------------------------------------------
// Analog I2C master (`regi2c_ctrl_ll.h`, `regi2c_ctrl.c`)

#[derive(Clone, Copy)]
enum I2cBlock {
    Bbpll,
    Bias,
}

impl I2cBlock {
    #[inline(always)]
    fn addr(self) -> u8 {
        match self {
            I2cBlock::Bbpll => 0x66,
            I2cBlock::Bias => 0x6a,
        }
    }
}

/// `regi2c_write_mask`: read-modify-write bits `msb..=lsb` of an analog
/// register through the I2C master serving `block`.
#[unsafe(link_section = ".rwtext")]
#[inline(never)]
fn regi2c_write_mask(block: I2cBlock, reg: u8, msb: u8, lsb: u8, data: u8) {
    let mst = i2c_ana_mst();
    unsafe {
        modem_lpcon()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().set_bit());
    }
    // Enable the slave and pick the master it is wired to.
    let sel = mst.ana_conf2().read();
    let selected = match block {
        I2cBlock::Bbpll => sel.bbpll_mst_sel().bit_is_set(),
        I2cBlock::Bias => sel.bias_mst_sel().bit_is_set(),
    };
    unsafe {
        mst.ana_conf1().write(|w| {
            w.bits(0x00FF_FFFF);
            match block {
                I2cBlock::Bbpll => w.bbpll_rd().clear_bit(),
                I2cBlock::Bias => w.bias_rd().clear_bit(),
            }
        });
    }
    let master = if selected { 0 } else { 1 };
    let addr = block.addr();
    unsafe {
        mst.i2c_ctrl(master).write(|w| {
            w.slave_addr().bits(addr);
            w.slave_reg_addr().bits(reg)
        });
    }
    while mst.i2c_ctrl(master).read().busy().bit_is_set() {}
    let unwritten = (!(u32::MAX << lsb) | (u32::MAX << (msb + 1))) as u8;
    let mask = !(u32::MAX << (msb - lsb + 1)) as u8;
    let bits = (data & mask) << lsb;
    unsafe {
        mst.i2c_ctrl(master).modify(|r, w| {
            w.slave_addr().bits(addr);
            w.slave_reg_addr().bits(reg);
            w.read_write().set_bit();
            w.data().bits((r.data().bits() & unwritten) | bits)
        });
    }
    while mst.i2c_ctrl(master).read().busy().bit_is_set() {}
}

// ---------------------------------------------------------------------------
// CPU clock (`rtc_clk.c`)

fn current_clock() -> Result<ClockConfig, SleepError> {
    let pcr = pcr();
    ClockConfig::decode(
        pcr.sysclk_conf().read().soc_clk_sel().bits(),
        pcr.cpu_freq_conf().read().cpu_div_num().bits(),
        pcr.ahb_freq_conf().read().ahb_div_num().bits(),
    )
    .ok_or(SleepError::UnsupportedClock)
}

#[inline(always)]
fn latch_bus_clock() {
    let pcr = pcr();
    unsafe {
        pcr.bus_clk_update()
            .modify(|_, w| w.bus_clock_update().set_bit());
    }
    while pcr.bus_clk_update().read().bus_clock_update().bit_is_set() {}
}

/// `rtc_clk_cpu_freq_set_xtal` (H2 `rtc_clk_cpu_freq_to_xtal`, divider 1).
/// Runs from flash: on H2 MSPI keeps its own PLL-derived clock and BBPLL is
/// not touched here.
fn switch_to_xtal() {
    let pcr = pcr();
    rom::update_cpu_frequency(32);
    unsafe {
        pcr.ahb_freq_conf().modify(|_, w| w.ahb_div_num().bits(0));
        pcr.cpu_freq_conf().modify(|_, w| w.cpu_div_num().bits(0));
        pcr.sysclk_conf().modify(|_, w| w.soc_clk_sel().bits(0));
    }
    latch_bus_clock();
}

/// `rtc_clk_bbpll_enable`.
#[inline(always)]
fn bbpll_enable() {
    unsafe {
        pmu().imm_hp_ck_power().modify(|_, w| {
            w.tie_high_xpd_bb_i2c().set_bit();
            w.tie_high_xpd_bbpll().set_bit();
            w.tie_high_xpd_bbpll_i2c().set_bit()
        });
        pmu()
            .imm_hp_ck_power()
            .modify(|_, w| w.tie_high_global_bbpll_icg().set_bit());
    }
}

/// `rtc_clk_bbpll_configure` (XTAL 32 MHz -> BBPLL 96 MHz).
#[inline(always)]
fn bbpll_configure() {
    const MODEM_LPCON_CLK_I2C_SEL_96M: u32 = 1 << 0;
    let mst = i2c_ana_mst();
    unsafe {
        modem_lpcon()
            .clk_conf_force_on()
            .modify(|_, w| w.clk_i2c_mst_fo().set_bit());
        modem_lpcon()
            .clk_conf()
            .modify(|r, w| w.bits(r.bits() | MODEM_LPCON_CLK_I2C_SEL_96M));
        mst.ana_conf0().modify(|_, w| {
            w.bbpll_stop_force_high().clear_bit();
            w.bbpll_stop_force_low().set_bit()
        });
    }
    // oc_ref_div, oc_div, oc_dhref_sel, oc_dlref_sel
    regi2c_write_mask(I2cBlock::Bbpll, 2, 3, 0, 0);
    regi2c_write_mask(I2cBlock::Bbpll, 3, 5, 0, 1);
    regi2c_write_mask(I2cBlock::Bbpll, 5, 5, 4, 3);
    regi2c_write_mask(I2cBlock::Bbpll, 5, 7, 6, 1);
    while mst.ana_conf0().read().cal_done().bit_is_clear() {}
    // Workaround: BBPLL calibration might stop early.
    rom::delay_us(10);
    unsafe {
        mst.ana_conf0().modify(|_, w| {
            w.bbpll_stop_force_high().set_bit();
            w.bbpll_stop_force_low().clear_bit()
        });
    }
}

/// `rtc_clk_cpu_freq_set_config` with the configuration saved before sleep.
#[unsafe(link_section = ".rwtext")]
#[inline(never)]
fn restore_clock(config: ClockConfig) {
    if config.needs_bbpll() {
        bbpll_enable();
        bbpll_configure();
    }
    let pcr = pcr();
    restore_order(&config, |register, value| match register {
        Register::Cpu => unsafe {
            pcr.cpu_freq_conf()
                .modify(|_, w| w.cpu_div_num().bits(value as u8));
        },
        Register::Ahb => unsafe {
            pcr.ahb_freq_conf()
                .modify(|_, w| w.ahb_div_num().bits(value as u8));
        },
        Register::Source => unsafe {
            pcr.sysclk_conf()
                .modify(|_, w| w.soc_clk_sel().bits(value as u8));
        },
        Register::LatchBus => latch_bus_clock(),
        Register::RomTicks => rom::update_cpu_frequency(value),
    });
}

// ---------------------------------------------------------------------------
// RC_FAST calibration (`rtc_time.c` `rtc_clk_cal_internal`, RC_FAST branch)

/// REF_TICK divider the ECO2+ divided measurement needs
/// (`clk_ll_rc_fast_tick_conf`).
const FOSC_TICK_NUM: u8 = 255;

/// Counted XTAL cycles for `cycles` RC_FAST periods; 0 on timeout.
fn calibrate_rc_fast(revision: u16, cycles: u32) -> u32 {
    let pmu = pmu();
    let clkrst = lp_clkrst();
    let pcr = pcr();
    let timg = timg0();

    let xpd_was_on = pmu
        .hp_sleep_lp_ck_power()
        .read()
        .hp_sleep_xpd_fosc_clk()
        .bit_is_set();
    let icg_was_on = clkrst.clk_to_hp().read().icg_hp_fosc().bit_is_set();
    if !xpd_was_on {
        unsafe {
            pmu.hp_sleep_lp_ck_power()
                .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
        }
        rom::delay_us(50);
    }
    if !icg_was_on {
        unsafe {
            clkrst.clk_to_hp().modify(|_, w| w.icg_hp_fosc().set_bit());
        }
        rom::delay_us(5);
    }

    // Stop a running cycling calibration with a tiny timeout.
    if timg
        .rtccalicfg()
        .read()
        .rtc_cali_start_cycling()
        .bit_is_set()
    {
        unsafe {
            timg.rtccalicfg2()
                .modify(|_, w| w.rtc_cali_timeout_thres().bits(1));
        }
        while !timg.rtccalicfg().read().rtc_cali_rdy().bit_is_set()
            && !timg.rtccalicfg2().read().rtc_cali_timeout().bit_is_set()
        {}
    }

    unsafe {
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_clk_sel().bits(1));
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_max().bits(cycles as u16));
        timg.rtccalicfg2()
            .modify(|_, w| w.rtc_cali_timeout_thres().bits(0x01FF_FFFF));
        pcr.ctrl_tick_conf()
            .modify(|_, w| w.fosc_tick_num().bits(FOSC_TICK_NUM));
    }
    let divided = divided_fast_clock(revision);
    let tick_was_enabled = pcr.ctrl_tick_conf().read().tick_enable().bit();
    if divided {
        unsafe {
            pcr.ctrl_tick_conf()
                .modify(|_, w| w.tick_enable().set_bit());
        }
    }

    unsafe {
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());
    }
    // Expected duration at the nominal 8.5 MHz RC_FAST.
    rom::delay_us(cycles * 1_000_000 / 8_500_000);
    let value = loop {
        if timg.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
            let raw = timg.rtccalicfg1().read().rtc_cali_value().bits();
            break if divided { raw >> 5 } else { raw };
        }
        if timg.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
            break 0;
        }
    };
    unsafe {
        timg.rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
    }

    if !icg_was_on {
        unsafe {
            clkrst
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_fosc().clear_bit());
        }
    }
    if !xpd_was_on {
        unsafe {
            pmu.hp_sleep_lp_ck_power()
                .modify(|_, w| w.hp_sleep_xpd_fosc_clk().clear_bit());
        }
    }
    unsafe {
        pcr.ctrl_tick_conf()
            .modify(|_, w| w.tick_enable().bit(tick_was_enabled));
    }
    value
}

/// `SleepTimeConfig::new`: the boot-time slow period and a fresh RC_FAST
/// calibration (must run after the switch to XTAL, as IDF).
fn sleep_time(revision: u16) -> Result<SleepTime, SleepError> {
    let slow_period = lp_aon().store1().read().bits();
    if slow_period == 0 {
        return Err(SleepError::InvalidCalibration(
            crate::CalibrationFailure::SlowClockPeriodZero,
        ));
    }
    let cycles = fast_calibration_cycles(revision);
    let xtal_cycles = calibrate_rc_fast(revision, cycles);
    let fast_period = fast_clock_period(xtal_cycles, 32, cycles)?;
    Ok(SleepTime {
        slow_period,
        fast_period,
    })
}

// ---------------------------------------------------------------------------
// LP timer

fn rtc_now() -> u64 {
    let timer = lp_timer();
    unsafe {
        timer.update().write(|w| w.main_timer_update().set_bit());
    }
    let high = timer.main_buf0_high().read().main_timer_buf0_high().bits();
    let low = timer.main_buf0_low().read().main_timer_buf0_low().bits();
    (u64::from(high) << 32) | u64::from(low)
}

/// Arm LP timer target 0 `duration_us` from now (`lp_timer_hal_set_alarm_target`).
fn arm_timer(duration_us: u64) -> Result<(), SleepError> {
    let ticks = timer_ticks(u128::from(duration_us), lp_aon().store1().read().bits())?;
    let target = (rtc_now() + ticks) & ((1 << 48) - 1);
    let timer = lp_timer();
    unsafe {
        timer.tar0_high().write(|w| {
            w.main_timer_tar_high0()
                .bits(((target >> 32) & 0xffff) as u16)
        });
        timer
            .tar0_low()
            .write(|w| w.main_timer_tar_low0().bits(target as u32));
        timer.int_clr().write(|w| w.soc_wakeup_int_clr().set_bit());
        timer
            .tar0_high()
            .modify(|_, w| w.main_timer_tar_en0().set_bit());
    }
    Ok(())
}

/// Disable target 0 and clear its wake interrupt.
fn disarm_timer() {
    let timer = lp_timer();
    unsafe {
        timer
            .tar0_high()
            .modify(|_, w| w.main_timer_tar_en0().clear_bit());
        timer.int_clr().write(|w| w.soc_wakeup_int_clr().set_bit());
    }
}

// ---------------------------------------------------------------------------
// PMU sleep configuration (`pmu_sleep_init`)

fn slow_clock() -> SlowClock {
    SlowClock::from_sel(lp_clkrst().lp_clk_conf().read().slow_clk_sel().bits())
}

/// `pmu_sleep_power_init`.
fn apply_power(words: &PowerWords) {
    let pmu = pmu();
    unsafe {
        pmu.hp_sleep_dig_power().modify(|_, w| w.bits(words.hp_dig));
        pmu.hp_sleep_hp_ck_power()
            .modify(|_, w| w.bits(words.hp_clk));
        pmu.hp_sleep_xtal()
            .modify(|_, w| w.hp_sleep_xpd_xtal().bit(words.hp_xpd_xtal));
        pmu.hp_sleep_lp_dig_power()
            .modify(|_, w| w.bits(words.lp_active_dig));
        pmu.hp_sleep_lp_ck_power()
            .modify(|_, w| w.bits(words.lp_active_clk));
        pmu.lp_sleep_lp_dig_power()
            .modify(|_, w| w.bits(words.lp_sleep_dig));
        pmu.lp_sleep_lp_ck_power()
            .modify(|_, w| w.bits(words.lp_sleep_clk));
        pmu.lp_sleep_xtal()
            .modify(|_, w| w.lp_sleep_xpd_xtal().bit(words.lp_xpd_xtal));
    }
}

/// `pmu_sleep_digital_init`: every HP function clock gated (no UART/LEDC
/// sleep reference); light sleep stalls the CPU and pauses the watchdogs.
fn apply_digital(light: bool, top_pd: bool) {
    let pmu = pmu();
    unsafe {
        pmu.hp_sleep_sysclk()
            .modify(|_, w| w.hp_sleep_icg_sys_clock_en().clear_bit());
        pmu.hp_sleep_icg_hp_func()
            .modify(|_, w| w.hp_sleep_dig_icg_func_en().bits(0));
        if light {
            pmu.hp_sleep_hp_sys_cntl().modify(|_, w| {
                w.hp_sleep_dig_pad_slp_sel()
                    .bit(!top_pd)
                    .hp_sleep_dig_cpu_stall()
                    .set_bit()
                    .hp_sleep_dig_pause_wdt()
                    .set_bit()
            });
        }
    }
}

/// `pmu_sleep_analog_init` (HP_SLEEP / LP_SLEEP regulator and bias).
fn apply_analog(analog: &SleepAnalog) {
    let pmu = pmu();
    unsafe {
        pmu.hp_sleep_bias().modify(|_, w| {
            w.hp_sleep_xpd_trx()
                .bit(analog.hp_xpd_trx)
                .hp_sleep_pd_cur()
                .bit(analog.hp_pd_cur)
                .sleep()
                .bit(analog.hp_bias_sleep)
        });
        pmu.hp_sleep_hp_regulator0().modify(|_, w| {
            w.hp_sleep_hp_regulator_xpd()
                .bit(analog.hp_xpd)
                .hp_sleep_hp_regulator_dbias()
                .bits(analog.hp_dbias)
        });
        pmu.hp_sleep_hp_regulator1()
            .modify(|_, w| w.hp_sleep_hp_regulator_drv_b().bits(analog.hp_drv_b));
        pmu.lp_sleep_bias().modify(|_, w| {
            w.lp_sleep_pd_cur()
                .bit(analog.lp_pd_cur)
                .sleep()
                .bit(analog.lp_bias_sleep)
        });
        pmu.lp_sleep_lp_regulator0().modify(|_, w| {
            w.lp_sleep_lp_regulator_slp_xpd()
                .bit(analog.lp_slp_xpd)
                .lp_sleep_lp_regulator_xpd()
                .bit(analog.lp_xpd)
                .lp_sleep_lp_regulator_slp_dbias()
                .bits(analog.lp_slp_dbias)
                .lp_sleep_lp_regulator_dbias()
                .bits(analog.lp_dbias)
        });
        pmu.lp_sleep_lp_regulator1()
            .modify(|_, w| w.lp_sleep_lp_regulator_drv_b().bits(analog.lp_drv_b));
    }
}

/// HP_SLEEP/LP_SLEEP analog registers before an oscillator-off sleep, so a
/// later oscillator-on sleep keeps the original retained configuration.
#[derive(Clone, Copy)]
struct SavedAnalog([u32; 6]);

impl SavedAnalog {
    fn read() -> Self {
        let pmu = pmu();
        Self([
            pmu.hp_sleep_bias().read().bits(),
            pmu.hp_sleep_hp_regulator0().read().bits(),
            pmu.hp_sleep_hp_regulator1().read().bits(),
            pmu.lp_sleep_bias().read().bits(),
            pmu.lp_sleep_lp_regulator0().read().bits(),
            pmu.lp_sleep_lp_regulator1().read().bits(),
        ])
    }

    #[inline(always)]
    fn restore(self) {
        let pmu = pmu();
        let [bias, reg0, reg1, lp_bias, lp_reg0, lp_reg1] = self.0;
        unsafe {
            pmu.hp_sleep_bias().write(|w| w.bits(bias));
            pmu.hp_sleep_hp_regulator0().write(|w| w.bits(reg0));
            pmu.hp_sleep_hp_regulator1().write(|w| w.bits(reg1));
            pmu.lp_sleep_bias().write(|w| w.bits(lp_bias));
            pmu.lp_sleep_lp_regulator0().write(|w| w.bits(lp_reg0));
            pmu.lp_sleep_lp_regulator1().write(|w| w.bits(lp_reg1));
        }
    }
}

/// `pmu_sleep_param_init`.
fn apply_params(p: &SleepParams) {
    let pmu = pmu();
    unsafe {
        pmu.slp_wakeup_cntl3().modify(|_, w| {
            w.hp_min_slp_val()
                .bits(p.hp_min_slp)
                .lp_min_slp_val()
                .bits(p.lp_min_slp)
        });
        pmu.slp_wakeup_cntl7()
            .modify(|_, w| w.ana_wait_target().bits(p.hp_analog_wait));
        pmu.slp_wakeup_cntl5()
            .modify(|_, w| w.lp_ana_wait_target().bits(p.lp_analog_wait));
        pmu.power_wait_timer0().modify(|_, w| {
            w.dg_hp_wait_timer()
                .bits(p.hp_supply_wait)
                .dg_hp_powerup_timer()
                .bits(p.hp_power_up_wait)
        });
        pmu.power_wait_timer1().modify(|_, w| {
            w.dg_lp_wait_timer()
                .bits(p.lp_supply_wait)
                .dg_lp_powerup_timer()
                .bits(p.lp_power_up_wait)
        });
        pmu.power_ck_wait_cntl().modify(|_, w| {
            w.wait_xtl_stable()
                .bits(p.xtal_wait)
                .wait_pll_stable()
                .bits(p.pll_wait)
        });
    }
}

/// `misc_modules_sleep_prepare`: the SAR ADC power detector (IDF-7370).
struct SarPowerDetect(bool);

impl SarPowerDetect {
    fn suspend() -> Self {
        let ctrl = apb_saradc().ctrl();
        let saved = ctrl.read().saradc2_pwdet_drv().bit();
        unsafe {
            ctrl.modify(|_, w| w.saradc2_pwdet_drv().clear_bit());
        }
        Self(saved)
    }

    fn resume(self) {
        unsafe {
            apb_saradc()
                .ctrl()
                .modify(|_, w| w.saradc2_pwdet_drv().bit(self.0));
        }
    }
}

// ---------------------------------------------------------------------------
// Sleep request (`pmu_sleep_start` .. `pmu_sleep_finish`)

/// Arm wake/reject sources and request sleep; spin until the PMU reports a
/// wake or a reject. Returns whether the request was rejected.
#[inline(always)]
fn request_sleep(deep: bool, wake: u32, reject: u32) -> bool {
    let pmu = pmu();
    unsafe {
        // lp_aon_hal_inform_wakeup_type: the ROM wake stub selection.
        lp_aon()
            .store9()
            .modify(|r, w| w.bits((r.bits() & !1) | u32::from(deep)));
        pmu.slp_wakeup_cntl2().write(|w| w.bits(wake));
        pmu.slp_wakeup_cntl1()
            .modify(|_, w| w.slp_reject_en().set_bit().sleep_reject_ena().bits(reject));
        pmu.int_clr().write(|w| {
            w.soc_wakeup()
                .clear_bit_by_one()
                .soc_sleep_reject()
                .clear_bit_by_one()
        });
        pmu.slp_wakeup_cntl4()
            .write(|w| w.slp_reject_cause_clr().set_bit());
        pmu.slp_wakeup_cntl0().write(|w| w.sleep_req().set_bit());
    }
    loop {
        let raw = pmu.int_raw().read();
        if raw.soc_wakeup().bit_is_set() || raw.soc_sleep_reject().bit_is_set() {
            return raw.soc_sleep_reject().bit_is_set();
        }
    }
}

/// `pmu_sleep_finish` low-temperature bias fix (`regi2c_bias.h`).
#[inline(always)]
fn restore_sleep_bias() {
    regi2c_write_mask(I2cBlock::Bias, 0, 7, 4, 8);
}

/// Sleep with every domain powered and restore the analog state and the CPU
/// clock, without leaving RAM.
#[unsafe(link_section = ".rwtext")]
#[inline(never)]
fn sleep_retained(
    deep: bool,
    wake: u32,
    reject: u32,
    saved: Option<SavedAnalog>,
    clock: ClockConfig,
) -> bool {
    let rejected = request_sleep(deep, wake, reject);
    restore_sleep_bias();
    if let Some(saved) = saved {
        saved.restore();
    }
    restore_clock(clock);
    rejected
}

// ---------------------------------------------------------------------------
// Public entry points

/// Enter PMU light sleep.
///
/// # Safety
///
/// The caller owns the PMU, LP timer, LP_AON and TIMG0 calibration, holds a
/// critical section, has stopped the radio and every PLL-dependent activity,
/// and has finished flash/DMA/UART/USB work. For GPIO wake the caller armed
/// the pads. A returned [`Wake`] is not proof of sleep: inspect
/// [`Wake::rejected`] and the causes. With [`crate::SleepMode::TopPowerDown`]
/// the caller must re-initialise state outside the retained set when
/// [`Wake::top_powered_down`] is set.
pub unsafe fn light_sleep(chip: &ChipInfo, request: &Request) -> Result<Wake, SleepError> {
    let plan = plan_light(request, TOP_PD_SUPPORTED)?;
    let clock = current_clock()?;
    arm_timer(request.duration_us)?;
    #[cfg(feature = "top-pd")]
    if plan.top_pd() {
        top::configure_ext1(plan.ext1_sel, plan.ext1_high);
    }

    let result = prepare_and_sleep(chip, &plan, clock);
    disarm_timer();
    result
}

fn prepare_and_sleep(
    chip: &ChipInfo,
    plan: &crate::mode::Plan,
    clock: ClockConfig,
) -> Result<Wake, SleepError> {
    let top_pd = plan.top_pd();
    switch_to_xtal();
    let sar = SarPowerDetect::suspend();
    let words = power_words(plan.pd, slow_clock());
    apply_power(&words);
    let time = match sleep_time(chip.revision) {
        Ok(time) => time,
        Err(error) => {
            sar.resume();
            restore_clock(clock);
            return Err(error);
        }
    };
    apply_digital(true, top_pd);
    unsafe {
        pmu()
            .hp_sleep_hp_regulator0()
            .modify(|_, w| w.hp_sleep_hp_regulator_xpd().set_bit());
    }
    let saved = (plan.pd != 0).then(|| {
        let saved = SavedAnalog::read();
        apply_analog(&h2_light_sleep_analog(plan.pd, &chip.fuses));
        saved
    });
    apply_params(&time.params(words.hp_xpd_xtal));

    #[cfg(feature = "top-pd")]
    if top_pd {
        let outcome = unsafe { top::sleep_top_pd(plan.wake, plan.reject, saved, clock) };
        sar.resume();
        return Ok(read_wake(outcome.rejected, outcome.powered_down));
    }

    let rejected = sleep_retained(false, plan.wake, plan.reject, saved, clock);
    sar.resume();
    Ok(read_wake(rejected, false))
}

fn read_wake(rejected: bool, top_powered_down: bool) -> Wake {
    let pmu = pmu();
    Wake {
        rejected,
        wake_cause: pmu.slp_wakeup_status0().read().bits(),
        reject_cause: pmu.slp_wakeup_status1().read().bits(),
        top_powered_down,
    }
}

/// Enter PMU deep sleep with LP-timer wake; the wake is a chip reset.
/// Returns only when the request is refused before any power state changed.
///
/// # Safety
///
/// As [`light_sleep`]; nothing in RAM survives except RTC/LP memory.
pub unsafe fn deep_sleep(chip: &ChipInfo, duration_us: u64) -> Result<Infallible, SleepError> {
    validate_deep(WAKE_TIMER)?;
    let clock = current_clock()?;
    arm_timer(duration_us)?;
    switch_to_xtal();
    let time = match sleep_time(chip.revision) {
        Ok(time) => time,
        Err(error) => {
            restore_clock(clock);
            disarm_timer();
            return Err(error);
        }
    };
    let words = power_words(DEEP_PD, slow_clock());
    apply_power(&words);
    apply_digital(false, true);
    unsafe {
        pmu()
            .hp_sleep_hp_regulator0()
            .modify(|_, w| w.hp_sleep_hp_regulator_xpd().clear_bit());
    }
    apply_analog(&h2_deep_sleep_analog(&chip.fuses));
    apply_params(&time.params(words.hp_xpd_xtal));
    // Deep sleep has no reject sources (IDF passes none for dslp).
    sleep_retained(true, WAKE_TIMER, 0, None, clock);
    loop {
        core::hint::spin_loop();
    }
}
