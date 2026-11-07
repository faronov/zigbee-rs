//! # ESP32-H2 sleep-floor lab (no Zigbee)
//!
//! Lab/diagnostic image that separates the board's quiescent current from
//! the chip's light-sleep configuration. Never a product image; it owns no
//! persistence and starts no radio. Timing and oscillator flags come from
//! `floor_lab.toml`.
//!
//! ```bash
//! cargo build --release --no-default-features --features floor-lab --bin esp32h2-floor-lab
//! ```
//!
//! * USB host attached: dump the PMU sleep, pad, USB and clock registers,
//!   run one production-flag light sleep, dump again after re-enumeration,
//!   then stay awake and re-dump periodically (the board stays flashable).
//! * No USB host (WSTK/AEM): disable the USB pad exactly as the thermometer
//!   does, print nothing, and cycle A-D (see `floor_lab.toml`), each phase
//!   separated by a CPU-busy marker. Phase D is a deep sleep whose LP timer
//!   wake resets the chip, so the cycle repeats from boot.
//!
//! Reading the trace: deep sleep near the light-sleep floor means the board
//! (always-powered WS2812 on 3V3, LDO) dominates; a light-sleep floor well
//! above deep sleep is chip configuration.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use esp32h2_sleep::{ChipInfo, PowerDown, Request, SleepError, SleepMode, Wake};
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{
    GPIO, IO_MUX, LP_AON, LP_CLKRST, MODEM_LPCON, MODEM_SYSCON, PCR, PMU, USB_DEVICE,
};
use esp_hal::rtc_cntl::Rtc;
use esp_println::println;

mod config {
    include!(concat!(env!("OUT_DIR"), "/floor_lab_config.rs"));
}

/// WS2812 data input on the ESP32-H2-Zero and DevKitM-1 boards (lab only;
/// the board crate does not claim a fitted LED for the unconfirmed board).
const LED_DIN_GPIO: usize = 8;
/// BOOT button, external 10 kOhm pull-up on the H2-Zero.
const BOOT_GPIO: usize = 9;

const POWER_DOWN: PowerDown = PowerDown {
    xtal: config::SLEEP_XTAL_OFF,
    rc_fast: config::SLEEP_RC_FAST_OFF,
};

const LIGHT_MODE: SleepMode = if config::LIGHT_TOP_PD {
    SleepMode::TopPowerDown
} else {
    SleepMode::Retained
};

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let rtc = Rtc::new(peripherals.LPWR);
    let chip = chip_info();
    let mut led_din = peripherals.GPIO8;
    let delay = Delay::new();

    delay.delay_millis(config::USB_HOST_WAIT_MS);
    let host = USB_DEVICE::regs().int_raw().read().sof().bit_is_set();
    if host {
        usb_session(&rtc, &chip, &delay)
    }

    // Battery/bench power: no console from here on.
    USB_DEVICE::regs()
        .conf0()
        .modify(|_, w| w.usb_pad_enable().clear_bit().dp_pullup().clear_bit());

    // A: production flags, LED DIN at its reset configuration.
    let _ = light_sleep(&chip, config::LIGHT_DEFAULT_S);
    marker(&delay);
    // B: LED DIN input buffer, pulls and output driver off.
    isolate(LED_DIN_GPIO);
    let _ = light_sleep(&chip, config::LIGHT_ISOLATED_S);
    marker(&delay);
    // C: LED DIN driven low (pads keep their active state in light sleep).
    {
        let _low = Output::new(led_din.reborrow(), Level::Low, OutputConfig::default());
        let _ = light_sleep(&chip, config::LIGHT_DRIVE_LOW_S);
        marker(&delay);
    }
    isolate(LED_DIN_GPIO);
    // D: deep sleep; the LP timer wake resets the chip.
    // SAFETY: as in `light_sleep`; RAM is lost, the wake is a chip reset.
    let _refused = critical_section::with(|_| unsafe {
        esp32h2_sleep::deep_sleep(&chip, config::DEEP_S * 1_000_000)
    });
    // Refused before any power state changed: stay CPU-busy on the trace.
    loop {
        marker(&delay);
    }
}

/// Host attached: register evidence before and after one real light sleep.
fn usb_session(rtc: &Rtc<'_>, chip: &ChipInfo, delay: &Delay) -> ! {
    println!(
        "[floor] boot: reset {:?}, wake {:?}; chip {:?}; light {:?} {:?}, A/B/C/D {}/{}/{}/{} s",
        esp_hal::system::reset_reason(),
        esp_hal::system::wakeup_cause(),
        chip,
        LIGHT_MODE,
        POWER_DOWN,
        config::LIGHT_DEFAULT_S,
        config::LIGHT_ISOLATED_S,
        config::LIGHT_DRIVE_LOW_S,
        config::DEEP_S,
    );
    dump("boot");
    // Let the console drain before the clocks stop.
    delay.delay_millis(100);
    let before = rtc.time_since_boot();
    let result = light_sleep(chip, 2);
    let slept = rtc.time_since_boot() - before;
    // The host re-enumerates the device after the stopped period.
    delay.delay_millis(3_000);
    println!(
        "[floor] light sleep: {:?}, RTC {} us",
        result,
        slept.as_micros(),
    );
    dump("after-sleep");
    loop {
        delay.delay_millis(config::USB_DUMP_INTERVAL_MS);
        dump("idle");
    }
}

fn light_sleep(chip: &ChipInfo, seconds: u64) -> Result<Wake, SleepError> {
    let request = Request {
        duration_us: seconds * 1_000_000,
        power_down: POWER_DOWN,
        gpio_wake: false,
        ext1: None,
        mode: LIGHT_MODE,
    };
    // SAFETY: no radio, flash writer, DMA or other RTC/PMU user exists in
    // this image; watchdogs are disabled by `esp_hal::init`; interrupts are
    // masked for the whole sequence.
    critical_section::with(|_| unsafe { esp32h2_sleep::light_sleep(chip, &request) })
}

/// eFuse facts for the sleep backend (read before sleeping: eFuse access is
/// flash-resident code).
fn chip_info() -> ChipInfo {
    use esp_hal::efuse::{Efuse, ACTIVE_HP_DBIAS, ACTIVE_LP_DBIAS, DSLP_DBIAS};
    let (blk_major, blk_minor) = Efuse::block_version();
    ChipInfo {
        fuses: esp32h2_sleep::DbiasFuses {
            blk_major,
            blk_minor,
            active_hp: Efuse::read_field_le::<u8>(ACTIVE_HP_DBIAS),
            active_lp: Efuse::read_field_le::<u8>(ACTIVE_LP_DBIAS),
            deep_sleep: Efuse::read_field_le::<u8>(DSLP_DBIAS),
        },
        revision: Efuse::chip_revision(),
    }
}

/// CPU-busy period that separates phases on the current trace.
fn marker(delay: &Delay) {
    delay.delay_millis(config::MARKER_MS);
}

/// Output driver, input buffer and both pulls off for `pin`.
fn isolate(pin: usize) {
    GPIO::regs()
        .enable_w1tc()
        .write(|w| unsafe { w.bits(1 << pin) });
    IO_MUX::regs().gpio(pin).modify(|_, w| {
        w.fun_ie()
            .clear_bit()
            .fun_wpu()
            .clear_bit()
            .fun_wpd()
            .clear_bit()
            .mcu_ie()
            .clear_bit()
            .mcu_wpu()
            .clear_bit()
            .mcu_wpd()
            .clear_bit()
            .mcu_oe()
            .clear_bit()
    });
}

/// Read-only register evidence for the light-sleep audit.
fn dump(tag: &str) {
    let pmu = PMU::regs();
    let aon = LP_AON::regs();
    let pcr = PCR::regs();
    let gpio = GPIO::regs();
    let io = IO_MUX::regs();
    println!(
        "[floor] {} chip v{}.{} ctrl_tick_conf 0x{:08X} slow period 0x{:08X} cpu 0x{:08X} ahb 0x{:08X} sysclk 0x{:08X} mspi 0x{:08X}",
        tag,
        esp_hal::efuse::Efuse::major_chip_version(),
        esp_hal::efuse::Efuse::minor_chip_version(),
        pcr.ctrl_tick_conf().read().bits(),
        aon.store1().read().bits(),
        pcr.cpu_freq_conf().read().bits(),
        pcr.ahb_freq_conf().read().bits(),
        pcr.sysclk_conf().read().bits(),
        pcr.mspi_clk_conf().read().bits(),
    );
    let regs: [(&str, u32); 31] = [
        ("hp_sleep_dig_power", pmu.hp_sleep_dig_power().read().bits()),
        (
            "hp_sleep_icg_hp_func",
            pmu.hp_sleep_icg_hp_func().read().bits(),
        ),
        (
            "hp_sleep_icg_hp_apb",
            pmu.hp_sleep_icg_hp_apb().read().bits(),
        ),
        ("hp_sleep_icg_modem", pmu.hp_sleep_icg_modem().read().bits()),
        (
            "hp_sleep_hp_sys_cntl",
            pmu.hp_sleep_hp_sys_cntl().read().bits(),
        ),
        (
            "hp_sleep_hp_ck_power",
            pmu.hp_sleep_hp_ck_power().read().bits(),
        ),
        ("hp_sleep_bias", pmu.hp_sleep_bias().read().bits()),
        ("hp_sleep_sysclk", pmu.hp_sleep_sysclk().read().bits()),
        (
            "hp_sleep_hp_regulator0",
            pmu.hp_sleep_hp_regulator0().read().bits(),
        ),
        ("hp_sleep_xtal", pmu.hp_sleep_xtal().read().bits()),
        (
            "hp_sleep_lp_regulator0",
            pmu.hp_sleep_lp_regulator0().read().bits(),
        ),
        (
            "hp_sleep_lp_dig_power",
            pmu.hp_sleep_lp_dig_power().read().bits(),
        ),
        (
            "hp_sleep_lp_ck_power",
            pmu.hp_sleep_lp_ck_power().read().bits(),
        ),
        (
            "lp_sleep_lp_regulator0",
            pmu.lp_sleep_lp_regulator0().read().bits(),
        ),
        ("lp_sleep_xtal", pmu.lp_sleep_xtal().read().bits()),
        (
            "lp_sleep_lp_dig_power",
            pmu.lp_sleep_lp_dig_power().read().bits(),
        ),
        (
            "lp_sleep_lp_ck_power",
            pmu.lp_sleep_lp_ck_power().read().bits(),
        ),
        ("lp_sleep_bias", pmu.lp_sleep_bias().read().bits()),
        ("power_pd_top_cntl", pmu.power_pd_top_cntl().read().bits()),
        (
            "power_pd_hpcpu_cntl",
            pmu.power_pd_hpcpu_cntl().read().bits(),
        ),
        (
            "power_pd_hpwifi_cntl",
            pmu.power_pd_hpwifi_cntl().read().bits(),
        ),
        ("power_pd_mem_cntl", pmu.power_pd_mem_cntl().read().bits()),
        ("power_hp_pad", pmu.power_hp_pad().read().bits()),
        ("power_vdd_spi_cntl", pmu.power_vdd_spi_cntl().read().bits()),
        (
            "hp_active_hp_sys_cntl",
            pmu.hp_active_hp_sys_cntl().read().bits(),
        ),
        ("usb_conf0", USB_DEVICE::regs().conf0().read().bits()),
        ("lp_aon_usb", aon.usb().read().bits()),
        ("lp_aon_gpio_hold0", aon.gpio_hold0().read().bits()),
        (
            "lp_clkrst_fosc_cntl",
            LP_CLKRST::regs().fosc_cntl().read().bits(),
        ),
        (
            "modem_lpcon_clk_conf",
            MODEM_LPCON::regs().clk_conf().read().bits(),
        ),
        (
            "modem_syscon_clk_conf",
            MODEM_SYSCON::regs().clk_conf().read().bits(),
        ),
    ];
    for (name, value) in regs {
        println!("[floor] {} {} 0x{:08X}", tag, name, value);
    }
    for pin in [LED_DIN_GPIO, BOOT_GPIO] {
        println!(
            "[floor] {} io_mux gpio{} 0x{:08X} out_sel 0x{:08X}",
            tag,
            pin,
            io.gpio(pin).read().bits(),
            gpio.func_out_sel_cfg(pin).read().bits(),
        );
    }
    println!(
        "[floor] {} gpio enable 0x{:08X} out 0x{:08X} in 0x{:08X}",
        tag,
        gpio.enable().read().bits(),
        gpio.out().read().bits(),
        gpio.in_().read().bits(),
    );
}
