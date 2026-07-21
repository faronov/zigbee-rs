//! # Zigbee-RS EFR32MG1P Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for EFR32MG1P-based boards.
//! Pure-Rust radio driver — no RAIL library, no GSDK, no binary blobs.
//!
//! # Hardware
//! - EFR32MG1P (256KB flash, 31KB SRAM), ARM Cortex-M4F @ 38.4 MHz
//! - 2.4 GHz radio with IEEE 802.15.4 + BLE support
//! - Common boards: IKEA TRÅDFRI, Thunderboard Sense, BRD4151A
//!
//! # Features
//! - Auto-join on boot (no button required)
//! - Sleepy End Device: poll parent for indirect frames
//! - Fast poll (250ms) during ZHA interview, slow poll (10s) normal
//! - LED status: triple-blink boot, double-blink joining, solid joined
//! - Button: short press = toggle join/leave, long press = factory reset
//! - Device_annce retries for reliable coordinator discovery
//!
//! # Build
//! ```bash
//! cargo build --release
//! ```

#![no_std]
#![no_main]
#![cfg_attr(
    not(any(feature = "diag-nv", feature = "diag-em2")),
    feature(impl_trait_in_assoc_type)
)]

#[cfg(not(any(
    feature = "sensor",
    feature = "diag-beacon",
    feature = "diag-join",
    feature = "diag-sht",
    feature = "diag-nv",
    feature = "diag-em2",
    feature = "diag-rtcc-time",
    feature = "diag-radio-em2"
)))]
compile_error!(
    "enable exactly one of `sensor`, `diag-join`, `diag-beacon`, `diag-sht`, `diag-nv`, `diag-em2`, `diag-rtcc-time`, or `diag-radio-em2`"
);

#[cfg(any(
    all(feature = "sensor", feature = "diag-beacon"),
    all(feature = "sensor", feature = "diag-join"),
    all(feature = "sensor", feature = "diag-sht"),
    all(feature = "sensor", feature = "diag-nv"),
    all(feature = "sensor", feature = "diag-em2"),
    all(feature = "sensor", feature = "diag-rtcc-time"),
    all(feature = "sensor", feature = "diag-radio-em2"),
    all(feature = "diag-beacon", feature = "diag-join"),
    all(feature = "diag-beacon", feature = "diag-sht"),
    all(feature = "diag-beacon", feature = "diag-nv"),
    all(feature = "diag-beacon", feature = "diag-em2"),
    all(feature = "diag-beacon", feature = "diag-rtcc-time"),
    all(feature = "diag-beacon", feature = "diag-radio-em2"),
    all(feature = "diag-join", feature = "diag-sht"),
    all(feature = "diag-join", feature = "diag-nv"),
    all(feature = "diag-join", feature = "diag-em2"),
    all(feature = "diag-join", feature = "diag-rtcc-time"),
    all(feature = "diag-join", feature = "diag-radio-em2"),
    all(feature = "diag-sht", feature = "diag-nv"),
    all(feature = "diag-sht", feature = "diag-em2"),
    all(feature = "diag-sht", feature = "diag-rtcc-time"),
    all(feature = "diag-sht", feature = "diag-radio-em2"),
    all(feature = "diag-nv", feature = "diag-em2"),
    all(feature = "diag-nv", feature = "diag-rtcc-time"),
    all(feature = "diag-nv", feature = "diag-radio-em2"),
    all(feature = "diag-em2", feature = "diag-rtcc-time"),
    all(feature = "diag-em2", feature = "diag-radio-em2"),
    all(feature = "diag-rtcc-time", feature = "diag-radio-em2"),
))]
compile_error!(
    "EFR32 application profiles are mutually exclusive"
);

#[cfg(feature = "stubs")]
mod stubs;

#[cfg(any(
    feature = "sensor",
    feature = "diag-beacon",
    feature = "diag-join",
    feature = "diag-sht",
    feature = "diag-nv",
    feature = "diag-rtcc-time",
    feature = "diag-radio-em2"
))]
mod time_driver;
mod vectors;

use core::mem::MaybeUninit;
use cortex_m as _;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;

#[cfg(feature = "trace")]
struct RttLogger;

#[cfg(feature = "trace")]
static LOGGER: RttLogger = RttLogger;

#[cfg(feature = "trace")]
impl log::Log for RttLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        rtt_target::rprintln!("[{}] {}", record.level(), record.args());
    }

    fn flush(&self) {}
}

fn init_logging() {
    #[cfg(any(
        feature = "diag-beacon",
        feature = "diag-em2",
        feature = "diag-rtcc-time",
        feature = "diag-radio-em2"
    ))]
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 4096,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 16,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    #[cfg(not(any(
        feature = "diag-beacon",
        feature = "diag-em2",
        feature = "diag-rtcc-time",
        feature = "diag-radio-em2"
    )))]
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 256,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 16,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    #[cfg(feature = "trace")]
    {
        let _ = log::set_logger(&LOGGER);
        log::set_max_level(log::LevelFilter::Trace);
    }
}

#[repr(C)]
struct FaultLog {
    hardfault_magic: u32,
    hardfault_pc: u32,
    hardfault_lr: u32,
    hardfault_xpsr: u32,
    hardfault_msp: u32,
    hardfault_r0: u32,
    hardfault_r12: u32,
    panic_magic: u32,
    panic_line: u32,
    panic_column: u32,
    panic_file_ptr: u32,
    panic_file_len: u32,
}

#[unsafe(link_section = ".uninit.fault_log")]
static mut FAULT_LOG: MaybeUninit<FaultLog> = MaybeUninit::uninit();

#[inline(always)]
unsafe fn fault_log_mut() -> *mut FaultLog {
    core::ptr::addr_of_mut!(FAULT_LOG).cast::<FaultLog>()
}

#[cfg(feature = "sed-diag")]
#[repr(C)]
struct SedDiag {
    magic: u32,
    version: u32,
    event_seq: u32,
    factory_new_seq: u32,
    start_ok_seq: u32,
    interview_seq: u32,
    first_em2_seq: u32,
    em2_wakes: u32,
    identify_rx_seq: u32,
    identify_complete_seq: u32,
    network_address: u32,
    configured_clusters: u32,
    last_em2_ticks: u32,
    flags: u32,
}

#[cfg(feature = "sed-diag")]
#[unsafe(link_section = ".uninit.sed_diag")]
static mut SED_DIAG: MaybeUninit<SedDiag> = MaybeUninit::uninit();

#[cfg(feature = "sed-diag")]
#[inline(always)]
unsafe fn sed_diag_mut() -> *mut SedDiag {
    core::ptr::addr_of_mut!(SED_DIAG).cast::<SedDiag>()
}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_init() {
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).magic).read_volatile() == 0x5345_4444
            && core::ptr::addr_of!((*diag).version).read_volatile() == 1
        {
            return;
        }
        diag.write_volatile(SedDiag {
            magic: 0x5345_4444,
            version: 1,
            event_seq: 1,
            factory_new_seq: 0,
            start_ok_seq: 0,
            interview_seq: 0,
            first_em2_seq: 0,
            em2_wakes: 0,
            identify_rx_seq: 0,
            identify_complete_seq: 0,
            network_address: 0xFFFF,
            configured_clusters: 0,
            last_em2_ticks: 0,
            flags: 0,
        });
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_init() {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
unsafe fn sed_diag_next_seq(diag: *mut SedDiag) -> u32 {
    let next = unsafe {
        core::ptr::addr_of!((*diag).event_seq)
            .read_volatile()
            .wrapping_add(1)
    };
    unsafe { core::ptr::addr_of_mut!((*diag).event_seq).write_volatile(next) };
    next
}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_factory_new(factory_new: bool) {
    if !factory_new {
        return;
    }
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).factory_new_seq).read_volatile() != 0 {
            return;
        }
        let seq = sed_diag_next_seq(diag);
        core::ptr::addr_of_mut!((*diag).factory_new_seq).write_volatile(seq);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | 1;
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_factory_new(_factory_new: bool) {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_start_ok(network_address: u16) {
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).start_ok_seq).read_volatile() != 0 {
            return;
        }
        let seq = sed_diag_next_seq(diag);
        core::ptr::addr_of_mut!((*diag).start_ok_seq).write_volatile(seq);
        core::ptr::addr_of_mut!((*diag).network_address)
            .write_volatile(network_address as u32);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | (1 << 1);
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_start_ok(_network_address: u16) {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_interview(configured_clusters: usize) {
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).interview_seq).read_volatile() != 0 {
            return;
        }
        let seq = sed_diag_next_seq(diag);
        core::ptr::addr_of_mut!((*diag).interview_seq).write_volatile(seq);
        core::ptr::addr_of_mut!((*diag).configured_clusters)
            .write_volatile(configured_clusters as u32);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | (1 << 2);
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_interview(_configured_clusters: usize) {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_em2_wake(elapsed_ticks: u32) {
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).identify_rx_seq).read_volatile() != 0 {
            return;
        }
        let seq = sed_diag_next_seq(diag);
        if core::ptr::addr_of!((*diag).first_em2_seq).read_volatile() == 0 {
            core::ptr::addr_of_mut!((*diag).first_em2_seq).write_volatile(seq);
        }
        let wakes = core::ptr::addr_of!((*diag).em2_wakes)
            .read_volatile()
            .wrapping_add(1);
        core::ptr::addr_of_mut!((*diag).em2_wakes).write_volatile(wakes);
        core::ptr::addr_of_mut!((*diag).last_em2_ticks).write_volatile(elapsed_ticks);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | (1 << 3);
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_em2_wake(_elapsed_ticks: u32) {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_identify_received() {
    unsafe {
        let diag = sed_diag_mut();
        if core::ptr::addr_of!((*diag).identify_complete_seq).read_volatile() != 0 {
            return;
        }
        let seq = sed_diag_next_seq(diag);
        core::ptr::addr_of_mut!((*diag).identify_rx_seq).write_volatile(seq);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | (1 << 4);
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_identify_received() {}

#[cfg(feature = "sed-diag")]
#[inline(always)]
fn sed_diag_identify_complete() {
    unsafe {
        let diag = sed_diag_mut();
        let seq = sed_diag_next_seq(diag);
        core::ptr::addr_of_mut!((*diag).identify_complete_seq).write_volatile(seq);
        let flags = core::ptr::addr_of!((*diag).flags).read_volatile() | (1 << 5);
        core::ptr::addr_of_mut!((*diag).flags).write_volatile(flags);
    }
}

#[cfg(not(feature = "sed-diag"))]
#[inline(always)]
fn sed_diag_identify_complete() {}

// Custom HardFault handler that saves faulting PC to known RAM location
// so we can read it via J-Link after the crash.
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe {
        let msp: u32;
        core::arch::asm!("mrs {}, msp", out(reg) msp);
        let log = fault_log_mut();
        core::ptr::addr_of_mut!((*log).hardfault_magic).write_volatile(0xDEAD_BEEF);
        core::ptr::addr_of_mut!((*log).hardfault_pc).write_volatile(ef.pc());
        core::ptr::addr_of_mut!((*log).hardfault_lr).write_volatile(ef.lr());
        core::ptr::addr_of_mut!((*log).hardfault_xpsr).write_volatile(ef.xpsr());
        core::ptr::addr_of_mut!((*log).hardfault_msp).write_volatile(msp);
        core::ptr::addr_of_mut!((*log).hardfault_r0).write_volatile(ef.r0());
        core::ptr::addr_of_mut!((*log).hardfault_r12).write_volatile(ef.r12());
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    unsafe {
        let log = fault_log_mut();
        core::ptr::addr_of_mut!((*log).panic_magic).write_volatile(0x5041_4E49); // "PANI"
        if let Some(location) = info.location() {
            core::ptr::addr_of_mut!((*log).panic_line).write_volatile(location.line());
            core::ptr::addr_of_mut!((*log).panic_column).write_volatile(location.column());
            core::ptr::addr_of_mut!((*log).panic_file_ptr)
                .write_volatile(location.file().as_ptr() as u32);
            core::ptr::addr_of_mut!((*log).panic_file_len)
                .write_volatile(location.file().len() as u32);
        } else {
            core::ptr::addr_of_mut!((*log).panic_line).write_volatile(0);
            core::ptr::addr_of_mut!((*log).panic_column).write_volatile(0);
            core::ptr::addr_of_mut!((*log).panic_file_ptr).write_volatile(0);
            core::ptr::addr_of_mut!((*log).panic_file_len).write_volatile(0);
        }
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[cfg(any(feature = "sensor", feature = "diag-nv"))]
use efr32mg1_tradfri::storage;
#[cfg(any(
    feature = "diag-em2",
    feature = "diag-rtcc-time",
    feature = "diag-radio-em2",
    feature = "sed"
))]
use efr32mg1_hal::pm;
#[cfg(feature = "sensor")]
use efr32mg1_tradfri::storage::SecurityStore;
#[cfg(feature = "sensor")]
use efr32mg1_tradfri::Button;
use efr32mg1_tradfri::Led;
#[cfg(feature = "sensor")]
use embassy_executor::Spawner;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use embassy_futures::select;
#[cfg(any(feature = "sensor", feature = "diag-join", feature = "diag-rtcc-time"))]
use embassy_time::Instant;
#[cfg(not(any(feature = "diag-nv", feature = "diag-em2")))]
use embassy_time::{Duration, Timer};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use static_cell::StaticCell;

#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_aps::PROFILE_HOME_AUTOMATION;
#[cfg(any(
    feature = "diag-beacon",
    feature = "diag-join",
    feature = "diag-radio-em2"
))]
use zigbee_mac::MacDriver;
#[cfg(any(
    feature = "sensor",
    feature = "diag-beacon",
    feature = "diag-join",
    feature = "diag-radio-em2"
))]
use zigbee_mac::efr32::Efr32Mac;
#[cfg(any(feature = "diag-beacon", feature = "diag-radio-em2"))]
use zigbee_mac::frames::build_beacon_request;
#[cfg(any(feature = "diag-beacon", feature = "diag-radio-em2"))]
use zigbee_mac::pib::{PibAttribute, PibValue};
#[cfg(any(feature = "diag-beacon", feature = "diag-join"))]
use zigbee_mac::primitives::{MlmeScanRequest, ScanType};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_nwk::DeviceType;
#[cfg(feature = "sensor")]
use zigbee_runtime::UserAction;
#[cfg(feature = "diag-nv")]
use zigbee_runtime::nv_storage::{NvItemId, NvStorage};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::event_loop::{StackEvent, TickResult};
#[cfg(feature = "sensor")]
use zigbee_runtime::event_loop::StartError;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::power::PowerMode;
#[cfg(feature = "sensor")]
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
#[cfg(any(feature = "sensor", feature = "diag-beacon", feature = "diag-join"))]
use zigbee_types::ChannelMask;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_zcl::clusters::basic::PowerSource;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::humidity::HumidityCluster;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::temperature::TemperatureCluster;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_zcl::{ClusterId, DeviceId};

#[cfg(any(feature = "sensor", feature = "diag-join"))]
const JOIN_RETRY_SECS: u64 = 15;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_POLL_MS: u64 = 250;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_STEADY_POLL_MS: u64 = 1_000;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_FAST_POLL_SECS: u64 = 90;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_ANNCE_RETRY_SECS: u64 = 2;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_ANNCE_RETRIES: u8 = 5;
#[cfg(feature = "sensor")]
const REPORT_INTERVAL_SECS: u64 = 60;
#[cfg(feature = "sensor")]
const FAST_POLL_MS: u64 = 250;
#[cfg(feature = "sensor")]
const SLOW_POLL_SECS: u64 = 30;
#[cfg(feature = "sensor")]
const FAST_POLL_DURATION_SECS: u64 = 120;
#[cfg(all(feature = "sensor", feature = "sed"))]
const RESTORED_FAST_POLL_SECS: u64 = 60;
#[cfg(feature = "sensor")]
const EXPECTED_REPORT_CLUSTERS: usize = 3; // PowerConfig + Temp + Humidity
#[cfg(feature = "sensor")]
const SENSOR_JOIN_CHANNEL_MASK: ChannelMask = ChannelMask(1u32 << 15);
#[cfg(all(feature = "sensor", not(feature = "sed")))]
const SENSOR_POWER_MODE: PowerMode = PowerMode::AlwaysOn;
#[cfg(all(feature = "sensor", feature = "sed"))]
const SENSOR_POWER_MODE: PowerMode = PowerMode::Sleepy {
    poll_interval_ms: (SLOW_POLL_SECS * 1_000) as u32,
    wake_duration_ms: FAST_POLL_MS as u32,
};
#[cfg(any(feature = "diag-beacon", feature = "diag-join"))]
const DIAG_SCAN_DURATION: u8 = 4;
#[cfg(any(feature = "diag-beacon", feature = "diag-radio-em2"))]
const DIAG_SCAN_CHANNEL: u8 = 15;
#[cfg(feature = "diag-beacon")]
const DIAG_SCAN_MASK_2GHZ: u32 = 1u32 << DIAG_SCAN_CHANNEL;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_MASK_2GHZ: ChannelMask = ChannelMask::ALL_2_4GHZ;
#[cfg(any(feature = "diag-beacon", feature = "diag-radio-em2"))]
const DIAG_TX_TEST_SEQ: u8 = 0xA5;
#[cfg(feature = "diag-radio-em2")]
const DIAG_RADIO_EM2_ITERATIONS: u8 = 5;
#[cfg(feature = "diag-radio-em2")]
const DIAG_RADIO_EM2_SLEEP_MS: u32 = 500;
#[cfg(feature = "diag-radio-em2")]
const DIAG_RADIO_EM2_TOLERANCE_PERCENT: u32 = 10;
#[cfg(feature = "diag-sht")]
const DIAG_SHT_SAMPLE_INTERVAL_MS: u64 = 2_000;

// ── TRÅDFRI board startup and GPIO ──────────────────────────────

static BOARD_LED: Led = Led::new();
#[cfg(feature = "sensor")]
static BOARD_BUTTON: Button = Button::new();

fn led_on() {
    BOARD_LED.on();
}
fn led_off() {
    BOARD_LED.off();
}

fn platform_init(profile: &str) {
    init_logging();
    // GPIO is usable from reset HFRCO, so clock failures can still be marked.
    BOARD_LED.init();
    led_off();
    #[cfg(feature = "sensor")]
    BOARD_BUTTON.init();

    match efr32mg1_tradfri::init_clocks() {
        Ok(()) => {
            rtt_target::rprintln!(
                "[EFR32][{}] CLOCK_READY hclk={} ctune={}",
                profile,
                efr32mg1_tradfri::HCLK_HZ,
                efr32mg1_tradfri::HFXO_CTUNE
            );
        }
        Err(error) => {
            rtt_target::rprintln!("[EFR32][{}] CLOCK_FATAL {:?}", profile, error);
            loop {
                led_on();
                busy_delay();
                led_off();
                busy_delay();
            }
        }
    }
    #[cfg(not(feature = "diag-em2"))]
    time_driver::init();
}

#[inline(never)]
fn busy_delay() {
    for _ in 0..750_000 {
        cortex_m::asm::nop();
    }
}

#[cfg(feature = "sensor")]
async fn boot_signal() {
    platform_init("sensor");

    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
}

#[cfg(feature = "sensor")]
fn cluster_refs<'a>(
    temp_cluster: &'a mut TemperatureCluster,
    hum_cluster: &'a mut HumidityCluster,
    power_cluster: &'a mut PowerConfigCluster,
) -> [ClusterRef<'a>; 3] {
    [
        ClusterRef {
            endpoint: 1,
            cluster: temp_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: hum_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: power_cluster,
        },
    ]
}

#[cfg(feature = "diag-join")]
fn join_cluster_refs<'a>(
) -> [ClusterRef<'a>; 0] {
    []
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_tick(
    device: &mut ZigbeeDevice<Efr32Mac>,
    elapsed_secs: u16,
) -> TickResult {
    let mut clusters = join_cluster_refs();
    device.tick(elapsed_secs, &mut clusters).await
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_process_incoming(
    device: &mut ZigbeeDevice<Efr32Mac>,
    indication: &zigbee_mac::McpsDataIndication,
) -> Option<StackEvent> {
    let mut clusters = join_cluster_refs();
    device.process_incoming(indication, &mut clusters).await
}

#[cfg(feature = "diag-join")]
#[derive(Copy, Clone)]
struct DiagJoinHandles {
    device: *mut ZigbeeDevice<Efr32Mac>,
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_tick_handles(handles: DiagJoinHandles, elapsed_secs: u16) -> TickResult {
    unsafe { diag_join_tick(&mut *handles.device, elapsed_secs).await }
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_process_incoming_handles(
    handles: DiagJoinHandles,
    indication: &zigbee_mac::McpsDataIndication,
) -> Option<StackEvent> {
    unsafe { diag_join_process_incoming(&mut *handles.device, indication).await }
}

// ── Main ────────────────────────────────────────────────────────

#[cfg(feature = "sensor")]
#[inline(never)]
fn persistence_failure(error: SecurityStoreError) -> ! {
    rtt_target::rprintln!("[EFR32] SECURITY_STORAGE_FATAL error={:?}", error);
    loop {
        cortex_m::asm::nop();
    }
}

#[cfg(feature = "sensor")]
struct SensorApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    security_store: &'static mut SecurityStore,
    sht: zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c>,
    temp_cluster: &'static mut TemperatureCluster,
    hum_cluster: &'static mut HumidityCluster,
    power_cluster: &'static mut PowerConfigCluster,
    last_report: Instant,
    last_tick: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: Instant,
    was_fast_polling: bool,
    was_identifying: bool,
    interview_done: bool,
    #[allow(dead_code)]
    button_was_pressed: bool,
    needs_checkpoint: bool,
    needs_bootstrap_join: bool,
    awaiting_initial_configuration: bool,
    #[cfg(feature = "sed")]
    restoring_commissioned_state: bool,
    #[cfg(feature = "sed-migrate")]
    resuming_pending_rejoin: bool,
}

#[cfg(feature = "sensor")]
impl SensorApp {
    fn new(
        device: &'static mut ZigbeeDevice<Efr32Mac>,
        security_store: &'static mut SecurityStore,
        sht: zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c>,
        temp_cluster: &'static mut TemperatureCluster,
        hum_cluster: &'static mut HumidityCluster,
        power_cluster: &'static mut PowerConfigCluster,
    ) -> Self {
        let now = Instant::now();
        let joined = device.is_joined();
        Self {
            device,
            security_store,
            sht,
            temp_cluster,
            hum_cluster,
            power_cluster,
            last_report: now,
            last_tick: now,
            fast_poll_until: if joined {
                now + Duration::from_secs(FAST_POLL_DURATION_SECS)
            } else {
                now
            },
            last_rejoin_attempt: now,
            rejoin_count: 0,
            annce_retries_left: 0,
            last_annce: now,
            was_fast_polling: joined,
            was_identifying: false,
            interview_done: false,
            button_was_pressed: false,
            needs_checkpoint: false,
            needs_bootstrap_join: !joined,
            awaiting_initial_configuration: false,
            #[cfg(feature = "sed")]
            restoring_commissioned_state: false,
            #[cfg(feature = "sed-migrate")]
            resuming_pending_rejoin: false,
        }
    }

    #[inline(always)]
    fn reset_post_join_state(&mut self) {
        let now = Instant::now();
        self.fast_poll_until = now + Duration::from_secs(FAST_POLL_DURATION_SECS);
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.annce_retries_left = 5;
        self.last_annce = now;
        self.interview_done = false;
        self.was_identifying = false;
        self.was_fast_polling = true;
        led_on();
    }

    fn update_interview_state(&mut self) {
        if self.interview_done {
            return;
        }

        let configured = self.device.configured_cluster_count(1);
        if configured < EXPECTED_REPORT_CLUSTERS {
            return;
        }

        sed_diag_interview(configured);
        rtt_target::rprintln!(
            "[EFR32][sed] INTERVIEW_CONFIGURED clusters={}/{}",
            configured,
            EXPECTED_REPORT_CLUSTERS
        );
        log::info!(
            "[EFR32] Local endpoint ready: {}/{} clusters configured",
            configured,
            EXPECTED_REPORT_CLUSTERS
        );
        self.fast_poll_until = Instant::now() + Duration::from_secs(5);
        self.interview_done = true;
        self.awaiting_initial_configuration = false;
        led_off();
    }

    #[cfg(feature = "sed")]
    fn shorten_restored_fast_poll(&mut self) {
        self.fast_poll_until = Instant::now() + Duration::from_secs(RESTORED_FAST_POLL_SECS);
    }

    #[inline(always)]
    fn checkpoint_security(&mut self) {
        if let Err(error) = self
            .device
            .refresh_security_state(&mut *self.security_store)
        {
            persistence_failure(error);
        }
    }

    #[inline(always)]
    async fn factory_reset(&mut self) {
        if let Err(error) = self
            .device
            .factory_reset_with_security_store(&mut *self.security_store)
            .await
        {
            match error {
                StartError::PersistenceFailed(error) => persistence_failure(error),
                other => rtt_target::rprintln!("[EFR32] Factory reset failed: {:?}", other),
            }
        }
    }

    #[inline(always)]
    async fn secure_rejoin(&mut self) -> bool {
        match self
            .device
            .secure_rejoin_with_security_store(&mut *self.security_store)
            .await
        {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(error) => {
                rtt_target::rprintln!("[EFR32] Secure rejoin failed: {:?}", error);
                return false;
            }
        }
        self.reset_post_join_state();
        self.needs_bootstrap_join = false;
        self.needs_checkpoint = true;
        rtt_target::rprintln!("[EFR32] Secure rejoin succeeded");
        log::info!("[EFR32] Secure rejoin succeeded");
        true
    }

    #[inline(always)]
    async fn bootstrap_join(&mut self, reason: &'static str) -> bool {
        self.last_rejoin_attempt = Instant::now();
        self.rejoin_count = self.rejoin_count.wrapping_add(1);
        rtt_target::rprintln!("[EFR32] {} join (attempt {})", reason, self.rejoin_count);
        log::info!("[EFR32] {} join (attempt {})", reason, self.rejoin_count);

        let restored_state = match self.security_store.load() {
            Ok(state) => state,
            Err(error) => persistence_failure(error),
        };
        let had_commissioned_state = restored_state.is_some_and(|state| state.commissioned);
        self.awaiting_initial_configuration = !had_commissioned_state;
        sed_diag_factory_new(!had_commissioned_state);
        #[cfg(feature = "sed")]
        {
            self.restoring_commissioned_state = had_commissioned_state;
        }
        #[cfg(feature = "sed-migrate")]
        {
            self.resuming_pending_rejoin =
                restored_state.is_some_and(|state| state.rejoin_pending);
        }
        rtt_target::rprintln!(
            "[EFR32] {}",
            if had_commissioned_state {
                "RESTORE_STATE_FOUND"
            } else {
                "FACTORY_NEW_STATE"
            }
        );
        rtt_target::rprintln!("[EFR32] {} start_or_resume...", reason);
        match self
            .device
            .start_or_resume_with_security_store(&mut *self.security_store)
            .await
        {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(error) => {
                rtt_target::rprintln!("[EFR32] {} start_or_resume=err {:?}", reason, error);
                let rejoin = self.device.bdb().zdo().nwk().rejoin_diagnostics();
                let steering = self.device.bdb().steering_diagnostics();
                let scan = self
                    .device
                    .bdb()
                    .zdo()
                    .nwk()
                    .mac()
                    .scan_diagnostics();
                let (cca_rssi, cca_status, cca_clear, cca_state, cca_samples) =
                    self.device.bdb().zdo().nwk().mac().cca_snapshot();
                rtt_target::rprintln!(
                    "[EFR32] SCAN_DIAG tx={} tx_fail={} rx={} rx_err={} beacons={}",
                    scan.tx_attempts,
                    scan.tx_failures,
                    scan.rx_frames,
                    scan.rx_errors,
                    scan.beacons
                );
                rtt_target::rprintln!(
                    "[EFR32] CCA_DIAG samples={} rssi={} status=0x{:08X} clear={} rac={}",
                    cca_samples,
                    cca_rssi,
                    cca_status,
                    cca_clear,
                    cca_state
                );
                rtt_target::rprintln!(
                    "[EFR32] REJOIN_DIAG stage={} candidates={} tx={} noack={} cca={} other={} polls={} rx={} status=0x{:02X} parent=0x{:04X}",
                    rejoin.stage,
                    rejoin.candidate_attempts,
                    rejoin.tx_attempts,
                    rejoin.no_ack_failures,
                    rejoin.channel_access_failures,
                    rejoin.other_tx_failures,
                    rejoin.poll_attempts,
                    rejoin.rx_frames,
                    rejoin.last_status,
                    rejoin.last_parent
                );
                rtt_target::rprintln!(
                    "[EFR32] STEERING_DIAG stage={} scans={} networks={} closed={} joins={} joined={} status=0x{:02X} parent=0x{:04X} polls={} data={} poll_err={} frame_len={}",
                    steering.stage as u8,
                    steering.scan_requests,
                    steering.networks_discovered,
                    steering.permit_closed_rejects,
                    steering.join_attempts,
                    steering.join_successes,
                    steering.last_join_status,
                    steering.parent_address,
                    steering.poll_attempts,
                    steering.poll_data_frames,
                    steering.poll_errors,
                    steering.last_frame_len
                );
                #[cfg(feature = "sed-migrate")]
                loop {
                    led_on();
                    cortex_m::asm::nop();
                }
                #[cfg(not(feature = "sed-migrate"))]
                return false;
            }
        }

        let nib = self.device.bdb().zdo().nwk().nib();
        sed_diag_start_ok(nib.network_address.0);
        rtt_target::rprintln!(
            "[EFR32] {} start_or_resume=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            reason,
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );
        log::info!(
            "[EFR32] {} start_or_resume=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            reason,
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );

        #[cfg(feature = "sed-migrate")]
        if !self.resuming_pending_rejoin {
            rtt_target::rprintln!("[EFR32] SECURE_REJOIN_GATE_BEGIN");
            match self
                .device
                .secure_rejoin_with_security_store(&mut *self.security_store)
                .await
            {
                Ok(address) => {
                    rtt_target::rprintln!(
                        "[EFR32] SECURE_REJOIN_GATE_PASS addr=0x{:04X}",
                        address
                    );
                }
                Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
                Err(error) => {
                    rtt_target::rprintln!("[EFR32] SECURE_REJOIN_GATE_FAIL {:?}", error);
                    loop {
                        led_on();
                        cortex_m::asm::nop();
                    }
                }
            }
        }

        self.checkpoint_security();
        let _ = self.device.send_device_annce().await;
        self.checkpoint_security();
        self.reset_post_join_state();
        #[cfg(feature = "sed")]
        if self.restoring_commissioned_state {
            self.shorten_restored_fast_poll();
            self.restoring_commissioned_state = false;
        }
        #[cfg(feature = "sed-migrate")]
        {
            self.resuming_pending_rejoin = false;
        }
        self.needs_bootstrap_join = false;
        self.needs_checkpoint = true;
        true
    }

    #[inline(always)]
    async fn run_first_tick(&mut self) {
        log::info!("[EFR32] First tick...");
        if self.needs_bootstrap_join && !self.device.is_joined() {
            let _ = self.bootstrap_join("startup").await;
        }
        let mut clusters = cluster_refs(
            &mut *self.temp_cluster,
            &mut *self.hum_cluster,
            &mut *self.power_cluster,
        );
        let tick_result = match self
            .device
            .tick_with_security_store(0, &mut clusters, &mut *self.security_store)
            .await
        {
            Ok(result) => result,
            Err(error) => persistence_failure(error),
        };
        if let TickResult::Event(ref e) = tick_result {
            log::info!("[EFR32] First tick event: {:?}", core::mem::discriminant(e));
            if log_event(e) {
                self.checkpoint_security();
                rtt_target::rprintln!("[EFR32] Security checkpointed (first tick)");
            }
        }
        log::info!(
            "[EFR32] First tick done, joined={}",
            self.device.is_joined()
        );

        if !self.awaiting_initial_configuration {
            setup_default_reporting(&mut *self.device);
        }
        self.power_cluster.set_battery_voltage(30);
        self.power_cluster.set_battery_percentage(100 * 2);
        self.sample_sht().await;
        self.last_report = Instant::now();

        if self.device.is_joined() {
            log::info!("[EFR32] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
            self.reset_post_join_state();
        }
    }

    #[inline(always)]
    fn update_fast_poll_window(&mut self, now: Instant) -> u64 {
        let in_fast_poll = self.awaiting_initial_configuration
            || self.device.is_identifying(1)
            || now < self.fast_poll_until;
        if self.was_fast_polling && !in_fast_poll {
            let cfg = self.device.configured_cluster_count(1);
            log::info!(
                "[EFR32] Fast poll OFF - {}/{} clusters configured",
                cfg,
                EXPECTED_REPORT_CLUSTERS
            );
            self.was_fast_polling = false;
            if !self.interview_done {
                led_off();
            }
        } else if in_fast_poll {
            self.was_fast_polling = true;
        }

        if in_fast_poll {
            FAST_POLL_MS
        } else {
            SLOW_POLL_SECS * 1000
        }
    }

    #[inline(always)]
    async fn request_join_retry(&mut self, reason: &'static str) {
        if self.device.secure_rejoin_pending() {
            self.last_rejoin_attempt = Instant::now();
            let mut clusters = cluster_refs(
                &mut *self.temp_cluster,
                &mut *self.hum_cluster,
                &mut *self.power_cluster,
            );
            if let Err(error) = self
                .device
                .tick_with_security_store(0, &mut clusters, &mut *self.security_store)
                .await
            {
                persistence_failure(error);
            }
            return;
        }
        let _ = self.bootstrap_join(reason).await;
    }

    #[inline(always)]
    #[allow(dead_code)]
    async fn handle_button_press(&mut self) {
        let mut held_long = false;
        let press_start = Instant::now();
        while BOARD_BUTTON.is_pressed() {
            if press_start.elapsed().as_secs() >= 3 {
                held_long = true;
                break;
            }
            Timer::after(Duration::from_millis(50)).await;
        }

        if held_long {
            log::info!("[EFR32] FACTORY RESET");
            self.factory_reset().await;
            log::info!("[EFR32] NV cleared - rebooting");
            for _ in 0..5u8 {
                led_on();
                Timer::after(Duration::from_millis(100)).await;
                led_off();
                Timer::after(Duration::from_millis(100)).await;
            }
            cortex_m::peripheral::SCB::sys_reset();
        }

        log::info!(
            "[EFR32] Button -> {}",
            if self.device.is_joined() {
                "leave"
            } else {
                "join"
            }
        );
        self.device.user_action(UserAction::Toggle);
        let mut clusters = cluster_refs(
            &mut *self.temp_cluster,
            &mut *self.hum_cluster,
            &mut *self.power_cluster,
        );
        let tick_result = match self
            .device
            .tick_with_security_store(0, &mut clusters, &mut *self.security_store)
            .await
        {
            Ok(result) => result,
            Err(error) => persistence_failure(error),
        };
        if let TickResult::Event(ref e) = tick_result {
            match e {
                StackEvent::Joined { .. } => {
                    log_event(e);
                    self.reset_post_join_state();
                    self.needs_checkpoint = true;
                }
                StackEvent::Left => {
                    log_event(e);
                    self.factory_reset().await;
                    log::info!("[EFR32] NV cleared");
                }
                _ => {
                    log_event(e);
                }
            }
        }
        Timer::after(Duration::from_millis(300)).await;
    }

    #[inline(always)]
    #[allow(dead_code)]
    async fn handle_button_edge(&mut self) {
        // Disabled in the sensor profile: PF7 is floating on this board
        // configuration and can otherwise trigger false long-press resets.
    }

    #[inline(always)]
    async fn service_polled_frame(&mut self, ind: zigbee_mac::McpsDataIndication) -> bool {
        let was_identifying = self.device.is_identifying(1);
        let mut clusters = cluster_refs(
            &mut *self.temp_cluster,
            &mut *self.hum_cluster,
            &mut *self.power_cluster,
        );
        let event = match self
            .device
            .process_incoming_with_security_store(
                &ind,
                &mut clusters,
                &mut *self.security_store,
            )
            .await
        {
            Ok(event) => event,
            Err(error) => persistence_failure(error),
        };
        if !was_identifying && self.device.is_identifying(1) {
            sed_diag_identify_received();
            rtt_target::rprintln!("[EFR32][sed] IDENTIFY_RECEIVED source=poll");
        }
        if let Some(ev) = event {
            if matches!(&ev, StackEvent::RejoinRequested) {
                log::info!("[EFR32] Secure rejoin requested");
                let _ = self.secure_rejoin().await;
                return true;
            }
            if matches!(&ev, StackEvent::LeaveRequested) {
                log::info!("[EFR32] Leave requested - erasing NV and rejoining");
                self.factory_reset().await;
                self.needs_bootstrap_join = true;
                let _ = self.bootstrap_join("rejoin").await;
                self.needs_checkpoint = false;
                return true;
            }
            if log_event(&ev) {
                self.reset_post_join_state();
                log::info!("[EFR32] Fast poll ON ({})", FAST_POLL_DURATION_SECS);
                self.needs_checkpoint = true;
            }
        }

        self.update_interview_state();

        let mut clusters = cluster_refs(
            &mut *self.temp_cluster,
            &mut *self.hum_cluster,
            &mut *self.power_cluster,
        );
        if let Err(error) = self
            .device
            .tick_with_security_store(0, &mut clusters, &mut *self.security_store)
            .await
        {
            persistence_failure(error);
        }
        false
    }

    #[inline(always)]
    async fn service_joined_polls(&mut self) {
        for _poll_round in 0..4u8 {
            match self.device.poll().await {
                Ok(Some(ind)) => {
                    if self.service_polled_frame(ind).await {
                        break;
                    }
                }
                Ok(None) => break,
                Err(_) => break,
            }
        }
    }

    #[inline(always)]
    async fn service_direct_rx_window(&mut self, window_ms: u64) {
        let deadline = Instant::now() + Duration::from_millis(window_ms);

        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }

            let remaining = deadline - now;
            match select::select(self.device.receive(), Timer::after(remaining)).await {
                select::Either::First(Ok(ind)) => {
                    log::trace!("[EFR32] Direct RX {} bytes", ind.payload.len());
                    let was_identifying = self.device.is_identifying(1);
                    let mut clusters = cluster_refs(
                        &mut *self.temp_cluster,
                        &mut *self.hum_cluster,
                        &mut *self.power_cluster,
                    );
                    let event = match self
                        .device
                        .process_incoming_with_security_store(
                            &ind,
                            &mut clusters,
                            &mut *self.security_store,
                        )
                        .await
                    {
                        Ok(event) => event,
                        Err(error) => persistence_failure(error),
                    };
                    if !was_identifying && self.device.is_identifying(1) {
                        sed_diag_identify_received();
                        rtt_target::rprintln!("[EFR32][sed] IDENTIFY_RECEIVED source=direct");
                    }
                    if let Some(ev) = event {
                        if matches!(&ev, StackEvent::RejoinRequested) {
                            rtt_target::rprintln!(
                                "[EFR32] Secure rejoin requested during direct RX"
                            );
                            let _ = self.secure_rejoin().await;
                            return;
                        }
                        if matches!(&ev, StackEvent::LeaveRequested) {
                            rtt_target::rprintln!("[EFR32] Leave requested during direct RX");
                            self.factory_reset().await;
                            self.needs_bootstrap_join = true;
                            return;
                        }
                        if log_event(&ev) {
                            self.reset_post_join_state();
                            self.needs_checkpoint = true;
                        }
                    }
                    self.update_interview_state();
                    let mut clusters = cluster_refs(
                        &mut *self.temp_cluster,
                        &mut *self.hum_cluster,
                        &mut *self.power_cluster,
                    );
                    if let Err(error) = self
                        .device
                        .tick_with_security_store(0, &mut clusters, &mut *self.security_store)
                        .await
                    {
                        persistence_failure(error);
                    }
                }
                select::Either::First(Err(_)) | select::Either::Second(_) => break,
            }
        }
    }

    #[inline(always)]
    async fn sample_sht(&mut self) {
        if let Err(error) = self.sht.start_measurement() {
            rtt_target::rprintln!("[EFR32][sensor] SHT_START_ERROR {:?}", error);
            return;
        }
        Timer::after(Duration::from_millis(20)).await;
        match self.sht.read_measurement() {
            Ok(measurement) => {
                self.temp_cluster
                    .set_temperature(measurement.temperature_centi_celsius);
                self.hum_cluster
                    .set_humidity(measurement.humidity_centi_percent);
                rtt_target::rprintln!(
                    "[EFR32][sensor] SHT_MEAS_OK temp_centi_c={} humidity_centi_percent={}",
                    measurement.temperature_centi_celsius,
                    measurement.humidity_centi_percent
                );
            }
            Err(error) => {
                rtt_target::rprintln!("[EFR32][sensor] SHT_READ_ERROR {:?}", error);
            }
        }
    }

    #[inline(always)]
    async fn update_measurements(&mut self, now: Instant) {
        let elapsed_s = now.duration_since(self.last_report).as_secs();
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.sample_sht().await;
        }
    }

    #[inline(always)]
    fn tick_elapsed_seconds(&mut self, now: Instant) -> u16 {
        let elapsed = now.duration_since(self.last_tick).as_secs().min(60);
        if elapsed != 0 {
            self.last_tick = self.last_tick + Duration::from_secs(elapsed);
        }
        elapsed as u16
    }

    #[inline(always)]
    async fn service_joined_tick(&mut self, now: Instant) {
        self.update_measurements(now).await;
        let tick_elapsed = self.tick_elapsed_seconds(now);
        let mut clusters = cluster_refs(
            &mut *self.temp_cluster,
            &mut *self.hum_cluster,
            &mut *self.power_cluster,
        );
        let tick_result = match self
            .device
            .tick_with_security_store(
                tick_elapsed,
                &mut clusters,
                &mut *self.security_store,
            )
            .await
        {
            Ok(result) => result,
            Err(error) => persistence_failure(error),
        };
        if let TickResult::Event(ref e) = tick_result
            && log_event(e)
        {
            self.reset_post_join_state();
        }

        if self.annce_retries_left > 0 && now.duration_since(self.last_annce).as_secs() >= 8 {
            self.annce_retries_left -= 1;
            self.last_annce = now;
            log::info!(
                "[EFR32] Device_annce retry ({} left)",
                self.annce_retries_left
            );
            self.checkpoint_security();
            let _ = self.device.send_device_annce().await;
            self.checkpoint_security();
        }
    }

    #[inline(always)]
    fn service_joined_post_rx(&mut self) {
        let identifying = self.device.is_identifying(1);
        if identifying && !self.was_identifying {
            rtt_target::rprintln!("[EFR32][sed] IDENTIFY_ACTIVE");
        } else if !identifying && self.was_identifying {
            sed_diag_identify_complete();
            rtt_target::rprintln!("[EFR32][sed] IDENTIFY_COMPLETE");
        }
        self.was_identifying = identifying;

        if identifying {
            if BOARD_LED.is_on() {
                led_off();
            } else {
                led_on();
            }
        }
        if self.needs_checkpoint {
            self.needs_checkpoint = false;
            self.checkpoint_security();
            rtt_target::rprintln!("[EFR32] Security checkpointed (deferred)");
            log::info!("[EFR32] Security checkpointed (deferred)");
        }
    }

    #[inline(always)]
    async fn service_unjoined_cycle(&mut self, now: Instant) {
        if now.duration_since(self.last_rejoin_attempt).as_secs() >= 1 {
            led_on();
            Timer::after(Duration::from_millis(80)).await;
            led_off();
            Timer::after(Duration::from_millis(120)).await;
            led_on();
            Timer::after(Duration::from_millis(80)).await;
            led_off();
        }

        if now.duration_since(self.last_rejoin_attempt).as_secs() >= JOIN_RETRY_SECS {
            self.request_join_retry("Retry").await;
        }
    }

    #[cfg(feature = "sed")]
    #[inline(never)]
    fn sleep_joined_until_next_poll(&mut self) {
        self.device.mac_mut().radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);

        let requested_ticks = pm::ms_to_ticks((SLOW_POLL_SECS * 1_000) as u32, pm::LFRCO_HZ);
        let before = pm::now();
        if let Err(error) = pm::sleep_for_ticks_polled(requested_ticks) {
            let _ = error;
            rtt_target::rprintln!("[EFR32][sed] EM2_FATAL");
            loop {
                led_on();
                cortex_m::asm::nop();
            }
        }
        let elapsed_ticks = pm::elapsed_ticks(before, pm::now());

        if let Err(error) = efr32mg1_tradfri::init_clocks() {
            let _ = error;
            rtt_target::rprintln!("[EFR32][sed] CLOCK_RESTORE_FATAL");
            loop {
                led_on();
                cortex_m::asm::nop();
            }
        }

        rtt_target::rprintln!(
            "[EFR32][sed] EM2_WAKE ticks={}",
            elapsed_ticks
        );
        sed_diag_em2_wake(elapsed_ticks);
    }

    #[inline(always)]
    async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            let now = Instant::now();
            let poll_ms = self.update_fast_poll_window(now);

            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                self.service_joined_tick(now).await;
                #[cfg(feature = "sed")]
                let direct_rx_ms = if poll_ms == FAST_POLL_MS { poll_ms } else { 0 };
                #[cfg(not(feature = "sed"))]
                let direct_rx_ms = poll_ms;
                self.service_direct_rx_window(direct_rx_ms).await;
                self.service_joined_polls().await;
                self.service_joined_post_rx();
                #[cfg(feature = "sed")]
                if !self.awaiting_initial_configuration
                    && !self.device.is_identifying(1)
                    && Instant::now() >= self.fast_poll_until
                {
                    self.sleep_joined_until_next_poll();
                }
            } else {
                self.device.mac_mut().radio_sleep();
                Timer::after(Duration::from_millis(poll_ms)).await;
                self.device.mac_mut().radio_wake();
                let now2 = Instant::now();
                self.service_unjoined_cycle(now2).await;
            }
        }
    }
}

#[cfg(feature = "sensor")]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    boot_signal().await;
    sed_diag_init();

    rtt_target::rprintln!("[EFR32] Starting...");

    led_off();
    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;

    let mac = Efr32Mac::new();
    rtt_target::rprintln!("[EFR32] Radio ready");

    let i2c = match efr32mg1_tradfri::sensor_i2c() {
        Ok(i2c) => i2c,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][sensor] I2C_FATAL error={:?}", error);
            loop {
                cortex_m::asm::nop();
            }
        }
    };
    let sht = sht_probe(i2c, "sensor").await;

    static SECURITY_STORE_CELL: StaticCell<SecurityStore> = StaticCell::new();
    let security_store = SECURITY_STORE_CELL.init(storage::security_store());
    rtt_target::rprintln!("[EFR32] Security store ready");

    static TEMP_CELL: StaticCell<TemperatureCluster> = StaticCell::new();
    let temp_cluster = TEMP_CELL.init(TemperatureCluster::new(-4000, 12500));
    static HUM_CELL: StaticCell<HumidityCluster> = StaticCell::new();
    let hum_cluster = HUM_CELL.init(HumidityCluster::new(0, 10000));
    static POWER_CELL: StaticCell<PowerConfigCluster> = StaticCell::new();
    let power_cluster = POWER_CELL.init(PowerConfigCluster::new());
    power_cluster.set_battery_size(4);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);

    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(SENSOR_POWER_MODE)
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-Sensor")
        .date_code("20260402")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(SENSOR_JOIN_CHANNEL_MASK)
        .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::TEMPERATURE_SENSOR, |ep| {
            ep.cluster_server(ClusterId::BASIC)
                .cluster_server(ClusterId::IDENTIFY)
                .cluster_server(ClusterId::POWER_CONFIG)
                .cluster_server(ClusterId::TEMPERATURE)
                .cluster_server(ClusterId::HUMIDITY)
        })
        .build_into(DEVICE.uninit());

    static APP_CELL: StaticCell<SensorApp> = StaticCell::new();
    let app = APP_CELL.init(SensorApp::new(
        device,
        security_store,
        sht,
        temp_cluster,
        hum_cluster,
        power_cluster,
    ));
    app.run().await
}

// ── Application-NV-only diagnostic ─────────────────────────────

#[cfg(feature = "diag-nv")]
#[cortex_m_rt::entry]
fn diag_nv_entry() -> ! {
    const TEST_ITEM: NvItemId = NvItemId::AppCustomBase;
    const TEST_PAYLOAD: [u8; 16] = *b"EFR32-NV-PROBE!!";

    platform_init("diag-nv");
    rtt_target::rprintln!(
        "[EFR32][diag-nv] BOOT nv=0x{:08X}..0x{:08X} radio=off",
        storage::APP_NV_PARTITION_START,
        storage::APP_NV_PARTITION_START + storage::APP_NV_PARTITION_SIZE as u32
    );

    let mut nv = match storage::application_nv() {
        Ok(nv) => nv,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-nv] OPEN_FAIL error={:?}", error);
            loop {
                cortex_m::asm::nop();
            }
        }
    };

    if let Err(error) = nv.write(TEST_ITEM, &TEST_PAYLOAD) {
        rtt_target::rprintln!("[EFR32][diag-nv] WRITE_FAIL error={:?}", error);
        loop {
            cortex_m::asm::nop();
        }
    }

    let mut readback = [0u8; TEST_PAYLOAD.len()];
    match nv.read(TEST_ITEM, &mut readback) {
        Ok(length) if length == TEST_PAYLOAD.len() && readback == TEST_PAYLOAD => {
            rtt_target::rprintln!(
                "[EFR32][diag-nv] PASS bytes={} page_size=0x800",
                length
            );
            led_on();
        }
        Ok(length) => {
            rtt_target::rprintln!(
                "[EFR32][diag-nv] VERIFY_FAIL bytes={} data={:02X?}",
                length,
                &readback[..length.min(readback.len())]
            );
        }
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-nv] READ_FAIL error={:?}", error);
        }
    }

    loop {
        cortex_m::asm::nop();
    }
}

// ── EM2 power-management diagnostic ─────────────────────────────
//
// Isolated bring-up gate for the RTCC-backed EM2 wake timer in
// `efr32mg1_hal::pm`, ahead of a future SED conversion. Never touches
// Zigbee, the security journal, application NV, I2C, or the radio — only
// clocks, LED, RTT, RTCC, and the Series-1 DCDC LN safety gate.

#[cfg(feature = "diag-em2")]
const DIAG_EM2_ITERATIONS: u32 = 10;
#[cfg(feature = "diag-em2")]
const DIAG_EM2_SLEEP_MS: u32 = 1_000;
#[cfg(feature = "diag-em2")]
const DIAG_EM2_TOLERANCE_PERCENT: u32 = 10; // LFRCO is not crystal-accurate.
#[cfg(feature = "diag-em2")]
const DIAG_EM2_CANARY_LEN: usize = 4;
#[cfg(feature = "diag-em2")]
const DIAG_EM2_CANARY_SEED_BSS: u32 = 0xC0FF_EE00;
#[cfg(feature = "diag-em2")]
const DIAG_EM2_CANARY_SEED_UNINIT: u32 = 0xC0FF_EE10;
#[cfg(feature = "diag-em2")]
const DIAG_EM2_CANARY_SEED_STACK: u32 = 0xC0FF_EE20;

// SRAM canary #1: ordinary `.bss` (zero-initialized at boot, low in RAM).
#[cfg(feature = "diag-em2")]
static mut DIAG_EM2_CANARY_BSS: [u32; DIAG_EM2_CANARY_LEN] = [0; DIAG_EM2_CANARY_LEN];

// SRAM canary #2: `.uninit`, the same NOLOAD section `FAULT_LOG` uses,
// placed immediately below the stack near the top of usable RAM. Modeled
// on the existing `FAULT_LOG` pattern: `MaybeUninit` + explicit write
// before first read, since `.uninit` receives no zero/copy at startup.
#[cfg(feature = "diag-em2")]
#[unsafe(link_section = ".uninit.diag_em2_canary")]
static mut DIAG_EM2_CANARY_UNINIT: MaybeUninit<[u32; DIAG_EM2_CANARY_LEN]> =
    MaybeUninit::uninit();

#[cfg(feature = "diag-em2")]
#[inline(always)]
unsafe fn diag_em2_canary_bss_mut() -> &'static mut [u32; DIAG_EM2_CANARY_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(DIAG_EM2_CANARY_BSS) }
}

#[cfg(feature = "diag-em2")]
#[inline(always)]
unsafe fn diag_em2_canary_uninit_mut() -> &'static mut [u32; DIAG_EM2_CANARY_LEN] {
    unsafe {
        &mut *core::ptr::addr_of_mut!(DIAG_EM2_CANARY_UNINIT)
            .cast::<[u32; DIAG_EM2_CANARY_LEN]>()
    }
}

#[cfg(feature = "diag-em2")]
fn diag_em2_fill_canary(seed: u32, slot: &mut [u32; DIAG_EM2_CANARY_LEN]) {
    for (index, word) in slot.iter_mut().enumerate() {
        *word = pm::canary_pattern(seed, index);
    }
}

/// RTCC interrupt handler. Required so a CC0 compare-match wakes the core
/// via a real, bounded ISR instead of falling through to `DefaultHandler`
/// (an infinite loop) once we unmask it in the NVIC. Only ever linked in
/// for the `diag-em2` binary.
#[cfg(feature = "diag-em2")]
#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    pm::handle_interrupt();
}

#[cfg(feature = "diag-em2")]
fn diag_em2_halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[cfg(feature = "diag-em2")]
#[cortex_m_rt::entry]
fn diag_em2_entry() -> ! {
    platform_init("diag-em2");
    rtt_target::rprintln!(
        "[EFR32][diag-em2] BOOT lfrco_hz={} iterations={} sleep_ms={} tolerance_pct={}",
        pm::LFRCO_HZ,
        DIAG_EM2_ITERATIONS,
        DIAG_EM2_SLEEP_MS,
        DIAG_EM2_TOLERANCE_PERCENT
    );

    // Place all SRAM canaries before the first sleep so any corruption
    // caused by RTCC bring-up itself is also caught.
    let mut canary_stack = [0u32; DIAG_EM2_CANARY_LEN];
    diag_em2_fill_canary(DIAG_EM2_CANARY_SEED_STACK, &mut canary_stack);
    unsafe {
        diag_em2_fill_canary(DIAG_EM2_CANARY_SEED_BSS, diag_em2_canary_bss_mut());
        diag_em2_fill_canary(DIAG_EM2_CANARY_SEED_UNINIT, diag_em2_canary_uninit_mut());
    }

    if let Err(error) = pm::init() {
        rtt_target::rprintln!("[EFR32][diag-em2] RTCC_INIT_FAIL error={:?}", error);
        diag_em2_halt();
    }

    // Our RTCC() handler above is a real, bounded ISR, so it is safe to
    // unmask now: a compare-match can never fall through to the infinite
    // `DefaultHandler` loop.
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::Rtcc);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::Rtcc) };

    let ticks_per_sleep = pm::ms_to_ticks(DIAG_EM2_SLEEP_MS, pm::LFRCO_HZ);
    let mut failures: u32 = 0;

    for iteration in 1..=DIAG_EM2_ITERATIONS {
        let before = pm::now();

        let (deadline, cause) = match pm::sleep_for_ticks(ticks_per_sleep) {
            Ok(result) => result,
            Err(error) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-em2] iter={} SLEEP_FAIL error={:?}",
                    iteration,
                    error
                );
                failures += 1;
                continue;
            }
        };

        let after = pm::now();
        let elapsed = pm::elapsed_ticks(before, after);
        let elapsed_ms = pm::ticks_to_ms(elapsed, pm::LFRCO_HZ);
        let progressed =
            pm::progressed_within_tolerance(elapsed, ticks_per_sleep, DIAG_EM2_TOLERANCE_PERCENT);

        let canary_bss = unsafe {
            pm::validate_canaries(DIAG_EM2_CANARY_SEED_BSS, diag_em2_canary_bss_mut())
        };
        let canary_uninit = unsafe {
            pm::validate_canaries(DIAG_EM2_CANARY_SEED_UNINIT, diag_em2_canary_uninit_mut())
        };
        let canary_stack = pm::validate_canaries(DIAG_EM2_CANARY_SEED_STACK, &canary_stack);

        let ok = matches!(cause, pm::WakeCause::RtccCompare)
            && progressed
            && canary_bss.is_ok()
            && canary_uninit.is_ok()
            && canary_stack.is_ok();

        rtt_target::rprintln!(
            "[EFR32][diag-em2] iter={} cause={:?} deadline=0x{:08X} elapsed_ticks={} \
             elapsed_ms={} progressed={} canary_bss={} canary_uninit={} canary_stack={} => {}",
            iteration,
            cause,
            deadline,
            elapsed,
            elapsed_ms,
            progressed,
            canary_bss.is_ok(),
            canary_uninit.is_ok(),
            canary_stack.is_ok(),
            if ok { "PASS" } else { "FAIL" }
        );

        if !ok {
            failures += 1;
            if let Err(mismatch) = canary_bss {
                rtt_target::rprintln!("[EFR32][diag-em2] CANARY_BSS_FAIL {:?}", mismatch);
            }
            if let Err(mismatch) = canary_uninit {
                rtt_target::rprintln!("[EFR32][diag-em2] CANARY_UNINIT_FAIL {:?}", mismatch);
            }
            if let Err(mismatch) = canary_stack {
                rtt_target::rprintln!("[EFR32][diag-em2] CANARY_STACK_FAIL {:?}", mismatch);
            }
        }
    }

    if failures == 0 {
        rtt_target::rprintln!("[EFR32][diag-em2] PASS iterations={} failures=0", DIAG_EM2_ITERATIONS);
        led_on();
    } else {
        rtt_target::rprintln!(
            "[EFR32][diag-em2] FAIL iterations={} failures={}",
            DIAG_EM2_ITERATIONS,
            failures
        );
    }

    diag_em2_halt();
}

// ── RTCC Embassy time-driver diagnostic ─────────────────────────
//
// Proves the RTCC/LFRCO Embassy time driver (`time_driver.rs`) end to end
// with a real Embassy executor + `Timer`, ahead of a future SED
// conversion. Two phases, both gated behind this feature alone — never
// touches Zigbee, NV, I2C, or the radio:
//
//   1. Short *active* waits through the normal `Timer::after` path (queue
//      scheduling + `now()`/`schedule_wake` via RTCC's single CC0 compare).
//   2. An *explicit* EM2 wait, bypassing `Timer` entirely and calling
//      `pm::sleep_for_ticks_polled` directly (same bounded
//      apply-DCDC-gate/SLEEPDEEP/WFI primitive `diag-em2` already proved),
//      run strictly after phase 1 completes so there is only ever one
//      owner of RTCC's CC0 channel at a time.

#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_ACTIVE_ITERATIONS: u32 = 10;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_ACTIVE_WAIT_MS: u64 = 200;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_ACTIVE_TOLERANCE_PERCENT: u32 = 20;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_EM2_ITERATIONS: u32 = 5;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_EM2_WAIT_MS: u32 = 500;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_EM2_TOLERANCE_PERCENT: u32 = 10; // LFRCO is not crystal-accurate.
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_CANARY_LEN: usize = 4;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_CANARY_SEED_BSS: u32 = 0xC0FF_EE30;
#[cfg(feature = "diag-rtcc-time")]
const DIAG_RTCC_TIME_CANARY_SEED_STACK: u32 = 0xC0FF_EE40;

// Reuses the same SRAM-canary technique as `diag-em2` (`.bss` word array +
// a stack-local array threaded through as a task argument) to catch any
// memory corruption from the EM2 phase.
#[cfg(feature = "diag-rtcc-time")]
static mut DIAG_RTCC_TIME_CANARY_BSS: [u32; DIAG_RTCC_TIME_CANARY_LEN] =
    [0; DIAG_RTCC_TIME_CANARY_LEN];

#[cfg(feature = "diag-rtcc-time")]
#[inline(always)]
unsafe fn diag_rtcc_time_canary_bss_mut() -> &'static mut [u32; DIAG_RTCC_TIME_CANARY_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(DIAG_RTCC_TIME_CANARY_BSS) }
}

#[cfg(feature = "diag-rtcc-time")]
fn diag_rtcc_time_fill_canary(seed: u32, slot: &mut [u32; DIAG_RTCC_TIME_CANARY_LEN]) {
    for (index, word) in slot.iter_mut().enumerate() {
        *word = pm::canary_pattern(seed, index);
    }
}

#[cfg(feature = "diag-rtcc-time")]
#[embassy_executor::task]
async fn diag_rtcc_time_task(canary_stack: [u32; DIAG_RTCC_TIME_CANARY_LEN]) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-rtcc-time] BOOT lfrco_hz={} embassy_tick_hz={} active_iterations={} \
         active_wait_ms={} em2_iterations={} em2_wait_ms={}",
        pm::LFRCO_HZ,
        embassy_time::TICK_HZ,
        DIAG_RTCC_TIME_ACTIVE_ITERATIONS,
        DIAG_RTCC_TIME_ACTIVE_WAIT_MS,
        DIAG_RTCC_TIME_EM2_ITERATIONS,
        DIAG_RTCC_TIME_EM2_WAIT_MS,
    );

    let mut failures: u32 = 0;

    // ── Phase 1: short active waits via the Embassy Timer queue ──
    for iteration in 1..=DIAG_RTCC_TIME_ACTIVE_ITERATIONS {
        let before = Instant::now();
        Timer::after(Duration::from_millis(DIAG_RTCC_TIME_ACTIVE_WAIT_MS)).await;
        let after = Instant::now();
        let elapsed_ms = (after - before).as_millis();
        let progressed = pm::progressed_within_tolerance(
            elapsed_ms as u32,
            DIAG_RTCC_TIME_ACTIVE_WAIT_MS as u32,
            DIAG_RTCC_TIME_ACTIVE_TOLERANCE_PERCENT,
        );
        rtt_target::rprintln!(
            "[EFR32][diag-rtcc-time] active_iter={} elapsed_ms={} progressed={} => {}",
            iteration,
            elapsed_ms,
            progressed,
            if progressed { "PASS" } else { "FAIL" }
        );
        if !progressed {
            failures += 1;
        }
    }

    // ── Phase 2: one explicit EM2 wait per iteration ─────────────
    // Safe to bypass the Embassy queue here: phase 1 has fully completed
    // (no `Timer` await is outstanding), and this task is the only one
    // spawned on this executor, so nothing else can be contending for
    // RTCC's single CC0 channel at the same time.
    for iteration in 1..=DIAG_RTCC_TIME_EM2_ITERATIONS {
        let ticks = pm::ms_to_ticks(DIAG_RTCC_TIME_EM2_WAIT_MS, pm::LFRCO_HZ);
        let before = pm::now();

        match pm::sleep_for_ticks_polled(ticks) {
            Ok(deadline) => {
                let after = pm::now();
                let elapsed = pm::elapsed_ticks(before, after);
                let elapsed_ms = pm::ticks_to_ms(elapsed, pm::LFRCO_HZ);
                let progressed = pm::progressed_within_tolerance(
                    elapsed,
                    ticks,
                    DIAG_RTCC_TIME_EM2_TOLERANCE_PERCENT,
                );
                // `enter_em2_once` (inside `sleep_for_ticks_polled`) must
                // never leave SLEEPDEEP armed once it returns, whatever
                // woke the core.
                let sleepdeep_cleared = !pm::sleepdeep_is_set();
                let canary_bss = unsafe {
                    pm::validate_canaries(
                        DIAG_RTCC_TIME_CANARY_SEED_BSS,
                        diag_rtcc_time_canary_bss_mut(),
                    )
                };
                let canary_stack =
                    pm::validate_canaries(DIAG_RTCC_TIME_CANARY_SEED_STACK, &canary_stack);
                let ok = progressed
                    && sleepdeep_cleared
                    && canary_bss.is_ok()
                    && canary_stack.is_ok();

                rtt_target::rprintln!(
                    "[EFR32][diag-rtcc-time] em2_iter={} deadline=0x{:08X} elapsed_ticks={} \
                     elapsed_ms={} progressed={} sleepdeep_cleared={} canary_bss={} \
                     canary_stack={} => {}",
                    iteration,
                    deadline,
                    elapsed,
                    elapsed_ms,
                    progressed,
                    sleepdeep_cleared,
                    canary_bss.is_ok(),
                    canary_stack.is_ok(),
                    if ok { "PASS" } else { "FAIL" }
                );
                if !ok {
                    failures += 1;
                    if let Err(mismatch) = canary_bss {
                        rtt_target::rprintln!(
                            "[EFR32][diag-rtcc-time] CANARY_BSS_FAIL {:?}",
                            mismatch
                        );
                    }
                    if let Err(mismatch) = canary_stack {
                        rtt_target::rprintln!(
                            "[EFR32][diag-rtcc-time] CANARY_STACK_FAIL {:?}",
                            mismatch
                        );
                    }
                }
            }
            Err(error) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-rtcc-time] em2_iter={} SLEEP_FAIL error={:?}",
                    iteration,
                    error
                );
                failures += 1;
            }
        }
    }

    if failures == 0 {
        rtt_target::rprintln!(
            "[EFR32][diag-rtcc-time] PASS active_iterations={} em2_iterations={} failures=0",
            DIAG_RTCC_TIME_ACTIVE_ITERATIONS,
            DIAG_RTCC_TIME_EM2_ITERATIONS
        );
        led_on();
    } else {
        rtt_target::rprintln!("[EFR32][diag-rtcc-time] FAIL failures={}", failures);
    }

    // Embassy tasks cannot return (`-> !`); idle on ordinary long Timer
    // waits so the executor keeps servicing the RTCC-backed queue forever
    // instead of busy-looping.
    loop {
        Timer::after(Duration::from_secs(3_600)).await;
    }
}

#[cfg(feature = "diag-rtcc-time")]
#[cortex_m_rt::entry]
fn diag_rtcc_time_entry() -> ! {
    platform_init("diag-rtcc-time");
    rtt_target::rprintln!("[EFR32][diag-rtcc-time] BOOT phase=power-management-gate nv=off radio=off i2c=off");

    let mut canary_stack = [0u32; DIAG_RTCC_TIME_CANARY_LEN];
    diag_rtcc_time_fill_canary(DIAG_RTCC_TIME_CANARY_SEED_STACK, &mut canary_stack);
    unsafe {
        diag_rtcc_time_fill_canary(
            DIAG_RTCC_TIME_CANARY_SEED_BSS,
            diag_rtcc_time_canary_bss_mut(),
        );
    }

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| spawner.must_spawn(diag_rtcc_time_task(canary_stack)))
}

// ── Radio + EM2 restoration diagnostic ─────────────────────────

#[cfg(feature = "diag-radio-em2")]
async fn diag_radio_em2_tx(mac: &mut Efr32Mac, seq: u8) -> bool {
    if let Err(error) = mac
        .mlme_set(
            PibAttribute::PhyCurrentChannel,
            PibValue::U8(DIAG_SCAN_CHANNEL),
        )
        .await
    {
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] CHANNEL_FAIL error={:?}",
            error
        );
        return false;
    }

    let frame = build_beacon_request(seq);
    match mac.debug_transmit_raw(&frame).await {
        Ok(()) => true,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-radio-em2] TX_FAIL error={:?}", error);
            false
        }
    }
}

#[cfg(feature = "diag-radio-em2")]
#[embassy_executor::task]
async fn diag_radio_em2_task(mut mac: Efr32Mac) -> ! {
    let sleep_ticks = pm::ms_to_ticks(DIAG_RADIO_EM2_SLEEP_MS, pm::LFRCO_HZ);
    let mut failures = 0u8;

    rtt_target::rprintln!(
        "[EFR32][diag-radio-em2] BOOT iterations={} sleep_ms={} channel={} nv=off i2c=off zigbee=off",
        DIAG_RADIO_EM2_ITERATIONS,
        DIAG_RADIO_EM2_SLEEP_MS,
        DIAG_SCAN_CHANNEL
    );

    if !diag_radio_em2_tx(&mut mac, DIAG_TX_TEST_SEQ).await {
        failures = failures.saturating_add(1);
    } else {
        rtt_target::rprintln!("[EFR32][diag-radio-em2] initial_tx=PASS");
    }

    for iteration in 1..=DIAG_RADIO_EM2_ITERATIONS {
        mac.radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);

        let before = pm::now();
        let sleep_result = pm::sleep_for_ticks_polled(sleep_ticks);
        let after = pm::now();
        let elapsed_ticks = pm::elapsed_ticks(before, after);
        let elapsed_ms = pm::ticks_to_ms(elapsed_ticks, pm::LFRCO_HZ);

        let clock_ready = efr32mg1_tradfri::init_clocks().is_ok();
        mac.radio_wake();
        let tx_ok = diag_radio_em2_tx(&mut mac, DIAG_TX_TEST_SEQ.wrapping_add(iteration)).await;
        let progressed = pm::progressed_within_tolerance(
            elapsed_ticks,
            sleep_ticks,
            DIAG_RADIO_EM2_TOLERANCE_PERCENT,
        );
        let sleepdeep_cleared = !pm::sleepdeep_is_set();
        let passed =
            sleep_result.is_ok() && clock_ready && tx_ok && progressed && sleepdeep_cleared;

        if !passed {
            failures = failures.saturating_add(1);
        }

        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] iter={} elapsed_ticks={} elapsed_ms={} sleep_ok={} clock_ready={} tx_ok={} progressed={} sleepdeep_cleared={} => {}",
            iteration,
            elapsed_ticks,
            elapsed_ms,
            sleep_result.is_ok(),
            clock_ready,
            tx_ok,
            progressed,
            sleepdeep_cleared,
            if passed { "PASS" } else { "FAIL" }
        );
    }

    if failures == 0 {
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] PASS iterations={} failures=0",
            DIAG_RADIO_EM2_ITERATIONS
        );
        led_on();
    } else {
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] FAIL iterations={} failures={}",
            DIAG_RADIO_EM2_ITERATIONS,
            failures
        );
    }

    loop {
        Timer::after(Duration::from_secs(3_600)).await;
    }
}

#[cfg(feature = "diag-radio-em2")]
#[cortex_m_rt::entry]
fn diag_radio_em2_entry() -> ! {
    platform_init("diag-radio-em2");
    let mac = Efr32Mac::new();

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| spawner.must_spawn(diag_radio_em2_task(mac)))
}

// ── Phase 2 SHT3x-only diagnostic ───────────────────────────────

#[cfg(any(feature = "sensor", feature = "diag-sht"))]
async fn sht_probe(
    mut i2c: efr32mg1_tradfri::SensorI2c,
    profile: &'static str,
) -> zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c> {
    loop {
        for address in [
            zigbee_sht3x::PRIMARY_ADDRESS,
            zigbee_sht3x::SECONDARY_ADDRESS,
        ] {
            rtt_target::rprintln!("[EFR32][{}] PROBE address=0x{:02X}", profile, address);
            let mut sensor = zigbee_sht3x::Sht3x::new(i2c, address);

            match sensor.soft_reset() {
                Ok(()) => {
                    Timer::after(Duration::from_millis(2)).await;
                    match sensor.read_status() {
                        Ok(status) => {
                            rtt_target::rprintln!(
                                "[EFR32][{}] SHT_FOUND address=0x{:02X} status=0x{:04X} crc=ok",
                                profile,
                                address,
                                status.raw
                            );
                            return sensor;
                        }
                        Err(error) => {
                            rtt_target::rprintln!(
                                "[EFR32][{}] STATUS_ERROR address=0x{:02X} error={:?}",
                                profile,
                                address,
                                error
                            );
                        }
                    }
                }
                Err(error) => {
                    rtt_target::rprintln!(
                        "[EFR32][{}] PROBE_MISS address=0x{:02X} error={:?}",
                        profile,
                        address,
                        error
                    );
                }
            }
            i2c = sensor.release();
        }

        rtt_target::rprintln!("[EFR32][{}] SHT_NOT_FOUND retry_ms=3000", profile);
        for _ in 0..2 {
            led_on();
            Timer::after(Duration::from_millis(100)).await;
            led_off();
            Timer::after(Duration::from_millis(100)).await;
        }
        Timer::after(Duration::from_millis(2_600)).await;
    }
}

#[cfg(feature = "diag-sht")]
#[embassy_executor::task]
async fn diag_sht_task(i2c: efr32mg1_tradfri::SensorI2c) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-sht] I2C_READY controller=I2C0 sda=PC10 scl=PC11 loc=15 hz={}",
        efr32mg1_tradfri::SENSOR_I2C_HZ
    );
    let mut sensor = sht_probe(i2c, "diag-sht").await;
    let mut successful_samples = 0u32;
    let mut failed_samples = 0u32;

    loop {
        match sensor.start_measurement() {
            Ok(()) => {
                Timer::after(Duration::from_millis(20)).await;
                match sensor.read_measurement() {
                    Ok(measurement) => {
                        successful_samples = successful_samples.wrapping_add(1);
                        rtt_target::rprintln!(
                            "[EFR32][diag-sht] MEAS_OK seq={} errors={} address=0x{:02X} temp_centi_c={} humidity_centi_percent={} crc=ok",
                            successful_samples,
                            failed_samples,
                            sensor.address(),
                            measurement.temperature_centi_celsius,
                            measurement.humidity_centi_percent
                        );
                        led_on();
                        Timer::after(Duration::from_millis(80)).await;
                        led_off();
                    }
                    Err(error) => {
                        failed_samples = failed_samples.wrapping_add(1);
                        rtt_target::rprintln!(
                            "[EFR32][diag-sht] MEAS_READ_ERROR seq={} errors={} error={:?}",
                            successful_samples,
                            failed_samples,
                            error
                        );
                    }
                }
            }
            Err(error) => {
                failed_samples = failed_samples.wrapping_add(1);
                rtt_target::rprintln!(
                    "[EFR32][diag-sht] MEAS_START_ERROR seq={} errors={} error={:?}",
                    successful_samples,
                    failed_samples,
                    error
                );
            }
        }
        Timer::after(Duration::from_millis(DIAG_SHT_SAMPLE_INTERVAL_MS)).await;
    }
}

#[cfg(feature = "diag-sht")]
#[cortex_m_rt::entry]
fn diag_sht_entry() -> ! {
    platform_init("diag-sht");
    rtt_target::rprintln!("[EFR32][diag-sht] BOOT phase=2 nv=off radio=off");

    let i2c = match efr32mg1_tradfri::sensor_i2c() {
        Ok(i2c) => i2c,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-sht] I2C_FATAL error={:?}", error);
            loop {
                for _ in 0..3 {
                    led_on();
                    busy_delay();
                    led_off();
                    busy_delay();
                }
                busy_delay();
                busy_delay();
            }
        }
    };

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| spawner.must_spawn(diag_sht_task(i2c)))
}

#[cfg(feature = "diag-join")]
struct DiagJoinApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    last_join_attempt: Instant,
    join_attempts: u8,
    joined_at: Option<Instant>,
    last_annce_retry: Instant,
    annce_retries: u8,
    rx_frames: u32,
}

#[cfg(feature = "diag-join")]
impl DiagJoinApp {
    fn new(device: &'static mut ZigbeeDevice<Efr32Mac>) -> Self {
        Self {
            device,
            last_join_attempt: Instant::now(),
            join_attempts: 0,
            joined_at: None,
            last_annce_retry: Instant::now(),
            annce_retries: 0,
            rx_frames: 0,
        }
    }

    #[inline(always)]
    fn handles(&mut self) -> DiagJoinHandles {
        DiagJoinHandles {
            device: self.device as *mut ZigbeeDevice<Efr32Mac>,
        }
    }

    #[inline(always)]
    fn handle_event(&mut self, event: &StackEvent) {
        match event {
            StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => {
                self.joined_at = Some(Instant::now());
                self.last_annce_retry = Instant::now();
                self.annce_retries = 0;
                led_on();
                rtt_target::rprintln!(
                    "[EFR32][diag-join] Joined addr=0x{:04X} ch={} pan=0x{:04X}",
                    short_address,
                    channel,
                    pan_id
                );
            }
            StackEvent::Left => {
                self.joined_at = None;
                self.annce_retries = 0;
                led_off();
                rtt_target::rprintln!("[EFR32][diag-join] Left network");
            }
            StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
                self.joined_at = None;
                self.annce_retries = 0;
                led_off();
                rtt_target::rprintln!("[EFR32][diag-join] Leave requested by coordinator");
            }
            StackEvent::CommissioningComplete { success } => {
                rtt_target::rprintln!(
                    "[EFR32][diag-join] Commissioning {}",
                    if *success { "ok" } else { "failed" }
                );
            }
            StackEvent::ReportSent => {
                rtt_target::rprintln!("[EFR32][diag-join] Report sent");
            }
            _ => {
                rtt_target::rprintln!("[EFR32][diag-join] Stack event");
            }
        }
    }

    #[inline(always)]
    fn poll_interval_ms(&self, now: Instant) -> u64 {
        match self.joined_at {
            Some(joined_at)
                if now.duration_since(joined_at).as_secs() < DIAG_JOIN_FAST_POLL_SECS =>
            {
                DIAG_JOIN_POLL_MS
            }
            Some(_) => DIAG_JOIN_STEADY_POLL_MS,
            None => 500,
        }
    }

    #[inline(always)]
    async fn request_join(&mut self, reason: &'static str) {
        let channel_mask = DIAG_JOIN_MASK_2GHZ;

        self.last_join_attempt = Instant::now();
        self.join_attempts = self.join_attempts.wrapping_add(1);
        rtt_target::rprintln!(
            "[EFR32][diag-join] {} join attempt {}",
            reason,
            self.join_attempts
        );

        rtt_target::rprintln!("[EFR32][diag-join] step=bdb_init");
        if self.device.bdb_mut().initialize().is_err() {
            rtt_target::rprintln!("[EFR32][diag-join] bdb_init=err");
            return;
        }
        rtt_target::rprintln!("[EFR32][diag-join] bdb_init=ok");

        // Diagnostic: raw MAC scan to verify RX state
        rtt_target::rprintln!(
            "[EFR32][diag-join] step=raw_scan mask=0x{:08X}",
            channel_mask.0
        );
        match self
            .device
            .mac_mut()
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask,
                scan_duration: DIAG_SCAN_DURATION,
            })
            .await
        {
            Ok(confirm) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-join] raw_scan={} beacon(s)",
                    confirm.pan_descriptors.len()
                );
            }
            Err(_) => {
                rtt_target::rprintln!("[EFR32][diag-join] raw_scan=err");
            }
        }

        // NWK-level discovery
        rtt_target::rprintln!(
            "[EFR32][diag-join] step=pre_scan mask=0x{:08X}",
            channel_mask.0
        );
        match self
            .device
            .bdb_mut()
            .zdo_mut()
            .nlme_network_discovery(channel_mask, DIAG_SCAN_DURATION)
            .await
        {
            Ok(networks) => {
                rtt_target::rprintln!("[EFR32][diag-join] pre_scan={} network(s)", networks.len());
                for (idx, network) in networks.iter().take(4).enumerate() {
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] net[{}] pan=0x{:04X} ch={} permit={} depth={} lqi={} via=0x{:04X}",
                        idx,
                        network.pan_id.0,
                        network.logical_channel,
                        network.permit_joining as u8,
                        network.depth,
                        network.lqi,
                        network.router_address.0
                    );
                }
            }
            Err(_) => {
                rtt_target::rprintln!("[EFR32][diag-join] pre_scan=err");
            }
        }

        self.device.bdb_mut().attributes_mut().primary_channel_set = channel_mask;
        self.device.bdb_mut().attributes_mut().secondary_channel_set = ChannelMask(0);
        rtt_target::rprintln!(
            "[EFR32][diag-join] step=network_steering mask=0x{:08X}",
            channel_mask.0
        );
        if self.device.bdb_mut().network_steering().await.is_err() {
            rtt_target::rprintln!("[EFR32][diag-join] network_steering=err");
            return;
        }
        let nib = self.device.bdb().zdo().nwk().nib();
        self.joined_at = Some(Instant::now());
        self.last_annce_retry = Instant::now();
        self.annce_retries = 0;
        rtt_target::rprintln!(
            "[EFR32][diag-join] network_steering=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );
        match self.device.send_device_annce().await {
            Ok(()) => {
                rtt_target::rprintln!("[EFR32][diag-join] device_annce retry=ok");
            }
            Err(e) => {
                rtt_target::rprintln!("[EFR32][diag-join] device_annce retry=err {:?}", e);
            }
        }
    }

    #[inline(always)]
    async fn maybe_retry_device_annce(&mut self, now: Instant) {
        let Some(joined_at) = self.joined_at else {
            return;
        };
        if now.duration_since(joined_at).as_secs() >= DIAG_JOIN_FAST_POLL_SECS {
            return;
        }
        if self.annce_retries >= DIAG_JOIN_ANNCE_RETRIES {
            return;
        }
        if now.duration_since(self.last_annce_retry).as_secs() < DIAG_JOIN_ANNCE_RETRY_SECS {
            return;
        }

        self.last_annce_retry = now;
        self.annce_retries = self.annce_retries.wrapping_add(1);
        match self.device.send_device_annce().await {
            Ok(()) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-join] periodic device_annce {}=ok",
                    self.annce_retries
                );
            }
            Err(e) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-join] periodic device_annce {}=err {:?}",
                    self.annce_retries,
                    e
                );
            }
        }
    }

    #[inline(always)]
    async fn service_polls(&mut self) {
        for _ in 0..4u8 {
            match self.device.poll().await {
                Ok(Some(ind)) => {
                    self.rx_frames = self.rx_frames.wrapping_add(1);
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] Poll RX {} bytes (frame #{})",
                        ind.payload.len(),
                        self.rx_frames
                    );
                    if let Some(event) =
                        diag_join_process_incoming_handles(self.handles(), &ind).await
                    {
                        self.handle_event(&event);
                    }
                    if let TickResult::Event(ref event) =
                        diag_join_tick_handles(self.handles(), 0).await
                    {
                        self.handle_event(event);
                    }
                }
                Ok(None) => break,
                Err(_) => {
                    rtt_target::rprintln!("[EFR32][diag-join] Poll failed");
                    break;
                }
            }
        }
    }

    #[inline(always)]
    async fn service_direct_rx_window(&mut self, window_ms: u64) {
        let deadline = Instant::now() + Duration::from_millis(window_ms);

        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }

            let remaining = deadline - now;
            match select::select(self.device.receive(), Timer::after(remaining)).await {
                select::Either::First(Ok(ind)) => {
                    self.rx_frames = self.rx_frames.wrapping_add(1);
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] Direct RX {} bytes (frame #{})",
                        ind.payload.len(),
                        self.rx_frames
                    );
                    if let Some(event) =
                        diag_join_process_incoming_handles(self.handles(), &ind).await
                    {
                        self.handle_event(&event);
                    }
                    if let TickResult::Event(ref event) =
                        diag_join_tick_handles(self.handles(), 0).await
                    {
                        self.handle_event(event);
                    }
                }
                select::Either::First(Err(_)) | select::Either::Second(_) => break,
            }
        }
    }

    #[inline(always)]
    async fn run(&mut self) -> ! {
        rtt_target::rprintln!("[EFR32][diag-join] Starting...");
        self.request_join("startup").await;

        loop {
            let now = Instant::now();
            if !self.device.is_joined()
                && now.duration_since(self.last_join_attempt).as_secs() >= JOIN_RETRY_SECS
            {
                self.request_join("retry").await;
            }

            let poll_ms = self.poll_interval_ms(now);

            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                self.service_direct_rx_window(poll_ms).await;
                self.service_polls().await;
                self.maybe_retry_device_annce(Instant::now()).await;
                if let TickResult::Event(ref event) =
                    diag_join_tick_handles(self.handles(), 1).await
                {
                    self.handle_event(event);
                }
            } else {
                self.device.mac_mut().radio_sleep();
                Timer::after(Duration::from_millis(poll_ms)).await;
                self.device.mac_mut().radio_wake();
                led_on();
                Timer::after(Duration::from_millis(80)).await;
                led_off();
            }
        }
    }
}

#[cfg(feature = "diag-join")]
static DIAG_DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
#[cfg(feature = "diag-join")]
static DIAG_APP_CELL: StaticCell<DiagJoinApp> = StaticCell::new();

#[cfg(feature = "diag-join")]
#[inline(always)]
fn init_zigbee_device(mac: Efr32Mac) -> &'static mut ZigbeeDevice<Efr32Mac> {
    ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 1_000,
            wake_duration_ms: 300,
        })
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-JoinDiag")
        .date_code("20260422")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::TEMPERATURE_SENSOR, |ep| {
            ep.cluster_server(ClusterId::BASIC)
                .cluster_server(ClusterId::IDENTIFY)
        })
        .build_into(DIAG_DEVICE.uninit())
}

#[cfg(feature = "diag-join")]
#[inline(always)]
fn init_diag_app(device: &'static mut ZigbeeDevice<Efr32Mac>) -> &'static mut DiagJoinApp {
    DIAG_APP_CELL.init_with(|| DiagJoinApp::new(device))
}

#[cfg(feature = "diag-join")]
#[inline(always)]
fn build_diag_app() -> &'static mut DiagJoinApp {
    rtt_target::rprintln!("[EFR32][diag-join] step=mac_new");
    // Keep no-NV diagnostic commissioning separate from the production EUI.
    // Deriving from the factory value preserves uniqueness across boards.
    let mac = Efr32Mac::new();
    let mut diagnostic_ieee = mac.extended_address();
    diagnostic_ieee[0] |= 0x02;
    diagnostic_ieee[7] ^= 0xD1;
    let mac = mac.with_extended_address(diagnostic_ieee);
    rtt_target::rprintln!("[EFR32][diag-join] step=device_build");
    let device = init_zigbee_device(mac);
    rtt_target::rprintln!("[EFR32][diag-join] step=device_built");
    init_diag_app(device)
}

#[cfg(feature = "diag-join")]
#[embassy_executor::task]
async fn diag_join_task(app: &'static mut DiagJoinApp) -> ! {
    // LED boot blink in task so it doesn't stack with build_diag_app.
    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
    rtt_target::rprintln!("[EFR32][diag-join] step=app_run");
    app.run().await
}

#[cfg(feature = "diag-join")]
#[cortex_m_rt::entry]
fn diag_join_entry() -> ! {
    // Sync boot init (what boot_signal() does minus the async waits).
    platform_init("diag-join");
    rtt_target::rprintln!("[EFR32][diag-join] Booted");

    // Heavy build runs in #[entry] stack frame BEFORE executor starts,
    // so its ~17 KB of transient stack is released before any poll frame.
    let app = build_diag_app();
    rtt_target::rprintln!("[EFR32][diag-join] built, starting executor");

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(diag_join_task(app));
    })
}

#[cfg(feature = "diag-beacon")]
async fn diag_beacon_known_tx(mac: &mut Efr32Mac, seq: u8) -> bool {
    if let Err(err) = mac
        .mlme_set(
            PibAttribute::PhyCurrentChannel,
            PibValue::U8(DIAG_SCAN_CHANNEL),
        )
        .await
    {
        rtt_target::rprintln!("[EFR32][diag-beacon] tx-only channel-set=err {:?}", err);
        return false;
    }
    let beacon_req = build_beacon_request(seq);

    rtt_target::rprintln!(
        "[EFR32][diag-beacon] TX_ONLY_BEGIN raw-beacon-req ch={} len={} seq={}",
        DIAG_SCAN_CHANNEL,
        beacon_req.len(),
        seq
    );

    match mac.debug_transmit_raw(&beacon_req).await {
        Ok(_) => {
            rtt_target::rprintln!("[EFR32][diag-beacon] TX_ONLY_PASS scan_gate=open");
            true
        }
        Err(err) => {
            rtt_target::rprintln!(
                "[EFR32][diag-beacon] TX_ONLY_FAIL {:?} scan_gate=closed",
                err
            );
            false
        }
    }
}

#[cfg(feature = "diag-beacon")]
#[embassy_executor::task]
async fn diag_beacon_task(mut mac: Efr32Mac) -> ! {
    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    // Leave a deterministic post-flash window for an RTT client to attach
    // before the single TX gate consumes its one attempt.
    Timer::after(Duration::from_secs(5)).await;
    rtt_target::rprintln!("[EFR32][diag-beacon] Starting...");
    mac.debug_radio_snapshot("init");
    rtt_target::rprintln!(
        "[EFR32][diag-beacon] Radio initialized; one bounded TX-only gate precedes scan"
    );

    if !diag_beacon_known_tx(&mut mac, DIAG_TX_TEST_SEQ).await {
        led_on();
        loop {
            rtt_target::rprintln!(
                "[EFR32][diag-beacon] ACTIVE_SCAN_BLOCKED tx-only did not complete; inspect tx recovery snapshot"
            );
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    let channel_mask = ChannelMask(DIAG_SCAN_MASK_2GHZ);
    rtt_target::rprintln!(
        "[EFR32][diag-beacon] ACTIVE_SCAN_ENABLED mask={:#010X}",
        channel_mask.0
    );

    loop {
        let req = MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask,
            scan_duration: DIAG_SCAN_DURATION,
        };

        rtt_target::rprintln!(
            "[EFR32][diag-beacon] Scan request: type=active mask={:#010X} duration={}",
            channel_mask.0,
            DIAG_SCAN_DURATION
        );
        mac.debug_radio_snapshot("before-scan");

        match mac.mlme_scan(req).await {
            Ok(confirm) => {
                mac.debug_radio_snapshot("after-scan");
                rtt_target::rprintln!(
                    "[EFR32][diag-beacon] Scan done: {} beacon(s)",
                    confirm.pan_descriptors.len()
                );

                if confirm.pan_descriptors.is_empty() {
                    led_off();
                } else {
                    for pan in confirm.pan_descriptors.iter() {
                        rtt_target::rprintln!(
                            "[EFR32][diag-beacon] beacon: ch={} coord={:?} lqi={} permit={} depth={} router_cap={} enddev_cap={}",
                            pan.channel,
                            pan.coord_address,
                            pan.lqi,
                            pan.superframe_spec.association_permit,
                            pan.zigbee_beacon.device_depth,
                            pan.zigbee_beacon.router_capacity,
                            pan.zigbee_beacon.end_device_capacity
                        );
                    }

                    led_on();
                    Timer::after(Duration::from_millis(150)).await;
                    led_off();
                    Timer::after(Duration::from_millis(150)).await;
                    led_on();
                    Timer::after(Duration::from_millis(150)).await;
                    led_off();
                }
            }
            Err(err) => {
                mac.debug_radio_snapshot("scan-error");
                rtt_target::rprintln!("[EFR32][diag-beacon] Scan error: {:?}", err);
                led_on();
                Timer::after(Duration::from_millis(600)).await;
                led_off();
            }
        }

        Timer::after(Duration::from_secs(2)).await;
    }
}

#[cfg(feature = "diag-beacon")]
#[cortex_m_rt::entry]
fn diag_beacon_entry() -> ! {
    platform_init("diag-beacon");
    rtt_target::rprintln!("[EFR32][diag-beacon] Booted");

    let mac = Efr32Mac::new();

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    let executor = EXECUTOR.init(embassy_executor::Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(diag_beacon_task(mac));
    })
}

/// Log stack events. Returns true on join event.
#[cfg(feature = "sensor")]
fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            led_on();
            log::info!(
                "[EFR32] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                channel,
                pan_id
            );
            true
        }
        StackEvent::Left => {
            led_off();
            log::info!("[EFR32] Left network");
            false
        }
        StackEvent::ReportSent => {
            log::info!("[EFR32] Report sent");
            false
        }
        StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
            led_on();
            log::info!("[EFR32] Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            log::info!(
                "[EFR32] Commissioning: {}",
                if *success { "ok" } else { "failed" }
            );
            false
        }
        _ => {
            log::info!("[EFR32] Stack event");
            false
        }
    }
}

/// Configure default reporting intervals with reportable change thresholds.
#[cfg(feature = "sensor")]
fn setup_default_reporting(device: &mut ZigbeeDevice<Efr32Mac>) {
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

    // Temperature: report every 60-300s, min change 0.5°C (50 centidegrees)
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::TEMPERATURE.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::I16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::I16(50)),
        },
    );

    // Humidity: report every 60-300s, min change 1% (100 centi-%)
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::HUMIDITY.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::U16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::U16(100)),
        },
    );

    // Battery: report every 300-3600s, min change 2% (4 in 0.5% units)
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::POWER_CONFIG.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id:
                zigbee_zcl::clusters::power_config::ATTR_BATTERY_PERCENTAGE_REMAINING,
            data_type: ZclDataType::U8,
            min_interval: 300,
            max_interval: 3600,
            reportable_change: Some(ZclValue::U8(4)),
        },
    );

    log::info!("[EFR32] Default reporting configured (with change thresholds)");
}
