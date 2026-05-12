//! # Zigbee-RS EFR32MG1P Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for EFR32MG1P-based boards.
//! Pure-Rust radio driver — no RAIL library, no GSDK, no binary blobs.
//!
//! # Hardware
//! - EFR32MG1P (256KB flash, 32KB SRAM), ARM Cortex-M4F @ 40 MHz
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

#[cfg(not(any(feature = "sensor", feature = "diag-beacon", feature = "diag-join")))]
compile_error!("enable exactly one of `sensor`, `diag-join`, or `diag-beacon`");

#[cfg(any(
    all(feature = "sensor", feature = "diag-beacon"),
    all(feature = "sensor", feature = "diag-join"),
    all(feature = "diag-beacon", feature = "diag-join"),
))]
compile_error!("features `sensor`, `diag-join`, and `diag-beacon` are mutually exclusive");

#[cfg(feature = "stubs")]
mod stubs;

#[cfg(feature = "sensor")]
mod flash_nv;
mod time_driver;
mod vectors;

use core::mem::MaybeUninit;
use cortex_m as _;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;

fn init_logging() {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 512,
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
}

// ── Gecko Bootloader Application Properties ─────────────────────
//
// The Gecko Bootloader requires an ApplicationProperties_t struct
// in flash so it can identify, validate, and boot the application.
// The bootloader finds it via word 13 (offset 0x34) of the vector table.
//
// Struct layout from Silicon Labs application_properties.h:
//   magic[16]         — 16-byte magic identifier
//   structVersion     — version of this struct (0x0100)
//   signatureType     — 0 = none, 1 = ECDSA-P256, 2 = CRC32
//   signatureLocation — address of signature (0xFFFFFFFF = none)
//   app.type          — APPLICATION_TYPE_ZIGBEE = 1
//   app.version       — application version number
//   app.capabilities  — 0
//   app.productId[16] — UUID (all zeros)

#[repr(C)]
struct ApplicationProperties {
    magic: [u8; 16],
    struct_version: u32,
    signature_type: u32,
    signature_location: u32,
    // ApplicationData_t inline:
    app_type: u32,
    app_version: u32,
    app_capabilities: u32,
    app_product_id: [u8; 16],
}

#[unsafe(no_mangle)]
#[used]
static APP_PROPERTIES: ApplicationProperties = ApplicationProperties {
    magic: [
        0x13, 0xb7, 0x79, 0xfa, 0xc9, 0x25, 0xdd, 0xb7, 0xad, 0xf3, 0xcf, 0xe0, 0xf1, 0xb6, 0x14,
        0xb8,
    ],
    struct_version: 0x0000_0100,     // Version 1.0
    signature_type: 0,               // APPLICATION_SIGNATURE_NONE
    signature_location: 0xFFFF_FFFF, // No signature
    app_type: 1,                     // APPLICATION_TYPE_ZIGBEE
    app_version: 1,                  // Version 1
    app_capabilities: 0,
    app_product_id: [0u8; 16],
};

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

// ── VTOR setup ──────────────────────────────────────────────────
//
// When booting via Gecko Bootloader at 0x4000, uncomment this to
// redirect the vector table. For bare-metal boot at 0x0, VTOR
// defaults to 0x0 which is correct.

// When using Gecko Bootloader (app at 0x4000), uncomment to redirect VTOR:
// #[cortex_m_rt::pre_init]
// unsafe fn pre_init() {
//     unsafe {
//         core::ptr::write_volatile(0xE000_ED08 as *mut u32, 0x0000_4000);
//     }
// }

// Custom HardFault handler that saves faulting PC to known RAM location
// so we can read it via J-Link after the crash.
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe {
        let msp: u32;
        core::arch::asm!("mrs {}, msp", out(reg) msp);
        let log = fault_log_mut();
        core::ptr::addr_of_mut!((*log).hardfault_magic).write_volatile(0xDEAD_BEEF);
        core::ptr::addr_of_mut!((*log).hardfault_pc).write_volatile(ef.pc() as u32);
        core::ptr::addr_of_mut!((*log).hardfault_lr).write_volatile(ef.lr() as u32);
        core::ptr::addr_of_mut!((*log).hardfault_xpsr).write_volatile(ef.xpsr() as u32);
        core::ptr::addr_of_mut!((*log).hardfault_msp).write_volatile(msp);
        core::ptr::addr_of_mut!((*log).hardfault_r0).write_volatile(ef.r0() as u32);
        core::ptr::addr_of_mut!((*log).hardfault_r12).write_volatile(ef.r12() as u32);
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

// Set VTOR to 0x4000 — required when Gecko Bootloader is present.
// The bootloader at 0x0 jumps to our app at 0x4000, but cortex-m-rt
// reset handler may run before VTOR is properly set.

#[cfg(feature = "sensor")]
use embassy_executor::Spawner;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use embassy_futures::select;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use embassy_time::Instant;
use embassy_time::{Duration, Timer};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use static_cell::StaticCell;

#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_aps::PROFILE_HOME_AUTOMATION;
#[cfg(any(feature = "diag-beacon", feature = "diag-join"))]
use zigbee_mac::MacDriver;
#[cfg(feature = "diag-beacon")]
use zigbee_mac::pib::{PibAttribute, PibValue};
#[cfg(any(feature = "diag-beacon", feature = "diag-join"))]
use zigbee_mac::primitives::{MlmeScanRequest, ScanType};
#[cfg(feature = "diag-beacon")]
use zigbee_mac::frames::build_beacon_request;
#[cfg(any(feature = "sensor", feature = "diag-beacon", feature = "diag-join"))]
use zigbee_types::ChannelMask;
use zigbee_mac::efr32::Efr32Mac;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_nwk::DeviceType;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::event_loop::{StackEvent, TickResult};
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::power::PowerMode;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
#[cfg(feature = "sensor")]
use zigbee_runtime::UserAction;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_zcl::clusters::basic::BasicCluster;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::humidity::HumidityCluster;
#[cfg(any(feature = "sensor", feature = "diag-join"))]
use zigbee_zcl::clusters::identify::IdentifyCluster;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
#[cfg(feature = "sensor")]
use zigbee_zcl::clusters::temperature::TemperatureCluster;

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
#[cfg(feature = "sensor")]
const EXPECTED_REPORT_CLUSTERS: usize = 3; // PowerConfig + Temp + Humidity
#[cfg(feature = "sensor")]
const FORCE_FRESH_JOIN_ON_BOOT: bool = true;
#[cfg(feature = "sensor")]
const SENSOR_JOIN_CHANNEL_MASK: ChannelMask = ChannelMask(1u32 << 15);
#[cfg(any(feature = "diag-beacon", feature = "diag-join"))]
const DIAG_SCAN_DURATION: u8 = 4;
#[cfg(feature = "diag-beacon")]
const DIAG_SCAN_CHANNEL: u8 = 15;
#[cfg(feature = "diag-beacon")]
const DIAG_SCAN_MASK_2GHZ: u32 = 1u32 << DIAG_SCAN_CHANNEL;
#[cfg(feature = "diag-join")]
const DIAG_JOIN_MASK_2GHZ: ChannelMask = ChannelMask::ALL_2_4GHZ;
#[cfg(feature = "diag-beacon")]
const DIAG_TX_TEST_SEQ: u8 = 0xA5;
#[cfg(feature = "diag-beacon")]
const DIAG_TX_PRESCAN_BURSTS: u8 = 8;

// ── EFR32MG1P GPIO ─────────────────────────────────────────────

mod pins {
    // GPIO pin numbers — adjust for your board
    pub const LED: u8 = 6; // LED on PF6 (Thunderboard Sense)
    #[cfg(feature = "sensor")]
    #[allow(dead_code)]
    pub const BTN: u8 = 7; // Button on PF7 (Thunderboard Sense)
}

// Simple GPIO access via memory-mapped registers
// EFR32MG1P GPIO base = 0x4000_A000
fn gpio_set_output(pin: u8) {
    let port = (pin / 16) as u32;
    let pin_in_port = (pin % 16) as u32;
    let mode_reg = 0x4000_A004 + port * 0x30 + if pin_in_port >= 8 { 4 } else { 0 };
    let shift = (pin_in_port % 8) * 4;
    unsafe {
        let old = core::ptr::read_volatile(mode_reg as *const u32);
        // Mode 4 = push-pull output
        core::ptr::write_volatile(mode_reg as *mut u32, (old & !(0xF << shift)) | (4 << shift));
    }
}

fn gpio_write(pin: u8, high: bool) {
    let port = (pin / 16) as u32;
    let pin_in_port = (pin % 16) as u32;
    let reg = if high {
        0x4000_A018 + port * 0x30 // DOUTSET
    } else {
        0x4000_A01C + port * 0x30 // DOUTCLR
    };
    unsafe {
        core::ptr::write_volatile(reg as *mut u32, 1 << pin_in_port);
    }
}

#[cfg(feature = "sensor")]
fn gpio_read(pin: u8) -> bool {
    let port = (pin / 16) as u32;
    let pin_in_port = (pin % 16) as u32;
    let din_reg = 0x4000_A010 + port * 0x30; // DIN
    let val = unsafe { core::ptr::read_volatile(din_reg as *const u32) };
    (val >> pin_in_port) & 1 != 0
}

fn led_on() {
    gpio_write(pins::LED, true);
}
fn led_off() {
    gpio_write(pins::LED, false);
}

#[cfg(feature = "sensor")]
async fn boot_signal() {
    init_logging();
    time_driver::init();

    gpio_set_output(pins::LED);
    led_off();

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
    basic_cluster: &'a mut BasicCluster,
    temp_cluster: &'a mut TemperatureCluster,
    hum_cluster: &'a mut HumidityCluster,
    power_cluster: &'a mut PowerConfigCluster,
    identify_cluster: &'a mut IdentifyCluster,
) -> [ClusterRef<'a>; 5] {
    [
        ClusterRef {
            endpoint: 1,
            cluster: basic_cluster,
        },
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
        ClusterRef {
            endpoint: 1,
            cluster: identify_cluster,
        },
    ]
}

#[cfg(feature = "diag-join")]
fn join_cluster_refs<'a>(
    basic_cluster: &'a mut BasicCluster,
    identify_cluster: &'a mut IdentifyCluster,
) -> [ClusterRef<'a>; 2] {
    [
        ClusterRef {
            endpoint: 1,
            cluster: basic_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: identify_cluster,
        },
    ]
}

#[cfg(feature = "sensor")]
#[inline(always)]
async fn sensor_tick(
    device: &mut ZigbeeDevice<Efr32Mac>,
    elapsed_secs: u16,
    basic_cluster: &mut BasicCluster,
    temp_cluster: &mut TemperatureCluster,
    hum_cluster: &mut HumidityCluster,
    power_cluster: &mut PowerConfigCluster,
    identify_cluster: &mut IdentifyCluster,
) -> TickResult {
    let mut clusters = cluster_refs(
        basic_cluster,
        temp_cluster,
        hum_cluster,
        power_cluster,
        identify_cluster,
    );
    device.tick(elapsed_secs, &mut clusters).await
}

#[cfg(feature = "sensor")]
#[inline(always)]
async fn sensor_process_incoming(
    device: &mut ZigbeeDevice<Efr32Mac>,
    indication: &zigbee_mac::McpsDataIndication,
    basic_cluster: &mut BasicCluster,
    temp_cluster: &mut TemperatureCluster,
    hum_cluster: &mut HumidityCluster,
    power_cluster: &mut PowerConfigCluster,
    identify_cluster: &mut IdentifyCluster,
) -> Option<StackEvent> {
    let mut clusters = cluster_refs(
        basic_cluster,
        temp_cluster,
        hum_cluster,
        power_cluster,
        identify_cluster,
    );
    device.process_incoming(indication, &mut clusters).await
}

#[cfg(feature = "sensor")]
#[derive(Copy, Clone)]
struct SensorHandles {
    device: *mut ZigbeeDevice<Efr32Mac>,
    basic_cluster: *mut BasicCluster,
    temp_cluster: *mut TemperatureCluster,
    hum_cluster: *mut HumidityCluster,
    power_cluster: *mut PowerConfigCluster,
    identify_cluster: *mut IdentifyCluster,
}

#[cfg(feature = "sensor")]
#[inline(always)]
async fn sensor_tick_handles(handles: SensorHandles, elapsed_secs: u16) -> TickResult {
    unsafe {
        sensor_tick(
            &mut *handles.device,
            elapsed_secs,
            &mut *handles.basic_cluster,
            &mut *handles.temp_cluster,
            &mut *handles.hum_cluster,
            &mut *handles.power_cluster,
            &mut *handles.identify_cluster,
        )
        .await
    }
}

#[cfg(feature = "sensor")]
#[inline(always)]
async fn sensor_process_incoming_handles(
    handles: SensorHandles,
    indication: &zigbee_mac::McpsDataIndication,
) -> Option<StackEvent> {
    unsafe {
        sensor_process_incoming(
            &mut *handles.device,
            indication,
            &mut *handles.basic_cluster,
            &mut *handles.temp_cluster,
            &mut *handles.hum_cluster,
            &mut *handles.power_cluster,
            &mut *handles.identify_cluster,
        )
        .await
    }
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_tick(
    device: &mut ZigbeeDevice<Efr32Mac>,
    elapsed_secs: u16,
    basic_cluster: &mut BasicCluster,
    identify_cluster: &mut IdentifyCluster,
) -> TickResult {
    let mut clusters = join_cluster_refs(basic_cluster, identify_cluster);
    device.tick(elapsed_secs, &mut clusters).await
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_process_incoming(
    device: &mut ZigbeeDevice<Efr32Mac>,
    indication: &zigbee_mac::McpsDataIndication,
    basic_cluster: &mut BasicCluster,
    identify_cluster: &mut IdentifyCluster,
) -> Option<StackEvent> {
    let mut clusters = join_cluster_refs(basic_cluster, identify_cluster);
    device.process_incoming(indication, &mut clusters).await
}

#[cfg(feature = "diag-join")]
#[derive(Copy, Clone)]
struct DiagJoinHandles {
    device: *mut ZigbeeDevice<Efr32Mac>,
    basic_cluster: *mut BasicCluster,
    identify_cluster: *mut IdentifyCluster,
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_tick_handles(handles: DiagJoinHandles, elapsed_secs: u16) -> TickResult {
    unsafe {
        diag_join_tick(
            &mut *handles.device,
            elapsed_secs,
            &mut *handles.basic_cluster,
            &mut *handles.identify_cluster,
        )
        .await
    }
}

#[cfg(feature = "diag-join")]
#[inline(always)]
async fn diag_join_process_incoming_handles(
    handles: DiagJoinHandles,
    indication: &zigbee_mac::McpsDataIndication,
) -> Option<StackEvent> {
    unsafe {
        diag_join_process_incoming(
            &mut *handles.device,
            indication,
            &mut *handles.basic_cluster,
            &mut *handles.identify_cluster,
        )
        .await
    }
}

// ── Main ────────────────────────────────────────────────────────

#[cfg(feature = "sensor")]
struct SensorApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    nv: &'static mut flash_nv::Nv,
    basic_cluster: &'static mut BasicCluster,
    temp_cluster: &'static mut TemperatureCluster,
    hum_cluster: &'static mut HumidityCluster,
    power_cluster: &'static mut PowerConfigCluster,
    identify_cluster: &'static mut IdentifyCluster,
    hum_tick: u32,
    last_report: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: Instant,
    was_fast_polling: bool,
    interview_done: bool,
    #[allow(dead_code)]
    button_was_pressed: bool,
    needs_save: bool,
    needs_bootstrap_join: bool,
}

#[cfg(feature = "sensor")]
impl SensorApp {
    fn new(
        device: &'static mut ZigbeeDevice<Efr32Mac>,
        nv: &'static mut flash_nv::Nv,
        basic_cluster: &'static mut BasicCluster,
        temp_cluster: &'static mut TemperatureCluster,
        hum_cluster: &'static mut HumidityCluster,
        power_cluster: &'static mut PowerConfigCluster,
        identify_cluster: &'static mut IdentifyCluster,
    ) -> Self {
        let now = Instant::now();
        let joined = device.is_joined();
        Self {
            device,
            nv,
            basic_cluster,
            temp_cluster,
            hum_cluster,
            power_cluster,
            identify_cluster,
            hum_tick: 0,
            last_report: now,
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
            interview_done: false,
            button_was_pressed: false,
            needs_save: false,
            needs_bootstrap_join: !joined,
        }
    }

    #[inline(always)]
    fn reset_post_join_state(&mut self) {
        let now = Instant::now();
        self.fast_poll_until = now + Duration::from_secs(FAST_POLL_DURATION_SECS);
        self.last_rejoin_attempt = now;
        self.annce_retries_left = 0;
        self.last_annce = now;
        self.interview_done = false;
        self.was_fast_polling = true;
        led_on();
    }

    #[inline(always)]
    fn handles(&mut self) -> SensorHandles {
        SensorHandles {
            device: self.device as *mut ZigbeeDevice<Efr32Mac>,
            basic_cluster: self.basic_cluster as *mut BasicCluster,
            temp_cluster: self.temp_cluster as *mut TemperatureCluster,
            hum_cluster: self.hum_cluster as *mut HumidityCluster,
            power_cluster: self.power_cluster as *mut PowerConfigCluster,
            identify_cluster: self.identify_cluster as *mut IdentifyCluster,
        }
    }

    #[inline(always)]
    fn save_state(&mut self) {
        self.device.save_state(&mut *self.nv);
    }

    #[inline(always)]
    async fn factory_reset(&mut self) {
        let device = self.device as *mut ZigbeeDevice<Efr32Mac>;
        let nv = self.nv as *mut flash_nv::Nv;
        unsafe {
            (&mut *device).factory_reset(Some(&mut *nv)).await;
        }
    }

    #[inline(always)]
    async fn bootstrap_join(&mut self, reason: &'static str) -> bool {
        self.last_rejoin_attempt = Instant::now();
        self.rejoin_count = self.rejoin_count.wrapping_add(1);
        rtt_target::rprintln!("[EFR32] {} join (attempt {})", reason, self.rejoin_count);
        log::info!("[EFR32] {} join (attempt {})", reason, self.rejoin_count);

        if self.device.bdb_mut().initialize().await.is_err() {
            rtt_target::rprintln!("[EFR32] {} bdb_init=err", reason);
            log::info!("[EFR32] {} bdb_init=err", reason);
            return false;
        }
        rtt_target::rprintln!("[EFR32] {} bdb_init=ok", reason);

        self.device.bdb_mut().attributes_mut().primary_channel_set = SENSOR_JOIN_CHANNEL_MASK;
        self.device.bdb_mut().attributes_mut().secondary_channel_set = ChannelMask(0);

        rtt_target::rprintln!("[EFR32] {} network_steering...", reason);
        if self.device.bdb_mut().network_steering().await.is_err() {
            rtt_target::rprintln!("[EFR32] {} network_steering=err", reason);
            log::info!("[EFR32] {} network_steering=err", reason);
            return false;
        }

        let nib = self.device.bdb().zdo().nwk().nib();
        rtt_target::rprintln!(
            "[EFR32] {} network_steering=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            reason,
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );
        log::info!(
            "[EFR32] {} network_steering=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            reason,
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );

        self.reset_post_join_state();
        self.needs_bootstrap_join = false;
        self.needs_save = true;
        true
    }

    #[inline(always)]
    async fn run_first_tick(&mut self) {
        log::info!("[EFR32] First tick...");
        if self.needs_bootstrap_join && !self.device.is_joined() {
            if FORCE_FRESH_JOIN_ON_BOOT {
                rtt_target::rprintln!("[EFR32] Clearing NV/BDB before join");
                self.factory_reset().await;
            }
            let _ = self.bootstrap_join("startup").await;
        }
        if let TickResult::Event(ref e) = sensor_tick_handles(self.handles(), 0).await {
            log::info!("[EFR32] First tick event: {:?}", core::mem::discriminant(e));
            if log_event(e) {
                self.save_state();
            }
        }
        log::info!(
            "[EFR32] First tick done, joined={}",
            self.device.is_joined()
        );

        setup_default_reporting(&mut *self.device);
        self.temp_cluster.set_temperature(2250);
        self.hum_cluster.set_humidity(5000u16);
        self.power_cluster.set_battery_voltage(30);
        self.power_cluster.set_battery_percentage(100 * 2);
        log::info!("[EFR32] Initial: T=22.50°C H=50.00% Batt=3000mV (100%)");

        if self.device.is_joined() {
            log::info!("[EFR32] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
            self.reset_post_join_state();
        }
    }

    #[inline(always)]
    fn update_fast_poll_window(&mut self, now: Instant) -> u64 {
        let in_fast_poll = now < self.fast_poll_until;
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
        let _ = self.bootstrap_join(reason).await;
    }

    #[inline(always)]
    #[allow(dead_code)]
    async fn handle_button_press(&mut self) {
        let mut held_long = false;
        let press_start = Instant::now();
        while !gpio_read(pins::BTN) {
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
        if let TickResult::Event(ref e) = sensor_tick_handles(self.handles(), 0).await {
            match e {
                StackEvent::Joined { .. } => {
                    log_event(e);
                    self.reset_post_join_state();
                    self.needs_save = true;
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
        if let Some(ev) = sensor_process_incoming_handles(self.handles(), &ind).await {
            if let StackEvent::LeaveRequested = &ev {
                log::info!("[EFR32] Leave requested - erasing NV and rejoining");
                self.factory_reset().await;
                self.needs_bootstrap_join = true;
                let _ = self.bootstrap_join("rejoin").await;
                self.needs_save = false;
                return true;
            }
            if log_event(&ev) {
                self.reset_post_join_state();
                log::info!("[EFR32] Fast poll ON ({})", FAST_POLL_DURATION_SECS);
                self.needs_save = true;
            }
        }

        if !self.interview_done {
            let cfg_count = self.device.configured_cluster_count(1);
            if cfg_count >= EXPECTED_REPORT_CLUSTERS {
                log::info!(
                    "[EFR32] Local endpoint ready: {}/{} clusters configured",
                    cfg_count,
                    EXPECTED_REPORT_CLUSTERS
                );
                self.fast_poll_until = Instant::now() + Duration::from_secs(5);
                self.interview_done = true;
                led_off();
            }
        }

        let _ = sensor_tick_handles(self.handles(), 0).await;
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
                    rtt_target::rprintln!("[EFR32] Direct RX {} bytes", ind.payload.len());
                    if let Some(ev) = sensor_process_incoming_handles(self.handles(), &ind).await {
                        if let StackEvent::LeaveRequested = &ev {
                            rtt_target::rprintln!("[EFR32] Leave requested during direct RX");
                            self.factory_reset().await;
                            self.needs_bootstrap_join = true;
                            return;
                        }
                        if log_event(&ev) {
                            self.reset_post_join_state();
                            self.needs_save = true;
                        }
                    }
                    let _ = sensor_tick_handles(self.handles(), 0).await;
                }
                select::Either::First(Err(_)) | select::Either::Second(_) => break,
            }
        }
    }

    #[inline(always)]
    fn update_measurements(&mut self, now: Instant) -> u16 {
        let elapsed_s = now.duration_since(self.last_report).as_secs();
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            let temp_hundredths: i16 = 2250 + ((self.hum_tick % 50) as i16 - 25);
            self.hum_tick = self.hum_tick.wrapping_add(1);
            let hum_hundredths: u16 = 5000 + ((self.hum_tick % 100) as u16) * 10;
            self.temp_cluster.set_temperature(temp_hundredths);
            self.hum_cluster.set_humidity(hum_hundredths);
            log::info!(
                "[EFR32] T={}.{:02}C H={}.{:02}%",
                temp_hundredths / 100,
                (temp_hundredths % 100).unsigned_abs(),
                hum_hundredths / 100,
                hum_hundredths % 100,
            );
        }
        elapsed_s.min(60) as u16
    }

    #[inline(always)]
    async fn service_joined_cycle(&mut self, now: Instant) {
        self.service_joined_polls().await;

        let tick_elapsed = self.update_measurements(now);
        if let TickResult::Event(ref e) = sensor_tick_handles(self.handles(), tick_elapsed).await {
            if log_event(e) {
                self.reset_post_join_state();
            }
        }

        self.identify_cluster.tick(tick_elapsed);
        if self.identify_cluster.is_identifying() {
            let on = gpio_read(pins::LED);
            gpio_write(pins::LED, !on);
        }

        if self.annce_retries_left > 0 && now.duration_since(self.last_annce).as_secs() >= 8 {
            self.annce_retries_left -= 1;
            self.last_annce = now;
            log::info!(
                "[EFR32] Device_annce retry ({} left)",
                self.annce_retries_left
            );
            let _ = self.device.send_device_annce().await;
        }

        if self.needs_save {
            self.needs_save = false;
            self.save_state();
            log::info!("[EFR32] State saved to flash (deferred)");
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

    #[inline(always)]
    async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            let now = Instant::now();
            let poll_ms = self.update_fast_poll_window(now);

            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                self.service_direct_rx_window(poll_ms).await;
                self.service_joined_cycle(Instant::now()).await;
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

    rtt_target::rprintln!("[EFR32] Starting...");

    gpio_set_output(pins::LED);
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

    static NV_CELL: StaticCell<flash_nv::Nv> = StaticCell::new();
    let nv = NV_CELL.init(flash_nv::create_nv());
    rtt_target::rprintln!("[EFR32] NV ready");

    static BASIC_CELL: StaticCell<BasicCluster> = StaticCell::new();
    let basic_cluster = BASIC_CELL.init({
        let mut c = BasicCluster::new(b"Zigbee-RS", b"EFR32MG1-Sensor", b"20260402", b"0.1.0");
        c.set_power_source(0x03);
        c
    });
    static TEMP_CELL: StaticCell<TemperatureCluster> = StaticCell::new();
    let temp_cluster = TEMP_CELL.init(TemperatureCluster::new(-4000, 12500));
    static HUM_CELL: StaticCell<HumidityCluster> = StaticCell::new();
    let hum_cluster = HUM_CELL.init(HumidityCluster::new(0, 10000));
    static POWER_CELL: StaticCell<PowerConfigCluster> = StaticCell::new();
    let power_cluster = POWER_CELL.init(PowerConfigCluster::new());
    static IDENTIFY_CELL: StaticCell<IdentifyCluster> = StaticCell::new();
    let identify_cluster = IDENTIFY_CELL.init(IdentifyCluster::new());
    power_cluster.set_battery_size(4);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);

    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-Sensor")
        .sw_build("0.1.0")
        .channels(SENSOR_JOIN_CHANNEL_MASK)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000)
                .cluster_server(0x0003)
                .cluster_server(0x0001)
                .cluster_server(0x0402)
                .cluster_server(0x0405)
        })
        .build_into(DEVICE.uninit());

    let needs_bootstrap_join = if FORCE_FRESH_JOIN_ON_BOOT {
        rtt_target::rprintln!("[EFR32] Fresh join mode — skipping restore_state");
        true
    } else if device.restore_state(&*nv) {
        rtt_target::rprintln!("[EFR32] Restored — rejoin");
        false
    } else {
        rtt_target::rprintln!("[EFR32] No state — joining");
        true
    };

    static APP_CELL: StaticCell<SensorApp> = StaticCell::new();
    let app = APP_CELL.init(SensorApp::new(
        device,
        nv,
        basic_cluster,
        temp_cluster,
        hum_cluster,
        power_cluster,
        identify_cluster,
    ));
    app.needs_bootstrap_join = needs_bootstrap_join;
    app.run().await
}

#[cfg(feature = "diag-join")]
struct DiagJoinApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    basic_cluster: &'static mut BasicCluster,
    identify_cluster: &'static mut IdentifyCluster,
    last_join_attempt: Instant,
    join_attempts: u8,
    joined_at: Option<Instant>,
    last_annce_retry: Instant,
    annce_retries: u8,
    rx_frames: u32,
}

#[cfg(feature = "diag-join")]
impl DiagJoinApp {
    fn new(
        device: &'static mut ZigbeeDevice<Efr32Mac>,
        basic_cluster: &'static mut BasicCluster,
        identify_cluster: &'static mut IdentifyCluster,
    ) -> Self {
        Self {
            device,
            basic_cluster,
            identify_cluster,
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
            basic_cluster: self.basic_cluster as *mut BasicCluster,
            identify_cluster: self.identify_cluster as *mut IdentifyCluster,
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
            StackEvent::LeaveRequested => {
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
        if self.device.bdb_mut().initialize().await.is_err() {
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
            Ok(()) => rtt_target::rprintln!("[EFR32][diag-join] device_annce retry=ok"),
            Err(e) => rtt_target::rprintln!("[EFR32][diag-join] device_annce retry=err {:?}", e),
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
            Ok(()) => rtt_target::rprintln!(
                "[EFR32][diag-join] periodic device_annce {}=ok",
                self.annce_retries
            ),
            Err(e) => rtt_target::rprintln!(
                "[EFR32][diag-join] periodic device_annce {}=err {:?}",
                self.annce_retries,
                e
            ),
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
static DIAG_BASIC_CELL: StaticCell<BasicCluster> = StaticCell::new();
#[cfg(feature = "diag-join")]
static DIAG_IDENTIFY_CELL: StaticCell<IdentifyCluster> = StaticCell::new();
#[cfg(feature = "diag-join")]
static DIAG_DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
#[cfg(feature = "diag-join")]
static DIAG_APP_CELL: StaticCell<DiagJoinApp> = StaticCell::new();

#[cfg(feature = "diag-join")]
#[inline(always)]
fn init_basic_cluster() -> &'static mut BasicCluster {
    DIAG_BASIC_CELL.init_with(|| {
        let mut c = BasicCluster::new(b"Zigbee-RS", b"EFR32MG1-JoinDiag", b"20260422", b"0.1.0");
        c.set_power_source(0x03);
        c
    })
}

#[cfg(feature = "diag-join")]
#[inline(always)]
fn init_identify_cluster() -> &'static mut IdentifyCluster {
    DIAG_IDENTIFY_CELL.init_with(IdentifyCluster::new)
}

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
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000).cluster_server(0x0003)
        })
        .build_into(DIAG_DEVICE.uninit())
}

#[cfg(feature = "diag-join")]
#[inline(always)]
fn init_diag_app(
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    basic: &'static mut BasicCluster,
    identify: &'static mut IdentifyCluster,
) -> &'static mut DiagJoinApp {
    DIAG_APP_CELL.init_with(|| DiagJoinApp::new(device, basic, identify))
}

#[cfg(feature = "diag-join")]
#[inline(always)]
fn build_diag_app() -> &'static mut DiagJoinApp {
    rtt_target::rprintln!("[EFR32][diag-join] step=mac_new");
    let mac = Efr32Mac::new();
    rtt_target::rprintln!("[EFR32][diag-join] step=basic_cluster");
    let basic = init_basic_cluster();
    rtt_target::rprintln!("[EFR32][diag-join] step=identify_cluster");
    let identify = init_identify_cluster();
    rtt_target::rprintln!("[EFR32][diag-join] step=device_build");
    let device = init_zigbee_device(mac);
    rtt_target::rprintln!("[EFR32][diag-join] step=device_built");
    init_diag_app(device, basic, identify)
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
    init_logging();
    time_driver::init();
    gpio_set_output(pins::LED);
    led_off();
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
async fn diag_beacon_known_tx(mac: &mut Efr32Mac, seq: u8) {
    let _ = mac
        .mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(DIAG_SCAN_CHANNEL))
        .await;
    let beacon_req = build_beacon_request(seq);

    rtt_target::rprintln!(
        "[EFR32][diag-beacon] tx-test: raw-beacon-req ch={} len={} seq={}",
        DIAG_SCAN_CHANNEL,
        beacon_req.len(),
        seq
    );

    match mac.debug_transmit_raw(&beacon_req).await {
        Ok(_) => {
            rtt_target::rprintln!("[EFR32][diag-beacon] tx-test=ok");
        }
        Err(err) => {
            rtt_target::rprintln!("[EFR32][diag-beacon] tx-test=err {:?}", err);
        }
    }
}

#[cfg(feature = "diag-beacon")]
fn build_diag_broadcast_data(seq: u8, src_ext: &[u8; 8], marker: u8) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();
    let fc: u16 = 0xC841; // data, PAN compression, dst=short, src=extended
    let payload = [b'Z', b'B', b'R', b'S', marker];

    let _ = frame.extend_from_slice(&fc.to_le_bytes());
    let _ = frame.push(seq);
    let _ = frame.extend_from_slice(&0xFFFFu16.to_le_bytes()); // broadcast PAN
    let _ = frame.extend_from_slice(&0xFFFFu16.to_le_bytes()); // broadcast short addr
    let _ = frame.extend_from_slice(src_ext);
    let _ = frame.extend_from_slice(&payload);

    frame
}

#[cfg(feature = "diag-beacon")]
async fn diag_data_probe_tx(mac: &mut Efr32Mac, seq: u8, marker: u8) {
    let _ = mac
        .mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(DIAG_SCAN_CHANNEL))
        .await;
    let src_ext = mac.extended_address();
    let frame = build_diag_broadcast_data(seq, &src_ext, marker);

    rtt_target::rprintln!(
        "[EFR32][diag-beacon] tx-data: ch={} len={} seq={} marker={} src={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} payload=ZBRS{:02X}",
        DIAG_SCAN_CHANNEL,
        frame.len(),
        seq,
        marker,
        src_ext[0],
        src_ext[1],
        src_ext[2],
        src_ext[3],
        src_ext[4],
        src_ext[5],
        src_ext[6],
        src_ext[7],
        marker
    );

    match mac.debug_transmit_raw(&frame).await {
        Ok(_) => rtt_target::rprintln!("[EFR32][diag-beacon] tx-data=ok"),
        Err(err) => rtt_target::rprintln!("[EFR32][diag-beacon] tx-data=err {:?}", err),
    }
}

#[cfg(feature = "diag-beacon")]
#[embassy_executor::task]
async fn diag_beacon_task(mut mac: Efr32Mac) -> ! {
    let mut data_seq: u8 = 0x40;
    let mut data_marker: u8 = 0;

    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
    rtt_target::rprintln!("[EFR32][diag-beacon] Starting...");
    mac.debug_radio_snapshot("init");
    rtt_target::rprintln!(
        "[EFR32][diag-beacon] Radio initialized, active-scan on Zigbee channels 11-26"
    );

    rtt_target::rprintln!(
        "[EFR32][diag-beacon] pre-scan raw TX phase: {} burst(s)",
        DIAG_TX_PRESCAN_BURSTS
    );
    for _ in 0..DIAG_TX_PRESCAN_BURSTS {
        diag_beacon_known_tx(&mut mac, DIAG_TX_TEST_SEQ).await;
        diag_data_probe_tx(&mut mac, data_seq, data_marker).await;
        data_seq = data_seq.wrapping_add(1);
        data_marker = data_marker.wrapping_add(1);
        Timer::after(Duration::from_millis(500)).await;
    }

    let channel_mask = ChannelMask(DIAG_SCAN_MASK_2GHZ);

    loop {
        diag_beacon_known_tx(&mut mac, DIAG_TX_TEST_SEQ).await;
        diag_data_probe_tx(&mut mac, data_seq, data_marker).await;
        data_seq = data_seq.wrapping_add(1);
        data_marker = data_marker.wrapping_add(1);
        Timer::after(Duration::from_millis(250)).await;

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
    init_logging();
    time_driver::init();
    gpio_set_output(pins::LED);
    led_off();
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
        StackEvent::LeaveRequested => {
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
        0x0402,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0000),
            data_type: ZclDataType::I16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::I16(50)),
        },
    );

    // Humidity: report every 60-300s, min change 1% (100 centi-%)
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        0x0405,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0000),
            data_type: ZclDataType::U16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::U16(100)),
        },
    );

    // Battery: report every 300-3600s, min change 2% (4 in 0.5% units)
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        0x0001,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0021),
            data_type: ZclDataType::U8,
            min_interval: 300,
            max_interval: 3600,
            reportable_change: Some(ZclValue::U8(4)),
        },
    );

    log::info!("[EFR32] Default reporting configured (with change thresholds)");
}
