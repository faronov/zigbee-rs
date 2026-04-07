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

#[cfg(feature = "stubs")]
mod stubs;

mod time_driver;
mod vectors;
mod flash_nv;

use cortex_m as _;
use panic_halt as _;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;

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
        0x13, 0xb7, 0x79, 0xfa,
        0xc9, 0x25, 0xdd, 0xb7,
        0xad, 0xf3, 0xcf, 0xe0,
        0xf1, 0xb6, 0x14, 0xb8,
    ],
    struct_version: 0x0000_0100,     // Version 1.0
    signature_type: 0,                // APPLICATION_SIGNATURE_NONE
    signature_location: 0xFFFF_FFFF,  // No signature
    app_type: 1,                      // APPLICATION_TYPE_ZIGBEE
    app_version: 1,                   // Version 1
    app_capabilities: 0,
    app_product_id: [0u8; 16],
};

// ── VTOR setup ──────────────────────────────────────────────────
//
// When booting via Gecko Bootloader at 0x4000, uncomment this to
// redirect the vector table. For bare-metal boot at 0x0, VTOR
// defaults to 0x0 which is correct.

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
        core::ptr::write_volatile(0x20000000 as *mut u32, 0xDEAD_BEEF);
        core::ptr::write_volatile(0x20000004 as *mut u32, ef.pc() as u32);
        core::ptr::write_volatile(0x20000008 as *mut u32, ef.lr() as u32);
        core::ptr::write_volatile(0x2000000C as *mut u32, ef.xpsr() as u32);
        core::ptr::write_volatile(0x20000010 as *mut u32, msp);
        core::ptr::write_volatile(0x20000014 as *mut u32, ef.r0() as u32);
        core::ptr::write_volatile(0x20000018 as *mut u32, ef.r12() as u32);
    }
    loop { cortex_m::asm::nop(); }
}

// Set VTOR to 0x4000 — required when Gecko Bootloader is present.
// The bootloader at 0x0 jumps to our app at 0x4000, but cortex-m-rt
// reset handler may run before VTOR is properly set.

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::efr32::Efr32Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3; // PowerConfig + Temp + Humidity

// ── EFR32MG1P GPIO ─────────────────────────────────────────────

mod pins {
    // GPIO pin numbers — adjust for your board
    pub const LED: u8 = 6;  // LED on PF6 (Thunderboard Sense)
    pub const BTN: u8 = 7;  // Button on PF7 (Thunderboard Sense)
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

fn gpio_read(pin: u8) -> bool {
    let port = (pin / 16) as u32;
    let pin_in_port = (pin % 16) as u32;
    let din_reg = 0x4000_A010 + port * 0x30; // DIN
    let val = unsafe { core::ptr::read_volatile(din_reg as *const u32) };
    (val >> pin_in_port) & 1 != 0
}

fn led_on() { gpio_write(pins::LED, true); }
fn led_off() { gpio_write(pins::LED, false); }

// ── Main ────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_print!();
    time_driver::init();

    // Unmask radio IRQ (FRC_PRI = IRQ 1)
    unsafe {
        // Enable radio NVIC interrupts:
        // - FrcPri (IRQ 1): high-priority FRC events (TX/RX done)
        // - Frc (IRQ 3): regular FRC events 
        // - Bufc (IRQ 7): buffer controller events
        // NOTE: RAC_SEQ (IRQ 5) and RAC_RSM (IRQ 6) are NOT enabled —
        // they fire continuously and starve the main executor loop.
        // TX/RX completion is signaled via FRC interrupts instead.
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::FrcPri);
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::Frc);
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::Bufc);
    }

    rtt_target::rprintln!("[EFR32] Starting...");

    // GPIO init
    gpio_set_output(pins::LED);
    led_off();

    // Boot signal: triple blink
    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;

    // Radio + MAC
    let mac = Efr32Mac::new();
    rtt_target::rprintln!("[EFR32] Radio ready");

    // Flash NV storage — static to reduce async future size
    static NV_CELL: StaticCell<flash_nv::Nv> = StaticCell::new();
    let nv = NV_CELL.init(flash_nv::create_nv());
    rtt_target::rprintln!("[EFR32] NV ready");

    // ZCL clusters — static to reduce async future size
    static BASIC_CELL: StaticCell<BasicCluster> = StaticCell::new();
    let basic_cluster = BASIC_CELL.init({
        let mut c = BasicCluster::new(
            b"Zigbee-RS",
            b"EFR32MG1-Sensor",
            b"20260402",
            b"0.1.0",
        );
        c.set_power_source(0x03); // Battery
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
    power_cluster.set_battery_size(4);     // AAA
    power_cluster.set_battery_quantity(2); // 2× AAA
    power_cluster.set_battery_rated_voltage(15); // 1.5V

    // Simulated sensor state
    let mut hum_tick: u32 = 0;

    // Build device (SED) — place in static to reduce async future size.
    // Without this, the entire ZigbeeDevice (~10KB+) gets inlined into
    // the main async future, requiring a 20KB+ arena on a 32KB SRAM chip.
    // With StaticCell, only a &mut reference (4 bytes) is in the future.
    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
    let device = DEVICE.init(ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build());

    // Restore previous network state from flash
    let restored = device.restore_state(&*nv);
    if restored {
        rtt_target::rprintln!("[EFR32] Restored — rejoin");
        device.user_action(UserAction::Rejoin);
    } else {
        rtt_target::rprintln!("[EFR32] No state — joining");
        device.user_action(UserAction::Join);
    }
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
        ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
        ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
        ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
        ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
    ];
    rtt_target::rprintln!("[EFR32] First tick...");
    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
        rtt_target::rprintln!("[EFR32] First tick event: {:?}", core::mem::discriminant(e));
        if log_event(e) {
            device.save_state(&mut *nv);
        }
    }
    rtt_target::rprintln!("[EFR32] First tick done, joined={}", device.is_joined());

    // Default reporting so device reports even before ZHA interview
    setup_default_reporting(device);

    // Set initial sensor values
    {
        let temp: i16 = 2250;
        temp_cluster.set_temperature(temp);
        hum_cluster.set_humidity(5000u16);

        // Simulated battery (replace with real ADC reading)
        let batt_pct: u8 = 100;
        power_cluster.set_battery_voltage(30); // 3.0V
        power_cluster.set_battery_percentage(batt_pct * 2);
        log::info!("[EFR32] Initial: T=22.50°C H=50.00% Batt=3000mV (100%)");
    }

    // ── Main loop state ──
    let mut last_report = Instant::now();
    let mut fast_poll_until = if device.is_joined() {
        log::info!("[EFR32] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
        led_on();
        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
    } else {
        Instant::now()
    };
    let mut last_rejoin_attempt = Instant::now();
    let mut rejoin_count: u8 = 0;
    let mut annce_retries_left: u8 = if device.is_joined() { 5 } else { 0 };
    let mut last_annce = Instant::now();
    let mut was_fast_polling = device.is_joined();
    let mut interview_done = false;
    let mut button_was_pressed = false;
    let mut needs_save = false;

    loop {
        let now = Instant::now();
        let in_fast_poll = now < fast_poll_until;
        let poll_ms = if in_fast_poll { FAST_POLL_MS } else { SLOW_POLL_SECS * 1000 };

        // ── Rejoin if not joined ──
        if !device.is_joined() && now.duration_since(last_rejoin_attempt).as_secs() > 15 {
            rtt_target::rprintln!("[EFR32] Retry join...");
            last_rejoin_attempt = now;
            rejoin_count += 1;
            device.user_action(UserAction::Join);
        }

        if was_fast_polling && !in_fast_poll {
            let cfg = device.configured_cluster_count(1);
            log::info!("[EFR32] Fast poll OFF — {}/{} clusters configured", cfg, EXPECTED_REPORT_CLUSTERS);
            was_fast_polling = false;
            if !interview_done {
                led_off();
            }
        } else if in_fast_poll {
            was_fast_polling = true;
        }

        // ── Button check ──
        let pressed = !gpio_read(pins::BTN); // active LOW
        if pressed && !button_was_pressed {
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
                device.factory_reset(Some(&mut *nv)).await;
                log::info!("[EFR32] NV cleared — rebooting");
                for _ in 0..5u8 {
                    led_on();
                    Timer::after(Duration::from_millis(100)).await;
                    led_off();
                    Timer::after(Duration::from_millis(100)).await;
                }
                cortex_m::peripheral::SCB::sys_reset();
            } else {
                log::info!("[EFR32] Button → {}", if device.is_joined() { "leave" } else { "join" });
                device.user_action(UserAction::Toggle);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut cls).await {
                    match e {
                        StackEvent::Joined { .. } => {
                            log_event(e);
                            device.save_state(&mut *nv);
                            log::info!("[EFR32] State saved to flash");
                            fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                            annce_retries_left = 5;
                            last_annce = Instant::now();
                            interview_done = false;
                        }
                        StackEvent::Left => {
                            log_event(e);
                            device.factory_reset(Some(&mut *nv)).await;
                            log::info!("[EFR32] NV cleared");
                        }
                        _ => { log_event(e); }
                    }
                }
                Timer::after(Duration::from_millis(300)).await;
            }
        }
        button_was_pressed = pressed;

        // ── Sleep until next poll ──
        // Light sleep: radio off, CPU in WFE via embassy Timer
        device.mac_mut().radio_sleep();
        Timer::after(Duration::from_millis(poll_ms)).await;
        device.mac_mut().radio_wake();

        // ── Poll parent for indirect frames (SED core) ──
        if device.is_joined() {
            for _poll_round in 0..4u8 {
                match device.poll().await {
                    Ok(Some(ind)) => {
                        let mut cls = [
                            ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
                        ];
                        if let Some(ev) = device.process_incoming(&ind, &mut cls).await {
                            match &ev {
                                StackEvent::LeaveRequested => {
                                    log::info!("[EFR32] Leave requested — erasing NV and rejoining");
                                    device.factory_reset(Some(&mut *nv)).await;
                                    device.user_action(UserAction::Join);
                                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                    interview_done = false;
                                    annce_retries_left = 5;
                                    last_annce = Instant::now();
                                    led_on();
                                    break;
                                }
                                _ => {}
                            }
                            if log_event(&ev) {
                                fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                log::info!("[EFR32] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
                                needs_save = true;
                            }
                        }
                        if !interview_done {
                            let cfg_count = device.configured_cluster_count(1);
                            if cfg_count >= EXPECTED_REPORT_CLUSTERS {
                                log::info!("[EFR32] Interview done! {}/{} clusters", cfg_count, EXPECTED_REPORT_CLUSTERS);
                                fast_poll_until = Instant::now() + Duration::from_secs(5);
                                interview_done = true;
                                led_off();
                            }
                        }
                        let mut cls2 = [
                            ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
                        ];
                        let _ = device.tick(0, &mut cls2).await;
                    }
                    Ok(None) => break,
                    Err(_) => break,
                }
            }

            // ── Periodic sensor readings ──
            let now2 = Instant::now();
            let elapsed_s = now2.duration_since(last_report).as_secs();

            if elapsed_s >= REPORT_INTERVAL_SECS {
                last_report = now2;

                // Simulated temp/humidity (replace with I2C sensor)
                let temp_hundredths: i16 = 2250 + ((hum_tick % 50) as i16 - 25);
                hum_tick = hum_tick.wrapping_add(1);
                let hum_hundredths: u16 = 5000 + ((hum_tick % 100) as u16) * 10;
                temp_cluster.set_temperature(temp_hundredths);
                hum_cluster.set_humidity(hum_hundredths);
                log::info!(
                    "[EFR32] T={}.{:02}°C H={}.{:02}%",
                    temp_hundredths / 100,
                    (temp_hundredths % 100).unsigned_abs(),
                    hum_hundredths / 100,
                    hum_hundredths % 100,
                );
            }

            let tick_elapsed = elapsed_s.min(60) as u16;
            let mut clusters = [
                ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
                ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
                ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
                ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
            ];
            if let TickResult::Event(ref e) = device.tick(tick_elapsed, &mut clusters).await {
                if log_event(e) {
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                }
            }

            // Identify LED blink
            identify_cluster.tick(tick_elapsed);
            if identify_cluster.is_identifying() {
                let on = gpio_read(pins::LED);
                gpio_write(pins::LED, !on);
            }

            // Device_annce retry
            if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                annce_retries_left -= 1;
                last_annce = now2;
                log::info!("[EFR32] Device_annce retry ({} left)", annce_retries_left);
                let _ = device.send_device_annce().await;
            }

            if needs_save {
                needs_save = false;
                device.save_state(&mut *nv);
                log::info!("[EFR32] State saved to flash (deferred)");
            }
        } else {
            // ── Not joined — blink and auto-retry ──
            let now2 = Instant::now();
            if now2.duration_since(last_rejoin_attempt).as_secs() >= 1 {
                led_on();
                Timer::after(Duration::from_millis(80)).await;
                led_off();
                Timer::after(Duration::from_millis(120)).await;
                led_on();
                Timer::after(Duration::from_millis(80)).await;
                led_off();
            }

            if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                rejoin_count = rejoin_count.wrapping_add(1);
                last_rejoin_attempt = Instant::now();
                log::info!("[EFR32] Not joined — retrying (attempt {})…", rejoin_count);
                device.user_action(UserAction::Join);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut *basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *power_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut *identify_cluster },
                ];
                let _ = device.tick(0, &mut cls).await;
                if device.is_joined() {
                    log::info!("[EFR32] Joined! addr=0x{:04X}", device.short_address());
                    led_on();
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    annce_retries_left = 5;
                    last_annce = Instant::now();
                    interview_done = false;
                    device.save_state(&mut *nv);
                    log::info!("[EFR32] State saved to flash");
                }
            }
        }
    }
}

/// Log stack events. Returns true on join event.
fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            led_on();
            log::info!(
                "[EFR32] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id
            );
            true
        }
        StackEvent::Left => {
            led_off();
            log::info!("[EFR32] Left network");
            false
        }
        StackEvent::ReportSent => { log::info!("[EFR32] Report sent"); false }
        StackEvent::LeaveRequested => {
            led_on();
            log::info!("[EFR32] Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            log::info!("[EFR32] Commissioning: {}", if *success { "ok" } else { "failed" });
            false
        }
        _ => { log::info!("[EFR32] Stack event"); false }
    }
}

/// Configure default reporting intervals with reportable change thresholds.
fn setup_default_reporting(device: &mut ZigbeeDevice<Efr32Mac>) {
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};

    // Temperature: report every 60-300s, min change 0.5°C (50 centidegrees)
    let _ = device.reporting_mut().configure_for_cluster(
        1, 0x0402,
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
        1, 0x0405,
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
        1, 0x0001,
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
