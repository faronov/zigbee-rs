//! # Zigbee-RS nRF52833 Temperature Sensor
//!
//! Embassy-based Zigbee 3.0 end device for the Nordic nRF52833 that reports
//! real temperature data from the on-chip TEMP sensor, plus simulated humidity.
//!
//! ## Hardware
//! - nRF52833-DK (or any nRF52833 board)
//! - Button 1 (P0.11): press to join / leave the Zigbee network
//!
//! ## Operation
//! 1. Power on → device starts idle (not joined)
//! 2. Press Button 1 → initiates BDB commissioning (network join)
//! 3. Once joined: reads temperature every 30 s and ticks the stack
//! 4. Press Button 1 again → leaves the network

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

/// Ensure all RAM banks are powered on. POWER registers survive soft reset,
/// so a previous firmware run may have powered down banks the stack needs.
/// Pure assembly: zero stack usage (stack RAM may be powered off).
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904",  // POWER.RAM[0].POWERSET
    "mvn r1, #0",           // r1 = 0xFFFFFFFF
    "str r1, [r0, #0x00]",  // RAM[0].POWERSET
    "str r1, [r0, #0x10]",  // RAM[1].POWERSET
    "str r1, [r0, #0x20]",  // RAM[2].POWERSET
    "str r1, [r0, #0x30]",  // RAM[3].POWERSET
    "str r1, [r0, #0x40]",  // RAM[4].POWERSET
    "str r1, [r0, #0x50]",  // RAM[5].POWERSET
    "str r1, [r0, #0x60]",  // RAM[6].POWERSET
    "str r1, [r0, #0x70]",  // RAM[7].POWERSET
    "str r1, [r0, #0x80]",  // RAM[8].POWERSET
    "bx lr",
);

/// Power down unused RAM sections to reduce sleep current.
/// nRF52833 has 128 KB RAM: banks 0–7 (8×8KB, 2 sections each) + bank 8 (64KB, 2×32KB sections).
fn power_down_unused_ram() {
    extern "C" {
        static __sheap: u8;
    }
    struct RamBank { start: u32, section_count: u8, section_size: u32 }
    const BANKS: [RamBank; 9] = [
        RamBank { start: 0x2000_0000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_2000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_4000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_6000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_8000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_A000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_C000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2000_E000, section_count: 2, section_size: 0x1000 },
        RamBank { start: 0x2001_0000, section_count: 2, section_size: 0x8000 },
    ];
    let ram_used_end = unsafe { &__sheap as *const u8 as u32 };
    let stack_bottom: u32 = 0x2002_0000 - 8 * 1024; // 128 KB RAM top
    let power = 0x4000_0000u32;
    let mut total_saved: u32 = 0;
    for (bank_idx, bank) in BANKS.iter().enumerate() {
        for section in 0..bank.section_count {
            let section_start = bank.start + (section as u32) * bank.section_size;
            let section_end = section_start + bank.section_size;
            if section_start >= ram_used_end && section_end <= stack_bottom {
                let powerclr = (power + 0x900 + (bank_idx as u32) * 0x10 + 0x08) as *mut u32;
                let mask = (1u32 << section) | (1u32 << (section + 16));
                unsafe { core::ptr::write_volatile(powerclr, mask) };
                total_saved += bank.section_size;
            }
        }
    }
    info!("RAM power-down: used={} KB, saved={} KB", ram_used_end.wrapping_sub(0x2000_0000) / 1024, total_saved / 1024);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // HFCLK from external crystal — REQUIRED for 802.15.4 radio
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52833 sensor starting…");

    // On-chip temperature sensor (real hardware reading)
    let mut temp_sensor = Temp::new(p.TEMP, Irqs);

    // SAADC: measure VDD — default 12-bit, ref=0.6V, gain=1/6 → range 3.6V
    let mut saadc_sensor = Saadc::new(
        p.SAADC, Irqs, saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    saadc_sensor.calibrate().await;

    // Button 1 on nRF52833-DK (P0.11, active low with on-board pull-up)
    let mut button = gpio::Input::new(p.P0_11, gpio::Pull::Up);

    // IEEE 802.15.4 MAC driver
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mac = zigbee_mac::nrf::NrfMac::new(radio);

    info!("Radio ready");

    // ZCL cluster instances
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"nRF52833-Sensor",
        b"20260331",
        b"0.1.0",
    );
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    let mut power_cluster = PowerConfigCluster::new();
    let mut identify_cluster = IdentifyCluster::new();
    power_cluster.set_battery_size(4);     // AAA
    power_cluster.set_battery_quantity(2); // 2× AAA
    power_cluster.set_battery_rated_voltage(15); // 1.5V per cell

    // Simulated humidity baseline (no on-chip humidity sensor)
    let mut hum_tick: u32 = 0;

    // Build the Zigbee device
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("nRF52833-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    info!("Device ready — press Button 1 to join/leave");

    loop {
        match select3(
            device.receive(),
            button.wait_for_falling_edge(),
            Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)),
        )
        .await
        {
            // ── Incoming MAC frame ──────────────────────────────
            Either3::First(Ok(indication)) => {
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    log_event(&event);
                }
            }
            Either3::First(Err(_)) => {
                warn!("MAC receive error");
            }

            // ── Button press → toggle join/leave ────────────────
            Either3::Second(_) => {
                if device.is_joined() {
                    info!("Button → leaving network…");
                } else {
                    info!("Button → joining network…");
                }
                device.user_action(UserAction::Toggle);
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
                    log_event(e);
                }
                // Debounce
                Timer::after(Duration::from_millis(300)).await;
            }

            // ── Report timer ────────────────────────────────────
            Either3::Third(_) => {
                if device.is_joined() {
                    // Read on-chip temperature (returns fixed-point °C)
                    let raw_temp = temp_sensor.read().await;
                    let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;

                    // Simulated humidity (slowly varies 45–55 %)
                    hum_tick = hum_tick.wrapping_add(1);
                    let hum_hundredths =
                        5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);

                    temp_cluster.set_temperature(temp_hundredths);
                    hum_cluster.set_humidity(hum_hundredths);

                    // Battery: read VDD via SAADC
                    let mut buf = [0i16; 1];
                    saadc_sensor.sample(&mut buf).await;
                    let raw = buf[0].max(0) as u32;
                    let voltage_mv = raw * 3600 / 4096;
                    let pct = if voltage_mv >= 3000 { 100u8 }
                              else if voltage_mv <= 1800 { 0 }
                              else { ((voltage_mv - 1800) * 100 / 1200) as u8 };
                    power_cluster.set_battery_voltage((voltage_mv / 100) as u8);
                    power_cluster.set_battery_percentage(pct * 2); // ZCL: 0.5% units

                    info!(
                        "T={}.{:02}°C  H={}.{:02}%  Bat={}mV ({}%)",
                        temp_hundredths / 100,
                        (temp_hundredths % 100).unsigned_abs(),
                        hum_hundredths / 100,
                        hum_hundredths % 100,
                        voltage_mv,
                        pct,
                    );
                }
                if let TickResult::Event(ref e) =
                    device.tick(REPORT_INTERVAL_SECS as u16, &mut [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                    ]).await
                {
                    log_event(e);
                }
            }
        }
    }
}

fn log_event(event: &StackEvent) {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id,
            );
        }
        StackEvent::Left => info!("Left network"),
        StackEvent::ReportSent => info!("Report sent"),
        StackEvent::CommissioningComplete { success } => {
            info!(
                "Commissioning: {}",
                if *success { "ok" } else { "failed" }
            );
        }
        _ => info!("Stack event"),
    }
}
