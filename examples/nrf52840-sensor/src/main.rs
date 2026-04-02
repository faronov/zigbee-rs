//! # Zigbee-RS nRF52840 Sensor (DK / J-Link)
//!
//! Embassy-based Zigbee 3.0 sleepy end device for the Nordic nRF52840-DK.
//! Flashed via probe-rs (J-Link). Supports external I2C sensors:
//!
//! | Feature         | Sensor  | Clusters                         |
//! |-----------------|---------|----------------------------------|
//! | (none)          | On-chip | Temp + fake humidity             |
//! | `sensor-bme280` | BME280  | Temp + humidity + pressure       |
//! | `sensor-sht31`  | SHT31   | Temp + humidity                  |
//!
//! ## Build & flash
//! ```sh
//! # On-chip only:
//! cargo build --release
//! probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
//!
//! # With BME280:
//! cargo build --release --features sensor-bme280
//! # With SHT31:
//! cargo build --release --features sensor-sht31
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
use embassy_nrf::temp::Temp;
#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "sensor-bme280")]
mod bme280;
mod flash_nv;
#[cfg(feature = "sensor-sht31")]
mod sht31;

// Bridge `log` crate → defmt so stack-internal log::info!/debug! appear in RTT output.
struct DefmtLogger;
impl log::Log for DefmtLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool { true }
    fn log(&self, record: &log::Record) {
        match record.level() {
            log::Level::Error => defmt::error!("{}", defmt::Display2Format(record.args())),
            log::Level::Warn  => defmt::warn!("{}", defmt::Display2Format(record.args())),
            log::Level::Info  => defmt::info!("{}", defmt::Display2Format(record.args())),
            log::Level::Debug => defmt::debug!("{}", defmt::Display2Format(record.args())),
            log::Level::Trace => defmt::trace!("{}", defmt::Display2Format(record.args())),
        }
    }
    fn flush(&self) {}
}
static LOGGER: DefmtLogger = DefmtLogger;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
#[cfg(feature = "sensor-bme280")]
use zigbee_zcl::clusters::pressure::PressureCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
const I2C_SENSOR_ADDR: u8 = {
    #[cfg(feature = "sensor-bme280")]
    { 0x76 }
    #[cfg(all(feature = "sensor-sht31", not(feature = "sensor-bme280")))]
    { 0x44 }
};

#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    SAADC => saadc::InterruptHandler;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// Ensure all RAM banks are powered on. POWER registers survive soft reset,
/// so a previous firmware run may have powered down banks the stack needs.
/// Runs as __pre_init — before .bss zero, .data copy, and main().
/// Pure assembly: zero stack usage (bank 8 section 5 may be powered off).
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

/// Power down unused RAM banks to reduce sleep current.
/// nRF52840 has 256 KB RAM in 9 banks; we keep only what's needed.
fn power_down_unused_ram() {
    extern "C" { static __sheap: u8; }
    let heap_start = unsafe { core::ptr::addr_of!(__sheap) as usize };
    let ram_start: usize = 0x2000_0000;
    let ram_end: usize = 0x2004_0000; // 256 KB
    let stack_reserve: usize = 8 * 1024;
    let used_end = heap_start;
    let keep_end = ram_end - stack_reserve;
    const POWER_BASE: usize = 0x4000_0900;

    // Banks 0-7: 8 KB each (2 sections of 4 KB)
    for bank in 0u32..8 {
        let bank_start = ram_start + (bank as usize) * 8192;
        let bank_end = bank_start + 8192;
        if bank_start >= used_end && bank_end <= keep_end {
            let powerclr = (POWER_BASE + (bank as usize) * 0x10 + 0x08) as *mut u32;
            unsafe { core::ptr::write_volatile(powerclr, 0x0003_0003); }
        }
    }
    // Bank 8: 192 KB (6 sections of 32 KB)
    let powerclr8 = (POWER_BASE + 8 * 0x10 + 0x08) as *mut u32;
    let mut mask8 = 0u32;
    for section in 0u32..6 {
        let section_start = ram_start + 64 * 1024 + (section as usize) * 32768;
        let section_end = section_start + 32768;
        if section_start >= used_end && section_end <= keep_end {
            mask8 |= (1u32 << section) | (1u32 << (section + 16));
        }
    }
    if mask8 != 0 {
        unsafe { core::ptr::write_volatile(powerclr8, mask8); }
    }
    let saved_kb = (keep_end.saturating_sub(used_end)) / 1024;
    info!("RAM: used ~{}KB, powered down ~{}KB", (used_end - ram_start) / 1024, saved_kb);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Debug);

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    power_down_unused_ram();
    info!("Zigbee-RS nRF52840 sensor starting…");

    // Button 1 on nRF52840-DK (P0.11, active low)
    let mut button = gpio::Input::new(p.P0_11, gpio::Pull::Up);

    // ── Sensor init ──
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    let mut temp_sensor = Temp::new(p.TEMP, Irqs);

    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
    let mut i2c = {
        let mut cfg = twim::Config::default();
        cfg.frequency = twim::Frequency::K400;
        Twim::new(p.TWISPI0, Irqs, p.P0_26, p.P0_27, cfg)
    };

    #[cfg(feature = "sensor-bme280")]
    let mut sensor_ok = bme280::init(&mut i2c, I2C_SENSOR_ADDR).await;
    #[cfg(feature = "sensor-bme280")]
    if sensor_ok { info!("BME280 ready"); } else { warn!("BME280 not found"); }

    #[cfg(feature = "sensor-sht31")]
    let mut sensor_ok = sht31::init(&mut i2c, I2C_SENSOR_ADDR).await;
    #[cfg(feature = "sensor-sht31")]
    if sensor_ok { info!("SHT31 ready"); } else { warn!("SHT31 not found"); }

    // SAADC for battery voltage
    let mut saadc_sensor = Saadc::new(
        p.SAADC, Irqs, saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    saadc_sensor.calibrate().await;

    // Radio + MAC
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio);
    mac.set_tx_power(8);
    info!("Radio ready");

    // ── Flash NV storage (last 2 pages of 1 MB flash) ──
    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut nv = flash_nv::FlashNvStorage::new(nvmc);
    info!("Flash NV storage ready");

    // ── ZCL clusters ──
    let mut basic_cluster = BasicCluster::new(b"Zigbee-RS", b"nRF52840-Sensor", b"20260401", b"0.1.0");
    basic_cluster.set_power_source(0x03);
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    #[cfg(feature = "sensor-bme280")]
    let mut press_cluster = PressureCluster::new(3000, 11000);
    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_size(4);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    let mut hum_tick: u32 = 0;

    // ── Build device ──
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy { poll_interval_ms: 10_000, wake_duration_ms: 500 })
        .manufacturer("Zigbee-RS")
        .model("nRF52840-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            let ep = ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0001)        // Power Configuration
                .cluster_server(0x0402)        // Temperature Measurement
                .cluster_server(0x0405);       // Relative Humidity
            #[cfg(feature = "sensor-bme280")]
            let ep = ep.cluster_server(0x0403); // Pressure Measurement
            ep
        })
        .build();

    // ── Restore previous network state from flash ──
    let restored = device.restore_state(&nv);
    if restored {
        info!("Restored network state from flash — will rejoin existing network");
        // Set MAC address filter so the radio accepts frames for our stored address
        device.user_action(UserAction::Join);
    } else {
        info!("No saved state — press Button 1 to join a network");
    }

    // Track whether we need to save state after next tick
    let mut needs_save = false;

    loop {
        match select3(
            device.receive(),
            button.wait_for_falling_edge(),
            Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)),
        ).await {
            // ── Incoming MAC frame ──
            Either3::First(Ok(indication)) => {
                #[cfg(feature = "sensor-bme280")]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                #[cfg(not(feature = "sensor-bme280"))]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    log_event(&event);
                    if matches!(event, StackEvent::Joined { .. }) {
                        needs_save = true;
                    }
                }
            }
            Either3::First(Err(_)) => { warn!("MAC receive error"); }

            // ── Button press → toggle join/leave ──
            Either3::Second(_) => {
                if device.is_joined() { info!("Button → leaving…"); }
                else { info!("Button → joining…"); }
                device.user_action(UserAction::Toggle);
                #[cfg(feature = "sensor-bme280")]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                #[cfg(not(feature = "sensor-bme280"))]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
                    log_event(e);
                    match e {
                        StackEvent::Joined { .. } => {
                            device.save_state(&mut nv);
                            info!("Network state saved to flash");
                        }
                        StackEvent::Left => {
                            device.factory_reset(Some(&mut nv)).await;
                            info!("NV storage cleared");
                        }
                        _ => {}
                    }
                }
                Timer::after(Duration::from_millis(300)).await;
            }

            // ── Report timer ──
            Either3::Third(_) => {
                if device.is_joined() {
                    // Read sensors
                    #[cfg(feature = "sensor-bme280")]
                    {
                        if !sensor_ok {
                            sensor_ok = bme280::init(&mut i2c, I2C_SENSOR_ADDR).await;
                            if sensor_ok { info!("BME280 recovered"); }
                        }
                        if sensor_ok {
                            if let Some(data) = bme280::read(&mut i2c, I2C_SENSOR_ADDR).await {
                                temp_cluster.set_temperature(data.temperature_centideg);
                                hum_cluster.set_humidity(data.humidity_centipct);
                                press_cluster.set_pressure(data.pressure_hpa as i16);
                                info!("T={}.{:02}°C H={}.{:02}% P={}hPa",
                                    data.temperature_centideg / 100,
                                    (data.temperature_centideg % 100).unsigned_abs(),
                                    data.humidity_centipct / 100, data.humidity_centipct % 100,
                                    data.pressure_hpa);
                            } else { warn!("BME280 read failed"); }
                        }
                    }

                    #[cfg(feature = "sensor-sht31")]
                    {
                        if !sensor_ok {
                            sensor_ok = sht31::init(&mut i2c, I2C_SENSOR_ADDR).await;
                            if sensor_ok { info!("SHT31 recovered"); }
                        }
                        if sensor_ok {
                            if let Some(data) = sht31::read(&mut i2c, I2C_SENSOR_ADDR).await {
                                temp_cluster.set_temperature(data.temperature_centideg);
                                hum_cluster.set_humidity(data.humidity_centipct);
                                info!("T={}.{:02}°C H={}.{:02}%",
                                    data.temperature_centideg / 100,
                                    (data.temperature_centideg % 100).unsigned_abs(),
                                    data.humidity_centipct / 100, data.humidity_centipct % 100);
                            } else { warn!("SHT31 read failed"); }
                        }
                    }

                    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
                    {
                        let raw_temp = temp_sensor.read().await;
                        let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;
                        hum_tick = hum_tick.wrapping_add(1);
                        let hum_hundredths = 5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);
                        temp_cluster.set_temperature(temp_hundredths);
                        hum_cluster.set_humidity(hum_hundredths);
                        info!("T={}.{:02}°C H={}.{:02}% (on-chip)",
                            temp_hundredths / 100, (temp_hundredths % 100).unsigned_abs(),
                            hum_hundredths / 100, hum_hundredths % 100);
                    }

                    // Battery
                    let mut buf = [0i16; 1];
                    saadc_sensor.sample(&mut buf).await;
                    let raw = buf[0].max(0) as u32;
                    let voltage_mv = raw * 3600 / 4096;
                    let pct = if voltage_mv <= 2200 { 0u8 }
                        else if voltage_mv >= 3300 { 200u8 }
                        else { ((voltage_mv - 2200) * 200 / 1100) as u8 };
                    power_cluster.set_battery_voltage((voltage_mv / 100) as u8);
                    power_cluster.set_battery_percentage(pct);
                }

                #[cfg(feature = "sensor-bme280")]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                #[cfg(not(feature = "sensor-bme280"))]
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(REPORT_INTERVAL_SECS as u16, &mut clusters).await {
                    log_event(e);
                }

                // Save state after successful join (deferred from receive path)
                if needs_save && device.is_joined() {
                    device.save_state(&mut nv);
                    needs_save = false;
                    info!("Network state saved to flash");
                }
            }
        }
    }
}

fn log_event(event: &StackEvent) {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            info!("Joined! addr=0x{:04X} ch={} pan=0x{:04X}", short_address, channel, pan_id);
        }
        StackEvent::Left => info!("Left network"),
        StackEvent::ReportSent => info!("Report sent"),
        StackEvent::CommissioningComplete { success } => {
            info!("Commissioning: {}", if *success { "ok" } else { "failed" });
        }
        _ => info!("Stack event"),
    }
}
