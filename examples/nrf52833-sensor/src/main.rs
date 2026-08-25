//! # Zigbee-RS nRF52833 Sensor (DK / J-Link)
//!
//! Embassy-based Zigbee 3.0 sleepy end device for the Nordic nRF52833-DK
//! (PCA10100). Flashed via probe-rs (J-Link). Supports external I2C
//! sensors:
//!
//! | Feature         | Sensor  | Clusters                         |
//! |-----------------|---------|----------------------------------|
//! | (none)          | On-chip | Temp + synthetic humidity        |
//! | `sensor-bme280` | BME280  | Temp + humidity + pressure       |
//! | `sensor-sht31`  | SHT31   | Temp + humidity                  |
//!
//! This file is only a composition root: it owns Embassy/Nordic platform
//! startup (clocks, DC-DC, RAM power state, boot signal), board resource
//! and sensor construction from the `nrf52833-dk` board and
//! `nrf52833-sensor-product` product crates, hardware AES install + startup
//! KAT, the crash-safe security journal, the concrete Zigbee profile, and
//! the identity guard — then hands all of it to
//! [`nrf_sensor_app::SensorApp`], which owns the full
//! commissioning/event-loop lifecycle *shared with the nRF52840 sensor*
//! through the compatibility adapter over `apps/sensor-sed`.
//! Endpoint/cluster composition, reporting defaults, and measurement mapping
//! live in the shared `zigbee_runtime::profile` archetype selected by the
//! product crate; NWK/APS/ZDO/BDB state machines live in `zigbee-runtime`.
//!
//! The device commissions automatically at boot and resumes silently after
//! a reset; Button 1 is only an operator override (short press =
//! immediate join while unjoined or a forced telemetry report while joined;
//! 3 s hold = durable factory reset + reboot).
//!
//! ## Build & flash
//! ```sh
//! # On-chip only:
//! cargo build --release
//! probe-rs run --chip nRF52833_xxAA target/thumbv7em-none-eabihf/release/nrf52833-sensor
//!
//! # With BME280:
//! cargo build --release --features sensor-bme280
//! # With SHT31:
//! cargo build --release --features sensor-sht31
//! ```

#![no_std]
#![no_main]

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
mod sensor;

use embassy_executor::Spawner;
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, peripherals, radio, rng};
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
use nrf_sensor_app::OnChipTemperature;
use nrf_sensor_app::{BatteryPolicy, SensorApp};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, BatteryMeasurement};
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

// Bridge `log` crate → defmt so stack-internal log::info!/warn!/error! from
// the MAC/NWK/APS layers appear in RTT output. Without this the whole
// networking stack is silent and commissioning failures are undiagnosable.
struct DefmtLogger;
impl log::Log for DefmtLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }
    fn log(&self, record: &log::Record) {
        match record.level() {
            log::Level::Error => defmt::error!("{}", defmt::Display2Format(record.args())),
            log::Level::Warn => defmt::warn!("{}", defmt::Display2Format(record.args())),
            log::Level::Info => defmt::info!("{}", defmt::Display2Format(record.args())),
            log::Level::Debug => defmt::debug!("{}", defmt::Display2Format(record.args())),
            log::Level::Trace => defmt::trace!("{}", defmt::Display2Format(record.args())),
        }
    }
    fn flush(&self) {}
}
static LOGGER: DefmtLogger = DefmtLogger;

/// Binds this product's battery chemistry (`products/nrf52833-sensor`) to
/// the shared application. Zero-sized: the calls monomorphize into the
/// product's own arithmetic.
struct Battery;

impl BatteryPolicy for Battery {
    fn millivolts(raw_sample: i16) -> u32 {
        nrf52833_sensor_product::battery::millivolts(raw_sample)
    }

    fn measurement(raw_sample: i16) -> BatteryMeasurement {
        nrf52833_sensor_product::battery::battery_measurement(raw_sample)
    }
}

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    SAADC => saadc::InterruptHandler;
    TWISPI0 => embassy_nrf::twim::InterruptHandler<peripherals::TWISPI0>;
});

#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

// Ensure all RAM banks are powered on. POWER registers survive soft reset,
// so a previous firmware run may have powered down banks the stack needs.
// Runs as __pre_init before .bss zero, .data copy, and main().
// Pure assembly: zero stack usage (the stack itself may be in a bank that
// is currently powered off).
//
// nRF52833 RAM: banks 0-7 (8 KiB each) + bank 8 (64 KiB) = 128 KiB. The
// nRF52840 firmware writes the same nine POWERSET registers; writing all
// ones to a bank with fewer sections than nRF52840's bank 8 is harmless
// (reserved bits are ignored), which keeps this sequence identical across
// both nRF products.
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904", // POWER.RAM[0].POWERSET
    "mvn r1, #0",          // r1 = 0xFFFFFFFF
    "str r1, [r0, #0x00]", // RAM[0].POWERSET
    "str r1, [r0, #0x10]", // RAM[1].POWERSET
    "str r1, [r0, #0x20]", // RAM[2].POWERSET
    "str r1, [r0, #0x30]", // RAM[3].POWERSET
    "str r1, [r0, #0x40]", // RAM[4].POWERSET
    "str r1, [r0, #0x50]", // RAM[5].POWERSET
    "str r1, [r0, #0x60]", // RAM[6].POWERSET
    "str r1, [r0, #0x70]", // RAM[7].POWERSET
    "str r1, [r0, #0x80]", // RAM[8].POWERSET
    "bx lr",
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Debug);

    let mut config = embassy_nrf::config::Config::default();
    // HFCLK from the DK's external crystal — required for the 802.15.4 radio.
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // Enable the DC/DC converter (PCA10100 has the inductors fitted) for a
    // significantly lower average current draw.
    //
    // Hardware difference vs. the nRF52840 product: `embassy-nrf` exposes
    // `reg0` (the VDDH→VDD high-voltage stage) only for nRF52840, so this
    // build configures the second stage only. The DK runs in normal-voltage
    // mode with VDDH tied to VDD, where REG0 is bypassed anyway.
    config.dcdc = embassy_nrf::config::DcdcConfig { reg1: true };
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52833 sensor starting…");

    // LED1 / Button 1 (board-owned physical wiring).
    let mut led = nrf52833_dk::led(p.P0_13);
    let button = nrf52833_dk::button(p.P0_11);

    // Boot signal: LED solid ON 2 seconds
    led.set_low(); // active LOW = ON
    Timer::after(Duration::from_secs(2)).await;
    led.set_high(); // OFF
    Timer::after(Duration::from_millis(500)).await;

    // ── Sensor init ──
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    let environment = OnChipTemperature::new(Temp::new(p.TEMP, Irqs));

    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
    let environment =
        sensor::Sensor::new(nrf52833_dk::sensor_i2c(p.TWISPI0, Irqs, p.P0_26, p.P0_27));

    // SAADC for battery voltage
    let saadc_sensor = Saadc::new(
        p.SAADC,
        Irqs,
        saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    saadc_sensor.calibrate().await;

    // Radio + MAC
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let rng = rng::Rng::new(p.RNG, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio, rng);
    let Some(aes) = zigbee_mac::nrf::NrfEcbToken::take() else {
        error!("Nordic ECB AES already owned; networking halted");
        loop {
            cortex_m::asm::wfi();
        }
    };
    if mac.install_aes_engine(aes).is_err() {
        error!("Nordic ECB AES startup KAT failed; networking halted");
        loop {
            cortex_m::asm::wfi();
        }
    }
    // Factory-programmed FICR DEVICEID-derived EUI-64 — never a constant.
    let ieee = mac.extended_address();
    info!("Nordic ECB hardware AES KAT passed");
    mac.set_tx_power(0);
    info!("Radio ready (TX 0 dBm)");

    // ── Product profile (endpoint, clusters, reporting defaults) ──
    //
    // `device`, `security_store`, `profile`, and `app` below are plain
    // locals, not `StaticCell`s, for the same reason as the nRF52840
    // firmware: this is a single `#[embassy_executor::main]` future and the
    // executor's task arena (`task-arena-size-32768`) is a fixed static
    // reservation regardless of what the future stores, so a `StaticCell`
    // would only add a second reservation on top of it.
    let mut profile = nrf52833_sensor_product::profile::sensor_profile();

    // ── Build device ──
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer(nrf52833_sensor_product::MANUFACTURER)
        .model(nrf52833_sensor_product::MODEL)
        .date_code(nrf52833_sensor_product::DATE_CODE)
        .sw_build(nrf52833_sensor_product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build();

    // ── Atomic security journal (last 8 KiB of 512 KiB flash) ──
    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut security_store = nrf52833_sensor_product::storage::security_store(nvmc);
    info!("Security journal ready");

    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => warn!("Cleared persisted network state after IEEE address change"),
        Ok(false) => {}
        Err(error) => nrf_sensor_app::persistence_failure(error),
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);

    let mut app: SensorApp<'_, _, _, _, Battery> =
        SensorApp::new(node, led, button, environment, saadc_sensor);

    app.run().await
}
