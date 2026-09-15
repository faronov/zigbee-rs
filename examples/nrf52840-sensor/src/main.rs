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
//! This file is only a composition root: it owns Embassy/Nordic platform
//! startup (clocks, DC-DC, RAM power state, boot signal), board resource
//! and sensor construction from the `nrf52840-dk` board and
//! `nrf52840-sensor-product` product crates, hardware AES install + startup
//! KAT, the crash-safe security journal, the concrete Zigbee profile, and
//! the identity guard — then hands all of it to
//! [`sensor_sed_app::SensorApp`], which owns the full
//! commissioning/event-loop lifecycle (see `apps/sensor-sed`).
//! Endpoint/cluster composition, reporting defaults, and measurement mapping
//! live in the shared `zigbee_runtime::profile` archetype selected by the
//! product crate; NWK/APS/ZDO/BDB state machines live in `zigbee-runtime`.
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
use nrf_sensor_app::{
    BatteryPolicy, NrfBattery, NrfDiagnostics, NrfStatus, NrfSupervisor, NrfWakeController,
};
use sensor_sed_app::{NoOta, SensorApp, SensorSedParts};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{ApplicationProfile, BatteryMeasurement};
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

/// Binds this product's battery chemistry (`products/nrf52840-sensor`) to
/// the shared application. Zero-sized: the calls monomorphize back into the
/// same direct arithmetic the firmware inlined before the extraction.
struct Battery;

impl BatteryPolicy for Battery {
    fn millivolts(raw_sample: i16) -> u32 {
        nrf52840_sensor_product::battery::millivolts(raw_sample)
    }

    fn measurement(raw_sample: i16) -> BatteryMeasurement {
        nrf52840_sensor_product::battery::battery_measurement(raw_sample)
    }
}

// Bridge `log` crate → defmt so stack-internal log::info!/debug! appear in RTT output.
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
// Pure assembly: zero stack usage (bank 8 section 5 may be powered off).
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

#[allow(dead_code)]
/// Power down unused high RAM banks to reduce sleep current.
///
/// nRF52840 RAM layout: Banks 0-7 (8KB each, 64KB total) + Bank 8 (6×32KB, 192KB).
/// Embassy allocates task stacks from the top of RAM downward, so we can only
/// safely power down Bank 8 sections that are clearly above any possible stack use.
/// For a SED sensor (~37KB BSS + 8KB stack), banks 0-7 (64KB) are sufficient.
/// Bank 8 (0x20010000-0x20040000, 192KB) can be fully powered down.
///
/// Not currently called (kept from the original firmware unchanged): wiring
/// this in requires re-verifying stack/BSS headroom on hardware first.
fn power_down_unused_ram() {
    // Power down entire Bank 8 (192KB in 6 sections of 32KB)
    // Bank 8 starts at 0x20010000 — well above our ~37KB BSS + stack
    const POWER_BASE: usize = 0x4000_0900;
    let powerclr8 = (POWER_BASE + 8 * 0x10 + 0x08) as *mut u32;
    // All 6 sections off (bits 0-5 for power, bits 16-21 for retention)
    unsafe {
        core::ptr::write_volatile(powerclr8, 0x003F_003F);
    }
    info!("RAM: powered down 192KB (bank 8)");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Debug);

    let mut config = embassy_nrf::config::Config::default();
    // HFCLK from the DK's external crystal — required for the 802.15.4 radio.
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // Enable DC-DC converter for ~40% lower current draw
    config.dcdc = embassy_nrf::config::DcdcConfig {
        reg0: true,
        reg0_voltage: None, // keep UICR default
        reg1: true,
    };
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52840 sensor starting…");

    // LED1 / Button 1 (board-owned physical wiring).
    let mut led = nrf52840_dk::led(p.P0_13);
    let button = nrf52840_dk::button(p.P0_11);

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
        sensor::Sensor::new(nrf52840_dk::sensor_i2c(p.TWISPI0, Irqs, p.P0_26, p.P0_27));

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
    let ieee = mac.extended_address();
    info!("Nordic ECB hardware AES KAT passed");
    mac.set_tx_power(0);
    info!("Radio ready (TX 0 dBm)");

    // ── Product profile (endpoint, clusters, reporting defaults) ──
    //
    // `device`, `security_store`, `profile`, and `app` below are plain
    // locals, not `StaticCell`s: this whole firmware is a single
    // `#[embassy_executor::main]` future (never `Spawner::spawn`-ed
    // alongside others), and `embassy-executor`'s task arena
    // (`task-arena-size-32768`, see `Cargo.toml`) is a fixed-size static
    // reservation regardless of what is stored inside the future — moving
    // these into `StaticCell`s would only add a second, *additional*
    // reservation on top of that unchanged 32 KiB arena instead of
    // shrinking it, a net RAM cost with no benefit here. (That pattern
    // pays off on the EFR32MG1/ESP32-H2 sensors specifically because they
    // do *not* size the arena up for this data — they use the crate's
    // default 4 KiB arena — so pulling large owned state out of the future
    // is required just to fit.) Measured effect of *not* using
    // `StaticCell` here: identical `.bss` to the pre-refactor single-file
    // firmware (see the nRF platform guide for the exact byte counts).
    let mut profile = nrf52840_sensor_product::profile::sensor_profile();

    // ── Build device ──
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(nrf52840_sensor_product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(nrf52840_sensor_product::MANUFACTURER)
        .model(nrf52840_sensor_product::MODEL)
        .date_code(nrf52840_sensor_product::DATE_CODE)
        .sw_build(nrf52840_sensor_product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build();

    // ── Atomic security journal (last 8 KiB of 1 MiB flash) ──
    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut security_store = nrf52840_sensor_product::storage::security_store(nvmc);
    info!("Security journal ready");

    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => warn!("Cleared persisted network state after IEEE address change"),
        Ok(false) => {}
        Err(error) => nrf_sensor_app::persistence_failure(error),
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);

    let mut app = SensorApp::new(
        node,
        &nrf52840_sensor_product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: NrfWakeController::new(button),
            status: NrfStatus::new(led),
            environment,
            battery: NrfBattery::<Battery>::new(saadc_sensor),
            ota: NoOta,
            actions: nrf52840_sensor_product::policy::USER_ACTIONS,
            supervisor: NrfSupervisor,
            diagnostics: NrfDiagnostics,
        },
    )
    .expect("nRF52840 sensor composition must disable automatic polling");

    app.run().await
}
