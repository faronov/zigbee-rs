//! Production EFR32MG1P TRÅDFRI Zigbee temperature/humidity SED.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod diagnostics;
mod platform;
mod sensor;
mod time_driver;
mod vectors;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use cortex_m as _;
use efr32mg1_hal::peripherals::Peripherals;
use efr32mg1_tradfri::resources::BoardResources;
use embassy_executor::Spawner;
use sensor_sed_app::{SensorApp, SensorSedParts};
use static_cell::StaticCell;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;
use zigbee_bdb::attributes::{BDB_POPULAR_CHANNEL_FALLBACK_SET, BDB_POPULAR_CHANNEL_SET};
use zigbee_mac::efr32::{Efr32Mac, Efr32RadioPhy};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type AppParts = SensorSedParts<
    platform::Efr32WakeController,
    platform::Efr32Status,
    sensor::Sensor,
    efr32mg1_tradfri_product::battery::BatteryAdapter,
    efr32mg1_tradfri_product::ota::Efr32OtaLifecycle,
    sensor_sed_app::ForceReportAction,
    platform::Efr32Supervisor,
    diagnostics::RttDiagnostics,
>;
type SharedSensorApp = SensorApp<
    'static,
    Efr32Mac,
    efr32mg1_tradfri_product::storage::SecurityStore,
    efr32mg1_tradfri_product::profile::SensorProfile,
    AppParts,
>;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    platform::halt_with_led_raw()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ── Board resource ownership ─────────────────────────────────────────
    // Take the singleton resource set. Each token is consumed exactly once,
    // enforcing mutual exclusion (PA0 → LED not PWM; flash → bootloader not SPI).
    let board = BoardResources::take().unwrap_or_else(|| platform::halt_with_led_raw());
    let chip = Peripherals::take().unwrap_or_else(|| platform::halt_with_led_raw());

    // Platform startup owns only physical startup resources.
    platform::init(board.pa0, board.button);
    platform::init_stack_watermark();
    platform::signal_boot().await;

    // Product policy selects the resident Gecko Bootloader OTA path rather
    // than direct USART0 SPI access to the same external flash.
    let ota_flash = board.external_flash.into_bootloader_managed();

    // ── Sensor I2C and supply ADC ────────────────────────────────────────
    let i2c = board
        .sensor_i2c
        .into_sensor_i2c()
        .unwrap_or_else(|_| platform::halt_with_led());
    let sht = sensor::Sensor::new(i2c);

    let battery = board.supply_adc.into_supply_monitor().ok().map(|supply| {
        efr32mg1_tradfri_product::battery::BatteryMonitor::from_supply(
            supply,
            efr32mg1_tradfri_product::battery::DEFAULT_BATTERY_CURVE,
        )
    });

    // ── Zigbee stack and product profile ─────────────────────────────────
    static SECURITY: StaticCell<efr32mg1_tradfri_product::storage::SecurityStore> =
        StaticCell::new();
    static PROFILE: StaticCell<efr32mg1_tradfri_product::profile::SensorProfile> =
        StaticCell::new();
    static APP: StaticCell<SharedSensorApp> = StaticCell::new();

    // Scoped so that no construction temporary is kept in this task's
    // future across `run().await`.
    let app = {
        let profile = PROFILE.init(
            efr32mg1_tradfri_product::profile::sensor_profile(FIRMWARE_VERSION, ota_flash)
                .unwrap_or_else(|_| platform::halt_with_led()),
        );
        let device = build_device(chip.crypto, profile);
        let ieee = device.mac().pib().extended_address();

        let (security_partition, _application_nv) =
            efr32mg1_tradfri_product::storage::split_flash(chip.flash);
        let security = SECURITY.init(efr32mg1_tradfri_product::storage::security_store(
            security_partition,
        ));
        match device.reset_security_state_if_identity_changed(security, ieee) {
            Ok(true) => {
                rtt_target::rprintln!("[EFR32][sensor] CLEARED_STALE_IDENTITY");
            }
            Ok(false) => {}
            Err(_) => platform::halt_with_led(),
        }

        let node = ZigbeeNode::new(device, security, profile);

        let app = SensorApp::new(
            node,
            &efr32mg1_tradfri_product::policy::SENSOR_POLICY,
            SensorSedParts {
                wake: platform::Efr32WakeController::new(),
                status: platform::Efr32Status::new(),
                environment: sht,
                battery: efr32mg1_tradfri_product::battery::BatteryAdapter::new(battery),
                ota: efr32mg1_tradfri_product::ota::Efr32OtaLifecycle::new(),
                actions: efr32mg1_tradfri_product::policy::USER_ACTIONS,
                supervisor: platform::Efr32Supervisor,
                diagnostics: diagnostics::RttDiagnostics,
            },
        )
        .unwrap_or_else(|_| platform::halt_with_led());
        APP.init(app)
    };
    app.run().await
}

/// Bring up the radio and AES engine and build the `ZigbeeDevice` in place.
///
/// The PHY, the `SoftMacCore` value, its `Result` and the builder are
/// temporaries; in the async `main` they would occupy the task's future and
/// poll frame for the whole run. Here they live in a frame that is popped
/// before `run()`.
#[inline(never)]
fn build_device(
    crypto: efr32mg1_hal::peripherals::Crypto,
    profile: &mut efr32mg1_tradfri_product::profile::SensorProfile,
) -> &'static mut ZigbeeDevice<Efr32Mac> {
    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();

    let mut phy = Efr32RadioPhy::new();
    phy.install_aes_engine(crypto)
        .unwrap_or_else(|_| platform::halt_with_led());
    let mac = phy
        .into_soft_mac()
        .unwrap_or_else(|_| platform::halt_with_led());
    rtt_target::rprintln!(
        "[EFR32][sensor] AES_KAT_PASS ieee={:02X?}",
        mac.pib().extended_address()
    );

    let device = ZigbeeDevice::builder(mac)
        .power_mode(efr32mg1_tradfri_product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(efr32mg1_tradfri_product::MANUFACTURER)
        .model(efr32mg1_tradfri_product::MODEL)
        .application_version(FIRMWARE_APPLICATION_VERSION)
        .date_code(efr32mg1_tradfri_product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build_into(DEVICE.uninit());
    device.bdb_mut().attributes_mut().primary_channel_set = BDB_POPULAR_CHANNEL_SET;
    device.bdb_mut().attributes_mut().secondary_channel_set = BDB_POPULAR_CHANNEL_FALLBACK_SET;
    device
}
