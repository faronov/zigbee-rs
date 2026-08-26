//! BRD4181A EFR32MG21 environmental sleepy-end-device composition root.
//!
//! The reusable lifecycle is `sensor-sed-app`; this file owns only startup,
//! exclusive board resources, identity validation, and static capability
//! assembly. Temperature, humidity, and battery readings are explicitly
//! synthetic/fixed until real sensors are fitted and qualified.

#![no_std]
#![no_main]

mod platform;
#[cfg(feature = "stubs")]
mod stubs;
mod time_driver;
mod vectors;

use efr32mg21_devkit::resources::BoardResources;
use efr32mg21_sensor_product as product;
use embassy_executor::Spawner;
use sensor_sed_app::{FixedBattery, NoOta, SensorApp, SensorSedParts};
use static_cell::StaticCell;
use zigbee_mac::{
    MacDriver,
    efr32s2::Efr32s2Mac,
    pib::{PibAttribute, PibValue},
};
use zigbee_runtime::{
    ZigbeeDevice,
    node::ZigbeeNode,
    profile::{ApplicationProfile, BatteryMeasurement},
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use vectors::__INTERRUPTS;

type AppParts = SensorSedParts<
    platform::Brd4181aWake,
    platform::Pb0Status,
    platform::SyntheticEnvironment,
    FixedBattery,
    NoOta,
    sensor_sed_app::ToggleJoinAction,
    platform::Efr32Supervisor,
    platform::RttDiagnostics,
>;
type SharedSensorApp = SensorApp<
    'static,
    Efr32s2Mac,
    product::storage::SecurityStore,
    product::profile::SensorProfile,
    AppParts,
>;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    platform::halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let BoardResources {
        clocks,
        led0,
        button0,
        flash,
    } = BoardResources::take().unwrap_or_else(|| platform::halt());

    efr32mg21_devkit::init_clocks(clocks).unwrap_or_else(|_| platform::halt());
    time_driver::init();
    rtt_target::rtt_init_log!(log::LevelFilter::Info);

    let mut led = led0.into_led();
    let button = button0.into_button();
    platform::enable_button_interrupt();
    platform::signal_boot(&mut led).await;

    log::info!(
        "[EFR32] {} on {} / {} ({})",
        product::MODEL,
        efr32mg21_devkit::BOARD_RADIO,
        efr32mg21_devkit::BOARD_MAIN,
        efr32mg21_devkit::MCU_PART
    );
    log::warn!("[EFR32] SYNTHETIC temperature/humidity and FIXED 3000 mV battery");
    log::info!("[EFR32] idle mode is radio-off WFE with 1 kHz SysTick; not EM2");

    // The radio FRC interrupt is independent from the PD2 GPIO_EVEN input.
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);
    // SAFETY: Efr32s2Mac is the sole FRC peripheral owner.
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::FrcPri) };

    let mac = Efr32s2Mac::new();
    let ieee = match mac.mlme_get(PibAttribute::MacExtendedAddress).await {
        Ok(PibValue::ExtendedAddress(address)) if address != [0; 8] && address != [0xFF; 8] => {
            address
        }
        _ => {
            log::error!("[EFR32] no valid IEEE EUI-64");
            platform::halt()
        }
    };

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<product::profile::SensorProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Efr32s2Mac>> = StaticCell::new();
    static APP: StaticCell<SharedSensorApp> = StaticCell::new();

    let profile = PROFILE.init(product::profile::sensor_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .application_version(product::APPLICATION_VERSION)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let (security_store, migration) =
        product::storage::security_store(flash).unwrap_or_else(|error| {
            log::error!("[EFR32] security journal open/migration failed: {error:?}");
            platform::halt()
        });
    let security_store = SECURITY.init(security_store);
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[EFR32] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[EFR32] blank/legacy persistence reset; fresh join required")
        }
    }

    // This identity guard is deliberately before ZigbeeNode construction:
    // persisted keys/counters may never be resumed under a different EUI-64.
    match device.reset_security_state_if_identity_changed(security_store, ieee) {
        Ok(true) => log::warn!("[EFR32] cleared journal after IEEE identity change"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[EFR32] identity guard failed: {error:?}");
            platform::halt()
        }
    }

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: platform::Brd4181aWake::new(button),
            status: platform::Pb0Status::new(led),
            environment: platform::SyntheticEnvironment::new(),
            battery: FixedBattery::new(
                3_000,
                BatteryMeasurement {
                    voltage_100mv: 30,
                    percentage_remaining: 200,
                },
            ),
            ota: NoOta,
            actions: product::policy::USER_ACTIONS,
            supervisor: platform::Efr32Supervisor,
            diagnostics: platform::RttDiagnostics,
        },
    )
    .unwrap_or_else(|error| {
        log::error!("[EFR32] invalid SensorApp composition: {error:?}");
        platform::halt()
    });

    APP.init(app).run().await
}

#[unsafe(no_mangle)]
extern "C" fn GPIO_EVEN() {
    platform::gpio_even_irq();
}
