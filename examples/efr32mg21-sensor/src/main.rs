//! BRD4181A EFR32MG21 sleepy environmental sensor composition root.
//!
//! The reusable lifecycle is `sensor-sed-app`; this file owns only startup,
//! exclusive board resources, identity validation and static capability
//! assembly. The radio is the pure-Rust Series 2 driver (`efr32s2-hal`) on
//! the shared software MAC (`Efr32s2RadioPhy` + `SoftMacCore`).
//!
//! Measurements: temperature is the EFR32 die sensor (EMU_TEMP), humidity is
//! synthetic and the battery is a fixed 3000 mV.
//!
//! Radio status: the xG21 analog/PHY tables have not been captured yet, so
//! `Efr32s2RadioPhy::new` returns `PhyError::Unsupported` on this chip and
//! the firmware halts with a logged error rather than programming xG22
//! values into an xG21 radio. Everything up to radio init (clocks, VCOM,
//! RTCC, persistence) is shared with the hardware-proven MG22 path.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;
mod vcom_log;

use efr32mg21_devkit::resources::BoardResources;
use efr32mg21_sensor_product as product;
use efr32s2_hal::power;
use embassy_executor::Spawner;
use sensor_sed_app::{FixedBattery, NoOta, SensorApp, SensorSedParts};
use static_cell::StaticCell;
use zigbee_mac::efr32s2::{Efr32s2RadioPhy, Efr32s2SoftMac};
use zigbee_runtime::{
    ZigbeeDevice,
    node::ZigbeeNode,
    profile::{ApplicationProfile, BatteryMeasurement},
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type AppParts = SensorSedParts<
    platform::Mg21Wake,
    platform::Led0Status,
    platform::DieTemperature,
    FixedBattery,
    NoOta,
    sensor_sed_app::ToggleJoinAction,
    platform::Efr32Supervisor,
    platform::VcomDiagnostics,
>;
type SharedSensorApp = SensorApp<
    'static,
    Efr32s2SoftMac,
    product::storage::SecurityStore,
    product::profile::SensorProfile,
    AppParts,
>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let BoardResources {
        clocks,
        led0,
        button0,
        vcom,
        radio,
        rtcc,
        flash,
    } = BoardResources::take().unwrap_or_else(|| platform::halt());

    let clocks = efr32mg21_devkit::init_clocks(clocks);
    vcom_log::init(vcom.into_vcom());
    if let Err(error) = clocks {
        log::error!("[MG21] HFXO start failed: {error:?}");
        platform::halt()
    }
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[MG21] RTCC start failed: {error:?}");
        platform::halt()
    });
    efr32s2_hal::time_driver::init(rtcc);
    // Representative EM2 current: do not keep the debug domain powered.
    // Re-enable (or attach before reset) for SWD access while asleep.
    power::set_em2_debug(cfg!(feature = "em2-debug"));

    let mut led = led0.into_led();
    let button = button0.into_button();
    platform::enable_button_interrupt();
    platform::signal_boot(&mut led).await;

    log::info!(
        "[MG21] {} on {} / {} ({})",
        product::MODEL,
        efr32mg21_devkit::BOARD_RADIO,
        efr32mg21_devkit::BOARD_MAIN,
        efr32mg21_devkit::MCU_PART
    );
    log::warn!("[MG21] die temperature, SYNTHETIC humidity, FIXED 3000 mV battery");

    // Composition runs in its own (non-inlined) frame so the large
    // temporaries of building the MAC, device and app are released before
    // the run loop (same structure as the 32 KiB MG22 image).
    compose(radio, flash, button, led).run().await
}

#[inline(never)]
fn compose(
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
    button: efr32mg21_devkit::Button0,
    led: efr32mg21_devkit::Led0,
) -> &'static mut SharedSensorApp {
    let mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[MG21] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_soft_mac()
                .map_err(|error| log::error!("[MG21] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| platform::halt());
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[MG21] no valid IEEE EUI-64");
        platform::halt()
    }
    log::info!("[MG21] EUI-64 {:02X?}", ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<product::profile::SensorProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Efr32s2SoftMac>> = StaticCell::new();
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
            log::error!("[MG21] security journal open failed: {error:?}");
            platform::halt()
        });
    let security_store = SECURITY.init(security_store);
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[MG21] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[MG21] blank/corrupt persistence reset; fresh join required")
        }
    }

    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64.
    match device.reset_security_state_if_identity_changed(security_store, ieee) {
        Ok(true) => log::warn!("[MG21] cleared journal after IEEE identity change"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[MG21] identity guard failed: {error:?}");
            platform::halt()
        }
    }

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: platform::Mg21Wake::new(button),
            status: platform::Led0Status::new(led),
            environment: platform::DieTemperature::new(),
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
            diagnostics: platform::VcomDiagnostics,
        },
    )
    .unwrap_or_else(|error| {
        log::error!("[MG21] invalid SensorApp composition: {error:?}");
        platform::halt()
    });

    APP.init(app)
}

#[unsafe(no_mangle)]
extern "C" fn GPIO_EVEN() {
    platform::gpio_even_irq();
}
