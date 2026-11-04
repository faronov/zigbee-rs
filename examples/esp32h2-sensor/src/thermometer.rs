//! # ESP32-H2 thermometer (sleepy end device)
//!
//! Battery thermometer for boards without a status LED (e.g. the ESP32-H2
//! SuperMini): one Home Automation Temperature Sensor endpoint with Basic,
//! Identify, Temperature Measurement (eFuse-calibrated die temperature) and
//! the OTA Upgrade client. Reporting defaults, poll and sample cadence,
//! identity and the USB policy come from `thermometer.toml`; Configure
//! Reporting from the coordinator overrides the reporting defaults.
//!
//! BOOT (GPIO9): short press = report now + fast-poll window (or join when
//! unjoined), 3 s hold = factory reset.
//!
//! ```bash
//! cargo build --release --bin esp32h2-thermometer --features thermometer,light-sleep,quiet
//! ```
//! `quiet` compiles out every log statement and the serial console.

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod chip_temperature;
mod thermometer_platform;
mod thermometer_profile;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use esp32_zigbee_devkit_product as product;
use esp32_zigbee_devkit_product::executor::block_on;
use esp32_zigbee_devkit_product::wake::SensorWake;
use esp_hal::gpio::{Input, InputConfig, Pull};
use sensor_sed_app::{ForceReportAction, NoBattery, NoStatus, SensorApp, SensorSedParts};
use static_cell::StaticCell;
use zigbee_mac::esp::{EspRadioPhy, EspSoftMac};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

use chip_temperature::H2TemperatureSensor;
use thermometer_platform::{ThermoDiagnostics, ThermoSupervisor};
use thermometer_profile::{config, DieThermometer, ThermoProfile, POLICY};

#[cfg(not(feature = "quiet"))]
use esp_backtrace as _;

type Parts = SensorSedParts<
    SensorWake<'static>,
    NoStatus,
    DieThermometer,
    NoBattery,
    product::ota_transport::OtaTransport,
    ForceReportAction,
    ThermoSupervisor,
    ThermoDiagnostics,
>;
type App =
    SensorApp<'static, EspSoftMac<'static>, product::storage::SecurityStore, ThermoProfile, Parts>;

/// Quiet builds carry no panic formatter or console: reset and let the
/// security journal resume the network.
#[cfg(feature = "quiet")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    esp_hal::system::software_reset()
}

/// Unrecoverable startup failure. Spin rather than reset so a bad partition
/// table or persistence error cannot become a reset loop that rewrites NV.
fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[inline(never)]
fn build_mac(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> EspSoftMac<'static> {
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(radio);
    let mut phy = EspRadioPhy::new(ieee802154, esp_radio::ieee802154::Config::default());
    if phy.install_aes_engine(aes).is_err() {
        halt()
    }
    let Ok(mac) = phy.into_soft_mac() else { halt() };
    mac
}

/// Build MAC, device, store and profile in static storage; not inlined so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> (
    &'static mut ZigbeeDevice<EspSoftMac<'static>>,
    &'static mut product::storage::SecurityStore,
    &'static mut ThermoProfile,
) {
    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<ThermoProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<EspSoftMac<'static>>> = StaticCell::new();

    // A legacy-migration or partition-table error may leave NV intact: stop
    // rather than commission as factory-new.
    let Ok((store, _migration)) = product::storage::open_security_store() else {
        halt()
    };
    let Ok(profile) =
        thermometer_profile::thermo_profile(FIRMWARE_VERSION, esp_hal::system::software_reset)
    else {
        halt()
    };
    let security = SECURITY.init(store);
    let profile = PROFILE.init(profile);

    let mac = build_mac(radio, aes);
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        halt()
    }
    let device = ZigbeeDevice::builder(mac)
        .power_mode(POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(config::MODEL)
        .application_version(product::firmware::application_version(FIRMWARE_VERSION))
        .date_code(product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build_into(DEVICE.uninit());

    // Keys and frame counters are never resumed under another EUI-64 or by
    // another role image; a switch clears the network state (keeping the
    // counter reservations) and forces a fresh join.
    if device
        .reset_security_state_if_identity_changed(security, ieee)
        .is_err()
        || security.select_role_tag(config::ROLE_TAG).is_err()
    {
        halt()
    }
    (device, security, profile)
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // The radio's receive queue allocates from this heap.
    esp_alloc::heap_allocator!(size: 32768);
    #[cfg(not(feature = "quiet"))]
    thermometer_platform::console::init();
    product::time_driver::init(peripherals.SYSTIMER);

    // Quiet builds also suppress the ROM bootloader banner on later resets.
    let rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);
    #[cfg(feature = "quiet")]
    rtc.disable_rom_message_printing();
    #[cfg(feature = "light-sleep")]
    let sleep = Some(
        product::sleep::LightSleep::new(
            rtc,
            peripherals.LP_TIMER,
            peripherals.PMU,
            peripherals.LP_AON,
        )
        .with_power_down(product::sleep::RetainedPowerDown {
            xtal: config::SLEEP_XTAL_OFF,
            rc_fast: config::SLEEP_RC_FAST_OFF,
        }),
    );
    #[cfg(not(feature = "light-sleep"))]
    let sleep = {
        let _rtc = rtc;
        None
    };

    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );
    let Ok(sensor) = H2TemperatureSensor::new() else {
        halt()
    };

    // An unconfirmed OTA image rolls back before persistence is touched.
    if let Ok(product::ota::BootCheck::RolledBack { .. }) = product::ota_boot::begin() {
        esp_hal::system::software_reset();
    }

    static APP: StaticCell<App> = StaticCell::new();
    let (device, security, profile) = compose(peripherals.IEEE802154, peripherals.AES);
    let parts = SensorSedParts {
        wake: SensorWake::new(button, sleep),
        status: NoStatus,
        environment: DieThermometer::new(sensor),
        battery: NoBattery,
        ota: product::ota_transport::OtaTransport::new(),
        actions: ForceReportAction,
        supervisor: ThermoSupervisor::new(),
        diagnostics: ThermoDiagnostics,
    };
    let node = ZigbeeNode::new(device, security, profile);
    let Ok(app) = SensorApp::new(node, &POLICY, parts) else {
        halt()
    };
    block_on(run(APP.init(app)))
}

/// `initialize` → restore the persisted reporting configuration → `step`
/// forever, persisting a changed configuration after each iteration.
async fn run(app: &'static mut App) -> ! {
    if app.initialize().await.is_err() {
        halt()
    }
    #[cfg(feature = "reporting-persist")]
    let mut reporting = Some(product::reporting::reporting_persistence());
    #[cfg(feature = "reporting-persist")]
    if let Some(persistence) = reporting.as_mut() {
        // A failed restore keeps the product defaults; the next sync
        // replaces the unreadable snapshot.
        let _ = persistence.restore(app.device_mut());
    }
    loop {
        let _ = app.step().await;
        #[cfg(feature = "reporting-persist")]
        if let Some(persistence) = reporting.as_mut() {
            if persistence.sync(app.device()).is_err() {
                // Flash refused the commit: stop for this boot instead of
                // re-erasing the window on every wake.
                reporting = None;
            }
        }
    }
}
