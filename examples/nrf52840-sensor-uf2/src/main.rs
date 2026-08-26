//! nRF52840 sleepy environmental sensor for proven UF2 deployments.
//!
//! This file is a composition root only. The shared `sensor-sed` application
//! owns commissioning, manual parent polling, reporting, durable reset, and
//! rejoin lifecycle. Product code owns the `DeviceProfile`, policy, deployment
//! map, and security journal. Board crates own fitted pins and polarity.

#![no_std]
#![no_main]

#[cfg(not(any(
    feature = "board-promicro",
    feature = "board-mdk",
    feature = "board-nrf-dongle",
    feature = "board-nrf-dk"
)))]
compile_error!("select exactly one nRF52840 UF2 board feature");

#[cfg(any(
    all(feature = "board-promicro", feature = "board-mdk"),
    all(feature = "board-promicro", feature = "board-nrf-dongle"),
    all(feature = "board-promicro", feature = "board-nrf-dk"),
    all(feature = "board-mdk", feature = "board-nrf-dongle"),
    all(feature = "board-mdk", feature = "board-nrf-dk"),
    all(feature = "board-nrf-dongle", feature = "board-nrf-dk")
))]
compile_error!("select exactly one nRF52840 UF2 board feature");

#[cfg(feature = "board-promicro")]
const _: () = core::assert!(!nrf52840_promicro::HAS_USER_BUTTON);
#[cfg(feature = "board-mdk")]
const _: () = core::assert!(!nrf52840_mdk_usb_dongle::HAS_USER_BUTTON);
#[cfg(feature = "board-nrf-dongle")]
const _: () =
    core::assert!(nrf52840_pca10059::HAS_USER_BUTTON && nrf52840_pca10059::BUTTON_ACTIVE_LOW);
#[cfg(feature = "board-nrf-dk")]
const _: () = core::assert!(nrf52840_dk::BUTTON_ACTIVE_LOW);

use embassy_executor::Spawner;
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, peripherals, radio, rng};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[cfg(any(feature = "board-promicro", feature = "board-mdk"))]
use nrf_sensor_app::NrfTimerWakeController;
#[cfg(any(feature = "board-nrf-dongle", feature = "board-nrf-dk"))]
use nrf_sensor_app::NrfWakeController;
use nrf_sensor_app::{
    BatteryPolicy, NrfBattery, NrfDiagnostics, NrfPolarityStatus, NrfSupervisor, OnChipTemperature,
};
#[cfg(any(feature = "board-promicro", feature = "board-mdk"))]
use sensor_sed_app::NoUserAction;
use sensor_sed_app::{NoOta, SensorApp, SensorSedParts};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{ApplicationProfile, BatteryMeasurement};
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

struct Battery;

impl BatteryPolicy for Battery {
    fn millivolts(raw_sample: i16) -> u32 {
        nrf52840_sensor_product::battery::millivolts(raw_sample)
    }

    fn measurement(raw_sample: i16) -> BatteryMeasurement {
        nrf52840_sensor_product::battery::battery_measurement(raw_sample)
    }
}

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

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

// POWER state survives reset. Restore every RAM bank before cortex-m-rt
// initializes memory, including when replacing firmware that powered banks
// down. Pure assembly avoids touching a potentially unpowered stack.
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904",
    "mvn r1, #0",
    "str r1, [r0, #0x00]",
    "str r1, [r0, #0x10]",
    "str r1, [r0, #0x20]",
    "str r1, [r0, #0x30]",
    "str r1, [r0, #0x40]",
    "str r1, [r0, #0x50]",
    "str r1, [r0, #0x60]",
    "str r1, [r0, #0x70]",
    "str r1, [r0, #0x80]",
    "bx lr",
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "board-promicro")]
    let softdevice_disable_status = unsafe { nrf52840_promicro::disable_softdevice() };
    #[cfg(feature = "board-promicro")]
    if softdevice_disable_status != 0 {
        error!(
            "S140 disable failed with 0x{:08X}; refusing direct peripheral ownership",
            softdevice_disable_status
        );
        loop {
            cortex_m::asm::wfi();
        }
    }
    #[cfg(feature = "board-promicro")]
    info!("S140 disabled before Nordic peripheral initialization");

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.dcdc = embassy_nrf::config::DcdcConfig {
        reg0: true,
        reg0_voltage: None,
        reg1: true,
    };
    let p = embassy_nrf::init(config);

    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    info!(
        "nRF52840 UF2 sensor: {}",
        nrf52840_sensor_product::deployment::SELECTED.name
    );

    #[cfg(feature = "board-promicro")]
    let status = NrfPolarityStatus::<{ nrf52840_promicro::STATUS_LED_ACTIVE_LOW }>::new(
        nrf52840_promicro::status_led(p.P0_15),
    );
    #[cfg(feature = "board-mdk")]
    let status = NrfPolarityStatus::<{ nrf52840_mdk_usb_dongle::STATUS_LED_ACTIVE_LOW }>::new(
        nrf52840_mdk_usb_dongle::status_led(p.P0_22),
    );
    #[cfg(feature = "board-nrf-dongle")]
    let status = NrfPolarityStatus::<{ nrf52840_pca10059::STATUS_LED_ACTIVE_LOW }>::new(
        nrf52840_pca10059::status_led(p.P0_06),
    );
    #[cfg(feature = "board-nrf-dk")]
    let status =
        NrfPolarityStatus::<{ nrf52840_dk::LED_ACTIVE_LOW }>::new(nrf52840_dk::status_led(p.P0_13));

    #[cfg(any(feature = "board-promicro", feature = "board-mdk"))]
    let wake = NrfTimerWakeController;
    #[cfg(feature = "board-nrf-dongle")]
    let wake = NrfWakeController::new(nrf52840_pca10059::button(p.P1_06));
    #[cfg(feature = "board-nrf-dk")]
    let wake = NrfWakeController::new(nrf52840_dk::button(p.P0_11));

    #[cfg(any(feature = "board-promicro", feature = "board-mdk"))]
    let actions = NoUserAction;
    #[cfg(any(feature = "board-nrf-dongle", feature = "board-nrf-dk"))]
    let actions = nrf52840_sensor_product::policy::USER_ACTIONS;

    let environment = OnChipTemperature::new(Temp::new(p.TEMP, Irqs));
    let battery = Saadc::new(
        p.SAADC,
        Irqs,
        saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    battery.calibrate().await;

    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let rng = rng::Rng::new(p.RNG, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio, rng);
    let Some(aes) = zigbee_mac::nrf::NrfEcbToken::take() else {
        error!("Nordic ECB AES already owned; networking halted");
        loop {
            cortex_m::asm::wfi();
        }
    };
    if let Err(error) = mac.install_aes_engine(aes) {
        error!(
            "Nordic ECB dual-KAT failed; networking halted: {:?}",
            defmt::Debug2Format(&error)
        );
        loop {
            cortex_m::asm::wfi();
        }
    }
    let ieee = mac.extended_address();
    info!("Nordic ECB hardware AES dual-KAT passed");
    mac.set_tx_power(0);

    let mut profile = nrf52840_sensor_product::profile::sensor_profile();
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(nrf52840_sensor_product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(nrf52840_sensor_product::MANUFACTURER)
        .model(nrf52840_sensor_product::deployment::UF2_MODEL)
        .date_code(nrf52840_sensor_product::DATE_CODE)
        .sw_build(nrf52840_sensor_product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();

    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut security_store = nrf52840_sensor_product::uf2_storage::security_store(nvmc);
    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => warn!("Cleared persisted network state after FICR EUI-64 change"),
        Ok(false) => {}
        Err(error) => nrf_sensor_app::persistence_failure(error),
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
    let mut app = SensorApp::new(
        node,
        &nrf52840_sensor_product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake,
            status,
            environment,
            battery: NrfBattery::<Battery>::new(battery),
            ota: NoOta,
            actions,
            supervisor: NrfSupervisor,
            diagnostics: NrfDiagnostics,
        },
    )
    .expect("UF2 sensor composition must have one manual poll owner");

    app.run().await
}
