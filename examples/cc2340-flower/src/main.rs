//! Composition root for koptserg's CC2340R5 "flower" soil-moisture sensor.
//!
//! - The board crate owns the pin map and the fitted peripherals.
//! - The product crate owns identity, the flash layout, sensor detection,
//!   the endpoint and the measurement policy.
//! - [`SensorApp`] owns commissioning, polling, reporting, reset/rejoin and
//!   durable security checkpoints.
//!
//! This file only wires them together. The endpoint's cluster set is
//! resolved *before* the device is built (the Simple Descriptor is fixed
//! from then on): a commissioned boot reuses the persisted detection, while
//! a factory-new boot probes the I2C bus. After the device leaves its
//! network the firmware reboots, so the next boot is factory-new and probes
//! again.
//!
//! Not validated on hardware: see the README.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;

use core::cell::{Cell, RefCell};
use core::mem::MaybeUninit;

use cortex_m as _;
use panic_halt as _;

use cc2340_flower_product as product;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use flower_cc2340r5 as board;
use sensor_sed_app::{DiagnosticEvent, Diagnostics, NoOta, SensorApp, SensorSedParts};
use zigbee_mac::cc2340::{Cc2340Mac, RadioError};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_zcl::clusters::basic::PowerSource;

/// Pause between the leave indication and the reboot (as the reference).
const REBOOT_AFTER_LEAVE_MS: u64 = 2_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let left_network = Cell::new(false);
    let mut diagnostics = platform::init_diagnostics(&left_network);
    let Some(resources) = board::take() else {
        platform::halt();
    };
    let board::Resources {
        green_led,
        red_led,
        button,
        mut i2c,
        soil,
        mut flash,
        batmon,
        reset,
    } = resources;
    let leds = RefCell::new(platform::Indicators::new(green_led, red_led));
    log::info!(
        "[FLOWER] starting {} / {} ({} calibration)",
        board::BOARD_NAME,
        board::TARGET_PART,
        product::calibration::ACTIVE.name
    );

    let ieee = match board::identity::factory_ieee_address() {
        Ok(address) => address,
        Err(error) => {
            log::error!("[FLOWER] factory identity unavailable: {error}");
            fatal(&leds);
        }
    };
    log::info!("[FLOWER] factory EUI-64: {:02X?}", ieee);

    // The endpoint is fixed from here on.
    let (sensors, origin) = product::boot::resolve_sensors(&mut flash, ieee, &mut i2c);
    log::info!("[FLOWER] sensors {sensors:?} ({origin:?})");

    let mut mac = match Cc2340Mac::new(ieee) {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("[FLOWER] rejected factory identity: {error:?}");
            fatal(&leds);
        }
    };
    if let Err(error) = mac.initialize_radio() {
        match error {
            RadioError::FirmwareUnavailable => {
                log::error!("[FLOWER] radio firmware unavailable (fallback build)");
            }
            RadioError::RadioConfigUnavailable => {
                log::error!("[FLOWER] PHY/board radio configuration unavailable (partial SDK)");
            }
            _ => log::error!("[FLOWER] radio initialization failed: {error:?}"),
        }
        fatal(&leds);
    }

    // Build the device in static storage: by-value `build()` needs a frame
    // larger than this part's whole stack.
    static mut DEVICE_STORAGE: MaybeUninit<ZigbeeDevice<Cc2340Mac>> = MaybeUninit::uninit();
    let mut profile = product::profile::flower_profile(sensors);
    let device = ZigbeeDevice::builder(mac)
        .power_mode(product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        // SAFETY: the embassy main task runs once, so this is the only
        // reference ever taken to DEVICE_STORAGE.
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    let mut security_store = product::storage::security_store(flash);
    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => log::warn!("[FLOWER] cleared persisted state for a different EUI-64"),
        Ok(false) => {}
        Err(error) => {
            diagnostics.record(DiagnosticEvent::SecurityFailure(error));
            fatal(&leds);
        }
    }

    let measurement = product::measurement::FlowerMeasurement::new(
        i2c,
        soil,
        batmon,
        embassy_time::Delay,
        sensors,
        product::calibration::ACTIVE,
        product::measurement::CADENCE,
    );
    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let mut app = SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: platform::FlowerWake::new(button, &leds),
            status: platform::StatusLeds::new(&leds),
            environment: measurement,
            battery: product::battery::SampledWithSensors,
            ota: NoOta,
            actions: product::policy::USER_ACTIONS,
            supervisor: platform::ResetOnlySupervisor::new(&reset),
            diagnostics,
        },
    )
    .expect("flower SensorApp requires manual runtime polling and a valid policy");

    if app.initialize().await.is_err() {
        platform::halt();
    }
    loop {
        if app.step().await.is_err() {
            platform::halt();
        }
        if left_network.get() {
            // The step that observed the leave has already persisted the
            // factory-new security state.
            platform::play(&leds, platform::Pattern::Left).await;
            Timer::after(Duration::from_millis(REBOOT_AFTER_LEAVE_MS)).await;
            reset.reset();
        }
    }
}

fn fatal(leds: &RefCell<platform::Indicators>) -> ! {
    leds.borrow_mut().fault();
    platform::halt()
}
