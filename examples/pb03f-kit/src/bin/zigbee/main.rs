//! PB-03F-Kit Zigbee end-device composition root.
//!
//! Joins a Zigbee 3.0 network with the pure-Rust PHY6252 radio and reports
//! the product's synthetic temperature/humidity series. Diagnostic logging
//! goes to the CH340 UART console (P9/P10, 115200 8N1).

#![no_std]
#![no_main]

mod platform;

use cortex_m as _;
use embassy_executor::Spawner;
use panic_halt as _;
use pb03f_kit::{Resources, console_log, time, vectors};
use pb03f_kit_product as product;
use sensor_sed_app::{BlockingBattery, BlockingEnvironment, NoOta, SensorApp, SensorSedParts};
use zigbee_mac::phy6222::Phy6222RadioPhy;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;

use crate::platform::{PhyDiagnostics, PhyStatus, PhySupervisor, PhyWakeController};

fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

fn fault(status: &mut PhyStatus, reason: &str) -> ! {
    log::error!("fault: {}", reason);
    status.fault();
    halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    time::init();

    let Ok(resources) = Resources::take() else {
        halt()
    };
    let Resources {
        flash,
        mut leds,
        user_button,
        console,
        supply_monitor,
    } = resources;
    console_log::init(console.into_uart(), log::LevelFilter::Info);
    log::info!(
        "{} {} {} (HCLK {} Hz)",
        product::MANUFACTURER,
        product::MODEL,
        product::SW_BUILD,
        phy6222_hal::clock::hclk_hz()
    );
    leds.all_off();
    let mut status = PhyStatus::new(leds.green);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::LlIrq);
    }
    let Some(radio) = Phy6222RadioPhy::take_with_config(product::radio::radio_config()) else {
        fault(&mut status, "radio already taken")
    };
    let Ok(ieee) = product::identity::validate_ieee_address(radio.extended_address()) else {
        fault(&mut status, "factory IEEE address missing")
    };
    let Ok(mac) = radio.into_soft_mac() else {
        fault(&mut status, "radio configuration rejected")
    };

    let battery = product::battery::SupplyBattery::new(supply_monitor);

    let mut profile = product::profile::sensor_profile();
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(product::POWER_SOURCE)
        .channels(product::CHANNELS)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();

    let mut security_store = product::storage::security_store(flash);
    if device
        .reset_security_state_if_identity_changed(&mut security_store, ieee)
        .is_err()
    {
        fault(&mut status, "security journal");
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
    let mut app = match SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: PhyWakeController::new(user_button),
            status,
            environment: BlockingEnvironment::new(product::environment::SyntheticEnvironment::new()),
            battery: BlockingBattery::new(battery),
            ota: NoOta,
            actions: product::policy::USER_ACTIONS,
            supervisor: PhySupervisor,
            diagnostics: PhyDiagnostics,
        },
    ) {
        Ok(app) => app,
        Err(_) => halt(),
    };

    app.run().await
}
