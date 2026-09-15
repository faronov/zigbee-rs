//! PHY62x2 environmental sleepy-end-device composition root.

#![no_std]
#![no_main]

mod platform;

use cortex_m as _;
use embassy_executor::Spawner;
use panic_halt as _;
use phy62x2_evk::Resources;
use phy62x2_evk_product as product;
use sensor_sed_app::{BlockingBattery, BlockingEnvironment, NoOta, SensorApp, SensorSedParts};
use zigbee_mac::phy6222::Phy6222Mac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;

use crate::platform::{PhyDiagnostics, PhyStatus, PhySupervisor, PhyWakeController};

fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

fn fault(status: &mut PhyStatus) -> ! {
    status.fault();
    halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    phy62x2_evk::time::init();
    unsafe {
        cortex_m::peripheral::NVIC::unmask(phy62x2_evk::vectors::Interrupt::LlIrq);
    }

    let resources = match Resources::take() {
        Ok(resources) => resources,
        Err(_) => halt(),
    };
    let Resources {
        flash,
        supply_monitor,
        sensor_i2c: _,
        status_led,
        user_button,
    } = resources;
    let mut status = PhyStatus::new(status_led);

    let battery = match product::battery::SupplyBattery::new(supply_monitor) {
        Ok(battery) => battery,
        Err(_) => fault(&mut status),
    };

    let mac = match Phy6222Mac::take() {
        Some(mac) => mac,
        None => fault(&mut status),
    };
    let ieee = match product::identity::validate_ieee_address(mac.extended_address()) {
        Ok(ieee) => ieee,
        Err(_) => fault(&mut status),
    };

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
        fault(&mut status);
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
