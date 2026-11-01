//! nRF52833 DK always-on End Device (`--no-default-features --features ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the interrupt-driven nRF PHY (Imm-Ack sent by TIMER0/PPI from the
//! RADIO interrupt) under [`router_app::AlwaysOnEndDeviceApp`]. Basic +
//! Identify on endpoint 1; LED1 shows the online state, Button 1 re-arms
//! commissioning after a Leave. Log: VCOM (UARTE0, 115200 8N1).
//!
//! Identity, policy and the security journal are owned by
//! `products/nrf52833-sensor`; physical resources by `boards/nrf52833-dk`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod vcom_log;

use embassy_executor::Spawner;
use embassy_nrf::nvmc::Nvmc;
use nrf52833_sensor_product as product;
use product::end_device::EndDeviceProfile;
use roles::{halt, CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::nrf::NrfIsrSoftMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, ZigbeeDevice,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = NrfIsrSoftMac<'static, embassy_nrf::peripherals::RNG>;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    product::storage::SecurityStore<'static>,
    EndDeviceProfile,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let started = roles::start(spawner, product::END_DEVICE_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.phy, started.nvmc, started.led);
    if let Err(error) = app.initialize().await {
        roles::fatal(app.parts_mut(), ARCHETYPE, error);
    }
    let mut step_errors = roles::StepErrors::new();
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice.
        match app.step().await {
            Ok(_) => step_errors.ok(),
            Err(error) => step_errors.handle(app.parts_mut(), ARCHETYPE, error).await,
        }
    }
}

/// Build MAC, device, store and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    phy: roles::Phy,
    nvmc: Nvmc<'static>,
    led: embassy_nrf::gpio::Output<'static>,
) -> &'static mut App {
    let mac: Mac = phy.into_soft_mac().unwrap_or_else(|error| {
        log::error!("[NRF] MAC init failed: {error:?}");
        halt()
    });
    let ieee = mac.pib().extended_address();
    roles::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore<'static>> = StaticCell::new();
    static PROFILE: StaticCell<EndDeviceProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::end_device::end_device_profile());
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::END_DEVICE_MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(roles::open_security_store(nvmc));
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    roles::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::END_DEVICE),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = AlwaysOnEndDeviceApp::new(
        node,
        &product::end_device::END_DEVICE_POLICY,
        roles::parts(led),
    )
    .unwrap_or_else(|error| {
        log::error!("[NRF] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}
