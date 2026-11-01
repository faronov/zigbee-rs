//! PHY62x2 EVK always-on End Device
//! (`--no-default-features --features ed,phy6222` or `ed,phy6252`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the PHY62x2 radio (`LL_IRQ` sends the Imm-Ack, never with Frame
//! Pending) under [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on
//! endpoint 1; the status LED shows the online state and the user button
//! re-arms commissioning after a Leave.
//!
//! Identity, policy and the security journal are owned by
//! `products/phy62x2-evk`; physical resources by `boards/phy62x2-evk`. The
//! image keeps the sensor's 187,104 B XIP slot and security journal.
//!
//! Validation: builds and passes `check-layout.sh` only; the EVK images
//! are not hardware-validated (the radio driver is shared with the
//! hardware-proven PB-03F-Kit).

#![no_std]
#![no_main]

mod roles;

use cortex_m as _;
use embassy_executor::Spawner;
use panic_halt as _;
use phy62x2_evk::{InternalFlash, StatusLed};
use phy62x2_evk_product as product;
use product::end_device::EndDeviceProfile;
use roles::{CommissioningTrigger, LedStatus, NoDiagnostics, ResetSupervisor};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::phy6222::{Phy6222RadioPhy, Phy6222SoftMac};
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile,
};
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Phy6222SoftMac;
type SecurityStore = product::storage::HardwareSecurityStore;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    SecurityStore,
    EndDeviceProfile,
    LedStatus,
    ResetSupervisor,
    NoDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let roles::Started {
        ieee,
        radio,
        flash,
        led,
        button,
    } = roles::start();
    let mut trigger = CommissioningTrigger::new(button);
    let app = compose(ieee, radio, flash, led);
    if let Err(error) = app.initialize().await {
        roles::fatal(app.parts_mut(), ARCHETYPE, error);
    }
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice.
        if let Err(error) = app.step().await {
            roles::fatal(app.parts_mut(), ARCHETYPE, error);
        }
    }
}

/// Build MAC, device, store and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    ieee: zigbee_types::IeeeAddress,
    radio: Phy6222RadioPhy,
    flash: InternalFlash,
    mut led: StatusLed,
) -> &'static mut App {
    let mac: Mac = roles::mac(radio.into_soft_mac(), &mut led);

    static SECURITY: StaticCell<SecurityStore> = StaticCell::new();
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
        .channels(product::CHANNELS)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(product::storage::security_store(flash));
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard(
        device.reset_security_state_if_identity_changed(security_store, ieee),
        &mut led,
    );
    roles::guard(
        security_store.select_role_tag(product::role::END_DEVICE),
        &mut led,
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let Ok(app) = AlwaysOnEndDeviceApp::new(
        node,
        &product::end_device::END_DEVICE_POLICY,
        roles::parts(led),
    ) else {
        roles::halt()
    };
    APP.init(app)
}
