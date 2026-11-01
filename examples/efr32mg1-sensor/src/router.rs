//! EFR32MG1P TRÅDFRI Zigbee Router size probe
//! (`--no-default-features --features router`).
//!
//! A parent router composition: `Efr32RouterMac` (the shared `SoftMacCore`
//! in the `Parent<8, 8, 0>` role over the EFR32MG1 PHY) under
//! [`router_app::ParentRouterApp`], with durable child and APS tables.
//!
//! **Does not fit the EFR32MG1P132F256 (0x7C00 B SRAM, 256 KiB flash).**
//! Measured against an enlarged measurement map with the
//! `constrained-memory` ZCL/ZDO sizes and the unmodified NWK router tables:
//!
//! - flash 248,344 B against the 192,512 B router window `0x4000..0x33000`
//!   (55,832 B over; the child/APS journals occupy `0x33000..0x37000`);
//! - static RAM 29,072 B (`ZigbeeDevice<_, Router>` 18,032 B, main future
//!   4,568 B, security journal 2,464 B, app 2,388 B, radio buffers 768 B),
//!   leaving 2,672 B of stack against the 16 KiB reserve (13,712 B short).
//!
//! `link/router-tables.x` rejects the link with both reasons. The RAM
//! shortfall exceeds the whole router-only growth of `ZigbeeDevice`
//! (11,120 B over the End Device), so no table shrink alone makes it fit.
//!
//! Frame Pending: the MG1 FRC ISR pre-arms every Imm-Ack with FP=0 and has no
//! source-match table yet, so `Efr32RadioPhy::set_ack_frame_pending` stays
//! `Unsupported` and the `Parent<8, 8, 0>` core reports that as a MAC error
//! instead of silently acknowledging a poll without Frame Pending.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod time_driver;
mod vectors;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use efr32mg1_tradfri_product as product;
use embassy_executor::Spawner;
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;
use zigbee_mac::efr32::{Efr32RadioPhy, Efr32RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Efr32RouterMac;
type App = ParentRouterApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    RouterProfile,
    ChildStore,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = roles::start(product::ROUTER_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.crypto, started.flash, started.led);
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

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    crypto: efr32mg1_hal::peripherals::Crypto,
    flash: efr32mg1_hal::flash::Efr32mg1Flash,
    led: efr32mg1_tradfri::Led,
) -> &'static mut App {
    let mut phy = Efr32RadioPhy::new();
    phy.install_aes_engine(crypto).unwrap_or_else(|_| halt());
    let mac: Mac = phy.into_parent_mac().unwrap_or_else(|_| halt());
    let ieee = mac.pib().extended_address();
    roles::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<RouterProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::router::router_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::ROUTER_MODEL)
        .application_version(FIRMWARE_APPLICATION_VERSION)
        .date_code(product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_router_into(DEVICE.uninit());

    let (flash, child_store, aps_store) = product::router::router_stores(flash);
    let (security_partition, _application_nv) = product::storage::split_flash(flash);
    let security_store = SECURITY.init(product::storage::security_store(security_partition));
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    roles::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::ROUTER),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(child_store),
        PersistentApsTables::new(aps_store),
        &product::router::ROUTER_POLICY,
        roles::parts(led),
    )
    .unwrap_or_else(|error| {
        rtt_target::rprintln!("[EFR32] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}
