//! LP-EM-CC2340R5 Zigbee Router size probe
//! (`--no-default-features --features router`).
//!
//! A parent router composition: `Cc2340RouterMac` (the shared `SoftMacCore`
//! in the `Parent<8, 8, 0>` role over the CC2340 PHY) under
//! [`router_app::ParentRouterApp`], with durable child and APS tables.
//!
//! **Does not fit the CC2340R52 (36 KiB SRAM).** Measured on the pinned-SDK
//! build against an enlarged measurement map, with the default ZCL/ZDO sizes
//! and the unmodified NWK router tables:
//!
//! - static RAM 32,828 B (`ZigbeeDevice<_, Router>` 22,672 B, main future
//!   4,600 B, security journal 2,468 B, app 2,396 B, RTT 512 B), leaving
//!   4,036 B of stack against the 16 KiB reserve `memory.x` holds every
//!   image to (12,348 B short);
//! - flash 285,524 B, inside the 499,712 B router window `0x0..0x7A000`.
//!
//! `memory.x` and `link/router-tables.x` reject the link. The router-only
//! growth of `ZigbeeDevice` over the End Device is 10,824 B, less than the
//! shortfall, so no table shrink alone makes it fit.
//!
//! Frame Pending: the PBE acknowledges in hardware, always with FP=0, and
//! has no source-match table, so `Cc2340RadioPhy::set_ack_frame_pending`
//! stays `Unsupported` and the `Parent<8, 8, 0>` core reports that as a MAC
//! error instead of silently acknowledging a poll without Frame Pending.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;

use core::cell::RefCell;

use cc2340_sensor_product as product;
use embassy_executor::Spawner;
use lp_em_cc2340r5::flash::InternalFlash;
use product::router::{ApsTableStore, ChildStore, RouterProfile, RouterSecurityStore};
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::cc2340::{CC2340_PENDING_RX, Cc2340RadioPhy, Cc2340RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Cc2340RouterMac;
type App = ParentRouterApp<
    'static,
    Mac,
    RouterSecurityStore<'static, InternalFlash>,
    RouterProfile,
    ChildStore<'static, InternalFlash>,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore<'static, InternalFlash>>,
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
    let app = compose(started.ieee, started.flash, started.led, started.reset);
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
    ieee: zigbee_types::IeeeAddress,
    flash: InternalFlash,
    led: lp_em_cc2340r5::pins::Led1,
    reset: lp_em_cc2340r5::reset::SystemReset,
) -> &'static mut App {
    let phy: Cc2340RadioPhy = roles::radio(ieee);
    let mac: Mac = roles::mac(phy.into_parent_mac::<_, CC2340_PENDING_RX>());

    static FLASH: StaticCell<RefCell<InternalFlash>> = StaticCell::new();
    static SECURITY: StaticCell<RouterSecurityStore<'static, InternalFlash>> = StaticCell::new();
    static PROFILE: StaticCell<RouterProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::router::router_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::ROUTER_MODEL)
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
        .build_router_into(DEVICE.uninit());

    let stores = product::router::router_stores(FLASH.init(RefCell::new(flash)));
    let security_store = SECURITY.init(stores.security);
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
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps),
        &product::router::ROUTER_POLICY,
        roles::parts(led, reset),
    )
    .unwrap_or_else(|error| {
        rtt_target::rprintln!("[CC2340] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}
