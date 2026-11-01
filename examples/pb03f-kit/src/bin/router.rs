//! PB-03F-Kit Router size probe (`--no-default-features --features router`).
//!
//! **Does not fit the XIP application slot.** Measured against an enlarged
//! measurement map (default ZCL/ZDO sizes, unmodified NWK router tables,
//! `constrained-memory`, console logging as in the other kit images):
//!
//! - XIP occupied span 297,696 B against the 200,448 B slot (97,248 B
//!   over; the End Device is 195,328 B, so the router adds 102,368 B of
//!   code and `Debug` text);
//! - static RAM fits: `.bss` 39,520 B (`ZigbeeDevice<_, Router>` 18,040 B,
//!   executor arena 16,388 B, security journal 2,468 B, app 2,400 B),
//!   leaving 18,584 B of stack against the 16 KiB floor of
//!   `link/router-tables.x`.
//!
//! `pb03f.x` rejects the link. Table sizes do not drive the overflow (it is
//! code), so no table shrink makes it fit; see the platform guide for the
//! layout options.
//!
//! Frame Pending: `LL_IRQ` sends every Imm-Ack, always with FP=0, and has
//! no source-match table yet, so `Phy6222RadioPhy::set_ack_frame_pending`
//! stays `Unsupported` and the `Parent<8, 8, 0>` core reports that as a MAC
//! error instead of silently acknowledging a poll without Frame Pending.

#![no_std]
#![no_main]

#[path = "../roles.rs"]
mod roles;

use core::cell::RefCell;

use cortex_m as _;
use embassy_executor::Spawner;
use panic_halt as _;
use pb03f_kit::{InternalFlash, Led};
use pb03f_kit_product as product;
use product::router::{ApsTableStore, ChildStore, RouterProfile, RouterSecurityStore};
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::phy6222::{PHY6222_ROUTER_PENDING_RX, Phy6222RadioPhy, Phy6222RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Phy6222RouterMac;
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let roles::Started {
        ieee,
        radio,
        flash,
        led,
        button,
    } = roles::start(product::ROUTER_MODEL);
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

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    ieee: zigbee_types::IeeeAddress,
    radio: Phy6222RadioPhy,
    flash: InternalFlash,
    mut led: Led,
) -> &'static mut App {
    let mac: Mac = roles::mac(
        radio.into_parent_mac::<_, PHY6222_ROUTER_PENDING_RX>(),
        &mut led,
    );

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
        .channels(product::CHANNELS)
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
    roles::guard(
        device.reset_security_state_if_identity_changed(security_store, ieee),
        &mut led,
    );
    roles::guard(
        security_store.select_role_tag(product::role::ROUTER),
        &mut led,
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let Ok(app) = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps),
        &product::router::ROUTER_POLICY,
        roles::parts(led),
    ) else {
        roles::halt()
    };
    APP.init(app)
}
