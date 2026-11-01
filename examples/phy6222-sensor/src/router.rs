//! PHY62x2 EVK Router size probe
//! (`--no-default-features --features router,phy6222` or `router,phy6252`).
//!
//! **Does not fit the XIP application slot.** Measured against an enlarged
//! measurement map (default ZCL/ZDO sizes, unmodified NWK router tables,
//! `constrained-memory` as in the sensor):
//!
//! - XIP occupied span 221,120 B against the 187,104 B slot (34,016 B
//!   over; the End Device is 136,544 B, so the router adds 84,576 B of
//!   code). On the PHY6252 the router journals also take 0x3A000..0x3E000
//!   from the slot, leaving 170,720 B (50,400 B over);
//! - static RAM fits: `.bss` 39,488 B (`ZigbeeDevice<_, Router>` 18,040 B,
//!   executor arena 16,388 B, security journal 2,468 B, app 2,384 B),
//!   leaving 18,624 B of stack against the 16 KiB floor of
//!   `link/router-tables-*.x`.
//!
//! `phy6222.x` (and, on the PHY6252, `router-tables-phy6252.x`) reject the
//! link. Table sizes do not drive the overflow (it is code), so no table
//! shrink makes it fit; see the platform guide for the layout options.
//!
//! Frame Pending: `LL_IRQ` sends every Imm-Ack, always with FP=0, and has
//! no source-match table yet, so `Phy6222RadioPhy::set_ack_frame_pending`
//! stays `Unsupported` and the `Parent<8, 8, 0>` core reports that as a MAC
//! error instead of silently acknowledging a poll without Frame Pending.

#![no_std]
#![no_main]

mod roles;

use core::cell::RefCell;

use cortex_m as _;
use embassy_executor::Spawner;
use panic_halt as _;
use phy62x2_evk::{InternalFlash, StatusLed};
use phy62x2_evk_product as product;
use product::router::{ApsTableStore, ChildStore, RouterProfile, RouterSecurityStore};
use roles::{CommissioningTrigger, LedStatus, NoDiagnostics, ResetSupervisor};
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
    NoDiagnostics,
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

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    ieee: zigbee_types::IeeeAddress,
    radio: Phy6222RadioPhy,
    flash: InternalFlash,
    mut led: StatusLed,
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
