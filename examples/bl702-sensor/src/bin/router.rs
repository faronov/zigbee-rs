//! XT-ZB1 Router (`--features router --bin bl702-router`).
//!
//! Routing, child-capable HA Range Extender (Basic + Identify on
//! endpoint 1) under [`router_app::ParentRouterApp`] with persistent child
//! and APS tables (`products/bl702-xt-zb1/src/router.rs`, journals at
//! `0xF9000..0xFD000`) and the shared security journal (role tag
//! `product::role::ROUTER`). No Zigbee OTA client.
//!
//! Radio: [`zigbee_mac::bl702::parent_phy::Bl702ParentRadioPhy`]. The M154
//! hardware sends every Imm-Ack and takes Frame Pending from its own
//! source-address table, which `SoftMacCore` programs through
//! `RadioPhy::set_ack_frame_pending`; the core runs `Parent<8, 8, 0>`
//! (no software Frame Pending table), so the hardware is the single Frame
//! Pending owner. `M154_IRQ` copies each received frame into a ring and
//! keeps the receiver on while idle.
//!
//! Validation: builds, links and passes the role-symbol check; the table
//! logic is host-tested. **Not hardware-tested**: auto-ACK timing, the FPT
//! encoding and the interrupt path need an XT-ZB1 with a sniffer.

#![no_std]
#![no_main]

#[path = "../hal.rs"]
#[allow(dead_code)]
mod hal;
#[path = "../roles.rs"]
mod roles;
#[path = "../time_driver.rs"]
#[allow(dead_code)]
mod time_driver;

use bl702_hal::flash::XipFlash;
use bl702_hal::interrupt::{self, M154_IRQ};
use bl702_xt_zb1_product as product;
use embassy_executor::Executor;
use panic_halt as _;
use product::router::{ApsTableStore, ChildStore, RouterProfile, RouterSecurityStore};
use roles::{CommissioningTrigger, LogDiagnostics, LogStatus, ResetSupervisor};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::bl702::parent_phy::{BL702_ROUTER_PENDING_RX, Bl702RouterMac, Bl702RouterRole};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Bl702RouterMac;
type App = ParentRouterApp<
    'static,
    Mac,
    RouterSecurityStore<'static, XipFlash>,
    RouterProfile,
    ChildStore<'static, XipFlash>,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore<'static, XipFlash>>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;

#[riscv_rt::entry]
fn main() -> ! {
    let resources = roles::boot(product::ROUTER_MODEL);
    let mut executor = Executor::new();
    // SAFETY: `main` never returns, so the executor lives forever.
    let executor: &'static mut Executor = unsafe { core::mem::transmute(&mut executor) };
    executor.run(move |spawner| spawner.must_spawn(run(resources)))
}

#[embassy_executor::task]
async fn run(resources: hal::ApplicationResources) {
    let started = roles::start(resources);
    let mut trigger = CommissioningTrigger::default();
    let app = compose(started);
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
fn compose(started: roles::Started) -> &'static mut App {
    let roles::Started {
        ieee,
        radio,
        pib,
        flash,
    } = started;
    let mac: Mac = roles::mac(
        radio
            .into_parent_phy()
            .into_parent_mac::<Bl702RouterRole, BL702_ROUTER_PENDING_RX>(pib),
    );
    // The receive ring and the hardware-ACK re-arm run from M154_IRQ
    // (`time_driver::dispatch`); the parent radio is idle until the MAC
    // installs the address filter.
    interrupt::clear_pending(M154_IRQ);
    // SAFETY: the dispatcher installed by `hal::init` handles M154_IRQ.
    unsafe { interrupt::enable(M154_IRQ) };

    static SECURITY: StaticCell<RouterSecurityStore<'static, XipFlash>> = StaticCell::new();
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
        .channels(product::CHANNEL_MASK)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_router_into(DEVICE.uninit());

    let stores = product::router::router_stores(flash);
    let security_store = SECURITY.init(stores.security);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard(device.reset_security_state_if_identity_changed(security_store, ieee));
    roles::guard(security_store.select_role_tag(product::role::ROUTER));

    let node = ZigbeeNode::new(device, security_store, profile);
    let Ok(app) = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps),
        &product::router::ROUTER_POLICY,
        roles::parts(),
    ) else {
        hal::halt()
    };
    APP.init(app)
}
