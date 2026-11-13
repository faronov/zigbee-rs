//! STM32WB5MM-DK Zigbee Router.
//!
//! CPU2 runs ST's IEEE 802.15.4 MAC firmware: it sends Imm-Acks, answers
//! data polls with Frame Pending, holds indirect frames for sleepy children
//! and transmits beacons. [`Stm32wbMac`] maps that MAC onto zigbee-rs's
//! parent MAC contract, and [`router_app::ParentRouterApp`] runs the Zigbee
//! router on top: routing, association, durable child and APS tables.
//!
//! Button B1: short press re-arms commissioning after a Leave; a 3 s hold
//! commits a journal-aware factory reset. Log: ST-LINK VCOM (USART1,
//! 115200 8N1), with a periodic table and coprocessor summary.

#![no_std]
#![no_main]

mod platform;
mod vcom_log;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use platform::{LogDiagnostics, LogStatus, ResetSupervisor, halt};
use router_app::{
    DiagnosticEvent, Diagnostics, NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables,
    PersistentChildren, RouterStatus, StatusSink, Supervisor,
};
use static_cell::StaticCell;
use stm32wb_hal::embassy_stm32_wpan::sub::mm::MemoryManager;
use stm32wb_hal::mac::WbTransport;
use stm32wb5mm_dk_router_product::{
    self as product,
    button::{ButtonGesture, ButtonGestures},
    router::RouterProfile,
    storage::{ApsTableStore, ChildStore, SecurityStore},
};
use zigbee_mac::stm32wb::Stm32wbMac;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Stm32wbMac<WbTransport>;
type App = ParentRouterApp<
    'static,
    Mac,
    SecurityStore,
    RouterProfile,
    ChildStore,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child/coprocessor summary on VCOM.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

/// Returns mailbox buffers to CPU2; must run for the life of the MAC.
#[embassy_executor::task]
async fn run_memory_manager(mut memory_manager: MemoryManager<'static>) {
    memory_manager.run_queue().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let started = platform::start(spawner, product::ROUTER_MODEL);
    let (transport, memory_manager) =
        match stm32wb_hal::mac::start(started.ipcc, platform::Irqs, started.rng).await {
            Ok(parts) => parts,
            Err(error) => {
                log::error!("[WB] coprocessor MAC start failed: {error:?}");
                halt()
            }
        };
    spawner.spawn(run_memory_manager(memory_manager).unwrap_or_else(|_| halt()));
    log::info!("[WB] coprocessor MAC running");

    let app = compose(transport);
    if let Err(error) = app.initialize().await {
        platform::fatal(app.parts_mut(), ARCHETYPE, error);
    }

    let button = started.button_b1;
    let mut gestures = ButtonGestures::new();
    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    let mut step_errors = platform::StepErrors::new();
    loop {
        match gestures.sample(button.is_low(), Instant::now().as_millis()) {
            Some(ButtonGesture::FactoryReset) => factory_reset(app).await,
            Some(ButtonGesture::CommissioningRequest) if !app.request_commissioning() => {
                log::info!("[WB] B1 ignored: joined or reset pending");
            }
            Some(ButtonGesture::CommissioningRequest) | None => {}
        }
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= product::RECOMMISSION_AFTER_LEAVE_MS {
                awaiting_since = None;
                app.request_commissioning();
            }
        } else {
            awaiting_since = None;
        }

        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline).
        match app.step().await {
            Ok(_) => step_errors.ok(),
            Err(error) => step_errors.handle(app.parts_mut(), ARCHETYPE, error).await,
        }
        if mac(app).transport().faulted() {
            // Opcode and leading octets of the notification CPU2 left
            // pending identify an ABI or sequencing fault after the reset.
            let mut raw = [0u8; 16];
            let len = stm32wb_hal::mac::pending_notification(&mut raw);
            log::error!(
                "[WB] IPCC mailbox stalled; pending notification len {} {:02X?}",
                len,
                &raw[..len.min(raw.len())]
            );
            // Let the VCP drain the fault line before the reset.
            embassy_time::Timer::after_millis(200).await;
            app.parts_mut().supervisor.reset();
        }
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}

fn mac(app: &App) -> &Mac {
    app.node().device().bdb().zdo().nwk().mac()
}

async fn factory_reset(app: &mut App) -> ! {
    app.parts_mut().status.set(RouterStatus::Resetting {
        archetype: ARCHETYPE,
    });
    log::info!("[WB] B1 hold: committing durable factory reset");
    match app.urgent_factory_reset_and_recommission().await {
        Ok(()) => {
            let parts = app.parts_mut();
            parts.diagnostics.record(DiagnosticEvent::FactoryReset);
            parts.supervisor.reset()
        }
        Err(error) => platform::fatal(app.parts_mut(), ARCHETYPE, error),
    }
}

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(transport: WbTransport) -> &'static mut App {
    let ieee = stm32wb_hal::identity::eui64();
    platform::check_ieee(ieee);
    let mac = Stm32wbMac::new(transport, ieee);

    let stores = product::storage::open().unwrap_or_else(|error| {
        log::error!("[WB] journals unavailable: {error:?}");
        halt()
    });

    static SECURITY: StaticCell<SecurityStore> = StaticCell::new();
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

    let security_store = SECURITY.init(stores.security);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    platform::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    platform::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::ROUTER),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps_tables),
        &product::router::ROUTER_POLICY,
        platform::parts(),
    )
    .unwrap_or_else(|error| {
        log::error!("[WB] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}

/// Neighbour/child/route tables and coprocessor counters on VCOM.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    log::info!(
        "[WB] neighbours {} children {} routes {} logdrop {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        vcom_log::dropped_lines(),
    );
    // The VCP logger truncates long lines; keep each under ~120 columns.
    let stats = nwk.mac().stats();
    log::info!(
        "[WB] tx ok {} noack {} cca {} other {} | rx drop {} cmd drop {} stray {}",
        stats.tx_ok,
        stats.tx_no_ack,
        stats.tx_channel_access_failures,
        stats.tx_other_failures,
        stats.rx_dropped,
        stats.commands_dropped,
        stats.stray_confirms,
    );
    log::info!(
        "[WB] indirect {}/{} exp {} fail {} | beacon {} sync-loss {} other {}",
        stats.indirect_delivered,
        stats.indirect_handed_off,
        stats.indirect_expired,
        stats.indirect_failed,
        stats.beacon_refreshes,
        stats.sync_loss,
        stats.other_events,
    );
    for neighbor in neighbors.iter() {
        log::info!(
            "[WB]  nbr 0x{:04X} {:?} rx_on={} lqi={}",
            neighbor.network_address.0,
            neighbor.relationship,
            neighbor.rx_on_when_idle,
            neighbor.lqi
        );
    }
}
