//! Home Assistant Connect ZBT-1 Zigbee Router (`--features router`).
//!
//! A true parent router: `Efr32s2RouterMac` (the shared `SoftMacCore` in the
//! `Parent` role over the Series 2 PHY with software Imm-Acks, so Frame
//! Pending follows the core's indirect queue) under
//! [`router_app::ParentRouterApp`]: beacons, association, indirect TX to
//! sleepy children, routing, durable child and APS tables.
//!
//! Product identity, policy, TX power and the journals are owned by
//! `products/ha-connect-zbt1`; physical resources by `boards/ha-connect-zbt1`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;
mod rtt_log;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use ha_connect_zbt1_product as product;
use platform::halt;
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, ParentRouterApp, PersistentApsTables,
    PersistentChildren, RouterAppError, RouterParts, RouterStatus, StackEventSummary, StatusSink,
    Supervisor,
};
use static_cell::StaticCell;
use zigbee_mac::efr32s2::{EFR32S2_ROUTER_PENDING_RX, Efr32s2RadioPhy, Efr32s2RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Mac = Efr32s2RouterMac<{ product::TX_POWER_DBM }>;
type Parts = RouterParts<LogStatus, ResetSupervisor, LogDiagnostics>;
type App = ParentRouterApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    RouterProfile,
    ChildStore,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    router_app::NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child summary on RTT.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = platform::start(product::ROUTER_MODEL);
    let app = compose(started.radio, started.flash);
    if let Err(error) = app.initialize().await {
        fatal(app.parts_mut(), error);
    }

    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    loop {
        // No button: re-arm commissioning automatically after a Leave.
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= product::policy::RECOMMISSION_AFTER_LEAVE_MS {
                app.request_commissioning();
                awaiting_since = None;
            }
        } else {
            awaiting_since = None;
        }

        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline); the receiver stays on between slices.
        if let Err(error) = app.step().await {
            fatal(app.parts_mut(), error);
        }

        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the large builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> &'static mut App {
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[ZBT1] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_parent_mac()
                .map_err(|error| log::error!("[ZBT1] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| halt());
    let ieee = mac.pib().extended_address();
    platform::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
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

    let (flash, child_store, aps_store) = product::router::router_stores(flash);
    let security_store = SECURITY.init(platform::open_security_store(flash));
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
        PersistentChildren::new(child_store),
        PersistentApsTables::new(aps_store),
        &product::router::ROUTER_POLICY,
        RouterParts::new(LogStatus::default(), ResetSupervisor, LogDiagnostics),
    )
    .unwrap_or_else(|error| {
        log::error!("[ZBT1] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}

/// Neighbour/child table and parent-MAC health on RTT.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let phy = nwk.mac().phy();
    log::info!(
        "[ZBT1] neighbours {} children {} routes {} stale-ack drops {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        phy.stale_ack_drops()
    );
    for child in neighbors.children() {
        log::info!(
            "[ZBT1]  child 0x{:04X} rx_on={} lqi={}",
            child.network_address.0,
            child.rx_on_when_idle,
            child.lqi
        );
    }
}

/// No LED is wired to the EFR32: report lifecycle changes on RTT.
#[derive(Default)]
struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            log::info!("[ZBT1] status {status:?}");
        }
    }
}

struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[ZBT1] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[ZBT1] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[ZBT1] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[ZBT1] router online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[ZBT1] {count} children restored")
            }
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[ZBT1] child table discarded: {error:?}")
            }
            DiagnosticEvent::ChildTableSaved => log::info!("[ZBT1] child table saved"),
            DiagnosticEvent::ChildTableCleared => log::info!("[ZBT1] child table cleared"),
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!("[ZBT1] child 0x{short_address:04X} removed (delivered={delivered})"),
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[ZBT1] {count} APS table entries restored")
            }
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[ZBT1] APS tables discarded: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[ZBT1] permit join {open}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!("[ZBT1] device announce 0x{short_address:04X} {address:02X?}"),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[ZBT1] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[ZBT1] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[ZBT1] factory-new after Leave; recommissioning in {} s",
                product::policy::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[ZBT1] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[ZBT1] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[ZBT1] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[ZBT1] secure rejoin limit reached at {failures}")
            }
            DiagnosticEvent::FactoryReset => log::info!("[ZBT1] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[ZBT1] fatal: {error:?}"),
            _ => {}
        }
    }
}

/// No watchdog claim yet; fatal errors reset the MCU.
struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

fn fatal(parts: &mut Parts, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault {
        archetype: ARCHETYPE,
    });
    parts.supervisor.reset()
}

// The router PHY retains this many received frames between runtime calls.
const _: () = assert!(EFR32S2_ROUTER_PENDING_RX >= 4);
