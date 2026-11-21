//! CC2652P / CC2652P7 USB dongle Zigbee Router.
//!
//! A parent router: `Cc26x2RouterMac` (the shared `SoftMacCore` in the
//! `Parent` role over the CC26x2 RF core PHY; the RF core sends every
//! Imm-Ack, with Frame Pending from its source-match tables that the core
//! arms for queued indirect frames) under [`router_app::ParentRouterApp`]:
//! beacons, association, indirect TX to sleepy children, routing, durable
//! child and APS tables.
//!
//! Identity, policy, TX power, CCFG and the journals are owned by
//! `products/cc26x2-dongle`; physical resources by `boards/cc26x2-dongle`.
//! The log is plain text on the dongle's UART (115200 8N1).

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;

use cc26x2_dongle_product as product;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use platform::halt;
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, ParentRouterApp, PersistentApsTables,
    PersistentChildren, RouterAppError, RouterParts, RouterStatus, StackEventSummary, StatusSink,
    Supervisor,
};
use static_cell::StaticCell;
use zigbee_mac::cc26x2::{CC26X2_ROUTER_PENDING_RX, Cc26x2RadioPhy, Cc26x2RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::{ChannelMask, ShortAddress};
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use cc26x2_hal::vectors::__INTERRUPTS;

type Mac = Cc26x2RouterMac<{ product::TX_POWER_DBM }>;
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
/// Channels network steering scans: all 2.4 GHz channels, or the single
/// channel given at build time in `CC26X2_ROUTER_CHANNEL` (TEST-ONLY lab
/// images: keeps a bench router off every other network, e.g. an isolated
/// channel while unattended, or the lab coordinator's channel for a join).
const CHANNELS: ChannelMask = match option_env!("CC26X2_ROUTER_CHANNEL") {
    None => ChannelMask::ALL_2_4GHZ,
    Some(text) => ChannelMask(1 << parse_channel(text)),
};

const fn parse_channel(text: &str) -> u32 {
    let b = text.as_bytes();
    assert!(b.len() == 2, "CC26X2_ROUTER_CHANNEL must be 11..=26");
    let channel = ((b[0] - b'0') * 10 + (b[1] - b'0')) as u32;
    assert!(
        channel >= 11 && channel <= 26,
        "CC26X2_ROUTER_CHANNEL must be 11..=26"
    );
    channel
}
/// Period of the neighbour/child summary on the log.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Keep the product CCFG in the image.
    let _ = unsafe { core::ptr::read_volatile(&product::ccfg::CCFG[0]) };
    let started = platform::start(product::ROUTER_MODEL);
    if ACK_PROBE {
        log::warn!("[CC26X2] TEST-ONLY unicast ACK probe enabled");
    }
    if CHANNELS.0 != ChannelMask::ALL_2_4GHZ.0 {
        log::warn!("[CC26X2] TEST-ONLY channel mask 0x{:08X}", CHANNELS.0);
    }
    let app = compose(started);
    if let Err(error) = app.initialize().await {
        fatal(app.parts_mut(), error);
    }

    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    let mut probe_slot: Option<usize> = None;
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
            if ACK_PROBE {
                ack_probe(app, &mut probe_slot).await;
            }
        }
    }
}

/// TEST-ONLY (`CC26X2_ROUTER_ACK_PROBE=1` at build time): once per summary
/// period, send a unicast IEEE_addr_req to the coordinator. Its response
/// arrives as a MAC unicast with AR set, which the RF core must
/// auto-acknowledge (compare `ar-unicast` with `auto-acks`).
const ACK_PROBE: bool = match option_env!("CC26X2_ROUTER_ACK_PROBE") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

async fn ack_probe(app: &mut App, slot: &mut Option<usize>) {
    let zdo = app.node_mut().device_mut().bdb_mut().zdo_mut();
    if let Some(previous) = slot.take() {
        let answered = zdo.take_response(previous).is_some();
        zdo.cancel_pending(previous);
        log::info!("[CC26X2] TEST-ONLY ack probe answered: {answered}");
    }
    if !zdo.nwk().is_joined() {
        return;
    }
    match zdo.start_ieee_addr_req(ShortAddress(0x0000)).await {
        Ok((pending, tsn)) => {
            log::info!("[CC26X2] TEST-ONLY ack probe IEEE_addr_req tsn {tsn} to 0x0000");
            *slot = Some(pending);
        }
        Err(status) => log::warn!("[CC26X2] TEST-ONLY ack probe failed: {status:?}"),
    }
}

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the large builder temporaries are released before the run loop.
#[inline(never)]
fn compose(started: platform::Started) -> &'static mut App {
    let platform::Started {
        identity,
        radio,
        trng,
        crypto,
        flash,
    } = started;
    let radio = radio.init(product::TX_POWER_DBM).unwrap_or_else(|error| {
        log::error!("[CC26X2] RF core init failed: {error:?}");
        halt()
    });
    let trng = cc26x2_hal::trng::Trng::new(trng).unwrap_or_else(|error| {
        log::error!("[CC26X2] TRNG start failed: {error:?}");
        halt()
    });
    let mut phy = Cc26x2RadioPhy::new(radio, trng, identity.eui64).unwrap_or_else(|error| {
        log::error!("[CC26X2] PHY init failed: {error:?}");
        halt()
    });
    // NWK/APS AES-128 on the CRYPTO engine (no software AES in the image).
    if let Err(error) = phy.install_aes_engine(crypto) {
        log::error!("[CC26X2] AES engine rejected: {error:?}");
        halt()
    }
    let mac: Mac = phy.into_parent_mac().unwrap_or_else(|error| {
        log::error!("[CC26X2] MAC init failed: {error:?}");
        halt()
    });
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[CC26X2] no valid factory EUI-64");
        halt()
    }
    log::info!("[CC26X2] EUI-64 {:02X?}", ieee);

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
        .channels(CHANNELS)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_router_into(DEVICE.uninit());

    let (flash, child_store, aps_store) = product::router::router_stores(flash);
    #[cfg(feature = "green-power")]
    let (flash, gp_store) = product::router::gp_proxy_table_store(flash);
    let (security_store, migration) =
        product::storage::security_store(flash).unwrap_or_else(|error| {
            log::error!("[CC26X2] security journal open failed: {error:?}");
            halt()
        });
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[CC26X2] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[CC26X2] foreign/blank persistence erased; fresh join required")
        }
    }
    let security_store = SECURITY.init(security_store);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    guard_outcome(
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
        log::error!("[CC26X2] invalid composition: {error:?}");
        halt()
    });
    #[cfg(feature = "green-power")]
    let app = {
        static GP_STORE: StaticCell<product::router::GpProxyTableStore> = StaticCell::new();
        let mut app = app;
        app.set_green_power_store(GP_STORE.init(gp_store));
        app
    };
    APP.init(app)
}

fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[CC26X2] cleared network state: {what} changed"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[CC26X2] {what} guard failed: {error:?}");
            halt()
        }
    }
}

/// Neighbour/child table and RF core health on the log.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let phy = nwk.mac().phy();
    let radio = phy.radio_stats();
    let (last_error, rx_status) = phy.last_error();
    log::info!(
        "[CC26X2] neighbours {} children {} routes {} | rx {} restarts {} ar-unicast {} auto-acks {} buf-full {} nok {} | tx {} no-ack {} busy {} err {} | rx 0x{:04X} last-err {:?}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        radio.rx_frames,
        radio.rx_restarts,
        radio.rx_ack_requested,
        radio.auto_acks,
        radio.rx_buf_full,
        radio.rx_nok,
        radio.tx_frames,
        radio.tx_no_ack,
        radio.tx_busy,
        radio.tx_errors,
        rx_status,
        last_error
    );
    let security = nwk.rx_security_stats();
    log::info!(
        "[CC26X2] nwk-sec secured {} ok {} replay {} mic-fail {}",
        security.secured_frames,
        security.decrypt_successes,
        security.replay_rejections,
        security.decrypt_failures
    );
    for child in neighbors.children() {
        log::info!(
            "[CC26X2]  child 0x{:04X} rx_on={} lqi={}",
            child.network_address.0,
            child.rx_on_when_idle,
            child.lqi
        );
    }
}

/// No LED is wired: report lifecycle changes on the log.
#[derive(Default)]
struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            log::info!("[CC26X2] status {status:?}");
        }
    }
}

struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[CC26X2] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[CC26X2] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[CC26X2] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[CC26X2] router online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[CC26X2] {count} children restored")
            }
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[CC26X2] child table discarded: {error:?}")
            }
            DiagnosticEvent::ChildTableSaved => log::info!("[CC26X2] child table saved"),
            DiagnosticEvent::ChildTableCleared => log::info!("[CC26X2] child table cleared"),
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!("[CC26X2] child 0x{short_address:04X} removed (delivered={delivered})"),
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[CC26X2] {count} APS table entries restored")
            }
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[CC26X2] APS tables discarded: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[CC26X2] permit join {open}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!("[CC26X2] device announce 0x{short_address:04X} {address:02X?}"),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[CC26X2] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[CC26X2] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[CC26X2] factory-new after Leave; recommissioning in {} s",
                product::policy::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[CC26X2] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[CC26X2] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[CC26X2] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[CC26X2] secure rejoin limit reached at {failures}")
            }
            DiagnosticEvent::FactoryReset => log::info!("[CC26X2] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[CC26X2] fatal: {error:?}"),
            _ => {}
        }
    }
}

/// No watchdog yet; fatal errors reset the MCU.
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
const _: () = assert!(CC26X2_ROUTER_PENDING_RX >= 4);
