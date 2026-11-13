//! Home Assistant Connect ZBT-1 Zigbee Router (`--features router`).
//!
//! A true parent router: `Efr32s2RouterMac` (the shared `SoftMacCore` in the
//! `Parent` role over the Series 2 PHY; the HAL `isr-ack` interrupt sends
//! each Imm-Ack in time, with Frame Pending from the HAL source-match table
//! the core arms for queued indirect frames) under
//! [`router_app::ParentRouterApp`]: beacons, association, indirect TX to
//! sleepy children, routing, durable child and APS tables, and a Green Power
//! Proxy Basic with a durable Proxy Table at `0xF0000` (`router-core` builds
//! without it).
//!
//! Product identity, policy, TX power and the journals are owned by
//! `products/ha-connect-zbt1`; physical resources by `boards/ha-connect-zbt1`.
//!
//! With feature `zrs-boot` (combined with `router`, or `router-core`
//! without Green Power) the image runs from the primary slot of the
//! zigbee-rs loader (`products/ha-connect-zbt1/boot/zbt1.layout`), chained
//! after the Gecko Bootloader. It logs which loader started it and, once
//! back on the network, confirms an image the loader installed on trial
//! (`product::boot::TrialConfirm`). The GP Proxy Table journal, the router
//! tables and the security journal stay at the direct map's addresses.

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
    #[cfg(feature = "zrs-boot")]
    let (flash, mut trial) = loader_trial(started.flash);
    #[cfg(not(feature = "zrs-boot"))]
    let flash = started.flash;
    let app = compose(started.radio, flash);
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

        #[cfg(feature = "zrs-boot")]
        trial.confirm_when_online(app);

        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}

/// The running image's trial, confirmed once the router is back online.
#[cfg(feature = "zrs-boot")]
struct LoaderTrial {
    confirm: product::boot::TrialConfirm<product::boot::BootFlash>,
    /// `trial-no-confirm` reported that it leaves the trial running.
    skipped: bool,
}

/// Split the flash for the loader view, report which loader started this
/// image and read its trial state.
#[cfg(feature = "zrs-boot")]
fn loader_trial(
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> (efr32s2_hal::flash::Efr32s2Flash, LoaderTrial) {
    let (flash, mut low) = product::boot::split_flash(flash);
    let channel = product::boot::channel(&mut low);
    let confirm = product::boot::TrialConfirm::new(low);
    let (trial, boots) = trial_label(confirm.state());
    log::info!(
        "[ZBT1] ZBT1 app {}{}: loader {channel:?}, image 0x{:04X}/0x{:04X} file 0x{:08X}, trial {trial} {boots}",
        product::SW_BUILD,
        if product::VERSION_OVERRIDDEN {
            " (TEST-ONLY version)"
        } else {
            ""
        },
        product::boot::MANUFACTURER_CODE,
        product::boot::IMAGE_TYPE,
        product::boot::FILE_VERSION,
    );
    (
        flash,
        LoaderTrial {
            confirm,
            skipped: false,
        },
    )
}

#[cfg(feature = "zrs-boot")]
impl LoaderTrial {
    /// Confirm the trial once the router is online and NWK security works
    /// on the air; one attempt per boot.
    fn confirm_when_online(&mut self, app: &App) {
        if !self.confirm.pending() || self.skipped {
            return;
        }
        let online = matches!(app.parts().status.last, Some(RouterStatus::Online { .. }));
        let decrypted = app
            .node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .rx_security_stats()
            .decrypt_successes;
        if !product::boot::confirm_due(online, decrypted) {
            return;
        }
        if cfg!(feature = "trial-no-confirm") {
            self.skipped = true;
            log::warn!("[ZBT1] trial-no-confirm: online, leaving the trial unconfirmed");
            return;
        }
        let (before, boots) = trial_label(self.confirm.confirm());
        let (after, _) = trial_label(self.confirm.state());
        log::info!("[ZBT1] trial confirm: {before} {boots} -> {after}");
    }
}

/// Compact trial-state label for RTT (no `Debug` of the loader types):
/// `(state, boots on trial)`.
#[cfg(feature = "zrs-boot")]
fn trial_label(
    state: Result<product::boot::TrialState, product::boot::BootError>,
) -> (&'static str, u16) {
    use product::boot::TrialState;
    match state {
        Ok(TrialState::Permanent) => ("permanent", 0),
        Ok(TrialState::Testing { attempts }) => ("testing", attempts),
        Ok(TrialState::Confirmed) => ("confirmed", 0),
        Err(_) => ("unreadable", 0),
    }
}

/// Identity read by `tools/zrs_boot_ota.py` from the ELF.
#[cfg(feature = "zrs-boot")]
#[unsafe(no_mangle)]
#[used]
static ZRS_OTA_IDENTITY: [u32; 2] = product::boot::OTA_IDENTITY;

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
    #[cfg(feature = "green-power")]
    let (flash, gp_store) = product::router::gp_proxy_table_store(flash);
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
    #[cfg(feature = "green-power")]
    let app = {
        static GP_STORE: StaticCell<product::router::GpProxyTableStore> = StaticCell::new();
        let mut app = app;
        app.set_green_power_store(GP_STORE.init(gp_store));
        app
    };
    APP.init(app)
}

/// Neighbour/child table and parent-MAC health on RTT.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let phy = nwk.mac().phy();
    let radio = phy.radio_stats();
    let ack = phy.ack_stats();
    log::info!(
        "[ZBT1] neighbours {} children {} routes {} rx {} rx-ovf {} tx {} tx-fail {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        radio.rx_frames,
        radio.rx_overflows,
        radio.tx_frames,
        radio.tx_failures
    );
    log::info!(
        "[ZBT1] isr-ack acks {} fp {} late {} filtered {} ovf {} queue-full {} unacked {} resync {} ack-fail {} rx-restart-fail {} max-turnaround {}us",
        ack.acks,
        ack.fp_acks,
        ack.late,
        ack.filtered,
        ack.overflows,
        ack.queue_full,
        ack.queue_full_unacked,
        ack.resyncs,
        ack.ack_tx_fail,
        ack.rx_restart_fail,
        ack.max_turnaround_us
    );
    let security = nwk.rx_security_stats();
    log::info!(
        "[ZBT1] nwk-sec secured {} ok {} replay {} replay-table-full {} mic-fail {}",
        security.secured_frames,
        security.decrypt_successes,
        security.replay_rejections,
        security.replay_table_full,
        security.decrypt_failures
    );
    for child in neighbors.children() {
        log::info!(
            "[ZBT1]  child 0x{:04X} rx_on={} lqi={}",
            child.network_address.0,
            child.rx_on_when_idle,
            child.lqi
        );
    }
    #[cfg(feature = "green-power")]
    {
        let proxy = app.node().device().gp_proxy();
        let stats = proxy.forward_stats();
        log::info!(
            "[ZBT1] GP proxy entries {} | forwards sent {} cancelled {} (other proxy first)",
            proxy.entries().len(),
            stats.sent,
            stats.cancelled
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
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableRestored { count } => {
                log::info!("[ZBT1] {count} GP proxy table entries restored")
            }
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableDiscarded { error } => {
                log::warn!("[ZBT1] GP proxy table discarded: {error:?}")
            }
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableSaved => log::info!("[ZBT1] GP proxy table saved"),
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableCleared => log::info!("[ZBT1] GP proxy table cleared"),
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
