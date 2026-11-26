//! zigbee-rs Router for EFR32MG24 boards (composition root): an HA Range
//! Extender with the Green Power Proxy Basic on the RF-BM-MG24B1 dev board
//! (default), the SONOFF Dongle-M (`--features sonoff-dongle-m`) or a
//! BRD4187 radio board on a BRD4001A WSTK (`--features wstk-brd4187`).
//!
//! The radio is the pure-Rust Series 2 driver as a parent MAC (interrupt
//! Imm-Ack, HAL-owned Frame Pending); the lifecycle is
//! [`router_app::ParentRouterApp`] with durable child, APS and GP Proxy
//! tables in the product's shared journal windows. Logs go out on the host
//! USART at 115 200 (TX only). A panic, a HardFault, an unrecoverable
//! router-app error or a poisoned Secure Engine ends in a reset that
//! records its cause (`product::fatal`); the next boot logs it.
//!
//! Status: on a BRD4187A (xG24 20 dBm PA, 39 MHz) it joined a test network,
//! passed the interview, sent Imm-Acks from the interrupt in 174-181 us
//! (sniffer) and resumed from its journals after a reset and a reflash.
//! RF-BM-MG24B1 images (10 dBm PA, no captured table) and Dongle-M images
//! (38.4 MHz crystal, PHY not verified) fail radio init with `Unsupported`,
//! log it and halt.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use efr32mg24_product as product;
use efr32s2_hal::usart::UartTx;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use product::board::resources::BoardResources;
use product::fatal::Fatal;
use product::router::{RECOMMISSION_AFTER_LEAVE_MS, ROUTER_POLICY, RouterProfile};
use product::stores::{self, Opened};
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
    stores::SecurityStore,
    RouterProfile,
    stores::ChildStore,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    router_app::NoObserver,
    PersistentApsTables<stores::ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);
/// Log line rate (the Gecko bootloader's and the stock firmware's rate).
const LOG_BAUD: u32 = 115_200;

static LOG_UART: Mutex<RefCell<Option<UartTx>>> = Mutex::new(RefCell::new(None));

struct UartLogger;

impl log::Log for UartLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }
    fn log(&self, record: &log::Record<'_>) {
        // Borrow the UART in a short critical section and write the line
        // with interrupts enabled: a blocking line takes ~9 ms at 115 200
        // baud, and masking interrupts that long delays the FRC interrupt's
        // Imm-Acks past the sender's 864 us wait (sniffer-observed on the
        // BRD4187: 21 of 105 ACKs in time during an interview). Only task
        // code logs; a line logged while the UART is borrowed is dropped.
        let Some(mut uart) = cortex_m::interrupt::free(|cs| LOG_UART.borrow(cs).take()) else {
            return;
        };
        let _ = writeln!(
            uart,
            "{:>8} {:5} {}",
            Instant::now().as_millis(),
            record.level(),
            record.args()
        );
        cortex_m::interrupt::free(|cs| LOG_UART.borrow(cs).replace(Some(uart)));
    }
    fn flush(&self) {}
}

static LOGGER: UartLogger = UartLogger;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    product::fatal::reset(Fatal::Panic)
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    product::fatal::reset(Fatal::HardFault)
}

fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Keep the Gecko ApplicationProperties record in the image.
    let _ =
        unsafe { core::ptr::read_volatile(&product::gecko::APPLICATION_PROPERTIES.struct_version) };
    let (cause, raw) = efr32s2_hal::reset::take_reset_cause();
    let previous = product::fatal::take_previous(cause);
    let BoardResources {
        clocks,
        host_uart,
        radio,
        rtcc,
        wdog: _,
        flash,
        ..
    } = BoardResources::take().unwrap_or_else(|| halt());
    if product::board::init_clocks(clocks).is_err() {
        halt()
    }
    cortex_m::interrupt::free(|cs| {
        LOG_UART
            .borrow(cs)
            .replace(Some(host_uart.into_uart_tx(LOG_BAUD)))
    });
    // SAFETY: called once before anything logs.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[MG24] SYSRTC start failed: {error:?}");
        halt()
    });
    efr32s2_hal::time_driver::init(rtcc);
    log::info!(
        "[MG24] {} on {} ({}), CTUNE {}, TX {} dBm, reset {cause:?} ({raw:#x})",
        product::MODEL,
        product::BOARD_NAME,
        product::board::MCU_PART,
        product::board::hfxo_ctune(),
        product::TX_POWER_DBM
    );
    if let Some(fatal) = previous {
        log::warn!("[MG24] previous reset: fatal {fatal:?}");
    }
    let capacity = efr32s2_hal::flash::device_flash_capacity();
    if capacity != product::layout::FLASH_SIZE as usize {
        log::error!("[MG24] DEVINFO flash size {capacity} B, product needs 1536 KiB");
        halt()
    }

    let app = compose(radio, flash);
    if let Err(error) = app.initialize().await {
        fatal(app.parts_mut(), error);
    }
    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    loop {
        // No button: re-arm commissioning automatically after a Leave.
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= RECOMMISSION_AFTER_LEAVE_MS {
                app.request_commissioning();
                awaiting_since = None;
            }
        } else {
            awaiting_since = None;
        }
        if let Err(error) = app.step().await {
            fatal(app.parts_mut(), error);
        }
        if let Err(fault) = product::fatal::check_platform() {
            platform_fatal(app.parts_mut(), fault);
        }
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}

/// Build MAC, device, stores and app into static storage.
#[inline(never)]
fn compose(
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> &'static mut App {
    // Fails with `Unsupported` where the xG24 tables are not validated: the
    // 10 dBm PA (RF-BM-MG24B1) and the 38.4 MHz crystal (Dongle-M).
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[MG24] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_parent_mac()
                .map_err(|error| log::error!("[MG24] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| halt());
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[MG24] no valid DEVINFO EUI-64");
        halt()
    }
    log::info!("[MG24] EUI-64 {:02X?}", ieee);

    static SECURITY: StaticCell<stores::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<RouterProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();
    static GP_STORE: StaticCell<stores::GreenPowerStore> = StaticCell::new();

    let profile = PROFILE.init(product::router::router_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_router_into(DEVICE.uninit());

    let (stores, opened) = stores::open(flash, product::ROLE_TAG).unwrap_or_else(|error| {
        log::error!("[MG24] journals failed to open: {error:?}");
        halt()
    });
    match opened {
        Opened::Existing => log::info!("[MG24] journals preserved"),
        Opened::Fresh => log::warn!("[MG24] blank/foreign NV erased; fresh join required"),
        Opened::RoleChanged => log::warn!("[MG24] journal of another role: keys cleared"),
    }
    let stores::Stores {
        security,
        children,
        green_power,
        aps_tables,
    } = stores;
    let security = SECURITY.init(security);
    match device.reset_security_state_if_identity_changed(security, ieee) {
        Ok(true) => log::warn!("[MG24] IEEE changed: network state cleared"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[MG24] identity guard failed: {error:?}");
            halt()
        }
    }
    let node = ZigbeeNode::new(device, security, profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(children),
        PersistentApsTables::new(aps_tables),
        &ROUTER_POLICY,
        RouterParts::new(LogStatus::default(), ResetSupervisor, LogDiagnostics),
    )
    .unwrap_or_else(|error| {
        log::error!("[MG24] invalid composition: {error:?}");
        halt()
    });
    app.set_green_power_store(GP_STORE.init(green_power));
    APP.init(app)
}

#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let phy = nwk.mac().phy();
    let radio = phy.radio_stats();
    let ack = phy.ack_stats();
    log::info!(
        "[MG24] neighbours {} children {} routes {} rx {} rx-ovf {} tx {} tx-fail {} acks {} late {} \
         queue-full {} ({} unACKed) ack-fail {} max-turnaround {} us",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        radio.rx_frames,
        radio.rx_overflows,
        radio.tx_frames,
        radio.tx_failures,
        ack.acks,
        ack.late,
        ack.queue_full,
        ack.queue_full_unacked,
        ack.ack_tx_fail,
        ack.max_turnaround_us
    );
}

#[derive(Default)]
struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            log::info!("[MG24] status {status:?}");
        }
    }
}

struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[MG24] router online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[MG24] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                short_address,
                ..
            }) => log::info!("[MG24] device announce 0x{short_address:04X}"),
            DiagnosticEvent::Fatal(error) => log::error!("[MG24] fatal: {error:?}"),
            other => log::info!("[MG24] {other:?}"),
        }
    }
}

struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    /// Only called after an unrecoverable router-app error.
    fn reset(&mut self) -> ! {
        product::fatal::reset(Fatal::Application)
    }
}

fn fatal(parts: &mut Parts, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault {
        archetype: ARCHETYPE,
    });
    parts.supervisor.reset()
}

/// A product fatal fault: log it, let the UART finish the line, then reset
/// with the cause recorded for the next boot.
fn platform_fatal(parts: &mut Parts, fault: Fatal) -> ! {
    log::error!("[MG24] fatal: {fault:?}; controlled reset");
    parts.status.set(RouterStatus::Fault {
        archetype: ARCHETYPE,
    });
    cortex_m::interrupt::free(|cs| {
        if let Some(uart) = LOG_UART.borrow(cs).borrow_mut().as_mut() {
            uart.flush();
        }
    });
    product::fatal::reset(fault)
}

const _: () = assert!(EFR32S2_ROUTER_PENDING_RX >= 4);
