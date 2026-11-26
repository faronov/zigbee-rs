//! zigbee-rs ZRHP v1 network co-processor for EFR32MG24 boards
//! (composition root): the RF-BM-MG24B1 dev board (default) or the SONOFF
//! Dongle-M (`--features sonoff-dongle-m`).
//!
//! Brings up the HFXO, the SYSRTC time base and the host USART
//! (`products/efr32mg24` transport policy), the radio as a parent MAC
//! with PAN-coordinator support, the product's NV journals and the Trust
//! Center coordinator with the Green Power combo, then runs the NCP loop:
//! service the host link, run queued stack operations, step the stack once
//! the host has started it (`NWK_INIT`/`NWK_FORM`).
//!
//! The UART carries only ZRHP: logs go to the ZRHP LOG endpoint. A panic,
//! a HardFault or a poisoned Secure Engine (the TRNG, the NCP's only
//! entropy source) ends in a reset that records its cause, so the next
//! `EVT_NCP_READY` reports `reset_reason` 6 (fatal) rather than 2, the
//! answer to a host `SYS_RESET` (`product::fatal`). The poisoned engine is
//! also logged, with a bounded window for the host to read it.
//! Firmware updates go through the resident Gecko bootloader: `SYS_RESET`
//! mode 1 reboots into it (GBL over XMODEM at 115 200); `FW_*` is not
//! built in.
//!
//! Status: builds and links; no MG24 has run it. The RF-BM-MG24B1 image
//! uses the 10 dBm PA tables verified on an EK2703A; on the Dongle-M (38.4
//! MHz crystal, PHY not verified) the radio init returns `Unsupported` and
//! the image halts before the NCP starts (see
//! `efr32s2-hal/src/radio/xg24.rs`).

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
// The NCP service and Trust Center step futures nest deeply.
#![recursion_limit = "256"]

use efr32mg24_product as product;
use efr32s2_hal::reset::take_reset_cause;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use ncp_coordinator_app::log_ring::level_code;
use ncp_coordinator_app::platform::NoPlatform;
use ncp_coordinator_app::tc::TcPort;
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity, PlatformRequest};
use product::board::resources::BoardResources;
use product::fatal::Fatal;
use product::stores::{self, Opened};
use product::transport::Host;
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentApsTables, PersistentChildren, RouterParts,
    TrustCenterCoordinatorApp,
};
use static_cell::StaticCell;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::efr32s2::{Efr32s2RadioPhy, Efr32s2RouterMac};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Mac = Efr32s2RouterMac<{ product::TX_POWER_DBM }>;
type Profile = DeviceProfile<RangeExtender>;
type App = TrustCenterCoordinatorApp<
    'static,
    Mac,
    stores::SecurityStore,
    Profile,
    stores::ChildStore,
    stores::TrustCenterStore,
    NoStatus,
    NoSupervisor,
    NoDiagnostics,
    router_app::NoObserver,
    PersistentApsTables<stores::ApsTableStore>,
>;
/// The app lives in its own static storage, constructed in place; the port
/// only borrows it.
type Port = TcPort<&'static mut App>;

/// ZRHP build record: `tools/efr32-gbl` patches the build ID into the
/// packaged image and its GBL metadata (spec 7.9, Gecko bootloader path).
#[used]
#[unsafe(link_section = ".rodata.zrhp_build_info")]
static BUILD_INFO: product::gecko::BuildInfo =
    product::gecko::BuildInfo::new(env!("CARGO_PKG_VERSION"));

const LOG_RECORDS: usize = 16;
static LOG: LogRing<LOG_RECORDS> = LogRing::new();
/// How long a fatal fault is served to the host before the reset.
const FATAL_REPORT_MS: u64 = 100;

struct RingLogger;

impl log::Log for RingLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        level_code(metadata.level()) <= LOG.level()
    }
    fn log(&self, record: &log::Record<'_>) {
        LOG.push(clock_ms(), level_code(record.level()), *record.args());
    }
    fn flush(&self) {}
}

static LOGGER: RingLogger = RingLogger;

fn clock_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// Stop: interrupts masked, the host sees no further frames. Used only for
/// boot failures the NCP cannot report (the LOG endpoint needs a session).
fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    product::fatal::reset(Fatal::Panic)
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    product::fatal::reset(Fatal::HardFault)
}

/// Resources after clocks and time are running.
struct Started {
    host: Host,
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
}

/// HFXO, SYSRTC time driver and the host UART.
#[inline(never)]
fn start() -> Started {
    let BoardResources {
        clocks,
        host_uart,
        radio,
        rtcc,
        wdog: _,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());
    // SAFETY: called once before anything logs.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
    if product::board::init_clocks(clocks).is_err() {
        halt()
    }
    // The TIMER0 time base must run before anything reads it (the session
    // cookie below); an unclocked TIMER0 read bus-faults (hardware-observed
    // on the BRD4187A: HardFault, fatal reset, boot loop).
    efr32s2_hal::delay::enable_cycle_counter();
    let rtcc = rtcc.start().unwrap_or_else(|_| halt());
    efr32s2_hal::time_driver::init(rtcc);
    // The journals sit at the top of 1536 KiB: refuse a smaller part.
    if efr32s2_hal::flash::device_flash_capacity() != product::layout::FLASH_SIZE as usize {
        halt()
    }
    let uart = host_uart
        .open(product::transport::UART_DEFAULT_BAUD)
        .unwrap_or_else(|_| halt());
    Started {
        host: Host::new(uart),
        radio,
        flash,
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Keep the Gecko ApplicationProperties record in the image.
    let _ =
        unsafe { core::ptr::read_volatile(&product::gecko::APPLICATION_PROPERTIES.struct_version) };
    let (cause, _raw) = take_reset_cause();
    let previous = product::fatal::take_previous(cause);
    let Started { host, radio, flash } = start();
    static HOST: StaticCell<Host> = StaticCell::new();
    let host = HOST.init(host);
    if let Some(fatal) = previous {
        log::error!("previous reset: fatal {:?}", fatal);
    }

    let ieee_factory = efr32s2_hal::devinfo::eui64();
    // Session cookie: not from the SE TRNG (unproven on hardware, and a
    // boot-time failure would leave the NCP without a session), so it
    // mixes the EUI-64 with the HFXO/LFRCO start-up timing. It only has to
    // differ between boots of this NCP (spec 5.3), not be secret.
    let cookie = {
        let low = u32::from_le_bytes([
            ieee_factory[0],
            ieee_factory[1],
            ieee_factory[2],
            ieee_factory[3],
        ]);
        let mixed = low ^ efr32s2_hal::delay::cycles() ^ efr32s2_hal::rtcc::now().rotate_left(16);
        if mixed == 0 { 1 } else { mixed }
    };
    let (port, opened) = compose(radio, flash);
    match opened {
        Opened::Existing => log::info!("NV journals preserved"),
        Opened::Fresh => log::warn!("blank/foreign NV erased"),
        Opened::RoleChanged => log::warn!("journal of another role: keys cleared"),
    }
    let ncp_identity = NcpIdentity {
        fw_version: env!("CARGO_PKG_VERSION"),
        build_id: BUILD_INFO.build_id(),
        chip: product::CHIP,
        chip_rev: 0,
        flash_size: product::layout::FLASH_SIZE,
        board: product::BOARD_NAME,
        product: product::PRODUCT_NAME,
        reset_reason: product::fatal::zrhp_reset_reason(cause, previous),
        key_export: true,
        max_children: zigbee_runtime::child_store::MAX_PERSISTED_CHILDREN as u32,
        max_tc_devices: zigbee_bdb::trust_center::MAX_TRUST_CENTER_DEVICES as u32,
        stack_allowed: true,
    };
    log::info!(
        "{} {} on {} ({:?})",
        product::MODEL,
        ncp_identity.fw_version,
        product::BOARD_NAME,
        cause
    );
    static NCP: StaticCell<Ncp<LOG_RECORDS>> = StaticCell::new();
    static PORT: StaticCell<Port> = StaticCell::new();
    let ncp = new_ncp(NCP.uninit(), ncp_identity, cookie);
    run(ncp, host, PORT.init(port)).await
}

/// Radio, stores, device and Trust Center coordinator in static storage.
#[inline(never)]
fn compose(
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> (Port, Opened) {
    static PROFILE: StaticCell<Profile> = StaticCell::new();
    let profile = PROFILE.init(DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    ));
    let (device, ieee) = compose_device(radio, profile);
    let (stores, opened) = stores::open(flash, product::ROLE_TAG).unwrap_or_else(|_| halt());
    let stores::Stores {
        security,
        trust_center,
        children,
        green_power,
        aps_tables,
    } = stores;
    static SECURITY: StaticCell<stores::SecurityStore> = StaticCell::new();
    let security = SECURITY.init(security);
    // Persisted keys/counters may never be resumed under another EUI-64.
    if device
        .reset_security_state_if_identity_changed(security, ieee)
        .is_err()
    {
        halt()
    }
    let app = compose_app(
        ZigbeeNode::new(device, security, profile),
        children,
        aps_tables,
        trust_center,
    );
    static GP_STORE: StaticCell<stores::GreenPowerStore> = StaticCell::new();
    app.set_green_power_store(GP_STORE.init(green_power));
    (TcPort::new(app, ieee, clock_ms), opened)
}

/// The coordinator device, built in place around the parent MAC, and its
/// IEEE address.
#[inline(never)]
fn compose_device(
    radio: efr32s2_hal::radio::RadioToken,
    profile: &Profile,
) -> (&'static mut ZigbeeDevice<Mac, Router>, [u8; 8]) {
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    // Fails with `Unsupported` on the Dongle-M (38.4 MHz crystal).
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .ok()
        .and_then(|phy| phy.into_parent_mac().ok())
        .unwrap_or_else(|| halt());
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        halt()
    }
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
        .build_coordinator_into(DEVICE.uninit());
    (device, ieee)
}

/// The Trust Center coordinator app, constructed in its static storage.
#[inline(never)]
fn compose_app(
    node: ZigbeeNode<'static, Mac, stores::SecurityStore, Profile, Router>,
    children: stores::ChildStore,
    aps_tables: stores::ApsTableStore,
    trust_center: stores::TrustCenterStore,
) -> &'static mut App {
    static APP: StaticCell<App> = StaticCell::new();
    TrustCenterCoordinatorApp::new_with_aps_tables_into(
        APP.uninit(),
        node,
        PersistentChildren::new(children),
        PersistentApsTables::new(aps_tables),
        trust_center,
        &product::POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap_or_else(|_| halt())
}

/// The NCP in static storage (its own frame).
#[inline(never)]
fn new_ncp(
    slot: &'static mut core::mem::MaybeUninit<Ncp<LOG_RECORDS>>,
    identity: NcpIdentity,
    cookie: u32,
) -> &'static mut Ncp<LOG_RECORDS> {
    Ncp::new_into(slot, identity, cookie, now_us())
}

async fn run(ncp: &mut Ncp<LOG_RECORDS>, host: &mut Host, port: &mut Port) -> ! {
    let mut platform = NoPlatform;
    loop {
        // After every stack step: a poisoned Secure Engine never recovers
        // without a reset, and the NCP cannot form or key a network
        // without it.
        if let Err(fatal) = product::fatal::check_platform() {
            fatal_reset(ncp, host, port, fatal)
        }
        if let Some(PlatformRequest::Reset { mode }) =
            ncp.service_with(now_us(), host, port, &mut platform, &LOG)
        {
            // The response went out with the request; let it drain.
            let drained = Instant::now();
            while !host.tx_idle() && drained.elapsed().as_millis() < 50 {}
            if mode == 1 {
                // Gecko bootloader (GBL over XMODEM at 115 200).
                efr32s2_hal::gecko_bootloader::reboot_to_bootloader();
            }
            efr32s2_hal::reset::system_reset();
        }
        while ncp.drive_with(port, &mut platform, now_us).await {
            ncp.service_with(now_us(), host, port, &mut platform, &LOG);
        }
        let dropped = zigbee_runtime::host_passthrough::take_dropped();
        if dropped != 0 {
            ncp.note_indications_dropped(dropped);
        }
        if port.running() {
            // Bounded by the 2 ms receive slice of `product::POLICY`.
            if let Err(error) = port.step().await {
                log::error!("stack step failed: {:?}", error);
            }
        } else {
            // The USART receive interrupt does not wake this task: poll the
            // ring at least every millisecond.
            let until = ncp
                .next_deadline()
                .map_or(1_000, |at| at.saturating_sub(now_us()));
            Timer::after_micros(until.clamp(100, 1_000)).await;
        }
    }
}

/// Report `fatal` on the LOG endpoint, serve the host link for a bounded
/// window so a connected host can read it, then record it and reset.
/// Requests that arrive in the window die with the reset.
#[inline(never)]
fn fatal_reset(ncp: &mut Ncp<LOG_RECORDS>, host: &mut Host, port: &mut Port, fatal: Fatal) -> ! {
    log::error!("fatal: {:?}; controlled reset", fatal);
    let mut platform = NoPlatform;
    let started = Instant::now();
    while started.elapsed().as_millis() < FATAL_REPORT_MS {
        let _ = ncp.service_with(now_us(), host, port, &mut platform, &LOG);
    }
    let drained = Instant::now();
    while !host.tx_idle() && drained.elapsed().as_millis() < 50 {}
    product::fatal::reset(fatal)
}
