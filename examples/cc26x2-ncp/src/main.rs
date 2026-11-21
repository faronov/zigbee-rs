//! zigbee-rs ZRHP v1 network co-processor for CC2652P7 USB dongles
//! (composition root).
//!
//! Brings up trim and clocks, the host UART (the dongle's USB-UART bridge,
//! `products/cc26x2-ncp` transport policy), the RF core as a parent MAC
//! with PAN-coordinator support, the product's NV journals and the Trust
//! Center coordinator with the Green Power combo, then runs the NCP loop:
//! service the host link, run queued stack operations, step the stack once
//! the host has started it (`NWK_INIT`/`NWK_FORM`).
//!
//! The UART carries only ZRHP: logs go to the ZRHP LOG endpoint. A panic
//! or a fault resets the chip. Firmware updates go through the serial ROM
//! bootloader (CCFG backdoor kept enabled); `FW_*` is not built in.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
// The NCP service and Trust Center step futures nest deeply.
#![recursion_limit = "256"]

use cc26x2_dongle::BoardResources;
use cc26x2_hal::chip::Identity;
use cc26x2_hal::reset::{ResetSource, system_reset};
use cc26x2_ncp_product as product;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use ncp_coordinator_app::log_ring::level_code;
use ncp_coordinator_app::platform::NoPlatform;
use ncp_coordinator_app::tc::TcPort;
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity, PlatformRequest};
use product::stores::{self, Opened};
use product::transport::Host;
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentApsTables, PersistentChildren, RouterParts,
    TrustCenterCoordinatorApp,
};
use static_cell::StaticCell;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::cc26x2::{Cc26x2RadioPhy, Cc26x2RouterMac};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use cc26x2_hal::vectors::__INTERRUPTS;

type Mac = Cc26x2RouterMac<{ product::TX_POWER_DBM }>;
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

const LOG_RECORDS: usize = 16;
static LOG: LogRing<LOG_RECORDS> = LogRing::new();

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
    system_reset()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    system_reset()
}

/// `EVT_NCP_READY.reset_reason` (spec 7.1) from `RESETCTL.RESET_SRC`. The
/// RESET pin of a USB dongle is the bridge's DTR/RTS circuit: a host
/// request.
fn reset_reason(source: ResetSource) -> u8 {
    match source {
        ResetSource::PowerOn => 1,
        ResetSource::SystemReset | ResetSource::WarmReset => 2,
        ResetSource::VddsLoss | ResetSource::VddrLoss => 4,
        ResetSource::ClockLoss => 6,
        ResetSource::Pin => 7,
    }
}

/// Resources after clocks and time are running.
struct Started {
    identity: Identity,
    host: Host,
    radio: cc26x2_dongle::BoardRadio,
    trng: cc26x2_hal::trng::TrngToken,
    flash: cc26x2_hal::flash::Flash,
}

/// Trim, time driver, XOSC_HF, LF clock and the host UART.
#[inline(never)]
fn start() -> Started {
    let trim = cc26x2_hal::setup::trim_device();
    let BoardResources {
        host_uart,
        radio,
        rtc,
        trng,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());
    // SAFETY: called once before anything logs.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
    cc26x2_hal::time_driver::init(rtc);
    let identity = Identity::read();
    if trim.is_err() || cc26x2_hal::setup::start_xosc_hf().is_err() {
        halt()
    }
    // XOSC_LF, or RCOSC_LF when the crystal does not qualify in time.
    match cc26x2_hal::setup::start_lf_clock(cc26x2_hal::setup::XOSC_LF_TIMEOUT_MS) {
        Ok(lf) => log::info!("LF clock {:?}", lf),
        Err(_) => halt(),
    }
    let uart = host_uart
        .open(product::transport::UART_DEFAULT_BAUD)
        .unwrap_or_else(|_| halt());
    Started {
        identity,
        host: Host::new(uart),
        radio,
        trng,
        flash,
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Keep the product CCFG in the image.
    let _ = unsafe { core::ptr::read_volatile(&product::ccfg::CCFG[0]) };
    let reset = ResetSource::read();
    let Started {
        identity,
        host,
        radio,
        trng,
        flash,
    } = start();
    static HOST: StaticCell<Host> = StaticCell::new();
    let host = HOST.init(host);

    let mut trng = cc26x2_hal::trng::Trng::new(trng).unwrap_or_else(|_| halt());
    let cookie = loop {
        let mut bytes = [0u8; 4];
        let _ = trng.fill(&mut bytes);
        let value = u32::from_le_bytes(bytes);
        if value != 0 {
            break value;
        }
    };
    let (port, opened) = compose(radio, trng, flash, identity.eui64);
    match opened {
        Opened::Existing => log::info!("NV journals preserved"),
        Opened::Fresh => log::warn!("blank/foreign NV erased"),
        Opened::RoleChanged => log::warn!("journal of another role: keys cleared"),
    }
    let ncp_identity = NcpIdentity {
        fw_version: env!("CARGO_PKG_VERSION"),
        build_id: [0; 8],
        chip: product::CHIP,
        chip_rev: u16::from(identity.pg_rev),
        flash_size: identity.flash_size as u32,
        board: product::BOARD_NAME,
        product: product::PRODUCT_NAME,
        reset_reason: reset_reason(reset),
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
        reset
    );
    static NCP: StaticCell<Ncp<LOG_RECORDS>> = StaticCell::new();
    static PORT: StaticCell<Port> = StaticCell::new();
    let ncp = new_ncp(NCP.uninit(), ncp_identity, cookie);
    run(ncp, host, PORT.init(port)).await
}

/// Radio, stores, device and Trust Center coordinator in static storage.
#[inline(never)]
fn compose(
    radio: cc26x2_dongle::BoardRadio,
    trng: cc26x2_hal::trng::Trng,
    flash: cc26x2_hal::flash::Flash,
    eui64: [u8; 8],
) -> (Port, Opened) {
    static PROFILE: StaticCell<Profile> = StaticCell::new();
    let profile = PROFILE.init(DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    ));
    let (device, ieee) = compose_device(radio, trng, eui64, profile);
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
    radio: cc26x2_dongle::BoardRadio,
    trng: cc26x2_hal::trng::Trng,
    eui64: [u8; 8],
    profile: &Profile,
) -> (&'static mut ZigbeeDevice<Mac, Router>, [u8; 8]) {
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    let radio = radio.init(product::TX_POWER_DBM).unwrap_or_else(|_| halt());
    let mac: Mac = Cc26x2RadioPhy::new(radio, trng, eui64)
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
        if let Some(PlatformRequest::Reset { .. }) =
            ncp.service_with(now_us(), host, port, &mut platform, &LOG)
        {
            // Mode 1 (ROM bootloader) is entered by the host through the
            // bridge's DTR/RTS lines (CCFG backdoor); both modes reboot.
            system_reset();
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
            // The UART receive interrupt does not wake this task: poll the
            // ring at least every millisecond.
            let until = ncp
                .next_deadline()
                .map_or(1_000, |at| at.saturating_sub(now_us()));
            Timer::after_micros(until.clamp(100, 1_000)).await;
        }
    }
}
