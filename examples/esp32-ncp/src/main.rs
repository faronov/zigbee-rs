//! zigbee-rs ZRHP v1 network co-processor (composition root).
//!
//! Brings up the clock, the host transport (USB-Serial-JTAG, or UART with
//! DMA receive on the Thread Border Router board), the radio as a parent
//! MAC, the product's NV journals and the Trust Center coordinator, then
//! runs the NCP loop: service the host link, run queued stack operations,
//! step the stack once the host has started it (`NWK_INIT`/`NWK_FORM`).
//!
//! The coordinator runs the Green Power combo (Proxy Basic, sink and
//! TempMaster; spec 7.11) with its tables in the `zbnv` GP journal; the
//! host selects the sink authority (`GP_SINK_CONFIG`).
//!
//! Logs go only to the ZRHP LOG endpoint: when USB-Serial-JTAG carries
//! ZRHP no console may share it (spec 11.2). A panic resets the chip.
//!
//! Before the stack is composed, the product platform reads the identity
//! item and runs the `otadata` boot check: a persisted IEEE override
//! (`SYS_SET_IEEE`, spec 9.4 rule 7) becomes `MacExtendedAddress` and the
//! active IEEE everywhere; an image on trial (`FW_ACTIVATE`, spec 7.9) must
//! get `FW_CONFIRM` within the trial window or is rolled back.

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_time::{Duration, Instant, Timer};
use esp32_zigbee_devkit_product::executor::block_on;
use esp32_zigbee_ncp_product as product;
use ncp_coordinator_app::log_ring::level_code;
use ncp_coordinator_app::tc::TcPort;
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity, PlatformRequest};
use product::flash_size::FlashVerdict;
use product::platform::{BootOutcome, EspPlatform, TrialPoll};
use product::stores;
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentApsTables, PersistentChildren, RouterParts,
    TrustCenterCoordinatorApp,
};
#[cfg(feature = "transport-uart")]
use static_cell::ConstStaticCell;
use static_cell::StaticCell;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::esp::EspRouterMac;
use zigbee_mac::{PibAttribute, PibValue};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = EspRouterMac<'static>;
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
/// only borrows it (no app-sized value ever crosses the stack).
type Port = TcPort<&'static mut App>;

const LOG_RECORDS: usize = 16;
static LOG: LogRing<LOG_RECORDS> = LogRing::new();

struct RingLogger;

impl log::Log for RingLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        level_code(metadata.level()) <= LOG.level()
    }
    fn log(&self, record: &log::Record) {
        LOG.push(clock_ms(), level_code(record.level()), *record.args());
    }
    fn flush(&self) {}
}

static LOGGER: RingLogger = RingLogger;

/// Version word `0xMMmmrrrr` of this image (`FW_*` downgrade policy and
/// `FW_GET_SLOTS`), from the same `CARGO_PKG_VERSION` the app descriptor
/// and `SYS_GET_INFO.fw_version` carry.
const FW_VERSION: u32 = product::fw::parse_version(env!("CARGO_PKG_VERSION").as_bytes());

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    lab::panic(info);
    esp_hal::system::software_reset()
}

fn clock_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn now_us() -> u64 {
    Instant::now().as_micros()
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// `EVT_NCP_READY.reset_reason` from the chip's reset cause.
fn reset_reason() -> u8 {
    use esp_hal::rtc_cntl::SocResetReason as R;
    match esp_hal::system::reset_reason() {
        Some(R::ChipPowerOn) => 1,
        Some(R::CoreSw | R::Cpu0Sw) => 2,
        // Debug-module (JTAG) and USB-Serial-JTAG DTR/RTS resets: host request.
        Some(R::CoreUsbUart | R::CoreUsbJtag) => 7,
        Some(
            R::CoreMwdt0
            | R::CoreMwdt1
            | R::CoreRtcWdt
            | R::Cpu0Mwdt0
            | R::Cpu0Mwdt1
            | R::Cpu0RtcWdt
            | R::SysRtcWdt
            | R::SysSuperWdt,
        ) => 3,
        Some(R::SysBrownOut) => 4,
        _ => 0,
    }
}

mod lab;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    lab::boot();
    // zigbee-mac's esp-radio driver allocates its receive queue. The heap
    // lives in the RAM the 2nd-stage bootloader used (`dram2_seg`: H2
    // 0x4083EFD0..0x4084FEE0, C6 0x4086E610..0x4087E610), free once the app
    // runs, so the main stack keeps the room the GP combo's tables took
    // (`tools/esp32-ncp-stack-gate.py`). Not yet booted on hardware.
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 32768);
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);
    esp32_zigbee_devkit_product::time_driver::init(peripherals.SYSTIMER);

    // Identity item and otadata boot check, before the radio and the
    // stack exist (the check may program otadata).
    static PLATFORM: StaticCell<EspPlatform> = StaticCell::new();
    let (platform, boot) = boot_platform();
    // The first boot of a new image reports `reset_reason` 5.
    let trial = matches!(boot, BootOutcome::Trial { .. });
    match boot {
        BootOutcome::RolledBack { slot } => {
            log::error!("unconfirmed image rolled back to slot {}", slot);
            esp_hal::system::software_reset()
        }
        BootOutcome::Trial { slot } => log::warn!("slot {} on trial: FW_CONFIRM required", slot),
        _ => {}
    }
    let platform = PLATFORM.init(platform);

    let verdict =
        FlashVerdict::from_jedec(product::flash::read_jedec_id(), &product::layout::ACTIVE);
    let rng = esp_hal::rng::Rng::new();
    let cookie = loop {
        let value = rng.random();
        if value != 0 {
            break value;
        }
    };

    #[cfg(feature = "transport-usj")]
    let host = usj_host(peripherals.USB_DEVICE);
    #[cfg(feature = "transport-uart")]
    let host = uart_host(
        peripherals.UART0,
        peripherals.GPIO24,
        peripherals.GPIO23,
        #[cfg(feature = "rtscts-reserved-lines")]
        peripherals.GPIO4,
        #[cfg(feature = "rtscts-reserved-lines")]
        peripherals.GPIO5,
        peripherals.UHCI0,
        peripherals.DMA_CH0,
    );

    let mut factory_ieee = [0; 8];
    let port = compose(
        peripherals.IEEE802154,
        peripherals.AES,
        platform.boot_ieee_override(),
        &mut factory_ieee,
    );
    platform.set_factory_ieee(factory_ieee);
    platform.attach_mac_counters(mac_counters);
    #[cfg(feature = "pta")]
    attach_pta(
        platform,
        peripherals.GPIO0,
        peripherals.GPIO1,
        peripherals.GPIO2,
        peripherals.GPIO3,
    );
    let identity = NcpIdentity {
        fw_version: env!("CARGO_PKG_VERSION"),
        build_id: BUILD_ID,
        chip: product::CHIP,
        chip_rev: 0,
        flash_size: verdict.reported_size(),
        board: product::BOARD_NAME,
        product: product::PRODUCT_NAME,
        reset_reason: if trial { 5 } else { reset_reason() },
        key_export: true,
        max_children: zigbee_runtime::child_store::MAX_PERSISTED_CHILDREN as u32,
        max_tc_devices: zigbee_bdb::trust_center::MAX_TRUST_CENTER_DEVICES as u32,
        stack_allowed: verdict.stack_allowed(),
    };
    log::info!(
        "{} {} on {}",
        product::MODEL,
        identity.fw_version,
        product::BOARD_NAME
    );
    if !verdict.stack_allowed() {
        log::error!(
            "partition table larger than the fitted flash: {:?}",
            verdict
        );
    }
    static NCP: StaticCell<Ncp<LOG_RECORDS>> = StaticCell::new();
    static PORT: StaticCell<Port> = StaticCell::new();
    let ncp = new_ncp(NCP.uninit(), identity, cookie);
    lab::stage(9);
    serve(ncp, host, PORT.init(port), platform)
}

/// Run the NCP loop. A separate frame: the root future (moved into the
/// executor) is not part of `main`'s frame under the composition steps.
#[inline(never)]
fn serve(
    ncp: &'static mut Ncp<LOG_RECORDS>,
    host: &'static mut product::host::Host,
    port: &'static mut Port,
    platform: &'static mut EspPlatform,
) -> ! {
    block_on(run(ncp, host, port, platform))
}

/// The USB-Serial-JTAG host link in static storage (its own frame).
#[cfg(feature = "transport-usj")]
#[inline(never)]
fn usj_host(usb: esp_hal::peripherals::USB_DEVICE<'static>) -> &'static mut product::host::Host {
    static HOST: StaticCell<product::host::Host> = StaticCell::new();
    HOST.init(product::host::Host::new(usb).unwrap_or_else(|| halt()))
}

/// The UART host link with DMA receive in static storage (its own frame;
/// the DMA memory is a zero-initialized static, never a stack value).
#[cfg(feature = "transport-uart")]
#[inline(never)]
fn uart_host(
    uart0: esp_hal::peripherals::UART0<'static>,
    tx: esp_hal::peripherals::GPIO24<'static>,
    rx: esp_hal::peripherals::GPIO23<'static>,
    #[cfg(feature = "rtscts-reserved-lines")] rts: esp_hal::peripherals::GPIO4<'static>,
    #[cfg(feature = "rtscts-reserved-lines")] cts: esp_hal::peripherals::GPIO5<'static>,
    uhci: esp_hal::peripherals::UHCI0<'static>,
    dma: esp_hal::peripherals::DMA_CH0<'static>,
) -> &'static mut product::host::Host {
    use esp_serial_stream::uart::{LineConfig, UartDmaMemory, UartDmaStream, UartPort};
    let line = LineConfig {
        baud: product::transport::UART_DEFAULT_BAUD,
        flow_control: product::transport::UART_DEFAULT_FLOW,
    };
    let config = esp_hal::uart::Config::default().with_baudrate(line.baud);
    let uart = esp_thread_br_h2::thread_br_h2_uart(
        uart0,
        tx,
        rx,
        #[cfg(feature = "rtscts-reserved-lines")]
        rts,
        #[cfg(feature = "rtscts-reserved-lines")]
        cts,
        config,
    )
    .unwrap_or_else(|_| halt());
    static MEMORY: ConstStaticCell<UartDmaMemory> = ConstStaticCell::new(UartDmaMemory::new());
    let stream = UartDmaStream::new(uart, UartPort::Uart0, uhci, dma, 0, MEMORY.take(), line)
        .unwrap_or_else(|_| halt());
    static HOST: StaticCell<product::host::Host> = StaticCell::new();
    HOST.init(product::host::Host::new(stream))
}

/// Radio, stores, device and Trust Center coordinator in static storage.
/// `ieee_override` (the persisted identity item) replaces the factory
/// EUI-64 before anything reads the MAC's extended address.
///
/// Each large object is constructed in its own non-inlined step, directly
/// in static storage where its constructor allows it (the device, the app),
/// so no step's temporaries stack on another's and no app-sized value ever
/// crosses the stack (boot stack budget, see `tools/esp32-ncp-stack-gate.py`).
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    ieee_override: Option<[u8; 8]>,
    factory_ieee: &mut [u8; 8],
) -> Port {
    static PROFILE: StaticCell<Profile> = StaticCell::new();
    let profile = PROFILE.init(DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    ));
    let (device, ieee) = compose_device(radio, aes, ieee_override, profile, factory_ieee);
    let security = compose_security(device, ieee);
    let app = compose_app(ZigbeeNode::new(device, security, profile));
    // GP combo: Proxy and Sink Tables in the `zbnv` GP journal.
    static GP_STORE: StaticCell<stores::GreenPowerStore> = StaticCell::new();
    app.set_green_power_store(GP_STORE.init(stores::green_power_store()));
    let mut port = TcPort::new(app, ieee, clock_ms);
    port.set_radio_idle_check(radio_idle);
    port
}

/// The coordinator device, built in place around the parent MAC, and the
/// active IEEE.
#[inline(never)]
fn compose_device(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    ieee_override: Option<[u8; 8]>,
    profile: &Profile,
    factory_ieee: &mut [u8; 8],
) -> (&'static mut ZigbeeDevice<Mac, Router>, [u8; 8]) {
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    let mut ieee = [0; 8];
    let device = device_builder(radio, aes, ieee_override, profile, &mut ieee, factory_ieee)
        .build_coordinator_into(DEVICE.uninit());
    (device, ieee)
}

/// The radio as a parent MAC with the AES engine and the IEEE override.
/// A separate frame: the PHY and MAC temporaries of the conversion are
/// released before the device is assembled.
#[inline(never)]
fn radio_mac(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    ieee_override: Option<[u8; 8]>,
    factory_ieee: &mut [u8; 8],
) -> Mac {
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(radio);
    let mut phy =
        zigbee_mac::esp::EspRadioPhy::new(ieee802154, esp_radio::ieee802154::Config::default());
    log::info!(
        "[RADIO] RSSI compensation {} dB",
        phy.rssi_compensation_db()
    );
    if phy.install_aes_engine(aes).is_err() {
        halt()
    }
    let Ok(mut mac) = phy.into_parent_mac() else {
        halt()
    };
    // The eFuse EUI-64, before the override replaces it.
    *factory_ieee = mac.pib().extended_address();
    if let Some(ieee) = ieee_override
        && mac
            .set_pib(
                PibAttribute::MacExtendedAddress,
                PibValue::ExtendedAddress(ieee),
            )
            .is_err()
    {
        halt()
    }
    mac
}

/// The configured device builder around the parent MAC; `ieee` receives the
/// active IEEE. A separate frame: the MAC and the by-value builder chain's
/// temporaries are released before the device is assembled.
#[inline(never)]
fn device_builder(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    ieee_override: Option<[u8; 8]>,
    profile: &Profile,
    ieee: &mut [u8; 8],
    factory_ieee: &mut [u8; 8],
) -> zigbee_runtime::builder::DeviceBuilder<Mac> {
    let mac = radio_mac(radio, aes, ieee_override, factory_ieee);
    // The active IEEE: the override, else the factory eFuse EUI-64.
    *ieee = mac.pib().extended_address();
    if *ieee == [0; 8] || *ieee == [0xFF; 8] {
        halt()
    }
    ZigbeeDevice::builder(mac)
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
}

/// The security journal, guarded against another EUI-64 or role image.
#[inline(never)]
fn compose_security(
    device: &mut ZigbeeDevice<Mac, Router>,
    ieee: [u8; 8],
) -> &'static mut stores::SecurityStore {
    static SECURITY: StaticCell<stores::SecurityStore> = StaticCell::new();
    let security = SECURITY.init(stores::security_store());
    // Persisted keys/counters may never be resumed under another EUI-64 or
    // by another role image.
    if device
        .reset_security_state_if_identity_changed(security, ieee)
        .is_err()
        || security.select_role_tag(product::ROLE_TAG).is_err()
    {
        log::error!("security journal guard failed");
        halt()
    }
    security
}

/// The Trust Center coordinator app, constructed in its static storage.
#[inline(never)]
fn compose_app(
    node: ZigbeeNode<'static, Mac, stores::SecurityStore, Profile, Router>,
) -> &'static mut App {
    static APP: StaticCell<App> = StaticCell::new();
    TrustCenterCoordinatorApp::new_with_aps_tables_into(
        APP.uninit(),
        node,
        PersistentChildren::new(stores::child_store()),
        PersistentApsTables::new(stores::aps_table_store()),
        stores::trust_center_store(),
        &product::POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap_or_else(|_| halt())
}

async fn run(
    ncp: &mut Ncp<LOG_RECORDS>,
    host: &mut product::host::Host,
    port: &mut Port,
    platform: &mut EspPlatform,
) -> ! {
    loop {
        lab::tick(host);
        radio_watch(now_us());
        radio_stats(now_us());
        #[cfg(feature = "transport-uart")]
        uart_stats(now_us(), host);
        // Trial window without FW_CONFIRM: the previous image is selected
        // (or the next boot's check selects it); restart into it.
        if let TrialPoll::Expired { .. } = platform.poll_trial(now_ms()) {
            esp_hal::system::software_reset();
        }
        // PTA fails closed: a coexistence-library request the OS adapter
        // could not honour stops the arbiter until reboot (spec 11.5).
        #[cfg(feature = "pta")]
        platform.poll_coex();
        if let Some(PlatformRequest::Reset { .. }) =
            ncp.service_with(now_us(), host, port, platform, &LOG)
        {
            // Mode 1 (ROM download) is entered by the host through the USJ
            // DTR/RTS lines or the board's BOOT line; both modes reboot.
            esp_hal::system::software_reset();
        }
        while ncp.drive_with(port, platform, now_us).await {
            ncp.service_with(now_us(), host, port, platform, &LOG);
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
            let until = ncp
                .next_deadline()
                .map_or(1_000, |at| at.saturating_sub(now_us()));
            Nap::new(until.clamp(100, 1_000)).await;
        }
    }
}

/// Sleep until a timer or any interrupt (USB/UART receive) wakes the CPU.
struct Nap {
    timer: Timer,
    polled: bool,
}

impl Nap {
    fn new(us: u64) -> Self {
        Self {
            timer: Timer::after(Duration::from_micros(us)),
            polled: false,
        }
    }
}

impl Future for Nap {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.polled {
            return Poll::Ready(());
        }
        self.polled = true;
        match Pin::new(&mut self.timer).poll(cx) {
            Poll::Ready(()) => Poll::Ready(()),
            Poll::Pending => Poll::Pending,
        }
    }
}

/// Identity item and otadata boot check (its own frame: the FW engine and
/// the journals are large and must not be inlined into `main`).
#[inline(never)]
fn boot_platform() -> (EspPlatform, BootOutcome) {
    product::platform::boot(FW_VERSION, now_ms())
}

/// The NCP in static storage (its own frame, see `boot_platform`).
#[inline(never)]
fn new_ncp(
    slot: &'static mut core::mem::MaybeUninit<Ncp<LOG_RECORDS>>,
    identity: NcpIdentity,
    cookie: u32,
) -> &'static mut Ncp<LOG_RECORDS> {
    Ncp::new_into(slot, identity, cookie, now_us())
}

/// The Thread BR board's PTA lines, disabled until the host sets
/// `coex_enable` (spec 11.5).
#[cfg(feature = "pta")]
#[inline(never)]
fn attach_pta(
    platform: &mut EspPlatform,
    request: esp_hal::peripherals::GPIO0<'static>,
    grant: esp_hal::peripherals::GPIO1<'static>,
    priority: esp_hal::peripherals::GPIO2<'static>,
    tx_line: esp_hal::peripherals::GPIO3<'static>,
) {
    static PTA: StaticCell<product::pta::Pta<'static>> = StaticCell::new();
    platform.attach_coex(PTA.init(product::pta::Pta::new(request, grant, priority, tx_line)));
}

/// Reports a radio driver that stays out of "receiving, nothing pending"
/// (a stalled state machine refuses every transmission as busy) and the
/// stalled automatic ACKs the driver's watchdog left. State in statics, so
/// the root future does not carry it. Its own frame: the log formatting
/// stays out of the root future's poll frame (stack gate `serve` chain).
#[inline(never)]
/// The radio is receiving with no interrupt work pending: a journal
/// pre-erase (interrupts masked for ~30 ms) cannot cut an automatic ACK.
fn radio_idle() -> bool {
    let (state, events, _, _) = esp_radio::ieee802154::debug_state();
    state == 1 && events == 0
}

/// Parent radio counters once a minute when they changed (LOG endpoint):
/// children's Data Requests (polls), Frame Pending ACKs and transmissions.
/// Out of line: its formatting frame stays off the serve loop's stack.
#[inline(never)]
fn radio_stats(now_us: u64) {
    use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
    use portable_atomic::AtomicU64;
    const PERIOD_US: u64 = 60_000_000;
    static NEXT_US: AtomicU64 = AtomicU64::new(PERIOD_US);
    static LAST_POLLS: AtomicU32 = AtomicU32::new(0);
    static LAST_TX: AtomicU32 = AtomicU32::new(0);
    if now_us < NEXT_US.load(Relaxed) {
        return;
    }
    NEXT_US.store(now_us + PERIOD_US, Relaxed);
    let stats = zigbee_mac::esp::parent_stats();
    let polls = LAST_POLLS.swap(stats.data_requests, Relaxed) != stats.data_requests;
    let tx = LAST_TX.swap(stats.raw_tx, Relaxed) != stats.raw_tx;
    if polls || tx {
        log::info!(
            "[RADIO] data-req {} fp-acks {} tx {} acked-tx {} no-ack {} tx-err {}",
            stats.data_requests,
            stats.frame_pending_acks,
            stats.raw_tx,
            stats.acked_tx,
            stats.tx_no_ack,
            stats.tx_errors
        );
    }
}

/// `DIAG_GET_COUNTERS` MAC group (`product::radio::MacCounters` order).
fn mac_counters() -> product::radio::MacCounters {
    let stats = zigbee_mac::esp::parent_stats();
    [
        stats.raw_tx,
        stats.acked_tx,
        stats.tx_no_ack,
        stats.tx_errors,
        stats.tx_deferred,
        stats.data_requests,
        stats.frame_pending_acks,
        esp_radio::ieee802154::tx_ack_recoveries(),
    ]
}

/// Host UART counters once a minute when the receive side saw faults
/// (LOG endpoint): line errors (framing/parity/glitch) and overflows (FIFO
/// overflow or DMA restart: bytes lost). With RTS/CTS both stay 0.
#[cfg(feature = "transport-uart")]
#[inline(never)]
fn uart_stats(now_us: u64, host: &product::host::Host) {
    use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
    use portable_atomic::AtomicU64;
    const PERIOD_US: u64 = 60_000_000;
    static NEXT_US: AtomicU64 = AtomicU64::new(PERIOD_US);
    static LAST_FAULTS: AtomicU32 = AtomicU32::new(0);
    if now_us < NEXT_US.load(Relaxed) {
        return;
    }
    NEXT_US.store(now_us + PERIOD_US, Relaxed);
    let c = host.counters();
    let faults = c.rx_line_errors.wrapping_add(c.rx_overflow);
    if LAST_FAULTS.swap(faults, Relaxed) != faults {
        log::info!(
            "[UART] rx {} tx {} line-errors {} overflow {}",
            c.rx_bytes,
            c.tx_bytes,
            c.rx_line_errors,
            c.rx_overflow
        );
    }
}

#[inline(never)]
fn radio_watch(now_us: u64) {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};
    use portable_atomic::AtomicU64;
    const STALL_US: u64 = 200_000;
    static SINCE_US: AtomicU64 = AtomicU64::new(0);
    static REPORTED: AtomicBool = AtomicBool::new(false);
    static RECOVERIES: AtomicU32 = AtomicU32::new(0);
    let recoveries = esp_radio::ieee802154::tx_ack_recoveries();
    if recoveries != RECOVERIES.swap(recoveries, Relaxed) {
        log::warn!(
            "[RADIO] stalled automatic ACK left ({} since boot)",
            recoveries
        );
    }
    let state = esp_radio::ieee802154::debug_state();
    let (driver, events, _, _) = state;
    if driver == 1 && events == 0 {
        if REPORTED.swap(false, Relaxed) {
            log::warn!("[RADIO] receiving again: {:?}", state);
        }
        SINCE_US.store(0, Relaxed);
        return;
    }
    let since = match SINCE_US.load(Relaxed) {
        0 => {
            SINCE_US.store(now_us.max(1), Relaxed);
            now_us
        }
        since => since,
    };
    if !REPORTED.load(Relaxed) && now_us.saturating_sub(since) > STALL_US {
        REPORTED.store(true, Relaxed);
        log::error!(
            "[RADIO] stalled {} ms: (state, events, rx_state, queued) = {:?}",
            (now_us - since) / 1000,
            state
        );
    }
}

/// `SYS_GET_INFO.build_id`: the first 8 bytes of the source commit, which
/// the release workflow passes as `ZRHP_BUILD_ID` (16 hex digits); zero in
/// other builds.
const BUILD_ID: [u8; 8] = match option_env!("ZRHP_BUILD_ID") {
    Some(hex) => parse_build_id(hex.as_bytes()),
    None => [0; 8],
};

const fn parse_build_id(hex: &[u8]) -> [u8; 8] {
    const fn nibble(c: u8) -> u8 {
        match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => panic!("ZRHP_BUILD_ID: not a hex digit"),
        }
    }
    assert!(hex.len() == 16, "ZRHP_BUILD_ID: expected 16 hex digits");
    let mut id = [0; 8];
    let mut i = 0;
    while i < 8 {
        id[i] = nibble(hex[2 * i]) << 4 | nibble(hex[2 * i + 1]);
        i += 1;
    }
    id
}
