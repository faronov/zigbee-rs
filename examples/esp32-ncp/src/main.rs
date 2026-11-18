//! zigbee-rs ZRHP v1 network co-processor (composition root).
//!
//! Brings up the clock, the host transport (USB-Serial-JTAG, or UART with
//! DMA receive on the Thread Border Router board), the radio as a parent
//! MAC, the product's NV journals and the Trust Center coordinator, then
//! runs the NCP loop: service the host link, run queued stack operations,
//! step the stack once the host has started it (`NWK_INIT`/`NWK_FORM`).
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
fn panic(_: &core::panic::PanicInfo) -> ! {
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

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // zigbee-mac's esp-radio driver allocates its receive queue.
    esp_alloc::heap_allocator!(size: 32768);
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);
    esp32_zigbee_devkit_product::time_driver::init(peripherals.SYSTIMER);

    // Identity item and otadata boot check, before the radio and the
    // stack exist (the check may program otadata).
    static PLATFORM: StaticCell<EspPlatform> = StaticCell::new();
    let (platform, boot) = product::platform::boot(FW_VERSION, now_ms());
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
    let mut host = product::host::Host::new(peripherals.USB_DEVICE).unwrap_or_else(|| halt());
    #[cfg(feature = "transport-uart")]
    let mut host = uart_host(
        peripherals.UART0,
        peripherals.GPIO24,
        peripherals.GPIO23,
        peripherals.UHCI0,
        peripherals.DMA_CH0,
    );

    let port = compose(
        peripherals.IEEE802154,
        peripherals.AES,
        platform.boot_ieee_override(),
    );
    let identity = NcpIdentity {
        fw_version: env!("CARGO_PKG_VERSION"),
        build_id: [0; 8],
        chip: product::CHIP,
        chip_rev: 0,
        flash_size: verdict.reported_size(),
        board: product::BOARD_NAME,
        product: product::PRODUCT_NAME,
        reset_reason: reset_reason(),
        key_export: true,
        max_children: zigbee_runtime::child_store::MAX_PERSISTED_CHILDREN as u32,
        max_tc_devices: 32,
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
    static PORT: StaticCell<TcPort<App>> = StaticCell::new();
    let ncp = NCP.init(Ncp::new(identity, cookie, now_us()));
    block_on(run(ncp, &mut host, PORT.init(port), platform))
}

#[cfg(feature = "transport-uart")]
fn uart_host(
    uart0: esp_hal::peripherals::UART0<'static>,
    tx: esp_hal::peripherals::GPIO24<'static>,
    rx: esp_hal::peripherals::GPIO23<'static>,
    uhci: esp_hal::peripherals::UHCI0<'static>,
    dma: esp_hal::peripherals::DMA_CH0<'static>,
) -> product::host::Host {
    use esp_serial_stream::uart::{LineConfig, UartDmaMemory, UartDmaStream, UartPort};
    let line = LineConfig {
        baud: product::transport::UART_DEFAULT_BAUD,
        flow_control: product::transport::UART_DEFAULT_FLOW,
    };
    let config = esp_hal::uart::Config::default().with_baudrate(line.baud);
    let uart =
        esp_thread_br_h2::thread_br_h2_uart(uart0, tx, rx, config).unwrap_or_else(|_| halt());
    static MEMORY: StaticCell<UartDmaMemory> = StaticCell::new();
    let stream = UartDmaStream::new(
        uart,
        UartPort::Uart0,
        uhci,
        dma,
        MEMORY.init(UartDmaMemory::new()),
        line,
    )
    .unwrap_or_else(|_| halt());
    product::host::Host::new(stream)
}

/// Radio, stores, device and Trust Center coordinator in static storage.
/// `ieee_override` (the persisted identity item) replaces the factory
/// EUI-64 before anything reads the MAC's extended address.
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    ieee_override: Option<[u8; 8]>,
) -> TcPort<App> {
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(radio);
    let mut phy =
        zigbee_mac::esp::EspRadioPhy::new(ieee802154, esp_radio::ieee802154::Config::default());
    if phy.install_aes_engine(aes).is_err() {
        halt()
    }
    let Ok(mut mac) = phy.into_parent_mac() else {
        halt()
    };
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
    // The active IEEE: the override, else the factory eFuse EUI-64.
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        halt()
    }

    static SECURITY: StaticCell<stores::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<Profile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();

    let profile = PROFILE.init(DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    ));
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
    let node = ZigbeeNode::new(device, security, profile);
    let app = TrustCenterCoordinatorApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores::child_store()),
        PersistentApsTables::new(stores::aps_table_store()),
        stores::trust_center_store(),
        &product::POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap_or_else(|_| halt());
    TcPort::new(app, ieee, clock_ms)
}

async fn run(
    ncp: &mut Ncp<LOG_RECORDS>,
    host: &mut product::host::Host,
    port: &mut TcPort<App>,
    platform: &mut EspPlatform,
) -> ! {
    loop {
        // Trial window without FW_CONFIRM: the previous image is selected
        // (or the next boot's check selects it); restart into it.
        if let TrialPoll::Expired { .. } = platform.poll_trial(now_ms()) {
            esp_hal::system::software_reset();
        }
        if let Some(PlatformRequest::Reset { .. }) = ncp.service(now_us(), host, port, &LOG) {
            // Mode 1 (ROM download) is entered by the host through the USJ
            // DTR/RTS lines or the board's BOOT line; both modes reboot.
            esp_hal::system::software_reset();
        }
        while ncp.drive(port, now_us).await {
            ncp.service(now_us(), host, port, &LOG);
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
