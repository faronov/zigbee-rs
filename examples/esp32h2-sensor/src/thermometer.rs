//! # ESP32-H2 thermometer (sleepy end device)
//!
//! Battery thermometer that drives no status hardware (lab board: the
//! Waveshare ESP32-H2-Zero, whose WS2812 DIN on GPIO8 is only isolated with
//! the other unused pads, `sleep_isolate_gpio_mask`): one
//! Home Automation Temperature Sensor endpoint with Basic, Identify,
//! Temperature Measurement (eFuse-calibrated die temperature) and the OTA
//! Upgrade client. Reporting defaults, poll and sample cadence,
//! identity and the USB policy come from `thermometer.toml`; Configure
//! Reporting from the coordinator overrides the reporting defaults.
//!
//! BOOT (GPIO9): short press = report now + fast-poll window (or join when
//! unjoined), 3 s hold = factory reset.
//!
//! ```bash
//! cargo build --release --bin esp32h2-thermometer --features thermometer,light-sleep,quiet
//! ```
//! `quiet` compiles out every log statement and the serial console.

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod chip_temperature;
#[cfg(feature = "anomaly-record")]
mod thermometer_anomaly;
#[cfg(not(feature = "quiet"))]
mod thermometer_diag;
mod thermometer_platform;
mod thermometer_profile;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use esp32_zigbee_devkit_product as product;
use esp32_zigbee_devkit_product::executor::block_on;
use esp32_zigbee_devkit_product::wake::SensorWake;
use esp_hal::gpio::{Input, InputConfig, Pull};
use sensor_sed_app::{ForceReportAction, NoBattery, NoStatus, SensorApp, SensorSedParts};
use static_cell::StaticCell;
use zigbee_mac::esp::{EspRadioPhy, EspSoftMac};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

use chip_temperature::H2TemperatureSensor;
use thermometer_platform::{ThermoDiagnostics, ThermoSupervisor};
use thermometer_profile::{config, DieThermometer, ThermoProfile, POLICY};

/// Unused pads isolated at boot; flash/USB pads fail the build here.
const UNUSED_PADS: product::pad_isolation::UnusedPads =
    product::pad_isolation::UnusedPads::checked(config::SLEEP_ISOLATE_GPIO_MASK);
// GPIO9 is the BOOT button and GPIO wake source below.
const _: () = assert!(!UNUSED_PADS.contains(9));

#[cfg(not(feature = "quiet"))]
use esp_backtrace as _;

/// `quiet` uses the product wake controller directly; other builds wrap it
/// in the wait-loop summary.
#[cfg(feature = "quiet")]
type Wake = SensorWake<'static>;
#[cfg(not(feature = "quiet"))]
type Wake = thermometer_diag::DiagWake<SensorWake<'static>>;

type Parts = SensorSedParts<
    Wake,
    NoStatus,
    DieThermometer,
    NoBattery,
    product::ota_transport::OtaTransport,
    ForceReportAction,
    ThermoSupervisor,
    ThermoDiagnostics,
>;
type App =
    SensorApp<'static, EspSoftMac<'static>, product::storage::SecurityStore, ThermoProfile, Parts>;

/// Quiet builds carry no panic formatter or console: record a panic-site
/// hint for the next boot's anomaly record, reset and let the security
/// journal resume the network. `PanicInfo::location` is deliberately not
/// read (it would keep every panic `Location` and path string, ~35 KB).
#[cfg(feature = "quiet")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let site = panic_site();
    thermometer_platform::record_reset(sensor_sed_app::ResetCause::panic_site(site));
    esp_hal::system::software_reset()
}

/// Flash-code offset (from `0x4200_0000`) of the panicking function's saved
/// return address, 0 if it is not found within 64 stack words.
///
/// With `panic = "abort"` and an unused `PanicInfo` the `core::panicking`
/// shims are frameless calls, so the panicking call site itself is lost.
/// The handler's frame holds its own return address into `panic_fmt` (the
/// entry `ra`, read below before any call); the next word above it that
/// lies in the code window (above `_rodata_end`) is the return address the
/// panicking function saved, i.e. a call site in its caller. It is a
/// heuristic hint (a code pointer in a local can precede it): resolve it
/// against the exact ELF with `addr2line -f -e <elf> <0x4200_0000+offset>`.
#[cfg(feature = "quiet")]
#[inline(always)]
fn panic_site() -> u32 {
    const CODE_BASE: usize = 0x4200_0000;
    const CODE_END: usize = 0x4280_0000;
    const SCAN_WORDS: usize = 64;
    unsafe extern "C" {
        static _rodata_end: u32;
        static _stack_start: u32;
    }
    let entry_ra: usize;
    let sp: usize;
    // SAFETY: register reads only; inlined into the handler before it
    // makes any call, so `ra` is still the handler's return address.
    unsafe {
        core::arch::asm!("mv {}, ra", out(reg) entry_ra, options(nomem, nostack));
        core::arch::asm!("mv {}, sp", out(reg) sp, options(nomem, nostack));
    }
    let code = core::ptr::addr_of!(_rodata_end) as usize..CODE_END;
    let stack_top = core::ptr::addr_of!(_stack_start) as usize;
    let mut past_handler = false;
    let mut at = sp;
    for _ in 0..SCAN_WORDS {
        if at + 4 > stack_top {
            break;
        }
        // SAFETY: an aligned word between the live stack pointer and the
        // top of the stack region.
        let word = unsafe { core::ptr::read_volatile(at as *const usize) };
        at += 4;
        if !past_handler {
            past_handler = word == entry_ra;
        } else if code.contains(&word) && word & 1 == 0 {
            return (word - CODE_BASE) as u32;
        }
    }
    0
}

/// Unrecoverable startup failure. Spin rather than reset so a bad partition
/// table or persistence error cannot become a reset loop that rewrites NV.
fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[inline(never)]
fn build_mac(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> EspSoftMac<'static> {
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(radio);
    const POLL_WAIT_US: core::num::NonZeroU32 =
        match core::num::NonZeroU32::new(config::POLL_RESPONSE_WAIT_US) {
            Some(wait_us) => wait_us,
            None => panic!("poll_response_wait_ms must be non-zero"),
        };
    let mut phy = EspRadioPhy::new(ieee802154, esp_radio::ieee802154::Config::default())
        .with_poll_response_wait_us(POLL_WAIT_US);
    if phy.install_aes_engine(aes).is_err() {
        halt()
    }
    let Ok(mac) = phy.into_soft_mac() else { halt() };
    mac
}

/// Build MAC, device, store and profile in static storage; not inlined so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> (
    &'static mut ZigbeeDevice<EspSoftMac<'static>>,
    &'static mut product::storage::SecurityStore,
    &'static mut ThermoProfile,
) {
    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<ThermoProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<EspSoftMac<'static>>> = StaticCell::new();

    // A legacy-migration or partition-table error may leave NV intact: stop
    // rather than commission as factory-new.
    let Ok((store, _migration)) = product::storage::open_security_store() else {
        halt()
    };
    let Ok(profile) =
        thermometer_profile::thermo_profile(FIRMWARE_VERSION, esp_hal::system::software_reset)
    else {
        halt()
    };
    let security = SECURITY.init(store);
    let profile = PROFILE.init(profile);

    let mac = build_mac(radio, aes);
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        halt()
    }
    let device = ZigbeeDevice::builder(mac)
        .power_mode(POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(config::MODEL)
        .application_version(product::firmware::application_version(FIRMWARE_VERSION))
        .date_code(product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build_into(DEVICE.uninit());

    // Keys and frame counters are never resumed under another EUI-64 or by
    // another role image; a switch clears the network state (keeping the
    // counter reservations) and forces a fresh join.
    if device
        .reset_security_state_if_identity_changed(security, ieee)
        .is_err()
        || security.select_role_tag(config::ROLE_TAG).is_err()
    {
        halt()
    }
    (device, security, profile)
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // The radio's receive queue allocates from this heap.
    esp_alloc::heap_allocator!(size: 32768);
    #[cfg(not(feature = "quiet"))]
    thermometer_platform::console::init();
    product::time_driver::init(peripherals.SYSTIMER);
    // Consume the previous boot's reset cause and rapid-reset count before
    // anything below can reset again.
    let boot = product::reset_cause::boot(sensor_sed_app::RapidResetPolicy {
        limit: config::RAPID_RESET_LIMIT,
        window_ms: config::RAPID_RESET_WINDOW_MS,
    });

    // Quiet builds also suppress the ROM bootloader banner on later resets.
    let rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);
    #[cfg(feature = "quiet")]
    rtc.disable_rom_message_printing();
    #[cfg(feature = "light-sleep")]
    let sleep = Some(
        product::sleep::LightSleep::new(
            rtc,
            peripherals.LP_TIMER,
            peripherals.PMU,
            peripherals.LP_AON,
        )
        .with_power_down(product::sleep::RetainedPowerDown {
            xtal: config::SLEEP_XTAL_OFF,
            rc_fast: config::SLEEP_RC_FAST_OFF,
        }),
    );
    #[cfg(not(feature = "light-sleep"))]
    let sleep = {
        let _rtc = rtc;
        None
    };
    // SAFETY: no pad in the set is used by this image (build-checked: not
    // flash, USB, the button or the console UART); before the first sleep.
    unsafe { UNUSED_PADS.apply() };
    // `sleep_selftest`: real light sleeps before the radio exists.
    #[cfg(all(feature = "light-sleep", not(feature = "quiet")))]
    let mut sleep = sleep;
    #[cfg(all(feature = "light-sleep", not(feature = "quiet")))]
    thermometer_diag::sleep_selftest(sleep.as_mut());

    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );
    let Ok(sensor) = H2TemperatureSensor::new() else {
        halt()
    };

    // An unconfirmed OTA image rolls back before persistence is touched.
    if let Ok(product::ota::BootCheck::RolledBack { .. }) = product::ota_boot::begin() {
        esp_hal::system::software_reset();
    }

    static APP: StaticCell<App> = StaticCell::new();
    let (device, security, profile) = compose(peripherals.IEEE802154, peripherals.AES);
    let parts = SensorSedParts {
        #[cfg(feature = "quiet")]
        wake: SensorWake::new(button, sleep),
        #[cfg(not(feature = "quiet"))]
        wake: thermometer_diag::DiagWake::new(SensorWake::new(button, sleep)),
        status: NoStatus,
        environment: DieThermometer::new(sensor),
        battery: NoBattery,
        ota: product::ota_transport::OtaTransport::new(),
        actions: ForceReportAction,
        supervisor: ThermoSupervisor::new(),
        diagnostics: ThermoDiagnostics,
    };
    let node = ZigbeeNode::new(device, security, profile);
    let Ok(app) = SensorApp::new(node, &POLICY, parts) else {
        halt()
    };
    block_on(run(APP.init(app), boot))
}

/// Restore the anomaly record → `initialize` → restore the persisted
/// reporting configuration and Poll Control values → `step` forever,
/// persisting a changed configuration (and a new anomaly) after each
/// iteration.
async fn run(app: &'static mut App, boot: sensor_sed_app::BootRecord) -> ! {
    #[cfg(feature = "anomaly-record")]
    let mut anomalies = product::anomaly::anomaly_store(config::ANOMALY_MAX_WRITES_PER_BOOT);
    #[cfg(feature = "anomaly-record")]
    if let Ok(record) = anomalies.load() {
        app.restore_anomalies(record);
        publish_anomalies(app, &record);
    }
    // After the restore (which clears the pending update), so a carried
    // cause is published and persisted by the first loop iteration.
    if let Some(cause) = boot.carried {
        app.note_reset(cause);
    }
    if boot.guard_tripped {
        app.disable_sleep(boot.guard_first_trip, boot.carried);
    }
    log::info!(
        "[boot] reset 0x{:02X} cause 0x{:08X} rapid {} sleep {}",
        product::reset_cause::raw_reset_reason(),
        boot.carried.map_or(0, sensor_sed_app::ResetCause::word),
        boot.rapid_resets,
        if boot.guard_tripped { "off" } else { "on" },
    );
    let stable_at = embassy_time::Instant::now()
        + embassy_time::Duration::from_millis(u64::from(config::RAPID_RESET_WINDOW_MS));
    let mut stable = false;
    if app.initialize().await.is_err() {
        halt()
    }
    #[cfg(feature = "reporting-persist")]
    let mut reporting = Some(product::reporting::reporting_persistence());
    #[cfg(feature = "reporting-persist")]
    if let Some(persistence) = reporting.as_mut() {
        // A failed restore keeps the product defaults; the next sync
        // replaces the unreadable snapshot.
        let (device, poll_control) = app.device_and_poll_control_mut();
        let _ = persistence.restore_with(device, poll_control);
    }
    loop {
        let _ = app.step().await;
        if !stable && embassy_time::Instant::now() >= stable_at {
            product::reset_cause::mark_stable();
            stable = true;
        }
        #[cfg(feature = "anomaly-record")]
        if let Some(record) = app.take_anomaly_update() {
            publish_anomalies(app, &record);
            // Rare by construction (a fired limit) and capped per boot by
            // `anomaly_max_writes_per_boot`; a failed write only loses the
            // persistent copy, the attributes still carry it.
            let _ = anomalies.store(&record);
        }
        #[cfg(feature = "reporting-persist")]
        if let Some(persistence) = reporting.as_mut() {
            let (device, poll_control) = app.device_and_poll_control_mut();
            if persistence
                .sync_with(device, poll_control.as_deref())
                .is_err()
            {
                // Flash refused the commit: stop for this boot instead of
                // re-erasing the window on every wake.
                reporting = None;
            }
        }
    }
}

#[cfg(feature = "anomaly-record")]
fn publish_anomalies(app: &mut App, record: &sensor_sed_app::AnomalyRecord) {
    app.profile_mut()
        .inner_mut()
        .component_mut()
        .anomalies_mut()
        .publish(record);
}
