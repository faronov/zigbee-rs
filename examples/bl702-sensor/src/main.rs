//! Pure-Rust BL702 Zigbee temperature, humidity, and battery sensor.
//!
//! This is a composition root only: BL702 startup, exclusive resource
//! construction, radio/AES initialization, the product identity guard, and
//! assembly of the shared `sensor-sed-app` lifecycle.

#![no_std]
#![no_main]

#[cfg(all(feature = "production", feature = "diagnostic-logging"))]
compile_error!("select either production or diagnostic-logging, not both");
#[cfg(not(any(feature = "production", feature = "diagnostic-logging")))]
compile_error!("select production or diagnostic-logging");

mod hal;
mod time_driver;
mod wake;

use core::cell::RefCell;
use core::fmt::{Display, Formatter, Write};

use bl702_xt_zb1_product as product;
use embassy_executor::Executor;
use panic_halt as _;
use product::ota::{Bl702OtaLifecycle, Channel};
use product::storage::{OtaFlash, SharedFlash};
use sensor_sed_app::{BlockingBattery, BlockingEnvironment, NoStatus, SensorApp, SensorSedParts};
use zigbee_mac::{MacPib, SoftMacCore, bl702::radio_phy::Bl702RadioPhy};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_zcl::clusters::basic::PowerSource;

struct UartWriter;

impl Write for UartWriter {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        for byte in text.bytes() {
            hal::uart_write(byte).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

struct Logger;

impl log::Log for Logger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let mut uart = UartWriter;
        let _ = writeln!(uart, "[{}] {}\r", record.level(), record.args());
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

struct Hex<'a>(&'a [u8]);

impl Display for Hex<'_> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> core::fmt::Result {
        for byte in self.0.iter().rev() {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let application_resources = match hal::init() {
        Ok(resources) => resources,
        Err(_) => hal::halt(),
    };
    marker(b"BL702 boot\r\n");

    // BL702 is single-hart and has no RV32A extension. Interrupts are still
    // disabled here, so the non-atomic logger setup is the correct startup path.
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(log::LevelFilter::Info);
    }
    marker(b"logger ready\r\n");

    // Enable the TIMER channel 0 alarm interrupt so the executor sleeps in
    // WFI between timer deadlines. On failure the driver keeps the previous
    // polling behavior.
    if time_driver::start() {
        marker(b"alarm irq ready\r\n");
    } else {
        marker(b"alarm irq FAILED: polling executor\r\n");
    }

    let mut executor = Executor::new();
    let executor: &'static mut Executor = unsafe { core::mem::transmute(&mut executor) };
    marker(b"executor ready\r\n");
    executor.run(move |spawner| spawner.must_spawn(sensor(application_resources)))
}

#[embassy_executor::task]
async fn sensor(application: hal::ApplicationResources) {
    marker(b"sensor task\r\n");
    log::info!("Zigbee-RS BL702 pure-Rust sensor");

    let hal::ApplicationResources {
        i2c0,
        spi_or_usb,
        adc,
        flash,
        power,
        pwm,
        uart1,
        aes,
        other_pins,
    } = application;
    let battery = hal::SupplyBattery::new(adc);

    // One flash token shared by the security journal, the OTA writer and
    // the loader-record read; each operation borrows it briefly and every
    // user is confined to its own partition.
    let flash = match product::storage::main_flash(flash) {
        Ok(flash) => RefCell::new(flash),
        Err(error) => {
            log::error!("security storage initialization failed: {error:?}");
            hal::halt()
        }
    };
    let mut security_store = product::storage::security_store(SharedFlash::new(&flash));
    let channel = Channel::read(&mut OtaFlash::new(SharedFlash::new(&flash)));
    log_version(channel);

    // Retain ownership of unused fitted/application resources for the task's
    // lifetime. No physical environmental sensor is selected for this product.
    let _reserved_peripherals = (i2c0, spi_or_usb, power, pwm, uart1, other_pins);

    let ieee = product::identity::ieee_address(hal::chip_id());
    log::info!("IEEE address: {}", Hex(&ieee));
    log::info!("initializing RF and running per-die calibration");

    let mut radio = match unsafe { Bl702RadioPhy::initialize(hal::delay_us) } {
        Ok(radio) => radio,
        Err(error) => {
            log::error!("RF initialization failed: {error:?}");
            hal::halt()
        }
    };
    if let Err(error) = zigbee_mac::RadioPhy::set_tx_power(&mut radio, product::TX_POWER_DBM) {
        log::error!("TX power setup failed: {error:?}");
        hal::halt()
    }

    // The exclusive SEC_ENG token keeps CCM*/AES-MMO on hardware. Startup
    // runs both on-silicon known-answer tests and fails closed.
    if let Err(error) = radio.install_aes_engine(aes) {
        log::error!("hardware AES self-test failed: {error:?}");
        hal::halt()
    }

    let pib = MacPib::new(ieee, ieee[0], ieee[1]);
    let mac = match SoftMacCore::new(radio, pib) {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("MAC initialization failed: {error:?}");
            hal::halt()
        }
    };
    log::info!(
        "radio ready: channel scan={}, tx_power={} dBm",
        product::CHANNEL,
        product::TX_POWER_DBM
    );

    let mut profile = match product::ota::sensor_ota_profile(SharedFlash::new(&flash)) {
        Ok(profile) => profile,
        Err(error) => {
            log::error!("OTA profile composition failed: {error:?}");
            hal::halt()
        }
    };
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(product::CHANNEL_MASK)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();

    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => log::warn!("cleared persisted network state after IEEE address change"),
        Ok(false) => {}
        Err(error) => {
            log::error!("security state validation failed: {error:?}");
            hal::halt()
        }
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
    let mut app = match SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake: wake::Bl702Wake::new(),
            status: NoStatus,
            environment: BlockingEnvironment::new(product::environment::SyntheticEnvironment::new()),
            battery: BlockingBattery::new(battery),
            ota: Bl702OtaLifecycle::new(channel, bl702_hal::pm::software_reset),
            actions: product::policy::USER_ACTIONS,
            supervisor: hal::Bl702Supervisor,
            diagnostics: hal::Bl702Diagnostics,
        },
    ) {
        Ok(app) => app,
        Err(error) => {
            log::error!("invalid BL702 SensorApp composition: {error:?}");
            hal::halt()
        }
    };

    app.run().await
}

/// The running firmware version for the bootloader/OTA bench log. The raw
/// marker line also appears in production builds, whose `log` output is
/// compiled out.
fn log_version(channel: Channel) {
    let mut uart = UartWriter;
    let _ = write!(
        uart,
        "BL702 app {} (OTA file 0x{:08X}{})\r\n",
        product::SW_BUILD,
        product::ota::FILE_VERSION,
        if product::VERSION_OVERRIDDEN {
            ", TEST-ONLY version override"
        } else {
            ""
        }
    );
    log::info!(
        "OTA channel {channel:?}: manufacturer 0x{:04X} image type 0x{:04X} version 0x{:08X}",
        product::ota::MANUFACTURER_CODE,
        product::ota::IMAGE_TYPE,
        product::ota::FILE_VERSION
    );
}

/// The OTA file version, kept in the ELF so `tools/bl702_ota.py` can check
/// the Zigbee OTA header it writes against the firmware it wraps.
#[unsafe(no_mangle)]
#[used]
static BL702_OTA_FILE_VERSION: u32 = product::ota::FILE_VERSION;

fn marker(text: &[u8]) {
    for byte in text {
        let _ = hal::uart_write(*byte);
    }
}
