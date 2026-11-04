//! Thermometer supervisor and diagnostics.
//!
//! With `quiet`, no logger is installed and `log` is compiled out, so
//! diagnostics only feed the OTA boot-confirmation observer.

use embassy_time::Instant;
use esp32_zigbee_devkit_product as product;
use esp_hal::peripherals::USB_DEVICE;
use sensor_sed_app::{DiagnosticEvent, Diagnostics, Supervisor};

use crate::thermometer_profile::config;

/// OTA boot supervision plus the one-shot USB-Serial-JTAG pad shutdown.
pub struct ThermoSupervisor {
    usb: UsbPad,
}

impl ThermoSupervisor {
    pub fn new() -> Self {
        Self {
            usb: UsbPad::Pending(Instant::now()),
        }
    }
}

impl Supervisor for ThermoSupervisor {
    fn heartbeat(&mut self) {
        product::ota_boot::poll();
        self.usb.service();
    }

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        esp_hal::system::software_reset()
    }
}

/// USB-Serial-JTAG pad state.
///
/// If no host start-of-frame was latched within `usb_host_wait_ms` of boot,
/// the device is battery/bench powered: the USB PHY pad is disabled for the
/// rest of the power cycle. A host-attached boot keeps it (and therefore
/// JTAG flashing) available. `usb_host_wait_ms = 0` keeps the pad enabled.
/// Re-plugging USB later needs a power cycle or reset to re-enumerate.
enum UsbPad {
    Pending(Instant),
    Settled,
}

impl UsbPad {
    fn service(&mut self) {
        let Self::Pending(boot) = *self else {
            return;
        };
        if config::USB_HOST_WAIT_MS == 0 {
            *self = Self::Settled;
            return;
        }
        if boot.elapsed().as_millis() < u64::from(config::USB_HOST_WAIT_MS) {
            return;
        }
        let usb = USB_DEVICE::regs();
        if !usb.int_raw().read().sof().bit_is_set() {
            usb.conf0()
                .modify(|_, w| w.usb_pad_enable().clear_bit().dp_pullup().clear_bit());
            log::info!("[H2-T] no USB host: USB pad disabled");
        }
        *self = Self::Settled;
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ThermoDiagnostics;

impl Diagnostics for ThermoDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        log::info!("[H2-T] {:?}", event);
        product::ota_boot::observe(&event);
    }
}

/// Logger for non-`quiet` builds.
#[cfg(not(feature = "quiet"))]
pub mod console {
    struct EspLogger;

    impl log::Log for EspLogger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            esp_println::println!("[{}] {}", record.level(), record.args());
        }

        fn flush(&self) {}
    }

    static LOGGER: EspLogger = EspLogger;

    pub fn init() {
        let _ = log::set_logger(&LOGGER);
        log::set_max_level(log::LevelFilter::Info);
    }
}
