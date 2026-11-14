//! USB CDC ACM transport of the router log, for boards without an ST-LINK
//! virtual COM port (the P-NUCLEO-WB55 USB dongle).
//!
//! [`build`] creates a full-speed device with one CDC ACM function from the
//! chip's USB peripheral; the composition root runs [`LogDevice::run`] and
//! [`drain`] in two tasks. The log pipe ([`crate::log`]) keeps dropping
//! whole lines while no host reads the port, so a closed terminal never
//! stalls the router. The USB device is bus-powered by the same port: when
//! it is unplugged the firmware is not running either, so VBUS detection is
//! not needed.
//!
//! The fault path ([`crate::log::emergency`]) stays silent with this
//! transport: it cannot run the USB stack with interrupts disabled.

use embassy_usb::class::cdc_acm::device::{CdcAcmClass, Config as AcmConfig, State};
use embassy_usb::{Builder, UsbDevice};
use static_cell::StaticCell;
use stm32wb_hal::embassy_stm32::peripherals::USB;
use stm32wb_hal::embassy_stm32::usb::Driver;

pub type UsbDriver = Driver<'static, USB>;
pub type LogClass = CdcAcmClass<'static, UsbDriver>;
pub type LogDevice = UsbDevice<'static, UsbDriver>;

/// USB identity of the log port.
pub struct Descriptor {
    pub vid: u16,
    pub pid: u16,
    pub manufacturer: &'static str,
    pub product: &'static str,
}

/// Bulk packets stay one byte below the 64-byte maximum so that no
/// zero-length packet is ever needed to end a transfer.
const CHUNK: usize = 63;

struct Buffers {
    config: [u8; 128],
    bos: [u8; 32],
    control: [u8; 64],
    state: State<'static>,
    serial: [u8; 16],
}

static BUFFERS: StaticCell<Buffers> = StaticCell::new();

/// Build the CDC ACM log device. The serial number is the factory EUI-64
/// in hex, so a host can tell several dongles apart. Call once.
pub fn build(driver: UsbDriver, descriptor: &Descriptor) -> (LogDevice, LogClass) {
    let buffers = BUFFERS.init(Buffers {
        config: [0; 128],
        bos: [0; 32],
        control: [0; 64],
        state: State::new(),
        serial: [0; 16],
    });
    let serial = hex_serial(&mut buffers.serial);

    let mut config = embassy_usb::Config::new(descriptor.vid, descriptor.pid);
    config.manufacturer = Some(descriptor.manufacturer);
    config.product = Some(descriptor.product);
    config.serial_number = Some(serial);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        &mut buffers.config,
        &mut buffers.bos,
        &mut [],
        &mut buffers.control,
    );
    let class = CdcAcmClass::new(&mut builder, &mut buffers.state, AcmConfig::new());
    (builder.build(), class)
}

/// EUI-64 (most significant byte first) as 16 upper-case hex digits.
fn hex_serial(out: &'static mut [u8; 16]) -> &'static str {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let eui = stm32wb_hal::identity::eui64();
    for (i, byte) in eui.iter().rev().enumerate() {
        out[2 * i] = HEX[usize::from(byte >> 4)];
        out[2 * i + 1] = HEX[usize::from(byte & 0xF)];
    }
    core::str::from_utf8(out).unwrap_or("0")
}

/// Move queued log bytes to the CDC ACM port; run it in its own task.
pub async fn drain(mut class: LogClass) -> ! {
    let mut chunk = [0u8; CHUNK];
    loop {
        class.wait_connection().await;
        loop {
            let count = crate::log::read(&mut chunk).await;
            if class.write_packet(&chunk[..count]).await.is_err() {
                break;
            }
        }
    }
}
