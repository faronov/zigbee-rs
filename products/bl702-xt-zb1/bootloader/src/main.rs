//! Bootloader for the BL702 XT-ZB1 sensor: MCUboot image format,
//! overwrite-only, with this project's own signing key.
//!
//! The mask ROM authenticates nothing but this loader (boot-header CRC and
//! SHA-256). Each reset the loader installs a pending OTA image from the
//! secondary slot (power-fail safe and resumable), validates the primary
//! image (SHA-256 and ECDSA P-256 over the `imgtool` TLVs, downgrade
//! prevention) and starts it at XIP `0x2300E100`. Status lines go to the
//! monitor UART (GPIO14, 2 Mbaud); with nothing bootable it halts until the
//! next reset.
//!
//! The 32-byte [`LOADER_INFO`] record at physical `0xFFE0` tells the
//! application that it runs under this loader and which key it trusts,
//! which selects the application's OTA channel.

#![no_std]
#![no_main]

use bl702_boot::{Config, Console, TrustedKey};
use bl702_hal::flash::XipFlash;
use bl702_hal::uart::Uart0Tx;
use bl702_xt_zb1::{ONBOARD_FLASH_CAPACITY, Resources};
use bl702_xt_zb1_layout::{LOADER_START, LOADER_VERSION, MCUBOOT, OWN_KEY_XY, XIP_BASE};
use panic_halt as _;

/// LoaderInfo record ("ZRSBOOT1", loader version, first 8 bytes of the
/// trusted key's X coordinate).
#[unsafe(link_section = ".loader_info")]
#[used]
pub static LOADER_INFO: [u8; 32] = bl702_boot::loader_info(LOADER_VERSION, &OWN_KEY_XY);

static CONFIG: Config = Config {
    layout: MCUBOOT,
    key: TrustedKey::new(OWN_KEY_XY),
    img_start: LOADER_START,
    xip_base: XIP_BASE,
};

struct Uart(Option<Uart0Tx<14, 15>>);

impl Console for Uart {
    fn write(&mut self, bytes: &[u8]) {
        if let Some(uart) = self.0.as_mut() {
            // A stuck FIFO must not stop the boot decision.
            let _ = uart.write_all(bytes);
        }
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let Some(resources) = Resources::take() else {
        halt()
    };
    let (runtime, application) = resources.split();
    let mut console = Uart(runtime.monitor.into_uart(runtime.clocks).ok());
    console.write(b"\r\nbl702-boot: v");
    bl702_boot::write_decimal(&mut console, LOADER_VERSION);
    console.write(b"\r\n");
    // Keep the record even if nothing else references it.
    // SAFETY: plain read of an initialized static.
    let _ = unsafe { core::ptr::read_volatile(&LOADER_INFO[0]) };
    match XipFlash::new(application.flash, ONBOARD_FLASH_CAPACITY) {
        Ok(flash) => bl702_boot::run(flash, &CONFIG, &mut console),
        Err(_) => {
            console.write(b"bl702-boot: flash config error\r\nbl702-boot: halted\r\n");
            halt()
        }
    }
}

fn halt() -> ! {
    loop {
        riscv::asm::wfi();
    }
}
