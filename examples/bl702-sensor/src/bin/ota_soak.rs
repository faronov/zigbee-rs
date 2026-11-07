//! LAB ONLY: loader power-cut soak for the XT-ZB1 (`bl702-ota-soak`).
//!
//! Build and package with `BL702_LAB_SOAK=1 ./build-image.sh`; the binary
//! alone is `cargo build --release --locked --features ota-soak --bin
//! bl702-ota-soak`.
//!
//! On every boot it:
//!
//! 1. confirms itself, as a healthy production image would;
//! 2. prints CRC-32s of the loader region (`0x0..0x10000`) and the security
//!    journal (`0xFE000..0x100000`); neither this image nor the loader may
//!    ever change them;
//! 3. after one second, re-stages its own primary image into the secondary
//!    slot through the product's `SlotWriter` and requests an install;
//! 4. resets into the loader, which erases the primary and copies the image
//!    back (an equal version is not a downgrade).
//!
//! The loop never ends, so a power cut at a random moment lands either in
//! the loader's erase/copy or in this re-staging.
//! `tools/bl702_soak_monitor.py` classifies and counts the cuts from the
//! UART log. A padding blob makes the image about as large as the sensor,
//! so the copy takes as long as a real install.
//!
//! Never publish this image: it has no Zigbee stack and re-installs itself
//! forever. Leave the loop over ISP with a full image whose version is
//! HIGHER than this one; with an equal or lower version the loader would
//! install the staged soak image again.

#![no_std]
#![no_main]

#[path = "../hal.rs"]
#[allow(dead_code)]
mod hal;
#[path = "../time_driver.rs"]
#[allow(dead_code)]
mod time_driver;

use core::fmt::Write;

use bl702_xt_zb1_layout::{LOADER_END, NV_SIZE, NV_START, PRIMARY_SLOT};
use bl702_xt_zb1_product as product;
use embedded_storage::nor_flash::ReadNorFlash;
use mcuboot_image::ImageHeader;
use panic_halt as _;
use product::storage::MainFlash;
use zigbee_runtime::firmware_writer::FirmwareWriter;

/// Makes the image about as large as the sensor (see `build.rs`).
static PAD: [u8; include_bytes!(concat!(env!("OUT_DIR"), "/soak-pad.bin")).len()] =
    *include_bytes!(concat!(env!("OUT_DIR"), "/soak-pad.bin"));

/// Read by `tools/bl702_ota.py release` for the image and OTA file version.
#[unsafe(no_mangle)]
#[used]
static BL702_OTA_FILE_VERSION: u32 = product::ota::FILE_VERSION;

/// MCUboot unprotected TLV area magic.
const TLV_INFO_MAGIC: u16 = 0x6907;
/// Delay between the report and the re-staging, so cuts also land here.
const RESTAGE_DELAY_US: u32 = 1_000_000;

struct Uart;

impl Write for Uart {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        for byte in text.bytes() {
            hal::uart_write(byte).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

fn fail(what: &str) -> ! {
    let _ = write!(Uart, "soak: FAIL {what}\r\n");
    hal::halt()
}

/// CRC-32 (IEEE) of `len` bytes of flash at `start`.
fn flash_crc(flash: &mut MainFlash, start: u32, len: u32) -> u32 {
    let mut crc = !0u32;
    let mut buf = [0u8; 256];
    let mut offset = 0;
    while offset < len {
        let take = core::cmp::min(buf.len() as u32, len - offset) as usize;
        if flash.read(start + offset, &mut buf[..take]).is_err() {
            fail("flash read");
        }
        for &byte in &buf[..take] {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
            }
        }
        offset += take as u32;
    }
    !crc
}

/// Length of the primary image, from its header through the unprotected
/// TLV area.
fn primary_len<F: ReadNorFlash>(flash: &mut F) -> u32 {
    let header = match ImageHeader::read(flash, PRIMARY_SLOT) {
        Ok(Some(header)) => header,
        _ => fail("no primary header"),
    };
    let tlv = u32::from(header.hdr_size) + header.img_size + u32::from(header.protect_tlv_size);
    let mut info = [0u8; 4];
    if flash.read(PRIMARY_SLOT + tlv, &mut info).is_err()
        || u16::from_le_bytes([info[0], info[1]]) != TLV_INFO_MAGIC
    {
        fail("no TLV area");
    }
    tlv + u32::from(u16::from_le_bytes([info[2], info[3]]))
}

#[riscv_rt::entry]
fn main() -> ! {
    product::stack_guard::arm();
    let Ok(resources) = hal::init() else {
        hal::halt()
    };
    let _ = write!(
        Uart,
        "soak: bl702-ota-soak {} file 0x{:08x} pad {} B (LAB ONLY)\r\n",
        product::SW_BUILD,
        BL702_OTA_FILE_VERSION,
        core::hint::black_box(&PAD).len(),
    );
    let Ok(mut flash) = product::storage::main_flash(resources.flash) else {
        fail("flash init")
    };

    let loader = flash_crc(&mut flash, 0, LOADER_END);
    let nv = flash_crc(&mut flash, NV_START, NV_SIZE);
    let _ = write!(Uart, "soak: crc loader={loader:08x} nv={nv:08x}\r\n");

    let mut writer = product::ota::writer(&mut flash);
    let confirmed = writer.confirm_image().is_ok();
    let _ = write!(
        Uart,
        "soak: confirm {} trial {:?}\r\n",
        if confirmed { "ok" } else { "FAILED" },
        writer.trial_state(),
    );
    if !confirmed {
        fail("confirm");
    }

    hal::delay_us(RESTAGE_DELAY_US);
    let len = primary_len(writer.flash_mut());
    let _ = write!(Uart, "soak: re-staging {len} B\r\n");
    if writer.erase_slot().is_err() {
        fail("erase secondary");
    }
    let mut buf = [0u8; 256];
    let mut offset = 0;
    while offset < len {
        let take = core::cmp::min(buf.len() as u32, len - offset) as usize;
        if writer
            .flash_mut()
            .read(PRIMARY_SLOT + offset, &mut buf[..take])
            .is_err()
        {
            fail("read primary");
        }
        if writer.write_block(offset, &buf[..take]).is_err() {
            fail("write secondary");
        }
        offset += take as u32;
    }
    if writer.verify(len, None).is_err() {
        fail("verify staged image");
    }
    if writer.activate().is_err() {
        fail("request install");
    }
    let _ = write!(Uart, "soak: re-armed, resetting\r\n");
    hal::delay_us(2_000);
    bl702_hal::pm::software_reset()
}
