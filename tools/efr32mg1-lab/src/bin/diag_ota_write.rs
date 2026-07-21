#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use efr32mg1_tradfri::ota::Efr32FirmwareWriter;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

const FIRST_LEN: usize = 250;
const SECOND_LEN: usize = 17;
const TOTAL_LEN: usize = FIRST_LEN + SECOND_LEN;

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-ota-write");

    let mut writer = Efr32FirmwareWriter::new().unwrap_or_else(|_| fail("DISCOVERY_FAIL"));
    writer.erase_slot().unwrap_or_else(|_| fail("ERASE1_FAIL"));

    let mut first = [0u8; FIRST_LEN];
    for (index, byte) in first.iter_mut().enumerate() {
        *byte = (index as u8).wrapping_mul(37) ^ 0xA5;
    }
    let mut second = [0u8; SECOND_LEN];
    for (index, byte) in second.iter_mut().enumerate() {
        *byte = (index as u8).wrapping_mul(19) ^ 0x5A;
    }

    writer
        .write_block(0, &first)
        .unwrap_or_else(|_| fail("WRITE1_FAIL"));
    writer
        .write_block(FIRST_LEN as u32, &second)
        .unwrap_or_else(|_| fail("WRITE2_FAIL"));

    let mut readback = [0u8; TOTAL_LEN];
    writer
        .read_block(0, &mut readback)
        .unwrap_or_else(|_| fail("READBACK_FAIL"));
    if readback[..FIRST_LEN] != first || readback[FIRST_LEN..] != second {
        fail("COMPARE_FAIL");
    }
    if writer.verify(TOTAL_LEN as u32, None) != Err(FirmwareError::VerifyFailed) {
        fail("INVALID_GBL_ACCEPTED");
    }

    writer.erase_slot().unwrap_or_else(|_| fail("ERASE2_FAIL"));
    let mut erased = [0u8; 4];
    writer
        .read_block(0, &mut erased)
        .unwrap_or_else(|_| fail("ERASE_READ_FAIL"));
    if erased != [0xFF; 4] {
        fail("ERASE_COMPARE_FAIL");
    }
    writer.abort().unwrap_or_else(|_| fail("DEINIT_FAIL"));

    rtt_target::rprintln!(
        "[EFR32][diag-ota-write] PASS slot={} bytes={} cross=256 invalid=reject erased={:02X?}",
        writer.slot_size(),
        TOTAL_LEN,
        erased
    );
    platform::led_on();
    platform::halt()
}

fn fail(reason: &str) -> ! {
    rtt_target::rprintln!("[EFR32][diag-ota-write] {}", reason);
    platform::halt()
}
