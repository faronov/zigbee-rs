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
use zigbee_runtime::firmware_writer::FirmwareWriter;

const GBL: &[u8] = include_bytes!(env!("EFR32_GBL_PATH"));
const WRITE_CHUNK: usize = 256;

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-ota-install");

    let mut writer = Efr32FirmwareWriter::new().unwrap_or_else(|_| fail("DISCOVERY_FAIL"));
    writer
        .erase_slot()
        .unwrap_or_else(|_| fail("ERASE_FAIL"));
    if GBL.len() as u32 > writer.slot_size() {
        fail("GBL_TOO_LARGE");
    }

    for (index, chunk) in GBL.chunks(WRITE_CHUNK).enumerate() {
        writer
            .write_block((index * WRITE_CHUNK) as u32, chunk)
            .unwrap_or_else(|_| fail("WRITE_FAIL"));
    }
    writer
        .verify(GBL.len() as u32, None)
        .unwrap_or_else(|_| fail("VERIFY_FAIL"));

    rtt_target::rprintln!(
        "[EFR32][diag-ota-install] VERIFIED bytes={} installing",
        GBL.len()
    );
    platform::led_on();
    writer
        .activate()
        .unwrap_or_else(|_| fail("ACTIVATE_FAIL"));
    platform::halt()
}

fn fail(reason: &str) -> ! {
    rtt_target::rprintln!("[EFR32][diag-ota-install] {}", reason);
    platform::halt()
}
