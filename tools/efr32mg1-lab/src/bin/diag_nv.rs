#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use zigbee_runtime::nv_storage::{NvItemId, NvStorage};

#[cortex_m_rt::entry]
fn main() -> ! {
    const TEST_ITEM: NvItemId = NvItemId::AppCustomBase;
    const TEST_PAYLOAD: [u8; 16] = *b"EFR32-NV-PROBE!!";

    platform::init_small!("diag-nv");
    rtt_target::rprintln!(
        "[EFR32][diag-nv] BOOT nv=0x{:08X}..0x{:08X} radio=off rtcc=off",
        efr32mg1_tradfri::storage::APP_NV_PARTITION_START,
        efr32mg1_tradfri::storage::APP_NV_PARTITION_START
            + efr32mg1_tradfri::storage::APP_NV_PARTITION_SIZE as u32
    );

    let mut nv = match efr32mg1_tradfri::storage::application_nv() {
        Ok(nv) => nv,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-nv] OPEN_FAIL error={:?}", error);
            platform::halt()
        }
    };
    if let Err(error) = nv.write(TEST_ITEM, &TEST_PAYLOAD) {
        rtt_target::rprintln!("[EFR32][diag-nv] WRITE_FAIL error={:?}", error);
        platform::halt()
    }

    let mut readback = [0; TEST_PAYLOAD.len()];
    match nv.read(TEST_ITEM, &mut readback) {
        Ok(length) if length == TEST_PAYLOAD.len() && readback == TEST_PAYLOAD => {
            rtt_target::rprintln!("[EFR32][diag-nv] PASS bytes={} page_size=0x800", length);
            platform::led_on();
        }
        Ok(length) => rtt_target::rprintln!(
            "[EFR32][diag-nv] VERIFY_FAIL bytes={} data={:02X?}",
            length,
            &readback[..length.min(readback.len())]
        ),
        Err(error) => rtt_target::rprintln!("[EFR32][diag-nv] READ_FAIL error={:?}", error),
    }
    platform::halt()
}
