#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use efr32mg1_hal::bootloader::Bootloader;

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-ota-storage");

    let mut bootloader = match Bootloader::discover() {
        Ok(bootloader) => bootloader,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-ota-storage] DISCOVERY_FAIL error={:?}", error);
            platform::halt()
        }
    };
    let info = bootloader.info();
    if let Err(error) = bootloader.init() {
        rtt_target::rprintln!("[EFR32][diag-ota-storage] INIT_FAIL error={:?}", error);
        platform::halt()
    }

    let storage = match bootloader.storage_info() {
        Ok(storage) => storage,
        Err(error) => {
            rtt_target::rprintln!(
                "[EFR32][diag-ota-storage] STORAGE_INFO_FAIL error={:?}",
                error
            );
            let _ = bootloader.deinit();
            platform::halt()
        }
    };
    let slot = match bootloader.storage_slot(0) {
        Ok(slot) => slot,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-ota-storage] SLOT0_FAIL error={:?}", error);
            let _ = bootloader.deinit();
            platform::halt()
        }
    };
    let mut bytes = [0u8; 4];
    match bootloader.read_slot(0, 0, &mut bytes) {
        Ok(()) => {
            rtt_target::rprintln!(
                "[EFR32][diag-ota-storage] PASS v={:08X} c={:08X} {:?} n={} p={} e={} w={} s={:08X}+{} d={:02X?}",
                info.version,
                info.capabilities,
                storage.storage_type,
                storage.num_slots,
                storage.implementation.part_size,
                storage.implementation.page_size,
                storage.implementation.word_size_bytes,
                slot.address,
                slot.length,
                bytes
            );
            platform::led_on();
        }
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-ota-storage] READ_FAIL error={:?}", error)
        }
    }

    if let Err(error) = bootloader.deinit() {
        rtt_target::rprintln!("[EFR32][diag-ota-storage] DEINIT_FAIL error={:?}", error);
    }
    platform::halt()
}
