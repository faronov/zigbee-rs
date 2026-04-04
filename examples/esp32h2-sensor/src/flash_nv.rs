//! ESP32-H2 flash NV storage using esp-storage low-level API.

use zigbee_runtime::log_nv::{FlashDriver, LogStructuredNv};

const NV_PAGE_A: u32 = 0x003F_E000;
const NV_PAGE_B: u32 = 0x003F_F000;

pub struct EspFlashDriver;

impl EspFlashDriver {
    pub fn new() -> Self { Self }
}

impl FlashDriver for EspFlashDriver {
    fn read(&self, offset: u32, buf: &mut [u8]) {
        unsafe {
            let _ = esp_storage::ll::spiflash_read(
                offset,
                buf.as_mut_ptr() as *mut u32,
                buf.len() as u32,
            );
        }
    }

    fn write(&mut self, offset: u32, data: &[u8]) {
        unsafe {
            let _ = esp_storage::ll::spiflash_unlock();
            let _ = esp_storage::ll::spiflash_write(
                offset,
                data.as_ptr() as *const u32,
                data.len() as u32,
            );
        }
    }

    fn erase_sector(&mut self, offset: u32) {
        unsafe {
            let _ = esp_storage::ll::spiflash_unlock();
            let _ = esp_storage::ll::spiflash_erase_sector(offset / 4096);
        }
    }

    fn sector_size(&self) -> usize {
        4096
    }
}

pub fn create_nv() -> LogStructuredNv<EspFlashDriver> {
    LogStructuredNv::new(EspFlashDriver::new(), NV_PAGE_A, NV_PAGE_B)
}
