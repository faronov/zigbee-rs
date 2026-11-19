//! Host fakes shared by the `fw` and `platform` tests: a NOR flash with the
//! NCP partition table, ESP application images and signed envelopes.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use esp32_zigbee_devkit_product::esp_image::{
    APP_DESC_MAGIC, APP_DESC_SIZE, EXPECTED_CHIP_ID, HEADER_SIZE, IMAGE_MAGIC, ImageCompatibility,
    SEGMENT_HEADER_SIZE,
};
use esp32_zigbee_devkit_product::ota::{OtaFlash, RunningImageError};
use esp32_zigbee_devkit_product::otadata::{ENTRY_SIZE, OtaData};
use esp32_zigbee_devkit_product::sha256::sha256;
use mcuboot_image::{KeySet, TrustedKey};
use p256::ecdsa::signature::hazmat::PrehashSigner;
use p256::ecdsa::{Signature, SigningKey};
use zigbee_runtime::firmware_writer::FirmwareError;

use crate::fw::{FwPolicy, partition_entry};
use crate::layout::{
    FLASH_SIZE, PARTITION_TABLE_OFFSET, SECTOR_SIZE, WORD_SIZE, is_fw_writable,
    otadata_sector_offset,
};

/// NOR flash of the whole chip (shared between "boots"): only erased bytes
/// are programmed, writes are word aligned, and every write/erase must be
/// inside `otadata` or an application slot (spec 7.9 protected regions).
#[derive(Clone)]
pub struct MockFlash {
    pub data: Rc<RefCell<Vec<u8>>>,
    pub running: Rc<Cell<Result<u8, RunningImageError>>>,
    /// Every erase/write `(address, len)` since the last `take_log`.
    pub erases: Rc<RefCell<Vec<u32>>>,
    pub writes: Rc<RefCell<Vec<(u32, usize)>>>,
    pub fail_write_at: Rc<Cell<Option<u32>>>,
}

impl MockFlash {
    /// Erased chip with the NCP partition table, running from `ota_0`.
    pub fn new() -> Self {
        let flash = Self {
            data: Rc::new(RefCell::new(vec![0xFF; FLASH_SIZE as usize])),
            running: Rc::new(Cell::new(Ok(0))),
            erases: Rc::default(),
            writes: Rc::default(),
            fail_write_at: Rc::default(),
        };
        for index in 0..4 {
            let start = PARTITION_TABLE_OFFSET as usize + index * 32;
            flash.data.borrow_mut()[start..start + 32].copy_from_slice(&partition_entry(index));
        }
        flash
    }

    pub fn set_running(&self, slot: u8) {
        self.running.set(Ok(slot));
    }

    pub fn bytes(&self, address: u32, len: usize) -> Vec<u8> {
        self.data.borrow()[address as usize..address as usize + len].to_vec()
    }

    pub fn put(&self, address: u32, bytes: &[u8]) {
        self.data.borrow_mut()[address as usize..address as usize + bytes.len()]
            .copy_from_slice(bytes);
    }

    pub fn otadata(&self) -> OtaData {
        let first: [u8; ENTRY_SIZE] = self
            .bytes(otadata_sector_offset(0), ENTRY_SIZE)
            .try_into()
            .unwrap();
        let second: [u8; ENTRY_SIZE] = self
            .bytes(otadata_sector_offset(1), ENTRY_SIZE)
            .try_into()
            .unwrap();
        OtaData::decode([&first, &second])
    }

    /// Erases and writes since the last call.
    pub fn take_log(&self) -> (Vec<u32>, Vec<(u32, usize)>) {
        (
            core::mem::take(&mut *self.erases.borrow_mut()),
            core::mem::take(&mut *self.writes.borrow_mut()),
        )
    }
}

impl OtaFlash for MockFlash {
    fn running_slot(&mut self) -> Result<u8, RunningImageError> {
        self.running.get()
    }

    fn image_compatibility(&mut self) -> Result<ImageCompatibility, FirmwareError> {
        Ok(ImageCompatibility {
            chip_id: EXPECTED_CHIP_ID,
            chip_revision: 0,
            efuse_block_revision: 0,
        })
    }

    fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FirmwareError> {
        let end = address as usize + buffer.len();
        assert!(end <= FLASH_SIZE as usize, "read beyond the chip");
        buffer.copy_from_slice(&self.data.borrow()[address as usize..end]);
        Ok(())
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FirmwareError> {
        assert_eq!(address % WORD_SIZE, 0, "unaligned write at {address:#x}");
        assert_eq!(data.len() % WORD_SIZE as usize, 0, "unaligned length");
        assert!(
            is_fw_writable(address, data.len() as u32),
            "write outside otadata/slots at {address:#x}"
        );
        if self.fail_write_at.get() == Some(address) {
            return Err(FirmwareError::WriteFailed);
        }
        let mut flash = self.data.borrow_mut();
        for (index, byte) in data.iter().enumerate() {
            let cell = &mut flash[address as usize + index];
            assert_eq!(*cell, 0xFF, "write to an unerased byte at {address:#x}");
            *cell = *byte;
        }
        self.writes.borrow_mut().push((address, data.len()));
        Ok(())
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), FirmwareError> {
        assert_eq!(address % SECTOR_SIZE, 0, "unaligned erase");
        assert!(
            is_fw_writable(address, SECTOR_SIZE),
            "erase outside otadata/slots at {address:#x}"
        );
        let start = address as usize;
        self.data.borrow_mut()[start..start + SECTOR_SIZE as usize].fill(0xFF);
        self.erases.borrow_mut().push(address);
        Ok(())
    }
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}

/// A complete ESP application image (app descriptor with `version`,
/// RISC-V code, checksum, appended SHA-256), as the devkit's fixture.
pub fn esp_app(code_len: usize, version: &str) -> Vec<u8> {
    let code_len = (code_len.max(4) + 3) & !3;
    let code_offset = HEADER_SIZE + SEGMENT_HEADER_SIZE + APP_DESC_SIZE + SEGMENT_HEADER_SIZE;
    let code_addr = 0x4200_0000 + code_offset as u32;
    let mut image = vec![0; HEADER_SIZE];
    image[0] = IMAGE_MAGIC;
    image[1] = 2;
    image[4..8].copy_from_slice(&code_addr.to_le_bytes());
    image[12..14].copy_from_slice(&EXPECTED_CHIP_ID.to_le_bytes());
    image[17..19].copy_from_slice(&u16::MAX.to_le_bytes());
    image[23] = 1;
    image.extend_from_slice(&0x4200_0020u32.to_le_bytes());
    image.extend_from_slice(&(APP_DESC_SIZE as u32).to_le_bytes());
    let mut descriptor = [0; APP_DESC_SIZE];
    descriptor[..4].copy_from_slice(&APP_DESC_MAGIC.to_le_bytes());
    descriptor[16..16 + version.len()].copy_from_slice(version.as_bytes());
    descriptor[178..180].copy_from_slice(&u16::MAX.to_le_bytes());
    descriptor[180] = 16;
    image.extend_from_slice(&descriptor);
    image.extend_from_slice(&code_addr.to_le_bytes());
    image.extend_from_slice(&(code_len as u32).to_le_bytes());
    image.extend_from_slice(&0x0000_006fu32.to_le_bytes()); // JAL x0,0
    for index in 1..code_len / 4 {
        // NOPs with a varying immediate so chunks differ.
        let nop = 0x0000_0013u32 | ((index as u32 & 0x7FF) << 20);
        image.extend_from_slice(&nop.to_le_bytes());
    }
    let mut cursor = HEADER_SIZE;
    let mut checksum = 0xEF;
    for _ in 0..image[1] {
        let len = u32_at(&image, cursor + 4) as usize;
        cursor += SEGMENT_HEADER_SIZE;
        for byte in &image[cursor..cursor + len] {
            checksum ^= byte;
        }
        cursor += len;
    }
    let end = (image.len() + 16) & !15;
    image.resize(end, 0);
    image[end - 1] = checksum;
    let digest = sha256(&image);
    image.extend_from_slice(&digest);
    image
}

pub fn signing_key(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&sha256(&[seed]).into()).unwrap()
}

pub fn trusted(key: &SigningKey) -> KeySet {
    let point = key.verifying_key().to_encoded_point(false);
    let xy: [u8; 64] = point.as_bytes()[1..].try_into().unwrap();
    KeySet::new(Box::leak(Box::new([TrustedKey::new(xy)])))
}

/// `imgtool sign --header-size 0x20 --pad-header` around `image`.
pub fn envelope(image: &[u8], version: (u8, u8, u16), key: &SigningKey) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&0x96F3_B83Du32.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // load_addr
    out.extend_from_slice(&0x20u16.to_le_bytes()); // hdr_size
    out.extend_from_slice(&0u16.to_le_bytes()); // protect_tlv_size
    out.extend_from_slice(&(image.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // flags
    out.extend_from_slice(&[version.0, version.1]);
    out.extend_from_slice(&version.2.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // build
    out.extend_from_slice(&0u32.to_le_bytes()); // pad
    out.extend_from_slice(image);
    let digest = sha256(&out);
    let signature: Signature = key.sign_prehash(&digest).unwrap();
    let der = signature.to_der();
    let key_hash = trusted(key).iter().next().unwrap().key_hash();
    let mut tlvs = Vec::new();
    for (kind, value) in [
        (0x10u16, &digest[..]),
        (0x01, &key_hash[..]),
        (0x22, der.as_bytes()),
    ] {
        tlvs.extend_from_slice(&kind.to_le_bytes());
        tlvs.extend_from_slice(&(value.len() as u16).to_le_bytes());
        tlvs.extend_from_slice(value);
    }
    out.extend_from_slice(&0x6907u16.to_le_bytes());
    out.extend_from_slice(&((tlvs.len() + 4) as u16).to_le_bytes());
    out.extend_from_slice(&tlvs);
    out
}

/// Running version 1.2.0 trusting `signing_key(1)`.
pub const RUNNING: u32 = 0x0102_0000;

pub fn policy(dev_build: bool) -> FwPolicy {
    FwPolicy {
        keys: trusted(&signing_key(1)),
        running_version: RUNNING,
        dev_build,
    }
}
