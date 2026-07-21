//! Safe application-side access to the resident Series 1 Gecko Bootloader.
//!
//! The bootloader jump tables are untrusted until every address used by this
//! module has been checked. Calls are not reentrant; the mutable handle keeps
//! normal Rust callers serialized, but interrupt handlers must not call these
//! APIs concurrently.

#![cfg_attr(not(target_arch = "arm"), allow(dead_code))]

use core::ffi::c_void;
#[cfg(target_arch = "arm")]
use core::sync::atomic::{AtomicBool, Ordering};

const FLASH_END: u32 = 0x0004_0000;
const FIRST_STAGE_END: u32 = 0x0000_0800;
const MAIN_STAGE_START: u32 = FIRST_STAGE_END;
const APPLICATION_START: u32 = 0x0000_4000;
const SRAM_START: u32 = 0x2000_0000;
const SRAM_END: u32 = 0x2000_8000;

const BARE_TABLE_POINTER_OFFSET: u32 = core::mem::offset_of!(RawBareBootTable, table) as u32;
const FIRST_TABLE_POINTER_ADDRESS: u32 = BARE_TABLE_POINTER_OFFSET;

const FIRST_MAGIC: u32 = 0xB007_10AD;
const MAIN_MAGIC: u32 = 0x5ECD_B007;
const FIRST_LAYOUT: u32 = 1;
const MAIN_LAYOUT: u32 = 2;

pub const CAPABILITY_GBL: u32 = 1 << 5;
pub const CAPABILITY_GBL_SIGNATURE: u32 = 1 << 6;
pub const CAPABILITY_GBL_ENCRYPTION: u32 = 1 << 7;
pub const CAPABILITY_STORAGE: u32 = 1 << 16;

const STORAGE_FUNCTIONS_VERSION_GSDK_4_5: u32 = 0x0000_0100;
const STORAGE_FUNCTIONS_VERSION_RESIDENT: u32 = 0x0001_0000;
const STORAGE_INFO_VERSION_WITH_INLINE_FLASH_INFO: u32 = 0x0002_0000;

const GECKO_OK: i32 = 0;
const GECKO_PARSE_CONTINUE: i32 = 0x0201;
const GECKO_PARSE_SUCCESS: i32 = 0x0203;

const VERIFICATION_CONTEXT_CAPACITY: usize = 512;

#[cfg(target_arch = "arm")]
static BOOTLOADER_TAKEN: AtomicBool = AtomicBool::new(false);

type InitFn = unsafe extern "C" fn() -> i32;
type VerifyApplicationFn = unsafe extern "C" fn(u32) -> bool;
type InitParserFn = unsafe extern "C" fn(*mut c_void, usize) -> i32;
type ParseBufferFn = unsafe extern "C" fn(*mut c_void, *const c_void, *mut u8, usize) -> i32;
type ParseImageInfoFn =
    unsafe extern "C" fn(*mut c_void, *mut u8, usize, *mut c_void, *mut u32) -> i32;
type ParserContextSizeFn = unsafe extern "C" fn() -> u32;
type RemainingUpgradesFn = unsafe extern "C" fn() -> u32;
type PeripheralListFn = unsafe extern "C" fn(*mut u32, *mut u32);
type UpgradeLocationFn = unsafe extern "C" fn() -> u32;

type StorageGetInfoFn = unsafe extern "C" fn(*mut RawStorageInformation);
type StorageGetSlotFn = unsafe extern "C" fn(u32, *mut StorageSlot) -> i32;
type StorageReadFn = unsafe extern "C" fn(u32, u32, *mut u8, usize) -> i32;
type StorageWriteFn = unsafe extern "C" fn(u32, u32, *mut u8, usize) -> i32;
type StorageEraseFn = unsafe extern "C" fn(u32) -> i32;
type StorageSetImagesFn = unsafe extern "C" fn(*mut i32, usize) -> i32;
type StorageGetImagesFn = unsafe extern "C" fn(*mut i32, usize) -> i32;
type StorageAppendImageFn = unsafe extern "C" fn(i32) -> i32;
type StorageInitParseFn = unsafe extern "C" fn(u32, *mut c_void, usize) -> i32;
type ParserCallback = unsafe extern "C" fn(u32, *mut u8, usize, *mut c_void);
type StorageVerifyFn = unsafe extern "C" fn(*mut c_void, Option<ParserCallback>) -> i32;
type StorageGetImageInfoFn = unsafe extern "C" fn(*mut c_void, *mut c_void, *mut u32) -> i32;
type StorageIsBusyFn = unsafe extern "C" fn() -> bool;
type StorageReadRawFn = unsafe extern "C" fn(u32, *mut u8, usize) -> i32;
type StorageWriteRawFn = unsafe extern "C" fn(u32, *mut u8, usize) -> i32;
type StorageEraseRawFn = unsafe extern "C" fn(u32, usize) -> i32;
type StorageDmaChannelFn = unsafe extern "C" fn() -> i32;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct BootloaderHeader {
    image_type: u32,
    layout: u32,
    version: u32,
}

/// Exact 32-bit C ABI layout of `BareBootTable_t`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RawBareBootTable {
    stack_top: u32,
    reset_vector: u32,
    reserved0: [u32; 5],
    reserved1: [u32; 3],
    table: u32,
    reserved2: [u32; 2],
    signature: u32,
}

/// Exact 32-bit C ABI layout of `FirstBootloaderTable_t`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RawFirstTable {
    header: BootloaderHeader,
    main_bootloader: u32,
    upgrade_location: u32,
}

/// Exact 32-bit C ABI layout of `MainBootloaderTable_t`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RawMainTable {
    header: BootloaderHeader,
    size: u32,
    start_of_app_space: u32,
    end_of_app_space: u32,
    capabilities: u32,
    init: u32,
    deinit: u32,
    verify_application: u32,
    init_parser: u32,
    parse_buffer: u32,
    storage: u32,
    parse_image_info: u32,
    parser_context_size: u32,
    remaining_application_upgrades: u32,
    get_peripheral_list: u32,
    get_upgrade_location: u32,
}

/// Exact 32-bit C ABI layout of `BootloaderStorageFunctions_t`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RawStorageFunctions {
    version: u32,
    get_info: u32,
    get_slot_info: u32,
    read: u32,
    write: u32,
    erase: u32,
    set_images_to_bootload: u32,
    get_images_to_bootload: u32,
    append_image_to_bootload_list: u32,
    init_parse_image: u32,
    verify_image: u32,
    get_image_info: u32,
    is_busy: u32,
    read_raw: u32,
    write_raw: u32,
    erase_raw: u32,
    get_dma_channel: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct RawStorageImplementationInformation {
    version: u16,
    capabilities_mask: u16,
    page_erase_ms: u32,
    part_erase_ms: u32,
    page_size: u32,
    part_size: u32,
    part_description: u32,
    word_size_bytes: u8,
    reserved: [u8; 3],
    part_type: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
struct RawStorageInformation {
    version: u32,
    capabilities: u32,
    storage_type: u32,
    num_storage_slots: u32,
    info: u32,
    flash_info: RawStorageImplementationInformation,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StorageSlot {
    pub address: u32,
    pub length: u32,
}

#[repr(C, align(8))]
struct VerificationContext([u8; VERIFICATION_CONTEXT_CAPACITY]);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageType {
    SpiFlash,
    InternalFlash,
    Custom,
    Unknown(u32),
}

impl StorageType {
    fn from_raw(value: u32) -> Self {
        match value {
            0 => Self::SpiFlash,
            1 => Self::InternalFlash,
            2 => Self::Custom,
            other => Self::Unknown(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootloaderInfo {
    pub version: u32,
    pub capabilities: u32,
    pub main_stage_size: u32,
    pub application_start: u32,
    pub application_end: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageImplementationInfo {
    pub version: u16,
    pub capabilities: u16,
    pub page_erase_ms: u32,
    pub part_erase_ms: u32,
    pub page_size: u32,
    pub part_size: u32,
    /// Address of the bootloader-owned, NUL-terminated description.
    ///
    /// It is intentionally not exposed as a Rust string because the ABI does
    /// not provide a length with which to validate it.
    pub part_description_address: u32,
    pub word_size_bytes: u8,
    pub part_type: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageInfo {
    pub version: u32,
    pub capabilities: u32,
    pub storage_type: StorageType,
    pub num_slots: u32,
    pub implementation: StorageImplementationInfo,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Function {
    Init,
    Deinit,
    ParserContextSize,
    StorageGetInfo,
    StorageGetSlotInfo,
    StorageRead,
    StorageWrite,
    StorageErase,
    StorageSetImages,
    StorageGetImages,
    StorageAppendImage,
    StorageInitParse,
    StorageVerify,
    StorageGetImageInfo,
    StorageIsBusy,
    StorageReadRaw,
    StorageWriteRaw,
    StorageEraseRaw,
    StorageGetDmaChannel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    UnsupportedTarget,
    InvalidFirstTablePointer(u32),
    InvalidFirstMagic(u32),
    UnsupportedFirstLayout(u32),
    InvalidMainVector(u32),
    InvalidMainTablePointer(u32),
    InvalidMainMagic(u32),
    UnsupportedMainLayout(u32),
    InvalidMainStageSize(u32),
    InvalidApplicationRange { start: u32, end: u32 },
    MissingCapability(u32),
    InvalidStorageTablePointer(u32),
    UnsupportedStorageTableVersion(u32),
    InvalidFunctionPointer { function: Function, address: u32 },
    AlreadyTaken,
    NotInitialized,
    Gecko(i32),
    SlotOutOfBounds,
    InvalidStorageInformation,
    ParserContextSize { required: u32, capacity: u32 },
    VerificationDidNotFinish,
}

#[derive(Debug, Clone, Copy)]
struct ValidatedTables {
    main: RawMainTable,
    storage: RawStorageFunctions,
}

/// Validated handle to the resident Gecko Bootloader application ABI.
pub struct Bootloader {
    tables: ValidatedTables,
    initialized: bool,
}

impl Bootloader {
    /// Discover and validate the resident first-stage, main-stage, and storage
    /// jump tables. On a non-ARM host this never touches address zero.
    pub fn discover() -> Result<Self, Error> {
        #[cfg(target_arch = "arm")]
        {
            BOOTLOADER_TAKEN
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .map_err(|_| Error::AlreadyTaken)?;
            let tables = match unsafe { discover_target() } {
                Ok(tables) => tables,
                Err(error) => {
                    BOOTLOADER_TAKEN.store(false, Ordering::Release);
                    return Err(error);
                }
            };
            Ok(Self {
                tables,
                initialized: false,
            })
        }
        #[cfg(not(target_arch = "arm"))]
        {
            Err(Error::UnsupportedTarget)
        }
    }

    pub const fn info(&self) -> BootloaderInfo {
        BootloaderInfo {
            version: self.tables.main.header.version,
            capabilities: self.tables.main.capabilities,
            main_stage_size: self.tables.main.size,
            application_start: self.tables.main.start_of_app_space,
            application_end: self.tables.main.end_of_app_space,
        }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        if self.initialized {
            return Ok(());
        }
        result(unsafe { abi::call_init(self.tables.main.init) })?;
        self.initialized = true;
        Ok(())
    }

    pub fn deinit(&mut self) -> Result<(), Error> {
        if !self.initialized {
            return Ok(());
        }
        result(unsafe { abi::call_init(self.tables.main.deinit) })?;
        self.initialized = false;
        Ok(())
    }

    pub fn storage_info(&mut self) -> Result<StorageInfo, Error> {
        self.require_initialized()?;
        let mut raw = RawStorageInformation::default();
        unsafe { abi::call_storage_get_info(self.tables.storage.get_info, &mut raw) };

        let implementation = if raw.version >= STORAGE_INFO_VERSION_WITH_INLINE_FLASH_INFO {
            raw.flash_info
        } else {
            unsafe { read_storage_implementation(raw.info)? }
        };
        if implementation.page_size == 0
            || implementation.part_size == 0
            || implementation.page_size > implementation.part_size
            || implementation.word_size_bytes == 0
        {
            return Err(Error::InvalidStorageInformation);
        }

        Ok(StorageInfo {
            version: raw.version,
            capabilities: raw.capabilities,
            storage_type: StorageType::from_raw(raw.storage_type),
            num_slots: raw.num_storage_slots,
            implementation: StorageImplementationInfo {
                version: implementation.version,
                capabilities: implementation.capabilities_mask,
                page_erase_ms: implementation.page_erase_ms,
                part_erase_ms: implementation.part_erase_ms,
                page_size: implementation.page_size,
                part_size: implementation.part_size,
                part_description_address: implementation.part_description,
                word_size_bytes: implementation.word_size_bytes,
                part_type: implementation.part_type,
            },
        })
    }

    pub fn storage_slot(&mut self, slot_id: u32) -> Result<StorageSlot, Error> {
        self.require_initialized()?;
        let mut slot = StorageSlot::default();
        result(unsafe {
            abi::call_storage_get_slot(self.tables.storage.get_slot_info, slot_id, &mut slot)
        })?;
        if slot.address.checked_add(slot.length).is_none() {
            return Err(Error::InvalidStorageInformation);
        }
        Ok(slot)
    }

    pub fn read_slot(&mut self, slot_id: u32, offset: u32, buffer: &mut [u8]) -> Result<(), Error> {
        let slot = self.storage_slot(slot_id)?;
        check_slot_range(slot, offset, buffer.len())?;
        if buffer.is_empty() {
            return Ok(());
        }
        result(unsafe {
            abi::call_storage_read(
                self.tables.storage.read,
                slot_id,
                offset,
                buffer.as_mut_ptr(),
                buffer.len(),
            )
        })
    }

    /// Program an already-erased slot range.
    ///
    /// The resident SPI-flash storage accepts byte-granular writes and handles
    /// page boundaries internally.
    pub fn write_slot(&mut self, slot_id: u32, offset: u32, buffer: &[u8]) -> Result<(), Error> {
        let slot = self.storage_slot(slot_id)?;
        check_slot_range(slot, offset, buffer.len())?;
        if buffer.is_empty() {
            return Ok(());
        }
        result(unsafe {
            abi::call_storage_write(
                self.tables.storage.write,
                slot_id,
                offset,
                buffer.as_ptr().cast_mut(),
                buffer.len(),
            )
        })
    }

    pub fn erase_slot(&mut self, slot_id: u32) -> Result<(), Error> {
        let _ = self.storage_slot(slot_id)?;
        result(unsafe { abi::call_storage_erase(self.tables.storage.erase, slot_id) })
    }

    /// Replace the prioritized bootload list.
    pub fn set_bootload_list(&mut self, slot_ids: &mut [i32]) -> Result<(), Error> {
        self.require_initialized()?;
        if slot_ids.is_empty() {
            return self.clear_bootload_list();
        }
        for &slot_id in slot_ids.iter() {
            if slot_id < 0 {
                return Err(Error::SlotOutOfBounds);
            }
            let _ = self.storage_slot(slot_id as u32)?;
        }
        result(unsafe {
            abi::call_storage_set_images(
                self.tables.storage.set_images_to_bootload,
                slot_ids.as_mut_ptr(),
                slot_ids.len(),
            )
        })
    }

    /// Clear the bootload selection using Gecko's `-1` sentinel.
    pub fn clear_bootload_list(&mut self) -> Result<(), Error> {
        self.require_initialized()?;
        let mut empty = -1i32;
        result(unsafe {
            abi::call_storage_set_images(self.tables.storage.set_images_to_bootload, &mut empty, 1)
        })
    }

    /// Verify the complete GBL in a storage slot.
    pub fn verify_gbl_slot(&mut self, slot_id: u32) -> Result<(), Error> {
        let slot = self.storage_slot(slot_id)?;
        let required =
            unsafe { abi::call_parser_context_size(self.tables.main.parser_context_size) };
        if required == 0 || required as usize > VERIFICATION_CONTEXT_CAPACITY {
            return Err(Error::ParserContextSize {
                required,
                capacity: VERIFICATION_CONTEXT_CAPACITY as u32,
            });
        }

        let mut context = VerificationContext([0; VERIFICATION_CONTEXT_CAPACITY]);
        result(unsafe {
            abi::call_storage_init_parse(
                self.tables.storage.init_parse_image,
                slot_id,
                context.0.as_mut_ptr().cast(),
                VERIFICATION_CONTEXT_CAPACITY,
            )
        })?;

        // A valid parser must consume at least one slot byte per continuation.
        // The deliberately loose bound catches a broken resident table without
        // assuming the bootloader's internal read-buffer size.
        for _ in 0..=slot.length {
            let status = unsafe {
                abi::call_storage_verify(
                    self.tables.storage.verify_image,
                    context.0.as_mut_ptr().cast(),
                )
            };
            match status {
                GECKO_PARSE_CONTINUE => {}
                GECKO_PARSE_SUCCESS | GECKO_OK => return Ok(()),
                code => return Err(Error::Gecko(code)),
            }
        }
        Err(Error::VerificationDidNotFinish)
    }

    /// Write the Gecko bootload reset reason and request a full system reset.
    ///
    /// This function never returns. The selected slot must have been set and
    /// verified by the caller before invoking it.
    pub fn reboot_and_install(self) -> ! {
        #[cfg(target_arch = "arm")]
        unsafe {
            abi::reboot_and_install()
        }
        #[cfg(not(target_arch = "arm"))]
        panic!("Gecko bootloader reset is only available on ARM")
    }

    fn require_initialized(&self) -> Result<(), Error> {
        if self.initialized {
            Ok(())
        } else {
            Err(Error::NotInitialized)
        }
    }
}

impl Drop for Bootloader {
    fn drop(&mut self) {
        let _ = self.deinit();
        #[cfg(target_arch = "arm")]
        BOOTLOADER_TAKEN.store(false, Ordering::Release);
    }
}

fn result(code: i32) -> Result<(), Error> {
    if code == GECKO_OK {
        Ok(())
    } else {
        Err(Error::Gecko(code))
    }
}

fn check_slot_range(slot: StorageSlot, offset: u32, length: usize) -> Result<(), Error> {
    let length = u32::try_from(length).map_err(|_| Error::SlotOutOfBounds)?;
    let end = offset.checked_add(length).ok_or(Error::SlotOutOfBounds)?;
    if end <= slot.length {
        Ok(())
    } else {
        Err(Error::SlotOutOfBounds)
    }
}

fn address_range_valid(address: u32, size: usize, start: u32, end: u32) -> bool {
    address % 4 == 0
        && u32::try_from(size)
            .ok()
            .and_then(|size| address.checked_add(size))
            .is_some_and(|range_end| address >= start && range_end <= end)
}

fn thumb_function_valid(address: u32) -> bool {
    address & 1 != 0 && (MAIN_STAGE_START..APPLICATION_START).contains(&(address & !1))
}

fn validate_function(function: Function, address: u32) -> Result<(), Error> {
    if thumb_function_valid(address) {
        Ok(())
    } else {
        Err(Error::InvalidFunctionPointer { function, address })
    }
}

fn validate_tables(
    first_address: u32,
    first: RawFirstTable,
    main_address: u32,
    main: RawMainTable,
    storage_address: u32,
    storage: RawStorageFunctions,
) -> Result<ValidatedTables, Error> {
    if !address_range_valid(
        first_address,
        core::mem::size_of::<RawFirstTable>(),
        0,
        FIRST_STAGE_END,
    ) {
        return Err(Error::InvalidFirstTablePointer(first_address));
    }
    if first.header.image_type != FIRST_MAGIC {
        return Err(Error::InvalidFirstMagic(first.header.image_type));
    }
    if first.header.layout != FIRST_LAYOUT {
        return Err(Error::UnsupportedFirstLayout(first.header.layout));
    }
    if first.main_bootloader != MAIN_STAGE_START {
        return Err(Error::InvalidMainVector(first.main_bootloader));
    }
    if !address_range_valid(
        main_address,
        core::mem::size_of::<RawMainTable>(),
        MAIN_STAGE_START,
        APPLICATION_START,
    ) {
        return Err(Error::InvalidMainTablePointer(main_address));
    }
    if main.header.image_type != MAIN_MAGIC {
        return Err(Error::InvalidMainMagic(main.header.image_type));
    }
    if main.header.layout != MAIN_LAYOUT {
        return Err(Error::UnsupportedMainLayout(main.header.layout));
    }
    if main.size == 0
        || first
            .main_bootloader
            .checked_add(main.size)
            .is_none_or(|end| end > APPLICATION_START)
    {
        return Err(Error::InvalidMainStageSize(main.size));
    }
    if main.start_of_app_space != APPLICATION_START
        || main.end_of_app_space != FLASH_END
        || main.start_of_app_space >= main.end_of_app_space
    {
        return Err(Error::InvalidApplicationRange {
            start: main.start_of_app_space,
            end: main.end_of_app_space,
        });
    }
    for capability in [CAPABILITY_STORAGE, CAPABILITY_GBL] {
        if main.capabilities & capability == 0 {
            return Err(Error::MissingCapability(capability));
        }
    }
    if main.storage != storage_address
        || !address_range_valid(
            storage_address,
            core::mem::size_of::<RawStorageFunctions>(),
            MAIN_STAGE_START,
            APPLICATION_START,
        )
    {
        return Err(Error::InvalidStorageTablePointer(storage_address));
    }
    if !matches!(
        storage.version,
        STORAGE_FUNCTIONS_VERSION_GSDK_4_5 | STORAGE_FUNCTIONS_VERSION_RESIDENT
    ) {
        return Err(Error::UnsupportedStorageTableVersion(storage.version));
    }

    for (function, address) in [
        (Function::Init, main.init),
        (Function::Deinit, main.deinit),
        (Function::ParserContextSize, main.parser_context_size),
        (Function::StorageGetInfo, storage.get_info),
        (Function::StorageGetSlotInfo, storage.get_slot_info),
        (Function::StorageRead, storage.read),
        (Function::StorageWrite, storage.write),
        (Function::StorageErase, storage.erase),
        (Function::StorageSetImages, storage.set_images_to_bootload),
        (Function::StorageGetImages, storage.get_images_to_bootload),
        (
            Function::StorageAppendImage,
            storage.append_image_to_bootload_list,
        ),
        (Function::StorageInitParse, storage.init_parse_image),
        (Function::StorageVerify, storage.verify_image),
        (Function::StorageGetImageInfo, storage.get_image_info),
        (Function::StorageIsBusy, storage.is_busy),
        (Function::StorageReadRaw, storage.read_raw),
        (Function::StorageWriteRaw, storage.write_raw),
        (Function::StorageEraseRaw, storage.erase_raw),
        (Function::StorageGetDmaChannel, storage.get_dma_channel),
    ] {
        validate_function(function, address)?;
    }

    Ok(ValidatedTables { main, storage })
}

#[cfg(target_arch = "arm")]
unsafe fn discover_target() -> Result<ValidatedTables, Error> {
    let first_address = unsafe { abi::read_u32(FIRST_TABLE_POINTER_ADDRESS) };
    if !address_range_valid(
        first_address,
        core::mem::size_of::<RawFirstTable>(),
        0,
        FIRST_STAGE_END,
    ) {
        return Err(Error::InvalidFirstTablePointer(first_address));
    }
    let first = unsafe { abi::read_value::<RawFirstTable>(first_address) };
    if first.main_bootloader != MAIN_STAGE_START {
        return Err(Error::InvalidMainVector(first.main_bootloader));
    }

    let main_pointer_address = first
        .main_bootloader
        .checked_add(BARE_TABLE_POINTER_OFFSET)
        .ok_or(Error::InvalidMainVector(first.main_bootloader))?;
    let main_address = unsafe { abi::read_u32(main_pointer_address) };
    if !address_range_valid(
        main_address,
        core::mem::size_of::<RawMainTable>(),
        MAIN_STAGE_START,
        APPLICATION_START,
    ) {
        return Err(Error::InvalidMainTablePointer(main_address));
    }
    let main = unsafe { abi::read_value::<RawMainTable>(main_address) };
    let storage_address = main.storage;
    if !address_range_valid(
        storage_address,
        core::mem::size_of::<RawStorageFunctions>(),
        MAIN_STAGE_START,
        APPLICATION_START,
    ) {
        return Err(Error::InvalidStorageTablePointer(storage_address));
    }
    let storage = unsafe { abi::read_value::<RawStorageFunctions>(storage_address) };
    validate_tables(
        first_address,
        first,
        main_address,
        main,
        storage_address,
        storage,
    )
}

unsafe fn read_storage_implementation(
    address: u32,
) -> Result<RawStorageImplementationInformation, Error> {
    let size = core::mem::size_of::<RawStorageImplementationInformation>();
    if !address_range_valid(address, size, MAIN_STAGE_START, APPLICATION_START)
        && !address_range_valid(address, size, SRAM_START, SRAM_END)
    {
        return Err(Error::InvalidStorageInformation);
    }
    #[cfg(target_arch = "arm")]
    {
        Ok(unsafe { abi::read_value(address) })
    }
    #[cfg(not(target_arch = "arm"))]
    {
        let _ = address;
        Err(Error::UnsupportedTarget)
    }
}

mod abi {
    use super::*;

    pub(super) unsafe fn read_u32(address: u32) -> u32 {
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    pub(super) unsafe fn read_value<T: Copy>(address: u32) -> T {
        unsafe { core::ptr::read_volatile(address as *const T) }
    }

    pub(super) unsafe fn call_init(address: u32) -> i32 {
        let function: InitFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function() }
    }

    pub(super) unsafe fn call_parser_context_size(address: u32) -> u32 {
        let function: ParserContextSizeFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function() }
    }

    pub(super) unsafe fn call_storage_get_info(address: u32, info: &mut RawStorageInformation) {
        let function: StorageGetInfoFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(info) }
    }

    pub(super) unsafe fn call_storage_get_slot(
        address: u32,
        slot_id: u32,
        slot: &mut StorageSlot,
    ) -> i32 {
        let function: StorageGetSlotFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slot_id, slot) }
    }

    pub(super) unsafe fn call_storage_read(
        address: u32,
        slot_id: u32,
        offset: u32,
        buffer: *mut u8,
        length: usize,
    ) -> i32 {
        let function: StorageReadFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slot_id, offset, buffer, length) }
    }

    pub(super) unsafe fn call_storage_write(
        address: u32,
        slot_id: u32,
        offset: u32,
        buffer: *mut u8,
        length: usize,
    ) -> i32 {
        let function: StorageWriteFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slot_id, offset, buffer, length) }
    }

    pub(super) unsafe fn call_storage_erase(address: u32, slot_id: u32) -> i32 {
        let function: StorageEraseFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slot_id) }
    }

    pub(super) unsafe fn call_storage_set_images(
        address: u32,
        slots: *mut i32,
        length: usize,
    ) -> i32 {
        let function: StorageSetImagesFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slots, length) }
    }

    pub(super) unsafe fn call_storage_init_parse(
        address: u32,
        slot_id: u32,
        context: *mut c_void,
        context_size: usize,
    ) -> i32 {
        let function: StorageInitParseFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(slot_id, context, context_size) }
    }

    pub(super) unsafe fn call_storage_verify(address: u32, context: *mut c_void) -> i32 {
        let function: StorageVerifyFn = unsafe { core::mem::transmute(address as usize) };
        unsafe { function(context, None) }
    }

    #[cfg(target_arch = "arm")]
    pub(super) unsafe fn reboot_and_install() -> ! {
        const RESET_CAUSE: *mut u32 = SRAM_START as *mut u32;
        const RESET_REASON_BOOTLOAD: u32 = 0xF00F_0202;
        const RMU_CTRL: *mut u32 = 0x400E_5000 as *mut u32;
        const RMU_CMD: *mut u32 = 0x400E_5008 as *mut u32;
        const RMU_SYSRMODE_MASK: u32 = 0x700;
        const RMU_SYSRMODE_FULL: u32 = 0x400;
        const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;
        const AIRCR_VECTKEY: u32 = 0x05FA << 16;
        const AIRCR_PRIGROUP_MASK: u32 = 0x700;
        const AIRCR_SYSRESETREQ: u32 = 1 << 2;

        unsafe {
            core::ptr::write_volatile(RESET_CAUSE, RESET_REASON_BOOTLOAD);
            core::ptr::write_volatile(RMU_CMD, 1);
            let control = core::ptr::read_volatile(RMU_CTRL);
            core::ptr::write_volatile(RMU_CTRL, (control & !RMU_SYSRMODE_MASK) | RMU_SYSRMODE_FULL);
            core::arch::asm!("dsb 0xF", options(nostack, preserves_flags));
            let aircr = core::ptr::read_volatile(SCB_AIRCR);
            core::ptr::write_volatile(
                SCB_AIRCR,
                AIRCR_VECTKEY | (aircr & AIRCR_PRIGROUP_MASK) | AIRCR_SYSRESETREQ,
            );
            core::arch::asm!("dsb 0xF", options(nostack, preserves_flags));
        }
        loop {
            core::hint::spin_loop();
        }
    }
}

// These aliases document every function-pointer field in the GSDK structs and
// make accidental ABI drift visible to the compiler even when a field is not
// called by this milestone.
const _: Option<VerifyApplicationFn> = None;
const _: Option<InitParserFn> = None;
const _: Option<ParseBufferFn> = None;
const _: Option<ParseImageInfoFn> = None;
const _: Option<RemainingUpgradesFn> = None;
const _: Option<PeripheralListFn> = None;
const _: Option<UpgradeLocationFn> = None;
const _: Option<StorageGetImagesFn> = None;
const _: Option<StorageAppendImageFn> = None;
const _: Option<StorageGetImageInfoFn> = None;
const _: Option<StorageIsBusyFn> = None;
const _: Option<StorageReadRawFn> = None;
const _: Option<StorageWriteRawFn> = None;
const _: Option<StorageEraseRawFn> = None;
const _: Option<StorageDmaChannelFn> = None;

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_tables() -> (
        u32,
        RawFirstTable,
        u32,
        RawMainTable,
        u32,
        RawStorageFunctions,
    ) {
        let function = 0x0000_1001;
        let first_address = 0x05E4;
        let main_address = 0x3864;
        let storage_address = 0x3A24;
        let first = RawFirstTable {
            header: BootloaderHeader {
                image_type: FIRST_MAGIC,
                layout: FIRST_LAYOUT,
                version: 0x0000_0001,
            },
            main_bootloader: MAIN_STAGE_START,
            upgrade_location: 0,
        };
        let main = RawMainTable {
            header: BootloaderHeader {
                image_type: MAIN_MAGIC,
                layout: MAIN_LAYOUT,
                version: 0x0204_0002,
            },
            size: 0x3200,
            start_of_app_space: APPLICATION_START,
            end_of_app_space: FLASH_END,
            capabilities: 0x0001_00F0,
            init: function,
            deinit: function,
            verify_application: function,
            init_parser: function,
            parse_buffer: function,
            storage: storage_address,
            parse_image_info: function,
            parser_context_size: function,
            remaining_application_upgrades: 0,
            get_peripheral_list: 0,
            get_upgrade_location: function,
        };
        let storage = RawStorageFunctions {
            version: STORAGE_FUNCTIONS_VERSION_RESIDENT,
            get_info: function,
            get_slot_info: function,
            read: function,
            write: function,
            erase: function,
            set_images_to_bootload: function,
            get_images_to_bootload: function,
            append_image_to_bootload_list: function,
            init_parse_image: function,
            verify_image: function,
            get_image_info: function,
            is_busy: function,
            read_raw: function,
            write_raw: function,
            erase_raw: function,
            get_dma_channel: function,
        };
        (
            first_address,
            first,
            main_address,
            main,
            storage_address,
            storage,
        )
    }

    #[test]
    fn abi_struct_sizes_match_gsdk_series_1() {
        assert_eq!(core::mem::size_of::<RawBareBootTable>(), 56);
        assert_eq!(BARE_TABLE_POINTER_OFFSET, 40);
        assert_eq!(core::mem::size_of::<RawFirstTable>(), 20);
        assert_eq!(core::mem::size_of::<RawMainTable>(), 72);
        assert_eq!(core::mem::size_of::<RawStorageFunctions>(), 68);
        assert_eq!(
            core::mem::size_of::<RawStorageImplementationInformation>(),
            32
        );
        assert_eq!(core::mem::size_of::<RawStorageInformation>(), 52);
        assert_eq!(core::mem::align_of::<VerificationContext>(), 8);
    }

    #[test]
    fn observed_resident_tables_validate_without_dereferencing_flash() {
        let (fa, first, ma, main, sa, storage) = valid_tables();
        let tables = validate_tables(fa, first, ma, main, sa, storage).unwrap();
        assert_eq!(tables.main.header.version, 0x0204_0002);
        assert_eq!(tables.main.capabilities, 0x0001_00F0);
    }

    #[test]
    fn bad_magic_and_layout_are_rejected() {
        let (fa, mut first, ma, main, sa, storage) = valid_tables();
        first.header.image_type = 0;
        assert_eq!(
            validate_tables(fa, first, ma, main, sa, storage).unwrap_err(),
            Error::InvalidFirstMagic(0)
        );

        let (fa, first, ma, mut main, sa, storage) = valid_tables();
        main.header.layout = 3;
        assert_eq!(
            validate_tables(fa, first, ma, main, sa, storage).unwrap_err(),
            Error::UnsupportedMainLayout(3)
        );
    }

    #[test]
    fn storage_capability_and_pointer_are_required() {
        let (fa, first, ma, mut main, sa, storage) = valid_tables();
        main.capabilities &= !CAPABILITY_STORAGE;
        assert_eq!(
            validate_tables(fa, first, ma, main, sa, storage).unwrap_err(),
            Error::MissingCapability(CAPABILITY_STORAGE)
        );

        let (fa, first, ma, mut main, sa, storage) = valid_tables();
        main.storage = 0x4000;
        assert_eq!(
            validate_tables(fa, first, ma, main, sa, storage).unwrap_err(),
            Error::InvalidStorageTablePointer(sa)
        );
    }

    #[test]
    fn non_thumb_or_application_function_is_rejected() {
        let (fa, first, ma, main, sa, mut storage) = valid_tables();
        storage.read = 0x1000;
        assert_eq!(
            validate_tables(fa, first, ma, main, sa, storage).unwrap_err(),
            Error::InvalidFunctionPointer {
                function: Function::StorageRead,
                address: 0x1000,
            }
        );

        let (fa, first, ma, main, sa, mut storage) = valid_tables();
        storage.read = APPLICATION_START | 1;
        assert!(matches!(
            validate_tables(fa, first, ma, main, sa, storage),
            Err(Error::InvalidFunctionPointer {
                function: Function::StorageRead,
                ..
            })
        ));
    }

    #[test]
    fn slot_ranges_are_overflow_safe() {
        let slot = StorageSlot {
            address: 0,
            length: 16,
        };
        assert_eq!(check_slot_range(slot, 12, 4), Ok(()));
        assert_eq!(check_slot_range(slot, 13, 4), Err(Error::SlotOutOfBounds));
        assert_eq!(
            check_slot_range(slot, u32::MAX, 2),
            Err(Error::SlotOutOfBounds)
        );
    }

    #[test]
    fn host_discovery_never_reads_address_zero() {
        assert!(matches!(
            Bootloader::discover(),
            Err(Error::UnsupportedTarget)
        ));
    }
}
