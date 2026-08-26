//! TLSR8258 composition of the shared sleepy-sensor lifecycle.

use core::mem::MaybeUninit;
#[cfg(feature = "retention-proof")]
use core::cell::UnsafeCell;

use sensor_sed_app::{NoOta, SensorApp, SensorSedParts};
use zigbee_mac::telink::TelinkMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_zcl::clusters::basic::PowerSource;

use tlsr8258_tb04::{leds::StatusLeds, resources::BoardResources};
use tlsr8258_tb04_product::sensor::{
    DATE_CODE, MANUFACTURER, MODEL, SENSOR_POLICY, SW_BUILD, SensorRgbStatus, SyntheticEnvironment,
    TelinkNoDiagnostics, TelinkSupervisor, USER_ACTIONS, fixed_battery, sensor_profile,
};
#[cfg(not(feature = "retention-proof"))]
use tlsr8258_tb04_product::sensor::TelinkSuspendWake;
#[cfg(feature = "retention-proof")]
use tlsr8258_tb04_product::sensor::{
    RetainedSensorApp, TelinkRetentionWake, fail_closed_retention_reset,
    initialize_retention_context, restore_retained_platform,
};
#[cfg(feature = "retention-proof")]
use tlsr8258_tb04_product::storage::SecurityStore;

// Preserve the IEEE address used by the hardware-proven runtime image so the
// existing journal and ZHA device identity remain valid across this refactor.
const DEVICE_EUI_OFFSET: u8 = 0x33;

fn failure(leds: &StatusLeds) -> ! {
    leds.green.write(false);
    leds.blue.write(false);
    leds.red.write(true);
    halted()
}

fn halted() -> ! {
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}

#[cfg(not(feature = "retention-proof"))]
pub fn run() -> ! {
    type Device = ZigbeeDevice<TelinkMac>;

    tlsr8258_hal::timer::init();
    let resources = match BoardResources::take() {
        Some(resources) => resources,
        None => loop {
            core::hint::spin_loop();
        },
    };
    let leds = resources.lighting.into_status_leds();
    if leds.init().is_err() {
        failure(&leds);
    }

    // Install the PC5/Zbit voltage guard before the flash token can enter
    // either persistent journal.
    let adc = match tlsr8258_hal::adc::Adc::new(
        resources.adc,
        tlsr8258_hal::flash::FlashGeometry::KiB512,
    ) {
        Ok(adc) => adc,
        Err(_) => failure(&leds),
    };
    if adc.install_flash_voltage_guard(resources.adc_pc5).is_err() {
        failure(&leds);
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mut mac = TelinkMac::with_extended_address(ieee_address);
    if mac.install_aes_engine(resources.aes).is_err() {
        failure(&leds);
    }

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    let mut profile = sensor_profile();
    let device = ZigbeeDevice::builder(mac)
        .power_mode(SENSOR_POLICY.power_mode())
        // SensorApp is the sole owner of all four-round parent poll windows.
        .automatic_polling(false)
        .manufacturer(MANUFACTURER)
        .model(MODEL)
        .date_code(DATE_CODE)
        .sw_build(SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    // This end device owns only the existing two-sector security journal.
    let (security_partition, _child_partition) =
        tlsr8258_tb04_product::storage::split_flash(resources.flash);
    let mut security_store = tlsr8258_tb04_product::storage::security_store(security_partition);

    // Identity reset must happen before ZigbeeNode borrows the device/store.
    if device
        .reset_security_state_if_identity_changed(&mut security_store, ieee_address)
        .is_err()
    {
        failure(&leds);
    }

    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let parts = SensorSedParts {
        wake: TelinkSuspendWake,
        status: SensorRgbStatus::new(leds),
        environment: SyntheticEnvironment::new(),
        battery: fixed_battery(),
        ota: NoOta,
        actions: USER_ACTIONS,
        supervisor: TelinkSupervisor,
        diagnostics: TelinkNoDiagnostics,
    };
    let mut app = match SensorApp::new(node, &SENSOR_POLICY, parts) {
        Ok(app) => app,
        // The RGB adapter has not changed the initialized red state yet.
        Err(_) => halted(),
    };

    // One caller-owned device, one shared lifecycle, and one executor root.
    // Fast waits remain Active; slow waits select the product's atomic
    // full-SRAM SUSPEND transaction. Retention is explicitly unsupported.
    tlsr8258_rt::block_on(app.run())
}

// -------------------------------------------------------------------------
// Explicit LOW32K proof composition
// -------------------------------------------------------------------------

#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[used]
#[unsafe(link_section = ".retention_image_marker")]
pub static TELINK_RETENTION_IMAGE: u32 = 0x4c33_324b;

#[cfg(feature = "retention-proof")]
struct RetainedCell<T>(UnsafeCell<MaybeUninit<T>>);

#[cfg(feature = "retention-proof")]
unsafe impl<T> Sync for RetainedCell<T> {}

#[cfg(feature = "retention-proof")]
impl<T> RetainedCell<T> {
    const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    unsafe fn slot(&'static self) -> &'static mut MaybeUninit<T> {
        unsafe { &mut *self.0.get() }
    }

    unsafe fn initialize(&'static self, value: T) -> &'static mut T {
        let slot = unsafe { self.slot() };
        slot.write(value)
    }

    unsafe fn get(&'static self) -> &'static mut T {
        unsafe { (&mut *self.0.get()).assume_init_mut() }
    }

    fn address(&'static self) -> u32 {
        self.0.get() as u32
    }
}

#[cfg(feature = "retention-proof")]
type RetainedDevice = ZigbeeDevice<TelinkMac>;
#[cfg(feature = "retention-proof")]
type RetainedProfile = tlsr8258_tb04_product::sensor::SensorProfile;

#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_DEVICE_STORAGE: RetainedCell<RetainedDevice> = RetainedCell::new();
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_PROFILE_STORAGE: RetainedCell<RetainedProfile> = RetainedCell::new();
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_SECURITY_STORAGE: RetainedCell<SecurityStore> = RetainedCell::new();
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_APP_STORAGE: RetainedCell<RetainedSensorApp> = RetainedCell::new();

#[cfg(feature = "retention-proof")]
const RETAINED_APP_MAGIC: u32 = 0x5345_4431; // "SED1"
#[cfg(feature = "retention-proof")]
const RETAINED_APP_VERSION: u32 = 1;
#[cfg(feature = "retention-proof")]
const STACK_GUARD_BYTE: u8 = 0xa5;

#[cfg(feature = "retention-proof")]
#[repr(C)]
#[derive(Clone, Copy)]
struct RetainedAppHeader {
    magic: u32,
    magic_inverse: u32,
    version: u32,
    device_address: u32,
    profile_address: u32,
    security_address: u32,
    app_address: u32,
    checksum: u32,
}

#[cfg(feature = "retention-proof")]
impl RetainedAppHeader {
    const fn invalid() -> Self {
        Self {
            magic: 0,
            magic_inverse: !0,
            version: 0,
            device_address: 0,
            profile_address: 0,
            security_address: 0,
            app_address: 0,
            checksum: 0,
        }
    }

    fn checksum(&self) -> u32 {
        let mut value = 0x6d5a_56a9u32;
        for word in [
            RETAINED_APP_MAGIC,
            !RETAINED_APP_MAGIC,
            self.version,
            self.device_address,
            self.profile_address,
            self.security_address,
            self.app_address,
        ] {
            value = value.rotate_left(7) ^ word;
            value = value.wrapping_mul(0x9e37_79b1);
        }
        value
    }

    fn valid(&self) -> bool {
        self.magic == RETAINED_APP_MAGIC
            && self.magic_inverse == !RETAINED_APP_MAGIC
            && self.version == RETAINED_APP_VERSION
            && self.device_address == TELINK_RETAINED_DEVICE_STORAGE.address()
            && self.profile_address == TELINK_RETAINED_PROFILE_STORAGE.address()
            && self.security_address == TELINK_RETAINED_SECURITY_STORAGE.address()
            && self.app_address == TELINK_RETAINED_APP_STORAGE.address()
            && self.checksum == self.checksum()
    }
}

#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_APP_HEADER: RetainedCell<RetainedAppHeader> = RetainedCell::new();

#[cfg(feature = "retention-proof")]
unsafe extern "C" {
    static _retention_stack_guard_start_: u8;
    static _retention_stack_guard_end_: u8;
}

#[cfg(feature = "retention-proof")]
fn stack_guard_range() -> (*mut u8, *mut u8) {
    (
        core::ptr::addr_of!(_retention_stack_guard_start_) as *mut u8,
        core::ptr::addr_of!(_retention_stack_guard_end_) as *mut u8,
    )
}

#[cfg(feature = "retention-proof")]
fn initialize_stack_guard() {
    let (mut cursor, end) = stack_guard_range();
    while cursor < end {
        unsafe { core::ptr::write_volatile(cursor, STACK_GUARD_BYTE) };
        cursor = unsafe { cursor.add(1) };
    }
}

#[cfg(feature = "retention-proof")]
fn stack_guard_is_intact() -> bool {
    let (mut cursor, end) = stack_guard_range();
    while cursor < end {
        if unsafe { core::ptr::read_volatile(cursor) } != STACK_GUARD_BYTE {
            return false;
        }
        cursor = unsafe { cursor.add(1) };
    }
    true
}

#[cfg(feature = "retention-proof")]
fn invalidate_retained_app_header() {
    unsafe {
        core::ptr::write(
            TELINK_RETAINED_APP_HEADER.0.get(),
            MaybeUninit::new(RetainedAppHeader::invalid()),
        );
    }
}

#[cfg(feature = "retention-proof")]
fn commit_retained_app_header() {
    use core::sync::atomic::{Ordering, compiler_fence};

    let mut header = RetainedAppHeader {
        magic: 0,
        magic_inverse: !RETAINED_APP_MAGIC,
        version: RETAINED_APP_VERSION,
        device_address: TELINK_RETAINED_DEVICE_STORAGE.address(),
        profile_address: TELINK_RETAINED_PROFILE_STORAGE.address(),
        security_address: TELINK_RETAINED_SECURITY_STORAGE.address(),
        app_address: TELINK_RETAINED_APP_STORAGE.address(),
        checksum: 0,
    };
    header.checksum = header.checksum();
    let stored = unsafe { TELINK_RETAINED_APP_HEADER.initialize(header) };
    compiler_fence(Ordering::SeqCst);
    unsafe { core::ptr::write_volatile(&mut stored.magic, RETAINED_APP_MAGIC) };
    compiler_fence(Ordering::SeqCst);
}

#[cfg(feature = "retention-proof")]
fn retained_app_valid() -> bool {
    let header = unsafe { &*TELINK_RETAINED_APP_HEADER.0.get().cast::<RetainedAppHeader>() };
    header.valid() && stack_guard_is_intact()
}

#[cfg(feature = "retention-proof")]
pub fn cold_run() -> ! {
    tlsr8258_hal::timer::init();
    invalidate_retained_app_header();
    initialize_stack_guard();

    let resources = match BoardResources::take() {
        Some(resources) => resources,
        None => loop {
            core::hint::spin_loop();
        },
    };
    let leds = resources.lighting.into_status_leds();
    if leds.init().is_err() {
        failure(&leds);
    }

    let adc = match tlsr8258_hal::adc::Adc::new(
        resources.adc,
        tlsr8258_hal::flash::FlashGeometry::KiB512,
    ) {
        Ok(adc) => adc,
        Err(_) => failure(&leds),
    };
    if adc.install_flash_voltage_guard(resources.adc_pc5).is_err() {
        failure(&leds);
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mut mac = TelinkMac::with_extended_address(ieee_address);
    if mac.install_aes_engine(resources.aes).is_err() {
        failure(&leds);
    }

    let profile = unsafe { TELINK_RETAINED_PROFILE_STORAGE.initialize(sensor_profile()) };
    let device = ZigbeeDevice::builder(mac)
        .power_mode(SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(MANUFACTURER)
        .model(MODEL)
        .date_code(DATE_CODE)
        .sw_build(SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(unsafe { TELINK_RETAINED_DEVICE_STORAGE.slot() });

    // The pointer is registered only after static placement and before the
    // node creates its long-lived mutable device borrow.
    initialize_retention_context(device.mac_mut());

    let (security_partition, _child_partition) =
        tlsr8258_tb04_product::storage::split_flash(resources.flash);
    let security_store = unsafe {
        TELINK_RETAINED_SECURITY_STORAGE.initialize(
            tlsr8258_tb04_product::storage::security_store(security_partition),
        )
    };
    if device
        .reset_security_state_if_identity_changed(security_store, ieee_address)
        .is_err()
    {
        failure(&leds);
    }

    let node = ZigbeeNode::new(device, security_store, profile);
    let parts = SensorSedParts {
        wake: TelinkRetentionWake,
        status: SensorRgbStatus::new(leds),
        environment: SyntheticEnvironment::new(),
        battery: fixed_battery(),
        ota: NoOta,
        actions: USER_ACTIONS,
        supervisor: TelinkSupervisor,
        diagnostics: TelinkNoDiagnostics,
    };
    let app = match SensorApp::new(node, &SENSOR_POLICY, parts) {
        Ok(app) => app,
        Err(_) => halted(),
    };
    unsafe {
        TELINK_RETAINED_APP_STORAGE.initialize(app);
    }
    commit_retained_app_header();
    run_fresh_retained_root(true)
}

#[cfg(feature = "retention-proof")]
pub fn retention_run() -> ! {
    if !retained_app_valid() {
        fail_closed_retention_reset();
    }
    if restore_retained_platform().is_err() {
        fail_closed_retention_reset();
    }
    if !stack_guard_is_intact() {
        fail_closed_retention_reset();
    }
    run_fresh_retained_root(false)
}

#[cfg(feature = "retention-proof")]
async fn retained_lifecycle_root(cold: bool) -> ! {
    let app = unsafe { TELINK_RETAINED_APP_STORAGE.get() };
    if cold && app.initialize().await.is_err() {
        halted();
    }
    loop {
        if app.step().await.is_err() {
            fail_closed_retention_reset();
        }
    }
}

/// One monomorphized executor root. Both reset entries create this future
/// afresh; neither can branch back to the pre-sleep async stack frame.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn TELINK_RETENTION_FRESH_ROOT(cold: bool) -> ! {
    tlsr8258_rt::block_on(retained_lifecycle_root(cold))
}

#[cfg(feature = "retention-proof")]
fn run_fresh_retained_root(cold: bool) -> ! {
    TELINK_RETENTION_FRESH_ROOT(cold)
}
