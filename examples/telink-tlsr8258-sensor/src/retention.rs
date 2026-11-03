//! LOW32K reset-on-wake composition: the same sensor, built once into
//! retained cells and re-entered through a fresh root after every wake.

use sensor_sed_app::SensorApp;
use tlsr8258_rt::retention::{RetainedCell, RetainedRoot};
use tlsr8258_sensor_app::retention as platform;
use tlsr8258_tb04_product::boot::{boot, failure, halted};
use tlsr8258_tb04_product::sensor::{
    DEVICE_EUI_OFFSET, SENSOR_POLICY, SensorProfile, SensorRgbStatus, sensor_profile,
};
use tlsr8258_tb04_product::storage::{SecurityStore, security_store, split_flash};
use zigbee_mac::telink::TelinkMac;
use zigbee_runtime::node::ZigbeeNode;

use crate::app::{Device, Parts, build_device, parts};

type App = SensorApp<'static, TelinkMac, SecurityStore, SensorProfile, Parts>;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_DEVICE_STORAGE: RetainedCell<Device> = RetainedCell::new();
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_PROFILE_STORAGE: RetainedCell<SensorProfile> = RetainedCell::new();
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_SECURITY_STORAGE: RetainedCell<SecurityStore> = RetainedCell::new();
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_APP_STORAGE: RetainedCell<App> = RetainedCell::new();
#[unsafe(no_mangle)]
#[unsafe(link_section = ".retained.app")]
static TELINK_RETAINED_APP_HEADER: RetainedRoot<4> = RetainedRoot::new();

fn retained_cells() -> [u32; 4] {
    [
        TELINK_RETAINED_DEVICE_STORAGE.address(),
        TELINK_RETAINED_PROFILE_STORAGE.address(),
        TELINK_RETAINED_SECURITY_STORAGE.address(),
        TELINK_RETAINED_APP_STORAGE.address(),
    ]
}

pub fn cold_run() -> ! {
    TELINK_RETAINED_APP_HEADER.begin_cold();
    let board = boot(DEVICE_EUI_OFFSET);
    let profile = unsafe { TELINK_RETAINED_PROFILE_STORAGE.initialize(sensor_profile()) };
    let device = build_device(
        board.ieee_address,
        board.aes,
        &board.leds,
        profile,
        unsafe { TELINK_RETAINED_DEVICE_STORAGE.slot() },
    );
    // Registered after static placement, before the node borrows the device.
    platform::initialize_context::<SensorRgbStatus>(device.mac_mut());

    let (security_partition, _child_partition, _aps_partition) = split_flash(board.flash);
    let security_store =
        unsafe { TELINK_RETAINED_SECURITY_STORAGE.initialize(security_store(security_partition)) };
    if device
        .reset_security_state_if_identity_changed(security_store, board.ieee_address)
        .is_err()
    {
        failure(&board.leds);
    }

    let node = ZigbeeNode::new(device, security_store, profile);
    match SensorApp::new(node, &SENSOR_POLICY, parts(board.leds)) {
        Ok(app) => unsafe { TELINK_RETAINED_APP_STORAGE.initialize(app) },
        Err(_) => halted(),
    };
    TELINK_RETAINED_APP_HEADER.commit(retained_cells());
    TELINK_RETENTION_FRESH_ROOT(true)
}

pub fn retention_run() -> ! {
    platform::resume::<SensorRgbStatus, 4>(&TELINK_RETAINED_APP_HEADER, retained_cells);
    TELINK_RETENTION_FRESH_ROOT(false)
}

pub fn fail_closed_reset() -> ! {
    platform::fail_closed_reset::<SensorRgbStatus>()
}

/// The single executor root. Both reset entries create this future afresh;
/// neither can branch back into a pre-sleep async frame.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn TELINK_RETENTION_FRESH_ROOT(cold: bool) -> ! {
    tlsr8258_rt::block_on(async move {
        let app = unsafe { TELINK_RETAINED_APP_STORAGE.get() };
        if cold {
            if app.initialize().await.is_err() {
                halted();
            }
        } else if app.resume_after_timer_wake().await.is_err() {
            // The LOW32K sleep never returns from `wait`: finish the
            // iteration that slept before starting the next one.
            fail_closed_reset();
        }
        loop {
            if app.step().await.is_err() {
                fail_closed_reset();
            }
            tlsr8258_tb04_product::stack_guard::check();
        }
    })
}
