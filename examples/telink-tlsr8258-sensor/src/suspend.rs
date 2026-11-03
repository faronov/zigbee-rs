//! Default image root: one static device, the rest on the root frame, one
//! executor root. Joined waits use full-SRAM SUSPEND.

use core::mem::MaybeUninit;

use sensor_sed_app::SensorApp;
use tlsr8258_tb04_product::boot::{boot, failure, halted};
use tlsr8258_tb04_product::sensor::{DEVICE_EUI_OFFSET, SENSOR_POLICY, sensor_profile};
use tlsr8258_tb04_product::storage::{security_store, split_flash};
use zigbee_runtime::node::ZigbeeNode;

use crate::app::{Device, build_device, parts};

pub fn run() -> ! {
    let board = boot(DEVICE_EUI_OFFSET);
    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    let mut profile = sensor_profile();
    let device = build_device(
        board.ieee_address,
        board.aes,
        &board.leds,
        &mut profile,
        unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) },
    );

    // This end device owns only the two-sector security journal. Identity
    // reset must happen before ZigbeeNode borrows the device/store.
    let (security_partition, _child_partition, _aps_partition) = split_flash(board.flash);
    let mut security_store = security_store(security_partition);
    if device
        .reset_security_state_if_identity_changed(&mut security_store, board.ieee_address)
        .is_err()
    {
        failure(&board.leds);
    }

    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let mut app = match SensorApp::new(node, &SENSOR_POLICY, parts(board.leds)) {
        Ok(app) => app,
        // The RGB adapter has not changed the initialized red state yet.
        Err(_) => halted(),
    };
    tlsr8258_rt::block_on(app.run())
}
