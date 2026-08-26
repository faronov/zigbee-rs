#![cfg(not(feature = "router"))]

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::StartError;
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore, SecurityStoreError,
};
use zigbee_types::ShortAddress;

fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = std::pin::pin!(future);

    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        std::thread::yield_now();
    }
}

#[test]
fn no_router_startup_rejects_a_commissioned_coordinator_record() {
    const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
    const GLOBAL_FLOOR: u32 = 0x800;
    const TCLK_FLOOR: u32 = 0x600;

    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = [0xCC; 8];
    state.pan_id = 0x2A2A;
    state.short_address = ShortAddress::COORDINATOR.0;
    state.ieee_address = IEEE_ADDRESS;
    state.channel = 20;
    state.depth = 0;
    state.parent_address = 0xFFFF;
    state.update_id = 9;
    state.update_id_valid = true;
    state.network_key = [0xA5; 16];
    state.key_sequence = 7;
    state.global_counter_limit = GLOBAL_FLOOR;
    state.tclk_counter_limit = TCLK_FLOOR;
    assert_eq!(state.validate(), Ok(()));

    let mut store = RamSecurityStateStore::new();
    store.store(&state).unwrap();
    let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS)).build();

    assert_eq!(
        block_on(device.start_or_resume_with_security_store(&mut store)),
        Err(StartError::PersistenceFailed(SecurityStoreError::Corrupt))
    );

    let persisted = store.load().unwrap().unwrap();
    assert_eq!(persisted.global_counter_limit, GLOBAL_FLOOR);
    assert_eq!(persisted.tclk_counter_limit, TCLK_FLOOR);

    let mut steering_device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS)).build();
    assert_eq!(
        block_on(steering_device.start_or_resume_steering_with_security_store(&mut store)),
        Err(StartError::PersistenceFailed(SecurityStoreError::Corrupt))
    );
    let persisted = store.load().unwrap().unwrap();
    assert_eq!(persisted.global_counter_limit, GLOBAL_FLOOR);
    assert_eq!(persisted.tclk_counter_limit, TCLK_FLOOR);
}
