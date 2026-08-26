#![cfg(not(feature = "router"))]

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_aps::ApsLayer;
use zigbee_bdb::{BdbLayer, BdbStatus};
use zigbee_mac::mock::MockMac;
use zigbee_nwk::{DeviceType, NwkLayer};
use zigbee_zdo::ZdoLayer;

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
fn no_router_end_device_cannot_open_permit_joining() {
    let mac = MockMac::new([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    let nwk = NwkLayer::new(mac, DeviceType::EndDevice);
    let aps = ApsLayer::new(nwk);
    let zdo = ZdoLayer::new(aps);
    let mut bdb = BdbLayer::new(zdo);
    bdb.attributes_mut().node_is_on_a_network = true;

    assert_eq!(
        block_on(bdb.network_steering()),
        Err(BdbStatus::NotPermitted)
    );
    assert!(bdb.zdo().nwk().mac().tx_history().is_empty());
}
