//! End to end on a combo router (`green-power-channel-request` with
//! `green-power-bidirectional`): a GPD looking for the network sends
//! Channel Requests; the sink, in commissioning mode, answers with a GP
//! Response appointing this node's own proxy as TempMaster, and the GPD's
//! next Channel Request gets the Channel Configuration by timed TX.

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_gp::frame::{FrameType, GpdfParams};
use zigbee_gp::security;
use zigbee_gp::types::{SecurityLevel, cmd};
use zigbee_mac::mock::MockMac;
use zigbee_mac::{MacFrame, McpsDataIndication};
use zigbee_nwk::DeviceType;
use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

use crate::ZigbeeDevice;
use crate::role::Router;

const PAN: PanId = PanId(0x1234);
const ROUTER: ShortAddress = ShortAddress(0x0001);
const ROUTER_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
const RX_END_US: u32 = 4_000_000;

fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        std::thread::yield_now();
    }
}

/// A joined combo router on channel 15 with an On/Off light endpoint.
fn combo() -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(ROUTER_IEEE))
        .device_type(DeviceType::Router)
        .endpoint(1, 0x0104, crate::DeviceId::ON_OFF_LIGHT, |endpoint| {
            endpoint.cluster_server(crate::ClusterId::ON_OFF)
        })
        .build_router();
    device.bdb_mut().attributes_mut().node_is_on_a_network = true;
    let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
    nwk.set_joined(true);
    nwk.security_mut().set_network_key([0x55; 16], 0);
    let nib = nwk.nib_mut();
    nib.pan_id = PAN;
    nib.network_address = ROUTER;
    nib.extended_pan_id = [0xA5; 8];
    nib.ieee_address = ROUTER_IEEE;
    nib.parent_address = ShortAddress::COORDINATOR;
    nib.security_enabled = true;
    nib.outgoing_frame_counter_limit = 0x400;
    nib.logical_channel = 15;
    device
        .gp_sink_mut()
        .set_config(zigbee_gp::sink::SinkConfig {
            endpoint: Some(1),
            ..zigbee_gp::sink::SinkConfig::default()
        });
    device
}

/// A Channel Request as the radio delivers it: the GPD listens next on
/// channel 15 (this network's channel), then 20.
fn channel_request(mac_seq: u8) -> McpsDataIndication {
    let mut raw = [0u8; 127];
    let len = security::build_gpdf(
        &mut MockMac::new([0; 8]),
        None,
        &GpdfParams {
            frame_type: FrameType::Maintenance,
            auto_commissioning: false,
            src_id: None,
            security_level: SecurityLevel::None,
            individual_key: false,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: 0,
        },
        mac_seq,
        cmd::CHANNEL_REQUEST,
        &[zigbee_gp::commissioning::channel_request(15, 20).unwrap()],
        &mut raw,
    )
    .unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        lqi: 180,
        payload: MacFrame::from_slice(&raw[7..len])
            .unwrap()
            .with_mac_sequence(mac_seq)
            .with_rx_end_us(RX_END_US),
        security_use: false,
    }
}

#[test]
fn a_combo_in_commissioning_mode_answers_a_channel_request_by_timed_tx() {
    let mut router = combo();
    router.mac_mut().set_timed_tx(true);

    // Outside commissioning mode the GPD gets nothing.
    router.mac_mut().set_time_micros(RX_END_US + 900);
    block_on(router.process_incoming(&channel_request(1), &mut []));
    block_on(router.gp_poll());
    assert!(router.mac().timed_tx_history().is_empty());

    router.gp_sink_enter_commissioning(false).unwrap();
    block_on(router.gp_poll());
    router.mac_mut().clear_tx_history();

    // First request: the sink queues its GP Response (broadcast, so other
    // sinks see the TempMaster choice) and its own proxy takes it.
    block_on(router.process_incoming(&channel_request(2), &mut []));
    block_on(router.gp_poll());
    assert!(router.mac().timed_tx_history().is_empty());
    assert!(
        !router.mac().tx_history().is_empty(),
        "the GP Response goes on the network"
    );

    // The GPD's next request opens its window: Channel Configuration.
    router.mac_mut().set_time_micros(RX_END_US + 900);
    block_on(router.process_incoming(&channel_request(3), &mut []));
    let sent = router.mac().timed_tx_history();
    assert_eq!(sent.len(), 1, "Channel Configuration in the GPD's window");
    assert_eq!(sent[0].at_us, RX_END_US + 20_000);
    let frame = zigbee_gp::frame::parse(&sent[0].frame[7..], sent[0].frame[2], None).unwrap();
    // A Maintenance frame without the extended frame control: no SrcID
    // and no direction bit (A.1.4.1.2).
    assert_eq!(frame.header.frame_type, FrameType::Maintenance);
    assert_eq!(frame.header.gpd_id, None);
    // Operational channel 15 (0x04) with the Basic bit.
    assert_eq!(frame.payload, [cmd::CHANNEL_CONFIGURATION, 0x14]);
}
