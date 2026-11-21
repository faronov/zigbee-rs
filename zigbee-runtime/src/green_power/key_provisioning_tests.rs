//! End to end on a combo router (`green-power-key-provisioning` with
//! `green-power-bidirectional`): a GPD without a key asks for one; the
//! sink hands it the network key (gpSharedSecurityKeyType 1), which the
//! runtime keeps current, and the combo's own proxy delivers the
//! Commissioning Reply in the GPD's window.

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_gp::commissioning::{CommissioningPayload, CommissioningReply, CommissioningSecurity};
use zigbee_gp::frame::{FrameType, GpdfParams};
use zigbee_gp::security;
use zigbee_gp::types::{KeyType, SecurityLevel, cmd};
use zigbee_mac::mock::MockMac;
use zigbee_mac::{MacFrame, McpsDataIndication};
use zigbee_nwk::DeviceType;
use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

use crate::ZigbeeDevice;
use crate::role::Router;

const ROUTER_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
const NETWORK_KEY: [u8; 16] = [0x55; 16];
const SRC: u32 = 0x293C_9879;
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
    nwk.security_mut().set_network_key(NETWORK_KEY, 0);
    let nib = nwk.nib_mut();
    nib.pan_id = PanId(0x1234);
    nib.network_address = ShortAddress(0x0001);
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

/// A level 3 GPD without a key asking for one, rxAfterTx.
fn commissioning(mac_seq: u8) -> McpsDataIndication {
    let p = CommissioningPayload {
        device_id: 0x02,
        mac_seq_num_capability: true,
        rx_on_capability: true,
        pan_id_request: false,
        security_key_request: true,
        fixed_location: true,
        security: Some(CommissioningSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: None,
            outgoing_counter: Some(40),
        }),
        app_info: None,
    };
    let mut b = [0u8; 64];
    let n = p.encode(&mut b).unwrap();
    let mut raw = [0u8; 127];
    let len = security::build_gpdf(
        &mut MockMac::new([0; 8]),
        None,
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::None,
            individual_key: false,
            rx_after_tx: true,
            to_gpd: false,
            frame_counter: 0,
        },
        mac_seq,
        cmd::COMMISSIONING,
        &b[..n],
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
fn a_combo_hands_the_network_key_to_a_gpd_that_asks_for_one() {
    let mut router = combo();
    router.mac_mut().set_timed_tx(true);
    router
        .gp_sink_mut()
        .set_shared_key(KeyType::NwkKey, [0; 16]);
    router.gp_sink_enter_commissioning(false).unwrap();
    block_on(router.gp_poll());

    router.mac_mut().set_time_micros(RX_END_US + 900);
    block_on(router.process_incoming(&commissioning(1), &mut []));
    block_on(router.gp_poll());
    router.mac_mut().set_time_micros(RX_END_US + 900);
    block_on(router.process_incoming(&commissioning(2), &mut []));
    let sent = router.mac().timed_tx_history();
    assert_eq!(sent.len(), 1, "Commissioning Reply in the GPD's window");
    let frame = zigbee_gp::frame::parse(&sent[0].frame[7..], sent[0].frame[2], None).unwrap();
    assert_eq!(frame.clear_command_id(), Some(cmd::COMMISSIONING_REPLY));
    let reply = CommissioningReply::decode(&frame.payload[1..]).unwrap();
    assert_eq!(reply.key_type, KeyType::NwkKey);
    // The network key never goes on air in clear: TC-LK protected with
    // the GPD's outgoing counter + 1, although the GPD did not ask.
    let Some((key, Some((mic, fc)))) = reply.key else {
        panic!("the network key must be protected");
    };
    assert_eq!(fc, 41);
    let clear = security::unwrap_key(
        &mut MockMac::new([0; 8]),
        &security::DEFAULT_TC_LINK_KEY,
        &zigbee_gp::types::GpdId::SrcId(SRC),
        security::Direction::ToGpd,
        fc,
        &security::WrappedKey {
            key,
            mic: mic.to_le_bytes(),
        },
    )
    .unwrap();
    assert_eq!(clear, NETWORK_KEY);
}
