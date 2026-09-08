//! Run this target in isolation, without workspace/all-features unification.
//! The facade includes the same test under its independent dependency graph.
use core::future::Future;
use core::task::{Context, Poll, Waker};
use zigbee_aps::frames::{ApsCommandId, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::security::{
    ApsSecurity, ApsSecurityHeader, KEY_ID_KEY_TRANSPORT, SEC_LEVEL_ENC_MIC_32,
    derive_key_transport_key,
};
use zigbee_mac::PlatformServices;
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{
    AssociationStatus, MacFrame, McpsDataIndication, MlmeAssociateConfirm, PanDescriptor,
    SuperframeSpec, ZigbeeBeaconPayload,
};
use zigbee_nwk::DeviceType;
use zigbee_nwk::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_store::{RamSecurityStateStore, SecurityStateStore};
use zigbee_types::{ChannelMask, MacAddress, PanId, ShortAddress};

const IEEE: [u8; 8] = [0x11; 8];
const TC: [u8; 8] = [0x22; 8];
const PAN: PanId = PanId(0x1234);
const ADDRESS: ShortAddress = ShortAddress(0x3344);

fn block_on<F: Future>(future: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    let mut future = std::pin::pin!(future);
    loop {
        if let Poll::Ready(result) = future.as_mut().poll(&mut cx) {
            return result;
        }
        std::thread::yield_now();
    }
}

fn initial_key() -> MacFrame {
    let mut command = [0u8; 35];
    command[0] = ApsCommandId::TransportKey as u8;
    command[1] = 1;
    command[2..18].fill(0x42);
    command[19..27].copy_from_slice(&IEEE);
    command[27..35].copy_from_slice(&TC);
    let security = ApsSecurity::new();
    let key = derive_key_transport_key(security.default_tc_link_key());
    let header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            security: true,
            ..Default::default()
        },
        aps_counter: 1,
        ..Default::default()
    };
    let auxiliary = ApsSecurityHeader {
        security_control: (KEY_ID_KEY_TRANSPORT << 3) | (1 << 5),
        frame_counter: 1,
        source_address: Some(TC),
        key_seq_number: None,
    };
    let mut aps = [0u8; 96];
    let hlen = header.serialize(&mut aps);
    let aad_len = hlen + auxiliary.serialize(&mut aps[hlen..]);
    aps[hlen] |= SEC_LEVEL_ENC_MIC_32;
    let encrypted = security
        .encrypt(&aps[..aad_len], &command, &key, &auxiliary)
        .unwrap();
    aps[hlen] &= !0x07;
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    let length = aad_len + encrypted.len();
    let nwk = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 2,
            ..Default::default()
        },
        dst_addr: ADDRESS,
        src_addr: ShortAddress::COORDINATOR,
        radius: 30,
        seq_number: 1,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let n = nwk.serialize(&mut bytes);
    bytes[n..n + length].copy_from_slice(&aps[..length]);
    MacFrame::from_slice(&bytes[..n + length]).unwrap()
}

#[test]
fn centralized_join_arms_and_runtime_advances_tclk_without_feature_unification() {
    let mut mac = MockMac::new(IEEE);
    mac.add_beacon(PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(PAN, ShortAddress::COORDINATOR),
        superframe_spec: SuperframeSpec {
            association_permit: true,
            pan_coordinator: true,
            ..Default::default()
        },
        lqi: 220,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id: [0x33; 8],
            tx_offset: [0xFF; 3],
            update_id: 0,
        },
    });
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: ADDRESS,
        status: AssociationStatus::Success,
    });
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PAN, ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PAN, ADDRESS),
        lqi: 220,
        payload: initial_key(),
        security_use: false,
    });
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask(1 << 15))
        .build();
    let mut store = RamSecurityStateStore::new();
    assert_eq!(
        block_on(device.start_or_resume_steering_with_security_store(&mut store)),
        Ok(ADDRESS.0)
    );
    assert!(device.bdb().tclk_exchange_active());
    assert!(!store.load().unwrap().unwrap().commissioned);
    let start = device.bdb().tclk_exchange_stage();
    block_on(device.mac_mut().delay_micros(400_000));
    for _ in 0..3 {
        block_on(device.tick_with_steering_security_store(0, &mut [], &mut store)).unwrap();
    }
    assert_ne!(
        device.bdb().tclk_exchange_stage(),
        start,
        "BDB arming without the runtime tick driver must not pass this gate"
    );
    assert!(device.bdb().tclk_exchange_active());
}
