use zigbee_types::ShortAddress;
use zigbee_zdo::binding_mgmt::{BindReq, BindRsp, BindTarget};
use zigbee_zdo::descriptors::{LogicalType, NodeDescriptor, PowerDescriptor, SimpleDescriptor};
use zigbee_zdo::device_announce::DeviceAnnounce;
use zigbee_zdo::discovery::{
    IeeeAddrReq, MatchDescReq, NodeDescReq, NwkAddrReq, RequestType, SimpleDescReq,
};
use zigbee_zdo::network_mgmt::{MgmtLeaveReq, MgmtLqiReq, MgmtPermitJoiningReq};
use zigbee_zdo::ZdpStatus;

// ---------------------------------------------------------------------------
// 1. Descriptor construction
// ---------------------------------------------------------------------------

#[test]
fn node_descriptor_default() {
    let nd = NodeDescriptor::default();
    assert_eq!(nd.logical_type, LogicalType::EndDevice);
    assert_eq!(nd.frequency_band, 0x08);
    assert_eq!(nd.max_buffer_size, 127);
    assert_eq!(nd.max_incoming_transfer, 127);
    assert_eq!(nd.max_outgoing_transfer, 127);
    assert!(!nd.complex_desc_available);
    assert!(!nd.user_desc_available);
}

#[test]
fn node_descriptor_serialize_roundtrip() {
    let nd = NodeDescriptor {
        logical_type: LogicalType::Router,
        complex_desc_available: false,
        user_desc_available: true,
        aps_flags: 0,
        frequency_band: 0x08,
        mac_capabilities: 0x8E,
        manufacturer_code: 0x1234,
        max_buffer_size: 80,
        max_incoming_transfer: 256,
        server_mask: 0x0041,
        max_outgoing_transfer: 256,
        descriptor_capabilities: 0x00,
    };

    let mut buf = [0u8; 32];
    let len = nd.serialize(&mut buf).unwrap();
    assert_eq!(len, NodeDescriptor::WIRE_SIZE);

    let parsed = NodeDescriptor::parse(&buf[..len]).unwrap();
    assert_eq!(parsed, nd);
}

#[test]
fn power_descriptor_default_and_roundtrip() {
    let pd = PowerDescriptor::default();
    assert_eq!(pd.available_power_sources, 0x01);
    assert_eq!(pd.current_power_source, 0x01);
    assert_eq!(pd.current_power_level, 0x0C);

    let mut buf = [0u8; 8];
    let len = pd.serialize(&mut buf).unwrap();
    assert_eq!(len, PowerDescriptor::WIRE_SIZE);

    let parsed = PowerDescriptor::parse(&buf[..len]).unwrap();
    assert_eq!(parsed, pd);
}

#[test]
fn simple_descriptor_roundtrip() {
    let mut sd = SimpleDescriptor {
        endpoint: 1,
        profile_id: 0x0104,
        device_id: 0x0100,
        device_version: 1,
        input_clusters: heapless::Vec::new(),
        output_clusters: heapless::Vec::new(),
    };
    sd.input_clusters.push(0x0000).unwrap(); // Basic
    sd.input_clusters.push(0x0006).unwrap(); // On/Off
    sd.output_clusters.push(0x000A).unwrap(); // Time

    let wire = sd.wire_size();
    let mut buf = [0u8; 64];
    let len = sd.serialize(&mut buf).unwrap();
    assert_eq!(len, wire);

    let parsed = SimpleDescriptor::parse(&buf[..len]).unwrap();
    assert_eq!(parsed, sd);
}

#[test]
fn simple_descriptor_empty_clusters() {
    let sd = SimpleDescriptor {
        endpoint: 42,
        profile_id: 0xC05E,
        device_id: 0x0210,
        device_version: 0,
        input_clusters: heapless::Vec::new(),
        output_clusters: heapless::Vec::new(),
    };

    let mut buf = [0u8; 64];
    let len = sd.serialize(&mut buf).unwrap();
    assert_eq!(len, SimpleDescriptor::MIN_WIRE_SIZE);

    let parsed = SimpleDescriptor::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.endpoint, 42);
    assert_eq!(parsed.profile_id, 0xC05E);
    assert!(parsed.input_clusters.is_empty());
    assert!(parsed.output_clusters.is_empty());
}

// ---------------------------------------------------------------------------
// 2. ZDP request / response roundtrips
// ---------------------------------------------------------------------------

#[test]
fn nwk_addr_req_roundtrip() {
    let req = NwkAddrReq {
        ieee_addr: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        request_type: RequestType::Single,
        start_index: 0,
    };

    let mut buf = [0u8; 32];
    let len = req.serialize(&mut buf).unwrap();
    assert!(len >= NwkAddrReq::MIN_SIZE);

    let parsed = NwkAddrReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.ieee_addr, req.ieee_addr);
    assert_eq!(parsed.request_type, RequestType::Single);
    assert_eq!(parsed.start_index, 0);
}

#[test]
fn ieee_addr_req_roundtrip() {
    let req = IeeeAddrReq {
        nwk_addr_of_interest: ShortAddress(0x1234),
        request_type: RequestType::Extended,
        start_index: 3,
    };

    let mut buf = [0u8; 16];
    let len = req.serialize(&mut buf).unwrap();

    let parsed = IeeeAddrReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.nwk_addr_of_interest, ShortAddress(0x1234));
    assert_eq!(parsed.request_type, RequestType::Extended);
    assert_eq!(parsed.start_index, 3);
}

#[test]
fn simple_desc_req_roundtrip() {
    let req = SimpleDescReq {
        nwk_addr_of_interest: ShortAddress(0x0001),
        endpoint: 8,
    };

    let mut buf = [0u8; 8];
    let len = req.serialize(&mut buf).unwrap();
    assert_eq!(len, SimpleDescReq::SIZE);

    let parsed = SimpleDescReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.nwk_addr_of_interest, ShortAddress(0x0001));
    assert_eq!(parsed.endpoint, 8);
}

#[test]
fn node_desc_req_roundtrip() {
    let req = NodeDescReq {
        nwk_addr_of_interest: ShortAddress(0xABCD),
    };

    let mut buf = [0u8; 8];
    let len = req.serialize(&mut buf).unwrap();
    assert_eq!(len, NodeDescReq::SIZE);

    let parsed = NodeDescReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.nwk_addr_of_interest, ShortAddress(0xABCD));
}

#[test]
fn match_desc_req_roundtrip() {
    let mut req = MatchDescReq {
        nwk_addr_of_interest: ShortAddress(0xFFFF),
        profile_id: 0x0104,
        input_clusters: heapless::Vec::new(),
        output_clusters: heapless::Vec::new(),
    };
    req.input_clusters.push(0x0006).unwrap();
    req.input_clusters.push(0x0008).unwrap();
    req.output_clusters.push(0x000A).unwrap();

    let mut buf = [0u8; 64];
    let len = req.serialize(&mut buf).unwrap();

    let parsed = MatchDescReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.nwk_addr_of_interest, ShortAddress(0xFFFF));
    assert_eq!(parsed.profile_id, 0x0104);
    assert_eq!(parsed.input_clusters.len(), 2);
    assert_eq!(parsed.output_clusters.len(), 1);
    assert_eq!(parsed.output_clusters[0], 0x000A);
}

// ---------------------------------------------------------------------------
// 3. Bind / Unbind
// ---------------------------------------------------------------------------

#[test]
fn bind_req_unicast_roundtrip() {
    let req = BindReq {
        src_addr: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        src_endpoint: 1,
        cluster_id: 0x0006,
        dst: BindTarget::Unicast {
            dst_addr: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11],
            dst_endpoint: 2,
        },
    };

    let mut buf = [0u8; 32];
    let len = req.serialize(&mut buf).unwrap();
    assert!(len >= BindReq::MIN_SIZE);

    let parsed = BindReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.src_addr, req.src_addr);
    assert_eq!(parsed.src_endpoint, 1);
    assert_eq!(parsed.cluster_id, 0x0006);
    match parsed.dst {
        BindTarget::Unicast {
            dst_addr,
            dst_endpoint,
        } => {
            assert_eq!(dst_addr, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11]);
            assert_eq!(dst_endpoint, 2);
        }
        _ => panic!("Expected Unicast target"),
    }
}

#[test]
fn bind_req_group_roundtrip() {
    let req = BindReq {
        src_addr: [0x01; 8],
        src_endpoint: 5,
        cluster_id: 0x0008,
        dst: BindTarget::Group(0x1234),
    };

    let mut buf = [0u8; 32];
    let len = req.serialize(&mut buf).unwrap();

    let parsed = BindReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.cluster_id, 0x0008);
    assert_eq!(parsed.dst, BindTarget::Group(0x1234));
}

#[test]
fn bind_rsp_roundtrip() {
    let rsp = BindRsp {
        status: ZdpStatus::Success,
    };

    let mut buf = [0u8; 4];
    let len = rsp.serialize(&mut buf).unwrap();

    let parsed = BindRsp::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.status, ZdpStatus::Success);
}

// ---------------------------------------------------------------------------
// 4. Management requests
// ---------------------------------------------------------------------------

#[test]
fn mgmt_lqi_req_roundtrip() {
    let req = MgmtLqiReq { start_index: 5 };

    let mut buf = [0u8; 4];
    let len = req.serialize(&mut buf).unwrap();

    let parsed = MgmtLqiReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.start_index, 5);
}

#[test]
fn mgmt_leave_req_roundtrip() {
    let req = MgmtLeaveReq {
        device_address: [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE],
        remove_children: true,
        rejoin: false,
    };

    let mut buf = [0u8; 16];
    let len = req.serialize(&mut buf).unwrap();
    assert_eq!(len, MgmtLeaveReq::SIZE);

    let parsed = MgmtLeaveReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.device_address, req.device_address);
    assert!(parsed.remove_children);
    assert!(!parsed.rejoin);
}

#[test]
fn mgmt_permit_joining_req_roundtrip() {
    let req = MgmtPermitJoiningReq {
        permit_duration: 60,
        tc_significance: 1,
    };

    let mut buf = [0u8; 4];
    let len = req.serialize(&mut buf).unwrap();
    assert_eq!(len, MgmtPermitJoiningReq::SIZE);

    let parsed = MgmtPermitJoiningReq::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.permit_duration, 60);
    assert_eq!(parsed.tc_significance, 1);
}

// ---------------------------------------------------------------------------
// 5. Cluster ID constants
// ---------------------------------------------------------------------------

#[test]
fn cluster_id_constants() {
    assert_eq!(zigbee_zdo::NWK_ADDR_REQ, 0x0000);
    assert_eq!(zigbee_zdo::NWK_ADDR_RSP, 0x8000);
    assert_eq!(zigbee_zdo::IEEE_ADDR_REQ, 0x0001);
    assert_eq!(zigbee_zdo::IEEE_ADDR_RSP, 0x8001);
    assert_eq!(zigbee_zdo::ACTIVE_EP_REQ, 0x0005);
    assert_eq!(zigbee_zdo::ACTIVE_EP_RSP, 0x8005);
    assert_eq!(zigbee_zdo::BIND_REQ, 0x0021);
    assert_eq!(zigbee_zdo::UNBIND_REQ, 0x0022);
    assert_eq!(zigbee_zdo::MGMT_LQI_REQ, 0x0031);
    assert_eq!(zigbee_zdo::MGMT_LEAVE_REQ, 0x0034);
    assert_eq!(zigbee_zdo::DEVICE_ANNCE, 0x0013);
}

// ---------------------------------------------------------------------------
// 6. ZdpStatus helpers
// ---------------------------------------------------------------------------

#[test]
fn zdp_status_from_u8() {
    assert_eq!(ZdpStatus::from_u8(0x00), Some(ZdpStatus::Success));
    assert_eq!(ZdpStatus::from_u8(0x81), Some(ZdpStatus::DeviceNotFound));
    assert_eq!(ZdpStatus::from_u8(0x82), Some(ZdpStatus::InvalidEp));
    assert_eq!(ZdpStatus::from_u8(0xFF), None);
}

// ---------------------------------------------------------------------------
// 7. Device announce
// ---------------------------------------------------------------------------

#[test]
fn device_announce_roundtrip() {
    let da = DeviceAnnounce {
        nwk_addr: ShortAddress(0x1234),
        ieee_addr: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        capability: 0x8E,
    };

    let mut buf = [0u8; 16];
    let len = da.serialize(&mut buf).unwrap();
    assert_eq!(len, DeviceAnnounce::WIRE_SIZE);

    let parsed = DeviceAnnounce::parse(&buf[..len]).unwrap();
    assert_eq!(parsed, da);
}
