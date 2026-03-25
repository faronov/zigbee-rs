//! Comprehensive tests for the zigbee-zcl crate.

use zigbee_zcl::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use zigbee_zcl::clusters::basic::{self, BasicCluster};
use zigbee_zcl::clusters::on_off::{self, OnOffCluster};
use zigbee_zcl::clusters::temperature::{self, TemperatureCluster};
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::data_types::{self, ZclDataType, ZclValue};
use zigbee_zcl::foundation::discover::{self, DiscoverAttributesRequest};
use zigbee_zcl::foundation::read_attributes::{self, ReadAttributesRequest};
use zigbee_zcl::foundation::reporting::{
    ConfigureReportingRequest, ReportAttributes, ReportDirection, ReportingConfig, ReportingEngine,
};
use zigbee_zcl::foundation::write_attributes::{
    self, WriteAttributeRecord, WriteAttributesRequest,
};
use zigbee_zcl::foundation::FoundationCommandId;
use zigbee_zcl::frame::{ZclFrame, ZclFrameError, ZclFrameType};
use zigbee_zcl::{AttributeId, ClusterDirection, ClusterId, CommandId, ZclStatus};

// ---------------------------------------------------------------------------
// 1. ZclFrame / ZclHeader
// ---------------------------------------------------------------------------

#[test]
fn frame_global_roundtrip() {
    let frame = ZclFrame::new_global(
        0x42,
        CommandId(0x00), // ReadAttributes
        ClusterDirection::ClientToServer,
        false,
    );
    assert_eq!(frame.header.frame_type(), ZclFrameType::Global);
    assert_eq!(frame.header.seq_number, 0x42);
    assert_eq!(frame.header.command_id, CommandId(0x00));
    assert_eq!(frame.header.direction(), ClusterDirection::ClientToServer);
    assert!(!frame.header.disable_default_response());
    assert!(!frame.header.is_manufacturer_specific());

    let mut buf = [0u8; 64];
    let len = frame.serialize(&mut buf).unwrap();
    let parsed = ZclFrame::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.header.frame_type(), ZclFrameType::Global);
    assert_eq!(parsed.header.seq_number, 0x42);
    assert_eq!(parsed.header.command_id, CommandId(0x00));
    assert_eq!(parsed.header.direction(), ClusterDirection::ClientToServer);
}

#[test]
fn frame_cluster_specific_roundtrip() {
    let mut frame = ZclFrame::new_cluster_specific(
        0x01,
        CommandId(0x02), // Toggle
        ClusterDirection::ClientToServer,
        true,
    );
    // Add some payload
    frame.payload.push(0xAA).unwrap();
    frame.payload.push(0xBB).unwrap();

    assert_eq!(frame.header.frame_type(), ZclFrameType::ClusterSpecific);
    assert!(frame.header.disable_default_response());

    let mut buf = [0u8; 64];
    let len = frame.serialize(&mut buf).unwrap();
    let parsed = ZclFrame::parse(&buf[..len]).unwrap();

    assert_eq!(parsed.header.frame_type(), ZclFrameType::ClusterSpecific);
    assert_eq!(parsed.header.seq_number, 0x01);
    assert_eq!(parsed.header.command_id, CommandId(0x02));
    assert!(parsed.header.disable_default_response());
    assert_eq!(parsed.payload.len(), 2);
    assert_eq!(parsed.payload[0], 0xAA);
    assert_eq!(parsed.payload[1], 0xBB);
}

#[test]
fn frame_server_to_client_direction() {
    let frame = ZclFrame::new_global(
        0x10,
        CommandId(0x01),
        ClusterDirection::ServerToClient,
        false,
    );
    assert_eq!(frame.header.direction(), ClusterDirection::ServerToClient);

    let mut buf = [0u8; 64];
    let len = frame.serialize(&mut buf).unwrap();
    let parsed = ZclFrame::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.header.direction(), ClusterDirection::ServerToClient);
}

#[test]
fn frame_parse_too_short() {
    assert!(matches!(
        ZclFrame::parse(&[0x00, 0x01]),
        Err(ZclFrameError::TooShort)
    ));
    assert!(matches!(ZclFrame::parse(&[]), Err(ZclFrameError::TooShort)));
}

#[test]
fn frame_manufacturer_specific() {
    // Build a manufacturer-specific frame manually
    let fc = zigbee_zcl::frame::ZclFrameHeader::build_frame_control(
        ZclFrameType::Global,
        true, // manufacturer_specific
        ClusterDirection::ClientToServer,
        false,
    );
    let frame = ZclFrame {
        header: zigbee_zcl::frame::ZclFrameHeader {
            frame_control: fc,
            manufacturer_code: Some(0x1234),
            seq_number: 0x05,
            command_id: CommandId(0x00),
        },
        payload: heapless::Vec::new(),
    };

    assert!(frame.header.is_manufacturer_specific());
    assert_eq!(frame.header.manufacturer_code, Some(0x1234));

    let mut buf = [0u8; 64];
    let len = frame.serialize(&mut buf).unwrap();
    // Manufacturer-specific header: fc(1) + mfr(2) + seq(1) + cmd(1) = 5 bytes
    assert_eq!(len, 5);

    let parsed = ZclFrame::parse(&buf[..len]).unwrap();
    assert!(parsed.header.is_manufacturer_specific());
    assert_eq!(parsed.header.manufacturer_code, Some(0x1234));
    assert_eq!(parsed.header.seq_number, 0x05);
}

// ---------------------------------------------------------------------------
// 2. AttributeStore
// ---------------------------------------------------------------------------

#[test]
fn attribute_store_register_and_get() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    assert!(store.is_empty());
    assert_eq!(store.len(), 0);

    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "TestAttr",
            },
            ZclValue::U8(42),
        )
        .unwrap();

    assert_eq!(store.len(), 1);
    assert!(!store.is_empty());
    assert_eq!(store.get(AttributeId(0x0000)), Some(&ZclValue::U8(42)));
}

#[test]
fn attribute_store_get_nonexistent() {
    let store: AttributeStore<8> = AttributeStore::new();
    assert_eq!(store.get(AttributeId(0x9999)), None);
}

#[test]
fn attribute_store_write_readwrite() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0010),
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "RWAttr",
            },
            ZclValue::U16(100),
        )
        .unwrap();

    store.set(AttributeId(0x0010), ZclValue::U16(200)).unwrap();
    assert_eq!(store.get(AttributeId(0x0010)), Some(&ZclValue::U16(200)));
}

#[test]
fn attribute_store_write_readonly_rejected() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ROAttr",
            },
            ZclValue::U8(1),
        )
        .unwrap();

    let result = store.set(AttributeId(0x0000), ZclValue::U8(2));
    assert_eq!(result, Err(ZclStatus::ReadOnly));
    // Value unchanged
    assert_eq!(store.get(AttributeId(0x0000)), Some(&ZclValue::U8(1)));
}

#[test]
fn attribute_store_write_wrong_type_rejected() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0010),
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "U16Attr",
            },
            ZclValue::U16(0),
        )
        .unwrap();

    let result = store.set(AttributeId(0x0010), ZclValue::U8(5));
    assert_eq!(result, Err(ZclStatus::InvalidDataType));
}

#[test]
fn attribute_store_set_raw_bypasses_access() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "Sensor",
            },
            ZclValue::I16(0),
        )
        .unwrap();

    // Reportable is not writable, but set_raw should work
    store
        .set_raw(AttributeId(0x0000), ZclValue::I16(2500))
        .unwrap();
    assert_eq!(store.get(AttributeId(0x0000)), Some(&ZclValue::I16(2500)));
}

#[test]
fn attribute_store_find_definition() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0001),
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadWrite,
                name: "TestBool",
            },
            ZclValue::Bool(false),
        )
        .unwrap();

    let def = store.find(AttributeId(0x0001)).unwrap();
    assert_eq!(def.data_type, ZclDataType::Bool);
    assert_eq!(def.name, "TestBool");
    assert!(store.find(AttributeId(0x9999)).is_none());
}

// ---------------------------------------------------------------------------
// 3. ZclDataType / ZclValue
// ---------------------------------------------------------------------------

#[test]
fn data_type_from_u8() {
    assert_eq!(ZclDataType::from_u8(0x10), Some(ZclDataType::Bool));
    assert_eq!(ZclDataType::from_u8(0x20), Some(ZclDataType::U8));
    assert_eq!(ZclDataType::from_u8(0x21), Some(ZclDataType::U16));
    assert_eq!(ZclDataType::from_u8(0x29), Some(ZclDataType::I16));
    assert_eq!(ZclDataType::from_u8(0x42), Some(ZclDataType::CharString));
    assert_eq!(ZclDataType::from_u8(0xFF), None);
}

#[test]
fn data_type_size_fixed() {
    assert_eq!(data_types::data_type_size(ZclDataType::Bool), Some(1));
    assert_eq!(data_types::data_type_size(ZclDataType::U16), Some(2));
    assert_eq!(data_types::data_type_size(ZclDataType::I32), Some(4));
    assert_eq!(data_types::data_type_size(ZclDataType::U64), Some(8));
    assert_eq!(
        data_types::data_type_size(ZclDataType::SecurityKey128),
        Some(16)
    );
    // Variable-length
    assert_eq!(data_types::data_type_size(ZclDataType::CharString), None);
    assert_eq!(data_types::data_type_size(ZclDataType::OctetString), None);
}

#[test]
fn zcl_value_data_type_tag() {
    assert_eq!(ZclValue::Bool(true).data_type(), ZclDataType::Bool);
    assert_eq!(ZclValue::U8(0).data_type(), ZclDataType::U8);
    assert_eq!(ZclValue::U16(0).data_type(), ZclDataType::U16);
    assert_eq!(ZclValue::I16(0).data_type(), ZclDataType::I16);
    assert_eq!(ZclValue::Enum8(0).data_type(), ZclDataType::Enum8);
    assert_eq!(
        ZclValue::CharString(heapless::Vec::new()).data_type(),
        ZclDataType::CharString
    );
}

#[test]
fn zcl_value_serialize_deserialize_roundtrip() {
    let test_cases: Vec<(ZclDataType, ZclValue)> = vec![
        (ZclDataType::Bool, ZclValue::Bool(true)),
        (ZclDataType::U8, ZclValue::U8(0xAB)),
        (ZclDataType::U16, ZclValue::U16(0x1234)),
        (ZclDataType::I16, ZclValue::I16(-500)),
        (ZclDataType::U32, ZclValue::U32(0xDEADBEEF)),
        (ZclDataType::I32, ZclValue::I32(-100_000)),
        (ZclDataType::U64, ZclValue::U64(0x0102030405060708)),
        (ZclDataType::Enum8, ZclValue::Enum8(0x03)),
        (ZclDataType::Float32, ZclValue::Float32(3.14)),
        (ZclDataType::UtcTime, ZclValue::UtcTime(1_700_000_000)),
        (
            ZclDataType::IeeeAddr,
            ZclValue::IeeeAddr(0x00124B001CAFBABE),
        ),
    ];

    for (dt, val) in &test_cases {
        let mut buf = [0u8; 32];
        let written = val.serialize(&mut buf);
        let (deserialized, consumed) = ZclValue::deserialize(*dt, &buf[..written]).unwrap();
        assert_eq!(consumed, written, "size mismatch for {:?}", dt);
        assert_eq!(&deserialized, val, "roundtrip failed for {:?}", dt);
    }
}

#[test]
fn zcl_value_char_string_roundtrip() {
    let s = ZclValue::CharString(heapless::Vec::from_slice(b"Zigbee").unwrap());
    let mut buf = [0u8; 64];
    let written = s.serialize(&mut buf);
    // length prefix (1) + 6 chars
    assert_eq!(written, 7);
    assert_eq!(buf[0], 6); // length byte

    let (parsed, consumed) =
        ZclValue::deserialize(ZclDataType::CharString, &buf[..written]).unwrap();
    assert_eq!(consumed, 7);
    if let ZclValue::CharString(v) = &parsed {
        assert_eq!(v.as_slice(), b"Zigbee");
    } else {
        panic!("expected CharString");
    }
}

#[test]
fn analog_vs_discrete_types() {
    assert!(data_types::is_analog_type(ZclDataType::U16));
    assert!(data_types::is_analog_type(ZclDataType::I16));
    assert!(data_types::is_analog_type(ZclDataType::Float32));
    assert!(!data_types::is_analog_type(ZclDataType::Bool));
    assert!(!data_types::is_analog_type(ZclDataType::Enum8));
    assert!(data_types::is_discrete_type(ZclDataType::Bool));
    assert!(!data_types::is_discrete_type(ZclDataType::U16));
}

// ---------------------------------------------------------------------------
// 4. Cluster instantiation
// ---------------------------------------------------------------------------

#[test]
fn basic_cluster_id_and_defaults() {
    let cluster = BasicCluster::new(b"TestMfr", b"Model1", b"20240101", b"1.0.0");
    assert_eq!(cluster.cluster_id(), ClusterId::BASIC);

    let attrs = cluster.attributes();
    // ZCL version = 8
    assert_eq!(attrs.get(basic::ATTR_ZCL_VERSION), Some(&ZclValue::U8(8)));
    // Manufacturer name
    if let Some(ZclValue::CharString(v)) = attrs.get(basic::ATTR_MANUFACTURER_NAME) {
        assert_eq!(v.as_slice(), b"TestMfr");
    } else {
        panic!("expected CharString for ManufacturerName");
    }
    // Model identifier
    if let Some(ZclValue::CharString(v)) = attrs.get(basic::ATTR_MODEL_IDENTIFIER) {
        assert_eq!(v.as_slice(), b"Model1");
    } else {
        panic!("expected CharString for ModelIdentifier");
    }
    // Power source
    assert_eq!(
        attrs.get(basic::ATTR_POWER_SOURCE),
        Some(&ZclValue::Enum8(0x01))
    );
}

#[test]
fn on_off_cluster_defaults_and_commands() {
    let mut cluster = OnOffCluster::new();
    assert_eq!(cluster.cluster_id(), ClusterId::ON_OFF);
    assert!(!cluster.is_on());

    // CMD_ON
    cluster.handle_command(on_off::CMD_ON, &[]).unwrap();
    assert!(cluster.is_on());

    // CMD_OFF
    cluster.handle_command(on_off::CMD_OFF, &[]).unwrap();
    assert!(!cluster.is_on());

    // CMD_TOGGLE
    cluster.handle_command(on_off::CMD_TOGGLE, &[]).unwrap();
    assert!(cluster.is_on());
    cluster.handle_command(on_off::CMD_TOGGLE, &[]).unwrap();
    assert!(!cluster.is_on());
}

#[test]
fn on_off_unsupported_command() {
    let mut cluster = OnOffCluster::new();
    let result = cluster.handle_command(CommandId(0xFF), &[]);
    assert_eq!(result, Err(ZclStatus::UnsupClusterCommand));
}

#[test]
fn temperature_cluster_id_and_update() {
    let mut cluster = TemperatureCluster::new(-4000, 8500);
    assert_eq!(cluster.cluster_id(), ClusterId::TEMPERATURE);

    let attrs = cluster.attributes();
    assert_eq!(
        attrs.get(temperature::ATTR_MEASURED_VALUE),
        Some(&ZclValue::I16(0))
    );
    assert_eq!(
        attrs.get(temperature::ATTR_MIN_MEASURED_VALUE),
        Some(&ZclValue::I16(-4000))
    );
    assert_eq!(
        attrs.get(temperature::ATTR_MAX_MEASURED_VALUE),
        Some(&ZclValue::I16(8500))
    );

    cluster.set_temperature(2350);
    assert_eq!(
        cluster.attributes().get(temperature::ATTR_MEASURED_VALUE),
        Some(&ZclValue::I16(2350))
    );
}

// ---------------------------------------------------------------------------
// 5. Foundation commands
// ---------------------------------------------------------------------------

#[test]
fn read_attributes_request_roundtrip() {
    let mut req = ReadAttributesRequest {
        attributes: heapless::Vec::new(),
    };
    req.attributes.push(AttributeId(0x0000)).unwrap();
    req.attributes.push(AttributeId(0x0005)).unwrap();

    let mut buf = [0u8; 64];
    let len = req.serialize(&mut buf);
    assert_eq!(len, 4); // 2 attrs * 2 bytes each

    let parsed = ReadAttributesRequest::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.attributes.len(), 2);
    assert_eq!(parsed.attributes[0], AttributeId(0x0000));
    assert_eq!(parsed.attributes[1], AttributeId(0x0005));
}

#[test]
fn process_read_existing_attribute() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ZCLVersion",
            },
            ZclValue::U8(8),
        )
        .unwrap();

    let mut req = ReadAttributesRequest {
        attributes: heapless::Vec::new(),
    };
    req.attributes.push(AttributeId(0x0000)).unwrap();

    let resp = read_attributes::process_read(&store, &req);
    assert_eq!(resp.records.len(), 1);
    assert_eq!(resp.records[0].status, ZclStatus::Success);
    assert_eq!(resp.records[0].value, Some(ZclValue::U8(8)));
}

#[test]
fn process_read_unsupported_attribute() {
    let store: AttributeStore<4> = AttributeStore::new();
    let mut req = ReadAttributesRequest {
        attributes: heapless::Vec::new(),
    };
    req.attributes.push(AttributeId(0xFFFF)).unwrap();

    let resp = read_attributes::process_read(&store, &req);
    assert_eq!(resp.records.len(), 1);
    assert_eq!(resp.records[0].status, ZclStatus::UnsupportedAttribute);
    assert_eq!(resp.records[0].value, None);
}

#[test]
fn read_attributes_response_serialize_parse_roundtrip() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ZCLVersion",
            },
            ZclValue::U8(8),
        )
        .unwrap();

    let mut req = ReadAttributesRequest {
        attributes: heapless::Vec::new(),
    };
    req.attributes.push(AttributeId(0x0000)).unwrap();
    let resp = read_attributes::process_read(&store, &req);

    let mut buf = [0u8; 128];
    let len = resp.serialize(&mut buf);
    let parsed =
        zigbee_zcl::foundation::read_attributes::ReadAttributesResponse::parse(&buf[..len])
            .unwrap();
    assert_eq!(parsed.records.len(), 1);
    assert_eq!(parsed.records[0].status, ZclStatus::Success);
    assert_eq!(parsed.records[0].value, Some(ZclValue::U8(8)));
}

#[test]
fn write_attributes_request_roundtrip() {
    let mut req = WriteAttributesRequest {
        records: heapless::Vec::new(),
    };
    req.records
        .push(WriteAttributeRecord {
            id: AttributeId(0x0010),
            data_type: ZclDataType::U16,
            value: ZclValue::U16(300),
        })
        .unwrap();

    let mut buf = [0u8; 64];
    let len = req.serialize(&mut buf);
    // attr_id(2) + data_type(1) + value(2) = 5
    assert_eq!(len, 5);

    let parsed = WriteAttributesRequest::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.records.len(), 1);
    assert_eq!(parsed.records[0].id, AttributeId(0x0010));
    assert_eq!(parsed.records[0].value, ZclValue::U16(300));
}

#[test]
fn process_write_success() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0010),
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadWrite,
                name: "Location",
            },
            ZclValue::CharString(heapless::Vec::new()),
        )
        .unwrap();

    let mut req = WriteAttributesRequest {
        records: heapless::Vec::new(),
    };
    req.records
        .push(WriteAttributeRecord {
            id: AttributeId(0x0010),
            data_type: ZclDataType::CharString,
            value: ZclValue::CharString(heapless::Vec::from_slice(b"Office").unwrap()),
        })
        .unwrap();

    let resp = write_attributes::process_write(&mut store, &req);
    assert_eq!(resp.records.len(), 1);
    assert_eq!(resp.records[0].status, ZclStatus::Success);

    if let Some(ZclValue::CharString(v)) = store.get(AttributeId(0x0010)) {
        assert_eq!(v.as_slice(), b"Office");
    } else {
        panic!("expected CharString");
    }
}

#[test]
fn process_write_readonly_fails() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ReadOnly",
            },
            ZclValue::U8(1),
        )
        .unwrap();

    let mut req = WriteAttributesRequest {
        records: heapless::Vec::new(),
    };
    req.records
        .push(WriteAttributeRecord {
            id: AttributeId(0x0000),
            data_type: ZclDataType::U8,
            value: ZclValue::U8(99),
        })
        .unwrap();

    let resp = write_attributes::process_write(&mut store, &req);
    assert_eq!(resp.records[0].status, ZclStatus::ReadOnly);
    // Value unchanged
    assert_eq!(store.get(AttributeId(0x0000)), Some(&ZclValue::U8(1)));
}

// ---------------------------------------------------------------------------
// 6. Discover Attributes
// ---------------------------------------------------------------------------

#[test]
fn discover_attributes_all() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "A",
            },
            ZclValue::U8(0),
        )
        .unwrap();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0001),
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadOnly,
                name: "B",
            },
            ZclValue::Bool(false),
        )
        .unwrap();

    let req = DiscoverAttributesRequest {
        start_id: AttributeId(0x0000),
        max_results: 10,
    };
    let resp = discover::process_discover(&store, &req);
    assert!(resp.complete);
    assert_eq!(resp.attributes.len(), 2);
    assert_eq!(resp.attributes[0].id, AttributeId(0x0000));
    assert_eq!(resp.attributes[0].data_type, ZclDataType::U8);
    assert_eq!(resp.attributes[1].id, AttributeId(0x0001));
}

#[test]
fn discover_attributes_partial() {
    let mut store: AttributeStore<8> = AttributeStore::new();
    for i in 0..5u16 {
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(i),
                    data_type: ZclDataType::U8,
                    access: AttributeAccess::ReadOnly,
                    name: "X",
                },
                ZclValue::U8(i as u8),
            )
            .unwrap();
    }

    let req = DiscoverAttributesRequest {
        start_id: AttributeId(0x0000),
        max_results: 3,
    };
    let resp = discover::process_discover(&store, &req);
    assert!(!resp.complete); // more attributes remain
    assert_eq!(resp.attributes.len(), 3);
}

// ---------------------------------------------------------------------------
// 7. Reporting
// ---------------------------------------------------------------------------

#[test]
fn reporting_engine_max_interval() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "Temp",
            },
            ZclValue::I16(2200),
        )
        .unwrap();

    let mut engine = ReportingEngine::new();
    engine
        .configure(ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::I16,
            min_interval: 0,
            max_interval: 60,
            reportable_change: None,
        })
        .unwrap();

    // First check should trigger (no previous value, min_interval=0)
    let report = engine.check_and_report(&store);
    assert!(report.is_some());
    let rpt = report.unwrap();
    assert_eq!(rpt.reports.len(), 1);
    assert_eq!(rpt.reports[0].value, ZclValue::I16(2200));

    // Immediately after, no report (value unchanged, max not elapsed)
    let report = engine.check_and_report(&store);
    assert!(report.is_none());

    // Advance past max_interval → periodic report even without value change
    engine.tick(61);
    let report = engine.check_and_report(&store);
    assert!(report.is_some());
}

#[test]
fn reporting_engine_value_change() {
    let mut store: AttributeStore<4> = AttributeStore::new();
    store
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::Bool,
                access: AttributeAccess::Reportable,
                name: "OnOff",
            },
            ZclValue::Bool(false),
        )
        .unwrap();

    let mut engine = ReportingEngine::new();
    engine
        .configure(ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::Bool,
            min_interval: 0,
            max_interval: 300,
            reportable_change: None,
        })
        .unwrap();

    // First report (initial, min_interval=0 so triggers immediately)
    let report = engine.check_and_report(&store);
    assert!(report.is_some());

    // Change value — should report immediately since min_interval=0
    store
        .set_raw(AttributeId(0x0000), ZclValue::Bool(true))
        .unwrap();

    let report = engine.check_and_report(&store);
    assert!(report.is_some());
    assert_eq!(report.unwrap().reports[0].value, ZclValue::Bool(true));
}

#[test]
fn configure_reporting_parse() {
    // Construct a ConfigureReporting payload for a discrete Bool attribute
    // direction=0x00 (Send), attr_id=0x0000, data_type=0x10 (Bool),
    // min=0x0001, max=0x012C (300)
    let payload: &[u8] = &[
        0x00, // direction: Send
        0x00, 0x00, // attribute_id: 0x0000
        0x10, // data_type: Bool
        0x01, 0x00, // min_interval: 1
        0x2C, 0x01, // max_interval: 300
              // no reportable_change for discrete
    ];

    let parsed = ConfigureReportingRequest::parse(payload).unwrap();
    assert_eq!(parsed.configs.len(), 1);
    assert_eq!(parsed.configs[0].direction, ReportDirection::Send);
    assert_eq!(parsed.configs[0].attribute_id, AttributeId(0x0000));
    assert_eq!(parsed.configs[0].min_interval, 1);
    assert_eq!(parsed.configs[0].max_interval, 300);
    assert!(parsed.configs[0].reportable_change.is_none());
}

#[test]
fn report_attributes_serialize() {
    let mut reports_vec = heapless::Vec::new();
    reports_vec
        .push(zigbee_zcl::foundation::reporting::AttributeReport {
            id: AttributeId(0x0000),
            data_type: ZclDataType::Bool,
            value: ZclValue::Bool(true),
        })
        .unwrap();

    let report = ReportAttributes {
        reports: reports_vec,
    };

    let mut buf = [0u8; 64];
    let len = report.serialize(&mut buf);
    // attr_id(2) + data_type(1) + value(1) = 4
    assert_eq!(len, 4);
    assert_eq!(buf[0], 0x00); // attr_id low
    assert_eq!(buf[1], 0x00); // attr_id high
    assert_eq!(buf[2], 0x10); // data_type Bool
    assert_eq!(buf[3], 0x01); // value true
}

#[test]
fn reporting_engine_cluster_aware() {
    // Two clusters on different endpoints, each with a MeasuredValue (attr 0x0000)
    let mut store_temp: AttributeStore<4> = AttributeStore::new();
    store_temp
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "TempValue",
            },
            ZclValue::I16(2200),
        )
        .unwrap();

    let mut store_hum: AttributeStore<4> = AttributeStore::new();
    store_hum
        .register(
            AttributeDefinition {
                id: AttributeId(0x0000),
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "HumValue",
            },
            ZclValue::U16(5000),
        )
        .unwrap();

    let mut engine = ReportingEngine::new();

    // Configure temp (ep=1, cluster=0x0402) and humidity (ep=1, cluster=0x0405)
    engine
        .configure_for_cluster(
            1,
            0x0402,
            ReportingConfig {
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0000),
                data_type: ZclDataType::I16,
                min_interval: 0,
                max_interval: 60,
                reportable_change: None,
            },
        )
        .unwrap();

    engine
        .configure_for_cluster(
            1,
            0x0405,
            ReportingConfig {
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0000),
                data_type: ZclDataType::U16,
                min_interval: 0,
                max_interval: 60,
                reportable_change: None,
            },
        )
        .unwrap();

    // Check temp cluster only — should get temp report
    let report = engine.check_and_report_cluster(1, 0x0402, &store_temp);
    assert!(report.is_some());
    assert_eq!(report.unwrap().reports[0].value, ZclValue::I16(2200));

    // Check humidity cluster only — should get humidity report
    let report = engine.check_and_report_cluster(1, 0x0405, &store_hum);
    assert!(report.is_some());
    assert_eq!(report.unwrap().reports[0].value, ZclValue::U16(5000));

    // No repeat report without time or value change
    let report = engine.check_and_report_cluster(1, 0x0402, &store_temp);
    assert!(report.is_none());
}

#[test]
fn reporting_engine_get_config() {
    let mut engine = ReportingEngine::new();
    engine
        .configure_for_cluster(
            1,
            0x0402,
            ReportingConfig {
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0000),
                data_type: ZclDataType::I16,
                min_interval: 10,
                max_interval: 300,
                reportable_change: None,
            },
        )
        .unwrap();

    // Should find it
    let cfg = engine.get_config(1, 0x0402, ReportDirection::Send, AttributeId(0x0000));
    assert!(cfg.is_some());
    assert_eq!(cfg.unwrap().min_interval, 10);
    assert_eq!(cfg.unwrap().max_interval, 300);

    // Wrong cluster — should not find
    let cfg = engine.get_config(1, 0x0405, ReportDirection::Send, AttributeId(0x0000));
    assert!(cfg.is_none());

    // Wrong endpoint — should not find
    let cfg = engine.get_config(2, 0x0402, ReportDirection::Send, AttributeId(0x0000));
    assert!(cfg.is_none());
}

#[test]
fn configure_reporting_response_serialize() {
    use zigbee_zcl::foundation::reporting::{
        ConfigureReportingResponse, ConfigureReportingStatusRecord,
    };

    // All success → single byte
    let response = ConfigureReportingResponse {
        records: {
            let mut v = heapless::Vec::new();
            let _ = v.push(ConfigureReportingStatusRecord {
                status: ZclStatus::Success,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0000),
            });
            v
        },
    };
    let mut buf = [0u8; 64];
    let len = response.serialize(&mut buf);
    assert_eq!(len, 1);
    assert_eq!(buf[0], 0x00); // Success

    // Mixed results → individual records
    let response = ConfigureReportingResponse {
        records: {
            let mut v = heapless::Vec::new();
            let _ = v.push(ConfigureReportingStatusRecord {
                status: ZclStatus::Success,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0000),
            });
            let _ = v.push(ConfigureReportingStatusRecord {
                status: ZclStatus::UnsupportedAttribute,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(0x0001),
            });
            v
        },
    };
    let len = response.serialize(&mut buf);
    assert_eq!(len, 8); // 2 records × (status + direction + attr_id) = 2×4
}

#[test]
fn foundation_command_id_from_u8() {
    assert_eq!(
        FoundationCommandId::from_u8(0x00),
        Some(FoundationCommandId::ReadAttributes)
    );
    assert_eq!(
        FoundationCommandId::from_u8(0x02),
        Some(FoundationCommandId::WriteAttributes)
    );
    assert_eq!(
        FoundationCommandId::from_u8(0x06),
        Some(FoundationCommandId::ConfigureReporting)
    );
    assert_eq!(
        FoundationCommandId::from_u8(0x0A),
        Some(FoundationCommandId::ReportAttributes)
    );
    assert_eq!(
        FoundationCommandId::from_u8(0x0C),
        Some(FoundationCommandId::DiscoverAttributes)
    );
    assert_eq!(FoundationCommandId::from_u8(0xFF), None);
}

// ---------------------------------------------------------------------------
// 9. ZclStatus
// ---------------------------------------------------------------------------

#[test]
fn zcl_status_from_u8() {
    assert_eq!(ZclStatus::from_u8(0x00), ZclStatus::Success);
    assert_eq!(ZclStatus::from_u8(0x86), ZclStatus::UnsupportedAttribute);
    assert_eq!(ZclStatus::from_u8(0x88), ZclStatus::ReadOnly);
    assert_eq!(ZclStatus::from_u8(0x8D), ZclStatus::InvalidDataType);
    // Unknown maps to Failure
    assert_eq!(ZclStatus::from_u8(0xFE), ZclStatus::Failure);
}

// ---------------------------------------------------------------------------
// 10. ClusterId constants
// ---------------------------------------------------------------------------

#[test]
fn cluster_id_constants() {
    assert_eq!(ClusterId::BASIC.0, 0x0000);
    assert_eq!(ClusterId::ON_OFF.0, 0x0006);
    assert_eq!(ClusterId::TEMPERATURE.0, 0x0402);
    assert_eq!(ClusterId::HUMIDITY.0, 0x0405);
    assert_eq!(ClusterId::PRESSURE.0, 0x0403);
    assert_eq!(ClusterId::COLOR_CONTROL.0, 0x0300);
    assert_eq!(ClusterId::LEVEL_CONTROL.0, 0x0008);
}

// ---------------------------------------------------------------------------
// 11. SecurityKey128 roundtrip
// ---------------------------------------------------------------------------

#[test]
fn security_key_roundtrip() {
    let key = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    let val = ZclValue::SecurityKey128(key);
    let mut buf = [0u8; 32];
    let written = val.serialize(&mut buf);
    assert_eq!(written, 16);

    let (parsed, consumed) =
        ZclValue::deserialize(ZclDataType::SecurityKey128, &buf[..16]).unwrap();
    assert_eq!(consumed, 16);
    assert_eq!(parsed, ZclValue::SecurityKey128(key));
}
