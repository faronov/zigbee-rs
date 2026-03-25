//! OTA Upgrade tests — image header parsing, state machine, and mock download.

use zigbee_zcl::clusters::ota::*;
use zigbee_zcl::clusters::ota_image::*;

// ── Image header parsing tests ──────────────────────────────────

#[test]
fn parse_ota_header_minimal() {
    // Build a minimal valid OTA header (56 bytes)
    let mut data = [0u8; 64];
    // Magic
    data[0..4].copy_from_slice(&OTA_MAGIC.to_le_bytes());
    // Header version
    data[4..6].copy_from_slice(&0x0100u16.to_le_bytes());
    // Header length
    data[6..8].copy_from_slice(&56u16.to_le_bytes());
    // Field control (no optional fields)
    data[8..10].copy_from_slice(&0u16.to_le_bytes());
    // Manufacturer code
    data[10..12].copy_from_slice(&0x1234u16.to_le_bytes());
    // Image type
    data[12..14].copy_from_slice(&0x0001u16.to_le_bytes());
    // File version
    data[14..18].copy_from_slice(&0x00000002u32.to_le_bytes());
    // Stack version
    data[18..20].copy_from_slice(&0x0002u16.to_le_bytes());
    // Header string (32 bytes)
    let header_str = b"Test OTA Image\0";
    data[20..20 + header_str.len()].copy_from_slice(header_str);
    // Total image size
    data[52..56].copy_from_slice(&1024u32.to_le_bytes());

    let (header, consumed) = OtaImageHeader::parse(&data).unwrap();
    assert_eq!(header.magic, OTA_MAGIC);
    assert_eq!(header.header_version, 0x0100);
    assert_eq!(header.header_length, 56);
    assert_eq!(header.manufacturer_code, 0x1234);
    assert_eq!(header.image_type, 0x0001);
    assert_eq!(header.file_version, 0x00000002);
    assert_eq!(header.stack_version, 0x0002);
    assert_eq!(header.total_image_size, 1024);
    assert_eq!(header.payload_size(), 1024 - 56);
    assert_eq!(consumed, 56);
    assert!(header.header_string_str().starts_with("Test OTA Image"));
    assert!(header.security_credential_version.is_none());
    assert!(header.min_hardware_version.is_none());
    assert!(header.max_hardware_version.is_none());
}

#[test]
fn parse_ota_header_bad_magic() {
    let data = [0u8; 64];
    assert_eq!(OtaImageHeader::parse(&data), Err(OtaImageError::BadMagic));
}

#[test]
fn parse_ota_header_too_short() {
    let data = [0u8; 10];
    assert_eq!(OtaImageHeader::parse(&data), Err(OtaImageError::TooShort));
}

#[test]
fn parse_sub_element() {
    let mut data = [0u8; 8];
    // Tag: UpgradeImage (0x0000)
    data[0..2].copy_from_slice(&0x0000u16.to_le_bytes());
    // Length: 512
    data[2..6].copy_from_slice(&512u32.to_le_bytes());

    let (elem, consumed) = OtaSubElement::parse(&data).unwrap();
    assert_eq!(elem.tag, OtaTagId::UpgradeImage);
    assert_eq!(elem.length, 512);
    assert_eq!(consumed, 6);
}

// ── Command serialization/parsing tests ─────────────────────────

#[test]
fn query_request_serialize() {
    let req = QueryNextImageRequest {
        field_control: 0x00,
        manufacturer_code: 0x1234,
        image_type: 0x0001,
        current_file_version: 0x00000001,
        hardware_version: None,
    };
    let mut buf = [0u8; 16];
    let len = req.serialize(&mut buf);
    assert_eq!(len, 9);
    assert_eq!(buf[0], 0x00); // field_control
    assert_eq!(u16::from_le_bytes([buf[1], buf[2]]), 0x1234);
    assert_eq!(u16::from_le_bytes([buf[3], buf[4]]), 0x0001);
    assert_eq!(
        u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]),
        0x00000001
    );
}

#[test]
fn query_response_parse_success() {
    let mut data = [0u8; 13];
    data[0] = 0x00; // Success
    data[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    data[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    data[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    data[9..13].copy_from_slice(&4096u32.to_le_bytes());

    let resp = QueryNextImageResponse::parse(&data).unwrap();
    assert_eq!(resp.status, 0x00);
    assert_eq!(resp.manufacturer_code, Some(0x1234));
    assert_eq!(resp.image_type, Some(0x0001));
    assert_eq!(resp.file_version, Some(0x00000002));
    assert_eq!(resp.image_size, Some(4096));
}

#[test]
fn query_response_parse_no_image() {
    let data = [0x98u8]; // NO_IMAGE_AVAILABLE
    let resp = QueryNextImageResponse::parse(&data).unwrap();
    assert_eq!(resp.status, 0x98);
    assert!(resp.file_version.is_none());
}

#[test]
fn block_request_serialize() {
    let req = ImageBlockRequest {
        field_control: 0x00,
        manufacturer_code: 0x1234,
        image_type: 0x0001,
        file_version: 0x00000002,
        file_offset: 256,
        max_data_size: 48,
    };
    let mut buf = [0u8; 16];
    let len = req.serialize(&mut buf);
    assert_eq!(len, 14);
    assert_eq!(u32::from_le_bytes([buf[9], buf[10], buf[11], buf[12]]), 256);
    assert_eq!(buf[13], 48);
}

#[test]
fn block_response_parse_success() {
    let mut data = [0u8; 20];
    data[0] = 0x00; // Success
    data[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    data[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    data[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    data[9..13].copy_from_slice(&0u32.to_le_bytes()); // offset
    data[13] = 4; // data_size
    data[14..18].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);

    let parsed = ParsedBlockResponse::parse(&data).unwrap();
    match parsed {
        ParsedBlockResponse::Success(block) => {
            assert_eq!(block.file_offset, 0);
            assert_eq!(block.data_size, 4);
            assert_eq!(block.data.len(), 4);
            assert_eq!(block.data[0], 0xAA);
        }
        _ => panic!("Expected Success"),
    }
}

#[test]
fn block_response_parse_wait() {
    let mut data = [0u8; 11];
    data[0] = 0x97; // WAIT_FOR_DATA
    data[1..5].copy_from_slice(&1000u32.to_le_bytes()); // current_time
    data[5..9].copy_from_slice(&1010u32.to_le_bytes()); // request_time
    data[9..11].copy_from_slice(&5u16.to_le_bytes()); // min_block_period

    let parsed = ParsedBlockResponse::parse(&data).unwrap();
    match parsed {
        ParsedBlockResponse::WaitForData(wait) => {
            assert_eq!(wait.current_time, 1000);
            assert_eq!(wait.request_time, 1010);
            assert_eq!(wait.minimum_block_period, 5);
        }
        _ => panic!("Expected WaitForData"),
    }
}

#[test]
fn upgrade_end_request_serialize() {
    let req = UpgradeEndRequest {
        status: 0x00,
        manufacturer_code: 0x1234,
        image_type: 0x0001,
        file_version: 0x00000002,
    };
    let mut buf = [0u8; 12];
    let len = req.serialize(&mut buf);
    assert_eq!(len, 9);
    assert_eq!(buf[0], 0x00);
}

#[test]
fn upgrade_end_response_parse() {
    let mut data = [0u8; 16];
    data[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
    data[2..4].copy_from_slice(&0x0001u16.to_le_bytes());
    data[4..8].copy_from_slice(&0x00000002u32.to_le_bytes());
    data[8..12].copy_from_slice(&1000u32.to_le_bytes()); // current_time
    data[12..16].copy_from_slice(&0u32.to_le_bytes()); // upgrade_time = NOW

    let resp = UpgradeEndResponse::parse(&data).unwrap();
    assert_eq!(resp.manufacturer_code, 0x1234);
    assert_eq!(resp.upgrade_time, 0); // Immediate upgrade
}

// ── OTA State Machine tests ─────────────────────────────────────

#[test]
fn ota_cluster_initial_state() {
    let cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    assert_eq!(cluster.state(), OtaState::Idle);
    assert_eq!(cluster.progress_percent(), 0);
}

#[test]
fn ota_cluster_start_query() {
    let mut cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    let action = cluster.start_query();
    assert_eq!(cluster.state(), OtaState::QuerySent);
    match action {
        OtaAction::SendQuery(req) => {
            assert_eq!(req.manufacturer_code, 0x1234);
            assert_eq!(req.image_type, 0x0001);
            assert_eq!(req.current_file_version, 0x00000001);
        }
        _ => panic!("Expected SendQuery"),
    }
}

#[test]
fn ota_cluster_no_image_available() {
    let mut cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    cluster.start_query();

    // Server responds with no image
    let action = cluster.process_server_command(0x02, &[0x98]);
    assert_eq!(cluster.state(), OtaState::Idle);
    matches!(action, OtaAction::None);
}

#[test]
fn ota_cluster_image_available_starts_download() {
    let mut cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    cluster.start_query();

    // Server responds with image available
    let mut resp = [0u8; 13];
    resp[0] = 0x00; // Success
    resp[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    resp[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    resp[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    resp[9..13].copy_from_slice(&256u32.to_le_bytes()); // 256 bytes

    let action = cluster.process_server_command(0x02, &resp);
    match cluster.state() {
        OtaState::Downloading { offset, total_size } => {
            assert_eq!(offset, 0);
            assert_eq!(total_size, 256);
        }
        s => panic!("Expected Downloading, got {:?}", s),
    }
    match action {
        OtaAction::SendBlockRequest(req) => {
            assert_eq!(req.file_offset, 0);
            assert_eq!(req.max_data_size, DEFAULT_BLOCK_SIZE);
        }
        a => panic!("Expected SendBlockRequest, got {:?}", a),
    }
}

#[test]
fn ota_cluster_block_write_action() {
    let mut cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    cluster.start_query();

    // Query response with 100 byte image
    let mut resp = [0u8; 13];
    resp[0] = 0x00;
    resp[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    resp[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    resp[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    resp[9..13].copy_from_slice(&100u32.to_le_bytes());
    cluster.process_server_command(0x02, &resp);

    // Server sends a block
    let mut block = [0u8; 20];
    block[0] = 0x00; // Success
    block[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    block[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    block[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    block[9..13].copy_from_slice(&0u32.to_le_bytes()); // offset=0
    block[13] = 4; // data_size=4
    block[14..18].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);

    let action = cluster.process_server_command(0x05, &block);
    match action {
        OtaAction::WriteBlock { offset, data } => {
            assert_eq!(offset, 0);
            assert_eq!(data.len(), 4);
            assert_eq!(data[0], 0x11);
        }
        a => panic!("Expected WriteBlock, got {:?}", a),
    }
    // State should advance to offset=4
    match cluster.state() {
        OtaState::Downloading { offset, .. } => assert_eq!(offset, 4),
        s => panic!("Expected Downloading, got {:?}", s),
    }
}

#[test]
fn ota_cluster_abort() {
    let mut cluster = OtaCluster::new(0x1234, 0x0001, 0x00000001);
    cluster.start_query();
    assert_eq!(cluster.state(), OtaState::QuerySent);
    cluster.abort();
    assert_eq!(cluster.state(), OtaState::Idle);
}

// ── Mock Firmware Writer tests ──────────────────────────────────

#[test]
fn mock_firmware_writer_basic() {
    use zigbee_runtime::firmware_writer::{FirmwareWriter, MockFirmwareWriter};

    let mut writer = MockFirmwareWriter::new(4096);
    assert_eq!(writer.slot_size(), 4096);
    assert!(!writer.is_activated());

    writer.erase_slot().unwrap();
    writer.write_block(0, &[1, 2, 3, 4]).unwrap();
    writer.write_block(4, &[5, 6, 7, 8]).unwrap();
    assert_eq!(writer.bytes_written(), 8);
    assert_eq!(writer.data(), &[1, 2, 3, 4, 5, 6, 7, 8]);

    writer.verify(8, None).unwrap();
    writer.activate().unwrap();
    assert!(writer.is_activated());
}

#[test]
fn mock_firmware_writer_verify_size_mismatch() {
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter, MockFirmwareWriter};

    let mut writer = MockFirmwareWriter::new(4096);
    writer.erase_slot().unwrap();
    writer.write_block(0, &[1, 2, 3, 4]).unwrap();
    assert_eq!(writer.verify(100, None), Err(FirmwareError::VerifyFailed));
}

#[test]
fn mock_firmware_writer_abort() {
    use zigbee_runtime::firmware_writer::{FirmwareWriter, MockFirmwareWriter};

    let mut writer = MockFirmwareWriter::new(4096);
    writer.erase_slot().unwrap();
    writer.write_block(0, &[1, 2]).unwrap();
    writer.abort().unwrap();
    assert_eq!(writer.bytes_written(), 0);
    assert!(!writer.is_activated());
}

// ── OTA Manager tests ───────────────────────────────────────────

#[test]
fn ota_manager_full_download_flow() {
    use zigbee_runtime::firmware_writer::{FirmwareWriter, MockFirmwareWriter};
    use zigbee_runtime::ota::{OtaConfig, OtaManager};

    let writer = MockFirmwareWriter::new(4096);
    let config = OtaConfig {
        manufacturer_code: 0x1234,
        image_type: 0x0001,
        current_version: 0x00000001,
        endpoint: 1,
        block_size: 4,
        auto_accept: true,
    };
    let mut mgr = OtaManager::new(writer, config);

    // 1. Start query
    mgr.start_query();
    assert!(mgr.take_pending_frame().is_some()); // Query request queued

    // 2. Receive query response: image available, 12 bytes
    let mut resp = [0u8; 13];
    resp[0] = 0x00;
    resp[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
    resp[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
    resp[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
    resp[9..13].copy_from_slice(&12u32.to_le_bytes()); // 12 byte image
    mgr.handle_incoming(0x02, &resp);
    assert!(mgr.take_pending_frame().is_some()); // Block request queued

    // 3. Receive 3 blocks of 4 bytes each
    for i in 0..3 {
        let offset = i * 4;
        let mut block = [0u8; 18];
        block[0] = 0x00;
        block[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
        block[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
        block[5..9].copy_from_slice(&0x00000002u32.to_le_bytes());
        block[9..13].copy_from_slice(&(offset as u32).to_le_bytes());
        block[13] = 4;
        block[14..18].copy_from_slice(&[
            (i * 4 + 1) as u8,
            (i * 4 + 2) as u8,
            (i * 4 + 3) as u8,
            (i * 4 + 4) as u8,
        ]);

        let event = mgr.handle_incoming(0x05, &block);
        assert!(event.is_some()); // Progress event

        // Tick to process the write → next block request
        mgr.tick(0);
        // Should have a pending frame (next block request or end request)
    }

    // At this point download should be complete (12/12 bytes)
    // The last tick should have triggered verify + end request
    assert!(mgr.take_pending_frame().is_some()); // End request

    // 4. Receive upgrade end response: upgrade NOW
    let mut end_resp = [0u8; 16];
    end_resp[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
    end_resp[2..4].copy_from_slice(&0x0001u16.to_le_bytes());
    end_resp[4..8].copy_from_slice(&0x00000002u32.to_le_bytes());
    end_resp[8..12].copy_from_slice(&1000u32.to_le_bytes()); // current_time
    end_resp[12..16].copy_from_slice(&0u32.to_le_bytes()); // upgrade NOW

    let event = mgr.handle_incoming(0x07, &end_resp);
    // Should get OtaComplete
    assert!(event.is_some());
}
