//! Compile-time check of the embassy-stm32-wpan MAC SAP structures against
//! the C layout CPU2 uses.
//!
//! Every size and offset below was produced by `arm-none-eabi-gcc` (Cortex-M4)
//! from ST's `802_15_4_mac_types.h` in `STMicroelectronics/stm32-mw-wpan` at
//! `1a66948ee871e7665712aa53dd3cbb9a71c4985a` (the STM32_WPAN middleware of
//! current STM32CubeWB, paired with the MAC 802.15.4 coprocessor binary).
//! A structure that drifts from the coprocessor ABI fails the build instead
//! of corrupting a mailbox exchange; that is how the `PanDescriptor` and
//! `StartRequest` defects patched in `vendor/embassy-stm32-wpan` were found.
//! `SetRequest` has no trailing `a_stuffing` field; its padding gives the same
//! 8-octet size and CPU2 ignores those octets. Fields that embassy keeps
//! private cannot be named by `offset_of!` here; they are pinned by the
//! structure size and the offsets of their public neighbours.

use core::mem::{offset_of, size_of};

use embassy_stm32_wpan::net::{commands, indications, responses, typedefs};

const _: () = {
    assert!(size_of::<typedefs::PanDescriptor>() == 24);
    assert!(offset_of!(typedefs::PanDescriptor, coord_pan_id) == 0); // a_coord_PAN_id
    assert!(offset_of!(typedefs::PanDescriptor, coord_addr_mode) == 2); // coord_addr_mode
    assert!(offset_of!(typedefs::PanDescriptor, logical_channel) == 3); // logical_channel
    assert!(offset_of!(typedefs::PanDescriptor, coord_addr) == 4); // coord_addr
    assert!(offset_of!(typedefs::PanDescriptor, channel_page) == 12); // channel_page
    assert!(offset_of!(typedefs::PanDescriptor, gts_permit) == 13); // gts_permit
    assert!(offset_of!(typedefs::PanDescriptor, superframe_spec) == 14); // a_superframe_spec
    assert!(offset_of!(typedefs::PanDescriptor, time_stamp) == 16); // a_time_stamp
    assert!(offset_of!(typedefs::PanDescriptor, link_quality) == 20); // link_quality
    assert!(offset_of!(typedefs::PanDescriptor, security_level) == 21); // security_level
    assert!(offset_of!(typedefs::PanDescriptor, a_stuffing) == 22); // a_stuffing
    assert!(size_of::<indications::BeaconNotifyIndication>() == 88);
    assert!(offset_of!(indications::BeaconNotifyIndication, sdu_ptr) == 0); // sduPtr
    assert!(offset_of!(indications::BeaconNotifyIndication, pan_descriptor) == 4); // PAN_descriptor
    assert!(offset_of!(indications::BeaconNotifyIndication, addr_list) == 28); // a_addr_list
    assert!(offset_of!(indications::BeaconNotifyIndication, bsn) == 84); // BSN
    assert!(offset_of!(indications::BeaconNotifyIndication, pend_addr_spec) == 85); // pend_addr_spec
    assert!(offset_of!(indications::BeaconNotifyIndication, sdu_length) == 86); // sdu_length
    assert!(size_of::<responses::ScanConfirm>() == 185);
    assert!(offset_of!(responses::ScanConfirm, status) == 0); // status
    assert!(offset_of!(responses::ScanConfirm, scan_type) == 1); // scan_type
    assert!(offset_of!(responses::ScanConfirm, channel_page) == 2); // channel_page
    assert!(offset_of!(responses::ScanConfirm, unscanned_channels) == 3); // a_unscanned_channels
    assert!(offset_of!(responses::ScanConfirm, result_list_size) == 7); // result_list_size
    assert!(offset_of!(responses::ScanConfirm, energy_detect_list) == 8); // a_energy_detect_list
    assert!(offset_of!(responses::ScanConfirm, pan_descriptor_list) == 24); // a_PAN_descriptor_list
    assert!(offset_of!(responses::ScanConfirm, detected_category) == 168); // detected_category
    assert!(offset_of!(responses::ScanConfirm, uwb_energy_detect_list) == 169); // a_UWB_energy_detect_list
    assert!(size_of::<indications::DataIndication>() == 68);
    assert!(offset_of!(indications::DataIndication, msdu_ptr) == 0); // msduPtr
    assert!(offset_of!(indications::DataIndication, src_addr_mode) == 4); // src_addr_mode
    assert!(offset_of!(indications::DataIndication, src_pan_id) == 5); // a_src_PAN_id
    assert!(offset_of!(indications::DataIndication, src_address) == 7); // src_address
    assert!(offset_of!(indications::DataIndication, dst_addr_mode) == 15); // dst_addr_mode
    assert!(offset_of!(indications::DataIndication, dst_pan_id) == 16); // a_dst_PAN_id
    assert!(offset_of!(indications::DataIndication, dst_address) == 18); // dst_address
    assert!(offset_of!(indications::DataIndication, msdu_length) == 26); // msdu_length
    assert!(offset_of!(indications::DataIndication, mpdu_link_quality) == 27); // mpdu_link_quality
    assert!(offset_of!(indications::DataIndication, dsn) == 28); // DSN
    assert!(offset_of!(indications::DataIndication, time_stamp) == 29); // a_time_stamp
    assert!(offset_of!(indications::DataIndication, key_source) == 35); // a_key_source
    assert!(offset_of!(indications::DataIndication, key_index) == 43); // key_index
    assert!(offset_of!(indications::DataIndication, uwbprf) == 44); // UWBPRF
    assert!(offset_of!(indications::DataIndication, uwn_preamble_symbol_repetitions) == 45); // UWB_preamble_symbol_repetitions
    assert!(offset_of!(indications::DataIndication, datrate) == 46); // data_rate
    assert!(offset_of!(indications::DataIndication, ranging_received) == 47); // ranging_received
    assert!(offset_of!(indications::DataIndication, ranging_counter_start) == 48); // ranging_counter_start
    assert!(offset_of!(indications::DataIndication, ranging_counter_stop) == 52); // ranging_counter_stop
    assert!(offset_of!(indications::DataIndication, ranging_tracking_interval) == 56); // ranging_tracking_interval
    assert!(offset_of!(indications::DataIndication, ranging_offset) == 60); // ranging_offset
    assert!(offset_of!(indications::DataIndication, ranging_fom) == 64); // ranging_FOM
    assert!(offset_of!(indications::DataIndication, rssi) == 65); // rssi
    assert!(size_of::<indications::PollIndication>() == 9);
    assert!(offset_of!(indications::PollIndication, addr_mode) == 0); // addr_mode
    assert!(offset_of!(indications::PollIndication, request_address) == 1); // request_address
    assert!(size_of::<indications::AssociateIndication>() == 20);
    assert!(offset_of!(indications::AssociateIndication, device_address) == 0); // a_device_address
    assert!(offset_of!(indications::AssociateIndication, capability_information) == 8); // capability_information
    assert!(offset_of!(indications::AssociateIndication, security_level) == 9); // security_level
    assert!(offset_of!(indications::AssociateIndication, key_id_mode) == 10); // key_id_mode
    assert!(offset_of!(indications::AssociateIndication, key_index) == 11); // key_index
    assert!(offset_of!(indications::AssociateIndication, key_source) == 12); // a_key_source
    assert!(size_of::<indications::CommStatusIndication>() == 32);
    assert!(offset_of!(indications::CommStatusIndication, pan_id) == 0); // a_PAN_id
    assert!(offset_of!(indications::CommStatusIndication, src_addr_mode) == 2); // src_addr_mode
    assert!(offset_of!(indications::CommStatusIndication, dst_addr_mode) == 3); // dst_addr_mode
    assert!(offset_of!(indications::CommStatusIndication, src_address) == 4); // src_address
    assert!(offset_of!(indications::CommStatusIndication, dst_address) == 12); // dst_address
    assert!(offset_of!(indications::CommStatusIndication, status) == 20); // status
    assert!(offset_of!(indications::CommStatusIndication, security_level) == 21); // security_level
    assert!(offset_of!(indications::CommStatusIndication, key_id_mode) == 22); // key_id_mode
    assert!(offset_of!(indications::CommStatusIndication, key_index) == 23); // key_index
    assert!(offset_of!(indications::CommStatusIndication, key_source) == 24); // a_key_source
    assert!(size_of::<responses::AssociateConfirm>() == 16);
    assert!(offset_of!(responses::AssociateConfirm, assoc_short_address) == 0); // a_assoc_short_address
    assert!(offset_of!(responses::AssociateConfirm, status) == 2); // status
    assert!(offset_of!(responses::AssociateConfirm, security_level) == 3); // security_level
    assert!(offset_of!(responses::AssociateConfirm, key_source) == 4); // a_key_source
    assert!(offset_of!(responses::AssociateConfirm, key_id_mode) == 12); // key_id_mode
    assert!(offset_of!(responses::AssociateConfirm, key_index) == 13); // key_index
    assert!(size_of::<responses::DataConfirm>() == 28);
    assert!(offset_of!(responses::DataConfirm, msdu_handle) == 0); // msdu_handle
    assert!(offset_of!(responses::DataConfirm, time_stamp) == 1); // a_time_stamp
    assert!(offset_of!(responses::DataConfirm, ranging_received) == 5); // ranging_received
    assert!(offset_of!(responses::DataConfirm, status) == 6); // status
    assert!(offset_of!(responses::DataConfirm, ranging_counter_start) == 8); // ranging_counter_start
    assert!(offset_of!(responses::DataConfirm, ranging_counter_stop) == 12); // ranging_counter_stop
    assert!(offset_of!(responses::DataConfirm, ranging_tracking_interval) == 16); // ranging_tracking_interval
    assert!(offset_of!(responses::DataConfirm, ranging_offset) == 20); // ranging_offset
    assert!(offset_of!(responses::DataConfirm, ranging_fom) == 24); // ranging_FOM
    assert!(size_of::<commands::StartRequest>() == 35);
    assert!(offset_of!(commands::StartRequest, pan_id) == 0); // a_PAN_id
    assert!(offset_of!(commands::StartRequest, channel_number) == 2); // channel_number
    assert!(offset_of!(commands::StartRequest, channel_page) == 3); // channel_page
    assert!(offset_of!(commands::StartRequest, start_time) == 4); // a_start_time
    assert!(offset_of!(commands::StartRequest, beacon_order) == 8); // beacon_order
    assert!(offset_of!(commands::StartRequest, superframe_order) == 9); // superframe_order
    assert!(offset_of!(commands::StartRequest, pan_coordinator) == 10); // PAN_coordinator
    assert!(offset_of!(commands::StartRequest, battery_life_extension) == 11); // battery_life_extension
    assert!(offset_of!(commands::StartRequest, coord_realignment) == 12); // coord_realignment
    assert!(offset_of!(commands::StartRequest, coord_realign_security_level) == 13); // coord_realign_security_level
    assert!(offset_of!(commands::StartRequest, coord_realign_key_id_mode) == 14); // coord_realign_key_id_mode
    assert!(offset_of!(commands::StartRequest, coord_realign_key_id_index) == 15); // coord_realign_key_index
    assert!(offset_of!(commands::StartRequest, coord_realign_key_source) == 16); // a_coord_realign_key_source
    assert!(offset_of!(commands::StartRequest, beacon_security_level) == 24); // beacon_security_level
    assert!(offset_of!(commands::StartRequest, beacon_key_id_mode) == 25); // beacon_key_id_mode
    assert!(offset_of!(commands::StartRequest, beacon_key_index) == 26); // beacon_key_index
    assert!(offset_of!(commands::StartRequest, beacon_key_source) == 27); // a_beacon_key_source
    assert!(size_of::<commands::DataRequest>() == 36);
    assert!(offset_of!(commands::DataRequest, msdu_ptr) == 0); // msduPtr
    assert!(offset_of!(commands::DataRequest, src_addr_mode) == 4); // src_addr_mode
    assert!(offset_of!(commands::DataRequest, dst_addr_mode) == 5); // dst_addr_mode
    assert!(offset_of!(commands::DataRequest, dst_pan_id) == 6); // a_dst_PAN_id
    assert!(offset_of!(commands::DataRequest, dst_address) == 8); // dst_address
    assert!(offset_of!(commands::DataRequest, msdu_length) == 16); // msdu_length
    assert!(offset_of!(commands::DataRequest, msdu_handle) == 17); // msdu_handle
    assert!(offset_of!(commands::DataRequest, ack_tx) == 18); // ack_Tx
    assert!(offset_of!(commands::DataRequest, gts_tx) == 19); // GTS_Tx
    assert!(offset_of!(commands::DataRequest, indirect_tx) == 20); // indirect_Tx
    assert!(offset_of!(commands::DataRequest, security_level) == 21); // security_level
    assert!(offset_of!(commands::DataRequest, key_id_mode) == 22); // key_id_mode
    assert!(offset_of!(commands::DataRequest, key_index) == 23); // key_index
    assert!(offset_of!(commands::DataRequest, key_source) == 24); // a_key_source
    assert!(offset_of!(commands::DataRequest, uwbprf) == 32); // UWBPRF
    assert!(offset_of!(commands::DataRequest, ranging) == 33); // ranging
    assert!(offset_of!(commands::DataRequest, uwb_preamble_symbol_repetitions) == 34); // UWB_preamble_symbol_repetitions
    assert!(offset_of!(commands::DataRequest, datrate) == 35); // data_rate
    assert!(size_of::<commands::AssociateRequest>() == 25);
    assert!(offset_of!(commands::AssociateRequest, channel_number) == 0); // channel_number
    assert!(offset_of!(commands::AssociateRequest, channel_page) == 1); // channel_page
    assert!(offset_of!(commands::AssociateRequest, coord_addr_mode) == 2); // coord_addr_mode
    assert!(offset_of!(commands::AssociateRequest, capability_information) == 3); // capability_information
    assert!(offset_of!(commands::AssociateRequest, coord_pan_id) == 4); // a_coord_PAN_id
    assert!(offset_of!(commands::AssociateRequest, security_level) == 6); // security_level
    assert!(offset_of!(commands::AssociateRequest, key_id_mode) == 7); // key_id_mode
    assert!(offset_of!(commands::AssociateRequest, key_source) == 8); // a_key_source
    assert!(offset_of!(commands::AssociateRequest, coord_address) == 16); // coord_address
    assert!(offset_of!(commands::AssociateRequest, key_index) == 24); // key_index
    assert!(size_of::<commands::AssociateResponse>() == 24);
    assert!(offset_of!(commands::AssociateResponse, device_address) == 0); // a_device_address
    assert!(offset_of!(commands::AssociateResponse, assoc_short_address) == 8); // a_assoc_short_address
    assert!(offset_of!(commands::AssociateResponse, status) == 10); // status
    assert!(offset_of!(commands::AssociateResponse, security_level) == 11); // security_level
    assert!(offset_of!(commands::AssociateResponse, key_source) == 12); // a_key_source
    assert!(offset_of!(commands::AssociateResponse, key_id_mode) == 20); // key_id_mode
    assert!(offset_of!(commands::AssociateResponse, key_index) == 21); // key_index
    assert!(offset_of!(commands::AssociateResponse, a_stuffing) == 22); // a_stuffing
    assert!(size_of::<commands::ScanRequest>() == 20);
    assert!(offset_of!(commands::ScanRequest, scan_type) == 0); // scan_type
    assert!(offset_of!(commands::ScanRequest, scan_duration) == 1); // scan_duration
    assert!(offset_of!(commands::ScanRequest, channel_page) == 2); // channel_page
    assert!(offset_of!(commands::ScanRequest, security_level) == 3); // security_level
    assert!(offset_of!(commands::ScanRequest, scan_channels) == 4); // a_scan_channels
    assert!(offset_of!(commands::ScanRequest, key_source) == 8); // a_key_source
    assert!(offset_of!(commands::ScanRequest, key_id_mode) == 16); // key_id_mode
    assert!(offset_of!(commands::ScanRequest, key_index) == 17); // key_index
    assert!(offset_of!(commands::ScanRequest, a_stuffing) == 18); // a_stuffing
    assert!(size_of::<commands::PollRequest>() == 24);
    assert!(offset_of!(commands::PollRequest, coord_addr_mode) == 0); // coord_addr_mode
    assert!(offset_of!(commands::PollRequest, security_level) == 1); // security_level
    assert!(offset_of!(commands::PollRequest, key_id_mode) == 2); // key_id_mode
    assert!(offset_of!(commands::PollRequest, key_index) == 3); // key_index
    assert!(offset_of!(commands::PollRequest, coord_address) == 4); // coord_address
    assert!(offset_of!(commands::PollRequest, key_source) == 12); // a_key_source
    assert!(offset_of!(commands::PollRequest, coord_pan_id) == 20); // a_coord_PAN_id
    assert!(offset_of!(commands::PollRequest, a_stuffing) == 22); // a_stuffing
    assert!(size_of::<commands::SetRequest>() == 8);
    assert!(offset_of!(commands::SetRequest, pib_attribute_ptr) == 0); // PIB_attribute_valuePtr
    assert!(offset_of!(commands::SetRequest, pib_attribute) == 4); // PIB_attribute
    assert!(size_of::<commands::DisassociateRequest>() == 24);
    assert!(offset_of!(commands::DisassociateRequest, device_addr_mode) == 0); // device_addr_mode
    assert!(offset_of!(commands::DisassociateRequest, device_pan_id) == 1); // a_device_PAN_id
    assert!(offset_of!(commands::DisassociateRequest, disassociation_reason) == 3); // disassociate_reason
    assert!(offset_of!(commands::DisassociateRequest, device_address) == 4); // device_address
    assert!(offset_of!(commands::DisassociateRequest, tx_indirect) == 12); // tx_Indirect
    assert!(offset_of!(commands::DisassociateRequest, security_level) == 13); // security_level
    assert!(offset_of!(commands::DisassociateRequest, key_id_mode) == 14); // key_id_mode
    assert!(offset_of!(commands::DisassociateRequest, key_index) == 15); // key_index
    assert!(offset_of!(commands::DisassociateRequest, key_source) == 16); // a_key_source
    assert!(size_of::<commands::ResetRequest>() == 4);
    assert!(offset_of!(commands::ResetRequest, set_default_pib) == 0); // set_default_PIB
    assert!(offset_of!(commands::ResetRequest, a_stuffing) == 1); // a_stuffing
    assert!(size_of::<commands::PurgeRequest>() == 4);
    assert!(offset_of!(commands::PurgeRequest, msdu_handle) == 0); // msdu_handle
    assert!(offset_of!(commands::PurgeRequest, a_stuffing) == 1); // a_stuffing
    assert!(size_of::<responses::StartConfirm>() == 4);
    assert!(offset_of!(responses::StartConfirm, status) == 0); // status
    assert!(size_of::<responses::SetConfirm>() == 4);
    assert!(offset_of!(responses::SetConfirm, status) == 0); // status
    assert!(offset_of!(responses::SetConfirm, pin_attribute) == 1); // PIB_attribute
    assert!(size_of::<responses::ResetConfirm>() == 4);
    assert!(offset_of!(responses::ResetConfirm, status) == 0); // status
    assert!(size_of::<responses::PurgeConfirm>() == 4);
    assert!(offset_of!(responses::PurgeConfirm, msdu_handle) == 0); // msdu_handle
    assert!(offset_of!(responses::PurgeConfirm, status) == 1); // status
    assert!(size_of::<responses::PollConfirm>() == 4);
    assert!(offset_of!(responses::PollConfirm, status) == 0); // status
    assert!(size_of::<responses::DisassociateConfirm>() == 12);
    assert!(offset_of!(responses::DisassociateConfirm, status) == 0); // status
    assert!(offset_of!(responses::DisassociateConfirm, device_addr_mode) == 1); // device_addr_mode
    assert!(offset_of!(responses::DisassociateConfirm, device_pan_id) == 2); // a_device_PAN_id
    assert!(offset_of!(responses::DisassociateConfirm, device_address) == 4); // device_address
    assert!(size_of::<indications::DisassociateIndication>() == 20);
    assert!(offset_of!(indications::DisassociateIndication, device_address) == 0); // a_device_address
    assert!(offset_of!(indications::DisassociateIndication, disassociation_reason) == 8); // disassociate_reason
    assert!(offset_of!(indications::DisassociateIndication, security_level) == 9); // security_level
    assert!(offset_of!(indications::DisassociateIndication, key_id_mode) == 10); // key_id_mode
    assert!(offset_of!(indications::DisassociateIndication, key_index) == 11); // key_index
    assert!(offset_of!(indications::DisassociateIndication, key_source) == 12); // a_key_source
};
