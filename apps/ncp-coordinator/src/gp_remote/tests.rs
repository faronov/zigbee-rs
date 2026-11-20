//! Request mapping of remote GP provisioning.

use super::*;
use zigbee_gp::types::GpdId;
use zigbee_zrhp::codec::List;
use zigbee_zrhp::msg::gp::GpGroup;
use zigbee_zrhp::records::GpdId as ZGpdId;
use zigbee_zrhp::types::Ieee;

fn request() -> GpPairingConfiguration<'static> {
    GpPairingConfiguration {
        dst: 0x7C87,
        actions: 0x0A,
        gpd_id: ZGpdId::SrcId(0x7A6B_9879),
        device_id: 0x02,
        communication_mode: 1,
        flags: pairing_config_flags::SECURITY
            | pairing_config_flags::MAC_SEQ_CAPABLE
            | pairing_config_flags::FIXED_LOCATION,
        assigned_alias: 0,
        groupcast_radius: 0,
        security_level: 3,
        key_type: 4,
        key: [0x5A; 16],
        frame_counter: 10,
        groups: List::new(&[]),
        paired: paired_endpoints::SINK_DERIVES,
        endpoints: List::new(&[]),
        app_info: 0,
        manufacturer_id: 0,
        model_id: 0,
        gpd_commands: List::new(&[]),
        server_clusters: List::new(&[]),
        client_clusters: List::new(&[]),
    }
}

#[test]
fn pairing_configuration_maps_to_the_gp_command() {
    let mut out = [0u8; 79];
    let n = pairing_configuration(&request(), &mut out).unwrap();
    let pc = PairingConfiguration::decode(&out[..n]).unwrap();
    assert_eq!(pc.actions, 0x0A);
    assert_eq!(pc.gpd_id, GpdId::SrcId(0x7A6B_9879));
    assert_eq!(pc.communication_mode, CommunicationMode::DerivedGroupcast);
    assert_eq!(pc.frame_counter, Some(10));
    let s = pc.security.unwrap();
    assert_eq!(
        (s.level, s.key_type, s.key),
        (SecurityLevel::Encrypted, KeyType::IndividualKey, [0x5A; 16])
    );
    assert_eq!(pc.number_of_paired_endpoints, 0xFE);
    assert!(pc.mac_seq_cap && pc.fixed_location && !pc.rx_on_cap);

    // Pre-commissioned groupcast with a group list, listed endpoints and
    // application information.
    let groups = [GpGroup {
        group: 0x0B84,
        alias: 0xFFFF,
    }];
    let endpoints = [1u8, 2];
    let commands = [0x20u8, 0x21, 0x22];
    let mut r = request();
    r.communication_mode = 2;
    r.groups = List::new(&groups);
    r.paired = paired_endpoints::LISTED;
    r.endpoints = List::new(&endpoints);
    r.app_info = 0x05;
    r.manufacturer_id = 0x126B;
    r.gpd_commands = List::new(&commands);
    let n = pairing_configuration(&r, &mut out).unwrap();
    let pc = PairingConfiguration::decode(&out[..n]).unwrap();
    assert_eq!(pc.group_list().next(), Some((0x0B84, 0xFFFF)));
    assert_eq!(pc.paired_endpoints, &[1, 2]);
    let info = pc.application_information.unwrap();
    assert_eq!((info.manufacturer_id, info.model_id), (Some(0x126B), None));
    assert_eq!(info.gpd_commands, &commands);
}

#[test]
fn invalid_pairing_configurations_are_refused() {
    let mut out = [0u8; 79];
    let mut r = request();
    r.actions = 0x05; // Application Description
    assert_eq!(
        pairing_configuration(&r, &mut out),
        Err(Status::InvalidParameter)
    );
    let mut r = request();
    r.security_level = 1;
    assert_eq!(
        pairing_configuration(&r, &mut out),
        Err(Status::InvalidParameter)
    );
    let mut r = request();
    r.communication_mode = 2; // groupcast without groups
    assert_eq!(
        pairing_configuration(&r, &mut out),
        Err(Status::InvalidParameter)
    );
    let mut r = request();
    r.paired = paired_endpoints::LISTED; // no endpoint listed
    assert_eq!(
        pairing_configuration(&r, &mut out),
        Err(Status::InvalidParameter)
    );
    let mut r = request();
    r.app_info = 0x10; // switch information is not provisioned
    assert_eq!(
        pairing_configuration(&r, &mut out),
        Err(Status::InvalidParameter)
    );
    assert_eq!(
        destination(0x0000, 0x0000, false),
        Err(Status::InvalidParameter)
    );
    assert_eq!(
        destination(0xFFFD, 0x0000, true),
        Err(Status::InvalidParameter)
    );
    assert_eq!(
        destination(0xFFFD, 0x0000, false),
        Ok(Destination::Broadcast(0xFFFD))
    );
    assert_eq!(
        destination(0x7C87, 0x0000, true),
        Ok(Destination::Nwk(0x7C87))
    );
}

#[test]
fn send_pairing_maps_flags_and_sink_addressing() {
    let r = GpSendPairing {
        dst: 0xFFFD,
        flags: send_pairing_flags::ADD_SINK
            | send_pairing_flags::KEY
            | send_pairing_flags::FRAME_COUNTER
            | send_pairing_flags::MAC_SEQ_CAPABLE,
        gpd_id: ZGpdId::SrcId(0x7A6B_9879),
        communication_mode: 3,
        security_level: 3,
        key_type: 4,
        sink_ieee: Ieee([1, 2, 3, 4, 5, 6, 7, 8]),
        sink_nwk: 0x7C87,
        sink_group: 0,
        device_id: 0x02,
        frame_counter: 10,
        key: [0x5A; 16],
        assigned_alias: 0,
        groupcast_radius: 0,
    };
    let p = send_pairing(&r).unwrap();
    assert!(p.add_sink && !p.remove_gpd);
    assert_eq!(
        p.sink,
        PairingSink::Unicast {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            nwk: 0x7C87
        }
    );
    assert_eq!(
        (p.key, p.frame_counter, p.device_id),
        (Some([0x5A; 16]), Some(10), Some(2))
    );
    let mut out = [0u8; 79];
    let n = p.encode(&mut out).unwrap();
    assert_eq!(Pairing::decode(&out[..n]).unwrap(), p);
    // Remove GPD: no sink fields.
    let remove = GpSendPairing {
        flags: send_pairing_flags::REMOVE_GPD,
        ..r
    };
    let p = send_pairing(&remove).unwrap();
    assert_eq!(p.sink, PairingSink::None);
    assert!(p.encode(&mut out).is_ok());
    // A key without AddSink does not encode.
    let bad = GpSendPairing {
        flags: send_pairing_flags::KEY,
        ..r
    };
    assert!(send_pairing(&bad).unwrap().encode(&mut out).is_err());
}

#[test]
fn translations_map_both_ways() {
    let t = GpTranslation {
        gpd_command: 0x22,
        endpoint: 2,
        profile: 0x0104,
        cluster: 0x0008,
        zcl_command: 0x04,
        payload_len: 3,
        payload: &[0x80, 0x0A, 0x00],
    };
    let r = translation_record(&t);
    assert_eq!(r.payload, TtPayload::Fixed(&[0x80, 0x0A, 0x00]));
    assert_eq!(zrhp_translation(&r), t);
    for len in [
        gp_translation_payload::DERIVED,
        gp_translation_payload::COPY_GPD,
    ] {
        let t = GpTranslation {
            payload_len: len,
            payload: &[],
            ..t
        };
        assert_eq!(zrhp_translation(&translation_record(&t)), t);
    }
}
