use super::*;
use crate::proxy::TxFrame;
use zigbee_crypto::SoftwareAesProvider;

const SRC: u32 = 0x7A6B_9879;
const GPD: GpdId = GpdId::SrcId(SRC);
const KEY: AesKey = [0x5A; 16];
const PROXY: u16 = 0x6D6B;

type S = Sink<4, 4>;

fn paired(mode: CommunicationMode) -> S {
    let mut s = S::new(SinkConfig::default());
    s.set_own_address(0x7C87, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
    let mut e = SinkEntry::new(GPD);
    e.communication_mode = mode;
    e.device_id = 0x02;
    e.security_level = SecurityLevel::Encrypted;
    e.key_type = KeyType::IndividualKey;
    e.key = Some(KEY);
    e.frame_counter = 0x1160;
    if mode == CommunicationMode::CommissionedGroupcast {
        e.groups
            .push(SinkGroup {
                group: 0x0B84,
                alias: None,
            })
            .unwrap();
    }
    s.import(e, false).unwrap();
    while s.poll(0).is_some() {}
    s
}

fn search(s: &mut S, raw: &[u8]) -> Option<(DestAddr, Pairing)> {
    let command = ServerCommand {
        src_nwk: PROXY,
        unicast: false,
        tsn: 0x10,
        command_id: server_cmd::PAIRING_SEARCH,
        payload: raw,
    };
    let result = s.on_server_command(&mut SoftwareAesProvider, &command, 0);
    assert_eq!(result, Ok((zcl_status::SUCCESS, None)));
    let tx = s.poll(0)?.tx;
    assert!(s.poll(0).is_none());
    let TxFrame::GpCommand {
        command_id,
        payload,
    } = &tx.frame
    else {
        panic!("unexpected frame {:?}", tx.frame);
    };
    assert_eq!(*command_id, client_cmd::PAIRING);
    Some((tx.dest, Pairing::decode(payload).unwrap()))
}

#[test]
fn answers_a_matching_search_to_the_proxy() {
    let mut s = paired(CommunicationMode::DerivedGroupcast);
    // gecko_sdk layout: options 0x00F8 (every sink kind, counter and key),
    // SrcID 0x7a6b9879.
    let (dest, p) = search(&mut s, &[0xF8, 0x00, 0x79, 0x98, 0x6B, 0x7A]).unwrap();
    assert_eq!(
        dest,
        DestAddr::Unicast {
            nwk: PROXY,
            ieee: None
        }
    );
    assert!(p.add_sink && !p.remove_gpd);
    assert_eq!(p.gpd_id, GPD);
    assert_eq!(p.communication_mode, CommunicationMode::DerivedGroupcast);
    assert_eq!((p.frame_counter, p.key), (Some(0x1160), Some(KEY)));
    assert_eq!(p.device_id, Some(0x02));
}

#[test]
fn counter_and_key_only_when_requested() {
    let mut s = paired(CommunicationMode::CommissionedGroupcast);
    // Commissioned groupcast sinks, no counter, no key.
    let (_, p) = search(&mut s, &[0x20, 0x00, 0x79, 0x98, 0x6B, 0x7A]).unwrap();
    assert_eq!(p.sink, PairingSink::Group(0x0B84));
    assert_eq!((p.frame_counter, p.key), (None, None));
    let (_, p) = search(&mut s, &[0x60, 0x00, 0x79, 0x98, 0x6B, 0x7A]).unwrap();
    assert_eq!((p.frame_counter, p.key), (Some(0x1160), None));
}

#[test]
fn no_answer_without_a_matching_pairing() {
    let mut s = paired(CommunicationMode::LightweightUnicast);
    // Groupcast sinks only.
    assert!(search(&mut s, &[0x30, 0x00, 0x79, 0x98, 0x6B, 0x7A]).is_none());
    // Unknown GPD.
    assert!(search(&mut s, &[0xF8, 0x00, 0x01, 0x00, 0x00, 0x00]).is_none());
    // Unicast sinks: answered with this sink's address.
    let (_, p) = search(&mut s, &[0x08, 0x00, 0x79, 0x98, 0x6B, 0x7A]).unwrap();
    assert_eq!(
        p.sink,
        PairingSink::Unicast {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            nwk: 0x7C87
        }
    );
    // Malformed.
    let command = ServerCommand {
        src_nwk: PROXY,
        unicast: false,
        tsn: 0x10,
        command_id: server_cmd::PAIRING_SEARCH,
        payload: &[0x08, 0x00, 0x79],
    };
    assert!(
        s.on_server_command(&mut SoftwareAesProvider, &command, 0)
            .is_err()
    );
}
