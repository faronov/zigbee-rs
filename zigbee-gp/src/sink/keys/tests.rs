use super::super::{Out, Sink, SinkConfig, SinkDrop, SinkOutcome, zcl_status};
use super::{MAX_OOB_KEYS, derive_shared_key, gpd_key_encryption};
use crate::Error;
use crate::cluster::{Response, attr, client_cmd};
use crate::commissioning::{CommissioningPayload, CommissioningReply, CommissioningSecurity};
use crate::frame::{FrameType, GpdfParams, parse, parse_mac};
use crate::proxy::TxFrame;
use crate::security::{self, DEFAULT_TC_LINK_KEY, Direction, WrappedKey};
use crate::types::{GpdId, KeyType, SecurityLevel, cmd};
use zigbee_crypto::{AesKey, SoftwareAesProvider};

const KEY_C0: AesKey = [
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
];
const GPD_KEY: AesKey = [0x5D; 16];
const SRC: u32 = 0x1234_5678;
const GPD: GpdId = GpdId::SrcId(SRC);
const OWN: u16 = 0x4A76;

type S = Sink<4, 4>;

fn sink() -> S {
    let mut s = S::new(SinkConfig {
        endpoint: Some(1),
        ..SinkConfig::default()
    });
    s.set_own_address(OWN, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
    s.set_operational_channel(15);
    s
}

/// gecko_sdk `green-power-security-test-vectors.c` (GP spec A.1.5.7.1,
/// A.1.5.7.2, A.1.5.12.1).
#[test]
fn shared_key_derivation_matches_the_spec_vectors() {
    let mut aes = SoftwareAesProvider;
    let nwk = [
        0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c,
        0x0d,
    ];
    assert_eq!(
        derive_shared_key(
            &mut aes,
            KeyType::NwkKeyDerivedGroupKey,
            &KEY_C0,
            Some(&nwk),
            &GPD
        ),
        Some([
            0xBA, 0x88, 0x86, 0x7F, 0xC0, 0x09, 0x39, 0x87, 0xEB, 0x88, 0x64, 0xCE, 0xBE, 0x5F,
            0xC6, 0x13
        ])
    );
    assert_eq!(
        derive_shared_key(
            &mut aes,
            KeyType::DerivedIndividualKey,
            &KEY_C0,
            None,
            &GpdId::SrcId(0x8765_4321)
        ),
        Some([
            0x7A, 0x3A, 0x73, 0x43, 0x8D, 0x6E, 0x47, 0x55, 0x28, 0x81, 0xA0, 0x28, 0xAD, 0x59,
            0x23, 0x2E
        ])
    );
    assert_eq!(
        derive_shared_key(
            &mut aes,
            KeyType::DerivedIndividualKey,
            &KEY_C0,
            None,
            &GpdId::Ieee {
                ieee: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
                endpoint: 0x0A
            }
        ),
        Some([
            0x8A, 0xE7, 0x5B, 0x07, 0x5F, 0x7A, 0x13, 0x23, 0x06, 0x08, 0xFF, 0x7E, 0x93, 0x07,
            0x97, 0x6D
        ])
    );
    assert_eq!(
        derive_shared_key(&mut aes, KeyType::NwkKey, &KEY_C0, Some(&nwk), &GPD),
        Some(nwk)
    );
    assert_eq!(
        derive_shared_key(&mut aes, KeyType::NwkKey, &KEY_C0, None, &GPD),
        None
    );
    assert_eq!(
        derive_shared_key(&mut aes, KeyType::GpdGroupKey, &KEY_C0, None, &GPD),
        Some(KEY_C0)
    );
    assert_eq!(
        derive_shared_key(&mut aes, KeyType::NoKey, &KEY_C0, None, &GPD),
        None
    );
}

/// A.1.5.8.3: the Commissioning Reply key for SrcID 0x12345678 with frame
/// counter 4 (the GPD's counter 3 + 1).
#[test]
fn the_reply_key_is_protected_as_the_spec_vector() {
    let s = sink();
    let (key, enc) = s
        .reply_key(&mut SoftwareAesProvider, &GPD, &KEY_C0, true, 3)
        .unwrap();
    assert_eq!(
        key,
        [
            0xE9, 0x00, 0x06, 0x63, 0x1D, 0x0D, 0xFD, 0xC6, 0x38, 0x06, 0x8E, 0x5E, 0x69, 0x67,
            0xD3, 0x25
        ]
    );
    assert_eq!(enc, Some((u32::from_le_bytes([0x27, 0x55, 0x9F, 0x75]), 4)));
    assert_eq!(
        s.reply_key(&mut SoftwareAesProvider, &GPD, &KEY_C0, false, 3),
        Some((KEY_C0, None))
    );
}

#[test]
fn key_attributes_are_writable_and_the_keys_write_only() {
    let mut s = sink();
    let mut out = [0u8; 16];
    let read = |s: &S, id, out: &mut [u8]| s.read_attribute(id, out).unwrap();
    assert_eq!(
        read(&s, attr::GP_SHARED_SECURITY_KEY_TYPE, &mut out),
        Some((0x18, 1))
    );
    assert_eq!(out[0], 0);
    assert_eq!(
        s.write_attribute(attr::GP_SHARED_SECURITY_KEY_TYPE, 0x18, &[7]),
        zcl_status::SUCCESS
    );
    assert_eq!(s.shared_key_type(), KeyType::DerivedIndividualKey);
    assert_eq!(
        s.write_attribute(attr::GP_SHARED_SECURITY_KEY_TYPE, 0x18, &[5]),
        zcl_status::INVALID_VALUE
    );
    assert_eq!(
        s.write_attribute(attr::GP_SHARED_SECURITY_KEY, 0xF1, &KEY_C0),
        zcl_status::SUCCESS
    );
    assert_eq!(
        s.write_attribute(attr::GP_SHARED_SECURITY_KEY, 0x18, &KEY_C0),
        zcl_status::INVALID_DATA_TYPE
    );
    assert_eq!(
        s.write_attribute(attr::GP_LINK_KEY, 0xF1, &KEY_C0),
        zcl_status::SUCCESS
    );
    assert_eq!(s.gp_link_key(), &KEY_C0);
    assert_eq!(read(&s, attr::GP_SHARED_SECURITY_KEY, &mut out), None);
    assert_eq!(read(&s, attr::GP_LINK_KEY, &mut out), None);
}

/// A Commissioning GPDF heard directly.
fn commissioning_gpdf(
    s: &mut S,
    seq: u8,
    rx_after_tx: bool,
    p: &CommissioningPayload,
) -> SinkOutcome {
    let mut b = [0u8; 64];
    let n = p.encode(&mut b).unwrap();
    let mut raw = [0u8; 127];
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        None,
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::None,
            individual_key: false,
            rx_after_tx,
            to_gpd: false,
            frame_counter: 0,
        },
        seq,
        cmd::COMMISSIONING,
        &b[..n],
        &mut raw,
    )
    .unwrap();
    let mac = parse_mac(&raw[..len]).unwrap();
    let frame = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &frame, u32::from(seq) * 10)
}

/// A level 3 GPDF protected with `key`.
fn protected_gpdf(s: &mut S, key: &AesKey, counter: u32, command: u8) -> SinkOutcome {
    let mut raw = [0u8; 127];
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        Some(key),
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::Encrypted,
            individual_key: true,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: counter,
        },
        counter as u8,
        command,
        &[],
        &mut raw,
    )
    .unwrap();
    let mac = parse_mac(&raw[..len]).unwrap();
    let frame = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &frame, 1_000 + counter)
}

fn payload(key: Option<(AesKey, Option<u32>)>, key_request: bool) -> CommissioningPayload {
    CommissioningPayload {
        device_id: 0x02,
        mac_seq_num_capability: true,
        rx_on_capability: key_request,
        pan_id_request: false,
        security_key_request: key_request,
        fixed_location: true,
        security: Some(CommissioningSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key,
            outgoing_counter: Some(20),
        }),
        app_info: None,
    }
}

/// The Commissioning Reply in the sink's GP Response.
fn reply(s: &mut S) -> CommissioningReply {
    while let Some(tx) = s.poll(100) {
        if let TxFrame::GpCommand {
            command_id: client_cmd::RESPONSE,
            payload,
        } = &tx.tx.frame
        {
            let r = Response::decode(payload).unwrap();
            assert_eq!(r.command_id, cmd::COMMISSIONING_REPLY);
            return CommissioningReply::decode(r.payload).unwrap();
        }
    }
    panic!("no GP Response");
}

#[test]
fn a_gpd_asking_for_a_key_gets_the_derived_individual_key() {
    let mut s = sink();
    s.set_direct_bidirectional(true);
    s.set_shared_key(KeyType::DerivedIndividualKey, KEY_C0);
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    let expected = derive_shared_key(
        &mut SoftwareAesProvider,
        KeyType::DerivedIndividualKey,
        &KEY_C0,
        None,
        &GPD,
    )
    .unwrap();
    // The GPD proposes its own key, TC-LK protected, and asks for one.
    let wrapped = security::wrap_key(
        &mut SoftwareAesProvider,
        &DEFAULT_TC_LINK_KEY,
        &GPD,
        Direction::FromGpd,
        0,
        &GPD_KEY,
    )
    .unwrap();
    let p = payload(
        Some((wrapped.key, Some(u32::from_le_bytes(wrapped.mic)))),
        true,
    );
    assert_eq!(
        commissioning_gpdf(&mut s, 1, true, &p),
        SinkOutcome::CommissioningReplySent
    );
    let r = reply(&mut s);
    assert_eq!(r.key_type, KeyType::DerivedIndividualKey);
    // Protected like the GPD's own key, with its outgoing counter + 1.
    let (key, Some((mic, fc))) = r.key.unwrap() else {
        panic!("the key must be TC-LK protected");
    };
    assert_eq!(fc, 21);
    let clear = security::unwrap_key(
        &mut SoftwareAesProvider,
        &DEFAULT_TC_LINK_KEY,
        &GPD,
        Direction::ToGpd,
        fc,
        &WrappedKey {
            key,
            mic: mic.to_le_bytes(),
        },
    )
    .unwrap();
    assert_eq!(clear, expected);
    // The GPD's Success, protected with the provided key, completes it.
    assert_eq!(
        protected_gpdf(&mut s, &expected, 21, cmd::SUCCESS),
        SinkOutcome::Commissioned
    );
    let e = &s.entries()[0];
    assert_eq!(
        (e.key, e.key_type),
        (Some(expected), KeyType::DerivedIndividualKey)
    );
}

#[test]
fn a_gpd_without_a_key_request_keeps_its_own_key() {
    let mut s = sink();
    s.set_direct_bidirectional(true);
    s.set_shared_key(KeyType::GpdGroupKey, KEY_C0);
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    let p = payload(Some((GPD_KEY, None)), false);
    assert_eq!(
        commissioning_gpdf(&mut s, 1, true, &p),
        SinkOutcome::CommissioningReplySent
    );
    let r = reply(&mut s);
    assert_eq!((r.key, r.key_type), (None, KeyType::IndividualKey));
}

#[test]
fn an_out_of_band_key_commissions_a_gpd_that_sends_none() {
    let mut s = sink();
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    // Without its key the protected GPD cannot commission.
    assert_eq!(
        commissioning_gpdf(&mut s, 1, false, &payload(None, false)),
        SinkOutcome::Dropped(SinkDrop::NoKey)
    );
    s.provision_oob_key(GPD, GPD_KEY).unwrap();
    assert_eq!(
        commissioning_gpdf(&mut s, 2, false, &payload(None, false)),
        SinkOutcome::Commissioned
    );
    assert_eq!(s.entries()[0].key, Some(GPD_KEY));
    // Used once; a re-commissioning without a key keeps the entry's key.
    assert!(!s.forget_oob_key(&GPD));
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    assert_eq!(
        commissioning_gpdf(&mut s, 3, false, &payload(None, false)),
        SinkOutcome::Commissioned
    );
    assert_eq!(s.entries()[0].key, Some(GPD_KEY));
    // The store is bounded and refuses reserved GPD IDs.
    for i in 0..MAX_OOB_KEYS as u32 {
        s.provision_oob_key(GpdId::SrcId(0x1000 + i), GPD_KEY)
            .unwrap();
    }
    assert_eq!(
        s.provision_oob_key(GpdId::SrcId(0x2000), GPD_KEY),
        Err(Error::BufferFull)
    );
    assert_eq!(
        s.provision_oob_key(GpdId::SrcId(0), GPD_KEY),
        Err(Error::Unsupported)
    );
}

#[test]
fn the_key_encryption_bit_is_read_from_the_extended_options() {
    assert!(gpd_key_encryption(&[0x02, 0x80, 0x40]));
    assert!(!gpd_key_encryption(&[0x02, 0x80, 0x00]));
    assert!(!gpd_key_encryption(&[0x02, 0x00, 0x40]));
    assert!(!gpd_key_encryption(&[0x02]));
    let _ = Out::DeviceAnnce(0);
}
