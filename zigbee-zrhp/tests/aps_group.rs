//! `APS_GROUP` (`0x0D`) record codec: round trips at the codec's bounds,
//! truncation, and the record growth rule (spec 9.3).

use zigbee_zrhp::codec::{CodecError, Reader, Wire, Writer};
use zigbee_zrhp::records::{ApsGroupRecord, Record, record_type};

/// TLV bytes of `record`.
fn encode(record: &Record<'_>) -> Result<Vec<u8>, CodecError> {
    let mut buf = vec![0u8; 3 + 3 + 300];
    let mut w = Writer::new(&mut buf);
    record.write(&mut w)?;
    let len = w.len();
    buf.truncate(len);
    Ok(buf)
}

#[test]
fn round_trips_from_no_endpoint_to_the_codec_limit() {
    for n in [0usize, 1, 8, 9, 255] {
        let endpoints: Vec<u8> = (0..n).map(|i| (i % 240) as u8 + 1).collect();
        let record = Record::ApsGroup(ApsGroupRecord {
            group: 0x1234,
            endpoints: &endpoints,
        });
        let tlv = encode(&record).unwrap();
        // u8 type, u16 len, u16 group, u8 n, n × u8.
        assert_eq!(tlv.len(), 3 + 3 + n, "n {n}");
        assert_eq!(tlv[0], record_type::APS_GROUP);
        assert_eq!(u16::from_le_bytes([tlv[1], tlv[2]]) as usize, 3 + n);
        assert_eq!(&tlv[3..6], &[0x34, 0x12, n as u8]);
        let mut r = Reader::new(&tlv);
        assert_eq!(r.read::<Record<'_>>().unwrap(), record, "n {n}");
        assert!(r.is_empty());
    }
}

#[test]
fn more_than_255_endpoints_cannot_be_written() {
    let endpoints = [1u8; 256];
    let record = ApsGroupRecord {
        group: 0x0001,
        endpoints: &endpoints,
    };
    let mut buf = [0u8; 300];
    let mut w = Writer::new(&mut buf);
    assert_eq!(w.write(&record), Err(CodecError::Invalid));
    assert_eq!(
        encode(&Record::ApsGroup(record)),
        Err(CodecError::Invalid),
        "inside a TLV too"
    );
}

#[test]
fn a_count_beyond_the_value_is_truncated() {
    // n = 3 but only two endpoints in the value.
    let tlv = [record_type::APS_GROUP, 5, 0, 0x84, 0x0B, 3, 1, 2];
    assert_eq!(
        Reader::new(&tlv).read::<Record<'_>>(),
        Err(CodecError::Truncated)
    );
    // The group itself cut short.
    let tlv = [record_type::APS_GROUP, 1, 0, 0x84];
    assert_eq!(
        Reader::new(&tlv).read::<Record<'_>>(),
        Err(CodecError::Truncated)
    );
}

#[test]
fn appended_value_bytes_are_ignored() {
    // A later minor version may append fields (9.3): `group 0x0B84, n 1, 242`
    // followed by two unknown bytes.
    let tlv = [record_type::APS_GROUP, 6, 0, 0x84, 0x0B, 1, 242, 0xAA, 0xBB];
    let mut r = Reader::new(&tlv);
    let Record::ApsGroup(group) = r.read::<Record<'_>>().unwrap() else {
        panic!("not an APS_GROUP");
    };
    assert_eq!((group.group, group.endpoints), (0x0B84, &[242u8][..]));
    assert!(r.is_empty(), "the whole TLV is consumed");
}
