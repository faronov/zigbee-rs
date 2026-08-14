//! Pure Telink factory-EUI layout helpers shared by the TC32 flash driver
//! and host tests.

const GENERATED_IEEE_SUFFIX: [u8; 3] = [0x38, 0xC1, 0xA4];

pub(crate) const fn decode_telink_ieee(raw: [u8; 8]) -> [u8; 8] {
    [
        raw[6], raw[7], raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
    ]
}

#[cfg(test)]
const fn encode_telink_ieee(ieee: [u8; 8]) -> [u8; 8] {
    [
        ieee[2], ieee[3], ieee[4], ieee[5], ieee[6], ieee[7], ieee[0], ieee[1],
    ]
}

pub(crate) const fn valid_factory_bytes(raw: [u8; 8]) -> bool {
    matches!(
        (raw[3], raw[4], raw[5]),
        (0x38, 0xC1, 0xA4)
            | (0xD1, 0x19, 0xC4)
            | (0xCB, 0x0B, 0xD8)
            | (0x77, 0x5F, 0xD8)
            | (0xB4, 0xCF, 0x3C)
            | (0xC7, 0xA3, 0xC0)
            | (0x28, 0x22, 0x38)
    )
}

pub(crate) const fn generated_telink_ieee(uid: &[u8; 16]) -> [u8; 8] {
    [
        uid[0],
        uid[1],
        uid[2],
        uid[3],
        uid[4],
        GENERATED_IEEE_SUFFIX[0],
        GENERATED_IEEE_SUFFIX[1],
        GENERATED_IEEE_SUFFIX[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_telink_factory_ieee() {
        let raw = [0xDB, 0x77, 0x69, 0x38, 0xC1, 0xA4, 0x55, 0xED];

        assert_eq!(
            decode_telink_ieee(raw),
            [0x55, 0xED, 0xDB, 0x77, 0x69, 0x38, 0xC1, 0xA4]
        );
    }

    #[test]
    fn generated_ieee_has_telink_prefix() {
        let uid = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let ieee = generated_telink_ieee(&uid);

        assert_eq!(ieee, [1, 2, 3, 4, 5, 0x38, 0xC1, 0xA4]);
        assert_eq!(
            [
                ieee[7], ieee[6], ieee[5], ieee[4], ieee[3], ieee[2], ieee[1], ieee[0],
            ],
            [0xA4, 0xC1, 0x38, 5, 4, 3, 2, 1]
        );
    }

    #[test]
    fn telink_ieee_encoding_round_trips() {
        let raw = [0xDB, 0x77, 0x69, 0x38, 0xC1, 0xA4, 0x55, 0xED];

        assert_eq!(encode_telink_ieee(decode_telink_ieee(raw)), raw);
    }

    #[test]
    fn validates_only_vendor_factory_prefixes() {
        for prefix in [
            [0x38, 0xC1, 0xA4],
            [0xD1, 0x19, 0xC4],
            [0xCB, 0x0B, 0xD8],
            [0x77, 0x5F, 0xD8],
            [0xB4, 0xCF, 0x3C],
            [0xC7, 0xA3, 0xC0],
            [0x28, 0x22, 0x38],
        ] {
            assert!(valid_factory_bytes([
                1, 2, 3, prefix[0], prefix[1], prefix[2], 4, 5,
            ]));
        }

        assert!(!valid_factory_bytes([0xFF; 8]));
        assert!(!valid_factory_bytes([0; 8]));
        assert!(!valid_factory_bytes([1, 2, 3, 4, 5, 6, 7, 8]));
    }
}
