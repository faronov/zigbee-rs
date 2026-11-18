//! Frame Control Field readers for the length-stripped PSDU (local patch
//! `ieee802154-fcf-offsets`, backport of esp-hal #5650; also exercised by
//! source-linked host tests).
//!
//! Every caller passes the MAC frame without the PHY length byte: TX frames
//! start one byte past the PHR and RX frames are `RX_BUFFER[1..]`. Byte 0 is
//! FCF octet 0, which carries the AR bit; FCF octet 1 carries the frame
//! version. (ESP-IDF uses offsets 1 and 2 because its frame pointers include
//! the PHR.)

pub(crate) const FRAME_VERSION_1: u8 = 0x10; // IEEE 802.15.4 - 2006 & 2011
pub(crate) const FRAME_VERSION_2: u8 = 0x20; // IEEE 802.15.4 - 2015

const FRAME_AR_OFFSET: usize = 0;
const FRAME_AR_BIT: u8 = 0x20;
const FRAME_VERSION_OFFSET: usize = 1;
const FRAME_VERSION_MASK: u8 = 0x30;

pub(crate) fn frame_is_ack_required(frame: &[u8]) -> bool {
    // Keep the offset-based bounds check (same codegen as the shipped image);
    // FRAME_AR_OFFSET is 0, which clippy flags as an extreme comparison.
    #[allow(clippy::absurd_extreme_comparisons)]
    if frame.len() <= FRAME_AR_OFFSET {
        return false;
    }
    (frame[FRAME_AR_OFFSET] & FRAME_AR_BIT) != 0
}

pub(crate) fn frame_get_version(frame: &[u8]) -> u8 {
    if frame.len() <= FRAME_VERSION_OFFSET {
        return 0;
    }
    frame[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Join-time Data Request: cmd, AR, PAN-ID compression, short
    /// destination, extended source, 2003 version (FCF 0xC863).
    const DATA_REQUEST_EXT_SRC: [u8; 18] = [
        0x63, 0xC8, 0x2A, 0xE9, 0xDF, 0x7F, 0xA1, 0x35, 0xAB, 0x40, 0xFE, 0xFF, 0x14, 0x2E, 0x84,
        0x04, 0xC0, 0x3F,
    ];

    #[test]
    fn data_request_requires_a_2006_auto_ack() {
        assert!(frame_is_ack_required(&DATA_REQUEST_EXT_SRC));
        assert!(frame_get_version(&DATA_REQUEST_EXT_SRC) <= FRAME_VERSION_1);
    }

    #[test]
    fn ack_request_is_read_from_fcf_octet_0() {
        // Zigbee unicast data, 2003 (FCF 0x8861) and 2006 (FCF 0x9861): the
        // old offset read octet 1, whose bit 5 is clear for both.
        assert!(frame_is_ack_required(&[0x61, 0x88]));
        assert!(frame_is_ack_required(&[0x61, 0x98]));
        // Broadcast data and a beacon request: no ACK.
        assert!(!frame_is_ack_required(&[0x41, 0x88]));
        assert!(!frame_is_ack_required(&[0x03, 0x08]));
        // Octet 1 bit 5 (version bit 1) alone is not an ACK request.
        assert!(!frame_is_ack_required(&[0x41, 0xEC]));
    }

    #[test]
    fn version_is_read_from_fcf_octet_1() {
        assert_eq!(frame_get_version(&[0x61, 0x88]), 0x00);
        assert_eq!(frame_get_version(&[0x61, 0x98]), FRAME_VERSION_1);
        assert_eq!(frame_get_version(&[0x61, 0xA8]), FRAME_VERSION_2);
        // Octet 2 (sequence number) never contributes.
        assert_eq!(frame_get_version(&[0x61, 0x88, 0x30]), 0x00);
    }

    #[test]
    fn short_frames_need_no_ack() {
        assert!(!frame_is_ack_required(&[]));
        assert_eq!(frame_get_version(&[]), 0);
        assert_eq!(frame_get_version(&[0x63]), 0);
    }
}
