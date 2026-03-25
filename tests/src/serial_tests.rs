//! Tests for the serial MAC protocol codec (custom framing + ASH/EZSP + AES-CCM*).

#[cfg(test)]
mod tests {
    use zigbee_mac::serial::{
        crc16_ccitt, SerialCodec, SerialError, SerialFrame, CMD_DATA_REQ, CMD_RESET_REQ,
        CMD_SCAN_REQ, FRAME_START, MAX_FRAME_SIZE,
    };
    use zigbee_mac::serial::ezsp::{AshFrameType, EzspCodec, EzspFrame, ASH_FLAG, EZSP_VERSION};
    use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};

    // ── 1. CRC16-CCITT ──────────────────────────────────────────

    #[test]
    fn crc16_ccitt_known_vector_123456789() {
        // Standard test vector: ASCII "123456789" → 0x29B1
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn crc16_ccitt_empty_input() {
        // Empty data with init=0xFFFF should stay 0xFFFF
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn crc16_ccitt_single_byte() {
        // Single byte 0x00 — deterministic, just verify round-trip consistency
        let crc = crc16_ccitt(&[0x00]);
        assert_eq!(crc, crc16_ccitt(&[0x00]));
        assert_ne!(crc, 0xFFFF); // must differ from empty
    }

    #[test]
    fn crc16_ccitt_all_zeros() {
        let data = [0u8; 16];
        let crc = crc16_ccitt(&data);
        // Different length should produce a different CRC
        assert_ne!(crc, crc16_ccitt(&[0u8; 15]));
    }

    // ── 2. SerialFrame round-trip ───────────────────────────────

    #[test]
    fn serial_frame_roundtrip_empty_payload() {
        let frame = SerialFrame::new(CMD_RESET_REQ, 0x00, &[]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        let (parsed, consumed) = SerialFrame::parse(&buf[..len]).unwrap();
        assert_eq!(consumed, len);
        assert_eq!(parsed.cmd, CMD_RESET_REQ);
        assert_eq!(parsed.seq, 0x00);
        assert!(parsed.payload.is_empty());
    }

    #[test]
    fn serial_frame_roundtrip_with_payload() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF, 0x42];
        let frame = SerialFrame::new(CMD_DATA_REQ, 0x7F, &payload).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();

        assert_eq!(buf[0], FRAME_START);
        let (parsed, consumed) = SerialFrame::parse(&buf[..len]).unwrap();
        assert_eq!(consumed, len);
        assert_eq!(parsed.cmd, CMD_DATA_REQ);
        assert_eq!(parsed.seq, 0x7F);
        assert_eq!(parsed.payload.as_slice(), &payload);
    }

    #[test]
    fn serial_frame_roundtrip_large_payload() {
        let payload = [0xAB; 200];
        let frame = SerialFrame::new(CMD_SCAN_REQ, 0xFF, &payload).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        let (parsed, _) = SerialFrame::parse(&buf[..len]).unwrap();
        assert_eq!(parsed.payload.as_slice(), &payload);
    }

    #[test]
    fn serial_frame_crc_corruption_detected() {
        let frame = SerialFrame::new(CMD_RESET_REQ, 0x01, &[0x01]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        // Flip a bit in the CRC
        buf[len - 1] ^= 0x01;
        assert_eq!(
            SerialFrame::parse(&buf[..len]).unwrap_err(),
            SerialError::CrcError
        );
    }

    #[test]
    fn serial_frame_payload_corruption_detected() {
        let frame = SerialFrame::new(CMD_DATA_REQ, 0x01, &[0x01, 0x02, 0x03]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        // Corrupt a payload byte
        buf[5] ^= 0xFF;
        assert_eq!(
            SerialFrame::parse(&buf[..len]).unwrap_err(),
            SerialError::CrcError
        );
    }

    // ── 3. SerialCodec streaming ────────────────────────────────

    #[test]
    fn serial_codec_feed_byte_by_byte() {
        let payload = [0x01, 0x00, 0x00, 0x00, 0x00, 0x03];
        let frame = SerialFrame::new(CMD_SCAN_REQ, 0x02, &payload).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();

        let mut codec = SerialCodec::new();
        for i in 0..len - 1 {
            assert!(codec.feed(&buf[i..i + 1]).unwrap().is_none());
        }
        let result = codec.feed(&buf[len - 1..len]).unwrap();
        let parsed = result.expect("should yield a frame");
        assert_eq!(parsed.cmd, CMD_SCAN_REQ);
        assert_eq!(parsed.seq, 0x02);
        assert_eq!(parsed.payload.as_slice(), &payload);
    }

    #[test]
    fn serial_codec_feed_with_garbage_prefix() {
        let frame = SerialFrame::new(CMD_RESET_REQ, 0x05, &[0xFF]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();

        let mut codec = SerialCodec::new();
        // Feed garbage bytes first — codec should skip them
        assert!(codec.feed(&[0x00, 0xAA, 0xBB, 0xCC]).unwrap().is_none());
        // Now feed the real frame
        let result = codec.feed(&buf[..len]).unwrap();
        let parsed = result.expect("should yield a frame after garbage");
        assert_eq!(parsed.cmd, CMD_RESET_REQ);
        assert_eq!(parsed.seq, 0x05);
    }

    #[test]
    fn serial_codec_two_frames_back_to_back() {
        let f1 = SerialFrame::new(CMD_RESET_REQ, 0x01, &[0x00]).unwrap();
        let f2 = SerialFrame::new(CMD_SCAN_REQ, 0x02, &[0x01, 0x02]).unwrap();

        let mut combined = [0u8; MAX_FRAME_SIZE * 2];
        let len1 = f1.serialize(&mut combined).unwrap();
        let len2 = f2.serialize(&mut combined[len1..]).unwrap();

        let mut codec = SerialCodec::new();
        // Feed just the first frame
        let r1 = codec.feed(&combined[..len1]).unwrap();
        let p1 = r1.expect("first frame");
        assert_eq!(p1.cmd, CMD_RESET_REQ);

        // Feed the second frame
        let r2 = codec.feed(&combined[len1..len1 + len2]).unwrap();
        let p2 = r2.expect("second frame");
        assert_eq!(p2.cmd, CMD_SCAN_REQ);
        assert_eq!(p2.payload.as_slice(), &[0x01, 0x02]);
    }

    #[test]
    fn serial_codec_split_in_header() {
        let frame = SerialFrame::new(CMD_DATA_REQ, 0x10, &[0xAA, 0xBB]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();

        let mut codec = SerialCodec::new();
        // Split in the middle of the header (after START + CMD)
        assert!(codec.feed(&buf[..2]).unwrap().is_none());
        // Feed the rest
        let result = codec.feed(&buf[2..len]).unwrap();
        let parsed = result.expect("should reassemble after header split");
        assert_eq!(parsed.cmd, CMD_DATA_REQ);
        assert_eq!(parsed.payload.as_slice(), &[0xAA, 0xBB]);
    }

    // ── 4. AshFrame byte-stuffing round-trip ────────────────────

    #[test]
    fn ash_rst_frame_roundtrip() {
        let codec = EzspCodec::new();        let mut out = [0u8; 64];
        let len = codec.build_rst(&mut out[..]).unwrap();

        // Frame must be delimited by FLAG bytes
        assert_eq!(out[0], ASH_FLAG);
        assert_eq!(out[len - 1], ASH_FLAG);

        // Feed it back into a fresh codec
        let mut rx = EzspCodec::new();
        let result = rx.feed(&out[..len]).unwrap();
        let (frame_type, _data) = result.expect("should decode RST frame");
        assert!(matches!(frame_type, AshFrameType::Rst));
    }

    #[test]
    fn ash_ack_frame_roundtrip() {
        let codec = EzspCodec::new();
        let mut out = [0u8; 64];
        let len = codec.build_ack(&mut out[..]).unwrap();

        assert_eq!(out[0], ASH_FLAG);
        assert_eq!(out[len - 1], ASH_FLAG);

        let mut rx = EzspCodec::new();
        let result = rx.feed(&out[..len]).unwrap();
        let (frame_type, _) = result.expect("should decode ACK frame");
        assert!(matches!(frame_type, AshFrameType::Ack { .. }));
    }

    #[test]
    fn ash_data_frame_roundtrip() {
        let mut tx = EzspCodec::new();
        let ezsp = EzspFrame::command(0, EZSP_VERSION, &[8]).unwrap();

        let mut out = [0u8; 512];
        let len = tx.build_data(&ezsp, &mut out).unwrap();

        // Verify FLAG delimiters
        assert_eq!(out[0], ASH_FLAG);
        assert_eq!(out[len - 1], ASH_FLAG);

        // Feed into receiver and decode
        let mut rx = EzspCodec::new();
        let result = rx.feed(&out[..len]).unwrap();
        let (frame_type, ezsp_data) = result.expect("should decode DATA frame");
        assert!(matches!(frame_type, AshFrameType::Data { frm_num: 0, .. }));

        // The de-randomized EZSP data should parse back to the original frame
        let parsed_ezsp = EzspFrame::parse(&ezsp_data).unwrap();
        assert_eq!(parsed_ezsp.frame_id, EZSP_VERSION);
        assert_eq!(parsed_ezsp.payload.as_slice(), &[8]);
    }

    // ── 5. LFSR randomization/de-randomization round-trip ───────
    //    (tested via the EzspCodec DATA path which applies LFSR internally)

    #[test]
    fn lfsr_roundtrip_via_codec_various_payloads() {
        // Test several payloads to exercise different LFSR states
        let payloads: &[&[u8]] = &[
            &[],
            &[0x00],
            &[0xFF; 20],
            &[0x11, 0x13, 0x7D, 0x7E, 0x18, 0x1A], // all ASH special bytes
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
        ];

        for (i, payload) in payloads.iter().enumerate() {
            let mut tx = EzspCodec::new();
            let ezsp = EzspFrame::command(i as u8, EZSP_VERSION, payload).unwrap();

            let mut out = [0u8; 512];
            let len = tx.build_data(&ezsp, &mut out).unwrap();

            let mut rx = EzspCodec::new();
            let result = rx
                .feed(&out[..len])
                .unwrap()
                .expect("should decode DATA frame");

            let parsed = EzspFrame::parse(&result.1).unwrap();
            assert_eq!(
                parsed.payload.as_slice(),
                *payload,
                "payload mismatch at index {i}"
            );
        }
    }

    #[test]
    fn lfsr_data_is_actually_randomized() {
        // Verify that the wire bytes differ from the plaintext
        let mut tx = EzspCodec::new();
        let payload = [0x00; 10]; // all zeros — LFSR should change them
        let ezsp = EzspFrame::command(0, EZSP_VERSION, &payload).unwrap();

        let mut out = [0u8; 512];
        let len = tx.build_data(&ezsp, &mut out).unwrap();

        // The stuffed frame between the FLAGs should not contain long runs of 0x00
        // (because LFSR XORs with a non-zero PRNG stream starting at seed 0x42)
        let inner = &out[1..len - 1]; // strip FLAG delimiters
        let zero_count = inner.iter().filter(|&&b| b == 0x00).count();
        assert!(
            zero_count < inner.len() / 2,
            "too many zero bytes after LFSR — randomization may be broken"
        );
    }

    // ── 6. AES-CCM* encrypt → decrypt round-trip ────────────────

    #[test]
    fn aes_ccm_star_encrypt_decrypt_roundtrip() {
        let mut sec = NwkSecurity::new();
        let key = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        sec.set_network_key(key, 0);

        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 1,
            source_address: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            key_seq_number: 0,
        };

        let nwk_header = [0x08, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x1E, 0x00];
        let plaintext = b"Hello Zigbee NWK";

        let ciphertext = sec
            .encrypt(&nwk_header, plaintext, &key, &sec_hdr)
            .expect("encryption should succeed");

        // Ciphertext should be plaintext.len() + 4 (MIC) bytes
        assert_eq!(ciphertext.len(), plaintext.len() + 4);
        // Ciphertext must differ from plaintext
        assert_ne!(&ciphertext[..plaintext.len()], &plaintext[..]);

        let decrypted = sec
            .decrypt(&nwk_header, &ciphertext, &key, &sec_hdr)
            .expect("decryption should succeed");

        assert_eq!(decrypted.as_slice(), plaintext);
    }

    #[test]
    fn aes_ccm_star_tampered_ciphertext_fails() {
        let mut sec = NwkSecurity::new();
        let key = [0xAA; 16];
        sec.set_network_key(key, 1);

        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 42,
            source_address: [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80],
            key_seq_number: 1,
        };

        let nwk_header = [0x08, 0x40];
        let plaintext = b"tamper test data";

        let mut ciphertext = sec
            .encrypt(&nwk_header, plaintext, &key, &sec_hdr)
            .expect("encryption should succeed");

        // Tamper with a ciphertext byte
        ciphertext[0] ^= 0x01;

        let result = sec.decrypt(&nwk_header, &ciphertext, &key, &sec_hdr);
        assert!(result.is_none(), "tampered ciphertext must fail decryption");
    }

    #[test]
    fn aes_ccm_star_wrong_key_fails() {
        let mut sec = NwkSecurity::new();
        let key = [0x11; 16];
        sec.set_network_key(key, 0);

        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 100,
            source_address: [0xFF; 8],
            key_seq_number: 0,
        };

        let nwk_header = [0x08, 0x00];
        let plaintext = b"key mismatch";

        let ciphertext = sec
            .encrypt(&nwk_header, plaintext, &key, &sec_hdr)
            .expect("encryption should succeed");

        // Decrypt with a different key
        let wrong_key = [0x22; 16];
        let result = sec.decrypt(&nwk_header, &ciphertext, &wrong_key, &sec_hdr);
        assert!(result.is_none(), "wrong key must fail decryption");
    }

    #[test]
    fn aes_ccm_star_empty_plaintext() {
        let mut sec = NwkSecurity::new();
        let key = [0x55; 16];
        sec.set_network_key(key, 0);

        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 0,
            source_address: [0; 8],
            key_seq_number: 0,
        };

        let nwk_header = [0x08, 0x00];
        let plaintext: &[u8] = &[];

        let ciphertext = sec
            .encrypt(&nwk_header, plaintext, &key, &sec_hdr)
            .expect("encrypting empty plaintext should succeed");

        // Should be just the 4-byte MIC
        assert_eq!(ciphertext.len(), 4);

        let decrypted = sec
            .decrypt(&nwk_header, &ciphertext, &key, &sec_hdr)
            .expect("decrypting should succeed");

        assert!(decrypted.is_empty());
    }
}
