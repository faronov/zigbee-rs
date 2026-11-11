//! Envelope verification against imgtool vectors signed with
//! `--header-size 0x20 --pad-header` (see `tests/vectors/generate.sh`).

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};

use crate::envelope::{ENVELOPE_HEADER_LEN, validate_envelope};
use crate::image::{ImageError, ImageHeader, ImageVersion, KeySet, TrustedKey};

const TEST_KEY: [u8; 64] = *include_bytes!("../tests/vectors/test-key.xy");
const KEY_B: [u8; 64] = *include_bytes!("../tests/vectors/test-key-b.xy");
const ENV_V2: &[u8] = include_bytes!("../tests/vectors/test-env-v2.bin");
const ENV_V1_PROTECTED: &[u8] = include_bytes!("../tests/vectors/test-env-v1-protected.bin");
/// A regular 0x100-header image: not an envelope.
const V2: &[u8] = include_bytes!("../tests/vectors/test-v2.bin");

fn keys(key: [u8; 64]) -> KeySet {
    KeySet::new(Box::leak(Box::new([TrustedKey::new(key)])))
}

/// Split an envelope into (header, native image, trailer).
fn split(envelope: &[u8]) -> ([u8; ENVELOPE_HEADER_LEN], Vec<u8>, Vec<u8>) {
    let header: [u8; ENVELOPE_HEADER_LEN] = envelope[..ENVELOPE_HEADER_LEN].try_into().unwrap();
    let size = ImageHeader::parse(&header).unwrap().img_size as usize;
    let payload_end = ENVELOPE_HEADER_LEN + size;
    (
        header,
        envelope[ENVELOPE_HEADER_LEN..payload_end].to_vec(),
        envelope[payload_end..].to_vec(),
    )
}

/// Byte-readable storage holding the native image at `at`.
struct Bytes {
    data: Vec<u8>,
}

#[derive(Debug)]
struct OutOfRange;

impl NorFlashError for OutOfRange {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::OutOfBounds
    }
}

impl ErrorType for Bytes {
    type Error = OutOfRange;
}

impl ReadNorFlash for Bytes {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        let source = self
            .data
            .get(start..start + bytes.len())
            .ok_or(OutOfRange)?;
        bytes.copy_from_slice(source);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.data.len()
    }
}

fn check(envelope: &[u8], key: [u8; 64]) -> Result<ImageVersion, ImageError> {
    let (header, payload, trailer) = split(envelope);
    let mut flash = Bytes {
        data: [vec![0xA5; 0x40], payload].concat(),
    };
    validate_envelope(&header, &trailer, &mut flash, 0x40, &keys(key)).map(|v| v.header.version)
}

#[test]
fn envelopes_verify() {
    assert_eq!(check(ENV_V2, TEST_KEY), Ok(ImageVersion::new(2, 0, 0, 0)));
    // Protected TLVs (security counter) sit at the start of the trailer and
    // are hashed.
    assert_eq!(
        check(ENV_V1_PROTECTED, TEST_KEY),
        Ok(ImageVersion::new(1, 2, 3, 4))
    );
}

#[test]
fn envelopes_from_untrusted_keys_are_refused() {
    assert_eq!(check(ENV_V2, KEY_B), Err(ImageError::BadSignature));
    let (header, payload, trailer) = split(ENV_V2);
    let mut flash = Bytes { data: payload };
    assert_eq!(
        validate_envelope(&header, &trailer, &mut flash, 0, &KeySet::new(&[])),
        Err(ImageError::BadSignature)
    );
}

#[test]
fn tampering_is_detected() {
    for envelope in [ENV_V2, ENV_V1_PROTECTED] {
        let (header, payload, trailer) = split(envelope);
        let protect = ImageHeader::parse(&header).unwrap().protect_tlv_size as usize;
        // A native-image byte.
        let mut bad = payload.clone();
        bad[1234] ^= 1;
        let mut flash = Bytes { data: bad };
        assert_eq!(
            validate_envelope(&header, &trailer, &mut flash, 0, &keys(TEST_KEY)),
            Err(ImageError::HashMismatch)
        );
        // The header (version), which the signature covers.
        let mut bad_header = header;
        bad_header[20] ^= 1;
        let mut flash = Bytes {
            data: payload.clone(),
        };
        assert_eq!(
            validate_envelope(&bad_header, &trailer, &mut flash, 0, &keys(TEST_KEY)),
            Err(ImageError::HashMismatch)
        );
        // The last signature byte.
        let mut bad_trailer = trailer.clone();
        *bad_trailer.last_mut().unwrap() ^= 1;
        let mut flash = Bytes {
            data: payload.clone(),
        };
        assert!(validate_envelope(&header, &bad_trailer, &mut flash, 0, &keys(TEST_KEY)).is_err());
        if protect != 0 {
            // A protected TLV byte (the security counter value).
            let mut bad_protected = trailer.clone();
            bad_protected[protect - 1] ^= 1;
            assert_eq!(
                validate_envelope(&header, &bad_protected, &mut flash, 0, &keys(TEST_KEY)),
                Err(ImageError::HashMismatch)
            );
        }
    }
}

#[test]
fn malformed_envelopes_are_refused() {
    let (header, payload, trailer) = split(ENV_V2);
    // Trailing bytes after the TLV area.
    let mut long = trailer.clone();
    long.push(0xFF);
    let mut flash = Bytes {
        data: payload.clone(),
    };
    assert_eq!(
        validate_envelope(&header, &long, &mut flash, 0, &keys(TEST_KEY)),
        Err(ImageError::Malformed)
    );
    // A truncated TLV area.
    assert_eq!(
        validate_envelope(
            &header,
            &trailer[..trailer.len() - 1],
            &mut flash,
            0,
            &keys(TEST_KEY)
        ),
        Err(ImageError::Malformed)
    );
    // A staged image shorter than the header claims.
    let mut short = Bytes {
        data: payload[..payload.len() - 1].to_vec(),
    };
    assert_eq!(
        validate_envelope(&header, &trailer, &mut short, 0, &keys(TEST_KEY)),
        Err(ImageError::Flash)
    );
    // A regular image with a padded header is not an envelope.
    let regular: [u8; ENVELOPE_HEADER_LEN] = V2[..ENVELOPE_HEADER_LEN].try_into().unwrap();
    assert_eq!(
        validate_envelope(&regular, &trailer, &mut flash, 0, &keys(TEST_KEY)),
        Err(ImageError::Malformed)
    );
    // No image magic.
    assert_eq!(
        validate_envelope(&[0xFF; 32], &trailer, &mut flash, 0, &keys(TEST_KEY)),
        Err(ImageError::NoImage)
    );
}

#[cfg(feature = "firmware-writer")]
mod writer {
    use super::*;
    use crate::envelope::{EnvelopePolicy, EnvelopeWriter, StagedImage};
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

    /// A platform writer: sequential RAM staging with read-back.
    #[derive(Default)]
    struct Native {
        slot: Vec<u8>,
        capacity: u32,
        staging: bool,
        verified_len: Option<u32>,
        activated: bool,
        confirmed: bool,
        aborts: u32,
    }

    impl FirmwareWriter for Native {
        fn erase_slot(&mut self) -> Result<(), FirmwareError> {
            self.slot.clear();
            self.staging = true;
            self.verified_len = None;
            Ok(())
        }

        fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
            if !self.staging || offset as usize != self.slot.len() {
                return Err(FirmwareError::WriteFailed);
            }
            if self.slot.len() + data.len() > self.capacity as usize {
                return Err(FirmwareError::OutOfRange);
            }
            self.slot.extend_from_slice(data);
            Ok(())
        }

        fn verify(&mut self, size: u32, _hash: Option<&[u8]>) -> Result<(), FirmwareError> {
            if size as usize != self.slot.len() {
                return Err(FirmwareError::VerifyFailed);
            }
            self.verified_len = Some(size);
            Ok(())
        }

        fn activate(&mut self) -> Result<(), FirmwareError> {
            if self.verified_len.is_none() {
                return Err(FirmwareError::ActivateFailed);
            }
            self.activated = true;
            Ok(())
        }

        fn slot_size(&self) -> u32 {
            self.capacity
        }

        fn abort(&mut self) -> Result<(), FirmwareError> {
            self.staging = false;
            self.slot.clear();
            self.verified_len = None;
            self.aborts += 1;
            Ok(())
        }

        fn confirm_image(&mut self) -> Result<(), FirmwareError> {
            self.confirmed = true;
            Ok(())
        }
    }

    impl StagedImage for Native {
        fn read_staged(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FirmwareError> {
            let start = offset as usize;
            let source = self
                .slot
                .get(start..start + bytes.len())
                .ok_or(FirmwareError::OutOfRange)?;
            bytes.copy_from_slice(source);
            Ok(())
        }
    }

    fn writer(running: ImageVersion) -> EnvelopeWriter<Native> {
        EnvelopeWriter::new(
            Native {
                capacity: 0x4000,
                ..Default::default()
            },
            EnvelopePolicy {
                keys: keys(TEST_KEY),
                running,
            },
        )
    }

    fn download<W: FirmwareWriter>(
        writer: &mut W,
        envelope: &[u8],
        block: usize,
    ) -> Result<(), FirmwareError> {
        writer.erase_slot()?;
        let mut offset = 0;
        for chunk in envelope.chunks(block) {
            writer.write_block(offset, chunk)?;
            offset += chunk.len() as u32;
        }
        Ok(())
    }

    #[test]
    fn verified_envelope_stages_the_native_image() {
        for (envelope, block) in [
            (ENV_V2, 48),
            (ENV_V2, 1),
            (ENV_V1_PROTECTED, 7),
            (ENV_V2, 4096),
        ] {
            let mut writer = writer(ImageVersion::new(1, 0, 0, 0));
            download(&mut writer, envelope, block).unwrap();
            assert_eq!(
                writer.activate(),
                Err(FirmwareError::ActivateFailed),
                "no activation before verification"
            );
            writer.verify(envelope.len() as u32, None).unwrap();
            let (_, payload, _) = split(envelope);
            assert_eq!(
                writer.inner().slot,
                payload,
                "only the native image is staged"
            );
            assert_eq!(writer.inner().verified_len, Some(payload.len() as u32));
            assert!(writer.verified_header().is_some());
            writer.activate().unwrap();
            assert!(writer.inner().activated);
        }
    }

    /// Every refusal in `verify` aborts the native writer, so the platform
    /// can invalidate the staged image instead of leaving it bootable.
    #[test]
    fn refused_envelopes_are_discarded_from_the_native_slot() {
        let mut tampered = ENV_V2.to_vec();
        tampered[ENVELOPE_HEADER_LEN + 100] ^= 1;
        /// Envelope, running version, trusted key, expected hash.
        type Case<'a> = (&'a [u8], ImageVersion, [u8; 64], Option<&'a [u8; 32]>);
        let cases: [Case; 4] = [
            // Downgrade (version below the running image).
            (ENV_V2, ImageVersion::new(2, 0, 1, 0), TEST_KEY, None),
            // Untrusted signer.
            (ENV_V2, ImageVersion::default(), KEY_B, None),
            // Tampered native payload.
            (&tampered, ImageVersion::default(), TEST_KEY, None),
            // Hash that is not the envelope's.
            (ENV_V2, ImageVersion::default(), TEST_KEY, Some(&[0; 32])),
        ];
        for (envelope, running, key, hash) in cases {
            let mut writer = EnvelopeWriter::<_, 512>::new(
                Native {
                    capacity: 0x4000,
                    ..Default::default()
                },
                EnvelopePolicy {
                    keys: keys(key),
                    running,
                },
            );
            download(&mut writer, envelope, 64).unwrap();
            assert_eq!(
                writer.verify(envelope.len() as u32, hash.map(|h| &h[..])),
                Err(FirmwareError::VerifyFailed)
            );
            assert_eq!(writer.inner().aborts, 1, "native writer discarded");
            assert!(writer.inner().slot.is_empty());
            assert!(writer.verified_header().is_none());
            assert_eq!(writer.activate(), Err(FirmwareError::ActivateFailed));
            // A retried verify cannot resurrect the discarded image.
            assert_eq!(
                writer.verify(envelope.len() as u32, None),
                Err(FirmwareError::VerifyFailed)
            );
        }
        // Size mismatch is refused before the native writer verifies, and is
        // discarded the same way.
        let mut short = writer(ImageVersion::default());
        download(&mut short, ENV_V2, 64).unwrap();
        assert_eq!(
            short.verify(ENV_V2.len() as u32 + 1, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(short.inner().aborts, 1);
        // An accepted envelope is not aborted.
        let mut good = writer(ImageVersion::default());
        download(&mut good, ENV_V2, 64).unwrap();
        good.verify(ENV_V2.len() as u32, None).unwrap();
        assert_eq!(good.inner().aborts, 0);
    }

    #[test]
    fn downgrades_are_refused_and_equal_versions_accepted() {
        let mut older = writer(ImageVersion::new(2, 0, 1, 0));
        download(&mut older, ENV_V2, 64).unwrap();
        assert_eq!(
            older.verify(ENV_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(older.activate(), Err(FirmwareError::ActivateFailed));
        // The build number is ignored, as in MCUboot.
        let mut same = writer(ImageVersion::new(2, 0, 0, 99));
        download(&mut same, ENV_V2, 64).unwrap();
        same.verify(ENV_V2.len() as u32, None).unwrap();
    }

    #[test]
    fn bad_envelopes_never_activate() {
        // Tampered native image.
        let mut bad = ENV_V2.to_vec();
        bad[ENVELOPE_HEADER_LEN + 100] ^= 1;
        let mut tampered = writer(ImageVersion::default());
        download(&mut tampered, &bad, 64).unwrap();
        assert_eq!(
            tampered.verify(bad.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(tampered.activate(), Err(FirmwareError::ActivateFailed));

        // Size disagreeing with the download.
        let mut short = writer(ImageVersion::default());
        download(&mut short, ENV_V2, 64).unwrap();
        assert_eq!(
            short.verify(ENV_V2.len() as u32 + 1, None),
            Err(FirmwareError::VerifyFailed)
        );

        // A hash that is not the envelope's.
        let mut other_hash = writer(ImageVersion::default());
        download(&mut other_hash, ENV_V2, 64).unwrap();
        assert_eq!(
            other_hash.verify(ENV_V2.len() as u32, Some(&[0; 32])),
            Err(FirmwareError::VerifyFailed)
        );

        // An untrusted signer.
        let mut foreign = EnvelopeWriter::<_, 512>::new(
            Native {
                capacity: 0x4000,
                ..Default::default()
            },
            EnvelopePolicy {
                keys: keys(KEY_B),
                running: ImageVersion::default(),
            },
        );
        download(&mut foreign, ENV_V2, 64).unwrap();
        assert_eq!(
            foreign.verify(ENV_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );

        // A regular (0x100-header) image is refused at its header, before any
        // native byte is staged.
        let mut regular = writer(ImageVersion::default());
        assert_eq!(
            download(&mut regular, V2, 64),
            Err(FirmwareError::VerifyFailed)
        );
        assert!(regular.inner().slot.is_empty());
    }

    #[test]
    fn oversized_parts_are_refused() {
        // Native image larger than the platform slot.
        let mut small = EnvelopeWriter::<_, 512>::new(
            Native {
                capacity: 1024,
                ..Default::default()
            },
            EnvelopePolicy {
                keys: keys(TEST_KEY),
                running: ImageVersion::default(),
            },
        );
        assert_eq!(small.slot_size(), 1024 + 32 + 512);
        assert_eq!(
            download(&mut small, ENV_V2, 64),
            Err(FirmwareError::ImageTooLarge)
        );
        assert!(small.inner().slot.is_empty());
        // TLVs larger than the RAM trailer.
        let mut tight = EnvelopeWriter::<_, 64>::new(
            Native {
                capacity: 0x4000,
                ..Default::default()
            },
            EnvelopePolicy {
                keys: keys(TEST_KEY),
                running: ImageVersion::default(),
            },
        );
        assert_eq!(
            download(&mut tight, ENV_V2, 64),
            Err(FirmwareError::ImageTooLarge)
        );
    }

    #[test]
    fn restart_abort_and_confirm_reach_the_platform_writer() {
        let mut writer = writer(ImageVersion::default());
        download(&mut writer, &ENV_V2[..500], 64).unwrap();
        // Out-of-order block.
        assert_eq!(
            writer.write_block(0, &ENV_V2[..10]),
            Err(FirmwareError::WriteFailed)
        );
        writer.abort().unwrap();
        assert!(!writer.inner().staging);
        // A fresh download after the abort verifies.
        download(&mut writer, ENV_V2, 100).unwrap();
        writer.verify(ENV_V2.len() as u32, None).unwrap();
        // No writes after verification.
        assert_eq!(
            writer.write_block(ENV_V2.len() as u32, &[0]),
            Err(FirmwareError::WriteFailed)
        );
        writer.confirm_image().unwrap();
        assert!(writer.inner().confirmed);
    }
}
