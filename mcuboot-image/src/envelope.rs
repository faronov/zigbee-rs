//! App-side MCUboot verification for platforms whose ROM or SDK boot chain
//! stays in place (ESP-IDF second-stage bootloader, Telink boot ROM).
//!
//! Those boot chains require the native image at the start of the
//! application slot, so the MCUboot header cannot stay in front of it. An
//! *envelope* is an unmodified `imgtool sign --header-size 0x20 --pad-header`
//! output around the native image:
//!
//! ```text
//! OTA payload  = [32-byte MCUboot header][native image][protected TLVs][TLVs]
//! staged slot  =                         [native image]
//! kept in RAM  = [32-byte MCUboot header]               [all TLVs]
//! ```
//!
//! The signature covers header, native image and protected TLVs exactly as
//! MCUboot defines it, so the standard tools sign and inspect envelopes.
//! [`validate_envelope`] reassembles that view from the RAM parts and the
//! staged native image; [`EnvelopeWriter`] (feature `firmware-writer`)
//! splits a downloaded envelope in front of a platform `FirmwareWriter` and
//! refuses activation unless the envelope verifies with a trusted key and is
//! not older than the running image.
//!
//! This authenticates what the *application* accepts over the air. It does
//! not make the ROM/SDK boot chain verify anything: an image written by
//! other means (UART, SWD, the vendor's own OTA) still boots.

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};

use crate::image::{IMAGE_HEADER_LEN, ImageError, ImageHeader, KeySet, ValidImage, validate_by};

/// Bytes of MCUboot header in front of the native image.
pub const ENVELOPE_HEADER_LEN: usize = IMAGE_HEADER_LEN;

/// Parse an envelope header: a bootable MCUboot header of exactly
/// [`ENVELOPE_HEADER_LEN`] bytes (no padding in front of the native image).
pub fn parse_envelope_header(bytes: &[u8; ENVELOPE_HEADER_LEN]) -> Result<ImageHeader, ImageError> {
    let header = ImageHeader::parse(bytes).ok_or(ImageError::NoImage)?;
    if header.flags != 0 {
        return Err(ImageError::Unsupported);
    }
    if header.hdr_size as usize != ENVELOPE_HEADER_LEN {
        return Err(ImageError::Malformed);
    }
    Ok(header)
}

/// Validate an envelope: `header` and `trailer` (everything after the native
/// image: protected TLVs, then the TLV area) from RAM, and the
/// `header.img_size`-byte native image at `payload_at` in `payload`.
///
/// The trailer must hold exactly the TLV areas; trailing bytes are refused.
/// `payload` must accept reads at any offset and length (`READ_SIZE == 1`).
pub fn validate_envelope<F: ReadNorFlash>(
    header: &[u8; ENVELOPE_HEADER_LEN],
    trailer: &[u8],
    payload: &mut F,
    payload_at: u32,
    keys: &KeySet,
) -> Result<ValidImage, ImageError> {
    const {
        assert!(
            F::READ_SIZE == 1,
            "envelope payloads need byte-granular reads"
        )
    };
    let parsed = parse_envelope_header(header)?;
    let total = (ENVELOPE_HEADER_LEN as u32)
        .checked_add(parsed.img_size)
        .and_then(|end| end.checked_add(u32::try_from(trailer.len()).ok()?))
        .ok_or(ImageError::Malformed)?;
    let mut view = View {
        header,
        payload,
        payload_at,
        img_size: parsed.img_size,
        trailer,
    };
    let image = validate_by(&mut view, 0, total, keys.iter())?;
    if image.end != total {
        return Err(ImageError::Malformed);
    }
    Ok(image)
}

/// The envelope as one contiguous image for [`validate_by`].
struct View<'a, F> {
    header: &'a [u8; ENVELOPE_HEADER_LEN],
    payload: &'a mut F,
    payload_at: u32,
    img_size: u32,
    trailer: &'a [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ViewError;

impl NorFlashError for ViewError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

impl<F> ErrorType for View<'_, F> {
    type Error = ViewError;
}

impl<F: ReadNorFlash> ReadNorFlash for View<'_, F> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let header_end = ENVELOPE_HEADER_LEN as u32;
        let payload_end = header_end + self.img_size;
        let mut at = offset;
        let mut rest = bytes;
        while !rest.is_empty() {
            let (source_end, take) = if at < header_end {
                (header_end, (header_end - at) as usize)
            } else if at < payload_end {
                (payload_end, (payload_end - at) as usize)
            } else {
                let start = (at - payload_end) as usize;
                let available = self.trailer.len().checked_sub(start).ok_or(ViewError)?;
                (u32::MAX, available)
            };
            let take = take.min(rest.len());
            if take == 0 {
                return Err(ViewError);
            }
            let (now, later) = rest.split_at_mut(take);
            if at < header_end {
                now.copy_from_slice(&self.header[at as usize..at as usize + take]);
            } else if source_end == payload_end {
                let address = self
                    .payload_at
                    .checked_add(at - header_end)
                    .ok_or(ViewError)?;
                self.payload.read(address, now).map_err(|_| ViewError)?;
            } else {
                let start = (at - payload_end) as usize;
                now.copy_from_slice(&self.trailer[start..start + take]);
            }
            at += take as u32;
            rest = later;
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        ENVELOPE_HEADER_LEN + self.img_size as usize + self.trailer.len()
    }
}

#[cfg(feature = "firmware-writer")]
pub use writer::{EnvelopePolicy, EnvelopeWriter, StagedImage};

#[cfg(feature = "firmware-writer")]
mod writer {
    use embedded_storage::nor_flash::{ErrorType, ReadNorFlash};
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

    use super::{ENVELOPE_HEADER_LEN, ViewError, parse_envelope_header, validate_envelope};
    use crate::image::{ImageHeader, ImageVersion, KeySet};

    /// A platform writer whose staged native image can be read back.
    pub trait StagedImage: FirmwareWriter {
        /// Read staged bytes at native-image offset `offset`. Any offset and
        /// length inside the staged image must be accepted; reads past it
        /// fail.
        fn read_staged(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FirmwareError>;
    }

    /// What an envelope must satisfy before activation.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct EnvelopePolicy {
        /// Keys whose signature is accepted. Empty refuses every image.
        pub keys: KeySet,
        /// The running image's version; older envelopes are refused
        /// (major.minor.revision, as MCUboot's downgrade prevention).
        pub running: ImageVersion,
    }

    /// Splits a downloaded envelope: the native image goes to `inner`, the
    /// header and up to `TRAILER` bytes of TLVs stay in RAM. Activation is
    /// refused until [`verify`](FirmwareWriter::verify) has checked the
    /// native image with `inner` and the envelope signature and version.
    pub struct EnvelopeWriter<W, const TRAILER: usize = 512> {
        inner: W,
        policy: EnvelopePolicy,
        header: [u8; ENVELOPE_HEADER_LEN],
        parsed: Option<ImageHeader>,
        trailer: [u8; TRAILER],
        trailer_len: usize,
        /// Next expected envelope offset.
        written: u32,
        verified: bool,
    }

    impl<W: StagedImage, const TRAILER: usize> EnvelopeWriter<W, TRAILER> {
        pub const fn new(inner: W, policy: EnvelopePolicy) -> Self {
            Self {
                inner,
                policy,
                header: [0; ENVELOPE_HEADER_LEN],
                parsed: None,
                trailer: [0; TRAILER],
                trailer_len: 0,
                written: 0,
                verified: false,
            }
        }

        pub fn inner(&self) -> &W {
            &self.inner
        }

        pub fn inner_mut(&mut self) -> &mut W {
            &mut self.inner
        }

        /// The verified envelope's header (after a successful `verify`).
        pub fn verified_header(&self) -> Option<&ImageHeader> {
            self.parsed.as_ref().filter(|_| self.verified)
        }

        fn reset(&mut self) {
            self.parsed = None;
            self.trailer_len = 0;
            self.written = 0;
            self.verified = false;
        }
    }

    impl<W: StagedImage, const TRAILER: usize> FirmwareWriter for EnvelopeWriter<W, TRAILER> {
        fn erase_slot(&mut self) -> Result<(), FirmwareError> {
            self.reset();
            self.inner.erase_slot()
        }

        fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
            if offset != self.written || self.verified {
                return Err(FirmwareError::WriteFailed);
            }
            let length = u32::try_from(data.len()).map_err(|_| FirmwareError::OutOfRange)?;
            let end = offset
                .checked_add(length)
                .ok_or(FirmwareError::OutOfRange)?;
            let mut at = offset;
            let mut rest = data;
            while !rest.is_empty() {
                let Some(header) = self.parsed else {
                    let start = at as usize;
                    let take = (ENVELOPE_HEADER_LEN - start).min(rest.len());
                    self.header[start..start + take].copy_from_slice(&rest[..take]);
                    rest = &rest[take..];
                    at += take as u32;
                    if at as usize == ENVELOPE_HEADER_LEN {
                        let header = parse_envelope_header(&self.header)
                            .map_err(|_| FirmwareError::VerifyFailed)?;
                        if header.img_size > self.inner.slot_size() {
                            return Err(FirmwareError::ImageTooLarge);
                        }
                        self.parsed = Some(header);
                    }
                    continue;
                };
                let payload_end = ENVELOPE_HEADER_LEN as u32 + header.img_size;
                if at < payload_end {
                    let take = ((payload_end - at) as usize).min(rest.len());
                    self.inner
                        .write_block(at - ENVELOPE_HEADER_LEN as u32, &rest[..take])?;
                    rest = &rest[take..];
                    at += take as u32;
                } else {
                    let space = TRAILER - self.trailer_len;
                    if rest.len() > space {
                        return Err(FirmwareError::ImageTooLarge);
                    }
                    self.trailer[self.trailer_len..self.trailer_len + rest.len()]
                        .copy_from_slice(rest);
                    self.trailer_len += rest.len();
                    at += rest.len() as u32;
                    rest = &[];
                }
            }
            self.written = end;
            Ok(())
        }

        /// `expected_size` is the whole envelope; `expected_hash`, when
        /// given, must be the MCUboot image hash (the SHA-256 TLV).
        ///
        /// A refused envelope is discarded: the native writer is aborted, so
        /// it drops (and, where the platform does so, invalidates) the staged
        /// image instead of keeping a bootable copy of an image that failed
        /// the signature or version check.
        fn verify(
            &mut self,
            expected_size: u32,
            expected_hash: Option<&[u8]>,
        ) -> Result<(), FirmwareError> {
            let checked = self.check(expected_size, expected_hash);
            if checked.is_err() {
                self.reset();
                let _ = self.inner.abort();
            }
            checked
        }

        fn activate(&mut self) -> Result<(), FirmwareError> {
            if !self.verified {
                return Err(FirmwareError::ActivateFailed);
            }
            self.inner.activate()
        }

        /// The largest envelope: a full native slot plus header and trailer.
        fn slot_size(&self) -> u32 {
            self.inner
                .slot_size()
                .saturating_add((ENVELOPE_HEADER_LEN + TRAILER) as u32)
        }

        fn abort(&mut self) -> Result<(), FirmwareError> {
            self.reset();
            self.inner.abort()
        }

        fn confirm_image(&mut self) -> Result<(), FirmwareError> {
            self.inner.confirm_image()
        }
    }

    impl<W: StagedImage, const TRAILER: usize> EnvelopeWriter<W, TRAILER> {
        fn check(
            &mut self,
            expected_size: u32,
            expected_hash: Option<&[u8]>,
        ) -> Result<(), FirmwareError> {
            self.verified = false;
            let header = self.parsed.ok_or(FirmwareError::VerifyFailed)?;
            if self.written != expected_size {
                return Err(FirmwareError::VerifyFailed);
            }
            // The native writer checks its own format (size, structure,
            // built-in digests); `expected_hash` names the envelope.
            self.inner.verify(header.img_size, None)?;
            let mut staged = Staged(&mut self.inner);
            let image = validate_envelope(
                &self.header,
                &self.trailer[..self.trailer_len],
                &mut staged,
                0,
                &self.policy.keys,
            )
            .map_err(|_| FirmwareError::VerifyFailed)?;
            if expected_hash.is_some_and(|hash| hash != image.digest) {
                return Err(FirmwareError::VerifyFailed);
            }
            if image
                .header
                .version
                .cmp_release(&self.policy.running)
                .is_lt()
            {
                return Err(FirmwareError::VerifyFailed);
            }
            self.verified = true;
            Ok(())
        }
    }

    /// The staged native image as byte-readable flash.
    struct Staged<'a, W>(&'a mut W);

    impl<W> ErrorType for Staged<'_, W> {
        type Error = ViewError;
    }

    impl<W: StagedImage> ReadNorFlash for Staged<'_, W> {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.0.read_staged(offset, bytes).map_err(|_| ViewError)
        }

        fn capacity(&self) -> usize {
            self.0.slot_size() as usize
        }
    }
}
