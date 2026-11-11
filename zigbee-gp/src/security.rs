//! Green Power security processing (GP spec A.1.5.3 – A.1.5.4, A.3.7).
//!
//! Only security levels 0b10 (full counter + MIC) and 0b11 (encryption +
//! full counter + MIC) are implemented; level 0b01 is reserved since GP 1.1.
//! All functions are generic over a [`ForwardAesProvider`] so platforms with
//! an AES accelerator can use it.

use zigbee_crypto::{AesKey, ForwardAesProvider, ccm_star_decrypt_with, ccm_star_encrypt_with};

use crate::frame::{Gpdf, MAX_GPD_PAYLOAD, MIC_LEN};
use crate::types::{GpdId, SecurityLevel};
use crate::{Error, Result};

/// The default Zigbee trust-centre link key "ZigBeeAlliance09", used by GP
/// for over-the-air protection of GPD keys during commissioning.
pub const DEFAULT_TC_LINK_KEY: AesKey = *b"ZigBeeAlliance09";

/// CCM* nonce security-control byte for GP (A.1.5.4.1).
pub const SECURITY_CONTROL: u8 = 0x05;
/// Security-control byte for GPDFs sent to an ApplicationID 0b010 GPD.
pub const SECURITY_CONTROL_TO_IEEE_GPD: u8 = 0xC5;

/// Frame direction relative to the GPD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Frame sent by the GPD.
    FromGpd,
    /// Frame sent to the GPD.
    ToGpd,
}

/// Build the 13-byte GP CCM* nonce.
pub fn nonce(gpd: &GpdId, frame_counter: u32, direction: Direction) -> [u8; 13] {
    let mut n = [0u8; 13];
    match *gpd {
        GpdId::SrcId(id) => {
            let b = id.to_le_bytes();
            if direction == Direction::FromGpd {
                n[0..4].copy_from_slice(&b);
            }
            n[4..8].copy_from_slice(&b);
            n[12] = SECURITY_CONTROL;
        }
        GpdId::Ieee { ieee, .. } => {
            n[0..8].copy_from_slice(&ieee);
            n[12] = if direction == Direction::ToGpd {
                SECURITY_CONTROL_TO_IEEE_GPD
            } else {
                SECURITY_CONTROL
            };
        }
    }
    n[8..12].copy_from_slice(&frame_counter.to_le_bytes());
    n
}

fn ccm_seal<P: ForwardAesProvider>(
    provider: &mut P,
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    plaintext: &[u8],
) -> Result<heapless::Vec<u8, { zigbee_crypto::CCM_STAR_BUFFER_CAPACITY }>> {
    let mut cipher = provider.forward_cipher(key);
    ccm_star_encrypt_with(&mut cipher, nonce, aad, plaintext)
        .map_err(|_| Error::CipherFailure)?
        .ok_or(Error::BufferFull)
}

fn ccm_open<P: ForwardAesProvider>(
    provider: &mut P,
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Result<heapless::Vec<u8, { zigbee_crypto::CCM_STAR_BUFFER_CAPACITY }>> {
    let mut cipher = provider.forward_cipher(key);
    ccm_star_decrypt_with(&mut cipher, nonce, aad, ciphertext_and_mic)
        .map_err(|_| Error::CipherFailure)?
        .ok_or(Error::AuthFailed)
}

/// Clear GPD CommandID + payload buffer.
pub type GpdPayload = heapless::Vec<u8, MAX_GPD_PAYLOAD>;

/// Verify (and for level 0b11 decrypt) a received protected GPDF.
///
/// Returns the clear GPD CommandID + payload. Unprotected frames are
/// returned unchanged. `direction` is the direction the frame was sent in
/// (a proxy receiving from a GPD uses [`Direction::FromGpd`]).
pub fn open<P: ForwardAesProvider>(
    provider: &mut P,
    key: &AesKey,
    frame: &Gpdf<'_>,
    direction: Direction,
) -> Result<GpdPayload> {
    let mut out = GpdPayload::new();
    let level = frame.header.security_level;
    if !level.is_protected() {
        out.extend_from_slice(frame.payload)
            .map_err(|_| Error::BufferFull)?;
        return Ok(out);
    }
    let gpd = frame.header.gpd_id.ok_or(Error::Unsupported)?;
    let mic = frame.mic.ok_or(Error::Truncated)?;
    let n = nonce(&gpd, frame.header.frame_counter, direction);
    if frame.payload.len() > MAX_GPD_PAYLOAD {
        return Err(Error::BufferFull);
    }
    match level {
        SecurityLevel::Encrypted => {
            let mut ct = [0u8; MAX_GPD_PAYLOAD + MIC_LEN];
            let len = frame.payload.len();
            ct[..len].copy_from_slice(frame.payload);
            ct[len..len + MIC_LEN].copy_from_slice(&mic);
            let pt = ccm_open(provider, key, &n, frame.raw_header, &ct[..len + MIC_LEN])?;
            out.extend_from_slice(&pt).map_err(|_| Error::BufferFull)?;
        }
        SecurityLevel::FullCounterMic => {
            let mut aad = [0u8; 10 + MAX_GPD_PAYLOAD];
            let hl = frame.raw_header.len();
            let pl = frame.payload.len();
            if hl > 10 {
                return Err(Error::Unsupported);
            }
            aad[..hl].copy_from_slice(frame.raw_header);
            aad[hl..hl + pl].copy_from_slice(frame.payload);
            ccm_open(provider, key, &n, &aad[..hl + pl], &mic)?;
            out.extend_from_slice(frame.payload)
                .map_err(|_| Error::BufferFull)?;
        }
        _ => return Err(Error::Unsupported),
    }
    Ok(out)
}

/// Parameters of an outgoing protected GPDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SealContext {
    /// GPD the frame is sent by / to.
    pub gpd: GpdId,
    /// Security frame counter carried in the header.
    pub frame_counter: u32,
    /// Security level (0b10 or 0b11).
    pub level: SecurityLevel,
    /// Direction of the frame.
    pub direction: Direction,
}

/// Protect an outgoing GPD CommandID + payload.
///
/// `header` is the GPDF NWK header (NWK FC .. frame counter) exactly as it
/// will be transmitted. Writes the protected payload followed by the MIC
/// into `out` and returns the written length.
pub fn seal<P: ForwardAesProvider>(
    provider: &mut P,
    key: &AesKey,
    ctx: &SealContext,
    header: &[u8],
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize> {
    let level = ctx.level;
    if payload.len() > MAX_GPD_PAYLOAD || header.len() > 10 {
        return Err(Error::BufferFull);
    }
    let needed = payload.len() + MIC_LEN;
    if out.len() < needed {
        return Err(Error::BufferFull);
    }
    let n = nonce(&ctx.gpd, ctx.frame_counter, ctx.direction);
    match level {
        SecurityLevel::Encrypted => {
            let c = ccm_seal(provider, key, &n, header, payload)?;
            out[..c.len()].copy_from_slice(&c);
        }
        SecurityLevel::FullCounterMic => {
            let mut aad = [0u8; 10 + MAX_GPD_PAYLOAD];
            aad[..header.len()].copy_from_slice(header);
            aad[header.len()..header.len() + payload.len()].copy_from_slice(payload);
            let mic = ccm_seal(provider, key, &n, &aad[..header.len() + payload.len()], &[])?;
            out[..payload.len()].copy_from_slice(payload);
            out[payload.len()..needed].copy_from_slice(&mic);
        }
        _ => return Err(Error::Unsupported),
    }
    Ok(needed)
}

/// A TC-LK protected GPD key: 16 encrypted key bytes and a 4-byte MIC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WrappedKey {
    /// Encrypted key.
    pub key: [u8; 16],
    /// Key MIC.
    pub mic: [u8; 4],
}

fn key_wrap_params(gpd: &GpdId, direction: Direction, frame_counter: u32) -> ([u8; 13], [u8; 4]) {
    let header = gpd.low32().to_le_bytes();
    let fc = match direction {
        Direction::FromGpd => gpd.low32(),
        Direction::ToGpd => frame_counter,
    };
    (nonce(gpd, fc, direction), header)
}

/// Protect a GPD key with a TC link key (A.1.5.3.3.3).
///
/// For [`Direction::FromGpd`] (key inside a Commissioning GPDF)
/// `frame_counter` is ignored. For [`Direction::ToGpd`] (Commissioning
/// Reply) it is the counter of the triggering GPDF + 1.
pub fn wrap_key<P: ForwardAesProvider>(
    provider: &mut P,
    tc_link_key: &AesKey,
    gpd: &GpdId,
    direction: Direction,
    frame_counter: u32,
    key: &AesKey,
) -> Result<WrappedKey> {
    let (n, header) = key_wrap_params(gpd, direction, frame_counter);
    let c = ccm_seal(provider, tc_link_key, &n, &header, key)?;
    let mut w = WrappedKey {
        key: [0; 16],
        mic: [0; 4],
    };
    w.key.copy_from_slice(&c[..16]);
    w.mic.copy_from_slice(&c[16..20]);
    Ok(w)
}

/// Recover a TC-LK protected GPD key (inverse of [`wrap_key`]).
pub fn unwrap_key<P: ForwardAesProvider>(
    provider: &mut P,
    tc_link_key: &AesKey,
    gpd: &GpdId,
    direction: Direction,
    frame_counter: u32,
    wrapped: &WrappedKey,
) -> Result<AesKey> {
    let (n, header) = key_wrap_params(gpd, direction, frame_counter);
    let mut ct = [0u8; 20];
    ct[..16].copy_from_slice(&wrapped.key);
    ct[16..].copy_from_slice(&wrapped.mic);
    let pt = ccm_open(provider, tc_link_key, &n, &header, &ct)?;
    let mut key = [0u8; 16];
    key.copy_from_slice(&pt);
    Ok(key)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{self, parse};
    use zigbee_crypto::SoftwareAesProvider;

    const KEY_C0: AesKey = [
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
        0xCF,
    ];

    #[test]
    fn spec_nonce_vectors() {
        let gpd = GpdId::SrcId(0x8765_4321);
        assert_eq!(
            nonce(&gpd, 2, Direction::FromGpd),
            [
                0x21, 0x43, 0x65, 0x87, 0x21, 0x43, 0x65, 0x87, 0x02, 0, 0, 0, 0x05
            ]
        );
        assert_eq!(
            nonce(&gpd, 2, Direction::ToGpd),
            [0, 0, 0, 0, 0x21, 0x43, 0x65, 0x87, 0x02, 0, 0, 0, 0x05]
        );
    }

    /// The spec packets use NWK FC 0x8E (reserved frame type 0b10), which
    /// `frame::parse` correctly rejects, so build the parsed view directly
    /// from the spec inputs: SrcID 0x87654321, FC 2, shared key C0..CF.
    fn spec_gpdf(msdu: &[u8], level: SecurityLevel) -> frame::Gpdf<'_> {
        let n = msdu.len();
        frame::Gpdf {
            header: frame::GpdfHeader {
                frame_type: frame::FrameType::Data,
                auto_commissioning: false,
                ext_present: true,
                app_id: crate::types::ApplicationId::SrcId,
                security_level: level,
                individual_key: false,
                rx_after_tx: false,
                to_gpd: false,
                gpd_id: Some(GpdId::SrcId(0x8765_4321)),
                frame_counter: u32::from_le_bytes([msdu[6], msdu[7], msdu[8], msdu[9]]),
            },
            mac_seq: 2,
            raw_header: &msdu[..10],
            payload: &msdu[10..n - 4],
            mic: Some([msdu[n - 4], msdu[n - 3], msdu[n - 2], msdu[n - 1]]),
        }
    }

    #[test]
    fn spec_level2_shared_key_vector() {
        // A.1.5.5.3 transmitted packet: 8E 10 21 43 65 87 02 00 00 00 20 | 0F C0 B0 79
        let msdu = [
            0x8E, 0x10, 0x21, 0x43, 0x65, 0x87, 0x02, 0x00, 0x00, 0x00, 0x20, 0x0F, 0xC0, 0xB0,
            0x79,
        ];
        let f = spec_gpdf(&msdu, SecurityLevel::FullCounterMic);
        let mut p = SoftwareAesProvider;
        let pt = open(&mut p, &KEY_C0, &f, Direction::FromGpd).unwrap();
        assert_eq!(&pt[..], &[0x20]);

        let mut out = [0u8; 8];
        let n = seal(
            &mut p,
            &KEY_C0,
            &SealContext {
                gpd: GpdId::SrcId(0x8765_4321),
                frame_counter: 2,
                level: SecurityLevel::FullCounterMic,
                direction: Direction::FromGpd,
            },
            &msdu[..10],
            &[0x20],
            &mut out,
        )
        .unwrap();
        assert_eq!(&out[..n], &msdu[10..]);
    }

    #[test]
    fn spec_level3_shared_key_vector() {
        // A.1.5.5.4 transmitted packet: 8E 18 .. 02 00 00 00 | 83 | 0F 98 8F C2.
        // (The spec's step-by-step "calculation" wrongly appends the
        // plaintext to the AAD and yields D2 EC 4A B6; the transmitted
        // packet matches standard CCM*.)
        let msdu = [
            0x8E, 0x18, 0x21, 0x43, 0x65, 0x87, 0x02, 0x00, 0x00, 0x00, 0x83, 0x0F, 0x98, 0x8F,
            0xC2,
        ];
        let f = spec_gpdf(&msdu, SecurityLevel::Encrypted);
        let mut p = SoftwareAesProvider;
        let pt = open(&mut p, &KEY_C0, &f, Direction::FromGpd).unwrap();
        assert_eq!(&pt[..], &[0x20]);

        let mut out = [0u8; 8];
        let n = seal(
            &mut p,
            &KEY_C0,
            &SealContext {
                gpd: GpdId::SrcId(0x8765_4321),
                frame_counter: 2,
                level: SecurityLevel::Encrypted,
                direction: Direction::FromGpd,
            },
            &msdu[..10],
            &[0x20],
            &mut out,
        )
        .unwrap();
        assert_eq!(&out[..n], &msdu[10..]);
    }

    #[test]
    fn tampered_frame_fails_auth() {
        // Frame counter changed from 2 to 3 in the authenticated header.
        let msdu = [
            0x8E, 0x18, 0x21, 0x43, 0x65, 0x87, 0x03, 0x00, 0x00, 0x00, 0x83, 0x0F, 0x98, 0x8F,
            0xC2,
        ];
        let f = spec_gpdf(&msdu, SecurityLevel::Encrypted);
        let mut p = SoftwareAesProvider;
        assert_eq!(
            open(&mut p, &KEY_C0, &f, Direction::FromGpd),
            Err(Error::AuthFailed)
        );
    }

    #[test]
    fn spec_tclk_key_protection_vectors() {
        let mut p = SoftwareAesProvider;
        let gpd = GpdId::SrcId(0x1234_5678);
        let w = wrap_key(
            &mut p,
            &DEFAULT_TC_LINK_KEY,
            &gpd,
            Direction::FromGpd,
            0,
            &KEY_C0,
        )
        .unwrap();
        assert_eq!(
            w.key,
            [
                0x7D, 0x17, 0x7B, 0xD2, 0x9E, 0xA0, 0xFD, 0xA6, 0xB0, 0x17, 0x03, 0x65, 0x87, 0xDC,
                0x26, 0x00
            ]
        );
        assert_eq!(w.mic, [0x61, 0xF1, 0x63, 0xA9]);
        let k = unwrap_key(
            &mut p,
            &DEFAULT_TC_LINK_KEY,
            &gpd,
            Direction::FromGpd,
            0,
            &w,
        )
        .unwrap();
        assert_eq!(k, KEY_C0);

        let key2 = [
            0x16, 0x68, 0x16, 0x68, 0x16, 0x68, 0x16, 0x68, 0x16, 0x68, 0x16, 0x68, 0x16, 0x68,
            0x16, 0x68,
        ];
        let w = wrap_key(
            &mut p,
            &DEFAULT_TC_LINK_KEY,
            &gpd,
            Direction::FromGpd,
            0,
            &key2,
        )
        .unwrap();
        assert_eq!(
            w.key,
            [
                0xAB, 0xBE, 0xAF, 0x79, 0x4C, 0x0D, 0x2D, 0x09, 0x6E, 0xB6, 0xDF, 0xC6, 0x5D, 0x79,
                0xFE, 0xA7
            ]
        );
        assert_eq!(w.mic, [0x67, 0x31, 0x42, 0x6A]);
    }

    #[test]
    fn seal_open_round_trip_to_gpd() {
        let mut p = SoftwareAesProvider;
        let params = frame::GpdfParams {
            frame_type: frame::FrameType::Data,
            auto_commissioning: false,
            src_id: Some(0x1234_5678),
            security_level: SecurityLevel::Encrypted,
            individual_key: true,
            rx_after_tx: false,
            to_gpd: true,
            frame_counter: 77,
        };
        let mut buf = [0u8; 40];
        let hl = frame::write_header(&mut buf, &params).unwrap();
        let (h, rest) = buf.split_at_mut(hl);
        let n = seal(
            &mut p,
            &KEY_C0,
            &SealContext {
                gpd: GpdId::SrcId(0x1234_5678),
                frame_counter: 77,
                level: SecurityLevel::Encrypted,
                direction: Direction::ToGpd,
            },
            h,
            &[0xF0, 0x00],
            rest,
        )
        .unwrap();
        let f = parse(&buf[..hl + n], 0, None).unwrap();
        assert!(f.header.to_gpd);
        let pt = open(&mut p, &KEY_C0, &f, Direction::ToGpd).unwrap();
        assert_eq!(&pt[..], &[0xF0, 0x00]);
        assert_eq!(
            open(&mut p, &KEY_C0, &f, Direction::FromGpd),
            Err(Error::AuthFailed)
        );
    }
}
