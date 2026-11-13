//! GPDF (Green Power Device Frame) parsing and building.
//!
//! A GPDF is an IEEE 802.15.4 data frame whose MAC payload starts with a
//! NWK Frame Control field carrying protocol version 3 (GP spec A.1.4.1).
//! [`parse`] operates on that MAC payload (MSDU). The MAC sequence number
//! and, for ApplicationID 0b010, the MAC source IEEE address come from the
//! MAC layer.

use crate::codec::{Reader, Writer};
use crate::types::{ApplicationId, GpdId, IeeeAddress, SecurityLevel};
use crate::{Error, Result};

/// NWK protocol version carried by every GPDF.
pub const GP_PROTOCOL_VERSION: u8 = 3;
/// MIC length used by security levels 0b10 and 0b11.
pub const MIC_LEN: usize = 4;
/// Largest GPD CommandID + payload length that fits a GPDF with full
/// security and a minimal MAC header (127 − 7 MAC − 2 FCS − 14 NWK/MIC).
pub const MAX_GPD_PAYLOAD: usize = 104;
/// Largest NWK-part GPDF built by [`write`].
pub const MAX_GPDF_NWK_LEN: usize = 1 + 1 + 4 + 4 + MAX_GPD_PAYLOAD + MIC_LEN;
/// Length of the MAC header written by [`write_mac_header`].
pub const GPD_MAC_HEADER_LEN: usize = 7;

/// GPDF frame type (NWK FC bits 0-1 with protocol version 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    /// Data frame.
    Data,
    /// Maintenance frame.
    Maintenance,
}

/// True if `msdu` starts with a NWK FC of protocol version 3 and a defined
/// GP frame type. Cheap pre-check for the NWK receive demultiplexer.
pub fn is_gpdf(msdu: &[u8]) -> bool {
    match msdu.first() {
        Some(&fc) => (fc >> 2) & 0x0F == GP_PROTOCOL_VERSION && fc & 0x03 <= 1,
        None => false,
    }
}

/// Decoded GPDF header fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpdfHeader {
    /// Frame type.
    pub frame_type: FrameType,
    /// Auto-Commissioning sub-field (1 = GPD does not implement Commissioning).
    pub auto_commissioning: bool,
    /// Whether the Extended NWK Frame Control field is present.
    pub ext_present: bool,
    /// ApplicationID.
    pub app_id: ApplicationId,
    /// Security level.
    pub security_level: SecurityLevel,
    /// SecurityKey sub-field (1 = individual key).
    pub individual_key: bool,
    /// RxAfterTx sub-field.
    pub rx_after_tx: bool,
    /// Direction sub-field (true = GPP → GPD).
    pub to_gpd: bool,
    /// GPD identity, if carried by the frame.
    pub gpd_id: Option<GpdId>,
    /// Security frame counter (0 when not present).
    pub frame_counter: u32,
}

/// A parsed GPDF borrowing from the receive buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gpdf<'a> {
    /// Header fields.
    pub header: GpdfHeader,
    /// MAC sequence number of the frame.
    pub mac_seq: u8,
    /// NWK FC .. frame counter, as received (the CCM* header for levels 2/3).
    pub raw_header: &'a [u8],
    /// GPD CommandID and payload (ciphertext for security level 0b11).
    pub payload: &'a [u8],
    /// MIC, for protected frames.
    pub mic: Option<[u8; MIC_LEN]>,
}

impl Gpdf<'_> {
    /// GPD CommandID if the payload is in the clear (level < 0b11).
    pub fn clear_command_id(&self) -> Option<u8> {
        if self.header.security_level == SecurityLevel::Encrypted {
            None
        } else {
            self.payload.first().copied()
        }
    }
}

/// Parse the NWK part of a GPDF.
///
/// `mac_src_ieee` must carry the MAC source extended address for
/// ApplicationID 0b010 frames. Security level 0b01 (deprecated) and
/// protected ApplicationID 0b010 frames return [`Error::Unsupported`].
pub fn parse(msdu: &[u8], mac_seq: u8, mac_src_ieee: Option<IeeeAddress>) -> Result<Gpdf<'_>> {
    let mut r = Reader::new(msdu);
    let fc = r.u8()?;
    if (fc >> 2) & 0x0F != GP_PROTOCOL_VERSION {
        return Err(Error::NotGpdf);
    }
    let frame_type = match fc & 0x03 {
        0 => FrameType::Data,
        1 => FrameType::Maintenance,
        _ => return Err(Error::Unsupported),
    };
    let auto_commissioning = fc & 0x40 != 0;
    let ext_present = fc & 0x80 != 0;

    let mut header = GpdfHeader {
        frame_type,
        auto_commissioning,
        ext_present,
        app_id: ApplicationId::SrcId,
        security_level: SecurityLevel::None,
        individual_key: false,
        rx_after_tx: false,
        to_gpd: false,
        gpd_id: None,
        frame_counter: 0,
    };
    if ext_present {
        let ext = r.u8()?;
        header.app_id = ApplicationId::from_bits(ext)?;
        header.security_level = SecurityLevel::from_bits(ext >> 3);
        header.individual_key = ext & 0x20 != 0;
        header.rx_after_tx = ext & 0x40 != 0;
        header.to_gpd = ext & 0x80 != 0;
    }
    if header.app_id == ApplicationId::Lped {
        return Err(Error::Unsupported);
    }
    if header.security_level == SecurityLevel::Reserved {
        return Err(Error::Unsupported);
    }
    if frame_type == FrameType::Maintenance && header.security_level != SecurityLevel::None {
        return Err(Error::Unsupported);
    }

    match header.app_id {
        ApplicationId::SrcId => {
            if frame_type == FrameType::Data || ext_present {
                header.gpd_id = Some(GpdId::SrcId(r.u32()?));
            }
        }
        ApplicationId::Ieee => {
            if header.security_level != SecurityLevel::None {
                return Err(Error::Unsupported);
            }
            let endpoint = r.u8()?;
            let ieee = mac_src_ieee.ok_or(Error::Unsupported)?;
            header.gpd_id = Some(GpdId::Ieee { ieee, endpoint });
        }
        ApplicationId::Lped => unreachable!(),
    }

    let protected = header.security_level.is_protected();
    if protected {
        header.frame_counter = r.u32()?;
    }
    let header_len = msdu.len() - r.remaining();
    let body = r.rest();
    let (payload, mic) = if protected {
        if body.len() < MIC_LEN + 1 {
            return Err(Error::Truncated);
        }
        let split = body.len() - MIC_LEN;
        let mut mic = [0u8; MIC_LEN];
        mic.copy_from_slice(&body[split..]);
        (&body[..split], Some(mic))
    } else {
        (body, None)
    };
    if payload.is_empty() {
        return Err(Error::Truncated);
    }
    Ok(Gpdf {
        header,
        mac_seq,
        raw_header: &msdu[..header_len],
        payload,
        mic,
    })
}

/// Parameters for building an outgoing GPDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpdfParams {
    /// Frame type.
    pub frame_type: FrameType,
    /// Auto-Commissioning sub-field.
    pub auto_commissioning: bool,
    /// GPD SrcID (ApplicationID 0b000 only; IEEE GPDs are not built here).
    /// `None` omits the SrcID (maintenance frames).
    pub src_id: Option<u32>,
    /// Security level (0b00, 0b10 or 0b11).
    pub security_level: SecurityLevel,
    /// SecurityKey sub-field.
    pub individual_key: bool,
    /// RxAfterTx sub-field.
    pub rx_after_tx: bool,
    /// Direction (true = GPP → GPD).
    pub to_gpd: bool,
    /// Security frame counter (protected levels only).
    pub frame_counter: u32,
}

impl GpdfParams {
    /// Whether the Extended NWK FC must be present (A.1.4.1.3).
    pub fn needs_ext(&self) -> bool {
        self.security_level != SecurityLevel::None
            || self.rx_after_tx
            || self.to_gpd
            || (self.frame_type == FrameType::Maintenance && self.src_id.is_some())
    }

    /// The NWK FC and Extended NWK FC bytes.
    pub fn control_bytes(&self) -> (u8, Option<u8>) {
        let ext = self.needs_ext();
        let ft = match self.frame_type {
            FrameType::Data => 0,
            FrameType::Maintenance => 1,
        };
        let fc = ft
            | (GP_PROTOCOL_VERSION << 2)
            | if self.auto_commissioning { 0x40 } else { 0 }
            | if ext { 0x80 } else { 0 };
        let ext_fc = ext.then_some(
            (ApplicationId::SrcId as u8)
                | ((self.security_level as u8) << 3)
                | if self.individual_key { 0x20 } else { 0 }
                | if self.rx_after_tx { 0x40 } else { 0 }
                | if self.to_gpd { 0x80 } else { 0 },
        );
        (fc, ext_fc)
    }
}

/// Write the GPDF NWK header (NWK FC .. frame counter) into `out` and return
/// its length. This is also the CCM* header for security levels 0b10/0b11.
pub fn write_header(out: &mut [u8], p: &GpdfParams) -> Result<usize> {
    if p.security_level == SecurityLevel::Reserved {
        return Err(Error::Unsupported);
    }
    if p.frame_type == FrameType::Data && p.src_id.is_none() {
        return Err(Error::Unsupported);
    }
    let mut w = Writer::new(out);
    let (fc, ext) = p.control_bytes();
    w.u8(fc)?;
    if let Some(ext) = ext {
        w.u8(ext)?;
    }
    if let Some(src) = p.src_id {
        w.u32(src)?;
    }
    if p.security_level.is_protected() {
        w.u32(p.frame_counter)?;
    }
    Ok(w.len())
}

/// Write the minimal GPDF MAC header: data frame, no ACK request,
/// destination PAN 0xFFFF / short 0xFFFF, no source address.
pub fn write_mac_header(out: &mut [u8], seq: u8) -> Result<usize> {
    let mut w = Writer::new(out);
    w.bytes(&[0x01, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF])?;
    Ok(w.len())
}

/// MAC-level information extracted from a raw 802.15.4 frame by
/// [`parse_mac`] (used by GPD-side receivers that have no MAC layer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacFrameInfo {
    /// MAC sequence number.
    pub seq: u8,
    /// MAC source extended address, when present.
    pub src_ieee: Option<IeeeAddress>,
    /// Offset of the MAC payload within the frame.
    pub payload_offset: usize,
}

/// Parse an unsecured IEEE 802.15.4-2006 data frame header (without FCS).
pub fn parse_mac(frame: &[u8]) -> Result<MacFrameInfo> {
    let mut r = Reader::new(frame);
    let fc = r.u16()?;
    if fc & 0x07 != 0x01 {
        return Err(Error::NotGpdf);
    }
    if fc & 0x08 != 0 {
        return Err(Error::Unsupported);
    }
    let pan_compress = fc & 0x40 != 0;
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let seq = r.u8()?;
    match dst_mode {
        0 => {}
        2 => {
            r.bytes(4)?;
        }
        3 => {
            r.bytes(10)?;
        }
        _ => return Err(Error::Unsupported),
    }
    if src_mode != 0 && !(pan_compress && dst_mode != 0) {
        r.bytes(2)?;
    }
    let src_ieee = match src_mode {
        0 => None,
        2 => {
            r.bytes(2)?;
            None
        }
        3 => Some(r.array::<8>()?),
        _ => return Err(Error::Unsupported),
    };
    Ok(MacFrameInfo {
        seq,
        src_ieee,
        payload_offset: frame.len() - r.remaining(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // GP spec A.1.5.5.4 inputs (SrcID 0x87654321, FC 2, level 3 shared
    // key C0..CF, command 0x20) with the data frame type NWK FC 0x8C. The
    // spec's packet uses the reserved frame type 0x8E; ciphertext/MIC here
    // were computed with an independent CCM* implementation.
    const L3_SHARED: [u8; 22] = [
        0x01, 0x08, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x8C, 0x18, 0x21, 0x43, 0x65, 0x87, 0x02, 0x00,
        0x00, 0x00, 0x83, 0xCA, 0x43, 0x24, 0xDD,
    ];

    #[test]
    fn parses_spec_level3_frame() {
        let mac = parse_mac(&L3_SHARED).unwrap();
        assert_eq!(mac.seq, 2);
        assert_eq!(mac.payload_offset, 7);
        let msdu = &L3_SHARED[mac.payload_offset..];
        assert!(is_gpdf(msdu));
        let f = parse(msdu, mac.seq, None).unwrap();
        assert_eq!(f.header.gpd_id, Some(GpdId::SrcId(0x8765_4321)));
        assert_eq!(f.header.security_level, SecurityLevel::Encrypted);
        assert_eq!(f.header.frame_counter, 2);
        assert_eq!(f.payload, &[0x83]);
        assert_eq!(f.mic, Some([0xCA, 0x43, 0x24, 0xDD]));
        assert_eq!(f.raw_header, &msdu[..10]);
    }

    #[test]
    fn rejects_non_gp_and_level1() {
        assert_eq!(parse(&[0x08, 0x02], 0, None), Err(Error::NotGpdf));
        // Level 0b01 (deprecated).
        let l1 = [0x8C, 0x08, 0x21, 0x43, 0x65, 0x87, 0x20, 0xB9, 0xB3];
        assert_eq!(parse(&l1, 2, None), Err(Error::Unsupported));
    }

    #[test]
    fn unprotected_and_maintenance_frames() {
        // No ext FC: App 0, level 0, SrcID present, Toggle.
        let f = parse(&[0x0C, 0x78, 0x56, 0x34, 0x12, 0x22], 9, None).unwrap();
        assert_eq!(f.header.gpd_id, Some(GpdId::SrcId(0x1234_5678)));
        assert_eq!(f.payload, &[0x22]);
        assert_eq!(f.mic, None);
        // Maintenance frame without ext FC: no SrcID (Channel Request).
        let f = parse(&[0x0D, 0xE3, 0x45], 1, None).unwrap();
        assert_eq!(f.header.frame_type, FrameType::Maintenance);
        assert_eq!(f.header.gpd_id, None);
        assert_eq!(f.payload, &[0xE3, 0x45]);
    }

    #[test]
    fn header_builder_round_trips() {
        let p = GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(0x8765_4321),
            security_level: SecurityLevel::Encrypted,
            individual_key: false,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: 2,
        };
        let mut buf = [0u8; 16];
        let n = write_header(&mut buf, &p).unwrap();
        assert_eq!(&buf[..n], &L3_SHARED[7..17]);
    }
}
