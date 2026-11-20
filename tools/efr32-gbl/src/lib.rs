//! GBL3 (Gecko Bootloader upgrade file) writer and reader, plus the
//! application-image packaging the Gecko bootloader expects.
//!
//! Format (GSDK 4.5.0 `platform/bootloader/parser/gbl/btl_gbl_format.h`):
//! a sequence of `{u32 tag, u32 length, length bytes}` records, little
//! endian:
//!
//! | tag | record |
//! |---|---|
//! | `0x03A617EB` | header: `u32 version = 0x0300_0000`, `u32 type` (0 = plain; bit 0 AES-CCM encrypted, bit 8 ECDSA signed) |
//! | `0xF40A0AF4` | application: `ApplicationData_t` (type, version, capabilities, product ID) |
//! | `0xFD0303FD` | erase-and-program: `u32 address`, data |
//! | `0xF60808F6` | metadata (opaque; Nabu Casa flashers read JSON here) |
//! | `0xFC0404FC` | end: CRC-32 (IEEE, zlib) of every byte before the CRC |
//!
//! [`write()`] reproduces Simplicity Commander 1v22p1 `gbl create --app
//! [--metadata]` byte for byte (tests against Commander output), including
//! its `0xFF` padding of the file to a multiple of four bytes after the
//! end record. Only
//! plain (unsigned, unencrypted) GBLs are written; [`parse`] reports
//! signature, encryption, bootloader and SE-upgrade tags so a bootloader's
//! requirements can be checked against stock images and a host can refuse
//! images that would replace the bootloader.
//!
//! ZRHP NCP images carry a build record (`ZRHPBID1`, see
//! `products/efr32mg24/src/gecko.rs`). [`prepare_zrhp_build`] stores the
//! build ID in it (the first 8 bytes of the SHA-256 of the image with the
//! field zeroed), and the GBL then carries the ZRHP metadata record of
//! `docs/book/src/advanced/host-protocol.md` 7.9:
//! `"ZRHP", u8 version = 1, [8] build_id, u8 n, n bytes firmware version`.

use std::fmt;

pub const TAG_HEADER: u32 = 0x03A6_17EB;
pub const TAG_BOOTLOADER: u32 = 0xF509_09F5;
pub const TAG_APPLICATION: u32 = 0xF40A_0AF4;
pub const TAG_METADATA: u32 = 0xF608_08F6;
pub const TAG_PROG: u32 = 0xFE01_01FE;
pub const TAG_PROG_LZ4: u32 = 0xFD05_05FD;
pub const TAG_PROG_LZMA: u32 = 0xFD07_07FD;
pub const TAG_ERASEPROG: u32 = 0xFD03_03FD;
pub const TAG_END: u32 = 0xFC04_04FC;
pub const TAG_SE_UPGRADE: u32 = 0x5EA6_17EB;
pub const TAG_VERSION_DEPENDENCY: u32 = 0x76A6_17EB;
pub const TAG_ENC_HEADER: u32 = 0xFB05_05FB;
pub const TAG_ENC_INIT: u32 = 0xFA06_06FA;
pub const TAG_ENC_GBL_DATA: u32 = 0xF907_07F9;
pub const TAG_SIGNATURE_ECDSA_P256: u32 = 0xF70A_0AF7;
pub const TAG_CERTIFICATE_ECDSA_P256: u32 = 0xF30B_0BF3;

pub const GBL_VERSION: u32 = 0x0300_0000;
pub const GBL_TYPE_ENCRYPTION_AESCCM: u32 = 0x0000_0001;
pub const GBL_TYPE_SIGNATURE_ECDSA: u32 = 0x0000_0100;

/// `APPLICATION_PROPERTIES_MAGIC`.
pub const APP_PROPERTIES_MAGIC: [u8; 16] = [
    0x13, 0xB7, 0x79, 0xFA, 0xC9, 0x25, 0xDD, 0xB7, 0xAD, 0xF3, 0xCF, 0xE0, 0xF1, 0xB6, 0x14, 0xB8,
];
/// Size of `ApplicationProperties_t` v1.2.
pub const APP_PROPERTIES_LEN: usize = 80;
/// Vector-table word that points at the properties (GSDK startup files
/// and Commander use the reserved exception slot 13).
pub const APP_PROPERTIES_VECTOR_WORD: usize = 13;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Truncated,
    BadHeader,
    BadCrc { stored: u32, computed: u32 },
    MissingEnd,
    NoAppProperties,
    AmbiguousAppProperties,
    Word13Conflict { found: u32, expected: u32 },
    BadVectorTable(&'static str),
    Elf(&'static str),
    Layout(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Truncated => write!(f, "truncated GBL"),
            Error::BadHeader => write!(f, "not a GBL3 file (bad header tag/version)"),
            Error::BadCrc { stored, computed } => {
                write!(
                    f,
                    "CRC mismatch: stored {stored:#010x}, computed {computed:#010x}"
                )
            }
            Error::MissingEnd => write!(f, "no end tag"),
            Error::NoAppProperties => write!(f, "no ApplicationProperties_t magic in the image"),
            Error::AmbiguousAppProperties => {
                write!(
                    f,
                    "more than one ApplicationProperties_t magic in the image"
                )
            }
            Error::Word13Conflict { found, expected } => write!(
                f,
                "vector word 13 is {found:#010x}, not 0 or the properties address {expected:#010x}"
            ),
            Error::BadVectorTable(what) => write!(f, "vector table: {what}"),
            Error::Elf(what) => write!(f, "ELF: {what}"),
            Error::Layout(what) => write!(f, "layout: {what}"),
        }
    }
}

impl std::error::Error for Error {}

/// `ApplicationData_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApplicationData {
    pub app_type: u32,
    pub version: u32,
    pub capabilities: u32,
    pub product_id: [u8; 16],
}

impl ApplicationData {
    pub fn to_bytes(&self) -> [u8; 28] {
        let mut out = [0u8; 28];
        out[0..4].copy_from_slice(&self.app_type.to_le_bytes());
        out[4..8].copy_from_slice(&self.version.to_le_bytes());
        out[8..12].copy_from_slice(&self.capabilities.to_le_bytes());
        out[12..28].copy_from_slice(&self.product_id);
        out
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 28 {
            return None;
        }
        let word = |i: usize| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap());
        Some(Self {
            app_type: word(0),
            version: word(4),
            capabilities: word(8),
            product_id: bytes[12..28].try_into().unwrap(),
        })
    }
}

/// The parts of `ApplicationProperties_t` the packager checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AppProperties {
    /// Bus address of the record.
    pub address: u32,
    pub struct_version: u32,
    pub signature_type: u32,
    pub app: ApplicationData,
}

/// Magic of the ZRHP build record in an NCP image.
pub const ZRHP_BUILD_MAGIC: [u8; 8] = *b"ZRHPBID1";
/// Size of the build record: magic, build ID, version length, 31 bytes.
pub const ZRHP_BUILD_LEN: usize = 48;
/// Version of the ZRHP GBL metadata record.
pub const ZRHP_METADATA_VERSION: u8 = 1;

/// The ZRHP identity of a packaged NCP image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZrhpBuild {
    /// Bus address of the build record.
    pub address: u32,
    pub build_id: [u8; 8],
    pub fw_version: Vec<u8>,
}

impl ZrhpBuild {
    /// The GBL metadata record: `"ZRHP", u8 version, [8] build_id, u8 n,
    /// n bytes firmware version`.
    pub fn metadata(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(14 + self.fw_version.len());
        out.extend_from_slice(b"ZRHP");
        out.push(ZRHP_METADATA_VERSION);
        out.extend_from_slice(&self.build_id);
        out.push(self.fw_version.len() as u8);
        out.extend_from_slice(&self.fw_version);
        out
    }

    /// Decode a ZRHP metadata record (`None` for any other metadata).
    pub fn from_metadata(metadata: &[u8]) -> Option<([u8; 8], Vec<u8>)> {
        if metadata.len() < 14 || &metadata[0..4] != b"ZRHP" || metadata[4] != 1 {
            return None;
        }
        let n = metadata[13] as usize;
        if metadata.len() != 14 + n {
            return None;
        }
        Some((metadata[5..13].try_into().unwrap(), metadata[14..].to_vec()))
    }
}

/// Find the ZRHP build record (if any), store the build ID derived from the
/// image in it and return the identity. Call after [`prepare_app_image`]
/// (the hash covers the final image, vector word 13 included).
pub fn prepare_zrhp_build(image: &mut [u8], base: u32) -> Result<Option<ZrhpBuild>, Error> {
    use sha2::{Digest, Sha256};
    let mut found = None;
    let mut offset = 0;
    while offset + ZRHP_BUILD_LEN <= image.len() {
        if image[offset..offset + 8] == ZRHP_BUILD_MAGIC {
            if found.is_some() {
                return Err(Error::Layout("more than one ZRHP build record".into()));
            }
            found = Some(offset);
        }
        offset += 4;
    }
    let Some(offset) = found else {
        return Ok(None);
    };
    let n = image[offset + 16] as usize;
    if n > ZRHP_BUILD_LEN - 17 {
        return Err(Error::Layout(
            "ZRHP build record version is too long".into(),
        ));
    }
    let fw_version = image[offset + 17..offset + 17 + n].to_vec();
    image[offset + 8..offset + 16].fill(0);
    let digest = Sha256::digest(&*image);
    let build_id: [u8; 8] = digest[..8].try_into().unwrap();
    image[offset + 8..offset + 16].copy_from_slice(&build_id);
    Ok(Some(ZrhpBuild {
        address: base + offset as u32,
        build_id,
        fw_version,
    }))
}

/// CRC-32 (IEEE 802.3, reflected, init/xorout `0xFFFF_FFFF`), as zlib.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn push_tag(out: &mut Vec<u8>, tag: u32, body: &[u8]) {
    out.extend_from_slice(&tag.to_le_bytes());
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(body);
}

/// Write a plain GBL3: header, application, erase-and-program at
/// `address`, optional metadata, end with CRC (Commander's order).
pub fn write(
    app: &ApplicationData,
    address: u32,
    image: &[u8],
    metadata: Option<&[u8]>,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(image.len() + 128);
    let mut header = [0u8; 8];
    header[0..4].copy_from_slice(&GBL_VERSION.to_le_bytes());
    push_tag(&mut out, TAG_HEADER, &header);
    push_tag(&mut out, TAG_APPLICATION, &app.to_bytes());
    let mut prog = Vec::with_capacity(4 + image.len());
    prog.extend_from_slice(&address.to_le_bytes());
    prog.extend_from_slice(image);
    push_tag(&mut out, TAG_ERASEPROG, &prog);
    if let Some(metadata) = metadata {
        push_tag(&mut out, TAG_METADATA, metadata);
    }
    out.extend_from_slice(&TAG_END.to_le_bytes());
    out.extend_from_slice(&4u32.to_le_bytes());
    let crc = crc32(&out);
    out.extend_from_slice(&crc.to_le_bytes());
    // Commander pads the file (not a record) to a multiple of four bytes.
    while out.len() % 4 != 0 {
        out.push(0xFF);
    }
    out
}

/// One parsed record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    pub tag: u32,
    pub offset: usize,
    pub body: Vec<u8>,
}

/// Summary of a GBL file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Parsed {
    pub records: Vec<Record>,
    pub gbl_type: u32,
    pub application: Option<ApplicationData>,
    /// `(address, length)` of every program record.
    pub programs: Vec<(u32, usize)>,
    pub metadata: Option<Vec<u8>>,
    pub signed: bool,
    pub encrypted: bool,
    /// A bootloader (`0xF50909F5`) or SE-upgrade (`0x5EA617EB`) record: the
    /// file would replace the bootloader or the Secure Engine firmware.
    pub updates_bootloader: bool,
    pub crc: u32,
}

/// Parse and verify a GBL3 file (header, record chain, end CRC).
pub fn parse(data: &[u8]) -> Result<Parsed, Error> {
    let mut records = Vec::new();
    let mut offset = 0;
    let mut end_crc = None;
    while offset + 8 <= data.len() {
        let tag = u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        let length = u32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap()) as usize;
        let body_start = offset + 8;
        let body_end = body_start.checked_add(length).ok_or(Error::Truncated)?;
        if body_end > data.len() {
            return Err(Error::Truncated);
        }
        records.push(Record {
            tag,
            offset,
            body: data[body_start..body_end].to_vec(),
        });
        offset = body_end;
        if tag == TAG_END {
            if length != 4 {
                return Err(Error::Truncated);
            }
            let stored = u32::from_le_bytes(data[body_start..body_end].try_into().unwrap());
            let computed = crc32(&data[..body_start]);
            if stored != computed {
                return Err(Error::BadCrc { stored, computed });
            }
            end_crc = Some(stored);
            break;
        }
    }
    let crc = end_crc.ok_or(Error::MissingEnd)?;
    let first = records.first().ok_or(Error::BadHeader)?;
    if first.tag != TAG_HEADER || first.body.len() < 8 {
        return Err(Error::BadHeader);
    }
    let version = u32::from_le_bytes(first.body[0..4].try_into().unwrap());
    if version & 0xFF00_0000 != GBL_VERSION {
        return Err(Error::BadHeader);
    }
    let gbl_type = u32::from_le_bytes(first.body[4..8].try_into().unwrap());
    let mut parsed = Parsed {
        gbl_type,
        application: None,
        programs: Vec::new(),
        metadata: None,
        signed: gbl_type & GBL_TYPE_SIGNATURE_ECDSA != 0,
        encrypted: gbl_type & GBL_TYPE_ENCRYPTION_AESCCM != 0,
        updates_bootloader: false,
        crc,
        records: Vec::new(),
    };
    for record in &records {
        match record.tag {
            TAG_APPLICATION => parsed.application = ApplicationData::from_bytes(&record.body),
            TAG_PROG | TAG_ERASEPROG | TAG_PROG_LZ4 | TAG_PROG_LZMA if record.body.len() >= 4 => {
                let address = u32::from_le_bytes(record.body[0..4].try_into().unwrap());
                parsed.programs.push((address, record.body.len() - 4));
            }
            TAG_METADATA => parsed.metadata = Some(record.body.clone()),
            TAG_SIGNATURE_ECDSA_P256 | TAG_CERTIFICATE_ECDSA_P256 => parsed.signed = true,
            TAG_ENC_HEADER | TAG_ENC_INIT | TAG_ENC_GBL_DATA => parsed.encrypted = true,
            TAG_BOOTLOADER | TAG_SE_UPGRADE => parsed.updates_bootloader = true,
            _ => {}
        }
    }
    parsed.records = records;
    Ok(parsed)
}

/// Find the single `ApplicationProperties_t` in `image` (loaded at `base`).
pub fn find_app_properties(image: &[u8], base: u32) -> Result<AppProperties, Error> {
    let mut found = None;
    let mut offset = 0;
    while offset + APP_PROPERTIES_LEN <= image.len() {
        if image[offset..offset + 16] == APP_PROPERTIES_MAGIC {
            if found.is_some() {
                return Err(Error::AmbiguousAppProperties);
            }
            found = Some(offset);
        }
        offset += 4;
    }
    let offset = found.ok_or(Error::NoAppProperties)?;
    let word = |i: usize| u32::from_le_bytes(image[offset + i..offset + i + 4].try_into().unwrap());
    Ok(AppProperties {
        address: base + offset as u32,
        struct_version: word(16),
        signature_type: word(20),
        app: ApplicationData::from_bytes(&image[offset + 28..offset + 56]).unwrap(),
    })
}

/// Store the properties address in vector word 13 (as GSDK startup code
/// does), check the vector table, and return the properties.
pub fn prepare_app_image(
    image: &mut [u8],
    base: u32,
    ram: core::ops::Range<u32>,
) -> Result<AppProperties, Error> {
    if image.len() < 16 * 4 {
        return Err(Error::BadVectorTable("image shorter than the core vectors"));
    }
    let word =
        |image: &[u8], i: usize| u32::from_le_bytes(image[4 * i..4 * i + 4].try_into().unwrap());
    let sp = word(image, 0);
    let reset = word(image, 1);
    if !(ram.start < sp && sp <= ram.end) {
        return Err(Error::BadVectorTable("initial SP outside RAM"));
    }
    let end = base + image.len() as u32;
    if reset & 1 == 0 || !(base..end).contains(&(reset & !1)) {
        return Err(Error::BadVectorTable(
            "reset vector outside the image or not Thumb",
        ));
    }
    let props = find_app_properties(image, base)?;
    let current = word(image, APP_PROPERTIES_VECTOR_WORD);
    if current != 0 && current != props.address {
        return Err(Error::Word13Conflict {
            found: current,
            expected: props.address,
        });
    }
    let at = 4 * APP_PROPERTIES_VECTOR_WORD;
    image[at..at + 4].copy_from_slice(&props.address.to_le_bytes());
    Ok(props)
}

/// Flash image of an ELF32 little-endian file: every `PT_LOAD` segment with
/// file contents whose physical (load) address lies in `flash`, merged into
/// one image (gaps filled with `0xFF`). Returns `(base, bytes)`.
pub fn elf_flash_image(elf: &[u8], flash: core::ops::Range<u32>) -> Result<(u32, Vec<u8>), Error> {
    if elf.len() < 52 || elf[0..4] != [0x7F, b'E', b'L', b'F'] {
        return Err(Error::Elf("not an ELF file"));
    }
    if elf[4] != 1 || elf[5] != 1 {
        return Err(Error::Elf("not ELF32 little endian"));
    }
    let half = |i: usize| u16::from_le_bytes(elf[i..i + 2].try_into().unwrap()) as usize;
    let word = |i: usize| u32::from_le_bytes(elf[i..i + 4].try_into().unwrap());
    let phoff = word(28) as usize;
    let phentsize = half(42);
    let phnum = half(44);
    let mut segments = Vec::new();
    for index in 0..phnum {
        let at = phoff + index * phentsize;
        if at + 32 > elf.len() {
            return Err(Error::Elf("program header outside the file"));
        }
        let p_type = word(at);
        let p_offset = word(at + 4) as usize;
        let p_paddr = word(at + 12);
        let p_filesz = word(at + 16) as usize;
        if p_type != 1 || p_filesz == 0 {
            continue;
        }
        if !flash.contains(&p_paddr) {
            continue;
        }
        if p_offset + p_filesz > elf.len() {
            return Err(Error::Elf("segment outside the file"));
        }
        let end = p_paddr as u64 + p_filesz as u64;
        if end > flash.end as u64 {
            return Err(Error::Elf("segment runs past the flash range"));
        }
        segments.push((p_paddr, &elf[p_offset..p_offset + p_filesz]));
    }
    if segments.is_empty() {
        return Err(Error::Elf("no loadable flash segment"));
    }
    segments.sort_by_key(|s| s.0);
    let base = segments[0].0;
    let end = segments
        .iter()
        .map(|(a, d)| *a + d.len() as u32)
        .max()
        .unwrap();
    let mut image = vec![0xFF; (end - base) as usize];
    for (address, data) in segments {
        let at = (address - base) as usize;
        if image[at..at + data.len()].iter().any(|&b| b != 0xFF) {
            return Err(Error::Elf("overlapping flash segments"));
        }
        image[at..at + data.len()].copy_from_slice(data);
    }
    Ok((base, image))
}

/// Intel HEX of `image` at `base` (extended linear addressing, 16-byte
/// data records), for J-Link `loadfile`.
pub fn intel_hex(image: &[u8], base: u32) -> String {
    fn record(out: &mut String, kind: u8, address: u16, data: &[u8]) {
        let mut sum = data.len() as u8;
        sum = sum
            .wrapping_add((address >> 8) as u8)
            .wrapping_add(address as u8);
        sum = sum.wrapping_add(kind);
        out.push_str(&format!(":{:02X}{:04X}{:02X}", data.len(), address, kind));
        for &byte in data {
            out.push_str(&format!("{byte:02X}"));
            sum = sum.wrapping_add(byte);
        }
        out.push_str(&format!("{:02X}\n", sum.wrapping_neg()));
    }
    let mut out = String::new();
    let mut upper = None;
    for (index, chunk) in image.chunks(16).enumerate() {
        let address = base + 16 * index as u32;
        if upper != Some(address >> 16) {
            upper = Some(address >> 16);
            record(&mut out, 4, 0, &((address >> 16) as u16).to_be_bytes());
        }
        record(&mut out, 0, address as u16, chunk);
    }
    record(&mut out, 1, 0, &[]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_matches_zlib() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn zrhp_build_record_and_metadata() {
        let mut image = vec![0xFFu8; 256];
        let at = 64;
        image[at..at + 8].copy_from_slice(&ZRHP_BUILD_MAGIC);
        image[at + 8..at + 16].fill(0);
        image[at + 16] = 5;
        image[at + 17..at + 22].copy_from_slice(b"0.1.0");
        let build = prepare_zrhp_build(&mut image, 0x0800_6000)
            .unwrap()
            .unwrap();
        assert_eq!(build.address, 0x0800_6040);
        assert_eq!(&image[at + 8..at + 16], &build.build_id);
        assert_ne!(build.build_id, [0; 8]);
        // Deterministic: re-packaging the packaged image gives the same ID.
        let again = prepare_zrhp_build(&mut image, 0x0800_6000)
            .unwrap()
            .unwrap();
        assert_eq!(again.build_id, build.build_id);
        let metadata = build.metadata();
        assert_eq!(&metadata[..5], b"ZRHP\x01");
        assert_eq!(metadata.len(), 14 + 5);
        assert_eq!(
            ZrhpBuild::from_metadata(&metadata),
            Some((build.build_id, b"0.1.0".to_vec()))
        );
        assert_eq!(ZrhpBuild::from_metadata(b"{\"fw_type\": 1}"), None);
        let mut plain = vec![0u8; 64];
        assert_eq!(prepare_zrhp_build(&mut plain, 0).unwrap(), None);
    }

    #[test]
    fn bootloader_records_are_flagged() {
        let app = ApplicationData {
            app_type: 1,
            version: 0,
            capabilities: 0,
            product_id: [0; 16],
        };
        let plain = write(&app, 0x0800_6000, &[0; 8], None);
        assert!(!parse(&plain).unwrap().updates_bootloader);
        // Splice a bootloader record in front of the end tag and fix the CRC.
        let end = plain.len() - 12;
        let mut gbl = plain[..end].to_vec();
        gbl.extend_from_slice(&TAG_BOOTLOADER.to_le_bytes());
        gbl.extend_from_slice(&4u32.to_le_bytes());
        gbl.extend_from_slice(&[0; 4]);
        gbl.extend_from_slice(&TAG_END.to_le_bytes());
        gbl.extend_from_slice(&4u32.to_le_bytes());
        let crc = crc32(&gbl);
        gbl.extend_from_slice(&crc.to_le_bytes());
        assert!(parse(&gbl).unwrap().updates_bootloader);
    }

    #[test]
    fn intel_hex_records() {
        let hex = intel_hex(&[0x01, 0x02], 0x0800_6000);
        assert_eq!(hex, ":020000040800F2\n:0260000001029B\n:00000001FF\n");
    }
}
