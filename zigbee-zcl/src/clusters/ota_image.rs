//! Zigbee OTA image file format parser.
//!
//! Parses the OTA Upgrade Image file header (ZCL spec Table 11-2).
//! The header precedes the actual firmware payload in the OTA file.

/// OTA upgrade file magic number (0x0BEEF11E).
pub const OTA_MAGIC: u32 = 0x0BEEF11E;

/// Minimum OTA header size (no optional fields).
pub const OTA_HEADER_MIN_SIZE: usize = 56;

/// OTA file header field control bits.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct OtaHeaderFieldControl {
    /// Security credential version present.
    pub security_credential: bool,
    /// Device-specific file (hardware version range present).
    pub device_specific: bool,
    /// Hardware version range present.
    pub hardware_versions: bool,
}

impl OtaHeaderFieldControl {
    pub fn from_u16(val: u16) -> Self {
        Self {
            security_credential: (val & 0x01) != 0,
            device_specific: (val & 0x02) != 0,
            hardware_versions: (val & 0x04) != 0,
        }
    }

    pub fn to_u16(self) -> u16 {
        let mut val = 0u16;
        if self.security_credential {
            val |= 0x01;
        }
        if self.device_specific {
            val |= 0x02;
        }
        if self.hardware_versions {
            val |= 0x04;
        }
        val
    }
}

/// Parsed OTA upgrade image header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OtaImageHeader {
    /// Magic number (must be 0x0BEEF11E).
    pub magic: u32,
    /// Header version (0x0100 for ZCL 7+).
    pub header_version: u16,
    /// Total header length in bytes.
    pub header_length: u16,
    /// Header field control.
    pub field_control: OtaHeaderFieldControl,
    /// Manufacturer code.
    pub manufacturer_code: u16,
    /// Image type (manufacturer-specific).
    pub image_type: u16,
    /// File version (new firmware version).
    pub file_version: u32,
    /// Zigbee stack version.
    pub stack_version: u16,
    /// Header string (up to 32 bytes, UTF-8).
    pub header_string: [u8; 32],
    /// Total image size (header + payload + optional elements).
    pub total_image_size: u32,
    // Optional fields
    /// Security credential version (if field_control bit 0 set).
    pub security_credential_version: Option<u8>,
    /// Minimum hardware version (if field_control bit 2 set).
    pub min_hardware_version: Option<u16>,
    /// Maximum hardware version (if field_control bit 2 set).
    pub max_hardware_version: Option<u16>,
}

/// OTA image sub-element tag IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum OtaTagId {
    /// Upgrade image (the actual firmware binary).
    UpgradeImage = 0x0000,
    /// ECDSA signing certificate.
    EcdsaCert = 0x0001,
    /// ECDSA signature.
    EcdsaSignature = 0x0002,
    /// Image integrity code (hash).
    ImageIntegrity = 0x0003,
    /// Picture data.
    PictureData = 0x0004,
    /// Unknown/vendor-specific tag.
    Unknown = 0xFFFF,
}

impl OtaTagId {
    pub fn from_u16(val: u16) -> Self {
        match val {
            0x0000 => Self::UpgradeImage,
            0x0001 => Self::EcdsaCert,
            0x0002 => Self::EcdsaSignature,
            0x0003 => Self::ImageIntegrity,
            0x0004 => Self::PictureData,
            _ => Self::Unknown,
        }
    }
}

/// A parsed sub-element header.
#[derive(Debug, Clone, Copy)]
pub struct OtaSubElement {
    /// Tag identifying the sub-element type.
    pub tag: OtaTagId,
    /// Length of the sub-element data.
    pub length: u32,
}

/// Errors from OTA image parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaImageError {
    /// Data too short for header.
    TooShort,
    /// Invalid magic number.
    BadMagic,
    /// Unsupported header version.
    UnsupportedVersion,
    /// Header length field doesn't match actual data.
    BadHeaderLength,
    /// Image size exceeds available space.
    ImageTooLarge,
}

impl OtaImageHeader {
    /// Parse an OTA image header from raw bytes.
    ///
    /// Returns the parsed header and the number of bytes consumed.
    pub fn parse(data: &[u8]) -> Result<(Self, usize), OtaImageError> {
        if data.len() < OTA_HEADER_MIN_SIZE {
            return Err(OtaImageError::TooShort);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        if magic != OTA_MAGIC {
            return Err(OtaImageError::BadMagic);
        }

        let header_version = u16::from_le_bytes([data[4], data[5]]);
        if header_version != 0x0100 {
            return Err(OtaImageError::UnsupportedVersion);
        }

        let header_length = u16::from_le_bytes([data[6], data[7]]);
        let field_control = OtaHeaderFieldControl::from_u16(u16::from_le_bytes([data[8], data[9]]));
        let manufacturer_code = u16::from_le_bytes([data[10], data[11]]);
        let image_type = u16::from_le_bytes([data[12], data[13]]);
        let file_version = u32::from_le_bytes([data[14], data[15], data[16], data[17]]);
        let stack_version = u16::from_le_bytes([data[18], data[19]]);

        let mut header_string = [0u8; 32];
        header_string.copy_from_slice(&data[20..52]);

        let total_image_size = u32::from_le_bytes([data[52], data[53], data[54], data[55]]);

        let mut offset = 56;

        let security_credential_version = if field_control.security_credential {
            if data.len() <= offset {
                return Err(OtaImageError::TooShort);
            }
            let v = data[offset];
            offset += 1;
            Some(v)
        } else {
            None
        };

        let (min_hardware_version, max_hardware_version) = if field_control.hardware_versions {
            if data.len() < offset + 4 {
                return Err(OtaImageError::TooShort);
            }
            let min_hw = u16::from_le_bytes([data[offset], data[offset + 1]]);
            let max_hw = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            offset += 4;
            (Some(min_hw), Some(max_hw))
        } else {
            (None, None)
        };

        if (header_length as usize) < offset {
            return Err(OtaImageError::BadHeaderLength);
        }

        Ok((
            Self {
                magic,
                header_version,
                header_length,
                field_control,
                manufacturer_code,
                image_type,
                file_version,
                stack_version,
                header_string,
                total_image_size,
                security_credential_version,
                min_hardware_version,
                max_hardware_version,
            },
            header_length as usize,
        ))
    }

    /// The size of the payload data (total - header).
    pub fn payload_size(&self) -> u32 {
        self.total_image_size
            .saturating_sub(self.header_length as u32)
    }

    /// Get the header string as a UTF-8 str (trimmed of null bytes).
    pub fn header_string_str(&self) -> &str {
        let end = self
            .header_string
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(32);
        core::str::from_utf8(&self.header_string[..end]).unwrap_or("")
    }
}

impl OtaSubElement {
    /// Parse a sub-element header from raw bytes.
    ///
    /// Returns the parsed sub-element and bytes consumed (6 = tag(2) + length(4)).
    pub fn parse(data: &[u8]) -> Result<(Self, usize), OtaImageError> {
        if data.len() < 6 {
            return Err(OtaImageError::TooShort);
        }
        let tag = OtaTagId::from_u16(u16::from_le_bytes([data[0], data[1]]));
        let length = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
        Ok((Self { tag, length }, 6))
    }
}
