//! Gecko bootloader integration: the `ApplicationProperties_t` record, the
//! ZRHP build record and the update path.
//!
//! The resident Gecko bootloader (`0x0800_0000..0x0800_6000`) stays the
//! NCP's firmware-update and recovery path: the host reboots the NCP into
//! it (`SYS_RESET` mode 1 → `efr32s2_hal::gecko_bootloader::
//! reboot_to_bootloader`, or on the Dongle-M the ESP32's GPIO12 + RESET
//! lines) and uploads a GBL over XMODEM at 115 200 baud. The GBL is built
//! from the ELF by `tools/efr32-gbl`, which also stores the address of
//! [`APPLICATION_PROPERTIES`] in word 13 of the vector table (as Simplicity
//! Commander does) so the bootloader finds it.
//!
//! Layout follows GSDK 4.5.0 `platform/bootloader/api/application_properties.h`
//! (struct version 1.2, 80 bytes). Signature type NONE: the stock Dongle-M
//! bootloader does not enforce signed images (its stock and community GBLs
//! carry no signature tags). A bootloader built with secure boot would
//! refuse this image; that is reported, not worked around.

/// `APPLICATION_PROPERTIES_MAGIC`.
pub const MAGIC: [u8; 16] = [
    0x13, 0xB7, 0x79, 0xFA, 0xC9, 0x25, 0xDD, 0xB7, 0xAD, 0xF3, 0xCF, 0xE0, 0xF1, 0xB6, 0x14, 0xB8,
];
/// `APPLICATION_PROPERTIES_VERSION`: minor 2 << 8 | major 1.
pub const STRUCT_VERSION: u32 = 0x0000_0201;
/// `APPLICATION_SIGNATURE_NONE`.
pub const SIGNATURE_NONE: u32 = 0;
/// `APPLICATION_TYPE_ZIGBEE`.
pub const APPLICATION_TYPE_ZIGBEE: u32 = 1 << 0;

/// zigbee-rs EFR32MG24 ZRHP NCP product ID (a fixed random UUID, so
/// tools can tell this firmware from EmberZNet images).
pub const PRODUCT_ID: [u8; 16] = [
    0x7A, 0x72, 0x73, 0x2D, 0x6E, 0x63, 0x70, 0x2D, 0x6D, 0x67, 0x32, 0x34, 0x9C, 0x1F, 0x4B, 0x07,
];

/// `ApplicationData_t`.
#[repr(C)]
pub struct ApplicationData {
    pub app_type: u32,
    pub version: u32,
    pub capabilities: u32,
    pub product_id: [u8; 16],
}

/// `ApplicationProperties_t` (v1.2).
#[repr(C)]
pub struct ApplicationProperties {
    pub magic: [u8; 16],
    pub struct_version: u32,
    pub signature_type: u32,
    pub signature_location: u32,
    pub app: ApplicationData,
    /// `ApplicationCertificate_t *` (none).
    pub cert: u32,
    /// `uint8_t *longTokenSectionAddress` (none).
    pub long_token_section_address: u32,
    pub decrypt_key: [u8; 16],
}

const _: () = assert!(core::mem::size_of::<ApplicationProperties>() == 80);

/// `MAJOR.MINOR.PATCH` of this crate as `0xMMmmpp00` (the EmberZNet
/// convention, e.g. 7.4.5.0 = `0x0704_0500`).
pub const APP_VERSION: u32 = parse_version(env!("CARGO_PKG_VERSION"));

const fn parse_version(text: &str) -> u32 {
    let bytes = text.as_bytes();
    let mut parts = [0u32; 3];
    let mut part = 0;
    let mut index = 0;
    while index < bytes.len() && part < 3 {
        let byte = bytes[index];
        if byte == b'.' {
            part += 1;
        } else if byte.is_ascii_digit() {
            parts[part] = parts[part] * 10 + (byte - b'0') as u32;
        } else {
            break;
        }
        index += 1;
    }
    assert!(parts[0] < 256 && parts[1] < 256 && parts[2] < 256);
    (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8)
}

/// The record the Gecko bootloader and Simplicity Commander look for. The
/// composition root must reference it (it is `#[used]`, but the linker
/// still drops an unreferenced section without `KEEP`).
#[used]
#[cfg_attr(
    target_os = "none",
    unsafe(link_section = ".rodata.gecko_app_properties")
)]
pub static APPLICATION_PROPERTIES: ApplicationProperties = ApplicationProperties {
    magic: MAGIC,
    struct_version: STRUCT_VERSION,
    signature_type: SIGNATURE_NONE,
    signature_location: 0,
    app: ApplicationData {
        app_type: APPLICATION_TYPE_ZIGBEE,
        version: APP_VERSION,
        capabilities: 0,
        product_id: PRODUCT_ID,
    },
    cert: 0,
    long_token_section_address: 0,
    decrypt_key: [0; 16],
};

/// Magic of the ZRHP build record ([`BuildInfo`]).
pub const BUILD_INFO_MAGIC: [u8; 8] = *b"ZRHPBID1";
/// Longest firmware version string the record holds.
pub const BUILD_INFO_VERSION_MAX: usize = 31;

/// ZRHP build record, placed in `.zrhp_build_info` by the NCP composition
/// root. `tools/efr32-gbl` fills `build_id` when it packages the image: the
/// first 8 bytes of the SHA-256 of the raw image with this field zeroed.
/// The NCP reports it in `SYS_NEGOTIATE.build_id`, and the GBL carries it
/// in its ZRHP metadata record, so a host can check after a Gecko update
/// that the intended build is running (spec 7.9, Gecko bootloader path).
/// An image flashed from the ELF (not packaged) reports all zeros.
#[repr(C)]
pub struct BuildInfo {
    pub magic: [u8; 8],
    pub build_id: [u8; 8],
    pub version_len: u8,
    pub version: [u8; BUILD_INFO_VERSION_MAX],
}

const _: () = assert!(core::mem::size_of::<BuildInfo>() == 48);

impl BuildInfo {
    /// Record for `version` (the composition root's package version).
    pub const fn new(version: &str) -> Self {
        let bytes = version.as_bytes();
        assert!(bytes.len() <= BUILD_INFO_VERSION_MAX);
        let mut out = [0u8; BUILD_INFO_VERSION_MAX];
        let mut index = 0;
        while index < bytes.len() {
            out[index] = bytes[index];
            index += 1;
        }
        Self {
            magic: BUILD_INFO_MAGIC,
            build_id: [0; 8],
            version_len: bytes.len() as u8,
            version: out,
        }
    }

    /// The build ID as stored in flash (read volatile: the packager patches
    /// it after linking, so the compiler must not use the zero initialiser).
    pub fn build_id(&'static self) -> [u8; 8] {
        // SAFETY: `self` is a `'static` record in flash.
        unsafe { core::ptr::read_volatile(&self.build_id) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_record_layout() {
        static INFO: BuildInfo = BuildInfo::new("0.1.0");
        assert_eq!(INFO.magic, BUILD_INFO_MAGIC);
        assert_eq!(INFO.version_len, 5);
        assert_eq!(&INFO.version[..5], b"0.1.0");
        assert_eq!(INFO.build_id(), [0; 8]);
        assert_eq!(core::mem::offset_of!(BuildInfo, build_id), 8);
        assert_eq!(core::mem::offset_of!(BuildInfo, version_len), 16);
    }

    #[test]
    fn version_encoding() {
        assert_eq!(parse_version("7.4.5"), 0x0704_0500);
        assert_eq!(parse_version("0.1.0"), 0x0001_0000);
        // No room for a pre-release suffix: `0.2.0-rc.1` runs as 0.2.0
        // (the release tooling compares the Gecko word with that).
        assert_eq!(parse_version("0.2.0-rc.1"), 0x0002_0000);
        assert_eq!(parse_version("1.2.3+build.4"), 0x0102_0300);
        assert_eq!(APP_VERSION, parse_version(env!("CARGO_PKG_VERSION")));
    }

    #[test]
    fn record_layout_matches_gsdk() {
        use core::mem::offset_of;
        assert_eq!(offset_of!(ApplicationProperties, struct_version), 16);
        assert_eq!(offset_of!(ApplicationProperties, app), 28);
        assert_eq!(offset_of!(ApplicationProperties, cert), 56);
        assert_eq!(offset_of!(ApplicationProperties, decrypt_key), 64);
        assert_eq!(APPLICATION_PROPERTIES.magic, MAGIC);
    }
}
