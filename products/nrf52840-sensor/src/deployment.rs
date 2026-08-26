//! Proven application and persistence maps for nRF52840 sensor deployments.
//!
//! The three UF2 contracts match the Adafruit nRF52 Bootloader nRF52840
//! linker map: bootloader at `0xF4000`, bootloader/config below `0xFE000`,
//! MBR parameters at `0xFE000`, and settings at `0xFF000`. ProMicro also
//! retains its existing S140 application origin and conservative
//! `0xF0000..0xF4000` guard. The DK contract has no resident bootloader.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeploymentLayout {
    pub name: &'static str,
    pub protected_low_end: u32,
    pub application_start: u32,
    pub application_end: u32,
    pub security_start: u32,
    pub security_end: u32,
    pub bootloader_start: u32,
    pub boot_state_start: u32,
    pub flash_end: u32,
    pub ram_start: u32,
    pub ram_end: u32,
}

impl DeploymentLayout {
    pub const fn is_valid(self) -> bool {
        self.protected_low_end == self.application_start
            && self.application_start < self.application_end
            && self.application_end == self.security_start
            && self.security_end - self.security_start == 8 * 1024
            && self.security_end <= self.bootloader_start
            && self.bootloader_start <= self.boot_state_start
            && self.boot_state_start <= self.flash_end
            && self.flash_end == 0x0010_0000
            && self.ram_start < self.ram_end
            && self.ram_end == 0x2004_0000
    }
}

pub const PROMICRO_UF2: DeploymentLayout = DeploymentLayout {
    name: "promicro-s140-uf2",
    protected_low_end: 0x0002_6000,
    application_start: 0x0002_6000,
    application_end: 0x000E_E000,
    security_start: 0x000E_E000,
    security_end: 0x000F_0000,
    bootloader_start: 0x000F_4000,
    boot_state_start: 0x000F_E000,
    flash_end: 0x0010_0000,
    ram_start: 0x2000_2000,
    ram_end: 0x2004_0000,
};

pub const MDK_UF2: DeploymentLayout = DeploymentLayout {
    name: "mdk-uf2",
    protected_low_end: 0x0000_1000,
    application_start: 0x0000_1000,
    application_end: 0x000F_2000,
    security_start: 0x000F_2000,
    security_end: 0x000F_4000,
    bootloader_start: 0x000F_4000,
    boot_state_start: 0x000F_E000,
    flash_end: 0x0010_0000,
    ram_start: 0x2000_0000,
    ram_end: 0x2004_0000,
};

pub const PCA10059_UF2: DeploymentLayout = DeploymentLayout {
    name: "pca10059-uf2",
    ..MDK_UF2
};

pub const DK: DeploymentLayout = DeploymentLayout {
    name: "nrf52840-dk",
    protected_low_end: 0x0000_0000,
    application_start: 0x0000_0000,
    application_end: 0x000F_E000,
    security_start: 0x000F_E000,
    security_end: 0x0010_0000,
    bootloader_start: 0x0010_0000,
    boot_state_start: 0x0010_0000,
    flash_end: 0x0010_0000,
    ram_start: 0x2000_0000,
    ram_end: 0x2004_0000,
};

#[cfg(any(
    all(feature = "deployment-promicro-uf2", feature = "deployment-mdk-uf2"),
    all(
        feature = "deployment-promicro-uf2",
        feature = "deployment-pca10059-uf2"
    ),
    all(feature = "deployment-promicro-uf2", feature = "deployment-dk"),
    all(feature = "deployment-mdk-uf2", feature = "deployment-pca10059-uf2"),
    all(feature = "deployment-mdk-uf2", feature = "deployment-dk"),
    all(feature = "deployment-pca10059-uf2", feature = "deployment-dk")
))]
compile_error!("select exactly one nRF52840 sensor deployment feature");

#[cfg(feature = "deployment-promicro-uf2")]
pub const SELECTED: DeploymentLayout = PROMICRO_UF2;
#[cfg(feature = "deployment-mdk-uf2")]
pub const SELECTED: DeploymentLayout = MDK_UF2;
#[cfg(feature = "deployment-pca10059-uf2")]
pub const SELECTED: DeploymentLayout = PCA10059_UF2;
#[cfg(feature = "deployment-dk")]
pub const SELECTED: DeploymentLayout = DK;

pub const UF2_MODEL: &str = "nRF52840-UF2-Sensor";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_deployment_has_a_disjoint_two_page_journal() {
        for layout in [PROMICRO_UF2, MDK_UF2, PCA10059_UF2, DK] {
            assert!(layout.is_valid(), "invalid deployment: {}", layout.name);
        }
    }

    #[test]
    fn uf2_bootloader_and_state_ranges_are_never_application_owned() {
        for layout in [PROMICRO_UF2, MDK_UF2, PCA10059_UF2] {
            assert!(layout.security_end <= 0x000F_4000);
            assert_eq!(layout.bootloader_start, 0x000F_4000);
            assert_eq!(layout.boot_state_start, 0x000F_E000);
            assert_eq!(layout.flash_end, 0x0010_0000);
        }
    }

    #[test]
    fn application_origins_preserve_existing_uf2_contracts() {
        assert_eq!(PROMICRO_UF2.application_start, 0x0002_6000);
        assert_eq!(MDK_UF2.application_start, 0x0000_1000);
        assert_eq!(PCA10059_UF2.application_start, 0x0000_1000);
        assert_eq!(DK.application_start, 0);
    }
}
