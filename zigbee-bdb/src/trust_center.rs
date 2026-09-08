//! Centralized Trust Center admission and link-key policy.

use crate::attributes::{
    ApplicationLinkKeyRequestPolicy, BdbAttributes, TrustCenterInstallCodePolicy,
    TrustCenterLinkKeyRequestPolicy,
};
use zigbee_aps::apsme::{
    ApsRequestKeyType, ApsUpdateDeviceStatus, ApsmeRequestKeyIndication,
    ApsmeUpdateDeviceIndication, ApsmeVerifyKeyIndication,
};
use zigbee_aps::security::{AesKey, DEFAULT_TC_LINK_KEY, derive_verify_key_hash};
use zigbee_crypto::{InstallCodeError, derive_install_code_key};
use zigbee_types::{IeeeAddress, ShortAddress};

pub const MAX_TRUST_CENTER_DEVICES: usize = 32;
const _: () = assert!(MAX_TRUST_CENTER_DEVICES <= u32::BITS as usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustCenterKeyOrigin {
    DefaultGlobal,
    InstallCode,
    ApplicationProvisioned,
    GeneratedUnique,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TrustCenterKeyAttributes {
    Provisional = 0x00,
    Unverified = 0x01,
    Verified = 0x02,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrustCenterDevice {
    pub ieee_address: IeeeAddress,
    pub parent_address: IeeeAddress,
    pub short_address: ShortAddress,
    pub link_key: AesKey,
    pub key_origin: TrustCenterKeyOrigin,
    pub key_attributes: TrustCenterKeyAttributes,
    pub join_timeout_remaining_secs: u8,
    pub active: bool,
}

impl TrustCenterDevice {
    fn provisioned(
        ieee_address: IeeeAddress,
        link_key: AesKey,
        key_origin: TrustCenterKeyOrigin,
    ) -> Self {
        Self {
            ieee_address,
            parent_address: [0u8; 8],
            short_address: ShortAddress(0xFFFF),
            link_key,
            key_origin,
            key_attributes: TrustCenterKeyAttributes::Provisional,
            join_timeout_remaining_secs: 0,
            active: true,
        }
    }

    fn has_verified_replacement_key(&self) -> bool {
        matches!(
            (self.key_origin, self.key_attributes),
            (
                TrustCenterKeyOrigin::GeneratedUnique,
                TrustCenterKeyAttributes::Verified
            )
        )
    }

    fn has_completed_admission(&self) -> bool {
        self.join_timeout_remaining_secs == 0
            && self.parent_address != [0u8; 8]
            && self.parent_address != [0xFFu8; 8]
            && self.short_address.0 <= 0xFFF7
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrustCenterPolicy {
    pub allow_joins: bool,
    pub use_whitelist: bool,
    pub install_codes: TrustCenterInstallCodePolicy,
    pub require_link_key_update: bool,
    pub allow_rejoins: bool,
    pub link_key_requests: TrustCenterLinkKeyRequestPolicy,
    pub application_key_requests: ApplicationLinkKeyRequestPolicy,
    pub join_timeout_secs: u8,
}

impl TrustCenterPolicy {
    pub const fn from_attributes(attributes: &BdbAttributes) -> Self {
        Self {
            allow_joins: attributes.trust_center_allow_joins,
            use_whitelist: attributes.trust_center_use_whitelist,
            install_codes: attributes.trust_center_install_code_policy,
            require_link_key_update: attributes.trust_center_require_key_exchange,
            allow_rejoins: attributes.trust_center_allow_rejoins,
            link_key_requests: attributes.trust_center_link_key_request_policy,
            application_key_requests: attributes.trust_center_application_key_request_policy,
            join_timeout_secs: attributes.trust_center_node_join_timeout,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustCenterAction {
    None,
    TransportNetworkKey {
        parent_address: IeeeAddress,
        device_address: IeeeAddress,
        device_short_address: ShortAddress,
        link_key: AesKey,
        require_link_key_update: bool,
    },
    GenerateTrustCenterLinkKey {
        device_address: IeeeAddress,
    },
    GenerateApplicationLinkKey {
        initiator_address: IeeeAddress,
        responder_address: IeeeAddress,
    },
    ConfirmKey {
        device_address: IeeeAddress,
        status: u8,
    },
    RemoveDevice {
        parent_address: IeeeAddress,
        device_address: IeeeAddress,
        device_short_address: ShortAddress,
    },
    RevokeDevice {
        device_address: IeeeAddress,
    },
    RejectedWithoutCommand {
        device_address: IeeeAddress,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustCenterError {
    InvalidAddress,
    InvalidInstallCode(InstallCodeError),
    DeviceTableFull,
    DeviceNotFound,
    RequestNotPermitted,
}

pub struct TrustCenterTable {
    /// Committed admission/current keys. A generated replacement stays out of
    /// this table until Verify-Key proves that the joining device received it.
    devices: [Option<TrustCenterDevice>; MAX_TRUST_CENTER_DEVICES],
    /// Replacement keys awaiting Verify-Key, indexed with `devices`.
    pending_link_keys: [AesKey; MAX_TRUST_CENTER_DEVICES],
    pending_link_key_mask: u32,
}

impl TrustCenterTable {
    pub fn new() -> Self {
        Self {
            devices: core::array::from_fn(|_| None),
            pending_link_keys: [[0u8; 16]; MAX_TRUST_CENTER_DEVICES],
            pending_link_key_mask: 0,
        }
    }

    pub fn devices(&self) -> impl Iterator<Item = &TrustCenterDevice> {
        self.devices.iter().flatten().filter(|device| device.active)
    }

    pub fn device(&self, address: &IeeeAddress) -> Option<&TrustCenterDevice> {
        self.devices()
            .find(|device| device.ieee_address == *address)
    }

    pub fn device_mut(&mut self, address: &IeeeAddress) -> Option<&mut TrustCenterDevice> {
        self.devices
            .iter_mut()
            .flatten()
            .find(|device| device.active && device.ieee_address == *address)
    }

    fn device_index(&self, address: &IeeeAddress) -> Option<usize> {
        self.devices.iter().position(|slot| {
            slot.is_some_and(|device| device.active && device.ieee_address == *address)
        })
    }

    pub fn restore_device(&mut self, device: TrustCenterDevice) -> Result<(), TrustCenterError> {
        validate_device_address(device.ieee_address)?;
        if let Some(index) = self.device_index(&device.ieee_address) {
            self.devices[index] = Some(device);
            self.clear_pending_link_key(index);
            return Ok(());
        }
        let index = self
            .devices
            .iter()
            .position(|slot| slot.is_none() || slot.is_some_and(|entry| !entry.active))
            .ok_or(TrustCenterError::DeviceTableFull)?;
        self.devices[index] = Some(device);
        self.clear_pending_link_key(index);
        Ok(())
    }

    pub fn clear(&mut self) {
        self.devices.fill(None);
        self.pending_link_keys.fill([0u8; 16]);
        self.pending_link_key_mask = 0;
    }

    pub fn provision_install_code(
        &mut self,
        address: IeeeAddress,
        install_code: &[u8],
    ) -> Result<AesKey, TrustCenterError> {
        if install_code.len() != 18 {
            return Err(TrustCenterError::InvalidInstallCode(
                InstallCodeError::InvalidLength,
            ));
        }
        let key =
            derive_install_code_key(install_code).map_err(TrustCenterError::InvalidInstallCode)?;
        self.provision_link_key(address, key, TrustCenterKeyOrigin::InstallCode)?;
        Ok(key)
    }

    pub fn provision_link_key(
        &mut self,
        address: IeeeAddress,
        key: AesKey,
        origin: TrustCenterKeyOrigin,
    ) -> Result<(), TrustCenterError> {
        validate_device_address(address)?;
        if let Some(index) = self.device_index(&address) {
            let device = self.devices[index]
                .as_mut()
                .ok_or(TrustCenterError::DeviceNotFound)?;
            device.link_key = key;
            device.key_origin = origin;
            device.key_attributes = TrustCenterKeyAttributes::Provisional;
            device.join_timeout_remaining_secs = 0;
            self.clear_pending_link_key(index);
            return Ok(());
        }
        let index = self
            .devices
            .iter()
            .position(|slot| slot.is_none() || slot.is_some_and(|device| !device.active))
            .ok_or(TrustCenterError::DeviceTableFull)?;
        self.devices[index] = Some(TrustCenterDevice::provisioned(address, key, origin));
        self.clear_pending_link_key(index);
        Ok(())
    }

    pub fn handle_update_device(
        &mut self,
        policy: TrustCenterPolicy,
        indication: ApsmeUpdateDeviceIndication,
    ) -> Result<TrustCenterAction, TrustCenterError> {
        validate_device_address(indication.device_address)?;
        validate_device_address(indication.source_address)?;
        match indication.status {
            ApsUpdateDeviceStatus::DeviceLeft => Ok(TrustCenterAction::RevokeDevice {
                device_address: indication.device_address,
            }),
            ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin => {
                let admission_key_allowed =
                    self.install_code_policy_allows_admission(policy, &indication.device_address);
                let accepted = admission_key_allowed
                    && (self
                        .device(&indication.device_address)
                        .is_some_and(TrustCenterDevice::has_completed_admission)
                        || policy.allow_rejoins);
                if !accepted {
                    return Ok(TrustCenterAction::RemoveDevice {
                        parent_address: indication.source_address,
                        device_address: indication.device_address,
                        device_short_address: indication.device_short_address,
                    });
                }
                let index = self.ensure_default_device_index(indication.device_address)?;
                let device = self.devices[index]
                    .as_mut()
                    .ok_or(TrustCenterError::DeviceNotFound)?;
                update_location(device, indication, 0);
                Ok(TrustCenterAction::None)
            }
            ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin => {
                if !policy.allow_joins
                    || (policy.use_whitelist && self.device(&indication.device_address).is_none())
                    || !self
                        .install_code_policy_allows_admission(policy, &indication.device_address)
                {
                    return Ok(TrustCenterAction::RejectedWithoutCommand {
                        device_address: indication.device_address,
                    });
                }
                self.admit_with_network_key(policy, indication)
            }
            ApsUpdateDeviceStatus::StandardDeviceTrustCenterRejoin => {
                let known_admitted = self
                    .device(&indication.device_address)
                    .is_some_and(TrustCenterDevice::has_completed_admission);
                if !self.install_code_policy_allows_admission(policy, &indication.device_address)
                    || (!known_admitted && !policy.allow_rejoins)
                {
                    return Ok(TrustCenterAction::RejectedWithoutCommand {
                        device_address: indication.device_address,
                    });
                }
                self.admit_with_network_key(policy, indication)
            }
        }
    }

    fn admit_with_network_key(
        &mut self,
        policy: TrustCenterPolicy,
        indication: ApsmeUpdateDeviceIndication,
    ) -> Result<TrustCenterAction, TrustCenterError> {
        let index = self.ensure_default_device_index(indication.device_address)?;
        let replacement_pending = self.has_pending_link_key(index);
        let device = self.devices[index]
            .as_mut()
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let require_link_key_update = replacement_pending
            || (policy.require_link_key_update && !device.has_verified_replacement_key());
        let timeout = if require_link_key_update {
            policy.join_timeout_secs.max(1)
        } else {
            0
        };
        update_location(device, indication, timeout);
        Ok(TrustCenterAction::TransportNetworkKey {
            parent_address: indication.source_address,
            device_address: indication.device_address,
            device_short_address: indication.device_short_address,
            link_key: device.link_key,
            require_link_key_update,
        })
    }

    pub fn handle_request_key(
        &self,
        policy: TrustCenterPolicy,
        indication: ApsmeRequestKeyIndication,
        trust_center_address: IeeeAddress,
    ) -> Result<TrustCenterAction, TrustCenterError> {
        let device = self
            .device(&indication.source_address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        match indication.key_type {
            ApsRequestKeyType::TrustCenterLink => {
                let allowed = match policy.link_key_requests {
                    TrustCenterLinkKeyRequestPolicy::Never => false,
                    TrustCenterLinkKeyRequestPolicy::AnyDevice => true,
                    TrustCenterLinkKeyRequestPolicy::ProvisionalOnly => {
                        device.key_attributes == TrustCenterKeyAttributes::Provisional
                    }
                };
                if !allowed {
                    return Err(TrustCenterError::RequestNotPermitted);
                }
                Ok(TrustCenterAction::GenerateTrustCenterLinkKey {
                    device_address: indication.source_address,
                })
            }
            ApsRequestKeyType::ApplicationLink => {
                let responder = indication
                    .partner_address
                    .ok_or(TrustCenterError::InvalidAddress)?;
                let policy_allows = match policy.application_key_requests {
                    ApplicationLinkKeyRequestPolicy::Never
                    | ApplicationLinkKeyRequestPolicy::AllowListOnly => false,
                    ApplicationLinkKeyRequestPolicy::AnyPair => true,
                };
                if responder == trust_center_address
                    || self.device(&responder).is_none()
                    || !policy_allows
                {
                    return Err(TrustCenterError::RequestNotPermitted);
                }
                Ok(TrustCenterAction::GenerateApplicationLinkKey {
                    initiator_address: indication.source_address,
                    responder_address: responder,
                })
            }
        }
    }

    pub fn install_generated_trust_center_link_key(
        &mut self,
        address: IeeeAddress,
        key: AesKey,
        join_timeout_secs: u8,
    ) -> Result<(), TrustCenterError> {
        let index = self
            .device_index(&address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        self.set_pending_link_key(index, key);
        let device = self.devices[index]
            .as_mut()
            .ok_or(TrustCenterError::DeviceNotFound)?;
        device.join_timeout_remaining_secs = join_timeout_secs.max(1);
        Ok(())
    }

    pub fn handle_verify_key(
        &mut self,
        indication: ApsmeVerifyKeyIndication,
    ) -> Result<TrustCenterAction, TrustCenterError> {
        let index = self
            .device_index(&indication.source_address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let (key, commits_replacement) = if let Some(pending) = self.pending_link_key(index) {
            (pending, true)
        } else {
            let device = self.devices[index]
                .as_ref()
                .ok_or(TrustCenterError::DeviceNotFound)?;
            if !device.has_verified_replacement_key() {
                return Err(TrustCenterError::RequestNotPermitted);
            }
            (device.link_key, false)
        };
        let expected = derive_verify_key_hash(&key);
        let success = constant_time_eq(&expected, &indication.hash);
        if success && commits_replacement {
            let device = self.devices[index]
                .as_mut()
                .ok_or(TrustCenterError::DeviceNotFound)?;
            device.link_key = key;
            device.key_origin = TrustCenterKeyOrigin::GeneratedUnique;
            device.key_attributes = TrustCenterKeyAttributes::Verified;
            device.join_timeout_remaining_secs = 0;
            self.clear_pending_link_key(index);
        }
        Ok(TrustCenterAction::ConfirmKey {
            device_address: indication.source_address,
            status: if success { 0x00 } else { 0xAD },
        })
    }

    pub fn tick(&mut self, elapsed_secs: u8) -> TrustCenterAction {
        if elapsed_secs == 0 {
            return TrustCenterAction::None;
        }
        for (index, slot) in self.devices.iter_mut().enumerate() {
            let Some(device) = slot.as_mut().filter(|device| device.active) else {
                continue;
            };
            let initial_exchange_pending = device.join_timeout_remaining_secs != 0
                && device.key_attributes == TrustCenterKeyAttributes::Provisional;
            let replacement_pending = self.pending_link_key_mask & (1u32 << index) != 0;
            if !initial_exchange_pending && !replacement_pending {
                continue;
            }
            device.join_timeout_remaining_secs = device
                .join_timeout_remaining_secs
                .saturating_sub(elapsed_secs);
            if device.join_timeout_remaining_secs == 0 {
                let action = TrustCenterAction::RemoveDevice {
                    parent_address: device.parent_address,
                    device_address: device.ieee_address,
                    device_short_address: device.short_address,
                };
                device.active = false;
                self.pending_link_keys[index] = [0u8; 16];
                self.pending_link_key_mask &= !(1u32 << index);
                return action;
            }
        }
        TrustCenterAction::None
    }

    pub fn revoke(&mut self, address: &IeeeAddress) -> bool {
        if let Some(index) = self.device_index(address) {
            let Some(device) = self.devices[index].as_mut() else {
                return false;
            };
            device.active = false;
            device.join_timeout_remaining_secs = 0;
            self.clear_pending_link_key(index);
            return true;
        }
        false
    }

    fn ensure_default_device_index(
        &mut self,
        address: IeeeAddress,
    ) -> Result<usize, TrustCenterError> {
        if self.device_index(&address).is_none() {
            self.provision_link_key(
                address,
                DEFAULT_TC_LINK_KEY,
                TrustCenterKeyOrigin::DefaultGlobal,
            )?;
        }
        self.device_index(&address)
            .ok_or(TrustCenterError::DeviceTableFull)
    }

    fn install_code_policy_allows_admission(
        &self,
        policy: TrustCenterPolicy,
        address: &IeeeAddress,
    ) -> bool {
        policy.install_codes != TrustCenterInstallCodePolicy::Required
            || self.device(address).is_some_and(|device| {
                device.key_origin == TrustCenterKeyOrigin::InstallCode
                    || device.has_verified_replacement_key()
            })
    }

    fn pending_link_key(&self, index: usize) -> Option<AesKey> {
        self.has_pending_link_key(index)
            .then_some(self.pending_link_keys[index])
    }

    fn has_pending_link_key(&self, index: usize) -> bool {
        self.pending_link_key_mask & (1u32 << index) != 0
    }

    fn set_pending_link_key(&mut self, index: usize, key: AesKey) {
        self.pending_link_keys[index] = key;
        self.pending_link_key_mask |= 1u32 << index;
    }

    fn clear_pending_link_key(&mut self, index: usize) {
        self.pending_link_keys[index] = [0u8; 16];
        self.pending_link_key_mask &= !(1u32 << index);
    }
}

impl Default for TrustCenterTable {
    fn default() -> Self {
        Self::new()
    }
}

fn update_location(
    device: &mut TrustCenterDevice,
    indication: ApsmeUpdateDeviceIndication,
    timeout: u8,
) {
    device.parent_address = indication.source_address;
    device.short_address = indication.device_short_address;
    device.join_timeout_remaining_secs = timeout;
    device.active = true;
}

fn validate_device_address(address: IeeeAddress) -> Result<(), TrustCenterError> {
    if address == [0u8; 8] || address == [0xFFu8; 8] {
        Err(TrustCenterError::InvalidAddress)
    } else {
        Ok(())
    }
}

fn constant_time_eq(left: &[u8; 16], right: &[u8; 16]) -> bool {
    left.iter()
        .zip(right)
        .fold(0u8, |difference, (left, right)| difference | (left ^ right))
        == 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_aps::apsme::{
        ApsmeRequestKeyIndication, ApsmeUpdateDeviceIndication, ApsmeVerifyKeyIndication,
    };

    const TC: IeeeAddress = [0x10; 8];
    const PARENT: IeeeAddress = [0x20; 8];
    const DEVICE: IeeeAddress = [0x30; 8];
    const PARTNER: IeeeAddress = [0x40; 8];
    const INSTALL_CODE: [u8; 18] = [
        0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0xD4, 0x90,
    ];
    const INSTALL_CODE_KEY: AesKey = [
        0xFA, 0x80, 0x81, 0xCA, 0xAA, 0x41, 0xD5, 0xAD, 0xE9, 0xB5, 0x65, 0x87, 0x99, 0x26, 0x8B,
        0x88,
    ];

    fn policy() -> TrustCenterPolicy {
        TrustCenterPolicy::from_attributes(&BdbAttributes::default())
    }

    fn update(status: ApsUpdateDeviceStatus) -> ApsmeUpdateDeviceIndication {
        ApsmeUpdateDeviceIndication {
            source_address: PARENT,
            device_address: DEVICE,
            device_short_address: ShortAddress(0x1234),
            status,
        }
    }

    #[test]
    fn default_join_requires_unique_key_exchange_and_verification() {
        let mut table = TrustCenterTable::new();
        let action = table
            .handle_update_device(
                policy(),
                update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin),
            )
            .unwrap();
        assert!(matches!(
            action,
            TrustCenterAction::TransportNetworkKey {
                link_key: DEFAULT_TC_LINK_KEY,
                require_link_key_update: true,
                ..
            }
        ));
        assert_eq!(
            table.device(&DEVICE).unwrap().key_attributes,
            TrustCenterKeyAttributes::Provisional
        );

        assert_eq!(
            table
                .handle_request_key(
                    policy(),
                    ApsmeRequestKeyIndication {
                        source_address: DEVICE,
                        key_type: ApsRequestKeyType::TrustCenterLink,
                        partner_address: None,
                    },
                    TC,
                )
                .unwrap(),
            TrustCenterAction::GenerateTrustCenterLinkKey {
                device_address: DEVICE
            }
        );

        let unique = [0xA5; 16];
        table
            .install_generated_trust_center_link_key(DEVICE, unique, 15)
            .unwrap();
        let bad = table
            .handle_verify_key(ApsmeVerifyKeyIndication {
                source_address: DEVICE,
                key_type: 0x04,
                hash: [0u8; 16],
            })
            .unwrap();
        assert_eq!(
            bad,
            TrustCenterAction::ConfirmKey {
                device_address: DEVICE,
                status: 0xAD
            }
        );
        assert_eq!(
            table.device(&DEVICE),
            Some(&TrustCenterDevice {
                ieee_address: DEVICE,
                parent_address: PARENT,
                short_address: ShortAddress(0x1234),
                link_key: DEFAULT_TC_LINK_KEY,
                key_origin: TrustCenterKeyOrigin::DefaultGlobal,
                key_attributes: TrustCenterKeyAttributes::Provisional,
                join_timeout_remaining_secs: 15,
                active: true,
            })
        );

        let good = table
            .handle_verify_key(ApsmeVerifyKeyIndication {
                source_address: DEVICE,
                key_type: 0x04,
                hash: derive_verify_key_hash(&unique),
            })
            .unwrap();
        assert_eq!(
            good,
            TrustCenterAction::ConfirmKey {
                device_address: DEVICE,
                status: 0x00
            }
        );
        assert_eq!(
            table.device(&DEVICE).unwrap().key_attributes,
            TrustCenterKeyAttributes::Verified
        );
        assert_eq!(table.device(&DEVICE).unwrap().link_key, unique);
        assert_eq!(
            table.device(&DEVICE).unwrap().key_origin,
            TrustCenterKeyOrigin::GeneratedUnique
        );
    }

    #[test]
    fn install_code_only_policy_never_falls_back_to_the_global_key() {
        let mut table = TrustCenterTable::new();
        let mut required = policy();
        required.install_codes = TrustCenterInstallCodePolicy::Required;
        required.use_whitelist = false;
        assert_eq!(
            table
                .handle_update_device(
                    required,
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::RejectedWithoutCommand {
                device_address: DEVICE
            }
        );
        assert!(table.device(&DEVICE).is_none());

        required.allow_rejoins = true;
        assert_eq!(
            table
                .handle_update_device(
                    required,
                    update(ApsUpdateDeviceStatus::StandardDeviceTrustCenterRejoin)
                )
                .unwrap(),
            TrustCenterAction::RejectedWithoutCommand {
                device_address: DEVICE
            }
        );
        assert!(table.device(&DEVICE).is_none());
        assert_eq!(
            table
                .handle_update_device(
                    required,
                    update(ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin)
                )
                .unwrap(),
            TrustCenterAction::RemoveDevice {
                parent_address: PARENT,
                device_address: DEVICE,
                device_short_address: ShortAddress(0x1234),
            }
        );
        assert!(table.device(&DEVICE).is_none());

        let key = table.provision_install_code(DEVICE, &INSTALL_CODE).unwrap();
        assert_eq!(key, INSTALL_CODE_KEY);
        assert!(matches!(
            table
                .handle_update_device(
                    required,
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::TransportNetworkKey { link_key, .. } if link_key == key
        ));
    }

    #[test]
    fn idle_preprovisioned_install_code_survives_ticks() {
        let mut table = TrustCenterTable::new();
        table.provision_install_code(DEVICE, &INSTALL_CODE).unwrap();

        assert_eq!(table.tick(1), TrustCenterAction::None);
        assert_eq!(table.tick(u8::MAX), TrustCenterAction::None);
        let device = table.device(&DEVICE).unwrap();
        assert_eq!(device.link_key, INSTALL_CODE_KEY);
        assert_eq!(device.key_origin, TrustCenterKeyOrigin::InstallCode);
        assert_eq!(device.key_attributes, TrustCenterKeyAttributes::Provisional);
        assert_eq!(device.join_timeout_remaining_secs, 0);
    }

    #[test]
    fn initial_install_code_key_is_not_a_completed_replacement_exchange() {
        let mut table = TrustCenterTable::new();
        table.provision_install_code(DEVICE, &INSTALL_CODE).unwrap();
        let mut no_replacement = policy();
        no_replacement.require_link_key_update = false;

        assert!(matches!(
            table
                .handle_update_device(
                    no_replacement,
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::TransportNetworkKey {
                link_key: INSTALL_CODE_KEY,
                require_link_key_update: false,
                ..
            }
        ));
        assert_eq!(
            table.handle_verify_key(ApsmeVerifyKeyIndication {
                source_address: DEVICE,
                key_type: 0x04,
                hash: derive_verify_key_hash(&INSTALL_CODE_KEY),
            }),
            Err(TrustCenterError::RequestNotPermitted)
        );

        let initial = table.device(&DEVICE).unwrap();
        assert_eq!(initial.link_key, INSTALL_CODE_KEY);
        assert_eq!(initial.key_origin, TrustCenterKeyOrigin::InstallCode);
        assert_eq!(
            initial.key_attributes,
            TrustCenterKeyAttributes::Provisional
        );

        assert!(matches!(
            table
                .handle_update_device(
                    policy(),
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::TransportNetworkKey {
                link_key: INSTALL_CODE_KEY,
                require_link_key_update: true,
                ..
            }
        ));

        let replacement = [0xA6; 16];
        table
            .install_generated_trust_center_link_key(DEVICE, replacement, 15)
            .unwrap();
        let pending = table.device(&DEVICE).unwrap();
        assert_eq!(pending.link_key, INSTALL_CODE_KEY);
        assert_eq!(pending.key_origin, TrustCenterKeyOrigin::InstallCode);
        assert_eq!(
            pending.key_attributes,
            TrustCenterKeyAttributes::Provisional
        );

        assert_eq!(
            table
                .handle_verify_key(ApsmeVerifyKeyIndication {
                    source_address: DEVICE,
                    key_type: 0x04,
                    hash: derive_verify_key_hash(&replacement),
                })
                .unwrap(),
            TrustCenterAction::ConfirmKey {
                device_address: DEVICE,
                status: 0x00,
            }
        );
        let verified = table.device(&DEVICE).unwrap();
        assert_eq!(verified.link_key, replacement);
        assert_eq!(verified.key_origin, TrustCenterKeyOrigin::GeneratedUnique);
        assert_eq!(verified.key_attributes, TrustCenterKeyAttributes::Verified);
    }

    #[test]
    fn initial_global_key_is_not_a_completed_replacement_exchange() {
        let mut table = TrustCenterTable::new();
        let mut no_replacement = policy();
        no_replacement.require_link_key_update = false;

        assert!(matches!(
            table
                .handle_update_device(
                    no_replacement,
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::TransportNetworkKey {
                link_key: DEFAULT_TC_LINK_KEY,
                require_link_key_update: false,
                ..
            }
        ));
        let initial = table.device(&DEVICE).unwrap();
        assert_eq!(initial.key_origin, TrustCenterKeyOrigin::DefaultGlobal);
        assert_eq!(
            initial.key_attributes,
            TrustCenterKeyAttributes::Provisional
        );
        assert_eq!(
            table.handle_verify_key(ApsmeVerifyKeyIndication {
                source_address: DEVICE,
                key_type: 0x04,
                hash: derive_verify_key_hash(&DEFAULT_TC_LINK_KEY),
            }),
            Err(TrustCenterError::RequestNotPermitted)
        );
        assert!(matches!(
            table
                .handle_update_device(
                    policy(),
                    update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin)
                )
                .unwrap(),
            TrustCenterAction::TransportNetworkKey {
                link_key: DEFAULT_TC_LINK_KEY,
                require_link_key_update: true,
                ..
            }
        ));
    }

    #[test]
    fn secured_rejoin_is_removed_without_verified_state() {
        let mut table = TrustCenterTable::new();
        assert_eq!(
            table
                .handle_update_device(
                    policy(),
                    update(ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin)
                )
                .unwrap(),
            TrustCenterAction::RemoveDevice {
                parent_address: PARENT,
                device_address: DEVICE,
                device_short_address: ShortAddress(0x1234),
            }
        );
    }

    #[test]
    fn admission_timeout_removes_and_revokes_the_device() {
        let mut table = TrustCenterTable::new();
        let mut policy = policy();
        policy.join_timeout_secs = 2;
        table
            .handle_update_device(
                policy,
                update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin),
            )
            .unwrap();
        assert_eq!(table.tick(1), TrustCenterAction::None);
        assert!(matches!(
            table.tick(1),
            TrustCenterAction::RemoveDevice {
                device_address: DEVICE,
                ..
            }
        ));
        assert!(table.device(&DEVICE).is_none());
    }

    #[test]
    fn pending_replacement_key_uses_the_join_timeout() {
        let mut table = TrustCenterTable::new();
        table
            .handle_update_device(
                policy(),
                update(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin),
            )
            .unwrap();
        table
            .install_generated_trust_center_link_key(DEVICE, [0xA7; 16], 2)
            .unwrap();

        assert_eq!(table.tick(1), TrustCenterAction::None);
        assert!(matches!(
            table.tick(1),
            TrustCenterAction::RemoveDevice {
                device_address: DEVICE,
                ..
            }
        ));
        assert!(table.device(&DEVICE).is_none());
    }

    #[test]
    fn application_key_request_requires_a_known_non_tc_partner() {
        let mut table = TrustCenterTable::new();
        table
            .provision_link_key(
                DEVICE,
                [0x51; 16],
                TrustCenterKeyOrigin::ApplicationProvisioned,
            )
            .unwrap();
        table
            .provision_link_key(
                PARTNER,
                [0x52; 16],
                TrustCenterKeyOrigin::ApplicationProvisioned,
            )
            .unwrap();
        assert_eq!(
            table
                .handle_request_key(
                    policy(),
                    ApsmeRequestKeyIndication {
                        source_address: DEVICE,
                        key_type: ApsRequestKeyType::ApplicationLink,
                        partner_address: Some(PARTNER),
                    },
                    TC,
                )
                .unwrap(),
            TrustCenterAction::GenerateApplicationLinkKey {
                initiator_address: DEVICE,
                responder_address: PARTNER,
            }
        );
    }

    #[test]
    fn allowlist_policy_fails_closed_without_external_authorization() {
        let mut table = TrustCenterTable::new();
        for (address, key) in [(DEVICE, [0x51; 16]), (PARTNER, [0x52; 16])] {
            table
                .provision_link_key(address, key, TrustCenterKeyOrigin::ApplicationProvisioned)
                .unwrap();
        }
        let mut policy = policy();
        policy.application_key_requests = ApplicationLinkKeyRequestPolicy::AllowListOnly;

        assert_eq!(
            table.handle_request_key(
                policy,
                ApsmeRequestKeyIndication {
                    source_address: DEVICE,
                    key_type: ApsRequestKeyType::ApplicationLink,
                    partner_address: Some(PARTNER),
                },
                TC,
            ),
            Err(TrustCenterError::RequestNotPermitted)
        );
    }
}
