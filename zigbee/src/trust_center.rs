//! Trust Center — key management and device authentication.
//!
//! The Trust Center (TC) is responsible for:
//! - Distributing the network key to joining devices
//! - Authenticating devices via install codes or well-known key
//! - Managing link keys (TC link key per device)
//! - Handling key updates and switching
//! - Device removal and key revocation

use zigbee_types::IeeeAddress;

/// Well-known default Trust Center link key (ZigBeeAlliance09).
/// Used when install codes are not required.
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = [
    0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39,
];

/// Distributed security global link key.
pub const DISTRIBUTED_SECURITY_KEY: [u8; 16] = [
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
];

/// Install code derived key (placeholder — real derivation uses MMO hash).
pub const INSTALL_CODE_DERIVED_KEY: [u8; 16] = [0; 16];

/// Key type for TC link keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcKeyType {
    /// Default global key (ZigBeeAlliance09).
    DefaultGlobal,
    /// Install-code derived unique key per device.
    InstallCode,
    /// Application-provisioned unique key.
    ApplicationDefined,
}

/// A TC link key entry (one per joined device).
#[derive(Debug, Clone)]
pub struct TcLinkKeyEntry {
    pub ieee_address: IeeeAddress,
    pub key: [u8; 16],
    pub key_type: TcKeyType,
    /// Incoming frame counter from this device.
    pub incoming_frame_counter: u32,
    /// Whether the key has been verified (APSME-VERIFY-KEY).
    pub verified: bool,
    pub active: bool,
}

/// Trust Center state.
pub struct TrustCenter {
    /// Network key (shared by all devices in the network).
    network_key: [u8; 16],
    /// Network key sequence number.
    key_seq_number: u8,
    /// Per-device link keys.
    link_keys: [Option<TcLinkKeyEntry>; 32],
    /// Whether install codes are required for joining.
    require_install_codes: bool,
    /// Whether to allow unauthenticated joins (using default TC key).
    #[allow(dead_code)]
    allow_unsecured_join: bool,
    /// Outgoing frame counter.
    frame_counter: u32,
}

impl TrustCenter {
    pub fn new(network_key: [u8; 16]) -> Self {
        Self {
            network_key,
            key_seq_number: 0,
            link_keys: core::array::from_fn(|_| None),
            require_install_codes: false,
            allow_unsecured_join: true,
            frame_counter: 0,
        }
    }

    /// Get the current network key.
    pub fn network_key(&self) -> &[u8; 16] {
        &self.network_key
    }

    /// Set a new network key (for key rotation).
    pub fn set_network_key(&mut self, key: [u8; 16]) {
        self.network_key = key;
        self.key_seq_number = self.key_seq_number.wrapping_add(1);
    }

    /// Get the network key sequence number.
    pub fn key_seq_number(&self) -> u8 {
        self.key_seq_number
    }

    /// Set whether install codes are required.
    pub fn set_require_install_codes(&mut self, require: bool) {
        self.require_install_codes = require;
    }

    /// Get the link key for a device (or default TC key if none provisioned).
    pub fn link_key_for_device(&self, ieee: &IeeeAddress) -> [u8; 16] {
        self.link_keys
            .iter()
            .flatten()
            .find(|e| e.active && &e.ieee_address == ieee)
            .map(|e| e.key)
            .unwrap_or(DEFAULT_TC_LINK_KEY)
    }

    /// Add or update a link key entry for a device.
    #[allow(clippy::result_unit_err)]
    pub fn set_link_key(
        &mut self,
        ieee: IeeeAddress,
        key: [u8; 16],
        key_type: TcKeyType,
    ) -> Result<(), ()> {
        // Update existing entry
        if let Some(entry) = self
            .link_keys
            .iter_mut()
            .flatten()
            .find(|e| e.ieee_address == ieee)
        {
            entry.key = key;
            entry.key_type = key_type;
            entry.verified = false;
            return Ok(());
        }

        // Find empty slot
        if let Some(slot) = self.link_keys.iter_mut().find(|s| s.is_none()) {
            *slot = Some(TcLinkKeyEntry {
                ieee_address: ieee,
                key,
                key_type,
                incoming_frame_counter: 0,
                verified: false,
                active: true,
            });
            Ok(())
        } else {
            Err(()) // Table full
        }
    }

    /// Remove a device's link key (on device removal).
    pub fn remove_link_key(&mut self, ieee: &IeeeAddress) {
        if let Some(slot) = self
            .link_keys
            .iter_mut()
            .find(|s| s.as_ref().is_some_and(|e| &e.ieee_address == ieee))
        {
            *slot = None;
        }
    }

    /// Mark a device's key as verified.
    pub fn mark_key_verified(&mut self, ieee: &IeeeAddress) {
        if let Some(entry) = self
            .link_keys
            .iter_mut()
            .flatten()
            .find(|e| &e.ieee_address == ieee)
        {
            entry.verified = true;
        }
    }

    /// Check if a device's join should be accepted.
    pub fn should_accept_join(&self, ieee: &IeeeAddress) -> bool {
        if !self.require_install_codes {
            return true; // Accept all with default TC key
        }
        // Require pre-provisioned install code key
        self.link_keys
            .iter()
            .flatten()
            .any(|e| &e.ieee_address == ieee && e.key_type == TcKeyType::InstallCode)
    }

    /// Update incoming frame counter for a device (replay protection).
    pub fn update_frame_counter(&mut self, ieee: &IeeeAddress, counter: u32) -> bool {
        if let Some(entry) = self
            .link_keys
            .iter_mut()
            .flatten()
            .find(|e| &e.ieee_address == ieee)
        {
            if counter > entry.incoming_frame_counter {
                entry.incoming_frame_counter = counter;
                return true;
            }
            return false; // Replay detected
        }
        true // Unknown device, accept (will be authenticated separately)
    }

    /// Get next outgoing frame counter.
    pub fn next_frame_counter(&mut self) -> u32 {
        let fc = self.frame_counter;
        self.frame_counter += 1;
        fc
    }

    /// Number of devices with link keys.
    pub fn device_count(&self) -> usize {
        self.link_keys.iter().flatten().filter(|e| e.active).count()
    }
}
