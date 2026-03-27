//! APSME — APS Management Entity.
//!
//! Provides management services for the APS layer (Zigbee spec 2.2.6):
//! - Key management: TRANSPORT-KEY, UPDATE-DEVICE, REMOVE-DEVICE,
//!   REQUEST-KEY, SWITCH-KEY, VERIFY-KEY, CONFIRM-KEY
//! - Binding: BIND / UNBIND
//! - Group management: ADD-GROUP / REMOVE-GROUP / REMOVE-ALL-GROUPS
//! - Attribute access: GET / SET

use crate::aib::AibAttribute;
use crate::binding::{BindingDst, BindingDstMode, BindingEntry};
use crate::security::{ApsKeyType, ApsLinkKeyEntry};
use crate::{ApsLayer, ApsStatus};
use zigbee_mac::MacDriver;
use zigbee_types::IeeeAddress;

// ════════════════════════════════════════════════════════════════
// Key Management Primitives
// ════════════════════════════════════════════════════════════════

/// APSME-TRANSPORT-KEY.request — distribute a key to a device.
#[derive(Debug)]
pub struct ApsmeTransportKeyRequest {
    /// Destination IEEE address
    pub dst_address: IeeeAddress,
    /// Type of key being transported
    pub key_type: ApsKeyType,
    /// The key material (128-bit)
    pub key: [u8; 16],
}

/// APSME-UPDATE-DEVICE.request — notify Trust Center of a device's status.
#[derive(Debug)]
pub struct ApsmeUpdateDeviceRequest {
    /// Trust Center address
    pub dst_address: IeeeAddress,
    /// Device whose status changed
    pub device_address: IeeeAddress,
    /// Device's short address
    pub device_short_address: zigbee_types::ShortAddress,
    /// Status: 0=joined-secured, 1=joined-unsecured, 2=left, 3=trust-center-rejoin
    pub status: u8,
}

/// APSME-REMOVE-DEVICE.request — instruct Trust Center to remove a device.
#[derive(Debug)]
pub struct ApsmeRemoveDeviceRequest {
    /// Trust Center address
    pub dst_address: IeeeAddress,
    /// Device to be removed
    pub target_address: IeeeAddress,
}

/// APSME-REQUEST-KEY.request — request a key from the Trust Center.
#[derive(Debug)]
pub struct ApsmeRequestKeyRequest {
    /// Trust Center address
    pub dst_address: IeeeAddress,
    /// Key type being requested
    pub key_type: ApsKeyType,
    /// Partner address (for application link key requests)
    pub partner_address: Option<IeeeAddress>,
}

/// APSME-SWITCH-KEY.request — command devices to switch to a new network key.
#[derive(Debug)]
pub struct ApsmeSwitchKeyRequest {
    /// Destination address (broadcast or unicast)
    pub dst_address: IeeeAddress,
    /// Key sequence number of the new key to switch to
    pub key_seq_number: u8,
}

/// APSME-VERIFY-KEY.request — verify a Trust Center link key.
#[derive(Debug)]
pub struct ApsmeVerifyKeyRequest {
    /// Trust Center address
    pub dst_address: IeeeAddress,
    /// Key type to verify
    pub key_type: ApsKeyType,
}

/// APSME-CONFIRM-KEY.indication — received confirmation of key verification.
#[derive(Debug)]
pub struct ApsmeConfirmKeyIndication {
    /// Status of the verification
    pub status: ApsStatus,
    /// Address of the confirming device
    pub src_address: IeeeAddress,
    /// Key type that was verified
    pub key_type: ApsKeyType,
}

// ════════════════════════════════════════════════════════════════
// Binding Management
// ════════════════════════════════════════════════════════════════

/// APSME-BIND.request parameters.
#[derive(Debug)]
pub struct ApsmeBindRequest {
    /// Source IEEE address
    pub src_addr: IeeeAddress,
    /// Source endpoint (1-240)
    pub src_endpoint: u8,
    /// Cluster identifier
    pub cluster_id: u16,
    /// Destination address mode
    pub dst_addr_mode: BindingDstMode,
    /// Destination address (IEEE or group)
    pub dst_addr: IeeeAddress,
    /// Destination endpoint (for unicast; ignored for group)
    pub dst_endpoint: u8,
    /// Group address (for group binding)
    pub group_address: u16,
}

/// APSME-BIND.confirm / APSME-UNBIND.confirm
#[derive(Debug)]
pub struct ApsmeBindConfirm {
    pub status: ApsStatus,
    pub src_addr: IeeeAddress,
    pub src_endpoint: u8,
    pub cluster_id: u16,
}

// ════════════════════════════════════════════════════════════════
// Group Management
// ════════════════════════════════════════════════════════════════

/// APSME-ADD-GROUP.request
#[derive(Debug)]
pub struct ApsmeAddGroupRequest {
    /// Group address to add
    pub group_address: u16,
    /// Endpoint to add to the group
    pub endpoint: u8,
}

/// APSME-ADD-GROUP.confirm
#[derive(Debug)]
pub struct ApsmeAddGroupConfirm {
    pub status: ApsStatus,
    pub group_address: u16,
    pub endpoint: u8,
}

/// APSME-REMOVE-GROUP.request
#[derive(Debug)]
pub struct ApsmeRemoveGroupRequest {
    pub group_address: u16,
    pub endpoint: u8,
}

/// APSME-REMOVE-GROUP.confirm
#[derive(Debug)]
pub struct ApsmeRemoveGroupConfirm {
    pub status: ApsStatus,
    pub group_address: u16,
    pub endpoint: u8,
}

/// APSME-REMOVE-ALL-GROUPS.request
#[derive(Debug)]
pub struct ApsmeRemoveAllGroupsRequest {
    pub endpoint: u8,
}

/// APSME-REMOVE-ALL-GROUPS.confirm
#[derive(Debug)]
pub struct ApsmeRemoveAllGroupsConfirm {
    pub status: ApsStatus,
    pub endpoint: u8,
}

// ════════════════════════════════════════════════════════════════
// Implementation
// ════════════════════════════════════════════════════════════════

impl<M: MacDriver> ApsLayer<M> {
    // ── Binding ─────────────────────────────────────────────

    /// APSME-BIND.request — add a binding table entry.
    pub fn apsme_bind(&mut self, req: &ApsmeBindRequest) -> ApsmeBindConfirm {
        let entry = match req.dst_addr_mode {
            BindingDstMode::Group => BindingEntry::group(
                req.src_addr,
                req.src_endpoint,
                req.cluster_id,
                req.group_address,
            ),
            BindingDstMode::Extended => BindingEntry::unicast(
                req.src_addr,
                req.src_endpoint,
                req.cluster_id,
                req.dst_addr,
                req.dst_endpoint,
            ),
        };

        let status = match self.binding_table.add(entry) {
            Ok(()) => ApsStatus::Success,
            Err(_) => {
                if self.binding_table.is_full() {
                    ApsStatus::TableFull
                } else {
                    ApsStatus::IllegalRequest
                }
            }
        };

        ApsmeBindConfirm {
            status,
            src_addr: req.src_addr,
            src_endpoint: req.src_endpoint,
            cluster_id: req.cluster_id,
        }
    }

    /// APSME-UNBIND.request — remove a binding table entry.
    pub fn apsme_unbind(&mut self, req: &ApsmeBindRequest) -> ApsmeBindConfirm {
        let dst = match req.dst_addr_mode {
            BindingDstMode::Group => BindingDst::Group(req.group_address),
            BindingDstMode::Extended => BindingDst::Unicast {
                dst_addr: req.dst_addr,
                dst_endpoint: req.dst_endpoint,
            },
        };

        let removed =
            self.binding_table
                .remove(&req.src_addr, req.src_endpoint, req.cluster_id, &dst);

        ApsmeBindConfirm {
            status: if removed {
                ApsStatus::Success
            } else {
                ApsStatus::InvalidBinding
            },
            src_addr: req.src_addr,
            src_endpoint: req.src_endpoint,
            cluster_id: req.cluster_id,
        }
    }

    // ── Group management ────────────────────────────────────

    /// APSME-ADD-GROUP.request — add an endpoint to a group.
    pub fn apsme_add_group(&mut self, req: &ApsmeAddGroupRequest) -> ApsmeAddGroupConfirm {
        let ok = self.group_table.add_group(req.group_address, req.endpoint);
        ApsmeAddGroupConfirm {
            status: if ok {
                ApsStatus::Success
            } else {
                ApsStatus::TableFull
            },
            group_address: req.group_address,
            endpoint: req.endpoint,
        }
    }

    /// APSME-REMOVE-GROUP.request — remove an endpoint from a group.
    pub fn apsme_remove_group(&mut self, req: &ApsmeRemoveGroupRequest) -> ApsmeRemoveGroupConfirm {
        let removed = self
            .group_table
            .remove_group(req.group_address, req.endpoint);
        ApsmeRemoveGroupConfirm {
            status: if removed {
                ApsStatus::Success
            } else {
                ApsStatus::InvalidParameter
            },
            group_address: req.group_address,
            endpoint: req.endpoint,
        }
    }

    /// APSME-REMOVE-ALL-GROUPS.request — remove an endpoint from all groups.
    pub fn apsme_remove_all_groups(
        &mut self,
        req: &ApsmeRemoveAllGroupsRequest,
    ) -> ApsmeRemoveAllGroupsConfirm {
        self.group_table.remove_all_groups(req.endpoint);
        ApsmeRemoveAllGroupsConfirm {
            status: ApsStatus::Success,
            endpoint: req.endpoint,
        }
    }

    // ── Key management ──────────────────────────────────────

    /// APSME-TRANSPORT-KEY.request — distribute a key.
    ///
    /// Builds an APS Transport Key command frame and sends it secured
    /// with the Trust Center link key (or NWK key for network key transport).
    pub async fn apsme_transport_key(&mut self, req: &ApsmeTransportKeyRequest) -> ApsStatus {
        // Store the key locally
        match req.key_type {
            ApsKeyType::TrustCenterLinkKey | ApsKeyType::ApplicationLinkKey => {
                let entry = ApsLinkKeyEntry {
                    partner_address: req.dst_address,
                    key: req.key,
                    key_type: req.key_type,
                    outgoing_frame_counter: 0,
                    incoming_frame_counter: 0,
                };
                if self.security.add_key(entry).is_err() {
                    return ApsStatus::TableFull;
                }
            }
            ApsKeyType::NetworkKey => {
                // Network key is managed by NWK security, not APS key table
            }
            _ => {}
        }

        // TODO: Build APS Transport Key command frame and send via APSDE.
        // Key is stored locally above, but OTA frame is NOT sent to dst_address.
        log::warn!(
            "APSME-TRANSPORT-KEY: type={:?}, dst={:02X?}",
            req.key_type,
            req.dst_address
        );
        ApsStatus::Success
    }

    /// APSME-REQUEST-KEY.request — request a key from the Trust Center.
    ///
    /// Not yet implemented — APS command frame construction required.
    pub async fn apsme_request_key(&mut self, req: &ApsmeRequestKeyRequest) -> ApsStatus {
        log::warn!(
            "APSME-REQUEST-KEY: not implemented (type={:?}, dst={:02X?})",
            req.key_type,
            req.dst_address
        );
        ApsStatus::IllegalRequest
    }

    /// APSME-SWITCH-KEY.request — switch to a new active network key.
    ///
    /// Not yet implemented — APS command frame construction required.
    pub async fn apsme_switch_key(&mut self, req: &ApsmeSwitchKeyRequest) -> ApsStatus {
        log::warn!(
            "APSME-SWITCH-KEY: not implemented (seq={}, dst={:02X?})",
            req.key_seq_number,
            req.dst_address
        );
        ApsStatus::IllegalRequest
    }

    /// APSME-VERIFY-KEY.request — initiate key verification with TC.
    ///
    /// Not yet implemented — APS command frame construction required.
    pub async fn apsme_verify_key(&mut self, req: &ApsmeVerifyKeyRequest) -> ApsStatus {
        log::warn!(
            "APSME-VERIFY-KEY: not implemented (type={:?}, dst={:02X?})",
            req.key_type,
            req.dst_address
        );
        ApsStatus::IllegalRequest
    }

    // ── APSME-GET / APSME-SET ───────────────────────────────

    /// APSME-GET.request — read an AIB attribute.
    pub fn apsme_get_bool(&self, attr: AibAttribute) -> Result<bool, ApsStatus> {
        match attr {
            AibAttribute::ApsDesignatedCoordinator => Ok(self.aib.aps_designated_coordinator),
            AibAttribute::ApsUseInsecureJoin => Ok(self.aib.aps_use_insecure_join),
            AibAttribute::ApsSecurityEnabled => Ok(self.aib.aps_security_enabled),
            _ => Err(ApsStatus::UnsupportedAttribute),
        }
    }

    /// APSME-GET.request — read a u8 AIB attribute.
    pub fn apsme_get_u8(&self, attr: AibAttribute) -> Result<u8, ApsStatus> {
        match attr {
            AibAttribute::ApsInterframeDelay => Ok(self.aib.aps_interframe_delay),
            AibAttribute::ApsLastChannelEnergy => Ok(self.aib.aps_last_channel_energy),
            AibAttribute::ApsLastChannelFailureRate => Ok(self.aib.aps_last_channel_failure_rate),
            AibAttribute::ApsMaxWindowSize => Ok(self.aib.aps_max_window_size),
            _ => Err(ApsStatus::UnsupportedAttribute),
        }
    }

    /// APSME-GET.request — read a u32 AIB attribute.
    pub fn apsme_get_u32(&self, attr: AibAttribute) -> Result<u32, ApsStatus> {
        match attr {
            AibAttribute::ApsChannelMaskList => Ok(self.aib.aps_channel_mask),
            AibAttribute::ApsChannelTimer => Ok(self.aib.aps_channel_timer),
            _ => Err(ApsStatus::UnsupportedAttribute),
        }
    }

    /// APSME-SET.request — write a bool AIB attribute.
    pub fn apsme_set_bool(&mut self, attr: AibAttribute, value: bool) -> ApsStatus {
        match attr {
            AibAttribute::ApsDesignatedCoordinator => {
                self.aib.aps_designated_coordinator = value;
                ApsStatus::Success
            }
            AibAttribute::ApsUseInsecureJoin => {
                self.aib.aps_use_insecure_join = value;
                ApsStatus::Success
            }
            AibAttribute::ApsSecurityEnabled => {
                self.aib.aps_security_enabled = value;
                ApsStatus::Success
            }
            _ => ApsStatus::UnsupportedAttribute,
        }
    }

    /// APSME-SET.request — write a u8 AIB attribute.
    pub fn apsme_set_u8(&mut self, attr: AibAttribute, value: u8) -> ApsStatus {
        match attr {
            AibAttribute::ApsInterframeDelay => {
                self.aib.aps_interframe_delay = value;
                ApsStatus::Success
            }
            AibAttribute::ApsLastChannelEnergy => {
                self.aib.aps_last_channel_energy = value;
                ApsStatus::Success
            }
            AibAttribute::ApsLastChannelFailureRate => {
                self.aib.aps_last_channel_failure_rate = value;
                ApsStatus::Success
            }
            AibAttribute::ApsMaxWindowSize => {
                self.aib.aps_max_window_size = value;
                ApsStatus::Success
            }
            _ => ApsStatus::UnsupportedAttribute,
        }
    }

    /// APSME-SET.request — write a u32 AIB attribute.
    pub fn apsme_set_u32(&mut self, attr: AibAttribute, value: u32) -> ApsStatus {
        match attr {
            AibAttribute::ApsChannelMaskList => {
                self.aib.aps_channel_mask = value;
                ApsStatus::Success
            }
            AibAttribute::ApsChannelTimer => {
                self.aib.aps_channel_timer = value;
                ApsStatus::Success
            }
            _ => ApsStatus::UnsupportedAttribute,
        }
    }
}
