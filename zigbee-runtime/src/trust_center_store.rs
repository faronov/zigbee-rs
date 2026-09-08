//! Network-bound, crash-safe Trust Center device/key persistence.

use embedded_storage::nor_flash::NorFlash;
use heapless::Vec;
use zigbee_bdb::attributes::NetworkKeyUpdateMethod;
use zigbee_bdb::trust_center::{
    MAX_TRUST_CENTER_DEVICES, TrustCenterDevice, TrustCenterKeyAttributes, TrustCenterKeyOrigin,
};
use zigbee_types::{IeeeAddress, ShortAddress};

const DEVICE_ENCODED_LEN: usize = 75;
const STATE_HEADER_LEN: usize = 43;
const APPLICATION_KEY_REQUEST_ENTRY_LEN: usize = 16;
const ROTATION_TRAILER_LEN: usize = 3;
const UNKNOWN_REMOVAL_ENTRY_LEN: usize = 16;
const UNKNOWN_REMOVAL_HEADER_LEN: usize = 1;
pub const MAX_APPLICATION_KEY_REQUESTS: usize = 8;
pub const MAX_ENCODED_TRUST_CENTER_STATE_LEN: usize = STATE_HEADER_LEN
    + MAX_TRUST_CENTER_DEVICES * DEVICE_ENCODED_LEN
    + 1
    + MAX_APPLICATION_KEY_REQUESTS * APPLICATION_KEY_REQUEST_ENTRY_LEN
    + ROTATION_TRAILER_LEN
    + UNKNOWN_REMOVAL_HEADER_LEN
    + MAX_TRUST_CENTER_DEVICES * UNKNOWN_REMOVAL_ENTRY_LEN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NetworkKeyRotationPhase {
    Distributing = 1,
    Switching = 2,
    Activating = 3,
    WaitingForPropagation = 4,
}

impl NetworkKeyRotationPhase {
    const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Distributing),
            2 => Some(Self::Switching),
            3 => Some(Self::Activating),
            4 => Some(Self::WaitingForPropagation),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetworkKeyRotation {
    pub target_sequence: u8,
    pub phase: NetworkKeyRotationPhase,
    pub method: NetworkKeyUpdateMethod,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentTrustCenterDevice {
    pub device: TrustCenterDevice,
    pub outgoing_frame_counter: u32,
    pub outgoing_frame_counter_limit: u32,
    pub incoming_frame_counter: u32,
    pub incoming_frame_counter_valid: bool,
    /// Initial Network-Key transport has not completed yet.
    pub network_key_pending: bool,
    /// Confirm-Key local submission has not been durably recorded.
    pub confirm_key_pending: Option<u8>,
    /// Parent-directed removal/local revocation intent, not proof of departure.
    pub removal_pending: bool,
    /// The router's unicast update was transmitted, not a receipt confirmation.
    pub new_network_key_delivered: bool,
    /// Router capability learned from an authenticated Device_annce.
    pub is_router: Option<bool>,
    /// New TCLK durably staged before its Transport-Key is transmitted.
    pub pending_link_key: Option<PendingTrustCenterLinkKey>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendingTrustCenterLinkKey {
    pub key: [u8; 16],
    pub outgoing_frame_counter: u32,
    pub outgoing_frame_counter_limit: u32,
    /// Transport-Key was sent; the old key remains current until Verify-Key.
    pub transported: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendingApplicationLinkKey {
    pub initiator_address: IeeeAddress,
    pub responder_address: IeeeAddress,
    pub key: [u8; 16],
    /// Local send progress only; AR=0 supplies no peer installation receipt.
    pub delivered_to_initiator: bool,
    /// Local send progress only; AR=0 supplies no peer installation receipt.
    pub delivered_to_responder: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApplicationKeyRequestEntry {
    pub initiator_address: IeeeAddress,
    pub responder_address: IeeeAddress,
}

/// A rejected peer is not an admitted device and owns no Trust Center key.
/// Keep its parent-directed removal durable without inventing a device entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendingUnknownRemoval {
    pub parent_address: IeeeAddress,
    pub device_address: IeeeAddress,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PersistentTrustCenterState {
    extended_pan_id: IeeeAddress,
    pending_application_key: Option<PendingApplicationLinkKey>,
    rotation: Option<NetworkKeyRotation>,
    application_key_requests: Vec<ApplicationKeyRequestEntry, MAX_APPLICATION_KEY_REQUESTS>,
    devices: Vec<PersistentTrustCenterDevice, MAX_TRUST_CENTER_DEVICES>,
    pending_unknown_removals: Vec<PendingUnknownRemoval, MAX_TRUST_CENTER_DEVICES>,
}

impl PersistentTrustCenterState {
    pub fn new(extended_pan_id: IeeeAddress) -> Self {
        Self {
            extended_pan_id,
            pending_application_key: None,
            rotation: None,
            application_key_requests: Vec::new(),
            devices: Vec::new(),
            pending_unknown_removals: Vec::new(),
        }
    }

    pub const fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    pub fn matches_network(&self, extended_pan_id: &IeeeAddress) -> bool {
        self.extended_pan_id == *extended_pan_id
    }

    pub fn devices(&self) -> &[PersistentTrustCenterDevice] {
        self.devices.as_slice()
    }

    pub fn pending_unknown_removals(&self) -> &[PendingUnknownRemoval] {
        &self.pending_unknown_removals
    }

    pub(crate) fn stage_unknown_removal(
        &mut self,
        pending: PendingUnknownRemoval,
    ) -> Result<(), TrustCenterStoreError> {
        if pending.parent_address == [0; 8]
            || pending.parent_address == [0xFF; 8]
            || pending.device_address == [0; 8]
            || pending.device_address == [0xFF; 8]
            || self.device(&pending.device_address).is_some()
        {
            return Err(TrustCenterStoreError::Corrupt);
        }
        if let Some(existing) = self
            .pending_unknown_removals
            .iter_mut()
            .find(|existing| existing.device_address == pending.device_address)
        {
            *existing = pending;
        } else {
            self.pending_unknown_removals
                .push(pending)
                .map_err(|_| TrustCenterStoreError::Full)?;
        }
        Ok(())
    }

    pub(crate) fn remove_unknown_removal(&mut self, address: &IeeeAddress) -> bool {
        if let Some(index) = self
            .pending_unknown_removals
            .iter()
            .position(|pending| pending.device_address == *address)
        {
            self.pending_unknown_removals.swap_remove(index);
            true
        } else {
            false
        }
    }

    pub(crate) fn devices_mut(&mut self) -> &mut [PersistentTrustCenterDevice] {
        self.devices.as_mut_slice()
    }

    pub const fn pending_application_key(&self) -> Option<PendingApplicationLinkKey> {
        self.pending_application_key
    }

    pub const fn rotation(&self) -> Option<NetworkKeyRotation> {
        self.rotation
    }

    pub fn set_rotation(&mut self, rotation: Option<NetworkKeyRotation>) {
        self.rotation = rotation;
        if rotation.is_none() {
            for stored in &mut self.devices {
                stored.new_network_key_delivered = false;
            }
        }
    }

    pub fn mark_network_key_delivered(
        &mut self,
        address: &IeeeAddress,
    ) -> Result<(), TrustCenterStoreError> {
        let stored = self
            .device_mut(address)
            .ok_or(TrustCenterStoreError::Corrupt)?;
        stored.new_network_key_delivered = true;
        self.validate()
    }

    pub fn set_pending_application_key(&mut self, pending: Option<PendingApplicationLinkKey>) {
        self.pending_application_key = pending;
    }

    pub fn application_key_request_allowed(
        &self,
        initiator_address: &IeeeAddress,
        responder_address: &IeeeAddress,
    ) -> bool {
        self.application_key_requests.iter().any(|entry| {
            entry.initiator_address == *initiator_address
                && entry.responder_address == *responder_address
        })
    }

    pub fn allow_application_key_request(
        &mut self,
        initiator_address: IeeeAddress,
        responder_address: IeeeAddress,
    ) -> Result<(), TrustCenterStoreError> {
        let entry = ApplicationKeyRequestEntry {
            initiator_address,
            responder_address,
        };
        if !self.application_key_requests.contains(&entry) {
            self.application_key_requests
                .push(entry)
                .map_err(|_| TrustCenterStoreError::Full)?;
        }
        self.validate()
    }

    pub fn deny_application_key_request(
        &mut self,
        initiator_address: &IeeeAddress,
        responder_address: &IeeeAddress,
    ) -> bool {
        if let Some(index) = self.application_key_requests.iter().position(|entry| {
            entry.initiator_address == *initiator_address
                && entry.responder_address == *responder_address
        }) {
            self.application_key_requests.swap_remove(index);
            true
        } else {
            false
        }
    }

    pub fn remove_application_key_requests_for(&mut self, address: &IeeeAddress) -> bool {
        let mut changed = false;
        let mut index = 0;
        while index < self.application_key_requests.len() {
            let entry = self.application_key_requests[index];
            if entry.initiator_address == *address || entry.responder_address == *address {
                self.application_key_requests.swap_remove(index);
                changed = true;
            } else {
                index += 1;
            }
        }
        changed
    }

    pub fn application_key_requests(&self) -> &[ApplicationKeyRequestEntry] {
        self.application_key_requests.as_slice()
    }

    pub fn device(&self, address: &IeeeAddress) -> Option<&PersistentTrustCenterDevice> {
        self.devices
            .iter()
            .find(|device| device.device.ieee_address == *address)
    }

    pub fn device_mut(
        &mut self,
        address: &IeeeAddress,
    ) -> Option<&mut PersistentTrustCenterDevice> {
        self.devices
            .iter_mut()
            .find(|device| device.device.ieee_address == *address)
    }

    pub fn upsert(
        &mut self,
        device: PersistentTrustCenterDevice,
    ) -> Result<(), TrustCenterStoreError> {
        if let Some(existing) = self.device_mut(&device.device.ieee_address) {
            *existing = device;
        } else {
            self.devices
                .push(device)
                .map_err(|_| TrustCenterStoreError::Full)?;
        }
        self.validate()
    }

    pub fn remove(&mut self, address: &IeeeAddress) -> bool {
        if let Some(index) = self
            .devices
            .iter()
            .position(|device| device.device.ieee_address == *address)
        {
            self.devices.swap_remove(index);
            true
        } else {
            false
        }
    }

    pub fn validate(&self) -> Result<(), TrustCenterStoreError> {
        if self.extended_pan_id == [0u8; 8] || self.extended_pan_id == [0xFFu8; 8] {
            return Err(TrustCenterStoreError::Corrupt);
        }
        for (index, pending) in self.pending_unknown_removals.iter().enumerate() {
            if pending.parent_address == [0; 8]
                || pending.parent_address == [0xFF; 8]
                || pending.device_address == [0; 8]
                || pending.device_address == [0xFF; 8]
                || self.device(&pending.device_address).is_some()
                || self.pending_unknown_removals[index + 1..]
                    .iter()
                    .any(|other| other.device_address == pending.device_address)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
        }
        if self.pending_application_key.is_some_and(|pending| {
            pending.initiator_address == [0u8; 8]
                || pending.initiator_address == [0xFFu8; 8]
                || pending.responder_address == [0u8; 8]
                || pending.responder_address == [0xFFu8; 8]
                || pending.initiator_address == pending.responder_address
                || self.device(&pending.initiator_address).is_none()
                || self.device(&pending.responder_address).is_none()
        }) {
            return Err(TrustCenterStoreError::Corrupt);
        }
        if self.rotation.is_none()
            && self
                .devices
                .iter()
                .any(|stored| stored.new_network_key_delivered)
        {
            return Err(TrustCenterStoreError::Corrupt);
        }
        for (index, entry) in self.application_key_requests.iter().enumerate() {
            if entry.initiator_address == [0u8; 8]
                || entry.initiator_address == [0xFFu8; 8]
                || entry.responder_address == [0u8; 8]
                || entry.responder_address == [0xFFu8; 8]
                || entry.initiator_address == entry.responder_address
                || self.application_key_requests[index + 1..]
                    .iter()
                    .any(|other| other == entry)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
        }
        for (index, stored) in self.devices.iter().enumerate() {
            let device = stored.device;
            if !device.active
                || device.ieee_address == [0u8; 8]
                || device.ieee_address == [0xFFu8; 8]
                || self.devices[index + 1..]
                    .iter()
                    .any(|other| other.device.ieee_address == device.ieee_address)
                || stored.outgoing_frame_counter >= stored.outgoing_frame_counter_limit
                || stored.pending_link_key.is_some_and(|pending| {
                    pending.outgoing_frame_counter >= pending.outgoing_frame_counter_limit
                })
                || stored
                    .confirm_key_pending
                    .is_some_and(|status| status != 0x00 && status != 0xAD)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            let provisioned =
                device.parent_address == [0u8; 8] && device.short_address == ShortAddress(0xFFFF);
            let joined = device.parent_address != [0u8; 8]
                && device.parent_address != [0xFFu8; 8]
                && device.short_address.0 <= 0xFFF7;
            if !provisioned && !joined {
                return Err(TrustCenterStoreError::Corrupt);
            }
        }
        Ok(())
    }

    pub fn encode(&self, output: &mut [u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN]) -> usize {
        output.fill(0);
        output[0..8].copy_from_slice(&self.extended_pan_id);
        output[8] = self.devices.len() as u8;
        if let Some(pending) = self.pending_application_key {
            output[9] = 1;
            output[10..18].copy_from_slice(&pending.initiator_address);
            output[18..26].copy_from_slice(&pending.responder_address);
            output[26..42].copy_from_slice(&pending.key);
            output[42] = u8::from(pending.delivered_to_initiator)
                | (u8::from(pending.delivered_to_responder) << 1);
        }
        let mut offset = STATE_HEADER_LEN;
        for stored in &self.devices {
            let device = stored.device;
            output[offset..offset + 8].copy_from_slice(&device.ieee_address);
            output[offset + 8..offset + 16].copy_from_slice(&device.parent_address);
            output[offset + 16..offset + 18].copy_from_slice(&device.short_address.0.to_le_bytes());
            output[offset + 18..offset + 34].copy_from_slice(&device.link_key);
            output[offset + 34] = encode_origin(device.key_origin);
            output[offset + 35] = device.key_attributes as u8;
            output[offset + 36] = device.join_timeout_remaining_secs;
            output[offset + 37] = u8::from(stored.incoming_frame_counter_valid)
                | (u8::from(stored.pending_link_key.is_some()) << 1)
                | (u8::from(stored.network_key_pending) << 2)
                | (u8::from(stored.confirm_key_pending.is_some()) << 3)
                | (u8::from(stored.removal_pending) << 4)
                | (u8::from(
                    stored
                        .pending_link_key
                        .is_some_and(|pending| pending.transported),
                ) << 5)
                | (u8::from(stored.new_network_key_delivered) << 6)
                | (u8::from(stored.is_router.is_some()) << 7);
            output[offset + 38..offset + 42]
                .copy_from_slice(&stored.outgoing_frame_counter.to_le_bytes());
            output[offset + 42..offset + 46]
                .copy_from_slice(&stored.outgoing_frame_counter_limit.to_le_bytes());
            output[offset + 46..offset + 50]
                .copy_from_slice(&stored.incoming_frame_counter.to_le_bytes());
            if let Some(pending) = stored.pending_link_key {
                output[offset + 50..offset + 66].copy_from_slice(&pending.key);
                output[offset + 66..offset + 70]
                    .copy_from_slice(&pending.outgoing_frame_counter.to_le_bytes());
                output[offset + 70..offset + 74]
                    .copy_from_slice(&pending.outgoing_frame_counter_limit.to_le_bytes());
            }
            output[offset + 74] = stored.confirm_key_pending.unwrap_or(0)
                | (u8::from(stored.is_router == Some(true)) << 6);
            offset += DEVICE_ENCODED_LEN;
        }
        output[offset] = self.application_key_requests.len() as u8;
        offset += 1;
        for entry in &self.application_key_requests {
            output[offset..offset + 8].copy_from_slice(&entry.initiator_address);
            output[offset + 8..offset + 16].copy_from_slice(&entry.responder_address);
            offset += APPLICATION_KEY_REQUEST_ENTRY_LEN;
        }
        if let Some(rotation) = self.rotation {
            output[offset] = rotation.phase as u8;
            output[offset + 1] = rotation.target_sequence;
            output[offset + 2] = rotation.method as u8;
        }
        offset += ROTATION_TRAILER_LEN;
        output[offset] = self.pending_unknown_removals.len() as u8;
        offset += UNKNOWN_REMOVAL_HEADER_LEN;
        for pending in &self.pending_unknown_removals {
            output[offset..offset + 8].copy_from_slice(&pending.parent_address);
            output[offset + 8..offset + 16].copy_from_slice(&pending.device_address);
            offset += UNKNOWN_REMOVAL_ENTRY_LEN;
        }
        offset
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_version(bytes, true, true, true)
    }

    fn decode_v3(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_version(bytes, true, true, false)
    }

    fn decode_v2(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_version(bytes, true, false, false)
    }

    fn decode_v1(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_version(bytes, false, false, false)
    }

    fn decode_version(
        bytes: &[u8],
        has_rotation: bool,
        has_unknown_removals: bool,
        has_router_capability: bool,
    ) -> Result<Self, TrustCenterStoreError> {
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(bytes.get(0..8).ok_or(TrustCenterStoreError::Corrupt)?);
        let count = usize::from(*bytes.get(8).ok_or(TrustCenterStoreError::Corrupt)?);
        let device_bytes_len = STATE_HEADER_LEN + count * DEVICE_ENCODED_LEN;
        if count > MAX_TRUST_CENTER_DEVICES || bytes.len() < device_bytes_len {
            return Err(TrustCenterStoreError::Corrupt);
        }
        let mut state = Self::new(extended_pan_id);
        state.pending_application_key = match bytes[9] {
            0 => {
                if bytes[10..43].iter().any(|byte| *byte != 0) {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                None
            }
            1 => {
                if bytes[42] & !0x03 != 0 {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                let mut initiator_address = [0u8; 8];
                initiator_address.copy_from_slice(&bytes[10..18]);
                let mut responder_address = [0u8; 8];
                responder_address.copy_from_slice(&bytes[18..26]);
                let mut key = [0u8; 16];
                key.copy_from_slice(&bytes[26..42]);
                Some(PendingApplicationLinkKey {
                    initiator_address,
                    responder_address,
                    key,
                    delivered_to_initiator: bytes[42] & 0x01 != 0,
                    delivered_to_responder: bytes[42] & 0x02 != 0,
                })
            }
            _ => return Err(TrustCenterStoreError::Corrupt),
        };
        let mut offset = STATE_HEADER_LEN;
        for _ in 0..count {
            let encoded = &bytes[offset..offset + DEVICE_ENCODED_LEN];
            let mut ieee_address = [0u8; 8];
            ieee_address.copy_from_slice(&encoded[0..8]);
            let mut parent_address = [0u8; 8];
            parent_address.copy_from_slice(&encoded[8..16]);
            let mut link_key = [0u8; 16];
            link_key.copy_from_slice(&encoded[18..34]);
            let key_origin = decode_origin(encoded[34])?;
            let key_attributes = decode_attributes(encoded[35])?;
            let allowed_flags = if has_router_capability {
                0xFF
            } else if has_rotation {
                0x7F
            } else {
                0x3F
            };
            if encoded[37] & !allowed_flags != 0
                || (encoded[37] & 0x20 != 0 && encoded[37] & 0x02 == 0)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            let pending_link_key = if encoded[37] & 0x02 != 0 {
                let mut key = [0u8; 16];
                key.copy_from_slice(&encoded[50..66]);
                Some(PendingTrustCenterLinkKey {
                    key,
                    outgoing_frame_counter: u32::from_le_bytes([
                        encoded[66],
                        encoded[67],
                        encoded[68],
                        encoded[69],
                    ]),
                    outgoing_frame_counter_limit: u32::from_le_bytes([
                        encoded[70],
                        encoded[71],
                        encoded[72],
                        encoded[73],
                    ]),
                    transported: encoded[37] & 0x20 != 0,
                })
            } else {
                if encoded[50..74].iter().any(|byte| *byte != 0) {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                None
            };
            let is_router = if has_router_capability && encoded[37] & 0x80 != 0 {
                Some(encoded[74] & 0x40 != 0)
            } else {
                if encoded[74] & 0x40 != 0 {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                None
            };
            let status = encoded[74] & !0x40;
            let confirm_key_pending = if encoded[37] & 0x08 != 0 {
                Some(status)
            } else {
                if status != 0 {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                None
            };
            state
                .devices
                .push(PersistentTrustCenterDevice {
                    device: TrustCenterDevice {
                        ieee_address,
                        parent_address,
                        short_address: ShortAddress(u16::from_le_bytes([encoded[16], encoded[17]])),
                        link_key,
                        key_origin,
                        key_attributes,
                        join_timeout_remaining_secs: encoded[36],
                        active: true,
                    },
                    incoming_frame_counter_valid: encoded[37] & 0x01 != 0,
                    network_key_pending: encoded[37] & 0x04 != 0,
                    confirm_key_pending,
                    removal_pending: encoded[37] & 0x10 != 0,
                    new_network_key_delivered: has_rotation && encoded[37] & 0x40 != 0,
                    is_router,
                    outgoing_frame_counter: u32::from_le_bytes([
                        encoded[38],
                        encoded[39],
                        encoded[40],
                        encoded[41],
                    ]),
                    outgoing_frame_counter_limit: u32::from_le_bytes([
                        encoded[42],
                        encoded[43],
                        encoded[44],
                        encoded[45],
                    ]),
                    incoming_frame_counter: u32::from_le_bytes([
                        encoded[46],
                        encoded[47],
                        encoded[48],
                        encoded[49],
                    ]),
                    pending_link_key,
                })
                .map_err(|_| TrustCenterStoreError::Full)?;
            offset += DEVICE_ENCODED_LEN;
        }
        if has_rotation {
            let allow_count = usize::from(
                *bytes
                    .get(device_bytes_len)
                    .ok_or(TrustCenterStoreError::Corrupt)?,
            );
            let rotation_offset =
                device_bytes_len + 1 + allow_count * APPLICATION_KEY_REQUEST_ENTRY_LEN;
            let removal_offset = rotation_offset + ROTATION_TRAILER_LEN;
            if allow_count > MAX_APPLICATION_KEY_REQUESTS
                || bytes.len() < removal_offset
                || (!has_unknown_removals && bytes.len() != removal_offset)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            let mut allow_offset = device_bytes_len + 1;
            for _ in 0..allow_count {
                let mut initiator_address = [0u8; 8];
                initiator_address.copy_from_slice(&bytes[allow_offset..allow_offset + 8]);
                let mut responder_address = [0u8; 8];
                responder_address.copy_from_slice(&bytes[allow_offset + 8..allow_offset + 16]);
                state
                    .application_key_requests
                    .push(ApplicationKeyRequestEntry {
                        initiator_address,
                        responder_address,
                    })
                    .map_err(|_| TrustCenterStoreError::Full)?;
                allow_offset += APPLICATION_KEY_REQUEST_ENTRY_LEN;
            }
            state.rotation = match bytes[rotation_offset] {
                0 => {
                    if bytes[rotation_offset + 1] != 0 || bytes[rotation_offset + 2] != 0 {
                        return Err(TrustCenterStoreError::Corrupt);
                    }
                    None
                }
                phase => Some(NetworkKeyRotation {
                    target_sequence: bytes[rotation_offset + 1],
                    phase: NetworkKeyRotationPhase::from_u8(phase)
                        .ok_or(TrustCenterStoreError::Corrupt)?,
                    method: NetworkKeyUpdateMethod::from_u8(bytes[rotation_offset + 2])
                        .ok_or(TrustCenterStoreError::Corrupt)?,
                }),
            };
            if has_unknown_removals {
                let removal_count = usize::from(
                    *bytes
                        .get(removal_offset)
                        .ok_or(TrustCenterStoreError::Corrupt)?,
                );
                if removal_count > MAX_TRUST_CENTER_DEVICES
                    || bytes.len()
                        != removal_offset
                            + UNKNOWN_REMOVAL_HEADER_LEN
                            + removal_count * UNKNOWN_REMOVAL_ENTRY_LEN
                {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                let mut offset = removal_offset + UNKNOWN_REMOVAL_HEADER_LEN;
                for _ in 0..removal_count {
                    let mut parent_address = [0; 8];
                    parent_address.copy_from_slice(&bytes[offset..offset + 8]);
                    let mut device_address = [0; 8];
                    device_address.copy_from_slice(&bytes[offset + 8..offset + 16]);
                    state
                        .pending_unknown_removals
                        .push(PendingUnknownRemoval {
                            parent_address,
                            device_address,
                        })
                        .map_err(|_| TrustCenterStoreError::Full)?;
                    offset += UNKNOWN_REMOVAL_ENTRY_LEN;
                }
            }
        } else if bytes.len() != device_bytes_len {
            let allow_count = usize::from(
                *bytes
                    .get(device_bytes_len)
                    .ok_or(TrustCenterStoreError::Corrupt)?,
            );
            if allow_count > MAX_APPLICATION_KEY_REQUESTS
                || bytes.len()
                    != device_bytes_len + 1 + allow_count * APPLICATION_KEY_REQUEST_ENTRY_LEN
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            let mut allow_offset = device_bytes_len + 1;
            for _ in 0..allow_count {
                let mut initiator_address = [0u8; 8];
                initiator_address.copy_from_slice(&bytes[allow_offset..allow_offset + 8]);
                let mut responder_address = [0u8; 8];
                responder_address.copy_from_slice(&bytes[allow_offset + 8..allow_offset + 16]);
                state
                    .application_key_requests
                    .push(ApplicationKeyRequestEntry {
                        initiator_address,
                        responder_address,
                    })
                    .map_err(|_| TrustCenterStoreError::Full)?;
                allow_offset += APPLICATION_KEY_REQUEST_ENTRY_LEN;
            }
        }
        state.validate()?;
        Ok(state)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustCenterStoreError {
    Corrupt,
    Full,
    Hardware,
    GenerationExhausted,
    ForeignNetwork,
    CounterExhausted,
}

pub trait TrustCenterDeviceStore {
    fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError>;
    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError>;
    fn clear(&mut self) -> Result<(), TrustCenterStoreError>;
}

#[derive(Debug, Default)]
pub struct RamTrustCenterDeviceStore {
    state: Option<PersistentTrustCenterState>,
}

impl RamTrustCenterDeviceStore {
    pub const fn new() -> Self {
        Self { state: None }
    }
}

impl TrustCenterDeviceStore for RamTrustCenterDeviceStore {
    fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
        Ok(self.state.clone())
    }

    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        state.validate()?;
        self.state = Some(state.clone());
        Ok(())
    }

    fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
        self.state = None;
        Ok(())
    }
}

pub const TRUST_CENTER_JOURNAL_SECTOR_SIZE: usize = 4096;
pub const TRUST_CENTER_JOURNAL_SLOT_SIZE: usize = 4096;
pub const TRUST_CENTER_JOURNAL_SLOTS_PER_SECTOR: usize =
    TRUST_CENTER_JOURNAL_SECTOR_SIZE / TRUST_CENTER_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBTC";
const LEGACY_RECORD_VERSION: u8 = 1;
const ROTATION_RECORD_VERSION: u8 = 2;
const UNKNOWN_REMOVAL_RECORD_VERSION: u8 = 3;
/// Trust Center journal epoch that production Coordinator rollback policy
/// must not cross after committing this format.
pub const TRUST_CENTER_JOURNAL_FORMAT_VERSION: u8 = 4;
const RECORD_VERSION: u8 = TRUST_CENTER_JOURNAL_FORMAT_VERSION;
const RECORD_ENCODED_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 12;
const RECORD_PREFIX_LEN: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT_OFFSET: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

const _: () =
    assert!(RECORD_ENCODED_OFFSET + MAX_ENCODED_TRUST_CENTER_STATE_LEN <= RECORD_CRC_OFFSET);

pub struct TrustCenterDeviceJournal<S> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedState>,
    scanned: bool,
}

#[derive(Clone)]
struct LocatedState {
    generation: u32,
    sector: usize,
    state: PersistentTrustCenterState,
}

impl<S: NorFlash> TrustCenterDeviceJournal<S> {
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        Self {
            storage,
            sectors: [first_sector, second_sector],
            cached: None,
            scanned: false,
        }
    }

    pub fn storage(&self) -> &S {
        &self.storage
    }

    pub fn into_storage(self) -> S {
        self.storage
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0] != self.sectors[1]
            && self.sectors[0].abs_diff(self.sectors[1]) >= TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && TRUST_CENTER_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            && TRUST_CENTER_JOURNAL_SLOT_SIZE.is_multiple_of(S::WRITE_SIZE)
            && TRUST_CENTER_JOURNAL_SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            && RECORD_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            && self
                .sectors
                .iter()
                .all(|sector| (*sector as usize).is_multiple_of(S::ERASE_SIZE))
            && self.sectors.iter().all(|sector| {
                (*sector as usize)
                    .checked_add(TRUST_CENTER_JOURNAL_SECTOR_SIZE)
                    .is_some_and(|end| end <= self.storage.capacity())
            })
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; TRUST_CENTER_JOURNAL_SLOT_SIZE],
    ) -> Result<(), TrustCenterStoreError> {
        self.storage
            .read(
                self.sectors[sector] + (slot * TRUST_CENTER_JOURNAL_SLOT_SIZE) as u32,
                output,
            )
            .map_err(|_| TrustCenterStoreError::Hardware)
    }

    fn decode_record(
        record: &[u8; TRUST_CENTER_JOURNAL_SLOT_SIZE],
    ) -> Option<(u32, PersistentTrustCenterState)> {
        if record[0..4] != RECORD_MAGIC
            || !matches!(
                record[4],
                LEGACY_RECORD_VERSION
                    | ROTATION_RECORD_VERSION
                    | UNKNOWN_REMOVAL_RECORD_VERSION
                    | RECORD_VERSION
            )
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if encoded_len > MAX_ENCODED_TRUST_CENTER_STATE_LEN
            || RECORD_ENCODED_OFFSET + encoded_len > RECORD_CRC_OFFSET
        {
            return None;
        }
        let expected_crc = u32::from_le_bytes([
            record[RECORD_CRC_OFFSET],
            record[RECORD_CRC_OFFSET + 1],
            record[RECORD_CRC_OFFSET + 2],
            record[RECORD_CRC_OFFSET + 3],
        ]);
        if crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]) != expected_crc {
            return None;
        }
        let generation = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
        let encoded = &record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len];
        let state = match record[4] {
            RECORD_VERSION => PersistentTrustCenterState::decode(encoded),
            UNKNOWN_REMOVAL_RECORD_VERSION => PersistentTrustCenterState::decode_v3(encoded),
            ROTATION_RECORD_VERSION => PersistentTrustCenterState::decode_v2(encoded),
            LEGACY_RECORD_VERSION => PersistentTrustCenterState::decode_v1(encoded),
            _ => return None,
        }
        .ok()?;
        Some((generation, state))
    }

    fn newest(&mut self) -> Result<Option<LocatedState>, TrustCenterStoreError> {
        let mut newest: Option<LocatedState> = None;
        let mut record = [0u8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        for sector in 0..2 {
            for slot in 0..TRUST_CENTER_JOURNAL_SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, state)) = Self::decode_record(&record) else {
                    continue;
                };
                if newest
                    .as_ref()
                    .is_none_or(|current| generation > current.generation)
                {
                    newest = Some(LocatedState {
                        generation,
                        sector,
                        state,
                    });
                }
            }
        }
        Ok(newest)
    }

    fn current(&mut self) -> Result<Option<LocatedState>, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        if !self.scanned {
            self.cached = self.newest()?;
            self.scanned = true;
        }
        Ok(self.cached.clone())
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, TrustCenterStoreError> {
        let mut record = [0u8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        for slot in 0..TRUST_CENTER_JOURNAL_SLOTS_PER_SECTOR {
            self.read_slot(sector, slot, &mut record)?;
            if record.iter().all(|byte| *byte == 0xFF) {
                return Ok(Some(slot));
            }
        }
        Ok(None)
    }

    fn write_record(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
        state: &PersistentTrustCenterState,
    ) -> Result<(), TrustCenterStoreError> {
        state.validate()?;
        let mut record = [0xFFu8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let encoded_len = state.encode(&mut encoded);
        record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len]
            .copy_from_slice(&encoded[..encoded_len]);
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.sectors[sector] + (slot * TRUST_CENTER_JOURNAL_SLOT_SIZE) as u32;
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| TrustCenterStoreError::Hardware)?;
        self.storage
            .write(address + RECORD_COMMIT_OFFSET as u32, &RECORD_COMMIT)
            .map_err(|_| TrustCenterStoreError::Hardware)?;

        let mut verify = [0u8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        match Self::decode_record(&verify) {
            Some((stored_generation, stored_state))
                if stored_generation == generation && stored_state == *state =>
            {
                Ok(())
            }
            _ => Err(TrustCenterStoreError::Hardware),
        }
    }

    fn cache_result(
        &mut self,
        result: &Result<(), TrustCenterStoreError>,
        generation: u32,
        sector: usize,
        state: &PersistentTrustCenterState,
    ) {
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector,
                state: state.clone(),
            });
        } else {
            self.cached = None;
            self.scanned = false;
        }
    }
}

impl<S: NorFlash> TrustCenterDeviceStore for TrustCenterDeviceJournal<S> {
    fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
        Ok(self.current()?.map(|located| located.state))
    }

    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        let current = self.current()?;
        let generation = match &current {
            Some(located) => located
                .generation
                .checked_add(1)
                .ok_or(TrustCenterStoreError::GenerationExhausted)?,
            None => 0,
        };

        if let Some(located) = current {
            if let Some(slot) = self.first_erased_slot(located.sector)? {
                let result = self.write_record(located.sector, slot, generation, state);
                self.cache_result(&result, generation, located.sector, state);
                return result;
            }
            let target = 1 - located.sector;
            let sector = self.sectors[target];
            let result = self
                .storage
                .erase(sector, sector + TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| TrustCenterStoreError::Hardware)
                .and_then(|()| self.write_record(target, 0, generation, state));
            self.cache_result(&result, generation, target, state);
            return result;
        }

        let sector = self.sectors[0];
        let result = self
            .storage
            .erase(sector, sector + TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32)
            .map_err(|_| TrustCenterStoreError::Hardware)
            .and_then(|()| self.write_record(0, 0, generation, state));
        self.cache_result(&result, generation, 0, state);
        result
    }

    fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        for sector in self.sectors {
            self.storage
                .erase(sector, sector + TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| TrustCenterStoreError::Hardware)?;
        }
        self.cached = None;
        self.scanned = true;
        Ok(())
    }
}

fn encode_origin(origin: TrustCenterKeyOrigin) -> u8 {
    match origin {
        TrustCenterKeyOrigin::DefaultGlobal => 0,
        TrustCenterKeyOrigin::InstallCode => 1,
        TrustCenterKeyOrigin::ApplicationProvisioned => 2,
        TrustCenterKeyOrigin::GeneratedUnique => 3,
    }
}

fn decode_origin(value: u8) -> Result<TrustCenterKeyOrigin, TrustCenterStoreError> {
    match value {
        0 => Ok(TrustCenterKeyOrigin::DefaultGlobal),
        1 => Ok(TrustCenterKeyOrigin::InstallCode),
        2 => Ok(TrustCenterKeyOrigin::ApplicationProvisioned),
        3 => Ok(TrustCenterKeyOrigin::GeneratedUnique),
        _ => Err(TrustCenterStoreError::Corrupt),
    }
}

fn decode_attributes(value: u8) -> Result<TrustCenterKeyAttributes, TrustCenterStoreError> {
    match value {
        0 => Ok(TrustCenterKeyAttributes::Provisional),
        1 => Ok(TrustCenterKeyAttributes::Unverified),
        2 => Ok(TrustCenterKeyAttributes::Verified),
        _ => Err(TrustCenterStoreError::Corrupt),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    const EPID: IeeeAddress = [0x11; 8];
    const DEVICE: IeeeAddress = [0x22; 8];
    const PARENT: IeeeAddress = [0x33; 8];

    fn state() -> PersistentTrustCenterState {
        let mut state = PersistentTrustCenterState::new(EPID);
        state
            .upsert(PersistentTrustCenterDevice {
                device: TrustCenterDevice {
                    ieee_address: DEVICE,
                    parent_address: PARENT,
                    short_address: ShortAddress(0x1234),
                    link_key: [0x44; 16],
                    key_origin: TrustCenterKeyOrigin::GeneratedUnique,
                    key_attributes: TrustCenterKeyAttributes::Verified,
                    join_timeout_remaining_secs: 0,
                    active: true,
                },
                outgoing_frame_counter: 0x1000,
                outgoing_frame_counter_limit: 0x1400,
                incoming_frame_counter: 0x200,
                incoming_frame_counter_valid: true,
                network_key_pending: false,
                confirm_key_pending: None,
                removal_pending: false,
                new_network_key_delivered: false,
                is_router: None,
                pending_link_key: None,
            })
            .unwrap();
        state
    }

    #[test]
    fn snapshot_round_trip_preserves_device_security_state() {
        let state = state();
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);
        assert_eq!(
            PersistentTrustCenterState::decode(&encoded[..len]).unwrap(),
            state
        );
        assert_eq!(
            PersistentTrustCenterState::decode_v1(
                &encoded[..len - ROTATION_TRAILER_LEN - UNKNOWN_REMOVAL_HEADER_LEN]
            )
            .unwrap(),
            state,
            "the version-1 snapshot layout remains readable"
        );
    }

    #[test]
    fn router_capability_round_trips_without_corrupting_confirm_key_status() {
        for is_router in [None, Some(false), Some(true)] {
            for status in [None, Some(0x00), Some(0xAD)] {
                let mut state = state();
                let device = state.device_mut(&DEVICE).unwrap();
                device.is_router = is_router;
                device.confirm_key_pending = status;
                let mut encoded = [0; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
                let len = state.encode(&mut encoded);
                assert_eq!(
                    PersistentTrustCenterState::decode(&encoded[..len]),
                    Ok(state)
                );
            }
        }
    }

    #[test]
    fn previous_journal_does_not_infer_an_end_device_from_unknown_capability() {
        let state = state();
        let mut encoded = [0; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);
        let decoded = PersistentTrustCenterState::decode_v3(&encoded[..len]).unwrap();
        assert_eq!(decoded.device(&DEVICE).unwrap().is_router, None);
        encoded[STATE_HEADER_LEN + 37] |= 0x80;
        assert_eq!(
            PersistentTrustCenterState::decode_v3(&encoded[..len]),
            Err(TrustCenterStoreError::Corrupt)
        );
    }

    #[test]
    fn snapshot_round_trip_preserves_directional_application_key_allowlist() {
        let mut state = state();
        state
            .allow_application_key_request(DEVICE, [0x55; 8])
            .unwrap();
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);

        let decoded = PersistentTrustCenterState::decode(&encoded[..len]).unwrap();
        assert!(decoded.application_key_request_allowed(&DEVICE, &[0x55; 8]));
        assert!(!decoded.application_key_request_allowed(&[0x55; 8], &DEVICE));
    }

    #[test]
    fn snapshot_round_trip_preserves_unicast_rotation_progress() {
        let mut state = state();
        state.set_rotation(Some(NetworkKeyRotation {
            phase: NetworkKeyRotationPhase::Distributing,
            method: NetworkKeyUpdateMethod::Unicast,
            target_sequence: 0,
        }));
        state.mark_network_key_delivered(&DEVICE).unwrap();
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);

        assert_eq!(
            PersistentTrustCenterState::decode(&encoded[..len]).unwrap(),
            state
        );
        encoded[STATE_HEADER_LEN + 37] &= !0x40;
        assert!(
            !PersistentTrustCenterState::decode_v1(
                &encoded[..len - ROTATION_TRAILER_LEN - UNKNOWN_REMOVAL_HEADER_LEN]
            )
            .unwrap()
            .device(&DEVICE)
            .unwrap()
            .new_network_key_delivered,
            "version-1 records never reinterpret the formerly reserved flag bit"
        );
    }

    #[test]
    fn snapshot_rejects_rotation_progress_without_a_unicast_transaction() {
        let state = state();
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);
        encoded[STATE_HEADER_LEN + 37] |= 0x40;
        assert_eq!(
            PersistentTrustCenterState::decode(&encoded[..len]),
            Err(TrustCenterStoreError::Corrupt)
        );
    }

    #[test]
    fn ram_store_rejects_invalid_counter_ranges_and_clears_atomically() {
        let mut invalid = state();
        let device = invalid.device_mut(&DEVICE).unwrap();
        device.outgoing_frame_counter = device.outgoing_frame_counter_limit;
        let mut store = RamTrustCenterDeviceStore::new();
        assert_eq!(store.store(&invalid), Err(TrustCenterStoreError::Corrupt));

        let valid = state();
        store.store(&valid).unwrap();
        assert_eq!(store.load().unwrap(), Some(valid));
        store.clear().unwrap();
        assert!(store.load().unwrap().is_none());
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockError;

    impl embedded_storage::nor_flash::NorFlashError for MockError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    #[derive(Clone)]
    struct MockFlash {
        bytes: [u8; TRUST_CENTER_JOURNAL_SECTOR_SIZE * 2],
        programs_before_failure: Option<usize>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: [0xFF; TRUST_CENTER_JOURNAL_SECTOR_SIZE * 2],
                programs_before_failure: None,
            }
        }

        fn legacy_record(
            state: &PersistentTrustCenterState,
            version: u8,
        ) -> [u8; TRUST_CENTER_JOURNAL_SLOT_SIZE] {
            let mut record = [0xFFu8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
            record[0..4].copy_from_slice(&RECORD_MAGIC);
            record[4] = version;
            let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
            let current_len = state.encode(&mut encoded);
            let encoded_len = current_len
                - if version < UNKNOWN_REMOVAL_RECORD_VERSION {
                    UNKNOWN_REMOVAL_HEADER_LEN
                } else {
                    0
                }
                - if version == LEGACY_RECORD_VERSION {
                    ROTATION_TRAILER_LEN
                } else {
                    0
                };
            record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
            record[7] = 0;
            record[8..12].copy_from_slice(&1u32.to_le_bytes());
            record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len]
                .copy_from_slice(&encoded[..encoded_len]);
            let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
            record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
            record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
            record
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start.checked_add(bytes.len()).ok_or(MockError)?;
            let source = self.bytes.get(start..end).ok_or(MockError)?;
            bytes.copy_from_slice(source);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = TRUST_CENTER_JOURNAL_SECTOR_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let range = self
                .bytes
                .get_mut(from as usize..to as usize)
                .ok_or(MockError)?;
            range.fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if let Some(remaining) = self.programs_before_failure.as_mut() {
                if *remaining == 0 {
                    return Err(MockError);
                }
                *remaining -= 1;
            }
            let start = offset as usize;
            let end = start.checked_add(bytes.len()).ok_or(MockError)?;
            let destination = self.bytes.get_mut(start..end).ok_or(MockError)?;
            for (dst, src) in destination.iter_mut().zip(bytes) {
                if (*dst & *src) != *src {
                    return Err(MockError);
                }
                *dst &= *src;
            }
            Ok(())
        }
    }

    #[test]
    fn journal_rollover_restores_the_latest_committed_generation() {
        let mut journal = TrustCenterDeviceJournal::new(
            MockFlash::new(),
            0,
            TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut expected = state();
        for counter in 0x1000..0x1005 {
            let device = expected.device_mut(&DEVICE).unwrap();
            device.outgoing_frame_counter = counter;
            device.outgoing_frame_counter_limit = counter + 0x400;
            journal.store(&expected).unwrap();
        }

        let flash = journal.into_storage();
        let mut reopened =
            TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn journal_migrates_previous_versions_and_rewrites_current_version() {
        for version in [
            LEGACY_RECORD_VERSION,
            ROTATION_RECORD_VERSION,
            UNKNOWN_REMOVAL_RECORD_VERSION,
        ] {
            let expected = state();
            let mut flash = MockFlash::new();
            flash.bytes[..TRUST_CENTER_JOURNAL_SLOT_SIZE]
                .copy_from_slice(&MockFlash::legacy_record(&expected, version));
            let mut journal =
                TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
            assert_eq!(journal.load(), Ok(Some(expected.clone())));

            let mut updated = expected;
            updated.device_mut(&DEVICE).unwrap().incoming_frame_counter = 0x345;
            journal.store(&updated).unwrap();
            let flash = journal.into_storage();
            assert_eq!(
                flash.bytes[TRUST_CENTER_JOURNAL_SECTOR_SIZE + 4],
                RECORD_VERSION
            );

            let mut reopened =
                TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
            assert_eq!(reopened.load(), Ok(Some(updated)));
        }
    }

    #[test]
    fn interrupted_commit_preserves_the_previous_device_database() {
        let mut journal = TrustCenterDeviceJournal::new(
            MockFlash::new(),
            0,
            TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
        );
        let expected = state();
        journal.store(&expected).unwrap();

        let mut flash = journal.into_storage();
        flash.programs_before_failure = Some(1);
        let mut interrupted =
            TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        let mut replacement = expected.clone();
        replacement
            .device_mut(&DEVICE)
            .unwrap()
            .incoming_frame_counter = 0x333;
        assert_eq!(
            interrupted.store(&replacement),
            Err(TrustCenterStoreError::Hardware)
        );

        let flash = interrupted.into_storage();
        let mut reopened =
            TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn unknown_removal_round_trip_is_bounded_and_never_creates_device_keys() {
        let mut state = state();
        for index in 0..MAX_TRUST_CENTER_DEVICES {
            let mut address = [0x99; 8];
            address[0] = index as u8;
            state
                .stage_unknown_removal(PendingUnknownRemoval {
                    parent_address: PARENT,
                    device_address: address,
                })
                .unwrap();
        }
        let mut bytes = [0; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut bytes);
        assert_eq!(
            PersistentTrustCenterState::decode(&bytes[..len]),
            Ok(state.clone())
        );
        assert_eq!(state.devices().len(), 1);
        assert_eq!(
            state.pending_unknown_removals().len(),
            MAX_TRUST_CENTER_DEVICES
        );
        let before = state.clone();
        assert_eq!(
            state.stage_unknown_removal(PendingUnknownRemoval {
                parent_address: PARENT,
                device_address: [0x88; 8],
            }),
            Err(TrustCenterStoreError::Full)
        );
        assert_eq!(
            state, before,
            "capacity failure must not silently drop or replace an intent"
        );
        let pending = state.pending_unknown_removals()[0];
        state
            .stage_unknown_removal(PendingUnknownRemoval {
                parent_address: [0x77; 8],
                ..pending
            })
            .unwrap();
        assert_eq!(
            state.pending_unknown_removals().len(),
            MAX_TRUST_CENTER_DEVICES
        );
        assert!(state.device(&pending.device_address).is_none());
    }

    #[test]
    fn unknown_removal_decoder_rejects_truncation_duplicates_and_invalid_addresses() {
        let mut state = state();
        for device_address in [[0x55; 8], [0x66; 8]] {
            state
                .stage_unknown_removal(PendingUnknownRemoval {
                    parent_address: PARENT,
                    device_address,
                })
                .unwrap();
        }
        let mut bytes = [0; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut bytes);
        let count_offset = len - UNKNOWN_REMOVAL_HEADER_LEN - 2 * UNKNOWN_REMOVAL_ENTRY_LEN;
        let first = count_offset + UNKNOWN_REMOVAL_HEADER_LEN;
        assert_eq!(
            PersistentTrustCenterState::decode(&bytes[..len - 1]),
            Err(TrustCenterStoreError::Corrupt)
        );
        assert_eq!(
            PersistentTrustCenterState::decode(&bytes[..len + 1]),
            Err(TrustCenterStoreError::Corrupt)
        );
        let mut bad = bytes;
        bad[count_offset] = (MAX_TRUST_CENTER_DEVICES + 1) as u8;
        assert_eq!(
            PersistentTrustCenterState::decode(&bad[..len]),
            Err(TrustCenterStoreError::Corrupt)
        );
        for bad_address in [[0; 8], [0xFF; 8], DEVICE, [0x66; 8]] {
            let mut bad = bytes;
            bad[first + 8..first + 16].copy_from_slice(&bad_address);
            assert_eq!(
                PersistentTrustCenterState::decode(&bad[..len]),
                Err(TrustCenterStoreError::Corrupt)
            );
        }
        let mut bad = bytes;
        bad[first..first + 8].fill(0);
        assert_eq!(
            PersistentTrustCenterState::decode(&bad[..len]),
            Err(TrustCenterStoreError::Corrupt)
        );
    }

    #[test]
    fn unknown_removal_power_cuts_preserve_admission_and_completion_boundaries() {
        let empty = state();
        let mut pending = empty.clone();
        pending
            .stage_unknown_removal(PendingUnknownRemoval {
                parent_address: PARENT,
                device_address: [0x99; 8],
            })
            .unwrap();
        for (before, after) in [(&empty, &pending), (&pending, &empty)] {
            let mut initial = TrustCenterDeviceJournal::new(
                MockFlash::new(),
                0,
                TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
            );
            initial.store(before).unwrap();
            let flash = initial.into_storage();
            for programs in 0..=2 {
                let mut faulty = flash.clone();
                faulty.programs_before_failure = Some(programs);
                let mut journal = TrustCenterDeviceJournal::new(
                    faulty,
                    0,
                    TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
                );
                let result = journal.store(after);
                let expected = if programs < 2 {
                    assert_eq!(result, Err(TrustCenterStoreError::Hardware));
                    before
                } else {
                    assert_eq!(result, Ok(()));
                    after
                };
                let mut rebooted = TrustCenterDeviceJournal::new(
                    journal.into_storage(),
                    0,
                    TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
                );
                assert_eq!(rebooted.load(), Ok(Some(expected.clone())));
            }
        }
    }

    #[test]
    fn journal_checks_crc_and_refuses_generation_wraparound() {
        let mut journal = TrustCenterDeviceJournal::new(
            MockFlash::new(),
            0,
            TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
        );
        assert_eq!(journal.load(), Ok(None), "erased journal");
        let expected = state();
        journal.store(&expected).unwrap();
        let flash = journal.into_storage();
        let mut bad_crc = flash.clone();
        bad_crc.bytes[RECORD_ENCODED_OFFSET] ^= 1;
        let mut corrupt =
            TrustCenterDeviceJournal::new(bad_crc, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(
            corrupt.load(),
            Ok(None),
            "an invalid CRC cannot restore keys or intents"
        );

        let mut exhausted = flash;
        exhausted.bytes[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        let crc = crate::security_journal::crc32(&exhausted.bytes[..RECORD_CRC_OFFSET]);
        exhausted.bytes[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4]
            .copy_from_slice(&crc.to_le_bytes());
        let before = exhausted.bytes;
        let mut journal =
            TrustCenterDeviceJournal::new(exhausted, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(Some(expected.clone())));
        assert_eq!(
            journal.store(&expected),
            Err(TrustCenterStoreError::GenerationExhausted)
        );
        assert_eq!(
            journal.storage().bytes,
            before,
            "exhaustion must not erase the last valid snapshot"
        );
    }
}
