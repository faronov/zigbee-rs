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
/// Parent-directed removal intents for rejected (unknown) devices. The
/// coordinator NCP (`ncp-capacity`) bounds them at 32 (ZRHP spec 8.9);
/// other builds keep one per Trust Center device slot.
#[cfg(not(feature = "ncp-capacity"))]
pub const MAX_PENDING_UNKNOWN_REMOVALS: usize = MAX_TRUST_CENTER_DEVICES;
#[cfg(feature = "ncp-capacity")]
pub const MAX_PENDING_UNKNOWN_REMOVALS: usize = 32;
const _: () = assert!(MAX_PENDING_UNKNOWN_REMOVALS <= u8::MAX as usize);
const _: () = assert!(MAX_APPLICATION_KEY_REQUESTS <= u8::MAX as usize);
/// Trailer the current record version appends to the version-4 encoding:
/// the durable global outgoing APS counter limit (version 5,
/// `global-aps-counter`), preceded by the high byte of the device count
/// (version 6, `ncp-capacity`: up to 256 devices).
#[cfg(all(feature = "global-aps-counter", not(feature = "ncp-capacity")))]
const GLOBAL_APS_TRAILER_LEN: usize = 4;
#[cfg(feature = "ncp-capacity")]
const GLOBAL_APS_TRAILER_LEN: usize = 5;
#[cfg(not(feature = "global-aps-counter"))]
const GLOBAL_APS_TRAILER_LEN: usize = 0;
#[cfg(not(feature = "ncp-capacity"))]
const _: () = assert!(MAX_TRUST_CENTER_DEVICES <= u8::MAX as usize);
#[cfg(feature = "ncp-capacity")]
const _: () = assert!(MAX_TRUST_CENTER_DEVICES <= u16::MAX as usize);
pub const MAX_ENCODED_TRUST_CENTER_STATE_LEN: usize = STATE_HEADER_LEN
    + MAX_TRUST_CENTER_DEVICES * DEVICE_ENCODED_LEN
    + 1
    + MAX_APPLICATION_KEY_REQUESTS * APPLICATION_KEY_REQUEST_ENTRY_LEN
    + ROTATION_TRAILER_LEN
    + UNKNOWN_REMOVAL_HEADER_LEN
    + MAX_PENDING_UNKNOWN_REMOVALS * UNKNOWN_REMOVAL_ENTRY_LEN
    + GLOBAL_APS_TRAILER_LEN;

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
    pending_unknown_removals: Vec<PendingUnknownRemoval, MAX_PENDING_UNKNOWN_REMOVALS>,
    /// Exclusive, durably reserved limit of the global outgoing APS counter
    /// (`global-aps-counter`); 0 = never reserved (a record written before
    /// the migration, version <= 4).
    #[cfg(feature = "global-aps-counter")]
    global_aps_limit: u32,
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
            #[cfg(feature = "global-aps-counter")]
            global_aps_limit: 0,
        }
    }

    /// [`Self::new`] directly into caller-provided storage (no whole-state
    /// temporary on the stack).
    ///
    /// # Safety
    /// `slot` must point to valid, properly aligned, uninitialized storage for `Self`.
    pub unsafe fn write_into(slot: *mut Self, extended_pan_id: IeeeAddress) {
        // Every field is written below; a new field must be added there.
        let _exhaustive = |state: &Self| {
            let Self {
                extended_pan_id: _,
                pending_application_key: _,
                rotation: _,
                application_key_requests: _,
                devices: _,
                pending_unknown_removals: _,
                #[cfg(feature = "global-aps-counter")]
                    global_aps_limit: _,
            } = state;
        };
        unsafe {
            core::ptr::addr_of_mut!((*slot).extended_pan_id).write(extended_pan_id);
            core::ptr::addr_of_mut!((*slot).pending_application_key).write(None);
            core::ptr::addr_of_mut!((*slot).rotation).write(None);
            core::ptr::addr_of_mut!((*slot).application_key_requests).write(Vec::new());
            core::ptr::addr_of_mut!((*slot).devices).write(Vec::new());
            core::ptr::addr_of_mut!((*slot).pending_unknown_removals).write(Vec::new());
            #[cfg(feature = "global-aps-counter")]
            core::ptr::addr_of_mut!((*slot).global_aps_limit).write(0);
        }
    }

    /// Durable limit of the global outgoing APS counter (0 = none yet).
    #[cfg(feature = "global-aps-counter")]
    pub const fn global_aps_limit(&self) -> u32 {
        self.global_aps_limit
    }

    /// Record a new durable global APS limit; it never moves backwards.
    #[cfg(feature = "global-aps-counter")]
    pub fn set_global_aps_limit(&mut self, limit: u32) -> bool {
        if limit < self.global_aps_limit {
            return false;
        }
        self.global_aps_limit = limit;
        true
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

    /// Encoded length of [`Self::encode`] for this state.
    pub fn encoded_len(&self) -> usize {
        STATE_HEADER_LEN
            + self.devices.len() * DEVICE_ENCODED_LEN
            + 1
            + self.application_key_requests.len() * APPLICATION_KEY_REQUEST_ENTRY_LEN
            + ROTATION_TRAILER_LEN
            + UNKNOWN_REMOVAL_HEADER_LEN
            + self.pending_unknown_removals.len() * UNKNOWN_REMOVAL_ENTRY_LEN
            + GLOBAL_APS_TRAILER_LEN
    }

    pub fn encode(&self, output: &mut [u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN]) -> usize {
        output.fill(0);
        let mut sink = SliceSink::new(output);
        // The bounded tables never encode past `MAX_ENCODED_...`.
        self.encode_into(&mut sink).unwrap_or_default()
    }

    /// Stream the current record version's encoding into `sink` (the
    /// streamed multi-slot journal encodes without a full-size buffer).
    pub(crate) fn encode_into(
        &self,
        sink: &mut impl SnapshotSink,
    ) -> Result<usize, TrustCenterStoreError> {
        let mut header = [0u8; STATE_HEADER_LEN];
        header[0..8].copy_from_slice(&self.extended_pan_id);
        // Low byte; version 6 appends the high byte (see the trailer below).
        header[8] = self.devices.len() as u8;
        if let Some(pending) = self.pending_application_key {
            header[9] = 1;
            header[10..18].copy_from_slice(&pending.initiator_address);
            header[18..26].copy_from_slice(&pending.responder_address);
            header[26..42].copy_from_slice(&pending.key);
            header[42] = u8::from(pending.delivered_to_initiator)
                | (u8::from(pending.delivered_to_responder) << 1);
        }
        sink.put(&header)?;
        for stored in &self.devices {
            sink.put(&encode_device(stored))?;
        }
        sink.put(&[self.application_key_requests.len() as u8])?;
        for entry in &self.application_key_requests {
            let mut encoded = [0u8; APPLICATION_KEY_REQUEST_ENTRY_LEN];
            encoded[0..8].copy_from_slice(&entry.initiator_address);
            encoded[8..16].copy_from_slice(&entry.responder_address);
            sink.put(&encoded)?;
        }
        let mut rotation = [0u8; ROTATION_TRAILER_LEN];
        if let Some(active) = self.rotation {
            rotation = [
                active.phase as u8,
                active.target_sequence,
                active.method as u8,
            ];
        }
        sink.put(&rotation)?;
        sink.put(&[self.pending_unknown_removals.len() as u8])?;
        for pending in &self.pending_unknown_removals {
            let mut encoded = [0u8; UNKNOWN_REMOVAL_ENTRY_LEN];
            encoded[0..8].copy_from_slice(&pending.parent_address);
            encoded[8..16].copy_from_slice(&pending.device_address);
            sink.put(&encoded)?;
        }
        #[cfg(feature = "ncp-capacity")]
        sink.put(&[(self.devices.len() >> 8) as u8])?;
        #[cfg(feature = "global-aps-counter")]
        sink.put(&self.global_aps_limit.to_le_bytes())?;
        Ok(self.encoded_len())
    }

    /// Decode the current record version's encoding (by value; host and
    /// test convenience; embedded paths use [`Self::decode_into`]).
    pub fn decode(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        let mut state = Self::new([0; 8]);
        state.decode_into(bytes)?;
        Ok(state)
    }

    /// Decode the current record version's encoding into `self` in place,
    /// without a whole-state temporary. On error `self` holds an
    /// unspecified (partially decoded) state.
    pub fn decode_into(&mut self, bytes: &[u8]) -> Result<(), TrustCenterStoreError> {
        self.decode_from_into(&mut SliceSource::new(bytes))
    }

    /// Decode the current record version's encoding from `source` into
    /// `self` in place (see [`Self::decode_into`]).
    pub(crate) fn decode_from_into(
        &mut self,
        source: &mut impl SnapshotSource,
    ) -> Result<(), TrustCenterStoreError> {
        self.decode_version_into(source, Layout::CURRENT)
    }

    /// Decode a record of `version`'s layout into `self` in place; `None`
    /// for a version this build cannot read.
    pub(crate) fn decode_record_version_into(
        &mut self,
        version: u8,
        bytes: &[u8],
    ) -> Option<Result<(), TrustCenterStoreError>> {
        let layout = match version {
            RECORD_VERSION => Layout::CURRENT,
            #[cfg(feature = "ncp-capacity")]
            GLOBAL_APS_RECORD_VERSION => Layout::V5,
            #[cfg(feature = "global-aps-counter")]
            GLOBAL_APS_RECORD_VERSION_PREDECESSOR => Layout::V4,
            UNKNOWN_REMOVAL_RECORD_VERSION => Layout::V3,
            ROTATION_RECORD_VERSION => Layout::V2,
            LEGACY_RECORD_VERSION => Layout::V1,
            _ => return None,
        };
        Some(self.decode_version_into(&mut SliceSource::new(bytes), layout))
    }

    #[cfg(test)]
    fn decode_layout(bytes: &[u8], layout: Layout) -> Result<Self, TrustCenterStoreError> {
        let mut state = Self::new([0; 8]);
        state.decode_version_into(&mut SliceSource::new(bytes), layout)?;
        Ok(state)
    }

    /// Version 4: no global APS limit (migrated on the first global-mode
    /// start).
    #[cfg(all(test, feature = "global-aps-counter"))]
    fn decode_v4(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_layout(bytes, Layout::V4)
    }

    #[cfg(test)]
    fn decode_v3(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_layout(bytes, Layout::V3)
    }

    #[cfg(test)]
    fn decode_v1(bytes: &[u8]) -> Result<Self, TrustCenterStoreError> {
        Self::decode_layout(bytes, Layout::V1)
    }

    /// Reset `self` in place to the empty state of `extended_pan_id`.
    pub fn reset(&mut self, extended_pan_id: IeeeAddress) {
        self.extended_pan_id = extended_pan_id;
        self.pending_application_key = None;
        self.rotation = None;
        self.application_key_requests.clear();
        self.devices.clear();
        self.pending_unknown_removals.clear();
        #[cfg(feature = "global-aps-counter")]
        {
            self.global_aps_limit = 0;
        }
    }

    /// Overwrite `self` with `other` element by element (no whole-state
    /// temporary, unlike `*self = other.clone()`).
    pub fn copy_from(&mut self, other: &Self) {
        self.reset(other.extended_pan_id);
        self.pending_application_key = other.pending_application_key;
        self.rotation = other.rotation;
        // Same capacities: these cannot fail.
        let _ = self
            .application_key_requests
            .extend_from_slice(&other.application_key_requests);
        let _ = self.devices.extend_from_slice(&other.devices);
        let _ = self
            .pending_unknown_removals
            .extend_from_slice(&other.pending_unknown_removals);
        #[cfg(feature = "global-aps-counter")]
        {
            self.global_aps_limit = other.global_aps_limit;
        }
    }

    /// Decode into `self` in place: every field is overwritten, nothing of
    /// the previous contents survives a successful decode.
    fn decode_version_into(
        &mut self,
        source: &mut impl SnapshotSource,
        layout: Layout,
    ) -> Result<(), TrustCenterStoreError> {
        let total = source.remaining();
        let trailer_len = usize::from(layout.count_high) + if layout.global_limit { 4 } else { 0 };
        let body_len = total
            .checked_sub(trailer_len)
            .ok_or(TrustCenterStoreError::Corrupt)?;
        let count_high = if layout.count_high {
            source.byte_at(body_len)?
        } else {
            0
        };
        let mut header = [0u8; STATE_HEADER_LEN];
        if body_len < STATE_HEADER_LEN {
            return Err(TrustCenterStoreError::Corrupt);
        }
        source.take(&mut header)?;
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(&header[0..8]);
        let count = usize::from(header[8]) | (usize::from(count_high) << 8);
        let device_bytes_len = STATE_HEADER_LEN + count * DEVICE_ENCODED_LEN;
        if count > MAX_TRUST_CENTER_DEVICES || body_len < device_bytes_len {
            return Err(TrustCenterStoreError::Corrupt);
        }
        self.reset(extended_pan_id);
        let state = self;
        state.pending_application_key = match header[9] {
            0 => {
                if header[10..43].iter().any(|byte| *byte != 0) {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                None
            }
            1 => {
                if header[42] & !0x03 != 0 {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                let mut initiator_address = [0u8; 8];
                initiator_address.copy_from_slice(&header[10..18]);
                let mut responder_address = [0u8; 8];
                responder_address.copy_from_slice(&header[18..26]);
                let mut key = [0u8; 16];
                key.copy_from_slice(&header[26..42]);
                Some(PendingApplicationLinkKey {
                    initiator_address,
                    responder_address,
                    key,
                    delivered_to_initiator: header[42] & 0x01 != 0,
                    delivered_to_responder: header[42] & 0x02 != 0,
                })
            }
            _ => return Err(TrustCenterStoreError::Corrupt),
        };
        let mut encoded = [0u8; DEVICE_ENCODED_LEN];
        for _ in 0..count {
            source.take(&mut encoded)?;
            state
                .devices
                .push(decode_device(&encoded, layout)?)
                .map_err(|_| TrustCenterStoreError::Full)?;
        }
        // Body bytes after the device records.
        let rest = body_len - device_bytes_len;
        if layout.rotation {
            if rest == 0 {
                return Err(TrustCenterStoreError::Corrupt);
            }
            let allow_count = usize::from(source.byte()?);
            let removal_offset = 1 + allow_count * APPLICATION_KEY_REQUEST_ENTRY_LEN + 3;
            if allow_count > MAX_APPLICATION_KEY_REQUESTS
                || rest < removal_offset
                || (!layout.unknown_removals && rest != removal_offset)
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            state.decode_application_key_requests(source, allow_count)?;
            let mut rotation = [0u8; ROTATION_TRAILER_LEN];
            source.take(&mut rotation)?;
            state.rotation = match rotation[0] {
                0 => {
                    if rotation[1] != 0 || rotation[2] != 0 {
                        return Err(TrustCenterStoreError::Corrupt);
                    }
                    None
                }
                phase => Some(NetworkKeyRotation {
                    target_sequence: rotation[1],
                    phase: NetworkKeyRotationPhase::from_u8(phase)
                        .ok_or(TrustCenterStoreError::Corrupt)?,
                    method: NetworkKeyUpdateMethod::from_u8(rotation[2])
                        .ok_or(TrustCenterStoreError::Corrupt)?,
                }),
            };
            if layout.unknown_removals {
                if rest == removal_offset {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                let removal_count = usize::from(source.byte()?);
                if removal_count > MAX_PENDING_UNKNOWN_REMOVALS
                    || rest
                        != removal_offset
                            + UNKNOWN_REMOVAL_HEADER_LEN
                            + removal_count * UNKNOWN_REMOVAL_ENTRY_LEN
                {
                    return Err(TrustCenterStoreError::Corrupt);
                }
                let mut entry = [0u8; UNKNOWN_REMOVAL_ENTRY_LEN];
                for _ in 0..removal_count {
                    source.take(&mut entry)?;
                    let mut parent_address = [0; 8];
                    parent_address.copy_from_slice(&entry[0..8]);
                    let mut device_address = [0; 8];
                    device_address.copy_from_slice(&entry[8..16]);
                    state
                        .pending_unknown_removals
                        .push(PendingUnknownRemoval {
                            parent_address,
                            device_address,
                        })
                        .map_err(|_| TrustCenterStoreError::Full)?;
                }
            }
        } else if rest != 0 {
            let allow_count = usize::from(source.byte()?);
            if allow_count > MAX_APPLICATION_KEY_REQUESTS
                || rest != 1 + allow_count * APPLICATION_KEY_REQUEST_ENTRY_LEN
            {
                return Err(TrustCenterStoreError::Corrupt);
            }
            state.decode_application_key_requests(source, allow_count)?;
        }
        if layout.count_high {
            // Already applied above (peeked before the devices).
            source.byte()?;
        }
        if layout.global_limit {
            let mut limit = [0u8; 4];
            source.take(&mut limit)?;
            #[cfg(feature = "global-aps-counter")]
            {
                state.global_aps_limit = u32::from_le_bytes(limit);
            }
        }
        if source.remaining() != 0 {
            return Err(TrustCenterStoreError::Corrupt);
        }
        state.validate()
    }

    fn decode_application_key_requests(
        &mut self,
        source: &mut impl SnapshotSource,
        count: usize,
    ) -> Result<(), TrustCenterStoreError> {
        let mut entry = [0u8; APPLICATION_KEY_REQUEST_ENTRY_LEN];
        for _ in 0..count {
            source.take(&mut entry)?;
            let mut initiator_address = [0u8; 8];
            initiator_address.copy_from_slice(&entry[0..8]);
            let mut responder_address = [0u8; 8];
            responder_address.copy_from_slice(&entry[8..16]);
            self.application_key_requests
                .push(ApplicationKeyRequestEntry {
                    initiator_address,
                    responder_address,
                })
                .map_err(|_| TrustCenterStoreError::Full)?;
        }
        Ok(())
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

/// Durable backend of the Trust Center state.
///
/// The state is large (about 23 KiB with `ncp-capacity`), so the backend
/// decodes into caller-owned storage instead of returning it by value.
pub trait TrustCenterDeviceStore {
    /// Decode the newest committed state into `state` in place. `Ok(false)`
    /// when nothing is committed. Unless `Ok(true)` is returned, `state`
    /// holds unspecified contents (a backend may use it as decode scratch)
    /// and the caller must reset or discard it.
    fn load_into(
        &mut self,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError>;
    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError>;
    fn clear(&mut self) -> Result<(), TrustCenterStoreError>;

    /// Erase one more unit of the sector the next rollover will need, ahead
    /// of time (`journal-pre-erase`; see `TrustCenterStreamJournal`). Call it
    /// only when an erase may stall the caller. Returns whether it erased
    /// (call again later); stores without a spare sector do nothing.
    fn pre_erase_step(&mut self) -> Result<bool, TrustCenterStoreError> {
        Ok(false)
    }

    /// The current sector is half used and the spare one is fully
    /// pre-erased: re-storing the current state now (in a quiet window)
    /// rolls over without an erase and leaves a whole free sector plus a
    /// spare for the next burst of stores (a join stores about ten
    /// snapshots). Stores without a spare sector never ask.
    fn wants_compaction(&self) -> bool {
        false
    }

    /// Store `state` into the spare sector even if the current one has room
    /// (see [`Self::wants_compaction`]); same durability as [`Self::store`].
    fn compact(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        self.store(state)
    }

    /// By-value [`Self::load_into`] for hosts and tests. Embedded stack
    /// paths must use [`Self::load_into`]: this returns the whole state on
    /// the stack.
    fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
        let mut state = PersistentTrustCenterState::new([0; 8]);
        Ok(self.load_into(&mut state)?.then_some(state))
    }
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
    fn load_into(
        &mut self,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError> {
        match &self.state {
            Some(stored) => {
                state.copy_from(stored);
                Ok(true)
            }
            None => Ok(false),
        }
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

pub(crate) const RECORD_MAGIC: [u8; 4] = *b"ZBTC";
const LEGACY_RECORD_VERSION: u8 = 1;
const ROTATION_RECORD_VERSION: u8 = 2;
const UNKNOWN_REMOVAL_RECORD_VERSION: u8 = 3;
/// Trust Center journal epoch that production Coordinator rollback policy
/// must not cross after committing this format.
#[cfg(not(feature = "global-aps-counter"))]
pub const TRUST_CENTER_JOURNAL_FORMAT_VERSION: u8 = 4;
/// Version 5 (`global-aps-counter`) appends the global outgoing APS limit.
/// Firmware without the feature does not recognize it and starts with an
/// empty Trust Center table rather than reusing a counter (spec 8.4).
#[cfg(all(feature = "global-aps-counter", not(feature = "ncp-capacity")))]
pub const TRUST_CENTER_JOURNAL_FORMAT_VERSION: u8 = 5;
/// Version 6 (`ncp-capacity`, coordinator NCP) inserts the high byte of a
/// 16-bit device count (up to 256 devices) before the global APS limit and
/// bounds removal intents at [`MAX_PENDING_UNKNOWN_REMOVALS`]. Its full
/// snapshot no longer fits one 4 KiB slot; the NCP stores it with the
/// streamed multi-slot `trust_center_stream::TrustCenterStreamJournal`.
#[cfg(feature = "ncp-capacity")]
pub const TRUST_CENTER_JOURNAL_FORMAT_VERSION: u8 = 6;
#[cfg(feature = "global-aps-counter")]
const GLOBAL_APS_RECORD_VERSION_PREDECESSOR: u8 = 4;
#[cfg(feature = "ncp-capacity")]
const GLOBAL_APS_RECORD_VERSION: u8 = 5;
pub(crate) const RECORD_VERSION: u8 = TRUST_CENTER_JOURNAL_FORMAT_VERSION;
const RECORD_ENCODED_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 12;
const RECORD_PREFIX_LEN: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT_OFFSET: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

/// Bytes of encoded state one single-slot record holds.
const RECORD_ENCODED_CAPACITY: usize = RECORD_CRC_OFFSET - RECORD_ENCODED_OFFSET;
// Without `ncp-capacity` every state fits one slot. The 256-device NCP state
// does not: this journal then refuses (`Full`) a state that does not fit,
// and the NCP uses the streamed journal instead.
#[cfg(not(feature = "ncp-capacity"))]
const _: () = assert!(MAX_ENCODED_TRUST_CENTER_STATE_LEN <= RECORD_ENCODED_CAPACITY);

pub struct TrustCenterDeviceJournal<S> {
    storage: S,
    sectors: [u32; 2],
    /// Location of the current record (the state itself is not cached).
    current: Option<LocatedRecord>,
    scanned: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LocatedRecord {
    generation: u32,
    sector: usize,
    slot: usize,
}

/// Read-back granularity of the write verification.
const RECORD_READBACK: usize = 128;
const _: () = assert!(TRUST_CENTER_JOURNAL_SLOT_SIZE.is_multiple_of(RECORD_READBACK));

impl<S: NorFlash> TrustCenterDeviceJournal<S> {
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        Self {
            storage,
            sectors: [first_sector, second_sector],
            current: None,
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
            && RECORD_READBACK.is_multiple_of(S::READ_SIZE)
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

    fn slot_address(&self, sector: usize, slot: usize) -> u32 {
        self.sectors[sector] + (slot * TRUST_CENTER_JOURNAL_SLOT_SIZE) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; TRUST_CENTER_JOURNAL_SLOT_SIZE],
    ) -> Result<(), TrustCenterStoreError> {
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, output)
            .map_err(|_| TrustCenterStoreError::Hardware)
    }

    /// Generation of a committed record whose framing (magic, known version,
    /// commit marker, length) and CRC are valid, without decoding the state.
    pub(crate) fn frame_record(record: &[u8; TRUST_CENTER_JOURNAL_SLOT_SIZE]) -> Option<u32> {
        #[cfg(feature = "ncp-capacity")]
        let known_version = matches!(
            record[4],
            LEGACY_RECORD_VERSION
                | ROTATION_RECORD_VERSION
                | UNKNOWN_REMOVAL_RECORD_VERSION
                | GLOBAL_APS_RECORD_VERSION_PREDECESSOR
                | GLOBAL_APS_RECORD_VERSION
                | RECORD_VERSION
        );
        #[cfg(all(feature = "global-aps-counter", not(feature = "ncp-capacity")))]
        let known_version = matches!(
            record[4],
            LEGACY_RECORD_VERSION
                | ROTATION_RECORD_VERSION
                | UNKNOWN_REMOVAL_RECORD_VERSION
                | GLOBAL_APS_RECORD_VERSION_PREDECESSOR
                | RECORD_VERSION
        );
        #[cfg(not(feature = "global-aps-counter"))]
        let known_version = matches!(
            record[4],
            LEGACY_RECORD_VERSION
                | ROTATION_RECORD_VERSION
                | UNKNOWN_REMOVAL_RECORD_VERSION
                | RECORD_VERSION
        );
        if record[0..4] != RECORD_MAGIC
            || !known_version
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if encoded_len > MAX_ENCODED_TRUST_CENTER_STATE_LEN || encoded_len > RECORD_ENCODED_CAPACITY
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
        Some(u32::from_le_bytes([
            record[8], record[9], record[10], record[11],
        ]))
    }

    /// Validate and decode one record into `state` in place; its generation,
    /// or `None` (with `state` unspecified) if it is not a complete,
    /// committed, decodable record.
    pub(crate) fn decode_record_into(
        record: &[u8; TRUST_CENTER_JOURNAL_SLOT_SIZE],
        state: &mut PersistentTrustCenterState,
    ) -> Option<u32> {
        let generation = Self::frame_record(record)?;
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        let encoded = &record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len];
        state.decode_record_version_into(record[4], encoded)?.ok()?;
        Some(generation)
    }

    /// Newest committed record by framing and CRC only (no decode), with a
    /// generation below `below` when given.
    fn newest_framed(
        &mut self,
        below: Option<u32>,
    ) -> Result<Option<LocatedRecord>, TrustCenterStoreError> {
        let mut newest: Option<LocatedRecord> = None;
        let mut record = [0u8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        for sector in 0..2 {
            for slot in 0..TRUST_CENTER_JOURNAL_SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some(generation) = Self::frame_record(&record) else {
                    continue;
                };
                if below.is_none_or(|bound| generation < bound)
                    && newest.is_none_or(|current| generation > current.generation)
                {
                    newest = Some(LocatedRecord {
                        generation,
                        sector,
                        slot,
                    });
                }
            }
        }
        Ok(newest)
    }

    /// Decode the record at `located` into `state`; `false` if it no longer
    /// decodes.
    fn decode_at(
        &mut self,
        located: LocatedRecord,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError> {
        let mut record = [0u8; TRUST_CENTER_JOURNAL_SLOT_SIZE];
        self.read_slot(located.sector, located.slot, &mut record)?;
        Ok(Self::decode_record_into(&record, state) == Some(located.generation))
    }

    /// The current record for a store. Without a prior [`Self::load_into`]
    /// scan, the newest committed record by framing and CRC: its generation
    /// is at least that of the newest decodable one, so the next record is
    /// still ordered after every survivor.
    fn current(&mut self) -> Result<Option<LocatedRecord>, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        if !self.scanned {
            self.current = self.newest_framed(None)?;
            self.scanned = true;
        }
        Ok(self.current)
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, TrustCenterStoreError> {
        let mut chunk = [0u8; RECORD_READBACK];
        'slots: for slot in 0..TRUST_CENTER_JOURNAL_SLOTS_PER_SECTOR {
            let base = self.slot_address(sector, slot);
            for offset in (0..TRUST_CENTER_JOURNAL_SLOT_SIZE).step_by(RECORD_READBACK) {
                self.storage
                    .read(base + offset as u32, &mut chunk)
                    .map_err(|_| TrustCenterStoreError::Hardware)?;
                if chunk.iter().any(|byte| *byte != 0xFF) {
                    continue 'slots;
                }
            }
            return Ok(Some(slot));
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
        // Encode in place: the bytes after the encoding stay erased.
        let encoded_len = state.encode_into(&mut SliceSink::new(
            &mut record[RECORD_ENCODED_OFFSET..RECORD_CRC_OFFSET],
        ))?;
        record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.slot_address(sector, slot);
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| TrustCenterStoreError::Hardware)?;
        // TLSR8258 can only program from SRAM; stage the marker on the stack.
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| TrustCenterStoreError::Hardware)?;

        // Byte-compare the whole programmed slot with the record: identical
        // bytes decode to the identical state.
        let mut chunk = [0u8; RECORD_READBACK];
        for offset in (0..TRUST_CENTER_JOURNAL_SLOT_SIZE).step_by(RECORD_READBACK) {
            self.storage
                .read(address + offset as u32, &mut chunk)
                .map_err(|_| TrustCenterStoreError::Hardware)?;
            if chunk[..] != record[offset..offset + RECORD_READBACK] {
                return Err(TrustCenterStoreError::Hardware);
            }
        }
        Ok(())
    }

    fn cache_result(&mut self, result: &Result<(), TrustCenterStoreError>, located: LocatedRecord) {
        if result.is_ok() {
            self.current = Some(located);
        } else {
            self.current = None;
            self.scanned = false;
        }
    }
}

impl<S: NorFlash> TrustCenterDeviceStore for TrustCenterDeviceJournal<S> {
    fn load_into(
        &mut self,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        if self.scanned {
            let Some(located) = self.current else {
                return Ok(false);
            };
            if self.decode_at(located, state)? {
                return Ok(true);
            }
            // The flash changed under the cache: rescan on the next call.
            self.current = None;
            self.scanned = false;
            return Err(TrustCenterStoreError::Corrupt);
        }
        // Newest decodable record: candidates newest first, `state` is the
        // decode scratch.
        let mut below = None;
        loop {
            let Some(located) = self.newest_framed(below)? else {
                self.current = None;
                self.scanned = true;
                return Ok(false);
            };
            if self.decode_at(located, state)? {
                self.current = Some(located);
                self.scanned = true;
                return Ok(true);
            }
            below = Some(located.generation);
        }
    }

    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        // Refuse before any erase: a state larger than one slot (possible only
        // with `ncp-capacity`) belongs in the streamed journal.
        if state.encoded_len() > RECORD_ENCODED_CAPACITY {
            return Err(TrustCenterStoreError::Full);
        }
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
                self.cache_result(
                    &result,
                    LocatedRecord {
                        generation,
                        sector: located.sector,
                        slot,
                    },
                );
                return result;
            }
            let target = 1 - located.sector;
            let sector = self.sectors[target];
            let result = self
                .storage
                .erase(sector, sector + TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| TrustCenterStoreError::Hardware)
                .and_then(|()| self.write_record(target, 0, generation, state));
            self.cache_result(
                &result,
                LocatedRecord {
                    generation,
                    sector: target,
                    slot: 0,
                },
            );
            return result;
        }

        let sector = self.sectors[0];
        let result = self
            .storage
            .erase(sector, sector + TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32)
            .map_err(|_| TrustCenterStoreError::Hardware)
            .and_then(|()| self.write_record(0, 0, generation, state));
        self.cache_result(
            &result,
            LocatedRecord {
                generation,
                sector: 0,
                slot: 0,
            },
        );
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
        self.current = None;
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

/// Optional sections of one record version's encoding.
#[derive(Debug, Clone, Copy)]
struct Layout {
    /// Version 2: application-key allowlist count is mandatory, followed by
    /// the network-key rotation trailer.
    rotation: bool,
    /// Version 3: parent-directed removal intents for unknown devices.
    unknown_removals: bool,
    /// Version 4: router capability flag in each device record.
    router_capability: bool,
    /// Version 6: high byte of the device count before the APS limit.
    count_high: bool,
    /// Version 5: durable global outgoing APS limit.
    global_limit: bool,
}

impl Layout {
    const V1: Self = Self {
        rotation: false,
        unknown_removals: false,
        router_capability: false,
        count_high: false,
        global_limit: false,
    };
    const V2: Self = Self {
        rotation: true,
        ..Self::V1
    };
    const V3: Self = Self {
        unknown_removals: true,
        ..Self::V2
    };
    const V4: Self = Self {
        router_capability: true,
        ..Self::V3
    };
    #[cfg(feature = "global-aps-counter")]
    const V5: Self = Self {
        global_limit: true,
        ..Self::V4
    };
    #[cfg(feature = "ncp-capacity")]
    const V6: Self = Self {
        count_high: true,
        ..Self::V5
    };
    #[cfg(not(feature = "global-aps-counter"))]
    const CURRENT: Self = Self::V4;
    #[cfg(all(feature = "global-aps-counter", not(feature = "ncp-capacity")))]
    const CURRENT: Self = Self::V5;
    #[cfg(feature = "ncp-capacity")]
    const CURRENT: Self = Self::V6;
}

/// Sequential byte sink of the snapshot encoder.
pub(crate) trait SnapshotSink {
    /// Append `bytes`; `Full` when the destination cannot hold them.
    fn put(&mut self, bytes: &[u8]) -> Result<(), TrustCenterStoreError>;
}

/// Byte source of the snapshot decoder: sequential reads plus one look-ahead
/// at an absolute position (the version-6 device-count high byte).
pub(crate) trait SnapshotSource {
    /// Bytes not consumed yet.
    fn remaining(&self) -> usize;
    /// Copy the next `out.len()` bytes; `Corrupt` if fewer remain.
    fn take(&mut self, out: &mut [u8]) -> Result<(), TrustCenterStoreError>;
    /// The byte at `index` counted from the current position, without
    /// consuming anything.
    fn byte_at(&mut self, index: usize) -> Result<u8, TrustCenterStoreError>;

    fn byte(&mut self) -> Result<u8, TrustCenterStoreError> {
        let mut byte = [0u8];
        self.take(&mut byte)?;
        Ok(byte[0])
    }
}

/// [`SnapshotSink`] over a bounded byte slice.
pub(crate) struct SliceSink<'a> {
    output: &'a mut [u8],
    len: usize,
}

impl<'a> SliceSink<'a> {
    pub(crate) fn new(output: &'a mut [u8]) -> Self {
        Self { output, len: 0 }
    }
}

impl SnapshotSink for SliceSink<'_> {
    fn put(&mut self, bytes: &[u8]) -> Result<(), TrustCenterStoreError> {
        let end = self
            .len
            .checked_add(bytes.len())
            .filter(|end| *end <= self.output.len())
            .ok_or(TrustCenterStoreError::Full)?;
        self.output[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }
}

/// [`SnapshotSource`] over a byte slice.
pub(crate) struct SliceSource<'a> {
    bytes: &'a [u8],
}

impl<'a> SliceSource<'a> {
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }
}

impl SnapshotSource for SliceSource<'_> {
    fn remaining(&self) -> usize {
        self.bytes.len()
    }

    fn take(&mut self, out: &mut [u8]) -> Result<(), TrustCenterStoreError> {
        if out.len() > self.bytes.len() {
            return Err(TrustCenterStoreError::Corrupt);
        }
        let (head, tail) = self.bytes.split_at(out.len());
        out.copy_from_slice(head);
        self.bytes = tail;
        Ok(())
    }

    fn byte_at(&mut self, index: usize) -> Result<u8, TrustCenterStoreError> {
        self.bytes
            .get(index)
            .copied()
            .ok_or(TrustCenterStoreError::Corrupt)
    }
}

fn encode_device(stored: &PersistentTrustCenterDevice) -> [u8; DEVICE_ENCODED_LEN] {
    let mut output = [0u8; DEVICE_ENCODED_LEN];
    let device = stored.device;
    output[0..8].copy_from_slice(&device.ieee_address);
    output[8..16].copy_from_slice(&device.parent_address);
    output[16..18].copy_from_slice(&device.short_address.0.to_le_bytes());
    output[18..34].copy_from_slice(&device.link_key);
    output[34] = encode_origin(device.key_origin);
    output[35] = device.key_attributes as u8;
    output[36] = device.join_timeout_remaining_secs;
    output[37] = u8::from(stored.incoming_frame_counter_valid)
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
    output[38..42].copy_from_slice(&stored.outgoing_frame_counter.to_le_bytes());
    output[42..46].copy_from_slice(&stored.outgoing_frame_counter_limit.to_le_bytes());
    output[46..50].copy_from_slice(&stored.incoming_frame_counter.to_le_bytes());
    if let Some(pending) = stored.pending_link_key {
        output[50..66].copy_from_slice(&pending.key);
        output[66..70].copy_from_slice(&pending.outgoing_frame_counter.to_le_bytes());
        output[70..74].copy_from_slice(&pending.outgoing_frame_counter_limit.to_le_bytes());
    }
    output[74] =
        stored.confirm_key_pending.unwrap_or(0) | (u8::from(stored.is_router == Some(true)) << 6);
    output
}

fn decode_device(
    encoded: &[u8; DEVICE_ENCODED_LEN],
    layout: Layout,
) -> Result<PersistentTrustCenterDevice, TrustCenterStoreError> {
    let mut ieee_address = [0u8; 8];
    ieee_address.copy_from_slice(&encoded[0..8]);
    let mut parent_address = [0u8; 8];
    parent_address.copy_from_slice(&encoded[8..16]);
    let mut link_key = [0u8; 16];
    link_key.copy_from_slice(&encoded[18..34]);
    let key_origin = decode_origin(encoded[34])?;
    let key_attributes = decode_attributes(encoded[35])?;
    let allowed_flags = if layout.router_capability {
        0xFF
    } else if layout.rotation {
        0x7F
    } else {
        0x3F
    };
    if encoded[37] & !allowed_flags != 0 || (encoded[37] & 0x20 != 0 && encoded[37] & 0x02 == 0) {
        return Err(TrustCenterStoreError::Corrupt);
    }
    let word = |offset: usize| {
        u32::from_le_bytes([
            encoded[offset],
            encoded[offset + 1],
            encoded[offset + 2],
            encoded[offset + 3],
        ])
    };
    let pending_link_key = if encoded[37] & 0x02 != 0 {
        let mut key = [0u8; 16];
        key.copy_from_slice(&encoded[50..66]);
        Some(PendingTrustCenterLinkKey {
            key,
            outgoing_frame_counter: word(66),
            outgoing_frame_counter_limit: word(70),
            transported: encoded[37] & 0x20 != 0,
        })
    } else {
        if encoded[50..74].iter().any(|byte| *byte != 0) {
            return Err(TrustCenterStoreError::Corrupt);
        }
        None
    };
    let is_router = if layout.router_capability && encoded[37] & 0x80 != 0 {
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
    Ok(PersistentTrustCenterDevice {
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
        new_network_key_delivered: layout.rotation && encoded[37] & 0x40 != 0,
        is_router,
        outgoing_frame_counter: word(38),
        outgoing_frame_counter_limit: word(42),
        incoming_frame_counter: word(46),
        pending_link_key,
    })
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

    /// `state()` plus every optional section, so in-place copies and decodes
    /// are checked field by field.
    fn rich_state() -> PersistentTrustCenterState {
        let mut state = state();
        state.set_rotation(Some(NetworkKeyRotation {
            phase: NetworkKeyRotationPhase::Distributing,
            method: NetworkKeyUpdateMethod::Broadcast,
            target_sequence: 3,
        }));
        state.allow_application_key_request(DEVICE, PARENT).unwrap();
        state
            .stage_unknown_removal(PendingUnknownRemoval {
                parent_address: PARENT,
                device_address: [0x55; 8],
            })
            .unwrap();
        #[cfg(feature = "global-aps-counter")]
        assert!(state.set_global_aps_limit(0x8000));
        state
    }

    #[test]
    fn in_place_construction_copy_reset_and_decode_match_the_by_value_forms() {
        let mut storage = core::mem::MaybeUninit::<PersistentTrustCenterState>::uninit();
        // Poison: every field must be written, nothing read first.
        unsafe {
            storage.as_mut_ptr().write_bytes(0xA5, 1);
            PersistentTrustCenterState::write_into(storage.as_mut_ptr(), EPID);
        }
        let in_place = unsafe { storage.assume_init_mut() };
        assert_eq!(*in_place, PersistentTrustCenterState::new(EPID));

        let rich = rich_state();
        in_place.copy_from(&rich);
        assert_eq!(*in_place, rich);
        in_place.reset([0x66; 8]);
        assert_eq!(*in_place, PersistentTrustCenterState::new([0x66; 8]));

        // Decoding overwrites every field of a non-empty destination.
        let plain = state();
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = plain.encode(&mut encoded);
        let mut destination = rich_state();
        destination.decode_into(&encoded[..len]).unwrap();
        assert_eq!(destination, plain);
        let len = rich.encode(&mut encoded);
        destination.decode_into(&encoded[..len]).unwrap();
        assert_eq!(destination, rich);
        unsafe { storage.assume_init_drop() };
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
                &encoded[..len
                    - GLOBAL_APS_TRAILER_LEN
                    - ROTATION_TRAILER_LEN
                    - UNKNOWN_REMOVAL_HEADER_LEN]
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
        let decoded =
            PersistentTrustCenterState::decode_v3(&encoded[..len - GLOBAL_APS_TRAILER_LEN])
                .unwrap();
        assert_eq!(decoded.device(&DEVICE).unwrap().is_router, None);
        encoded[STATE_HEADER_LEN + 37] |= 0x80;
        assert_eq!(
            PersistentTrustCenterState::decode_v3(&encoded[..len - GLOBAL_APS_TRAILER_LEN]),
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
                &encoded[..len
                    - GLOBAL_APS_TRAILER_LEN
                    - ROTATION_TRAILER_LEN
                    - UNKNOWN_REMOVAL_HEADER_LEN]
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
                - if version < RECORD_VERSION {
                    GLOBAL_APS_TRAILER_LEN
                } else {
                    0
                }
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
            // Version 5 is the version-4 body plus the APS limit, without the
            // version-6 device-count high byte.
            #[cfg(feature = "ncp-capacity")]
            let encoded_len = if version == GLOBAL_APS_RECORD_VERSION {
                let body = current_len - GLOBAL_APS_TRAILER_LEN;
                record[RECORD_ENCODED_OFFSET + body..RECORD_ENCODED_OFFSET + body + 4]
                    .copy_from_slice(&encoded[current_len - 4..current_len]);
                body + 4
            } else {
                encoded_len
            };
            record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
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
            #[cfg(feature = "global-aps-counter")]
            GLOBAL_APS_RECORD_VERSION_PREDECESSOR,
            #[cfg(feature = "ncp-capacity")]
            GLOBAL_APS_RECORD_VERSION,
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
        for index in 0..MAX_PENDING_UNKNOWN_REMOVALS {
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
            MAX_PENDING_UNKNOWN_REMOVALS
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
            MAX_PENDING_UNKNOWN_REMOVALS
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
        let count_offset = len
            - GLOBAL_APS_TRAILER_LEN
            - UNKNOWN_REMOVAL_HEADER_LEN
            - 2 * UNKNOWN_REMOVAL_ENTRY_LEN;
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
        bad[count_offset] = (MAX_PENDING_UNKNOWN_REMOVALS + 1) as u8;
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

    #[test]
    fn hw03_commit_marker_is_not_programmed_from_static_constant() {
        use crate::flash_source_guard::StaticSourceGuard;
        static FORBIDDEN: [&[u8]; 1] = [&RECORD_COMMIT];
        let mut journal = TrustCenterDeviceJournal::new(
            StaticSourceGuard::new(MockFlash::new(), &FORBIDDEN),
            0,
            TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
        );
        let expected = state();
        journal.store(&expected).unwrap();
        assert_eq!(journal.storage().rejected, 0);
        assert_eq!(journal.load(), Ok(Some(expected)));
    }

    #[cfg(feature = "global-aps-counter")]
    #[test]
    fn version_5_carries_the_global_aps_limit_and_version_4_reads_as_unmigrated() {
        let mut state = state();
        assert_eq!(state.global_aps_limit(), 0);
        assert!(state.set_global_aps_limit(0x0012_3400));
        assert!(
            !state.set_global_aps_limit(0x0012_3000),
            "the limit never moves back"
        );
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = state.encode(&mut encoded);
        assert_eq!(&encoded[len - 4..len], &0x0012_3400u32.to_le_bytes());
        assert_eq!(
            PersistentTrustCenterState::decode(&encoded[..len]),
            Ok(state.clone())
        );
        let v4 = PersistentTrustCenterState::decode_v4(&encoded[..len - GLOBAL_APS_TRAILER_LEN])
            .unwrap();
        assert_eq!(v4.global_aps_limit(), 0);

        let mut journal = TrustCenterDeviceJournal::new(
            MockFlash::new(),
            0,
            TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32,
        );
        journal.store(&state).unwrap();
        let flash = journal.into_storage();
        assert_eq!(flash.bytes[4], RECORD_VERSION);
        let mut reopened =
            TrustCenterDeviceJournal::new(flash, 0, TRUST_CENTER_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(
            reopened.load().unwrap().unwrap().global_aps_limit(),
            0x0012_3400
        );
    }
}
