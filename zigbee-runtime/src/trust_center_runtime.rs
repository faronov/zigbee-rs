//! Production Trust Center policy, key-table, and durable transaction executor.

use crate::role::Router;
use crate::security_store::{SecurityStateStore, SecurityStoreError};
use crate::trust_center_store::{
    NetworkKeyRotation, NetworkKeyRotationPhase, PendingApplicationLinkKey,
    PendingTrustCenterLinkKey, PendingUnknownRemoval, PersistentTrustCenterDevice,
    PersistentTrustCenterState, TrustCenterDeviceStore, TrustCenterStoreError,
};
use zigbee_aps::ApsStatus;
use zigbee_aps::apsme::{
    ApsFrameCounterReservation, ApsmeRequestKeyIndication, ApsmeSecurityIndication,
    ApsmeUpdateDeviceIndication, ApsmeVerifyKeyIndication,
};
use zigbee_aps::security::{ApsKeyType, ApsLinkKeyEntry};
pub use zigbee_bdb::attributes::NetworkKeyUpdateMethod;
use zigbee_bdb::trust_center::{
    MAX_TRUST_CENTER_DEVICES, TrustCenterAction, TrustCenterError, TrustCenterPolicy,
    TrustCenterTable,
};
use zigbee_mac::MacDriver;
use zigbee_nwk::{DeviceType, NwkStatus};
use zigbee_types::{IeeeAddress, ShortAddress};

const APS_COUNTER_RESERVATION: u32 = 0x400;
const MICROS_PER_SECOND: u64 = 1_000_000;
const INITIAL_NETWORK_KEY_RETRY_US: u32 = 5_000_000;
// Product policy, not an R22-mandated switch delay.
const NETWORK_KEY_PROPAGATION_DELAY_US: u32 = 30_000_000;
type ZigbeeDevice<M> = crate::ZigbeeDevice<M, Router>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrustCenterDeliveryKind {
    ConfirmKey,
    RemoveDevice,
    ApplicationKeyInitiator,
    ApplicationKeyResponder,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingTrustCenterDelivery {
    kind: TrustCenterDeliveryKind,
    address: IeeeAddress,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeliveryProgress {
    Ready,
    Submitted,
    Busy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustCenterRuntimeError {
    NotCoordinator,
    NotJoined,
    MissingNetworkKey,
    EntropyUnavailable,
    CounterExhausted,
    KeyTableFull,
    NetworkKeyRotationInProgress,
    NetworkKeyRotationCooldown,
    NetworkKeyRotationNotPrepared,
    NetworkKeyActivationFailed,
    RouterListIncomplete,
    Store(TrustCenterStoreError),
    Policy(TrustCenterError),
    Aps(ApsStatus),
    Nwk(NwkStatus),
    Persistence(SecurityStoreError),
}

impl From<TrustCenterStoreError> for TrustCenterRuntimeError {
    fn from(error: TrustCenterStoreError) -> Self {
        Self::Store(error)
    }
}

impl From<TrustCenterError> for TrustCenterRuntimeError {
    fn from(error: TrustCenterError) -> Self {
        Self::Policy(error)
    }
}

impl From<SecurityStoreError> for TrustCenterRuntimeError {
    fn from(error: SecurityStoreError) -> Self {
        Self::Persistence(error)
    }
}

/// Owns the authoritative per-device Trust Center state for one Coordinator.
pub struct TrustCenterRuntime<S> {
    store: S,
    table: TrustCenterTable,
    state: Option<PersistentTrustCenterState>,
    last_poll_us: u32,
    residual_us: u64,
    network_key_update_elapsed_secs: u64,
    network_key_cooldown_started: Option<u32>,
    last_transport_error: Option<TrustCenterRuntimeError>,
    pending_delivery: Option<PendingTrustCenterDelivery>,
    pending_device_replay_tombstone: Option<IeeeAddress>,
    initialization_pending: bool,
    initial_key_attempts: heapless::Vec<(IeeeAddress, u32), MAX_TRUST_CENTER_DEVICES>,
    network_key_propagation_started: Option<u32>,
}

impl<S: TrustCenterDeviceStore> TrustCenterRuntime<S> {
    pub fn new(store: S) -> Self {
        Self {
            store,
            table: TrustCenterTable::new(),
            state: None,
            last_poll_us: 0,
            residual_us: 0,
            network_key_update_elapsed_secs: 0,
            network_key_cooldown_started: None,
            last_transport_error: None,
            pending_delivery: None,
            pending_device_replay_tombstone: None,
            initialization_pending: false,
            initial_key_attempts: heapless::Vec::new(),
            network_key_propagation_started: None,
        }
    }

    pub const fn is_initialized(&self) -> bool {
        self.state.is_some() && !self.initialization_pending
    }

    pub fn store(&self) -> &S {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut S {
        &mut self.store
    }

    pub fn table(&self) -> &TrustCenterTable {
        &self.table
    }

    /// Provision and durably install an install-code-derived key before join.
    pub fn provision_install_code<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
        install_code: &[u8],
    ) -> Result<[u8; 16], TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let key = self.table.provision_install_code(address, install_code)?;
        self.upsert_table_device(device, &address, false)?;
        Ok(key)
    }

    /// Provision an application-supplied unique TCLK before join.
    pub fn provision_link_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
        key: [u8; 16],
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        self.table.provision_link_key(
            address,
            key,
            zigbee_bdb::trust_center::TrustCenterKeyOrigin::ApplicationProvisioned,
        )?;
        self.upsert_table_device(device, &address, false)
    }

    /// Add one directional application-link-key request pair to the durable allowlist.
    pub fn allow_application_key_request<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
        initiator_address: IeeeAddress,
        responder_address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        state.allow_application_key_request(initiator_address, responder_address)?;
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    /// Remove one directional pair from the durable application-key allowlist.
    pub fn deny_application_key_request<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
        initiator_address: &IeeeAddress,
        responder_address: &IeeeAddress,
    ) -> Result<bool, TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let removed = state.deny_application_key_request(initiator_address, responder_address);
        if removed {
            self.store.store(&state)?;
            self.state = Some(state);
        }
        Ok(removed)
    }

    pub fn has_pending_transactions(&self) -> bool {
        self.state.as_ref().is_some_and(|state| {
            state.rotation().is_some()
                || state.pending_application_key().is_some()
                || !state.pending_unknown_removals().is_empty()
                || state.devices().iter().any(|stored| {
                    stored.network_key_pending
                        || stored.pending_link_key.is_some()
                        || stored.confirm_key_pending.is_some()
                        || stored.removal_pending
                })
        })
    }

    pub const fn last_transport_error(&self) -> Option<TrustCenterRuntimeError> {
        self.last_transport_error
    }

    /// Return a TC removal that must be executed through the local parent's
    /// crash-safe child journal before the TC record is revoked.
    pub fn pending_local_child_removal<M: MacDriver>(
        &self,
        device: &ZigbeeDevice<M>,
    ) -> Option<IeeeAddress> {
        let local_ieee = device.aps().nwk().nib().ieee_address;
        self.state
            .as_ref()
            .and_then(|state| {
                state.devices().iter().find_map(|stored| {
                    (stored.removal_pending
                        && stored.device.parent_address == local_ieee
                        && device
                            .aps()
                            .nwk()
                            .known_child_by_ieee(&stored.device.ieee_address)
                            .is_some())
                    .then_some(stored.device.ieee_address)
                })
            })
            .or_else(|| {
                self.state
                    .as_ref()?
                    .pending_unknown_removals()
                    .iter()
                    .find_map(|pending| {
                        (pending.parent_address == local_ieee
                            && device
                                .aps()
                                .nwk()
                                .known_child_by_ieee(&pending.device_address)
                                .is_some())
                        .then_some(pending.device_address)
                    })
            })
    }

    /// Restore the network-bound database, reserve fresh counter ranges, and
    /// install every live key without transmitting or accepting transactions.
    ///
    /// The caller must restore the durable replay journal and then call
    /// [`Self::resume_after_replay_restore`] before exposing the coordinator
    /// receive loop.
    fn restore_before_replay<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        // Close the barrier before the first fallible operation, including a
        // re-initialization of a previously live runtime. A partial key-table
        // restore must never be mistaken for a completed restore.
        self.initialization_pending = true;
        self.state = None;
        self.cancel_delivery();
        self.initial_key_attempts.clear();
        self.network_key_propagation_started = None;
        let extended_pan_id = validate_coordinator(device)?;
        let mut state = match self.store.load()? {
            Some(state) if state.matches_network(&extended_pan_id) => state,
            Some(_) => {
                self.store.clear()?;
                PersistentTrustCenterState::new(extended_pan_id)
            }
            None => PersistentTrustCenterState::new(extended_pan_id),
        };
        self.table.clear();
        for stored in state.devices() {
            self.table.restore_device(stored.device)?;
            if let Some(pending) = stored.pending_link_key {
                self.table.install_generated_trust_center_link_key(
                    stored.device.ieee_address,
                    pending.key,
                    stored.device.join_timeout_remaining_secs.max(1),
                )?;
            }
        }
        reserve_restored_ranges(&mut state)?;
        self.store.store(&state)?;
        for stored in state.devices() {
            install_key(device, stored)?;
        }
        self.state = Some(state);
        self.last_poll_us = device.mac().monotonic_micros();
        self.residual_us = 0;
        self.network_key_update_elapsed_secs = 0;
        self.network_key_cooldown_started = device
            .aps()
            .nwk()
            .security()
            .secondary_key()
            .is_some()
            .then_some(self.last_poll_us);
        self.last_transport_error = None;
        Ok(())
    }

    /// Resume durable Trust Center transactions after replay floors have been
    /// re-applied against the restored per-device keys.
    async fn resume_after_replay_restore<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        if self.state.is_none() {
            return Err(TrustCenterRuntimeError::NotJoined);
        }
        if !self.initialization_pending {
            return Ok(());
        }
        // A successful pending Confirm-Key may represent a key replacement.
        // The composition root must first tombstone replay state against the
        // fully restored Trust Center key table, so initialization resumes all
        // other transactions but defers Confirm-Key transmission to `poll`.
        self.resume_pending(device, false).await?;
        self.initialization_pending = false;
        Ok(())
    }

    /// Restore all TC keys and reservations, re-apply the authoritative replay
    /// journal, then resume transactions. No await/transmit occurs before the
    /// replay barrier. Failure leaves `poll` and indications disabled.
    pub async fn initialize_with_security_store<M: MacDriver, R: SecurityStateStore>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        replay_store: &mut R,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.restore_before_replay(device)?;
        device.restore_incoming_replay_state(replay_store)?;
        if self
            .state
            .as_ref()
            .and_then(PersistentTrustCenterState::rotation)
            .is_some_and(|rotation| {
                rotation.method == NetworkKeyUpdateMethod::Broadcast
                    && rotation.phase != NetworkKeyRotationPhase::Activating
            })
            && !device.begin_network_key_forwarding(replay_store)?
        {
            return Err(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared);
        }
        self.resume_after_replay_restore(device).await
    }

    // Unit fixtures without an external replay log still use the very same
    // initialization path. There is no journal-bypassing production API.
    #[cfg(test)]
    async fn initialize<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.initialize_with_security_store(
            device,
            &mut crate::security_store::RamSecurityStateStore::new(),
        )
        .await
    }

    pub fn clear<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        if let Some(state) = self.state.as_ref() {
            for stored in state.devices() {
                device
                    .aps_mut()
                    .security_mut()
                    .remove_key(&stored.device.ieee_address, ApsKeyType::TrustCenterLinkKey);
            }
        }
        self.table.clear();
        self.state = None;
        self.last_poll_us = 0;
        self.residual_us = 0;
        self.network_key_update_elapsed_secs = 0;
        self.network_key_cooldown_started = None;
        self.last_transport_error = None;
        self.pending_device_replay_tombstone = None;
        self.initialization_pending = false;
        self.initial_key_attempts.clear();
        self.network_key_propagation_started = None;
        self.cancel_delivery();
        self.store.clear()?;
        Ok(())
    }

    /// Execute one authenticated APS security command indication.
    pub async fn handle_indication<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        indication: ApsmeSecurityIndication,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        self.checkpoint_live_counters(device)?;
        let mut policy = TrustCenterPolicy::from_attributes(device.bdb().attributes());
        policy.allow_joins &= device.aps().nwk().nib().permit_joining;
        match indication {
            ApsmeSecurityIndication::UpdateDevice(indication) => {
                self.handle_update_device(device, policy, indication)
                    .await?;
            }
            ApsmeSecurityIndication::RequestKey(indication) => {
                self.complete_initial_network_key(indication.source_address)?;
                self.handle_request_key(device, policy, indication).await?;
            }
            ApsmeSecurityIndication::VerifyKey(indication) => {
                self.handle_verify_key(device, indication).await?;
            }
            ApsmeSecurityIndication::RemoveDevice(indication) => {
                log::warn!(
                    "[TC] Ignoring unauthorized Remove-Device from {:02X?} for {:02X?}",
                    indication.source_address,
                    indication.child_address
                );
            }
        }
        self.checkpoint_live_counters(device)?;
        Ok(())
    }

    /// Advance Trust Center deadlines and retry crash-safe transactions.
    pub async fn poll<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        self.resume_pending(device, true).await?;
        let now = device.mac().monotonic_micros();
        let delta = u64::from(now.wrapping_sub(self.last_poll_us));
        self.last_poll_us = now;
        if self.network_key_cooldown_started.is_some_and(|started| {
            now.wrapping_sub(started)
                >= u32::from(device.aps().nwk().nib().broadcast_delivery_time) * 1_000_000
        }) {
            self.network_key_cooldown_started = None;
        }
        self.residual_us = self.residual_us.saturating_add(delta);
        let elapsed_total_secs = self.residual_us / MICROS_PER_SECOND;
        self.residual_us %= MICROS_PER_SECOND;
        if self
            .state
            .as_ref()
            .is_some_and(|state| state.rotation().is_none())
        {
            self.network_key_update_elapsed_secs = self
                .network_key_update_elapsed_secs
                .saturating_add(elapsed_total_secs);
        }
        let elapsed_secs = elapsed_total_secs.min(u64::from(u8::MAX)) as u8;
        if elapsed_secs != 0 {
            let action = self.table.tick(elapsed_secs);
            self.execute_action(device, action).await?;
            self.checkpoint_table(device)?;
        }
        Ok(())
    }

    /// Whether the configured periodic key-update interval has elapsed.
    pub fn network_key_rotation_due<M: MacDriver>(&self, device: &ZigbeeDevice<M>) -> bool {
        let period_minutes = device
            .bdb()
            .attributes()
            .trust_center_network_key_update_period;
        period_minutes != 0
            && self
                .state
                .as_ref()
                .is_some_and(|state| state.rotation().is_none())
            && self.network_key_update_elapsed_secs >= u64::from(period_minutes).saturating_mul(60)
    }

    /// Return a staged next-sequence key that was durably checkpointed before
    /// the Trust Center transaction record could be committed.
    pub fn has_orphaned_network_key_preparation<M: MacDriver>(
        &self,
        device: &ZigbeeDevice<M>,
    ) -> bool {
        if self
            .state
            .as_ref()
            .is_none_or(|state| state.rotation().is_some())
        {
            return false;
        }
        let security = device.aps().nwk().security();
        let Some(active) = security.active_key() else {
            return false;
        };
        security
            .staged_key()
            .is_some_and(|staged| staged.seq_number == active.seq_number.wrapping_add(1))
    }

    /// Stage the next sequential NWK key in RAM.
    ///
    /// The caller must checkpoint the security store before calling
    /// [`Self::commit_network_key_rotation`].
    pub fn prepare_network_key_rotation<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<u8, TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        if self
            .state
            .as_ref()
            .is_some_and(|state| state.rotation().is_some())
        {
            return Err(TrustCenterRuntimeError::NetworkKeyRotationInProgress);
        }
        let active_sequence = device
            .aps()
            .nwk()
            .security()
            .active_key()
            .map(|entry| entry.seq_number)
            .ok_or(TrustCenterRuntimeError::MissingNetworkKey)?;
        let target_sequence = active_sequence.wrapping_add(1);
        if device
            .aps()
            .nwk()
            .security()
            .staged_key()
            .is_some_and(|entry| entry.seq_number == target_sequence)
        {
            return Ok(target_sequence);
        }

        // R22 4.7.3.10.6 recommends expiring the preceding broadcasts
        // before starting another update/switch pair. Restart this guard
        // after reboot, when the previous-key slot proves a prior switch.
        if let Some(started) = self.network_key_cooldown_started {
            let cooldown_us =
                u32::from(device.aps().nwk().nib().broadcast_delivery_time) * 1_000_000;
            if device.mac().monotonic_micros().wrapping_sub(started) < cooldown_us {
                return Err(TrustCenterRuntimeError::NetworkKeyRotationCooldown);
            }
            self.network_key_cooldown_started = None;
        }

        let mut key = [0u8; 16];
        device
            .mac_mut()
            .fill_random(&mut key)
            .map_err(|_| TrustCenterRuntimeError::EntropyUnavailable)?;
        if !device
            .aps_mut()
            .nwk_mut()
            .security_mut()
            .stage_network_key(key, target_sequence)
        {
            return Err(TrustCenterRuntimeError::NetworkKeyActivationFailed);
        }
        Ok(target_sequence)
    }

    /// Commit the durable Trust Center intent after the staged NWK key is
    /// already durable in the security journal.
    pub fn commit_network_key_rotation<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
        target_sequence: u8,
        method: NetworkKeyUpdateMethod,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if state.rotation().is_some() {
            return Err(TrustCenterRuntimeError::NetworkKeyRotationInProgress);
        }
        if method == NetworkKeyUpdateMethod::Unicast
            && state.devices().iter().any(|stored| {
                stored.device.short_address.0 < 0xFFF8
                    && !stored.removal_pending
                    && stored.is_router.is_none()
            })
        {
            return Err(TrustCenterRuntimeError::RouterListIncomplete);
        }
        let security = device.aps().nwk().security();
        let active_sequence = security
            .active_key()
            .map(|entry| entry.seq_number)
            .ok_or(TrustCenterRuntimeError::MissingNetworkKey)?;
        if target_sequence != active_sequence.wrapping_add(1)
            || security
                .staged_key()
                .is_none_or(|entry| entry.seq_number != target_sequence)
        {
            return Err(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared);
        }
        state.set_rotation(Some(NetworkKeyRotation {
            phase: NetworkKeyRotationPhase::Distributing,
            method,
            target_sequence,
        }));
        self.store.store(&state)?;
        self.state = Some(state);
        self.network_key_update_elapsed_secs = 0;
        self.network_key_cooldown_started = Some(device.mac().monotonic_micros());
        Ok(())
    }

    pub const fn network_key_rotation(&self) -> Option<NetworkKeyRotation> {
        match self.state.as_ref() {
            Some(state) => state.rotation(),
            None => None,
        }
    }

    pub fn handle_device_announce<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
        address: IeeeAddress,
        short_address: ShortAddress,
        capabilities: u8,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let Some(stored) = state.device_mut(&address) else {
            log::debug!("[TC] Ignoring announcement from an unadmitted peer");
            return Ok(());
        };
        if stored.removal_pending || stored.device.short_address != short_address {
            log::warn!("[TC] Ignoring announcement with a stale or rejected location");
            return Ok(());
        }
        let is_router = Some(capabilities & 0x02 != 0);
        if stored.is_router != is_router || stored.network_key_pending {
            stored.is_router = is_router;
            stored.network_key_pending = false;
            self.store.store(&state)?;
            self.state = Some(state);
        }
        self.initial_key_attempts
            .retain(|(peer, _)| *peer != address);
        Ok(())
    }

    pub fn pending_network_key_activation(&self) -> Option<u8> {
        self.state
            .as_ref()?
            .rotation()
            .filter(|rotation| rotation.phase == NetworkKeyRotationPhase::Activating)
            .map(|rotation| rotation.target_sequence)
    }

    /// Activate the key selected by a durably recorded Switch-Key phase.
    ///
    /// The caller must checkpoint the security journal before completing the
    /// Trust Center transaction.
    pub fn activate_pending_network_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<bool, TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let target_sequence = self
            .pending_network_key_activation()
            .ok_or(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared)?;
        let nwk = device.aps_mut().nwk_mut();
        let already_active = nwk
            .security()
            .active_key()
            .is_some_and(|entry| entry.seq_number == target_sequence)
            && nwk.nib().active_key_seq_number == target_sequence;
        if already_active {
            return Ok(false);
        }
        if !nwk.switch_active_network_key(target_sequence) {
            return Err(TrustCenterRuntimeError::NetworkKeyActivationFailed);
        }
        Ok(true)
    }

    /// Clear the transaction only after the active key and NIB sequence have
    /// both been durably checkpointed by the caller.
    pub fn complete_network_key_rotation<M: MacDriver, R: SecurityStateStore>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        replay_store: &mut R,
    ) -> Result<(), TrustCenterRuntimeError> {
        self.require_initialized_network(device)?;
        let target_sequence = self
            .pending_network_key_activation()
            .ok_or(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared)?;
        let nwk = device.aps().nwk();
        if nwk
            .security()
            .active_key()
            .is_none_or(|entry| entry.seq_number != target_sequence)
            || nwk.nib().active_key_seq_number != target_sequence
        {
            return Err(TrustCenterRuntimeError::NetworkKeyActivationFailed);
        }
        // A sleepy peer may have stored the new key without seeing Switch-Key.
        // Retain the old key until the next staged update replaces its slot.
        // Its next authenticated new-key packet can activate the staged key.
        device.refresh_security_state(replay_store)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        state.set_rotation(None);
        self.store.store(&state)?;
        self.state = Some(state);
        self.network_key_update_elapsed_secs = 0;
        Ok(())
    }

    fn require_initialized_network<M: MacDriver>(
        &self,
        device: &ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        let extended_pan_id = validate_coordinator(device)?;
        if self
            .state
            .as_ref()
            .is_some_and(|state| state.matches_network(&extended_pan_id))
            && !self.initialization_pending
        {
            Ok(())
        } else {
            Err(TrustCenterRuntimeError::NotJoined)
        }
    }

    async fn handle_update_device<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        policy: TrustCenterPolicy,
        indication: ApsmeUpdateDeviceIndication,
    ) -> Result<(), TrustCenterRuntimeError> {
        if indication.source_address != device.aps().nwk().nib().ieee_address
            && self.table.device(&indication.source_address).is_none()
        {
            // Possession of shared network/commissioning material does not
            // make an unregistered forwarder an authoritative parent.
            log::warn!("[TC] Ignoring Update-Device from an unregistered parent");
            return Ok(());
        }
        if indication.status == zigbee_aps::apsme::ApsUpdateDeviceStatus::DeviceLeft
            && self
                .table
                .device(&indication.device_address)
                .is_some_and(|stored| {
                    stored.parent_address != [0; 8]
                        && (stored.parent_address != indication.source_address
                            || stored.short_address != indication.device_short_address)
                })
        {
            log::warn!("[TC] Ignoring DeviceLeft for a superseded parent/address");
            return Ok(());
        }
        // Do not admit a peer while a previously committed rejection is still
        // being delivered. Repeated updates refresh its parent, not its trust.
        if indication.status != zigbee_aps::apsme::ApsUpdateDeviceStatus::DeviceLeft
            && self.state.as_ref().is_some_and(|state| {
                state
                    .pending_unknown_removals()
                    .iter()
                    .any(|pending| pending.device_address == indication.device_address)
            })
        {
            self.stage_unknown_removal(
                device,
                indication.source_address,
                indication.device_address,
            )?;
            let result = self.resume_unknown_removal(device).await;
            return self.defer_transport_error(result);
        }
        let action = match self.table.handle_update_device(policy, indication) {
            Ok(action) => action,
            Err(error) => {
                log::warn!(
                    "[TC] Rejected Update-Device for {:02X?}: {:?}",
                    indication.device_address,
                    error
                );
                return Ok(());
            }
        };
        match action {
            TrustCenterAction::TransportNetworkKey { device_address, .. } => {
                self.upsert_table_device(device, &device_address, true)?;
                let result = self.resume_network_key(device, device_address).await;
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::None => {
                if self.table.device(&indication.device_address).is_some() {
                    self.upsert_table_device(device, &indication.device_address, false)?;
                }
            }
            other => self.execute_action(device, other).await?,
        }
        self.checkpoint_table(device)
    }

    async fn handle_request_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        mut policy: TrustCenterPolicy,
        indication: ApsmeRequestKeyIndication,
    ) -> Result<(), TrustCenterRuntimeError> {
        if indication.key_type == zigbee_aps::apsme::ApsRequestKeyType::TrustCenterLink
            && self
                .state
                .as_ref()
                .and_then(|state| state.device(&indication.source_address))
                .is_some_and(|stored| stored.pending_link_key.is_some())
        {
            self.stage_trust_center_link_key(device, indication.source_address)?;
            let result = self
                .resume_trust_center_link_key(device, indication.source_address)
                .await;
            return self.defer_transport_error(result);
        }
        if indication.key_type == zigbee_aps::apsme::ApsRequestKeyType::ApplicationLink
            && policy.application_key_requests
                == zigbee_bdb::attributes::ApplicationLinkKeyRequestPolicy::AllowListOnly
        {
            let Some(responder_address) = indication.partner_address else {
                log::warn!(
                    "[TC] Rejected allowlisted Request-Key without a responder from {:02X?}",
                    indication.source_address
                );
                return Ok(());
            };
            let allowed = self.state.as_ref().is_some_and(|state| {
                state
                    .application_key_request_allowed(&indication.source_address, &responder_address)
            });
            if !allowed {
                log::warn!(
                    "[TC] Rejected non-allowlisted application key pair {:02X?} -> {:02X?}",
                    indication.source_address,
                    responder_address
                );
                return Ok(());
            }
            policy.application_key_requests =
                zigbee_bdb::attributes::ApplicationLinkKeyRequestPolicy::AnyPair;
        }
        let local_ieee = device.aps().nwk().nib().ieee_address;
        let action = match self
            .table
            .handle_request_key(policy, indication, local_ieee)
        {
            Ok(action) => action,
            Err(error) => {
                log::warn!(
                    "[TC] Rejected Request-Key from {:02X?}: {:?}",
                    indication.source_address,
                    error
                );
                return Ok(());
            }
        };
        match action {
            TrustCenterAction::GenerateTrustCenterLinkKey { device_address } => {
                self.stage_trust_center_link_key(device, device_address)?;
                let result = self
                    .resume_trust_center_link_key(device, device_address)
                    .await;
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::GenerateApplicationLinkKey {
                initiator_address,
                responder_address,
            } => {
                self.stage_application_link_key(device, initiator_address, responder_address)?;
                let result = self.resume_application_link_key(device).await;
                self.defer_transport_error(result)?;
            }
            other => self.execute_action(device, other).await?,
        }
        Ok(())
    }

    async fn handle_verify_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        indication: ApsmeVerifyKeyIndication,
    ) -> Result<(), TrustCenterRuntimeError> {
        let action = match self.table.handle_verify_key(indication) {
            Ok(action) => action,
            Err(error) => {
                log::warn!(
                    "[TC] Rejected Verify-Key from {:02X?}: {:?}",
                    indication.source_address,
                    error
                );
                return Ok(());
            }
        };
        self.execute_action(device, action).await
    }

    async fn execute_action<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        action: TrustCenterAction,
    ) -> Result<(), TrustCenterRuntimeError> {
        match action {
            TrustCenterAction::None => {}
            TrustCenterAction::TransportNetworkKey { device_address, .. } => {
                self.upsert_table_device(device, &device_address, true)?;
                let result = self.resume_network_key(device, device_address).await;
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::GenerateTrustCenterLinkKey { device_address } => {
                self.stage_trust_center_link_key(device, device_address)?;
                let result = self
                    .resume_trust_center_link_key(device, device_address)
                    .await;
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::GenerateApplicationLinkKey {
                initiator_address,
                responder_address,
            } => {
                self.stage_application_link_key(device, initiator_address, responder_address)?;
                let result = self.resume_application_link_key(device).await;
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::ConfirmKey {
                device_address,
                status,
            } => {
                self.commit_and_stage_confirm_key(device, device_address, status)?;
            }
            TrustCenterAction::RemoveDevice {
                parent_address,
                device_address,
                device_short_address,
            } => {
                let known = self
                    .state
                    .as_ref()
                    .is_some_and(|state| state.device(&device_address).is_some());
                let result = if known {
                    self.stage_removal(
                        device,
                        device_address,
                        parent_address,
                        device_short_address,
                    )?;
                    self.resume_removal(device, device_address).await
                } else {
                    self.stage_unknown_removal(device, parent_address, device_address)?;
                    self.resume_unknown_removal(device).await
                };
                self.defer_transport_error(result)?;
            }
            TrustCenterAction::RevokeDevice { device_address } => {
                if let Some(stored) = self
                    .state
                    .as_ref()
                    .and_then(|state| state.device(&device_address))
                    .copied()
                {
                    self.stage_removal(
                        device,
                        device_address,
                        stored.device.parent_address,
                        stored.device.short_address,
                    )?;
                }
                self.pending_device_replay_tombstone = Some(device_address);
            }
            TrustCenterAction::RejectedWithoutCommand { .. } => {}
        }
        Ok(())
    }

    async fn send_remote_remove_device<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        parent_address: IeeeAddress,
        device_address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let local_ieee = device.aps().nwk().nib().ieee_address;
        if parent_address == local_ieee {
            return Err(TrustCenterRuntimeError::Policy(
                TrustCenterError::RequestNotPermitted,
            ));
        }
        self.ensure_counter_available(device, &parent_address)?;
        let parent_short = self.device_short(&parent_address)?;
        device
            .aps_mut()
            .send_remove_device_to(parent_short, &parent_address, &device_address)
            .await
            .map_err(TrustCenterRuntimeError::Aps)
    }

    fn stage_trust_center_link_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let stored = state
            .device_mut(&address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        if let Some(pending) = stored.pending_link_key.as_mut() {
            pending.transported = false;
        } else {
            let mut key = [0u8; 16];
            device
                .mac_mut()
                .fill_random(&mut key)
                .map_err(|_| TrustCenterRuntimeError::EntropyUnavailable)?;
            let start = stored.outgoing_frame_counter_limit;
            let end = reserve_end(start)?;
            stored.pending_link_key = Some(PendingTrustCenterLinkKey {
                key,
                outgoing_frame_counter: start,
                outgoing_frame_counter_limit: end,
                transported: false,
            });
        }
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    fn stage_application_link_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        initiator_address: IeeeAddress,
        responder_address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if let Some(pending) = state.pending_application_key() {
            if pending.initiator_address != initiator_address
                || pending.responder_address != responder_address
            {
                return Err(TrustCenterRuntimeError::Policy(
                    TrustCenterError::RequestNotPermitted,
                ));
            }
            return Ok(());
        }
        let mut key = [0u8; 16];
        device
            .mac_mut()
            .fill_random(&mut key)
            .map_err(|_| TrustCenterRuntimeError::EntropyUnavailable)?;
        state.set_pending_application_key(Some(PendingApplicationLinkKey {
            initiator_address,
            responder_address,
            key,
            delivered_to_initiator: false,
            delivered_to_responder: false,
        }));
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    async fn resume_pending<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        resume_confirm_key: bool,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut network_pending = [None; MAX_TRUST_CENTER_DEVICES];
        let mut link_pending = [None; MAX_TRUST_CENTER_DEVICES];
        let mut confirm_pending = [None; MAX_TRUST_CENTER_DEVICES];
        let mut removal_pending = [None; MAX_TRUST_CENTER_DEVICES];
        self.last_transport_error = None;
        if let Some(state) = self.state.as_ref() {
            for (index, stored) in state.devices().iter().enumerate() {
                if stored.removal_pending {
                    removal_pending[index] = Some(stored.device.ieee_address);
                    continue;
                }
                if stored.network_key_pending {
                    network_pending[index] = Some(stored.device.ieee_address);
                }
                if stored.pending_link_key.is_some() {
                    link_pending[index] = Some(stored.device.ieee_address);
                }
                if stored.confirm_key_pending.is_some() {
                    confirm_pending[index] = Some(stored.device.ieee_address);
                }
            }
        }
        for address in removal_pending.into_iter().flatten() {
            let result = self.resume_removal(device, address).await;
            self.defer_transport_error(result)?;
        }
        let result = self.resume_unknown_removal(device).await;
        self.defer_transport_error(result)?;
        for address in network_pending.into_iter().flatten() {
            let result = self.resume_network_key(device, address).await;
            self.defer_transport_error(result)?;
        }
        for address in link_pending.into_iter().flatten() {
            let result = self.resume_trust_center_link_key(device, address).await;
            self.defer_transport_error(result)?;
        }
        if resume_confirm_key {
            for address in confirm_pending.into_iter().flatten() {
                let result = self.resume_confirm_key(device, address).await;
                self.defer_transport_error(result)?;
            }
        }
        let result = self.resume_application_link_key(device).await;
        self.defer_transport_error(result)?;
        let result = self.resume_network_key_rotation(device).await;
        self.defer_transport_error(result)
    }

    async fn resume_network_key_rotation<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        loop {
            let Some(rotation) = self
                .state
                .as_ref()
                .and_then(PersistentTrustCenterState::rotation)
            else {
                return Ok(());
            };
            match rotation.phase {
                NetworkKeyRotationPhase::Distributing => {
                    let key = device
                        .aps()
                        .nwk()
                        .security()
                        .key_by_seq(rotation.target_sequence)
                        .map(|entry| entry.key)
                        .ok_or(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared)?;
                    let local_ieee = device.aps().nwk().nib().ieee_address;
                    if rotation.method == NetworkKeyUpdateMethod::Broadcast {
                        device
                            .aps_mut()
                            .send_transport_key(
                                ShortAddress::BROADCAST,
                                &[0; 8],
                                0x01,
                                &key,
                                rotation.target_sequence,
                                &local_ieee,
                            )
                            .await
                            .map_err(TrustCenterRuntimeError::Aps)?;
                    } else if let Some(address) = self.state.as_ref().and_then(|state| {
                        state.devices().iter().find_map(|stored| {
                            (stored.is_router == Some(true)
                                && !stored.new_network_key_delivered
                                && !stored.removal_pending
                                && stored.device.short_address.0 < 0xFFF8)
                                .then_some(stored.device.ieee_address)
                        })
                    }) {
                        self.ensure_counter_available(device, &address)?;
                        let destination = self.device_short(&address)?;
                        device
                            .aps_mut()
                            .send_transport_key(
                                destination,
                                &address,
                                0x01,
                                &key,
                                rotation.target_sequence,
                                &local_ieee,
                            )
                            .await
                            .map_err(TrustCenterRuntimeError::Aps)?;
                        self.checkpoint_live_counters(device)?;
                        let mut state = self
                            .state
                            .clone()
                            .ok_or(TrustCenterRuntimeError::NotJoined)?;
                        state.mark_network_key_delivered(&address)?;
                        self.store.store(&state)?;
                        self.state = Some(state);
                        return Ok(());
                    }
                    self.set_network_key_rotation_phase(
                        NetworkKeyRotationPhase::WaitingForPropagation,
                    )?;
                    self.network_key_propagation_started = Some(device.mac().monotonic_micros());
                }
                NetworkKeyRotationPhase::WaitingForPropagation => {
                    let now = device.mac().monotonic_micros();
                    let started = *self.network_key_propagation_started.get_or_insert(now);
                    let has_peers = device
                        .aps()
                        .nwk()
                        .neighbor_table()
                        .children()
                        .next()
                        .is_some()
                        || self.state.as_ref().is_some_and(|state| {
                            state.devices().iter().any(|stored| {
                                !stored.removal_pending && stored.device.short_address.0 < 0xFFF8
                            })
                        });
                    let grace_us = NETWORK_KEY_PROPAGATION_DELAY_US
                        .max(device.network_key_forwarding_window_us());
                    if has_peers && now.wrapping_sub(started) < grace_us {
                        return Ok(());
                    }
                    self.set_network_key_rotation_phase(NetworkKeyRotationPhase::Switching)?;
                }
                NetworkKeyRotationPhase::Switching => {
                    device
                        .aps_mut()
                        .send_switch_key(
                            ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
                            &[0xFF; 8],
                            rotation.target_sequence,
                        )
                        .await
                        .map_err(TrustCenterRuntimeError::Aps)?;
                    self.set_network_key_rotation_phase(NetworkKeyRotationPhase::Activating)?;
                }
                NetworkKeyRotationPhase::Activating => return Ok(()),
            }
        }
    }

    fn set_network_key_rotation_phase(
        &mut self,
        phase: NetworkKeyRotationPhase,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let rotation = state
            .rotation()
            .ok_or(TrustCenterRuntimeError::NetworkKeyRotationNotPrepared)?;
        state.set_rotation(Some(NetworkKeyRotation { phase, ..rotation }));
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    async fn resume_network_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let stored = *self
            .state
            .as_ref()
            .and_then(|state| state.device(&address))
            .ok_or(TrustCenterError::DeviceNotFound)?;
        if !stored.network_key_pending {
            return Ok(());
        }
        // Restored neighbor authorization may belong to an earlier join.
        // Only a fresh authenticated announcement/security exchange closes
        // this intent; neither a cached neighbor nor a MAC poll proves receipt.
        let now = device.mac().monotonic_micros();
        if self.initial_key_attempts.iter().any(|(peer, sent)| {
            *peer == address && now.wrapping_sub(*sent) < INITIAL_NETWORK_KEY_RETRY_US
        }) {
            return Ok(());
        }
        self.ensure_counter_available(device, &address)?;
        let network_key = device
            .aps()
            .nwk()
            .security()
            .active_key()
            .cloned()
            .ok_or(TrustCenterRuntimeError::MissingNetworkKey)?;
        let local_ieee = device.aps().nwk().nib().ieee_address;
        let parent_short = if stored.device.parent_address == local_ieee {
            ShortAddress::COORDINATOR
        } else {
            self.device_short(&stored.device.parent_address)?
        };
        device
            .aps_mut()
            .send_initial_network_key_to(
                parent_short,
                &stored.device.parent_address,
                stored.device.short_address,
                &address,
                &network_key.key,
                network_key.seq_number,
            )
            .await
            .map_err(TrustCenterRuntimeError::Aps)?;
        self.checkpoint_live_counters(device)?;
        if let Some((_, sent)) = self
            .initial_key_attempts
            .iter_mut()
            .find(|(peer, _)| *peer == address)
        {
            *sent = now;
        } else {
            self.initial_key_attempts
                .push((address, now))
                .map_err(|_| TrustCenterRuntimeError::Store(TrustCenterStoreError::Full))?;
        }
        Ok(())
    }

    /// Queue acceptance (including a remote parent's Tunnel acceptance) is
    /// not delivery. Only authenticated child traffic closes the durable intent.
    fn complete_initial_network_key(
        &mut self,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if let Some(stored) = state.device_mut(&address)
            && stored.network_key_pending
        {
            stored.network_key_pending = false;
            self.store.store(&state)?;
            self.state = Some(state);
        }
        self.initial_key_attempts
            .retain(|(peer, _)| *peer != address);
        Ok(())
    }

    async fn resume_trust_center_link_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let stored = *self
            .state
            .as_ref()
            .and_then(|state| state.device(&address))
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let Some(pending) = stored.pending_link_key else {
            return Ok(());
        };
        if pending.transported {
            return Ok(());
        }
        self.ensure_counter_available(device, &address)?;
        let local_ieee = device.aps().nwk().nib().ieee_address;
        device
            .aps_mut()
            .send_transport_key(
                stored.device.short_address,
                &address,
                0x04,
                &pending.key,
                0,
                &local_ieee,
            )
            .await
            .map_err(TrustCenterRuntimeError::Aps)?;

        let join_timeout = device.bdb().attributes().trust_center_node_join_timeout;
        self.table
            .install_generated_trust_center_link_key(address, pending.key, join_timeout)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let stored = state
            .device_mut(&address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        stored
            .pending_link_key
            .as_mut()
            .ok_or(TrustCenterError::DeviceNotFound)?
            .transported = true;
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    async fn resume_application_link_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        let Some(mut pending) = self
            .state
            .as_ref()
            .and_then(PersistentTrustCenterState::pending_application_key)
        else {
            return Ok(());
        };
        if !pending.delivered_to_initiator {
            match self.delivery_progress(
                TrustCenterDeliveryKind::ApplicationKeyInitiator,
                pending.initiator_address,
            ) {
                DeliveryProgress::Busy => return Ok(()),
                DeliveryProgress::Submitted => {
                    pending.delivered_to_initiator = true;
                    let mut state = self
                        .state
                        .clone()
                        .ok_or(TrustCenterRuntimeError::NotJoined)?;
                    state.set_pending_application_key(Some(pending));
                    self.store.store(&state)?;
                    self.state = Some(state);
                    self.cancel_delivery();
                }
                DeliveryProgress::Ready => {
                    self.ensure_counter_available(device, &pending.initiator_address)?;
                    let initiator_short = self.device_short(&pending.initiator_address)?;
                    device
                        .aps_mut()
                        .send_application_link_key_to(
                            initiator_short,
                            &pending.initiator_address,
                            &pending.responder_address,
                            &pending.key,
                            true,
                        )
                        .await
                        .map_err(TrustCenterRuntimeError::Aps)?;
                    self.start_delivery(
                        TrustCenterDeliveryKind::ApplicationKeyInitiator,
                        pending.initiator_address,
                    );
                    self.checkpoint_live_counters(device)?;
                    return Ok(());
                }
            }
        }
        if !pending.delivered_to_responder {
            match self.delivery_progress(
                TrustCenterDeliveryKind::ApplicationKeyResponder,
                pending.responder_address,
            ) {
                DeliveryProgress::Busy => return Ok(()),
                DeliveryProgress::Submitted => {
                    pending.delivered_to_responder = true;
                    let mut state = self
                        .state
                        .clone()
                        .ok_or(TrustCenterRuntimeError::NotJoined)?;
                    state.set_pending_application_key(Some(pending));
                    self.store.store(&state)?;
                    self.state = Some(state);
                    self.cancel_delivery();
                }
                DeliveryProgress::Ready => {
                    self.ensure_counter_available(device, &pending.responder_address)?;
                    let responder_short = self.device_short(&pending.responder_address)?;
                    device
                        .aps_mut()
                        .send_application_link_key_to(
                            responder_short,
                            &pending.responder_address,
                            &pending.initiator_address,
                            &pending.key,
                            false,
                        )
                        .await
                        .map_err(TrustCenterRuntimeError::Aps)?;
                    self.start_delivery(
                        TrustCenterDeliveryKind::ApplicationKeyResponder,
                        pending.responder_address,
                    );
                    self.checkpoint_live_counters(device)?;
                    return Ok(());
                }
            }
        }
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        state.set_pending_application_key(None);
        self.store.store(&state)?;
        self.state = Some(state);
        Ok(())
    }

    fn upsert_table_device<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: &IeeeAddress,
        network_key_pending: bool,
    ) -> Result<(), TrustCenterRuntimeError> {
        let table_device = *self
            .table
            .device(address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if let Some(stored) = state.device_mut(address) {
            if stored.device.link_key != table_device.link_key {
                let start = stored.outgoing_frame_counter_limit;
                stored.outgoing_frame_counter = start;
                stored.outgoing_frame_counter_limit = reserve_end(start)?;
                stored.incoming_frame_counter = 0;
                stored.incoming_frame_counter_valid = false;
                stored.pending_link_key = None;
                stored.confirm_key_pending = None;
            }
            stored.device = table_device;
            stored.network_key_pending |= network_key_pending;
            if network_key_pending {
                stored.is_router = None;
            }
        } else {
            state.upsert(PersistentTrustCenterDevice {
                device: table_device,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: APS_COUNTER_RESERVATION,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
                network_key_pending,
                confirm_key_pending: None,
                removal_pending: false,
                new_network_key_delivered: false,
                is_router: None,
                pending_link_key: None,
            })?;
        }
        if let Err(error) = self.store.store(&state) {
            self.restore_policy_device_from_state(address)?;
            return Err(error.into());
        }
        self.state = Some(state);
        if network_key_pending {
            self.initial_key_attempts
                .retain(|(peer, _)| peer != address);
        }
        self.install_live_key(device, *address)
    }

    fn checkpoint_table<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let mut changed = false;
        for table_device in self.table.devices() {
            if let Some(stored) = state.device_mut(&table_device.ieee_address) {
                if stored.pending_link_key.is_none() {
                    if stored.device != *table_device {
                        stored.device = *table_device;
                        changed = true;
                    }
                } else {
                    if stored.device.parent_address != table_device.parent_address {
                        stored.device.parent_address = table_device.parent_address;
                        changed = true;
                    }
                    if stored.device.short_address != table_device.short_address {
                        stored.device.short_address = table_device.short_address;
                        changed = true;
                    }
                    if stored.device.join_timeout_remaining_secs
                        != table_device.join_timeout_remaining_secs
                    {
                        stored.device.join_timeout_remaining_secs =
                            table_device.join_timeout_remaining_secs;
                        changed = true;
                    }
                }
            }
        }
        changed |= sync_live_counters(&mut state, device);
        if changed {
            self.store.store(&state)?;
            self.state = Some(state);
        }
        Ok(())
    }

    fn checkpoint_live_counters<M: MacDriver>(
        &mut self,
        device: &ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if sync_live_counters(&mut state, device) {
            self.store.store(&state)?;
            self.state = Some(state);
        }
        Ok(())
    }

    fn ensure_counter_available<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: &IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let exhausted = device
            .aps()
            .security()
            .find_key(address, ApsKeyType::TrustCenterLinkKey)
            .is_none_or(|entry| {
                entry.outgoing_frame_counter.saturating_add(1) >= entry.outgoing_frame_counter_limit
            });
        if exhausted {
            let mut state = self
                .state
                .clone()
                .ok_or(TrustCenterRuntimeError::NotJoined)?;
            let stored = state
                .device_mut(address)
                .ok_or(TrustCenterError::DeviceNotFound)?;
            let start = stored.outgoing_frame_counter_limit;
            stored.outgoing_frame_counter = start;
            stored.outgoing_frame_counter_limit = reserve_end(start)?;
            self.store.store(&state)?;
            self.state = Some(state);
            self.install_live_key(device, *address)?;
        }
        Ok(())
    }

    fn restore_policy_device_from_state(
        &mut self,
        address: &IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let state = self
            .state
            .as_ref()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        if let Some(stored) = state.device(address) {
            self.table.restore_device(stored.device)?;
            if let Some(pending) = stored.pending_link_key {
                self.table.install_generated_trust_center_link_key(
                    *address,
                    pending.key,
                    stored.device.join_timeout_remaining_secs.max(1),
                )?;
            }
        } else {
            self.table.revoke(address);
        }
        Ok(())
    }

    fn install_live_key<M: MacDriver>(
        &self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let stored = self
            .state
            .as_ref()
            .and_then(|state| state.device(&address))
            .ok_or(TrustCenterError::DeviceNotFound)?;
        install_key(device, stored)
    }

    fn revoke<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: &IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut cancel_application_delivery = false;
        if let Some(mut state) = self.state.clone() {
            let mut changed = false;
            if state.pending_application_key().is_some_and(|pending| {
                pending.initiator_address == *address || pending.responder_address == *address
            }) {
                state.set_pending_application_key(None);
                cancel_application_delivery = true;
                changed = true;
            }
            changed |= state.remove_application_key_requests_for(address);
            changed |= state.remove(address);
            changed |= state.remove_unknown_removal(address);
            if changed {
                self.store.store(&state)?;
                self.state = Some(state);
            }
        }
        if self.pending_delivery.is_some_and(|pending| {
            pending.address == *address
                || (pending.kind == TrustCenterDeliveryKind::RemoveDevice
                    && self.state.as_ref().is_some_and(|state| {
                        state.pending_unknown_removals().iter().any(|removal| {
                            removal.parent_address == *address
                                && removal.device_address == pending.address
                        })
                    }))
                || (cancel_application_delivery
                    && matches!(
                        pending.kind,
                        TrustCenterDeliveryKind::ApplicationKeyInitiator
                            | TrustCenterDeliveryKind::ApplicationKeyResponder
                    ))
        }) {
            self.cancel_delivery();
        }
        self.table.revoke(address);
        self.initial_key_attempts
            .retain(|(peer, _)| peer != address);
        device
            .aps_mut()
            .security_mut()
            .remove_key(address, ApsKeyType::TrustCenterLinkKey);
        Ok(())
    }

    pub const fn pending_device_replay_tombstone(&self) -> Option<IeeeAddress> {
        self.pending_device_replay_tombstone
    }

    /// Finish a device revocation only after its replay domains were durably
    /// tombstoned by the caller.
    pub fn complete_pending_device_revocation<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        if self.pending_device_replay_tombstone != Some(address) {
            return Err(TrustCenterRuntimeError::Policy(
                TrustCenterError::RequestNotPermitted,
            ));
        }
        self.revoke(device, &address)?;
        self.pending_device_replay_tombstone = None;
        Ok(())
    }

    fn commit_and_stage_confirm_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
        status: u8,
    ) -> Result<(), TrustCenterRuntimeError> {
        let table_device = *self
            .table
            .device(&address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let stored = state
            .device_mut(&address)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        let previous_status = stored.confirm_key_pending;
        if status == 0 {
            if let Some(pending) = stored.pending_link_key {
                if !pending.transported || table_device.link_key != pending.key {
                    return Err(TrustCenterRuntimeError::Policy(
                        TrustCenterError::RequestNotPermitted,
                    ));
                }
                stored.device = table_device;
                stored.outgoing_frame_counter = pending.outgoing_frame_counter;
                stored.outgoing_frame_counter_limit = pending.outgoing_frame_counter_limit;
                stored.incoming_frame_counter = 0;
                stored.incoming_frame_counter_valid = false;
                stored.pending_link_key = None;
            } else if stored.device.link_key != table_device.link_key
                || table_device.key_attributes
                    != zigbee_bdb::trust_center::TrustCenterKeyAttributes::Verified
            {
                return Err(TrustCenterRuntimeError::Policy(
                    TrustCenterError::RequestNotPermitted,
                ));
            } else {
                stored.device = table_device;
            }
            stored.network_key_pending = false;
        }
        stored.confirm_key_pending = Some(status);
        if let Err(error) = self.store.store(&state) {
            // Verify-Key mutates the policy table first. Restore it too if
            // the verified key and response obligation cannot be committed.
            self.restore_policy_device_from_state(&address)?;
            return Err(error.into());
        }
        self.state = Some(state);
        if previous_status.is_some_and(|previous| previous != status)
            && self.pending_delivery.is_some_and(|pending| {
                pending.kind == TrustCenterDeliveryKind::ConfirmKey && pending.address == address
            })
        {
            self.cancel_delivery();
        }
        if status == 0 {
            self.install_live_key(device, address)?;
            self.initial_key_attempts
                .retain(|(peer, _)| *peer != address);
        }
        Ok(())
    }

    async fn resume_confirm_key<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let status = self
            .state
            .as_ref()
            .and_then(|state| state.device(&address))
            .and_then(|stored| stored.confirm_key_pending)
            .ok_or(TrustCenterError::DeviceNotFound)?;
        match self.delivery_progress(TrustCenterDeliveryKind::ConfirmKey, address) {
            DeliveryProgress::Busy => return Ok(()),
            DeliveryProgress::Submitted => {
                let mut state = self
                    .state
                    .clone()
                    .ok_or(TrustCenterRuntimeError::NotJoined)?;
                sync_live_counters(&mut state, device);
                state
                    .device_mut(&address)
                    .ok_or(TrustCenterError::DeviceNotFound)?
                    .confirm_key_pending = None;
                self.store.store(&state)?;
                self.state = Some(state);
                self.cancel_delivery();
                return Ok(());
            }
            DeliveryProgress::Ready => {}
        }
        if status == 0 {
            self.ensure_counter_available(device, &address)?;
        }
        let destination_short = self.device_short(&address)?;
        device
            .aps_mut()
            .send_confirm_key_to(destination_short, &address, status, 0x04)
            .await
            .map_err(TrustCenterRuntimeError::Aps)?;
        self.start_delivery(TrustCenterDeliveryKind::ConfirmKey, address);
        self.checkpoint_live_counters(device)?;
        Ok(())
    }

    fn stage_removal<M: MacDriver>(
        &mut self,
        _device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
        parent_address: IeeeAddress,
        short_address: ShortAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let mut state = self
            .state
            .clone()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let pending_application_key = state.pending_application_key();
        {
            let stored = state
                .device_mut(&address)
                .ok_or(TrustCenterError::DeviceNotFound)?;
            stored.device.parent_address = parent_address;
            stored.device.short_address = short_address;
            stored.network_key_pending = false;
            stored.pending_link_key = None;
            stored.confirm_key_pending = None;
            stored.removal_pending = true;
        }
        if state.pending_application_key().is_some_and(|pending| {
            pending.initiator_address == address || pending.responder_address == address
        }) {
            state.set_pending_application_key(None);
        }
        state.remove_application_key_requests_for(&address);
        self.store.store(&state)?;
        self.state = Some(state);
        let cancel_delivery = self.pending_delivery.is_some_and(|pending| {
            pending.kind != TrustCenterDeliveryKind::RemoveDevice
                && (pending.address == address
                    || pending_application_key.is_some_and(|application| {
                        (application.initiator_address == address
                            || application.responder_address == address)
                            && matches!(
                                pending.kind,
                                TrustCenterDeliveryKind::ApplicationKeyInitiator
                                    | TrustCenterDeliveryKind::ApplicationKeyResponder
                            )
                    }))
        });
        if cancel_delivery {
            self.cancel_delivery();
        }
        Ok(())
    }

    async fn resume_removal<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let stored = *self
            .state
            .as_ref()
            .and_then(|state| state.device(&address))
            .ok_or(TrustCenterError::DeviceNotFound)?;
        if !stored.removal_pending {
            return Ok(());
        }
        if self.pending_device_replay_tombstone == Some(address) {
            return Ok(());
        }
        let local_ieee = device.aps().nwk().nib().ieee_address;
        if stored.device.parent_address == local_ieee {
            if device.aps().nwk().known_child_by_ieee(&address).is_some() {
                return Ok(());
            }
            self.pending_device_replay_tombstone = Some(address);
            return Ok(());
        }
        match self.delivery_progress(TrustCenterDeliveryKind::RemoveDevice, address) {
            DeliveryProgress::Busy => Ok(()),
            DeliveryProgress::Submitted => {
                // Revocation is local TC policy, not proof that the child left.
                self.pending_device_replay_tombstone = Some(address);
                Ok(())
            }
            DeliveryProgress::Ready => {
                self.send_remote_remove_device(device, stored.device.parent_address, address)
                    .await?;
                self.start_delivery(TrustCenterDeliveryKind::RemoveDevice, address);
                self.checkpoint_live_counters(device)?;
                Ok(())
            }
        }
    }

    fn stage_unknown_removal<M: MacDriver>(
        &mut self,
        _device: &mut ZigbeeDevice<M>,
        parent_address: IeeeAddress,
        device_address: IeeeAddress,
    ) -> Result<(), TrustCenterRuntimeError> {
        let state = self
            .state
            .as_ref()
            .ok_or(TrustCenterRuntimeError::NotJoined)?;
        let pending = PendingUnknownRemoval {
            parent_address,
            device_address,
        };
        if state.pending_unknown_removals().contains(&pending) {
            return Ok(());
        }
        // Publish only after the journal commits. Otherwise a subsequent poll
        // could transmit a rejection whose failed write was lost.
        let mut next = state.clone();
        next.stage_unknown_removal(pending)?;
        self.store.store(&next)?;
        self.state = Some(next);
        if self.pending_delivery.is_some_and(|delivery| {
            delivery.kind == TrustCenterDeliveryKind::RemoveDevice
                && delivery.address == device_address
        }) {
            // Submission to the old parent cannot complete the new send intent.
            self.cancel_delivery();
        }
        Ok(())
    }

    async fn resume_unknown_removal<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), TrustCenterRuntimeError> {
        let Some(pending) = self
            .state
            .as_ref()
            .and_then(|state| state.pending_unknown_removals().first())
            .copied()
        else {
            return Ok(());
        };
        if self.pending_device_replay_tombstone.is_some() {
            return Ok(());
        }
        let local_ieee = device.aps().nwk().nib().ieee_address;
        if pending.parent_address == local_ieee {
            if device
                .aps()
                .nwk()
                .known_child_by_ieee(&pending.device_address)
                .is_some()
            {
                return Ok(());
            }
            self.pending_device_replay_tombstone = Some(pending.device_address);
            return Ok(());
        }
        if self.table.device(&pending.parent_address).is_none() {
            // A durably revoked parent has no delivery route/key left. The
            // peer stays unadmitted; retire its rejection rather than trying
            // to transmit through a nonexistent TC device record.
            self.pending_device_replay_tombstone = Some(pending.device_address);
            return Ok(());
        }
        match self.delivery_progress(
            TrustCenterDeliveryKind::RemoveDevice,
            pending.device_address,
        ) {
            DeliveryProgress::Busy => Ok(()),
            DeliveryProgress::Submitted => {
                self.pending_device_replay_tombstone = Some(pending.device_address);
                Ok(())
            }
            DeliveryProgress::Ready => {
                self.send_remote_remove_device(
                    device,
                    pending.parent_address,
                    pending.device_address,
                )
                .await?;
                self.start_delivery(
                    TrustCenterDeliveryKind::RemoveDevice,
                    pending.device_address,
                );
                self.checkpoint_live_counters(device)?;
                Ok(())
            }
        }
    }

    fn defer_transport_error(
        &mut self,
        result: Result<(), TrustCenterRuntimeError>,
    ) -> Result<(), TrustCenterRuntimeError> {
        match result {
            Ok(()) => Ok(()),
            Err(error @ (TrustCenterRuntimeError::Aps(_) | TrustCenterRuntimeError::Nwk(_))) => {
                log::warn!("Trust Center transaction deferred: {error:?}");
                self.last_transport_error = Some(error);
                Ok(())
            }
            Err(error) => Err(error),
        }
    }

    fn delivery_progress(
        &self,
        kind: TrustCenterDeliveryKind,
        address: IeeeAddress,
    ) -> DeliveryProgress {
        let Some(pending) = self.pending_delivery else {
            return DeliveryProgress::Ready;
        };
        if pending.kind != kind || pending.address != address {
            return DeliveryProgress::Busy;
        }
        DeliveryProgress::Submitted
    }

    fn start_delivery(&mut self, kind: TrustCenterDeliveryKind, address: IeeeAddress) {
        // R22 security commands use AR=0. This records only local NLDE
        // acceptance, including indirect enqueue, never peer installation.
        // Keep the marker until the journal commits so a failed write does
        // not cause another Confirm-Key request and reset replay state again.
        self.pending_delivery = Some(PendingTrustCenterDelivery { kind, address });
    }

    fn cancel_delivery(&mut self) {
        self.pending_delivery = None;
    }

    fn device_short(&self, address: &IeeeAddress) -> Result<ShortAddress, TrustCenterRuntimeError> {
        self.state
            .as_ref()
            .and_then(|state| state.device(address))
            .map(|stored| stored.device.short_address)
            .ok_or_else(|| TrustCenterError::DeviceNotFound.into())
    }
}

fn sync_live_counters<M: MacDriver>(
    state: &mut PersistentTrustCenterState,
    device: &ZigbeeDevice<M>,
) -> bool {
    let mut changed = false;
    for stored in state.devices_mut() {
        let Some(entry) = device
            .aps()
            .security()
            .find_key(&stored.device.ieee_address, ApsKeyType::TrustCenterLinkKey)
        else {
            continue;
        };
        let outgoing_frame_counter = entry
            .outgoing_frame_counter
            .min(entry.outgoing_frame_counter_limit.saturating_sub(1));
        if stored.outgoing_frame_counter != outgoing_frame_counter {
            stored.outgoing_frame_counter = outgoing_frame_counter;
            changed = true;
        }
        if stored.outgoing_frame_counter_limit != entry.outgoing_frame_counter_limit {
            stored.outgoing_frame_counter_limit = entry.outgoing_frame_counter_limit;
            changed = true;
        }
        if stored.incoming_frame_counter != entry.incoming_frame_counter {
            stored.incoming_frame_counter = entry.incoming_frame_counter;
            changed = true;
        }
        if stored.incoming_frame_counter_valid != entry.incoming_frame_counter_valid {
            stored.incoming_frame_counter_valid = entry.incoming_frame_counter_valid;
            changed = true;
        }
    }
    changed
}

fn validate_coordinator<M: MacDriver>(
    device: &ZigbeeDevice<M>,
) -> Result<IeeeAddress, TrustCenterRuntimeError> {
    if device.device_type() != DeviceType::Coordinator {
        return Err(TrustCenterRuntimeError::NotCoordinator);
    }
    if !device.is_joined() {
        return Err(TrustCenterRuntimeError::NotJoined);
    }
    let extended_pan_id = device.aps().nwk().nib().extended_pan_id;
    if extended_pan_id == [0u8; 8] || extended_pan_id == [0xFFu8; 8] {
        return Err(TrustCenterRuntimeError::NotJoined);
    }
    Ok(extended_pan_id)
}

fn reserve_restored_ranges(
    state: &mut PersistentTrustCenterState,
) -> Result<(), TrustCenterRuntimeError> {
    for stored in state.devices_mut() {
        let start = stored.outgoing_frame_counter_limit;
        stored.outgoing_frame_counter = start;
        stored.outgoing_frame_counter_limit = reserve_end(start)?;
        if let Some(pending) = stored.pending_link_key.as_mut() {
            let start = pending.outgoing_frame_counter_limit;
            pending.outgoing_frame_counter = start;
            pending.outgoing_frame_counter_limit = reserve_end(start)?;
        }
    }
    Ok(())
}

fn reserve_end(start: u32) -> Result<u32, TrustCenterRuntimeError> {
    start
        .checked_add(APS_COUNTER_RESERVATION)
        .ok_or(TrustCenterRuntimeError::CounterExhausted)
}

fn install_key<M: MacDriver>(
    device: &mut ZigbeeDevice<M>,
    stored: &PersistentTrustCenterDevice,
) -> Result<(), TrustCenterRuntimeError> {
    let reservation = ApsFrameCounterReservation::new(
        stored.outgoing_frame_counter,
        stored.outgoing_frame_counter_limit,
    )
    .ok_or(TrustCenterRuntimeError::CounterExhausted)?;
    let entry = ApsLinkKeyEntry {
        partner_address: stored.device.ieee_address,
        key: stored.device.link_key,
        key_type: ApsKeyType::TrustCenterLinkKey,
        outgoing_frame_counter: reservation.current(),
        outgoing_frame_counter_limit: reservation.limit(),
        incoming_frame_counter: stored.incoming_frame_counter,
        incoming_frame_counter_valid: stored.incoming_frame_counter_valid,
    };
    device
        .aps_mut()
        .security_mut()
        .add_key(entry)
        .map_err(|_| TrustCenterRuntimeError::KeyTableFull)
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::task::{Context, Poll, Waker};

    use super::*;
    use crate::security_store::{RamSecurityStateStore, SecurityStateStore};
    use crate::trust_center_store::{RamTrustCenterDeviceStore, TrustCenterDeviceStore};
    use zigbee_aps::apsme::{
        ApsRequestKeyType, ApsUpdateDeviceStatus, ApsmeRemoveDeviceIndication,
        ApsmeRequestKeyIndication, ApsmeUpdateDeviceIndication, ApsmeVerifyKeyIndication,
    };
    use zigbee_aps::security::{ApsKeyType, DEFAULT_TC_LINK_KEY, derive_verify_key_hash};
    use zigbee_bdb::trust_center::{TrustCenterKeyAttributes, TrustCenterKeyOrigin};
    use zigbee_mac::mock::MockMac;
    use zigbee_mac::{CapabilityInfo, EdValue, PlatformServices};
    use zigbee_types::ChannelMask;

    const TC: IeeeAddress = [0x10; 8];
    const CHILD_A: IeeeAddress = [0x21; 8];
    const CHILD_B: IeeeAddress = [0x22; 8];

    #[derive(Debug, Default)]
    struct CountingStore {
        inner: RamTrustCenterDeviceStore,
        stores: usize,
    }

    impl TrustCenterDeviceStore for CountingStore {
        fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
            self.inner.load()
        }

        fn store(
            &mut self,
            state: &PersistentTrustCenterState,
        ) -> Result<(), TrustCenterStoreError> {
            self.stores += 1;
            self.inner.store(state)
        }

        fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
            self.inner.clear()
        }
    }

    #[derive(Debug, Default)]
    struct FailingStore {
        inner: RamTrustCenterDeviceStore,
        fail_next_store: bool,
    }

    impl TrustCenterDeviceStore for FailingStore {
        fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
            self.inner.load()
        }

        fn store(
            &mut self,
            state: &PersistentTrustCenterState,
        ) -> Result<(), TrustCenterStoreError> {
            if self.fail_next_store {
                self.fail_next_store = false;
                return Err(TrustCenterStoreError::Hardware);
            }
            self.inner.store(state)
        }

        fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
            self.inner.clear()
        }
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(Waker::noop());
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    fn coordinator_with_security_store() -> (ZigbeeDevice<MockMac>, RamSecurityStateStore) {
        let mut mac = MockMac::new(TC);
        mac.add_energy(EdValue {
            channel: 11,
            energy: 80,
        });
        mac.add_energy(EdValue {
            channel: 15,
            energy: 10,
        });
        let mut device = crate::ZigbeeDevice::builder(mac)
            .channels(ChannelMask((1u32 << 11) | (1u32 << 15)))
            .build_coordinator();
        let mut security_store = RamSecurityStateStore::new();
        block_on(device.start_or_resume_coordinator_with_security_store(&mut security_store))
            .unwrap();
        device.bdb_mut().attributes_mut().trust_center_allow_joins = true;
        device.aps_mut().nwk_mut().nib_mut().permit_joining = true;
        device.mac_mut().clear_tx_history();
        (device, security_store)
    }

    fn coordinator() -> ZigbeeDevice<MockMac> {
        coordinator_with_security_store().0
    }

    fn restored_coordinator(security_store: &mut RamSecurityStateStore) -> ZigbeeDevice<MockMac> {
        let mut device = crate::ZigbeeDevice::builder(MockMac::new(TC)).build_coordinator();
        block_on(device.start_or_resume_coordinator_with_security_store(security_store)).unwrap();
        device.bdb_mut().attributes_mut().trust_center_allow_joins = true;
        device.aps_mut().nwk_mut().nib_mut().permit_joining = true;
        device.mac_mut().clear_tx_history();
        device
    }

    fn begin_rotation<S: TrustCenterDeviceStore>(
        runtime: &mut TrustCenterRuntime<S>,
        device: &mut ZigbeeDevice<MockMac>,
        security_store: &mut RamSecurityStateStore,
        method: NetworkKeyUpdateMethod,
    ) -> u8 {
        let target = runtime.prepare_network_key_rotation(device).unwrap();
        assert!(device.refresh_security_state(security_store).unwrap());
        runtime
            .commit_network_key_rotation(device, target, method)
            .unwrap();
        target
    }

    fn add_child(device: &mut ZigbeeDevice<MockMac>, address: IeeeAddress) -> ShortAddress {
        device
            .aps_mut()
            .nwk_mut()
            .handle_child_association(
                address,
                CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: true,
                    rx_on_when_idle: true,
                    security_capable: true,
                    allocate_address: true,
                }
                .to_byte(),
            )
            .unwrap()
    }

    fn begin_join<S: TrustCenterDeviceStore>(
        runtime: &mut TrustCenterRuntime<S>,
        device: &mut ZigbeeDevice<MockMac>,
        address: IeeeAddress,
        short: ShortAddress,
    ) {
        block_on(runtime.handle_indication(
            device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: TC,
                device_address: address,
                device_short_address: short,
                status: ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
            }),
        ))
        .unwrap();
    }

    fn join<S: TrustCenterDeviceStore>(
        runtime: &mut TrustCenterRuntime<S>,
        device: &mut ZigbeeDevice<MockMac>,
        address: IeeeAddress,
        short: ShortAddress,
    ) {
        begin_join(runtime, device, address, short);
        assert!(device.aps_mut().nwk_mut().authorize_child(short));
        runtime
            .handle_device_announce(device, address, short, 0x8C)
            .unwrap();
        block_on(runtime.poll(device)).unwrap();
    }

    fn add_sleepy_child(device: &mut ZigbeeDevice<MockMac>, address: IeeeAddress) -> ShortAddress {
        device
            .aps_mut()
            .nwk_mut()
            .handle_child_association(
                address,
                CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: false,
                    rx_on_when_idle: false,
                    security_capable: true,
                    allocate_address: true,
                }
                .to_byte(),
            )
            .unwrap()
    }

    fn poll_child(device: &mut ZigbeeDevice<MockMac>, short: ShortAddress) {
        let pan = device.aps().nwk().nib().pan_id;
        device
            .mac_mut()
            .enqueue_command_event(zigbee_mac::MacCommandEvent::DataRequest(
                zigbee_mac::MlmeDataRequestIndication {
                    source_address: zigbee_types::MacAddress::Short(pan, short),
                    destination_address: zigbee_types::MacAddress::Short(
                        pan,
                        ShortAddress::COORDINATOR,
                    ),
                    lqi: 200,
                    security_use: false,
                },
            ));
        block_on(device.service_parent_commands());
    }

    #[test]
    fn initial_sleepy_key_intent_survives_queue_loss_and_mac_delivery() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let short = add_sleepy_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        assert!(device.mac().tx_history().is_empty());
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        block_on(runtime.poll(&mut device)).unwrap();
        let saved = runtime.store_mut().load().unwrap().unwrap();
        let mut reboot_store = RamTrustCenterDeviceStore::new();
        reboot_store.store(&saved).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        assert_eq!(add_sleepy_child(&mut rebooted, CHILD_A), short);
        let mut runtime = TrustCenterRuntime::new(reboot_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert!(rebooted.mac().tx_history().is_empty());

        poll_child(&mut rebooted, short);
        assert_eq!(rebooted.mac().tx_history().len(), 1);
        assert!(rebooted.mac().tx_history()[0].indirect);
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending,
            "even MAC delivery is not proof of key installation"
        );
        poll_child(&mut rebooted, short);
        assert_eq!(
            rebooted.mac().tx_history().len(),
            1,
            "ordinary polls do not multiply retries"
        );
        block_on(
            rebooted
                .mac_mut()
                .delay_micros(INITIAL_NETWORK_KEY_RETRY_US),
        );
        block_on(runtime.poll(&mut rebooted)).unwrap();
        poll_child(&mut rebooted, short);
        assert_eq!(
            rebooted.mac().tx_history().len(),
            2,
            "lost delivery is retried after a bounded interval"
        );

        assert!(rebooted.aps_mut().nwk_mut().authorize_child(short));
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        runtime
            .handle_device_announce(&rebooted, CHILD_A, short, 0x80)
            .unwrap();
        assert!(
            !runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
    }

    #[test]
    fn initial_key_intent_creation_failure_never_sends_or_admits() {
        let mut device = coordinator();
        let short = add_sleepy_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            block_on(runtime.handle_indication(
                &mut device,
                ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                    source_address: TC,
                    device_address: CHILD_A,
                    device_short_address: short,
                    status: ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
                }),
            )),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        block_on(device.mac_mut().delay_micros(1_000_000));
        block_on(runtime.poll(&mut device)).unwrap();
        poll_child(&mut device, short);
        assert!(device.mac().tx_history().is_empty());
        assert!(runtime.table().device(&CHILD_A).is_none());
        assert!(runtime.state.as_ref().unwrap().device(&CHILD_A).is_none());
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .is_none()
        );
        assert!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .is_none()
        );
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        poll_child(&mut device, short);
        assert_eq!(device.mac().tx_history().len(), 1);
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
    }

    #[test]
    fn initial_key_completion_failure_keeps_live_and_durable_intent() {
        let mut device = coordinator();
        let short = add_sleepy_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        poll_child(&mut device, short);
        assert!(device.aps_mut().nwk_mut().authorize_child(short));
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            runtime.handle_device_announce(&device, CHILD_A, short, 0x80),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        runtime
            .handle_device_announce(&device, CHILD_A, short, 0x80)
            .unwrap();
        assert!(
            !runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
    }

    #[test]
    fn new_initial_key_intent_does_not_complete_from_restored_authorization() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, short);
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        assert!(device.aps().nwk().child_is_authorized(&CHILD_A));
        assert!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        let store = core::mem::take(runtime.store_mut());

        let mut rebooted = restored_coordinator(&mut security_store);
        assert!(
            rebooted
                .aps_mut()
                .nwk_mut()
                .restore_child(CHILD_A, short, true, true, false, 8)
        );
        let mut runtime = TrustCenterRuntime::new(store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
        runtime
            .handle_device_announce(&rebooted, CHILD_A, short, 0x8C)
            .unwrap();
        assert!(
            !runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );
    }

    #[test]
    fn device_announce_completion_is_atomic_and_does_not_admit_unknown_peers() {
        let mut device = coordinator();
        let short = add_sleepy_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        runtime.store_mut().fail_next_store = true;
        runtime
            .handle_device_announce(&device, CHILD_B, ShortAddress(0x1234), 0x8E)
            .unwrap();
        assert!(runtime.store().fail_next_store);
        assert!(runtime.state.as_ref().unwrap().device(&CHILD_B).is_none());
        assert_eq!(
            runtime.handle_device_announce(&device, CHILD_A, short, 0x8E),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        let live = runtime.state.as_ref().unwrap().device(&CHILD_A).unwrap();
        assert!(live.network_key_pending);
        assert_eq!(live.is_router, None);
        let durable = runtime.store_mut().load().unwrap().unwrap();
        assert!(durable.device(&CHILD_A).unwrap().network_key_pending);
        assert_eq!(durable.device(&CHILD_A).unwrap().is_router, None);
        runtime
            .handle_device_announce(&device, CHILD_A, short, 0x8E)
            .unwrap();
        let durable = runtime.store_mut().load().unwrap().unwrap();
        assert!(!durable.device(&CHILD_A).unwrap().network_key_pending);
        assert_eq!(durable.device(&CHILD_A).unwrap().is_router, Some(true));
    }

    #[test]
    fn remote_initial_key_waits_for_child_evidence_not_tunnel_delivery() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let parent = add_child(&mut device, CHILD_A);
        let child = ShortAddress(0x3344);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent);
        device.mac_mut().clear_tx_history();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_A,
                device_address: CHILD_B,
                device_short_address: child,
                status: ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
            }),
        ))
        .unwrap();
        assert_eq!(device.mac().tx_history().len(), 1);
        let snapshot = runtime.store_mut().load().unwrap().unwrap();
        assert!(snapshot.device(&CHILD_B).unwrap().network_key_pending);

        let mut reboot_store = RamTrustCenterDeviceStore::new();
        reboot_store.store(&snapshot).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        assert_eq!(add_child(&mut rebooted, CHILD_A), parent);
        let mut runtime = TrustCenterRuntime::new(reboot_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert_eq!(
            rebooted.mac().tx_history().len(),
            1,
            "reboot resends the tunnel"
        );
        runtime
            .handle_device_announce(&rebooted, CHILD_B, parent, 0x80)
            .unwrap();
        assert!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_B)
                .unwrap()
                .network_key_pending
        );
        runtime
            .handle_device_announce(&rebooted, CHILD_B, child, 0x80)
            .unwrap();
        assert!(
            !runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_B)
                .unwrap()
                .network_key_pending
        );
    }

    fn complete_pending_submission<S: TrustCenterDeviceStore>(
        runtime: &mut TrustCenterRuntime<S>,
        device: &mut ZigbeeDevice<MockMac>,
    ) {
        assert!(runtime.pending_delivery.is_some());
        block_on(runtime.poll(device)).unwrap();
        assert_security_commands_do_not_request_ack(device);
    }

    fn assert_security_commands_do_not_request_ack(device: &ZigbeeDevice<MockMac>) {
        use zigbee_aps::frames::{ApsFrameType, ApsHeader};
        use zigbee_nwk::frames::{NwkFrameType, NwkHeader};
        use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};

        for record in device.mac().tx_history() {
            let bytes = record.payload.as_slice();
            let (nwk, header_len) = NwkHeader::parse(bytes).unwrap();
            if nwk.frame_control.frame_type != NwkFrameType::Data as u8 {
                continue;
            }
            let plaintext;
            let payload = if nwk.frame_control.security {
                let (security, security_len) =
                    NwkSecurityHeader::parse(&bytes[header_len..]).unwrap();
                let aad_len = header_len + security_len;
                let mut aad = bytes[..aad_len].to_vec();
                aad[header_len] = (aad[header_len] & !0x07) | 0x05;
                let key = device
                    .aps()
                    .nwk()
                    .security()
                    .key_by_seq(security.key_seq_number)
                    .unwrap();
                plaintext = NwkSecurity::new()
                    .decrypt(&aad, &bytes[aad_len..], &key.key, &security)
                    .unwrap();
                plaintext.as_slice()
            } else {
                &bytes[header_len..]
            };
            let (aps, _) = ApsHeader::parse(payload).unwrap();
            if aps.frame_control.frame_type == ApsFrameType::Command as u8 {
                assert!(!aps.frame_control.ack_request, "R22 4.4.10 requires AR=0");
            }
        }
    }

    #[test]
    fn unique_tclk_keeps_the_old_key_until_verify_and_survives_a_runtime_restart() {
        let mut device = coordinator();
        let child_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_short);
        device.mac_mut().clear_tx_history();

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: CHILD_A,
                key_type: ApsRequestKeyType::TrustCenterLink,
                partner_address: None,
            }),
        ))
        .unwrap();
        let pending = runtime
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .device(&CHILD_A)
            .unwrap()
            .pending_link_key
            .unwrap();
        assert!(pending.transported);
        assert_ne!(pending.key, DEFAULT_TC_LINK_KEY);
        assert_eq!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .unwrap()
                .key,
            DEFAULT_TC_LINK_KEY,
            "the old key remains usable until Verify-Key succeeds"
        );

        let persisted = runtime.store_mut().load().unwrap().unwrap();
        let mut reboot_store = RamTrustCenterDeviceStore::new();
        reboot_store.store(&persisted).unwrap();
        device.aps_mut().security_mut().clear_keys();
        device.mac_mut().clear_tx_history();
        let mut runtime = TrustCenterRuntime::new(reboot_store);
        block_on(runtime.initialize(&mut device)).unwrap();
        assert!(device.mac().tx_history().is_empty());
        assert_eq!(
            runtime.table().device(&CHILD_A).unwrap().link_key,
            DEFAULT_TC_LINK_KEY
        );
        assert_eq!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .unwrap()
                .key,
            DEFAULT_TC_LINK_KEY
        );

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: CHILD_A,
                key_type: ApsRequestKeyType::TrustCenterLink,
                partner_address: None,
            }),
        ))
        .unwrap();
        let resent = runtime
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .device(&CHILD_A)
            .unwrap()
            .pending_link_key
            .unwrap();
        assert_eq!(resent.key, pending.key);
        assert!(resent.transported);
        device.mac_mut().clear_tx_history();

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::VerifyKey(ApsmeVerifyKeyIndication {
                source_address: CHILD_A,
                key_type: 0x04,
                hash: derive_verify_key_hash(&pending.key),
            }),
        ))
        .unwrap();
        assert_eq!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending,
            Some(0)
        );
        block_on(runtime.poll(&mut device)).unwrap();
        let committed = runtime.store_mut().load().unwrap().unwrap();
        let committed = committed.device(&CHILD_A).unwrap();
        assert!(committed.pending_link_key.is_none());
        assert_eq!(committed.confirm_key_pending, Some(0));
        assert_eq!(committed.device.link_key, pending.key);
        assert_eq!(
            committed.device.key_origin,
            TrustCenterKeyOrigin::GeneratedUnique
        );
        assert_eq!(
            committed.device.key_attributes,
            TrustCenterKeyAttributes::Verified
        );
        assert_eq!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .unwrap()
                .key,
            pending.key
        );
        complete_pending_submission(&mut runtime, &mut device);
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending
                .is_none()
        );

        device.mac_mut().clear_tx_history();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::VerifyKey(ApsmeVerifyKeyIndication {
                source_address: CHILD_A,
                key_type: 0x04,
                hash: derive_verify_key_hash(&pending.key),
            }),
        ))
        .unwrap();
        block_on(runtime.poll(&mut device)).unwrap();
        assert_eq!(
            device.mac().tx_history().len(),
            1,
            "a repeated Verify-Key restages and resends Confirm-Key"
        );
        assert_eq!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending,
            Some(0)
        );
        complete_pending_submission(&mut runtime, &mut device);
        let repeated = runtime.store_mut().load().unwrap().unwrap();
        let repeated = repeated.device(&CHILD_A).unwrap();
        assert!(repeated.pending_link_key.is_none());
        assert!(repeated.confirm_key_pending.is_none());
    }

    #[test]
    fn confirm_key_send_intent_survives_reboot_and_completes_without_aps_ack() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let child_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_short);
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: CHILD_A,
                key_type: ApsRequestKeyType::TrustCenterLink,
                partner_address: None,
            }),
        ))
        .unwrap();
        let key = runtime
            .state
            .as_ref()
            .unwrap()
            .device(&CHILD_A)
            .unwrap()
            .pending_link_key
            .unwrap()
            .key;
        device.mac_mut().clear_tx_history();

        let verification = ApsmeVerifyKeyIndication {
            source_address: CHILD_A,
            key_type: 0x04,
            hash: derive_verify_key_hash(&key),
        };
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            block_on(runtime.handle_verify_key(&mut device, verification)),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert_eq!(
            runtime.table().device(&CHILD_A).unwrap().link_key,
            DEFAULT_TC_LINK_KEY
        );
        let unchanged = runtime.state.as_ref().unwrap().device(&CHILD_A).unwrap();
        assert_eq!(unchanged.device.link_key, DEFAULT_TC_LINK_KEY);
        assert!(unchanged.pending_link_key.is_some());
        assert!(unchanged.confirm_key_pending.is_none());
        assert!(device.mac().tx_history().is_empty());
        assert_eq!(
            runtime.store_mut().load().unwrap().as_ref(),
            runtime.state.as_ref()
        );

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::VerifyKey(verification),
        ))
        .unwrap();
        assert_eq!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending,
            Some(0)
        );
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            block_on(runtime.resume_confirm_key(&mut device, CHILD_A)),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert!(runtime.pending_delivery.is_some());
        assert_eq!(
            device.mac().tx_history().len(),
            1,
            "Confirm-Key submission precedes the failed counter checkpoint: {:?}, pending {:?}",
            runtime.last_transport_error(),
            runtime.pending_delivery
        );
        let live = device
            .aps_mut()
            .security_mut()
            .find_key_mut(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        live.incoming_frame_counter = 23;
        live.incoming_frame_counter_valid = true;
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            block_on(runtime.resume_confirm_key(&mut device, CHILD_A)),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert!(runtime.pending_delivery.is_some());
        assert_eq!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .unwrap()
                .incoming_frame_counter,
            23,
            "bookkeeping retries must not repeat Confirm-Key's incoming-counter reset"
        );
        assert_eq!(
            device.mac().tx_history().len(),
            1,
            "failed send bookkeeping must not resend"
        );
        assert_security_commands_do_not_request_ack(&device);
        let durable = runtime.store_mut().load().unwrap().unwrap();
        assert_eq!(
            durable.device(&CHILD_A).unwrap().confirm_key_pending,
            Some(0)
        );

        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&durable).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        assert_eq!(add_child(&mut rebooted, CHILD_A), child_short);
        rebooted.mac_mut().clear_tx_history();
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert_eq!(
            rebooted.mac().tx_history().len(),
            1,
            "restart resends the durable Confirm-Key obligation: {:?}",
            runtime.last_transport_error()
        );
        assert_eq!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending,
            Some(0)
        );

        complete_pending_submission(&mut runtime, &mut rebooted);
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .confirm_key_pending
                .is_none()
        );
    }

    #[test]
    fn remote_remove_device_revokes_after_local_submission_and_replay_tombstone() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let parent_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent_short);
        let child_short = ShortAddress(0x3344);
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_A,
                device_address: CHILD_B,
                device_short_address: child_short,
                status: ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
            }),
        ))
        .unwrap();
        device.mac_mut().clear_tx_history();

        runtime
            .stage_removal(&mut device, CHILD_B, CHILD_A, child_short)
            .unwrap();
        block_on(runtime.resume_removal(&mut device, CHILD_B)).unwrap();
        assert_eq!(device.mac().tx_history().len(), 1);
        let durable = runtime.store_mut().load().unwrap().unwrap();
        assert!(durable.device(&CHILD_B).unwrap().removal_pending);

        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&durable).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        assert_eq!(add_child(&mut rebooted, CHILD_A), parent_short);
        rebooted.mac_mut().clear_tx_history();
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert_eq!(
            rebooted.mac().tx_history().len(),
            1,
            "restart resends the uncommitted Remove-Device send intent: {:?}",
            runtime.last_transport_error()
        );
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_B)
                .unwrap()
                .removal_pending
        );

        complete_pending_submission(&mut runtime, &mut rebooted);
        let address = runtime.pending_device_replay_tombstone().unwrap();
        security_store
            .tombstone_replay_counters(crate::security_store::ReplayCounterTombstone::Device(
                address,
            ))
            .unwrap();
        runtime
            .complete_pending_device_revocation(&mut rebooted, address)
            .unwrap();
        let completed = runtime.store_mut().load().unwrap().unwrap();
        assert!(completed.device(&CHILD_B).is_none());
        assert!(completed.device(&CHILD_A).is_some());
    }

    #[test]
    fn application_key_broker_delivers_one_key_without_installing_it_at_the_tc() {
        let mut device = coordinator();
        let child_a_short = add_child(&mut device, CHILD_A);
        let child_b_short = add_child(&mut device, CHILD_B);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_a_short);
        join(&mut runtime, &mut device, CHILD_B, child_b_short);
        device.mac_mut().clear_tx_history();

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: CHILD_A,
                key_type: ApsRequestKeyType::ApplicationLink,
                partner_address: Some(CHILD_B),
            }),
        ))
        .unwrap();

        assert!(runtime.has_pending_transactions());
        assert_eq!(device.mac().tx_history().len(), 1);
        complete_pending_submission(&mut runtime, &mut device);
        assert_eq!(device.mac().tx_history().len(), 2);
        complete_pending_submission(&mut runtime, &mut device);
        assert!(!runtime.has_pending_transactions());
        assert!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::ApplicationLinkKey)
                .is_none()
        );
        assert!(
            device
                .aps()
                .security()
                .find_key(&CHILD_B, ApsKeyType::ApplicationLinkKey)
                .is_none()
        );
    }

    #[test]
    fn application_key_submission_failure_preserves_the_generation_and_progress() {
        let mut device = coordinator();
        let child_a_short = add_child(&mut device, CHILD_A);
        let child_b_short = add_child(&mut device, CHILD_B);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_a_short);
        join(&mut runtime, &mut device, CHILD_B, child_b_short);
        device.mac_mut().clear_tx_history();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: CHILD_A,
                key_type: ApsRequestKeyType::ApplicationLink,
                partner_address: Some(CHILD_B),
            }),
        ))
        .unwrap();
        let generation = runtime
            .state
            .as_ref()
            .unwrap()
            .pending_application_key()
            .unwrap()
            .key;

        for submitted in 1..=2 {
            assert_eq!(device.mac().tx_history().len(), submitted);
            runtime.store_mut().fail_next_store = true;
            assert_eq!(
                block_on(runtime.resume_application_link_key(&mut device)),
                Err(TrustCenterRuntimeError::Store(
                    TrustCenterStoreError::Hardware
                ))
            );
            assert_eq!(device.mac().tx_history().len(), submitted);
            let durable = runtime.store_mut().load().unwrap().unwrap();
            assert_eq!(runtime.state.as_ref(), Some(&durable));
            assert_eq!(durable.pending_application_key().unwrap().key, generation);
            assert!(runtime.pending_delivery.is_some());
            block_on(runtime.resume_application_link_key(&mut device)).unwrap();
        }
        assert!(
            runtime
                .state
                .as_ref()
                .unwrap()
                .pending_application_key()
                .is_none()
        );
        assert_eq!(device.mac().tx_history().len(), 2);
        assert_security_commands_do_not_request_ack(&device);
    }

    #[test]
    fn application_key_allowlist_is_directional_and_durable() {
        let mut device = coordinator();
        let child_a_short = add_child(&mut device, CHILD_A);
        let child_b_short = add_child(&mut device, CHILD_B);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_a_short);
        join(&mut runtime, &mut device, CHILD_B, child_b_short);
        device
            .bdb_mut()
            .attributes_mut()
            .trust_center_application_key_request_policy =
            zigbee_bdb::attributes::ApplicationLinkKeyRequestPolicy::AllowListOnly;
        device.mac_mut().clear_tx_history();

        let request = ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
            source_address: CHILD_A,
            key_type: ApsRequestKeyType::ApplicationLink,
            partner_address: Some(CHILD_B),
        });
        block_on(runtime.handle_indication(&mut device, request)).unwrap();
        assert!(device.mac().tx_history().is_empty());

        runtime
            .allow_application_key_request(&device, CHILD_A, CHILD_B)
            .unwrap();
        block_on(runtime.handle_indication(&mut device, request)).unwrap();
        assert_eq!(device.mac().tx_history().len(), 1);
        complete_pending_submission(&mut runtime, &mut device);
        assert_eq!(device.mac().tx_history().len(), 2);
        complete_pending_submission(&mut runtime, &mut device);
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .application_key_request_allowed(&CHILD_A, &CHILD_B)
        );

        device.mac_mut().clear_tx_history();
        let reverse = ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
            source_address: CHILD_B,
            key_type: ApsRequestKeyType::ApplicationLink,
            partner_address: Some(CHILD_A),
        });
        block_on(runtime.handle_indication(&mut device, reverse)).unwrap();
        assert!(device.mac().tx_history().is_empty());
    }

    #[test]
    fn idle_poll_does_not_rewrite_the_trust_center_journal() {
        let mut device = coordinator();
        let mut runtime = TrustCenterRuntime::new(CountingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        let stores_after_initialize = runtime.store().stores;

        block_on(device.mac_mut().delay_micros(1_000_000));
        block_on(runtime.poll(&mut device)).unwrap();

        assert_eq!(runtime.store().stores, stores_after_initialize);
    }

    #[test]
    fn poll_clock_handles_u32_monotonic_wrap() {
        let mut device = coordinator();
        block_on(
            device
                .mac_mut()
                .delay_micros(u32::MAX.saturating_sub(500_000)),
        );
        let mut runtime = TrustCenterRuntime::new(CountingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();

        block_on(device.mac_mut().delay_micros(1_000_000));
        block_on(runtime.poll(&mut device)).unwrap();

        assert_eq!(runtime.residual_us, 0);
        assert_eq!(
            runtime.last_poll_us,
            u32::MAX.saturating_sub(500_000).wrapping_add(1_000_000)
        );
    }

    #[test]
    fn broadcast_network_key_rotation_switches_only_after_distribution_is_durable() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let old_sequence = device
            .aps()
            .nwk()
            .security()
            .active_key()
            .unwrap()
            .seq_number;
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let target = begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Broadcast,
        );
        assert_eq!(
            runtime.prepare_network_key_rotation(&mut device),
            Err(TrustCenterRuntimeError::NetworkKeyRotationInProgress)
        );

        device.mac_mut().clear_tx_history();
        block_on(runtime.poll(&mut device)).unwrap();
        assert_eq!(device.mac().tx_history().len(), 2);
        let first = device.mac().tx_history()[0].payload.as_slice();
        let second = device.mac().tx_history()[1].payload.as_slice();
        assert_eq!(u16::from_le_bytes([first[2], first[3]]), 0xFFFF);
        assert_eq!(u16::from_le_bytes([second[2], second[3]]), 0xFFFD);
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::Activating
        );
        assert_eq!(
            device
                .aps()
                .nwk()
                .security()
                .active_key()
                .unwrap()
                .seq_number,
            old_sequence,
            "the coordinator remains on the old key until Switch-Key is sent"
        );

        assert!(runtime.activate_pending_network_key(&mut device).unwrap());
        assert!(device.refresh_security_state(&mut security_store).unwrap());
        runtime
            .complete_network_key_rotation(&mut device, &mut security_store)
            .unwrap();
        assert!(runtime.network_key_rotation().is_none());
        assert_eq!(device.aps().nwk().nib().active_key_seq_number, target);
        let persisted = security_store.load().unwrap().unwrap();
        assert_eq!(persisted.key_sequence, target);
        assert!(persisted.staged_network_key_present);
        assert_eq!(persisted.staged_key_sequence, old_sequence);
        assert!(persisted.secondary_network_key_is_previous);
    }

    #[test]
    fn populated_network_restarts_propagation_wait_after_reboot() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        device
            .bdb_mut()
            .attributes_mut()
            .trust_center_require_key_exchange = false;
        let short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, short);
        begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Broadcast,
        );
        device.mac_mut().clear_tx_history();
        block_on(runtime.poll(&mut device)).unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::WaitingForPropagation
        );
        assert_eq!(
            device.mac().tx_history().len(),
            1,
            "no immediate Switch-Key"
        );
        block_on(
            device
                .mac_mut()
                .delay_micros(NETWORK_KEY_PROPAGATION_DELAY_US - 1),
        );
        block_on(runtime.poll(&mut device)).unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::WaitingForPropagation
        );

        let mut reboot_store = RamTrustCenterDeviceStore::new();
        reboot_store
            .store(&runtime.store_mut().load().unwrap().unwrap())
            .unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        let mut runtime = TrustCenterRuntime::new(reboot_store);
        block_on(runtime.initialize_with_security_store(&mut rebooted, &mut security_store))
            .unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::WaitingForPropagation
        );
        assert!(rebooted.mac().tx_history().is_empty());
        block_on(
            rebooted
                .mac_mut()
                .delay_micros(NETWORK_KEY_PROPAGATION_DELAY_US),
        );
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::Activating
        );
        assert_eq!(rebooted.mac().tx_history().len(), 1);
    }

    #[test]
    fn unicast_rotation_requires_known_roles_and_skips_end_devices() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        device
            .bdb_mut()
            .attributes_mut()
            .trust_center_require_key_exchange = false;
        let short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        begin_join(&mut runtime, &mut device, CHILD_A, short);
        let target = runtime.prepare_network_key_rotation(&mut device).unwrap();
        device.refresh_security_state(&mut security_store).unwrap();
        assert_eq!(
            runtime.commit_network_key_rotation(&device, target, NetworkKeyUpdateMethod::Unicast),
            Err(TrustCenterRuntimeError::RouterListIncomplete)
        );
        assert!(runtime.network_key_rotation().is_none());
        runtime
            .handle_device_announce(&device, CHILD_A, short, 0x80)
            .unwrap();
        runtime
            .commit_network_key_rotation(&device, target, NetworkKeyUpdateMethod::Unicast)
            .unwrap();
        device.mac_mut().clear_tx_history();
        block_on(runtime.poll(&mut device)).unwrap();
        assert!(
            device.mac().tx_history().is_empty(),
            "R22 unicast policy targets routers only"
        );
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::WaitingForPropagation
        );
    }

    #[test]
    fn switching_phase_reboots_before_local_activation_and_resends_switch_key() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let old_sequence = device
            .aps()
            .nwk()
            .security()
            .active_key()
            .unwrap()
            .seq_number;
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let target = begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Broadcast,
        );
        let mut switching = runtime.store_mut().load().unwrap().unwrap();
        switching.set_rotation(Some(NetworkKeyRotation {
            target_sequence: target,
            phase: NetworkKeyRotationPhase::Switching,
            method: NetworkKeyUpdateMethod::Broadcast,
        }));
        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&switching).unwrap();

        let mut rebooted = restored_coordinator(&mut security_store);
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize_with_security_store(&mut rebooted, &mut security_store))
            .unwrap();
        assert_eq!(rebooted.mac().tx_history().len(), 1);
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::Activating
        );
        assert_eq!(
            rebooted
                .aps()
                .nwk()
                .security()
                .active_key()
                .unwrap()
                .seq_number,
            old_sequence
        );
    }

    #[test]
    fn unicast_rotation_resumes_per_device_progress_after_transport_failure_and_reboot() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        device
            .bdb_mut()
            .attributes_mut()
            .trust_center_require_key_exchange = false;
        let child_a_short = add_child(&mut device, CHILD_A);
        let child_b_short = add_child(&mut device, CHILD_B);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_a_short);
        join(&mut runtime, &mut device, CHILD_B, child_b_short);
        runtime
            .handle_device_announce(&device, CHILD_A, child_a_short, 0x8E)
            .unwrap();
        runtime
            .handle_device_announce(&device, CHILD_B, child_b_short, 0x8E)
            .unwrap();
        let target = begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Unicast,
        );

        device.mac_mut().clear_tx_history();
        device.mac_mut().set_tx_failures(1);
        block_on(runtime.poll(&mut device)).unwrap();
        assert!(matches!(
            runtime.last_transport_error(),
            Some(TrustCenterRuntimeError::Aps(_))
        ));
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .devices()
                .iter()
                .all(|stored| !stored.new_network_key_delivered)
        );

        block_on(runtime.poll(&mut device)).unwrap();
        let after_first = runtime.store_mut().load().unwrap().unwrap();
        assert!(
            runtime.pending_delivery.is_none(),
            "R22 Transport-Key does not request APS ACK"
        );
        assert_eq!(
            after_first
                .devices()
                .iter()
                .filter(|stored| stored.new_network_key_delivered)
                .count(),
            1
        );

        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&after_first).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        assert_eq!(add_child(&mut rebooted, CHILD_A), child_a_short);
        assert_eq!(add_child(&mut rebooted, CHILD_B), child_b_short);
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert_eq!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .devices()
                .iter()
                .filter(|stored| stored.new_network_key_delivered)
                .count(),
            2
        );
        let after_second = runtime.store_mut().load().unwrap().unwrap();
        assert_eq!(
            after_second
                .devices()
                .iter()
                .filter(|stored| stored.new_network_key_delivered)
                .count(),
            2
        );

        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::WaitingForPropagation
        );
        block_on(
            rebooted
                .mac_mut()
                .delay_micros(NETWORK_KEY_PROPAGATION_DELAY_US),
        );
        block_on(runtime.poll(&mut rebooted)).unwrap();
        assert_eq!(
            runtime.network_key_rotation().unwrap().phase,
            NetworkKeyRotationPhase::Activating
        );
        runtime.activate_pending_network_key(&mut rebooted).unwrap();
        rebooted
            .refresh_security_state(&mut security_store)
            .unwrap();
        runtime
            .complete_network_key_rotation(&mut rebooted, &mut security_store)
            .unwrap();
        assert_eq!(
            rebooted
                .aps()
                .nwk()
                .security()
                .active_key()
                .unwrap()
                .seq_number,
            target
        );
    }

    #[test]
    fn activating_phase_survives_reboot_before_and_after_the_security_checkpoint() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let target = begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Broadcast,
        );
        block_on(runtime.poll(&mut device)).unwrap();
        let activating = runtime.store_mut().load().unwrap().unwrap();

        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&activating).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert!(runtime.activate_pending_network_key(&mut rebooted).unwrap());
        assert!(
            rebooted
                .refresh_security_state(&mut security_store)
                .unwrap()
        );

        let mut trust_center_store = RamTrustCenterDeviceStore::new();
        trust_center_store.store(&activating).unwrap();
        let mut checkpointed_reboot = restored_coordinator(&mut security_store);
        let mut runtime = TrustCenterRuntime::new(trust_center_store);
        block_on(runtime.initialize(&mut checkpointed_reboot)).unwrap();
        assert!(
            !runtime
                .activate_pending_network_key(&mut checkpointed_reboot)
                .unwrap()
        );
        runtime
            .complete_network_key_rotation(&mut checkpointed_reboot, &mut security_store)
            .unwrap();
        assert_eq!(
            checkpointed_reboot
                .aps()
                .nwk()
                .security()
                .active_key()
                .unwrap()
                .seq_number,
            target
        );
    }

    #[test]
    fn rotation_cooldown_uses_completion_time_across_clock_wrap() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let advance = u32::MAX - device.mac().monotonic_micros() - 4_000_000;
        block_on(device.mac_mut().delay_micros(advance));
        begin_rotation(
            &mut runtime,
            &mut device,
            &mut security_store,
            NetworkKeyUpdateMethod::Broadcast,
        );
        block_on(runtime.poll(&mut device)).unwrap();
        runtime.activate_pending_network_key(&mut device).unwrap();
        device.refresh_security_state(&mut security_store).unwrap();
        runtime
            .complete_network_key_rotation(&mut device, &mut security_store)
            .unwrap();
        runtime.residual_us = 999_999;

        let cooldown_us = u32::from(device.aps().nwk().nib().broadcast_delivery_time) * 1_000_000;
        block_on(device.mac_mut().delay_micros(cooldown_us - 1));
        assert_eq!(
            runtime.prepare_network_key_rotation(&mut device),
            Err(TrustCenterRuntimeError::NetworkKeyRotationCooldown)
        );
        block_on(device.mac_mut().delay_micros(1));
        assert!(runtime.prepare_network_key_rotation(&mut device).is_ok());
    }

    #[test]
    fn rotation_sequence_wraps_from_255_to_zero() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        {
            let nwk = device.aps_mut().nwk_mut();
            nwk.security_mut().set_network_key([0xA5; 16], 0xFF);
            nwk.nib_mut().active_key_seq_number = 0xFF;
        }
        assert!(device.refresh_security_state(&mut security_store).unwrap());
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();

        let cooldown_us = u32::from(device.aps().nwk().nib().broadcast_delivery_time) * 1_000_000;
        block_on(device.mac_mut().delay_micros(cooldown_us));
        assert_eq!(
            runtime.prepare_network_key_rotation(&mut device).unwrap(),
            0
        );
    }

    #[test]
    fn failed_intent_commit_leaves_a_recoverable_orphaned_staged_key() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        let target = runtime.prepare_network_key_rotation(&mut device).unwrap();
        assert!(device.refresh_security_state(&mut security_store).unwrap());
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            runtime.commit_network_key_rotation(&device, target, NetworkKeyUpdateMethod::Broadcast),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        let durable = runtime.store_mut().inner.load().unwrap().unwrap();
        assert!(durable.rotation().is_none());

        let mut store = RamTrustCenterDeviceStore::new();
        store.store(&durable).unwrap();
        let mut rebooted = restored_coordinator(&mut security_store);
        let mut runtime = TrustCenterRuntime::new(store);
        block_on(runtime.initialize(&mut rebooted)).unwrap();
        assert!(runtime.has_orphaned_network_key_preparation(&rebooted));
        assert_eq!(
            runtime.prepare_network_key_rotation(&mut rebooted).unwrap(),
            target
        );
    }

    #[test]
    fn configured_period_marks_rotation_due_without_rewriting_the_store() {
        let mut device = coordinator();
        device
            .bdb_mut()
            .attributes_mut()
            .trust_center_network_key_update_period = 1;
        let mut runtime = TrustCenterRuntime::new(CountingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        let stores = runtime.store().stores;
        block_on(device.mac_mut().delay_micros(60_000_000));
        block_on(runtime.poll(&mut device)).unwrap();

        assert!(runtime.network_key_rotation_due(&device));
        assert_eq!(runtime.store().stores, stores);
    }

    #[test]
    fn authenticated_policy_denials_do_not_stop_the_trust_center() {
        let mut device = coordinator();
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let unknown = [0x99; 8];

        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RequestKey(ApsmeRequestKeyIndication {
                source_address: unknown,
                key_type: ApsRequestKeyType::TrustCenterLink,
                partner_address: None,
            }),
        ))
        .unwrap();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::VerifyKey(ApsmeVerifyKeyIndication {
                source_address: unknown,
                key_type: 0x04,
                hash: [0; 16],
            }),
        ))
        .unwrap();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::RemoveDevice(ApsmeRemoveDeviceIndication {
                source_address: unknown,
                child_address: CHILD_A,
            }),
        ))
        .unwrap();

        assert!(!runtime.has_pending_transactions());
    }

    #[test]
    fn restore_barrier_blocks_pending_delivery_and_indications() {
        let mut device = coordinator();
        let child_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        begin_join(&mut runtime, &mut device, CHILD_A, child_short);
        let mut snapshot = runtime.store_mut().load().unwrap().unwrap();
        snapshot.device_mut(&CHILD_A).unwrap().network_key_pending = true;
        runtime.store_mut().store(&snapshot).unwrap();
        device.mac_mut().clear_tx_history();

        runtime.restore_before_replay(&mut device).unwrap();
        assert!(!runtime.is_initialized());
        assert_eq!(
            block_on(runtime.poll(&mut device)),
            Err(TrustCenterRuntimeError::NotJoined)
        );
        assert_eq!(
            block_on(runtime.handle_indication(
                &mut device,
                ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                    source_address: TC,
                    device_address: CHILD_A,
                    device_short_address: child_short,
                    status: ApsUpdateDeviceStatus::DeviceLeft,
                }),
            )),
            Err(TrustCenterRuntimeError::NotJoined)
        );
        assert!(device.mac().tx_history().is_empty());
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD_A)
                .unwrap()
                .network_key_pending
        );

        device
            .restore_incoming_replay_state(&mut RamSecurityStateStore::new())
            .unwrap();
        block_on(runtime.resume_after_replay_restore(&mut device)).unwrap();
        assert!(runtime.is_initialized());
        assert!(!device.mac().tx_history().is_empty());
    }

    #[test]
    fn failed_restore_of_a_live_runtime_closes_the_barrier() {
        let mut device = coordinator();
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        runtime.store_mut().fail_next_store = true;
        device.mac_mut().clear_tx_history();
        assert_eq!(
            runtime.restore_before_replay(&mut device),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert!(!runtime.is_initialized());
        assert_eq!(
            block_on(runtime.poll(&mut device)),
            Err(TrustCenterRuntimeError::NotJoined)
        );
        assert_eq!(
            block_on(runtime.resume_after_replay_restore(&mut device)),
            Err(TrustCenterRuntimeError::NotJoined)
        );
        assert!(device.mac().tx_history().is_empty());
        runtime.restore_before_replay(&mut device).unwrap();
        block_on(runtime.resume_after_replay_restore(&mut device)).unwrap();
        assert!(runtime.is_initialized());
    }

    #[test]
    fn duplicate_device_left_for_unknown_device_is_idempotent() {
        let mut device = coordinator();
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        let unknown = [0x99; 8];

        for _ in 0..2 {
            block_on(runtime.handle_indication(
                &mut device,
                ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                    source_address: TC,
                    device_address: unknown,
                    device_short_address: ShortAddress(0x4455),
                    status: ApsUpdateDeviceStatus::DeviceLeft,
                }),
            ))
            .unwrap();
            assert_eq!(runtime.pending_device_replay_tombstone(), Some(unknown));
            runtime
                .complete_pending_device_revocation(&mut device, unknown)
                .unwrap();
        }

        assert!(!runtime.has_pending_transactions());
    }

    #[test]
    fn rejected_unknown_secured_rejoin_is_removed_without_a_device_record() {
        let mut device = coordinator();
        let parent_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent_short);
        device.mac_mut().clear_tx_history();

        let unknown = [0x99; 8];
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_A,
                device_address: unknown,
                device_short_address: ShortAddress(0x4455),
                status: ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
            }),
        ))
        .unwrap();

        assert!(runtime.table().device(&unknown).is_none());
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&unknown)
                .is_none()
        );
        complete_pending_submission(&mut runtime, &mut device);
        assert_eq!(runtime.pending_device_replay_tombstone(), Some(unknown));
        runtime
            .complete_pending_device_revocation(&mut device, unknown)
            .unwrap();
        assert!(!runtime.has_pending_transactions());
    }

    #[test]
    fn delayed_device_left_does_not_revoke_a_new_parent_or_address() {
        let mut device = coordinator();
        device.bdb_mut().attributes_mut().trust_center_allow_rejoins = true;
        let old_short = add_child(&mut device, CHILD_A);
        let parent_short = add_child(&mut device, CHILD_B);
        let mut runtime = TrustCenterRuntime::new(CountingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, old_short);
        join(&mut runtime, &mut device, CHILD_B, parent_short);
        device.aps_mut().nwk_mut().remove_neighbor(old_short);
        let new_short = ShortAddress(0x4455);
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_B,
                device_address: CHILD_A,
                device_short_address: new_short,
                status: ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
            }),
        ))
        .unwrap();
        assert_eq!(
            runtime.table().device(&CHILD_A).unwrap().parent_address,
            CHILD_B
        );
        let stores = runtime.store().stores;
        for (parent, short) in [(TC, old_short), (CHILD_B, old_short)] {
            block_on(runtime.handle_indication(
                &mut device,
                ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                    source_address: parent,
                    device_address: CHILD_A,
                    device_short_address: short,
                    status: ApsUpdateDeviceStatus::DeviceLeft,
                }),
            ))
            .unwrap();
            assert_eq!(runtime.pending_device_replay_tombstone(), None);
            assert_eq!(
                runtime.table().device(&CHILD_A).unwrap().short_address,
                new_short
            );
        }
        assert_eq!(runtime.store().stores, stores);
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_B,
                device_address: CHILD_A,
                device_short_address: new_short,
                status: ApsUpdateDeviceStatus::DeviceLeft,
            }),
        ))
        .unwrap();
        assert_eq!(runtime.pending_device_replay_tombstone(), Some(CHILD_A));
    }

    #[test]
    fn device_left_after_completed_revocation_does_not_rewrite_the_tc_store() {
        let mut device = coordinator();
        let child_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(CountingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_short);
        let left = ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
            source_address: TC,
            device_address: CHILD_A,
            device_short_address: child_short,
            status: ApsUpdateDeviceStatus::DeviceLeft,
        });
        block_on(runtime.handle_indication(&mut device, left)).unwrap();
        runtime
            .complete_pending_device_revocation(&mut device, CHILD_A)
            .unwrap();
        let stores = runtime.store().stores;
        for _ in 0..2 {
            block_on(runtime.handle_indication(&mut device, left)).unwrap();
            runtime
                .complete_pending_device_revocation(&mut device, CHILD_A)
                .unwrap();
        }
        assert_eq!(runtime.store().stores, stores);
        assert!(runtime.table().device(&CHILD_A).is_none());
        assert!(
            device
                .aps()
                .security()
                .find_key(&CHILD_A, ApsKeyType::TrustCenterLinkKey)
                .is_none()
        );
    }

    #[test]
    fn rejected_unknown_rejoin_propagates_store_failure_before_transmit() {
        let mut device = coordinator();
        let parent_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(FailingStore::default());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent_short);
        let unknown = [0x99; 8];
        let indication = ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
            source_address: CHILD_A,
            device_address: unknown,
            device_short_address: ShortAddress(0x4455),
            status: ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
        });
        runtime.store_mut().fail_next_store = true;
        device.mac_mut().clear_tx_history();
        assert_eq!(
            block_on(runtime.handle_indication(&mut device, indication)),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        block_on(runtime.poll(&mut device)).unwrap();
        assert!(device.mac().tx_history().is_empty());
        assert!(!runtime.has_pending_transactions());
        assert!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .pending_unknown_removals()
                .is_empty()
        );

        block_on(runtime.handle_indication(&mut device, indication)).unwrap();
        assert!(!device.mac().tx_history().is_empty());
        complete_pending_submission(&mut runtime, &mut device);
        runtime.store_mut().fail_next_store = true;
        assert_eq!(
            runtime.complete_pending_device_revocation(&mut device, unknown),
            Err(TrustCenterRuntimeError::Store(
                TrustCenterStoreError::Hardware
            ))
        );
        assert_eq!(runtime.pending_device_replay_tombstone(), Some(unknown));
        assert_eq!(
            runtime
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .pending_unknown_removals()
                .len(),
            1
        );
        // Retrying completion must retry the failed write, not silently clear
        // the in-memory transaction and claim success.
        runtime
            .complete_pending_device_revocation(&mut device, unknown)
            .unwrap();
        assert!(!runtime.has_pending_transactions());
    }

    #[test]
    fn unknown_rejection_finishes_if_its_parent_is_revoked_before_send_commit() {
        let mut device = coordinator();
        let parent_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent_short);
        let unknown = [0x99; 8];
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: CHILD_A,
                device_address: unknown,
                device_short_address: ShortAddress(0x4455),
                status: ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
            }),
        ))
        .unwrap();
        assert!(runtime.pending_delivery.is_some());
        device.mac_mut().clear_tx_history();
        block_on(runtime.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                source_address: TC,
                device_address: CHILD_A,
                device_short_address: parent_short,
                status: ApsUpdateDeviceStatus::DeviceLeft,
            }),
        ))
        .unwrap();
        runtime
            .complete_pending_device_revocation(&mut device, CHILD_A)
            .unwrap();
        assert!(
            runtime.pending_delivery.is_none(),
            "do not retransmit to a revoked parent"
        );
        block_on(runtime.poll(&mut device)).unwrap();
        assert_eq!(runtime.pending_device_replay_tombstone(), Some(unknown));
        runtime
            .complete_pending_device_revocation(&mut device, unknown)
            .unwrap();
        assert!(!runtime.has_pending_transactions());
        assert!(device.mac().tx_history().is_empty());
    }

    #[test]
    fn unregistered_forwarder_cannot_admit_or_remove_devices() {
        let mut device = coordinator();
        let child_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, child_short);
        let before = runtime.store_mut().load().unwrap();
        for status in [
            ApsUpdateDeviceStatus::DeviceLeft,
            ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
        ] {
            block_on(runtime.handle_indication(
                &mut device,
                ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                    source_address: [0x99; 8],
                    device_address: CHILD_A,
                    device_short_address: child_short,
                    status,
                }),
            ))
            .unwrap();
        }
        assert_eq!(runtime.store_mut().load().unwrap(), before);
        assert!(runtime.pending_device_replay_tombstone().is_none());
        assert!(!runtime.has_pending_transactions());
    }

    #[test]
    fn rejected_unknown_rejoin_survives_reboot_without_becoming_admitted() {
        let (mut device, mut security_store) = coordinator_with_security_store();
        let parent_short = add_child(&mut device, CHILD_A);
        let mut runtime = TrustCenterRuntime::new(RamTrustCenterDeviceStore::new());
        block_on(runtime.initialize(&mut device)).unwrap();
        join(&mut runtime, &mut device, CHILD_A, parent_short);
        let unknown = [0x99; 8];
        let update = ApsmeUpdateDeviceIndication {
            source_address: CHILD_A,
            device_address: unknown,
            device_short_address: ShortAddress(0x4455),
            status: ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
        };
        block_on(
            runtime.handle_indication(&mut device, ApsmeSecurityIndication::UpdateDevice(update)),
        )
        .unwrap();
        assert!(runtime.pending_delivery.is_some());
        let store = core::mem::take(runtime.store_mut());
        drop(runtime);
        drop(device); // Lose the volatile local-submission state.

        let mut device = restored_coordinator(&mut security_store);
        assert!(device.aps_mut().nwk_mut().restore_child(
            CHILD_A,
            parent_short,
            true,
            true,
            true,
            8
        ));
        let mut rebooted = TrustCenterRuntime::new(store);
        rebooted.restore_before_replay(&mut device).unwrap();
        device
            .restore_incoming_replay_state(&mut security_store)
            .unwrap();
        assert!(device.mac().tx_history().is_empty());
        block_on(rebooted.resume_after_replay_restore(&mut device)).unwrap();
        assert!(
            rebooted.pending_delivery.is_some(),
            "removal is resent after the restore barrier"
        );
        assert!(rebooted.table().device(&unknown).is_none());
        assert!(
            rebooted
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&unknown)
                .is_none()
        );

        // Even an otherwise-permitted new join cannot overtake removal at the
        // old parent and create an admitted entry while that command is live.
        block_on(rebooted.handle_indication(
            &mut device,
            ApsmeSecurityIndication::UpdateDevice(ApsmeUpdateDeviceIndication {
                status: ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin,
                ..update
            }),
        ))
        .unwrap();
        assert!(rebooted.table().device(&unknown).is_none());
        complete_pending_submission(&mut rebooted, &mut device);
        rebooted
            .complete_pending_device_revocation(&mut device, unknown)
            .unwrap();
        assert!(
            rebooted
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .pending_unknown_removals()
                .is_empty()
        );
    }
}
