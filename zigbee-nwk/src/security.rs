//! NWK layer security — AES-128-CCM* encryption/decryption.
//!
//! Zigbee PRO uses NWK-level security for ALL routed frames:
//! - Security Level 5: ENC-MIC-32 (AES-128-CCM* with 4-byte MIC)
//! - Network key shared by all devices in the network
//! - Frame counter for replay protection
//!
//! MAC-level security is NOT used for normal Zigbee 3.0 data frames.

pub use zigbee_crypto::AesKey;
use zigbee_crypto::{
    ForwardAesProvider, SoftwareAesProvider, ccm_star_decrypt_with, ccm_star_encrypt_with,
};
use zigbee_types::IeeeAddress;

/// Maximum number of network keys we can store (current + previous)
pub const MAX_NETWORK_KEYS: usize = 2;
#[cfg(feature = "router")]
pub const MAX_FRAME_COUNTER_ENTRIES: usize = 64;
#[cfg(not(feature = "router"))]
pub const MAX_FRAME_COUNTER_ENTRIES: usize = 16;

/// Default silence, in seconds, after which a router may evict a replay
/// floor that is neither its parent's nor a child's (one hour).
#[cfg(feature = "router")]
pub const DEFAULT_REPLAY_STALE_SECS: u16 = 3_600;

/// Outcome of checking an incoming NWK frame counter before MIC verification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayAdmission {
    /// The counter is newer than the recorded floor, or a new source fits.
    Fresh,
    /// The counter is not strictly greater than the recorded floor.
    Replay,
    /// A new source cannot be recorded without forgetting a protected or
    /// recently heard floor, so it is refused (fail closed).
    TableFull,
}

/// Serialized length of the NWK security auxiliary header.
///
/// security control (1) + frame counter (4) + source IEEE (8) + key sequence (1).
pub const NWK_AUX_HEADER_LEN: usize = 14;

/// NWK security material for one key
#[derive(Debug, Clone)]
pub struct NetworkKeyEntry {
    /// The 128-bit network key
    pub key: AesKey,
    /// Key sequence number (0-255)
    pub seq_number: u8,
    /// Whether this key is active
    pub active: bool,
}

/// Durable identity and floor of one NWK incoming replay domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NwkReplayCounter {
    pub source: IeeeAddress,
    pub key_sequence: u8,
    pub key_fingerprint: u32,
    pub counter: u32,
}

impl NwkReplayCounter {
    /// Capture the durable identity and floor of a successfully verified
    /// incoming NWK frame.
    pub fn from_verified(
        source: IeeeAddress,
        key_sequence: u8,
        key: &AesKey,
        counter: u32,
    ) -> Self {
        Self {
            source,
            key_sequence,
            key_fingerprint: zigbee_crypto::key_fingerprint(key),
            counter,
        }
    }
}

/// NWK security auxiliary header (prepended to encrypted NWK payload)
#[derive(Debug, Clone)]
pub struct NwkSecurityHeader {
    /// Security control field
    pub security_control: u8,
    /// Frame counter (32-bit, for replay protection)
    pub frame_counter: u32,
    /// Source IEEE address (64-bit — identifies key origin)
    pub source_address: IeeeAddress,
    /// Key sequence number
    pub key_seq_number: u8,
}

impl NwkSecurityHeader {
    /// Security control field value for standard Zigbee:
    /// Security Level = 5 (ENC-MIC-32), Key Identifier = 1 (Network Key),
    /// Extended Nonce = 1 (source address present in aux header)
    /// Per Zigbee PRO R22 §4.5.1.1: extended nonce SHALL be set to 1.
    pub const ZIGBEE_DEFAULT: u8 = 0x05 | (0x01 << 3) | (1 << 5); // 0x2D

    /// `true` when the over-the-air security control names the network key
    /// (key identifier 1) with the extended nonce, i.e. the only NWK-layer
    /// form R22 §4.3.1.2 / §4.5.1.1 allows. The security level bits are
    /// ignored because they are zeroed on the air (§4.3.1.1 step 6).
    pub fn uses_network_key_with_extended_nonce(&self) -> bool {
        self.security_control & !0x07 == Self::ZIGBEE_DEFAULT & !0x07
    }

    pub fn parse(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < NWK_AUX_HEADER_LEN {
            return None;
        }
        let security_control = data[0];
        let frame_counter = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
        let mut source_address = [0u8; 8];
        source_address.copy_from_slice(&data[5..13]);
        let key_seq_number = data[13];

        Some((
            Self {
                security_control,
                frame_counter,
                source_address,
                key_seq_number,
            },
            NWK_AUX_HEADER_LEN,
        ))
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.security_control;
        buf[1..5].copy_from_slice(&self.frame_counter.to_le_bytes());
        buf[5..13].copy_from_slice(&self.source_address);
        buf[13] = self.key_seq_number;
        NWK_AUX_HEADER_LEN
    }
}

/// NWK security context — manages keys, frame counters, encryption.
pub struct NwkSecurity {
    /// Stored network keys
    keys: [Option<NetworkKeyEntry>; MAX_NETWORK_KEYS],
    /// Sequence number installed by Transport-Key but not yet activated.
    staged_key_sequence: Option<u8>,
    /// A receive-triggered activation must be checkpointed before this key
    /// can secure outgoing traffic. Survives a failed lifecycle transaction;
    /// clearing the security context (leave/reset) also clears this gate.
    pub(crate) activation_pending: bool,
    /// Incoming frame counter table (for replay protection).
    /// Maps source IEEE address and network-key sequence to the last counter.
    ///
    /// This table is the *only* authority for incoming NWK replay floors:
    /// they are deliberately not persisted (see `docs/book/src/advanced/
    /// nv-storage.md`). It therefore never forgets a floor under capacity
    /// pressure — see [`Self::check_frame_counter_reserving`].
    frame_counter_table: heapless::Vec<FrameCounterEntry, MAX_FRAME_COUNTER_ENTRIES>,
    /// The retained previous key has given up at least one replay floor to
    /// make room for a current-key source, so it admits no new sources until
    /// the installed key set changes. A reclaimed source can then never be
    /// re-admitted at an old counter.
    #[cfg(feature = "router")]
    previous_key_replay_sealed: bool,
    /// Silence after which an unprotected floor may be evicted to admit a
    /// new source (see [`Self::set_replay_stale_timeout_secs`]).
    #[cfg(feature = "router")]
    replay_stale_secs: u16,
}

#[derive(Debug, Clone)]
struct FrameCounterEntry {
    source: IeeeAddress,
    key_sequence: u8,
    counter: u32,
    /// Seconds since this floor last advanced (saturating). Fits the
    /// structure's padding, so routers pay no RAM for it.
    #[cfg(feature = "router")]
    silent_secs: u16,
}

impl NwkSecurity {
    pub fn new() -> Self {
        Self {
            keys: [None, None],
            staged_key_sequence: None,
            activation_pending: false,
            frame_counter_table: heapless::Vec::new(),
            #[cfg(feature = "router")]
            previous_key_replay_sealed: false,
            #[cfg(feature = "router")]
            replay_stale_secs: DEFAULT_REPLAY_STALE_SECS,
        }
    }

    /// Re-open the retained previous key to new replay sources after the
    /// installed key set changed.
    #[inline]
    fn unseal_previous_key(&mut self) {
        #[cfg(feature = "router")]
        {
            self.previous_key_replay_sealed = false;
        }
    }

    /// Set how long a floor must be silent before a router may evict it to
    /// admit a new source. Floors of the parent and children are never
    /// evicted. `u16::MAX` (about 18 h) is the longest supported silence.
    #[cfg(feature = "router")]
    pub fn set_replay_stale_timeout_secs(&mut self, secs: u16) {
        self.replay_stale_secs = secs;
    }

    /// Current stale-floor eviction threshold in seconds.
    #[cfg(feature = "router")]
    pub fn replay_stale_timeout_secs(&self) -> u16 {
        self.replay_stale_secs
    }

    /// Advance the silence of every recorded floor by `elapsed_secs`.
    ///
    /// Called from the router's once-per-second NWK maintenance; a commit
    /// resets the floor's silence to zero.
    #[cfg(feature = "router")]
    pub fn age_replay_floors(&mut self, elapsed_secs: u16) {
        for entry in self.frame_counter_table.iter_mut() {
            entry.silent_secs = entry.silent_secs.saturating_add(elapsed_secs);
        }
    }

    /// Drop all live NWK keys and their replay state before a Trust Center
    /// rejoin waits for the current key under APS link-key protection.
    pub fn clear_network_keys(&mut self) {
        self.keys = [None, None];
        self.staged_key_sequence = None;
        self.activation_pending = false;
        self.frame_counter_table.clear();
        self.unseal_previous_key();
    }

    /// Set the active network key.
    pub fn set_network_key(&mut self, key: AesKey, seq_number: u8) {
        if let Some(active) = self.keys[0].as_mut()
            && active.seq_number == seq_number
        {
            let key_changed = active.key != key;
            active.key = key;
            active.active = true;
            self.staged_key_sequence = None;
            if key_changed {
                self.clear_frame_counters_for_key(seq_number);
            }
            return;
        }
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number && entry.key == key)
        {
            self.keys.swap(0, 1);
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            self.staged_key_sequence = None;
            self.unseal_previous_key();
            return;
        }

        self.clear_frame_counters_for_key(seq_number);
        let previous = self.keys[0].take().map(|mut entry| {
            entry.active = false;
            entry
        });
        self.keys[0] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: true,
        });
        self.keys[1] = previous.filter(|entry| entry.seq_number != seq_number);
        self.staged_key_sequence = None;
        self.retain_frame_counters_for_installed_keys();
    }

    /// Install a future network key without activating it.
    ///
    /// Returns `false` for conflicting active-key material or a sequence that
    /// is not newer in the 8-bit serial-number space. An identical delivery
    /// of the retained previous key is an idempotent success, not a new stage.
    pub fn stage_network_key(&mut self, key: AesKey, seq_number: u8) -> bool {
        let Some(active) = self.keys[0].as_ref() else {
            self.set_network_key(key, seq_number);
            return true;
        };
        if active.seq_number == seq_number {
            return active.key == key;
        }
        let newer = Self::key_sequence_is_newer(seq_number, active.seq_number);
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number && entry.key == key)
        {
            if newer {
                self.staged_key_sequence = Some(seq_number);
            }
            return true;
        }
        if !newer {
            return false;
        }

        self.clear_frame_counters_for_key(seq_number);
        self.keys[1] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: false,
        });
        self.staged_key_sequence = Some(seq_number);
        self.retain_frame_counters_for_installed_keys();
        true
    }

    /// Return the key waiting for Switch-Key or authenticated newer-key traffic.
    pub fn staged_key(&self) -> Option<&NetworkKeyEntry> {
        let sequence = self.staged_key_sequence?;
        self.key_by_seq(sequence)
    }

    /// Return the inactive network-key slot, whether it is a future staged
    /// key or the previous key retained after a switch.
    pub fn secondary_key(&self) -> Option<&NetworkKeyEntry> {
        self.keys[1].as_ref()
    }

    /// Whether any currently installed NWK key uses `key_fingerprint`.
    pub fn key_fingerprint_is_live(&self, key_fingerprint: u32) -> bool {
        self.keys
            .iter()
            .flatten()
            .any(|entry| zigbee_crypto::key_fingerprint(&entry.key) == key_fingerprint)
    }

    /// Whether `replay` still names the same installed NWK key.
    pub fn replay_counter_key_is_live(&self, replay: &NwkReplayCounter) -> bool {
        self.key_by_seq(replay.key_sequence).is_some_and(|entry| {
            zigbee_crypto::key_fingerprint(&entry.key) == replay.key_fingerprint
        })
    }

    /// Restore the previous inactive network key without making it a future
    /// Switch-Key target.
    pub fn restore_previous_network_key(&mut self, key: AesKey, seq_number: u8) -> bool {
        let Some(active) = self.keys[0].as_ref() else {
            return false;
        };
        if active.seq_number == seq_number {
            return active.key == key;
        }
        self.clear_frame_counters_for_key(seq_number);
        self.keys[1] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: false,
        });
        self.staged_key_sequence = None;
        self.retain_frame_counters_for_installed_keys();
        true
    }

    /// Activate only a staged, strictly newer network key by sequence number.
    ///
    /// Repeating the active sequence is an idempotent success. A retained
    /// previous key is never a Switch-Key target, including after a duplicate
    /// Transport-Key delivery; sequence recency includes 255 -> 0 wrap.
    pub fn activate_network_key(&mut self, seq_number: u8) -> bool {
        if self.keys[0]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number)
        {
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            return true;
        }
        if self.staged_key_sequence != Some(seq_number)
            || !self
                .active_key()
                .is_some_and(|active| Self::key_sequence_is_newer(seq_number, active.seq_number))
        {
            return false;
        }
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number)
        {
            self.keys.swap(0, 1);
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            self.staged_key_sequence = None;
            self.unseal_previous_key();
            return true;
        }
        false
    }

    /// Activate only a staged, strictly newer key after receive admission,
    /// replay checking and MIC verification. A retained previous key is not
    /// an activation candidate, even if its sequence is numerically larger.
    pub(crate) fn activate_received_key(&mut self, seq_number: u8) -> bool {
        // Unlike an explicit Switch-Key, observing the already-active key
        // must not report a new activation or acquire a fresh transmit fence.
        self.staged_key_sequence == Some(seq_number) && self.activate_network_key(seq_number)
    }

    /// Strict serial-number recency: 255 -> 0 advances; the ambiguous
    /// half-window is deliberately not eligible for activation.
    const fn key_sequence_is_newer(candidate: u8, active: u8) -> bool {
        let delta = candidate.wrapping_sub(active);
        delta != 0 && delta < 0x80
    }

    /// Retire the previous key retained after a successful Switch-Key.
    ///
    /// A future staged key is never removed by this operation. The caller
    /// must durably record the retirement intent before calling this method,
    /// tombstone the retired key's replay domains, and only then checkpoint
    /// the now-single-key state.
    pub fn retire_previous_network_key(&mut self) -> Option<NetworkKeyEntry> {
        if self.staged_key_sequence.is_some() {
            return None;
        }
        let retired = self.keys[1].take()?;
        self.retain_frame_counters_for_installed_keys();
        Some(retired)
    }

    /// Get the active network key.
    pub fn active_key(&self) -> Option<&NetworkKeyEntry> {
        self.keys[0].as_ref().filter(|k| k.active)
    }

    /// Find key by sequence number.
    pub fn key_by_seq(&self, seq: u8) -> Option<&NetworkKeyEntry> {
        self.keys.iter().flatten().find(|k| k.seq_number == seq)
    }

    fn clear_frame_counters_for_key(&mut self, key_sequence: u8) {
        self.frame_counter_table
            .retain(|entry| entry.key_sequence != key_sequence);
    }

    fn retain_frame_counters_for_installed_keys(&mut self) {
        self.unseal_previous_key();
        let active_sequence = self.keys[0].as_ref().map(|entry| entry.seq_number);
        let alternate_sequence = self.keys[1].as_ref().map(|entry| entry.seq_number);
        self.frame_counter_table.retain(|entry| {
            Some(entry.key_sequence) == active_sequence
                || Some(entry.key_sequence) == alternate_sequence
        });
    }

    /// Clear all incoming replay state for one device identity.
    pub fn clear_frame_counters_for_source(&mut self, source: &IeeeAddress) {
        self.frame_counter_table
            .retain(|entry| entry.source != *source);
    }

    /// Check incoming frame counter (replay protection) WITHOUT committing.
    /// Returns true if the frame counter is valid (newer than last seen).
    /// Call `commit_frame_counter()` AFTER successful MIC verification.
    pub fn check_frame_counter(&self, source: &IeeeAddress, counter: u32) -> bool {
        let key_sequence = self.active_key().map(|entry| entry.seq_number).unwrap_or(0);
        self.check_frame_counter_for_key(source, key_sequence, counter)
    }

    /// Check an incoming counter in the replay domain of one network key.
    ///
    /// Conservative form of the admission check: no slots are reserved and
    /// no floor counts as evictable, so a full table refuses a new domain.
    pub fn check_frame_counter_for_key(
        &self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
    ) -> bool {
        #[cfg(feature = "router")]
        let admission =
            self.check_frame_counter_reserving(source, key_sequence, counter, || 0, |_| true);
        #[cfg(not(feature = "router"))]
        let admission = self.replay_admission(source, key_sequence, counter);
        admission == ReplayAdmission::Fresh
    }

    /// Check an incoming counter WITHOUT committing it (end-device build).
    ///
    /// A known `(source, key_sequence)` domain admits only a strictly greater
    /// counter (R22 §4.3.1.2). An unknown domain is admitted only into a free
    /// slot: incoming NWK floors are RAM-only, so this table is their sole
    /// authority for the rest of the boot and never forgets one. A new
    /// domain that does not fit is refused (fail closed).
    #[cfg(not(feature = "router"))]
    pub fn replay_admission(
        &self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
    ) -> ReplayAdmission {
        match self
            .frame_counter_table
            .iter()
            .find(|e| e.source == *source && e.key_sequence == key_sequence)
        {
            Some(entry) if counter > entry.counter => ReplayAdmission::Fresh,
            Some(_) => ReplayAdmission::Replay,
            None if self.frame_counter_table.is_full() => ReplayAdmission::TableFull,
            None => ReplayAdmission::Fresh,
        }
    }

    /// Check an incoming counter WITHOUT committing it (router build).
    ///
    /// A known `(source, key_sequence)` domain admits only a strictly greater
    /// counter (R22 §4.3.1.2). An unknown domain is admitted only while the
    /// table can record its floor: a free slot, a reclaimable floor of the
    /// retained *previous* key, or a stale floor (see
    /// [`Self::commit_frame_counter_for_key_protecting`]). More than
    /// `reserved()` such slots must remain, so the caller keeps room for its
    /// parent and children (`reserved` is only evaluated for a new domain).
    ///
    /// `protect` names the sources whose floors are never evicted (the
    /// parent and children). A new domain that does not fit is refused with
    /// [`ReplayAdmission::TableFull`] (fail closed).
    #[cfg(feature = "router")]
    pub fn check_frame_counter_reserving(
        &self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
        reserved: impl FnOnce() -> usize,
        protect: impl Fn(&IeeeAddress) -> bool,
    ) -> ReplayAdmission {
        if let Some(entry) = self
            .frame_counter_table
            .iter()
            .find(|e| e.source == *source && e.key_sequence == key_sequence)
        {
            return if counter > entry.counter {
                ReplayAdmission::Fresh
            } else {
                ReplayAdmission::Replay
            };
        }
        let Some(reclaim_from) = self.reclaimable_key_sequence(key_sequence) else {
            return ReplayAdmission::TableFull;
        };
        let reclaimable = reclaim_from.map_or(0, |previous| {
            self.frame_counter_table
                .iter()
                .filter(|e| e.key_sequence == previous)
                .count()
        });
        let available = MAX_FRAME_COUNTER_ENTRIES - self.frame_counter_table.len() + reclaimable;
        let reserved = reserved();
        if available > reserved {
            return ReplayAdmission::Fresh;
        }
        // Only now pay for the protection lookups: count stale floors (other
        // than the reclaimable ones already counted) until enough are found.
        let stale = self
            .frame_counter_table
            .iter()
            .filter(|e| {
                Some(e.key_sequence) != reclaim_from
                    && e.silent_secs >= self.replay_stale_secs
                    && !protect(&e.source)
            })
            .take(reserved + 1 - available)
            .count();
        if available + stale > reserved {
            ReplayAdmission::Fresh
        } else {
            ReplayAdmission::TableFull
        }
    }

    /// Commit frame counter after successful MIC verification.
    /// Must only be called after decrypt/verify succeeds.
    pub fn commit_frame_counter(&mut self, source: &IeeeAddress, counter: u32) -> bool {
        let key_sequence = self.active_key().map(|entry| entry.seq_number).unwrap_or(0);
        self.commit_frame_counter_for_key(source, key_sequence, counter)
    }

    /// Commit a verified counter in the replay domain of one network key.
    ///
    /// Returns `false`, recording nothing, when a new domain does not fit.
    /// No floor is evicted by this form; see
    /// [`Self::commit_frame_counter_for_key_protecting`] on routers.
    pub fn commit_frame_counter_for_key(
        &mut self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
    ) -> bool {
        #[cfg(feature = "router")]
        {
            self.commit_frame_counter_for_key_protecting(source, key_sequence, counter, |_| true)
        }
        #[cfg(not(feature = "router"))]
        {
            if let Some(entry) = self
                .frame_counter_table
                .iter_mut()
                .find(|e| e.source == *source && e.key_sequence == key_sequence)
            {
                entry.counter = entry.counter.max(counter);
                return true;
            }
            let recorded = self
                .frame_counter_table
                .push(FrameCounterEntry {
                    source: *source,
                    key_sequence,
                    counter,
                })
                .is_ok();
            if !recorded {
                log::warn!("[NWK] Replay table full — refusing a new source");
            }
            recorded
        }
    }

    /// Commit a verified counter, making room for a new domain if needed
    /// (router build).
    ///
    /// Returns `false`, recording nothing, when a new domain does not fit.
    /// A full table makes room, in order, by:
    /// 1. giving up a floor of the retained previous key (never the active or
    ///    a staged key), then sealing that key against new sources so the
    ///    reclaimed source cannot be re-admitted under it;
    /// 2. evicting the longest-silent floor whose source has been silent for
    ///    at least the stale timeout and that `protect` does not claim (the
    ///    NWK layer protects its parent and children).
    ///
    /// An evicted source re-enters at whatever counter it next presents, so
    /// frames it sent before falling silent could be replayed once. The
    /// window is limited to neighbours unheard for the whole timeout.
    #[cfg(feature = "router")]
    pub fn commit_frame_counter_for_key_protecting(
        &mut self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
        protect: impl Fn(&IeeeAddress) -> bool,
    ) -> bool {
        if let Some(entry) = self
            .frame_counter_table
            .iter_mut()
            .find(|e| e.source == *source && e.key_sequence == key_sequence)
        {
            entry.counter = entry.counter.max(counter);
            entry.silent_secs = 0;
            return true;
        }
        let Some(reclaim_from) = self.reclaimable_key_sequence(key_sequence) else {
            log::warn!("[NWK] Replay table: previous key sealed — refusing a new source");
            return false;
        };
        let entry = FrameCounterEntry {
            source: *source,
            key_sequence,
            counter,
            silent_secs: 0,
        };
        if self.frame_counter_table.push(entry.clone()).is_ok() {
            return true;
        }
        if let Some(slot) = reclaim_from.and_then(|previous| {
            self.frame_counter_table
                .iter_mut()
                .find(|e| e.key_sequence == previous)
        }) {
            log::warn!("[NWK] Replay table full — retiring a previous-key floor");
            *slot = entry;
            self.previous_key_replay_sealed = true;
            return true;
        }
        let stale_secs = self.replay_stale_secs;
        let Some(slot) = self
            .frame_counter_table
            .iter_mut()
            .filter(|e| e.silent_secs >= stale_secs && !protect(&e.source))
            .max_by_key(|e| e.silent_secs)
        else {
            log::warn!("[NWK] Replay table full — refusing a new source");
            return false;
        };
        log::warn!(
            "[NWK] Replay table full — evicting a floor silent for {}s",
            slot.silent_secs
        );
        *slot = entry;
        true
    }

    /// Admission rule for a new domain under `key_sequence`.
    ///
    /// `None`: refused, because the retained previous key is sealed.
    /// `Some(Some(previous))`: the domain may reclaim a floor of the
    /// retained previous key when the table is full. `Some(None)`: free slots
    /// only — there is no previous key, or the newcomer belongs to it.
    #[cfg(feature = "router")]
    fn reclaimable_key_sequence(&self, key_sequence: u8) -> Option<Option<u8>> {
        let previous = self.keys[1]
            .as_ref()
            .map(|entry| entry.seq_number)
            .filter(|sequence| self.staged_key_sequence != Some(*sequence));
        match previous {
            Some(previous) if previous == key_sequence => {
                (!self.previous_key_replay_sealed).then_some(None)
            }
            previous => Some(previous),
        }
    }

    /// Whether `(source, key_sequence)` already has a recorded floor.
    pub fn has_frame_counter_for_key(&self, source: &IeeeAddress, key_sequence: u8) -> bool {
        self.frame_counter_table
            .iter()
            .any(|e| e.source == *source && e.key_sequence == key_sequence)
    }

    /// Restore a durable replay floor only when the installed key still
    /// matches the key identity that produced it.
    pub fn restore_replay_counter(&mut self, replay: NwkReplayCounter) -> bool {
        let Some(key) = self.key_by_seq(replay.key_sequence) else {
            return false;
        };
        if zigbee_crypto::key_fingerprint(&key.key) != replay.key_fingerprint {
            return false;
        }
        self.commit_frame_counter_for_key(&replay.source, replay.key_sequence, replay.counter)
    }

    /// Encrypt a NWK frame payload using AES-128-CCM*.
    ///
    /// # Arguments
    /// * `nwk_header` - Serialized NWK header (used as 'a' in CCM*)
    /// * `payload` - Plaintext NWK payload to encrypt
    /// * `key` - Network key to use
    /// * `security_header` - Security auxiliary header
    ///
    /// Returns: encrypted payload + 4-byte MIC appended.
    ///
    /// Software-AES wrapper over [`Self::encrypt_with`] — used by host tests
    /// and any caller without a platform. The embedded data path uses
    /// [`Self::encrypt_with`] with the MAC so a hardware AES backend can
    /// serve CCM*.
    pub fn encrypt(
        &self,
        nwk_header: &[u8],
        payload: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        self.encrypt_with(
            &mut SoftwareAesProvider::new(),
            nwk_header,
            payload,
            key,
            security_header,
        )
    }

    /// Encrypt a NWK frame payload using CCM* keyed by `provider`.
    ///
    /// The forward AES permutation comes from `provider.forward_cipher(key)`,
    /// which is the software core by default and a hardware accelerator on
    /// platforms that override [`ForwardAesProvider::forward_cipher`]. A
    /// hardware AES failure is surfaced as `None` (the caller reports
    /// `BadCcmOutput`) rather than silently retried in software — see the
    /// `Err` arm below.
    pub fn encrypt_with<P: ForwardAesProvider>(
        &self,
        provider: &mut P,
        nwk_header: &[u8],
        payload: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        // AES-128-CCM* encryption with:
        // - M=4 (MIC length)
        // - a = nwk_header || security_header (authenticated but not encrypted)
        // - m = payload (encrypted and authenticated)
        let mut cipher = provider.forward_cipher(key);
        match ccm_star_encrypt_with(&mut cipher, &nonce, nwk_header, payload) {
            Ok(result) => result,
            Err(_hardware_error) => {
                log::error!("[NWK] Hardware AES encrypt failed; dropping frame");
                None
            }
        }
    }

    /// Decrypt a NWK frame payload (software-AES wrapper over
    /// [`Self::decrypt_with`]).
    pub fn decrypt(
        &self,
        nwk_header: &[u8],
        ciphertext: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        self.decrypt_with(
            &mut SoftwareAesProvider::new(),
            nwk_header,
            ciphertext,
            key,
            security_header,
        )
    }

    /// Decrypt and verify a NWK frame payload using CCM* keyed by `provider`.
    ///
    /// Returns `None` on a MIC mismatch (forged/tampered frame) **and** on a
    /// hardware AES failure — both are hard failures with no software
    /// fall-back. The caller must not commit the replay frame counter unless
    /// this returns `Some`.
    pub fn decrypt_with<P: ForwardAesProvider>(
        &self,
        provider: &mut P,
        nwk_header: &[u8],
        ciphertext: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        let mut cipher = provider.forward_cipher(key);
        match ccm_star_decrypt_with(&mut cipher, &nonce, nwk_header, ciphertext) {
            Ok(result) => result,
            Err(_hardware_error) => {
                log::error!("[NWK] Hardware AES decrypt failed; dropping frame");
                None
            }
        }
    }

    /// Build CCM* nonce from security header.
    /// Nonce = source_address(8) || frame_counter(4) || security_control(1)
    ///
    /// Per Zigbee spec §4.3.1.2: the SecurityLevel in the nonce must use the
    /// ACTUAL security level (5 = ENC-MIC-32), not the OTA value (always 0).
    fn build_nonce(&self, hdr: &NwkSecurityHeader) -> [u8; 13] {
        let mut nonce = [0u8; 13];
        nonce[0..8].copy_from_slice(&hdr.source_address);
        nonce[8..12].copy_from_slice(&hdr.frame_counter.to_le_bytes());
        // Replace OTA security level (0) with actual level (5 = ENC-MIC-32)
        let actual_sc = (hdr.security_control & !0x07) | 0x05;
        nonce[12] = actual_sc;
        nonce
    }
}

impl Default for NwkSecurity {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nwk_security_builds_the_expected_nonce() {
        let security = NwkSecurity::new();
        let header = NwkSecurityHeader {
            security_control: 0x28,
            frame_counter: 1,
            source_address: [0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08],
            key_seq_number: 0,
        };
        let expected_nonce = [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ];

        assert_eq!(security.build_nonce(&header), expected_nonce);
    }

    #[test]
    fn nwk_security_matches_the_shared_ccm_golden_vector() {
        let key = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ];
        let header = NwkSecurityHeader {
            security_control: 0x28,
            frame_counter: 1,
            source_address: [0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08],
            key_seq_number: 0,
        };
        let expected = [
            0xAC, 0x17, 0x74, 0xEC, 0x17, 0x76, 0xC2, 0x7C, 0x41, 0xEE, 0x31, 0x0A, 0xE0, 0x0B,
            0x5B, 0x5A, 0xA0, 0x05, 0xC9,
        ];

        let encrypted = NwkSecurity::new()
            .encrypt(b"NWK-HDR+AUX", b"hello-nwk-frame", &key, &header)
            .expect("encrypt");
        assert_eq!(encrypted.as_slice(), expected);
    }

    /// A hardware-shaped forward-AES provider (a cipher with a *fallible*,
    /// non-`Infallible` error, like the real TLSR8258 backend) drives
    /// [`NwkSecurity::encrypt_with`]/[`decrypt_with`] to byte-identical
    /// output as the software wrapper, and a *failing* provider surfaces a
    /// hard `None` (dropped frame) rather than a silent software fall-back.
    #[test]
    fn nwk_security_is_backend_agnostic_and_surfaces_hardware_errors() {
        use zigbee_crypto::{Aes128Forward, ForwardAesProvider, SoftwareAes128};

        struct FlakyCipher {
            inner: SoftwareAes128,
            fail: bool,
        }
        impl Aes128Forward for FlakyCipher {
            type Error = ();
            fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), ()> {
                if self.fail {
                    return Err(());
                }
                self.inner.encrypt_block(block).unwrap();
                Ok(())
            }
        }
        struct FlakyProvider {
            fail: bool,
        }
        impl ForwardAesProvider for FlakyProvider {
            fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
                FlakyCipher {
                    inner: SoftwareAes128::new(key),
                    fail: self.fail,
                }
            }
        }

        let key = [0x24; 16];
        let header = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: 7,
            source_address: [0xAA; 8],
            key_seq_number: 0,
        };
        let security = NwkSecurity::new();

        // Equivalence: hardware-shaped provider == software wrapper.
        let via_sw = security
            .encrypt(b"aad", b"secret-payload", &key, &header)
            .expect("sw encrypt");
        let via_hw = security
            .encrypt_with(
                &mut FlakyProvider { fail: false },
                b"aad",
                b"secret-payload",
                &key,
                &header,
            )
            .expect("hw encrypt");
        assert_eq!(via_sw, via_hw);
        let round_trip = security
            .decrypt_with(
                &mut FlakyProvider { fail: false },
                b"aad",
                &via_hw,
                &key,
                &header,
            )
            .expect("hw decrypt");
        assert_eq!(round_trip.as_slice(), b"secret-payload");

        // A failing backend drops the frame (None), never silently
        // succeeding or falling back to software.
        assert!(
            security
                .encrypt_with(
                    &mut FlakyProvider { fail: true },
                    b"aad",
                    b"secret-payload",
                    &key,
                    &header
                )
                .is_none()
        );
        assert!(
            security
                .decrypt_with(
                    &mut FlakyProvider { fail: true },
                    b"aad",
                    &via_hw,
                    &key,
                    &header
                )
                .is_none()
        );
    }

    #[test]
    fn staging_network_key_alone_does_not_activate_it() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        assert!(security.stage_network_key([0x22; 16], 2));

        assert_eq!(security.active_key().unwrap().seq_number, 1);
        assert_eq!(security.key_by_seq(2).unwrap().key, [0x22; 16]);
        assert!(!security.key_by_seq(2).unwrap().active);
        assert_eq!(security.staged_key().unwrap().seq_number, 2);

        assert!(security.activate_network_key(2));
        assert_eq!(security.active_key().unwrap().seq_number, 2);
        assert!(security.staged_key().is_none());
        assert_eq!(security.key_by_seq(1).unwrap().key, [0x11; 16]);
        assert!(!security.key_by_seq(1).unwrap().active);
        assert!(!security.activate_network_key(3));
    }

    #[test]
    fn receive_activation_requires_staged_marker_and_strict_serial_recency() {
        for (active, candidate, newer) in [
            (3, 4, true),
            (255, 0, true),
            (0, 255, false),
            (4, 3, false),
            (3, 3, false),
            (0, 127, true),
            (0, 128, false),
            (128, 0, false),
        ] {
            let mut security = NwkSecurity::new();
            security.set_network_key([0x11; 16], active);
            // A retained slot is never an implicit-switch candidate, even if
            // the sequence by itself would compare newer.
            assert!(security.restore_previous_network_key([0x11; 16], candidate));
            assert!(!security.activate_received_key(candidate));
            assert_eq!(
                security.activate_network_key(candidate),
                candidate == active
            );
            assert_eq!(security.active_key().unwrap().seq_number, active);

            if candidate != active {
                assert!(security.stage_network_key([0x11; 16], candidate));
            }
            assert_eq!(security.activate_received_key(candidate), newer);
            assert_eq!(
                security.active_key().unwrap().seq_number,
                if newer { candidate } else { active },
            );
        }
    }

    #[test]
    fn stale_transport_key_does_not_displace_a_staged_key_or_reset_replay() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 255);
        assert!(security.stage_network_key([0x22; 16], 0));
        assert!(security.activate_received_key(0));
        security.commit_frame_counter_for_key(&[0x44; 8], 255, 100);

        assert!(security.stage_network_key([0x11; 16], 255));
        assert!(security.staged_key().is_none());
        assert!(!security.activate_received_key(255));
        assert!(!security.activate_network_key(255));
        assert!(!security.check_frame_counter_for_key(&[0x44; 8], 255, 100));
        assert!(!security.stage_network_key([0x33; 16], 255));

        assert!(security.stage_network_key([0x33; 16], 1));
        assert!(!security.stage_network_key([0x11; 16], 255));
        assert_eq!(security.staged_key().unwrap().seq_number, 1);
        assert_eq!(security.active_key().unwrap().seq_number, 0);
    }

    #[test]
    fn replay_counters_are_scoped_by_network_key_sequence() {
        let source = [0x44; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);

        assert!(!security.check_frame_counter_for_key(&source, 1, 1));
        assert!(security.check_frame_counter_for_key(&source, 2, 1));
        security.commit_frame_counter_for_key(&source, 2, 1);
        assert!(!security.check_frame_counter_for_key(&source, 2, 1));
        assert!(!security.check_frame_counter_for_key(&source, 1, 100));
    }

    #[test]
    fn a_full_replay_table_never_forgets_a_floor() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        let source = |i: usize| [i as u8 + 1; 8];
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(security.commit_frame_counter_for_key(&source(i), 1, 100));
        }

        // A new source is refused (fail closed) instead of evicting anyone.
        let newcomer = [0xF0; 8];
        assert!(!security.check_frame_counter_for_key(&newcomer, 1, 5));
        assert!(!security.commit_frame_counter_for_key(&newcomer, 1, 5));

        // Every recorded floor survives, and known sources still advance.
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(!security.check_frame_counter_for_key(&source(i), 1, 100));
            assert!(security.check_frame_counter_for_key(&source(i), 1, 101));
        }
        assert!(security.commit_frame_counter_for_key(&source(0), 1, 101));
        assert!(!security.check_frame_counter_for_key(&source(0), 1, 101));
    }

    #[test]
    #[cfg(feature = "router")]
    fn reserved_slots_keep_room_for_protected_neighbours() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        let source = |i: usize| [i as u8 + 1; 8];
        for i in 0..MAX_FRAME_COUNTER_ENTRIES - 1 {
            assert!(security.commit_frame_counter_for_key(&source(i), 1, 100));
        }
        let stranger = [0xF0; 8];
        // One free slot, still needed by a parent/child without a floor.
        assert_eq!(
            security.check_frame_counter_reserving(&stranger, 1, 5, || 1, |_| false),
            ReplayAdmission::TableFull
        );
        // The protected source itself reserves nothing for itself.
        assert_eq!(
            security.check_frame_counter_reserving(&stranger, 1, 5, || 0, |_| false),
            ReplayAdmission::Fresh
        );
    }

    #[test]
    fn a_replay_and_a_full_table_are_reported_differently() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        let source = |i: usize| [i as u8 + 1; 8];
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(security.commit_frame_counter_for_key(&source(i), 1, 100));
        }
        #[cfg(feature = "router")]
        let admission = |security: &NwkSecurity, source: &IeeeAddress, counter| {
            security.check_frame_counter_reserving(source, 1, counter, || 0, |_| false)
        };
        #[cfg(not(feature = "router"))]
        let admission = |security: &NwkSecurity, source: &IeeeAddress, counter| {
            security.replay_admission(source, 1, counter)
        };
        assert_eq!(
            admission(&security, &source(0), 100),
            ReplayAdmission::Replay
        );
        assert_eq!(
            admission(&security, &source(0), 101),
            ReplayAdmission::Fresh
        );
        assert_eq!(
            admission(&security, &[0xF0; 8], 1),
            ReplayAdmission::TableFull
        );
    }

    /// Fill the table with key-1 floors for sources `1..=MAX`, then let
    /// `silent` seconds pass for all of them.
    #[cfg(feature = "router")]
    fn full_router_table(silent: u16) -> NwkSecurity {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(security.commit_frame_counter_for_key(&[i as u8 + 1; 8], 1, 100));
        }
        security.age_replay_floors(silent);
        security
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_stale_unprotected_floor_is_evicted_for_a_new_source() {
        let mut security = full_router_table(DEFAULT_REPLAY_STALE_SECS);
        // Source 1 keeps talking; every other floor has been silent an hour.
        assert!(security.commit_frame_counter_for_key(&[1; 8], 1, 101));
        let newcomer = [0xF0; 8];
        let unprotected = |_: &IeeeAddress| false;

        assert_eq!(
            security.check_frame_counter_reserving(&newcomer, 1, 5, || 0, unprotected),
            ReplayAdmission::Fresh
        );
        assert!(security.commit_frame_counter_for_key_protecting(&newcomer, 1, 5, unprotected));
        assert!(security.has_frame_counter_for_key(&newcomer, 1));
        assert!(!security.check_frame_counter_for_key(&newcomer, 1, 5));
        // Exactly one stale floor went; the fresh one stayed.
        let survivors = (0..MAX_FRAME_COUNTER_ENTRIES)
            .filter(|i| security.has_frame_counter_for_key(&[*i as u8 + 1; 8], 1))
            .count();
        assert_eq!(survivors, MAX_FRAME_COUNTER_ENTRIES - 1);
        assert!(security.has_frame_counter_for_key(&[1; 8], 1));
        assert!(!security.check_frame_counter_for_key(&[1; 8], 1, 101));
    }

    #[test]
    #[cfg(feature = "router")]
    fn the_longest_silent_floor_is_evicted_first() {
        let mut security = full_router_table(0);
        security.age_replay_floors(DEFAULT_REPLAY_STALE_SECS);
        // Everyone but source 7 speaks once, then another hour passes.
        for i in (0..MAX_FRAME_COUNTER_ENTRIES).filter(|i| *i != 6) {
            assert!(security.commit_frame_counter_for_key(&[i as u8 + 1; 8], 1, 101));
        }
        security.age_replay_floors(DEFAULT_REPLAY_STALE_SECS);
        assert!(security.commit_frame_counter_for_key_protecting(&[0xF0; 8], 1, 5, |_| false));
        assert!(!security.has_frame_counter_for_key(&[7; 8], 1));
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_recently_heard_floor_is_never_evicted() {
        let mut security = full_router_table(DEFAULT_REPLAY_STALE_SECS - 1);
        let newcomer = [0xF0; 8];
        assert_eq!(
            security.check_frame_counter_reserving(&newcomer, 1, 5, || 0, |_| false),
            ReplayAdmission::TableFull
        );
        assert!(!security.commit_frame_counter_for_key_protecting(&newcomer, 1, 5, |_| false));
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(!security.check_frame_counter_for_key(&[i as u8 + 1; 8], 1, 100));
        }
        // One more second makes them all stale.
        security.age_replay_floors(1);
        assert!(security.commit_frame_counter_for_key_protecting(&newcomer, 1, 5, |_| false));
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_and_child_floors_are_never_evicted_even_when_stale() {
        let mut security = full_router_table(u16::MAX);
        let newcomer = [0xF0; 8];
        // Every source is the parent or a child.
        assert_eq!(
            security.check_frame_counter_reserving(&newcomer, 1, 5, || 0, |_| true),
            ReplayAdmission::TableFull
        );
        assert!(!security.commit_frame_counter_for_key_protecting(&newcomer, 1, 5, |_| true));
        // The conservative form never evicts at all.
        assert!(!security.check_frame_counter_for_key(&newcomer, 1, 5));
        assert!(!security.commit_frame_counter_for_key(&newcomer, 1, 5));
        // Only the single unprotected source can give way.
        let protect = |source: &IeeeAddress| *source != [5; 8];
        assert!(security.commit_frame_counter_for_key_protecting(&newcomer, 1, 5, protect));
        assert!(!security.has_frame_counter_for_key(&[5; 8], 1));
        assert!(!security.commit_frame_counter_for_key_protecting(&[0xF1; 8], 1, 5, protect));
        for i in (0..MAX_FRAME_COUNTER_ENTRIES).filter(|i| *i != 4) {
            assert!(security.has_frame_counter_for_key(&[i as u8 + 1; 8], 1));
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn the_stale_timeout_is_configurable() {
        let mut security = full_router_table(0);
        assert_eq!(
            security.replay_stale_timeout_secs(),
            DEFAULT_REPLAY_STALE_SECS
        );
        security.set_replay_stale_timeout_secs(60);
        security.age_replay_floors(59);
        assert!(!security.commit_frame_counter_for_key_protecting(&[0xF0; 8], 1, 5, |_| false));
        security.age_replay_floors(1);
        assert!(security.commit_frame_counter_for_key_protecting(&[0xF0; 8], 1, 5, |_| false));
        // A longer timeout makes the remaining one-minute-old floors fresh.
        security.set_replay_stale_timeout_secs(DEFAULT_REPLAY_STALE_SECS);
        assert!(!security.commit_frame_counter_for_key_protecting(&[0xF1; 8], 1, 5, |_| false));
    }

    #[test]
    #[cfg(feature = "router")]
    fn reserved_slots_also_cap_stale_eviction() {
        let security = full_router_table(DEFAULT_REPLAY_STALE_SECS);
        let stale_but_two = |source: &IeeeAddress| source[0] > 2;
        // Two evictable floors: a stranger may take one while one parent/child
        // still needs room, but not while two still do.
        assert_eq!(
            security.check_frame_counter_reserving(&[0xF0; 8], 1, 5, || 1, stale_but_two),
            ReplayAdmission::Fresh
        );
        assert_eq!(
            security.check_frame_counter_reserving(&[0xF0; 8], 1, 5, || 2, stale_but_two),
            ReplayAdmission::TableFull
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_full_table_reclaims_only_previous_key_floors_and_seals_that_key() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        let source = |i: usize| [i as u8 + 1; 8];
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(security.commit_frame_counter_for_key(&source(i), 1, 100));
        }
        // Switch to key 2; key 1 stays installed as the previous key.
        security.set_network_key([0x22; 16], 2);
        assert_eq!(security.secondary_key().map(|k| k.seq_number), Some(1));

        // A current-key newcomer takes a previous-key floor...
        assert!(security.check_frame_counter_for_key(&source(0), 2, 200));
        assert!(security.commit_frame_counter_for_key(&source(0), 2, 200));
        // ...and the reclaimed previous-key source cannot come back under the
        // previous key at an old counter, nor can any other new source.
        assert!(!security.check_frame_counter_for_key(&source(0), 1, 1));
        assert!(!security.check_frame_counter_for_key(&[0xF0; 8], 1, 1));
        assert!(!security.commit_frame_counter_for_key(&[0xF0; 8], 1, 1));
        // Unreclaimed previous-key floors still hold and still advance.
        for i in 1..MAX_FRAME_COUNTER_ENTRIES {
            assert!(!security.check_frame_counter_for_key(&source(i), 1, 100));
            assert!(security.check_frame_counter_for_key(&source(i), 1, 101));
        }
        // Current-key floors are never reclaimed.
        assert!(!security.check_frame_counter_for_key(&source(0), 2, 200));
    }

    #[test]
    fn active_and_staged_key_floors_are_never_reclaimed() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        let source = |i: usize| [i as u8 + 1; 8];
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(security.commit_frame_counter_for_key(&source(i), 1, 100));
        }
        assert!(security.stage_network_key([0x22; 16], 2));
        // The staged key's first sender does not displace an active-key floor.
        assert!(!security.check_frame_counter_for_key(&[0xF0; 8], 2, 1));
        assert!(!security.commit_frame_counter_for_key(&[0xF0; 8], 2, 1));
        for i in 0..MAX_FRAME_COUNTER_ENTRIES {
            assert!(!security.check_frame_counter_for_key(&source(i), 1, 100));
        }
    }

    #[test]
    fn replacing_key_material_resets_that_sequences_replay_domain() {
        let source = [0x44; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);

        security.set_network_key([0x22; 16], 1);

        assert!(security.check_frame_counter_for_key(&source, 1, 0));
    }

    #[test]
    fn clearing_one_sources_replay_state_preserves_other_devices() {
        let source = [0x44; 8];
        let other = [0x55; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);
        security.commit_frame_counter_for_key(&source, 2, 200);
        security.commit_frame_counter_for_key(&other, 1, 300);

        security.clear_frame_counters_for_source(&source);

        assert!(security.check_frame_counter_for_key(&source, 1, 0));
        assert!(security.check_frame_counter_for_key(&source, 2, 0));
        assert!(!security.check_frame_counter_for_key(&other, 1, 1));
    }
}
