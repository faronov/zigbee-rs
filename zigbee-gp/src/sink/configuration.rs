//! GP Pairing Configuration and the Translation Table of the sink (feature
//! `translation-table`): a commissioning tool (a hub, through a network
//! co-processor) configures Sink Table entries and per-GPD translations
//! remotely; GPD commands with explicit translations drive the configured
//! local endpoints and ZCL commands, all others keep the built-in default
//! translation ([`crate::translation`]).
//!
//! Behaviour mirrors gecko_sdk 4.5 `green-power-server.c`
//! (`emberAfGreenPowerClusterGpPairingConfigurationCallback`) and
//! `green-power-translation-table.c`, with these explicit differences:
//!
//! * every refusal is answered with a ZCL status instead of a silent drop
//!   (reserved GPD ID or security level, a security level below
//!   gpsSecurityLevel, unknown entry);
//! * Application Description (action 0b101), and Extend/Replace announcing
//!   one ("application description follows"), are refused with FAILURE:
//!   this sink keeps no report descriptors;
//! * a GP Translation Table Update for a GPD without a Sink Table entry is
//!   refused with NOT_FOUND (gecko_sdk accepts it): translations live and
//!   are persisted with their pairing;
//! * Extend keeps the GPD's translations and merges commissioned groups;
//!   Replace starts a fresh entry and drops the GPD's translations
//!   (gecko_sdk drops them for both);
//! * the sink's own proxy is not put into commissioning mode: a combo hears
//!   its GPDs directly, and remote proxies learn the pairing from the GP
//!   Pairing the "send GP Pairing" flag requests.
//!
//! Application information (manufacturer and model ID, GPD command and
//! cluster lists, switch information) is validated but not stored.

use heapless::Vec;

use super::{
    Out, Received, SINK_COMMAND_QUEUE_LEN, ServerCommand, Sink, SinkCommand, SinkDrop, SinkEntry,
    SinkEvent, SinkOutcome, security_level_attr, zcl_status,
};
use crate::cluster::{Pairing, PairingSink, server_cmd};
use crate::config::{
    PASS_TO_APPLICATION, PairingConfiguration, TranslationTableRequest, TranslationTableUpdate,
    paired_endpoints, pairing_action,
};
use crate::proxy::{GroupChange, MAX_SINK_GROUPS, MAX_TX_PAYLOAD, SinkGroup};
use crate::translation::{self, Translation};
use crate::translation_table::{
    MAX_PAIRED_ENDPOINTS, PairedEndpoints, TranslationEntry, TranslationTable, ZclPayload,
};
use crate::types::{CommunicationMode, GpdId, KeyType, SecurityLevel, derive_alias};
use crate::{Error, Result};

/// Translation Table capacity of the sink (4 with
/// `compact-translation-table`).
pub const SINK_TRANSLATION_ENTRIES: usize = if cfg!(feature = "compact-translation-table") {
    4
} else {
    16
};
/// ZCL FAILURE.
const FAILURE: u8 = 0x01;
/// ZCL INSUFFICIENT_SPACE.
const INSUFFICIENT_SPACE: u8 = 0x89;

/// The sink's Translation Table type.
pub type SinkTranslationTable = TranslationTable<SINK_TRANSLATION_ENTRIES>;

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// Translation Table entries, in table order.
    pub fn translations(&self) -> &[TranslationEntry] {
        self.translations.entries()
    }

    /// Restore a Translation Table entry from persistent storage.
    pub fn restore_translation(&mut self, entry: TranslationEntry) -> Result<()> {
        self.translations.restore(entry)
    }

    /// Restore the paired endpoints of Sink Table entry `index` from
    /// persistent storage (they are not part of the Sink Table attribute).
    /// Returns whether the entry exists.
    pub fn restore_paired_endpoints(&mut self, index: usize, endpoints: PairedEndpoints) -> bool {
        match self.table.get_mut(index) {
            Some(entry) => {
                entry.paired_endpoints = endpoints;
                true
            }
            None => false,
        }
    }

    /// GP Translation Table Update, Request and Pairing Configuration.
    pub(super) fn on_configuration_command(
        &mut self,
        command: &ServerCommand<'_>,
    ) -> Result<(u8, Option<SinkOutcome>)> {
        match command.command_id {
            server_cmd::TRANSLATION_TABLE_UPDATE => {
                let update = TranslationTableUpdate::decode(command.payload)?;
                // Translations exist only for paired GPDs, so they are
                // persisted with, and removed with, the Sink Table entry.
                if self.find(&update.gpd_id).is_none() {
                    return Ok((zcl_status::NOT_FOUND, None));
                }
                let (status, changed) = self.translations.apply(&update);
                if changed {
                    self.dirty = true;
                }
                Ok((status, None))
            }
            server_cmd::TRANSLATION_TABLE_REQUEST => {
                let request = TranslationTableRequest::decode(command.payload)?;
                // gecko_sdk answers unicast requests only.
                if command.unicast {
                    self.queue(Out::TranslationTableResponse {
                        dst: command.src_nwk,
                        tsn: command.tsn,
                        start_index: request.start_index,
                    });
                }
                Ok((zcl_status::SUCCESS, None))
            }
            server_cmd::PAIRING_CONFIGURATION => {
                let pc = PairingConfiguration::decode(command.payload)?;
                Ok(self.on_pairing_configuration(command.src_nwk, &pc))
            }
            _ => Err(Error::Unsupported),
        }
    }

    fn on_pairing_configuration(
        &mut self,
        src_nwk: u16,
        pc: &PairingConfiguration<'_>,
    ) -> (u8, Option<SinkOutcome>) {
        use pairing_action::*;
        if src_nwk == self.own_nwk {
            // gecko_sdk drops its own looped-back command.
            return (zcl_status::SUCCESS, None);
        }
        if pc.gpd_id.is_reserved()
            || pc
                .security
                .is_some_and(|s| s.level == SecurityLevel::Reserved)
        {
            return (zcl_status::INVALID_FIELD, None);
        }
        let gpd = pc.gpd_id;
        match pc.action() {
            NO_ACTION => {
                if pc.send_gp_pairing() {
                    if self.find(&gpd).is_none() {
                        return (zcl_status::NOT_FOUND, None);
                    }
                    self.queue(Out::PairingAdd(gpd));
                }
                (zcl_status::SUCCESS, None)
            }
            REMOVE_GPD => {
                if self.translations.remove_gpd(&gpd) {
                    self.dirty = true;
                }
                let removed = if pc.send_gp_pairing() {
                    self.remove(&gpd)
                } else {
                    self.remove_quietly(&gpd)
                };
                (
                    zcl_status::SUCCESS,
                    removed.then_some(SinkOutcome::Decommissioned),
                )
            }
            REMOVE_PAIRING => self.remove_pairing(pc),
            EXTEND | REPLACE => self.configure_pairing(pc),
            APPLICATION_DESCRIPTION => self.configured_description(pc),
            _ => (zcl_status::INVALID_FIELD, None),
        }
    }

    /// Action 0b011: remove this sink's pairing when the communication mode
    /// (and, for commissioned groupcast, every listed group) matches.
    fn remove_pairing(&mut self, pc: &PairingConfiguration<'_>) -> (u8, Option<SinkOutcome>) {
        let gpd = pc.gpd_id;
        let Some(i) = self.find(&gpd) else {
            return (zcl_status::NOT_FOUND, None);
        };
        let e = &self.table[i];
        let mode = e.communication_mode;
        if mode != pc.communication_mode
            || (mode == CommunicationMode::CommissionedGroupcast
                && pc
                    .group_list()
                    .any(|(g, _)| !e.groups.iter().any(|s| s.group == g)))
        {
            return (zcl_status::NOT_FOUND, None);
        }
        if pc.send_gp_pairing() {
            let mut groups: Vec<Option<u16>, MAX_SINK_GROUPS> = Vec::new();
            match mode {
                CommunicationMode::CommissionedGroupcast => {
                    for g in &e.groups {
                        let _ = groups.push(Some(g.group));
                    }
                }
                CommunicationMode::DerivedGroupcast => {
                    let _ = groups.push(Some(derive_alias(&gpd)));
                }
                _ => {
                    let _ = groups.push(None);
                }
            }
            for group in groups {
                self.queue(Out::PairingRemoveSink {
                    gpd_id: gpd,
                    mode,
                    group,
                });
            }
        }
        if self.translations.remove_gpd(&gpd) {
            self.dirty = true;
        }
        self.remove_quietly(&gpd);
        (zcl_status::SUCCESS, Some(SinkOutcome::Decommissioned))
    }

    /// Actions 0b001 (Extend) and 0b010 (Replace).
    fn configure_pairing(&mut self, pc: &PairingConfiguration<'_>) -> (u8, Option<SinkOutcome>) {
        let policy = self.config.security_level;
        if policy & security_level_attr::INVOLVE_TC != 0
            || pc
                .application_information
                .is_some_and(|a| self.refuses_description(&a))
        {
            return (FAILURE, None);
        }
        let level = pc.security.map_or(SecurityLevel::None, |s| s.level);
        if policy & security_level_attr::MINIMAL_LEVEL_MASK > level as u8 {
            return (FAILURE, None);
        }
        if level.is_protected() && pc.security.is_some_and(|s| s.key_type == KeyType::NoKey) {
            return (zcl_status::INVALID_FIELD, None);
        }
        let paired = match pc.number_of_paired_endpoints {
            paired_endpoints::NONE => PairedEndpoints::None,
            paired_endpoints::RESERVED => return (zcl_status::INVALID_FIELD, None),
            paired_endpoints::SINK_DERIVES | paired_endpoints::ALL => PairedEndpoints::Default,
            _ => {
                if pc
                    .paired_endpoints
                    .iter()
                    .any(|ep| !(1..=0xF0).contains(ep))
                {
                    return (zcl_status::INVALID_FIELD, None);
                }
                match Vec::<u8, MAX_PAIRED_ENDPOINTS>::from_slice(pc.paired_endpoints) {
                    Ok(list) => PairedEndpoints::List(list),
                    Err(_) => return (INSUFFICIENT_SPACE, None),
                }
            }
        };
        let groupcast = pc.communication_mode == CommunicationMode::CommissionedGroupcast;
        if groupcast && pc.groups.is_empty() {
            return (zcl_status::INVALID_FIELD, None);
        }
        let extend = pc.action() == pairing_action::EXTEND;
        let existing = self.find(&pc.gpd_id).map(|i| self.table[i].clone());
        let mut entry = match existing {
            Some(ref e) if extend && e.communication_mode == pc.communication_mode => e.clone(),
            _ => SinkEntry::new(pc.gpd_id),
        };
        entry.communication_mode = pc.communication_mode;
        entry.device_id = pc.device_id;
        entry.mac_seq_cap = pc.mac_seq_cap;
        entry.rx_on_cap = pc.rx_on_cap;
        entry.fixed_location = pc.fixed_location;
        if pc.assigned_alias.is_some() || !extend {
            entry.assigned_alias = pc.assigned_alias;
        }
        entry.groupcast_radius = pc.groupcast_radius;
        if let Some(s) = pc.security {
            if s.level.is_protected() {
                entry.security_level = s.level;
                entry.key_type = s.key_type;
                entry.key = Some(s.key);
            } else {
                entry.security_level = SecurityLevel::None;
                entry.key_type = KeyType::NoKey;
                entry.key = None;
            }
        }
        if let Some(fc) = pc.frame_counter {
            entry.frame_counter = fc;
        }
        if !groupcast {
            entry.groups.clear();
        }
        for (group, alias) in pc.group_list() {
            if entry.groups.iter().any(|g| g.group == group) {
                continue;
            }
            let g = SinkGroup {
                group,
                alias: (alias != 0xFFFF).then_some(alias),
            };
            if entry.groups.push(g).is_err() {
                return (INSUFFICIENT_SPACE, None);
            }
        }
        entry.paired_endpoints = paired;
        if let Err(status) = self.store_configured(entry, existing.as_ref()) {
            return (status, None);
        }
        if !extend && self.translations.remove_gpd(&pc.gpd_id) {
            self.dirty = true;
        }
        if pc.send_gp_pairing() {
            self.queue(Out::PairingAdd(pc.gpd_id));
            if pc.communication_mode != CommunicationMode::LightweightUnicast {
                let alias = pc
                    .assigned_alias
                    .unwrap_or_else(|| derive_alias(&pc.gpd_id));
                self.queue(Out::DeviceAnnce(alias));
            }
        }
        self.event(SinkEvent::Commissioned {
            gpd_id: pc.gpd_id,
            device_id: pc.device_id,
        });
        self.configured_with_description(pc)
    }

    /// Insert or replace a configured entry: GPEP groups follow it, the
    /// counter is persisted exactly, a changed key forgets the duplicate
    /// filter's history.
    fn store_configured(
        &mut self,
        mut entry: SinkEntry,
        old: Option<&SinkEntry>,
    ) -> core::result::Result<(), u8> {
        entry.persisted_counter = entry.frame_counter;
        entry.counter_exact = self.config.exact_counter_burst != 0;
        let new_groups = entry.group_ids();
        let gpd = entry.gpd_id;
        let old_groups = match self.find(&gpd) {
            Some(i) => {
                if old.is_some_and(|o| o.key != entry.key) {
                    self.dup.forget(&gpd);
                }
                let old_groups = self.table[i].group_ids();
                self.table[i] = entry;
                old_groups
            }
            None => {
                if self.table.push(entry).is_err() {
                    return Err(INSUFFICIENT_SPACE);
                }
                Vec::new()
            }
        };
        self.dirty = true;
        for g in &new_groups {
            if !old_groups.contains(g) {
                let _ = self.group_changes.push_back(GroupChange::Join(*g));
            }
        }
        for &g in &old_groups {
            if !new_groups.contains(&g) {
                self.release_groups(&[g]);
            }
        }
        Ok(())
    }

    /// GP Pairing removing this sink (AddSink 0, RemoveGPD 0) for a group
    /// or, without one, for this node's unicast address.
    pub(super) fn pairing_remove_sink(
        &self,
        gpd_id: GpdId,
        mode: CommunicationMode,
        group: Option<u16>,
    ) -> Pairing {
        Pairing {
            gpd_id,
            add_sink: false,
            remove_gpd: false,
            communication_mode: mode,
            gpd_fixed: false,
            mac_seq_cap: false,
            security_level: SecurityLevel::None,
            key_type: KeyType::NoKey,
            sink: match group {
                Some(g) => PairingSink::Group(g),
                None => PairingSink::Unicast {
                    ieee: self.own_ieee,
                    nwk: self.own_nwk,
                },
            },
            device_id: None,
            frame_counter: None,
            key: None,
            assigned_alias: None,
            groupcast_radius: None,
        }
    }

    /// GP Translation Table Response payload for `start_index`.
    pub(super) fn translation_table_response(
        &self,
        start_index: u8,
        out: &mut [u8],
    ) -> Result<usize> {
        self.translations.response(start_index, out, MAX_TX_PAYLOAD)
    }

    /// Translate an accepted operational command of entry `i` with the
    /// Translation Table or the entry's paired endpoints. `None` leaves it to
    /// the built-in translation for the sink's endpoint.
    pub(super) fn translate_configured(
        &mut self,
        i: usize,
        rx: &Received<'_>,
    ) -> Option<SinkOutcome> {
        let mut explicit = false;
        let mut translated = false;
        let mut pass = false;
        for e in self.translations.matching(&rx.gpd, rx.command_id) {
            explicit = true;
            if e.endpoint == PASS_TO_APPLICATION {
                pass = true;
                continue;
            }
            let payload = match &e.payload {
                ZclPayload::Fixed(p) => Vec::from_slice(p).ok(),
                ZclPayload::CopyGpd => Vec::from_slice(rx.payload).ok(),
                ZclPayload::Derived => {
                    translation::translate(rx.command_id, rx.payload).map(|t| t.payload)
                }
            };
            let Some(payload) = payload else {
                self.stats.translations_dropped = self.stats.translations_dropped.wrapping_add(1);
                continue;
            };
            push_command(
                &mut self.commands,
                SinkCommand {
                    gpd_id: rx.gpd,
                    endpoint: e.endpoint,
                    translation: Translation {
                        cluster_id: e.cluster,
                        command_id: e.zcl_command,
                        payload,
                    },
                },
            );
            translated = true;
        }
        if !explicit {
            match &self.table[i].paired_endpoints {
                PairedEndpoints::Default => return None,
                PairedEndpoints::None => pass = true,
                PairedEndpoints::List(endpoints) => {
                    match translation::translate(rx.command_id, rx.payload) {
                        Some(t) => {
                            for &endpoint in endpoints {
                                push_command(
                                    &mut self.commands,
                                    SinkCommand {
                                        gpd_id: rx.gpd,
                                        endpoint,
                                        translation: t.clone(),
                                    },
                                );
                            }
                            translated = !endpoints.is_empty();
                        }
                        None => pass = true,
                    }
                }
            }
        }
        if pass || !translated {
            let device_id = self.table[i].device_id;
            match Vec::from_slice(rx.payload) {
                Ok(payload) => self.event(SinkEvent::GpdCommand {
                    gpd_id: rx.gpd,
                    device_id,
                    command_id: rx.command_id,
                    payload,
                }),
                Err(_) if !translated => return Some(SinkOutcome::Dropped(SinkDrop::Invalid)),
                Err(_) => {}
            }
        }
        Some(if translated {
            SinkOutcome::Translated
        } else {
            SinkOutcome::PassedThrough
        })
    }
}

fn push_command(
    commands: &mut heapless::Deque<SinkCommand, SINK_COMMAND_QUEUE_LEN>,
    command: SinkCommand,
) {
    if commands.is_full() {
        commands.pop_front();
    }
    let _ = commands.push_back(command);
}

#[cfg(test)]
mod tests;
