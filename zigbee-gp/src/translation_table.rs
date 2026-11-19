//! Per-GPD translation entries of a GP sink (feature `translation-table`):
//! the Translation Table written by GP Translation Table Update, read by GP
//! Translation Table Request, and consulted before the built-in default
//! translation ([`crate::translation`]).
//!
//! Behaviour follows gecko_sdk 4.5 `green-power-translation-table.c` where
//! it is interoperable and the GP 1.1 wording it quotes:
//!
//! * the Index field of an update is ignored (GP 1.1 sends 0xFF);
//! * Add appends the translations (an identical entry is not duplicated);
//! * Replace removes every entry of the same (GPD ID, GPD CommandID,
//!   endpoint, profile, cluster) quintuple and adds the supplied ones;
//! * Remove removes every entry of the supplied quintuples (removing
//!   nothing is not an error, as in gecko_sdk);
//! * every GPD command with explicit entries is translated by all of them,
//!   in table order (gecko_sdk
//!   `sli_zigbee_af_gp_forward_gpd_command_based_on_translation_table`).
//!
//! Differences, all explicit: an update is validated completely before it
//! changes anything (gecko_sdk applies translations one by one); additional
//! information blocks (generic switch contact masks, compact attribute
//! reporting) are refused with FAILURE; a fixed payload is limited to
//! [`MAX_TT_PAYLOAD`] bytes (INSUFFICIENT_SPACE).

use crate::config::{
    PASS_TO_APPLICATION, PAYLOAD_COPY_GPD, PAYLOAD_DERIVED, TranslationRecord,
    TranslationTableResponse, TranslationTableUpdate, TtPayload, tt_action,
};
use crate::sink::zcl_status;
use crate::types::{ApplicationId, GpdId};
use crate::{Error, Result};

/// Largest fixed Zigbee payload of a translation entry (Move to Colour
/// with options is 8 bytes).
pub const MAX_TT_PAYLOAD: usize = 8;
/// Largest encoded entry: IEEE GPD ID (9), GPD command, endpoint, profile,
/// cluster, Zigbee command, payload length and payload.
pub const MAX_TT_ENTRY_ENCODED_LEN: usize = 9 + 1 + 1 + 2 + 2 + 1 + 1 + MAX_TT_PAYLOAD;
/// Paired endpoints a Sink Table entry keeps from GP Pairing Configuration.
pub const MAX_PAIRED_ENDPOINTS: usize = 4;
/// ZCL FAILURE.
const FAILURE: u8 = 0x01;
/// ZCL INSUFFICIENT_SPACE.
const INSUFFICIENT_SPACE: u8 = 0x89;

/// Owned Zigbee payload of a translation entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ZclPayload {
    /// A fixed payload (empty: none).
    Fixed(heapless::Vec<u8, MAX_TT_PAYLOAD>),
    /// Derived from the GPD command as by the default translation.
    Derived,
    /// The GPD command payload.
    CopyGpd,
}

/// One Translation Table entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TranslationEntry {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// GPD CommandID.
    pub gpd_command: u8,
    /// Local endpoint ([`PASS_TO_APPLICATION`]: hand the GPD command to the
    /// application untranslated).
    pub endpoint: u8,
    /// Zigbee profile (kept and reported; the local endpoint's profile is
    /// used for delivery).
    pub profile: u16,
    /// ZCL cluster.
    pub cluster: u16,
    /// ZCL cluster-specific command (client → server).
    pub zcl_command: u8,
    /// Payload.
    pub payload: ZclPayload,
}

impl TranslationEntry {
    /// Validate a received translation for `gpd_id`; `Err` carries the ZCL
    /// status.
    pub fn from_record(gpd_id: GpdId, t: &TranslationRecord<'_>) -> core::result::Result<Self, u8> {
        if t.additional_info.is_some() {
            return Err(FAILURE);
        }
        if !(1..=0xF0).contains(&t.endpoint) && t.endpoint != PASS_TO_APPLICATION {
            return Err(zcl_status::INVALID_FIELD);
        }
        let payload = match t.payload {
            TtPayload::Fixed(p) => {
                ZclPayload::Fixed(heapless::Vec::from_slice(p).map_err(|_| INSUFFICIENT_SPACE)?)
            }
            TtPayload::Derived => ZclPayload::Derived,
            TtPayload::CopyGpd => ZclPayload::CopyGpd,
        };
        Ok(Self {
            gpd_id,
            gpd_command: t.gpd_command,
            endpoint: t.endpoint,
            profile: t.profile,
            cluster: t.cluster,
            zcl_command: t.zcl_command,
            payload,
        })
    }

    /// The entry as a wire record (index 0xFF, no additional information).
    pub fn record(&self) -> TranslationRecord<'_> {
        TranslationRecord {
            index: 0xFF,
            gpd_command: self.gpd_command,
            endpoint: self.endpoint,
            profile: self.profile,
            cluster: self.cluster,
            zcl_command: self.zcl_command,
            payload: match &self.payload {
                ZclPayload::Fixed(p) => TtPayload::Fixed(p),
                ZclPayload::Derived => TtPayload::Derived,
                ZclPayload::CopyGpd => TtPayload::CopyGpd,
            },
            additional_info: None,
        }
    }

    fn same_quintuple(&self, gpd_id: &GpdId, key: (u8, u8, u16, u16)) -> bool {
        self.gpd_id.matches(gpd_id)
            && (self.gpd_command, self.endpoint, self.profile, self.cluster) == key
    }

    /// Encode for storage: ApplicationID, then the GP Translation Table
    /// Response entry (GPD ID and translation).
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let (first, rest) = out.split_first_mut().ok_or(Error::BufferFull)?;
        *first = self.gpd_id.application_id() as u8;
        Ok(1 + TranslationTableResponse::encode_entry(&self.gpd_id, &self.record(), rest)?)
    }

    /// Decode an entry written by [`encode`](Self::encode), strictly.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let (&app, rest) = buf.split_first().ok_or(Error::Truncated)?;
        let mut header = [0u8; 5];
        header[1] = app;
        header[4] = 1;
        // Reuse the response decoder for one entry.
        let mut frame = [0u8; 5 + MAX_TT_ENTRY_ENCODED_LEN];
        let total = 5 + rest.len();
        if total > frame.len() {
            return Err(Error::BufferFull);
        }
        frame[..5].copy_from_slice(&header);
        frame[5..total].copy_from_slice(rest);
        let response = TranslationTableResponse::decode(&frame[..total])?;
        let (gpd_id, record) = response.entries().next().ok_or(Error::Truncated)?;
        Self::from_record(gpd_id, &record).map_err(|_| Error::Unsupported)
    }
}

/// Endpoints a Sink Table entry is paired to (GP Pairing Configuration
/// `NumberOfPairedEndpoints`), consulted when no explicit translation
/// exists.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum PairedEndpoints {
    /// The sink's application endpoint ([`crate::sink::SinkConfig::endpoint`];
    /// commissioning, and 0xFE "sink derives" / 0xFF "all").
    #[default]
    Default,
    /// No endpoint (0x00): GPD commands are only reported to the
    /// application.
    None,
    /// The listed endpoints.
    List(heapless::Vec<u8, MAX_PAIRED_ENDPOINTS>),
}

/// The Translation Table with at most `T` entries.
#[derive(Debug, Clone, Default)]
pub struct TranslationTable<const T: usize> {
    entries: heapless::Vec<TranslationEntry, T>,
}

impl<const T: usize> TranslationTable<T> {
    /// An empty table.
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    /// The entries, in table order.
    pub fn entries(&self) -> &[TranslationEntry] {
        &self.entries
    }

    /// Drop every entry.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Restore an entry from storage.
    pub fn restore(&mut self, entry: TranslationEntry) -> Result<()> {
        self.entries.push(entry).map_err(|_| Error::BufferFull)
    }

    /// Entries translating `gpd_command` of `gpd`.
    pub fn matching<'s>(
        &'s self,
        gpd: &'s GpdId,
        gpd_command: u8,
    ) -> impl Iterator<Item = &'s TranslationEntry> + 's {
        self.entries
            .iter()
            .filter(move |e| e.gpd_id.matches(gpd) && e.gpd_command == gpd_command)
    }

    /// Remove every entry of `gpd`; returns whether one was removed.
    pub fn remove_gpd(&mut self, gpd: &GpdId) -> bool {
        let before = self.entries.len();
        self.entries.retain(|e| !e.gpd_id.matches(gpd));
        self.entries.len() != before
    }

    /// Apply a GP Translation Table Update; returns the ZCL status and
    /// whether the table changed. Nothing changes unless the whole update
    /// is valid and fits.
    pub fn apply(&mut self, update: &TranslationTableUpdate<'_>) -> (u8, bool) {
        if update.gpd_id.is_reserved() {
            return (zcl_status::INVALID_FIELD, false);
        }
        if update.additional_info {
            return (FAILURE, false);
        }
        let mut new =
            heapless::Vec::<TranslationEntry, { crate::config::MAX_TRANSLATIONS_PER_UPDATE }>::new(
            );
        for t in update.translations() {
            match TranslationEntry::from_record(update.gpd_id, &t) {
                Ok(e) => {
                    let _ = new.push(e);
                }
                Err(status) => return (status, false),
            }
        }
        let gpd = update.gpd_id;
        let key = |e: &TranslationEntry| (e.gpd_command, e.endpoint, e.profile, e.cluster);
        let replaced = |e: &TranslationEntry| new.iter().any(|n| e.same_quintuple(&gpd, key(n)));
        match update.action {
            tt_action::ADD => {
                let mut added = 0;
                for (i, n) in new.iter().enumerate() {
                    let duplicate =
                        self.entries.contains(n) || new[..i].iter().any(|earlier| earlier == n);
                    if !duplicate {
                        added += 1;
                    }
                }
                if self.entries.len() + added > T {
                    return (INSUFFICIENT_SPACE, false);
                }
                let mut changed = false;
                for n in new {
                    if !self.entries.contains(&n) {
                        let _ = self.entries.push(n);
                        changed = true;
                    }
                }
                (zcl_status::SUCCESS, changed)
            }
            tt_action::REPLACE => {
                let kept = self.entries.iter().filter(|e| !replaced(e)).count();
                if kept + new.len() > T {
                    return (INSUFFICIENT_SPACE, false);
                }
                let unchanged = self
                    .entries
                    .iter()
                    .filter(|e| replaced(e))
                    .all(|e| new.contains(e))
                    && new.iter().all(|n| self.entries.contains(n));
                if unchanged {
                    return (zcl_status::SUCCESS, false);
                }
                self.entries.retain(|e| !replaced(e));
                for n in new {
                    if !self.entries.contains(&n) {
                        let _ = self.entries.push(n);
                    }
                }
                (zcl_status::SUCCESS, true)
            }
            tt_action::REMOVE => {
                let before = self.entries.len();
                self.entries.retain(|e| !replaced(e));
                (zcl_status::SUCCESS, self.entries.len() != before)
            }
            _ => (zcl_status::INVALID_FIELD, false),
        }
    }

    /// GP Translation Table Response payload for a request from
    /// `start_index` (gecko_sdk
    /// `emberAfGreenPowerClusterGpTranslationTableRequestCallback`): the
    /// entries from `start_index` with the ApplicationID of the first one,
    /// as many as fit `max_len`; NOT_FOUND with no entries when the index is
    /// past the table.
    pub fn response(&self, start_index: u8, out: &mut [u8], max_len: usize) -> Result<usize> {
        let start = usize::from(start_index);
        let total = self.entries.len().min(u8::MAX as usize) as u8;
        let Some(first) = self.entries.get(start) else {
            return TranslationTableResponse::encode_header(
                out,
                zcl_status::NOT_FOUND,
                ApplicationId::SrcId,
                total,
                0,
                0,
            );
        };
        let app = first.gpd_id.application_id();
        let limit = max_len.min(out.len());
        let mut len = TranslationTableResponse::encode_header(
            &mut out[..limit],
            zcl_status::SUCCESS,
            app,
            total,
            start_index,
            0,
        )?;
        let mut count = 0u8;
        for e in self.entries[start..]
            .iter()
            .take_while(|e| e.gpd_id.application_id() == app)
        {
            let mut tmp = [0u8; MAX_TT_ENTRY_ENCODED_LEN];
            let n = TranslationTableResponse::encode_entry(&e.gpd_id, &e.record(), &mut tmp)?;
            if len + n > limit {
                break;
            }
            out[len..len + n].copy_from_slice(&tmp[..n]);
            len += n;
            count += 1;
        }
        if count == 0 {
            return TranslationTableResponse::encode_header(
                out,
                INSUFFICIENT_SPACE,
                ApplicationId::SrcId,
                total,
                0,
                0,
            );
        }
        out[4] = count;
        Ok(len)
    }
}

/// The `ZigbeePayloadLength` byte of a payload (for logs and tools).
pub fn payload_length_byte(payload: &ZclPayload) -> u8 {
    match payload {
        ZclPayload::Fixed(p) => p.len() as u8,
        ZclPayload::Derived => PAYLOAD_DERIVED,
        ZclPayload::CopyGpd => PAYLOAD_COPY_GPD,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GPD: GpdId = GpdId::SrcId(0x7A6B_9879);

    fn update(action: u8, records: &[TranslationRecord<'_>], buf: &mut [u8]) -> usize {
        TranslationTableUpdate::encode(&GPD, action, records, buf).unwrap()
    }

    fn rec(
        cmd: u8,
        ep: u8,
        cluster: u16,
        zcl: u8,
        payload: TtPayload<'static>,
    ) -> TranslationRecord<'static> {
        TranslationRecord {
            index: 0xFF,
            gpd_command: cmd,
            endpoint: ep,
            profile: 0x0104,
            cluster,
            zcl_command: zcl,
            payload,
            additional_info: None,
        }
    }

    fn apply<const T: usize>(
        t: &mut TranslationTable<T>,
        action: u8,
        r: &[TranslationRecord<'_>],
    ) -> (u8, bool) {
        let mut buf = [0u8; 128];
        let n = update(action, r, &mut buf);
        t.apply(&TranslationTableUpdate::decode(&buf[..n]).unwrap())
    }

    #[test]
    fn add_replace_remove_follow_the_quintuple() {
        let mut t = TranslationTable::<4>::new();
        let toggle_ep2 = rec(0x22, 2, 0x0006, 0x02, TtPayload::Fixed(&[]));
        assert_eq!(apply(&mut t, tt_action::ADD, &[toggle_ep2]), (0, true));
        // An identical Add changes nothing.
        assert_eq!(apply(&mut t, tt_action::ADD, &[toggle_ep2]), (0, false));
        let on_ep1 = rec(0x21, 1, 0x0006, 0x01, TtPayload::Fixed(&[]));
        assert_eq!(apply(&mut t, tt_action::ADD, &[on_ep1]), (0, true));
        assert_eq!(t.matching(&GPD, 0x22).count(), 1);
        // Replace the Toggle → endpoint 2 On/Off translation by Off.
        let off_ep2 = rec(0x22, 2, 0x0006, 0x00, TtPayload::Fixed(&[]));
        assert_eq!(apply(&mut t, tt_action::REPLACE, &[off_ep2]), (0, true));
        let m: heapless::Vec<u8, 4> = t.matching(&GPD, 0x22).map(|e| e.zcl_command).collect();
        assert_eq!(&m[..], &[0x00]);
        // Replace with no match adds.
        let level = rec(0x22, 3, 0x0008, 0x04, TtPayload::Fixed(&[0x80, 0x0A, 0x00]));
        assert_eq!(apply(&mut t, tt_action::REPLACE, &[level]), (0, true));
        assert_eq!(t.matching(&GPD, 0x22).count(), 2);
        // Remove by quintuple; the command and payload do not matter.
        let any = rec(0x22, 2, 0x0006, 0x55, TtPayload::CopyGpd);
        assert_eq!(apply(&mut t, tt_action::REMOVE, &[any]), (0, true));
        assert_eq!(apply(&mut t, tt_action::REMOVE, &[any]), (0, false));
        assert_eq!(t.entries().len(), 2);
        assert!(t.remove_gpd(&GPD));
        assert!(t.entries().is_empty());
    }

    #[test]
    fn invalid_or_oversized_updates_change_nothing() {
        let mut t = TranslationTable::<2>::new();
        let a = rec(0x20, 1, 0x0006, 0x00, TtPayload::Fixed(&[]));
        let b = rec(0x21, 1, 0x0006, 0x01, TtPayload::Fixed(&[]));
        let c = rec(0x22, 1, 0x0006, 0x02, TtPayload::Fixed(&[]));
        assert_eq!(
            apply(&mut t, tt_action::ADD, &[a, b, c]),
            (INSUFFICIENT_SPACE, false)
        );
        assert!(t.entries().is_empty());
        let bad_ep = rec(0x20, 0, 0x0006, 0x00, TtPayload::Fixed(&[]));
        assert_eq!(
            apply(&mut t, tt_action::ADD, &[a, bad_ep]),
            (zcl_status::INVALID_FIELD, false)
        );
        let long = rec(0x20, 1, 0x0300, 0x07, TtPayload::Fixed(&[0; 9]));
        assert_eq!(
            apply(&mut t, tt_action::ADD, &[long]),
            (INSUFFICIENT_SPACE, false)
        );
        assert!(t.entries().is_empty());
        let mut info = a;
        info.additional_info = Some(&[0x01]);
        assert_eq!(apply(&mut t, tt_action::ADD, &[info]), (FAILURE, false));
        let mut buf = [0u8; 64];
        let n = TranslationTableUpdate::encode(&GpdId::SrcId(0), tt_action::ADD, &[a], &mut buf)
            .unwrap();
        assert_eq!(
            t.apply(&TranslationTableUpdate::decode(&buf[..n]).unwrap()),
            (zcl_status::INVALID_FIELD, false)
        );
    }

    #[test]
    fn response_pages_and_entries_round_trip_through_storage() {
        let mut t = TranslationTable::<4>::new();
        let r = [
            rec(0x20, 1, 0x0006, 0x00, TtPayload::Fixed(&[])),
            rec(0x22, 2, 0x0008, 0x04, TtPayload::Fixed(&[0x80, 0x0A, 0x00])),
            rec(0x32, 1, 0x0008, 0x02, TtPayload::Derived),
        ];
        assert_eq!(apply(&mut t, tt_action::ADD, &r), (0, true));
        let mut out = [0u8; 64];
        let n = t.response(1, &mut out, 64).unwrap();
        let resp = TranslationTableResponse::decode(&out[..n]).unwrap();
        assert_eq!(
            (resp.status, resp.total, resp.start_index, resp.len()),
            (0, 3, 1, 2)
        );
        let mut got = resp.entries().map(|(_, r)| r);
        assert_eq!(got.next(), Some(r[1]));
        assert_eq!(got.next(), Some(r[2]));
        assert_eq!(got.next(), None);
        drop(got);
        // A small budget pages: one entry.
        let n = t.response(0, &mut out, 5 + 13).unwrap();
        assert_eq!(
            TranslationTableResponse::decode(&out[..n]).unwrap().len(),
            1
        );
        // Past the end.
        let n = t.response(3, &mut out, 64).unwrap();
        assert_eq!(&out[..n], &[zcl_status::NOT_FOUND, 0, 3, 0, 0]);
        for e in t.entries() {
            let mut buf = [0u8; 1 + MAX_TT_ENTRY_ENCODED_LEN];
            let n = e.encode(&mut buf).unwrap();
            assert_eq!(&TranslationEntry::decode(&buf[..n]).unwrap(), e);
        }
        assert!(TranslationEntry::decode(&[0x00, 1, 2]).is_err());
    }
}
