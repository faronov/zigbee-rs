//! Bounded keep-awake reasons and the compact, persistable anomaly record.
//!
//! A sleepy sensor stays awake (radio on, fast poll or no sleep at all) only
//! while some protocol reason asks for it. Every such reason is bounded by a
//! product-configured time limit ([`crate::RecoveryPolicy`]); when a limit
//! fires the shared lifecycle forces the reason to stop holding the chip
//! awake and records it in an [`AnomalyRecord`].
//!
//! The record is deliberately tiny and platform independent: a product that
//! has no console can persist it (see [`AnomalyRecord::encode`]) on the rare
//! occasions it changes and expose it as readable ZCL attributes
//! ([`AnomalyRecord::summary_word`], [`AnomalyRecord::counts_word`] and
//! [`AnomalyRecord::last_cause`]).

/// Why the shared lifecycle intervened.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnomalyReason {
    /// Pending protocol work (APS ACK wait, queued responses, key exchange,
    /// scheduled rejoin) outlived `max_active_ms`.
    PendingWork = 1,
    /// An OTA session kept the chip awake longer than `max_ota_awake_ms`
    /// and was aborted.
    Ota = 2,
    /// Identify kept the fast poll longer than `max_identify_awake_ms`.
    Identify = 3,
    /// Consecutive timed fast-poll windows (commissioning grace, Poll
    /// Control fast poll, control events) outlived `max_fast_poll_ms`.
    FastPoll = 4,
    /// No acknowledged report within the shortest configured reporting
    /// maximum interval plus `missed_report_grace_ms`.
    MissedReport = 5,
    /// Parent-loss evidence reached `parent_loss_strikes`; a rejoin started.
    ParentLoss = 6,
    /// The previous boot ended in an unplanned reset; the product carried
    /// its cause ([`crate::ResetCause`]) into
    /// [`AnomalyRecord::last_cause`].
    Reset = 7,
    /// Low-power waits were disabled for the rest of the boot: either
    /// `sleep_failure_limit` consecutive sleep transitions failed (the last
    /// wait error is in [`AnomalyRecord::last_cause`]), or the product's
    /// rapid-reset guard tripped (see [`crate::reset_cause`]).
    SleepDisabled = 8,
}

impl AnomalyReason {
    /// Number of distinct reasons (codes `1..=COUNT`). At most 8, so the
    /// seen mask fits the summary's high byte.
    pub const COUNT: usize = 8;

    pub const fn code(self) -> u8 {
        self as u8
    }

    pub const fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            1 => Self::PendingWork,
            2 => Self::Ota,
            3 => Self::Identify,
            4 => Self::FastPoll,
            5 => Self::MissedReport,
            6 => Self::ParentLoss,
            7 => Self::Reset,
            8 => Self::SleepDisabled,
            _ => return None,
        })
    }

    const fn index(self) -> usize {
        self as usize - 1
    }
}

/// One step of an [`AwakeBound`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundStep {
    /// The reason is not asking to keep the chip awake.
    Idle,
    /// The reason may keep the chip awake.
    Allowed,
    /// The reason is still asserted but its limit already fired; it must no
    /// longer keep the chip awake until it clears.
    Suppressed,
    /// The limit fired for the first time since the reason was asserted.
    /// Treated as [`Suppressed`](Self::Suppressed).
    Fired,
    /// The reason persisted for a further whole limit period after it
    /// fired. Lets a persisting reason keep earning evidence without being
    /// recorded as a new anomaly. Treated as
    /// [`Suppressed`](Self::Suppressed).
    Refired,
}

impl BoundStep {
    /// Whether the reason may keep the chip awake this iteration.
    pub const fn allows(self) -> bool {
        matches!(self, Self::Allowed)
    }
}

/// Time budget of one keep-awake reason.
///
/// Time is accumulated from per-iteration deltas rather than compared
/// against a long-lived start mark, so the budget stays correct on
/// platforms whose native counter wraps within minutes.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct AwakeBound {
    elapsed_ms: u32,
    running: bool,
    expired: bool,
}

impl AwakeBound {
    /// Advance by `delta_ms` of measured time with the reason `active` or
    /// not. The first iteration a reason becomes active only starts the
    /// budget, so time spent before it asked to stay awake is not charged.
    /// `limit_ms == 0` leaves the reason unbounded.
    pub fn step(&mut self, active: bool, delta_ms: u32, limit_ms: u32) -> BoundStep {
        if !active {
            *self = Self::default();
            return BoundStep::Idle;
        }
        if limit_ms == 0 {
            return BoundStep::Allowed;
        }
        if self.running {
            self.elapsed_ms = self.elapsed_ms.saturating_add(delta_ms);
        } else {
            self.running = true;
        }
        if self.elapsed_ms < limit_ms {
            return if self.expired {
                BoundStep::Suppressed
            } else {
                BoundStep::Allowed
            };
        }
        self.elapsed_ms = 0;
        if core::mem::replace(&mut self.expired, true) {
            BoundStep::Refired
        } else {
            BoundStep::Fired
        }
    }

    /// Forget the budget, e.g. after a rejoin.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Saturating anomaly counters: the last reason, a mask of every reason
/// seen, a total, one count per reason and the cause word of the latest
/// reset or sleep-disable anomaly.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct AnomalyRecord {
    pub total: u16,
    pub last: Option<AnomalyReason>,
    pub counts: [u8; AnomalyReason::COUNT],
    /// [`crate::ResetCause::word`] of the latest [`AnomalyReason::Reset`] or
    /// [`AnomalyReason::SleepDisabled`] (`0` = none recorded).
    pub last_cause: u32,
}

/// Fixed size of one encoded record (a whole number of 4-byte flash words).
pub const ANOMALY_RECORD_LEN: usize = 32;
const ANOMALY_MAGIC: [u8; 2] = [0xA7, 0x5E];
/// Version 2: 32-byte slot with eight reasons and the cause word. Version 1
/// (16-byte) slots decode as invalid; the journal then starts afresh.
const ANOMALY_VERSION: u8 = 2;
const COUNTS_AT: usize = 6;
const CAUSE_AT: usize = COUNTS_AT + AnomalyReason::COUNT;

impl AnomalyRecord {
    pub const EMPTY: Self = Self {
        total: 0,
        last: None,
        counts: [0; AnomalyReason::COUNT],
        last_cause: 0,
    };

    pub fn note(&mut self, reason: AnomalyReason) {
        self.total = self.total.saturating_add(1);
        self.last = Some(reason);
        let count = &mut self.counts[reason.index()];
        *count = count.saturating_add(1);
    }

    /// [`note`](Self::note) `reason` and remember `cause` (a
    /// [`crate::ResetCause::word`]) as [`last_cause`](Self::last_cause).
    pub fn note_with_cause(&mut self, reason: AnomalyReason, cause: u32) {
        self.note(reason);
        self.last_cause = cause;
    }

    /// Note an unplanned reset carried over from the previous boot, unless
    /// it repeats the newest entry (same reason and cause). A reset loop
    /// therefore changes the record, and costs a flash write, once per
    /// distinct cause rather than once per boot. Returns whether the record
    /// changed.
    pub fn note_reset(&mut self, cause: u32) -> bool {
        if self.last == Some(AnomalyReason::Reset) && self.last_cause == cause {
            return false;
        }
        self.note_with_cause(AnomalyReason::Reset, cause);
        true
    }

    pub const fn count(&self, reason: AnomalyReason) -> u8 {
        self.counts[reason.index()]
    }

    /// Bit `code - 1` set for every reason recorded at least once.
    pub fn seen_mask(&self) -> u8 {
        let mut mask = 0u8;
        for (index, count) in self.counts.iter().enumerate() {
            if *count != 0 {
                mask |= 1 << index;
            }
        }
        mask
    }

    /// 16-bit attribute form: bits 0–7 last reason code (0 = none), bits
    /// 8–15 [`seen_mask`](Self::seen_mask).
    pub fn summary_word(&self) -> u16 {
        u16::from(self.last.map_or(0, AnomalyReason::code)) | u16::from(self.seen_mask()) << 8
    }

    /// Four per-reason counts packed little-endian into one 32-bit
    /// attribute value: `group` 0 covers codes 1–4 (code 1 in bits 0–7),
    /// `group` 1 codes 5–8.
    pub fn counts_word(&self, group: usize) -> u32 {
        let mut bytes = [0u8; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.counts.get(group * 4 + index).copied().unwrap_or(0);
        }
        u32::from_le_bytes(bytes)
    }

    /// Encode as one self-checking [`ANOMALY_RECORD_LEN`]-byte slot.
    pub fn encode(&self) -> [u8; ANOMALY_RECORD_LEN] {
        let mut slot = [0u8; ANOMALY_RECORD_LEN];
        slot[..2].copy_from_slice(&ANOMALY_MAGIC);
        slot[2] = ANOMALY_VERSION;
        slot[3] = self.last.map_or(0, AnomalyReason::code);
        slot[4..6].copy_from_slice(&self.total.to_le_bytes());
        slot[COUNTS_AT..CAUSE_AT].copy_from_slice(&self.counts);
        slot[CAUSE_AT..CAUSE_AT + 4].copy_from_slice(&self.last_cause.to_le_bytes());
        let crc = crc16(&slot[..ANOMALY_RECORD_LEN - 2]);
        slot[ANOMALY_RECORD_LEN - 2..].copy_from_slice(&crc.to_le_bytes());
        slot
    }

    /// Decode a slot written by [`encode`](Self::encode). An erased slot,
    /// a torn write or an unknown version yields `None`.
    pub fn decode(slot: &[u8]) -> Option<Self> {
        if slot.len() != ANOMALY_RECORD_LEN
            || slot[..2] != ANOMALY_MAGIC
            || slot[2] != ANOMALY_VERSION
        {
            return None;
        }
        let crc = u16::from_le_bytes([slot[ANOMALY_RECORD_LEN - 2], slot[ANOMALY_RECORD_LEN - 1]]);
        if crc != crc16(&slot[..ANOMALY_RECORD_LEN - 2]) {
            return None;
        }
        let last = match slot[3] {
            0 => None,
            code => Some(AnomalyReason::from_code(code)?),
        };
        let mut counts = [0u8; AnomalyReason::COUNT];
        counts.copy_from_slice(&slot[COUNTS_AT..CAUSE_AT]);
        let mut cause = [0u8; 4];
        cause.copy_from_slice(&slot[CAUSE_AT..CAUSE_AT + 4]);
        Some(Self {
            total: u16::from_le_bytes([slot[4], slot[5]]),
            last,
            counts,
            last_cause: u32::from_le_bytes(cause),
        })
    }
}

/// Append-only flash journal of [`AnomalyRecord`] slots.
///
/// Written only when the record changed, and at most `max_writes` times per
/// boot, so normal operation never touches flash and a fault storm cannot
/// wear it out. Slots are appended across the window; reaching an erase
/// block boundary erases only that block, so the newest record of the
/// previous block survives a power loss during the erase. The newest record
/// is the valid slot with the highest `total` (the counter never goes
/// down), so torn or erased slots are simply skipped.
pub struct AnomalyJournal<F> {
    flash: F,
    next: u32,
    writes_left: u8,
}

impl<F: embedded_storage::nor_flash::NorFlash> AnomalyJournal<F> {
    const SLOT: u32 = ANOMALY_RECORD_LEN as u32;
    const _ALIGNED: () = assert!(
        ANOMALY_RECORD_LEN.is_multiple_of(F::WRITE_SIZE)
            && ANOMALY_RECORD_LEN.is_multiple_of(F::READ_SIZE)
            && F::ERASE_SIZE.is_multiple_of(ANOMALY_RECORD_LEN)
    );

    pub fn new(flash: F, max_writes: u8) -> Self {
        let () = Self::_ALIGNED;
        Self {
            flash,
            next: 0,
            writes_left: max_writes,
        }
    }

    fn slots(&self) -> u32 {
        (self.flash.capacity() as u32) / Self::SLOT
    }

    /// Scan the window for the newest valid record (`EMPTY` when there is
    /// none) and position the append cursor after it.
    pub fn load(&mut self) -> Result<AnomalyRecord, F::Error> {
        let mut newest: Option<(u32, AnomalyRecord)> = None;
        let mut slot = [0u8; ANOMALY_RECORD_LEN];
        for index in 0..self.slots() {
            self.flash.read(index * Self::SLOT, &mut slot)?;
            if let Some(record) = AnomalyRecord::decode(&slot)
                && newest.is_none_or(|(_, best)| record.total >= best.total)
            {
                newest = Some((index, record));
            }
        }
        self.next = newest.map_or(0, |(index, _)| (index + 1) % self.slots());
        Ok(newest.map_or(AnomalyRecord::EMPTY, |(_, record)| record))
    }

    /// Append `record`. Returns `Ok(false)` once this boot's write budget is
    /// spent.
    pub fn store(&mut self, record: &AnomalyRecord) -> Result<bool, F::Error> {
        if self.writes_left == 0 {
            return Ok(false);
        }
        self.writes_left -= 1;
        let erase = F::ERASE_SIZE as u32;
        let slots = self.slots();
        let mut probe = [0u8; ANOMALY_RECORD_LEN];
        // Skip slots that are not erased (a torn write) up to the next
        // erase block, which is then erased.
        loop {
            let offset = self.next * Self::SLOT;
            if offset.is_multiple_of(erase) {
                self.flash.erase(offset, offset + erase)?;
                break;
            }
            self.flash.read(offset, &mut probe)?;
            if probe.iter().all(|byte| *byte == 0xFF) {
                break;
            }
            self.next = (self.next + 1) % slots;
        }
        let offset = self.next * Self::SLOT;
        self.flash.write(offset, &record.encode())?;
        self.next = (self.next + 1) % slots;
        Ok(true)
    }
}

/// CRC-16/CCITT-FALSE.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in data {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bound_allows_until_limit_then_fires_once_per_period() {
        let mut bound = AwakeBound::default();
        // First active iteration only starts the budget.
        assert_eq!(bound.step(true, 50_000, 1_000), BoundStep::Allowed);
        assert_eq!(bound.step(true, 600, 1_000), BoundStep::Allowed);
        assert_eq!(bound.step(true, 400, 1_000), BoundStep::Fired);
        assert_eq!(bound.step(true, 999, 1_000), BoundStep::Suppressed);
        assert_eq!(bound.step(true, 1, 1_000), BoundStep::Refired);
        // Clearing the reason restores the full budget.
        assert_eq!(bound.step(false, 10, 1_000), BoundStep::Idle);
        assert_eq!(bound.step(true, 10, 1_000), BoundStep::Allowed);
    }

    #[test]
    fn zero_limit_is_unbounded() {
        let mut bound = AwakeBound::default();
        for _ in 0..10 {
            assert_eq!(bound.step(true, u32::MAX, 0), BoundStep::Allowed);
        }
    }

    #[test]
    fn bound_saturates_instead_of_wrapping() {
        let mut bound = AwakeBound::default();
        bound.step(true, 0, u32::MAX);
        assert_eq!(bound.step(true, u32::MAX, u32::MAX), BoundStep::Fired);
    }

    #[test]
    fn record_counts_saturate_and_summarise() {
        let mut record = AnomalyRecord::EMPTY;
        assert_eq!(record.summary_word(), 0);
        record.note(AnomalyReason::Ota);
        for _ in 0..300 {
            record.note(AnomalyReason::MissedReport);
        }
        assert_eq!(record.total, 301);
        assert_eq!(record.count(AnomalyReason::Ota), 1);
        assert_eq!(record.count(AnomalyReason::MissedReport), u8::MAX);
        assert_eq!(record.seen_mask(), 0b1_0010);
        // A uint16 attribute: last reason in the low byte, mask above it.
        assert_eq!(record.summary_word(), 5 | 0b1_0010 << 8);
        assert_eq!(record.counts_word(0), 1 << 8);
        assert_eq!(record.counts_word(1), 0xFF);
        record.note_with_cause(AnomalyReason::SleepDisabled, 0x1234_5678);
        assert_eq!(record.summary_word(), 8 | 0b1001_0010 << 8);
        assert_eq!(record.counts_word(1), 0xFF | 1 << 24);
        assert_eq!(record.last_cause, 0x1234_5678);
    }

    #[test]
    fn repeated_reset_cause_is_noted_once_per_change() {
        let mut record = AnomalyRecord::EMPTY;
        assert!(record.note_reset(0x1000_0042));
        // The same loop iterating again does not change the record.
        for _ in 0..100 {
            assert!(!record.note_reset(0x1000_0042));
        }
        assert_eq!(record.count(AnomalyReason::Reset), 1);
        // A different cause, or an unrelated anomaly in between, does.
        assert!(record.note_reset(0x2000_0001));
        record.note(AnomalyReason::MissedReport);
        assert!(record.note_reset(0x2000_0001));
        assert_eq!(record.count(AnomalyReason::Reset), 3);
        assert_eq!(record.total, 4);
    }

    #[test]
    fn record_round_trips_and_rejects_damage() {
        let mut record = AnomalyRecord::EMPTY;
        record.note(AnomalyReason::PendingWork);
        record.note(AnomalyReason::ParentLoss);
        record.note_with_cause(AnomalyReason::Reset, 0xDEAD_BEEF);
        let slot = record.encode();
        assert_eq!(AnomalyRecord::decode(&slot), Some(record));
        assert_eq!(
            AnomalyRecord::decode(&[0xFF; ANOMALY_RECORD_LEN]),
            None,
            "erased flash"
        );
        for index in 0..ANOMALY_RECORD_LEN {
            let mut torn = slot;
            torn[index] ^= 0x10;
            assert_eq!(AnomalyRecord::decode(&torn), None, "byte {index}");
        }
        assert_eq!(AnomalyRecord::decode(&slot[..8]), None);
        assert_eq!(
            AnomalyRecord::decode(&AnomalyRecord::EMPTY.encode()),
            Some(AnomalyRecord::EMPTY)
        );
    }

    #[test]
    fn reason_codes_are_stable() {
        for code in 1..=AnomalyReason::COUNT as u8 {
            assert_eq!(AnomalyReason::from_code(code).unwrap().code(), code);
        }
        assert_eq!(AnomalyReason::from_code(0), None);
        assert_eq!(AnomalyReason::from_code(9), None);
    }

    #[test]
    fn version_1_slots_are_not_misread() {
        // A 16-byte v1 slot followed by erased flash.
        let mut slot = [0xFFu8; ANOMALY_RECORD_LEN];
        slot[..16].copy_from_slice(&[
            0xA7, 0x5E, 1, 5, 1, 0, 0, 0, 0, 0, 1, 0, 0x12, 0x34, 0x56, 0x78,
        ]);
        assert_eq!(AnomalyRecord::decode(&slot), None);
    }

    /// 4 × 64-byte erase blocks of NOR flash in RAM, counting operations.
    struct RamFlash {
        bytes: [u8; 256],
        erases: u32,
        writes: u32,
        /// Bytes of the next write that land before a simulated power loss.
        tear_next_write: Option<usize>,
    }

    impl RamFlash {
        fn new() -> Self {
            Self {
                bytes: [0xFF; 256],
                erases: 0,
                writes: 0,
                tear_next_write: None,
            }
        }
    }

    impl embedded_storage::nor_flash::ErrorType for RamFlash {
        type Error = core::convert::Infallible;
    }

    impl embedded_storage::nor_flash::ReadNorFlash for RamFlash {
        const READ_SIZE: usize = 4;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            bytes.copy_from_slice(&self.bytes[start..start + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl embedded_storage::nor_flash::NorFlash for RamFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = 64;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            assert!(from.is_multiple_of(64) && to.is_multiple_of(64));
            self.bytes[from as usize..to as usize].fill(0xFF);
            self.erases += 1;
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let len = self.tear_next_write.take().unwrap_or(bytes.len());
            for (cell, byte) in self.bytes[start..start + len].iter_mut().zip(bytes) {
                assert_eq!(*cell, 0xFF, "NOR write over unerased bytes");
                *cell = *byte;
            }
            self.writes += 1;
            Ok(())
        }
    }

    fn noted(reasons: &[AnomalyReason]) -> AnomalyRecord {
        let mut record = AnomalyRecord::EMPTY;
        for reason in reasons {
            record.note(*reason);
        }
        record
    }

    #[test]
    fn erased_journal_loads_empty_without_writing() {
        let mut journal = AnomalyJournal::new(RamFlash::new(), 8);
        assert_eq!(journal.load(), Ok(AnomalyRecord::EMPTY));
        assert_eq!((journal.flash.erases, journal.flash.writes), (0, 0));
    }

    #[test]
    fn journal_returns_the_newest_record_across_reboots_and_wrap() {
        let mut flash = RamFlash::new();
        let mut record = AnomalyRecord::EMPTY;
        // 8 slots in the window; 20 records wrap it twice over 5 boots.
        for _boot in 0..5 {
            let mut journal = AnomalyJournal::new(flash, 4);
            assert_eq!(journal.load(), Ok(record));
            for _ in 0..4 {
                record.note(AnomalyReason::Ota);
                assert_eq!(journal.store(&record), Ok(true));
            }
            flash = journal.flash;
        }
        let mut journal = AnomalyJournal::new(flash, 0);
        assert_eq!(journal.load().unwrap().total, 20);
        // Only block boundaries were erased (one erase per 2 slots).
        assert_eq!(journal.flash.erases, 10);
    }

    #[test]
    fn write_budget_per_boot_bounds_flash_wear() {
        let mut journal = AnomalyJournal::new(RamFlash::new(), 2);
        let _ = journal.load();
        let mut record = AnomalyRecord::EMPTY;
        let mut stored = 0;
        for _ in 0..50 {
            record.note(AnomalyReason::PendingWork);
            if journal.store(&record) == Ok(true) {
                stored += 1;
            }
        }
        assert_eq!(stored, 2);
        assert_eq!(journal.flash.writes, 2);
    }

    #[test]
    fn torn_write_keeps_the_previous_record_and_is_skipped() {
        let mut journal = AnomalyJournal::new(RamFlash::new(), 8);
        let _ = journal.load();
        let first = noted(&[AnomalyReason::Identify]);
        journal.store(&first).unwrap();
        journal.flash.tear_next_write = Some(12);
        journal
            .store(&noted(&[AnomalyReason::Identify, AnomalyReason::Ota]))
            .unwrap();
        let mut rebooted = AnomalyJournal::new(journal.flash, 8);
        assert_eq!(rebooted.load(), Ok(first));
        let third = noted(&[
            AnomalyReason::Identify,
            AnomalyReason::Ota,
            AnomalyReason::FastPoll,
        ]);
        rebooted.store(&third).unwrap();
        let mut again = AnomalyJournal::new(rebooted.flash, 8);
        assert_eq!(again.load(), Ok(third));
    }
}
