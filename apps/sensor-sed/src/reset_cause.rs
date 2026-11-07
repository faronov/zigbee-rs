//! Why the previous boot ended, carried across a reset in retained RAM.
//!
//! A battery sensor without a console resets itself on an unrecoverable
//! fault (a radio that cannot be restored after sleep, a panic). Before such a
//! reset the product writes a [`ResetCause`] into a few words of RAM that
//! survive a reset but not a power cycle (for example ESP RTC fast memory).
//! At the next boot [`RetainedBoot::boot`] consumes it; the product notes it
//! in the persistent [`crate::AnomalyRecord`] (reason
//! [`crate::AnomalyReason::Reset`]) so it remains readable after the device
//! is power-cycled onto a debug supply.
//!
//! The same words count boots that ended before a stability window. When
//! `limit` such quick resets happen in a row, [`BootRecord::guard_tripped`]
//! tells the product to disable low-power waits for that boot (reason
//! [`crate::AnomalyReason::SleepDisabled`]), so a sleep-related reset loop
//! settles into an awake, working device instead of rebooting forever.
//!
//! Everything here is platform independent and host tested; the product only
//! supplies the retained storage and its classification of the hardware
//! reset reason.

/// Encoded reset or failure cause: kind in bits 28–31, payload in 0–27.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetCause(u32);

impl ResetCause {
    /// No cause was recorded.
    pub const KIND_NONE: u8 = 0;
    /// A failed low-power wait; payload is the platform's
    /// [`WakeController::error_code`](crate::WakeController::error_code).
    pub const KIND_WAKE: u8 = 1;
    /// A panic; payload is a platform-defined code offset of the panic
    /// site, 0 if unknown (see [`panic_site`](Self::panic_site)).
    pub const KIND_PANIC: u8 = 2;
    /// A profile/cluster configuration failure restarted the node.
    pub const KIND_PROFILE: u8 = 3;
    /// No software cause was recorded, but the hardware reports an unplanned
    /// reset (watchdog, brownout, clock glitch); payload is the chip's raw
    /// reset-reason code.
    pub const KIND_HARDWARE: u8 = 4;

    const PAYLOAD_MASK: u32 = (1 << 28) - 1;

    pub const fn new(kind: u8, payload: u32) -> Self {
        Self(((kind as u32 & 0xF) << 28) | (payload & Self::PAYLOAD_MASK))
    }

    pub const fn wake(code: u32) -> Self {
        Self::new(Self::KIND_WAKE, code)
    }

    /// A panic near code offset `offset` (relative to a platform-defined
    /// text base, resolved with the image's ELF; 0 = unknown).
    ///
    /// A source location is deliberately not used: reading
    /// `PanicInfo::location` keeps every panic site's `Location` and file
    /// path string in the image (about 35 KB on the ESP32-H2 thermometer).
    pub const fn panic_site(offset: u32) -> Self {
        Self::new(Self::KIND_PANIC, offset)
    }

    pub const fn profile() -> Self {
        Self::new(Self::KIND_PROFILE, 0)
    }

    pub const fn hardware(reset_reason: u32) -> Self {
        Self::new(Self::KIND_HARDWARE, reset_reason)
    }

    pub const fn from_word(word: u32) -> Option<Self> {
        if word == 0 { None } else { Some(Self(word)) }
    }

    pub const fn word(self) -> u32 {
        self.0
    }

    pub const fn kind(self) -> u8 {
        (self.0 >> 28) as u8
    }

    pub const fn payload(self) -> u32 {
        self.0 & Self::PAYLOAD_MASK
    }
}

/// How the chip says it came out of reset, classified by the product.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetKind {
    /// Power-on (or anything else that invalidates retained RAM): forget all
    /// retained state.
    PowerOn,
    /// A software or debug-host reset. Carries an anomaly only when the
    /// firmware recorded a cause first; planned resets (OTA activation,
    /// factory reset, rollback, flashing) record none.
    Software,
    /// A watchdog, brownout or other unplanned hardware reset, with the
    /// chip's raw reset-reason code.
    Unplanned(u32),
}

/// Rapid-reset guard limits (product configuration).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RapidResetPolicy {
    /// Consecutive boots ending before `window_ms` of uptime that disable
    /// low-power waits for the next boot. `0` disables the guard.
    pub limit: u8,
    /// Uptime after which a boot counts as stable (see
    /// [`RetainedBoot::mark_stable`]).
    pub window_ms: u32,
}

/// What [`RetainedBoot::boot`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootRecord {
    /// Cause of the previous boot's unplanned reset, to be noted with
    /// [`crate::AnomalyRecord::note_reset`].
    pub carried: Option<ResetCause>,
    /// Consecutive boots (before this one) that ended before the stability
    /// window.
    pub rapid_resets: u8,
    /// `rapid_resets` reached the policy limit: keep this boot awake.
    pub guard_tripped: bool,
    /// The guard tripped for the first time in this burst; a product notes
    /// [`crate::AnomalyReason::SleepDisabled`] only then, so a loop that
    /// survives the guard does not keep writing flash.
    pub guard_first_trip: bool,
}

/// Number of retained words used by [`RetainedBoot`].
pub const RETAINED_WORDS: usize = 4;

const MAGIC: u32 = 0x5EB0_07C5;
const CHECK_SALT: u32 = 0xA5A5_5A5A;
const STABLE: u32 = 1 << 8;

/// Reset-cause and rapid-reset state in [`RETAINED_WORDS`] retained words:
/// `[magic, cause word, flags, check]`, flags = rapid count (bits 0–7),
/// stable mark (bit 8), resets since power-on (bits 16–31).
///
/// Words with a bad magic or check (power-on garbage, a reset during an
/// update) read as empty, never as a cause.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetainedBoot {
    pub words: [u32; RETAINED_WORDS],
}

impl RetainedBoot {
    pub const fn from_words(words: [u32; RETAINED_WORDS]) -> Self {
        Self { words }
    }

    const fn check(cause: u32, flags: u32) -> u32 {
        MAGIC ^ cause ^ flags.rotate_left(7) ^ CHECK_SALT
    }

    fn valid(&self) -> bool {
        self.words[0] == MAGIC && self.words[3] == Self::check(self.words[1], self.words[2])
    }

    fn set(&mut self, cause: u32, flags: u32) {
        self.words = [MAGIC, cause, flags, Self::check(cause, flags)];
    }

    fn flags(&self) -> u32 {
        if self.valid() { self.words[2] } else { 0 }
    }

    /// Resets since the last power-on (saturating).
    pub fn resets_since_power_on(&self) -> u16 {
        (self.flags() >> 16) as u16
    }

    /// Consume the retained state at boot and start this boot's entry.
    pub fn boot(&mut self, kind: ResetKind, policy: RapidResetPolicy) -> BootRecord {
        let valid = self.valid();
        if kind == ResetKind::PowerOn || !valid {
            self.set(0, 0);
            // Retained RAM invalid after a non-power-on reset: the cause is
            // lost, but an unplanned hardware reset is still worth noting.
            let carried = match kind {
                ResetKind::Unplanned(code) => Some(ResetCause::hardware(code)),
                _ => None,
            };
            return BootRecord {
                carried,
                rapid_resets: 0,
                guard_tripped: false,
                guard_first_trip: false,
            };
        }
        let flags = self.words[2];
        let carried = ResetCause::from_word(self.words[1]).or(match kind {
            ResetKind::Unplanned(code) => Some(ResetCause::hardware(code)),
            _ => None,
        });
        let rapid = if flags & STABLE != 0 {
            0
        } else {
            ((flags & 0xFF) as u8).saturating_add(1)
        };
        let resets = ((flags >> 16) as u16).saturating_add(1);
        self.set(0, u32::from(rapid) | (u32::from(resets) << 16));
        let guard_tripped = policy.limit != 0 && rapid >= policy.limit;
        BootRecord {
            carried,
            rapid_resets: rapid,
            guard_tripped,
            guard_first_trip: guard_tripped && rapid == policy.limit,
        }
    }

    /// Record why this boot is about to reset; the next
    /// [`boot`](Self::boot) carries it.
    pub fn record(&mut self, cause: ResetCause) {
        let flags = self.flags();
        self.set(cause.word(), flags);
    }

    /// This boot survived the stability window: its eventual reset is not a
    /// quick one. Clears the rapid count.
    pub fn mark_stable(&mut self) {
        let flags = self.flags();
        let cause = if self.valid() { self.words[1] } else { 0 };
        self.set(cause, (flags & !0xFF) | STABLE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GUARD: RapidResetPolicy = RapidResetPolicy {
        limit: 3,
        window_ms: 30_000,
    };

    fn power_on() -> RetainedBoot {
        // Arbitrary power-on content must not be read as state.
        let mut retained = RetainedBoot::from_words([0x1234, 0xFFFF_FFFF, 7, 9]);
        let record = retained.boot(ResetKind::PowerOn, GUARD);
        assert_eq!(record.carried, None);
        assert_eq!(record.rapid_resets, 0);
        retained
    }

    #[test]
    fn cause_encoding_is_stable() {
        let wake = ResetCause::wake(0x0400_0003);
        assert_eq!(wake.word(), 0x1400_0003);
        assert_eq!(wake.kind(), ResetCause::KIND_WAKE);
        assert_eq!(wake.payload(), 0x0400_0003);
        // Payload never spills into the kind.
        assert_eq!(ResetCause::wake(u32::MAX).kind(), ResetCause::KIND_WAKE);
        let panic = ResetCause::panic_site(0x0001_2344);
        assert_eq!(panic.word(), 0x2001_2344);
        assert_eq!(panic.kind(), ResetCause::KIND_PANIC);
        assert_eq!(panic.payload(), 0x0001_2344);
        assert_eq!(ResetCause::from_word(0), None);
        assert_eq!(ResetCause::hardware(0x0F).word(), 0x4000_000F);
    }

    #[test]
    fn recorded_cause_is_carried_once() {
        let mut retained = power_on();
        retained.record(ResetCause::wake(42));
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.carried, Some(ResetCause::wake(42)));
        assert_eq!(retained.resets_since_power_on(), 1);
        // A later planned reset carries nothing.
        retained.mark_stable();
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.carried, None);
        assert_eq!(record.rapid_resets, 0);
    }

    #[test]
    fn power_on_forgets_everything() {
        let mut retained = power_on();
        retained.record(ResetCause::wake(42));
        let record = retained.boot(ResetKind::PowerOn, GUARD);
        assert_eq!(record.carried, None);
        assert_eq!(retained.resets_since_power_on(), 0);
    }

    #[test]
    fn unplanned_hardware_reset_is_carried_even_without_a_cause() {
        let mut retained = power_on();
        let record = retained.boot(ResetKind::Unplanned(0x0F), GUARD);
        assert_eq!(record.carried, Some(ResetCause::hardware(0x0F)));
        // A recorded software cause takes precedence.
        retained.record(ResetCause::profile());
        let record = retained.boot(ResetKind::Unplanned(0x07), GUARD);
        assert_eq!(record.carried, Some(ResetCause::profile()));
        // Damaged retained words after an unplanned reset still note it.
        let mut damaged = RetainedBoot::from_words([0; RETAINED_WORDS]);
        let record = damaged.boot(ResetKind::Unplanned(0x10), GUARD);
        assert_eq!(record.carried, Some(ResetCause::hardware(0x10)));
        let record = damaged.boot(ResetKind::Software, GUARD);
        assert_eq!(record.carried, None);
    }

    #[test]
    fn torn_or_corrupt_words_never_produce_a_cause() {
        let mut retained = power_on();
        retained.record(ResetCause::wake(7));
        for index in 0..RETAINED_WORDS {
            let mut torn = retained;
            torn.words[index] ^= 0x10;
            let record = torn.boot(ResetKind::Software, GUARD);
            assert_eq!(record.carried, None, "word {index}");
            assert_eq!(record.rapid_resets, 0);
        }
    }

    #[test]
    fn guard_trips_after_limit_quick_resets_and_reports_first_trip_once() {
        let mut retained = power_on();
        let mut trips = [false; 6];
        let mut first = [false; 6];
        for boot in 0..6 {
            retained.record(ResetCause::wake(1));
            let record = retained.boot(ResetKind::Software, GUARD);
            assert_eq!(record.rapid_resets, boot as u8 + 1);
            trips[boot] = record.guard_tripped;
            first[boot] = record.guard_first_trip;
        }
        assert_eq!(trips, [false, false, true, true, true, true]);
        assert_eq!(first, [false, false, true, false, false, false]);
    }

    #[test]
    fn a_stable_boot_restarts_the_count() {
        let mut retained = power_on();
        retained.boot(ResetKind::Software, GUARD);
        retained.boot(ResetKind::Software, GUARD);
        retained.mark_stable();
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.rapid_resets, 0);
        assert!(!record.guard_tripped);
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.rapid_resets, 1);
    }

    #[test]
    fn mark_stable_keeps_a_recorded_cause() {
        let mut retained = power_on();
        retained.record(ResetCause::profile());
        retained.mark_stable();
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.carried, Some(ResetCause::profile()));
        assert_eq!(record.rapid_resets, 0);
    }

    #[test]
    fn zero_limit_disables_the_guard() {
        let mut retained = power_on();
        let policy = RapidResetPolicy {
            limit: 0,
            window_ms: 1,
        };
        for _ in 0..300 {
            assert!(!retained.boot(ResetKind::Software, policy).guard_tripped);
        }
        // The count saturates instead of wrapping back below a limit.
        let record = retained.boot(ResetKind::Software, GUARD);
        assert_eq!(record.rapid_resets, u8::MAX);
        assert!(record.guard_tripped && !record.guard_first_trip);
    }
}
