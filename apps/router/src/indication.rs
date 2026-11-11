//! Product-configurable status indication for one RGB indicator.
//!
//! [`StatusIndicator`] turns the semantic lifecycle a [`crate::StatusSink`]
//! receives, plus the network's permit-join and OTA state, into a single
//! colour for the current instant. It owns no pin, timer or peripheral:
//! the composition root feeds it a millisecond clock and writes the
//! returned [`Rgb`] to whatever indicator the board fits (an addressable
//! WS2812, three PWM channels, ...). Colours, brightness, blink patterns and
//! the joined hold time come from a product [`StatusPalette`].
//!
//! Indication priority, highest first:
//!
//! 1. [`Indication::Error`] — [`RouterStatus::Fault`];
//! 2. [`Indication::Identify`] — ZCL Identify running on the node endpoint;
//! 3. [`Indication::Ota`] — an OTA download in progress;
//! 4. [`Indication::PermitJoin`] — the node accepts joins (online only);
//! 5. [`Indication::Joined`] — online, then [`Indication::Off`] after
//!    [`StatusPalette::joined_hold_ms`];
//! 6. [`Indication::Searching`] — commissioning, recommissioning or a
//!    secured rejoin;
//! 7. [`Indication::Booting`] — before the first status and while the node
//!    starts or resets to factory-new.

use crate::{RouterStatus, StackEventSummary};

/// 8-bit RGB colour; the indicator driver owns the wire colour order.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const OFF: Self = Self::new(0, 0, 0);

    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn is_off(self) -> bool {
        self.r == 0 && self.g == 0 && self.b == 0
    }
}

/// What the indicator currently shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Indication {
    Booting,
    Searching,
    Joined,
    PermitJoin,
    Identify,
    Ota,
    Error,
    /// Online after the joined hold expired: the indicator is dark.
    Off,
}

/// Temporal pattern of one indication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pattern {
    Solid,
    /// Lit for `on_ms` at the start of every `period_ms`, starting lit when
    /// the indication begins. Requires `0 < on_ms < period_ms`.
    Blink {
        on_ms: u16,
        period_ms: u16,
    },
}

/// Colour and pattern of one indication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndicationStyle {
    pub color: Rgb,
    pub pattern: Pattern,
}

impl IndicationStyle {
    pub const fn solid(color: Rgb) -> Self {
        Self {
            color,
            pattern: Pattern::Solid,
        }
    }

    pub const fn blink(color: Rgb, on_ms: u16, period_ms: u16) -> Self {
        Self {
            color,
            pattern: Pattern::Blink { on_ms, period_ms },
        }
    }

    /// Colour `elapsed_ms` after this indication began.
    pub const fn color_at(&self, elapsed_ms: u32) -> Rgb {
        match self.pattern {
            Pattern::Solid => self.color,
            Pattern::Blink { on_ms, period_ms } => {
                if period_ms == 0 || elapsed_ms % period_ms as u32 >= on_ms as u32 {
                    Rgb::OFF
                } else {
                    self.color
                }
            }
        }
    }

    const fn is_valid(&self) -> bool {
        match self.pattern {
            Pattern::Solid => true,
            Pattern::Blink { on_ms, period_ms } => on_ms > 0 && on_ms < period_ms,
        }
    }
}

/// Product indication configuration. Colours are the final drive values,
/// so brightness is chosen here (keep them low for an indoor indicator).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusPalette {
    pub booting: IndicationStyle,
    pub searching: IndicationStyle,
    pub joined: IndicationStyle,
    pub permit_join: IndicationStyle,
    pub identify: IndicationStyle,
    pub ota: IndicationStyle,
    pub error: IndicationStyle,
    /// How long [`Indication::Joined`] stays lit after the node comes
    /// online; `None` keeps it lit.
    pub joined_hold_ms: Option<u32>,
}

impl StatusPalette {
    /// Whether every blink pattern is well formed; check it in a `const`
    /// assertion next to the product palette.
    pub const fn is_valid(&self) -> bool {
        self.booting.is_valid()
            && self.searching.is_valid()
            && self.joined.is_valid()
            && self.permit_join.is_valid()
            && self.identify.is_valid()
            && self.ota.is_valid()
            && self.error.is_valid()
    }

    /// Style of `indication`; `None` for [`Indication::Off`].
    pub const fn style(&self, indication: Indication) -> Option<IndicationStyle> {
        match indication {
            Indication::Booting => Some(self.booting),
            Indication::Searching => Some(self.searching),
            Indication::Joined => Some(self.joined),
            Indication::PermitJoin => Some(self.permit_join),
            Indication::Identify => Some(self.identify),
            Indication::Ota => Some(self.ota),
            Indication::Error => Some(self.error),
            Indication::Off => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Lifecycle {
    Booting,
    Searching,
    Online { identifying: bool },
    Fault,
}

/// Lifecycle + network state → colour, against a caller-supplied
/// wrapping millisecond clock. Call [`Self::update`] at least every few
/// hundred milliseconds while a blink pattern or the joined hold runs.
#[derive(Debug, Clone)]
pub struct StatusIndicator<'p> {
    palette: &'p StatusPalette,
    lifecycle: Lifecycle,
    permit_join: bool,
    ota: bool,
    online_since_ms: u32,
    /// Latched once the joined hold expired, so a wrapping clock can never
    /// relight the indicator.
    joined_expired: bool,
    active: Indication,
    active_since_ms: u32,
}

impl<'p> StatusIndicator<'p> {
    /// Starts in [`Indication::Booting`] at `now_ms`.
    pub const fn new(palette: &'p StatusPalette, now_ms: u32) -> Self {
        Self {
            palette,
            lifecycle: Lifecycle::Booting,
            permit_join: false,
            ota: false,
            online_since_ms: now_ms,
            joined_expired: false,
            active: Indication::Booting,
            active_since_ms: now_ms,
        }
    }

    /// Apply a lifecycle status from [`crate::StatusSink::set`].
    pub fn set_status(&mut self, status: RouterStatus, now_ms: u32) {
        let lifecycle = match status {
            RouterStatus::Starting { .. } | RouterStatus::Resetting { .. } => Lifecycle::Booting,
            RouterStatus::Commissioning { .. }
            | RouterStatus::Recommissioning { .. }
            | RouterStatus::Rejoining { .. } => Lifecycle::Searching,
            RouterStatus::Online { identifying, .. } => Lifecycle::Online { identifying },
            RouterStatus::Fault { .. } => Lifecycle::Fault,
        };
        let online = matches!(lifecycle, Lifecycle::Online { .. });
        if online != matches!(self.lifecycle, Lifecycle::Online { .. }) {
            // Neither carries across joining or leaving a network.
            self.permit_join = false;
            self.ota = false;
            if online {
                self.online_since_ms = now_ms;
                self.joined_expired = false;
            }
        }
        self.lifecycle = lifecycle;
    }

    /// Whether the node currently permits joining (read from the NWK NIB;
    /// the stack reports only openings as events, not the timed close).
    /// Ignored unless online.
    pub fn set_permit_join(&mut self, open: bool) {
        self.permit_join = open;
    }

    /// Whether an OTA download is in progress. Ignored unless online.
    pub fn set_ota_active(&mut self, active: bool) {
        self.ota = active;
    }

    /// Track OTA progress from stack events; every other event is ignored.
    pub fn observe(&mut self, event: &StackEventSummary) {
        match event {
            StackEventSummary::OtaImageAvailable { .. } | StackEventSummary::OtaProgress { .. } => {
                self.ota = true
            }
            StackEventSummary::OtaComplete
            | StackEventSummary::OtaFailed
            | StackEventSummary::OtaDelayedActivation { .. } => self.ota = false,
            _ => {}
        }
    }

    /// The indication selected at `now_ms` (without advancing blink phase).
    pub fn indication(&self, now_ms: u32) -> Indication {
        match self.lifecycle {
            Lifecycle::Fault => Indication::Error,
            Lifecycle::Booting => Indication::Booting,
            Lifecycle::Searching => Indication::Searching,
            Lifecycle::Online { identifying: true } => Indication::Identify,
            Lifecycle::Online { .. } if self.ota => Indication::Ota,
            Lifecycle::Online { .. } if self.permit_join => Indication::PermitJoin,
            Lifecycle::Online { .. } => {
                if self.joined_hold_expired(now_ms) {
                    Indication::Off
                } else {
                    Indication::Joined
                }
            }
        }
    }

    fn joined_hold_expired(&self, now_ms: u32) -> bool {
        self.joined_expired
            || self
                .palette
                .joined_hold_ms
                .is_some_and(|hold| now_ms.wrapping_sub(self.online_since_ms) >= hold)
    }

    /// Advance to `now_ms` and return the colour to show. A newly selected
    /// indication restarts its blink phase lit.
    pub fn update(&mut self, now_ms: u32) -> Rgb {
        if matches!(self.lifecycle, Lifecycle::Online { .. }) && self.joined_hold_expired(now_ms) {
            self.joined_expired = true;
        }
        let indication = self.indication(now_ms);
        if indication != self.active {
            self.active = indication;
            self.active_since_ms = now_ms;
        }
        match self.palette.style(indication) {
            Some(style) => style.color_at(now_ms.wrapping_sub(self.active_since_ms)),
            None => Rgb::OFF,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NodeArchetype;

    const A: NodeArchetype = NodeArchetype::ParentRouter;
    const WHITE: Rgb = Rgb::new(4, 4, 4);
    const AMBER: Rgb = Rgb::new(12, 4, 0);
    const GREEN: Rgb = Rgb::new(0, 8, 0);
    const BLUE: Rgb = Rgb::new(0, 0, 12);
    const MAGENTA: Rgb = Rgb::new(10, 0, 10);
    const CYAN: Rgb = Rgb::new(0, 8, 8);
    const RED: Rgb = Rgb::new(16, 0, 0);

    static PALETTE: StatusPalette = StatusPalette {
        booting: IndicationStyle::solid(WHITE),
        searching: IndicationStyle::blink(AMBER, 500, 1_000),
        joined: IndicationStyle::solid(GREEN),
        permit_join: IndicationStyle::blink(BLUE, 1_000, 2_000),
        identify: IndicationStyle::blink(MAGENTA, 250, 500),
        ota: IndicationStyle::blink(CYAN, 100, 1_000),
        error: IndicationStyle::solid(RED),
        joined_hold_ms: Some(30_000),
    };
    const _: () = assert!(PALETTE.is_valid());

    fn online(identifying: bool) -> RouterStatus {
        RouterStatus::Online {
            archetype: A,
            short_address: 0x1234,
            identifying,
        }
    }

    #[test]
    fn boots_lit_before_any_status() {
        let mut ind = StatusIndicator::new(&PALETTE, 7);
        assert_eq!(ind.indication(7), Indication::Booting);
        assert_eq!(ind.update(7), WHITE);
        assert_eq!(ind.update(60_000), WHITE);
    }

    #[test]
    fn lifecycle_statuses_map_to_indications() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        let cases = [
            (RouterStatus::Starting { archetype: A }, Indication::Booting),
            (
                RouterStatus::Commissioning {
                    archetype: A,
                    attempt: 3,
                },
                Indication::Searching,
            ),
            (
                RouterStatus::Recommissioning {
                    archetype: A,
                    attempt: 1,
                    retry_in_ms: 5_000,
                },
                Indication::Searching,
            ),
            (
                RouterStatus::Rejoining {
                    archetype: A,
                    failures: 2,
                },
                Indication::Searching,
            ),
            (online(false), Indication::Joined),
            (online(true), Indication::Identify),
            (
                RouterStatus::Resetting { archetype: A },
                Indication::Booting,
            ),
            (RouterStatus::Fault { archetype: A }, Indication::Error),
        ];
        for (status, expected) in cases {
            ind.set_status(status, 0);
            assert_eq!(ind.indication(0), expected, "{status:?}");
            assert_eq!(
                ind.update(0),
                PALETTE.style(expected).unwrap().color,
                "{status:?}"
            );
        }
    }

    #[test]
    fn blink_restarts_lit_on_each_new_indication() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_status(
            RouterStatus::Commissioning {
                archetype: A,
                attempt: 1,
            },
            1_234,
        );
        assert_eq!(ind.update(1_234), AMBER);
        assert_eq!(ind.update(1_733), AMBER);
        assert_eq!(ind.update(1_734), Rgb::OFF);
        assert_eq!(ind.update(2_233), Rgb::OFF);
        assert_eq!(ind.update(2_234), AMBER);
    }

    #[test]
    fn joined_goes_dark_after_hold_and_stays_dark_across_clock_wrap() {
        let start = u32::MAX - 10_000;
        let mut ind = StatusIndicator::new(&PALETTE, start);
        ind.set_status(online(false), start);
        assert_eq!(ind.update(start), GREEN);
        assert_eq!(ind.update(start.wrapping_add(29_999)), GREEN);
        assert_eq!(ind.update(start.wrapping_add(30_000)), Rgb::OFF);
        assert_eq!(ind.indication(start.wrapping_add(30_000)), Indication::Off);
        // A full clock wrap later the raw elapsed time is small again.
        assert_eq!(ind.update(start.wrapping_add(5)), Rgb::OFF);
        // Re-reporting the same online status does not restart the hold.
        ind.set_status(online(false), start.wrapping_add(10));
        assert_eq!(ind.update(start.wrapping_add(10)), Rgb::OFF);
    }

    #[test]
    fn rejoining_the_network_restarts_the_joined_hold() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_status(online(false), 0);
        assert_eq!(ind.update(40_000), Rgb::OFF);
        ind.set_status(
            RouterStatus::Rejoining {
                archetype: A,
                failures: 0,
            },
            40_000,
        );
        assert_eq!(ind.update(40_000), AMBER);
        ind.set_status(online(false), 41_000);
        assert_eq!(ind.update(41_000), GREEN);
        assert_eq!(ind.update(71_000), Rgb::OFF);
    }

    #[test]
    fn no_hold_keeps_joined_lit() {
        let palette = StatusPalette {
            joined_hold_ms: None,
            ..PALETTE
        };
        let mut ind = StatusIndicator::new(&palette, 0);
        ind.set_status(online(false), 0);
        assert_eq!(ind.update(u32::MAX), GREEN);
    }

    #[test]
    fn permit_join_shows_only_while_online_and_returns_to_dark() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_permit_join(true);
        assert_eq!(ind.indication(0), Indication::Booting);
        ind.set_status(online(false), 0);
        // Leaving the pre-online state cleared the stale permit-join flag.
        assert_eq!(ind.indication(0), Indication::Joined);
        assert_eq!(ind.update(31_000), Rgb::OFF);
        ind.set_permit_join(true);
        assert_eq!(ind.update(32_000), BLUE);
        assert_eq!(ind.update(33_000), Rgb::OFF);
        assert_eq!(ind.update(34_000), BLUE);
        ind.set_permit_join(false);
        assert_eq!(ind.update(34_001), Rgb::OFF);
        assert_eq!(ind.indication(34_001), Indication::Off);
    }

    #[test]
    fn priorities_error_identify_ota_permit_join() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_status(online(false), 0);
        ind.set_permit_join(true);
        assert_eq!(ind.indication(0), Indication::PermitJoin);
        ind.observe(&StackEventSummary::OtaProgress { percent: 10 });
        assert_eq!(ind.indication(0), Indication::Ota);
        ind.set_status(online(true), 0);
        assert_eq!(ind.indication(0), Indication::Identify);
        assert_eq!(ind.update(0), MAGENTA);
        assert_eq!(ind.update(250), Rgb::OFF);
        ind.set_status(online(false), 300);
        assert_eq!(ind.indication(300), Indication::Ota);
        assert_eq!(ind.update(300), CYAN);
        ind.observe(&StackEventSummary::OtaFailed);
        assert_eq!(ind.indication(300), Indication::PermitJoin);
        ind.set_status(RouterStatus::Fault { archetype: A }, 400);
        assert_eq!(ind.update(400), RED);
    }

    #[test]
    fn leaving_the_network_clears_ota_and_permit_join() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_status(online(false), 0);
        ind.set_ota_active(true);
        ind.set_permit_join(true);
        ind.set_status(RouterStatus::Resetting { archetype: A }, 1);
        ind.set_status(online(false), 2);
        assert_eq!(ind.indication(2), Indication::Joined);
    }

    #[test]
    fn non_ota_events_are_ignored() {
        let mut ind = StatusIndicator::new(&PALETTE, 0);
        ind.set_status(online(false), 0);
        ind.observe(&StackEventSummary::PermitJoinChanged { open: true });
        ind.observe(&StackEventSummary::ReportSent);
        assert_eq!(ind.indication(0), Indication::Joined);
        ind.observe(&StackEventSummary::OtaImageAvailable {
            version: 2,
            size: 1,
        });
        assert_eq!(ind.indication(0), Indication::Ota);
        ind.observe(&StackEventSummary::OtaComplete);
        assert_eq!(ind.indication(0), Indication::Joined);
    }

    #[test]
    fn palette_validation_rejects_degenerate_blinks() {
        for (on_ms, period_ms) in [(0, 1_000), (1_000, 1_000), (1_001, 1_000), (0, 0)] {
            let palette = StatusPalette {
                searching: IndicationStyle::blink(AMBER, on_ms, period_ms),
                ..PALETTE
            };
            assert!(!palette.is_valid(), "{on_ms}/{period_ms}");
        }
        assert_eq!(
            IndicationStyle::blink(AMBER, 1, 0).color_at(5),
            Rgb::OFF,
            "a zero period never divides by zero"
        );
    }
}
