//! Status indication on the board's WS2812 for the mains roles (feature
//! `status-led`).
//!
//! The product owns the palette (colours, brightness, blink patterns, the
//! joined hold) and composes the platform-independent
//! [`router_app::StatusIndicator`] with the board's RMT-driven
//! [`esp32_zigbee_devkit::status_led`] LED. Sleepy images never enable this
//! feature: they keep GPIO8 isolated and link no RMT driver.

use router_app::{IndicationStyle, Rgb, StatusPalette};

/// Highest drive value of any channel in [`STATUS_PALETTE`] (of 255): a
/// bare WS2812 is readable indoors at under 10 % drive.
pub const MAX_CHANNEL: u8 = 24;

/// Mains-role palette:
///
/// | state        | colour  | pattern                         |
/// |--------------|---------|---------------------------------|
/// | booting      | white   | solid                           |
/// | searching    | amber   | 0.5 s on / 0.5 s off            |
/// | joined       | green   | solid for 30 s, then dark       |
/// | permit join  | blue    | 1 s on / 1 s off                |
/// | identify     | magenta | 0.25 s on / 0.25 s off          |
/// | OTA          | cyan    | 0.1 s flash every second        |
/// | error        | red     | solid                           |
pub static STATUS_PALETTE: StatusPalette = StatusPalette {
    booting: IndicationStyle::solid(Rgb::new(6, 6, 6)),
    searching: IndicationStyle::blink(Rgb::new(16, 6, 0), 500, 1_000),
    joined: IndicationStyle::solid(Rgb::new(0, 12, 0)),
    permit_join: IndicationStyle::blink(Rgb::new(0, 0, 20), 1_000, 2_000),
    identify: IndicationStyle::blink(Rgb::new(16, 0, 16), 250, 500),
    ota: IndicationStyle::blink(Rgb::new(0, 12, 12), 100, 1_000),
    error: IndicationStyle::solid(Rgb::new(MAX_CHANNEL, 0, 0)),
    joined_hold_ms: Some(30_000),
};

const _: () = assert!(STATUS_PALETTE.is_valid());

#[cfg(target_os = "none")]
pub use adapter::RgbStatusLed;
/// The board's RMT WS2812 driver, re-exported for the composition roots.
#[cfg(target_os = "none")]
pub use esp32_zigbee_devkit::status_led::{StatusLed, StatusLedError};

#[cfg(target_os = "none")]
mod adapter {
    use esp32_zigbee_devkit::status_led::{StatusLed, StatusLedError};
    use router_app::{Rgb, RouterStatus, StatusIndicator, StatusPalette, StatusSink};

    fn now_ms() -> u32 {
        embassy_time::Instant::now().as_millis() as u32
    }

    /// [`StatusSink`] that forwards every status to `inner` (the console
    /// log) and shows it on the WS2812. Only colour changes are written,
    /// so the RMT is idle between blink edges.
    pub struct RgbStatusLed<S> {
        inner: S,
        led: Option<StatusLed>,
        indicator: StatusIndicator<'static>,
        shown: Option<Rgb>,
        write_failed: bool,
    }

    impl<S: StatusSink> RgbStatusLed<S> {
        /// `led` is `None` when the RMT could not be configured; the
        /// sink then only forwards to `inner`. Shows the booting colour.
        pub fn new(inner: S, led: Option<StatusLed>, palette: &'static StatusPalette) -> Self {
            let mut this = Self {
                inner,
                led,
                indicator: StatusIndicator::new(palette, now_ms()),
                shown: None,
                write_failed: false,
            };
            this.render();
            this
        }

        /// Call once per run-loop step: advances blink phases and the
        /// joined hold, and applies the NWK permit-join state.
        pub fn refresh(&mut self, permit_join: bool) {
            self.indicator.set_permit_join(permit_join);
            self.render();
        }

        /// Mutable access to the indicator, e.g. to feed OTA events.
        pub fn indicator_mut(&mut self) -> &mut StatusIndicator<'static> {
            &mut self.indicator
        }

        fn render(&mut self) {
            let color = self.indicator.update(now_ms());
            if self.shown == Some(color) {
                return;
            }
            let Some(led) = self.led.as_mut() else {
                return;
            };
            match led.write(color.r, color.g, color.b) {
                Ok(()) => self.shown = Some(color),
                Err(error) => {
                    if !self.write_failed {
                        self.write_failed = true;
                        log::warn!("status LED write failed: {:?}", error);
                    }
                    if error == StatusLedError::ChannelLost {
                        self.led = None;
                    }
                }
            }
        }
    }

    impl<S: StatusSink> StatusSink for RgbStatusLed<S> {
        fn set(&mut self, status: RouterStatus) {
            self.inner.set(status);
            self.indicator.set_status(status, now_ms());
            self.render();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use router_app::{Indication, NodeArchetype, RouterStatus, StatusIndicator};

    const ALL: [Indication; 7] = [
        Indication::Booting,
        Indication::Searching,
        Indication::Joined,
        Indication::PermitJoin,
        Indication::Identify,
        Indication::Ota,
        Indication::Error,
    ];

    #[test]
    fn palette_is_dim_and_every_state_is_lit_and_distinct() {
        for (i, a) in ALL.iter().enumerate() {
            let style = STATUS_PALETTE.style(*a).unwrap();
            let c = style.color;
            assert!(!c.is_off(), "{a:?} must be visible");
            assert!(
                c.r <= MAX_CHANNEL && c.g <= MAX_CHANNEL && c.b <= MAX_CHANNEL,
                "{a:?} too bright"
            );
            for b in &ALL[i + 1..] {
                assert_ne!(c, STATUS_PALETTE.style(*b).unwrap().color, "{a:?}/{b:?}");
            }
        }
        assert_eq!(STATUS_PALETTE.style(Indication::Off), None);
    }

    #[test]
    fn router_goes_dark_thirty_seconds_after_joining() {
        let mut indicator = StatusIndicator::new(&STATUS_PALETTE, 0);
        indicator.set_status(
            RouterStatus::Online {
                archetype: NodeArchetype::ParentRouter,
                short_address: 0x0001,
                identifying: false,
            },
            1_000,
        );
        assert_eq!(indicator.update(30_999), STATUS_PALETTE.joined.color);
        assert_eq!(indicator.update(31_000), Rgb::OFF);
    }
}
