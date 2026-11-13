//! Status palette of the DK's TLC59731 RGB LED.
//!
//! The shared [`router_app::StatusIndicator`] selects the indication from
//! the router lifecycle, the NWK permit-join state and identify; the product
//! owns colours, brightness and patterns. The board's LED driver
//! (`boards/stm32wb5mm-dk`, feature `rgb-led`) only writes the colour.

use router_app::{IndicationStyle, Rgb, StatusPalette};

/// Highest drive value of any channel (of 255). The module's LED is
/// readable indoors at a few percent.
pub const MAX_CHANNEL: u8 = 32;

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
    booting: IndicationStyle::solid(Rgb::new(8, 8, 8)),
    searching: IndicationStyle::blink(Rgb::new(24, 8, 0), 500, 1_000),
    joined: IndicationStyle::solid(Rgb::new(0, 16, 0)),
    permit_join: IndicationStyle::blink(Rgb::new(0, 0, 28), 1_000, 2_000),
    identify: IndicationStyle::blink(Rgb::new(20, 0, 20), 250, 500),
    ota: IndicationStyle::blink(Rgb::new(0, 16, 16), 100, 1_000),
    error: IndicationStyle::solid(Rgb::new(MAX_CHANNEL, 0, 0)),
    joined_hold_ms: Some(30_000),
};

const _: () = assert!(STATUS_PALETTE.is_valid());

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
            let c = STATUS_PALETTE.style(*a).unwrap().color;
            assert!(!c.is_off(), "{a:?} must be visible");
            assert!(c.r <= MAX_CHANNEL && c.g <= MAX_CHANNEL && c.b <= MAX_CHANNEL);
            for b in &ALL[i + 1..] {
                assert_ne!(c, STATUS_PALETTE.style(*b).unwrap().color, "{a:?}/{b:?}");
            }
        }
    }

    #[test]
    fn identify_blinks_magenta_and_permit_join_blue() {
        let mut indicator = StatusIndicator::new(&STATUS_PALETTE, 0);
        let online = |identifying| RouterStatus::Online {
            archetype: NodeArchetype::ParentRouter,
            short_address: 0xC1E6,
            identifying,
        };
        indicator.set_status(online(true), 0);
        assert_eq!(indicator.update(0), Rgb::new(20, 0, 20));
        assert_eq!(indicator.update(300), Rgb::OFF);
        indicator.set_status(online(false), 400);
        indicator.set_permit_join(true);
        assert_eq!(indicator.update(400), Rgb::new(0, 0, 28));
    }
}
