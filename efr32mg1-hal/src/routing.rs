use crate::gpio::{Pin, Port};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RouteSignal {
    Primary,
    Secondary,
    Tertiary,
}

/// EFR32xG1 USART0 and TIMER0 signals share the same three LOC tables.
pub(crate) const fn signal_pin(signal: RouteSignal, location: u8) -> Option<Pin> {
    match signal {
        RouteSignal::Primary => primary_pin(location),
        RouteSignal::Secondary => secondary_pin(location),
        RouteSignal::Tertiary => tertiary_pin(location),
    }
}

const fn primary_pin(location: u8) -> Option<Pin> {
    match location {
        0..=5 => Some(Pin::new(Port::A, location)),
        6..=10 => Some(Pin::new(Port::B, location + 5)),
        11..=16 => Some(Pin::new(Port::C, location - 5)),
        17..=23 => Some(Pin::new(Port::D, location - 8)),
        24..=31 => Some(Pin::new(Port::F, location - 24)),
        _ => None,
    }
}

const fn secondary_pin(location: u8) -> Option<Pin> {
    match location {
        0..=4 => Some(Pin::new(Port::A, location + 1)),
        5..=9 => Some(Pin::new(Port::B, location + 6)),
        10..=15 => Some(Pin::new(Port::C, location - 4)),
        16..=22 => Some(Pin::new(Port::D, location - 7)),
        23..=30 => Some(Pin::new(Port::F, location - 23)),
        31 => Some(Pin::new(Port::A, 0)),
        _ => None,
    }
}

const fn tertiary_pin(location: u8) -> Option<Pin> {
    match location {
        0..=3 => Some(Pin::new(Port::A, location + 2)),
        4..=8 => Some(Pin::new(Port::B, location + 7)),
        9..=14 => Some(Pin::new(Port::C, location - 3)),
        15..=21 => Some(Pin::new(Port::D, location - 6)),
        22..=29 => Some(Pin::new(Port::F, location - 22)),
        30..=31 => Some(Pin::new(Port::A, location - 30)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{RouteSignal, signal_pin};
    use crate::gpio::{Pin, Port};

    #[test]
    fn tradfri_usart0_routes_match_device_tables() {
        assert_eq!(
            signal_pin(RouteSignal::Primary, 23),
            Some(Pin::new(Port::D, 15))
        );
        assert_eq!(
            signal_pin(RouteSignal::Secondary, 21),
            Some(Pin::new(Port::D, 14))
        );
        assert_eq!(
            signal_pin(RouteSignal::Tertiary, 19),
            Some(Pin::new(Port::D, 13))
        );
    }

    #[test]
    fn timer0_led_route_is_pa0_location_zero() {
        assert_eq!(
            signal_pin(RouteSignal::Primary, 0),
            Some(Pin::new(Port::A, 0))
        );
    }
}
