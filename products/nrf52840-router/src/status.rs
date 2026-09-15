//! Product mapping from semantic always-on End Device lifecycle state to DK LED1.

use router_app::RouterStatus;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusLedState {
    Off,
    On,
}

/// Map semantic application state to the product's active-low status LED.
///
/// Online is solid on. Commissioning and rejoin attempts alternate the LED on
/// each attempt without introducing an application-side delay. Identify makes
/// the normal online LED dark for the identify interval. Reset and fault are
/// solid on so neither can be mistaken for an idle, unpowered application.
pub const fn led1_state(status: RouterStatus) -> StatusLedState {
    match status {
        RouterStatus::Starting { .. } => StatusLedState::Off,
        RouterStatus::Commissioning { attempt, .. }
        | RouterStatus::Recommissioning { attempt, .. } => {
            if attempt & 1 == 1 {
                StatusLedState::On
            } else {
                StatusLedState::Off
            }
        }
        RouterStatus::Rejoining { failures, .. } => {
            if failures & 1 == 0 {
                StatusLedState::On
            } else {
                StatusLedState::Off
            }
        }
        RouterStatus::Online { identifying, .. } => {
            if identifying {
                StatusLedState::Off
            } else {
                StatusLedState::On
            }
        }
        RouterStatus::Resetting { .. } | RouterStatus::Fault { .. } => StatusLedState::On,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use router_app::NodeArchetype;

    const ALWAYS_ON_END_DEVICE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

    #[test]
    fn online_and_identify_have_distinct_semantics() {
        assert_eq!(
            led1_state(RouterStatus::Online {
                archetype: ALWAYS_ON_END_DEVICE,
                short_address: 0x1234,
                identifying: false,
            }),
            StatusLedState::On
        );
        assert_eq!(
            led1_state(RouterStatus::Online {
                archetype: ALWAYS_ON_END_DEVICE,
                short_address: 0x1234,
                identifying: true,
            }),
            StatusLedState::Off
        );
    }

    #[test]
    fn retry_attempts_alternate_without_unbounded_waits() {
        assert_eq!(
            led1_state(RouterStatus::Commissioning {
                archetype: ALWAYS_ON_END_DEVICE,
                attempt: 1,
            }),
            StatusLedState::On
        );
        assert_eq!(
            led1_state(RouterStatus::Recommissioning {
                archetype: ALWAYS_ON_END_DEVICE,
                attempt: 2,
                retry_in_ms: 60_000,
            }),
            StatusLedState::Off
        );
    }

    #[test]
    fn reset_and_fault_are_visible() {
        assert_eq!(
            led1_state(RouterStatus::Resetting {
                archetype: ALWAYS_ON_END_DEVICE
            }),
            StatusLedState::On
        );
        assert_eq!(
            led1_state(RouterStatus::Fault {
                archetype: ALWAYS_ON_END_DEVICE
            }),
            StatusLedState::On
        );
    }
}
