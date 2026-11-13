//! Router actions shared by the STM32WB products. Each product maps its
//! buttons' gestures ([`crate::button`]) onto these; the composition root
//! carries them out.

use crate::button::ButtonGesture;

/// Local permit-join window opened on a joined router.
pub const PERMIT_JOIN_SECS: u8 = 180;
/// Local identify time.
pub const IDENTIFY_SECS: u16 = 10;

// 0 would close and 0xFF would open indefinitely.
const _: () = assert!(PERMIT_JOIN_SECS > 0 && PERMIT_JOIN_SECS < 0xFF);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Request commissioning (join, or rejoin after a Leave); if the app
    /// refuses because the node is joined, open permit-join for
    /// [`PERMIT_JOIN_SECS`] instead.
    CommissionOrPermitJoin,
    /// Journal-aware factory reset, then commissioning from factory-new.
    FactoryReset,
    /// Identify for [`IDENTIFY_SECS`].
    Identify,
    /// Close this router for joining.
    ClosePermitJoin,
    /// Reset the chip; a commissioned router resumes its network from the
    /// journals.
    Restart,
}

/// What one button's gestures do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonMap {
    /// Button name in the log (the board silkscreen).
    pub name: &'static str,
    pub short_press: Option<Action>,
    pub long_hold: Option<Action>,
}

impl ButtonMap {
    pub const fn action(&self, gesture: ButtonGesture) -> Option<Action> {
        match gesture {
            ButtonGesture::ShortPress => self.short_press,
            ButtonGesture::LongHold => self.long_hold,
        }
    }
}

/// Count the maps whose long hold factory-resets.
pub fn factory_reset_holds(maps: &[ButtonMap]) -> usize {
    maps.iter()
        .filter(|map| map.long_hold == Some(Action::FactoryReset))
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_selects_the_gesture_action() {
        const MAP: ButtonMap = ButtonMap {
            name: "SW1",
            short_press: Some(Action::CommissionOrPermitJoin),
            long_hold: Some(Action::FactoryReset),
        };
        assert_eq!(
            MAP.action(ButtonGesture::ShortPress),
            Some(Action::CommissionOrPermitJoin)
        );
        assert_eq!(
            MAP.action(ButtonGesture::LongHold),
            Some(Action::FactoryReset)
        );
        assert_eq!(factory_reset_holds(&[MAP]), 1);
    }
}
