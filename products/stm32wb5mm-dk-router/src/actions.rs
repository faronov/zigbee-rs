//! What the DK's buttons and touch key do. The composition root classifies
//! each input with [`crate::button::ButtonGestures`] and applies the
//! returned [`Action`].
//!
//! | Input | Gesture | Action |
//! |-------|---------|--------|
//! | B1 | short press | factory-new: start commissioning; joined: open this router for joining for [`PERMIT_JOIN_SECS`] |
//! | B1 | hold [`crate::button::LONG_HOLD_MS`] | journal-aware factory reset |
//! | B2 | short press | identify for [`IDENTIFY_SECS`] (status LED blinks) |
//! | B2 | hold | close this router for joining |
//! | touch key | tap | next display page |
//! | touch key | hold | first display page |

use crate::button::ButtonGesture;

/// Local permit-join window opened by B1 on a joined router.
pub const PERMIT_JOIN_SECS: u8 = 180;
/// Local identify time started by B2.
pub const IDENTIFY_SECS: u16 = 10;

// 0 would close and 0xFF would open indefinitely.
const _: () = assert!(PERMIT_JOIN_SECS > 0 && PERMIT_JOIN_SECS < 0xFF);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    B1(ButtonGesture),
    B2(ButtonGesture),
    Touch(ButtonGesture),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Request commissioning; if the app refuses because the node is
    /// joined, open permit-join instead.
    CommissionOrPermitJoin,
    FactoryReset,
    Identify,
    ClosePermitJoin,
    NextPage,
    FirstPage,
}

pub const fn action(input: Input) -> Action {
    match input {
        Input::B1(ButtonGesture::ShortPress) => Action::CommissionOrPermitJoin,
        Input::B1(ButtonGesture::LongHold) => Action::FactoryReset,
        Input::B2(ButtonGesture::ShortPress) => Action::Identify,
        Input::B2(ButtonGesture::LongHold) => Action::ClosePermitJoin,
        Input::Touch(ButtonGesture::ShortPress) => Action::NextPage,
        Input::Touch(ButtonGesture::LongHold) => Action::FirstPage,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_b1_hold_resets() {
        let inputs = [
            Input::B1(ButtonGesture::ShortPress),
            Input::B1(ButtonGesture::LongHold),
            Input::B2(ButtonGesture::ShortPress),
            Input::B2(ButtonGesture::LongHold),
            Input::Touch(ButtonGesture::ShortPress),
            Input::Touch(ButtonGesture::LongHold),
        ];
        let resets = inputs
            .iter()
            .filter(|i| action(**i) == Action::FactoryReset)
            .count();
        assert_eq!(resets, 1);
        assert_eq!(
            action(Input::B1(ButtonGesture::LongHold)),
            Action::FactoryReset
        );
        assert_eq!(
            action(Input::B2(ButtonGesture::ShortPress)),
            Action::Identify
        );
    }
}
