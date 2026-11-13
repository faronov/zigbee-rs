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

//!
//! The router actions and their timings are shared with the other STM32WB
//! routers ([`stm32wb_router_product::actions`]).

use crate::button::ButtonGesture;
pub use stm32wb_router_product::actions::{
    Action as RouterAction, ButtonMap, IDENTIFY_SECS, PERMIT_JOIN_SECS,
};

/// B1 and B2 as shared router button maps.
pub const B1: ButtonMap = ButtonMap {
    name: "B1",
    short_press: Some(RouterAction::CommissionOrPermitJoin),
    long_hold: Some(RouterAction::FactoryReset),
};
pub const B2: ButtonMap = ButtonMap {
    name: "B2",
    short_press: Some(RouterAction::Identify),
    long_hold: Some(RouterAction::ClosePermitJoin),
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    B1(ButtonGesture),
    B2(ButtonGesture),
    Touch(ButtonGesture),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// A shared router action, named by the button that requested it.
    Router(RouterAction, &'static str),
    NextPage,
    FirstPage,
}

/// `None` for an unmapped gesture (every DK gesture is mapped).
pub const fn action(input: Input) -> Option<Action> {
    let (map, gesture) = match input {
        Input::B1(gesture) => (&B1, gesture),
        Input::B2(gesture) => (&B2, gesture),
        Input::Touch(ButtonGesture::ShortPress) => return Some(Action::NextPage),
        Input::Touch(ButtonGesture::LongHold) => return Some(Action::FirstPage),
    };
    match map.action(gesture) {
        Some(action) => Some(Action::Router(action, map.name)),
        None => None,
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
            .filter(|i| {
                matches!(
                    action(**i),
                    Some(Action::Router(RouterAction::FactoryReset, _))
                )
            })
            .count();
        assert_eq!(resets, 1);
        assert_eq!(
            action(Input::B1(ButtonGesture::LongHold)),
            Some(Action::Router(RouterAction::FactoryReset, "B1"))
        );
        assert_eq!(
            action(Input::B2(ButtonGesture::ShortPress)),
            Some(Action::Router(RouterAction::Identify, "B2"))
        );
        assert!(inputs.iter().all(|i| action(*i).is_some()));
    }
}
