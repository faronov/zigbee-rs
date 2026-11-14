//! Button gestures shared by the STM32WB router products.
//!
//! The composition root samples each board button between bounded
//! application steps and feeds its logical level and a monotonic
//! millisecond clock into its own [`ButtonGestures`]. Each product decides
//! what a gesture means (its `actions` table, expressed in
//! [`crate::actions::Action`]):
//!
//! - **short press** (released after at least [`SHORT_PRESS_MIN_MS`] and
//!   before [`LONG_HOLD_MS`]), reported on release;
//! - **long hold** for [`LONG_HOLD_MS`], reported once while the button is
//!   still held. Releasing the button after a hold never also produces a
//!   short press.

/// Hold duration of a long hold (B1: factory reset).
pub const LONG_HOLD_MS: u64 = 3_000;

/// Shortest press accepted as a short press. Contact bounce and a single
/// glitching sample between receive slices are rejected.
pub const SHORT_PRESS_MIN_MS: u64 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButtonGesture {
    /// Debounced short press, reported on release.
    ShortPress,
    /// Button held for [`LONG_HOLD_MS`], reported once per press.
    LongHold,
}

/// Bounded, allocation-free button gesture classifier.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ButtonGestures {
    pressed_at_ms: Option<u64>,
    hold_reported: bool,
}

impl ButtonGestures {
    pub const fn new() -> Self {
        Self {
            pressed_at_ms: None,
            hold_reported: false,
        }
    }

    /// Feed one button sample. `pressed` is the logical (polarity-corrected)
    /// level and `now_ms` a monotonic millisecond timestamp.
    pub fn sample(&mut self, pressed: bool, now_ms: u64) -> Option<ButtonGesture> {
        if pressed {
            let pressed_at = *self.pressed_at_ms.get_or_insert(now_ms);
            if !self.hold_reported && now_ms.saturating_sub(pressed_at) >= LONG_HOLD_MS {
                self.hold_reported = true;
                return Some(ButtonGesture::LongHold);
            }
            return None;
        }

        let pressed_at = self.pressed_at_ms.take()?;
        let hold_reported = core::mem::take(&mut self.hold_reported);
        let held_ms = now_ms.saturating_sub(pressed_at);
        (!hold_reported && held_ms >= SHORT_PRESS_MIN_MS).then_some(ButtonGesture::ShortPress)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    fn press(gestures: &mut ButtonGestures, from_ms: u64, to_ms: u64) -> Vec<ButtonGesture> {
        let mut out = Vec::new();
        let mut now = from_ms;
        while now < to_ms {
            out.extend(gestures.sample(true, now));
            now += 20;
        }
        out.extend(gestures.sample(false, to_ms));
        out
    }

    #[test]
    fn short_press_is_reported_on_release() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(gestures.sample(true, 1_000), None);
        assert_eq!(gestures.sample(true, 1_150), None);
        assert_eq!(
            gestures.sample(false, 1_200),
            Some(ButtonGesture::ShortPress)
        );
        assert_eq!(gestures, ButtonGestures::new());
    }

    #[test]
    fn idle_and_bounce_samples_produce_nothing() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(gestures.sample(false, 0), None);
        assert_eq!(gestures.sample(true, 10), None);
        assert_eq!(gestures.sample(false, 10 + SHORT_PRESS_MIN_MS - 1), None);
        assert_eq!(gestures.sample(false, 500), None);
    }

    #[test]
    fn hold_is_reported_once_and_no_short_press_on_release() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(
            press(&mut gestures, 0, LONG_HOLD_MS + 2_000),
            [ButtonGesture::LongHold]
        );
        assert_eq!(gestures, ButtonGestures::new());
    }

    #[test]
    fn press_just_below_hold_is_still_a_short_press() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(gestures.sample(true, 0), None);
        assert_eq!(gestures.sample(true, LONG_HOLD_MS - 1), None);
        assert_eq!(
            gestures.sample(false, LONG_HOLD_MS - 1),
            Some(ButtonGesture::ShortPress)
        );
    }

    #[test]
    fn consecutive_presses_are_classified_independently() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(
            press(&mut gestures, 0, LONG_HOLD_MS + 20),
            [ButtonGesture::LongHold]
        );
        assert_eq!(
            press(&mut gestures, 10_000, 10_200),
            [ButtonGesture::ShortPress]
        );
        assert_eq!(
            press(&mut gestures, 20_000, 20_200),
            [ButtonGesture::ShortPress]
        );
    }
}
