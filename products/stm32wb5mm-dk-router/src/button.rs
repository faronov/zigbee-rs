//! Product-owned button B1 gestures for the router.
//!
//! The composition root samples the active-low DK button B1 (PC12) between bounded
//! application steps and feeds the level and a monotonic millisecond clock
//! into [`ButtonGestures`]. The product decides what each gesture means:
//!
//! - **short press** (released after at least [`SHORT_PRESS_MIN_MS`] and
//!   before [`FACTORY_RESET_HOLD_MS`]): request commissioning. After a Leave
//!   without rejoin the shared router app stays factory-new and performs no
//!   automatic network search until this request; it also skips a running
//!   join backoff. The app ignores the request while joined or while a
//!   factory reset is pending, so a short press never disturbs a joined node.
//! - **hold** for [`FACTORY_RESET_HOLD_MS`]: journal-aware factory reset,
//!   reported once while the button is still held. Releasing the button after
//!   a reset hold never also produces a short press.

/// Hold duration that commits a factory reset.
pub const FACTORY_RESET_HOLD_MS: u64 = 3_000;

/// Shortest press accepted as a commissioning request. Contact bounce and a
/// single glitching sample between receive slices are rejected.
pub const SHORT_PRESS_MIN_MS: u64 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButtonGesture {
    /// Debounced short press, reported on release.
    CommissioningRequest,
    /// Button held for [`FACTORY_RESET_HOLD_MS`], reported once per press.
    FactoryReset,
}

/// Bounded, allocation-free button gesture classifier.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ButtonGestures {
    pressed_at_ms: Option<u64>,
    reset_reported: bool,
}

impl ButtonGestures {
    pub const fn new() -> Self {
        Self {
            pressed_at_ms: None,
            reset_reported: false,
        }
    }

    /// Feed one button sample. `pressed` is the logical (polarity-corrected)
    /// level and `now_ms` a monotonic millisecond timestamp.
    pub fn sample(&mut self, pressed: bool, now_ms: u64) -> Option<ButtonGesture> {
        if pressed {
            let pressed_at = *self.pressed_at_ms.get_or_insert(now_ms);
            if !self.reset_reported && now_ms.saturating_sub(pressed_at) >= FACTORY_RESET_HOLD_MS {
                self.reset_reported = true;
                return Some(ButtonGesture::FactoryReset);
            }
            return None;
        }

        let pressed_at = self.pressed_at_ms.take()?;
        let reset_reported = core::mem::take(&mut self.reset_reported);
        let held_ms = now_ms.saturating_sub(pressed_at);
        (!reset_reported && held_ms >= SHORT_PRESS_MIN_MS)
            .then_some(ButtonGesture::CommissioningRequest)
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
    fn short_press_requests_commissioning_on_release() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(gestures.sample(true, 1_000), None);
        assert_eq!(gestures.sample(true, 1_150), None);
        assert_eq!(
            gestures.sample(false, 1_200),
            Some(ButtonGesture::CommissioningRequest)
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
    fn hold_reports_factory_reset_once_and_no_short_press_on_release() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(
            press(&mut gestures, 0, FACTORY_RESET_HOLD_MS + 2_000),
            [ButtonGesture::FactoryReset]
        );
        assert_eq!(gestures, ButtonGestures::new());
    }

    #[test]
    fn press_just_below_hold_is_still_a_short_press() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(gestures.sample(true, 0), None);
        assert_eq!(gestures.sample(true, FACTORY_RESET_HOLD_MS - 1), None);
        assert_eq!(
            gestures.sample(false, FACTORY_RESET_HOLD_MS - 1),
            Some(ButtonGesture::CommissioningRequest)
        );
    }

    #[test]
    fn consecutive_presses_are_classified_independently() {
        let mut gestures = ButtonGestures::new();
        assert_eq!(
            press(&mut gestures, 0, FACTORY_RESET_HOLD_MS + 20),
            [ButtonGesture::FactoryReset]
        );
        assert_eq!(
            press(&mut gestures, 10_000, 10_200),
            [ButtonGesture::CommissioningRequest]
        );
        assert_eq!(
            press(&mut gestures, 20_000, 20_200),
            [ButtonGesture::CommissioningRequest]
        );
    }
}
