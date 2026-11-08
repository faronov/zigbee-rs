//! Status LEDs while the loader runs.

use crate::BootEvent;

/// Status output (typically LEDs) while the loader runs.
pub trait Indicator {
    /// An upgrade is being installed; called repeatedly with alternating
    /// `on` while copying.
    fn installing(&mut self, on: bool);
    /// Nothing can be booted.
    fn fault(&mut self, on: bool);
    /// Everything off before jumping to the application.
    fn off(&mut self);
}

/// Copy progress granularity of the installing indication.
pub const BLINK_EVERY: u32 = 8 * 1024;

/// Map boot progress onto the indicator: the installing output toggles
/// every [`BLINK_EVERY`] bytes copied.
pub fn indicate(indicator: &mut impl Indicator, event: BootEvent) {
    if let BootEvent::Copying { done, .. } = event {
        indicator.installing((done / BLINK_EVERY).is_multiple_of(2));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Recorder {
        installing: Vec<bool>,
    }

    impl Indicator for Recorder {
        fn installing(&mut self, on: bool) {
            self.installing.push(on);
        }
        fn fault(&mut self, _on: bool) {}
        fn off(&mut self) {}
    }

    #[test]
    fn copy_progress_blinks_per_8k() {
        let mut recorder = Recorder::default();
        for done in (0..32 * 1024).step_by(4096) {
            indicate(
                &mut recorder,
                BootEvent::Copying {
                    done,
                    total: 32 * 1024,
                },
            );
        }
        indicate(&mut recorder, BootEvent::Validating);
        assert_eq!(
            recorder.installing,
            [true, true, false, false, true, true, false, false]
        );
    }
}
