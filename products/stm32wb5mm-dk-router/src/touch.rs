//! Touch-key decision for the DK's TSC key.
//!
//! The board acquires raw charge-transfer counts; a finger lowers the
//! count. [`TouchDetector`] learns the untouched baseline, applies the ST
//! BSP's hysteresis (enter [`ENTER_DELTA`] / exit [`EXIT_DELTA`] counts
//! below the baseline), tracks slow drift only while released and close to
//! the baseline (an approaching finger must not drag it down), and
//! recalibrates if a "touch" lasts implausibly long (a wet or covered key)
//! or the count rises [`ENTER_DELTA`] above the baseline (it was learned
//! while touched).
//! The resulting level feeds a [`crate::button::ButtonGestures`] like a
//! button.

/// Counts below the baseline that start a touch (ST BSP TSL threshold).
pub const ENTER_DELTA: u16 = 120;
/// Counts below the baseline that end a touch.
pub const EXIT_DELTA: u16 = 110;
/// Samples averaged into the initial baseline.
pub const CALIBRATION_SAMPLES: u8 = 8;
/// A touch longer than this recalibrates the baseline.
pub const MAX_TOUCH_MS: u64 = 20_000;
/// Baseline drift tracking: the baseline moves by 1/2^shift of the
/// difference per released sample...
pub const DRIFT_SHIFT: u32 = 8;
/// ...that lies within this many counts of it.
pub const DRIFT_WINDOW: u16 = 20;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TouchDetector {
    /// Baseline in 1/16 counts; `None` while calibrating.
    baseline_q4: Option<u32>,
    calibration_sum: u32,
    calibration_count: u8,
    touched_since_ms: Option<u64>,
}

impl Default for TouchDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl TouchDetector {
    pub const fn new() -> Self {
        Self {
            baseline_q4: None,
            calibration_sum: 0,
            calibration_count: 0,
            touched_since_ms: None,
        }
    }

    /// Untouched reference count, once calibrated.
    pub fn baseline(&self) -> Option<u16> {
        self.baseline_q4.map(round_q4)
    }

    pub fn touched(&self) -> bool {
        self.touched_since_ms.is_some()
    }

    /// Feed one acquisition taken at `now_ms`; returns whether the key is
    /// touched. A failed acquisition (no count) should simply not be fed.
    pub fn update(&mut self, count: u16, now_ms: u64) -> bool {
        let Some(baseline_q4) = self.baseline_q4 else {
            self.calibration_sum += u32::from(count);
            self.calibration_count += 1;
            if self.calibration_count >= CALIBRATION_SAMPLES {
                self.baseline_q4 =
                    Some((self.calibration_sum << 4) / u32::from(self.calibration_count));
            }
            return false;
        };
        let baseline = round_q4(baseline_q4);
        let depth = baseline.saturating_sub(count);
        let rise = count.saturating_sub(baseline);
        match self.touched_since_ms {
            _ if rise >= ENTER_DELTA => self.recalibrate(count),
            Some(since) if now_ms.saturating_sub(since) >= MAX_TOUCH_MS => {
                self.recalibrate(count);
            }
            Some(_) if depth < EXIT_DELTA => self.touched_since_ms = None,
            Some(_) => {}
            None if depth >= ENTER_DELTA => self.touched_since_ms = Some(now_ms),
            None if depth > DRIFT_WINDOW || rise > DRIFT_WINDOW => {}
            None => {
                let target = u32::from(count) << 4;
                let next = if target >= baseline_q4 {
                    baseline_q4 + ((target - baseline_q4) >> DRIFT_SHIFT)
                } else {
                    baseline_q4 - ((baseline_q4 - target) >> DRIFT_SHIFT)
                };
                self.baseline_q4 = Some(next);
            }
        }
        self.touched()
    }

    fn recalibrate(&mut self, count: u16) {
        self.baseline_q4 = Some(u32::from(count) << 4);
        self.touched_since_ms = None;
    }
}

fn round_q4(q4: u32) -> u16 {
    ((q4 + 8) >> 4).min(u32::from(u16::MAX)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibrated(level: u16) -> TouchDetector {
        let mut touch = TouchDetector::new();
        for n in 0..CALIBRATION_SAMPLES {
            assert!(!touch.update(level, u64::from(n) * 50));
        }
        assert_eq!(touch.baseline(), Some(level));
        touch
    }

    #[test]
    fn calibrates_from_the_first_samples() {
        let mut touch = TouchDetector::new();
        assert_eq!(touch.baseline(), None);
        for (n, count) in [2448u16, 2452, 2450, 2449, 2451, 2450, 2450, 2450]
            .into_iter()
            .enumerate()
        {
            touch.update(count, n as u64 * 50);
        }
        assert_eq!(touch.baseline(), Some(2450));
    }

    #[test]
    fn hysteresis_enters_at_120_and_exits_at_110() {
        let mut touch = calibrated(2450);
        assert!(!touch.update(2450 - ENTER_DELTA + 1, 1_000));
        assert!(touch.update(2450 - ENTER_DELTA, 1_050));
        assert!(touch.update(2450 - EXIT_DELTA, 1_100), "still inside");
        assert!(!touch.update(2450 - EXIT_DELTA + 1, 1_150));
    }

    #[test]
    fn noise_never_touches_and_slow_drift_is_tracked() {
        let mut touch = calibrated(2450);
        let mut now = 1_000;
        for step in 0..2_000u32 {
            let drift = (step / 20) as u16; // +100 counts over the run
            let noise = [0u16, 2, 4, 2][step as usize % 4];
            assert!(!touch.update(2450 + drift + noise - 2, now));
            now += 50;
        }
        let baseline = touch.baseline().unwrap();
        assert!((2_530..=2_552).contains(&baseline), "{baseline}");
    }

    #[test]
    fn a_slow_approach_does_not_drag_the_baseline() {
        let mut touch = calibrated(2450);
        let mut now = 1_000;
        // A finger closing in over 1.5 s (30 samples).
        for count in (2_330..=2_450).rev().step_by(4) {
            touch.update(count, now);
            now += 50;
        }
        assert!(touch.touched());
        assert!(touch.baseline().unwrap() >= 2_449);
    }

    #[test]
    fn a_baseline_learned_while_touched_recovers() {
        let mut touch = calibrated(2_200);
        assert!(!touch.update(2_450, 1_000));
        assert_eq!(touch.baseline(), Some(2_450));
        assert!(touch.update(2_300, 1_050));
    }

    #[test]
    fn a_stuck_touch_recalibrates() {
        let mut touch = calibrated(2450);
        assert!(touch.update(2200, 1_000));
        assert!(touch.update(2200, 1_000 + MAX_TOUCH_MS - 1));
        assert!(!touch.update(2200, 1_000 + MAX_TOUCH_MS));
        assert_eq!(touch.baseline(), Some(2200));
        assert!(!touch.update(2205, 1_000 + MAX_TOUCH_MS + 50));
    }
}
