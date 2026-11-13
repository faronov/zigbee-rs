//! Conversion of the DK's sensor readings into the values the product
//! reports. Platform independent and host tested; the composition root
//! reads the parts and feeds the results to the sensor profile.

/// Lowest total acceleration accepted as a static gravity vector.
pub const MIN_GRAVITY_MG: i32 = 700;
/// Highest total acceleration accepted as a static gravity vector.
pub const MAX_GRAVITY_MG: i32 = 1_300;

/// Inclination of the module's Z axis from the upward vertical, in
/// centi-degrees: 0 lying face up, 9000 standing on an edge, 18000 face
/// down. `None` while the total acceleration is not close to 1 g (the
/// board is being moved or is falling), when no tilt can be derived.
pub fn inclination_centi_deg(x_mg: i32, y_mg: i32, z_mg: i32) -> Option<u16> {
    let (x, y, z) = (x_mg as f32, y_mg as f32, z_mg as f32);
    let horizontal = libm::sqrtf(x * x + y * y);
    let total = libm::sqrtf(horizontal * horizontal + z * z);
    if total < MIN_GRAVITY_MG as f32 || total > MAX_GRAVITY_MG as f32 {
        return None;
    }
    let degrees = libm::atan2f(horizontal, z).to_degrees();
    Some(libm::roundf(degrees * 100.0).clamp(0.0, 18_000.0) as u16)
}

/// Change of acceleration between two samples, on any axis, that counts
/// as motion. The ISM330DHCX noise at the default rate is a few mg.
pub const MOTION_THRESHOLD_MG: i32 = 50;
/// How long the product stays "occupied" after the last motion.
pub const OCCUPANCY_HOLD_MS: u64 = 60_000;

/// Motion from successive accelerometer samples: occupied while the board
/// was moved within the last [`OCCUPANCY_HOLD_MS`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct MotionDetector {
    last: Option<[i32; 3]>,
    last_motion_ms: Option<u64>,
}

impl MotionDetector {
    pub const fn new() -> Self {
        Self {
            last: None,
            last_motion_ms: None,
        }
    }

    /// Feed one sample (mg) taken at `now_ms`; returns whether the board
    /// counts as occupied.
    pub fn update(&mut self, accel_mg: [i32; 3], now_ms: u64) -> bool {
        if let Some(last) = self.last {
            let moved = last
                .iter()
                .zip(accel_mg.iter())
                .any(|(a, b)| a.abs_diff(*b) > MOTION_THRESHOLD_MG as u32);
            if moved {
                self.last_motion_ms = Some(now_ms);
            }
        }
        self.last = Some(accel_mg);
        self.occupied(now_ms)
    }

    /// Whether motion was seen within the hold time before `now_ms`.
    pub fn occupied(&self, now_ms: u64) -> bool {
        self.last_motion_ms
            .is_some_and(|at| now_ms.saturating_sub(at) < OCCUPANCY_HOLD_MS)
    }
}

/// Longest distance the product reports as a valid ToF range. The VL53L0X
/// default (non long-range) mode is specified to about 1.2 m indoors and
/// returns up to ~2 m on bright targets.
pub const MAX_DISTANCE_MM: u16 = 2_000;

/// Distance to report from one VL53L0X result: `Some(mm)` for a valid
/// range (driver range status 0) up to [`MAX_DISTANCE_MM`], else `None`
/// (no target, signal or sigma failure, or out of range).
pub const fn distance_mm(range_valid: bool, distance_mm: u16) -> Option<u16> {
    if range_valid && distance_mm <= MAX_DISTANCE_MM {
        Some(distance_mm)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inclination_follows_gravity() {
        assert_eq!(inclination_centi_deg(0, 0, 1_000), Some(0));
        assert_eq!(inclination_centi_deg(0, 0, -1_000), Some(18_000));
        assert_eq!(inclination_centi_deg(0, -1_000, 0), Some(9_000));
        assert_eq!(inclination_centi_deg(707, 0, 707), Some(4_500));
        // The DK on its stand, as measured: x=-34 y=-1018 z=19 mg.
        let standing = inclination_centi_deg(-34, -1_018, 19).unwrap();
        assert!((8_880..=8_910).contains(&standing), "{standing}");
    }

    #[test]
    fn inclination_needs_about_one_g() {
        assert_eq!(inclination_centi_deg(0, 0, 0), None);
        assert_eq!(inclination_centi_deg(0, 0, 500), None);
        assert_eq!(inclination_centi_deg(1_500, 0, 1_000), None);
        assert!(inclination_centi_deg(0, 0, MIN_GRAVITY_MG).is_some());
        assert!(inclination_centi_deg(0, 0, MAX_GRAVITY_MG).is_some());
    }

    #[test]
    fn motion_holds_occupancy_then_clears() {
        let mut motion = MotionDetector::new();
        let rest = [-34, -1_018, 19];
        assert!(!motion.update(rest, 0), "first sample has no reference");
        assert!(
            !motion.update([-30, -1_015, 22], 200),
            "noise is not motion"
        );
        assert!(motion.update([-34, -900, 19], 400), "a nudge is motion");
        assert!(motion.update(rest, 600), "settling back is motion too");
        assert!(motion.update(rest, 600 + OCCUPANCY_HOLD_MS - 1));
        assert!(!motion.update(rest, 600 + OCCUPANCY_HOLD_MS));
    }

    #[test]
    fn threshold_is_exclusive() {
        let mut motion = MotionDetector::new();
        motion.update([0, 0, 1_000], 0);
        assert!(!motion.update([MOTION_THRESHOLD_MG, 0, 1_000], 1));
        assert!(!motion.update([0, 0, 1_000], 2));
        assert!(motion.update([MOTION_THRESHOLD_MG + 1, 0, 1_000], 3));
    }

    #[test]
    fn distance_accepts_only_valid_in_range_results() {
        assert_eq!(distance_mm(true, 123), Some(123));
        assert_eq!(distance_mm(true, MAX_DISTANCE_MM), Some(MAX_DISTANCE_MM));
        assert_eq!(distance_mm(true, MAX_DISTANCE_MM + 1), None);
        assert_eq!(distance_mm(false, 123), None);
        // No target: the driver reports 8191 mm with a non-zero status.
        assert_eq!(distance_mm(false, 8_191), None);
    }
}
