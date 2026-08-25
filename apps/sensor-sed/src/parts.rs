//! Explicit resource bundle supplied by the composition root.

use zigbee_mac::MacDriver;

use crate::capabilities::WakeController;

/// Concrete capabilities owned by one environmental sleepy-sensor instance.
///
/// Grouping the values shortens the public [`crate::SensorApp`] type without
/// hiding ownership or constructing any peripheral inside reusable code.
pub struct SensorSedParts<W, St, E, B, O, A, Sv, D> {
    pub wake: W,
    pub status: St,
    pub environment: E,
    pub battery: B,
    pub ota: O,
    pub actions: A,
    pub supervisor: Sv,
    pub diagnostics: D,
}

mod sealed {
    pub trait Sealed {}

    impl<W, St, E, B, O, A, Sv, D> Sealed for super::SensorSedParts<W, St, E, B, O, A, Sv, D> {}
}

/// Internal descriptor used only to name timing storage in [`crate::SensorApp`].
///
/// This is intentionally not a platform provider: it has no constructors,
/// lookup methods, MAC, profile, store, or peripheral acquisition.
#[doc(hidden)]
pub trait SensorSedResources<M: MacDriver>: sealed::Sealed {
    type Mark: Copy;
}

impl<M, W, St, E, B, O, A, Sv, D> SensorSedResources<M> for SensorSedParts<W, St, E, B, O, A, Sv, D>
where
    M: MacDriver,
    W: WakeController<M>,
{
    type Mark = W::Mark;
}
