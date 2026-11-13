//! State shared between the router loop (`main.rs`) and the peripheral
//! tasks. Each value is a small `Copy` snapshot behind a critical section:
//! the router loop publishes network state, the indicator and display
//! tasks read it; the sensor tasks publish readings that the router loop
//! applies to the ZCL profile. No task ever holds the router.

#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
use core::cell::Cell;

#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use router_app::RouterStatus;
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
use stm32wb5mm_dk_router_product::screen::NetworkSnapshot;
#[cfg(any(feature = "lcd", feature = "sensors", feature = "mic"))]
use stm32wb5mm_dk_router_product::screen::SensorSnapshot;

#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
pub struct Shared<T: Copy>(Mutex<CriticalSectionRawMutex, Cell<T>>);

#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
impl<T: Copy> Shared<T> {
    pub const fn new(value: T) -> Self {
        Self(Mutex::new(Cell::new(value)))
    }

    pub fn get(&self) -> T {
        self.0.lock(Cell::get)
    }

    #[cfg_attr(
        not(any(feature = "rgb-led", feature = "lcd", feature = "sensors")),
        allow(dead_code)
    )]
    pub fn set(&self, value: T) {
        self.0.lock(|cell| cell.set(value))
    }

    #[cfg_attr(not(any(feature = "sensors", feature = "mic")), allow(dead_code))]
    pub fn update(&self, f: impl FnOnce(&mut T)) {
        self.0.lock(|cell| {
            let mut value = cell.get();
            f(&mut value);
            cell.set(value);
        })
    }
}

/// Latest router lifecycle status.
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
pub static STATUS: Shared<Option<RouterStatus>> = Shared::new(None);

/// Network state, refreshed by the router loop.
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
pub static NETWORK: Shared<NetworkSnapshot> = Shared::new(NetworkSnapshot::EMPTY);

/// Latest sensor readings, refreshed by the sensor tasks.
#[cfg(any(feature = "lcd", feature = "sensors", feature = "mic"))]
pub static SENSORS: Shared<SensorSnapshot> = Shared::new(SensorSnapshot::EMPTY);

/// Occupancy Sensing motion hold, seconds: written by the router loop
/// from the ZCL attribute, read by the sensing task.
#[cfg(feature = "sensors")]
pub static MOTION_HOLD_SECS: Shared<u16> =
    Shared::new(stm32wb5mm_dk_router_product::motion::DEFAULT_HOLD_SECS);

/// Record a lifecycle change for the indicator and display tasks.
pub fn publish_status(status: RouterStatus) {
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    STATUS.set(Some(status));
    let _ = status;
}
