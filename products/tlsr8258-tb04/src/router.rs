//! TB-04 composition adapters for the shared parent-router application.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

pub const ENDPOINT: u8 = 1;

/// Preserve the production router's 20 ms receive bound and 5..60 second
/// exponential retry. One failed secured rejoin immediately returns to fresh
/// commissioning, matching the pre-app-model firmware.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 1,
};

pub type RouterProfile = DeviceProfile<RangeExtender>;

pub const fn range_extender_profile() -> RouterProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

pub use crate::status_leds::{RouterLedStatus, RouterLedSupervisor, led_adapters};
