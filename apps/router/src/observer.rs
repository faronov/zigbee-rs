//! Zero-storage static observer hooks for platform-specific metrics.

use zigbee_mac::MacDriver;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::role::DeviceRole;

use crate::RouterAppError;

/// Static metrics observer.
///
/// Implementations normally forward these associated functions to a
/// fixed-address or `static` metrics block. No observer value is stored in the
/// application, so selecting [`NoObserver`] costs no RAM and a Telink product
/// can retain debugger-stable global metrics.
///
/// Hooks run in the application future and must remain bounded and
/// nonblocking. They receive only immutable stack access and must not perform
/// protocol traffic or per-frame logging that perturbs MAC timing.
pub trait RouterObserver<M, R>
where
    M: MacDriver,
    R: DeviceRole,
{
    fn on_commissioning_attempt(_device: &ZigbeeDevice<M, R>, _attempt: u32, _started_us: u32) {}

    fn on_start_result(_device: &ZigbeeDevice<M, R>, _result: Result<u16, StartError>) {}

    fn on_secure_rejoin_attempt(_device: &ZigbeeDevice<M, R>, _started_us: u32) {}

    fn on_secure_rejoin_result(_device: &ZigbeeDevice<M, R>, _result: Result<u16, StartError>) {}

    /// Called after a committed factory reset has either completed its
    /// journal and child-table work or failed.
    fn on_urgent_factory_reset_result(
        _device: &ZigbeeDevice<M, R>,
        _result: Result<(), RouterAppError>,
    ) {
    }

    /// Called only after security is checkpointed and any child table is live.
    fn on_network_ready(_device: &ZigbeeDevice<M, R>) {}

    /// Called immediately before the bounded receive primitive.
    fn on_before_receive(_device: &ZigbeeDevice<M, R>, _timeout_us: u32) {}

    /// Called as soon as the MAC returns one normal data indication.
    fn on_frame_received(_device: &ZigbeeDevice<M, R>, _receive_elapsed_us: u32) {}

    /// Called after one incoming frame has been processed by the runtime.
    fn on_frame_processed(
        _device: &ZigbeeDevice<M, R>,
        _event: Option<&StackEvent>,
        _elapsed_us: u32,
    ) {
    }

    fn on_stack_event(_device: &ZigbeeDevice<M, R>, _event: &StackEvent) {}

    fn on_tick(_device: &ZigbeeDevice<M, R>, _elapsed_secs: u16, _result: &TickResult) {}

    fn on_fault(_device: &ZigbeeDevice<M, R>, _error: RouterAppError) {}
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoObserver;

impl<M, R> RouterObserver<M, R> for NoObserver
where
    M: MacDriver,
    R: DeviceRole,
{
}
