//! ZRHP v1 network co-processor application (spec
//! `docs/book/src/advanced/host-protocol.md`).
//!
//! * [`ncp::Ncp`] — link pump (zigbee-zrhp `Link<16, 16>`) and command
//!   dispatch; never blocks, queues stack work as [`port::Op`]s.
//! * [`port::CoordinatorPort`] — what the dispatcher needs from the stack;
//!   [`tc::TcPort`] implements it over `router_app::TrustCenterCoordinatorApp`
//!   with the global APS counter and the host-passthrough indication queue.
//! * [`transport::HostTransport`] — the non-blocking byte stream the
//!   product provides (ESP32-H2: USB-Serial-JTAG or UART with DMA receive).
//! * [`log_ring::LogRing`] — UTF-8 records for the LOG endpoint.
//! * [`gp`] — the GREEN POWER group over the runtime's GP combo (sink mode
//!   switch, Sink/Proxy Table access, `EVT_GP_*`); [`gp_remote`] — remote
//!   GP provisioning of other sinks and proxies (`EVT_GP_REMOTE_*`).
//!
//! Validation: host-tested against the zigbee-zrhp host link over an
//! in-memory pipe with a mock MAC (`tests/`). Nothing here has run on
//! hardware.

#![cfg_attr(not(test), no_std)]

/// Await `future` through a `dyn Future`, so that its poll runs in its own
/// frame instead of being merged into the caller's (the root future's poll
/// frame is on every stack chain of the NCP loop).
macro_rules! await_out_of_line {
    ($future:expr) => {{
        let future = core::pin::pin!($future);
        let future: core::pin::Pin<&mut dyn core::future::Future<Output = _>> = future;
        future.await
    }};
}

pub mod backup;
pub mod diag;
pub mod firmware;
pub mod gp;
pub mod gp_remote;
pub mod log_ring;
pub mod ncp;
pub mod platform;
pub mod port;
pub mod radio;
pub mod retry;
pub mod tables;
pub mod tc;
pub mod transport;

pub use log_ring::LogRing;
pub use ncp::{Ncp, NcpIdentity, PlatformRequest};
pub use platform::{NcpPlatform, NoPlatform};
pub use port::CoordinatorPort;
pub use transport::HostTransport;

/// Capacity per NCP tier: `full` (`ncp-capacity`), `compact` (feature
/// `compact`, CC2652P7) and `tiny` (feature `tiny`, 64 KiB parts). Only
/// evaluated in constant contexts (declared last: keeps the line numbers
/// above, which panic locations embed, unchanged).
pub(crate) const fn capacity_tier(full: usize, compact: usize, tiny: usize) -> usize {
    if cfg!(feature = "tiny") {
        tiny
    } else if cfg!(feature = "compact") {
        compact
    } else {
        full
    }
}

#[cfg(all(feature = "compact", feature = "tiny"))]
compile_error!("ncp-coordinator-app: `compact` and `tiny` are exclusive capacity tiers");

/// `cell.fetch_add(1) + 1` (wrapping). Targets without atomic
/// read-modify-write (TC32, Cortex-M0+) do the same under a critical
/// section (declared last, like the function above).
#[cfg(target_has_atomic = "8")]
#[inline(always)]
pub(crate) fn increment_u8(cell: &core::sync::atomic::AtomicU8) -> u8 {
    cell.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
        .wrapping_add(1)
}

#[cfg(not(target_has_atomic = "8"))]
pub(crate) fn increment_u8(cell: &core::sync::atomic::AtomicU8) -> u8 {
    use core::sync::atomic::Ordering;
    critical_section::with(|_| {
        let next = cell.load(Ordering::Relaxed).wrapping_add(1);
        cell.store(next, Ordering::Relaxed);
        next
    })
}
