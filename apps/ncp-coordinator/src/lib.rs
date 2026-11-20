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
pub mod retry;
pub mod tables;
pub mod tc;
pub mod transport;

pub use log_ring::LogRing;
pub use ncp::{Ncp, NcpIdentity, PlatformRequest};
pub use platform::{NcpPlatform, NoPlatform};
pub use port::CoordinatorPort;
pub use transport::HostTransport;
