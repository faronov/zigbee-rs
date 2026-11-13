//! Zigbee Green Power (GP) building blocks for zigbee-rs.
//!
//! This crate is `no_std`, allocation-free and sans-IO. It contains:
//!
//! * [`types`] – GP identifiers, enumerations and alias derivation;
//! * [`frame`] – GPDF (NWK protocol version 3) parsing and building;
//! * [`security`] – GP CCM* nonce/header construction, level 2/3
//!   protection and TC-LK key wrapping;
//! * [`cluster`] – Green Power cluster (0x0021) command codecs used between
//!   proxies and sinks over the Zigbee network (GP endpoint 242, profile
//!   0xA1E0);
//! * [`commissioning`] – GPD commissioning-related command payloads
//!   (Commissioning, Commissioning Reply, Channel Request/Configuration);
//! * [`dup`] – the GP duplicate filter;
//! * [`proxy`] (feature `proxy`) – the Proxy Basic state machine and proxy
//!   table;
//! * [`gpd`] (feature `gpd`) – Green Power Device-side transmission helpers.
//!
//! The crate never touches a radio, a clock or flash directly. Callers feed
//! received frames and the current monotonic time in milliseconds, and drain
//! the resulting transmissions and persistence requests.
//!
//! Wire formats follow the Green Power specification (docs-14-0563) and are
//! cross-checked against the zigpy sink implementation used by Home Assistant
//! ZHA; see the golden-vector tests in `tests/`.

#![no_std]
#![deny(unsafe_code)]

mod codec;

pub mod cluster;
pub mod commissioning;
pub mod dup;
pub mod frame;
pub mod security;
pub mod types;

#[cfg(feature = "gpd")]
pub mod gpd;
#[cfg(feature = "proxy")]
pub mod proxy;

/// The Green Power endpoint (GPEP).
pub const GP_ENDPOINT: u8 = 242;
/// The Green Power application profile identifier.
pub const GP_PROFILE_ID: u16 = 0xA1E0;
/// The Green Power cluster identifier.
pub const GP_CLUSTER_ID: u16 = 0x0021;

/// Errors produced by the codecs in this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The input ended before a mandatory field.
    Truncated,
    /// The input had bytes after the last field (strict parsers only).
    TrailingBytes,
    /// A field carried a value that is reserved or not supported.
    Unsupported,
    /// The frame is not a GPDF (e.g. NWK protocol version is not 3).
    NotGpdf,
    /// An output buffer was too small.
    BufferFull,
    /// Security processing failed (bad MIC or invalid key material).
    AuthFailed,
    /// The AES backend reported a failure.
    CipherFailure,
}

/// Result type used throughout the crate.
pub type Result<T> = core::result::Result<T, Error>;
