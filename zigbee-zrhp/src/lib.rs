//! # zigbee-zrhp — ZRHP v1, the zigbee-rs host ↔ NCP protocol
//!
//! Shared reference implementation of the protocol specified in
//! `docs/book/src/advanced/host-protocol.md` (ZRHP v1). The same code runs on
//! the ESP32-H2 NCP, on an embedded host (ESP32-S3 hub) and on desktop
//! hosts.
//!
//! * `#![no_std]`, no allocation, no dependencies, no `unsafe`.
//! * Sans-IO and poll-style: no clock, executor or transport is assumed; the
//!   caller passes monotonic microseconds and moves bytes.
//! * Fixed memory chosen by const generics ([`link::Link`]`<TXW, RXW>`).
//!
//! ## Layers
//!
//! | Module | Spec | Contents |
//! |---|---|---|
//! | [`crc`] | 5.2 | CRC-32C (Castagnoli) |
//! | [`cobs`] | 5.1 | COBS encode/decode, bounded streaming [`cobs::Accumulator`] |
//! | [`frame`] | 5.2–5.5 | 8-byte header, frame types, `ACK`/`NAK`/`RESET`/`REJECT` bodies, epoch |
//! | [`link`] | 5.5–5.13 | session handshake, SR-ARQ with SACK, RTO, NAK/`nbad`, credits, scheduler, keepalive |
//! | [`queue`] | 5.10 | [`queue::TxSource`] and fixed per-endpoint message queues |
//! | [`msg`] | 6–9 | message header, batches, IDs, typed codecs for every v1 command and event |
//! | [`records`] | 9.3–9.4 | backup/table TLV records, `END` CRC, restore counter rule |
//! | [`codec`] | — | little-endian [`codec::Wire`] primitives used by all codecs |
//!
//! ## Validation level
//!
//! Host-tested only: byte-exact against every JSON vector in
//! `docs/book/src/advanced/host-protocol-vectors/`, the normative ARQ traces,
//! and link property tests (loss, reordering, duplication, corruption,
//! resets). No hardware gate has been run.

#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "std")]
extern crate std;

pub mod cobs;
pub mod codec;
pub mod crc;
pub mod frame;
pub mod link;
pub mod msg;
pub mod queue;
pub mod records;
pub mod types;

pub use types::{Endpoint, Ieee, Key, Status};

/// Protocol major version implemented by this crate.
pub const PROTOCOL_MAJOR: u8 = 1;
/// Protocol minor version implemented by this crate.
pub const PROTOCOL_MINOR: u8 = 0;
