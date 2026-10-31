//! BL702 MAC backend.
//!
//! Experimental backend for the Bouffalo Lab BL702 RISC-V SoC.
//!
//! [`radio_phy::Bl702RadioPhy`] is a direct-register Rust implementation of
//! digital PHY reset, per-die RF calibration, CCA, TX, RX, and energy detect.
//! All MLME/MCPS behaviour comes from the shared
//! [`SoftMacCore`](crate::SoftMacCore) ([`radio_phy::Bl702SoftMac`]).
//!
//! # Architecture
//! ```text
//! MacDriver trait methods
//!        │
//!        ▼
//! SoftMacCore<Bl702RadioPhy>   scan, association, polling, CSMA-CA, ACK wait
//!        │
//!        ▼
//! Bl702RadioPhy (radio_phy.rs)
//!   ├── pure-Rust RF reset + calibration
//!   └── direct M154 polling TX/RX/CCA
//! ```
//!
//! The earlier `Bl702Mac` wrapper around Bouffalo's binary `liblmac154.a`
//! has been removed; no vendor radio archive is linked.
//!
//! # Dependencies
//! - the Rust radio path needs no vendor archive
//! - platform startup, clocks, and image packaging are supplied by the
//!   firmware crate
//! - Embassy async primitives (`embassy-sync`, `embassy-time`, `embassy-futures`)
//!
//! # Hardware
//! - BL702 module boards (XT-ZB1, DT-BL10, Pine64 Pinenut)
//! - BL706 IoT Development Board (Sipeed)
//!
//! The Rust path is hardware-tested on an XT-ZB1 using
//! `riscv32imc-unknown-none-elf`. Cold-start calibration, energy detect, RX,
//! and TX have been observed on channel 15.

pub mod radio_phy;
mod registers;
mod rf;
