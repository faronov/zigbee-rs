//! Complete Zigbee PRO R22 Stack
//!
//! This is the top-level crate that re-exports all layers and provides
//! coordinator/router role implementations.
//!
//! # Crate Architecture
//! ```text
//!                    ┌─────────────────────┐
//!                    │   zigbee (this)      │
//!                    │  ├── coordinator     │
//!                    │  ├── router          │
//!                    │  └── re-exports      │
//!                    └────────┬────────────┘
//!                             │
//!                    ┌────────┴────────────┐
//!                    │   zigbee-runtime     │
//!                    │  builder, event loop │
//!                    │  NV storage, power   │
//!                    └────────┬────────────┘
//!                             │
//!            ┌────────────────┼────────────────┐
//!            │                │                 │
//!     ┌──────┴─────┐  ┌──────┴──────┐  ┌──────┴──────┐
//!     │ zigbee-bdb │  │ zigbee-zcl  │  │ zigbee-zdo  │
//!     │ commission │  │ clusters    │  │ discovery   │
//!     └──────┬─────┘  └─────────────┘  └──────┬──────┘
//!            │                                  │
//!            └────────────┬─────────────────────┘
//!                         │
//!                  ┌──────┴──────┐
//!                  │ zigbee-aps  │
//!                  │ binding/grp │
//!                  └──────┬──────┘
//!                         │
//!                  ┌──────┴──────┐
//!                  │ zigbee-nwk  │
//!                  │ join/route  │
//!                  └──────┬──────┘
//!                         │
//!                  ┌──────┴──────┐
//!                  │ zigbee-mac  │
//!                  │ MacDriver   │
//!                  └──────┬──────┘
//!                         │
//!              ┌──────────┼──────────┐
//!              │          │          │
//!           ESP32-C6  nRF52840    Mock
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

pub mod coordinator;
pub mod router;
pub mod trust_center;

// Re-export all sub-crates for convenience
pub use zigbee_aps as aps;
pub use zigbee_bdb as bdb;
pub use zigbee_mac as mac;
pub use zigbee_nwk as nwk;
pub use zigbee_runtime as runtime;
pub use zigbee_types as types;
pub use zigbee_zcl as zcl;
pub use zigbee_zdo as zdo;

// Re-export commonly used types at top level
pub use zigbee_nwk::routing::ConcentratorType;
pub use zigbee_runtime::ZigbeeDevice;
pub use zigbee_types::{Channel, ChannelMask, IeeeAddress, MacAddress, PanId, ShortAddress};
