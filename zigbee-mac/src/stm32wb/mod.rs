//! STM32WB coprocessor 802.15.4 MAC backend.
//!
//! On STM32WB the radio is owned by CPU2 (Cortex-M0+), which runs ST's
//! full-function IEEE 802.15.4-2011 MAC (`stm32wb5x_Mac_802_15_4_fw.bin`,
//! validated with 1.24.0). CPU1 cannot drive the radio directly; it issues
//! MLME/MCPS primitives over IPCC. This backend maps those primitives onto
//! [`crate::MacDriver`] and the sealed [`crate::ParentMacDriver`] router
//! surface. It has no hardware dependency: [`WbMacTransport`] is implemented
//! by the chip HAL (`stm32wb-hal`, over `embassy-stm32-wpan`), and the
//! mapping is host-tested with a scripted transport.
//!
//! # Router capability mapping
//!
//! | Zigbee router need              | Coprocessor MAC facility                      |
//! |---------------------------------|-----------------------------------------------|
//! | router start (non-beacon)       | MLME-START, `PANCoordinator = FALSE`, BO=SO=15 |
//! | beacon on Beacon Request        | autonomous, content from `macBeaconPayload`   |
//! | association                     | MLME-ASSOCIATE.indication / .response         |
//! | association response delivery   | MLME-COMM-STATUS.indication                   |
//! | indirect data, Frame Pending    | MCPS-DATA.request `TxOptions = indirect`      |
//! | child poll (keepalive)          | MLME-POLL.indication (ST extension)           |
//! | orphan realignment              | MLME-ORPHAN.indication / .response            |
//! | address filtering               | `macShortAddress`, `macPANId`, EUI-64 PIB     |
//! | TX power                        | `phyTransmitPower` (−20..+6 dBm)              |
//!
//! # Deviations from a host-controlled MAC (documented gaps)
//!
//! - **Beacons are autonomous.** No Beacon Request indication reaches the
//!   host. The runtime's beacon decision is pushed into the coprocessor PIB
//!   by a synthetic [`MacCommandEvent::BeaconRequest`] every
//!   [`BEACON_REFRESH_US`]; only changes are programmed. A beacon
//!   may therefore lag a permit-join change by up to that period.
//! - **Indirect ownership moves to CPU2.** When the NWK layer marks a child
//!   pending, the backend emits a *virtual* Data Request so the runtime
//!   hands the frame over at once; CPU2 then holds it for
//!   `macTransactionPersistenceTime` and sets Frame Pending in the poll ACK.
//!   Consequently `mcps_indirect_data` completes on hand-off, not on
//!   over-the-air delivery: a Leave or Child-Update is considered delivered
//!   when queued, and the child keepalive is refreshed once per handed-off
//!   frame. Delivery outcomes are counted in [`Stm32wbMacStats`].
//!   At most [`MAX_INDIRECT_IN_FLIGHT`] frames are outstanding.
//! - **No per-request beacon pending-address lists** (Zigbee non-beacon
//!   networks do not use them) and no PAN-coordinator start.
//! - **No MAC-layer security** (Zigbee secures at NWK/APS).
//! - **Orphan scan** is not offered (as in the soft MAC).
//! - LQI is not reported for MAC commands (association, poll); `0` is used.
//!
//! Router start with `PANCoordinator = FALSE`, data and ACK exchange,
//! retries, relaying, Link Status and RREP have been observed on air with
//! MAC firmware 1.24.0. The following have not been observed on air yet,
//! so this backend is not yet child-parent proven:
//! - beacon response to a third-party Beacon Request;
//! - child association;
//! - indirect delivery.

mod driver;
pub mod sap;

pub use driver::{
    BEACON_REFRESH_US, COMMAND_QUEUE_DEPTH, MAX_INDIRECT_IN_FLIGHT, MAX_PENDING_ASSOCIATIONS,
    MAX_PENDING_CHILDREN, RX_QUEUE_DEPTH, Stm32wbMac, Stm32wbMacStats, map_status,
};
pub use sap::{WbMacTransport, status};

#[cfg(test)]
mod tests;
