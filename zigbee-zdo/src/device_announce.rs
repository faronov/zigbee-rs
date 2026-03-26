//! Device_annce (ZDP cluster 0x0013).
//!
//! After a device joins a network it broadcasts a `Device_annce` so that
//! other devices can update their address mappings.

use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, ShortAddress};

use crate::{DEVICE_ANNCE, ZdoError, ZdoLayer};

// ── Device_annce payload ────────────────────────────────────────

/// Device_annce frame (11 bytes after the ZDP TSN).
///
/// ```text
/// NWK_addr(2) | IEEE_addr(8) | capability(1)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceAnnounce {
    pub nwk_addr: ShortAddress,
    pub ieee_addr: IeeeAddress,
    pub capability: u8,
}

impl DeviceAnnounce {
    pub const WIRE_SIZE: usize = 11;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::WIRE_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.nwk_addr.0.to_le_bytes());
        buf[2..10].copy_from_slice(&self.ieee_addr);
        buf[10] = self.capability;
        Ok(Self::WIRE_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let nwk_addr = ShortAddress(u16::from_le_bytes([data[0], data[1]]));
        let mut ieee_addr = [0u8; 8];
        ieee_addr.copy_from_slice(&data[2..10]);
        Ok(Self {
            nwk_addr,
            ieee_addr,
            capability: data[10],
        })
    }
}

// ── ZdoLayer methods ────────────────────────────────────────────

impl<M: MacDriver> ZdoLayer<M> {
    /// Broadcast a `Device_annce` for this node (should be called after join).
    pub async fn send_device_annce(&mut self) -> Result<(), ZdoError> {
        let annce = DeviceAnnounce {
            nwk_addr: self.local_nwk_addr(),
            ieee_addr: self.local_ieee_addr(),
            capability: self.node_descriptor().mac_capabilities,
        };

        let mut buf = [0u8; 1 + DeviceAnnounce::WIRE_SIZE]; // TSN + payload
        buf[0] = self.next_seq();
        annce.serialize(&mut buf[1..])?;

        self.send_zdp_broadcast(DEVICE_ANNCE, &buf).await
    }

    /// Process an incoming `Device_annce` (called by the handler).
    ///
    /// Updates the NWK neighbor table with the announced address mapping.
    pub(crate) fn process_device_annce(
        &mut self,
        payload: &[u8],
    ) -> Result<DeviceAnnounce, ZdoError> {
        let annce = DeviceAnnounce::parse(payload)?;
        log::info!(
            "Device_annce: NWK=0x{:04X} IEEE={:02X?} cap=0x{:02X}",
            annce.nwk_addr.0,
            annce.ieee_addr,
            annce.capability,
        );
        // Update NWK neighbor table with the announced short/IEEE address mapping
        self.nwk_mut().update_neighbor_address(annce.nwk_addr, annce.ieee_addr);
        Ok(annce)
    }
}
